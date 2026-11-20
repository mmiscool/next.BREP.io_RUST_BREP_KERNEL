//! What a flat-bottomed upright end mill removes from the level it is moved at: the plane region
//! its circle covers as its axis runs a program of level moves (straight lines and arcs about
//! upright axes), extruded from that level up through the workpiece's top.
//!
//! The region is the union of the discs of the cutter's radius about every point of the moves.
//! Its boundary lies on exact curves of three kinds: each straight move's two offset lines, each
//! arc move's two offset arcs (about the arc's own centre), and the circle about every move's
//! ends. Those carriers are made once each (coincident ones merged), cut where they cross or touch
//! one another (in closed form: lines and circles), and a piece is kept where the region lies on
//! exactly one side of it (two probes along it, read by the exact distance to the moves). The kept
//! pieces chain into loops: counter-clockwise outer loops and clockwise holes, each hole in the
//! smallest outer loop holding it. Each outer loop with its holes is one prism, its walls planes
//! and cylinders, its ends planes: no boolean and no fitted curve.
//!
//! One call makes every depth band's prism at once only when the caller asks for one band: the
//! moves given are at one level; a program at several depths is several calls (a deeper move
//! covers a shallower one over the same ground, so the caller passes the deepest).
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{make_arc, make_line, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};
use std::f64::consts::TAU;

/// A level move of the cutter's axis, in the plane of the level (x, y).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MillMove {
    /// Straight from `from` to `to` (the same point: the cutter standing, a plunge seen from above).
    Line { from: [f64; 2], to: [f64; 2] },
    /// About `centre` at `radius`, from the angle `start` (radians, from +x toward +y) through
    /// `sweep` (negative: clockwise).
    Arc { centre: [f64; 2], radius: f64, start: f64, sweep: f64 },
}

/// The prefix of every refusal of this module.
pub const MILL_REGION_REFUSAL: &str = "millRegion";

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{MILL_REGION_REFUSAL}: {message}"))
}

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{MILL_REGION_REFUSAL}: {message}"))
}

fn frag<T>(result: Result<T, String>, what: &'static str) -> Result<T, KernelRefusal> {
    result.or_refuse(KernelStage::Fragment, what)
}

type P = [f64; 2];

fn sub(a: P, b: P) -> P {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P, b: P) -> P {
    [a[0] + b[0], a[1] + b[1]]
}
fn scale(a: P, k: f64) -> P {
    [a[0] * k, a[1] * k]
}
fn dot(a: P, b: P) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P, b: P) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn norm(a: P) -> f64 {
    a[0].hypot(a[1])
}
fn dist(a: P, b: P) -> f64 {
    norm(sub(a, b))
}

/// A carrier of the region's boundary: a segment, or an arc counter-clockwise from `a0` through
/// `span` (a whole circle when `span` is TAU). Parameter t in [0, 1].
#[derive(Clone, Copy, Debug)]
enum Carrier {
    Seg { a: P, b: P },
    Arc { c: P, r: f64, a0: f64, span: f64 },
}

impl Carrier {
    fn at(&self, t: f64) -> P {
        match *self {
            Carrier::Seg { a, b } => add(a, scale(sub(b, a), t)),
            Carrier::Arc { c, r, a0, span } => {
                let angle = a0 + span * t;
                [c[0] + r * angle.cos(), c[1] + r * angle.sin()]
            }
        }
    }
    /// The direction of increasing t (not unit).
    fn tangent(&self, t: f64) -> P {
        match *self {
            Carrier::Seg { a, b } => sub(b, a),
            Carrier::Arc { r, a0, span, .. } => {
                let angle = a0 + span * t;
                [-r * span * angle.sin(), r * span * angle.cos()]
            }
        }
    }
    fn whole_circle(&self) -> bool {
        matches!(*self, Carrier::Arc { span, .. } if span >= TAU - 1e-12)
    }
    /// The parameter of `p`, where `p` lies on the carrier within `tol`.
    fn param_of(&self, p: P, tol: f64) -> Option<f64> {
        match *self {
            Carrier::Seg { a, b } => {
                let d = sub(b, a);
                let length = norm(d);
                let t = dot(sub(p, a), d) / (length * length);
                let foot = add(a, scale(d, t));
                let slack = tol / length;
                (dist(foot, p) <= tol && t >= -slack && t <= 1.0 + slack).then(|| t.clamp(0.0, 1.0))
            }
            Carrier::Arc { c, r, a0, span } => {
                if (dist(p, c) - r).abs() > tol {
                    return None;
                }
                let angle = (p[1] - c[1]).atan2(p[0] - c[0]);
                let mut t = (angle - a0).rem_euclid(TAU) / span;
                if self.whole_circle() {
                    return Some(t.rem_euclid(1.0));
                }
                let slack = tol / (r * span);
                if t > 1.0 + slack {
                    // past the end: near the start from below?
                    let before = t - TAU / span;
                    if before >= -slack {
                        t = 0.0;
                    } else {
                        return None;
                    }
                }
                Some(t.clamp(0.0, 1.0))
            }
        }
    }
}

/// The distance from `q` to the move's path.
fn move_distance(m: &MillMove, q: P) -> f64 {
    match *m {
        MillMove::Line { from, to } => {
            let d = sub(to, from);
            let ll = dot(d, d);
            if ll == 0.0 {
                return dist(q, from);
            }
            let t = (dot(sub(q, from), d) / ll).clamp(0.0, 1.0);
            dist(q, add(from, scale(d, t)))
        }
        MillMove::Arc { centre, radius, start, sweep } => {
            let (a0, span) = if sweep >= 0.0 { (start, sweep) } else { (start + sweep, -sweep) };
            let angle = (q[1] - centre[1]).atan2(q[0] - centre[0]);
            if span >= TAU || (angle - a0).rem_euclid(TAU) <= span {
                (dist(q, centre) - radius).abs()
            } else {
                let end = |a: f64| [centre[0] + radius * a.cos(), centre[1] + radius * a.sin()];
                dist(q, end(a0)).min(dist(q, end(a0 + span)))
            }
        }
    }
}

/// Where the carriers `x` and `y` cross or touch (within `tol`).
fn meetings(x: &Carrier, y: &Carrier, tol: f64) -> Vec<P> {
    let line_circle = |a: P, b: P, c: P, r: f64| -> Vec<P> {
        let d = sub(b, a);
        let length = norm(d);
        let u = scale(d, 1.0 / length);
        let foot = add(a, scale(u, dot(sub(c, a), u)));
        let h = dist(foot, c);
        if h > r + tol {
            Vec::new()
        } else if (h - r).abs() <= tol {
            vec![foot]
        } else {
            let k = (r * r - h * h).sqrt();
            vec![add(foot, scale(u, k)), sub(foot, scale(u, k))]
        }
    };
    match (*x, *y) {
        (Carrier::Seg { a: p0, b: p1 }, Carrier::Seg { a: q0, b: q1 }) => {
            let (d1, d2) = (sub(p1, p0), sub(q1, q0));
            let det = cross(d1, d2);
            if det.abs() < 1e-15 * norm(d1) * norm(d2) {
                return Vec::new();
            }
            let r = sub(q0, p0);
            let s = cross(r, d2) / det;
            vec![add(p0, scale(d1, s))]
        }
        (Carrier::Seg { a, b }, Carrier::Arc { c, r, .. }) | (Carrier::Arc { c, r, .. }, Carrier::Seg { a, b }) => line_circle(a, b, c, r),
        (Carrier::Arc { c: c1, r: r1, .. }, Carrier::Arc { c: c2, r: r2, .. }) => {
            let d = dist(c1, c2);
            if d <= tol {
                return Vec::new();
            }
            let u = scale(sub(c2, c1), 1.0 / d);
            if (d - (r1 + r2)).abs() <= tol {
                return vec![add(c1, scale(u, r1))];
            }
            if (d - (r1 - r2).abs()).abs() <= tol {
                let sign = if r1 >= r2 { 1.0 } else { -1.0 };
                return vec![add(c1, scale(u, sign * r1))];
            }
            if d > r1 + r2 || d < (r1 - r2).abs() {
                return Vec::new();
            }
            let along = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);
            let h = (r1 * r1 - along * along).max(0.0).sqrt();
            let mid = add(c1, scale(u, along));
            let n = [-u[1], u[0]];
            vec![add(mid, scale(n, h)), sub(mid, scale(n, h))]
        }
    }
}

/// A piece of a carrier from t0 to t1 (t1 may pass 1 on a whole circle), traversed forward or back.
#[derive(Clone, Copy, Debug)]
struct Piece {
    carrier: usize,
    t0: f64,
    t1: f64,
    forward: bool,
}

/// The region an upright flat-bottomed cutter of `radius` covers as its axis runs `moves` (all at
/// one level), extruded from `bottom` to `top`: one prism for each connected part of the region
/// (its holes through it). The moves' order does not matter.
pub fn flat_mill_region_prisms(moves: &[MillMove], radius: f64, bottom: f64, top: f64) -> Result<Vec<BrepSolid>, KernelRefusal> {
    if moves.is_empty() {
        return Err(input("moves", "no moves".into()));
    }
    if !(radius.is_finite() && radius > 0.0) {
        return Err(input("radius", "the cutter's radius must be positive".into()));
    }
    if !(bottom.is_finite() && top.is_finite() && top > bottom) {
        return Err(input("levels", "the top must lie above the bottom".into()));
    }
    // The moves' box: worked about its centre (exactly translated back in the prism), tolerances
    // read from its size, so that the result does not depend on where the moves lie.
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    let mut grow = |q: P, pad: f64| {
        for k in 0..2 {
            lo[k] = lo[k].min(q[k] - pad);
            hi[k] = hi[k].max(q[k] + pad);
        }
    };
    for m in moves {
        match *m {
            MillMove::Line { from, to } => {
                if !(from.iter().chain(to.iter()).all(|v| v.is_finite())) {
                    return Err(input("line", "a move's ends must be finite".into()));
                }
                grow(from, radius);
                grow(to, radius);
            }
            MillMove::Arc { centre, radius: ar, start, sweep } => {
                if !(ar.is_finite() && ar > 0.0 && sweep.is_finite() && sweep != 0.0 && start.is_finite() && centre.iter().all(|v| v.is_finite())) {
                    return Err(input("arc", "an arc move needs a positive radius and a sweep".into()));
                }
                grow(centre, ar + radius);
            }
        }
    }
    let origin: P = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])];
    let scale_len = (hi[0] - lo[0]).hypot(hi[1] - lo[1]).max(radius);
    let local: Vec<MillMove> = moves
        .iter()
        .map(|m| match *m {
            MillMove::Line { from, to } => MillMove::Line { from: sub(from, origin), to: sub(to, origin) },
            MillMove::Arc { centre, radius, start, sweep } => MillMove::Arc { centre: sub(centre, origin), radius, start, sweep },
        })
        .collect();
    let moves = &local[..];
    // Merging and meeting tolerance: the cutter's size, not the moves' spread (two circles a unit
    // apart stay apart however far the program reaches), floored at what the coordinates resolve.
    let tol = (1e-9 * radius).max(1e-14 * scale_len);
    // Carriers.
    let mut carriers: Vec<Carrier> = Vec::new();
    let circle = |c: P, r: f64, a0: f64, span: f64, carriers: &mut Vec<Carrier>| carriers.push(Carrier::Arc { c, r, a0, span });
    for m in moves {
        match *m {
            MillMove::Line { from, to } => {
                let d = sub(to, from);
                let length = norm(d);
                circle(from, radius, 0.0, TAU, &mut carriers);
                if length > tol {
                    let n = [-d[1] / length * radius, d[0] / length * radius];
                    carriers.push(Carrier::Seg { a: add(from, n), b: add(to, n) });
                    carriers.push(Carrier::Seg { a: sub(from, n), b: sub(to, n) });
                    circle(to, radius, 0.0, TAU, &mut carriers);
                }
            }
            MillMove::Arc { centre, radius: ar, start, sweep } => {
                let (a0, span) = if sweep >= 0.0 { (start, sweep.min(TAU)) } else { (start + sweep, (-sweep).min(TAU)) };
                circle(centre, ar + radius, a0, span, &mut carriers);
                if ar - radius > tol {
                    circle(centre, ar - radius, a0, span, &mut carriers);
                }
                for a in [start, start + sweep] {
                    circle([centre[0] + ar * a.cos(), centre[1] + ar * a.sin()], radius, 0.0, TAU, &mut carriers);
                }
            }
        }
    }
    let carriers = merge_carriers(carriers, tol);
    // Cuts: ends, crossings and touches.
    let mut cuts: Vec<Vec<f64>> = carriers.iter().map(|c| if c.whole_circle() { Vec::new() } else { vec![0.0, 1.0] }).collect();
    let boxes: Vec<[f64; 4]> = carriers
        .iter()
        .map(|c| match *c {
            Carrier::Seg { a, b } => [a[0].min(b[0]), a[1].min(b[1]), a[0].max(b[0]), a[1].max(b[1])],
            Carrier::Arc { c, r, .. } => [c[0] - r, c[1] - r, c[0] + r, c[1] + r],
        })
        .collect();
    for i in 0..carriers.len() {
        for j in i + 1..carriers.len() {
            let (bi, bj) = (boxes[i], boxes[j]);
            if bi[0] > bj[2] + tol || bj[0] > bi[2] + tol || bi[1] > bj[3] + tol || bj[1] > bi[3] + tol {
                continue;
            }
            for p in meetings(&carriers[i], &carriers[j], tol) {
                if let (Some(ti), Some(tj)) = (carriers[i].param_of(p, 1e3 * tol), carriers[j].param_of(p, 1e3 * tol)) {
                    cuts[i].push(ti);
                    cuts[j].push(tj);
                }
            }
        }
    }
    // Pieces on the boundary, the region on their left.
    let depth = |q: P| moves.iter().map(|m| move_distance(m, q)).fold(f64::INFINITY, f64::min) - radius;
    // Probes stand off a piece by far less than the smallest feature (the cutter, every arc, every
    // segment), so that none jumps a narrow hole or sliver.
    let feature = carriers.iter().fold(radius, |m, c| match *c {
        Carrier::Seg { a, b } => m.min(dist(a, b)),
        Carrier::Arc { r, .. } => m.min(r),
    });
    // (The exact distance to the moves resolves far below this, about the box's centre; a gap or
    // hole narrower than it is below the coordinates' resolution at this size.)
    let probe = (1e-3 * feature).min(1e-10 * scale_len).max(1e-13 * scale_len);
    let mut pieces: Vec<Piece> = Vec::new();
    for (index, carrier) in carriers.iter().enumerate() {
        let c = &mut cuts[index];
        c.sort_by(f64::total_cmp);
        c.dedup_by(|x, y| (*x - *y).abs() < 1e-12);
        let mut spans: Vec<(f64, f64)> = c.windows(2).map(|w| (w[0], w[1])).collect();
        if carrier.whole_circle() {
            match c.len() {
                0 => spans.push((0.0, 1.0)),
                _ => spans.push((c[c.len() - 1], c[0] + 1.0)),
            }
        }
        for (t0, t1) in spans {
            if t1 - t0 < 1e-12 || dist(carrier.at(t0), carrier.at(t1)) < tol && !(carrier.whole_circle() && (t1 - t0) > 0.5) {
                continue;
            }
            let mut sides = Vec::new();
            for f in [1.0 / 3.0, 2.0 / 3.0] {
                let t = t0 + (t1 - t0) * f;
                let (p, d) = (carrier.at(t), carrier.tangent(t));
                let n = scale([-d[1], d[0]], probe / norm(d));
                let (left, right) = (depth(add(p, n)) < 0.0, depth(sub(p, n)) < 0.0);
                sides.push(match (left, right) {
                    (true, false) => Some(true),
                    (false, true) => Some(false),
                    _ => None,
                });
            }
            match (sides[0], sides[1]) {
                (Some(a), Some(b)) if a == b => pieces.push(Piece { carrier: index, t0, t1, forward: a }),
                (None, None) => {}
                _ => return Err(refuse("mill_region_piece", "a piece of the region's boundary is on the boundary only in part".into())),
            }
        }
    }
    if pieces.is_empty() {
        return Err(refuse("mill_region_empty", "no boundary found".into()));
    }
    let start_of = |p: &Piece| carriers[p.carrier].at(if p.forward { p.t0 } else { p.t1 });
    let end_of = |p: &Piece| carriers[p.carrier].at(if p.forward { p.t1 } else { p.t0 });
    let dir_at = |p: &Piece, at_start: bool| -> P {
        let t = if at_start == p.forward { p.t0 } else { p.t1 };
        let d = carriers[p.carrier].tangent(t);
        if p.forward { d } else { scale(d, -1.0) }
    };
    // Loops: at a junction take the piece turning furthest right (keeps touching parts apart).
    let join = 1e3 * tol;
    let mut used = vec![false; pieces.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    while let Some(first) = (0..pieces.len()).find(|&k| !used[k]) {
        used[first] = true;
        let mut order = vec![first];
        let origin = start_of(&pieces[first]);
        loop {
            let last = &pieces[*order.last().unwrap()];
            let end = end_of(last);
            if dist(end, origin) < join && order.len() > 0 && (order.len() > 1 || carriers[last.carrier].whole_circle() || dist(start_of(last), end) < join) {
                break;
            }
            let incoming = dir_at(last, false);
            let next = (0..pieces.len())
                .filter(|&k| !used[k] && dist(start_of(&pieces[k]), end) < join)
                .min_by(|&x, &y| {
                    let turn = |k: usize| {
                        let out = dir_at(&pieces[k], true);
                        cross(incoming, out).atan2(dot(incoming, out))
                    };
                    turn(x).total_cmp(&turn(y))
                });
            let Some(next) = next else {
                return Err(refuse("mill_region_open", "the region's boundary does not close".into()));
            };
            used[next] = true;
            order.push(next);
        }
        loops.push(order);
    }
    // Sampled polygons: orientation and nesting.
    let polygon = |order: &Vec<usize>| -> Vec<P> {
        let mut points = Vec::new();
        for &k in order {
            let p = &pieces[k];
            for i in 0..32 {
                let f = i as f64 / 32.0;
                let t = if p.forward { p.t0 + (p.t1 - p.t0) * f } else { p.t1 - (p.t1 - p.t0) * f };
                points.push(carriers[p.carrier].at(t));
            }
        }
        points
    };
    let polygons: Vec<Vec<P>> = loops.iter().map(polygon).collect();
    let area = |poly: &Vec<P>| (0..poly.len()).map(|i| cross(poly[i], poly[(i + 1) % poly.len()])).sum::<f64>() * 0.5;
    let areas: Vec<f64> = polygons.iter().map(area).collect();
    let inside = |poly: &Vec<P>, q: P| {
        let mut inside = false;
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                inside = !inside;
            }
        }
        inside
    };
    let outers: Vec<usize> = (0..loops.len()).filter(|&k| areas[k] > 0.0).collect();
    let mut holes_of: Vec<Vec<usize>> = vec![Vec::new(); loops.len()];
    for hole in (0..loops.len()).filter(|&k| areas[k] <= 0.0) {
        let q = polygons[hole][0];
        let host = outers.iter().copied().filter(|&o| inside(&polygons[o], q)).min_by(|&x, &y| areas[x].total_cmp(&areas[y]));
        let Some(host) = host else {
            return Err(refuse("mill_region_hole", "a hole of the region lies in no outer loop".into()));
        };
        holes_of[host].push(hole);
    }
    let mut solids = Vec::with_capacity(outers.len());
    for &outer in &outers {
        let mut part = vec![loops[outer].clone()];
        part.extend(holes_of[outer].iter().map(|&h| loops[h].clone()));
        solids.push(prism(&carriers, &pieces, &part, bottom, top, origin)?);
    }
    Ok(solids)
}

/// Carriers on one line (segments) or one circle (arcs) merged: overlapping segments into one, arcs
/// into the circle's covered arcs (a whole circle where any is whole).
fn merge_carriers(carriers: Vec<Carrier>, tol: f64) -> Vec<Carrier> {
    let mut out: Vec<Carrier> = Vec::new();
    // Circles.
    let mut circles: Vec<(P, f64, Vec<(f64, f64)>)> = Vec::new();
    let mut lines: Vec<(P, P, Vec<(f64, f64)>)> = Vec::new(); // (point on line, unit direction, intervals)
    for c in carriers {
        match c {
            Carrier::Arc { c, r, a0, span } => {
                let found = circles.iter_mut().find(|(cc, rr, _)| dist(*cc, c) <= tol && (rr - r).abs() <= tol);
                let a0 = a0.rem_euclid(TAU);
                match found {
                    Some(entry) => entry.2.push((a0, span)),
                    None => circles.push((c, r, vec![(a0, span)])),
                }
            }
            Carrier::Seg { a, b } => {
                let d = sub(b, a);
                let l = norm(d);
                let mut u = scale(d, 1.0 / l);
                if u[0] < -1e-12 || (u[0].abs() <= 1e-12 && u[1] < 0.0) {
                    u = scale(u, -1.0);
                }
                let found = lines.iter_mut().find(|(o, uu, _)| (cross(*uu, u)).abs() <= 1e-12 && cross(*uu, sub(a, *o)).abs() <= tol);
                let (sa, sb) = match &found {
                    Some((o, uu, _)) => (dot(sub(a, *o), *uu), dot(sub(b, *o), *uu)),
                    None => (0.0, dot(sub(b, a), u)),
                };
                let iv = (sa.min(sb), sa.max(sb));
                match found {
                    Some(entry) => entry.2.push(iv),
                    None => lines.push((a, u, vec![iv])),
                }
            }
        }
    }
    for (c, r, arcs) in circles {
        if arcs.iter().any(|&(_, s)| s >= TAU - 1e-12) {
            out.push(Carrier::Arc { c, r, a0: 0.0, span: TAU });
            continue;
        }
        // Union of arcs on the circle: each cut at TAU into pieces within [0, TAU], merged on the
        // line, then the piece ending at TAU joined to the one starting at 0 (one arc across 0).
        let mut pieces: Vec<(f64, f64)> = Vec::new();
        for (a0, span) in arcs {
            let (s0, e0) = (a0.rem_euclid(TAU), a0.rem_euclid(TAU) + span);
            if e0 > TAU {
                pieces.push((s0, TAU));
                pieces.push((0.0, e0 - TAU));
            } else {
                pieces.push((s0, e0));
            }
        }
        pieces.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for (s, e) in pieces {
            if let Some(last) = merged.last_mut() {
                if s <= last.1 + 1e-12 {
                    last.1 = last.1.max(e);
                    continue;
                }
            }
            merged.push((s, e));
        }
        if merged.len() == 1 && merged[0].0 <= 1e-12 && merged[0].1 >= TAU - 1e-12 {
            out.push(Carrier::Arc { c, r, a0: 0.0, span: TAU });
            continue;
        }
        if merged.len() > 1 && merged[0].0 <= 1e-12 && merged[merged.len() - 1].1 >= TAU - 1e-12 {
            let first = merged.remove(0);
            merged.last_mut().unwrap().1 = first.1 + TAU;
        }
        for (s, e) in merged {
            let span = (e - s).min(TAU);
            out.push(Carrier::Arc { c, r, a0: s, span });
        }
    }
    for (o, u, mut ivs) in lines {
        ivs.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut merged: Vec<(f64, f64)> = Vec::new();
        for (s, e) in ivs {
            if let Some(last) = merged.last_mut() {
                if s <= last.1 + tol {
                    last.1 = last.1.max(e);
                    continue;
                }
            }
            merged.push((s, e));
        }
        for (s, e) in merged {
            out.push(Carrier::Seg { a: add(o, scale(u, s)), b: add(o, scale(u, e)) });
        }
    }
    out
}

/// One prism: `loops` (the outer loop first) of pieces, from `bottom` to `top`.
fn prism(carriers: &[Carrier], pieces: &[Piece], loops: &[Vec<usize>], bottom: f64, top: f64, origin: P) -> Result<BrepSolid, KernelRefusal> {
    let (ex, ey, ez) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
    let world = |q: P, h: f64| Vec3::new(q[0] + origin[0], q[1] + origin[1], h);
    // The piece's curve at height h, in its direction of travel.
    let curve = |piece: &Piece, h: f64| -> Result<NurbsCurve, KernelRefusal> {
        let carrier = &carriers[piece.carrier];
        let (p0, p1) = (carrier.at(piece.t0), carrier.at(piece.t1));
        match *carrier {
            Carrier::Seg { .. } => {
                let (a, b) = if piece.forward { (p0, p1) } else { (p1, p0) };
                frag(make_line(world(a, h), world(b, h)), "mill_line")
            }
            Carrier::Arc { c, r, a0, span } => {
                let (s, e) = (a0 + span * piece.t0, a0 + span * piece.t1);
                let arc = frag(make_arc(world(c, h), ex, ey, r, s, e), "mill_arc")?;
                if piece.forward { Ok(arc) } else { frag(arc.reversed(), "mill_arc") }
            }
        }
    };
    let mut shell = Shell::new();
    let mut top_loops = Vec::new();
    let mut bottom_loops = Vec::new();
    let mut extent: Vec<P> = Vec::new();
    for order in loops {
        let count = order.len();
        let corners: Vec<P> = order
            .iter()
            .map(|&k| {
                let p = &pieces[k];
                carriers[p.carrier].at(if p.forward { p.t0 } else { p.t1 })
            })
            .collect();
        extent.extend(corners.iter().copied());
        for &k in order {
            if let Carrier::Arc { c, r, .. } = carriers[pieces[k].carrier] {
                extent.extend([[c[0] - r, c[1] - r], [c[0] + r, c[1] + r]]);
            }
        }
        let (bottom_v, top_v): (Vec<u64>, Vec<u64>) = corners.iter().map(|&q| (shell.vertex(world(q, bottom)), shell.vertex(world(q, top)))).unzip();
        let mut bottom_e = Vec::with_capacity(count);
        let mut top_e = Vec::with_capacity(count);
        let mut curves = Vec::with_capacity(count);
        for (slot, &k) in order.iter().enumerate() {
            let next = (slot + 1) % count;
            let (cb, ct) = (curve(&pieces[k], bottom)?, curve(&pieces[k], top)?);
            bottom_e.push(shell.edge(cb.clone(), bottom_v[slot], bottom_v[next])?);
            top_e.push(shell.edge(ct.clone(), top_v[slot], top_v[next])?);
            curves.push((cb, ct));
        }
        let verticals: Vec<u64> = (0..count)
            .map(|k| shell.edge(frag(make_line(world(corners[k], bottom), world(corners[k], top)), "mill_vertical")?, bottom_v[k], top_v[k]))
            .collect::<Result<_, _>>()?;
        top_loops.push((0..count).map(|k| Ok((top_e[k], true, plane_pcurve(world([0.0, 0.0], top), ex, ey, &curves[k].1, true)?))).collect::<Result<Vec<_>, KernelRefusal>>()?);
        bottom_loops.push((0..count).rev().map(|k| Ok((bottom_e[k], false, plane_pcurve(world([0.0, 0.0], bottom), ey, ex, &curves[k].0, false)?))).collect::<Result<Vec<_>, KernelRefusal>>()?);
        for k in 0..count {
            let next = (k + 1) % count;
            let cb = &curves[k].0;
            let rows: Vec<Vec<Vec4>> = cb
                .control_points
                .iter()
                .map(|p| {
                    let x = p.point().expect("a control point");
                    vec![Vec4::from_point(x, p.w), Vec4::from_point(x.add(ez.scale(top - bottom)), p.w)]
                })
                .collect();
            let surface = frag(NurbsSurface::new(cb.degree, 1, cb.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], rows), "mill_wall")?;
            let [u0, u1] = frag(cb.domain(), "mill_domain")?;
            shell.face(
                surface,
                vec![
                    (bottom_e[k], true, uv_line([u0, 0.0], [u1, 0.0])?),
                    (verticals[next], true, uv_line([u1, 0.0], [u1, 1.0])?),
                    (top_e[k], false, uv_line([u1, 1.0], [u0, 1.0])?),
                    (verticals[k], false, uv_line([u0, 1.0], [u0, 0.0])?),
                ],
            );
        }
    }
    let (lo, hi) = extent.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), q| ([lo[0].min(q[0]), lo[1].min(q[1])], [hi[0].max(q[0]), hi[1].max(q[1])]));
    shell.face_loops(plane_patch(world([0.0, 0.0], top), ex, ey, lo, hi)?, top_loops);
    shell.face_loops(plane_patch(world([0.0, 0.0], bottom), ey, ex, [lo[1], lo[0]], [hi[1], hi[0]])?, bottom_loops);
    let body = shell.solid();
    if let Some(issue) = body.validate().first() {
        return Err(refuse("mill_region_assembly", format!("the region's prism is not valid: {}", issue.message)));
    }
    Ok(body)
}

fn uv_line(a: [f64; 2], b: [f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
    frag(make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0)), "pcurve")
}

/// The plane through `origin` spanned by unit `u`, `v`, over the box `lo`..`hi` of (u, v).
fn plane_patch(origin: Vec3, u: Vec3, v: Vec3, lo: [f64; 2], hi: [f64; 2]) -> Result<NurbsSurface, KernelRefusal> {
    let (lo, hi) = ([lo[0] - 1.0, lo[1] - 1.0], [hi[0] + 1.0, hi[1] + 1.0]);
    let at = |s: f64, t: f64| Vec4::from_point(origin.add(u.scale(s)).add(v.scale(t)), 1.0);
    frag(
        NurbsSurface::new(
            1,
            1,
            vec![lo[0], lo[0], hi[0], hi[0]],
            vec![lo[1], lo[1], hi[1], hi[1]],
            vec![vec![at(lo[0], lo[1]), at(lo[0], hi[1])], vec![at(hi[0], lo[1]), at(hi[0], hi[1])]],
        ),
        "mill_plane",
    )
}

fn plane_pcurve(origin: Vec3, u: Vec3, v: Vec3, curve: &NurbsCurve, forward: bool) -> Result<NurbsCurve, KernelRefusal> {
    let mut points = Vec::with_capacity(curve.control_points.len());
    for p in &curve.control_points {
        let x = frag(p.point(), "control_point")?.sub(origin);
        points.push(Vec4::from_point(Vec3::new(x.dot(u), x.dot(v), 0.0), p.w));
    }
    let pcurve = frag(NurbsCurve::new(curve.degree, curve.knots.clone(), points), "plane_pcurve")?;
    if forward { Ok(pcurve) } else { frag(pcurve.reversed(), "plane_pcurve") }
}

struct Shell {
    vertices: Vec<VertexRecord>,
    edges: Vec<EdgeRecord>,
    faces: Vec<FaceRecord>,
    next: u64,
    holes: usize,
}

impl Shell {
    fn new() -> Self {
        Self { vertices: Vec::new(), edges: Vec::new(), faces: Vec::new(), next: 1, holes: 0 }
    }
    fn id(&mut self) -> u64 {
        self.next += 1;
        self.next
    }
    fn vertex(&mut self, point: Vec3) -> u64 {
        let id = self.id();
        self.vertices.push(VertexRecord { id, point });
        id
    }
    fn edge(&mut self, curve: NurbsCurve, start: u64, end: u64) -> Result<u64, KernelRefusal> {
        let domain = frag(curve.domain(), "edge_domain")?;
        let id = self.id();
        self.edges.push(EdgeRecord { id, curve, t0: domain[0], t1: domain[1], start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None });
        Ok(id)
    }
    fn face(&mut self, surface: NurbsSurface, coedges: Vec<(u64, bool, NurbsCurve)>) {
        self.face_loops(surface, vec![coedges]);
    }
    fn face_loops(&mut self, surface: NurbsSurface, loops: Vec<Vec<(u64, bool, NurbsCurve)>>) {
        let mut records = Vec::with_capacity(loops.len());
        for coedges in loops {
            let mut list = Vec::with_capacity(coedges.len());
            for (edge, forward, pcurve) in coedges {
                let id = self.id();
                list.push(CoedgeRecord { id, edge_id: edge, forward, pcurve });
            }
            let id = self.id();
            records.push(LoopRecord { id, coedges: list });
        }
        self.holes += records.len().saturating_sub(1);
        let id = self.id();
        self.faces.push(FaceRecord { id, surface, same_sense: true, loops: records, name: None });
    }
    fn solid(mut self) -> BrepSolid {
        let shell = self.id();
        let id = self.id();
        let euler = self.vertices.len() as i64 - self.edges.len() as i64 + self.faces.len() as i64 - self.holes as i64;
        BrepSolid {
            mass_properties_cache: Default::default(),
            id,
            vertices: self.vertices,
            edges: self.edges,
            shells: vec![ShellRecord { id: shell, faces: self.faces }],
            genus: (2 - euler) / 2,
            kind: Default::default(),
            cells: Default::default(),
        }
    }
}

