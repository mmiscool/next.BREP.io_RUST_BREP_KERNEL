//! A pyramid frustum swept by turns about axes through its apex: a gear's
//! tooth rolled in mesh on shafts that meet (a bevel gear cut by its
//! pinion), or any convex-sided cutter whose flanks are planes through the
//! point its motion holds still.
//!
//! Every turn holds the apex O still and every side of the pyramid is a plane
//! through O, so the pyramid (without its end planes) at every time is the
//! cone over a convex spherical polygon, and what all of them sweep is the
//! cone over the region of directions those polygons sweep on the sphere.
//! The frustum's end planes cut it off: within the distances from O between
//! the inner end's farthest point and the outer end's nearest, the sweep is
//! exactly that cone ([`sweep_pyramid_within`] refuses a range outside them).
//!
//! The region is read in the plane square to a middle direction m, at
//! distance 1 from O (each direction d its point d/(d·m)). Its boundary
//! lies on three kinds of curve, each known in closed form at every time:
//! - the track of a corner (a corner direction turned);
//! - the envelope of a side: at each time the point of the side whose
//!   velocity lies in the side's own plane, where that point lies on the side;
//! - the sides at the first and last time.
//!
//! Each curve is sampled, the samples crossing one another found and solved
//! on the exact curves (a side's envelope meets the corner it hands over to
//! at the time its point reaches the end, solved on that), and each stretch
//! between crossings kept where membership differs on its two sides.
//! Membership steps the time by the largest angle by which the direction
//! lies outside a side's plane (in the moving tool's frame) over a bound on
//! how fast the direction turns there, so it skips no time at which the
//! direction is inside. The kept stretches close into one loop; its curves
//! are fitted through their exact points on the planes square to m at the
//! range's ends, and lofted (each wall a ruled cone through O).

use crate::{loft_profile_brep, KernelRefusal, KernelStage, NurbsCurve, SolidMotion, Vec3};

/// The prefix of every refusal of this operation.
pub const APEX_SWEEP_REFUSAL: &str = "sweepPyramidWithin";

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Collect, what, format!("{APEX_SWEEP_REFUSAL}: {message}"))
}

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{APEX_SWEEP_REFUSAL}: {message}"))
}

/// A frustum of a pyramid: the convex polygon `section` (corners in order, in
/// the plane square to `axis` at distance `at` from `apex`, coordinates along
/// `x_axis` and `axis × x_axis` from the axis's point there), the pyramid
/// from `apex` over it, cut off by the planes square to `axis` at distances
/// `from` and `to` from the apex.
#[derive(Clone, Debug)]
pub struct Pyramid {
    pub apex: Vec3,
    pub axis: Vec3,
    pub x_axis: Vec3,
    pub at: f64,
    pub section: Vec<[f64; 2]>,
    pub from: f64,
    pub to: f64,
}

type P2 = [f64; 2];

fn rotate(v: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (c, s) = (angle.cos(), angle.sin());
    v.scale(c).add(axis.cross(v).scale(s)).add(axis.scale(axis.dot(v) * (1.0 - c)))
}

/// The motion: turns about unit axes through the apex at rates, applied in
/// order (the first to the tool first).
struct Turns {
    axes: Vec<(Vec3, f64)>,
    t0: f64,
    t1: f64,
}

impl Turns {
    fn apply(&self, t: f64, v: Vec3) -> Vec3 {
        self.axes.iter().fold(v, |v, &(a, w)| rotate(v, a, w * t))
    }
    fn invert(&self, t: f64, v: Vec3) -> Vec3 {
        self.axes.iter().rev().fold(v, |v, &(a, w)| rotate(v, a, -w * t))
    }
    /// The angular velocity at t, in the world: each turn's axis carried
    /// by the turns after it.
    fn omega(&self, t: f64) -> Vec3 {
        let mut total = Vec3::new(0.0, 0.0, 0.0);
        for (k, &(a, w)) in self.axes.iter().enumerate() {
            let carried = self.axes[k + 1..].iter().fold(a, |v, &(b, u)| rotate(v, b, u * t));
            total = total.add(carried.scale(w));
        }
        total
    }
    /// A bound on how fast a direction turns in the tool's frame.
    fn rate(&self) -> f64 {
        self.axes.iter().map(|(_, w)| w.abs()).sum::<f64>() * (1.0 + 1e-12)
    }
}

/// A boundary candidate.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Kind {
    Corner(usize),
    Envelope(usize),
    /// Side k at the first (false) or last (true) time: parameter its fraction.
    End(usize, bool),
}

struct Region {
    turns: Turns,
    /// Corner directions in the tool's frame (unit), in order round.
    corners: Vec<Vec3>,
    /// Each side's outward unit normal in the tool's frame (its plane through O).
    normals: Vec<Vec3>,
    /// The chart: middle direction and its frame.
    m: Vec3,
    e1: Vec3,
    e2: Vec3,
}

impl Region {
    fn chart(&self, d: Vec3) -> P2 {
        let n = d.dot(self.m);
        [d.dot(self.e1) / n, d.dot(self.e2) / n]
    }
    fn direction(&self, y: P2) -> Vec3 {
        let d = self.m.add(self.e1.scale(y[0])).add(self.e2.scale(y[1]));
        d.scale(1.0 / d.length())
    }

    /// The fraction along side k (from corner k to k + 1, as the chord of the
    /// two directions) of its envelope point at t: where the side's points'
    /// velocities lie in its plane.
    fn foot(&self, k: usize, t: f64) -> f64 {
        let n = self.corners.len();
        let (a, b) = (self.turns.apply(t, self.corners[k]), self.turns.apply(t, self.corners[(k + 1) % n]));
        let normal = self.turns.apply(t, self.normals[k]);
        let w = self.turns.omega(t);
        // (ω × x)·n = x·(n × ω), linear in x along the chord
        let g = normal.cross(w);
        let (ga, gb) = (a.dot(g), b.dot(g));
        if ga == gb { f64::NAN } else { ga / (ga - gb) }
    }

    fn eval(&self, kind: Kind, s: f64) -> P2 {
        let n = self.corners.len();
        let on_side = |k: usize, f: f64, t: f64| {
            let (a, b) = (self.corners[k], self.corners[(k + 1) % n]);
            self.turns.apply(t, a.scale(1.0 - f).add(b.scale(f)))
        };
        let d = match kind {
            Kind::Corner(k) => self.turns.apply(s, self.corners[k]),
            Kind::Envelope(k) => on_side(k, self.foot(k, s).clamp(0.0, 1.0), s),
            Kind::End(k, end) => on_side(k, s, if end { self.turns.t1 } else { self.turns.t0 }),
        };
        self.chart(d)
    }

    /// Whether the direction of chart point `y` lies in the swept region:
    /// the time stepped by the largest angle outside a side's plane over
    /// the turning rate. None where ten million steps do not reach the end
    /// (a direction lingering just outside a side; cross review: it was
    /// read as inside).
    fn member(&self, y: P2) -> Option<bool> {
        let d = self.direction(y);
        let (t0, t1) = (self.turns.t0, self.turns.t1);
        let rate = self.turns.rate();
        let floor = (t1 - t0) * 1e-12;
        let mut t = t0;
        for _ in 0..10_000_000 {
            let p = self.turns.invert(t, d);
            let outside = self.normals.iter().map(|n| p.dot(*n).clamp(-1.0, 1.0).asin()).fold(f64::NEG_INFINITY, f64::max);
            if outside < 0.0 {
                return Some(true);
            }
            if t >= t1 {
                return Some(false);
            }
            t = (t + (outside / rate).max(floor)).min(t1);
        }
        None
    }
}

/// A stretch of a candidate between two parameters.
#[derive(Clone, Copy, Debug)]
struct Piece {
    kind: Kind,
    a: f64,
    b: f64,
}

/// The root of `f` on [a, b] where it changes sign, by bisection.
fn bisect(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    let fa = f(a);
    for _ in 0..200 {
        let m = 0.5 * (a + b);
        if m <= a || m >= b {
            break;
        }
        if (f(m) < 0.0) == (fa < 0.0) { a = m } else { b = m }
    }
    0.5 * (a + b)
}

const SAMPLES: usize = 512;

/// The region's boundary: one loop of stretches, the region on its left.
fn boundary(region: &Region) -> Result<Vec<Piece>, KernelRefusal> {
    let n = region.corners.len();
    let (t0, t1) = (region.turns.t0, region.turns.t1);
    // The candidates: each corner's track, each side's envelope over the runs
    // of time its point lies on the side, each side at both ends.
    let mut candidates: Vec<Piece> = Vec::new();
    for k in 0..n {
        candidates.push(Piece { kind: Kind::Corner(k), a: t0, b: t1 });
        candidates.push(Piece { kind: Kind::End(k, false), a: 0.0, b: 1.0 });
        candidates.push(Piece { kind: Kind::End(k, true), a: 0.0, b: 1.0 });
        let at = |i: usize| t0 + (t1 - t0) * i as f64 / SAMPLES as f64;
        let on = |t: f64| { let f = region.foot(k, t); f.is_finite() && (0.0..=1.0).contains(&f) };
        let mut start: Option<f64> = if on(t0) { Some(t0) } else { None };
        for i in 1..=SAMPLES {
            let (ta, tb) = (at(i - 1), at(i));
            match (on(ta), on(tb)) {
                (false, true) => start = Some(edge_of_run(region, k, ta, tb)),
                (true, false) => {
                    let end = edge_of_run(region, k, ta, tb);
                    if let Some(s) = start.take() {
                        if end > s {
                            candidates.push(Piece { kind: Kind::Envelope(k), a: s, b: end });
                        }
                    }
                }
                _ => {}
            }
        }
        if let Some(s) = start {
            if t1 > s {
                candidates.push(Piece { kind: Kind::Envelope(k), a: s, b: t1 });
            }
        }
    }
    // Sample each, and find where any two cross (solved on the exact curves).
    let polyline = |p: &Piece| -> Vec<(f64, P2)> { (0..=SAMPLES).map(|i| { let s = p.a + (p.b - p.a) * i as f64 / SAMPLES as f64; (s, region.eval(p.kind, s)) }).collect() };
    let lines: Vec<Vec<(f64, P2)>> = candidates.iter().map(polyline).collect();
    let size = lines.iter().flatten().map(|(_, q)| q[0].abs().max(q[1].abs())).fold(0.0f64, f64::max).max(1e-3);
    let mut cuts: Vec<Vec<f64>> = candidates.iter().map(|p| vec![p.a, p.b]).collect();
    // An envelope hands over (tangent) to the corner its point reaches, or
    // starts or ends on a side at the first or last time: cut that curve there.
    let index = |kind: Kind| candidates.iter().position(|c| c.kind == kind);
    for i in 0..candidates.len() {
        let Kind::Envelope(k) = candidates[i].kind else { continue };
        for t in [candidates[i].a, candidates[i].b] {
            let f = region.foot(k, t);
            let (target, at) = if t == t0 {
                (Kind::End(k, false), f.clamp(0.0, 1.0))
            } else if t == t1 {
                (Kind::End(k, true), f.clamp(0.0, 1.0))
            } else if f < 0.5 {
                (Kind::Corner(k), t)
            } else {
                (Kind::Corner((k + 1) % n), t)
            };
            if let Some(j) = index(target) {
                cuts[j].push(at);
            }
        }
    }
    for i in 0..candidates.len() {
        for j in i + 1..candidates.len() {
            for w in 0..SAMPLES {
                for v in 0..SAMPLES {
                    let (p0, p1, q0, q1) = (lines[i][w].1, lines[i][w + 1].1, lines[j][v].1, lines[j][v + 1].1);
                    // a quick box reject
                    if p0[0].max(p1[0]) < q0[0].min(q1[0]) || q0[0].max(q1[0]) < p0[0].min(p1[0]) || p0[1].max(p1[1]) < q0[1].min(q1[1]) || q0[1].max(q1[1]) < p0[1].min(p1[1]) {
                        continue;
                    }
                    let (r, s) = ([p1[0] - p0[0], p1[1] - p0[1]], [q1[0] - q0[0], q1[1] - q0[1]]);
                    let den = r[0] * s[1] - r[1] * s[0];
                    if den == 0.0 {
                        continue;
                    }
                    let (dx, dy) = (q0[0] - p0[0], q0[1] - p0[1]);
                    let (u, z) = ((dx * s[1] - dy * s[0]) / den, (dx * r[1] - dy * r[0]) / den);
                    // the sample stretches' ends (shared points of curves that
                    // hand over) are found as ends, not crossings
                    if !(u > 1e-9 && u < 1.0 - 1e-9 && z > 1e-9 && z < 1.0 - 1e-9) {
                        continue;
                    }
                    let (sa, sb) = (lines[i][w].0 + u * (lines[i][w + 1].0 - lines[i][w].0), lines[j][v].0 + z * (lines[j][v + 1].0 - lines[j][v].0));
                    if let Some((sa, sb)) = newton(region, candidates[i], candidates[j], sa, sb, size) {
                        cuts[i].push(sa);
                        cuts[j].push(sb);
                    }
                }
            }
        }
    }
    // The stretches between cuts, kept where membership differs across them.
    let eps = 1e-7 * size;
    let mut kept: Vec<(Piece, P2, P2)> = Vec::new();
    for (i, piece) in candidates.iter().enumerate() {
        let mut c = cuts[i].clone();
        c.sort_by(|x, y| x.total_cmp(y));
        c.dedup_by(|x, y| (*x - *y).abs() <= 1e-12 * (piece.b - piece.a).abs().max(1e-300));
        for w in c.windows(2) {
            let (a, b) = (w[0], w[1]);
            if !(b > a) {
                continue;
            }
            let m = 0.5 * (a + b);
            let h = 1e-6 * (b - a);
            let (y, ya, yb) = (region.eval(piece.kind, m), region.eval(piece.kind, m - h), region.eval(piece.kind, m + h));
            let (dx, dy) = (yb[0] - ya[0], yb[1] - ya[1]);
            let l = dx.hypot(dy);
            if !(l > 0.0) {
                continue;
            }
            let left = [-dy / l, dx / l];
            let read = |q: P2| region.member(q).ok_or_else(|| refuse("apex_member", format!("membership at ({:.6}, {:.6}) does not settle", q[0], q[1])));
            let (inside_left, inside_right) = (read([y[0] + left[0] * eps, y[1] + left[1] * eps])?, read([y[0] - left[0] * eps, y[1] - left[1] * eps])?);
            if inside_left == inside_right {
                continue;
            }
            let stretch = if inside_left { Piece { kind: piece.kind, a, b } } else { Piece { kind: piece.kind, a: b, b: a } };
            kept.push((stretch, region.eval(piece.kind, stretch.a), region.eval(piece.kind, stretch.b)));
        }
    }
    // One loop, end to start.
    if kept.is_empty() {
        return Err(refuse("apex_empty", "the section sweeps no region".into()));
    }
    let close = 1e-7 * size;
    let gap = |p: P2, q: P2| (p[0] - q[0]).hypot(p[1] - q[1]);
    let mut used = vec![false; kept.len()];
    let mut chain = vec![0usize];
    used[0] = true;
    loop {
        let end = kept[*chain.last().unwrap()].2;
        if gap(end, kept[chain[0]].1) <= close && chain.len() > 1 {
            break;
        }
        let next = (0..kept.len()).filter(|&j| !used[j]).min_by(|&x, &y| gap(end, kept[x].1).total_cmp(&gap(end, kept[y].1)));
        match next {
            Some(j) if gap(end, kept[j].1) <= close => {
                used[j] = true;
                chain.push(j);
            }
            _ => return Err(refuse("apex_open", format!("the region's boundary does not close (at ({:.6}, {:.6}))", end[0], end[1]))),
        }
    }
    if used.iter().any(|u| !u) {
        return Err(refuse("apex_loops", format!("the region's boundary is more than one loop ({} stretches left)", used.iter().filter(|u| !**u).count())));
    }
    Ok(chain.into_iter().map(|j| kept[j].0).collect())
}

/// The edge of a run of time where side k's envelope point lies on the side,
/// between `ta` and `tb` (one on it, one off): where the fraction crosses 0
/// or 1, by bisection (the envelope hands over to that corner's track there).
fn edge_of_run(region: &Region, k: usize, ta: f64, tb: f64) -> f64 {
    let (fa, fb) = (region.foot(k, ta), region.foot(k, tb));
    let bound = if fa.is_finite() && fb.is_finite() && (fa < 0.0 || fb < 0.0) && !(fa > 1.0 || fb > 1.0) { 0.0 } else { 1.0 };
    bisect(|t| region.foot(k, t) - bound, ta, tb)
}

/// Two candidates' crossing near (sa, sb), by Newton's method; None where it
/// does not converge within both stretches.
fn newton(region: &Region, p: Piece, q: Piece, mut sa: f64, mut sb: f64, size: f64) -> Option<(f64, f64)> {
    let (lo_a, hi_a) = (p.a.min(p.b), p.a.max(p.b));
    let (lo_b, hi_b) = (q.a.min(q.b), q.a.max(q.b));
    for _ in 0..60 {
        let (x, y) = (region.eval(p.kind, sa), region.eval(q.kind, sb));
        let r = [x[0] - y[0], x[1] - y[1]];
        if r[0].hypot(r[1]) <= 1e-14 * size {
            return (sa >= lo_a && sa <= hi_a && sb >= lo_b && sb <= hi_b).then_some((sa, sb));
        }
        let (ha, hb) = (1e-7 * (hi_a - lo_a), 1e-7 * (hi_b - lo_b));
        let (xa, yb) = (region.eval(p.kind, sa + ha), region.eval(q.kind, sb + hb));
        let j = [[(xa[0] - x[0]) / ha, -(yb[0] - y[0]) / hb], [(xa[1] - x[1]) / ha, -(yb[1] - y[1]) / hb]];
        let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
        if !(det.is_finite() && det != 0.0) {
            return None;
        }
        sa -= (j[1][1] * r[0] - j[0][1] * r[1]) / det;
        sb -= (-j[1][0] * r[0] + j[0][0] * r[1]) / det;
        if !(sa.is_finite() && sb.is_finite()) {
            return None;
        }
    }
    None
}

/// What `tool` sweeps under `motion` (turns about axes through its apex),
/// within the distances `within` = [r0, r1] from the apex: the cone over the
/// region its section sweeps, capped by planes square to the region's middle
/// direction (inside r0 for its widest direction, and at r1): within [r0, r1]
/// it is the sweep, beyond it the result is not. Refuses a motion with an advance or an axis missing the apex,
/// a section that is not convex, a range beyond where the sweep is the cone
/// (past the inner end's farthest point and short of the outer end's
/// nearest), and a region whose boundary is not one loop.
pub fn sweep_pyramid_within(tool: &Pyramid, motion: &SolidMotion, within: [f64; 2]) -> Result<crate::BrepSolid, KernelRefusal> {
    let axis = tool.axis.normalized().map_err(|e| input("apex_axis", e))?;
    let x = tool.x_axis.sub(axis.scale(tool.x_axis.dot(axis))).normalized().map_err(|e| input("apex_x_axis", e))?;
    let y = axis.cross(x);
    let n = tool.section.len();
    if n < 3 || !(tool.at > 0.0 && tool.from > 0.0 && tool.to > tool.from) {
        return Err(input("apex_tool", "a section of three or more corners, at a positive distance, between positive end distances".into()));
    }
    // convex, counter-clockwise or not
    let turn = |k: usize| {
        let (a, b, c) = (tool.section[k], tool.section[(k + 1) % n], tool.section[(k + 2) % n]);
        (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
    };
    let sense = turn(0).signum();
    if sense == 0.0 || (0..n).any(|k| turn(k).signum() != sense) {
        return Err(input("apex_convex", "the section must be convex".into()));
    }
    let point = |q: P2, at: f64| axis.scale(at).add(x.scale(q[0] * at / tool.at)).add(y.scale(q[1] * at / tool.at));
    let corners: Vec<Vec3> = tool.section.iter().map(|&q| { let p = point(q, tool.at); p.scale(1.0 / p.length()) }).collect();
    // each side's plane through O, its normal pointing out of the pyramid
    let normals: Vec<Vec3> = (0..n)
        .map(|k| {
            let c = corners[k].cross(corners[(k + 1) % n]);
            let c = c.scale(1.0 / c.length());
            if c.dot(corners[(k + 2) % n]) > 0.0 { c.scale(-1.0) } else { c }
        })
        .collect();
    // where the sweep is the cone: past the inner end's farthest point, short of the outer end's nearest
    let inner = tool.section.iter().map(|&q| point(q, tool.from).length()).fold(0.0f64, f64::max);
    let outer = {
        // the outer end's nearest point to O: its foot if the axis meets the section, else on its rim
        let inside = (0..n).all(|k| { let (a, b) = (tool.section[k], tool.section[(k + 1) % n]); ((b[0] - a[0]) * (0.0 - a[1]) - (b[1] - a[1]) * (0.0 - a[0])).signum() == sense });
        if inside {
            tool.to
        } else {
            (0..n).map(|k| {
                let (a, b) = (point(tool.section[k], tool.to), point(tool.section[(k + 1) % n], tool.to));
                let e = b.sub(a);
                let s = (-a.dot(e) / e.dot(e)).clamp(0.0, 1.0);
                a.add(e.scale(s)).length()
            }).fold(f64::INFINITY, f64::min)
        }
    };
    let [r0, r1] = within;
    if !(r0.is_finite() && r1.is_finite() && r1 > r0) {
        return Err(input("apex_within", "the distances must be finite, the first below the second".into()));
    }
    if !(r0 >= inner && r1 <= outer) {
        return Err(refuse("apex_within", format!("the sweep is the cone only between {inner} and {outer} from the apex, not across [{r0}, {r1}]")));
    }
    // the motion: turns about axes through the apex, no advance
    let mut axes = Vec::with_capacity(motion.steps.len());
    for step in &motion.steps {
        let a = step.axis_direction.normalized().map_err(|e| input("apex_step", e))?;
        let off = step.axis_origin.sub(tool.apex);
        let miss = off.sub(a.scale(off.dot(a))).length();
        // the tool's own size sets the tolerance (cross review: a far
        // origin along the axis must not hide a miss)
        if step.advance != 0.0 || miss > 1e-12 * tool.to {
            return Err(refuse("apex_motion", "every step must be a turn about an axis through the apex".into()));
        }
        axes.push((a, step.turn));
    }
    if !(motion.to > motion.from) || axes.iter().all(|(_, w)| *w == 0.0) {
        return Err(input("apex_motion", "the motion must turn over a positive range".into()));
    }
    let turns = Turns { axes, t0: motion.from, t1: motion.to };
    // the chart: square to the tool's axis at the middle time
    let m = turns.apply(0.5 * (turns.t0 + turns.t1), axis);
    let e1 = m.perpendicular().map_err(|e| input("apex_chart", e))?;
    let e2 = m.cross(e1);
    // every corner stays within 80° of the middle direction
    for i in 0..=64 {
        let t = turns.t0 + (turns.t1 - turns.t0) * i as f64 / 64.0;
        if corners.iter().any(|c| turns.apply(t, *c).dot(m) < 0.17) {
            return Err(refuse("apex_spread", "the tool turns more than 80° from its middle direction".into()));
        }
    }
    let region = Region { turns, corners, normals, m, e1, e2 };
    let pieces = boundary(&region)?;
    // the planes square to m: the inner just inside r0 (every direction's
    // point there no farther than r0), the outer at r1 (no farther than the
    // cone is exact)
    let mut widest = 1.0f64;
    let mut samples: Vec<Vec<(f64, P2)>> = Vec::with_capacity(pieces.len());
    for piece in &pieces {
        let points: Vec<(f64, P2)> = (0..=96).map(|i| { let s = piece.a + (piece.b - piece.a) * i as f64 / 96.0; (s, region.eval(piece.kind, s)) }).collect();
        for (_, q) in &points {
            widest = widest.max((1.0 + q[0] * q[0] + q[1] * q[1]).sqrt());
        }
        samples.push(points);
    }
    // Within [r0, r1] the slab between these planes holds every point of
    // the cone (x·m lies between |x|/widest and |x|), and there the cone is
    // the sweep; beyond the range the result is not the sweep.
    let (near, far) = (r0 / widest * (1.0 - 1e-9), r1);
    let lift = |q: P2, h: f64| tool.apex.add(region.m.scale(h)).add(region.e1.scale(q[0] * h)).add(region.e2.scale(q[1] * h));
    let section = |h: f64| -> Result<Vec<NurbsCurve>, KernelRefusal> {
        samples
            .iter()
            .map(|points| {
                let xs: Vec<Vec3> = points.iter().map(|(_, q)| lift(*q, h)).collect();
                let mut s = vec![0.0];
                for w in xs.windows(2) {
                    s.push(s.last().unwrap() + w[1].sub(w[0]).length());
                }
                let total = *s.last().unwrap();
                let parameters: Vec<f64> = s.iter().map(|v| v / total).collect();
                crate::interpolate_curve(&xs, 3, &parameters).map_err(|e| refuse("apex_fit", e))
            })
            .collect()
    };
    let (inner_section, outer_section) = (section(near)?, section(far)?);
    loft_profile_brep(&[inner_section, outer_section])
}

