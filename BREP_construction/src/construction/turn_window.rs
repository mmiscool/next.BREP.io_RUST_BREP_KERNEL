//! A prism standing along an axis, turned partway about it, asked for within an angular window
//! about that axis that one end of the turn keeps clear of: what it passes over there, exactly,
//! without the general region engine (`BREP_SWEEP_TURN_WINDOW`, default on; `=0` restores the engine).
//!
//! Every circle about the axis meets the prism's section (in the plane square to the axis) in an
//! arc, or in nothing. Where each such arc is ONE arc and all of them hold one angle α, the turn
//! carries that circle's arc from [lo + θ₀, hi + θ₀] to [lo + θ₁, hi + θ₁] and passes over every
//! angle between: the whole interval [lo + θ₀, hi + θ₁] (a turn of under half a circle, the section
//! within a half plane, so no wrap). When the start pose lies wholly before the window (every
//! hi + θ₀ below its first angle), inside the window that is everything from the window's start up
//! to hi + θ₁: the band of circles the section reaches (its distances from the axis, along the
//! prism's height) turned from before the window to θ₁ + α, and the prism where it stops, which
//! holds [α + θ₁, hi + θ₁] on each circle. When the end pose lies wholly past the window, the mirror:
//! the prism where it starts, and the band from θ₀ + α to past the window. Both parts lie within
//! the sweep, and together they hold all of it inside the window: the caller cuts by every part.
//!
//! The one-arc test is exact: the number of points where a circle meets the outline, and whether
//! the ray at α lies inside at that distance, change only at the critical distances (the outline's
//! corners, its segments' feet, its arcs' nearest and farthest points, the ray's crossings of the
//! outline); one circle between each two of them settles every circle there.
use crate::solid_sweep::{outline_signed_distance, OutlinePiece, SolidMotion, SweepClip, SweptTool};
use crate::{extrude_profile_brep, make_arc, make_line, revolve_profile_brep, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};
use std::f64::consts::{PI, TAU};

/// `BREP_SWEEP_TURN_WINDOW` (default on since its promotion; `=0` restores the engine): the
/// turn-window route of `sweep_solid_within`.
pub(crate) fn turn_window_on() -> bool {
    crate::switch!("BREP_SWEEP_TURN_WINDOW", true)
}

type P = [f64; 2];

fn wrap(a: f64, near: f64) -> f64 {
    near + (a - near + PI).rem_euclid(TAU) - PI
}

/// The section's pieces in the clip's polar plane: lines, and circular arcs as (centre, radius,
/// start, end) with `end < start` running clockwise.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line(P, P),
    Arc(P, f64, f64, f64),
}

impl Piece {
    fn ends(&self) -> [P; 2] {
        match *self {
            Piece::Line(a, b) => [a, b],
            Piece::Arc(c, r, s, e) => [[c[0] + r * s.cos(), c[1] + r * s.sin()], [c[0] + r * e.cos(), c[1] + r * e.sin()]],
        }
    }
    /// Whether the arc runs through the direction `angle` (about its own centre).
    fn arc_holds(s: f64, e: f64, angle: f64) -> bool {
        let (lo, span) = if e >= s { (s, e - s) } else { (e, s - e) };
        span >= TAU - 1e-12 || (angle - lo).rem_euclid(TAU) <= span
    }
    /// The nearest and farthest distances from the origin, and the critical distances on it.
    fn distances(&self) -> (f64, f64, Vec<f64>) {
        let [a, b] = self.ends();
        let (na, nb) = (a[0].hypot(a[1]), b[0].hypot(b[1]));
        let mut critical = vec![na, nb];
        let (mut lo, mut hi) = (na.min(nb), na.max(nb));
        match *self {
            Piece::Line(a, b) => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let l2 = d[0] * d[0] + d[1] * d[1];
                let t = -(a[0] * d[0] + a[1] * d[1]) / l2;
                if t > 0.0 && t < 1.0 {
                    let foot = (a[0] + d[0] * t).hypot(a[1] + d[1] * t);
                    critical.push(foot);
                    lo = lo.min(foot);
                }
            }
            Piece::Arc(c, r, s, e) => {
                let d = c[0].hypot(c[1]);
                let toward = c[1].atan2(c[0]);
                if Self::arc_holds(s, e, toward + PI) {
                    let near = (d - r).abs();
                    critical.push(near);
                    lo = lo.min(near);
                }
                if Self::arc_holds(s, e, toward) {
                    critical.push(d + r);
                    hi = hi.max(d + r);
                }
            }
        }
        (lo, hi, critical)
    }
    /// The angles (about the origin) where the circle of `rho` meets the piece.
    fn meets(&self, rho: f64) -> Vec<f64> {
        let mut out = Vec::new();
        match *self {
            Piece::Line(a, b) => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let (qa, qb, qc) = (d[0] * d[0] + d[1] * d[1], 2.0 * (a[0] * d[0] + a[1] * d[1]), a[0] * a[0] + a[1] * a[1] - rho * rho);
                let disc = qb * qb - 4.0 * qa * qc;
                if disc > 0.0 {
                    for t in [(-qb - disc.sqrt()) / (2.0 * qa), (-qb + disc.sqrt()) / (2.0 * qa)] {
                        if (0.0..1.0).contains(&t) {
                            out.push((a[1] + d[1] * t).atan2(a[0] + d[0] * t));
                        }
                    }
                }
            }
            Piece::Arc(c, r, s, e) => {
                let d = c[0].hypot(c[1]);
                if d == 0.0 || d > rho + r || d < (rho - r).abs() {
                    return out;
                }
                let along = (rho * rho + d * d - r * r) / (2.0 * d);
                let h = (rho * rho - along * along).max(0.0).sqrt();
                let u = [c[0] / d, c[1] / d];
                for sign in [1.0, -1.0] {
                    let q = [u[0] * along - sign * u[1] * h, u[1] * along + sign * u[0] * h];
                    let about = (q[1] - c[1]).atan2(q[0] - c[0]);
                    // half-open along the piece, so a shared corner counts once
                    let start_point = (q[0] - (c[0] + r * e.cos())).hypot(q[1] - (c[1] + r * e.sin())) < 1e-12 * (1.0 + rho);
                    if Self::arc_holds(s, e, about) && !start_point {
                        out.push(q[1].atan2(q[0]));
                    }
                }
            }
        }
        out
    }
    /// Distances from the origin where the ray at `angle` crosses the piece.
    fn ray_crossings(&self, angle: f64) -> Vec<f64> {
        let u = [angle.cos(), angle.sin()];
        let mut out = Vec::new();
        match *self {
            Piece::Line(a, b) => {
                let d = [b[0] - a[0], b[1] - a[1]];
                let det = u[0] * (-d[1]) - u[1] * (-d[0]);
                if det.abs() > 1e-300 {
                    let s = (a[0] * (-d[1]) - a[1] * (-d[0])) / det;
                    let t = (u[0] * a[1] - u[1] * a[0]) / det;
                    if s > 0.0 && (0.0..=1.0).contains(&t) {
                        out.push(s);
                    }
                }
            }
            Piece::Arc(c, r, s, e) => {
                let b = -(u[0] * c[0] + u[1] * c[1]);
                let cc = c[0] * c[0] + c[1] * c[1] - r * r;
                let disc = b * b - cc;
                if disc >= 0.0 {
                    for t in [-b - disc.sqrt(), -b + disc.sqrt()] {
                        let q = [u[0] * t, u[1] * t];
                        if t > 0.0 && Self::arc_holds(s, e, (q[1] - c[1]).atan2(q[0] - c[0])) {
                            out.push(t);
                        }
                    }
                }
            }
        }
        out
    }
    /// The least and greatest polar angle on the piece, about `near` (no wrap: the section lies in
    /// a half plane about `near`).
    fn angle_range(&self, near: f64) -> (f64, f64) {
        let [a, b] = self.ends();
        let mut angles = vec![wrap(a[1].atan2(a[0]), near), wrap(b[1].atan2(b[0]), near)];
        if let Piece::Arc(c, r, s, e) = *self {
            let d = c[0].hypot(c[1]);
            if d > r {
                let base = c[1].atan2(c[0]);
                let off = (r / d).asin();
                for (polar, about) in [(base + off, base + off + PI / 2.0), (base - off, base - off - PI / 2.0)] {
                    if Self::arc_holds(s, e, about) {
                        angles.push(wrap(polar, near));
                    }
                }
            }
        }
        (angles.iter().cloned().fold(f64::INFINITY, f64::min), angles.iter().cloned().fold(f64::NEG_INFINITY, f64::max))
    }
}

/// The turn-window sweep of `tool` along `motion` inside `clip`: parts whose union is the sweep
/// there (and which lie within the sweep everywhere), or None where the tool, motion or window is
/// not of that kind (the caller's general engine then runs).
pub(crate) fn turn_window(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let SweptTool::Prism { origin, normal, x_axis, outline, from, to } = tool else { return Ok(None) };
    let Some(window) = clip.angle else { return Ok(None) };
    let [step] = motion.steps.as_slice() else { return Ok(None) };
    let ez = clip.axis_direction.normalized().or_refuse(KernelStage::Collect, "clip_axis")?;
    let e1 = clip.reference.sub(ez.scale(clip.reference.dot(ez))).normalized().or_refuse(KernelStage::Collect, "clip_reference")?;
    let e2 = ez.cross(e1);
    let Ok(n) = normal.normalized() else { return Ok(None) };
    let Ok(axis) = step.axis_direction.normalized() else { return Ok(None) };
    // One turn, about the clip's own axis, the prism standing along it.
    let off_axis = step.axis_origin.sub(clip.axis_origin);
    if step.advance != 0.0 || step.turn == 0.0 || axis.cross(ez).length() > 1e-12 || n.cross(ez).length() > 1e-12 {
        return Ok(None);
    }
    if !(to > from) {
        return Ok(None); // a reversed or empty extent: the engine refuses it by name
    }
    // A clip the engine would refuse (reversed or empty ranges) is the engine's to refuse.
    let good = |r: [f64; 2]| r[0].is_finite() && r[1].is_finite() && r[1] > r[0];
    if !(good(clip.radius) && clip.radius[0] >= 0.0 && good(clip.axial) && good(window)) {
        return Ok(None);
    }
    if outline.iter().any(|p| matches!(p, OutlinePiece::Ellipse { .. })) || !(motion.to > motion.from) {
        return Ok(None);
    }
    // The section in the clip's plane, polar about the axis.
    let Ok(xa) = x_axis.sub(n.scale(x_axis.dot(n))).normalized() else { return Ok(None) };
    let ya = n.cross(xa);
    let flat = |q: P| -> P {
        let x = origin.add(xa.scale(q[0])).add(ya.scale(q[1])).sub(clip.axis_origin);
        [x.dot(e1), x.dot(e2)]
    };
    let xa_angle = { let f = [xa.dot(e1), xa.dot(e2)]; f[1].atan2(f[0]) };
    // (xa, ya) turns the clip's way where the prism points along +ez, else it is mirrored.
    let mirror = n.dot(ez).signum();
    let pieces: Vec<Piece> = outline
        .iter()
        .map(|p| match *p {
            OutlinePiece::Line { from, to } => Piece::Line(flat(from), flat(to)),
            OutlinePiece::Arc { centre, radius, start, end } => Piece::Arc(flat(centre), radius, xa_angle + mirror * start, xa_angle + mirror * end),
            OutlinePiece::Ellipse { .. } => unreachable!(),
        })
        .collect();
    let flat_outline: Vec<OutlinePiece> = pieces
        .iter()
        .map(|p| match *p {
            Piece::Line(a, b) => OutlinePiece::Line { from: a, to: b },
            Piece::Arc(c, r, s, e) => OutlinePiece::Arc { centre: c, radius: r, start: s, end: e },
        })
        .collect();
    let size = pieces.iter().map(|p| p.distances().1).fold(0.0f64, f64::max).max(1e-300);
    // The prism's own direction along the axis over its whole extent: a tilt, however slight,
    // carries the section sideways by the tilt times how far it runs.
    if n.cross(ez).length() * from.abs().max(to.abs()).max(origin.sub(clip.axis_origin).length()) > 1e-12 * size {
        return Ok(None);
    }
    // The turn's axis the clip's where the prism stands: its points at the prism's two end heights
    // within a hair of the clip's axis, to the section's own size (not to how far along it lies).
    {
        let base = origin.sub(clip.axis_origin).dot(ez);
        let along = axis.dot(ez);
        for h in [base + from * n.dot(ez), base + to * n.dot(ez)] {
            let q = off_axis.add(axis.scale((h - off_axis.dot(ez)) / along));
            if q.sub(ez.scale(q.dot(ez))).length() > 1e-12 * size {
                return Ok(None);
            }
        }
    }
    if outline_signed_distance(&flat_outline, [0.0, 0.0]) <= 1e-12 * size {
        return Ok(None); // the axis runs through (or along) the section
    }
    let (rho_min, rho_max) = pieces.iter().map(|p| p.distances()).fold((f64::INFINITY, 0.0f64), |(lo, hi), (l, h, _)| (lo.min(l), hi.max(h)));
    // α: the direction of the section's farthest point, where the circle of rho_max touches it.
    let mut alpha = None;
    for p in &pieces {
        for q in p.ends() {
            if (q[0].hypot(q[1]) - rho_max).abs() <= 1e-12 * size {
                alpha = Some(q[1].atan2(q[0]));
            }
        }
        if let Piece::Arc(c, r, s, e) = *p {
            let d = c[0].hypot(c[1]);
            if ((d + r) - rho_max).abs() <= 1e-12 * size && Piece::arc_holds(s, e, c[1].atan2(c[0])) {
                alpha = Some(c[1].atan2(c[0]));
            }
        }
    }
    let Some(alpha) = alpha else { return Ok(None) };
    // The section within the half plane facing α (no wrap of angles about the axis).
    let (lo_angle, hi_angle) = pieces.iter().map(|p| p.angle_range(alpha)).fold((f64::INFINITY, f64::NEG_INFINITY), |(l, h), (a, b)| (l.min(a), h.max(b)));
    if !(hi_angle - alpha < PI / 2.0 - 1e-9 && alpha - lo_angle < PI / 2.0 - 1e-9) {
        return Ok(None);
    }
    // One arc on every circle, each holding α.
    let mut critical: Vec<f64> = pieces.iter().flat_map(|p| p.distances().2).collect();
    critical.extend(pieces.iter().flat_map(|p| p.ray_crossings(alpha)));
    critical.push(rho_min);
    critical.push(rho_max);
    critical.retain(|&r| r >= rho_min - 1e-12 * size && r <= rho_max + 1e-12 * size);
    critical.sort_by(f64::total_cmp);
    critical.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * size);
    for w in critical.windows(2) {
        if w[1] - w[0] <= 1e-12 * size {
            continue;
        }
        let rho = 0.5 * (w[0] + w[1]);
        let count: usize = pieces.iter().map(|p| p.meets(rho).len()).sum();
        // (α may run along the outline, a flat face ending each arc: the arc still holds it.)
        if count != 2 || outline_signed_distance(&flat_outline, [rho * alpha.cos(), rho * alpha.sin()]) > 1e-12 * size {
            return Ok(None);
        }
    }
    // The turn in the clip's angles, run forward (the angle growing).
    let rate = step.turn * axis.dot(ez).signum();
    let sense = rate.signum();
    let (t0, t1) = (rate * motion.from, rate * motion.to); // the section's turn at each end, true angles
    if (t1 - t0).abs() + (hi_angle - lo_angle) >= TAU - 1e-9 {
        return Ok(None);
    }
    let fwd = |a: f64| sense * a; // forward angles
    let (alpha_f, lo_f, hi_f) = if sense > 0.0 { (alpha, lo_angle, hi_angle) } else { (-alpha, -hi_angle, -lo_angle) };
    let (w0, w1) = if sense > 0.0 { (window[0], window[1]) } else { (-window[1], -window[0]) };
    let (s0, s1) = (fwd(t0), fwd(t1));
    let margin = 1e-9;
    // Angles are periodic: the swept interval [s0 + lo, s1 + hi] turned by a whole turn either way
    // must miss the window, or a pose of the turn reaches it from there.
    if w1 - w0 >= TAU {
        return Ok(None);
    }
    for k in [-2.0f64, -1.0, 1.0, 2.0] {
        let (a, b) = (s0 + lo_f + k * TAU, s1 + hi_f + k * TAU);
        if b >= w0 - margin && a <= w1 + margin {
            return Ok(None);
        }
    }
    // Heights along the axis: the prism's extent.
    let base = origin.sub(clip.axis_origin).dot(ez);
    let (h0, h1) = { let (a, b) = (base + from * n.dot(ez), base + to * n.dot(ez)); (a.min(b), a.max(b)) };
    let band = |a_lo: f64, a_hi: f64| -> Result<BrepSolid, KernelRefusal> {
        // forward angles a_lo..a_hi -> true angles, revolved about ±ez so that it runs forward
        let start = sense * a_lo;
        let dir = [start.cos(), start.sin()];
        let at = |rho: f64, h: f64| clip.axis_origin.add(e1.scale(rho * dir[0])).add(e2.scale(rho * dir[1])).add(ez.scale(h));
        let corners = [at(rho_min, h0), at(rho_max, h0), at(rho_max, h1), at(rho_min, h1)];
        let curves: Vec<NurbsCurve> = (0..4).map(|k| make_line(corners[k], corners[(k + 1) % 4]).or_refuse(KernelStage::Fragment, "turn_window_band")).collect::<Result<_, _>>()?;
        revolve_profile_brep(&curves, clip.axis_origin, ez.scale(sense), a_hi - a_lo).or_refuse(KernelStage::Fragment, "turn_window_band")
    };
    let posed = |turn: f64| -> Result<BrepSolid, KernelRefusal> {
        // the prism with its section turned by `turn` (true angle) about the axis
        let (c, s) = (turn.cos(), turn.sin());
        let r = |q: P| [c * q[0] - s * q[1], s * q[0] + c * q[1]];
        let at = |q: P| clip.axis_origin.add(e1.scale(q[0])).add(e2.scale(q[1])).add(ez.scale(h0));
        let mut curves = Vec::with_capacity(pieces.len());
        for p in &pieces {
            curves.push(match *p {
                Piece::Line(a, b) => make_line(at(r(a)), at(r(b))),
                Piece::Arc(cc, rr, st, en) => {
                    let centre = at(r(cc));
                    if en >= st {
                        make_arc(centre, e1, e2, rr, st + turn, en + turn)
                    } else {
                        make_arc(centre, e1, e2, rr, en + turn, st + turn).and_then(|a| a.reversed())
                    }
                }
            }
            .or_refuse(KernelStage::Fragment, "turn_window_section")?);
        }
        extrude_profile_brep(&curves, ez, h1 - h0)
    };
    if s0 + hi_f < w0 - margin {
        // Start pose wholly before the window.
        let reach = s1 + alpha_f;
        let from_angle = (s0 + hi_f).max(w0 - 0.1).min(w0);
        if reach <= from_angle + 1e-6 {
            return Ok(None);
        }
        // (A part the kernel will not build leaves the whole to the engine, never a refusal.)
        return Ok(match (band(from_angle, reach), posed(t1)) {
            (Ok(b), Ok(p)) => Some(vec![b, p]),
            _ => None,
        });
    }
    if s1 + lo_f > w1 + margin {
        // End pose wholly past the window.
        let reach = s0 + alpha_f;
        let to_angle = (s1 + lo_f).min(w1 + 0.1).max(w1);
        if to_angle <= reach + 1e-6 {
            return Ok(None);
        }
        return Ok(match (posed(t0), band(reach, to_angle)) {
            (Ok(p), Ok(b)) => Some(vec![p, b]),
            _ => None,
        });
    }
    Ok(None)
}

