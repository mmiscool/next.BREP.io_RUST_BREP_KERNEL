//! A solid of revolution turned about an axis that crosses its own at right
//! angles, swept within a wedge its turn's two ends stay clear of: the sweep
//! there is the tool's (ρ, z) shadow about the turn's axis, turned over the
//! wedge, and that shadow has an exact form.
//!
//! Put the crossing point O at the origin, the tool's axis along `A`, its
//! radius `r`. A tool point (A, r cos θ, r sin θ) turned about the axis square
//! to the tool's reads ρ² = A² + r² cos² θ, z = r sin θ: as θ runs, it covers
//! the arc of the circle of radius R = √(A² + r²) about O from (|A|, ±r) to
//! (R, 0). So the shadow is the meridian's ANGULAR CLOSURE about O: a point at
//! distance R and angle φ from the ρ axis is covered iff some meridian point
//! at that distance stands at least |φ| off the tool's axis. Its upper edge,
//! for each R, is the meridian point of largest angle at that distance, which
//! runs along the meridian's own lines and arcs (ρ = A, z = r) between the
//! distances where the largest changes piece, with an arc of constant R
//! wherever it jumps, and at the inner and outer distances; the lower edge is
//! its mirror in z. Lines and arcs of circles, all exact.
use crate::solid_sweep::{OutlinePiece, SolidMotion, SweepClip, SweptTool};
use crate::{make_arc, make_line, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};
use std::f64::consts::{PI, TAU};

/// `BREP_SWEEP_REVOLVED_TURN_SHADOW` (default on; `=0` restores the sweep's own routes).
pub(crate) fn revolved_turn_shadow_on() -> bool {
    crate::switch!("BREP_SWEEP_REVOLVED_TURN_SHADOW", true)
}

/// `BREP_SWEEP_REVOLVED_TURN_SHADOW_CHECK` (default off): the shadow built and
/// its volume read against the sweep's own route's, the latter kept
/// ([`revolved_turn_shadow_check_take`]).
pub(crate) fn revolved_turn_shadow_check_on() -> bool {
    crate::switch!("BREP_SWEEP_REVOLVED_TURN_SHADOW_CHECK", false)
}

thread_local! {
    static CHECK: std::cell::Cell<(u64, u64, f64)> = const { std::cell::Cell::new((0, 0, 0.0)) };
}

/// (compared, differing past 1e-6 relative, worst relative difference) on
/// this thread since the last call.
pub fn revolved_turn_shadow_check_take() -> (u64, u64, f64) {
    CHECK.with(|cell| cell.replace((0, 0, 0.0)))
}

pub(crate) fn record_check(shadow: &BrepSolid, other: &[BrepSolid], clip_volume: impl Fn(&BrepSolid) -> Option<f64>) {
    let a = clip_volume(shadow);
    let b: Option<f64> = other.iter().map(&clip_volume).sum();
    let worst = match (a, b) {
        (Some(a), Some(b)) if b.abs() > 0.0 => ((a - b) / b).abs(),
        _ => f64::INFINITY,
    };
    CHECK.with(|cell| {
        let (n, bad, w) = cell.get();
        cell.set((n + 1, bad + u64::from(!(worst <= 1e-6)), w.max(worst)));
    });
}

/// One meridian piece in the (A, r) plane: a segment, or an arc about `c` of
/// radius `rho` over angles `a0..a1` (any order), as `point(t)` for t in 0..1.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line([f64; 2], [f64; 2]),
    Arc { c: [f64; 2], rho: f64, a0: f64, a1: f64 },
}

impl Piece {
    fn point(&self, t: f64) -> [f64; 2] {
        match *self {
            Piece::Line(p, q) => [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t],
            Piece::Arc { c, rho, a0, a1 } => {
                let a = a0 + (a1 - a0) * t;
                [c[0] + rho * a.cos(), c[1] + rho * a.sin()]
            }
        }
    }
    fn distance(&self, t: f64) -> f64 {
        let p = self.point(t);
        p[0].hypot(p[1])
    }
    /// Parameters where the distance from O is extremal inside (0, 1).
    fn extrema(&self) -> Vec<f64> {
        match *self {
            Piece::Line(p, q) => {
                let d = [q[0] - p[0], q[1] - p[1]];
                let dd = d[0] * d[0] + d[1] * d[1];
                if dd <= 0.0 {
                    return Vec::new();
                }
                let t = -(p[0] * d[0] + p[1] * d[1]) / dd;
                if t > 1e-12 && t < 1.0 - 1e-12 { vec![t] } else { Vec::new() }
            }
            Piece::Arc { c, a0, a1, .. } => {
                // Every angle of the arc's own range (any period) where its
                // radius points along the line through O and its centre.
                let toward = c[1].atan2(c[0]);
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let mut out = Vec::new();
                if !(hi > lo) || !lo.is_finite() || !hi.is_finite() {
                    return out;
                }
                for base in [toward, toward + PI] {
                    let first = ((lo - base) / TAU).ceil() as i64;
                    let last = ((hi - base) / TAU).floor() as i64;
                    for k in first..=last {
                        let a = base + k as f64 * TAU;
                        let t = (a - a0) / (a1 - a0);
                        if t > 1e-12 && t < 1.0 - 1e-12 {
                            out.push(t);
                        }
                    }
                }
                out.sort_by(f64::total_cmp);
                out
            }
        }
    }
    /// The piece over `t0..t1`.
    fn sub(&self, t0: f64, t1: f64) -> Piece {
        match *self {
            Piece::Line(_, _) => Piece::Line(self.point(t0), self.point(t1)),
            Piece::Arc { c, rho, a0, a1 } => Piece::Arc { c, rho, a0: a0 + (a1 - a0) * t0, a1: a0 + (a1 - a0) * t1 },
        }
    }
}

/// A run of a piece on which the distance from O is monotone, from `t0` to `t1`.
#[derive(Clone, Copy, Debug)]
struct Run {
    piece: Piece,
    t0: f64,
    t1: f64,
}

impl Run {
    fn r_range(&self) -> (f64, f64) {
        let (a, b) = (self.piece.distance(self.t0), self.piece.distance(self.t1));
        (a.min(b), a.max(b))
    }
    /// The parameter where the run is `radius` from O (monotone: bisection).
    fn at(&self, radius: f64) -> f64 {
        // At one of its own ends: that end, exactly (the envelope's corners
        // and its meeting with the axis are then the meridian's own points).
        let band = 1e-14 * radius.abs().max(1.0);
        if (self.piece.distance(self.t0) - radius).abs() <= band {
            return self.t0;
        }
        if (self.piece.distance(self.t1) - radius).abs() <= band {
            return self.t1;
        }
        let (mut lo, mut hi) = (self.t0, self.t1);
        let rising = self.piece.distance(hi) >= self.piece.distance(lo);
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if (self.piece.distance(mid) < radius) == rising { lo = mid } else { hi = mid }
        }
        0.5 * (lo + hi)
    }
    fn angle_at(&self, radius: f64) -> f64 {
        let p = self.piece.point(self.at(radius));
        p[1].atan2(p[0])
    }
}

/// A piece of the shadow's upper edge in the (ρ, z) plane: a meridian piece
/// itself, or an arc of constant distance `radius` about O between angles.
#[derive(Clone, Copy, Debug)]
enum Edge {
    Along(Piece),
    Around { radius: f64, from: f64, to: f64 },
}

fn turn_of(motion: &SolidMotion) -> Option<(Vec3, Vec3, f64, f64)> {
    let [step] = motion.steps.as_slice() else { return None };
    if step.advance != 0.0 || step.turn == 0.0 {
        return None;
    }
    let axis = step.axis_direction.normalized().ok()?;
    Some((step.axis_origin, axis, step.turn * motion.from, step.turn * motion.to))
}

/// The sweep of a revolved `tool` turned by `motion` within `clip`'s wedge,
/// as its turned shadow (see the module), or `None` where the tool, motion or
/// clip is not of that kind.
pub(crate) fn revolved_turn_shadow(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<BrepSolid>, KernelRefusal> {
    let SweptTool::Revolved { axis_origin, axis_direction, radial: _, meridian } = tool else { return Ok(None) };
    let Some(wedge) = clip.angle else { return Ok(None) };
    let Some((turn_origin, turn_axis, angle0, angle1)) = turn_of(motion) else { return Ok(None) };
    let (Ok(ez), Ok(d)) = (clip.axis_direction.normalized(), axis_direction.normalized()) else { return Ok(None) };
    let scale = (clip.radius[1] + axis_origin.sub(clip.axis_origin).length()).max(1.0);
    // The turn about the clip's own axis.
    if turn_axis.cross(ez).length() > 1e-12 || turn_origin.sub(clip.axis_origin).cross(ez).length() > 1e-9 * scale {
        return Ok(None);
    }
    // The tool's axis square to the turn's, and crossing it.
    if d.dot(ez).abs() > 1e-12 {
        return Ok(None);
    }
    let to_axis = axis_origin.sub(clip.axis_origin);
    let lateral = to_axis.sub(ez.scale(to_axis.dot(ez)));
    if lateral.cross(d).length() > 1e-9 * scale {
        return Ok(None);
    }
    // O: where the tool's axis meets the turn's; A measured from it along d.
    let height = to_axis.dot(ez);
    let origin_o = clip.axis_origin.add(ez.scale(height));
    let offset = axis_origin.sub(origin_o).dot(d);
    // The meridian in (A, r): its points (radial, axial) at A = offset + axial.
    let mut pieces = Vec::with_capacity(meridian.len());
    for piece in meridian {
        pieces.push(match *piece {
            OutlinePiece::Line { from, to } => Piece::Line([offset + from[1], from[0]], [offset + to[1], to[0]]),
            OutlinePiece::Arc { centre, radius, start, end } => {
                // (radial, axial) = c + ρ(cos a, sin a) → (A, r) = (offset + c_y + ρ sin a, c_x + ρ cos a)
                // = (offset + c_y, c_x) + ρ (cos b, sin b) with b = π/2 − a.
                Piece::Arc { c: [offset + centre[1], centre[0]], rho: radius, a0: 0.5 * PI - start, a1: 0.5 * PI - end }
            }
            _ => return Ok(None),
        });
    }
    if pieces.is_empty() {
        return Ok(None);
    }
    // The tool on one side of O along its axis (fold A to |A| by the side).
    let samples: Vec<[f64; 2]> = pieces.iter().flat_map(|p| (0..=16).map(move |k| p.point(k as f64 / 16.0))).collect();
    let (a_min, a_max) = samples.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[0]), hi.max(p[0])));
    let r_max = samples.iter().map(|p| p[1]).fold(0.0f64, f64::max);
    let side = if a_min > 0.0 { 1.0 } else if a_max < 0.0 { -1.0 } else { return Ok(None) };
    if side < 0.0 {
        for piece in &mut pieces {
            *piece = match *piece {
                Piece::Line(p, q) => Piece::Line([-p[0], p[1]], [-q[0], q[1]]),
                Piece::Arc { c, rho, a0, a1 } => Piece::Arc { c: [-c[0], c[1]], rho, a0: PI - a0, a1: PI - a1 },
            };
        }
    }
    let near_axis = if side > 0.0 { a_min } else { -a_max };
    // Off the turn's axis by more than its own reach across: the tool's
    // angular half-width about the turn axis is at most asin(r_max / A_min).
    if !(near_axis > r_max * (1.0 + 1e-9)) {
        return Ok(None);
    }
    let half_width = (r_max / near_axis).asin();
    // The turn's ends clear of the wedge (as `turn_shadow` reads them).
    let ex = clip.reference.sub(ez.scale(clip.reference.dot(ez))).normalized().map_err(|_| KernelRefusal::input(KernelStage::Collect, "clip_reference", "the clip's reference lies along its axis"))?;
    let ey = ez.cross(ex);
    let direction = d.scale(side);
    let axis_angle = direction.dot(ey).atan2(direction.dot(ex));
    let rate_sign = turn_axis.dot(ez).signum();
    let (start, end) = (axis_angle + rate_sign * angle0, axis_angle + rate_sign * angle1);
    let (first, last) = if end >= start { (start, end) } else { (end, start) };
    // The wedge placed in the turn's own (unwrapped) interval: its first copy
    // starting past the tool's start pose lies wholly inside the turn, the
    // tool's end pose past it, and the copy before wholly before the start
    // pose, so the whole tool passes every angle of the wedge and no pose
    // stands in it.
    let span = wedge[1] - wedge[0];
    if !(span > 0.0 && span < TAU) || !((last - first) < TAU) {
        return Ok(None);
    }
    let w0 = wedge[0] + TAU * ((first + half_width - wedge[0]) / TAU).ceil();
    let clear = w0 > first + half_width + 1e-9 && w0 + span < last - half_width - 1e-9 && w0 - TAU + span < first - half_width - 1e-9;
    if !clear {
        return Ok(None);
    }
    // Monotone runs of every piece.
    let mut runs = Vec::new();
    for piece in &pieces {
        let mut cuts = vec![0.0];
        cuts.extend(piece.extrema());
        cuts.push(1.0);
        for w in cuts.windows(2) {
            if w[1] - w[0] > 1e-12 {
                runs.push(Run { piece: *piece, t0: w[0], t1: w[1] });
            }
        }
    }
    let mut breaks: Vec<f64> = runs.iter().flat_map(|run| {
        let (a, b) = run.r_range();
        [a, b]
    }).collect();
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * scale);
    let (Some(&r_lo), Some(&r_hi)) = (breaks.first(), breaks.last()) else { return Ok(None) };
    if !(r_lo.is_finite() && r_hi.is_finite()) {
        return Ok(None);
    }
    if !(r_hi > r_lo) {
        return Ok(None);
    }
    // The upper edge interval by interval: the run of largest angle.
    let mut edges: Vec<Edge> = Vec::new();
    let mut last_angle: Option<f64> = None;
    for w in breaks.windows(2) {
        let (lo, hi) = (w[0], w[1]);
        let mid = 0.5 * (lo + hi);
        let best = runs
            .iter()
            .filter(|run| {
                let (a, b) = run.r_range();
                a <= lo + 1e-12 * scale && b >= hi - 1e-12 * scale
            })
            .map(|run| (run.angle_at(mid), run))
            .max_by(|x, y| x.0.total_cmp(&y.0));
        let Some((_, run)) = best else { return Ok(None) };
        let (t_lo, t_hi) = (run.at(lo), run.at(hi));
        let enter = run.angle_at(lo);
        if let Some(previous) = last_angle {
            if (enter - previous).abs() > 1e-12 {
                edges.push(Edge::Around { radius: lo, from: previous, to: enter });
            }
        }
        edges.push(Edge::Along(run.piece.sub(t_lo, t_hi)));
        last_angle = Some(run.angle_at(hi));
    }
    let top_end = last_angle.unwrap_or(0.0);
    let top_start = match edges.first() {
        Some(Edge::Along(piece)) => {
            let p = piece.point(0.0);
            p[1].atan2(p[0])
        }
        _ => 0.0,
    };
    // The closed shadow in (ρ, z): upper edge out, the outer arc down through
    // φ = 0, the lower edge (mirrored) back, the inner arc up.
    let world = |rho: f64, z: f64, ex_w: Vec3| origin_o.add(ez.scale(z)).add(ex_w.scale(rho));
    let (c0, s0) = (wedge[0].cos(), wedge[0].sin());
    let ex_w = ex.scale(c0).add(ey.scale(s0));
    let line = |p: [f64; 2], q: [f64; 2]| make_line(world(p[0], p[1], ex_w), world(q[0], q[1], ex_w)).or_refuse(KernelStage::Fragment, "make_line");
    // An arc in quarter turns at most (one rational segment cannot turn a
    // half), each run the way it goes: make_arc runs counter-clockwise, so a
    // clockwise piece is built the other way and reversed.
    let arc_about = |c: [f64; 2], rho: f64, from: f64, to: f64| -> Result<Vec<NurbsCurve>, KernelRefusal> {
        let pieces = ((to - from).abs() / (0.5 * PI)).ceil().max(1.0) as usize;
        let mut out = Vec::with_capacity(pieces);
        for k in 0..pieces {
            let (a, b) = (from + (to - from) * k as f64 / pieces as f64, from + (to - from) * (k + 1) as f64 / pieces as f64);
            // Angles brought into one period (make_arc reads them as given).
            let shift = TAU * (a.min(b) / TAU).floor();
            let (a, b) = (a - shift, b - shift);
            let curve = if b >= a {
                make_arc(world(c[0], c[1], ex_w), ex_w, ez, rho, a, b).or_refuse(KernelStage::Fragment, "make_arc")?
            } else {
                make_arc(world(c[0], c[1], ex_w), ex_w, ez, rho, b, a).or_refuse(KernelStage::Fragment, "make_arc")?.reversed().or_refuse(KernelStage::Fragment, "reversed")?
            };
            out.push(curve);
        }
        Ok(out)
    };
    let curve_of = |edge: &Edge, mirror: bool| -> Result<Vec<NurbsCurve>, KernelRefusal> {
        let m = |p: [f64; 2]| if mirror { [p[0], -p[1]] } else { p };
        match *edge {
            Edge::Along(Piece::Line(p, q)) => Ok(vec![line(m(p), m(q))?]),
            Edge::Along(Piece::Arc { c, rho, a0, a1 }) => {
                if mirror { arc_about(m(c), rho, -a0, -a1) } else { arc_about(c, rho, a0, a1) }
            }
            Edge::Around { radius, from, to } => {
                if mirror { arc_about([0.0, 0.0], radius, -from, -to) } else { arc_about([0.0, 0.0], radius, from, to) }
            }
        }
    };
    let mut curves = Vec::new();
    for edge in &edges {
        curves.extend(curve_of(edge, false)?);
    }
    if top_end > 1e-12 {
        curves.extend(arc_about([0.0, 0.0], r_hi, top_end, -top_end)?);
    }
    for edge in edges.iter().rev() {
        for curve in curve_of(edge, true)?.into_iter().rev() {
            curves.push(curve.reversed().or_refuse(KernelStage::Fragment, "reversed")?);
        }
    }
    if top_start > 1e-12 {
        curves.extend(arc_about([0.0, 0.0], r_lo, -top_start, top_start)?);
    }
    let body = crate::revolve_profile_brep(&curves, origin_o, ez, wedge[1] - wedge[0]).or_refuse(KernelStage::Fragment, "revolve_profile_brep")?;
    Ok(Some(body))
}
