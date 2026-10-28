//! Curve-following mates (path; slot and cam build on the same atoms): a body
//! point held on a curve of another body.
//!
//! The Jacobian is finite-difference, so every evaluation must be a pure
//! function of the poses: the closest curve parameter is found afresh each
//! time — nearest of a FIXED sample set, then a fixed-count Newton refinement
//! — and nothing is carried between evaluations.

use super::*;
use std::sync::Arc;

/// Uniform parameter samples that seed the closest-point search.
const CURVE_SAMPLES: usize = 256;
/// Newton refinement steps (each one is cheap; converged steps are no-ops).
const CURVE_NEWTON_STEPS: usize = 24;

/// A [`MateCurve`] prepared for evaluation: its fixed samples, its length
/// estimate (the along row's scale), and whether it closes on itself.
#[derive(Debug)]
pub(super) struct SampledCurve {
    curve: crate::NurbsCurve,
    t0: f64,
    t1: f64,
    closed: bool,
    length: f64,
    samples: Vec<(f64, Vec3)>,
}

impl SampledCurve {
    pub(super) fn new(mate: &MateCurve, name: &str) -> Result<Self, String> {
        let (t0, t1) = (mate.t0.min(mate.t1), mate.t0.max(mate.t1));
        if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
            return Err(format!("{name}: curve parameter range [{}, {}] is empty", mate.t0, mate.t1));
        }
        let mut samples = Vec::with_capacity(CURVE_SAMPLES + 1);
        for i in 0..=CURVE_SAMPLES {
            let t = t0 + (t1 - t0) * i as f64 / CURVE_SAMPLES as f64;
            let p = mate.curve.evaluate(t).map_err(|error| format!("{name}: curve: {error}"))?;
            samples.push((t, p));
        }
        let length: f64 = samples.windows(2).map(|w| w[1].1.sub(w[0].1).length()).sum();
        if !(length.is_finite() && length > ASM_DIR_EPS) {
            return Err(format!("{name}: curve has no length"));
        }
        let closed = samples[0].1.sub(samples[CURVE_SAMPLES].1).length() <= 1e-9 * length.max(1.0);
        Ok(Self { curve: mate.curve.clone(), t0, t1, closed, length, samples })
    }

    /// The parameter of the curve point closest to `p` (curve frame), and
    /// that point. Deterministic: nearest fixed sample, then Newton on
    /// `(C(t) − p)·C'(t) = 0`, clamped (or wrapped, when closed) to the range.
    fn closest(&self, p: Vec3) -> (f64, Vec3) {
        let (mut t, mut best) = self.samples[0];
        let mut best_d = best.sub(p).length();
        for &(ts, ps) in &self.samples[1..] {
            let d = ps.sub(p).length();
            if d < best_d {
                (t, best, best_d) = (ts, ps, d);
            }
        }
        let span = self.t1 - self.t0;
        for _ in 0..CURVE_NEWTON_STEPS {
            let Ok(d) = self.curve.derivatives(t, 2) else { break };
            let (c, c1, c2) = (d[0], d[1], d[2]);
            let r = c.sub(p);
            let g = r.dot(c1);
            let h = c1.dot(c1) + r.dot(c2);
            if !(h.is_finite() && h > 0.0) {
                break;
            }
            let mut next = t - g / h;
            next = if self.closed {
                self.t0 + (next - self.t0).rem_euclid(span)
            } else {
                next.clamp(self.t0, self.t1)
            };
            if (next - t).abs() <= 1e-15 * span.max(1.0) {
                t = next;
                break;
            }
            t = next;
        }
        let point = self.curve.evaluate(t).unwrap_or(best);
        (t, point)
    }

    /// The curve's coordinate scale (the solver's length scale).
    pub(super) fn scale(&self) -> f64 {
        self.samples
            .iter()
            .map(|(_, p)| p.x.abs().max(p.y.abs()).max(p.z.abs()))
            .fold(self.length, f64::max)
    }

    /// The curve's first point (its frame).
    pub(super) fn start(&self) -> Vec3 {
        self.samples[0].1
    }

    /// +1 when the curve runs counter-clockwise about `normal` (or is open),
    /// −1 when clockwise — the sign that turns `tangent × normal` outward.
    pub(super) fn orientation(&self, normal: Vec3) -> f64 {
        if !self.closed {
            return 1.0;
        }
        let o = self.samples[0].1;
        let twice_area: f64 = self
            .samples
            .windows(2)
            .map(|w| w[0].1.sub(o).cross(w[1].1.sub(o)).dot(normal))
            .sum();
        if twice_area < 0.0 { -1.0 } else { 1.0 }
    }

    /// Where `t` sits along the range: 0 at the start, 1 at the end.
    fn fraction(&self, t: f64) -> f64 {
        (t - self.t0) / (self.t1 - self.t0)
    }
}

/// A curve on one body.
#[derive(Clone, Debug)]
pub(super) struct CurveRef {
    pub(super) body: usize,
    pub(super) curve: Arc<SampledCurve>,
}

fn quat_conjugate(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// The closest point of `curve` to world point `wp`: `(parameter, world point)`.
fn closest_world(poses: &[Pose], curve: &CurveRef, wp: Vec3) -> (f64, Vec3) {
    let pose = poses[curve.body];
    let local = quat_rotate(quat_conjugate(pose.q), wp.sub(pose.t));
    let (t, c) = curve.curve.closest(local);
    (t, quat_rotate(pose.q, c).add(pose.t))
}

/// Point-on-curve rows: world `p − C(t*)` (3 rows; ⟂ the tangent at a
/// solution, so 2 effective).
pub(super) fn eval_point_on_curve(poses: &[Pose], p: LocalPoint, curve: &CurveRef, out: &mut Vec<f64>) {
    let wp = quat_rotate(poses[p.body].q, p.p).add(poses[p.body].t);
    let (_, wc) = closest_world(poses, curve, wp);
    let r = wp.sub(wc);
    out.extend([r.x, r.y, r.z]);
}

/// Where the axis line `(wp, wd)` crosses the plane through the curve's start
/// with the axis as normal — the point of the axis that is "at" the curve.
fn axis_crossing(curve: &CurveRef, poses: &[Pose], wp: Vec3, wd: Vec3) -> Vec3 {
    let pose = poses[curve.body];
    let start = quat_rotate(pose.q, curve.curve.samples[0].1).add(pose.t);
    wp.sub(wd.scale(wd.dot(wp.sub(start))))
}

/// Axis-on-curve rows: the part of `p − C(t*)` across the axis (3 rows; 2
/// effective), `t*` the closest curve point to the axis's crossing.
pub(super) fn eval_axis_on_curve(poses: &[Pose], p: LocalPoint, d: LocalDir, curve: &CurveRef, out: &mut Vec<f64>) {
    let wp = quat_rotate(poses[p.body].q, p.p).add(poses[p.body].t);
    let wd = quat_rotate(poses[d.body].q, d.d);
    let (_, wc) = closest_world(poses, curve, axis_crossing(curve, poses, wp, wd));
    let v = wp.sub(wc);
    let lateral = v.sub(wd.scale(wd.dot(v)));
    out.extend([lateral.x, lateral.y, lateral.z]);
}

/// The along row: `(fraction(t*) − target) · length` (model units), with the
/// fraction measured from the end when `reverse`, wrapped on a closed curve.
pub(super) fn eval_curve_along(
    poses: &[Pose],
    p: LocalPoint,
    axis: Option<LocalDir>,
    curve: &CurveRef,
    target: f64,
    reverse: bool,
    out: &mut Vec<f64>,
) {
    let mut wp = quat_rotate(poses[p.body].q, p.p).add(poses[p.body].t);
    if let Some(d) = axis {
        wp = axis_crossing(curve, poses, wp, quat_rotate(poses[d.body].q, d.d));
    }
    let (t, _) = closest_world(poses, curve, wp);
    let mut fraction = curve.curve.fraction(t);
    if reverse {
        fraction = 1.0 - fraction;
    }
    let mut delta = fraction - target;
    if curve.curve.closed {
        delta -= delta.round();
    }
    out.push(delta * curve.curve.length);
}

/// Cam contact row: the follower's in-plane offset from the profile along the
/// profile's outward side, minus the radius.
#[allow(clippy::too_many_arguments)]
pub(super) fn eval_cam_contact(
    poses: &[Pose],
    p: LocalPoint,
    axis: Option<LocalDir>,
    curve: &CurveRef,
    normal: LocalDir,
    radius: f64,
    side: f64,
    out: &mut Vec<f64>,
) {
    let wp = quat_rotate(poses[p.body].q, p.p).add(poses[p.body].t);
    let wn = quat_rotate(poses[normal.body].q, normal.d);
    // The follower's centre in the profile's plane: along the roller's axis,
    // else straight down the normal.
    let along = axis.map(|d| quat_rotate(poses[d.body].q, d.d)).unwrap_or(wn);
    let at = axis_crossing(curve, poses, wp, along);
    let (t, wc) = closest_world(poses, curve, at);
    let pose = poses[curve.body];
    let tangent = curve
        .curve
        .curve
        .derivatives(t, 1)
        .ok()
        .map(|d| quat_rotate(pose.q, d[1]))
        .unwrap_or(Vec3::new(0.0, 0.0, 0.0));
    let outward = tangent.cross(wn);
    let length = outward.length();
    let v = at.sub(wc);
    let signed = if length > ASM_DIR_EPS { v.dot(outward.scale(1.0 / length)) } else { v.length() };
    out.push(side * signed - radius);
}
