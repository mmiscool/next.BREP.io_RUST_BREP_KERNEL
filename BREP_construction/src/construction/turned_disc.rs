//! A disc's prism turned partway about an axis along it: the plane region it
//! passes over is bounded by circular arcs alone, built exactly and extruded,
//! without the level engine (`BREP_SWEEP_TURNED_DISC`, default on; `=0` restores
//! the engine).
//!
//! A disc of radius r whose centre stands d from the pivot, turned through α < 2π:
//! at distance ρ from the pivot the turned disc reaches the angles where the
//! centre comes within r, an arc of directions that widens as ρ nears d. So
//! - with the pivot inside the disc (d < r), along every direction of the turn
//!   the region reaches d + r (the centre's own direction); outside the turn the
//!   nearer end's disc reaches farthest (a disc's reach falls with the angle off
//!   its centre's direction), the two ends meeting on the bisector behind. The
//!   boundary is the arc of radius d + r over the turn, the end disc's arc on
//!   to the point Q behind where the two end discs cross, and the start disc's
//!   arc on from Q to where it began.
//! - with the pivot outside (d > r), the region is the band d − r .. d + r over
//!   the turn and the two end discs' halves beyond it: the outer arc, the end
//!   disc's leading half, the inner arc back, the start disc's trailing half
//!   (while the two halves keep apart: α + 2·asin(r/d) < 2π).
//! The prism is answered whole, so only where the clip holds all of it (an
//! upright clip about a parallel axis whose band and heights take the
//! region's bounding circle).
use crate::solid_sweep::{OutlinePiece, SolidMotion, SweepClip, SweptTool};
use crate::{extrude_profile_brep, make_arc, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3};
use std::f64::consts::{PI, TAU};

/// The switch (default on; `BREP_SWEEP_TURNED_DISC=0` restores the level engine).
pub(crate) fn turned_disc_on() -> bool {
    crate::switch!("BREP_SWEEP_TURNED_DISC", true)
}

/// The arcs of the region's boundary, anticlockwise in the plane square to the
/// turn's axis about the pivot, as (centre, radius, start, end) with end > start;
/// `None` where it is not of the two kinds above.
pub(crate) fn turned_disc_region(d: f64, phi0: f64, alpha: f64, r: f64) -> Option<Vec<([f64; 2], f64, f64, f64)>> {
    if !(r > 0.0 && d >= 0.0 && alpha > 0.0 && alpha < TAU - 1e-9 && d.is_finite() && phi0.is_finite()) {
        return None;
    }
    let dir = |a: f64| [a.cos(), a.sin()];
    let at = |a: f64, s: f64| [s * a.cos(), s * a.sin()];
    let (c0, c1) = (at(phi0, d), at(phi0 + alpha, d));
    let end = phi0 + alpha;
    if d < (1.0 - 1e-9) * r && d > 1e-9 * r {
        // Q: on the bisector behind, where the two end discs cross.
        let half = 0.5 * alpha;
        let t = d * half.cos() - (r * r - d * d * half.sin() * half.sin()).sqrt();
        let b = dir(phi0 + half);
        let q = [t * b[0], t * b[1]];
        let angle_from = |c: [f64; 2], from: f64| from + (((q[1] - c[1]).atan2(q[0] - c[0])) - from).rem_euclid(TAU);
        let beta1 = angle_from(c1, end);
        let beta0 = angle_from(c0, phi0);
        return Some(vec![([0.0, 0.0], d + r, phi0, end), (c1, r, end, beta1), (c0, r, beta0, phi0 + TAU)]);
    }
    if d > (1.0 + 1e-9) * r {
        if !(alpha + 2.0 * (r / d).asin() < TAU - 1e-9) {
            return None;
        }
        // The inner arc runs back: written forward here, reversed by the caller (marked by a negative radius).
        return Some(vec![([0.0, 0.0], d + r, phi0, end), (c1, r, end, end + PI), ([0.0, 0.0], -(d - r), phi0, end), (c0, r, phi0 + PI, phi0 + TAU)]);
    }
    None
}

/// The sweep of `tool` along `motion` inside `clip` where it is a disc's prism
/// turned partway about an axis along it and the clip holds all of it: the
/// region, exactly, extruded. `None` leaves it to the caller's engine.
pub(crate) fn turned_disc(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    if !turned_disc_on() {
        return Ok(None);
    }
    let SweptTool::Prism { origin, normal, x_axis, outline, from, to } = tool else { return Ok(None) };
    let [OutlinePiece::Arc { centre, radius, start, end }] = outline.as_slice() else { return Ok(None) };
    let r = *radius;
    if !(r > 0.0 && r.is_finite() && ((end - start).abs() - TAU).abs() < 1e-12 && to > from) {
        return Ok(None);
    }
    let [step] = motion.steps.as_slice() else { return Ok(None) };
    if step.advance != 0.0 || step.turn == 0.0 || !(motion.to > motion.from) || clip.angle.is_some() {
        return Ok(None);
    }
    let (Ok(e3), Ok(n), Ok(ez), Ok(xa)) = (step.axis_direction.normalized(), normal.normalized(), clip.axis_direction.normalized(), x_axis.normalized()) else { return Ok(None) };
    // The prism, the turn and the clip all upright; the outline's axes square to the prism.
    if n.cross(e3).length() > 1e-12 || ez.cross(e3).length() > 1e-12 || xa.dot(n).abs() > 1e-12 {
        return Ok(None);
    }
    let ya = n.cross(xa);
    let e1 = xa.sub(e3.scale(xa.dot(e3))).normalized().or_refuse(KernelStage::Collect, "turned_disc_frame")?;
    let e2 = e3.cross(e1);
    let pivot = step.axis_origin;
    let flat = |x: Vec3| -> [f64; 2] {
        let q = x.sub(pivot);
        [q.dot(e1), q.dot(e2)]
    };
    let c = flat(origin.add(xa.scale(centre[0])).add(ya.scale(centre[1])));
    let d = c[0].hypot(c[1]);
    let (ta, tb) = (step.turn * motion.from, step.turn * motion.to);
    let (lo, hi) = (ta.min(tb), ta.max(tb));
    let alpha = hi - lo;
    let phi0 = c[1].atan2(c[0]) + lo;
    let Some(arcs) = turned_disc_region(d, phi0, alpha, r) else { return Ok(None) };
    // Heights along the turn's axis, from the pivot.
    let base = origin.sub(pivot).dot(e3);
    let sense = n.dot(e3);
    let (ha, hb) = (base + from * sense, base + to * sense);
    let (h0, h1) = (ha.min(hb), ha.max(hb));
    // The clip holds the whole prism: its heights, and the region's bounding
    // circle (radius d + r about the pivot) within its band.
    let reach = d + r;
    let pc = flat(clip.axis_origin);
    let gap = pc[0].hypot(pc[1]);
    let clip_height = |h: f64| pivot.sub(clip.axis_origin).dot(ez) + h * e3.dot(ez);
    let (ca, cb) = (clip_height(h0), clip_height(h1));
    // Hairs of the region's own sizes (across and along), not of where its
    // pivot or the clip stand.
    let (across, along) = (1e-9 * reach, 1e-9 * (h1 - h0));
    let held = ca.min(cb) >= clip.axial[0] - along
        && ca.max(cb) <= clip.axial[1] + along
        && gap + reach <= clip.radius[1] + across
        && (clip.radius[0] <= 0.0 || gap - reach >= clip.radius[0] - across);
    if !held {
        return Ok(None);
    }
    let at = |q: [f64; 2]| pivot.add(e1.scale(q[0])).add(e2.scale(q[1])).add(e3.scale(h0));
    let mut curves: Vec<NurbsCurve> = Vec::with_capacity(arcs.len());
    for (centre, radius, a, b) in arcs {
        let curve = if radius < 0.0 {
            make_arc(at(centre), e1, e2, -radius, a, b).and_then(|arc| arc.reversed())
        } else {
            make_arc(at(centre), e1, e2, radius, a, b)
        };
        // An arc too short to build (a turn of a hair) is the engine's to sweep.
        let Ok(curve) = curve else { return Ok(None) };
        curves.push(curve);
    }
    // The arcs chain end to start (the region's corners), or this is not built.
    let tolerance = 1e-9 * reach.max(r);
    for k in 0..curves.len() {
        let (Ok(da), Ok(db)) = (curves[k].domain(), curves[(k + 1) % curves.len()].domain()) else { return Ok(None) };
        let (Ok(p), Ok(q)) = (curves[k].evaluate(da[1]), curves[(k + 1) % curves.len()].evaluate(db[0])) else { return Ok(None) };
        if p.sub(q).length() > tolerance {
            return Ok(None);
        }
    }
    Ok(extrude_profile_brep(&curves, e3, h1 - h0).ok().map(|body| vec![body]))
}

