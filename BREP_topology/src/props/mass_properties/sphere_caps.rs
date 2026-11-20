//! A sphere face whose every trim is a circle of the sphere, read in closed
//! form (`BREP_MASS_SPHERE_CAPS`).
//!
//! Such a face is one spherical cap, or the whole sphere less disjoint caps.
//! A cap about the unit axis `a` above the level `h` is an untrimmed
//! rectangle in the frame turned onto `a`: x = c + R(t a + s(cos φ e1 +
//! sin φ e2)), s = √(1 − t²), with t over [h/R, 1], φ over [0, 2π) and
//! dA = R² dt dφ. Every integrand the mass code reads is a polynomial of
//! degree at most four in x and the unit normal. Integrated over φ, the
//! terms odd in s vanish and the even ones are polynomials in t, so the
//! trapezoid rule in φ (exact for trigonometric degree below its station
//! count) and Gauss–Legendre in t read every kind exactly, up to rounding.
//! The trimmed route instead reads the same region by chord polygons and a
//! cell quadtree: on a 4-face sphere shell, 270 chords and up to 3e5
//! stations per call.
//!
//! The region is the one the face's pcurves bound. A loop is taken as a
//! circle only when every lifted station lies on one plane to
//! `PLANE_RESIDUAL_REL` × the cap's height. Anything else declines to the
//! trimmed route:
//! - a loop that is not one full turn of one circle,
//! - two holes that touch,
//! - a face with no circle,
//! - a coarse or sliver-bound measure in progress.

use super::*;
use crate::env_read::Mode;

/// `BREP_MASS_SPHERE_CAPS`: `0|off` (default) keeps the trimmed route;
/// `check` reads both, prints the comparison and returns the trimmed
/// reading; `1|on` returns the closed form.
pub(super) fn sphere_caps_mode() -> Mode {
    crate::mode_switch!(live "BREP_MASS_SPHERE_CAPS", false)
}


/// A lifted trim station may sit this far (× the cap's height, the smaller
/// side's) off its circle's plane.
/// The bar is a reading of the pcurve, not a geometric tolerance. The region
/// the closed form reads and the one the pcurves bound differ by a strip no
/// wider than this along each circle, and the bar keeps that strip under
/// the trimmed route's own quadrature error.
pub const PLANE_RESIDUAL_REL: f64 = 1e-10;

/// The carrier's evaluated stations may sit this far (× R) off the
/// recognized sphere: rounding of the rational evaluation, no more.
pub const CARRIER_RESIDUAL_REL: f64 = 1e-12;

/// Stations per coedge for the circle test.
const STATIONS: usize = 48;

/// Trapezoid stations in φ: exact through trigonometric degree 15.
const PHI_STATIONS: usize = 16;

/// One circle of the face: the material is the cap {(x − c)·axis ≥ level}.
#[derive(Clone, Copy, Debug)]
pub struct Cap {
    pub axis: Vec3,
    pub level: f64,
    /// Largest station distance off the circle's plane, over R.
    pub residual: f64,
}

/// The face's circles, or why it is not read here.
pub fn sphere_face_caps(face: &FaceRecord) -> Result<Result<(Vec3, f64, f64, Vec<Cap>), String>, String> {
    let Some(crate::AnalyticSurface::Sphere { frame, radius }) = face.surface.analytic() else {
        return Ok(Err("not a sphere".into()));
    };
    let (centre, radius) = (frame.origin, *radius);
    if !(radius > 0.0) || face.loops.is_empty() {
        return Ok(Err("no loops".into()));
    }
    // The face's normal against the outward radial direction, read once off
    // the poles.
    let ku = crate::KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u)?;
    let kv = crate::KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v)?;
    let [u0, u1] = ku.domain();
    let [v0, v1] = kv.domain();
    let (point, su, sv) = face.surface.deriv1(0.5 * (u0 + u1), 0.5 * (v0 + v1))?;
    let normal = su.cross(sv).scale(if face.same_sense { 1.0 } else { -1.0 });
    let outward = normal.dot(point.sub(centre));
    if !(outward.abs() > 1e-6 * normal.length() * radius) {
        return Ok(Err("normal unread".into()));
    }
    let sense = outward.signum();
    // Recognition matches the net to 1e-9 of max(scale, 1), so on a small
    // sphere the recognized centre and radius may not be the net's own. The
    // closed form reads the recognized sphere, so the net must lie on it:
    // stations inside every knot span, against CARRIER_RESIDUAL_REL × R.
    let (u_breaks, v_breaks) = surface_breaks(&face.surface)?;
    let mut carrier = 0.0f64;
    for u in u_breaks.windows(2) {
        for v in v_breaks.windows(2) {
            for i in 0..=4 {
                for j in 0..=4 {
                    let (s, t) = (i as f64 / 4.0, j as f64 / 4.0);
                    let point = face.surface.evaluate(u[0] + (u[1] - u[0]) * s, v[0] + (v[1] - v[0]) * t)?;
                    carrier = carrier.max((point.sub(centre).length() - radius).abs());
                }
            }
        }
    }
    if carrier > CARRIER_RESIDUAL_REL * radius {
        return Ok(Err(format!("carrier off its sphere by {:.3e} R", carrier / radius)));
    }
    let degenerate = 1e-9 * radius;
    let mut caps = Vec::new();
    for loop_record in &face.loops {
        // A seam is walked twice in its loop and a pole coedge has no
        // length; neither bounds anything.
        let mut uses = std::collections::HashMap::new();
        for coedge in &loop_record.coedges {
            *uses.entry(coedge.edge_id).or_insert(0usize) += 1;
        }
        let mut runs: Vec<Vec<Vec3>> = Vec::new();
        for coedge in &loop_record.coedges {
            if uses[&coedge.edge_id] > 1 {
                continue;
            }
            let [a, b] = coedge.pcurve.domain()?;
            let mut run = Vec::with_capacity(STATIONS + 1);
            for index in 0..=STATIONS {
                let t = a + (b - a) * index as f64 / STATIONS as f64;
                // A station past the carrier's domain declines.
                let Ok(uv) = coedge.pcurve.evaluate(t) else {
                    return Ok(Err("pcurve unread".into()));
                };
                let Ok(lifted) = face.surface.evaluate(uv.x, uv.y) else {
                    return Ok(Err("station past the domain".into()));
                };
                run.push(lifted);
            }
            let reach = run.iter().map(|p| p.sub(run[0]).length()).fold(0.0, f64::max);
            if reach > degenerate {
                runs.push(run);
            }
        }
        // A loop may carry more than one circle (a zone's two rims joined
        // by its seam): group the runs by the plane each one's own closed
        // polygon names, then read every group as one circle.
        let mut groups: Vec<(Vec3, f64, Vec<&Vec<Vec3>>)> = Vec::new();
        for run in &runs {
            let near = |axis: Vec3, level: f64| {
                run.iter().all(|p| (p.sub(centre).dot(axis) - level).abs() <= 1e-7 * radius)
            };
            if let Some(group) = groups.iter_mut().find(|(axis, level, _)| near(*axis, *level)) {
                group.2.push(run);
                continue;
            }
            let Some((axis, level)) = run_plane(run, centre, radius) else {
                return Ok(Err("short arc".into()));
            };
            groups.push((axis, level, vec![run]));
        }
        for (_, _, members) in groups {
            match circle_cap(&members, centre, radius) {
                Ok((axis, level, residual)) => {
                    // The circle runs counter-clockwise about `axis`. The
                    // material lies to its left under the face normal, toward
                    // +axis when that normal points out of the sphere and
                    // toward −axis when it points in.
                    caps.push(Cap { axis: axis.scale(sense), level: level * sense, residual });
                }
                Err(reason) => return Ok(Err(reason)),
            }
        }
    }
    if caps.is_empty() {
        return Ok(Err("no circle".into()));
    }
    if caps.len() > 1 {
        // The holes are the caps' complements; they must not meet.
        let hole = |cap: &Cap| (cap.axis.scale(-1.0), (-cap.level / radius).clamp(-1.0, 1.0).acos());
        for (i, first) in caps.iter().enumerate() {
            for second in &caps[i + 1..] {
                let ((a, alpha), (b, beta)) = (hole(first), hole(second));
                let apart = a.dot(b).clamp(-1.0, 1.0).acos();
                if !(apart > alpha + beta + 1e-6) {
                    return Ok(Err("holes meet".into()));
                }
            }
        }
    }
    Ok(Ok((centre, radius, sense, caps)))
}

/// The plane of one run: the vector area of the run closed by its chord.
fn run_plane(run: &[Vec3], centre: Vec3, radius: f64) -> Option<(Vec3, f64)> {
    let mean = run.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, p| sum.add(*p)).scale(1.0 / run.len() as f64);
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    for (i, p) in run.iter().enumerate() {
        let q = run[(i + 1) % run.len()];
        area = area.add(p.sub(mean).cross(q.sub(mean)));
    }
    if !(area.length() > 1e-8 * radius * radius) {
        return None;
    }
    let axis = area.normalized().ok()?;
    let level = run.iter().map(|p| p.sub(centre).dot(axis)).sum::<f64>() / run.len() as f64;
    Some((axis, level))
}

/// One circle from the runs that walk it: its axis (the walk is
/// counter-clockwise about it), its level and its residual, or why not.
fn circle_cap(runs: &[&Vec<Vec3>], centre: Vec3, radius: f64) -> Result<(Vec3, f64, f64), String> {
    let stations = runs.iter().map(|run| run.len()).sum::<usize>() as f64;
    let mean = runs
        .iter()
        .flat_map(|run| run.iter())
        .fold(Vec3::new(0.0, 0.0, 0.0), |sum, p| sum.add(*p))
        .scale(1.0 / stations);
    // Newell's vector area: the runs join end to end round one circle, so
    // their chord terms sum to the closed polygon's, in any order.
    let mut area = Vec3::new(0.0, 0.0, 0.0);
    for run in runs {
        for pair in run.windows(2) {
            area = area.add(pair[0].sub(mean).cross(pair[1].sub(mean)));
        }
    }
    let Ok(axis) = area.normalized() else {
        return Err("flat loop".into());
    };
    let level = runs.iter().flat_map(|run| run.iter()).map(|p| p.sub(centre).dot(axis)).sum::<f64>() / stations;
    let residual = runs
        .iter()
        .flat_map(|run| run.iter())
        .map(|p| (p.sub(centre).dot(axis) - level).abs())
        .fold(0.0, f64::max)
        / radius;
    let rim = (radius * radius - level * level).max(0.0);
    if !(rim > 1e-12 * radius * radius) {
        return Err("point circle".into());
    }
    // One full turn: the inscribed polygon holds just under the disc.
    let turn = 0.5 * area.length() / (std::f64::consts::PI * rim);
    if !(0.95..=1.0 + 1e-9).contains(&turn) {
        return Err(format!("not one turn ({turn:.4}, {} runs)", runs.len()));
    }
    // A level error δ moves the cap's measures by δ over its height, so the
    // bar is on the height of the smaller side, not on R.
    let height = radius - level.abs();
    if residual * radius > PLANE_RESIDUAL_REL * height {
        return Err(format!("off its plane by {residual:.3e} R (height {:.3e} R)", height / radius));
    }
    Ok((axis, level, residual))
}

/// Every `kind` over the cap {(x − centre)·axis ≥ level} of the sphere,
/// with the face's normal `sense` (+1 outward).
fn cap_integral(centre: Vec3, radius: f64, sense: f64, axis: Vec3, level: f64, kinds: &[Integrand], totals: &mut [f64]) -> Result<(), String> {
    let e1 = axis.perpendicular()?.normalized()?;
    let e2 = axis.cross(e1);
    let t0 = (level / radius).clamp(-1.0, 1.0);
    let half = 0.5 * (1.0 - t0);
    let middle = 0.5 * (1.0 + t0);
    let dphi = std::f64::consts::TAU / PHI_STATIONS as f64;
    for (x, w) in GAUSS_X.iter().zip(GAUSS_W.iter()) {
        let t = middle + half * x;
        let s = (1.0 - t * t).max(0.0).sqrt();
        let weight = w * half * dphi;
        for j in 0..PHI_STATIONS {
            let phi = dphi * j as f64;
            let unit = axis.scale(t).add(e1.scale(s * phi.cos())).add(e2.scale(s * phi.sin()));
            let point = centre.add(unit.scale(radius));
            let weighted_normal = unit.scale(sense * radius * radius);
            for (total, kind) in totals.iter_mut().zip(kinds) {
                *total += weight * integrand_value(*kind, point, weighted_normal);
            }
        }
    }
    Ok(())
}

/// The closed-form reading of `face`, or `None` when its trims are not
/// circles the route reads.
pub fn sphere_caps_integral(face: &FaceRecord, kinds: &[Integrand]) -> Result<Option<Vec<f64>>, String> {
    let Ok((centre, radius, sense, caps)) = sphere_face_caps(face)? else {
        return Ok(None);
    };
    let mut totals = vec![0.0; kinds.len()];
    if caps.len() == 1 {
        let cap = caps[0];
        cap_integral(centre, radius, sense, cap.axis, cap.level, kinds, &mut totals)?;
        return Ok(Some(totals));
    }
    // The whole sphere less each hole: a hole is the cap about −axis above
    // −level.
    let any = caps[0].axis;
    cap_integral(centre, radius, sense, any, -radius, kinds, &mut totals)?;
    let mut hole = vec![0.0; kinds.len()];
    for cap in &caps {
        hole.iter_mut().for_each(|value| *value = 0.0);
        cap_integral(centre, radius, sense, cap.axis.scale(-1.0), -cap.level, kinds, &mut hole)?;
        for (total, value) in totals.iter_mut().zip(&hole) {
            *total -= value;
        }
    }
    Ok(Some(totals))
}
