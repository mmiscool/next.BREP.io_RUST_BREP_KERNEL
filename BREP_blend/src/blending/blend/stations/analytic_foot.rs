//! Optional analytic rail projection on affine planes and closed ring tori. Every accepted reading remains
//! a distance to an evaluated point of the stored carrier.
use super::*;

pub(super) fn enabled() -> bool {
    switch!("BREP_BLEND_ANALYTIC_RAIL_FOOT", true)
}

pub(super) fn distance(carrier: &NurbsSurface, point: Vec3, seed: [f64; 2]) -> Option<Result<f64, KernelRefusal>> {
    let analytic = carrier.analytic()?;
    match analytic {
        crate::AnalyticSurface::Torus { major_radius, minor_radius, .. }
            if major_radius > minor_radius && *minor_radius > 0.0
                && carrier.closed_directions().ok()? == (true, true) => {},
        crate::AnalyticSurface::Plane { .. } => {},
        _ => return None,
    }
    if ![point.x, point.y, point.z, seed[0], seed[1]].iter().all(|x| x.is_finite()) { return Some(Ok(f64::INFINITY)); }
    let initial = match crate::blend::track_fit::carrier_point(carrier, seed[0], seed[1]).or_refuse(KernelStage::Refine, "evaluate_extended") {
        Ok(p) => p.sub(point).length(), Err(e) => return Some(Err(e)),
    };
    if !initial.is_finite() { return Some(Ok(f64::INFINITY)); }
    let foot = match analytic {
        crate::AnalyticSurface::Plane { origin, u_dir, v_dir, u_domain, v_domain } => {
            // The rail check is on the carrier's extension, so do not clamp
            // this affine inverse to the face's finite parameter rectangle.
            let d = point.sub(*origin);
            let (a, b, c) = (u_dir.dot(*u_dir), u_dir.dot(*v_dir), v_dir.dot(*v_dir));
            let det = a * c - b * b;
            if !det.is_finite() || det.abs() <= 1e-16 * (a * c).max(1.0) { return None; }
            let (fu, fv) = (u_dir.dot(d), v_dir.dot(d));
            let u = (fu * c - fv * b) / det + u_domain[0];
            let v = (fv * a - fu * b) / det + v_domain[0];
            carrier.evaluate_extended(u, v).ok()?
        },
        _ => analytic.project(carrier, point)?.point,
    };
    let distance = foot.sub(point).length();
    if !distance.is_finite() { return None; }
    crate::blend::record_count("count.closed.analytic_rail_foot", 1);
    Some(Ok(initial.min(distance)))
}

