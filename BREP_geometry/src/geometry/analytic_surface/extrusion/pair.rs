use super::*;

/// Parallel extrusions meet along rulings through the crossings of their
/// profiles projected perpendicular to the common axis. Only profile pairs
/// covered by the exact curve crossing lanes are admitted; overlaps,
/// unsupported conics and uncertain contacts keep general SSI.
pub fn parallel_extrusion_sections(a: &ExtrusionStructure, b: &ExtrusionStructure, tolerance: f64) -> Option<Vec<NurbsCurve>> {
    if !tolerance.is_finite() || tolerance <= 0.0 { return None; }
    let axis = a.direction.normalized().ok()?;
    let other = b.direction.normalized().ok()?;
    if axis.cross(other).length() > 32.0*f64::EPSILON { return None; }
    // Bound the discarded perpendicular motion over both entire sweeps,
    // including normalization roundoff in the chosen common axis.
    let drift = |e: &ExtrusionStructure| -> Option<f64> {
        let length = e.sweep_domain[1]-e.sweep_domain[0];
        let value = e.direction.sub(axis.scale(e.direction.dot(axis))).length()*length;
        if !length.is_finite() || length <= 0.0 || !value.is_finite() { return None; }
        Some(value)
    };
    let drift_a = drift(a)?;
    let drift_b = drift(b)?;
    if drift_a+drift_b > tolerance { return None; }
    let origin = a.profile.control_points.first()?.point().ok()?;
    let projected = |curve: &NurbsCurve| -> Option<NurbsCurve> {
        let controls = curve.control_points.iter().map(|p| {
            let delta = p.point().ok()?.sub(origin);
            let q = delta.sub(axis.scale(delta.dot(axis)));
            Some(crate::Vec4{x:q.x*p.w,y:q.y*p.w,z:q.z*p.w,w:p.w})
        }).collect::<Option<Vec<_>>>()?;
        NurbsCurve::new(curve.degree,curve.knots.clone(),controls).ok()
    };
    let pa = projected(&a.profile)?; let pb = projected(&b.profile)?;
    let hits = crate::curve_curve_intersection::intersect_curves_exact_only(&pa,&pb,tolerance)?;
    if hits.is_empty() || hits.iter().any(|h|h.tangential) { return None; }
    let mut curves = Vec::new();
    for hit in hits {
        let p = a.profile.evaluate(hit.s).ok()?;
        let q = b.profile.evaluate(hit.t).ok()?;
        let relative = q.sub(p);
        let residual = relative.sub(axis.scale(relative.dot(axis))).length();
        if !residual.is_finite() || residual+drift_a+drift_b>tolerance { return None; }
        let a_end = a.direction.dot(axis)*(a.sweep_domain[1]-a.sweep_domain[0]);
        let b_start = relative.dot(axis);
        let b_end = b_start+b.direction.dot(axis)*(b.sweep_domain[1]-b.sweep_domain[0]);
        let low = 0.0_f64.max(b_start.min(b_end));
        let high = a_end.min(b_start.max(b_end));
        if high < low-tolerance { continue; }
        if high-low <= tolerance { return None; }
        curves.push(crate::make_line(p.add(axis.scale(low)),p.add(axis.scale(high))).ok()?);
    }
    Some(curves)
}
