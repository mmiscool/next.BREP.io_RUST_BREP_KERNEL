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
    let hits = match crate::curve_curve_intersection::intersect_curves_exact_only(&pa,&pb,tolerance) {
        Some(hits) => {
            if hits.iter().any(|h|h.tangential) { return None; }
            hits
        }
        None if spline_crossings_enabled() => {
            let mut hits = crate::curve_curve_intersection::intersect_curves(&pa,&pb,tolerance).ok()?;
            hits.sort_by(|x,y| x.s.total_cmp(&y.s));
            // The general search marks a crossing under a milliradian as
            // tangential: profiles a hair apart, one moved or turned slightly
            // off the other, cross at such angles. A shallow hit counts only
            // when the profiles read as crossing there: midway between it and
            // each neighbour, b lies clear of a by ten tolerances, on
            // alternating sides. One reading per stretch does not exclude a
            // close pair of crossings the search merged inside a stretch;
            // this route stays behind its switch until that is bounded.
            if hits.iter().any(|h|h.tangential) && !crossings_alternate(&pa,&pb,axis,&hits,tolerance) { return None; }
            hits
        }
        None => return None,
    };
    if hits.is_empty() { return None; }
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

/// `BREP_EXTRUSION_SPLINE_CROSSINGS` (default off): parallel extrusions whose
/// profiles no closed-form lane covers (B-splines) take their crossings from
/// the general planar curve search, shallow ones certified by
/// [`crossings_alternate`].
fn spline_crossings_enabled() -> bool {
    crate::switch!("BREP_EXTRUSION_SPLINE_CROSSINGS", false)
}

/// Whether the projected profiles `a` and `b` (in the plane normal to `axis`)
/// lie on alternating sides of each other between consecutive hits (sorted
/// by `s`), each read at least ten tolerances clear. For closed profiles the
/// stretch from the last hit around to the first counts too. A reading that
/// fails to converge or comes out within the band declines.
fn crossings_alternate(a: &NurbsCurve, b: &NurbsCurve, axis: Vec3, hits: &[crate::curve_curve_intersection::CurveCurveIntersection], tolerance: f64) -> bool {
    let (Ok([s0, s1]), Ok([t0, t1])) = (a.domain(), b.domain()) else { return false };
    let closed = |c: &NurbsCurve, lo: f64, hi: f64| match (c.evaluate(lo), c.evaluate(hi)) {
        (Ok(p), Ok(q)) => p.sub(q).length() <= tolerance,
        _ => false,
    };
    let (a_closed, b_closed) = (closed(a, s0, s1), closed(b, t0, t1));
    // The signed side of a(s) from b, read at b's foot point near `t`.
    let side = |s: f64, mut t: f64| -> Option<f64> {
        let p = a.evaluate(s).ok()?;
        for _ in 0..32 {
            let (q, d) = b.deriv1(t).ok()?;
            let dd = d.dot(d);
            if !(dd > 0.0) { return None; }
            let step = p.sub(q).dot(d) / dd;
            t += step;
            if b_closed { t = t0 + (t - t0).rem_euclid(t1 - t0); } else { t = t.clamp(t0, t1); }
            if step.abs() <= 1e-15 * (t1 - t0) { break; }
        }
        let (q, d) = b.deriv1(t).ok()?;
        let gap = p.sub(q);
        // Off b's span end the foot is not a perpendicular one: decline.
        if gap.dot(d).abs() > 1e-6 * gap.length() * d.length() + tolerance * d.length() { return None; }
        Some(d.normalized().ok()?.cross(gap).dot(axis))
    };
    let mut pairs: Vec<(usize, usize)> = (1..hits.len()).map(|i| (i - 1, i)).collect();
    if a_closed && b_closed && hits.len() >= 2 { pairs.push((hits.len() - 1, 0)); }
    if a_closed != b_closed || pairs.is_empty() { return false; }
    let mut last: Option<f64> = None;
    for (i, j) in pairs {
        let (h, k) = (&hits[i], &hits[j]);
        let span_s = if k.s > h.s { k.s - h.s } else { k.s + (s1 - s0) - h.s };
        let mut span_t = k.t - h.t;
        if b_closed { span_t = (span_t - t0 + (t1 - t0) * 0.5).rem_euclid(t1 - t0) - (t1 - t0) * 0.5 + t0; }
        let wrap = |x: f64, lo: f64, hi: f64| if a_closed { lo + (x - lo).rem_euclid(hi - lo) } else { x };
        let Some(value) = side(wrap(h.s + 0.5 * span_s, s0, s1), wrap(h.t + 0.5 * span_t, t0, t1)) else { return false };
        if !(value.abs() > 10.0 * tolerance) { return false; }
        if let Some(previous) = last {
            if previous.signum() == value.signum() { return false; }
        }
        last = Some(value);
    }
    // Around a closed loop the sides alternate an even number of times.
    !(a_closed && hits.len() % 2 == 1)
}
