//! The point classifier's exact ray lane (lane B row 1, 2026-10-17): the
//! real roots of a STRAIGHT segment against an ANALYTIC carrier, read in
//! closed form and handed to [`crate::curve_surface_intersection::intersect_curve_surface_from_seeds`]
//! as its Newton seeds.
//!
//! The general search ([`crate::intersect_curve_surface`]) samples the
//! segment 32 times, filters the samples by span boxes and runs one Newton
//! per surviving sample — 56–185 µs per ray and face on the rim-fillet rows
//! of the perf slate, a quarter of each case, for a LINE against a plane, a
//! cylinder or a torus band whose roots are a linear, quadratic or quartic
//! equation. This lane produces exactly those roots as seeds; the Newton,
//! the gap filter, the duplicate guard and the tangency reading that follow
//! are the general search's own code, so a hit set and its `tangential`
//! flags are the general search's whenever its sampling found every root.
//!
//! It DECLINES (returns `None`, the general search answers) whenever the
//! closed form is not a safe reading: a curve that is not a straight segment,
//! a surface with no analytic recognition or of a kind not covered (a general
//! revolution whose generatrix is neither a straight span nor one circular
//! arc), a leading coefficient or a near-double root within
//! [`DEGENERATE`] of zero relative to the polynomial's own size (a grazing
//! ray, where the sampled Newton's tangency reading must stand), or a ray
//! parallel to a plane.
//!
//! Nothing here reads a clock; the work is counted at the classifier
//! (`classify.profile … exact_faces= fallback_faces=`).

use crate::poly::real_roots;
use crate::{AnalyticSurface, NurbsCurve, NurbsSurface, RevolutionFrame, Vec3};

/// Relative bar under which a coefficient, a discriminant or a critical
/// value of the root polynomial reads as zero and the lane declines.
const DEGENERATE: f64 = 1e-9;

/// The closed-form Newton seeds of `curve` (a straight segment) against
/// `surface` (an analytic carrier), as curve parameters inside the curve's
/// domain and sorted; `None` when the lane declines (see the module note).
/// `tolerance` is the caller's intersection tolerance: a root up to ten
/// tolerances past either end of the segment is kept, clamped to the end,
/// because the general search's clamped Newton reports such a hit too.
pub(crate) fn exact_line_surface_seeds(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<f64>> {
    let (p0, p1, [k0, k1]) = straight_segment(curve)?;
    let d = p1.sub(p0);
    let length = d.length();
    if !(length > 0.0) || !length.is_finite() {
        return None;
    }
    let analytic = surface.analytic()?;
    let roots = match analytic {
        AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
            plane_roots(p0, d, *origin, u_dir.cross(*v_dir))?
        }
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } => {
            if !(height.abs() > 0.0) {
                return None;
            }
            cone_roots(p0, d, frame, 0.0, *rho0, (rho1 - rho0) / height)?
        }
        AnalyticSurface::Sphere { frame, radius } => sphere_roots(p0, d, frame.origin, *radius)?,
        AnalyticSurface::Torus { frame, major_radius, minor_radius } => {
            torus_roots(p0, d, frame, 0.0, *major_radius, *minor_radius)?
        }
        AnalyticSurface::Revolution { frame, generatrix, .. } => {
            revolution_roots(p0, d, frame, generatrix)?
        }
    };
    let slack = (10.0 * tolerance / length).max(0.0);
    let mut seeds: Vec<f64> = roots
        .into_iter()
        .filter(|s| s.is_finite() && *s >= -slack && *s <= 1.0 + slack)
        .map(|s| k0 + s.clamp(0.0, 1.0) * (k1 - k0))
        .collect();
    seeds.sort_by(f64::total_cmp);
    Some(seeds)
}

/// Lane O row 16 (2026-10-28): the closed-form Newton seeds of a NURBS curve
/// of degree at most three against an affine PLANE carrier. The weighted
/// signed distance f(t) = n·(w·C(t)) − (n·origin)·w(t) is a plain polynomial
/// of the curve's degree on each knot span (the homogeneous numerator is
/// polynomial), so it is read EXACTLY from degree + 1 samples per span and its
/// real roots solved there. A leading coefficient under [`DEGENERATE`] of the
/// span's largest is dropped (its other root lies ~1e9 spans away); a span
/// lying in the plane, a double root (a grazing arc) or a non-finite reading
/// declines the whole curve, and the general search answers. `tolerance`
/// keeps a root up to ten tolerances past either end of the curve, clamped to
/// that end, as the line lane does.
pub(crate) fn exact_curve_plane_seeds(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<f64>> {
    let degree = curve.degree;
    if !(1..=3).contains(&degree) {
        return None;
    }
    let AnalyticSurface::Plane { origin, u_dir, v_dir, .. } = surface.analytic()? else { return None };
    let normal = u_dir.cross(*v_dir);
    let normal = normal.scale(1.0 / normal.length().max(f64::MIN_POSITIVE));
    if !normal.length().is_finite() || !(normal.length() > 0.5) {
        return None;
    }
    let offset = normal.dot(*origin);
    let [d0, d1] = curve.domain().ok()?;
    let mut breaks: Vec<f64> = curve.knots.iter().copied().filter(|&k| k >= d0 && k <= d1).collect();
    breaks.dedup();
    let reading = |t: f64| -> Option<f64> {
        let h = curve.evaluate_homogeneous(t).ok()?;
        let value = normal.x * h.x + normal.y * h.y + normal.z * h.z - offset * h.w;
        value.is_finite().then_some(value)
    };
    let start = curve.evaluate(d0).ok()?;
    let end = curve.evaluate(d1).ok()?;
    let mut seeds = Vec::new();
    for span in breaks.windows(2) {
        let (k0, k1) = (span[0], span[1]);
        if !(k1 > k0) {
            continue;
        }
        // Samples at s = i / degree, then the power-basis coefficients in s.
        let values = (0..=degree).map(|i| reading(k0 + (k1 - k0) * i as f64 / degree as f64)).collect::<Option<Vec<_>>>()?;
        let mut coefficients = match degree {
            1 => vec![values[0], values[1] - values[0]],
            2 => {
                // f(0) = a, f(1/2) = a + b/2 + c/4, f(1) = a + b + c.
                let (f0, fh, f1) = (values[0], values[1], values[2]);
                vec![f0, 4.0 * fh - 3.0 * f0 - f1, 2.0 * f1 + 2.0 * f0 - 4.0 * fh]
            }
            _ => {
                // Samples at 0, 1/3, 2/3, 1 (Newton forward differences, h = 1/3).
                let (f0, f1, f2, f3) = (values[0], values[1], values[2], values[3]);
                let d1 = f1 - f0;
                let d2 = f2 - 2.0 * f1 + f0;
                let d3 = f3 - 3.0 * f2 + 3.0 * f1 - f0;
                // f(s) = f0 + d1·x + d2·x(x−1)/2 + d3·x(x−1)(x−2)/6 with x = 3s.
                let c3 = d3 * 27.0 / 6.0;
                let c2 = (d2 / 2.0 - d3 / 2.0) * 9.0;
                let c1 = (d1 - d2 / 2.0 + d3 / 3.0) * 3.0;
                vec![f0, c1, c2, c3]
            }
        };
        let largest = coefficients.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        let scale = values.iter().fold(0.0_f64, |m, v| m.max(v.abs())).max(largest);
        let weight_scale = curve.control_points.iter().fold(0.0_f64, |m, p| m.max(p.w.abs()));
        if !(largest > DEGENERATE * weight_scale * (1.0 + start.length().max(end.length()))) {
            // The span lies in the plane (or reads nothing): decline.
            return None;
        }
        while coefficients.len() > 2 && coefficients.last().is_some_and(|c| c.abs() <= DEGENERATE * scale) {
            coefficients.pop();
        }
        if coefficients.len() == 2 && coefficients[1].abs() <= DEGENERATE * scale {
            // Constant and nonzero over the span: no root here.
            continue;
        }
        let span_chord = curve.evaluate(k0).ok()?.sub(curve.evaluate(k1).ok()?).length().max(f64::MIN_POSITIVE);
        let slack = (10.0 * tolerance / span_chord).max(0.0);
        for root in real_roots(&coefficients)? {
            if !(root.is_finite() && root >= -slack && root <= 1.0 + slack) {
                continue;
            }
            seeds.push((k0 + root * (k1 - k0)).clamp(d0, d1));
        }
    }
    seeds.sort_by(f64::total_cmp);
    seeds.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (1.0 + b.abs()));
    Some(seeds)
}

/// A quadric carrier as its implicit equation: a sphere `|P − center| = radius`,
/// or a surface of revolution whose radius is linear in the axial coordinate
/// (`cone_roots`' ρ(h) = rho_at_h0 + slope·(h − h_reference); a cylinder has
/// slope 0).
enum Quadric<'a> {
    Sphere { center: Vec3, radius: f64 },
    Cone { frame: &'a RevolutionFrame, h_reference: f64, rho_at_h0: f64, slope: f64 },
}

/// The quadric behind an analytic carrier, or `None` for a plane (row 16's
/// lane), a torus, an annulus or a revolution with a curved generatrix.
fn quadric_of(analytic: &AnalyticSurface) -> Option<Quadric<'_>> {
    match analytic {
        AnalyticSurface::Sphere { frame, radius } => Some(Quadric::Sphere { center: frame.origin, radius: *radius }),
        AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } => {
            (height.abs() > 0.0).then(|| Quadric::Cone { frame, h_reference: 0.0, rho_at_h0: *rho0, slope: (rho1 - rho0) / height })
        }
        AnalyticSurface::Revolution { frame, generatrix, .. } => {
            let controls = &generatrix.control_points;
            if generatrix.degree != 1 || controls.len() != 2 || controls[0].w != controls[1].w {
                return None;
            }
            let meridian = |point: Vec3| -> Option<(f64, f64)> {
                let offset = point.sub(frame.origin);
                let axial = offset.dot(frame.axis);
                let radial = offset.sub(frame.axis.scale(axial));
                let rho = radial.dot(frame.x_axis);
                let off_plane = radial.dot(frame.y_axis).abs();
                (off_plane <= 1e-9 * (1.0 + rho.abs() + axial.abs())).then_some((rho, axial))
            };
            let (rho_a, h_a) = meridian(controls[0].point().ok()?)?;
            let (rho_b, h_b) = meridian(controls[1].point().ok()?)?;
            let span = ((rho_b - rho_a).powi(2) + (h_b - h_a).powi(2)).sqrt();
            if !(span > 0.0) || (h_b - h_a).abs() <= DEGENERATE * span {
                return None;
            }
            Some(Quadric::Cone { frame, h_reference: h_a, rho_at_h0: rho_a, slope: (rho_b - rho_a) / (h_b - h_a) })
        }
        _ => None,
    }
}

/// The power-basis coefficients in s of the polynomial of degree n through
/// `values` sampled at s = i / n, i = 0..=n (Newton forward differences).
fn power_coefficients(values: &[f64]) -> Vec<f64> {
    let n = values.len() - 1;
    let mut differences = values.to_vec();
    let mut leading = vec![values[0]];
    for k in 1..=n {
        for i in 0..=n - k {
            differences[i] = differences[i + 1] - differences[i];
        }
        leading.push(differences[0]);
    }
    // f = Σ Δᵏf₀ · x(x − 1)…(x − k + 1) / k!, with x = n·s.
    let mut in_x = vec![0.0; n + 1];
    let mut falling = vec![1.0];
    let mut factorial = 1.0;
    for (k, delta) in leading.iter().enumerate() {
        if k > 0 {
            factorial *= k as f64;
            let mut next = vec![0.0; falling.len() + 1];
            for (j, c) in falling.iter().enumerate() {
                next[j + 1] += c;
                next[j] -= c * (k - 1) as f64;
            }
            falling = next;
        }
        for (j, c) in falling.iter().enumerate() {
            in_x[j] += delta * c / factorial;
        }
    }
    let mut power = 1.0;
    for c in &mut in_x {
        *c *= power;
        power *= n as f64;
    }
    in_x
}

/// Lane O row 27 (2026-10-30): the closed-form Newton seeds of a NURBS curve
/// of degree two, a rational arc included, against a QUADRIC carrier
/// (a sphere, a cylinder or a cone; `quadric_of`). Row 16's reading one degree
/// up: the quadric's equation in the curve's homogeneous point, F(w·C, w), is
/// a plain polynomial of degree 2·degree on each knot span, so it is read
/// EXACTLY from 2·degree + 1 samples per span and its real roots solved there
/// (`real_roots`, degree ≤ 4). A span lying on the carrier, a double root (a
/// grazing arc) or a non-finite reading declines the whole curve, and the
/// general search answers; a span whose reading has no real root seeds
/// nothing, which is how an arc that misses the carrier skips the general
/// search's 32 starts. `tolerance` keeps a root up to ten tolerances past
/// either end of a span, clamped to the curve, as the plane lane does.
pub(crate) fn exact_curve_quadric_seeds(
    curve: &NurbsCurve,
    surface: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<f64>> {
    // A straight edge is row 15's (`exact_line_surface_seeds`); where that
    // lane declines a line, the general search answers.
    if curve.degree != 2 {
        return None;
    }
    curve_quadric_seeds(curve, quadric_of(surface.analytic()?)?, tolerance)
}

/// Lane O row 40: [`exact_curve_quadric_seeds`] against the sphere of `center`
/// and `radius`, which a curve lying in a circle's plane crosses exactly where
/// it crosses that circle.
pub(crate) fn exact_curve_sphere_seeds(curve: &NurbsCurve, center: Vec3, radius: f64, tolerance: f64) -> Option<Vec<f64>> {
    if curve.degree != 2 || !(radius > 0.0 && radius.is_finite()) {
        return None;
    }
    curve_quadric_seeds(curve, Quadric::Sphere { center, radius }, tolerance)
}

fn curve_quadric_seeds(curve: &NurbsCurve, quadric: Quadric<'_>, tolerance: f64) -> Option<Vec<f64>> {
    let degree = curve.degree;
    let [d0, d1] = curve.domain().ok()?;
    let mut breaks: Vec<f64> = curve.knots.iter().copied().filter(|&k| k >= d0 && k <= d1).collect();
    breaks.dedup();
    let reading = |t: f64| -> Option<f64> {
        let h = curve.evaluate_homogeneous(t).ok()?;
        let weighted = Vec3::new(h.x, h.y, h.z);
        let value = match &quadric {
            Quadric::Sphere { center, radius } => {
                let q = weighted.sub(center.scale(h.w));
                q.dot(q) - radius * radius * h.w * h.w
            }
            Quadric::Cone { frame, h_reference, rho_at_h0, slope } => {
                let q = weighted.sub(frame.origin.scale(h.w));
                let axial = q.dot(frame.axis);
                let radial = q.sub(frame.axis.scale(axial));
                let rho = h.w * rho_at_h0 + slope * (axial - h.w * h_reference);
                radial.dot(radial) - rho * rho
            }
        };
        value.is_finite().then_some(value)
    };
    // The size a reading of a point near the curve takes, for the decline bars.
    let (origin, radius) = match &quadric {
        Quadric::Sphere { center, radius } => (*center, radius.abs()),
        Quadric::Cone { frame, rho_at_h0, .. } => (frame.origin, rho_at_h0.abs()),
    };
    let extent = curve.control_points.iter().filter_map(|p| p.point().ok()).fold(radius, |m, p| m.max(p.sub(origin).length()));
    let weight_scale = curve.control_points.iter().fold(0.0_f64, |m, p| m.max(p.w.abs()));
    let reference = (weight_scale * (1.0 + extent)).powi(2);
    let mut seeds = Vec::new();
    for span in breaks.windows(2) {
        let (k0, k1) = (span[0], span[1]);
        if !(k1 > k0) {
            continue;
        }
        let samples = 2 * degree;
        let values = (0..=samples).map(|i| reading(k0 + (k1 - k0) * i as f64 / samples as f64)).collect::<Option<Vec<_>>>()?;
        let mut coefficients = power_coefficients(&values);
        let largest = coefficients.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        let scale = values.iter().fold(0.0_f64, |m, v| m.max(v.abs())).max(largest);
        if !(largest > DEGENERATE * reference) {
            // The span lies on the carrier (or reads nothing): decline.
            return None;
        }
        while coefficients.len() > 2 && coefficients.last().is_some_and(|c| c.abs() <= DEGENERATE * scale) {
            coefficients.pop();
        }
        if coefficients.len() == 2 && coefficients[1].abs() <= DEGENERATE * scale {
            // Constant and nonzero over the span: no root here.
            continue;
        }
        let span_chord = curve.evaluate(k0).ok()?.sub(curve.evaluate(k1).ok()?).length().max(f64::MIN_POSITIVE);
        let slack = (10.0 * tolerance / span_chord).max(0.0);
        for root in real_roots(&coefficients)? {
            if !(root.is_finite() && root >= -slack && root <= 1.0 + slack) {
                continue;
            }
            seeds.push((k0 + root * (k1 - k0)).clamp(d0, d1));
        }
        // A graze: the curve touches the carrier within the caller's
        // tolerance without the reading crossing zero (a near-double root
        // whose discriminant fell below zero), which the general search
        // reports and the roots alone miss (lane O row 40 census, records 6
        // and 7). Like row 28's line-conic seeds, the span's stationary
        // points and its ends are seeded when the curve point lies within a
        // hundred tolerances of the carrier. A stationary point the solver
        // declines (a repeated one) leaves this span to its roots.
        let derivative: Vec<f64> = coefficients.iter().enumerate().skip(1).map(|(i, c)| i as f64 * c).collect();
        let stationary = if derivative.len() >= 2 { real_roots(&derivative).unwrap_or_default() } else { Vec::new() };
        for candidate in stationary.into_iter().chain([0.0, 1.0]) {
            if !(candidate.is_finite() && candidate >= -slack && candidate <= 1.0 + slack) {
                continue;
            }
            let t = (k0 + candidate * (k1 - k0)).clamp(d0, d1);
            let Ok(point) = curve.evaluate(t) else { continue };
            let graze = match &quadric {
                Quadric::Sphere { center, radius } => (point.sub(*center).length() - radius).abs(),
                Quadric::Cone { frame, h_reference, rho_at_h0, slope } => {
                    let q = point.sub(frame.origin);
                    let axial = q.dot(frame.axis);
                    let radial = q.sub(frame.axis.scale(axial)).length();
                    let rho = rho_at_h0 + slope * (axial - h_reference);
                    (radial - rho.abs()).abs()
                }
            };
            if graze <= 100.0 * tolerance {
                seeds.push(t);
            }
        }
    }
    seeds.sort_by(f64::total_cmp);
    seeds.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (1.0 + b.abs()));
    Some(seeds)
}

/// Lane O row 28 (2026-10-30): the closed-form Newton seeds, as parameters of
/// `conic` (a planar curve of degree two, a rational arc included), of its
/// crossings with the straight segment `p0 → p1`. A point of the line lies in
/// the plane through the line that contains the conic plane's normal n, whose
/// own normal is m = n × (p1 − p0); so every crossing is a root of the
/// weighted reading g(t) = m·(w·C(t)) − (m·p0)·w(t), a quadratic on each knot
/// span read EXACTLY from three samples (row 16's plane reading). That plane
/// also holds a crossing of a line that pierces the conic's plane, so the
/// roots seed both a coplanar line and a transverse one.
///
/// Beside the roots, a point where the conic only GRAZES the line within the
/// caller's tolerance must be seeded too, since the general search reports
/// it: each span's stationary point of g, and each knot of the domain, whose
/// in-plane distance |g| / w is within a hundred tolerances (the distance to
/// the line in 3D is never smaller). The span quadratic's roots are solved in
/// closed form, so a graze never declines the curve. Roots up to ten
/// tolerances past a span's ends are kept and clamped to the curve, as the
/// plane lane does.
///
/// Declines (`None`, the general search answers) for a curve that is not of
/// degree two with positive finite weights, control points that are collinear
/// or out of one plane by more than 1e-12 of their extent, a segment within a
/// milliradian of the conic plane's normal (its trace in that plane is a
/// point), a span lying on the reading's plane, or a non-finite reading.
pub(crate) fn exact_line_conic_seeds(p0: Vec3, p1: Vec3, conic: &NurbsCurve, tolerance: f64) -> Option<Vec<f64>> {
    if conic.degree != 2 || conic.control_points.len() < 3 {
        return None;
    }
    if conic.control_points.iter().any(|p| !(p.w > 0.0 && p.w.is_finite())) {
        return None;
    }
    let points = conic.control_points.iter().map(|p| p.point().ok()).collect::<Option<Vec<_>>>()?;
    let base = points[0];
    let extent = points.iter().fold(0.0_f64, |m, p| m.max(p.sub(base).length()));
    if !(extent > 0.0 && extent.is_finite()) {
        return None;
    }
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    let mut area = 0.0_f64;
    for i in 1..points.len() {
        for j in i + 1..points.len() {
            let cross = points[i].sub(base).cross(points[j].sub(base));
            if cross.length() > area {
                area = cross.length();
                normal = cross;
            }
        }
    }
    if !(area > DEGENERATE * extent * extent) {
        return None;
    }
    let normal = normal.scale(1.0 / area);
    if points.iter().any(|p| normal.dot(p.sub(base)).abs() > 1e-12 * extent) {
        return None;
    }
    let direction = p1.sub(p0);
    let length = direction.length();
    if !(length > 0.0 && length.is_finite()) {
        return None;
    }
    let across = normal.cross(direction);
    if !(across.length() > 1e-3_f64.sin() * length) {
        return None;
    }
    let m = across.scale(1.0 / across.length());
    let offset = m.dot(p0);
    let reading = |t: f64| -> Option<(f64, f64)> {
        let h = conic.evaluate_homogeneous(t).ok()?;
        let value = m.x * h.x + m.y * h.y + m.z * h.z - offset * h.w;
        (value.is_finite() && h.w > 0.0).then_some((value, h.w))
    };
    let band = 100.0 * tolerance;
    let [d0, d1] = conic.domain().ok()?;
    let mut breaks: Vec<f64> = conic.knots.iter().copied().filter(|&k| k >= d0 && k <= d1).collect();
    breaks.dedup();
    let weight_scale = conic.control_points.iter().fold(0.0_f64, |m, p| m.max(p.w));
    let reach = points.iter().fold(0.0_f64, |m, p| m.max(p.sub(p0).length()));
    let reference = weight_scale * (1.0 + reach);
    let mut seeds = Vec::new();
    for &knot in &breaks {
        let (value, weight) = reading(knot)?;
        if value.abs() <= band * weight {
            seeds.push(knot);
        }
    }
    for span in breaks.windows(2) {
        let (k0, k1) = (span[0], span[1]);
        if !(k1 > k0) {
            continue;
        }
        let values = (0..=2).map(|i| reading(k0 + (k1 - k0) * i as f64 / 2.0).map(|(value, _)| value)).collect::<Option<Vec<_>>>()?;
        let mut coefficients = power_coefficients(&values);
        let largest = coefficients.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
        let scale = values.iter().fold(0.0_f64, |m, v| m.max(v.abs())).max(largest);
        if !(largest > DEGENERATE * reference) {
            // The span lies on the reading's plane (or reads nothing): decline.
            return None;
        }
        let span_chord = conic.evaluate(k0).ok()?.sub(conic.evaluate(k1).ok()?).length().max(f64::MIN_POSITIVE);
        let slack = (10.0 * tolerance / span_chord).max(0.0);
        let inside = |root: f64| root.is_finite() && root >= -slack && root <= 1.0 + slack;
        if coefficients[2] != 0.0 {
            let stationary = -coefficients[1] / (2.0 * coefficients[2]);
            if inside(stationary) {
                let t = (k0 + stationary * (k1 - k0)).clamp(d0, d1);
                let (value, weight) = reading(t)?;
                if value.abs() <= band * weight {
                    seeds.push(t);
                }
            }
        }
        while coefficients.len() > 2 && coefficients.last().is_some_and(|c| c.abs() <= DEGENERATE * scale) {
            coefficients.pop();
        }
        if coefficients.len() == 2 && coefficients[1].abs() <= DEGENERATE * scale {
            // Constant and nonzero over the span: no root here.
            continue;
        }
        // The roots in closed form (`real_roots` would decline a double root
        // anywhere on the real line, and each quarter span of a circle,
        // extended, traces the whole circle). A double or missing root is a
        // graze, which the stationary seed above covers.
        let roots = if coefficients.len() == 2 {
            vec![-coefficients[0] / coefficients[1]]
        } else {
            let (c0, c1, c2) = (coefficients[0], coefficients[1], coefficients[2]);
            let discriminant = c1 * c1 - 4.0 * c2 * c0;
            if !(discriminant >= 0.0) {
                Vec::new()
            } else {
                crate::poly::solve_quadratic(c0, c1, c2)
            }
        };
        for root in roots {
            if inside(root) {
                seeds.push((k0 + root * (k1 - k0)).clamp(d0, d1));
            }
        }
    }
    seeds.sort_by(f64::total_cmp);
    seeds.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * (1.0 + b.abs()));
    Some(seeds)
}

/// The segment a degree-1 curve of two equal-weight control points traces
/// linearly over its clamped knots: (start, end, domain).
fn straight_segment(curve: &NurbsCurve) -> Option<(Vec3, Vec3, [f64; 2])> {
    if curve.degree != 1 || curve.control_points.len() != 2 || curve.knots.len() != 4 {
        return None;
    }
    let k = &curve.knots;
    if k[0] != k[1] || k[2] != k[3] || !(k[1] < k[2]) {
        return None;
    }
    if curve.control_points[0].w != curve.control_points[1].w {
        return None;
    }
    Some((curve.control_points[0].point().ok()?, curve.control_points[1].point().ok()?, [k[1], k[2]]))
}

/// P(s) = p0 + s·d on the plane through `origin` with normal `normal`.
fn plane_roots(p0: Vec3, d: Vec3, origin: Vec3, normal: Vec3) -> Option<Vec<f64>> {
    let denominator = normal.dot(d);
    if denominator.abs() <= DEGENERATE * normal.length() * d.length() {
        return None;
    }
    Some(vec![normal.dot(origin.sub(p0)) / denominator])
}

/// P(s) on the sphere `|P − center| = radius`.
fn sphere_roots(p0: Vec3, d: Vec3, center: Vec3, radius: f64) -> Option<Vec<f64>> {
    let w = p0.sub(center);
    real_roots(&[w.dot(w) - radius * radius, 2.0 * w.dot(d), d.dot(d)])
}

/// P(s) on the surface of revolution about `frame` whose radius is linear in
/// the axial coordinate: ρ(h) = rho_at_h0 + slope·(h − h_reference). A
/// cylinder has slope 0; a cone's apex is where ρ reads 0. The equation is
/// squared, so both nappes are roots: the Newton and the gap filter keep the
/// ones on the carrier.
fn cone_roots(p0: Vec3, d: Vec3, frame: &RevolutionFrame, h_reference: f64, rho_at_h0: f64, slope: f64) -> Option<Vec<f64>> {
    let w = p0.sub(frame.origin);
    let h0 = w.dot(frame.axis);
    let hd = d.dot(frame.axis);
    let a = w.sub(frame.axis.scale(h0));
    let b = d.sub(frame.axis.scale(hd));
    let c0 = rho_at_h0 + slope * (h0 - h_reference);
    let c1 = slope * hd;
    real_roots(&[a.dot(a) - c0 * c0, 2.0 * (a.dot(b) - c0 * c1), b.dot(b) - c1 * c1])
}

/// P(s) on the torus about `frame` whose tube centre circle lies at axial
/// coordinate `h_center`, with `major` and `minor` radii:
/// (|Q|² + R² − r²)² = 4R²(|Q|² − (Q·axis)²), Q = P − (origin + h_center·axis).
fn torus_roots(p0: Vec3, d: Vec3, frame: &RevolutionFrame, h_center: f64, major: f64, minor: f64) -> Option<Vec<f64>> {
    let scale = major.abs().max(minor.abs());
    if !(major.abs() > DEGENERATE * scale) || !(minor > 0.0) {
        return None;
    }
    let w = p0.sub(frame.origin).sub(frame.axis.scale(h_center));
    let a = d.dot(d);
    let b = 2.0 * w.dot(d);
    let c = w.dot(w);
    let h0 = w.dot(frame.axis);
    let hd = d.dot(frame.axis);
    let e = c + major * major - minor * minor;
    let four_r2 = 4.0 * major * major;
    real_roots(&[
        e * e - four_r2 * (c - h0 * h0),
        2.0 * b * e - four_r2 * (b - 2.0 * h0 * hd),
        b * b + 2.0 * a * e - four_r2 * (a - hd * hd),
        2.0 * a * b,
        a * a,
    ])
}

/// A general revolution whose generatrix, in the frame's zero meridian, is
/// one straight span (a cylinder, cone or annulus band) or one circular arc
/// (a torus or sphere band): the corresponding quadratic or quartic.
fn revolution_roots(p0: Vec3, d: Vec3, frame: &RevolutionFrame, generatrix: &NurbsCurve) -> Option<Vec<f64>> {
    // A control point in the meridian plane: (signed radial along x_axis, axial).
    let meridian = |point: Vec3| -> Option<(f64, f64)> {
        let offset = point.sub(frame.origin);
        let axial = offset.dot(frame.axis);
        let radial = offset.sub(frame.axis.scale(axial));
        let rho = radial.dot(frame.x_axis);
        let off_plane = radial.dot(frame.y_axis).abs();
        (off_plane <= 1e-9 * (1.0 + rho.abs() + axial.abs())).then_some((rho, axial))
    };
    let controls = &generatrix.control_points;
    match (generatrix.degree, controls.len()) {
        (1, 2) => {
            if controls[0].w != controls[1].w {
                return None;
            }
            let (rho_a, h_a) = meridian(controls[0].point().ok()?)?;
            let (rho_b, h_b) = meridian(controls[1].point().ok()?)?;
            let span = ((rho_b - rho_a).powi(2) + (h_b - h_a).powi(2)).sqrt();
            if !(span > 0.0) {
                return None;
            }
            if (h_b - h_a).abs() <= DEGENERATE * span {
                // An annulus: the plane h = h_a.
                return plane_roots(p0, d, frame.origin.add(frame.axis.scale(h_a)), frame.axis);
            }
            cone_roots(p0, d, frame, h_a, rho_a, (rho_b - rho_a) / (h_b - h_a))
        }
        (2, 3) => {
            let (w0, w1, w2) = (controls[0].w, controls[1].w, controls[2].w);
            if w0 != w2 || !(w0 > 0.0) || !(w1 > 0.0) || !(w1 < w0) {
                return None;
            }
            let a = meridian(controls[0].point().ok()?)?;
            let m = meridian(controls[1].point().ok()?)?;
            let b = meridian(controls[2].point().ok()?)?;
            // The circle's centre: on the perpendicular to the start tangent
            // through the start, and to the end tangent through the end.
            let ta = (m.0 - a.0, m.1 - a.1);
            let tb = (b.0 - m.0, b.1 - m.1);
            let det = ta.0 * tb.1 - ta.1 * tb.0;
            let span = (ta.0 * ta.0 + ta.1 * ta.1).sqrt() * (tb.0 * tb.0 + tb.1 * tb.1).sqrt();
            if !(span > 0.0) || det.abs() <= DEGENERATE * span {
                return None;
            }
            // ta·C = ta·a ; tb·C = tb·b
            let ra = ta.0 * a.0 + ta.1 * a.1;
            let rb = tb.0 * b.0 + tb.1 * b.1;
            let center = ((ra * tb.1 - ta.1 * rb) / det, (ta.0 * rb - ra * tb.0) / det);
            let radius = ((a.0 - center.0).powi(2) + (a.1 - center.1).powi(2)).sqrt();
            let radius_b = ((b.0 - center.0).powi(2) + (b.1 - center.1).powi(2)).sqrt();
            if !(radius > 0.0) || (radius_b - radius).abs() > 1e-9 * radius {
                return None;
            }
            // The shoulder weight of a circular arc is the cosine of half the
            // arc angle, and the arc's midpoint lies on the circle.
            let ca = ((a.0 - center.0) / radius, (a.1 - center.1) / radius);
            let cb = ((b.0 - center.0) / radius, (b.1 - center.1) / radius);
            let cos_angle = (ca.0 * cb.0 + ca.1 * cb.1).clamp(-1.0, 1.0);
            let cos_half = ((1.0 + cos_angle) * 0.5).sqrt();
            if (w1 / w0 - cos_half).abs() > 1e-9 {
                return None;
            }
            let mid_weight = 2.0 * w0 + 2.0 * w1;
            let mid = ((w0 * a.0 + 2.0 * w1 * m.0 + w0 * b.0) / mid_weight, (w0 * a.1 + 2.0 * w1 * m.1 + w0 * b.1) / mid_weight);
            let radius_mid = ((mid.0 - center.0).powi(2) + (mid.1 - center.1).powi(2)).sqrt();
            if (radius_mid - radius).abs() > 1e-9 * radius {
                return None;
            }
            let major = center.0.abs();
            if major <= DEGENERATE * radius {
                return sphere_roots(p0, d, frame.origin.add(frame.axis.scale(center.1)), radius);
            }
            torus_roots(p0, d, frame, center.1, major, radius)
        }
        _ => None,
    }
}


