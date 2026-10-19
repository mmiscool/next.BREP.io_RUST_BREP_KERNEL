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

/// Every real root of the polynomial Σ coefficients[i]·sⁱ (degree ≤ 4), by
/// bisection on the monotonic intervals between the roots of its derivative
/// (found the same way), polished by Newton. `None` when the polynomial is
/// degenerate: a leading coefficient within [`DEGENERATE`] of zero relative to
/// the others, or a critical value within [`DEGENERATE`] of zero relative to
/// the polynomial's terms there — a double or near-double root, where the
/// sampled search's tangency reading must stand.
fn real_roots(coefficients: &[f64]) -> Option<Vec<f64>> {
    let mut coefficients: Vec<f64> = coefficients.to_vec();
    if coefficients.iter().any(|c| !c.is_finite()) {
        return None;
    }
    let largest = coefficients.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if !(largest > 0.0) {
        return None;
    }
    let leading = *coefficients.last()?;
    if leading.abs() <= DEGENERATE * largest {
        return None;
    }
    while coefficients.len() > 1 && coefficients.last() == Some(&0.0) {
        coefficients.pop();
    }
    let degree = coefficients.len() - 1;
    if degree == 0 {
        return Some(Vec::new());
    }
    let evaluate = |s: f64| coefficients.iter().rev().fold(0.0, |acc, c| acc * s + c);
    let terms = |s: f64| coefficients.iter().enumerate().fold(0.0, |acc, (i, c)| acc + (c * s.powi(i as i32)).abs());
    if degree == 1 {
        return Some(vec![-coefficients[0] / coefficients[1]]);
    }
    // Cauchy's bound on the root magnitude.
    let bound = 1.0 + coefficients[..degree].iter().map(|c| (c / leading).abs()).fold(0.0, f64::max);
    let derivative: Vec<f64> = coefficients.iter().enumerate().skip(1).map(|(i, c)| c * i as f64).collect();
    let mut critical = real_roots(&derivative)?;
    critical.retain(|c| c.abs() <= bound);
    critical.sort_by(f64::total_cmp);
    let mut nodes = vec![-bound];
    nodes.extend(critical.iter().copied());
    nodes.push(bound);
    // A critical value at zero is a double root: decline.
    for &c in &critical {
        if evaluate(c).abs() <= DEGENERATE * terms(c).max(largest * DEGENERATE) {
            return None;
        }
    }
    let mut roots = Vec::new();
    for pair in nodes.windows(2) {
        let (mut lo, mut hi) = (pair[0], pair[1]);
        let (mut f_lo, f_hi) = (evaluate(lo), evaluate(hi));
        if f_lo == 0.0 {
            roots.push(lo);
            continue;
        }
        if f_lo.signum() == f_hi.signum() {
            continue;
        }
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            let f_mid = evaluate(mid);
            if f_mid == 0.0 {
                lo = mid;
                hi = mid;
                break;
            }
            if f_mid.signum() == f_lo.signum() {
                lo = mid;
                f_lo = f_mid;
            } else {
                hi = mid;
            }
        }
        let mut root = 0.5 * (lo + hi);
        for _ in 0..3 {
            let slope = derivative.iter().rev().fold(0.0, |acc, c| acc * root + c);
            if slope == 0.0 {
                break;
            }
            let next = root - evaluate(root) / slope;
            if !next.is_finite() || next < pair[0] || next > pair[1] {
                break;
            }
            root = next;
        }
        roots.push(root);
    }
    if let Some(&last) = nodes.last() {
        if evaluate(last) == 0.0 {
            roots.push(last);
        }
    }
    Some(roots)
}

