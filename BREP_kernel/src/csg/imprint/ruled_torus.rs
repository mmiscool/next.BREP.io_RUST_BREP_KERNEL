//! Constant axial ruling × certified circular meridian of a reconstructed revolution.
//!
//! This supplies a proposal only. The assembler remains responsible for settled
//! endpoint agreement, both final coedge uses and nonincident material clearance.
use crate::analytic_surface::circle_angle_to_parameter;
use crate::{AnalyticSurface, NurbsCurve, NurbsSurface, RevolutionFrame, Vec3, Vec4};

#[derive(Debug)]
pub(crate) struct RuledTorusBoundary {
    pub boundary: NurbsCurve,
    pub own_pcurve: NurbsCurve,
    pub partner_pcurve: NurbsCurve,
    /// Numerical off-node witnesses, not a continuous interval certificate.
    pub maximum_image_residual: f64,
    pub maximum_displacement: f64,
    pub segments: usize,
}

struct Profile<'a> {
    frame: &'a RevolutionFrame,
    curve: &'a NurbsCurve,
    spans: usize,
    sweep: f64,
    major: f64,
    axial: f64,
    minor: f64,
    domain: [f64; 2],
    radial_ends: [f64; 2],
    sign: f64,
    angular_curve: NurbsCurve,
    deviation: f64,
}

fn xyz(p: Vec4) -> Vec3 {
    Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w)
}
fn homogeneous(p: Vec3) -> Vec4 {
    Vec4 {
        x: p.x,
        y: p.y,
        z: p.z,
        w: 1.0,
    }
}
fn meridian(frame: &RevolutionFrame, p: Vec3) -> (f64, f64) {
    let d = p.sub(frame.origin);
    (d.dot(frame.x_axis), d.dot(frame.axis))
}

/// The positive Bernstein denominator is >= min(weight). Normalize before
/// products, then convert the complete quartic identity to a physical radial
/// deviation bound: |distance-radius| <= radius*max(coefficient)/minweight².
fn certify_circle(
    curve: &NurbsCurve,
    center: Vec3,
    radius: f64,
    reserve: f64,
) -> Result<f64, String> {
    let maxw = curve.control_points.iter().map(|p| p.w).fold(0.0, f64::max);
    if !radius.is_finite() || radius <= 0.0 || !maxw.is_finite() || maxw <= 0.0 {
        return Err("ruled-torus: nonfinite circular proof scale".into());
    }
    let h: Vec<_> = curve
        .control_points
        .iter()
        .map(|p| {
            let w = p.w / maxw;
            (xyz(*p).sub(center).scale(1.0 / radius).scale(w), w)
        })
        .collect();
    let minw = h.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    if !minw.is_finite() || minw < 1e-6 {
        return Err("ruled-torus: circular proof denominator is ill-conditioned".into());
    }
    let bin2 = [1.0, 2.0, 1.0];
    let bin4 = [1.0, 4.0, 6.0, 4.0, 1.0];
    let mut maximum: f64 = 0.0;
    for k in 0..5 {
        let mut coefficient = 0.0;
        for i in 0..3 {
            for j in 0..3 {
                if i + j == k {
                    let product = h[i].0.dot(h[j].0) - h[i].1 * h[j].1;
                    if !product.is_finite() {
                        return Err("ruled-torus: nonfinite circular identity product".into());
                    }
                    coefficient += bin2[i] * bin2[j] / bin4[k] * product;
                }
            }
        }
        if !coefficient.is_finite() {
            return Err("ruled-torus: nonfinite circular identity coefficient".into());
        }
        maximum = maximum.max(coefficient.abs());
    }
    let deviation = radius * maximum / (minw * minw);
    if !deviation.is_finite() || deviation > reserve {
        return Err(
            "ruled-torus: whole-span circular identity failed physical denominator bound".into(),
        );
    }
    Ok(deviation)
}

impl<'a> Profile<'a> {
    fn recognize(surface: &'a NurbsSurface, tolerance: f64) -> Result<Self, String> {
        let Some(AnalyticSurface::Revolution {
            frame,
            spans,
            sweep,
            generatrix: curve,
        }) = surface.analytic()
        else {
            return Err("ruled-torus: partner is not a reconstructed general revolution".into());
        };
        let domain = curve.domain()?;
        if curve.degree != 2
            || curve.control_points.len() != 3
            || curve.knots
                != vec![
                    domain[0], domain[0], domain[0], domain[1], domain[1], domain[1],
                ]
            || curve
                .control_points
                .iter()
                .any(|p| !p.w.is_finite() || p.w <= 0.0)
        {
            return Err("ruled-torus: requires a positive single quadratic meridian".into());
        }
        let a = curve.evaluate(domain[0])?;
        let b = curve.evaluate((domain[0] + domain[1]) * 0.5)?;
        let c = curve.evaluate(domain[1])?;
        let ab = b.sub(a);
        let ac = c.sub(a);
        let n = ab.cross(ac);
        if n.dot(n) <= 1e-28 * ab.dot(ab) * ac.dot(ac) {
            return Err("ruled-torus: degenerate circular profile".into());
        }
        let center = a.add(
            n.cross(ab)
                .scale(ac.dot(ac))
                .add(ac.cross(n).scale(ab.dot(ab)))
                .scale(0.5 / n.dot(n)),
        );
        let (major, axial) = meridian(frame, center);
        let minor = a.sub(center).length();
        let scale = major.abs().max(minor).max(1.0);
        let proof = 2e-12 * scale;
        if !minor.is_finite()
            || !major.is_finite()
            || !(scale * scale).is_finite()
            || minor <= proof
            || major <= minor + proof
        {
            return Err("ruled-torus: pole or spindle profile unsupported".into());
        }
        // A whole-profile physical bound, with a positive denominator lower
        // bound and explicit normalization, independent of homogeneous scale.
        let deviation = certify_circle(curve, center, minor, (tolerance * 0.05).min(proof))?;
        let mut coords = Vec::new();
        for p in &curve.control_points {
            let q = xyz(*p);
            let d = q.sub(frame.origin);
            if d.dot(frame.y_axis).abs() > proof {
                return Err("ruled-torus: profile leaves meridian plane".into());
            }
            coords.push(meridian(frame, q));
        }
        // Ordered control radii with positive weights certify radial monotonicity.
        let direction = (coords[2].0 - coords[0].0).signum();
        let sign = (b.sub(center).dot(frame.axis)).signum();
        if direction == 0.0
            || sign == 0.0
            || coords
                .windows(2)
                .any(|q| direction * (q[1].0 - q[0].0) < -proof)
            || coords.iter().any(|q| sign * (q.1 - axial) < -proof)
        {
            return Err("ruled-torus: turning or multiple axial branches".into());
        }
        let angular_curve =
            crate::make_arc(frame.origin, frame.x_axis, frame.y_axis, 1.0, 0.0, *sweep)?;
        Ok(Self {
            frame,
            curve,
            spans: *spans,
            sweep: *sweep,
            major,
            axial,
            minor,
            domain,
            radial_ends: [coords[0].0, coords[2].0],
            sign,
            angular_curve,
            deviation,
        })
    }

    /// Invert the monotone radial coordinate; outside the finite profile use
    /// the native endpoint TANGENT, never an ideal-circle continuation.
    fn inverse(&self, rho: f64, extension_limit: f64) -> Result<(f64, f64), String> {
        let [r0, r1] = self.radial_ends;
        let direction = (r1 - r0).signum();
        for (index, radius) in [r0, r1].into_iter().enumerate() {
            if (index == 0 && direction * (rho - radius) < 0.0)
                || (index == 1 && direction * (rho - radius) > 0.0)
            {
                let v = self.domain[index];
                let d = self.curve.derivatives(v, 1)?;
                let (rd, ad) = (d[1].dot(self.frame.x_axis), d[1].dot(self.frame.axis));
                if rd.abs() <= 1e-10 * self.minor / (self.domain[1] - self.domain[0]) {
                    return Err("ruled-torus: radial turning endpoint extension".into());
                }
                let dv = (rho - radius) / rd;
                if d[1].length() * dv.abs() > extension_limit {
                    return Err("ruled-torus: native extension exceeds displacement budget".into());
                }
                return Ok((v + dv, meridian(self.frame, d[0]).1 + dv * ad));
            }
        }
        let radicand = self.minor * self.minor - (rho - self.major).powi(2);
        if radicand <= 1e-14 * self.minor * self.minor {
            return Err("ruled-torus: axial square-root turning point".into());
        }
        let axial = self.axial + self.sign * radicand.sqrt();
        let mut lo = self.domain[0];
        let mut hi = self.domain[1];
        for _ in 0..55 {
            let mid = (lo + hi) * 0.5;
            let r = meridian(self.frame, self.curve.evaluate(mid)?).0;
            if direction * (r - rho) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let v = (lo + hi) * 0.5;
        let actual = meridian(self.frame, self.curve.evaluate(v)?);
        if (actual.0 - rho).abs().max((actual.1 - axial).abs()) > 2e-11 * self.minor.max(1.0) {
            return Err("ruled-torus: circular inverse failed independent profile image".into());
        }
        Ok((v, axial))
    }

    fn image(&self, u: f64, v: f64) -> Result<Vec3, String> {
        // Independent of the surface's corner extension derivative table.
        let (r, a) = meridian(self.frame, self.curve.evaluate_extended(v)?);
        // Get the angular rational map from the represented u-isocurve, not
        // by pretending rational circle parameter is linear in angle.
        let direction = self
            .angular_curve
            .evaluate(u)?
            .sub(self.frame.origin)
            .normalized()?;
        Ok(self
            .frame
            .origin
            .add(direction.scale(r))
            .add(self.frame.axis.scale(a)))
    }
}

/// Read-only material data from the SAME whole-profile proof used by the
/// section constructor. No circle, denominator or native-extension guard is
/// weakened; the actual quartic deviation bound is retained for evidence.
pub(crate) struct CircularRevolutionProfile<'a> {
    pub frame: &'a RevolutionFrame,
    pub curve: &'a NurbsCurve,
    pub angular_curve: NurbsCurve,
    pub major: f64,
    pub axial: f64,
    pub minor: f64,
    pub domain: [f64; 2],
    pub deviation: f64,
}
pub(crate) fn circular_revolution_profile(
    surface: &NurbsSurface,
    tolerance: f64,
) -> Result<CircularRevolutionProfile<'_>, String> {
    if !tolerance.is_finite() || tolerance <= 0.0 || tolerance > 1e-8 {
        return Err("ruled-torus: invalid material profile tolerance".into());
    }
    let p = Profile::recognize(surface, tolerance)?;
    Ok(CircularRevolutionProfile {
        frame: p.frame,
        curve: p.curve,
        angular_curve: p.angular_curve,
        major: p.major,
        axial: p.axial,
        minor: p.minor,
        domain: p.domain,
        deviation: p.deviation,
    })
}

/// Construct a precision section on [0,1]. `own_u` is the settled longitudinal
/// interval (either orientation). `inherited_own` is a UV trim used only to
/// reject excessive displacement, including wrong branch selection. It is
/// never used to generate intersection points. The physical tolerance must be
/// <=1e-8, reserving at least two orders below the stored 1e-6 joint bar.
///
/// Supported own nets have degree-v=1, two columns, equal positive weights in
/// each ruling, and a constant axis-parallel displacement over EVERY row.
/// The partial circular meridian must be single-span and radially monotone.
/// Errors explicitly decline unsupported data or exhausted construction work.
/// Witness bounds are numerical; downstream clearance and endpoint guards
/// remain mandatory. Native angular extrapolation and sweep wrapping decline.
pub(crate) fn construct_ruled_torus(
    own: &NurbsSurface,
    partner: &NurbsSurface,
    own_u: [f64; 2],
    inherited_own: &NurbsCurve,
    max_displacement: f64,
    tolerance: f64,
) -> Result<RuledTorusBoundary, String> {
    if !tolerance.is_finite()
        || tolerance <= 0.0
        || tolerance > 1e-8
        || !max_displacement.is_finite()
        || max_displacement <= 0.0
    {
        return Err("ruled-torus: invalid precision or displacement budget".into());
    }
    let profile = Profile::recognize(partner, tolerance)?;
    let ud = own.domain_u()?;
    let vd = own.domain_v()?;
    if own.degree_u > 8
        || inherited_own.degree > 8
        || own.knots_u.len() > 4096
        || inherited_own.knots.len() > 4096
    {
        return Err("ruled-torus: input work budget exceeded".into());
    }
    if own.degree_v != 1
        || own.control_points.iter().any(|r| r.len() != 2)
        || own.knots_v != vec![vd[0], vd[0], vd[1], vd[1]]
        || own_u
            .iter()
            .any(|u| !u.is_finite() || *u < ud[0] || *u > ud[1])
        || own_u[0] == own_u[1]
    {
        return Err("ruled-torus: unsupported ruling net or longitudinal range".into());
    }
    let rows = &own.control_points;
    if rows
        .iter()
        .flatten()
        .any(|p| !p.w.is_finite() || p.w <= 0.0)
        || rows.iter().any(|r| r[0].w != r[1].w)
    {
        return Err("ruled-torus: distinct or nonpositive ruling weights".into());
    }
    let delta = xyz(rows[0][1]).sub(xyz(rows[0][0]));
    let length = delta.length();
    if !length.is_finite()
        || length <= 1e-12
        || delta.cross(profile.frame.axis).length() > 2e-12 * length
        || rows
            .iter()
            .any(|r| xyz(r[1]).sub(xyz(r[0])).sub(delta).length() > 2e-12 * length)
    {
        return Err("ruled-torus: nonconstant or nonaxial ruling".into());
    }
    let axial_delta = delta.dot(profile.frame.axis);
    let hint_domain = inherited_own.domain()?;
    for p in &inherited_own.control_points {
        let q = xyz(*p);
        if !q.x.is_finite() || !q.y.is_finite() || !q.z.is_finite() {
            return Err("ruled-torus: inherited guide has nonfinite Euclidean controls".into());
        }
        if q.x < ud[0] || q.x > ud[1] {
            return Err("ruled-torus: inherited guide leaves longitudinal chart".into());
        }
    }
    let hint_start = inherited_own.evaluate(hint_domain[0])?.x;
    let hint_end = inherited_own.evaluate(hint_domain[1])?.x;
    let hint_direction = (hint_end - hint_start).signum();
    if hint_direction == 0.0
        || inherited_own.control_points.iter().any(|p| p.w <= 0.0)
        || inherited_own
            .control_points
            .windows(2)
            .any(|p| hint_direction * (p[1].x / p[1].w - p[0].x / p[0].w) < 0.0)
    {
        return Err("ruled-torus: inherited longitudinal guide is not monotone".into());
    }
    // Compare at equal OWN u, not equal legacy curve parameter. Old station
    // spacing and biased interior v affect only this rejection guard.
    let hint_at_u = |u: f64| -> Result<Vec3, String> {
        let target = u.clamp(hint_start.min(hint_end), hint_start.max(hint_end));
        let mut lo = hint_domain[0];
        let mut hi = hint_domain[1];
        for _ in 0..45 {
            let mid = (lo + hi) * 0.5;
            if hint_direction * (inherited_own.evaluate(mid)?.x - target) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let uv = inherited_own.evaluate((lo + hi) * 0.5)?;
        // Exact affine ruling image, including v-extension. This rejection
        // guard also avoids any dependence on native corner derivatives.
        Ok(own
            .evaluate(uv.x, vd[0])?
            .add(delta.scale((uv.y - vd[0]) / (vd[1] - vd[0]))))
    };
    let station = |t: f64| -> Result<[Vec3; 3], String> {
        let u = own_u[0] + t * (own_u[1] - own_u[0]);
        let base = own.evaluate(u, vd[0])?;
        let d = base.sub(profile.frame.origin);
        let a0 = d.dot(profile.frame.axis);
        let radial = d.sub(profile.frame.axis.scale(a0));
        let rho = radial.length();
        if rho <= 1e-10 * profile.major {
            return Err("ruled-torus: own station reaches revolution pole".into());
        }
        let mut angle = radial
            .dot(profile.frame.y_axis)
            .atan2(radial.dot(profile.frame.x_axis));
        if angle < 0.0 {
            angle += std::f64::consts::TAU;
        }
        // An angular seam is valid only on the represented side, never wrapped
        // onto the other sweep endpoint or extended in two coordinates.
        if angle > profile.sweep || angle < 0.0 {
            return Err("ruled-torus: station outside angular sweep".into());
        }
        let pu = circle_angle_to_parameter(profile.spans, profile.sweep, angle);
        let (pv, axial) = profile.inverse(rho, max_displacement)?;
        let v = vd[0] + (vd[1] - vd[0]) * (axial - a0) / axial_delta;
        if v < vd[0] || v > vd[1] {
            return Err("ruled-torus: intersection outside own ruling range".into());
        }
        let point = base.add(delta.scale((v - vd[0]) / (vd[1] - vd[0])));
        let station_error = point.sub(profile.image(pu, pv)?).length();
        if !station_error.is_finite() || station_error > tolerance * 0.1 {
            return Err("ruled-torus: independent analytic station images disagree".into());
        }
        let movement = point.sub(hint_at_u(u)?).length();
        if !movement.is_finite() || movement > max_displacement {
            return Err("ruled-torus: inherited branch displacement exceeded".into());
        }
        Ok([point, Vec3::new(u, v, 0.0), Vec3::new(pu, pv, 0.0)])
    };
    let mut accepted = Vec::new();
    let mut breaks = vec![0.0, 1.0];
    for &knot in &own.knots_u {
        let t = (knot - own_u[0]) / (own_u[1] - own_u[0]);
        if t > 0.0 && t < 1.0 {
            breaks.push(t);
        }
    }
    // Include every guide span in the displacement census as well.
    for &knot in &inherited_own.knots {
        if knot >= hint_domain[0] && knot <= hint_domain[1] {
            let u = inherited_own.evaluate(knot)?.x;
            let t = (u - own_u[0]) / (own_u[1] - own_u[0]);
            if t > 0.0 && t < 1.0 {
                breaks.push(t);
            }
        }
    }
    breaks.sort_by(f64::total_cmp);
    breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-13);
    let mut pending: Vec<_> = breaks
        .windows(2)
        .rev()
        .map(|b| (b[0], b[1], 0usize))
        .collect();
    let mut maximum_image_residual: f64 = 0.0;
    let mut maximum_displacement: f64 = 0.0;
    let mut work = 0usize;
    while let Some((a, b, depth)) = pending.pop() {
        work += 1;
        if work > 8191 || depth > 24 {
            return Err("ruled-torus: bounded refinement exhausted".into());
        }
        let samples = [
            station(a)?,
            station(a + (b - a) / 3.0)?,
            station(a + 2.0 * (b - a) / 3.0)?,
            station(b)?,
        ];
        let mut curves = Vec::new();
        for k in 0..3 {
            let p: Vec<_> = samples.iter().map(|s| s[k]).collect();
            let c1 = p[0]
                .scale(-5.0 / 6.0)
                .add(p[1].scale(3.0))
                .add(p[2].scale(-1.5))
                .add(p[3].scale(1.0 / 3.0));
            let c2 = p[0]
                .scale(1.0 / 3.0)
                .add(p[1].scale(-1.5))
                .add(p[2].scale(3.0))
                .add(p[3].scale(-5.0 / 6.0));
            curves.push(NurbsCurve::new(
                3,
                vec![a, a, a, a, b, b, b, b],
                vec![
                    homogeneous(p[0]),
                    homogeneous(c1),
                    homogeneous(c2),
                    homogeneous(p[3]),
                ],
            )?);
        }
        let mut residual: f64 = 0.0;
        let mut movement: f64 = 0.0;
        for q in [
            0.0,
            0.0198550717512319,
            0.101666761293187,
            0.237233795041836,
            0.408282678752175,
            0.5,
            0.591717321247825,
            0.762766204958164,
            0.898333238706813,
            0.980144928248768,
            1.0,
        ] {
            let t = a + (b - a) * q;
            let exact = station(t)?;
            let edge = curves[0].evaluate(t)?;
            let ownuv = curves[1].evaluate(t)?;
            let uv = curves[2].evaluate(t)?;
            if uv.x < 0.0
                || uv.x > 1.0
                || ownuv.x < ud[0]
                || ownuv.x > ud[1]
                || ownuv.y < vd[0]
                || ownuv.y > vd[1]
            {
                residual = f64::INFINITY;
                continue;
            }
            let reference = profile.image(uv.x, uv.y)?;
            for image in [
                exact[0],
                own.evaluate(ownuv.x, ownuv.y)?,
                reference,
                partner.evaluate_extended(uv.x, uv.y)?,
            ] {
                let error = image.sub(edge).length();
                if !error.is_finite() {
                    return Err("ruled-torus: nonfinite fitted image witness".into());
                }
                residual = residual.max(error);
            }
            let displacement = edge.sub(hint_at_u(exact[1].x)?).length();
            if !displacement.is_finite() {
                return Err("ruled-torus: nonfinite fitted displacement witness".into());
            }
            movement = movement.max(displacement);
        }
        if movement > max_displacement {
            return Err("ruled-torus: fitted boundary exceeds displacement budget".into());
        }
        if residual > tolerance {
            let mid = (a + b) * 0.5;
            pending.push((mid, b, depth + 1));
            pending.push((a, mid, depth + 1));
        } else {
            maximum_image_residual = maximum_image_residual.max(residual);
            maximum_displacement = maximum_displacement.max(movement);
            accepted.push(curves);
        }
    }
    let segments = accepted.len();
    let join = |k: usize| -> Result<NurbsCurve, String> {
        let mut points = Vec::new();
        let mut knots = vec![0.0; 4];
        for (i, c) in accepted.iter().enumerate() {
            if i > 0 {
                knots.extend([c[k].knots[0]; 3]);
            }
            points.extend(
                c[k].control_points
                    .iter()
                    .skip(if i == 0 { 0 } else { 1 })
                    .copied(),
            );
        }
        knots.extend([1.0; 4]);
        NurbsCurve::new(3, knots, points)
    };
    Ok(RuledTorusBoundary {
        boundary: join(0)?,
        own_pcurve: join(1)?,
        partner_pcurve: join(2)?,
        maximum_image_residual,
        maximum_displacement,
        segments,
    })
}

