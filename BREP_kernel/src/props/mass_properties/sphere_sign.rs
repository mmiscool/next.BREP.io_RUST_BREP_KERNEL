//! Conservative flux bound for native sphere supports; never a mass measurement.
use super::*;
use crate::AnalyticSurface;

/// Full sphere flux minus an absolute bound on each interior wire's flux.
/// For a polynomial wire, Green's theorem with primitive
/// G(u,v)=integral(mid_u..u) f(s,v) ds gives
/// |integral(winding*f)| <= sup|f| * width_u/2 * variation_v.
/// The B-spline derivative is a positive partition of control differences,
/// so its total variation is no larger than the control polygon's variation.
/// This holds even for self-crossing wires; no sampled Jordan recognition.
/// Exactly two supports and at most one interior wire per support are the
/// deliberately narrow scope of this first certificate.
pub(super) fn positive_volume_bound(solid: &BrepSolid) -> Option<f64> {
    if solid.shells.len() != 1 || solid.shells[0].faces.len() != 2 {
        return None;
    }
    let shell = &solid.shells[0];
    let reference = super::solid_props::shell_volume_reference(shell).ok()?;
    let mut lower = 0.0;
    let mut magnitude = 0.0;
    for face in &shell.faces {
        if !face.same_sense || face.loops.len() != 2 || !domain_wire(face) {
            return None;
        }
        let actual = &face.surface;
        if actual.degree_u != 2
            || actual.degree_v != 2
            || actual.control_points.len() != 9
            || actual.control_points.iter().any(|row| row.len() != 5)
        {
            return None;
        }
        let AnalyticSurface::Sphere { frame, radius: r } =
            crate::analytic_surface::recognize(&face.surface)?
        else {
            return None;
        };
        if !r.is_finite() || r <= 0.0 {
            return None;
        }
        let ideal =
            crate::make_sphere_surface_framed(frame.origin, r, frame.axis, Some(frame.x_axis))
                .ok()?;
        if actual.knots_u != ideal.knots_u || actual.knots_v != ideal.knots_v {
            return None;
        }
        // Recognition only selects the template. The perturbation envelope
        // below, read afresh from every stored homogeneous control, supplies
        // the proof; a populated analytic cache cannot grant it.
        let mut dx = 0.0f64;
        let mut dw = 0.0f64;
        let mut wmin = 0.5f64;
        for (a, b) in actual
            .control_points
            .iter()
            .flatten()
            .zip(ideal.control_points.iter().flatten())
        {
            if ![a.x, a.y, a.z, a.w, b.x, b.y, b.z, b.w]
                .iter()
                .all(|v| v.is_finite())
            {
                return None;
            }
            let x = Vec3::new(
                a.x - frame.origin.x * a.w,
                a.y - frame.origin.y * a.w,
                a.z - frame.origin.z * a.w,
            );
            let y = Vec3::new(
                b.x - frame.origin.x * b.w,
                b.y - frame.origin.y * b.w,
                b.z - frame.origin.z * b.w,
            );
            dx = dx.max(x.sub(y).length());
            dw = dw.max((a.w - b.w).abs());
            wmin = wmin.min(a.w);
        }
        // Covers constructor trigonometry, frame normalization and subtraction
        // rounding against the mathematical orthonormal sphere template.
        let round = 4096.0 * f64::EPSILON * (1.0 + r + frame.origin.length());
        dx += round;
        dw += 4096.0 * f64::EPSILON;
        if wmin < 0.49 || dx > 1e-8 * r || dw > 1e-8 {
            return None;
        }
        let d = dx + r * dw;
        let dp = d / wmin;
        // Quadratic knot-span derivative controls have multipliers <=16
        // (u) and <=8 (v). Applying the rational quotient rule twice,
        // with ideal homogeneous position <=r and weight derivative <=16,
        // is bounded by 1000*d/wmin^2 in either direction.
        let dt = 1000.0 * d / (wmin * wmin);
        // Quarter-circle parameter speed <7; polar half-circle speed <4.
        let reach = r + frame.origin.sub(reference).length();
        let error =
            (dp * (7.0 * r + dt) * (4.0 * r + dt) + reach * (11.0 * r * dt + dt * dt)) / 3.0;
        let flux = reach * 28.0 * r * r / 3.0 + error;
        let wire = &face.loops[1];
        if wire.coedges.is_empty() {
            return None;
        }
        let mut umin = 1.0f64;
        let mut umax = 0.0f64;
        let mut variation = 0.0;
        let mut ends = Vec::new();
        for coedge in &wire.coedges {
            let c = &coedge.pcurve;
            if NurbsCurve::new(c.degree, c.knots.clone(), c.control_points.clone()).is_err()
                || c.knots.windows(2).any(|k| k[1] < k[0])
                || c.control_points.iter().any(|p| p.w != 1.0)
            {
                return None;
            }
            let dom = c.domain().ok()?;
            if c.knots[..=c.degree].iter().any(|t| *t != dom[0])
                || c.knots[c.knots.len() - 1 - c.degree..]
                    .iter()
                    .any(|t| *t != dom[1])
            {
                return None;
            }
            let (mut previous, mut multiplicity) = (f64::NAN, 0usize);
            for &knot in &c.knots {
                multiplicity = if knot == previous {
                    multiplicity + 1
                } else {
                    1
                };
                previous = knot;
                if knot > dom[0] && knot < dom[1] && multiplicity > c.degree {
                    return None;
                }
            }
            for p in &c.control_points {
                if ![p.x, p.y, p.z].iter().all(|v| v.is_finite())
                    || p.x <= 0.0
                    || p.x >= 1.0
                    || p.y <= 0.0
                    || p.y >= 1.0
                    || p.z != 0.0
                {
                    return None;
                }
                umin = umin.min(p.x);
                umax = umax.max(p.x);
            }
            // Outward rounding keeps the variation bound sound independently
            // of the number of controls; the constant final reserve need not
            // cover a long, downward-rounded summation.
            for pair in c.control_points.windows(2) {
                variation = (variation + (pair[1].y - pair[0].y).abs().next_up()).next_up();
            }
            let first = c.control_points[0];
            let last = c.control_points[c.control_points.len() - 1];
            ends.push((
                Vec3::new(first.x, first.y, first.z),
                Vec3::new(last.x, last.y, last.z),
            ));
        }
        for i in 0..ends.len() {
            let gap = ends[i].1.sub(ends[(i + 1) % ends.len()].0);
            if gap.length() > 1e-12 {
                return None;
            }
            variation = (variation + gap.y.abs().next_up()).next_up();
        }
        let wire_area = ((umax - umin).next_up() * variation).next_up() * 0.5;
        let full = 4.0 * std::f64::consts::PI * r * r * r / 3.0;
        let charge = error + flux * wire_area;
        lower += full - charge;
        magnitude += full + charge;
    }
    // Outward rounding reserve for positive arithmetic above (including the
    // control-variation sum). A sign proof needs a wide, positive margin.
    lower -= 1e-9 * magnitude.max(1.0);
    (lower.is_finite() && lower > 0.0).then_some(lower)
}

fn domain_wire(face: &FaceRecord) -> bool {
    let corners = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
    let wire = &face.loops[0];
    if wire.coedges.len() != 4 {
        return false;
    }
    wire.coedges.iter().enumerate().all(|(i, c)| {
        let p = &c.pcurve;
        p.degree == 1
            && p.control_points.len() == 2
            && p.knots.len() == 4
            && p.knots[0] == p.knots[1]
            && p.knots[2] == p.knots[3]
            && p.knots[2] > p.knots[1]
            && NurbsCurve::new(p.degree, p.knots.clone(), p.control_points.clone()).is_ok()
            && p.control_points.iter().all(|v| v.w == 1.0 && v.z == 0.0)
            && (p.control_points[0].x, p.control_points[0].y) == corners[i]
            && (p.control_points[1].x, p.control_points[1].y) == corners[(i + 1) % 4]
    })
}

