use super::*;
use super::recognition::{full_circle_knots, nearly_equal_points, surface_scale};

/// Frame, generatrix, and sweep of any surface built by `make_revolution`
/// (full or partial), validated by exact reconstruction. Unlike
/// `AnalyticSurface` this does not require the generatrix to be a line or
/// classic quadric profile.
pub struct RevolutionStructure {
    pub frame: RevolutionFrame,
    pub generatrix: NurbsCurve,
    pub sweep: f64,
}

pub(crate) fn circumcenter(a: Vec3, b: Vec3, c: Vec3) -> Option<Vec3> {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let normal = ab.cross(ac);
    let normal_squared = normal.dot(normal);
    if normal_squared <= 1e-30 {
        return None;
    }
    let offset = normal
        .cross(ab)
        .scale(ac.dot(ac))
        .add(ac.cross(normal).scale(ab.dot(ab)))
        .scale(1.0 / (2.0 * normal_squared));
    Some(a.add(offset))
}

pub fn revolution_structure(surface: &NurbsSurface) -> Option<RevolutionStructure> {
    if surface.degree_u != 2 {
        return None;
    }
    let rows = &surface.control_points;
    if rows.len() < 3 || rows.len() % 2 == 0 {
        return None;
    }
    let spans = (rows.len() - 1) / 2;
    if spans > 4 {
        return None;
    }
    let expected = full_circle_knots(spans);
    if surface.knots_u.len() != expected.len()
        || surface
            .knots_u
            .iter()
            .zip(&expected)
            .any(|(a, b)| (a - b).abs() > 1e-12)
    {
        return None;
    }
    let scale = surface_scale(surface);
    // The mid-row weight ratio encodes the per-span arc angle.
    let mut sweep = None;
    for column in 0..rows[0].len() {
        let corner = rows[0][column].w;
        let middle = rows[1][column].w;
        if corner.abs() <= 1e-12 {
            continue;
        }
        let ratio = middle / corner;
        if !(ratio > 0.0 && ratio <= 1.0 + 1e-12) {
            return None;
        }
        let arc_angle = 2.0 * ratio.min(1.0).acos();
        if arc_angle <= 1e-9 {
            return None;
        }
        sweep = Some(arc_angle * spans as f64);
        break;
    }
    let sweep = sweep?;
    if sweep > std::f64::consts::TAU + 1e-9 {
        return None;
    }
    // Frame from three points on a radiused angular arc (0.4/0.8 avoid the
    // closed-circle wraparound where evaluate(1.0) == evaluate(0.0)).
    let mut frame = None;
    for column in 0..rows[0].len() {
        let arc_points: Vec<crate::Vec4> = rows.iter().map(|row| row[column]).collect();
        let Ok(arc) = NurbsCurve::new(2, surface.knots_u.clone(), arc_points) else {
            return None;
        };
        let p0 = arc.evaluate(0.0).ok()?;
        let p1 = arc.evaluate(0.4).ok()?;
        let p2 = arc.evaluate(0.8).ok()?;
        let Some(center) = circumcenter(p0, p1, p2) else {
            continue;
        };
        let radial = p0.sub(center);
        let radius = radial.length();
        if radius <= RECOGNITION_TOLERANCE * scale.max(1.0) {
            continue;
        }
        let axis = radial.cross(p1.sub(center)).normalized().ok()?;
        let x_axis = radial.scale(1.0 / radius);
        let y_axis = axis.cross(x_axis);
        frame = Some(RevolutionFrame {
            origin: center,
            axis,
            x_axis,
            y_axis,
        });
        break;
    }
    let frame = frame?;
    let generatrix =
        NurbsCurve::new(surface.degree_v, surface.knots_v.clone(), rows[0].clone()).ok()?;
    let rebuilt = make_revolution(frame.origin, frame.axis, &generatrix, sweep).ok()?;
    if !nearly_equal_points(surface, &rebuilt, scale) {
        return None;
    }
    Some(RevolutionStructure {
        frame,
        generatrix,
        sweep,
    })
}

pub(super) fn rotate_curve_about_axis(
    curve: &NurbsCurve,
    origin: Vec3,
    axis: Vec3,
    angle: f64,
) -> Option<NurbsCurve> {
    let (sin, cos) = angle.sin_cos();
    let controls = curve
        .control_points
        .iter()
        .map(|control| {
            let point = control.point().ok()?;
            let d = point.sub(origin);
            let axial = axis.scale(d.dot(axis));
            let radial = d.sub(axial);
            let ortho = axis.cross(radial);
            let rotated = origin
                .add(axial)
                .add(radial.scale(cos))
                .add(ortho.scale(sin));
            Some(crate::Vec4::from_point(rotated, control.w))
        })
        .collect::<Option<Vec<_>>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).ok()
}

/// Exact intersection of two surfaces of revolution about a COMMON axis:
/// by rotational symmetry the intersection is the revolution of the 2D
/// profile intersections, i.e. circular arcs over the angular overlap.
/// One exact curve×surface hit-test on a generatrix placed mid-overlap
/// finds the (radius, height) pairs; empty hits prove the surfaces
/// disjoint (skipping the marcher, which is ill-conditioned exactly here
/// when the profiles touch tangentially).
pub(super) fn intersect_coaxial_revolutions(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let a = revolution_structure(first)?;
    let b = revolution_structure(second)?;
    let scale = surface_scale(first).max(surface_scale(second)).max(1.0);
    let alignment = a.frame.axis.dot(b.frame.axis);
    if alignment.abs() < 1.0 - 1e-9 {
        return None;
    }
    let offset = b.frame.origin.sub(a.frame.origin);
    let perpendicular = offset.sub(a.frame.axis.scale(offset.dot(a.frame.axis)));
    if perpendicular.length() > 1e-9 * scale {
        return None;
    }
    // Angular interval of `second` expressed in `first`'s frame.
    let b_start = b
        .frame
        .x_axis
        .dot(a.frame.y_axis)
        .atan2(b.frame.x_axis.dot(a.frame.x_axis));
    let (b_lo, b_hi) = if alignment > 0.0 {
        (b_start, b_start + b.sweep)
    } else {
        (b_start - b.sweep, b_start)
    };
    let tau = std::f64::consts::TAU;
    let mut intervals = Vec::new();
    for shift in [-tau, 0.0, tau] {
        let lo = (b_lo + shift).max(0.0);
        let hi = (b_hi + shift).min(a.sweep);
        if hi - lo > 1e-9 {
            intervals.push((lo, hi));
        }
    }
    if intervals.is_empty() {
        return Some(Vec::new());
    }
    let middle = (intervals[0].0 + intervals[0].1) / 2.0;
    let generatrix = rotate_curve_about_axis(&a.generatrix, a.frame.origin, a.frame.axis, middle)?;
    let hits = crate::intersect_curve_surface(&generatrix, second, tolerance).ok()?;
    if hits.len() > 8 {
        // A hit spray means the profiles overlap (coincident surfaces);
        // leave that to the cosurface machinery.
        return None;
    }
    let radius_floor = (tolerance * 10.0).max(1e-9 * scale);
    let mut seen: Vec<(f64, f64)> = Vec::new();
    let mut curves = Vec::new();
    for hit in hits {
        // TANGENT-CONTACT SKIP (problemInbox equator-tangent: sphere r=R
        // inscribed in a cylinder r=R): a meridian hit whose curve tangent
        // lies IN the other surface (hit.tangential — computed exactly by
        // the curve×surface intersector, no projector involved) sweeps a
        // CONTACT circle where the coaxial surfaces touch without crossing.
        // Minting it duplicates the true section ring (the cap-plane circle)
        // and strands one-use edges at assembly. A transversal meridian
        // crossing keeps tangential=false and is unaffected. Escape hatch:
        // BREP_COAXIAL_TANGENT_SKIP=0.
        if hit.tangential
            && std::env::var("BREP_COAXIAL_TANGENT_SKIP").as_deref() != Ok("0")
        {
            continue;
        }
        let (_, radius, axial) = a.frame.cylindrical(hit.point);
        if radius <= radius_floor {
            continue;
        }
        if seen
            .iter()
            .any(|(r, z)| (r - radius).abs() + (z - axial).abs() <= radius_floor)
        {
            continue;
        }
        seen.push((radius, axial));
        for (lo, hi) in &intervals {
            let center = a.frame.origin.add(a.frame.axis.scale(axial));
            curves.push(
                make_arc(center, a.frame.x_axis, a.frame.y_axis, radius, *lo, *hi).ok()?,
            );
        }
    }
    Some(curves)
}
