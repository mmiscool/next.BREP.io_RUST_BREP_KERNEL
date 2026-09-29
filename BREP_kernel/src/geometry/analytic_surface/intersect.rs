use super::*;
use super::recognition::{generatrix_is_meridional_half_ray, surface_scale};
use super::revolution::{intersect_coaxial_revolutions, rotate_curve_about_axis};

/// Exact plane data extracted from an affine patch.
struct PlaneData {
    origin: Vec3,
    normal: Vec3,
}

fn plane_data(analytic: &AnalyticSurface) -> Option<PlaneData> {
    match analytic {
        AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        } => Some(PlaneData {
            origin: *origin,
            normal: u_dir.cross(*v_dir).normalized().ok()?,
        }),
        _ => None,
    }
}

/// Apply a homogeneous 4x4-style map to a rational curve's control points.
/// `map` produces the image (x', y', z', w') of each homogeneous control
/// point; weights must come back strictly one-signed or the section is not
/// an ellipse/circle and we refuse it.
fn map_rational_curve(
    curve: &NurbsCurve,
    map: impl Fn(crate::Vec4) -> crate::Vec4,
) -> Option<NurbsCurve> {
    let mut controls = Vec::with_capacity(curve.control_points.len());
    let mut sign = 0.0f64;
    for point in &curve.control_points {
        let image = map(*point);
        if image.w == 0.0 || !image.w.is_finite() {
            return None;
        }
        if sign == 0.0 {
            sign = image.w.signum();
        } else if image.w.signum() != sign {
            return None;
        }
        controls.push(image);
    }
    if sign < 0.0 {
        for point in &mut controls {
            point.x = -point.x;
            point.y = -point.y;
            point.z = -point.z;
            point.w = -point.w;
        }
    }
    NurbsCurve::new(curve.degree, curve.knots.clone(), controls).ok()
}

/// The full u-circle of a revolution surface at radial distance `rho` and
/// axial height `z`, built with the same arc construction as the surface.
fn frame_circle(frame: &RevolutionFrame, rho: f64, z: f64) -> Option<NurbsCurve> {
    make_arc(
        frame.origin.add(frame.axis.scale(z)),
        frame.x_axis,
        frame.y_axis,
        rho,
        0.0,
        std::f64::consts::TAU,
    )
    .ok()
}

/// Exact intersection curves for recognized analytic pairs, or `None` when
/// the pair is not handled and the caller must fall back to SSI marching.
/// `Some(vec![])` means "provably empty" and skips marching entirely.
pub fn intersect_analytic_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    intersect_analytic_pair_with(first, second, tolerance, ruled_gate())
}

/// Which predicate admits a general revolution's generatrix as the straight
/// line of a cone, frustum or cylinder in [`intersect_analytic_pair`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RuledGate {
    /// The count proxy: degree 1, exactly two control points, unit weights.
    Count,
    /// What the section constructions consume: the generatrix's two ends, and
    /// that the curve between them is the straight segment joining them
    /// ([`NurbsCurve::straight_segment`]).
    Geometry,
}

/// The gate the boolean runs: straightness, which is all the section
/// constructions read. The count proxy declined a knot-refined, degree-elevated
/// or re-weighted generatrix of the same cone and sent the pair to the marcher.
/// Escape hatch for measurement: `BREP_RULED_GATE_GEOMETRY=0` runs the count
/// gate.
pub(crate) fn ruled_gate() -> RuledGate {
    if std::env::var("BREP_RULED_GATE_GEOMETRY").as_deref() == Ok("0") {
        RuledGate::Count
    } else {
        RuledGate::Geometry
    }
}

/// [`intersect_analytic_pair`] under a named generatrix gate — what the fit
/// census asks of the gate the boolean does NOT run.
pub(crate) fn intersect_analytic_pair_with(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    if let Some(curves) = intersect_recognized_pair(first, second, tolerance, gate) {
        return Some(curves);
    }
    if let Some(curves) = intersect_coaxial_revolutions(first, second, tolerance) {
        return Some(curves);
    }
    if let Some(curves) = intersect_axial_plane_revolution(first, second, tolerance) {
        return Some(curves);
    }
    intersect_axial_plane_revolution(second, first, tolerance)
}

/// Exact intersection of a plane CONTAINING the revolution axis with a
/// surface of revolution: the meridian profile rotated to the (at most
/// two) angles where the plane crosses the sweep. The marcher fits these
/// as long noisy polylines otherwise (a revolve's start/end caps against
/// the other operand's revolved walls in revolve-with-holes booleans).
fn intersect_axial_plane_revolution(
    plane_surface: &NurbsSurface,
    revolved: &NurbsSurface,
    _tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let plane = match plane_surface.analytic()? {
        AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        } => (*origin, u_dir.cross(*v_dir).normalized().ok()?),
        _ => return None,
    };
    let structure = revolution_structure(revolved)?;
    if !generatrix_is_meridional_half_ray(&structure.generatrix, &structure.frame) {
        return None;
    }
    let scale = surface_scale(plane_surface)
        .max(surface_scale(revolved))
        .max(1.0);
    let (plane_origin, plane_normal) = plane;
    if structure.frame.axis.dot(plane_normal).abs() > 1e-9 {
        return None;
    }
    if structure
        .frame
        .origin
        .sub(plane_origin)
        .dot(plane_normal)
        .abs()
        > 1e-9 * scale
    {
        return None;
    }
    let radial = structure.frame.axis.cross(plane_normal).normalized().ok()?;
    let tau = std::f64::consts::TAU;
    let mut curves = Vec::new();
    for direction in [radial, radial.scale(-1.0)] {
        let mut angle = direction
            .dot(structure.frame.y_axis)
            .atan2(direction.dot(structure.frame.x_axis));
        if angle < -1e-9 {
            angle += tau;
        }
        if angle <= structure.sweep + 1e-9 {
            curves.push(rotate_curve_about_axis(
                &structure.generatrix,
                structure.frame.origin,
                structure.frame.axis,
                angle.clamp(0.0, structure.sweep),
            )?);
        }
    }
    Some(curves)
}

fn intersect_recognized_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    let a = first.analytic()?;
    let b = second.analytic()?;
    if !matches!(b, AnalyticSurface::Plane { .. }) {
        if let Some(plane) = plane_data(a) {
            return intersect_plane_quadric(&plane, b, tolerance, gate);
        }
    }
    if !matches!(a, AnalyticSurface::Plane { .. }) {
        if let Some(plane) = plane_data(b) {
            return intersect_plane_quadric(&plane, a, tolerance, gate);
        }
    }
    // Sphere×sphere, whichever way each sphere is parameterized (a reflected
    // sphere recognizes as a general `Revolution`; see `sphere_geometry`).
    if let (Some((center_a, radius_a)), Some((center_b, radius_b))) =
        (a.sphere_geometry(), b.sphere_geometry())
    {
        return intersect_sphere_sphere(center_a, radius_a, center_b, radius_b, tolerance);
    }
    // Plane×plane is deliberately NOT special-cased: the iso path
    // already returns axis-aligned box intersections exactly, and a
    // synthesized over-long line segment interacts poorly with the
    // shared-parameter pcurve contract in process_curve (the pcurve is
    // fitted over the face crossing only, so the curve's [t0,t1]
    // subrange and the pcurve domain drift apart on long overhangs).
    None
}

/// Frame/meridian data for any revolution with a straight-line generatrix:
/// the `RuledRevolution` quadrics themselves, plus partial-sweep
/// `Revolution`s whose generatrix the named gate admits as straight (fillet
/// cutter walls are quarter cylinders of this shape). The plane
/// intersectors only need the carrier geometry — the boolean trims the
/// returned curves to the actual face domains afterwards.
fn ruled_revolution_data(
    quadric: &AnalyticSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<(RevolutionFrame, f64, f64, f64)> {
    match quadric {
        AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } => Some((frame.clone(), *rho0, *rho1, *height)),
        AnalyticSurface::Revolution {
            frame, generatrix, ..
        } => {
            let (start, end) = match gate {
                RuledGate::Count => {
                    let controls = &generatrix.control_points;
                    if generatrix.degree != 1
                        || controls.len() != 2
                        || (controls[0].w - 1.0).abs() > RECOGNITION_TOLERANCE
                        || (controls[1].w - 1.0).abs() > RECOGNITION_TOLERANCE
                    {
                        return None;
                    }
                    (controls[0].point().ok()?, controls[1].point().ok()?)
                }
                RuledGate::Geometry => generatrix.straight_segment(tolerance)?,
            };
            let (_, rho0, z0) = frame.cylindrical(start);
            let (_, rho1, z1) = frame.cylindrical(end);
            let height = z1 - z0;
            if height.abs() <= 1e-12 * (1.0 + rho0.abs().max(rho1.abs())) {
                return None;
            }
            let origin = frame.origin.add(frame.axis.scale(z0));
            Some((
                RevolutionFrame {
                    origin,
                    ..frame.clone()
                },
                rho0,
                rho1,
                height,
            ))
        }
        _ => None,
    }
}

/// CENSUS ONLY (`BREP_FIT_CENSUS=1`): a description of `surface`'s generatrix
/// when it is a general revolution the two generatrix gates answer
/// differently, `None` otherwise.
pub(crate) fn ruled_gate_census(surface: &NurbsSurface, tolerance: f64) -> Option<String> {
    let quadric = surface.analytic()?;
    let AnalyticSurface::Revolution { generatrix, .. } = quadric else {
        return None;
    };
    let count = ruled_revolution_data(quadric, tolerance, RuledGate::Count).is_some();
    let geometry = ruled_revolution_data(quadric, tolerance, RuledGate::Geometry).is_some();
    if count == geometry {
        return None;
    }
    let controls = &generatrix.control_points;
    let points: Vec<Vec3> = controls
        .iter()
        .map(|control| control.point().unwrap_or_default())
        .collect();
    let (first, last) = (points[0], points[points.len() - 1]);
    let direction = last.sub(first);
    let length_squared = direction.length_squared();
    let mut collinearity = 0.0_f64;
    for point in &points {
        collinearity = collinearity.max(if length_squared <= 0.0 {
            point.sub(first).length()
        } else {
            let fraction = (point.sub(first).dot(direction) / length_squared).clamp(0.0, 1.0);
            point.sub(first.add(direction.scale(fraction))).length()
        });
    }
    let unit_weights = controls
        .iter()
        .all(|control| (control.w - 1.0).abs() <= RECOGNITION_TOLERANCE);
    Some(format!(
        "degree={} controls={} collinearity={collinearity:.3e} unit_weights={unit_weights} \
         count_gate={count} geometry_gate={geometry}",
        generatrix.degree,
        controls.len()
    ))
}

/// TRUE when a recognized frustum's radius drift is below the precision floor
/// of the cone construction itself, so the plane section must take the
/// CYLINDER branch. The cone branch builds its conic as a projective image of
/// a base circle toward the apex at distance ~|height|·max|ρ|/|Δρ|; the map's
/// floating-point error grows like that distance × machine epsilon, so for a
/// near-cylinder frustum (t363: Δρ = 2.23e-12 over height 1500, apex ~1.5e16)
/// the "exact analytic" section comes back up to ~1 mm off BOTH surfaces.
/// Downstream, `build_pcurve_on_surface` honestly measures that deviation on
/// the curved operand, drops every interior sample, and collapses the ring's
/// pcurve to a single point — the face under-splits and the boolean fails
/// with one-use edges / non-integral genus.
///
/// Branch selection by error balance: the cylinder approximation errs by at
/// most |Δρ|, the cone construction by ~ε·|height|·max|ρ|/|Δρ|; prefer the
/// cylinder when its error is smaller, i.e. Δρ² ≤ ε·|height|·max|ρ| (with a
/// modest constant for the map's factors). This is a deterministic choice
/// between two exact-in-infinite-precision constructions — not a tolerance —
/// and can only ever pick the MORE accurate branch. The legacy absolute
/// 1e-12 acceptance remains as a floor at the call site. Escape hatch:
/// BREP_CONE_APEX_GUARD=0 restores the bare 1e-12 discrimination.
fn degenerate_apex_frustum(rho0: f64, rho1: f64, height: f64) -> bool {
    if std::env::var("BREP_CONE_APEX_GUARD").as_deref() == Ok("0") {
        return false;
    }
    let delta = rho1 - rho0;
    delta * delta <= 64.0 * f64::EPSILON * height.abs() * rho0.abs().max(rho1.abs())
}

/// The EXACT arc of `plane ∩ ruled revolution` spanning a given AZIMUTH range
/// about the carrier's own frame — the arc-restricted, seam-anchorable form of
/// the full sections [`intersect_plane_quadric`] returns.
///
/// ADDITION, not a change: `intersect_plane_quadric` is untouched and still
/// answers every pair it answered before, bit for bit. This function exists
/// because a direct edit needs the section as an EDGE, not as a carrier
/// section, and an edge has endpoints: a closed rim must START at the face's
/// own seam vertex (which need not sit at the carrier's `u = 0` — see the
/// push-face seam fix), and an open rim is a sub-arc between two re-solved
/// corners. `intersect_plane_quadric` always returns the whole section anchored
/// at the frame's `x_axis`, so neither is expressible through it.
///
/// Both constructions below are the ones that function already uses, and both
/// preserve AZIMUTH — the cylinder's map adds only an axial component, the
/// cone's is a central projection from a point ON the axis — so "the arc from
/// azimuth `a` through `sweep`" is well defined on the section itself and the
/// anchoring is exact rather than fitted:
///
/// - **Cylinder** (`rho1 ≈ rho0`): the section is the AFFINE image of the base
///   circle under the axial shear `X ↦ X + axis·(c − n·X)/(n·axis)`, which is
///   linear in the homogeneous control point, so a rational-quadratic arc maps
///   to a rational-quadratic arc on the SAME knot vector.
/// - **Cone/frustum**: the section is the PROJECTIVE image from the apex,
///   `X ↦ apex + (k/((X−apex)·n))·(X−apex)`, again linear in the homogeneous
///   control point. Weight signs flip for a parabolic/hyperbolic section (the
///   arc crosses the apex plane), and those are refused by name rather than
///   silently mis-built.
///
/// `sweep` must be positive and at most a full turn; a reversed rim is built
/// forward and reversed by the caller, which is what keeps the knot vector the
/// arc construction's own.
pub fn plane_ruled_section_arc(
    plane_origin: Vec3,
    plane_normal: Vec3,
    frame: &RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    start_azimuth: f64,
    sweep: f64,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    let tau = std::f64::consts::TAU;
    if !(start_azimuth.is_finite() && sweep.is_finite()) {
        return Err("plane_ruled_section_arc: non-finite azimuth range".into());
    }
    if sweep <= 1e-12 || sweep > tau + 1e-9 {
        return Err(format!(
            "plane_ruled_section_arc: the section arc sweeps {sweep:.6e} rad, which is not a \
             positive arc of at most one turn — refusing"
        ));
    }
    let normal = plane_normal.normalized()?;
    let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
    let axis = frame.axis;
    let is_cylinder = (rho1 - rho0).abs() <= 1e-9 * radius_scale
        || degenerate_apex_frustum(rho0, rho1, height);
    let curve = if is_cylinder {
        let alignment = normal.dot(axis);
        if alignment.abs() <= 1e-12 {
            return Err(
                "plane_ruled_section_arc: the plane is parallel to the cylinder axis, so the \
                 section is a generatrix pair rather than a conic — refusing"
                    .into(),
            );
        }
        if rho0.abs() <= tolerance {
            return Err("plane_ruled_section_arc: the cylinder carrier has no radius".into());
        }
        let base = make_arc(
            frame.origin,
            frame.x_axis,
            frame.y_axis,
            rho0.abs(),
            start_azimuth,
            start_azimuth + sweep,
        )?;
        let origin_dot = plane_origin.dot(normal);
        map_rational_curve(&base, |p| {
            let t = (origin_dot * p.w - Vec3::new(p.x, p.y, p.z).dot(normal)) / alignment;
            crate::Vec4 {
                x: p.x + axis.x * t,
                y: p.y + axis.y * t,
                z: p.z + axis.z * t,
                w: p.w,
            }
        })
        .ok_or_else(|| {
            "plane_ruled_section_arc: the cylinder section's weights are not one-signed \
             — refusing"
                .to_string()
        })?
    } else {
        let apex = frame.origin.add(axis.scale(height * rho0 / (rho0 - rho1)));
        let k = plane_origin.sub(apex).dot(normal);
        if k.abs() <= tolerance {
            return Err(
                "plane_ruled_section_arc: the plane passes through the cone apex, so the \
                 section is a line pair rather than a conic — refusing"
                    .into(),
            );
        }
        let (reference_rho, reference_z) = if rho0.abs() > rho1.abs() {
            (rho0.abs(), 0.0)
        } else {
            (rho1.abs(), height)
        };
        if reference_rho <= tolerance {
            return Err("plane_ruled_section_arc: the cone carrier degenerates to its apex".into());
        }
        let base = make_arc(
            frame.origin.add(axis.scale(reference_z)),
            frame.x_axis,
            frame.y_axis,
            reference_rho,
            start_azimuth,
            start_azimuth + sweep,
        )?;
        map_rational_curve(&base, |p| {
            let relative = Vec3::new(p.x - p.w * apex.x, p.y - p.w * apex.y, p.z - p.w * apex.z);
            let w_new = relative.dot(normal);
            let scaled = relative.scale(k);
            crate::Vec4 {
                x: apex.x * w_new + scaled.x,
                y: apex.y * w_new + scaled.y,
                z: apex.z * w_new + scaled.z,
                w: w_new,
            }
        })
        .ok_or_else(|| {
            "plane_ruled_section_arc: the section crosses the cone's apex plane (a parabolic \
             or hyperbolic branch, whose weights change sign) — refusing"
                .to_string()
        })?
    };
    // The construction is exact, so this measures rather than fits: every
    // sample must sit on BOTH carriers, at the carrier's own radius for its own
    // axial station and on the plane.
    let [d0, d1] = curve.domain()?;
    let slope = if height == 0.0 {
        0.0
    } else {
        (rho1 - rho0) / height
    };
    for step in 0..=8 {
        let point = curve.evaluate(d0 + (d1 - d0) * (step as f64 / 8.0))?;
        let delta = point.sub(frame.origin);
        let axial = delta.dot(axis);
        let radial = delta.sub(axis.scale(axial)).length();
        let off_ruled = (radial - (rho0 + slope * axial)).abs();
        let off_plane = point.sub(plane_origin).dot(normal).abs();
        if off_ruled > 10.0 * tolerance || off_plane > 10.0 * tolerance {
            return Err(format!(
                "plane_ruled_section_arc: the section does not lie on both carriers (off ruled \
                 {off_ruled:.3e}, off plane {off_plane:.3e}) — refusing"
            ));
        }
    }
    Ok(curve)
}

fn intersect_plane_quadric(
    plane: &PlaneData,
    quadric: &AnalyticSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    match quadric {
        AnalyticSurface::RuledRevolution { .. } | AnalyticSurface::Revolution { .. }
            if ruled_revolution_data(quadric, tolerance, gate).is_some() =>
        {
            let (frame, rho0, rho1, height) = ruled_revolution_data(quadric, tolerance, gate)?;
            let (frame, rho0, rho1, height) = (&frame, &rho0, &rho1, &height);
            let alignment = plane.normal.dot(frame.axis);
            if alignment.abs() >= 1.0 - 1e-12 {
                // Perpendicular plane: one circle at the plane's height.
                let z = plane.origin.sub(frame.origin).dot(frame.axis);
                let t = z / height;
                if !(-1e-9..=1.0 + 1e-9).contains(&t) {
                    return Some(Vec::new());
                }
                let rho = rho0 + (rho1 - rho0) * t.clamp(0.0, 1.0);
                if rho <= tolerance {
                    return Some(Vec::new());
                }
                return Some(vec![frame_circle(frame, rho, z)?]);
            }
            if (rho1 - rho0).abs() <= 1e-12 || degenerate_apex_frustum(*rho0, *rho1, *height) {
                // Cylinder.
                if alignment.abs() <= 1e-12 {
                    // Plane parallel to the axis: zero, one, or two
                    // generatrix lines at the circle/line crossing angles.
                    return cylinder_parallel_plane_lines(plane, frame, *rho0, *height, tolerance);
                }
                // Oblique section: exact ellipse as the affine image of a
                // base circle along the axis direction.
                let circle = frame_circle(frame, *rho0, 0.0)?;
                let denominator = alignment;
                let origin_dot = plane.origin.dot(plane.normal);
                let normal = plane.normal;
                let axis = frame.axis;
                let ellipse = map_rational_curve(&circle, |p| {
                    let t = (origin_dot * p.w - Vec3::new(p.x, p.y, p.z).dot(normal)) / denominator;
                    crate::Vec4 {
                        x: p.x + axis.x * t,
                        y: p.y + axis.y * t,
                        z: p.z + axis.z * t,
                        w: p.w,
                    }
                })?;
                if !section_within_band(&ellipse, frame, *height, tolerance) {
                    return Some(Vec::new());
                }
                return Some(vec![ellipse]);
            }
            // Cone/frustum: projective image of a base circle toward the
            // apex. Weight signs flip for parabolic/hyperbolic sections and
            // map_rational_curve refuses those (marcher fallback).
            let apex_t = rho0 / (rho0 - rho1);
            let apex = frame.origin.add(frame.axis.scale(height * apex_t));
            let reference_rho = if rho0.abs() > rho1.abs() {
                *rho0
            } else {
                *rho1
            };
            let reference_z = if rho0.abs() > rho1.abs() {
                0.0
            } else {
                *height
            };
            let circle = frame_circle(frame, reference_rho, reference_z)?;
            let k = plane.origin.sub(apex).dot(plane.normal);
            if k.abs() <= tolerance {
                // Plane through the apex: line pair; leave to the marcher.
                return None;
            }
            let normal = plane.normal;
            let conic = map_rational_curve(&circle, |p| {
                let relative =
                    Vec3::new(p.x - p.w * apex.x, p.y - p.w * apex.y, p.z - p.w * apex.z);
                let w_new = relative.dot(normal);
                let scaled = relative.scale(k);
                crate::Vec4 {
                    x: apex.x * w_new + scaled.x,
                    y: apex.y * w_new + scaled.y,
                    z: apex.z * w_new + scaled.z,
                    w: w_new,
                }
            })?;
            if !section_within_band(&conic, frame, *height, tolerance) {
                return Some(Vec::new());
            }
            Some(vec![conic])
        }
        AnalyticSurface::Sphere { frame, radius } => {
            plane_sphere_section(plane, frame.origin, *radius, tolerance)
        }
        AnalyticSurface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => plane_torus_section(plane, frame, *major_radius, *minor_radius, tolerance),
        AnalyticSurface::Plane { .. } => None,
        AnalyticSurface::RuledRevolution { .. } => None,
        // A REFLECTED sphere or torus (mirror feature, negative-determinant
        // transform) recognizes as a general `Revolution` — same point set,
        // opposite parameter handedness — and keeps its closed-form sections.
        // Any other general revolution has none; the axial-plane and
        // tangential-contact paths in imprint own those.
        AnalyticSurface::Revolution { .. } => {
            if let Some((center, radius)) = quadric.sphere_geometry() {
                return plane_sphere_section(plane, center, radius, tolerance);
            }
            if let Some((frame, major_radius, minor_radius)) = quadric.torus_geometry() {
                return plane_torus_section(plane, &frame, major_radius, minor_radius, tolerance);
            }
            None
        }
    }
}

/// Plane × sphere: one circle, or provably empty when the plane misses or
/// merely TOUCHES the sphere (a point contact imprints nothing).
fn plane_sphere_section(
    plane: &PlaneData,
    center: Vec3,
    radius: f64,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let distance = center.sub(plane.origin).dot(plane.normal);
    if distance.abs() >= radius - tolerance {
        return Some(Vec::new());
    }
    let circle_center = center.sub(plane.normal.scale(distance));
    let circle_radius = (radius * radius - distance * distance).sqrt();
    let x_axis = plane.normal.perpendicular().ok()?;
    let y_axis = plane.normal.cross(x_axis);
    Some(vec![make_arc(
        circle_center,
        x_axis,
        y_axis,
        circle_radius,
        0.0,
        std::f64::consts::TAU,
    )
    .ok()?])
}

/// Plane ⟂ torus axis: the two latitude circles at the plane's height (one
/// when it grazes the tube's inner side), or provably empty above/below the
/// tube.  `frame.origin` must sit at the tube centre's axial position.  An
/// oblique plane has no closed form here (`None` → marcher).
fn plane_torus_section(
    plane: &PlaneData,
    frame: &RevolutionFrame,
    major_radius: f64,
    minor_radius: f64,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let alignment = plane.normal.dot(frame.axis);
    if alignment.abs() < 1.0 - 1e-12 {
        return None;
    }
    let z = plane.origin.sub(frame.origin).dot(frame.axis);
    if z.abs() >= minor_radius - tolerance {
        return Some(Vec::new());
    }
    let offset = (minor_radius * minor_radius - z * z).sqrt();
    let mut circles = Vec::new();
    for rho in [major_radius - offset, major_radius + offset] {
        if rho > tolerance {
            circles.push(frame_circle(frame, rho, z)?);
        }
    }
    Some(circles)
}

/// Reject sections that entirely miss the ruled surface's axial band; the
/// imprint trimmer would discard them anyway, but skipping early avoids
/// building pcurves for out-of-band curves.
///
/// Uses the convex-hull property of a same-sign-weight rational curve: every
/// curve point's axial coordinate lies within the [min, max] axial span of the
/// Cartesian control points. So the section can only miss the band when the
/// WHOLE control-point span sits above or below it. The earlier `any(CP in
/// band)` test was wrong for near-axis-parallel oblique sections: the affine
/// map divides the axial displacement by `alignment`, flinging the rational
/// control points far to BOTH sides of the band (they straddle it with none
/// landing inside), so a genuine crossing arc was falsely rejected — the
/// missing cap∩cylinder imprint behind the curved-primitive open-edge /
/// non-integral-genus booleans. An interval-overlap test can never falsely
/// reject (hull property); a false accept is harmless (the imprint trimmer
/// clips the returned curve to the real face domains regardless).
fn section_within_band(
    curve: &NurbsCurve,
    frame: &RevolutionFrame,
    height: f64,
    tolerance: f64,
) -> bool {
    let (band_low, band_high) = if height >= 0.0 {
        (0.0, height)
    } else {
        (height, 0.0)
    };
    let margin = tolerance.max(1e-9) + 1e-9;
    let mut min_z = f64::INFINITY;
    let mut max_z = f64::NEG_INFINITY;
    for p in &curve.control_points {
        let z = Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w)
            .sub(frame.origin)
            .dot(frame.axis);
        min_z = min_z.min(z);
        max_z = max_z.max(z);
    }
    // The control-point axial span overlaps the band.
    max_z >= band_low - margin && min_z <= band_high + margin
}

fn cylinder_parallel_plane_lines(
    plane: &PlaneData,
    frame: &RevolutionFrame,
    radius: f64,
    height: f64,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    // Distance from the axis to the plane, measured in the cross-section.
    let axis_to_plane = plane.origin.sub(frame.origin).dot(plane.normal);
    if axis_to_plane.abs() >= radius - tolerance {
        return Some(Vec::new());
    }
    // In-plane offset of the chord from the axis foot.
    let chord_half = (radius * radius - axis_to_plane * axis_to_plane).sqrt();
    let chord_direction = frame.axis.cross(plane.normal).normalized().ok()?;
    let foot = frame.origin.add(plane.normal.scale(axis_to_plane));
    let mut lines = Vec::new();
    for sign in [-1.0, 1.0] {
        let base = foot.add(chord_direction.scale(sign * chord_half));
        let top = base.add(frame.axis.scale(height));
        lines.push(crate::make_line(base, top).ok()?);
    }
    Some(lines)
}

fn intersect_sphere_sphere(
    center_a: Vec3,
    radius_a: f64,
    center_b: Vec3,
    radius_b: f64,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let offset = center_b.sub(center_a);
    let distance = offset.length();
    if distance <= tolerance {
        // Concentric: empty or coincident; the cosurface path owns
        // coincident carriers.
        return if (radius_a - radius_b).abs() <= tolerance {
            None
        } else {
            Some(Vec::new())
        };
    }
    if distance >= radius_a + radius_b - tolerance
        || distance <= (radius_a - radius_b).abs() + tolerance
    {
        return Some(Vec::new());
    }
    let normal = offset.scale(1.0 / distance);
    let along =
        (distance * distance + radius_a * radius_a - radius_b * radius_b) / (2.0 * distance);
    let circle_radius_squared = radius_a * radius_a - along * along;
    if circle_radius_squared <= tolerance * tolerance {
        return Some(Vec::new());
    }
    let center = center_a.add(normal.scale(along));
    let x_axis = normal.perpendicular().ok()?;
    let y_axis = normal.cross(x_axis);
    Some(vec![make_arc(
        center,
        x_axis,
        y_axis,
        circle_radius_squared.sqrt(),
        0.0,
        std::f64::consts::TAU,
    )
    .ok()?])
}
