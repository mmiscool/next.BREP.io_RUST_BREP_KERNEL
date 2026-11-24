use super::*;
pub use super::recognition::{generatrix_is_meridional_half_ray, surface_scale};
use super::revolution::{intersect_coaxial_revolutions, intersect_cylinder_radius_invariant, rotate_curve_about_axis};

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

/// `BREP_CONE_PLANE_OPEN_SECTION` (default off, `1` on): a plane cutting a
/// cone or frustum in a parabola or hyperbola is answered with the section's
/// arcs inside the carrier's axial band ([`cone_plane_open_section`]) instead
/// of being left to the marcher.
fn cone_plane_open_section_enabled() -> bool {
    crate::switch!(live "BREP_CONE_PLANE_OPEN_SECTION", false)
}

/// A plane's open (parabolic or hyperbolic) section of a cone or frustum,
/// exactly, over the carrier's axial band. The section point over the base
/// circle's angle θ is the circle point projected from the apex onto the
/// plane: homogeneous-linear in (cos θ, sin θ, 1), with weight
/// `w(θ) = A + B cos θ + C sin θ` and axial coordinate `z_apex + K / w(θ)`
/// (every base point sits at one height). The band's ends and `w = 0` (the
/// section's points at infinity) are each `R cos(θ − φ) = c` in closed form;
/// between consecutive roots `w` keeps its sign, so a base-circle arc there
/// maps to an exact rational quadratic. The arcs whose middle lies in the band
/// are returned.
#[allow(clippy::too_many_arguments)]
fn cone_plane_open_section(
    frame: &RevolutionFrame,
    height: f64,
    apex: Vec3,
    reference_rho: f64,
    reference_z: f64,
    k: f64,
    normal: Vec3,
    tolerance: f64,
    project: &dyn Fn(crate::Vec4) -> crate::Vec4,
) -> Option<Vec<NurbsCurve>> {
    let center = frame.origin.add(frame.axis.scale(reference_z));
    let a = center.sub(apex).dot(normal);
    let b = reference_rho * frame.x_axis.dot(normal);
    let c = reference_rho * frame.y_axis.dot(normal);
    let r = (b * b + c * c).sqrt();
    let z_apex = apex.sub(frame.origin).dot(frame.axis);
    // `K` in z(θ) = z_apex + K / w(θ): k times the base circle's height over the apex.
    let big_k = k * (reference_z - z_apex);
    let (band_low, band_high) = (0.0_f64.min(height), 0.0_f64.max(height));
    let margin = tolerance.max(1e-12);
    let axial = |theta: f64| -> Option<f64> {
        let w = a + b * theta.cos() + c * theta.sin();
        (w.abs() > 1e-300).then(|| z_apex + big_k / w)
    };
    let mut roots: Vec<f64> = Vec::new();
    let mut solve = |target: f64| {
        // b cos θ + c sin θ = target − a.
        if r <= 0.0 {
            return;
        }
        let ratio = (target - a) / r;
        if ratio.abs() > 1.0 {
            return;
        }
        let phi = c.atan2(b);
        let spread = ratio.acos();
        for theta in [phi + spread, phi - spread] {
            roots.push(theta.rem_euclid(std::f64::consts::TAU));
        }
    };
    solve(0.0);
    for end in [band_low, band_high] {
        let offset = end - z_apex;
        if offset.abs() > 1e-300 {
            solve(big_k / offset);
        }
    }
    roots.sort_by(f64::total_cmp);
    roots.dedup();
    if roots.is_empty() {
        roots.push(0.0);
    }
    let mut arcs = Vec::new();
    for index in 0..roots.len() {
        let start = roots[index];
        let mut end = if index + 1 < roots.len() { roots[index + 1] } else { roots[0] + std::f64::consts::TAU };
        if roots.len() == 1 {
            end = start + std::f64::consts::TAU;
        }
        // Only an empty interval is skipped: a thin one in angle can be a
        // long branch (a plane close to a large cone's axis); one too thin to
        // build an arc on refuses below and leaves the pair to the marcher.
        if !(end > start) {
            continue;
        }
        let middle = 0.5 * (start + end);
        let Some(z) = axial(middle) else { continue };
        if z < band_low - margin || z > band_high + margin {
            continue;
        }
        let arc = make_arc(center, frame.x_axis, frame.y_axis, reference_rho, start, end).ok()?;
        // Within the interval the weight keeps its sign: the image is exact.
        arcs.push(map_rational_curve(&arc, project)?);
    }
    Some(arcs)
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
pub enum RuledGate {
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
pub fn ruled_gate() -> RuledGate {
    if std::env::var("BREP_RULED_GATE_GEOMETRY").as_deref() == Ok("0") {
        RuledGate::Count
    } else {
        RuledGate::Geometry
    }
}

/// [`intersect_analytic_pair`] under a named generatrix gate — what the fit
/// census asks of the gate the boolean does NOT run.
pub fn intersect_analytic_pair_with(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    // A ball and a cylinder tangent inside it: Viviani's curve, exact
    // (`BREP_SPHERE_CYLINDER_TANGENT_SECTION`).
    if let Some(curves) = super::sphere_cylinder_tangent::intersect_sphere_cylinder_tangent(first, second, tolerance) {
        return Some(curves);
    }
    if let Some(curves) = intersect_recognized_pair(first, second, tolerance, gate) {
        return Some(curves);
    }
    if crate::switch!("BREP_EXTRUSION_SECTIONS", true) {
        for (plane, swept) in [(first,second),(second,first)] {
            if matches!(swept.analytic(),Some(AnalyticSurface::Plane { .. })) { continue; }
            if let (Some(plane),Some(extrusion)) = (plane.analytic().and_then(plane_data),swept.linear_extrusion()) {
                if let Some(curves) = super::plane_extrusion_section(extrusion,plane.origin,plane.normal,tolerance) { return Some(curves); }
            }
        }
        if !matches!(first.analytic(),Some(AnalyticSurface::Plane { .. })) && !matches!(second.analytic(),Some(AnalyticSurface::Plane { .. })) {
        if let (Some(a),Some(b)) = (first.linear_extrusion(),second.linear_extrusion()) {
            if let Some(curves) = super::parallel_extrusion_sections(a,b,tolerance) { return Some(curves); }
        }
        }
    }
    if let Some(curves) = intersect_plane_affine_sphere(first, second, tolerance) {
        return Some(curves);
    }
    if let Some(curves) = intersect_coaxial_revolutions(first, second, tolerance) {
        return Some(curves);
    }
    if coaxial_iso_section_enabled() {
        if let Some(curves) = intersect_cylinder_radius_invariant(first, second, tolerance) {
            return Some(curves);
        }
        if let Some(curves) = intersect_cylinder_radius_invariant(second, first, tolerance) {
            return Some(curves);
        }
    }
    // Only under the production gate: the count gate is the escape hatch that
    // measures the old routing (`ruled_gate_admits_a_straight_generatrix_
    // whatever_it_counts` reads it sending a re-weighted cone to the marcher).
    if gate == RuledGate::Geometry && plane_revolution_section_enabled() {
        if let Some(curves) = intersect_perpendicular_plane_revolution(first, second, tolerance) {
            return Some(curves);
        }
        if let Some(curves) = intersect_perpendicular_plane_revolution(second, first, tolerance) {
            return Some(curves);
        }
    }
    if let Some(curves) = intersect_axial_plane_revolution(first, second, tolerance) {
        return Some(curves);
    }
    intersect_axial_plane_revolution(second, first, tolerance)
}

/// `BREP_COAXIAL_ISO_SECTION` (default on; `=0` leaves the pair to the
/// marcher): a cylinder against a surface
/// whose radius about its axis is invariant along one parameter
/// ([`intersect_cylinder_radius_invariant`]).
fn coaxial_iso_section_enabled() -> bool {
    crate::switch!("BREP_COAXIAL_ISO_SECTION", true)
}

/// PLANE x REVOLUTION SECTION (2026-11-06;
/// `BREP_PLANE_REVOLUTION_SECTION`, default on, `=0` off). A plane square to the axis
/// of a general surface of revolution (one no quadric arm reads: a revolved
/// parabola, a revolved spline) meets it in one circle per height at which the
/// generatrix crosses the plane, over the revolution's sweep. No arm read this
/// pair, so it went to the marcher, which seeds on a grid and finds no loop
/// much smaller than the faces: brep-bench
/// `tangent/paraboloid-tip-a-ten-thousandth-high` (a size-8 paraboloid cut by
/// z = 1e-4, a rim circle of radius 0.028) read "no boundary faces".
///
/// The lane answers only when every crossing is clean: each a sign change of
/// the generatrix's height over the plane, transversal, and away from the
/// generatrix's ends and from the axis, and the scan's crossings are as many
/// as the generatrix's control points change sides of the plane (so none can
/// hide between two stations). A touching height, a crossing at a
/// rim the face already has as an edge, or no crossing at all keeps the pair
/// on the paths it took before (`None`).
fn plane_revolution_section_enabled() -> bool {
    crate::switch!("BREP_PLANE_REVOLUTION_SECTION", true)
}

fn intersect_perpendicular_plane_revolution(
    plane_surface: &NurbsSurface,
    revolution: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    let plane = plane_data(plane_surface.analytic()?)?;
    if matches!(revolution.analytic(), Some(AnalyticSurface::Plane { .. })) {
        return None;
    }
    let structure = super::revolution::revolution_structure(revolution)?;
    let frame = &structure.frame;
    if plane.normal.dot(frame.axis).abs() < 1.0 - 1e-12 {
        return None;
    }
    let generatrix = &structure.generatrix;
    let [start, end] = generatrix.domain().ok()?;
    let height = |t: f64| -> Option<f64> { Some(generatrix.evaluate(t).ok()?.sub(plane.origin).dot(plane.normal)) };
    let scale = surface_scale(revolution).max(1.0);
    let band = tolerance.max(1e-9 * scale);
    let spans = generatrix.knots.windows(2).filter(|pair| pair[1] > pair[0]).count().max(1);
    let stations = 64 * spans;
    let mut previous_t = start;
    let mut previous = height(start)?;
    // A generatrix end ON the plane is a rim the face has as an edge.
    if previous.abs() <= band || height(end)?.abs() <= band {
        return None;
    }
    let mut roots = Vec::new();
    for index in 1..=stations {
        let t = start + (end - start) * index as f64 / stations as f64;
        let value = height(t)?;
        if value.abs() <= band && index < stations {
            // A station on the plane: a touching height or a crossing exactly
            // here; the bisection below cannot tell them apart.
            return None;
        }
        if (previous < 0.0) != (value < 0.0) {
            let (mut lo, mut hi, mut lo_value) = (previous_t, t, previous);
            for _ in 0..200 {
                let middle = 0.5 * (lo + hi);
                if middle <= lo || middle >= hi {
                    break;
                }
                let middle_value = height(middle)?;
                if (middle_value < 0.0) == (lo_value < 0.0) {
                    lo = middle;
                    lo_value = middle_value;
                } else {
                    hi = middle;
                }
            }
            roots.push(0.5 * (lo + hi));
        }
        previous_t = t;
        previous = value;
    }
    if roots.is_empty() {
        return None;
    }
    // The height's numerator `sum w_i ((P_i - o).n) N_i(t)` is a B-spline, so
    // it has no more roots than its coefficients have sign changes (variation
    // diminishing). A count above the scan's means two crossings may share a
    // station, which the scan cannot see: decline rather than drop a circle.
    let mut coefficient_signs = Vec::new();
    for control in &generatrix.control_points {
        if control.w <= 0.0 {
            return None;
        }
        let value = control.point().ok()?.sub(plane.origin).dot(plane.normal);
        if value.abs() <= band {
            return None;
        }
        coefficient_signs.push(value > 0.0);
    }
    let sign_changes = coefficient_signs.windows(2).filter(|pair| pair[0] != pair[1]).count();
    if sign_changes != roots.len() {
        return None;
    }
    let radius_floor = (tolerance * 10.0).max(1e-9 * scale);
    let mut curves = Vec::new();
    for root in roots {
        let derivatives = generatrix.derivatives(root, 1).ok()?;
        let tangent = derivatives.get(1)?.normalized().ok()?;
        if tangent.dot(plane.normal).abs() <= 1e-9 {
            return None;
        }
        let point = generatrix.evaluate(root).ok()?;
        let (_, radius, axial) = frame.cylindrical(point);
        if radius <= radius_floor {
            return None;
        }
        let center = frame.origin.add(frame.axis.scale(axial));
        // The crossing turns from ITS azimuth: zero for a profile in the start
        // meridian, not for a straight profile skew to the axis (a hyperboloid).
        let phase = if generatrix_is_meridional_half_ray(generatrix, frame) {
            0.0
        } else {
            let radial = point.sub(center);
            radial.dot(frame.y_axis).atan2(radial.dot(frame.x_axis))
        };
        curves.push(make_arc(center, frame.x_axis, frame.y_axis, radius, phase, phase + structure.sweep).ok()?);
    }
    Some(curves)
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

/// Plane x affine sphere (`geometry/affine_sphere.rs`, under
/// `BREP_AFFINE_SPHERE`): the exact ellipse, either operand order. Nothing
/// else pairs with an ellipsoid in closed form (sphere, cylinder and cone
/// sections of it are quartics; the marcher keeps them).
fn intersect_plane_affine_sphere(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    for (plane_surface, other) in [(first, second), (second, first)] {
        let Some(ellipsoid) = other.affine_sphere() else { continue };
        let Some(plane) = plane_surface.analytic().and_then(plane_data) else { continue };
        return ellipsoid.plane_section(plane.origin, plane.normal, tolerance);
    }
    None
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
    if let Some(curves) = intersect_sphere_coaxial_ruled(a, b, tolerance, gate)
        .or_else(|| intersect_sphere_coaxial_ruled(b, a, tolerance, gate))
    {
        return Some(curves);
    }
    if let Some(curves) = intersect_equal_cylinders_meeting_axes(first, second, a, b, tolerance, gate) {
        return Some(curves);
    }
    // The residual conic is cut from the first carrier and checked on the
    // second; a wide partial cone first can fail where the other order
    // reads the same section, so both orders are asked.
    if let Some(curves) = intersect_cones_sharing_a_ruling(a, b, tolerance, gate)
        .or_else(|| intersect_cones_sharing_a_ruling(b, a, tolerance, gate))
    {
        return Some(curves);
    }
    // Plane×plane is deliberately NOT special-cased: the iso path
    // already returns axis-aligned box intersections exactly, and a
    // synthesized over-long line segment interacts poorly with the
    // shared-parameter pcurve contract in process_curve (the pcurve is
    // fitted over the face crossing only, so the curve's [t0,t1]
    // subrange and the pcurve domain drift apart on long overhangs).
    None
}

/// `BREP_SPHERE_CONE_COAXIAL` (default off, `1` on): a sphere against a cone,
/// frustum or cylinder whose axis passes through the sphere's centre
/// ([`intersect_sphere_coaxial_ruled`]).
fn sphere_cone_coaxial_enabled() -> bool {
    crate::switch!(live "BREP_SPHERE_CONE_COAXIAL", false)
}

/// A sphere (any carrier [`AnalyticSurface::sphere_geometry`] reads, a partial
/// sweep included under `BREP_PARTIAL_SPHERE`) against a ruled revolution
/// whose axis passes through its centre: the pair is coaxial, so the section
/// is the circles about that axis at the heights `z` where the generatrix's
/// radius `ρ(z) = ρ₀ + m z` reaches the sphere, `(z − z_c)² + ρ(z)² = r²`, a
/// quadratic. Roots off the generatrix's own nappe or outside its axial band
/// are dropped; no real root is a proved-empty pair.
fn intersect_sphere_coaxial_ruled(
    sphere: &AnalyticSurface,
    ruled: &AnalyticSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    if !sphere_cone_coaxial_enabled() {
        return None;
    }
    let (center, radius) = sphere.sphere_geometry()?;
    let (frame, rho0, rho1, height) = ruled_revolution_data(ruled, tolerance, gate)?;
    let offset = center.sub(frame.origin);
    let center_z = offset.dot(frame.axis);
    let off_axis = offset.sub(frame.axis.scale(center_z)).length();
    // Bands relative to the sphere, where the section is: never to the
    // ruled carrier's length or its distance from its frame origin.
    // The coaxial test is relative to the sphere; the section's own bands
    // (a circle's radius, the carrier's axial ends) are the tolerance itself.
    if off_axis > tolerance.max(1e-12) * (1.0 + radius) {
        return None;
    }
    let margin = tolerance.max(1e-12);
    let slope = (rho1 - rho0) / height;
    // In the centre's axial frame, z' = z − z_c, with ρ_c the generatrix's
    // radius at the centre's height: z'² + (ρ_c + m z')² = r² (no cancellation
    // against a far frame origin).
    let rho_center = rho0 + slope * center_z;
    let a = 1.0 + slope * slope;
    let b = 2.0 * slope * rho_center;
    let c = rho_center * rho_center - radius * radius;
    let discriminant = b * b - 4.0 * a * c;
    let mut heights = Vec::new();
    if discriminant > 0.0 {
        let root = discriminant.sqrt();
        let q = -0.5 * (b + if b >= 0.0 { root } else { -root });
        if q != 0.0 {
            heights.push(center_z + q / a);
            heights.push(center_z + c / q);
        } else {
            heights.push(center_z);
        }
    }
    // A tangent contact (a double root, the generatrix touching the sphere
    // along a circle) is left to the tangent-contact lanes, which place the
    // circle's seam and its touching vertices; this lane answers transverse
    // sections only.
    // Roots closer than 2e-6 of the radius (√disc / a is their separation)
    // are a tangency up to round-off, e.g. an equator contact about a
    // rotated axis: as ill-conditioned as an exact one, so left there too.
    let separation_bar = 2e-6 * radius * a;
    if discriminant.abs() <= 1e-9 * (b * b + (4.0 * a * c).abs()).max(radius * radius * 1e-24)
        || (discriminant > 0.0 && discriminant <= separation_bar * separation_bar)
    {
        return None;
    }
    let (band_low, band_high) = (0.0_f64.min(height), 0.0_f64.max(height));
    let mut circles = Vec::new();
    for z in heights {
        let rho = rho0 + slope * z;
        if rho < -margin || z < band_low - margin || z > band_high + margin || rho <= margin {
            continue;
        }
        circles.push(frame_circle(&frame, rho, z)?);
    }
    Some(circles)
}

/// `BREP_CONE_SHARED_RULING` (default off, `1` on): two cones tangent along a
/// shared ruling ([`intersect_cones_sharing_a_ruling`]).
fn cone_shared_ruling_enabled() -> bool {
    crate::switch!(live "BREP_CONE_SHARED_RULING", false)
}

/// A cone (or frustum) carrier as its double cone: apex, unit axis, and
/// `cos²` of the half-angle, with the frame and extent it was read from.
struct ConeData {
    apex: Vec3,
    axis: Vec3,
    cos2: f64,
    frame: RevolutionFrame,
    height: f64,
    /// The swept angle from the frame's x axis (`TAU` for a full revolution).
    sweep: f64,
    /// The carrier's own size: its axial extent and largest radius.
    size: f64,
}

fn cone_data(quadric: &AnalyticSurface, tolerance: f64, gate: RuledGate) -> Option<ConeData> {
    let (frame, rho0, rho1, height) = ruled_revolution_data(quadric, tolerance, gate)?;
    let slope = (rho1 - rho0) / height;
    if !(slope.is_finite() && slope.abs() > 1e-9) {
        return None;
    }
    let apex = frame.origin.add(frame.axis.scale(-rho0 / slope));
    let sweep = match quadric {
        AnalyticSurface::Revolution { sweep, .. } => *sweep,
        _ => std::f64::consts::TAU,
    };
    let size = height.abs().max(rho0.abs()).max(rho1.abs());
    Some(ConeData { apex, axis: frame.axis, cos2: 1.0 / (1.0 + slope * slope), frame, height, sweep, size })
}

impl ConeData {
    /// `((X − A)·d)² − cos²α |X − A|²`: zero on the double cone.
    fn value(&self, point: Vec3) -> f64 {
        let d = point.sub(self.apex);
        let along = d.dot(self.axis);
        along * along - self.cos2 * d.dot(d)
    }

    /// The distance from `point` to the double cone, to first order.
    fn distance(&self, point: Vec3) -> f64 {
        let d = point.sub(self.apex);
        let gradient = self.axis.scale(2.0 * d.dot(self.axis)).sub(d.scale(2.0 * self.cos2));
        let norm = gradient.length();
        if norm > 0.0 { self.value(point).abs() / norm } else { d.length() }
    }

    /// The quadric's symmetric 4×4 matrix on homogeneous `(x, 1)`.
    fn matrix(&self) -> [[f64; 4]; 4] {
        let d = [self.axis.x, self.axis.y, self.axis.z];
        let a = [self.apex.x, self.apex.y, self.apex.z];
        let mut m = [[0.0; 4]; 4];
        let mut da = [0.0; 3];
        for i in 0..3 {
            for j in 0..3 {
                m[i][j] = d[i] * d[j] - if i == j { self.cos2 } else { 0.0 };
            }
        }
        for i in 0..3 {
            da[i] = (0..3).map(|j| m[i][j] * a[j]).sum();
        }
        for i in 0..3 {
            m[i][3] = -da[i];
            m[3][i] = -da[i];
        }
        m[3][3] = (0..3).map(|i| a[i] * da[i]).sum();
        m
    }

    /// Whether `point`'s azimuth about the axis lies in the swept range (with
    /// `slack` radians either side); a point on the axis is in every one.
    fn within_sweep(&self, point: Vec3, slack: f64) -> bool {
        if self.sweep >= std::f64::consts::TAU - 1e-12 {
            return true;
        }
        match self.frame.cylindrical(point).0 {
            None => true,
            Some(theta) => theta <= self.sweep + slack || theta >= std::f64::consts::TAU - slack,
        }
    }

    /// Whether `point` lies within the carrier's axial extent (so on its own
    /// nappe of the double cone), with `slack` either side.
    fn within_extent(&self, point: Vec3, slack: f64) -> bool {
        let axial = point.sub(self.frame.origin).dot(self.frame.axis);
        axial >= 0.0_f64.min(self.height) - slack && axial <= 0.0_f64.max(self.height) + slack
    }

    /// The parameter range along the line `origin + s·direction` over which
    /// it lies within this carrier's axial extent.
    fn extent_along(&self, origin: Vec3, direction: Vec3) -> Option<[f64; 2]> {
        let rate = direction.dot(self.frame.axis);
        if rate.abs() <= 1e-12 {
            return None;
        }
        let at = origin.sub(self.frame.origin).dot(self.frame.axis);
        let (low, high) = (0.0_f64.min(self.height), 0.0_f64.max(self.height));
        let (s0, s1) = ((low - at) / rate, (high - at) / rate);
        Some([s0.min(s1), s0.max(s1)])
    }
}

/// The pieces of `curve` (rational, degree at most 2) within `cone`'s axial
/// extent, cut EXACTLY where it crosses either end of the band: within each
/// knot span the axial coordinate times the homogeneous weight is a
/// polynomial of degree at most 2 in the parameter, read from three
/// homogeneous evaluations, so every crossing is a root of a quadratic and no
/// arc that enters and leaves the band between samples is missed. `None` for a
/// curve of higher degree or a cut that cannot be made.
fn pieces_within_extent(curve: &NurbsCurve, cone: &ConeData, slack: f64) -> Option<Vec<NurbsCurve>> {
    if curve.degree > 2 {
        return None;
    }
    let (low, high) = (0.0_f64.min(cone.height) - slack, 0.0_f64.max(cone.height) + slack);
    let axial_times_weight = |t: f64, bound: f64| -> Option<f64> {
        let h = curve.evaluate_homogeneous(t).ok()?;
        let along = Vec3::new(h.x, h.y, h.z).sub(cone.frame.origin.scale(h.w)).dot(cone.frame.axis);
        Some(along - bound * h.w)
    };
    let mut breaks: Vec<f64> = curve.knots.iter().copied().collect();
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let mut cuts = Vec::new();
    for pair in breaks.windows(2) {
        let (t0, t1) = (pair[0], pair[1]);
        if !(t1 > t0) {
            continue;
        }
        for bound in [low, high] {
            // f(t) = a s² + b s + c on s ∈ [0, 1] from f(0), f(½), f(1).
            let (f0, fm, f1) = (axial_times_weight(t0, bound)?, axial_times_weight(0.5 * (t0 + t1), bound)?, axial_times_weight(t1, bound)?);
            let a = 2.0 * f1 - 4.0 * fm + 2.0 * f0;
            let b = 4.0 * fm - f1 - 3.0 * f0;
            let c = f0;
            let mut roots = Vec::new();
            if a.abs() <= 1e-14 * (b.abs() + c.abs()).max(1e-300) {
                if b != 0.0 {
                    roots.push(-c / b);
                }
            } else {
                let discriminant = b * b - 4.0 * a * c;
                if discriminant >= 0.0 {
                    let root = discriminant.sqrt();
                    let q = -0.5 * (b + b.signum() * root);
                    if q != 0.0 {
                        roots.push(q / a);
                        roots.push(c / q);
                    }
                }
            }
            // A crossing at a span's end (an internal knot) is a cut too;
            // the curve's own ends are dropped below.
            for s in roots {
                if s >= -1e-12 && s <= 1.0 + 1e-12 {
                    cuts.push(t0 + s.clamp(0.0, 1.0) * (t1 - t0));
                }
            }
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup();
    let inside = |point: Vec3| cone.within_extent(point, slack);
    let mut pieces = Vec::new();
    let mut rest = curve.clone();
    for cut in cuts {
        let [r0, r1] = rest.domain().ok()?;
        if cut <= r0 + 1e-12 * (r1 - r0).abs().max(1.0) || cut >= r1 - 1e-12 * (r1 - r0).abs().max(1.0) {
            continue;
        }
        let (left, right) = rest.split(cut).ok()?;
        pieces.push(left);
        rest = right;
    }
    pieces.push(rest);
    let mut kept = Vec::new();
    for piece in pieces {
        let [p0, p1] = piece.domain().ok()?;
        if inside(piece.evaluate(0.5 * (p0 + p1)).ok()?) {
            kept.push(piece);
        }
    }
    Some(kept)
}

/// Two cones TANGENT ALONG A SHARED RULING. A ruling of each passes through
/// its apex, so a shared one is the line through both apexes; it lies on
/// each double cone when it makes that cone's half-angle with the axis, and
/// a cone's tangent plane is constant along a ruling, so the cones are
/// tangent along the whole line when their tangent planes there agree. The
/// quartic intersection is then the ruling twice and a residual conic: in
/// the pencil `Q₁ − λQ₂` the member vanishing on the common tangent plane `T`
/// (λ read at one point of `T` off the ruling, where each cone restricts to
/// the doubled ruling) is the plane pair `T ∪ π`, and the conic is either cone
/// cut by `π`. The ruling is returned over both carriers' axial extents and
/// the conic as the first carrier's plane section, checked on the second.
/// A pair whose residual section cannot be read is left to the marcher.
///
/// A cone filled out to its sphere and the revolves of its half-sections
/// about a line through the apex plane (brep-bench cone-spun): the cones the
/// half-section's generators sweep are tangent to the turned cone along the
/// generators, and the marcher could not settle the tangent pair, so one
/// union order refused.
fn intersect_cones_sharing_a_ruling(
    a: &AnalyticSurface,
    b: &AnalyticSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    if !cone_shared_ruling_enabled() {
        return None;
    }
    let (first, second) = (cone_data(a, tolerance, gate)?, cone_data(b, tolerance, gate)?);
    let between = second.apex.sub(first.apex);
    let length = between.length();
    // Bands relative to the carriers' own size, not to where they sit.
    let scale = 1.0 + first.size.max(second.size);
    if !(length > 1e-9 * scale) {
        return None;
    }
    let line = between.scale(1.0 / length);
    // The apex line lies on both double cones.
    for cone in [&first, &second] {
        let along = line.dot(cone.axis);
        if (along * along - cone.cos2).abs() > 1e-9 {
            return None;
        }
    }
    // Their tangent planes along it agree.
    let normal_first = first.axis.cross(line).cross(line).normalized().ok()?;
    let normal_second = second.axis.cross(line).cross(line).normalized().ok()?;
    if normal_first.cross(normal_second).length() > 1e-9 {
        return None;
    }
    // λ at a point of T off the ruling.
    let off = first.apex.add(normal_first.cross(line).scale(length.max(1.0)));
    let (value_first, value_second) = (first.value(off), second.value(off));
    if !(value_second.abs() > 1e-12 * (1.0 + value_first.abs())) {
        return None;
    }
    let lambda = value_first / value_second;
    let (m_first, m_second) = (first.matrix(), second.matrix());
    let mut pencil = [[0.0; 4]; 4];
    for i in 0..4 {
        for j in 0..4 {
            pencil[i][j] = m_first[i][j] - lambda * m_second[i][j];
        }
    }
    let t = [normal_first.x, normal_first.y, normal_first.z, -normal_first.dot(first.apex)];
    let homogeneous = |point: Vec3| [point.x, point.y, point.z, 1.0];
    let apply = |m: &[[f64; 4]; 4], x: [f64; 4]| -> [f64; 4] {
        let mut out = [0.0; 4];
        for i in 0..4 {
            out[i] = (0..4).map(|j| m[i][j] * x[j]).sum();
        }
        out
    };
    let dot4 = |p: [f64; 4], q: [f64; 4]| p[0] * q[0] + p[1] * q[1] + p[2] * q[2] + p[3] * q[3];
    // π from one point y off T: P y = ½(t (π·y) + π (t·y)).
    let y = homogeneous(first.apex.add(normal_first.scale(length.max(1.0))));
    let s = dot4(t, y);
    if !(s.abs() > 1e-12) {
        return None;
    }
    let py = apply(&pencil, y);
    let pi_y = dot4(y, py) / s;
    let mut pi = [0.0; 4];
    for i in 0..4 {
        pi[i] = (2.0 * py[i] - t[i] * pi_y) / s;
    }
    // The member is that plane pair: its value at a few points is (t·x)(π·x).
    for probe in [second.apex.add(Vec3::new(0.37, -0.21, 0.53).scale(length)), first.apex.add(Vec3::new(-0.44, 0.61, 0.17).scale(length)), off.add(line.scale(0.3 * length))] {
        let x = homogeneous(probe);
        let member = dot4(x, apply(&pencil, x));
        let pair = dot4(t, x) * dot4(pi, x);
        if (member - pair).abs() > 1e-8 * (1.0 + member.abs().max(pair.abs())) {
            return None;
        }
    }
    let mut curves = Vec::new();
    // The ruling over both carriers' axial extents.
    if let (Some([a0, a1]), Some([b0, b1])) = (first.extent_along(first.apex, line), second.extent_along(first.apex, line)) {
        let (s0, s1) = (a0.max(b0), a1.min(b1));
        let (start, end) = (first.apex.add(line.scale(s0)), first.apex.add(line.scale(s1)));
        let middle = start.add(end).scale(0.5);
        if std::env::var_os("BREP_DEBUG_CONE_RULING").is_some() {
            eprintln!(
                "cone ruling: s [{s0:.6}, {s1:.6}] from {start:?} to {end:?}; middle theta first {:?} (sweep {:.6}) second {:?} (sweep {:.6})",
                first.frame.cylindrical(middle).0, first.sweep, second.frame.cylindrical(middle).0, second.sweep
            );
        }
        // A ruling outside a partial revolution's sweep is not on that face.
        // Any ruling longer than the tolerance itself is a section.
        if s1 - s0 > tolerance.max(1e-12) && first.within_sweep(middle, 1e-9) && second.within_sweep(middle, 1e-9) {
            curves.push(crate::make_line(start, end).ok()?);
        }
    }
    // The residual conic on π, when π is a real plane other than T.
    let pi_normal = Vec3::new(pi[0], pi[1], pi[2]);
    let pi_length = pi_normal.length();
    if pi_length > 1e-12 * (1.0 + pi[3].abs()) {
        let unit = pi_normal.scale(1.0 / pi_length);
        let origin = pi_normal.scale(-pi[3] / (pi_length * pi_length));
        let same_as_t = unit.cross(normal_first).length() <= 1e-9 && (unit.dot(first.apex) + pi[3] / pi_length).abs() <= tolerance;
        if !same_as_t {
            let plane = PlaneData { origin, normal: unit };
            let conics = intersect_plane_quadric(&plane, a, tolerance, gate)?;
            for conic in conics {
                let [c0, c1] = conic.domain().ok()?;
                let (mut inside, mut outside) = (0, 0);
                for k in 0..=32 {
                    let point = conic.evaluate(c0 + (c1 - c0) * k as f64 / 32.0).ok()?;
                    if second.distance(point) > 1e3 * tolerance.max(1e-12) * scale {
                        return None;
                    }
                    if first.within_sweep(point, 1e-9) && second.within_sweep(point, 1e-9) {
                        inside += 1;
                    } else {
                        outside += 1;
                    }
                }
                if std::env::var_os("BREP_DEBUG_CONE_RULING").is_some() {
                    let ends = (conic.evaluate(c0).ok(), conic.evaluate(c1).ok());
                    eprintln!("cone ruling conic: inside {inside} outside {outside}; ends {ends:?}; sweeps {:.6} {:.6}", first.sweep, second.sweep);
                }
                // The sweep counts above are read for the debug line only:
                // nothing declines on them. The section is of the first
                // carrier, whose plane section is returned whole once its
                // band admits it (an ellipse is not cut to the first
                // carrier's extent); on the second it must also lie on that
                // carrier's own nappe and axial extent, so the pieces past it
                // are cut off. A conic past a partial sweep's end continues on
                // the revolution's own extension; the imprint clips it to the
                // faces' trims.
                let slack = tolerance.max(1e-12) * scale;
                curves.extend(pieces_within_extent(&conic, &second, slack)?);
            }
        }
    }
    Some(curves)
}

/// `BREP_EQUAL_CYLINDER_SECTIONS` (default on, `0` off): two cylinders of one
/// radius whose axes meet ([`intersect_equal_cylinders_meeting_axes`]).
fn equal_cylinder_sections_enabled() -> bool {
    crate::switch!(live "BREP_EQUAL_CYLINDER_SECTIONS", true)
}

/// Two cylinders of ONE radius whose axes meet at a point `C` (not parallel):
/// a point at distance `r` from both axes lies on one of the two planes
/// through `C` that bisect the axes (normals `a - b` and `a + b`), so the
/// intersection is exactly the two plane sections of either cylinder, two
/// ellipses that cross where the two surfaces are tangent (the points of
/// the circle of radius `r` about `C` along `a x b`). This is the joint of
/// two straight slides of one ball-ended tool, whose envelope cylinders
/// share the joint ball: their crease is the section square to the mean
/// axis, and at a shallow angle a march seeded at the tangent points reads
/// the pair as touching only (2026-11-18, the ball-end raster). The sections
/// are of the first carrier; every station is checked on the second.
fn intersect_equal_cylinders_meeting_axes(
    surface_a: &NurbsSurface,
    surface_b: &NurbsSurface,
    a: &AnalyticSurface,
    b: &AnalyticSurface,
    tolerance: f64,
    gate: RuledGate,
) -> Option<Vec<NurbsCurve>> {
    if !equal_cylinder_sections_enabled() {
        return None;
    }
    let (frame_a, rho0_a, rho1_a, _) = ruled_revolution_data(a, tolerance, gate)?;
    let (frame_b, rho0_b, rho1_b, _) = ruled_revolution_data(b, tolerance, gate)?;
    // An absolute band at the caller's tolerance: one radius means one radius
    // to the tolerance whatever the size (floored at what the radius itself
    // can represent), never a share of the size or of the distance from the
    // world origin.
    let band = tolerance.max(4.0 * f64::EPSILON * 1.0f64.max(rho0_a.abs()).max(rho0_b.abs()));
    // ONE radius means one radius to round-off, not to the tolerance: two
    // radii a tolerance apart meet in two branches that pass each other near
    // the tangent points (a gap of order sqrt(r dr)), not in two crossing
    // ellipses, and such a pair is the march's.
    let same_radius = 16.0 * f64::EPSILON * 1.0f64.max(rho0_a.abs()).max(rho0_b.abs());
    if (rho0_a - rho1_a).abs() > band || (rho0_b - rho1_b).abs() > band || (rho0_a - rho0_b).abs() > same_radius || rho0_a <= band {
        return None;
    }
    let (radius, radius_b) = (rho0_a, rho0_b);
    let (da, mut db) = (frame_a.axis.normalized().ok()?, frame_b.axis.normalized().ok()?);
    if da.dot(db) < 0.0 {
        db = db.scale(-1.0);
    }
    let cross = da.cross(db);
    let sine = cross.length();
    // Parallel axes are the coaxial / parallel lanes' business; below this
    // angle the meeting point is not determined to `band`.
    if sine <= 1e-6 {
        return None;
    }
    // The axes must MEET, to round-off as the radii must agree: axes a
    // tolerance apart (skew) give two branches that pass each other near the
    // tangent points (a gap of order sqrt(r d)), not crossing ellipses.
    let offset = frame_b.origin.sub(frame_a.origin);
    let skew = offset.dot(cross.scale(1.0 / sine)).abs();
    let meet = 16.0 * f64::EPSILON * 1.0f64.max(rho0_a.abs()).max(frame_a.origin.length()).max(frame_b.origin.length());
    if std::env::var_os("BREP_DEBUG_EQUAL_CYLINDERS").is_some() {
        eprintln!("equal cylinders: skew {skew:.3e} (round-off bar {meet:.3e}), radii {rho0_a} / {rho0_b}");
    }
    if skew > meet {
        return None;
    }
    // C = A0 + s da with (A0 + s da - B0) x db = 0: s = ((B0 - A0) x db)·(da x db) / |da x db|^2.
    let s = offset.cross(db).dot(cross) / (sine * sine);
    let centre = frame_a.origin.add(da.scale(s));
    let off_axis = |point: Vec3, origin: Vec3, axis: Vec3, rho: f64| {
        let delta = point.sub(origin);
        (delta.sub(axis.scale(delta.dot(axis))).length() - rho).abs()
    };
    // Each CARRIER is the cylinder its recognition names: a straight
    // generatrix with equal end radii also describes a hyperboloid (a line
    // skew to the axis), which is not this closed form.
    for (surface, origin, axis, rho) in [(surface_a, frame_a.origin, da, radius), (surface_b, frame_b.origin, db, radius_b)] {
        let ([u0, u1], [v0, v1]) = (surface.domain_u().ok()?, surface.domain_v().ok()?);
        for i in 0..=6 {
            for j in 0..=6 {
                let point = surface.evaluate(u0 + (u1 - u0) * i as f64 / 6.0, v0 + (v1 - v0) * j as f64 / 6.0).ok()?;
                if off_axis(point, origin, axis, rho) > band {
                    return None;
                }
            }
        }
    }
    let on_b = |point: Vec3| off_axis(point, frame_b.origin, db, radius_b);
    // Where the surfaces are tangent and the two ellipses cross.
    let across = cross.scale(radius / sine);
    let tangent_points = [centre.add(across), centre.sub(across)];
    let mut curves = Vec::new();
    for normal in [da.sub(db), da.add(db)] {
        let normal = normal.normalized().ok()?;
        let plane = PlaneData { origin: centre, normal };
        for curve in intersect_plane_quadric(&plane, a, tolerance, gate)? {
            let [t0, t1] = curve.domain().ok()?;
            for index in 0..=16 {
                let point = curve.evaluate(t0 + (t1 - t0) * index as f64 / 16.0).ok()?;
                if on_b(point) > band * 10.0 {
                    return None;
                }
            }
            if equal_cylinder_tangent_split_enabled() {
                curves.extend(split_at_points(curve, &tangent_points, band * 10.0)?);
            } else {
                curves.push(curve);
            }
        }
    }
    Some(curves)
}

/// `BREP_EQUAL_CYLINDER_TANGENT_SPLIT` (default on; `=0` keeps them whole): each ellipse of
/// [`intersect_equal_cylinders_meeting_axes`] is split at the two points where
/// the surfaces are tangent, where it crosses the other ellipse, so the four
/// arcs meet at shared vertices. Whole, the two closed ellipses of a Steinmetz
/// pair at the origin crossed without a junction: every boolean of the pair
/// left open loops, and the perturbation ladder then marched for minutes.
fn equal_cylinder_tangent_split_enabled() -> bool {
    crate::switch!(live "BREP_EQUAL_CYLINDER_TANGENT_SPLIT", true)
}

/// `curve` cut at each of `points` it passes within `reach` of (by its nearest
/// parameter), pieces in order; the curve whole when it passes none. A point
/// an end of the curve already stands on needs no cut (that end is the
/// junction). `None`, so the caller declines to the march, when a cut cannot
/// be made where the point is: the split refuses a parameter at an end, and
/// it snaps one near a knot, so every cut's junction is read again after it.
fn split_at_points(curve: NurbsCurve, points: &[Vec3], reach: f64) -> Option<Vec<NurbsCurve>> {
    let [t0, t1] = curve.domain().ok()?;
    let (start, end) = (curve.evaluate(t0).ok()?, curve.evaluate(t1).ok()?);
    let mut cuts: Vec<(f64, Vec3)> = Vec::new();
    for &point in points {
        if start.sub(point).length() <= reach || end.sub(point).length() <= reach {
            continue;
        }
        let foot = crate::project_point_to_curve(&curve, point).ok()?;
        if foot.distance <= reach {
            if !(foot.u > t0 && foot.u < t1) {
                return None;
            }
            cuts.push((foot.u, point));
        }
    }
    cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
    cuts.dedup_by(|a, b| (a.0 - b.0).abs() <= 1e-12 * (t1 - t0));
    let mut pieces = Vec::new();
    let mut rest = curve;
    for (cut, point) in cuts.into_iter().rev() {
        let (left, right) = rest.split(cut).ok()?;
        let [_, left_end] = left.domain().ok()?;
        let [right_start, _] = right.domain().ok()?;
        if left.evaluate(left_end).ok()?.sub(point).length() > reach || right.evaluate(right_start).ok()?.sub(point).length() > reach {
            return None;
        }
        pieces.push(right);
        rest = left;
    }
    pieces.push(rest);
    pieces.reverse();
    Some(pieces)
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
            // A straight profile skew to the axis sweeps a hyperboloid, not a
            // cone: only a profile in the start meridian reads as one.
            if !generatrix_is_meridional_half_ray(generatrix, frame) {
                return None;
            }
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
pub fn ruled_gate_census(surface: &NurbsSurface, tolerance: f64) -> Option<String> {
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
/// the full sections `intersect_plane_quadric` returns.
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
///   control point. A parabolic or hyperbolic section is built like an
///   elliptic one while the ARC stays clear of the generatrices parallel to the
///   plane: every point of it is then a finite image on one nappe and the
///   weights keep one sign. An arc that reaches such a generatrix (a closed rim
///   always does) has weights that change sign, and is refused by name rather
///   than silently mis-built — that generatrix never meets the plane.
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
                // Plane through the apex: none, one (tangent) or two rulings.
                if cone_apex_plane_rulings_enabled() {
                    return cone_apex_plane_rulings(plane, frame, *rho0, *rho1, *height, apex, apex_t, tolerance);
                }
                return None;
            }
            let normal = plane.normal;
            let project = |p: crate::Vec4| {
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
            };
            let Some(conic) = map_rational_curve(&circle, project) else {
                // A parabola or hyperbola: its arcs inside the band.
                if cone_plane_open_section_enabled() {
                    return cone_plane_open_section(frame, *height, apex, reference_rho, reference_z, k, normal, tolerance, &project);
                }
                return None;
            };
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

/// `BREP_CONE_APEX_PLANE_RULINGS` (default on, `=0` restores the marcher): a
/// plane through a cone's apex is answered by [`cone_apex_plane_rulings`].
fn cone_apex_plane_rulings_enabled() -> bool {
    crate::switch!(live "BREP_CONE_APEX_PLANE_RULINGS", true)
}

/// A plane through the apex of a cone or frustum meets its double cone in
/// rulings, in closed form. Measured from the apex along the axis toward the
/// band, the cone's cross-section at distance `z` is a circle of radius
/// `z tan α`, and the plane's trace there stands `z cot β` from the axis, `β`
/// the angle between the axis and the plane's normal. The plane dips into the
/// cone by `z (tan α − cot β)` at most, at the band's far end: within the
/// tolerance it is a tangent plane resting on one ruling, or touching only
/// the apex, and no section is returned, as
/// [`cylinder_parallel_plane_lines`] does for a plane resting on a cylinder.
/// Marching that tangency instead traces a ruling a square root of round-off
/// off the true one, which reads as a crossing of a few micro-radians and
/// classifies one side of the resting cone inside the plane's solid. Deeper,
/// the two rulings making the cone's half-angle with the axis inside the
/// plane are returned over the band.
#[allow(clippy::too_many_arguments)]
fn cone_apex_plane_rulings(
    plane: &PlaneData,
    frame: &RevolutionFrame,
    rho0: f64,
    rho1: f64,
    height: f64,
    apex: Vec3,
    apex_t: f64,
    tolerance: f64,
) -> Option<Vec<NurbsCurve>> {
    if !(apex_t.is_finite() && height.is_finite()) || height == 0.0 {
        return None;
    }
    let normal = plane.normal.normalized().ok()?;
    // The axial distances of the band's ends from the apex, and the axis
    // turned to point from the apex into the band.
    let apex_z = height * apex_t;
    let (end_a, end_b) = ((0.0 - apex_z).abs(), (height - apex_z).abs());
    let (near, far) = (end_a.min(end_b), end_a.max(end_b));
    let toward = if 0.5 * height - apex_z >= 0.0 { 1.0 } else { -1.0 };
    let axis = frame.axis.normalized().ok()?.scale(toward);
    let tan_alpha = (rho1 - rho0).abs() / height.abs();
    let cos_alpha = 1.0 / (1.0 + tan_alpha * tan_alpha).sqrt();
    let along = axis.dot(normal);
    let in_plane = axis.sub(normal.scale(along));
    let sin_beta = in_plane.length();
    if sin_beta <= 1e-12 {
        // The plane is perpendicular to the axis: it meets the cone at the
        // apex only (the perpendicular branch above takes the usual case).
        return Some(Vec::new());
    }
    let depth = far * (tan_alpha - along.abs() / sin_beta);
    if depth <= tolerance {
        return Some(Vec::new());
    }
    // A ruling direction d in the plane with d·axis = cos α: d = cos φ e1 ±
    // sin φ e2, e1 the axis's in-plane direction, e2 = n × e1.
    let e1 = in_plane.scale(1.0 / sin_beta);
    let e2 = normal.cross(e1);
    let cos_phi = (cos_alpha / sin_beta).min(1.0);
    let sin_phi = (1.0 - cos_phi * cos_phi).max(0.0).sqrt();
    // The apex projected onto the plane, so the rulings lie on it exactly.
    let foot = apex.sub(normal.scale(apex.sub(plane.origin).dot(normal)));
    let mut lines = Vec::new();
    for sign in [1.0, -1.0] {
        let direction = e1.scale(cos_phi).add(e2.scale(sign * sin_phi));
        let start = foot.add(direction.scale(near / cos_alpha));
        let end = foot.add(direction.scale(far / cos_alpha));
        lines.push(crate::make_line(start, end).ok()?);
    }
    Some(lines)
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





