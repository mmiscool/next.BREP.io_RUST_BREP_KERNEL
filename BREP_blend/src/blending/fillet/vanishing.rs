//! A round (or chamfer) whose radius runs linearly to nought at one end of a
//! straight edge between two planes: the cutter is the cone, apex at that end
//! vertex, over the round's section at the other end.
use super::*;
use super::edges::{per_edge_name, refuse_invalid_result, resolve_edge_by_point};
use crate::{NurbsSurface, Vec4};

/// `BREP_FILLET_VANISHING_CONE` (default on, `=0` refuses as before): the
/// cone cutter of [`vanishing_round`].
fn vanishing_cone_on() -> bool {
    switch!("BREP_FILLET_VANISHING_CONE", true)
}

/// The round of ONE straight edge between two planes under `law` when the law
/// is linear along the edge and nought at exactly one end, or `None` where it
/// is not that (the law's other paths then answer).
///
/// Every section square to the edge is the section at the full end scaled
/// from the vanishing end's vertex by the radius's share there (the contacts,
/// the ball's centre and its arc all scale with `r` in a fixed wedge), so the
/// material the round takes is the cone with its apex at that vertex over
/// the full end's section: two triangles on the faces and an oblique circular
/// cone between them (a plane between them for a chamfer). The cutter runs
/// on past the full end by its radius, as a constant round's runs into its
/// neighbours, and is cut from the solid as a constant round's is.
pub(super) fn vanishing_round(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    law: &crate::law::RadiusLaw,
    chamfer: bool,
    name: Option<&str>,
    entry: &str,
) -> Option<Result<BrepSolid, KernelRefusal>> {
    if !vanishing_cone_on() || edge_points.len() != 1 {
        return None;
    }
    // One straight ramp over the whole edge (read on the law's pieces, not
    // sampled), nought at exactly one end.
    let (r0, r1) = law.linear_ends()?;
    let full = r0.max(r1);
    if !((r0 == 0.0) != (r1 == 0.0)) || !(full.is_finite() && full > 0.0) {
        return None;
    }
    let edge_id = resolve_edge_by_point(solid, edge_points[0]).ok()?;
    let (_, faces) = super::analyze::edge_and_faces(solid, edge_id).ok()?;
    if faces.len() != 2 || !faces.iter().all(|face| matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. }))) {
        return None;
    }
    let cross = analyze_edge(solid, edge_id, full).ok()?;
    let EdgePath::Straight { direction, length } = cross.path else { return None };
    // Convex only: a concave cone, run on past the full end, would add
    // material beyond the solid's end face (a fill is clipped there; this
    // cutter is not).
    if !cross.convex {
        return None;
    }
    // The contacts must stay on their faces, read as the law's other paths
    // read them (either way along the edge).
    let forward = super::chamfer_extent::check_ball_extent(solid, edge_id, &|f| law.radius_at_fraction(f), entry);
    let backward = super::chamfer_extent::check_ball_extent(solid, edge_id, &|f| law.radius_at_fraction(1.0 - f), entry);
    if let (Err(too_large), Err(_)) = (forward, backward) {
        return Some(Err(too_large));
    }
    Some(build(solid, &cross, direction, length, r0 == 0.0, full, chamfer, per_edge_name(edge_names, name, 0), entry))
}

#[allow(clippy::too_many_arguments)]
fn build(
    solid: &BrepSolid,
    cross: &EdgeCross,
    direction: Vec3,
    length: f64,
    vanishes_at_start: bool,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
    entry: &str,
) -> Result<BrepSolid, KernelRefusal> {
    let far = cross.start.add(direction.scale(length));
    let (apex, base) = if vanishes_at_start { (cross.start, far) } else { (far, cross.start) };
    let at_base = EdgeCross { start: base, path: EdgePath::Straight { direction, length }, into_first: cross.into_first, into_second: cross.into_second, convex: cross.convex };
    let section = super::tool::cross_section_profile(&at_base, radius, chamfer)?;
    // Past the full end by the radius: the section scaled from the apex.
    let k = 1.0 + radius / length;
    let scaled: Vec<NurbsCurve> = section
        .iter()
        .map(|curve| {
            let mut out = curve.clone();
            for cp in out.control_points.iter_mut() {
                cp.x = apex.x * cp.w + k * (cp.x - apex.x * cp.w);
                cp.y = apex.y * cp.w + k * (cp.y - apex.y * cp.w);
                cp.z = apex.z * cp.w + k * (cp.z - apex.z * cp.w);
            }
            out
        })
        .collect();
    let fragment = |message: String| KernelRefusal::internal(KernelStage::Fragment, "vanishing_cone", format!("{entry}: {message}"));
    // The cone over the arc (the second section curve) is the round; every
    // other face is scaffold. Named on the sheets, before they are joined.
    let wall = super::tool::scaffold_tag(name);
    let named = |mut sheet: BrepSolid, round: bool| {
        for face in sheet.shells.iter_mut().flat_map(|shell| shell.faces.iter_mut()) {
            face.name = if round { name.map(str::to_string) } else { Some(wall.clone()) };
        }
        sheet
    };
    let mut sheets = Vec::with_capacity(scaled.len() + 1);
    for (index, curve) in scaled.iter().enumerate() {
        // Ruled from the curve (v = 0) to the apex (v = 1), the curve's own weights.
        let rows = curve.control_points.iter().map(|cp| vec![*cp, Vec4 { x: apex.x * cp.w, y: apex.y * cp.w, z: apex.z * cp.w, w: cp.w }]).collect();
        let surface = NurbsSurface::new(curve.degree, 1, curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], rows).map_err(fragment)?;
        sheets.push(named(crate::face_from_surface(&surface)?, index == 1));
    }
    // The cap square to the edge, through the scaled section.
    let cap_origin = apex.add(base.sub(apex).scale(k));
    let u = cross.into_first.sub(direction.scale(cross.into_first.dot(direction))).normalized().map_err(|_| fragment("the section's plane".into()))?;
    let v = direction.cross(u);
    let reach = 4.0 * k * radius / (1.0 - cross.into_first.dot(cross.into_second).abs()).max(1e-3).sqrt() + 1.0;
    let plane = crate::make_plane(cap_origin.sub(u.scale(reach)).sub(v.scale(reach)), u, v, 2.0 * reach, 2.0 * reach).map_err(fragment)?;
    sheets.push(named(crate::face_from_surface_trimmed(&plane, &[scaled.clone()])?, false));
    // Joined as the kernel's polygon solids are (the sheets' shared rulings
    // are the same lines, read from different surfaces).
    let tool = crate::solid_from_sheets(&sheets, (1e-8 * (length + radius)).max(1e-7))?;
    let result = super::tool::apply_tool(solid, &tool, cross.convex).map(super::tool::settle_cutter_scaffold)?;
    refuse_invalid_result(&result, entry)?;
    Ok(result)
}
