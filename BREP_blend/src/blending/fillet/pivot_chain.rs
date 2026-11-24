//! A constant round of closed tangent chains of straight and circular edges
//! where some circular edge's ball centre lies on its own axis (the round as
//! tight as the corner it turns: a pocket's floor round of the pocket's corner
//! radius). There the ball pivots about one point and its wall is a sphere
//! patch; the chain's march meets a singular station Newton and the radius
//! relief would build a smaller round in its place. Each edge's exact cutter
//! (its section extruded along a straight edge, revolved about a circular
//! edge's axis) is flush at the edge's ends, and the cutters, which meet in
//! their shared end sections, are joined to the body in one boolean.
use super::*;
use super::edges::{per_edge_name, refuse_invalid_result, resolve_edge_by_point};

/// `BREP_FILLET_PIVOT_CHAIN` (default on, `=0` leaves such a chain to the
/// march and its relief as before).
fn pivot_chain_on() -> bool {
    switch!("BREP_FILLET_PIVOT_CHAIN", true)
}

/// The round of `edge_points`' edges by per-edge exact cutters, or `None`
/// where the selection is not that (see the module).
pub(super) fn pivot_chain(solid: &BrepSolid, edge_points: &[Vec3], edge_names: Option<&[String]>, radius: f64, chamfer: bool, name: Option<&str>) -> Option<Result<BrepSolid, KernelRefusal>> {
    // One boolean takes the body and every cutter (at most 255 operands).
    if !pivot_chain_on() || chamfer || edge_points.len() < 2 || edge_points.len() > 254 || !(radius > 0.0 && radius.is_finite()) {
        return None;
    }
    let ids: Vec<u64> = edge_points.iter().map(|p| resolve_edge_by_point(solid, *p).ok()).collect::<Option<_>>()?;
    let mut unique = ids.clone();
    unique.sort_unstable();
    unique.dedup();
    if unique.len() != ids.len() {
        return None;
    }
    let is_plane = |face: &crate::FaceRecord| matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. }));
    let mut crosses = Vec::with_capacity(ids.len());
    let mut pivots = false;
    for &id in &ids {
        let cross = analyze_edge(solid, id, radius).ok()?;
        match &cross.path {
            EdgePath::Straight { .. } => {
                let (_, faces) = super::analyze::edge_and_faces(solid, id).ok()?;
                if faces.len() != 2 || !faces.iter().all(|f| is_plane(f)) {
                    return None;
                }
            }
            EdgePath::Circular { center, axis, .. } => {
                // The ball's centre at the section: on the corner's bisector
                // r / sin(θ/2) in; on the circle's own axis it pivots.
                let cos = cross.into_first.dot(cross.into_second).clamp(-1.0, 1.0);
                let theta = cos.acos();
                let bisector = cross.into_first.add(cross.into_second).normalized().ok()?;
                let ball = cross.start.add(bisector.scale(radius / (0.5 * theta).sin()));
                let off = ball.sub(*center);
                let from_axis = off.sub(axis.scale(off.dot(*axis))).length();
                if from_axis <= 1e-9 * (1.0 + radius) {
                    pivots = true;
                }
            }
        }
        crosses.push(cross);
    }
    if !pivots || !crosses.iter().all(|c| c.convex == crosses[0].convex) {
        return None;
    }
    // Closed tangent chains: every end vertex shared by exactly two selected
    // edges, their directions there one line.
    let mut ends: Vec<(Vec3, Vec3)> = Vec::new();
    for &id in &ids {
        let edge = solid.edges.iter().find(|e| e.id == id)?;
        let span = edge.t1 - edge.t0;
        let at = |t: f64| edge.curve.evaluate(t).ok();
        let (p0, p1) = (at(edge.t0)?, at(edge.t1)?);
        let (q0, q1) = (at(edge.t0 + 1e-7 * span)?, at(edge.t1 - 1e-7 * span)?);
        ends.push((p0, q0.sub(p0).normalized().ok()?));
        ends.push((p1, p1.sub(q1).normalized().ok()?));
    }
    // The selection's own size (not its distance from the origin).
    let (lo, hi) = ends.iter().fold(([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]), |(lo, hi), (p, _)| ([lo[0].min(p.x), lo[1].min(p.y), lo[2].min(p.z)], [hi[0].max(p.x), hi[1].max(p.y), hi[2].max(p.z)]));
    let scale = (hi[0] - lo[0]).hypot(hi[1] - lo[1]).hypot(hi[2] - lo[2]).max(radius);
    for (i, (p, d)) in ends.iter().enumerate() {
        let partners: Vec<usize> = (0..ends.len()).filter(|&j| j / 2 != i / 2 && ends[j].0.sub(*p).length() <= 1e-9 * scale).collect();
        if partners.len() != 1 || d.cross(ends[partners[0]].1).length() > 1e-5 {
            return None;
        }
    }
    Some(build(solid, &ids, &crosses, radius, edge_names, name))
}

/// The fractions along each edge its contacts are checked at.
const STATIONS: [f64; 17] = [0.0, 0.0625, 0.125, 0.1875, 0.25, 0.3125, 0.375, 0.4375, 0.5, 0.5625, 0.625, 0.6875, 0.75, 0.8125, 0.875, 0.9375, 1.0];

fn build(solid: &BrepSolid, ids: &[u64], crosses: &[EdgeCross], radius: f64, edge_names: Option<&[String]>, name: Option<&str>) -> Result<BrepSolid, KernelRefusal> {
    // Each edge's contacts on its own faces, inside their trims (a wall or
    // floor shorter than the round would leave the fill unclipped), at the
    // section's place (the edge's start) and at its middle and end: the
    // section slides along a straight edge and turns about a circular edge's
    // axis; at sixteen intervals.
    for (&id, cross) in ids.iter().zip(crosses) {
        let cos = cross.into_first.dot(cross.into_second).clamp(-1.0, 1.0);
        let setback = radius / (0.5 * cos.acos()).tan();
        let (_, faces) = super::analyze::edge_and_faces(solid, id)?;
        let stations: Vec<(Vec3, Vec3, Vec3)> = match &cross.path {
            EdgePath::Straight { direction, length } => STATIONS.iter().map(|f| (cross.start.add(direction.scale(f * length)), cross.into_first, cross.into_second)).collect(),
            EdgePath::Circular { center, axis, sweep } => STATIONS
                .iter()
                .map(|f| {
                    let angle = f * sweep;
                    let (c, s) = (angle.cos(), angle.sin());
                    let turn = |v: Vec3| v.scale(c).add(axis.cross(v).scale(s)).add(axis.scale(axis.dot(v) * (1.0 - c)));
                    (center.add(turn(cross.start.sub(*center))), turn(cross.into_first), turn(cross.into_second))
                })
                .collect(),
        };
        for (at, first, second) in stations {
            for (face, into) in faces.iter().zip([first, second]) {
                let contact = at.add(into.scale(setback));
                let on = crate::project_point_to_surface(&face.surface, contact).ok().filter(|foot| foot.distance <= 1e-7 * (1.0 + radius)).is_some_and(|foot| {
                    crate::parameter_point_in_face(face, crate::Vec2 { x: foot.u, y: foot.v }, 1e-9).is_ok_and(|class| class != crate::classification::PolygonClass::Outside)
                });
                if !on {
                    return Err(KernelRefusal::input(KernelStage::Classify, "support_extent", format!("fillet_edges: the round's contact {setback} from edge {id} runs off face {}", face.id)));
                }
            }
        }
    }
    let mut operands = vec![solid.clone()];
    for (index, cross) in crosses.iter().enumerate() {
        let profile = super::tool::cross_section_profile(cross, radius, false)?;
        operands.push(super::tool::build_exact_tool(cross, &profile, 0.0, super::tool::ToolEnds::default(), per_edge_name(edge_names, name, index))?);
    }
    let operation = if crosses[0].convex { crate::BooleanOperation::Subtract } else { crate::BooleanOperation::Union };
    let result = crate::boolean_operation_nary(&operands, operation)?;
    let result = super::tool::settle_cutter_scaffold(result);
    refuse_invalid_result(&result, "fillet_edges")?;
    Ok(result)
}
