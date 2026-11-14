//! A sweep of a plane section along straight runs, carried round each corner
//! by turning it about the corner.
//!
//! Each run is the section slid straight along it, ending flat at the corner.
//! At a corner the section is turned about the line through the corner square
//! to both runs, from square to the one run to square to the next. That line
//! lies in the section's plane and may cross the section, so the section is
//! cut along it and each part revolved by itself; the part outside the corner
//! fills the wedge the two runs leave open (for a disc, a piece of a ball),
//! and the part inside it lies where the runs overlap. The pieces are united.
use crate::{
    boolean_operation, boolean_operation_nary, extrude_profile_brep, face_from_surface_trimmed, make_plane, revolve_profile_brep, solid_from_polygons,
    BooleanOperation, BooleanOptions, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3, Vec4,
};

/// The prefix of every refusal of this operation.
pub const SWEEP_ROUND_CORNERS_REFUSAL: &str = "sweepRoundCorners";

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{SWEEP_ROUND_CORNERS_REFUSAL}: {message}"))
}

/// `curve` with every point of it moved by `map`, a rigid motion (which
/// carries a rational curve pole by pole).
fn moved(curve: &NurbsCurve, map: &dyn Fn(Vec3) -> Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let poles = curve.control_points.iter().map(|pole| Vec4::from_point(map(Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)), pole.w)).collect();
    NurbsCurve::new(curve.degree, curve.knots.clone(), poles).or_refuse(KernelStage::Collect, "sweepRoundCorners.move")
}

/// The solid `section` sweeps along the straight runs through `path`, turned
/// about each corner.
///
/// `section` is a closed loop of curves (or one closed curve) in the plane
/// through `path[0]` square to the first run. Consecutive runs may turn by any
/// angle short of a half turn; runs in one line simply continue.
///
/// Refuses a path of fewer than two points or with a run of no length, a
/// corner that turns straight back, and whatever the extrudes, the revolves
/// or the union refuse.
pub fn sweep_profile_round_corners(section: &[NurbsCurve], path: &[Vec3]) -> Result<BrepSolid, KernelRefusal> {
    if path.len() < 2 {
        return Err(input("path", "the path needs at least two points".into()));
    }
    if section.is_empty() {
        return Err(input("section", "the section is empty".into()));
    }
    let stage = KernelStage::Collect;
    let poles = |curves: &[NurbsCurve]| -> Vec<Vec3> {
        curves.iter().flat_map(|curve| &curve.control_points).map(|pole| Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)).collect()
    };
    let reach = poles(section).iter().map(|point| point.sub(path[0]).length()).fold(0.0, f64::max);
    if reach <= 0.0 {
        return Err(input("section", "the section has no extent".into()));
    }
    let mut section = section.to_vec();
    let mut parts = Vec::new();
    let mut fixed: Vec<Vec3> = Vec::new();
    for index in 0..path.len() - 1 {
        let run = path[index + 1].sub(path[index]);
        let along = run.normalized().map_err(|_| input("path", format!("run {index} has no length")))?;
        parts.push(extrude_profile_brep(&section, along, run.length())?);
        section = section.iter().map(|curve| moved(curve, &|point| point.add(run))).collect::<Result<_, _>>()?;
        let Some(after) = path.get(index + 2) else { break };
        let next = after.sub(path[index + 1]).normalized().map_err(|_| input("path", format!("run {} has no length", index + 1)))?;
        let turn = along.cross(next);
        if turn.length() <= 1e-12 {
            if along.dot(next) < 0.0 {
                return Err(input("corner", format!("the path turns straight back at point {}", index + 1)));
            }
            continue;
        }
        let corner = path[index + 1];
        let axis = turn.normalized().or_refuse(stage, "sweepRoundCorners.axis")?;
        let angle = turn.length().atan2(along.dot(next));
        // the section's plane at the corner: the axis, and the line across it
        let across = axis.cross(along);
        let span = 4.0 * reach;
        let plane = make_plane(corner.sub(axis.scale(span)).sub(across.scale(span)), axis, across, 2.0 * span, 2.0 * span).or_refuse(stage, "sweepRoundCorners.plane")?;
        let sheet = face_from_surface_trimmed(&plane, &[section.clone()])?;
        for side in [1.0, -1.0] {
            // the block on one side of the axis, holding that side of the section
            let at = |a: f64, b: f64, c: f64| corner.add(axis.scale(a * span)).add(across.scale(side * b * span)).add(along.scale(c * span));
            let mut faces = Vec::new();
            for s in [0.0, 1.0] {
                faces.push(vec![at(-1.0, s, -1.0), at(1.0, s, -1.0), at(1.0, s, 1.0), at(-1.0, s, 1.0)]);
                faces.push(vec![at(2.0 * s - 1.0, 0.0, -1.0), at(2.0 * s - 1.0, 1.0, -1.0), at(2.0 * s - 1.0, 1.0, 1.0), at(2.0 * s - 1.0, 0.0, 1.0)]);
                faces.push(vec![at(-1.0, 0.0, 2.0 * s - 1.0), at(1.0, 0.0, 2.0 * s - 1.0), at(1.0, 1.0, 2.0 * s - 1.0), at(-1.0, 1.0, 2.0 * s - 1.0)]);
            }
            let block = solid_from_polygons(&faces)?;
            let part = boolean_operation(&sheet, &block, BooleanOperation::Intersect, &BooleanOptions::default())?;
            for face in part.shells.iter().flat_map(|shell| &shell.faces) {
                if face.loops.len() != 1 {
                    return Err(KernelRefusal::unsupported(KernelStage::Fragment, "section_holes", format!("{SWEEP_ROUND_CORNERS_REFUSAL}: a section with a hole is not turned about a corner")));
                }
                let mut outline = Vec::new();
                for coedge in &face.loops[0].coedges {
                    let edge = part.edges.iter().find(|edge| edge.id == coedge.edge_id).ok_or_else(|| input("section", "the cut section refers to a missing edge".into()))?;
                    // the edge is its curve between t0 and t1
                    let [low, high] = edge.curve.domain().or_refuse(stage, "sweepRoundCorners.domain")?;
                    let margin = 1e-9 * (high - low);
                    let mut piece = edge.curve.clone();
                    if edge.t1 < high - margin {
                        piece = piece.split(edge.t1).or_refuse(stage, "sweepRoundCorners.trim")?.0;
                    }
                    if edge.t0 > low + margin {
                        piece = piece.split(edge.t0).or_refuse(stage, "sweepRoundCorners.trim")?.1;
                    }
                    outline.push(if coedge.forward { piece } else { piece.reversed().or_refuse(stage, "sweepRoundCorners.reverse")? });
                }
                let turned = revolve_profile_brep(&outline, corner, axis, angle).or_refuse(KernelStage::Fragment, "sweepRoundCorners.turn")?;
                // where the section crosses the corner's axis the turn has a point that does not move
                for vertex in &turned.vertices {
                    let d = vertex.point.sub(corner);
                    if d.sub(axis.scale(d.dot(axis))).length() <= 1e-9 * reach {
                        fixed.push(vertex.point);
                    }
                }
                parts.push(turned);
            }
        }
        section = section.iter().map(|curve| moved(curve, &|point| corner.add(point.sub(corner).rotate_about(axis, angle)))).collect::<Result<_, _>>()?;
    }
    if parts.len() == 1 {
        return Ok(parts.remove(0));
    }
    let mut solid = boolean_operation_nary(&parts, BooleanOperation::Union)?;
    if axis_points_on() {
        // Both runs and the turn touch along the section's tangents at a point
        // on the corner's axis, so the union finds its vertex there only to the
        // square root of round-off; the point itself is known.
        let mut put = Vec::new();
        for vertex in &mut solid.vertices {
            if let Some(point) = fixed.iter().find(|point| point.sub(vertex.point).length() <= 1e-6 * reach) {
                vertex.point = *point;
                put.push((vertex.id, *point));
            }
        }
        // and the edges that end there end there: an edge that is the whole of
        // its curve has that end's control point at the vertex
        for edge in &mut solid.edges {
            let Ok([low, high]) = edge.curve.domain() else { continue };
            let margin = 1e-12 * (high - low);
            let last = edge.curve.control_points.len() - 1;
            for (vertex, at_end, index) in [(edge.start_vertex_id, (edge.t0 - low).abs() <= margin, 0), (edge.end_vertex_id, (edge.t1 - high).abs() <= margin, last)] {
                let Some((_, point)) = put.iter().find(|(id, _)| *id == vertex) else { continue };
                if at_end {
                    let control = &mut edge.curve.control_points[index];
                    (control.x, control.y, control.z) = (point.x * control.w, point.y * control.w, point.z * control.w);
                }
            }
        }
    }
    Ok(solid)
}

/// `BREP_SWEEP_CORNER_AXIS_POINTS` (default on, `=0` restores): the vertices
/// of a sweep with round corners that lie where the section crosses a
/// corner's axis are put on those points exactly.
fn axis_points_on() -> bool {
    crate::switch!("BREP_SWEEP_CORNER_AXIS_POINTS", true)
}

