//! A free end across TWO faces.
//!
//! The §6.9 free end closes a stripe on the one face sharing a boundary edge
//! with each mate. Where two faces stand across the corner, joined at the
//! vertex by a third edge, the wall is trimmed by each of them instead: the
//! section runs on the first face's extension up to the plane through the
//! blended edge and the shared edge, and on the second's beyond it. Both faces
//! grow to the wall. Two surfaces that do not meet there (a plane against the
//! cone it touches along one generator) leave a step on that plane, one small
//! triangle whose apex is the old vertex.
use super::*;
use super::support::EndCorner;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Whether a two-face end is built by surgery: on unless
/// `BREP_FILLET_SPLIT_FLOOR_END=0`/`off`.
fn split_floor_on() -> bool {
    switch!(live "BREP_FILLET_SPLIT_FLOOR_END", true)
}

const SECTIONS: usize = 16;

#[derive(Clone, Copy)]
enum Floor {
    Plane { origin: Vec3, normal: Vec3 },
    Cone { apex: Vec3, axis: Vec3, slope: f64 },
}

impl Floor {
    fn height(&self, point: Vec3) -> f64 {
        match *self {
            Floor::Plane { origin, normal } => point.sub(origin).dot(normal),
            Floor::Cone { apex, axis, slope } => {
                let delta = point.sub(apex);
                let along = delta.dot(axis);
                along - slope * delta.sub(axis.scale(along)).length()
            }
        }
    }
}

/// The plan for one two-face end; the first piece of the section lives in the
/// [`EndSurgery`] that carries this.
#[derive(Clone)]
pub(in crate::blend) struct SplitFloor {
    second_face: u64,
    shared_edge: u64,
    q1: Vec3,
    q2: Vec3,
    /// The two floors meet the wall apart, so a step edge joins the pieces.
    step: bool,
    step_blend_pcurve: NurbsCurve,
    second_curve: NurbsCurve,
    second_blend_pcurve: NurbsCurve,
    second_end_pcurve: NurbsCurve,
    first_divider_pcurve: NurbsCurve,
    second_divider_pcurve: NurbsCurve,
    /// Which floors are cones, whose carriers are rebuilt after the splice.
    cones: [bool; 2],
}

fn refuse(what: &'static str) -> KernelRefusal {
    KernelRefusal::internal(KernelStage::Sew, "split_floor", what)
}

fn face_of<'a>(solid: &'a BrepSolid, id: u64) -> Option<&'a FaceRecord> {
    solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == id)
}

fn uses_edge(face: &FaceRecord, edge: u64) -> bool {
    face.loops.iter().flat_map(|l| &l.coedges).any(|coedge| coedge.edge_id == edge)
}

fn touches_vertex(solid: &BrepSolid, face: &FaceRecord, vertex: u64) -> bool {
    face.loops.iter().flat_map(|l| &l.coedges).any(|coedge| {
        solid.edges.iter().any(|edge| edge.id == coedge.edge_id && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex))
    })
}

fn floor_of(face: &FaceRecord, corner: Vec3, axis: Vec3, generator: Vec3) -> Option<Floor> {
    match face.surface.analytic()? {
        crate::AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
            let normal = u_dir.cross(*v_dir).normalized().ok()?;
            (corner.sub(*origin).dot(normal).abs() <= 1e-7).then_some(Floor::Plane { origin: *origin, normal })
        }
        analytic => {
            let frame = analytic.frame()?;
            if frame.axis.dot(axis).abs() < 1.0 - 1e-9 {
                return None;
            }
            let offset = corner.sub(frame.origin);
            if offset.sub(frame.axis.scale(offset.dot(frame.axis))).length() > 1e-7 * (1.0 + offset.length()) {
                return None;
            }
            if let crate::AnalyticSurface::Revolution { generatrix, .. } = analytic {
                let [t0, t1] = generatrix.domain().ok()?;
                let (a, b, m) = (generatrix.evaluate(t0).ok()?, generatrix.evaluate(t1).ok()?, generatrix.evaluate(0.5 * (t0 + t1)).ok()?);
                if m.sub(a.add(b).scale(0.5)).length() > 1e-9 * (1.0 + a.sub(b).length()) {
                    return None;
                }
            }
            let rise = generator.dot(axis);
            let run = generator.sub(axis.scale(rise)).length();
            (run > 1e-9).then_some(Floor::Cone { apex: corner, axis, slope: rise / run })
        }
    }
}

/// Where the section at `z` meets `floor`, by bisection over the half of the
/// rows holding this end.
fn crossing_at(rows: &FittedRows, floor: &Floor, z: f64, at_start: bool) -> Option<(f64, Vec3)> {
    let [d0, d1] = rows.u_domain;
    let middle = 0.5 * (d0 + d1);
    let (mut low, mut high) = if at_start { (d0, middle) } else { (middle, d1) };
    let at = |u: f64| rows.surface.evaluate(u, z).ok();
    let mut low_height = floor.height(at(low)?);
    if low_height * floor.height(at(high)?) > 0.0 {
        return None;
    }
    for _ in 0..80 {
        let mid = 0.5 * (low + high);
        let height = floor.height(at(mid)?);
        if low_height * height <= 0.0 {
            high = mid;
        } else {
            low = mid;
            low_height = height;
        }
    }
    let u = 0.5 * (low + high);
    Some((u, at(u)?))
}

/// The affine chart of a plane carrier, unclamped; a cone's trims are rebuilt
/// with its carrier, so it answers `None`.
fn plane_chart(face: &FaceRecord) -> Option<impl Fn(Vec3) -> Vec3> {
    if !matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
        return None;
    }
    let ([u0, u1], [v0, v1]) = (face.surface.domain_u().ok()?, face.surface.domain_v().ok()?);
    let origin = face.surface.evaluate(u0, v0).ok()?;
    let du = face.surface.evaluate(u1, v0).ok()?.sub(origin);
    let dv = face.surface.evaluate(u0, v1).ok()?.sub(origin);
    let (lu, lv) = (du.length(), dv.length());
    let (eu, ev) = (du.normalized().ok()?, dv.normalized().ok()?);
    Some(move |point: Vec3| {
        let delta = point.sub(origin);
        Vec3::new(u0 + delta.dot(eu) / lu * (u1 - u0), v0 + delta.dot(ev) / lv * (v1 - v0), 0.0)
    })
}

/// One piece of the section across `z0..z1` on `floor`: the 3D curve, its
/// pcurve on the wall and its pcurve on the floor.
fn piece(rows: &FittedRows, floor: &Floor, face: &FaceRecord, z0: f64, z1: f64, at_start: bool) -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
    let parameters: Vec<f64> = (0..=SECTIONS).map(|k| k as f64 / SECTIONS as f64).collect();
    let (mut points, mut wall) = (Vec::new(), Vec::new());
    for t in &parameters {
        let z = z0 + (z1 - z0) * t;
        let (u, point) = crossing_at(rows, floor, z, at_start)?;
        points.push(point);
        wall.push(Vec3::new(u, z, 0.0));
    }
    let on_floor = match plane_chart(face) {
        Some(chart) => crate::fit::interpolate_curve(&points.iter().map(|p| chart(*p)).collect::<Vec<_>>(), 3, &parameters).ok()?,
        None => placeholder()?,
    };
    Some((crate::fit::interpolate_curve(&points, 3, &parameters).ok()?, crate::fit::interpolate_curve(&wall, 3, &parameters).ok()?, on_floor))
}

/// A trim on a cone, replaced when its carrier is rebuilt.
fn placeholder() -> Option<NurbsCurve> {
    crate::make_line(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0)).ok()
}

/// The two-face end at `vertex`, with the first piece of its section, or
/// `None` when this end is not that configuration.
pub(in crate::blend) fn plan_split_floor(
    solid: &BrepSolid,
    blended: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    rows: &FittedRows,
    vertex: u64,
    at_start: bool,
    first_edge_id: u64,
    second_edge_id: u64,
) -> Option<(u64, SplitFloor, (NurbsCurve, NurbsCurve, NurbsCurve))> {
    if !split_floor_on() || !rows.exact_extrusion {
        return None;
    }
    let faces = || solid.shells.iter().flat_map(|shell| &shell.faces);
    let across = |edge: u64, mate: u64| faces().find(|face| face.id != mate && uses_edge(face, edge));
    let (floor_a, floor_b) = (across(first_edge_id, first.face.id)?, across(second_edge_id, second.face.id)?);
    if floor_a.id == floor_b.id || faces().filter(|face| touches_vertex(solid, face, vertex)).count() != 4 {
        return None;
    }
    let mut shared = solid.edges.iter().filter(|edge| {
        !edge.degenerate && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex) && uses_edge(floor_a, edge.id) && uses_edge(floor_b, edge.id)
    });
    let shared_edge = shared.next()?;
    if shared.next().is_some() {
        return None;
    }
    let corner = solid.vertices.iter().find(|v| v.id == vertex)?.point;
    let axis = super::super::network::tangent_away_from(blended, vertex).ok()?;
    let generator = super::super::network::tangent_away_from(shared_edge, vertex).ok()?;
    let (first_floor, second_floor) = (floor_of(floor_a, corner, axis, generator)?, floor_of(floor_b, corner, axis, generator)?);
    // The section changes floor on the plane through the edge and the shared edge.
    let divider = axis.cross(generator).normalized().ok()?;
    let side = |z: f64| -> Option<f64> { Some(crossing_at(rows, &first_floor, z, at_start)?.1.sub(corner).dot(divider)) };
    let (mut low, mut high) = (0.0_f64, 1.0_f64);
    let mut low_side = side(low)?;
    if low_side * side(high)? >= 0.0 {
        return None;
    }
    for _ in 0..80 {
        let mid = 0.5 * (low + high);
        let here = side(mid)?;
        if low_side * here <= 0.0 {
            high = mid;
        } else {
            low = mid;
            low_side = here;
        }
    }
    let z = 0.5 * (low + high);
    let (u1, q1) = crossing_at(rows, &first_floor, z, at_start)?;
    let (u2, q2) = crossing_at(rows, &second_floor, z, at_start)?;
    // The surgery GROWS both floors to the wall: the wall stands past the old
    // vertex, and each divider runs on from it along the shared edge's line.
    // Where the wall cuts the shared edge instead, as the third chamfer at a
    // mitered cube corner does, the floors would shrink, each divider would run
    // back over the shared edge from the old vertex to the wall, and both
    // floors would keep a zero-width spike to the old vertex. The measures and
    // `validate` cannot see it; the mesh can (two nodes at the miter point).
    // That end is not this surgery's.
    let band = 1e-9 * (1.0 + corner.length());
    if q1.sub(corner).dot(generator) > band || q2.sub(corner).dot(generator) > band {
        return None;
    }
    let first_piece = piece(rows, &first_floor, floor_a, 0.0, z, at_start)?;
    let second_piece = piece(rows, &second_floor, floor_b, z, 1.0, at_start)?;
    let divider_pcurve = |face: &FaceRecord, end: Vec3| match plane_chart(face) {
        Some(chart) => crate::make_line(chart(corner), chart(end)).ok(),
        None => placeholder(),
    };
    Some((
        floor_a.id,
        SplitFloor {
            second_face: floor_b.id,
            shared_edge: shared_edge.id,
            q1,
            q2,
            step: q1.sub(q2).length() > 1e-9,
            step_blend_pcurve: crate::make_line(Vec3::new(u1, z, 0.0), Vec3::new(u2, z, 0.0)).ok()?,
            second_curve: second_piece.0,
            second_blend_pcurve: second_piece.1,
            second_end_pcurve: second_piece.2,
            first_divider_pcurve: divider_pcurve(floor_a, q1)?,
            second_divider_pcurve: divider_pcurve(floor_b, q2)?,
            cones: [matches!(first_floor, Floor::Cone { .. }), matches!(second_floor, Floor::Cone { .. })],
        },
        first_piece,
    ))
}

/// The edges and vertices a two-face end adds.
pub(in crate::blend) struct SplitCommit {
    pub(in crate::blend) q1_vertex: u64,
    step_edge: Option<u64>,
    second_edge: u64,
    first_divider: u64,
    second_divider: u64,
}

fn push_edge(result: &mut BrepSolid, id: u64, curve: NurbsCurve, from: u64, to: u64) -> Result<(), KernelRefusal> {
    let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    result.edges.push(EdgeRecord { id, curve, t0, t1, start_vertex_id: from, end_vertex_id: to, degenerate: false, name: None });
    Ok(())
}

pub(in crate::blend) fn commit_split_end(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    plan: &SplitFloor,
    second_rim: u64,
    corner: u64,
) -> Result<SplitCommit, KernelRefusal> {
    let corner_point = result.vertices.iter().find(|v| v.id == corner).ok_or_else(|| refuse("the corner vertex is missing"))?.point;
    let line = |a: Vec3, b: Vec3| crate::make_line(a, b).or_refuse(KernelStage::Refine, "split_floor_line");
    let q1_vertex = take_id();
    result.vertices.push(VertexRecord { id: q1_vertex, point: plan.q1 });
    let q2_vertex = if plan.step {
        let id = take_id();
        result.vertices.push(VertexRecord { id, point: plan.q2 });
        id
    } else {
        q1_vertex
    };
    let second_edge = take_id();
    push_edge(result, second_edge, plan.second_curve.clone(), q2_vertex, second_rim)?;
    let first_divider = take_id();
    push_edge(result, first_divider, line(corner_point, plan.q1)?, corner, q1_vertex)?;
    let (step_edge, second_divider) = if plan.step {
        let step = take_id();
        push_edge(result, step, line(plan.q1, plan.q2)?, q1_vertex, q2_vertex)?;
        let divider = take_id();
        push_edge(result, divider, line(corner_point, plan.q2)?, corner, q2_vertex)?;
        (Some(step), divider)
    } else {
        (None, first_divider)
    };
    Ok(SplitCommit { q1_vertex, step_edge, second_edge, first_divider, second_divider })
}

/// The wall's coedges across this end, first mate to second when forward.
pub(in crate::blend) fn split_slot(
    end: &EndSurgery,
    plan: &SplitFloor,
    commit: &SplitCommit,
    first_edge: u64,
    forward: bool,
    take_id: &mut dyn FnMut() -> u64,
) -> Result<Vec<CoedgeRecord>, KernelRefusal> {
    let mut pieces = vec![(first_edge, end.transverse_blend_pcurve.clone())];
    if let Some(step) = commit.step_edge {
        pieces.push((step, plan.step_blend_pcurve.clone()));
    }
    pieces.push((commit.second_edge, plan.second_blend_pcurve.clone()));
    if !forward {
        pieces.reverse();
    }
    pieces
        .into_iter()
        .map(|(edge_id, pcurve)| {
            let pcurve = if forward { pcurve } else { pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")? };
            Ok(CoedgeRecord { id: take_id(), edge_id, forward, pcurve })
        })
        .collect()
}

/// Both floors' corners, located before the boundaries are trimmed.
pub(in crate::blend) fn locate_split_corners(
    result: &BrepSolid,
    end: &EndSurgery,
    plan: &SplitFloor,
    corner: u64,
) -> Result<[EndCorner; 2], KernelRefusal> {
    let locate = |face: u64, boundary: u64| {
        let face = face_of(result, face).ok_or_else(|| refuse("a floor face is missing"))?;
        locate_end_corner(result, face, corner, boundary, plan.shared_edge).ok_or_else(|| refuse("a floor's corner was not found"))
    };
    Ok([locate(end.end_face_id, end.first_edge_id)?, locate(plan.second_face, end.second_edge_id)?])
}

fn splice(result: &mut BrepSolid, face: u64, corner: &EndCorner, chain: Vec<CoedgeRecord>) -> Result<(), KernelRefusal> {
    let face = result.shells.iter_mut().flat_map(|shell| &mut shell.faces).find(|f| f.id == face).ok_or_else(|| refuse("a floor face is lost"))?;
    let loop_record = face.loops.get_mut(corner.loop_index).ok_or_else(|| refuse("a floor loop is lost"))?;
    let poles: Vec<CoedgeRecord> = loop_record.coedges.iter().filter(|c| corner.pole_coedge_ids.contains(&c.id)).cloned().collect();
    let mut rebuilt = Vec::with_capacity(loop_record.coedges.len() + chain.len());
    let mut chain = Some(chain);
    for coedge in &loop_record.coedges {
        if corner.pole_coedge_ids.contains(&coedge.id) {
            continue;
        }
        rebuilt.push(coedge.clone());
        if coedge.id == corner.x_coedge_id {
            let chain = chain.take().ok_or_else(|| refuse("the chain was spliced twice"))?;
            // The pole stays beside the shared edge.
            if corner.x_is_first {
                rebuilt.extend(chain);
                rebuilt.extend(poles.iter().cloned());
            } else {
                rebuilt.extend(poles.iter().cloned());
                rebuilt.extend(chain);
            }
        }
    }
    if chain.is_some() {
        return Err(refuse("a floor's corner coedge is lost"));
    }
    loop_record.coedges = rebuilt;
    Ok(())
}

/// Grow both floors to the wall, add the step, and rebuild any cone carrier.
pub(in crate::blend) fn finish_split_end(
    result: &mut BrepSolid,
    take_id: &mut dyn FnMut() -> u64,
    end: &EndSurgery,
    plan: &SplitFloor,
    commit: &SplitCommit,
    first_edge: u64,
    corners: &[EndCorner; 2],
    corner: u64,
    forward: bool,
    name: Option<&str>,
) -> Result<(), KernelRefusal> {
    let reversed = |curve: &NurbsCurve| curve.reversed().or_refuse(KernelStage::Refine, "reversed");
    // The wall uses each piece opposite to its floor.
    if corners[0].x_is_first == forward || corners[1].x_is_first != forward {
        return Err(refuse("the floors' loops disagree with the wall's direction"));
    }
    let chain_a = if corners[0].x_is_first {
        vec![(first_edge, true, end.transverse_end_pcurve.clone()), (commit.first_divider, false, reversed(&plan.first_divider_pcurve)?)]
    } else {
        vec![(commit.first_divider, true, plan.first_divider_pcurve.clone()), (first_edge, false, reversed(&end.transverse_end_pcurve)?)]
    };
    let chain_b = if corners[1].x_is_first {
        vec![(commit.second_edge, false, reversed(&plan.second_end_pcurve)?), (commit.second_divider, false, reversed(&plan.second_divider_pcurve)?)]
    } else {
        vec![(commit.second_divider, true, plan.second_divider_pcurve.clone()), (commit.second_edge, true, plan.second_end_pcurve.clone())]
    };
    let mut build = |chain: Vec<(u64, bool, NurbsCurve)>| -> Vec<CoedgeRecord> {
        chain.into_iter().map(|(edge_id, forward, pcurve)| CoedgeRecord { id: take_id(), edge_id, forward, pcurve }).collect()
    };
    let (chain_a, chain_b) = (build(chain_a), build(chain_b));
    splice(result, end.end_face_id, &corners[0], chain_a)?;
    splice(result, plan.second_face, &corners[1], chain_b)?;
    if let Some(step) = commit.step_edge {
        let corner_point = result.vertices.iter().find(|v| v.id == corner).ok_or_else(|| refuse("the corner vertex is missing"))?.point;
        let (u, v) = (plan.q1.sub(corner_point), plan.q2.sub(corner_point));
        let eu = u.normalized().or_refuse(KernelStage::Sew, "normalized")?;
        let normal = u.cross(v).normalized().or_refuse(KernelStage::Sew, "normalized")?;
        let ev = normal.cross(eu);
        let reach = 4.0 * u.length().max(v.length());
        let origin = corner_point.sub(eu.scale(reach)).sub(ev.scale(reach));
        let surface = crate::make_plane(origin, eu, ev, 2.0 * reach, 2.0 * reach).or_refuse(KernelStage::Sew, "make_plane")?;
        let chart = plane_chart(&FaceRecord { id: 0, surface: surface.clone(), same_sense: true, loops: Vec::new(), name: None }).ok_or_else(|| refuse("the step has no chart"))?;
        let line = |a: Vec3, b: Vec3| crate::make_line(chart(a), chart(b)).or_refuse(KernelStage::Sew, "step_pcurve");
        // Opposite to the wall and to both floors.
        let walk = if forward {
            [(step, false, plan.q2, plan.q1), (commit.first_divider, false, plan.q1, corner_point), (commit.second_divider, true, corner_point, plan.q2)]
        } else {
            [(step, true, plan.q1, plan.q2), (commit.second_divider, false, plan.q2, corner_point), (commit.first_divider, true, corner_point, plan.q1)]
        };
        let mut coedges = Vec::with_capacity(3);
        let mut area = 0.0;
        for (edge_id, edge_forward, from, to) in walk {
            let (a, b) = (chart(from), chart(to));
            area += a.x * b.y - b.x * a.y;
            coedges.push(CoedgeRecord { id: take_id(), edge_id, forward: edge_forward, pcurve: line(from, to)? });
        }
        let face = FaceRecord {
            id: take_id(),
            surface,
            same_sense: area > 0.0,
            loops: vec![LoopRecord { id: 1, coedges }],
            name: name.map(|name| format!("{name}:STEP")),
        };
        let shell = result.shells.iter_mut().find(|shell| shell.faces.iter().any(|f| f.id == plan.second_face)).ok_or_else(|| refuse("the floor's shell is lost"))?;
        shell.faces.push(face);
    }
    for (cone, face) in plan.cones.into_iter().zip([end.end_face_id, plan.second_face]) {
        if cone && !crate::face_merge::recarry_cone_apex_face(result, face, 1e-7)? {
            return Err(refuse("a cone floor could not take the extended trim"));
        }
    }
    Ok(())
}
