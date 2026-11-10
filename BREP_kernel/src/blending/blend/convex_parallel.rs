//! The exact parallel body of a convex polyhedron, built face by face.
//!
//! A rounded offset normally moves the faces out to a mitred body and rolls a
//! ball along its edges. Where four or more faces meet at a vertex, the moved
//! planes no longer meet in a point: the mitred body has a short extra edge
//! there, both ends of which seat the same ball, and no strip can be rolled
//! along it. The parallel body itself has no such edge. It is each face moved
//! out by `r`, a cylinder along each edge and a sphere patch at each vertex,
//! whatever the vertices' degrees, so it is assembled directly.

use super::corner::build_corner_patch_exact;
use super::network::{build_line_pcurve, corner_section_arc};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3, Vec4};
use rustc_hash::FxHashMap as HashMap;

/// `BREP_OFFSET_ROUND_CONVEX` (default on; `=0` restores the refusal).
fn round_convex_on() -> bool {
    switch!("BREP_OFFSET_ROUND_CONVEX", true)
}

/// One source edge: its ends, and the faces that walk it forward and backward.
struct Ridge {
    ends: [u64; 2],
    forward: usize,
    backward: usize,
}

/// A closed convex polyhedron's parallel body at `radius`, and for each face
/// of it the source face it moved from (`None` for a cylinder or sphere).
/// `None` when `source` is not a single closed shell of planar, one-loop
/// faces meeting at strictly convex edges.
pub(crate) fn convex_polyhedron_parallel(
    source: &BrepSolid,
    radius: f64,
) -> Result<Option<(BrepSolid, Vec<Option<u64>>)>, KernelRefusal> {
    if !round_convex_on() || radius <= 0.0 {
        return Ok(None);
    }
    let [shell] = source.shells.as_slice() else { return Ok(None) };
    let point_of: HashMap<u64, Vec3> = source.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let edge_of: HashMap<u64, &EdgeRecord> = source.edges.iter().map(|edge| (edge.id, edge)).collect();

    // Outward unit normals; a face's walk must run anticlockwise seen from outside.
    let mut normals = Vec::with_capacity(shell.faces.len());
    for face in &shell.faces {
        let [walk] = face.loops.as_slice() else { return Ok(None) };
        let Some(crate::AnalyticSurface::Plane { u_dir, v_dir, .. }) = face.surface.analytic() else { return Ok(None) };
        let carrier = u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Sew, "normalized")?;
        let normal = if face.same_sense { carrier } else { carrier.scale(-1.0) };
        let mut area = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
        let mut corners = Vec::with_capacity(walk.coedges.len());
        for coedge in &walk.coedges {
            let Some(edge) = edge_of.get(&coedge.edge_id) else { return Ok(None) };
            corners.push(point_of[&if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id }]);
        }
        for (index, corner) in corners.iter().enumerate() {
            area = area.add(corner.cross(corners[(index + 1) % corners.len()]));
        }
        if area.dot(normal) <= 0.0 {
            return Ok(None);
        }
        normals.push(normal);
    }

    // Each edge used exactly once in each direction, every crease strictly convex.
    let mut ridges: HashMap<u64, (Option<usize>, Option<usize>)> = HashMap::default();
    let mut ridge_order = Vec::new();
    for (index, face) in shell.faces.iter().enumerate() {
        for coedge in &face.loops[0].coedges {
            let slot = ridges.entry(coedge.edge_id).or_insert_with(|| {
                ridge_order.push(coedge.edge_id);
                (None, None)
            });
            let side = if coedge.forward { &mut slot.0 } else { &mut slot.1 };
            if side.replace(index).is_some() {
                return Ok(None);
            }
        }
    }
    let mut ridge_of: HashMap<u64, Ridge> = HashMap::default();
    for id in &ridge_order {
        let (Some(forward), Some(backward)) = ridges[id] else { return Ok(None) };
        let edge = edge_of[id];
        let ends = [edge.start_vertex_id, edge.end_vertex_id];
        let along = point_of[&ends[1]].sub(point_of[&ends[0]]).normalized().or_refuse(KernelStage::Sew, "normalized")?;
        if along.cross(normals[forward]).dot(normals[backward]) <= 1e-9 {
            return Ok(None);
        }
        ridge_of.insert(*id, Ridge { ends, forward, backward });
    }
    // A closed surface whose creases are all convex is a convex body only on the sphere.
    if source.vertices.len() as i64 - ridge_order.len() as i64 + shell.faces.len() as i64 != 2 {
        return Ok(None);
    }

    let mut next = 1_u64;
    let mut take_id = move || {
        next += 1;
        next
    };
    let mut result = BrepSolid { mass_properties_cache: Default::default(), vertices: Vec::new(), edges: Vec::new(), shells: Vec::new(), ..source.clone() };

    // One vertex for each corner of each moved face.
    let mut seat: HashMap<(usize, u64), u64> = HashMap::default();
    for (index, face) in shell.faces.iter().enumerate() {
        for coedge in &face.loops[0].coedges {
            let edge = edge_of[&coedge.edge_id];
            let vertex = if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id };
            let id = take_id();
            seat.insert((index, vertex), id);
            result.vertices.push(VertexRecord { id, point: point_of[&vertex].add(normals[index].scale(radius)) });
        }
    }

    // Rails: each source edge, moved with each of its two faces, directed as the edge is.
    let mut rail: HashMap<(usize, u64), EdgeRecord> = HashMap::default();
    for id in &ridge_order {
        let ridge = &ridge_of[id];
        for side in [ridge.forward, ridge.backward] {
            let shift = normals[side].scale(radius);
            let curve = crate::make_line(point_of[&ridge.ends[0]].add(shift), point_of[&ridge.ends[1]].add(shift)).or_refuse(KernelStage::Sew, "make_line")?;
            let record = EdgeRecord { id: take_id(), curve, t0: 0.0, t1: 1.0, start_vertex_id: seat[&(side, ridge.ends[0])], end_vertex_id: seat[&(side, ridge.ends[1])], degenerate: false, name: None };
            result.edges.push(record.clone());
            rail.insert((side, *id), record);
        }
    }

    // Arcs: at each end of each edge. The start's runs from the forward face to
    // the backward one and the end's returns, so a cylinder walks both forward
    // and, seen round a vertex, the arcs chain face to face.
    let mut arc: HashMap<(u64, u64), EdgeRecord> = HashMap::default();
    for id in &ridge_order {
        let ridge = &ridge_of[id];
        for (end, faces) in [(ridge.ends[0], [ridge.forward, ridge.backward]), (ridge.ends[1], [ridge.backward, ridge.forward])] {
            let center = point_of[&end];
            let contact = |face: usize| center.add(normals[face].scale(radius));
            let curve = corner_section_arc(center, radius, contact(faces[0]), contact(faces[1]))?;
            let domain = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
            let record = EdgeRecord { id: take_id(), curve, t0: domain[0], t1: domain[1], start_vertex_id: seat[&(faces[0], end)], end_vertex_id: seat[&(faces[1], end)], degenerate: false, name: None };
            result.edges.push(record.clone());
            arc.insert((*id, end), record);
        }
    }

    let mut faces = Vec::new();
    let mut images = Vec::new();

    // The moved planes, same chart, new rails.
    for (index, face) in shell.faces.iter().enumerate() {
        let shift = normals[index].scale(radius);
        let moved: Vec<Vec<Vec4>> = face.surface.control_points.iter()
            .map(|row| row.iter().map(|p| Vec4 { x: p.x + p.w * shift.x, y: p.y + p.w * shift.y, z: p.z + p.w * shift.z, w: p.w }).collect())
            .collect();
        let surface = crate::NurbsSurface::new(face.surface.degree_u, face.surface.degree_v, face.surface.knots_u.clone(), face.surface.knots_v.clone(), moved)
            .or_refuse(KernelStage::Sew, "moved_plane")?;
        let mut coedges = Vec::with_capacity(face.loops[0].coedges.len());
        for coedge in &face.loops[0].coedges {
            let edge = &rail[&(index, coedge.edge_id)];
            let at = |t: f64| edge.curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate");
            let (start, finish) = if coedge.forward { (at(0.0)?, at(1.0)?) } else { (at(1.0)?, at(0.0)?) };
            coedges.push(CoedgeRecord { id: take_id(), edge_id: edge.id, forward: coedge.forward, pcurve: build_line_pcurve(&surface, start, finish)? });
        }
        faces.push(FaceRecord { id: take_id(), surface, same_sense: face.same_sense, loops: vec![LoopRecord { id: take_id(), coedges }], name: face.name.clone() });
        images.push(Some(face.id));
    }

    // A cylinder along each edge: the start's arc carried to the end.
    for id in &ridge_order {
        let ridge = &ridge_of[id];
        let start_arc = &arc[&(*id, ridge.ends[0])];
        let end_arc = &arc[&(*id, ridge.ends[1])];
        let shift = point_of[&ridge.ends[1]].sub(point_of[&ridge.ends[0]]);
        let carried: Vec<Vec<Vec4>> = start_arc.curve.control_points.iter()
            .map(|p| vec![*p, Vec4 { x: p.x + p.w * shift.x, y: p.y + p.w * shift.y, z: p.z + p.w * shift.z, w: p.w }])
            .collect();
        let surface = crate::NurbsSurface::new(start_arc.curve.degree, 1, start_arc.curve.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], carried)
            .or_refuse(KernelStage::Sew, "edge_cylinder")?;
        let (u0, u1) = (start_arc.t0, start_arc.t1);
        let line = |a: f64, b: f64, c: f64, d: f64| -> Result<NurbsCurve, KernelRefusal> {
            crate::sweep_topology::parameter_line(a, b, c, d).or_refuse(KernelStage::Sew, "parameter_line")
        };
        let coedges = vec![
            CoedgeRecord { id: take_id(), edge_id: start_arc.id, forward: true, pcurve: line(u0, 0.0, u1, 0.0)? },
            CoedgeRecord { id: take_id(), edge_id: rail[&(ridge.backward, *id)].id, forward: true, pcurve: line(u1, 0.0, u1, 1.0)? },
            CoedgeRecord { id: take_id(), edge_id: end_arc.id, forward: true, pcurve: line(u1, 1.0, u0, 1.0)? },
            CoedgeRecord { id: take_id(), edge_id: rail[&(ridge.forward, *id)].id, forward: false, pcurve: line(u0, 1.0, u0, 0.0)? },
        ];
        faces.push(FaceRecord { id: take_id(), surface, same_sense: true, loops: vec![LoopRecord { id: take_id(), coedges }], name: None });
        images.push(None);
    }

    // A sphere patch at each vertex: round the faces there, the arcs between them.
    for vertex in &source.vertices {
        let leaving = |face: usize| -> Option<u64> {
            shell.faces[face].loops[0].coedges.iter().find_map(|coedge| {
                let edge = edge_of[&coedge.edge_id];
                let start = if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id };
                (start == vertex.id).then_some(coedge.edge_id)
            })
        };
        let Some(first) = (0..shell.faces.len()).find(|&face| leaving(face).is_some()) else { return Ok(None) };
        let (mut face, mut arcs, mut around) = (first, Vec::new(), Vec::new());
        loop {
            let Some(id) = leaving(face) else { return Ok(None) };
            let ridge = &ridge_of[&id];
            around.push(normals[face]);
            arcs.push(arc[&(id, vertex.id)].clone());
            face = if ridge.forward == face { ridge.backward } else { ridge.forward };
            if face == first {
                break;
            }
            if arcs.len() > shell.faces.len() {
                return Ok(None);
            }
        }
        let (patch, pole) = build_corner_patch_exact(vertex.point, radius, &around, &arcs, true, None, &mut take_id)?;
        if let Some(edge) = pole {
            result.edges.push(edge);
        }
        faces.push(patch);
        images.push(None);
    }

    result.shells.push(ShellRecord { id: shell.id, faces });
    Ok(Some((result, images)))
}
