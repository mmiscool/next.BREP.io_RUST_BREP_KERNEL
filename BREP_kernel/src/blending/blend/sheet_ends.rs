//! Blending an edge of a SHEET up to the sheet's free boundary (refusal
//! census 2026-11-02 row 2: three faces of a cube corner as an open sheet,
//! the three corner edges filleted).
//!
//! Every lane that ends a stripe on its own closes it on the single face
//! across the corner (`edge/support.rs` `end_face_id`, the §6.9 transverse
//! construction). A sheet's edge that runs out at the free boundary has no
//! such face: both mates' boundary edges at its end vertex are FREE (used by
//! one face), and the network refuses "no single end face across the
//! corner". The rolling ball itself is well posed there; the blend simply
//! ends where its rails cross the free boundary, with its end section free.
//!
//! So the end is closed by a TEMPORARY planar face through the two free
//! boundary edges, the ordinary lanes blend the body, and the temporary
//! faces are removed again: the transverse edge they carried, and the
//! trimmed pieces of the boundary edges, are free edges of the sheet that
//! comes out.
//!
//! The class is narrow by construction. The body is a Sheet; at every
//! closed end, the end vertex lies on no other selected edge, the two mates'
//! boundary edges there are distinct, free, straight and not collinear, and
//! no boundary edge is asked for by two ends. Anything else, or a result
//! that does not validate as a sheet with every temporary face gone, keeps
//! the ordinary path's refusal.

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord};
use crate::{KernelRefusal, Vec3};

/// Whether a sheet's free ends are closed by temporary faces: ON unless
/// `BREP_SHEET_FREE_END=0` (or `off`), or a test's forced value.
pub(crate) fn sheet_free_end_on() -> bool {
    switch!("BREP_SHEET_FREE_END", true)
}


/// The name every temporary end face carries, followed by its index.
const TEMPORARY_FACE: &str = "__sheet_free_end_";

/// How many coedges of the body use `edge_id`.
fn uses_of(solid: &BrepSolid, edge_id: u64) -> usize {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .filter(|coedge| coedge.edge_id == edge_id)
        .count()
}

/// The coedges of `edge_id`, with their faces.
fn coedges_of(solid: &BrepSolid, edge_id: u64) -> Vec<(&FaceRecord, &CoedgeRecord)> {
    let mut found = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if coedge.edge_id == edge_id {
                found.push((face, coedge));
            }
        }
    }
    found
}

fn vertex_point(solid: &BrepSolid, id: u64) -> Option<Vec3> {
    solid.vertices.iter().find(|vertex| vertex.id == id).map(|vertex| vertex.point)
}

/// One end to close: the end vertex and the two free boundary edges there.
struct FreeEnd {
    vertex: u64,
    boundaries: [u64; 2],
}

/// The free end of selected edge `edge_id` at `vertex`, if the class admits it.
fn free_end_at(solid: &BrepSolid, edge_id: u64, vertex: u64, selected: &[u64]) -> Option<FreeEnd> {
    let mates = coedges_of(solid, edge_id);
    if mates.len() != 2 || mates[0].0.id == mates[1].0.id {
        return None;
    }
    // The end vertex is not a corner of the selection.
    let on_other = selected.iter().filter(|id| **id != edge_id).any(|id| {
        solid.edges.iter().find(|edge| edge.id == *id).is_some_and(|edge| edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
    });
    if on_other {
        return None;
    }
    let mut boundaries = [0u64; 2];
    for (slot, (face, _)) in mates.iter().enumerate() {
        let mut at_vertex = face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .filter(|coedge| coedge.edge_id != edge_id)
            .filter_map(|coedge| solid.edges.iter().find(|edge| edge.id == coedge.edge_id))
            .filter(|edge| !edge.degenerate && (edge.start_vertex_id == vertex || edge.end_vertex_id == vertex));
        let boundary = at_vertex.next()?;
        if at_vertex.next().is_some() || uses_of(solid, boundary.id) != 1 || boundary.start_vertex_id == boundary.end_vertex_id {
            return None;
        }
        boundary.curve.straight_segment(1e-9)?;
        boundaries[slot] = boundary.id;
    }
    if boundaries[0] == boundaries[1] {
        return None;
    }
    Some(FreeEnd { vertex, boundaries })
}

/// The far vertex of a boundary edge from `vertex`.
fn far_vertex(edge: &EdgeRecord, vertex: u64) -> u64 {
    if edge.start_vertex_id == vertex {
        edge.end_vertex_id
    } else {
        edge.start_vertex_id
    }
}

/// The body with every admitted free end of the selection closed by a
/// temporary planar face, or `None` when none is admitted or one is not.
pub(crate) fn close_free_ends(solid: &BrepSolid, edge_ids: &[u64]) -> Option<BrepSolid> {
    if solid.kind != crate::BodyKind::Sheet {
        return None;
    }
    let mut ends: Vec<FreeEnd> = Vec::new();
    for edge_id in edge_ids {
        let edge = solid.edges.iter().find(|edge| edge.id == *edge_id)?;
        if edge.start_vertex_id == edge.end_vertex_id {
            continue;
        }
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            if let Some(end) = free_end_at(solid, *edge_id, vertex, edge_ids) {
                ends.push(end);
            }
        }
    }
    if ends.is_empty() {
        return None;
    }
    let mut claimed: Vec<u64> = ends.iter().flat_map(|end| end.boundaries).collect();
    claimed.sort_unstable();
    if claimed.windows(2).any(|pair| pair[0] == pair[1]) {
        return None;
    }
    let mut next_id = max_id(solid) + 1;
    let mut take_id = || {
        let id = next_id;
        next_id += 1;
        id
    };
    let mut body = solid.clone();
    for (index, end) in ends.iter().enumerate() {
        let first = solid.edges.iter().find(|edge| edge.id == end.boundaries[0])?;
        let second = solid.edges.iter().find(|edge| edge.id == end.boundaries[1])?;
        let first_far = far_vertex(first, end.vertex);
        let second_far = far_vertex(second, end.vertex);
        if first_far == second_far {
            return None;
        }
        let corner = vertex_point(solid, end.vertex)?;
        let a = vertex_point(solid, first_far)?;
        let b = vertex_point(solid, second_far)?;
        // Each boundary is used once by its mate; the temporary face uses it
        // the other way. The loop then runs through the end vertex from one
        // boundary to the other and back along the closing chord.
        let (_, first_use) = coedges_of(solid, first.id).into_iter().next()?;
        let (_, second_use) = coedges_of(solid, second.id).into_iter().next()?;
        // Reversing the mate's use of `first`: does that reversed use ARRIVE
        // at the end vertex? Then the loop goes first -> end vertex -> second.
        let first_reverse_forward = !first_use.forward;
        let first_reverse_arrives = (if first_reverse_forward { first.end_vertex_id } else { first.start_vertex_id }) == end.vertex;
        let second_reverse_forward = !second_use.forward;
        let second_reverse_leaves = (if second_reverse_forward { second.start_vertex_id } else { second.end_vertex_id }) == end.vertex;
        // A consistent loop needs one boundary arriving at the vertex and the
        // other leaving it in the reversed uses.
        let (lead, lead_forward, trail, trail_forward, from, to) = if first_reverse_arrives && second_reverse_leaves {
            (first, first_reverse_forward, second, second_reverse_forward, b, a)
        } else if !first_reverse_arrives && !second_reverse_leaves {
            (second, second_reverse_forward, first, first_reverse_forward, a, b)
        } else {
            return None;
        };
        // Loop: lead (arrives at the vertex), trail (leaves it), chord back.
        let chord = crate::make_line(from, to).ok()?;
        let polygon = [lead_start(lead, lead_forward, solid)?, corner, from];
        let normal = newell(&polygon);
        let length = normal.length();
        // Collinear boundaries span no plane.
        if !(length > 1e-9 * a.sub(corner).length() * b.sub(corner).length()) {
            return None;
        }
        let normal = normal.scale(1.0 / length);
        let x_axis = a.sub(corner).normalized().ok()?;
        let y_axis = normal.cross(x_axis).normalized().ok()?;
        let points = [corner, a, b];
        let (mut min_x, mut max_x, mut min_y, mut max_y) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for point in points {
            let delta = point.sub(corner);
            min_x = min_x.min(delta.dot(x_axis));
            max_x = max_x.max(delta.dot(x_axis));
            min_y = min_y.min(delta.dot(y_axis));
            max_y = max_y.max(delta.dot(y_axis));
        }
        let padding = (max_x - min_x).max(max_y - min_y) * 0.05 + 1e-6;
        let origin = corner.add(x_axis.scale(min_x - padding)).add(y_axis.scale(min_y - padding));
        let surface = crate::make_plane(origin, x_axis, y_axis, max_x - min_x + 2.0 * padding, max_y - min_y + 2.0 * padding).ok()?;
        let plane_pcurve = |curve: &crate::NurbsCurve, forward: bool| -> Option<crate::NurbsCurve> {
            let pcurve = crate::curve::curve_to_plane_parameters(curve, origin, x_axis, y_axis).ok()?;
            if forward {
                Some(pcurve)
            } else {
                pcurve.reversed().ok()
            }
        };
        let chord_domain = chord.domain().ok()?;
        let chord_id = take_id();
        let (from_vertex, to_vertex) = if std::ptr::eq(lead, first) { (second_far, first_far) } else { (first_far, second_far) };
        let coedges = vec![
            CoedgeRecord { id: take_id(), edge_id: lead.id, forward: lead_forward, pcurve: plane_pcurve(&lead.curve, lead_forward)? },
            CoedgeRecord { id: take_id(), edge_id: trail.id, forward: trail_forward, pcurve: plane_pcurve(&trail.curve, trail_forward)? },
            CoedgeRecord { id: take_id(), edge_id: chord_id, forward: true, pcurve: plane_pcurve(&chord, true)? },
        ];
        body.edges.push(EdgeRecord {
            id: chord_id,
            curve: chord,
            t0: chord_domain[0],
            t1: chord_domain[1],
            start_vertex_id: from_vertex,
            end_vertex_id: to_vertex,
            degenerate: false,
            name: None,
        });
        let loop_id = take_id();
        let face_id = take_id();
        body.shells.first_mut()?.faces.push(FaceRecord {
            id: face_id,
            surface,
            same_sense: true,
            loops: vec![LoopRecord { id: loop_id, coedges }],
            name: Some(format!("{TEMPORARY_FACE}{index}")),
        });
    }
    body.id = take_id();
    Some(body)
}

/// Where the lead boundary's use starts (its far vertex).
fn lead_start(edge: &EdgeRecord, forward: bool, solid: &BrepSolid) -> Option<Vec3> {
    vertex_point(solid, if forward { edge.start_vertex_id } else { edge.end_vertex_id })
}

/// Newell's normal of a closed polygon, in its order.
fn newell(points: &[Vec3]) -> Vec3 {
    let mut normal = Vec3::new(0.0, 0.0, 0.0);
    for (index, current) in points.iter().enumerate() {
        let next = points[(index + 1) % points.len()];
        normal = normal.add(Vec3::new(
            (current.y - next.y) * (current.z + next.z),
            (current.z - next.z) * (current.x + next.x),
            (current.x - next.x) * (current.y + next.y),
        ));
    }
    normal
}

/// The largest id any record of the body carries.
fn max_id(solid: &BrepSolid) -> u64 {
    let mut largest = solid.id;
    for vertex in &solid.vertices {
        largest = largest.max(vertex.id);
    }
    for edge in &solid.edges {
        largest = largest.max(edge.id);
    }
    for shell in &solid.shells {
        largest = largest.max(shell.id);
        for face in &shell.faces {
            largest = largest.max(face.id);
            for loop_record in &face.loops {
                largest = largest.max(loop_record.id);
                for coedge in &loop_record.coedges {
                    largest = largest.max(coedge.id);
                }
            }
        }
    }
    largest
}

/// The blended body with every temporary end face removed, and the edges
/// and vertices only those faces used with them; `None` when a temporary
/// face is missing (a lane merged or split it) or the sheet does not
/// validate.
pub(crate) fn open_free_ends(mut body: BrepSolid, count: usize) -> Option<BrepSolid> {
    let mut removed = 0usize;
    for shell in &mut body.shells {
        shell.faces.retain(|face| {
            let temporary = face.name.as_deref().is_some_and(|name| name.starts_with(TEMPORARY_FACE));
            removed += temporary as usize;
            !temporary
        });
    }
    if removed != count {
        return None;
    }
    let used: rustc_hash::FxHashSet<u64> = body
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    body.edges.retain(|edge| used.contains(&edge.id));
    let vertices: rustc_hash::FxHashSet<u64> = body.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
    body.vertices.retain(|vertex| vertices.contains(&vertex.id));
    body.kind = crate::BodyKind::Sheet;
    body.mass_properties_cache = Default::default();
    body.validate().is_empty().then_some(body)
}

/// How many temporary faces `close_free_ends` added.
pub(crate) fn temporary_faces(body: &BrepSolid) -> usize {
    body.shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.name.as_deref().is_some_and(|name| name.starts_with(TEMPORARY_FACE)))
        .count()
}
