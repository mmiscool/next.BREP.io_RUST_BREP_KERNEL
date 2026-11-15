//! Deleting a RING of faces between one planar cap and a wall of many faces
//! that all rise straight towards it: a chamfer run round a filleted block's
//! top, with its straight strips and the corner patches between them.
//!
//! The band lanes need two wall carriers. Here the wall is a plane, a cylinder,
//! a plane, and so on round the part, so there are as many carriers as faces.
//! The answer is simple all the same: every wall is a prism along the cap's
//! normal, so it runs on unchanged up to the cap. Each rim edge rises to the
//! cap's plane as the same curve, the edges between walls grow to meet it, and
//! the risen rim becomes the cap's boundary.
//!
//! Picking only the straight strips asks for the same body. A corner patch
//! between two strips that are both leaving has nothing left to blend between,
//! and cannot stay: past its ends is no surface that closes the gap to the
//! corner. So it leaves with them, as a fillet's end caps do
//! (`with_owned_caps`). [`delete_faces_and_heal`] tries this lane only after
//! every other lane has refused, so no selection that healed before changes.

use super::*;
use super::delete_faces::euler_characteristic;
use super::face_move::ruled_revolution_carrier;
use crate::offset_retrim::plane_of_surface;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Sine of the angle a fitted corner patch may stand off its strip.
const PATCH_TANGENCY: f64 = 1e-2;

pub(super) struct PrismRing {
    shell_index: usize,
    faces: HashSet<u64>,
    edges: HashSet<u64>,
    cap: u64,
    cap_loop: usize,
    /// Outward normal of the cap.
    normal: Vec3,
    /// `normal · x` on the cap.
    level: f64,
    walls: Vec<u64>,
}

/// The face's normal at the midpoint of its use of an edge.
fn normal_along(face: &FaceRecord, coedge: &CoedgeRecord) -> Option<Vec3> {
    let [q0, q1] = coedge.pcurve.domain().ok()?;
    let uv = coedge.pcurve.evaluate(0.5 * (q0 + q1)).ok()?;
    face.surface.normal(uv.x, uv.y).ok()
}

/// The selection with the curved patches that continue it: a curved face
/// joined TANGENTIALLY to two or more selected faces, and to selected faces
/// only tangentially. A strip the user kept meets its neighbours at a mitre, so
/// it is not one of these and stays.
fn with_enclosed_patches(solid: &BrepSolid, face_ids: &[u64], tolerance: f64) -> HashSet<u64> {
    let selected: HashSet<u64> = face_ids.iter().copied().collect();
    let mut users: HashMap<u64, Vec<(&FaceRecord, &CoedgeRecord)>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            users.entry(coedge.edge_id).or_default().push((face, coedge));
        }
    }
    let mut all = selected.clone();
    'faces: for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        if selected.contains(&face.id) || plane_of_surface(&face.surface, tolerance, "delete_faces_and_heal").is_ok() {
            continue;
        }
        let mut beside: HashSet<u64> = HashSet::default();
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            for (other, other_use) in users.get(&coedge.edge_id).into_iter().flatten() {
                if !selected.contains(&other.id) {
                    continue;
                }
                let (Some(own), Some(across)) = (normal_along(face, coedge), normal_along(other, other_use)) else {
                    continue 'faces;
                };
                if own.cross(across).length() > PATCH_TANGENCY {
                    continue 'faces;
                }
                beside.insert(other.id);
            }
        }
        if beside.len() >= 2 {
            all.insert(face.id);
        }
    }
    all
}

/// Does this selection, with its enclosed patches, read as a ring between one
/// planar cap and prismatic walls? `None` leaves the original refusal standing.
pub(super) fn classify_prism_ring(solid: &BrepSolid, face_ids: &[u64]) -> Option<PrismRing> {
    let tolerance = (solid_model_scale(solid) * 1e-6).max(1e-9);
    let faces = with_enclosed_patches(solid, face_ids, tolerance);
    if faces.len() < 2 {
        return None;
    }
    let first = *faces.iter().next()?;
    let shell_index = find_face(solid, first)?.0;

    let mut uses: HashMap<u64, (usize, usize)> = HashMap::default();
    let mut keeper: HashMap<u64, u64> = HashMap::default();
    let mut owners: HashMap<u64, Vec<u64>> = HashMap::default();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let inside = faces.contains(&face.id);
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let entry = uses.entry(coedge.edge_id).or_default();
            if inside {
                entry.0 += 1;
                owners.entry(coedge.edge_id).or_default().push(face.id);
            } else {
                entry.1 += 1;
                keeper.insert(coedge.edge_id, face.id);
            }
        }
    }
    let edge_records: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge)).collect();
    let edges: HashSet<u64> =
        uses.iter().filter(|(_, (inside, _))| *inside > 0).map(|(edge_id, _)| *edge_id).collect();
    for edge_id in &edges {
        let (inside, kept) = uses[edge_id];
        if edge_records.get(edge_id)?.degenerate || inside + kept != 2 {
            return None;
        }
    }

    // One piece.
    let mut reached: HashSet<u64> = HashSet::default();
    let mut stack = vec![first];
    reached.insert(first);
    while let Some(face_id) = stack.pop() {
        let (shell, position) = find_face(solid, face_id)?;
        if shell != shell_index {
            return None;
        }
        for coedge in solid.shells[shell].faces[position].loops.iter().flat_map(|l| &l.coedges) {
            for other in owners.get(&coedge.edge_id).into_iter().flatten() {
                if reached.insert(*other) {
                    stack.push(*other);
                }
            }
        }
    }
    if reached.len() != faces.len() {
        return None;
    }

    let mut survivors: Vec<u64> =
        edges.iter().filter(|edge_id| uses[*edge_id].1 == 1).map(|edge_id| keeper[edge_id]).collect();
    survivors.sort_unstable();
    survivors.dedup();

    // The cap: planar, and one whole loop of it borders the ring.
    let mut cap: Option<(u64, usize, Vec3, f64)> = None;
    for &survivor in &survivors {
        let (shell, position) = find_face(solid, survivor)?;
        let face = &solid.shells[shell].faces[position];
        let Ok(plane) = plane_of_surface(&face.surface, tolerance, "delete_faces_and_heal") else {
            continue;
        };
        for (loop_index, loop_record) in face.loops.iter().enumerate() {
            if loop_record.coedges.iter().all(|coedge| edges.contains(&coedge.edge_id)) {
                if cap.is_some() {
                    return None;
                }
                let normal = if face.same_sense { plane.normal } else { plane.normal.scale(-1.0) };
                cap = Some((survivor, loop_index, normal, normal.dot(plane.origin)));
            }
        }
    }
    let (cap, cap_loop, normal, level) = cap?;
    let walls: Vec<u64> = survivors.into_iter().filter(|id| *id != cap).collect();
    if walls.is_empty() {
        return None;
    }

    // Every wall is a prism along the cap's normal, and meets the ring below it.
    for &wall in &walls {
        let (shell, position) = find_face(solid, wall)?;
        let face = &solid.shells[shell].faces[position];
        if let Ok(plane) = plane_of_surface(&face.surface, tolerance, "delete_faces_and_heal") {
            if plane.normal.dot(normal).abs() > 1e-7 {
                return None;
            }
        } else {
            let (frame, rho0, rho1, _) = ruled_revolution_carrier(&face.surface)?;
            if frame.axis.cross(normal).length() > 1e-7 || (rho0 - rho1).abs() > tolerance {
                return None;
            }
        }
        for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
            if !edges.contains(&coedge.edge_id) {
                continue;
            }
            let edge = edge_records.get(&coedge.edge_id)?;
            let at_start = normal.dot(edge.curve.evaluate(edge.t0).ok()?);
            if at_start >= level - tolerance {
                return None;
            }
            for step in 1..=8 {
                let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0;
                if (normal.dot(edge.curve.evaluate(t).ok()?) - at_start).abs() > tolerance {
                    return None;
                }
            }
        }
    }
    Some(PrismRing { shell_index, faces, edges, cap, cap_loop, normal, level, walls })
}

/// Heal a [`PrismRing`]: see the module docs.
pub(super) fn heal_prism_ring(solid: &BrepSolid, ring: &PrismRing, op: &str) -> Result<BrepSolid, KernelRefusal> {
    let original = solid;
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-6).max(1e-9);
    let pcurve_tolerance = (scale * 1e-7).max(1e-9);
    let refusal = |slug: &'static str, reason: String| {
        KernelRefusal::unsupported(KernelStage::Sew, slug, format!("{op}: the prism ring {reason}"))
    };
    let missing = |what: &str, id: u64| {
        KernelRefusal::internal(KernelStage::Collect, "topology", format!("{op}: missing {what} {id}"))
    };

    // --- The rim, as each wall uses it ----------------------------------------
    let mut rim: Vec<(u64, bool)> = Vec::new();
    for &wall in &ring.walls {
        let (shell, position) = find_face(&solid, wall).ok_or_else(|| missing("face", wall))?;
        for coedge in solid.shells[shell].faces[position].loops.iter().flat_map(|l| &l.coedges) {
            if ring.edges.contains(&coedge.edge_id) {
                rim.push((coedge.edge_id, coedge.forward));
            }
        }
    }
    let rim_ids: HashSet<u64> = rim.iter().map(|(edge_id, _)| *edge_id).collect();

    // --- Rise: rim vertices, then rim curves -----------------------------------
    let edge_map: HashMap<u64, EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    let mut rise: HashMap<u64, f64> = HashMap::default();
    for edge_id in &rim_ids {
        let edge = edge_map.get(edge_id).ok_or_else(|| missing("edge", *edge_id))?;
        for vertex_id in [edge.start_vertex_id, edge.end_vertex_id] {
            let point = solid
                .vertices
                .iter()
                .find(|vertex| vertex.id == vertex_id)
                .ok_or_else(|| missing("vertex", vertex_id))?
                .point;
            rise.insert(vertex_id, ring.level - ring.normal.dot(point));
        }
    }
    for vertex in &mut solid.vertices {
        if let Some(height) = rise.get(&vertex.id) {
            vertex.point = vertex.point.add(ring.normal.scale(*height));
        }
    }
    let raised: HashMap<u64, Vec3> = solid
        .vertices
        .iter()
        .filter(|vertex| rise.contains_key(&vertex.id))
        .map(|vertex| (vertex.id, vertex.point))
        .collect();
    let mut wall_extent: Vec<Vec3> = raised.values().copied().collect();
    for edge in &mut solid.edges {
        if !rim_ids.contains(&edge.id) {
            continue;
        }
        let height = rise[&edge.start_vertex_id];
        let lift = ring.normal.scale(height);
        for control in &mut edge.curve.control_points {
            control.x += lift.x * control.w;
            control.y += lift.y * control.w;
            control.z += lift.z * control.w;
        }
        for step in 0..=8 {
            let t = edge.t0 + (edge.t1 - edge.t0) * step as f64 / 8.0;
            wall_extent.push(edge.curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?);
        }
    }
    for &wall in &ring.walls {
        extend_ruled_neighbour_over(&mut solid, wall, &wall_extent, tolerance)?;
        extend_revolution_carrier_over(&mut solid, wall, &wall_extent, tolerance)?;
    }

    // --- The edges between walls grow to the cap -------------------------------
    let mut grown: HashSet<u64> = HashSet::default();
    for edge in &mut solid.edges {
        if ring.edges.contains(&edge.id) {
            continue;
        }
        let start = raised.get(&edge.start_vertex_id).copied();
        let end = raised.get(&edge.end_vertex_id).copied();
        if start.is_none() && end.is_none() {
            continue;
        }
        if start.is_some() && end.is_some() {
            return Err(refusal("edge_span", format!("has edge {} reaching it at both ends", edge.id)));
        }
        let Some(_) = edge.curve.straight_segment(tolerance) else {
            return Err(refusal("curved_side_edge", format!("has a curved edge {} reaching it", edge.id)));
        };
        if let Some(target) = start {
            let anchor = edge.curve.evaluate(edge.t1).or_refuse(KernelStage::Sew, "evaluate")?;
            if target.sub(anchor).cross(ring.normal).length() > tolerance {
                return Err(refusal("slanted_side_edge", format!("has edge {} not along the cap's normal", edge.id)));
            }
            edge.curve = make_line(target, anchor).or_refuse(KernelStage::Sew, "make_line")?;
        } else if let Some(target) = end {
            let anchor = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?;
            if target.sub(anchor).cross(ring.normal).length() > tolerance {
                return Err(refusal("slanted_side_edge", format!("has edge {} not along the cap's normal", edge.id)));
            }
            edge.curve = make_line(anchor, target).or_refuse(KernelStage::Sew, "make_line")?;
        }
        [edge.t0, edge.t1] = edge.curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        grown.insert(edge.id);
    }

    // --- The risen rim is the cap's boundary -----------------------------------
    let mut by_start: HashMap<u64, (u64, bool)> = HashMap::default();
    for (edge_id, wall_forward) in &rim {
        let edge = edge_map.get(edge_id).ok_or_else(|| missing("edge", *edge_id))?;
        // The cap uses each rim edge against the wall's sense.
        let forward = !wall_forward;
        let start = if forward { edge.start_vertex_id } else { edge.end_vertex_id };
        if by_start.insert(start, (*edge_id, forward)).is_some() {
            return Err(refusal("rim_branch", format!("branches at vertex {start}")));
        }
    }
    let (first_vertex, _) = rim
        .first()
        .map(|(edge_id, forward)| {
            let edge = &edge_map[edge_id];
            (if !forward { edge.start_vertex_id } else { edge.end_vertex_id }, ())
        })
        .ok_or_else(|| refusal("no_rim", "has no rim".to_string()))?;
    let (cap_shell, cap_position) = find_face(&solid, ring.cap).ok_or_else(|| missing("face", ring.cap))?;
    let placeholder = solid.shells[cap_shell].faces[cap_position].loops[ring.cap_loop].coedges[0].pcurve.clone();
    let mut next_id = solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.id)
        .max()
        .unwrap_or(0);
    let mut coedges = Vec::with_capacity(rim.len());
    let mut at = first_vertex;
    for _ in 0..rim.len() {
        let &(edge_id, forward) =
            by_start.get(&at).ok_or_else(|| refusal("open_rim", format!("does not close at vertex {at}")))?;
        next_id += 1;
        coedges.push(CoedgeRecord { id: next_id, edge_id, forward, pcurve: placeholder.clone() });
        let edge = &edge_map[&edge_id];
        at = if forward { edge.end_vertex_id } else { edge.start_vertex_id };
    }
    if at != first_vertex {
        return Err(refusal("open_rim", format!("does not close at vertex {first_vertex}")));
    }
    solid.shells[cap_shell].faces[cap_position].loops[ring.cap_loop].coedges = coedges;

    // --- Pcurves ----------------------------------------------------------------
    let edges_by_id: HashMap<u64, EdgeRecord> = solid.edges.iter().map(|edge| (edge.id, edge.clone())).collect();
    let touched: HashSet<u64> = rim_ids.iter().chain(grown.iter()).copied().collect();
    for shell in &mut solid.shells {
        for face in &mut shell.faces {
            if ring.faces.contains(&face.id) {
                continue;
            }
            if face.loops.iter().flat_map(|l| &l.coedges).any(|coedge| touched.contains(&coedge.edge_id)) {
                refit_touched_pcurves(face, &edges_by_id, &touched, false, pcurve_tolerance, op)?;
            }
        }
    }

    // --- Drop the ring ----------------------------------------------------------
    solid.shells[ring.shell_index].faces.retain(|face| !ring.faces.contains(&face.id));
    solid.edges.retain(|edge| !ring.edges.contains(&edge.id) || rim_ids.contains(&edge.id));
    let used: HashSet<u64> = solid.edges.iter().flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id]).collect();
    solid.vertices.retain(|vertex| used.contains(&vertex.id));

    let (before, after) = (euler_characteristic(original), euler_characteristic(&solid));
    if before != after {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "euler",
            format!("{op}: closing the prism ring changes the Euler characteristic {before} -> {after}"),
        ));
    }
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validation",
            format!("{op}: prism-ring heal failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}
