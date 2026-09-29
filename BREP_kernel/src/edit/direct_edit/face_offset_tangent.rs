use super::plan::{EdgeEnd, PlanEdge, PlanVertex, PlannedCoedge, PlannedEdge, PlannedFace, RebuildPlan};
use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

// ---------------------------------------------------------------------------
// A push whose MOUTH meets its floor at TANGENT vertices.
//
// The reporter's gear bore (2026-09-14) is inscribed in a shallow hexagonal
// pocket: the six flats stand at the bore radius, so the pocket floor is six
// corner regions that pinch to nothing where the rim touches each flat. At
// every such vertex four edges meet — two rim arcs, one per floor region, and
// the two halves of the flat's bottom edge — and four faces: the pushed wall,
// two floor regions and the flat.
//
// The rim pass solves the new corner correctly (the two rim arcs share the
// floor's plane, so the corner keeps its place along the rebuilt conic), but
// the flat does not move, so the vertex cannot follow the rim: that is the
// corner guard's refusal. The answer is a topology change, and which one
// depends on where the rim went:
//
// * AWAY from the floor (the bore shrinks: `d > 0` on a hole) — the floor
//   extends to follow it. Each tangent vertex stays on its flat and the rim
//   gets a NEW vertex; a straight BRIDGE edge joins the two across the new
//   strip of floor, between the two floor regions, which keep their records
//   and their names.
// * OVER the floor (the bore grows) — when the new mouth clears every corner
//   of every floor region, the floor is gone. What the plane carries instead is
//   ONE face with two loops: the rebuilt rim outside, the flats' bottom edges
//   inside, with its material on the other side. The floor records merge into
//   the lowest-id one, flipped. A mouth that clears some corners and not others
//   (between the tangent radius and a corner) needs new faces between the rim
//   and the flats and is refused, naming the corner.
//
// A tangent vertex whose floor is not planar is refused by name, before the rim
// pass reaches it.
// ---------------------------------------------------------------------------

/// One tangent vertex on the pushed face's mouth.
#[derive(Clone, Debug)]
pub(super) struct TangentVertex {
    pub(super) vertex: u64,
    /// The rim arc on floor face `floor_a`, and that floor's other edge here.
    pub(super) rim_a: u64,
    pub(super) floor_a: u64,
    pub(super) floor_edge_a: u64,
    pub(super) rim_b: u64,
    pub(super) floor_b: u64,
    pub(super) floor_edge_b: u64,
}

fn traversal(edge: &EdgeRecord, forward: bool) -> (u64, u64) {
    if forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    }
}

fn face_record(solid: &BrepSolid, face_id: u64) -> Result<&FaceRecord, KernelRefusal> {
    find_face(solid, face_id)
        .map(|(shell, position)| &solid.shells[shell].faces[position])
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "missing_face",
                format!("offset_ruled_face: missing face {face_id}"),
            )
        })
}

/// The tangent vertices of the pushed face, read off topology, with their
/// floors' planarity checked.
///
/// A vertex qualifies when two consecutive RIM coedges of the pushed face meet
/// there with DIFFERENT neighbours `A` and `B`, and exactly two more edges end
/// there, one bounding `A` and one bounding `B`, neither bounding the pushed
/// face. Anything else is not this shape and is left to the lanes that always
/// decided it. A qualifying vertex whose floors are not both planes is refused
/// by name; one whose floors are planes in different planes is a genuine
/// corner, not a tangency, and is left alone too.
pub(super) fn tangent_vertices(
    solid: &BrepSolid,
    face_id: u64,
    faces_of_edge: &HashMap<u64, Vec<u64>>,
    edge_by_id: &HashMap<u64, &EdgeRecord>,
    plane_tolerance: f64,
) -> Result<Vec<TangentVertex>, KernelRefusal> {
    let pushed = face_record(solid, face_id)?;
    let other = |edge_id: u64| -> Option<u64> {
        let incident = faces_of_edge.get(&edge_id)?;
        if incident.len() != 2 || incident.iter().all(|f| *f == face_id) {
            return None;
        }
        incident.iter().copied().find(|f| *f != face_id)
    };
    let mut found: Vec<TangentVertex> = Vec::new();
    for loop_record in &pushed.loops {
        let count = loop_record.coedges.len();
        for index in 0..count {
            let here = &loop_record.coedges[index];
            let next = &loop_record.coedges[(index + 1) % count];
            let (Some(first), Some(second)) =
                (edge_by_id.get(&here.edge_id), edge_by_id.get(&next.edge_id))
            else {
                continue;
            };
            let (_, vertex) = traversal(first, here.forward);
            if traversal(second, next.forward).0 != vertex || first.id == second.id {
                continue;
            }
            let (Some(floor_first), Some(floor_second)) = (other(first.id), other(second.id)) else {
                continue;
            };
            if floor_first == floor_second || found.iter().any(|t| t.vertex == vertex) {
                continue;
            }
            let star: Vec<&EdgeRecord> = solid
                .edges
                .iter()
                .filter(|edge| edge.start_vertex_id == vertex || edge.end_vertex_id == vertex)
                .collect();
            if star.len() != 4 {
                continue;
            }
            let bounds = |edge: &EdgeRecord, face: u64| {
                faces_of_edge.get(&edge.id).is_some_and(|faces| faces.contains(&face))
            };
            let floor_edge = |floor: u64, not: u64| {
                let candidates: Vec<u64> = star
                    .iter()
                    .filter(|edge| edge.id != first.id && edge.id != second.id)
                    .filter(|edge| bounds(edge, floor) && !bounds(edge, not) && !bounds(edge, face_id))
                    .map(|edge| edge.id)
                    .collect();
                (candidates.len() == 1).then(|| candidates[0])
            };
            let (Some(edge_first), Some(edge_second)) =
                (floor_edge(floor_first, floor_second), floor_edge(floor_second, floor_first))
            else {
                continue;
            };
            if edge_first == edge_second {
                continue;
            }
            let point = solid
                .vertices
                .iter()
                .find(|record| record.id == vertex)
                .map(|record| record.point)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Classify,
                        "missing_vertex",
                        format!("offset_ruled_face: missing vertex {vertex}"),
                    )
                })?;
            let surface_of = |face: u64| face_record(solid, face).map(|record| &record.surface);
            for floor in [floor_first, floor_second] {
                if !matches!(surface_of(floor)?.analytic(), Some(AnalyticSurface::Plane { .. })) {
                    return Err(KernelRefusal::unsupported(
                        KernelStage::Classify,
                        "tangent_floor_not_planar",
                        format!(
                        "offset_ruled_face: the push's mouth meets floor faces {floor_first} and \
                         {floor_second} at tangent vertex {vertex}, and face {floor} is not planar; \
                         a tangent vertex is split only on a PLANAR floor (refusing)"
                    ),
                    ));
                }
            }
            let plane_first = plane_of_surface(surface_of(floor_first)?, plane_tolerance, "offset_ruled_face")?;
            let plane_second = plane_of_surface(surface_of(floor_second)?, plane_tolerance, "offset_ruled_face")?;
            let coplanar = plane_first.normal.cross(plane_second.normal).length() <= 1e-9
                && plane_second.origin.sub(plane_first.origin).dot(plane_first.normal).abs()
                    <= plane_tolerance
                && point.sub(plane_first.origin).dot(plane_first.normal).abs() <= plane_tolerance;
            if !coplanar {
                continue;
            }
            found.push(TangentVertex {
                vertex,
                rim_a: first.id,
                floor_a: floor_first,
                floor_edge_a: edge_first,
                rim_b: second.id,
                floor_b: floor_second,
                floor_edge_b: edge_second,
            });
        }
    }
    Ok(found)
}

/// Where a floor face's interior lies from a point on one of its boundary
/// edges: to the LEFT of the edge's traversal, seen along the face's outward
/// normal. `at_start` names which end of the edge the point is.
fn into_face(
    solid: &BrepSolid,
    face_id: u64,
    edge: &EdgeRecord,
    at_start: bool,
) -> Result<Vec3, KernelRefusal> {
    let face = face_record(solid, face_id)?;
    let coedge = face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .find(|coedge| coedge.edge_id == edge.id)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Classify,
                "edge_use",
                format!("offset_ruled_face: face {face_id} does not use edge {}", edge.id),
            )
        })?;
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
    let mut normal = face.surface.normal(0.5 * (u0 + u1), 0.5 * (v0 + v1)).or_refuse(KernelStage::Classify, "normal")?;
    if !face.same_sense {
        normal = normal.scale(-1.0);
    }
    let parameter = if at_start { edge.t0 } else { edge.t1 };
    let mut tangent = edge.curve.derivatives(parameter, 1).or_refuse(KernelStage::Classify, "derivatives")?[1];
    if !coedge.forward {
        tangent = tangent.scale(-1.0);
    }
    normal.cross(tangent).normalized().or_refuse(KernelStage::Classify, "normalized")
}

/// The pushed carrier's radius at `point`'s axial station, and `point`'s own
/// distance from the axis.
fn mouth_radius_at(s_prime: &NurbsSurface, point: Vec3) -> Result<(f64, f64), KernelRefusal> {
    let Some(AnalyticSurface::RuledRevolution {
        frame,
        rho0,
        rho1,
        height,
    }) = s_prime.analytic()
    else {
        return Err(KernelRefusal::internal(
            KernelStage::Classify,
            "carrier_kind",
            "offset_ruled_face: the pushed carrier is not a ruled revolution",
        ));
    };
    let offset = point.sub(frame.origin);
    let station = offset.dot(frame.axis);
    let radial = offset.sub(frame.axis.scale(station)).length();
    Ok((rho0 + (rho1 - rho0) * station / height, radial))
}

/// Turn the tangent vertices into topology on `plan`, after the rim pass has
/// solved every rim corner into `new_vertex`.
///
/// Each tangent vertex is taken back out of `new_vertex` — it stays on its flat
/// — and its solved position becomes a new vertex that the two rim arcs are
/// re-attached to. The rest is the bridge lane or the merge lane.
#[allow(clippy::too_many_arguments)]
pub(super) fn plan_tangent_vertices(
    solid: &BrepSolid,
    face_id: u64,
    tangents: &[TangentVertex],
    faces_of_edge: &HashMap<u64, Vec<u64>>,
    edge_by_id: &HashMap<u64, &EdgeRecord>,
    vertex_pos: &HashMap<u64, Vec3>,
    s_prime: &NurbsSurface,
    plane_tolerance: f64,
    new_vertex: &mut HashMap<u64, Vec3>,
    cap_faces: &mut HashSet<u64>,
    plan: &mut RebuildPlan,
) -> Result<(), KernelRefusal> {
    if tangents.is_empty() {
        return Ok(());
    }
    let edge = |id: u64| {
        edge_by_id
            .get(&id)
            .copied()
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "missing_edge",
                    format!("offset_ruled_face: missing edge {id}"),
                )
            })
    };
    let old = |id: u64| {
        vertex_pos
            .get(&id)
            .copied()
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Fragment,
                    "vertex_pos",
                    format!("offset_ruled_face: missing vertex {id}"),
                )
            })
    };

    // Which way did the rim go at each tangent vertex?
    let mut over_floor: Option<bool> = None;
    let mut added: HashMap<u64, usize> = HashMap::default();
    for tangent in tangents {
        let before = old(tangent.vertex)?;
        let after = new_vertex.get(&tangent.vertex).copied().ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Fragment,
                "tangent_unsolved",
                format!(
                "offset_ruled_face: tangent vertex {} was not solved by the rim pass — refusing",
                tangent.vertex
            ),
            )
        })?;
        let moved = after.sub(before);
        if moved.length() <= plane_tolerance {
            return Err(KernelRefusal::input(
                KernelStage::Classify,
                "tangent_unmoved",
                format!(
                "offset_ruled_face: the push leaves tangent vertex {} where it is, so there is \
                 nothing to split — refusing",
                tangent.vertex
            ),
            ));
        }
        let mut sides = Vec::new();
        for (rim, floor) in [(tangent.rim_a, tangent.floor_a), (tangent.rim_b, tangent.floor_b)] {
            let rim = edge(rim)?;
            let inward = into_face(solid, floor, rim, rim.start_vertex_id == tangent.vertex)?;
            sides.push(moved.dot(inward) > 0.0);
        }
        if sides[0] != sides[1] {
            return Err(KernelRefusal::ill_posed(
                KernelStage::Classify,
                "tangent_mixed_sides",
                format!(
                "offset_ruled_face: at tangent vertex {} the rebuilt rim moves over one floor face \
                 and away from the other — refusing",
                tangent.vertex
            ),
            ));
        }
        match over_floor {
            Some(known) if known != sides[0] => {
                return Err(KernelRefusal::ill_posed(
                    KernelStage::Classify,
                    "tangent_mixed_vertices",
                    "offset_ruled_face: the rebuilt rim moves over the floor at some tangent \
                     vertices and away from it at others — refusing",
                ))
            }
            _ => over_floor = Some(sides[0]),
        }
        new_vertex.remove(&tangent.vertex);
        added.insert(tangent.vertex, plan.added_vertices.len());
        plan.added_vertices.push(after);
        for rim in [tangent.rim_a, tangent.rim_b] {
            let record = edge(rim)?;
            let end = if record.start_vertex_id == tangent.vertex {
                EdgeEnd::Start
            } else {
                EdgeEnd::End
            };
            plan.reattached.push((rim, end, PlanVertex::Added(added[&tangent.vertex])));
        }
    }
    let is_rim = |edge_id: u64| {
        faces_of_edge
            .get(&edge_id)
            .is_some_and(|faces| faces.contains(&face_id))
    };
    // A vertex as it stands once the rim arcs are re-attached.
    let end_of = |edge: &EdgeRecord, vertex: u64| -> PlanVertex {
        match added.get(&vertex) {
            Some(index) if is_rim(edge.id) => PlanVertex::Added(*index),
            _ => PlanVertex::Existing(vertex),
        }
    };

    let mut floors: Vec<u64> = tangents
        .iter()
        .flat_map(|tangent| [tangent.floor_a, tangent.floor_b])
        .collect();
    floors.sort_unstable();
    floors.dedup();

    if over_floor != Some(true) {
        // BRIDGE lane: one straight edge per tangent vertex, from the vertex on
        // its flat to the new one on the rim, spliced into both floor loops.
        let mut bridge: HashMap<u64, usize> = HashMap::default();
        for tangent in tangents {
            bridge.insert(tangent.vertex, plan.added_edges.len());
            plan.added_edges.push(PlannedEdge {
                start: PlanVertex::Existing(tangent.vertex),
                end: PlanVertex::Added(added[&tangent.vertex]),
                curve: make_line(old(tangent.vertex)?, plan.added_vertices[added[&tangent.vertex]]).or_refuse(KernelStage::Fragment, "make_line")?,
            });
        }
        for floor in &floors {
            let face = face_record(solid, *floor)?;
            let mut spliced = 0usize;
            let mut loops: Vec<Vec<PlannedCoedge>> = Vec::with_capacity(face.loops.len());
            for loop_record in &face.loops {
                let count = loop_record.coedges.len();
                let mut planned: Vec<PlannedCoedge> = Vec::with_capacity(count + 2);
                for index in 0..count {
                    let here = &loop_record.coedges[index];
                    let next = &loop_record.coedges[(index + 1) % count];
                    planned.push(PlannedCoedge {
                        edge: PlanEdge::Existing(here.edge_id),
                        forward: here.forward,
                    });
                    let (_, head) = traversal(edge(here.edge_id)?, here.forward);
                    let Some(tangent) = tangents.iter().find(|tangent| {
                        tangent.vertex == head && (tangent.floor_a == *floor || tangent.floor_b == *floor)
                    }) else {
                        continue;
                    };
                    let rims = [tangent.rim_a, tangent.rim_b];
                    // Arriving along the floor and leaving along the rim, the loop
                    // runs out across the bridge (vertex → rim); arriving along the
                    // rim, it runs back.
                    let forward = if rims.contains(&next.edge_id) && !rims.contains(&here.edge_id) {
                        true
                    } else if rims.contains(&here.edge_id) && !rims.contains(&next.edge_id) {
                        false
                    } else {
                        return Err(KernelRefusal::internal(
                            KernelStage::Fragment,
                            "bridge_splice",
                            format!(
                            "offset_ruled_face: floor face {floor} does not pass tangent vertex {} \
                             between its rim and its own edge — refusing",
                            tangent.vertex
                        ),
                        ));
                    };
                    planned.push(PlannedCoedge {
                        edge: PlanEdge::Added(bridge[&tangent.vertex]),
                        forward,
                    });
                    spliced += 1;
                }
                loops.push(planned);
            }
            let expected = tangents
                .iter()
                .filter(|tangent| tangent.floor_a == *floor || tangent.floor_b == *floor)
                .count();
            if spliced != expected {
                return Err(KernelRefusal::internal(
                    KernelStage::Fragment,
                    "bridge_count",
                    format!(
                    "offset_ruled_face: floor face {floor} meets {expected} tangent vertices but its \
                     loops pass {spliced} of them — refusing"
                ),
                ));
            }
            plan.faces.push(PlannedFace {
                face_id: *floor,
                flip_sense: false,
                loops,
            });
        }
        return Ok(());
    }

    // MERGE lane. The floor regions go only if the new mouth clears every one of
    // their corners; one it does not clear is where new faces would have to
    // start, so that is the refusal.
    let plane = plane_of_surface(&face_record(solid, floors[0])?.surface, plane_tolerance, "offset_ruled_face")?;
    let mut uses: Vec<(PlannedCoedge, PlanVertex, PlanVertex, bool)> = Vec::new();
    let mut beyond: Vec<u64> = Vec::new();
    for floor in &floors {
        let face = face_record(solid, *floor)?;
        let own = plane_of_surface(&face.surface, plane_tolerance, "offset_ruled_face")?;
        if own.normal.cross(plane.normal).length() > 1e-9
            || own.origin.sub(plane.origin).dot(plane.normal).abs() > plane_tolerance
        {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "floors_not_coplanar",
                format!(
                "offset_ruled_face: the floor faces the mouth crosses at its tangent vertices do not \
                 share one plane (face {floor}) — refusing"
            ),
            ));
        }
        for loop_record in &face.loops {
            for coedge in &loop_record.coedges {
                let record = edge(coedge.edge_id)?;
                let rim = is_rim(record.id);
                if !rim {
                    for vertex in [record.start_vertex_id, record.end_vertex_id] {
                        let point = old(vertex)?;
                        let (mouth, radial) = mouth_radius_at(s_prime, point)?;
                        if radial >= mouth - plane_tolerance {
                            return Err(KernelRefusal::unsupported(
                                KernelStage::Classify,
                                "mouth_short_of_corner",
                                format!(
                                "offset_ruled_face: the push carries the mouth over floor face \
                                 {floor} but not past its corner vertex {vertex} (the far end of \
                                 edge {}: {radial:.6} from the axis, the new mouth {mouth:.6}); the \
                                 floor would need new faces between the rim and that corner — refusing",
                                record.id
                            ),
                            ));
                        }
                    }
                    for neighbour in faces_of_edge.get(&record.id).cloned().unwrap_or_default() {
                        if !floors.contains(&neighbour) && !beyond.contains(&neighbour) {
                            beyond.push(neighbour);
                        }
                    }
                }
                let (tail, head) = traversal(record, coedge.forward);
                uses.push((
                    PlannedCoedge {
                        edge: PlanEdge::Existing(record.id),
                        forward: coedge.forward,
                    },
                    end_of(record, tail),
                    end_of(record, head),
                    rim,
                ));
            }
        }
    }
    // With the floor gone, its other neighbours (the flats) are bounded below
    // by the new face, so they must stand on the far side of the floor's plane
    // from the pushed face — otherwise the grown mouth cuts through them.
    let side = |face: u64| -> Result<(f64, f64), KernelRefusal> {
        let record = face_record(solid, face)?;
        let (mut low, mut high) = (0.0f64, 0.0f64);
        for coedge in record.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            let record = edge(coedge.edge_id)?;
            for vertex in [record.start_vertex_id, record.end_vertex_id] {
                let distance = old(vertex)?.sub(plane.origin).dot(plane.normal);
                low = low.min(distance);
                high = high.max(distance);
            }
        }
        Ok((low, high))
    };
    let (pushed_low, pushed_high) = side(face_id)?;
    let pushed_sign = if pushed_high > plane_tolerance && pushed_low >= -plane_tolerance {
        1.0
    } else if pushed_low < -plane_tolerance && pushed_high <= plane_tolerance {
        -1.0
    } else {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "pushed_face_straddles_floor",
            format!(
            "offset_ruled_face: the pushed face does not lie on one side of floor face {}'s plane \
             — refusing",
            floors[0]
        ),
        ));
    };
    for neighbour in &beyond {
        let (low, high) = side(*neighbour)?;
        let wrong = if pushed_sign > 0.0 {
            high > plane_tolerance
        } else {
            low < -plane_tolerance
        };
        if wrong {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "neighbour_cut_by_mouth",
                format!(
                "offset_ruled_face: face {neighbour} meets the floor on the pushed face's side of \
                 its plane, so a mouth grown past the floor would cut it — refusing"
            ),
            ));
        }
    }

    // Re-chain every floor coedge into loops. The rim becomes the outer loop.
    let mut open: Vec<bool> = vec![true; uses.len()];
    let mut loops: Vec<(bool, Vec<PlannedCoedge>)> = Vec::new();
    for seed in 0..uses.len() {
        if !open[seed] {
            continue;
        }
        let mut chain = Vec::new();
        let mut rim_count = 0usize;
        let mut at = seed;
        loop {
            open[at] = false;
            chain.push(uses[at].0);
            rim_count += usize::from(uses[at].3);
            let head = uses[at].2;
            if head == uses[seed].1 {
                break;
            }
            let successors: Vec<usize> = (0..uses.len())
                .filter(|index| open[*index] && uses[*index].1 == head)
                .collect();
            if successors.len() != 1 {
                return Err(KernelRefusal::internal(
                    KernelStage::Fragment,
                    "merge_loops",
                    format!(
                    "offset_ruled_face: merging floor faces {floors:?} leaves a boundary that does \
                     not close into loops ({} ways on from {head:?}) — refusing",
                    successors.len()
                ),
                ));
            }
            at = successors[0];
        }
        if rim_count != 0 && rim_count != chain.len() {
            return Err(KernelRefusal::internal(
                KernelStage::Fragment,
                "merge_rim_loop",
                format!(
                "offset_ruled_face: merging floor faces {floors:?} leaves a loop that runs along the \
                 rim and off it — refusing"
            ),
            ));
        }
        loops.push((rim_count != 0, chain));
    }
    if loops.iter().filter(|(rim, _)| *rim).count() != 1 {
        return Err(KernelRefusal::internal(
            KernelStage::Fragment,
            "merge_rim_count",
            format!(
            "offset_ruled_face: merging floor faces {floors:?} does not leave the rim as one loop — \
             refusing"
        ),
        ));
    }
    loops.sort_by_key(|(rim, _)| !*rim);
    let keep = floors[0];
    plan.faces.push(PlannedFace {
        face_id: keep,
        flip_sense: true,
        loops: loops.into_iter().map(|(_, chain)| chain).collect(),
    });
    for removed in &floors[1..] {
        plan.removed_faces.push(*removed);
        cap_faces.remove(removed);
    }
    Ok(())
}
