use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

struct HealPlan {
    /// The two boundary-edge indices whose neighbours are the primary pair.
    primary: [usize; 2],
    /// The two lateral boundary-edge indices (end caps).
    lateral: [usize; 2],
    /// Triple point (recovered corner) for each lateral index.
    lateral_point: HashMap<usize, Vec3>,
}

/// Choose the unique opposite neighbour pair whose planes re-intersect through
/// the transition face's region.
fn plan_heal(planes: &[Plane; 4], f_center: Vec3, f_reach: f64) -> Result<HealPlan, KernelRefusal> {
    let mut candidates: Vec<HealPlan> = Vec::new();
    for start in 0..2usize {
        let primary = [start, start + 2];
        let lateral = [(start + 1) % 4, (start + 3) % 4];
        let Some(line) = intersect_planes(&planes[primary[0]], &planes[primary[1]]) else {
            continue;
        };
        let mut lateral_point = HashMap::default();
        let mut ok = true;
        for &lat in &lateral {
            match intersect_line_plane(&line, &planes[lat]) {
                Some(point) if point.sub(f_center).length() <= f_reach => {
                    lateral_point.insert(lat, point);
                }
                _ => {
                    ok = false;
                    break;
                }
            }
        }
        if !ok {
            continue;
        }
        candidates.push(HealPlan {
            primary,
            lateral,
            lateral_point,
        });
    }
    match candidates.len() {
        1 => Ok(candidates.pop().unwrap()),
        0 => Err(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "no_reintersection",
            "delete_face_and_heal: the neighbours do not re-intersect cleanly \
                  (parallel planes, non-adjacent healing, or a multi-face gap) — refusing \
                  rather than emitting an invalid solid",
        )),
        _ => Err(KernelRefusal::ill_posed(
            KernelStage::Classify,
            "ambiguous_pair",
            "delete_face_and_heal: healing is ambiguous — both opposite neighbour \
                  pairs re-intersect through the deleted face",
        )),
    }
}

/// The position of a neighbour the transition face's own boundary named, so a
/// miss is a consistency failure of the solid rather than a caller error.
fn neighbour_position(solid: &BrepSolid, neighbour_id: u64) -> Result<(usize, usize), KernelRefusal> {
    find_face(solid, neighbour_id).ok_or_else(|| {
        KernelRefusal::internal(
            KernelStage::Collect,
            "missing_neighbour",
            format!("delete_face_and_heal: missing neighbour {neighbour_id}"),
        )
    })
}

// `retrim_planar_face` moved to the shared re-trim home
// (`crate::offset_retrim`, offset-unification audit §10) — its body is
// unchanged and it is re-exported here so its nine direct-edit call sites keep
// the bare name they have always used.
pub(super) use crate::offset_retrim::retrim_planar_face;

/// Find, in a face's loops, the (loop index, coedge index) of the coedge that
/// references `edge_id`.
pub(super) fn locate_coedge(face: &FaceRecord, edge_id: u64) -> Option<(usize, usize)> {
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            if coedge.edge_id == edge_id {
                return Some((loop_index, coedge_index));
            }
        }
    }
    None
}

pub(super) fn coedge_from_vertex(coedge: &CoedgeRecord, edges: &HashMap<u64, EdgeRecord>) -> Option<u64> {
    let edge = edges.get(&coedge.edge_id)?;
    Some(if coedge.forward {
        edge.start_vertex_id
    } else {
        edge.end_vertex_id
    })
}

pub(super) fn coedge_to_vertex(coedge: &CoedgeRecord, edges: &HashMap<u64, EdgeRecord>) -> Option<u64> {
    let edge = edges.get(&coedge.edge_id)?;
    Some(if coedge.forward {
        edge.end_vertex_id
    } else {
        edge.start_vertex_id
    })
}

/// Golovanov §6.12 — delete a transition face and heal the hole by extending
/// and re-intersecting its immediate neighbours. See the module docs for the
/// covered vs deferred cases. Returns a fresh solid that is guaranteed to
/// `validate()` AND to be sound, or a clear `Err` describing why the heal was
/// refused.
///
/// `validate()` is an incidence test and cannot see a body passing through
/// itself, which a re-intersected rim is exactly the kind of thing to produce:
/// a fold or a crossing changes the AREA where it does not change the volume,
/// so no closed-form oracle downstream notices. [`crate::accept_sound`] is
/// that floor, and it runs on the way out of the public entry only — the
/// set driver's chain calls [`delete_face_and_heal_impl`] so a ten-face
/// selection pays for one scan and not ten.
pub fn delete_face_and_heal(solid: &BrepSolid, face_id: u64) -> Result<BrepSolid, KernelRefusal> {
    crate::accept_sound(delete_face_and_heal_impl(solid, face_id)?, "deleteFace")
}

/// The heal itself, without the soundness floor: see [`delete_face_and_heal`].
pub(super) fn delete_face_and_heal_impl(
    solid: &BrepSolid,
    face_id: u64,
) -> Result<BrepSolid, KernelRefusal> {
    let mut solid = solid.clone();
    let scale = solid_model_scale(&solid);
    let tolerance = (scale * 1e-7).max(1e-9);

    let (shell_index, face_index) = find_face(&solid, face_id).ok_or_else(|| {
        KernelRefusal::input(
            KernelStage::Collect,
            "face_id",
            format!("delete_face_and_heal: no face with id {face_id}"),
        )
    })?;

    // --- Read the transition face's boundary (immutable snapshot) ---------
    let boundary: Vec<(u64, bool)> = {
        let face = &solid.shells[shell_index].faces[face_index];
        if face.loops.len() != 1 {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "multi_loop",
                format!(
                    "delete_face_and_heal: face {face_id} has {} loops; only a simple \
                     single-loop transition face is supported",
                    face.loops.len()
                ),
            ));
        }
        face.loops[0]
            .coedges
            .iter()
            .map(|coedge| (coedge.edge_id, coedge.forward))
            .collect()
    };
    if boundary.len() != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "side_count",
            format!(
                "delete_face_and_heal: face {face_id} has {} boundary edges; only 4-sided \
                 transition faces (a single chamfer/fillet edge) are supported \
                 (deferred: multi-face gaps)",
                boundary.len()
            ),
        ));
    }
    let boundary_edge_ids: HashSet<u64> = boundary.iter().map(|(edge_id, _)| *edge_id).collect();
    if boundary_edge_ids.len() == 3 {
        // A CLOSED transition strip (a fillet/chamfer around a full rim):
        // loop = [seam+, rim_a, seam-, rim_b] — the seam doubled, two closed
        // rims. Heals by re-intersecting the two neighbours analytically.
        return heal_closed_transition(&solid, shell_index, face_index, &boundary)
            ;
    }
    if boundary_edge_ids.len() != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "repeated_edge",
            "delete_face_and_heal: transition face uses an edge more than once \
                    (deferred: periodic/closed transition)",
        ));
    }

    // Neighbour faces (one per boundary edge, in loop order).
    let mut neighbour_ids = [0u64; 4];
    for (index, (edge_id, _)) in boundary.iter().enumerate() {
        neighbour_ids[index] =
            other_face_of_edge(&solid, *edge_id, face_id)?;
    }
    let unique: HashSet<u64> = neighbour_ids.iter().copied().collect();
    if unique.len() != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "repeated_neighbour",
            "delete_face_and_heal: the transition face touches a neighbour more \
                    than once (deferred: periodic/closed transition)",
        ));
    }

    // Carriers of the neighbours: all-planar takes the closed-form planar
    // path below; any curved analytic neighbour routes to the mixed
    // open-chain heal (plane × cylinder/ruled-revolution re-intersection).
    let neighbour_planes: [Plane; 4] = {
        let mut planes: Vec<Option<Plane>> = Vec::with_capacity(4);
        for &neighbour_id in &neighbour_ids {
            let (ns, nf) = neighbour_position(&solid, neighbour_id)?;
            planes.push(
                plane_of_surface(
                    &solid.shells[ns].faces[nf].surface,
                    (scale * 1e-6).max(1e-7),
                    "delete_face_and_heal",
                )
                .ok(),
            );
        }
        if planes.iter().any(Option::is_none) {
            return heal_open_transition_mixed(
                &solid,
                shell_index,
                face_index,
                &boundary,
                &neighbour_ids,
            )
            ;
        }
        [
            planes[0].unwrap(),
            planes[1].unwrap(),
            planes[2].unwrap(),
            planes[3].unwrap(),
        ]
    };

    // Transition-face region gate (centre + reach), from its loop vertices.
    let transition_vertices: Vec<u64> = boundary
        .iter()
        .map(|(edge_id, forward)| {
            let edge = solid
                .edges
                .iter()
                .find(|edge| edge.id == *edge_id)
                .ok_or_else(|| {
                    KernelRefusal::internal(
                        KernelStage::Collect,
                        "missing_edge",
                        format!("delete_face_and_heal: missing edge {edge_id}"),
                    )
                })?;
            Ok(if *forward {
                edge.start_vertex_id
            } else {
                edge.end_vertex_id
            })
        })
        .collect::<Result<Vec<u64>, KernelRefusal>>()?;
    if transition_vertices.iter().collect::<HashSet<_>>().len() != 4 {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "repeated_corner",
            "delete_face_and_heal: transition face has repeated corner vertices \
                    (deferred: degenerate transition)",
        ));
    }
    let mut f_center = Vec3::default();
    for &vertex_id in &transition_vertices {
        f_center = f_center.add(edge_point(&solid, vertex_id)?);
    }
    f_center = f_center.scale(0.25);
    let mut f_reach = 0.0f64;
    for &vertex_id in &transition_vertices {
        f_reach = f_reach.max(
            edge_point(&solid, vertex_id)
                ?
                .sub(f_center)
                .length(),
        );
    }
    let f_reach = f_reach * 3.0 + tolerance;

    if let Some(directory) = census_directory() {
        record_planar_open_heal(&directory, &solid, face_id);
    }
    let plan = plan_heal(&neighbour_planes, f_center, f_reach)?;

    // --- New recovered corners (triple points) and the new sharp edge ------
    let mut next_id = max_topology_id(&solid) + 1;
    let mut alloc = || {
        let value = next_id;
        next_id += 1;
        value
    };

    let lateral_a = plan.lateral[0];
    let lateral_b = plan.lateral[1];
    let point_a = plan.lateral_point[&lateral_a];
    let point_b = plan.lateral_point[&lateral_b];
    if point_a.sub(point_b).length() <= tolerance {
        return Err(KernelRefusal::unsupported(
            KernelStage::Classify,
            "coincident_corners",
            "delete_face_and_heal: recovered corners coincide — the neighbours \
                    do not bound a clean edge",
        ));
    }
    let vertex_a = alloc();
    let vertex_b = alloc();
    let mut lateral_vertex: HashMap<usize, u64> = HashMap::default();
    lateral_vertex.insert(lateral_a, vertex_a);
    lateral_vertex.insert(lateral_b, vertex_b);

    // The new sharp edge S (start = corner A, end = corner B).
    let sharp_edge_id = alloc();
    let sharp_edge = EdgeRecord {
        id: sharp_edge_id,
        curve: make_line(point_a, point_b).or_refuse(KernelStage::Fragment, "make_line")?,
        t0: 0.0,
        t1: 1.0,
        start_vertex_id: vertex_a,
        end_vertex_id: vertex_b,
        degenerate: false,
        name: None,
    };

    // --- Collapse each transition corner onto its recovered corner ---------
    // Corner vertex `transition_vertices[i]` sits between neighbour[(i+3)%4]
    // and neighbour[i]; exactly one of those two is a lateral face, and the
    // corner collapses onto that lateral's recovered corner.
    let lateral_set: HashSet<usize> = plan.lateral.iter().copied().collect();
    let mut collapse: HashMap<u64, u64> = HashMap::default();
    for index in 0..4usize {
        let previous = (index + 3) % 4;
        let lateral_index = if lateral_set.contains(&previous) {
            previous
        } else if lateral_set.contains(&index) {
            index
        } else {
            return Err(KernelRefusal::internal(
                KernelStage::Classify,
                "corner_flank",
                "delete_face_and_heal: transition corner is not flanked by a \
                        lateral face (unexpected neighbour ordering)",
            ));
        };
        let target = lateral_vertex[&lateral_index];
        collapse.insert(transition_vertices[index], target);
    }
    let new_vertex_points: HashMap<u64, Vec3> = [(vertex_a, point_a), (vertex_b, point_b)]
        .into_iter()
        .collect();

    // Relocate every non-transition edge that ends on a collapsed corner.
    let unmoved = solid.edges.clone();
    let mut moved: HashSet<u64> = HashSet::default();
    for edge in &mut solid.edges {
        if boundary_edge_ids.contains(&edge.id) {
            continue;
        }
        let start_target = collapse.get(&edge.start_vertex_id).copied();
        let end_target = collapse.get(&edge.end_vertex_id).copied();
        if start_target.is_none() && end_target.is_none() {
            continue;
        }
        moved.insert(edge.id);
        if edge.curve.straight_segment(tolerance).is_none() {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "curved_side_edge",
                "delete_face_and_heal: a side edge meeting the transition face is \
                        not a straight line (deferred: curved neighbour edges)",
            ));
        }
        // The edge's OWN ends, at `t0`/`t1`. `straight_segment` answers "is this
        // a line" with the first and last control points — the WHOLE curve's ends
        // — and a side edge is often a subrange of its line: a cap's top edge
        // already trimmed back by another blend on its far end. Taking the curve's
        // ends there moved that far end back out to the old sharp corner, so
        // deleting one of two chamfers rebuilt the cap edge right through the
        // other one.
        let mut start_point = edge
            .curve
            .evaluate(edge.t0)
            .or_refuse(KernelStage::Fragment, "evaluate")?;
        let mut end_point = edge
            .curve
            .evaluate(edge.t1)
            .or_refuse(KernelStage::Fragment, "evaluate")?;
        if let Some(target) = start_target {
            edge.start_vertex_id = target;
            start_point = new_vertex_points[&target];
        }
        if let Some(target) = end_target {
            edge.end_vertex_id = target;
            end_point = new_vertex_points[&target];
        }
        if start_point.sub(end_point).length() <= tolerance {
            return Err(KernelRefusal::unsupported(
                KernelStage::Fragment,
                "collapsed_side_edge",
                "delete_face_and_heal: healing would collapse a side edge to zero \
                        length (deferred: degenerate transition)",
            ));
        }
        edge.curve = make_line(start_point, end_point).or_refuse(KernelStage::Fragment, "make_line")?;
        edge.t0 = 0.0;
        edge.t1 = 1.0;
    }
    // One plane per neighbour, read by the gate and by the re-trim below alike.
    let retrimmed: HashMap<u64, Plane> =
        neighbour_ids.iter().copied().zip(neighbour_planes.iter().copied()).collect();
    moved_edges_lie_on_their_faces(
        &unmoved,
        &solid,
        &moved,
        &retrimmed,
        (scale * 1e-6).max(1e-7),
        "delete_face_and_heal",
    )
    ?;

    // Index the (now relocated) edges for loop rewrites and pcurve rebuilds.
    let mut edges_by_id: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    edges_by_id.insert(sharp_edge_id, sharp_edge.clone());

    // --- Rewrite neighbour loops ------------------------------------------
    // Primary faces: replace their support coedge (on the deleted face's edge)
    // with a coedge on the new sharp edge S.
    for &primary_index in &plan.primary {
        let primary_edge = boundary[primary_index].0;
        let neighbour_id = neighbour_ids[primary_index];
        let (ns, nf) = neighbour_position(&solid, neighbour_id)?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, primary_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "primary_coedge",
                format!(
                    "delete_face_and_heal: neighbour {neighbour_id} does not use edge {primary_edge}"
                ),
            )
        })?;
        let coedges = &face.loops[loop_index].coedges;
        let count = coedges.len();
        let previous = &coedges[(coedge_index + count - 1) % count];
        let next = &coedges[(coedge_index + 1) % count];
        let required_from = coedge_to_vertex(previous, &edges_by_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "loop_connectivity_from",
                "delete_face_and_heal: could not resolve loop connectivity",
            )
        })?;
        let required_to = coedge_from_vertex(next, &edges_by_id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "loop_connectivity_to",
                "delete_face_and_heal: could not resolve loop connectivity",
            )
        })?;
        let forward = if required_from == vertex_a && required_to == vertex_b {
            true
        } else if required_from == vertex_b && required_to == vertex_a {
            false
        } else {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "primary_loop_closure",
                "delete_face_and_heal: new edge does not close the primary loop \
                        (unexpected connectivity)",
            ));
        };
        let new_coedge = CoedgeRecord {
            id: alloc(),
            edge_id: sharp_edge_id,
            forward,
            // Placeholder; retrim_planar_face recomputes every pcurve below.
            pcurve: make_line(Vec3::default(), Vec3::new(1.0, 0.0, 0.0))
                .or_refuse(KernelStage::Sew, "make_line")?,
        };
        face.loops[loop_index].coedges[coedge_index] = new_coedge;
    }

    // Lateral (cap) faces: drop the cap coedge; its two neighbours already
    // meet at the recovered corner.
    for &lateral_index in &plan.lateral {
        let lateral_edge = boundary[lateral_index].0;
        let neighbour_id = neighbour_ids[lateral_index];
        let (ns, nf) = neighbour_position(&solid, neighbour_id)?;
        let face = &mut solid.shells[ns].faces[nf];
        let (loop_index, coedge_index) = locate_coedge(face, lateral_edge).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "lateral_coedge",
                format!(
                    "delete_face_and_heal: neighbour {neighbour_id} does not use edge {lateral_edge}"
                ),
            )
        })?;
        face.loops[loop_index].coedges.remove(coedge_index);
        if face.loops[loop_index].coedges.is_empty() {
            return Err(KernelRefusal::internal(
                KernelStage::Sew,
                "emptied_loop",
                "delete_face_and_heal: healing emptied a lateral face loop",
            ));
        }
    }

    // --- Prune the deleted face, its edges, and its corner vertices --------
    solid.shells[shell_index]
        .faces
        .retain(|face| face.id != face_id);
    solid
        .edges
        .retain(|edge| !boundary_edge_ids.contains(&edge.id));
    let removed_vertices: HashSet<u64> = transition_vertices.iter().copied().collect();
    solid.edges.push(sharp_edge);
    solid
        .vertices
        .retain(|vertex| !removed_vertices.contains(&vertex.id));
    solid.vertices.push(VertexRecord {
        id: vertex_a,
        point: point_a,
    });
    solid.vertices.push(VertexRecord {
        id: vertex_b,
        point: point_b,
    });

    // --- Re-trim every affected neighbour on its (extended) carrier --------
    let final_edges: HashMap<u64, EdgeRecord> = solid
        .edges
        .iter()
        .map(|edge| (edge.id, edge.clone()))
        .collect();
    for neighbour_id in neighbour_ids {
        let plane = &retrimmed[&neighbour_id];
        let (ns, nf) = neighbour_position(&solid, neighbour_id)?;
        retrim_planar_face(
            &mut solid.shells[ns].faces[nf],
            plane,
            &final_edges,
            scale,
            "delete_face_and_heal",
        )
        ?;
    }

    // Genus is preserved by removing a genus-neutral transition face; the
    // Euler check inside validate() confirms it.
    let issues = solid.validate();
    if !issues.is_empty() {
        return Err(KernelRefusal::internal(
            KernelStage::Validate,
            "validate",
            format!("delete_face_and_heal: healed solid failed validation: {issues:?}"),
        ));
    }
    Ok(solid)
}

/// Resolve the id of the face nearest a 3D point (used by the app: the picked
/// point lies on the selected face). Mirrors `resolve_edge_by_point`.
pub fn resolve_face_by_point(solid: &BrepSolid, point: Vec3) -> Result<u64, KernelRefusal> {
    let scale = solid_model_scale(&solid);
    let mut best: Option<(u64, f64)> = None;
    for shell in &solid.shells {
        for face in &shell.faces {
            let Ok(projection) = crate::project_point_to_surface(&face.surface, point) else {
                continue;
            };
            if best
                .map(|(_, known)| projection.distance < known)
                .unwrap_or(true)
            {
                best = Some((face.id, projection.distance));
            }
        }
    }
    match best {
        Some((face_id, distance)) if distance <= (scale * 1e-3).max(1e-4) => Ok(face_id),
        Some((_, distance)) => Err(KernelRefusal::input(
            KernelStage::Collect,
            "point_off_faces",
            format!(
                "delete_face_and_heal: no face within tolerance of the point (nearest {distance:.6})"
            ),
        )),
        None => Err(KernelRefusal::input(
            KernelStage::Collect,
            "no_faces",
            "delete_face_and_heal: solid has no faces",
        )),
    }
}

