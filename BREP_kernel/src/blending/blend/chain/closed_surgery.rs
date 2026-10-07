use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

pub(super) fn chain_surgery(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    rows: ChainRows,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);
    let [u_start, u_end] = rows.u_domain;

    // ---- side descriptors -------------------------------------------
    // For each side, group consecutive segments by face; a junction
    // between different faces needs a spoke edge; a side whose every
    // segment shares one face has no crossings at all.
    struct SideJunction {
        /// Junction index (between segment j-1 and j, cyclic).
        junction: usize,
        spoke_edge_id: u64,
        crossing_support: f64,
        crossing_spoke: f64,
        vertex_id: u64,
    }
    struct SidePlan {
        junctions: Vec<SideJunction>,
    }
    let chain_vertex_at = |junction: usize| -> u64 {
        // Junction j sits between segment j-1 and segment j (cyclic);
        // it is segment j's chain-entry vertex.
        let segment = &segments[junction % segments.len()];
        if segment.forward {
            segment.edge.start_vertex_id
        } else {
            segment.edge.end_vertex_id
        }
    };
    let mut plans = Vec::new();
    // Tracks whether any junction below needed a looser-than-1e-7 spoke
    // crossing; if so the finished solid is validated before it is handed back.
    let mut crossings = SpokeCrossings::default();
    // A side "needs seam surgery" when any of its junctions is a carrier
    // SEAM (the mate face is the same across the junction, i.e. a closed
    // cylinder/cone/sphere whose seam the chain crosses) rather than a
    // face-change SPOKE.  Those sides rebuild their mate loops run-by-run.
    let mut side_has_seam = [false, false];
    for side in 0..2 {
        let support = if side == 0 { &rows.cr } else { &rows.cs };
        let mut junctions = Vec::new();
        for junction in 0..segments.len() {
            let before = &segments[(junction + segments.len() - 1) % segments.len()];
            let after = &segments[junction];
            let before_mate = before.mate(side);
            let after_mate = after.mate(side);
            let (face_before, loop_before) = (before_mate.face, before_mate.loop_index);
            let (face_after, loop_after) = (after_mate.face, after_mate.loop_index);
            let vertex = chain_vertex_at(junction);
            let Some(spoke) = cross_edge_at(
                solid,
                face_before,
                loop_before,
                face_after,
                loop_after,
                vertex,
                before.edge.id,
                after.edge.id,
            )?
            else {
                // Same face and no seam here: the chain flows through.
                continue;
            };
            if face_before.id == face_after.id {
                side_has_seam[side] = true;
            }
            let spoke_record = solid
                .edges
                .iter()
                .find(|edge| edge.id == spoke)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "chain_spoke_edge", "blend: chain spoke edge missing"))?;
            let vertex_point = result
                .vertices
                .iter()
                .find(|candidate| candidate.id == vertex)
                .map(|candidate| candidate.point)
                .unwrap_or_default();
            let Some((crossing_support, crossing_spoke, escalated)) =
                spoke_crossing(solid, support, spoke_record, vertex_point)?
            else {
                return Err(KernelRefusal::internal(KernelStage::Sew, "support_misses_spoke", "blend: support does not cross a chain spoke"));
            };
            crossings.note(escalated);
            junctions.push(SideJunction {
                junction,
                spoke_edge_id: spoke,
                crossing_support,
                crossing_spoke,
                vertex_id: vertex,
            });
        }
        plans.push(SidePlan { junctions });
    }

    // ---- support edges: split at crossings and the seam --------------
    // Crossing vertices first.
    let mut side_edges: Vec<Vec<RimPiece>> = Vec::new();
    let mut seam_vertices = [0u64; 2];
    // Crossing (cut) vertices per side — used by the seam-aware run rebuild
    // to pin piece pcurves onto the carrier meridian.
    let mut side_crossing_vertices: Vec<Vec<u64>> = Vec::new();
    for (side, plan) in plans.iter().enumerate() {
        let support = if side == 0 { &rows.cr } else { &rows.cs };
        let seam_point = support.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
        let seam_vertex = take_id();
        result.vertices.push(VertexRecord {
            id: seam_vertex,
            point: seam_point,
        });
        seam_vertices[side] = seam_vertex;
        // Sorted crossing parameters with their new vertices.
        let mut cuts: Vec<(f64, u64, usize)> = Vec::new();
        let mut crossing_vertices = Vec::new();
        for junction in &plan.junctions {
            let vertex = take_id();
            result.vertices.push(VertexRecord {
                id: vertex,
                point: support.evaluate(junction.crossing_support).or_refuse(KernelStage::Refine, "evaluate")?,
            });
            crossing_vertices.push(vertex);
            cuts.push((junction.crossing_support, vertex, junction.junction));
        }
        side_crossing_vertices.push(crossing_vertices);
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
        // Pieces between consecutive cut parameters (wrapping through the
        // seam): [seam..c0], [c0..c1], ..., [ck..seam-end].
        let mut pieces = Vec::new();
        let mut boundaries = vec![(u_start, seam_vertex)];
        boundaries.extend(
            cuts.iter()
                .map(|(parameter, vertex, _)| (*parameter, *vertex)),
        );
        boundaries.push((u_end, seam_vertex));
        for window in boundaries.windows(2) {
            let (a, vertex_a) = window[0];
            let (b, vertex_b) = window[1];
            if b - a <= 1e-9 {
                return Err(KernelRefusal::internal(KernelStage::Sew, "degenerate_support_piece", "blend: degenerate support piece between crossings"));
            }
            let piece_curve = {
                let (_, tail) = if a > u_start + 1e-12 {
                    support.split(a).or_refuse(KernelStage::Refine, "split")?
                } else {
                    (support.clone(), support.clone())
                };
                let tail = if a > u_start + 1e-12 {
                    tail
                } else {
                    support.clone()
                };
                if b < u_end - 1e-12 {
                    tail.split(b).or_refuse(KernelStage::Refine, "split")?.0
                } else {
                    tail
                }
            };
            let edge_id = take_id();
            let [piece_t0, piece_t1] = piece_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            result.edges.push(EdgeRecord {
                id: edge_id,
                curve: piece_curve,
                t0: piece_t0,
                t1: piece_t1,
                start_vertex_id: vertex_a,
                end_vertex_id: vertex_b,
                degenerate: false,
                name: None,
            });
            pieces.push(RimPiece {
                face_id: 0,
                loop_index: 0,
                window: [a, b],
                edge_id,
            });
        }
        side_edges.push(pieces);
    }

    // Blend seam edge between the two rim seam vertices.
    let seam_curve = rows.surface.iso_curve_u(u_start).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [seam_t0, seam_t1] = seam_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let blend_seam_id = take_id();
    result.edges.push(EdgeRecord {
        id: blend_seam_id,
        curve: seam_curve,
        t0: seam_t0,
        t1: seam_t1,
        start_vertex_id: seam_vertices[0],
        end_vertex_id: seam_vertices[1],
        degenerate: false,
        name: None,
    });

    // ---- trim spokes -------------------------------------------------
    for (side, plan) in plans.iter().enumerate() {
        for junction in &plan.junctions {
            // The crossing vertex created for this junction.
            let vertex = side_edges[side]
                .iter()
                .find_map(|piece| {
                    (piece.window[0] - junction.crossing_support)
                        .abs()
                        .le(&1e-12)
                        .then(|| {
                            result
                                .edges
                                .iter()
                                .find(|edge| edge.id == piece.edge_id)
                                .map(|edge| edge.start_vertex_id)
                        })
                })
                .flatten()
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "crossing_vertex", "blend: crossing vertex lost"))?;
            trim_edge_at(
                &mut result,
                junction.spoke_edge_id,
                junction.crossing_spoke,
                junction.vertex_id,
                vertex,
            )?;
        }
    }

    // ---- replace chain coedges in the mate faces ---------------------
    // For each side, each face's run of chain coedges is replaced by its
    // support pieces (in loop-traversal order and sense).
    let mut side_use_forward = [true, true];
    for side in 0..2 {
        // Map each piece window to the segment(s) it covers, and thus the
        // face.  A piece's midpoint parameter falls inside exactly one
        // segment's window unless the side has no crossings (single face).
        let pieces = &side_edges[side];
        let face_info = |segment: &ChainSegment| {
            let mate = segment.mate(side);
            (mate.face.id, mate.loop_index, mate.coedge.id)
        };
        if side_has_seam[side] {
            // Seam-carrier mate(s): one closed carrier's loop holds the
            // chain coedges in MULTIPLE runs split by its OWN seam edges
            // (which we already trimmed to the support crossings).  Rebuild
            // each mate loop run-by-run, projecting each support piece onto
            // the carrier so the pcurve never wraps the seam.
            use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
            let fitted = if side == 0 {
                &rows.pcurves1
            } else {
                &rows.pcurves2
            };
            let segment_mid = |index: usize| -> f64 {
                let (low, high, _) = &fitted[index];
                (low + high) * 0.5
            };
            let piece_segment = |piece: &RimPiece| -> usize {
                let midpoint = rows.to_global((piece.window[0] + piece.window[1]) * 0.5);
                let mut best = (0usize, f64::INFINITY);
                for segment_index in 0..segments.len() {
                    let sm = segment_mid(segment_index);
                    let distance = (sm - midpoint)
                        .abs()
                        .min((sm - midpoint + 1.0).abs())
                        .min((sm - midpoint - 1.0).abs());
                    if distance < best.1 {
                        best = (segment_index, distance);
                    }
                }
                best.0
            };
            let crossing_set: HashSet<u64> = side_crossing_vertices[side].iter().copied().collect();
            // Distinct mate faces on this side (usually a single carrier).
            let mut faces: Vec<(u64, usize)> = Vec::new();
            for segment in segments {
                let (face_id, loop_index, _) = face_info(segment);
                if !faces.iter().any(|(id, _)| *id == face_id) {
                    faces.push((face_id, loop_index));
                }
            }
            for (face_id, loop_index) in faces {
                // Which chain segments live in this face, keyed by edge id.
                let mut edge_to_segment: HashMap<u64, usize> = HashMap::default();
                for (segment_index, segment) in segments.iter().enumerate() {
                    if face_info(segment).0 == face_id {
                        edge_to_segment.insert(segment.edge.id, segment_index);
                    }
                }
                let surface = {
                    let face = result
                        .shells
                        .iter()
                        .flat_map(|shell| &shell.faces)
                        .find(|face| face.id == face_id)
                        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during chain surgery"))?;
                    face.surface.clone()
                };
                let loop_coedges = {
                    let face = result
                        .shells
                        .iter()
                        .flat_map(|shell| &shell.faces)
                        .find(|face| face.id == face_id)
                        .unwrap();
                    face.loops[loop_index].coedges.clone()
                };
                let count = loop_coedges.len();
                let is_chain =
                    |coedge: &CoedgeRecord| edge_to_segment.contains_key(&coedge.edge_id);
                // Rotate so index 0 is a NON-chain coedge (a carrier seam)
                // — then no run of chain coedges wraps the coedge vector.
                // If a face carries no seam edge of its own (a plain face on
                // a mixed side), its chain coedges form a single whole-loop
                // run; anchoring at 0 is then fine.
                let rotation = (0..count)
                    .find(|&i| !is_chain(&loop_coedges[i]))
                    .unwrap_or(0);
                let rotated: Vec<CoedgeRecord> = (0..count)
                    .map(|i| loop_coedges[(i + rotation) % count].clone())
                    .collect();
                // The face's loop winds +chain or −chain (constant per face).
                let chain_forward = {
                    let first_chain = rotated.iter().find(|coedge| is_chain(coedge)).unwrap();
                    let segment_index = edge_to_segment[&first_chain.edge_id];
                    first_chain.forward == segments[segment_index].forward
                };
                side_use_forward[side] = chain_forward;
                // EACH SEAM-BOUNDED RUN IS BUILT BY ITS TOPOLOGY, not by which
                // segment a piece's parameter window is nearest. The rail is
                // offset from the edge it replaces, so it crosses a carrier seam
                // where the edge did not (the 20-degree seam-split chain's fat rail
                // crosses inside segment 0): assigning pieces to runs by segment
                // spliced a piece into the wrong run and left the loop open. A run
                // starts at the END vertex and uv of the carrier coedge before it
                // and must arrive at the START vertex and uv of the one after; each
                // step takes the one unused piece of this face whose run-sense start
                // vertex is the current vertex AND whose projected start uv is the
                // current uv. A run never jumps a period: only the carrier's own
                // seam coedges carry the shift. Continuity is read at the kernel's
                // pcurve-consistency contract turned into a uv band at the point
                // (`surface_uv_tolerance`). That band is the SPATIAL bar; which
                // chart branch a joint sits on is a separate check
                // (`uv_joint_continuous`): strict raw uv never accepts a jump of
                // half a closed period or more, however large the band (the
                // derivative floor makes it huge at a pole).
                let spatial = crate::KernelTolerances::for_solid(solid, 1e-7).pcurve_consistency;
                let uv_unreadable = |why: String| {
                    KernelRefusal::internal(KernelStage::Sew, "chain_seam_run", format!(
                        "blend: a seam-split rim's uv on face {face_id} is unreadable: {why}"
                    ))
                };
                let uv_band = |uv: [f64; 2]| -> Result<f64, KernelRefusal> {
                    if !uv.iter().all(|value| value.is_finite()) {
                        return Err(uv_unreadable(format!("uv ({}, {})", uv[0], uv[1])));
                    }
                    let derivatives = surface.derivatives_extended(uv[0], uv[1], 1).or_refuse(KernelStage::Sew, "derivatives_extended")?;
                    checked_uv_band(spatial, derivatives[1][0].length(), derivatives[0][1].length()).map_err(uv_unreadable)
                };
                let edge_vertices = |edge_id: u64| -> Result<(u64, u64), KernelRefusal> {
                    result
                        .edges
                        .iter()
                        .find(|edge| edge.id == edge_id)
                        .map(|edge| (edge.start_vertex_id, edge.end_vertex_id))
                        .ok_or(KernelRefusal::internal(KernelStage::Sew, "support_piece_edge", "blend: support piece edge lost"))
                };
                let pcurve_end = |pcurve: &crate::NurbsCurve, at_start: bool| -> Result<[f64; 2], KernelRefusal> {
                    let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
                    let uv = pcurve.evaluate(if at_start { t0 } else { t1 }).or_refuse(KernelStage::Sew, "evaluate")?;
                    Ok([uv.x, uv.y])
                };
                // A coedge's (start vertex, start uv, end vertex, end uv) in loop sense.
                let coedge_ends = |coedge: &CoedgeRecord| -> Result<(u64, [f64; 2], u64, [f64; 2]), KernelRefusal> {
                    let (start, end) = edge_vertices(coedge.edge_id)?;
                    let (start, end) = if coedge.forward { (start, end) } else { (end, start) };
                    Ok((start, pcurve_end(&coedge.pcurve, true)?, end, pcurve_end(&coedge.pcurve, false)?))
                };
                // This face's pieces (the same face association as before: the
                // pieces of the segments the face carries), each projected once —
                // the projection is a pure function of the piece — and oriented
                // to the loop's sense.
                let face_segments: HashSet<usize> = edge_to_segment.values().copied().collect();
                let mut pool: Vec<(&RimPiece, u64, u64, crate::NurbsCurve)> = Vec::new();
                for piece in pieces.iter().filter(|piece| face_segments.contains(&piece_segment(piece))) {
                    let curve = result
                        .edges
                        .iter()
                        .find(|edge| edge.id == piece.edge_id)
                        .ok_or(KernelRefusal::internal(KernelStage::Sew, "support_piece_edge", "blend: support piece edge lost"))?
                        .curve
                        .clone();
                    let (start_vertex, end_vertex) = edge_vertices(piece.edge_id)?;
                    let base = project_piece_pcurve(
                        &curve,
                        &surface,
                        crossing_set.contains(&start_vertex),
                        crossing_set.contains(&end_vertex),
                    )?;
                    let (from, to, pcurve) = if chain_forward {
                        (start_vertex, end_vertex, base)
                    } else {
                        (end_vertex, start_vertex, base.reversed().or_refuse(KernelStage::Refine, "reversed")?)
                    };
                    pool.push((piece, from, to, pcurve));
                }
                // The surface's actual closed periods. Read after every piece is
                // projected: `project_piece_pcurve` (and `derivatives_extended`)
                // read the same three on this surface first, so these reads add
                // no refusal of their own. An empty pool projected nothing, and
                // its first run is `Stuck` before any joint is read, so the
                // periods are not read there either.
                let periods: [Option<f64>; 2] = if pool.is_empty() {
                    [None, None]
                } else {
                    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Sew, "closed_directions")?;
                    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
                    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
                    [closed_u.then_some(u1 - u0), closed_v.then_some(v1 - v0)]
                };
                let joint = |a: [f64; 2], b: [f64; 2], rule: UvJoint| -> Result<bool, KernelRefusal> {
                    uv_joint_continuous(a, b, uv_band(a)?, periods, rule).map_err(uv_unreadable)
                };
                let same_uv = |a: [f64; 2], b: [f64; 2]| -> Result<bool, KernelRefusal> { joint(a, b, UvJoint::Strict) };
                let mut used = vec![false; pool.len()];
                // Each piece's run-sense ends, read once for every run's walk.
                let entries: Vec<(u64, u64, [f64; 2], [f64; 2])> = pool
                    .iter()
                    .map(|(_, from, to, pcurve)| Ok((*from, *to, pcurve_end(pcurve, true)?, pcurve_end(pcurve, false)?)))
                    .collect::<Result<_, KernelRefusal>>()?;
                let decline = |why: String| {
                    KernelRefusal::internal(KernelStage::Sew, "chain_seam_run", format!(
                        "blend: the seam-split rim's support pieces on face {face_id} do not close its loop: {why}"
                    ))
                };
                let mut new_coedges: Vec<CoedgeRecord> = Vec::new();
                // Parallel to `new_coedges`: whether the coedge is a newly walked
                // piece (true) or the loop's own untouched carrier coedge (false).
                let mut walked_coedges: Vec<bool> = Vec::new();
                let mut position = 0;
                while position < count {
                    if !is_chain(&rotated[position]) {
                        new_coedges.push(rotated[position].clone());
                        walked_coedges.push(false);
                        position += 1;
                        continue;
                    }
                    // A maximal run of chain coedges (bounded by carrier seams).
                    let _run_start = position;
                    while position < count && is_chain(&rotated[position]) {
                        position += 1;
                    }
                    let previous = new_coedges.last().ok_or_else(|| decline("a run has no carrier coedge before it".into()))?;
                    let (_, _, vertex, uv) = coedge_ends(previous)?;
                    let (target_vertex, target_uv, _, _) = coedge_ends(&rotated[position % count])?;
                    #[cfg(not(test))]
                    let legacy: Option<Vec<usize>> = None;
                    let walked = match legacy {
                        Some(order) => {
                            for &index in &order {
                                used[index] = true;
                            }
                            RunWalk::Path(order)
                        }
                        None => walk_seam_run(&entries, &mut used, (vertex, uv), (target_vertex, target_uv), &same_uv)?,
                    };
                    match walked {
                        RunWalk::Path(path) => {
                            for index in path {
                                let (piece, _, _, pcurve) = &pool[index];
                                new_coedges.push(CoedgeRecord {
                                    id: take_id(),
                                    edge_id: piece.edge_id,
                                    forward: chain_forward,
                                    pcurve: pcurve.clone(),
                                });
                                walked_coedges.push(true);
                            }
                        }
                        RunWalk::Ambiguous(at, candidates) => {
                            return Err(KernelRefusal::unsupported(KernelStage::Classify, "chain_cross_edges", format!(
                                "blend: {} support pieces (edges {:?}) leave chain vertex {at} at the same uv on face {face_id}",
                                candidates.len(),
                                candidates.iter().map(|&i| pool[i].0.edge_id).collect::<Vec<_>>()
                            )));
                        }
                        RunWalk::Stuck(at, at_uv) => {
                            return Err(decline(format!(
                                "no unused piece leaves vertex {at} at uv ({:.9}, {:.9})", at_uv[0], at_uv[1]
                            )));
                        }
                    }
                }
                if let Some(index) = used.iter().position(|used| !used) {
                    return Err(decline(format!("support piece edge {} is left out of every run", pool[index].0.edge_id)));
                }
                // The rebuilt loop closes at every joint: in vertex everywhere; in
                // STRICT raw uv at every joint that touches a walked piece; and,
                // at a joint between two untouched carrier coedges, in uv modulo
                // only the surface's actual closed periods (an untouched loop may
                // already cross its own chart edge there).
                let ends: Vec<(u64, [f64; 2], u64, [f64; 2])> =
                    new_coedges.iter().map(&coedge_ends).collect::<Result<_, KernelRefusal>>()?;
                if let Some(index) = first_open_joint(&ends, &walked_coedges, &joint)? {
                    return Err(decline(format!(
                        "the rebuilt loop is open between its coedges {} and {}",
                        new_coedges[index].id,
                        new_coedges[(index + 1) % new_coedges.len()].id
                    )));
                }
                let face = result
                    .shells
                    .iter_mut()
                    .flat_map(|shell| &mut shell.faces)
                    .find(|face| face.id == face_id)
                    .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during chain surgery"))?;
                face.loops[loop_index].coedges = new_coedges;
            }
            continue;
        }
        // Global window of each segment from the in-segment samples: use
        // the pcurve fit windows (low..high include overshoot; the
        // in-segment span is [low + overshoot, high - overshoot], but for
        // piece assignment the crossing parameters already delimit
        // exactly — assign by nearest segment midpoint.
        let fitted = if side == 0 {
            &rows.pcurves1
        } else {
            &rows.pcurves2
        };
        let segment_mid = |index: usize| -> f64 {
            let (low, high, _) = &fitted[index];
            (low + high) * 0.5
        };
        // Group pieces per face.
        use rustc_hash::FxHashMap as HashMap;
        let mut per_face: HashMap<u64, Vec<usize>> = HashMap::default();
        for (piece_index, piece) in pieces.iter().enumerate() {
            let midpoint = rows.to_global((piece.window[0] + piece.window[1]) * 0.5);
            let mut best = (0usize, f64::INFINITY);
            for (segment_index, _) in segments.iter().enumerate() {
                let distance = (segment_mid(segment_index) - midpoint).abs();
                let wrapped = (segment_mid(segment_index) - midpoint + 1.0).abs();
                let wrapped2 = (segment_mid(segment_index) - midpoint - 1.0).abs();
                let distance = distance.min(wrapped).min(wrapped2);
                if distance < best.1 {
                    best = (segment_index, distance);
                }
            }
            let (face_id, ..) = face_info(&segments[best.0]);
            per_face.entry(face_id).or_default().push(piece_index);
        }
        for (face_id, piece_indices) in per_face {
            // The run of chain coedges in this face's loop.
            let (loop_index, coedge_ids): (usize, Vec<u64>) = {
                let mut loop_index = None;
                let mut ids = Vec::new();
                for segment in segments {
                    let (candidate_face, candidate_loop, coedge_id) = face_info(segment);
                    if candidate_face == face_id {
                        loop_index = Some(candidate_loop);
                        ids.push(coedge_id);
                    }
                }
                (loop_index.ok_or(KernelRefusal::internal(KernelStage::Sew, "chain_coedges", "blend: face lost its chain coedges"))?, ids)
            };
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == face_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during chain surgery"))?;
            let loop_record = &mut face.loops[loop_index];
            let positions: Vec<usize> = loop_record
                .coedges
                .iter()
                .enumerate()
                .filter(|(_, coedge)| coedge_ids.contains(&coedge.id))
                .map(|(position, _)| position)
                .collect();
            if positions.is_empty() {
                return Err(KernelRefusal::internal(KernelStage::Sew, "chain_coedges", "blend: chain coedges missing from mate loop"));
            }
            let old_forward = loop_record.coedges[positions[0]].forward;
            // Sense: the support pieces run in chain direction; whether
            // the loop traverses them forward mirrors the old coedges
            // relative to their segments' chain direction.
            let chain_forward = {
                let segment = segments
                    .iter()
                    .find(|segment| face_info(segment).0 == face_id)
                    .unwrap();
                let (.., coedge_id) = face_info(segment);
                let old = loop_record
                    .coedges
                    .iter()
                    .find(|coedge| coedge.id == coedge_id)
                    .unwrap();
                old.forward == segment.forward
            };
            // Build the replacement coedges: pieces sorted by window in
            // chain direction, oriented to the loop traversal.
            let mut ordered: Vec<&RimPiece> =
                piece_indices.iter().map(|index| &pieces[*index]).collect();
            ordered.sort_by(|a, b| a.window[0].total_cmp(&b.window[0]));
            // Wrap: if the face owns both the first and last pieces (its
            // run spans the seam), rotate so the sequence is contiguous
            // in traversal: [.., last piece, first piece, ..].
            if ordered.len() > 1
                && (ordered[0].window[0] - u_start).abs() < 1e-12
                && (ordered[ordered.len() - 1].window[1] - u_end).abs() < 1e-12
            {
                let rotation = ordered.len() - 1;
                ordered.rotate_left(rotation);
            }
            if !chain_forward {
                ordered.reverse();
            }
            let mut replacements = Vec::new();
            for piece in ordered {
                if let Some(whole) = &rows.whole_pcurves[side] {
                    // Rows-space fit: split directly at the piece window.
                    let [w0, w1] = piece.window;
                    let epsilon = 1e-9;
                    let portion = {
                        let (_, tail) = if w0 > u_start + epsilon {
                            whole.split(w0).or_refuse(KernelStage::Refine, "split")?
                        } else {
                            (whole.clone(), whole.clone())
                        };
                        let tail = if w0 > u_start + epsilon {
                            tail
                        } else {
                            whole.clone()
                        };
                        if w1 < u_end - epsilon {
                            tail.split(w1).or_refuse(KernelStage::Refine, "split")?.0
                        } else {
                            tail
                        }
                    };
                    replacements.push(CoedgeRecord {
                        id: take_id(),
                        edge_id: piece.edge_id,
                        forward: chain_forward,
                        pcurve: if chain_forward {
                            portion
                        } else {
                            portion.reversed().or_refuse(KernelStage::Refine, "reversed")?
                        },
                    });
                    continue;
                }
                let global_window = [
                    rows.to_global(piece.window[0]),
                    rows.to_global(piece.window[1]),
                ];
                let pcurve = pcurve_portion(
                    fitted
                        .iter()
                        .enumerate()
                        .min_by(|(a, _), (b, _)| {
                            let mid = (global_window[0] + global_window[1]) * 0.5;
                            let da = (segment_mid(*a) - mid)
                                .abs()
                                .min((segment_mid(*a) - mid + 1.0).abs())
                                .min((segment_mid(*a) - mid - 1.0).abs());
                            let db = (segment_mid(*b) - mid)
                                .abs()
                                .min((segment_mid(*b) - mid + 1.0).abs())
                                .min((segment_mid(*b) - mid - 1.0).abs());
                            da.total_cmp(&db)
                        })
                        .map(|(_, fitted)| fitted)
                        .unwrap(),
                    global_window[0],
                    global_window[1],
                )?;
                replacements.push(CoedgeRecord {
                    id: take_id(),
                    edge_id: piece.edge_id,
                    forward: chain_forward,
                    pcurve: if chain_forward {
                        pcurve
                    } else {
                        pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
                    },
                });
            }
            let _ = old_forward;
            side_use_forward[side] = chain_forward;
            // Splice: replace the run (positions may be non-contiguous
            // only through the loop's own wrap; handle the contiguous
            // rotation) with the replacement sequence.
            let first_position = positions[0];
            let mut kept: Vec<CoedgeRecord> = Vec::new();
            for (position, coedge) in loop_record.coedges.iter().enumerate() {
                if !positions.contains(&position) {
                    kept.push(coedge.clone());
                } else if position == first_position {
                    for replacement in &replacements {
                        kept.push(replacement.clone());
                    }
                }
            }
            loop_record.coedges = kept;
        }
    }

    // ---- blend face ---------------------------------------------------
    // The blend's use of every support piece must OPPOSE the mate side's
    // use (manifold pairing); the loop winding follows, and same_sense
    // follows the winding (loops run CCW around the face normal).
    let v0_forward = !side_use_forward[0];
    if side_use_forward[1] != v0_forward {
        return Err(KernelRefusal::internal(KernelStage::Sew, "chain_side_orientation", "blend: chain sides have inconsistent orientations"));
    }
    let same_sense = v0_forward;
    let loop_id = take_id();
    let mut coedges = Vec::new();
    if v0_forward {
        for piece in &side_edges[0] {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: piece.edge_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(
                    piece.window[0],
                    0.0,
                    piece.window[1],
                    0.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: blend_seam_id,
            forward: true,
            pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_end, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
        for piece in side_edges[1].iter().rev() {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: piece.edge_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(
                    piece.window[1],
                    1.0,
                    piece.window[0],
                    1.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: blend_seam_id,
            forward: false,
            pcurve: crate::sweep_topology::parameter_line(u_start, 1.0, u_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
    } else {
        // Mirror winding: v=1 side forward, v=0 side reversed.
        for piece in &side_edges[1] {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: piece.edge_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(
                    piece.window[0],
                    1.0,
                    piece.window[1],
                    1.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: blend_seam_id,
            forward: false,
            pcurve: crate::sweep_topology::parameter_line(u_end, 1.0, u_end, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
        for piece in side_edges[0].iter().rev() {
            coedges.push(CoedgeRecord {
                id: take_id(),
                edge_id: piece.edge_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(
                    piece.window[1],
                    0.0,
                    piece.window[0],
                    0.0,
                ).or_refuse(KernelStage::Refine, "parameter_line")?,
            });
        }
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: blend_seam_id,
            forward: true,
            pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_start, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
    }
    let mut loops = vec![LoopRecord {
        id: loop_id,
        coedges,
    }];

    // The CARVED lens: an INNER loop of two coedges on ONE new edge — the
    // crease — both of them this face's own.  That is the slit pattern the
    // torus mate's v-seam already carries, and it is what a crease IS: the two
    // sheets of the wall are one surface, so the curve where they meet has one
    // edge and two coedges rather than two faces' worth.  The lens between them
    // is the part of the swept envelope inside the swept ball volume, and
    // dropping it is what makes the wall the blend.
    if let Some(crease) = rows.crease {
        let start_vertex = take_id();
        let end_vertex = take_id();
        let crease_edge = take_id();
        result.vertices.push(VertexRecord {
            id: start_vertex,
            point: crease.start,
        });
        result.vertices.push(VertexRecord {
            id: end_vertex,
            point: crease.end,
        });
        let [crease_t0, crease_t1] = crease.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        result.edges.push(EdgeRecord {
            id: crease_edge,
            curve: crease.curve,
            t0: crease_t0,
            t1: crease_t1,
            start_vertex_id: start_vertex,
            end_vertex_id: end_vertex,
            degenerate: false,
            name: name.map(|value| format!("{value}:CREASE")),
        });
        // WINDING: an inner loop runs against the outer one.  Measured from the
        // two loops themselves rather than assumed from a convention, so the
        // hole is a hole whichever way the outer loop was wound (this surgery
        // winds it two ways already — see `v0_forward` above).
        let outer = signed_area(&loops[0].coedges)?;
        let forward_first = signed_area(&[
            CoedgeRecord {
                id: 0,
                edge_id: crease_edge,
                forward: true,
                pcurve: crease.first.clone(),
            },
            CoedgeRecord {
                id: 0,
                edge_id: crease_edge,
                forward: false,
                pcurve: crease.second.reversed().or_refuse(KernelStage::Refine, "reversed")?,
            },
        ])?;
        let coedges = if outer * forward_first < 0.0 {
            vec![
                CoedgeRecord {
                    id: take_id(),
                    edge_id: crease_edge,
                    forward: true,
                    pcurve: crease.first,
                },
                CoedgeRecord {
                    id: take_id(),
                    edge_id: crease_edge,
                    forward: false,
                    pcurve: crease.second.reversed().or_refuse(KernelStage::Refine, "reversed")?,
                },
            ]
        } else {
            vec![
                CoedgeRecord {
                    id: take_id(),
                    edge_id: crease_edge,
                    forward: true,
                    pcurve: crease.second,
                },
                CoedgeRecord {
                    id: take_id(),
                    edge_id: crease_edge,
                    forward: false,
                    pcurve: crease.first.reversed().or_refuse(KernelStage::Refine, "reversed")?,
                },
            ]
        };
        loops.push(LoopRecord {
            id: take_id(),
            coedges,
        });
    }

    let blend_face = FaceRecord {
        id: take_id(),
        surface: rows.surface,
        same_sense,
        loops,
        name: name.map(|value| value.to_string()),
    };
    let shell_index = result
        .shells
        .iter()
        .position(|shell| {
            shell
                .faces
                .iter()
                .any(|face| face.id == segments[0].first.face.id)
        })
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during chain surgery"))?;
    result.shells[shell_index].faces.push(blend_face);

    // ---- drop the chain edges and orphaned vertices -------------------
    let chain_edge_ids: Vec<u64> = segments.iter().map(|segment| segment.edge.id).collect();
    result
        .edges
        .retain(|candidate| !chain_edge_ids.contains(&candidate.id));
    let old_vertices: Vec<u64> = (0..segments.len()).map(chain_vertex_at).collect();
    for vertex in old_vertices {
        let still_used = result.edges.iter().any(|candidate| {
            candidate.start_vertex_id == vertex || candidate.end_vertex_id == vertex
        });
        if !still_used {
            result.vertices.retain(|candidate| candidate.id != vertex);
        }
    }
    crossings.gate(result)
}


/// Signed area of a loop in the face's own (u, v), from its pcurves.
///
/// The sign is all that is used: an inner loop must run AGAINST its outer one,
/// and reading both off the same sampler makes that true by measurement rather
/// than by a convention this surgery would otherwise have to restate (it winds
/// the outer loop two different ways already).
pub(in crate::blend) fn signed_area(coedges: &[CoedgeRecord]) -> Result<f64, KernelRefusal> {
    const SAMPLES: usize = 24;
    let mut twice_area = 0.0;
    for coedge in coedges {
        let [t0, t1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut previous: Option<crate::Vec3> = None;
        for index in 0..=SAMPLES {
            let t = t0 + (t1 - t0) * index as f64 / SAMPLES as f64;
            let point = coedge.pcurve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
            if let Some(before) = previous {
                twice_area += before.x * point.y - point.x * before.y;
            }
            previous = Some(point);
        }
    }
    Ok(0.5 * twice_area)
}

/// How one seam-bounded run of a carrier loop is filled.
#[derive(Debug, PartialEq)]
pub(in crate::blend) enum RunWalk {
    /// The pool indices, in loop order, from the run's start to its target.
    Path(Vec<usize>),
    /// Two or more unused pieces leave this vertex at the same uv.
    Ambiguous(u64, Vec<usize>),
    /// No unused piece leaves this vertex at this uv (the target unreached).
    Stuck(u64, [f64; 2]),
}

/// Fill one seam-bounded run by its topology: from `start` (vertex, uv), take
/// at each step the ONE unused entry (from vertex, to vertex, start uv, end uv)
/// whose from-vertex is the current vertex and whose start uv is the current
/// uv under `same`, until the current vertex and uv are `target`'s (after at
/// least one entry). Never a first-found pick: two matches are `Ambiguous`.
/// `used` is updated only for a completed path.
pub(in crate::blend) fn walk_seam_run(
    entries: &[(u64, u64, [f64; 2], [f64; 2])],
    used: &mut [bool],
    start: (u64, [f64; 2]),
    target: (u64, [f64; 2]),
    same: &dyn Fn([f64; 2], [f64; 2]) -> Result<bool, KernelRefusal>,
) -> Result<RunWalk, KernelRefusal> {
    let mut taken = used.to_vec();
    let (mut vertex, mut uv) = start;
    let mut path = Vec::new();
    loop {
        if !path.is_empty() && vertex == target.0 && same(uv, target.1)? {
            used.copy_from_slice(&taken);
            return Ok(RunWalk::Path(path));
        }
        let mut matches = Vec::new();
        for (index, (from, _, start_uv, _)) in entries.iter().enumerate() {
            if !taken[index] && *from == vertex && same(uv, *start_uv)? {
                matches.push(index);
            }
        }
        match matches.as_slice() {
            [one] => {
                taken[*one] = true;
                path.push(*one);
                vertex = entries[*one].1;
                uv = entries[*one].3;
            }
            [] => return Ok(RunWalk::Stuck(vertex, uv)),
            _ => return Ok(RunWalk::Ambiguous(vertex, matches)),
        }
    }
}

/// The first joint (coedge `index` -> `index + 1`, cyclic) of a rebuilt loop
/// that is open, from each coedge's (start vertex, start uv, end vertex, end
/// uv) and whether it is a walked piece. Vertex continuity is required at
/// EVERY joint; a joint touching a walked piece is read [`UvJoint::Strict`],
/// one between two untouched carrier coedges [`UvJoint::ModuloClosedPeriods`].
pub(in crate::blend) fn first_open_joint(
    ends: &[(u64, [f64; 2], u64, [f64; 2])],
    walked: &[bool],
    joint: &dyn Fn([f64; 2], [f64; 2], UvJoint) -> Result<bool, KernelRefusal>,
) -> Result<Option<usize>, KernelRefusal> {
    for index in 0..ends.len() {
        let next = (index + 1) % ends.len();
        let rule = if walked[index] || walked[next] { UvJoint::Strict } else { UvJoint::ModuloClosedPeriods };
        if ends[index].2 != ends[next].0 || !joint(ends[index].3, ends[next].1, rule)? {
            return Ok(Some(index));
        }
    }
    Ok(None)
}

/// Which chart branches two uv readings at one loop joint may sit on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(in crate::blend) enum UvJoint {
    /// Raw uv only: never a jump of half a closed period or more, whatever
    /// the band. Every walk step, and every joint touching a walked piece.
    Strict,
    /// Raw uv up to whole multiples of the surface's ACTUAL closed periods;
    /// a component the surface is not closed in stays raw. Only a joint
    /// between two untouched carrier coedges.
    ModuloClosedPeriods,
}

/// The spatial bar turned into a uv band at a point whose derivative
/// magnitudes are `du`, `dv` — refusing an unreadable derivative or band
/// BEFORE `surface_uv_tolerance`'s min/max (`f64::min` would drop a NaN and
/// return the other derivative's band).
pub(in crate::blend) fn checked_uv_band(spatial: f64, du: f64, dv: f64) -> Result<f64, String> {
    if !(spatial.is_finite() && du.is_finite() && dv.is_finite()) {
        return Err(format!("derivative magnitudes ({du}, {dv}) at spatial bar {spatial}"));
    }
    let band = crate::tolerance::surface_uv_tolerance(spatial, du, dv);
    if !(band.is_finite() && band >= 0.0) {
        return Err(format!("uv band {band}"));
    }
    Ok(band)
}

/// Chart-branch continuity of `a` -> `b` at one loop joint, separate from the
/// spatial distance `band` (the spatial bar as uv at `a`). Every input must be
/// finite (a period also positive) or the reading is refused, never compared.
/// Under [`UvJoint::Strict`] a component the surface is closed in with a jump
/// of half its period or more is discontinuous whatever `band` says: at a
/// pole the derivative floor makes the band span whole periods, and a large
/// band must never authorise a wrap.
pub(in crate::blend) fn uv_joint_continuous(
    a: [f64; 2],
    b: [f64; 2],
    band: f64,
    periods: [Option<f64>; 2],
    rule: UvJoint,
) -> Result<bool, String> {
    if !(a.iter().chain(&b).all(|value| value.is_finite()) && band.is_finite() && band >= 0.0) {
        return Err(format!("uv ({}, {}) -> ({}, {}) at band {band}", a[0], a[1], b[0], b[1]));
    }
    if let Some(period) = periods.iter().flatten().find(|period| !(period.is_finite() && **period > 0.0)) {
        return Err(format!("closed period {period}"));
    }
    let mut continuous = true;
    for direction in 0..2 {
        let mut jump = b[direction] - a[direction];
        if let Some(period) = periods[direction] {
            let turns = (jump / period).round();
            match rule {
                UvJoint::Strict if turns != 0.0 => continuous = false,
                UvJoint::Strict => {}
                UvJoint::ModuloClosedPeriods => jump -= turns * period,
            }
        }
        if !(jump.abs() <= band) {
            continuous = false;
        }
    }
    Ok(continuous)
}

