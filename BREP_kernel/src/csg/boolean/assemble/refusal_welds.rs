use crate::KernelRefusal;
use super::*;
use super::edge_conform::edge_lies_on;

/// Last-resort weld for a CLOSED assembly still carrying one-use edges: two
/// geometrically IDENTICAL one-use edges (same span, mutually coincident within
/// tolerance) are one shared boundary that fragmentation duplicated instead of
/// welding — an FP-sensitive miss in `Assembler::edge` (observed only under the
/// wasm build when unioning an extruded partial-revolve SECTOR face back onto
/// its revolve, where a residual off-axis apex leaves near-coincident arc
/// slivers). `conform_overlapping_one_use_edges` only handles PARTIAL overlaps
/// (it splits at a partner's interior vertex); identical spans have no interior
/// split point and slip through. Redirect every coedge of the duplicate onto
/// the survivor with matching orientation so the edge becomes two-use and the
/// shell closes; the now-unused duplicate is dropped by the later
/// used-edge retain. A one-use edge only exists in an ALREADY-BROKEN closed
/// assembly, so this cannot alter a well-formed result, and `validate()` still
/// guards the output.
pub(super) fn weld_identical_coincident_one_use_edges(
    assembler: &mut Assembler,
    faces: &mut [FaceRecord],
) -> Result<bool, KernelRefusal> {
    let mut use_counts = HashMap::<u64, usize>::default();
    for coedge in faces
        .iter()
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
    {
        *use_counts.entry(coedge.edge_id).or_default() += 1;
    }
    let vertex_map = assembler
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    let one_use = assembler
        .edges
        .iter()
        .filter(|edge| use_counts.get(&edge.id) == Some(&1) && !edge.degenerate)
        .cloned()
        .collect::<Vec<_>>();
    if one_use.len() < 2 {
        return Ok(false);
    }
    let tolerance = assembler.tolerance.max(1e-3);
    // duplicate edge id -> (survivor id, flip coedge orientation)
    let mut redirect = HashMap::<u64, (u64, bool)>::default();
    // Sharing an edge also identifies its two endpoint pairs. Leaving the
    // duplicate's neighbours on separate vertices opens their loops even when
    // every edge has two coedges. Plan the endpoint identifications atomically,
    // rejecting any plan that would collapse an existing non-degenerate edge.
    let mut vertex_redirect = HashMap::<u64, u64>::default();
    fn root(redirect: &HashMap<u64, u64>, mut id: u64) -> u64 {
        while let Some(&next) = redirect.get(&id) {
            id = next;
        }
        id
    }
    for (index, survivor) in one_use.iter().enumerate() {
        if redirect.contains_key(&survivor.id) {
            continue;
        }
        let survivor_start = vertex_map[&survivor.start_vertex_id];
        let survivor_end = vertex_map[&survivor.end_vertex_id];
        for duplicate in one_use.iter().skip(index + 1) {
            if redirect.contains_key(&duplicate.id) {
                continue;
            }
            let duplicate_start = vertex_map[&duplicate.start_vertex_id];
            let duplicate_end = vertex_map[&duplicate.end_vertex_id];
            let aligned_error = survivor_start.sub(duplicate_start).length()
                .max(survivor_end.sub(duplicate_end).length());
            let reversed_error = survivor_start.sub(duplicate_end).length()
                .max(survivor_end.sub(duplicate_start).length());
            let aligned = aligned_error <= tolerance;
            let reversed = reversed_error <= tolerance;
            if !aligned && !reversed {
                continue;
            }
            // Endpoint coincidence alone can pair a chord with a bulged arc:
            // require the whole spans to lie on each other before welding.
            let coincident_span = (edge_lies_on(survivor, duplicate, tolerance)?
                && edge_lies_on(duplicate, survivor, tolerance)?)
                // Endpoint coincidence is already established by `aligned`/
                // `reversed`. When a duplicate's CURVE endpoint drifts a whisker
                // off its own vertex (an imported SSI endpoint a couple 1e-4
                // short — the very corner that also broke the face-111 cut), its
                // 0.0/1.0 samples project just OUTSIDE the survivor's [0,1] span,
                // so the full-span `edge_lies_on` spuriously fails one direction
                // on a fraction-range technicality while the spans genuinely
                // coincide. Fall back to an INTERIOR-only span check (0.25/0.5/
                // 0.75) for a pair whose endpoints already match. Additive: only
                // rescues welds the strict test rejects, and a chord-vs-bulged-
                // arc still fails because the interior sagitta exceeds tolerance.
                || (edge_interior_lies_on(survivor, duplicate, tolerance)?
                    && edge_interior_lies_on(duplicate, survivor, tolerance)?);
            if !coincident_span {
                continue;
            }
            // A short edge can fit BOTH endpoint correspondences inside the
            // repair band. The band admits the weld; the closer correspondence
            // determines its direction. Otherwise an aligned micro-edge is
            // reversed merely because its length is smaller than the band.
            let flip = reversed && (!aligned || reversed_error <= aligned_error);
            let mut proposed = vertex_redirect.clone();
            let pairs = if flip {
                [(duplicate.start_vertex_id, survivor.end_vertex_id),
                 (duplicate.end_vertex_id, survivor.start_vertex_id)]
            } else {
                [(duplicate.start_vertex_id, survivor.start_vertex_id),
                 (duplicate.end_vertex_id, survivor.end_vertex_id)]
            };
            for (from, to) in pairs {
                let (from, to) = (root(&proposed, from), root(&proposed, to));
                if from != to {
                    proposed.insert(from, to);
                }
            }
            if proposed.keys().any(|id|
                vertex_map[id].sub(vertex_map[&root(&proposed, *id)]).length() > tolerance
            ) || assembler.edges.iter().any(|edge| {
                !edge.degenerate && edge.start_vertex_id != edge.end_vertex_id
                    && root(&proposed, edge.start_vertex_id)
                        == root(&proposed, edge.end_vertex_id)
            }) {
                continue;
            }
            vertex_redirect = proposed;
            redirect.insert(duplicate.id, (survivor.id, flip));
        }
    }
    if redirect.is_empty() {
        return Ok(false);
    }
    for edge in &mut assembler.edges {
        let start = root(&vertex_redirect, edge.start_vertex_id);
        let end = root(&vertex_redirect, edge.end_vertex_id);
        if start == edge.start_vertex_id && end == edge.end_vertex_id {
            continue;
        }
        let curve = super::builder::snap_edge_curve_endpoints(
            edge.curve.clone(), edge.t0, edge.t1,
            vertex_map[&start], vertex_map[&end],
        )?;
        let [t0, t1] = curve.domain().map_err(|error|
            crate::KernelRefusal::internal(crate::KernelStage::Sew,
                "assemble.refusal_welds", error.to_string()))?;
        edge.curve = curve;
        edge.t0 = t0;
        edge.t1 = t1;
        edge.start_vertex_id = start;
        edge.end_vertex_id = end;
    }
    for face in faces.iter_mut() {
        for loop_record in &mut face.loops {
            for coedge in &mut loop_record.coedges {
                if let Some(&(survivor_id, flip)) = redirect.get(&coedge.edge_id) {
                    coedge.edge_id = survivor_id;
                    if flip {
                        coedge.forward = !coedge.forward;
                    }
                }
            }
        }
    }
    Ok(true)
}

/// Weld near-duplicate vertices left by tangency-degraded SSI endpoints (see
/// call site).  Greedy nearest-first over pairs closer than a √tol-scaled band,
/// skipping pairs already joined by a non-degenerate edge (welding those would
/// collapse the edge).  The survivor is the vertex its incident curve endpoints
/// agree with best (the operand's exact vertex, not the marched seam end); the
/// loser's edges are re-pointed and any curve endpoint that no longer lands on
/// the surviving vertex is clamped onto it (only when the edge parameter is the
/// curve's own domain end, so trimmed sub-curves are never distorted).  Returns
/// the number of welds performed.  One weld per Euler deficit unit is enough,
/// but the caller re-checks the genus, so welding stops via the pair list.
pub(super) fn weld_tangency_duplicate_vertices(
    vertices: &mut Vec<VertexRecord>,
    edges: &mut [EdgeRecord],
    tolerance: f64,
    limit: usize,
) -> usize {
    let scale = vertices
        .iter()
        .map(|v| v.point.length())
        .fold(1.0_f64, f64::max);
    // Tangent-order marching drift is O(√tol·scale); 10× headroom, floored at
    // the model tolerance itself.
    let band = (tolerance.max(1e-12).sqrt() * scale * 10.0).max(tolerance * 10.0);

    let mut connected = rustc_hash::FxHashSet::<(u64, u64)>::default();
    for edge in edges.iter().filter(|edge| !edge.degenerate) {
        let (a, b) = (edge.start_vertex_id, edge.end_vertex_id);
        connected.insert((a.min(b), a.max(b)));
    }

    let mut pairs: Vec<(f64, u64, u64)> = Vec::new();
    for (i, a) in vertices.iter().enumerate() {
        for b in vertices.iter().skip(i + 1) {
            let distance = a.point.sub(b.point).length();
            if distance < band && !connected.contains(&(a.id.min(b.id), a.id.max(b.id))) {
                pairs.push((distance, a.id, b.id));
            }
        }
    }
    pairs.sort_by(|x, y| x.0.partial_cmp(&y.0).expect("finite distances"));

    // How well a vertex's incident curve endpoints agree with its position —
    // the exact operand vertex scores ~0, the drifted seam endpoint ~O(band).
    let endpoint_error = |vertices: &[VertexRecord], edges: &[EdgeRecord], id: u64| -> f64 {
        let point = match vertices.iter().find(|v| v.id == id) {
            Some(v) => v.point,
            None => return f64::INFINITY,
        };
        let mut worst = 0.0_f64;
        for edge in edges.iter().filter(|edge| !edge.degenerate) {
            if edge.start_vertex_id == id {
                if let Ok(p) = edge.curve.evaluate(edge.t0) {
                    worst = worst.max(p.sub(point).length());
                }
            }
            if edge.end_vertex_id == id {
                if let Ok(p) = edge.curve.evaluate(edge.t1) {
                    worst = worst.max(p.sub(point).length());
                }
            }
        }
        worst
    };

    let mut welds = 0usize;
    for (_, a, b) in pairs {
        // `limit` caps welds per call so the caller can DEFICIT-GATE: weld one
        // pair, recompute the genus, and stop the instant it is integral. The
        // greedy pair list contains more merges than the Euler deficit requires
        // (every duplicate vertex, not just the deficit count), so welding it to
        // exhaustion OVERSHOOTS an integral state back to non-integral (observed:
        // deficit -1, two duplicates, welding both lands at +1). Stopping at the
        // deficit keeps the genuine tangent-seam merges without inventing a
        // spurious pinch from a coincidence the assembly did not need closed.
        if welds >= limit {
            break;
        }
        if !vertices.iter().any(|v| v.id == a) || !vertices.iter().any(|v| v.id == b) {
            continue; // already consumed by an earlier weld
        }
        let (survivor, loser) =
            if endpoint_error(vertices, edges, a) <= endpoint_error(vertices, edges, b) {
                (a, b)
            } else {
                (b, a)
            };
        weld_vertex_onto(vertices, edges, loser, survivor, band);
        welds += 1;
    }
    welds
}

/// Re-point every edge endpoint from `loser` to `survivor`, clamp curve
/// domain-end endpoints that drifted (by less than `band`) onto the surviving
/// vertex, and drop the losing vertex record.  Shared by the tangency
/// duplicate-vertex weld and the edge-conformance merge lane.
fn weld_vertex_onto(
    vertices: &mut Vec<VertexRecord>,
    edges: &mut [EdgeRecord],
    loser: u64,
    survivor: u64,
    band: f64,
) {
    let Some(target) = vertices.iter().find(|v| v.id == survivor).map(|v| v.point) else {
        return;
    };
    for edge in edges.iter_mut() {
        if edge.start_vertex_id == loser {
            edge.start_vertex_id = survivor;
        }
        if edge.end_vertex_id == loser {
            edge.end_vertex_id = survivor;
        }
        if edge.degenerate {
            continue;
        }
        // Clamp drifted curve endpoints onto the surviving vertex.
        if edge.start_vertex_id == survivor || edge.end_vertex_id == survivor {
            if let Ok([d0, d1]) = edge.curve.domain() {
                if edge.start_vertex_id == survivor && (edge.t0 - d0).abs() < 1e-12 {
                    if let Ok(p) = edge.curve.evaluate(edge.t0) {
                        let gap = p.sub(target).length();
                        if gap > 0.0 && gap < band {
                            let w = edge.curve.control_points[0].w;
                            edge.curve.control_points[0] = crate::Vec4 {
                                x: target.x * w,
                                y: target.y * w,
                                z: target.z * w,
                                w,
                            };
                        }
                    }
                }
                if edge.end_vertex_id == survivor && (edge.t1 - d1).abs() < 1e-12 {
                    if let Ok(p) = edge.curve.evaluate(edge.t1) {
                        let gap = p.sub(target).length();
                        if gap > 0.0 && gap < band {
                            let last = edge.curve.control_points.len() - 1;
                            let w = edge.curve.control_points[last].w;
                            edge.curve.control_points[last] = crate::Vec4 {
                                x: target.x * w,
                                y: target.y * w,
                                z: target.z * w,
                                w,
                            };
                        }
                    }
                }
            }
        }
    }
    vertices.retain(|v| v.id != loser);
}

/// EDGE-conformance repair for an otherwise-refused CLOSED assembly — the
/// edge-level sibling of `weld_tangency_duplicate_vertices`.  Tangency-degraded
/// SSI endpoints converge only O(√tol·scale) from the operand's exact station
/// (measured 1.568e-4 on the scale-14 extruded-boss fixture at tol 1e-7, where
/// √tol·scale ≈ 4.4e-3), so nearby face boundaries can follow the same locus
/// within that band yet be emitted as DIFFERENT edges/trims that the assembler
/// weld (exact-band) cannot unify.  Two deterministic lanes, each verified
/// geometrically before any mutation:
///
/// 1. MERGE — two one-use edges from different loops whose spans mutually lie
///    within the band and whose endpoints correspond pairwise are one boundary
///    emitted twice.  The lower (degree, id) record survives (operand
///    analytics beat marched SSI fits); the loser's coedge is redirected onto
///    it with a pcurve rebuilt on its own face's surface, and the loser's
///    endpoint vertices are welded onto the survivor's.
/// 2. BRIDGE — a loop that is OPEN between vertices va→vb whose gap is exactly
///    spanned by a one-use edge of ANOTHER loop (the hairline sliver the
///    neighbour's arrangement split created but this face's did not) gets a
///    conforming coedge of that edge inserted with the opposite sense and a
///    pcurve built on the gap face's surface.
///
/// Runs ONLY from refusal paths, so it can never alter a succeeding boolean;
/// the caller re-runs the Euler check and the final full `validate()` still
/// gates the result.  Returns the number of repairs applied.
pub(in crate::boolean) fn conform_unmatched_one_use_edges(
    vertices: &mut Vec<VertexRecord>,
    edges: &mut Vec<EdgeRecord>,
    shells: &mut [ShellRecord],
    tolerance: f64,
) -> Result<usize, KernelRefusal> {
    let scale = vertices
        .iter()
        .map(|v| v.point.length())
        .fold(1.0_f64, f64::max);
    // SSI tangency drift is O(√tol·scale); no headroom multiplier (the band is
    // already ~28× the measured drift), floored at 10× the model tolerance and
    // therefore far below any feature size the arrangement can represent.
    let band = (tolerance.max(1e-12).sqrt() * scale).max(tolerance * 10.0);
    let fit_tolerance = tolerance.max(band / 10.0);
    let pcurve_limit = KernelTolerances::for_scale(scale, tolerance).pcurve_consistency;
    let debug = std::env::var("BREP_DEBUG_BOOL").is_ok();

    // edge id -> every (shell, face, loop, coedge, forward) use.
    fn collect_uses(
        shells: &[ShellRecord],
    ) -> HashMap<u64, Vec<(usize, usize, usize, usize, bool)>> {
        let mut uses = HashMap::<u64, Vec<(usize, usize, usize, usize, bool)>>::default();
        for (shell_index, shell) in shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                for (loop_index, loop_record) in face.loops.iter().enumerate() {
                    for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                        uses.entry(coedge.edge_id).or_default().push((
                            shell_index,
                            face_index,
                            loop_index,
                            coedge_index,
                            coedge.forward,
                        ));
                    }
                }
            }
        }
        uses
    }

    let mut repairs = 0usize;

    // LANE A: merge duplicated hairline one-use pairs.
    {
        let uses = collect_uses(shells);
        let vertex_point = vertices
            .iter()
            .map(|v| (v.id, v.point))
            .collect::<HashMap<_, _>>();
        let mut one_use = edges
            .iter()
            .filter(|edge| !edge.degenerate && uses.get(&edge.id).map(Vec::len) == Some(1))
            .map(|edge| edge.id)
            .collect::<Vec<_>>();
        one_use.sort_unstable();
        let mut consumed = HashSet::<u64>::default();
        // (survivor id, loser id, reversed correspondence)
        let mut merges = Vec::<(u64, u64, bool)>::new();
        for (index, &first_id) in one_use.iter().enumerate() {
            if consumed.contains(&first_id) {
                continue;
            }
            for &second_id in &one_use[index + 1..] {
                if consumed.contains(&second_id) {
                    continue;
                }
                let first = edges.iter().find(|edge| edge.id == first_id).unwrap();
                let second = edges.iter().find(|edge| edge.id == second_id).unwrap();
                // Never merge two edges of the same face loop.
                let first_use = uses[&first_id][0];
                let second_use = uses[&second_id][0];
                if (first_use.0, first_use.1, first_use.2)
                    == (second_use.0, second_use.1, second_use.2)
                {
                    continue;
                }
                let (Some(&fs), Some(&fe), Some(&ss), Some(&se)) = (
                    vertex_point.get(&first.start_vertex_id),
                    vertex_point.get(&first.end_vertex_id),
                    vertex_point.get(&second.start_vertex_id),
                    vertex_point.get(&second.end_vertex_id),
                ) else {
                    continue;
                };
                let aligned = fs.sub(ss).length() <= band && fe.sub(se).length() <= band;
                let reversed = fs.sub(se).length() <= band && fe.sub(ss).length() <= band;
                if !aligned && !reversed {
                    continue;
                }
                if !(edge_lies_on(first, second, band)? && edge_lies_on(second, first, band)?) {
                    continue;
                }
                let first_wins = (first.curve.degree, first.id) <= (second.curve.degree, second.id);
                let (survivor, loser) = if first_wins {
                    (first_id, second_id)
                } else {
                    (second_id, first_id)
                };
                merges.push((survivor, loser, !aligned));
                consumed.insert(first_id);
                consumed.insert(second_id);
                break;
            }
        }
        for (survivor_id, loser_id, reversed) in merges {
            let survivor = edges
                .iter()
                .find(|edge| edge.id == survivor_id)
                .unwrap()
                .clone();
            let loser = edges
                .iter()
                .find(|edge| edge.id == loser_id)
                .unwrap()
                .clone();
            if !(survivor.t0.is_finite() && survivor.t1.is_finite() && survivor.t0 < survivor.t1) {
                continue;
            }
            // Evaluate both endpoint welds against the current edge table,
            // including changes from earlier successful merges. Close rims
            // cannot authorize erasing an existing material-thickness edge.
            let survivor_pair = if reversed {
                [survivor.end_vertex_id, survivor.start_vertex_id]
            } else {
                [survivor.start_vertex_id, survivor.end_vertex_id]
            };
            let endpoint_pairs = [loser.start_vertex_id, loser.end_vertex_id]
                .into_iter().zip(survivor_pair).collect::<Vec<_>>();
            let redirected = |mut id| {
                for &(from, to) in &endpoint_pairs {
                    if id == from { id = to; }
                }
                id
            };
            if edges.iter().any(|edge| !edge.degenerate
                && edge.start_vertex_id != edge.end_vertex_id
                && redirected(edge.start_vertex_id) == redirected(edge.end_vertex_id)) {
                if debug {
                    eprintln!("conform: merge {loser_id}->{survivor_id} rejected (collapses an existing edge)");
                }
                continue;
            }
            let (shell_index, face_index, loop_index, coedge_index, old_forward) =
                uses[&loser_id][0];
            let new_forward = if reversed { !old_forward } else { old_forward };
            let face = &mut shells[shell_index].faces[face_index];
            let Ok(pcurve) = build_pcurve_on_surface_range(
                &face.surface,
                &survivor.curve,
                survivor.t0,
                survivor.t1,
                new_forward,
                fit_tolerance,
            ) else {
                continue;
            };
            let Ok(error) = adaptive_coedge_error(
                &face.surface,
                &pcurve,
                &survivor.curve,
                &survivor,
                new_forward,
                pcurve_limit,
            ) else {
                continue;
            };
            if error > pcurve_limit {
                if debug {
                    eprintln!(
                        "conform: merge {loser_id}->{survivor_id} rejected (pcurve error {error:.2e} > {pcurve_limit:.2e})"
                    );
                }
                continue;
            }
            {
                let coedge = &mut face.loops[loop_index].coedges[coedge_index];
                coedge.edge_id = survivor_id;
                coedge.forward = new_forward;
                coedge.pcurve = pcurve;
            }
            edges.retain(|edge| edge.id != loser_id);
            for (loser_vertex, survivor_vertex) in [loser.start_vertex_id, loser.end_vertex_id]
                .into_iter()
                .zip(survivor_pair)
            {
                if loser_vertex != survivor_vertex {
                    weld_vertex_onto(vertices, edges, loser_vertex, survivor_vertex, band);
                }
            }
            if debug {
                eprintln!(
                    "conform: merged duplicate one-use edge {loser_id} onto {survivor_id} (reversed={reversed})"
                );
            }
            repairs += 1;
        }
    }

    // LANE B: bridge open loop gaps with an existing one-use edge.
    {
        let uses = collect_uses(shells);
        let edge_by_id = edges
            .iter()
            .map(|edge| (edge.id, edge.clone()))
            .collect::<HashMap<_, _>>();
        let mut sorted_edge_ids = edges
            .iter()
            .filter(|edge| !edge.degenerate)
            .map(|edge| edge.id)
            .collect::<Vec<_>>();
        sorted_edge_ids.sort_unstable();
        let mut next_coedge_id = shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.id)
            .max()
            .unwrap_or(0)
            + 1;
        let mut consumed = HashSet::<u64>::default();
        // (shell, face, loop, insert position, coedge)
        let mut plans = Vec::<(usize, usize, usize, usize, CoedgeRecord)>::new();
        for (shell_index, shell) in shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                for (loop_index, loop_record) in face.loops.iter().enumerate() {
                    let count = loop_record.coedges.len();
                    if count == 0 {
                        continue;
                    }
                    for position in 0..count {
                        let current = &loop_record.coedges[position];
                        let next = &loop_record.coedges[(position + 1) % count];
                        let (Some(current_edge), Some(next_edge)) = (
                            edge_by_id.get(&current.edge_id),
                            edge_by_id.get(&next.edge_id),
                        ) else {
                            continue;
                        };
                        let gap_start = if current.forward {
                            current_edge.end_vertex_id
                        } else {
                            current_edge.start_vertex_id
                        };
                        let gap_end = if next.forward {
                            next_edge.start_vertex_id
                        } else {
                            next_edge.end_vertex_id
                        };
                        if gap_start == gap_end {
                            continue;
                        }
                        for &candidate_id in &sorted_edge_ids {
                            if consumed.contains(&candidate_id) {
                                continue;
                            }
                            let Some(edge_uses) = uses.get(&candidate_id) else {
                                continue;
                            };
                            if edge_uses.len() != 1 {
                                continue;
                            }
                            let (use_shell, use_face, use_loop, _, use_forward) = edge_uses[0];
                            if (use_shell, use_face, use_loop)
                                == (shell_index, face_index, loop_index)
                            {
                                continue;
                            }
                            let edge = &edge_by_id[&candidate_id];
                            let forward = if edge.start_vertex_id == gap_start
                                && edge.end_vertex_id == gap_end
                            {
                                true
                            } else if edge.end_vertex_id == gap_start
                                && edge.start_vertex_id == gap_end
                            {
                                false
                            } else {
                                continue;
                            };
                            // A manifold edge is traversed once in each sense.
                            if use_forward == forward {
                                continue;
                            }
                            if !(edge.t0.is_finite() && edge.t1.is_finite() && edge.t0 < edge.t1) {
                                continue;
                            }
                            let Ok(pcurve) = build_pcurve_on_surface_range(
                                &face.surface,
                                &edge.curve,
                                edge.t0,
                                edge.t1,
                                forward,
                                fit_tolerance,
                            ) else {
                                continue;
                            };
                            let Ok(error) = adaptive_coedge_error(
                                &face.surface,
                                &pcurve,
                                &edge.curve,
                                edge,
                                forward,
                                pcurve_limit,
                            ) else {
                                continue;
                            };
                            if error > pcurve_limit {
                                if debug {
                                    eprintln!(
                                        "conform: bridge of edge {candidate_id} into face {} rejected (pcurve error {error:.2e} > {pcurve_limit:.2e})",
                                        face.id
                                    );
                                }
                                continue;
                            }
                            plans.push((
                                shell_index,
                                face_index,
                                loop_index,
                                position + 1,
                                CoedgeRecord {
                                    id: next_coedge_id,
                                    edge_id: candidate_id,
                                    forward,
                                    pcurve,
                                },
                            ));
                            next_coedge_id += 1;
                            consumed.insert(candidate_id);
                            if debug {
                                eprintln!(
                                    "conform: bridging open gap in face {} loop {} with one-use edge {candidate_id} (forward={forward})",
                                    face.id, loop_index
                                );
                            }
                            break;
                        }
                    }
                }
            }
        }
        // Apply per loop in descending position so earlier insertions do not
        // shift later ones.
        plans.sort_by(|a, b| (a.0, a.1, a.2, b.3).cmp(&(b.0, b.1, b.2, a.3)));
        for (shell_index, face_index, loop_index, position, coedge) in plans {
            shells[shell_index].faces[face_index].loops[loop_index]
                .coedges
                .insert(position, coedge);
            repairs += 1;
        }
    }

    Ok(repairs)
}

/// Recompute the derived genus with the same shell-aware Euler formula
/// `validate()` checks against (left untouched when the counts do not give an
/// integral genus — validation then reports the real defect).
pub(super) fn refresh_derived_genus(solid: &mut BrepSolid) {
    let hole_count = solid.bounding_hole_count();
    let face_count: i64 = solid
        .shells
        .iter()
        .map(|shell| shell.faces.len() as i64)
        .sum();
    let edge_count = solid.edges.iter().filter(|edge| !edge.degenerate).count() as i64;
    let live_vertices = solid
        .edges
        .iter()
        .filter(|edge| !edge.degenerate)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect::<HashSet<_>>();
    let vertex_count = solid
        .vertices
        .iter()
        .filter(|vertex| live_vertices.contains(&vertex.id))
        .count() as i64;
    let euler = vertex_count - edge_count + face_count - hole_count;
    let numerator = solid.shells.len() as i64 * 2 - euler;
    if numerator >= 0 && numerator % 2 == 0 {
        solid.genus = numerator / 2;
    }
}

