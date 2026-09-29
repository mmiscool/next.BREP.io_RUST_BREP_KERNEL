use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Rebuild a piece's pcurve on `surface`, PRESERVING the period branch of the
/// pcurve it replaces. `build_pcurve_on_surface` projects every sample onto
/// the surface's canonical branch; on a closed surface that silently moves a
/// seam-adjacent pcurve by a whole period, tearing it off the face's
/// unwrapped boundary band in the fragment arrangement — the cut chain then
/// cannot anchor and the face never splits (helmet Face_15: the truncation
/// bridge rebuilt onto u∈[0.822,1] while the stub chain ends at u=0). Shift
/// the rebuilt pcurve by the integer period multiple per closed direction
/// that lands it nearest the old one; the shift is 0 whenever the rebuild
/// stayed on the old branch, so every previously-working case is
/// bit-identical. Escape hatch `BREP_PCURVE_BRANCH_ALIGN=0`.
fn rebuild_pcurve_preserving_branch(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    previous: &NurbsCurve,
) -> Result<NurbsCurve, KernelRefusal> {
    // Marched builder (see build_pcurve_on_surface_marched): the mint uses it so
    // a section riding a compressed carrier row does not fold; the canonicalize
    // rebuild must use it too or the fold returns. Fail-soft to the global build.
    let mut rebuilt = build_pcurve_on_surface_marched(surface, curve).or_refuse(KernelStage::Intersect, "build_pcurve_on_surface_marched")?;
    // NOTE (2026-09-16): this hatch's ONLY tamper guard,
    // `csg/fragment/tests.rs::seam_band_cut_chain_splits_periodic_face`, went
    // INERT when the march-window lift landed — that test now passes with this
    // hatch off, so nothing tamper-verifies this mechanism any more. Not that it
    // is untested: no other test NAMES this hatch.
    if std::env::var("BREP_PCURVE_BRANCH_ALIGN").as_deref() == Ok("0") {
        return Ok(rebuilt);
    }
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Intersect, "closed_directions")?;
    if !closed_u && !closed_v {
        return Ok(rebuilt);
    }
    let mean = |pcurve: &NurbsCurve| -> Option<(f64, f64)> {
        let mut sum_u = 0.0;
        let mut sum_v = 0.0;
        let mut count = 0.0;
        for point in &pcurve.control_points {
            if point.w.abs() > 1e-300 {
                sum_u += point.x / point.w;
                sum_v += point.y / point.w;
                count += 1.0;
            }
        }
        (count > 0.0).then(|| (sum_u / count, sum_v / count))
    };
    let (Some((old_u, old_v)), Some((new_u, new_v))) = (mean(previous), mean(&rebuilt)) else {
        return Ok(rebuilt);
    };
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Intersect, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Intersect, "domain_v")?;
    let shift_u = if closed_u && u1 > u0 {
        ((old_u - new_u) / (u1 - u0)).round()
    } else {
        0.0
    };
    let shift_v = if closed_v && v1 > v0 {
        ((old_v - new_v) / (v1 - v0)).round()
    } else {
        0.0
    };
    if shift_u == 0.0 && shift_v == 0.0 {
        return Ok(rebuilt);
    }
    for point in &mut rebuilt.control_points {
        point.x += shift_u * (u1 - u0) * point.w;
        point.y += shift_v * (v1 - v0) * point.w;
    }
    Ok(rebuilt)
}

pub(super) fn canonicalize_imprint_junctions(
    result: &mut ImprintResultRecord,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    let face_map = faces(solid_a, 0)
        .into_iter()
        .chain(faces(solid_b, 1))
        .map(|face| (face.key(), face.face))
        .collect::<HashMap<_, _>>();
    let mut incident_faces = HashMap::<u64, HashSet<FaceKey>>::default();
    let mut incident_points = HashMap::<u64, Vec<Vec3>>::default();
    for piece in &result.pieces {
        let [start, end] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        for (vertex_id, point) in [
            (piece.start_vertex_id, piece.curve.evaluate(start).or_refuse(KernelStage::Intersect, "evaluate")?),
            (piece.end_vertex_id, piece.curve.evaluate(end).or_refuse(KernelStage::Intersect, "evaluate")?),
        ] {
            let entry = incident_faces.entry(vertex_id).or_default();
            entry.extend(piece.support_faces);
            incident_points.entry(vertex_id).or_default().push(point);
        }
    }
    // Residual-derived coincident-vertex merge (OCCT's ComputeTolReached3d idea).
    // Two imprint junctions that are the SAME intersection point — proven by the
    // whole segment between them lying on the UNION of their incident surfaces
    // within the weld band — collapse to ONE vertex here, at the imprint, with
    // full incidence available. This fixes the near-tangent triple-point
    // double-mint: two marches meeting at one grazing junction disagree along the
    // ill-conditioned tangent direction and mint two vertices a fixed radius
    // apart, which the geometry-only assembler then cannot weld (they exceed the
    // weld radius) → one-use edges / non-integral genus. The merge is gated so it
    // fires ONLY when the residual evidence proves one point (see
    // `merge_residual_coincident_vertices`). Escape hatch:
    // `BREP_IMPRINT_RESIDUAL_MERGE=0` restores the pre-merge behaviour.
    let raw_extent = raw_solid_extent(solid_a).max(raw_solid_extent(solid_b));
    if std::env::var("BREP_IMPRINT_RESIDUAL_MERGE").as_deref() != Ok("0")
        && merge_residual_coincident_vertices(
            result,
            &face_map,
            charts,
            &incident_faces,
            tolerance,
            raw_extent,
        )?
    {
        // Incidence changed under the merge — rebuild it for the polish below.
        incident_faces.clear();
        incident_points.clear();
        for piece in &result.pieces {
            let [start, end] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
            for (vertex_id, point) in [
                (piece.start_vertex_id, piece.curve.evaluate(start).or_refuse(KernelStage::Intersect, "evaluate")?),
                (piece.end_vertex_id, piece.curve.evaluate(end).or_refuse(KernelStage::Intersect, "evaluate")?),
            ] {
                incident_faces
                    .entry(vertex_id)
                    .or_default()
                    .extend(piece.support_faces);
                incident_points.entry(vertex_id).or_default().push(point);
            }
        }
    }
    let mut canonical_vertices = result
        .vertices
        .iter()
        .filter(|vertex| {
            let limit = tolerance * 10.0 * (1.0 + vertex.point.length());
            incident_points.get(&vertex.id).is_some_and(|points| {
                points
                    .iter()
                    .all(|point| point.sub(vertex.point).length() <= limit)
            })
        })
        .map(|vertex| vertex.id)
        .collect::<HashSet<_>>();
    
    for vertex in &mut result.vertices {
        if !canonical_vertices.contains(&vertex.id) {
            continue;
        }
        let Some(keys) = incident_faces.get(&vertex.id) else {
            continue;
        };
        let surfaces = keys
            .iter()
            .filter_map(|key| face_map.get(key).map(|face| &face.surface))
            .collect::<Vec<_>>();
        if surfaces.len() < 2 {
            continue;
        }
        let residual = |point: Vec3| -> Result<f64, KernelRefusal> {
            surfaces
                .iter()
                .map(|surface| Ok(project_point_to_surface(surface, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance))
                .try_fold(0.0f64, |maximum, distance| {
                    distance.map(|distance| maximum.max(distance))
                })
        };
        let mut current = vertex.point;
        let mut best = current;
        let mut best_residual = residual(current)?;
        for _ in 0..60 {
            let projected = surfaces
                .iter()
                .map(|surface| project_point_to_surface(surface, current).map(|value| value.point))
                .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Intersect, "csg.imprint.junctions")?;
            let candidate = projected
                .iter()
                .copied()
                .fold(Vec3::default(), Vec3::add)
                .scale(1.0 / projected.len() as f64);
            let candidate_residual = residual(candidate)?;
            if candidate_residual < best_residual {
                best = candidate;
                best_residual = candidate_residual;
            }
            if candidate.sub(current).length() <= tolerance.max(1e-12) {
                break;
            }
            current = candidate;
        }
        // Do not degrade an already-exact analytic intersection just to make
        // an over-constrained average marginally different.
        let maximum_move = tolerance * 100.0 * (1.0 + vertex.point.length());
        if best.sub(vertex.point).length() <= maximum_move
            && best_residual <= residual(vertex.point)?
        {
            vertex.point = best;
        }
    }

    let vertices = result
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    for piece in &mut result.pieces {
        let [curve_start, curve_end] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        let start = if canonical_vertices.contains(&piece.start_vertex_id) {
            *vertices
                .get(&piece.start_vertex_id)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing imprint vertex {}", piece.start_vertex_id)))?
        } else {
            piece.curve.evaluate(curve_start).or_refuse(KernelStage::Intersect, "evaluate")?
        };
        let end = if canonical_vertices.contains(&piece.end_vertex_id) {
            *vertices
                .get(&piece.end_vertex_id)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing imprint vertex {}", piece.end_vertex_id)))?
        } else {
            piece.curve.evaluate(curve_end).or_refuse(KernelStage::Intersect, "evaluate")?
        };
        if !canonical_vertices.contains(&piece.start_vertex_id)
            && !canonical_vertices.contains(&piece.end_vertex_id)
        {
            continue;
        }
        piece.curve = snap_curve_ends(&piece.curve, start, end)?;
        for pcurve in &mut piece.pcurves {
            let key = FaceKey {
                operand: pcurve.operand,
                face_id: pcurve.face_id,
            };
            let face = face_map
                .get(&key)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing imprint support face {key:?}")))?;
            // In the face's CHART: the mint drew this pcurve there, and on a
            // lifted face the carrier's own frame is a period away from it.
            pcurve.pcurve = rebuild_pcurve_preserving_branch(chart_of(charts, key, face), &piece.curve, &pcurve.pcurve)?;
        }
    }
    Ok(())
}

/// Unfloored bbox diagonal of a solid's vertices (unlike [`solid_scale`], which
/// floors at 1.0). The residual-merge bands must reflect the part's ACTUAL size:
/// flooring at 1.0 would give a sub-unit part a merge radius the size of a whole
/// unit, collapsing genuinely-distinct near-tangent junctions.
pub(super) fn raw_solid_extent(solid: &BrepSolid) -> f64 {
    if solid.vertices.is_empty() {
        return 0.0;
    }
    let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for vertex in &solid.vertices {
        low.x = low.x.min(vertex.point.x);
        low.y = low.y.min(vertex.point.y);
        low.z = low.z.min(vertex.point.z);
        high.x = high.x.max(vertex.point.x);
        high.y = high.y.max(vertex.point.y);
        high.z = high.z.max(vertex.point.z);
    }
    high.sub(low).length()
}

/// Bands for the residual-derived vertex merge, single-sourced so
/// `merge_residual_coincident_vertices` and its regression test stay in lockstep.
///
/// * `band` — the tolerance the intersection ACHIEVED: `assembler_weld`, grown by
///   the part size for genuinely large parts (below unit size it is the plain
///   weld, ~2e-5). Two points mutually within `band` of a surface set are one
///   point to the accuracy the marches could determine.
/// * `sep_cap` — the largest separation a double-mint can plausibly span. Uses
///   the kernel's junction-radius family (`1.5e-3` per unit extent — the same
///   coefficient as the fuzzy-junction clamp `.min(1.5e-3)` and fragment's
///   `diagonal * 1.5e-3` junction radius), scaled by the part's ACTUAL extent so
///   a small part gets a small cap. This bound is the deliberately-narrow part of
///   the gate: in the ill-conditioned (near-tangent) regime where this merge adds
///   value, `band` alone cannot separate a double-mint from two distinct points,
///   so the separation cap does. See the function docs for the limitation.
pub(super) fn residual_merge_bands(raw_extent: f64, tolerance: f64) -> (f64, f64) {
    let band = 2.0 * assembler_weld(tolerance) * raw_extent.max(1.0);
    let sep_cap = (1.5e-3 * raw_extent).max(band);
    (band, sep_cap)
}

/// Union-find `find` with path halving over a vertex-id parent map.
fn junction_find(parent: &mut HashMap<u64, u64>, mut x: u64) -> u64 {
    loop {
        let p = *parent.get(&x).unwrap_or(&x);
        if p == x {
            return x;
        }
        let grand = *parent.get(&p).unwrap_or(&p);
        parent.insert(x, grand);
        x = grand;
    }
}

/// Collapse imprint junction vertices that are the SAME intersection point into
/// one vertex, using the residual the imprint actually achieved as the identity
/// evidence (OCCT's `ComputeTolReached3d`). Returns whether any merge happened.
///
/// Two vertices merge only when the residual evidence PROVES they are one point:
/// * their incident surfaces span **≥ 3 distinct** carriers (the two endpoints of
///   a single section curve share only their two support surfaces — merging those
///   would collapse a real edge, so that case is excluded);
/// * **both endpoints AND their raw midpoint** lie on **every** union surface
///   within `band` — a transverse pair of genuinely-distinct roots (two real
///   corners) fails this at the midpoint, which sits off at least one surface by
///   the local sagitta, so only an under-determined single junction passes;
/// * the joint (multi-surface Newton) solution stays within `band` of all
///   surfaces and between the two vertices.
///
/// `band`/`sep_cap` come from [`residual_merge_bands`]. A genuinely-imperfect
/// imported triple point (surfaces that do not meet at a point within `band`,
/// e.g. a vendor-duplicated edge whose two copies sit ~2e-4 apart) is left for
/// the downstream weld/heal chain, NOT force-merged here.
///
/// LIMITATION (why this is scoped, not general). A well-conditioned (transverse)
/// double-mint separated by ≤ `band` is already welded by the existing assembler
/// machinery, so the value this merge adds is confined to the ILL-CONDITIONED
/// (near-tangent) regime — precisely where two marches legitimately disagree by
/// far more than `band` along the under-determined tangent. There, `band` alone
/// cannot distinguish one double-minted junction from two genuinely-distinct
/// nearby junctions (both lie on the shared near-tangent surfaces), so the
/// `sep_cap` — a bounded-empirical separation limit — carries the discrimination.
/// The principled future bound is `sep ≤ band / sin θ` (θ the surface crossing
/// angle); until that is plumbed, `sep_cap` is intentionally narrow and the
/// escape hatch exists.
pub(super) fn merge_residual_coincident_vertices(
    result: &mut ImprintResultRecord,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    charts: &FaceCharts<'_>,
    incident_faces: &HashMap<u64, HashSet<FaceKey>>,
    tolerance: f64,
    raw_extent: f64,
) -> Result<bool, KernelRefusal> {
    let (band, sep_cap) = residual_merge_bands(raw_extent, tolerance);

    let verts: Vec<(u64, Vec3)> = result.vertices.iter().map(|v| (v.id, v.point)).collect();
    let mut parent: HashMap<u64, u64> = verts.iter().map(|(id, _)| (*id, *id)).collect();
    let mut merged_point: HashMap<u64, Vec3> = verts.iter().map(|(id, p)| (*id, *p)).collect();
    let mut merged_any = false;
    // MICRO-PIECE GUARD (t20's terminal root cause): two junction vertices
    // joined by an EXISTING section piece are the endpoints of REAL geometry
    // — a micro-segment crossing a sub-mm model feature (00000231-p4's
    // 0.3 mm sliver face 591). The residual proof alone cannot distinguish
    // that from a double-minted point, because a micro-feature's flanking
    // junctions genuinely lie on the union of all incident surfaces; merging
    // them collapses the correctly-minted micro-piece into a loop-back stub
    // that later cleanup silently discards, tearing a hole in the section
    // chain ("edge curve end does not match vertex", gap = the micro-span).
    // The piece's own existence is the structural evidence: never merge
    // across it. Escape hatch: BREP_RESIDUAL_MERGE_PIECE_GUARD=0.
    let piece_connected: HashSet<(u64, u64)> =
        if std::env::var("BREP_RESIDUAL_MERGE_PIECE_GUARD").as_deref() != Ok("0") {
            result
                .pieces
                .iter()
                .map(|piece| {
                    let a = piece.start_vertex_id.min(piece.end_vertex_id);
                    let b = piece.start_vertex_id.max(piece.end_vertex_id);
                    (a, b)
                })
                .collect()
        } else {
            HashSet::default()
        };

    for a in 0..verts.len() {
        for b in (a + 1)..verts.len() {
            let (id_a, p_a) = verts[a];
            let (id_b, p_b) = verts[b];
            let sep = p_a.sub(p_b).length();
            if sep < 1e-12 || sep > sep_cap {
                continue;
            }
            if piece_connected.contains(&(id_a.min(id_b), id_a.max(id_b))) {
                continue;
            }
            let debug_junction = std::env::var("BREP_DEBUG_JUNCTION").is_ok();
            let mut union_keys: HashSet<FaceKey> = HashSet::default();
            for id in [id_a, id_b] {
                if let Some(keys) = incident_faces.get(&id) {
                    union_keys.extend(keys.iter().copied());
                }
            }
            if union_keys.len() < 3 {
                if debug_junction {
                    eprintln!(
                        "residual-merge v{id_a}/v{id_b} sep={sep:.4e}: REFUSED union_keys={} (<3)",
                        union_keys.len()
                    );
                }
                continue;
            }
            let union_surfaces: Vec<&NurbsSurface> = union_keys
                .iter()
                .filter_map(|key| face_map.get(key).map(|face| &face.surface))
                .collect();
            if union_surfaces.len() < 3 {
                continue;
            }
            let joint_residual = |point: Vec3| -> Result<f64, KernelRefusal> {
                let mut worst = 0.0f64;
                for surface in &union_surfaces {
                    worst = worst.max(project_point_to_surface(surface, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance);
                }
                Ok(worst)
            };
            // The whole segment (both ends + midpoint) must lie on every surface.
            let mid = p_a.add(p_b).scale(0.5);
            if joint_residual(p_a)? > band
                || joint_residual(p_b)? > band
                || joint_residual(mid)? > band
            {
                if debug_junction {
                    eprintln!(
                        "residual-merge v{id_a}/v{id_b} sep={sep:.4e}: REFUSED residual a={:.4e} b={:.4e} mid={:.4e} band={band:.4e} keys={}",
                        joint_residual(p_a)?,
                        joint_residual(p_b)?,
                        joint_residual(mid)?,
                        union_keys.len()
                    );
                }
                continue;
            }
            // Averaged-projection Newton for the joint point over all surfaces.
            let mut current = mid;
            let mut best = mid;
            let mut best_res = joint_residual(mid)?;
            for _ in 0..60 {
                let mut acc = Vec3::default();
                for surface in &union_surfaces {
                    acc = acc.add(project_point_to_surface(surface, current).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.point);
                }
                let candidate = acc.scale(1.0 / union_surfaces.len() as f64);
                let candidate_res = joint_residual(candidate)?;
                if candidate_res < best_res {
                    best = candidate;
                    best_res = candidate_res;
                }
                if candidate.sub(current).length() <= tolerance.max(1e-12) {
                    break;
                }
                current = candidate;
            }
            if best_res > band
                || best.sub(p_a).length() > sep_cap
                || best.sub(p_b).length() > sep_cap
            {
                continue;
            }
            let ra = junction_find(&mut parent, id_a);
            let rb = junction_find(&mut parent, id_b);
            if ra == rb {
                continue;
            }
            if std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
                eprintln!(
                    "[merge] v{id_a} <- v{id_b} sep={sep:.3e} band={band:.3e} sep_cap={sep_cap:.3e} best_res={best_res:.3e} to=({:.7},{:.7},{:.7})",
                    best.x, best.y, best.z,
                );
            }
            let rep = ra.min(rb);
            let absorbed = ra.max(rb);
            parent.insert(absorbed, rep);
            merged_point.insert(rep, best);
            merged_any = true;
        }
    }
    if !merged_any {
        return Ok(false);
    }

    let all_ids: Vec<u64> = parent.keys().copied().collect();
    let mut rep_of: HashMap<u64, u64> = HashMap::default();
    for id in all_ids {
        let rep = junction_find(&mut parent, id);
        rep_of.insert(id, rep);
    }
    // Keep one vertex per representative, at the merged joint point.
    let mut seen: HashSet<u64> = HashSet::default();
    let mut kept = Vec::new();
    for vertex in &result.vertices {
        let rep = rep_of.get(&vertex.id).copied().unwrap_or(vertex.id);
        if rep != vertex.id || !seen.insert(rep) {
            continue;
        }
        let point = merged_point.get(&rep).copied().unwrap_or(vertex.point);
        kept.push(ImprintVertex { id: rep, point });
    }
    result.vertices = kept;
    let position: HashMap<u64, Vec3> =
        result.vertices.iter().map(|v| (v.id, v.point)).collect();

    // Repoint pieces onto representatives; re-snap the moved curve ends and
    // rebuild their pcurves; drop a piece that collapsed to a zero-length spur.
    let mut kept_pieces = Vec::new();
    let mut dropped_pieces: HashSet<u64> = HashSet::default();
    for mut piece in std::mem::take(&mut result.pieces) {
        let start_rep = rep_of
            .get(&piece.start_vertex_id)
            .copied()
            .unwrap_or(piece.start_vertex_id);
        let end_rep = rep_of
            .get(&piece.end_vertex_id)
            .copied()
            .unwrap_or(piece.end_vertex_id);
        let moved = start_rep != piece.start_vertex_id || end_rep != piece.end_vertex_id;
        piece.start_vertex_id = start_rep;
        piece.end_vertex_id = end_rep;
        if moved {
            let start = *position
                .get(&start_rep)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing merged imprint vertex {start_rep}")))?;
            let end = *position
                .get(&end_rep)
                .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing merged imprint vertex {end_rep}")))?;
            // Both ends collapsed to one vertex and the piece is short: it can no
            // longer bound a fragment — drop it (mirrors the creation-time guard
            // in `process_curve`). A genuine long closed loop is preserved.
            if start_rep == end_rep
                && curve_length_rough(&piece.curve)? < (4.0 * sep_cap).max(1e-3)
            {
                dropped_pieces.insert(piece.id);
                continue;
            }
            piece.curve = snap_curve_ends(&piece.curve, start, end)?;
            for pcurve in &mut piece.pcurves {
                let key = FaceKey {
                    operand: pcurve.operand,
                    face_id: pcurve.face_id,
                };
                let face = face_map
                    .get(&key)
                    .ok_or_else(|| KernelRefusal::internal(KernelStage::Intersect, "imprint.junctions", format!("missing imprint support face {key:?}")))?;
                // In the face's CHART: the mint drew this pcurve there, and
                // on a lifted face the carrier's frame is a period from it.
                pcurve.pcurve = rebuild_pcurve_preserving_branch(
                    chart_of(charts, key, face),
                    &piece.curve,
                    &pcurve.pcurve,
                )?;
            }
        }
        kept_pieces.push(piece);
    }
    result.pieces = kept_pieces;
    // A dropped piece must also leave the per-face imprint lists, or `fragment`
    // errors on a piece id it can no longer resolve.
    if !dropped_pieces.is_empty() {
        for face in &mut result.by_face {
            face.piece_ids.retain(|id| !dropped_pieces.contains(id));
        }
    }
    Ok(true)
}
