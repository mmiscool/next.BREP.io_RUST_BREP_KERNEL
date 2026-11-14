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
    emitted: bool,
) -> Result<NurbsCurve, KernelRefusal> {
    // An EMITTED pcurve (`imprint/hermite.rs`, named by the builder's
    // `emitted_pcurves`) shares the curve's knots; the snap moved the
    // curve's end control points only, so its ends move to the chart images
    // of the new ends and nothing is re-fitted. The branch is preserved by
    // construction. A fitted pcurve is re-marched as before, whatever its
    // shape (fixture 25's `pcurve.fits` pins that).
    if emitted {
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        let start = curve.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?;
        let end = curve.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?;
        return super::hermite::piece_pcurve_on_chart(previous, surface, start, end).or_refuse(KernelStage::Intersect, "piece_pcurve_on_chart");
    }
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

/// What minted an imprint junction vertex, in the order a combine prefers:
/// an operand's own vertex, a point on an operand edge (the section crossed
/// or pierced it), a section end polished onto its two carriers.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum JunctionSource {
    OperandVertex,
    EdgePoint,
    FacePoint,
}

/// Per-vertex evidence the builder kept: the merge radius each mint asked
/// for and the lowest-dimensional source that minted it.
#[derive(Default)]
pub(super) struct JunctionEvidence {
    pub(super) radii: HashMap<u64, f64>,
    pub(super) sources: HashMap<u64, JunctionSource>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum JunctionCombineMode {
    Off,
    Check,
    On,
}

/// [`ImprintOptions::two_radius_junctions`] under `BREP_JUNCTION_COMBINE`.
pub(super) fn junction_combine_mode(options: &ImprintOptions) -> JunctionCombineMode {
    match mode_switch!(live "BREP_JUNCTION_COMBINE", options.two_radius_junctions) {
        crate::env_read::Mode::Off => JunctionCombineMode::Off,
        crate::env_read::Mode::Check => JunctionCombineMode::Check,
        crate::env_read::Mode::On => JunctionCombineMode::On,
    }
}

/// Strict combine radius as a multiple of the two candidates' own evidence
/// radii: two definitions of one junction combine only within
/// `STRICT_FACTOR × (r_a + r_b)`. Chosen from the census in the record.
const STRICT_FACTOR: f64 = 5.0;

/// `BREP_JUNCTION_CENSUS=<file>`: one line per junction vertex in check and
/// on mode, labelled by `BREP_CENSUS_LABEL`.
fn junction_census_line(line: impl FnOnce() -> String) {
    if let Ok(path) = std::env::var("BREP_JUNCTION_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let label = std::env::var("BREP_CENSUS_LABEL").unwrap_or_default();
            let _ = writeln!(file, "junction census: label {label} {}", line());
        }
    }
}

/// Every vertex's incident support faces and the piece end points that
/// reference it.
fn junction_incidence(
    result: &ImprintResultRecord,
) -> Result<(HashMap<u64, HashSet<FaceKey>>, HashMap<u64, Vec<Vec3>>), KernelRefusal> {
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
    Ok((incident_faces, incident_points))
}

/// For every junction vertex, the operand vertices of its support faces and
/// the other junctions sharing a support face (not joined to it by a piece)
/// within the wide residual band are its candidates; an operand vertex
/// within the strict radius anchors the junction there, else another
/// junction within the strict radius merges with it, the surviving position
/// being the lower-dimensional source's (the smaller evidence radius on a
/// tie). In check mode nothing moves; both modes log one census line per
/// junction. Returns whether any junction moved or merged, with the anchored
/// representatives in `anchored`.
#[allow(clippy::too_many_arguments)]
fn two_radius_combine(
    result: &mut ImprintResultRecord,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    charts: &FaceCharts<'_>,
    incident_faces: &HashMap<u64, HashSet<FaceKey>>,
    tolerance: f64,
    raw_extent: f64,
    evidence: &JunctionEvidence,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
    mode: JunctionCombineMode,
    anchored: &mut HashSet<u64>,
) -> Result<bool, KernelRefusal> {
    let weld = assembler_weld(tolerance);
    let (_, capture) = residual_merge_bands(raw_extent, tolerance);
    // The operand vertices on each support face's loops.
    let mut face_vertices: HashMap<FaceKey, Vec<(u64, Vec3)>> = HashMap::default();
    for (operand, solid) in [(0u8, solid_a), (1u8, solid_b)] {
        let points: HashMap<u64, Vec3> = solid.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
        let ends: HashMap<u64, (u64, u64)> =
            solid.edges.iter().map(|edge| (edge.id, (edge.start_vertex_id, edge.end_vertex_id))).collect();
        for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
            let mut ids: Vec<u64> = Vec::new();
            for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
                if let Some(&(start, end)) = ends.get(&coedge.edge_id) {
                    for id in [start, end] {
                        if !ids.contains(&id) {
                            ids.push(id);
                        }
                    }
                }
            }
            let entries = ids.into_iter().filter_map(|id| points.get(&id).map(|point| (id, *point))).collect();
            face_vertices.insert(FaceKey { operand, face_id: face.id }, entries);
        }
    }
    let radius_of = |id: u64| evidence.radii.get(&id).copied().unwrap_or(0.0).max(weld);
    let source_of = |id: u64| evidence.sources.get(&id).copied().unwrap_or(JunctionSource::FacePoint);
    let piece_connected: HashSet<(u64, u64)> = result
        .pieces
        .iter()
        .map(|piece| (piece.start_vertex_id.min(piece.end_vertex_id), piece.start_vertex_id.max(piece.end_vertex_id)))
        .collect();
    let verts: Vec<(u64, Vec3)> = result.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let empty = HashSet::default();
    // Pass 1: the anchors (an operand vertex within the strict radius).
    let mut anchor: HashMap<u64, (u8, u64, Vec3)> = HashMap::default();
    let mut nearest_junction: HashMap<u64, (f64, u64, f64)> = HashMap::default();
    for &(id, point) in &verts {
        let keys = incident_faces.get(&id).unwrap_or(&empty);
        let radius = radius_of(id);
        let mut nearest_vertex: Option<(f64, u8, u64, Vec3)> = None;
        for key in keys {
            for &(vertex_id, vertex_point) in face_vertices.get(key).map(Vec::as_slice).unwrap_or(&[]) {
                let distance = vertex_point.sub(point).length();
                if distance <= capture && nearest_vertex.is_none_or(|best| distance < best.0) {
                    nearest_vertex = Some((distance, key.operand, vertex_id, vertex_point));
                }
            }
        }
        let mut nearest: Option<(f64, u64, f64)> = None;
        for &(other_id, other_point) in &verts {
            if other_id == id || piece_connected.contains(&(id.min(other_id), id.max(other_id))) {
                continue;
            }
            let other_keys = incident_faces.get(&other_id).unwrap_or(&empty);
            if keys.is_disjoint(other_keys) {
                continue;
            }
            let distance = other_point.sub(point).length();
            if distance <= capture && nearest.is_none_or(|best| distance < best.0) {
                nearest = Some((distance, other_id, STRICT_FACTOR * (radius + radius_of(other_id))));
            }
        }
        let strict_vertex = STRICT_FACTOR * (radius + weld);
        junction_census_line(|| {
            format!(
                "vertex={id} source={:?} radius={radius:.3e} faces={} capture={capture:.3e} operand_vertex={} junction={}",
                source_of(id),
                keys.len(),
                match nearest_vertex {
                    Some((distance, operand, vertex_id, _)) => format!(
                        "{distance:.3e}/{operand}:{vertex_id} strict={strict_vertex:.3e} inside={}",
                        distance <= strict_vertex
                    ),
                    None => "none".to_string(),
                },
                match nearest {
                    Some((distance, other_id, strict)) => format!(
                        "{distance:.3e}/{other_id}:{:?} strict={strict:.3e} inside={}",
                        source_of(other_id),
                        distance <= strict
                    ),
                    None => "none".to_string(),
                }
            )
        });
        // A junction already on the operand vertex (within the model
        // tolerance) has nothing to combine: the pieces end there as minted.
        if let Some((distance, operand, vertex_id, vertex_point)) = nearest_vertex {
            if distance > tolerance && distance <= strict_vertex {
                anchor.insert(id, (operand, vertex_id, vertex_point));
            }
        }
        if let Some(found) = nearest {
            nearest_junction.insert(id, found);
        }
    }
    if mode != JunctionCombineMode::On {
        return Ok(false);
    }
    // Pass 2: merges between junctions within their strict radius, the
    // position of the lower-dimensional one surviving (an anchored junction
    // counts as an operand vertex).
    let mut parent: HashMap<u64, u64> = verts.iter().map(|(id, _)| (*id, *id)).collect();
    let mut merged_point: HashMap<u64, Vec3> = HashMap::default();
    let anchors = anchor.clone();
    let effective = |id: u64| -> (JunctionSource, f64, Vec3) {
        match anchors.get(&id) {
            Some((_, _, point)) => (JunctionSource::OperandVertex, weld, *point),
            None => (
                source_of(id),
                radius_of(id),
                verts.iter().find(|(other, _)| *other == id).map(|(_, point)| *point).unwrap_or_default(),
            ),
        }
    };
    let mut changed = false;
    for &(id, _) in &verts {
        let Some(&(distance, other_id, strict)) = nearest_junction.get(&id) else {
            continue;
        };
        if distance <= tolerance || distance > strict {
            continue;
        }
        let (ra, rb) = (junction_find(&mut parent, id), junction_find(&mut parent, other_id));
        if ra == rb {
            continue;
        }
        let (source_a, radius_a, point_a) = effective(ra);
        let (source_b, radius_b, point_b) = effective(rb);
        let keep_a = (source_a, radius_a, ra) <= (source_b, radius_b, rb);
        let (rep, absorbed, point) = if keep_a { (ra, rb, point_a) } else { (rb, ra, point_b) };
        parent.insert(absorbed, rep);
        merged_point.insert(rep, point);
        if anchor.contains_key(&absorbed) && !anchor.contains_key(&rep) {
            if let Some(found) = anchor.get(&absorbed).copied() {
                anchor.insert(rep, found);
            }
        }
        if std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
            eprintln!(
                "[two-radius] v{rep} <- v{absorbed} sep={distance:.3e} strict={strict:.3e} sources {source_a:?}/{source_b:?} to=({:.7},{:.7},{:.7})",
                point.x, point.y, point.z
            );
        }
        changed = true;
    }
    // An anchored junction's operand vertex beats the operand edge too: the
    // split the crossing or pierce recorded a strict radius from that vertex
    // is retracted, so the pieces end at the vertex and the edge is not cut
    // beside it.
    let edge_curves: HashMap<(u8, u64), &EdgeRecord> = [(0u8, solid_a), (1u8, solid_b)]
        .into_iter()
        .flat_map(|(operand, solid)| solid.edges.iter().map(move |edge| ((operand, edge.id), edge)))
        .collect();
    for (&id, &(operand, vertex_id, point)) in &anchor {
        let rep = junction_find(&mut parent, id);
        merged_point.insert(rep, point);
        anchored.insert(rep);
        let strict = STRICT_FACTOR * (radius_of(id) + weld);
        for record in result.edge_splits.iter_mut().filter(|record| record.operand == operand) {
            let Some(edge) = edge_curves.get(&(operand, record.edge_id)) else {
                continue;
            };
            if edge.start_vertex_id != vertex_id && edge.end_vertex_id != vertex_id {
                continue;
            }
            record.parameters.retain(|&parameter| {
                edge.curve
                    .evaluate(parameter)
                    .map(|split_point| split_point.sub(point).length() > strict)
                    .unwrap_or(true)
            });
        }
        if std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
            eprintln!(
                "[two-radius] v{id} anchored to operand {operand} vertex {vertex_id} at ({:.7},{:.7},{:.7})",
                point.x, point.y, point.z
            );
        }
        changed = true;
    }
    if !changed {
        return Ok(false);
    }
    let (_, sep_cap) = residual_merge_bands(raw_extent, tolerance);
    relocate_junctions(result, &mut parent, &merged_point, face_map, charts, emitted_pcurves, (4.0 * sep_cap).max(1e-3), true)?;
    Ok(true)
}

#[allow(clippy::too_many_arguments)]
/// `residual_merge`: whether the residual-derived coincident-vertex merge
/// below may run. A composite one-call carrier imprint
/// ([`super::build_carrier_imprints_within`], offset-shell lane 2026-10-27)
/// passes `false`: each of its admitted face pairs was, until the one call,
/// its own `build_imprints` holding two carriers, and the merge's "≥ 3
/// distinct incident carriers" gate could never fire inside one pair. Across
/// pairs it can — the successive-fillet history's corner-patch offset pokes
/// 1e-3 through two tangent planes, and the two sections' ends 9.5e-4 apart
/// were merged to one joint vertex, after which the y = 9.8 wall carrier's cut
/// chain no longer ended where the fragmenter's bridge found it and the wall
/// never fragmented (O.S10 refused as an unwelded curved rim). Per-pair the
/// pipeline's positional `merge_vertex` is the only cross-pair identity, and
/// it never moves a curve end; the composite keeps that contract.
pub(super) fn canonicalize_imprint_junctions(
    result: &mut ImprintResultRecord,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
    evidence: &JunctionEvidence,
    combine: JunctionCombineMode,
    feature_scale: &crate::feature_scale::FeatureScale,
    residual_merge: bool,
) -> Result<(), KernelRefusal> {
    let face_map = faces(solid_a, 0)
        .into_iter()
        .chain(faces(solid_b, 1))
        .map(|face| (face.key(), face.face))
        .collect::<HashMap<_, _>>();
    let (mut incident_faces, mut incident_points) = junction_incidence(result)?;
    let edge_curves: HashMap<(u8, u64), &EdgeRecord> = [(0u8, solid_a), (1u8, solid_b)]
        .into_iter()
        .flat_map(|(operand, solid)| solid.edges.iter().map(move |edge| ((operand, edge.id), edge)))
        .collect();
    let operand_vertices: Vec<(u8, Vec3)> = solid_a
        .vertices
        .iter()
        .map(|vertex| (0u8, vertex.point))
        .chain(solid_b.vertices.iter().map(|vertex| (1u8, vertex.point)))
        .collect();
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
    if residual_merge
        && std::env::var("BREP_IMPRINT_RESIDUAL_MERGE").as_deref() != Ok("0")
        && merge_residual_coincident_vertices(
            result,
            &face_map,
            &edge_curves,
            &operand_vertices,
            charts,
            &incident_faces,
            tolerance,
            raw_extent,
            evidence,
            emitted_pcurves,
            feature_scale,
        )?
    {
        // Incidence changed under the merge — rebuild it for the polish below.
        (incident_faces, incident_points) = junction_incidence(result)?;
    }
    let mut anchored: HashSet<u64> = HashSet::default();
    if combine != JunctionCombineMode::Off
        && two_radius_combine(
            result,
            solid_a,
            solid_b,
            &face_map,
            charts,
            &incident_faces,
            tolerance,
            raw_extent,
            evidence,
            emitted_pcurves,
            combine,
            &mut anchored,
        )?
    {
        (incident_faces, incident_points) = junction_incidence(result)?;
    }
    // A piece end the builder minted within its vertex's evidence radius but
    // off the vertex ends there now (see `consolidate_fuzzy_piece_ends`).
    if std::env::var("BREP_RESIDUAL_MERGE_EDGE_PLACEMENT").as_deref() != Ok("0")
        && consolidate_fuzzy_piece_ends(result, &face_map, &edge_curves, charts, tolerance, evidence, emitted_pcurves)?
    {
        (incident_faces, incident_points) = junction_incidence(result)?;
    }
    // Freeze an intrinsic reference before any vertex is polished. A world
    // origin gives identical junctions different acceptance budgets after a
    // rigid placement. Relative coordinates also avoid summing large offsets.
    let reference = junction_reference(result.vertices.iter().map(|vertex| vertex.point));
    if std::env::var("BREP_DEBUG_CANONICAL_JUNCTIONS").is_ok() {
        eprintln!("JUNCTION-REFERENCE point={reference:?} vertices={}", result.vertices.len());
    }
    let canonical_vertices = result
        .vertices
        .iter()
        .filter(|vertex| {
            let limit = tolerance * 10.0 * (1.0 + vertex.point.sub(reference).length());
            incident_points.get(&vertex.id).is_some_and(|points| {
                points
                    .iter()
                    .all(|point| point.sub(vertex.point).length() <= limit)
            })
        })
        .map(|vertex| vertex.id)
        .collect::<HashSet<_>>();
    
    if std::env::var("BREP_DEBUG_CANONICAL_JUNCTIONS").is_ok() {
        for vertex in &result.vertices {
            let limit = tolerance * 10.0 * (1.0 + vertex.point.sub(reference).length());
            let gap = incident_points.get(&vertex.id).map(|points| points.iter()
                .map(|p| p.sub(vertex.point).length()).fold(0.0_f64, f64::max));
            let old_limit = tolerance * 10.0 * (1.0 + vertex.point.length());
            let old_member = gap.is_some_and(|gap| gap <= old_limit);
            let member = canonical_vertices.contains(&vertex.id);
            eprintln!("JUNCTION-MEMBERSHIP vertex={} point={:?} tolerance={tolerance:.17e} incident_gap={gap:?} limit={limit:.17e} canonical={member} old_limit={old_limit:.17e} old_canonical={old_member} flip={}", vertex.id, vertex.point, member != old_member);
            if member != old_member {
                eprintln!("JUNCTION-FLIP vertex={} faces={:?} pieces={:?}", vertex.id, incident_faces.get(&vertex.id), result.pieces.iter().filter(|piece| piece.start_vertex_id == vertex.id || piece.end_vertex_id == vertex.id).map(|piece| piece.id).collect::<Vec<_>>());
            }
        }
    }
    for vertex in &mut result.vertices {
        if !canonical_vertices.contains(&vertex.id) {
            continue;
        }
        // An operand vertex's position wins over the carriers' average: a
        // junction anchored to one is not polished off it.
        if anchored.contains(&vertex.id) {
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
        let maximum_move = tolerance * 100.0 * (1.0 + vertex.point.sub(reference).length());
        if std::env::var("BREP_DEBUG_CANONICAL_JUNCTIONS").is_ok() {
            eprintln!("JUNCTION-POLISH vertex={} point={:?} candidate={best:?} tolerance={tolerance:.17e} movement={:.17e} maximum_move={maximum_move:.17e} before_residual={:.17e} best_residual={best_residual:.17e} inside_movement={}", vertex.id, vertex.point, best.sub(vertex.point).length(), residual(vertex.point)?, best.sub(vertex.point).length() <= maximum_move);
            let old_maximum = tolerance * 100.0 * (1.0 + vertex.point.length());
            let movement = best.sub(vertex.point).length();
            eprintln!("JUNCTION-OLD-MOVE vertex={} old_maximum={old_maximum:.17e} old_inside={} flip={} faces={keys:?}", vertex.id, movement <= old_maximum, (movement <= old_maximum) != (movement <= maximum_move));
        }
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
            pcurve.pcurve = rebuild_pcurve_preserving_branch(chart_of(charts, key, face), &piece.curve, &pcurve.pcurve, emitted_pcurves.contains(&(piece.id, key)))?;
        }
    }
    Ok(())
}

/// Unfloored bbox diagonal of a solid's vertices (unlike [`solid_scale`], which
/// floors at 1.0). The residual-merge bands must reflect the part's ACTUAL size:
/// flooring at 1.0 would give a sub-unit part a merge radius the size of a whole
/// unit, collapsing genuinely-distinct near-tangent junctions.
fn junction_reference(mut points: impl Iterator<Item = Vec3>) -> Vec3 {
    let Some(anchor) = points.next() else {
        return Vec3::default();
    };
    let (sum, count) = points.fold((Vec3::default(), 1usize), |(sum, count), point| {
        (sum.add(point.sub(anchor)), count + 1)
    });
    anchor.add(sum.scale(1.0 / count as f64))
}

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
/// POSITION (user bug follow-up 2026-11-02, F10's opposite cap). The joint
/// Newton solution is under-determined along the very tangent the merge
/// exists for: every point of the segment lies on every union surface, so
/// the averaged projection settles wherever it started (the midpoint). An
/// operand EDGE pins that direction. The group's position is therefore the
/// member that splits the most operand edges (the crossing of two edges is
/// fully determined, a point on one edge along it), and the joint Newton
/// point only when no member splits an edge. The cap union's junction 8 was
/// the crossing of edges 46 and 152 (both splits within 1e-10 of it) and the
/// Newton point slid it 2.3e-3 along edge 152 toward four noise crossings of
/// the tangent section with that edge, off edge 46 whose split stayed at the
/// true crossing; the pieces already ending at junction 8 were never snapped
/// either (`relocate_junctions` ran with `snap_moved = false`), so the
/// fragmenter met curve ends 2.3e-3 from their vertex. Now every piece that
/// references a moved representative is snapped to it, and an absorbed
/// member's split on an edge the holder already splits is retracted, so the
/// edge is not cut again beside the junction.
///
/// OPERAND-VERTEX GUARD (user bug follow-up 2026-11-02, the visor clip). Two
/// members that sit on two DISTINCT vertices of the SAME operand are two
/// junctions whatever the residual proof reads: that operand's own topology
/// holds them apart (an edge of it longer than the weld runs between them),
/// and merging them collapses that edge. The visor clip's fourth slab shares
/// its front plane with the third's, whose front face the second slab's
/// slanted wall had already cut 3.3e-4 from its end wall; the boundary copies
/// of that 3.3e-4 strip's two verticals onto the new slab's face minted
/// (14.49967, 14.99, z) and (14.5, 14.99, z) as distinct vertices, and this
/// merge welded them (`sep=3.3e-4`, `band=1.0e-3` = 2·weld·extent: the
/// strip's surfaces are three planes the midpoint lies within 1.7e-4 of).
/// The copy of the slant junction collapsed onto the end-wall line, the strip
/// of the new face went out with the In fragment, the old sliver lost the
/// coincident-copy vote, and both verticals went one-use. The pinch fixture
/// (`the_radius_gate_leaves_pinching_and_star_selections_alone`) spans two
/// operand vertices too, but one PER operand — a rail end of each round —
/// and keeps merging. Escape hatch `BREP_RESIDUAL_MERGE_OPERAND_GUARD=0`.
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
#[allow(clippy::too_many_arguments)]
pub(super) fn merge_residual_coincident_vertices(
    result: &mut ImprintResultRecord,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    edge_curves: &HashMap<(u8, u64), &EdgeRecord>,
    operand_vertices: &[(u8, Vec3)],
    charts: &FaceCharts<'_>,
    incident_faces: &HashMap<u64, HashSet<FaceKey>>,
    tolerance: f64,
    raw_extent: f64,
    evidence: &JunctionEvidence,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
    feature_scale: &crate::feature_scale::FeatureScale,
) -> Result<bool, KernelRefusal> {
    let (band, sep_cap) = residual_merge_bands(raw_extent, tolerance);
    let weld = assembler_weld(tolerance);
    let radius_of = |id: u64| evidence.radii.get(&id).copied().unwrap_or(0.0).max(weld);
    // The operand vertex a junction sits on (within its evidence radius), the
    // nearest when several do; see OPERAND-VERTEX GUARD on this function.
    let operand_vertex_at = |point: Vec3, radius: f64| -> Option<(u8, Vec3)> {
        operand_vertices
            .iter()
            .filter(|(_, vertex)| vertex.sub(point).length() <= radius)
            .min_by(|a, b| a.1.sub(point).length().total_cmp(&b.1.sub(point).length()))
            .copied()
    };
    let operand_guard = std::env::var("BREP_RESIDUAL_MERGE_OPERAND_GUARD").as_deref() != Ok("0");

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
            if operand_guard {
                if let (Some((operand_a, vertex_a)), Some((operand_b, vertex_b))) =
                    (operand_vertex_at(p_a, radius_of(id_a)), operand_vertex_at(p_b, radius_of(id_b)))
                {
                    if operand_a == operand_b && vertex_a.sub(vertex_b).length() > weld {
                        if debug_junction {
                            eprintln!(
                                "residual-merge v{id_a}/v{id_b} sep={sep:.4e}: REFUSED two distinct vertices of operand {operand_a} ({:.3e} apart)",
                                vertex_a.sub(vertex_b).length()
                            );
                        }
                        continue;
                    }
                }
            }
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
            // FEATURE SCALE B3 (`BREP_FEATURE_SCALE_IMPRINT`): beside a sliver
            // the identity proof's band (2·weld·extent, 2e-5 on a 10 mm part)
            // is capped at a quarter of it. The hairline slot's two top corners,
            // one on each wall 1e-6 apart, passed the uncapped proof and were
            // averaged onto x = 5.0000004375, off both walls; their midpoint
            // lies 5e-7 from each, past the 2.5e-7 cap. A near-tangent
            // double-mint's points lie on every surface to the march's
            // accuracy, so the cap leaves it alone (fixture
            // 10_regression_step269_cone's plates are 1.5e-3 thick or more).
            // Slabs of one operand only: see `FeatureScale::cap_by_slabs`.
            let band = if feature_scale.is_empty() { band } else { feature_scale.cap_by_slabs(mid, band) };
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
    // Escape hatch for the 2026-11-02 repair (POSITION above): the joint
    // point, unsnapped representatives and untouched splits of before.
    if std::env::var("BREP_RESIDUAL_MERGE_EDGE_PLACEMENT").as_deref() == Ok("0") {
        relocate_junctions(result, &mut parent, &merged_point, face_map, charts, emitted_pcurves, (4.0 * sep_cap).max(1e-3), false)?;
        return Ok(true);
    }
    absorb_junctions_enclosed_by_merged_groups(result, face_map, incident_faces, &verts, &mut parent, band, sep_cap)?;
    let retracted = place_merged_groups_on_operand_edges(result, edge_curves, operand_vertices, &verts, &mut parent, &mut merged_point, tolerance, band, evidence)?;
    // Only a merged group's representative counts as moved: the pieces that
    // reference it are snapped to the group's position, every other piece is
    // left as the builder minted it.
    let mut group_size: HashMap<u64, usize> = HashMap::default();
    for &(id, _) in &verts {
        *group_size.entry(junction_find(&mut parent, id)).or_default() += 1;
    }
    let moved: HashMap<u64, Vec3> = merged_point
        .iter()
        .filter(|(rep, _)| group_size.get(rep).is_some_and(|size| *size > 1))
        .map(|(rep, point)| (*rep, *point))
        .collect();
    relocate_junctions(result, &mut parent, &moved, face_map, charts, emitted_pcurves, (4.0 * sep_cap).max(1e-3), true)?;
    if retracted > 0 && std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
        eprintln!("[merge] retracted {retracted} edge split(s) of absorbed junctions");
    }
    Ok(true)
}

/// Whether both ends of the segment `a`–`b` and its midpoint lie within
/// `band` of every surface: the residual merge's identity proof.
fn segment_on_surfaces(surfaces: &[&NurbsSurface], a: Vec3, b: Vec3, band: f64) -> Result<bool, KernelRefusal> {
    for point in [a, b, a.add(b).scale(0.5)] {
        for surface in surfaces {
            if project_point_to_surface(surface, point).or_refuse(KernelStage::Intersect, "project_point_to_surface")?.distance > band {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// A junction that is NOT in a merged group but whose every piece (two or
/// more) leads into ONE merged group is enclosed by that group: its pieces
/// now form a closed loop between one vertex and itself, geometry the merge
/// created rather than found (the cap union's junction 18, 4.1e-3 along the
/// tangent section from the edge crossing, kept piece 19 to the crossing and
/// piece 21 to the first absorbed noise crossing; the fragmenter then kept
/// four regions of 1e-11 to 1e-9 area between them). The micro-piece guard
/// does not protect such a loop. The junction joins the group when it is
/// within `sep_cap` of a member and the segment to that member passes the
/// residual proof on the union of their incident surfaces; the loop's pieces
/// then drop as spurs in `relocate_junctions`. Repeats until no junction
/// qualifies, so a chain of such vertices folds in too.
fn absorb_junctions_enclosed_by_merged_groups(
    result: &ImprintResultRecord,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    incident_faces: &HashMap<u64, HashSet<FaceKey>>,
    verts: &[(u64, Vec3)],
    parent: &mut HashMap<u64, u64>,
    band: f64,
    sep_cap: f64,
) -> Result<(), KernelRefusal> {
    let debug = std::env::var("BREP_DEBUG_JUNCTION").is_ok();
    let empty = HashSet::default();
    for _ in 0..verts.len() {
        let mut absorbed_any = false;
        let mut members: HashMap<u64, Vec<(u64, Vec3)>> = HashMap::default();
        for &(id, point) in verts {
            let rep = junction_find(parent, id);
            members.entry(rep).or_default().push((id, point));
        }
        for &(id, point) in verts {
            let own_rep = junction_find(parent, id);
            if members.get(&own_rep).is_some_and(|group| group.len() > 1) {
                continue;
            }
            let mut target: Option<u64> = None;
            let mut pieces = 0usize;
            for piece in &result.pieces {
                let other = if piece.start_vertex_id == id {
                    piece.end_vertex_id
                } else if piece.end_vertex_id == id {
                    piece.start_vertex_id
                } else {
                    continue;
                };
                pieces += 1;
                let rep = junction_find(parent, other);
                if target.is_some_and(|target| target != rep) {
                    target = None;
                    pieces = 0;
                    break;
                }
                target = Some(rep);
            }
            let Some(rep) = target else {
                continue;
            };
            if pieces < 2 || rep == id {
                continue;
            }
            let Some(group) = members.get(&rep).filter(|group| group.len() > 1) else {
                continue;
            };
            let own_keys = incident_faces.get(&id).unwrap_or(&empty);
            let mut proven: Option<(u64, f64)> = None;
            for &(member_id, member_point) in group {
                let sep = member_point.sub(point).length();
                if sep > sep_cap {
                    continue;
                }
                let mut keys = own_keys.clone();
                keys.extend(incident_faces.get(&member_id).unwrap_or(&empty).iter().copied());
                let surfaces: Vec<&NurbsSurface> = keys.iter().filter_map(|key| face_map.get(key).map(|face| &face.surface)).collect();
                if surfaces.is_empty() || !segment_on_surfaces(&surfaces, point, member_point, band)? {
                    continue;
                }
                if proven.is_none_or(|(_, best)| sep < best) {
                    proven = Some((member_id, sep));
                }
            }
            let Some((member_id, sep)) = proven else {
                continue;
            };
            if debug {
                eprintln!("[merge] v{rep} <- v{id} enclosed: {pieces} pieces lead into the group, {sep:.3e} from member v{member_id}, band={band:.3e}");
            }
            parent.insert(own_rep, rep);
            absorbed_any = true;
        }
        if !absorbed_any {
            break;
        }
    }
    Ok(())
}

/// A piece end minted within its vertex's own evidence radius but off the
/// vertex ends AT the vertex. The builder's `vertex` hands a caller whose
/// point lies within an existing vertex's merge radius that vertex's id but
/// the caller's own point, so the piece is minted ending beside the vertex
/// it names (the cap union's piece 38 ended 9.06e-5 from junction 31, whose
/// radius was 1.33e-4, with edge 68 split at both points 3.8e-6 apart in
/// parameter); the canonical polish then skips the vertex because its
/// incident ends disagree, and the end is never closed. The assembler
/// refused that union with one-use edges on the cap. The edit is bounded by
/// the radius the builder already used to call the two points one junction,
/// and the duplicate split of an operand edge within that radius is
/// retracted, the one nearest the vertex surviving. Returns whether any
/// piece end moved.
fn consolidate_fuzzy_piece_ends(
    result: &mut ImprintResultRecord,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    edge_curves: &HashMap<(u8, u64), &EdgeRecord>,
    charts: &FaceCharts<'_>,
    tolerance: f64,
    evidence: &JunctionEvidence,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
) -> Result<bool, KernelRefusal> {
    let weld = assembler_weld(tolerance);
    let radius_of = |id: u64| evidence.radii.get(&id).copied().unwrap_or(0.0).max(weld);
    let position: HashMap<u64, Vec3> = result.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let mut stale: HashMap<u64, Vec3> = HashMap::default();
    for piece in &result.pieces {
        let [t0, t1] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
        for (parameter, id) in [(t0, piece.start_vertex_id), (t1, piece.end_vertex_id)] {
            let Some(point) = position.get(&id) else {
                continue;
            };
            let gap = piece.curve.evaluate(parameter).or_refuse(KernelStage::Intersect, "evaluate")?.sub(*point).length();
            if gap > tolerance && gap <= radius_of(id) {
                stale.insert(id, *point);
            }
        }
    }
    if stale.is_empty() {
        return Ok(false);
    }
    if std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
        let mut ids: Vec<u64> = stale.keys().copied().collect();
        ids.sort_unstable();
        eprintln!("[consolidate] piece ends within their vertex's evidence radius snapped onto it: {ids:?}");
    }
    let mut parent: HashMap<u64, u64> = position.keys().map(|id| (*id, *id)).collect();
    relocate_junctions(result, &mut parent, &stale, face_map, charts, emitted_pcurves, 0.0, true)?;
    let mut retracted = 0usize;
    let chains = SplitChains::new(edge_curves)?;
    for (id, point) in &stale {
        let radius = radius_of(*id);
        for record in &mut result.edge_splits {
            let Some(edge) = edge_curves.get(&(record.operand, record.edge_id)) else {
                continue;
            };
            let mut within: Vec<(f64, f64)> = Vec::new();
            for &parameter in &record.parameters {
                let distance = edge.curve.evaluate(parameter).or_refuse(KernelStage::Intersect, "evaluate")?.sub(*point).length();
                if distance <= radius {
                    within.push((distance, parameter));
                }
            }
            // An end of this edge where its other half continues it is a
            // split already made (`SplitChains`): within the radius it is one
            // of the duplicates, and it is the one that cannot be retracted.
            let joint = chains
                .joint_ends
                .get(&(record.operand, record.edge_id))
                .into_iter()
                .flatten()
                .map(|end| end.sub(*point).length())
                .filter(|distance| *distance <= radius)
                .min_by(|a, b| a.total_cmp(b));
            if within.len() + usize::from(joint.is_some()) < 2 {
                continue;
            }
            within.sort_by(|a, b| a.0.total_cmp(&b.0));
            let keep = if joint.is_some() { None } else { Some(within[0].1) };
            let before = record.parameters.len();
            record
                .parameters
                .retain(|&parameter| Some(parameter) == keep || !within.iter().any(|(_, other)| *other == parameter));
            retracted += before - record.parameters.len();
        }
    }
    if retracted > 0 && std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
        eprintln!("[consolidate] retracted {retracted} duplicate edge split(s) within the vertex radius");
    }
    Ok(true)
}

/// Whether two curves are one curve bit for bit (degree, knots, controls).
pub(super) fn same_curve_bits(a: &NurbsCurve, b: &NurbsCurve) -> bool {
    a.degree == b.degree
        && a.knots.len() == b.knots.len()
        && a.control_points.len() == b.control_points.len()
        && a.knots.iter().zip(&b.knots).all(|(x, y)| x.to_bits() == y.to_bits())
        && a.control_points.iter().zip(&b.control_points).all(|(x, y)| {
            x.x.to_bits() == y.x.to_bits() && x.y.to_bits() == y.y.to_bits() && x.z.to_bits() == y.z.to_bits() && x.w.to_bits() == y.w.to_bits()
        })
}

/// The operand edges that are HALVES OF ONE EDGE: a vertex put on an edge
/// (`apply_edge_splits`) leaves two edges carrying the parent's curve bit for
/// bit over adjoining windows. Such a vertex is no corner of the operand; it
/// is a split the imprint would otherwise record itself, and the junction
/// rules below must read it as one (a boolean on its own split operands reads
/// as on its operands, `boolean_split_operands`). Fixture 31's union on its
/// split operands: the crossing of model edge 16 and pad edge 3 was already a
/// vertex of both, so it "split" no edge, ranked below the tangent landing
/// 9.06e-5 away that split one half, and the merged junction left the
/// crossing (TangentNodeSingularity where the operands' own union builds).
struct SplitChains {
    /// Every edge's chain: the smallest edge id of the chain it belongs to.
    chain_of: HashMap<(u8, u64), (u8, u64)>,
    /// `(chain, point)` of every vertex interior to a chain.
    joints: Vec<((u8, u64), Vec3)>,
    /// The joints at each edge's own ends.
    joint_ends: HashMap<(u8, u64), Vec<Vec3>>,
}

impl SplitChains {
    fn new(edge_curves: &HashMap<(u8, u64), &EdgeRecord>) -> Result<Self, KernelRefusal> {
        let mut keys: Vec<(u8, u64)> = edge_curves.keys().copied().filter(|key| !edge_curves[key].degenerate).collect();
        // By operand, then by window start, so halves of one curve meet in order.
        keys.sort_by(|a, b| a.0.cmp(&b.0).then(edge_curves[a].t0.total_cmp(&edge_curves[b].t0)).then(a.1.cmp(&b.1)));
        let mut chains = SplitChains { chain_of: HashMap::default(), joints: Vec::new(), joint_ends: HashMap::default() };
        // The chains still open, by (operand, bits of the window end reached):
        // (chain, last edge of the chain so far).
        let mut open: HashMap<(u8, u64), Vec<((u8, u64), (u8, u64))>> = HashMap::default();
        for key in keys {
            let edge = edge_curves[&key];
            let continued = open.get_mut(&(key.0, edge.t0.to_bits())).and_then(|waiting| {
                let index = waiting.iter().position(|(_, last)| same_curve_bits(&edge_curves[last].curve, &edge.curve))?;
                Some(waiting.swap_remove(index))
            });
            let chain = match continued {
                Some((chain, last)) => {
                    let point = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Intersect, "evaluate")?;
                    chains.joints.push((chain, point));
                    chains.joint_ends.entry(last).or_default().push(point);
                    chains.joint_ends.entry(key).or_default().push(point);
                    chain
                }
                None => key,
            };
            chains.chain_of.insert(key, chain);
            open.entry((key.0, edge.t1.to_bits())).or_default().push((chain, key));
        }
        // A chain is named by its smallest edge id, whichever half came first.
        let mut smallest: HashMap<(u8, u64), (u8, u64)> = HashMap::default();
        for (key, chain) in &chains.chain_of {
            let entry = smallest.entry(*chain).or_insert(*key);
            if key.1 < entry.1 {
                *entry = *key;
            }
        }
        for chain in chains.chain_of.values_mut() {
            *chain = smallest[chain];
        }
        for (chain, _) in &mut chains.joints {
            *chain = smallest[chain];
        }
        Ok(chains)
    }

    fn chain(&self, key: (u8, u64)) -> (u8, u64) {
        self.chain_of.get(&key).copied().unwrap_or(key)
    }
}

/// The operand edges a junction splits: every `(operand, edge)` with a split
/// parameter whose point lies within the junction's evidence radius.
fn split_edges_of(
    split_points: &[((u8, u64), Vec3)],
    point: Vec3,
    radius: f64,
) -> HashSet<(u8, u64)> {
    split_points
        .iter()
        .filter(|(_, split)| split.sub(point).length() <= radius)
        .map(|(key, _)| *key)
        .collect()
}

/// Give every merged group the position its operand evidence pins (see the
/// POSITION paragraph on [`merge_residual_coincident_vertices`]): a member on
/// an operand VERTEX (within its evidence radius) holds the group at that
/// vertex's exact point — the split operands of the idempotence tests carry
/// every junction as a vertex and record no split for it, and fixture 28's
/// intersect refused with one-use edges when such a group went 3e-4 to the
/// joint point; else the member that splits the most operand edges; a group
/// of face points keeps the joint Newton point already in `merged_point`.
/// Then retract an absorbed member's split on an edge the holder splits too:
/// the member's vertex is gone and the holder's split already cuts that edge
/// at the junction. Returns the number of split parameters retracted.
#[allow(clippy::too_many_arguments)]
fn place_merged_groups_on_operand_edges(
    result: &mut ImprintResultRecord,
    edge_curves: &HashMap<(u8, u64), &EdgeRecord>,
    operand_vertices: &[(u8, Vec3)],
    verts: &[(u64, Vec3)],
    parent: &mut HashMap<u64, u64>,
    merged_point: &mut HashMap<u64, Vec3>,
    tolerance: f64,
    band: f64,
    evidence: &JunctionEvidence,
) -> Result<usize, KernelRefusal> {
    let weld = assembler_weld(tolerance);
    let radius_of = |id: u64| evidence.radii.get(&id).copied().unwrap_or(0.0).max(weld);
    // Splits are read per CHAIN (`SplitChains`): the halves of an edge that
    // already carries a vertex are one edge, and that vertex is a split of
    // it. `own_splits` keeps each recorded split under the edge that holds
    // its parameter, for the retraction.
    let chains = SplitChains::new(edge_curves)?;
    let mut split_points: Vec<((u8, u64), Vec3)> = chains.joints.clone();
    let mut own_splits: Vec<((u8, u64), Vec3)> = Vec::new();
    for record in &result.edge_splits {
        let Some(edge) = edge_curves.get(&(record.operand, record.edge_id)) else {
            continue;
        };
        for &parameter in &record.parameters {
            let point = edge.curve.evaluate(parameter).or_refuse(KernelStage::Intersect, "evaluate")?;
            split_points.push((chains.chain((record.operand, record.edge_id)), point));
            own_splits.push(((record.operand, record.edge_id), point));
        }
    }
    let mut groups: HashMap<u64, Vec<(u64, Vec3)>> = HashMap::default();
    for &(id, point) in verts {
        let rep = junction_find(parent, id);
        groups.entry(rep).or_default().push((id, point));
    }
    let mut groups: Vec<(u64, Vec<(u64, Vec3)>)> = groups.into_iter().collect();
    groups.sort_by_key(|(rep, _)| *rep);
    let debug = std::env::var("BREP_DEBUG_JUNCTION").is_ok();
    // (edge, absorbed point, its radius): the splits to retract.
    let mut retract: Vec<((u8, u64), Vec3, f64)> = Vec::new();
    for (rep, members) in &groups {
        if members.len() < 2 {
            continue;
        }
        // (rank, id, position): ONE operand vertex outranks any count of
        // edges. Two distinct operand vertices inside the group (the pinch
        // of two r = 10.001 rounds on a 20-wide face: the first round's rail
        // end and the cutter's, 1e-3 apart, `the_radius_gate_leaves_pinching_
        // and_star_selections_alone`) pin nothing — dragging the cutter's
        // rail end onto the other operand's vertex broke the tangent-curve
        // contact that the joint point keeps — so the joint point stands.
        let mut pins: Vec<(u64, Vec3)> = Vec::new();
        for &(id, point) in members {
            let radius = radius_of(id);
            for &(_, vertex) in operand_vertices {
                if vertex.sub(point).length() <= radius && !pins.iter().any(|(_, pin)| pin.sub(vertex).length() <= weld) {
                    pins.push((id, vertex));
                }
            }
        }
        if pins.len() > 1 {
            if debug {
                eprintln!("[merge] group v{rep} ({} members) spans {} distinct operand vertices: the joint point stands", members.len(), pins.len());
            }
            continue;
        }
        // The vertex pins a group whose other members carry no operand-edge
        // evidence of their own (fixture 28: a face point 3.9e-4 away) or sit
        // within the band of it. A member that itself splits an operand edge
        // farther away is a distinct crossing (the pinch: the cutter's rail
        // crosses the top edge 2e-3 from the first round's vertex), and the
        // vertex yields to the edge rule below, as before this pin.
        let pin = pins.first().copied().filter(|&(_, vertex)| {
            members.iter().all(|&(id, point)| {
                point.sub(vertex).length() <= band.max(radius_of(id))
                    || split_edges_of(&split_points, point, radius_of(id)).is_empty()
            })
        });
        if pin.is_none() && !pins.is_empty() && debug {
            eprintln!("[merge] group v{rep} ({} members): the operand vertex yields to a member's own edge evidence", members.len());
        }
        let mut holder: Option<(usize, u64, Vec3)> = pin.map(|(id, vertex)| (usize::MAX, id, vertex));
        if holder.is_none() {
            for &(id, point) in members {
                let rank = split_edges_of(&split_points, point, radius_of(id)).len();
                if rank == 0 {
                    continue;
                }
                if holder.is_none_or(|(best, best_id, _)| rank > best || (rank == best && id < best_id)) {
                    holder = Some((rank, id, point));
                }
            }
        }
        let Some((rank, holder_id, position)) = holder else {
            continue;
        };
        if debug {
            let joint = merged_point.get(rep).copied().unwrap_or(position);
            eprintln!(
                "[merge] group v{rep} ({} members) placed on v{holder_id}: {}, {:.3e} from the joint point",
                members.len(),
                if rank == usize::MAX { "an operand vertex".to_string() } else { format!("{rank} operand edge(s)") },
                joint.sub(position).length()
            );
        }
        merged_point.insert(*rep, position);
        let holder_edges = split_edges_of(&split_points, position, radius_of(holder_id));
        for &(id, point) in members {
            let radius = radius_of(id);
            if id == holder_id || point.sub(position).length() <= radius {
                continue;
            }
            for key in split_edges_of(&own_splits, point, radius) {
                if holder_edges.contains(&chains.chain(key)) {
                    retract.push((key, point, radius));
                }
            }
        }
    }
    let mut retracted = 0usize;
    for (key, point, radius) in retract {
        let Some(edge) = edge_curves.get(&key) else {
            continue;
        };
        let Some(record) = result
            .edge_splits
            .iter_mut()
            .find(|record| (record.operand, record.edge_id) == key)
        else {
            continue;
        };
        let before = record.parameters.len();
        record.parameters.retain(|&parameter| {
            edge.curve
                .evaluate(parameter)
                .map(|split| split.sub(point).length() > radius)
                .unwrap_or(true)
        });
        retracted += before - record.parameters.len();
    }
    Ok(retracted)
}

/// Apply a set of junction merges and moves: every vertex takes its
/// union-find representative, a representative with a point in
/// `merged_point` moves there, pieces whose vertex ids changed are
/// repointed, their curve ends snapped and their pcurves rebuilt, and a
/// piece whose two ends collapsed to one vertex and is shorter than
/// `spur_limit` is dropped. With `snap_moved` a piece whose vertex kept its
/// id but moved is snapped as well (the combine's anchors, and since the
/// 2026-11-02 cap-union repair the residual merge's representatives too: a
/// curve end left where the march put it, 2.3e-3 from the vertex the piece
/// names, is the inconsistency the fragmenter refused). A piece whose ends
/// already sit on its vertices is left bit-identical. Shared by the residual
/// merge and the two-radius combine.
#[allow(clippy::too_many_arguments)]
fn relocate_junctions(
    result: &mut ImprintResultRecord,
    parent: &mut HashMap<u64, u64>,
    merged_point: &HashMap<u64, Vec3>,
    face_map: &HashMap<FaceKey, &FaceRecord>,
    charts: &FaceCharts<'_>,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
    spur_limit: f64,
    snap_moved: bool,
) -> Result<(), KernelRefusal> {
    let all_ids: Vec<u64> = parent.keys().copied().collect();
    let mut rep_of: HashMap<u64, u64> = HashMap::default();
    for id in all_ids {
        let rep = junction_find(parent, id);
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
        let repointed = start_rep != piece.start_vertex_id || end_rep != piece.end_vertex_id;
        let moved = repointed
            || (snap_moved && (merged_point.contains_key(&start_rep) || merged_point.contains_key(&end_rep)));
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
            // in `process_curve`). A genuine long closed loop is preserved, and
            // a closed piece whose one vertex merely moved is not a spur.
            if repointed && start_rep == end_rep && curve_length_rough(&piece.curve)? < spur_limit {
                dropped_pieces.insert(piece.id);
                continue;
            }
            let [t0, t1] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
            let start_gap = piece.curve.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(start).length();
            let end_gap = piece.curve.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?.sub(end).length();
            if !repointed && start_gap <= 1e-12 && end_gap <= 1e-12 {
                kept_pieces.push(piece);
                continue;
            }
            piece.curve = snap_curve_ends(&piece.curve, start, end)?;
            if std::env::var("BREP_DEBUG_JUNCTION").is_ok() {
                let [t0, t1] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
                let left_start = piece.curve.evaluate(t0).or_refuse(KernelStage::Intersect, "evaluate")?.sub(start).length();
                let left_end = piece.curve.evaluate(t1).or_refuse(KernelStage::Intersect, "evaluate")?.sub(end).length();
                eprintln!(
                    "[relocate] piece {} v{start_rep}->v{end_rep} gaps {start_gap:.3e}/{end_gap:.3e} -> {left_start:.3e}/{left_end:.3e}{}",
                    piece.id,
                    if left_start > 1e-9 || left_end > 1e-9 { " NOT SNAPPED (beyond the snap cap)" } else { "" }
                );
            }
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
                    emitted_pcurves.contains(&(piece.id, key)),
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
    Ok(())
}

