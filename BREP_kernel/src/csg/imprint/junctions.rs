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
    match std::env::var("BREP_JUNCTION_COMBINE").as_deref() {
        Ok("0") | Ok("off") => JunctionCombineMode::Off,
        Ok("check") => JunctionCombineMode::Check,
        Ok(_) => JunctionCombineMode::On,
        Err(_) => {
            if options.two_radius_junctions {
                JunctionCombineMode::On
            } else {
                JunctionCombineMode::Off
            }
        }
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
pub(super) fn canonicalize_imprint_junctions(
    result: &mut ImprintResultRecord,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    charts: &FaceCharts<'_>,
    tolerance: f64,
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
    evidence: &JunctionEvidence,
    combine: JunctionCombineMode,
) -> Result<(), KernelRefusal> {
    let face_map = faces(solid_a, 0)
        .into_iter()
        .chain(faces(solid_b, 1))
        .map(|face| (face.key(), face.face))
        .collect::<HashMap<_, _>>();
    let (mut incident_faces, mut incident_points) = junction_incidence(result)?;
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
            emitted_pcurves,
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
    // Freeze an intrinsic reference before any vertex is polished. A world
    // origin gives identical junctions different acceptance budgets after a
    // rigid placement. Relative coordinates also avoid summing large offsets.
    let reference = junction_reference(result.vertices.iter().map(|vertex| vertex.point));
    if std::env::var("BREP_DEBUG_CANONICAL_JUNCTIONS").is_ok() {
        eprintln!("JUNCTION-REFERENCE point={reference:?} vertices={}", result.vertices.len());
    }
    let mut canonical_vertices = result
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
    emitted_pcurves: &HashSet<(u64, FaceKey)>,
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
    relocate_junctions(result, &mut parent, &merged_point, face_map, charts, emitted_pcurves, (4.0 * sep_cap).max(1e-3), false)?;
    Ok(true)
}

/// Apply a set of junction merges and moves: every vertex takes its
/// union-find representative, a representative with a point in
/// `merged_point` moves there, pieces whose vertex ids changed are
/// repointed, their curve ends snapped and their pcurves rebuilt, and a
/// piece whose two ends collapsed to one vertex and is shorter than
/// `spur_limit` is dropped. With `snap_moved` a piece whose vertex kept its
/// id but moved is snapped as well (the combine's anchors); the residual
/// merge keeps its historical behaviour of leaving those pieces' ends where
/// the march put them. Shared by the residual merge and the two-radius
/// combine.
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
        let moved = start_rep != piece.start_vertex_id
            || end_rep != piece.end_vertex_id
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
            // in `process_curve`). A genuine long closed loop is preserved.
            if start_rep == end_rep && curve_length_rough(&piece.curve)? < spur_limit {
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

