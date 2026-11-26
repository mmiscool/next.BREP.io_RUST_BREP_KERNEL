use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// `BREP_OFFSET_TANGENT_RIM_CUTS` (default on; `=0` restores): a carrier whose
/// trim was rebuilt over its whole domain is cut along the rims it shares
/// TANGENTIALLY with retained neighbours (`imprint_tangent_rim_cuts`).
pub(super) fn tangent_rim_cuts_enabled() -> bool {
    switch!("BREP_OFFSET_TANGENT_RIM_CUTS", true)
}

/// TANGENT RIM CUTS: the offsets of two faces that meet tangentially along a
/// source edge stay joined edge to edge along that edge's image. While both
/// carriers keep the image of their source trim, the image is a boundary edge
/// of each and boundary synchronization (or the coincident-rim weld) joins
/// them. A carrier REBUILT over its whole domain (a curved face with a reflex
/// junction, `rebuild_reflex_rim_carriers`; a plane with a curved outline and a
/// reflex junction, `full_rectangle_face`) no longer carries that image, and
/// nothing puts it back: the pair imprint of two tangent surfaces has no
/// transverse section, so the rebuilt sheet runs on past the rim, beside its
/// neighbour's skin, to the next carrier that does cross it. A revolved
/// cylinder beside its own fillet ran on to the next offset plane that way,
/// and the two parallel rims were read as a sharp seam.
///
/// The cut is the neighbour's own rim: the boundary edge of the neighbour's
/// carrier that images the tangent source edge, imprinted on the rebuilt
/// carrier as a section piece (its pcurve fitted on that carrier, or written
/// down on a plane). The neighbour is not cut: the curve is already its
/// boundary. The rim's reach past the source edge is whatever the neighbour's
/// own trim was grown by, which is what the neighbour needs to meet the same
/// reflex faces.
///
/// The pair is still marched: tangency along one rim does not exclude a
/// transverse section elsewhere (a fillet's grown strip crossing the plane it
/// is tangent to). But along the rim itself two tangent carriers have no
/// transverse section, and what a march finds there is the noise of their
/// fits (a wobbling branch beside the rim, 2.6e-3 off it past the source
/// vertex, that left sliver fragments and two vertices where the rim meets
/// the next carrier): a marched piece of the pair that lies along the rim is
/// dropped in favour of the rim (`drop_marched_rim_noise`).
pub(super) struct TangentRimCut {
    /// The carrier to cut and the carrier whose rim cuts it.
    pub(super) cut: usize,
    pub(super) rim: usize,
    source_edge_id: u64,
    edge: EdgeRecord,
    pcurve: crate::NurbsCurve,
}

/// The cuts to make, read from the carriers before they are imprinted.
pub(super) fn plan_tangent_rim_cuts(
    carriers: &[Carrier],
    source_faces: &[&FaceRecord],
    smooth_pairs: &HashSet<(usize, usize)>,
    tolerance: f64,
    scale: f64,
) -> Result<Vec<TangentRimCut>, KernelRefusal> {
    let source_by_id = source_faces.iter().map(|face| (face.id, *face)).collect::<HashMap<_, _>>();
    // How far a rim may stand off the rebuilt carrier and still be its own
    // curve: the band the pipeline reads a piece as a carrier's boundary by,
    // widened to the carrier fits' own bar.
    let band = (2e-5f64.max(scale * 1e-7)).max(tolerance * 100.0);
    let mut cuts = Vec::new();
    for first in 0..carriers.len() {
        for second in 0..carriers.len() {
            if first == second
                || !matches!(carriers[first].kind, OffsetFaceRole::Offset)
                || !matches!(carriers[second].kind, OffsetFaceRole::Offset)
                || carriers[first].source_face_id == carriers[second].source_face_id
                || smooth_pairs.contains(&(first.min(second), first.max(second)))
            {
                continue;
            }
            // `first` is the carrier to cut, `second` the neighbour whose rim cuts it.
            let (Some(cut_source), Some(rim_source)) = (
                source_by_id.get(&carriers[first].source_face_id),
                source_by_id.get(&carriers[second].source_face_id),
            ) else {
                continue;
            };
            let cut_face = &carriers[first].solid.shells[0].faces[0];
            let rim_face = &carriers[second].solid.shells[0].faces[0];
            let used_edges = cut_face
                .loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .map(|coedge| coedge.edge_id)
                .collect::<HashSet<_>>();
            for (loop_index, rim_loop) in rim_source.loops.iter().enumerate() {
                for (use_index, rim_use) in rim_loop.coedges.iter().enumerate() {
                    let Some(cut_use) = cut_source
                        .loops
                        .iter()
                        .flat_map(|loop_record| &loop_record.coedges)
                        .find(|coedge| coedge.edge_id == rim_use.edge_id)
                    else {
                        continue;
                    };
                    if !uses_are_tangent(rim_source, rim_use, cut_source, cut_use)? {
                        continue;
                    }
                    let Some(carrier_index) = carrier_use_imaging(rim_face, rim_source, loop_index, use_index)? else {
                        continue;
                    };
                    let Some(rim_coedge) = rim_face.loops.get(loop_index).and_then(|record| record.coedges.get(carrier_index)) else {
                        continue;
                    };
                    let Some(rim) = carriers[second].solid.edges.iter().find(|edge| edge.id == rim_coedge.edge_id) else {
                        continue;
                    };
                    if rim.degenerate {
                        continue;
                    }
                    // The rim must be a curve of the carrier to cut ...
                    if !edge_on_surface(rim, &cut_face.surface, band)? {
                        os_debug!(
                            "tangent rim cut ({first},{second}) edge {}: the neighbour's rim is off the carrier to cut",
                            rim_use.edge_id
                        );
                        continue;
                    }
                    // ... that the carrier does not already carry as its own boundary.
                    let rim_geometry = FragmentEdgeGeometry { curve: rim.curve.clone(), t0: rim.t0, t1: rim.t1 };
                    let mut carried = false;
                    for edge in &carriers[first].solid.edges {
                        // Only an edge the trim USES: a rebuilt carrier keeps
                        // the records of the trim it replaced.
                        if edge.degenerate || !used_edges.contains(&edge.id) {
                            continue;
                        }
                        let own = FragmentEdgeGeometry { curve: edge.curve.clone(), t0: edge.t0, t1: edge.t1 };
                        if fragment_edge_lies_on(&rim_geometry, &own, band)? || fragment_edge_lies_on(&own, &rim_geometry, band)? {
                            carried = true;
                            break;
                        }
                    }
                    if carried {
                        continue;
                    }
                    let pcurve = match boundary_pcurve(rim, true, &cut_face.surface, band) {
                        Ok(Some(pcurve)) => pcurve,
                        Ok(None) => continue,
                        Err(refusal) => {
                            os_debug!(
                                "tangent rim cut ({first},{second}) edge {}: no pcurve on the carrier to cut ({})",
                                rim_use.edge_id,
                                refusal.message
                            );
                            continue;
                        }
                    };
                    os_debug!(
                        "tangent rim cut planned: carrier {first} (src {}) along carrier {second}'s (src {}) image of edge {}",
                        carriers[first].source_face_id,
                        carriers[second].source_face_id,
                        rim_use.edge_id,
                    );
                    cuts.push(TangentRimCut { cut: first, rim: second, source_edge_id: rim_use.edge_id, edge: rim.clone(), pcurve });
                }
            }
        }
    }
    Ok(cuts)
}

/// Drop the marched pieces of each cut's own pair that lie along its rim,
/// whole, within a fiftieth of the distance (the wobble measured was a
/// hundredth: 2.6e-3 at 0.25). The band scales with the shell and has no
/// fixed floor: a thin shell's genuine branch a wall thickness from the rim
/// is far outside it.
pub(super) fn drop_marched_rim_noise(
    imprint: &mut ImprintResultRecord,
    cuts: &[TangentRimCut],
    distance: f64,
) -> Result<usize, KernelRefusal> {
    let band = 0.02 * distance.abs();
    let mut dropped = Vec::new();
    for piece in &imprint.pieces {
        if !(piece.t1 > piece.t0) {
            continue;
        }
        let geometry = FragmentEdgeGeometry { curve: piece.curve.clone(), t0: piece.t0, t1: piece.t1 };
        for planned in cuts {
            let pair = [planned.cut as u8, planned.rim as u8];
            if !piece.support_faces.iter().all(|key| pair.contains(&key.operand))
                || piece.support_faces[0].operand == piece.support_faces[1].operand
            {
                continue;
            }
            let rim = FragmentEdgeGeometry { curve: planned.edge.curve.clone(), t0: planned.edge.t0, t1: planned.edge.t1 };
            if fragment_edge_lies_on(&geometry, &rim, band)? {
                os_debug!(
                    "tangent rim cut: marched piece {} of pair ({},{}) lies along the rim of edge {} and is dropped",
                    piece.id, planned.cut, planned.rim, planned.source_edge_id,
                );
                dropped.push(piece.id);
                break;
            }
        }
    }
    for entry in &mut imprint.by_face {
        entry.piece_ids.retain(|id| !dropped.contains(id));
    }
    Ok(dropped.len())
}

/// Mint the planned cuts as section pieces on the carriers they cut; returns
/// the pieces' ids.
pub(super) fn imprint_tangent_rim_cuts(
    imprint: &mut ImprintResultRecord,
    carriers: &[Carrier],
    cuts: Vec<TangentRimCut>,
    next_piece_id: &mut u64,
    next_vertex_id: &mut u64,
    distance: f64,
    tolerance: f64,
    scale: f64,
) -> Result<HashSet<u64>, KernelRefusal> {
    let mut minted = HashSet::default();
    // Every rim planned on one carrier, as planned: a rim that ends on another
    // rim's end (a fillet running into the next fillet) is anchored there.
    let all_rims = cuts.iter().map(|planned| (planned.cut, planned.source_edge_id, planned.edge.clone())).collect::<Vec<_>>();
    for mut planned in cuts {
        let cut_face = &carriers[planned.cut].solid.shells[0].faces[0];
        let sibling_rims = all_rims
            .iter()
            .filter(|(cut, source_edge_id, _)| *cut == planned.cut && *source_edge_id != planned.source_edge_id)
            .map(|(_, _, edge)| edge)
            .collect::<Vec<_>>();
        trim_rim_tails(&mut planned, imprint, &sibling_rims, cut_face, distance, tolerance, scale)?;
        let rim = &planned.edge;
        let start = rim.curve.evaluate(rim.t0).or_refuse(KernelStage::Refine, "evaluate")?;
        let end = rim.curve.evaluate(rim.t1).or_refuse(KernelStage::Refine, "evaluate")?;
        let start_vertex_id = merge_vertex(&mut imprint.vertices, start, next_vertex_id, tolerance, scale);
        let end_vertex_id = merge_vertex(&mut imprint.vertices, end, next_vertex_id, tolerance, scale);
        let id = *next_piece_id;
        *next_piece_id += 1;
        minted.insert(id);
        let cut_key = FaceKey { operand: planned.cut as u8, face_id: carriers[planned.cut].solid.shells[0].faces[0].id };
        let rim_key = FaceKey { operand: planned.rim as u8, face_id: carriers[planned.rim].solid.shells[0].faces[0].id };
        imprint.pieces.push(ImprintPieceRecord {
            id,
            curve: rim.curve.clone(),
            t0: rim.t0,
            t1: rim.t1,
            start_vertex_id,
            end_vertex_id,
            pcurves: vec![FacePcurve { operand: cut_key.operand, face_id: cut_key.face_id, pcurve: planned.pcurve }],
            support_faces: [cut_key, rim_key],
            shared_edge: None,
                certified_ruling_side: None,
        });
        if let Some(entry) = imprint
            .by_face
            .iter_mut()
            .find(|entry| entry.operand == cut_key.operand && entry.face_id == cut_key.face_id)
        {
            entry.piece_ids.push(id);
        } else {
            imprint.by_face.push(FaceImprints { operand: cut_key.operand, face_id: cut_key.face_id, piece_ids: vec![id] });
        }
        os_debug!(
            "tangent rim cut: carrier {} (src {}) cut along carrier {}'s (src {}) image of edge {} \
             ({:.4},{:.4},{:.4})..({:.4},{:.4},{:.4}), piece {id}",
            planned.cut,
            carriers[planned.cut].source_face_id,
            planned.rim,
            carriers[planned.rim].source_face_id,
            planned.source_edge_id,
            start.x, start.y, start.z, end.x, end.y, end.z,
        );
    }
    Ok(minted)
}

/// End a rim cut ON the last section it crosses before each of its ends.
///
/// The neighbour's rim runs past the carrier that bounds both skins (the
/// reflex wall) by whatever its own trim was grown by beyond the meeting
/// line: half the distance, with the usual 1.5 headroom. The face
/// arrangement prunes such an overshoot only when it is longer than its
/// junction radius (0.15% of the face's parameter diagonal); a shorter one
/// is read as a loose cut end, paired with the next loose end for bridging
/// and refused ("cut junction bridge refused", every distance under about a
/// tenth here). A rim that ends on the crossing section is a T-junction on
/// that section's run, which the arrangement anchors. Only a crossing within
/// one distance of a FREE end is taken: an end that already lies on another
/// section (the next fillet's rim) has no tail, and farther in the rim is real
/// boundary (a rim trimmed back to a crossing 0.73 from its end at d = 0.45
/// lost that much of the seam).
fn trim_rim_tails(
    planned: &mut TangentRimCut,
    imprint: &ImprintResultRecord,
    sibling_rims: &[&EdgeRecord],
    cut_face: &FaceRecord,
    distance: f64,
    tolerance: f64,
    scale: f64,
) -> Result<(), KernelRefusal> {
    let band = (2e-5f64.max(scale * 1e-7)).max(tolerance * 100.0);
    let Some(entry) = imprint
        .by_face
        .iter()
        .find(|entry| entry.operand == planned.cut as u8 && entry.face_id == cut_face.id)
    else {
        return Ok(());
    };
    let others = imprint
        .pieces
        .iter()
        .filter(|piece| entry.piece_ids.contains(&piece.id) && piece.t1 > piece.t0)
        .map(|piece| (&piece.curve, piece.t0, piece.t1))
        .chain(sibling_rims.iter().map(|edge| (&edge.curve, edge.t0, edge.t1)))
        .collect::<Vec<_>>();
    if others.is_empty() {
        return Ok(());
    }
    let rim = planned.edge.clone();
    let point_at = |t: f64| rim.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate");
    // Distance from the rim at `t` to the nearest of the other sections (each within its own range).
    let gap_at = |t: f64| -> Result<f64, KernelRefusal> {
        let point = point_at(t)?;
        let mut nearest = f64::INFINITY;
        for (curve, t0, t1) in &others {
            let projection = crate::project_point_to_curve(curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
            let u = projection.u.clamp(*t0, *t1);
            let on = curve.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
            nearest = nearest.min(on.sub(point).length());
        }
        Ok(nearest)
    };
    let reach = distance.abs();
    let span = rim.t1 - rim.t0;
    let mut ends = [rim.t0, rim.t1];
    for (slot, (end, inward)) in [(rim.t0, 1.0), (rim.t1, -1.0)].into_iter().enumerate() {
        let end_point = point_at(end)?;
        // An end that already lies on another section is anchored: no tail.
        if gap_at(end)? <= band {
            continue;
        }
        // The parameter window whose far side is `reach` from the end.
        let mut window = span / 1024.0;
        while window < span * 0.5 && point_at(end + inward * window)?.sub(end_point).length() < reach {
            window *= 2.0;
        }
        let window = window.min(span * 0.5);
        // The crossing nearest the end: the first local minimum of the gap, walking in from the end.
        const STEPS: usize = 96;
        let mut samples = Vec::with_capacity(STEPS + 1);
        for step in 0..=STEPS {
            let t = end + inward * window * step as f64 / STEPS as f64;
            samples.push((t, gap_at(t)?));
        }
        let Some(best) = (1..STEPS).find(|&index| {
            samples[index].1 <= samples[index - 1].1 && samples[index].1 <= samples[index + 1].1 && samples[index].1 <= window_gap(reach)
        }) else {
            continue;
        };
        let (mut low, mut high) = (samples[best - 1].0, samples[best + 1].0);
        for _ in 0..80 {
            let third = (high - low) / 3.0;
            let (a, b) = (low + third, high - third);
            if gap_at(a)? <= gap_at(b)? {
                high = b;
            } else {
                low = a;
            }
        }
        let at = 0.5 * (low + high);
        let tail = point_at(at)?.sub(end_point).length();
        if gap_at(at)? > band || tail <= band || tail > reach {
            continue;
        }
        ends[slot] = at;
    }
    if ends == [rim.t0, rim.t1] || !(ends[1] > ends[0]) {
        return Ok(());
    }
    let mut trimmed = rim.clone();
    trimmed.t0 = ends[0];
    trimmed.t1 = ends[1];
    let Ok(Some(pcurve)) = boundary_pcurve(&trimmed, true, &cut_face.surface, band) else {
        return Ok(());
    };
    os_debug!(
        "tangent rim cut on carrier {} along edge {}: ended on its crossing sections, t [{:.9}, {:.9}] -> [{:.9}, {:.9}]",
        planned.cut, planned.source_edge_id, rim.t0, rim.t1, ends[0], ends[1],
    );
    planned.edge = trimmed;
    planned.pcurve = pcurve;
    Ok(())
}

/// A sampled gap worth refining: within a tenth of the window's reach.
fn window_gap(reach: f64) -> f64 {
    0.1 * reach
}
