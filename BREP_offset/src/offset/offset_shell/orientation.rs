use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
#[allow(unused_imports)]
pub use crate::orientation::{flip_all_faces, flip_face_orientation, flip_shell_faces, orient_open_solid_faces};

/// Re-analyticize a fitted-polyline rim as the EXACT v-isoline of its owner
/// surface, rewriting the owner's rim coedge to the straight parameter-space
/// line pinned through its loop neighbours. Returns the updated edge record
/// and the owner's new `forward` flag, or `None` (with the solid untouched)
/// when the rim is not a circle riding an owner isoline — callers keep the
/// faithful polyline in that case. The carrier polylines' chord sag is the
/// dominant wall-volume error this removes.
pub(super) fn upgrade_rim_to_owner_isoline(
    solid: &mut BrepSolid,
    edge_id: u64,
    owner_shell: usize,
    owner_face: usize,
    tolerance: f64,
) -> Result<Option<(EdgeRecord, bool)>, KernelRefusal> {
    let Some(edge) = solid.edges.iter().find(|edge| edge.id == edge_id).cloned() else {
        return Ok(None);
    };
    if fit_rim_circle(&edge, tolerance)?.is_none() {
        return Ok(None);
    }
    let owner = &solid.shells[owner_shell].faces[owner_face];
    let Some((prev_end, next_start)) =
        crate::topology::coedge_neighbor_endpoints(&owner.loops, edge_id)
    else {
        return Ok(None);
    };
    if (prev_end.y - next_start.y).abs() > tolerance.max(1e-4) {
        return Ok(None);
    }
    let Ok(iso) = owner.surface.iso_curve_v(prev_end.y) else {
        return Ok(None);
    };
    let [iso_t0, iso_t1] = iso.domain().or_refuse(KernelStage::Sew, "domain")?;
    // The isoline must actually be this rim (same closed locus).
    let mut worst = 0.0f64;
    for sample in 0..=32 {
        let fraction = sample as f64 / 32.0;
        let on_iso = iso.evaluate(iso_t0 + (iso_t1 - iso_t0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        worst = worst.max(crate::project_point_to_curve(&edge.curve, on_iso).or_refuse(KernelStage::Sew, "project_point_to_curve")?.distance);
    }
    if worst > tolerance.max(1e-3) {
        return Ok(None);
    }
    let owner_pcurve = crate::make_line(
        Vec3::new(prev_end.x, prev_end.y, 0.0),
        Vec3::new(next_start.x, next_start.y, 0.0),
    ).or_refuse(KernelStage::Sew, "make_line")?;
    let owner_forward_new = isoline_forward(&owner_pcurve, &owner.surface, &iso)?;
    let updated = EdgeRecord {
        curve: iso,
        t0: iso_t0,
        t1: iso_t1,
        ..edge.clone()
    };
    let owner = &mut solid.shells[owner_shell].faces[owner_face];
    for coedge in owner
        .loops
        .iter_mut()
        .flat_map(|loop_record| &mut loop_record.coedges)
        .filter(|coedge| coedge.edge_id == edge_id)
    {
        coedge.pcurve = owner_pcurve.clone();
        coedge.forward = owner_forward_new;
    }
    if let Some(record) = solid.edges.iter_mut().find(|record| record.id == edge_id) {
        record.curve = updated.curve.clone();
        record.t0 = updated.t0;
        record.t1 = updated.t1;
    }
    Ok(Some((updated, owner_forward_new)))
}

/// Sew a one-use closed rim into a containing planar face as an inner loop.
/// This repairs closed-circle/open-chain mismatches left by intersection; rims
/// outside the face's plane and point-like pole placeholders do not qualify.
pub(super) fn weld_coplanar_orphan_rims(solid: &mut BrepSolid, tolerance: f64) -> Result<usize, KernelRefusal> {
    let use_counts = crate::topology::edge_use_counts(solid);
    let orphan_ids = solid
        .edges
        .iter()
        .filter(|edge| {
            use_counts.get(&edge.id).copied().unwrap_or(0) == 1
                && edge.start_vertex_id == edge.end_vertex_id
        })
        .map(|edge| edge.id)
        .collect::<Vec<_>>();
    let mut welded = 0usize;
    let mut shell_unions: Vec<(usize, usize)> = Vec::new();
    for edge_id in orphan_ids {
        let edge = solid
            .edges
            .iter()
            .find(|edge| edge.id == edge_id)
            .cloned()
            .ok_or_else(|| "offset_shell: orphan rim vanished".to_string()).or_refuse(KernelStage::Sew, "offset_shell_orphan_rim_vanished")?;
        // Reject true point placeholders (poles): a rim must sweep measurably
        // away from its seam vertex.
        let anchor = edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Sew, "evaluate")?;
        let sweeps = [0.25, 0.5, 0.75].into_iter().any(|fraction| {
            edge.curve
                .evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction)
                .map(|point| point.sub(anchor).length() > 10.0 * tolerance)
                .unwrap_or(false)
        });
        if !sweeps {
            continue;
        }
        // The single existing use fixes the manifold direction: the wall must
        // trace the shared rim the opposite way.
        let (owner_shell, owner_face, owner_forward) = solid
            .shells
            .iter()
            .enumerate()
            .find_map(|(shell_index, shell)| {
                shell
                    .faces
                    .iter()
                    .enumerate()
                    .find_map(|(face_index, face)| {
                        face.loops
                            .iter()
                            .flat_map(|loop_record| &loop_record.coedges)
                            .find(|coedge| coedge.edge_id == edge_id)
                            .map(|coedge| (shell_index, face_index, coedge.forward))
                    })
            })
            .ok_or_else(|| "offset_shell: orphan rim has no use".to_string()).or_refuse(KernelStage::Sew, "offset_shell_orphan_rim_has_no_use")?;
        let mut wall_forward = !owner_forward;
        let rim_mid = edge.curve.evaluate((edge.t0 + edge.t1) * 0.5).or_refuse(KernelStage::Sew, "evaluate")?;
        // Locate a planar face that both carries the rim and strictly contains
        // it — the opening cap the rim should hole.
        let mut target: Option<(usize, usize)> = None;
        'search: for (shell_index, shell) in solid.shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                if !face.surface.is_affine().unwrap_or(false) {
                    continue;
                }
                if face
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .any(|coedge| coedge.edge_id == edge_id)
                {
                    continue;
                }
                if !edge_on_surface(&edge, &face.surface, tolerance)? {
                    continue;
                }
                let projection = project_point_to_surface(&face.surface, rim_mid).or_refuse(KernelStage::Sew, "project_point_to_surface")?;
                let uv = Vec2 {
                    x: projection.u,
                    y: projection.v,
                };
                if projection.distance > tolerance
                    || parameter_point_in_face(face, uv, tolerance).or_refuse(KernelStage::Sew, "parameter_point_in_face")? == PolygonClass::Outside
                {
                    continue;
                }
                target = Some((shell_index, face_index));
                break 'search;
            }
        }
        let Some((shell_index, face_index)) = target else {
            continue;
        };
        // Exactness upgrade (best effort, full fallback): when the rim is a
        // circle that runs along a v-isoline of its owner surface, swap the
        // carrier's fitted polyline for the owner's EXACT isoline and pin the
        // owner pcurve to the straight parameter-space line through its loop
        // neighbours — the same re-analyticization the ruled weld performs.
        // The polyline's chord sag is the only thing standing between a
        // cylinder-wall shell and an exact annulus wall volume. Non-circular
        // rims (a prism hole's rectangle) keep the faithful polyline weld.
        let edge =
            match upgrade_rim_to_owner_isoline(solid, edge_id, owner_shell, owner_face, tolerance)?
            {
                Some((updated, owner_forward_new)) => {
                    wall_forward = !owner_forward_new;
                    updated
                }
                None => edge,
            };
        let Some(pcurve) = boundary_pcurve(
            &edge,
            wall_forward,
            &solid.shells[shell_index].faces[face_index].surface,
            tolerance,
        )?
        else {
            continue;
        };
        let next_loop_id = solid.shells[shell_index].faces[face_index]
            .loops
            .iter()
            .map(|loop_record| loop_record.id)
            .max()
            .unwrap_or(0)
            + 1;
        let next_coedge_id = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.id)
            .max()
            .unwrap_or(0)
            + 1;
        solid.shells[shell_index].faces[face_index]
            .loops
            .push(LoopRecord {
                id: next_loop_id,
                coedges: vec![CoedgeRecord {
                    id: next_coedge_id,
                    edge_id,
                    forward: wall_forward,
                    pcurve,
                }],
            });
        // The rim is now shared real topology, not a face-local placeholder:
        // clear the historical degenerate marking the assembler left on the
        // unwelded closed edge so Euler/degenerate-use validation see the 1-cell.
        if let Some(record) = solid.edges.iter_mut().find(|record| record.id == edge_id) {
            record.degenerate = false;
        }
        if owner_shell != shell_index {
            shell_unions.push((owner_shell, shell_index));
        }
        welded += 1;
    }
    // A welded rim joins the offset skin to its opening cap; if they lived in
    // separate assembled shell containers, they are now one connected shell.
    merge_connected_shells(solid, shell_unions);
    Ok(welded)
}
