use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Consume F2 and reuse its far open boundary as the blend's v=1 rim.
/// The clamped patch joins cr on F1 to the preserved edge through two end
/// seams. At each end, trim F1's boundary to cr, consume F2's boundary,
/// and splice the transverse iso-curve into the adjacent end face.
#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn build_keep_open_surgery(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    preserved_edge: &EdgeRecord,
    stations: &[KeepStation],
    parameters: &[f64],
    rows: FittedRows,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // The lossy exit for the closed-keep caller (`edge/keep.rs`), which still
    // returns text and enters here with a bare `return`; the typed body is
    // `keep_open_surgery`. Retired with that caller.
    keep_open_surgery(
        solid,
        edge,
        first,
        second,
        preserved_edge,
        stations,
        parameters,
        rows,
        name,
    )
    
}

#[allow(clippy::too_many_arguments)]
pub(in crate::blend) fn keep_open_surgery(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    preserved_edge: &EdgeRecord,
    stations: &[KeepStation],
    parameters: &[f64],
    rows: FittedRows,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);
    let [u_start, u_end] = rows.u_domain;

    // Pair each march end (station 0 / station N) with the preserved edge's
    // own end vertex — station 0's rest s sits at whichever preserved endpoint
    // parameter it is nearest.
    let s_first = stations[0].s;
    let s_last = stations[stations.len() - 1].s;
    let s_span = s_last - s_first;
    if s_span.abs() < 1e-9 {
        return Err(KernelRefusal::internal(
            KernelStage::Collect,
            "preserved_span",
            "blend: preserved-edge parameterisation collapsed",
        ));
    }
    let (start_pv, finish_pv) =
        if (s_first - preserved_edge.t0).abs() <= (s_first - preserved_edge.t1).abs() {
            (preserved_edge.start_vertex_id, preserved_edge.end_vertex_id)
        } else {
            (preserved_edge.end_vertex_id, preserved_edge.start_vertex_id)
        };

    // Gather both ends: the F1 boundary edge (trimmed), the F2 boundary edge
    // (consumed), the end face across the corner, the cr crossing, and the
    // transverse end iso-curve with its blend/end pcurves.
    struct KeepEnd {
        corner: u64,
        preserved_vertex: u64,
        first_edge_id: u64,
        first_edge_parameter: f64,
        second_edge_id: u64,
        end_face_id: u64,
        transverse: NurbsCurve,
        transverse_end_pcurve: NurbsCurve,
    }
    let mut ends: Vec<KeepEnd> = Vec::with_capacity(2);
    // A crossing that only resolves on a looser rung of the ladder builds what
    // the tight rung refused, so the answer is gated on being a valid solid
    // (see `SpokeCrossings`).
    let mut crossings = SpokeCrossings::default();
    for (corner, at_start, preserved_vertex, cs_parameter) in [
        (edge.start_vertex_id, true, start_pv, u_start),
        (edge.end_vertex_id, false, finish_pv, u_end),
    ] {
        let first_edge_id = boundary_edge_at_vertex(solid, first.face, corner, edge.id)?;
        let second_edge_id = boundary_edge_at_vertex(solid, second.face, corner, edge.id)?;
        if first_edge_id == second_edge_id {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "shared_end_edge",
                "blend: keep-edge mates share their end boundary edge",
            ));
        }
        let end_face = end_face_id(
            solid,
            first.face.id,
            second.face.id,
            first_edge_id,
            second_edge_id,
        )?;
        let first_boundary = solid
            .edges
            .iter()
            .find(|candidate| candidate.id == first_edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "end_boundary_edge",
                    "blend: keep end boundary edge missing",
                )
            })?;
        let first_boundary_coedge = first
            .face
            .loops
            .iter()
            .flat_map(|loop_record| &loop_record.coedges)
            .find(|coedge| coedge.edge_id == first_edge_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "end_boundary_coedge",
                    "blend: F1 does not use its end boundary edge",
                )
            })?;
        let (cr_parameter, first_edge_parameter, escalated) =
            match support_crossing(solid, &rows.cr, first_boundary, at_start) {
                Ok((s, t, _, escalated)) => (s, t, escalated),
                Err(_) => support_crossing_uv(
                    solid,
                    &rows.cr,
                    &rows.cr_pcurve,
                    first_boundary,
                    first_boundary_coedge,
                    at_start,
                )?,
            };
        crossings.note(escalated);
        let end_record = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == end_face)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Classify,
                    "end_face",
                    "blend: keep end face missing",
                )
            })?;
        let (transverse, _blend_pcurve, end_pcurve) = transverse_curve(
            &rows,
            end_record,
            cr_parameter.clamp(u_start, u_end),
            cs_parameter,
        )?;
        ends.push(KeepEnd {
            corner,
            preserved_vertex,
            first_edge_id,
            first_edge_parameter,
            second_edge_id,
            end_face_id: end_face,
            transverse,
            transverse_end_pcurve: end_pcurve,
        });
    }
    if ends.len() != 2 {
        return Err(KernelRefusal::internal(
            KernelStage::Sew,
            "end_count",
            "blend: keep-open surgery needs exactly two ends",
        ));
    }
    let mut ends = ends.into_iter();
    let start_end = ends.next().unwrap();
    let finish_end = ends.next().unwrap();

    // The cr row already spans corner-to-corner (the open march ran the whole
    // blended edge), so its endpoints ARE the two rim vertices — no trim.
    let w1a = take_id();
    result.vertices.push(VertexRecord {
        id: w1a,
        point: rows
            .cr
            .evaluate(u_start)
            .or_refuse(KernelStage::Sew, "evaluate")?,
    });
    let w1b = take_id();
    result.vertices.push(VertexRecord {
        id: w1b,
        point: rows
            .cr
            .evaluate(u_end)
            .or_refuse(KernelStage::Sew, "evaluate")?,
    });
    let cr_edge_id = take_id();
    result.edges.push(EdgeRecord {
        id: cr_edge_id,
        curve: rows.cr.clone(),
        t0: u_start,
        t1: u_end,
        start_vertex_id: w1a,
        end_vertex_id: w1b,
        degenerate: false,
        name: None,
    });
    let transverse_a_id = take_id();
    let ta_domain = start_end
        .transverse
        .domain()
        .or_refuse(KernelStage::Sew, "domain")?;
    result.edges.push(EdgeRecord {
        id: transverse_a_id,
        curve: start_end.transverse.clone(),
        t0: ta_domain[0],
        t1: ta_domain[1],
        start_vertex_id: w1a,
        end_vertex_id: start_end.preserved_vertex,
        degenerate: false,
        name: None,
    });
    let transverse_b_id = take_id();
    let tb_domain = finish_end
        .transverse
        .domain()
        .or_refuse(KernelStage::Sew, "domain")?;
    result.edges.push(EdgeRecord {
        id: transverse_b_id,
        curve: finish_end.transverse.clone(),
        t0: tb_domain[0],
        t1: tb_domain[1],
        start_vertex_id: w1b,
        end_vertex_id: finish_end.preserved_vertex,
        degenerate: false,
        name: None,
    });

    // F1: swap the blended edge's coedge for cr (keeping F1's other coedges).
    let first_use_forward;
    {
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == first.face.id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "mate_face",
                    "blend: mate face lost during keep surgery",
                )
            })?;
        let loop_record = &mut face.loops[first.loop_index];
        let position = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.edge_id == edge.id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "edge_coedge",
                    "blend: edge coedge lost during keep surgery",
                )
            })?;
        let old_forward = loop_record.coedges[position].forward;
        first_use_forward = old_forward;
        loop_record.coedges[position] = CoedgeRecord {
            id: loop_record.coedges[position].id,
            edge_id: cr_edge_id,
            forward: old_forward,
            pcurve: if old_forward {
                rows.cr_pcurve.clone()
            } else {
                rows.cr_pcurve
                    .reversed()
                    .or_refuse(KernelStage::Sew, "reversed")?
            },
        };
    }

    // Locate each end face's corner (the boundary coedge pair meeting at the
    // old corner vertex, across any pole between them) BEFORE trimming, keyed
    // on stable coedge ids.
    let mut end_plans = Vec::with_capacity(2);
    let mut consumed_poles: Vec<u64> = Vec::new();
    for (end, transverse_id) in [
        (&start_end, transverse_a_id),
        (&finish_end, transverse_b_id),
    ] {
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == end.end_face_id)
            .ok_or_else(|| {
                KernelRefusal::internal(
                    KernelStage::Sew,
                    "end_face_lost",
                    "blend: end face lost during keep surgery",
                )
            })?;
        let Some(located) = locate_end_corner(
            &result,
            face,
            end.corner,
            end.first_edge_id,
            end.second_edge_id,
        ) else {
            return Err(KernelRefusal::unsupported(
                KernelStage::Classify,
                "end_corner",
                "blend: keep end-face corner (adjacent boundary coedges) not found",
            ));
        };
        for pole in &located.pole_edge_ids {
            if !consumed_poles.contains(pole) {
                consumed_poles.push(*pole);
            }
        }
        end_plans.push((
            end.end_face_id,
            transverse_id,
            end.transverse_end_pcurve.clone(),
            located,
        ));
    }

    // Trim the F1 boundary edge at each end (moving the old corner vertex to
    // the cr rim vertex).  The F2 boundary edges are consumed, not trimmed.
    trim_edge_at(
        &mut result,
        start_end.first_edge_id,
        start_end.first_edge_parameter,
        start_end.corner,
        w1a,
    )?;
    trim_edge_at(
        &mut result,
        finish_end.first_edge_id,
        finish_end.first_edge_parameter,
        finish_end.corner,
        w1b,
    )?;

    // Rebuild each end face's corner: drop the consumed F2 boundary coedge,
    // keep the (trimmed) F1 boundary coedge, and splice in the transverse.
    // The F1 side (first_edge) is never consumed; the F2 side (second_edge)
    // always is.
    for (end_face, transverse_id, transverse_end_pcurve, located) in end_plans {
        let forward = located.x_is_first;
        let pcurve = if forward {
            transverse_end_pcurve
        } else {
            transverse_end_pcurve
                .reversed()
                .or_refuse(KernelStage::Sew, "reversed")?
        };
        // first_edge (F1) is kept; second_edge (F2) is consumed.
        let x_consumed = !located.x_is_first;
        let y_consumed = located.x_is_first;
        let transverse_coedge = CoedgeRecord {
            id: take_id(),
            edge_id: transverse_id,
            forward,
            pcurve,
        };
        splice_end_corner(
            &mut result,
            end_face,
            &located,
            transverse_coedge,
            x_consumed,
            y_consumed,
        )?;
    }
    // The apexes the transverse curves cut off leave with them.
    result
        .edges
        .retain(|candidate| !consumed_poles.contains(&candidate.id));

    // Preserved edge pcurve on the blend + orientation (as in the closed keep:
    // the blend replaces F2's use of the preserved edge, and its use of cr is
    // opposite F1's).
    let consumed_use_forward = second
        .face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .find(|coedge| coedge.edge_id == preserved_edge.id)
        .map(|coedge| coedge.forward)
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "preserved_use",
                "blend: consumed face does not use the preserved edge",
            )
        })?;
    let blend_preserved_forward = consumed_use_forward;
    let blend_cr_forward = !first_use_forward;
    let preserved_pcurve_t = fit_preserved_pcurve(
        stations,
        parameters,
        [s_first, s_last],
        [u_start, u_end],
    )?;
    let preserved_pcurve = if blend_preserved_forward {
        preserved_pcurve_t.clone()
    } else {
        preserved_pcurve_t
            .reversed()
            .or_refuse(KernelStage::Sew, "reversed")?
    };

    // Blend face loop: cr (v=0), preserved (v=1), and the two transverse end
    // seams — same layout as the closed keep but with distinct end seams.
    let loop_id = take_id();
    let coedges = if blend_cr_forward {
        vec![
            CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_end, 0.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: transverse_b_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_end, 1.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: preserved_edge.id,
                forward: blend_preserved_forward,
                pcurve: preserved_pcurve,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: transverse_a_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_start, 1.0, u_start, 0.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
        ]
    } else {
        vec![
            CoedgeRecord {
                id: take_id(),
                edge_id: preserved_edge.id,
                forward: blend_preserved_forward,
                pcurve: preserved_pcurve,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: transverse_b_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_end, 1.0, u_end, 0.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_start, 0.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: transverse_a_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_start, 1.0)
                    .or_refuse(KernelStage::Sew, "parameter_line")?,
            },
        ]
    };
    let same_sense = blend_cr_forward;
    let blend_face = FaceRecord {
        id: take_id(),
        surface: rows.surface,
        same_sense,
        loops: vec![LoopRecord {
            id: loop_id,
            coedges,
        }],
        name: name.map(|value| value.to_string()),
    };
    let shell_index = result
        .shells
        .iter()
        .position(|shell| shell.faces.iter().any(|face| face.id == first.face.id))
        .ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Sew,
                "mate_shell",
                "blend: mate shell lost during keep surgery",
            )
        })?;
    result.shells[shell_index].faces.push(blend_face);

    // Delete the consumed face; then drop every edge no longer referenced by
    // any coedge (the blended edge and F2's now-orphaned boundary edges) and
    // prune orphaned vertices.
    let consumed_id = second.face.id;
    for shell in &mut result.shells {
        shell.faces.retain(|face| face.id != consumed_id);
    }
    let used_ids: rustc_hash::FxHashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    result
        .edges
        .retain(|candidate| used_ids.contains(&candidate.id));
    let used_vertices: rustc_hash::FxHashSet<u64> = result
        .edges
        .iter()
        .flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|candidate| used_vertices.contains(&candidate.id));
    crossings
        .gate(result)
}
