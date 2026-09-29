use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

// ====================================================================
// Edge-preserving blends (4.9.7): ball tangent to F1, resting on a
// boundary edge of F2 (closed edges; auto-triggered when the standard
// support exits F2's trim)
// ====================================================================

pub(in crate::blend) struct KeepStation {
    uv1: [f64; 2],
    pub(in crate::blend) s: f64,
    p1: Vec3,
    q: Vec3,
    center: Vec3,
    weight: f64,
    apex: Vec3,
}

/// Fit the v=1 rim against the preserved edge's parameter, since chord-based
/// surface u need not vary affinely with it. Callers validate the edge span.
pub(in crate::blend) fn fit_preserved_pcurve(
    stations: &[KeepStation],
    parameters: &[f64],
    edge_domain: [f64; 2],
    u_domain: [f64; 2],
) -> Result<NurbsCurve, KernelRefusal> {
    let [s_first, s_last] = edge_domain;
    let s_span = s_last - s_first;
    let [u_start, u_end] = u_domain;
    let mut pairs: Vec<(f64, f64)> = stations
        .iter()
        .enumerate()
        .map(|(index, station)| {
            let edge_param = (station.s - s_first) / s_span;
            let fit_u = u_start + (u_end - u_start) * parameters[index];
            (edge_param, fit_u)
        })
        .collect();
    if s_span < 0.0 {
        pairs.reverse();
    }
    let params: Vec<f64> = pairs.iter().map(|(p, _)| *p).collect();
    let points: Vec<Vec4> = pairs
        .iter()
        .map(|(_, u)| Vec4::from_point(Vec3::new(*u, 1.0, 0.0), 1.0))
        .collect();
    fit::interpolate_homogeneous(&points, FIT_DEGREE, &params).or_refuse(KernelStage::Refine, "interpolate_homogeneous")
}

/// Residual for the edge-preserving tangency: c = s1(u,v) + ρ1·n1 must
/// sit at distance ρ from the boundary curve point q(s), the contact must
/// be a rest (c−q ⟂ q'), and the center rides the section plane.
fn keep_residual(
    first: &NurbsSurface,
    boundary: &NurbsCurve,
    rho1: f64,
    radius: f64,
    x: [f64; 3],
    section_point: Vec3,
    section_tangent: Vec3,
) -> Result<([f64; 3], Vec3, Vec3, Vec3), KernelRefusal> {
    // Rest the ball on F2's boundary curve while remaining tangent to F1.
    let offset = blend_offset(first).at(x[0], x[1], rho1).or_refuse(KernelStage::Refine, "at")?;
    let (p1, center) = (offset.source, offset.point);
    let derivatives = boundary.derivatives_extended(x[2], 1).or_refuse(KernelStage::Refine, "derivatives_extended")?;
    let q = derivatives[0];
    let q_tangent = derivatives[1];
    let offset = center.sub(q);
    Ok((
        [
            offset.dot(offset) - radius * radius,
            offset.dot(q_tangent),
            center.sub(section_point).dot(section_tangent),
        ],
        p1,
        q,
        center,
    ))
}

#[allow(clippy::too_many_arguments)]
fn solve_keep_station(
    first: &NurbsSurface,
    boundary: &NurbsCurve,
    rho1: f64,
    radius: f64,
    seed: [f64; 3],
    section_point: Vec3,
    section_tangent: Vec3,
    scale: f64,
) -> Result<[f64; 3], KernelRefusal> {
    let mut x = seed;
    let tolerance = 1e-10 * (1.0 + scale * scale);
    for _ in 0..NEWTON_ITERATIONS {
        let (residual, ..) = keep_residual(
            first,
            boundary,
            rho1,
            radius,
            x,
            section_point,
            section_tangent,
        )?;
        let error = residual.iter().map(|value| value.abs()).fold(0.0, f64::max);
        if error <= tolerance {
            return Ok(x);
        }
        let mut jacobian = [[0.0f64; 3]; 3];
        let step = 1e-7;
        for column in 0..3 {
            let mut probe = x;
            probe[column] += step;
            let (probed, ..) = keep_residual(
                first,
                boundary,
                rho1,
                radius,
                probe,
                section_point,
                section_tangent,
            )?;
            for row in 0..3 {
                jacobian[row][column] = (probed[row] - residual[row]) / step;
            }
        }
        let delta = fit::solve_small::<3>(jacobian, residual, 3)
            .map_err(|error| format!("blend: keep-edge Newton is singular ({error})")).or_refuse(KernelStage::Refine, "solve_small")?;
        for (value, correction) in x.iter_mut().zip(delta) {
            *value -= correction;
        }
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "keep_tangency", "blend: keep-edge tangency did not converge"))
}

/// Build one keep-march station (contact/apex/weight) from a converged
/// tangency solution `x = [u, v, s]`.  Shared by the closed and open keep
/// marches.
fn keep_station_from(
    first: &NurbsSurface,
    boundary: &NurbsCurve,
    rho1: f64,
    radius: f64,
    x: [f64; 3],
    section_point: Vec3,
    section_tangent: Vec3,
) -> Result<KeepStation, KernelRefusal> {
    let (_, p1, q, center) = keep_residual(
        first,
        boundary,
        rho1,
        radius,
        x,
        section_point,
        section_tangent,
    )?;
    let n1 = raw_normal(first, x[0], x[1])?;
    let toward1 = n1.scale(rho1).scale(1.0 / radius);
    let toward_q = center.sub(q).scale(1.0 / radius);
    let cos_alpha = toward1.dot(toward_q);
    let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
    if weight <= 1e-6 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "keep_sections", "blend: keep-edge sections are degenerate"));
    }
    let apex = apex_point(p1, n1, q, toward_q, center)?;
    Ok(KeepStation {
        uv1: [x[0], x[1]],
        s: x[2],
        p1,
        q,
        center,
        weight,
        apex,
    })
}

/// Closed-edge edge-preserving blend: F2 is consumed between the blended
/// edge and its preserved far boundary; the blend face runs from the F1
/// support curve to the preserved edge itself.
pub(in crate::blend) fn blend_closed_edge_keep(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius: f64,
    rho1: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    // Fillet (rational arc) and chamfer (ruled chord) share the same rolling-
    // ball march and the same surgery topology (cr on F1 at v=0, the preserved
    // edge at v=1); only the fitted blend surface differs.
    // The preserved boundary: the consumed face's edge that shares NEITHER
    // endpoint with the blended edge (its "far" side).  This picks the same
    // edge whether it is CLOSED (a full rim whose single seam vertex differs
    // from the blended edge's) or OPEN (its two distinct end vertices differ
    // from the blended edge's endpoints); the transverse/seam edges of F2 are
    // excluded because they touch a blended-edge endpoint.
    let blended_ends = [edge.start_vertex_id, edge.end_vertex_id];
    let mut preserved = None;
    for loop_record in &second.face.loops {
        for coedge in &loop_record.coedges {
            if coedge.edge_id == edge.id {
                continue;
            }
            let candidate = solid
                .edges
                .iter()
                .find(|candidate| candidate.id == coedge.edge_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "missing_edge", "blend: consumed face references a missing edge"))?;
            if blended_ends.contains(&candidate.start_vertex_id)
                || blended_ends.contains(&candidate.end_vertex_id)
            {
                continue;
            }
            if preserved.is_some() && preserved != Some(coedge.edge_id) {
                return Err(KernelRefusal::ill_posed(KernelStage::Classify, "preserved_edge", "blend: ambiguous preserved boundary edge"));
            }
            preserved = Some(coedge.edge_id);
        }
    }
    let preserved_id = preserved.ok_or(KernelRefusal::internal(KernelStage::Classify, "preserved_edge", "blend: no preserved boundary edge on the consumed face"))?;
    let preserved_edge = solid
        .edges
        .iter()
        .find(|candidate| candidate.id == preserved_id)
        .ok_or(KernelRefusal::internal(KernelStage::Classify, "preserved_edge", "blend: preserved edge missing"))?;
    // An OPEN preserved edge (two distinct end vertices) takes the clamped
    // open path below; a CLOSED one keeps the wrapped-overlap closed path.
    let open_preserved = preserved_edge.start_vertex_id != preserved_edge.end_vertex_id;
    let surface1 = &first.face.surface;
    let boundary = &preserved_edge.curve;
    let span = edge.t1 - edge.t0;
    let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius)?;
    crate::report_scale_migration("blend_march_keep", scale, || {
        edge.curve
            .evaluate(edge.t0)
            .unwrap_or_default()
            .length()
            .max(1.0)
    });

    // March, anchored so station 0's rest point is the preserved edge's
    // own vertex (its curve start) — the blend seam then reuses it.
    let anchor_s = preserved_edge.t0;
    let mut stations: Vec<KeepStation> = Vec::with_capacity(STATIONS + 1);
    // Seed the tangency point INSIDE the tangent face: on the rim itself
    // the center sits straight above the rest point and the Newton
    // Jacobian is singular (the residual gradient along the face vanishes at
    // the symmetric configuration).  Seed at the edge MIDPOINT for the open
    // case — the endpoints are corners of F1 where the in-plane "into face"
    // probe is ambiguous — and at the start for the closed anchor.
    let seed_t = if open_preserved {
        (edge.t0 + edge.t1) * 0.5
    } else {
        edge.t0
    };
    let seed_s = if open_preserved {
        (preserved_edge.t0 + preserved_edge.t1) * 0.5
    } else {
        anchor_s
    };
    let seed0 = {
        let rim_point = edge.curve.evaluate(seed_t).or_refuse(KernelStage::Refine, "evaluate")?;
        let rim_tangent = edge
            .curve
            .unit_tangent(seed_t, edge.t0, edge.t1)
            .map_err(|error| format!("blend keep: edge {}: {error}", edge.id)).or_refuse(KernelStage::Refine, "unit_tangent")?;
        let into1 = crate::fillet::into_face_direction(
            first.face,
            rim_point,
            rim_tangent,
            rim_point,
            rim_tangent,
            (radius * 0.25).max(1e-4),
        )?;
        let nudged = rim_point.add(into1.scale(radius * 0.5));
        let projection = crate::project_point_to_surface(&first.face.surface, nudged).or_refuse(KernelStage::Refine, "project_point_to_surface")?;
        [projection.u, projection.v, seed_s]
    };
    // The section plane at `t` — the extended point and the edge's unit
    // tangent, the one-sided limit where the parameterization is stationary.
    let section_frame = |t: f64| -> Result<(Vec3, Vec3), KernelRefusal> {
        edge.curve
            .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
            .map_err(|error| format!("blend keep: edge {}: {error}", edge.id)).or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")
    };
    if open_preserved {
        // OPEN keep march: run along the WHOLE blended edge [t0, t1]
        // (clamped, not periodic), solving the rest point s freely per
        // section.  The blended and preserved edges span the same sections,
        // so the two end sections land on the preserved edge's own end
        // vertices — no wrap, no closure.  March MID-OUT (seed the best-
        // conditioned middle station, then each end from its neighbour) so
        // the corner sections inherit a converged seed.
        let mid_index = STATIONS / 2;
        let section_at = |index: usize| -> Result<(Vec3, Vec3), KernelRefusal> {
            let t = edge.t0 + span * index as f64 / STATIONS as f64;
            section_frame(t)
        };
        let mut solutions = vec![[0.0f64; 3]; STATIONS + 1];
        let (mid_point, mid_tangent) = section_at(mid_index)?;
        solutions[mid_index] = solve_keep_station(
            surface1,
            boundary,
            rho1,
            radius,
            seed0,
            mid_point,
            mid_tangent,
            scale,
        )?;
        for index in (0..mid_index).rev() {
            let (section_point, section_tangent) = section_at(index)?;
            solutions[index] = solve_keep_station(
                surface1,
                boundary,
                rho1,
                radius,
                solutions[index + 1],
                section_point,
                section_tangent,
                scale,
            )?;
        }
        for index in mid_index + 1..=STATIONS {
            let (section_point, section_tangent) = section_at(index)?;
            solutions[index] = solve_keep_station(
                surface1,
                boundary,
                rho1,
                radius,
                solutions[index - 1],
                section_point,
                section_tangent,
                scale,
            )?;
        }
        for index in 0..=STATIONS {
            let (section_point, section_tangent) = section_at(index)?;
            stations.push(keep_station_from(
                surface1,
                boundary,
                rho1,
                radius,
                solutions[index],
                section_point,
                section_tangent,
            )?);
        }
    } else {
        // CLOSED keep: anchor station 0's rest point to the preserved edge's
        // own vertex (its curve start) so the blend seam reuses it, then
        // march a full lap and check closure.
        // Find the edge parameter whose section contains the anchored rest
        // point: Newton on t with s locked.
        let t_start;
        {
            let mut x = [edge.t0, seed0[0], seed0[1]];
            let residual_at = |x: &[f64; 3]| -> Result<[f64; 3], KernelRefusal> {
                let (section_point, section_tangent) = section_frame(x[0])?;
                let (residual, ..) = keep_residual(
                    surface1,
                    boundary,
                    rho1,
                    radius,
                    [x[1], x[2], anchor_s],
                    section_point,
                    section_tangent,
                )?;
                Ok(residual)
            };
            let tolerance = 1e-10 * (1.0 + scale * scale);
            let mut converged = false;
            for iteration in 0..NEWTON_ITERATIONS {
                let residual = residual_at(&x)?;
                if std::env::var("BREP_DEBUG_KEEP").is_ok() {
                    eprintln!(
                        "keep anchor it{iteration}: x=({:.6},{:.6},{:.6}) r=({:.3e},{:.3e},{:.3e})",
                        x[0], x[1], x[2], residual[0], residual[1], residual[2]
                    );
                }
                if residual.iter().map(|value| value.abs()).fold(0.0, f64::max) <= tolerance {
                    converged = true;
                    break;
                }
                let mut jacobian = [[0.0f64; 3]; 3];
                let step = 1e-7;
                for column in 0..3 {
                    let mut probe = x;
                    probe[column] += step;
                    let probed = residual_at(&probe)?;
                    for row in 0..3 {
                        jacobian[row][column] = (probed[row] - residual[row]) / step;
                    }
                }
                let delta = fit::solve_small::<3>(jacobian, residual, 3)
                    .map_err(|error| format!("blend: keep-edge anchor is singular ({error})")).or_refuse(KernelStage::Refine, "solve_small")?;
                for (value, correction) in x.iter_mut().zip(delta) {
                    *value -= correction;
                }
            }
            if !converged {
                return Err(KernelRefusal::non_convergence(KernelStage::Refine, "keep_anchored_start", "blend: keep-edge anchored start did not converge"));
            }
            t_start = x[0];
        }
        let mut previous = [seed0[0], seed0[1], anchor_s];
        for index in 0..=STATIONS {
            let t = t_start + span * index as f64 / STATIONS as f64;
            let (section_point, section_tangent) = section_frame(t)?;
            let x = solve_keep_station(
                surface1,
                boundary,
                rho1,
                radius,
                previous,
                section_point,
                section_tangent,
                scale,
            )?;
            stations.push(keep_station_from(
                surface1,
                boundary,
                rho1,
                radius,
                x,
                section_point,
                section_tangent,
            )?);
            previous = x;
        }
        let closure = stations[STATIONS].p1.sub(stations[0].p1).length();
        if closure > 1e-6 * scale {
            return Err(KernelRefusal::non_convergence(KernelStage::Refine, "keep_march_closure", "blend: keep-edge march did not close"));
        }
    }

    // Fit rows: cr on F1 (with pcurve) and the weighted apex; the v=1 row
    // is the preserved edge itself, sampled at the marched s values so all
    // rows share the chord parameterisation.
    let full_stations: Vec<Station> = stations
        .iter()
        .map(|station| Station {
            uv1: station.uv1,
            uv2: [station.s, 0.0],
            p1: station.p1,
            p2: station.q,
            center: station.center,
            weight: station.weight,
            apex: station.apex,
        })
        .collect();
    let parameters = station_parameters(&full_stations);
    if open_preserved {
        // Clamped open fit + dedicated open surgery (two transverse seams,
        // F1 boundary trims, F2 consumed, preserved edge reused).
        let rows = fit_open_rows(&full_stations, &parameters, chamfer, None, None)?;
        return build_keep_open_surgery(
            solid,
            edge,
            first,
            second,
            preserved_edge,
            &stations,
            &parameters,
            rows,
            name,
        );
    }
    let rows = fit_closed_rows(&full_stations, &parameters, chamfer)?;

    // ---- surgery ------------------------------------------------------
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);
    let [u_start, u_end] = rows.u_domain;
    let vertex1_id = take_id();
    result.vertices.push(VertexRecord {
        id: vertex1_id,
        point: rows.cr.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?,
    });
    let cr_edge_id = take_id();
    result.edges.push(EdgeRecord {
        id: cr_edge_id,
        curve: rows.cr.clone(),
        t0: u_start,
        t1: u_end,
        start_vertex_id: vertex1_id,
        end_vertex_id: vertex1_id,
        degenerate: false,
        name: None,
    });
    // Blend seam: from the cr seam vertex to the preserved edge's vertex.
    let seam_curve = rows.surface.iso_curve_u(u_start).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [seam_t0, seam_t1] = seam_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let blend_seam_id = take_id();
    result.edges.push(EdgeRecord {
        id: blend_seam_id,
        curve: seam_curve,
        t0: seam_t0,
        t1: seam_t1,
        start_vertex_id: vertex1_id,
        end_vertex_id: preserved_edge.start_vertex_id,
        degenerate: false,
        name: None,
    });

    // F1: swap the blended edge's coedge for cr.
    {
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == first.face.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during keep surgery"))?;
        let loop_record = &mut face.loops[first.loop_index];
        let position = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.edge_id == edge.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_coedge", "blend: edge coedge lost during keep surgery"))?;
        let old_forward = loop_record.coedges[position].forward;
        loop_record.coedges[position] = CoedgeRecord {
            id: loop_record.coedges[position].id,
            edge_id: cr_edge_id,
            forward: old_forward,
            pcurve: if old_forward {
                rows.cr_pcurve.clone()
            } else {
                rows.cr_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
            },
        };
        if loop_record.coedges.len() > 1 {
            return Err(KernelRefusal::unsupported(KernelStage::Classify, "keep_seam_face", "blend: keep-edge with a seam-structured F1 is unsupported"));
        }
    }

    // Blend face: manifold pairing with F1's use of cr, preserved edge on
    // the v=1 rim (the preserved edge's second use replaces the consumed
    // face's).
    let first_use_forward = {
        let face = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == first.face.id)
            .unwrap();
        face.loops[first.loop_index]
            .coedges
            .iter()
            .find(|coedge| coedge.edge_id == edge.id)
            .unwrap()
            .forward
    };
    // The consumed face's use of the preserved edge tells the blend's
    // required sense there (the blend inherits it).
    let consumed_use_forward = second
        .face
        .loops
        .iter()
        .flat_map(|loop_record| &loop_record.coedges)
        .find(|coedge| coedge.edge_id == preserved_id)
        .map(|coedge| coedge.forward)
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "preserved_edge", "blend: consumed face does not use the preserved edge"))?;
    // The blend REPLACES the consumed face's use of the preserved edge —
    // manifold pairing keeps that use's sense.
    let blend_preserved_forward = consumed_use_forward;
    let blend_cr_forward = !first_use_forward;
    let s_first = stations[0].s;
    let s_last = stations[STATIONS].s;
    let s_span = s_last - s_first;
    if s_span.abs() < 1e-9 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "preserved_edge_parameter", "blend: preserved-edge parameterisation collapsed"));
    }
    let preserved_pcurve_t = fit_preserved_pcurve(
        &stations,
        &parameters,
        [s_first, s_last],
        [u_start, u_end],
    )?;
    // Traversal direction of the preserved coedge in u: forward traversal
    // walks s (= edge t) increasing, which maps to u increasing iff the
    // march's s grew with u.
    let preserved_up_in_u = blend_preserved_forward == (s_span > 0.0);
    // The loop layout must place the v=1 rim in that direction AND the
    // v=0 rim opposite F1's use; orientable inputs make these agree.
    if preserved_up_in_u != !blend_cr_forward {
        return Err(KernelRefusal::internal(KernelStage::Sew, "keep_orientation", "blend: keep-edge orientations are inconsistent"));
    }
    let preserved_pcurve = if blend_preserved_forward {
        preserved_pcurve_t.clone()
    } else {
        preserved_pcurve_t.reversed().or_refuse(KernelStage::Refine, "reversed")?
    };
    let loop_id = take_id();
    let coedges = if blend_cr_forward {
        vec![
            CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_end, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: blend_seam_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_end, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: preserved_id,
                forward: blend_preserved_forward,
                pcurve: preserved_pcurve,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: blend_seam_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_start, 1.0, u_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
            },
        ]
    } else {
        vec![
            CoedgeRecord {
                id: take_id(),
                edge_id: preserved_id,
                forward: blend_preserved_forward,
                pcurve: preserved_pcurve,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: blend_seam_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_end, 1.0, u_end, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: cr_edge_id,
                forward: false,
                pcurve: crate::sweep_topology::parameter_line(u_end, 0.0, u_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
            },
            CoedgeRecord {
                id: take_id(),
                edge_id: blend_seam_id,
                forward: true,
                pcurve: crate::sweep_topology::parameter_line(u_start, 0.0, u_start, 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
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
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during keep surgery"))?;
    result.shells[shell_index].faces.push(blend_face);

    // Delete the consumed face, its exclusive seam edges, the blended
    // edge, and the orphaned rim vertex.
    let consumed_id = second.face.id;
    for shell in &mut result.shells {
        shell.faces.retain(|face| face.id != consumed_id);
    }
    let old_vertex = edge.start_vertex_id;
    result.edges.retain(|candidate| candidate.id != edge.id);
    // Seam edges of the consumed face that no other face uses.
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
    let old_vertex_still_used = result.edges.iter().any(|candidate| {
        candidate.start_vertex_id == old_vertex || candidate.end_vertex_id == old_vertex
    });
    if !old_vertex_still_used {
        result
            .vertices
            .retain(|candidate| candidate.id != old_vertex);
    }
    Ok(result)
}

