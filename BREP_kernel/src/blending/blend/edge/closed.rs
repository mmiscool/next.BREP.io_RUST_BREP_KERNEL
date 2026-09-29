use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// General closed-edge rolling-ball fillet or chamfer by direct §6.9
/// topology surgery.
pub fn blend_closed_edge(
    solid: &BrepSolid,
    edge_id: u64,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    if !(radius > 0.0) || !radius.is_finite() {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius", "blend: radius must be positive"));
    }
    blend_closed_edge_impl(solid, edge_id, &|_| radius, chamfer, name)
}

/// Variable-radius blend (4.9.5): radius stops as (edge fraction, radius)
/// pairs, linearly interpolated along the edge parameter and clamped at
/// the ends.  Closed edges must supply matching first/last radii.
pub fn blend_edge_variable(
    solid: &BrepSolid,
    edge_id: u64,
    radii: &[(f64, f64)],
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    if radii.is_empty() || radii.iter().any(|(_, radius)| !(*radius > 0.0)) {
        return Err(KernelRefusal::input(KernelStage::Collect, "radius_stop", "blend: every radius stop must be positive"));
    }
    let mut stops = radii.to_vec();
    stops.sort_by(|a, b| a.0.total_cmp(&b.0));
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("blend: edge {edge_id} not found")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    let closed = edge.start_vertex_id == edge.end_vertex_id;
    if closed && (stops[0].1 - stops[stops.len() - 1].1).abs() > 1e-12 {
        return Err(KernelRefusal::input(KernelStage::Collect, "closed_edge_radii", "blend: closed edges need equal first/last radii"));
    }
    // CONSTANT stops must DEGENERATE to the exact constant-radius
    // machinery (§6.9 exact cylinder cutters where the mates allow them,
    // the same general march otherwise).  The general march's fitted
    // NURBS is geometrically exact for a constant radius, but downstream
    // booleans treat it as a generic surface: marched SSI curves against
    // a tangent mate drift O(√ε) along the tangency (measured 1.2e-3 on
    // a 10-box chain miter), while the exact representation intersects
    // analytically — this is what keeps chain-corner miters composable.
    let first_radius = stops[0].1;
    if stops
        .iter()
        .all(|(_, radius)| (radius - first_radius).abs() <= 1e-12 * (1.0 + first_radius.abs()))
    {
        return if chamfer {
            crate::fillet::chamfer_edge(solid, edge_id, first_radius, name)
        } else {
            crate::fillet::fillet_edge(solid, edge_id, first_radius, name)
        };
    }
    let (t0, t1) = (edge.t0, edge.t1);
    let radius_at = move |t: f64| -> f64 {
        let raw = (t - t0) / (t1 - t0);
        // Closed edges WRAP (the anchored march may start mid-edge and run
        // past t1 once around — clamping misplaced mid-profile stops
        // there); open edges CLAMP flat past the ends, which is only the
        // marched overshoot region beyond the trims.  The blend-end
        // stations are pinned exactly at t0/t1 by the open march, so the
        // profile kink the clamp creates sits ON an interpolation node and
        // the fitted rows still pass through the closed-form end stations.
        let fraction = if closed {
            raw.rem_euclid(1.0)
        } else {
            raw.clamp(0.0, 1.0)
        };
        if fraction <= stops[0].0 {
            return stops[0].1;
        }
        for pair in stops.windows(2) {
            if fraction <= pair[1].0 {
                let width = (pair[1].0 - pair[0].0).max(1e-12);
                let local = (fraction - pair[0].0) / width;
                return pair[0].1 + (pair[1].1 - pair[0].1) * local;
            }
        }
        stops[stops.len() - 1].1
    };
    if closed {
        blend_closed_edge_impl(solid, edge_id, &radius_at, chamfer, name)
    } else {
        blend_open_edge_impl(solid, edge_id, &radius_at, chamfer, name)
    }
}

fn blend_closed_edge_impl(
    solid: &BrepSolid,
    edge_id: u64,
    radius_at: &dyn Fn(f64) -> f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("blend: edge {edge_id} not found")).or_refuse(KernelStage::Refine, "ok_or_else")?;
    if edge.start_vertex_id != edge.end_vertex_id {
        return Err(KernelRefusal::input(KernelStage::Collect, "closed_edge", "blend: general path currently requires a CLOSED edge"));
    }
    let (face_a, loop_a, coedge_a) = locate_mate(solid, edge_id, None)?;
    let (face_b, loop_b, coedge_b) = locate_mate(solid, edge_id, Some((face_a.id, loop_a)))?;
    if face_a.id == face_b.id && loop_a == loop_b {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "seam_edge", "blend: edge is used twice by one loop (seam edge?)"));
    }

    // Seam-structured loops force the blend seam onto that carrier's seam
    // meridian; at most one mate may need it.
    let seam_a = face_a.loops[loop_a].coedges.len() > 1;
    let seam_b = face_b.loops[loop_b].coedges.len() > 1;
    let (first_face, first_loop, first_coedge, second_face, second_loop, second_coedge) =
        if seam_a || !seam_b {
            (face_a, loop_a, coedge_a, face_b, loop_b, coedge_b)
        } else {
            (face_b, loop_b, coedge_b, face_a, loop_a, coedge_a)
        };
    let anchored = first_face.loops[first_loop].coedges.len() > 1;
    let second_seam = anchored && second_face.loops[second_loop].coedges.len() > 1;
    let anchor_u = if anchored {
        // Lock onto the carrier's seam meridian: the u-domain start.
        Some(first_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?[0])
    } else {
        None
    };

    // Signed radii from the cross-section seed at the edge midpoint.
    let mid_radius = radius_at(edge.t0 + (edge.t1 - edge.t0) * 0.5);
    let (rho1, rho2) = signed_radii(
        edge,
        first_face,
        first_coedge,
        second_face,
        second_coedge,
        mid_radius,
    )?;

    let first_mate = BlendMate {
        face: first_face,
        coedge: first_coedge,
        loop_index: first_loop,
        rho: rho1,
    };
    let second_mate = BlendMate {
        face: second_face,
        coedge: second_coedge,
        loop_index: second_loop,
        rho: rho2,
    };
    let mid_t = edge.t0 + (edge.t1 - edge.t0) * 0.5;
    // Anchored FIRST, always — everything that already works takes this rung
    // and is unchanged.  An anchored march freezes the first carrier's u on
    // its seam meridian and frees t (`solve_anchored_start`), so it is only
    // solvable when the edge actually reaches that meridian.  A closed edge
    // cut by a plane through a surface of revolution's AXIS is one whole
    // MERIDIAN of that carrier — constant u, a full turn in v — so no station
    // on it can sit at the seam, and the seam may even have been trimmed off
    // the face (the 2026-09-01 reported document: the kept torus was
    // u ∈ [0.1369, 0.8869] while the anchor locked u = 0).  That march reports
    // "anchored seam start did not converge"; the UNANCHORED march solves the
    // same edge exactly, because the edge's own endpoints already sit on the
    // carrier's OTHER (v) seam.  Retrying is what makes this a rung of the
    // existing ladder rather than a predicate with a tolerance to tune: a
    // march that needs its anchor still gets one.
    let marched = match march_stations(edge, &first_mate, &second_mate, radius_at, anchor_u) {
        Ok(stations) => Ok(stations),
        Err(anchored_error) if anchor_u.is_some() => {
            march_stations(edge, &first_mate, &second_mate, radius_at, None).map_err(
                |retry_error| {
                    // A FOLD found by the un-anchored march is the SHAPE's
                    // answer, and it outranks the anchored march's complaint:
                    // reporting the anchor's non-convergence instead would send
                    // a proven-folding centre curve on to the edge-preserving
                    // construction below.
                    if crate::blend::is_wall_fold(&retry_error) {
                        retry_error
                    } else {
                        anchored_error
                    }
                },
            )
        }
        Err(anchored_error) => Err(anchored_error),
    };
    let stations = match marched {
        Ok(stations) => stations,
        // A wall that FOLDS is refused here for the same reason an escaped
        // march is: the edge-preserving construction would build a blend on the
        // same folding centre curve, and if it converged it would return the
        // very surface this refusal exists to stop.  It is not the fallback's
        // question to answer, so the geometry is reported as-is.
        Err(march_error) if crate::blend::is_wall_fold(&march_error) => return Err(march_error),
        Err(march_error) if crate::blend::is_ball_off_carrier(&march_error) => {
            // An ESCAPED march is not a non-convergence: the tangency system
            // had no solution at all, so no fallback can build the blend the
            // rolling ball never made.  Report the escape itself — routing it
            // to the edge-preserving construction below only reports THAT
            // lane's complaint about a face it was never meant to consume
            // ("ambiguous preserved boundary edge" on the 2026-09-10 report).
            return Err(march_error);
        }
        Err(march_error) => {
            // A non-converging march often means the rolling ball falls off
            // one face — try the edge-preserving construction (4.9.7).
            return blend_closed_edge_keep(
                solid,
                edge,
                &first_mate,
                &second_mate,
                radius_at(mid_t),
                rho1,
                chamfer,
                name,
            )
            .or_else(|_| {
                blend_closed_edge_keep(
                    solid,
                    edge,
                    &second_mate,
                    &first_mate,
                    radius_at(mid_t),
                    rho2,
                    chamfer,
                    name,
                )
            })
            .map_err(|keep_error| {
                format!("{march_error}; edge-preserving blend also failed: {keep_error}")
            }).or_refuse(KernelStage::Refine, "map_err");
        }
    };
    // Support-out-of-trim detection (4.9.7 trigger): a tangency track that
    // leaves its face's trimmed region cannot be trimmed there — the far
    // boundary must be PRESERVED instead.
    let support_exits = |face: &FaceRecord, second_side: bool| -> bool {
        // Adaptive marches vary the station count; keep at least the old fixed
        // grid's ~16-probe floor while inheriting extra density where the
        // march refined (which is where an exit would hide).
        let stride = (stations.len() / 16).max(1);
        stations.iter().step_by(stride).any(|station| {
            let mut uv = if second_side {
                station.uv2
            } else {
                station.uv1
            };
            if let Ok((closed_u, closed_v)) = face.surface.closed_directions() {
                if closed_u {
                    if let Ok([low, high]) = face.surface.domain_u() {
                        uv[0] = low + (uv[0] - low).rem_euclid(high - low);
                    }
                }
                if closed_v {
                    if let Ok([low, high]) = face.surface.domain_v() {
                        uv[1] = low + (uv[1] - low).rem_euclid(high - low);
                    }
                }
            }
            crate::parameter_point_in_face(face, crate::Vec2 { x: uv[0], y: uv[1] }, 1e-6)
                .map(|class| class == crate::PolygonClass::Outside)
                .unwrap_or(true)
        })
    };
    if support_exits(second_face, true) {
        return blend_closed_edge_keep(
            solid,
            edge,
            &first_mate,
            &second_mate,
            radius_at(mid_t),
            rho1,
            chamfer,
            name,
        );
    }
    if support_exits(first_face, false) {
        return blend_closed_edge_keep(
            solid,
            edge,
            &second_mate,
            &first_mate,
            radius_at(mid_t),
            rho2,
            chamfer,
            name,
        );
    }
    if std::env::var("BREP_DEBUG_BLEND_MARCH").is_ok() {
        let last = stations.len() - 1;
        for index in [
            0usize,
            1.min(last),
            2.min(last),
            last.saturating_sub(1),
            last,
        ] {
            let station = &stations[index];
            eprintln!(
                "station {index}: uv1=({:.6},{:.6}) uv2=({:.6},{:.6}) p1=({:.6},{:.6},{:.6}) w={:.6}",
                station.uv1[0], station.uv1[1], station.uv2[0], station.uv2[1],
                station.p1.x, station.p1.y, station.p1.z, station.weight,
            );
        }
    }
    let parameters = station_parameters(&stations);
    let rows = match exact_closed_revolution_rows(&stations, &first_mate, &second_mate, chamfer) {
        Some(rows) => rows?,
        None => fit_closed_rows(&stations, &parameters, chamfer)?,
    };

    // Both mates seam-structured: the second support crosses ITS carrier's
    // seam meridian somewhere mid-loop; split cs there so face2's loop can
    // keep its seam-in-one-loop structure.
    let second_split = if second_seam {
        let seam2 = second_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?[0];
        let period2 = {
            let [d0, d1] = second_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            d1 - d0
        };
        // Bracket: unwrapped uv2 track crossing seam2 + k·period.
        let u2_first = stations[0].uv2[0];
        let u2_last = stations[stations.len() - 1].uv2[0];
        let direction = (u2_last - u2_first).signum();
        let mut k = ((u2_first - seam2) / period2).ceil();
        if direction < 0.0 {
            k = ((u2_first - seam2) / period2).floor();
        }
        let target = seam2 + k * period2;
        let inside =
            (target - u2_first) * direction > 1e-6 && (u2_last - target) * direction > 1e-6;
        // ALIGNED seams (both carriers' meridians meet at the shared
        // vertex — the natural coaxial construction): the crossing sits
        // at the march boundary and no mid-loop split is needed; the
        // blend seam vertex already lies on both meridians.
        let aligned = (target - u2_first).abs() <= 1e-6
            || (target - u2_last).abs() <= 1e-6
            || ((u2_first - seam2) / period2).fract().abs() <= 1e-9;
        if !inside && aligned {
            None
        } else if !inside {
            return Err(KernelRefusal::internal(KernelStage::Refine, "seam_crossing_bracket", "blend: second seam crossing not bracketed by the march"));
        } else {
            // Fit-space crossing parameter on the fitted cs (Newton via the
            // fitted pcurve so the split lands exactly where the FITTED track
            // crosses the meridian).
            let [fit_low, fit_high] = rows.u_domain;
            let mut p = fit_low
                + (fit_high - fit_low) * {
                    // Seed from the bracketing stations.
                    let mut seed = 0.5;
                    for pair in 0..stations.len() - 1 {
                        let a = stations[pair].uv2[0];
                        let b = stations[pair + 1].uv2[0];
                        if (target - a) * (target - b) <= 0.0 {
                            let local = (target - a) / (b - a);
                            seed = (parameters[pair]
                                + (parameters[pair + 1] - parameters[pair]) * local)
                                .clamp(0.0, 1.0);
                            break;
                        }
                    }
                    seed
                };
            for _ in 0..NEWTON_ITERATIONS {
                let value = rows.cs_pcurve.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?.x - target;
                if value.abs() <= 1e-12 {
                    break;
                }
                let step = 1e-8;
                let probed = rows.cs_pcurve.evaluate(p + step).or_refuse(KernelStage::Refine, "evaluate")?.x - target;
                let derivative = (probed - value) / step;
                if derivative.abs() <= 1e-14 {
                    return Err(KernelRefusal::non_convergence(KernelStage::Refine, "seam_crossing_newton", "blend: second seam crossing Newton stalled"));
                }
                p -= value / derivative;
            }
            let crossing_v = rows.cs_pcurve.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?.y;
            Some(SecondSeamSplit {
                fit_parameter: p,
                crossing_v,
                period: period2 * direction,
            })
        }
    } else {
        None
    };

    build_surgery(
        solid,
        edge,
        &first_mate,
        &second_mate,
        rows,
        second_split,
        name,
    )
}

/// Where the second support crosses ITS seam-structured carrier's seam
/// meridian (both-seam closed edges).
struct SecondSeamSplit {
    fit_parameter: f64,
    crossing_v: f64,
    /// Signed carrier period travelled by the unwrapped pcurve track.
    period: f64,
}

/// Replace the blended edge in both mating loops, trim the seam edge of an
/// anchored closed carrier, and insert the blend face.
fn build_surgery(
    solid: &BrepSolid,
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    rows: FittedRows,
    second_split: Option<SecondSeamSplit>,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);

    let [u_start, u_end] = rows.u_domain;
    let cr_start = rows.cr.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let cs_start = rows.cs.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let vertex1_id = take_id();
    let vertex2_id = take_id();
    result.vertices.push(VertexRecord {
        id: vertex1_id,
        point: cr_start,
    });
    result.vertices.push(VertexRecord {
        id: vertex2_id,
        point: cs_start,
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
    // Second support: whole closed edge, or two pieces split where it
    // crosses ITS carrier's seam meridian (both-seam closed edges).  Each
    // piece records (edge id, fit-space window, forward pcurve).
    let mut cs_pieces: Vec<(u64, [f64; 2], NurbsCurve)> = Vec::new();
    let mut second_seam_vertex = vertex2_id;
    let mut second_seam_point = cs_start;
    let mut second_crossing_v = 0.0;
    if let Some(split) = &second_split {
        let p = split.fit_parameter;
        second_crossing_v = split.crossing_v;
        let (cs_a, cs_b) = rows.cs.split(p).or_refuse(KernelStage::Refine, "split")?;
        let (pc_a, pc_b) = rows.cs_pcurve.split(p).or_refuse(KernelStage::Refine, "split")?;
        // Bring the wrapped second piece back into the carrier's domain.
        let mut pc_b = pc_b;
        for point in pc_b.control_points.iter_mut() {
            point.x -= split.period * point.w;
        }
        let w2s = take_id();
        second_seam_vertex = w2s;
        second_seam_point = rows.cs.evaluate(p).or_refuse(KernelStage::Refine, "evaluate")?;
        result.vertices.push(VertexRecord {
            id: w2s,
            point: second_seam_point,
        });
        let edge_a = take_id();
        result.edges.push(EdgeRecord {
            id: edge_a,
            curve: cs_a,
            t0: u_start,
            t1: p,
            start_vertex_id: vertex2_id,
            end_vertex_id: w2s,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((edge_a, [u_start, p], pc_a));
        let edge_b = take_id();
        result.edges.push(EdgeRecord {
            id: edge_b,
            curve: cs_b,
            t0: p,
            t1: u_end,
            start_vertex_id: w2s,
            end_vertex_id: vertex2_id,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((edge_b, [p, u_end], pc_b));
    } else {
        let cs_edge_id = take_id();
        result.edges.push(EdgeRecord {
            id: cs_edge_id,
            curve: rows.cs.clone(),
            t0: u_start,
            t1: u_end,
            start_vertex_id: vertex2_id,
            end_vertex_id: vertex2_id,
            degenerate: false,
            name: None,
        });
        cs_pieces.push((cs_edge_id, [u_start, u_end], rows.cs_pcurve.clone()));
    }
    let seam_curve = rows.surface.iso_curve_u(u_start).or_refuse(KernelStage::Refine, "iso_curve_u")?;
    let [seam_t0, seam_t1] = seam_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let blend_seam_id = take_id();
    result.edges.push(EdgeRecord {
        id: blend_seam_id,
        curve: seam_curve,
        t0: seam_t0,
        t1: seam_t1,
        start_vertex_id: vertex1_id,
        end_vertex_id: vertex2_id,
        degenerate: false,
        name: None,
    });

    let old_vertex = edge.start_vertex_id;
    let replace_in_face = |result: &mut BrepSolid,
                           mate: &BlendMate,
                           pieces: &[(u64, [f64; 2], NurbsCurve)],
                           uv_new: [f64; 2]|
     -> Result<bool, KernelRefusal> {
        let v_new = uv_new[1];
        // (edge id, the START vertex is the removed one, the END vertex is).
        let seam_edge_ends: Vec<(u64, bool, bool)> = result
            .edges
            .iter()
            .filter(|candidate| {
                candidate.id != edge.id
                    && (candidate.start_vertex_id == old_vertex
                        || candidate.end_vertex_id == old_vertex)
            })
            .map(|candidate| {
                (
                    candidate.id,
                    candidate.start_vertex_id == old_vertex,
                    candidate.end_vertex_id == old_vertex,
                )
            })
            .collect();
        let seam_edge_ids: Vec<u64> = seam_edge_ends.iter().map(|(id, ..)| *id).collect();
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == mate.face.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during surgery"))?;
        let loop_record = &mut face.loops[mate.loop_index];
        let position = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.edge_id == edge.id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "edge_coedge", "blend: edge coedge lost during surgery"))?;
        let old_forward = loop_record.coedges[position].forward;
        let old_seam_v = {
            let pcurve = &loop_record.coedges[position].pcurve;
            let [d0, _] = pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            pcurve.evaluate(d0).or_refuse(KernelStage::Refine, "evaluate")?.y
        };
        // Replacement coedges: pieces run in u order; a reversed loop use
        // takes them in reverse order, each reversed.
        let mut replacements = Vec::new();
        let ordered: Vec<&(u64, [f64; 2], NurbsCurve)> = if old_forward {
            pieces.iter().collect()
        } else {
            pieces.iter().rev().collect()
        };
        for (index, (edge_id, _, pcurve)) in ordered.iter().enumerate() {
            replacements.push(CoedgeRecord {
                id: if index == 0 {
                    loop_record.coedges[position].id
                } else {
                    0 // patched by the caller with fresh ids
                },
                edge_id: *edge_id,
                forward: old_forward,
                pcurve: if old_forward {
                    pcurve.clone()
                } else {
                    pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
                },
            });
        }
        loop_record
            .coedges
            .splice(position..=position, replacements);
        let mut repaired = false;
        if loop_record.coedges.len() > 1 {
            // Anchored carrier: the loop's other coedges ride the seam
            // meridian between the old edge and the rest of the trim.
            // Pull the pcurve endpoint that met the old edge down to the
            // support crossing.  (The seam EDGE itself is trimmed after
            // both replacements, outside this borrow.)
            for coedge in &mut loop_record.coedges {
                if pieces
                    .iter()
                    .any(|(edge_id, ..)| *edge_id == coedge.edge_id)
                    || !seam_edge_ids.contains(&coedge.edge_id)
                {
                    continue;
                }
                let controls = &mut coedge.pcurve.control_points;
                let last_index = controls.len() - 1;
                let first_u = controls[0].x / controls[0].w;
                let last_u = controls[last_index].x / controls[last_index].w;
                let constant_u = (first_u - last_u).abs()
                    <= 1e-9 * (1.0 + first_u.abs().max(last_u.abs()));
                if constant_u {
                    // Seam MERIDIAN pcurve: constant u, runs along the v
                    // direction; the endpoint at the removed edge's v moves
                    // onto the support crossing.
                    let first_v = controls[0].y / controls[0].w;
                    let last_v = controls[last_index].y / controls[last_index].w;
                    let target = if (first_v - old_seam_v).abs() < (last_v - old_seam_v).abs() {
                        0
                    } else {
                        last_index
                    };
                    let w = controls[target].w;
                    controls[target].y = v_new * w;
                    continue;
                }
                // The adjacent trim runs along U at constant v — the carrier's
                // OTHER (v) seam, which is what a closed edge lying on one
                // whole MERIDIAN borders (an axial-plane collar on a torus).
                // Its endpoint moves in U, not V, and the v-distance rule
                // above cannot pick the end at all: both ends share the same
                // v, so it always lands on `last_index`, which is wrong for
                // one of the seam PAIR (the v-seam edge appears twice in the
                // loop, at v = 0 and v = 1).  Take the end from the TOPOLOGY.
                let Some((_, start_is_old, end_is_old)) = seam_edge_ends
                    .iter()
                    .find(|(id, ..)| *id == coedge.edge_id)
                    .copied()
                else {
                    continue;
                };
                if start_is_old == end_is_old {
                    return Err(KernelRefusal::ill_posed(KernelStage::Sew, "neighbour_trim_end", format!(
                        "blend: neighbour edge {} meets the blended edge at both ends; \
                         its trim end is ambiguous",
                        coedge.edge_id
                    )));
                }
                // `forward` maps the coedge's pcurve domain start onto the
                // edge's t0 (its start vertex).
                let target = if start_is_old == coedge.forward {
                    0
                } else {
                    last_index
                };
                let w = controls[target].w;
                controls[target].x = uv_new[0] * w;
                repaired = true;
            }
        }
        Ok(repaired)
    };
    let uv1_new = rows.cr_pcurve.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let v1_new = uv1_new.y;
    let cr_pieces = vec![(cr_edge_id, [u_start, u_end], rows.cr_pcurve.clone())];
    let mut repaired = replace_in_face(&mut result, first, &cr_pieces, [uv1_new.x, v1_new])?;
    let uv2_new = rows.cs_pcurve.evaluate(u_start).or_refuse(KernelStage::Refine, "evaluate")?;
    let v2_new = if second_split.is_some() {
        second_crossing_v
    } else {
        uv2_new.y
    };
    repaired |= replace_in_face(&mut result, second, &cs_pieces, [uv2_new.x, v2_new])?;

    // A contact circle about an axis oblique to a sphere's stored polar axis
    // can wind once around that sphere's U period.  Such a circle is not an
    // ordinary hole in the full-domain sphere rectangle: it bounds a cap with
    // one of the collapsed pole rims.  Keep that pole as the companion loop so
    // containment, integration, and tessellation see the actual cap topology.
    let collapse_winding_sphere_cap =
        |result: &mut BrepSolid, face_id: u64, support_ids: &[u64]| -> Result<(), KernelRefusal> {
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == face_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "sphere_carrier", "blend: sphere carrier lost during cap surgery"))?;
            if !matches!(
                face.surface.analytic(),
                Some(crate::AnalyticSurface::Sphere { .. })
            ) || face.surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")? != (true, false)
            {
                return Ok(());
            }
            let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            let period = u1 - u0;
            let support_loop = face.loops.iter().position(|loop_record| {
                !loop_record.coedges.is_empty()
                    && loop_record
                        .coedges
                        .iter()
                        .all(|coedge| support_ids.contains(&coedge.edge_id))
            });
            let Some(support_loop) = support_loop else {
                return Ok(());
            };
            if face.loops[support_loop].coedges.len() != 1 {
                return Ok(());
            }
            let support = &face.loops[support_loop].coedges[0];
            let [s0, s1] = support.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let support_start = support.pcurve.evaluate(s0).or_refuse(KernelStage::Refine, "evaluate")?;
            let support_end = support.pcurve.evaluate(s1).or_refuse(KernelStage::Refine, "evaluate")?;
            let support_winding = support_end.x - support_start.x;
            if (support_winding.abs() - period).abs() > 0.05 * period {
                return Ok(());
            }

            let mut pole: Option<(usize, CoedgeRecord)> = None;
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                if loop_index == support_loop {
                    continue;
                }
                for coedge in &loop_record.coedges {
                    let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                    let start = coedge.pcurve.evaluate(p0).or_refuse(KernelStage::Refine, "evaluate")?;
                    let end = coedge.pcurve.evaluate(p1).or_refuse(KernelStage::Refine, "evaluate")?;
                    let winding = end.x - start.x;
                    let at_pole = (start.y - v0).abs() <= 1e-8 || (start.y - v1).abs() <= 1e-8;
                    if at_pole
                        && (end.y - start.y).abs() <= 1e-8
                        && (winding.abs() - period).abs() <= 0.05 * period
                        && winding * support_winding < 0.0
                    {
                        pole = Some((loop_index, coedge.clone()));
                        break;
                    }
                }
                if pole.is_some() {
                    break;
                }
            }
            let Some((pole_loop, pole_coedge)) = pole else {
                return Ok(());
            };
            let support_record = face.loops[support_loop].clone();
            let pole_id = face.loops[pole_loop].id;
            face.loops = vec![
                LoopRecord {
                    id: pole_id,
                    coedges: vec![pole_coedge],
                },
                support_record,
            ];
            Ok(())
        };
    collapse_winding_sphere_cap(&mut result, first.face.id, &[cr_edge_id])?;
    let cs_edge_ids: Vec<u64> = cs_pieces.iter().map(|(edge_id, ..)| *edge_id).collect();
    collapse_winding_sphere_cap(&mut result, second.face.id, &cs_edge_ids)?;
    // Patch the zero coedge ids introduced for extra pieces.
    for shell in &mut result.shells {
        for face in &mut shell.faces {
            for loop_record in &mut face.loops {
                for coedge in &mut loop_record.coedges {
                    if coedge.id == 0 {
                        coedge.id = take_id();
                    }
                }
            }
        }
    }

    // Trim seam EDGES that ended on the removed vertex: their endpoint
    // moves onto the support-curve crossing on THEIR carrier (first
    // mate's seam -> the blend seam vertex; second mate's -> its own
    // seam-crossing vertex).
    let new_edge_ids: Vec<u64> = std::iter::once(cr_edge_id)
        .chain(cs_pieces.iter().map(|(edge_id, ..)| *edge_id))
        .chain(std::iter::once(blend_seam_id))
        .collect();
    let first_face_edges: Vec<u64> = first
        .face
        .loops
        .iter()
        .flat_map(|loop_record| loop_record.coedges.iter().map(|coedge| coedge.edge_id))
        .collect();
    for seam_edge in result.edges.iter_mut() {
        if new_edge_ids.contains(&seam_edge.id) {
            continue;
        }
        if seam_edge.start_vertex_id != old_vertex && seam_edge.end_vertex_id != old_vertex {
            continue;
        }
        let (target_vertex, target_v, target_point) = if first_face_edges.contains(&seam_edge.id) {
            (vertex1_id, v1_new, cr_start)
        } else {
            (second_seam_vertex, v2_new, second_seam_point)
        };
        let [c0, c1] = seam_edge.curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let clamped = target_v.clamp(c0.min(c1), c0.max(c1));
        // Using the support crossing's V as the neighbour's CURVE parameter is
        // only valid when that neighbour is the carrier's seam MERIDIAN, whose
        // 3D curve is parameterized by v.  Verify it against the vertex the
        // endpoint is moving to; a neighbour running along U instead (the
        // carrier's v-seam, bordered by a collar edge that lies on one whole
        // meridian) lands nowhere near it — the 2026-09-01 report trimmed the
        // torus' equator seam to u = 0, 10.885 away from its own vertex, and
        // the solid failed validation with "curve start does not match vertex".
        let band = 1e-6 * (1.0 + target_point.length());
        let split_parameter = if seam_edge.id == edge.id {
            // The BLENDED edge itself still carries the removed vertex at this
            // point.  Its coedges have already been replaced in both mates, so
            // the used-edge sweep below discards it; leave its (meaningless)
            // trim exactly as it was rather than asking a doomed edge to pass
            // through the support start.
            clamped
        } else {
            match seam_edge.curve.evaluate(clamped) {
                Ok(point) if point.sub(target_point).length() <= band => clamped,
                _ => {
                    repaired = true;
                    let projection =
                        crate::project_point_to_curve(&seam_edge.curve, target_point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
                    if projection.distance > band {
                        return Err(KernelRefusal::unsupported(KernelStage::Sew, "neighbour_off_support", format!(
                            "blend: neighbour edge {} does not pass through the blend's support \
                             start (off by {:.9}); the trim has no parameter to move to",
                            seam_edge.id, projection.distance
                        )));
                    }
                    projection.u.clamp(c0.min(c1), c0.max(c1))
                }
            }
        };
        if seam_edge.start_vertex_id == old_vertex {
            seam_edge.t0 = split_parameter;
            seam_edge.start_vertex_id = target_vertex;
        }
        if seam_edge.end_vertex_id == old_vertex {
            seam_edge.t1 = split_parameter;
            seam_edge.end_vertex_id = target_vertex;
        }
    }

    // Blend face: outward orientation matches F1's outward at the v=0 rim.
    let mid_u = (u_start + u_end) * 0.5;
    let blend_normal = raw_normal(&rows.surface, mid_u, 0.0)?;
    let station_uv = rows.cr_pcurve.evaluate(mid_u).or_refuse(KernelStage::Refine, "evaluate")?;
    let n1 = raw_normal(&first.face.surface, station_uv.x, station_uv.y)?;
    let out1 = if first.face.same_sense {
        n1
    } else {
        n1.scale(-1.0)
    };
    let same_sense = blend_normal.dot(out1) >= 0.0;
    let loop_id = take_id();
    let mut coedges = vec![
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
    ];
    for (edge_id, window, _) in cs_pieces.iter().rev() {
        coedges.push(CoedgeRecord {
            id: take_id(),
            edge_id: *edge_id,
            forward: false,
            pcurve: crate::sweep_topology::parameter_line(window[1], 1.0, window[0], 1.0).or_refuse(KernelStage::Refine, "parameter_line")?,
        });
    }
    coedges.push(CoedgeRecord {
        id: take_id(),
        edge_id: blend_seam_id,
        forward: false,
        pcurve: crate::sweep_topology::parameter_line(u_start, 1.0, u_start, 0.0).or_refuse(KernelStage::Refine, "parameter_line")?,
    });
    // The coedges above trace the parameter rectangle counter-clockwise in
    // (u, v); that traversal is only the outward boundary when the blend
    // surface normal already points outward (same_sense).  When it does not
    // (a mate whose support rim runs the other way — e.g. the reversed cap
    // of a seam carrier), the loop must wind the OTHER way so its coedges
    // traverse the shared support edges opposite to the mate, and so the
    // classification tangent test (parameter_point_in_face) stays consistent
    // with same_sense.  Reversing the coedge order and each coedge (forward
    // flag + pcurve) flips the winding without touching same_sense.
    if !same_sense {
        coedges.reverse();
        for coedge in &mut coedges {
            coedge.forward = !coedge.forward;
            coedge.pcurve = coedge.pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?;
        }
    }
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
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during surgery"))?;
    result.shells[shell_index].faces.push(blend_face);

    let used_edges: std::collections::HashSet<u64> = result
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .flat_map(|face| &face.loops)
        .flat_map(|loop_record| &loop_record.coedges)
        .map(|coedge| coedge.edge_id)
        .collect();
    result
        .edges
        .retain(|candidate| used_edges.contains(&candidate.id));
    let used_vertices: std::collections::HashSet<u64> = result
        .edges
        .iter()
        .flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|candidate| used_vertices.contains(&candidate.id));
    // A surgery that had to REPAIR a neighbour trim took a branch no existing
    // fixture exercises, so it does not get the benefit of the doubt: prove the
    // result before returning it.  Paths that never repaired are untouched (the
    // check does not run), so this cannot slow or change anything that already
    // works — and the new branch can never ship a plausible-looking wrong solid
    // the way the un-repaired trim did.
    if repaired {
        let problems = result.validate();
        if !problems.is_empty() {
            return Err(KernelRefusal::internal(KernelStage::Validate, "neighbour_trim_closure", format!(
                "blend: the repaired neighbour trim did not close ({} issue(s), first: {})",
                problems.len(),
                problems
                    .first()
                    .map(|issue| issue.message.as_str())
                    .unwrap_or("unknown")
            )));
        }
    }
    Ok(result)
}
