use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;

/// Fitted rows + end caps for an OPEN smooth chain.
struct OpenChainRows {
    surface: NurbsSurface,
    cr: NurbsCurve,
    cs: NurbsCurve,
    /// Per-segment (window_low, window_high, uv curve over [0,1]) in GLOBAL
    /// parameters, one per segment on each side.
    pcurves1: Vec<(f64, f64, NurbsCurve)>,
    pcurves2: Vec<(f64, f64, NurbsCurve)>,
    /// Whole-chain uv fits (row parameter space) for a side that stays on a
    /// single face (e.g. the top cap).
    whole_pcurves: [Option<NurbsCurve>; 2],
    /// The two free ends, reusing the single open-edge end construction.
    start_end: EndSurgery,
    finish_end: EndSurgery,
}

/// Blend an OPEN chain of conjugated edges: march every segment mid-out
/// (reusing march_chain) with a CLAMPED global chord parameter, fit clamped
/// blend rows, and cap each of the two FREE ends with the transverse-edge
/// construction on its end face (§6.9.5).  The end helpers are shared with
/// the single open-edge path (boundary_edge_at_vertex / end_face_id /
/// support_crossing / transverse_curve); interior junctions are trimmed with
/// spokes exactly like the closed chain.
pub(super) fn blend_open_smooth_chain(
    solid: &BrepSolid,
    chain: &SmoothChain<'_>,
    radius: f64,
    chamfer: bool,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    let segments = &chain.segments;
    let last_seg = segments.len() - 1;
    let samples = march_chain(segments, radius, true, CHAIN_PER_SEGMENT, FoldPolicy::Refuse)?;

    // Global 3D rows: segment 0's low overshoot + every in-segment station +
    // the last segment's high overshoot.  Interior overshoot is dropped (it
    // overlaps the neighbour's window and would collide in parameter).
    let mut global: Vec<(f64, &Station)> = Vec::new();
    for sample in &samples {
        let keep = if sample.segment == 0 && sample.position < 0 {
            true
        } else if sample.segment == last_seg && sample.position >= CHAIN_PER_SEGMENT as isize {
            true
        } else {
            (0..CHAIN_PER_SEGMENT as isize).contains(&sample.position)
        };
        if keep {
            global.push((sample.parameter, &sample.station));
        }
    }
    global.sort_by(|a, b| a.0.total_cmp(&b.0));
    let raw_params: Vec<f64> = global.iter().map(|(parameter, _)| *parameter).collect();
    for pair in raw_params.windows(2) {
        if pair[1] - pair[0] <= 1e-12 {
            return Err(KernelRefusal::internal(KernelStage::Refine, "coincident_rows", "blend: open chain produced coincident row parameters"));
        }
    }
    // Normalise the global chord parameter onto [0, 1]: interpolate_homogeneous
    // clamps the fitted domain to [0, 1], and the free-end overshoot leaves the
    // raw chord slightly outside that range.  Every window/crossing parameter
    // used by the surgery lives in this normalised space too.
    let global_low = raw_params[0];
    let global_range = (raw_params[raw_params.len() - 1] - global_low).max(1e-12);
    let normalize = |value: f64| (value - global_low) / global_range;
    let params: Vec<f64> = raw_params.iter().map(|value| normalize(*value)).collect();
    let degree = FIT_DEGREE;
    let mut samples_cr = Vec::with_capacity(global.len());
    let mut samples_cs = Vec::with_capacity(global.len());
    let mut samples_mid = Vec::with_capacity(global.len());
    for (_, station) in &global {
        samples_cr.push(Vec4::from_point(station.p1, 1.0));
        samples_cs.push(Vec4::from_point(station.p2, 1.0));
        samples_mid.push(Vec4 {
            x: station.apex.x * station.weight,
            y: station.apex.y * station.weight,
            z: station.apex.z * station.weight,
            w: station.weight,
        });
    }
    let cr = fit::interpolate_homogeneous(&samples_cr, degree, &params).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    let cs = fit::interpolate_homogeneous(&samples_cs, degree, &params).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
    let mid = if chamfer {
        None
    } else {
        Some(fit::interpolate_homogeneous(&samples_mid, degree, &params).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?)
    };
    let u_domain = cr.domain().or_refuse(KernelStage::Refine, "domain")?;
    let surface = crate::blend::rows::surface_from_rows(degree, &cr, &cs, mid.as_ref(), false)?;

    // Per-segment mate pcurves (uv1/uv2) over each segment's full window in
    // GLOBAL parameters — identical to the closed path.
    let mut pcurves1 = Vec::with_capacity(segments.len());
    let mut pcurves2 = Vec::with_capacity(segments.len());
    for segment in 0..segments.len() {
        let mut window: Vec<&ChainSample> = samples
            .iter()
            .filter(|sample| sample.segment == segment)
            .collect();
        window.sort_by(|a, b| a.parameter.total_cmp(&b.parameter));
        let window_params: Vec<f64> = window.iter().map(|sample| sample.parameter).collect();
        let low = window_params[0];
        let high = *window_params.last().unwrap();
        let normalized: Vec<f64> = window_params
            .iter()
            .map(|parameter| (parameter - low) / (high - low))
            .collect();
        let fit_uv = |select: &dyn Fn(&Station) -> [f64; 2]| -> Result<NurbsCurve, KernelRefusal> {
            let points: Vec<Vec4> = window
                .iter()
                .map(|sample| {
                    let uv = select(&sample.station);
                    Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0)
                })
                .collect();
            fit::interpolate_homogeneous(&points, degree, &normalized).or_refuse(KernelStage::Refine, "interpolate_homogeneous")
        };
        // The window bounds are stored in the NORMALISED global space so
        // pcurve_portion maps the surgery's piece windows onto the fit.
        let low = normalize(low);
        let high = normalize(high);
        pcurves1.push((low, high, fit_uv(&|station| station.uv1)?));
        pcurves2.push((low, high, fit_uv(&|station| station.uv2)?));
    }

    // Whole-chain mate pcurve for a side that stays on ONE face (top cap):
    // fit over the SAME extended stations as cr/cs so it spans the row domain.
    let single_face = |side: usize| -> bool {
        let first_id = segments[0].mate(side).face.id;
        segments
            .iter()
            .all(|segment| segment.mate(side).face.id == first_id)
    };
    let mut whole_pcurves: [Option<NurbsCurve>; 2] = [None, None];
    for side in 0..2 {
        if !single_face(side) {
            continue;
        }
        let mut points = Vec::with_capacity(global.len());
        for (_, station) in &global {
            let uv = if side == 0 { station.uv1 } else { station.uv2 };
            points.push(Vec4::from_point(Vec3::new(uv[0], uv[1], 0.0), 1.0));
        }
        whole_pcurves[side] = Some(fit::interpolate_homogeneous(&points, degree, &params).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?);
    }

    // Placeholder pcurves for FittedRows (the open-chain ends cross via 3D
    // intersection, so support_crossing_uv is never used here).
    let rows = FittedRows {
        cr_pcurve: cr.clone(),
        cs_pcurve: cs.clone(),
        surface,
        cr,
        cs,
        u_domain,
        center: None,
        exact_extrusion: false,
        vertex_stations: None,
    };

    // ---- the two free ends (shared open-edge end construction) -------
    let start_corner = chain.start_vertex;
    let finish_corner = chain.end_vertex;
    let mut ends: Vec<EndSurgery> = Vec::with_capacity(2);
    for (corner, at_start, seg_index) in [
        (start_corner, true, 0usize),
        (finish_corner, false, last_seg),
    ] {
        let segment = &segments[seg_index];
        let first_face = segment.first.face;
        let second_face = segment.second.face;
        let first_edge_id = boundary_edge_at_vertex(solid, first_face, corner, segment.edge.id)?;
        let second_edge_id = boundary_edge_at_vertex(solid, second_face, corner, segment.edge.id)?;
        if first_edge_id == second_edge_id {
            return Err(
                KernelRefusal::unsupported(KernelStage::Classify, "shared_end_boundary", "blend: open chain end mates share their boundary edge (unsupported)"),
            );
        }
        let end_face = end_face_id(
            solid,
            first_face.id,
            second_face.id,
            first_edge_id,
            second_edge_id,
        )?;
        let first_boundary = solid
            .edges
            .iter()
            .find(|candidate| candidate.id == first_edge_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_boundary_edge", "blend: end boundary edge missing"))?;
        let second_boundary = solid
            .edges
            .iter()
            .find(|candidate| candidate.id == second_edge_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_boundary_edge", "blend: end boundary edge missing"))?;
        let (cr_parameter, first_edge_parameter, _, cr_escalated) =
            support_crossing(solid, &rows.cr, first_boundary, at_start)?;
        let (cs_parameter, second_edge_parameter, _, cs_escalated) =
            support_crossing(solid, &rows.cs, second_boundary, at_start)?;
        let end_record = solid
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == end_face)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_face", "blend: end face missing"))?;
        let (transverse, blend_pcurve, end_pcurve) =
            transverse_curve(&rows, end_record, cr_parameter, cs_parameter)?;
        ends.push(EndSurgery {
            cr_parameter,
            cs_parameter,
            first_edge_id,
            first_edge_parameter,
            second_edge_id,
            second_edge_parameter,
            end_face_id: end_face,
            transverse_curve: transverse,
            transverse_blend_pcurve: blend_pcurve,
            transverse_end_pcurve: end_pcurve,
            escalated: cr_escalated || cs_escalated,
        });
    }
    let [start_end, finish_end] = match <[EndSurgery; 2]>::try_from(ends) {
        Ok(pair) => pair,
        Err(_) => return Err(KernelRefusal::internal(KernelStage::Refine, "open_chain_ends", "blend: open chain needs exactly two ends")),
    };

    let FittedRows {
        surface, cr, cs, ..
    } = rows;
    open_chain_surgery(
        solid,
        segments,
        OpenChainRows {
            surface,
            cr,
            cs,
            pcurves1,
            pcurves2,
            whole_pcurves,
            start_end,
            finish_end,
        },
        name,
    )
}

fn open_chain_surgery(
    solid: &BrepSolid,
    segments: &[ChainSegment<'_>],
    rows: OpenChainRows,
    name: Option<&str>,
) -> Result<BrepSolid, KernelRefusal> {
    use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
    let OpenChainRows {
        surface,
        cr,
        cs,
        pcurves1,
        pcurves2,
        whole_pcurves,
        start_end,
        finish_end,
    } = rows;
    let debug = std::env::var("BREP_DEBUG_CHAIN").is_ok();
    let mut result = solid.clone();
    let mut take_id = crate::blend::edge::fresh_id_source(solid);
    let n = segments.len();

    // ---- interior junction plans (spokes) ---------------------------
    struct SideJunction {
        junction: usize,
        spoke_edge_id: u64,
        crossing_support: f64,
        crossing_spoke: f64,
        vertex_id: u64,
    }
    let chain_vertex_at = |junction: usize| -> u64 {
        let segment = &segments[junction];
        if segment.forward {
            segment.edge.start_vertex_id
        } else {
            segment.edge.end_vertex_id
        }
    };
    let mut plans: Vec<Vec<SideJunction>> = Vec::new();
    // Tracks whether any junction below needed a looser-than-1e-7 spoke
    // crossing; if so the finished solid is validated before it is handed back.
    let mut crossings = SpokeCrossings::default();
    // The two FREE ends' own crossings escalate the same way a spoke's does.
    crossings.note(start_end.escalated || finish_end.escalated);
    for side in 0..2 {
        let support = if side == 0 { &cr } else { &cs };
        let mut junctions = Vec::new();
        for junction in 1..n {
            let before = &segments[junction - 1];
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
                continue;
            };
            if face_before.id == face_after.id {
                return Err(
                    KernelRefusal::unsupported(KernelStage::Classify, "open_chain_seam_junction", "blend: open smooth chain with a carrier-seam junction is not yet supported"),
                );
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
        plans.push(junctions);
    }

    // ---- support pieces per side ------------------------------------
    let mut side_edges: Vec<Vec<RimPiece>> = Vec::new();
    let mut rim_vertices = [[0u64; 2]; 2];
    let mut side_cut_vertices: Vec<HashMap<usize, u64>> = Vec::new();
    for side in 0..2 {
        let support = if side == 0 { &cr } else { &cs };
        let (start_param, finish_param) = if side == 0 {
            (start_end.cr_parameter, finish_end.cr_parameter)
        } else {
            (start_end.cs_parameter, finish_end.cs_parameter)
        };
        if finish_param - start_param <= 1e-9 {
            return Err(KernelRefusal::internal(KernelStage::Sew, "support_forward_span", "blend: open chain support has no forward span"));
        }
        let start_vertex = take_id();
        result.vertices.push(VertexRecord {
            id: start_vertex,
            point: support.evaluate(start_param).or_refuse(KernelStage::Refine, "evaluate")?,
        });
        let finish_vertex = take_id();
        result.vertices.push(VertexRecord {
            id: finish_vertex,
            point: support.evaluate(finish_param).or_refuse(KernelStage::Refine, "evaluate")?,
        });
        rim_vertices[side] = [start_vertex, finish_vertex];
        let mut cut_vertex: HashMap<usize, u64> = HashMap::default();
        let mut cuts: Vec<(f64, u64)> = Vec::new();
        for junction in &plans[side] {
            let vertex = take_id();
            result.vertices.push(VertexRecord {
                id: vertex,
                point: support.evaluate(junction.crossing_support).or_refuse(KernelStage::Refine, "evaluate")?,
            });
            cut_vertex.insert(junction.junction, vertex);
            cuts.push((junction.crossing_support, vertex));
        }
        side_cut_vertices.push(cut_vertex);
        cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut boundaries = vec![(start_param, start_vertex)];
        boundaries.extend(cuts.iter().map(|(parameter, vertex)| (*parameter, *vertex)));
        boundaries.push((finish_param, finish_vertex));
        let mut pieces = Vec::new();
        for window in boundaries.windows(2) {
            let (a, vertex_a) = window[0];
            let (b, vertex_b) = window[1];
            if b - a <= 1e-9 {
                return Err(KernelRefusal::internal(KernelStage::Sew, "degenerate_support_piece", "blend: degenerate open-chain support piece"));
            }
            let piece_curve = {
                let (_, tail) = support.split(a).or_refuse(KernelStage::Refine, "split")?;
                tail.split(b).or_refuse(KernelStage::Refine, "split")?.0
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

    // ---- transverse edges at the two free ends ----------------------
    let transverse_a_id = take_id();
    let ta_domain = start_end.transverse_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    result.edges.push(EdgeRecord {
        id: transverse_a_id,
        curve: start_end.transverse_curve.clone(),
        t0: ta_domain[0],
        t1: ta_domain[1],
        start_vertex_id: rim_vertices[0][0],
        end_vertex_id: rim_vertices[1][0],
        degenerate: false,
        name: None,
    });
    let transverse_b_id = take_id();
    let tb_domain = finish_end.transverse_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
    result.edges.push(EdgeRecord {
        id: transverse_b_id,
        curve: finish_end.transverse_curve.clone(),
        t0: tb_domain[0],
        t1: tb_domain[1],
        start_vertex_id: rim_vertices[0][1],
        end_vertex_id: rim_vertices[1][1],
        degenerate: false,
        name: None,
    });

    // ---- locate the end-face corners BEFORE trimming ----------------
    let start_corner = chain_vertex_at(0);
    let finish_corner = {
        let segment = &segments[n - 1];
        if segment.forward {
            segment.edge.end_vertex_id
        } else {
            segment.edge.start_vertex_id
        }
    };
    let traversal_vertices = |result: &BrepSolid, coedge: &CoedgeRecord| -> Option<(u64, u64)> {
        let edge = result
            .edges
            .iter()
            .find(|candidate| candidate.id == coedge.edge_id)?;
        Some(if coedge.forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        })
    };
    let mut end_plans = Vec::with_capacity(2);
    for (end, transverse_id, corner) in [
        (&start_end, transverse_a_id, start_corner),
        (&finish_end, transverse_b_id, finish_corner),
    ] {
        let face = result
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .find(|face| face.id == end.end_face_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_face", "blend: end face lost during surgery"))?;
        let mut located = None;
        'search: for (loop_index, loop_record) in face.loops.iter().enumerate() {
            let count = loop_record.coedges.len();
            for index in 0..count {
                let this = &loop_record.coedges[index];
                let next = &loop_record.coedges[(index + 1) % count];
                let this_pair = (this.edge_id, next.edge_id);
                let matches_pair = this_pair == (end.first_edge_id, end.second_edge_id)
                    || this_pair == (end.second_edge_id, end.first_edge_id);
                if !matches_pair {
                    continue;
                }
                let Some((_, this_end)) = traversal_vertices(&result, this) else {
                    continue;
                };
                let Some((next_start, _)) = traversal_vertices(&result, next) else {
                    continue;
                };
                if this_end == corner && next_start == corner {
                    located = Some((
                        loop_index,
                        this.id,
                        next.id,
                        this.edge_id == end.first_edge_id,
                    ));
                    break 'search;
                }
            }
        }
        let Some((loop_index, x_coedge_id, y_coedge_id, x_is_first)) = located else {
            return Err(KernelRefusal::internal(KernelStage::Sew, "end_face_corner", "blend: open chain end-face corner not found"));
        };
        end_plans.push((
            end.end_face_id,
            transverse_id,
            end.transverse_end_pcurve.clone(),
            loop_index,
            x_coedge_id,
            y_coedge_id,
            x_is_first,
        ));
    }

    // ---- trim spokes at interior junctions --------------------------
    for side in 0..2 {
        for junction in &plans[side] {
            let rim = side_cut_vertices[side][&junction.junction];
            trim_edge_at(
                &mut result,
                junction.spoke_edge_id,
                junction.crossing_spoke,
                junction.vertex_id,
                rim,
            )?;
        }
    }

    // ---- trim the four end boundary edges to their crossings ---------
    for (boundary_id, boundary_param, corner, rim) in [
        (
            start_end.first_edge_id,
            start_end.first_edge_parameter,
            start_corner,
            rim_vertices[0][0],
        ),
        (
            start_end.second_edge_id,
            start_end.second_edge_parameter,
            start_corner,
            rim_vertices[1][0],
        ),
        (
            finish_end.first_edge_id,
            finish_end.first_edge_parameter,
            finish_corner,
            rim_vertices[0][1],
        ),
        (
            finish_end.second_edge_id,
            finish_end.second_edge_parameter,
            finish_corner,
            rim_vertices[1][1],
        ),
    ] {
        trim_edge_at(&mut result, boundary_id, boundary_param, corner, rim)?;
    }

    // ---- replace chain coedges in the mate faces --------------------
    let mut side_use_forward = [true, true];
    for side in 0..2 {
        let pieces = &side_edges[side];
        let face_info = |segment: &ChainSegment| {
            let mate = segment.mate(side);
            (mate.face.id, mate.loop_index, mate.coedge.id)
        };
        let fitted = if side == 0 { &pcurves1 } else { &pcurves2 };
        let segment_mid = |index: usize| -> f64 {
            let (low, high, _) = &fitted[index];
            (low + high) * 0.5
        };
        let mut per_face: HashMap<u64, Vec<usize>> = HashMap::default();
        for (piece_index, piece) in pieces.iter().enumerate() {
            let midpoint = (piece.window[0] + piece.window[1]) * 0.5;
            let mut best = (0usize, f64::INFINITY);
            for segment_index in 0..segments.len() {
                let distance = (segment_mid(segment_index) - midpoint).abs();
                if distance < best.1 {
                    best = (segment_index, distance);
                }
            }
            let (face_id, ..) = face_info(&segments[best.0]);
            per_face.entry(face_id).or_default().push(piece_index);
        }
        for (face_id, piece_indices) in per_face {
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
            let chain_forward = {
                let segment = segments
                    .iter()
                    .find(|segment| face_info(segment).0 == face_id)
                    .unwrap();
                let (.., coedge_id) = face_info(segment);
                let face = result
                    .shells
                    .iter()
                    .flat_map(|shell| &shell.faces)
                    .find(|face| face.id == face_id)
                    .unwrap();
                let old = face.loops[loop_index]
                    .coedges
                    .iter()
                    .find(|coedge| coedge.id == coedge_id)
                    .unwrap();
                old.forward == segment.forward
            };
            side_use_forward[side] = chain_forward;
            let mut ordered: Vec<&RimPiece> =
                piece_indices.iter().map(|index| &pieces[*index]).collect();
            ordered.sort_by(|a, b| a.window[0].total_cmp(&b.window[0]));
            if !chain_forward {
                ordered.reverse();
            }
            let mut replacements = Vec::new();
            for piece in ordered {
                let pcurve = if let Some(whole) = &whole_pcurves[side] {
                    let [w0, w1] = piece.window;
                    let (_, tail) = whole.split(w0).or_refuse(KernelStage::Refine, "split")?;
                    tail.split(w1).or_refuse(KernelStage::Refine, "split")?.0
                } else {
                    let midpoint = (piece.window[0] + piece.window[1]) * 0.5;
                    let nearest = fitted
                        .iter()
                        .enumerate()
                        .min_by(|(a, _), (b, _)| {
                            (segment_mid(*a) - midpoint)
                                .abs()
                                .total_cmp(&(segment_mid(*b) - midpoint).abs())
                        })
                        .map(|(_, fitted)| fitted)
                        .unwrap();
                    pcurve_portion(nearest, piece.window[0], piece.window[1])?
                };
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
            let face = result
                .shells
                .iter_mut()
                .flat_map(|shell| &mut shell.faces)
                .find(|face| face.id == face_id)
                .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_face", "blend: mate face lost during open chain surgery"))?;
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

    // ---- splice the transverse coedge into each end face ------------
    for (
        end_face,
        transverse_id,
        transverse_end_pcurve,
        loop_index,
        x_coedge_id,
        y_coedge_id,
        x_is_first,
    ) in end_plans
    {
        let forward = x_is_first;
        let pcurve = if forward {
            transverse_end_pcurve
        } else {
            transverse_end_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?
        };
        let coedge_id = take_id();
        let mut transverse_coedge = Some(CoedgeRecord {
            id: coedge_id,
            edge_id: transverse_id,
            forward,
            pcurve,
        });
        let face = result
            .shells
            .iter_mut()
            .flat_map(|shell| &mut shell.faces)
            .find(|face| face.id == end_face)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_face", "blend: end face lost during surgery"))?;
        let loop_record = &mut face.loops[loop_index];
        let count = loop_record.coedges.len();
        let seg_index = loop_record
            .coedges
            .iter()
            .position(|coedge| coedge.id == x_coedge_id)
            .ok_or(KernelRefusal::internal(KernelStage::Sew, "end_face_corner", "blend: end-face corner coedge lost during surgery"))?;
        let y_index = (seg_index + 1) % count;
        if loop_record.coedges[y_index].id != y_coedge_id {
            return Err(KernelRefusal::internal(KernelStage::Sew, "end_face_corner", "blend: end-face corner pair no longer adjacent"));
        }
        let mut rebuilt = Vec::with_capacity(count + 1);
        for index in 0..count {
            rebuilt.push(loop_record.coedges[index].clone());
            if index == seg_index {
                rebuilt.push(transverse_coedge.take().expect("transverse spliced once"));
            }
        }
        loop_record.coedges = rebuilt;
    }

    // ---- blend face -------------------------------------------------
    // same_sense from the middle segment's first mate normal vs the blend
    // surface normal (the loop winds so its use of each piece opposes the
    // mate's use — manifold pairing).
    let mid_seg = segments.len() / 2;
    let (low_mid, high_mid, pc1_mid) = &pcurves1[mid_seg];
    let mid_uv = pc1_mid.evaluate(0.5).or_refuse(KernelStage::Refine, "evaluate")?;
    let mate_normal = raw_normal(&segments[mid_seg].first.face.surface, mid_uv.x, mid_uv.y)?;
    let out1 = if segments[mid_seg].first.face.same_sense {
        mate_normal
    } else {
        mate_normal.scale(-1.0)
    };
    let blend_normal = raw_normal(&surface, (low_mid + high_mid) * 0.5, 0.0)?;
    let same_sense = blend_normal.dot(out1) >= 0.0;
    let blend_cr_forward = !side_use_forward[0];
    if debug {
        eprintln!(
            "OPEN chain: {} segs, side_use_forward={:?} blend_cr_forward={blend_cr_forward} same_sense={same_sense}",
            n, side_use_forward
        );
    }
    let loop_id = take_id();
    let mut coedges = Vec::new();
    if blend_cr_forward {
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
            edge_id: transverse_b_id,
            forward: true,
            pcurve: finish_end.transverse_blend_pcurve.clone(),
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
            edge_id: transverse_a_id,
            forward: false,
            pcurve: start_end.transverse_blend_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
        });
    } else {
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
            edge_id: transverse_a_id,
            forward: true,
            pcurve: start_end.transverse_blend_pcurve.clone(),
        });
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
            edge_id: transverse_b_id,
            forward: false,
            pcurve: finish_end.transverse_blend_pcurve.reversed().or_refuse(KernelStage::Refine, "reversed")?,
        });
    }
    let blend_face = FaceRecord {
        id: take_id(),
        surface,
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
        .position(|shell| {
            shell
                .faces
                .iter()
                .any(|face| face.id == segments[0].first.face.id)
        })
        .ok_or(KernelRefusal::internal(KernelStage::Sew, "mate_shell", "blend: mate shell lost during open chain surgery"))?;
    result.shells[shell_index].faces.push(blend_face);

    // ---- drop chain edges and prune orphan vertices -----------------
    let chain_edge_ids: Vec<u64> = segments.iter().map(|segment| segment.edge.id).collect();
    result
        .edges
        .retain(|candidate| !chain_edge_ids.contains(&candidate.id));
    let used: HashSet<u64> = result
        .edges
        .iter()
        .flat_map(|candidate| [candidate.start_vertex_id, candidate.end_vertex_id])
        .collect();
    result
        .vertices
        .retain(|candidate| used.contains(&candidate.id));
    crossings.gate(result)
}
