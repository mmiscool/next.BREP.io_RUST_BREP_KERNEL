use crate::mass_properties::parameter_space_area;
use crate::topology::{adaptive_coedge_error, BrepSolid, CoedgeRecord, EdgeRecord};
use crate::{
    build_pcurve_on_surface, interpolate_curve, KernelTolerances, NurbsCurve, NurbsSurface, Vec3,
};
use crate::{KernelRefusal, KernelStage, OrRefuse};
use rustc_hash::FxHashMap as HashMap;

fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot
    // tolerance (1e-9) of the domain ends; the skip epsilon must cover that
    // or a near-edge trim parameter slips past the guard and errors.
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if t0 > start + epsilon && t0 < end - epsilon {
        result = result.split(t0).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Sew, "domain")?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        result = result.split(t1).or_refuse(KernelStage::Sew, "split")?.0;
    } else if std::env::var("BREP_DEBUG_SUBRANGE").is_ok() && t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in coalesce trimmed_curve");
    }
    Ok(result)
}

fn curvature_at(curve: &NurbsCurve, parameter: f64) -> Result<f64, KernelRefusal> {
    let derivatives = curve.derivatives(parameter, 2).or_refuse(KernelStage::Sew, "derivatives")?;
    let speed = derivatives[1].length();
    if speed <= 1e-12 {
        return Ok(0.0);
    }
    Ok(derivatives[1].cross(derivatives[2]).length() / speed.powi(3))
}

/// Preserve the topology contract after a sampled continuation fit.
///
/// Interpolation is mathematically endpoint-interpolating, but a high-degree
/// solve can lose several digits on long or unevenly parameterized pieces.
/// Clamped NURBS evaluate to their active endpoint controls. Extra endpoint
/// knots can leave inactive controls before or after those controls; use the
/// evaluator's span selection to locate the controls that actually contribute.
/// Restoring those controls leaves all other fit controls unchanged.
fn constrain_curve_endpoints(
    mut curve: NurbsCurve,
    start: Vec3,
    end: Vec3,
) -> Result<NurbsCurve, KernelRefusal> {
    let [domain_start, domain_end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // Over-clamped vectors have inactive controls beyond their endpoint
    // multiplicities. Anchor the controls that actually evaluate there.
    let first = crate::curve::knot_find_span(&curve.knots, curve.degree, domain_start)
        - curve.degree;
    let last = crate::curve::knot_find_span(&curve.knots, curve.degree, domain_end);
    let first_weight = curve.control_points[first].w;
    let last_weight = curve.control_points[last].w;
    if first_weight.abs() <= 1e-15 || last_weight.abs() <= 1e-15 {
        return Err(KernelRefusal::internal(KernelStage::Sew, "constrain_zero_weight", "cannot constrain a curve endpoint with zero weight"));
    }
    curve.control_points[first].x = start.x * first_weight;
    curve.control_points[first].y = start.y * first_weight;
    curve.control_points[first].z = start.z * first_weight;
    curve.control_points[last].x = end.x * last_weight;
    curve.control_points[last].y = end.y * last_weight;
    curve.control_points[last].z = end.z * last_weight;
    let endpoint_epsilon = 1e-10;
    if curve.evaluate(domain_start).or_refuse(KernelStage::Sew, "evaluate")?.sub(start).length() > endpoint_epsilon
        || curve.evaluate(domain_end).or_refuse(KernelStage::Sew, "evaluate")?.sub(end).length() > endpoint_epsilon
    {
        return Err(KernelRefusal::non_convergence(KernelStage::Sew, "continuation_endpoint_clamp", "continuation curve is not clamped at its topology endpoints"));
    }
    Ok(curve)
}

/// Join separately fitted pieces of the same smooth carrier curve. Exact
/// homogeneous concatenation is preferred, but independently fitted SSI
/// branches can represent the same circle with slightly different weights.
fn concatenate_sampled_pieces(
    first: &NurbsCurve,
    second: &NurbsCurve,
    split: f64,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    let first_domain = first.domain().or_refuse(KernelStage::Sew, "domain")?;
    let second_domain = second.domain().or_refuse(KernelStage::Sew, "domain")?;
    let samples_per_piece = 48;
    let mut points = Vec::<Vec3>::with_capacity(samples_per_piece * 2 + 1);
    let mut parameters = Vec::with_capacity(samples_per_piece * 2 + 1);
    for index in 0..=samples_per_piece {
        let fraction = index as f64 / samples_per_piece as f64;
        points.push(
            first.evaluate(first_domain[0] + (first_domain[1] - first_domain[0]) * fraction).or_refuse(KernelStage::Sew, "evaluate")?,
        );
        parameters.push(split * fraction);
    }
    for index in 1..=samples_per_piece {
        let fraction = index as f64 / samples_per_piece as f64;
        points.push(
            second.evaluate(second_domain[0] + (second_domain[1] - second_domain[0]) * fraction).or_refuse(KernelStage::Sew, "evaluate")?,
        );
        parameters.push(split + (1.0 - split) * fraction);
    }
    let joined = interpolate_curve(&points, first.degree.min(second.degree), &parameters).or_refuse(KernelStage::Sew, "interpolate_curve")?;
    for index in 0..=64 {
        let fraction = index as f64 / 64.0;
        let expected = if fraction <= split {
            let local = if split <= 1e-12 {
                0.0
            } else {
                fraction / split
            };
            first.evaluate(first_domain[0] + (first_domain[1] - first_domain[0]) * local).or_refuse(KernelStage::Sew, "evaluate")?
        } else {
            let local = (fraction - split) / (1.0 - split);
            second.evaluate(second_domain[0] + (second_domain[1] - second_domain[0]) * local).or_refuse(KernelStage::Sew, "evaluate")?
        };
        if joined.evaluate(fraction).or_refuse(KernelStage::Sew, "evaluate")?.sub(expected).length() > tolerance {
            return Ok(None);
        }
    }
    Ok(Some(joined))
}

fn concatenate_compatible_fitted_pieces(
    first: &NurbsCurve,
    second: &NurbsCurve,
    split: f64,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    let first_domain = first.domain().or_refuse(KernelStage::Sew, "domain")?;
    let second_domain = second.domain().or_refuse(KernelStage::Sew, "domain")?;
    let first_curvature = curvature_at(first, first_domain[1])?;
    let second_curvature = curvature_at(second, second_domain[0])?;
    let curvature_scale = first_curvature.abs().max(second_curvature.abs()).max(1.0);
    // Independently fitted SSI pieces can differ by a few tenths of a
    // percent in endpoint curvature even when their tangent and sampled
    // loci form one carrier curve. The sampled concatenation below remains
    // the authoritative geometric guard.
    if (first_curvature - second_curvature).abs() > curvature_scale * 2e-3 {
        return Ok(None);
    }
    concatenate_sampled_pieces(first, second, split, tolerance)
}

/// Join two exact NURBS pieces with a degree-multiplicity internal knot.
pub fn concatenate_exact_curve_pieces(
    first: &NurbsCurve,
    second: &NurbsCurve,
    split: f64,
    tolerance: f64,
) -> Result<Option<NurbsCurve>, KernelRefusal> {
    if first.degree != second.degree || split <= 1e-9 || split >= 1.0 - 1e-9 {
        return Ok(None);
    }
    let degree = first.degree;
    let [first_start, first_end] = first.domain().or_refuse(KernelStage::Sew, "domain")?;
    let [second_start, second_end] = second.domain().or_refuse(KernelStage::Sew, "domain")?;
    let first_span = first_end - first_start;
    let second_span = second_end - second_start;
    if first_span <= 1e-9 || second_span <= 1e-9 {
        return Ok(None);
    }
    let first_end_control = first.control_points[first.control_points.len() - 1];
    let second_start_control = second.control_points[0];
    let second_scale = first_end_control.w / second_start_control.w;
    let scaled_second = second
        .control_points
        .iter()
        .map(|control| control.scale(second_scale))
        .collect::<Vec<_>>();
    let scaled_start = scaled_second[0];
    let scale = 1.0f64
        .max(first_end_control.x.abs())
        .max(first_end_control.y.abs())
        .max(first_end_control.z.abs())
        .max(first_end_control.w.abs());
    let homogeneous_tolerance = (1e-10f64).max(tolerance) * scale;
    if (first_end_control.x - scaled_start.x).abs() > homogeneous_tolerance
        || (first_end_control.y - scaled_start.y).abs() > homogeneous_tolerance
        || (first_end_control.z - scaled_start.z).abs() > homogeneous_tolerance
        || (first_end_control.w - scaled_start.w).abs() > homogeneous_tolerance
    {
        return Ok(None);
    }
    let remap = |curve: &NurbsCurve, from_start, from_end, to_start, to_end| {
        curve
            .knots
            .iter()
            .map(|knot| {
                to_start + (to_end - to_start) * (knot - from_start) / (from_end - from_start)
            })
            .collect::<Vec<_>>()
    };
    let first_knots = remap(first, first_start, first_end, 0.0, split);
    let second_knots = remap(second, second_start, second_end, split, 1.0);
    let mut knots = first_knots[..first_knots.len() - degree - 1].to_vec();
    knots.extend(std::iter::repeat_n(split, degree));
    knots.extend_from_slice(&second_knots[degree + 1..]);
    let mut controls = first.control_points.clone();
    controls.extend_from_slice(&scaled_second[1..]);
    let joined = NurbsCurve::new(degree, knots, controls).or_refuse(KernelStage::Sew, "NurbsCurve::new")?;
    for fraction in [0.0, 0.23, 0.61, 1.0] {
        if joined
            .evaluate(split * fraction).or_refuse(KernelStage::Sew, "evaluate")?
            .sub(first.evaluate(first_start + first_span * fraction).or_refuse(KernelStage::Sew, "evaluate")?)
            .length()
            > tolerance
            || joined
                .evaluate(split + (1.0 - split) * fraction).or_refuse(KernelStage::Sew, "evaluate")?
                .sub(second.evaluate(second_start + second_span * fraction).or_refuse(KernelStage::Sew, "evaluate")?)
                .length()
                > tolerance
        {
            return Ok(None);
        }
    }
    Ok(Some(joined))
}

fn pcurve_matches_edge(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    reversed: bool,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let [c0, c1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // Topology validation rejects pcurve-vs-edge deviations above 4e-3.
    // Guard merges below that threshold with a 25% margin so a concatenation
    // with drifted parameterization is rebuilt or skipped before validation.
    let geometry_tolerance = (tolerance * 1.5).max(1e-4).min(3e-3);
    for index in 0..=32 {
        let fraction = index as f64 / 32.0;
        let uv = pcurve.evaluate(p0 + (p1 - p0) * fraction).or_refuse(KernelStage::Sew, "evaluate")?;
        let curve_fraction = if reversed { 1.0 - fraction } else { fraction };
        if surface
            .evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?
            .sub(curve.evaluate(c0 + (c1 - c0) * curve_fraction).or_refuse(KernelStage::Sew, "evaluate")?)
            .length()
            > geometry_tolerance
        {
            return Ok(false);
        }
    }
    Ok(true)
}

#[derive(Clone, Copy)]
struct UseLocation {
    face: usize,
    loop_index: usize,
    coedge: usize,
}

fn replace_adjacent(
    coedges: &mut Vec<CoedgeRecord>,
    first_index: usize,
    replacement: CoedgeRecord,
) -> bool {
    if coedges.len() < 2 {
        return false;
    }
    if first_index + 1 < coedges.len() {
        coedges.splice(first_index..first_index + 2, [replacement]);
    } else {
        coedges.pop();
        coedges[0] = replacement;
    }
    true
}

/// An exact conic piece: a rational quadratic, the form every analytic
/// section circle or ellipse is built in (`make_circle`), and never the form a
/// fit takes (`fit_polyline` interpolates non-rational cubics).
fn is_exact_conic(curve: &NurbsCurve) -> bool {
    curve.degree == 2 && curve.control_points.iter().any(|point| (point.w - 1.0).abs() > 1e-12)
}

fn traversal_vertex_ids(edge: &EdgeRecord, coedge: &CoedgeRecord) -> (u64, u64) {
    if coedge.forward {
        (edge.start_vertex_id, edge.end_vertex_id)
    } else {
        (edge.end_vertex_id, edge.start_vertex_id)
    }
}

/// The largest turn, in radians, two edges may make at their shared vertex and
/// still count as one carrier continued.
///
/// This is the ONLY guard against merging across a genuine corner, because
/// [`concatenate_exact_curve_pieces`] joins with a knot of degree multiplicity
/// and therefore reproduces both pieces EXACTLY — a box's two top edges would
/// concatenate into one C0 polyline and pass every geometric check below. So
/// the band has to stay small.
///
/// It cannot be arbitrarily small either. Edges on one SSI-fitted intersection
/// curve are fitted PIECE BY PIECE, and a fit is accurate in position, not in
/// endpoint derivative: two degree-3 pieces of the same sphere/cylinder circle
/// (the tube joint in report 20260909T063157Z) sit within 4.6e-6 of that circle
/// and still disagree by 2.8e-4 rad at their shared vertex — the fitted
/// endpoint derivative is two orders of magnitude looser than the fitted point.
/// A band of 1e-3 rad clears that measured noise by 3.5x and still refuses any
/// corner a modeller can express — 0.057 degrees.
const CONTINUATION_TURN: f64 = 1e-3;

/// The band a merge that CLOSES its edge into a loop must meet instead: the
/// historical `1 - dot < 1e-10`, restated as the angle it always was.
///
/// Measured, on the two shapes that made the distinction necessary. The
/// crossing-pipe tee (`push_cylinder_side_against_a_crossing_pipe_matches_the_
/// rebuilt_tee`) has a closed form —
/// `V = π·9·10 + π·2.25·12 − 4·∫₋ᵦᵇ √((a²−y²)(b²−y²)) dy = 326.525260789` — and
/// its marched rim, whose two SSI halves turn 8.8e-4 at their join, sits 2.8e-7
/// from it as two arcs and 1.7e-6 as one closed edge: six times the error, for
/// a vertex nobody asked to lose. The torus collar
/// (`collar_chain_on_a_torus_carrier_crosses_the_v_seam`) is the same shape at
/// 6.9e-4 and 2.2e-4. Against that, the plane × sphere rim of
/// `pushing_an_obliquely_capped_ball_refuses_as_an_oblique_rim` is two pieces
/// of ONE analytic circle: it turns nothing measurable, closes as it always
/// has, and is the reason this is a second band rather than a flat refusal.
///
/// Neither parameter area (5.3e-7 for the good closure against 1.1e-7 for a bad
/// one) nor the p-curve gap at the join (zero for all of them) separates the
/// cases. Exactness of the tangency does.
const EXACT_CLOSING_TURN: f64 = 1.4142135623730951e-5;

/// Collapse tangent continuation edges seen by the same two incident faces —
/// or by the same face TWICE, which is what a split seam looks like.
///
/// Both 3D curves and both face p-curves are concatenated exactly. Periodic
/// branch changes are rejected if they alter either face's parameter area.
pub fn merge_curve_continuation_edges(
    solid: &BrepSolid,
    tolerance: f64,
) -> Result<BrepSolid, KernelRefusal> {
    merge_curve_continuation_edges_impl(solid, tolerance, None)
}

/// [`merge_curve_continuation_edges`] at the listed vertices only: a pair of
/// continuation edges is joined when the vertex between them is in `at`, and
/// every other vertex of the solid is left standing.
pub(crate) fn merge_curve_continuation_edges_at(
    solid: &BrepSolid,
    tolerance: f64,
    at: &rustc_hash::FxHashSet<u64>,
) -> Result<BrepSolid, KernelRefusal> {
    merge_curve_continuation_edges_impl(solid, tolerance, Some(at))
}

fn merge_curve_continuation_edges_impl(
    solid: &BrepSolid,
    tolerance: f64,
    at: Option<&rustc_hash::FxHashSet<u64>>,
) -> Result<BrepSolid, KernelRefusal> {
    let mut result = solid.clone();
    let mut rejected = rustc_hash::FxHashSet::<(u64, u64)>::default();
    loop {
        let edges = result
            .edges
            .iter()
            .map(|edge| (edge.id, edge))
            .collect::<HashMap<_, _>>();
        let vertices = result
            .vertices
            .iter()
            .map(|vertex| (vertex.id, vertex.point))
            .collect::<HashMap<_, _>>();
        let faces = result
            .shells
            .iter()
            .flat_map(|shell| shell.faces.iter())
            .collect::<Vec<_>>();
        let mut uses: HashMap<u64, Vec<UseLocation>> = HashMap::default();
        for (face_index, face) in faces.iter().enumerate() {
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
                    uses.entry(coedge.edge_id).or_default().push(UseLocation {
                        face: face_index,
                        loop_index,
                        coedge: coedge_index,
                    });
                }
            }
        }
        let mut accepted = None;
        'faces: for (face_index, face) in faces.iter().enumerate() {
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                if loop_record.coedges.len() < 2 {
                    continue;
                }
                for first_index in 0..loop_record.coedges.len() {
                    let second_index = (first_index + 1) % loop_record.coedges.len();
                    let first = &loop_record.coedges[first_index];
                    let second = &loop_record.coedges[second_index];
                    let first_edge = edges[&first.edge_id];
                    let second_edge = edges[&second.edge_id];
                    let pair_key = if first.edge_id < second.edge_id {
                        (first.edge_id, second.edge_id)
                    } else {
                        (second.edge_id, first.edge_id)
                    };
                    if rejected.contains(&pair_key) {
                        continue;
                    }
                    if first_edge.degenerate || second_edge.degenerate {
                        continue;
                    }
                    let (_, first_end) = traversal_vertex_ids(first_edge, first);
                    let (second_start, _) = traversal_vertex_ids(second_edge, second);
                    if first_end != second_start {
                        continue;
                    }
                    if at.is_some_and(|at| !at.contains(&first_end)) {
                        continue;
                    }
                    let Some(first_uses) = uses.get(&first.edge_id) else {
                        continue;
                    };
                    let Some(second_uses) = uses.get(&second.edge_id) else {
                        continue;
                    };
                    if first_uses.len() != 2 || second_uses.len() != 2 {
                        continue;
                    }
                    let first_mate = first_uses
                        .iter()
                        .find(|location| {
                            location.face != face_index
                                || location.loop_index != loop_index
                                || location.coedge != first_index
                        })
                        .copied()
                        .unwrap();
                    let second_mate = second_uses
                        .iter()
                        .find(|location| {
                            location.face != face_index
                                || location.loop_index != loop_index
                                || location.coedge != second_index
                        })
                        .copied()
                        .unwrap();
                    // A SEAM exposes both uses of both edges on one loop of one
                    // face — the two sides of the parametric rectangle. That is
                    // the shape a split sphere seam wears, so it is a case to
                    // answer rather than to decline; the two indexed splices
                    // are applied high index first below so neither shifts the
                    // other. What it must NOT be is the same PAIR twice: the
                    // two runs have to be disjoint for both splices to land.
                    let same_loop =
                        first_mate.face == face_index && first_mate.loop_index == loop_index;
                    if same_loop
                        && (first_mate.coedge == first_index
                            || first_mate.coedge == second_index
                            || second_mate.coedge == first_index
                            || second_mate.coedge == second_index)
                    {
                        continue;
                    }
                    if first_mate.face != second_mate.face
                        || first_mate.loop_index != second_mate.loop_index
                        || (second_mate.coedge + 1)
                            % faces[second_mate.face].loops[second_mate.loop_index]
                                .coedges
                                .len()
                            != first_mate.coedge
                    {
                        continue;
                    }
                    let first_parameter = if first.forward {
                        first_edge.t1
                    } else {
                        first_edge.t0
                    };
                    let second_parameter = if second.forward {
                        second_edge.t0
                    } else {
                        second_edge.t1
                    };
                    let mut first_tangent = first_edge.curve.derivatives(first_parameter, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    let mut second_tangent = second_edge.curve.derivatives(second_parameter, 1).or_refuse(KernelStage::Sew, "derivatives")?[1];
                    if !first.forward {
                        first_tangent = first_tangent.scale(-1.0);
                    }
                    if !second.forward {
                        second_tangent = second_tangent.scale(-1.0);
                    }
                    let first_tangent = first_tangent.normalized().or_refuse(KernelStage::Sew, "normalized")?;
                    let second_tangent = second_tangent.normalized().or_refuse(KernelStage::Sew, "normalized")?;
                    // Measure the turn as an ANGLE, not as `1 - dot`: the dot
                    // of two near-parallel unit vectors loses most of its
                    // digits to cancellation exactly where this test lives.
                    let turn = first_tangent
                        .cross(second_tangent)
                        .length()
                        .atan2(first_tangent.dot(second_tangent));
                    // Closing a LOOP asks more of the evidence than continuing
                    // an open edge. Merging the last two pieces of a closed
                    // loop removes the loop's only vertex and leaves one closed
                    // edge whose p-curve has to span the carrier's whole period
                    // on at least one face — which no exact concatenation can
                    // produce across the seam, so the p-curve is re-projected,
                    // and a re-projected full period is where the fit is worst.
                    // So a closure keeps the historical EXACT band: two pieces
                    // of one analytic circle (a plane cutting a sphere) still
                    // close, and two SSI fits of a marched quartic do not.
                    let closes = traversal_vertex_ids(first_edge, first).0
                        == traversal_vertex_ids(second_edge, second).1;
                    let band = if closes {
                        EXACT_CLOSING_TURN
                    } else {
                        CONTINUATION_TURN
                    };
                    if turn.abs() > band {
                        continue;
                    }
                    // The band stood in for provenance — "two pieces of one
                    // analytic circle close, two SSI fits do not" — only while
                    // fitted pieces met with sloppy endpoint derivatives. Held
                    // to their carriers at every mid-span (2026-09-26) they meet
                    // inside it, and the torus collar closed into one edge the
                    // blend layer does not chain. So a closure asks for the
                    // provenance itself: both pieces EXACT conics.
                    // Escape hatch BREP_CLOSE_EXACT_CONICS_ONLY=0.
                    if closes
                        && std::env::var("BREP_CLOSE_EXACT_CONICS_ONLY").as_deref() != Ok("0")
                        && !(is_exact_conic(&first_edge.curve) && is_exact_conic(&second_edge.curve))
                    {
                        continue;
                    }
                    let mut first_curve =
                        trimmed_curve(&first_edge.curve, first_edge.t0, first_edge.t1)?;
                    let mut second_curve =
                        trimmed_curve(&second_edge.curve, second_edge.t0, second_edge.t1)?;
                    if !first.forward {
                        first_curve = first_curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                    }
                    if !second.forward {
                        second_curve = second_curve.reversed().or_refuse(KernelStage::Sew, "reversed")?;
                    }
                    let first_span = {
                        let domain = first_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                        domain[1] - domain[0]
                    };
                    let second_span = {
                        let domain = second_curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                        domain[1] - domain[0]
                    };
                    let split = first_span / (first_span + second_span);
                    let curve = if let Some(curve) = concatenate_exact_curve_pieces(
                        &first_curve,
                        &second_curve,
                        split,
                        tolerance,
                    )? {
                        curve
                    } else if let Some(curve) = concatenate_compatible_fitted_pieces(
                        &first_curve,
                        &second_curve,
                        split,
                        tolerance,
                    )? {
                        curve
                    } else {
                        continue;
                    };
                    let (start_vertex_id, _) = traversal_vertex_ids(first_edge, first);
                    let (_, end_vertex_id) = traversal_vertex_ids(second_edge, second);
                    let Some(start_point) = vertices.get(&start_vertex_id).copied() else {
                        continue;
                    };
                    let Some(end_point) = vertices.get(&end_vertex_id).copied() else {
                        continue;
                    };
                    let curve = constrain_curve_endpoints(curve, start_point, end_point)?;
                    let (mut face_pcurve, face_pcurve_rebuilt) = if let Some(pcurve) =
                        concatenate_exact_curve_pieces(
                            &first.pcurve,
                            &second.pcurve,
                            split,
                            tolerance,
                        )? {
                        (pcurve, false)
                    } else {
                        (build_pcurve_on_surface(&face.surface, &curve).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?, true)
                    };
                    let mate_loop = &faces[first_mate.face].loops[first_mate.loop_index];
                    let second_mate_coedge = &mate_loop.coedges[second_mate.coedge];
                    let first_mate_coedge = &mate_loop.coedges[first_mate.coedge];
                    let (mut mate_pcurve, mate_pcurve_rebuilt) = if let Some(pcurve) =
                        concatenate_exact_curve_pieces(
                            &second_mate_coedge.pcurve,
                            &first_mate_coedge.pcurve,
                            1.0 - split,
                            tolerance,
                        )? {
                        (pcurve, false)
                    } else {
                        (
                            build_pcurve_on_surface(
                                &faces[first_mate.face].surface,
                                &curve.reversed().or_refuse(KernelStage::Sew, "reversed")?,
                            ).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?,
                            true,
                        )
                    };
                    let mut pcurve_rebuilt = face_pcurve_rebuilt || mate_pcurve_rebuilt;
                    if !pcurve_matches_edge(&face.surface, &face_pcurve, &curve, false, tolerance)?
                    {
                        face_pcurve = concatenate_sampled_pieces(
                            &first.pcurve,
                            &second.pcurve,
                            split,
                            tolerance * 10.0,
                        )?
                        .unwrap_or(build_pcurve_on_surface(&face.surface, &curve).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?);
                        pcurve_rebuilt = true;
                    }
                    if !pcurve_matches_edge(
                        &faces[first_mate.face].surface,
                        &mate_pcurve,
                        &curve,
                        true,
                        tolerance,
                    )? {
                        mate_pcurve = concatenate_sampled_pieces(
                            &second_mate_coedge.pcurve,
                            &first_mate_coedge.pcurve,
                            1.0 - split,
                            tolerance * 10.0,
                        )?
                        .unwrap_or(build_pcurve_on_surface(
                            &faces[first_mate.face].surface,
                            &curve.reversed().or_refuse(KernelStage::Sew, "reversed")?,
                        ).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?);
                        pcurve_rebuilt = true;
                    }
                    if !pcurve_matches_edge(&face.surface, &face_pcurve, &curve, false, tolerance)?
                        || !pcurve_matches_edge(
                            &faces[first_mate.face].surface,
                            &mate_pcurve,
                            &curve,
                            true,
                            tolerance,
                        )?
                    {
                        // Mixed parameterizations (one pcurve concatenated
                        // exactly with a speed kink, the other rebuilt
                        // uniformly) cannot both track the curve linearly.
                        // Re-derive BOTH pcurves from the merged curve so all
                        // three share one parameterization by construction.
                        face_pcurve = build_pcurve_on_surface(&face.surface, &curve).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
                        mate_pcurve = build_pcurve_on_surface(
                            &faces[first_mate.face].surface,
                            &curve.reversed().or_refuse(KernelStage::Sew, "reversed")?,
                        ).or_refuse(KernelStage::Sew, "build_pcurve_on_surface")?;
                        pcurve_rebuilt = true;
                        // Both p-curves now come from the merged curve, so
                        // there is no third construction left to fall back to
                        // and nothing for the coarse screen to CHOOSE between.
                        // Its 1e-4 floor is tighter than the kernel's own
                        // pcurve contract by a factor of 40, and a rebuilt
                        // p-curve can sit a few 1e-4 off the equal-fraction
                        // station on a curved carrier while tracking the same
                        // locus (the tube joint's cylinder/cylinder edge in
                        // report 20260909T063157Z measures 3.5e-4 against a
                        // 4e-3 contract). Let the adaptive contract below —
                        // the same one final validation applies — arbitrate
                        // instead of vetoing here on the cheap screen.
                    }
                    // The local 32-point guard above is a cheap branch check.
                    // Before committing topology, enforce the same adaptive
                    // contract as final validation so a narrow fitted-curve
                    // error cannot be introduced by coalescing.
                    let [candidate_t0, candidate_t1] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
                    let candidate_edge = EdgeRecord {
                        id: first.edge_id,
                        curve: curve.clone(),
                        t0: candidate_t0,
                        t1: candidate_t1,
                        start_vertex_id,
                        end_vertex_id,
                        degenerate: false,
                        // A concatenated continuation keeps the first
                        // constituent's persistent name.
                        name: first_edge.name.clone(),
                    };
                    let pcurve_contract =
                        KernelTolerances::for_solid(&result, 1e-7).pcurve_consistency;
                    if adaptive_coedge_error(
                        &face.surface,
                        &face_pcurve,
                        &curve,
                        &candidate_edge,
                        true,
                        pcurve_contract,
                    ).or_refuse(KernelStage::Sew, "adaptive_coedge_error")? > pcurve_contract
                        || adaptive_coedge_error(
                            &faces[first_mate.face].surface,
                            &mate_pcurve,
                            &curve,
                            &candidate_edge,
                            false,
                            pcurve_contract,
                        ).or_refuse(KernelStage::Sew, "adaptive_coedge_error")? > pcurve_contract
                    {
                        rejected.insert(pair_key);
                        continue;
                    }
                    accepted = Some((
                        face_index,
                        loop_index,
                        first_index,
                        first_mate,
                        second_mate,
                        first.edge_id,
                        second.edge_id,
                        curve,
                        face_pcurve,
                        mate_pcurve,
                        pcurve_rebuilt,
                        start_vertex_id,
                        end_vertex_id,
                    ));
                    break 'faces;
                }
            }
        }
        let Some((
            face_index,
            loop_index,
            first_index,
            first_mate,
            second_mate,
            first_edge_id,
            second_edge_id,
            curve,
            face_pcurve,
            mate_pcurve,
            pcurve_rebuilt,
            start_vertex_id,
            end_vertex_id,
        )) = accepted
        else {
            break;
        };
        let before_face = parameter_space_area(faces[face_index]).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        let before_mate = parameter_space_area(faces[first_mate.face]).or_refuse(KernelStage::Sew, "parameter_space_area")?;
        drop(faces);
        drop(edges);
        let mut candidate = result.clone();
        let new_edge_id = candidate
            .edges
            .iter()
            .map(|edge| edge.id)
            .max()
            .unwrap_or(0)
            + 1;
        let domain = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
        let merged_name = candidate
            .edges
            .iter()
            .find(|edge| edge.id == first_edge_id)
            .and_then(|edge| edge.name.clone());
        candidate.edges.push(EdgeRecord {
            id: new_edge_id,
            curve,
            t0: domain[0],
            t1: domain[1],
            start_vertex_id,
            end_vertex_id,
            degenerate: false,
            name: merged_name,
        });
        let mut next_coedge_id = candidate
            .shells
            .iter()
            .flat_map(|shell| &shell.faces)
            .flat_map(|face| &face.loops)
            .flat_map(|loop_record| &loop_record.coedges)
            .map(|coedge| coedge.id)
            .max()
            .unwrap_or(0)
            + 1;
        let face_replacement = CoedgeRecord {
            id: next_coedge_id,
            edge_id: new_edge_id,
            forward: true,
            pcurve: face_pcurve,
        };
        next_coedge_id += 1;
        let mate_replacement = CoedgeRecord {
            id: next_coedge_id,
            edge_id: new_edge_id,
            forward: false,
            pcurve: mate_pcurve,
        };
        let mut candidate_faces = candidate
            .shells
            .iter_mut()
            .flat_map(|shell| shell.faces.iter_mut())
            .collect::<Vec<_>>();
        // A seam puts both runs on ONE loop, and a splice shortens the vector
        // under every index above the one it consumed. Doing the higher run
        // first leaves the lower run's index still pointing at its own pair —
        // and the wrap-around run, which starts at the last index, is the
        // higher one whenever it is present. Two runs on different loops do
        // not interact, so one order serves both cases.
        let splices = {
            let face_splice = (face_index, loop_index, first_index, face_replacement);
            let mate_splice = (
                first_mate.face,
                first_mate.loop_index,
                second_mate.coedge,
                mate_replacement,
            );
            if face_splice.0 == mate_splice.0
                && face_splice.1 == mate_splice.1
                && mate_splice.2 > face_splice.2
            {
                [mate_splice, face_splice]
            } else {
                [face_splice, mate_splice]
            }
        };
        for (splice_face, splice_loop, splice_index, replacement) in splices {
            replace_adjacent(
                &mut candidate_faces[splice_face].loops[splice_loop].coedges,
                splice_index,
                replacement,
            );
        }
        // Even exact pcurve concatenation changes the quadrature partition at
        // the removed knot. Allow its few-parts-per-million integration drift
        // while still rejecting material trim changes and branch crossings.
        let area_tolerance_ratio = if pcurve_rebuilt { 5e-3 } else { 3e-6 };
        let preserves_area = |before: f64, after: f64| {
            (after - before).abs() <= 1e-9f64.max(before.abs() * area_tolerance_ratio)
                && (before.abs() <= 1e-12 || before.is_sign_positive() == after.is_sign_positive())
        };
        if !preserves_area(
            before_face,
            parameter_space_area(candidate_faces[face_index]).or_refuse(KernelStage::Sew, "parameter_space_area")?,
        ) || !preserves_area(
            before_mate,
            parameter_space_area(candidate_faces[first_mate.face]).or_refuse(KernelStage::Sew, "parameter_space_area")?,
        ) {
            // This exact pair crosses a periodic p-curve branch. Leave it
            // split and continue searching for other safe candidates.
            rejected.insert(if first_edge_id < second_edge_id {
                (first_edge_id, second_edge_id)
            } else {
                (second_edge_id, first_edge_id)
            });
            continue;
        }
        drop(candidate_faces);
        candidate
            .edges
            .retain(|edge| edge.id != first_edge_id && edge.id != second_edge_id);
        let used_vertices = candidate
            .edges
            .iter()
            .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
            .collect::<rustc_hash::FxHashSet<_>>();
        candidate
            .vertices
            .retain(|vertex| used_vertices.contains(&vertex.id));
        // Offset-shell coalescing also runs while completion boundaries are
        // intentionally still open. Requiring closed-solid validation here
        // misattributes those pre-existing one-use edges to this local
        // rewrite. The completed result is validated at the kernel boundary.
        result = candidate;
    }
    Ok(result)
}

