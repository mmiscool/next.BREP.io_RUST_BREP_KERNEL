use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord};
use crate::offset_point::StationaryChart;
use crate::{fit, NurbsCurve, NurbsSurface, OffsetEvaluator, OffsetNormal, Vec3, Vec4};

/// Fixed uniform station count still used by the OPEN-edge and
/// edge-preserving marches (edge/support.rs, edge/keep.rs).  The CLOSED
/// general march is adaptive — see [`SEED_INTERVALS`] / [`MAX_STATIONS`].
pub(super) const STATIONS: usize = 64;

/// Ceiling for the OPEN march's measured refinement ([`STATIONS`] doubled
/// twice).  A stripe that still misses its bar here keeps its best rung rather
/// than refusing: the cap is a COST decision, not an accuracy one -- the rail
/// reading is still falling at 256 (1.419e-6 / 3.534e-7 / 3.019e-8 on the
/// notched cap's lobe) and the spans it buys are paid for by every stage
/// downstream.
pub(super) const MAX_OPEN_STATIONS: usize = 256;
pub(super) const NEWTON_ITERATIONS: usize = 24;
pub(super) const FIT_DEGREE: usize = 3;

/// Uniform seed intervals for sagitta-driven refinement. A span/16 step keeps
/// continuation inside the tangency Newton solver's convergence basin.
pub(super) const SEED_INTERVALS: usize = 16;

/// Shared depth budget for sagitta refinement and Newton step-halving retries.
pub(super) const MAX_REFINEMENT_DEPTH: usize = 4;

/// Maximum adaptive intervals; the station count is one greater. Intervals
/// reaching this cap retain their best available refinement.
pub(super) const MAX_STATIONS: usize = SEED_INTERVALS << MAX_REFINEMENT_DEPTH;

/// How an open march's local station refinement ended
/// ([`crate::blend::edge::march_open_stations_ending`]'s `refine_bar`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(in crate::blend) enum StationRefineExit {
    /// No refinement was asked for.
    #[default]
    NotAsked,
    /// Every checked interval's mid-span station sits within the bar of
    /// the cubic the neighbouring stations predict there.
    Converged,
    /// A round's worst miss did not fall below 0.9 of the previous round's.
    NoProgress,
    /// The rounds ran out with misses left.
    RoundBudget,
    /// Inserting the next round's stations would pass the station ceiling.
    StationCeiling,
}

/// What an open march's local refinement did: its exit, the stations it
/// inserted, and the worst mid-span miss still over the bar when it did not
/// converge (with the edge parameter where it sits).
#[derive(Clone, Copy, Debug, Default)]
pub(in crate::blend) struct StationRefinement {
    pub(in crate::blend) exit: StationRefineExit,
    pub(in crate::blend) inserted: usize,
    pub(in crate::blend) worst_miss: f64,
    pub(in crate::blend) worst_t: f64,
}

/// Rounds of local station refinement an open march may take: each halves
/// the spacing of the intervals that still miss, so six rounds reach 1/64 of
/// the uniform spacing where it is needed.
pub(in crate::blend) const REFINE_ROUNDS: usize = 6;

/// The station ceiling of a refined open march, as a multiple of its uniform
/// station count.
pub(in crate::blend) const REFINE_STATION_FACTOR: usize = 4;

#[derive(Clone, Copy, Debug)]
pub(super) struct Station {
    pub(super) uv1: [f64; 2],
    pub(super) uv2: [f64; 2],
    pub(super) p1: Vec3,
    pub(super) p2: Vec3,
    pub(super) center: Vec3,
    /// cos(α/2), the middle-row weight (4.9.3).
    pub(super) weight: f64,
    /// Tangent-plane intersection point in the section (the arc's middle
    /// control point before weighting).
    pub(super) apex: Vec3,
}

/// The blend march's normal convention: the bare `S_u × S_v` through the C¹
/// domain extension, with the orientation carried in the SIGN OF ρ rather than
/// in the face's `same_sense` (`signed_radii`). That is the shared evaluator's
/// [`OffsetNormal::Raw`] lane; this is its adapter.
pub(super) fn raw_normal(surface: &NurbsSurface, u: f64, v: f64) -> Result<Vec3, KernelRefusal> {
    blend_offset(surface).normal(u, v).or_refuse(KernelStage::Refine, "normal")
}

/// The march's pointwise offset evaluator for one support carrier.
///
/// `center = p + ρ·n` at `stations.rs`'s tangency residual IS an offset-surface
/// evaluation, and requiring the two supports' centres to coincide IS
/// intersecting the two offset surfaces — pointwise, without ever materializing
/// one. Routing it through the shared evaluator says so in code. The march must
/// NOT inherit [`OffsetNormal::FaceStable`]'s singular recovery — a nudged
/// normal would silently change a residual whose bar is `1e-11·(1 + scale)`.
/// Where `S_u × S_v` vanishes, the `Raw` lane reads the cross product's own
/// one-sided limit instead (a support extruded from a profile with a stationary
/// vertex is regular there), and refuses only where no regular point is in
/// reach. Past such a boundary its POINT moves along the same limit, so an
/// overshoot station seated beyond the vertex solves on the face's
/// continuation rather than on a Jacobian column of zeros. Near it the vanishing
/// partial is read from the control net's differences, so the normal's rounding
/// stays below that bar up to the isoline, and the station Newton steps that
/// parameter in a regular chart ([`station_charts`]).
pub(super) fn blend_offset(surface: &NurbsSurface) -> OffsetEvaluator<'_> {
    OffsetEvaluator::new("blend_march", surface, OffsetNormal::Raw)
}

/// The section plane at `t` of a march along `edge` whose stations are
/// `station_step` apart in `t`: the extended point and the edge's unit tangent,
/// read as the one-sided limit where the parameterization is stationary.
///
/// Past a STATIONARY open end the extended point does not move
/// (`P(end) + C'(end)·(t − end)` with `C'(end) = 0`), so every overshoot station
/// would solve on the vertex's own section plane: the chain window's chord
/// parameters repeat, and the open march's rows stand still past the vertex. The
/// overshoot exists to seat the rolling ball's contacts BEYOND the vertex, and a
/// stationary parameterization is a degenerate parameterization of a regular
/// curve, not a degenerate curve: the curve still has a direction there (the
/// one-sided limit) and only its speed is missing. So the section advances along
/// that limit by the chord the march was using, the distance between the vertex
/// station and the station one step inside it, per station step. That keeps the
/// station spacing continuous across the vertex; the regular twin's own
/// extension (a line at constant speed) is the same rule. A regular end is
/// untouched, to the bit. `site` names the march in a refusal.
pub(super) fn march_section(
    edge: &EdgeRecord,
    t: f64,
    station_step: f64,
    site: &str,
) -> Result<(Vec3, Vec3), KernelRefusal> {
    let named = |error: String| format!("{site}: edge {}: {error}", edge.id);
    let (point, tangent) = edge
        .curve
        .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
        .map_err(named).or_refuse(KernelStage::Refine, "map_err")?;
    let Some(boundary) = edge.curve.stationary_open_end(t).map_err(named).or_refuse(KernelStage::Refine, "map_err")? else {
        return Ok((point, tangent));
    };
    let [start, end] = edge.curve.domain().map_err(named).or_refuse(KernelStage::Refine, "map_err")?;
    let outward = if t < boundary { -1.0 } else { 1.0 };
    let previous = (boundary - outward * station_step).clamp(start, end);
    let chord = point
        .sub(edge.curve.evaluate(previous).map_err(named).or_refuse(KernelStage::Refine, "map_err")?)
        .length();
    let steps = (t - boundary).abs() / station_step;
    Ok((point.add(tangent.scale(outward * steps * chord)), tangent))
}

/// Evaluate a coedge's pcurve at the EDGE parameter, honouring traversal
/// direction, and return the face (u, v).
pub(super) fn edge_uv_on_face(
    coedge: &CoedgeRecord,
    edge: &EdgeRecord,
    t: f64,
) -> Result<[f64; 2], KernelRefusal> {
    let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
    let span = edge.t1 - edge.t0;
    let fraction = if span.abs() <= 1e-15 {
        0.0
    } else {
        ((t - edge.t0) / span).clamp(0.0, 1.0)
    };
    let parameter = if coedge.forward {
        p0 + (p1 - p0) * fraction
    } else {
        p1 - (p1 - p0) * fraction
    };
    let point = coedge.pcurve.evaluate(parameter).or_refuse(KernelStage::Refine, "evaluate")?;
    Ok([point.x, point.y])
}

pub(super) struct BlendMate<'a> {
    pub(super) face: &'a FaceRecord,
    pub(super) coedge: &'a CoedgeRecord,
    pub(super) loop_index: usize,
    /// Signed radius ρ: center = p + ρ·n_raw(p).
    pub(super) rho: f64,
}

/// Full evaluation of the tangency system (4.9.2) + (4.9.5) at
/// uv = (u, v, a, b): the residual plus every by-product the station
/// construction needs — contact points, ball center, and the RAW surface
/// normals the residual computed anyway.  The solvers hand back the LAST
/// converged evaluation so accepted stations are built without
/// re-evaluating either surface (OCCT's `ComputeValues` unchanged-argument
/// guard, filleting study §6 lesson 8).
pub(super) struct TangencyState {
    pub(super) residual: [f64; 4],
    pub(super) p1: Vec3,
    pub(super) p2: Vec3,
    pub(super) center: Vec3,
    pub(super) n1: Vec3,
    pub(super) n2: Vec3,
}

fn tangency_state(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    uv: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
) -> Result<TangencyState, KernelRefusal> {
    // The two offset evaluations the whole residual is built from. Each is ONE
    // surface evaluation: `deriv1_extended` returns the point together with the
    // partials the normal needs, and its point is bit-identical to
    // `evaluate_extended` because
    // `derivatives_extended(..,0)[0][0] == derivatives_extended(..,1)[0][0]`.
    let first = blend_offset(first).at(uv[0], uv[1], rho[0]).or_refuse(KernelStage::Refine, "at")?;
    let second = blend_offset(second).at(uv[2], uv[3], rho[1]).or_refuse(KernelStage::Refine, "at")?;
    // Coincident ball centres = the two offset surfaces intersect here.
    let center = first.point;
    let mismatch = center.sub(second.point);
    let plane = center.sub(section_point).dot(section_tangent);
    Ok(TangencyState {
        residual: [mismatch.x, mismatch.y, mismatch.z, plane],
        p1: first.source,
        p2: second.source,
        center,
        n1: first.normal,
        n2: second.normal,
    })
}

/// Residual of (4.9.2) + (4.9.5) at uv = (u, v, a, b).
pub(super) fn tangency_residual(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    uv: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
) -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
    let state = tangency_state(first, second, rho, uv, section_point, section_tangent)?;
    Ok((state.residual, state.p1, state.p2, state.center))
}

/// Newton on (u, v, a, b) with the section plane fixed, returning the
/// converged unknowns together with the tangency state of the FINAL
/// evaluation, so callers never re-evaluate a solved station.
fn solve_station_state(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    seed: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
    scale: f64,
) -> Result<([f64; 4], TangencyState), KernelRefusal> {
    if !newton_trace_on() {
        return solve_station_state_traced(first, second, rho, seed, section_point, section_tangent, scale, None);
    }
    let mut trace = NewtonTrace::default();
    let result = solve_station_state_traced(first, second, rho, seed, section_point, section_tangent, scale, Some(&mut trace));
    emit_newton_trace(seed, 1e-11 * (1.0 + scale), &trace, &result);
    result
}

/// The station Newton itself. `trace`, when given, only RECORDS what each
/// iteration read (its residual, the Jacobian's conditioning, the exit); it is
/// never read back, so a traced solve takes exactly the steps an untraced one
/// does.
#[allow(clippy::too_many_arguments)]
fn solve_station_state_traced(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    seed: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
    scale: f64,
    mut trace: Option<&mut NewtonTrace>,
) -> Result<([f64; 4], TangencyState), KernelRefusal> {
    let mut uv = seed;
    let tolerance = 1e-11 * (1.0 + scale);
    for _ in 0..NEWTON_ITERATIONS {
        let state = match tangency_state(first, second, rho, uv, section_point, section_tangent) {
            Ok(state) => state,
            Err(error) => {
                if let Some(trace) = trace.as_deref_mut() {
                    trace.exit = "state_unreadable";
                }
                return Err(error);
            }
        };
        let error = state
            .residual
            .iter()
            .map(|value| value.abs())
            .fold(0.0, f64::max);
        if let Some(trace) = trace.as_deref_mut() {
            // The decision below reads the folded max, which DROPS a NaN
            // component (`f64::max`); the trace keeps each component's own
            // finiteness, read on the raw residual, beside it.
            trace.residuals.push(error);
            trace.components_finite.push(state.residual.iter().all(|value| value.is_finite()));
        }
        if error <= tolerance {
            if let Some(trace) = trace.as_deref_mut() {
                trace.exit = "converged";
                trace.solution = Some(uv);
            }
            return Ok((uv, state));
        }
        let mut jacobian = [[0.0f64; 4]; 4];
        let step = 1e-7;
        for column in 0..4 {
            let mut probe = uv;
            probe[column] += step;
            let probed = tangency_state(first, second, rho, probe, section_point, section_tangent)?;
            for row in 0..4 {
                jacobian[row][column] = (probed.residual[row] - state.residual[row]) / step;
            }
        }
        let charts = station_charts([first, second], uv, &jacobian)?;
        for (column, chart) in charts.iter().enumerate() {
            let Some(chart) = chart else {
                continue;
            };
            let mut probe = uv;
            probe[column] = chart.parameter(chart.regular(uv[column]) + step);
            let probed = tangency_state(first, second, rho, probe, section_point, section_tangent)?;
            for row in 0..4 {
                jacobian[row][column] = (probed.residual[row] - state.residual[row]) / step;
            }
        }
        if let Some(trace) = trace.as_deref_mut() {
            trace.conditioning = Some(jacobian_conditioning(&jacobian));
            trace.charts = charts.iter().filter(|chart| chart.is_some()).count();
        }
        let delta = match fit::solve_small::<4>(jacobian, state.residual, 4) {
            Ok(delta) => delta,
            Err(error) => {
                if let Some(trace) = trace.as_deref_mut() {
                    trace.exit = "singular";
                }
                return Err(format!("blend: station Newton is singular ({error})")).or_refuse(KernelStage::Refine, "solve_small");
            }
        };
        for ((value, correction), chart) in uv.iter_mut().zip(delta).zip(charts) {
            match chart {
                Some(chart) => *value = chart.parameter(chart.regular(*value) - correction),
                None => *value -= correction,
            }
        }
    }
    if let Some(trace) = trace.as_deref_mut() {
        trace.exit = "iterations_spent";
        trace.solution = Some(uv);
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "tangency_newton", "blend: tangency Newton did not converge"))
}

// ====================================================================
// `BREP_DEBUG_STATION_NEWTON=1`: a bounded trace of every station Newton
// ====================================================================

/// Where a station Newton runs, as its caller names it: the march `phase`
/// (`open-initial`, `open-march`, `open-refine`, `open-dense`, ...), the
/// station `index` and section parameter `t`, the edge's NATIVE domain (so an
/// overshoot station reads as outside it), and the interval `[left, t]` a
/// continuation solves across. `stage` is set by [`continue_station`]:
/// `continuation` for the plain neighbour-seeded attempt, `retry` for every
/// halved-seed attempt after a branch hop. Diagnostic only.
#[derive(Clone, Copy, Debug, Default)]
pub(in crate::blend) struct NewtonContext {
    pub(in crate::blend) phase: &'static str,
    pub(in crate::blend) stage: &'static str,
    pub(in crate::blend) index: Option<usize>,
    pub(in crate::blend) t: Option<f64>,
    pub(in crate::blend) native: Option<[f64; 2]>,
    pub(in crate::blend) interval: Option<[f64; 2]>,
    pub(in crate::blend) depth: usize,
}

thread_local! {
    static NEWTON_CONTEXT: std::cell::Cell<Option<NewtonContext>> = const { std::cell::Cell::new(None) };
}

/// What one traced Newton read: every iteration's max residual, the last
/// Jacobian's conditioning, the charts it stepped in, how it exited and where.
#[derive(Default)]
struct NewtonTrace {
    residuals: Vec<f64>,
    /// Per iteration: whether EVERY raw residual component was finite (the
    /// folded max in `residuals` cannot say; `f64::max` discards a NaN).
    components_finite: Vec<bool>,
    conditioning: Option<JacobianConditioning>,
    charts: usize,
    exit: &'static str,
    solution: Option<[f64; 4]>,
}

/// The Jacobian's conditioning, read on a copy: the smallest and largest
/// column norms, and the Hadamard ratio |det J| / prod(column norms) in
/// [0, 1] (0: singular, 1: orthogonal columns), with whether every entry was
/// finite.
#[derive(Clone, Copy, Debug)]
struct JacobianConditioning {
    finite: bool,
    column_min: f64,
    column_max: f64,
    hadamard: f64,
}

fn jacobian_conditioning(jacobian: &[[f64; 4]; 4]) -> JacobianConditioning {
    let finite = jacobian.iter().flatten().all(|value| value.is_finite());
    let norms: Vec<f64> = (0..4).map(|column| (0..4).map(|row| jacobian[row][column].powi(2)).sum::<f64>().sqrt()).collect();
    let mut a = *jacobian;
    let mut det = 1.0_f64;
    for column in 0..4 {
        let pivot = (column..4).max_by(|&x, &y| a[x][column].abs().total_cmp(&a[y][column].abs())).unwrap_or(column);
        if pivot != column {
            a.swap(pivot, column);
            det = -det;
        }
        let head = a[column][column];
        det *= head;
        if head == 0.0 {
            break;
        }
        for row in column + 1..4 {
            let factor = a[row][column] / head;
            for k in column..4 {
                a[row][k] -= factor * a[column][k];
            }
        }
    }
    let product: f64 = norms.iter().product();
    JacobianConditioning {
        finite,
        column_min: norms.iter().copied().fold(f64::INFINITY, f64::min),
        column_max: norms.iter().copied().fold(0.0, f64::max),
        hadamard: if product > 0.0 { det.abs() / product } else { 0.0 },
    }
}

/// Whether station Newtons are traced: `BREP_DEBUG_STATION_NEWTON=1`, read
/// once, or (tests only) the calling thread's capture.
pub(in crate::blend) fn newton_trace_on() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_DEBUG_STATION_NEWTON").ok().as_deref() == Some("1"))
}

/// Run `body` with `context` as the station Newtons' context (the previous
/// one restored after). A no-op wrapper when tracing is off.
pub(in crate::blend) fn with_newton_context<R>(context: NewtonContext, body: impl FnOnce() -> R) -> R {
    if !newton_trace_on() {
        return body();
    }
    // Restored on EVERY exit -- a return, an error, or a panic unwinding
    // through `body` -- so a context never leaks into a later solve.
    struct Restore(Option<NewtonContext>);
    impl Drop for Restore {
        fn drop(&mut self) {
            NEWTON_CONTEXT.with(|cell| cell.set(self.0));
        }
    }
    let _restore = Restore(NEWTON_CONTEXT.with(|cell| cell.replace(Some(context))));
    body()
}

/// The current context with `edit` applied (tracing on only).
fn newton_context_edited(edit: impl FnOnce(&mut NewtonContext)) -> NewtonContext {
    let mut context = NEWTON_CONTEXT.with(|cell| cell.get()).unwrap_or_default();
    edit(&mut context);
    context
}

/// Emit one bounded line per traced solve (at most
/// `BREP_DEBUG_STATION_NEWTON_MAX` lines a process, default 20000).
fn emit_newton_trace(seed: [f64; 4], tolerance: f64, trace: &NewtonTrace, result: &Result<([f64; 4], TangencyState), KernelRefusal>) {
    static LINES: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
    static MAX: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    let context = NEWTON_CONTEXT.with(|cell| cell.get()).unwrap_or_default();
    let inside = match (context.t, context.native) {
        (Some(t), Some([a, b])) => Some(t >= a.min(b) && t <= a.max(b)),
        _ => None,
    };
    let line = format!(
        "STATION_NEWTON phase {} stage {} index {:?} t {:?} native {:?} inside {inside:?} interval {:?} depth {} \
         seed {seed:?} exit {} iterations {} residuals {:?} tolerance {tolerance:.3e} folded_max_finite {} \
         components_finite {} first_nonfinite_component_iteration {:?} \
         jacobian {:?} charts {} solution {:?} ok {}",
        if context.phase.is_empty() { "-" } else { context.phase },
        if context.stage.is_empty() { "-" } else { context.stage },
        context.index,
        context.t,
        context.native,
        context.interval,
        context.depth,
        // A Jacobian probe or chart query that refused leaves no exit of its own.
        if !trace.exit.is_empty() { trace.exit } else if result.is_ok() { "converged" } else { "probe_or_chart_error" },
        trace.residuals.len(),
        trace.residuals,
        trace.residuals.iter().all(|value| value.is_finite()),
        trace.components_finite.iter().all(|finite| *finite),
        trace.components_finite.iter().position(|finite| !*finite),
        trace.conditioning,
        trace.charts,
        trace.solution,
        result.is_ok(),
    );
    let max = *MAX.get_or_init(|| std::env::var("BREP_DEBUG_STATION_NEWTON_MAX").ok().and_then(|value| value.parse().ok()).unwrap_or(20000));
    let count = LINES.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    if count < max {
        eprintln!("{line}");
    } else if count == max {
        eprintln!("STATION_NEWTON truncated after {max} lines (BREP_DEBUG_STATION_NEWTON_MAX)");
    }
}


/// A column whose reach over its carrier's domain is below this fraction of the
/// same carrier's other column is small enough to ask whether a stationary
/// isoline made it so. A gate on cost only: the answer is
/// `OffsetEvaluator::stationary_chart`'s.
const STATION_CHART_REACH: f64 = 0.1;

/// Which of the station Newton's four unknowns `(u, v, a, b)` step in a
/// [`StationaryChart`] this iteration rather than in the carrier's own parameter.
///
/// The station at a stationary VERTEX — an edge whose parameterization stops
/// there, so the extruded support carries that parameter as an isoline with
/// `S_u = 0` — has its contact ON that isoline, a root of multiplicity two (for
/// coincident end poles) in the support's `u`. Solved in `u`, the Newton halves
/// its distance per iteration on a column that shrinks with it. In the chart the
/// point moves at unit speed through the isoline and on along the `Raw` lane's
/// continuation past it, so the column does not vanish. On the open tombstone
/// fixtures that is three steps from the neighbour's seed to 1e-14, where the
/// parameter took 14 and 16 (with the `Raw` normal's rounding fixed) or never
/// settled (without it: 7.5e-11..3.8e-9 against the 5e-11 bar).
///
/// Asked per iteration from the Jacobian already built: a column is a candidate
/// when its reach (norm × domain span) is small against its carrier's other
/// column, and the chart decides. Everywhere else every unknown steps in its own
/// parameter, to the bit.
fn station_charts(
    surfaces: [&NurbsSurface; 2],
    uv: [f64; 4],
    jacobian: &[[f64; 4]; 4],
) -> Result<[Option<StationaryChart>; 4], KernelRefusal> {
    let mut charts = [None; 4];
    let reach = |column: usize, span: f64| -> f64 {
        jacobian
            .iter()
            .map(|row| row[column] * row[column])
            .sum::<f64>()
            .sqrt()
            * span
    };
    for (index, surface) in surfaces.into_iter().enumerate() {
        let [u_column, v_column] = [2 * index, 2 * index + 1];
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let reach_u = reach(u_column, u1 - u0);
        let reach_v = reach(v_column, v1 - v0);
        for (column, along_u, small) in [
            (u_column, true, reach_u <= STATION_CHART_REACH * reach_v),
            (v_column, false, reach_v <= STATION_CHART_REACH * reach_u),
        ] {
            if small {
                charts[column] =
                    blend_offset(surface).stationary_chart(along_u, uv[u_column], uv[v_column]).or_refuse(KernelStage::Refine, "stationary_chart")?;
            }
        }
    }
    Ok(charts)
}

/// Newton on (u, v, a, b) with the section plane fixed.
pub(super) fn solve_station(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    seed: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
    scale: f64,
) -> Result<[f64; 4], KernelRefusal> {
    solve_station_state(
        first,
        second,
        rho,
        seed,
        section_point,
        section_tangent,
        scale,
    )
    .map(|(uv, _)| uv)
}

/// [`solve_station`] that also returns the converged ball centre, read off
/// the solver's final tangency evaluation instead of re-evaluating both
/// surfaces.  The uniform marches feed it to [`branch_hop`].
pub(super) fn solve_station_centered(
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho: [f64; 2],
    seed: [f64; 4],
    section_point: Vec3,
    section_tangent: Vec3,
    scale: f64,
) -> Result<([f64; 4], Vec3), KernelRefusal> {
    solve_station_state(
        first,
        second,
        rho,
        seed,
        section_point,
        section_tangent,
        scale,
    )
    .map(|(uv, state)| (uv, state.center))
}

/// One converged station of a march, as the continuation guard reads it:
/// its section parameter, its solve and the ball centre the solve rests on.
/// A pole station of the open march is set from the edge's pcurves rather
/// than solved, so its centre may be unreadable; the centre bar is skipped
/// across such a step and the period bar still applies.
#[derive(Clone, Copy)]
pub(super) struct Continued {
    pub(super) t: f64,
    pub(super) uv: [f64; 4],
    pub(super) center: Option<Vec3>,
}

/// Is the converged station `right` on ANOTHER branch of the tangency
/// system than its neighbour `left`?  `Some(reason)` when it is.
///
/// The tangency Newton is undamped, and a converged answer is only a root
/// of the system — not necessarily the one the march is following.  A
/// section plane can cut the ball-centre locus more than once (an
/// elongated intersection loop is cut again across its own width), and
/// from a seed one station back the first Newton step can fly many
/// carrier periods before settling on the far crossing.  Two readings say
/// a node is on another branch:
///
/// * A contact moved half a period or more round a CLOSED carrier
///   direction.  The 2026-09-23 skew-cylinder report's far-branch root read
///   11.7 periods on the closed-edge march.
/// * The ball CENTRE moved further than the edge chord plus the ball's
///   diameter.  The same document's second loop at radius 1.5 hopped
///   within one period — support 1 moved 0.34 of a lap, the centre 8.7
///   units for an edge chord under one.  On the open march of a 20°
///   crossing's notched crotch arc (2026-09-26) the hop read support 2
///   moved 0.85 of a period and the centre 9.9 for a 2.1 chord, where the
///   root on the followed branch was 0.86 away.
///
/// Neither bar is a theorem about every blend: a sharp crotch swings the
/// centre round the edge faster than the edge advances, and a coarse seed
/// step there can exceed either (a 34° crossing's chain march read a
/// centre step of 3.9 for a 3.0 diameter at its coarsest rung).  Every
/// caller therefore answers a trip with a RETRY from a nearer seed and
/// never with a refusal: [`MarchFrame::continue_node`] bisects its
/// interval, [`continue_station`] halves its seed.
///
/// The edge is read continued past its ends (`point_and_unit_tangent_extended`):
/// the anchored closed march runs t once round past t1, and the open march
/// overshoots both vertices.  A carrier that cannot answer a closure or
/// domain query is not checked here; the solve itself already evaluated
/// it, so any real fault surfaced there.  An unreadable chord skips the
/// centre bar the same way.
pub(super) fn branch_hop(
    surfaces: [&NurbsSurface; 2],
    edge: &EdgeRecord,
    rho_at: &dyn Fn(f64) -> [f64; 2],
    left: &Continued,
    right: &Continued,
) -> Option<KernelRefusal> {
    for (index, surface) in surfaces.into_iter().enumerate() {
        let (Ok((closed_u, closed_v)), Ok(domain_u), Ok(domain_v)) =
            (surface.closed_directions(), surface.domain_u(), surface.domain_v())
        else {
            continue;
        };
        for (direction, closed, domain) in [(0, closed_u, domain_u), (1, closed_v, domain_v)] {
            let period = domain[1] - domain[0];
            let column = 2 * index + direction;
            let step = (right.uv[column] - left.uv[column]).abs();
            if closed && step >= 0.5 * period {
                return Some(KernelRefusal::non_convergence(
                    KernelStage::Refine,
                    "branch_hop",
                    format!(
                        "blend: the station Newton converged on another branch between t={:.9} \
                         and t={:.9} — support {} moved {:.3} of a period round its closed \
                         carrier in one step",
                        left.t,
                        right.t,
                        index + 1,
                        step / period,
                    ),
                ));
            }
        }
    }
    let (Some(left_center), Some(right_center)) = (left.center, right.center) else {
        return None;
    };
    let point = |t: f64| {
        edge.curve
            .point_and_unit_tangent_extended(t, edge.t0, edge.t1)
            .map(|(point, _)| point)
    };
    let chord = point(right.t).and_then(|right| Ok(right.sub(point(left.t)?).length()));
    let Ok(chord) = chord else {
        return None;
    };
    let [rho_left, _] = rho_at(left.t);
    let [rho_right, _] = rho_at(right.t);
    let diameter = 2.0 * rho_left.abs().max(rho_right.abs());
    let moved = right_center.sub(left_center).length();
    if moved > chord + diameter {
        return Some(KernelRefusal::non_convergence(
            KernelStage::Refine,
            "branch_hop",
            format!(
                "blend: the station Newton converged on another branch between t={:.9} \
                 and t={:.9} — the ball centre moved {moved:.6} for an edge chord of \
                 {chord:.6} and a ball diameter of {diameter:.6}",
                left.t, right.t
            ),
        ));
    }
    None
}

/// Solve the station at `t` of a UNIFORM march as the continuation of its
/// converged neighbour `left`, halving the seed toward it when the answer
/// is on another branch ([`branch_hop`]).
///
/// The uniform marches (`chain/march.rs`, `edge/support.rs`) keep their
/// station grid, so unlike [`MarchFrame::march_interval`] a retry inserts
/// no station: the halfway solve is only a nearer SEED for the station
/// asked for, itself continued from `left` under the same guard.  The
/// first attempt is the plain neighbour-seeded solve every station took
/// before the guard existed, so a march that never hops is unchanged to
/// the bit.  Both recursions share one `depth` budget of
/// [`MAX_REFINEMENT_DEPTH`]; at the cap the node is taken as before — the
/// guard adds retries, never a refusal — and a Newton that does not
/// converge surfaces as it always did.
pub(super) fn continue_station(
    solve: &dyn Fn(f64, [f64; 4]) -> Result<([f64; 4], Vec3), KernelRefusal>,
    hop: &dyn Fn(&Continued, &Continued) -> Option<KernelRefusal>,
    left: &Continued,
    t: f64,
    depth: usize,
) -> Result<Continued, KernelRefusal> {
    let context = newton_context_edited(|context| {
        context.stage = if depth == 0 { "continuation" } else { "retry" };
        context.t = Some(t);
        context.interval = Some([left.t, t]);
        context.depth = depth;
    });
    let (uv, center) = with_newton_context(context, || solve(t, left.uv))?;
    let node = Continued { t, uv, center: Some(center) };
    if depth >= MAX_REFINEMENT_DEPTH {
        return Ok(node);
    }
    let Some(reason) = hop(left, &node) else {
        return Ok(node);
    };
    // `BREP_BLEND_STATION_TRACE=1` names every retry, so a march's station
    // trace can be read against the hops it was guarded from.
    if std::env::var("BREP_BLEND_STATION_TRACE").ok().as_deref() == Some("1") {
        eprintln!("blend guard: depth {depth}, seed halved — {reason}");
    }
    let halfway = continue_station(solve, hop, left, 0.5 * (left.t + t), depth + 1)?;
    continue_station(solve, hop, &halfway, t, depth + 1)
}

/// Anchored start: one surface parameter frozen on a seam meridian while
/// the section plane floats along the edge — unknowns are t plus the
/// three unfrozen surface parameters.  `lock_index` is the frozen slot in
/// (u, v, a, b): 0 anchors the FIRST carrier's meridian, 2 the SECOND's.
#[allow(clippy::too_many_arguments)]
fn solve_anchored_start(
    edge: &EdgeRecord,
    first: &NurbsSurface,
    second: &NurbsSurface,
    rho_at: &dyn Fn(f64) -> [f64; 2],
    lock_index: usize,
    u_lock: f64,
    seed_t: f64,
    seed_uv: [f64; 4],
    scale: f64,
) -> Result<(f64, [f64; 4], TangencyState), KernelRefusal> {
    let free: [usize; 3] = match lock_index {
        0 => [1, 2, 3],
        2 => [0, 1, 3],
        _ => return Err(KernelRefusal::internal(KernelStage::Refine, "anchor_lock", "blend: unsupported anchor lock index")),
    };
    let mut x = [seed_t, seed_uv[free[0]], seed_uv[free[1]], seed_uv[free[2]]];
    let tolerance = 1e-11 * (1.0 + scale);
    let assemble = |x: &[f64; 4]| -> [f64; 4] {
        let mut uv = [0.0f64; 4];
        uv[lock_index] = u_lock;
        uv[free[0]] = x[1];
        uv[free[1]] = x[2];
        uv[free[2]] = x[3];
        uv
    };
    let state_at = |x: &[f64; 4]| -> Result<TangencyState, KernelRefusal> {
        let (section_point, section_tangent) = edge
            .curve
            .point_and_unit_tangent_extended(x[0], edge.t0, edge.t1)
            .map_err(|error| format!("blend anchored start: edge {}: {error}", edge.id)).or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")?;
        tangency_state(
            first,
            second,
            rho_at(x[0]),
            assemble(x),
            section_point,
            section_tangent,
        )
    };
    for _ in 0..NEWTON_ITERATIONS {
        let state = state_at(&x)?;
        let error = state
            .residual
            .iter()
            .map(|value| value.abs())
            .fold(0.0, f64::max);
        if error <= tolerance {
            return Ok((x[0], assemble(&x), state));
        }
        let mut jacobian = [[0.0f64; 4]; 4];
        let step = 1e-7;
        for column in 0..4 {
            let mut probe = x;
            probe[column] += step;
            let probed = state_at(&probe)?;
            for row in 0..4 {
                jacobian[row][column] = (probed.residual[row] - state.residual[row]) / step;
            }
        }
        let delta = fit::solve_small::<4>(jacobian, state.residual, 4)
            .map_err(|error| format!("blend: anchored-start Newton is singular ({error})")).or_refuse(KernelStage::Refine, "solve_small")?;
        for (value, correction) in x.iter_mut().zip(delta) {
            *value -= correction;
        }
    }
    // At an exact periodic seam endpoint the section parameter is already
    // known.  The four-variable system can nevertheless stall because the
    // rational closed edge has two one-sided parameterisations at that point:
    // the contact mismatch converges while the finite-difference t column
    // leaves a tiny section-plane residual.  Pin the known endpoint and solve
    // the three independent center-mismatch equations for the three remaining
    // surface parameters.  Acceptance still requires ALL FOUR original
    // residuals, so this cannot hide a genuinely wrong anchor.
    let mut fixed = [x[1], x[2], x[3]];
    let fixed_state = |values: &[f64; 3]| -> Result<TangencyState, KernelRefusal> {
        let full = [seed_t, values[0], values[1], values[2]];
        state_at(&full)
    };
    let fixed_plane_tolerance = 2e-6 * (1.0 + scale);
    for _ in 0..NEWTON_ITERATIONS {
        let state = fixed_state(&fixed)?;
        let mismatch_error = state.residual[..3]
            .iter()
            .map(|value| value.abs())
            .fold(0.0, f64::max);
        if mismatch_error <= tolerance && state.residual[3].abs() <= fixed_plane_tolerance {
            let full = [seed_t, fixed[0], fixed[1], fixed[2]];
            return Ok((seed_t, assemble(&full), state));
        }
        let mut jacobian = [[0.0f64; 3]; 3];
        let step = 1e-7;
        for column in 0..3 {
            let mut probe = fixed;
            probe[column] += step;
            let probed = fixed_state(&probe)?;
            for row in 0..3 {
                jacobian[row][column] =
                    (probed.residual[row] - state.residual[row]) / step;
            }
        }
        let mismatch = [state.residual[0], state.residual[1], state.residual[2]];
        let delta = fit::solve_small::<3>(jacobian, mismatch, 3)
            .map_err(|error| format!("blend: fixed-seam Newton is singular ({error})")).or_refuse(KernelStage::Refine, "solve_small")?;
        for (value, correction) in fixed.iter_mut().zip(delta) {
            *value -= correction;
        }
    }
    if std::env::var("BREP_DEBUG_BLEND_MARCH").is_ok() {
        if let Ok(state) = fixed_state(&fixed) {
            eprintln!(
                "fixed-seam Newton failed: t={seed_t:.9} values={fixed:?} residual={:?}",
                state.residual
            );
        }
        if let Ok(state) = state_at(&x) {
            eprintln!(
                "anchored Newton failed: lock_index={lock_index} lock={u_lock:.9} \
                 seed_t={seed_t:.9} final=({:.9},{:.9},{:.9},{:.9}) residual={:?}",
                x[0], x[1], x[2], x[3], state.residual
            );
        }
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "anchored_seam_start", "blend: anchored seam start did not converge"))
}

pub(super) fn apex_point(
    p1: Vec3,
    n1: Vec3,
    p2: Vec3,
    n2: Vec3,
    center: Vec3,
) -> Result<Vec3, KernelRefusal> {
    let section_normal = p2
        .sub(p1)
        .cross(center.sub(p1))
        .normalized()
        .map_err(|_| "blend: degenerate section (tangency points collapsed)".to_string()).or_refuse(KernelStage::Refine, "map_err")?;
    let matrix = [
        [n1.x, n1.y, n1.z],
        [n2.x, n2.y, n2.z],
        [section_normal.x, section_normal.y, section_normal.z],
    ];
    let rhs = [n1.dot(p1), n2.dot(p2), section_normal.dot(p1)];
    let solution = fit::solve_small::<3>(matrix, rhs, 3)
        .map_err(|_| "blend: tangent planes are parallel in the section".to_string()).or_refuse(KernelStage::Refine, "map_err")?;
    Ok(Vec3::new(solution[0], solution[1], solution[2]))
}

/// One accepted station of the adaptive march: the edge parameter it was
/// solved at, the converged unknowns (the continuation seed for the next
/// solve), and the finished section sample.
#[derive(Clone)]
struct MarchNode {
    t: f64,
    uv: [f64; 4],
    station: Station,
}

/// The prefix of the refusal a march makes when the ball's tangency has left
/// a carrier's own surface — see [`MarchFrame::check_supports_on_carriers`].
/// Callers match it to tell an ESCAPED march from a merely non-converging one:
/// the fallbacks that answer a non-convergence (the edge-preserving
/// construction, the tool cutter) build a blend the rolling ball never made,
/// so an escape is reported as-is instead.
pub(crate) const BALL_OFF_CARRIER: &str = "blend: no ball of radius";

/// The slug of that refusal (`UnsupportedGeometry`).
pub(crate) const BALL_OFF_CARRIER_WHAT: &str = "ball_off_carrier";

/// Is this the escaped march's refusal ([`BALL_OFF_CARRIER`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_ball_off_carrier(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::UnsupportedGeometry { what } if what == BALL_OFF_CARRIER_WHAT)
}

/// Shared context of one closed-edge march.
struct MarchFrame<'a> {
    edge: &'a EdgeRecord,
    surface1: &'a NurbsSurface,
    surface2: &'a NurbsSurface,
    rho_at: &'a dyn Fn(f64) -> [f64; 2],
    signs: [f64; 2],
    scale: f64,
    /// Committed fit accuracy the sagitta refinement drives toward — see
    /// the derivation comment in [`march_stations`].
    fit_tolerance: f64,
    /// Depth budget shared by the sagitta refinement and the Newton
    /// step-halving retries: [`MAX_REFINEMENT_DEPTH`] for every march that
    /// refuses a fold, raised rung by rung by the closed-edge lane's CARVE
    /// ladder, whose wall has to stand on the rolling ball between stations
    /// to the network's rail bar before its fold can be cut (the chain lane
    /// climbs its own per-segment ladder for the same reason).
    depth_budget: usize,
}

impl MarchFrame<'_> {
    /// Refuse a converged solve whose contact point is not on the carrier at
    /// all.
    ///
    /// `evaluate_extended` continues an OPEN parameter direction along the
    /// boundary tangent plane, without bound — a required capability, because
    /// the tangency Newton and the projections legitimately probe just past a
    /// trim (a support that runs off its face across a G1 edge overshoots by a
    /// fraction of the patch, and the surgery still trims it there).  Far
    /// outside the domain that continuation is a fictitious plane, and the
    /// solver will happily rest the ball on it: when the radius is larger than
    /// the local wall the true tangency system has NO solution, so the Newton
    /// slides down the degenerate ridge onto the extension and converges to a
    /// station whose contact is nowhere near the solid.  That station passes
    /// every downstream residual and poisons the fit — the reported 2026-09-10
    /// document came back "control points must be finite with positive
    /// weights" from the rational row fit, and "ambiguous preserved boundary
    /// edge" from the fallback the escape routed to.
    ///
    /// One full domain span outside is the sanity bound: it is not a
    /// tolerance to tune but the difference between an overshoot onto a
    /// neighbour (measured 4e-3 of the span on the reported edge's first
    /// station) and an escape (283 spans on its middle ones).  CLOSED
    /// directions are exempt — the march deliberately unwraps a periodic
    /// carrier's parameter and every image is the same real surface.
    fn check_supports_on_carriers(&self, t: f64, uv: [f64; 4]) -> Result<(), KernelRefusal> {
        for (index, surface, uv) in [
            (1usize, self.surface1, [uv[0], uv[1]]),
            (2usize, self.surface2, [uv[2], uv[3]]),
        ] {
            let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
            for (axis, closed, value, domain) in [
                ("u", closed_u, uv[0], surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?),
                ("v", closed_v, uv[1], surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?),
            ] {
                let span = domain[1] - domain[0];
                if closed || (value >= domain[0] - span && value <= domain[1] + span) {
                    continue;
                }
                let radius = (self.rho_at)(t)[index - 1].abs();
                // The anchored march runs t PAST t1 (it starts on the seam
                // meridian and closes one lap later), so the station's own
                // parameter has to come back into the edge's domain before it
                // names a point: `evaluate` would clamp it to the seam and
                // report the wrong end of the lap.
                let point = self.edge.curve.evaluate(self.wrapped(t)).unwrap_or_default();
                return Err(KernelRefusal::unsupported(KernelStage::Refine, BALL_OFF_CARRIER_WHAT, format!(
                    "{BALL_OFF_CARRIER} {radius} is tangent to both faces at \
                     ({:.6}, {:.6}, {:.6}): the contact ran off carrier {index}'s \
                     surface ({axis} = {value:.6} outside [{:.6}, {:.6}])",
                    point.x, point.y, point.z, domain[0], domain[1]
                )));
            }
        }
        Ok(())
    }

    /// A march parameter brought back into the edge's own domain: closed edges
    /// wrap (the anchored march closes a full lap past `t1`), open ones clamp.
    fn wrapped(&self, t: f64) -> f64 {
        let [t0, t1] = [self.edge.t0, self.edge.t1];
        if self.edge.start_vertex_id == self.edge.end_vertex_id && t1 > t0 {
            t0 + (t - t0).rem_euclid(t1 - t0)
        } else {
            t.clamp(t0, t1)
        }
    }

    /// Finish a converged solve into a station, reusing the solver's final
    /// tangency evaluation (points, center, normals) instead of
    /// re-evaluating both surfaces.
    fn node(&self, t: f64, uv: [f64; 4], state: &TangencyState) -> Result<MarchNode, KernelRefusal> {
        self.check_supports_on_carriers(t, uv)?;
        let cos_alpha = self.signs[0] * self.signs[1] * state.n1.dot(state.n2);
        let weight = ((1.0 + cos_alpha) * 0.5).max(0.0).sqrt();
        if weight <= 1e-6 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "tangent_faces", "blend: faces are tangent at a station (α = π)"));
        }
        let apex = apex_point(state.p1, state.n1, state.p2, state.n2, state.center)?;
        Ok(MarchNode {
            t,
            uv,
            station: Station {
                uv1: [uv[0], uv[1]],
                uv2: [uv[2], uv[3]],
                p1: state.p1,
                p2: state.p2,
                center: state.center,
                weight,
                apex,
            },
        })
    }

    /// Solve one station at `t`.  `lock` freezes the FIRST carrier's u on a
    /// seam meridian and frees t instead (the anchored endpoint contract).
    /// The seed is a neighbouring converged solution, but that alone does
    /// not keep the uv track continuous: an undamped Newton can still hop to
    /// another root — see [`MarchFrame::continue_node`], which the march
    /// solves through.
    fn solve_node(&self, t: f64, seed: [f64; 4], lock: Option<f64>) -> Result<MarchNode, KernelRefusal> {
        if let Some(u_lock) = lock {
            let (t_solved, uv, state) = solve_anchored_start(
                self.edge,
                self.surface1,
                self.surface2,
                self.rho_at,
                0,
                u_lock,
                t,
                seed,
                self.scale,
            )?;
            self.node(t_solved, uv, &state)
        } else {
            let (section_point, section_tangent) = self
                .edge
                .curve
                .point_and_unit_tangent_extended(t, self.edge.t0, self.edge.t1)
                .map_err(|error| format!("blend march: edge {}: {error}", self.edge.id)).or_refuse(KernelStage::Refine, "point_and_unit_tangent_extended")?;
            let (uv, state) = solve_station_state(
                self.surface1,
                self.surface2,
                (self.rho_at)(t),
                seed,
                section_point,
                section_tangent,
                self.scale,
            )?;
            self.node(t, uv, &state)
        }
    }

    /// Should the interval (left, right) keep its midpoint as a station?
    /// Checked on every row the fit commits: the two rails, the ball
    /// center, and the section apex (the middle-row control point).
    fn needs_refinement(&self, left: &Station, probe: &Station, right: &Station) -> bool {
        [
            (left.p1, probe.p1, right.p1),
            (left.p2, probe.p2, right.p2),
            (left.center, probe.center, right.center),
            (left.apex, probe.apex, right.apex),
        ]
        .into_iter()
        .any(|(a, m, b)| {
            let chord = b.sub(a);
            let chord_sq = chord.dot(chord);
            let offset = m.sub(a);
            // Sagitta: distance from the midpoint sample to the chord line
            // (falling back to the plain offset when the chord collapses —
            // a stalled track with a displaced midpoint must refine).
            let sagitta = if chord_sq > 0.0 {
                offset.cross(chord).length() / chord_sq.sqrt()
            } else {
                offset.length()
            };
            // Three consecutive samples estimate the local track curvature
            // through their circumscribed circle, κ ≈ 8·s/ℓ² (s = sagitta,
            // ℓ = chord).  The committed rows are degree-3 interpolants; a
            // cubic's maximum error over one spacing-ℓ interval of a
            // curvature-κ track is ≈ (5/384)·ℓ⁴·|f⁗| ≈ ℓ⁴·κ³/77, which in
            // the measured quantities is ≈ 8·s³/ℓ².  Refine while that
            // estimate exceeds the committed fit accuracy.
            8.0 * sagitta * sagitta * sagitta > self.fit_tolerance * chord_sq
        })
    }

    /// Solve the station at `t` as the CONTINUATION of the converged station
    /// `(left_t, left_uv)`, refusing a root on another branch.
    ///
    /// The tangency Newton is undamped, and a converged answer is only a root
    /// of the system — not necessarily the one the march is following.  The
    /// 2026-09-23 reported document — a thin cylinder unioned through a fat
    /// one at 34° — hopped past the loop's sharp crotch: from t = 0.625 the
    /// first step moved u₁ by 8.5 periods and the solve converged on the
    /// other side of the loop, 7 units from its neighbour.  Every station
    /// after it walked the wrong branch, the anchored closing station hopped
    /// back, and the rows fitted through both branches ran through the
    /// material the interference gate then found.
    ///
    /// The two readings are [`branch_hop`], shared with the uniform marches.
    /// Either refuses the node like a Newton that did not converge: the
    /// interval bisects and the solve is retried from a nearer seed.  At the
    /// depth cap (`enforce` false) the node is taken as before: halving can
    /// no longer help, and where no nearby root exists at all — a ball over
    /// its ceiling, whose Newton escapes onto a carrier's extension — the
    /// escape guard downstream names the geometry
    /// (`inbox_20260910_fillet_mouth_over_wall` pins that message), which a
    /// branch-hop refusal here would pre-empt.  A trip only ever bisects:
    /// this check adds retries, never a refusal.
    fn continue_node(
        &self,
        left_t: f64,
        left_uv: [f64; 4],
        left_center: Vec3,
        t: f64,
        lock: Option<f64>,
        enforce: bool,
    ) -> Result<MarchNode, KernelRefusal> {
        let node = self.solve_node(t, left_uv, lock)?;
        if !enforce {
            return Ok(node);
        }
        let left = Continued { t: left_t, uv: left_uv, center: Some(left_center) };
        let right = Continued { t: node.t, uv: node.uv, center: Some(node.station.center) };
        match branch_hop([self.surface1, self.surface2], self.edge, self.rho_at, &left, &right) {
            Some(reason) => Err(reason),
            None => Ok(node),
        }
    }

    /// Solve and append every station in (nodes.last().t, right_t], in
    /// strictly increasing parameter order.  An interval is bisected when
    /// its midpoint sagitta says the fitted rows would miss the tolerance
    /// (the probe becomes a station) or when the continuation Newton cannot
    /// cross it in one step (OCCT's step-halving retry, walking study §1
    /// stage 4).  Both recursions share one `depth` budget, so a march can
    /// never exceed SEED_INTERVALS·2^MAX_REFINEMENT_DEPTH = MAX_STATIONS
    /// intervals; at the cap the best-effort grid stands and only a
    /// persistent Newton failure surfaces (named error, no fallback).
    fn march_interval(
        &self,
        nodes: &mut Vec<MarchNode>,
        right_t: f64,
        lock: Option<f64>,
        presolved: Option<MarchNode>,
        depth: usize,
    ) -> Result<(), KernelRefusal> {
        let (left_t, left_uv, left_center) = {
            let left = nodes
                .last()
                .expect("march interval requires a solved left station");
            (left.t, left.uv, left.station.center)
        };
        // A right station solved before subdivision may belong to a root
        // reached from the old coarse left seed. Once the left interval has
        // refined, continue from its actual last node instead of committing
        // that stale solution. Reuse the existing branch reading; do not add
        // a continuation bar or change the depth-cap geometry policy.
        let mut reseeded = false;
        let mut attempt: Option<u64> = None;
        let presolved = presolved.filter(|node| {
            let left = Continued { t: left_t, uv: left_uv, center: Some(left_center) };
            let right = Continued { t: node.t, uv: node.uv, center: Some(node.station.center) };
            let reuse = branch_hop([self.surface1, self.surface2], self.edge, self.rho_at, &left, &right).is_none();
            reseeded = !reuse;
            if reseeded {
                attempt = Some(next_reseed_attempt());
                trace_closed_reseed(attempt, self.edge.id, left_t, node.t, depth, lock, "attempt", true);
            }
            reuse
        });
        let right = match presolved {
            Some(node) => node,
            None => match self.continue_node(
                left_t,
                left_uv,
                left_center,
                right_t,
                lock,
                depth < self.depth_budget,
            ) {
                Ok(node) => node,
                Err(error) => {
                    if depth >= self.depth_budget {
                        self.trace_reseed_terminal(attempt, nodes, left_t, right_t, depth, lock, "declined_at_cap", Err(&error));
                        return Err(error);
                    }
                    if attempt.is_some() {
                        trace_closed_reseed(attempt, self.edge.id, left_t, right_t, depth, lock, "deferred_newton", true);
                    }
                    let first = self.march_interval(nodes, 0.5 * (left_t + right_t), None, None, depth + 1);
                    let result = first.and_then(|()| self.march_interval(nodes, right_t, lock, None, depth + 1));
                    self.trace_reseed_terminal(attempt, nodes, left_t, right_t, depth, lock, "descendant_newton", result.as_ref().map(|_| ()));
                    return result;
                }
            },
        };
        if reseeded && reseed_census_enabled() {
            let left = Continued { t: left_t, uv: left_uv, center: Some(left_center) };
            let current = Continued { t: right.t, uv: right.uv, center: Some(right.station.center) };
            let incoherent = branch_hop([self.surface1, self.surface2], self.edge, self.rho_at, &left, &current).is_some();
            trace_closed_reseed(attempt, self.edge.id, left_t, right.t, depth, lock, "resolved", incoherent);
        }
        if depth < self.depth_budget {
            let mid_t = 0.5 * (left_t + right.t);
            match self.continue_node(left_t, left_uv, left_center, mid_t, None, true) {
                Ok(probe) => {
                    let split = self.needs_refinement(
                        &nodes.last().expect("left station").station,
                        &probe.station,
                        &right.station,
                    );
                    if split {
                        let right_t = right.t;
                        let first = self.march_interval(nodes, mid_t, None, Some(probe), depth + 1);
                        let result = first.and_then(|()| self.march_interval(nodes, right_t, lock, Some(right), depth + 1));
                        self.trace_reseed_terminal(attempt, nodes, left_t, right_t, depth, lock, "descendant_sagitta", result.as_ref().map(|_| ()));
                        return result;
                    }
                }
                Err(_) => {
                    // The sagitta probe would not converge from the
                    // interval's left seed, so the interval cannot be
                    // certified flat: subdivide into it.  The recursion
                    // re-attempts the SAME solve once per level (cheap and
                    // deterministic) before halving toward it with closer
                    // seeds; if the failure persists at the depth cap the
                    // named Newton error surfaces.
                    let right_t = right.t;
                    let first = self.march_interval(nodes, mid_t, None, None, depth + 1);
                    let result = first.and_then(|()| self.march_interval(nodes, right_t, lock, Some(right), depth + 1));
                    self.trace_reseed_terminal(attempt, nodes, left_t, right_t, depth, lock, "descendant_probe", result.as_ref().map(|_| ()));
                    return result;
                }
            }
        }
        nodes.push(right);
        self.trace_reseed_terminal(attempt, nodes, left_t, right_t, depth, lock, "direct", Ok(()));
        Ok(())
    }

    /// Read-only terminal provenance for a re-seeded cached station: after the
    /// interval returns, the right end it actually installed (the free-t anchor
    /// node when `lock` applies), its installed predecessor, and the existing
    /// branch reading between them. Never feeds back into acceptance or order.
    #[allow(clippy::too_many_arguments)]
    fn trace_reseed_terminal(
        &self,
        attempt: Option<u64>,
        nodes: &[MarchNode],
        left_t: f64,
        right_t: f64,
        depth: usize,
        lock: Option<f64>,
        path: &str,
        result: Result<(), &KernelRefusal>,
    ) {
        let Some(attempt) = attempt else { return };
        if !reseed_census_enabled() {
            return;
        }
        let mut row = serde_json::json!({
            "attempt": attempt, "edge": self.edge.id, "left_t": left_t, "right_t": right_t,
            "depth": depth, "lock": lock, "phase": "terminal", "path": path,
        });
        match result {
            Err(error) => {
                row["outcome"] = serde_json::json!("declined");
                row["error"] = serde_json::json!(error.to_string());
            }
            Ok(()) => {
                row["outcome"] = serde_json::json!("installed");
                if let [.., before, installed] = nodes {
                    let left = Continued { t: before.t, uv: before.uv, center: Some(before.station.center) };
                    let right = Continued { t: installed.t, uv: installed.uv, center: Some(installed.station.center) };
                    let hop = branch_hop([self.surface1, self.surface2], self.edge, self.rho_at, &left, &right);
                    row["installed_t"] = serde_json::json!(installed.t);
                    row["installed_center"] = serde_json::json!([installed.station.center.x, installed.station.center.y, installed.station.center.z]);
                    row["predecessor_t"] = serde_json::json!(before.t);
                    row["predecessor_center"] = serde_json::json!([before.station.center.x, before.station.center.y, before.station.center.z]);
                    row["step"] = serde_json::json!(installed.station.center.sub(before.station.center).length());
                    row["incoherent"] = serde_json::json!(hop.is_some());
                    row["hop"] = serde_json::json!(hop.map(|reason| reason.to_string()));
                }
            }
        }
        write_reseed_row(row);
    }
}

/// Optional read-only census of cached stations re-solved after refinement.
/// The post-solve coherence reading never feeds back into acceptance.
#[allow(clippy::too_many_arguments)]
fn trace_closed_reseed(attempt: Option<u64>, edge: u64, left_t: f64, right_t: f64, depth: usize, lock: Option<f64>, phase: &str, incoherent: bool) {
    if !reseed_census_enabled() {
        return;
    }
    write_reseed_row(serde_json::json!({
        "attempt":attempt,"edge":edge,"left_t":left_t,"right_t":right_t,"depth":depth,
        "lock":lock,"phase":phase,"incoherent":incoherent
    }));
}

/// Whether anything reads the census: the environment file, or a private
/// test sink. Checked before any row or branch reading is built.
fn reseed_census_enabled() -> bool {
    #[cfg(not(test))]
    let sink = false;
    sink || std::env::var_os("BREP_CLOSED_RESEED_CENSUS").is_some()
}

/// Correlates every census row of one re-seeded station; numbering only.
fn next_reseed_attempt() -> u64 {
    static NEXT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    NEXT.fetch_add(1, std::sync::atomic::Ordering::Relaxed)
}

/// One census row, written whole with a single `write_all` so rows from
/// parallel marches never splice.
fn write_reseed_row(row: serde_json::Value) {
    #[cfg(not(test))]
    let captured = false;
    if captured {
        return;
    }
    if let Ok(path) = std::env::var("BREP_CLOSED_RESEED_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let _ = file.write_all(format!("{row}\n").as_bytes());
        }
    }
}


/// Characteristic length of a blend march: the LOCAL model scale of the
/// geometry actually being solved.
///
/// The march is a *local* solve — one edge, its two supports, and a rolling
/// ball — so the length its residual bars are measured against is the extent of
/// that neighbourhood, not the extent of the whole solid and emphatically not
/// its distance from the world origin (see [`crate::model_scale`]).  Two terms:
///
/// * the marched edge's own extent ([`crate::curve_model_scale`]), and
/// * the blend radius, because a fat ball on a short edge puts the contact
///   points and the spine `|ρ|` away from the edge — there the ball, not the
///   edge, sets the size of the geometry being solved.
///
/// Floored at 1.0, exactly as the origin-distance form it replaces was, so the
/// small end of the corpus keeps the bands it was tuned against and this change
/// moves ONE thing: placement-dependence.
pub(super) fn march_model_scale(
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    radius: f64,
) -> Result<f64, KernelRefusal> {
    Ok(crate::curve_model_scale(curve, t0, t1).or_refuse(KernelStage::Refine, "curve_model_scale")?
        .max(radius.abs())
        .max(1.0))
}

/// March the closed edge once around with ADAPTIVE, sagitta-driven station
/// placement (OCCT `BRepBlend_Walking` deflection control — filleting
/// study §1 stage 4, §6 lesson 4).  Returns the stations in parameter
/// order; the last is the first again (u-coordinates unwrapped
/// continuously, so on a closed carrier the final u differs from the first
/// by the carrier period).  The endpoint stations sit exactly at t_start /
/// t_start + span (or, when anchored, exactly ON the seam meridian with t
/// freed — unchanged from the fixed grid), and the count adapts between
/// SEED_INTERVALS + 1 and MAX_STATIONS + 1 with the spine's curvature.
/// Fallback eligibility follows the solve phase, never the refusal's wording.
/// Once local refinement starts, keep its anchors and its original typed error.
#[derive(Debug)]
pub(super) enum ClosedMarchFailure {
    Initial(KernelRefusal),
    Refinement(KernelRefusal),
}
impl From<KernelRefusal> for ClosedMarchFailure {
    fn from(error: KernelRefusal) -> Self { Self::Initial(error) }
}
impl ClosedMarchFailure {
    fn into_refusal(self) -> KernelRefusal {
        match self { Self::Initial(error) | Self::Refinement(error) => error }
    }
    pub(super) fn into_initial(self) -> Result<KernelRefusal, KernelRefusal> {
        match self { Self::Initial(error) => Ok(error), Self::Refinement(error) => Err(error) }
    }
}

pub(super) fn march_stations(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius_at: &dyn Fn(f64) -> f64,
    anchor_u: Option<f64>,
    fold: super::chain::FoldPolicy,
    depth_budget: usize,
) -> Result<Vec<Station>, KernelRefusal> {
    march_stations_impl(edge, first, second, radius_at, anchor_u, fold, depth_budget, None, None)
        .map_err(ClosedMarchFailure::into_refusal)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn march_stations_with_rail_bar(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius_at: &dyn Fn(f64) -> f64,
    anchor_u: Option<f64>,
    fold: super::chain::FoldPolicy,
    depth_budget: usize,
    rail_bar: f64,
    wall: Option<ClosedWallCheck>,
) -> Result<Vec<Station>, ClosedMarchFailure> {
    march_stations_impl(edge, first, second, radius_at, anchor_u, fold, depth_budget, Some(rail_bar), wall)
}

/// The closed-edge FILLET's wall verdict: a constant-radius rolling ball's
/// wall is judged between its rails against the ball's own sweep, at the
/// same declared reading and probes as the closed chain's
/// (`chain::declared_wall_probe`), against the unchanged `bar`
/// (`intersection_fit`). A chamfer, or a varying radius, has no single ball
/// to judge against and passes `None`.
#[derive(Clone, Copy, Debug)]
pub(super) struct ClosedWallCheck {
    pub(super) radius: f64,
    pub(super) bar: f64,
    /// The construction request (`KernelTolerances::model`) a PLAIN fillet's
    /// accepted wall is then refined toward, as the closed chain's is; `None`
    /// judges acceptance only.
    pub(super) model: Option<f64>,
}

thread_local! {
    /// The closed-edge march's unmet construction request, for the lane to
    /// report once its body is built: (declared reading, why it stopped,
    /// local rounds ATTEMPTED, stations of the rung that SHIPS, station
    /// ceiling). On `Rejected` the attempted round is one past the kept rung;
    /// the stations and reading are the kept rung's. Written by the march,
    /// taken by `blend_closed_edge_impl`.
    /// The closed-edge request's last reading: (worst rail over the request
    /// fractions, unreadable rail intervals, worst declared wall), for the
    /// report's detail. Written by the march, taken by `blend_closed_edge_impl`.
    pub(super) static CLOSED_REQUEST_RAILS: std::cell::Cell<Option<(f64, usize, f64)>> = const { std::cell::Cell::new(None) };
    pub(super) static CLOSED_MODEL_UNMET: std::cell::Cell<Option<(f64, crate::BudgetReason, usize, usize, usize)>> =
        const { std::cell::Cell::new(None) };
}

/// Centres per interval for the closed-edge wall verdict (probes at the even
/// ones, as the chain's `2 * LOCAL_RAIL_STENCIL`).
const CLOSED_WALL_CENTRES: usize = 32;
/// Section-angle probes across the wall, k/8 for k = 1..7.
const CLOSED_WALL_SECTIONS: usize = 8;

/// Every interval's largest DECLARED wall reading (reading + uncertainty)
/// on the fitted fillet `rows`, `None` where any centre did not solve or any
/// probe was unreadable (which fails the interval). Each interval's exact ball
/// centres are continued node to node from its left station, under the
/// march's own branch guard.
fn closed_wall_interval_misses(
    rows: &FittedRows,
    nodes: &[MarchNode],
    parameters: &[f64],
    radius: f64,
    solve: impl Fn(&MarchNode, f64) -> Result<MarchNode, KernelRefusal>,
) -> Vec<Option<f64>> {
    let [low, high] = rows.u_domain;
    let [v_low, v_high] = match rows.surface.domain_v() {
        Ok(domain) => domain,
        Err(_) => return vec![None; nodes.len().saturating_sub(1)],
    };
    (0..nodes.len().saturating_sub(1))
        .map(|interval| {
            let (left, right) = (&nodes[interval], &nodes[interval + 1]);
            let mut centres = Vec::with_capacity(CLOSED_WALL_CENTRES + 1);
            centres.push(left.station.center);
            let mut previous = left.clone();
            for step in 1..CLOSED_WALL_CENTRES {
                let t = left.t + (right.t - left.t) * step as f64 / CLOSED_WALL_CENTRES as f64;
                previous = solve(&previous, t).ok()?;
                centres.push(previous.station.center);
            }
            centres.push(right.station.center);
            let (from, to) = (parameters[interval], parameters[interval + 1]);
            let mut declared = 0.0_f64;
            for step in (2..CLOSED_WALL_CENTRES).step_by(2) {
                let u = low + (high - low) * (from + (to - from) * step as f64 / CLOSED_WALL_CENTRES as f64);
                for k in 1..CLOSED_WALL_SECTIONS {
                    let v = v_low + (v_high - v_low) * k as f64 / CLOSED_WALL_SECTIONS as f64;
                    let point = rows.surface.evaluate(u, v).ok()?;
                    let (reading, uncertainty) =
                        super::chain::declared_wall_probe_located(point, &centres, step, step as f64 / CLOSED_WALL_CENTRES as f64, radius)?;
                    declared = declared.max(reading + uncertainty);
                }
            }
            declared.is_finite().then_some(declared)
        })
        .collect()
}

fn march_stations_impl(
    edge: &EdgeRecord,
    first: &BlendMate,
    second: &BlendMate,
    radius_at: &dyn Fn(f64) -> f64,
    anchor_u: Option<f64>,
    fold: super::chain::FoldPolicy,
    depth_budget: usize,
    rail_bar: Option<f64>,
    wall: Option<ClosedWallCheck>,
) -> Result<Vec<Station>, ClosedMarchFailure> {
    let span = edge.t1 - edge.t0;
    let radius_extent = [0.0, 0.5, 1.0]
        .into_iter()
        .map(|fraction| radius_at(edge.t0 + span * fraction).abs())
        .fold(0.0, f64::max);
    let scale = march_model_scale(&edge.curve, edge.t0, edge.t1, radius_extent)?;
    crate::report_scale_migration("blend_march_closed", scale, || {
        let a = edge.curve.evaluate(edge.t0).unwrap_or_default();
        let b = edge.curve.evaluate(edge.t0 + span * 0.5).unwrap_or_default();
        a.length().max(b.length()).max(1.0)
    });
    let surface1 = &first.face.surface;
    let surface2 = &second.face.surface;
    let signs = [first.rho.signum(), second.rho.signum()];
    let rho_at = |t: f64| -> [f64; 2] {
        let radius = radius_at(t);
        [signs[0] * radius, signs[1] * radius]
    };
    let u_span = {
        let [u0, u1] = surface1.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        u1 - u0
    };
    // Anchoring: find the edge parameter whose tangency point sits on ANY
    // periodic image of the F1 seam meridian, then distribute stations from
    // there once around. Boolean pcurves are deliberately allowed to stay
    // unwrapped (for example [0.25, 1.25]); comparing only against the base
    // domain start would choose the wrong meridian and make the anchored
    // Newton cross most of the loop before it could converge.
    let (t_start, anchor_u) = if let Some(base_lock) = anchor_u {
        // Seed t: where the EDGE pcurve on F1 crosses the seam meridian.
        let mut best = (edge.t0, f64::INFINITY, base_lock);
        for index in 0..=64 {
            let t = edge.t0 + span * index as f64 / 64.0;
            let uv = edge_uv_on_face(first.coedge, edge, t)?;
            let lap = ((uv[0] - base_lock) / u_span).round();
            let lock = base_lock + lap * u_span;
            let distance = (uv[0] - lock).abs();
            if distance < best.1 {
                best = (t, distance, lock);
            }
        }
        (best.0, Some(best.2))
    } else {
        (edge.t0, None)
    };
    let mut seed = {
        let t = t_start.clamp(edge.t0, edge.t1);
        let uv1 = edge_uv_on_face(first.coedge, edge, t)?;
        let uv2 = edge_uv_on_face(second.coedge, edge, t)?;
        [uv1[0], uv1[1], uv2[0], uv2[1]]
    };
    if let Some(u_lock) = anchor_u {
        seed[0] = u_lock;
    }
    if std::env::var("BREP_DEBUG_BLEND_MARCH").is_ok() {
        eprintln!(
            "anchor seed: t={t_start:.9} lock={anchor_u:?} uv={seed:?} u_span={u_span:.9}"
        );
    }
    // Refinement bar: the kernel's committed fit accuracy, from the SAME
    // policy the blending pipeline instantiates at its tolerance sites
    // (fillet/edges.rs, fillet/tool.rs pass model = 1e-7).
    // `intersection_fit` is the "maximum geometric error accepted while
    // fitting" band — the accuracy the marched-and-fitted rows commit to —
    // NOT OCCT's fixed 0.88/0.98 walking constants.  `scale` is this module's
    // march scale ([`march_model_scale`] — the marched edge's own extent, the
    // same convention the Newton and closure tolerances use); its only effect
    // here is the ladder's scale·1e-8 floor, which the 20·model term dominates
    // for anything smaller than ~200 units ACROSS.  (It read "within ~200 units
    // of the origin" until audit slice 0, when the scale stopped depending on
    // where the part sits and started depending on how big it is.)
    let fit_tolerance = crate::KernelTolerances::for_scale(scale, 1e-7).intersection_fit;
    let frame = MarchFrame {
        edge,
        surface1,
        surface2,
        rho_at: &rho_at,
        signs,
        scale,
        fit_tolerance,
        depth_budget,
    };
    let mut nodes: Vec<MarchNode> = Vec::with_capacity(SEED_INTERVALS + 1);
    nodes.push(frame.solve_node(t_start, seed, anchor_u)?);
    for index in 1..=SEED_INTERVALS {
        let t = t_start + span * index as f64 / SEED_INTERVALS as f64;
        // Anchored marches free t and freeze u on the seam meridian at BOTH
        // endpoints so both ends of the support pcurve land exactly on the
        // seam; the closing lock advances by the carrier period travelled.
        let lock = if anchor_u.is_some() && index == SEED_INTERVALS {
            let u_direction = (nodes[1].uv[0] - nodes[0].uv[0]).signum();
            anchor_u.map(|u| u + u_direction * u_span)
        } else {
            None
        };
        frame.march_interval(&mut nodes, t, lock, None, 0)?;
    }
    // Read-only census: distinguish closing-lock winding from curve/edge wrapping.
    // This flag never changes the marched stations or any acceptance decision.
    if std::env::var_os("BREP_TRACE_CLOSED_STATIONS").is_some() {
        let edge_uv0 = edge_uv_on_face(first.coedge, edge, edge.t0);
        let edge_uv1 = edge_uv_on_face(first.coedge, edge, edge.t1);
        eprintln!("C3-CLOSE-RANGE edge={} edge_range=[{:.17e},{:.17e}] curve_domain={:?} t_start={:.17e} intended_end={:.17e} anchor={:?} u_span={:.17e} edge_uv0={:?} edge_uv1={:?}",
            edge.id, edge.t0, edge.t1, edge.curve.domain(), t_start, t_start + span, anchor_u, u_span, edge_uv0, edge_uv1);
        for fraction in [1e-6, 1e-3, 0.1] {
            let delta = span * fraction;
            let past = edge.curve.point_and_unit_tangent_extended(edge.t1 + delta, edge.t0, edge.t1);
            let wrapped = edge.curve.point_and_unit_tangent_extended(edge.t0 + delta, edge.t0, edge.t1);
            eprintln!("C3-CLOSE-WRAP edge={} fraction={fraction:.17e} past={past:?} edge_wrapped={wrapped:?}", edge.id);
        }
        for (index, node) in nodes.iter().enumerate() {
            match edge.curve.point_and_unit_tangent_extended(node.t, edge.t0, edge.t1) {
                Ok((point, tangent)) => eprintln!("C3-CLOSE-SPINE {}", serde_json::json!({
                    "edge":edge.id, "index":index, "t":node.t, "spine":point,
                    "tangent":tangent, "weight":node.station.weight, "radius":radius_at(node.t),
                    "reach_ratio":node.station.center.sub(point).length()*node.station.weight/radius_at(node.t).abs()
                })),
                Err(error) => eprintln!("C3-CLOSE-SPINE-ERROR edge={} index={index} {error}", edge.id),
            }
            eprintln!("C3-CLOSE-NODE edge={} index={index} t={:.17e} uv={:?} p1={:?} p2={:?} center={:?}",
                edge.id, node.t, node.uv, node.station.p1, node.station.p2, node.station.center);
        }
    }
    debug_assert!(
        nodes.len() <= (SEED_INTERVALS << depth_budget) + 1,
        "adaptive march exceeded its hard station cap"
    );
    // Does the wall this march would carry FOLD? The centre curve is the same
    // whichever lane marches it, so the closed-edge march asks the same
    // question the chain march does (`blend/fold.rs`) — measured by re-solving
    // the tangency system either side of each station, never off the station
    // spacing, which on a bend the size of the radius reads clean.  Under
    // `FoldPolicy::Carve` the caller has already read the fold and is about to
    // carve the folded lens out of the wall (`blend/carve.rs`), so the march
    // builds it.
    let check_fold = |nodes: &[MarchNode]| -> Result<(), KernelRefusal> {
      if fold == super::chain::FoldPolicy::Refuse {
        let probe = |t: f64,
                     seed: [f64; 4]|
         -> Result<([f64; 4], Vec3, Vec3, Vec3), KernelRefusal> {
            let node = frame.solve_node(t, seed, None)?;
            Ok((node.uv, node.station.p1, node.station.p2, node.station.center))
        };
        let probed: Vec<(f64, [f64; 4])> = nodes.iter().map(|node| (node.t, node.uv)).collect();
        let fold_radius = |t: f64| radius_at(t).abs();
        super::fold::check_wall_fold(
            &fold_radius,
            span,
            &probed,
            &probe,
            edge,
            [first.face, second.face],
            scale,
        )?;
    }
      Ok(())
    };
    check_fold(&nodes)?;
    let mut refinement_started = false;
    if let Some(bar) = rail_bar {
        let initial: Vec<Station> = nodes.iter().map(|node| node.station).collect();
        // Analytic revolution rows already carry the exact rail geometry.
        if exact_closed_revolution_rows(&initial, first, second, false).is_none() {
            refinement_started = true;
            (|| -> Result<(), KernelRefusal> {
            let original_count = nodes.len();
            let station_ceiling = (SEED_INTERVALS << (MAX_REFINEMENT_DEPTH + 4)) + 1;
            // The rung last ACCEPTED (rails and wall inside their bars) while
            // the construction request refines past it, with its declared
            // wall reading; and why the request stopped short, if it did.
            let mut accepted: Option<(Vec<MarchNode>, f64)> = None;
            let mut unmet: Option<(f64, crate::BudgetReason, usize, usize)> = None;
            for round in 0..=REFINE_ROUNDS {
                let stations: Vec<Station> = nodes.iter().map(|node| node.station).collect();
                let parameters = station_parameters(&stations);
                // The two rails are independent of the rational middle row.
                // Fitting the chamfer rows here lets rail refinement proceed
                // even when a coarse fillet middle row has negative weights.
                let rows = fit_closed_rows(&stations, &parameters, true)?;
                if std::env::var_os("BREP_TRACE_CLOSED_RAIL_FEET").is_some() {
                    trace_closed_rail_feet(&rows, &stations, &parameters,
                        [&first.face.surface, &second.face.surface]);
                }
                let misses = closed_rail_interval_misses(&rows, &stations, &parameters,
                    [&first.face.surface, &second.face.surface])?;
                let worst = misses.iter().copied().fold(0.0_f64, f64::max);
                let mut count = misses.iter().filter(|&&miss| miss > bar).count();
                if std::env::var("BREP_DEBUG_CLOSED_RAILS").is_ok() {
                    eprintln!("closed local rails: edge {} round {round} original {original_count} stations {} failing {count} worst {worst:.9e} bar {bar:.9e} anchor={anchor_u:?}", edge.id, nodes.len());
                }
                // Rails on their carriers, a FILLET's wall is judged between
                // them against the ball's own sweep (`ClosedWallCheck`): the
                // rails can stand on their carriers while the wall stands off
                // the ball (the 20-degree crossing's single closed exit edge
                // shipped a wall 4.98e-6 off the exact sweep against the 2e-6
                // bar on c5ea8fb80, read input-only). An interval whose
                // declared reading is over the bar, or unreadable, refines
                // like a rail interval; one that passes adds no station, so a
                // wall already inside the bar builds exactly as it did.
                let mut wall_failing: Vec<bool> = Vec::new();
                let mut wall_worst = 0.0_f64;
                let mut walls_read: Option<Vec<Option<f64>>> = None;
                if count == 0 {
                    if let Some(check) = wall {
                        match fit_closed_rows(&stations, &parameters, false) {
                            Ok(fillet) => {
                                let walls = closed_wall_interval_misses(&fillet, &nodes, &parameters, check.radius, |previous, t| {
                                    frame.continue_node(previous.t, previous.uv, previous.station.center, t, None, true)
                                });
                                wall_failing = walls.iter().map(|wall| !matches!(wall, Some(declared) if *declared <= check.bar)).collect();
                                wall_worst = walls.iter().flatten().copied().fold(0.0_f64, f64::max);
                                count = wall_failing.iter().filter(|&&failing| failing).count();
                                if std::env::var("BREP_DEBUG_CLOSED_RAILS").is_ok() {
                                    eprintln!("closed local wall: edge {} round {round} stations {} failing {count} ({} unreadable) worst declared {wall_worst:.9e} bar {:.9e}",
                                        edge.id, nodes.len(), walls.iter().filter(|wall| wall.is_none()).count(), check.bar);
                                }
                                walls_read = Some(walls);
                            }
                            // The fillet rows themselves do not fit: the lane's
                            // own fit after the march refuses as it always has.
                            Err(_) => {}
                        }
                    }
                }
                if count == 0 {
                    // ACCEPTED. A plain fillet's wall is then refined toward
                    // the construction request, in exactly the intervals whose
                    // declared reading is over it (the closed chain's request,
                    // `chain::wall_model_missing`, on the same reading), within
                    // the same rounds and ceiling; a wall inside it everywhere
                    // inserts nothing and builds as accepted.
                    let request = wall.and_then(|check| check.model).zip(walls_read.as_ref());
                    let Some((model, walls)) = request else { break };
                    // The RAILS against the same request: the shipped support
                    // curves ARE these rails, so a rail standing off its
                    // carrier ships that miss. Read at j/16 of every interval
                    // (the acceptance read above reads three points); an
                    // unreadable read is short. Acceptance is unchanged.
                    let rails = closed_rail_interval_misses_at(&rows, &stations, &parameters,
                        [&first.face.surface, &second.face.surface], &CLOSED_REQUEST_RAIL_FRACTIONS)
                        .unwrap_or_else(|_| vec![f64::NAN; walls.len()]);
                    let rails = if rails.len() == walls.len() { rails } else { vec![f64::NAN; walls.len()] };
                    let missing: Vec<bool> = walls
                        .iter()
                        .zip(&rails)
                        .map(|(wall, &rail)| !matches!(wall, Some(declared) if *declared <= model) || !(rail <= model))
                        .collect();
                    let short = missing.iter().filter(|&&missing| missing).count();
                    if short == 0 {
                        break;
                    }
                    let rail_worst = rails.iter().copied().filter(|rail| rail.is_finite()).fold(0.0_f64, f64::max);
                    let declared = walls.iter().flatten().copied().fold(0.0_f64, f64::max).max(rail_worst);
                    CLOSED_REQUEST_RAILS.with(|slot| slot.set(Some((
                        rail_worst,
                        rails.iter().filter(|rail| !rail.is_finite()).count(),
                        walls.iter().flatten().copied().fold(0.0_f64, f64::max),
                    ))));
                    if round == REFINE_ROUNDS {
                        unmet = Some((declared, crate::BudgetReason::RoundsSpent, round, nodes.len()));
                        break;
                    }
                    if nodes.len() + short > station_ceiling {
                        unmet = Some((declared, crate::BudgetReason::StationCeiling, round, nodes.len()));
                        break;
                    }
                    // Each short interval by its measured declared reading
                    // against the request.
                    let counts: Vec<usize> = walls
                        .iter()
                        .zip(&rails)
                        .map(|(&reading, &rail)| stations_for_miss(reading, model).max(stations_for_miss(Some(rail), model)))
                        .collect();
                    let allocated: usize = counts.iter().sum();
                    // What is actually inserted: the allocation, or one station
                    // per short interval when it would pass the ceiling.
                    let short = if nodes.len() + allocated > station_ceiling { short } else { allocated };
                    match refine_closed_node_counts(&nodes, &counts, station_ceiling, |previous, t| {
                        frame.continue_node(previous.t, previous.uv, previous.station.center, t, None, true)
                    }) {
                        Ok(refined) => {
                            accepted = Some((nodes.clone(), declared));
                            nodes = refined;
                            continue;
                        }
                        Err(_) => {
                            // The attempted round counts; the rung that ships
                            // is this accepted one.
                            unmet = Some((declared, crate::BudgetReason::Incoherent, round + 1, nodes.len()));
                            break;
                        }
                    }
                }
                // A rung the REQUEST refined that no longer passes acceptance:
                // the accepted rung stands, and says so.
                if let Some((kept, declared)) = accepted.take() {
                    unmet = Some((declared, crate::BudgetReason::Rejected, round, kept.len()));
                    nodes = kept;
                    break;
                }
                if round == REFINE_ROUNDS || nodes.len() + count > station_ceiling {
                    let what = if wall_failing.is_empty() {
                        format!("rails stand {worst:.9e} off their carriers (bar {bar:.9e})")
                    } else {
                        format!("wall stands {wall_worst:.9e} off the rolling ball between its rails ({count} interval(s) over the bar or unreadable)")
                    };
                    return Err(KernelRefusal::non_convergence(KernelStage::Refine,
                        super::miter::MARCHED_FIT_OFF_CARRIERS_WHAT,
                        format!("blend: closed edge {} {what} after {} stations and {round} local rounds; bounded refinement exhausted", edge.id, nodes.len())));
                }
                // The intervals to refine, each by its measured miss: rails
                // over their bar, or (rails passing) walls failing theirs.
                let counts: Vec<usize> = if wall_failing.is_empty() {
                    misses.iter().map(|&miss| stations_for_miss(Some(miss), bar)).collect()
                } else {
                    let walls = walls_read.as_ref().expect("a wall verdict read");
                    let target = wall.map_or(f64::NAN, |check| check.bar);
                    walls.iter().map(|&reading| stations_for_miss(reading, target)).collect()
                };
                nodes = refine_closed_node_counts(&nodes, &counts, station_ceiling, |previous, t| {
                    frame.continue_node(previous.t, previous.uv, previous.station.center, t, None, true)
                })?;
            }
            check_fold(&nodes)?;
            // `round` counts the local rounds ATTEMPTED; on `Rejected` the
            // stations (and the declared reading) are the KEPT accepted rung's
            // — the geometry that ships — not the attempted one's.
            CLOSED_MODEL_UNMET.with(|slot| slot.set(unmet.map(|(declared, reason, round, stations)| (declared, reason, round, stations, station_ceiling))));
            Ok(())
            })().map_err(ClosedMarchFailure::Refinement)?;
        }
    }
    let stations: Vec<Station> = nodes.into_iter().map(|node| node.station).collect();
    let first_station = &stations[0];
    let last_station = stations.last().expect("march produced stations");
    if last_station.p1.sub(first_station.p1).length() > 1e-6 * scale
        || last_station.p2.sub(first_station.p2).length() > 1e-6 * scale
    {
        let error = KernelRefusal::non_convergence(KernelStage::Refine, "closed_march_closure", "blend: closed-edge march did not return to its start");
        return Err(if refinement_started { ClosedMarchFailure::Refinement(error) } else { ClosedMarchFailure::Initial(error) });
    }
    Ok(stations)
}


/// Chord-length parameters over the mean of the two support polylines,
/// normalised to [0, 1] (§4.9: station spacing proportional to the mean
/// support arc length).
pub(super) fn station_parameters(stations: &[Station]) -> Vec<f64> {
    let mut parameters = Vec::with_capacity(stations.len());
    let mut accumulated = 0.0;
    parameters.push(0.0);
    for pair in stations.windows(2) {
        let step1 = pair[1].p1.sub(pair[0].p1).length();
        let step2 = pair[1].p2.sub(pair[0].p2).length();
        accumulated += 0.5 * (step1 + step2);
        parameters.push(accumulated);
    }
    let total = accumulated.max(1e-12);
    for parameter in &mut parameters {
        *parameter /= total;
    }
    parameters
}

pub(super) struct FittedRows {
    pub(super) surface: NurbsSurface,
    pub(super) cr: NurbsCurve,
    pub(super) cs: NurbsCurve,
    /// Support pcurves in station order (forward traversal).
    pub(super) cr_pcurve: NurbsCurve,
    pub(super) cs_pcurve: NurbsCurve,
    pub(super) u_domain: [f64; 2],
    /// The rolling ball's centre path, fitted over the SAME parameters as the
    /// rows, so `center(u)` is the centre of the section at `surface(u, ·)`.
    /// A constant-radius blend is the radius-r canal of this curve, which is
    /// what lets a sibling stripe's surface be tested as a DISTANCE to it (the
    /// two-edge miter seam, `miter.rs`).  Only the open march fits it.
    pub(super) center: Option<NurbsCurve>,
    /// True when the surface is the EXACT extrusion of one section along a
    /// straight spine (a straight edge between two planes, or between a plane
    /// or cylinder and a cylinder whose axis runs along it: the blend is a
    /// cylinder patch), laid out with the section along v.  The surgery
    /// transposes such a face once it is built so the analytic recogniser —
    /// which wants the circle along u — sees the cylinder it is.
    pub(super) exact_extrusion: bool,
    /// Row parameters of the sections AT the edge's two vertices (the march
    /// snaps one station onto each), start then end.  A stripe that stops at
    /// its own vertex — a flush join with a tangent neighbour, or a capped
    /// end — stops here.  Only the open march sets them.
    pub(super) vertex_stations: Option<[f64; 2]>,
}

/// Preserve the exact rotational geometry of a coaxial analytic blend.
/// The general closed-row interpolator is intentionally approximate in the
/// marching direction; when every station is a rigid rotation of one exact
/// section, rebuilding that section as a revolution preserves exact circular
/// support rims.  Spheres are symmetric about every axis through their centre;
/// other revolution carriers must expose the same axis line.
pub(super) fn exact_closed_revolution_rows(
    stations: &[Station],
    first: &BlendMate,
    second: &BlendMate,
    chamfer: bool,
) -> Option<Result<FittedRows, KernelRefusal>> {
    let analytic1 = first.face.surface.analytic()?;
    let analytic2 = second.face.surface.analytic()?;
    let (axis_origin, base_axis) = match (analytic1, analytic2) {
        (
            crate::AnalyticSurface::Sphere { frame: frame1, .. },
            crate::AnalyticSurface::Sphere { frame: frame2, .. },
        ) => {
            // A sphere's stored frame is only its construction history.  Two
            // distinct sphere centres define the actual blend symmetry axis.
            let axis = frame2.origin.sub(frame1.origin).normalized().ok()?;
            (frame1.origin, axis)
        }
        _ => {
            let carrier = if !matches!(analytic1, crate::AnalyticSurface::Sphere { .. }) {
                analytic1.frame()?
            } else {
                analytic2.frame()?
            };
            let axis = carrier.axis.normalized().ok()?;
            let origin = carrier.origin;
            let scale = origin.length().max(1.0);
            let on_same_axis = |surface: &crate::AnalyticSurface| {
                let (point, aligned) = match surface {
                    crate::AnalyticSurface::Sphere { frame, .. } => (frame.origin, true),
                    other => {
                        let Some(frame) = other.frame() else {
                            return false;
                        };
                        (frame.origin, frame.axis.cross(axis).length() <= 1e-7)
                    }
                };
                let offset = point.sub(origin);
                let distance = offset.sub(axis.scale(offset.dot(axis))).length();
                aligned && distance <= 1e-7 * scale.max(point.length())
            };
            if !on_same_axis(analytic1) || !on_same_axis(analytic2) {
                return None;
            }
            (origin, axis)
        }
    };
    Some((|| {
        let first_station = stations.first().ok_or(KernelRefusal::internal(KernelStage::Refine, "sphere_stations", "blend: no sphere stations"))?;
        let next_station = stations
            .get(1)
            .ok_or(KernelRefusal::internal(KernelStage::Refine, "revolution_stations", "blend: exact revolution needs two stations"))?;
        let radial = |point: Vec3| {
            let offset = point.sub(axis_origin);
            offset.sub(base_axis.scale(offset.dot(base_axis)))
        };
        let sweep_sign = radial(first_station.p1)
            .cross(radial(next_station.p1))
            .dot(base_axis)
            .signum();
        let axis = if sweep_sign < 0.0 {
            base_axis.scale(-1.0)
        } else {
            base_axis
        };
        let generatrix = if chamfer {
            crate::make_line(first_station.p1, first_station.p2).or_refuse(KernelStage::Refine, "make_line")?
        } else {
            NurbsCurve::new(
                2,
                vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
                vec![
                    Vec4::from_point(first_station.p1, 1.0),
                    Vec4::from_point(first_station.apex, first_station.weight),
                    Vec4::from_point(first_station.p2, 1.0),
                ],
            ).or_refuse(KernelStage::Refine, "new")?
        };
        let surface =
            crate::make_revolution(axis_origin, axis, &generatrix, std::f64::consts::TAU).or_refuse(KernelStage::Refine, "make_revolution")?;
        let cr = surface.iso_curve_v(0.0).or_refuse(KernelStage::Refine, "iso_curve_v")?;
        let cs = surface.iso_curve_v(1.0).or_refuse(KernelStage::Refine, "iso_curve_v")?;
        // The exact 3D rims are circles about the centre line, but on a
        // sphere whose stored polar frame uses another axis their UV tracks
        // are curved.  Fit those marched tracks against the exact rational
        // revolution parameter (not the generic chord-length parameter), so
        // each pcurve parameter evaluates to the same point as the exact rim.
        let mut angle = 0.0;
        let mut revolution_parameters = Vec::with_capacity(stations.len());
        revolution_parameters.push(0.0);
        for pair in stations.windows(2) {
            let from = radial(pair[0].p1).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let to = radial(pair[1].p1).normalized().or_refuse(KernelStage::Refine, "normalized")?;
            let mut step = from.cross(to).dot(axis).atan2(from.dot(to));
            if step < 0.0 {
                step += std::f64::consts::TAU;
            }
            angle += step;
            revolution_parameters.push(crate::analytic_surface::circle_angle_to_parameter(
                4,
                std::f64::consts::TAU,
                angle.min(std::f64::consts::TAU),
            ));
        }
        *revolution_parameters.last_mut().unwrap() = 1.0;
        let fitted = fit_closed_rows(stations, &revolution_parameters, chamfer)?;
        let sphere_pcurve = |mate: &BlendMate, curve: &NurbsCurve, fallback: NurbsCurve,
                             start_uv: [f64; 2]| -> Result<NurbsCurve, KernelRefusal> {
            if !matches!(mate.face.surface.analytic(), Some(crate::AnalyticSurface::Sphere { .. })) {
                return Ok(fallback);
            }
            // Refining the march against 3D chords does not bound the error
            // of a sphere's curved UV track. Fit against the exact circle's
            // image, checking between interpolation stations on the carrier.
            let mut pcurve = crate::build_pcurve_on_surface_stations(
                &mate.face.surface, &|t| curve.evaluate(t), &curve.knots,
                crate::PCURVE_REFINEMENT_TOLERANCE, 64, 12, 4097,
            ).or_refuse(KernelStage::Refine, "sphere_contact_pcurve")?;
            let start = pcurve.evaluate(0.0).or_refuse(KernelStage::Refine, "sphere_contact_start")?;
            let closed = mate.face.surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
            for (axis, periodic, value, target) in [
                (0, closed.0, start.x, start_uv[0]),
                (1, closed.1, start.y, start_uv[1]),
            ] {
                if !periodic { continue; }
                let [low, high] = if axis == 0 { mate.face.surface.domain_u() } else { mate.face.surface.domain_v() }
                    .or_refuse(KernelStage::Refine, "sphere_contact_domain")?;
                let shift = (high - low) * ((target - value) / (high - low)).round();
                for control in &mut pcurve.control_points {
                    if axis == 0 { control.x += shift * control.w; } else { control.y += shift * control.w; }
                }
            }
            Ok(pcurve)
        };
        let cr_pcurve = sphere_pcurve(first, &cr, fitted.cr_pcurve, first_station.uv1)?;
        let cs_pcurve = sphere_pcurve(second, &cs, fitted.cs_pcurve, first_station.uv2)?;
        Ok(FittedRows {
            surface,
            cr,
            cs,
            cr_pcurve,
            cs_pcurve,
            u_domain: [0.0, 1.0],
            center: None,
            exact_extrusion: false,
            vertex_stations: None,
        })
    })())
}

/// Wrapped-overlap closed interpolation: extend the station list by
/// FIT_DEGREE wrapped stations on each side, interpolate every row with
/// the SAME parameters (shared knots), split out the middle span, then
/// snap the end control columns equal.  The seam endpoints are
/// interpolation points, so all rows close exactly; the wrap keeps the
/// seam tangent mismatch at interpolation-error scale.
pub(super) fn fit_closed_rows(
    stations: &[Station],
    parameters: &[f64],
    chamfer: bool,
) -> Result<FittedRows, KernelRefusal> {
    let count = stations.len();
    // Unwrapped pcurve periods: how far each uv track travelled in one
    // loop (zero on open carriers, ±domain-span across a seam).
    let period1 = stations[count - 1].uv1[0] - stations[0].uv1[0];
    let period1_v = stations[count - 1].uv1[1] - stations[0].uv1[1];
    let period2 = stations[count - 1].uv2[0] - stations[0].uv2[0];
    let period2_v = stations[count - 1].uv2[1] - stations[0].uv2[1];
    let wrap = |index: isize| -> usize {
        let n = (count - 1) as isize;
        (((index % n) + n) % n) as usize
    };
    let mut extended_params = Vec::new();
    let mut samples_cr = Vec::new();
    let mut samples_mid = Vec::new();
    let mut samples_cs = Vec::new();
    let mut samples_uv1 = Vec::new();
    let mut samples_uv2 = Vec::new();
    let mut push = |station: &Station, parameter: f64, laps: f64| {
        extended_params.push(parameter);
        samples_cr.push(Vec4::from_point(station.p1, 1.0));
        samples_cs.push(Vec4::from_point(station.p2, 1.0));
        samples_mid.push(Vec4 {
            x: station.apex.x * station.weight,
            y: station.apex.y * station.weight,
            z: station.apex.z * station.weight,
            w: station.weight,
        });
        samples_uv1.push(Vec4::from_point(
            Vec3::new(
                station.uv1[0] + laps * period1,
                station.uv1[1] + laps * period1_v,
                0.0,
            ),
            1.0,
        ));
        samples_uv2.push(Vec4::from_point(
            Vec3::new(
                station.uv2[0] + laps * period2,
                station.uv2[1] + laps * period2_v,
                0.0,
            ),
            1.0,
        ));
    };
    for offset in (1..=FIT_DEGREE).rev() {
        let index = wrap(-(offset as isize));
        push(&stations[index], parameters[index] - 1.0, -1.0);
    }
    for index in 0..count {
        // The final station already carries the +period uv from the march.
        push(&stations[index], parameters[index], 0.0);
    }
    for offset in 1..=FIT_DEGREE {
        let index = wrap(offset as isize);
        push(&stations[index], parameters[index] + 1.0, 1.0);
    }
    let low = extended_params[0];
    let high = *extended_params.last().unwrap();
    let range = high - low;
    let normalized: Vec<f64> = extended_params
        .iter()
        .map(|parameter| (parameter - low) / range)
        .collect();
    let seam_low = (0.0 - low) / range;
    let seam_high = (1.0 - low) / range;
    let fit_row = |samples: &[Vec4]| -> Result<NurbsCurve, KernelRefusal> {
        let curve = fit::interpolate_homogeneous(samples, FIT_DEGREE, &normalized).or_refuse(KernelStage::Refine, "interpolate_homogeneous")?;
        let (_, tail) = curve.split(seam_low).or_refuse(KernelStage::Refine, "split")?;
        let (middle, _) = tail.split(seam_high).or_refuse(KernelStage::Refine, "split")?;
        Ok(middle)
    };
    let cr = fit_row(&samples_cr)?;
    let cs = fit_row(&samples_cs)?;
    let cr_pcurve = fit_row(&samples_uv1)?;
    let cs_pcurve = fit_row(&samples_uv2)?;
    let mid = if chamfer {
        None
    } else {
        Some(fit_row(&samples_mid)?)
    };
    let u_domain = cr.domain().or_refuse(KernelStage::Refine, "domain")?;
    let surface = crate::blend::rows::surface_from_rows(FIT_DEGREE, &cr, &cs, mid.as_ref(), true)?;
    Ok(FittedRows {
        surface,
        cr,
        cs,
        cr_pcurve,
        cs_pcurve,
        u_domain,
        center: None,
        exact_extrusion: false,
        vertex_stations: None,
    })
}

fn refine_closed_node_intervals(
    nodes: &[MarchNode],
    misses: &[f64],
    bar: f64,
    solve: impl Fn(&MarchNode, &MarchNode) -> Result<MarchNode, KernelRefusal>,
) -> Result<Vec<MarchNode>, KernelRefusal> {
    let count = misses.iter().filter(|&&miss| miss > bar).count();
    let mut refined = Vec::with_capacity(nodes.len() + count);
    for (index, &miss) in misses.iter().enumerate() {
        let left = &nodes[index];
        let right = &nodes[index + 1];
        refined.push(left.clone());
        if miss > bar {
            refined.push(solve(left, right)?);
        }
    }
    refined.push(nodes.last().expect("closed endpoint").clone());
    Ok(refined)
}

/// How many stations a failing interval takes this round, from its MEASURED
/// miss against the target it must meet: an interpolant's error falls about
/// as the fourth power of its spacing, so an interval missing by `miss`
/// is split into ⌈(miss / target)^¼⌉ equal parts (one station — bisection —
/// for a miss within 16 × target, an unreadable or non-finite reading, or a
/// target that is not positive). A PRIORITISATION of the existing local
/// rounds, not an accuracy claim: every refined rung is measured again in
/// full before it is accepted.
fn stations_for_miss(miss: Option<f64>, target: f64) -> usize {
    match miss {
        Some(miss) if miss.is_finite() && target > 0.0 && miss > target => {
            let parts = (miss / target).powf(0.25).ceil();
            if parts.is_finite() && parts >= 2.0 && parts <= 1e6 { parts as usize - 1 } else { 1 }
        }
        Some(miss) if miss.is_finite() && miss <= target => 0,
        _ => 1,
    }
}

/// [`refine_closed_node_intervals`] with `counts[i]` stations in interval i,
/// at equal fractions of its edge parameter, each continued from the station
/// before it. An interval taking one station gets exactly the midpoint solve
/// from its left station that bisection gives. When the counts would pass
/// `ceiling` stations in all (summed with overflow checks), every refining
/// interval takes one station instead (the plain bisection the rounds always
/// made), and when even that would pass `ceiling` nothing is solved and the
/// call refuses. Only the stations actually inserted are reserved. `counts`
/// must hold one entry per interval, and an interval taking more than one
/// station must run over finite, strictly increasing parameters.
fn refine_closed_node_counts(
    nodes: &[MarchNode],
    counts: &[usize],
    ceiling: usize,
    solve: impl Fn(&MarchNode, f64) -> Result<MarchNode, KernelRefusal>,
) -> Result<Vec<MarchNode>, KernelRefusal> {
    if nodes.len() < 2 || counts.len() != nodes.len() - 1 {
        return Err(KernelRefusal::internal(KernelStage::Refine, "closed_refine_counts", format!(
            "blend: {} station counts for {} intervals", counts.len(), nodes.len().saturating_sub(1))));
    }
    let requested = counts.iter().try_fold(nodes.len(), |sum, &count| sum.checked_add(count));
    let counts: Vec<usize> = match requested {
        Some(total) if total <= ceiling => counts.to_vec(),
        _ => counts.iter().map(|&count| count.min(1)).collect(),
    };
    let total = nodes.len() + counts.iter().sum::<usize>();
    if total > ceiling {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "closed_refine_ceiling", format!(
            "blend: refining {} interval(s) would pass the {ceiling}-station ceiling at {} stations",
            counts.iter().filter(|&&count| count > 0).count(), nodes.len())));
    }
    let mut refined = Vec::with_capacity(total);
    for (index, &count) in counts.iter().enumerate() {
        let left = &nodes[index];
        let right = &nodes[index + 1];
        refined.push(left.clone());
        if count > 1 && !(left.t.is_finite() && right.t.is_finite() && left.t < right.t) {
            return Err(KernelRefusal::internal(KernelStage::Refine, "closed_refine_interval", format!(
                "blend: interval {index} runs over [{}, {}], not a finite increasing span", left.t, right.t)));
        }
        let mut previous = left.clone();
        for step in 1..=count {
            // One station: bisection's own midpoint, to the bit.
            let t = if count == 1 { 0.5 * (left.t + right.t) } else { left.t + (right.t - left.t) * step as f64 / (count + 1) as f64 };
            if count > 1 && !(t > previous.t && t < right.t) {
                return Err(KernelRefusal::internal(KernelStage::Refine, "closed_refine_interval", format!(
                    "blend: station {step} of {count} in interval {index} at t {t} is not strictly inside ({}, {})", previous.t, right.t)));
            }
            let node = solve(&previous, t)?;
            refined.push(node.clone());
            previous = node;
        }
    }
    refined.push(nodes.last().expect("closed endpoint").clone());
    Ok(refined)
}

/// Read-only comparison of global, station-seeded and fitted-pcurve feet.
/// The acceptance reading and refinement decisions remain unchanged.
fn trace_closed_rail_feet(
    rows: &FittedRows,
    stations: &[Station],
    parameters: &[f64],
    carriers: [&NurbsSurface; 2],
) {
    let [low, high] = rows.u_domain;
    for (interval, pair) in parameters.windows(2).enumerate() {
        for fraction in [0.25, 0.5, 0.75] {
            let u = low + (high - low) * (pair[0] + (pair[1] - pair[0]) * fraction);
            for (side, rail, pcurve, carrier, left, right) in [
                (0, &rows.cr, &rows.cr_pcurve, carriers[0], stations[interval].uv1, stations[interval + 1].uv1),
                (1, &rows.cs, &rows.cs_pcurve, carriers[1], stations[interval].uv2, stations[interval + 1].uv2),
            ] {
                let read = (|| -> Result<serde_json::Value, String> {
                    let point = rail.evaluate(u)?;
                    let seed = [left[0] + fraction * (right[0] - left[0]), left[1] + fraction * (right[1] - left[1])];
                    let global = crate::project_point_to_surface(carrier, point);
                    let seeded = crate::project_point_to_surface_seeded(carrier, point, seed[0], seed[1]);
                    let uv = pcurve.evaluate(u)?;
                    let represented = carrier.evaluate_extended(uv.x, uv.y)?;
                    let foot = |value: &Result<crate::SurfaceProjection, String>| match value {
                        Ok(p) => serde_json::json!({"uv":[p.u,p.v],"distance":p.distance,
                            "extended_distance":carrier.evaluate_extended(p.u,p.v).ok().map(|q|q.sub(point).length())}),
                        Err(error) => serde_json::json!({"error":error}),
                    };
                    Ok(serde_json::json!({"interval":interval,"fraction":fraction,"side":side,"u":u,
                        "point":[point.x,point.y,point.z],"seed":seed,"left_uv":left,"right_uv":right,
                        "global":foot(&global),"seeded":foot(&seeded),"pcurve_uv":[uv.x,uv.y],
                        "pcurve_distance":represented.sub(point).length(),"closed":carrier.closed_directions()?}))
                })();
                match read {
                    Ok(row) => eprintln!("C3-RAIL-FOOT {row}"),
                    Err(error) => eprintln!("C3-RAIL-FOOT-ERROR interval={interval} side={side} {error}"),
                }
            }
        }
    }
}

/// How far each fitted rail stands off its carrier between stations: an
/// ACCURACY reading, so it is taken against the carrier's own extension
/// (`NurbsSurface::derivatives_extended`), the surface the stations were
/// solved on. A rail whose contact runs past the finite patch — the
/// edge-preserving wall whose ball rests on the bottom rim, 0.5 below a
/// washer's wall — is exactly on that extension, and the finite-patch
/// projector, which clamps its foot to the patch, would read the overshoot
/// as a miss no refinement can shrink. Whether such a contact is FEASIBLE is
/// the edge-preserving / support lane's question, unchanged here. Each foot is
/// seeded on the march's own branch: the interval's two station (u, v),
/// continued and so unwrapped, interpolated at the probe's fraction.
pub(super) fn closed_rail_interval_misses(
    rows: &FittedRows,
    stations: &[Station],
    parameters: &[f64],
    carriers: [&NurbsSurface; 2],
) -> Result<Vec<f64>, KernelRefusal> {
    closed_rail_interval_misses_at(rows, stations, parameters, carriers, &[0.25, 0.5, 0.75])
}

/// The fractions of every interval the closed-edge construction REQUEST reads
/// the rails at, j/16: the shipped support curves are these rails, and a miss
/// between the acceptance read's quarter points ships too.
const CLOSED_REQUEST_RAIL_FRACTIONS: [f64; 15] = [
    1.0 / 16.0, 2.0 / 16.0, 3.0 / 16.0, 4.0 / 16.0, 5.0 / 16.0, 6.0 / 16.0, 7.0 / 16.0, 8.0 / 16.0,
    9.0 / 16.0, 10.0 / 16.0, 11.0 / 16.0, 12.0 / 16.0, 13.0 / 16.0, 14.0 / 16.0, 15.0 / 16.0,
];

/// [`closed_rail_interval_misses`] at the given fractions of every interval.
pub(super) fn closed_rail_interval_misses_at(
    rows: &FittedRows,
    stations: &[Station],
    parameters: &[f64],
    carriers: [&NurbsSurface; 2],
    fractions: &[f64],
) -> Result<Vec<f64>, KernelRefusal> {
    let [low, high] = rows.u_domain;
    parameters.windows(2).enumerate().map(|(interval, pair)| {
        let mut worst = 0.0_f64;
        for &fraction in fractions {
            let normalized = pair[0] + (pair[1] - pair[0]) * fraction;
            let u = low + (high - low) * normalized;
            let (left, right) = (&stations[interval], &stations[interval + 1]);
            let lerp = |a: [f64; 2], b: [f64; 2]| [a[0] + fraction * (b[0] - a[0]), a[1] + fraction * (b[1] - a[1])];
            for (rail, seed, carrier) in [
                (&rows.cr, lerp(left.uv1, right.uv1), carriers[0]),
                (&rows.cs, lerp(left.uv2, right.uv2), carriers[1]),
            ] {
                let point = rail.evaluate(u).or_refuse(KernelStage::Refine, "evaluate")?;
                let gap = extended_foot_distance(carrier, point, seed)?;
                worst = worst.max(gap);
            }
        }
        Ok(worst)
    }).collect()
}

/// Distance from `point` to `carrier`'s extension near `seed`: damped
/// Gauss-Newton on the extended evaluation, taking only steps that bring the
/// carrier closer, so the reading never exceeds the seed's own distance and
/// is always the distance to an actual point of the extended carrier.
pub(in crate::blend) fn extended_foot_distance(carrier: &NurbsSurface, point: Vec3, seed: [f64; 2]) -> Result<f64, KernelRefusal> {
    let distance_at = |uv: [f64; 2]| -> Result<f64, KernelRefusal> {
        Ok(carrier.evaluate_extended(uv[0], uv[1]).or_refuse(KernelStage::Refine, "evaluate_extended")?.sub(point).length())
    };
    if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite() && seed[0].is_finite() && seed[1].is_finite()) {
        return Ok(f64::INFINITY);
    }
    let mut uv = seed;
    let mut best = distance_at(uv)?;
    if !best.is_finite() {
        return Ok(f64::INFINITY);
    }
    for _ in 0..30 {
        let d = carrier.derivatives_extended(uv[0], uv[1], 1).or_refuse(KernelStage::Refine, "derivatives_extended")?;
        let (offset, su, sv) = (d[0][0].sub(point), d[1][0], d[0][1]);
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (gu, gv) = (su.dot(offset), sv.dot(offset));
        let det = a * c - b * b;
        // UNREADABLE (any non-finite derivative, normal matrix, gradient or
        // step) is never accurate: it reads infinitely far, deterministically.
        if ![a, b, c, gu, gv, det].iter().all(|value| value.is_finite()) {
            return Ok(f64::INFINITY);
        }
        // Finite but singular: no direction to improve in, so the foot stays
        // where it is and its finite distance is the reading.
        if !(det.abs() > 1e-300) {
            break;
        }
        let mut step = [(c * gu - b * gv) / det, (a * gv - b * gu) / det];
        if !(step[0].is_finite() && step[1].is_finite()) {
            return Ok(f64::INFINITY);
        }
        let mut improved = false;
        for _ in 0..30 {
            let trial = [uv[0] - step[0], uv[1] - step[1]];
            let distance = distance_at(trial)?;
            if !distance.is_finite() {
                return Ok(f64::INFINITY);
            }
            if distance < best {
                uv = trial;
                best = distance;
                improved = true;
                break;
            }
            step = [0.5 * step[0], 0.5 * step[1]];
        }
        // Zero progress (no decreasing step, or a step below roundoff): the
        // finite distance reached is the reading.
        if !improved || step[0].abs().max(step[1].abs()) < 1e-15 {
            break;
        }
    }
    // `best` only ever takes finite values above; kept explicit because
    // `f64::max` would drop a NaN and read it as accurate.
    Ok(if best.is_finite() { best } else { f64::INFINITY })
}

pub(super) fn locate_mate<'a>(
    solid: &'a BrepSolid,
    edge_id: u64,
    skip: Option<(u64, usize)>,
) -> Result<(&'a FaceRecord, usize, &'a CoedgeRecord), KernelRefusal> {
    for shell in &solid.shells {
        for face in &shell.faces {
            for (loop_index, loop_record) in face.loops.iter().enumerate() {
                if skip == Some((face.id, loop_index)) {
                    continue;
                }
                if let Some(coedge) = loop_record
                    .coedges
                    .iter()
                    .find(|coedge| coedge.edge_id == edge_id)
                {
                    return Ok((face, loop_index, coedge));
                }
            }
        }
    }
    Err(KernelRefusal::internal(KernelStage::Classify, "mating_face", format!("blend: edge {edge_id} has no (second) mating face")))
}

/// Signed radii ρ1/ρ2 (4.9.2): the ball center sits along the inward
/// bisector of the mid-edge cross-section; ρ is its projection sign onto
/// each face's RAW normal times the radius.
pub(super) fn signed_radii(
    edge: &EdgeRecord,
    first_face: &FaceRecord,
    first_coedge: &CoedgeRecord,
    second_face: &FaceRecord,
    second_coedge: &CoedgeRecord,
    radius: f64,
) -> Result<(f64, f64), KernelRefusal> {
    let probe_step = (radius * 0.25).max(1e-4);
    // A closed circle's parameter midpoint can land exactly on a periodic
    // carrier seam even though it is far from the edge's topological seam.
    // UV classification calls that probe `Boundary`, so sample symmetric
    // interior fractions until both support-side signs are unambiguous.
    let mut seed = None;
    for fraction in [0.5, 0.375, 0.625, 0.25, 0.75] {
        let t = edge.t0 + (edge.t1 - edge.t0) * fraction;
        let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let tangent = edge
            .curve
            .unit_tangent(t, edge.t0, edge.t1)
            .map_err(|error| format!("blend: edge {}: {error}", edge.id)).or_refuse(KernelStage::Refine, "unit_tangent")?;
        let Ok(into1) = crate::fillet::into_face_direction(
            first_face, point, tangent, point, tangent, probe_step,
        ) else {
            continue;
        };
        let Ok(into2) = crate::fillet::into_face_direction(
            second_face,
            point,
            tangent,
            point,
            tangent,
            probe_step,
        ) else {
            continue;
        };
        let uv1 = edge_uv_on_face(first_coedge, edge, t)?;
        let uv2 = edge_uv_on_face(second_coedge, edge, t)?;
        seed = Some((point, into1, into2, uv1, uv2));
        break;
    }
    let (mid_point, into1, into2, uv1_mid, uv2_mid) =
        seed.ok_or(KernelRefusal::internal(KernelStage::Classify, "support_orientation", "blend: could not orient support directions away from carrier seams"))?;
    let n1_mid = raw_normal(&first_face.surface, uv1_mid[0], uv1_mid[1])?;
    let n2_mid = raw_normal(&second_face.surface, uv2_mid[0], uv2_mid[1])?;
    let cos_theta = into1.dot(into2).clamp(-1.0, 1.0);
    let theta = cos_theta.acos();
    if theta <= 1e-6 || theta >= std::f64::consts::PI - 1e-6 {
        return Err(KernelRefusal::unsupported(KernelStage::Classify, "degenerate_dihedral", "blend: dihedral angle too degenerate"));
    }
    let bisector = into1.add(into2).normalized().or_refuse(KernelStage::Refine, "normalized")?;
    let center_seed = mid_point.add(bisector.scale(radius / (theta * 0.5).sin()));
    let tangent_offset = radius / (theta * 0.5).tan();
    let p1_seed = mid_point.add(into1.scale(tangent_offset));
    let p2_seed = mid_point.add(into2.scale(tangent_offset));
    let rho1 = if center_seed.sub(p1_seed).dot(n1_mid) >= 0.0 {
        radius
    } else {
        -radius
    };
    let rho2 = if center_seed.sub(p2_seed).dot(n2_mid) >= 0.0 {
        radius
    } else {
        -radius
    };
    Ok((rho1, rho2))
}

