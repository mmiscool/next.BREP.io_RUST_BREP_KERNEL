use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass, MAX_FIT_STATIONS};

/// The degree a marched section is interpolated at (2026-09-27). The march
/// puts every station on both carriers; the fit decides how far the curve
/// strays BETWEEN them, and a cubic's O(h^4) there left `BadBoolean`'s
/// sphere×bore rim 2.5e-7 off the bore with a mean bias of −1.2e-8 that both
/// trims inherited (+1.14e-6 on the solid). A quintic through the same
/// stations reads 8.5e-9 and +6e-11 with the same knots and control-point
/// count. More stations bought the same accuracy on 2026-09-26 and were
/// reverted for their cost: the watertight tessellator samples every interior
/// knot, so every scan downstream scales with station count, and a degree does
/// not add knots. `BREP_SECTION_FIT_DEGREE=3` restores the cubic. The gate on
/// the mid-span refinement, as a multiple of the fit tolerance (2026-09-27): a
/// marched section whose unrefined fit misses a carrier between stations by
/// more than this is UNDERSAMPLED — the 20° cylinder crossing of
/// `fillet-skew-cylinder-seam-split-20deg` sits 4e-5..8e-5 off, 470 times the
/// tolerance, at any degree — and only such a section is given true
/// intersection points until its mid-spans hold. Refining every section cost
/// more downstream than it returned (the tessellator samples every knot).
/// `BREP_SECTION_REFINE_GATE` sets the multiple (`on` for
/// [`SECTION_REFINE_GATE`]). OFF BY DEFAULT: at k = 100 the sequential
/// blend/direct_edit/imprint/boolean lib set costs +64 % and
/// `oblique_multi_rim_cone_cap_tearing_push_refuses` goes red; k = 1000 costs
/// +32 % and leaves the skew union +5.3e-4 off. This global switch stays off.
/// Independently, a pair of grazing crossings on one trim edge requests a
/// fit-floor check for that face pair below: those two nearby junctions and
/// the intervening cap must survive the fit.
fn section_refine_gate() -> Option<f64> {
    static GATE: std::sync::OnceLock<Option<f64>> = std::sync::OnceLock::new();
    *GATE.get_or_init(|| match std::env::var("BREP_SECTION_REFINE_GATE") {
        Ok(value) if value == "on" => Some(SECTION_REFINE_GATE),
        Ok(value) => value.parse::<f64>().ok().filter(|gate| *gate >= 0.0),
        Err(_) => None,
    })
}

/// Why a section goes to [`refine_section_against_carriers`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum SectionRefineRoute {
    /// `BREP_SECTION_REFINE_GATE`: every section, at the variable's multiple.
    Global,
    /// A recovered fit reproduces its march points; check it between them.
    Recovered,
    /// A pair of grazing crossings on one trim edge.
    Grazing,
    /// The standing fit missed the tolerance it was asked for. The polyline
    /// deviation can overstate the carrier miss (a band-raised floor reads the
    /// input's noise), so the refiner's own carrier measure decides: a section
    /// whose mid-spans already hold the gate leaves at round 0 as `BelowGate`,
    /// unchanged. Measured 2026-10-04: the StationCeiling fits it reaches ship
    /// 4.7e-5 to 2.1e-4 off the true section; the refiner brings them to
    /// 1e-8..3e-8.
    UnmetRequest,
    /// A standing fit that MET its request on no other route, whose kept
    /// stations all lie on both carriers within the fit tolerance, but whose
    /// curve stands further off the carriers BETWEEN them than the local gate
    /// ([`carrier_miss_route`]). The polyline it met is the march's chords; an
    /// interpolant through consistent stations can still ring between them.
    /// Measured 2026-10-04 (08356963b, 94 cases): 23 of 285 fast-path sections,
    /// 13 distinct pairs, read their mid-spans up to 6.03e-6 off while every
    /// kept station read within 1e-7; fixture 28 t217 pair 118x474 read 1.59e-6
    /// between stations 1.8e-9 off.
    CarrierMiss,
}

/// The carrier-miss route for a fit that met its request on no other route:
/// refine at the local gate when its mid-spans miss the carriers by more than
/// the fit tolerance WHILE every kept station is within it. Stations off a
/// carrier are the input's (moving them is the reserved relocation ruling),
/// so such a section keeps the fast path and is not reported as refined. An
/// unreadable reading (NaN) never routes.
pub(super) fn carrier_miss_route(station_upper: f64, midspan_upper: f64, fit_tolerance: f64) -> Option<(f64, SectionRefineRoute)> {
    (station_upper.is_finite() && midspan_upper.is_finite() && station_upper <= fit_tolerance && midspan_upper > fit_tolerance)
        .then_some((1.0, SectionRefineRoute::CarrierMiss))
}

/// The carrier readings [`carrier_miss_route`] decides on: the worst of both
/// carriers over EVERY point of the section's run (the march's original
/// stations, kept or dropped by the fit, so a fit through consistent kept
/// stations cannot hide an inconsistent dropped one), and at the fit's
/// mid-spans. Projector distances, so upper readings; NaN as soon as any is
/// unreadable. The mid-span read is the route's TRIGGER, not a bound on the
/// curve's maximum; [`knot_interior_carrier_upper`] is the finer read.
pub(super) fn fast_path_carrier_readings(fit: &PolylineFit, run: &[Vec3], first: &NurbsSurface, second: &NurbsSurface) -> (f64, f64) {
    let read = |surface: &NurbsSurface, point: Vec3| project_point_to_surface(surface, point).map_or(f64::NAN, |projection| projection.distance);
    let mut station = 0.0f64;
    for point in run {
        let (a, b) = (read(first, *point), read(second, *point));
        station = if a.is_nan() || b.is_nan() || station.is_nan() { f64::NAN } else { station.max(a).max(b) };
    }
    (station, midspan_carrier_upper(fit, first, second))
}

/// The worst of BOTH carriers at the fit's mid-spans, each carrier read on its
/// own and checked finite BEFORE any max (`f64::max` would drop a NaN from one
/// carrier and pass the other's reading): NaN as soon as either is unreadable.
pub(super) fn midspan_carrier_upper(fit: &PolylineFit, first: &NurbsSurface, second: &NurbsSurface) -> f64 {
    let read = |surface: &NurbsSurface, point: Vec3| project_point_to_surface(surface, point).map_or(f64::NAN, |projection| projection.distance);
    let mut worst = 0.0f64;
    for span in 0..fit.parameters.len().saturating_sub(1) {
        let mid = 0.5 * (fit.parameters[span] + fit.parameters[span + 1]);
        let (a, b) = match fit.curve.evaluate(mid) {
            Ok(point) => (read(first, point), read(second, point)),
            Err(_) => (f64::NAN, f64::NAN),
        };
        if !(a.is_finite() && b.is_finite()) || worst.is_nan() {
            return f64::NAN;
        }
        worst = worst.max(a).max(b);
    }
    worst
}

/// The curve's worst distance from both carriers at the quarter points of
/// every one of its own knot spans (its native pieces, where an interpolant
/// is extremal between knots), NaN as soon as any read is unreadable. Sampled
/// and projector-based: an upper reading at its points, not a certified
/// maximum of the curve.
pub(super) fn knot_interior_carrier_upper(curve: &NurbsCurve, first: &NurbsSurface, second: &NurbsSurface) -> f64 {
    let read = |surface: &NurbsSurface, point: Vec3| project_point_to_surface(surface, point).map_or(f64::NAN, |projection| projection.distance);
    let Ok([t0, t1]) = curve.domain() else { return f64::NAN };
    let mut knots: Vec<f64> = curve.knots.iter().copied().filter(|knot| *knot >= t0 && *knot <= t1).collect();
    knots.dedup();
    let mut worst = 0.0f64;
    for pair in knots.windows(2) {
        if pair[1] <= pair[0] {
            continue;
        }
        for fraction in [0.25, 0.5, 0.75] {
            let value = match curve.evaluate(pair[0] + (pair[1] - pair[0]) * fraction) {
                Ok(point) => {
                    let (a, b) = (read(first, point), read(second, point));
                    if a.is_finite() && b.is_finite() { a.max(b) } else { f64::NAN }
                }
                Err(_) => f64::NAN,
            };
            worst = if worst.is_nan() || !value.is_finite() { f64::NAN } else { worst.max(value) };
        }
    }
    worst
}

/// The refine route and its gate (a multiple of the fit tolerance) for one
/// section, or `None` for the fast path.
///
/// A standing fit that MET its request takes exactly the routes that existed
/// before: the global gate alone when set, else a recovered fit, else a grazing
/// pair (read lazily, only when the first two do not apply), else nothing — it
/// goes straight to `process_curve`, untouched.
///
/// A fit that MISSED its request is always refined, at the local gate 1, or at
/// a tighter global gate. A looser global gate does not mask the miss: seam 60
/// was a recovered fit missing its 1e-7 request, and `BREP_SECTION_REFINE_GATE`
/// above 1 used to let it through. Its route stays `UnmetRequest` under any
/// global gate, so its cost reads apart from the global gate's.
pub(super) fn section_refine_route(
    global: Option<f64>,
    recovered: bool,
    grazing: impl FnOnce() -> bool,
    met_requested: bool,
) -> Option<(f64, SectionRefineRoute)> {
    if !met_requested {
        return Some((global.map_or(1.0, |gate| gate.min(1.0)), SectionRefineRoute::UnmetRequest));
    }
    global
        .map(|gate| (gate, SectionRefineRoute::Global))
        .or_else(|| recovered.then_some((1.0, SectionRefineRoute::Recovered)))
        .or_else(|| grazing().then_some((1.0, SectionRefineRoute::Grazing)))
}

/// The multiple `BREP_SECTION_REFINE_GATE=on` uses: the smallest measured
/// that trips the skew union's undersampled sections and leaves fixture 22 as
/// it was (k = 10 flips it).
const SECTION_REFINE_GATE: f64 = 100.0;

/// A clipped run's END span can be a sliver: the clip endpoint lands beside a
/// march station, a spacing ratio to the run's median of 0.011 on
/// `anotherBooleanFail`, and a global interpolant rings next to such a jump —
/// the mid-spans near the ends miss their carriers by 1e-3, and bisecting them
/// only makes the spacing more uneven (2026-09-27). The reduced witness
/// (`imprint/tests.rs` `sliver_end_evening`) says what makes the jump ring: a
/// sliver end ON both carriers fits to 3e-8, while the same end 2.7e-4 off
/// the torus — the vendor's edge curves sit that far off the tori they
/// bound, and the run's end is solved on such an edge — rings at 2e-3 with
/// the interior at 3e-13 (2026-09-30). Below
/// `SLIVER_END_FRACTION` of the median spacing the station beside the
/// endpoint is RE-SOLVED at the middle of the merged gap as a fresh
/// intersection point (the default, `BREP_SECTION_SLIVER_END=resolve`) or
/// dropped (`drop`); `off` restores the unevened fit for a before/after
/// reading. Returns how many ends were evened.
///
/// ON BY DEFAULT since 2026-09-30 (lane K6), behind the measurement in
/// [`even_sliver_ends_measured`]: only a section whose fit shows the ringing signature
/// ([`ringing_signature`]) is evened, and the evened fit stands only where it reads closer to
/// the carriers by [`SLIVER_END_GAIN`]. The ringing is what a marched section carries into the
/// result: on `anotherBooleanFail` the sphere sections leave the analytic vendor torus by
/// 3.1e-3 within 1% of their span (measured against the STEP file's own `TOROIDAL_SURFACE`,
/// not the projector), the cut torus faces depart from their edges by 2.6e-4 and the result
/// shell's vector-area closure reads 1.88e-4 against a 3.83e-5 bar. Evened, the sections sit
/// 5e-8 off, the shell closes at 1.7e-6 and the boolean takes 2.3 s instead of 9.1 s. The
/// 2026-09-27 shelving reason was three fixture-25 exact-value pins moving under a blanket
/// `resolve`; a blanket default also refused fixture 28's reversed subtract and broke fixture
/// 26's idempotence, which is why the signature gates it. Fixture 25's sections do not show
/// the signature at the shipped contrast and its pins stand.
fn even_sliver_ends(
    section: &mut Vec<Vec3>,
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<usize, String> {
    static MODE: std::sync::OnceLock<String> = std::sync::OnceLock::new();
    let mode = MODE.get_or_init(|| {
        std::env::var("BREP_SECTION_SLIVER_END").unwrap_or_else(|_| "resolve".to_string())
    });
    if !(mode == "drop" || mode == "resolve") {
        return Ok(0);
    }
    let mut evened = 0;
    // One sliver at a time: each removal changes the spans it sits between.
    for _ in 0..SLIVER_END_ROUNDS {
        let count = section.len();
        if count < 6 {
            break;
        }
        let mut chords: Vec<f64> = section.windows(2).map(|pair| pair[1].sub(pair[0]).length()).collect();
        let lengths = chords.clone();
        chords.sort_by(f64::total_cmp);
        let median = chords[chords.len() / 2];
        let spans = lengths.len();
        let near_ends = [0, 1, 2, spans - 3, spans - 2, spans - 1];
        let Some(&sliver) = near_ends
            .iter()
            .filter(|&&span| lengths[span] < sliver_end_fraction() * median)
            .min_by(|a, b| lengths[**a].total_cmp(&lengths[**b]))
        else {
            break;
        };
        // The sliver's bounding stations are `sliver` and `sliver + 1`; a run
        // endpoint is never moved. Of the interior ones, remove the one whose
        // removal leaves the shorter merged span.
        let mut candidates = Vec::new();
        for station in [sliver, sliver + 1] {
            if station == 0 || station == count - 1 {
                continue;
            }
            let merged = section[station + 1].sub(section[station - 1]).length();
            candidates.push((station, merged));
        }
        let Some(&(station, _)) = candidates.iter().min_by(|a, b| a.1.total_cmp(&b.1)) else {
            break;
        };
        if mode == "resolve" {
            let (before, after) = (section[station - 1], section[station + 1]);
            let seed = before.add(after).scale(0.5);
            let reach = 0.5 * after.sub(before).length();
            match crate::surface_surface_intersection::refine_to_intersection(first, second, seed, tolerance, reach)? {
                Some(point) => section[station] = point,
                None => {
                    section.remove(station);
                }
            }
        } else {
            section.remove(station);
        }
        evened += 1;
    }
    Ok(evened)
}

/// At 0.25 six of `anotherBooleanFail`'s eight sections stay NoProgress
/// (worst miss 2.7e-4); at 0.5 all eight are evened and fall below the
/// refinement gate (worst miss 2.8e-3 -> 5.4e-8).
const SLIVER_END_FRACTION: f64 = 0.5;

/// How many sliver spans one section may lose, at most, over both ends. Every
/// `anotherBooleanFail` section reaches this cap (the march lands several
/// closely spaced stations beside each clip end), and the evened fits read
/// 2.2e-8..5.4e-8 against the carriers under it; whether a higher cap would
/// read differently was not measured (lane K6, 2026-09-30).
const SLIVER_END_ROUNDS: usize = 4;

/// `BREP_SECTION_SLIVER_FRACTION` overrides [`SLIVER_END_FRACTION`] for the
/// measurement that picks it.
fn sliver_end_fraction() -> f64 {
    static FRACTION: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *FRACTION.get_or_init(|| {
        std::env::var("BREP_SECTION_SLIVER_FRACTION")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|fraction| *fraction > 0.0 && *fraction < 1.0)
            .unwrap_or(SLIVER_END_FRACTION)
    })
}

fn section_fit_degree() -> usize {
    static DEGREE: std::sync::OnceLock<usize> = std::sync::OnceLock::new();
    *DEGREE.get_or_init(|| {
        std::env::var("BREP_SECTION_FIT_DEGREE")
            .ok()
            .and_then(|value| value.parse::<usize>().ok())
            .filter(|degree| (1..=7).contains(degree))
            .unwrap_or(5)
    })
}

/// A marched section is held to the fit tolerance against the CARRIERS, not
/// only against the polyline it was fitted through (2026-09-26). The march
/// refines every station onto both surfaces to `tolerance * 0.01`, but the
/// cubic interpolant between stations is measured by `fit_polyline` against the
/// polyline's own chords, which are further from the true curve than the
/// interpolant is — so a systematic bias of the interpolant (2.8e-8 mean,
/// 4.7e-7 worst, inside the bore on `BadBoolean`'s sphere/bore rim at 135
/// stations) passed every check the fit could make, and both trims inherited it
/// (+1.14e-6 on the solid's volume, against two independent readings agreeing
/// to 1e-8). The check below evaluates the fitted curve at every mid-span,
/// measures it against both carriers, and where it misses `fit tolerance *
/// SECTION_MIDSPAN_FRACTION` inserts a TRUE intersection point there and
/// refits. The interpolant's bias falls as h^4, so one round usually suffices.
/// A round that neither halves the worst miss nor lowers the count of spans
/// missing is not in that regime — a carrier's C0 crease puts a corner in the
/// section (fixture 25's pierce solid), where every round only doubles the
/// stations — so that round is discarded and the loop stops. It is also capped
/// in rounds and stations, and names how it left.
/// `BREP_SECTION_REFINE_GATE=off` switches it off, for the before/after reading
/// and nothing else.
pub(super) const SECTION_MIDSPAN_FRACTION: f64 = 0.1;
const SECTION_MIDSPAN_ROUNDS: usize = 4;
/// A round earns another if it cuts the worst mid-span miss by this factor
/// (h^4 promises 16 for a halved spacing) OR lowers the count of spans
/// missing: near the floor a few isolated spans hold the worst while the count
/// falls, and on a corner the count GROWS round on round.
const SECTION_MIDSPAN_PROGRESS: f64 = 0.5;

/// Squared distance from `point` to the segment `from`-`to`.
fn distance_to_segment_sq(point: Vec3, from: Vec3, to: Vec3) -> f64 {
    let d = to.sub(from);
    let l2 = d.dot(d);
    let t = if l2 > 0.0 { (point.sub(from).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
    let q = from.add(d.scale(t));
    point.sub(q).dot(point.sub(q))
}

/// How a mid-span refinement left (`SECTION_MIDSPAN_*`), for the pair trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum MidspanExit {
    /// Every mid-span within the floor of both carriers.
    Converged,
    /// The unrefined fit's worst mid-span is within the gate: the section is
    /// sampled well enough that its degree carries it, and stations would
    /// cost more downstream than they return.
    BelowGate,
    /// Some mid-span still misses and no inserted point could improve it
    /// (the Newton failed, wandered off the span, or landed on a station
    /// already present).
    Stalled,
    /// A round neither halved the worst miss nor lowered the count of spans
    /// missing: not the h^4 regime, so the section has a corner the
    /// interpolant cannot follow. That round's fit is discarded.
    NoProgress,
    RoundBudget,
    StationCeiling,
}

/// Measure `fit` at every mid-span against both carriers; return the worst
/// distance, the count of spans that miss `floor` and, for each of them the
/// Newton could place, the refined intersection point to insert (with the
/// span's index).
fn midspan_misses(
    fit: &PolylineFit,
    first: &NurbsSurface,
    second: &NurbsSurface,
    floor: f64,
    tolerance: f64,
) -> Result<(f64, usize, Vec<(usize, Vec3)>), String> {
    let mut worst = 0.0f64;
    let mut missing = 0usize;
    let mut inserts = Vec::new();
    for span in 0..fit.parameters.len().saturating_sub(1) {
        let mid = 0.5 * (fit.parameters[span] + fit.parameters[span + 1]);
        let point = fit.curve.evaluate(mid)?;
        let off = project_point_to_surface(first, point)?
            .distance
            .max(project_point_to_surface(second, point)?.distance);
        worst = worst.max(off);
        if off <= floor {
            continue;
        }
        missing += 1;
        // The refined point must stay on THIS span: half its chord is the reach.
        let reach = 0.5 * fit.kept[span + 1].sub(fit.kept[span]).length();
        if let Some(refined) = crate::surface_surface_intersection::refine_to_intersection(
            first, second, point, tolerance, reach,
        )? {
            inserts.push((span, refined));
        }
    }
    Ok((worst, missing, inserts))
}

/// Every mid-span miss of `fit` against both carriers, measured and nothing
/// else: what [`even_sliver_ends_measured`] reads before and after evening.
/// It refines nothing, so a section without a sliver end pays only the
/// projections.
fn midspan_misses_only(fit: &PolylineFit, first: &NurbsSurface, second: &NurbsSurface) -> Result<Vec<f64>, String> {
    let mut misses = Vec::with_capacity(fit.parameters.len());
    for span in 0..fit.parameters.len().saturating_sub(1) {
        let mid = 0.5 * (fit.parameters[span] + fit.parameters[span + 1]);
        let point = fit.curve.evaluate(mid)?;
        let off = project_point_to_surface(first, point)?
            .distance
            .max(project_point_to_surface(second, point)?.distance);
        misses.push(off);
    }
    Ok(misses)
}

/// The signature of a global interpolant ringing on a sliver end span: the
/// worst mid-span miss sits in one of the [`SLIVER_END_SPANS`] spans at either
/// end, is over the fit `floor`, AND stands [`SLIVER_RING_CONTRAST`] times
/// above the median of the interior spans. A section sampled too coarsely for the carriers misses
/// everywhere at once and does not show this; a section with fewer than two
/// interior spans offers no evidence either way. Returns the end-span worst
/// and the interior median when the signature is present.
fn ringing_signature(misses: &[f64], floor: f64) -> Option<(f64, f64)> {
    let spans = misses.len();
    if spans < 2 * SLIVER_END_SPANS + 2 {
        return None;
    }
    let ends = misses[..SLIVER_END_SPANS]
        .iter()
        .chain(&misses[spans - SLIVER_END_SPANS..])
        .copied()
        .fold(0.0f64, f64::max);
    let mut interior: Vec<f64> = misses[SLIVER_END_SPANS..spans - SLIVER_END_SPANS].to_vec();
    interior.sort_by(f64::total_cmp);
    let median = interior[interior.len() / 2];
    let worst = misses.iter().copied().fold(0.0f64, f64::max);
    // Below the fit floor there is nothing to even: a 1e-8 miss on a planar
    // section passed the contrast on fixture 26 and flipped its idempotence
    // reading while changing nothing a bar could see.
    (ends > floor.max(sliver_min_miss()) && ends >= worst && ends > sliver_ring_contrast() * median.max(f64::MIN_POSITIVE))
        .then_some((ends, median))
}

/// How many spans at each end of a section [`even_sliver_ends`] may even —
/// the same three it inspects — and where [`ringing_signature`] looks.
const SLIVER_END_SPANS: usize = 3;

/// The ringing signature's contrast: the end spans' worst mid-span miss over
/// the interior spans' median. A converged march leaves its interior at
/// 1e-9 and a sliver end rings at 1e-4..1e-3, so the ratio is 1e5 and up:
/// `anotherBooleanFail`'s eight sections read 1.3e5..2.2e6 (interior
/// 6e-10..2.6e-9, ends 1.6e-4..2.8e-3). A section whose interior itself
/// misses is under-sampled rather than ringing, and evening its end moves a
/// junction the assembler then cannot repair: fixture 28's pair 822×118
/// (interior 1.5e-6, ends 1.4e-3, contrast 930) refused the reversed
/// subtract whenever it was evened, while its pairs 474×118 (1.7e4) and
/// 842×118 (1.4e5) are evened and build. Fixture 25's two sections read
/// 2.2e3 and 5.9e3 (interior 2e-8) and keep their stations and their pins.
/// 3e4 sits at least 4× from the nearest reading on either side
/// (2026-09-30).
const SLIVER_RING_CONTRAST: f64 = 3.0e4;

/// `BREP_SECTION_SLIVER_CONTRAST` overrides [`SLIVER_RING_CONTRAST`] for the
/// measurement that picks it.
fn sliver_ring_contrast() -> f64 {
    static CONTRAST: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *CONTRAST.get_or_init(|| {
        std::env::var("BREP_SECTION_SLIVER_CONTRAST")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|contrast| *contrast >= 1.0)
            .unwrap_or(SLIVER_RING_CONTRAST)
    })
}

/// `BREP_SECTION_SLIVER_MIN_MISS` raises the absolute floor under the end
/// spans' worst miss, for measurement only (the shipped floor is the fit
/// tolerance).
fn sliver_min_miss() -> f64 {
    static MIN: std::sync::OnceLock<f64> = std::sync::OnceLock::new();
    *MIN.get_or_init(|| {
        std::env::var("BREP_SECTION_SLIVER_MIN_MISS")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .filter(|miss| *miss >= 0.0)
            .unwrap_or(0.0)
    })
}

/// How a sliver-end evening left, for the census line.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum SliverEvening {
    /// No end span was shorter than the fraction, or the hatch is `off`.
    NoSliver,
    /// A sliver end exists but the unevened fit does not show the ringing
    /// signature ([`ringing_signature`]): its stations stand.
    NotRinging { slivers: usize, worst: f64 },
    /// The evened section's fit reads at least [`SLIVER_END_GAIN`] times
    /// closer to the carriers at its worst mid-span: the evened fit stands.
    Accepted { slivers: usize, before: f64, after: f64, interior: f64 },
    /// Evening did not earn its keep: the unevened fit stands, the stations
    /// untouched.
    Rejected { slivers: usize, before: f64, after: f64, interior: f64 },
}

/// A sliver-end evening must earn its keep: the evened fit stands only when
/// its worst mid-span miss against both carriers falls by at least this
/// factor. Measured on 2026-09-30 against the blanket default: `resolve` on
/// every section that HAS a sliver end refused the reversed subtract of
/// fixture 28 (`march_swap_rescue_recovers_reversed_subtract_t217`, an open
/// loop) and made fixture 26's union read differently before and after
/// operand splitting, while on `anotherBooleanFail` (2.8e-3 → 5e-8) and
/// fixture 25 (2.0e-5 / 3.4e-5 → 9.2e-7) it cut the miss by 20× to 5e4×. A
/// section whose evened fit is not clearly better keeps its stations.
const SLIVER_END_GAIN: f64 = 2.0;

/// [`even_sliver_ends`] with the measurement that decides whether it stands:
/// fit the section as marched; if its mid-spans show the ringing signature
/// ([`ringing_signature`]), even its sliver ends, fit again, and keep the
/// evened fit only when its worst mid-span wins by [`SLIVER_END_GAIN`].
/// Returns the section, its fit, the recovery flag of the fit that stands and
/// the outcome.
pub(super) fn even_sliver_ends_measured(
    section: Vec<Vec3>,
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    fit_tolerance: f64,
    maximum_fit_points: usize,
    local_fit: bool,
) -> Result<(Vec<Vec3>, PolylineFit, bool, SliverEvening), String> {
    let (fit, recovered) = crate::fit::fit_polyline_of_degree_with_recovery(
        &section,
        fit_tolerance,
        maximum_fit_points,
        local_fit,
        section_fit_degree(),
    )?;
    let mut evened = section.clone();
    let slivers = even_sliver_ends(&mut evened, first, second, tolerance)?;
    if slivers == 0 {
        return Ok((section, fit, recovered, SliverEvening::NoSliver));
    }
    let misses = midspan_misses_only(&fit, first, second)?;
    let Some((before, interior)) = ringing_signature(&misses, fit_tolerance) else {
        let worst = misses.iter().copied().fold(0.0f64, f64::max);
        return Ok((section, fit, recovered, SliverEvening::NotRinging { slivers, worst }));
    };
    let (evened_fit, evened_recovered) = crate::fit::fit_polyline_of_degree_with_recovery(
        &evened,
        fit_tolerance,
        maximum_fit_points,
        local_fit,
        section_fit_degree(),
    )?;
    let after_misses = midspan_misses_only(&evened_fit, first, second)?;
    let after = after_misses.iter().copied().fold(0.0f64, f64::max);
    if let Ok(path) = std::env::var("BREP_SECTION_SLIVER_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let chords = |points: &[Vec3]| points.windows(2).map(|pair| format!("{:.4e}", pair[1].sub(pair[0]).length())).collect::<Vec<_>>().join(" ");
            let fmt = |values: &[f64]| values.iter().map(|value| format!("{value:.2e}")).collect::<Vec<_>>().join(" ");
            let _ = writeln!(file, "sliver-end detail: stations {} -> {} kept {} -> {}", section.len(), evened.len(), fit.kept.len(), evened_fit.kept.len());
            let _ = writeln!(file, "sliver-end detail: chords before: {}", chords(&section));
            let _ = writeln!(file, "sliver-end detail: chords after:  {}", chords(&evened));
            let _ = writeln!(file, "sliver-end detail: misses before: {}", fmt(&misses));
            let _ = writeln!(file, "sliver-end detail: misses after:  {}", fmt(&after_misses));
            let _ = writeln!(file, "sliver-end detail: ends before: {:?} {:?}", section[0], section[section.len() - 1]);
            let _ = writeln!(file, "sliver-end detail: ends after:  {:?} {:?}", evened[0], evened[evened.len() - 1]);
        }
    }
    if after * SLIVER_END_GAIN <= before {
        Ok((evened, evened_fit, evened_recovered, SliverEvening::Accepted { slivers, before, after, interior }))
    } else {
        Ok((section, fit, recovered, SliverEvening::Rejected { slivers, before, after, interior }))
    }
}

/// Refit `section` until its mid-spans sit within `floor` of both carriers,
/// inserting true intersection points where they do not. Returns the fit that
/// stands, the worst mid-span miss it still carries, the unrefined fit's worst
/// (what the gate read), the rounds spent and the exit.
/// Keep the single best measured refiner state (root grant 2026-10-04,
/// accepted-representative sections only): replaced when `worst` is finite and
/// no larger than the kept one (ties go to the later state). An unreadable
/// reading is never kept. At most one snapshot is held.
pub(super) fn keep_best_state<S>(best: &mut Option<(S, f64)>, state: impl FnOnce() -> S, worst: f64) {
    if worst.is_finite() && best.as_ref().is_none_or(|(_, kept)| worst <= *kept) {
        *best = Some((state(), worst));
    }
}

/// The rollback a budget-type exit (RoundBudget, StationCeiling, Stalled)
/// takes: on a section where an ACCEPTED representative round ran, the kept
/// best state when its COMMON reading is strictly lower than the current
/// state's common reading on the same instrument (both carriers, finite, at the
/// fit's mid-spans and its own knot-span quarter points). Otherwise `None`:
/// the exit returns the current state exactly as before, with nothing cloned.
/// An unreadable current reading never rolls back (it is not hidden).
pub(super) fn take_rollback<S>(best: &mut Option<(S, f64)>, current_common: f64, representative_ran: bool) -> Option<(S, f64)> {
    let lower = matches!(best, Some((_, kept)) if representative_ran && current_common.is_finite() && *kept < current_common);
    if lower { best.take() } else { None }
}

/// Which fit a refinement RETURNED: a receipt for controls, deciding nothing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum RefinedFitSource {
    /// The standing fit the caller passed in.
    Standing,
    /// The original full refit of the run, made directly (`full-unmatched`,
    /// or the retry after an unreadable representative or a seeded stall).
    FullRefit,
    /// The seeded (forced-station) call and its own path: a `Fallback` path
    /// IS the original full refit.
    Seeded(crate::fit::SeededPath),
    /// The representative candidate call and its own path: a `Fallback` path
    /// IS the original full refit.
    Representatives(crate::fit::SeededPath),
    /// The best measured state the budget-exit rollback restored.
    Rollback,
}

/// The refiner's receipt: whether the run took the representative route at
/// all, and which fit it returned (None on an error return). Production
/// callers discard it; only controls read it.
#[allow(dead_code)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct RefineReceipt {
    pub original_consistent: bool,
    pub returned: Option<RefinedFitSource>,
}

pub(super) fn refine_section_against_carriers(
    section: &mut Vec<Vec3>,
    fit: PolylineFit,
    first: &NurbsSurface,
    second: &NurbsSurface,
    fit_tolerance: f64,
    tolerance: f64,
    local_fit: bool,
    gate: f64,
) -> Result<(PolylineFit, f64, f64, usize, MidspanExit), String> {
    // `BREP_SECTION_REPRESENTATIVES=0` restores the seeded path everywhere for
    // an attribution A/B only; read here, once per call, as before.
    let representatives = std::env::var("BREP_SECTION_REPRESENTATIVES").as_deref() != Ok("0");
    refine_section_against_carriers_with(section, fit, first, second, fit_tolerance, tolerance, local_fit, gate, representatives, &mut RefineReceipt::default())
}

/// [`refine_section_against_carriers`] with the representative switch passed
/// in (the env hatch's value, so a control can take the seeded route without
/// touching the process environment) and a receipt of the fit returned.
#[allow(clippy::too_many_arguments)]
pub(super) fn refine_section_against_carriers_with(
    section: &mut Vec<Vec3>,
    fit: PolylineFit,
    first: &NurbsSurface,
    second: &NurbsSurface,
    fit_tolerance: f64,
    tolerance: f64,
    local_fit: bool,
    gate: f64,
    representatives: bool,
    receipt: &mut RefineReceipt,
) -> Result<(PolylineFit, f64, f64, usize, MidspanExit), String> {
    // A reused receipt starts clean, so an error return leaves `returned`
    // None as documented, never a previous call's value.
    *receipt = RefineReceipt::default();
    // Which fit `fit` currently is, and which fit `previous` holds.
    let mut fit_source = RefinedFitSource::Standing;
    let mut previous_source = RefinedFitSource::Standing;
    let floor = fit_tolerance * SECTION_MIDSPAN_FRACTION;
    let mut unrefined = f64::NAN;
    let mut fit = fit;
    let mut rounds = 0usize;
    let mut previous: Option<(PolylineFit, f64, usize)> = None;
    // Every true intersection point inserted so far: the residual-driven bend
    // witnesses, which the refit must keep as stations.
    let mut inserted_points: Vec<Vec3> = Vec::new();
    // `BREP_SECTION_REFINE_CENSUS`: one `section-refine round:` line per round,
    // bound to its section by the run's head and tail, so a stall can be read
    // round by round (debug only; nothing below reads it).
    let round_census = std::env::var("BREP_SECTION_REFINE_CENSUS").ok();
    let (head, tail) = (section[0], section[section.len() - 1]);
    // Whether EVERY original point of the run lies on both carriers within the
    // fit tolerance (projector upper readings, NaN = no): only such a run takes
    // the refiner-specific representative candidate below. A run with any
    // point off a carrier keeps the existing seeded path exactly (moving or
    // discounting input points is the reserved relocation ruling).
    // `representatives` false (the `BREP_SECTION_REPRESENTATIVES=0` hatch)
    // restores the seeded path everywhere for an attribution A/B only.
    let original_consistent = representatives
        && section.iter().all(|point| {
            let a = project_point_to_surface(first, *point).map_or(f64::NAN, |projection| projection.distance);
            let b = project_point_to_surface(second, *point).map_or(f64::NAN, |projection| projection.distance);
            a.is_finite() && b.is_finite() && a <= fit_tolerance && b <= fit_tolerance
        });
    receipt.original_consistent = original_consistent;
    let mut refit_path = String::from("standing");
    let mut refit_seeds = 0usize;
    let mut refit_seeds_kept = 0usize;
    let mut refit_seeds_outside_pool = 0usize;
    let mut refit_pool = 0usize;
    let mut refit_pool_discarded = 0usize;
    let mut refit_pool_floor = f64::NAN;
    let mut refit_seed_min_chord = f64::NAN;
    // Trace only: what the closest seed pair is made of (kept station or
    // refiner insert at each end), to place the near pair upstream.
    let mut refit_seed_min_pair = "n/a";
    // Trace only: every seed the full refit's pool rejects, by ORIGIN (a march
    // station of the run, or a refiner insert) and by REASON (dropped by
    // `station_pool`'s simplification, or kept by it and then conditioned
    // away as a near-duplicate): [march simplified, march conditioned,
    // insert simplified, insert conditioned].
    let mut refit_rejected = [0usize; 4];
    // Fits made for the round being judged (2 when its seeded fit stalled and
    // the full refit was tried as well), and that retry's own cost.
    let mut refit_trials = 1usize;
    let mut retry_ms = 0.0f64;
    // Whether the fit judged this round is an ACCEPTED seeded fit (not merely
    // a seeded call: a seeded call that fell back already IS the full refit).
    let mut seeded_refit = false;
    // Whether the fit judged this round is an ACCEPTED representative
    // candidate, and what that candidate call spent (its own interpolations
    // and whether it ended on the full refit), kept apart from the fit's
    // report, which on a fallback must stay the full refit's to the bit.
    let mut representative_fit = false;
    let mut refit_cost = crate::fit::RepresentativeCost::default();
    // Whether an ACCEPTED representative round ran on this section, and the one
    // best measured (fit, section) state kept for its budget exits.
    let mut representative_ran = false;
    let mut best_state: Option<((PolylineFit, Vec<Vec3>), f64)> = None;
    loop {
        // On an accepted representative candidate a read or refinement ERROR is
        // not a refusal: the section takes the original full refit of the same
        // run instead, as an unreadable read does below.
        let (mut worst, missing, inserts) = match midspan_misses(&fit, first, second, floor, tolerance) {
            Ok(reading) => reading,
            Err(_) if representative_fit => {
                representative_fit = false;
                seeded_refit = false;
                let maximum_points = section.len().min(MAX_FIT_STATIONS);
                fit = fit_polyline_of_degree(section, fit_tolerance, maximum_points, local_fit, section_fit_degree())?;
                fit_source = RefinedFitSource::FullRefit;
                refit_path = String::from("full-after-unreadable-representative");
                refit_trials += 1;
                refit_cost.full_refit = true;
                continue;
            }
            Err(error) => return Err(error),
        };
        // An ACCEPTED representative candidate is judged on more than its
        // mid-spans: both carriers must read finite at every mid-span and at
        // the quarter points of the curve's own knot spans, and its worst is
        // the worse of the two reads (sampled, projector upper readings; the
        // carrier target is unchanged). An unreadable read never passes: the
        // section takes the original full refit of the same run instead.
        if representative_fit {
            let interior = knot_interior_carrier_upper(&fit.curve, first, second);
            let midspans = midspan_carrier_upper(&fit, first, second);
            if interior.is_finite() && midspans.is_finite() {
                worst = worst.max(interior).max(midspans);
            } else {
                representative_fit = false;
                seeded_refit = false;
                let maximum_points = section.len().min(MAX_FIT_STATIONS);
                fit = fit_polyline_of_degree(section, fit_tolerance, maximum_points, local_fit, section_fit_degree())?;
                fit_source = RefinedFitSource::FullRefit;
                refit_path = String::from("full-after-unreadable-representative");
                refit_trials += 1;
                refit_cost.full_refit = true;
                continue;
            }
        }
        if let Some(path) = &round_census {
            use std::io::Write;
            if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                let _ = writeln!(
                    file,
                    "section-refine round: head ({:.9}, {:.9}, {:.9}) tail ({:.9}, {:.9}, {:.9}) round {rounds} fit {refit_path} trials {refit_trials} retry_ms {retry_ms:.3} kept {} seeds {refit_seeds} seeds_kept {refit_seeds_kept} seeds_outside_pool {refit_seeds_outside_pool} seed_min_chord {refit_seed_min_chord:.3e} seed_min_pair {refit_seed_min_pair} rejected_march_simplified {} rejected_march_conditioned {} rejected_insert_simplified {} rejected_insert_conditioned {} pool {refit_pool} pool_discarded {refit_pool_discarded} pool_floor {refit_pool_floor:.3e} exit {:?} deviation {:.3e} floor {:.3e} run {} worst {:.3e} missing {} inserts_offered {} representatives {} witnesses {} candidate_fits {} candidate_full_refit {} candidate_end_offsets {:.3e}/{:.3e} seed_role {}",
                    head.x, head.y, head.z, tail.x, tail.y, tail.z,
                    fit.kept.len(),
                    refit_rejected[0], refit_rejected[1], refit_rejected[2], refit_rejected[3], fit.report.exit, fit.report.deviation, fit.report.floor, section.len(), worst, missing, inserts.len(),
                    refit_cost.representatives, refit_cost.witnesses, refit_cost.candidate_fits, refit_cost.full_refit,
                    refit_cost.start_offset, refit_cost.end_offset,
                    // On the representative path the seeds counted above are
                    // WITNESSES (not forced); on the seeded path they are forced.
                    if original_consistent { "witness" } else { "forced" }
                );
                // Trace only: where the full refit's band-raised floor comes
                // from on THIS run, and whether the run's own points are on
                // the carriers (input-consistent) or not. The band is
                // `dropped_band`'s reading (largest distance of a point the
                // pool drops from the segment between its surviving
                // neighbours); its point is classified as simplified away or
                // conditioned away, with its carrier distance and its gap to
                // the nearest run neighbour. Projector distances are upper
                // readings.
                let pool = crate::fit::polyline_station_pool(section, fit_tolerance);
                let simplified = crate::fit::simplify_polyline_indices(section, fit_tolerance);
                let mut band = (0.0f64, usize::MAX);
                for pair in pool.windows(2) {
                    for index in pair[0] + 1..pair[1] {
                        let off = distance_to_segment_sq(section[index], section[pair[0]], section[pair[1]]).sqrt();
                        if off > band.0 {
                            band = (off, index);
                        }
                    }
                }
                let carrier = |point: Vec3| -> f64 {
                    let a = project_point_to_surface(first, point).map_or(f64::NAN, |projection| projection.distance);
                    let b = project_point_to_surface(second, point).map_or(f64::NAN, |projection| projection.distance);
                    if a.is_nan() || b.is_nan() { f64::NAN } else { a.max(b) }
                };
                let mut run_carrier = 0.0f64;
                for point in section.iter() {
                    let off = carrier(*point);
                    run_carrier = if off.is_nan() || run_carrier.is_nan() { f64::NAN } else { run_carrier.max(off) };
                }
                let (kind, band_carrier, gap) = if band.1 == usize::MAX {
                    ("none", f64::NAN, f64::NAN)
                } else {
                    let index = band.1;
                    let kind = if simplified.binary_search(&index).is_ok() { "conditioned" } else { "simplified" };
                    let mut gap = f64::INFINITY;
                    if index > 0 {
                        gap = gap.min(section[index].sub(section[index - 1]).length());
                    }
                    if index + 1 < section.len() {
                        gap = gap.min(section[index + 1].sub(section[index]).length());
                    }
                    (kind, carrier(section[index]), gap)
                };
                let _ = writeln!(
                    file,
                    "section-refine band: head ({:.9}, {:.9}, {:.9}) tail ({:.9}, {:.9}, {:.9}) round {rounds} run {} pool {} tolerance {:.3e} band {:.3e} band_index {} of {} band_kind {kind} band_point_carrier_upper {band_carrier:.3e} band_point_gap {gap:.3e} run_carrier_upper {run_carrier:.3e}",
                    head.x, head.y, head.z, tail.x, tail.y, tail.z,
                    section.len(), pool.len(), fit_tolerance, band.0,
                    if band.1 == usize::MAX { -1 } else { band.1 as i64 }, section.len()
                );
            }
        }
        if rounds == 0 {
            unrefined = worst;
        }
        // The rollback path's COMMON reading, only once an accepted
        // representative round has run and only on a run the refit can hold
        // (<= MAX_FIT_STATIONS): both carriers, finite, at the fit's mid-spans
        // and its own knot-span quarter points, for EVERY state compared
        // (representative, standing or full alike). One snapshot, the fit and
        // the run it was measured on as one coherent pair: its payload is one
        // PolylineFit and at most MAX_FIT_STATIONS points. Off this path
        // nothing is read or cloned.
        let mut current_common = f64::NAN;
        if representative_ran && section.len() <= MAX_FIT_STATIONS {
            let interior = knot_interior_carrier_upper(&fit.curve, first, second);
            let midspans = midspan_carrier_upper(&fit, first, second);
            current_common = if interior.is_finite() && midspans.is_finite() { interior.max(midspans) } else { f64::NAN };
            keep_best_state(&mut best_state, || (fit.clone(), section.clone()), current_common);
        }
        if worst <= floor {
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Converged));
        }
        if rounds == 0 && worst <= gate {
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::BelowGate));
        }
        if let Some((earlier, earlier_worst, earlier_missing)) = previous.take() {
            // A round that did not earn its keep is discarded whole: the fit
            // of the last round that did stands (on a corner, the unrefined
            // fit), not whichever of the two happens to read a smaller max.
            if worst > earlier_worst * SECTION_MIDSPAN_PROGRESS && missing >= earlier_missing {
                // A SEEDED round that did not earn its keep is refitted once
                // with the original full refit of the same denser run, and
                // that is judged against the same earlier round before the
                // section stops. The seeded fit keeps fewer stations; on the
                // helmet's 2036x3009 section that gave back the whole of the
                // full refit's gain (6.335e-6 stood where 5.596e-7 was
                // reached). The full refit is the round the refiner made
                // before the seeded entry existed; only a round that fails
                // both ends the section, exactly as before.
                if seeded_refit {
                    seeded_refit = false;
                    let maximum_points = section.len().min(MAX_FIT_STATIONS);
                    let retry_started = round_census.is_some().then(Instant::now);
                    fit = fit_polyline_of_degree(section, fit_tolerance, maximum_points, local_fit, section_fit_degree())?;
                    fit_source = RefinedFitSource::FullRefit;
                    // Trace: one more fit for the SAME round, its cost, and no
                    // seed counters carried over from the rejected seeded fit.
                    refit_path = String::from("full-after-seeded-stall");
                    // Every fit this round made: the candidate's (or the seeded
                    // call's one) plus this retry.
                    refit_trials += 1;
                    retry_ms = retry_started.map_or(0.0, |started| started.elapsed().as_secs_f64() * 1e3);
                    refit_seeds = 0;
                    refit_seeds_kept = 0;
                    refit_seeds_outside_pool = 0;
                    refit_seed_min_chord = f64::NAN;
                    refit_seed_min_pair = "n/a";
                    refit_rejected = [0; 4];
                    representative_fit = false;
                    // Keep the candidate's own counts; the retry is the full refit.
                    refit_cost.full_refit = true;
                    previous = Some((earlier, earlier_worst, earlier_missing));
                    continue;
                }
                receipt.returned = Some(previous_source);
                return Ok((earlier, earlier_worst, unrefined, rounds, MidspanExit::NoProgress));
            }
        }
        if inserts.is_empty() {
            if let Some(((kept_fit, kept_run), kept_common)) = take_rollback(&mut best_state, current_common, representative_ran) {
                *section = kept_run;
                receipt.returned = Some(RefinedFitSource::Rollback);
                return Ok((kept_fit, kept_common, unrefined, rounds, MidspanExit::Stalled));
            }
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Stalled));
        }
        if rounds >= SECTION_MIDSPAN_ROUNDS {
            if let Some(((kept_fit, kept_run), kept_common)) = take_rollback(&mut best_state, current_common, representative_ran) {
                *section = kept_run;
                receipt.returned = Some(RefinedFitSource::Rollback);
                return Ok((kept_fit, kept_common, unrefined, rounds, MidspanExit::RoundBudget));
            }
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::RoundBudget));
        }
        if section.len() + inserts.len() > MAX_FIT_STATIONS {
            if let Some(((kept_fit, kept_run), kept_common)) = take_rollback(&mut best_state, current_common, representative_ran) {
                *section = kept_run;
                receipt.returned = Some(RefinedFitSource::Rollback);
                return Ok((kept_fit, kept_common, unrefined, rounds, MidspanExit::StationCeiling));
            }
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::StationCeiling));
        }
        // Insert each refined point into the run between its span's stations,
        // on the nearest segment of the run there. Positions are located
        // against the run BEFORE any insertion of this round, then applied
        // from the back so earlier indices stay valid. The kept stations are
        // found by a FORWARD search from the previous one: a closed section
        // ends on its first station, and a search from the start would put
        // the last span's far end at index 0 and skip that span forever.
        let station_index = |point: Vec3, from: usize| -> Option<usize> {
            section[from..]
                .iter()
                .position(|p| p.sub(point).length() <= 1e-15 * (1.0 + point.length()))
                .map(|offset| from + offset)
        };
        let mut placed: Vec<(usize, Vec3)> = Vec::new();
        for (span, point) in inserts {
            let Some(lo) = station_index(fit.kept[span], 0) else {
                continue;
            };
            let Some(hi) = station_index(fit.kept[span + 1], lo + 1) else {
                continue;
            };
            if hi <= lo {
                continue;
            }
            if section[lo..=hi].iter().any(|p| p.sub(point).length() <= floor) {
                continue; // already a station: inserting it again refines nothing
            }
            let mut best = (lo, f64::INFINITY);
            for index in lo..hi {
                let d2 = distance_to_segment_sq(point, section[index], section[index + 1]);
                if d2 < best.1 {
                    best = (index, d2);
                }
            }
            placed.push((best.0 + 1, point));
        }
        if placed.is_empty() {
            if let Some(((kept_fit, kept_run), kept_common)) = take_rollback(&mut best_state, current_common, representative_ran) {
                *section = kept_run;
                receipt.returned = Some(RefinedFitSource::Rollback);
                return Ok((kept_fit, kept_common, unrefined, rounds, MidspanExit::Stalled));
            }
            receipt.returned = Some(fit_source);
            return Ok((fit, worst, unrefined, rounds, MidspanExit::Stalled));
        }
        placed.sort_by(|a, b| b.0.cmp(&a.0));
        for (at, point) in placed {
            section.insert(at, point);
            inserted_points.push(point);
        }
        // The refit keeps the stations the current fit kept and every point
        // inserted so far as SEEDS, and grows only where it is measured to
        // miss (`fit_polyline_seeded_with_path`). It is no longer the whole denser run
        // from the run's own count: that interpolated every march station the
        // standing fit had dropped as well, 3x the controls on the skew-cylinder
        // loop for the same carrier floor. The ceiling is the one the full refit
        // derives from the same `maximum_points`, and a seeded fit that cannot
        // meet the floor, or cannot be read, IS the full refit. The carrier
        // measure above still decides every round.
        let same = |a: &Vec3, b: &Vec3| a.x.to_bits() == b.x.to_bits() && a.y.to_bits() == b.y.to_bits() && a.z.to_bits() == b.z.to_bits();
        let mut seeds = Vec::with_capacity(fit.kept.len() + inserted_points.len());
        // The previous fit's kept stations alone: the representative candidate's
        // starting set (inserts are its witnesses, not forced stations).
        let mut kept_seeds = Vec::with_capacity(fit.kept.len());
        // The refiner's own inserted true-carrier points: the representative
        // candidate's TIGHT witnesses, held to this refiner's carrier floor.
        let mut insert_indices = Vec::with_capacity(inserted_points.len());
        let mut next_kept = 0usize;
        for (index, point) in section.iter().enumerate() {
            // The kept stations are a subsequence of the run, in run order
            // (insertion only adds points), so a closed run's two equal ends
            // each match their own station.
            // An inserted point is a tight witness whether or not the previous
            // fit also kept it as a station (membership is independent).
            let inserted = inserted_points.iter().any(|candidate| same(point, candidate));
            if inserted {
                insert_indices.push(index);
            }
            if next_kept < fit.kept.len() && same(point, &fit.kept[next_kept]) {
                seeds.push(index);
                kept_seeds.push(index);
                next_kept += 1;
            } else if inserted {
                seeds.push(index);
            }
        }
        let maximum_points = section.len().min(MAX_FIT_STATIONS);
        // Every standing station must be found as a seed; if the pass missed
        // one, the seeded refit would silently lose it, so the section takes the
        // original full refit instead.
        refit_trials = 1;
        retry_ms = 0.0;
        let refit_source;
        let refit = if next_kept == fit.kept.len() {
            let (refit, path) = if original_consistent {
                let (refit, path, cost) = crate::fit::fit_polyline_representatives_with_witnesses(
                    section,
                    &kept_seeds,
                    &insert_indices,
                    floor,
                    fit_tolerance,
                    maximum_points,
                    local_fit,
                    section_fit_degree(),
                )?;
                // MIXED UNITS, named as such: the candidate's own
                // `fit_station_set` interpolations plus one per call of the
                // full refit `fit_polyline_of_degree`, which may itself run
                // several ladder and recovery interpolations that are NOT
                // counted here. Whole-job cost is the matched timing.
                refit_cost = cost;
                refit_trials = cost.candidate_fits + usize::from(cost.full_refit);
                (refit, path)
            } else {
                refit_cost = crate::fit::RepresentativeCost::default();
                crate::fit::fit_polyline_seeded_with_path(
                    section,
                    &seeds,
                    fit_tolerance,
                    maximum_points,
                    local_fit,
                    section_fit_degree(),
                )?
            };
            // Only an ACCEPTED seeded fit can be retried in full: a fallback
            // already is the full refit, and retrying it would repeat it.
            seeded_refit = matches!(path, crate::fit::SeededPath::Accepted { .. });
            representative_fit = original_consistent && seeded_refit;
            refit_source = if original_consistent { RefinedFitSource::Representatives(path) } else { RefinedFitSource::Seeded(path) };
            representative_ran |= representative_fit;
            if round_census.is_some() {
                refit_path = if original_consistent { format!("representatives-call:{path:?}") } else { format!("seeded-call:{path:?}") };
            }
            refit
        } else {
            seeded_refit = false;
            representative_fit = false;
            refit_cost = crate::fit::RepresentativeCost::default();
            refit_path = String::from("full-unmatched");
            refit_source = RefinedFitSource::FullRefit;
            fit_polyline_of_degree(section, fit_tolerance, maximum_points, local_fit, section_fit_degree())?
        };
        if round_census.is_some() {
            // Trace only: how many seeds the refit kept. This does NOT tell an
            // accepted seeded fit from the fallback (a full refit can keep
            // every seed too); the returned `SeededPath`, traced as `fit`, does.
            refit_seeds = seeds.len();
            refit_seeds_kept = seeds.iter().filter(|&&index| refit.kept.iter().any(|kept| same(kept, &section[index]))).count();
            // Seeds the full refit's pool would NOT interpolate (simplified
            // or conditioned away): forced stations only the seeded fit keeps.
            let pool = crate::fit::polyline_station_pool(section, fit_tolerance);
            refit_pool = pool.len();
            refit_pool_discarded = section.len() - pool.len();
            refit_pool_floor = crate::fit::polyline_station_pool_floor(section, fit_tolerance);
            refit_seeds_outside_pool = seeds.iter().filter(|index| !pool.contains(index)).count();
            // Seeds are in run order; the closest consecutive pair is what a
            // near-duplicate conditioning floor would act on.
            refit_seed_min_chord = seeds
                .windows(2)
                .map(|pair| section[pair[1]].sub(section[pair[0]]).length())
                .fold(f64::INFINITY, f64::min);
            let is_insert = |index: usize| inserted_points.iter().any(|inserted| same(inserted, &section[index]));
            refit_seed_min_pair = seeds
                .windows(2)
                .min_by(|a, b| {
                    section[a[1]].sub(section[a[0]]).length().total_cmp(&section[b[1]].sub(section[b[0]]).length())
                })
                .map_or("n/a", |pair| match (is_insert(pair[0]), is_insert(pair[1])) {
                    (false, false) => "station-station",
                    (false, true) => "station-insert",
                    (true, false) => "insert-station",
                    (true, true) => "insert-insert",
                });
            // Every rejected seed, classified. `station_pool` starts from this
            // same simplification, so a non-member that is not in it was
            // simplified away, and one that is in it was conditioned away.
            let simplified = crate::fit::simplify_polyline_indices(section, fit_tolerance);
            refit_rejected = [0; 4];
            for &index in seeds.iter().filter(|index| !pool.contains(index)) {
                let conditioned = simplified.binary_search(&index).is_ok();
                refit_rejected[2 * usize::from(is_insert(index)) + usize::from(conditioned)] += 1;
            }
        }
        previous = Some((std::mem::replace(&mut fit, refit), worst, missing));
        previous_source = std::mem::replace(&mut fit_source, refit_source);
        rounds += 1;
    }
}
use super::*;
use super::gate_census::{census_pair, fit_census_enabled};

pub fn build_imprints(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
) -> Result<ImprintResultRecord, KernelRefusal> {
    build_imprints_impl(solid_a, solid_b, options, false)
}

/// Imprint temporary open offset sheets. A section riding one sheet's trim
/// still cuts the other sheet when no boundary copy supplied that section.
pub(crate) fn build_carrier_imprints(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
) -> Result<ImprintResultRecord, KernelRefusal> {
    build_imprints_impl(solid_a, solid_b, options, true)
}

fn build_imprints_impl(
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    options: &ImprintOptions,
    open_carriers: bool,
) -> Result<ImprintResultRecord, KernelRefusal> {
    let mut section_evidence = false;
    let edges = edge_map(solid_a, solid_b);
    let mut builder = ImprintBuilder {
        open_carriers,
        edges,
        tolerance: options.tolerance,
        barrier_edges: HashSet::default(),
        overlap_ridden_edges: HashSet::default(),
        scale: solid_scale(solid_a).max(solid_scale(solid_b)),
        vertices: Vec::new(),
        vertex_radii: HashMap::default(),
        pieces: Vec::new(),
        by_face: HashMap::default(),
        edge_splits: HashMap::default(),
        marched_pieces: HashSet::default(),
        next_id: 1,
    };
    let mut first_faces = faces(solid_a, 0);
    let mut second_faces = faces(solid_b, 1);
    // Per-face trim-window carriers, computed once: tighter BVH bounds, and
    // the seed/march stages walk the window instead of the full carrier
    // (identical geometry and parameterization inside the window).
    //
    // A window LIFTED across a closed direction's seam (`restricted_carrier`)
    // is the exception to "same parameterization": it is the trim's frame, a
    // period off the carrier's on one side. Its face carries it as its
    // `chart`, and every uv the imprint reads for that face — the clip's
    // containment, the sections' pcurves, a seed's normal — is read there.
    let first_restricted: Vec<Option<MarchWindow>> = first_faces
        .iter()
        .map(|tagged| restricted_carrier(tagged.face))
        .collect();
    let second_restricted: Vec<Option<MarchWindow>> = second_faces
        .iter()
        .map(|tagged| restricted_carrier(tagged.face))
        .collect();
    for (tagged, window) in first_faces.iter_mut().zip(&first_restricted) {
        tagged.lifted = window.as_ref().filter(|window| window.lifted).map(|window| &window.surface);
    }
    for (tagged, window) in second_faces.iter_mut().zip(&second_restricted) {
        tagged.lifted = window.as_ref().filter(|window| window.lifted).map(|window| &window.surface);
    }
    let first_faces = first_faces;
    let second_faces = second_faces;
    // The lifted charts, by key, for the post-passes that rebuild a pcurve
    // after the mint (junction canonicalization, the origin dissolve, the
    // truncation bridge). A face absent here is read on its own carrier.
    let charts: FaceCharts<'_> = first_faces
        .iter()
        .chain(second_faces.iter())
        .filter_map(|tagged| tagged.lifted.map(|surface| (tagged.key(), surface)))
        .collect();
    let first_bounds = first_faces
        .iter()
        .zip(&first_restricted)
        .map(|(tagged, restricted)| {
            face_bounds(
                tagged.face,
                restricted.as_ref().map(|window| &window.surface),
                &builder.edges,
                tagged.operand,
                options.tolerance,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let second_bounds = second_faces
        .iter()
        .zip(&second_restricted)
        .map(|(tagged, restricted)| {
            face_bounds(
                tagged.face,
                restricted.as_ref().map(|window| &window.surface),
                &builder.edges,
                tagged.operand,
                options.tolerance,
            )
        })
        .collect::<Result<Vec<_>, _>>()?;
    let second_bvh = Bvh::build(&second_bounds);
    // Per-face classifier data and edge lists/subcurves, computed once and
    // reused across every pair the face participates in.
    let first_classify = first_faces
        .iter()
        .map(|tagged| SurfaceClassifyData::build(&tagged.face.surface, options.tolerance))
        .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
    let second_classify = second_faces
        .iter()
        .map(|tagged| SurfaceClassifyData::build(&tagged.face.surface, options.tolerance))
        .collect::<Result<Vec<_>, _>>().or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
    let mut face_edge_lists: HashMap<FaceKey, Vec<&EdgeRecord>> = HashMap::default();
    for tagged in first_faces.iter().chain(second_faces.iter()) {
        face_edge_lists.insert(tagged.key(), face_edges(*tagged, &builder.edges)?);
    }
    let mut subcurves: HashMap<(u8, u64), NurbsCurve> = HashMap::default();
    for (&key, edge) in &builder.edges {
        if !edge.degenerate {
            // Failures fall through: the use sites recompute and surface the
            // original error exactly where the uncached code did.
            if let Ok(curve) = edge_subcurve(edge) {
                subcurves.insert(key, curve);
            }
        }
    }
    let cached_subcurve = |operand: u8, edge: &EdgeRecord| -> Result<NurbsCurve, KernelRefusal> {
        match subcurves.get(&(operand, edge.id)) {
            Some(curve) => Ok(curve.clone()),
            None => edge_subcurve(edge),
        }
    };
    let mut profile = ImprintProfile::new();
    let mut tangent_nodes: Vec<Vec3> = Vec::new();
    let mut cosurface_pairs: Vec<(FaceKey, FaceKey)> = Vec::new();
    let mut paired = Vec::new();
    for (first_index, first) in first_faces.iter().enumerate() {
        let first = *first;
        paired.clear();
        second_bvh.overlapping(
            first_bounds[first_index],
            options.tolerance * 100.0,
            &mut paired,
        );
        paired.sort_unstable();
        let debug_pairs = std::env::var("BREP_DEBUG_PAIRS").is_ok();
        if debug_pairs {
            eprintln!(
                "bvh first_face={} -> {} candidate pairs {:?}",
                first.face.id,
                paired.len(),
                paired
                    .iter()
                    .map(|&index| second_faces[index].face.id)
                    .collect::<Vec<_>>()
            );
        }
        for &second_index in &paired {
            let second = second_faces[second_index];
            profile.pairs += 1;
            // Set when this pair's section passes through an isolated TANGENT
            // NODE inside both trims (see the classification below). The march
            // is attempted anyway — that is the whole point — but a marcher
            // that cannot get through a node must refuse in the tangent-node
            // class naming the node, not leak its own step-budget message.
            let mut tangent_node: Option<Vec3> = None;
            let mut lap_start = None;
            profile.lap(&mut lap_start);
            let pair_classification = classify_surface_pair_cached(
                &first.face.surface,
                &first_classify[first_index],
                &second.face.surface,
                &second_classify[second_index],
                options.tolerance,
                PAIR_ANGULAR_TOLERANCE,
            ).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
            profile.classify += profile.lap(&mut lap_start);
            if pair_classification.relation == SurfacePairRelation::Disjoint {
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: DISJOINT-cull sep={:.4e}",
                        first.face.id, second.face.id, pair_classification.minimum_separation
                    );
                }
                continue;
            }
            // Coincident carriers are never marched: their true intersection
            // is a 2D region, not a curve, so anything the marcher traces on
            // them is noise ("similar faces we do not intersect" — Golovanov
            // §6.2).  The sampled classification catches coincident pairs
            // the strict reconstruction test misses (partial overlaps,
            // differing parameterizations); both route to the boundary-curve
            // exchange below.
            let is_cosurface = pair_classification.relation == SurfacePairRelation::Cosurface
                || cosurface_pair(&first.face.surface, &second.face.surface, options.tolerance)?;
            profile.cosurface += profile.lap(&mut lap_start);
            if is_cosurface {
                if debug_pairs {
                    eprintln!("pair {}x{}: cosurface", first.face.id, second.face.id);
                }
                cosurface_pairs.push((first.key(), second.key()));
                for edge in &face_edge_lists[&second.key()] {
                    if !edge.degenerate {
                        builder.process_curve(
                            cached_subcurve(second.operand, edge)?,
                            first,
                            second,
                            &[first],
                            &[first],
                            false,
                        )?;
                    }
                }
                for edge in &face_edge_lists[&first.key()] {
                    if !edge.degenerate {
                        builder.process_curve(
                            cached_subcurve(first.operand, edge)?,
                            first,
                            second,
                            &[second],
                            &[second],
                            false,
                        )?;
                    }
                }
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // The exact section lanes below (planar-iso, analytic) mint this
            // pair's whole intersection, the part over either trim included, so
            // a SPAN of an edge lying on the other face would be a second copy
            // of a curve they already mint — which reorders the face's pieces
            // and with them its fragments' names (the 2026-09-10 chamfer groove
            // rim's `Box_PX` / `Box_PX_1`). Where they answer, only a whole
            // edge is exchanged, as before.
            let planar_iso = planar_iso_intersection(
                &first.face.surface,
                &second.face.surface,
                options.tolerance,
            )?;
            profile.planar_iso += profile.lap(&mut lap_start);
            let analytic = crate::intersect_analytic_pair(
                &first.face.surface,
                &second.face.surface,
                options.tolerance,
            );
            profile.analytic += profile.lap(&mut lap_start);
            let exact_section = planar_iso.is_some() || analytic.is_some();

            for edge in &face_edge_lists[&first.key()] {
                if !edge.degenerate {
                    let curve = cached_subcurve(first.operand, edge)?;
                    let spans = curve_spans_on_face(curve, second.face, &face_edge_lists[&second.key()], options.tolerance, exact_section)?;
                    for span in spans {
                        builder.process_curve(span, first, second, &[second], &[second], false)?;
                    }
                }
            }
            for edge in &face_edge_lists[&second.key()] {
                if !edge.degenerate {
                    let curve = cached_subcurve(second.operand, edge)?;
                    let spans = curve_spans_on_face(curve, first.face, &face_edge_lists[&first.key()], options.tolerance, exact_section)?;
                    for span in spans {
                        builder.process_curve(span, first, second, &[first], &[first], false)?;
                    }
                }
            }
            profile.lies_on += profile.lap(&mut lap_start);
            // CENSUS ONLY (`BREP_FIT_CENSUS=1`): the lane this pair takes under
            // the count-keyed and the geometry-keyed gates, and — when it is
            // marched here and answered exactly there — the point-set comparison
            // of the two, printed when `pair_census` is dropped. `imprint/gate_census.rs`.
            let mut pair_census = if fit_census_enabled() {
                census_pair(first, second, options.tolerance)?
            } else {
                None
            };
            if let Some(curve) = planar_iso {
                if debug_pairs {
                    eprintln!("pair {}x{}: planar_iso", first.face.id, second.face.id);
                }
                builder.process_curve(
                    curve,
                    first,
                    second,
                    &[first, second],
                    &[first, second],
                    true,
                )?;
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // Recognized analytic pairs produce their exact intersection
            // curves (lines, circles, ellipses) directly — no marching, no
            // polyline fitting, no chord-sag drift. An empty result is a
            // proof of non-intersection and also skips the marcher. (Read
            // above, with the planar-iso curve.)
            if let Some(curves) = analytic {
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: analytic x{}",
                        first.face.id,
                        second.face.id,
                        curves.len()
                    );
                }
                for curve in curves {
                    builder.process_curve(
                        curve,
                        first,
                        second,
                        &[first, second],
                        &[first, second],
                        true,
                    )?;
                }
                profile.process_curve += profile.lap(&mut lap_start);
                continue;
            }

            // A pair whose every touching sample is TANGENTIAL cannot
            // contain a transverse intersection curve: the marcher would
            // walk the tangency band's noise, which neither closes nor
            // reaches a boundary (the distance-0 glue-extrude runaway).
            // Shared topology at a tangential contact comes from the
            // boundary-curve exchange above ("similar faces we do not
            // intersect" extended to tangential contacts — Golovanov §6.2;
            // where surfaces CROSS through a tangency, off-line samples
            // have non-parallel normals and the pair still marches).
            if pair_classification.relation == SurfacePairRelation::NearTangent
                && pair_classification.tangential_only
            {
                // The coarse 5×5 classifier can flag `tangential_only` off an
                // incidental tangential KISS between two curved carriers and
                // miss the transverse loop where they actually cross (two
                // overlapping tori: their tubes cross while their inner walls
                // just touch). Before honouring the skip, a GATED supplemental
                // detector (denser two-sided seeding, tangency-band seeds
                // rejected, trace-exhaustion swallowed, transverse-only
                // branches) checks whether a genuine transverse curve of
                // meaningful length lies inside BOTH trims.
                //
                // Such a pair's intersection is SINGULAR where a tangency
                // sits on it, and what the imprint can do about that depends
                // entirely on the SHAPE of the tangency — see
                // `imprint/tangent_contact.rs`. An isolated NODE (two branches
                // crossing, as every equal-radius pair produces) is already
                // assembled correctly by the ordinary march plus the 2D
                // arrangement's pinch carving; an EXTENDED contact along a
                // whole curve has no section to imprint at all and tears the
                // shell. Both are classified below and only the second is
                // refused. Skipping either silently would DROP the
                // intersection (torus∪torus double-counts; torus−torus removes
                // nothing — the two structural bugs the semantic oracle
                // found), so nothing here ever falls through quietly.
                let first_march = first_restricted[first_index]
                    .as_ref()
                    .map(|window| &window.surface)
                    .unwrap_or(&first.face.surface);
                let second_march = second_restricted[second_index]
                    .as_ref()
                    .map(|window| &window.surface)
                    .unwrap_or(&second.face.surface);
                let supplemental = intersect_surfaces_supplemental(
                    first_march,
                    second_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: options.maximum_ssi_step,
                        ..Default::default()
                    },
                ).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
                let mut dropped_length = 0.0f64;
                let mut clipped: Vec<Vec<Vec3>> = Vec::new();
                for branch in &supplemental {
                    for run in clip_branch_to_trims(&branch.points, first, second)? {
                        if run.len() < 2 {
                            continue;
                        }
                        let length: f64 = run
                            .windows(2)
                            .map(|pair| pair[1].sub(pair[0]).length())
                            .sum();
                        dropped_length = dropped_length.max(length);
                        clipped.push(run);
                    }
                }
                if dropped_length <= options.tolerance * 100.0 {
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: tangential-only contact, march skipped",
                            first.face.id, second.face.id
                        );
                    }
                    continue;
                }
                // A transverse curve EXISTS. What would make it unimprintable is
                // a tangent NODE on it — and the node need not lie inside the
                // trims. Two equal-radius pipe arms are tangent to each other
                // where their axes' common perpendicular leaves the junction,
                // and a joint ball wider than the arms trims that crotch off
                // both faces; what is left inside the trims is an ordinary
                // crossing. So the question is not "does a transverse curve
                // exist" but "does the curve INSIDE BOTH TRIMS reach a
                // tangency". Only the latter is the checkerboard case.
                let mut tangency_inside_trims = false;
                let mut tangency_witness = Vec3::default();
                'clipped: for run in &clipped {
                    for (index, &point) in run.iter().enumerate() {
                        if pair_normals_parallel_at(first, second, point)? {
                            tangency_inside_trims = true;
                            tangency_witness = point;
                            if debug_pairs {
                                eprintln!(
                                    "pair {}x{}: TANGENCY at run index {}/{} ({:.6},{:.6},{:.6}) dropped_len={:.4e}",
                                    first.face.id, second.face.id, index, run.len(),
                                    point.x, point.y, point.z, dropped_length
                                );
                            }
                            break 'clipped;
                        }
                    }
                }
                if tangency_inside_trims {
                    // WHAT SHAPE is the tangency? The witness is only a marched
                    // sample within the transverse-seed angular gate, so it is
                    // REFINED onto the contact before being classified — at the
                    // raw witness a G1 cylinder/torus join and a genuine
                    // equal-radius node are eighteen-fold apart, which would be
                    // a band; at the refined contact they are thirty orders
                    // apart, which is a rank question. See
                    // `imprint/tangent_contact.rs`.
                    let contact = classify_tangent_contact(
                        &first.face.surface,
                        &second.face.surface,
                        tangency_witness,
                        options.tolerance,
                    )?;
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: contact classification {:?}",
                            first.face.id, second.face.id, contact
                        );
                    }
                    match contact {
                        // An isolated node: the section is a curve everywhere
                        // but that one point, so the ordinary march below runs
                        // and the 2D arrangement carves the pinch on both
                        // faces. `tangent_node` records it so a march that
                        // cannot get through still refuses in THIS class rather
                        // than leaking the marcher's own message.
                        Some(contact) if contact.is_isolated_node() => {
                            tangent_node = Some(contact.point);
                            tangent_nodes.push(contact.point);
                        }
                        // An extended contact (rank-deficient) or a witness we
                        // could not refine onto any contact at all (None — so
                        // nothing is proven and the pre-classification refusal
                        // stands). Neither has a section curve the imprint can
                        // represent.
                        other => {
                            let shape = match other {
                                Some(contact) => contact.describe(),
                                None => format!(
                                    "the tangency near ({:.6},{:.6},{:.6}) could not be refined \
                                     onto a contact, so its shape is unproven",
                                    tangency_witness.x, tangency_witness.y, tangency_witness.z
                                ),
                            };
                            return Err(KernelRefusal::new(
                                RefusalClass::TangentNodeSingularity,
                                KernelStage::Intersect,
                                format!(
                                    "boolean: unsupported singular/tangent-node surface intersection \
                                     between faces {} and {}: {shape}",
                                    first.face.id, second.face.id
                                ),
                            ));
                        }
                    }
                }
                if debug_pairs && tangent_node.is_none() {
                    // Transverse the whole way inside both trims: the coarse
                    // 5x5 classifier only saw the tangency the trims cut away.
                    eprintln!(
                        "pair {}x{}: classified tangential-only, but the curve inside both \
                         trims is transverse ({dropped_length:.4}) — marching",
                        first.face.id, second.face.id
                    );
                }
            }

            // The trim-window carriers: same surface and parameterization
            // over the window, so hit (u, v) values remain valid on the
            // originals; out-of-window intersections could never survive
            // clip_branch_to_trims and are not walked at all.
            let first_march = first_restricted[first_index]
                .as_ref()
                .map(|window| &window.surface)
                .unwrap_or(&first.face.surface);
            let second_march = second_restricted[second_index]
                .as_ref()
                .map(|window| &window.surface)
                .unwrap_or(&second.face.surface);
            let mut seed_points = Vec::new();
            // Smallest |edge_tangent · surface_normal| over accepted seeds — the
            // local grazing measure at the trim crossings (near 0 = the edge
            // pierces the other surface tangentially). Gates the near-tangent
            // clip-order rescue below to genuinely grazing pairs.
            let mut min_seed_tangency = f64::INFINITY;
            // Two grazing crossings of one trim edge bound a real, narrow
            // interval. It cannot tolerate a section fit that misses either
            // crossing, even when a coarse whole-model distance looks small.
            let mut paired_grazing_crossings = false;
            for (face, other_march, other) in
                [(first, second_march, second), (second, first_march, first)]
            {
                for edge in &face_edge_lists[&face.key()] {
                    if edge.degenerate {
                        continue;
                    }
                    let mut grazing_crossings: Vec<Vec3> = Vec::new();
                    for hit in intersect_curve_surface(&edge.curve, other_march, options.tolerance).or_refuse(KernelStage::Intersect, "intersect_curve_surface")?
                    {
                        if hit.t < edge.t0 - 1e-9 || hit.t > edge.t1 + 1e-9 {
                            continue;
                        }
                        let derivative = edge.curve.derivatives(hit.t, 1).or_refuse(KernelStage::Intersect, "derivatives")?[1];
                        // A STATIONARY POINT on the edge curve is not a reason
                        // to refuse the boolean. An involute flank starts at
                        // the base circle with an exactly zero tangent, so a
                        // gear tooth's profile carries one cusp per flank, and
                        // a hit landing on that parameter used to propagate
                        // "Vec3.normalized: zero-length vector" out of the
                        // whole union (the 2026-09-14 herringbone report: the
                        // second tooth band's z=0 cap edge meets the first
                        // band's flank exactly at the cusp). The direction is
                        // still defined as the one-sided limit, so read it from
                        // just inside the edge's own range; an edge with no
                        // direction at all simply contributes no seed, exactly
                        // as a hit whose face normal is unavailable already
                        // does two lines below. Seeds are marcher hints, so a
                        // dropped one costs a hint, never an answer.
                        // Escape hatch for tamper-verification:
                        // BREP_CUSP_TANGENT_RESCUE=0 restores the refusal.
                        let tangent = match derivative.normalized() {
                            Ok(tangent) => tangent,
                            Err(error) => {
                                if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
                                    return Err(error)
                                        .or_refuse(KernelStage::Intersect, "normalized");
                                }
                                match edge.curve.stationary_tangent_rescue(hit.t, edge.t0, edge.t1)
                                {
                                    Some(tangent) => tangent,
                                    None => continue,
                                }
                            }
                        };
                        // `hit` was solved on the other face's MARCH WINDOW, so
                        // its uv is in that face's chart — a period off the
                        // carrier's where the window is lifted, and `normal`
                        // clamps a parameter past its domain.
                        let normal = match other.chart().normal(hit.u, hit.v) {
                            Ok(normal) => normal,
                            Err(_) => continue,
                        };
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: seed edge {} t={:.6} p=({:.5},{:.5},{:.5}) |tan.n|={:.4} {}",
                                first.face.id,
                                second.face.id,
                                edge.id,
                                hit.t,
                                hit.point.x,
                                hit.point.y,
                                hit.point.z,
                                tangent.dot(normal).abs(),
                                if tangent.dot(normal).abs() >= 0.1 { "ACCEPT" } else { "reject" }
                            );
                        }
                        if tangent.dot(normal).abs() >= 0.1 {
                            seed_points.push(hit.point);
                            min_seed_tangency = min_seed_tangency.min(tangent.dot(normal).abs());
                            section_evidence = true;
                        } else {
                            if grazing_crossings.iter().any(|point| {
                                point.sub(hit.point).length() > assembler_weld(options.tolerance)
                            }) {
                                paired_grazing_crossings = true;
                            }
                            grazing_crossings.push(hit.point);
                        }
                    }
                }
            }
            profile.seeds += profile.lap(&mut lap_start);
            profile.marched_pairs += 1;
            if debug_pairs {
                eprintln!(
                    "pair {}x{}: marching (relation {:?})...",
                    first.face.id, second.face.id, pair_classification.relation
                );
            }
            let marched = intersect_surfaces(
                first_march,
                second_march,
                &SurfaceIntersectionOptions {
                    tolerance: options.tolerance,
                    maximum_step: march_maximum_step(
                        &pair_classification,
                        options.maximum_ssi_step,
                        builder.scale,
                    ),
                    seed_points: seed_points.clone(),
                    ..Default::default()
                },
            );
            // A pair carrying a tangent node keeps its OWN refusal class when
            // the march fails: the node is why the trace cannot close, and the
            // equal-radius torus pair depends on being told so rather than on
            // reading a step-budget message it cannot act on.
            let marched = match (marched, tangent_node) {
                (Err(error), Some(node)) => {
                    return Err(KernelRefusal::new(
                        RefusalClass::TangentNodeSingularity,
                        KernelStage::Intersect,
                        format!(
                            "boolean: unsupported singular/tangent-node surface intersection \
                             between faces {} and {}: the section through the tangent node at \
                             ({:.6},{:.6},{:.6}) could not be marched ({error})",
                            first.face.id, second.face.id, node.x, node.y, node.z
                        ),
                    ));
                }
                (marched, _) => marched,
            };
            let mut branches = marched
            .map_err(|error| {
                format!(
                    "{error} (marching faces {} and {})",
                    first.face.id, second.face.id
                )
            }).or_refuse(KernelStage::Intersect, "csg.imprint.driver")?;
            profile.march += profile.lap(&mut lap_start);
            let rescue_started = profile.enabled.then(Instant::now);
            // MARCH-ORDER SWAP RESCUE (hatch BREP_MARCH_SWAP_RESCUE=0).
            // `intersect_surfaces` is not order-symmetric: the coupled Newton
            // trace can fail to start/continue from a valid seed when the two
            // surfaces are presented in one operand order yet succeed in the
            // other. This is the sole reason t217's `subtract(sphere, step)`
            // fails while every other op passes — pair 118x842 (sphere-first)
            // marches ZERO branches, while 842x118 (step-first, the working
            // a\b order) marches the section from the IDENTICAL accepted pierce
            // seeds. When the forward order returns no branch AND an accepted
            // transverse pierce seed exists (so a real section provably crosses
            // both trims), retry the march with the surfaces swapped — i.e.
            // reproduce the exact call the working operand order makes for this
            // pair (measured: seed_only alone does NOT recover it — the section
            // is found from an auto-grid start, not from the near-tangent pierce
            // seeds, which the seed normal-cross gate rejects). The returned
            // branch points are 3D and therefore order-independent, so no
            // parameter remap is needed; they flow through the identical clip /
            // process_curve gates below, which drop any out-of-trim or
            // sub-length run exactly as today. The rescue only ever runs when
            // the forward order found NOTHING, so it cannot alter a pair that
            // already marched.
            if branches.is_empty()
                && !seed_points.is_empty()
                && pair_classification.relation == SurfacePairRelation::Candidate
                && std::env::var("BREP_MARCH_SWAP_RESCUE").as_deref() != Ok("0")
            {
                // FAIL-SOFT: the forward order already returned Ok(empty); a
                // swapped-march error (trace-exhaustion is a real error class —
                // `intersect_surfaces_supplemental` swallows it for exactly this
                // reason) must NOT convert that graceful empty into a hard error.
                // On Err, keep the (empty) forward result and carry on.
                let swapped = intersect_surfaces(
                    second_march,
                    first_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: march_maximum_step(
                            &pair_classification,
                            options.maximum_ssi_step,
                            builder.scale,
                        ),
                        seed_points: seed_points.clone(),
                        ..Default::default()
                    },
                )
                .unwrap_or_default();
                if debug_pairs {
                    eprintln!(
                        "pair {}x{}: MARCH-SWAP RESCUE attempt -> {} branches, pts {:?}",
                        first.face.id,
                        second.face.id,
                        swapped.len(),
                        swapped.iter().map(|b| b.points.len()).collect::<Vec<_>>()
                    );
                }
                if !swapped.is_empty() {
                    branches = swapped;
                }
            }
            // NEAR-TANGENT CLIP-ORDER RESCUE (hatch BREP_MARCH_SWAP_CLIP_RESCUE=0).
            // Companion to the empty-branch MARCH-SWAP RESCUE above:
            // `intersect_surfaces` is order-asymmetric not only in WHETHER it
            // marches, but in the exact sample positions of the section
            // polyline. On a NEAR-TANGENT graze, `clip_branch_to_trims`
            // classifies those samples against the mutual trims, and a sample
            // landing just past the near-tangent boundary is dropped as a
            // false-Outside that TRUNCATES the clipped section. The two operand
            // orders drop DIFFERENT near-tangent tail samples, so one order
            // yields a section ~0.1-0.2mm shorter at ONE endpoint (t660: b\a's
            // step-first order clips pairs 223x105/399x105 ~0.22/0.14mm short of
            // the section a\b's cyl-first order keeps, stranding edge 173
            // one-use). Near-tangent clip errors are almost exclusively
            // false-Outside (a point truly outside a trim rarely projects to an
            // in-trim uv), so the order with the LONGER clipped section suffered
            // fewer drops and is the more complete one. When the swapped order
            // marches the SAME branch structure with a meaningfully longer
            // clipped total, adopt its branches (3D points, so they flow through
            // the identical clip/process_curve gates below). Gated to Candidate
            // pairs (the near-tangent/ambiguous class; clean Transverse
            // crossings clip identically in both orders and never differ) with
            // an accepted GRAZING pierce seed (min |tan·n| < 0.5 — a real
            // section provably crosses, and does so near-tangentially, the only
            // regime where the clip wobbles; this also bounds the extra march to
            // grazing pairs), and requires a >0.1%-of-length margin so
            // raw-sampling jitter (t660: 0.02%) cannot flip a clean pair.
            // Fail-soft: any swapped-march or comparison-clip error keeps the
            // forward result.
            if !branches.is_empty()
                && !seed_points.is_empty()
                && min_seed_tangency < 0.5
                && pair_classification.relation == SurfacePairRelation::Candidate
                && std::env::var("BREP_MARCH_SWAP_CLIP_RESCUE").as_deref() != Ok("0")
            {
                let swapped = intersect_surfaces(
                    second_march,
                    first_march,
                    &SurfaceIntersectionOptions {
                        tolerance: options.tolerance,
                        maximum_step: march_maximum_step(
                            &pair_classification,
                            options.maximum_ssi_step,
                            builder.scale,
                        ),
                        seed_points: seed_points.clone(),
                        ..Default::default()
                    },
                )
                .unwrap_or_default();
                // Only a swap that reproduces the SAME branch count — a
                // refined-endpoint variant of the same section, not a different
                // branch decomposition (guards against adopting a spurious
                // extra branch as "longer").
                if !swapped.is_empty() && swapped.len() == branches.len() {
                    let totals = (|| -> Result<(f64, f64), KernelRefusal> {
                        let mut fwd = 0.0;
                        for b in &branches {
                            let refined = insert_seed_points_into_branch(
                                &b.points,
                                &seed_points,
                                options.tolerance,
                            );
                            for run in clip_branch_to_trims(&refined, first, second)? {
                                fwd += run
                                    .windows(2)
                                    .map(|p| p[1].sub(p[0]).length())
                                    .sum::<f64>();
                            }
                        }
                        let mut swp = 0.0;
                        for b in &swapped {
                            let refined = insert_seed_points_into_branch(
                                &b.points,
                                &seed_points,
                                options.tolerance,
                            );
                            for run in clip_branch_to_trims(&refined, first, second)? {
                                swp += run
                                    .windows(2)
                                    .map(|p| p[1].sub(p[0]).length())
                                    .sum::<f64>();
                            }
                        }
                        Ok((fwd, swp))
                    })();
                    if let Ok((fwd_clip, swp_clip)) = totals {
                        let margin = fwd_clip.max(swp_clip) * 1.0e-3;
                        let adopt = swp_clip > fwd_clip + margin;
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: CLIP-SWAP RESCUE fwd_clip={:.6} swp_clip={:.6} margin={:.6}{}",
                                first.face.id,
                                second.face.id,
                                fwd_clip,
                                swp_clip,
                                margin,
                                if adopt { " ADOPT" } else { "" }
                            );
                        }
                        if adopt {
                            branches = swapped;
                        }
                    }
                }
            }
            if let Some(started) = rescue_started {
                profile.rescue_march += started.elapsed();
            }
            if branches.iter().any(|branch| branch.points.len() >= 2) {
                section_evidence = true;
            }
            if debug_pairs {
                eprintln!(
                    "pair {}x{}: MARCH {} branches, pts {:?}",
                    first.face.id,
                    second.face.id,
                    branches.len(),
                    branches.iter().map(|b| b.points.len()).collect::<Vec<_>>()
                );
            }
            for branch in branches {
                // The pierce seeds are the section's exact trim-crossing
                // points — insert them so no trim interval shorter than the
                // march step is invisible to the point-classification clip.
                let refined_points =
                    insert_seed_points_into_branch(&branch.points, &seed_points, options.tolerance);
                let clip_started = profile.enabled.then(Instant::now);
                let runs = clip_branch_to_trims(&refined_points, first, second)?;
                if debug_pairs {
                    let dups = |points: &[Vec3]| points.windows(2).enumerate().filter(|(_, pair)| pair[1].sub(pair[0]).length() <= options.tolerance).map(|(i, pair)| format!("{i}:{:.2e}", pair[1].sub(pair[0]).length())).collect::<Vec<_>>();
                    eprintln!(
                        "pair {}x{}: DUPS branch {:?} seeded {:?} runs {:?} seeds {} branch_pts {} seeded_pts {}",
                        first.face.id, second.face.id, dups(&branch.points), dups(&refined_points), runs.iter().map(|run| dups(run)).collect::<Vec<_>>(), seed_points.len(), branch.points.len(), refined_points.len()
                    );
                }
                if let Some(started) = clip_started {
                    profile.clip += started.elapsed();
                }
                for run in runs {
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: clip run len_pts={} length={:.4e}",
                            first.face.id,
                            second.face.id,
                            run.len(),
                            run.windows(2)
                                .map(|pair| pair[1].sub(pair[0]).length())
                                .sum::<f64>()
                        );
                    }
                    if run.len() < 2 {
                        continue;
                    }
                    let length: f64 = run
                        .windows(2)
                        .map(|pair| pair[1].sub(pair[0]).length())
                        .sum();
                    if length <= options.tolerance * 100.0 {
                        continue;
                    }
                    let shared_started = profile.enabled.then(Instant::now);
                    let follows_shared = branch_follows_shared_boundary(
                        &run,
                        first,
                        second,
                        &builder.edges,
                        options.tolerance,
                        builder.scale,
                    )?;
                    if let Some(started) = shared_started {
                        profile.shared_boundary += started.elapsed();
                    }
                    if follows_shared {
                        if debug_pairs {
                            eprintln!(
                                "pair {}x{}: run dropped (follows shared boundary)",
                                first.face.id, second.face.id
                            );
                        }
                        continue;
                    }
                    if let Some(census) = pair_census.as_mut() {
                        census.record_stations(&run);
                    }
                    let pieces_before = builder.pieces.len();
                    let chunk_points = options.fit_chunk_points.unwrap_or(run.len()).max(2);
                    let mut start = 0;
                    while start + 1 < run.len() {
                        let end = (start + chunk_points - 1).min(run.len() - 1);
                        let fit_started = profile.enabled.then(Instant::now);
                        let fit_tolerance = options.tolerance.max(1e-7);
                        let (mut section, mut fit, recovered, evening) = even_sliver_ends_measured(
                            run[start..=end].to_vec(),
                            &first.face.surface,
                            &second.face.surface,
                            options.tolerance,
                            fit_tolerance,
                            options.maximum_fit_points,
                            options.local_fit,
                        )
                        .or_refuse(KernelStage::Intersect, "csg.imprint.driver.sliver_end")?;
                        if evening != SliverEvening::NoSliver {
                            if let Ok(path) = std::env::var("BREP_SECTION_SLIVER_CENSUS") {
                                use std::io::Write;
                                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                                    let _ = writeln!(file, "sliver-end census: pair {}x{} {evening:?} stations {}", first.face.id, second.face.id, section.len());
                                }
                            }
                        }
                        // A recovered fit may reproduce every march point yet
                        // still wander off the carriers between sparse stations.
                        // Measure it against both supports before accepting that
                        // recovery, and use the existing bounded SSI refiner.
                        let met_requested = fit.report.met_requested_tolerance();
                        let refine_route = section_refine_route(
                            section_refine_gate(),
                            recovered,
                            || {
                                paired_grazing_crossings
                                    && std::env::var("BREP_GRAZE_SECTION_REFINE").as_deref() != Ok("0")
                            },
                            met_requested,
                        )
                        .or_else(|| {
                            // `BREP_SECTION_CARRIER_MISS=0` restores the fast path
                            // for an attribution A/B only (default on).
                            if std::env::var("BREP_SECTION_CARRIER_MISS").as_deref() == Ok("0") {
                                return None;
                            }
                            let (station_upper, midspan_upper) =
                                fast_path_carrier_readings(&fit, &section, &first.face.surface, &second.face.surface);
                            carrier_miss_route(station_upper, midspan_upper, fit_tolerance)
                        });
                        if let Some((gate, route)) = refine_route {
                            let stations_before = fit.kept.len();
                            let refine_census = std::env::var("BREP_SECTION_REFINE_CENSUS").ok();
                            let (census_head, census_tail) = (section[0], section[section.len() - 1]);
                            let refine_started = refine_census.is_some().then(Instant::now);
                            // The wrapper's own two lines, inlined so the receipt
                            // can be read (behaviour identical to
                            // `refine_section_against_carriers`).
                            let representatives = std::env::var("BREP_SECTION_REPRESENTATIVES").as_deref() != Ok("0");
                            let mut receipt = RefineReceipt::default();
                            let (refined, worst, unrefined, rounds, exit) = refine_section_against_carriers_with(
                                &mut section,
                                fit,
                                &first.face.surface,
                                &second.face.surface,
                                fit_tolerance,
                                options.tolerance,
                                options.local_fit,
                                gate * fit_tolerance,
                                representatives,
                                &mut receipt,
                            ).or_refuse(KernelStage::Intersect, "csg.imprint.driver.midspan")?;
                            let _ = &receipt;
                            // `BREP_SECTION_REFINE_CENSUS=<file>`: one line per section,
                            // appended, so a corpus replay (whose children's stderr is
                            // captured) can be read for trip counts and growth.
                            // The route, whether the standing fit met its request, and
                            // the refiner's own wall time close the line, so a run can
                            // total the UNMET-REQUEST route's cost apart from the
                            // global gate's (which reads every section).
                            if let Some(path) = refine_census {
                                use std::io::Write;
                                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                                    let _ = writeln!(
                                        file,
                                        "section-refine census: pair {}x{} exit {:?} rounds {} stations {} -> {} unrefined {:.3e} worst {:.3e} route {:?} met_requested {} ms {:.3} head ({:.9}, {:.9}, {:.9}) tail ({:.9}, {:.9}, {:.9})",
                                        first.face.id, second.face.id, exit, rounds, stations_before, refined.kept.len(), unrefined, worst,
                                        route, met_requested, refine_started.map_or(0.0, |started| started.elapsed().as_secs_f64() * 1e3),
                                        census_head.x, census_head.y, census_head.z, census_tail.x, census_tail.y, census_tail.z
                                    );
                                }
                            }
                            if debug_pairs && (rounds > 0 || !matches!(exit, MidspanExit::Converged | MidspanExit::BelowGate)) {
                                eprintln!(
                                    "pair {}x{}: mid-span refinement {:?} after {} round(s): {} stations, worst {:.3e}",
                                    first.face.id, second.face.id, exit, rounds, section.len(), worst
                                );
                            }
                            fit = refined;
                        }
                        // `BREP_SECTION_OVERSHOOT_CENSUS=<file>` (debug only; nothing
                        // reads it): EVERY section's final fit, on any route, is read
                        // against both carriers at its kept stations and at its
                        // mid-spans, so a fit that stands further off the carriers
                        // between its stations than at them (fixture 28 t217: 5.07e-4
                        // shipped against a 1.32e-4 worst station) is counted, not
                        // guessed. Projector distances are upper readings; the line
                        // says so by its field names.
                        {
                            if let Ok(path) = std::env::var("BREP_SECTION_OVERSHOOT_CENSUS") {
                                // An unreadable station makes the whole reading NaN
                                // (f64::max would silently drop it).
                                let read = |surface: &NurbsSurface, point: Vec3| {
                                    project_point_to_surface(surface, point).map_or(f64::NAN, |projection| projection.distance)
                                };
                                let mut station_worst = 0.0f64;
                                for point in &fit.kept {
                                    let (a, b) = (read(&first.face.surface, *point), read(&second.face.surface, *point));
                                    station_worst = if a.is_nan() || b.is_nan() || station_worst.is_nan() { f64::NAN } else { station_worst.max(a).max(b) };
                                }
                                let (run_worst, _) = fast_path_carrier_readings(&fit, &section, &first.face.surface, &second.face.surface);
                                let interior_worst = knot_interior_carrier_upper(&fit.curve, &first.face.surface, &second.face.surface);
                                let misses = midspan_misses_only(&fit, &first.face.surface, &second.face.surface).unwrap_or_default();
                                let (worst_span, midspan_worst) = misses
                                    .iter()
                                    .copied()
                                    .enumerate()
                                    .fold((usize::MAX, 0.0f64), |best, (span, miss)| if miss > best.1 { (span, miss) } else { best });
                                let line = format!(
                                    "section-overshoot census: pair {}x{} route {:?} kept {} spans_read {} station_upper {:.3e} midspan_upper {:.3e} worst_span {} fit_tolerance {:.1e} ratio {:.3} run {} run_upper {run_worst:.3e} knot_interior_upper {interior_worst:.3e} head ({:.9}, {:.9}, {:.9}) tail ({:.9}, {:.9}, {:.9})\n",
                                    first.face.id, second.face.id, refine_route.map(|(_, route)| route), fit.kept.len(), misses.len(), station_worst, midspan_worst,
                                    if worst_span == usize::MAX { -1 } else { worst_span as i64 }, fit_tolerance,
                                    midspan_worst / station_worst.max(fit_tolerance),
                                    section.len(),
                                    section[0].x, section[0].y, section[0].z,
                                    section[section.len() - 1].x, section[section.len() - 1].y, section[section.len() - 1].z
                                );
                                use std::io::Write;
                                if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
                                    let _ = file.write_all(line.as_bytes());
                                }
                            }
                        }
                        if let Some(started) = fit_started {
                            profile.fit += started.elapsed();
                        }
                        if let Some(census) = pair_census.as_mut() {
                            census.record_fit(&fit);
                        }
                        let process_started = profile.enabled.then(Instant::now);
                        builder.process_curve(
                            fit.curve,
                            first,
                            second,
                            &[first, second],
                            &[first, second],
                            true,
                        )?;
                        if let Some(started) = process_started {
                            profile.process += started.elapsed();
                        }
                        start = end;
                    }
                    for piece in &builder.pieces[pieces_before..] {
                        builder.marched_pieces.insert(piece.id);
                    }
                    if debug_pairs {
                        eprintln!(
                            "pair {}x{}: run -> {} pieces",
                            first.face.id,
                            second.face.id,
                            builder.pieces.len() - pieces_before
                        );
                    }
                }
            }
            profile.clip_and_fit += profile.lap(&mut lap_start);
        }
    }
    profile.report();
    // SELF-TOUCH SPLIT: a face whose loops touch at an edge interior (a hole
    // tangent to a fillet setback) gets a vertex minted at the touch on BOTH
    // edges, so the fragment arrangement's pinch resolution assembles — see
    // `imprint/self_touch.rs`. Escape hatch: BREP_SELF_TOUCH_SPLIT=0.
    let mut tail_lap: Option<Instant> = None;
    let mut tail_ms = Vec::<(&str, f64)>::new();
    let mut lap = |name: &'static str, started: &mut Option<Instant>, tail_ms: &mut Vec<(&str, f64)>| {
        if profile.enabled {
            let now = Instant::now();
            if let Some(previous) = *started {
                tail_ms.push((name, (now - previous).as_secs_f64() * 1_000.0));
            }
            *started = Some(now);
        }
    };
    lap("pairs", &mut tail_lap, &mut tail_ms);
    if std::env::var("BREP_SELF_TOUCH_SPLIT").as_deref() != Ok("0") {
        builder.split_self_touching_loops(&first_faces, &face_edge_lists, &cached_subcurve)?;
        builder.split_self_touching_loops(&second_faces, &face_edge_lists, &cached_subcurve)?;
    }
    // PIECE-ENDPOINT EXCHANGE post-pass: with every pair's process_curve
    // done, the piece set is final for the ridden-edge class — imprint each
    // open riding piece's junction endpoints onto the ridden edges (see the
    // method doc; hatch BREP_OVERLAP_PIECE_ENDPOINT_SPLIT=0).
    builder.exchange_piece_endpoint_junctions()?;
    lap("self_touch", &mut tail_lap, &mut tail_ms);
    let mut edge_splits = builder
        .edge_splits
        .into_iter()
        .map(|((operand, edge_id), mut parameters)| {
            parameters.sort_by(f64::total_cmp);
            EdgeSplitRecord {
                operand,
                edge_id,
                parameters,
            }
        })
        .collect::<Vec<_>>();
    edge_splits.sort_by_key(|record| (record.operand, record.edge_id));
    let mut by_face = builder
        .by_face
        .into_iter()
        .map(|(face, piece_ids)| FaceImprints {
            operand: face.operand,
            face_id: face.face_id,
            piece_ids,
        })
        .collect::<Vec<_>>();
    by_face.sort_by_key(|record| (record.operand, record.face_id));
    let section_evidence = section_evidence || !builder.pieces.is_empty();
    let marched_pieces = std::mem::take(&mut builder.marched_pieces);
    let mut result = ImprintResultRecord {
        tangent_nodes,
        vertices: builder.vertices,
        pieces: builder.pieces,
        by_face,
        edge_splits,
        barrier_edges: builder.barrier_edges.into_iter().collect(),
        section_evidence,
        cosurface_pairs,
    };
    // COINCIDENT-PIECE MERGE (problemInbox equator-tangent, and the generic
    // one-circle-from-many-pairs class): the SAME section curve can be minted
    // by several pairs — a cosurface boundary-edge copy (the cylinder cap
    // ring lying ON the inscribed sphere) AND the cap-plane's analytic
    // section ring are one circle minted twice, each carrying only its own
    // pair's supports/pcurves. Assembly then builds duplicate edges that
    // cannot both be two-use → one-use strands. Merge pieces whose curves
    // coincide along their whole span (bidirectional max deviation within
    // the weld band): keep the first, union the supports/pcurves/by_face
    // registrations of the rest into it. Escape hatch:
    // BREP_COINCIDENT_PIECE_MERGE=0.
    lap("collect", &mut tail_lap, &mut tail_ms);
    if std::env::var("BREP_COINCIDENT_PIECE_MERGE").as_deref() != Ok("0") {
        let weld = assembler_weld(options.tolerance).max(options.tolerance * 10.0);
        // Two pieces that coincide along their WHOLE spans (deviation within
        // the weld both ways) also have matching end points, up to direction
        // and a weld or two of overhang, so a piece whose ends are nowhere
        // near the other's needs no 33-station projection sweep. The sweep is
        // what made this pass quadratic in wall time: 630 pieces cost 3.2 s
        // of a 3.8 s imprint on the 2026-09-12 mesh-import report, all of it
        // rejecting pairs an end-point look already rules out.
        let mut ends: Vec<[Vec3; 2]> = Vec::with_capacity(result.pieces.len());
        for piece in &result.pieces {
            let [d0, d1] = piece.curve.domain().or_refuse(KernelStage::Intersect, "domain")?;
            ends.push([
                piece.curve.evaluate(d0).or_refuse(KernelStage::Intersect, "evaluate")?,
                piece.curve.evaluate(d1).or_refuse(KernelStage::Intersect, "evaluate")?,
            ]);
        }
        let end_band = 4.0 * weld;
        // A CLOSED piece (a full ring) has no end points to speak of: two
        // rings of the same circle seamed at different azimuths coincide
        // along their whole spans while their domain ends sit anywhere on
        // the ring, so a closed piece always takes the full sweep.
        let ends_match = |a: &[Vec3; 2], b: &[Vec3; 2]| -> bool {
            let near = |p: Vec3, q: Vec3| p.sub(q).length() <= end_band;
            near(a[0], a[1])
                || near(b[0], b[1])
                || (near(a[0], b[0]) && near(a[1], b[1]))
                || (near(a[0], b[1]) && near(a[1], b[0]))
        };
        let mut removed: Vec<u64> = Vec::new();
        let mut index = 0;
        while index < result.pieces.len() {
            let mut other = index + 1;
            while other < result.pieces.len() {
                let coincide = ends_match(&ends[index], &ends[other]) && {
                    let a = &result.pieces[index];
                    let b = &result.pieces[other];
                    max_curve_deviation(&a.curve, &b.curve)? <= weld
                        && max_curve_deviation(&b.curve, &a.curve)? <= weld
                };
                if coincide {
                    let absorbed = result.pieces.remove(other);
                    ends.remove(other);
                    removed.push(absorbed.id);
                    let keeper = &mut result.pieces[index];
                    for pcurve in absorbed.pcurves {
                        if !keeper
                            .pcurves
                            .iter()
                            .any(|existing| {
                                existing.operand == pcurve.operand
                                    && existing.face_id == pcurve.face_id
                            })
                        {
                            keeper.pcurves.push(pcurve);
                        }
                    }
                    let keeper_id = keeper.id;
                    for record in &mut result.by_face {
                        if let Some(position) =
                            record.piece_ids.iter().position(|&id| id == absorbed.id)
                        {
                            if record.piece_ids.contains(&keeper_id) {
                                record.piece_ids.remove(position);
                            } else {
                                record.piece_ids[position] = keeper_id;
                            }
                        }
                    }
                } else {
                    other += 1;
                }
            }
            index += 1;
        }
        if !removed.is_empty() && std::env::var("BREP_DEBUG_BOOL").is_ok() {
            eprintln!("coincident-piece merge: absorbed {:?}", removed);
        }
    }
    // Rescue near-tangent SSI truncations BEFORE canonicalization so the added
    // bridge pieces' endpoints (existing crossing/stub vertices) fold into the
    // same junction merges as every other section.
    lap("coincident_merge", &mut tail_lap, &mut tail_ms);
    extend_truncated_sections(
        &mut result,
        &face_edge_lists,
        solid_a,
        solid_b,
        &charts,
        options.tolerance,
    )?;
    lap("extend_truncated", &mut tail_lap, &mut tail_ms);
    canonicalize_imprint_junctions(&mut result, solid_a, solid_b, &charts, options.tolerance)?;
    lap("canonicalize", &mut tail_lap, &mut tail_ms);
    // ONE RIM, ONE EDGE: with vertex identity final, dissolve the section
    // vertices that no operand edge passes through — the marcher's own
    // parameterization origin, which `process_curve` cannot tell from a
    // junction because it sees one pair at a time. See the method doc.
    let dissolved = dissolve_section_origin_vertices(
        &mut result,
        &marched_pieces,
        solid_a,
        solid_b,
        &charts,
        options.tolerance,
    )?;
    if dissolved > 0 && std::env::var("BREP_DEBUG_BOOL").is_ok() {
        eprintln!("origin-dissolve: {dissolved} parameterization vertex/vertices removed");
    }
    lap("origin_dissolve", &mut tail_lap, &mut tail_ms);
    // B2: reuse an existing boundary edge as the shared section edge wherever a
    // section coincides with one along its whole span (vertices are final after
    // canonicalization; `face_edge_lists` holds each face's boundary edges on
    // the healed operands). Runs here so both operands reference ONE edge.
    reuse_boundary_section_edges(
        &mut result,
        &face_edge_lists,
        solid_a,
        solid_b,
        options.tolerance,
    )?;
    // Capstone step 1 — instrumentation only, zero behavior change: report
    // every (section piece × boundary edge) contact where the piece runs
    // within the scale-derived band of the edge over a real span. Measuring
    // the bands here first validates the graze-contact model on the
    // acceptance suite before step 2's common-block machinery replaces a
    // grazed overlap with a shared edge.
    lap("reuse_boundary", &mut tail_lap, &mut tail_ms);
    report_graze_contacts(&result, &face_edge_lists, solid_a, solid_b, options.tolerance)?;
    if profile.enabled {
        let line = tail_ms
            .iter()
            .map(|(name, ms)| format!("{name}={ms:.2}"))
            .collect::<Vec<_>>()
            .join(" ");
        eprintln!("imprint.stages {line}");
    }
    Ok(result)
}

/// Debug-only graze-contact survey (`BREP_DEBUG_GRAZE=1`): for each section
/// piece and each boundary edge of its support faces, sample the piece and
/// measure distance to the edge; report contacts whose in-band span exceeds
/// both the weld scale and 4× the minimum deviation (span-wise proximity, not
/// a point touch). `band_cap` reuses the residual-merge `sep_cap` ceiling —
/// measured, never grown. The output is the raw material for capstone step 2
/// (partial-span common-block): which contacts exist, their spans, and their
/// measured bands.
fn report_graze_contacts(
    result: &ImprintResultRecord,
    face_edge_lists: &HashMap<FaceKey, Vec<&EdgeRecord>>,
    solid_a: &BrepSolid,
    solid_b: &BrepSolid,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    if std::env::var("BREP_DEBUG_GRAZE").as_deref() != Ok("1") {
        return Ok(());
    }
    let raw_extent = raw_solid_extent(solid_a).max(raw_solid_extent(solid_b));
    let (_, band_cap) = residual_merge_bands(raw_extent, tolerance);
    const SAMPLES: usize = 17;
    for piece in &result.pieces {
        let [t0, t1] = [piece.t0, piece.t1];
        if !(t1 > t0) {
            continue;
        }
        for key in piece.support_faces {
            let Some(edges) = face_edge_lists.get(&key) else {
                continue;
            };
            for edge in edges {
                if edge.degenerate {
                    continue;
                }
                let mut in_band = 0usize;
                let mut min_dev = f64::INFINITY;
                let mut max_dev_in_band = 0.0f64;
                let mut span = 0.0f64;
                let mut prev: Option<(bool, Vec3)> = None;
                for k in 0..SAMPLES {
                    let t = t0 + (t1 - t0) * k as f64 / (SAMPLES - 1) as f64;
                    let point = piece.curve.evaluate(t).or_refuse(KernelStage::Intersect, "evaluate")?;
                    let deviation = project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Intersect, "project_point_to_curve")?.distance;
                    min_dev = min_dev.min(deviation);
                    let inside = deviation <= band_cap;
                    if inside {
                        in_band += 1;
                        max_dev_in_band = max_dev_in_band.max(deviation);
                        if let Some((true, prev_point)) = prev {
                            span += point.sub(prev_point).length();
                        }
                    }
                    prev = Some((inside, point));
                }
                // Span-wise contact: several consecutive samples in band and a
                // span that dwarfs the closest-approach (not a transversal
                // crossing, which dips in and out at one sample).
                if in_band >= 3 && span > (4.0 * min_dev).max(assembler_weld(tolerance)) {
                    eprintln!(
                        "graze: piece {} sup=[{}:{},{}:{}] ~ edge {}:{} span={:.3e} band=[{:.3e},{:.3e}] samples_in_band={}/{}",
                        piece.id,
                        piece.support_faces[0].operand,
                        piece.support_faces[0].face_id,
                        piece.support_faces[1].operand,
                        piece.support_faces[1].face_id,
                        key.operand,
                        edge.id,
                        span,
                        min_dev,
                        max_dev_in_band,
                        in_band,
                        SAMPLES
                    );
                }
            }
        }
    }
    Ok(())
}

/// Are the two carriers TANGENT (normals parallel) where they both pass through
/// `point`?
///
/// The test the tangential-only refusal above needs: a point on the marched
/// intersection is a TANGENT NODE when the two surface normals there are
/// parallel. The threshold is the transversality bound the supplemental
/// detector already accepts seeds by (`TRANSVERSE_SEED_CROSS`), not the
/// far tighter pair-classifier bound — a node the march merely passes CLOSE to
/// still poisons the assembly, so this errs toward calling a pair singular.
fn pair_normals_parallel_at(
    first: TaggedFace<'_>,
    second: TaggedFace<'_>,
    point: Vec3,
) -> Result<bool, KernelRefusal> {
    let mut normals = [Vec3::default(); 2];
    for (slot, face) in normals.iter_mut().zip([first, second]) {
        let projection = project_point_to_surface(&face.face.surface, point)
            .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
        let Ok(normal) = face.face.surface.normal(projection.u, projection.v) else {
            // A pole/singular parameter point cannot witness transversality;
            // treat it as tangential so the pair stays refused.
            return Ok(true);
        };
        *slot = normal;
    }
    Ok(normals[0].cross(normals[1]).length() <= crate::TRANSVERSE_SEED_CROSS)
}
