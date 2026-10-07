//! Error-driven station placement, and the MEASURED bound a station sweep
//! reports (kinematic-sweep plan, item 2).
//!
//! A station sweep places rigid copies of the section at sampled stations and
//! lofts through them; between stations the loft's cubic v-interpolant stands
//! in for the path, and everything it gets wrong is a function of where the
//! stations are. Two instruments here read that:
//!
//! - [`spine_deviations`]: the loft's own interpolant of the STATION POINTS
//!   (the same fit, the same chord parameters) against the true path between
//!   the stations, one number per span. The chain sampler refines the spans
//!   that read over the requested tolerance ([`refine_parameters`]).
//! - [`station_report`]: after the sections are placed, the same interpolants
//!   of the section's centre track and frame axes — which, because the loft's
//!   fit is linear in the data and every column shares one parameter vector,
//!   ARE the built wall between stations — against the true path and the true
//!   transported frame: the worst wall deviation, and the built solid's volume
//!   by quadrature against the kinematic sweep's own volume. The built volume
//!   is the loft's to quadrature precision (`the_station_report_reads_the_
//!   built_volume` pins it against `solid_mass_properties` at 1e-10), so the
//!   reported residual is the Pappus residual itself, not a bound on it.
use crate::mass_properties::{GAUSS_W, GAUSS_X};
use crate::{interpolate_curve, interpolate_curve_closed, NurbsCurve, Vec3};
use crate::{Approximation, KernelRefusal, KernelStage, OrRefuse};

/// The centre-track deviation a station sweep is refined to when its caller
/// names none: 1e-4 mm. Read off the tolerance ladder in the record: the
/// loosest value at which both closed rings the plan named sit under 1e-6
/// relative on their closed forms (the rounded rectangle 4.8e-4 → 1.9e-7 at
/// 211 stations from 81, the filleted skew quad 8.3e-4 → 7.4e-8 at 248 from
/// 92), while the `R = 10` ring keeps its 64 stations and every single-curve
/// fixture the landed geometry pins (a quarter arc at 32 stations reads
/// 1.4e-7, a 350° arc 3.9e-5) stays under it and so bit-identical. One
/// decade tighter costs half again as many stations for a residual already
/// under 1e-7.
pub const SWEEP_DEFAULT_TOLERANCE: f64 = 1e-4;

/// The loft's station cap: its interpolation solve is per control column and
/// a run past this looks like a hang. Refinement stops here and says so.
pub const SWEEP_MAX_STATIONS: usize = 1024;

/// How many refinement rounds the sampler runs before it stops and reports
/// what it reached (`StationBudget::rounds_exhausted`). A round splits an
/// offending span by up to 8, so six rounds cover a span 2.6e5× over under
/// the `h²` law a G1 joint's span obeys.
pub const SWEEP_REFINEMENT_ROUNDS: usize = 6;

/// Where a station sits on the chain: the segment its NEXT span runs on, and
/// its parameter there. A joint station belongs to the segment it departs
/// into; the last station of an open run keeps the segment it arrived on.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationParam {
    pub segment: usize,
    pub t: f64,
}

/// What the sampler did to reach its stations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationBudget {
    /// The stations the floors gave before any refinement.
    pub initial: usize,
    /// The stations lofted through.
    pub stations: usize,
    /// Refinement rounds that inserted at least one station.
    pub rounds: usize,
    /// True when refinement wanted more stations than the cap allows.
    pub at_cap: bool,
    /// True when [`SWEEP_REFINEMENT_ROUNDS`] rounds ran and the last
    /// measurement was still over the tolerance.
    pub rounds_exhausted: bool,
}

/// The measured bound a station sweep carries.
#[derive(Clone, Debug, PartialEq)]
pub struct SweepStationReport {
    pub budget: StationBudget,
    /// The centre-track deviation the run was refined to, mm.
    pub tolerance: f64,
    /// The worst distance from the true path to the loft's centre track, mm —
    /// the number read against `tolerance`, and the one refinement drives:
    /// the cubic's error falls as `h⁴` inside a segment and as `h²` at a G1
    /// joint's curvature jump.
    pub spine_deviation: f64,
    /// The worst deviation of the loft's frame axes from the transported
    /// frame, per unit of section reach. Reported, not driven: at a G1 joint
    /// the frame's rate of turn JUMPS, and a cubic through samples of a kinked
    /// function overshoots by a FIRST-order term in the local spacing (the
    /// rounded-rectangle ring reads 2.4e-3 at its floor budget where its
    /// centre track reads 6.6e-4, and 1.1e-3 against 7.6e-6 refined to 1e-5),
    /// so stations buy it down a decade a decade of track, not four; that is
    /// the fitted route's own limit and the plan's item 3 bar.
    pub frame_deviation: f64,
    /// `spine + reach × frame`: the worst distance any wall point stands off
    /// the kinematic sweep's wall, mm.
    pub wall_deviation: f64,
    /// The section's reach from its anchor, mm (a control-hull bound).
    pub reach: f64,
    /// The built solid's volume, by quadrature over the loft's own interpolants.
    pub built_volume: f64,
    /// The kinematic sweep's volume: `A·L` for a section centred on the path,
    /// with the section's first-moment terms and the carried anchor's own
    /// motion otherwise (Simpson over each span's quarter points).
    pub ideal_volume: f64,
    /// `|built − ideal|`, mm³: the Pappus residual, measured.
    pub volume_residual: f64,
    /// True when `volume_residual` is the crude `lateral area × wall
    /// deviation` bound instead of the quadrature (a seam-shifted ring).
    pub crude_bound: bool,
}

impl SweepStationReport {
    /// The report as the typed approximation a feature result carries
    /// (`sweep.stations`).
    pub fn approximation(&self, body: impl Into<String>) -> Approximation {
        let cap = if self.budget.at_cap {
            format!(
                "; refinement stopped at the {SWEEP_MAX_STATIONS}-station cap, so the run does \
                 not meet the requested tolerance"
            )
        } else if self.budget.rounds_exhausted {
            format!(
                "; refinement stopped after its {SWEEP_REFINEMENT_ROUNDS} rounds still over the \
                 requested tolerance"
            )
        } else {
            String::new()
        };
        let volume = if self.crude_bound {
            format!(
                "the volume is within {:.3e} mm³ of the kinematic sweep's (lateral area × wall \
                 deviation)",
                self.volume_residual
            )
        } else {
            format!(
                "the volume by quadrature is {:.9} against the kinematic sweep's {:.9} \
                 (residual {:.3e} mm³)",
                self.built_volume, self.ideal_volume, self.volume_residual
            )
        };
        Approximation {
            code: "sweep.stations".to_string(),
            body: body.into(),
            measured: self.spine_deviation,
            bar: self.tolerance,
            volume_bound: Some(self.volume_residual),
            edges: Vec::new(),
            budget: None,
            message: format!(
                "the swept wall is lofted through {} stations ({} from the floors, {} refinement \
                 round(s)); the loft's centre track stands within {:.3e} mm of the path against \
                 the requested {:.1e}{cap}; its frame stands within {:.3e} per unit reach of the \
                 transported frame (a reach of {:.3}), so the wall is within {:.3e} mm of the \
                 kinematic sweep's; {volume}",
                self.budget.stations,
                self.budget.initial,
                self.budget.rounds,
                self.spine_deviation,
                self.tolerance,
                self.frame_deviation,
                self.reach,
                self.wall_deviation,
            ),
        }
    }
}

thread_local! {
    /// The bite-proof STUB for this thread: `true` switches refinement off
    /// without touching the process environment, so a lib test can pull it
    /// while the rest of the module runs beside it.
    static REFINEMENT_STUBBED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether refinement is switched off — by the environment
/// (`BREP_SWEEP_STATION_REFINEMENT=0`, mirroring `BREP_SWEEP_BEND_GUARD`, the
/// switch a probe or a whole-module run pulls) or by this thread's stub.
pub(super) fn refinement_enabled() -> bool {
    !REFINEMENT_STUBBED.with(|stub| stub.get())
        && std::env::var("BREP_SWEEP_STATION_REFINEMENT").as_deref() != Ok("0")
}


/// Cumulative chord parameters over `points`, normalized to `[0, 1]`; a
/// CLOSED run has one more entry for the wrap chord back to point 0.
pub(super) fn chord_parameters(points: &[Vec3], closed: bool) -> Vec<f64> {
    let count = points.len();
    let mut parameters = Vec::with_capacity(count + usize::from(closed));
    let mut total = 0.0;
    parameters.push(0.0);
    for index in 1..count {
        total += points[index].sub(points[index - 1]).length();
        parameters.push(total);
    }
    if closed {
        total += points[0].sub(points[count - 1]).length();
        parameters.push(total);
    }
    if total > 0.0 {
        for parameter in parameters.iter_mut() {
            *parameter /= total;
        }
    }
    parameters
}

/// The loft's own v-parameters for a run of placed sections: the average over
/// every control-point column of that column's cumulative chord fraction, the
/// wrap chord included on a ring — `loft_topology/basic.rs` and `closed.rs`,
/// formula for formula, so the interpolants built on them are the loft's.
pub(super) fn loft_parameters(sections: &[Vec<NurbsCurve>], closed: bool) -> Result<Vec<f64>, KernelRefusal> {
    let count = sections.len();
    let spans = count - 1 + usize::from(closed);
    let mut accumulated = vec![0.0; spans + 1];
    let mut columns = 0usize;
    for curve_index in 0..sections[0].len() {
        for control_index in 0..sections[0][curve_index].control_points.len() {
            let mut chords = vec![0.0; spans + 1];
            let mut total = 0.0;
            for station in 1..=spans {
                let previous = sections[station - 1][curve_index].control_points[control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                let current = sections[station % count][curve_index].control_points[control_index]
                    .point()
                    .or_refuse(KernelStage::Refine, "control_point")?;
                total += current.sub(previous).length();
                chords[station] = total;
            }
            if total <= 1e-6 {
                continue;
            }
            for station in 0..=spans {
                accumulated[station] += chords[station] / total;
            }
            columns += 1;
        }
    }
    if columns == 0 {
        return Err(KernelRefusal::input(
            KernelStage::Classify,
            "coincident_sections",
            "sweepSolid: the placed sections coincide",
        ));
    }
    let mut parameters: Vec<f64> = accumulated.iter().map(|sum| sum / columns as f64).collect();
    parameters[0] = 0.0;
    parameters[spans] = 1.0;
    Ok(parameters)
}

/// The loft's v-interpolant through `points` at `parameters`: the open fit at
/// the loft's degree (cubic, or fewer points minus one), or the periodic
/// cubic on a ring (`parameters` then carries the wrap entry).
pub(super) fn interpolant(points: &[Vec3], parameters: &[f64], closed: bool) -> Result<NurbsCurve, String> {
    if closed {
        interpolate_curve_closed(points, parameters)
    } else {
        interpolate_curve(points, 3usize.min(points.len() - 1), parameters)
    }
}

/// The parameter on `curve` nearest `point`, from `guess`: Newton on the
/// distance's derivative, held inside the curve's domain. The spans are
/// short and the guess is the chord fraction, so a few steps converge; a step
/// that does not improve is discarded.
fn nearest_parameter(curve: &NurbsCurve, point: Vec3, guess: f64) -> Result<(f64, f64), String> {
    let [lo, hi] = curve.domain()?;
    let mut u = guess.clamp(lo, hi);
    let mut distance = curve.evaluate(u)?.sub(point).length();
    for _ in 0..6 {
        let derivatives = curve.derivatives_small(u, 2)?;
        let offset = derivatives[0].sub(point);
        let numerator = offset.dot(derivatives[1]);
        let denominator = derivatives[1].dot(derivatives[1]) + offset.dot(derivatives[2]);
        if denominator.abs() <= 1e-300 {
            break;
        }
        let next = (u - numerator / denominator).clamp(lo, hi);
        let next_distance = curve.evaluate(next)?.sub(point).length();
        if next_distance >= distance || (next - u).abs() <= 1e-15 * hi.abs().max(1.0) {
            break;
        }
        u = next;
        distance = next_distance;
    }
    Ok((u, distance))
}

/// The true path piece under span `k`: its segment and parameter interval.
/// A span runs from station `k` to station `k + 1` on station `k`'s departing
/// segment — to that segment's end when the next station is the joint (or
/// station 0 of a ring).
fn span_piece(chain: &[NurbsCurve], params: &[StationParam], k: usize, closed: bool) -> Result<(usize, f64, f64), String> {
    let count = params.len();
    let from = params[k];
    let next = (k + 1) % count;
    let end = if !closed && k + 1 == count {
        return Err("no span past the last station of an open run".into());
    } else if next != 0 && params[next].segment == from.segment {
        params[next].t
    } else {
        chain[from.segment].domain()?[1]
    };
    Ok((from.segment, from.t, end))
}

/// The fractions of a span the instruments sample the true path at.
const SPAN_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];

/// Per span, the worst distance from the true path to the loft's interpolant
/// of the station points (at the centre-chord parameters the sampler has
/// before any section is placed). An open run of `n` stations has `n − 1`
/// spans, a ring `n`.
pub(super) fn spine_deviations(
    chain: &[NurbsCurve],
    params: &[StationParam],
    points: &[Vec3],
    closed: bool,
) -> Result<Vec<f64>, String> {
    let parameters = chord_parameters(points, closed);
    let spine = interpolant(points, &parameters, closed)?;
    let spans = points.len() - 1 + usize::from(closed);
    let mut deviations = Vec::with_capacity(spans);
    for k in 0..spans {
        let (segment, t_a, t_b) = span_piece(chain, params, k, closed)?;
        let (u_a, u_b) = (parameters[k], parameters[k + 1]);
        let mut worst: f64 = 0.0;
        for fraction in SPAN_FRACTIONS {
            let truth = chain[segment].evaluate(t_a + (t_b - t_a) * fraction)?;
            let (_, distance) = nearest_parameter(&spine, truth, u_a + (u_b - u_a) * fraction)?;
            worst = worst.max(distance);
        }
        deviations.push(worst);
    }
    Ok(deviations)
}

/// One refinement round: for every span whose deviation is over `tolerance`,
/// split it into `ceil((deviation / tolerance)^(1/4))` equal parameter steps
/// — the cubic interpolant's `h⁴` law — or `^(1/2)` for a span that starts or
/// ends at a segment joint, where a curvature jump drops the cubic to `h²`
/// (at least 2, at most 8 pieces), and insert the new parameters into that
/// segment's list. Returns how many stations were inserted and whether the
/// round wanted more than the cap allows, in which case the spans are refined
/// worst-first until it is reached and the caller records `at_cap`.
pub(super) fn refine_parameters(
    chain: &[NurbsCurve],
    per_segment: &mut [Vec<f64>],
    params: &[StationParam],
    deviations: &[f64],
    tolerance: f64,
    closed: bool,
    emitted: usize,
) -> Result<(usize, bool), String> {
    let count = params.len();
    let mut wanted: Vec<(usize, usize)> = Vec::new();
    for (k, &deviation) in deviations.iter().enumerate() {
        if deviation > tolerance {
            let (segment, t_a, t_b) = span_piece(chain, params, k, closed)?;
            let [t0, t1] = chain[segment].domain()?;
            let next = (k + 1) % count;
            let at_joint = (t_a == t0 && (k > 0 || closed))
                || t_b == t1 && (next != 0 || closed) && (next == 0 || params[next].segment != segment);
            let exponent = if at_joint { 0.5 } else { 0.25 };
            let pieces = ((deviation / tolerance).powf(exponent).ceil() as usize).clamp(2, 8);
            wanted.push((k, pieces));
        }
    }
    if wanted.is_empty() {
        return Ok((0, false));
    }
    let asked: usize = wanted.iter().map(|(_, pieces)| pieces - 1).sum();
    let room = SWEEP_MAX_STATIONS.saturating_sub(emitted);
    let at_cap = asked > room;
    if at_cap {
        // Worst first, as many as fit; a span that does not fit whole is
        // bisected if one station fits.
        wanted.sort_by(|a, b| deviations[b.0].total_cmp(&deviations[a.0]));
    }
    let mut inserted = 0usize;
    for (k, pieces) in wanted {
        let mut pieces = pieces;
        while pieces > 1 && inserted + pieces - 1 > room {
            pieces -= 1;
        }
        if pieces < 2 {
            continue;
        }
        let (segment, t_a, t_b) = span_piece(chain, params, k, closed)?;
        for step in 1..pieces {
            per_segment[segment].push(t_a + (t_b - t_a) * step as f64 / pieces as f64);
        }
        inserted += pieces - 1;
    }
    for list in per_segment.iter_mut() {
        list.sort_by(|a, b| a.total_cmp(b));
        list.dedup();
    }
    Ok((inserted, at_cap))
}

/// The section's signed area and first moments about its anchor, in the
/// anchor's own `(a, b) = ((x − origin)·pu, (x − origin)·pv)` coordinates, by
/// Gauss quadrature on every knot span of every curve (Green's theorem:
/// `A = ½∮(a·b' − b·a')`, `∫a dA = ½∮a²·b'`, `∫b dA = −½∮b²·a'`). Exact on
/// polynomial curves; a rational arc reads to round-off at eight points a
/// span.
pub(super) fn section_moments(profile: &[NurbsCurve], origin: Vec3, pu: Vec3, pv: Vec3) -> Result<(f64, f64, f64), String> {
    let (mut area, mut moment_a, mut moment_b) = (0.0, 0.0, 0.0);
    for curve in profile {
        for pair in curve.knots.windows(2) {
            if pair[1] <= pair[0] {
                continue;
            }
            let half = 0.5 * (pair[1] - pair[0]);
            let middle = 0.5 * (pair[1] + pair[0]);
            for index in 0..GAUSS_X.len() {
                let derivatives = curve.derivatives_small(middle + half * GAUSS_X[index], 1)?;
                let local = derivatives[0].sub(origin);
                let (a, b) = (local.dot(pu), local.dot(pv));
                let (da, db) = (derivatives[1].dot(pu), derivatives[1].dot(pv));
                let weight = GAUSS_W[index] * half;
                area += weight * 0.5 * (a * db - b * da);
                moment_a += weight * 0.5 * a * a * db;
                moment_b -= weight * 0.5 * b * b * da;
            }
        }
    }
    Ok((area, moment_a, moment_b))
}

/// How far the section reaches from its anchor: the farthest control point
/// (the curves lie in their hulls, so this bounds every section point).
pub(super) fn section_reach(profile: &[NurbsCurve], origin: Vec3) -> f64 {
    profile
        .iter()
        .flat_map(|curve| curve.control_points.iter())
        .filter_map(|point| point.point().ok())
        .map(|point| point.sub(origin).length())
        .fold(0.0, f64::max)
}

/// Rodrigues about a unit axis.
fn rotated(x: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    let (sin, cos) = angle.sin_cos();
    x.scale(cos).add(axis.cross(x).scale(sin)).add(axis.scale(axis.dot(x) * (1.0 - cos)))
}

/// The inputs [`station_report`] reads off a placed run.
pub(super) struct PlacedRun<'a> {
    pub chain: &'a [NurbsCurve],
    pub params: &'a [StationParam],
    pub points: &'a [Vec3],
    pub tangents: &'a [Vec3],
    /// The axes the sections were placed on, twist included.
    pub axes: &'a [(Vec3, Vec3)],
    /// The image of the anchor origin at every station (the centre track).
    pub centres: &'a [Vec3],
    /// The images of `pu` and `pv` — the section's affine axes — per station.
    pub e1: &'a [Vec3],
    pub e2: &'a [Vec3],
    /// `(pu, pv)` in the placement basis `(t, r, s)`: the same coefficients
    /// at every station, which is what makes the carry rigid.
    pub pu_in_frame: [f64; 3],
    pub pv_in_frame: [f64; 3],
    pub closed: bool,
    pub sections: &'a [Vec<NurbsCurve>],
    /// Signed area and first moments of the section about its anchor.
    pub moments: (f64, f64, f64),
    pub reach: f64,
    /// The seam shift of a twisted ring; the quadrature is over one lap only.
    pub shift: usize,
    pub budget: StationBudget,
    pub tolerance: f64,
}

/// One RMF step (Wang et al. 2008), as `sweep.rs::rmf_step` takes it, for the
/// sub-station transport below.
fn rmf_step(r: Vec3, t: Vec3, t_next: Vec3, step: Vec3) -> Option<Vec3> {
    let c1 = step.dot(step);
    let candidate = if c1 <= 1e-18 {
        r
    } else {
        let reflected_r = r.sub(step.scale(2.0 / c1 * step.dot(r)));
        let reflected_t = t.sub(step.scale(2.0 / c1 * step.dot(t)));
        let v2 = t_next.sub(reflected_t);
        let c2 = v2.dot(v2);
        if c2 <= 1e-18 {
            reflected_r
        } else {
            reflected_r.sub(v2.scale(2.0 / c2 * v2.dot(reflected_r)))
        }
    };
    candidate.sub(t_next.scale(candidate.dot(t_next))).normalized().ok()
}

/// Sub-station fractions the report transports the frame through: the span's
/// quarter points, then its end (where the residual roll is read).
const SUB_FRACTIONS: [f64; 4] = [0.25, 0.5, 0.75, 1.0];

/// The measured bound of a placed run — see the module header.
pub(super) fn station_report(run: &PlacedRun<'_>) -> Result<SweepStationReport, KernelRefusal> {
    let refuse = |what: &str, error: String| {
        KernelRefusal::internal(KernelStage::Refine, "station_report", format!("sweepSolid: station report ({what}): {error}"))
    };
    let count = run.points.len();
    let closed = run.closed;
    let spans = count - 1 + usize::from(closed);
    let (area, moment_a, moment_b) = run.moments;
    // The section's anchor rides the path under `Transplant`; under `Rigid` it
    // rides where it was drawn, `offset` from the path in the station-0 frame
    // and carried rigidly — so the centre track is the path plus that offset
    // turned with the frame, and the track's own motion enters the volume.
    let t0 = run.tangents[0];
    let (r0, s0) = run.axes[0];
    let drawn = run.centres[0].sub(run.points[0]);
    let offset = [drawn.dot(t0), drawn.dot(r0), drawn.dot(s0)];
    let carried = offset.iter().any(|component| component.abs() > 1e-12 * run.reach.max(1e-9));
    let centred = moment_a.abs().max(moment_b.abs()) <= 1e-9 * area.abs() * run.reach.max(1e-9);
    let corrections = !centred || carried;

    // --- The loft's interpolants of the centre track and the affine axes, on
    //     the loft's own parameters.
    let parameters = loft_parameters(run.sections, closed)?;
    let centre = interpolant(run.centres, &parameters, closed).map_err(|e| refuse("centre", e))?;
    let axis_1 = interpolant(run.e1, &parameters, closed).map_err(|e| refuse("axis 1", e))?;
    let axis_2 = interpolant(run.e2, &parameters, closed).map_err(|e| refuse("axis 2", e))?;

    // --- The built volume: ∫ det[X_v, X_a, X_b] over the section and one lap,
    //     with X = C(v) + a·E1(v) + b·E2(v), by Gauss on every knot span of the
    //     interpolants (the integrand is a polynomial of degree 8 there).
    let mut built = 0.0;
    let mut lateral_length = 0.0;
    for pair in centre.knots.windows(2) {
        if pair[1] <= pair[0] {
            continue;
        }
        let half = 0.5 * (pair[1] - pair[0]);
        let middle = 0.5 * (pair[1] + pair[0]);
        for index in 0..GAUSS_X.len() {
            let v = middle + half * GAUSS_X[index];
            let c = centre.derivatives_small(v, 1).map_err(|e| refuse("centre", e))?;
            let e1 = axis_1.derivatives_small(v, 1).map_err(|e| refuse("axis 1", e))?;
            let e2 = axis_2.derivatives_small(v, 1).map_err(|e| refuse("axis 2", e))?;
            let normal = e1[0].cross(e2[0]);
            let weight = GAUSS_W[index] * half;
            built += weight * (area * c[1].dot(normal) + moment_a * e1[1].dot(normal) + moment_b * e2[1].dot(normal));
            lateral_length += weight * c[1].length();
        }
    }

    // --- The true path and the transported frame between stations: the
    //     deviations, the path length, and the kinematic sweep's own moment
    //     terms (Simpson over each span's quarter points).
    let [pu_t, pu_r, pu_s] = run.pu_in_frame;
    let [pv_t, pv_r, pv_s] = run.pv_in_frame;
    let axes_of = |t: Vec3, r: Vec3, s: Vec3| -> (Vec3, Vec3) {
        (
            t.scale(pu_t).add(r.scale(pu_r)).add(s.scale(pu_s)),
            t.scale(pv_t).add(r.scale(pv_r)).add(s.scale(pv_s)),
        )
    };
    let mut spine_deviation: f64 = 0.0;
    let mut frame_deviation: f64 = 0.0;
    let mut length = 0.0;
    let mut moment_integral = 0.0;
    for k in 0..spans {
        let (segment, t_a, t_b) = span_piece(run.chain, run.params, k, closed).map_err(|e| refuse("span", e))?;
        let curve = &run.chain[segment];
        let next = (k + 1) % count;
        let (u_a, u_b) = (parameters[k], parameters[k + 1]);
        // Path length over the span, by Gauss in the path parameter.
        let (half, middle) = (0.5 * (t_b - t_a), 0.5 * (t_b + t_a));
        for index in 0..GAUSS_X.len() {
            let derivatives = curve.derivatives_small(middle + half * GAUSS_X[index], 1).map_err(|e| refuse("path", e))?;
            length += GAUSS_W[index] * half * derivatives[1].length();
        }
        // Transport station k's placed frame through the quarter points to
        // station k + 1, read the roll it arrives short of the placed frame
        // there, and lay that roll down linearly in chord over the span.
        let (r_k, s_k) = run.axes[k];
        let mut r = r_k;
        let mut t_prev = run.tangents[k];
        let mut p_prev = run.points[k];
        let mut transported: Vec<(Vec3, Vec3, Vec3, Vec3)> = Vec::with_capacity(SUB_FRACTIONS.len());
        let [d0, d1] = curve.domain().map_err(|e| refuse("domain", e))?;
        for fraction in SUB_FRACTIONS {
            let t = t_a + (t_b - t_a) * fraction;
            let derivatives = curve.derivatives_small(t, 2).map_err(|e| refuse("path", e))?;
            let point = derivatives[0];
            // A stationary parameter (a cusp in the parameterization) still has
            // a direction of travel: the one-sided limit `unit_tangent` reads.
            let tangent = curve.unit_tangent(t, d0, d1).map_err(|e| refuse("tangent", e))?;
            let kappa_n = super::sweep::curvature_vector(derivatives[1], derivatives[2]);
            r = rmf_step(r, t_prev, tangent, point.sub(p_prev)).ok_or_else(|| refuse("transport", "the frame degenerated".into()))?;
            transported.push((point, tangent, kappa_n, r));
            t_prev = tangent;
            p_prev = point;
        }
        let (r_end, _) = run.axes[next];
        let t_end = run.tangents[next];
        let r_arrived = transported[SUB_FRACTIONS.len() - 1].3;
        let roll = r_arrived.cross(r_end).dot(t_end).atan2(r_arrived.dot(r_end));
        let chord: f64 = {
            let mut sum = 0.0;
            let mut prev = run.points[k];
            for entry in &transported {
                sum += entry.0.sub(prev).length();
                prev = entry.0;
            }
            sum
        };
        let _ = s_k;
        // Simpson over the five points of the span (station k, the quarter
        // points, station k + 1) for the moment terms.
        let mut integrand: Vec<(f64, f64)> = Vec::with_capacity(5);
        let mut chord_so_far = 0.0;
        let mut prev = run.points[k];
        for (which, fraction) in std::iter::once(0.0).chain(SUB_FRACTIONS).enumerate() {
            let (point, tangent, kappa_n, r_raw) = if which == 0 {
                let derivatives = curve.derivatives_small(t_a, 2).map_err(|e| refuse("path", e))?;
                (run.points[k], run.tangents[k], super::sweep::curvature_vector(derivatives[1], derivatives[2]), r_k)
            } else {
                transported[which - 1]
            };
            chord_so_far += point.sub(prev).length();
            prev = point;
            let phi = if chord > 0.0 { roll * chord_so_far / chord } else { 0.0 };
            let r_true = if which == 0 { r_k } else { rotated(r_raw, tangent, phi) };
            let s_true = tangent.cross(r_true);
            let (e1_true, e2_true) = axes_of(tangent, r_true, s_true);
            let centre_true = point
                .add(tangent.scale(offset[0]))
                .add(r_true.scale(offset[1]))
                .add(s_true.scale(offset[2]));
            let speed = curve.derivatives_small(t_a + (t_b - t_a) * fraction, 1).map_err(|e| refuse("path", e))?[1].length();
            if corrections {
                // d/ds of the carried axes: T' = κN, and the RMF's r' = −κ(r·N)T
                // plus the roll laid down, ω = roll / chord, about T. The
                // centre's own motion beyond T is the offset turned with them.
                let omega = if chord > 0.0 { roll / chord } else { 0.0 };
                let t_dot = kappa_n;
                let r_dot = tangent.scale(-kappa_n.dot(r_true)).add(s_true.scale(omega));
                let s_dot = tangent.scale(-kappa_n.dot(s_true)).sub(r_true.scale(omega));
                let e1_dot = t_dot.scale(pu_t).add(r_dot.scale(pu_r)).add(s_dot.scale(pu_s));
                let e2_dot = t_dot.scale(pv_t).add(r_dot.scale(pv_r)).add(s_dot.scale(pv_s));
                let c_dot = t_dot.scale(offset[0]).add(r_dot.scale(offset[1])).add(s_dot.scale(offset[2]));
                let normal = e1_true.cross(e2_true);
                integrand.push((
                    fraction,
                    (area * c_dot.dot(normal) + moment_a * e1_dot.dot(normal) + moment_b * e2_dot.dot(normal)) * speed,
                ));
            }
            if which == 0 || which == SUB_FRACTIONS.len() {
                continue;
            }
            // The deviations, at the interior quarter points.
            let guess = u_a + (u_b - u_a) * fraction;
            let (u, distance) = nearest_parameter(&centre, centre_true, guess).map_err(|e| refuse("projection", e))?;
            spine_deviation = spine_deviation.max(distance);
            let e1_built = axis_1.evaluate(u).map_err(|e| refuse("axis 1", e))?;
            let e2_built = axis_2.evaluate(u).map_err(|e| refuse("axis 2", e))?;
            let frame = (e1_built.sub(e1_true).length_squared() + e2_built.sub(e2_true).length_squared()).sqrt();
            frame_deviation = frame_deviation.max(frame);
        }
        if corrections {
            // Composite Simpson on four equal steps of the path parameter.
            let h = (t_b - t_a) / 4.0;
            let weights = [1.0, 4.0, 2.0, 4.0, 1.0];
            moment_integral += integrand.iter().zip(weights).map(|((_, g), w)| g * w).sum::<f64>() * h / 3.0;
        }
    }
    let (e1_0, e2_0) = (run.e1[0], run.e2[0]);
    let advance = run.tangents[0].dot(e1_0.cross(e2_0));
    let ideal = area * length * advance + moment_integral;
    let wall_deviation = spine_deviation + run.reach * frame_deviation;
    let (built_volume, ideal_volume, residual, crude) = if run.shift == 0 {
        (built.abs(), ideal.abs(), (built.abs() - ideal.abs()).abs(), false)
    } else {
        // A seam-shifted ring's columns run several laps; the one-lap
        // quadrature does not describe it. Bound it instead.
        let perimeter = section_perimeter(run.sections.first().map(|s| s.as_slice()).unwrap_or(&[]))?;
        (f64::NAN, ideal.abs(), perimeter * lateral_length * wall_deviation, true)
    };
    Ok(SweepStationReport {
        budget: run.budget,
        tolerance: run.tolerance,
        spine_deviation,
        frame_deviation,
        wall_deviation,
        reach: run.reach,
        built_volume,
        ideal_volume,
        volume_residual: residual,
        crude_bound: crude,
    })
}

/// A section's perimeter by chord sums (64 per curve) — only the crude bound
/// reads it.
fn section_perimeter(section: &[NurbsCurve]) -> Result<f64, KernelRefusal> {
    let mut total = 0.0;
    for curve in section {
        let [start, end] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let mut previous = curve.evaluate(start).or_refuse(KernelStage::Refine, "evaluate")?;
        for index in 1..=64 {
            let point = curve.evaluate(start + (end - start) * index as f64 / 64.0).or_refuse(KernelStage::Refine, "evaluate")?;
            total += point.sub(previous).length();
            previous = point;
        }
    }
    Ok(total)
}
