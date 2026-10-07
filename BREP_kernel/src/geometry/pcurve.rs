use crate::{
    interpolate_curve, project_point_to_surface, project_point_to_surface_seeded, AnalyticSurface,
    DiagnosticSeverity, KernelDiagnostics, KernelStage, KnotVector, NurbsCurve, NurbsSurface, Vec3,
    Vec4, KNOT_IDENTITY_TOL,
};

const EPSILON: f64 = 1e-12;
const LINEAR_TOLERANCE: f64 = 1e-7;

fn surface_domains(surface: &NurbsSurface) -> Result<([f64; 2], [f64; 2]), String> {
    Ok((
        KnotVector::new(surface.knots_u.clone(), surface.degree_u)?.domain(),
        KnotVector::new(surface.knots_v.clone(), surface.degree_v)?.domain(),
    ))
}

/// The carrier's closed directions, as [`NurbsSurface::closed_directions`]
/// reads them (cached on the surface, seam tolerance 1e-6, and a sliver patch
/// whose rows coincide without the surface travelling between them is OPEN).
/// Until 2026-09-26 this was a second reading with its own tolerance; the
/// unwrap in [`build_interpolant`] and the wrap in `evaluate_extended` must
/// agree on what a seam is, or a trim across a sliver reads a period wide.
fn surface_closedness(surface: &NurbsSurface) -> Result<(bool, bool), String> {
    surface.closed_directions()
}

fn invert_checked(surface: &NurbsSurface, point: Vec3) -> Result<([f64; 2], f64), String> {
    if surface.is_affine()? {
        let ([u0, _], [v0, _]) = surface_domains(surface)?;
        let (origin, du, dv) = surface.deriv1(u0, v0)?;
        let delta = point.sub(origin);
        let uu = du.dot(du);
        let uv = du.dot(dv);
        let vv = dv.dot(dv);
        let along_u = delta.dot(du);
        let along_v = delta.dot(dv);
        let determinant = uu * vv - uv * uv;
        if determinant.abs() > EPSILON {
            let parameter = [
                u0 + (along_u * vv - along_v * uv) / determinant,
                v0 + (along_v * uu - along_u * uv) / determinant,
            ];
            // The standoff is a MEASUREMENT here as on every other carrier:
            // until 2026-09-26 this returned 0.0 for "affine, not measured",
            // and a curve 1e-2 off its plane read as lying on it.
            let foot = origin.add(du.scale(parameter[0] - u0)).add(dv.scale(parameter[1] - v0));
            return Ok((parameter, point.sub(foot).length()));
        }
    }
    let projection = project_point_to_surface(surface, point)?;
    Ok(([projection.u, projection.v], projection.distance))
}

/// Whether an end station at a cone's apex takes its nearest regular
/// station's u (`true`), as in `build_interpolant`, or the projector's
/// arbitrary u there as before 2026-09-27. `BREP_PCURVE_CONE_APEX_U=0`
/// restores the projector's u, for a same-binary A/B.
fn cone_apex_u() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PCURVE_CONE_APEX_U").is_none_or(|value| value != "0"))
}

/// Whether a round with more candidates than the cap has room for keeps its
/// WORST probes (`true`), or the first in span order as before 2026-09-27.
/// `BREP_PCURVE_WORST_FIRST=0` restores span order, for a same-binary A/B.
fn worst_first() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PCURVE_WORST_FIRST").is_none_or(|value| value != "0"))
}

/// `BREP_PCURVE_C0_JOIN=0` builds one C2 interpolant through the stations as
/// before 2026-10-03 (the C0 joins at the curve's corners off), for a
/// same-binary A/B. Read once per process.
fn c0_joins() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PCURVE_C0_JOIN").is_none_or(|value| value != "0"))
}

fn seeded_inserts() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PCURVE_SEEDED_INSERTS").is_none_or(|value| value != "0"))
}

/// [`invert_checked`] for a station inserted between two whose feet are known:
/// Newton from the pcurve's own `(u, v)` at that fraction, which already lies
/// between the neighbours' feet on their branch, and the global search only
/// when that local foot stands off further than the neighbours do (plus the
/// refinement floor) — a local minimum the curve does not sit at. The global
/// search was 30 s of a 54 s `abc_00000011` import (732k calls, 41 µs each,
/// measured 2026-09-26). Planes and analytic carriers keep their closed forms.
/// `BREP_PCURVE_SEEDED_INSERTS=0` restores the global search at every insert,
/// for a same-binary A/B.
fn invert_near(
    surface: &NurbsSurface,
    point: Vec3,
    seed: [f64; 2],
    neighbour_gap: f64,
) -> Result<([f64; 2], f64), String> {
    if !seeded_inserts() || surface.is_affine()? || surface.analytic().is_some() {
        return invert_checked(surface, point);
    }
    let seeded = project_point_to_surface_seeded(surface, point, seed[0], seed[1])?;
    if seeded.distance <= neighbour_gap + PCURVE_REFINEMENT_TOLERANCE {
        return Ok(([seeded.u, seeded.v], seeded.distance));
    }
    invert_checked(surface, point)
}

fn unwrap_periodic(values: &mut [f64], minimum: f64, maximum: f64) {
    let period = maximum - minimum;
    for index in 1..values.len() {
        while values[index] - values[index - 1] > period / 2.0 {
            values[index] -= period;
        }
        while values[index] - values[index - 1] < -period / 2.0 {
            values[index] += period;
        }
    }
    if values.len() > 1 {
        while values[0] - values[1] > period / 2.0 {
            values[0] -= period;
        }
        while values[0] - values[1] < -period / 2.0 {
            values[0] += period;
        }
    }
    // Recenter the whole (now continuous) sequence into the domain by the
    // whole-period shift that leaves the LEAST parameter outside [minimum,
    // maximum]. A single middle/mean sample is NOT representative of a curve
    // that bulges out to touch — or straddle — the periodic seam: when that
    // sample is a boundary-kissing point (the at-seam projection unwraps a hair
    // past the boundary), a single-sample test misfires and shoves the entire
    // curve a full period out of range (ABC helmet 00000011/12 side channels:
    // a touching edge's parameter run snapped a whole period off its true
    // interior, tearing the loop open in parameter space).
    if values.len() > 1 {
        let excursion = |shift: f64| -> f64 {
            values
                .iter()
                .map(|value| {
                    let v = value + shift;
                    (minimum - v).max(0.0) + (v - maximum).max(0.0)
                })
                .sum::<f64>()
        };
        let mean = values.iter().copied().sum::<f64>() / values.len() as f64;
        let center = 0.5 * (minimum + maximum);
        let base_k = ((center - mean) / period).round() as i64;
        let mut best_shift = 0.0;
        let mut best_excursion = f64::INFINITY;
        for k in (base_k - 1)..=(base_k + 1) {
            let shift = k as f64 * period;
            let value = excursion(shift);
            if value < best_excursion {
                best_excursion = value;
                best_shift = shift;
            }
        }
        if best_shift != 0.0 {
            for value in values.iter_mut() {
                *value += best_shift;
            }
        }
    }
}

/// The interpolant through `raw_parameters`, and what building it CLAMPED:
/// the largest parameter distance a sample was pulled back into an open
/// direction's domain, and the 3D distance between the extended surface at
/// the raw parameter and the clamped foot. Both zero when nothing was pulled.
/// A closed direction wraps and is never clamped.
fn build_interpolant(
    surface: &NurbsSurface,
    raw_parameters: &[[f64; 2]],
    curve_parameters: &[f64],
) -> Result<(NurbsCurve, f64, f64), String> {
    build_interpolant_joined(surface, raw_parameters, curve_parameters, &[])
}

/// [`build_interpolant`] with C0 JOINS: `joins` are values of
/// `curve_parameters` at which the edge curve itself has a corner (a knot of
/// multiplicity ≥ its degree, `range_breaks`). One cubic through all the
/// stations is C2 everywhere, so across a corner it overshoots on both sides
/// by a first-order amount however many stations are inserted: `abc_00000011`
/// face #585 edge 179 (79 such knots on a 241-point curve) read 2.055e-4 off
/// at 153 stations with the file's curve 2.1e-7 off the carrier (measured
/// 2026-10-03). The stations have been planted at those knots since
/// 2026-09-26; what was missing was the interpolant honouring them. Each
/// piece between joins is interpolated on its own and the pieces are
/// concatenated at knots of multiplicity 3 — the join `blend/track_fit.rs`
/// makes for a blend track. A piece shorter than four stations cannot carry
/// a cubic, so its join is dropped and it merges with its neighbour.
fn build_interpolant_joined(
    surface: &NurbsSurface,
    raw_parameters: &[[f64; 2]],
    curve_parameters: &[f64],
    joins: &[f64],
) -> Result<(NurbsCurve, f64, f64), String> {
    let ([u0, u1], [v0, v1]) = surface_domains(surface)?;
    let (closed_u, closed_v) = surface_closedness(surface)?;
    let mut parameters = raw_parameters.to_vec();
    // A cone's apex has no longitude either: every u there is the same point,
    // and the projector answers one arbitrarily (0, which unwraps to 1). A trim
    // that ENDS at the apex then turns sideways in its last span —
    // `abc 00008080` face #163 edge 476, a generatrix at u = 0.80255 whose
    // apex station read u = 1.0 (2026-09-27). An apex station at either end of
    // the run takes the u of its nearest regular station, so a generatrix's
    // pcurve is the straight u = const it is. Interior apex crossings are left
    // to the branch logic below. `BREP_PCURVE_CONE_APEX_U=0` switches this off,
    // for a same-binary A/B.
    if cone_apex_u() && matches!(surface.analytic(), Some(AnalyticSurface::RuledRevolution { .. })) && parameters.len() >= 2 {
        let speeds = parameters
            .iter()
            .map(|parameter| surface.deriv1(parameter[0], parameter[1]).map(|(_, su, _)| su.length()))
            .collect::<Result<Vec<_>, _>>()?;
        let reference = speeds.iter().copied().fold(0.0_f64, f64::max);
        let singular = |index: usize| speeds[index] <= reference * 1e-2;
        if let (Some(first), Some(last)) = (
            (0..parameters.len()).find(|&index| !singular(index)),
            (0..parameters.len()).rfind(|&index| !singular(index)),
        ) {
            for index in 0..first {
                parameters[index][0] = parameters[first][0];
            }
            for index in last + 1..parameters.len() {
                parameters[index][0] = parameters[last][0];
            }
        }
    }
    // Longitude is undefined at a sphere pole: the surface's u tangent
    // vanishes there, and closest-point inversion is free to return any u.
    // Letting that arbitrary value participate in periodic unwrapping can move
    // the first real meridian by a whole period (e.g. 0.75 -> -0.25). Analytic
    // carriers are subsequently clamped, collapsing that meridian onto u=0.
    //
    // Keep this deliberately sphere- and singularity-gated. Split the u values
    // into maximal non-pole runs, unwrap each run independently, and leave the
    // pole samples untouched. The existing interpolant degree stays unchanged;
    // lowering it here would also change endpoint-tangent decisions made later
    // while stitching the face loop.
    let sphere_u_singular = if matches!(surface.analytic(), Some(AnalyticSurface::Sphere { .. })) {
        let speeds = parameters
            .iter()
            .map(|parameter| {
                surface
                    .deriv1(parameter[0], parameter[1])
                    .map(|(_, su, _)| su.length())
            })
            .collect::<Result<Vec<_>, _>>()?;
        let reference = speeds.iter().copied().fold(0.0_f64, f64::max);
        speeds
            .into_iter()
            // Uniform edge stations need not hit the pole exactly (ABC 5360
            // bottoms out at v=5.5e-4). In this small conditioning band,
            // longitude is already numerically arbitrary and must not connect
            // the two meridian runs across the pole.
            .map(|speed| speed <= reference * 1e-2)
            .collect::<Vec<_>>()
    } else {
        vec![false; parameters.len()]
    };
    // Do not perturb ordinary pole-touching trims. The special path is needed
    // only when a pole band is bracketed by nonsingular samples and its raw
    // longitude changes by at least half a period: a true through-pole branch
    // reset whose two meridian runs must be unwrapped independently.
    // Prefix/suffix pole samples on normal sphere caps retain the legacy
    // interpolation byte-for-byte. Include the exactly-half-period case so
    // floating-point noise cannot decide which entire run is lifted (ABC 5603).
    let has_interior_pole_crossing = sphere_u_singular
        .iter()
        .position(|singular| !singular)
        .zip(sphere_u_singular.iter().rposition(|singular| !singular))
        .is_some_and(|(first, last)| sphere_u_singular[first..=last].iter().any(|value| *value));
    let period = (u1 - u0).abs();
    let interior_pole_branch_reset = closed_u
        && has_interior_pole_crossing
        && parameters.windows(2).zip(sphere_u_singular.windows(2)).any(
            |(parameter_pair, singular_pair)| {
                (singular_pair[0] || singular_pair[1])
                    && (parameter_pair[1][0] - parameter_pair[0][0]).abs()
                        >= 0.5 * period - 1e-12 * period.max(1.0)
            },
        );
    if closed_u {
        let mut values = parameters.iter().map(|value| value[0]).collect::<Vec<_>>();
        if interior_pole_branch_reset {
            let mut start = 0;
            while start < values.len() {
                while start < values.len() && sphere_u_singular[start] {
                    start += 1;
                }
                let mut end = start;
                while end < values.len() && !sphere_u_singular[end] {
                    end += 1;
                }
                unwrap_periodic(&mut values[start..end], u0, u1);
                start = end;
            }
        } else {
            unwrap_periodic(&mut values, u0, u1);
        }
        for (parameter, value) in parameters.iter_mut().zip(values) {
            parameter[0] = value;
        }
    }
    if closed_v {
        let mut values = parameters.iter().map(|value| value[1]).collect::<Vec<_>>();
        unwrap_periodic(&mut values, v0, v1);
        for (parameter, value) in parameters.iter_mut().zip(values) {
            parameter[1] = value;
        }
    }
    // A closed direction WRAPS: an edge that straddles the seam has no single
    // in-domain parameter run, so keep the unwrapped (possibly slightly
    // out-of-domain) values and let the periodic evaluator wrap them — clamping
    // them onto the seam boundary would collapse the straddling geometry (and,
    // via the interpolant, spike neighbouring stations). This applies to
    // analytic carriers too: a refitted cylinder can have stepped rims that
    // cross its seam. evaluate_extended wraps every closed direction.
    // Open directions must stay in-domain; their extension is a ruling.
    //
    // The clamp is MEASURED, not silent: a sample pulled back into the domain
    // is reported as the parameter distance it moved and the 3D distance
    // between the extended surface at its raw parameter and the clamped foot
    // ([`PcurveFitReport::clamped_excursion`]). Until 2026-09-30 a trim that
    // ran past its chart came back folded onto the chart's boundary with
    // nothing in the report to say so; on the rib-base document that was
    // 3.03e-2 of volume on a body `validate()` passes.
    let mut excursion = 0.0_f64;
    let mut distance = 0.0_f64;
    let mut points = Vec::with_capacity(parameters.len());
    for parameter in parameters {
        let u = if closed_u {
            parameter[0]
        } else {
            parameter[0].clamp(u0, u1)
        };
        let v = if closed_v {
            parameter[1]
        } else {
            parameter[1].clamp(v0, v1)
        };
        let pulled = ((parameter[0] - u).powi(2) + (parameter[1] - v).powi(2)).sqrt();
        if pulled > 0.0 {
            excursion = excursion.max(pulled);
            let extended = surface.evaluate_extended(parameter[0], parameter[1])?;
            distance = distance.max(extended.sub(surface.evaluate(u, v)?).length());
        }
        points.push(Vec3::new(u, v, 0.0));
    }
    let degree = 3usize.min(points.len() - 1);
    let mut bounds: Vec<usize> = joins
        .iter()
        .filter_map(|join| curve_parameters.iter().position(|parameter| parameter == join))
        .filter(|&index| index > 0 && index + 1 < curve_parameters.len())
        .collect();
    bounds.sort_unstable();
    bounds.dedup();
    if degree < 3 || bounds.is_empty() || !c0_joins() {
        let curve = interpolate_curve(&points, degree, curve_parameters)?;
        return Ok((curve, excursion, distance));
    }
    // Every piece needs four stations for a cubic: drop the join that
    // closes a shorter piece, from the left, until all pieces carry one.
    let mut pieces = vec![0usize];
    for bound in bounds {
        if bound + 1 - pieces[pieces.len() - 1] >= 4 {
            pieces.push(bound);
        }
    }
    if curve_parameters.len() - pieces[pieces.len() - 1] < 4 {
        pieces.pop();
    }
    pieces.push(curve_parameters.len() - 1);
    if pieces.len() < 3 {
        let curve = interpolate_curve(&points, degree, curve_parameters)?;
        return Ok((curve, excursion, distance));
    }
    let mut knots: Vec<f64> = Vec::with_capacity(curve_parameters.len() + 4);
    let mut control: Vec<Vec4> = Vec::with_capacity(curve_parameters.len());
    let last_piece = pieces.len() - 2;
    for piece in 0..=last_piece {
        let (from, to) = (pieces[piece], pieces[piece + 1]);
        let fitted = interpolate_curve(&points[from..=to], 3, &curve_parameters[from..=to])?;
        if piece == 0 {
            knots.extend(fitted.knots[..4].iter().copied());
        }
        knots.extend(fitted.knots[4..fitted.knots.len() - 4].iter().copied());
        let multiplicity = if piece == last_piece { 4 } else { 3 };
        knots.extend(std::iter::repeat(curve_parameters[to]).take(multiplicity));
        control.extend(fitted.control_points.iter().skip(usize::from(piece > 0)).copied());
    }
    let curve = NurbsCurve::new(3, knots, control)?;
    Ok((curve, excursion, distance))
}

/// The 3D residual a pcurve's image must reach before `build_pcurve_on_surface`
/// accepts it, and the sample ceiling it refines up to.
///
/// A trimmed face's boundary is only ever as accurate as these two numbers, and
/// every mass property is an integral over that boundary — so this is the floor
/// under area, volume and centroid for every curved trim in the kernel.
///
/// These were `min(1e-3, 1e-4 * scale)` and 160, where `scale` is
/// `1 + ||curve midpoint||` — the DISTANCE FROM THE WORLD ORIGIN of the curve's
/// midpoint, not the part's size. That expression pinned to an absolute 1e-3
/// once the midpoint sat more than about nine units from the origin, so on an
/// ordinary part the bar was 1e-3 and tracked nothing.
///
/// Measured on `offset-shell-box-bore-dished-face`, whose dished wall is a
/// closed-form identity at 2468.431782123 (three independent routes agree). The
/// floor is one of TWO errors there; the other is the refitted sphere carrier
/// that `sphere_offset_surface` now builds in closed form, and before both moved
/// they partly CANCELLED — which is why either alone reads as a regression:
///
/// ```text
/// floor     Box_O.S2            abs err     relative   gate total  worst case
/// 1e-3   2468.442270355      +1.049e-02     4.25e-06        (old)
/// 1e-6   2468.431785429      +3.306e-06     1.34e-09       181.4 s     31.2 s
/// 1e-7   2468.431782170      +4.746e-08     1.92e-11       218.5 s     32.1 s
/// 1e-9   2468.431782103      -2.037e-08    -8.25e-12       305.5 s     46.8 s
/// ```
///
/// 1e-7 is chosen: 70x the accuracy of 1e-6 for +20% gate time, where 1e-9 costs
/// a further +40% for a 2.3x residual no oracle in this corpus could resolve.
/// All four cells are one session on one binary, so the times are comparable to
/// each other. Every case carrying a closed-form oracle moves TOWARD it and none
/// away (`PushFaceTest` 339x, `PushFaceTest2` 275x); no case changes class at
/// any floor.
///
/// A flat ABSOLUTE is what was measured, not what was designed. This codebase
/// otherwise keys identity bands to part size (`KernelTolerances::heal_band`,
/// the `1e-11*(1+scale)` blend family), and on a 3000 mm part 1e-7 is 3e-11
/// relative — refinement will exhaust `MAX_PCURVE_SAMPLES` and return whatever
/// it reached. Until 2026-09-12 it did so SILENTLY; now the fit's
/// [`PcurveFitReport`] carries the residual actually achieved and a
/// [`PcurveFitExit::SampleCeiling`] exit, and the owning operation's
/// diagnostics count it (`pcurve.unmet_floor`). That is far better than the
/// 1e-3 it replaces but it is not a size-aware bar; deriving one from the
/// surface's own extent is separate, unmeasured work. Being absolute does remove this gate's dependence on
/// distance from the origin as a side effect — a real translation-VARIANCE
/// defect, the same anti-pattern `tolerance::merge_scale` records being removed
/// elsewhere. `drop_tolerance` and `endpoint_tolerance` below still carry it.
pub const PCURVE_REFINEMENT_TOLERANCE: f64 = 1e-7;

/// Sample ceiling for a fit's FIRST refinement pass. 160 could not reach
/// [`PCURVE_REFINEMENT_TOLERANCE`] on a curved trim of any length; 600 is the
/// value every cell of the table above was measured at.
///
/// It BINDS. On `boolean_fuzz_corpus/25_cube_pierce_solid` the fitter stopped
/// here 1.65e-3 of volume short of the converged answer (2000 and 4000 samples
/// agree exactly). Raising it globally to 2000 was costed and rejected: +23%
/// case-gate time, concentrated in three offset/fillet cases, and one case
/// (`inbox-20260910-offset-shell-collapsed-fillet`) driven 26x outside its
/// volume band. So it is not raised here: a fit that stops at this ceiling
/// with a residual still worse than its edge curve's own distance from the
/// surface re-enters refinement with [`PCURVE_RAISED_SAMPLES`]
/// ([`refine_pcurve`]); one whose residual is already that gap stops here and
/// says so ([`PcurveFitReport::off_surface`]), because no sample count can
/// close it.
const MAX_PCURVE_SAMPLES: usize = 600;

/// Sample ceiling for the ONE raised pass a fit may take past
/// [`MAX_PCURVE_SAMPLES`]. 2000 is the ceiling the convergence sweep measured
/// on `25_cube_pierce_solid`: its 2000 and 4000 cells agree to every printed
/// digit because refinement there stops itself near 1540 samples, so the cap
/// is not what binds.
const PCURVE_RAISED_SAMPLES: usize = 2000;

/// What a pcurve fit ACHIEVED, returned beside the curve so a ceiling exit can
/// never be mistaken for a met floor.
///
/// [`fit_pcurve_on_surface`] asks for [`PCURVE_REFINEMENT_TOLERANCE`] and stops
/// at [`MAX_PCURVE_SAMPLES`] — or, where that ceiling bound and the floor is
/// reachable, at [`PCURVE_RAISED_SAMPLES`]. Until 2026-09-12 a trim that ran
/// out of samples came back exactly like one that met the floor; on
/// `boolean_fuzz_corpus/25_cube_pierce_solid` that silence hid a 1.65e-3 volume
/// error. Now every fit says which it was: `residual` is measured on the curve
/// that is actually returned and `exit` names why refinement stopped.
///
/// This is a MEASUREMENT taken at a construction, not a band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PcurveFitReport {
    /// Largest 3D distance between the returned pcurve's image on the surface
    /// and the edge curve. On a [`PcurveFitExit::Converged`] exit whose final
    /// round probed every span this is that round's maximum — the refinement's
    /// own acceptance test. On every other exit, including a converged round
    /// that skipped spans narrower than 1e-3 of the parameter range, it is a
    /// sweep over EVERY span of the returned curve.
    pub residual: f64,
    /// The residual the refinement was asked to reach.
    pub floor: f64,
    /// Interpolation samples in the returned curve.
    pub samples: usize,
    /// Why refinement stopped.
    pub exit: PcurveFitExit,
    /// Largest distance from the edge curve to the surface at every station
    /// the fit inverted — the part of `residual` no pcurve can remove, since a
    /// pcurve's image lies ON the surface. Zero for an affine carrier. A value
    /// above `floor` says the floor is unreachable for this input; a ceiling
    /// exit is retried only while `residual` exceeds it by more than the floor,
    /// i.e. while the returned curve is worse than its input.
    pub off_surface: f64,
    /// Whether the fit re-entered refinement with [`PCURVE_RAISED_SAMPLES`]
    /// after [`MAX_PCURVE_SAMPLES`] bound with the floor unmet.
    pub raised: bool,
    /// Largest parameter distance a station's inversion was pulled back into
    /// an OPEN direction's domain when the returned curve was interpolated: a
    /// trim asked for outside its chart is folded onto the chart's boundary,
    /// the edge stays right and the face becomes the wrong region on a body
    /// `validate()` passes (3.03e-2 of volume on the rib-base document). Zero
    /// when nothing was pulled. A REPORT, not a bar: nothing here refuses on
    /// it. Its coverage is the clamp in `build_interpolant`, which is the only
    /// place a raw out-of-domain parameter reaches today — an affine carrier's
    /// inversion is unclamped, so an affine plane fitted through the station
    /// lane reads here. It does NOT see a clamp the projector applied before
    /// the fit: the analytic `Plane` closed form (a non-affine net) clamps to
    /// its domain, `RuledRevolution` clamps its axial parameter to `[0, 1]`
    /// and the general Newton clamps through `fit_parameter`; those stations
    /// arrive in-domain and read as `off_surface` instead. The exact affine
    /// lanes (`fit_pcurve_on_surface` on an affine carrier,
    /// `affine_pcurve_on_range`) never clamp and read zero by construction.
    pub clamped_excursion: f64,
    /// The 3D distance the clamp corresponds to: between the extended surface
    /// at the raw parameter and the clamped foot, at the station of the
    /// largest such distance. Zero when nothing was pulled.
    pub clamped_distance: f64,
}

impl PcurveFitReport {
    /// Whether the returned curve reaches the floor it was asked for.
    pub fn met_floor(&self) -> bool {
        self.residual <= self.floor
    }

    /// Whether the returned curve is on the ACCEPTANCE BAR: the floor it was
    /// asked for, plus the stations' own standoff from the carrier.
    ///
    /// A pcurve's image lies on the carrier, so at a station standing `g` off
    /// it no image can come closer than `g`; a fit whose residual is within
    /// the floor of `off_surface` is as good as the input allows, and one over
    /// it is worse than its input — the same condition [`refine_pcurve`] uses
    /// to decide that a raise can still buy accuracy. This is the bar every
    /// station builder refuses over ([`build_pcurve_on_surface_stations`]).
    /// It is blind to sideways error quadratically (a station tracked `d`
    /// aside of the projection of a point `g` off the surface reads only
    /// `d²/(2g)` further away), which is stated rather than hidden: on the
    /// import corpus the stations that stand off their carriers are the
    /// FILE's curves, and the bar there is the file's own number.
    pub fn on_bar(&self) -> bool {
        self.residual <= self.floor + self.off_surface
    }
}

/// Why a pcurve refinement stopped. Only [`PcurveFitExit::Converged`] means the
/// floor was met; every other variant is a BUDGET exit, and the report's
/// residual was measured on the returned curve rather than assumed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PcurveFitExit {
    /// A probe round found every station within the floor. An affine carrier
    /// maps exactly and reports this with a zero residual.
    Converged,
    /// The sample ceiling bound before the floor was met: [`MAX_PCURVE_SAMPLES`]
    /// when the raise was declined (the residual is already explained by
    /// [`PcurveFitReport::off_surface`]), [`PCURVE_RAISED_SAMPLES`] when it was
    /// taken and bound too.
    SampleCeiling,
    /// Every refinement round was spent and the last one still inserted
    /// samples, so the curve it built was never re-checked by the loop.
    RoundBudget,
    /// A round found stations over the floor but could improve none of them:
    /// the edge point could not be inverted inside the drop band, or (in the
    /// raised pass) the pcurve already tracks the surface's nearest point to
    /// within the floor. Either way the edge curve itself sits off the surface
    /// there and no pcurve can close that gap; the residual reports how far.
    /// The range lane's end-span challenger (its station fitter's TRACKING
    /// mode) also exits here when a round's probes all track their feet
    /// within the floor while the deviation residual, which includes the
    /// edge's standoff, stays over it.
    Stalled,
    /// The last probe round found every span it PROBED within the floor, but
    /// skipped spans narrower than the loop's own resolution floor — 1e-3 of
    /// the parameter range in [`refine_pcurve`], [`SPAN_FLOOR`] (1e-6) in
    /// [`fit_pcurve_on_surface_stations`] — and a sweep over those found the
    /// floor unmet.
    /// Until 2026-09-12 this exit was reported as `Converged` with the probed
    /// spans' maximum: on `25_cube_pierce_solid` a 1539-sample fit with 1537
    /// narrow spans read 7e-8 that way while every span read 1.9e-4.
    SpanFloor,
}

/// A fitted pcurve with what the fit achieved.
#[derive(Clone, Debug)]
pub struct PcurveFit {
    pub curve: NurbsCurve,
    pub report: PcurveFitReport,
}

/// Per-operation tally of pcurve fits and how many missed their floor.
///
/// [`build_pcurve_on_surface`] and its marched twin are called from some forty
/// sites, most of which have no diagnostics record to write into. Rather than
/// thread one through every layer, an operation that returns
/// [`KernelDiagnostics`] opens a [`PcurveFitScope`]; every fit made on that
/// thread while the scope is open is tallied here, and closing the scope hands
/// the tally back for [`PcurveFitLedger::report_into`]. This is the shape the
/// boolean's `CONFORMANCE_REPAIRS` counter already uses. With no scope open,
/// recording is a no-op.
///
/// Scopes nest: CLOSING a child folds it into its parent, so an offset shell
/// sees the fits of the booleans it ran. A scope that is DROPPED without being
/// closed is discarded, not folded — a boolean attempt that refused (and may be
/// retried under perturbation) must not leave its fits on the caller's tally.
///
/// Thread-local by construction: a fit made on a rayon worker (the `parallel`
/// feature's STEP body import) is not seen by a scope on the calling thread. No
/// diagnostics-returning operation builds pcurves off-thread today.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PcurveFitLedger {
    /// Fits recorded, exact affine ones included.
    pub fits: u64,
    /// Fits whose returned curve did not reach its floor.
    pub unmet: u64,
    /// Unmet fits that stopped at a sample ceiling.
    pub sample_ceiling: u64,
    /// Unmet fits that spent every refinement round.
    pub round_budget: u64,
    /// Unmet fits that could insert nothing.
    pub stalled: u64,
    /// Unmet fits whose remaining over-floor spans were all below the loop's
    /// 1e-3 span floor.
    pub span_floor: u64,
    /// Largest residual among the unmet fits; zero when there are none.
    pub worst_unmet_residual: f64,
    /// Fits that re-entered refinement with [`PCURVE_RAISED_SAMPLES`], met or
    /// not.
    pub raised: u64,
    /// Unmet ceiling exits that were NOT raised because the residual is
    /// already within the floor of the edge curve's own gap from the surface.
    pub raise_declined: u64,
    /// Largest [`PcurveFitReport::off_surface`] over every fit.
    pub worst_off_surface: f64,
    /// Fits whose interpolant was clamped into an open direction's domain
    /// ([`PcurveFitReport::clamped_excursion`] above zero), met or not.
    pub clamped: u64,
    /// Largest [`PcurveFitReport::clamped_excursion`] over every fit.
    pub worst_clamped_excursion: f64,
    /// Largest [`PcurveFitReport::clamped_distance`] over every fit.
    pub worst_clamped_distance: f64,
}

impl PcurveFitLedger {
    fn record(&mut self, report: &PcurveFitReport) {
        self.fits += 1;
        if report.raised {
            self.raised += 1;
        }
        self.worst_off_surface = self.worst_off_surface.max(report.off_surface);
        if report.clamped_excursion > 0.0 {
            self.clamped += 1;
            self.worst_clamped_excursion = self.worst_clamped_excursion.max(report.clamped_excursion);
            self.worst_clamped_distance = self.worst_clamped_distance.max(report.clamped_distance);
        }
        if report.met_floor() {
            return;
        }
        self.unmet += 1;
        match report.exit {
            PcurveFitExit::Converged => {}
            PcurveFitExit::SampleCeiling => {
                self.sample_ceiling += 1;
                if !report.raised {
                    self.raise_declined += 1;
                }
            }
            PcurveFitExit::RoundBudget => self.round_budget += 1,
            PcurveFitExit::Stalled => self.stalled += 1,
            PcurveFitExit::SpanFloor => self.span_floor += 1,
        }
        self.worst_unmet_residual = self.worst_unmet_residual.max(report.residual);
    }

    fn fold(&mut self, child: &PcurveFitLedger) {
        self.fits += child.fits;
        self.unmet += child.unmet;
        self.sample_ceiling += child.sample_ceiling;
        self.round_budget += child.round_budget;
        self.stalled += child.stalled;
        self.span_floor += child.span_floor;
        self.worst_unmet_residual = self.worst_unmet_residual.max(child.worst_unmet_residual);
        self.raised += child.raised;
        self.raise_declined += child.raise_declined;
        self.worst_off_surface = self.worst_off_surface.max(child.worst_off_surface);
        self.clamped += child.clamped;
        self.worst_clamped_excursion = self.worst_clamped_excursion.max(child.worst_clamped_excursion);
        self.worst_clamped_distance = self.worst_clamped_distance.max(child.worst_clamped_distance);
    }

    /// Write the tally into an operation's diagnostics: the `pcurve.*`
    /// counters and measurements always, plus one `pcurve.unmet_floor` event at
    /// [`DiagnosticSeverity::Degraded`] when any fit missed its floor. Degraded,
    /// not Error: the result is still the kernel's best construction and stays
    /// shippable, but it no longer claims the accuracy its floor states.
    pub fn report_into(&self, diagnostics: &mut KernelDiagnostics) {
        diagnostics.count_n("pcurve.fits", self.fits);
        diagnostics.count_n("pcurve.unmet_floor", self.unmet);
        diagnostics.count_n("pcurve.exit.sample_ceiling", self.sample_ceiling);
        diagnostics.count_n("pcurve.exit.round_budget", self.round_budget);
        diagnostics.count_n("pcurve.exit.stalled", self.stalled);
        diagnostics.count_n("pcurve.exit.span_floor", self.span_floor);
        diagnostics.count_n("pcurve.budget_raised", self.raised);
        diagnostics.count_n("pcurve.budget_raise_declined", self.raise_declined);
        diagnostics.measure_max("pcurve.worst_unmet_residual", self.worst_unmet_residual);
        diagnostics.measure_max("pcurve.worst_off_surface", self.worst_off_surface);
        diagnostics.count_n("pcurve.clamped", self.clamped);
        diagnostics.measure_max("pcurve.worst_clamped_excursion", self.worst_clamped_excursion);
        diagnostics.measure_max("pcurve.worst_clamped_distance", self.worst_clamped_distance);
        if self.unmet > 0 {
            diagnostics.event(
                DiagnosticSeverity::Degraded,
                KernelStage::Refine,
                "pcurve.unmet_floor",
                format!(
                    "{} of {} pcurve fits did not reach the {:.0e} floor \
                     (sample ceiling {}, round budget {}, stalled {}, span floor {}); \
                     worst residual {:.3e}; budget raised on {}, declined on {}; \
                     edge curves sit off their surfaces by up to {:.3e}{}",
                    self.unmet,
                    self.fits,
                    PCURVE_REFINEMENT_TOLERANCE,
                    self.sample_ceiling,
                    self.round_budget,
                    self.stalled,
                    self.span_floor,
                    self.worst_unmet_residual,
                    self.raised,
                    self.raise_declined,
                    self.worst_off_surface,
                    if self.clamped > 0 {
                        format!(
                            "; {} fit(s) clamped into their chart by up to {:.3e} in parameter, {:.3e} in 3D",
                            self.clamped, self.worst_clamped_excursion, self.worst_clamped_distance
                        )
                    } else {
                        String::new()
                    },
                ),
            );
        }
    }
}

thread_local! {
    /// The open [`FootMemo`] on this thread, if any ([`FootMemoScope`]).
    static FOOT_MEMO: std::cell::RefCell<Option<FootMemo>> = const { std::cell::RefCell::new(None) };
}

/// Exact-input memo of the common reader's projections on ONE surface: the
/// global foot of an edge point and the foot seeded at a (u, v), keyed by the
/// bit patterns of their inputs. Both projectors are pure functions of
/// (surface, point[, seed]), so a hit returns the value a fresh call would:
/// the readings, decisions and fits are bit-identical with or without it.
/// It exists for cost: within one coedge the same edge points are read by
/// the trigger read, every tracking-mode verification round, the
/// challenger's final read and the importer's keep read, each of which
/// projected them again (`importTestWorking`, 2026-10-05: about 10k probes
/// read 3-5 times on each of nine off-carrier refits, 3-5 s apiece).
struct FootMemo {
    surface: *const NurbsSurface,
    global: rustc_hash::FxHashMap<[u64; 3], Option<crate::SurfaceProjection>>,
    seeded: rustc_hash::FxHashMap<[u64; 5], Option<crate::SurfaceProjection>>,
}

/// Opens a [`FootMemo`] for `surface` on this thread until dropped. Opening
/// one where a memo for the SAME surface is already open reuses it (an inner
/// scope, e.g. the range challenger inside an importer coedge); a memo for
/// another surface is set aside and restored on drop. The borrow of `surface`
/// keeps it alive and unmodified for the scope, so the pointer key cannot be
/// reused by another surface while the memo answers for it.
pub(crate) struct FootMemoScope<'a> {
    previous: Option<Option<FootMemo>>,
    _surface: std::marker::PhantomData<&'a NurbsSurface>,
    /// Not `Send`: the scope must close on the thread that opened it.
    _thread_bound: std::marker::PhantomData<*const ()>,
}

impl<'a> FootMemoScope<'a> {
    pub(crate) fn open(surface: &'a NurbsSurface) -> Self {
        let key = surface as *const NurbsSurface;
        let previous = FOOT_MEMO.with(|memo| {
            let mut memo = memo.borrow_mut();
            if memo.as_ref().is_some_and(|open| std::ptr::eq(open.surface, key)) {
                None
            } else {
                Some(memo.replace(FootMemo { surface: key, global: Default::default(), seeded: Default::default() }))
            }
        });
        Self { previous, _surface: std::marker::PhantomData, _thread_bound: std::marker::PhantomData }
    }
}

impl Drop for FootMemoScope<'_> {
    fn drop(&mut self) {
        if let Some(previous) = self.previous.take() {
            FOOT_MEMO.with(|memo| *memo.borrow_mut() = previous);
        }
    }
}

fn point_bits(point: Vec3) -> [u64; 3] {
    [point.x.to_bits(), point.y.to_bits(), point.z.to_bits()]
}

/// [`project_point_to_surface`] as `.ok()`, through the open memo for this
/// surface when there is one. `#[track_caller]`: the projector still sees the
/// reader's own call site. Under a `BREP_PROJECTION_GLOBAL_SITES`/`_EXCEPT`
/// filter the projector's answer depends on that site, so the memo is not
/// consulted then.
#[track_caller]
fn memo_global_foot(surface: &NurbsSurface, point: Vec3) -> Option<crate::SurfaceProjection> {
    if crate::projection::global_site_filter_active() {
        return project_point_to_surface(surface, point).ok();
    }
    let key = point_bits(point);
    let hit = FOOT_MEMO.with(|memo| {
        memo.borrow().as_ref().filter(|open| std::ptr::eq(open.surface, surface)).map(|open| open.global.get(&key).copied())
    });
    match hit {
        Some(Some(foot)) => foot,
        Some(None) => {
            let foot = project_point_to_surface(surface, point).ok();
            FOOT_MEMO.with(|memo| {
                if let Some(open) = memo.borrow_mut().as_mut().filter(|open| std::ptr::eq(open.surface, surface)) {
                    open.global.insert(key, foot);
                }
            });
            foot
        }
        None => project_point_to_surface(surface, point).ok(),
    }
}

/// [`project_point_to_surface_seeded`] as `.ok()`, through the open memo for
/// this surface when there is one.
fn memo_seeded_foot(surface: &NurbsSurface, point: Vec3, seed_u: f64, seed_v: f64) -> Option<crate::SurfaceProjection> {
    let [x, y, z] = point_bits(point);
    let key = [x, y, z, seed_u.to_bits(), seed_v.to_bits()];
    let hit = FOOT_MEMO.with(|memo| {
        memo.borrow().as_ref().filter(|open| std::ptr::eq(open.surface, surface)).map(|open| open.seeded.get(&key).copied())
    });
    match hit {
        Some(Some(foot)) => foot,
        Some(None) => {
            let foot = project_point_to_surface_seeded(surface, point, seed_u, seed_v).ok();
            FOOT_MEMO.with(|memo| {
                if let Some(open) = memo.borrow_mut().as_mut().filter(|open| std::ptr::eq(open.surface, surface)) {
                    open.seeded.insert(key, foot);
                }
            });
            foot
        }
        None => project_point_to_surface_seeded(surface, point, seed_u, seed_v).ok(),
    }
}

thread_local! {
    /// The open [`PcurveFitScope`]s on this thread, innermost last.
    static PCURVE_FIT_SCOPES: std::cell::RefCell<Vec<PcurveFitLedger>> =
        const { std::cell::RefCell::new(Vec::new()) };
}

/// `BREP_DEBUG_PCURVE_FIT` only: a process-wide id per [`refine_pcurve`]
/// call, so its first-pass and candidate `PCURVE-TRACK` lines pair up even
/// when fits run on several threads.
static PCURVE_TRACK_CALL: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// An open tally of the pcurve fits made on this thread — see
/// [`PcurveFitLedger`]. Open it where an operation starts, [`close`] it where
/// the operation returns its diagnostics; a `?` that unwinds past it drops the
/// tally.
///
/// [`close`]: PcurveFitScope::close
pub struct PcurveFitScope {
    /// Not `Send`: the scope must close on the thread that opened it.
    _thread_bound: std::marker::PhantomData<*const ()>,
}

impl PcurveFitScope {
    pub fn open() -> Self {
        PCURVE_FIT_SCOPES.with(|scopes| scopes.borrow_mut().push(PcurveFitLedger::default()));
        Self {
            _thread_bound: std::marker::PhantomData,
        }
    }

    /// Take the tally, folding it into the enclosing scope if there is one.
    pub fn close(self) -> PcurveFitLedger {
        let ledger = PCURVE_FIT_SCOPES.with(|scopes| {
            let mut scopes = scopes.borrow_mut();
            let ledger = scopes.pop().unwrap_or_default();
            if let Some(parent) = scopes.last_mut() {
                parent.fold(&ledger);
            }
            ledger
        });
        std::mem::forget(self);
        ledger
    }
}

impl Drop for PcurveFitScope {
    fn drop(&mut self) {
        PCURVE_FIT_SCOPES.with(|scopes| {
            scopes.borrow_mut().pop();
        });
    }
}

pub(crate) fn record_fit(report: &PcurveFitReport) {
    // Per-fit trace for the convergence probes (`trim_floor_sweep_probe` and
    // its offset-shell sibling): one line per fit with the whole report, the
    // way `BREP_DEBUG_PCURVE` traces an endpoint failure.
    if std::env::var_os("BREP_DEBUG_PCURVE_FIT").is_some() {
        eprintln!("PCURVE-FIT {report:?}");
    }
    PCURVE_FIT_SCOPES.with(|scopes| {
        if let Some(open) = scopes.borrow_mut().last_mut() {
            open.record(report);
        }
    });
}

/// Largest 3D distance between `pcurve`'s image and `curve` at the
/// quarter-points of every span of `parameters` — the stations refinement
/// probes, over EVERY span. Evaluations only; nothing is inverted. The sweep
/// measures, it does not decide.
fn probe_residual(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    pcurve: &NurbsCurve,
    parameters: &[f64],
    domain: [f64; 2],
    _u_domain: [f64; 2],
    _v_domain: [f64; 2],
) -> Result<f64, String> {
    probe_residual_at(surface, curve, pcurve, parameters, domain, &[0.25, 0.5, 0.75])
}

/// Extra interior points per sample span, at `j/17` for `j = 1..=16`, that
/// [`tracking_acceptance_residual`] reads IN ADDITION to [`probe_residual`]'s
/// quarter, mid and three-quarter points (nineteen fractions per span in
/// all), so it can never read below the three-point maximum on the same
/// curve. The three quarter-points under-read the sixteen-point maximum by
/// 3.3–6.5 % on the constructed-cylinder union's imprint pcurves, and sixteen
/// sit within 0.4 % of sixty-four there (2026-10-04); the Converged path keeps
/// its own read.
const TRACKING_ACCEPTANCE_POINTS: usize = 16;

/// The same-parameter image-vs-edge distance of `pcurve` at `locals` (span
/// fractions) inside every span of `parameters`: the largest, or an error if
/// any is not finite.
fn probe_residual_at(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    pcurve: &NurbsCurve,
    parameters: &[f64],
    [t0, t1]: [f64; 2],
    locals: &[f64],
) -> Result<f64, String> {
    let mut worst = 0.0_f64;
    for span in parameters.windows(2) {
        for &local in locals {
            let fraction = span[0] + (span[1] - span[0]) * local;
            let parameter = pcurve.evaluate(fraction)?;
            let on_surface =
                surface.evaluate_extended(parameter.x, parameter.y)?;
            let on_curve = curve.evaluate(t0 + (t1 - t0) * fraction)?;
            // `f64::max` drops a NaN operand, so a non-finite distance would
            // leave `worst` reading the finite stations only. A residual that
            // can accept a curve must have measured every station: fail
            // closed instead.
            let distance = on_surface.sub(on_curve).length();
            if !distance.is_finite() {
                return Err(format!(
                    "probe_residual: non-finite image distance {distance} at fraction {fraction}"
                ));
            }
            worst = worst.max(distance);
        }
    }
    Ok(worst)
}

/// The probe fractions a native read takes for the given pcurves: both
/// domain ENDPOINTS, and the quarter points plus 16 interior points of every
/// span between each curve's OWN distinct knots (the installed interpolant's
/// spans, not the construction stations). Merged over all the curves, sorted
/// and deduplicated, so a standing fit and a candidate are read at the SAME
/// fractions.
pub(crate) fn native_probe_fractions(curves: &[&NurbsCurve]) -> Result<Vec<f64>, String> {
    let locals: Vec<f64> = [0.25, 0.5, 0.75]
        .into_iter()
        .chain((1..=TRACKING_ACCEPTANCE_POINTS).map(|index| index as f64 / (TRACKING_ACCEPTANCE_POINTS + 1) as f64))
        .collect();
    let mut fractions = Vec::new();
    for curve in curves {
        let [q0, q1] = curve.domain()?;
        fractions.push(q0);
        fractions.push(q1);
        let mut knots: Vec<f64> = curve.knots.iter().copied().filter(|k| *k >= q0 && *k <= q1).collect();
        knots.dedup();
        for span in knots.windows(2).filter(|w| w[1] > w[0]) {
            for &local in &locals {
                fractions.push(span[0] + (span[1] - span[0]) * local);
            }
        }
    }
    fractions.sort_by(f64::total_cmp);
    fractions.dedup();
    Ok(fractions)
}

/// One pcurve read at given probe fractions against the edge curve (whose
/// parameter is t0 + (t1 - t0) x fraction): the image's distance from the
/// edge point's FOOT on the carrier (its TRACKING, the part a pcurve can
/// improve; the foot is seeded at the pcurve's own uv, closed form on an
/// analytic carrier), the image's distance from the edge point itself (the
/// same-parameter DEVIATION), and the edge point's distance from the carrier
/// (its INHERITED STANDOFF). Maxima over the probes; fails closed on any
/// non-finite point or distance.
#[derive(Clone, Copy, Debug, PartialEq)]
struct NativeRead {
    tracking: f64,
    deviation: f64,
    standoff: f64,
}

fn native_read(surface: &NurbsSurface, curve: &NurbsCurve, pcurve: &NurbsCurve, [t0, t1]: [f64; 2], fractions: &[f64]) -> Result<NativeRead, String> {
    let mut read = NativeRead { tracking: 0.0, deviation: 0.0, standoff: 0.0 };
    for &fraction in fractions {
        let parameter = pcurve.evaluate(fraction)?;
        let image = surface.evaluate_extended(parameter.x, parameter.y)?;
        let on_curve = curve.evaluate(t0 + (t1 - t0) * fraction)?;
        let foot = crate::projection::project_point_to_surface_from_seed(surface, on_curve, [parameter.x, parameter.y])?;
        let tracking = image.sub(foot.point).length();
        let deviation = image.sub(on_curve).length();
        if !(tracking.is_finite() && deviation.is_finite() && foot.distance.is_finite()) {
            return Err(format!("native read: non-finite reading at fraction {fraction}"));
        }
        read.tracking = read.tracking.max(tracking);
        read.deviation = read.deviation.max(deviation);
        read.standoff = read.standoff.max(foot.distance);
    }
    Ok(read)
}

/// Whether an off-carrier tracking candidate replaces the standing fit, both
/// read at the SAME probes ([`native_probe_fractions`] of both curves): its
/// tracking finite, within the unchanged floor and strictly lower, AND its
/// same-parameter deviation no worse (the reported total, inherited standoff
/// included, must not grow), AND its own refinement fields finite.
fn standoff_candidate_accepted(standing: NativeRead, candidate: NativeRead, candidate_fields_finite: bool, floor: f64) -> bool {
    candidate_fields_finite
        && candidate.tracking.is_finite()
        && candidate.deviation.is_finite()
        && candidate.tracking <= floor
        && candidate.tracking < standing.tracking
        && candidate.deviation <= standing.deviation
}

/// [`probe_residual_at`] at [`probe_residual`]'s three quarter-points plus
/// [`TRACKING_ACCEPTANCE_POINTS`] evenly spaced interior points of every span
/// (nineteen per span): the maximum of the old three-point reading and the
/// sixteen-point one. It is the reading a budget-exit tracking candidate is
/// accepted on, and the residual it is then reported with.
fn tracking_acceptance_residual(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    pcurve: &NurbsCurve,
    parameters: &[f64],
    domain: [f64; 2],
) -> Result<f64, String> {
    let locals: Vec<f64> = [0.25, 0.5, 0.75]
        .into_iter()
        .chain((1..=TRACKING_ACCEPTANCE_POINTS).map(|index| index as f64 / (TRACKING_ACCEPTANCE_POINTS + 1) as f64))
        .collect();
    probe_residual_at(surface, curve, pcurve, parameters, domain, &locals)
}

/// Whether a projection-tracking candidate replaces the fit standing in
/// [`refine_pcurve`], given the `reading` it is judged on (see
/// [`judge_tracking_candidate`]). `standing_dense` says the standing fit's
/// reading is itself a nineteen-point one: a budget-exit candidate was taken.
///
/// While the standing reading is the fitter's own, a `Converged` candidate is
/// taken exactly as before (within the floor, better). A BUDGET exit
/// (`SampleCeiling`, `RoundBudget`, `Stalled`, `SpanFloor`) is taken only to
/// repair a standing fit that MISSES the floor, and only when its
/// nineteen-point reading is finite, within the unchanged floor and strictly
/// better. Once the standing reading is a nineteen-point one, every later
/// candidate is compared nineteen against nineteen: a Converged one must read
/// finite, within the floor and strictly better on the same stencil, and a
/// budget exit is never taken (the standing fit is then within the floor).
/// The exit label says how the loop stopped, not whether the curve is good: on
/// the 15° constructed-cylinder union all six imprint pcurves that shipped
/// 1.3e-7..7.0e-6 off had a tracking candidate reading 1.4e-8..9.95e-8 at
/// three points per span, discarded for its label (2026-10-04). A standing
/// fit already within the floor is never replaced by a budget exit, so every
/// in-floor fit is unchanged; a taken candidate keeps its own exit, samples
/// and raise, and reports the reading it was taken on.
fn tracking_candidate_accepted(standing: f64, standing_dense: bool, floor: f64, exit: PcurveFitExit, reading: f64) -> bool {
    if exit == PcurveFitExit::Converged && !standing_dense {
        return reading <= floor && reading < standing;
    }
    if exit != PcurveFitExit::Converged && !(standing > floor) {
        return false;
    }
    reading.is_finite() && reading <= floor && reading < standing
}

/// The reading a tracking candidate is judged on, and whether it replaces the
/// standing fit: `own` (the candidate's own residual) for a Converged
/// candidate while the standing reading is the fitter's own; otherwise
/// `dense` ([`tracking_acceptance_residual`]'s nineteen points per span), read
/// only when the candidate could be taken. An error from `dense` declines.
/// [`refine_pcurve`] decides and traces through this one function.
fn judge_tracking_candidate(
    standing: f64,
    standing_dense: bool,
    floor: f64,
    exit: PcurveFitExit,
    own: f64,
    dense: impl FnOnce() -> Result<f64, String>,
) -> (Result<Option<f64>, String>, bool) {
    let reading = if exit == PcurveFitExit::Converged && !standing_dense {
        Ok(Some(own))
    } else if exit == PcurveFitExit::Converged || standing > floor {
        dense().map(Some)
    } else {
        Ok(None)
    };
    let accepted = matches!(
        reading,
        Ok(Some(value)) if tracking_candidate_accepted(standing, standing_dense, floor, exit, value)
    );
    (reading, accepted)
}

/// What one call to [`refine_pcurve`] achieved, beside the curve it left in
/// place.
struct Refinement {
    exit: PcurveFitExit,
    residual: f64,
    off_surface: f64,
    raised: bool,
    /// [`PcurveFitReport::clamped_excursion`] and
    /// [`PcurveFitReport::clamped_distance`] of the curve left in place.
    clamped: [f64; 2],
}

/// Adaptive 3D-residual refinement of an interpolant, with the PER-TRIM sample
/// budget. Shared by [`fit_pcurve_on_surface`] (global inversion) and
/// [`fit_pcurve_on_surface_marched`] (seeded projection); `insert` is the one
/// step that differs — given a probe station's fraction, the current pcurve's
/// parameter there and the edge point, it returns the surface parameter to
/// interpolate through (or `None` to decline the station) and the distance
/// from the edge point to the surface at that station, its GAP.
///
/// **First pass.** Four rounds at [`MAX_PCURVE_SAMPLES`], each probing the
/// quarter-points of every span wider than 1e-3 of the parameter range and
/// inserting the stations over `floor`. Byte-for-byte the pre-2026-09-12
/// loop; every exit is classified ([`PcurveFitExit`]) and the residual is
/// measured on the RETURNED curve — over every span, unless the final round
/// probed every span, in which case that round's maximum already is.
///
/// **The raise.** A [`PcurveFitExit::SampleCeiling`] exit re-enters the four
/// rounds ONCE with [`PCURVE_RAISED_SAMPLES`], continuing from the samples it
/// has, when the residual exceeds `off_surface` by more than the floor — the
/// returned curve is worse than the edge curve's own distance from the
/// surface, so more samples CAN improve it. Measured on `25_cube_pierce_solid`
/// (2026-09-12): the edge curves there sit off their surfaces by up to 1.9e-4,
/// four of the eight ceiling exits already read their gap to four digits, and
/// raising those spent 900 inversions each for no change in the swept
/// residual. The raised pass therefore inserts only where the pcurve's surface
/// point is further than the floor from the PROJECTION of the edge point — its
/// tracking error, which the inversion just computed: it stops when the pcurve
/// tracks the surface's nearest point to the floor, which is the best any
/// pcurve can do, and reports [`PcurveFitExit::Stalled`] with the deviation it
/// actually reached when that is still over the floor. The limit of that pass
/// as the floor tightens is the projected edge curve itself, so a volume built
/// on it can be shown to converge; the deviation test alone cannot say that on
/// off-surface input. Grow-once, capped, deterministic.
#[allow(clippy::too_many_arguments)]
fn refine_pcurve(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    domain: [f64; 2],
    u_domain: [f64; 2],
    v_domain: [f64; 2],
    floor: f64,
    joins: &[f64],
    parameters: &mut Vec<f64>,
    raw: &mut Vec<[f64; 2]>,
    pcurve: &mut NurbsCurve,
    off_surface: f64,
    clamped: [f64; 2],
    mut insert: impl FnMut(f64, Vec3, Vec3) -> Result<(Option<[f64; 2]>, f64), String>,
) -> Result<Refinement, String> {
    // Trace only (`BREP_DEBUG_PCURVE_FIT`): reads values the passes already
    // computed and decides nothing.
    let trace = std::env::var_os("BREP_DEBUG_PCURVE_FIT").is_some();
    let call = if trace {
        PCURVE_TRACK_CALL.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1
    } else {
        0
    };
    let mut best = refine_pcurve_pass(surface, curve, domain, u_domain, v_domain,
        floor, joins, parameters, raw, pcurve, off_surface, clamped, false, 1.0, false, &mut insert)?;
    if trace {
        eprintln!(
            "PCURVE-TRACK call={call} first-pass exit={:?} residual={:e} floor={floor:e} off_surface={:e} samples={} raised={} tracking={}",
            best.exit,
            best.residual,
            best.off_surface,
            parameters.len(),
            best.raised,
            if best.off_surface <= floor { "eligible" } else { "standoff-tracking" }
        );
    }
    if best.off_surface > floor {
        // The edge curve itself stands off the carrier by more than the floor
        // (its INHERITED standoff), so no pcurve can bring the image within the
        // floor of the edge point. What a pcurve CAN improve is its tracking
        // error: the image's distance from the edge point's foot. Measured
        // 2026-10-04 (a2b401d4d, 15-degree crossing, turn 295): a first pass
        // ended RoundBudget at 6.726e-6 over a 1.316e-7 standoff and every
        // tracking pass was skipped. No candidate refinement when the ACTUAL
        // tracking request is met: the first pass's own tracking, read on its
        // native spans and endpoints (a diagnostic read; it adds no station),
        // within the floor ([`standoff_tracking_rungs`]). A total residual
        // under standoff + floor does not prove that: a tangential slip t over
        // a standoff g reads sqrt(g^2 + t^2) < g + t.
        // `BREP_PCURVE_STANDOFF_TRACKING=0`: diagnostic A/B hatch, not a
        // policy. It restores the pre-lane answer (35c8610a1's parent: an
        // off-carrier first pass is returned as it stands) for a same-binary
        // attribution of a volume or census delta to this lane. Default (unset
        // or any other value) is unchanged. It does not touch the in-floor
        // tracking rungs or their raise rule, nor the range lane's end-span
        // challenger.
        if !standoff_tracking_enabled() {
            return Ok(best);
        }
        return standoff_tracking_rungs(surface, curve, domain, u_domain, v_domain, floor, joins,
            parameters, raw, pcurve, best, trace, call, &mut insert);
    }
    // Each target starts from the same baseline station set: retaining the
    // coarser target's inserts can exhaust the cap before the finer target
    // places its own error-directed stations. Keep the best rung
    // [`tracking_candidate_accepted`] takes.
    let baseline_parameters = parameters.clone();
    let baseline_raw = raw.clone();
    let baseline_curve = pcurve.clone();
    let baseline_off_surface = best.off_surface;
    let baseline_clamped = best.clamped;
    // Whether the first pass MISSED the floor: only then may a tracking
    // candidate that reaches the sample ceiling still missing its request take
    // the existing one-time raise (2026-10-04, 20-degree crossing, turn 295:
    // a 1.964e-6 first pass, a candidate at 600 samples reading 1.068e-7,
    // declined; the raise needed residual > off_surface + floor). A first pass
    // within the floor keeps exactly the work it did before.
    // (On this path the edge stands within the floor of the carrier, so a
    // first pass over the floor is a tracking miss, not inherited standoff.)
    let standing_misses = best.residual > floor;
    Ok(tracking_rungs(
        best,
        floor,
        trace,
        call,
        true,
        |fraction| {
            let mut refined_parameters = baseline_parameters.clone();
            let mut refined_raw = baseline_raw.clone();
            let mut refined_curve = baseline_curve.clone();
            let candidate = refine_pcurve_pass(surface, curve, domain, u_domain, v_domain,
                floor, joins, &mut refined_parameters, &mut refined_raw, &mut refined_curve,
                baseline_off_surface, baseline_clamped, true, fraction, standing_misses, &mut insert);
            (candidate, (refined_parameters, refined_raw, refined_curve))
        },
        |state: &RungState| tracking_acceptance_residual(surface, curve, &state.2, &state.0, domain),
        |state: &RungState| state.0.len(),
        |(refined_parameters, refined_raw, refined_curve): RungState| {
            *parameters = refined_parameters;
            *raw = refined_raw;
            *pcurve = refined_curve;
        },
    ))
}

/// A tracking candidate's station parameters, raw surface parameters and
/// pcurve, as [`refine_pcurve`] hands them to [`tracking_rungs`].
type RungState = (Vec<f64>, Vec<[f64; 2]>, NurbsCurve);

/// The tracking rungs of [`refine_pcurve`] after an eligible first pass: for
/// each tracking fraction, skip it if the standing fit is already that far
/// inside the floor, otherwise run the candidate pass (`run_pass`), judge it
/// ([`judge_tracking_candidate`], reading `dense` only when needed), trace
/// it, and either take it (`adopt` its state; it becomes the standing fit) or
/// stop. `samples` is the candidate's station count, for the trace. Separated
/// so the accepted-budget state can be driven with scripted passes;
/// `thread_dense` is `true` in production and `false` only to show, in a
/// test, what dropping the nineteen-point standing flag would take.
#[allow(clippy::too_many_arguments)]
fn tracking_rungs<S>(
    mut best: Refinement,
    floor: f64,
    trace: bool,
    call: u64,
    thread_dense: bool,
    mut run_pass: impl FnMut(f64) -> (Result<Refinement, String>, S),
    mut dense: impl FnMut(&S) -> Result<f64, String>,
    samples: impl Fn(&S) -> usize,
    mut adopt: impl FnMut(S),
) -> Refinement {
    // Whether the standing reading is a nineteen-point one (a budget-exit
    // candidate was taken); every later candidate is then judged on it.
    let mut standing_dense = false;
    for fraction in [0.1, 0.01] {
        if best.residual <= floor * fraction {
            if trace {
                eprintln!(
                    "PCURVE-TRACK call={call} candidate fraction={fraction} skipped best_residual={:e} <= {:e}",
                    best.residual,
                    floor * fraction
                );
            }
            continue;
        }
        let (candidate, state) = run_pass(fraction);
        // The reading the candidate is judged on and the decision (see
        // judge_tracking_candidate); an unreadable read declines like a
        // failed pass.
        let judged_dense = matches!(&candidate, Ok(seen) if seen.exit != PcurveFitExit::Converged || standing_dense);
        let (effective, accepted) = match &candidate {
            Ok(seen) => judge_tracking_candidate(best.residual, standing_dense, floor, seen.exit, seen.residual, || {
                dense(&state)
            }),
            Err(_) => (Ok(None), false),
        };
        if trace {
            let judged_on = match &effective {
                Ok(Some(reading)) => format!("{reading:e}"),
                Ok(None) => "-".to_string(),
                Err(message) => format!("error:{message:?}"),
            };
            match &candidate {
                Ok(seen) => eprintln!(
                    "PCURVE-TRACK call={call} candidate fraction={fraction} exit={:?} residual={:e} judged_on={judged_on} standing_dense={standing_dense} floor={floor:e} off_surface={:e} samples={} raised={} residual_meets_floor={} decision={}",
                    seen.exit,
                    seen.residual,
                    seen.off_surface,
                    samples(&state),
                    seen.raised,
                    seen.residual <= floor,
                    if accepted { "accept" } else { "decline-stop" }
                ),
                Err(message) => eprintln!(
                    "PCURVE-TRACK call={call} candidate fraction={fraction} error={message:?} decision=decline-stop"
                ),
            }
        }
        match candidate {
            Ok(mut candidate) if accepted => {
                // Report the reading it was taken on (its own, if Converged).
                if let Ok(Some(reading)) = effective {
                    candidate.residual = reading;
                }
                // From here on the standing reading is a nineteen-point one.
                if thread_dense {
                    standing_dense |= judged_dense;
                }
                adopt(state);
                best = candidate;
            }
            _ => break,
        }
    }
    best
}

/// Whether [`refine_pcurve`] takes [`standoff_tracking_rungs`] for an
/// off-carrier first pass: true unless `BREP_PCURVE_STANDOFF_TRACKING=0`
/// (diagnostic A/B hatch, read once per process).
fn standoff_tracking_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_PCURVE_STANDOFF_TRACKING").is_none_or(|value| value != "0"))
}

/// The tracking rungs for an edge curve that stands OFF its carrier by more
/// than the floor ([`refine_pcurve`]). The first pass is read natively
/// ([`native_read`] on its own knot spans and endpoints); when its tracking is
/// within the floor, or the read fails, nothing more is done. Otherwise each
/// candidate pass (fractions 0.1 then 0.01 of the floor, from the same
/// baseline stations, taking the one-time raise when its own tracking target
/// is cut short by the ceiling) is read with the standing fit at the SAME
/// probes (both curves' native spans and endpoints) and taken only per
/// [`standoff_candidate_accepted`]: tracking finite, within the unchanged
/// floor and strictly lower, same-parameter deviation no worse, refinement
/// fields finite. A taken fit is REPORTED with its same-probe deviation
/// (inherited standoff included, so still over the floor) and with an
/// `off_surface` that is the larger of its station reading and the probes'
/// standoff. Any error, or a candidate that does not qualify, stops the loop
/// and keeps the standing curve and report.
#[allow(clippy::too_many_arguments)]
fn standoff_tracking_rungs(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    domain: [f64; 2],
    u_domain: [f64; 2],
    v_domain: [f64; 2],
    floor: f64,
    joins: &[f64],
    parameters: &mut Vec<f64>,
    raw: &mut Vec<[f64; 2]>,
    pcurve: &mut NurbsCurve,
    mut best: Refinement,
    trace: bool,
    call: u64,
    insert: &mut impl FnMut(f64, Vec3, Vec3) -> Result<(Option<[f64; 2]>, f64), String>,
) -> Result<Refinement, String> {
    let first = native_probe_fractions(&[&*pcurve]).and_then(|probes| native_read(surface, curve, pcurve, domain, &probes));
    if trace {
        eprintln!("PCURVE-TRACK call={call} standoff-tracking first-pass native={first:?} floor={floor:e}");
    }
    let Ok(first) = first else { return Ok(best) };
    if first.tracking <= floor {
        return Ok(best);
    }
    let baseline_parameters = parameters.clone();
    let baseline_raw = raw.clone();
    let baseline_curve = pcurve.clone();
    for fraction in [0.1, 0.01] {
        let mut refined_parameters = baseline_parameters.clone();
        let mut refined_raw = baseline_raw.clone();
        let mut refined_curve = baseline_curve.clone();
        let candidate = refine_pcurve_pass(surface, curve, domain, u_domain, v_domain, floor, joins,
            &mut refined_parameters, &mut refined_raw, &mut refined_curve, best.off_surface, best.clamped,
            true, fraction, true, &mut *insert);
        let reads = candidate.as_ref().ok().map(|_| {
            native_probe_fractions(&[&*pcurve, &refined_curve]).and_then(|probes| {
                Ok((native_read(surface, curve, pcurve, domain, &probes)?, native_read(surface, curve, &refined_curve, domain, &probes)?))
            })
        });
        let fields_finite = matches!(&candidate, Ok(seen) if seen.residual.is_finite() && seen.off_surface.is_finite());
        let decision = match &reads {
            Some(Ok((standing, read))) => standoff_rung_decision(*standing, *read, fields_finite, floor),
            _ => RungDecision::Stop,
        };
        if trace {
            eprintln!(
                "PCURVE-TRACK call={call} standoff-tracking fraction={fraction} candidate={} reads={reads:?} decision={}",
                match &candidate { Ok(seen) => format!("{:?}/{:e}/raised={}", seen.exit, seen.residual, seen.raised), Err(message) => format!("error:{message:?}") },
                match decision { RungDecision::Accept => "accept", RungDecision::Continue => "decline-continue", RungDecision::Stop => "decline-stop" }
            );
        }
        let (Ok(mut taken), Some(Ok((_, read))), RungDecision::Accept) = (candidate, reads, decision) else {
            if decision == RungDecision::Continue {
                continue;
            }
            break;
        };
        taken.residual = read.deviation;
        taken.off_surface = taken.off_surface.max(read.standoff);
        *parameters = refined_parameters;
        *raw = refined_raw;
        *pcurve = refined_curve;
        best = taken;
        if read.tracking <= floor * 0.01 {
            break;
        }
    }
    Ok(best)
}

/// What one off-carrier tracking rung does with its candidate
/// ([`standoff_tracking_rungs`]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RungDecision {
    /// Taken ([`standoff_candidate_accepted`]).
    Accept,
    /// Not taken, but it read finite and LOWER tracking than the standing fit
    /// while over the floor: the finer fraction is still tried from the same
    /// baseline (fixture 25 on e90c30c94: one use read 2.33e-7 at 0.1 and
    /// stopped while its partner was taken at 9.0e-9).
    Continue,
    /// Not taken and not improving (or unreadable): the rungs end.
    Stop,
}

fn standoff_rung_decision(standing: NativeRead, candidate: NativeRead, fields_finite: bool, floor: f64) -> RungDecision {
    if standoff_candidate_accepted(standing, candidate, fields_finite, floor) {
        RungDecision::Accept
    } else if candidate.tracking.is_finite() && candidate.tracking < standing.tracking {
        RungDecision::Continue
    } else {
        RungDecision::Stop
    }
}

/// The one-time raise decision of [`refine_pcurve_pass`]: a ceiling exit not
/// yet raised, when the returned curve is worse than the edge's own gap by
/// more than the floor (the original rule), or when a tracking pass for an
/// unmet first pass returned a curve whose own tracking still misses the
/// pass's target (`unmet_tracking`, read on the returned curve). Nothing else
/// raises.
fn takes_raise(exit: PcurveFitExit, raised: bool, residual: f64, off_surface: f64, floor: f64, unmet_tracking: bool) -> bool {
    exit == PcurveFitExit::SampleCeiling && !raised && (residual > off_surface + floor || unmet_tracking)
}

fn refine_pcurve_pass(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    [t0, t1]: [f64; 2],
    [u0, u1]: [f64; 2],
    [v0, v1]: [f64; 2],
    floor: f64,
    joins: &[f64],
    parameters: &mut Vec<f64>,
    raw: &mut Vec<[f64; 2]>,
    pcurve: &mut NurbsCurve,
    mut off_surface: f64,
    mut clamped: [f64; 2],
    track_projection: bool,
    tracking_fraction: f64,
    raise_unmet_tracking: bool,
    mut insert: impl FnMut(f64, Vec3, Vec3) -> Result<(Option<[f64; 2]>, f64), String>,
) -> Result<Refinement, String> {
    let tracking_floor = if track_projection { floor * tracking_fraction } else { floor };
    let mut max_samples = MAX_PCURVE_SAMPLES;
    let mut raised = false;
    loop {
        // Every way out of this loop is CLASSIFIED (`PcurveFitExit`); a budget
        // exit returns the curve it built, but says so. Falling off the end
        // after a fourth round of inserts is the round-budget exit.
        let mut exit = PcurveFitExit::RoundBudget;
        let mut residual = None;
        for _ in 0..4 {
            if parameters.len() >= max_samples {
                exit = PcurveFitExit::SampleCeiling;
                break;
            }
            let mut inserts = Vec::new();
            let mut round_max = 0.0_f64;
            let mut truncated = false;
            let mut skipped_narrow = false;
            for index in 0..parameters.len() - 1 {
                if parameters.len() + inserts.len() >= max_samples {
                    truncated = true;
                    break;
                }
                let start = parameters[index];
                let end = parameters[index + 1];
                // Keep construction's existing first-pass policy. Extended
                // residual probes must not silently change its station ladder;
                // projection tracking retains its established finer floor.
                if end - start < if track_projection { SPAN_FLOOR } else { 1e-3 } {
                    skipped_narrow = true;
                    continue;
                }
                for local_fraction in [0.25, 0.5, 0.75] {
                    if parameters.len() + inserts.len() >= max_samples {
                        truncated = true;
                        break;
                    }
                    let fraction = start + (end - start) * local_fraction;
                    let parameter = pcurve.evaluate(fraction)?;
                    let on_surface =
                        surface.evaluate_extended(parameter.x, parameter.y)?;
                    let on_curve = curve.evaluate(t0 + (t1 - t0) * fraction)?;
                    let deviation = on_surface.sub(on_curve).length();
                    // A non-finite distance must not reach `f64::max` (which
                    // drops NaN) or the classification below, where a round
                    // with an unreadable station could still exit Converged on
                    // its readable ones. Fail closed: a first pass's error
                    // goes to its caller, a tracking candidate's declines it.
                    if !deviation.is_finite() {
                        return Err(format!(
                            "refine_pcurve_pass: non-finite station distance {deviation} at fraction {fraction}"
                        ));
                    }
                    round_max = round_max.max(deviation);
                    if deviation <= tracking_floor {
                        continue;
                    }
                    let (surface_parameter, gap) = insert(fraction, parameter, on_curve)?;
                    off_surface = off_surface.max(gap);
                    if let Some(surface_parameter) = surface_parameter {
                        // The raised pass stops where the pcurve TRACKS the
                        // edge curve's projection to the floor — the best any
                        // pcurve can do at this station. Not `deviation - gap`:
                        // a surface point at tangential distance d from the
                        // projection of a point g off the surface is only
                        // d^2/(2g) further from it, so the deviation test is
                        // quadratically blind to sideways error on off-surface
                        // input (6e-6 of slop at g = 1.9e-4, floor 1e-7).
                        if raised || track_projection {
                            let projected =
                                surface.evaluate(surface_parameter[0], surface_parameter[1])?;
                            if on_surface.sub(projected).length() <= tracking_floor {
                                continue;
                            }
                        }
                        inserts.push((index + 1, fraction, surface_parameter));
                    }
                }
            }
            if inserts.is_empty() {
                // Nothing to insert: every probed station is within the floor
                // (converged — unless a narrow span was skipped, in which case
                // the sweep below decides), or the ones over it could not be
                // improved (stalled — the edge curve is off the surface there).
                exit = if truncated {
                    PcurveFitExit::SampleCeiling
                } else if round_max > floor {
                    PcurveFitExit::Stalled
                } else if skipped_narrow {
                    PcurveFitExit::SpanFloor
                } else {
                    residual = Some(round_max);
                    PcurveFitExit::Converged
                };
                break;
            }
            for (at, fraction, surface_parameter) in inserts.into_iter().rev() {
                parameters.insert(at, fraction);
                raw.insert(at, surface_parameter);
            }
            let (rebuilt, excursion, distance) = build_interpolant_joined(surface, raw, parameters, joins)?;
            *pcurve = rebuilt;
            clamped = [excursion, distance];
            if truncated {
                exit = PcurveFitExit::SampleCeiling;
                break;
            }
        }
        let residual = match residual {
            Some(measured) => measured,
            // A budget exit never re-checked the curve it is returning, and a
            // converged round that skipped narrow spans never checked those:
            // measure every span.
            None => probe_residual(
                surface,
                curve,
                pcurve,
                parameters,
                [t0, t1],
                [u0, u1],
                [v0, v1],
            )?,
        };
        if exit == PcurveFitExit::SpanFloor && residual <= floor {
            exit = PcurveFitExit::Converged;
        }
        // The per-trim budget decision. Deterministic, grow-once, capped (I3):
        // the ceiling bound and the returned curve is worse than the edge
        // curve's own gap from the surface by more than the floor, so samples
        // can still buy accuracy. A curve already at its gap stops here —
        // reported, not retried.
        // A TRACKING pass for a standing fit that missed the floor (and only
        // then) also takes the same one-time raise when it hits the ceiling
        // still over the floor: its request is the projection's tracking, and
        // the ceiling cut it short. The raised maximum itself is unchanged.
        // The added raise serves a TRACKING pass for an unmet first pass whose
        // RETURNED curve still misses the pass's own target: its tracking (the
        // image against the projected foot, read natively on its own knot spans
        // and endpoints) over the tracking floor (floor x fraction). Reaching
        // the ceiling alone is exhaustion, not a miss, and the inherited
        // standoff never triggers it. Read only at a ceiling exit not yet
        // raised; an unreadable read does not raise.
        let unmet_tracking = track_projection
            && raise_unmet_tracking
            && exit == PcurveFitExit::SampleCeiling
            && !raised
            && native_probe_fractions(&[&*pcurve])
                .and_then(|probes| native_read(surface, curve, pcurve, [t0, t1], &probes))
                .is_ok_and(|read| read.tracking > tracking_floor);
        if takes_raise(exit, raised, residual, off_surface, floor, unmet_tracking) {
            raised = true;
            max_samples = PCURVE_RAISED_SAMPLES;
            continue;
        }
        return Ok(Refinement {
            exit,
            residual,
            off_surface,
            raised,
            clamped,
        });
    }
}

/// The exact inverse of an affine carrier's parameterization.
///
/// An affine patch (2x2 net, degree 1 both ways, equal weights) maps `uv` to 3D
/// by an affine map, so the inverse — a 3D point to the `uv` of its orthogonal
/// projection — is affine too. An affine map commutes with the rational basis:
/// sending a curve's control points through it and keeping the weights
/// produces the EXACT image of that curve in parameter space, not a fit of it.
struct AffineInversion {
    origin: Vec3,
    du: Vec3,
    dv: Vec3,
    u0: f64,
    v0: f64,
    uu: f64,
    uv: f64,
    vv: f64,
    determinant: f64,
}

impl AffineInversion {
    /// `None` when `surface` is not affine, or when its two directions are
    /// parallel enough that the inverse is not determined. The caller decides
    /// what that means: a refusal on the whole-curve lane, the sampled lane on
    /// the range lane.
    fn of(surface: &NurbsSurface) -> Result<Option<Self>, String> {
        if !surface.is_affine()? {
            return Ok(None);
        }
        let ([u0, _], [v0, _]) = surface_domains(surface)?;
        let (origin, du, dv) = surface.deriv1(u0, v0)?;
        let uu = du.dot(du);
        let uv = du.dot(dv);
        let vv = dv.dot(dv);
        let determinant = uu * vv - uv * uv;
        if determinant.abs() <= 1e-18 {
            return Ok(None);
        }
        Ok(Some(Self {
            origin,
            du,
            dv,
            u0,
            v0,
            uu,
            uv,
            vv,
            determinant,
        }))
    }

    fn uv_of(&self, point: Vec3) -> [f64; 2] {
        let delta = point.sub(self.origin);
        let along_u = delta.dot(self.du);
        let along_v = delta.dot(self.dv);
        [
            self.u0 + (along_u * self.vv - along_v * self.uv) / self.determinant,
            self.v0 + (along_v * self.uu - along_u * self.uv) / self.determinant,
        ]
    }

    /// The exact image of `curve` in this carrier's parameter space, with the
    /// curve's own degree, knots and weights.
    fn image_of(&self, curve: &NurbsCurve) -> Result<NurbsCurve, String> {
        let control_points = curve
            .control_points
            .iter()
            .map(|control| {
                let [u, v] = self.uv_of(control.point()?);
                Ok(Vec4::from_point(Vec3::new(u, v, 0.0), control.w))
            })
            .collect::<Result<Vec<_>, String>>()?;
        NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
    }
}

/// Build a parameter-space curve for a 3D curve lying on a surface, and say
/// what the fit achieved.
///
/// This mirrors the reference imprint implementation: affine patches map
/// homogeneous control points exactly; general patches use sampled inversion,
/// periodic seam unwrapping, and adaptive 3D residual refinement. The report
/// beside the curve carries the residual the RETURNED curve reaches and why
/// refinement stopped ([`PcurveFitReport`]); the curve itself is what
/// [`build_pcurve_on_surface`] has always returned.
pub fn fit_pcurve_on_surface(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
) -> Result<PcurveFit, String> {
    let [t0, t1] = curve.domain()?;
    if surface.is_affine()? {
        let Some(inversion) = AffineInversion::of(surface)? else {
            return Err("build_pcurve_on_surface: singular affine parameterization".into());
        };
        let pcurve = inversion.image_of(curve)?;
        return Ok(PcurveFit {
            report: PcurveFitReport {
                residual: 0.0,
                floor: PCURVE_REFINEMENT_TOLERANCE,
                samples: pcurve.control_points.len(),
                exit: PcurveFitExit::Converged,
                off_surface: 0.0,
                raised: false,
                clamped_excursion: 0.0,
                clamped_distance: 0.0,
            },
            curve: pcurve,
        });
    }

    let scale = 1.0 + curve.evaluate((t0 + t1) / 2.0)?.length();
    let drop_tolerance = 1e-4 * scale;
    let endpoint_tolerance = 1e-3 * scale;
    let refinement_tolerance = PCURVE_REFINEMENT_TOLERANCE;
    let ([u0, u1], [v0, v1]) = surface_domains(surface)?;
    let invert =
        |fraction: f64| invert_checked(surface, curve.evaluate(t0 + (t1 - t0) * fraction)?);

    let mut parameters = Vec::with_capacity(25);
    let mut raw_surface_parameters = Vec::with_capacity(25);
    // The edge curve's own distance from the surface at every station inverted,
    // accepted or not: the floor below which no pcurve can bring the residual.
    let mut off_surface = 0.0_f64;
    // The edge's own C0 knots must remain corners of its parameter image.
    // Plant those stations before inversion; refinement retains their joins.
    let corners = range_breaks(curve, t0, t1, true);
    let mut stations: Vec<f64> = (0..=24).map(|index| index as f64 / 24.0).collect();
    // Replace nearby uniform seeds, never the native corner itself. Keep
    // endpoints and other native corners even when their spans are short.
    stations.retain(|station| {
        *station == 0.0 || *station == 1.0 || corners.contains(station)
            || !corners.iter().any(|corner| (*station - corner).abs() <= SPAN_FLOOR)
    });
    for &corner in &corners {
        let at = stations.partition_point(|value| *value < corner);
        if stations.get(at) != Some(&corner) {
            stations.insert(at, corner);
        }
    }
    let joins = corners;
    for (index, &fraction) in stations.iter().enumerate() {
        let (parameter, distance) = invert(fraction)?;
        off_surface = off_surface.max(distance);
        if distance > drop_tolerance && index != 0 && index != stations.len() - 1 {
            continue;
        }
        if distance > endpoint_tolerance {
            if std::env::var("BREP_DEBUG_PCURVE").is_ok() {
                let p3 = curve.evaluate(t0 + (t1 - t0) * fraction).ok();
                let p0 = curve.evaluate(t0).ok();
                let p1 = curve.evaluate(t1).ok();
                eprintln!(
                    "PCURVE-FAIL idx={index} frac={fraction} dist={distance} endpt_tol={endpoint_tolerance} scale={scale}\n  curve3D@frac={p3:?}\n  curve3D@t0={p0:?} curve3D@t1={p1:?}\n  surf_domain=u[{u0},{u1}] v[{v0},{v1}] invpar={parameter:?}\n  surf@invpar={:?}",
                    surface.evaluate(parameter[0], parameter[1]).ok()
                );
            }
            return Err(format!(
                "build_pcurve_on_surface: endpoint projection failed (distance={distance})"
            ));
        }
        parameters.push(fraction);
        raw_surface_parameters.push(parameter);
    }

    let (mut pcurve, excursion, distance) =
        build_interpolant_joined(surface, &raw_surface_parameters, &parameters, &joins)?;
    let refinement = refine_pcurve(
        surface,
        curve,
        [t0, t1],
        [u0, u1],
        [v0, v1],
        refinement_tolerance,
        &joins,
        &mut parameters,
        &mut raw_surface_parameters,
        &mut pcurve,
        off_surface,
        [excursion, distance],
        |fraction, _parameter, _on_curve| {
            let (surface_parameter, distance) = invert(fraction)?;
            Ok((
                (distance <= drop_tolerance).then_some(surface_parameter),
                distance,
            ))
        },
    )?;
    Ok(PcurveFit {
        curve: pcurve,
        report: PcurveFitReport {
            residual: refinement.residual,
            floor: refinement_tolerance,
            samples: parameters.len(),
            exit: refinement.exit,
            off_surface: refinement.off_surface,
            raised: refinement.raised,
            clamped_excursion: refinement.clamped[0],
            clamped_distance: refinement.clamped[1],
        },
    })
}

/// [`fit_pcurve_on_surface`] for the callers that only need the curve. The
/// report is not dropped: it is tallied on the open [`PcurveFitScope`], if any,
/// so the operation that owns this fit can say in its diagnostics whether every
/// trim reached the floor.
pub fn build_pcurve_on_surface(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
) -> Result<NurbsCurve, String> {
    let fit = fit_pcurve_on_surface(surface, curve)?;
    record_fit(&fit.report);
    Ok(fit.curve)
}

/// SECTION-PCURVE SEEDED MARCH (t222: cone × ABC 00000327, non-integral genus).
///
/// `build_pcurve_on_surface` inverts every sample by GLOBAL closest point. Where
/// the carrier surface is COMPRESSED / self-overlapping along the row the
/// section rides, the global search aliases interior samples onto a DISTANT
/// preimage sheet, folding the pcurve out past the trim boundary and re-crossing
/// it. The arrangement then splits the shared section at that phantom crossing,
/// while the mate face (whose carrier is not folded there) keeps the section
/// whole — so the two operands' fragments disagree and the section strands
/// one-use (non-integral genus). Ground truth for t222: 00000327 face 496's
/// v≈0.7875 row maps u∈[0.55,0.81] into a ~0.18mm neighbourhood of the section
/// endpoint A; the global pcurve for piece 9 folded out to u=0.806 and back,
/// re-crossing trim edge 349 at v≈0.784, 0.003 below the corner vertex A.
///
/// This marches the interior inversions with each seeded from the PREVIOUS
/// accepted parameters, so a marched sample
/// adopts the nearby branch instead of the momentarily-closest distant sheet.
/// The two ENDPOINT samples keep the deterministic global inversion (they are
/// the piece's shared junction vertices — same reasoning as
/// `repair_branch_jumps`). Additive + fail-soft: the marched chain is used ONLY
/// when it CLOSES onto the global far endpoint (its last interior sample is
/// parameter-adjacent to it, measured against the chain's own median step);
/// otherwise the plain global build is returned byte-for-byte. A single-preimage
/// carrier (the cone side of the same section) marches to the same samples the
/// global search already had. Use the analytic-aware seeded projector for both
/// the march and inserted stations: raw Newton stops at 1e-7 even on an exact
/// cylinder, introducing avoidable trim error. General folded carriers retain
/// their local seeded Newton path; analytic carriers take their closed form.
///
/// Escape hatch: `BREP_SECTION_PCURVE_MARCH=0`.
pub fn fit_pcurve_on_surface_marched(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
) -> Result<PcurveFit, String> {
    let global = fit_pcurve_on_surface(surface, curve)?;
    if std::env::var("BREP_SECTION_PCURVE_MARCH").as_deref() == Ok("0") || surface.is_affine()? {
        return Ok(global);
    }
    let [t0, t1] = curve.domain()?;
    // A closed (ring) section has no distinct endpoints to anchor the march;
    // leave it to the existing path (closed rings are handled elsewhere).
    if curve.evaluate(t0)?.sub(curve.evaluate(t1)?).length() <= 1e-6 {
        return Ok(global);
    }
    const SAMPLES: usize = 24;
    let fractions: Vec<f64> = (0..=SAMPLES).map(|i| i as f64 / SAMPLES as f64).collect();
    let edge: Vec<Vec3> = fractions
        .iter()
        .map(|fraction| curve.evaluate(t0 + (t1 - t0) * fraction))
        .collect::<Result<Vec<_>, _>>()?;
    let scale = 1.0 + curve.evaluate((t0 + t1) / 2.0)?.length();
    let endpoint_tolerance = 1e-3 * scale;
    let (first, first_distance) = invert_checked(surface, edge[0])?;
    let (last, last_distance) = invert_checked(surface, edge[SAMPLES])?;
    if first_distance > endpoint_tolerance || last_distance > endpoint_tolerance {
        return Ok(global);
    }
    // The edge curve's distance from the surface at every station this chain
    // inverts — on the MARCHED branch, which is the one being fitted.
    let mut off_surface = first_distance.max(last_distance);
    let ([u0, u1], [v0, v1]) = surface_domains(surface)?;
    let u_span = (u1 - u0).abs().max(EPSILON);
    let v_span = (v1 - v0).abs().max(EPSILON);
    let (closed_u, closed_v) = surface_closedness(surface)?;
    let norm_step = |a: [f64; 2], b: [f64; 2]| -> f64 {
        let mut du = a[0] - b[0];
        if closed_u {
            while du > 0.5 * u_span {
                du -= u_span;
            }
            while du < -0.5 * u_span {
                du += u_span;
            }
        }
        let mut dv = a[1] - b[1];
        if closed_v {
            while dv > 0.5 * v_span {
                dv -= v_span;
            }
            while dv < -0.5 * v_span {
                dv += v_span;
            }
        }
        ((du / u_span).powi(2) + (dv / v_span).powi(2)).sqrt()
    };
    // Forward seeded march over the interior samples.
    let mut raw = vec![first];
    let mut previous = first;
    for index in 1..SAMPLES {
        let seeded =
            crate::projection::project_point_to_surface_from_seed(surface, edge[index], previous)?;
        let (_, global_distance) = invert_checked(surface, edge[index])?;
        // The seeded footpoint must still sit on the surface — as tight as the
        // fit band or the global answer already had it. If it fell off (the
        // seed led Newton into a valley), abandon the march and keep global.
        if seeded.distance > 4.0 * global_distance + endpoint_tolerance {
            return Ok(global);
        }
        off_surface = off_surface.max(seeded.distance);
        previous = [seeded.u, seeded.v];
        raw.push(previous);
    }
    raw.push(last);
    // CLOSURE GATE: the last marched interior sample must be parameter-adjacent
    // to the global far endpoint — the march stayed on ONE branch the whole way.
    // Measure the endpoint gap against the chain's OWN median step so a genuine
    // long edge is not rejected while a sheet-hop (which leaves a big gap to the
    // endpoint) is.
    let mut steps: Vec<f64> = (1..raw.len())
        .map(|i| norm_step(raw[i], raw[i - 1]))
        .collect();
    let endpoint_gap = steps.pop().unwrap_or(0.0);
    steps.sort_by(f64::total_cmp);
    let median = steps
        .get(steps.len() / 2)
        .copied()
        .unwrap_or(0.0)
        .max(EPSILON);
    if endpoint_gap > (4.0 * median).max(0.05) {
        return Ok(global);
    }
    // Build the interpolant from the marched samples, then refine — re-projecting
    // each insert SEEDED from the interpolant (already on the marched branch), so
    // a mid-refinement global inversion cannot re-alias onto the far sheet.
    let mut parameters = fractions;
    let (mut pcurve, excursion, distance) = build_interpolant(surface, &raw, &parameters)?;
    let refinement_tolerance = PCURVE_REFINEMENT_TOLERANCE;
    // Same classified exits and budget ladder as `fit_pcurve_on_surface`. A
    // seeded insert always lands, so this loop cannot stall.
    let refinement = refine_pcurve(
        surface,
        curve,
        [t0, t1],
        [u0, u1],
        [v0, v1],
        refinement_tolerance,
        &[],
        &mut parameters,
        &mut raw,
        &mut pcurve,
        off_surface,
        [excursion, distance],
        |_fraction, seed, on_curve| {
            let seeded = crate::projection::project_point_to_surface_from_seed(surface, on_curve, [seed.x, seed.y])?;
            Ok((Some([seeded.u, seeded.v]), seeded.distance))
        },
    )?;
    Ok(PcurveFit {
        curve: pcurve,
        report: PcurveFitReport {
            residual: refinement.residual,
            floor: refinement_tolerance,
            samples: parameters.len(),
            exit: refinement.exit,
            off_surface: refinement.off_surface,
            raised: refinement.raised,
            clamped_excursion: refinement.clamped[0],
            clamped_distance: refinement.clamped[1],
        },
    })
}

/// [`fit_pcurve_on_surface_marched`] for the callers that only need the curve;
/// the report is tallied on the open [`PcurveFitScope`] exactly as
/// [`build_pcurve_on_surface`] does.
pub fn build_pcurve_on_surface_marched(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
) -> Result<NurbsCurve, String> {
    let fit = fit_pcurve_on_surface_marched(surface, curve)?;
    record_fit(&fit.report);
    Ok(fit.curve)
}

/// Build a pcurve for a represented subrange of a larger edge curve.
///
/// On an affine carrier the answer is exact and is taken as such
/// ([`affine_pcurve_on_range`]). Otherwise this samples only the represented
/// interval, which is essential when off-interval control points do not lie on
/// the target carrier — the exact lane restricts the net by knot insertion
/// first, so those control points are removed rather than ignored.
pub fn fit_pcurve_on_surface_range(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
) -> Result<PcurveFit, String> {
    if let Some(exact) =
        affine_pcurve_on_range(surface, curve, edge_start, edge_end, forward, tolerance)?
    {
        return Ok(exact);
    }
    fit_pcurve_on_surface_range_dense(
        surface, curve, edge_start, edge_end, forward, tolerance, 64, 3, 513,
    )
}

/// [`fit_pcurve_on_surface_range`] for the callers that only need the curve:
/// the fit is ACCEPTED or REFUSED on its bar ([`accept_fit`]), never returned
/// off it.
pub fn build_pcurve_on_surface_range(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    accept_fit(fit_pcurve_on_surface_range(
        surface, curve, edge_start, edge_end, forward, tolerance,
    )?)
}

/// `BREP_DEBUG_STATION_FIT`: trace every station fit round by round, and the
/// stations around the worst span of a fit off its bar. Read once per process.
fn station_fit_tracing() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var_os("BREP_DEBUG_STATION_FIT").is_some())
}

/// Narrowest span, as a fraction of the parameter range, that
/// [`fit_pcurve_on_surface_stations`] subdivides. `refine_pcurve` floors at
/// 1e-3 from 25 base samples; this fitter starts from 64 and quarters, so a
/// floor of 1e-3 bound on its first pass and refused 67 of the 57-document
/// corpus's bodies (measured 2026-09-26). 1e-6 admits every halving a real
/// trim needs and stops the runaway that fits stations 4e-7 apart.
const SPAN_FLOOR: f64 = 1e-6;

/// Rounds the raised pass of [`fit_pcurve_on_surface_stations`] may spend:
/// enough halvings for any span the sample cap can resolve.
const RAISED_ROUNDS: usize = 40;

/// The refusal every station builder returns for a fit off its bar.
pub const PCURVE_OFF_BAR: &str = "pcurve fit refused: the trim misses its stations by";

/// The one acceptance decision behind [`build_pcurve_on_surface_range`],
/// [`build_pcurve_on_surface_range_dense`] and
/// [`build_pcurve_on_surface_stations`]: a fit on its bar
/// ([`PcurveFitReport::on_bar`]) is the curve, a fit off it is a refusal that
/// names the residual, the bar, what was asked, how far the stations stand off
/// the carrier and why refinement stopped.
///
/// Until 2026-09-26 these builders returned whatever their last round built
/// without measuring it (the 2026-09-18 census: 4564 of 54556 imported pcurves
/// over the ask they were built to, 2072 of them at a station the refinement
/// itself had tested and given up on). Nothing here widens a band: the bar is
/// the caller's own ask plus what the input makes unreachable.
fn accept_fit(fit: PcurveFit) -> Result<NurbsCurve, String> {
    let report = fit.report;
    if report.on_bar() {
        return Ok(fit.curve);
    }
    Err(off_bar_message(&report))
}

fn off_bar_message(report: &PcurveFitReport) -> String {
    format!(
        "{PCURVE_OFF_BAR} {:.3e} out of sample against a bar of {:.3e} (asked {:.1e}, the \
         stations stand {:.3e} off the carrier; {:?} at {} samples{}{})",
        report.residual,
        report.floor + report.off_surface,
        report.floor,
        report.off_surface,
        report.exit,
        report.samples,
        if report.raised { ", after the raise" } else { "" },
        if report.clamped_excursion > 0.0 {
            format!(
                "; clamped into the chart by {:.3e} in parameter, {:.3e} in 3D",
                report.clamped_excursion, report.clamped_distance
            )
        } else {
            String::new()
        },
    )
}

/// The class slug of a pcurve fit refused off its bar
/// ([`crate::RefusalClass::NonConvergence`]): minted by
/// [`build_pcurve_on_surface_range_classed`] and read by
/// [`is_pcurve_off_bar`], so a dispatcher decides on the class, never on the
/// message text.
pub(crate) const PCURVE_OFF_BAR_WHAT: &str = "pcurve_fit_off_bar";

/// Whether `refusal` is a pcurve fit that could not reach its bar.
pub(crate) fn is_pcurve_off_bar(refusal: &crate::KernelRefusal) -> bool {
    matches!(&refusal.class, crate::RefusalClass::NonConvergence { what } if what == PCURVE_OFF_BAR_WHAT)
}

/// [`build_pcurve_on_surface_range`] with a typed refusal: a fit off its bar
/// is `NonConvergence { what: PCURVE_OFF_BAR_WHAT }` at `stage`, with the same
/// message; any other failure of the fit is `Internal` under `what`.
pub(crate) fn build_pcurve_on_surface_range_classed(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
    stage: KernelStage,
    what: &'static str,
) -> Result<NurbsCurve, crate::KernelRefusal> {
    use crate::OrRefuse;
    let fit = fit_pcurve_on_surface_range(surface, curve, edge_start, edge_end, forward, tolerance)
        .or_refuse(stage, what)?;
    accept_fit_classed(fit, stage)
}

/// [`accept_fit`] with the refusal typed: `NonConvergence { what:
/// PCURVE_OFF_BAR_WHAT }` at `stage`, the message unchanged.
pub(crate) fn accept_fit_classed(fit: PcurveFit, stage: KernelStage) -> Result<NurbsCurve, crate::KernelRefusal> {
    if fit.report.on_bar() {
        return Ok(fit.curve);
    }
    Err(crate::KernelRefusal {
        class: crate::RefusalClass::NonConvergence { what: PCURVE_OFF_BAR_WHAT.into() },
        stage,
        message: off_bar_message(&fit.report),
    })
}

/// The EXACT pcurve for a coedge's subrange of `curve` on an affine carrier,
/// or `None` when there is no such answer and the sampled lane must run.
///
/// [`fit_pcurve_on_surface`] has always mapped a WHOLE curve's control net
/// exactly on an affine carrier. A coedge's trim is that same curve restricted
/// to `[edge_start, edge_end]` and oriented by `forward`, and both operations
/// are exact on a NURBS: [`NurbsCurve::split`] subdivides by knot insertion,
/// [`NurbsCurve::reversed`] mirrors the net, and rescaling the knots to
/// `[0, 1]` is affine in the edge's own parameter — which is precisely the
/// fraction convention the sampled lane uses, so the pcurve's correspondence
/// with the edge is the same one either lane produces.
///
/// Sampling where an exact answer exists leaves a real error: measured
/// 2026-09-17, a booleaned band rim's two planar trims sat 2.379e-4 and
/// 2.395e-4 off their edges, where this lane lands 1.4e-13 off.
///
/// The image is verified against the caller's own `tolerance` before it is
/// returned. The exact image of a curve that does NOT lie on this carrier is
/// its orthogonal projection, which is not that coedge's trim, so every such
/// case falls through to the sampled lane and keeps the refusals that lane
/// makes today.
fn affine_pcurve_on_range(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
) -> Result<Option<PcurveFit>, String> {
    // Opt-out for A/B bisection of this lane against the sampled one, in the
    // shape of the two hatches beside it (`BREP_NO_PCURVE_REPAIR`,
    // `BREP_SECTION_PCURVE_MARCH`). It is what showed that the boolean's
    // planar residuals do NOT come through here: the same binary with this set
    // and unset read identically to every digit on 21_band_rim.
    if std::env::var("BREP_AFFINE_RANGE_PCURVE").as_deref() == Ok("0") {
        return Ok(None);
    }
    let Some(inversion) = AffineInversion::of(surface)? else {
        return Ok(None);
    };
    if !(edge_start.is_finite() && edge_end.is_finite() && edge_start < edge_end) {
        // The sampled lane owns this refusal; it is not this lane's to reword.
        return Ok(None);
    }
    let [t0, t1] = curve.domain()?;
    let start = edge_start.max(t0);
    let end = edge_end.min(t1);
    // A range narrower than the knot identity tolerance cannot be split onto
    // its own endpoints at all.
    if end - start <= 2.0 * KNOT_IDENTITY_TOL {
        return Ok(None);
    }
    let mut restricted = curve.clone();
    if end < t1 - KNOT_IDENTITY_TOL {
        restricted = restricted.split(end)?.0;
    }
    if start > t0 + KNOT_IDENTITY_TOL {
        restricted = restricted.split(start)?.1;
    }
    let mut pcurve = inversion.image_of(&restricted)?;
    if !forward {
        pcurve = pcurve.reversed()?;
    }
    let first = pcurve.knots[0];
    let last = pcurve.knots[pcurve.knots.len() - 1];
    if !(last - first > 0.0) {
        return Ok(None);
    }
    let knots = pcurve
        .knots
        .iter()
        .map(|knot| (knot - first) / (last - first))
        .collect::<Vec<_>>();
    let pcurve = NurbsCurve::new(pcurve.degree, knots, pcurve.control_points.clone())?;

    // Verify against the range the CALLER asked for, not the one the
    // restriction produced: `split` snaps onto a nearby knot, so a snap that
    // moved the correspondence reads here as a deviation. Sampling at
    // fractions that are not the stations any lane interpolates through keeps
    // this a check on the whole trim rather than on its endpoints.
    let scale = 1.0 + curve.evaluate((start + end) / 2.0)?.length();
    let bar = tolerance.max(1e-11 * scale);
    let mut residual = 0.0_f64;
    for index in 0..=32 {
        let fraction = index as f64 / 32.0;
        let uv = pcurve.evaluate(fraction)?;
        // `evaluate` CLAMPS out-of-domain uv. A planar carrier's domain is a
        // bounding rectangle that a booleaned trim can leave, and an affine
        // patch's extension is the same plane, so read it unclamped.
        let on_carrier = surface.evaluate_extended(uv.x, uv.y)?;
        let edge_fraction = if forward { fraction } else { 1.0 - fraction };
        let on_edge = curve.evaluate(start + (end - start) * edge_fraction)?;
        residual = residual.max(on_carrier.sub(on_edge).length());
        if residual > bar {
            return Ok(None);
        }
    }
    let samples = pcurve.control_points.len();
    Ok(Some(PcurveFit {
        curve: pcurve,
        report: PcurveFitReport {
            residual,
            // The floor this lane VERIFIED against: the caller's ask, or the
            // floating-point floor of an exact map on a part this size. An
            // exact image is on its bar by construction; a caller asking for
            // less than the map's own precision is not refused for it.
            floor: bar,
            samples,
            exit: PcurveFitExit::Converged,
            off_surface: 0.0,
            raised: false,
            clamped_excursion: 0.0,
            clamped_distance: 0.0,
        },
    }))
}

fn align_collapsed_endpoints(surface: &NurbsSurface, raw: &mut [[f64; 2]]) -> Result<(), String> {
    let anchor = surface.control_points[0][0].point()?;
    let mut extent = 0.0_f64;
    for control in surface.control_points.iter().flatten() {
        extent = extent.max(control.point()?.sub(anchor).length());
    }
    // This is a geometric identity check, independent of the fitting allowance
    // and of world position. Positive rational weights keep the whole iso-curve
    // inside the convex hull of its Euclidean controls. Cap the size coupling
    // at the linear tolerance so a large patch cannot erase a thin finite row.
    let pole_band = (1e-10 * (1.0 + extent)).min(LINEAR_TOLERANCE);
    for (endpoint, neighbor) in [(0, 1), (raw.len() - 1, raw.len() - 2)] {
        let point = surface.evaluate(raw[endpoint][0], raw[endpoint][1])?;
        for axis in 0..2 {
            let mut candidate = raw[endpoint];
            candidate[axis] = raw[neighbor][axis];
            if candidate == raw[endpoint]
                || surface
                    .evaluate(candidate[0], candidate[1])?
                    .sub(point)
                    .length()
                    > pole_band
            {
                continue;
            }
            let row = if axis == 0 {
                surface.iso_curve_v(raw[endpoint][1])?
            } else {
                surface.iso_curve_u(raw[endpoint][0])?
            };
            let mut collapsed = true;
            for control in &row.control_points {
                if control.point()?.sub(point).length() > pole_band {
                    collapsed = false;
                    break;
                }
            }
            if collapsed {
                raw[endpoint] = candidate;
            }
        }
    }
    Ok(())
}

/// Re-seat any raw inversion sample that BRANCH-JUMPED — snapped to a distant
/// fold of a self-overlapping general carrier — back onto the branch traced by
/// its neighbours.
///
/// Each entry in `raw` is an INDEPENDENT global closest-point inversion of the
/// corresponding 3D edge sample. Every one is geometrically valid (on the
/// surface, at the edge), but they need not be CONTIGUOUS: where a rational
/// B-spline surface folds back over the small trimmed patch it carries, the
/// momentarily-closest fold flips from one sample to the next, so the raw
/// polygon zig-zags across the whole domain and self-crosses — and the region
/// its interpolant bounds no longer covers the true face (its surface flux can
/// exceed its own area). We anchor on the LONGEST run of mutually-continuous
/// samples — the fold the trimmed patch actually lies on, since a small face
/// lives on ONE fold and the jumped samples are the minority the global search
/// snapped elsewhere — then walk outward in both directions, re-seeding Newton
/// from the last good parameters and adopting the continuous footpoint whenever
/// it is geometrically as valid as the global one. Anchoring on consensus (not
/// blindly on sample 0, which can itself be an outlier that would drag the
/// whole edge onto the wrong branch and tear the loop open at its endpoint)
/// leaves continuous nonsingular fits unchanged. Certified collapsed endpoint
/// rows first adopt their neighbour's branch; a bad seed in the subsequent
/// jump repair cannot make a sample worse than the global search already had it.
fn repair_branch_jumps(
    surface: &NurbsSurface,
    edge_points: &[Vec3],
    raw: &mut [[f64; 2]],
    tolerance: f64,
) -> Result<(), String> {
    if raw.len() < 3 {
        return Ok(());
    }
    // Affine carriers invert linearly and exactly — no folds, nothing to chase.
    // Checked first so the common planar/affine face never touches the env.
    if surface.is_affine()? {
        return Ok(());
    }
    // Opt-out for A/B bisection of a STEP-import regression against this repair.
    if std::env::var("BREP_NO_PCURVE_REPAIR").is_ok() {
        return Ok(());
    }
    // A pole has no unique coordinate along its collapsed row. Continuing the
    // adjacent sample's branch is safe only when the ENTIRE row is collapsed;
    // coincident points across an ordinary seam or fold are not enough.
    align_collapsed_endpoints(surface, raw)?;
    let ([u0, u1], [v0, v1]) = surface_domains(surface)?;
    let u_span = (u1 - u0).abs().max(EPSILON);
    let v_span = (v1 - v0).abs().max(EPSILON);
    let (closed_u, closed_v) = surface_closedness(surface)?;
    // Normalised, seam-aware step between two parameter samples. Wrapping the
    // closed directions keeps a legitimate seam crossing SMALL, so the seam /
    // periodic-unwrap machinery elsewhere is never disturbed by this repair.
    let norm_step = |a: [f64; 2], b: [f64; 2]| -> f64 {
        let mut du = a[0] - b[0];
        if closed_u {
            while du > 0.5 * u_span {
                du -= u_span;
            }
            while du < -0.5 * u_span {
                du += u_span;
            }
        }
        let mut dv = a[1] - b[1];
        if closed_v {
            while dv > 0.5 * v_span {
                dv -= v_span;
            }
            while dv < -0.5 * v_span {
                dv += v_span;
            }
        }
        ((du / u_span).powi(2) + (dv / v_span).powi(2)).sqrt()
    };
    // A jump crosses a large fraction of the WHOLE domain — orders above the
    // per-sample motion of any real (even domain-spanning) edge, which advances
    // ~1/N of its traversal between consecutive stations.
    const JUMP_THRESHOLD: f64 = 0.2;
    let count = raw.len();

    // Locate the longest maximal run of consecutive continuous samples — the
    // branch to anchor on. If nothing jumps, this spans [0, count-1] and both
    // re-seat passes below are empty, leaving `raw` untouched.
    let (mut best_start, mut best_len, mut run_start) = (0usize, 1usize, 0usize);
    for index in 1..count {
        if norm_step(raw[index], raw[index - 1]) > JUMP_THRESHOLD {
            if index - run_start > best_len {
                best_len = index - run_start;
                best_start = run_start;
            }
            run_start = index;
        }
    }
    if count - run_start > best_len {
        best_len = count - run_start;
        best_start = run_start;
    }
    let (spine_lo, spine_hi) = (best_start, best_start + best_len - 1);

    // Adopt the continuous footpoint only when it (a) still sits on the surface
    // — as tight as the fit band or the global answer — and (b) genuinely
    // closes the jump rather than trading it for another. Returns the parameter
    // to carry forward as the next seed (the repaired one, or the untouched
    // global when no repair applies).
    let reseat = |edge_point: Vec3,
                  previous: [f64; 2],
                  global: [f64; 2]|
     -> Result<[f64; 2], String> {
        let global_step = norm_step(global, previous);
        if global_step <= JUMP_THRESHOLD {
            return Ok(global);
        }
        let seeded =
            project_point_to_surface_seeded(surface, edge_point, previous[0], previous[1])?;
        let candidate = [seeded.u, seeded.v];
        let global_residual = surface
            .evaluate(global[0], global[1])?
            .sub(edge_point)
            .length();
        let on_surface = seeded.distance <= 4.0 * global_residual + tolerance.max(1e-12);
        if on_surface && norm_step(candidate, previous) < 0.5 * global_step {
            if std::env::var("BREP_DEBUG_PCURVE").is_ok() {
                eprintln!(
                    "pcurve repair: jump {global_step:.4} ({global:?}) -> {:.4} ({candidate:?}) res {:.2e}->{:.2e}",
                    norm_step(candidate, previous),
                    global_residual,
                    seeded.distance
                );
            }
            Ok(candidate)
        } else {
            Ok(global)
        }
    };

    // Walk forward off the spine's high end, then backward off its low end,
    // chaining each repaired sample as the next seed so continuity propagates.
    //
    // Apart from the certified collapsed rows above, the two ENDPOINT samples
    // (fraction 0 and 1) are NEVER moved: they are the
    // edge's shared loop vertices. The global inversion is deterministic, so the
    // two coedges meeting at a vertex land it at the SAME parameters even when
    // that vertex sits on a fold reachable from two branches — moving one side
    // to a different branch tears the loop open there (`synthesize_pole_edge`
    // then rejects the non-collapsed gap). A vertex's genuine fold transition
    // (an edge whose interior rides u≈0.08 but whose endpoint must meet its
    // neighbour at u≈0.95) is exactly this case and must be preserved, not
    // "continuity-repaired" back onto the interior branch.
    let mut previous = raw[spine_hi];
    for index in (spine_hi + 1)..count.saturating_sub(1) {
        raw[index] = reseat(edge_points[index], previous, raw[index])?;
        previous = raw[index];
    }
    let mut previous = raw[spine_lo];
    for index in (1..spine_lo).rev() {
        raw[index] = reseat(edge_points[index], previous, raw[index])?;
        previous = raw[index];
    }
    Ok(())
}

/// Range fitter with explicit sampling knobs, reporting what the fit achieved.
/// [`fit_pcurve_on_surface_range`] keeps the long-standing (base 64, 3
/// refinement rounds, 513 cap) budget; a fit that binds at that cap with the
/// bar unmet grows once to [`PCURVE_RAISED_SAMPLES`]
/// ([`fit_pcurve_on_surface_stations`]).
///
/// NOTE (2026-09-03): this comment used to say "STEP import retries failed fits
/// with a denser budget". No caller passes these knobs any more — that retry
/// was removed and the sentence outlived it. The knobs are kept because they
/// are the honest way to ASK whether a fit had headroom left, which is what
/// `examples/pcurve_residual_probe.rs` uses them for: on every ABC corpus
/// coedge that failed `validate`'s pcurve check, budgets up to (512, 8, 8193)
/// with a 1000x tighter target reproduced the coarse fit's deviation to six
/// decimals, because that deviation is the edge-vs-surface closest-point
/// residual and a curve ON the surface cannot beat it.
#[allow(clippy::too_many_arguments)]
pub fn fit_pcurve_on_surface_range_dense(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
) -> Result<PcurveFit, String> {
    if !(edge_start.is_finite() && edge_end.is_finite() && edge_start < edge_end) {
        return Err("build_pcurve_on_surface_range: invalid edge interval".into());
    }
    let evaluate_edge = |fraction: f64| {
        let edge_fraction = if forward { fraction } else { 1.0 - fraction };
        curve.evaluate(edge_start + (edge_end - edge_start) * edge_fraction)
    };
    let breaks = range_breaks(curve, edge_start, edge_end, forward);
    let ask = tolerance.max(PCURVE_REFINEMENT_TOLERANCE);
    let fit = fit_station_pcurve(
        surface,
        &evaluate_edge,
        &breaks,
        &breaks,
        ask,
        base_samples,
        refinement_rounds,
        parameter_cap,
        SPAN_FLOOR,
    )?;
    end_span_challenger(surface, curve, &evaluate_edge, edge_start, edge_end, forward, &breaks, ask, base_samples, refinement_rounds, parameter_cap, fit)
}

/// The edge curve's OWN distinct interior knots inside `[edge_start,
/// edge_end]`, mapped through the trim's range and sense to coedge
/// fractions, that fall inside `pcurve`'s FIRST or LAST span (between the
/// domain end and its nearest interior knot), sorted.
fn native_end_knot_fractions(curve: &NurbsCurve, edge_start: f64, edge_end: f64, forward: bool, pcurve: &NurbsCurve) -> Result<Vec<f64>, String> {
    let [q0, q1] = pcurve.domain()?;
    let mut interior: Vec<f64> = pcurve.knots.iter().copied().filter(|k| *k > q0 && *k < q1).collect();
    interior.dedup();
    let (Some(&first), Some(&last)) = (interior.first(), interior.last()) else { return Ok(Vec::new()) };
    let (low, high) = (edge_start.min(edge_end), edge_start.max(edge_end));
    let span = edge_end - edge_start;
    let mut fractions: Vec<f64> = curve
        .knots
        .iter()
        .copied()
        .filter(|k| *k > low && *k < high)
        .map(|k| {
            let along = (k - edge_start) / span;
            if forward { along } else { 1.0 - along }
        })
        .filter(|f| f.is_finite() && ((*f > q0 && *f < first) || (*f > last && *f < q1)))
        .collect();
    fractions.sort_by(f64::total_cmp);
    fractions.dedup();
    Ok(fractions)
}

/// The admission rule for a foot read against an edge point: finite distance,
/// finite point, finite (u, v) inside the native domain. Shared by
/// [`common_station_read`] and the station fitter's tracking mode.
fn foot_admitted(foot: &crate::SurfaceProjection, [u0, u1]: [f64; 2], [v0, v1]: [f64; 2]) -> bool {
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    foot.distance.is_finite() && finite(foot.point) && foot.u.is_finite() && foot.v.is_finite() && (u0..=u1).contains(&foot.u) && (v0..=v1).contains(&foot.v)
}

/// The tracking mode's verification probes: the current pcurve's native probes
/// ([`native_probe_fractions`]: endpoints, quarters and 16 interior points of
/// each knot span), its EXACT knots (the final common read includes them, and
/// the native probes do not), and the caller's fixed fractions; finite, in
/// [0, 1], sorted, deduplicated.
fn verification_fractions(pcurve: &NurbsCurve, verify: &[f64]) -> Result<Vec<f64>, String> {
    let mut fractions = native_probe_fractions(&[pcurve])?;
    fractions.extend(pcurve.knots.iter().copied());
    fractions.extend(verify.iter().copied());
    fractions.retain(|f| f.is_finite() && (0.0..=1.0).contains(f));
    fractions.sort_by(f64::total_cmp);
    fractions.dedup();
    Ok(fractions)
}

/// The off-carrier candidate admission, at the same probes and shared feet
/// (`[tracking, deviation, excess]` per curve, [`common_station_read_with_excess`]):
/// every reading finite, the candidate's tracking strictly lower, and EITHER
/// its maximum deviation no worse than the legacy's (the original rule) OR the
/// candidate meeting the ORIGINAL floors outright, tracking and excess both
/// within `PCURVE_REFINEMENT_TOLERANCE` (1e-7).
///
/// Why the second branch: for fits whose images follow the shared admitted
/// feet, the maximum deviation can be dominated by the edge's standoff.
/// Their measured maxima can then differ by rounding while tracking improves.
/// This does not assert that the admitted foot is a certified global nearest
/// point or that an arbitrary pcurve has the same maximum deviation.
/// next5 fixture 25 (BREP_DEBUG_STATION_FIT): every tracking candidate reached
/// the floor (9.52e-8 .. 9.94e-8 tracking) and was declined on a deviation
/// 1.2e-14 .. 6.7e-13 above the legacy's at 2.1e-5 .. 9.0e-4 standoffs, while
/// the legacy kept 5.9e-7 .. 8.5e-5 tracking. A candidate within the original
/// floors on both readings is what those controls require; it is not a
/// standoff allowance and no bar moves.
pub(crate) fn floor_admits(old: [f64; 3], new: [f64; 3]) -> bool {
    let floor = PCURVE_REFINEMENT_TOLERANCE;
    old.iter().chain(new.iter()).all(|v| v.is_finite())
        && new[0] < old[0]
        && (new[1] <= old[1] || (new[0] <= floor && new[2] <= floor))
}

/// The common reader's foot: the NEAREST admitted ([`foot_admitted`]) of the
/// global projection, the projection seeded at the reference (legacy) uv and
/// the one seeded at the candidate (image) uv, in THAT order, so that between
/// admitted feet at equal distance the first in this order is taken
/// (`min_by` keeps the first minimum). [`common_station_read`] and the station
/// fitter's tracking mode both take their foot here, so an equal-distance tie
/// resolves to the same foot in both.
fn common_foot(
    global: Option<crate::SurfaceProjection>,
    reference: Option<crate::SurfaceProjection>,
    candidate: Option<crate::SurfaceProjection>,
    domain_u: [f64; 2],
    domain_v: [f64; 2],
) -> Option<crate::SurfaceProjection> {
    [global, reference, candidate]
        .into_iter()
        .flatten()
        .filter(|f| foot_admitted(f, domain_u, domain_v))
        .min_by(|a, b| a.distance.total_cmp(&b.distance))
}

/// The probes a RANGE trim is read at by the common reader: `curves`'
/// native spans (endpoints, quarters and 16 interior points of each knot
/// span) and exact knots, plus the edge curve's own knots CLIPPED to
/// `[t0, t1]` (with the range ends) and the same 19 points on every clipped
/// span, mapped through the coedge's sense to fractions; finite, in [0, 1],
/// sorted, deduplicated.
pub(crate) fn range_use_probes(curves: &[&NurbsCurve], edge_curve: &NurbsCurve, t0: f64, t1: f64, forward: bool) -> Result<Vec<f64>, String> {
    let mut probes = native_probe_fractions(curves)?;
    for curve in curves {
        probes.extend(curve.knots.iter().copied());
    }
    let span = t1 - t0;
    if span.is_finite() && span != 0.0 {
        let (low, high) = (t0.min(t1), t0.max(t1));
        let mut knots: Vec<f64> = edge_curve.knots.iter().copied().filter(|k| *k >= low && *k <= high).collect();
        knots.push(low);
        knots.push(high);
        knots.sort_by(f64::total_cmp);
        knots.dedup();
        let locals: Vec<f64> = [0.25, 0.5, 0.75]
            .into_iter()
            .chain((1..=TRACKING_ACCEPTANCE_POINTS).map(|index| index as f64 / (TRACKING_ACCEPTANCE_POINTS + 1) as f64))
            .collect();
        let mut ts = knots.clone();
        for pair in knots.windows(2).filter(|w| w[1] > w[0]) {
            for &local in &locals {
                ts.push(pair[0] + (pair[1] - pair[0]) * local);
            }
        }
        for t in ts {
            let along = (t - t0) / span;
            probes.push(if forward { along } else { 1.0 - along });
        }
    }
    probes.retain(|f| f.is_finite() && (0.0..=1.0).contains(f));
    probes.sort_by(f64::total_cmp);
    probes.dedup();
    Ok(probes)
}

/// [`common_station_read`] for two RANGE trims of one coedge (`old`, `new`,
/// both on the coedge fraction domain [0, 1]) against the edge curve over
/// `[t0, t1]` in the coedge's sense, at [`range_use_probes`] of both: the same
/// probes and one shared admitted foot per probe, finite or an error. A trim
/// not on [0, 1] is an error.
// Unused by the import lane since the tracking trigger and keep were withdrawn
// for this merge; kept, with its controls, for their re-proposal.
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn range_common_read(
    surface: &NurbsSurface,
    edge_curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    old: &NurbsCurve,
    new: &NurbsCurve,
) -> Result<((f64, f64), (f64, f64)), String> {
    let (o, n) = range_common_read_with_excess(surface, edge_curve, t0, t1, forward, old, new)?;
    Ok(((o[0], o[1]), (n[0], n[1])))
}

/// [`range_common_read`] with each trim's maximum pointwise excess over the
/// shared admitted foot's own distance: `[tracking, deviation, excess]` per
/// trim ([`common_station_read_with_excess`]).
// Unused by the import lane since the tracking trigger and keep were withdrawn
// for this merge; kept, with its controls, for their re-proposal.
#[cfg_attr(not(test), allow(dead_code))]
pub(crate) fn range_common_read_with_excess(
    surface: &NurbsSurface,
    edge_curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    forward: bool,
    old: &NurbsCurve,
    new: &NurbsCurve,
) -> Result<([f64; 3], [f64; 3]), String> {
    for curve in [old, new] {
        if !matches!(curve.domain(), Ok([a, b]) if a == 0.0 && b == 1.0) {
            return Err("range common read: a trim not on [0, 1]".into());
        }
    }
    let probes = range_use_probes(&[old, new], edge_curve, t0, t1, forward)?;
    let evaluate = |fraction: f64| -> Result<Vec3, String> {
        let along = if forward { fraction } else { 1.0 - fraction };
        edge_curve.evaluate(t0 + (t1 - t0) * along)
    };
    common_station_read_with_excess(surface, &evaluate, old, new, &probes)
}

/// The legacy and candidate pcurves read against the edge at the SAME probes
/// and against the SAME foot per probe: at each fraction, the edge point's
/// foot is the NEAREST admitted one among the global projection and the
/// projections seeded at the legacy and the candidate image's (u, v). A foot
/// is admitted with a finite distance and point and a finite (u, v) inside the
/// native domain. Both images are then measured against that one shared foot
/// (TRACKING) and against the edge point (same-parameter DEVIATION). The
/// shared foot is the nearest of these three, not a certified global nearest.
/// Any non-finite image, point or distance, or a probe with no admitted foot,
/// is an error (the caller keeps the legacy fit). Returns
/// ((legacy tracking, legacy deviation), (candidate tracking, candidate
/// deviation)), maxima over the probes.
pub(crate) fn common_station_read(
    surface: &NurbsSurface,
    evaluate_edge: &dyn Fn(f64) -> Result<Vec3, String>,
    legacy: &NurbsCurve,
    candidate: &NurbsCurve,
    probes: &[f64],
) -> Result<((f64, f64), (f64, f64)), String> {
    let (old, new) = common_station_read_with_excess(surface, evaluate_edge, legacy, candidate, probes)?;
    Ok(((old[0], old[1]), (new[0], new[1])))
}

/// [`common_station_read`] that also returns, per curve, the maximum
/// pointwise EXCESS: the same-parameter deviation minus the shared admitted
/// foot's own distance at that probe (the part over the edge's standoff
/// there, which the every-use controls bar at the original 1e-7). Returns
/// `[tracking, deviation, excess]` for the legacy and the candidate.
pub(crate) fn common_station_read_with_excess(
    surface: &NurbsSurface,
    evaluate_edge: &dyn Fn(f64) -> Result<Vec3, String>,
    legacy: &NurbsCurve,
    candidate: &NurbsCurve,
    probes: &[f64],
) -> Result<([f64; 3], [f64; 3]), String> {
    let (domain_u, domain_v) = surface_domains(surface)?;
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    // [tracking, deviation, excess] per curve; the excess (deviation minus
    // the shared foot's distance, per probe) starts at -inf, so a read with no
    // probe is not finite and admits nothing.
    let mut read = ([0.0_f64, 0.0, f64::NEG_INFINITY], [0.0_f64, 0.0, f64::NEG_INFINITY]);
    for &fraction in probes {
        let point = evaluate_edge(fraction)?;
        let (old_uv, new_uv) = (legacy.evaluate(fraction)?, candidate.evaluate(fraction)?);
        let (old_image, new_image) = (surface.evaluate_extended(old_uv.x, old_uv.y)?, surface.evaluate_extended(new_uv.x, new_uv.y)?);
        if !(finite(point) && finite(old_image) && finite(new_image)) {
            return Err(format!("common station read: non-finite point at fraction {fraction}"));
        }
        // Reading ONE curve against itself (`legacy` and `candidate` the same
        // object, as a trigger's read is): the candidate-seeded projection is
        // the reference-seeded one on identical inputs, so it is computed once
        // (same result, same tie order).
        let reference_foot = memo_seeded_foot(surface, point, old_uv.x, old_uv.y);
        let candidate_foot = if std::ptr::eq(legacy, candidate) {
            reference_foot
        } else {
            memo_seeded_foot(surface, point, new_uv.x, new_uv.y)
        };
        let foot = common_foot(
            memo_global_foot(surface, point),
            reference_foot,
            candidate_foot,
            domain_u,
            domain_v,
        )
        .ok_or_else(|| format!("common station read: no admitted foot at fraction {fraction}"))?;
        let values = [
            old_image.sub(foot.point).length(),
            old_image.sub(point).length(),
            new_image.sub(foot.point).length(),
            new_image.sub(point).length(),
        ];
        if values.iter().any(|v| !v.is_finite()) {
            return Err(format!("common station read: non-finite distance at fraction {fraction}"));
        }
        if !foot.distance.is_finite() {
            return Err(format!("common station read: non-finite foot distance at fraction {fraction}"));
        }
        read.0[0] = read.0[0].max(values[0]);
        read.0[1] = read.0[1].max(values[1]);
        read.0[2] = read.0[2].max(values[1] - foot.distance);
        read.1[0] = read.1[0].max(values[2]);
        read.1[1] = read.1[1].max(values[3]);
        read.1[2] = read.1[2].max(values[3] - foot.distance);
    }
    Ok(read)
}

/// The interior probe fractions of the edge curve's own FIRST and LAST native
/// knot spans inside `[edge_start, edge_end]` (quarter points and 16 interior
/// points of each), mapped through the trim's range and sense to coedge
/// fractions in [0, 1].
fn edge_end_span_fractions(curve: &NurbsCurve, edge_start: f64, edge_end: f64, forward: bool) -> Vec<f64> {
    let (low, high) = (edge_start.min(edge_end), edge_start.max(edge_end));
    let mut knots: Vec<f64> = curve.knots.iter().copied().filter(|k| *k > low && *k < high).collect();
    knots.dedup();
    let spans = match (knots.first(), knots.last()) {
        (Some(&first), Some(&last)) => vec![(low, first), (last, high)],
        _ => vec![(low, high)],
    };
    let span = edge_end - edge_start;
    let mut out = Vec::new();
    for (a, b) in spans {
        if !(b > a) {
            continue;
        }
        for local in [0.25, 0.5, 0.75].into_iter().chain((1..=TRACKING_ACCEPTANCE_POINTS).map(|index| index as f64 / (TRACKING_ACCEPTANCE_POINTS + 1) as f64)) {
            let along = (a + (b - a) * local - edge_start) / span;
            let fraction = if forward { along } else { 1.0 - along };
            if fraction.is_finite() && (0.0..=1.0).contains(&fraction) {
                out.push(fraction);
            }
        }
    }
    out.sort_by(f64::total_cmp);
    out.dedup();
    out
}

/// At most this many of the legacy pcurve's worst spans are seeded.
const WORST_SPANS: usize = 4;
/// Interior stations seeded in each of them.
const WORST_SPAN_SEEDS: usize = 7;

/// The seed fractions of [`end_span_challenger`]'s worst-span source: the
/// legacy pcurve's own spans (between its distinct knots) whose same-parameter
/// deviation, read at the quarter points and 16 interior points, and at any of
/// `extra_probes` (the edge's own end-span interior points mapped to coedge
/// fractions) that fall inside the span, exceeds
/// `bar`, worst first, at most [`WORST_SPANS`] of them, each seeded with
/// [`WORST_SPAN_SEEDS`] evenly spaced interior fractions. Fails closed on any
/// non-finite reading.
fn legacy_worst_span_seeds(
    evaluate_edge: &dyn Fn(f64) -> Result<Vec3, String>,
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    bar: f64,
    extra_probes: &[f64],
) -> Result<Vec<f64>, String> {
    let [q0, q1] = pcurve.domain()?;
    let mut knots: Vec<f64> = pcurve.knots.iter().copied().filter(|k| *k >= q0 && *k <= q1).collect();
    knots.dedup();
    let locals: Vec<f64> = [0.25, 0.5, 0.75]
        .into_iter()
        .chain((1..=TRACKING_ACCEPTANCE_POINTS).map(|index| index as f64 / (TRACKING_ACCEPTANCE_POINTS + 1) as f64))
        .collect();
    let mut over: Vec<(f64, f64, f64)> = Vec::new();
    for span in knots.windows(2).filter(|w| w[1] > w[0]) {
        let mut worst = 0.0_f64;
        let inside = extra_probes.iter().copied().filter(|f| *f > span[0] && *f < span[1]);
        for fraction in locals.iter().map(|local| span[0] + (span[1] - span[0]) * local).chain(inside) {
            let uv = pcurve.evaluate(fraction)?;
            let deviation = surface.evaluate_extended(uv.x, uv.y)?.sub(evaluate_edge(fraction)?).length();
            if !deviation.is_finite() {
                return Err(format!("worst-span read: non-finite deviation at fraction {fraction}"));
            }
            worst = worst.max(deviation);
        }
        if worst > bar {
            over.push((worst, span[0], span[1]));
        }
    }
    over.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.total_cmp(&b.1)));
    over.truncate(WORST_SPANS);
    Ok(over
        .into_iter()
        .flat_map(|(_, lo, hi)| (1..=WORST_SPAN_SEEDS).map(move |index| lo + (hi - lo) * index as f64 / (WORST_SPAN_SEEDS + 1) as f64))
        .collect())
}

/// Why the range lane's [`end_span_challenger`] runs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ChallengerTrigger {
    /// The legacy residual misses `ask + standoff` (the original trigger).
    Deviation,
    /// The legacy residual is within `ask + standoff`, the edge stands off its
    /// carrier by more than the ask, and the legacy fit's COMMON tracking
    /// (`common_station_read` at the fixed probes) misses the ask.
    Tracking,
}

/// The challenger's trigger. The original rule reads the same-parameter
/// DEVIATION against `ask + standoff`; over a standoff `g` a tangential slip
/// `d` adds only about d²/(2g) to it, so a slip up to sqrt(ask² + 2·ask·g)
/// never triggers (about 1.3e-5 at g = 9e-4, the fixture-25 inherited edges).
/// When the deviation rule does not fire and the edge stands off its carrier
/// by more than the ask (the same gate `refine_pcurve` takes its
/// standoff-tracking lane on), the legacy fit's common tracking is read
/// (`tracking`, called only then): over the ask, the challenger runs on
/// tracking. An unread tracking (None or non-finite) never triggers.
fn challenger_trigger(residual: f64, off_surface: f64, ask: f64, tracking: impl FnOnce() -> Option<f64>) -> Option<ChallengerTrigger> {
    if residual > ask + off_surface {
        return Some(ChallengerTrigger::Deviation);
    }
    if !(off_surface > ask) {
        return None;
    }
    match tracking() {
        Some(value) if value.is_finite() && value > ask => Some(ChallengerTrigger::Tracking),
        _ => None,
    }
}

/// The END-SPAN CHALLENGER of the range lane (2026-10-05, the four original
/// fitter rows of `abc_00000011/12` edges 7 and 243). On e90c30c94 their worst
/// excess (4.58e-6 and 5.14e-6) sat in the installed pcurve's LAST span, at the
/// edge's start, where the edge curve's own first knot span is only 4.5e-6
/// (e7) and 1.2e-4 (e243) wide while the fitter's end span was 1.4e-4 wide at
/// its cap. It runs when [`challenger_trigger`] fires: the legacy fit misses
/// its bar (residual over the ask plus its standoff), or, on an edge standing
/// off its carrier by more than the ask, its common tracking misses the ask.
/// It refits with the edge's own interior knots that
/// fall inside the legacy pcurve's first or last span as additional stations
/// (not joins; corners stay the legacy ones), through the same fitter with the
/// unchanged ask, SPAN_FLOOR, cap and rounds. The candidate replaces the legacy
/// fit only when both, read at COMMON probes (both pcurves' own knot spans,
/// knots and endpoints, plus the edge's own knots and knot spans mapped to
/// fractions) against one SHARED foot per probe ([`common_station_read`]),
/// read finite, the candidate's tracking is strictly lower and its
/// same-parameter deviation no worse. Otherwise, or on any error or unreadable read, the legacy fit
/// stands exactly.
#[allow(clippy::too_many_arguments)]
fn end_span_challenger(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    evaluate_edge: &dyn Fn(f64) -> Result<Vec3, String>,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    breaks: &[f64],
    ask: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
    legacy: PcurveFit,
) -> Result<PcurveFit, String> {
    // On the bar by deviation: nothing is read (the original zero-work
    // path). The TRACKING trigger (538dd700c: an off-carrier edge on its
    // deviation bar challenged on its common tracking) is withdrawn from this
    // merge (Claude master's ruling, 2026-10-05): bisected as the first commit
    // that moved step_import's a_trim_off_its_bar_is_accepted_and_named, and
    // the four original fitter rows do not need it (they enter on the
    // deviation trigger).
    if !(legacy.report.residual > ask + legacy.report.off_surface) {
        return Ok(legacy);
    }
    // The trigger read, the candidate's tracking verification and the final
    // read project the same edge points again: memoized, same answers.
    let _feet = FootMemoScope::open(surface);
    let Ok(end_knots) = native_end_knot_fractions(curve, edge_start, edge_end, forward, &legacy.curve) else { return Ok(legacy) };
    // The legacy fit's own WORST native spans (2026-10-05, root's boundary
    // audit of the four rows): the e90 witnesses read a 4.58e-6 / 5.14e-6
    // pointwise excess at q 0.99996 inside the closed native domain, in the
    // legacy pcurve's last span, where no edge knot falls. Each span between the
    // legacy pcurve's own distinct knots is read at its 19 probes; the spans
    // whose same-parameter deviation exceeds the ask plus the legacy standoff,
    // worst first and at most WORST_SPANS of them, each get WORST_SPAN_SEEDS
    // evenly spaced interior stations. Unreadable reads decline.
    // The edge's OWN first and last native knot spans, read at their quarter
    // points and 16 interior points, mapped through the trim's range and sense
    // to coedge fractions (e7's first edge span [0, 4.496e-6] of a
    // 0.018661-long trim maps reversed to q in [0.99976, 1], overlapping the
    // legacy pcurve's last span [0.99986, 1]).
    let edge_end_probes = edge_end_span_fractions(curve, edge_start, edge_end, forward);
    // The FIXED part of the common probe set, known before the candidate is
    // fitted: the legacy pcurve's own spans, endpoints and knots, and the
    // edge's own end-span probes, knot-span quarters and knots (every probe
    // that could have TRIGGERED the challenger among them). The candidate fit
    // verifies against these as well as its own spans, so it stops on the
    // reader that judges it.
    let Ok(mut fixed) = native_probe_fractions(&[&legacy.curve]) else { return Ok(legacy) };
    fixed.extend(edge_end_probes.iter().copied());
    let (low, high) = (edge_start.min(edge_end), edge_start.max(edge_end));
    let mut edge_knots: Vec<f64> = curve.knots.iter().copied().filter(|k| *k >= low && *k <= high).collect();
    edge_knots.push(low);
    edge_knots.push(high);
    edge_knots.sort_by(f64::total_cmp);
    edge_knots.dedup();
    let to_fraction = |t: f64| {
        let along = (t - edge_start) / (edge_end - edge_start);
        if forward { along } else { 1.0 - along }
    };
    for span in edge_knots.windows(2).filter(|w| w[1] > w[0]) {
        for local in [0.25, 0.5, 0.75] {
            fixed.push(to_fraction(span[0] + (span[1] - span[0]) * local));
        }
    }
    fixed.extend(edge_knots.iter().map(|&t| to_fraction(t)));
    fixed.extend(end_knots.iter().copied());
    fixed.extend(legacy.curve.knots.iter().copied());
    fixed.retain(|f| f.is_finite() && (0.0..=1.0).contains(f));
    fixed.sort_by(f64::total_cmp);
    fixed.dedup();
    let trigger = challenger_trigger(legacy.report.residual, legacy.report.off_surface, ask, || {
        common_station_read(surface, evaluate_edge, &legacy.curve, &legacy.curve, &fixed).ok().map(|((tracking, _), _)| tracking)
    });
    let Some(trigger) = trigger else { return Ok(legacy) };
    let Ok(worst_seeds) = legacy_worst_span_seeds(evaluate_edge, surface, &legacy.curve, ask + legacy.report.off_surface, &edge_end_probes) else { return Ok(legacy) };
    // The original guard holds for the deviation trigger only: a tracking
    // trigger needs no added station (the tracking mode and its verification
    // read the fixed probes).
    if trigger == ChallengerTrigger::Deviation && end_knots.is_empty() && worst_seeds.is_empty() {
        return Ok(legacy);
    }
    let mut stations: Vec<f64> = breaks.iter().copied().chain(end_knots.iter().copied()).chain(worst_seeds.iter().copied()).collect();
    stations.sort_by(f64::total_cmp);
    stations.dedup();
    // The fitter's cap is max(requested cap, initial station count): the added
    // stations must not raise it. Decline when the merged initial count
    // (base stations plus every break and end knot) could exceed the
    // requested cap.
    let initial = base_samples.max(1).checked_add(1).and_then(|count| count.checked_add(stations.len()));
    if initial.is_none_or(|count| count > parameter_cap) {
        return Ok(legacy);
    }
    let Ok(candidate) = fit_station_pcurve_mode(surface, evaluate_edge, &stations, breaks, ask, base_samples, refinement_rounds, parameter_cap, SPAN_FLOOR, true, Some(&legacy.curve), &fixed) else {
        return Ok(legacy);
    };
    if !(candidate.report.residual.is_finite() && candidate.report.off_surface.is_finite() && candidate.report.clamped_excursion.is_finite() && candidate.report.clamped_distance.is_finite()) {
        return Ok(legacy);
    }
    // Common probes: the fixed set plus the candidate's own knot spans,
    // endpoints and knots (the same set as both curves' native probes plus
    // the edge's, as before).
    let Ok(mut probes) = native_probe_fractions(&[&candidate.curve]) else { return Ok(legacy) };
    probes.extend(fixed.iter().copied());
    probes.extend(candidate.curve.knots.iter().copied());
    probes.retain(|f| f.is_finite() && (0.0..=1.0).contains(f));
    probes.sort_by(f64::total_cmp);
    probes.dedup();
    let readings = common_station_read_with_excess(surface, evaluate_edge, &legacy.curve, &candidate.curve, &probes);
    let accepted = matches!(&readings, Ok((old, new)) if floor_admits(*old, *new));
    if station_fit_tracing() {
        eprintln!(
            "STATION-FIT end-span challenger: trigger={trigger:?} end_knots={} worst_span_seeds={} edge_end_probes={} legacy={:?}/{:e} candidate={:?}/{:e} common={readings:?} decision={}",
            end_knots.len(), worst_seeds.len(), edge_end_probes.len(), legacy.report.exit, legacy.report.residual, candidate.report.exit, candidate.report.residual,
            if accepted { "candidate" } else { "legacy" }
        );
    }
    Ok(if accepted { candidate } else { legacy })
}


/// [`fit_pcurve_on_surface_range_dense`] for the callers that only need the
/// curve, accepted or refused on its bar ([`accept_fit`]).
#[allow(clippy::too_many_arguments)]
pub fn build_pcurve_on_surface_range_dense(
    surface: &NurbsSurface,
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
) -> Result<NurbsCurve, String> {
    accept_fit(fit_pcurve_on_surface_range_dense(
        surface,
        curve,
        edge_start,
        edge_end,
        forward,
        tolerance,
        base_samples,
        refinement_rounds,
        parameter_cap,
    )?)
}

/// The fractions of `[edge_start, edge_end]`, walked in the coedge's sense, at
/// which `curve` is not C2: interior knots of multiplicity at least its
/// degree. A rational-quadratic arc past a quarter turn is two segments joined
/// at a double knot, and an interpolant through uniform samples across that
/// joint cannot represent the second-derivative jump: on the 2026-09-18 import
/// census 1511 of the 2979 arc trims blamed on the fitter had their worst
/// miss at fraction 0.49–0.51, the mid double knot, exactly as the fillet's
/// corner patch had measured (`blend/track_fit.rs::curve_breaks`, which this
/// mirrors for a range).
pub(crate) fn range_breaks(
    curve: &NurbsCurve,
    edge_start: f64,
    edge_end: f64,
    forward: bool,
) -> Vec<f64> {
    let mut fractions = Vec::new();
    let span = edge_end - edge_start;
    if !(span.abs() > 0.0) {
        return fractions;
    }
    let (low, high) = (edge_start.min(edge_end), edge_start.max(edge_end));
    let mut index = 0;
    while index < curve.knots.len() {
        let value = curve.knots[index];
        let run = curve.knots[index..]
            .iter()
            .take_while(|knot| **knot == value)
            .count();
        if value > low && value < high && run >= curve.degree.max(1) {
            let along = (value - edge_start) / span;
            fractions.push(if forward { along } else { 1.0 - along });
        }
        index += run;
    }
    fractions.sort_by(f64::total_cmp);
    fractions
}

/// The body of [`fit_pcurve_on_surface_range_dense`], parameterized by the
/// 3D station the pcurve must pass through at each coedge fraction instead of
/// by an edge curve and a range.
///
/// The station supplier is the whole difference between deriving a pcurve and
/// reading one. `fit_pcurve_on_surface_range_dense` supplies points off the
/// edge's own 3D curve, so the answer is "the nearest point of the carrier",
/// which is ambiguous exactly where a trim is interesting — a seam branch, a
/// near-tangential approach, a pole. The STEP importer's supplied-pcurve lane
/// supplies points off the VENDOR's stated trim instead, and the ambiguity is
/// gone because the vendor resolved it.
///
/// `stations(fraction)` must be continuous in `fraction` over `[0, 1]` and land
/// on (or very near) `surface`; `breaks` are the fractions at which it is not
/// C2 ([`range_breaks`]), each of which becomes a station so no kink sits
/// between two nodes. Everything else — inversion, branch repair, refinement
/// and the measurement — is shared with the derived lane by construction.
///
/// The SURFACE's knot-line crossings are deliberately not stations here,
/// although `blend/track_fit.rs` makes them so. Measured 2026-09-26 over the
/// 57-document import corpus: with each crossing bisected on the fraction and
/// projected seeded from the left station, 34 bodies refused (19 with a foot
/// that misses its global distance rejected) against 15 without the pass —
/// every extra refusal a fit stalled around a crossing station, on
/// `AssemblyExample-Assembly` and `abc_00000014` among others. The raised
/// pass resolves the O(h²) tail across a crossing on its own.
///
/// # What the fit is measured against, and when it stops
///
/// The stations' own standoff from the carrier is read FIRST, from the base
/// inversions, and the refinement target is the ask plus that standoff
/// ([`PcurveFitReport::on_bar`]): a station standing `g` off the surface
/// cannot be imaged closer than `g`, so chasing it to the ask alone spends the
/// whole budget for nothing (the 2026-09-18 census: 2052 imported pcurves at
/// the 513 cap, 1958 of them on a curve that is not on its carrier).
///
/// Every probe and the final measurement read the carrier through
/// `evaluate_extended`: a pcurve riding a periodic seam legitimately carries
/// parameters past the domain, and the clamped `evaluate` this loop used until
/// 2026-09-26 read every such station as a gross miss — an insert per round
/// until the cap, on a trim that was right.
///
/// The RETURNED curve is measured out of sample — at the quarter points of
/// every span between its nodes, where an interpolant is exact at the nodes
/// and extremal between — unless its last round probed every span and found
/// nothing to insert, in which case that round's maximum already is that
/// measurement. A fit that binds at `parameter_cap` or spends its rounds with
/// the bar still unmet grows ONCE to [`PCURVE_RAISED_SAMPLES`]
/// (deterministic). The report says which exit it took and what it reached;
/// the `build_*` wrappers refuse over the bar, this function never does.
#[allow(clippy::too_many_arguments)]
pub fn fit_pcurve_on_surface_stations(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    breaks: &[f64],
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
) -> Result<PcurveFit, String> {
    fit_station_pcurve(
        surface, stations, breaks, &[], tolerance.max(PCURVE_REFINEMENT_TOLERANCE),
        base_samples, refinement_rounds, parameter_cap, SPAN_FLOOR,
    )
}

/// Precision construction for stations on recognized analytic carriers. Unlike
/// general projection, their inverse maps reach floating point accuracy. This
/// is an explicit construction contract, not a change to general fit policy.
pub(crate) fn analytic_station_pcurve(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    tolerance: f64,
) -> Result<NurbsCurve, String> {
    if surface.analytic().is_none() {
        return Err("analytic pcurve requires an analytic carrier".into());
    }
    let mut breaks = Vec::new();
    for _ in 0..4 {
        let fit = fit_station_pcurve(surface, stations, &breaks, &[], tolerance, 64, 64, 8192, 1e-10)?;
        if fit.report.residual > tolerance || fit.report.off_surface > tolerance {
            return Err(format!(
                "analytic pcurve construction missed its contract: {:?}",
                fit.report
            ));
        }
        let mut failed = Vec::new();
        // Independent Gauss stations in every returned knot span, rather than
        // re-reading the interpolation nodes or the fitter's quarter stations.
        for pair in fit.curve.knots.windows(2) {
            if pair[1] <= pair[0] {
                continue;
            }
            for local in [
                0.06943184420297371,
                0.33000947820757187,
                0.6699905217924281,
                0.9305681557970262,
            ] {
                let t = pair[0] + (pair[1] - pair[0]) * local;
                let uv = fit.curve.evaluate(t)?;
                if surface
                    .evaluate_extended(uv.x, uv.y)?
                    .sub(stations(t)?)
                    .length()
                    > tolerance
                {
                    failed.push(t);
                }
            }
        }
        if failed.is_empty() {
            return Ok(fit.curve);
        }
        if breaks.len() + failed.len() > 8192 {
            break;
        }
        breaks.extend(failed);
        breaks.sort_by(f64::total_cmp);
        breaks.dedup();
    }
    Err("analytic pcurve failed independent knot-span witnesses".into())
}

#[allow(clippy::too_many_arguments)]
fn fit_station_pcurve(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    breaks: &[f64],
    corners: &[f64],
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
    span_floor: f64,
) -> Result<PcurveFit, String> {
    fit_station_pcurve_mode(surface, stations, breaks, corners, tolerance, base_samples, refinement_rounds, parameter_cap, span_floor, false, None, &[])
}

/// [`fit_station_pcurve`] with an optional TRACKING mode, taken only by the
/// range lane's [`end_span_challenger`]; every other caller keeps the legacy
/// mode byte for byte. The legacy fitter probes, ranks and stops on the
/// image's same-parameter DEVIATION from the edge point. On an edge standing
/// off its carrier that deviation never falls under the ask, so every probe is
/// a candidate, the cap goes to the probes with the largest STANDOFF, and a
/// tangential slip of `d` adds only about d²/(2g) to a deviation over a
/// standoff `g`: the four original rows' challenger met its standoff (4.670e-6)
/// and kept a common tracking of 1.434e-6 (root's e24 run, 2026-10-05). In
/// tracking mode each probe reads its TRACKING, the image against the edge
/// point's foot (the nearer admitted one of the global projection and the
/// projection seeded at the image's uv: finite, inside the native domain, as
/// [`common_station_read`] admits). A probe is a candidate only over the ask,
/// the cap goes to the worst TRACKING first, and the one raise is taken when
/// the last round's tracking still misses the ask. The reported residual
/// stays the same-parameter deviation (standoff included). A probe with no
/// admitted foot is an error, which the challenger turns into the exact legacy
/// fit.
#[allow(clippy::too_many_arguments)]
fn fit_station_pcurve_mode(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    breaks: &[f64],
    corners: &[f64],
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
    span_floor: f64,
    track_projection: bool,
    reference: Option<&NurbsCurve>,
    verify: &[f64],
) -> Result<PcurveFit, String> {
    // Read only in tracking mode: the legacy mode makes no extra call.
    let domains = if track_projection { Some(surface_domains(surface)?) } else { None };
    // The tracking reading at one fraction, by the SAME foot rule as
    // [`common_station_read`], the reader the challenger accepts on: the
    // nearest admitted foot among the global projection and the projections
    // seeded at this image's uv and at the reference (legacy) pcurve's uv at
    // the same fraction. Returns the tracking and that foot (uv, distance),
    // which is where a tracking insert is placed.
    let tracking_of = |represented: Vec3, edge_point: Vec3, uv: Vec3, fraction: f64| -> Result<(f64, [f64; 2], f64), String> {
        let (domain_u, domain_v) = domains.ok_or_else(|| "station tracking: legacy mode".to_string())?;
        let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
        let reference_seed = match reference {
            Some(curve) => curve.evaluate(fraction).ok().and_then(|seed| memo_seeded_foot(surface, edge_point, seed.x, seed.y)),
            None => None,
        };
        // The common reader's order (global, reference, candidate), so an
        // equal-distance tie takes the same foot here as there.
        let foot = common_foot(
            memo_global_foot(surface, edge_point),
            reference_seed,
            memo_seeded_foot(surface, edge_point, uv.x, uv.y),
            domain_u,
            domain_v,
        )
        .ok_or_else(|| "station tracking: no admitted foot".to_string())?;
        let tracking = represented.sub(foot.point).length();
        if !(finite(represented) && tracking.is_finite()) {
            return Err("station tracking: non-finite reading".into());
        }
        Ok((tracking, [foot.u, foot.v], foot.distance))
    };
    // The last round's tracking missed the ask (tracking mode only).
    let mut tracking_unmet = false;
    let evaluate_edge = stations;
    let base_samples = base_samples.max(1);
    // The general caller floors its ask: its projector places a foot
    // to about `LINEAR_TOLERANCE` (1e-7), so a caller asking for 1e-9 asks for
    // more than any station can deliver. Chased to such an ask the fit spends
    // its cap and reads its own noise — the direct-edit heals asked 1e-9 on
    // stations 9.9e-8 off their carrier and were refused by 8.5e-9
    // (2026-09-26). The floor is the ask's floor; the report says what was
    // reached against it.

    let mut parameters = (0..=base_samples)
        .map(|index| index as f64 / base_samples as f64)
        .collect::<Vec<_>>();
    // A break within the span floor of a station IS that station: two nodes
    // closer than that make the interpolant swing between them, and over a
    // stretch that short the kink moves the fit by nothing a bar can read.
    // `abc_00000011` face #549's curve carries a triple knot 4e-7 before its
    // end; as a station it put the cubic 1.594e-2 off between it and the last
    // one (measured 2026-09-26), where the file's curve sits 1e-7 off.
    // A break that is a CORNER of the curve (`corners`, a subset of
    // `breaks`: the range lane passes its curve's knots of multiplicity >=
    // degree as both) is a C0 JOIN of the interpolant
    // (`build_interpolant_joined`): one cubic through the corner overshoots
    // on both sides whatever is inserted. A break that is only a station
    // (an analytic witness, a caller's own marker) joins nothing.
    let mut joins = Vec::with_capacity(corners.len());
    for &fraction in breaks {
        if !(fraction > 0.0 && fraction < 1.0) {
            continue;
        }
        let corner = corners.contains(&fraction);
        let at = parameters.partition_point(|parameter| *parameter < fraction);
        let near_left = at > 0 && fraction - parameters[at - 1] <= span_floor;
        let near_right = at < parameters.len() && parameters[at] - fraction <= span_floor;
        if near_left || near_right {
            // The break IS that station (a corner at a base station, e.g.
            // the midpoint of a two-piece curve at 32/64): no insert, but the
            // station is still the join.
            if corner {
                joins.push(if near_right { parameters[at] } else { parameters[at - 1] });
            }
            continue;
        }
        parameters.insert(at, fraction);
        if corner {
            joins.push(fraction);
        }
    }
    // The 3D edge point behind each raw inversion sample, kept parallel so the
    // continuity repair can re-seed a jumped station from its own footpoint.
    let mut edge_points = parameters
        .iter()
        .map(|fraction| evaluate_edge(*fraction))
        .collect::<Result<Vec<_>, _>>()?;
    // The stations' standoff from the carrier: the part of every residual no
    // pcurve can remove, known before a single probe is spent.
    let mut off_surface = 0.0_f64;
    let mut raw = Vec::with_capacity(edge_points.len());
    let mut gaps = Vec::with_capacity(edge_points.len());
    for point in &edge_points {
        let (parameter, gap) = invert_checked(surface, *point)?;
        off_surface = off_surface.max(gap);
        raw.push(parameter);
        gaps.push(gap);
    }
    repair_branch_jumps(surface, &edge_points, &mut raw, tolerance)?;
    let (mut pcurve, mut excursion, mut distance) =
        build_interpolant_joined(surface, &raw, &parameters, &joins)?;

    let mut cap = parameter_cap.max(parameters.len());
    let mut rounds = refinement_rounds;
    let mut raised = false;
    loop {
        let mut exit = PcurveFitExit::RoundBudget;
        let mut converged_max = None;
        // This pass's last round probed every span (none narrow, none
        // truncated) and every probe tracked within the ask (tracking mode).
        let mut tracking_met = false;
        for _ in 0..rounds {
            if parameters.len() >= cap {
                exit = PcurveFitExit::SampleCeiling;
                break;
            }
            // The target is the ASK, bare: a station standing off the carrier
            // is chased to the foot it projects to, as the fitter has always
            // done. A target of ask + standoff (one landing, 2026-09-26) let
            // such a fit stop at its base stations with the interpolant
            // wandering between them, folded a molex wall's mesh and put a
            // trim of abc_00000023 2.2e-5 off its own parameter; the bar
            // below still reads what it reached.
            let target = tolerance;
            let mut candidates = Vec::new();
            let mut round_max = 0.0_f64;
            let mut truncated = false;
            let mut skipped_narrow = false;
            let worst_first = worst_first();
            'spans: for index in 0..parameters.len() - 1 {
                // The loop's own resolution floor: a span narrower than this
                // is not subdivided. Without it the
                // raised pass inserted into a vanishing span at a trim's end
                // until the cap — `abc_00000011` face #549's last stations sat
                // 4e-7 apart with their feet clamped at the patch edge, and the
                // cubic through them swung 1.594e-2 off between them (measured
                // 2026-09-26). A miss that survives only inside such a span is
                // reported as [`PcurveFitExit::SpanFloor`].
                if parameters[index + 1] - parameters[index] < span_floor {
                    skipped_narrow = true;
                    continue;
                }
                for local in [0.25, 0.5, 0.75] {
                    if !worst_first && parameters.len() + candidates.len() >= cap {
                        truncated = true;
                        break 'spans;
                    }
                    let fraction =
                        parameters[index] + (parameters[index + 1] - parameters[index]) * local;
                    let uv = pcurve.evaluate(fraction)?;
                    let represented = surface.evaluate_extended(uv.x, uv.y)?;
                    let edge_point = evaluate_edge(fraction)?;
                    let deviation = represented.sub(edge_point).length();
                    // The probe's reading: its deviation (legacy), or its
                    // tracking against the admitted foot (tracking mode).
                    let (reading, foot) = if track_projection {
                        let (tracking, foot_uv, foot_gap) = tracking_of(represented, edge_point, uv, fraction)?;
                        (tracking, Some((foot_uv, foot_gap)))
                    } else {
                        (deviation, None)
                    };
                    round_max = round_max.max(reading);
                    if reading <= target {
                        continue;
                    }
                    candidates.push((index, fraction, edge_point, uv, reading, foot));
                }
            }
            // Tracking mode's VERIFICATION, before it may stop: a round whose
            // quarter probes all track within the ask is read again at the
            // probes the challenger's acceptance reads, the current pcurve's
            // own knot spans (endpoints, quarters and 16 interior points: the
            // interpolant's knots are averages of three stations, not the
            // stations) and the caller's fixed fractions (the legacy pcurve's
            // spans and the edge's own). A probe over the ask becomes a
            // candidate at its own fraction, placed at its own foot. One
            // within the span floor of a station cannot be inserted: it is
            // counted, and the fit may not report its tracking as met. On e24
            // the four rows stopped on their quarter probes and read 1.144e-7 /
            // 1.590e-7 at the common probes (a4c84871 runtime, 2026-10-05).
            let mut verify_blocked = false;
            if track_projection && candidates.is_empty() && !truncated && !skipped_narrow {
                for fraction in verification_fractions(&pcurve, verify)? {
                    let index = parameters.partition_point(|parameter| *parameter <= fraction).saturating_sub(1).min(parameters.len() - 2);
                    let uv = pcurve.evaluate(fraction)?;
                    let represented = surface.evaluate_extended(uv.x, uv.y)?;
                    let edge_point = evaluate_edge(fraction)?;
                    let (tracking, foot_uv, foot_gap) = tracking_of(represented, edge_point, uv, fraction)?;
                    round_max = round_max.max(tracking);
                    if tracking <= target {
                        continue;
                    }
                    if fraction - parameters[index] <= span_floor || parameters[index + 1] - fraction <= span_floor {
                        verify_blocked = true;
                        continue;
                    }
                    // Two over-ask probes closer than the span floor in one
                    // span: keep the worse, so no near-duplicate stations.
                    if let Some(last) = candidates.last_mut() {
                        if last.0 == index && fraction - last.1 <= span_floor {
                            if tracking > last.4 {
                                *last = (index, fraction, edge_point, uv, tracking, Some((foot_uv, foot_gap)));
                            }
                            continue;
                        }
                    }
                    candidates.push((index, fraction, edge_point, uv, tracking, Some((foot_uv, foot_gap))));
                }
            }
            tracking_unmet = track_projection && round_max > target;
            // A round with more candidates than the cap has room for spends
            // that room on its WORST probes, kept in curve order, rather than
            // on the first spans in order: truncating in span order left the
            // late spans of an off-carrier ring at their base spacing and its
            // image 1.523e-6 off its feet; worst first reaches 5.84e-8 on the
            // same 513 samples. An even stride along the curve reached 2.87e-7
            // there but diluted the inserts a local miss needs and lost fits
            // over the corpus (a 1.3e-5 trim of `abc_00000011` fell back to its
            // 4.4e-4 size-ask fit), measured 2026-09-27.
            let room = cap.saturating_sub(parameters.len());
            if candidates.len() > room {
                truncated = true;
                let mut order: Vec<usize> = (0..candidates.len()).collect();
                order.sort_by(|&a, &b| candidates[b].4.total_cmp(&candidates[a].4).then(a.cmp(&b)));
                order.truncate(room);
                order.sort_unstable();
                candidates = order.into_iter().map(|index| candidates[index]).collect();
            }
            let mut inserts = Vec::with_capacity(candidates.len());
            for (index, fraction, edge_point, uv, _, foot) in candidates {
                // A tracking candidate is placed at the very foot it was read
                // against, so the node's image IS that foot; the legacy
                // inversion is unchanged.
                let (parameter, gap) = match foot {
                    Some(placed) => placed,
                    None => invert_near(surface, edge_point, [uv.x, uv.y], gaps[index].max(gaps[index + 1]))?,
                };
                inserts.push((index + 1, fraction, edge_point, parameter, gap));
            }
            if station_fit_tracing() {
                eprintln!(
                    "STATION-FIT round: samples={} target={target:.3e} round_max={round_max:.3e} inserts={} truncated={truncated}{}",
                    parameters.len(),
                    inserts.len(),
                    // Tracking mode's round_max is a TRACKING reading.
                    if track_projection { " mode=tracking" } else { "" }
                );
            }
            if inserts.is_empty() {
                exit = if truncated {
                    PcurveFitExit::SampleCeiling
                } else if skipped_narrow {
                    // Every probed span is within the target; the narrow ones
                    // were not probed, and the sweep below measures them.
                    PcurveFitExit::SpanFloor
                } else if track_projection {
                    // Every probe TRACKS within the ask, but round_max is a
                    // tracking reading, not the deviation the report carries.
                    // The exit is Stalled (no probe can be improved; the
                    // residual measured below is the edge's own standoff),
                    // and it becomes Converged below only if that measured
                    // deviation itself meets the floor. A verification probe
                    // over the ask that could not be inserted leaves the
                    // tracking UNMET.
                    tracking_met = !verify_blocked;
                    PcurveFitExit::Stalled
                } else {
                    converged_max = Some(round_max);
                    PcurveFitExit::Converged
                };
                break;
            }
            for (index, fraction, edge_point, parameter, gap) in inserts.into_iter().rev() {
                parameters.insert(index, fraction);
                edge_points.insert(index, edge_point);
                raw.insert(index, parameter);
                gaps.insert(index, gap);
                off_surface = off_surface.max(gap);
            }
            // Inserts are independent global inversions too — re-run the repair
            // so a fold-flip introduced mid-refinement cannot poison the next
            // round's deviation interpolant.
            repair_branch_jumps(surface, &edge_points, &mut raw, tolerance)?;
            let rebuilt = build_interpolant_joined(surface, &raw, &parameters, &joins)?;
            pcurve = rebuilt.0;
            excursion = rebuilt.1;
            distance = rebuilt.2;
            if truncated {
                exit = PcurveFitExit::SampleCeiling;
                break;
            }
        }
        let residual = match converged_max {
            Some(measured) => measured,
            None => {
                station_residual(surface, evaluate_edge, &pcurve, &parameters, tolerance, &mut off_surface)?
            }
        };
        if station_fit_tracing() {
            eprintln!(
                "STATION-FIT pass raised={raised} exit={exit:?} samples={} residual={residual:.3e} off_surface={off_surface:.3e}",
                parameters.len()
            );
            if residual > tolerance + off_surface && (exit == PcurveFitExit::Converged || raised) {
                // Where the miss sits, and the uv track around it: the trace a
                // refusal needs to be attributed to a fold, a seam or the file.
                let (mut worst, mut at) = (0.0f64, 0usize);
                for (index, span) in parameters.windows(2).enumerate() {
                    let fraction = 0.5 * (span[0] + span[1]);
                    let uv = pcurve.evaluate(fraction)?;
                    let deviation = surface.evaluate_extended(uv.x, uv.y)?.sub(evaluate_edge(fraction)?).length();
                    if deviation > worst {
                        worst = deviation;
                        at = index;
                    }
                }
                let ([u0, u1], [v0, v1]) = surface_domains(surface)?;
                let (closed_u, closed_v) = surface_closedness(surface)?;
                eprintln!(
                    "STATION-FIT OFF BAR: worst {worst:.3e} between stations {at} and {}; surface degree {}x{} net {}x{} domain u[{u0},{u1}] v[{v0},{v1}] closed ({closed_u},{closed_v}) analytic {:?}",
                    at + 1,
                    surface.degree_u,
                    surface.degree_v,
                    surface.control_points.len(),
                    surface.control_points[0].len(),
                    surface.analytic().map(|_| "yes").unwrap_or("no")
                );
                {
                    let fraction = 0.5 * (parameters[at] + parameters[at + 1]);
                    let uv = pcurve.evaluate(fraction)?;
                    let image = surface.evaluate_extended(uv.x, uv.y)?;
                    let station = evaluate_edge(fraction)?;
                    let foot = project_point_to_surface(surface, station)?;
                    let seeded = project_point_to_surface_seeded(surface, station, raw[at][0], raw[at][1])?;
                    eprintln!(
                        "    at the worst midpoint: deviation {:.4e}, station gap {:.4e} (seeded {:.4e}), image-to-foot {:.4e} (to seeded foot {:.4e}), foot uv ({:.7}, {:.7}) seeded ({:.7}, {:.7}) image uv ({:.7}, {:.7})",
                        image.sub(station).length(), foot.distance, seeded.distance, image.sub(foot.point).length(), image.sub(seeded.point).length(), foot.u, foot.v, seeded.u, seeded.v, uv.x, uv.y
                    );
                }
                let lo = at.saturating_sub(3);
                let hi = (at + 4).min(parameters.len() - 1);
                for index in lo..=hi {
                    eprintln!(
                        "    station {index}: fraction {:.6} uv ({:.6}, {:.6}) point ({:.4}, {:.4}, {:.4})",
                        parameters[index], raw[index][0], raw[index][1], edge_points[index].x, edge_points[index].y, edge_points[index].z
                    );
                }
            }
        }
        // Legacy only: a SpanFloor exit whose measured DEVIATION is on the
        // bar is promoted. In tracking mode the deviation sits at the standoff
        // whatever the unprobed narrow spans' TRACKING is, so SpanFloor stays
        // SpanFloor (unread); a tracking-mode fit is Converged only when every
        // span was probed within the ask AND the measured deviation meets the
        // floor (the exit's documented meaning, never the standoff bar).
        if !track_projection && exit == PcurveFitExit::SpanFloor && residual <= tolerance + off_surface {
            exit = PcurveFitExit::Converged;
        }
        if tracking_met && residual <= tolerance {
            exit = PcurveFitExit::Converged;
        }
        if exit != PcurveFitExit::Converged
            && !raised
            && (residual > tolerance + off_surface || tracking_unmet)
            && cap < PCURVE_RAISED_SAMPLES
        {
            raised = true;
            cap = PCURVE_RAISED_SAMPLES;
            // The raised pass is bounded by SAMPLES, not rounds: a kink
            // converges O(h²), a round per halving, and the three rounds of
            // the first pass are what left the 2026-09-18 census's fits over
            // their ask with budget to spare (RoundBudget at 158–400 of 2000
            // samples, measured 2026-09-26). Forty rounds cover every halving
            // the cap admits; a fit that is not converging stalls out first.
            rounds = RAISED_ROUNDS;
            continue;
        }
        return Ok(PcurveFit {
            curve: pcurve,
            report: PcurveFitReport {
                residual,
                floor: tolerance,
                samples: parameters.len(),
                exit,
                off_surface,
                raised,
                clamped_excursion: excursion,
                clamped_distance: distance,
            },
        });
    }
}

/// [`fit_pcurve_on_surface_stations`] for the callers that only need the
/// curve, accepted or refused on its bar ([`accept_fit`]).
pub fn build_pcurve_on_surface_stations(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    breaks: &[f64],
    tolerance: f64,
    base_samples: usize,
    refinement_rounds: usize,
    parameter_cap: usize,
) -> Result<NurbsCurve, String> {
    accept_fit(fit_pcurve_on_surface_stations(
        surface,
        stations,
        breaks,
        tolerance,
        base_samples,
        refinement_rounds,
        parameter_cap,
    )?)
}

/// The largest 3D distance between `pcurve`'s image and its stations at the
/// quarter points of every span of `parameters` — out of sample by
/// construction, since the interpolant is exact at its nodes. Unclamped
/// evaluations; a station read over the running bar is inverted so that
/// `off_surface` covers the standoff AT the stations the residual is taken at,
/// not only at the nodes (a curve lifted off its plane by a bump stands
/// further off between two nodes than at either).
fn station_residual(
    surface: &NurbsSurface,
    stations: &dyn Fn(f64) -> Result<Vec3, String>,
    pcurve: &NurbsCurve,
    parameters: &[f64],
    tolerance: f64,
    off_surface: &mut f64,
) -> Result<f64, String> {
    let mut worst = 0.0_f64;
    let mut read = |fraction: f64| -> Result<(), String> {
        let uv = pcurve.evaluate(fraction)?;
        let represented = surface.evaluate_extended(uv.x, uv.y)?;
        let station = stations(fraction)?;
        let deviation = represented.sub(station).length();
        worst = worst.max(deviation);
        if deviation > tolerance + *off_surface {
            *off_surface = off_surface.max(invert_checked(surface, station)?.1);
        }
        Ok(())
    };
    for span in parameters.windows(2) {
        for local in [0.25, 0.5, 0.75] {
            read(span[0] + (span[1] - span[0]) * local)?;
        }
    }
    Ok(worst)
}

