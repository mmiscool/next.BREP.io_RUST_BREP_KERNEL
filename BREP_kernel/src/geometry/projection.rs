use crate::curve::interior_knots;
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};
use serde::Serialize;

const EPSILON: f64 = 1e-12;
const LINEAR_TOLERANCE: f64 = 1e-7;
const MAX_NEWTON_ITERATIONS: usize = 50;

#[derive(Clone, Copy, Debug, Serialize)]
pub struct CurveProjection {
    pub u: f64,
    pub point: Vec3,
    pub distance: f64,
}

#[derive(Clone, Copy, Debug, Serialize)]
pub struct SurfaceProjection {
    pub u: f64,
    pub v: f64,
    pub point: Vec3,
    pub distance: f64,
}

fn fit_parameter(value: f64, minimum: f64, maximum: f64, closed: bool) -> f64 {
    if closed {
        let period = maximum - minimum;
        (value - minimum).rem_euclid(period) + minimum
    } else {
        value.clamp(minimum, maximum)
    }
}

/// Escape hatch for tamper-verification: `BREP_PROJECTION_WITNESS=0` restores
/// the pre-2026-09-14 answer — the Newton walk from the nearest seed and
/// nothing else — so a fixture that depends on the rescue can be shown to
/// depend on it. Read once; the projector is called about a million times in
/// one boolean.
fn witness_rescue_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var("BREP_PROJECTION_WITNESS").as_deref() != Ok("0"))
}

/// Squared distance from `point` to the SEGMENT `a`-`b`.
///
/// The seed sweep's consecutive samples are the segments of a polyline that
/// approximates the curve, and this is how far that polyline passes from the
/// query — the witness the rescue below reads.
fn point_segment_distance_squared(point: Vec3, a: Vec3, b: Vec3) -> f64 {
    let along = b.sub(a);
    let length_squared = along.length_squared();
    let fraction = if length_squared > 0.0 {
        (point.sub(a).dot(along) / length_squared).clamp(0.0, 1.0)
    } else {
        0.0
    };
    a.add(along.scale(fraction)).sub(point).length_squared()
}

/// One Newton refinement of the foot parameter, seeded at `seed`.
///
/// `best_u` / `best_distance_squared` carry the best answer found SO FAR and
/// every iterate is compared against them, so a run that diverges or stalls can
/// only leave the answer where it was. The returned parameter is where the
/// iteration exited, which the caller still has to evaluate: the stall branch
/// takes its last step without evaluating it.
#[allow(clippy::too_many_arguments)]
fn curve_newton(
    curve: &NurbsCurve,
    point: Vec3,
    seed: f64,
    start: f64,
    end: f64,
    closed: bool,
    best_u: &mut f64,
    best_distance_squared: &mut f64,
) -> Result<f64, String> {
    let mut parameter = seed;
    for _ in 0..MAX_NEWTON_ITERATIONS {
        let derivatives = curve.derivatives_small(parameter, 2)?;
        let residual = derivatives[0].sub(point);
        let f = derivatives[1].dot(residual);
        let derivative = derivatives[2].dot(residual) + derivatives[1].length_squared();
        let distance_squared = residual.length_squared();
        if distance_squared < *best_distance_squared {
            *best_distance_squared = distance_squared;
            *best_u = parameter;
        }
        let residual_length = distance_squared.sqrt();
        let tangent_length = derivatives[1].length();
        if residual_length <= LINEAR_TOLERANCE
            || f.abs() <= EPSILON + 1e-10 * tangent_length * residual_length
            || derivative.abs() <= EPSILON
        {
            break;
        }
        let mut step = -f / derivative;
        let maximum_step = (end - start) / 4.0;
        if step.abs() > maximum_step {
            step = step.signum() * maximum_step;
        }
        // `fit_parameter` degenerates to exactly this `clamp` when `closed` is
        // false, so an OPEN curve takes the same path it always did.
        let next = fit_parameter(parameter + step, start, end, closed);
        // The step ACTUALLY taken. Wrapping makes the raw difference jump by a
        // whole period at the seam, so a 2e-16 step across it would read as a
        // full revolution and never satisfy the convergence test; and a step
        // that genuinely crosses the seam must not read as a stall.
        let moved = if closed {
            let period = end - start;
            let raw = next - parameter;
            raw - period * (raw / period).round()
        } else {
            next - parameter
        };
        if moved.abs() <= 1e-15 * (end - start) {
            parameter = next;
            break;
        }
        parameter = next;
    }
    Ok(parameter)
}

/// The parameter in `[low, high]` nearest `point`, by golden-section search on
/// the SQUARED distance.
///
/// Reads no derivative, so it is the one lane that still works where the
/// tangent vanishes, and it can never return a point farther than the bracket's
/// own ends. It assumes ONE minimum inside the bracket — true of a seed
/// interval, which is one sample step of a single knot span wide.
fn bracketed_nearest(curve: &NurbsCurve, point: Vec3, low: f64, high: f64) -> Result<f64, String> {
    // 1/φ. 64 contractions take a bracket to 2.5e-14 of its width, below the
    // parameter resolution the Newton polish that follows needs from a seed.
    const CONTRACTION: f64 = 0.618_033_988_749_894_9;
    const CONTRACTIONS: usize = 64;
    let width = high - low;
    if !(width > 0.0) {
        return Ok(low);
    }
    let (mut low, mut high) = (low, high);
    let mut c = high - CONTRACTION * (high - low);
    let mut d = low + CONTRACTION * (high - low);
    let mut value_c = curve.evaluate(c)?.sub(point).length_squared();
    let mut value_d = curve.evaluate(d)?.sub(point).length_squared();
    for _ in 0..CONTRACTIONS {
        if high - low <= 1e-15 * width {
            break;
        }
        if value_c < value_d {
            high = d;
            d = c;
            value_d = value_c;
            c = high - CONTRACTION * (high - low);
            value_c = curve.evaluate(c)?.sub(point).length_squared();
        } else {
            low = c;
            c = d;
            value_c = value_d;
            d = low + CONTRACTION * (high - low);
            value_d = curve.evaluate(d)?.sub(point).length_squared();
        }
    }
    Ok((low + high) * 0.5)
}

pub fn project_point_to_curve(curve: &NurbsCurve, point: Vec3) -> Result<CurveProjection, String> {
    let [start, end] = curve.domain()?;
    let mut breaks = vec![start];
    breaks.extend(interior_knots(&curve.knots, curve.degree));
    breaks.push(end);
    let samples_per_span = 4usize.max(curve.degree + 2);
    let mut best_u = start;
    let mut best_distance_squared = f64::INFINITY;
    // The sweep's FIRST sample is exactly `start` and its LAST is exactly `end`
    // (`breaks` opens at `start` and closes at `end`, and index 0 / index
    // `samples_per_span` hit a span's own ends), so closure costs no extra
    // evaluation: keep those two points and compare them.
    let mut start_point = None;
    let mut end_point = None;
    // The sampled polyline's own nearest approach, and the seed interval that
    // achieves it. Squared throughout, so the sweep pays no square root.
    let mut previous: Option<(f64, Vec3)> = None;
    let mut witness_distance_squared = f64::INFINITY;
    let mut witness_low = start;
    let mut witness_high = start;
    for pair in breaks.windows(2) {
        for index in 0..=samples_per_span {
            let parameter = pair[0] + (pair[1] - pair[0]) * index as f64 / samples_per_span as f64;
            let sample = curve.evaluate(parameter)?;
            if start_point.is_none() {
                start_point = Some(sample);
            }
            end_point = Some(sample);
            let distance_squared = sample.sub(point).length_squared();
            if distance_squared < best_distance_squared {
                best_distance_squared = distance_squared;
                best_u = parameter;
            }
            if let Some((previous_parameter, previous_sample)) = previous {
                let segment = point_segment_distance_squared(point, previous_sample, sample);
                if segment < witness_distance_squared {
                    witness_distance_squared = segment;
                    witness_low = previous_parameter;
                    witness_high = parameter;
                }
            }
            previous = Some((parameter, sample));
        }
    }
    // A closed curve's domain is PERIODIC, so Newton must wrap across the seam
    // rather than stop at it. Clamping made the projector return the seam for
    // any query whose foot sits within half a seed spacing of it: `start` and
    // `end` are the same POINT, the sweep keeps the first of equal distances and
    // so seeds at `start`, and a Newton step toward a foot just BEFORE the seam
    // is then clamped back to `start`, reads as a zero-length step, and breaks.
    // Measured on a rational-quadratic circle of radius 20: 4.796 mm out, for a
    // point taken off the curve itself. Swapping the sweep's tie-break to keep
    // the LAST equal sample does not fix this — it mirrors it, returning the
    // same 4.796 mm for queries just AFTER the seam. The surface projector has
    // always wrapped (`fit_parameter` at the `project_point_to_surface` Newton
    // step); this is the curve side catching up.
    const CLOSED_SEAM_TOLERANCE: f64 = 1e-6;
    let closed = match (start_point, end_point) {
        (Some(first), Some(last)) => first.sub(last).length() <= CLOSED_SEAM_TOLERANCE,
        _ => false,
    };

    let parameter = curve_newton(
        curve,
        point,
        best_u,
        start,
        end,
        closed,
        &mut best_u,
        &mut best_distance_squared,
    )?;
    for candidate in [start, end, parameter] {
        let distance_squared = curve.evaluate(candidate)?.sub(point).length_squared();
        if distance_squared < best_distance_squared {
            best_distance_squared = distance_squared;
            best_u = candidate;
        }
    }
    // AN ANSWER THE SEED POLYLINE REFUTES IS NOT AN ANSWER. The sweep's
    // consecutive samples are a polyline that approximates the curve, so the
    // nearest that polyline passes to the query is a witness: the curve
    // demonstrably comes about that close. When the answer is FARTHER than the
    // witness by more than a factor of two, Newton converged in the wrong basin
    // and the seed interval the witness names is where the foot really is.
    //
    // Two different defects reach here, both of them "a point ON the curve does
    // not project to its own parameter", and both measured on real geometry:
    //
    // * A STATIONARY seed. Newton is a first-order method and
    //   `f = C'(t) · (C(t) - P)` vanishes at a stationary parameter for EVERY
    //   `P`, so the iteration reads convergence there whatever the point is, and
    //   the step it would otherwise take is `-f/f''`, a second-order quantity
    //   whose sign has nothing to do with where the foot is. The 2026-09-14
    //   herringbone document is full of them: a flank pcurve is a two-span cubic
    //   whose first two control points COINCIDE (an involute's speed is
    //   `r_base * t`, so its Hermite start tangent is exactly zero), the sweep's
    //   nearest sample to a query near the start IS that cusp, and a point lying
    //   exactly on the curve at t = 0.125 answered t = 0, 2.159e-2 out in uv.
    //   That is the wrong answer `build_loop`'s partial-run rescue declined on.
    // * A seed grid coarser in ONE span than in the rest. An imported pcurve
    //   from `abc_00000013.step` carries 511 interior knots whose first span is
    //   eighty times the width of the median, and five samples per span leave
    //   its samples 4.6e-3 of uv apart — so a point on the curve at fraction
    //   0.01 read as 3.969e-3 from a DIFFERENT branch at fraction 0.99, which
    //   was genuinely nearer than any sample of its own.
    //
    // The witness refuted both by 71x and by 293000x. Every update below is a
    // strict improvement on the distance already found, so a rescue that finds
    // nothing better leaves the answer exactly where it was.
    const WITNESS_FACTOR: f64 = 2.0;
    if witness_distance_squared * (WITNESS_FACTOR * WITNESS_FACTOR) < best_distance_squared
        && witness_rescue_enabled()
    {
        let bracketed = bracketed_nearest(curve, point, witness_low, witness_high)?;
        let polished = curve_newton(
            curve,
            point,
            bracketed,
            start,
            end,
            closed,
            &mut best_u,
            &mut best_distance_squared,
        )?;
        for candidate in [bracketed, polished] {
            let distance_squared = curve.evaluate(candidate)?.sub(point).length_squared();
            if distance_squared < best_distance_squared {
                best_distance_squared = distance_squared;
                best_u = candidate;
            }
        }
    }
    let projected = curve.evaluate(best_u)?;
    Ok(CurveProjection {
        u: best_u,
        point: projected,
        distance: projected.sub(point).length(),
    })
}

fn project_linear_revolution(
    surface: &NurbsSurface,
    point: Vec3,
) -> Result<Option<SurfaceProjection>, String> {
    let rows = &surface.control_points;
    if surface.degree_u != 2
        || surface.degree_v != 1
        || rows.len() < 5
        || rows[0].len() != 2
        || !surface.closed_directions()?.0
    {
        return Ok(None);
    }
    let base = rows[0][0].point()?.add(rows[4][0].point()?).scale(0.5);
    let top = rows[0][1].point()?.add(rows[4][1].point()?).scale(0.5);
    let axis_vector = top.sub(base);
    let height = axis_vector.length();
    if height <= EPSILON {
        return Ok(None);
    }
    let axis = axis_vector.normalized()?;
    let [v0, v1] = surface.domain_v()?;
    let axial = point.sub(base).dot(axis).clamp(0.0, height);
    let v = v0 + (v1 - v0) * axial / height;
    let circle = surface.iso_curve_v(v)?;
    let projection = project_point_to_curve(&circle, point)?;
    let [u0, u1] = surface.domain_u()?;
    let u = projection.u.clamp(u0, u1);
    let projected = surface.evaluate(u, v)?;
    Ok(Some(SurfaceProjection {
        u,
        v,
        point: projected,
        distance: projected.sub(point).length(),
    }))
}

#[derive(Clone, Copy)]
struct NewtonResult {
    u: f64,
    v: f64,
    distance_squared: f64,
    converged: bool,
}

fn surface_newton(
    surface: &NurbsSurface,
    point: Vec3,
    seed_u: f64,
    seed_v: f64,
    domains: [f64; 4],
    closed_u: bool,
    closed_v: bool,
) -> Result<NewtonResult, String> {
    let [u0, u1, v0, v1] = domains;
    let mut u = seed_u;
    let mut v = seed_v;
    let mut converged = false;
    let mut best = NewtonResult {
        u,
        v,
        distance_squared: surface.evaluate(u, v)?.sub(point).length_squared(),
        converged,
    };
    for _ in 0..MAX_NEWTON_ITERATIONS {
        let derivatives = surface.derivatives_small(u, v, 2)?;
        let residual = derivatives[0][0].sub(point);
        let distance_squared = residual.length_squared();
        if distance_squared < best.distance_squared {
            best.u = u;
            best.v = v;
            best.distance_squared = distance_squared;
        }
        let f = derivatives[1][0].dot(residual);
        let g = derivatives[0][1].dot(residual);
        let residual_length = distance_squared.sqrt();
        if residual_length <= LINEAR_TOLERANCE {
            converged = true;
            break;
        }
        let tangent_u_length = derivatives[1][0].length();
        let tangent_v_length = derivatives[0][1].length();
        let cosine_u = if tangent_u_length * residual_length <= EPSILON {
            0.0
        } else {
            f.abs() / (tangent_u_length * residual_length)
        };
        let cosine_v = if tangent_v_length * residual_length <= EPSILON {
            0.0
        } else {
            g.abs() / (tangent_v_length * residual_length)
        };
        if cosine_u <= 1e-10 && cosine_v <= 1e-10 {
            converged = true;
            break;
        }
        let j00 = derivatives[2][0].dot(residual) + derivatives[1][0].length_squared();
        let j01 = derivatives[1][1].dot(residual) + derivatives[1][0].dot(derivatives[0][1]);
        let j11 = derivatives[0][2].dot(residual) + derivatives[0][1].length_squared();
        let determinant = j00 * j11 - j01 * j01;
        if determinant.abs() <= EPSILON {
            break;
        }
        let mut du = (-f * j11 + g * j01) / determinant;
        let mut dv = (-g * j00 + f * j01) / determinant;
        du = du.clamp(-(u1 - u0) / 4.0, (u1 - u0) / 4.0);
        dv = dv.clamp(-(v1 - v0) / 4.0, (v1 - v0) / 4.0);
        let next_u = fit_parameter(u + du, u0, u1, closed_u);
        let next_v = fit_parameter(v + dv, v0, v1, closed_v);
        let stalled =
            (next_u - u).abs() <= 1e-15 * (u1 - u0) && (next_v - v).abs() <= 1e-15 * (v1 - v0);
        u = next_u;
        v = next_v;
        if stalled {
            break;
        }
    }
    let final_distance_squared = surface.evaluate(u, v)?.sub(point).length_squared();
    if final_distance_squared < best.distance_squared {
        best.u = u;
        best.v = v;
        best.distance_squared = final_distance_squared;
    }
    best.converged = converged;
    Ok(best)
}

/// The point of `surface` nearest `point`, over the surface's whole domain.
///
/// Analytic carriers answer in closed form; every other surface goes through
/// [`project_point_to_surface_general`], which says what it guarantees.
/// `#[track_caller]` names the call site in the `BREP_PROJECTION_AUDIT` log.
#[track_caller]
pub fn project_point_to_surface(
    surface: &NurbsSurface,
    point: Vec3,
) -> Result<SurfaceProjection, String> {
    // Recognized analytic carriers (plane / cylinder / cone / sphere /
    // torus) have exact closed-form projections in the surface's own
    // rational parameterization — skip grid seeding and Newton entirely.
    if let Some(analytic) = surface.analytic() {
        if let Some(projection) = analytic.project(surface, point) {
            return Ok(projection);
        }
    }
    project_point_to_surface_general(surface, point)
}

/// Project a point onto a surface starting Newton from an explicit `(u, v)`
/// guess, WITHOUT the global grid seed. This is a footpoint refiner for
/// continuity-preserving curve-on-surface tracing: seeding each edge sample
/// from its neighbour's parameters keeps the fit on ONE branch of a surface
/// that folds back over the small trimmed patch, where an independent global
/// search would snap to whichever fold is momentarily closest and tear the
/// pcurve into a self-crossing zig-zag. The caller compares the returned
/// `distance` against the global answer and only adopts this result when it is
/// geometrically just as valid, so a bad seed can never make a fit worse.
pub fn project_point_to_surface_seeded(
    surface: &NurbsSurface,
    point: Vec3,
    seed_u: f64,
    seed_v: f64,
) -> Result<SurfaceProjection, String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let result = surface_newton(
        surface,
        point,
        seed_u,
        seed_v,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
    )?;
    let projected = surface.evaluate(result.u, result.v)?;
    Ok(SurfaceProjection {
        u: result.u,
        v: result.v,
        point: projected,
        distance: projected.sub(point).length(),
    })
}

/// The foot of `point` on the branch of `seed`: the closed form on an analytic
/// carrier, otherwise Newton from the caller's own `(u, v)`
/// ([`project_point_to_surface_seeded`]), WITHOUT the global search.
///
/// **Which callers.** One that already holds a foot on this surface from a
/// geometrically adjacent query and whose output needs continuity with it: a
/// pcurve's previous sample, a march's previous station, a Newton's previous
/// iterate. [`project_point_to_surface`] answers a different question — where
/// on the WHOLE surface the point is nearest — and on a carrier that comes back
/// near itself, or for an off-surface query about as far from two sheets, that
/// answer is a foot on another branch: a pcurve sample on the wrong sheet, an
/// iterate that jumps. A caller with no such foot (a vertex, a probe, the first
/// station) asks the nearest-point question and calls the projector.
///
/// **An analytic carrier** answers in closed form, as the projector does. Its
/// only places with two equally valid feet are its seam, its poles and its focal
/// set (axis, centre, tube circle), and the closed form answers those exactly as
/// the nearest-point query always has; no march on one changes.
///
/// **What it does not promise.** It is a local answer: its distance is an upper
/// bound, and a Newton that stalls returns its best iterate without saying so.
/// Every caller keeps its own on-surface check.
///
/// **Instruments**, each read once. `BREP_PROJECTION_FROM_SEED=0` returns
/// [`project_point_to_surface`]'s answer at every caller (the reading before
/// 2026-09-15, for a same-binary A/B); `BREP_PROJECTION_FROM_SEED_SITES=<a,b,…>`
/// keeps the seeded answer only at the call sites whose `file:line` contains
/// one of the substrings. Under `BREP_PROJECTION_AUDIT=<dir>` every query whose
/// seeded foot lies farther than the on-surface floor from the nearest point's
/// foot appends one line to `<dir>/projection-seed-audit-<pid>.tsv`. That
/// comparand is the nearest-point projector under the same caller, so its own
/// replacements land in `projection-audit-<pid>.tsv` at this site's line: they
/// are the audit's, not an answer the caller took.
#[track_caller]
pub(crate) fn project_point_to_surface_from_seed(
    surface: &NurbsSurface,
    point: Vec3,
    seed: [f64; 2],
) -> Result<SurfaceProjection, String> {
    if let Some(analytic) = surface.analytic() {
        if let Some(projection) = analytic.project(surface, point) {
            return Ok(projection);
        }
    }
    let caller = std::panic::Location::caller();
    if !seeded_answer_admitted(caller) {
        return project_point_to_surface_general(surface, point);
    }
    let seeded = project_point_to_surface_seeded(surface, point, seed[0], seed[1])?;
    if seed_audit_enabled() {
        let nearest = project_point_to_surface_general(surface, point)?;
        if seeded.point.sub(nearest.point).length() > LINEAR_TOLERANCE {
            seed_audit_record(caller, surface, seed, &seeded, &nearest);
        }
    }
    Ok(seeded)
}

/// `BREP_PROJECTION_FROM_SEED=0` / `BREP_PROJECTION_FROM_SEED_SITES`: see
/// [`project_point_to_surface_from_seed`].
fn seeded_answer_admitted(caller: &std::panic::Location<'_>) -> bool {
    static SITES: std::sync::OnceLock<Option<Option<Vec<String>>>> = std::sync::OnceLock::new();
    let sites = SITES.get_or_init(|| {
        if std::env::var("BREP_PROJECTION_FROM_SEED").as_deref() == Ok("0") {
            return None;
        }
        Some(std::env::var("BREP_PROJECTION_FROM_SEED_SITES").ok().map(|value| {
            value.split(',').map(str::to_string).filter(|site| !site.is_empty()).collect()
        }))
    });
    match sites {
        None => false,
        Some(None) => true,
        Some(Some(only)) => {
            let location = format!("{}:{}", caller.file(), caller.line());
            only.iter().any(|site| location.contains(site.as_str()))
        }
    }
}

fn seed_audit_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var_os("BREP_PROJECTION_AUDIT").is_some())
}

/// One line per seeded answer whose foot is not the nearest point's: the call
/// site, both distances, the seed, both feet's `(u, v)`, the separation of the
/// two feet and the surface's shape.
fn seed_audit_record(
    caller: &std::panic::Location<'_>,
    surface: &NurbsSurface,
    seed: [f64; 2],
    seeded: &SurfaceProjection,
    nearest: &SurfaceProjection,
) {
    use std::io::Write;
    static SINK: std::sync::OnceLock<Option<std::sync::Mutex<std::fs::File>>> =
        std::sync::OnceLock::new();
    let sink = SINK.get_or_init(|| {
        let directory = std::env::var_os("BREP_PROJECTION_AUDIT")?;
        let path = std::path::Path::new(&directory)
            .join(format!("projection-seed-audit-{}.tsv", std::process::id()));
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok()
            .map(std::sync::Mutex::new)
    });
    if let Some(file) = sink {
        if let Ok(mut file) = file.lock() {
            let _ = writeln!(
                file,
                "{}:{}\t{:.6e}\t{:.6e}\t{:.9}\t{:.9}\t{:.9}\t{:.9}\t{:.9}\t{:.9}\t{:.6e}\t{}x{} cp {}x{}",
                caller.file(),
                caller.line(),
                seeded.distance,
                nearest.distance,
                seed[0],
                seed[1],
                seeded.u,
                seeded.v,
                nearest.u,
                nearest.v,
                seeded.point.sub(nearest.point).length(),
                surface.degree_u,
                surface.degree_v,
                surface.control_points.len(),
                surface.control_points.first().map(Vec::len).unwrap_or(0),
            );
        }
    }
}

/// The Newton seed grid for general projection: (u, v, point) samples over
/// every knot span. A pure function of the surface, cached on it — building
/// the grid costs hundreds of evaluations and projection is the hottest
/// entry point in the kernel.
fn projection_seed_grid(surface: &NurbsSurface) -> Result<&[(f64, f64, Vec3)], String> {
    if let Some(grid) = surface.projection_grid.get() {
        return Ok(grid);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let mut breaks_u = vec![u0];
    breaks_u.extend(interior_knots(&surface.knots_u, surface.degree_u));
    breaks_u.push(u1);
    let mut breaks_v = vec![v0];
    breaks_v.extend(interior_knots(&surface.knots_v, surface.degree_v));
    breaks_v.push(v1);
    let samples_u = if closed_u {
        8usize.max(surface.degree_u * 4)
    } else {
        3usize.max(surface.degree_u + 1)
    };
    let samples_v = if closed_v {
        8usize.max(surface.degree_v * 4)
    } else {
        3usize.max(surface.degree_v + 1)
    };
    let mut grid = Vec::new();
    for u_pair in breaks_u.windows(2) {
        for v_pair in breaks_v.windows(2) {
            for i in 0..=samples_u {
                for j in 0..=samples_v {
                    let u = u_pair[0] + (u_pair[1] - u_pair[0]) * i as f64 / samples_u as f64;
                    let v = v_pair[0] + (v_pair[1] - v_pair[0]) * j as f64 / samples_v as f64;
                    grid.push((u, v, surface.evaluate(u, v)?));
                }
            }
        }
    }
    Ok(surface.projection_grid.get_or_init(|| grid))
}

/// The dense fallback grid for degenerate general projection: (u, v, point)
/// over a `(count_u+1) × (count_v+1)` lattice with `count_u/count_v` derived
/// from the surface's knot spans. A pure function of the surface, cached on
/// it — on metre-unit-mm parts the degenerate fallback fires ~1.7M times and
/// each re-evaluated all ~561 lattice points; caching them makes the argmin a
/// pure distance scan over reused points (bit-identical seed).
fn projection_dense_grid(surface: &NurbsSurface) -> Result<&[(f64, f64, Vec3)], String> {
    if let Some(grid) = surface.projection_dense_grid.get() {
        return Ok(grid);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let mut breaks_u = vec![u0];
    breaks_u.extend(interior_knots(&surface.knots_u, surface.degree_u));
    breaks_u.push(u1);
    let mut breaks_v = vec![v0];
    breaks_v.extend(interior_knots(&surface.knots_v, surface.degree_v));
    breaks_v.push(v1);
    let count_u = 32usize.max((breaks_u.len() - 1) * 8);
    let count_v = 16usize.max((breaks_v.len() - 1) * 8);
    let mut grid = Vec::with_capacity((count_u + 1) * (count_v + 1));
    for i in 0..=count_u {
        for j in 0..=count_v {
            let u = u0 + (u1 - u0) * i as f64 / count_u as f64;
            let v = v0 + (v1 - v0) * j as f64 / count_v as f64;
            grid.push((u, v, surface.evaluate(u, v)?));
        }
    }
    Ok(surface.projection_dense_grid.get_or_init(|| grid))
}

// Boundary-ring cache geometry: 4 sides × 22 exponents × 33 indices. The scan
// positions depend only on the surface domain and these fixed integers, so the
// evaluated points are a pure function of the surface and cacheable.
const RING_EXPONENTS: usize = 22;
const RING_INDICES: usize = 33; // index 0..=32

#[inline]
fn ring_slot(side: usize, exponent_index: usize, index: usize) -> usize {
    (side * RING_EXPONENTS + exponent_index) * RING_INDICES + index
}

/// The boundary-ring sample points for the degenerate projection fallback,
/// flattened by `ring_slot(side, exponent-1, index)`. Sides: 0 = v-low
/// (`v = v0 + (v1-v0)·offset`), 1 = v-high (`v1 - (v1-v0)·offset`), 2 = u-low
/// (`u0 + (u1-u0)·offset`), 3 = u-high (`u1 - (u1-u0)·offset`), where
/// `offset = 1/2^exponent`. A pure function of the surface, cached on it — the
/// fallback's boundary ring re-evaluated these on every one of ~1.7M calls.
fn projection_ring_grid(surface: &NurbsSurface) -> Result<&[Vec3], String> {
    if let Some(grid) = surface.projection_ring_grid.get() {
        return Ok(grid);
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let mut grid = vec![Vec3::default(); 4 * RING_EXPONENTS * RING_INDICES];
    for exponent in 1..=RING_EXPONENTS as i32 {
        let offset = 1.0 / 2f64.powi(exponent);
        let e = exponent as usize - 1;
        let v_lo = v0 + (v1 - v0) * offset;
        let v_hi = v1 - (v1 - v0) * offset;
        let u_lo = u0 + (u1 - u0) * offset;
        let u_hi = u1 - (u1 - u0) * offset;
        for index in 0..=32 {
            let u = u0 + (u1 - u0) * index as f64 / 32.0;
            grid[ring_slot(0, e, index as usize)] = surface.evaluate(u, v_lo)?;
            grid[ring_slot(1, e, index as usize)] = surface.evaluate(u, v_hi)?;
        }
        for index in 0..=32 {
            let v = v0 + (v1 - v0) * index as f64 / 32.0;
            grid[ring_slot(2, e, index as usize)] = surface.evaluate(u_lo, v)?;
            grid[ring_slot(3, e, index as usize)] = surface.evaluate(u_hi, v)?;
        }
    }
    Ok(surface.projection_ring_grid.get_or_init(|| grid))
}

/// The LOCAL answer: Newton from the seed grid's single nearest sample (or the
/// linear-revolution foot), with the degenerate-parameterization fallbacks.
///
/// This is the whole of what the general projector returned before
/// 2026-09-14, and it is a local minimum, not the minimum: on a general patch
/// nearly closed in one direction the nearest sample can sit on the far end,
/// and Newton converges there (abc_00000026's thread flanks, 2.16e-2 .. 4.1e-2
/// against a true 1e-13). [`project_point_to_surface_general`] runs it first
/// and keeps its answer wherever no other basin beats it.
fn nearest_seed_answer(
    surface: &NurbsSurface,
    point: Vec3,
    [u0, u1, v0, v1]: [f64; 4],
    closed_u: bool,
    closed_v: bool,
) -> Result<NewtonResult, String> {
    let mut best = NewtonResult {
        u: u0,
        v: v0,
        distance_squared: f64::INFINITY,
        converged: false,
    };
    for &(u, v, sample) in projection_seed_grid(surface)? {
        let distance_squared = sample.sub(point).length_squared();
        if distance_squared < best.distance_squared {
            best.u = u;
            best.v = v;
            best.distance_squared = distance_squared;
        }
    }
    if let Some(revolution) = project_linear_revolution(surface, point)? {
        if revolution.distance * revolution.distance < best.distance_squared {
            best.u = revolution.u;
            best.v = revolution.v;
            best.distance_squared = revolution.distance * revolution.distance;
        }
    }
    let first_newton = surface_newton(
        surface,
        point,
        best.u,
        best.v,
        [u0, u1, v0, v1],
        closed_u,
        closed_v,
    )?;
    if first_newton.distance_squared < best.distance_squared {
        best = first_newton;
    }
    let (_, su, sv) = surface.deriv1(best.u, best.v)?;
    let degenerate = su.cross(sv).length() <= 1e-5 * (1.0 + su.length() + sv.length());
    if degenerate || !first_newton.converged {
        let mut grid = best;
        for &(u, v, sample) in projection_dense_grid(surface)? {
            let distance_squared = sample.sub(point).length_squared();
            if distance_squared < grid.distance_squared {
                grid.u = u;
                grid.v = v;
                grid.distance_squared = distance_squared;
            }
        }
        if grid.distance_squared < best.distance_squared {
            best = grid;
        }
        let polished = surface_newton(
            surface,
            point,
            grid.u,
            grid.v,
            [u0, u1, v0, v1],
            closed_u,
            closed_v,
        )?;
        if polished.distance_squared < best.distance_squared {
            best = polished;
        }

        let (_, su, sv) = surface.deriv1(best.u, best.v)?;
        if su.cross(sv).length() <= 1e-5 * (1.0 + su.length() + sv.length()) {
            let ring_grid = projection_ring_grid(surface)?;
            let mut ring = best;
            // Read the pre-evaluated ring points at the exact same parameter
            // positions the live scans used (side/exponent/index → `ring_slot`),
            // preserving scan order and argmin tie-breaking for bit-identity.
            let scan = |side: usize,
                        exponent_index: usize,
                        along_u: bool,
                        fixed: f64,
                        candidate: &mut NewtonResult| {
                for index in 0..=32 {
                    let moving = if along_u {
                        u0 + (u1 - u0) * index as f64 / 32.0
                    } else {
                        v0 + (v1 - v0) * index as f64 / 32.0
                    };
                    let (u, v) = if along_u {
                        (moving, fixed)
                    } else {
                        (fixed, moving)
                    };
                    let distance_squared = ring_grid
                        [ring_slot(side, exponent_index, index as usize)]
                    .sub(point)
                    .length_squared();
                    if distance_squared < candidate.distance_squared {
                        candidate.u = u;
                        candidate.v = v;
                        candidate.distance_squared = distance_squared;
                    }
                }
            };
            for exponent in 1..=22 {
                let offset = 1.0 / 2f64.powi(exponent);
                let e = exponent as usize - 1;
                if (best.v - v0).abs() <= (v1 - v0) * 0.02 {
                    scan(0, e, true, v0 + (v1 - v0) * offset, &mut ring);
                }
                if (best.v - v1).abs() <= (v1 - v0) * 0.02 {
                    scan(1, e, true, v1 - (v1 - v0) * offset, &mut ring);
                }
                if (best.u - u0).abs() <= (u1 - u0) * 0.02 {
                    scan(2, e, false, u0 + (u1 - u0) * offset, &mut ring);
                }
                if (best.u - u1).abs() <= (u1 - u0) * 0.02 {
                    scan(3, e, false, u1 - (u1 - u0) * offset, &mut ring);
                }
            }
            if ring.distance_squared < best.distance_squared {
                best = ring;
            }
            let polished = surface_newton(
                surface,
                point,
                ring.u,
                ring.v,
                [u0, u1, v0, v1],
                closed_u,
                closed_v,
            )?;
            if polished.distance_squared < best.distance_squared {
                best = polished;
            }
        }
    }
    Ok(best)
}

/// Escape hatch for A/B measurement: `BREP_PROJECTION_GLOBAL=0` returns the
/// local answer alone ([`nearest_seed_answer`], the pre-2026-09-14 projector),
/// so a population run can be repeated with and without the global stage from
/// one binary. Read once.
fn global_stage_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| std::env::var("BREP_PROJECTION_GLOBAL").as_deref() != Ok("0"))
}

/// `BREP_PROJECTION_GLOBAL_SITES=<a,b,…>`: the global stage's answer is kept
/// only for call sites whose `file:line` contains one of the comma-separated
/// substrings, and the local answer is returned everywhere else — so a result
/// that moves can be attributed to the call site that moved it.
/// `BREP_PROJECTION_GLOBAL_EXCEPT=<a,b,…>` is the complement: every site but
/// those. Unset: every site. Read once.
fn global_answer_admitted(caller: &std::panic::Location<'_>) -> bool {
    fn list(name: &str) -> Option<Vec<String>> {
        std::env::var(name)
            .ok()
            .map(|value| value.split(',').map(str::to_string).filter(|site| !site.is_empty()).collect())
    }
    static SITES: std::sync::OnceLock<(Option<Vec<String>>, Option<Vec<String>>)> =
        std::sync::OnceLock::new();
    let (only, except) = SITES.get_or_init(|| {
        (list("BREP_PROJECTION_GLOBAL_SITES"), list("BREP_PROJECTION_GLOBAL_EXCEPT"))
    });
    if only.is_none() && except.is_none() {
        return true;
    }
    let location = format!("{}:{}", caller.file(), caller.line());
    let named = |sites: &Vec<String>| sites.iter().any(|site| location.contains(site.as_str()));
    only.as_ref().is_none_or(named) && !except.as_ref().is_some_and(named)
}

/// `BREP_PROJECTION_AUDIT=<dir>`: every query on which the global stage
/// replaces the local answer appends one tab-separated line to
/// `<dir>/projection-audit-<pid>.tsv` — the call site, the local and the
/// global distance, both feet's `(u, v)` and the surface's shape. Per process,
/// because the case gate forks a child per case and loses its stderr.
fn audit_record(caller: &std::panic::Location<'_>, surface: &NurbsSurface, local: &NewtonResult, global: &NewtonResult) {
    use std::io::Write;
    static SINK: std::sync::OnceLock<Option<std::sync::Mutex<std::fs::File>>> =
        std::sync::OnceLock::new();
    let sink = SINK.get_or_init(|| {
        let directory = std::env::var_os("BREP_PROJECTION_AUDIT")?;
        let path = std::path::Path::new(&directory)
            .join(format!("projection-audit-{}.tsv", std::process::id()));
        std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(path)
            .ok()
            .map(std::sync::Mutex::new)
    });
    if let Some(file) = sink {
        if let Ok(mut file) = file.lock() {
            let _ = writeln!(
                file,
                "{}:{}\t{:.6e}\t{:.6e}\t{:.9}\t{:.9}\t{:.9}\t{:.9}\t{}x{} cp {}x{}",
                caller.file(),
                caller.line(),
                local.distance_squared.sqrt(),
                global.distance_squared.sqrt(),
                local.u,
                local.v,
                global.u,
                global.v,
                surface.degree_u,
                surface.degree_v,
                surface.control_points.len(),
                surface.control_points.first().map(Vec::len).unwrap_or(0),
            );
        }
    }
}

/// The point of a general surface nearest `point`: the least local minimum of
/// the distance over the surface's whole domain.
///
/// **What it does.** First the local answer ([`nearest_seed_answer`]: Newton
/// from the nearest seed-grid sample, with its degenerate fallbacks — the whole
/// projector before 2026-09-14). Then the global stage ([`global_minimum`]):
/// the domain is cut into the cells of the projection lattice (every knot span
/// sampled `max(3, degree + 1)` times in an open direction and
/// `max(8, 4 × degree)` times in a closed one — the seed grid's own density),
/// and each cell is enclosed in a box built from the control points of the
/// surface REFINED to that cell. A NURBS patch with positive weights lies in
/// the convex hull of the control points that support it, so the distance from
/// the query to a cell's box is a rigorous LOWER bound on the distance to every
/// surface point of the cell. Every cell whose bound is below the best answer
/// so far is a candidate basin, and is refined by a descent confined to the
/// cell ([`confined_descent`]: Newton where the Hessian is positive definite,
/// the metric-scaled gradient where it is not, halved until the distance does
/// not rise, so it can end at a minimum but not at a saddle or a maximum),
/// seeded at the cell's nearest lattice node and continued over the domain
/// when the cell's least point is on its edge. The least answer wins. A closed
/// direction's seam is not special: the lattice spans the whole period, the
/// cells on either side of the seam are ordinary cells, and the continuation
/// wraps.
///
/// **What is guaranteed.**
/// * The answer is a point ON the surface, so its distance never under-reads.
/// * Every cell that could hold a point nearer than the answer by more than the
///   resolution has been descended in, unless it already held a verified
///   minimum: a cell left out was excluded by its hull bound, which cannot
///   over-read. [`GlobalStats::hull_lower_bound`] is that certificate: the
///   true distance lies in `[hull_lower_bound, answer]`, always.
/// * **The one step not certified in general: one basin in the cell that holds
///   the least minimum.** A descent finds the basin of its seed. A second basin
///   in any OTHER cell costs nothing — that cell's better basin is still no
///   nearer than the least minimum — but a second local minimum in the cell
///   holding the least minimum can catch the descent seeded at that cell's
///   nearest lattice node, and the least minimum is missed by at most
///   `answer − hull_lower_bound`. That takes a query past the focal distance of
///   part of the cell (farther than its radius of normal curvature on the
///   query's side, where the distance's Hessian `G − d·II` stops being positive
///   definite) or a cell whose image turns back toward the query. A descent that
///   runs out of iterations without a verified minimum fails the same way, with
///   the same bound. Seeding every corner of every candidate cell was measured
///   and changed no answer on the bench.
/// * **Certified on the analytic carriers** — the sphere, torus, cylinder and
///   cone, each a revolution of a circle or a line, and the plane. A lattice cell
///   there is an (azimuth × generatrix) rectangle under half a turn wide. In the
///   cell holding the least minimum the azimuthal derivative of the squared
///   distance, `2 ρ(v) ρ_P sin(λ − λ_P)`, vanishes only on the query's own
///   meridian, and moving toward that meridian lowers the distance from every
///   other point of the cell, so each local minimum over the cell is a local
///   minimum of the planar distance from the query's meridian image to the
///   generatrix over the cell's `v` range — exactly one on a circle arc under
///   half a turn or on a segment. At the focal sets (the axis, the sphere's
///   centre, the tube's centre circle) the minimum is a continuum of equal
///   distances and every foot is right. A general profile of revolution and a
///   general patch are not certified.
/// * Where the local answer is already the least minimum, it is returned
///   unchanged, bit for bit: the global stage replaces it only with a point
///   nearer by more than the resolution.
///
/// **The two numbers it reads.** The *on-surface floor* is `LINEAR_TOLERANCE`,
/// INHERITED unchanged: it is the local Newton's own stopping residual, and a
/// local answer within it of the query is on the surface and nothing is
/// searched. The *resolution* is the one new number, and it is the surface's
/// own: `(degree_u + 1)(degree_v + 1) · ε · scale`, `scale` the largest
/// control-point coordinate magnitude — the rounding of one evaluation, which
/// sums that many terms of that size. Every box is grown by it, and no
/// improvement finer than it is taken.
///
/// **What it costs.** A query settled by the floor costs what the local answer
/// always cost. Any other query builds the cell enclosure once per surface (one
/// knot refinement of the net to the lattice, cached on the surface beside the
/// seed grid), walks a bounding-sphere tree over the cells, and runs one
/// descent per candidate cell the hull bound does not exclude.
#[track_caller]
pub(crate) fn project_point_to_surface_general(
    surface: &NurbsSurface,
    point: Vec3,
) -> Result<SurfaceProjection, String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let local = nearest_seed_answer(surface, point, [u0, u1, v0, v1], closed_u, closed_v)?;
    let mut best = local;
    if global_stage_enabled() {
        global_minimum(
            surface,
            point,
            [u0, u1, v0, v1],
            (closed_u, closed_v),
            &mut best,
            &mut GlobalStats::default(),
        )?;
        if best.u != local.u || best.v != local.v {
            let caller = std::panic::Location::caller();
            if global_answer_admitted(caller) {
                audit_record(caller, surface, &local, &best);
            } else {
                best = local;
            }
        }
    }
    let projected = surface.evaluate(best.u, best.v)?;
    Ok(SurfaceProjection {
        u: best.u,
        v: best.v,
        point: projected,
        distance: projected.sub(point).length(),
    })
}

/// The single-seed inversion by itself: the closed form on an analytic
/// carrier, otherwise [`nearest_seed_answer`] — Newton from the seed grid's
/// nearest sample, WITHOUT the global stage. Its distance is an upper bound and
/// its foot a local minimum. It is not a nearest-point query: it exists for a
/// caller that asks a question ABOUT that inversion, and there is one —
/// `joint_branch_pcurves` (`io/step_import/builder/rims.rs`) offers a loop the
/// joint branch only when this inversion alternates between branches along it,
/// which it does on a carrier that comes back within one seed-grid spacing of
/// itself (`abc_00000026`'s nearly closed thread flanks).
pub(crate) fn project_point_to_surface_from_nearest_seed(
    surface: &NurbsSurface,
    point: Vec3,
) -> Result<SurfaceProjection, String> {
    if let Some(analytic) = surface.analytic() {
        if let Some(projection) = analytic.project(surface, point) {
            return Ok(projection);
        }
    }
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let local = nearest_seed_answer(surface, point, [u0, u1, v0, v1], closed_u, closed_v)?;
    let projected = surface.evaluate(local.u, local.v)?;
    Ok(SurfaceProjection {
        u: local.u,
        v: local.v,
        point: projected,
        distance: projected.sub(point).length(),
    })
}

/// Both answers for one query, and what the global stage did — the bench's
/// seam (`examples/projector_bench.rs`). Not for kernel callers.
#[doc(hidden)]
pub fn project_point_to_surface_general_lanes(
    surface: &NurbsSurface,
    point: Vec3,
    run_global: bool,
) -> Result<(SurfaceProjection, GlobalStats), String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let (closed_u, closed_v) = surface.closed_directions()?;
    let mut best = nearest_seed_answer(surface, point, [u0, u1, v0, v1], closed_u, closed_v)?;
    let mut stats = GlobalStats::default();
    if run_global {
        global_minimum(surface, point, [u0, u1, v0, v1], (closed_u, closed_v), &mut best, &mut stats)?;
    }
    let projected = surface.evaluate(best.u, best.v)?;
    Ok((
        SurfaceProjection {
            u: best.u,
            v: best.v,
            point: projected,
            distance: projected.sub(point).length(),
        },
        stats,
    ))
}

/// How many basins the cells of one query hold — the bench's measure of the
/// global stage's one uncertified step (see [`project_point_to_surface_general`]).
#[doc(hidden)]
#[derive(Clone, Debug, Default)]
pub struct BasinCensus {
    /// Cells whose hull bound is below the answer's distance: every cell the
    /// global stage could have descended in.
    pub cells: usize,
    /// Descents run: one from each corner of those cells and one from the middle.
    pub descents: usize,
    /// Descents that stopped without a verified minimum.
    pub unconverged: usize,
    /// Cells where two verified minima differ in distance by more than `bar`.
    /// Distances within the on-surface floor count as equal: the descent stops
    /// there, so two feet under it are one foot polished to different depths.
    pub cells_with_two_basins: usize,
    /// Of those, cells whose rectangle holds the least foot any descent found:
    /// the only cells where a second basin can cost the answer anything.
    pub foot_cells_with_two_basins: usize,
    /// The least distance any descent reached, and where.
    pub least: f64,
    pub least_uv: (f64, f64),
    /// Every descent in a two-basin cell that holds the least foot: the cell,
    /// its rectangle, the seed, where it stopped, its distance, and whether that
    /// is a verified minimum.
    pub foot_cell_descents: Vec<(usize, [f64; 4], (f64, f64), (f64, f64), f64, bool)>,
}

/// Descend in every cell whose hull bound is below `answer_distance` from each
/// of its corners and its middle, and count the cells whose verified minima
/// disagree by more than `bar`. For the bench only: it costs five descents per
/// cell, which the projector itself was measured not to need.
#[doc(hidden)]
pub fn project_point_to_surface_basin_census(
    surface: &NurbsSurface,
    point: Vec3,
    answer_distance: f64,
    bar: f64,
) -> Result<BasinCensus, String> {
    let mut census = BasinCensus {
        least: f64::INFINITY,
        ..BasinCensus::default()
    };
    let Some(cells) = projection_cells(surface)? else {
        return Ok(census);
    };
    let columns = cells.vs.len() - 1;
    let mut spreads: Vec<(usize, f64)> = Vec::new();
    let mut runs: Vec<(usize, [f64; 4], (f64, f64), (f64, f64), f64, bool)> = Vec::new();
    for (cell, cell_box) in cells.boxes.iter().enumerate() {
        if cell_box.lower_bound(point) >= answer_distance {
            continue;
        }
        census.cells += 1;
        let (row, column) = (cell / columns, cell % columns);
        let rectangle = [cells.us[row], cells.us[row + 1], cells.vs[column], cells.vs[column + 1]];
        let middle = ((rectangle[0] + rectangle[1]) * 0.5, (rectangle[2] + rectangle[3]) * 0.5);
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for seed in [
            (rectangle[0], rectangle[2]),
            (rectangle[1], rectangle[2]),
            (rectangle[0], rectangle[3]),
            (rectangle[1], rectangle[3]),
            middle,
        ] {
            census.descents += 1;
            let (found, _) = confined_descent(surface, point, seed, rectangle, (false, false))?;
            let distance = found.distance_squared.sqrt();
            runs.push((cell, rectangle, seed, (found.u, found.v), distance, found.converged));
            if distance < census.least {
                census.least = distance;
                census.least_uv = (found.u, found.v);
            }
            if !found.converged {
                census.unconverged += 1;
                continue;
            }
            low = low.min(distance.max(LINEAR_TOLERANCE));
            high = high.max(distance.max(LINEAR_TOLERANCE));
        }
        if high - low > bar {
            spreads.push((cell, high - low));
        }
    }
    census.cells_with_two_basins = spreads.len();
    let (u, v) = census.least_uv;
    let foot_cells: Vec<usize> = spreads
        .iter()
        .map(|&(cell, _)| cell)
        .filter(|&cell| {
            let (row, column) = (cell / columns, cell % columns);
            cells.us[row] <= u && u <= cells.us[row + 1] && cells.vs[column] <= v && v <= cells.vs[column + 1]
        })
        .collect();
    census.foot_cells_with_two_basins = foot_cells.len();
    census.foot_cell_descents = runs.into_iter().filter(|run| foot_cells.contains(&run.0)).collect();
    Ok(census)
}

/// What the global stage did on one query.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GlobalStats {
    /// The local answer was within the on-surface floor; nothing searched.
    pub settled_by_floor: bool,
    /// Cells whose hull bound admitted them.
    pub candidates: usize,
    /// Confined Newtons run (candidates not already holding a converged foot).
    pub refinements: usize,
    /// Continuations across the domain from a cell edge.
    pub continuations: usize,
    /// The global stage replaced the local answer.
    pub replaced: bool,
    /// The least hull bound over every cell — the true distance is never
    /// below it (0 when the floor settled the query).
    pub hull_lower_bound: f64,
}

/// One lattice cell's enclosure: an oriented box around the control points
/// that support the cell, in an orthonormal frame fitted to the cell.
#[derive(Clone, Debug)]
struct CellBox {
    axes: [Vec3; 3],
    low: [f64; 3],
    high: [f64; 3],
}

impl CellBox {
    /// Distance from `point` to the box: a lower bound on the distance to
    /// every surface point of the cell.
    fn lower_bound(&self, point: Vec3) -> f64 {
        let mut squared = 0.0;
        for axis in 0..3 {
            let coordinate = point.dot(self.axes[axis]);
            let excess = (self.low[axis] - coordinate).max(coordinate - self.high[axis]).max(0.0);
            squared += excess * excess;
        }
        squared.sqrt()
    }
}

/// A node of the bounding-sphere tree over the cells. A leaf names its cell;
/// an internal node its two children.
#[derive(Clone, Copy, Debug)]
struct CellTreeNode {
    centre: Vec3,
    radius: f64,
    children: Option<(u32, u32)>,
    cell: u32,
}

/// The cached enclosure of every projection lattice cell (see
/// [`project_point_to_surface_general`]).
#[derive(Clone, Debug)]
pub(crate) struct ProjectionCells {
    us: Vec<f64>,
    vs: Vec<f64>,
    /// `us.len() × vs.len()` lattice points, row-major in u.
    nodes: Vec<Vec3>,
    /// `(us.len() - 1) × (vs.len() - 1)` cells, row-major in u.
    boxes: Vec<CellBox>,
    /// The tree's root is the LAST node.
    tree: Vec<CellTreeNode>,
    resolution: f64,
}

/// The projection lattice's parameters along one direction: every knot span
/// cut into `samples` equal pieces, the seed grid's density, shared span ends
/// listed once.
fn lattice_parameters(knots: &[f64], degree: usize, closed: bool) -> Vec<f64> {
    let [start, end] = crate::curve::knot_domain(knots, degree);
    let samples = if closed {
        8usize.max(degree * 4)
    } else {
        3usize.max(degree + 1)
    };
    let mut breaks = vec![start];
    breaks.extend(interior_knots(knots, degree));
    breaks.push(end);
    let mut parameters = Vec::with_capacity((breaks.len() - 1) * samples + 1);
    for pair in breaks.windows(2) {
        for index in 0..samples {
            parameters.push(pair[0] + (pair[1] - pair[0]) * index as f64 / samples as f64);
        }
    }
    parameters.push(end);
    parameters
}

/// Raise every lattice parameter to full multiplicity in the knot vector, so
/// each lattice cell is one Bézier piece of the refined net.
///
/// `net[i]` is the i-th control point along the refined direction, held as a
/// row across the other direction, and every row takes the same combination.
/// Knot refinement in one pass (Piegl & Tiller, *The NURBS Book*, A5.4), in
/// homogeneous coordinates, so a rational net refines exactly and the cost is
/// linear in the refined net's size.
fn refine_to_lattice(net: &mut Vec<Vec<Vec4>>, knots: &mut Vec<f64>, degree: usize, parameters: &[f64]) {
    let p = degree;
    let n = net.len() - 1;
    let mut inserted = Vec::new();
    for &parameter in parameters {
        let multiplicity = knots.iter().filter(|&&knot| knot == parameter).count();
        for _ in multiplicity..p {
            inserted.push(parameter);
        }
    }
    if inserted.is_empty() || p == 0 {
        return;
    }
    inserted.sort_by(f64::total_cmp);
    let find_span = |parameter: f64| -> usize {
        if parameter >= knots[n + 1] {
            return n;
        }
        (knots.partition_point(|&knot| knot <= parameter) - 1).clamp(p, n)
    };
    let r = inserted.len() - 1;
    let m = n + p + 1;
    let a = find_span(inserted[0]);
    let b = find_span(inserted[r]) + 1;
    let columns = net[0].len();
    let blank = vec![
        Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        columns
    ];
    let mut refined = vec![blank; n + r + 2];
    let mut refined_knots = vec![0.0; m + r + 2];
    for j in 0..=a - p {
        refined[j] = net[j].clone();
    }
    for j in b - 1..=n {
        refined[j + r + 1] = net[j].clone();
    }
    refined_knots[..=a].copy_from_slice(&knots[..=a]);
    for j in b + p..=m {
        refined_knots[j + r + 1] = knots[j];
    }
    let mut i = b + p - 1;
    let mut k = b + p + r;
    for j in (0..=r).rev() {
        while inserted[j] <= knots[i] && i > a {
            refined[k - p - 1] = net[i - p - 1].clone();
            refined_knots[k] = knots[i];
            k -= 1;
            i -= 1;
        }
        refined[k - p - 1] = refined[k - p].clone();
        for l in 1..=p {
            let index = k - p + l;
            let numerator = refined_knots[k + l] - inserted[j];
            if numerator == 0.0 {
                refined[index - 1] = refined[index].clone();
            } else {
                let alpha = numerator / (refined_knots[k + l] - knots[i - p + l]);
                let (head, tail) = refined.split_at_mut(index);
                for (before, &here) in head[index - 1].iter_mut().zip(&tail[0]) {
                    *before = before.scale(alpha).add(here.scale(1.0 - alpha));
                }
            }
        }
        refined_knots[k] = inserted[j];
        k = k.saturating_sub(1);
    }
    *net = refined;
    *knots = refined_knots;
}

/// The control-point index range supporting the parameter interval
/// `[low, high]`: every basis function non-zero anywhere on it.
fn supporting_range(knots: &[f64], degree: usize, count: usize, low: f64, high: f64) -> (usize, usize) {
    let first_span = knots.partition_point(|&knot| knot <= low).saturating_sub(1);
    let last_span = knots.partition_point(|&knot| knot < high).saturating_sub(1);
    (
        first_span.saturating_sub(degree).min(count - 1),
        last_span.max(first_span).min(count - 1),
    )
}

fn unit_or(vector: Vec3, fallback: Vec3) -> Vec3 {
    let length = vector.length();
    if length > 0.0 && length.is_finite() {
        vector.scale(1.0 / length)
    } else {
        fallback
    }
}

fn enclosing_sphere(a: (Vec3, f64), b: (Vec3, f64)) -> (Vec3, f64) {
    let offset = b.0.sub(a.0);
    let separation = offset.length();
    if separation + b.1 <= a.1 {
        return a;
    }
    if separation + a.1 <= b.1 {
        return b;
    }
    let radius = (separation + a.1 + b.1) * 0.5;
    let centre = a.0.add(offset.scale((radius - a.1) / separation));
    (centre, radius)
}

fn build_cell_tree(
    tree: &mut Vec<CellTreeNode>,
    spheres: &[(Vec3, f64)],
    columns: usize,
    [row_low, row_high, column_low, column_high]: [usize; 4],
) -> u32 {
    if row_high - row_low == 1 && column_high - column_low == 1 {
        let cell = row_low * columns + column_low;
        tree.push(CellTreeNode {
            centre: spheres[cell].0,
            radius: spheres[cell].1,
            children: None,
            cell: cell as u32,
        });
        return (tree.len() - 1) as u32;
    }
    let (first, second) = if row_high - row_low >= column_high - column_low {
        let middle = (row_low + row_high) / 2;
        (
            [row_low, middle, column_low, column_high],
            [middle, row_high, column_low, column_high],
        )
    } else {
        let middle = (column_low + column_high) / 2;
        (
            [row_low, row_high, column_low, middle],
            [row_low, row_high, middle, column_high],
        )
    };
    let first = build_cell_tree(tree, spheres, columns, first);
    let second = build_cell_tree(tree, spheres, columns, second);
    let a = tree[first as usize];
    let b = tree[second as usize];
    // The resolution-inflated leaf spheres already absorb rounding; the merge
    // adds one more rounding of the same order, absorbed by growing the radius
    // by the larger child's own ulp.
    let (centre, radius) = enclosing_sphere((a.centre, a.radius), (b.centre, b.radius));
    tree.push(CellTreeNode {
        centre,
        radius: radius * (1.0 + 4.0 * f64::EPSILON),
        children: Some((first, second)),
        cell: u32::MAX,
    });
    (tree.len() - 1) as u32
}

fn projection_cells(surface: &NurbsSurface) -> Result<Option<&ProjectionCells>, String> {
    if let Some(cells) = surface.projection_cells.get() {
        return Ok(cells.as_ref());
    }
    let cells = build_projection_cells(surface)?;
    Ok(surface.projection_cells.get_or_init(|| cells).as_ref())
}

fn build_projection_cells(surface: &NurbsSurface) -> Result<Option<ProjectionCells>, String> {
    let (closed_u, closed_v) = surface.closed_directions()?;
    let us = lattice_parameters(&surface.knots_u, surface.degree_u, closed_u);
    let vs = lattice_parameters(&surface.knots_v, surface.degree_v, closed_v);
    if us.len() < 2 || vs.len() < 2 {
        return Ok(None);
    }
    let mut scale = 0.0f64;
    for control in surface.control_points.iter().flatten() {
        let point = control.point()?;
        scale = scale.max(point.x.abs()).max(point.y.abs()).max(point.z.abs());
    }
    let resolution =
        ((surface.degree_u + 1) * (surface.degree_v + 1)) as f64 * f64::EPSILON * scale;

    // Refine u (the net's rows), then v (its columns, as rows of the transpose).
    let mut net = surface.control_points.clone();
    let mut knots_u = surface.knots_u.clone();
    refine_to_lattice(&mut net, &mut knots_u, surface.degree_u, &us);
    let columns = net.first().map(Vec::len).unwrap_or(0);
    let mut transposed: Vec<Vec<Vec4>> =
        (0..columns).map(|column| net.iter().map(|row| row[column]).collect()).collect();
    let mut knots_v = surface.knots_v.clone();
    refine_to_lattice(&mut transposed, &mut knots_v, surface.degree_v, &vs);
    let rows = transposed.first().map(Vec::len).unwrap_or(0);
    let columns = transposed.len();
    if rows == 0 || columns == 0 {
        return Ok(None);
    }
    let mut points = vec![Vec3::default(); rows * columns];
    for (column, fibre) in transposed.iter().enumerate() {
        for (row, control) in fibre.iter().enumerate() {
            points[row * columns + column] = control.point()?;
        }
    }

    let mut nodes = Vec::with_capacity(us.len() * vs.len());
    for &u in &us {
        for &v in &vs {
            nodes.push(surface.evaluate(u, v)?);
        }
    }

    let cell_rows = us.len() - 1;
    let cell_columns = vs.len() - 1;
    let mut boxes = Vec::with_capacity(cell_rows * cell_columns);
    let mut spheres = Vec::with_capacity(cell_rows * cell_columns);
    for i in 0..cell_rows {
        let (row_low, row_high) =
            supporting_range(&knots_u, surface.degree_u, rows, us[i], us[i + 1]);
        for j in 0..cell_columns {
            let (column_low, column_high) =
                supporting_range(&knots_v, surface.degree_v, columns, vs[j], vs[j + 1]);
            let at = |row: usize, column: usize| points[row * columns + column];
            let c00 = at(row_low, column_low);
            let c10 = at(row_high, column_low);
            let c01 = at(row_low, column_high);
            let c11 = at(row_high, column_high);
            let along_u = c10.sub(c00).add(c11.sub(c01));
            let along_v = c01.sub(c00).add(c11.sub(c10));
            let e1 = unit_or(along_u, unit_or(along_v, Vec3::new(1.0, 0.0, 0.0)));
            let normal = unit_or(e1.cross(along_v), e1.perpendicular()?);
            let e2 = unit_or(normal.cross(e1), e1.perpendicular()?);
            let e3 = e1.cross(e2);
            let axes = [e1, e2, e3];
            let mut low = [f64::INFINITY; 3];
            let mut high = [f64::NEG_INFINITY; 3];
            for row in row_low..=row_high {
                for column in column_low..=column_high {
                    let point = at(row, column);
                    for axis in 0..3 {
                        let coordinate = point.dot(axes[axis]);
                        low[axis] = low[axis].min(coordinate);
                        high[axis] = high[axis].max(coordinate);
                    }
                }
            }
            for axis in 0..3 {
                low[axis] -= resolution;
                high[axis] += resolution;
            }
            let mut centre = Vec3::default();
            let mut half_diagonal_squared = 0.0;
            for axis in 0..3 {
                centre = centre.add(axes[axis].scale((low[axis] + high[axis]) * 0.5));
                let half = (high[axis] - low[axis]) * 0.5;
                half_diagonal_squared += half * half;
            }
            if !low.iter().chain(high.iter()).all(|value| value.is_finite()) {
                return Ok(None);
            }
            spheres.push((centre, half_diagonal_squared.sqrt() + resolution));
            boxes.push(CellBox { axes, low, high });
        }
    }
    let mut tree = Vec::with_capacity(2 * boxes.len());
    build_cell_tree(&mut tree, &spheres, cell_columns, [0, cell_rows, 0, cell_columns]);
    Ok(Some(ProjectionCells {
        us,
        vs,
        nodes,
        boxes,
        tree,
        resolution,
    }))
}

/// First- and second-order optimality of `(u, v)` for the distance over the
/// rectangle: stationary on the coordinates the rectangle leaves free (the
/// descent's own cosine test), with a positive definite Hessian there. A
/// `periodic` direction has no edge to be pinned against: its seam is interior.
fn optimality(
    derivatives: &[[Vec3; 3]; 3],
    point: Vec3,
    (u, v): (f64, f64),
    [ua, ub, va, vb]: [f64; 4],
    periodic: (bool, bool),
) -> Optimality {
    let residual = derivatives[0][0].sub(point);
    let residual_length = residual.length();
    let su = derivatives[1][0];
    let sv = derivatives[0][1];
    let f = su.dot(residual);
    let g = sv.dot(residual);
    let metric_u = su.length_squared();
    let metric_v = sv.length_squared();
    let stationary_u = metric_u.sqrt() * residual_length <= EPSILON
        || f.abs() <= 1e-10 * metric_u.sqrt() * residual_length;
    let stationary_v = metric_v.sqrt() * residual_length <= EPSILON
        || g.abs() <= 1e-10 * metric_v.sqrt() * residual_length;
    let pinned_u = !periodic.0 && ((u <= ua && f > 0.0) || (u >= ub && f < 0.0));
    let pinned_v = !periodic.1 && ((v <= va && g > 0.0) || (v >= vb && g < 0.0));
    let j00 = derivatives[2][0].dot(residual) + metric_u;
    let j01 = derivatives[1][1].dot(residual) + su.dot(sv);
    let j11 = derivatives[0][2].dot(residual) + metric_v;
    let determinant = j00 * j11 - j01 * j01;
    let convex = match (!pinned_u, !pinned_v) {
        (true, true) => j00 > 0.0 && determinant > 0.0,
        (true, false) => j00 > 0.0,
        (false, true) => j11 > 0.0,
        (false, false) => true,
    };
    Optimality {
        residual_length,
        gradient: (f, g),
        metric: (metric_u, metric_v),
        hessian: (j00, j01, j11),
        determinant,
        pinned: (pinned_u, pinned_v),
        convex,
        minimum: residual_length <= LINEAR_TOLERANCE
            || ((stationary_u || pinned_u) && (stationary_v || pinned_v) && convex),
    }
}

struct Optimality {
    residual_length: f64,
    gradient: (f64, f64),
    metric: (f64, f64),
    hessian: (f64, f64, f64),
    determinant: f64,
    pinned: (bool, bool),
    convex: bool,
    minimum: bool,
}

fn is_verified_minimum(
    surface: &NurbsSurface,
    point: Vec3,
    at: (f64, f64),
    rectangle: [f64; 4],
    periodic: (bool, bool),
) -> Result<bool, String> {
    let derivatives = surface.derivatives_small(at.0, at.1, 2)?;
    Ok(optimality(&derivatives, point, at, rectangle, periodic).minimum)
}

/// A descent to a local minimum of the distance over the rectangle
/// `[ua, ub] × [va, vb]`, from a seed inside it.
///
/// Newton on the gradient converges to ANY critical point — a saddle or a
/// maximum as readily as a minimum — and reads convergence there, so every
/// step here must lower the distance: the Newton step where the Hessian is
/// positive definite on the free coordinates, the metric-scaled gradient step
/// where it is not, halved until the distance does not rise. A coordinate
/// pinned at the rectangle's edge by a gradient pointing out of it is held
/// there and the other takes the one-dimensional step, so a minimum ON the edge
/// converges as one (the Karush-Kuhn-Tucker point) instead of stalling.
///
/// Returns the best iterate, whether it is a verified minimum (stationary on
/// the free coordinates with a positive definite Hessian there, or within the
/// on-surface floor), and whether it lies on the rectangle's edge.
fn confined_descent(
    surface: &NurbsSurface,
    point: Vec3,
    seed: (f64, f64),
    rectangle: [f64; 4],
    periodic: (bool, bool),
) -> Result<(NewtonResult, bool), String> {
    let [ua, ub, va, vb] = rectangle;
    let mut u = fit_parameter(seed.0, ua, ub, periodic.0);
    let mut v = fit_parameter(seed.1, va, vb, periodic.1);
    let mut converged = false;
    let mut current = surface.evaluate(u, v)?.sub(point).length_squared();
    for _ in 0..MAX_NEWTON_ITERATIONS {
        let derivatives = surface.derivatives_small(u, v, 2)?;
        let state = optimality(&derivatives, point, (u, v), rectangle, periodic);
        current = state.residual_length * state.residual_length;
        if state.minimum {
            converged = true;
            break;
        }
        let (f, g) = state.gradient;
        let (metric_u, metric_v) = state.metric;
        let (j00, j01, j11) = state.hessian;
        let determinant = state.determinant;
        let (free_u, free_v) = (!state.pinned.0, !state.pinned.1);
        let (mut du, mut dv) = match (free_u, free_v) {
            (true, true) if state.convex => (
                (-f * j11 + g * j01) / determinant,
                (-g * j00 + f * j01) / determinant,
            ),
            (true, false) if state.convex => (-f / j00, 0.0),
            (false, true) if state.convex => (0.0, -g / j11),
            _ => (
                if free_u && metric_u > 0.0 { -f / metric_u } else { 0.0 },
                if free_v && metric_v > 0.0 { -g / metric_v } else { 0.0 },
            ),
        };
        if !(du.is_finite() && dv.is_finite()) || (du == 0.0 && dv == 0.0) {
            break;
        }
        let mut moved = false;
        // The step actually taken, a whole period removed across a seam.
        let taken = |next: f64, here: f64, low: f64, high: f64, wraps: bool| {
            let raw = next - here;
            if wraps {
                raw - (high - low) * (raw / (high - low)).round()
            } else {
                raw
            }
        };
        for _ in 0..40 {
            let next_u = fit_parameter(u + du, ua, ub, periodic.0);
            let next_v = fit_parameter(v + dv, va, vb, periodic.1);
            if next_u == u && next_v == v {
                break;
            }
            let trial = surface.evaluate(next_u, next_v)?.sub(point).length_squared();
            if trial <= current {
                moved = taken(next_u, u, ua, ub, periodic.0).abs() > 1e-15 * (ub - ua)
                    || taken(next_v, v, va, vb, periodic.1).abs() > 1e-15 * (vb - va);
                u = next_u;
                v = next_v;
                current = trial;
                break;
            }
            du *= 0.5;
            dv *= 0.5;
        }
        if !moved {
            break;
        }
    }
    let result = NewtonResult {
        u,
        v,
        distance_squared: current,
        converged,
    };
    let on_edge =
        (!periodic.0 && (u <= ua || u >= ub)) || (!periodic.1 && (v <= va || v >= vb));
    Ok((result, on_edge))
}

/// The global stage of [`project_point_to_surface_general`]: replace `best`
/// with the least local minimum over every cell the hull bound admits.
#[allow(clippy::too_many_arguments)]
fn global_minimum(
    surface: &NurbsSurface,
    point: Vec3,
    domains: [f64; 4],
    periodic: (bool, bool),
    best: &mut NewtonResult,
    stats: &mut GlobalStats,
) -> Result<(), String> {
    if best.distance_squared.sqrt() <= LINEAR_TOLERANCE {
        stats.settled_by_floor = true;
        return Ok(());
    }
    let Some(cells) = projection_cells(surface)? else {
        return Ok(());
    };
    let resolution = cells.resolution;
    let columns = cells.vs.len() - 1;
    let columns_u = cells.us.len() - 1;
    let mut bound = best.distance_squared.sqrt() - resolution;

    // Every cell whose box comes nearer than the answer; the least bound over
    // what the walk excluded is the certificate.
    let mut candidates: Vec<(f64, usize)> = Vec::new();
    let mut excluded = f64::INFINITY;
    let mut stack: Vec<u32> = Vec::with_capacity(64);
    stack.push((cells.tree.len() - 1) as u32);
    while let Some(index) = stack.pop() {
        let node = cells.tree[index as usize];
        let sphere_bound = (point.sub(node.centre).length() - node.radius).max(0.0);
        if sphere_bound >= bound {
            excluded = excluded.min(sphere_bound);
            continue;
        }
        match node.children {
            Some((first, second)) => {
                stack.push(first);
                stack.push(second);
            }
            None => {
                let cell = node.cell as usize;
                let box_bound = cells.boxes[cell].lower_bound(point);
                if box_bound < bound {
                    candidates.push((box_bound, cell));
                } else {
                    excluded = excluded.min(box_bound);
                }
            }
        }
    }
    stats.candidates = candidates.len();
    candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
    stats.hull_lower_bound = candidates
        .first()
        .map_or(excluded, |&(least, _)| least.min(excluded));

    // Cells already holding a verified minimum need no second descent. A foot
    // names ONE cell even when it sits on a lattice line shared by up to four:
    // a neighbour can hold a better basin of its own in its interior, and it
    // was admitted precisely because its bound beat this answer. The local
    // answer is one when it passes the same test the descent applies.
    let cell_of = |u: f64, v: f64| {
        let row = (cells.us.partition_point(|&value| value <= u).max(1) - 1).min(columns_u - 1);
        let column = (cells.vs.partition_point(|&value| value <= v).max(1) - 1).min(columns - 1);
        row * columns + column
    };
    let mut settled: Vec<usize> = Vec::new();
    if is_verified_minimum(surface, point, (best.u, best.v), domains, periodic)? {
        settled.push(cell_of(best.u, best.v));
    }
    for (cell_bound, cell) in candidates {
        if cell_bound >= bound {
            break;
        }
        let row = cell / columns;
        let column = cell % columns;
        let rectangle = [
            cells.us[row],
            cells.us[row + 1],
            cells.vs[column],
            cells.vs[column + 1],
        ];
        if settled.contains(&cell) {
            continue;
        }
        let mut seed = (rectangle[0], rectangle[2]);
        let mut seed_distance = f64::INFINITY;
        for (node_row, node_column) in
            [(row, column), (row + 1, column), (row, column + 1), (row + 1, column + 1)]
        {
            let distance = cells.nodes[node_row * cells.vs.len() + node_column]
                .sub(point)
                .length_squared();
            if distance < seed_distance {
                seed_distance = distance;
                seed = (cells.us[node_row], cells.vs[node_column]);
            }
        }
        stats.refinements += 1;
        let (confined, on_edge) = confined_descent(surface, point, seed, rectangle, (false, false))?;
        let mut found = confined;
        if on_edge && confined.distance_squared.sqrt() < bound {
            // The cell's least point is on its edge and beats the answer:
            // the basin continues past the cell.
            stats.continuations += 1;
            let (continued, _) =
                confined_descent(surface, point, (confined.u, confined.v), domains, periodic)?;
            if continued.distance_squared < found.distance_squared {
                found = continued;
            }
            if continued.converged {
                settled.push(cell_of(continued.u, continued.v));
            }
        }
        if found.distance_squared.sqrt() < bound {
            *best = found;
            bound = best.distance_squared.sqrt() - resolution;
            stats.replaced = true;
            if best.distance_squared.sqrt() <= LINEAR_TOLERANCE {
                return Ok(());
            }
        }
    }
    Ok(())
}

