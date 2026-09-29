//! CARVE the folded lens out of a blend wall, and sew its CREASE.
//!
//! A wall whose ball-centre curve turns tighter than the ball is a canal
//! surface past its own fold (`blend/fold.rs` measures that, and refuses the
//! walls this module cannot carve).  The envelope such a march sweeps is not
//! the blend: it carries a lens of parameter that lies INSIDE the swept ball
//! volume, so the trimmed envelope — the actual boundary of the rolled ball —
//! is that envelope with the lens cut away and the cut sewn as a crease.
//!
//! # Why the cut is NOT the fold locus
//!
//! Both curves live on the same wall and they are not the same curve.  For
//! `P(s, psi) = c(s) + rho(cos psi N + sin psi B)`,
//!
//! ```text
//! P_s = (1 - rho kappa cos psi) T + tau P_psi
//! ```
//!
//! so the surface is SINGULAR on `1 - rho kappa cos psi = 0` — the cuspidal
//! edge, which is what the offset carve (`offset/fold_locus.rs`) traces for a
//! displaced surface.  Cutting there leaves two sheets that still pass THROUGH
//! each other: the sheets cross on the SELF-INTERSECTION, which lies strictly
//! outside the singular lens and meets it only at the two SWALLOWTAIL points.
//! So the carve here is at the self-intersection, the singular lens is interior
//! to what it discards, and that containment is asserted rather than assumed.
//!
//! # The three facts that make the trace deterministic
//!
//! 1. The section at `u` is a circular arc, so its CENTRE is the circumcentre
//!    of three of its points and the ball centre is available from the surface
//!    alone.
//! 2. With that centre, `g = (S_u x S_v) . (S - centre(u))` evaluates to
//!    `-rho^2 (1 - rho kappa cos psi)`, so the SIGN of one scalar is the fold
//!    locus — no reference normal, no heuristic threshold.
//! 3. The fold locus is a closed lens in `(u, v)` whose two extremal-`v` points
//!    (`g = 0` and `g_u = 0`) ARE the swallowtails, and the swallowtails are
//!    exactly the two ends of the crease.  They are therefore SOLVED, not
//!    marched to — which matters, because the two preimages merge there like a
//!    square root and a march would crawl.
//!
//! Between the ends the crease is marched in `v`, which is monotone along it
//! (the section angle runs from `+psi_max` through `0` to `-psi_max`).
//!
//! # What it refuses
//!
//! A band that reaches a RAIL or a wall boundary is not this construction's:
//! the blend does not end at a crease there, it runs out onto the neighbouring
//! face, which is the runout's territory.  Refused by name with the measured
//! margin.  So is a chamfer section (a straight section has no ball centre to
//! read), a lens whose two ends do not both solve, and a crease whose fitted
//! pcurves cannot be brought inside the caller's tolerance — each with the
//! number it saw.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::{fit, NurbsCurve, NurbsSurface, Vec3};

/// `BREP_BLEND_CARVE_TRACE=1` prints the marched crease and the fit measured on
/// it. The crease becomes a trim boundary, so a reader has to be able to see the
/// samples themselves rather than the interpolant through them.
pub(super) fn carve_trace(message: std::fmt::Arguments<'_>) {
    if std::env::var("BREP_BLEND_CARVE_TRACE").ok().as_deref() == Some("1") {
        eprintln!("{message}");
    }
}

/// How close to a rail the lens may come before the band counts as REACHING
/// it, as a fraction of the wall's own `v` extent. The crease's ends are
/// square-root tangent to the fold locus, so a lens that ends within a
/// thousandth of the rail is a lens whose end is not resolved by anything this
/// module can fit, and the honest answer there is the refusal.
const RAIL_FRACTION: f64 = 1.0e-3;

/// Newton's target on the crease system, in model units.
const CREASE_TOLERANCE: f64 = 1.0e-12;

/// Marched crease samples at the first fit. Chebyshev-spaced in `v`, which
/// clusters like `1/sqrt` at both ends — the profile the merging preimages
/// actually have.
const CREASE_SAMPLES: usize = 33;

/// Ceiling on the marched samples while the fitted pcurves are refined.
const MAX_CREASE_SAMPLES: usize = 513;

/// One sample of the crease: two distinct parameter feet on ONE point.
#[derive(Clone, Copy, Debug)]
pub(super) struct CreaseSample {
    pub(super) first: [f64; 2],
    pub(super) second: [f64; 2],
    pub(super) point: Vec3,
}

/// A wall's folded lens, bounded by the crease.
pub(super) struct WallCarve {
    /// The two swallowtail ends, where the preimages merge.
    pub(super) ends: [[f64; 2]; 2],
    /// The crease, from `ends[0]` to `ends[1]`, ends included.
    pub(super) samples: Vec<CreaseSample>,
    /// Worst `|S(first) - S(second)|` over the marched samples.
    pub(super) march_residual: f64,
    /// The solved `(u1, u2, v2)` at the crease's own middle, which is what a
    /// re-march has to seed from — the middle SAMPLE is not it once the ladder
    /// changes the count.
    pub(super) centre: [f64; 3],
}

/// The carved lens, ready to be sewn: the crease's 3-D curve and the two
/// pcurves that both trace it.
pub(super) struct CarvedCrease {
    pub(super) curve: NurbsCurve,
    /// Runs from `ends[0]` to `ends[1]` on the FIRST sheet.
    pub(super) first: NurbsCurve,
    /// Runs from `ends[0]` to `ends[1]` on the SECOND sheet.
    pub(super) second: NurbsCurve,
    pub(super) start: Vec3,
    pub(super) end: Vec3,
    /// Worst distance between either pcurve's image and the crease curve.
    pub(super) residual: f64,
}

/// The refusal prefix for a fold band this construction cannot carve. Composed
/// under [`super::fold::WALL_FOLDS`] so every caller that already treats a fold
/// as terminal keeps treating this one that way, and minted as the fold's own
/// class ([`super::fold::wall_fold_refusal`]); the band has no faces yet.
pub(super) fn band_refusal(radius: f64, reason: &str) -> KernelRefusal {
    super::fold::wall_fold_refusal(
        Vec::new(),
        format!(
            "{} {radius} fits this edge: the fold band {reason}, which is a runout rather than \
             a crease — the wall does not close on itself there",
            super::fold::WALL_FOLDS
        ),
    )
}

/// The section's ball centre at `u`, as the circumcentre of three of its
/// points. A CHAMFER section is straight and has none; that is an error here
/// rather than a fallback, because everything below reads the sign of a
/// Jacobian against this point.
fn section_centre(surface: &NurbsSurface, u: f64) -> Result<Vec3, KernelRefusal> {
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let a = surface.evaluate(u, v0).or_refuse(KernelStage::Sew, "evaluate")?;
    let b = surface.evaluate(u, 0.5 * (v0 + v1)).or_refuse(KernelStage::Sew, "evaluate")?;
    let c = surface.evaluate(u, v1).or_refuse(KernelStage::Sew, "evaluate")?;
    let first = b.sub(a);
    let second = c.sub(a);
    let cross = first.cross(second);
    let denominator = 2.0 * cross.dot(cross);
    if !(denominator > 0.0) || !denominator.is_finite() {
        return Err(KernelRefusal::internal(KernelStage::Sew, "straight_section", "blend carve: the section is straight — a chamfer has no ball centre"));
    }
    let to_centre = second
        .cross(cross)
        .scale(first.dot(first))
        .add(cross.cross(first).scale(second.dot(second)))
        .scale(1.0 / denominator);
    Ok(a.add(to_centre))
}

/// `-rho^2 (1 - rho kappa cos psi)` read off the surface: negative on the
/// regular part of a wall built this way, positive inside the fold.
fn jacobian(surface: &NurbsSurface, u: f64, v: f64) -> Result<f64, KernelRefusal> {
    jacobian_about(surface, u, v, section_centre(surface, u)?)
}

/// The same, with the section's centre already in hand: it depends on `u`
/// alone, so a scan down one section pays for it once.
fn jacobian_about(surface: &NurbsSurface, u: f64, v: f64, centre: Vec3) -> Result<f64, KernelRefusal> {
    let derivatives = surface.derivatives(u, v, 1).or_refuse(KernelStage::Sew, "derivatives")?;
    let point = derivatives[0][0];
    let du = derivatives[1][0];
    let dv = derivatives[0][1];
    Ok(du.cross(dv).dot(point.sub(centre)))
}


fn jacobian_du(surface: &NurbsSurface, u: f64, v: f64, step: f64) -> Result<f64, KernelRefusal> {
    Ok((jacobian(surface, u + step, v)? - jacobian(surface, u - step, v)?) / (2.0 * step))
}

/// Solve `g = 0` and `g_u = 0`: an extremal-`v` point of the fold lens, which
/// is a swallowtail and therefore one end of the crease.
fn swallowtail(
    surface: &NurbsSurface,
    seed: [f64; 2],
    step: f64,
    bounds: [f64; 4],
) -> Result<[f64; 2], KernelRefusal> {
    let mut point = seed;
    for _ in 0..60 {
        let residual = [
            jacobian(surface, point[0], point[1])?,
            jacobian_du(surface, point[0], point[1], step)?,
        ];
        let scale = residual[0].abs().max(residual[1].abs() * step);
        if scale < 1e-14 {
            break;
        }
        let mut matrix = [[0.0f64; 2]; 2];
        for column in 0..2 {
            let mut high = point;
            let mut low = point;
            high[column] += step;
            low[column] -= step;
            matrix[0][column] = (jacobian(surface, high[0], high[1])?
                - jacobian(surface, low[0], low[1])?)
                / (2.0 * step);
            matrix[1][column] = (jacobian_du(surface, high[0], high[1], step)?
                - jacobian_du(surface, low[0], low[1], step)?)
                / (2.0 * step);
        }
        let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
        if !(determinant.abs() > 0.0) || !determinant.is_finite() {
            return Err(KernelRefusal::internal(KernelStage::Sew, "fold_lens_extremum", "blend carve: the fold lens has no isolated extremal-v point"));
        }
        let delta = [
            (-residual[0] * matrix[1][1] + residual[1] * matrix[0][1]) / determinant,
            (-matrix[0][0] * residual[1] + matrix[1][0] * residual[0]) / determinant,
        ];
        point = [
            (point[0] + delta[0]).clamp(bounds[0], bounds[1]),
            (point[1] + delta[1]).clamp(bounds[2], bounds[3]),
        ];
        if delta[0].abs() + delta[1].abs() < 1e-14 {
            break;
        }
    }
    if jacobian(surface, point[0], point[1])?.abs() > 1e-6 * scale_of(surface)? {
        return Err(KernelRefusal::non_convergence(KernelStage::Sew, "fold_lens_end", "blend carve: the fold lens end did not converge"));
    }
    Ok(point)
}

/// A magnitude to judge the Jacobian residual against: its own worst value
/// over the wall, so the test is relative to the surface rather than to a
/// model size this module never sees.
fn scale_of(surface: &NurbsSurface) -> Result<f64, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let mut worst: f64 = 0.0;
    for index in 0..8 {
        let u = u0 + (u1 - u0) * (index as f64 + 0.5) / 8.0;
        for step in 0..3 {
            let v = v0 + (v1 - v0) * step as f64 / 2.0;
            worst = worst.max(jacobian(surface, u, v)?.abs());
        }
    }
    Ok(worst.max(1e-30))
}

/// One crease point: `S(u1, v) = S(u2, v2)`, with `v` pinned.
fn crease_at(
    surface: &NurbsSurface,
    v: f64,
    seed: [f64; 3],
    bounds: [f64; 4],
) -> Result<Option<[f64; 3]>, KernelRefusal> {
    // Unknowns: u1, u2, v2.
    let mut point = seed;
    let step = 1e-7 * (bounds[1] - bounds[0]).max(bounds[3] - bounds[2]);
    let residual = |value: [f64; 3]| -> Result<Vec3, KernelRefusal> {
        Ok(surface
            .evaluate(value[0], v).or_refuse(KernelStage::Sew, "evaluate")?
            .sub(surface.evaluate(value[1], value[2]).or_refuse(KernelStage::Sew, "evaluate")?))
    };
    for _ in 0..80 {
        let r = residual(point)?;
        if r.length() < CREASE_TOLERANCE {
            break;
        }
        let mut matrix = [[0.0f64; 3]; 3];
        for column in 0..3 {
            let mut high = point;
            let mut low = point;
            high[column] += step;
            low[column] -= step;
            let derivative = residual(high)?.sub(residual(low)?).scale(1.0 / (2.0 * step));
            matrix[0][column] = derivative.x;
            matrix[1][column] = derivative.y;
            matrix[2][column] = derivative.z;
        }
        let Some(delta) = solve3(matrix, [-r.x, -r.y, -r.z]) else {
            return Ok(None);
        };
        point = [
            (point[0] + delta[0]).clamp(bounds[0], bounds[1]),
            (point[1] + delta[1]).clamp(bounds[0], bounds[1]),
            (point[2] + delta[2]).clamp(bounds[2], bounds[3]),
        ];
    }
    if residual(point)?.length() > 1e-9 {
        return Ok(None);
    }
    // THE DIAGONAL IS ALWAYS A SOLUTION and it is never the crease. `S(u1, v) =
    // S(u2, v2)` is satisfied identically by `u2 = u1, v2 = v`, and that trivial
    // branch MEETS the crease at the swallowtail — so a march stepping toward an
    // end will fall onto it, and every sample after that collapses the two
    // pcurves onto one another. A solution whose feet have merged is reported as
    // no solution; the ends themselves are solved separately, from the fold
    // locus, where the merge is the answer rather than a failure.
    let span = (bounds[1] - bounds[0]).max(bounds[3] - bounds[2]);
    if (point[0] - point[1]).abs() + (v - point[2]).abs() < 1e-7 * span {
        return Ok(None);
    }
    Ok(Some(point))
}

fn solve3(mut matrix: [[f64; 3]; 3], mut rhs: [f64; 3]) -> Option<[f64; 3]> {
    for column in 0..3 {
        let mut pivot = column;
        for row in column + 1..3 {
            if matrix[row][column].abs() > matrix[pivot][column].abs() {
                pivot = row;
            }
        }
        if !(matrix[pivot][column].abs() > 0.0) {
            return None;
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        for row in column + 1..3 {
            let factor = matrix[row][column] / matrix[column][column];
            for inner in column..3 {
                matrix[row][inner] -= factor * matrix[column][inner];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    let mut out = [0.0f64; 3];
    for row in (0..3).rev() {
        let mut value = rhs[row];
        for column in row + 1..3 {
            value -= matrix[row][column] * out[column];
        }
        out[row] = value / matrix[row][row];
    }
    if out.iter().any(|value| !value.is_finite()) {
        return None;
    }
    Some(out)
}

/// Find the fold lens on a wall and trace the crease that bounds it.
///
/// `Ok(None)` means the wall carries no fold at this resolution, which is the
/// answer for every wall the check in `fold.rs` lets through.
pub(super) fn trace_wall_crease(
    surface: &NurbsSurface,
    radius: f64,
) -> Result<Option<WallCarve>, KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
    let bounds = [u0, u1, v0, v1];
    let step = 1e-6 * (u1 - u0);

    // Where does the Jacobian change sign? A grid dense enough to find the lens
    // at all: the lens is as long in u as the bend that made it, which is at
    // least one station, so the wall's own control net sets the sampling.
    let samples_u = (surface.control_points.len() * 4).max(256);
    let samples_v = 96usize;
    let mut inside: Vec<[f64; 2]> = Vec::new();
    for index in 0..=samples_u {
        let u = u0 + (u1 - u0) * index as f64 / samples_u as f64;
        let centre = section_centre(surface, u)?;
        for step_v in 0..=samples_v {
            let v = v0 + (v1 - v0) * step_v as f64 / samples_v as f64;
            if jacobian_about(surface, u, v, centre)? > 0.0 {
                inside.push([u, v]);
            }
        }
    }
    if inside.is_empty() {
        return Ok(None);
    }
    // A band that WRAPS the wall's whole u period is not a lens. That is the
    // circular spine: its evolute is a single point, every section crosses the
    // fold at the same two axis points, and the crease degenerates to those two
    // points with no curve between them to trim along. Refused by name rather
    // than handed to a swallowtail solve that has no isolated end to find.
    let cell = (u1 - u0) / samples_u as f64;
    let (band_low, band_high) = inside.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |acc, point| {
        (acc.0.min(point[0]), acc.1.max(point[0]))
    });
    if band_low <= u0 + cell && band_high >= u1 - cell {
        return Err(band_refusal(
            radius,
            "wraps the wall's whole u period, so it closes into no lens and its crease \
             degenerates to isolated points",
        ));
    }
    // The lens's extremal-v samples seed the two swallowtails.
    let mut low = inside[0];
    let mut high = inside[0];
    for point in &inside {
        if point[1] < low[1] {
            low = *point;
        }
        if point[1] > high[1] {
            high = *point;
        }
    }
    let rail_band = (v1 - v0) * RAIL_FRACTION;
    if low[1] - v0 <= rail_band || v1 - high[1] <= rail_band {
        return Err(band_refusal(
            radius,
            &format!(
                "reaches the wall's rail (its v extent is [{:.9}, {:.9}] in a domain of \
                 [{v0:.9}, {v1:.9}])",
                low[1], high[1]
            ),
        ));
    }
    let first_end = swallowtail(surface, low, step, bounds)?;
    let second_end = swallowtail(surface, high, step, bounds)?;
    if !(second_end[1] - first_end[1] > rail_band) {
        return Err(band_refusal(
            radius,
            &format!(
                "closes to nothing between v {:.9} and v {:.9}",
                first_end[1], second_end[1]
            ),
        ));
    }
    if first_end[1] - v0 <= rail_band || v1 - second_end[1] <= rail_band {
        return Err(band_refusal(
            radius,
            &format!(
                "ends on the wall's rail (v {:.9} and v {:.9} in a domain of [{v0:.9}, {v1:.9}])",
                first_end[1], second_end[1]
            ),
        ));
    }

    // The widest place: the lens's own u extent at the v halfway between the
    // ends seeds the march, and the march is continued outward from there.
    let middle = 0.5 * (first_end[1] + second_end[1]);
    let mut span: Option<[f64; 2]> = None;
    for point in &inside {
        if (point[1] - middle).abs() > (v1 - v0) / samples_v as f64 {
            continue;
        }
        span = Some(match span {
            None => [point[0], point[0]],
            Some([lo, hi]) => [lo.min(point[0]), hi.max(point[0])],
        });
    }
    let Some([lens_low, lens_high]) = span else {
        return Err(KernelRefusal::internal(KernelStage::Sew, "fold_lens_width", "blend carve: the fold lens has no width at its own middle"));
    };
    // The crease is OUTSIDE the singular lens, so seed the two feet a lens
    // width beyond each side and let Newton pull them in.
    let width = lens_high - lens_low;
    let seed = [lens_low - width, lens_high + width, middle];
    let Some(centre) = crease_at(surface, middle, seed, bounds)? else {
        return Err(KernelRefusal::non_convergence(KernelStage::Sew, "crease_middle", "blend carve: the crease did not solve at the lens's own middle"));
    };

    let samples = march_crease(
        surface,
        [first_end, second_end],
        centre,
        bounds,
        CREASE_SAMPLES,
    )?;
    let march_residual = samples
        .iter()
        .map(|sample| {
            surface
                .evaluate(sample.first[0], sample.first[1])
                .and_then(|a| Ok(a.sub(surface.evaluate(sample.second[0], sample.second[1])?).length()))
                .unwrap_or(f64::INFINITY)
        })
        .fold(0.0f64, f64::max);
    Ok(Some(WallCarve {
        ends: [first_end, second_end],
        samples,
        march_residual,
        centre,
    }))
}

/// March the crease between the two swallowtails, Chebyshev-spaced in `v`.
fn march_crease(
    surface: &NurbsSurface,
    ends: [[f64; 2]; 2],
    centre: [f64; 3],
    bounds: [f64; 4],
    count: usize,
) -> Result<Vec<CreaseSample>, KernelRefusal> {
    let (v_low, v_high) = (ends[0][1], ends[1][1]);
    let middle = 0.5 * (v_low + v_high);
    let half = 0.5 * (v_high - v_low);
    let mut forward: Vec<CreaseSample> = Vec::new();
    let mut backward: Vec<CreaseSample> = Vec::new();
    for direction in [1.0f64, -1.0] {
        let mut seed = centre;
        for index in 1..count {
            let angle = std::f64::consts::PI * index as f64 / (2.0 * (count - 1) as f64);
            let v = middle + direction * half * angle.sin();
            let Some(solved) = crease_at(surface, v, seed, bounds)? else {
                break;
            };
            seed = solved;
            let point = surface.evaluate(solved[0], v).or_refuse(KernelStage::Sew, "evaluate")?;
            let sample = CreaseSample {
                first: [solved[0], v],
                second: [solved[1], solved[2]],
                point,
            };
            if direction > 0.0 {
                forward.push(sample);
            } else {
                backward.push(sample);
            }
        }
    }
    let forward_len = forward.len();
    let mut samples = Vec::with_capacity(forward.len() + backward.len() + 3);
    let end_point = |uv: [f64; 2]| -> Result<CreaseSample, KernelRefusal> {
        Ok(CreaseSample {
            first: uv,
            second: uv,
            point: surface.evaluate(uv[0], uv[1]).or_refuse(KernelStage::Sew, "evaluate")?,
        })
    };
    samples.push(end_point(ends[0])?);
    backward.reverse();
    samples.extend(backward);
    samples.push(CreaseSample {
        first: [centre[0], 0.5 * (v_low + v_high)],
        second: [centre[1], centre[2]],
        point: surface.evaluate(centre[0], 0.5 * (v_low + v_high)).or_refuse(KernelStage::Sew, "evaluate")?,
    });
    samples.extend(forward);
    samples.push(end_point(ends[1])?);
    carve_trace(format_args!(
        "blend carve: crease marched with {} sample(s) of {count} asked ({} below the middle, \
         {} above)",
        samples.len(),
        samples.len().saturating_sub(forward_len + 3),
        forward_len
    ));
    for sample in &samples {
        carve_trace(format_args!(
            "  crease uv1 ({:.9}, {:.9}) uv2 ({:.9}, {:.9}) at ({:.9}, {:.9}, {:.9})",
            sample.first[0],
            sample.first[1],
            sample.second[0],
            sample.second[1],
            sample.point.x,
            sample.point.y,
            sample.point.z
        ));
    }
    Ok(samples)
}

/// Fit the crease and its two pcurves, and MEASURE the fit: the curve is what
/// becomes the trim boundary, so "through the marched points" is not the
/// contract — "the two sheets still meet on the fitted curve" is.
pub(super) fn fit_crease(
    surface: &NurbsSurface,
    carve: &WallCarve,
    tolerance: f64,
) -> Result<CarvedCrease, KernelRefusal> {
    let mut samples = carve.samples.clone();
    let mut count = CREASE_SAMPLES;
    let mut best: Option<(usize, CarvedCrease)> = None;
    loop {
        let fitted = fit_once(surface, &samples, tolerance)?;
        carve_trace(format_args!(
            "blend carve: crease fitted through {} sample(s), worst pcurve-vs-curve {:.6e} \
             against a bar of {tolerance:.6e} (the march itself closed to {:.3e})",
            samples.len(),
            fitted.residual,
            carve.march_residual
        ));
        if fitted.residual <= tolerance {
            return Ok(fitted);
        }
        // EVERY RUNG IS MEASURED AND THE BEST IS KEPT. More samples is not
        // monotonically better here — past a point the interpolant rings — so
        // the ladder reports the best it reached rather than the last it tried.
        if best
            .as_ref()
            .is_none_or(|(_, previous)| fitted.residual < previous.residual)
        {
            best = Some((samples.len(), fitted));
        }
        if count * 2 > MAX_CREASE_SAMPLES {
            let (best_count, best_fit) = best.expect("at least one rung was measured");
            if best_fit.residual <= tolerance {
                return Ok(best_fit);
            }
            let _ = best_count;
            // The interpolant is what becomes the trim boundary, so a fit that
            // does not name the crease is not shipped with a note — it is the
            // refusal, with the number it reached.
            return Err(KernelRefusal::non_convergence(KernelStage::Sew, "crease_fit", format!(
                "blend carve: the crease's pcurves stay {:.6e} from the crease curve at \
                 {} marched sample(s), against a bar of {tolerance:.6e} — the fold band is \
                 not resolved well enough to trim along",
                best_fit.residual,
                best_count
            )));
        }
        count = (count - 1) * 2 + 1;
        samples = march_crease(
            surface,
            carve.ends,
            carve.centre,
            [
                surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?[0],
                surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?[1],
                surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?[0],
                surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?[1],
            ],
            count,
        )?;
    }
}

fn fit_once(
    surface: &NurbsSurface,
    samples: &[CreaseSample],
    _tolerance: f64,
) -> Result<CarvedCrease, KernelRefusal> {
    if samples.len() < 4 {
        return Err(KernelRefusal::internal(KernelStage::Sew, "crease_points", format!(
            "blend carve: a crease of {} point(s) is not a curve",
            samples.len()
        )));
    }
    let points: Vec<Vec3> = samples.iter().map(|sample| sample.point).collect();
    // PARAMETERISE BY THE PARAMETER ARC, NOT THE 3-D ONE. The crease is a
    // regular curve in space, but its two PREIMAGES meet the swallowtail like a
    // square root: `u - u_end ~ sqrt(v - v_end)`. Chord length in 3-D is
    // proportional to `v - v_end` there, so in that parameter the pcurves carry
    // a cusp, a polynomial rings across it, and refining makes it WORSE —
    // measured on the 2026-09-02 collar, 4.153094e-4 at 65 samples, 1.311647e-4
    // at 129, then 2.827768e+1 at 258. Arc length in (u, v) is proportional to
    // `u - u_end` instead, which makes both pcurves smooth and leaves the 3-D
    // curve smooth with a vanishing end velocity — a shape a cubic fits.
    let mut parameters = Vec::with_capacity(points.len());
    let mut running = 0.0;
    parameters.push(0.0);
    for index in 1..points.len() {
        let previous = samples[index - 1];
        let current = samples[index];
        let first = (current.first[0] - previous.first[0])
            .hypot(current.first[1] - previous.first[1]);
        let second = (current.second[0] - previous.second[0])
            .hypot(current.second[1] - previous.second[1]);
        running += first + second;
        parameters.push(running);
    }
    if !(running > 0.0) {
        return Err(KernelRefusal::internal(KernelStage::Sew, "crease_length", "blend carve: the crease has no length"));
    }
    for parameter in parameters.iter_mut() {
        *parameter /= running;
    }
    if parameters.windows(2).any(|pair| pair[1] <= pair[0]) {
        return Err(KernelRefusal::internal(KernelStage::Sew, "crease_doubles_back", "blend carve: the crease doubles back on itself"));
    }
    let degree = 3.min(points.len() - 1);
    let curve = fit::interpolate_curve(&points, degree, &parameters).or_refuse(KernelStage::Sew, "interpolate_curve")?;
    let first = fit::interpolate_curve(
        &samples
            .iter()
            .map(|sample| Vec3::new(sample.first[0], sample.first[1], 0.0))
            .collect::<Vec<_>>(),
        degree,
        &parameters,
    ).or_refuse(KernelStage::Sew, "interpolate_curve")?;
    let second = fit::interpolate_curve(
        &samples
            .iter()
            .map(|sample| Vec3::new(sample.second[0], sample.second[1], 0.0))
            .collect::<Vec<_>>(),
        degree,
        &parameters,
    ).or_refuse(KernelStage::Sew, "interpolate_curve")?;
    // The measurement: does the FITTED pcurve still trace the FITTED curve?
    let mut residual: f64 = 0.0;
    let dense = (samples.len() * 4).max(64);
    for index in 0..=dense {
        let t = index as f64 / dense as f64;
        let target = curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?;
        for pcurve in [&first, &second] {
            let uv = pcurve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?;
            residual = residual.max(surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?.sub(target).length());
        }
    }
    Ok(CarvedCrease {
        curve,
        first,
        second,
        start: points[0],
        end: points[points.len() - 1],
        residual,
    })
}
