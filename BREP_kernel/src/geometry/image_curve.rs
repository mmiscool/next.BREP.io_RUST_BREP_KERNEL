//! The 3D IMAGE of a parameter-space curve on a surface: `t ↦ S(p(t))`.
//!
//! Four offset/push sites used to refuse any boundary whose pcurve was not an
//! iso-parameter line, because an iso-curve extraction was the only way they
//! knew to rematerialize a boundary in 3D.  OCCT has no such restriction: the
//! iso extraction is an *optimisation with a verified fallback*, and the
//! general case is a sampled approximation of the composed curve-on-surface
//! (`GeomLib::BuildCurve3d`, `GeomLib.cxx:960-1071`; the self-verifying iso
//! shortcut is `GeomLib::buildC3dOnIsoLine`, `:2878-3018`).  This module is
//! that ladder, written for our representation and gated on our own measured
//! deviation.  All four refusals were the same missing capability — the 3D
//! image of a general pcurve on a NURBS surface — and not a restriction
//! inherent to offsetting: OCCT installs the original pcurve on the offset face
//! verbatim, leaving only the 3D curve to be approximated.
//!
//! # The parametrisation contract
//!
//! [`BrepSolid::validate`]'s coedge/edge coincidence check
//! (`brep/topology/validate.rs`'s `coedge_sample`) compares the two
//! representations at MATCHED FRACTIONS: fraction `f` of the pcurve's own
//! domain against fraction `f` of the edge's `[t0, t1]`.  Every tier here
//! therefore returns the parameter pair `(t0, t1)` for which
//!
//! ```text
//!     curve(t0 + (t1 - t0)·f)  ≈  surface(pcurve(q0 + (q1 - q0)·f))
//! ```
//!
//! for all `f ∈ [0, 1]`, where `[q0, q1]` is the pcurve's domain.  `t0 > t1`
//! is legal and means the extracted curve runs against the pcurve — callers
//! that need an increasing edge range reverse the curve and reflect the
//! parameters, exactly as the existing iso branches do.
//!
//! # The tiers, and what each validates
//!
//! 1. [`ImageCurveTier::Affine`] — an affine (degree 1 × 1, equal-weight,
//!    parallelogram) surface.  An affine map commutes with the rational
//!    evaluation, so mapping the pcurve's HOMOGENEOUS control points through
//!    the frame is exact for ANY rational pcurve: a rational circle in `(u,v)`
//!    becomes the exact 3D circle, sharing degree, knots and weights.
//! 2. [`ImageCurveTier::Iso`] — the pcurve holds one coordinate constant.  The
//!    image is the surface's iso-curve in the other direction, whose own
//!    parameter IS that coordinate, so `(t0, t1)` are simply the varying
//!    coordinate's endpoint values.  This is exact only when the pcurve is also
//!    AFFINE in its parameter; a constant-`u` pcurve with a non-linear `v(t)`
//!    passes a constant-coordinate test yet breaks the fraction contract.  The
//!    tier therefore measures itself and falls through when it misses.
//! 3. [`ImageCurveTier::Approximated`] — the general case.  The composed curve
//!    is sampled (seeded at the pcurve's own knots, where its smoothness
//!    breaks), interpolated at those same parameters so the contract holds by
//!    construction at the nodes, then measured strictly BETWEEN the nodes —
//!    interpolation is exact at its own nodes, so an on-node check would
//!    always pass and prove nothing.  Intervals that miss are bisected and the
//!    fit repeated.  If the loop cannot reach the bar the tier REFUSES, with
//!    the measured deviation in the message: a wrong curve is worse than a
//!    refusal.
//!
//! Tiers 1 and 2 verify with the same off-node sweep tier 3 uses, and fall
//! THROUGH to the next tier on failure rather than refusing, so a shortcut that
//! does not apply costs accuracy nowhere.

use crate::curve::{NurbsCurve, Vec4};
use crate::fit::interpolate_curve;
use crate::surface::NurbsSurface;
use crate::Vec3;

/// Which lane produced an [`ImageCurve`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImageCurveTier {
    /// Exact homogeneous mapping through an affine surface's frame.
    Affine,
    /// Verified iso-curve extraction.
    Iso,
    /// Sampled-and-fitted approximation of the composed curve-on-surface.
    Approximated,
}

/// The 3D image of a pcurve on a surface, plus the parameter range that
/// matches the pcurve's own domain fraction for fraction.
#[derive(Clone, Debug)]
pub struct ImageCurve {
    pub curve: NurbsCurve,
    /// Curve parameter at the pcurve's domain START. May exceed `t1`.
    pub t0: f64,
    /// Curve parameter at the pcurve's domain END.
    pub t1: f64,
    pub tier: ImageCurveTier,
    /// Measured `max ‖curve(t) − surface(pcurve(q))‖` over the off-node sweep.
    pub deviation: f64,
}

/// Number of off-node samples used to verify an exact/iso tier, matching
/// OCCT's `THE_ISOLINE_CHECK_SEGMENTS` (`GeomLib.cxx:131`) density floor.
const VERIFY_SAMPLES: usize = 24;

/// Off-node probes per interval in the approximation loop.  Five interior
/// fractions at `k/6` never coincide with an interval end or midpoint, so a
/// bisection always lands on a previously unprobed parameter.
const PROBES_PER_INTERVAL: usize = 5;

/// Refinement rounds before the approximation tier gives up.
const MAX_ROUNDS: usize = 14;

/// Sample ceiling for the approximation tier.
const MAX_SAMPLES: usize = 4096;

/// Exact 3D image of a pcurve on an AFFINE sheet: an affine map applied to
/// the homogeneous control points commutes with the rational evaluation, so
/// the image shares the pcurve's degree / knots / weights and is
/// parametrized identically — a rational circle pcurve maps to the exact 3D
/// circle.
pub fn affine_image_curve(
    sheet: &NurbsSurface,
    pcurve: &NurbsCurve,
) -> Result<NurbsCurve, String> {
    let [u0, _] = sheet.domain_u()?;
    let [v0, _] = sheet.domain_v()?;
    let frame = sheet.derivatives(u0, v0, 1)?;
    let origin = frame[0][0];
    let du = frame[1][0];
    let dv = frame[0][1];
    let control_points = pcurve
        .control_points
        .iter()
        .map(|control| {
            let position = origin
                .scale(control.w)
                .add(du.scale(control.x - control.w * u0))
                .add(dv.scale(control.y - control.w * v0));
            Vec4 {
                x: position.x,
                y: position.y,
                z: position.z,
                w: control.w,
            }
        })
        .collect();
    NurbsCurve::new(pcurve.degree, pcurve.knots.clone(), control_points)
}

/// The composed curve-on-surface `q ↦ S(p(q))`.
///
/// `evaluate_extended` matches what the validator's `coedge_sample` does: a
/// pcurve that grazes or straddles a periodic seam carries parameters just past
/// the domain, and the surface WRAPS there.  Clamping instead would read as a
/// gross deviation and turn a legitimate seam-straddling trim into a refusal.
fn compose(surface: &NurbsSurface, pcurve: &NurbsCurve, q: f64) -> Result<Vec3, String> {
    let uv = pcurve.evaluate(q)?;
    surface.evaluate_extended(uv.x, uv.y)
}

/// Worst `‖curve(t0 + (t1−t0)·f) − S(p(q0 + (q1−q0)·f))‖` over `count`
/// interior fractions, none of which is an endpoint.
fn sweep_deviation(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    curve: &NurbsCurve,
    t0: f64,
    t1: f64,
    count: usize,
) -> Result<f64, String> {
    let [q0, q1] = pcurve.domain()?;
    let mut worst = 0.0f64;
    for index in 0..=count {
        let fraction = index as f64 / count as f64;
        let target = compose(surface, pcurve, q0 + (q1 - q0) * fraction)?;
        let value = curve.evaluate(t0 + (t1 - t0) * fraction)?;
        worst = worst.max(value.sub(target).length());
    }
    Ok(worst)
}

/// Is `pcurve` a constant-`u` or constant-`v` line whose varying coordinate is
/// AFFINE in the curve parameter?  Returns `(constant is u, varying value at
/// q0, varying value at q1)`.
///
/// Both halves matter.  Constancy alone picks the iso-curve; affinity is what
/// makes the iso-curve's own parameter a fraction-for-fraction stand-in for the
/// pcurve parameter.  The check is structural first (degree 1, two poles, equal
/// weights — exactly linear, the shape a boolean's straight trim and a seam
/// both take) and sampled second, so a degree-elevated but geometrically linear
/// pcurve is still recognised.
fn iso_line(pcurve: &NurbsCurve, eps_u: f64, eps_v: f64) -> Result<Option<(bool, f64, f64)>, String> {
    let [q0, q1] = pcurve.domain()?;
    let first = pcurve.evaluate(q0)?;
    let last = pcurve.evaluate(q1)?;
    let constant_u = (first.x - last.x).abs() <= eps_u;
    let constant_v = (first.y - last.y).abs() <= eps_v;
    // A pcurve constant in BOTH directions is a point; there is no iso-curve to
    // extract and no direction to trim along.
    if constant_u == constant_v {
        return Ok(None);
    }
    let span = q1 - q0;
    for index in 1..8 {
        let fraction = index as f64 / 8.0;
        let uv = pcurve.evaluate(q0 + span * fraction)?;
        let (held, varying, expected, eps_held, eps_vary) = if constant_u {
            (
                uv.x - first.x,
                uv.y,
                first.y + (last.y - first.y) * fraction,
                eps_u,
                eps_v,
            )
        } else {
            (
                uv.y - first.y,
                uv.x,
                first.x + (last.x - first.x) * fraction,
                eps_v,
                eps_u,
            )
        };
        if held.abs() > eps_held || (varying - expected).abs() > eps_vary {
            return Ok(None);
        }
    }
    Ok(Some(if constant_u {
        (true, first.y, last.y)
    } else {
        (false, first.x, last.x)
    }))
}

/// The constant coordinate an iso extraction should be taken at.
fn iso_constant(pcurve: &NurbsCurve, constant_u: bool) -> Result<f64, String> {
    let [q0, q1] = pcurve.domain()?;
    let first = pcurve.evaluate(q0)?;
    let last = pcurve.evaluate(q1)?;
    Ok(if constant_u {
        0.5 * (first.x + last.x)
    } else {
        0.5 * (first.y + last.y)
    })
}

/// Parametric detection bands for [`iso_line`], derived from the surface's own
/// domain so a `[0,1]²` patch and a `[0,2π]²` torus are judged alike.
fn iso_epsilons(surface: &NurbsSurface) -> Result<(f64, f64), String> {
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    Ok((1e-7 * (u1 - u0).abs(), 1e-7 * (v1 - v0).abs()))
}

/// Seed parameters for the approximation tier: the pcurve's ends, its distinct
/// interior knots (where the composed curve's smoothness breaks — the cheap
/// analogue of OCCT's C2/C3 discontinuity split at `GeomLib.cxx:1035-1042`) and
/// a uniform floor so a single-span pcurve still starts with real structure.
fn seed_parameters(pcurve: &NurbsCurve) -> Result<Vec<f64>, String> {
    let [q0, q1] = pcurve.domain()?;
    let span = q1 - q0;
    let mut parameters = vec![q0, q1];
    for knot in &pcurve.knots {
        if *knot > q0 && *knot < q1 {
            parameters.push(*knot);
        }
    }
    for index in 1..8 {
        parameters.push(q0 + span * index as f64 / 8.0);
    }
    parameters.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    parameters.dedup_by(|a, b| (*a - *b).abs() <= span.abs() * 1e-9);
    Ok(parameters)
}

/// Fit the composed curve at `parameters`, then measure it strictly BETWEEN
/// them.  Returns the fitted curve, the worst off-node deviation, and the
/// per-interval worst deviations (one shorter than `parameters`).
fn fit_and_measure(
    surfaces: &[&NurbsSurface],
    pcurve: &NurbsCurve,
    parameters: &[f64],
    which: usize,
) -> Result<(NurbsCurve, f64, Vec<f64>), String> {
    let surface = surfaces[which];
    let points = parameters
        .iter()
        .map(|q| compose(surface, pcurve, *q))
        .collect::<Result<Vec<_>, String>>()?;
    let degree = 3.min(points.len() - 1);
    let curve = interpolate_curve(&points, degree, parameters)?;
    let mut worst = 0.0f64;
    let mut per_interval = Vec::with_capacity(parameters.len() - 1);
    for window in parameters.windows(2) {
        let (a, b) = (window[0], window[1]);
        let mut local = 0.0f64;
        for probe in 1..=PROBES_PER_INTERVAL {
            let q = a + (b - a) * probe as f64 / (PROBES_PER_INTERVAL + 1) as f64;
            let target = compose(surface, pcurve, q)?;
            local = local.max(curve.evaluate(q)?.sub(target).length());
        }
        worst = worst.max(local);
        per_interval.push(local);
    }
    Ok((curve, worst, per_interval))
}

/// The approximation tier, shared by the single- and paired-surface entry
/// points.  Every surface in `surfaces` is fitted over ONE parameter set, so
/// the results share degree, knots and (unit) weights — the basis identity
/// `thicken`'s `ruled_wall` requires of a bottom/top image pair.
fn approximate(
    surfaces: &[&NurbsSurface],
    pcurve: &NurbsCurve,
    tolerance: f64,
    site: &str,
) -> Result<Vec<ImageCurve>, String> {
    let [q0, q1] = pcurve.domain()?;
    let mut parameters = seed_parameters(pcurve)?;
    let mut best = f64::INFINITY;
    for _ in 0..MAX_ROUNDS {
        let mut fits = Vec::with_capacity(surfaces.len());
        let mut worst = 0.0f64;
        let mut per_interval = vec![0.0f64; parameters.len() - 1];
        for which in 0..surfaces.len() {
            let (curve, sheet_worst, sheet_intervals) =
                fit_and_measure(surfaces, pcurve, &parameters, which)?;
            worst = worst.max(sheet_worst);
            for (slot, value) in per_interval.iter_mut().zip(&sheet_intervals) {
                *slot = slot.max(*value);
            }
            fits.push(curve);
        }
        best = best.min(worst);
        if worst <= tolerance {
            return Ok(fits
                .into_iter()
                .map(|curve| ImageCurve {
                    curve,
                    t0: q0,
                    t1: q1,
                    tier: ImageCurveTier::Approximated,
                    deviation: worst,
                })
                .collect());
        }
        // Bisect every interval that missed the bar. Refining only the single
        // worst interval converges linearly in the number of fits; refining all
        // failing intervals halves the whole failing region per round.
        let mut refined = Vec::with_capacity(parameters.len() * 2);
        for (index, window) in parameters.windows(2).enumerate() {
            refined.push(window[0]);
            if per_interval[index] > tolerance {
                refined.push(0.5 * (window[0] + window[1]));
            }
        }
        refined.push(parameters[parameters.len() - 1]);
        if refined.len() == parameters.len() || refined.len() > MAX_SAMPLES {
            break;
        }
        parameters = refined;
    }
    Err(format!(
        "{site}: the 3D image of a general pcurve could not be fitted to tolerance \
         (worst off-node deviation {best:.3e} > {tolerance:.3e} after {} samples) — refusing",
        parameters.len()
    ))
}

/// Build the 3D image of an ARBITRARY pcurve on `surface`.
///
/// See the module docs for the tier ladder and the parametrisation contract.
/// `tolerance` is the caller's own spatial accuracy bar for a committed edge;
/// it is the acceptance criterion for every tier, and the approximation tier
/// refuses rather than returning a fit that misses it.  `site` names the caller
/// in the refusal message.
pub fn image_curve(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    tolerance: f64,
    site: &str,
) -> Result<ImageCurve, String> {
    let [q0, q1] = pcurve.domain()?;
    if !(q1 - q0).is_finite() || (q1 - q0).abs() <= 0.0 {
        return Err(format!("{site}: pcurve has an empty parameter domain"));
    }

    if surface.is_affine()? {
        let curve = affine_image_curve(surface, pcurve)?;
        let deviation = sweep_deviation(surface, pcurve, &curve, q0, q1, VERIFY_SAMPLES)?;
        if deviation <= tolerance {
            return Ok(ImageCurve {
                curve,
                t0: q0,
                t1: q1,
                tier: ImageCurveTier::Affine,
                deviation,
            });
        }
    }

    let (eps_u, eps_v) = iso_epsilons(surface)?;
    if let Some((constant_u, start, end)) = iso_line(pcurve, eps_u, eps_v)? {
        let constant = iso_constant(pcurve, constant_u)?;
        let curve = if constant_u {
            surface.iso_curve_u(constant)?
        } else {
            surface.iso_curve_v(constant)?
        };
        let deviation = sweep_deviation(surface, pcurve, &curve, start, end, VERIFY_SAMPLES)?;
        if deviation <= tolerance {
            return Ok(ImageCurve {
                curve,
                t0: start,
                t1: end,
                tier: ImageCurveTier::Iso,
                deviation,
            });
        }
    }

    Ok(approximate(&[surface], pcurve, tolerance, site)?
        .pop()
        .expect("one surface in, one image out"))
}

/// The images of ONE pcurve on TWO surfaces, guaranteed to share a basis.
///
/// `thicken`'s side wall is `ruled_wall(bottom, top)`, which refuses unless the
/// two boundary curves agree in degree, knots and weights.  Fitting the two
/// sheets independently would let their adaptive sample sets diverge, so the
/// approximation tier fits both over ONE parameter set refined against the
/// worse of the two.  The tier is chosen once and applies to both.
pub fn image_curve_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    pcurve: &NurbsCurve,
    tolerance: f64,
    site: &str,
) -> Result<(ImageCurve, ImageCurve), String> {
    let [q0, q1] = pcurve.domain()?;
    if !(q1 - q0).is_finite() || (q1 - q0).abs() <= 0.0 {
        return Err(format!("{site}: pcurve has an empty parameter domain"));
    }

    if first.is_affine()? && second.is_affine()? {
        let a = affine_image_curve(first, pcurve)?;
        let b = affine_image_curve(second, pcurve)?;
        let deviation = sweep_deviation(first, pcurve, &a, q0, q1, VERIFY_SAMPLES)?
            .max(sweep_deviation(second, pcurve, &b, q0, q1, VERIFY_SAMPLES)?);
        if deviation <= tolerance {
            return Ok((
                ImageCurve {
                    curve: a,
                    t0: q0,
                    t1: q1,
                    tier: ImageCurveTier::Affine,
                    deviation,
                },
                ImageCurve {
                    curve: b,
                    t0: q0,
                    t1: q1,
                    tier: ImageCurveTier::Affine,
                    deviation,
                },
            ));
        }
    }

    let (eps_u, eps_v) = iso_epsilons(first)?;
    if let Some((constant_u, start, end)) = iso_line(pcurve, eps_u, eps_v)? {
        let constant = iso_constant(pcurve, constant_u)?;
        let (a, b) = if constant_u {
            (first.iso_curve_u(constant)?, second.iso_curve_u(constant)?)
        } else {
            (first.iso_curve_v(constant)?, second.iso_curve_v(constant)?)
        };
        let deviation = sweep_deviation(first, pcurve, &a, start, end, VERIFY_SAMPLES)?
            .max(sweep_deviation(second, pcurve, &b, start, end, VERIFY_SAMPLES)?);
        if deviation <= tolerance {
            return Ok((
                ImageCurve {
                    curve: a,
                    t0: start,
                    t1: end,
                    tier: ImageCurveTier::Iso,
                    deviation,
                },
                ImageCurve {
                    curve: b,
                    t0: start,
                    t1: end,
                    tier: ImageCurveTier::Iso,
                    deviation,
                },
            ));
        }
    }

    let mut images = approximate(&[first, second], pcurve, tolerance, site)?;
    let second_image = images.pop().expect("two surfaces in, two images out");
    let first_image = images.pop().expect("two surfaces in, two images out");
    Ok((first_image, second_image))
}

