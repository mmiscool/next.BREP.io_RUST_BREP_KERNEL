//! Measured tolerance for offset constructions — the deviation OBSERVED
//! between a built entity and the geometry it was built to reproduce.
//!
//! # What this module is
//!
//! [`crate::MeasuredTolerance`] is the vocabulary; this is the offset family's
//! measurement. Two quantities, one per construction kind:
//!
//! * [`measure_edge_against_pcurve_image`] — for a rim/trim edge built by
//!   sampling a pcurve through a surface and interpolating the images:
//!   `max_t ‖C_3d(t) − S(p(t))‖`. This is the number that makes the offset
//!   family's tolerance measured rather than predicted — the size-derived band
//!   stays the acceptance bar the caller gates on, and this observed deviation
//!   is what gets recorded — and the analogue of OCCT's
//!   `BRepOffset_SimpleOffset::FillEdgeData`
//!   (`BRepOffset_SimpleOffset.cxx:296-310`), which sets the edge tolerance to
//!   exactly this distance measured with `BRepLib_ValidateEdge`.
//! * [`measure_surface_fit_against_pointwise_offset`] — for a carrier built by
//!   collocation: `max_{u,v} ‖S_fit(u,v) − offset(u,v)‖`. Nothing in this tree
//!   has ever measured our Greville fit against the pointwise offset it
//!   interpolates; this does.
//!
//! Plus [`vertex_endpoint_gap`], the endpoint half of the edge → vertex
//! propagation whose rule (and whose verdict on OCCT's 1.001 factor) lives on
//! [`crate::vertex_tolerance_from_edges`].
//!
//! # Why the edge measurement is `adaptive_coedge_error` PLUS a span pass
//!
//! `brep/topology/validate.rs:453` already computes `max_s ‖S(q(s)) − c(t(s))‖`
//! over an adaptive subdivision, seam-aware through `evaluate_extended`, and it
//! is what [`crate::BrepSolid::validate`] itself runs on every coedge. Writing a
//! second sampler from scratch would produce a second answer to one question —
//! the failure the offset-unification audit spent five slices removing — so it
//! is kept as the general backstop and its band-driven refinement is passed
//! through exactly as validate passes `pcurve_limit`.
//!
//! **But it is not sufficient here, and this was measured, not assumed.** Its
//! sample set is 32 uniform intervals bisected to depth 3, i.e. the DYADIC grid
//! of multiples of 1/256. Every 3D curve `offset_face_carrier` built when this
//! was written came from `mapped_pcurve_polyline`, whose adaptive subdivision
//! bisects the same interval and therefore places its interpolation nodes on
//! the dyadic grid too (up to 1024 of them at its depth cap,
//! `offset/offset.rs:560`). Once the polyline is finer than 1/256, **every one
//! of the sampler's points is an interpolation node**, where a degree-1
//! interpolant is exact by construction. (Since 2026-09-06 the carrier builds
//! its edges through the `image_curve` ladder and reaches that polyline only
//! as the ladder's fallback — `offset.rs::carrier_edge_curve` — but the
//! fallback, and any degree-1 curve a caller transfers, is measured here the
//! same way.)
//! Measured on a 1200-long cylinder's cap rim: `adaptive_coedge_error` answers
//! `8.0e-14`; a dense scan of the same comparison answers `2.356e-3`. Ten orders
//! of magnitude, and the aliasing gets WORSE the finer the curve. (The same
//! blind spot is in `BrepSolid::validate`, which runs the same sampler over the
//! same curves — recorded in the study doc, not fixed here.)
//!
//! So the measurement adds [`span_midpoint_error`]: one sample at the MIDPOINT
//! of each of the curve's own knot spans. For a degree-1 interpolant — the
//! construction's fallback curve — the deviation from the smooth image is
//! zero at the nodes and extremal inside the span, so a midpoint per span is not
//! a heuristic, it is the right estimator for this construction class. The
//! recorded deviation is the maximum of the two passes: a general backstop that
//! can see between spans, and a span pass that cannot be aliased away.
//!
//! A caller transferring a HIGHER-degree curve (the general pcurve → 3D image
//! curve the `image_curve` ladder interpolates as a cubic once neither its
//! affine nor its iso tier applies) needs both for the opposite reason: a
//! cubic's worst point is not generally the span midpoint, so the adaptive pass
//! carries that case and the span pass only floors it.
//!
//! Cost is bounded by the curve's own span count plus one validate pass — a
//! ceiling this kernel already pays on every offset result.
//!
//! # The direction rule
//!
//! Everything here RECORDS. Nothing here widens a band. See
//! [`crate::MeasuredTolerance`]'s "direction rule".

use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::topology::{coedge_error_floored, EdgeRecord};
use crate::{MeasuredTolerance, NurbsCurve, NurbsSurface, OffsetEvaluator, OffsetNormal, Vec3};

/// The grid used by [`measure_surface_fit_against_pointwise_offset`], and the
/// two fractional offsets that keep its samples off the knot lines a
/// collocation fit interpolates exactly.
///
/// Both are copied deliberately from the free-form push's dense residual gate
/// (`edit/direct_edit/face_offset_freeform.rs:72-76`): 15x15 with `+0.31` /
/// `+0.43` cell offsets. Sampling the Greville parameters themselves would
/// report `0.0` by construction — the fit interpolates there — so the whole
/// value of this measurement is in sampling BETWEEN them.
const FIT_SAMPLES: usize = 15;
const FIT_OFFSET_U: f64 = 0.31;
const FIT_OFFSET_V: f64 = 0.43;

/// `max_t ‖C_3d(t) − S(p(t))‖` — how far the edge's own 3D curve sits from the
/// locus its pcurve traces on `surface`, recorded against `band`.
///
/// This is the measurement that makes a general pcurve → 3D transfer
/// trustworthy: the transfer's whole claim is that the interpolated 3D curve
/// reproduces the composed image, and this is that claim, measured rather than
/// assumed. It is equally what an iso SHORTCUT owes: `rim_welds.rs` accepts an
/// iso on a structural degree/knot match, and this is the geometric check that
/// would replace it.
///
/// `forward` follows the coedge's own sense, as in `BrepSolid::validate`.
pub fn measure_edge_against_pcurve_image(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
    band: f64,
) -> Result<MeasuredTolerance, KernelRefusal> {
    let adaptive = coedge_error_floored(surface, pcurve, &edge.curve, edge, forward, band).or_refuse(KernelStage::Validate, "coedge_error_floored")?;
    let spans = span_midpoint_error(surface, pcurve, edge, forward)?;
    Ok(MeasuredTolerance::new(adaptive.max(spans), band))
}

/// The same comparison as [`measure_edge_against_pcurve_image`], sampled once at
/// the midpoint of every knot span of the edge's own 3D curve.
///
/// This is the anti-aliasing half described in the module doc: it samples where
/// the curve is furthest from what it interpolates, by construction, and its
/// sample set is derived from the curve's own knots rather than from a fixed
/// grid that a dyadically-subdivided curve can hide behind.
///
/// Exposed so the general pcurve → 3D image-curve transfer can floor its own
/// self-verification with it without re-deriving the span walk.
pub fn span_midpoint_error(
    surface: &NurbsSurface,
    pcurve: &NurbsCurve,
    edge: &EdgeRecord,
    forward: bool,
) -> Result<f64, KernelRefusal> {
    let span = edge.t1 - edge.t0;
    if !span.is_finite() || span.abs() <= 0.0 {
        return Ok(0.0);
    }
    let [q0, q1] = pcurve.domain().or_refuse(KernelStage::Validate, "domain")?;
    // Distinct interior breakpoints of the curve, clipped to the edge's own
    // parameter range. Repeated knots (a degree-1 curve clamps its ends) collapse
    // to one, so a span is a real interval and its midpoint a real interior point.
    let low = edge.t0.min(edge.t1);
    let high = edge.t0.max(edge.t1);
    let mut breaks: Vec<f64> = vec![low];
    for &knot in &edge.curve.knots {
        if knot > low && knot < high && knot > *breaks.last().unwrap_or(&low) {
            breaks.push(knot);
        }
    }
    breaks.push(high);

    let mut worst = 0.0f64;
    for pair in breaks.windows(2) {
        let midpoint = (pair[0] + pair[1]) * 0.5;
        if !(midpoint > pair[0] && midpoint < pair[1]) {
            continue;
        }
        // `adaptive_coedge_error`'s own fraction -> parameter map, inverted, so
        // the two passes compare the SAME pairing of pcurve point to curve point.
        let fraction = if forward {
            (midpoint - edge.t0) / span
        } else {
            (edge.t1 - midpoint) / span
        };
        let uv = pcurve.evaluate(q0 + (q1 - q0) * fraction).or_refuse(KernelStage::Validate, "evaluate")?;
        let on_surface = surface.evaluate_extended(uv.x, uv.y).or_refuse(KernelStage::Validate, "evaluate_extended")?;
        let on_curve = edge.curve.evaluate(midpoint).or_refuse(KernelStage::Validate, "evaluate")?;
        worst = worst.max(on_surface.sub(on_curve).length());
    }
    Ok(worst)
}

/// `max_{u,v} ‖S_fit(u,v) − offset(u,v)‖` — how far a fitted offset carrier
/// sits from the pointwise offset it was fitted to, recorded against `band`.
///
/// `distance` and `same_sense` are `offset_surface`'s, and the evaluator is
/// constructed with the same convention that produced the fit's samples
/// ([`OffsetNormal::FaceStable`], with the same single negation), so this
/// measures the FIT and nothing else — not a sign disagreement, and not a
/// second opinion about which way "outward" points.
///
/// Only meaningful when the carrier really is a pointwise fit over the same
/// parameterisation. `offset_surface`'s planar/ruled extension and its
/// apex-cone pinch retrim both move the sample grid off the pointwise offset on
/// purpose; measuring one of those against a pointwise offset reports a
/// designed divergence, not an error. `offset_surface_measured` decides that
/// and does not call this in those cases.
pub fn measure_surface_fit_against_pointwise_offset(
    source: &NurbsSurface,
    same_sense: bool,
    fitted: &NurbsSurface,
    distance: f64,
    band: f64,
) -> Result<MeasuredTolerance, KernelRefusal> {
    let evaluator = OffsetEvaluator::new(
        "measure_offset_fit",
        source,
        OffsetNormal::FaceStable { same_sense },
    );
    let [u0, u1] = source.domain_u().or_refuse(KernelStage::Validate, "domain_u")?;
    let [v0, v1] = source.domain_v().or_refuse(KernelStage::Validate, "domain_v")?;
    let mut worst = 0.0f64;
    for iu in 0..FIT_SAMPLES {
        let u = u0 + (u1 - u0) * (iu as f64 + FIT_OFFSET_U) / FIT_SAMPLES as f64;
        for iv in 0..FIT_SAMPLES {
            let v = v0 + (v1 - v0) * (iv as f64 + FIT_OFFSET_V) / FIT_SAMPLES as f64;
            // `offset_surface`'s positive distance moves OPPOSITE the face
            // normal while the evaluator's moves ALONG it; the negation is the
            // fit's own, repeated here verbatim rather than re-derived.
            let want = evaluator.at(u, v, -distance).or_refuse(KernelStage::Validate, "at")?.point;
            let got = fitted.evaluate(u, v).or_refuse(KernelStage::Validate, "evaluate")?;
            worst = worst.max(got.sub(want).length());
        }
    }
    Ok(MeasuredTolerance::new(worst, band))
}

/// `|p_V − c_E(t_end)|` — how far a curve end sits from the vertex point the
/// topology says it meets.
///
/// The endpoint half of the edge → vertex propagation. On a freshly built
/// carrier this is exactly zero for the edge that CLAIMED the vertex (the
/// vertex was placed at that curve's end) and non-zero for every later edge
/// that reuses it — which is the whole reason it is worth measuring.
pub fn vertex_endpoint_gap(vertex_point: Vec3, curve_end: Vec3) -> f64 {
    curve_end.sub(vertex_point).length()
}




/// The completed spherical construction reading, reusable only by identical
/// source/result content. The first request always uses the ordinary sampler.
pub(crate) fn sphere_fit_reading(
    source: &NurbsSurface, same_sense: bool, fitted: &NurbsSurface, distance: f64, band: f64,
) -> Result<MeasuredTolerance, KernelRefusal> {
    sphere_fit_reading_mode(source, same_sense, fitted, distance, band,
        std::env::var("BREP_OFFSET_SPHERE_READING").map_or(true, |v| v != "0" && v != "off"))
}
thread_local! {
    static SPHERE_FIT_READINGS: std::cell::RefCell<rustc_hash::FxHashMap<Vec<u64>, f64>> =
        std::cell::RefCell::new(rustc_hash::FxHashMap::default());
}

// Exact content equality, including every homogeneous control bit. The nets
// are immutable by NurbsSurface's contract. No recognition label or sampled
// closeness is used as an identity. Limit both key size and entry count.
fn sphere_fit_key(source: &NurbsSurface, fitted: &NurbsSurface, same_sense: bool, distance: f64) -> Option<Vec<u64>> {
    let size = |s: &NurbsSurface| 5 + s.knots_u.len() + s.knots_v.len()
        + s.control_points.iter().map(|row| 1 + 4 * row.len()).sum::<usize>();
    if size(source) + size(fitted) + 3 > 4096 { return None; }
    let mut key = vec![same_sense as u64, distance.to_bits(),
        (std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0")) as u64];
    for surface in [source, fitted] {
        key.extend([surface.degree_u as u64, surface.degree_v as u64]);
        for knots in [&surface.knots_u, &surface.knots_v] {
            key.push(knots.len() as u64);
            key.extend(knots.iter().map(|v| v.to_bits()));
        }
        key.push(surface.control_points.len() as u64);
        for row in &surface.control_points {
            key.push(row.len() as u64);
            for p in row { key.extend([p.x.to_bits(), p.y.to_bits(), p.z.to_bits(), p.w.to_bits()]); }
        }
    }
    Some(key)
}

fn sphere_fit_reading_mode(
    source: &NurbsSurface, same_sense: bool, fitted: &NurbsSurface, distance: f64, band: f64, enabled: bool,
) -> Result<MeasuredTolerance, KernelRefusal> {
    // A requested diagnostic trace still visits every station.
    let key = (enabled && std::env::var_os("BREP_OFFSET_DIAG").is_none())
        .then(|| sphere_fit_key(source, fitted, same_sense, distance)).flatten();
    if let Some(deviation) = key.as_ref().and_then(|key|
        SPHERE_FIT_READINGS.with(|memo| memo.borrow().get(key).copied())) {
        if std::env::var_os("BREP_PROFILE").is_some() {
            eprintln!("offset.sphere_fit_memo hit=1 stations=0");
        }
        // Band is not an input to the fixed grid. Preserve the actual reading
        // and judge it against this caller's band, never the stored caller's.
        return Ok(MeasuredTolerance::new(deviation, band));
    }
    let measured = measure_surface_fit_against_pointwise_offset(source, same_sense, fitted, distance, band)?;
    if let Some(key) = key {
        SPHERE_FIT_READINGS.with(|memo| {
            let mut memo = memo.borrow_mut();
            if memo.len() >= 64 { memo.clear(); }
            memo.insert(key, measured.deviation());
        });
    }
    if std::env::var_os("BREP_PROFILE").is_some() {
        eprintln!("offset.sphere_fit_memo hit=0 stations={}", FIT_SAMPLES * FIT_SAMPLES);
    }
    Ok(measured)
}


