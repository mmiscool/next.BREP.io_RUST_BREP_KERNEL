//! A planar carrier must hold its own trims.
//!
//! # The defect
//!
//! A face's surface is a FINITE chart: a plane carrier is a rectangle, sized by
//! whichever feature built the face, not an infinite plane.  Every pcurve on it
//! is built by projecting the 3D edge and then CLAMPING the projection into the
//! chart's parameter box (`pcurve.rs`, the `parameter[0].clamp(u0, u1)` pair —
//! an open direction has no wrap to fall back on).  So when the blend's surgery
//! gives a face a trim that runs OUTSIDE the chart it was created with, the
//! clamp silently folds that trim onto the chart's boundary: the edges are
//! right, the loop closes in 3D, and the FACE is the wrong region.
//!
//! Nothing downstream notices. `validate()` sees a watertight, incident,
//! connected body; `check_loop_self_crossings` sees a simple loop (the folded
//! trim does not cross itself); only the shell vector-area scan reads it, and
//! only as a number nobody had attached to a cause.  On the 2026-09-08 rib-base
//! document the reported fillet F8 leaves `RIB6:S5:G0_TOP` with four trims up to
//! 3.030e-1 outside its 9.7705 x 2.0000 chart, and the body it integrates reads
//! **350.388894999** against the edge-bounded **350.358599** — a 3.03e-2 error
//! that `validate()` passes, with the face's closure departure at 2.026e-1
//! against its 2.15e-6 bar (7470x) and its loop gap at 6.869e-1.
//!
//! # The repair, and why it is the carrier that moves
//!
//! Two answers were possible: stop the rail at the face's real boundary, or
//! extend the carrier to hold it.  The measurement picks the second — the trims
//! that leave the chart lie IN the plane (off-plane <= 1.3e-12 on that document)
//! and the loop they form closes on its own vertices (3D joint gaps <= 4.2e-13).
//! The rail is right and the rectangle is too small; there is nothing to fix
//! about the rail, and stopping it would move geometry that is already correct.
//!
//! So: widen the plane's rectangle — same plane, same `eu`/`ev`, same normal, so
//! the face's sense and the solid's orientation are untouched — until it
//! contains every trim of that face, then rebuild that face's pcurves on the new
//! chart.  This is exact for a plane and only for a plane: a planar chart is an
//! affine window onto a surface that continues correctly outside it, which is
//! not true of a general carrier (a B-spline's extension is an extrapolation),
//! so nothing here fires on a non-planar face.
//!
//! # What it refuses
//!
//! The repair is held to the geometry it claims, not to `validate()`:
//!
//! * an out-of-chart trim whose EDGE does not lie in the plane is a wrong body,
//!   not a chart that is too small — [`PLANAR_CHART_EDGE_OFF_PLANE`];
//! * a rebuilt trim that does not follow its own edge to the refinement floor —
//!   [`PLANAR_CHART_WIDEN_UNSOUND`].
//!
//! Both are TERMINAL in the group lane, like [`crate::blend::is_consumed_snap`]:
//! the cutter composition is not a second opinion on a wrong body.

use crate::{KernelRefusal, KernelStage, OrRefuse, RefusalClass};
use std::collections::{HashMap, HashSet};

use crate::pcurve::PCURVE_REFINEMENT_TOLERANCE;
use crate::topology::BrepSolid;
use crate::{AnalyticSurface, NurbsCurve, Vec3};

/// An out-of-chart trim whose edge is not in the carrier's plane: widening the
/// rectangle cannot hold a curve that is not in the plane at all, and a trim
/// claiming such an edge is a wrong body rather than a small chart.
pub(crate) const PLANAR_CHART_EDGE_OFF_PLANE: &str =
    "blend: a trim leaves its planar carrier's chart and its edge is not in that plane —";

/// A widened chart whose rebuilt trim does not follow the edge it claims — see
/// [`fit_planar_charts_to_trims`].
pub(crate) const PLANAR_CHART_WIDEN_UNSOUND: &str =
    "blend: a widened planar carrier's trim does not follow its edge —";

/// The slugs of the planar-chart refusals: the off-plane edge and the widened
/// carrier are `UnsupportedGeometry`; the widened face that carries an edge the
/// solid does not have is `Internal` (its text is under
/// [`PLANAR_CHART_WIDEN_UNSOUND`] too, and the fallbacks treat it the same).
pub(crate) const PLANAR_CHART_OFF_PLANE_WHAT: &str = "planar_chart_off_plane";
pub(crate) const PLANAR_CHART_WIDEN_WHAT: &str = "planar_chart_widen";
pub(crate) const PLANAR_CHART_COEDGE_WHAT: &str = "planar_chart_coedge";

/// Is this a refusal of a trim that left its planar chart ([`PLANAR_CHART_EDGE_OFF_PLANE`], [`PLANAR_CHART_WIDEN_UNSOUND`])?
/// Read off the class and its slug, which the mint site and this check share.
pub(crate) fn is_planar_chart_refusal(refusal: &KernelRefusal) -> bool {
    matches!(&refusal.class, RefusalClass::UnsupportedGeometry { what } if what == PLANAR_CHART_OFF_PLANE_WHAT) ||
        matches!(&refusal.class, RefusalClass::UnsupportedGeometry { what } if what == PLANAR_CHART_WIDEN_WHAT) ||
        matches!(&refusal.class, RefusalClass::Internal { what } if what == PLANAR_CHART_COEDGE_WHAT)
}

/// Samples per edge when measuring how far a trim's edge leaves the chart.
const CHART_SAMPLES: usize = 256;

/// Samples per rebuilt trim when checking it against its own edge.
const VERIFY_SAMPLES: usize = 128;

/// One planar face whose chart does not hold its own trims.
struct Widening {
    shell: usize,
    face: usize,
    origin: Vec3,
    eu: Vec3,
    ev: Vec3,
    span_u: f64,
    span_v: f64,
    /// The excursion that triggered it, for the caller's diagnostics.
    excursion: f64,
}

/// How far the curve `at` over `[t0, t1]` runs outside the chart of a PLANE
/// carrier, in the chart's own frame — the excursion
/// [`fit_planar_charts_to_trims`] reads off a face's trims — or `None` when
/// `surface` is not a plane (a general carrier has no extension to widen onto).
/// A support trim whose rail overruns its plane's chart by more than the
/// consumed band is rebuilt from its edge on the widened chart, and verified
/// there at the refinement floor, by that pass; any pcurve fitted or read
/// against the unwidened chart before it is clamped onto the chart's
/// boundary and means nothing.
pub(crate) fn planar_chart_overrun(
    surface: &crate::NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    t0: f64,
    t1: f64,
) -> Result<Option<f64>, KernelRefusal> {
    if !matches!(surface.analytic(), Some(AnalyticSurface::Plane { .. })) {
        return Ok(None);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let origin = surface.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?;
    let du = surface.evaluate(u1, v0).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
    let dv = surface.evaluate(u0, v1).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
    let (length_u, length_v) = (du.length(), dv.length());
    let (eu, ev) = (du.normalized().or_refuse(KernelStage::Refine, "normalized")?, dv.normalized().or_refuse(KernelStage::Refine, "normalized")?);
    let mut excursion = 0.0_f64;
    for step in 0..=CHART_SAMPLES {
        let point = at(t0 + (t1 - t0) * step as f64 / CHART_SAMPLES as f64).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
        let (u, v) = (point.dot(eu), point.dot(ev));
        excursion = excursion.max(-u).max(u - length_u).max(-v).max(v - length_v);
    }
    Ok(Some(excursion))
}

/// Widen every planar carrier in `result` whose own trims leave its chart, and
/// rebuild those faces' pcurves.  Returns how many carriers moved.
///
/// Only faces the blend TOUCHED are considered — a face carrying at least one
/// coedge whose edge is not in `input`.  A chart that was already too small
/// before the operation is a defect of whatever built it, and repairing it here
/// would move a body for a reason this operation is not responsible for; the
/// corpus census of 2026-09-18 found none.
pub(crate) fn fit_planar_charts_to_trims(
    input: &BrepSolid,
    result: &mut BrepSolid,
) -> Result<usize, KernelRefusal> {
    // The SMALLER of the input's and the result's bands: a support trim the
    // surgery deferred to this pass (`planar_chart_overrun` over
    // `consumed_band` of the solid it sewed into, which every caller passes
    // here as `input`) is always widened and rebuilt here, whichever way the
    // operation moved the body's size.
    let band = crate::blend::consumed_band(input).min(crate::blend::consumed_band(result));
    let existing: HashSet<u64> = input.edges.iter().map(|edge| edge.id).collect();
    let curves: HashMap<u64, (&NurbsCurve, f64, f64)> = result
        .edges
        .iter()
        .map(|edge| (edge.id, (&edge.curve, edge.t0, edge.t1)))
        .collect();

    let mut widenings = Vec::new();
    for (shell_index, shell) in result.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            if !matches!(
                face.surface.analytic(),
                Some(AnalyticSurface::Plane { .. })
            ) {
                continue;
            }
            let touched = face
                .loops
                .iter()
                .flat_map(|hole| &hole.coedges)
                .any(|coedge| !existing.contains(&coedge.edge_id));
            if !touched {
                continue;
            }
            let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
            let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
            let origin = face.surface.evaluate(u0, v0).or_refuse(KernelStage::Sew, "evaluate")?;
            let du = face.surface.evaluate(u1, v0).or_refuse(KernelStage::Sew, "evaluate")?.sub(origin);
            let dv = face.surface.evaluate(u0, v1).or_refuse(KernelStage::Sew, "evaluate")?.sub(origin);
            let (length_u, length_v) = (du.length(), dv.length());
            let (eu, ev) = (du.normalized().or_refuse(KernelStage::Sew, "normalized")?, dv.normalized().or_refuse(KernelStage::Sew, "normalized")?);
            let normal = eu.cross(ev).normalized().or_refuse(KernelStage::Sew, "normalized")?;
            // The chart's own extent in its own frame, then every trim's edge.
            let (mut low_u, mut high_u) = (0.0_f64, length_u);
            let (mut low_v, mut high_v) = (0.0_f64, length_v);
            let mut off_plane = 0.0_f64;
            let mut worst_edge = 0_u64;
            for coedge in face.loops.iter().flat_map(|hole| &hole.coedges) {
                let Some((curve, t0, t1)) = curves.get(&coedge.edge_id) else {
                    continue;
                };
                for step in 0..=CHART_SAMPLES {
                    let t = t0 + (t1 - t0) * step as f64 / CHART_SAMPLES as f64;
                    let point = curve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?.sub(origin);
                    low_u = low_u.min(point.dot(eu));
                    high_u = high_u.max(point.dot(eu));
                    low_v = low_v.min(point.dot(ev));
                    high_v = high_v.max(point.dot(ev));
                    let height = point.dot(normal).abs();
                    if height > off_plane {
                        off_plane = height;
                        worst_edge = coedge.edge_id;
                    }
                }
            }
            let excursion = (-low_u)
                .max(high_u - length_u)
                .max(-low_v)
                .max(high_v - length_v);
            if excursion <= band {
                continue;
            }
            if off_plane > band {
                return Err(KernelRefusal::unsupported(KernelStage::Sew, PLANAR_CHART_OFF_PLANE_WHAT, format!(
                    "{PLANAR_CHART_EDGE_OFF_PLANE} face {} edge {worst_edge} is {off_plane:.3e} \
                     off the plane (band {band:.3e}), and its trim runs {excursion:.3e} outside \
                     the chart",
                    face.name.clone().unwrap_or_else(|| format!("{}", face.id)),
                )));
            }
            // Pad so the rebuilt trims land strictly inside the new box rather
            // than on its boundary, where the clamp would still be live.
            let pad = (1e-6 * (1.0 + length_u.max(length_v))).max(band);
            let (low_u, low_v) = (low_u - pad, low_v - pad);
            widenings.push(Widening {
                shell: shell_index,
                face: face_index,
                origin: origin.add(eu.scale(low_u)).add(ev.scale(low_v)),
                eu,
                ev,
                span_u: high_u + pad - low_u,
                span_v: high_v + pad - low_v,
                excursion,
            });
        }
    }
    if widenings.is_empty() {
        return fit_ruled_charts_to_trims(input, result);
    }

    // Phase two mutates, so take the curves it needs by value first.
    let curves: HashMap<u64, (NurbsCurve, f64, f64)> = result
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.curve.clone(), edge.t0, edge.t1)))
        .collect();
    let moved = widenings.len();
    for widening in widenings {
        let plane = crate::make_plane(
            widening.origin,
            widening.eu,
            widening.ev,
            widening.span_u,
            widening.span_v,
        ).or_refuse(KernelStage::Sew, "make_plane")?;
        let face = &mut result.shells[widening.shell].faces[widening.face];
        let name = face
            .name
            .clone()
            .unwrap_or_else(|| format!("{}", face.id));
        // Same plane, same frame: the carrier's normal must be untouched, or
        // the face's `same_sense` no longer means what it meant.
        let before = face.surface.normal(
            0.5 * (face.surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?[0] + face.surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?[1]),
            0.5 * (face.surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?[0] + face.surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?[1]),
        ).or_refuse(KernelStage::Sew, "normal")?;
        let after = plane.normal(0.5 * widening.span_u, 0.5 * widening.span_v).or_refuse(KernelStage::Sew, "normal")?;
        if before.dot(after) <= 0.0 {
            return Err(KernelRefusal::unsupported(KernelStage::Sew, PLANAR_CHART_WIDEN_WHAT, format!(
                "{PLANAR_CHART_WIDEN_UNSOUND} face {name}'s widened carrier reverses its normal",
            )));
        }
        for coedge in face.loops.iter_mut().flat_map(|hole| &mut hole.coedges) {
            let Some((curve, t0, t1)) = curves.get(&coedge.edge_id) else {
                return Err(KernelRefusal::internal(KernelStage::Sew, PLANAR_CHART_COEDGE_WHAT, format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name} carries coedge {} whose edge {} is \
                     not in the solid",
                    coedge.id, coedge.edge_id,
                )));
            };
            coedge.pcurve = crate::build_pcurve_on_surface_range(
                &plane,
                curve,
                *t0,
                *t1,
                coedge.forward,
                PCURVE_REFINEMENT_TOLERANCE,
            )
            .map_err(|error| {
                format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name}'s trim for edge {} does not rebuild \
                     on the widened chart ({error})",
                    coedge.edge_id,
                )
            }).or_refuse(KernelStage::Sew, "map_err")?;
            // The trim is what the shell's closure integrates, so check the
            // rebuilt one against the edge it claims rather than against the
            // fit's own samples.
            let [p0, p1] = coedge.pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
            let mut miss = 0.0_f64;
            for step in 0..=VERIFY_SAMPLES {
                let t = p0 + (p1 - p0) * step as f64 / VERIFY_SAMPLES as f64;
                let uv = coedge.pcurve.evaluate(t).or_refuse(KernelStage::Sew, "evaluate")?;
                let point = plane.evaluate(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate")?;
                miss = miss.max(crate::project_point_to_curve(curve, point).or_refuse(KernelStage::Sew, "project_point_to_curve")?.distance);
            }
            if miss > PCURVE_REFINEMENT_TOLERANCE {
                return Err(KernelRefusal::unsupported(KernelStage::Sew, PLANAR_CHART_WIDEN_WHAT, format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name}'s rebuilt trim for edge {} misses it \
                     by {miss:.3e} (floor {PCURVE_REFINEMENT_TOLERANCE:.0e}); the chart ran \
                     {:.3e} short",
                    coedge.edge_id, widening.excursion,
                )));
            }
        }
        face.surface = plane;
    }
    Ok(moved + fit_ruled_charts_to_trims(input, result)?)
}

// ======================================================================
// Lane L (2026-10-15): an exactly RULED carrier must hold its trims too.
//
// smFilletTest lCount_1 (cyl_cyl_219, r 1, edges 6 and 10) shipped a body
// failing `validate()`: each blend ends on a NURBS side face (2 x 9 control
// points, degree 1 along u) as an EXTEND landing, so that face's boundary on
// the floor is extended to u = -1, a radius past its domain [0, 10], and the
// blend's end edge on the face runs from u = -1 to u = 0.  The edges are right
// (Newton on `evaluate_extended` puts the end edge on the face's extension to
// 1e-13); the projection that built the end edge's pcurve CLAMPED it onto
// u = 0, so that coedge stood up to r = 1.0 off its edge.
//
// The planar pass above widens a plane because a plane continues itself
// exactly.  So does a direction of degree ONE with a single span whose two
// control rows carry equal weights: the surface is affine in that parameter
// (homogeneous coordinates affine, w constant along it), so
// `evaluate_extended`'s ruled continuation IS the surface, and extrapolating
// the two control rows linearly re-spans it with the SAME parameterisation.
// Every pcurve that followed its edge still does and is left untouched; only
// the coedges that MISS their edge are rebuilt.  The pass fires only on a
// measured miss, so a body whose trims all follow their edges is never moved.
// `BREP_BLEND_RULED_CHART=0`/`off` turns it off.
// ======================================================================

/// The ruled re-span's switch: on unless `BREP_BLEND_RULED_CHART=0`/`off`.
/// It also switches `transverse_curve`'s exact ends at EXTEND landings
/// (`edge/support.rs`), the other half of the same repair.
pub(in crate::blend) fn ruled_chart_on() -> bool {
    !matches!(std::env::var("BREP_BLEND_RULED_CHART").as_deref(), Ok("0") | Ok("off"))
}

/// Samples per coedge when measuring whether its pcurve follows its edge.
const RULED_MISS_SAMPLES: usize = 32;

/// Newton steps for a foot on a surface's extension.
const EXTENDED_FOOT_ITERATIONS: usize = 50;

/// The direction (`true` = u) along which `surface` is exactly ruled, and
/// so extends exactly: degree 1, one span, equal weights across it.
fn exact_ruled_direction(surface: &crate::NurbsSurface) -> Option<bool> {
    let rows = &surface.control_points;
    let same_weight = |a: &crate::Vec4, b: &crate::Vec4| (a.w - b.w).abs() <= 1e-12 * a.w.abs().max(b.w.abs()).max(1.0);
    if surface.degree_u == 1
        && surface.knots_u.len() == 4
        && rows.len() == 2
        && rows[0].len() == rows[1].len()
        && rows[0].iter().zip(&rows[1]).all(|(a, b)| same_weight(a, b))
    {
        return Some(true);
    }
    if surface.degree_v == 1
        && surface.knots_v.len() == 4
        && rows.iter().all(|row| row.len() == 2 && same_weight(&row[0], &row[1]))
    {
        return Some(false);
    }
    None
}

/// The foot of `point` on `surface`'s EXTENSION: the clamped projection as
/// the seed, then unclamped Newton on `derivatives_extended`.  (u, v, distance).
pub(in crate::blend) fn extended_foot(surface: &crate::NurbsSurface, point: Vec3) -> Result<(f64, f64, f64), KernelRefusal> {
    let seed = crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Sew, "project_point_to_surface")?;
    extended_foot_from(surface, point, [seed.u, seed.v])
}

/// [`extended_foot`] from a caller's `seed` — the foot a branch-continuing
/// projection already found — so the Newton stays on that branch.
pub(in crate::blend) fn extended_foot_from(surface: &crate::NurbsSurface, point: Vec3, seed: [f64; 2]) -> Result<(f64, f64, f64), KernelRefusal> {
    let [mut u, mut v] = seed;
    for _ in 0..EXTENDED_FOOT_ITERATIONS {
        let d = surface.derivatives_extended(u, v, 1).or_refuse(KernelStage::Sew, "derivatives_extended")?;
        let (r, su, sv) = (d[0][0].sub(point), d[1][0], d[0][1]);
        let (a11, a12, a22) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let (b1, b2) = (-su.dot(r), -sv.dot(r));
        let det = a11 * a22 - a12 * a12;
        if !(det.abs() > 1e-300) {
            break;
        }
        let (du, dv) = ((b1 * a22 - b2 * a12) / det, (a11 * b2 - a12 * b1) / det);
        u += du;
        v += dv;
        if du.abs() + dv.abs() <= 1e-15 * (1.0 + u.abs() + v.abs()) {
            break;
        }
    }
    let distance = surface.evaluate_extended(u, v).or_refuse(KernelStage::Sew, "evaluate_extended")?.sub(point).length();
    Ok((u, v, distance))
}

/// How far `pcurve` on `surface` stands off `curve` (the farthest of
/// [`RULED_MISS_SAMPLES`] samples, read by projection onto the curve), and
/// the range those samples span along the ruled direction (`along_u`).
fn pcurve_miss(surface: &crate::NurbsSurface, pcurve: &NurbsCurve, curve: &NurbsCurve, along_u: bool) -> Result<(f64, [f64; 2]), KernelRefusal> {
    let [p0, p1] = pcurve.domain().or_refuse(KernelStage::Sew, "domain")?;
    let mut miss = 0.0_f64;
    let mut range = [f64::INFINITY, f64::NEG_INFINITY];
    for step in 0..=RULED_MISS_SAMPLES {
        let uv = pcurve.evaluate(p0 + (p1 - p0) * step as f64 / RULED_MISS_SAMPLES as f64).or_refuse(KernelStage::Sew, "evaluate")?;
        let d = if along_u { uv.x } else { uv.y };
        range = [range[0].min(d), range[1].max(d)];
        let point = surface.evaluate_extended(uv.x, uv.y).or_refuse(KernelStage::Sew, "evaluate_extended")?;
        miss = miss.max(crate::project_point_to_curve(curve, point).or_refuse(KernelStage::Sew, "project_point_to_curve")?.distance);
    }
    Ok((miss, range))
}

/// Re-span every exactly ruled NURBS carrier in `result` that a coedge's
/// pcurve MISSES because its edge runs past the carrier's domain along the
/// ruled direction, and rebuild the missing coedges on it.  Only faces the
/// operation touched (a coedge whose edge is not in `input`).  Returns how
/// many carriers moved.
fn fit_ruled_charts_to_trims(input: &BrepSolid, result: &mut BrepSolid) -> Result<usize, KernelRefusal> {
    if !ruled_chart_on() {
        return Ok(0);
    }
    let band = crate::blend::consumed_band(input).min(crate::blend::consumed_band(result));
    // A trim MISSES its edge, and an edge lies ON the extension, by the
    // kernel's own contract for a fitted edge on its face — fitted transverse
    // curves stand a few microns off their faces (2e-6 to 5e-6 on lCount_1's
    // in-domain faces), the clamped trim this pass repairs a full radius.
    let consistency = crate::KernelTolerances::for_solid(input, 1e-7)
        .pcurve_consistency
        .min(crate::KernelTolerances::for_solid(result, 1e-7).pcurve_consistency);
    let existing: HashSet<u64> = input.edges.iter().map(|edge| edge.id).collect();
    let curves: HashMap<u64, (NurbsCurve, f64, f64)> = result
        .edges
        .iter()
        .map(|edge| (edge.id, (edge.curve.clone(), edge.t0, edge.t1)))
        .collect();
    // (shell, face, re-spanned carrier, the coedge ids to rebuild, excursion,
    // the missed edges' distance off the extension)
    let mut respans: Vec<(usize, usize, crate::NurbsSurface, Vec<u64>, f64, f64, bool)> = Vec::new();
    let debug = std::env::var("BREP_DEBUG_RULED_CHART").is_ok();
    if debug {
        eprintln!("ruled chart: pass over {} faces", result.shells.iter().map(|shell| shell.faces.len()).sum::<usize>());
    }
    for (shell_index, shell) in result.shells.iter().enumerate() {
        for (face_index, face) in shell.faces.iter().enumerate() {
            let surface = &face.surface;
            if surface.analytic().is_some() {
                continue;
            }
            let Some(along_u) = exact_ruled_direction(surface) else {
                if debug {
                    eprintln!("ruled chart: face {:?} not exactly ruled (degrees {}x{}, {}x{} control points)", face.name, surface.degree_u, surface.degree_v, surface.control_points.len(), surface.control_points.first().map_or(0, |row| row.len()));
                }
                continue;
            };
            let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Sew, "closed_directions")?;
            let (closed_ruled, closed_other) = if along_u { (closed_u, closed_v) } else { (closed_v, closed_u) };
            if closed_ruled {
                continue;
            }
            if !face.loops.iter().flat_map(|hole| &hole.coedges).any(|coedge| !existing.contains(&coedge.edge_id)) {
                continue;
            }
            let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Sew, "domain_u")?;
            let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Sew, "domain_v")?;
            let ((low_d, high_d), (low_o, high_o)) = if along_u { ((u0, u1), (v0, v1)) } else { ((v0, v1), (u0, u1)) };
            // Two triggers: a trim that MISSES its edge (a pcurve clamped onto
            // the domain), and a trim that follows its edge OUTSIDE the domain
            // along the ruled direction (built on the extension).
            let mut missing: Vec<u64> = Vec::new();
            let (mut low, mut high) = (low_d, high_d);
            for coedge in face.loops.iter().flat_map(|hole| &hole.coedges) {
                let Some((curve, _, _)) = curves.get(&coedge.edge_id) else { continue };
                let (miss, range) = pcurve_miss(surface, &coedge.pcurve, curve, along_u)?;
                if miss > consistency {
                    if debug {
                        let [q0, q1] = coedge.pcurve.domain().unwrap_or([f64::NAN; 2]);
                        let (a, b) = (coedge.pcurve.evaluate(q0), coedge.pcurve.evaluate(q1));
                        eprintln!("ruled chart: face {:?} coedge {} edge {} misses by {miss:.3e}; pcurve ends {a:?} {b:?}; ruled range {range:?}", face.name, coedge.id, coedge.edge_id);
                    }
                    missing.push(coedge.id);
                } else {
                    low = low.min(range[0]);
                    high = high.max(range[1]);
                }
            }
            if missing.is_empty() && low >= low_d && high <= high_d {
                if debug {
                    eprintln!("ruled chart: face {:?} along_u {along_u}: every coedge follows its edge inside the domain", face.name);
                }
                continue;
            }
            let at = |d: f64, o: f64| if along_u { surface.evaluate(d, o) } else { surface.evaluate(o, d) };
            let mid_o = 0.5 * (low_o + high_o);
            let speed = at(high_d, mid_o).or_refuse(KernelStage::Sew, "evaluate")?
                .sub(at(low_d, mid_o).or_refuse(KernelStage::Sew, "evaluate")?)
                .length() / (high_d - low_d);
            let mid_d = 0.5 * (low_d + high_d);
            let speed_o = at(mid_d, high_o).or_refuse(KernelStage::Sew, "evaluate")?
                .sub(at(mid_d, low_o).or_refuse(KernelStage::Sew, "evaluate")?)
                .length() / (high_o - low_o);
            // Where the missed edges really lie: their feet on the extension.
            let (mut off_surface, mut across) = (0.0_f64, 0.0_f64);
            for coedge in face.loops.iter().flat_map(|hole| &hole.coedges).filter(|coedge| missing.contains(&coedge.id)) {
                let Some((curve, t0, t1)) = curves.get(&coedge.edge_id) else { continue };
                for step in 0..=CHART_SAMPLES {
                    let point = curve.evaluate(t0 + (t1 - t0) * step as f64 / CHART_SAMPLES as f64).or_refuse(KernelStage::Sew, "evaluate")?;
                    let (u, v, distance) = extended_foot(surface, point)?;
                    let (d, o) = if along_u { (u, v) } else { (v, u) };
                    low = low.min(d);
                    high = high.max(d);
                    off_surface = off_surface.max(distance);
                    if !closed_other {
                        across = across.max((low_o - o) * speed_o).max((o - high_o) * speed_o);
                    }
                }
            }
            let excursion = (low_d - low).max(high - high_d) * speed;
            // Not this pass's to repair: no excursion along the ruled
            // direction, an excursion across the curved one (no exact
            // extension there), or an edge that is not on the extension.
            if debug {
                eprintln!("ruled chart: face {:?} along_u {along_u}: {} coedges miss; ruled range [{low:.6}, {high:.6}] of [{low_d}, {high_d}], excursion {excursion:.3e}, across {across:.3e}, off the extension {off_surface:.3e} (band {band:.3e}, contract {consistency:.3e})", face.name, missing.len());
            }
            if excursion <= band || across > band || off_surface > consistency {
                continue;
            }
            let pad = (1e-6 * (1.0 + (high_d - low_d))).max(band / speed.max(1e-300));
            let (new_low, new_high) = ((low - pad).min(low_d), (high + pad).max(high_d));
            let extrapolate = |a: &crate::Vec4, b: &crate::Vec4, s: f64| -> crate::Vec4 {
                let f = (s - low_d) / (high_d - low_d);
                crate::Vec4 { x: a.x + (b.x - a.x) * f, y: a.y + (b.y - a.y) * f, z: a.z + (b.z - a.z) * f, w: a.w + (b.w - a.w) * f }
            };
            let respanned = if along_u {
                let rows = &surface.control_points;
                let row_at = |s: f64| -> Vec<crate::Vec4> { rows[0].iter().zip(&rows[1]).map(|(a, b)| extrapolate(a, b, s)).collect() };
                crate::NurbsSurface::new(1, surface.degree_v, vec![new_low, new_low, new_high, new_high], surface.knots_v.clone(), vec![row_at(new_low), row_at(new_high)])
            } else {
                let rows: Vec<Vec<crate::Vec4>> = surface
                    .control_points
                    .iter()
                    .map(|row| vec![extrapolate(&row[0], &row[1], new_low), extrapolate(&row[0], &row[1], new_high)])
                    .collect();
                crate::NurbsSurface::new(surface.degree_u, 1, surface.knots_u.clone(), vec![new_low, new_low, new_high, new_high], rows)
            }
            .or_refuse(KernelStage::Sew, "NurbsSurface::new")?;
            respans.push((shell_index, face_index, respanned, missing, excursion, off_surface, along_u));
        }
    }
    let moved = respans.len();
    for (shell_index, face_index, surface, missing, excursion, off_surface, along) in respans {
        let face = &mut result.shells[shell_index].faces[face_index];
        let name = face.name.clone().unwrap_or_else(|| format!("{}", face.id));
        for coedge in face.loops.iter_mut().flat_map(|hole| &mut hole.coedges) {
            if !missing.contains(&coedge.id) {
                continue;
            }
            let Some((curve, t0, t1)) = curves.get(&coedge.edge_id) else {
                return Err(KernelRefusal::internal(KernelStage::Sew, PLANAR_CHART_COEDGE_WHAT, format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name} carries coedge {} whose edge {} is not in the solid",
                    coedge.id, coedge.edge_id,
                )));
            };
            coedge.pcurve = crate::build_pcurve_on_surface_range(&surface, curve, *t0, *t1, coedge.forward, PCURVE_REFINEMENT_TOLERANCE)
                .map_err(|error| format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name}'s trim for edge {} does not rebuild on the re-spanned ruled carrier ({error})",
                    coedge.edge_id,
                ))
                .or_refuse(KernelStage::Sew, "map_err")?;
            // The rebuilt trim can follow its edge no closer than the edge
            // lies to the carrier; past that, the refinement floor.
            let floor = off_surface + PCURVE_REFINEMENT_TOLERANCE;
            let (miss, _) = pcurve_miss(&surface, &coedge.pcurve, curve, along)?;
            if std::env::var("BREP_DEBUG_RULED_CHART").is_ok() {
                eprintln!("ruled chart: face {name} re-spanned; coedge {} on edge {} rebuilt, {miss:.3e} off its edge (floor {floor:.3e})", coedge.id, coedge.edge_id);
            }
            if miss > floor {
                return Err(KernelRefusal::unsupported(KernelStage::Sew, PLANAR_CHART_WIDEN_WHAT, format!(
                    "{PLANAR_CHART_WIDEN_UNSOUND} face {name}'s rebuilt trim for edge {} misses it by {miss:.3e} on the re-spanned ruled carrier (floor {floor:.3e}: the edge stands {off_surface:.3e} off the carrier); the trims ran {excursion:.3e} past it",
                    coedge.edge_id,
                )));
            }
        }
        face.surface = surface;
    }
    Ok(moved)
}
