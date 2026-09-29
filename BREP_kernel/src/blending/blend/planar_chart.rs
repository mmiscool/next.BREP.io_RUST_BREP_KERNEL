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
    let band = crate::blend::consumed_band(result);
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
        return Ok(0);
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
    Ok(moved)
}
