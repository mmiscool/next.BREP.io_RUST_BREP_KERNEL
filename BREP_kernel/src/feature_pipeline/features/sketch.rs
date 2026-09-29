//! S / SKETCH — the profile-producing construction feature.
//!
//! The 2D constraint
//! SOLVER already lives in Rust (`solvers/sketch_solver.rs`); what was missing —
//! and what this feature adds — is turning a solved sketch into an extractable
//! 3D PROFILE the add-material features (extrude/revolve/loft/sweep/rib +
//! sheet-metal tab/contour-flange) consume by name.
//!
//! Pipeline:
//!   1. Read the raw sketch (`persistentData.sketch` = `{points, geometries,
//!      constraints}`) and the plane `basis` (`persistentData.basis`). The basis
//!      is computed SCENE-SIDE (from object
//!      transforms / face normals) and marshaled in — directive §1 keeps the
//!      reference/plane resolution scene-side; Rust just consumes the frame.
//!   2. Pre-evaluate expression-string coordinates/values against the shared
//!      pipeline `Env` (a point `x`/`y` or a constraint `value`/`valueExpr` may be
//!      `"width/2"`) — the solver only understands numbers. An expression that
//!      does NOT resolve FAILS the sketch: the solver's `coerce_coord` reads a
//!      surviving string by its leading numeric PREFIX (`"20*Hh"` → `20`), which
//!      silently built a watertight solid of the wrong size.
//!   3. `solve_sketch` → solved point coordinates.
//!   4. Reconstruct the model geometries (skip `construction`), chain them into
//!      maximal endpoint-connected runs. CLOSED runs become profile loops
//!      (outer-vs-hole by containment); OPEN runs become the sketch PATH — the
//!      `loops` and `openChains` are built side by side,
//!      so a single sketch legally carries a closed profile PLUS a separate open
//!      sweep trajectory, and neither invalidates the other.
//!   5. Emit each loop as `NurbsCurve`s placed in the plane frame; store the
//!      `SketchProfile` in the scene under the sketch's id (contract: no solid,
//!      like a construction pass-through, but carrying a profile).
//!
//! Geometry point semantics are the ground truth from the edge builder:
//! `line = [p0, p1]`; `arc = [center, start, end]`
//! sweeping CCW start→end (the pipeline normalizes the sweep into `[0, 2π)`); `circle =
//! [center, radiusPoint]` (a closed loop by itself); `ellipse = [center,
//! majorAxisEnd, minorAxisEnd]` (the affine image of the unit circle, a closed
//! loop by itself); `bezier = [p0, c1, c2, p1, …]`
//! (`3n+1` control points, chained cubic spans) rebuilds as one clamped degree-3
//! B-spline (see [`SegKind::Bezier`] / [`bezier_chain_curve`]). Any OTHER geometry
//! type errors loudly (no-fallback); the sketcher's freehand/curve tool emits
//! `bezier` (the standalone 3D spline is a separate feature, not a sketch
//! geometry).

use std::f64::consts::{PI, TAU};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{
    Axis, FeatureContext, FeatureResult, Frame, ProfileLoop, SketchProfile,
};
use crate::{
    is_sketch_origin_point_id, make_arc, make_line, solve_sketch, NurbsCurve, SolveSketchRequest, Vec3,
    Vec4,
};
use serde_json::Value;

/// Endpoint-coincidence tolerance for chaining loop segments (sketch units = mm).
/// Matches the join tolerance — `PROFILE_ENDPOINT_EPS = 1e-5` and 5-decimal
/// endpoint keying. The kernel itself demands 1e-6 closure
/// (`extrude_profile_brep`), so after chaining every emitted run is WELDED
/// exactly head-to-tail by [`weld_chain`].
const JOIN_TOL: f64 = 1e-5;

mod frame;
mod geometry;
mod chain;
mod project;
mod loop_ids;
mod regions;
mod pipeline;

pub use loop_ids::assign_sketch_loop_ids;
pub use pipeline::{execute, schema};

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected face OR construction plane/datum can seat `sketchPlane` (the
/// `sketchPlane` field's `["PLANE","FACE"]` filter, resolved frame-or-face by
/// [`frame::resolve_frame`]), and edges can seed `projectedEdges` (in-context
/// sketching on model geometry).
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0 || probe.planes > 0
}

