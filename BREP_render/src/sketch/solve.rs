//! Solve wiring — call the kernel's 2D constraint solver in-process.
//!
//! `brep-render` links the kernel crate directly (a plain Rust path dependency,
//! for BOTH native and the wasm canvas build), so the solver is reached by a
//! direct `brep_kernel::solve_sketch(&SolveSketchRequest)` call on every
//! target — there is NO wasm-bindgen JSON hop (that boundary only existed for the
//! previous app calling INTO the wasm module from the browser; here the solver is
//! compiled into the same binary/module). The solver is pure numeric Rust with no `std::time`,
//! so it is wasm-clean.
//!
//! The solver returns `{sketch: {points, geometries, constraints, diagnostics}}`.
//! We split that into the solved [`SketchDoc`] (the three editable arrays) and the
//! read-only [`SketchDiagnostics`] the overlay coloring + DOF readout consume.

use super::doc::{SketchDiagnostics, SketchDoc};
use brep_kernel::{solve_sketch, solve_sketch_min_change, SolveSketchRequest};

/// Whether a drag frame solves for the smallest change from the previous frame
/// (`BREP_SKETCH_DRAG_MIN_CHANGE=1`; OFF by default). It was promoted and then set
/// back to off: dragging a point held on a line whose far end is free, or a line
/// parallel to a fixed one, the minimum-norm steps move free endpoints across the
/// line, so over a long drag such a line grows several times its length or
/// collapses, which is worse than the plain per-frame solve. It stays opt-in until
/// the step keeps free line lengths and directions.
pub fn drag_min_change() -> bool {
    brep_kernel::switch!("BREP_SKETCH_DRAG_MIN_CHANGE", false)
}

/// User-tunable 2D solver knobs (the "Solver Settings" section), all optional
/// — `None` keeps the kernel's default. Held on the [`super::SketchSession`] and
/// read by every `resolve`, so a change re-solves with the new settings without
/// threading params through each edit path.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchSolverSettings {
    /// Convergence tolerance (kernel default when `None`).
    pub tolerance: Option<f64>,
    /// Max solver iterations (defaults to 1000 when `None`).
    pub iterations: Option<u32>,
    /// Distance-slide threshold ratio (kernel default when `None`).
    pub distance_slide_threshold_ratio: Option<f64>,
    /// Distance-slide step ratio (kernel default when `None`).
    pub distance_slide_step_ratio: Option<f64>,
    /// Distance-slide minimum step (kernel default when `None`).
    pub distance_slide_min_step: Option<f64>,
}

impl Default for SketchSolverSettings {
    fn default() -> Self {
        // All `None`/default → byte-identical to the historical hard-coded request
        // (iterations 1000, everything else the kernel default), so an untouched
        // solver behaves exactly as before this knob existed.
        Self {
            tolerance: None,
            iterations: Some(1000),
            distance_slide_threshold_ratio: None,
            distance_slide_step_ratio: None,
            distance_slide_min_step: None,
        }
    }
}

/// Solve `doc` and return the solved coordinates plus the constraint diagnostics.
///
/// `remove_implied_duplicates` is left off (the display path never edits
/// constraints, so we keep every authored constraint) and the Newton polish is
/// left on (`polish: None` → default true) so the displayed coordinates match
/// what a committed feature-pipeline solve would produce.
pub fn solve(doc: &SketchDoc) -> Result<(SketchDoc, SketchDiagnostics), String> {
    solve_with(doc, &SketchSolverSettings::default())
}

/// Like [`solve`], but with user-tunable [`SketchSolverSettings`] (the Solver
/// Settings panel). Default settings reproduce [`solve`] byte-for-byte.
pub fn solve_with(
    doc: &SketchDoc,
    settings: &SketchSolverSettings,
) -> Result<(SketchDoc, SketchDiagnostics), String> {
    solve_entry(doc, settings, false)
}

/// Like [`solve_with`], for one interactive drag frame: the kernel's
/// [`solve_sketch_min_change`] first moves the free points by the smallest step that
/// meets every constraint, and relaxes as [`solve_with`] does only when it cannot.
pub fn solve_drag_with(
    doc: &SketchDoc,
    settings: &SketchSolverSettings,
) -> Result<(SketchDoc, SketchDiagnostics), String> {
    solve_entry(doc, settings, true)
}

fn solve_entry(
    doc: &SketchDoc,
    settings: &SketchSolverSettings,
    min_change: bool,
) -> Result<(SketchDoc, SketchDiagnostics), String> {
    let sketch = serde_json::to_value(doc).map_err(|e| format!("sketch serialize: {e}"))?;
    let request = SolveSketchRequest {
        sketch,
        iterations: settings.iterations,
        remove_implied_duplicates: false,
        tolerance: settings.tolerance,
        distance_slide_threshold_ratio: settings.distance_slide_threshold_ratio,
        distance_slide_step_ratio: settings.distance_slide_step_ratio,
        distance_slide_min_step: settings.distance_slide_min_step,
        polish: None,
    };

    let response = if min_change {
        solve_sketch_min_change(&request)?
    } else {
        solve_sketch(&request)?
    };
    let solved = response
        .get("sketch")
        .cloned()
        .ok_or_else(|| "solve_sketch: response missing `sketch`".to_string())?;

    let diagnostics = match solved.get("diagnostics").cloned() {
        Some(value) => {
            serde_json::from_value(value).map_err(|e| format!("diagnostics parse: {e}"))?
        }
        None => SketchDiagnostics::default(),
    };
    let solved_doc: SketchDoc =
        serde_json::from_value(solved).map_err(|e| format!("solved sketch parse: {e}"))?;

    Ok((solved_doc, diagnostics))
}
