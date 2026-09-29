//! 2D sketch constraint solver.
//!
//! Port of the iterative geometric constraint engine that previously lived in
//! the retiring app's `sketchSolver2D` (ConstraintEngine +
//! constraintDefinitions). The numerical behaviour is kept faithful to the
//! original implementation: same constraint residual handling, same pass
//! ordering, same rounding (6 decimals per tidy pass), same distance-target
//! slide/throttle semantics, and the same convergence check based on point
//! state signatures. The rounding is the RELAXATION's: a committed (polished)
//! solve starts its polish from the caller's own digits wherever relaxation
//! left a coordinate on the grid, so a sketch that already satisfies its
//! constraints comes back as it was stored rather than snapped.
//!
//! Constraints round-trip as JSON objects so that solver bookkeeping fields
//! (`status`, `error`, `previousPointValues`, `_previousSolveValue`,
//! `_distanceRequestedTarget`, `_distanceAppliedTarget`,
//! `_distanceThrottleActive`, `_distanceLastAppliedPassToken`,
//! `_linePointDistanceSign`) persist between solves exactly like the previous engine
//! which mutated live objects. Internally the engine parses each constraint
//! into a typed structure once per solve and compares point-state signatures
//! as normalized f64 bit patterns; signature strings are only materialized
//! when they must round-trip through JSON (`previousPointValues`). String
//! equality of the original signatures is equivalent to bit equality here because both
//! use shortest round-trip float formatting with -0 and NaN normalized.
//!
//! # Module map
//!
//! The solver is split into topic submodules (declared with explicit
//! `#[path]` because this root file is itself declared via `#[path]` in
//! `lib.rs`); this root file keeps the shared data model:
//!
//! - [`jsnum`] — JavaScript numeric semantics helpers (rounding, parseFloat/Number
//!   coercions, signature bit patterns, JSON number formatting).
//! - [`hot_constraint`] — `impl HotConstraint`: JSON parse/write-back of one
//!   constraint record and its solver bookkeeping fields.
//! - [`engine`] — `Engine` construction from sketch JSON, point/signature
//!   helpers, constraint dispatch and the iterative relaxation solve loop.
//! - [`constraints_basic`] — the original ported constraint functions
//!   (horizontal/vertical, distance, point-line distance, equal distance,
//!   parallel, perpendicular, angle, coincident, point-on-line, midpoint).
//! - [`constraints_shape`] — point-based shape constraints added in Rust
//!   (tangent incl. splines, spline curvature continuity, concentric,
//!   equal-radius, collinear, symmetric) plus their line/spline/circle
//!   geometry helpers.
//! - [`constraints_foot`] — foot-parameter tangency (`∿`): the stationarity
//!   solve that eliminates the touch parameter, its per-span roots, the
//!   residuals and the relaxation nudges.
//! - [`dof_polish`] — DOF diagnostics (residual atoms, finite-difference
//!   Jacobian, rank/null-space) and the Levenberg-Marquardt exactness polish.
//! - [`api`] — implied-duplicate constraint removal and the public
//!   `solve_sketch` / `solve_sketch_from_json` entry points.
//! - `tests` — the full test suite.

use rustc_hash::FxHashMap as HashMap;
use serde::Deserialize;
use serde_json::{Map, Value};
use std::sync::atomic::{AtomicU64, Ordering};

/// Mirrors `globalDistanceSolveCycleId` from ConstraintEngine.
static GLOBAL_DISTANCE_SOLVE_CYCLE_ID: AtomicU64 = AtomicU64::new(0);

const POINT_LINE_DISTANCE_TYPE: &str = "↥";

/// Solver tuning knobs. Defaults mirror constraintDefinitions.
#[derive(Clone, Copy, Debug)]
pub struct SketchSolverSettings {
    pub tolerance: f64,
    pub distance_slide_threshold_ratio: f64,
    pub distance_slide_step_ratio: f64,
    pub distance_slide_min_step: f64,
    /// Run the global Levenberg-Marquardt least-squares polish after the
    /// iterative relaxation solve so constrained geometry converges to
    /// machine precision instead of the ~1e-5 relaxation floor. Defaults on;
    /// callers can disable it (e.g. per interactive drag frame) via the
    /// request's `polish` field.
    pub newton_polish: bool,
}

impl Default for SketchSolverSettings {
    fn default() -> Self {
        Self {
            tolerance: 0.00001,
            distance_slide_threshold_ratio: 0.10,
            distance_slide_step_ratio: 0.10,
            distance_slide_min_step: 0.001,
            newton_polish: true,
        }
    }
}

/// The reserved id of every sketch's PART ORIGIN reference point: world
/// `(0, 0, 0)` projected onto the sketch plane. The editor adds it on entry as
/// a fixed, grounded, construction external reference, so geometry can be
/// constrained to the origin. The sketch feature re-projects it onto the
/// resolved frame before solving and never publishes it — it exists only
/// inside the sketch (no scene point, no hole center, no 3D display).
pub const SKETCH_ORIGIN_POINT_ID: &str = "origin";

/// Whether a sketch point id is the reserved [`SKETCH_ORIGIN_POINT_ID`].
pub fn is_sketch_origin_point_id(id: &Value) -> bool {
    id.as_str() == Some(SKETCH_ORIGIN_POINT_ID)
}

#[derive(Debug, Deserialize)]
pub struct SolveSketchRequest {
    pub sketch: Value,
    #[serde(default)]
    pub iterations: Option<u32>,
    #[serde(default)]
    pub remove_implied_duplicates: bool,
    #[serde(default)]
    pub tolerance: Option<f64>,
    #[serde(default)]
    pub distance_slide_threshold_ratio: Option<f64>,
    #[serde(default)]
    pub distance_slide_step_ratio: Option<f64>,
    #[serde(default)]
    pub distance_slide_min_step: Option<f64>,
    /// Enable/disable the post-relaxation Levenberg-Marquardt polish. Absent
    /// (`None`) keeps it enabled; interactive drag frames may pass `false` to
    /// keep only the cheap relaxation solve. Committed solves keep it on so the
    /// emitted geometry is exact.
    #[serde(default)]
    pub polish: Option<bool>,
}

// ---------------------------------------------------------------------------
// Point identity keys (JavaScript Map SameValueZero semantics)
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Eq, Hash)]
enum PointKey {
    Num(u64),
    Str(String),
    Bool(bool),
    Null,
}

fn point_key(value: &Value) -> PointKey {
    match value {
        Value::Number(number) => match number.as_f64() {
            Some(float) => PointKey::Num(sig_bits(float)),
            None => PointKey::Null,
        },
        Value::String(text) => PointKey::Str(text.clone()),
        Value::Bool(flag) => PointKey::Bool(*flag),
        _ => PointKey::Null,
    }
}

// ---------------------------------------------------------------------------
// Typed constraint model
// ---------------------------------------------------------------------------

// Symbols for the constraint types added on top of the original engine.
const TANGENT_TYPE: &str = "⌒";
const CONCENTRIC_TYPE: &str = "◎";
const EQUAL_RADIUS_TYPE: &str = "⊜";
const COLLINEAR_TYPE: &str = "⋰";
const SYMMETRIC_TYPE: &str = "⋈";
/// Curvature (G2) continuity. The plan's WORKING glyph — the final glyph is
/// still an open question there, and it is the stored constraint `type`, so
/// changing it later changes saved documents.
const SPLINE_CURVATURE_TYPE: &str = "ϰ";
/// Tangency at a point INTERIOR TO A SPAN — the foot-parameter lane. A separate
/// type string because `⌒` with a spline pair already MEANS anchor tangency and
/// this constraint's point list names a spline BODY, not an (anchor, handle)
/// pair. The plan's working glyph, an open question the same way `ϰ`'s is.
const SPLINE_FOOT_TANGENT_TYPE: &str = "∿";

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
enum CType {
    Horizontal,
    Vertical,
    Distance,
    PointLine,
    EqualDistance,
    Parallel,
    Perpendicular,
    Angle,
    Coincident,
    PointOnLine,
    Midpoint,
    Ground,
    // New point-based constraints (all CAD math lives here in Rust).
    Tangent,
    /// Curvature (G2) continuity at a spline joint, or between a spline end and
    /// a circle/arc. Emitted rows always INCLUDE the tangency (G1) the join
    /// needs, so "curvature continuous" implies "tangent" the way every
    /// mainstream CAD UX reads it.
    SplineCurvature,
    /// Tangency between a spline and a line or circle/arc at a point INTERIOR
    /// to one of the spline's spans. The touch parameter is not a solver
    /// unknown: it is eliminated by projection (the foot of the stationarity
    /// condition) and the residual is evaluated there, so the constraint is one
    /// row like every other tangency.
    SplineFootTangent,
    Concentric,
    EqualRadius,
    Collinear,
    Symmetric,
    #[default]
    Other,
}

fn ctype_from(type_str: &str) -> CType {
    match type_str {
        "━" => CType::Horizontal,
        "│" => CType::Vertical,
        "⟺" => CType::Distance,
        POINT_LINE_DISTANCE_TYPE => CType::PointLine,
        "⇌" => CType::EqualDistance,
        "∥" => CType::Parallel,
        "⟂" => CType::Perpendicular,
        "∠" => CType::Angle,
        "≡" => CType::Coincident,
        "⏛" => CType::PointOnLine,
        "⋯" => CType::Midpoint,
        "⏚" => CType::Ground,
        TANGENT_TYPE => CType::Tangent,
        SPLINE_CURVATURE_TYPE => CType::SplineCurvature,
        SPLINE_FOOT_TANGENT_TYPE => CType::SplineFootTangent,
        CONCENTRIC_TYPE => CType::Concentric,
        EQUAL_RADIUS_TYPE => CType::EqualRadius,
        COLLINEAR_TYPE => CType::Collinear,
        SYMMETRIC_TYPE => CType::Symmetric,
        _ => CType::Other,
    }
}

fn is_distance_ctype(ctype: CType) -> bool {
    matches!(ctype, CType::Distance | CType::PointLine)
}

/// Spline geometry kind. The app's sketcher stores a spline as geometry
/// `type: "bezier"`: a chained cubic Bézier CONTROL POLYGON (verified against
/// SketchMode3D — the sampler evaluates cubic Bernstein polynomials over
/// `points[3k..3k+3]`), NOT an interpolating through-point spline. `points`
/// holds `3*segCount + 1` point ids; on-curve anchors sit at indices 0, 3, 6,
/// …, and the ids between them are off-curve handles. Every one of those
/// points is an ordinary sketch point, so it participates in constraints, the
/// relaxation solve, the Newton polish and the DOF diagnostics exactly like a
/// free point. `"spline"` is accepted as an alias so a future app-side rename
/// keeps working.
fn is_spline_geometry_type(type_str: &str) -> bool {
    type_str == "bezier" || type_str == "spline"
}

/// One slot of a point-state signature (None = missing point → "null;").
type SigEntry = Option<(u64, u64, bool)>;

/// `_previousSolveValue` as seen through JavaScript strict-equality semantics.
#[derive(Clone, Copy, Debug, Default)]
enum PrevSolve {
    #[default]
    Missing,
    /// Numeric stored value; JSON null maps to NaN (a NaN that
    /// round-tripped through JSON as null).
    Num(f64),
    /// Any other stored type: strict equality with a number is always false.
    OtherType,
}

#[derive(Clone, Debug, Default)]
enum PrevPoints {
    /// Field absent in the input (JavaScript `undefined`).
    #[default]
    Missing,
    /// Field present but not a string (can never match a signature).
    PresentNonString,
    /// Signature string from the input JSON.
    Str(String),
    /// Signature written during this solve: bits for fast comparison plus the
    /// materialized string for JSON output.
    Bits(Vec<SigEntry>, String),
}

#[derive(Debug, Default)]
struct HotConstraint {
    /// Original JSON object; hot fields are written back on output.
    raw: Map<String, Value>,
    ctype: CType,
    /// Raw `type` string (None when missing or not a string).
    type_str: Option<String>,
    /// Raw point id values (mutated only by the ∠ negative-value swap).
    point_ids: Vec<Value>,
    /// Resolved indices into the engine point list (parallel to point_ids).
    point_idx: Vec<Option<usize>>,
    points_written: bool,
    temporary: bool,

    // --- value ---
    /// Overridden numeric value (set when the engine writes `value`).
    value_num: Option<f64>,
    /// parseFloat cache of the effective value.
    value_parsed: f64,
    /// Whether the effective value is null/undefined (∠ seeding check).
    value_nullish: bool,
    /// Number-coercion cache of the effective value (⇌ lookups).
    value_cv: f64,
    value_written: bool,

    // --- status / error ---
    status_solved: bool,
    status_written: bool,
    error: Option<Value>,
    error_written: bool,

    // --- solve bookkeeping ---
    prev_solve: PrevSolve,
    prev_solve_written: bool,
    prev_points: PrevPoints,
    prev_points_written: bool,

    // --- distance throttle state ---
    /// Number.isFinite view of `_distanceRequestedTarget`.
    req_target: Option<f64>,
    req_target_written: bool,
    /// Number.isFinite view of `_distanceAppliedTarget`.
    app_target: Option<f64>,
    app_target_written: bool,
    /// Strict `=== true` view of `_distanceThrottleActive`.
    throttle_true: bool,
    /// Truthiness view of `_distanceThrottleActive` (differs from strict for
    /// odd input values until first write).
    throttle_truthy: bool,
    throttle_written: bool,
    /// String view of `_distanceLastAppliedPassToken`.
    pass_token: Option<String>,
    pass_token_written: bool,
    /// Number() view of `_linePointDistanceSign` (missing → NaN, null → 0).
    line_sign: f64,
    line_sign_written: bool,
    /// `_splineFootT`: the foot-parameter tangency's touch point as
    /// `span + t` in the spline's own chain parameter. The SEED the next solve's
    /// foot solve starts from and the span index it freezes, so the touch point
    /// is stable across solves and across the relaxation-only frames an
    /// interactive drag runs. `None` when the field is absent or not finite —
    /// which is the FIRST solve of a freshly applied constraint, and is why the
    /// foot solve has a no-seed path (a coarse scan) at all.
    foot_t: Option<f64>,
    foot_t_written: bool,
}

enum Filter<'a> {
    All,
    Type(CType, &'a str),
}

// ---------------------------------------------------------------------------
// Engine
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
struct EnginePoint {
    id: Value,
    x: f64,
    y: f64,
    fixed: bool,
    construction: bool,
    external_reference: bool,
}

type CResult = Result<Value, String>;

struct Engine {
    points: Vec<EnginePoint>,
    /// `point_key` → index into `points`. Built once in [`Engine::new`] and valid
    /// for the whole solve: a solve moves points and flips their `fixed` flag but
    /// never adds, removes or renames one. Constraint point lists are resolved
    /// through it at parse time; the geometry lookups that resolve a control
    /// point's id DURING the solve (a curvature side's second control, a spline
    /// body's whole polygon) go through it too, because a linear scan per control
    /// per pass is quadratic in the sketch.
    point_index: HashMap<PointKey, usize>,
    geometries: Vec<Value>,
    constraints: Vec<HotConstraint>,
    settings: SketchSolverSettings,
    pass_token: String,
    sig_scratch_before: Vec<SigEntry>,
    sig_scratch_after: Vec<SigEntry>,
}

#[path = "sketch_solver/jsnum.rs"]
mod jsnum;
pub use jsnum::fmt_id as sketch_id_key;
#[path = "sketch_solver/hot_constraint.rs"]
mod hot_constraint;
#[path = "sketch_solver/engine.rs"]
mod engine;
#[path = "sketch_solver/constraints_basic.rs"]
mod constraints_basic;
#[path = "sketch_solver/constraints_shape.rs"]
mod constraints_shape;
#[path = "sketch_solver/constraints_foot.rs"]
mod constraints_foot;
#[path = "sketch_solver/dof_polish.rs"]
mod dof_polish;
#[path = "sketch_solver/api.rs"]
mod api;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------


use constraints_foot::{
    foot_in_span, solve_spline_foot, FootTarget, SpanControls, FOOT_EVAL_MARGIN,
};
use constraints_shape::{
    circle_curvature_residual, circle_curvature_sign, joint_curvature_residual,
    spline_end_curvature,
};
use jsnum::*;

pub use api::{solve_sketch, solve_sketch_from_json};
