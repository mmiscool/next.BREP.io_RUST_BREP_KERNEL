use super::*;

// ---------------------------------------------------------------------------
// Tuning constants (mirroring sketch_solver's polish constants)
// ---------------------------------------------------------------------------

/// Maximum damping increases per outer iteration before abandoning the step.
pub(super) const ASM_MAX_DAMPING: usize = 16;
/// Damping growth on a rejected step / Cholesky failure.
pub(super) const ASM_LAMBDA_UP: f64 = 8.0;
/// Damping shrink after an accepted step (drives λ toward pure Gauss-Newton).
pub(super) const ASM_LAMBDA_DOWN: f64 = 0.25;
/// An accepted step below this (relative to scale) counts as converged.
pub(super) const ASM_STEP_REL_TOL: f64 = 1e-13;
/// Central-difference step for the three rotation-vector columns (radians).
pub(super) const ASM_ROT_FD_STEP: f64 = 1e-6;
/// Central-difference step for translation columns, times the model scale.
pub(super) const ASM_TRANS_FD_STEP: f64 = 1e-6;
/// Relative pivot tolerance for the numerical Jacobian rank. Comfortably
/// above the central-difference noise floor (~1e-10 relative) yet far below
/// any genuine mate derivative.
pub(super) const ASM_RANK_REL_TOL: f64 = 1e-7;
/// Vectors shorter than this cannot be normalized into mate directions.
pub(super) const ASM_DIR_EPS: f64 = 1e-12;
/// Line-line distance blend band, in the SINE of the angle between the two
/// world directions. The skew closest-approach formula `|u·(da×db)|/‖da×db‖`
/// is 0/0 at exact parallelism, and near it the measure is both
/// ill-conditioned (its pose Jacobian grows like 1/sin) and genuinely
/// discontinuous (coplanar near-parallel lines intersect, so the skew measure
/// collapses to 0 regardless of the lateral gap). Below `PARALLEL` the mate
/// therefore measures the parallel distance (point-to-line of A's origin
/// about line B); above `SKEW` it is the true skew closest approach; between,
/// the SQUARED measures are combined by a C1 smoothstep in the sine, so LM's
/// central differences never straddle a derivative jump.
pub(super) const ASM_LL_SIN_PARALLEL: f64 = 1e-3;
pub(super) const ASM_LL_SIN_SKEW: f64 = 1e-2;

pub(super) fn default_tolerance() -> f64 {
    1e-10
}

fn default_max_iterations() -> usize {
    200
}

pub(super) fn identity_rotation() -> [f64; 4] {
    [1.0, 0.0, 0.0, 0.0]
}

// ---------------------------------------------------------------------------
// Public types (serde-serializable; WASM binding comes later, centrally)
// ---------------------------------------------------------------------------

/// One rigid body of the assembly with its initial (guess) pose.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssemblyBody {
    /// Display name used in reports and error messages.
    pub id: String,
    /// Grounded bodies are excluded from the unknowns and never move.
    #[serde(default)]
    pub fixed: bool,
    /// Unit quaternion `[w, x, y, z]` rotating body-local into world frame.
    /// Any nonzero quaternion is accepted and normalized on input.
    #[serde(default = "identity_rotation")]
    pub rotation: [f64; 4],
    /// World-frame translation of the body-local origin.
    #[serde(default)]
    pub translation: [f64; 3],
}

/// A plane in a body's local frame (origin + unit normal).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MatePlane {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
}

/// An axis in a body's local frame (origin + unit direction).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct MateAxis {
    pub origin: [f64; 3],
    pub direction: [f64; 3],
}

/// Direction-sense handling for direction-carrying mates. Mirrors the app's
/// "oppose normals" flag on parallel/coincident constraints.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MateAlign {
    /// World directions must point the same way (`da = db`).
    #[default]
    Aligned,
    /// World directions must oppose (`da = -db`); the face-to-face default.
    AntiAligned,
    /// Either sense is acceptable (cross-product residual).
    Any,
}

impl MateAlign {
    fn any() -> Self {
        Self::Any
    }
    fn anti() -> Self {
        Self::AntiAligned
    }
}

/// A curve a curve mate follows, in its body's LOCAL frame: an edge's carrier
/// over the edge's parameter range `[t0, t1]`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MateCurve {
    pub curve: crate::NurbsCurve,
    pub t0: f64,
    pub t1: f64,
}

/// Mate geometry + semantics. All points/directions/planes/axes are expressed
/// in the OWNING body's local frame (`*_a` on `body_a`, `*_b` on `body_b`).
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum MateKind {
    /// Two local points touch (3 DOF removed).
    CoincidentPointPoint {
        point_a: [f64; 3],
        point_b: [f64; 3],
    },
    /// A point of body A lies on a plane of body B (1 DOF removed).
    CoincidentPointPlane {
        point_a: [f64; 3],
        plane_b: MatePlane,
    },
    /// Two planes are coplanar with the given normal sense (3 DOF removed).
    CoincidentPlanePlane {
        plane_a: MatePlane,
        plane_b: MatePlane,
        #[serde(default = "MateAlign::anti")]
        align: MateAlign,
    },
    /// Two axes are collinear (4 DOF removed; spin + slide stay free).
    ConcentricAxisAxis {
        axis_a: MateAxis,
        axis_b: MateAxis,
        #[serde(default = "MateAlign::any")]
        align: MateAlign,
    },
    /// ‖pa − pb‖ equals `distance` (1 DOF removed). A zero distance degrades
    /// gracefully to a point-point coincidence.
    DistancePointPoint {
        point_a: [f64; 3],
        point_b: [f64; 3],
        distance: f64,
    },
    /// Signed height of the point above the plane (along its world normal)
    /// equals `distance` (1 DOF removed).
    DistancePointPlane {
        point_a: [f64; 3],
        plane_b: MatePlane,
        distance: f64,
    },
    /// Parallel planes at a signed offset: `distance` is measured from plane
    /// A to plane B's origin along plane A's world normal (3 DOF removed).
    DistancePlanePlane {
        plane_a: MatePlane,
        plane_b: MatePlane,
        distance: f64,
        #[serde(default = "MateAlign::anti")]
        align: MateAlign,
    },
    /// Distance from a point of body A to an INFINITE line of body B equals
    /// `distance` (1 DOF removed): residual `‖(p − o) ⊥ d‖ − distance`. A
    /// zero distance degrades gracefully to point-on-line coincidence (the
    /// `coincident_point_line` shortcut — the norm residual would be
    /// non-differentiable at zero).
    DistancePointLine {
        point_a: [f64; 3],
        axis_b: MateAxis,
        distance: f64,
    },
    /// Closest-approach distance between two INFINITE lines equals `distance`
    /// (1 DOF removed). Skew lines use `|u·(da×db)|/‖da×db‖`; nearly parallel
    /// lines (sine of the angle below `ASM_LL_SIN_PARALLEL`) are measured
    /// point-to-line instead — the skew formula degenerates there — with a C1
    /// smoothstep blend of the squared measures across the band up to
    /// `ASM_LL_SIN_SKEW`. A zero distance degrades to a signed touch
    /// constraint (intersecting when skew, collinear when parallel), so
    /// `parallel` + `distance_line_line(0)` is the `concentric` shortcut
    /// without the direction rows.
    DistanceLineLine {
        axis_a: MateAxis,
        axis_b: MateAxis,
        distance: f64,
    },
    /// Angle in degrees between two world directions (plane normals or axis
    /// directions); residual is `cos(measured) − cos(target)` (1 DOF removed).
    Angle {
        direction_a: [f64; 3],
        direction_b: [f64; 3],
        angle_deg: f64,
    },
    /// Signed angle from A to B about `axis_a` (in body A's local frame).
    /// Measures B's projection into the plane perpendicular to the axis.
    /// The wrapped angular residual removes one DOF and retains the rotation
    /// sense at negative/reflex targets and through parallel configurations.
    DirectedAngle {
        direction_a: [f64; 3],
        direction_b: [f64; 3],
        axis_a: [f64; 3],
        angle_deg: f64,
    },
    /// Directions parallel (2 DOF removed).
    Parallel {
        direction_a: [f64; 3],
        direction_b: [f64; 3],
        #[serde(default = "MateAlign::any")]
        align: MateAlign,
    },
    /// Directions perpendicular (1 DOF removed).
    Perpendicular {
        direction_a: [f64; 3],
        direction_b: [f64; 3],
    },
    /// Sphere (centre on body A, radius) tangent to a plane of body B, centre
    /// on the positive-normal side (1 DOF removed).
    TangentSpherePlane {
        center_a: [f64; 3],
        radius: f64,
        plane_b: MatePlane,
    },
    /// Cylinder (axis on body A, radius) tangent to a plane of body B, axis
    /// on the positive-normal side (2 DOF removed).
    TangentCylinderPlane {
        axis_a: MateAxis,
        radius: f64,
        plane_b: MatePlane,
    },
    /// A LIMIT mate: the quantity `mate` measures stays inside `[min, max]`
    /// instead of equalling its target (whose own value is ignored). `mate`
    /// is a distance kind (`min`/`max` in model units, signed exactly as that
    /// kind's `distance`) or an angle kind (degrees; `DirectedAngle` is
    /// signed, wraps, and the range runs from `min` counter-clockwise to
    /// `max`). The rows the kind holds besides its measure (the parallelism of
    /// `DistancePlanePlane`) still bind. Inside the range the limit row is
    /// zero and removes nothing; outside it is the signed distance to the
    /// nearer end.
    Limit {
        mate: Box<MateKind>,
        min: f64,
        max: f64,
    },
    /// Path: `point_a` lies on `curve_b` (3 rows, removes 2 DOF) — the closest
    /// point of the curve, found afresh from fixed samples at every evaluation
    /// (no state between evaluations). `along`, when set, also pins where on
    /// the curve: a percent of its parameter range from its start (from its
    /// end with `reverse`), wrapped on a closed curve (1 row more).
    PointOnCurve {
        point_a: [f64; 3],
        curve_b: MateCurve,
        #[serde(default)]
        along: Option<f64>,
        #[serde(default)]
        reverse: bool,
    },
    /// Slot: the axis `axis_a` stays parallel to `normal_b` (either sense)
    /// and passes through `curve_b`, the slot's centreline (3 + 3 rows,
    /// removes 4 DOF; the pin may slide along the curve, between its ends,
    /// spin, and slide along its own axis). `along` / `reverse` as for
    /// [`MateKind::PointOnCurve`], read where the axis crosses the curve.
    Slot {
        axis_a: MateAxis,
        curve_b: MateCurve,
        normal_b: [f64; 3],
        #[serde(default)]
        along: Option<f64>,
        #[serde(default)]
        reverse: bool,
    },
    /// Cam: a follower of radius `radius` centred at `center_a` (a roller
    /// when `axis_a` is set: its axis stays parallel to `normal_b`; else a
    /// ball or point, kept in the profile's plane) touches `profile_b`, a
    /// planar curve with plane normal `normal_b`: the follower's in-plane
    /// distance to the profile equals its radius, on the outside of a closed
    /// profile (the left of an open one, seen from the normal) unless
    /// `flip` (1 row + the plane rows).
    Cam {
        center_a: [f64; 3],
        #[serde(default)]
        axis_a: Option<[f64; 3]>,
        radius: f64,
        profile_b: MateCurve,
        normal_b: [f64; 3],
        #[serde(default)]
        flip: bool,
    },
    /// `element_a` (on body A) and `element_b` (on body B) are mirror images
    /// about `plane_c`, a plane of the mate's `body_c`: points mirror onto
    /// points (3 DOF), a plane onto a plane facing the mirrored way (3 DOF),
    /// an axis onto an axis in either sense (4 DOF). Both elements must be of
    /// one kind.
    Symmetric {
        element_a: SymmetricElement,
        element_b: SymmetricElement,
        plane_c: MatePlane,
    },
    /// Gear: A's spin about `axis_a` and B's spin about `axis_b` are coupled,
    /// `ratio·θa + θb = 0` (external mesh, opposite senses; `reverse` makes it
    /// `ratio·θa − θb = 0`). A spin is the signed angle, about the body's own
    /// axis, from the WORLD reference `world_ref_*` to the body's local
    /// `ref_*` (a direction ⟂ its axis). `ratio` is teeth(A)/teeth(B) — B's
    /// turns per turn of A. [`MateKind::rebase`] captures the references so the
    /// mate holds the poses it is given (1 DOF removed).
    Gear {
        axis_a: MateAxis,
        ref_a: [f64; 3],
        world_ref_a: [f64; 3],
        axis_b: MateAxis,
        ref_b: [f64; 3],
        world_ref_b: [f64; 3],
        ratio: f64,
        #[serde(default)]
        reverse: bool,
    },
    /// Rack and pinion: pinion A's spin about `pinion_axis` (measured as for
    /// `Gear`) drives rack B along its local `rack_direction`: the travel of
    /// `rack_point` from `world_rack_point` along that direction equals
    /// `distance_per_revolution · θ / 2π` (`reverse` flips the sense) (1 DOF).
    RackPinion {
        pinion_axis: MateAxis,
        pinion_ref: [f64; 3],
        world_pinion_ref: [f64; 3],
        rack_point: [f64; 3],
        rack_direction: [f64; 3],
        world_rack_point: [f64; 3],
        distance_per_revolution: f64,
        #[serde(default)]
        reverse: bool,
    },
    /// Screw: B advances along A's `axis_a` by `pitch` per turn it makes
    /// about it, relative to A: `(s − offset0) = ±pitch·(θ − angle0)/2π`,
    /// where `θ` is the signed angle about the axis from A's `ref_a` to B's
    /// `ref_b` and `s` the height of B's `point_b` along the axis (1 DOF).
    Screw {
        axis_a: MateAxis,
        ref_a: [f64; 3],
        ref_b: [f64; 3],
        point_b: [f64; 3],
        pitch: f64,
        #[serde(default)]
        reverse: bool,
        angle0: f64,
        offset0: f64,
    },
}

/// One side of a [`MateKind::Symmetric`] mate, in its owning body's frame.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum SymmetricElement {
    Point { point: [f64; 3] },
    Plane { plane: MatePlane },
    Axis { axis: MateAxis },
}

/// One mate between two distinct bodies of the assembly.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssemblyMate {
    /// Optional display name; defaults to `mate[i]` in reports.
    #[serde(default)]
    pub id: Option<String>,
    /// Index into the `bodies` slice owning the `*_a` geometry.
    pub body_a: usize,
    /// Index into the `bodies` slice owning the `*_b` geometry.
    pub body_b: usize,
    /// Index of a THIRD body owning `*_c` geometry (the mirror plane of a
    /// symmetric mate); `None` for every two-body mate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body_c: Option<usize>,
    #[serde(flatten)]
    pub kind: MateKind,
}

/// Solving strategy (§7.5–7.6). `Auto` decomposes when possible and falls
/// back to the monolithic solve when it must, so it is always safe.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SolveStrategy {
    /// Decompose when possible (sequential peeling + rigid clusters), fall
    /// back to monolithic when required. The default.
    #[default]
    Auto,
    /// One Levenberg-Marquardt solve over all movable bodies (§7.3 v1).
    Monolithic,
    /// Force the decomposition pipeline. Genuinely coupled cycles still get a
    /// per-component monolithic fallback, and a decomposed result that misses
    /// the global tolerance is redone monolithically (reported as such).
    Decomposed,
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AssemblySolveOptions {
    /// Convergence target for the maximum residual, relative to the model
    /// scale (lengths in model units, direction residuals dimensionless).
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    /// Maximum accepted Levenberg-Marquardt steps (per sub-solve when
    /// decomposed).
    #[serde(default = "default_max_iterations")]
    pub max_iterations: usize,
    /// §7.5–7.6 strategy selection; omitted in JSON means `Auto`.
    #[serde(default)]
    pub strategy: SolveStrategy,
    /// How each LM step's normal equations are built and solved. `Sparse`
    /// (the default) is block-sparse over the bodies; `Dense` is the original
    /// dense path, kept for one release as a fallback and an A/B reference.
    #[serde(default)]
    pub linear_algebra: LinearAlgebra,
    /// Compute the DOF diagnostics (Jacobian rank, `dof`, `redundant`,
    /// `status`). A caller that only needs poses (the live drag) turns it
    /// off; the report then carries `status: "unchecked"` and zero counts.
    #[serde(default = "default_true")]
    pub diagnostics: bool,
}

fn default_true() -> bool {
    true
}

/// The LM step's linear algebra (see [`AssemblySolveOptions::linear_algebra`]).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LinearAlgebra {
    #[default]
    Sparse,
    Dense,
}

impl Default for AssemblySolveOptions {
    fn default() -> Self {
        Self {
            tolerance: default_tolerance(),
            max_iterations: default_max_iterations(),
            strategy: SolveStrategy::default(),
            linear_algebra: LinearAlgebra::default(),
            diagnostics: true,
        }
    }
}

/// Solved pose of one body (same order as the input `bodies` slice).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyPose {
    pub id: String,
    /// Unit quaternion `[w, x, y, z]`, local → world.
    pub rotation: [f64; 4],
    pub translation: [f64; 3],
}

/// Worst absolute residual row of one mate at the solution.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MateResidualReport {
    pub mate: String,
    pub residual: f64,
}

/// One sub-solve of the decomposition pipeline (or the single monolithic
/// solve), in execution order.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SolveStepReport {
    /// Ids of the bodies moved by this step.
    pub bodies: Vec<String>,
    /// `"monolithic"` | `"sequential"` | `"cluster_internal"` |
    /// `"cluster_sequential"` | `"monolithic_fallback"` | `"unconstrained"`.
    pub method: String,
    /// Accepted LM iterations spent in this step.
    pub iterations: usize,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AssemblySolution {
    pub poses: Vec<BodyPose>,
    /// Per-mate worst residual, in input mate order.
    pub mate_residuals: Vec<MateResidualReport>,
    pub max_residual: f64,
    /// Accepted Levenberg-Marquardt steps (summed over sub-solves).
    pub iterations: usize,
    /// Strategy actually used: `"monolithic"`, `"decomposed"`, or
    /// `"monolithic_fallback"` (decomposition attempted, full monolithic
    /// rerun took over).
    pub strategy: String,
    /// Per-step diagnostics in execution order: which bodies/clusters were
    /// solved by which method (§7.5–7.6).
    pub steps: Vec<SolveStepReport>,
    /// Total scalar residual rows evaluated while solving (step trials plus
    /// finite-difference Jacobian columns); the work measure behind the
    /// decomposition performance guard.
    pub residual_rows_evaluated: usize,
    /// Remaining degrees of freedom: `6·movable − rank`.
    pub dof: usize,
    /// Numerical rank of the mate Jacobian at the solution.
    pub rank: usize,
    /// Redundant (but consistent) constraint rows: effective rows − rank.
    pub redundant: usize,
    /// `"well"`, `"under"`, or `"over"` (over = redundant rows exist; the
    /// solution is still exact unless the solve returned `Err`).
    pub status: String,
    /// Per movable body, the freedom the mates leave it at the solution (the
    /// Jacobian's null space restricted to that body's six columns). Empty
    /// when diagnostics were off or the assembly is too large to analyse.
    #[serde(default)]
    pub body_freedom: Vec<BodyFreedom>,
}

/// How free one body is left: free turns (rotations about its own origin)
/// and the remaining free slides (pure translations). Both zero = fully
/// defined.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyFreedom {
    pub id: String,
    pub rotations: usize,
    pub translations: usize,
    /// One axis line per free rotation (world), orthonormal directions.
    #[serde(default)]
    pub axes: Vec<FreeAxis>,
    /// One unit direction per free slide (world), orthonormal.
    #[serde(default)]
    pub slides: Vec<[f64; 3]>,
}

/// A line a body can still turn about: a point on it and its unit direction.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FreeAxis {
    pub point: [f64; 3],
    pub direction: [f64; 3],
}
