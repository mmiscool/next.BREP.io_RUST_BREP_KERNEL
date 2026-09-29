//! §7.3 3D assembly mate solver — the rigid-body sibling of `sketch_solver`.
//!
//! N rigid bodies, each with a pose (unit quaternion + translation); a set of
//! mates (coincident, concentric, distance — including point-line and
//! line-line closest approach —, angle, parallel, perpendicular,
//! tangent) whose geometry (points, directions, planes, axes) is given in each
//! body's LOCAL frame. Grounded (`fixed`) bodies are excluded from the
//! parameter vector; the solver finds poses for the movable bodies that
//! minimise the stacked mate residual vector in least squares.
//!
//! Pose parameterization: poses are STORED as unit quaternions `[w,x,y,z]`
//! plus translations, but the solver's unknowns are per-body LOCAL
//! increments `[δθx,δθy,δθz, δtx,δty,δtz]` (a rotation vector applied on the
//! left, `R ← exp(δθ)·R`, `t ← t + δt`) that are re-centred to zero after
//! every accepted step. This keeps the best of both worlds: quaternion
//! storage never hits the rotation-vector singularity at ‖θ‖ = π (the
//! increment stays near zero, so `exp` is always evaluated far from the
//! singularity), each movable body contributes exactly 6 Jacobian columns so
//! DOF counting is `6·movable − rank` with no unit-norm gauge row, and the
//! only bookkeeping cost is one quaternion renormalization per accepted step.
//!
//! Solver: Levenberg-Marquardt on the damped normal equations
//! `(JᵀJ + λI)Δ = −Jᵀr`, mirroring `sketch_solver::newton_polish` (same
//! central-difference Jacobian, same Cholesky solve, same λ up/down schedule,
//! same non-worsening best-max guard). Because `Jᵀr` lies in the row space of
//! `J`, the step has no component along free degrees of freedom: an
//! under-constrained assembly keeps its free DOF (e.g. a shaft's spin angle)
//! exactly where the initial guess put them.
//!
//! Diagnostics mirror `sketch_solver::compute_diagnostics`: numerical rank of
//! the Jacobian at the solution gives `dof = 6·movable − rank`; redundancy is
//! counted against each mate's EFFECTIVE row count (a 3-row direction-align
//! block only ever removes 2 DOF, a 3-row axis-lateral block 2), so a plain
//! plane-plane mate is not misreported as over-constrained. Conflicting mates
//! (residual stays above tolerance at the least-squares point) produce an
//! `Err` naming the worst mates with their residuals.
//!
//! Decomposition (§7.5–7.6): `SolveStrategy::Auto` (the default) and
//! `::Decomposed` split the solve into small sub-solves instead of one
//! monolithic LM over every movable body:
//!
//! - SEQUENTIAL PEELING (§7.6): starting from the grounded bodies, any body
//!   (or cluster) is solved on its own — a 6-DOF LM over just its mates to
//!   already-solved bodies — and marked solved when either ALL its mates
//!   reference solved bodies (free DOF keep the guess) or the solved-side
//!   mates alone FULLY constrain it (Jacobian rank 6 over its rigid DOF), in
//!   which case mates to later bodies are deferred and enforced when those
//!   bodies are solved. A chain of N bodies becomes N small solves.
//! - RIGID-CLUSTER DETECTION (§7.5, conservative solving): when peeling
//!   stalls, pairs of unsolved bodies whose mutual mates fully bind them
//!   (pairwise relative Jacobian rank 6) are merged via union-find; each
//!   resulting group is solved internally (first member pinned) and VERIFIED
//!   by the internal-mate Jacobian having rank exactly `6·(k−1)` at the
//!   internal solution. A verified cluster is contracted into ONE rigid node:
//!   its members thereafter move together (one 6-column block, rigid motion
//!   about the anchor member) when the cluster is placed by its external
//!   mates.
//! - MONOLITHIC FALLBACK: a remainder that can neither be peeled nor
//!   clustered (a genuinely coupled cycle of mates) is solved with one
//!   monolithic LM over exactly the remaining bodies; the step is reported as
//!   `"monolithic_fallback"`. If the decomposed result misses the global
//!   tolerance, the whole solve is redone monolithically from the original
//!   poses (`AssemblySolution::strategy == "monolithic_fallback"`), so the
//!   decomposed strategies are never worse than `Monolithic`.
//!
//! Every step is recorded in `AssemblySolution::steps` (bodies, method,
//! accepted LM iterations); `residual_rows_evaluated` totals the scalar
//! residual rows evaluated while solving (step trials + finite-difference
//! Jacobian columns), the work measure behind the decomposition performance
//! guard. Final diagnostics (rank/dof/redundancy, per-mate residuals) are
//! always computed globally at the final poses, so they are strategy
//! independent.
//!
//! Not here yet (later slices): limit/gear/cam mates, and cluster detection
//! for groups that are rigid only through ≥3-body interactions (no fully
//! binding pair) — those fall back to the monolithic path.

use crate::Vec3;
use serde::{Deserialize, Serialize};

#[path = "assembly_solver/types.rs"]
mod types;
#[path = "assembly_solver/atoms.rs"]
mod atoms;
#[path = "assembly_solver/mates.rs"]
mod mates;
#[path = "assembly_solver/numerics.rs"]
mod numerics;
#[path = "assembly_solver/decompose.rs"]
mod decompose;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------


use atoms::*;
use mates::*;
use numerics::*;
use types::*;

pub use decompose::solve_assembly;
pub use types::{
    AssemblyBody, AssemblyMate, AssemblySolution, AssemblySolveOptions, BodyPose, MateAlign,
    MateAxis, MateKind, MatePlane, MateResidualReport, SolveStepReport, SolveStrategy,
};
