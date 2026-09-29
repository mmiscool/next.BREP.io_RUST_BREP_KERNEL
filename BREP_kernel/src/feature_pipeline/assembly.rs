//! Assembly constraint state, lifecycle, and solver mapping.
//!
//! # The `assembly` block
//!
//! Constraints are kernel history state: a top-level `assembly` block on the
//! history request — `{ constraints: [ { type, inputParams, persistentData,
//! enabled, open } ], idCounter }` — that round-trips through the saved
//! document. The pre-purge envelope keys (`assemblyConstraints` /
//! `assemblyConstraintIdCounter`) are NOT read (no-backwards-compat).
//!
//! # Lifecycle, run at the tail of every history execution
//!
//! 1. VALIDATE ([`lifecycle`]): disabled → `status:'disabled'`; unknown type →
//!    an error entry without aborting the rest; duplicate detection across the
//!    overlapping family {touch_align, distance, angle, concentric,
//!    perpendicular, tangent} via order-independent selection-pair signatures.
//! 2. MAP ([`mapping`]): each element ref resolves through
//!    `solvers/assembly_resolve` to an analytic frame in the owning component's
//!    LOCAL space. Unresolvable/unsupported selections fail that constraint
//!    only (status from [`crate::ResolveError::status`]); the solve continues.
//! 3. SOLVE: ONE [`crate::solve_assembly`] call over every mapped constraint;
//!    per-mate residuals/statuses + solver diagnostics merge into each
//!    constraint's `persistentData`; distance/angle first-solve initialization
//!    commits into `inputParams` only on success.
//! 4. WRITE BACK: solved poses re-pose the scene components in place
//!    ([`crate::feature_pipeline::component`]) and are exported as generic
//!    `inputParams.transform` JSON updates for the owning ACOMP features (the
//!    pose-authority contract; applied by [`assembly_apply_document_json`] with
//!    no compile-time dependency on the ACOMP feature).
//!
//! # Pose self-consistency (why an unapplied write-back cannot corrupt)
//!
//! Mate-local frames are computed as `record.transform⁻¹ · world_frame`, and
//! each body's solver pose IS `record.transform` — so even when the record's
//! absolute pose is stale (the app has not yet folded a pose update back into
//! the ACOMP feature), the solve is self-consistent: reconstructed world frames
//! equal the actual world geometry, satisfied constraints produce no motion,
//! and a re-executed component simply re-solves to the same poses. The
//! write-back matters for PERSISTENCE (save/load) and cache fingerprints, not
//! for per-run geometric correctness.

use serde::{Deserialize, Serialize};

use crate::feature_pipeline::Env;

pub(crate) mod exports;
pub(crate) mod infer;
pub(crate) mod lifecycle;
pub(crate) mod mapping;
pub(crate) mod constraints;

pub use exports::{
    assembly_add_constraint_json, assembly_apply_document_json,
    assembly_apply_inferred_constraints_json, assembly_dof_json,
    assembly_infer_constraints_json, assembly_inferable_types_json,
    assembly_move_constraint_json, assembly_overlay_json, assembly_pose_updates_json,
    assembly_remove_constraint_json, assembly_run_solve_json,
    assembly_set_constraint_enabled_json, assembly_set_constraint_open_json,
    assembly_state_json, assembly_statuses_json, assembly_update_constraint_json,
    // The native doors: the same mutations with their errors as text. A native
    // caller must use these — a `JsValue` error aborts a native build.
    assembly_add_constraint_impl, assembly_apply_document_impl, assembly_move_constraint_impl,
    assembly_remove_constraint_impl, assembly_run_solve_impl,
    assembly_set_constraint_enabled_impl, assembly_set_constraint_open_impl,
    assembly_update_constraint_impl,
};
pub use constraints::{
    constraint_schema_catalogue, constraint_type, ConstraintTypeDef, CONSTRAINT_TYPES,
};
// The pose write-back encoder (`mapping::transform_to_pose_params`): the ONE
// matrix → `{translate, rotateEulerDeg}` (intrinsic XYZ, degrees) conversion,
// re-exported so out-of-crate ACOMP authors reuse it instead of re-deriving
// the Euler decomposition (and its gimbal-lock branch).
pub use mapping::transform_to_pose_params;

// ===========================================================================
// ===========================================================================

/// The document's `assembly` block: the ordered constraint list plus the
/// persistent id counter (monotonic, never reused — mirrors the feature
/// counter's contract).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct AssemblyState {
    #[serde(default)]
    pub constraints: Vec<ConstraintEntry>,
    #[serde(default, rename = "idCounter")]
    pub id_counter: u64,
}

/// One persisted constraint: `{ type, inputParams, persistentData, enabled,
/// open }`. `inputParams` carries `id`, `elements` (selection ref strings) and
/// the type-specific params; `persistentData` is the solver-state merge target
/// (status/message/diagnostics/orientation cache).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ConstraintEntry {
    #[serde(rename = "type")]
    pub constraint_type: String,
    #[serde(rename = "inputParams", default = "empty_object")]
    pub input_params: serde_json::Value,
    #[serde(rename = "persistentData", default = "empty_object")]
    pub persistent_data: serde_json::Value,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub open: bool,
}

fn empty_object() -> serde_json::Value {
    serde_json::Value::Object(serde_json::Map::new())
}

fn default_true() -> bool {
    true
}

impl ConstraintEntry {
    /// The constraint id (`inputParams.id`), empty when unset.
    pub fn id(&self) -> &str {
        self.input_params
            .get("id")
            .and_then(|value| value.as_str())
            .unwrap_or("")
    }

    /// The selection refs (`inputParams.elements`), string entries only.
    pub fn elements(&self) -> Vec<String> {
        self.input_params
            .get("elements")
            .and_then(|value| value.as_array())
            .map(|items| {
                items
                    .iter()
                    .filter_map(|item| item.as_str())
                    .map(str::to_string)
                    .collect()
            })
            .unwrap_or_default()
    }

    /// A boolean param (absent → false).
    pub fn flag(&self, key: &str) -> bool {
        self.input_params
            .get(key)
            .and_then(|value| value.as_bool())
            .unwrap_or(false)
    }

    /// A numeric param, expression-capable (a string evaluates against the
    /// part's expression scope). Absent → `default`.
    pub fn number(&self, key: &str, env: &Env, default: f64) -> Result<f64, String> {
        match self.input_params.get(key) {
            None | Some(serde_json::Value::Null) => Ok(default),
            Some(serde_json::Value::Number(number)) => number
                .as_f64()
                .ok_or_else(|| format!("param `{key}` is not a finite number")),
            Some(serde_json::Value::String(source)) => env
                .eval(source)
                .map_err(|error| format!("param `{key}`: {error}")),
            Some(other) => Err(format!(
                "param `{key}` must be a number or expression string, found {other}"
            )),
        }
    }

    /// Merge a key into `persistentData` (created as an object when missing).
    pub fn set_persistent(&mut self, key: &str, value: serde_json::Value) {
        if !self.persistent_data.is_object() {
            self.persistent_data = empty_object();
        }
        if let Some(map) = self.persistent_data.as_object_mut() {
            map.insert(key.to_string(), value);
        }
    }

    /// Remove a key from `persistentData` (no-op when absent).
    pub fn remove_persistent(&mut self, key: &str) {
        if let Some(map) = self.persistent_data.as_object_mut() {
            map.remove(key);
        }
    }

    /// A `persistentData` field, if present.
    pub fn persistent(&self, key: &str) -> Option<&serde_json::Value> {
        self.persistent_data.get(key)
    }

    /// Set the run status vocabulary trio (`status`, `message`, `satisfied`)
    /// and clear stale duplicate bookkeeping unless this IS a duplicate mark.
    pub fn set_status(&mut self, status: &str, message: impl Into<String>, satisfied: bool) {
        self.set_persistent("status", serde_json::Value::String(status.to_string()));
        self.set_persistent("message", serde_json::Value::String(message.into()));
        self.set_persistent("satisfied", serde_json::Value::Bool(satisfied));
        if status != "duplicate" {
            self.remove_persistent("duplicateConstraintIDs");
            self.remove_persistent("duplicateSignature");
        }
        if status != "satisfied" && status != "adjusted" {
            self.remove_persistent("error");
            self.remove_persistent("kernel");
        }
    }

    /// The current status word (empty when the constraint never ran).
    pub fn status(&self) -> &str {
        self.persistent("status")
            .and_then(|value| value.as_str())
            .unwrap_or("")
    }
}

// ===========================================================================
// History-tail hook
// ===========================================================================

/// Run the request's assembly constraints against the just-built scene and
/// install the post-solve session the exported ABI serves (`exports`). Called
/// unconditionally at the tail of [`crate::feature_pipeline::execute_history`]
/// — a request without an `assembly` block installs an EMPTY session, so a
/// document switch can never serve stale constraint state.
pub(crate) fn finish_history_run(
    request: &crate::feature_pipeline::HistoryRequest,
    scene: &mut crate::feature_pipeline::SceneMap,
    env: &Env,
) {
    let mut state = request.assembly.clone().unwrap_or_default();
    let outcome = if state.constraints.is_empty() {
        lifecycle::LifecycleOutcome::empty()
    } else {
        lifecycle::run_constraints(&mut state, scene, env)
    };
    exports::install_session(
        state,
        scene.clone(),
        request.expressions.clone(),
        request.configurator.clone(),
        outcome,
    );
}

