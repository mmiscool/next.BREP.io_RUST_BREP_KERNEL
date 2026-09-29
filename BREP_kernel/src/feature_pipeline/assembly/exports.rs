//! The assembly SESSION + exported ABI: constraint CRUD with auto-solve,
//! manual solve, per-constraint status JSON, DOF summary JSON,
//! overlay-geometry JSON, and the generic document fold that writes the solved
//! state + poses back into the history document.
//!
//! The session is installed by the history-execution tail
//! ([`super::finish_history_run`]) after EVERY run: the post-solve constraint
//! state, a clone of the final scene map (names + resident handles — the
//! handles stay owned by the history cache), and the expression sheet for
//! expression-capable params. Constraint mutations between runs operate on the
//! session and AUTO-SOLVE against the live resident geometry; the app folds
//! the result back into its document via [`assembly_apply_document_json`]
//! before the next history run (the pose-authority contract).

use std::cell::RefCell;
use std::collections::BTreeMap;

use wasm_bindgen::prelude::*;

use super::lifecycle::{self, LifecycleOutcome};
use super::{constraints, mapping, AssemblyState, ConstraintEntry};
use crate::feature_pipeline::{Env, SceneMap};

struct Session {
    state: AssemblyState,
    scene: SceneMap,
    expressions: String,
    configurator: serde_json::Value,
    /// Component id → solved `inputParams.transform` JSON. ACCUMULATES across
    /// mutation solves (later solves overwrite per id); replaced wholesale by
    /// the next history run.
    pose_updates: BTreeMap<String, serde_json::Value>,
    /// Component id → grounded write-back (`inputParams.isFixed`).
    fixed_updates: BTreeMap<String, bool>,
    /// The last solve's report (`ok` + diagnostics, or `error`).
    report: serde_json::Value,
}

thread_local! {
    static SESSION: RefCell<Option<Session>> = const { RefCell::new(None) };
}

/// Install the post-run session (the history tail). Pose/fixed updates start
/// from THIS run's outcome — the previous session's pending updates are
/// dropped (the run consumed the document they were meant for).
pub(super) fn install_session(
    state: AssemblyState,
    scene: SceneMap,
    expressions: String,
    configurator: serde_json::Value,
    outcome: LifecycleOutcome,
) {
    SESSION.with(|session| {
        *session.borrow_mut() = Some(Session {
            state,
            scene,
            expressions,
            configurator,
            pose_updates: outcome.pose_updates,
            fixed_updates: outcome.fixed_updates,
            report: outcome.report,
        });
    });
}

fn with_session<T>(f: impl FnOnce(&mut Session) -> Result<T, String>) -> Result<T, String> {
    SESSION.with(|session| {
        let mut session = session.borrow_mut();
        let session = session
            .as_mut()
            .ok_or_else(|| "no assembly session (run a history first)".to_string())?;
        f(session)
    })
}

/// Re-solve the session state against the session scene, merging the
/// outcome's write-backs (the auto-solve every mutation triggers). The
/// returned report carries `movedSolids` (built by the lifecycle) — the
/// display lane re-tessellates exactly those in-place-moved resident solids.
fn solve_session(session: &mut Session) -> serde_json::Value {
    let env = Env::build(&session.expressions, &session.configurator)
        .unwrap_or_else(Env::poisoned);
    let outcome = lifecycle::run_constraints(&mut session.state, &mut session.scene, &env);
    session.pose_updates.extend(outcome.pose_updates);
    session.fixed_updates.extend(outcome.fixed_updates);
    session.report = outcome.report.clone();
    outcome.report
}

// ===========================================================================
// Read surface
// ===========================================================================

/// The current `assembly` block (post-solve) as JSON — the panel's list source
/// and the state the app folds back into its document.
#[wasm_bindgen]
pub fn assembly_state_json() -> String {
    SESSION.with(|session| {
        session
            .borrow()
            .as_ref()
            .map(|session| serde_json::to_value(&session.state).unwrap_or_default())
            .unwrap_or_else(|| serde_json::to_value(AssemblyState::default()).unwrap_or_default())
            .to_string()
    })
}

/// Per-constraint status rows: `[{id, type, enabled, open, status, message,
/// satisfied, error}]`.
#[wasm_bindgen]
pub fn assembly_statuses_json() -> String {
    SESSION.with(|session| {
        let rows: Vec<serde_json::Value> = session
            .borrow()
            .as_ref()
            .map(|session| {
                session
                    .state
                    .constraints
                    .iter()
                    .map(|entry| {
                        serde_json::json!({
                            "id": entry.id(),
                            "type": entry.constraint_type,
                            "enabled": entry.enabled,
                            "open": entry.open,
                            "status": entry.status(),
                            "message": entry
                                .persistent("message")
                                .and_then(|value| value.as_str())
                                .unwrap_or(""),
                            "satisfied": entry
                                .persistent("satisfied")
                                .and_then(|value| value.as_bool())
                                .unwrap_or(false),
                            "error": entry.persistent("error").cloned(),
                        })
                    })
                    .collect()
            })
            .unwrap_or_default();
        serde_json::Value::Array(rows).to_string()
    })
}

/// The last solve's DOF/diagnostics summary (`{ok, dof, rank, redundant,
/// status, strategy, iterations, maxResidual, ...}` or `{ok:false, error}`).
#[wasm_bindgen]
pub fn assembly_dof_json() -> String {
    SESSION.with(|session| {
        session
            .borrow()
            .as_ref()
            .map(|session| session.report.to_string())
            .unwrap_or_else(|| serde_json::json!({ "ok": true, "mates": 0 }).to_string())
    })
}

/// The pending generic feature write-backs: `{"poses": {componentId:
/// transformParam}, "isFixed": {componentId: bool}}`. The transform param is
/// `{translate, rotateEulerDeg (deg, intrinsic XYZ)}` — exactly the shape the
/// ACOMP feature's `inputParams.transform` reader expects.
#[wasm_bindgen]
pub fn assembly_pose_updates_json() -> String {
    SESSION.with(|session| {
        session
            .borrow()
            .as_ref()
            .map(|session| {
                serde_json::json!({
                    "poses": session.pose_updates,
                    "isFixed": session.fixed_updates,
                })
                .to_string()
            })
            .unwrap_or_else(|| serde_json::json!({ "poses": {}, "isFixed": {} }).to_string())
    })
}

/// Fold the session's solved state into a history DOCUMENT: replaces the
/// top-level `assembly` block and writes each pending pose / grounded flag
/// into the owning feature's `inputParams` (matched by `inputParams.id` —
/// generic JSON, no dependency on the ACOMP feature shape). The app calls this
/// on its document before persisting or re-running (the pose-authority
/// write-back).
#[wasm_bindgen]
pub fn assembly_apply_document_json(document_json: &str) -> Result<String, JsValue> {
    assembly_apply_document_impl(document_json).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_apply_document_impl(document_json: &str) -> Result<String, String> {
    let mut document: serde_json::Value = serde_json::from_str(document_json)
        .map_err(|error| format!("assembly document fold: bad document: {error}"))?;
    with_session(|session| {
        let object = document
            .as_object_mut()
            .ok_or_else(|| "assembly document fold: document must be an object".to_string())?;
        object.insert(
            "assembly".into(),
            serde_json::to_value(&session.state)
                .map_err(|error| format!("assembly state serialize: {error}"))?,
        );
        if let Some(features) = object.get_mut("features").and_then(|v| v.as_array_mut()) {
            for feature in features {
                let Some(params) = feature
                    .get_mut("inputParams")
                    .and_then(|value| value.as_object_mut())
                else {
                    continue;
                };
                let Some(id) = params.get("id").and_then(|value| value.as_str()) else {
                    continue;
                };
                let id = id.to_string();
                if let Some(pose) = session.pose_updates.get(&id) {
                    params.insert("transform".into(), pose.clone());
                }
                if let Some(&fixed) = session.fixed_updates.get(&id) {
                    params.insert("isFixed".into(), serde_json::Value::Bool(fixed));
                }
            }
        }
        Ok(())
    })?;
    Ok(document.to_string())
}

/// Overlay geometry for the viewport graphics lane: per enabled constraint,
/// the resolved WORLD anchor points/directions/geometry classes (post-solve),
/// the status, the evaluated value, and (multi-element types) the element
/// role groups — `[{id, type, status, message, anchors, directions, geoms,
/// value, unit, target, groups}]`. Constraints whose selections do not
/// resolve emit status/message only (no anchors).
#[wasm_bindgen]
pub fn assembly_overlay_json() -> String {
    SESSION.with(|session| {
        let session = session.borrow();
        let Some(session) = session.as_ref() else {
            return serde_json::Value::Array(Vec::new()).to_string();
        };
        let env = Env::build(&session.expressions, &session.configurator)
            .unwrap_or_else(Env::poisoned);
        let rows: Vec<serde_json::Value> = session
            .state
            .constraints
            .iter()
            .filter(|entry| entry.enabled)
            .map(|entry| overlay_row(entry, &session.scene, &env))
            .collect();
        serde_json::Value::Array(rows).to_string()
    })
}

fn overlay_row(entry: &ConstraintEntry, scene: &SceneMap, env: &Env) -> serde_json::Value {
    let mut row = serde_json::json!({
        "id": entry.id(),
        "type": entry.constraint_type,
        "status": entry.status(),
        "message": entry
            .persistent("message")
            .and_then(|value| value.as_str())
            .unwrap_or(""),
    });
    let elements = entry.elements();
    let resolved: Vec<_> = elements
        .iter()
        .filter_map(|name| mapping::resolve_element(scene, name).ok())
        .collect();
    if resolved.len() != elements.len() || resolved.is_empty() {
        return row;
    }
    let object = row.as_object_mut().expect("row is an object");
    object.insert(
        "anchors".into(),
        serde_json::Value::Array(
            resolved
                .iter()
                .map(|element| {
                    let p = element.world.representative_point();
                    serde_json::json!([p.x, p.y, p.z])
                })
                .collect(),
        ),
    );
    object.insert(
        "directions".into(),
        serde_json::Value::Array(
            resolved
                .iter()
                .map(|element| match mapping::direction_of(&element.world) {
                    Some(direction) => serde_json::json!([direction.x, direction.y, direction.z]),
                    None => serde_json::Value::Null,
                })
                .collect(),
        ),
    );
    // Per-element geometry class, aligned index-wise with anchors/directions —
    // the render-side distance-arrow builder needs to identify the BASE PLANE
    // (the perpendicular-foot construction); a direction alone is ambiguous
    // (lines and circles carry one too).
    object.insert(
        "geoms".into(),
        serde_json::Value::Array(
            resolved
                .iter()
                .map(|element| serde_json::json!(geometry_tag(&element.world)))
                .collect(),
        ),
    );
    // Measured value/target — and the element ROLE groups a multi-element type
    // inferred (center's `[width pair, tab]`) — via the mapper on a THROWAWAY
    // clone (the mapper maintains the orientation-preference cache; an
    // overlay read must not mutate persisted state).
    // `fixed` has no mapper (it grounds a body; nothing to measure).
    if entry.constraint_type == "fixed" {
        return row;
    }
    let mut scratch = entry.clone();
    if let Ok(mapped) = mapping::map_constraint(&mut scratch, &resolved, env) {
        if let Some((value, unit)) = mapped.measured {
            object.insert("value".into(), serde_json::json!(value));
            object.insert("unit".into(), serde_json::json!(unit));
        }
        if let Some(target) = mapped.target {
            object.insert("target".into(), serde_json::json!(target));
        }
        if !mapped.groups.is_empty() {
            object.insert("groups".into(), serde_json::json!(mapped.groups));
        }
    }
    row
}

/// The coarse geometry class of a resolved selection (the overlay row's
/// `geoms` vocabulary — lowercase `SelectionGeometry` variant names).
fn geometry_tag(geometry: &crate::SelectionGeometry) -> &'static str {
    match geometry {
        crate::SelectionGeometry::Plane { .. } => "plane",
        crate::SelectionGeometry::Line { .. } => "line",
        crate::SelectionGeometry::Axis { .. } => "axis",
        crate::SelectionGeometry::Circle { .. } => "circle",
        crate::SelectionGeometry::Sphere { .. } => "sphere",
        crate::SelectionGeometry::Point { .. } => "point",
    }
}

// ===========================================================================
// Mutation surface
// ===========================================================================

/// Add a constraint of `constraint_type` with the given `inputParams` JSON
/// (elements etc.; `id` is minted from the type's short name + the persistent
/// counter when absent). Auto-solves; returns `{id, report}`.
#[wasm_bindgen]
pub fn assembly_add_constraint_json(
    constraint_type: &str,
    params_json: &str,
) -> Result<String, JsValue> {
    assembly_add_constraint_impl(constraint_type, params_json).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_add_constraint_impl(constraint_type: &str, params_json: &str) -> Result<String, String> {
    let def = constraints::constraint_type(constraint_type)
        .ok_or_else(|| format!("Unknown constraint type: {constraint_type}"))?;
    let mut params: serde_json::Value = serde_json::from_str(params_json)
        .map_err(|error| format!("constraint params: {error}"))?;
    if !params.is_object() {
        return Err("constraint params must be a JSON object".to_string());
    }
    with_session(|session| {
        let id = match params.get("id").and_then(|value| value.as_str()) {
            Some(id) if !id.is_empty() => id.to_string(),
            _ => {
                session.state.id_counter += 1;
                let id = format!("{}{}", def.short_name, session.state.id_counter);
                params
                    .as_object_mut()
                    .expect("checked object above")
                    .insert("id".into(), serde_json::Value::String(id.clone()));
                id
            }
        };
        session.state.constraints.push(ConstraintEntry {
            constraint_type: constraint_type.to_string(),
            input_params: params.clone(),
            persistent_data: serde_json::Value::Object(serde_json::Map::new()),
            enabled: true,
            open: true,
        });
        let report = solve_session(session);
        Ok(serde_json::json!({ "id": id, "report": report }).to_string())
    })
}

/// Replace a constraint's `inputParams` (the dialog commit). Auto-solves;
/// returns the solve report.
#[wasm_bindgen]
pub fn assembly_update_constraint_json(id: &str, params_json: &str) -> Result<String, JsValue> {
    assembly_update_constraint_impl(id, params_json).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_update_constraint_impl(id: &str, params_json: &str) -> Result<String, String> {
    let mut params: serde_json::Value = serde_json::from_str(params_json)
        .map_err(|error| format!("constraint params: {error}"))?;
    if !params.is_object() {
        return Err("constraint params must be a JSON object".to_string());
    }
    if params.get("id").and_then(|value| value.as_str()).is_none() {
        params
            .as_object_mut()
            .expect("checked object above")
            .insert("id".into(), serde_json::Value::String(id.to_string()));
    }
    with_session(|session| {
        let entry = find_entry(session, id)?;
        entry.input_params = params.clone();
        Ok(solve_session(session).to_string())
    })
}

/// Delete a constraint. Auto-solves; returns the solve report.
#[wasm_bindgen]
pub fn assembly_remove_constraint_json(id: &str) -> Result<String, JsValue> {
    assembly_remove_constraint_impl(id).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_remove_constraint_impl(id: &str) -> Result<String, String> {
    with_session(|session| {
        let index = find_index(session, id)?;
        session.state.constraints.remove(index);
        Ok(solve_session(session).to_string())
    })
}

/// Enable/disable a constraint. Auto-solves; returns the solve report.
#[wasm_bindgen]
pub fn assembly_set_constraint_enabled_json(id: &str, enabled: bool) -> Result<String, JsValue> {
    assembly_set_constraint_enabled_impl(id, enabled).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_set_constraint_enabled_impl(id: &str, enabled: bool) -> Result<String, String> {
    with_session(|session| {
        find_entry(session, id)?.enabled = enabled;
        Ok(solve_session(session).to_string())
    })
}

/// Persist a constraint row's dialog expansion state (view state — no solve).
#[wasm_bindgen]
pub fn assembly_set_constraint_open_json(id: &str, open: bool) -> Result<(), JsValue> {
    assembly_set_constraint_open_impl(id, open).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_set_constraint_open_impl(id: &str, open: bool) -> Result<(), String> {
    with_session(|session| {
        find_entry(session, id)?.open = open;
        Ok(())
    })
}

/// Reorder a constraint to `index` (clamped). Auto-solves; returns the report.
#[wasm_bindgen]
pub fn assembly_move_constraint_json(id: &str, index: usize) -> Result<String, JsValue> {
    assembly_move_constraint_impl(id, index).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_move_constraint_impl(id: &str, index: usize) -> Result<String, String> {
    with_session(|session| {
        let from = find_index(session, id)?;
        let entry = session.state.constraints.remove(from);
        let to = index.min(session.state.constraints.len());
        session.state.constraints.insert(to, entry);
        Ok(solve_session(session).to_string())
    })
}

// ===========================================================================
// Automatic inference (super::infer)
// ===========================================================================

/// The inferable constraint types, joined to the catalogue: `[{type, label,
/// icon, longName, detects, defaultOn}]`. Session-free — the dialog can list
/// what the button offers before any document is open.
#[wasm_bindgen]
pub fn assembly_inferable_types_json() -> String {
    super::infer::inferable_types_json().to_string()
}

/// SCAN the live scene for constraints its component placement implies, without
/// creating anything. `options_json` is [`super::infer::InferOptions`] (`{}` for
/// the defaults). Never `Err`: a missing session or a bad options object comes
/// back as `{ok: false, error}` — a `JsValue` error aborts a native build.
#[wasm_bindgen]
pub fn assembly_infer_constraints_json(options_json: &str) -> String {
    match infer_scan(options_json) {
        Ok(report) => report.to_string(),
        Err(error) => refusal(&error),
    }
}

/// Scan and CREATE: every accepted candidate becomes a constraint, then ONE
/// solve runs for the whole batch (each inferred constraint is already
/// satisfied at the pose it was read from, so a healthy assembly does not
/// move). Returns `{ok, created: [id…], report, solve}`.
#[wasm_bindgen]
pub fn assembly_apply_inferred_constraints_json(options_json: &str) -> String {
    match infer_apply(options_json) {
        Ok(report) => report.to_string(),
        Err(error) => refusal(&error),
    }
}

fn refusal(error: &str) -> String {
    serde_json::json!({ "ok": false, "error": error }).to_string()
}

fn infer_scan(options_json: &str) -> Result<serde_json::Value, String> {
    let options = super::infer::InferOptions::parse(options_json)?;
    with_session(|session| Ok(super::infer::scan(&session.state, &session.scene, &options).report()))
}

fn infer_apply(options_json: &str) -> Result<serde_json::Value, String> {
    let options = super::infer::InferOptions::parse(options_json)?;
    with_session(|session| {
        let scan = super::infer::scan(&session.state, &session.scene, &options);
        let report = scan.report();
        let created = super::infer::apply(&mut session.state, &scan.candidates);
        let solve = if created.is_empty() {
            serde_json::Value::Null
        } else {
            solve_session(session)
        };
        Ok(serde_json::json!({
            "ok": true,
            "created": created,
            "report": report,
            "solve": solve,
        }))
    })
}

/// Manual solve (the panel's Solve button). Returns the solve report.
#[wasm_bindgen]
pub fn assembly_run_solve_json() -> Result<String, JsValue> {
    assembly_run_solve_impl().map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_run_solve_impl() -> Result<String, String> {
    with_session(|session| Ok(solve_session(session).to_string()))
}

fn find_index(session: &Session, id: &str) -> Result<usize, String> {
    session
        .state
        .constraints
        .iter()
        .position(|entry| entry.id() == id)
        .ok_or_else(|| format!("unknown constraint '{id}'"))
}

fn find_entry<'a>(session: &'a mut Session, id: &str) -> Result<&'a mut ConstraintEntry, String> {
    let index = find_index(session, id)?;
    Ok(&mut session.state.constraints[index])
}
