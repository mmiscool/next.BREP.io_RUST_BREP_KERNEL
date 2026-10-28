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

#[derive(Clone)]
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
    /// Bumped by every install (a run's tail or an import): which session this
    /// thread holds, so a holder can tell whether another document's run or
    /// replay has replaced the one it installed.
    static SESSION_SERIAL: std::cell::Cell<u64> = const { std::cell::Cell::new(0) };
    /// Handles registered by the last [`assembly_session_import`]: owned by the
    /// imported session alone, freed when any other session replaces it.
    static IMPORTED: RefCell<Vec<u32>> = const { RefCell::new(Vec::new()) };
}

/// Replace this thread's session. What it replaces, and the solids a previous
/// IMPORT owned (out of the registry already), are handed back for the caller
/// to drop: freeing a few hundred solids costs milliseconds.
fn replace_session(next: Session) -> RetiredSession {
    let solids = IMPORTED
        .with(|imported| std::mem::take(&mut *imported.borrow_mut()))
        .into_iter()
        .filter_map(crate::take_registered_solid)
        .collect();
    let session = SESSION.with(|session| session.borrow_mut().replace(next));
    SESSION_SERIAL.with(|serial| serial.set(serial.get() + 1));
    RetiredSession { _session: session, _solids: solids }
}

/// A replaced session and the solids only it owned, to drop where it costs
/// nothing (another thread). Dropping it is all there is to do with it.
pub struct RetiredSession {
    _session: Option<Session>,
    _solids: Vec<crate::BrepSolid>,
}

/// Which session this thread holds (see `SESSION_SERIAL`); 0 = none yet.
pub fn assembly_session_serial() -> u64 {
    SESSION_SERIAL.with(std::cell::Cell::get)
}

/// An owned, self-contained copy of a thread's assembly session: the session
/// and a clone of every solid its scene names, keyed by the handle it had
/// THERE. It shares nothing with the thread it came from, so that thread may
/// free or reuse every handle afterwards ([`assembly_session_import`]).
pub struct SessionTransfer {
    session: Session,
    solids: Vec<(u32, crate::BrepSolid)>,
}

impl std::fmt::Debug for SessionTransfer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SessionTransfer").field("solids", &self.solids.len()).finish()
    }
}

/// Copy this thread's session for another thread (`None` without one).
pub fn assembly_session_export() -> Option<SessionTransfer> {
    SESSION.with(|session| {
        let session = session.borrow();
        let session = session.as_ref()?;
        let scene = &session.scene;
        let mut handles: Vec<u32> = scene
            .solids
            .values()
            .copied()
            .chain(scene.faces.values().map(|face| face.handle))
            .chain(scene.edges.values().map(|edge| edge.handle))
            .collect();
        handles.sort_unstable();
        handles.dedup();
        let solids = handles
            .into_iter()
            .filter_map(|handle| {
                crate::with_registered_solid_str(handle, |solid| Ok(solid.clone())).ok().map(|solid| (handle, solid))
            })
            .collect();
        Some(SessionTransfer { session: session.clone(), solids })
    })
}

/// Install a [`SessionTransfer`] as this thread's session: every solid is
/// registered HERE under a fresh handle and the scene is re-pointed at it.
/// Returns what it replaced (the previous session and the solids a previous
/// import owned) for the caller to drop.
pub fn assembly_session_import(transfer: SessionTransfer) -> RetiredSession {
    let SessionTransfer { mut session, solids } = transfer;
    let mut fresh = std::collections::HashMap::with_capacity(solids.len());
    let mut owned = Vec::with_capacity(solids.len());
    for (old, solid) in solids {
        let handle = crate::register_solid_value(solid);
        fresh.insert(old, handle);
        owned.push(handle);
    }
    // A handle with no copy (its solid was gone there) must not alias a
    // solid that happens to hold the same number here.
    let remap = |handle: &mut u32| *handle = fresh.get(handle).copied().unwrap_or(u32::MAX);
    let scene = &mut session.scene;
    scene.solids.values_mut().for_each(remap);
    scene.faces.values_mut().for_each(|face| remap(&mut face.handle));
    scene.edges.values_mut().for_each(|edge| remap(&mut edge.handle));
    let retired = replace_session(session);
    IMPORTED.with(|imported| *imported.borrow_mut() = owned);
    retired
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
    drop(replace_session(Session {
        state,
        scene,
        expressions,
        configurator,
        pose_updates: outcome.pose_updates,
        fixed_updates: outcome.fixed_updates,
        report: outcome.report,
    }));
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
                            "conflictWith": entry.persistent("conflictWith").cloned().unwrap_or(serde_json::json!([])),
                            "suppressSuggested": entry
                                .persistent("suppressSuggested")
                                .and_then(|value| value.as_bool())
                                .unwrap_or(false),
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
            .map(|session| with_nested_constraints(&session.report, &session.scene).to_string())
            .unwrap_or_else(|| serde_json::json!({ "ok": true, "mates": 0 }).to_string())
    })
}

/// `report` plus `nestedConstraints`: the rows of every sub-assembly's own
/// constraints (namespaced, every level), which solved inside the sub-assembly
/// and are reported, not re-solved, here. Read from the session scene's
/// component records, so a mutation re-solve (which replaces `report`) keeps
/// them. Absent when no component carries any.
fn with_nested_constraints(report: &serde_json::Value, scene: &SceneMap) -> serde_json::Value {
    let rows: Vec<&crate::feature_pipeline::component::NestedConstraint> = scene
        .iter_components()
        .flat_map(|record| &record.nested_constraints)
        .collect();
    let mut report = report.clone();
    if let (false, Some(map)) = (rows.is_empty(), report.as_object_mut()) {
        map.insert(
            "nestedConstraints".into(),
            serde_json::to_value(rows).unwrap_or_default(),
        );
    }
    report
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
                    // Numeric solver write-back must not erase authored motion
                    // expressions. The evaluated component record owns that pose.
                    fn has_expression(value: &serde_json::Value) -> bool {
                        match value {
                            serde_json::Value::String(_) => true,
                            serde_json::Value::Array(a) => a.iter().any(has_expression),
                            serde_json::Value::Object(o) => o.values().any(has_expression),
                            _ => false,
                        }
                    }
                    if !params.get("transform").is_some_and(has_expression) {
                        params.insert("transform".into(), pose.clone());
                    }
                }
                // A flexible instance's occurrence poses (`ACOMP1:ACOMP2`),
                // part-local, persist on the instance — never in the child
                // document.
                let prefix = format!("{id}:");
                for (key, pose) in session.pose_updates.range(prefix.clone()..) {
                    let Some(path) = key.strip_prefix(&prefix) else { break };
                    let poses = params
                        .entry("flexiblePoses")
                        .or_insert_with(|| serde_json::Value::Object(Default::default()));
                    if let Some(poses) = poses.as_object_mut() {
                        poses.insert(path.to_string(), pose.clone());
                    }
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
        if let Some(axis) = mapped.angle_axis {
            object.insert("angleAxis".into(), serde_json::json!([axis.x, axis.y, axis.z]));
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

/// One resolved element as the replace-component remap reads it: the geometry
/// class plus the frame in the owning component's LOCAL space (a component
/// swap keeps the ACOMP pose, so local frames of the old and the new part are
/// directly comparable).
fn local_element_json(element: &mapping::ResolvedElement) -> serde_json::Value {
    let point = mapping::local_point(element);
    let direction = mapping::direction_of(&element.local)
        .map(|d| serde_json::json!([d.x, d.y, d.z]))
        .unwrap_or(serde_json::Value::Null);
    let radius = match element.local {
        crate::SelectionGeometry::Axis { radius, .. } => radius,
        crate::SelectionGeometry::Sphere { radius, .. }
        | crate::SelectionGeometry::Circle { radius, .. } => Some(radius),
        _ => None,
    };
    serde_json::json!({
        "ref": element.name,
        "component": element.component,
        "geom": geometry_tag(&element.local),
        "point": point,
        "direction": direction,
        "radius": radius,
    })
}

/// Resolve selection refs against the live session WITHOUT touching any
/// constraint: `refs_json` is `["ACOMP2:Part_PZ", …]`; the reply is one row
/// per ref, `{ref, ok: true, component, geom, point, direction, radius}` in
/// the owning component's local frame, or `{ref, ok: false, status, error}`.
/// Read-only — the replace-component lane snapshots a mate's references
/// before the swap with it.
#[wasm_bindgen]
pub fn assembly_resolve_refs_json(refs_json: &str) -> String {
    let refs: Vec<String> = match serde_json::from_str(refs_json) {
        Ok(refs) => refs,
        Err(error) => return refusal(&format!("refs: {error}")),
    };
    SESSION.with(|session| {
        let session = session.borrow();
        let Some(session) = session.as_ref() else {
            return refusal("no assembly session (run a history first)");
        };
        let rows: Vec<serde_json::Value> = refs
            .iter()
            .map(|name| match mapping::resolve_element(&session.scene, name) {
                Ok(element) => {
                    let mut row = local_element_json(&element);
                    row["ok"] = serde_json::Value::Bool(true);
                    row
                }
                Err(failure) => serde_json::json!({
                    "ref": name,
                    "ok": false,
                    "status": failure.status,
                    "error": failure.message,
                }),
            })
            .collect();
        serde_json::Value::Array(rows).to_string()
    })
}

/// Every NAMED face and edge of one component, resolved like
/// [`assembly_resolve_refs_json`] (component-local frames). Entities without
/// an analytic frame are skipped: a mate could never have referenced them.
/// An unknown component is an empty list.
#[wasm_bindgen]
pub fn assembly_component_entities_json(component_id: &str) -> String {
    SESSION.with(|session| {
        let session = session.borrow();
        let Some(session) = session.as_ref() else {
            return serde_json::Value::Array(Vec::new()).to_string();
        };
        let mut names: Vec<(&'static str, String)> = Vec::new();
        for (_, handle) in session.scene.component_solids(component_id) {
            let _ = crate::with_registered_solid_str(handle, |solid| {
                for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                    if let Some(name) = &face.name {
                        names.push(("face", name.clone()));
                    }
                }
                for edge in &solid.edges {
                    if let Some(name) = &edge.name {
                        names.push(("edge", name.clone()));
                    }
                }
                Ok(())
            });
        }
        let rows: Vec<serde_json::Value> = names
            .into_iter()
            .filter_map(|(kind, name)| {
                let element = mapping::resolve_element(&session.scene, &name).ok()?;
                let mut row = local_element_json(&element);
                row["kind"] = serde_json::Value::String(kind.into());
                Some(row)
            })
            .collect();
        serde_json::Value::Array(rows).to_string()
    })
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
/// creating anything. `options_json` is `super::infer::InferOptions` (`{}` for
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

/// The live-drag dry run against the session (`lifecycle::drag_solve`).
/// Request `{poses: {componentId: {translate, rotateEulerDeg}}, unfix?: [id],
/// reference?: {componentId: pose}, explain?: bool}` — the dragged components at their
/// dragged poses, and the last good live poses to re-base mechanical mates at. Reply `{ok: true, poses:
/// {componentId: {translate, rotateEulerDeg}}, blocking: [constraintId]}` for every movable body
/// (`blocking`: with `explain`, the constraints that would not let the dragged
/// components reach the drag pose; empty otherwise);
/// `Err` when the mates cannot hold at this drag. Nothing in the session (the
/// state, the scene, the pending write-backs, the report) changes.
#[wasm_bindgen]
pub fn assembly_drag_solve_json(request_json: &str) -> Result<String, JsValue> {
    assembly_drag_solve_impl(request_json).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_drag_solve_impl(request_json: &str) -> Result<String, String> {
    let request: serde_json::Value =
        serde_json::from_str(request_json).map_err(|error| format!("drag solve request: {error}"))?;
    let mut drag = BTreeMap::new();
    for (id, pose) in request
        .get("poses")
        .and_then(|poses| poses.as_object())
        .ok_or("drag solve request: missing poses")?
    {
        drag.insert(id.clone(), mapping::pose_params_to_transform(pose)?);
    }
    let mut reference = BTreeMap::new();
    for (id, pose) in request.get("reference").and_then(|poses| poses.as_object()).into_iter().flatten() {
        reference.insert(id.clone(), mapping::pose_params_to_transform(pose)?);
    }
    let unfix: Vec<String> = request
        .get("unfix")
        .and_then(|unfix| unfix.as_array())
        .map(|ids| ids.iter().filter_map(|id| id.as_str().map(str::to_string)).collect())
        .unwrap_or_default();
    let explain = request.get("explain").and_then(|explain| explain.as_bool()).unwrap_or(false);
    with_session(|session| {
        let env = Env::build(&session.expressions, &session.configurator)
            .unwrap_or_else(Env::poisoned);
        let (poses, blocking) =
            lifecycle::drag_solve(&session.state, &mut session.scene, &env, &drag, &unfix, &reference, explain)?;
        let poses: serde_json::Map<String, serde_json::Value> = poses
            .iter()
            .map(|(id, transform)| (id.clone(), mapping::transform_to_pose_params(transform)))
            .collect();
        Ok(serde_json::json!({ "ok": true, "poses": poses, "blocking": blocking }).to_string())
    })
}

/// One frame of a motion study against the session (`lifecycle::frame_solve`).
/// Request `{overrides: {constraintId: {value?: number, enabled?: bool}},
/// reference?: {componentId: {translate, rotateEulerDeg}}}`: `value` drives a
/// distance or angle constraint to it (model units or degrees; an angle may
/// run past 360), `enabled` switches a constraint off or on for the frame, and
/// `reference` is the previous frame's poses (the warm start). Reply `{ok:
/// true, poses: {componentId: {translate, rotateEulerDeg}}}` for every movable
/// body; `Err` when the mates cannot hold the frame or an override names a
/// constraint the session lacks or drives one that is not a distance or angle.
/// Nothing in the session changes.
#[wasm_bindgen]
pub fn assembly_frame_solve_json(request_json: &str) -> Result<String, JsValue> {
    assembly_frame_solve_impl(request_json).map_err(|error| JsValue::from_str(&error))
}

pub fn assembly_frame_solve_impl(request_json: &str) -> Result<String, String> {
    let request: serde_json::Value =
        serde_json::from_str(request_json).map_err(|error| format!("frame solve request: {error}"))?;
    let mut reference = BTreeMap::new();
    for (id, pose) in request.get("reference").and_then(|poses| poses.as_object()).into_iter().flatten() {
        reference.insert(id.clone(), mapping::pose_params_to_transform(pose)?);
    }
    with_session(|session| {
        let mut overrides = BTreeMap::new();
        for (id, change) in request.get("overrides").and_then(|o| o.as_object()).into_iter().flatten() {
            let mut entry = lifecycle::FrameOverride::default();
            if let Some(value) = change.get("value").filter(|v| !v.is_null()) {
                let value = value.as_f64().ok_or_else(|| format!("frame solve: '{id}' value is not a number"))?;
                let kind = session
                    .state
                    .constraints
                    .iter()
                    .find(|c| c.id() == id)
                    .map(|c| c.constraint_type.as_str())
                    .ok_or_else(|| format!("frame solve: unknown constraint '{id}'"))?;
                let param = match kind {
                    "distance" => "distance",
                    "angle" => "angle",
                    other => return Err(format!("frame solve: '{id}' is a {other} constraint; a motor drives a distance or angle")),
                };
                entry.value = Some((param.to_string(), value));
            }
            entry.enabled = change.get("enabled").and_then(|e| e.as_bool());
            overrides.insert(id.clone(), entry);
        }
        let env = Env::build(&session.expressions, &session.configurator).unwrap_or_else(Env::poisoned);
        let poses = lifecycle::frame_solve(&session.state, &mut session.scene, &env, &overrides, &reference)?;
        let poses: serde_json::Map<String, serde_json::Value> = poses
            .iter()
            .map(|(id, transform)| (id.clone(), mapping::transform_to_pose_params(transform)))
            .collect();
        Ok(serde_json::json!({ "ok": true, "poses": poses }).to_string())
    })
}

/// Refresh the mechanical mates' stored references at the released poses of
/// a live drag (`lifecycle::rebase_mechanical`). Request `{poses:
/// {componentId: {translate, rotateEulerDeg}}}`. Reply: the updated
/// `assembly` block JSON when a mechanical mate was refreshed, else `null`.
/// The caller writes the block in the SAME undo step as the poses, so the
/// next run maps a multi-turn drag from where it ended.
pub fn assembly_rebase_mechanical_impl(request_json: &str) -> Result<String, String> {
    let request: serde_json::Value =
        serde_json::from_str(request_json).map_err(|error| format!("rebase request: {error}"))?;
    let mut poses = BTreeMap::new();
    for (id, pose) in request.get("poses").and_then(|poses| poses.as_object()).into_iter().flatten() {
        poses.insert(id.clone(), mapping::pose_params_to_transform(pose)?);
    }
    with_session(|session| {
        let env = Env::build(&session.expressions, &session.configurator).unwrap_or_else(Env::poisoned);
        let changed = lifecycle::rebase_mechanical(&mut session.state, &mut session.scene, &env, &poses);
        Ok(if changed {
            serde_json::to_string(&session.state).map_err(|error| error.to_string())?
        } else {
            "null".to_string()
        })
    })
}

/// What **Flip** would do to constraint `id`, read from the live session:
/// `{ok: true, patch, seed, elements: [{component, fixed, anchor, direction,
/// geom}], value, angleAxis}` (world frame, pick order), or `{ok: false,
/// reason}` for a type with no other solution. Read-only: the caller writes the
/// patch and the seeded pose as one edit (`constraints::flip_rule`).
#[wasm_bindgen]
pub fn assembly_flip_plan_json(id: &str) -> String {
    SESSION.with(|session| {
        let session = session.borrow();
        let Some(session) = session.as_ref() else {
            return refusal("no assembly session (run a history first)");
        };
        let Some(entry) = session.state.constraints.iter().find(|entry| entry.id() == id) else {
            return refusal(&format!("unknown constraint '{id}'"));
        };
        let env = Env::build(&session.expressions, &session.configurator).unwrap_or_else(Env::poisoned);
        let row = overlay_row(entry, &session.scene, &env);
        let geoms: Vec<&str> = row["geoms"].as_array().into_iter().flatten().filter_map(|g| g.as_str()).collect();
        let (patch, seed) = match constraints::flip_rule(&entry.constraint_type, &entry.input_params, &geoms) {
            Ok(rule) => rule,
            Err(reason) => return serde_json::json!({ "ok": false, "reason": reason }).to_string(),
        };
        let elements: Vec<serde_json::Value> = entry
            .elements()
            .iter()
            .enumerate()
            .map(|(index, name)| {
                let component = mapping::resolve_element(&session.scene, name).map(|element| element.component).unwrap_or_default();
                serde_json::json!({
                    "component": component,
                    "fixed": session.scene.component_fixed(&component).unwrap_or(false),
                    "anchor": row["anchors"].get(index).cloned().unwrap_or(serde_json::Value::Null),
                    "direction": row["directions"].get(index).cloned().unwrap_or(serde_json::Value::Null),
                    "geom": row["geoms"].get(index).cloned().unwrap_or(serde_json::Value::Null),
                })
            })
            .collect();
        serde_json::json!({
            "ok": true,
            "patch": patch,
            "seed": seed,
            "elements": elements,
            "value": row["value"],
            "angleAxis": row["angleAxis"],
        })
        .to_string()
    })
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
