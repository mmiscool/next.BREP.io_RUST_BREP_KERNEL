//! What each `cad.*` command does to the app (contract §8), over the host
//! protocol a session already speaks.
//!
//! Every write names the document context: the app checks it on the frame
//! thread right before the handler (`BREP_app/src/automation/queue.rs`
//! `bind_context`), so a tab switch or a user edit between steps is refused
//! there, never silently retargeted. Recipe writes go through the native
//! structured batch with `rollback_on_error`, which evaluates the staged
//! recipe and restores history, library, metadata, counters and undo when it
//! fails — so a refused sketch or extrude leaves nothing committed.
use super::artifacts::{Artifacts, Owner};
use super::wire::{self, context_json, doc_effect, ApiError, Effects};
use crate::session::Session;
use serde::Deserialize;
use serde_json::{json, Value};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// What a command needs besides its arguments.
pub struct Run<'a> {
    pub session: Arc<Session>,
    pub owner: Owner,
    pub artifacts: &'a Artifacts,
    pub cancel: &'a AtomicBool,
    pub progress: &'a (dyn Fn(&str) + Send + Sync),
    pub created_document: &'a (dyn Fn(&str) + Send + Sync),
    pub begin_commit: &'a (dyn Fn() -> Result<(), ApiError> + Send + Sync),
}

/// A command's result: its typed output, its actual effects and its geometry verdict.
pub struct Done {
    pub output: Value,
    pub effects: Effects,
    pub geometry: Value,
}

const WAIT_MS: u64 = 15 * 60 * 1000;

fn geometry(status: &str, summary: impl Into<String>) -> Value {
    json!({ "status": status, "report": null, "summary": wire::bounded(summary.into(), 1024) })
}

fn invalid(message: impl Into<String>) -> ApiError {
    ApiError::new("invalid_arguments", "validation", message)
}

fn parse<T: for<'de> Deserialize<'de>>(args: &Value) -> Result<T, ApiError> {
    serde_json::from_value(args.clone()).map_err(|e| invalid(e.to_string()))
}

async fn call(session: &Session, cmd: &str, args: Value, phase: &'static str) -> Result<Value, ApiError> {
    // A host that stopped under the call: whatever it had applied is unknown.
    let r = session.host.call(cmd, args).await.map_err(|e| ApiError::new("host_stopped", phase, wire::sanitize(&e)).effects(Effects::unknown()))?;
    if r.ok {
        Ok(r.result.unwrap_or(Value::Null))
    } else {
        let mut error = ApiError::from_app(&r.error.unwrap_or_default(), phase);
        if matches!(phase, "commit" | "external_io") && error.code == "internal_error" {
            error.effects = Effects::unknown();
        }
        Err(error)
    }
}

/// The tab record (`docs_list`) of a document handle.
async fn tab(session: &Session, id: &str) -> Result<Value, ApiError> {
    match wire::handle_shape_doc(id) {
        wire::HandleShape::Malformed => return Err(invalid("documentId is not a document handle")),
        _ => {}
    }
    let tabs = call(session, "docs_list", json!({}), "lookup").await?;
    tabs["tabs"]
        .as_array()
        .and_then(|t| t.iter().find(|t| t["id"] == id))
        .cloned()
        .ok_or_else(|| ApiError::new("document_unknown", "lookup", "no open document has that handle (closed, or never opened)"))
}

/// The commit fence before the first effect: the document must still be at
/// the admitted revision.
async fn fence(session: &Session, id: &str, expected: &str) -> Result<Value, ApiError> {
    let t = tab(session, id).await?;
    let current = t["revision"].as_str().unwrap_or("0");
    if current != expected {
        return Err(ApiError::new("revision_conflict", "admission", format!("the document is at revision {current}, not {expected}; read it again before writing"))
            .context(context_json(id, current)));
    }
    Ok(t)
}

/// Wait until THIS document's engine is idle (two quiet frames in a row):
/// `frame_info` with the document's context is routed to it, background tab
/// or not, so another tab's work neither delays nor hides this one's.
async fn settle(run: &Run<'_>, id: &str) -> Result<(), ApiError> {
    let platform = &run.session.host.platform;
    let start = platform.now_ms();
    let mut quiet = 0;
    loop {
        let info = call(&run.session, "frame_info", json!({ "context": context_json(id, "0") }), "evaluation").await?;
        if info["poisoned"] == true {
            return Err(ApiError::new("internal_error", "evaluation", "the app session is poisoned").effects(Effects::unknown()));
        }
        if info["idle"] == true {
            quiet += 1;
            if quiet > 2 {
                return Ok(());
            }
        } else {
            quiet = 0;
        }
        if platform.now_ms() - start > WAIT_MS as f64 {
            return Err(ApiError::new("evaluation_failed", "evaluation", "the document's evaluation did not settle within 15 minutes").effects(Effects::unknown()));
        }
        platform.sleep(if quiet > 0 { 5 } else { 30 }).await;
    }
}

/// The document's revision now (after a write has landed).
async fn revision(session: &Session, id: &str) -> Result<String, ApiError> {
    Ok(tab(session, id).await?["revision"].as_str().unwrap_or("0").to_string())
}

fn check_cancel(run: &Run<'_>) -> Result<(), ApiError> {
    if run.cancel.load(Ordering::Relaxed) {
        Err(ApiError::new("cancelled", "running", "cancelled before any effect"))
    } else {
        Ok(())
    }
}

pub async fn execute(path: &str, args: &Value, context: Option<(String, String)>, run: &Run<'_>) -> Result<Done, ApiError> {
    match path {
        "cad.documents.create" => create(args, run).await,
        "cad.documents.close" => close(args, context.ok_or_else(|| invalid("context is required"))?, run).await,
        "cad.sketch.create" => sketch(args, context.ok_or_else(|| invalid("context is required"))?, run).await,
        "cad.solids.extrude" => extrude(args, context.ok_or_else(|| invalid("context is required"))?, run).await,
        "cad.files.export" => export(args, context.ok_or_else(|| invalid("context is required"))?, run).await,
        "cad.files.import" => import(args, context.ok_or_else(|| invalid("context is required"))?, run).await,
        "cad.model.inspect" => {
            let output = inspect(context.ok_or_else(|| invalid("context is required"))?, &run.session).await?;
            Ok(Done { output, effects: Effects::none(), geometry: geometry("not_applicable", "read") })
        }
        other => Err(ApiError::new("method_unknown", "lookup", format!("no method `{other}`"))),
    }
}

/// Argument validation that does not need the app (run at admission).
pub fn validate(path: &str, args: &Value) -> Result<Value, ApiError> {
    let finite = |v: f64, what: &str| -> Result<(), ApiError> {
        if v.is_finite() && v > 0.0 && v <= 1_000_000.0 {
            Ok(())
        } else {
            Err(invalid(format!("{what} must be finite, > 0 and ≤ 1,000,000 mm")))
        }
    };
    let mut defaulted = args.clone();
    match path {
        "cad.documents.create" => {
            let a: CreateArgs = parse(args)?;
            let n = a.title.chars().count();
            if !(1..=128).contains(&n) {
                return Err(invalid("title must be 1–128 characters"));
            }
            if a.units != "mm" {
                return Err(invalid("units must be \"mm\""));
            }
        }
        "cad.documents.close" => {
            let a: CloseArgs = parse(args)?;
            defaulted = json!({ "discardUnsaved": a.discard_unsaved });
        }
        "cad.sketch.create" => {
            let a: SketchArgs = parse(args)?;
            if a.profile.kind != "rectangle" {
                return Err(invalid("profile.kind must be \"rectangle\""));
            }
            finite(a.profile.width_mm, "profile.widthMm")?;
            finite(a.profile.height_mm, "profile.heightMm")?;
            plane_basis(&a.plane)?;
        }
        "cad.solids.extrude" => {
            let a: ExtrudeArgs = parse(args)?;
            finite(a.distance_mm, "distanceMm")?;
            if a.sketch.kind != "sketch" {
                return Err(invalid("sketch must be a sketch reference"));
            }
        }
        "cad.model.inspect" => {
            let _: Empty = parse(args)?;
        }
        "cad.files.export" => {
            let a: ExportArgs = parse(args)?;
            if a.format != "step" {
                return Err(invalid("format must be \"step\""));
            }
            if a.entities.is_empty() || a.entities.len() > 256 {
                return Err(invalid("entities must hold 1–256 solids"));
            }
            if a.entities.iter().any(|e| e.kind != "solid") {
                return Err(invalid("export takes solid references only"));
            }
            let mut seen = std::collections::HashSet::new();
            if !a.entities.iter().all(|e| seen.insert(&e.entity_id)) {
                return Err(invalid("entities repeats a solid"));
            }
        }
        "cad.files.import" => {
            let a: ImportArgs = parse(args)?;
            if a.format != "step" {
                return Err(invalid("format must be \"step\""));
            }
        }
        other => return Err(ApiError::new("method_unknown", "lookup", format!("no method `{other}`"))),
    }
    Ok(defaulted)
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Empty {}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CreateArgs {
    title: String,
    units: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct CloseArgs {
    #[serde(default)]
    discard_unsaved: bool,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct Profile {
    kind: String,
    width_mm: f64,
    height_mm: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SketchArgs {
    plane: String,
    profile: Profile,
}

#[derive(Deserialize, Clone)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
pub struct EntityRef {
    pub document_id: String,
    pub entity_id: String,
    pub kind: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct ExtrudeArgs {
    sketch: EntityRef,
    distance_mm: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ExportArgs {
    format: String,
    entities: Vec<EntityRef>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct ImportArgs {
    artifact_id: String,
    format: String,
}

/// The base plane's frame, right-handed: x × y = normal.
fn plane_basis(plane: &str) -> Result<Value, ApiError> {
    let (x, y, z) = match plane {
        "XY" => ([1, 0, 0], [0, 1, 0], [0, 0, 1]),
        "XZ" => ([1, 0, 0], [0, 0, 1], [0, -1, 0]),
        "YZ" => ([0, 1, 0], [0, 0, 1], [1, 0, 0]),
        _ => return Err(invalid("plane must be XY, XZ or YZ")),
    };
    Ok(json!({ "origin": [0, 0, 0], "x": x, "y": y, "z": z }))
}

fn entity(document_id: &str, entity_id: &str, kind: &str) -> Value {
    json!({ "documentId": document_id, "entityId": entity_id, "kind": kind })
}

/// Keep confirmed effects when later read-only settling/inspection fails.
/// An unknown later outcome stays unknown, with the confirmed observations attached.
fn preserve_known(mut error: ApiError, known: Effects) -> ApiError {
    if error.effects.disposition == "none" { error.effects.disposition = known.disposition; }
    error.effects.documents.extend(known.documents);
    error.effects.external.extend(known.external);
    error.retry = "do_not_retry";
    error
}

async fn create(args: &Value, run: &Run<'_>) -> Result<Done, ApiError> {
    let a: CreateArgs = parse(args)?;
    check_cancel(run)?;
    (run.begin_commit)()?;
    (run.progress)("opening the document");
    let opened = call(&run.session, "doc_new", json!({ "title": a.title, "activate": false }), "commit").await?;
    let id = opened["context"]["documentId"].as_str().unwrap_or_default().to_string();
    if !id.is_ascii() || wire::handle_shape_doc(&id) == wire::HandleShape::Malformed {
        return Err(ApiError::new("reconciliation_required", "commit", "the host opened a document without a valid document identity").effects(Effects::unknown()));
    }
    (run.created_document)(&id);
    let known = || Effects { disposition:"committed",documents:vec![doc_effect(&id,None,None,true)],external:vec![] };
    // The new part's datum runs once; its landing moves the revision, so the
    // context handed back is the settled one.
    settle(run, &id).await.map_err(|e| preserve_known(e,known()))?;
    let t = tab(&run.session, &id).await.map_err(|e| preserve_known(e,known()))?;
    let rev = t["revision"].as_str().unwrap_or("0").to_string();
    Ok(Done {
        output: json!({ "context": context_json(&id, &rev), "title": t["title"], "lifecycle": "open", "dirty": t["dirty"] }),
        effects: Effects { disposition: "committed", documents: vec![doc_effect(&id, None, Some(&rev), true)], external: vec![] },
        geometry: geometry("not_applicable", "document opened"),
    })
}

async fn close(args: &Value, (id, expected): (String, String), run: &Run<'_>) -> Result<Done, ApiError> {
    let a: CloseArgs = parse(args)?;
    let t = fence(&run.session, &id, &expected).await?;
    if t["dirty"] == true && !a.discard_unsaved {
        return Err(ApiError::new("document_busy", "validation", "the document has unsaved changes; pass discardUnsaved: true").context(context_json(&id, &expected)));
    }
    check_cancel(run)?;
    (run.begin_commit)()?;
    call(&run.session, "doc_close", json!({ "id": id, "discard": a.discard_unsaved }), "commit").await?;
    Ok(Done {
        output: json!({ "documentId": id, "closed": true }),
        effects: Effects { disposition: "committed", documents: vec![doc_effect(&id, Some(&expected), None, true)], external: vec![] },
        geometry: geometry("not_applicable", "document closed"),
    })
}

/// Only the host's explicit stage acknowledgement proves commit or rollback.
/// Missing/contradictory replies cannot establish the absence of effects.
fn stage_committed(row: &Value) -> Result<bool, ApiError> {
    match (row["committed"].as_bool(), row["rolledBack"].as_bool(), row.get("pending").is_none_or(|v| *v == json!(false))) {
        (Some(true), Some(false), true) => Ok(true),
        (Some(false), Some(true), true) => Ok(false),
        _ => Err(ApiError::new("reconciliation_required", "evaluation", "the host did not confirm either stage commit or rollback").effects(Effects::unknown())),
    }
}

fn uncertain_stage(mut error: ApiError) -> ApiError {
    // After stage admission, a failed poll/cancel/lookup is not rollback proof.
    error.effects = Effects::unknown();
    error
}

/// One native item through the structured, rolled-back-on-error batch.
/// Returns the item's result row and the document's revision after it.
async fn author(run: &Run<'_>, id: &str, expected: &str, item: Value) -> Result<(Value, String), ApiError> {
    fence(&run.session, id, expected).await?;
    // The batch refuses while a run is still landing.
    settle(run, id).await?;
    let expected = fence(&run.session, id, expected).await?["revision"].as_str().unwrap_or("0").to_string();
    check_cancel(run)?;
    (run.progress)("evaluating");
    // Staged: the app appends the feature and returns at once; its run
    // evaluates on the runner while this polls with short requests, and the
    // stage ends committed or restored exactly. A seeding or validation
    // refusal answers before anything is written.
    match call(&run.session, "feature_stage", json!({ "item": item, "context": context_json(id, &expected) }), "evaluation").await {
        Ok(_) => {}
        Err(e) if e.code == "evaluation_failed" => {
            return Err(ApiError::new("invalid_arguments", "validation", e.message).context(context_json(id, &expected)));
        }
        Err(e) => return Err(e),
    }
    let platform = &run.session.host.platform;
    let mut braked = false;
    let row = loop {
        let r = call(&run.session, "feature_stage_result", json!({ "context": context_json(id, "0") }), "evaluation").await.map_err(uncertain_stage)?;
        if r["pending"] != true {
            break r;
        }
        // Cancelled while the geometry runs: pump the brakes with the app's
        // own run cancel — the Cancel button's path — and let the stage end
        // (it rolls back exactly, the run having produced nothing).
        if !braked && run.cancel.load(Ordering::Relaxed) {
            braked = true;
            (run.progress)("cancelling the evaluation");
            let rev = revision(&run.session, id).await.map_err(uncertain_stage)?;
            call(&run.session, "run_cancel", json!({ "context": context_json(id, &rev) }), "evaluation").await.map_err(uncertain_stage)?;
        }
        platform.sleep(50).await;
    };
    let committed = stage_committed(&row)?;
    let known = || Effects { disposition:if committed { "committed" } else { "none" }, documents:vec![doc_effect(id,Some(&expected),None,committed)],external:vec![] };
    if braked && !committed {
        let after = revision(&run.session, id).await.map_err(|e| preserve_known(e,known()))?;
        return Err(ApiError::new("cancelled", "evaluation", "cancelled: the evaluation was stopped and the change rolled back")
            .effects(Effects { disposition: "none", documents: vec![doc_effect(id, Some(&expected), Some(&after), false)], external: vec![] })
            .context(context_json(id, &after)));
    }
    settle(run, id).await.map_err(|e| preserve_known(e,known()))?;
    let after = revision(&run.session, id).await.map_err(|e| preserve_known(e,known()))?;
    if !committed {
        let why = row["error"].as_str().unwrap_or("the feature did not evaluate");
        // Rolled back: the recipe is as it was. A rollback still moves the
        // committed revision (the history was written and restored), so the
        // effects name it unchanged with the revision it is at now.
        return Err(ApiError::new("geometry_refused", "evaluation", wire::sanitize(why))
            .effects(Effects { disposition: "none", documents: vec![doc_effect(id, Some(&expected), Some(&after), false)], external: vec![] })
            .context(context_json(id, &after)));
    }
    Ok((row, after))
}

async fn sketch(args: &Value, (id, expected): (String, String), run: &Run<'_>) -> Result<Done, ApiError> {
    let a: SketchArgs = parse(args)?;
    let basis = plane_basis(&a.plane)?;
    let (w, h) = (a.profile.width_mm / 2.0, a.profile.height_mm / 2.0);
    let item = json!({
        "type": "S",
        "params": {},
        "persistent_data": {
            "basis": basis,
            "sketch": {
                "points": [
                    { "id": 0, "x": -w, "y": -h }, { "id": 1, "x": w, "y": -h },
                    { "id": 2, "x": w, "y": h }, { "id": 3, "x": -w, "y": h }
                ],
                "geometries": [
                    { "id": 10, "type": "line", "points": [0, 1] }, { "id": 11, "type": "line", "points": [1, 2] },
                    { "id": 12, "type": "line", "points": [2, 3] }, { "id": 13, "type": "line", "points": [3, 0] }
                ],
                "constraints": []
            }
        }
    });
    let (row, after) = author(run, &id, &expected, item).await?;
    let sketch_id = row["id"].as_str().unwrap_or_default().to_string();
    Ok(Done {
        output: json!({ "context": context_json(&id, &after), "sketch": entity(&id, &sketch_id, "sketch") }),
        effects: Effects { disposition: "committed", documents: vec![doc_effect(&id, Some(&expected), Some(&after), true)], external: vec![] },
        geometry: geometry("succeeded", format!("sketch {sketch_id} on {}", a.plane)),
    })
}

async fn extrude(args: &Value, (id, expected): (String, String), run: &Run<'_>) -> Result<Done, ApiError> {
    let a: ExtrudeArgs = parse(args)?;
    if a.sketch.document_id != id {
        return Err(invalid("the sketch belongs to another document"));
    }
    // The reference must name a sketch feature of this document now.
    let listing = call(&run.session, "history_listing", json!({ "context": context_json(&id, &expected) }), "validation").await?;
    let is_sketch = listing["listing"]
        .as_array()
        .or_else(|| listing["listing"]["features"].as_array())
        .is_some_and(|rows| rows.iter().any(|r| r["id"] == a.sketch.entity_id.as_str() && (r["type"] == "S" || r["shortName"] == "S")));
    if !is_sketch {
        return Err(invalid(format!("`{}` is not a sketch of this document (deleted, or another kind)", a.sketch.entity_id)));
    }
    let item = json!({
        "type": "E",
        "params": { "profile": a.sketch.entity_id, "distance": a.distance_mm, "distanceBack": 0 }
    });
    let (row, after) = author(run, &id, &expected, item).await?;
    let solid = row["generatedSolids"][0].as_str().map(str::to_string).ok_or_else(|| {
        ApiError::new("geometry_refused", "evaluation", "the extrude produced no solid")
            .effects(Effects { disposition: "partial", documents: vec![doc_effect(&id, Some(&expected), Some(&after), true)], external: vec![] })
    })?;
    Ok(Done {
        output: json!({ "context": context_json(&id, &after), "solid": entity(&id, &solid, "solid") }),
        effects: Effects { disposition: "committed", documents: vec![doc_effect(&id, Some(&expected), Some(&after), true)], external: vec![] },
        geometry: geometry("succeeded", format!("solid {solid}")),
    })
}

/// `cad.model.inspect`: an ordered read tagged with the current revision.
pub async fn inspect((id, expected): (String, String), session: &Session) -> Result<Value, ApiError> {
    let r = call(session, "model_inspect", json!({ "context": context_json(&id, &expected) }), "running").await?;
    let rev = r["context"]["expectedRevision"].as_str().unwrap_or("0");
    Ok(json!({
        "context": context_json(&id, rev),
        "solidCount": r["solidCount"],
        "valid": r["valid"],
        "boundsMm": r["boundsMm"],
        "volumeMm3": r["volumeMm3"],
        "warnings": r["warnings"],
    }))
}

async fn export(args: &Value, (id, expected): (String, String), run: &Run<'_>) -> Result<Done, ApiError> {
    let a: ExportArgs = parse(args)?;
    if a.entities.iter().any(|e| e.document_id != id) {
        return Err(invalid("every exported solid must belong to the context's document"));
    }
    fence(&run.session, &id, &expected).await?;
    settle(run, &id).await?;
    let model = call(&run.session, "model_inspect", json!({ "context": context_json(&id, &expected) }), "validation").await?;
    let names: std::collections::BTreeSet<&str> = model["solids"].as_array().into_iter().flatten().filter_map(|s| s["name"].as_str()).collect();
    for e in &a.entities {
        if !names.contains(e.entity_id.as_str()) {
            return Err(invalid(format!("`{}` is not a solid of this document", e.entity_id)));
        }
    }
    // Every solid named, in the caller's order; a strict subset writes only those.
    let wanted: Vec<&str> = a.entities.iter().map(|e| e.entity_id.as_str()).collect();
    check_cancel(run)?;
    (run.begin_commit)()?;
    (run.progress)("writing STEP");
    let r = call(&run.session, "doc_export", json!({ "format": "step", "solids": wanted, "context": context_json(&id, &expected) }), "external_io").await?;
    // The export is a snapshot of the admitted revision: if the document
    // moved under it, the bytes are of another model and are discarded.
    let at = r["context"]["expectedRevision"].as_str().unwrap_or("0");
    if at != expected {
        return Err(ApiError::new("revision_conflict", "external_io", format!("the document moved to revision {at} during the export")).context(context_json(&id, at)));
    }
    let text = r["text"].as_str().unwrap_or_default();
    if !text.starts_with("ISO-10303-21") {
        return Err(ApiError::new("external_io_failed", "external_io", "the STEP writer produced no STEP file"));
    }
    let artifact = run.artifacts.create(run.owner.clone(), "model/step", text.as_bytes().to_vec())?;
    Ok(Done {
        output: json!({ "context": context_json(&id, &expected), "artifact": artifact }),
        effects: Effects {
            disposition: "committed",
            documents: vec![doc_effect(&id, Some(&expected), Some(&expected), false)],
            external: vec![json!({ "kind": "artifact_created", "state": "completed", "artifactId": artifact["artifactId"] })],
        },
        geometry: geometry("succeeded", format!("{} of {} solid(s) written", wanted.len(), names.len())),
    })
}

async fn import(args: &Value, (id, expected): (String, String), run: &Run<'_>) -> Result<Done, ApiError> {
    let a: ImportArgs = parse(args)?;
    let (bytes, media) = run.artifacts.lease(&run.owner, &a.artifact_id)?;
    let result = import_leased(&a, bytes, &media, (&id, &expected), run).await;
    run.artifacts.release(&a.artifact_id);
    result
}

async fn import_leased(a: &ImportArgs, bytes: Vec<u8>, media: &str, (id, expected): (&str, &str), run: &Run<'_>) -> Result<Done, ApiError> {
    let read = json!({ "kind": "import_read", "state": "completed", "artifactId": a.artifact_id });
    // The format is read from the bytes, not trusted from the media type.
    if !bytes.starts_with(b"ISO-10303-21") {
        return Err(ApiError::new("invalid_arguments", "validation", format!("the artifact ({media}) is not a STEP file")).effects(Effects {
            disposition: "none",
            documents: vec![],
            external: vec![read],
        }));
    }
    let text = String::from_utf8(bytes).map_err(|_| {
        ApiError::new("invalid_arguments", "validation", "the STEP artifact is not text").effects(Effects { disposition: "none", documents: vec![], external: vec![read.clone()] })
    })?;
    (run.progress)("importing STEP");
    // Through the same rolled-back batch as sketch and extrude: an import
    // that evaluates to no solid leaves the recipe exactly as it was. (The
    // toolbar's import also lifts the file's AP242 PMI; this one does not.)
    let item = json!({ "type": "IMPORT3D", "params": { "stepText": text } });
    let (row, after) = author(run, id, expected, item).await.map_err(|mut e| {
        e.effects.external.push(read.clone());
        e
    })?;
    let solids: Vec<Value> = row["generatedSolids"].as_array().into_iter().flatten().filter_map(Value::as_str).map(|n| entity(id, n, "solid")).collect();
    let effects = |disposition| Effects { disposition, documents: vec![doc_effect(id, Some(expected), Some(&after), true)], external: vec![read.clone()] };
    if solids.is_empty() {
        return Err(ApiError::new("geometry_refused", "evaluation", "the STEP import produced no solid").effects(effects("partial")).context(context_json(id, &after)));
    }
    let model = call(&run.session, "model_inspect", json!({ "context": context_json(id, &after) }), "evaluation").await
        .map_err(|e| preserve_known(e,effects("committed")))?;
    if model["valid"] != true {
        return Err(ApiError::new("geometry_refused", "evaluation", "the imported model did not validate").effects(effects("committed")).context(context_json(id, &after)));
    }
    Ok(Done {
        output: json!({ "context": context_json(id, &after), "solids": solids }),
        effects: effects("committed"),
        geometry: geometry("succeeded", format!("{} solid(s) imported", solids.len())),
    })
}

