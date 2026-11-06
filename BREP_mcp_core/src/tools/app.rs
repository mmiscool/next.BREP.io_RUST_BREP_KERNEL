//! The adapter from the app's command registry to tools: every command the
//! running app describes becomes a [`ToolSpec`] whose handler submits the
//! envelope through the session's host. Commands whose annotations say
//! `waits` get the idle contract applied before the reply is returned.
//! Nothing here names a command; the app's `describe()` is the source.
use crate::session::Session;
use crate::tools::{Annotations, ToolImage, ToolOutput, ToolSpec};
use serde_json::{json, Value};
use std::sync::Arc;
use tokio::sync::RwLock;

pub type SessionSlot = Arc<RwLock<Option<Arc<Session>>>>;

/// Tool names the server implements itself (compositions and file halves);
/// the app command of the same name is not exposed twice.
pub const SERVER_OWNED: &[&str] = &["files", "illustration_presentation", "screenshot", "feature_add", "feature_add_many", "feature_set_params", "doc_import", "doc_export", "doc_load", "cam_post_text"];

/// The server-owned tool that WRAPS an app command under a different name.
///
/// The file halves are the only renamed ones: the app command takes or returns
/// content, and the server tool takes or returns a PATH (it does the reading and
/// writing). Anything a panel documents as its equivalent app command has to be
/// translated through this before it is published as a tool name, or the docs
/// name something `tools/call` will refuse. `SERVER_OWNED` entries missing here
/// keep their name.
pub const SERVER_TOOL_FOR: &[(&str, &str)] = &[
    ("doc_load", "document_open"),
    ("doc_json", "document_save"),
    ("doc_import", "document_import"),
    ("doc_export", "document_export"),
    ("cam_post_text", "cam_post"),
];

/// The tool a caller actually invokes for `command` — itself, unless the server
/// wraps it under another name.
pub fn tool_name_for(command: &str) -> &str {
    SERVER_TOOL_FOR
        .iter()
        .find(|(app, _)| *app == command)
        .map(|(_, tool)| *tool)
        .unwrap_or(command)
}

pub async fn current(slot: &SessionSlot) -> Result<Arc<Session>, String> {
    slot.read().await.clone().ok_or_else(|| "no live session: run_script starts one".to_string())
}

/// The longest one `settle` call waits: a host bounds how long a single
/// command may take (the desktop window's reply timeout), so a longer wait is
/// several calls.
const SETTLE_CHUNK_MS: u64 = 60_000;

/// Wait in the app (`settle`) until it is idle plus `settle` checks, or the
/// timeout. Returns the settled reply (`info`: the last frame_info, plus the
/// history listing, report and model), or `None` from a host that has no
/// `settle` command.
async fn settle_in_app(session: &Session, timeout_ms: u64, settle: u32) -> Result<Option<Value>, String> {
    let platform = &session.host.platform;
    let start = platform.now_ms();
    loop {
        let elapsed = (platform.now_ms() - start).max(0.0) as u64;
        let left = timeout_ms.saturating_sub(elapsed);
        let chunk = left.clamp(1, SETTLE_CHUNK_MS);
        let reply = session.host.call("settle", json!({ "settle": settle, "timeout_ms": chunk })).await?;
        if reply.ok {
            return Ok(Some(reply.result.unwrap_or(Value::Null)));
        }
        let error = reply.error.unwrap_or_else(|| "command failed".into());
        if error.starts_with("unknown command") {
            return Ok(None);
        }
        if let Some(rest) = error.strip_prefix(&format!("timeout after {chunk} ms")) {
            if chunk < left {
                continue;
            }
            return Err(format!("timeout after {timeout_ms} ms{rest}"));
        }
        return Err(error);
    }
}

/// Wait until the runner is idle plus `settle` checks, or the timeout.
/// Returns the last frame_info.
pub async fn wait_idle(session: &Session, timeout_ms: u64, settle: u32) -> Result<Value, String> {
    if let Some(settled) = settle_in_app(session, timeout_ms, settle).await? {
        return Ok(settled["info"].clone());
    }
    poll_idle(session, timeout_ms, settle).await
}

/// [`wait_idle`] for a host without `settle`: poll `frame_info`.
async fn poll_idle(session: &Session, timeout_ms: u64, settle: u32) -> Result<Value, String> {
    let platform = &session.host.platform;
    let start = platform.now_ms();
    let mut idle_seen = 0u32;
    loop {
        let r = session.host.call_ok("frame_info", json!({})).await?;
        let info = r.result.clone().unwrap_or(Value::Null);
        if info["poisoned"].as_bool() == Some(true) {
            return Err("session poisoned".into());
        }
        if info["idle"].as_bool() == Some(true) {
            idle_seen += 1;
            if idle_seen > settle {
                return Ok(info);
            }
        } else {
            idle_seen = 0;
            if platform.now_ms() - start > timeout_ms as f64 {
                // NAME the work still in flight, the way the headless host's
                // own settle does: "still pending: mesh_imports" says which
                // part of the app is stuck, where a bare timeout reads as a
                // slow machine (see the settle record, 2026-09-21).
                return Err(format!(
                    "timeout after {timeout_ms} ms; still pending: {}; run progress: {}",
                    pending_names(&info["pending"]),
                    info["progress"]
                ));
            }
            platform.sleep(30).await;
        }
    }
}

/// The `pending` block of a `frame_info` reply as a comma-separated list of
/// the flags that are true — what a timeout says it was still waiting for.
fn pending_names(pending: &Value) -> String {
    let names: Vec<&str> = pending
        .as_object()
        .map(|o| o.iter().filter(|(_, v)| v.as_bool() == Some(true)).map(|(k, _)| k.as_str()).collect())
        .unwrap_or_default();
    if names.is_empty() {
        "nothing, but the app had not been idle long enough".to_string()
    } else {
        names.join(", ")
    }
}

/// The report every waiting tool appends: listing, report, step, model summary.
pub async fn after_wait(session: &Session, timeout_ms: u64) -> Result<Value, String> {
    // One call: the app answers when it has settled, with the listing.
    if let Some(settled) = settle_in_app(session, timeout_ms, 2).await? {
        return Ok(json!({
            "frame": settled["frame"],
            "step": settled["step"],
            "listing": settled["listing"],
            "report": settled["report"],
            "model": settled["model"],
        }));
    }
    let start = session.host.platform.now_ms();
    loop {
        let elapsed = (session.host.platform.now_ms() - start).max(0.0) as u64;
        if elapsed >= timeout_ms { return Err("deadline_exceeded: the completed history snapshot did not settle".into()); }
        poll_idle(session, timeout_ms - elapsed, 2).await?;
        let snapshot = session.host.call_ok("history_listing", json!({})).await?;
        let listing = snapshot.result.unwrap_or(Value::Null);
        if listing["idle"] == false { continue; }
        // Current hosts publish model and report in the same app read phase.
        let model = if let Some(model) = listing.get("model") { model.clone() } else {
            session.host.call("state_get", json!({ "names": ["__brepModel"] })).await.ok().and_then(|r| r.result).map(|v| v["__brepModel"].clone()).unwrap_or(Value::Null)
        };
        return Ok(json!({
            "frame": snapshot.frame,
            "step": listing["step"],
            "listing": listing["listing"],
            "report": listing["report"],
            "model": model,
        }));
    }
}

/// Keep legacy top-level run fields aligned with the completed operation.
/// Submission-only replies are left alone; `after` remains for existing clients.
pub(super) fn settled_result(result: &mut Value, after: Value) {
    // Only legacy evaluation receipts carry both report and step. Domain
    // reports (inference and replacement) must survive settlement unchanged.
    if result.get("report").is_some() && result.get("step").is_some() {
        result["report"] = after["report"].clone();
        result["step"] = after["step"].clone();
        if result.get("history_len").is_some() {
            if let Some(listing) = after["listing"]["features"].as_array() {
                result["history_len"] = json!(listing.len());
            }
        }
    }
    if result.get("frame").is_some() { result["frame"] = after["frame"].clone(); }
    result["after"] = after;
}

fn annotations_of(desc: &Value) -> Annotations {
    let a = &desc["annotations"];
    Annotations {
        read_only: a["read_only"].as_bool().unwrap_or(false),
        destructive: a["destructive"].as_bool().unwrap_or(false),
        idempotent: a["idempotent"].as_bool().unwrap_or(false),
        waits: a["waits"].as_bool().unwrap_or(false),
    }
}

/// Add the fields written by the native tool adapter without discarding the
/// payload's dictionary value schema or opening closed union branches. Only
/// schemas for this object are extended; nested payload properties and $defs
/// describe different values and must remain untouched.
fn native_output_schema(mut schema: Value, waits: bool) -> Value {
    let mut envelope = json!({
        "frame":{"type":"integer","minimum":0},
        "notices":{"type":"array","items":{
            "type":"object",
            "properties":{
                "kind":{"type":"string","enum":["panic","log_error","runner_refusal","toast","console"]},
                "frame":{"type":"integer","minimum":0},"text":{"type":"string"}
            },
            "required":["kind","frame","text"],"additionalProperties":false
        }}
    });
    if waits {
        envelope["after"] = super::compose::after_result_schema();
    }
    fn extend(schema: &mut Value, envelope: &Value) {
        // The runtime preserves object payloads and wraps every non-object
        // payload under `value`. A declared null alternative must describe
        // that envelope rather than an impossible null with required frame.
        if matches!(schema.get("type").and_then(Value::as_str), Some("null" | "array" | "string" | "number" | "integer" | "boolean")) {
            let mut payload = schema.take();
            let definitions = payload.as_object_mut().unwrap().remove("$defs");
            *schema = json!({"type":"object","properties":{"value":payload},"required":["value"],"additionalProperties":false});
            if let Some(definitions) = definitions { schema["$defs"] = definitions; }
        }
        let Some(object) = schema.as_object_mut() else { return };
        // Combinators apply their branches to the same response object. A
        // closed branch needs the same explicit envelope properties as root.
        for keyword in ["oneOf", "anyOf", "allOf"] {
            if let Some(branches) = object.get_mut(keyword).and_then(Value::as_array_mut) {
                for branch in branches { extend(branch, envelope); }
            }
        }
        for keyword in ["then", "else"] {
            if let Some(branch) = object.get_mut(keyword) { extend(branch, envelope); }
        }
        let properties = object.entry("properties").or_insert_with(|| json!({}));
        if let Some(properties) = properties.as_object_mut() {
            for (name, value) in envelope.as_object().unwrap() {
                // frame is always overwritten by the adapter. Native notices
                // and after can remain when the adapter has none to add.
                if name == "frame" || !properties.contains_key(name) {
                    properties.insert(name.clone(), value.clone());
                }
            }
        }
        let required = object.entry("required").or_insert_with(|| json!([]));
        if let Some(required) = required.as_array_mut() {
            if !required.iter().any(|name| name == "frame") { required.push(json!("frame")); }
        }
    }
    extend(&mut schema, &envelope);
    // MCP output schemas describe the adapter's object, including when the
    // native declaration expresses its alternatives without a root type.
    schema["type"] = json!("object");
    schema
}

fn supported_native_output(schema: &Value) -> bool {
    if matches!(schema.get("type").and_then(Value::as_str), Some("object" | "null" | "array" | "string" | "number" | "integer" | "boolean")) {
        return true;
    }
    ["oneOf", "anyOf"].iter().any(|keyword| {
        schema.get(keyword).and_then(Value::as_array).is_some_and(|branches| {
            !branches.is_empty() && branches.iter().all(supported_native_output)
        })
    })
}

/// Build the tool list from what the app describes. `describe` is the JSON
/// `brep_app::automation::command::describe()` returns (read once per session
/// through the host, so a remote app's registry is honoured too).
pub fn app_tools(slot: SessionSlot, describe: &Value) -> Vec<ToolSpec> {
    let mut out = Vec::new();
    for desc in describe.as_array().cloned().unwrap_or_default() {
        let name = desc["name"].as_str().unwrap_or("").to_string();
        if name.is_empty() || SERVER_OWNED.contains(&name.as_str()) {
            continue;
        }
        let group: &'static str = Box::leak(desc["group"].as_str().unwrap_or("app").to_string().into_boxed_str());
        let doc = desc["doc"].as_str().unwrap_or("").to_string();
        let ann = annotations_of(&desc);
        let mut schema = desc["argsSchema"].clone();
        if ann.waits {
            // Every waiting tool takes the idle-contract knobs. A command that
            // takes nothing has no `properties` block at all (`doc_new`); the
            // knobs still need somewhere to go, or the tool waits by default
            // without saying it can be told not to.
            if let Some(o) = schema.as_object_mut() {
                o.entry("properties").or_insert_with(|| json!({}));
            }
            if let Some(props) = schema.get_mut("properties").and_then(Value::as_object_mut) {
                props.insert("wait".into(), json!({ "type": "boolean", "default": true, "description": "wait for the runner to go idle before replying" }));
                props.insert("timeout_ms".into(), json!({ "type": "integer", "default": 60000 }));
            }
        }
        let output_schema = desc["resultSchema"].clone();
        let slot = slot.clone();
        let cmd = name.clone();
        out.push(ToolSpec::new(name, group, doc, schema, ann, move |args| {
            let slot = slot.clone();
            let cmd = cmd.clone();
            Box::pin(async move {
                let session = current(&slot).await?;
                let mut args = args;
                let (wait, timeout_ms) = if ann.waits {
                    let o = args.as_object_mut();
                    let wait = o.as_ref().and_then(|o| o.get("wait")).and_then(Value::as_bool).unwrap_or(true);
                    let t = o.as_ref().and_then(|o| o.get("timeout_ms")).and_then(Value::as_u64).unwrap_or(60_000);
                    if let Some(o) = o {
                        o.remove("wait");
                        o.remove("timeout_ms");
                    }
                    (wait, t)
                } else {
                    (false, 0)
                };
                let reply = session.host.call(&cmd, args.clone()).await?;
                let raw = if reply.ok {
                    reply.result.clone().unwrap_or(Value::Null)
                } else {
                    let e = reply.error.clone().unwrap_or_else(|| "command failed".into());
                    session.record(&cmd, &args, false, json!({ "error": e }));
                    return Err(e);
                };
                // A reply that is not an object (describe_commands is an array)
                // is wrapped so the envelope fields have somewhere to go.
                let mut result = if raw.is_object() { raw } else { json!({ "value": raw }) };
                if !reply.notices.is_empty() {
                    result["notices"] = serde_json::to_value(&reply.notices).unwrap_or(Value::Null);
                }
                result["frame"] = json!(reply.frame);
                if wait {
                    settled_result(&mut result, after_wait(&session, timeout_ms).await?);
                }
                session.record(&cmd, &args, true, json!({ "frame": reply.frame }));
                let mut out = ToolOutput::json(result);
                if let Some(png) = reply.blob {
                    out.images.push(ToolImage { png, mime: "image/png" });
                }
                Ok(out)
            })
        }));
        if supported_native_output(&output_schema) {
            out.last_mut().unwrap().output_schema = native_output_schema(output_schema, ann.waits);
        }
    }
    out
}

