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
pub const SERVER_OWNED: &[&str] = &["illustration_presentation", "screenshot", "feature_add", "feature_add_many", "feature_set_params", "doc_import", "doc_export", "doc_load"];

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
    slot.read().await.clone().ok_or_else(|| "no live session: call session_start first".to_string())
}

/// Poll `frame_info` until the runner is idle plus `settle` frames, or the
/// timeout. Returns the last frame_info.
pub async fn wait_idle(session: &Session, timeout_ms: u64, settle: u32) -> Result<Value, String> {
    let start = std::time::Instant::now();
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
            if start.elapsed().as_millis() as u64 > timeout_ms {
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
            tokio::time::sleep(std::time::Duration::from_millis(30)).await;
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
    let info = wait_idle(session, timeout_ms, 2).await?;
    let listing = session.host.call_ok("history_listing", json!({})).await?.result.unwrap_or(Value::Null);
    let model = session.host.call("state_get", json!({ "names": ["__brepModel"] })).await.ok().and_then(|r| r.result).map(|v| v["__brepModel"].clone());
    Ok(json!({
        "frame": info["frame"],
        "step": listing["step"],
        "listing": listing["listing"],
        "report": listing["report"],
        "model": model,
    }))
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
            // Every waiting tool takes the idle-contract knobs.
            if let Some(props) = schema.get_mut("properties").and_then(Value::as_object_mut) {
                props.insert("wait".into(), json!({ "type": "boolean", "default": true, "description": "wait for the runner to go idle before replying" }));
                props.insert("timeout_ms".into(), json!({ "type": "integer", "default": 60000 }));
            }
        }
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
                    result["after"] = after_wait(&session, timeout_ms).await?;
                }
                session.record(&cmd, &args, true, json!({ "frame": reply.frame }));
                let mut out = ToolOutput::json(result);
                if let Some(png) = reply.blob {
                    out.images.push(ToolImage { png, mime: "image/png" });
                }
                Ok(out)
            })
        }));
    }
    out
}
