//! The MCP front door (`run_script` and its five companions) and the one
//! dispatcher every `cad.*` method goes through — a code-mode program's
//! bridge and the app's own editor alike.
//!
//! The transport admits a request only with the loopback credential (D3,
//! `http::Credential`; stdio is its launching process's own), so every
//! admitted client is the trusted local caller ([`caller_of_transport`]).
use super::ops::Caller;
use super::wire::{self, ApiError};
use super::CadService;
use crate::tools::{Annotations, ToolOutput, ToolSpec};
use serde_json::{json, Value};
use std::sync::Arc;

/// Who an admitted MCP request is: the local user, whose token the HTTP
/// transport checked (`http::Credential`) or whose process owns stdio.
pub fn caller_of_transport() -> Caller {
    Caller::local()
}

fn string<'a>(req: &'a Value, key: &str) -> Result<&'a str, ApiError> {
    req.get(key).and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", format!("{key} is required")))
}

fn only(req: &Value, allowed: &[&str]) -> Result<(), ApiError> {
    let obj = req.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "the request must be an object"))?;
    if let Some(k) = obj.keys().find(|k| !allowed.contains(&k.as_str())) {
        let takes = if allowed.is_empty() { "it takes no fields".to_string() } else { format!("it takes {{{}}}", allowed.join(", ")) };
        return Err(ApiError::new("invalid_arguments", "validation", format!("unknown field `{k}`; {takes}")));
    }
    Ok(())
}

/// `describe`'s paths: `paths` (an array), or one path or an array of them
/// under the names agents reach for first (`path`, `name`, `method`, `tool`).
fn describe_paths(req: &Value) -> Result<Vec<String>, ApiError> {
    const NAMES: &[&str] = &["paths", "path", "name", "names", "method", "tool"];
    let obj = req.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "the request must be an object"))?;
    if let Some(k) = obj.keys().find(|k| !NAMES.contains(&k.as_str())) {
        return Err(ApiError::new("invalid_arguments", "validation", format!("unknown field `{k}`; describe takes {{paths: string[]}}, e.g. cad.discovery.describe({{paths: [\"brep.features.feature_add\"]}})")));
    }
    let given: Vec<&Value> = NAMES.iter().filter_map(|k| obj.get(*k)).collect();
    let [value] = given[..] else {
        return Err(ApiError::new("invalid_arguments", "validation", "describe takes {paths: string[]}, e.g. cad.discovery.describe({paths: [\"brep.features.feature_add\"]})"));
    };
    let items: Vec<&Value> = match value { Value::Array(a) => a.iter().collect(), v => vec![v] };
    items.into_iter()
        .map(|p| p.as_str().map(str::to_string).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "paths holds strings: exact paths such as brep.features.feature_add or cad.sketch.create")))
        .collect()
}

/// Every logical service method, by its exact name.
pub async fn call_service(service: &CadService, caller: &Caller, method: &str, req: &Value) -> Result<Value, ApiError> {
    if req.to_string().len() > 1 << 20 {
        return Err(ApiError::new("resource_limit", "validation", "the request is larger than 1 MiB"));
    }
    match method {
        "cad.inputs.exportOptions" => service.code.export_options(caller, req).await,
        "cad.discovery.search" => {
            caller.require(&["catalogue.read"])?;
            only(req, &["query", "limit", "cursor"])?;
            service.search(req).await
        }
        "cad.discovery.describe" => {
            caller.require(&["catalogue.read"])?;
            let paths = describe_paths(req)?;
            service.describe(&paths).await
        }
        // The guides and declarations, for a client that cannot read MCP
        // resources: tools-only runs never saw skill://design-part, and
        // guessed the API one failed call at a time.
        "cad.docs.list" => {
            only(req, &[])?;
            Ok(json!({ "guides": crate::skills::SKILLS.iter().map(|s| json!({ "name": s.name, "description": s.description })).collect::<Vec<_>>() }))
        }
        "cad.docs.read" => {
            only(req, &["name"])?;
            let name = string(req, "name")?;
            let name = name.strip_prefix("skill://").unwrap_or(name);
            let name = name.strip_suffix("/SKILL.md").unwrap_or(name);
            let skill = crate::skills::SKILLS.iter().find(|s| s.name == name).ok_or_else(|| {
                let names: Vec<&str> = crate::skills::SKILLS.iter().map(|s| s.name).collect();
                ApiError::new("invalid_arguments", "lookup", format!("no guide `{name}`; the guides are {}", names.join(", ")))
            })?;
            Ok(json!({ "name": skill.name, "text": skill.text }))
        }
        "cad.docs.sdk" => {
            only(req, &["group"])?;
            service.sdk_declarations(req.get("group").and_then(Value::as_str)).await
        }
        "cad.documents.list" => {
            only(req, &[])?;
            service.ops.documents(caller, None).await
        }
        "cad.documents.get" => {
            only(req, &["documentId"])?;
            service.ops.documents(caller, Some(string(req, "documentId")?)).await
        }
        "cad.operations.issueKey" => service.ops.issue_key(caller, req),
        "cad.operations.submit" => service.ops.submit(caller, req),
        "cad.operations.status" => service.ops.status(caller, req),
        "cad.operations.result" => service.ops.result(caller, req),
        "cad.operations.cancel" => service.ops.cancel(caller, req),
        "cad.commands.query" => service.ops.query(caller, req).await,
        "cad.code.submit" => service.code.submit(caller, req),
        "cad.code.status" => service.code.status(caller, req),
        "cad.code.result" => service.code.result(caller, req),
        "cad.code.cancel" => service.code.cancel(caller, req),
        "cad.artifacts.read" => {
            caller.require(&["artifact.read"])?;
            only(req, &["artifactId", "offset", "limit"])?;
            let offset = req.get("offset").map_or(Some(0), Value::as_u64).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "offset must be a non-negative integer"))?;
            let limit = match req.get("limit") {
                None => None,
                Some(v) => Some(v.as_u64().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "limit must be a positive integer"))? as usize),
            };
            service.artifacts.read(&caller.owner(), string(req, "artifactId")?, offset as usize, limit)
        }
        "cad.artifacts.create" => {
            caller.require(&["artifact.write"])?;
            only(req, &["mediaType", "bytesBase64", "path"])?;
            if string(req, "mediaType")? != "model/step" {
                return Err(ApiError::new("invalid_arguments", "validation", "mediaType must be model/step"));
            }
            // Either the bytes, or (for the local user's run_script) a file
            // on the host: the way a large input reaches a program.
            let bytes = match (req.get("bytesBase64"), req.get("path").and_then(Value::as_str)) {
                (None, Some(path)) => {
                    caller.require(&["host.files.read"])?;
                    let len = std::fs::metadata(path).map_err(|e| ApiError::new("invalid_arguments", "external_io", format!("{path}: {e}")))?.len();
                    if len > super::artifacts::MAX_ARTIFACT as u64 {
                        return Err(ApiError::new("resource_limit", "external_io", format!("{path} is larger than {} bytes", super::artifacts::MAX_ARTIFACT)));
                    }
                    std::fs::read(path).map_err(|e| ApiError::new("invalid_arguments", "external_io", format!("{path}: {e}")))?
                }
                (Some(_), None) => {
                    use base64::Engine as _;
                    base64::engine::general_purpose::STANDARD
                        .decode(string(req, "bytesBase64")?)
                        .map_err(|e| ApiError::new("invalid_arguments", "validation", format!("bytesBase64: {e}")))?
                }
                _ => return Err(ApiError::new("invalid_arguments", "validation", "give exactly one of bytesBase64 and path")),
            };
            service.artifacts.create(caller.owner(), "model/step", bytes)
        }
        // The streaming route for an input above the 1 MiB transport limit
        // (contract §12): begin with the length and digest, append chunks
        // in order, commit — the artifact exists only once it all matches.
        "cad.artifacts.begin" => {
            caller.require(&["artifact.write"])?;
            only(req, &["mediaType", "byteLength", "sha256"])?;
            if string(req, "mediaType")? != "model/step" {
                return Err(ApiError::new("invalid_arguments", "validation", "mediaType must be model/step"));
            }
            let length = req.get("byteLength").and_then(Value::as_u64).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "byteLength is required"))?;
            service.artifacts.begin(caller.owner(), "model/step", length as usize, string(req, "sha256")?)
        }
        "cad.artifacts.append" => {
            caller.require(&["artifact.write"])?;
            only(req, &["uploadId", "offset", "bytesBase64"])?;
            let offset = req.get("offset").and_then(Value::as_u64).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "offset is required"))?;
            use base64::Engine as _;
            let bytes = base64::engine::general_purpose::STANDARD
                .decode(string(req, "bytesBase64")?)
                .map_err(|e| ApiError::new("invalid_arguments", "validation", format!("bytesBase64: {e}")))?;
            service.artifacts.append(&caller.owner(), string(req, "uploadId")?, offset as usize, bytes)
        }
        "cad.artifacts.commit" => {
            caller.require(&["artifact.write"])?;
            only(req, &["uploadId"])?;
            service.artifacts.commit(&caller.owner(), string(req, "uploadId")?)
        }
        other => Err(ApiError::new("method_unknown", "lookup", format!("no method `{other}`"))),
    }
}

/// The MCP's front door: the only tools a client lists and calls.
/// Everything else the app can do is a script function,
/// `brep.<group>.<tool>(args)`, and the `cad.*` service is the script's
/// `cad` client.
pub const FRONT_DOOR: &[&str] = &["run_script", "script_result", "script_halt", "screenshot", "input", "files"];

/// Retry keys `run_script` remembers; past this many the map starts again
/// (an issued key expires within minutes anyway).
const RETRY_KEYS: usize = 4096;

fn retry_key(keys: &mut std::collections::HashMap<String, String>, caller: &Caller, key: &str) -> Result<String, String> {
    if let Some(issued) = keys.get(key) { return Ok(issued.clone()); }
    if keys.len() >= RETRY_KEYS { return Err("retry key capacity reached; existing keys remain valid; reconcile work before restarting".into()); }
    let issued = super::keys::issue(&caller.scope, super::keys::Purpose::Execution).0;
    keys.insert(key.into(), issued.clone());
    Ok(issued)
}

/// Await the result when no budget is supplied; an explicit budget returns
/// status if the execution is still running at its deadline.
async fn settle_execution(service: &CadService, caller: &Caller, id: &str, wait_ms: Option<u64>) -> Value {
    let deadline = wait_ms.map(|ms| std::time::Instant::now() + std::time::Duration::from_millis(ms));
    loop {
        match service.code.result(caller, &json!({ "executionId": id })) {
            Ok(result) => return wire::ok(result),
            Err(e) if e.code == "not_ready" && deadline.is_none_or(|d| std::time::Instant::now() < d) => {
                tokio::time::sleep(std::time::Duration::from_millis(25)).await;
            }
            Err(e) if e.code == "not_ready" => {
                return wire::respond(service.code.status(caller, &json!({ "executionId": id })).map(|s| json!({ "running": true, "status": s })));
            }
            Err(e) => return wire::fail(&e),
        }
    }
}

/// Start the headless session if none is live, and wait for the app's tools
/// to register (the session's registry is read asynchronously).
pub(super) async fn ensure_session(service: &CadService, slot: &crate::tools::app::SessionSlot) -> Result<(), String> {
    let Some(state) = service.server.get().and_then(std::sync::Weak::upgrade) else { return Ok(()) };
    let _starting = state.session_start.lock().await;
    if slot.read().await.is_some() { return Ok(()); }
    let start = state.tools.read().await.get("session_start").cloned();
    if let Some(start) = start {
        (start.handler)(json!({ "backend": "headless", "seed": true, "record": false })).await?;
        crate::server::BrepServer { state: state.clone() }.rebuild_tools().await;
    }
    Ok(())
}

/// The front door (group `script`): `run_script`, `script_result`,
/// `script_halt`, `screenshot` (the capture tool, a session started first)
/// `input` and `files`.
pub fn front_door_tools(service: Arc<CadService>, slot: crate::tools::app::SessionSlot) -> Vec<ToolSpec> {
    let screenshot = crate::tools::compose::capture_tools(slot.clone()).into_iter().find(|t| t.name == "screenshot").expect("capture registry includes screenshot");
    let wait = json!({ "type": "integer", "minimum": 0, "maximum": 600000, "description": "Optional wait budget in ms. run_script awaits completion when omitted; script_result defaults to 0. An explicit budget returns status if work is still running." });
    let run = service.clone();
    let result = service.clone();
    let halt = service.clone();
    let input_slot = slot.clone();
    let shot = service.clone();
    let shot_slot = slot.clone();
    let files_service = service.clone();
    let mut tools = vec![
        ToolSpec::new(
            "files", "script", "Manage files under named roots: list/stat/read/write/mkdir/copy/move/delete. Paths are relative. Writes/copies/moves refuse replacement unless overwrite:true; directory copy/delete requires recursive:true. documents/workspace use the app store; artifacts uses the execution area (direct calls can address execution-id subdirectories); BREP_MCP_FILE_ROOTS configures extra native roots. Read/write transfer limit256KiB, page limit256, directory/tree limit1024 entries. This aggregate tool can perform destructive operations; list/stat/read do not modify files.",
            crate::files::input_schema(), Annotations::DESTRUCTIVE,
            move |args| {
                let service = files_service.clone();
                Box::pin(async move { service.files.call(&service, &Caller::local(), args).await })
            },
        ),
        ToolSpec::new(
            "screenshot",
            "script",
            "A PNG of the app as it is now (region: full | viewport | {x,y,w,h}, egui points). Starts a headless session if none is live.",
            screenshot.input_schema,
            Annotations { read_only: false, destructive: true, idempotent: false, waits: false },
            move |args| {
                let service = shot.clone();
                let slot = shot_slot.clone();
                Box::pin(async move {
                    ensure_session(&service, &slot).await?;
                    let state = service.server.get().and_then(std::sync::Weak::upgrade).ok_or("the server is stopping")?;
                    let capture = state.tools.read().await.specs.iter().find(|t| t.name == "screenshot" && t.group == "capture").cloned();
                    let capture = capture.ok_or("no capture tool: the session has no app")?;
                    (capture.handler)(args).await
                })
            },
        ),
        ToolSpec::new(
            "run_script",
            "script",
            "Run a JavaScript (or TypeScript) program on the CAD app: `export default async function main()` returning JSON. Every app tool is a script function, `await brep.<group>.<tool>(args)`, beside the `cad` client of `@brep/cad`. Before modelling, return `await cad.docs.read('design-part')` once: it is the modelling guide; `cad.docs.sdk(group)` gives a group's typed signatures. Awaits the completed result on this same tool call by default. An explicit wait_ms opts into a bounded wait/status response; retrying the same source and key recovers that same execution. script_result remains an optional recovery tool; script_halt cancels work. Starts a headless session if none is live.",
            json!({ "type": "object", "properties": { "source": { "type": "string", "minLength": 1, "maxLength": 262144, "description": "UTF-8 source, at most 256 KiB" }, "language": { "enum": ["javascript", "typescript"], "default": "javascript", "description": "TypeScript requires explicitly provisioned Node and TypeScript 7.0.2; tools/list reports this host's compiler availability. Strict checking precedes execution." }, "wait_ms": wait, "key": { "type": "string", "minLength": 1, "maxLength": 128, "description": "a retry key of your choosing: the same key with the same program answers the SAME execution (submitted once), so a run whose reply was lost is found again; the same key with another program is refused" } }, "required": ["source"], "additionalProperties": false }),
            Annotations { read_only: false, destructive: true, idempotent: false, waits: false },
            move |args| {
                let service = run.clone();
                let slot = slot.clone();
                Box::pin(async move {
                    let caller = caller_of_transport();
                    // A client's retry key stands for one issued key, so a
                    // resubmission binds to the execution it already started.
                    let key = match args.get("key").and_then(Value::as_str) {
                        Some(k) if k.is_empty() || k.len() > 128 => return Err("key must be 1 to 128 characters".into()),
                        Some(_) if service.code.persistent() => super::keys::issue(&caller.scope, super::keys::Purpose::Execution).0,
                        Some(k) => {
                            let mut keys = service.retry_keys.lock().unwrap();
                            retry_key(&mut keys, &caller, k)?
                        }
                        None => super::keys::issue(&caller.scope, super::keys::Purpose::Execution).0,
                    };
                    let req = json!({
                        "manifest": service.catalogue.identity(),
                        "language": args.get("language").and_then(Value::as_str).unwrap_or("javascript"),
                        "source": args.get("source").and_then(Value::as_str).unwrap_or_default(),
                        "initialDocuments": [],
                        "requestedCapabilities": [],
                        "idempotencyKey": key,
                    });
                    let retry = args.get("key").and_then(Value::as_str);
                    let replay = match service.code.replay(&caller, &req, true, retry) {
                        Ok(status) => status,
                        Err(e) => return Ok(ToolOutput::json(wire::fail(&e))),
                    };
                    let submitted = if let Some(status) = replay { Ok(status) } else {
                        ensure_session(&service, &slot).await?;
                        service.code.submit_with_retry(&caller, &req, true, retry)
                    };
                    let status = match submitted {
                        Ok(s) => s,
                        Err(e) => return Ok(ToolOutput::json(wire::fail(&e))),
                    };
                    let id = status["executionId"].as_str().unwrap_or_default().to_string();
                    let wait_ms = args.get("wait_ms").and_then(Value::as_u64).map(|ms| ms.min(600_000));
                    Ok(ToolOutput::json(settle_execution(&service, &caller, &id, wait_ms).await))
                })
            },
        ),
        ToolSpec::new(
            "script_result",
            "script",
            "A script's result once it has finished (waiting up to wait_ms, default 0), else its status.",
            json!({ "type": "object", "properties": { "executionId": { "type": "string" }, "wait_ms": wait }, "required": ["executionId"], "additionalProperties": false }),
            Annotations { read_only: true, destructive: false, idempotent: true, waits: false },
            move |args| {
                let service = result.clone();
                Box::pin(async move {
                    let caller = caller_of_transport();
                    let id = args.get("executionId").and_then(Value::as_str).unwrap_or_default().to_string();
                    let wait_ms = args.get("wait_ms").and_then(Value::as_u64).unwrap_or(0).min(600_000);
                    Ok(ToolOutput::json(settle_execution(&service, &caller, &id, Some(wait_ms)).await))
                })
            },
        ),
        ToolSpec::new(
            "script_halt",
            "script",
            "Halt a running script now: its worker is killed wherever it is. CAD operations it had already started run on (cancel one with its own cancel).",
            json!({ "type": "object", "properties": { "executionId": { "type": "string" } }, "required": ["executionId"], "additionalProperties": false }),
            Annotations { read_only: false, destructive: false, idempotent: true, waits: false },
            move |args| {
                let service = halt.clone();
                Box::pin(async move {
                    let caller = caller_of_transport();
                    Ok(ToolOutput::json(wire::respond(service.code.cancel(&caller, &args))))
                })
            },
        ),
        ToolSpec::new(
            "input",
            "script",
            "Inject mouse and keyboard input into the app: `steps`, each `{tool, args}` with tool one of the pointer/keyboard tools (click, drag, hotkey, type_text, key, text, pointer_move, pointer_down, pointer_up, wheel, click_entity, drag_entity, modifiers_set, paste, click_gap, pointer_gone), run in order. Answers each step's result; stops at the first that fails.",
            json!({ "type": "object", "properties": { "steps": { "type": "array", "maxItems": 256, "items": { "type": "object", "properties": { "tool": { "type": "string" }, "args": { "type": "object" } }, "required": ["tool"], "additionalProperties": false } } }, "required": ["steps"], "additionalProperties": false }),
            Annotations { read_only: false, destructive: true, idempotent: false, waits: false },
            move |args| {
                let service = service.clone();
                let _slot = input_slot.clone();
                Box::pin(async move {
                    let state = service.server.get().and_then(std::sync::Weak::upgrade).ok_or("the server is stopping")?;
                    let mut results = Vec::new();
                    for step in args.get("steps").and_then(Value::as_array).cloned().unwrap_or_default() {
                        let name = step.get("tool").and_then(Value::as_str).unwrap_or_default().to_string();
                        let spec = state.tools.read().await.get(&name).cloned();
                        let Some(spec) = spec.filter(|s| matches!(s.group, "pointer" | "keyboard")) else {
                            return Err(format!("`{name}` is not a pointer or keyboard tool"));
                        };
                        let out = (spec.handler)(step.get("args").cloned().unwrap_or(json!({}))).await.map_err(|e| format!("{name}: {e}"))?;
                        results.push(json!({ "tool": name, "result": out.json }));
                    }
                    Ok(ToolOutput::json(json!({ "steps": results })))
                })
            },
        ),
    ];
    for tool in &mut tools {
        match tool.name.as_str() {
            "run_script" | "script_result" => tool.output_schema = super::output_schema::execution(false),
            "script_halt" => tool.output_schema = super::output_schema::execution(true),
            "screenshot" => tool.output_schema = screenshot.output_schema.clone(),
            "files" => tool.output_schema = crate::files::output_schema(),
            "input" => tool.output_schema = crate::tools::object_schema(json!({
                "steps":{"type":"array","items":crate::tools::object_schema(json!({
                    "tool":{"type":"string"},
                    "result":{"type":"object","additionalProperties":true,"description":"The selected pointer/keyboard tool's JSON result; native envelope fields and composed results differ by tool."}
                }), &["tool","result"])}
            }), &["steps"]),
            _ => {},
        }
    }
    tools
}

