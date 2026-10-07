//! Server-side tools: session lifecycle, the pointer/keyboard compositions
//! (§6), screenshot post-processing (§5), the file halves of document tools,
//! and the schema-seeded `feature_add` / `feature_set_params` (§7). They are
//! registered in the same [`ToolSpec`] form as the app's commands, so the tool
//! list is one uniform walk. Everything here composes app commands by name
//! only where the composition itself is the point (click = move + down + up).
use crate::host::{Backend, HostConfig};
use crate::session::Session;
use crate::tools::app::{after_wait, current, wait_idle, SessionSlot};
use crate::tools::{object_schema, Annotations, ToolImage, ToolOutput, ToolSpec};
use crate::{image, schema, validate};
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::Arc;

pub struct ServerContext {
    pub slot: SessionSlot,
    pub session_root: PathBuf,
    pub backend: Backend,
    /// Called after a session starts or stops so the server rebuilds its tool
    /// list from the new registry and announces `tools/list_changed`.
    pub on_tools_changed: Arc<dyn Fn() + Send + Sync>,
}

fn s(v: &Value, k: &str) -> Option<String> {
    v.get(k).and_then(Value::as_str).map(str::to_string)
}
fn f(v: &Value, k: &str, d: f32) -> f32 {
    v.get(k).and_then(Value::as_f64).map(|x| x as f32).unwrap_or(d)
}
fn b(v: &Value, k: &str, d: bool) -> bool {
    v.get(k).and_then(Value::as_bool).unwrap_or(d)
}
fn u(v: &Value, k: &str, d: u64) -> u64 {
    v.get(k).and_then(Value::as_u64).unwrap_or(d)
}

pub fn session_tools(cx: Arc<ServerContext>) -> Vec<ToolSpec> {
    let start_cx = cx.clone();
    let stop_cx = cx.clone();
    let restart_cx = cx.clone();
    let info_cx = cx.clone();
    let rec_cx = cx.clone();
    let script_cx = cx.clone();
    let mut tools = vec![
        ToolSpec::new(
            "session_start",
            "session",
            session_start_doc(&cx.backend),
            session_start_schema(&cx.backend),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let cx = start_cx.clone();
                Box::pin(async move {
                    let session = match &cx.backend {
                        Backend::Attached { .. } => {
                            // The running app is the session. Starting again
                            // is a no-op apart from the recording flag and
                            // the document to open.
                            let session = match cx.slot.read().await.clone() {
                                Some(s) => s,
                                None => attach_session(&cx, b(&args, "record", true)).await?,
                            };
                            session.set_recording(b(&args, "record", true));
                            session
                        }
                        Backend::Spawn { name, spawn } => {
                            if cx.slot.read().await.is_some() {
                                return Err("a session is already live; call session_stop first".into());
                            }
                            let backend = s(&args, "backend").unwrap_or_else(|| name.to_string());
                            if backend != *name {
                                return Err(format!("backend `{backend}` is not available here; this server hosts `{name}`"));
                            }
                            let root = cx.session_root.clone();
                            std::fs::create_dir_all(&root).map_err(|e| format!("session root {}: {e}", root.display()))?;
                            let stamp = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map(|d| d.as_millis()).unwrap_or(0);
                            let store_dir = match s(&args, "store_root") {
                                Some(p) => PathBuf::from(p),
                                None => root.join(format!("store-{stamp}")),
                            };
                            let cfg = HostConfig {
                                width: f(&args, "width", 1400.0),
                                height: f(&args, "height", 960.0),
                                ppp: f(&args, "ppp", 1.0),
                                seed: b(&args, "seed", false),
                                store_dir,
                            };
                            let spawn = spawn.clone();
                            let built = cfg.clone();
                            let host = tokio::task::spawn_blocking(move || spawn(built)).await.map_err(|e| e.to_string())??;
                            Arc::new(Session::new(&root, host, Some(cfg), b(&args, "record", true))?)
                        }
                    };
                    if let Some(path) = s(&args, "document") {
                        let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
                        let name = std::path::Path::new(&path).file_name().map(|n| n.to_string_lossy().to_string());
                        session.host.call_ok("doc_load", json!({ "json": text, "name": name })).await?;
                    }
                    let describe = session.host.call_ok("describe_commands", json!({})).await?.result.unwrap_or(Value::Null);
                    let was_live = cx.slot.read().await.is_some();
                    *cx.slot.write().await = Some(session.clone());
                    let after = after_wait(&session, 60_000).await?;
                    if !was_live {
                        (cx.on_tools_changed)();
                    }
                    let mut info = serde_json::to_value(session.info()).map_err(|e| e.to_string())?;
                    info["after"] = after;
                    info["commands"] = json!(describe.as_array().map(|a| a.len()).unwrap_or(0));
                    Ok(ToolOutput::json(info))
                })
            },
        ),
        ToolSpec::new(
            "session_stop",
            "session",
            if cx.backend.is_attached() {
                "Stop recording and detach from the app; the app stays open and its tools stay available."
            } else {
                "Stop the live session and its app; flushes the recording."
            },
            object_schema(json!({}), &[]),
            Annotations { read_only: false, destructive: !cx.backend.is_attached(), idempotent: true, waits: false },
            move |_args| {
                let cx = stop_cx.clone();
                Box::pin(async move {
                    if cx.backend.is_attached() {
                        let session = current(&cx.slot).await?;
                        session.set_recording(false);
                        return Ok(ToolOutput::json(serde_json::to_value(session.info()).map_err(|e| e.to_string())?));
                    }
                    let Some(session) = cx.slot.write().await.take() else {
                        return Err("no live session".into());
                    };
                    let info = session.info();
                    match Arc::try_unwrap(session) {
                        Ok(s) => tokio::task::spawn_blocking(move || s.host.stop()).await.map_err(|e| e.to_string())?,
                        Err(_) => return Err("session still in use by another call".into()),
                    }
                    (cx.on_tools_changed)();
                    Ok(ToolOutput::json(serde_json::to_value(info).map_err(|e| e.to_string())?))
                })
            },
        ),
        ToolSpec::new(
            "session_restart",
            "session",
            "Stop the app and build it again on the SAME store, surface size, ppp and seed \u{2014} the native counterpart of reloading the page. \
             Everything the app PERSISTED (saved documents, settings, the dock layout, the autosave blob) is still there; everything it held in memory is gone, \
             so this is how a script reaches boot-time behaviour such as the crash-recovery offer. \
             Refused on an attached window: that app is the user's and the server never built it.",
            object_schema(json!({ "timeout_ms": { "type": "integer", "default": 60000 } }), &[]),
            Annotations { read_only: false, destructive: true, idempotent: false, waits: true },
            move |args| {
                let cx = restart_cx.clone();
                Box::pin(async move {
                    let Backend::Spawn { spawn, .. } = &cx.backend else {
                        return Err("this server attaches to a running app; it cannot restart it".into());
                    };
                    let Some(old) = cx.slot.write().await.take() else {
                        return Err("no live session".into());
                    };
                    let (config, record) = match (old.config.clone(), old.info().recording) {
                        (Some(c), r) => (c, r),
                        (None, _) => {
                            *cx.slot.write().await = Some(old);
                            return Err("this session was not built by the server; it cannot be restarted".into());
                        }
                    };
                    // The OLD app has to be gone before the new one opens the
                    // same store: two apps writing one directory is a race, and
                    // the whole point of a restart is to read what the first one
                    // left behind.
                    match Arc::try_unwrap(old) {
                        Ok(s) => tokio::task::spawn_blocking(move || s.host.stop()).await.map_err(|e| e.to_string())?,
                        Err(still_held) => {
                            *cx.slot.write().await = Some(still_held);
                            return Err("session still in use by another call".into());
                        }
                    }
                    let root = cx.session_root.clone();
                    let spawn = spawn.clone();
                    let built = config.clone();
                    let host = tokio::task::spawn_blocking(move || spawn(built)).await.map_err(|e| e.to_string())??;
                    let session = Arc::new(Session::new(&root, host, Some(config), record)?);
                    *cx.slot.write().await = Some(session.clone());
                    let after = after_wait(&session, u(&args, "timeout_ms", 60_000)).await?;
                    let mut info = serde_json::to_value(session.info()).map_err(|e| e.to_string())?;
                    info["after"] = after;
                    Ok(ToolOutput::json(info))
                })
            },
        ),
        ToolSpec::new(
            "session_info",
            "session",
            "The live session: id, directory, host, uptime, shots taken, recording state, and the current frame_info.",
            object_schema(json!({}), &[]),
            Annotations::READ,
            move |_args| {
                let cx = info_cx.clone();
                Box::pin(async move {
                    let session = current(&cx.slot).await?;
                    let mut v = serde_json::to_value(session.info()).map_err(|e| e.to_string())?;
                    v["frame_info"] = session.host.call_ok("frame_info", json!({})).await?.result.unwrap_or(Value::Null);
                    v["pointer"] = session.host.call_ok("pointer_state", json!({})).await?.result.unwrap_or(Value::Null);
                    Ok(ToolOutput::json(v))
                })
            },
        ),
        ToolSpec::new(
            "session_record",
            "session",
            "Turn call recording on or off.",
            object_schema(json!({ "on": { "type": "boolean" } }), &["on"]),
            Annotations { read_only: false, destructive: false, idempotent: true, waits: false },
            move |args| {
                let cx = rec_cx.clone();
                Box::pin(async move {
                    let session = current(&cx.slot).await?;
                    session.set_recording(b(&args, "on", true));
                    Ok(ToolOutput::json(json!({ "recording": b(&args, "on", true) })))
                })
            },
        ),
        ToolSpec::new(
            "session_script",
            "session",
            "The recorded calls of this session as a test-mcp script (expectations left for the author). `since` skips the first N calls.",
            object_schema(json!({ "name": { "type": "string", "default": "recorded" }, "since": { "type": "integer", "default": 0 } }), &[]),
            Annotations::READ,
            move |args| {
                let cx = script_cx.clone();
                Box::pin(async move {
                    let session = current(&cx.slot).await?;
                    let name = s(&args, "name").unwrap_or_else(|| "recorded".into());
                    Ok(ToolOutput::json(json!({ "script": session.script(&name, u(&args, "since", 0) as usize) })))
                })
            },
        ),
    ];
    // An attached window is the user's app: the server never built it and must
    // not offer to build it again. Dropping the tool rather than refusing it at
    // call time keeps `tools/list` honest about what this host can do.
    if cx.backend.is_attached() {
        tools.retain(|t| t.name != "session_restart");
    }
    tools
}

/// The session_start doc and schema follow the backend: a spawning host takes
/// the app's size and store, an attached one is the window as it is.
fn session_start_doc(backend: &Backend) -> String {
    match backend {
        Backend::Attached { .. } => format!(
            "Attach to the running app (backend `{}`): the window you see is the session, with its own document store. \
             document: a .nbrep path to open. record: log every call for session_script. \
             Returns the session id, directory and host info. Calling it again keeps the session and only applies `record` and `document`.",
            backend.name()
        ),
        Backend::Spawn { name, .. } => format!(
            "Start an app session. backend: {name} (no display needed). width/height in egui points (1400×960), ppp (1). \
             seed: start on the seed model (default false → empty document). document: a .nbrep path to open. \
             record: log every call for session_script. Returns the session id, directory and host info; \
             the tool list is regenerated from the app's registries."
        ),
    }
}

fn session_start_schema(backend: &Backend) -> Value {
    match backend {
        Backend::Attached { .. } => object_schema(json!({
            "document": { "type": "string" },
            "record": { "type": "boolean", "default": true }
        }), &[]),
        Backend::Spawn { name, .. } => object_schema(json!({
            "backend": { "type": "string", "enum": [name], "default": name },
            "width": { "type": "number", "default": 1400 },
            "height": { "type": "number", "default": 960 },
            "ppp": { "type": "number", "default": 1 },
            "seed": { "type": "boolean", "default": false },
            "document": { "type": "string" },
            "record": { "type": "boolean", "default": true },
            "store_root": { "type": "string", "description": "a persistent store directory (recovery tests); default: a fresh one under the session dir" }
        }), &[]),
    }
}

/// Make the attached app the live session: read its surface size from the
/// first `frame_info` (a window has whatever size the user gave it), create
/// the session directory, fill the slot.
pub async fn attach_session(cx: &ServerContext, record: bool) -> Result<Arc<Session>, String> {
    let mut host = cx.backend.attached_handle().ok_or("this server spawns its sessions; use session_start")?;
    let frame = host.call_ok("frame_info", json!({})).await?.result.unwrap_or(Value::Null);
    if let Some(surface) = frame["surface"].as_array() {
        host.info.width = surface.first().and_then(Value::as_f64).unwrap_or(host.info.width as f64) as f32;
        host.info.height = surface.get(1).and_then(Value::as_f64).unwrap_or(host.info.height as f64) as f32;
    }
    if let Some(ppp) = frame["ppp"].as_f64() {
        host.info.ppp = ppp as f32;
    }
    let root = cx.session_root.clone();
    std::fs::create_dir_all(&root).map_err(|e| format!("session root {}: {e}", root.display()))?;
    let session = Arc::new(Session::new(&root, host, None, record)?);
    *cx.slot.write().await = Some(session.clone());
    Ok(session)
}

/// One frame per primitive; the app answers each before the next is sent.
/// Frames a `dwell` click rests before pressing. The headless host steps the app
/// at a fixed 1/60 s per frame and the egui clock only advances inside a call
/// (see `wait_frames`), so 40 frames is two thirds of a second — comfortably past
/// the viewport's 0.5 s hover dwell, with room for a style that lengthens it a
/// little. Resting is how a pointer asks the 3D viewport for its PICK LIST
/// instead of selecting what is under it.
const DWELL_FRAMES: usize = 40;

/// The steps that rest the pointer where it already is: one `ping` per frame,
/// which is what makes the app draw and its clock run.
fn dwell_steps(args: &Value) -> Vec<(&'static str, Value)> {
    if !dwelling(args) {
        return Vec::new();
    }
    (0..DWELL_FRAMES).map(|_| ("ping", json!({}))).collect()
}

/// Whether this click is asking to dwell first.
fn dwelling(args: &Value) -> bool {
    match args.get("dwell") {
        Some(Value::Bool(b)) => *b,
        Some(v) => v.as_u64().is_some_and(|n| n > 0),
        None => false,
    }
}

async fn seq(session: &Session, steps: &[(&str, Value)]) -> Result<(), String> {
    for (cmd, args) in steps {
        session.host.call_ok(cmd, args.clone()).await?;
    }
    Ok(())
}

pub fn pointer_tools(slot: SessionSlot) -> Vec<ToolSpec> {
    let click_slot = slot.clone();
    let drag_slot = slot.clone();
    let hotkey_slot = slot.clone();
    let type_slot = slot.clone();
    let widget_slot = slot.clone();
    let entity_slot = slot.clone();
    let drag_widget_slot = slot.clone();
    let drag_entity_slot = slot.clone();
    let scroll_slot = slot.clone();
    let wait_slot = slot.clone();
    let frames_slot = slot.clone();
    let state_slot = slot.clone();
    vec![
        ToolSpec::new(
            "click",
            "pointer",
            "Move to (x, y) in egui points and click: down then up, one frame each. button primary|secondary|middle; count 2 for a double click, exact however recently the last click landed (`click_gap` first); modifiers {ctrl, shift, alt, command} held for the click. \
             `dwell` rests the pointer there first, which in the 3D viewport is the difference between the two plain-click behaviours: a click that arrives without dwelling SELECTS what is highlighted (the nearest hit by depth), and one that arrives after the dwell opens the PICK LIST.",
            object_schema(json!({
                "x": { "type": "number" }, "y": { "type": "number" },
                "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" },
                "count": { "type": "integer", "default": 1, "minimum": 1, "maximum": 3 },
                "dwell": { "type": "boolean", "default": false, "description": "rest the pointer at the point for the app's hover dwell before pressing" },
                "modifiers": { "type": "object", "properties": { "ctrl": {"type":"boolean"}, "shift": {"type":"boolean"}, "alt": {"type":"boolean"}, "command": {"type":"boolean"} } }
            }), &["x", "y"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = click_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let button = s(&args, "button").unwrap_or_else(|| "primary".into());
                    let mods = args.get("modifiers").cloned().unwrap_or(json!({}));
                    // `click_gap` first: `count` is then exact however recently
                    // the last click landed (see the app's `click_gap` command).
                    let mut steps = vec![("click_gap", json!({})), ("modifiers_set", mods), ("pointer_move", json!({ "x": f(&args, "x", 0.0), "y": f(&args, "y", 0.0) }))];
                    steps.extend(dwell_steps(&args));
                    for _ in 0..u(&args, "count", 1).clamp(1, 3) {
                        steps.push(("pointer_down", json!({ "button": button })));
                        steps.push(("pointer_up", json!({ "button": button })));
                    }
                    steps.push(("modifiers_set", json!({})));
                    seq(&session, &steps).await?;
                    session.record("click", &args, true, json!({}));
                    Ok(ToolOutput::json(json!({ "clicked": [f(&args, "x", 0.0), f(&args, "y", 0.0)] })))
                })
            },
        ),
        ToolSpec::new(
            "drag",
            "pointer",
            "Press at `from`, move to `to` over `steps` frames (12), release. button primary|secondary|middle (middle orbits/pans the viewport). Points in egui points.",
            object_schema(json!({
                "from": { "type": "array", "items": { "type": "number" }, "minItems": 2, "maxItems": 2 },
                "to": { "type": "array", "items": { "type": "number" }, "minItems": 2, "maxItems": 2 },
                "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" },
                "steps": { "type": "integer", "default": 12, "minimum": 1 },
                "hold": { "type": "boolean", "default": false, "description": "stop with the button still DOWN instead of releasing, so the next step can read a state that only exists mid-drag (a gizmo readout, a live preview); release it with `pointer_up`" },
                "modifiers": { "type": "object" },
                "capture": capture_schema()
            }), &["from", "to"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = drag_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let pt = |k: &str| -> Result<(f32, f32), String> {
                        let a = args.get(k).and_then(Value::as_array).ok_or_else(|| format!("`{k}` must be [x, y]"))?;
                        Ok((a.first().and_then(Value::as_f64).unwrap_or(0.0) as f32, a.get(1).and_then(Value::as_f64).unwrap_or(0.0) as f32))
                    };
                    let (x0, y0) = pt("from")?;
                    let (x1, y1) = pt("to")?;
                    let n = u(&args, "steps", 12).max(1) as usize;
                    let images = drag_from(&session, x0, y0, x1 - x0, y1 - y0, &args).await?;
                    session.record("drag", &args, true, json!({}));
                    Ok(ToolOutput { json: json!({ "from": [x0, y0], "to": [x1, y1], "steps": n }), images })
                })
            },
        ),
        ToolSpec::new(
            "hotkey",
            "keyboard",
            "Press a key combination such as `ctrl+z`, `ctrl+shift+z`, `Escape`, `Delete`: modifiers down, key press and release, modifiers up.",
            object_schema(json!({ "combo": { "type": "string" } }), &["combo"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = hotkey_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let combo = s(&args, "combo").ok_or("missing `combo`")?;
                    let mut mods = json!({});
                    let mut key = String::new();
                    for part in combo.split('+') {
                        match part.trim().to_ascii_lowercase().as_str() {
                            "ctrl" | "control" => mods["ctrl"] = json!(true),
                            "shift" => mods["shift"] = json!(true),
                            "alt" => mods["alt"] = json!(true),
                            "cmd" | "command" | "meta" => mods["command"] = json!(true),
                            other => key = if other.len() == 1 { other.to_ascii_uppercase() } else { part.trim().to_string() },
                        }
                    }
                    if key.is_empty() {
                        return Err("combo has no key".into());
                    }
                    seq(&session, &[
                        ("modifiers_set", mods),
                        ("key", json!({ "key": key, "pressed": true })),
                        ("key", json!({ "key": key, "pressed": false })),
                        ("modifiers_set", json!({})),
                    ]).await?;
                    session.record("hotkey", &args, true, json!({}));
                    Ok(ToolOutput::json(json!({ "key": key })))
                })
            },
        ),
        ToolSpec::new(
            "type_text",
            "keyboard",
            "Type text into the focused widget (click a field first). `enter: true` presses Enter afterwards to commit.",
            object_schema(json!({ "text": { "type": "string" }, "enter": { "type": "boolean", "default": false } }), &["text"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = type_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let text = s(&args, "text").unwrap_or_default();
                    let mut steps = vec![("text", json!({ "text": text }))];
                    if b(&args, "enter", false) {
                        steps.push(("key", json!({ "key": "Enter", "pressed": true })));
                        steps.push(("key", json!({ "key": "Enter", "pressed": false })));
                    }
                    seq(&session, &steps).await?;
                    session.record("type_text", &args, true, json!({}));
                    Ok(ToolOutput::json(json!({})))
                })
            },
        ),
        ToolSpec::new(
            "click_widget",
            "widgets",
            "Click a published widget by its `panel/key` (see hit_rects / hit_key_docs), scrolling its pane so the widget is inside `panel:clip` first. Errors if the key is not published this frame. `count` is EXACT: 1 is a single click, 2 a double, 3 a triple, however recently another click landed \u{2014} egui's clock is first moved past the multi-click window of the clicks before (`click_gap`).",
            object_schema(json!({ "key": { "type": "string" }, "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" }, "count": { "type": "integer", "default": 1 } }), &["key"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = widget_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let key = s(&args, "key").ok_or("missing `key`")?;
                    // The gap BEFORE the rect is read: the frames between
                    // reading it and pressing are the ones in which a layout
                    // change (a result arriving) moves the widget from under
                    // the pointer, and the gap must not add one.
                    session.host.call_ok("click_gap", json!({})).await?;
                    let rect = scroll_into_view(&session, &key).await?;
                    let (cx, cy) = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0);
                    let button = s(&args, "button").unwrap_or_else(|| "primary".into());
                    let mut steps = vec![("pointer_move", json!({ "x": cx, "y": cy }))];
                    for _ in 0..u(&args, "count", 1).clamp(1, 3) {
                        steps.push(("pointer_down", json!({ "button": button })));
                        steps.push(("pointer_up", json!({ "button": button })));
                    }
                    seq(&session, &steps).await?;
                    session.record("click_widget", &args, true, json!({ "at": [cx, cy] }));
                    Ok(ToolOutput::json(json!({ "key": key, "clicked": [cx, cy], "rect": rect })))
                })
            },
        ),
        ToolSpec::new(
            "click_entity",
            "pointer",
            "Click a scene entity by NAME in the 3D viewport: `locate` resolves `kind` (solid | face | edge | vertex) and `name` to a point on the surface at execution time, and the click lands there. \
             The entity's stand-in point is a face's triangle centroid, an edge's midpoint, a vertex's position, a solid's bounding-box centre. \
             Refuses, naming the entity, when a click there could not reach it: off screen, over a hole, hidden, excluded by the selection filter, or taken by something else (picking is priority-ordered \u{2014} vertex, edge, face \u{2014} so an edge within its pixel radius takes the click from the face behind it). A click that would silently hit the wrong thing is an error instead. A click selects it outright even where the point carries several admitted candidates — overlapping geometry no longer diverts the click into the pick list, which is now opened by DWELLING (rest the pointer on the highlight for the dwell, then click) or by `alt`. To take an obstructed entity instead, hold `alt` here and finish from `__brepCandidates`. A reference field's picker has no popup and takes the top candidate, and its filter resolves a face or an edge to the solid that owns it \u{2014} so `{kind: \"face\"}` is how a script says WHERE on a solid to press. \
             `count` 2 double-clicks; `modifiers` {ctrl, shift, alt, command} are held for the click (ctrl-click is how the app multi-selects).",
            object_schema(json!({
                "kind": { "type": "string", "enum": ["solid", "face", "edge", "vertex"] },
                "name": { "type": "string", "description": "the reference name; a vertex takes its topoId as a number string" },
                "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" },
                "count": { "type": "integer", "default": 1, "minimum": 1, "maximum": 3 },
                "dwell": { "type": "boolean", "default": false, "description": "rest the pointer on the entity for the app's hover dwell before pressing, which opens the PICK LIST with this entity in it instead of selecting outright" },
                "modifiers": { "type": "object", "properties": { "ctrl": {"type":"boolean"}, "shift": {"type":"boolean"}, "alt": {"type":"boolean"}, "command": {"type":"boolean"} } }
            }), &["kind", "name"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = entity_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let spot = locate_point(&session, &args).await?;
                    let button = s(&args, "button").unwrap_or_else(|| "primary".into());
                    let mods = args.get("modifiers").cloned().unwrap_or(json!({}));
                    let mut steps = vec![("modifiers_set", mods), ("pointer_move", json!({ "x": spot.x, "y": spot.y }))];
                    steps.extend(dwell_steps(&args));
                    for _ in 0..u(&args, "count", 1).clamp(1, 3) {
                        steps.push(("pointer_down", json!({ "button": button })));
                        steps.push(("pointer_up", json!({ "button": button })));
                    }
                    steps.push(("modifiers_set", json!({})));
                    seq(&session, &steps).await?;
                    session.record("click_entity", &args, true, json!({ "at": [spot.x, spot.y] }));
                    Ok(ToolOutput::json(json!({ "kind": spot.kind, "name": spot.name, "clicked": [spot.x, spot.y], "world": spot.world })))
                })
            },
        ),
        ToolSpec::new(
            "drag_widget",
            "widgets",
            "Drag a published widget by its `panel/key`: the press lands on the widget's centre (its pane scrolled so it is visible, as `click_widget` does) and the pointer travels `by` [dx, dy] egui points — or to the centre of the widget `to_key` names — over `steps` frames before releasing. Give exactly one of `by` and `to_key`; `to_key` is what drags one published thing onto another, such as a wiring connection from one pin to another.",
            object_schema(json!({
                "key": { "type": "string" },
                "by": { "type": "array", "items": { "type": "number" }, "minItems": 2, "maxItems": 2, "description": "[dx, dy] in egui points" },
                "to_key": { "type": "string", "description": "another published widget's `panel/key`: the drag ends on its centre instead of travelling `by`" },
                "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" },
                "steps": { "type": "integer", "default": 12, "minimum": 1 },
                "hold": { "type": "boolean", "default": false, "description": "stop with the button still DOWN instead of releasing, so the next step can read a state that only exists mid-drag (a gizmo readout, a live preview); release it with `pointer_up`" },
                "modifiers": { "type": "object" },
                "capture": capture_schema()
            }), &["key"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = drag_widget_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let key = s(&args, "key").ok_or("missing `key`")?;
                    let rect = scroll_into_view(&session, &key).await?;
                    let (x, y) = (rect[0] + rect[2] / 2.0, rect[1] + rect[3] / 2.0);
                    let (dx, dy) = match (s(&args, "to_key"), args.get("by")) {
                        (Some(_), Some(_)) => return Err("give `by` or `to_key`, not both".into()),
                        (Some(to), None) => {
                            let end = published_rect(&session, &to).await?;
                            (end[0] + end[2] / 2.0 - x, end[1] + end[3] / 2.0 - y)
                        }
                        (None, _) => delta(&args)?,
                    };
                    let images = drag_from(&session, x, y, dx, dy, &args).await?;
                    session.record("drag_widget", &args, true, json!({ "from": [x, y] }));
                    Ok(ToolOutput { json: json!({ "key": key, "from": [x, y], "to": [x + dx, y + dy], "rect": rect }), images })
                })
            },
        ),
        ToolSpec::new(
            "drag_entity",
            "pointer",
            "Drag from a scene entity by NAME: `locate` resolves `kind` and `name` to a point on the surface at execution time (the same refusals as `click_entity`), the press lands there, and the pointer travels `by` [dx, dy] egui points before releasing. \
             This is how a gizmo handle sitting on an entity is dragged without writing the projection arithmetic out.",
            object_schema(json!({
                "kind": { "type": "string", "enum": ["solid", "face", "edge", "vertex"] },
                "name": { "type": "string" },
                "by": { "type": "array", "items": { "type": "number" }, "minItems": 2, "maxItems": 2, "description": "[dx, dy] in egui points" },
                "button": { "type": "string", "enum": ["primary", "secondary", "middle"], "default": "primary" },
                "steps": { "type": "integer", "default": 12, "minimum": 1 },
                "hold": { "type": "boolean", "default": false, "description": "stop with the button still DOWN instead of releasing, so the next step can read a state that only exists mid-drag (a gizmo readout, a live preview); release it with `pointer_up`" },
                "modifiers": { "type": "object" },
                "capture": capture_schema()
            }), &["kind", "name", "by"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: false },
            move |args| {
                let slot = drag_entity_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let spot = locate_point(&session, &args).await?;
                    let (dx, dy) = delta(&args)?;
                    let images = drag_from(&session, spot.x, spot.y, dx, dy, &args).await?;
                    session.record("drag_entity", &args, true, json!({ "from": [spot.x, spot.y] }));
                    Ok(ToolOutput { json: json!({ "kind": spot.kind, "name": spot.name, "from": [spot.x, spot.y], "to": [spot.x + dx, spot.y + dy], "world": spot.world }), images })
                })
            },
        ),
        ToolSpec::new(
            "scroll_into_view",
            "widgets",
            "Scroll a pane until the widget `panel/key` is inside its `panel:clip` rect; returns the rect.",
            object_schema(json!({ "key": { "type": "string" } }), &["key"]),
            Annotations { read_only: false, destructive: false, idempotent: true, waits: false },
            move |args| {
                let slot = scroll_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let key = s(&args, "key").ok_or("missing `key`")?;
                    let rect = scroll_into_view(&session, &key).await?;
                    Ok(ToolOutput::json(json!({ "key": key, "rect": rect })))
                })
            },
        ),
        ToolSpec::new(
            "wait_idle",
            "capture",
            "Wait until the history runner and every background query are idle (plus settle frames), or the timeout. Returns frame_info.",
            object_schema(json!({ "timeout_ms": { "type": "integer", "default": 60000 }, "settle_frames": { "type": "integer", "default": 2 } }), &[]),
            Annotations::READ,
            move |args| {
                let slot = wait_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let info = wait_idle(&session, u(&args, "timeout_ms", 60_000), u(&args, "settle_frames", 2) as u32).await?;
                    Ok(ToolOutput::json(info))
                })
            },
        ),
        ToolSpec::new(
            "wait_frames",
            "capture",
            "Let the app draw `count` more frames. The headless host steps the app only while a call is in flight, so the egui CLOCK \u{2014} every debounce, tooltip delay and animation \u{2014} advances only inside a call: this is how a script spends time. \
             On an attached window it waits for the window's own frames instead. Returns the frame counter before and after.",
            object_schema(json!({ "count": { "type": "integer", "default": 1, "minimum": 1 }, "timeout_ms": { "type": "integer", "default": 60000 } }), &[]),
            Annotations::READ,
            move |args| {
                let slot = frames_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let want = u(&args, "count", 1).max(1);
                    let timeout_ms = u(&args, "timeout_ms", 60_000);
                    let started = std::time::Instant::now();
                    let from = session.host.call_ok("ping", json!({})).await?.frame;
                    let mut now = from;
                    while now < from + want {
                        if started.elapsed().as_millis() as u64 > timeout_ms {
                            return Err(format!("wait_frames: only {} of {want} frames in {timeout_ms} ms", now - from));
                        }
                        now = session.host.call_ok("ping", json!({})).await?.frame;
                    }
                    Ok(ToolOutput::json(json!({ "from": from, "to": now, "frames": now - from })))
                })
            },
        ),
        ToolSpec::new(
            "wait_state",
            "capture",
            "Step the app one frame at a time until every `until` expectation holds on `state_get(names)`, and return that state. Use it instead of `wait_frames` to catch something IN FLIGHT (a run's progress, the busy indicator): the history runner works on its own thread in wall time while the egui clock advances a fixed step per frame, so a fixed frame count samples a different moment on a loaded machine — this one samples the first frame where it is true. Fails, naming the first unmet expectation and the last value it read, after `max_frames` frames or `timeout_ms`. `until` entries are the script `expect` form (`path` plus one check).",
            object_schema(
                json!({
                    "names": { "type": "array", "items": { "type": "string" } },
                    "until": { "type": "array", "items": { "type": "object" } },
                    "max_frames": { "type": "integer", "default": 600, "minimum": 1 },
                    "timeout_ms": { "type": "integer", "default": 60000 }
                }),
                &["names", "until"],
            ),
            Annotations::READ,
            move |args| {
                let slot = state_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let names = args.get("names").cloned().unwrap_or(json!([]));
                    let until: Vec<crate::script::Expect> =
                        serde_json::from_value(args.get("until").cloned().unwrap_or(json!([]))).map_err(|e| format!("wait_state: `until`: {e}"))?;
                    let max_frames = u(&args, "max_frames", 600).max(1);
                    let timeout_ms = u(&args, "timeout_ms", 60_000);
                    let started = std::time::Instant::now();
                    let mut frames = 0;
                    loop {
                        let state = session.host.call_ok("state_get", json!({ "names": names })).await?.result.unwrap_or(Value::Null);
                        let unmet = until.iter().find_map(|expect| expect.check(&state).err().map(|why| format!("{}: {why}", expect.path)));
                        let Some(unmet) = unmet else {
                            return Ok(ToolOutput::json(json!({ "frames": frames, "state": state })));
                        };
                        if frames >= max_frames || started.elapsed().as_millis() as u64 > timeout_ms {
                            return Err(format!("wait_state: after {frames} frames, still {unmet}"));
                        }
                        session.host.call_ok("ping", json!({})).await?;
                        frames += 1;
                    }
                })
            },
        ),
    ]
}

/// Where `locate` puts an entity on the surface, refusing rather than clicking
/// into nothing: this is the whole of `click_entity` / `drag_entity` that is
/// about the ENTITY, and both spend it the same way.
struct Spot {
    kind: String,
    name: String,
    x: f32,
    y: f32,
    world: Value,
}

async fn locate_point(session: &Session, args: &Value) -> Result<Spot, String> {
    let kind = s(args, "kind").ok_or("missing `kind` (solid | face | edge | vertex)")?;
    let name = s(args, "name").ok_or("missing `name`")?;
    let r = session
        .host
        .call_ok("locate", json!({ "kind": kind, "name": name }))
        .await?
        .result
        .unwrap_or(Value::Null);
    let at = format!("({:.1}, {:.1})", r["x"].as_f64().unwrap_or(0.0), r["y"].as_f64().unwrap_or(0.0));
    if r["visible"].as_bool() != Some(true) {
        return Err(format!("{kind} `{name}` is not on screen (behind the camera, or outside the viewport); frame it first"));
    }
    // `reaches` is `locate`'s answer to the only question that matters here:
    // does a click at that point resolve to the entity that was NAMED? A plain
    // click takes the nearest admitted candidate by depth (the one the hover
    // highlight shows), and a reference field's picker still takes the top of
    // the ranked list; `locate` applies whichever rule is in force. Anything
    // else would land on something in front, so it is refused rather than
    // clicked. (It replaced a `rank == 0` test, which asked where the entity sat
    // in the pick LIST — the same question only while the list's head and the
    // click's answer were the same thing, which the selection UX changed.)
    // A click that OPENS THE PICK LIST — `alt`, or one that dwells first — asks
    // a different question of the point: the entity only has to BE in the list
    // it opens, and the caller finishes from `__brepCandidates`. Refusing those
    // on "something else is in front" would refuse the very gesture that exists
    // to reach what is behind.
    if dwelling(args) || args["modifiers"]["alt"].as_bool() == Some(true) {
        if r["rank"].is_null() {
            return Err(format!(
                "{kind} `{name}` is on screen at {at} but is not among the candidates a click there would list \u{2014} it is hidden, or the selection filter admits no kind that reaches it"
            ));
        }
        return Ok(Spot {
            kind,
            name,
            x: r["x"].as_f64().unwrap_or(0.0) as f32,
            y: r["y"].as_f64().unwrap_or(0.0) as f32,
            world: r["world"].clone(),
        });
    }
    if r["reaches"].as_bool() != Some(true) {
        let empty = r["under"].as_array().map(|a| a.is_empty()).unwrap_or(true);
        if empty {
            return Err(format!(
                "{kind} `{name}` is on screen at {at} but a click there would pick NOTHING — the anchor projects into a hole, the entity is hidden, or the selection filter admits no kind here"
            ));
        }
        // Not necessarily something NEARER: picking is priority-ordered
        // (vertex, then edge, then face), so an edge behind a face takes the
        // click when it is within its pixel radius — which an axis-aligned ISO
        // view of a box arranges exactly, corner edge onto face centroid.
        let other = &r["occluded_by"];
        let what = other.get("name").and_then(Value::as_str).filter(|n| !n.is_empty()).map(str::to_string);
        let what = what.unwrap_or_else(|| other.get("kind").and_then(Value::as_str).unwrap_or("something else").to_string());
        return Err(format!("a click at {kind} `{name}` ({at}) would pick `{what}` instead; frame it differently, or name what the pick finds"));
    }
    Ok(Spot {
        kind,
        name,
        x: r["x"].as_f64().unwrap_or(0.0) as f32,
        y: r["y"].as_f64().unwrap_or(0.0) as f32,
        world: r["world"].clone(),
    })
}

/// Capture the surface with the virtual pointer DRAWN on it, at full
/// resolution, and hand back the picture plus the capture's `frame_info`.
///
/// The pointer is drawn here rather than by the compositor's caller because it
/// has to go on BEFORE the downscale — a 12 px arrow squeezed with the frame
/// survives, one stamped on the thumbnail afterwards would have to be scaled
/// separately and would land off by the rounding. `scale` blows the shape up
/// for a walkthrough frame; an ordinary screenshot passes 1.
///
/// Shared by the `screenshot` tool and by a drag's `capture` (the walkthrough
/// recorder's motion frames), so the pointer in a filmed drag is the same
/// pointer a screenshot draws, in the same place.
async fn shot_with_pointer(
    session: &Session,
    region: Value,
    cursor: bool,
    cursor_scale: u32,
) -> Result<(Value, image::RgbaImage), String> {
    let (info, png) = session.host.screenshot(region).await?;
    let mut img = image::decode_png(&png)?;
    if cursor {
        if let Ok(r) = session.host.call_ok("pointer_state", json!({})).await {
            if let Some(pos) = r.result.as_ref().and_then(|v| v["pos"].as_array()) {
                let ppp = info["ppp"].as_f64().unwrap_or(1.0);
                let ox = info["region"][0].as_f64().unwrap_or(0.0);
                let oy = info["region"][1].as_f64().unwrap_or(0.0);
                let x = ((pos[0].as_f64().unwrap_or(0.0) - ox) * ppp).round() as i64;
                let y = ((pos[1].as_f64().unwrap_or(0.0) - oy) * ppp).round() as i64;
                let pressed = r.result.as_ref().and_then(|v| v["buttons"].as_array()).map(|b| !b.is_empty()).unwrap_or(false);
                image::draw_cursor(&mut img, x, y, pressed, cursor_scale);
            }
        }
    }
    Ok((info, img))
}

/// The `capture` argument the three drag compositions take: film the drag
/// instead of merely performing it.
///
/// This is the walkthrough recorder's motion support, and it lives on the TOOL
/// rather than in the runner for one reason — the press point. `drag_widget`
/// resolves it by scrolling a published rect into view, `drag_entity` by
/// projecting a named entity, `drag` reads it off the arguments; a recorder
/// that filmed the drag itself would have to resolve the press point a second
/// time, the same three ways, and would drift from the tool the day one of them
/// changed. So the tool performs its own ordinary drag and hands back the
/// pictures.
#[derive(Debug)]
struct Capture {
    /// Pictures taken between the press and the release. The still of the
    /// parked pointer before the press is always taken as well, so the tool
    /// returns `frames + 1` images.
    frames: usize,
    max_width: u32,
    cursor_scale: u32,
}

impl Capture {
    /// `None` when the caller did not ask to film.
    fn parse(args: &Value) -> Result<Option<Self>, String> {
        let Some(spec) = args.get("capture") else { return Ok(None) };
        if spec.is_null() {
            return Ok(None);
        }
        let obj = spec.as_object().ok_or("`capture` must be an object")?;
        for key in obj.keys() {
            if !matches!(key.as_str(), "frames" | "max_width" | "cursor_scale") {
                return Err(format!("`capture`: unknown field `{key}` (frames | max_width | cursor_scale)"));
            }
        }
        Ok(Some(Capture {
            frames: u(spec, "frames", 8).max(1) as usize,
            max_width: u(spec, "max_width", 1000) as u32,
            cursor_scale: u(spec, "cursor_scale", 3).max(1) as u32,
        }))
    }

    /// The `steps` indices a picture is taken after — `frames` of them, evenly
    /// spaced, the LAST always being the final move.
    ///
    /// Spaced over the drag's own `steps` rather than replacing them: the
    /// pointer path a filmed drag travels is exactly the path it travels
    /// unfilmed, so filming cannot change what the drag is worth.
    ///
    /// Callers pass a `steps` of at least [`frames`](Self::frames) (see
    /// [`travel_steps`](Self::travel_steps)), so no two pictures share a move
    /// and no frame of the film is a duplicate of its neighbour.
    fn shot_after(&self, steps: usize) -> Vec<usize> {
        (1..=self.frames)
            .map(|i| ((i * steps) as f64 / self.frames as f64).round().max(1.0) as usize)
            .collect()
    }

    /// How many moves the drag is broken into when it is filmed: its own
    /// `steps`, or `frames` if that is more.
    ///
    /// A drag with FEWER moves than pictures would otherwise shoot twice after
    /// the same move and put two identical frames in the GIF — a stutter, for
    /// no reason. Raising the move count instead interpolates the SAME path
    /// more finely, which changes neither the endpoints nor what the drag is
    /// worth. The defaults make this reachable (`frames` is 8, `steps` is 12,
    /// but a script may set `steps: 4`), so it is handled rather than
    /// documented away.
    fn travel_steps(&self, steps: usize) -> usize {
        steps.max(self.frames)
    }

    async fn shoot(&self, session: &Session) -> Result<ToolImage, String> {
        let (_, img) = shot_with_pointer(session, json!("full"), true, self.cursor_scale).await?;
        let inline = image::scale_to_width(&img, self.max_width);
        Ok(ToolImage { png: image::encode_png(&inline)?, mime: "image/png" })
    }
}


/// The JSON-schema fragment for `capture`, shared by the three drag tools.
fn capture_schema() -> Value {
    json!({
        "type": "object",
        "description": "FILM the drag: return one image of the pointer parked on the handle before the press, then `frames` images taken as it travels, so a walkthrough can show the drag instead of a still of the handle. The drag itself is unchanged.",
        "properties": {
            "frames": { "type": "integer", "default": 8, "minimum": 1, "description": "pictures taken between the press and the release" },
            "max_width": { "type": "integer", "default": 1000, "description": "pixel width each image is downscaled to" },
            "cursor_scale": { "type": "integer", "default": 3, "minimum": 1, "description": "size of the drawn pointer, as a multiple of the 12 px arrow" }
        },
        "additionalProperties": false
    })
}

/// `by: [dx, dy]`, the one delta both drag compositions take.
fn delta(args: &Value) -> Result<(f32, f32), String> {
    let a = args.get("by").and_then(Value::as_array).ok_or("`by` must be [dx, dy] in egui points")?;
    Ok((
        a.first().and_then(Value::as_f64).unwrap_or(0.0) as f32,
        a.get(1).and_then(Value::as_f64).unwrap_or(0.0) as f32,
    ))
}

/// Press at (x, y), travel `(dx, dy)` over `steps` frames, release — the body
/// of `drag`, reached by key or by name instead of by literal points.
///
/// `hold` keeps the button DOWN at the end of the travel instead of releasing
/// it, which is the only way a script can read a state that exists only WHILE
/// the pointer is down — a gizmo's live readout, a constraint handle's preview,
/// a drag's own highlight. Without it every such state was created and
/// destroyed inside one tool call and no expectation could ever see it. The
/// step that follows a held drag must release it (`pointer_up`), or the session
/// carries a pressed button into whatever it does next.
///
/// Returns the pictures a `capture` asked for, newest last, and an empty vec
/// when it did not — which is every call but a walkthrough's.
async fn drag_from(session: &Session, x: f32, y: f32, dx: f32, dy: f32, args: &Value) -> Result<Vec<ToolImage>, String> {
    let n = u(args, "steps", 12).max(1) as usize;
    let button = s(args, "button").unwrap_or_else(|| "primary".into());
    let Some(capture) = Capture::parse(args)? else {
        let mut steps = vec![
            ("modifiers_set", args.get("modifiers").cloned().unwrap_or(json!({}))),
            ("pointer_move", json!({ "x": x, "y": y })),
            ("pointer_down", json!({ "button": button })),
        ];
        for i in 1..=n {
            let t = i as f32 / n as f32;
            steps.push(("pointer_move", json!({ "x": x + dx * t, "y": y + dy * t })));
        }
        // HELD: stop with the button down and the modifiers still set — the
        // drag is mid-flight, which is the state the caller wants to read.
        if !args.get("hold").and_then(Value::as_bool).unwrap_or(false) {
            steps.push(("pointer_up", json!({ "button": button })));
            steps.push(("modifiers_set", json!({})));
        }
        seq(session, &steps).await?;
        return Ok(Vec::new());
    };

    // FILMED: the same moves, with the shutter open. The first picture is the
    // pointer parked on the handle with the button still UP — the "this is the
    // thing you grab" still — and the rest are taken after the moves
    // `shot_after` names, the last of which is the final one, so the drag ends
    // on a picture of where it got to.
    let mut shots = Vec::with_capacity(capture.frames + 1);
    let n = capture.travel_steps(n);
    seq(session, &[
        ("modifiers_set", args.get("modifiers").cloned().unwrap_or(json!({}))),
        ("pointer_move", json!({ "x": x, "y": y })),
    ]).await?;
    shots.push(capture.shoot(session).await?);
    session.host.call_ok("pointer_down", json!({ "button": button })).await?;
    let shot_after = capture.shot_after(n);
    for i in 1..=n {
        let t = i as f32 / n as f32;
        session
            .host
            .call_ok("pointer_move", json!({ "x": x + dx * t, "y": y + dy * t }))
            .await?;
        // A capture renders a frame of its own, so taking one mid-drag is not
        // a peek at a stale surface: the picture is the app as it stands with
        // the button still down and the geometry rebuilt to this pointer
        // position.
        for _ in shot_after.iter().filter(|s| **s == i) {
            shots.push(capture.shoot(session).await?);
        }
    }
    seq(session, &[
        ("pointer_up", json!({ "button": button })),
        ("modifiers_set", json!({})),
    ]).await?;
    Ok(shots)
}

/// Wheel the owning pane until `key` is inside its clip rect (§6). Returns
/// the rect in egui points.
/// The rect `key` (`panel/key`) is published at this frame, as it stands —
/// the END of a drag, which must not scroll the pane the drag starts in.
async fn published_rect(session: &Session, key: &str) -> Result<[f32; 4], String> {
    let (panel, _) = key.split_once('/').ok_or("key must be `panel/key`")?;
    let rects = session.host.call_ok("hit_rects", json!({ "prefix": format!("{panel}/") })).await?.result.unwrap_or(Value::Null);
    rects["rects"][key]
        .as_array()
        .map(|a| [a[0].as_f64().unwrap_or(0.0) as f32, a[1].as_f64().unwrap_or(0.0) as f32, a[2].as_f64().unwrap_or(0.0) as f32, a[3].as_f64().unwrap_or(0.0) as f32])
        .ok_or_else(|| format!("no widget `{key}` is published this frame (see hit_rects)"))
}

async fn scroll_into_view(session: &Session, key: &str) -> Result<[f32; 4], String> {
    let (panel, _) = key.split_once('/').ok_or("key must be `panel/key`")?;
    const SLACK: f32 = 1.0;
    // egui SMOOTHS a wheel scroll over several frames, so the rect a pane
    // reports on the frame its widget first comes inside the clip is still
    // TRAVELLING: a click at that point lands where the widget was, changes
    // nothing, and reads back as "the control does nothing". The answer is only
    // returned once two consecutive frames agree on it.
    let mut settled: Option<[f32; 4]> = None;
    let mut previous_top: Option<f32> = None;
    let mut stalled = 0;
    let mut gutter = false;
    for _ in 0..48 {
        let rects = session.host.call_ok("hit_rects", json!({ "prefix": format!("{panel}/") })).await?.result.unwrap_or(Value::Null);
        let rect = rects["rects"][key].as_array().map(|a| {
            [a[0].as_f64().unwrap_or(0.0) as f32, a[1].as_f64().unwrap_or(0.0) as f32, a[2].as_f64().unwrap_or(0.0) as f32, a[3].as_f64().unwrap_or(0.0) as f32]
        });
        let Some(rect) = rect else {
            return Err(format!("no widget `{key}` is published this frame (see hit_rects)"));
        };
        let clip = rects["rects"][format!("{panel}/panel:clip")].as_array().map(|a| {
            [a[0].as_f64().unwrap_or(0.0) as f32, a[1].as_f64().unwrap_or(0.0) as f32, a[2].as_f64().unwrap_or(0.0) as f32, a[3].as_f64().unwrap_or(0.0) as f32]
        });
        let Some(clip) = clip else { return Ok(rect) };
        let inside = rect[1] >= clip[1] - SLACK && rect[1] + rect[3] <= clip[1] + clip[3] + SLACK;
        if inside {
            if settled == Some(rect) {
                return Ok(rect);
            }
            settled = Some(rect);
            session.host.call_ok("ping", json!({})).await?;
            continue;
        }
        settled = None;
        // A nested list under the center can consume the wheel without
        // moving the owning pane. Try its gutter when progress stalls; some
        // panes have no scrollable gutter, so retain the center as a fallback.
        if previous_top.is_some_and(|top| (top - rect[1]).abs() < SLACK) {
            stalled += 1;
        } else {
            stalled = 0;
        }
        if stalled >= 2 {
            gutter = !gutter;
            stalled = 0;
        }
        previous_top = Some(rect[1]);
        let cx = clip[0] + if gutter { 2.0 } else { clip[2] / 2.0 };
        let cy = clip[1] + clip[3] / 2.0;
        // Long catalog/workspace lists can put the target thousands of points
        // away. Move by the remaining distance, capped at one viewport, so
        // the bounded retry loop also handles rows with image previews.
        let distance = if rect[1] < clip[1] {
            clip[1] - rect[1]
        } else {
            clip[1] + clip[3] - rect[1] - rect[3]
        };
        let dy = distance.clamp(-clip[3].max(1.0), clip[3].max(1.0));
        seq(session, &[("pointer_move", json!({ "x": cx, "y": cy })), ("wheel", json!({ "dx": 0, "dy": dy }))]).await?;
    }
    Err(format!("could not scroll `{key}` into view"))
}

/// Does `key` (without its panel) match a documented prefix? The rules mirror
/// `brep_app::automation::hit_keys::documented`: an empty prefix covers every
/// key of that panel, a prefix ending in `:` matches by start, a prefix that
/// starts with `:` matches anywhere (info windows key by `{name}:…`), and
/// anything else must match whole or followed by an INDEX (`refsel:x`
/// documents `refsel:x0`, `refsel:x1`, …).
pub fn key_documented(docs: &Value, panel: &str, key: &str) -> bool {
    docs.as_array().into_iter().flatten().any(|d| {
        d["panel"].as_str() == Some(panel) && {
            let prefix = d["prefix"].as_str().unwrap_or("");
            prefix.is_empty()
                || key == prefix
                || (prefix.ends_with(':') && key.starts_with(prefix))
                || (prefix.starts_with(':') && key.contains(prefix))
                || key
                    .strip_prefix(prefix)
                    .is_some_and(|index| index.bytes().all(|b| b.is_ascii_digit()))
        }
    })
}

pub fn registry_tools(slot: SessionSlot) -> Vec<ToolSpec> {
    vec![ToolSpec::new(
        "hit_keys_check",
        "widgets",
        "Every widget key the app publishes this frame that no panel has documented (the hit-key registry gate). An empty list is the pass condition.",
        object_schema(json!({}), &[]),
        Annotations::READ,
        move |_args| {
            let slot = slot.clone();
            Box::pin(async move {
                let session = current(&slot).await?;
                let rects = session.host.call_ok("hit_rects", json!({})).await?.result.unwrap_or(Value::Null);
                let docs = session.host.call_ok("hit_key_docs", json!({})).await?.result.unwrap_or(Value::Null);
                let docs = docs["keys"].clone();
                let mut undocumented = Vec::new();
                let mut checked = 0;
                if let Some(map) = rects["rects"].as_object() {
                    for full in map.keys() {
                        checked += 1;
                        if let Some((panel, key)) = full.split_once('/') {
                            if !key_documented(&docs, panel, key) {
                                undocumented.push(full.clone());
                            }
                        }
                    }
                }
                Ok(ToolOutput::json(json!({ "checked": checked, "undocumented": undocumented })))
            })
        },
    )]
}

/// The settings and selection a presentation capture borrowed, and how to give
/// them back. Only the keys the caller actually patched are restored, so a
/// capture never writes a setting it was not asked about.
struct Presentation {
    settings: Option<Value>,
    selection: Option<Value>,
}

impl Presentation {
    async fn apply(session: &Session, args: &Value) -> Result<Self, String> {
        let mut loan = Presentation { settings: None, selection: None };
        if let Some(patch) = args.get("settings").and_then(Value::as_object) {
            if !patch.is_empty() {
                let live = session.host.call_ok("settings_get", json!({})).await?.result.unwrap_or(Value::Null);
                let live = &live["settings"];
                let restore: serde_json::Map<String, Value> = patch
                    .keys()
                    .filter_map(|k| live.get(k).map(|v| (k.clone(), v.clone())))
                    .collect();
                session.host.call_ok("settings_set", json!({ "patch": patch })).await?;
                loan.settings = Some(Value::Object(restore));
            }
        }
        if b(args, "clear_selection", false) {
            let live = session.host.call_ok("selection", json!({})).await?.result.unwrap_or(Value::Null);
            session.host.call_ok("select_clear", json!({})).await?;
            loan.selection = Some(live["selection"].clone());
        }
        Ok(loan)
    }

    async fn restore(&self, session: &Session) {
        if let Some(patch) = &self.settings {
            let _ = session.host.call("settings_set", json!({ "patch": patch })).await;
        }
        if let Some(sel) = &self.selection {
            let _ = session
                .host
                .call(
                    "selection_set",
                    json!({
                        "solids": sel["solids"].clone(),
                        "faces": sel["faces"].clone(),
                        "edges": sel["edges"].clone(),
                        "datums": sel["datums"].clone(),
                    }),
                )
                .await;
        }
    }
}

pub fn capture_tools(slot: SessionSlot) -> Vec<ToolSpec> {
    let illustration_slot = slot.clone();
    let annotate_slot = slot.clone();
    vec![ToolSpec::new(
        "illustration_capture_many",
        "capture",
        "Capture multiple assembly illustrations using temporary presentation state. Each view starts from the original camera/visibility/selection/settings, optionally activates a saved 3D view, selects components, applies display-only explode translations and captures a PNG. All state is restored on success or failure. Cancellation stops between captures and still restores. No temporary documents or modeling features are created.",
        object_schema(json!({
            "views":{"type":"array","minItems":1,"maxItems":64,"items":{"type":"object","properties":{
                "id":{"type":"string"},"components":{"type":"array","items":{"type":"string"}},
                "visibility":{"type":"object","additionalProperties":{"type":"boolean"}},
                "saved_view":{"type":"string"},"standard_view":{"type":"string"},"camera":{"type":"object"},
                "explode":{"type":"object","description":"component ID to [x,y,z] presentation translation in mm"},
                "settings":{"type":"object"},"clear_selection":{"type":"boolean","default":true},
                "region":{"type":"string","enum":["full","viewport"],"default":"viewport"}
            },"required":["id"],"additionalProperties":false}},
            "max_width":{"type":"integer","minimum":1,"maximum":4096,"default":1024}
        }), &["views"]),
        Annotations { read_only:true, destructive:false, idempotent:false, waits:true },
        move |args| {
            let slot = illustration_slot.clone();
            Box::pin(async move {
                let session = current(&slot).await?;
                let views = args["views"].as_array().filter(|a| !a.is_empty() && a.len() <= 64).ok_or("views must contain 1..=64 views")?.clone();
                let mut ids = std::collections::HashSet::new();
                for view in &views {
                    let id = view["id"].as_str().ok_or("each view needs a string id")?;
                    if !ids.insert(id.to_string()) { return Err(format!("duplicate view id {id}")); }
                }
                // A dropped MCP future signals cancellation but does not abort
                // the owned cleanup task. It finishes the current capture and
                // restores the presentation before stopping.
                let (cancel_tx, mut cancel_rx) = tokio::sync::oneshot::channel::<()>();
                let task = tokio::spawn(async move {
                    wait_idle(&session, 60_000, 2).await?;
                    let begin = session.host.call_ok("illustration_presentation", json!({"action":"begin"})).await?.result.unwrap_or(Value::Null);
                    let document_id = begin["document_id"].clone();
                    let capture = async {
                        let mut captures = Vec::new();
                        let mut images = Vec::new();
                        for view in views {
                            if matches!(cancel_rx.try_recv(), Err(tokio::sync::oneshot::error::TryRecvError::Closed)) { break; }
                            let mapping = session.host.call_ok("illustration_presentation", json!({"action":"view","document_id":document_id,"view":view})).await?.result.unwrap_or(Value::Null);
                            let region = view.get("region").cloned().unwrap_or(json!("viewport"));
                            let (_, img) = shot_with_pointer(&session, region, false, 1).await?;
                            let full = image::encode_png(&img)?;
                            let (shot, path) = session.next_shot();
                            std::fs::write(&path, full).map_err(|e| format!("{}: {e}", path.display()))?;
                            let inline = image::scale_to_width(&img, u(&args,"max_width",1024).clamp(1,4096) as u32);
                            images.push(ToolImage {png:image::encode_png(&inline)?,mime:"image/png"});
                            let manifest = path.with_extension("json");
                            let capture = json!({"id":view["id"],"shot":shot,"path":path.display().to_string(),"manifest":manifest.display().to_string(),"components":mapping["components"],"solids":mapping["solids"],"visibleComponents":mapping["visibleComponents"],"visibleSolids":mapping["visibleSolids"],"camera":mapping["camera"],"requested":view,"document_id":document_id});
                            std::fs::write(&manifest, serde_json::to_vec_pretty(&capture).map_err(|e| e.to_string())?).map_err(|e| format!("{}: {e}", manifest.display()))?;
                            captures.push(capture);
                        }
                        Ok::<_,String>(ToolOutput {json:json!({"captures":captures,"document_id":document_id,"restored":true}),images})
                    }.await;
                    let restored = session.host.call_ok("illustration_presentation",json!({"action":"end","document_id":document_id})).await;
                    match (capture, restored) {
                        (Ok(output), Ok(_)) => Ok(output),
                        (Err(error), Ok(_)) => Err(error),
                        (capture, Err(restore)) => Err(format!("capture outcome: {}; presentation restoration failed: {restore}",capture.err().unwrap_or_else(|| "captured".into()))),
                    }
                });
                let result = task.await.map_err(|e| format!("illustration task: {e}"))?;
                drop(cancel_tx);
                result
            })
        },
    ),ToolSpec::new(
        "screenshot",
        "capture",
        "Capture the composited frame (panels and 3D view). region: full (default), viewport, or {x,y,w,h} in egui points. The image is returned inline, downscaled to max_width (1024), and the full-resolution PNG is written under the session's shots directory. cursor: draw the virtual pointer (cursor_scale makes it bigger). \
         For a clean presentation render pass `settings` and/or `clear_selection`: they apply for THIS capture only and are put back afterwards, so a screenshot never leaves the session looking different than it found it — e.g. `settings: {showVertices: false, showEdges: false}` for a shaded render.",
        object_schema(json!({
            "region": { "oneOf": [{ "type": "string", "enum": ["full", "viewport"] }, { "type": "object", "properties": { "x": {"type":"number"}, "y": {"type":"number"}, "w": {"type":"number"}, "h": {"type":"number"} }, "required": ["x","y","w","h"] }], "default": "full" },
            "max_width": { "type": "integer", "default": 1024 },
            "cursor": { "type": "boolean", "default": true },
            "cursor_scale": { "type": "integer", "default": 1, "minimum": 1, "description": "size of the drawn pointer, as a multiple of the 12 px arrow — raised for a picture that will be downscaled far enough to lose it" },
            "save_as": { "type": "string", "description": "also copy the full-resolution PNG to this path" },
            "settings": { "type": "object", "description": "a settings patch (any `settings_get` key) applied for this capture only and reverted afterwards — showFaces / showEdges / showVertices / wireframe / background / flatShading are the presentation knobs" },
            "clear_selection": { "type": "boolean", "default": false, "description": "drop the selection highlight for this capture, then put the selection back (position-keyed vertex selections do not survive the round trip)" }
        }), &[]),
        Annotations::READ,
        move |args| {
            let slot = slot.clone();
            Box::pin(async move {
                let session = current(&slot).await?;
                let region = args.get("region").cloned().unwrap_or(json!("full"));
                // Presentation state is a LOAN: read what it is, set what the
                // caller asked for, capture, put it back. The restore runs on the
                // error path too, so a failed capture cannot strand the session
                // with vertices switched off.
                let presentation = Presentation::apply(&session, &args).await?;
                let shot = shot_with_pointer(
                    &session,
                    region.clone(),
                    b(&args, "cursor", true),
                    u(&args, "cursor_scale", 1).max(1) as u32,
                )
                .await;
                presentation.restore(&session).await;
                let (mut info, img) = shot?;
                let full_png = image::encode_png(&img)?;
                let (n, path) = session.next_shot();
                std::fs::write(&path, &full_png).map_err(|e| format!("{}: {e}", path.display()))?;
                if let Some(p) = s(&args, "save_as") {
                    std::fs::write(&p, &full_png).map_err(|e| format!("{p}: {e}"))?;
                }
                let inline = image::scale_to_width(&img, u(&args, "max_width", 1024) as u32);
                let inline_png = image::encode_png(&inline)?;
                info["shot"] = json!(n);
                info["path"] = json!(path.display().to_string());
                info["inline_size"] = json!([inline.width(), inline.height()]);
                info["flat"] = json!(image::is_flat(&img));
                session.record("screenshot", &args, true, json!({ "shot": n }));
                Ok(ToolOutput { json: info, images: vec![ToolImage { png: inline_png, mime: "image/png" }] })
            })
        },
    ),
    ToolSpec::new(
        "annotate",
        "capture",
        "Set the DOCS ANNOTATION the host paints over the app's frame — a caption band along the bottom and an amber ring around named widgets — so the next `screenshot` is a captioned walkthrough frame. \
         `highlight` names widgets exactly as `hit_rects` publishes them and is REFUSED if a key is not being published, so a renamed control breaks a walkthrough by name instead of quietly losing its ring. \
         The overlay is paint only: no widget, no hit rect, no document change. It STAYS until it is changed, so clear it with `{}` before a capture that should show the product alone. Only the headless host draws it; a running window answers with an error.",
        object_schema(json!({
            "title": { "type": "string", "description": "a short lead above the caption, repeated on every frame of a walkthrough" },
            "caption": { "type": "string", "description": "the sentence describing what this frame shows" },
            "highlight": { "oneOf": [{ "type": "string" }, { "type": "array", "items": { "type": "string" } }], "description": "widget key(s) from `hit_rects` to ring" },
            "entity": { "description": "a scene entity `{kind, name}` to ring in the 3D view, or an array of them — each is resolved through `locate`, so the ring lands where a click on it would" },
            "point": { "type": "array", "description": "a surface point `[x, y]` in egui points, or an array of them, to ring" },
            "step": { "type": "array", "items": { "type": "integer" }, "minItems": 2, "maxItems": 2, "description": "[n, total], drawn at the right of the band" }
        }), &[]),
        Annotations::READ,
        move |args| {
            let slot = annotate_slot.clone();
            Box::pin(async move {
                let session = current(&slot).await?;
                // `entity` is the 3D half of `highlight`: a face or an edge has
                // no published rectangle, so it is projected to a surface point
                // by the app's OWN `locate` — the same call `click_entity`
                // spends — and handed to the host as a point to ring. Resolving
                // it here rather than in the host keeps the host's overlay
                // knowing nothing about the scene.
                let mut spec = args.clone();
                if let Some(entity) = spec.as_object_mut().and_then(|o| o.remove("entity")) {
                    let wanted = match entity {
                        Value::Array(a) => a,
                        one => vec![one],
                    };
                    let mut points = Vec::new();
                    for e in wanted {
                        let found = session.host.call_ok("locate", e.clone()).await?.result.unwrap_or(Value::Null);
                        if found["visible"].as_bool() != Some(true) {
                            return Err(format!("annotate: {e} is not visible on the surface, so there is nothing to ring"));
                        }
                        points.push(json!([found["x"], found["y"]]));
                    }
                    spec["point"] = Value::Array(points);
                }
                let result = session.host.annotate(spec).await?;
                session.record("annotate", &args, true, result.clone());
                Ok(ToolOutput::json(result))
            })
        },
    )]
}

pub fn document_tools(slot: SessionSlot) -> Vec<ToolSpec> {
    let open_slot = slot.clone();
    let save_slot = slot.clone();
    let import_slot = slot.clone();
    let export_slot = slot.clone();
    vec![
        ToolSpec::new(
            "document_open",
            "document",
            "Open a .nbrep file from disk in a new tab and run it (the server reads the file; the app sees only its content).",
            object_schema(json!({ "path": { "type": "string" }, "timeout_ms": { "type": "integer", "default": 60000 } }), &["path"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let slot = open_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let path = s(&args, "path").ok_or("missing `path`")?;
                    let text = std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))?;
                    let name = std::path::Path::new(&path).file_name().map(|n| n.to_string_lossy().to_string());
                    let mut r = session.host.call_ok("doc_load", json!({ "json": text, "name": name })).await?.result.unwrap_or(Value::Null);
                    r["after"] = after_wait(&session, u(&args, "timeout_ms", 60_000)).await?;
                    session.record("document_open", &args, true, json!({}));
                    Ok(ToolOutput::json(r))
                })
            },
        ),
        ToolSpec::new(
            "document_save",
            "document",
            "Write the active document as .nbrep to `path` and mark it clean. Refuses to overwrite unless overwrite is true.",
            object_schema(json!({ "path": { "type": "string" }, "overwrite": { "type": "boolean", "default": false } }), &["path"]),
            Annotations { read_only: false, destructive: false, idempotent: true, waits: false },
            move |args| {
                let slot = save_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let path = s(&args, "path").ok_or("missing `path`")?;
                    if std::path::Path::new(&path).exists() && !b(&args, "overwrite", false) {
                        return Err(format!("{path} exists; pass overwrite: true"));
                    }
                    let doc = session.host.call_ok("doc_json", json!({})).await?.result.unwrap_or(Value::Null);
                    let text = serde_json::to_string_pretty(&doc["document"]).map_err(|e| e.to_string())?;
                    if let Some(parent) = std::path::Path::new(&path).parent() {
                        let _ = std::fs::create_dir_all(parent);
                    }
                    std::fs::write(&path, &text).map_err(|e| format!("{path}: {e}"))?;
                    session.host.call_ok("doc_mark_clean", json!({})).await?;
                    session.record("document_save", &args, true, json!({}));
                    Ok(ToolOutput::json(json!({ "path": path, "bytes": text.len() })))
                })
            },
        ),
        ToolSpec::new(
            "document_import",
            "document",
            "Import a STEP / IGES / STL / OBJ / 3MF file from disk as an Import 3D Model feature (format by extension unless given).",
            object_schema(json!({ "path": { "type": "string" }, "format": { "type": "string", "enum": ["step", "iges", "stl", "obj", "3mf"] }, "timeout_ms": { "type": "integer", "default": 120000 } }), &["path"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let slot = import_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let path = s(&args, "path").ok_or("missing `path`")?;
                    let ext = std::path::Path::new(&path).extension().map(|e| e.to_string_lossy().to_ascii_lowercase()).unwrap_or_default();
                    let format = s(&args, "format").unwrap_or(match ext.as_str() {
                        "step" | "stp" => "step".into(),
                        "iges" | "igs" => "iges".into(),
                        "stl" => "stl".into(),
                        "obj" => "obj".into(),
                        "3mf" => "3mf".into(),
                        _ => return Err(format!("cannot tell the format of `{path}`; pass `format`")),
                    });
                    let bytes = std::fs::read(&path).map_err(|e| format!("{path}: {e}"))?;
                    let b64 = base64_encode(&bytes);
                    let name = std::path::Path::new(&path).file_name().map(|n| n.to_string_lossy().to_string()).unwrap_or_default();
                    let mut r = session.host.call_ok("doc_import", json!({ "format": format, "name": name, "base64": b64 })).await?.result.unwrap_or(Value::Null);
                    r["after"] = after_wait(&session, u(&args, "timeout_ms", 120_000)).await?;
                    session.record("document_import", &args, true, json!({}));
                    Ok(ToolOutput::json(r))
                })
            },
        ),
        ToolSpec::new(
            "document_export",
            "document",
            "Export the active document to a file: brep (native JSON), step, stl, obj, iges, glb (binary glTF 2.0 of the display mesh), the sheet-metal flat pattern as dxf / svg, the open drawing sheet as sheet_svg, every drawing sheet as sheet_pdf (one PDF, a page per sheet in sheet order), or that drawing set with the live 3D model as sheet_pdf_3d (a 3D box over every placement marked `threeD`, and a last page given over to the model), or, for a document whose PCB board has parts, its manufacturing files: fabrication (a zip of Gerber X2 layers, Excellon drills, pick-and-place and parts-BOM CSVs and a README), pick_place_csv or ecad_bom_csv. The reply carries the file's size and first bytes, for the sheet formats `texts`: every text string the file shows, read back from what was written, and for sheet_pdf_3d `model3d`: its 3D boxes, views, view buttons and U3D size, read back likewise, and for fabrication `files` (each file in the zip), `readBack` (copper flashes, drill hits, outline size and CSV rows, counted from the files) and `warnings`; the CSVs answer `rows`.",
            object_schema(json!({ "format": { "type": "string", "enum": ["brep", "step", "stl", "obj", "iges", "glb", "dxf", "svg", "sheet_svg", "sheet_pdf", "sheet_pdf_3d", "fabrication", "pick_place_csv", "ecad_bom_csv"] }, "path": { "type": "string" } }), &["format", "path"]),
            Annotations { read_only: false, destructive: false, idempotent: true, waits: false },
            move |args| {
                let slot = export_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let path = s(&args, "path").ok_or("missing `path`")?;
                    let format = s(&args, "format").ok_or("missing `format`")?;
                    let r = session.host.call_ok("doc_export", json!({ "format": format })).await?.result.unwrap_or(Value::Null);
                    // A BINARY format (glb) answers in `base64` and leaves
                    // `text` empty; everything else answers in `text`. The
                    // FIELD says which, so no format list has to be kept here.
                    let bytes = match r["base64"].as_str() {
                        Some(b64) => base64_decode(b64)?,
                        None => r["text"].as_str().unwrap_or("").as_bytes().to_vec(),
                    };
                    std::fs::write(&path, &bytes).map_err(|e| format!("{path}: {e}"))?;
                    session.record("document_export", &args, true, json!({}));
                    // `head` is the file's first bytes as text: what a script
                    // needs to assert that a format's MAGIC is there (`%PDF-1.4`,
                    // `solid `, `glTF`) without a second tool to read the file
                    // back. Lossy, because a binary format's first bytes are not
                    // text — the point is the prefix that is.
                    let head = String::from_utf8_lossy(&bytes[..bytes.len().min(16)]).into_owned();
                    let mut out = json!({ "path": path, "format": format, "bytes": bytes.len(), "head": head });
                    // Whatever else the app said ABOUT the file (a paged
                    // format's `pages`) rides through; the content fields do
                    // not, because the file on disk is the content.
                    if let (Some(extra), Some(target)) = (r.as_object(), out.as_object_mut()) {
                        for (key, value) in extra {
                            if !matches!(key.as_str(), "text" | "base64" | "format" | "bytes") {
                                target.insert(key.clone(), value.clone());
                            }
                        }
                    }
                    Ok(ToolOutput::json(out))
                })
            },
        ),
    ]
}

/// The schema of ONE item in `feature_add` / `feature_add_many`: the same
/// shape either way, so a batch is the single-add call repeated.
fn feature_item_schema() -> Value {
    json!({
        "type": { "type": "string", "description": "catalogue type or shortName, e.g. E, P.CU, B, F" },
        "params": { "type": "object", "default": {}, "description": "partial inputParams; every other key is seeded from the schema default" },
        "id": { "type": "string", "description": "the feature's id; also honoured as `params.id`, and minted from the shortName when neither is given" },
        "persistent_data": { "type": "object", "description": "persistentData (e.g. a sketch block)" }
    })
}

fn batch_item_schema() -> Value {
    let mut props = feature_item_schema();
    let extra = json!({
        "attributes": { "type": "object", "description": "Native component occurrenceAttributes" },
        "display_name": { "type": "string", "description": "Component occurrence Name attribute" },
        "material": { "type": "string", "description": "Component occurrence Material attribute" },
        "alias": { "type": "string", "description": "Batch client alias; reference as @alias, @alias:referenceSuffix, @alias/output/0, @alias/face/0 or @alias/edge/0" },
        "depends_on": { "type": "array", "items": {"type":"string"}, "description": "Batch dependency aliases" }
    });
    props.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    props
}

/// Turn one `{type, params, id?, persistent_data?}` item into the complete
/// `{type, inputParams, persistentData?}` feature the app's `feature_add` /
/// `feature_add_many` commands take: seed the schema defaults, overlay the
/// caller's params, mint an id, then validate against the kernel schema.
///
/// The ONE place a feature is built, so a batch cannot drift from a single
/// add — and the only place an id is assigned, so a batch item can never reach
/// the history without one (a feature with an empty id halts the rebuild and
/// no id-keyed `feature_delete` can remove it again).
async fn seed_feature(
    session: &Session,
    item: &Value,
    known: Option<&std::collections::HashSet<String>>,
) -> Result<(String, Value, Vec<String>), String> {
    let ty = s(item, "type").ok_or("missing `type`")?;
    let entry = super::session_feature_entry(session, &ty).await?;
    let id_ = schema::identity(&entry);
    let mut params = validate::merge_params(&schema::defaults_from_entry(&entry), item.get("params").unwrap_or(&json!({})));
    // The id may be given as the item's own `id`, or inside `params` — the
    // kernel's inputParams really does carry one, so writing it there is the
    // natural thing to do and used to be silently overwritten by a minted id
    // (which then broke every reference naming the id the caller chose).
    let id = match s(item, "id").or_else(|| s(item.get("params").unwrap_or(&json!({})), "id")) {
        Some(id) => id,
        None => {
            let r = session.host.call_ok("next_feature_id", json!({ "base": id_.short_name })).await?;
            r.result.and_then(|v| v["id"].as_str().map(str::to_string)).ok_or("next_feature_id gave no id")?
        }
    };
    params["id"] = json!(id);
    let v = validate::validate_entry(&entry, &params, known);
    if !v.ok() {
        return Err(format!("invalid parameters for {} `{id}`: {}", id_.long_name, v.errors.join("; ")));
    }
    let mut feature = json!({ "type": id_.feature_type, "inputParams": params });
    if let Some(pd) = item.get("persistent_data") {
        feature["persistentData"] = pd.clone();
    }
    Ok((id, feature, v.warnings))
}

/// Reuse feature_add's schema validator without reserving live IDs. The native
/// transaction subsequently validates identities, aliases and the whole graph.
fn batch_validation_errors(args: &Value) -> Vec<Value> {
    fn check(items: &[Value], prefix: &str, errors: &mut Vec<Value>) {
        for (i, item) in items.iter().enumerate() {
            let ty = item["type"].as_str().unwrap_or("");
            // A nested part can pin a different plugin schema from its parent.
            // Its runner validates the exact pinned callback, so the global
            // built-in catalogue cannot prevalidate namespaced extension nodes.
            if ty.contains('/') { continue; }
            let mut patch = item.get("params").or_else(|| item.get("inputParams")).cloned().unwrap_or(json!({}));
            if matches!(ty, "ACOMP" | "ASSEMBLY COMPONENT") { if let Some(o) = patch.as_object_mut() { o.remove("occurrenceAttributes"); } }
            let mut params = validate::merge_params(&schema::defaults(ty), &patch);
            if let Some(o) = params.as_object_mut() { o.insert("id".into(), item.get("id").cloned().unwrap_or(json!("batch_validation"))); }
            let v = validate::validate(ty, &params, None);
            for message in v.errors { errors.push(json!({"path":format!("{prefix}[{i}].params"),"position":i,"alias":item["alias"],"message":message})); }
        }
    }
    let mut errors = Vec::new();
    if let Some(items) = args["features"].as_array() { check(items, "features", &mut errors); }
    if let Some(parts) = args["parts"].as_array() {
        for (i, part) in parts.iter().enumerate() { if let Some(items) = part["document"]["features"].as_array() { check(items, &format!("parts[{i}].document.features"), &mut errors); } }
    }
    errors
}

pub fn feature_tools(slot: SessionSlot) -> Vec<ToolSpec> {
    let add_slot = slot.clone();
    let many_slot = slot.clone();
    let set_slot = slot.clone();
    vec![
        ToolSpec::new(
            "feature_add",
            "features",
            "Add a feature by catalogue type with a partial params object: every schema key is seeded from its default, `params` is overlaid (nested keys such as boolean.operation allowed), an id is assigned unless given, the result is validated against the kernel schema, then the feature is appended and run. Returns id, index, the run report and the listing. \
             The SOLID a feature produced is `after.report.featureOutputs[<feature id>]` — read it rather than assuming a name: a boolean names its result after its target when subtracting, but after its FIRST TOOL when unioning or intersecting.",
            {
                let mut props = feature_item_schema();
                let o = props.as_object_mut().expect("object");
                o.insert("wait".into(), json!({ "type": "boolean", "default": true }));
                o.insert("timeout_ms".into(), json!({ "type": "integer", "default": 60000 }));
                object_schema(props, &["type"])
            },
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let slot = add_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let known = known_names(&session).await;
                    let (id, feature, warnings) = seed_feature(&session, &args, known.as_ref()).await?;
                    let mut r = session.host.call_ok("feature_add", json!({ "feature": feature })).await?.result.unwrap_or(Value::Null);
                    if !warnings.is_empty() {
                        r["warnings"] = json!(warnings);
                    }
                    if b(&args, "wait", true) {
                        r["after"] = after_wait(&session, u(&args, "timeout_ms", 60_000)).await?;
                    }
                    session.record("feature_add", &args, true, json!({ "id": id }));
                    Ok(ToolOutput::json(r))
                })
            },
        ),
        ToolSpec::new(
            "feature_add_many",
            "features",
            "Append a native feature batch. The legacy fast lane uses one history run; the structured lane evaluates dependency stages and commits one undo checkpoint. Each item takes the same `{type, params, id?, persistent_data?}` shape as `feature_add` and is seeded, given an id and validated the same way. \
             ATOMIC in the history — if any item fails to validate nothing is appended, and the error names the item's position and id (the id counter still advances, so a refused batch leaves a gap in the numbering). \
             Set structured or rollback_on_error, or supply aliases/parts/constraints, for native assembly transactions: aliases resolve in dependency order and geometry errors return per-item outcomes. The legacy fast lane reserves IDs during validation; the structured lane rolls counters back with the document. \
             A later item may reference what an earlier one builds. \
             The legacy response includes ids and after.report.featureOutputs. Structured responses include items (position, alias, id, status, committed, generatedSolids, generatedReferences), aliases, outputs, constraint diagnostics and transaction coverage. Full failure details are retrieved with geometry_diagnostics.",
            object_schema(json!({
                "features": { "type": "array", "items": { "type": "object", "properties": batch_item_schema(), "required": ["type"], "additionalProperties": false } },
                "parts": { "type": "array", "items": {"type":"object", "properties":{"alias":{"type":"string"}, "document":{"type":"object"}}, "required":["alias","document"], "additionalProperties":false}, "description":"Native editable part documents, referenced by @alias in ACOMP partName; document.features accepts feature items or native descriptors" },
                "constraints": { "type": "array", "items": {"type":"object"}, "description":"Native assembly constraint type, params, optional alias and enabled" },
                "expressions": {"type":"string"},
                "metadata": {"type":"object", "description":"Native document metadata patch"},
                "rollback_on_error": {"type":"boolean", "default":false, "description":"Restore recipe, library, instances, constraints, metadata, counters and undo/redo on rebuild failure. Runtime generations/caches are invalidated."},
                "structured": {"type":"boolean", "default":false, "description":"Return per-item evaluated/failed/not_evaluated results and dependency diagnostics"},
                "wait": { "type": "boolean", "default": true },
                "timeout_ms": { "type": "integer", "default": 60000 }
            }), &["features"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let slot = many_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let items = args
                        .get("features")
                        .and_then(Value::as_array)
                        .ok_or("missing `features` (an array of {type, params, …} items)")?
                        .clone();
                    if items.is_empty() && ["parts", "constraints"].iter().all(|k| args[*k].as_array().is_none_or(|a| a.is_empty())) {
                        return Err("`features` is empty".into());
                    }
                    if b(&args, "structured", false) || b(&args, "rollback_on_error", false)
                        || ["parts", "constraints", "expressions", "metadata"].iter().any(|k| args.get(*k).is_some())
                        || items.iter().any(|f| ["alias", "depends_on", "attributes", "display_name", "material"].iter().any(|key| f.get(*key).is_some())) {
                        let errors = batch_validation_errors(&args);
                        if !errors.is_empty() {
                            let results: Vec<Value> = items.iter().enumerate().map(|(i,item)| {
                                let item_errors: Vec<&Value> = errors.iter().filter(|e| e["path"].as_str().is_some_and(|p| p.starts_with(&format!("features[{i}].")))).collect();
                                json!({"position":i,"alias":item["alias"],"id":item["id"],"status":if item_errors.is_empty() {"not_evaluated"} else {"failed"},"committed":false,"validationErrors":item_errors})
                            }).collect();
                            return Ok(ToolOutput::json(json!({"success":false,"status":"validation_failed","validationErrors":errors,"items":results,"modelChanged":false})));
                        }
                        // The native transaction owns allocation, dependency resolution,
                        // execution and rollback; no ID reservations cross its boundary.
                        let mut request = args.clone();
                        request.as_object_mut().unwrap().remove("wait");
                        request.as_object_mut().unwrap().remove("timeout_ms");
                        request["structured"] = json!(true);
                        wait_idle(&session, u(&args, "timeout_ms", 60_000), 2).await?;
                        let r = session.host.call_ok("feature_add_many", request).await?.result.unwrap_or(Value::Null);
                        session.record("feature_add_many", &args, r["success"] == true, r.clone());
                        return Ok(ToolOutput::json(r));
                    }
                    // Seed and validate EVERY item before the history sees any of
                    // them: a batch that would half-apply is refused whole, so a
                    // malformed item can never poison the run.
                    let mut known = known_names(&session).await;
                    let mut ids = Vec::with_capacity(items.len());
                    let mut features = Vec::with_capacity(items.len());
                    let mut warnings = Vec::new();
                    for (index, item) in items.iter().enumerate() {
                        let (id, feature, mut item_warnings) = seed_feature(&session, item, known.as_ref())
                            .await
                            .map_err(|e| format!("features[{index}]: {e} (nothing was added)"))?;
                        // A later item may reference what an earlier one builds —
                        // a boolean onto the cube two lines up is the whole point
                        // of a batch. The scene does not have that name yet (the
                        // run happens once, at the end), so carry each item's id
                        // forward: a primitive's solid takes its feature id.
                        if let Some(known) = known.as_mut() {
                            known.insert(id.clone());
                        }
                        for w in item_warnings.drain(..) {
                            warnings.push(format!("features[{index}] `{id}`: {w}"));
                        }
                        ids.push(id);
                        features.push(feature);
                    }
                    let mut r = session
                        .host
                        .call_ok("feature_add_many", json!({ "features": features }))
                        .await?
                        .result
                        .unwrap_or(Value::Null);
                    r["ids"] = json!(ids);
                    if !warnings.is_empty() {
                        r["warnings"] = json!(warnings);
                    }
                    if b(&args, "wait", true) {
                        r["after"] = after_wait(&session, u(&args, "timeout_ms", 60_000)).await?;
                    }
                    session.record("feature_add_many", &args, true, json!({ "ids": ids }));
                    Ok(ToolOutput::json(r))
                })
            },
        ),
        ToolSpec::new(
            "feature_set_params",
            "features",
            "Merge a patch into a feature's inputParams (nested keys such as boolean.operation allowed), validate, and rerun from it.",
            object_schema(json!({ "id": { "type": "string" }, "patch": { "type": "object" }, "wait": { "type": "boolean", "default": true }, "timeout_ms": { "type": "integer", "default": 60000 } }), &["id", "patch"]),
            Annotations { read_only: false, destructive: false, idempotent: false, waits: true },
            move |args| {
                let slot = set_slot.clone();
                Box::pin(async move {
                    let session = current(&slot).await?;
                    let id = s(&args, "id").ok_or("missing `id`")?;
                    let cur = session.host.call_ok("feature_params", json!({ "id": id })).await?.result.unwrap_or(Value::Null);
                    let ty = cur["type"].as_str().unwrap_or("").to_string();
                    let merged = validate::merge_params(&cur["inputParams"], args.get("patch").unwrap_or(&json!({})));
                    let known = known_names(&session).await;
                    let entry = super::session_feature_entry(&session, &ty).await?;
                    let v = validate::validate_entry(&entry, &merged, known.as_ref());
                    if !v.ok() {
                        return Err(format!("invalid parameters for {ty}: {}", v.errors.join("; ")));
                    }
                    let mut r = session.host.call_ok("feature_set_params", json!({ "id": id, "input_params": merged })).await?.result.unwrap_or(Value::Null);
                    if !v.warnings.is_empty() {
                        r["warnings"] = json!(v.warnings);
                    }
                    if b(&args, "wait", true) {
                        r["after"] = after_wait(&session, u(&args, "timeout_ms", 60_000)).await?;
                    }
                    session.record("feature_set_params", &args, true, json!({}));
                    Ok(ToolOutput::json(r))
                })
            },
        ),
    ]
}

/// Every reference name in the scene right now (solids, faces, edges).
async fn known_names(session: &Session) -> Option<std::collections::HashSet<String>> {
    let r = session.host.call_ok("scene_entities", json!({})).await.ok()?.result?;
    let mut set = std::collections::HashSet::new();
    for solid in r["solids"].as_array()? {
        if let Some(n) = solid["name"].as_str() {
            set.insert(n.to_string());
        }
        for k in ["faces", "edges"] {
            for n in solid[k].as_array().into_iter().flatten() {
                if let Some(n) = n.as_str() {
                    set.insert(n.to_string());
                }
            }
        }
    }
    Some(set)
}

fn base64_encode(bytes: &[u8]) -> String {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD.encode(bytes)
}

/// The inverse, for a BINARY export coming back through the JSON result.
fn base64_decode(text: &str) -> Result<Vec<u8>, String> {
    use base64::Engine as _;
    base64::engine::general_purpose::STANDARD
        .decode(text)
        .map_err(|e| format!("document_export: the app's base64 payload does not decode: {e}"))
}

