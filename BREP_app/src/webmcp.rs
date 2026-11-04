//! The browser's tool surface: WebMCP for the wasm build, serving the SAME
//! tool set the desktop MCP server does.
//!
//! [WebMCP](https://webmachinelearning.github.io/webmcp/) is the web
//! platform's way for a page to hand tools to the browser's own agent:
//! `document.modelContext.registerTool({name, description, inputSchema,
//! execute})`. The native app serves MCP out of `brep_mcp_core`
//! (`brep-app --mcp`, `brep-mcp`): the command registry adapted tool by tool,
//! plus the server's own compositions — `click_entity`, `drag_widget`,
//! `hotkey`, the schema-seeded `feature_add`, `document_open` / `document_save`
//! by path, `screenshot` with its presentation loan and inline image,
//! `wait_state`, `hit_keys_check`, the session recorder. The browser build has
//! no server process, but it has the same crate: `brep_mcp_core` without its
//! `server` feature compiles for wasm32, and this module is its HOST — the
//! page's answer to the host protocol (`brep_mcp_core::host`), the way
//! `src/mcp.rs` is the window's and `BREP_mcp` is the headless harness's.
//! One tool set, three hosts; nothing here lists a tool by hand.
//!
//! What a host supplies is the `Platform` (`brep_mcp_core::host::Platform`):
//! the clock is `Date.now()`, a pause is a `setTimeout`, a task runs on the
//! page's event loop, a kept screenshot is a `blob:` URL the page holds, and
//! a FILE is a name in the app's document store — IndexedDB outside a PLM
//! session, the PLM server inside one — reached through the app's own
//! `store_read` / `store_write` commands, since the tools run outside the
//! frame and the store lives inside it. So `document_save {path:
//! "bracket.nbrep"}` saves where the File menu's Save As would, and
//! `document_export {path: "bracket.step"}` lands as a download, where the
//! Export dialog's would.
//!
//! The page registers the server's front door (`FRONT_DOOR`): `run_script`,
//! `script_result`, `script_halt`, `screenshot`, `input` and `files`. A program runs in
//! a Web Worker of its own (`web/code-worker.js`, Boa in this same wasm,
//! `src/code_worker.rs`); its `brep.<group>.<tool>()` calls come back here and
//! run the tool set's handlers, and the page terminates the worker on a halt
//! or the wall-clock limit — the browser's counterpart of the native
//! worker process the server kills. Six tools also sit under every browser
//! agent's tool-count limit (ChatGPT's browser disabled a page with the full
//! set, 2026-10-14).
//!
//! The split with `web/webmcp.js`: everything about the tools is Rust — the
//! list (`front_door_list`), the executions, the invocation. The JS file only
//! registers what the list says with whichever `modelContext` object the
//! browser provides, and turns a reply's PNGs into blob URLs. The browser API
//! is still moving (it was `navigator.modelContext` and `provideContext`
//! before `document.modelContext` and `registerTool`), and that churn belongs
//! in the one small file that names it.
//!
//! The non-wasm half of this file is pure and compiles natively so the shape
//! of what the page registers is under test (`tests` below, over the very
//! tool set the page builds); the wasm half is the host and the
//! `#[wasm_bindgen]` exports `web/webmcp.js` imports.
use brep_mcp_core::tools::app::SessionSlot;
use brep_mcp_core::tools::compose::ServerContext;
use brep_mcp_core::tools::{ToolSet, ToolSpec};
use serde_json::{json, Value};
use std::sync::Arc;

/// The tool whose reply carries pictures the page can also inline.
pub const SCREENSHOT: &str = "screenshot";

/// The whole tool set the page serves, assembled exactly as the server
/// assembles it for a live session (`brep_mcp_core::server`): the catalogue
/// tools, the session tools, every command the registry describes, and the
/// compositions. `describe` is the registry's own description — the same
/// JSON `describe_commands` answers, read in-process here.
pub fn compose_tools(cx: Arc<ServerContext>, slot: SessionSlot, describe: &Value) -> Vec<ToolSpec> {
    use brep_mcp_core::tools::{app, compose, session_engine_tools};
    let mut tools = session_engine_tools(slot.clone());
    tools.extend(compose::described_session_tools(cx, Some(describe)));
    tools.extend(app::app_tools(slot.clone(), describe));
    tools.extend(compose::described_pointer_tools(slot.clone(), describe));
    tools.extend(compose::described_capture_tools(slot.clone(), describe));
    tools.extend(compose::document_tools(slot.clone(), describe));
    tools.extend(compose::feature_tools(slot.clone(), describe));
    tools.extend(compose::registry_tools(slot.clone()));
    let file_handler: Arc<dyn Fn(Value) -> brep_mcp_core::tools::ToolFuture + Send + Sync> = Arc::new(move |args| {
        let slot = slot.clone();
        Box::pin(async move {
            let session = app::current(&slot).await?;
            let reply = session.host.call_ok("files", args).await?;
            Ok(brep_mcp_core::tools::ToolOutput::json(reply.result.unwrap_or(Value::Null)))
        })
    });
    tools.extend(brep_mcp_core::files::script_tools(file_handler.clone()));
    let mut files = ToolSpec::new("files", "script", "Manage files under named documents/workspace roots in the browser store. Paths are relative; writes settle before success. Overwriting requires overwrite:true; directory copy/delete requires recursive:true. Transfer limit256KiB; list page limit256; tree entry limit1024.", brep_mcp_core::files::input_schema(), brep_mcp_core::tools::Annotations::DESTRUCTIVE, move |args| file_handler(args));
    files.output_schema = brep_mcp_core::files::output_schema();
    tools.push(files);
    tools
}

/// The page's front door, the server's (`brep_mcp_core::cad::tools::FRONT_DOOR`):
/// the six tools an agent lists. Everything else is a function of the
/// program `run_script` runs, `await brep.<group>.<tool>(args)`, generated
/// from this same set (`brep_mcp_core::script_sdk`).
pub const FRONT_DOOR: &[&str] = &["run_script", "script_result", "script_halt", SCREENSHOT, "input", "files"];

/// The tool list the page registers: the [`FRONT_DOOR`], `[{name, title,
/// description, inputSchema, annotations: {readOnlyHint, consequentialHint}}]`.
/// [`SCREENSHOT`] is the capture tool's schema with the page's `inline` knob.
pub fn front_door_list(set: &ToolSet) -> Value {
    let wait = json!({ "type": "integer", "minimum": 0, "maximum": 600000, "description": "how long to wait for the result, ms (default 60000); a longer run answers its status and keeps going" });
    let mut shot = set.specs.iter().find(|t| t.name == SCREENSHOT).map(|t| t.input_schema.clone()).unwrap_or_else(|| json!({ "type": "object" }));
    if let Some(o) = shot.as_object_mut() {
        o.entry("properties").or_insert_with(|| json!({}));
    }
    if let Some(props) = shot.get_mut("properties").and_then(Value::as_object_mut) {
        props.insert(
            "inline".into(),
            json!({ "type": "boolean", "default": false, "description": "carry the inline PNG in the reply as `data_url` (large); the page keeps the full capture as a blob URL either way" }),
        );
    }
    let tool = |name: &str, description: &str, schema: Value, read_only: bool| {
        json!({ "name": name, "title": name, "description": description, "inputSchema": schema, "annotations": { "readOnlyHint": read_only, "consequentialHint": !read_only } })
    };
    json!([
        tool(
            "run_script",
            "Run a JavaScript program on the CAD app in this page: `export default async function main()` returning JSON. Every app tool is a script function, `await brep.<group>.<tool>(args)` (import { brep } from \"@brep/cad\"). It runs in a Web Worker the page halts on request. Waits up to wait_ms for the result; a longer run answers its executionId and status and keeps going (script_result, script_halt). JavaScript only: TypeScript and the cad.* operation client run in the native app and brep-mcp.",
            json!({ "type": "object", "properties": { "source": { "type": "string", "minLength": 1, "maxLength": 262144 }, "language": { "enum": ["javascript"], "default": "javascript" }, "wait_ms": wait, "key": { "type": "string", "minLength": 1, "maxLength": 128, "description": "a retry key of your choosing: the same key with the same program answers the SAME execution" } }, "required": ["source"], "additionalProperties": false }),
            false,
        ),
        tool("script_result", "A script's result once it has finished (waiting up to wait_ms, default 0), else its status.", json!({ "type": "object", "properties": { "executionId": { "type": "string" }, "wait_ms": wait }, "required": ["executionId"], "additionalProperties": false }), true),
        tool("script_halt", "Halt a running script now: its Web Worker is terminated wherever it is. App commands it had already sent run on.", json!({ "type": "object", "properties": { "executionId": { "type": "string" } }, "required": ["executionId"], "additionalProperties": false }), false),
        tool(SCREENSHOT, "A PNG of the app as it is now (region: full | viewport | {x,y,w,h}, egui points). Optional save_as writes a file.", shot, false),
        tool(
            "input",
            "Inject mouse and keyboard input into the app: `steps`, each `{tool, args}` with tool one of the pointer/keyboard tools (click, drag, hotkey, type_text, key, text, pointer_move, pointer_down, pointer_up, wheel, click_entity, drag_entity, modifiers_set, paste, click_gap, pointer_gone), run in order. Answers each step's result; stops at the first that fails.",
            json!({ "type": "object", "properties": { "steps": { "type": "array", "maxItems": 256, "items": { "type": "object", "properties": { "tool": { "type": "string" }, "args": { "type": "object" } }, "required": ["tool"], "additionalProperties": false } } }, "required": ["steps"], "additionalProperties": false }),
            false,
        ),
        tool("files", "Bounded file management under named roots in this page's persistent store: list/stat/read/write/mkdir/copy/move/delete. Read-only operations do not mutate; replacement requires overwrite:true and directory copy/delete requires recursive:true.", brep_mcp_core::files::input_schema(), false),
    ])
}

/// Take the page's own `inline` knob out of a [`SCREENSHOT`] call's
/// arguments before the tool sees them (its schema denies unknown fields).
pub fn take_inline(name: &str, args: &mut Value) -> bool {
    if name != SCREENSHOT {
        return false;
    }
    args.as_object_mut()
        .and_then(|o| o.remove("inline"))
        .and_then(|v| v.as_bool())
        .unwrap_or(false)
}

#[cfg(target_arch = "wasm32")]
pub use web::*;

#[cfg(target_arch = "wasm32")]
mod web {
    use super::*;
    use crate::automation::command::{describe, Envelope as AppEnvelope};
    use crate::automation::queue::AutomationQueue;
    use brep_mcp_core::host::{Backend, HostInfo, HostRequest, KeptShot, Platform, PlatformFuture, Reply};
    use brep_mcp_core::tools::compose::attach_session;
    use brep_mcp_core::tools::ToolOutput;
    use std::cell::{Cell, RefCell};
    use std::future::Future;
    use std::path::PathBuf;
    use std::pin::Pin;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::task::{Context, Poll};
    use wasm_bindgen::prelude::*;
    use wasm_bindgen_futures::{future_to_promise, spawn_local};

    /// How long one command may wait for the app to answer: the native
    /// embed's bound, for the same reason (a tab that is hidden does not
    /// paint, and a paint is what answers).
    const REPLY_TIMEOUT_MS: f64 = 120_000.0;

    /// How many kept screenshots the page holds as blob URLs before the
    /// oldest is released.
    const KEEP_SHOTS: usize = 8;

    thread_local! {
        /// The tool set, once the session is attached; `None` until then.
        static TOOLS: RefCell<Option<Arc<ToolSet>>> = const { RefCell::new(None) };
        /// Why attaching failed, when it did.
        static FAILED: RefCell<Option<String>> = const { RefCell::new(None) };
        /// Whether `attach` has run at all.
        static ATTACHED: Cell<bool> = const { Cell::new(false) };
        /// The kept screenshots' blob URLs, oldest first.
        static SHOTS: RefCell<Vec<String>> = const { RefCell::new(Vec::new()) };
        static NEXT_ID: Cell<u64> = const { Cell::new(0) };
    }

    /// A pause on the page's event loop: a `setTimeout` that wakes the
    /// task. The future holds no JS value — the timer callback is handed to
    /// the browser inside `poll` — so it stays `Send`, as every tool future
    /// must be.
    struct Timer {
        ms: u64,
        armed: bool,
        fired: Arc<AtomicBool>,
    }

    impl Timer {
        fn new(ms: u64) -> Self {
            Self { ms, armed: false, fired: Arc::new(AtomicBool::new(false)) }
        }
    }

    impl Future for Timer {
        type Output = ();
        fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<()> {
            if self.fired.load(Ordering::Acquire) {
                return Poll::Ready(());
            }
            if !self.armed {
                self.armed = true;
                let fired = self.fired.clone();
                let waker = cx.waker().clone();
                let callback = Closure::once_into_js(move || {
                    fired.store(true, Ordering::Release);
                    waker.wake();
                });
                match web_sys::window() {
                    Some(win) => {
                        let _ = win.set_timeout_with_callback_and_timeout_and_arguments_0(callback.unchecked_ref(), self.ms as i32);
                    }
                    None => {
                        // No window (a worker): there is no timer to wait on;
                        // yield once and continue.
                        self.fired.store(true, Ordering::Release);
                        cx.waker().wake_by_ref();
                    }
                }
            }
            Poll::Pending
        }
    }

    fn sleep(ms: u64) -> Timer {
        Timer::new(ms)
    }

    fn now_ms() -> f64 {
        js_sys::Date::now()
    }

    /// Submit one command and wait for its reply without blocking: the app
    /// answers from a later frame, and the page's one thread has to paint
    /// that frame, so the wait yields to the browser between polls.
    async fn forward(q: &Arc<AutomationQueue>, cmd: String, args: Value) -> Reply {
        let id = NEXT_ID.with(|n| {
            n.set(n.get() + 1);
            n.get()
        });
        let rx = q.submit(AppEnvelope { id, cmd, args });
        let started = now_ms();
        loop {
            match rx.try_recv() {
                Ok(reply) => return reply.into_host(),
                Err(std::sync::mpsc::TryRecvError::Disconnected) => return Reply::err(id, q.frame(), "the app dropped the command"),
                Err(std::sync::mpsc::TryRecvError::Empty) => {
                    if q.is_poisoned() {
                        return Reply::err(id, q.frame(), "session poisoned");
                    }
                    if now_ms() - started > REPLY_TIMEOUT_MS {
                        return Reply::err(id, q.frame(), format!("no reply from the app within {} s (is the tab visible?)", REPLY_TIMEOUT_MS / 1000.0));
                    }
                    sleep(16).await;
                }
            }
        }
    }

    async fn call_ok(q: &Arc<AutomationQueue>, cmd: &str, args: Value) -> Result<Value, String> {
        let r = forward(q, cmd.to_string(), args).await;
        if r.ok {
            Ok(r.result.unwrap_or(Value::Null))
        } else {
            Err(r.error.unwrap_or_else(|| format!("{cmd} failed")))
        }
    }

    /// The page as the tools' platform (see the module doc).
    struct BrowserPlatform {
        queue: Arc<AutomationQueue>,
    }

    impl Platform for BrowserPlatform {
        fn now_ms(&self) -> f64 {
            now_ms()
        }

        fn sleep(&self, ms: u64) -> PlatformFuture<'_, ()> {
            Box::pin(sleep(ms))
        }

        fn spawn(&self, task: Pin<Box<dyn Future<Output = ()> + Send + 'static>>) {
            spawn_local(task);
        }

        fn stamp(&self) -> String {
            format!("{}-web", (now_ms() / 1000.0) as u64)
        }

        fn session_dir(&self, root: &std::path::Path, id: &str) -> Result<PathBuf, String> {
            // Nothing is made: the page keeps a session's shots as blob
            // URLs and its log in memory. The name is what `session_info`
            // reports.
            Ok(root.join(id))
        }

        fn append_log(&self, _dir: &std::path::Path, line: &str) {
            log::debug!("webmcp: {line}");
        }

        fn read_text(&self, path: String) -> PlatformFuture<'_, Result<String, String>> {
            let q = self.queue.clone();
            Box::pin(async move {
                let r = call_ok(&q, "store_read", json!({ "name": path })).await?;
                match r["text"].as_str() {
                    Some(text) if r["exists"] == true => Ok(text.to_string()),
                    _ => Err(format!("{path}: not in the page's document store (store_list names what is)")),
                }
            })
        }

        fn read_bytes(&self, path: String) -> PlatformFuture<'_, Result<Vec<u8>, String>> {
            let q = self.queue.clone();
            Box::pin(async move {
                let r = call_ok(&q, "store_read", json!({ "name": path })).await?;
                match r["text"].as_str() {
                    Some(text) if r["exists"] == true => Ok(text.as_bytes().to_vec()),
                    _ => Err(format!(
                        "{path}: not in the page's document store (store_list names what is); a file on your machine goes through `doc_import` with its content, or the page's file chooser (`file_pick`)"
                    )),
                }
            })
        }

        fn write_file(&self, path: String, bytes: Vec<u8>, overwrite: bool) -> PlatformFuture<'_, Result<String, String>> {
            let q = self.queue.clone();
            Box::pin(async move {
                // Text rides as text (a document, a STEP); anything else as
                // base64 — the command types the file by its name either way.
                let args = match String::from_utf8(bytes) {
                    Ok(text) => json!({ "name": path, "text": text, "overwrite": overwrite }),
                    Err(e) => json!({ "name": path, "base64": base64(e.as_bytes()), "overwrite": overwrite }),
                };
                let r = call_ok(&q, "store_write", args).await?;
                Ok(r["name"].as_str().unwrap_or(&path).to_string())
            })
        }

        fn keep_shot(&self, _dir: &std::path::Path, _n: u64, png: &[u8], _manifest: Option<&Value>) -> Result<KeptShot, String> {
            let url = blob_url(png)?;
            SHOTS.with(|s| {
                let mut s = s.borrow_mut();
                s.push(url.clone());
                while s.len() > KEEP_SHOTS {
                    let old = s.remove(0);
                    let _ = web_sys::Url::revoke_object_url(&old);
                }
            });
            // The manifest is the capture record itself, which the reply
            // carries; the page has nowhere to write it beside the shot.
            Ok(KeptShot { path: url, manifest: None })
        }
    }

    /// A `blob:` URL for `png`, held by the page until released.
    fn blob_url(png: &[u8]) -> Result<String, String> {
        let array = js_sys::Uint8Array::new_with_length(png.len() as u32);
        array.copy_from(png);
        let parts = js_sys::Array::of1(&array);
        let options = web_sys::BlobPropertyBag::new();
        options.set_type("image/png");
        let blob = web_sys::Blob::new_with_u8_array_sequence_and_options(&parts, &options).map_err(|_| "blob create failed".to_string())?;
        web_sys::Url::create_object_url_with_blob(&blob).map_err(|_| "object URL failed".to_string())
    }

    /// Hand the host the app's queue and start the session. Called from the
    /// eframe creation closure (`lib.rs`), once the app exists: the host loop
    /// starts at once, the session attaches from the first frame the app
    /// paints, and the exports below answer once it has.
    pub fn attach(queue: Arc<AutomationQueue>, adapter: String) {
        if ATTACHED.with(|a| a.replace(true)) {
            return;
        }
        // The state registry publishes only for a host; this page has one.
        crate::automation::registry::set_enabled(true);
        let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<HostRequest>();
        let q = queue.clone();
        spawn_local(async move {
            while let Some(req) = rx.recv().await {
                let q = q.clone();
                match req {
                    HostRequest::Command { envelope, reply } => {
                        spawn_local(async move {
                            let _ = reply.send(forward(&q, envelope.cmd, envelope.args).await);
                        });
                    }
                    // A capture is the app's own `screenshot` command (egui's
                    // viewport screenshot, completed on a later frame); the
                    // PNG rides in the reply blob — as the window embed does.
                    HostRequest::Screenshot { region, reply } => {
                        spawn_local(async move {
                            let r = forward(&q, "screenshot".into(), json!({ "region": region })).await;
                            let _ = reply.send(if !r.ok {
                                Err(r.error.unwrap_or_else(|| "screenshot failed".into()))
                            } else {
                                match r.blob {
                                    Some(png) => Ok((r.result.unwrap_or(Value::Null), png)),
                                    None => Err("the screenshot reply carried no image".into()),
                                }
                            });
                        });
                    }
                    // The DOCS ANNOTATION overlay belongs to the headless
                    // harness; a page draws the product, as the window does.
                    HostRequest::Annotate { reply, .. } => {
                        let _ = reply.send(Err(
                            "the page draws no annotation overlay: docs walkthroughs are recorded by the headless host (brep-mcp)".to_string(),
                        ));
                    }
                    // The app is not the host's to stop.
                    HostRequest::Stop => {}
                }
            }
        });
        let platform: Arc<dyn Platform> = Arc::new(BrowserPlatform { queue });
        let info = HostInfo { backend: "browser", adapter, platform: "web", width: 0.0, height: 0.0, ppp: 1.0 };
        let slot: SessionSlot = Arc::new(tokio::sync::RwLock::new(None));
        let cx = Arc::new(ServerContext {
            slot: slot.clone(),
            session_root: PathBuf::from("browser:/sessions"),
            backend: Backend::Attached { tx, info, platform },
            // The registry is static in the page; the list never changes.
            on_tools_changed: Arc::new(|| {}),
        });
        spawn_local(async move {
            match attach_session(&cx, true).await {
                Ok(_) => {
                    let set = ToolSet::new(compose_tools(cx, slot, &describe()));
                    TOOLS.with(|t| *t.borrow_mut() = Some(Arc::new(set)));
                }
                Err(e) => FAILED.with(|f| *f.borrow_mut() = Some(format!("the session did not attach: {e}"))),
            }
        });
    }

    /// The tool set, once the session has attached.
    async fn tools() -> Result<Arc<ToolSet>, String> {
        loop {
            if let Some(set) = TOOLS.with(|t| t.borrow().clone()) {
                return Ok(set);
            }
            if let Some(why) = FAILED.with(|f| f.borrow().clone()) {
                return Err(why);
            }
            if !ATTACHED.with(|a| a.get()) {
                return Err("the app has not started".into());
            }
            sleep(16).await;
        }
    }

    fn js_err(e: String) -> JsValue {
        JsValue::from_str(&e)
    }

    /// The tool list as JSON text (see [`front_door_list`]); resolves once
    /// the session has attached.
    #[wasm_bindgen]
    pub fn webmcp_tools() -> js_sys::Promise {
        future_to_promise(async move {
            let set = tools().await.map_err(js_err)?;
            Ok(JsValue::from_str(&front_door_list(&set).to_string()))
        })
    }

    /// The kept screenshots' `blob:` URLs, oldest first.
    #[wasm_bindgen]
    pub fn webmcp_shots() -> js_sys::Array {
        SHOTS.with(|s| s.borrow().iter().map(|u| JsValue::from_str(u)).collect())
    }

    /// Call one tool by name with a JSON-compatible `args` object. Resolves
    /// to `{result, images}`: the tool's JSON result and its PNGs (each a
    /// `Uint8Array`). Rejects with the error text when the tool refused or
    /// the app did not answer.
    #[wasm_bindgen]
    pub fn webmcp_invoke(name: String, args: JsValue) -> js_sys::Promise {
        future_to_promise(async move { invoke(&name, args).await.map_err(js_err) })
    }

    async fn invoke(name: &str, args: JsValue) -> Result<JsValue, String> {
        let set = tools().await?;
        let mut args = js_to_value(&args)?;
        if FRONT_DOOR.contains(&name) { brep_mcp_core::script_sdk::validate_front_door(name, &args)?; }
        let inline = take_inline(name, &mut args);
        let out = match name {
            "run_script" => ToolOutput::json(run_script(&set, &args).await),
            "script_result" => {
                let id = args.get("executionId").and_then(Value::as_str).unwrap_or_default().to_string();
                ToolOutput::json(settle(&id, args.get("wait_ms").and_then(Value::as_u64).unwrap_or(0)).await)
            }
            "script_halt" => ToolOutput::json(halt(args.get("executionId").and_then(Value::as_str).unwrap_or_default())),
            "files" => {
                let spec = set.get("files").ok_or("no files tool")?;
                (spec.handler)(args).await?
            }
            SCREENSHOT => {
                let spec = set.specs.iter().find(|t| t.name == SCREENSHOT).ok_or("no capture tool")?;
                (spec.handler)(args).await?
            }
            "input" => {
                let mut results = Vec::new();
                for step in args.get("steps").and_then(Value::as_array).cloned().unwrap_or_default() {
                    let tool = step.get("tool").and_then(Value::as_str).unwrap_or_default().to_string();
                    let spec = set.get(&tool).filter(|s| matches!(s.group, "pointer" | "keyboard")).ok_or_else(|| format!("`{tool}` is not a pointer or keyboard tool"))?;
                    let out = (spec.handler)(step.get("args").cloned().unwrap_or(json!({}))).await.map_err(|e| format!("{tool}: {e}"))?;
                    results.push(json!({ "tool": tool, "result": out.json }));
                }
                ToolOutput::json(json!({ "steps": results }))
            }
            other => {
                let hint = set
                    .get(other)
                    .filter(|t| brep_mcp_core::script_sdk::script_callable(t.group, &t.name))
                    .map(|t| format!("call it from run_script as `await brep.{}.{}(args)`", brep_mcp_core::script_sdk::ident(t.group), brep_mcp_core::script_sdk::method_name(t.group, &t.name)))
                    .unwrap_or_else(|| "it is not a tool of this page".into());
                return Err(format!("`{other}` is not a front-door tool: {hint}"));
            }
        };
        to_js(out, inline)
    }

    // ---- run_script in the browser: one Web Worker per execution ----------

    /// The limits a browser execution runs under: the native defaults that a
    /// page can enforce (wall clock, sizes, outstanding host calls) — the
    /// worker is terminated at the wall. CPU time and heap are not measurable
    /// in a page; the native runner's limits for them do not apply here.
    const WALL_MS: i32 = 15 * 60 * 1000;
    const SOURCE_BYTES: usize = 256 << 10;
    const OUTSTANDING_CALLS: u32 = 8;
    const LOG_BYTES: usize = 64 << 10;
    const RESULT_BYTES: usize = 64 << 10;
    /// Finished executions kept for `script_result`, newest last.
    const RETAINED: usize = 64;

    struct WebExec {
        id: String,
        worker: web_sys::Worker,
        source: String,
        key: Option<String>,
        state: &'static str,
        result: Option<Value>,
        logs: Vec<String>,
        log_bytes: usize,
        outstanding: u32,
        timer: i32,
        // Held for the worker's life; dropped only once another execution
        // evicts this one (never from inside their own invocation).
        _onmessage: Closure<dyn FnMut(web_sys::MessageEvent)>,
        _wall: Closure<dyn FnMut()>,
    }

    thread_local! {
        static EXECS: RefCell<Vec<WebExec>> = const { RefCell::new(Vec::new()) };
        static NEXT_EXEC: Cell<u64> = const { Cell::new(0) };
    }

    fn fail(code: &str, message: impl Into<String>) -> Value {
        json!({ "ok": false, "error": { "code": code, "message": message.into() } })
    }

    async fn run_script(set: &Arc<ToolSet>, args: &Value) -> Value {
        let source = args.get("source").and_then(Value::as_str).unwrap_or_default().to_string();
        if args.get("language").and_then(Value::as_str).is_some_and(|l| l != "javascript") {
            return fail("compile_failed", "the browser runs JavaScript: TypeScript is type-checked and run by the native app and brep-mcp");
        }
        if source.len() > SOURCE_BYTES {
            return fail("resource_limit", format!("source is larger than {SOURCE_BYTES} bytes"));
        }
        let key = args.get("key").and_then(Value::as_str).map(str::to_string);
        // A retry key: the same program answers the execution it started.
        let known = key.as_ref().and_then(|k| EXECS.with(|e| e.borrow().iter().find(|x| x.key.as_ref() == Some(k)).map(|x| (x.id.clone(), x.source == source))));
        let id = match known {
            Some((_, false)) => return fail("idempotency_conflict", "this key was used for a different program"),
            Some((id, true)) => id,
            None => match start(set, source, key) {
                Ok(id) => id,
                Err(e) => return fail("internal_error", e),
            },
        };
        settle(&id, args.get("wait_ms").and_then(Value::as_u64).unwrap_or(60_000).min(600_000)).await
    }

    fn start(set: &Arc<ToolSet>, source: String, key: Option<String>) -> Result<String, String> {
        if EXECS.with(|e| e.borrow().iter().filter(|x| x.result.is_none()).count()) >= 2 {
            return Err("too many scripts are running; wait for a result or halt a script".into());
        }
        if EXECS.with(|e| { let xs = e.borrow(); xs.len() >= RETAINED && !xs.iter().any(|x| x.result.is_some() && x.key.is_none()) }) {
            return Err("execution retention is full; retry keys are preserved until the page reloads".into());
        }
        let n = NEXT_EXEC.with(|c| {
            c.set(c.get() + 1);
            c.get()
        });
        let id = format!("exec_web_{n}");
        let options = web_sys::WorkerOptions::new();
        options.set_type(web_sys::WorkerType::Module);
        let worker = web_sys::Worker::new_with_options("./code-worker.js", &options).map_err(|e| format!("the code worker did not start: {e:?}"))?;
        let exec_id = id.clone();
        let onmessage = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
            if let Some(text) = event.data().as_string() {
                from_worker(&exec_id, &text);
            }
        }) as Box<dyn FnMut(web_sys::MessageEvent)>);
        worker.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
        let exec_id = id.clone();
        let wall = Closure::wrap(Box::new(move || {
            finish(&exec_id, "failed", None, Some(json!({ "code": "resource_limit", "message": format!("the program ran past its {} ms wall-clock limit and was stopped", WALL_MS) })));
        }) as Box<dyn FnMut()>);
        let timer = web_sys::window()
            .and_then(|w| w.set_timeout_with_callback_and_timeout_and_arguments_0(wall.as_ref().unchecked_ref(), WALL_MS).ok())
            .unwrap_or(0);
        let module = brep_mcp_core::script_sdk::browser_module(&brep_mcp_core::script_sdk::script_tools(&set.specs));
        let first = json!({ "source": source, "sdk": module, "manifest": "{}" }).to_string();
        // The page's own compiled module first: the worker instantiates it
        // instead of fetching and compiling the app's wasm again.
        worker.post_message(&wasm_bindgen::module()).map_err(|e| format!("the code worker refused the module: {e:?}"))?;
        worker.post_message(&JsValue::from_str(&first)).map_err(|e| format!("the code worker refused its program: {e:?}"))?;
        EXECS.with(|e| {
            let mut execs = e.borrow_mut();
            while execs.len() >= RETAINED {
                match execs.iter().position(|x| x.result.is_some() && x.key.is_none()) {
                    Some(i) => drop(execs.remove(i)),
                    None => break,
                }
            }
            execs.push(WebExec { id: id.clone(), worker, source, key, state: "running", result: None, logs: Vec::new(), log_bytes: 0, outstanding: 0, timer, _onmessage: onmessage, _wall: wall });
        });
        Ok(id)
    }

    /// One protocol message from execution `id`'s worker.
    fn from_worker(id: &str, text: &str) {
        let Ok(msg) = serde_json::from_str::<Value>(text) else { return };
        if let Some(line) = msg.get("log").and_then(Value::as_str) {
            EXECS.with(|e| {
                if let Some(x) = e.borrow_mut().iter_mut().find(|x| x.id == id) {
                    if x.log_bytes + line.len() <= LOG_BYTES {
                        x.log_bytes += line.len();
                        x.logs.push(line.to_string());
                    }
                }
            });
        } else if let Some(done) = msg.get("done") {
            let output = done.get("ok").cloned();
            let mut error = done.get("error").cloned();
            let mut state = if output.is_some() { "succeeded" } else { "failed" };
            if output.as_ref().is_some_and(|o| o.to_string().len() > RESULT_BYTES) {
                state = "failed";
                error = Some(json!({ "code": "resource_limit", "message": format!("main's result is larger than {RESULT_BYTES} bytes") }));
            }
            finish(id, state, if state == "succeeded" { output } else { None }, error);
        } else if let (Some(call), Some(method), Some(body)) = (msg.get("call").and_then(Value::as_u64), msg.get("method").and_then(Value::as_str), msg.get("body").and_then(Value::as_str)) {
            let admitted = EXECS.with(|e| {
                let mut execs = e.borrow_mut();
                let x = execs.iter_mut().find(|x| x.id == id && x.result.is_none())?;
                (x.outstanding < OUTSTANDING_CALLS).then(|| {
                    x.outstanding += 1;
                })
            });
            let (id, method, body) = (id.to_string(), method.to_string(), body.to_string());
            spawn_local(async move {
                let response = if admitted.is_some() {
                    host_call(&method, &body).await
                } else {
                    fail("resource_limit", format!("more than {OUTSTANDING_CALLS} host calls outstanding"))
                };
                EXECS.with(|e| {
                    let mut execs = e.borrow_mut();
                    if let Some(x) = execs.iter_mut().find(|x| x.id == id) {
                        if admitted.is_some() {
                            x.outstanding -= 1;
                        }
                        if x.result.is_none() {
                            let reply = json!({ "reply": call, "body": response.to_string() }).to_string();
                            let _ = x.worker.post_message(&JsValue::from_str(&reply));
                        }
                    }
                });
            });
        }
    }

    /// A program's bridge call: `tool.<name>` runs that script-callable
    /// tool's handler, answered as the service's `Response`.
    async fn host_call(method: &str, body: &str) -> Value {
        let Some(name) = method.strip_prefix("tool.") else {
            return fail("method_unknown", format!("`{method}`: the cad.* operation service runs in the native app and brep-mcp; in the browser use brep.<group>.<tool>()"));
        };
        let set = match tools().await {
            Ok(set) => set,
            Err(e) => return fail("host_stopped", e),
        };
        let Some(spec) = set.specs.iter().find(|t| t.name == name && brep_mcp_core::script_sdk::script_callable(t.group, &t.name)) else {
            return fail("method_unknown", format!("no script function for tool `{name}`"));
        };
        let args: Value = serde_json::from_str(body).unwrap_or(json!({}));
        match (spec.handler)(args).await {
            Ok(out) => {
                let mut value = out.json;
                if !out.images.is_empty() {
                    let images: Vec<Value> = out.images.iter().map(|i| json!({ "mediaType": i.mime, "dataUrl": format!("data:{};base64,{}", i.mime, base64(&i.png)) })).collect();
                    match value.as_object_mut() {
                        Some(o) => {
                            o.insert("images".into(), Value::Array(images));
                        }
                        None => value = json!({ "value": value, "images": images }),
                    }
                }
                json!({ "ok": true, "value": value })
            }
            Err(e) => fail("evaluation_failed", e),
        }
    }

    /// End execution `id`: terminate its worker, stop its wall timer, keep
    /// its result. A second end (the wall after a halt) changes nothing.
    fn finish(id: &str, state: &'static str, output: Option<Value>, error: Option<Value>) {
        EXECS.with(|e| {
            let mut execs = e.borrow_mut();
            let Some(x) = execs.iter_mut().find(|x| x.id == id && x.result.is_none()) else { return };
            x.worker.terminate();
            if let Some(w) = web_sys::window() {
                w.clear_timeout_with_handle(x.timer);
            }
            x.state = state;
            let error = error.map(|mut e| {
                e["executionId"] = json!(id);
                e
            });
            x.result = Some(json!({
                "executionId": id,
                "state": state,
                "typeChecking": "not_applicable",
                "output": output,
                "error": error,
                "operations": [],
                "logText": x.logs.join("\n"),
            }));
        });
    }

    /// `script_halt`: terminate the worker now.
    fn halt(id: &str) -> Value {
        let known = EXECS.with(|e| e.borrow().iter().any(|x| x.id == id));
        if !known {
            return fail("execution_unknown", "no retained execution has that id");
        }
        finish(id, "cancelled", None, Some(json!({ "code": "cancelled", "message": "halted by script_halt" })));
        status(id)
    }

    fn status(id: &str) -> Value {
        EXECS.with(|e| match e.borrow().iter().find(|x| x.id == id) {
            Some(x) => match &x.result {
                Some(r) => json!({ "ok": true, "value": r }),
                None => json!({ "ok": true, "value": { "running": true, "status": { "executionId": id, "state": x.state } } }),
            },
            None => fail("execution_unknown", "no retained execution has that id"),
        })
    }

    /// The JavaScript editor's runner in the browser: the page's own
    /// executions, the same Web Worker runner and `brep.*` functions an
    /// agent's `run_script` gets.
    pub struct PageScripts;

    impl crate::javascript::ScriptHost for PageScripts {
        fn ready(&self) -> Result<bool, String> {
            if let Some(why) = FAILED.with(|f| f.borrow().clone()) {
                return Err(why);
            }
            if !ATTACHED.with(|a| a.get()) {
                return Err("the page's tool host has not started".into());
            }
            Ok(TOOLS.with(|t| t.borrow().is_some()))
        }
        fn run(&self, source: &str) -> Result<String, String> {
            let set = TOOLS.with(|t| t.borrow().clone()).ok_or("the page's tools are not attached yet")?;
            if source.len() > SOURCE_BYTES {
                return Err(format!("source is larger than {SOURCE_BYTES} bytes"));
            }
            start(&set, source.to_string(), None)
        }
        fn result(&self, id: &str) -> Result<Option<Value>, String> {
            let s = status(id);
            if s["ok"] != true {
                return Err(s["error"]["message"].as_str().unwrap_or("unknown execution").to_string());
            }
            Ok((s["value"]["running"] != true).then(|| s["value"].clone()))
        }
        fn halt(&self, id: &str) -> Result<(), String> {
            let r = halt(id);
            if r["ok"] == true { Ok(()) } else { Err(r["error"]["message"].as_str().unwrap_or("unknown execution").to_string()) }
        }
    }

    /// The result once it has ended (waiting up to `wait_ms`), else its status.
    async fn settle(id: &str, wait_ms: u64) -> Value {
        let deadline = js_sys::Date::now() + wait_ms as f64;
        loop {
            let s = status(id);
            if s["ok"] != true || s["value"]["running"] != true || js_sys::Date::now() >= deadline {
                return s;
            }
            sleep(25).await;
        }
    }

    /// A JS value (an object, `null` or `undefined`) as the tool's
    /// arguments: `null`/`undefined` mean none.
    fn js_to_value(args: &JsValue) -> Result<Value, String> {
        if args.is_null() || args.is_undefined() {
            return Ok(json!({}));
        }
        let text = js_sys::JSON::stringify(args).map_err(|_| "arguments are not JSON".to_string())?;
        let text: String = text.into();
        serde_json::from_str(&text).map_err(|e| format!("arguments: {e}"))
    }

    /// `{result, images}` for the page; with `inline`, the result also
    /// carries `data_urls` (and `data_url`, the first) for an agent that
    /// reads the picture from the reply.
    fn to_js(out: ToolOutput, inline: bool) -> Result<JsValue, String> {
        let mut result = out.json;
        if inline && !out.images.is_empty() {
            let urls: Vec<String> = out.images.iter().map(|i| format!("data:{};base64,{}", i.mime, base64(&i.png))).collect();
            result["data_url"] = json!(urls[0]);
            result["data_urls"] = json!(urls);
        }
        let obj = js_sys::Object::new();
        let parsed = js_sys::JSON::parse(&result.to_string()).map_err(|_| "the reply is not JSON".to_string())?;
        js_sys::Reflect::set(&obj, &JsValue::from_str("result"), &parsed).map_err(|_| "result")?;
        let images = js_sys::Array::new();
        for image in &out.images {
            images.push(&js_sys::Uint8Array::from(image.png.as_slice()));
        }
        js_sys::Reflect::set(&obj, &JsValue::from_str("images"), &images).map_err(|_| "images")?;
        Ok(obj.into())
    }

    /// Standard base64: a data URL's payload, a binary file's content.
    fn base64(bytes: &[u8]) -> String {
        use base64::Engine as _;
        base64::engine::general_purpose::STANDARD.encode(bytes)
    }
}

