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
//! The split with `web/webmcp.js`: everything about the tools is Rust — the
//! list ([`tool_list`]), the direct tier ([`DIRECT`], [`compact`]), the
//! invocation. The JS file only registers what the list says with whichever
//! `modelContext` object the browser provides, and turns a reply's PNGs into
//! blob URLs. The browser API is still moving (it was `navigator.modelContext`
//! and `provideContext` before `document.modelContext` and `registerTool`),
//! and that churn belongs in the one small file that names it.
//!
//! Not every browser takes the whole set. Chrome's agent registers all of it;
//! ChatGPT's browser disables a page whose "WebMCP configuration exceeds
//! supported limits" (observed 2026-10-14 with the full set; the limit is
//! undocumented, and published accounts land at about 22 tools with
//! descriptions under 1,500 characters). So the page registers two tiers:
//! [`DIRECT`], a small set of the everyday tools with [`compact`] descriptions
//! and schemas, and three gateway tools — `brep_tools` lists every tool,
//! `brep_tool` returns one's full description and schema, `brep_call` invokes
//! any of them — through which the rest is reachable, in full, with the same
//! semantics. A page may ask for the full set instead (`web/webmcp.js`,
//! `all: true`; `?webmcp=all`).
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
    tools.extend(compose::session_tools(cx));
    tools.extend(app::app_tools(slot.clone(), describe));
    tools.extend(compose::pointer_tools(slot.clone()));
    tools.extend(compose::capture_tools(slot.clone()));
    tools.extend(compose::document_tools(slot.clone()));
    tools.extend(compose::feature_tools(slot.clone()));
    tools.extend(compose::registry_tools(slot));
    tools
}

/// The tool list the page registers: `[{name, title, group, summary,
/// description, inputSchema, annotations: {readOnlyHint, consequentialHint},
/// waits}]`, one per tool of the set, in the set's order. The schemas are
/// the tools' own (a waiting tool already carries the idle contract's `wait`
/// / `timeout_ms` knobs); [`SCREENSHOT`] gains the page's `inline` knob.
pub fn tool_list(set: &ToolSet) -> Value {
    let mut out = Vec::with_capacity(set.specs.len());
    for spec in &set.specs {
        let mut schema = spec.input_schema.clone();
        if spec.name == SCREENSHOT {
            if let Some(o) = schema.as_object_mut() {
                o.entry("properties").or_insert_with(|| json!({}));
            }
            if let Some(props) = schema.get_mut("properties").and_then(Value::as_object_mut) {
                props.insert(
                    "inline".into(),
                    json!({ "type": "boolean", "default": false, "description": "carry the inline PNG in the reply as `data_url` (large); the page keeps the full capture as a blob URL either way" }),
                );
            }
        }
        out.push(json!({
            "name": spec.name,
            "title": spec.name,
            "group": spec.group,
            "summary": cut(&spec.doc, SUMMARY_CHARS),
            "description": format!("[{}] {}", spec.group, spec.doc),
            "inputSchema": schema,
            "annotations": {
                "readOnlyHint": spec.annotations.read_only,
                "consequentialHint": spec.annotations.destructive,
            },
            "waits": spec.annotations.waits,
        }));
    }
    Value::Array(out)
}

/// The everyday tools a page registers directly, compacted: inspect, learn
/// the features and their schemas, add and edit features, undo, pick by
/// name, look. Everything else is reached through the gateway tools. Each
/// name is a tool of the set (the test below holds the list to that).
pub const DIRECT: &[&str] = &[
    "frame_info",
    "doc_new",
    "history_listing",
    "feature_catalogue",
    "feature_schema",
    "feature_add",
    "feature_params",
    "feature_set_params",
    "feature_delete",
    "undo",
    "redo",
    "scene_entities",
    "selection",
    "selection_set",
    "click_entity",
    "mass_properties",
    "zoom_to_fit",
    "standard_view",
    SCREENSHOT,
];

/// How much of a doc string a direct tool's description keeps, and a
/// gateway listing's summary. Cuts land on a sentence end where one falls
/// inside the budget, so a description reads whole.
pub const DESCRIPTION_CHARS: usize = 480;
pub const SUMMARY_CHARS: usize = 140;
/// The cap on a compacted schema's per-property description.
pub const PROPERTY_CHARS: usize = 160;

/// `text` within `max` characters: whole when it fits, otherwise cut at the
/// last sentence end inside the budget, or at the budget with an ellipsis.
pub fn cut(text: &str, max: usize) -> String {
    let text = text.trim();
    if text.chars().count() <= max {
        return text.to_string();
    }
    let head: String = text.chars().take(max).collect();
    if let Some(end) = head.rfind(". ").or_else(|| head.rfind(".\n")) {
        if end > max / 3 {
            return head[..=end].to_string();
        }
    }
    let mut cut: String = head.chars().take(max.saturating_sub(1)).collect();
    cut.push('…');
    cut
}

/// A direct tool as the page registers it: the description cut to
/// [`DESCRIPTION_CHARS`], the schema without its `$schema` and `title`,
/// `additionalProperties: false` at the top (ChatGPT's browser requires
/// it), and each property's description cut to [`PROPERTY_CHARS`]. The
/// full form stays reachable through `brep_tool`.
pub fn compact(tool: &Value) -> Value {
    let mut schema = tool["inputSchema"].clone();
    if let Some(o) = schema.as_object_mut() {
        o.remove("$schema");
        o.remove("title");
        o.insert("additionalProperties".into(), json!(false));
        if let Some(props) = o.get_mut("properties").and_then(Value::as_object_mut) {
            for prop in props.values_mut() {
                if let Some(d) = prop.get("description").and_then(Value::as_str).map(|d| cut(d, PROPERTY_CHARS)) {
                    prop["description"] = json!(d);
                }
            }
        }
    }
    json!({
        "name": tool["name"],
        "title": tool["title"],
        "description": cut(tool["description"].as_str().unwrap_or(""), DESCRIPTION_CHARS),
        "inputSchema": schema,
        "annotations": tool["annotations"],
    })
}

/// The direct tier: [`DIRECT`] in order, each [`compact`]ed.
pub fn direct_list(set: &ToolSet) -> Value {
    let all = tool_list(set);
    let all = all.as_array().cloned().unwrap_or_default();
    Value::Array(
        DIRECT
            .iter()
            .filter_map(|name| all.iter().find(|t| t["name"] == *name))
            .map(compact)
            .collect(),
    )
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
    use crate::automation::command::{describe, Envelope as AppEnvelope, Reply as AppReply};
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
                Ok(reply) => return convert(reply),
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

    /// The app's reply in the host protocol's shape — the same conversion
    /// the window embed makes.
    fn convert(mut r: AppReply) -> Reply {
        let blob = r.blob.take();
        let (id, frame) = (r.id, r.frame);
        match serde_json::to_value(&r).map_err(|e| e.to_string()).and_then(|v| Reply::from_app(v, blob)) {
            Ok(reply) => reply,
            Err(e) => Reply::err(id, frame, e),
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

    /// The tool list as JSON text (see [`tool_list`]); resolves once the
    /// session has attached.
    #[wasm_bindgen]
    pub fn webmcp_tools() -> js_sys::Promise {
        future_to_promise(async move {
            let set = tools().await.map_err(js_err)?;
            Ok(JsValue::from_str(&tool_list(&set).to_string()))
        })
    }

    /// The direct tier as JSON text (see [`direct_list`]).
    #[wasm_bindgen]
    pub fn webmcp_direct() -> js_sys::Promise {
        future_to_promise(async move {
            let set = tools().await.map_err(js_err)?;
            Ok(JsValue::from_str(&direct_list(&set).to_string()))
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
        let spec = set.get(name).ok_or_else(|| format!("unknown tool `{name}`"))?;
        let mut args = js_to_value(&args)?;
        let inline = take_inline(name, &mut args);
        let out = (spec.handler)(args).await?;
        to_js(out, inline)
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
        const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        let mut out = String::with_capacity(bytes.len().div_ceil(3) * 4);
        for chunk in bytes.chunks(3) {
            let b = [chunk[0], *chunk.get(1).unwrap_or(&0), *chunk.get(2).unwrap_or(&0)];
            let n = (u32::from(b[0]) << 16) | (u32::from(b[1]) << 8) | u32::from(b[2]);
            out.push(TABLE[(n >> 18) as usize & 63] as char);
            out.push(TABLE[(n >> 12) as usize & 63] as char);
            out.push(if chunk.len() > 1 { TABLE[(n >> 6) as usize & 63] as char } else { '=' });
            out.push(if chunk.len() > 2 { TABLE[n as usize & 63] as char } else { '=' });
        }
        out
    }
}

