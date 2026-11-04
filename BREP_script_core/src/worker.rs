//! The code-mode worker: one restricted JavaScript execution in a process of
//! its own, on Boa, unchanged. A host binary becomes this process when it is
//! started with the hidden worker flag (`BREP_mcp_core` `cad::code::worker_entry`).
//!
//! The host that starts this process owns it: it watches the process's CPU
//! time, memory and age, and KILLS it — on an agent's halt
//! (`cad_code_cancel`) or a limit — which stops any running script, however
//! it loops. Nothing in the host's address space is reachable from the
//! script, and a killed worker leaves nothing behind but the CAD operations
//! it had already handed to the service.
//!
//! Protocol, one JSON object per line:
//!
//! - stdin, first line: `{"source", "sdk", "manifest"}` — the program (an ES
//!   module with `export default async function main()`), the `@brep/cad`
//!   module source, and the manifest identity as JSON text;
//! - stdin, then: `{"reply": id, "body": text}` — a host call's answer;
//! - stdout: `{"call": id, "method": m, "body": text}` — a host call (the
//!   global `__brep_host(method, body)` the SDK uses, answered by a Promise);
//!   `{"log": line}`; and last `{"done": {"ok": json}}` or
//!   `{"done": {"error": {"code", "message"}}}`.
//!
//! The only importable module is `@brep/cad`; there is no filesystem,
//! network, process, timer or environment binding.
//!
//! The same protocol drives the browser's code worker (a Web Worker running
//! this crate in the app's wasm, messages in place of lines): `Realm` is the
//! event-driven core both share — `Realm::begin` runs the program until it
//! waits on the host or ends, `Realm::answer` delivers one host reply and
//! runs on. `main` is that core over stdin and stdout.
use crate::{describe, install_console, Logs};
use boa_engine::builtins::promise::PromiseState;
use boa_engine::module::{ModuleLoader, Referrer};
use boa_engine::object::builtins::{JsFunction, JsPromise};
use boa_engine::{js_string, Context, JsError, JsNativeError, JsResult, JsValue, Module, NativeFunction, Source};
use serde_json::{json, Value};
use std::cell::RefCell;
use std::collections::HashMap;
#[cfg(not(target_arch = "wasm32"))]
use std::io::{BufRead, Write};
use std::rc::Rc;

struct OnlySdk {
    sdk: String,
    parsed: RefCell<Option<Module>>,
}

impl ModuleLoader for OnlySdk {
    async fn load_imported_module(
        self: Rc<Self>,
        _referrer: Referrer,
        request: boa_engine::module::ModuleRequest,
        context: &RefCell<&mut Context>,
    ) -> JsResult<Module> {
        let spec = request.specifier().to_std_string_escaped();
        if spec != "@brep/cad" {
            return Err(JsNativeError::typ().with_message(format!("cannot import `{spec}`: the only importable module is @brep/cad")).into());
        }
        if let Some(m) = self.parsed.borrow().as_ref() {
            return Ok(m.clone());
        }
        let mut cx = context.borrow_mut();
        let module = Module::parse(Source::from_bytes(&self.sdk).with_path(std::path::Path::new("@brep/cad")), None, &mut cx)?;
        *self.parsed.borrow_mut() = Some(module.clone());
        Ok(module)
    }
}

thread_local! {
    static PENDING: RefCell<HashMap<u64, (JsFunction, JsFunction)>> = RefCell::new(HashMap::new());
    static NEXT: RefCell<u64> = const { RefCell::new(0) };
    /// Where this realm's protocol messages go: stdout natively, the Web
    /// Worker's `postMessage` in the browser.
    static SINK: RefCell<Option<Box<dyn Fn(&Value)>>> = RefCell::new(None);
}

fn emit(line: &Value) {
    SINK.with(|s| {
        if let Some(sink) = s.borrow().as_ref() {
            sink(line);
        }
    });
}

fn done_error(code: &str, message: String) -> Value {
    json!({ "done": { "error": { "code": code, "message": message } } })
}

/// `__brep_host(method, body)`: send the call to the host and answer a
/// pending Promise, settled when the host's reply arrives.
fn host_call(_: &JsValue, args: &[JsValue], cx: &mut Context) -> JsResult<JsValue> {
    let text = |i: usize| args.get(i).and_then(JsValue::as_string).map(|s| s.to_std_string_escaped()).unwrap_or_default();
    let (method, body) = (text(0), text(1));
    let (promise, resolvers) = JsPromise::new_pending(cx);
    let id = NEXT.with(|n| {
        *n.borrow_mut() += 1;
        *n.borrow()
    });
    PENDING.with(|p| p.borrow_mut().insert(id, (resolvers.resolve, resolvers.reject)));
    emit(&json!({ "call": id, "method": method, "body": body }));
    Ok(promise.into())
}

/// Boa's own runtime limits (loop iterations per frame, recursion, stack)
/// are uncatchable in JavaScript; reported as the resource limit they are.
fn fail(code: &str, e: JsError, cx: &mut Context) -> Value {
    let limit = e.as_engine().is_some();
    let message = describe(e, cx);
    done_error(if limit || message.contains("RuntimeLimitError") { "resource_limit" } else { code }, message)
}

/// One execution, driven by its host's messages.
pub struct Realm {
    cx: Context,
    logs: Logs,
    sent: usize,
    main: JsPromise,
}

impl Realm {
    /// Start an execution from its start message (`{source, sdk,
    /// manifest}`), its protocol messages going to `sink`: run the program
    /// until it waits on the host (`Some`) or ends (`None`; its `done` sent).
    pub fn begin(start: &str, sink: Box<dyn Fn(&Value)>) -> Option<Realm> {
        SINK.with(|s| *s.borrow_mut() = Some(sink));
        let start: Value = match serde_json::from_str(start) {
            Ok(v) => v,
            Err(e) => {
                emit(&done_error("internal_error", format!("bad start message: {e}")));
                return None;
            }
        };
        let field = |k: &str| start.get(k).and_then(Value::as_str).unwrap_or_default().to_string();
        let (source, sdk, manifest) = (field("source"), field("sdk"), field("manifest"));
        let loader = Rc::new(OnlySdk { sdk, parsed: RefCell::new(None) });
        let mut cx = match Context::builder().module_loader(loader).build() {
            Ok(cx) => cx,
            Err(e) => {
                emit(&done_error("internal_error", e.to_string()));
                return None;
            }
        };
        cx.runtime_limits_mut().set_loop_iteration_limit(crate::LOOP_ITERATION_LIMIT);
        cx.runtime_limits_mut().set_recursion_limit(crate::RECURSION_LIMIT);
        let logs: Logs = Rc::new(RefCell::new(Vec::new()));
        let setup = (|| -> JsResult<()> {
            install_console(&mut cx, &logs)?;
            cx.register_global_property(js_string!("__brep_manifest"), js_string!(manifest), boa_engine::property::Attribute::all())?;
            cx.register_global_builtin_callable(js_string!("__brep_host"), 2, NativeFunction::from_fn_ptr(host_call))?;
            Ok(())
        })();
        if let Err(e) = setup {
            let line = done_error("internal_error", describe(e, &mut cx));
            return Self::end(&logs, &mut 0, line);
        }
        let mut sent = 0;
        match Self::start_main(&source, &mut cx) {
            Ok(main) => {
                let mut realm = Realm { cx, logs, sent, main };
                if realm.progress() {
                    None
                } else {
                    Some(realm)
                }
            }
            Err(line) => Self::end(&logs, &mut sent, line),
        }
    }

    /// Evaluate the module and call its `main`: its Promise, or the `done`
    /// line when it cannot start.
    fn start_main(source: &str, cx: &mut Context) -> Result<JsPromise, Value> {
        let module = Module::parse(Source::from_bytes(source).with_path(std::path::Path::new("main.js")), None, cx).map_err(|e| fail("compile_failed", e, cx))?;
        let evaluated = module.load_link_evaluate(cx);
        cx.run_jobs().map_err(|e| fail("job_failed", e, cx))?;
        match evaluated.state() {
            PromiseState::Fulfilled(_) => {}
            PromiseState::Rejected(v) => {
                // A link failure (a refused import) or a top-level throw.
                let message = describe(JsError::from_opaque(v), cx);
                let code = if message.contains("cannot import") || message.contains("SyntaxError") { "compile_failed" } else { "script_failed" };
                return Err(done_error(code, message));
            }
            PromiseState::Pending => {
                return Err(done_error("invalid_arguments", "module evaluation is pending without progress: top-level code may not wait on the host".into()))
            }
        }
        let main = module.namespace(cx).get(js_string!("default"), cx).map_err(|e| fail("script_failed", e, cx))?;
        let Some(main) = main.as_callable() else {
            return Err(done_error("invalid_arguments", "the program must `export default async function main()`".into()));
        };
        let returned = main.call(&JsValue::undefined(), &[], cx).map_err(|e| fail("script_failed", e, cx))?;
        returned
            .as_object()
            .and_then(|o| JsPromise::from_object(o.clone()).ok())
            .ok_or_else(|| done_error("invalid_arguments", "main() must be async (return a Promise)".into()))
    }

    /// Deliver one host message (`{reply, body}`) and run on: whether the
    /// execution has ended (its `done` sent).
    pub fn answer(&mut self, message: &str) -> bool {
        let Ok(msg) = serde_json::from_str::<Value>(message) else { return false };
        let (Some(id), Some(body)) = (msg.get("reply").and_then(Value::as_u64), msg.get("body").and_then(Value::as_str)) else { return false };
        if let Some((resolve, _)) = PENDING.with(|p| p.borrow_mut().remove(&id)) {
            if let Err(e) = resolve.call(&JsValue::undefined(), &[js_string!(body).into()], &mut self.cx) {
                let line = fail("job_failed", e, &mut self.cx);
                Self::end(&self.logs, &mut self.sent, line);
                return true;
            }
        }
        self.progress()
    }

    /// The host closed the bridge with the execution still waiting.
    pub fn abandon(&mut self) {
        Self::end(&self.logs, &mut self.sent, done_error("internal_error", "the host closed the bridge".into()));
    }

    /// Run every job, then end if `main` settled or nothing can progress:
    /// whether it ended.
    fn progress(&mut self) -> bool {
        if let Err(e) = self.cx.run_jobs() {
            let line = fail("job_failed", e, &mut self.cx);
            Self::end(&self.logs, &mut self.sent, line);
            return true;
        }
        flush_logs(&self.logs, &mut self.sent);
        let line = match self.main.state() {
            PromiseState::Fulfilled(v) => match v.to_json(&mut self.cx) {
                Ok(json) => json!({ "done": { "ok": json.unwrap_or(Value::Null) } }),
                Err(e) => done_error("invalid_arguments", format!("main's result is not JSON: {}", describe(e, &mut self.cx))),
            },
            PromiseState::Rejected(v) => {
                let message = describe(JsError::from_opaque(v), &mut self.cx);
                done_error(if message.contains("RuntimeLimitError") { "resource_limit" } else { "script_failed" }, message)
            }
            PromiseState::Pending if PENDING.with(|p| p.borrow().is_empty()) => {
                done_error("invalid_arguments", "main is pending without progress: no host call is outstanding and no job can run".into())
            }
            // Park on the host: nothing can run until a reply arrives.
            PromiseState::Pending => return false,
        };
        Self::end(&self.logs, &mut self.sent, line);
        true
    }

    /// Send the remaining logs and the `done` line; drop unsettled host calls
    /// (they hold this realm's functions).
    fn end(logs: &Logs, sent: &mut usize, line: Value) -> Option<Realm> {
        flush_logs(logs, sent);
        emit(&line);
        PENDING.with(|p| p.borrow_mut().clear());
        None
    }
}

fn flush_logs(logs: &Logs, sent: &mut usize) {
    let lines = logs.borrow();
    for line in &lines[*sent..] {
        emit(&json!({ "log": line }));
    }
    *sent = lines.len();
}

/// Run one execution reading stdin and writing stdout; the process exits
/// after the `done` line.
#[cfg(not(target_arch = "wasm32"))]
pub fn main() {
    let stdin = std::io::stdin();
    let mut lines = stdin.lock().lines();
    let Some(Ok(first)) = lines.next() else { return };
    let sink = Box::new(|line: &Value| {
        let mut out = std::io::stdout().lock();
        let _ = writeln!(out, "{line}");
        let _ = out.flush();
    });
    let Some(mut realm) = Realm::begin(&first, sink) else { return };
    loop {
        let Some(Ok(line)) = lines.next() else {
            realm.abandon();
            return;
        };
        if realm.answer(&line) {
            return;
        }
    }
}
