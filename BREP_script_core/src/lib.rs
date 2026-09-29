//! The interpreter core: evaluate one script, call one global function with
//! one JSON argument, and hand back its JSON result or its refusal.
//!
//! This crate depends on Boa and `serde_json` and nothing else — no file system,
//! no network, no store. That is deliberate: the operator chose Boa so the same
//! interpreter runs inside the CAD app, including its wasm build
//! (plm-cad-integration-todo §3 S11). It was the PLM's `scripting::engine` module
//! and is still reached under that name there. Everything that reaches outside the
//! interpreter is installed by the caller through [`call`]'s `install` argument:
//! the server's bindings are in `BREP_plm/src/scripting/host.rs`, the CAD app's in
//! `BREP_app/src/scripting.rs`.
//!
//! # A fresh interpreter per call
//!
//! Every call builds a new [`Context`], evaluates the file, and drops it. No
//! state leaks from one hook call into the next, a script edited on disk is
//! simply the script the next call evaluates, and a `Context` (which is not
//! `Send`) never has to cross a thread.
//!
//! # What is bounded, honestly
//!
//! Boa's [`RuntimeLimits`](boa_engine::vm::RuntimeLimits) bound three things,
//! and they are set here:
//!
//! * each LOOP may run at most [`LOOP_ITERATION_LIMIT`] iterations — an
//!   infinite `while (true)` throws instead of hanging;
//! * call depth is at most [`RECURSION_LIMIT`] — runaway recursion throws;
//! * the VM stack size.
//!
//! There is NO wall-clock limit: Boa cannot pre-empt a running script. A script
//! of many individually-bounded loops can still run long, and a script blocked
//! inside a host function (an HTTP call, a child process) is bounded only by
//! that host function's own timeout.

use std::cell::RefCell;
use std::rc::Rc;

use boa_engine::object::ObjectInitializer;
use boa_engine::property::Attribute;
use boa_engine::{js_string, Context, JsError, JsResult, JsString, JsValue, NativeFunction, Source};
use serde_json::Value;

/// Boa itself, so a host names the same engine's types (`Context`, `JsValue`,
/// `ObjectInitializer`) its bindings are built from without a Boa line of its
/// own that could drift to another version.
pub use boa_engine;

/// The most iterations any one loop may run before it throws.
pub const LOOP_ITERATION_LIMIT: u64 = 10_000_000;

/// The deepest call stack a script may build before it throws.
pub const RECURSION_LIMIT: usize = 512;

/// Lines a script wrote through `console.log`, kept for the caller — the test
/// run shows them, and a failing hook's log goes to the server log.
pub type Logs = Rc<RefCell<Vec<String>>>;

/// A call that returned.
#[derive(Debug, Clone)]
pub struct Success {
    /// What the function returned, as JSON. `undefined` reads as `null`.
    pub value: Value,
    pub logs: Vec<String>,
}

/// A call that threw, failed to parse, or named a function the file does not
/// define. `message` is what a person reads: for `throw new Error("x")` it is
/// `x`, for `throw "x"` it is `x`.
#[derive(Debug, Clone)]
pub struct Failure {
    pub message: String,
    pub logs: Vec<String>,
}

/// Evaluate `source` (named `file` in messages), then call the global
/// function `function` with `input` as its single argument.
///
/// `install` runs after `console` is defined and before the file is
/// evaluated, and is where a caller adds its host objects.
pub fn call(
    source: &str,
    file: &str,
    function: &str,
    input: &Value,
    install: &dyn Fn(&mut Context, &Logs) -> JsResult<()>,
) -> Result<Success, Failure> {
    let mut context = Context::default();
    let limits = context.runtime_limits_mut();
    limits.set_loop_iteration_limit(LOOP_ITERATION_LIMIT);
    limits.set_recursion_limit(RECURSION_LIMIT);

    let logs: Logs = Rc::new(RefCell::new(Vec::new()));
    let fail = |message: String, logs: &Logs| Failure {
        message,
        logs: logs.borrow().clone(),
    };

    if let Err(error) = install_console(&mut context, &logs).and_then(|()| install(&mut context, &logs)) {
        return Err(fail(format!("setting up the script host failed: {}", describe(error, &mut context)), &logs));
    }

    let path = std::path::Path::new(file);
    if let Err(error) = context.eval(Source::from_bytes(source).with_path(path)) {
        let message = describe(error, &mut context);
        return Err(fail(format!("{file}: {message}"), &logs));
    }

    let global = context.global_object();
    let callee = match global.get(JsString::from(function), &mut context) {
        Ok(value) => value,
        Err(error) => return Err(fail(describe(error, &mut context), &logs)),
    };
    let Some(callee) = callee.as_callable() else {
        return Err(fail(format!("{file} defines no function {function}()"), &logs));
    };

    let argument = match JsValue::from_json(input, &mut context) {
        Ok(value) => value,
        Err(error) => return Err(fail(describe(error, &mut context), &logs)),
    };
    let returned = match callee.call(&JsValue::undefined(), &[argument], &mut context) {
        Ok(value) => value,
        Err(error) => return Err(fail(describe(error, &mut context), &logs)),
    };
    // A script may have queued promise jobs; run them so their side effects
    // (a log line, a write) are not silently dropped.
    let _ = context.run_jobs();

    let value = match returned.to_json(&mut context) {
        Ok(json) => json.unwrap_or(Value::Null),
        Err(error) => {
            return Err(fail(
                format!("{function}() returned a value that is not JSON: {}", describe(error, &mut context)),
                &logs,
            ))
        }
    };
    let logs = logs.borrow().clone();
    Ok(Success { value, logs })
}

/// The message a person should read for `error`.
pub fn describe(error: JsError, context: &mut Context) -> String {
    if let Some(value) = error.as_opaque() {
        if let Some(text) = value.as_string() {
            return text.to_std_string_escaped();
        }
    }
    match error.try_native(context) {
        Ok(native) if !native.message().is_empty() => native.message().to_string(),
        Ok(native) => native.to_string(),
        Err(_) => {
            // A thrown plain object: show it as JSON if it is JSON.
            if let Some(value) = error.as_opaque() {
                if let Ok(Some(json)) = value.to_json(context) {
                    return json.to_string();
                }
            }
            error.to_string()
        }
    }
}

/// One argument as the text a log line shows: strings verbatim, everything
/// else as JSON.
pub fn text_of(value: &JsValue, context: &mut Context) -> String {
    if let Some(text) = value.as_string() {
        return text.to_std_string_escaped();
    }
    match value.to_json(context) {
        Ok(Some(json)) => json.to_string(),
        Ok(None) => "undefined".to_string(),
        Err(_) => value.display().to_string(),
    }
}

/// `console.log / info / warn / error`, all writing into `logs`.
fn install_console(context: &mut Context, logs: &Logs) -> JsResult<()> {
    let make = |prefix: &'static str| {
        let logs = logs.clone();
        // SAFETY: the closure captures an `Rc<RefCell<Vec<String>>>`, which
        // holds no garbage-collected value, so the collector has nothing in it
        // to trace.
        unsafe {
            NativeFunction::from_closure(move |_, args, context| {
                let line = args
                    .iter()
                    .map(|value| text_of(value, context))
                    .collect::<Vec<_>>()
                    .join(" ");
                logs.borrow_mut().push(format!("{prefix}{line}"));
                Ok(JsValue::undefined())
            })
        }
    };
    let console = ObjectInitializer::new(context)
        .function(make(""), js_string!("log"), 0)
        .function(make(""), js_string!("info"), 0)
        .function(make("warn: "), js_string!("warn"), 0)
        .function(make("error: "), js_string!("error"), 0)
        .build();
    context.register_global_property(js_string!("console"), console, Attribute::all())
}

