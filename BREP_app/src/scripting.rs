//! Scripts in the CAD app: the CAD-side host for the interpreter the PLM's
//! administrator hooks run on (`BREP_script_core`, plm-cad-integration-todo
//! §3 S11).
//!
//! The core evaluates one file and calls one global function with one JSON
//! argument; this module decides what a script running inside the CAD app can
//! reach. Today that is:
//!
//! | Global | What it is |
//! | --- | --- |
//! | `console.log / info / warn / error` | lines kept with the outcome (the core's own binding) |
//! | `cad.version` | this app's version |
//! | `cad.store` | `"file"` natively, `"browser"` in the wasm build |
//! | `cad.plm` | `false`: no PLM is connected |
//! | `ui.notify(text)` | a message for the user, kept with the outcome for the host to show |
//!
//! There are NO `plm.*` bindings yet. They arrive with the PLM transport (S1),
//! and every one of them will need a signed-in session; [`PLM_BINDINGS`] names
//! them so a script can be told which of its calls need a server. Until then a
//! script that reaches for `plm` finds it undefined, which is also exactly
//! what it finds with no server connected — the file-only case is the only
//! case, and it runs the same natively and in the browser.

use std::cell::RefCell;
use std::rc::Rc;

use brep_script_core::boa_engine::object::ObjectInitializer;
use brep_script_core::boa_engine::property::Attribute;
// Keys are spelled `JsString::from`, not `js_string!`: that macro expands to
// `::boa_engine`, which this crate reaches only through BREP_script_core.
use brep_script_core::boa_engine::{Context, JsResult, JsString, JsValue, NativeFunction};
pub use brep_script_core::{Failure, Success};
use serde_json::Value;

/// The `plm.*` bindings a CAD script can call, each needing a connected PLM.
/// Empty until S1 lands the transport.
pub const PLM_BINDINGS: &[&str] = &[];

/// Where the app keeps its documents, as `cad.store` reports it.
pub fn store_kind() -> &'static str {
    if cfg!(target_arch = "wasm32") {
        "browser"
    } else {
        "file"
    }
}

/// A finished call: the core's result plus what the script asked the UI to
/// show.
#[derive(Debug, Clone)]
pub struct Outcome<T> {
    pub result: T,
    pub notices: Vec<String>,
}

/// Evaluate `source` (named `file` in messages) with the CAD host installed,
/// then call its global `function` with `input`.
pub fn call(source: &str, file: &str, function: &str, input: &Value) -> Result<Outcome<Success>, Outcome<Failure>> {
    let notices: Rc<RefCell<Vec<String>>> = Rc::new(RefCell::new(Vec::new()));
    let installed = notices.clone();
    let result = brep_script_core::call(source, file, function, input, &move |context, _| {
        install(context, &installed)
    });
    let notices = notices.borrow().clone();
    match result {
        Ok(result) => Ok(Outcome { result, notices }),
        Err(result) => Err(Outcome { result, notices }),
    }
}

/// `cad` and `ui`.
fn install(context: &mut Context, notices: &Rc<RefCell<Vec<String>>>) -> JsResult<()> {
    let cad = ObjectInitializer::new(context)
        .property(JsString::from("version"), JsString::from(env!("CARGO_PKG_VERSION")), Attribute::READONLY)
        .property(JsString::from("store"), JsString::from(store_kind()), Attribute::READONLY)
        .property(JsString::from("plm"), false, Attribute::READONLY)
        .build();
    context.register_global_property(JsString::from("cad"), cad, Attribute::all())?;

    let notices = notices.clone();
    // SAFETY: the closure captures an `Rc<RefCell<Vec<String>>>`, which holds
    // no garbage-collected value, so the collector has nothing in it to trace.
    let notify = unsafe {
        NativeFunction::from_closure(move |_, args, context| {
            let line = args
                .iter()
                .map(|value| brep_script_core::text_of(value, context))
                .collect::<Vec<_>>()
                .join(" ");
            notices.borrow_mut().push(line);
            Ok(JsValue::undefined())
        })
    };
    let ui = ObjectInitializer::new(context).function(notify, JsString::from("notify"), 1).build();
    context.register_global_property(JsString::from("ui"), ui, Attribute::all())
}

