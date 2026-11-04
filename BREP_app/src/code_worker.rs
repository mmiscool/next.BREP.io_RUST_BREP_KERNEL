//! The browser's code worker: one `run_script` execution in a Web Worker of
//! its own (`web/code-worker.js`, loading this same wasm bundle), on Boa, the
//! way the native app runs one in a process of its own. The page owns the
//! worker: it answers the program's host calls with the app's tools
//! (`webmcp.rs`) and TERMINATES the worker on a halt or a limit, which stops
//! the program wherever it is.
//!
//! The protocol is the native worker's (`brep_script_core::worker`), a
//! message per line: the first message starts the execution, each later one
//! answers a host call; the worker posts `{call}`, `{log}` and last `{done}`.
#![cfg(all(target_arch = "wasm32", feature = "scripting"))]

use brep_script_core::worker::Realm;
use std::cell::RefCell;
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

thread_local! {
    static REALM: RefCell<Option<Realm>> = const { RefCell::new(None) };
}

/// Install the worker's `onmessage`: the first message begins the
/// execution, the rest answer its host calls.
#[wasm_bindgen]
pub fn code_worker_entry() {
    let scope: web_sys::DedicatedWorkerGlobalScope = js_sys::global().unchecked_into();
    let post = scope.clone();
    let onmessage = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        let Some(text) = event.data().as_string() else { return };
        REALM.with(|slot| {
            let mut slot = slot.borrow_mut();
            match slot.as_mut() {
                None => {
                    let post = post.clone();
                    let sink = Box::new(move |line: &serde_json::Value| {
                        let _ = post.post_message(&JsValue::from_str(&line.to_string()));
                    });
                    *slot = Realm::begin(&text, sink);
                }
                Some(realm) => {
                    if realm.answer(&text) {
                        *slot = None;
                    }
                }
            }
        });
    }) as Box<dyn FnMut(web_sys::MessageEvent)>);
    scope.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    onmessage.forget();
}
