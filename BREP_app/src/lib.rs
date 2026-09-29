//! Shared eframe application for native windows and WebAssembly canvases.
//! [`app::BrepApp`] hosts the renderer and egui in one wgpu frame.

pub mod app;
pub mod automation;
mod color;
pub mod column_tree;
pub mod diagnostics;
pub mod document;
/// The three document classes (`.nbrep` / `.fbrep` / `.tbrep`).
pub mod document_class;
/// A family seed's table and its Generate (`.fbrep`).
pub mod family_table;
/// Register this build as the system's `.nbrep` opener
/// (`brep-app --install-file-associations`). Native only: the browser has no
/// file association to claim.
#[cfg(not(target_arch = "wasm32"))]
pub mod file_association;
pub mod fonts;
pub mod form;
pub mod form_view;
pub mod icon_text;
pub mod icons;
mod http;
/// The native binary's `log` sink (`main.rs`): `RUST_LOG` to stderr. Replaces
/// `env_logger`; the wasm build logs through the browser console instead.
#[cfg(not(target_arch = "wasm32"))]
pub mod logger;
/// The embedded MCP server (`brep-app --mcp`); native builds with `mcp`.
#[cfg(all(not(target_arch = "wasm32"), feature = "mcp"))]
pub mod mcp;
pub mod offsite;
pub mod palette;
pub mod panels;
/// Where a frame's milliseconds go — the per-frame timing record behind the
/// `__brepPerf` blob and the Info window's Performance rows.
pub mod perf;
/// The PLM client: config, transport, sign-in and the PLM store backend. Inert
/// with no server configured.
pub mod plm;
pub mod recovery;
/// Scripts in the CAD app: the CAD host for the PLM's interpreter core (S11).
#[cfg(feature = "scripting")]
pub mod scripting;
pub mod store;
/// A template's input marks and its spin-out (`.tbrep`).
pub mod template;
pub mod viewport;
pub mod workbench;


// The wasm history runner: a dedicated web worker so a history run stays OFF the
// browser main thread (single-threaded wasm) and the UI never freezes during a run.
// Its `worker_entry` is the worker-side onmessage loop. wasm only.
#[cfg(target_arch = "wasm32")]
pub mod worker;

// --- wasm entry ----------------------------------------------------------------
#[cfg(target_arch = "wasm32")]
mod web {
    use wasm_bindgen::prelude::*;
    use wasm_bindgen::JsCast;

    /// Start the eframe app on the given `<canvas>` element id. Called from JS.
    #[wasm_bindgen]
    pub async fn start(canvas_id: String) -> Result<(), JsValue> {
        brep_render::brep_kernel::panic_hook::set_once();

        // Bring up browser persistence FIRST. `ModelStore` is synchronous but every
        // browser store large enough for a native BREP payload is async, so the
        // whole key space is pulled into an in-memory mirror here — inside the one
        // async seam the app has — BEFORE `BrepApp::new` performs its first read.
        // See store.rs `mirror_store`.
        crate::store::hydrate_web_store().await;

        let document = web_sys::window()
            .ok_or_else(|| JsValue::from_str("no window"))?
            .document()
            .ok_or_else(|| JsValue::from_str("no document"))?;
        let canvas = document
            .get_element_by_id(&canvas_id)
            .ok_or_else(|| JsValue::from_str("canvas element not found"))?
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .map_err(|_| JsValue::from_str("element is not a <canvas>"))?;

        eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| {
                    crate::fonts::install(&cc.egui_ctx);
                    crate::app::BrepApp::new(cc)
                        .map(|app| Box::new(app) as Box<dyn eframe::App>)
                        .map_err(|e| e.into())
                }),
            )
            .await
    }
}
