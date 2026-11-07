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
pub mod handles;
pub mod fonts;
pub mod form;
pub mod form_view;
pub mod icon_text;
pub mod icons;
pub mod instructions;
mod http;
/// The native binary's `log` sink (`main.rs`): `RUST_LOG` to stderr. Replaces
/// `env_logger`; the wasm build logs through the browser console instead.
#[cfg(not(target_arch = "wasm32"))]
pub mod logger;
/// The embedded MCP server (`brep-app --mcp`); native builds with `mcp`.
#[cfg(all(not(target_arch = "wasm32"), feature = "mcp"))]
pub mod mcp;
/// What the Delete key / Delete command removes for the current selection (every target).
pub mod delete_selection;
/// The desktop event loop: eframe's winit application plus application-command
/// dispatch that does not wait for a frame (a hidden Wayland window gets none).
#[cfg(not(target_arch = "wasm32"))]
pub mod desktop_loop;
/// One egui pass drawn into an offscreen texture: the captures of a pass
/// nothing presents (the desktop loop's, the browser pump's).
pub mod offscreen;
/// One egui pass the app runs itself when nothing is drawing it.
#[cfg(feature = "automation")]
pub mod on_demand;
/// The browser's frame-free automation pump: commands answered while the tab
/// is in the background.
#[cfg(all(target_arch = "wasm32", feature = "automation"))]
pub mod web_pump;
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
/// The problem reports this user submitted, and the notice when one is resolved.
pub mod report_followup;
/// Scripts in the CAD app: the CAD host for the PLM's interpreter core (S11).
#[cfg(feature = "scripting")]
pub mod scripting;
#[cfg(not(target_arch = "wasm32"))]
pub mod simulation_export;
pub mod store;
/// A template's input marks and its spin-out (`.tbrep`).
pub mod template;
/// The desktop app's signed updates from next.brep.io (offered, never silent).
#[cfg(not(target_arch = "wasm32"))]
pub mod update;
pub mod viewport;
/// The browser's tool surface (WebMCP): the MCP server's tool set, served by
/// the page as the tools it registers with `document.modelContext`, answered
/// through the automation queue. The pure half compiles natively for its
/// tests; the host and the exports `web/webmcp.js` imports exist on wasm only.
#[cfg(feature = "mcp")]
pub mod webmcp;
pub mod workbench;
pub mod plugins;
pub mod javascript;


// The wasm history runner: a dedicated web worker so a history run stays OFF the
// browser main thread (single-threaded wasm) and the UI never freezes during a run.
// Its `worker_entry` is the worker-side onmessage loop. wasm only.
#[cfg(target_arch = "wasm32")]
pub mod worker;

// The browser's code worker: one run_script execution on Boa in a Web Worker
// the page can terminate (`web/code-worker.js` → `code_worker_entry`).
#[cfg(all(target_arch = "wasm32", feature = "scripting"))]
pub mod code_worker;

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

        let runner = eframe::WebRunner::new();
        // The automation pump borrows the app from this runner between
        // frames (`web_pump`): commands answer in a background tab too.
        #[cfg(feature = "automation")]
        crate::web_pump::set_runner(&runner);
        runner
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| {
                    crate::fonts::install(&cc.egui_ctx);
                    crate::app::BrepApp::new(cc)
                        .map(|mut app| {
                            app.start_report_followup(&cc.egui_ctx);
                            #[cfg(feature = "automation")]
                            crate::web_pump::install(&cc.egui_ctx, cc.wgpu_render_state.clone(), app.automation());
                            // The page's tool surface (WebMCP) answers through
                            // the app's own automation queue: hand it over
                            // here, where the app first exists.
                            #[cfg(feature = "mcp")]
                            crate::webmcp::attach(app.automation().clone(), app.diagnostics().adapter_line());
                            Box::new(app) as Box<dyn eframe::App>
                        })
                        .map_err(|e| e.into())
                }),
            )
            .await
    }
}
