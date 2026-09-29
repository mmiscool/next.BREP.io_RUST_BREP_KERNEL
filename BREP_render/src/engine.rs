//! The wasm-bindgen browser engine (R3): the narrow API the host UI programs
//! against. Wraps the platform-agnostic [`EngineState`] with a WebGPU/WebGL
//! device + canvas surface (R5). The host never holds a renderer object — every call
//! passes names, ids, and JSON.
//!
//! WASM PACKAGING (documented architectural fork, per the slice brief): this is
//! a SECOND wasm-pack artifact (its own `pkg`), statically
//! linking its own copy of the kernel. The alternative — merging wgpu into the
//! kernel wasm — was rejected because wgpu's dependency tree would bloat and
//! slow the kernel wasm build, and the kernel Cargo.toml's panic=abort/fat-LTO
//! profile must stay untouched (the standing R1 directive). The tradeoff: the
//! render wasm runs `execute_history` in ITS OWN kernel instance (separate
//! linear memory + solid registry from the kernel wasm the primary pipeline uses),
//! so the browser feeds the engine the SAME `HistoryRequest` JSON and the
//! engine re-runs it to build display — the history executes once per wasm.
//! The R1 "no Float64Array copies across the wasm boundary" promise still holds: inside the render
//! wasm, tessellation flows kernel→GPU with no host boundary crossing, and the previous worker-
//! pool mesh assembly is deleted. A future slice can unify the two
//! instances once the kernel wasm is retired behind this engine.

use crate::engine_state::EngineState;
use crate::render::{FrameParams, GpuScene, RenderCore};
use std::cell::RefCell;
use std::rc::Rc;
use wasm_bindgen::prelude::*;

/// Thrown-error helper: any Rust `String` error → a `js_sys::Error`.
fn js_err(message: String) -> JsValue {
    js_sys::Error::new(&message).into()
}

/// A web display handle for the WebGL2 fallback instance. wgpu-core's canvas
/// surface path requires the instance to carry a `RawDisplayHandle`; on the web
/// it is a marker with no payload, so this ZST supplies it. (The WebGPU
/// dispatch goes through the browser's WebGPU API directly and needs no display
/// handle, hence it is only used for the GL fallback.)
#[derive(Debug)]
struct WebDisplay;

impl wgpu::rwh::HasDisplayHandle for WebDisplay {
    fn display_handle(
        &self,
    ) -> Result<wgpu::rwh::DisplayHandle<'_>, wgpu::rwh::HandleError> {
        let raw = wgpu::rwh::RawDisplayHandle::Web(wgpu::rwh::WebDisplayHandle::new());
        // SAFETY: the web display handle carries no borrowed data — it is a unit
        // marker — so it is valid for any lifetime.
        Ok(unsafe { wgpu::rwh::DisplayHandle::borrow_raw(raw) })
    }
}

/// GPU + surface bits, created on `attach`.
struct WasmRender {
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    core: RenderCore,
    gpu_scene: GpuScene,
    /// Device pixel ratio (CSS px → physical px), for line/point widths.
    dpr: f32,
}

struct Inner {
    state: EngineState,
    render: Option<WasmRender>,
}

/// The browser rendering engine handle (R3). One per canvas.
#[wasm_bindgen]
pub struct Engine {
    inner: Rc<RefCell<Inner>>,
}

#[wasm_bindgen]
impl Engine {
    /// Construct a detached engine. Call [`Engine::attach`] with a canvas
    /// before rendering; scene/camera/pick queries work immediately (headless).
    #[wasm_bindgen(constructor)]
    pub fn new() -> Engine {
        brep_kernel::panic_hook::set_once();
        Engine {
            inner: Rc::new(RefCell::new(Inner {
                state: EngineState::new(),
                render: None,
            })),
        }
    }

    /// Attach to a canvas: create the WebGPU (or WebGL2 fallback) device +
    /// surface. Async — resolves once the device is ready. `css_width/height`
    /// are the CSS size; `dpr` the device pixel ratio.
    pub fn attach(
        &self,
        canvas: web_sys::HtmlCanvasElement,
        css_width: f64,
        css_height: f64,
        dpr: f64,
    ) -> js_sys::Promise {
        let inner = self.inner.clone();
        wasm_bindgen_futures::future_to_promise(async move {
            let dpr = (dpr.max(0.5)) as f32;
            let phys_w = (css_width * dpr as f64).round().max(1.0) as u32;
            let phys_h = (css_height * dpr as f64).round().max(1.0) as u32;
            canvas.set_width(phys_w);
            canvas.set_height(phys_h);

            // WebGPU primary, WebGL2 fallback (R5).
            //
            // wgpu's frontend commits to ONE dispatch inside a single
            // `Instance` (see wgpu api/instance.rs): if `BROWSER_WEBGPU` is
            // requested AND `navigator.gpu` merely *exists*, it picks the pure
            // WebGPU context — which has no WebGL fallback of its own. So on a
            // machine where `navigator.gpu` exists but yields no adapter, a
            // single `BROWSER_WEBGPU | GL` instance reports "webgpu found no
            // adapters" and never tries the (compiled-in) GL backend. We must
            // drive the fallback ourselves with two separate instances.
            //
            // Probe WebGPU adapter availability *surfacelessly* first, so we
            // don't hand the canvas a `webgpu` context (which would taint it and
            // block a later `getContext('webgl2')`) unless WebGPU truly works.
            let webgpu_instance = {
                let mut descriptor = wgpu::InstanceDescriptor::new_without_display_handle();
                descriptor.backends = wgpu::Backends::BROWSER_WEBGPU;
                wgpu::Instance::new(descriptor)
            };
            let webgpu_ok = webgpu_instance
                .request_adapter(&wgpu::RequestAdapterOptions {
                    power_preference: wgpu::PowerPreference::HighPerformance,
                    compatible_surface: None,
                    force_fallback_adapter: false,
                })
                .await
                .is_ok();

            let (instance, surface, adapter) = if webgpu_ok {
                // WebGPU is available: commit the canvas to a WebGPU surface and
                // re-request with it as the compatible surface.
                let surface = webgpu_instance
                    .create_surface(wgpu::SurfaceTarget::Canvas(canvas))
                    .map_err(|error| js_err(format!("create_surface(webgpu): {error}")))?;
                let adapter = webgpu_instance
                    .request_adapter(&wgpu::RequestAdapterOptions {
                        power_preference: wgpu::PowerPreference::HighPerformance,
                        compatible_surface: Some(&surface),
                        force_fallback_adapter: false,
                    })
                    .await
                    .map_err(|error| js_err(format!("request_adapter(webgpu): {error}")))?;
                (webgpu_instance, surface, adapter)
            } else {
                // No WebGPU adapter — fall back to WebGL2. A GL-only instance
                // makes wgpu's frontend take the wgpu_core/gles dispatch (it does
                // NOT request BROWSER_WEBGPU, so the WebGPU branch is skipped).
                // The canvas is still pristine, so its WebGL2 context is free.
                //
                // Unlike the WebGPU dispatch (direct browser WebGPU, surfaceless), the
                // wgpu_core/GL path routes canvas surface creation through
                // wgpu-core, which demands a display handle on the instance
                // (`create_surface(Canvas)` passes `raw_display_handle: None`, so
                // an instance without one fails with `MissingDisplayHandle`). The
                // web display handle is a ZST marker, so we can supply it here.
                drop(webgpu_instance);
                let mut descriptor =
                    wgpu::InstanceDescriptor::new_with_display_handle(Box::new(WebDisplay));
                descriptor.backends = wgpu::Backends::GL;
                let instance = wgpu::Instance::new(descriptor);
                let surface = instance
                    .create_surface(wgpu::SurfaceTarget::Canvas(canvas))
                    .map_err(|error| js_err(format!("create_surface(webgl2): {error}")))?;
                let adapter = instance
                    .request_adapter(&wgpu::RequestAdapterOptions {
                        power_preference: wgpu::PowerPreference::HighPerformance,
                        compatible_surface: Some(&surface),
                        force_fallback_adapter: false,
                    })
                    .await
                    .map_err(|error| js_err(format!("request_adapter(webgl2): {error}")))?;
                (instance, surface, adapter)
            };
            let _ = &instance;
            // WebGL2 needs downlevel limits; ask for what the adapter supports.
            let (device, queue) = adapter
                .request_device(&wgpu::DeviceDescriptor {
                    label: Some("brep-render-web"),
                    required_limits: adapter.limits(),
                    ..Default::default()
                })
                .await
                .map_err(|error| js_err(format!("request_device: {error}")))?;

            let caps = surface.get_capabilities(&adapter);
            let format = caps
                .formats
                .iter()
                .copied()
                .find(|format| !format.is_srgb())
                .unwrap_or(caps.formats[0]);
            let config = wgpu::SurfaceConfiguration {
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                format,
                width: phys_w,
                height: phys_h,
                present_mode: wgpu::PresentMode::AutoVsync,
                desired_maximum_frame_latency: 2,
                alpha_mode: caps.alpha_modes[0],
                view_formats: vec![],
            };
            surface.configure(&device, &config);

            let core = RenderCore::new(device, queue, format);
            let gpu_scene = GpuScene::default();

            {
                let mut inner = inner.borrow_mut();
                inner.state.resize(css_width, css_height);
                inner.render = Some(WasmRender {
                    surface,
                    config,
                    core,
                    gpu_scene,
                    dpr,
                });
            }
            Ok(JsValue::UNDEFINED)
        })
    }

    /// Whether a GPU device is attached.
    pub fn is_attached(&self) -> bool {
        self.inner.borrow().render.is_some()
    }

    /// Resize the surface + camera viewport (CSS px + device pixel ratio).
    pub fn resize(&self, css_width: f64, css_height: f64, dpr: f64) {
        let mut inner = self.inner.borrow_mut();
        inner.state.resize(css_width, css_height);
        if let Some(render) = &mut inner.render {
            render.dpr = (dpr.max(0.5)) as f32;
            render.config.width = (css_width * render.dpr as f64).round().max(1.0) as u32;
            render.config.height = (css_height * render.dpr as f64).round().max(1.0) as u32;
            render.surface.configure(&render.core.device, &render.config);
        }
    }

    /// Render one frame IF the engine is dirty (R22 on-demand). Returns true if
    /// a frame was drawn. Call from a rAF loop or after any mutating command.
    pub fn render(&self) -> bool {
        let mut inner = self.inner.borrow_mut();
        let Inner { state, render } = &mut *inner;
        if !state.dirty {
            return false;
        }
        let Some(render) = render else {
            return false;
        };
        let settings_generation = state.settings_generation;
        render
            .core
            .sync_scene(&mut render.gpu_scene, &state.scene, &state.settings, settings_generation);

        let frame = match render.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => {
                return false
            }
            _ => {
                render.surface.configure(&render.core.device, &render.config);
                match render.surface.get_current_texture() {
                    wgpu::CurrentSurfaceTexture::Success(frame)
                    | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
                    _ => return false,
                }
            }
        };
        let view = frame.texture.create_view(&Default::default());
        // Fit the depth window to everything drawn (solids + pushed overlay +
        // the full widget overlay's world bounds + the origin) and resolve the
        // camera in one shared step — see `EngineState::fit_camera_and_overlay`;
        // keeps orbiting from clipping construction geometry (datums/axes/gizmos).
        let (camera, overlay) = state.fit_camera_and_overlay();
        let params = FrameParams {
            camera: &camera,
            width: render.config.width,
            height: render.config.height,
            dpr: render.dpr,
            settings: &state.settings,
            emphasis: &state.emphasis,
            world_per_pixel: state.camera.world_per_pixel(),
            overlay: overlay.as_ref(),
        };
        render
            .core
            .render_to_view(&mut render.gpu_scene, &state.scene, &params, &view);
        frame.present();
        state.dirty = false;
        true
    }

    /// True when a render is pending (the host can skip the rAF cost otherwise).
    pub fn needs_render(&self) -> bool {
        self.inner.borrow().state.dirty
    }

    // --- Scene feed (R10) -------------------------------------------------

    /// Run a whole history and update the display scene. Returns the
    /// build-report JSON. Model colours come from the metadata store, not from
    /// this call — see `EngineState::sync_colors_from_metadata`.
    pub fn run_history(&self, request_json: &str) -> Result<String, JsValue> {
        let mut inner = self.inner.borrow_mut();
        inner.state.run_history_json(request_json).map_err(js_err)
    }

    /// Frame the whole scene.
    pub fn zoom_to_fit(&self) {
        self.inner.borrow_mut().state.zoom_to_fit();
    }

    // --- Pointer / wheel ingestion (R22) ----------------------------------

    pub fn pointer_down(&self, x: f64, y: f64, button: i32) -> bool {
        self.inner.borrow_mut().state.pointer_down(x, y, button)
    }

    pub fn pointer_move(&self, x: f64, y: f64) -> bool {
        self.inner.borrow_mut().state.pointer_move(x, y)
    }

    pub fn pointer_up(&self) -> bool {
        self.inner.borrow_mut().state.pointer_up()
    }

    pub fn wheel(&self, delta_y: f64, cursor_x: f64, cursor_y: f64) -> bool {
        // Zoom toward the cursor when a finite position is given (NaN = center).
        let cursor = if cursor_x.is_finite() && cursor_y.is_finite() {
            Some([cursor_x, cursor_y])
        } else {
            None
        };
        self.inner.borrow_mut().state.wheel(delta_y, cursor)
    }

    pub fn set_controls_enabled(&self, enabled: bool) {
        self.inner.borrow_mut().state.set_controls_enabled(enabled);
    }

    // --- Camera commands (R21) --------------------------------------------

    pub fn toggle_projection(&self) -> String {
        self.inner.borrow_mut().state.toggle_projection().to_string()
    }

    pub fn set_projection(&self, kind: &str) {
        self.inner.borrow_mut().state.set_projection(kind);
    }

    pub fn standard_view(&self, name: &str) -> bool {
        self.inner.borrow_mut().state.standard_view(name)
    }

    pub fn camera_state(&self) -> String {
        self.inner.borrow().state.camera_state_json()
    }

    pub fn apply_camera_state(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.apply_camera_state_json(json).map_err(js_err)
    }

    pub fn world_per_pixel(&self) -> f64 {
        self.inner.borrow().state.world_per_pixel()
    }

    /// Project world points (`[[x,y,z], …]` JSON) to CSS-pixel screen coords
    /// (`[[sx, sy, depth, inFront], …]`) for host label anchoring (R25).
    pub fn world_to_screen(&self, points_json: &str) -> Result<String, JsValue> {
        self.inner.borrow().state.world_to_screen_json(points_json).map_err(js_err)
    }

    /// The camera matrices for the host overlays' per-frame world→screen /
    /// screen→world hot path: `{ viewProj:[16], viewProjInverse:[16],
    /// viewport:[w,h] }` (column-major, index = `col*4 + row`). Lets the
    /// dimension + sketch overlays read the engine's view-projection directly
    /// instead of the compat mirror camera.
    pub fn camera_matrices(&self) -> String {
        self.inner.borrow().state.camera_matrices_json()
    }

    // --- Picking (R23/R24) ------------------------------------------------

    /// Ranked candidates under CSS-pixel `(x, y)` (kernel names, priority
    /// VERTEX > EDGE > FACE > … > SOLID) as JSON — feeds the host popup.
    pub fn pick(&self, x: f64, y: f64) -> String {
        self.inner.borrow().state.pick_json(x, y)
    }

    /// The single best candidate under `(x, y)` (hover) as JSON, or `null`.
    pub fn hover(&self, x: f64, y: f64) -> String {
        self.inner.borrow().state.hover_json(x, y)
    }

    // --- Settings / emphasis / visibility (R11/R14/R17) -------------------

    pub fn apply_settings(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.apply_settings_json(json).map_err(js_err)
    }

    /// Set the selection/hover emphasis (R17): the name-keyed set from the host's
    /// SelectionFilter, as `{selected:{solids,faces,edges,vertices}, hovered:{…}}`.
    pub fn apply_emphasis(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.apply_emphasis_json(json).map_err(js_err)
    }

    pub fn set_visible(&self, name: &str, visible: bool) -> bool {
        self.inner.borrow_mut().state.set_visible(name, visible)
    }

    /// Scene-tree feed (R11): names/kind/visibility/child counts as JSON.
    pub fn scene_listing(&self) -> String {
        self.inner.borrow().state.scene_listing_json()
    }

    // --- Overlay widgets (R28-R31) ----------------------------------------

    /// Feed the datum / curve display set (planes/axes/frames/curves) as JSON.
    pub fn set_datums(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.set_datums_json(json).map_err(js_err)
    }

    /// Feed the feature-dimension set (linear/angular/radial) as JSON.
    pub fn set_dimensions(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.set_dimensions_json(json).map_err(js_err)
    }

    /// Feed the GENERAL overlay geometry channel: named groups of arbitrary
    /// tri/line/point geometry (feature-dialog previews and other display-only
    /// overlays). `{groups:[{name, renderOrder?, tris, lines, points?}]}`; a
    /// group upserts by name, an empty group removes it, empty groups clears all.
    pub fn set_overlay(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.set_overlay_json(json).map_err(js_err)
    }

    /// Set (or clear with `"null"`) the transform gizmo for the edited feature.
    pub fn set_transform_gizmo(&self, json: &str) -> Result<(), JsValue> {
        self.inner.borrow_mut().state.set_transform_json(json).map_err(js_err)
    }

    /// Enable/disable the always-on engine ViewCube.
    pub fn set_viewcube_enabled(&self, enabled: bool) {
        self.inner.borrow_mut().state.set_viewcube_enabled(enabled);
    }

    /// The ViewCube corner rect `{x,y,w,h}` (CSS px).
    pub fn viewcube_rect(&self) -> String {
        self.inner.borrow().state.viewcube_rect_json()
    }

    /// Update the ViewCube hover from cube-local pixels; returns whether the
    /// highlight changed.
    pub fn viewcube_hover(&self, local_x: f64, local_y: f64) -> bool {
        self.inner.borrow_mut().state.viewcube_hover(local_x, local_y)
    }

    /// Clear the ViewCube hover highlight; returns whether it changed.
    pub fn viewcube_clear_hover(&self) -> bool {
        self.inner.borrow_mut().state.viewcube_clear_hover()
    }

    /// Click the ViewCube at cube-local pixels: snap the shared camera. Returns
    /// true if a region was hit (the engine animates/sets the camera there).
    pub fn viewcube_click(&self, local_x: f64, local_y: f64) -> bool {
        self.inner.borrow_mut().state.viewcube_click(local_x, local_y)
    }

    /// Pick the datum plane/axis under a screen pixel; returns its kernel name
    /// (empty string when none).
    pub fn datum_pick(&self, x: f64, y: f64) -> String {
        self.inner.borrow().state.datum_pick(x, y)
    }

    /// Update the transform-gizmo hover; returns the handle under the pointer
    /// (0 = none).
    pub fn transform_hover(&self, x: f64, y: f64) -> u32 {
        self.inner.borrow_mut().state.transform_hover(x, y)
    }

    /// The transform-gizmo handle under a screen pixel (0 = none) — echo back to
    /// start a drag.
    pub fn transform_pick(&self, x: f64, y: f64) -> u32 {
        self.inner.borrow().state.transform_pick(x, y)
    }

    /// Compute a transform drag; returns the frame-space + world delta JSON for
    /// the host's feature-edit commit.
    pub fn transform_drag(&self, handle: u32, sx: f64, sy: f64, cx: f64, cy: f64) -> String {
        self.inner.borrow_mut().state.transform_drag(handle, sx, sy, cx, cy)
    }

    /// End a transform drag (clears the active-handle highlight).
    pub fn transform_drag_end(&self) {
        self.inner.borrow_mut().state.transform_drag_end();
    }

    /// Per-dimension label anchors + their screen projections, as JSON.
    pub fn dimension_anchors(&self) -> String {
        self.inner.borrow().state.dimension_anchors_json()
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}
