//! brep-render-desktop — the native windowed presentation shell (the dual-
//! target requirement: desktop and web build from the SAME renderer core AND
//! the SAME windowing-agnostic viewer brain).
//!
//! Usage: brep-render-desktop <history-request.json>
//!
//! Opens a winit window and drives a [`brep_render::engine_state::EngineState`]
//! — the exact same scene + camera + controls + widgets + pick/overlay state the
//! wasm `Engine` (engine.rs) wraps. The desktop shell adds only the native
//! GPU/window bits (winit `Window`, wgpu surface + config, [`RenderCore`],
//! [`GpuScene`]); every interaction (orbit/pan/zoom/fit/projection, the
//! ViewCube, picking, overlays) flows through `EngineState`, so both targets run
//! the same code. On-demand rendering (R22): a frame is drawn only when
//! `EngineState::dirty` is set (the desktop analogue of `OrthoCameraIdle`).
//!
//! Controls: left-drag orbit, right/middle-drag pan, wheel zoom (toward cursor),
//! F zoom-to-fit, P toggle ortho/perspective, 1-6 standard views, ViewCube in
//! the corner (hover to highlight, click a face/arrow to snap/orbit).

use brep_render::controls::{BUTTON_LEFT, BUTTON_MIDDLE, BUTTON_RIGHT};
use brep_render::engine_state::EngineState;
use brep_render::render::{FrameParams, GpuScene, RenderCore, SAMPLES};
use std::sync::Arc;
use winit::application::ApplicationHandler;
use winit::event::{ElementState, MouseButton, MouseScrollDelta, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{Key, NamedKey};
use winit::window::{Window, WindowId};

struct App {
    /// The windowing-agnostic viewer brain (scene + camera + controls + settings
    /// + emphasis + widgets + pick/overlay). Shared verbatim with the wasm shell.
    state: EngineState,
    /// Last-known cursor position in physical framebuffer pixels — the space the
    /// camera viewport (and hence `EngineState`'s pointer/pick coords) live in on
    /// the desktop, since winit reports physical pixels for both.
    cursor: (f64, f64),
    /// Device pixel ratio (CSS px → physical px), for line/point widths.
    dpr: f32,
    /// True while a camera drag (orbit/pan) started outside the ViewCube is live,
    /// so moves keep routing to the controls even over the cube corner.
    dragging: bool,
    gpu: Option<Gpu>,
}

struct Gpu {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    core: RenderCore,
    gpu_scene: GpuScene,
}

impl App {
    fn init_gpu(&mut self, event_loop: &ActiveEventLoop) {
        let window = Arc::new(
            event_loop
                .create_window(Window::default_attributes().with_title("brep-render"))
                .expect("window"),
        );
        self.dpr = window.scale_factor() as f32;
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::new_without_display_handle_from_env());
        let surface = instance.create_surface(window.clone()).expect("surface");
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        }))
        .expect("adapter");
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("brep-render-desktop"),
            ..Default::default()
        }))
        .expect("device");

        let size = window.inner_size();
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
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::AutoVsync,
            desired_maximum_frame_latency: 2,
            alpha_mode: caps.alpha_modes[0],
            view_formats: vec![],
        };
        surface.configure(&device, &config);

        let core = RenderCore::new(device, queue, format);
        let gpu_scene = GpuScene::default();

        // The camera viewport tracks the physical framebuffer (winit reports
        // physical pixels for both the surface and the cursor, so pointer/pick
        // coords line up); `dpr` scales line/point widths in the frame params.
        self.state.resize(config.width as f64, config.height as f64);
        self.state.zoom_to_fit();

        self.gpu = Some(Gpu {
            window,
            surface,
            config,
            core,
            gpu_scene,
        });
        self.state.dirty = true;
    }

    /// Render one frame, replicating the wasm `Engine::render()` flow: sync the
    /// scene, resolve the camera, build the widget overlay from the state, and
    /// hand it all to the shared [`RenderCore`].
    fn redraw(&mut self) {
        let dpr = self.dpr;
        let Some(gpu) = &mut self.gpu else { return };
        let state = &mut self.state;

        gpu.core.sync_scene(
            &mut gpu.gpu_scene,
            &state.scene,
            &state.settings,
            state.settings_generation,
        );

        let frame = match gpu.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(frame)
            | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
            wgpu::CurrentSurfaceTexture::Timeout | wgpu::CurrentSurfaceTexture::Occluded => return,
            // Outdated/Lost/Validation: reconfigure and retry once.
            _ => {
                gpu.surface.configure(&gpu.core.device, &gpu.config);
                match gpu.surface.get_current_texture() {
                    wgpu::CurrentSurfaceTexture::Success(frame)
                    | wgpu::CurrentSurfaceTexture::Suboptimal(frame) => frame,
                    other => {
                        eprintln!("surface unavailable: {other:?}");
                        return;
                    }
                }
            }
        };
        let view = frame.texture.create_view(&Default::default());
        // Depth-fit to everything drawn + resolve the camera in one shared step
        // (mirrors the wasm `Engine::render` path) so construction geometry
        // (datums/axes/gizmos) beyond the solids never clips.
        let (camera, overlay) = state.fit_camera_and_overlay();
        let params = FrameParams {
            camera: &camera,
            width: gpu.config.width,
            height: gpu.config.height,
            dpr,
            settings: &state.settings,
            emphasis: &state.emphasis,
            world_per_pixel: state.camera.world_per_pixel(),
            overlay: overlay.as_ref(),
        };
        gpu.core
            .render_to_view(&mut gpu.gpu_scene, &state.scene, &params, &view);
        frame.present();
        state.dirty = false;
    }

    /// Queue a redraw when the engine reports itself dirty (R22 on-demand).
    fn request_redraw_if_dirty(&self) {
        if self.state.dirty {
            if let Some(gpu) = &self.gpu {
                gpu.window.request_redraw();
            }
        }
    }

    /// The ViewCube corner sub-rect `(x, y, w, h)` in framebuffer pixels, or
    /// `None` when the cube is disabled / not laid out.
    fn viewcube_rect(&self) -> Option<(f64, f64, f64, f64)> {
        let value: serde_json::Value = serde_json::from_str(&self.state.viewcube_rect_json()).ok()?;
        Some((
            value["x"].as_f64()?,
            value["y"].as_f64()?,
            value["w"].as_f64()?,
            value["h"].as_f64()?,
        ))
    }

    /// If `(x, y)` falls inside the ViewCube corner, its cube-local pixel coords.
    fn viewcube_local(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        let (rx, ry, rw, rh) = self.viewcube_rect()?;
        if rw > 0.0 && rh > 0.0 && x >= rx && x <= rx + rw && y >= ry && y <= ry + rh {
            Some((x - rx, y - ry))
        } else {
            None
        }
    }
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.gpu.is_none() {
            self.init_gpu(event_loop);
            self.request_redraw_if_dirty();
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        match event {
            WindowEvent::CloseRequested => {
                event_loop.exit();
                return;
            }
            WindowEvent::Resized(size) => {
                if let Some(gpu) = &mut self.gpu {
                    gpu.config.width = size.width.max(1);
                    gpu.config.height = size.height.max(1);
                    gpu.surface.configure(&gpu.core.device, &gpu.config);
                    self.state.resize(gpu.config.width as f64, gpu.config.height as f64);
                }
            }
            WindowEvent::ScaleFactorChanged { scale_factor, .. } => {
                self.dpr = scale_factor as f32;
                self.state.dirty = true;
            }
            WindowEvent::MouseInput { state, button, .. } => {
                let btn = match button {
                    MouseButton::Left => BUTTON_LEFT,
                    MouseButton::Middle => BUTTON_MIDDLE,
                    MouseButton::Right => BUTTON_RIGHT,
                    _ => -1,
                };
                if state == ElementState::Pressed {
                    // A press over the ViewCube corner snaps/orbits via the cube;
                    // anywhere else starts a camera drag through the controls.
                    if let Some((lx, ly)) = self.viewcube_local(self.cursor.0, self.cursor.1) {
                        self.state.viewcube_click(lx, ly);
                    } else {
                        self.state.pointer_down(self.cursor.0, self.cursor.1, btn);
                        self.dragging = true;
                    }
                } else {
                    if self.dragging {
                        self.state.pointer_up();
                        self.dragging = false;
                    }
                }
            }
            WindowEvent::CursorMoved { position, .. } => {
                self.cursor = (position.x, position.y);
                if self.dragging {
                    // An active camera drag keeps orbiting/panning even over the
                    // cube corner.
                    self.state.pointer_move(position.x, position.y);
                } else if let Some((lx, ly)) = self.viewcube_local(position.x, position.y) {
                    self.state.viewcube_hover(lx, ly);
                } else {
                    // Left the cube corner (or never over it): drop any highlight.
                    self.state.viewcube_clear_hover();
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let dy = match delta {
                    MouseScrollDelta::LineDelta(_, y) => -(y as f64) * 100.0,
                    MouseScrollDelta::PixelDelta(pos) => -pos.y,
                };
                // Zoom toward the cursor (R22).
                self.state.wheel(dy, Some([self.cursor.0, self.cursor.1]));
            }
            WindowEvent::KeyboardInput { event, .. } if event.state == ElementState::Pressed => {
                match event.logical_key.as_ref() {
                    Key::Character("f") | Key::Character("F") => self.state.zoom_to_fit(),
                    Key::Character("p") | Key::Character("P") => {
                        self.state.toggle_projection();
                    }
                    Key::Character("1") => {
                        self.state.standard_view("FRONT");
                    }
                    Key::Character("2") => {
                        self.state.standard_view("BACK");
                    }
                    Key::Character("3") => {
                        self.state.standard_view("RIGHT");
                    }
                    Key::Character("4") => {
                        self.state.standard_view("LEFT");
                    }
                    Key::Character("5") => {
                        self.state.standard_view("TOP");
                    }
                    Key::Character("6") => {
                        self.state.standard_view("BOTTOM");
                    }
                    Key::Named(NamedKey::Escape) => {
                        event_loop.exit();
                        return;
                    }
                    _ => {}
                }
            }
            WindowEvent::RedrawRequested => {
                if self.state.dirty {
                    self.redraw();
                }
                return;
            }
            _ => return,
        }
        self.request_redraw_if_dirty();
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("usage: brep-render-desktop <history-request.json>");
        std::process::exit(2);
    }
    let request_json = std::fs::read_to_string(&args[1]).expect("read history request");

    // Feed the demo model/history through the SHARED viewer brain (the same
    // `run_history_json` the wasm `Engine` calls) so the desktop and web builds
    // exercise identical scene-build + widget code.
    let mut state = EngineState::new();
    let report_json = state
        .run_history_json(&request_json)
        .expect("execute history");
    if let Ok(report) = serde_json::from_str::<serde_json::Value>(&report_json) {
        if let Some(errors) = report["featureErrors"].as_array() {
            for error in errors {
                if let Some(message) = error.as_str() {
                    eprintln!("feature error: {message}");
                }
            }
        }
    }
    state.set_viewcube_enabled(true);
    state.zoom_to_fit();
    eprintln!(
        "scene: {} solid(s), MSAA {SAMPLES}x — drag orbit, right-drag pan, wheel zoom, F fit, P projection, ViewCube in corner",
        state.scene.solids().len()
    );

    let event_loop = EventLoop::new().expect("event loop");
    let mut app = App {
        state,
        cursor: (0.0, 0.0),
        dpr: 1.0,
        dragging: false,
        gpu: None,
    };
    event_loop.run_app(&mut app).expect("run");
}
