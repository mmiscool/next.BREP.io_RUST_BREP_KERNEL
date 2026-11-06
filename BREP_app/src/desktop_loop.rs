//! The desktop event loop.
//!
//! `eframe::run_native` runs the app only under winit's `RedrawRequested`.
//! On Wayland winit withholds that event while the surface's frame callback
//! is outstanding (winit 0.30 `wayland/event_loop/mod.rs`, which skips a window
//! whose callback is `Requested`, with no timeout), and a compositor sends no
//! callbacks to a window it is not showing (another workspace, a hidden
//! scratchpad, a covered window under some compositors). An automation command
//! queued then waited until the user focused the window: the 2026-10-31
//! desktop report's 29 s initial read. eframe's fallback for invisible windows
//! does not apply, because Wayland reports neither visibility nor minimisation
//! (`is_visible`/`is_minimized` return `None`). And once a repaint is
//! outstanding with no pass run, egui stops calling its repaint callback, so a
//! later submit did not even wake the loop.
//!
//! `run` owns the winit loop instead (`eframe::create_native`) and wraps
//! eframe's handler in `DesktopLoop`. The automation queue's waker sends the
//! loop its own event (never forwarded to eframe), and after every batch of
//! events the loop runs `BrepApp::service_automation` on this, the app's
//! thread, whenever the queue's head can be dispatched without a frame. Frames
//! still drain the queue exactly as before; this path draws nothing.
//!
//! A command that needs more than app state (input egui must apply, a layout
//! read, a capture) does not wait for a frame either: the loop runs an egui
//! pass itself, on demand (`run_pass`). It is the same pass eframe runs
//! (`raw_input_hook`, then `logic` and `ui` inside `Context::run_ui`), on the
//! window's last known size and clock, and it is never presented: a capture
//! asked for in it is drawn into an offscreen texture (`crate::offscreen`).
//! Everything else the pass hands back goes where eframe would have put it:
//! its textures to the shared renderer, its viewport and platform commands
//! back onto the context for eframe's next real frame, and a capture's
//! `Event::Screenshot` into the next pass's input, as eframe delivers one.
//!
//! The loop also keeps an undrawn window idle. eframe arms
//! `ControlFlow::Poll` whenever it requests a redraw (`check_redraw_requests`)
//! and disarms it only in the pass that redraw runs, so a redraw the
//! compositor never delivers leaves the loop polling, a core spinning while
//! idle. winit wakes the loop itself when a requested redraw is delivered, so
//! after one polled iteration without a `RedrawRequested`, the loop waits.
use crate::app::BrepApp;
use eframe::UserEvent;
use std::cell::RefCell;
use std::rc::Rc;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};
use winit::application::ApplicationHandler;
use winit::event_loop::{ActiveEventLoop, ControlFlow, EventLoop, EventLoopProxy};

/// Wakes the loop thread (the automation queue's waker).
pub type Waker = Arc<dyn Fn() + Send + Sync>;

/// The pass number the wake event carries. It is never a real pass, and the
/// event is never forwarded to eframe, so egui's repaint bookkeeping cannot
/// see or swallow it.
const WAKE: u64 = u64::MAX;

/// While work a frame-free dispatch started is still in flight (a background
/// history run, a store write) and no frame comes, it is serviced at the frame
/// cadence: the rate a drawn window pumps it. Background runs report
/// completion only by being polled (`Documents::pump_all`). This paces
/// completion of accepted work only; dispatch itself is event-driven.
const SERVICE_PACE: Duration = Duration::from_millis(16);

/// The app, shared on the loop thread between eframe (through [`Shell`]) and
/// the loop's service step. The two never run at once: winit delivers events
/// to one handler, sequentially, on this thread.
#[derive(Clone, Default)]
pub struct Slot(Rc<RefCell<Option<Installed>>>);

#[derive(Clone)]
struct Installed {
    app: Rc<RefCell<BrepApp>>,
    ctx: egui::Context,
    /// The window's renderer (shared with eframe's painter): an on-demand
    /// pass uploads its textures there and draws its captures with it.
    render: Option<eframe::egui_wgpu::RenderState>,
    pass: Rc<RefCell<PassInput>>,
}

/// What an on-demand pass needs to stand in for eframe's input: the clock
/// eframe's passes run on, the window's size and scale, and the events a
/// capture delivers to the pass after it.
struct PassInput {
    /// `(egui time, when)` of the last real pass, to continue its clock.
    clock: Option<(f64, Instant)>,
    /// When the app was installed: the clock's origin before any real pass.
    started: Instant,
    /// The window's inner size in physical pixels, from `Resized`.
    size: Option<winit::dpi::PhysicalSize<u32>>,
    /// The window's scale factor, from `ScaleFactorChanged`.
    scale: Option<f64>,
    /// Events for the next pass, real or on demand (`Event::Screenshot`).
    events: Vec<egui::Event>,
}

impl PassInput {
    fn new() -> Self {
        PassInput { clock: None, started: Instant::now(), size: None, scale: None, events: Vec::new() }
    }

    fn now(&self) -> f64 {
        match self.clock {
            Some((time, at)) => time + at.elapsed().as_secs_f64(),
            None => self.started.elapsed().as_secs_f64(),
        }
    }
}

impl Slot {
    /// Install the app from the eframe creation closure; returns eframe's
    /// handle to it.
    pub fn install(&self, app: BrepApp, ctx: &egui::Context, render: Option<eframe::egui_wgpu::RenderState>) -> Shell {
        let app = Rc::new(RefCell::new(app));
        let pass = Rc::new(RefCell::new(PassInput::new()));
        *self.0.borrow_mut() = Some(Installed { app: app.clone(), ctx: ctx.clone(), render, pass: pass.clone() });
        Shell(app, pass)
    }

    fn get(&self) -> Option<Installed> {
        self.0.borrow().clone()
    }
}

/// eframe's view of the shared app: every [`eframe::App`] method forwards.
pub struct Shell(Rc<RefCell<BrepApp>>, Rc<RefCell<PassInput>>);

impl eframe::App for Shell {
    fn logic(&mut self, ctx: &egui::Context, frame: &mut eframe::Frame) {
        eframe::App::logic(&mut *self.0.borrow_mut(), ctx, frame)
    }
    fn ui(&mut self, ui: &mut egui::Ui, frame: &mut eframe::Frame) {
        eframe::App::ui(&mut *self.0.borrow_mut(), ui, frame)
    }
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::App::save(&mut *self.0.borrow_mut(), storage)
    }
    fn on_exit(&mut self) {
        eframe::App::on_exit(&mut *self.0.borrow_mut())
    }
    fn auto_save_interval(&self) -> Duration {
        eframe::App::auto_save_interval(&*self.0.borrow())
    }
    fn clear_color(&self, visuals: &egui::Visuals) -> [f32; 4] {
        eframe::App::clear_color(&*self.0.borrow(), visuals)
    }
    fn persist_egui_memory(&self) -> bool {
        eframe::App::persist_egui_memory(&*self.0.borrow())
    }
    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        {
            // A real pass: an on-demand one continues its clock, and the
            // events an on-demand capture left for "the next pass" are this one's.
            let mut pass = self.1.borrow_mut();
            if let Some(time) = raw_input.time {
                pass.clock = Some((time, Instant::now()));
            }
            raw_input.events.append(&mut pass.events);
        }
        eframe::App::raw_input_hook(&mut *self.0.borrow_mut(), ctx, raw_input)
    }
}

/// How many passes the loop has run on demand (never presented).
static ON_DEMAND_PASSES: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

/// How many passes the loop has run on demand in this process.
pub fn on_demand_passes() -> u64 {
    ON_DEMAND_PASSES.load(std::sync::atomic::Ordering::SeqCst)
}

/// On-demand passes run at most at the frame cadence. The egui clock is wall
/// time in them as in a drawn frame, so what counts time in frames (a click's
/// timing, a dwell, `wait_frames`) sees what a drawn window would; and the
/// loop goes back to the window's own events between two of them.
const PASS_PACE: Duration = SERVICE_PACE;

/// The surface an on-demand pass lays out on before the window has reported
/// any size (in points; winit's default window size, which eframe keeps).
const FALLBACK_SIZE: egui::Vec2 = egui::vec2(800.0, 600.0);

/// Run one egui pass of the app now, as eframe's `run_ui_and_paint` would,
/// but without presenting it. See the module doc.
#[cfg(feature = "automation")]
fn run_pass(app: &mut BrepApp, installed: &Installed) {
    let ctx = &installed.ctx;
    let mut raw = {
        let mut pass = installed.pass.borrow_mut();
        let mut raw = ctx.input(|i| i.raw.clone());
        // The last pass's input stands for the window (its size, scale,
        // focus, texture limit); its events are spent.
        raw.events = std::mem::take(&mut pass.events);
        raw.hovered_files.clear();
        raw.dropped_files.clear();
        raw.time = Some(pass.now());
        let root = raw.viewports.entry(egui::ViewportId::ROOT).or_default();
        root.events.clear();
        if let Some(scale) = pass.scale {
            root.native_pixels_per_point = Some(scale as f32);
        }
        // egui-winit's rule: points = physical pixels / (native scale x zoom).
        let pixels_per_point = root.native_pixels_per_point.unwrap_or(1.0) * ctx.zoom_factor();
        if let Some(size) = pass.size.filter(|s| s.width > 0 && s.height > 0) {
            raw.screen_rect = Some(egui::Rect::from_min_size(
                egui::Pos2::ZERO,
                egui::vec2(size.width as f32, size.height as f32) / pixels_per_point,
            ));
        }
        if raw.screen_rect.is_none() {
            // No pass and no `Resized` yet: the size eframe opened the window
            // at, rather than egui's unbounded default.
            raw.screen_rect = Some(egui::Rect::from_min_size(egui::Pos2::ZERO, FALLBACK_SIZE));
        }
        if raw.max_texture_side.is_none() {
            raw.max_texture_side = installed.render.as_ref().map(|r| r.device.limits().max_texture_dimension_2d as usize);
        }
        raw
    };
    // Counted as it begins: the pass answers commands from inside it.
    ON_DEMAND_PASSES.fetch_add(1, std::sync::atomic::Ordering::SeqCst);
    eframe::App::raw_input_hook(app, ctx, &mut raw);
    let mut frame = eframe::Frame::_new_kittest();
    let output = ctx.run_ui(raw, |ui| {
        eframe::App::logic(app, ui.ctx(), &mut frame);
        eframe::App::ui(app, ui, &mut frame);
    });
    let egui::FullOutput { platform_output, textures_delta, shapes, pixels_per_point, mut viewport_output } = output;
    if let Some(render) = &installed.render {
        crate::offscreen::apply_texture_sets(render, &textures_delta);
    }
    // Viewport commands: a capture is drawn here; everything else goes back
    // onto the context for eframe's next real frame to carry out.
    let mut captures = Vec::new();
    for (id, viewport) in viewport_output.iter_mut() {
        for command in std::mem::take(&mut viewport.commands) {
            match command {
                egui::ViewportCommand::Screenshot(user_data) if *id == egui::ViewportId::ROOT => captures.push(user_data),
                other => ctx.send_viewport_cmd_to(*id, other),
            }
        }
    }
    if !captures.is_empty() {
        let size = ctx.content_rect().size() * pixels_per_point;
        let size_px = [size.x.round() as u32, size.y.round() as u32];
        let clear = eframe::App::clear_color(app, &ctx.global_style().visuals);
        let image = match &installed.render {
            Some(render) => crate::offscreen::capture(render, ctx, shapes, pixels_per_point, size_px, clear),
            None => Err("offscreen capture: the window has no wgpu render state".to_string()),
        };
        let mut pass = installed.pass.borrow_mut();
        match image {
            Ok(image) => {
                let image = std::sync::Arc::new(image);
                for user_data in captures {
                    pass.events.push(egui::Event::Screenshot { viewport_id: egui::ViewportId::ROOT, user_data, image: image.clone() });
                }
            }
            Err(error) => {
                log::error!("{error}");
                drop(pass);
                app.automation().fail_captures(&error);
            }
        }
    }
    if let Some(render) = &installed.render {
        crate::offscreen::apply_texture_frees(render, &textures_delta);
    }
    // What the user would have been given (a copy, an opened link) still
    // happens, on eframe's next real frame.
    if !platform_output.commands.is_empty() {
        ctx.output_mut(|o| o.commands.extend(platform_output.commands));
    }
    // A window that IS drawn shows the pass's effect on its next frame.
    ctx.request_repaint();
}

/// A waker that sends the loop its wake event. Safe from any thread; a closed
/// loop ignores it.
pub fn waker(proxy: EventLoopProxy<UserEvent>) -> Waker {
    let proxy = Mutex::new(proxy);
    Arc::new(move || {
        let event = UserEvent::RequestRepaint { viewport_id: egui::ViewportId::ROOT, when: Instant::now(), cumulative_pass_nr: WAKE };
        let _ = proxy.lock().unwrap_or_else(|p| p.into_inner()).send_event(event);
    })
}

fn is_wake(event: &UserEvent) -> bool {
    matches!(event, UserEvent::RequestRepaint { cumulative_pass_nr: WAKE, .. })
}

/// eframe's winit handler plus the frame-free service step.
pub struct DesktopLoop<H> {
    inner: H,
    slot: Slot,
    /// Work the last service step started is still in flight.
    busy: bool,
    /// The `WaitUntil` deadline the service step armed to pace that work.
    paced: Option<Instant>,
    /// The last iteration ended polling (eframe requested a redraw).
    polled: bool,
    /// A `RedrawRequested` arrived since the last `about_to_wait`.
    redrawn: bool,
    /// When the loop last ran a pass on demand ([`PASS_PACE`]).
    last_pass: Option<Instant>,
}

impl<H: ApplicationHandler<UserEvent>> DesktopLoop<H> {
    pub fn new(inner: H, slot: Slot) -> Self {
        DesktopLoop { inner, slot, busy: false, paced: None, polled: false, redrawn: false, last_pass: None }
    }

    fn with_pass(&self, f: impl FnOnce(&mut PassInput)) {
        if let Some(installed) = self.slot.get() {
            f(&mut installed.pass.borrow_mut());
        }
    }

    /// End eframe's poll for a redraw that did not come within one polled
    /// iteration (a hidden Wayland window gets none). Stock behaviour while
    /// redraws are delivered: each one disarms the poll in its own pass.
    fn unstick_poll(&mut self, event_loop: &ActiveEventLoop) {
        if event_loop.control_flow() == ControlFlow::Poll {
            if self.polled && !self.redrawn {
                event_loop.set_control_flow(ControlFlow::Wait);
                self.polled = false;
            } else {
                self.polled = true;
            }
        } else {
            self.polled = false;
        }
        self.redrawn = false;
    }

    /// One frame-free service step, when the queue's head needs no frame or
    /// work it started is still in flight.
    fn service(&mut self, event_loop: &ActiveEventLoop) {
        #[cfg(feature = "automation")]
        self.step();
        #[cfg(not(feature = "automation"))]
        let _ = &self.slot;
        // The deadline this step armed last time, if the loop still holds it
        // (eframe replaces it with its own when it schedules a repaint).
        let ours = self.paced.take().filter(|at| event_loop.control_flow() == ControlFlow::WaitUntil(*at));
        if self.busy {
            let next = Instant::now() + SERVICE_PACE;
            match event_loop.control_flow() {
                ControlFlow::Poll => {}
                // eframe's own earlier wake stands; ours is renewed.
                ControlFlow::WaitUntil(at) if at <= next && ours.is_none() => {}
                _ => {
                    event_loop.set_control_flow(ControlFlow::WaitUntil(next));
                    self.paced = Some(next);
                }
            }
        } else if ours.is_some() {
            // Work done: a deadline left in the past would wake the loop at
            // once, every iteration.
            event_loop.set_control_flow(ControlFlow::Wait);
        }
    }

    /// Dispatch what needs no pass, and run the pass the queue's head needs
    /// ([`PASS_PACE`] after the last one; until then the step stays busy, so
    /// the loop comes back for it).
    #[cfg(feature = "automation")]
    fn step(&mut self) {
        let Some(installed) = self.slot.get() else { return };
        // Only eframe holds this borrow, and never across a loop callback.
        let Ok(mut app) = installed.app.try_borrow_mut() else { return };
        let queue = app.automation().clone();
        let owed = loop {
            if self.busy || queue.has_frameless_work() {
                self.busy = app.service_automation(&installed.ctx);
            }
            let events = !installed.pass.borrow().events.is_empty();
            // A measured spin advances one step per pass.
            let owed = queue.needs_pass() || events || app.spinning();
            if !owed || self.last_pass.is_some_and(|at| at.elapsed() < PASS_PACE) {
                break owed;
            }
            run_pass(&mut app, &installed);
            self.last_pass = Some(Instant::now());
        };
        self.busy |= owed;
    }
}

impl<H: ApplicationHandler<UserEvent>> ApplicationHandler<UserEvent> for DesktopLoop<H> {
    fn new_events(&mut self, event_loop: &ActiveEventLoop, cause: winit::event::StartCause) {
        self.inner.new_events(event_loop, cause);
    }

    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.resumed(event_loop);
    }

    fn user_event(&mut self, event_loop: &ActiveEventLoop, event: UserEvent) {
        // The wake is ours alone; `about_to_wait`, which always follows the
        // batch it arrived in, services the queue.
        if !is_wake(&event) {
            self.inner.user_event(event_loop, event);
        }
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, window_id: winit::window::WindowId, event: winit::event::WindowEvent) {
        self.redrawn |= matches!(event, winit::event::WindowEvent::RedrawRequested);
        // The window's size and scale, for an on-demand pass (eframe keeps
        // its own copy where the loop cannot read it).
        let resized = match &event {
            winit::event::WindowEvent::Resized(size) => {
                self.with_pass(|pass| pass.size = Some(*size));
                true
            }
            winit::event::WindowEvent::ScaleFactorChanged { scale_factor, .. } => {
                let scale = *scale_factor;
                self.with_pass(|pass| pass.scale = Some(scale));
                true
            }
            _ => false,
        };
        self.inner.window_event(event_loop, window_id, event);
        #[cfg(feature = "automation")]
        if resized {
            if let Some(installed) = self.slot.get() {
                if let Ok(app) = installed.app.try_borrow() {
                    app.automation().mark_layout_dirty();
                }
            }
        }
        #[cfg(not(feature = "automation"))]
        let _ = resized;
    }

    fn device_event(&mut self, event_loop: &ActiveEventLoop, device_id: winit::event::DeviceId, event: winit::event::DeviceEvent) {
        self.inner.device_event(event_loop, device_id, event);
    }

    /// After every batch of events, including the batch a wake arrived in
    /// and the one in which eframe created the app: a command queued before
    /// the app existed, or before the waker was installed, is found here.
    fn about_to_wait(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.about_to_wait(event_loop);
        self.unstick_poll(event_loop);
        if !event_loop.exiting() {
            self.service(event_loop);
        }
    }

    fn suspended(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.suspended(event_loop);
    }

    fn exiting(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.exiting(event_loop);
    }

    fn memory_warning(&mut self, event_loop: &ActiveEventLoop) {
        self.inner.memory_warning(event_loop);
    }
}

/// What the creation closure is given: eframe's context and the waker to
/// install on the app's automation queue before anything can submit to it.
pub type Create<'a> = Box<dyn 'a + FnOnce(&eframe::CreationContext<'_>, Waker) -> Result<BrepApp, String>>;

/// Run the app on a new event loop.
pub fn run(app_name: &str, options: eframe::NativeOptions, create: Create<'_>) -> Result<(), String> {
    let event_loop = EventLoop::<UserEvent>::with_user_event().build().map_err(|e| format!("event loop: {e}"))?;
    run_on(event_loop, app_name, options, create, |eframe| eframe)
}

/// Run the app on `event_loop`. `wrap` sits between [`DesktopLoop`] and
/// eframe's handler: the identity in production; a test interposes there to
/// play a compositor that withholds frames.
pub fn run_on<'a, H: ApplicationHandler<UserEvent>>(
    event_loop: EventLoop<UserEvent>,
    app_name: &str,
    options: eframe::NativeOptions,
    create: Create<'a>,
    wrap: impl FnOnce(eframe::EframeWinitApplication<'a>) -> H,
) -> Result<(), String> {
    let slot = Slot::default();
    let failure: Rc<RefCell<Option<String>>> = Rc::default();
    let waker = waker(event_loop.create_proxy());
    let (installer, failed) = (slot.clone(), failure.clone());
    // A history run (or any runner reply) landing wakes the loop to collect
    // it, rather than waiting for the service step's next timed check.
    brep_render::runner::set_reply_waker(Some(waker.clone()));
    let creator: eframe::AppCreator<'a> = Box::new(move |cc| match create(cc, waker) {
        Ok(app) => Ok(Box::new(installer.install(app, &cc.egui_ctx, cc.wgpu_render_state.clone())) as Box<dyn eframe::App>),
        Err(error) => {
            *failed.borrow_mut() = Some(error.clone());
            Err(error.into())
        }
    });
    let eframe = eframe::create_native(app_name, options, creator, &event_loop);
    let mut handler = DesktopLoop::new(wrap(eframe), slot);
    event_loop.run_app(&mut handler).map_err(|e| format!("event loop: {e}"))?;
    let failed = failure.borrow_mut().take();
    match failed {
        Some(error) => Err(error),
        None => Ok(()),
    }
}
