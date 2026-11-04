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
pub struct Slot(Rc<RefCell<Option<(Rc<RefCell<BrepApp>>, egui::Context)>>>);

impl Slot {
    /// Install the app from the eframe creation closure; returns eframe's
    /// handle to it.
    pub fn install(&self, app: BrepApp, ctx: &egui::Context) -> Shell {
        let app = Rc::new(RefCell::new(app));
        *self.0.borrow_mut() = Some((app.clone(), ctx.clone()));
        Shell(app)
    }

    fn get(&self) -> Option<(Rc<RefCell<BrepApp>>, egui::Context)> {
        self.0.borrow().clone()
    }
}

/// eframe's view of the shared app: every [`eframe::App`] method forwards.
pub struct Shell(Rc<RefCell<BrepApp>>);

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
        eframe::App::raw_input_hook(&mut *self.0.borrow_mut(), ctx, raw_input)
    }
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
}

impl<H: ApplicationHandler<UserEvent>> DesktopLoop<H> {
    pub fn new(inner: H, slot: Slot) -> Self {
        DesktopLoop { inner, slot, busy: false, paced: None, polled: false, redrawn: false }
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

    #[cfg(feature = "automation")]
    fn step(&mut self) {
        let Some((app, ctx)) = self.slot.get() else { return };
        // Only eframe holds this borrow, and never across a loop callback.
        let Ok(mut app) = app.try_borrow_mut() else { return };
        if !self.busy && !app.automation().has_frameless_work() {
            return;
        }
        self.busy = app.service_automation(&ctx);
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
        self.inner.window_event(event_loop, window_id, event);
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
    let creator: eframe::AppCreator<'a> = Box::new(move |cc| match create(cc, waker) {
        Ok(app) => Ok(Box::new(installer.install(app, &cc.egui_ctx)) as Box<dyn eframe::App>),
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
