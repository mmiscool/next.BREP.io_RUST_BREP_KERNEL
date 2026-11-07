//! The browser's frame-free automation pump: WebMCP commands answered while
//! the tab is in the background.
//!
//! eframe runs the app from `requestAnimationFrame`, and a hidden tab gets no
//! animation frames (and its timers are throttled to one a second, then one a
//! minute). The page's WebMCP host used to wait on those frames, so an agent
//! driving a background tab stalled. The desktop loop solved the same thing
//! for a hidden Wayland window (`desktop_loop`); this is the page's half:
//!
//! - a submit to the automation queue, and a reply from the history worker,
//!   schedule a pump on a `MessageChannel` (a task the browser does not
//!   throttle);
//! - the pump borrows the app from eframe's runner (`WebRunner::app_mut`) and
//!   runs the frame-free service step (`BrepApp::service_automation`): app
//!   state commands, polls, the work they started;
//! - a command that needs a pass gets one from `on_demand` when no frame is
//!   coming (the tab is hidden, or frames have stopped for
//!   [`FRAME_STALE_MS`]), paced at [`PASS_PACE_MS`]; a capture is drawn
//!   offscreen and read back asynchronously into the next pass's input;
//! - work still in flight re-arms the pump through a timer that runs in a
//!   tiny worker ([`after`]), which a background tab does not throttle; the
//!   WebMCP host's waits use it too.
//!
//! A visible tab keeps eframe's frames: the pump only adds the service step,
//! which answers state reads without waiting for the next frame.
use crate::app::BrepApp;
use eframe::egui_wgpu;
use std::cell::{Cell, RefCell};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;

/// The on-demand pass's pace: a 60 Hz frame.
const PASS_PACE_MS: f64 = 16.0;

/// A visible tab whose frames have stopped this long gets passes too (a
/// window minimised without `visibilitychange`, a throttled iframe).
const FRAME_STALE_MS: f64 = 100.0;

struct Pump {
    runner: eframe::WebRunner,
    ctx: egui::Context,
    render: Option<egui_wgpu::RenderState>,
    /// Posting on `port` runs [`pump`] as a fresh, unthrottled task.
    port: web_sys::MessagePort,
    _channel: web_sys::MessageChannel,
    _on_message: Closure<dyn FnMut(web_sys::MessageEvent)>,
}

thread_local! {
    static PUMP: RefCell<Option<Pump>> = const { RefCell::new(None) };
    static RUNNER: RefCell<Option<eframe::WebRunner>> = const { RefCell::new(None) };
    /// A pump is posted and has not run yet.
    static POSTED: Cell<bool> = const { Cell::new(false) };
    /// A timed pump is armed.
    static TIMED: Cell<bool> = const { Cell::new(false) };
    /// The unthrottled timer ([`after`]): its worker, and the callbacks it
    /// owes by id.
    static TIMER: RefCell<Option<Timer>> = const { RefCell::new(None) };
    /// When the pump last ran a pass (`performance.now()` ms).
    static LAST_PASS: Cell<f64> = const { Cell::new(f64::NEG_INFINITY) };
    /// The frame number last seen, and when it was first seen.
    static SEEN_FRAME: Cell<(u64, f64)> = const { Cell::new((u64::MAX, 0.0)) };
    /// Events for the next pass's input: the captures that landed.
    static EVENTS: RefCell<Vec<egui::Event>> = const { RefCell::new(Vec::new()) };
    /// Captures drawn and not read back yet.
    static CAPTURES_OUT: Cell<usize> = const { Cell::new(0) };
    static PASSES: Cell<u64> = const { Cell::new(0) };
}

fn now_ms() -> f64 {
    web_sys::window().and_then(|w| w.performance()).map_or_else(js_sys::Date::now, |p| p.now())
}

fn tab_hidden() -> bool {
    web_sys::window().and_then(|w| w.document()).is_some_and(|d| d.hidden())
}

/// Passes the pump has run (the `__brepWebPumpPasses` hook reads it).
pub fn passes() -> u64 {
    PASSES.with(Cell::get)
}

/// Remember eframe's runner, before `start`: the creation closure runs inside
/// it, and [`install`] needs the runner the app will live in.
pub fn set_runner(runner: &eframe::WebRunner) {
    RUNNER.with(|r| *r.borrow_mut() = Some(runner.clone()));
}

/// Start the pump for the app being created on `ctx` (from eframe's creation
/// closure, with the app's queue). Wakes on every submit and every history
/// worker reply from now on.
pub fn install(ctx: &egui::Context, render: Option<egui_wgpu::RenderState>, queue: &std::sync::Arc<crate::automation::queue::AutomationQueue>) {
    let Some(runner) = RUNNER.with(|r| r.borrow().clone()) else {
        log::error!("web_pump: no runner was set before the app started");
        return;
    };
    let Ok(channel) = web_sys::MessageChannel::new() else {
        log::error!("web_pump: no MessageChannel");
        return;
    };
    let on_message = Closure::wrap(Box::new(move |_: web_sys::MessageEvent| {
        POSTED.with(|p| p.set(false));
        pump();
    }) as Box<dyn FnMut(web_sys::MessageEvent)>);
    channel.port1().set_onmessage(Some(on_message.as_ref().unchecked_ref()));
    PUMP.with(|p| {
        *p.borrow_mut() = Some(Pump {
            runner,
            ctx: ctx.clone(),
            render,
            port: channel.port2(),
            _channel: channel,
            _on_message: on_message,
        })
    });
    queue.set_waker(std::sync::Arc::new(wake));
    crate::worker::set_reply_wake(Some(std::rc::Rc::new(wake)));
}

struct Timer {
    worker: web_sys::Worker,
    _on_message: Closure<dyn FnMut(web_sys::MessageEvent)>,
    next: u32,
    due: std::collections::HashMap<u32, Box<dyn FnOnce()>>,
}

/// The worker behind [`after`] (`web/timer-worker.js`, beside the page): its
/// `setTimeout` keeps pace in a background tab, where the page's is
/// throttled. A same-origin file, never a `blob:` URL: the PLM-hosted app
/// runs under `worker-src 'self'`, which refuses a blob worker silently (it
/// starts, never runs, and every wait on it would hang).
fn timer() -> Option<Timer> {
    let worker = web_sys::Worker::new("./timer-worker.js").ok()?;
    let on_message = Closure::wrap(Box::new(move |event: web_sys::MessageEvent| {
        if let Some(id) = event.data().as_f64() {
            fire(id as u32);
        }
    }) as Box<dyn FnMut(web_sys::MessageEvent)>);
    worker.set_onmessage(Some(on_message.as_ref().unchecked_ref()));
    Some(Timer { worker, _on_message: on_message, next: 0, due: Default::default() })
}

/// Run timer `id`'s callback, once: whichever of the worker and the page's
/// own timer gets there first ([`after`]).
fn fire(id: u32) {
    let callback = TIMER.with(|t| t.borrow_mut().as_mut().and_then(|t| t.due.remove(&id)));
    if let Some(callback) = callback {
        callback();
    }
}

/// Call `callback` after `ms`, on the page's thread. The delay runs in the
/// timer worker (on pace in a background tab) AND on the page's own timer;
/// the first to fire runs it. So a worker the page may not start, or one that
/// dies, costs only the page timer's throttling, never a wait that hangs.
pub fn after(ms: f64, callback: impl FnOnce() + 'static) {
    let ms = ms.max(0.0);
    let callback: Box<dyn FnOnce()> = Box::new(callback);
    // Stored under an id the worker's reply and the page timer both name.
    let stored = TIMER.with(|t| {
        let mut t = t.borrow_mut();
        if t.is_none() {
            *t = timer();
        }
        match t.as_mut() {
            Some(timer) => {
                timer.next = timer.next.wrapping_add(1);
                let id = timer.next;
                timer.due.insert(id, callback);
                let _ = timer.worker.post_message(&js_sys::Array::of2(&JsValue::from(id), &JsValue::from_f64(ms)));
                Ok(id)
            }
            None => Err(callback),
        }
    });
    let page_timer = match stored {
        Ok(id) => Closure::once_into_js(move || fire(id)),
        // No worker could be made: the page's timer alone.
        Err(callback) => Closure::once_into_js(callback),
    };
    if let Some(window) = web_sys::window() {
        let _ = window.set_timeout_with_callback_and_timeout_and_arguments_0(page_timer.unchecked_ref(), ms as i32);
    }
}

/// Run the pump as a fresh task, once, soon.
pub fn wake() {
    if POSTED.with(|p| p.replace(true)) {
        return;
    }
    PUMP.with(|p| {
        if let Some(pump) = p.borrow().as_ref() {
            let _ = pump.port.post_message(&JsValue::NULL);
        }
    });
}

/// Run the pump after `ms`, unless one is armed already.
fn wake_after(ms: f64) {
    if TIMED.with(|t| t.replace(true)) {
        return;
    }
    after(ms, || {
        TIMED.with(|t| t.set(false));
        pump();
    });
}

/// No frame is coming soon: the tab is hidden, or frames stopped.
fn frames_stopped(ctx: &egui::Context, now: f64) -> bool {
    let frame = ctx.cumulative_frame_nr();
    let (seen, at) = SEEN_FRAME.with(Cell::get);
    let since = if seen == frame { at } else {
        SEEN_FRAME.with(|s| s.set((frame, now)));
        now
    };
    tab_hidden() || now - since >= FRAME_STALE_MS
}

/// One service step, and a pass if one is owed and no frame will run it.
fn pump() {
    let Some((runner, ctx, render)) = PUMP.with(|p| p.borrow().as_ref().map(|p| (p.runner.clone(), p.ctx.clone(), p.render.clone()))) else {
        return;
    };
    // eframe holds the app while it runs a frame: come back right after.
    let Some(mut app) = runner.app_mut::<BrepApp>() else {
        wake_after(PASS_PACE_MS);
        return;
    };
    let busy = app.service_automation(&ctx);
    let queue = app.automation().clone();
    let landed = EVENTS.with(|e| !e.borrow().is_empty());
    let owed = queue.needs_pass() || landed;
    let now = now_ms();
    let mut rearm = busy || CAPTURES_OUT.with(Cell::get) > 0;
    if owed {
        if frames_stopped(&ctx, now) {
            let wait = LAST_PASS.with(Cell::get) + PASS_PACE_MS - now;
            if wait <= 0.0 {
                run_pass(&mut app, &ctx, render.as_ref());
                LAST_PASS.with(|l| l.set(now_ms()));
                // The pass may leave more owed (an input's follow-up pass).
                rearm = true;
            } else {
                drop(app);
                wake_after(wait);
                return;
            }
        } else {
            // A frame is coming; check again in case it does not.
            rearm = true;
        }
    }
    if let (Some(render), true) = (&render, CAPTURES_OUT.with(Cell::get) > 0) {
        // A WebGL device maps buffers on poll.
        let _ = render.device.poll(eframe::egui_wgpu::wgpu::PollType::Poll);
    }
    drop(app);
    if rearm {
        wake_after(PASS_PACE_MS);
    }
}

/// One pass of the app now, never presented ([`crate::on_demand`]), with the
/// last pass's input standing for the canvas.
fn run_pass(app: &mut BrepApp, ctx: &egui::Context, render: Option<&egui_wgpu::RenderState>) {
    let mut raw = ctx.input(|i| i.raw.clone());
    raw.events = EVENTS.with(|e| std::mem::take(&mut *e.borrow_mut()));
    raw.hovered_files.clear();
    raw.dropped_files.clear();
    // egui's clock on the web is `performance.now()` in seconds (eframe's).
    raw.time = Some(now_ms() / 1000.0);
    raw.viewports.entry(egui::ViewportId::ROOT).or_default().events.clear();
    PASSES.with(|p| p.set(p.get() + 1));
    let pass = crate::on_demand::run(app, ctx, render, raw);
    if !pass.captures.is_empty() {
        match render {
            Some(render) => {
                let (size_px, clear) = crate::on_demand::capture_target(app, ctx, pass.pixels_per_point);
                let drawn = crate::offscreen::draw(render, ctx, pass.shapes, pass.pixels_per_point, size_px, clear);
                CAPTURES_OUT.with(|c| c.set(c.get() + 1));
                let captures = pass.captures;
                let queue = app.automation().clone();
                drawn.on_ready(move |image| {
                    CAPTURES_OUT.with(|c| c.set(c.get().saturating_sub(1)));
                    match image {
                        Ok(image) => EVENTS.with(|e| e.borrow_mut().extend(crate::on_demand::screenshot_events(captures, image))),
                        Err(error) => {
                            log::error!("{error}");
                            queue.fail_captures(&error);
                        }
                    }
                    wake();
                });
                let _ = render.device.poll(eframe::egui_wgpu::wgpu::PollType::Poll);
            }
            None => app.automation().fail_captures("offscreen capture: the page has no wgpu render state"),
        }
    }
    if let Some(render) = render {
        crate::offscreen::apply_texture_frees(render, &pass.textures_delta);
    }
}
