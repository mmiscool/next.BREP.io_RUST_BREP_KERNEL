//! Where a frame's milliseconds go — the ONE per-frame timing record.
//!
//! A dropped frame rate and a frozen UI are the same fact on wasm: the app is
//! single-threaded, so every millisecond spent between two `ui()` calls is a
//! millisecond the browser cannot deliver a pointer event in. "Slow" is
//! therefore not one number but a split, and the split is what names the owner:
//! encode-and-submit time is the renderer's, publish time is the state
//! registry's, and the gap between them is egui's.
//!
//! Measured with [`web_time::Instant`], which is `performance.now()` on wasm and
//! the monotonic clock natively, so both shells report the same quantity.
//!
//! # Why a static, and why accumulate
//!
//! The phases are timed in three different modules (`app`, `viewport`,
//! `viewport::blit`) that do not share a `&mut` path, so the record lives here
//! as a static — the same shape [`crate::automation::registry`] uses, for the
//! same reason. A phase may be entered SEVERAL times in one frame (the registry
//! publishes from three places); each entry accumulates, and [`tick`] — called
//! once at the top of the frame — banks the frame's totals into the rolling
//! window. So every reported phase is "ms per frame", never "ms per call".
//!
//! `dt` is wall time between successive `tick`s: the frame PERIOD, including
//! whatever the browser did between our frames. It is the number a user feels;
//! the phases explain it.

use std::sync::{Mutex, MutexGuard};
use web_time::Instant;

/// Frames kept in the rolling window (~2 s at 60 fps).
const WINDOW: usize = 120;

/// The timed phases, in report order. `Ui` brackets the whole of
/// `BrepApp::ui`; the rest are nested inside it and do not sum to it — the
/// remainder is egui's own layout, painting and tessellation.
///
/// The reported key for `Ui` is `"ui"`, NOT `"frame"`: a command reply carries
/// its own `frame` (the frame NUMBER) at the top level, and a phase of that
/// name would be overwritten by it in the merged result.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Phase {
    /// The whole `BrepApp::ui` body.
    Ui,
    /// The `__brep*` state-registry publish blocks.
    Publish,
    /// `RenderCore::sync_scene`.
    Sync,
    /// `EngineState::fit_camera_and_overlay` (rebuilds the widget overlay).
    Fit,
    /// `EngineState::ensure_overlays_current`.
    Overlays,
    /// `RenderCore::render_to_view` — encode + submit. On the GL/WebGL2 backend
    /// the driver replays the command buffer synchronously inside submit, so
    /// this is very nearly the whole GPU cost; on WebGPU/Vulkan it is encode
    /// only and the GPU work shows up in `dt` instead.
    Draw,
}

const PHASES: usize = 6;

const NAMES: [&str; PHASES] = ["ui", "publish", "sync", "fit", "overlays", "draw"];

fn slot(phase: Phase) -> usize {
    match phase {
        Phase::Ui => 0,
        Phase::Publish => 1,
        Phase::Sync => 2,
        Phase::Fit => 3,
        Phase::Overlays => 4,
        Phase::Draw => 5,
    }
}

/// A fixed-capacity rolling window of per-frame milliseconds.
struct Ring {
    samples: [f32; WINDOW],
    len: usize,
    next: usize,
}

impl Ring {
    const fn new() -> Self {
        Self { samples: [0.0; WINDOW], len: 0, next: 0 }
    }

    fn push(&mut self, ms: f32) {
        self.samples[self.next] = ms;
        self.next = (self.next + 1) % WINDOW;
        self.len = (self.len + 1).min(WINDOW);
    }

    fn stats(&self) -> Stats {
        if self.len == 0 {
            return Stats::default();
        }
        let live = &self.samples[..self.len];
        let sum: f32 = live.iter().sum();
        let mut sorted: Vec<f32> = live.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // p95 by nearest-rank, so a 120-frame window reports its 114th slowest.
        let rank = ((self.len as f32 * 0.95).ceil() as usize).clamp(1, self.len) - 1;
        Stats {
            avg: sum / self.len as f32,
            p95: sorted[rank],
            max: sorted[self.len - 1],
        }
    }
}

#[derive(Default, Clone, Copy)]
struct Stats {
    avg: f32,
    p95: f32,
    max: f32,
}

struct Perf {
    /// Banked per-frame totals, one ring per phase.
    rings: [Ring; PHASES],
    /// Frame period (wall time between `tick`s).
    dt: Ring,
    /// This frame's running totals, banked by the next `tick`.
    pending: [f32; PHASES],
    /// When the current frame started, for `dt`.
    last_tick: Option<Instant>,
    /// Frames banked since startup.
    frames: u64,
}

static PERF: Mutex<Perf> = Mutex::new(Perf {
    rings: [Ring::new(), Ring::new(), Ring::new(), Ring::new(), Ring::new(), Ring::new()],
    dt: Ring::new(),
    pending: [0.0; PHASES],
    last_tick: None,
    frames: 0,
});

fn lock() -> MutexGuard<'static, Perf> {
    PERF.lock().unwrap_or_else(|p| p.into_inner())
}

/// Open the current frame: bank the previous frame's phase totals and record
/// its period. Call ONCE, at the top of `BrepApp::ui`.
pub fn tick() {
    let now = Instant::now();
    let mut perf = lock();
    if let Some(last) = perf.last_tick {
        let dt = now.duration_since(last).as_secs_f32() * 1000.0;
        perf.dt.push(dt);
        let pending = perf.pending;
        for (index, ms) in pending.iter().enumerate() {
            perf.rings[index].push(*ms);
        }
        perf.frames += 1;
    }
    perf.pending = [0.0; PHASES];
    perf.last_tick = Some(now);
}

/// Time a block into `phase`; the span ends when the guard drops. Several spans
/// of the same phase in one frame ADD UP — the window holds ms per frame.
#[must_use = "the span ends when the guard drops; bind it to a name"]
pub fn span(phase: Phase) -> Span {
    Span { phase, start: Instant::now() }
}

pub struct Span {
    phase: Phase,
    start: Instant,
}

impl Drop for Span {
    fn drop(&mut self) {
        let ms = self.start.elapsed().as_secs_f32() * 1000.0;
        let mut perf = lock();
        perf.pending[slot(self.phase)] += ms;
    }
}

/// The rolling window as JSON: the `__brepPerf` blob and the Info-window rows
/// read this one value, so they cannot disagree.
///
/// `{"frames":N,"window":120,"dt":{"avg":..,"p95":..,"max":..},"frame":{..},…}`
/// — every number in milliseconds.
pub fn json() -> String {
    let perf = lock();
    let mut out = String::with_capacity(320);
    out.push_str("{\"frames\":");
    out.push_str(&perf.frames.to_string());
    out.push_str(",\"window\":");
    out.push_str(&perf.dt.len.to_string());
    let push = |out: &mut String, name: &str, stats: Stats| {
        out.push_str(",\"");
        out.push_str(name);
        out.push_str("\":{\"avg\":");
        out.push_str(&format!("{:.3}", stats.avg));
        out.push_str(",\"p95\":");
        out.push_str(&format!("{:.3}", stats.p95));
        out.push_str(",\"max\":");
        out.push_str(&format!("{:.3}", stats.max));
        out.push('}');
    };
    push(&mut out, "dt", perf.dt.stats());
    for (index, name) in NAMES.iter().enumerate() {
        push(&mut out, name, perf.rings[index].stats());
    }
    out.push('}');
    out
}

/// Drop every sample, keeping the frame counter's meaning ("frames banked")
/// intact only for the new window. A measurement resets, then spins, then
/// reads — otherwise the idle frames before the spin dominate the average.
pub fn reset() {
    let mut perf = lock();
    for ring in perf.rings.iter_mut() {
        *ring = Ring::new();
    }
    perf.dt = Ring::new();
    perf.pending = [0.0; PHASES];
    perf.frames = 0;
    perf.last_tick = None;
}
