//! Blend timing and count ledger (`BREP_PROFILE`), shared by blends and the soundness checks.

use std::cell::{Cell, RefCell};
use web_time::Instant;

/// One ledger entry: the measure's name, its accumulated milliseconds
/// (zero for a `count.*` entry) and how many times it was recorded.
#[derive(Clone, Debug, PartialEq)]
pub struct BlendTiming {
    pub measure: String,
    pub ms: f64,
    pub count: u64,
}

thread_local! {
    static CAPTURE: Cell<bool> = const { Cell::new(false) };
    static ENABLED: Cell<bool> = const { Cell::new(false) };
    static TO_STDERR: Cell<bool> = const { Cell::new(false) };
    static LEDGER: RefCell<Vec<BlendTiming>> = const { RefCell::new(Vec::new()) };
}

/// Turn the ledger on (or off) for the blend operations that FOLLOW on
/// this thread; `BREP_PROFILE` turns it on for every operation.
pub fn blend_timing_capture(on: bool) {
    CAPTURE.with(|capture| capture.set(on));
}

/// Take every entry recorded on this thread since the last call (or the
/// last outermost operation's start), sorted by measure name.
pub fn take_blend_timings() -> Vec<BlendTiming> {
    let mut entries = LEDGER.with(|ledger| std::mem::take(&mut *ledger.borrow_mut()));
    entries.sort_by(|a, b| a.measure.cmp(&b.measure));
    entries
}

/// Whether the hooks record anything right now.
pub fn enabled() -> bool {
    ENABLED.with(|enabled| enabled.get())
}

pub fn begin_operation() {
    let profile = crate::profile::profile_enabled();
    TO_STDERR.with(|to_stderr| to_stderr.set(profile));
    let on = profile || CAPTURE.with(|capture| capture.get());
    ENABLED.with(|enabled| enabled.set(on));
    if on {
        LEDGER.with(|ledger| ledger.borrow_mut().clear());
    }
}

pub fn end_operation() {
    if TO_STDERR.with(|to_stderr| to_stderr.get()) {
        let mut entries = LEDGER.with(|ledger| ledger.borrow().clone());
        entries.sort_by(|a, b| a.measure.cmp(&b.measure));
        for entry in entries {
            if entry.measure.starts_with("timing.") {
                eprintln!("blend.{}={:.3} n={}", entry.measure, entry.ms, entry.count);
            } else {
                eprintln!("blend.{}={}", entry.measure, entry.count);
            }
        }
    }
    ENABLED.with(|enabled| enabled.set(false));
}

fn add(measure: &str, ms: f64, count: u64) {
    LEDGER.with(|ledger| {
        let mut ledger = ledger.borrow_mut();
        match ledger.iter_mut().find(|entry| entry.measure == measure) {
            Some(entry) => {
                entry.ms += ms;
                entry.count += count;
            }
            None => ledger.push(BlendTiming { measure: measure.to_string(), ms, count }),
        }
    });
}

/// Add `ms` to a `timing.*` measure (one more occurrence).
pub fn record_ms(measure: &str, ms: f64) {
    if enabled() {
        add(measure, ms, 1);
    }
}

/// Add `n` to a `count.*` measure.
pub fn record_count(measure: &str, n: u64) {
    if enabled() {
        add(measure, 0.0, n);
    }
}

/// A stage timer: records its measure when stopped or dropped. Costs one
/// flag read when the ledger is off.
pub struct BlendTimer {
    measure: &'static str,
    started: Option<Instant>,
}

impl BlendTimer {
    pub fn start(measure: &'static str) -> BlendTimer {
        BlendTimer { measure, started: enabled().then(Instant::now) }
    }

    /// Milliseconds since the start (zero when the ledger is off).
    pub fn elapsed_ms(&self) -> f64 {
        self.started.map_or(0.0, |started| started.elapsed().as_secs_f64() * 1_000.0)
    }

    /// Record now and return the reading.
    pub fn stop(mut self) -> f64 {
        let ms = self.elapsed_ms();
        if self.started.take().is_some() {
            add(self.measure, ms, 1);
        }
        ms
    }
}

impl Drop for BlendTimer {
    fn drop(&mut self) {
        if let Some(started) = self.started.take() {
            add(self.measure, started.elapsed().as_secs_f64() * 1_000.0, 1);
        }
    }
}
