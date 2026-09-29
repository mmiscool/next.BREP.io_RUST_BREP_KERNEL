//! The history-replay TRACE: a native instrument that counts and times every
//! `execute_history` this crate performs, tagged with the SITE that asked for
//! it and the thread it ran on.
//!
//! The engine executes a history on the RUNNER (a background thread natively,
//! a worker in the browser). Anything the main thread re-derives by running the
//! SAME history again is a second full execution of the document — the kernel's
//! incremental cache is per thread, and the wire-harness / assembly / PMI tails
//! are not cached at all, so a tail sweeps once per thread that replays.
//! Whether that is happening cannot be read off a frame time, so this module
//! makes it observable in two ways.
//!
//! A running TALLY, always on, one entry per site: how many executions that
//! site performed, how long they took, and what came back. It is what
//! `perf_stats`'s `runs` block reports and `perf_reset` clears, so a test can
//! bracket one edit and assert that it cost ONE execution — the property the
//! frame time cannot state, because a duplicate replay and a slow machine look
//! alike in milliseconds. The cost is one mutex lock per `execute_history`,
//! which is a millisecond-scale operation.
//!
//! Two things to know before reading a tally. It is PROCESS-global, not
//! per-session — several headless sessions in one process share it — so the
//! only meaningful read is one bracketed by [`reset`]; an unbracketed `runs` is
//! every execution since the process started. And a CANCELLED run contributes
//! nothing: the runner thread is terminated rather than joined, so its span is
//! never dropped and never folded in. A tally of zero after a cancel is the
//! expected reading, not a lost measurement.
//!
//! A per-execution LINE on stderr, when the environment asks for it:
//!
//! ```sh
//! BREP_RUN_TRACE=1 ./brep-app --mcp 2>trace.log
//! ```
//!
//! ```text
//! [run-trace] site=runner thread=ThreadId(2) ms=412.6 features=21 reused=20 errors=0
//! ```
//!
//! `site=runner` is the one execution a run is supposed to cost; every other
//! site on a different thread in the same edit is a duplicate replay. Both
//! halves are a no-op on wasm, where there is no second thread to trace.

/// One site's running total. `features`, `reused` and `errors` are summed over
/// the executions that reported a result — a replay that is "free" because
/// every feature errored out reads the same as a warm cache hit without them.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SiteTally {
    pub count: u64,
    pub ms: f64,
    pub features: u64,
    pub reused: u64,
    pub errors: u64,
}

#[cfg(not(target_arch = "wasm32"))]
mod native {
    use super::SiteTally;
    use std::collections::BTreeMap;
    use std::sync::atomic::{AtomicU8, Ordering};
    use std::sync::Mutex;
    use std::time::Instant;

    static ENABLED: AtomicU8 = AtomicU8::new(0);

    /// A set of per-site totals. A `BTreeMap` keyed by the site's
    /// `&'static str` so the report comes out in a stable order and a new site
    /// needs no registration.
    ///
    /// A TYPE, not just the one static, so a unit test can count into a tally
    /// of its own: the process-global [`TALLY`] is also where every history the
    /// crate's other tests run folds (`pipeline.rs` opens `span("runner")`), and
    /// a test that counted there read another test's run as its own (runner 3
    /// against an expected 2, once in a gate).
    pub struct Tally(Mutex<BTreeMap<&'static str, SiteTally>>);

    impl Tally {
        pub const fn new() -> Self {
            Tally(Mutex::new(BTreeMap::new()))
        }

        fn fold(&self, site: &'static str, ms: f64, features: u64, reused: u64, errors: u64) {
            if let Ok(mut map) = self.0.lock() {
                let entry = map.entry(site).or_default();
                entry.count += 1;
                entry.ms += ms;
                entry.features += features;
                entry.reused += reused;
                entry.errors += errors;
            }
        }

        /// Every site with a total, in name order.
        pub fn sites(&self) -> Vec<(&'static str, SiteTally)> {
            match self.0.lock() {
                Ok(map) => map.iter().map(|(site, t)| (*site, *t)).collect(),
                Err(poisoned) => poisoned.into_inner().iter().map(|(site, t)| (*site, *t)).collect(),
            }
        }

        pub fn clear(&self) {
            match self.0.lock() {
                Ok(mut map) => map.clear(),
                Err(poisoned) => poisoned.into_inner().clear(),
            }
        }
    }

    /// The tally every [`span`] folds into.
    static TALLY: Tally = Tally::new();

    /// Whether `BREP_RUN_TRACE` is set to anything but `0` (read once). Only
    /// the stderr line is gated on it; the tally is always kept.
    pub fn enabled() -> bool {
        match ENABLED.load(Ordering::Relaxed) {
            0 => {
                let on = std::env::var("BREP_RUN_TRACE").is_ok_and(|value| value != "0");
                ENABLED.store(if on { 2 } else { 1 }, Ordering::Relaxed);
                on
            }
            2 => true,
            _ => false,
        }
    }

    /// Every site that has executed a history since the last [`reset`], in name
    /// order.
    pub fn tally() -> Vec<(&'static str, SiteTally)> {
        TALLY.sites()
    }

    /// Drop every site's totals — the counterpart of clearing the frame-timing
    /// window, so a caller can bracket ONE interaction.
    pub fn reset() {
        TALLY.clear()
    }

    /// One traced execution: folds into the tally when dropped, and prints
    /// `site`, the running thread and the elapsed milliseconds when the trace
    /// is switched on.
    pub struct Span {
        sink: &'static Tally,
        site: &'static str,
        start: Instant,
        features: u64,
        reused: u64,
        errors: u64,
        reported: bool,
    }

    impl Span {
        /// Record what the execution actually did — how many features came
        /// back, how many of them the incremental cache replayed, and how many
        /// failed.
        pub fn result(&mut self, result: &brep_kernel::HistoryResult) {
            self.features = result.results.len() as u64;
            self.reused = result.results.iter().filter(|feature| feature.reused).count() as u64;
            self.errors = result.results.iter().filter(|feature| feature.error.is_some()).count() as u64;
            self.reported = true;
        }
    }

    impl Drop for Span {
        fn drop(&mut self) {
            let ms = self.start.elapsed().as_secs_f64() * 1e3;
            self.sink.fold(self.site, ms, self.features, self.reused, self.errors);
            if enabled() {
                let note = if self.reported {
                    format!(" features={} reused={} errors={}", self.features, self.reused, self.errors)
                } else {
                    String::new()
                };
                eprintln!(
                    "[run-trace] site={} thread={:?} ms={ms:.1}{note}",
                    self.site,
                    std::thread::current().id(),
                );
            }
        }
    }

    /// Open a trace span around an `execute_history` call. Always `Some`
    /// natively — the tally is not gated on the environment variable; the
    /// `Option` is the shape wasm needs.
    pub fn span(site: &'static str) -> Option<Span> {
        span_into(site, &TALLY)
    }

    /// A span that folds into `sink` instead of the process-global tally.
    pub(crate) fn span_into(site: &'static str, sink: &'static Tally) -> Option<Span> {
        Some(Span { sink, site, start: Instant::now(), features: 0, reused: 0, errors: 0, reported: false })
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use native::{reset, span, tally, Tally};

/// The wasm stand-in: there is no second thread to trace and no `std::time`.
#[cfg(target_arch = "wasm32")]
pub struct Span;

#[cfg(target_arch = "wasm32")]
impl Span {
    pub fn result(&mut self, _result: &brep_kernel::HistoryResult) {}
}

#[cfg(target_arch = "wasm32")]
pub fn span(_site: &'static str) -> Option<Span> {
    None
}

#[cfg(target_arch = "wasm32")]
pub fn tally() -> Vec<(&'static str, SiteTally)> {
    Vec::new()
}

#[cfg(target_arch = "wasm32")]
pub fn reset() {}

/// The tally as JSON: one object per site plus a `total`, the shape
/// `perf_stats` embeds under `runs`.
pub fn tally_json() -> serde_json::Value {
    json_of(tally())
}

/// [`tally_json`] of any tally's sites.
fn json_of(tally: Vec<(&'static str, SiteTally)>) -> serde_json::Value {
    let mut sites = serde_json::Map::new();
    let mut total = SiteTally::default();
    for (site, t) in tally {
        total.count += t.count;
        total.ms += t.ms;
        total.features += t.features;
        total.reused += t.reused;
        total.errors += t.errors;
        sites.insert(
            site.to_string(),
            serde_json::json!({
                "count": t.count,
                "ms": (t.ms * 10.0).round() / 10.0,
                "features": t.features,
                "reused": t.reused,
                "errors": t.errors,
            }),
        );
    }
    serde_json::json!({
        "sites": serde_json::Value::Object(sites),
        "total": {
            "count": total.count,
            "ms": (total.ms * 10.0).round() / 10.0,
            "features": total.features,
            "reused": total.reused,
            "errors": total.errors,
        },
    })
}

