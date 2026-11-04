//! PROFILE (consolidation I8, 2026-11-01): the kernel's `BREP_PROFILE`
//! plumbing in one place.
//!
//! - [`profile_enabled`]: whether `BREP_PROFILE` is set, read once per process
//!   (no test toggles it; never on in the browser).
//! - [`profile_emit`] / `profile_line!` and [`ProfileSink`] (lane P3b row 3,
//!   2026-10-26): the profile lines of one operation are gathered in a
//!   thread-local buffer and written to stderr ONCE when the outermost sink
//!   closes, instead of one unbuffered write per line (the `bench-occ`
//!   comparator timed our driver with `BREP_PROFILE=1` until row 5, and an
//!   outward box shell emitted ~435 lines). Line content is unchanged; lines
//!   emitted outside any open sink go straight to stderr.
//! - [`Counters`] with [`update`], [`armed`] and [`started`]: the thread-local
//!   `Cell` of counters a stage arms at its start and reads at its end
//!   (classification, containment, tessellation, mass properties).
//! - [`elapsed_ms`] and [`lap`]: the clocks those counters and the imprint
//!   and fragment profiles read.

use std::cell::Cell;
use std::thread::LocalKey;
use std::time::Duration;
use web_time::Instant;

/// Whether `BREP_PROFILE` is on, read once per process with the flag grammar
/// (`debug_flag!`): set to anything but `0`, `off` or `false`. A bare presence
/// test made `BREP_PROFILE=0` profile, and readings labelled "off" were
/// instrumented.
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn profile_enabled() -> bool {
    crate::debug_flag!("BREP_PROFILE")
}

/// No environment in the browser; the shipped wasm path never profiles.
#[cfg(target_arch = "wasm32")]
pub(crate) fn profile_enabled() -> bool {
    false
}

thread_local! {
    static PROFILE_SINK: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// One `BREP_PROFILE` line ([`profile_line!`]): buffered while a
/// [`ProfileSink`] is open on this thread, straight to stderr otherwise.
#[doc(hidden)]
pub(crate) fn profile_emit(args: std::fmt::Arguments<'_>) {
    use std::fmt::Write;
    PROFILE_SINK.with(|sink| {
        let mut sink = sink.borrow_mut();
        match sink.as_mut() {
            Some(buffer) => {
                let _ = buffer.write_fmt(args);
                buffer.push('\n');
            }
            None => eprintln!("{args}"),
        }
    });
}

/// Opens the operation's line buffer when `BREP_PROFILE` is set and no sink
/// is open on this thread (a nested operation writes into its parent's);
/// dropping the outermost sink writes the buffer in one call.
pub(crate) struct ProfileSink;
impl ProfileSink {
    pub(crate) fn open() -> Option<Self> {
        if !profile_enabled() {
            return None;
        }
        PROFILE_SINK.with(|sink| {
            let mut sink = sink.borrow_mut();
            if sink.is_some() {
                return None;
            }
            *sink = Some(String::with_capacity(16 * 1024));
            Some(Self)
        })
    }
}
impl Drop for ProfileSink {
    fn drop(&mut self) {
        if let Some(buffer) = PROFILE_SINK.with(|sink| sink.borrow_mut().take()) {
            if !buffer.is_empty() {
                eprint!("{buffer}");
            }
        }
    }
}

/// A stage's thread-local counters: copied out, updated and written back
/// only while armed.
pub(crate) trait Counters: Copy + 'static {
    fn armed(&self) -> bool;
}

/// Read-modify-write `cell`'s counters when they are armed; a no-op (one
/// flag read) otherwise.
#[inline]
pub(crate) fn update<T: Counters>(cell: &'static LocalKey<Cell<T>>, update: impl FnOnce(&mut T)) {
    cell.with(|cell| {
        let mut counters = cell.get();
        if counters.armed() {
            update(&mut counters);
            cell.set(counters);
        }
    });
}

/// Whether `cell`'s counters are armed.
#[inline]
pub(crate) fn armed<T: Counters>(cell: &'static LocalKey<Cell<T>>) -> bool {
    cell.with(|cell| cell.get().armed())
}

/// Now, when `cell`'s counters are armed; `None` otherwise.
#[inline]
pub(crate) fn started<T: Counters>(cell: &'static LocalKey<Cell<T>>) -> Option<Instant> {
    armed(cell).then(Instant::now)
}

/// Milliseconds since `started`; 0 when it is `None`.
#[inline]
pub(crate) fn elapsed_ms(started: Option<Instant>) -> f64 {
    started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0)
}

/// The time since `*started` (zero the first time), restarting the clock;
/// zero and untouched when profiling is off.
#[inline]
pub(crate) fn lap(enabled: bool, started: &mut Option<Instant>) -> Duration {
    if !enabled {
        return Duration::ZERO;
    }
    let now = Instant::now();
    let elapsed = started.map(|s| now - s).unwrap_or_default();
    *started = Some(now);
    elapsed
}
