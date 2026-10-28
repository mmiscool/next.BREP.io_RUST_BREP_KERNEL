//! `timing.*` measures of the shell pipeline. One accumulator per thread, reset
//! when `offset_shell` starts and read when it returns:
//! `offset_shell_with_diagnostics` copies every phase into the outcome's
//! `measurements` as `timing.<phase>_ms`, and `BREP_PROFILE` prints the same
//! split on stderr, in the shape `imprint.profile` and the boolean's `timing.*`
//! rows already use.
//!
//! The phases are the pipeline's own stages, in order, summed over EVERY
//! attempt the shell makes (the oblique-extension retry, the shadowed-support
//! re-run): a lap that fires twice counts twice, and `shell.attempts` says how
//! often `offset_shell_impl` ran. Carrier construction is split by the lane the
//! offset surface took (`OffsetSurfaceLane`), which is what D2's question —
//! does the fit lead? — needs answered per lane rather than per face.
use crate::OffsetSurfaceLane;
use std::cell::RefCell;
use std::collections::BTreeMap;
use web_time::{Duration, Instant};

/// Regularity decisions per retained face: the collapse probe and the carve.
pub(super) const CARRIER_REGULARITY: &str = "carrier_regularity";
/// The offset carrier itself, by lane.
pub(super) const CARRIER_AFFINE: &str = "carrier_affine";
pub(super) const CARRIER_ANALYTIC: &str = "carrier_analytic";
/// The collocation fit, and the reparameterised fit a ruled extension takes.
pub(super) const CARRIER_FIT: &str = "carrier_fit";
/// Everything else the per-face loop does to a carrier: reach and fall-short
/// probes, cut-rim and sharp-side substitutes, outline offsets, vacated rims.
pub(super) const CARRIER_TRIM: &str = "carrier_trim";
pub(super) const APEX_CAPS: &str = "apex_caps";
pub(super) const WALLS: &str = "walls";
/// Smooth-boundary synchronisation, reflex rebuilds and the oblique and
/// fall-short extensions past the opening planes.
pub(super) const EXTENSION: &str = "extension";
pub(super) const IMPRINT: &str = "imprint";
pub(super) const FRAGMENT: &str = "fragment";
pub(super) const SELECT: &str = "select";
/// Source fragments through `finalize_assembled_solid` and the analytic
/// reconstruction: assembly, orientation, connectors, welds, completion, merge.
pub(super) const ASSEMBLY: &str = "assembly";
/// The honesty gates and `accept_sound`, plus the outward connectivity check.
pub(super) const ACCEPTANCE: &str = "acceptance";
pub(super) const TOTAL: &str = "total";

#[derive(Clone, Debug, Default)]
pub(super) struct ShellTimings {
    pub attempts: u64,
    pub carriers: BTreeMap<&'static str, u64>,
    pub ms: BTreeMap<&'static str, f64>,
}

thread_local! {
    static TIMINGS: RefCell<ShellTimings> = RefCell::new(ShellTimings::default());
}

pub(super) fn reset() {
    TIMINGS.with(|timings| *timings.borrow_mut() = ShellTimings::default());
}

pub(super) fn attempt() {
    TIMINGS.with(|timings| timings.borrow_mut().attempts += 1);
}

pub(super) fn add(phase: &'static str, elapsed: Duration) {
    TIMINGS.with(|timings| {
        *timings.borrow_mut().ms.entry(phase).or_default() += elapsed.as_secs_f64() * 1_000.0;
    });
}

/// Charge the time since `started` to `phase` and restart the clock.
pub(super) fn lap(started: &mut Instant, phase: &'static str) {
    let now = Instant::now();
    add(phase, now - *started);
    *started = now;
}

/// The carrier-construction phase of one lane.
pub(super) fn carrier_phase(lane: OffsetSurfaceLane) -> &'static str {
    match lane {
        OffsetSurfaceLane::Affine => CARRIER_AFFINE,
        OffsetSurfaceLane::Analytic => CARRIER_ANALYTIC,
        OffsetSurfaceLane::Fit | OffsetSurfaceLane::Reparameterised => CARRIER_FIT,
    }
}

pub(super) fn carrier_built(lane: OffsetSurfaceLane, elapsed: Duration) {
    let phase = carrier_phase(lane);
    TIMINGS.with(|timings| *timings.borrow_mut().carriers.entry(phase).or_default() += 1);
    add(phase, elapsed);
}

pub(super) fn snapshot() -> ShellTimings {
    TIMINGS.with(|timings| timings.borrow().clone())
}

/// Callee timings stay on stderr, leaving the diagnostic measurement schema
/// unchanged. These are nested within the existing pipeline phase timers.
pub(super) fn detail<T>(callee: &'static str, call: impl FnOnce() -> T) -> T {
    if !crate::imprint::profile_enabled() {
        return call();
    }
    let started = Instant::now();
    let result = call();
    crate::imprint::profile_emit(format_args!("offset_detail.profile callee={callee} ms={:.3}", started.elapsed().as_secs_f64() * 1_000.0));
    result
}

/// The stderr line, when `BREP_PROFILE` is set.
pub(super) fn report(entry: &str) {
    if !crate::imprint::profile_enabled() {
        return;
    }
    let timings = snapshot();
    let phases = timings
        .ms
        .iter()
        .map(|(phase, ms)| format!("{phase}={ms:.2}"))
        .collect::<Vec<_>>()
        .join(" ");
    let carriers = timings
        .carriers
        .iter()
        .map(|(phase, count)| format!("{}={count}", phase.trim_start_matches("carrier_")))
        .collect::<Vec<_>>()
        .join(",");
    crate::imprint::profile_emit(format_args!(
        "{entry}.profile attempts={} carriers[{carriers}] {phases}",
        timings.attempts
    ));
}
