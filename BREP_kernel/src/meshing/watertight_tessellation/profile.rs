//! Stage attribution for the watertight tessellator, on the same switch the
//! boolean's `boolean.<stage>_ms` and the integrator's `mass.profile` lines
//! use: set `BREP_PROFILE` and one `tess.profile` line per
//! [`tessellate_brep_watertight`](super::tessellate_brep_watertight) call goes
//! to stderr. Disabled (one `Cell` read) otherwise.
//!
//! The counters are the ones that told the 2026-09-13 audit where a fine-chord
//! tessellation actually spends its time: the interior seed's per-station
//! rejection scans (`margin_tests`, `parity_tests` — both linear in the
//! boundary before the grid index) against the refinement passes.
use std::cell::Cell;
use web_time::Instant;

#[derive(Default, Clone, Copy)]
pub(in crate::watertight_tessellation) struct TessProfile {
    pub enabled: bool,
    pub faces: u64,
    pub edge_sampling_ms: f64,
    pub seed_ms: f64,
    pub refine_ms: f64,
    /// Grid stations the seed offered (`nu`×`nv` after the budget clamp).
    pub stations: u64,
    /// Stations that survived the margin/parity test and were inserted.
    pub stations_placed: u64,
    /// Point-to-segment distance tests the margin filter performed.
    pub margin_tests: u64,
    /// Ring-crossing tests the parity containment test performed.
    pub parity_tests: u64,
    /// Triangles visited by the host-triangle search.
    pub host_visits: u64,
    pub refine_passes: u64,
    pub splits: u64,
    /// Lawson flipping, from both the seed and the refinement.
    pub flip_ms: f64,
    pub flip_rounds: u64,
    pub flips: u64,
    /// The seed's host-triangle search.
    pub host_ms: f64,
    /// Protected-segment crossing tests a candidate flip performed.
    pub boundary_cross_tests: u64,
}

thread_local! {
    static PROFILE: Cell<TessProfile> = const { Cell::new(TessProfile {
        enabled: false, faces: 0, edge_sampling_ms: 0.0, seed_ms: 0.0, refine_ms: 0.0,
        stations: 0, stations_placed: 0, margin_tests: 0, parity_tests: 0, host_visits: 0,
        refine_passes: 0, splits: 0, flip_ms: 0.0, flip_rounds: 0, flips: 0, host_ms: 0.0,
        boundary_cross_tests: 0,
    }) };
}

/// Read-modify-write the thread-local profile when it is enabled; a no-op
/// (one flag read) otherwise.
pub(in crate::watertight_tessellation) fn tess_profile(update: impl FnOnce(&mut TessProfile)) {
    PROFILE.with(|cell| {
        let mut profile = cell.get();
        if profile.enabled {
            update(&mut profile);
            cell.set(profile);
        }
    });
}

pub(in crate::watertight_tessellation) fn tess_profile_enabled() -> bool {
    PROFILE.with(|cell| cell.get().enabled)
}

/// Milliseconds since `started` when profiling is on; `None` otherwise.
pub(in crate::watertight_tessellation) fn profile_started() -> Option<Instant> {
    tess_profile_enabled().then(Instant::now)
}

pub(in crate::watertight_tessellation) fn elapsed_ms(started: Option<Instant>) -> f64 {
    started.map_or(0.0, |s| s.elapsed().as_secs_f64() * 1_000.0)
}

/// Start a profiled tessellation: resets the counters and arms them when
/// `BREP_PROFILE` is set.
pub(in crate::watertight_tessellation) fn begin() -> Option<Instant> {
    let enabled = profile_switch();
    PROFILE.with(|cell| {
        cell.set(TessProfile {
            enabled,
            ..TessProfile::default()
        })
    });
    enabled.then(Instant::now)
}

#[cfg(not(target_arch = "wasm32"))]
fn profile_switch() -> bool {
    std::env::var("BREP_PROFILE").is_ok()
}

/// No env vars in the browser; the shipped wasm path never arms this.
#[cfg(target_arch = "wasm32")]
fn profile_switch() -> bool {
    false
}

/// End a profiled tessellation: print the line and disarm the counters.
pub(in crate::watertight_tessellation) fn end(started: Option<Instant>, chord: f64, triangles: usize) {
    let Some(started) = started else {
        return;
    };
    let p = PROFILE.with(|cell| cell.get());
    eprintln!(
        "tess.profile ms={:.2} chord={chord:.3e} faces={} tris={triangles} | \
         edge_sampling_ms={:.2} seed_ms={:.2} refine_ms={:.2} | stations={} placed={} \
         margin_tests={} parity_tests={} host_ms={:.2} host_visits={} \
         flip_ms={:.2} flip_rounds={} flips={} cross_tests={} refine_passes={} splits={}",
        started.elapsed().as_secs_f64() * 1_000.0,
        p.faces,
        p.edge_sampling_ms,
        p.seed_ms,
        p.refine_ms,
        p.stations,
        p.stations_placed,
        p.margin_tests,
        p.parity_tests,
        p.host_ms,
        p.host_visits,
        p.flip_ms,
        p.flip_rounds,
        p.flips,
        p.boundary_cross_tests,
        p.refine_passes,
        p.splits,
    );
    PROFILE.with(|cell| cell.set(TessProfile::default()));
}

/// Which accumulator a [`StageTimer`] adds its lifetime to.
#[derive(Clone, Copy)]
pub(in crate::watertight_tessellation) enum Stage {
    Seed,
    Refine,
    Flip,
    Host,
}

/// Scope timer: adds its lifetime to `stage` when it drops, so a function with
/// several early returns is attributed once at the top.
pub(in crate::watertight_tessellation) struct StageTimer {
    started: Option<Instant>,
    stage: Stage,
}

pub(in crate::watertight_tessellation) fn stage_timer(stage: Stage) -> StageTimer {
    StageTimer {
        started: profile_started(),
        stage,
    }
}

impl Drop for StageTimer {
    fn drop(&mut self) {
        let ms = elapsed_ms(self.started);
        match self.stage {
            Stage::Seed => tess_profile(|p| p.seed_ms += ms),
            Stage::Refine => tess_profile(|p| p.refine_ms += ms),
            Stage::Flip => tess_profile(|p| p.flip_ms += ms),
            Stage::Host => tess_profile(|p| p.host_ms += ms),
        }
    }
}
