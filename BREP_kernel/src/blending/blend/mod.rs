//! General rolling-ball blends (Golovanov §4.9 + §6.9 surgery).
//!
//! The blend surface is the rational surface (4.9.1) built from three
//! curves: the two support curves cr(t) ⊂ F1 and cs(t) ⊂ F2 where a
//! sphere of radius ρ touches both faces, and the middle curve at the
//! intersection of the two tangent planes, weighted w(t) = cos(α/2)
//! (4.9.3).  The tangency system (4.9.2) + the section-plane constraint
//! (4.9.5) — the sphere center pinned to the normal plane of the edge —
//! is solved per station by Newton with a finite-difference Jacobian.
//!
//! Topology is rebuilt the §6.9 way, with NO tool solid and NO boolean:
//! each mating face's loop swaps the blended edge's coedge for a coedge
//! on its support curve (trimming the adjacent seam edge when the face
//! is a closed carrier with seam-in-one-loop topology), and the blend
//! face is inserted with its own seam.  Open edges take the same route
//! (`edge/open.rs`), and a whole SELECTION goes through `network.rs`, which
//! marches every stripe against the original solid and solves each shared
//! corner from its ball before anything is cut.

mod carve;
mod chain;
mod collapse;
mod corner;
mod edge;
mod fold;
mod miter;
mod network;
mod planar_chart;
mod restrict;
pub(super) mod rollover;
mod rollover_surgery;
pub(crate) mod runout;
mod stations;
mod rows;
mod track_fit;

thread_local! {
    static REFINE_OFF_FOR_TEST: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Whether the open march's local station refinement runs. Escape hatch
/// `BREP_BLEND_REFINE=0` restores the uniform ladder exactly; a test turns it
/// off for its own thread with `set_refinement_off_for_test`.
pub(crate) fn station_refinement_on() -> bool {
    std::env::var("BREP_BLEND_REFINE").as_deref() != Ok("0")
        && !REFINE_OFF_FOR_TEST.with(|off| off.get())
}

thread_local! {
    static BLEND_NOTES: std::cell::RefCell<Vec<String>> = const { std::cell::RefCell::new(Vec::new()) };
}

thread_local! {
    /// Set only while the open-pad composition (`fillet/edges.rs`, the
    /// 2026-10-17 face-ring report) builds its two stages on this thread.
    static OPEN_PAD_NETWORK: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// Runs `build` as the open-pad composition's network: the open march takes
/// a station where a contact crosses its carrier's domain boundary and the
/// row fit breaks there (`march_open_stations_core`), and the march extends
/// past a section end that lies outside the overshoot
/// (`open_station_parameters`). Both are what the composition's walls need
/// (the F6 blend rail of the reported ring stood 7.1e-5 off its carrier at
/// the boundary; the pads' stripes start before their trims) and both move
/// a body the plain network already built (2026-10-18: the reported model's
/// F6 moved by a refinement rung and a broken row), so outside the
/// composition every network is built exactly as before. Restores the flag
/// after.
pub(crate) fn with_open_pad_network<T>(build: impl FnOnce() -> T) -> T {
    /// Restores the previous flag on every exit path, an unwinding panic
    /// included, so a later network on this thread never inherits it.
    struct Reset(bool);
    impl Drop for Reset {
        fn drop(&mut self) {
            OPEN_PAD_NETWORK.with(|flag| flag.set(self.0));
        }
    }
    let _reset = Reset(OPEN_PAD_NETWORK.with(|flag| flag.replace(true)));
    build()
}

/// Whether the march runs as the open-pad composition's network (see
/// [`with_open_pad_network`]).
pub(in crate::blend) fn open_pad_network_on() -> bool {
    OPEN_PAD_NETWORK.with(|flag| flag.get())
}


thread_local! {
    static EXACT_LANE_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static EXACT_REVOLUTION_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static EXACT_RIM_LANE_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static ACCEPT_ONCE_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static CHAIN_ARC_STATIONS_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static TAPER_NETWORK_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static ROLLOVER_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static EXACT_TRIM_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static UNEQUAL_MITER_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static NO_COMMON_BALL_STAR_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
    static CROSSED_RAILS_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

/// Off, the network names the stripe `crossed_rails` and the selection
/// falls through to the cutter composition, whose body is the union of the
/// canal removals (a ridge). `BREP_BLEND_CROSSED_RAILS=0` is the escape
/// hatch, `=1` forces it on; `set_crossed_rails_for_test` forces either way
/// on a test's thread.
const CROSSED_RAILS_DEFAULT: bool = true;

pub(crate) fn crossed_rails_on() -> bool {
    if let Some(forced) = CROSSED_RAILS_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_CROSSED_RAILS").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => CROSSED_RAILS_DEFAULT,
    }
}

/// Off, the network refuses `unequal_corner_radius` or `n_sided_corner` by
/// name as before. `BREP_BLEND_RIBBON_STAR=0` is the escape hatch, `=1`
/// forces it on; `set_no_common_ball_star_for_test` forces it on a test's
/// thread.
const NO_COMMON_BALL_STAR_DEFAULT: bool = true;

pub(crate) fn no_common_ball_star_on() -> bool {
    if let Some(forced) = NO_COMMON_BALL_STAR_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_RIBBON_STAR").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => NO_COMMON_BALL_STAR_DEFAULT,
    }
}

/// The corner has no common ball; the point both shared-face rails stop at is
/// their rail/rail point (`solve_rail_rail_point`), the seam between the blends
/// is marched on the larger stripe at the smaller stripe's own canal level, and
/// — the seam leaving the smaller strip through its side rail — the larger
/// stripe runs onto that side face and is closed by its section there (the
/// asymmetric miter's connector), so the side face gets two rail vertices.
/// `BREP_BLEND_UNEQUAL_MITER=0` is the escape hatch, `=1` forces it on;
/// `set_unequal_miter_for_test` forces it on a test's thread.
const UNEQUAL_MITER_DEFAULT: bool = true;

pub(crate) fn unequal_miter_on() -> bool {
    if let Some(forced) = UNEQUAL_MITER_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_UNEQUAL_MITER").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => UNEQUAL_MITER_DEFAULT,
    }
}

/// Off, the closed lane names the rollover and declines as it did when the
/// detection landed (`rollover.rs`), and the ladder falls through to the
/// cutter, whose body carries a crease. `BREP_BLEND_ROLLOVER=0` is the
/// escape hatch, `=1` forces it on; `set_rollover_for_test` forces either
/// way on a test's own thread.
const ROLLOVER_DEFAULT: bool = true;

pub(crate) fn rollover_on() -> bool {
    if let Some(forced) = ROLLOVER_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_ROLLOVER").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => ROLLOVER_DEFAULT,
    }
}

/// OPEN stripes: a straight edge between planar mates is one section
/// extruded, so the network marches its stations once and skips the ladder's
/// refinement, fold probe, dense centres and ball-resolve wall reading (the
/// wall is read off the exact rows instead); the body is the ladder's, byte
/// for byte. Landed behind this switch, off by default, until the census
/// promotes it: `BREP_BLEND_EXACT_LANE=1` turns it on, `=0` off; a test
/// forces either way on its own thread with `set_exact_lane_for_test`.
const EXACT_LANE_DEFAULT: bool = true;

/// The exact torus is not the fitted rows' body. `BREP_BLEND_EXACT_RIM=1` turns
/// it on, `=0` off; `set_exact_rim_lane_for_test` forces it on a test's own
/// thread.
const EXACT_RIM_LANE_DEFAULT: bool = true;

pub(crate) fn exact_rim_lane_on() -> bool {
    if let Some(forced) = EXACT_RIM_LANE_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_EXACT_RIM").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => EXACT_RIM_LANE_DEFAULT,
    }
}

pub(crate) fn exact_lane_on() -> bool {
    if let Some(forced) = EXACT_LANE_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_EXACT_LANE").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => EXACT_LANE_DEFAULT,
    }
}

/// The exact rows are not the fitted rows' bytes, so the lane landed behind
/// this switch OFF and was promoted ON by its census (the whole kernel
/// battery, the 96-case manifest at its baseline and the render HLR pins
/// with the switch on). `BREP_BLEND_EXACT_REVOLUTION=0` is the hatch back to
/// the fitted ladder, `=1` forces the lane; a test forces either way on its
/// own thread with `set_exact_revolution_for_test`. The lane declines by
/// name to the ladder (`count.march.exact_revolution_declined`) whenever its
/// witness does not hold.
const EXACT_REVOLUTION_DEFAULT: bool = true;

pub(crate) fn exact_revolution_on() -> bool {
    if let Some(forced) = EXACT_REVOLUTION_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_EXACT_REVOLUTION").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => EXACT_REVOLUTION_DEFAULT,
    }
}

/// The surgery lane's exact support trim (slice 1b of the box fillet's
/// hotspots): on an EXACT EXTRUSION row whose mate is a plane, the rail is
/// a straight line and the plane's (u, v) is affine, so the rail's support
/// trim is the straight line in uv between the feet of the rail's two ends,
/// read once as shipped at the pcurve floor (`exact_planar_trim`); the
/// march trim's read, the track refit and its three reads are skipped. The
/// line is not the refit's curve (two control points against its fitted
/// knots), so the body's pcurve bits move on every box-like fixture; the
/// lane shipped behind this switch, off, until its census (the surgery
/// record of 2026-10-11: battery, suites and case gate unchanged, one HLR
/// input digest moving in pcurve bytes with its drawing byte-identical)
/// re-pinned what moves, and it is ON by default since 2026-10-12.
/// `BREP_BLEND_EXACT_TRIM=0` is the escape hatch back to the refit road
/// (`=1` the lane, as before); `set_exact_trim_for_test` forces either way
/// on a test's own thread.
const EXACT_TRIM_DEFAULT: bool = true;

pub(crate) fn exact_trim_on() -> bool {
    if let Some(forced) = EXACT_TRIM_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_EXACT_TRIM").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => EXACT_TRIM_DEFAULT,
    }
}

/// Same landing rule and switch shape as [`exact_lane_on`]:
/// `BREP_BLEND_ACCEPT_ONCE=1` / `=0`, default below.
const ACCEPT_ONCE_DEFAULT: bool = true;

pub(crate) fn accept_once_on() -> bool {
    if let Some(forced) = ACCEPT_ONCE_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_ACCEPT_ONCE").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => ACCEPT_ONCE_DEFAULT,
    }
}

/// Equal-length chains are unchanged either way; unequal ones move their
/// stations, so it lands behind this switch until its census:
/// `BREP_BLEND_CHAIN_ARC_STATIONS=1` turns it on, `=0` off;
/// `set_chain_arc_stations_for_test` forces it on a test's own thread.
const CHAIN_ARC_STATIONS_DEFAULT: bool = false;

pub(crate) fn chain_arc_stations_on() -> bool {
    if let Some(forced) = CHAIN_ARC_STATIONS_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_CHAIN_ARC_STATIONS").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => CHAIN_ARC_STATIONS_DEFAULT,
    }
}

/// `BREP_BLEND_TAPER_NETWORK=0` is the escape hatch, `=1` forces it on;
/// `set_taper_network_for_test` forces it on a test's own thread.
const TAPER_NETWORK_DEFAULT: bool = true;

pub(crate) fn taper_network_on() -> bool {
    if let Some(forced) = TAPER_NETWORK_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_TAPER_NETWORK").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => TAPER_NETWORK_DEFAULT,
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) enum StripeRadius {
    Constant(f64),
    /// `(edge fraction, radius)` stops sorted by fraction, interpolated
    /// linearly between and held flat past the ends: `blend_edge_variable`'s
    /// convention, which the law entries sample into.
    Stops(Vec<(f64, f64)>),
}

impl StripeRadius {
    /// From a stop list: one constant when every stop carries the same
    /// radius (to 1e-12 relative, the single-edge lane's own test).
    pub(crate) fn from_stops(stops: &[(f64, f64)]) -> Self {
        let mut sorted = stops.to_vec();
        sorted.sort_by(|a, b| a.0.total_cmp(&b.0));
        match sorted.first() {
            Some(&(_, first)) if sorted.iter().all(|(_, r)| (r - first).abs() <= 1e-12 * (1.0 + first.abs())) => {
                StripeRadius::Constant(first)
            }
            _ => StripeRadius::Stops(sorted),
        }
    }

    /// The radius at an edge fraction in `[0, 1]` (clamped).
    pub(crate) fn at_fraction(&self, fraction: f64) -> f64 {
        match self {
            StripeRadius::Constant(radius) => *radius,
            StripeRadius::Stops(stops) => {
                let fraction = fraction.clamp(0.0, 1.0);
                if fraction <= stops[0].0 {
                    return stops[0].1;
                }
                for pair in stops.windows(2) {
                    if fraction <= pair[1].0 {
                        let width = (pair[1].0 - pair[0].0).max(1e-12);
                        return pair[0].1 + (pair[1].1 - pair[0].1) * (fraction - pair[0].0) / width;
                    }
                }
                stops[stops.len() - 1].1
            }
        }
    }

    /// The constant, for a constant law.
    pub(crate) fn constant(&self) -> Option<f64> {
        match self {
            StripeRadius::Constant(radius) => Some(*radius),
            StripeRadius::Stops(_) => None,
        }
    }

    /// The largest radius the law takes.
    pub(crate) fn largest(&self) -> f64 {
        match self {
            StripeRadius::Constant(radius) => *radius,
            StripeRadius::Stops(stops) => stops.iter().map(|(_, r)| *r).fold(0.0, f64::max),
        }
    }

    /// The law's END value at the edge's start (`at_start`) or finish: what
    /// the user asked for at that vertex, and the constant every corner solve
    /// there uses (C2).
    pub(crate) fn end(&self, at_start: bool) -> f64 {
        self.at_fraction(if at_start { 0.0 } else { 1.0 })
    }

    /// Whether every radius is positive and finite.
    pub(crate) fn is_positive(&self) -> bool {
        match self {
            StripeRadius::Constant(radius) => radius.is_finite() && *radius > 0.0,
            StripeRadius::Stops(stops) => !stops.is_empty() && stops.iter().all(|(_, r)| r.is_finite() && *r > 0.0),
        }
    }
}








/// Record what a blend that SHIPS needs its feature to say about it — today,
/// a march whose rails still stand off their carriers after its station
/// ladder and local refinement ran out. Drained per feature by the history
/// loop ([`take_blend_notes`]), beside the soundness acceptance's repairs.
pub(crate) fn record_blend_note(note: String) {
    BLEND_NOTES.with(|notes| notes.borrow_mut().push(note));
}

/// Take every blend note recorded on this thread since the last call.
/// Blending runs on the feature's own thread (no parallel march), so the
/// history loop's drain sees every note its feature recorded.
pub fn take_blend_notes() -> Vec<String> {
    BLEND_NOTES.with(|notes| std::mem::take(&mut *notes.borrow_mut()))
}

/// The ledger's length, and a way back to it: a lane that refuses takes back
/// the notes it recorded, so a note never describes a wall that did not ship.
pub(crate) fn blend_notes_mark() -> usize {
    BLEND_NOTES.with(|notes| notes.borrow().len())
}

pub(crate) fn blend_notes_truncate(mark: usize) {
    BLEND_NOTES.with(|notes| notes.borrow_mut().truncate(mark));
}

/// A fillet chain wall that SHIPPED short of the construction request its
/// accepted rung made (`KernelTolerances::model` on the wall between its
/// rails): which wall — the blend face's name and its exact surface — what
/// was asked, what the wall reads, and the typed budget that ran out.
#[derive(Clone, Debug)]
pub(crate) struct WallModelReport {
    pub(crate) name: Option<String>,
    pub(crate) surface: crate::NurbsSurface,
    pub(crate) request: f64,
    pub(crate) residual: f64,
    /// What else the lane measured that the message must say (an unaccepted
    /// rung's deficit, a ladder rung's local refinement), verbatim.
    pub(crate) detail: Option<String>,
    pub(crate) budget: crate::ApproximationBudget,
}

thread_local! {
    static BLEND_APPROXIMATIONS: std::cell::RefCell<Vec<WallModelReport>> = const { std::cell::RefCell::new(Vec::new()) };
}

thread_local! {
    static BLEND_OPERATION_DEPTH: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

/// One blend OPERATION's scope on this thread: every public blend entry point
/// (and the history loop, around each feature) holds one. The OUTERMOST
/// entry drops whatever an earlier operation recorded and nobody consumed —
/// a direct API caller consumes ([`take_blend_approximations`]) after the
/// operation whose reports it wants, before starting the next — while a
/// nested entry (a fillet's per-edge chain blend) clears nothing, so every
/// report of the operation in flight is kept, however many there are.
pub(crate) struct BlendOperation;

impl BlendOperation {
    pub(crate) fn enter() -> BlendOperation {
        BLEND_OPERATION_DEPTH.with(|depth| {
            if depth.get() == 0 {
                BLEND_APPROXIMATIONS.with(|reports| reports.borrow_mut().clear());
                timing::begin_operation();
            }
            depth.set(depth.get() + 1);
        });
        BlendOperation
    }
}

impl Drop for BlendOperation {
    fn drop(&mut self) {
        BLEND_OPERATION_DEPTH.with(|depth| {
            depth.set(depth.get().saturating_sub(1));
            if depth.get() == 0 {
                timing::end_operation();
            }
        });
    }
}

/// Phase timing of the blend lanes, on the same switch as the boolean's
/// `boolean.<stage>_ms` lines: set `BREP_PROFILE` and the outermost blend
/// operation prints one `blend.<measure>=<ms>` line per measure to stderr when
/// it ends. Off (one `Cell` read per hook), nothing is recorded.
///
/// Measures ACCUMULATE: a stage that runs per stripe, per rung or per
/// component adds to one entry and bumps its count, so the ledger reads as a
/// split of the operation, not as the worst instance. `timing.*` entries are
/// milliseconds, `count.*` entries are plain counters.
pub(crate) mod timing {
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
    pub(crate) fn enabled() -> bool {
        ENABLED.with(|enabled| enabled.get())
    }

    pub(super) fn begin_operation() {
        let profile = std::env::var_os("BREP_PROFILE").is_some();
        TO_STDERR.with(|to_stderr| to_stderr.set(profile));
        let on = profile || CAPTURE.with(|capture| capture.get());
        ENABLED.with(|enabled| enabled.set(on));
        if on {
            LEDGER.with(|ledger| ledger.borrow_mut().clear());
        }
    }

    pub(super) fn end_operation() {
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
    pub(crate) fn record_ms(measure: &str, ms: f64) {
        if enabled() {
            add(measure, ms, 1);
        }
    }

    /// Add `n` to a `count.*` measure.
    pub(crate) fn record_count(measure: &str, n: u64) {
        if enabled() {
            add(measure, 0.0, n);
        }
    }

    /// A stage timer: records its measure when stopped or dropped. Costs one
    /// flag read when the ledger is off.
    pub(crate) struct BlendTimer {
        measure: &'static str,
        started: Option<Instant>,
    }

    impl BlendTimer {
        pub(crate) fn start(measure: &'static str) -> BlendTimer {
            BlendTimer { measure, started: enabled().then(Instant::now) }
        }

        /// Milliseconds since the start (zero when the ledger is off).
        pub(crate) fn elapsed_ms(&self) -> f64 {
            self.started.map_or(0.0, |started| started.elapsed().as_secs_f64() * 1_000.0)
        }

        /// Record now and return the reading.
        pub(crate) fn stop(mut self) -> f64 {
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
}
pub use timing::{blend_timing_capture, take_blend_timings, BlendTiming};
pub(crate) use timing::{record_count, BlendTimer};

/// Record a wall that ships with its request unmet. Written by the rung that
/// keeps its body; whether the body reaches a caller is decided at
/// consumption ([`take_blend_approximations`]), not here. Every report of
/// the operation in flight is kept (see [`BlendOperation`]).
pub(crate) fn record_wall_model_report(report: WallModelReport) {
    BLEND_APPROXIMATIONS.with(|reports| reports.borrow_mut().push(report));
}


/// Whether any wall-model report is waiting (the history loop reads the
/// returned bodies only then).
pub(crate) fn blend_approximations_pending() -> bool {
    BLEND_APPROXIMATIONS.with(|reports| !reports.borrow().is_empty())
}

/// Bit-identical surfaces: the same degrees, knots and homogeneous control
/// points, compared by their bits.
pub(crate) fn same_surface(a: &crate::NurbsSurface, b: &crate::NurbsSurface) -> bool {
    let bits = |values: &[f64]| values.iter().map(|value| value.to_bits()).collect::<Vec<_>>();
    a.degree_u == b.degree_u
        && a.degree_v == b.degree_v
        && bits(&a.knots_u) == bits(&b.knots_u)
        && bits(&a.knots_v) == bits(&b.knots_v)
        && a.control_points.len() == b.control_points.len()
        && a.control_points.iter().zip(&b.control_points).all(|(row_a, row_b)| {
            row_a.len() == row_b.len()
                && row_a.iter().zip(row_b).all(|(p, q)| {
                    [p.x, p.y, p.z, p.w].map(f64::to_bits) == [q.x, q.y, q.z, q.w].map(f64::to_bits)
                })
        })
}

/// Whether `face_name` names the report's wall: its exact name, or that
/// name with the `[k]` (decimal) suffix `ensure_unique_face_names` gives a
/// colliding name at registration. Nothing else: no prefix, no other suffix.
fn names_the_wall(face_name: &Option<String>, report_name: &Option<String>) -> bool {
    match (face_name, report_name) {
        (Some(face), Some(report)) => {
            face == report
                || face.strip_prefix(report.as_str()).and_then(|rest| rest.strip_prefix('[')).and_then(|rest| rest.strip_suffix(']')).is_some_and(
                    |index| !index.is_empty() && index.bytes().all(|byte| byte.is_ascii_digit()),
                )
        }
        (None, None) => true,
        _ => false,
    }
}

/// Consume EVERY wall-model report recorded on this thread since the last
/// call, in ONE snapshot, against ALL the bodies a call returned
/// (`(body name, solid)`): a report becomes a typed `blend.wall_model`
/// [`crate::Approximation`] on each body carrying its wall — a face with a
/// bit-identical surface AND the report's name, exactly or as registration
/// renamed it (`NAME[k]`) — and is dropped when no returned body carries it
/// (a lane that refused, a body a later step discarded). A wall carried by
/// two returned bodies (one surface, bit for bit, in both) is reported on
/// both: the reading is a property of that surface. Reports for one wall
/// collapse to the last. Never a match by name alone.
/// `take_blend_approximations(&[])` discards everything recorded (the
/// history loop's stale drain before a feature runs).
pub fn take_blend_approximations(bodies: &[(&str, &crate::BrepSolid)]) -> Vec<crate::Approximation> {
    let snapshot = BLEND_APPROXIMATIONS.with(|reports| std::mem::take(&mut *reports.borrow_mut()));
    let mut unique: Vec<WallModelReport> = Vec::new();
    for report in snapshot {
        unique.retain(|kept| !(kept.name == report.name && same_surface(&kept.surface, &report.surface)));
        unique.push(report);
    }
    let mut approximations = Vec::new();
    for (body, solid) in bodies {
        for report in &unique {
            let carried = solid.shells.iter().flat_map(|shell| &shell.faces).any(|face| {
                names_the_wall(&face.name, &report.name) && same_surface(&face.surface, &report.surface)
            });
            if carried {
                let wall = report.name.as_deref().unwrap_or("(unnamed)");
                let unaccepted = matches!(report.budget.reason, crate::BudgetReason::Unaccepted | crate::BudgetReason::Unread);
                let budget = match report.budget.mechanism {
                    crate::BudgetMechanism::LocalRounds => format!(
                        "local refinement stopped: {:?} after {} of {} rounds, {} of {} stations per segment",
                        report.budget.reason, report.budget.rounds_used, report.budget.rounds_limit,
                        report.budget.stations, report.budget.station_limit,
                    ),
                    crate::BudgetMechanism::StationRungs => format!(
                        "the open march's station ladder stopped: {:?} after {} of {} global station rungs, {} of {} stations",
                        report.budget.reason, report.budget.rounds_used, report.budget.rounds_limit,
                        report.budget.stations, report.budget.station_limit,
                    ),
                };
                let unknown = report.budget.unread > 0;
                let rails = report.budget.measured_component == crate::MeasuredComponent::Rails;
                let mut message = if !unaccepted && (rails || unknown) {
                    // A request that covers the rails and the wall says WHICH
                    // it measured, and never offers a partial reading as the
                    // worst: with unread intervals the worst is unknown.
                    let what = if rails {
                        format!("the fillet wall {wall}'s support rails stand")
                    } else {
                        format!("the fillet wall {wall} stands")
                    };
                    let against = if rails {
                        "off their exact construction (their carriers / the exact ball contacts)"
                    } else {
                        "from the rolling ball between its rails"
                    };
                    let reading = if unknown {
                        format!(
                            "an UNKNOWN distance {against}: {} interval(s) could not be read (the largest READABLE reading is {:.3e} mm, not the worst)",
                            report.budget.unread, report.residual
                        )
                    } else {
                        format!("{:.3e} mm {against}", report.residual)
                    };
                    format!(
                        "{what} {reading}, against a construction request of {:.3e} mm (accepted at the intersection-fit bar); {budget}",
                        report.request,
                    )
                } else if unaccepted {
                    format!(
                        "the blend wall {wall} ships from a rung that never met acceptance: its worst reading \
                         {:.3e} mm against its acceptance bar {:.3e} mm; {budget}",
                        report.residual, report.request,
                    )
                } else {
                    format!(
                        "the fillet wall {wall} stands {:.3e} mm from the rolling ball between its rails, against a \
                         construction request of {:.3e} mm (accepted at the intersection-fit bar); {budget}",
                        report.residual, report.request,
                    )
                };
                if let Some(detail) = &report.detail {
                    message = format!("{message} ({detail})");
                }
                approximations.push(crate::Approximation {
                    code: if unaccepted { "blend.wall_acceptance" } else { "blend.wall_model" }.to_string(),
                    body: body.to_string(),
                    measured: report.residual,
                    bar: report.request,
                    volume_bound: None,
                    edges: Vec::new(),
                    budget: Some(report.budget.clone()),
                    message,
                });
            }
        }
    }
    approximations
}


pub use chain::{blend_smooth_chain, blend_smooth_chain_if_closed};
pub(crate) use corner::round_concave_chain_corner;
pub use corner::round_convex_corner;
pub use edge::{blend_closed_edge, blend_closed_edge_knot_aware, blend_edge_variable, blend_open_edge, is_pcurve_floor_refusal, with_pcurve_knot_retry};
pub(crate) use collapse::is_full_width_collapse;
pub(crate) use planar_chart::{
    fit_planar_charts_to_trims, is_planar_chart_refusal, planar_chart_overrun,
};
pub(crate) use track_fit::PCURVE_OFF_FLOOR;
pub(crate) use edge::{consumed_band, is_consumed_snap, is_rail_collapse};
pub(crate) use network::{
    blend_star_network, blend_star_network_laws, blend_star_network_with_corner_policy, degenerate_corner_setbacks, mixed_convexity_corner, mixed_convexity_corners, selection_convexity, DegenerateSetback,
    MixedCorner,
};
pub(crate) use miter::{is_marched_fit_off_carriers, is_miter_support_overrun};
/// The two faces each selected edge lies on, in the network's mate order
/// (the open-pad composition reads which is planar and which is the boss).
pub(crate) fn selection_mates<'a>(
    solid: &'a crate::topology::BrepSolid,
    edge_ids: &[u64],
    radius: f64,
) -> Result<Vec<(&'a crate::topology::EdgeRecord, [&'a crate::topology::FaceRecord; 2])>, crate::KernelRefusal> {
    Ok(network::stripe_mates(solid, edge_ids, radius)?
        .into_iter()
        .map(|(edge, first, second)| (edge, [first.face, second.face]))
        .collect())
}

pub(crate) use runout::{build_runout_walls, plan_runout, take_runout_reports, RunoutPlan, RunoutWallReport};
pub(crate) use fold::{is_wall_fold, wall_loop_refusal, WALL_FOLDS};
pub(crate) use stations::is_ball_off_carrier;
// Lane D 2026-10-14 (appended): the inside-corner classification's name and
// its opt-in miter switch, read by the runout composition and its tests.
pub(crate) use network::is_reentrant_concave;

/// Off the switch every closed rim's two support pcurves are refitted to their
/// rail's projected track (`edge/closed.rs::refit_closed_support_pcurve`, the
/// closed chain's support-piece fitter: a dense track of analytic feet, a
/// verified fit ladder and the shipped read), 96 of the half torus's 98 ms of
/// rows. On the switch each trim is first READ as shipped at the same floor
/// (`track_fit::read_shipped_pcurve`, `PCURVE_REFINEMENT_TOLERANCE`): a plane
/// mate's trim is the rail's own control points through the plane's affine
/// chart (exact by construction, the exact revolution lane's rule); any other
/// mate's trim, or an image that misses, takes the refit as before (the
/// march's own interpolated pcurve was measured as a candidate and dropped:
/// 8.5e-5 off on the half torus's torus mate). The shipped contract is
/// unchanged (every trim read within the floor); the bytes are not, so it
/// landed OFF and was promoted by its census (the kernel battery, the case
/// manifest and hlr_tests with the switch on). ON by default;
/// `BREP_BLEND_CLOSED_TRIM_READ_FIRST=0` is the escape hatch back to the refit
/// road, `=1` forces it on; `set_closed_trim_read_first_for_test` forces
/// either way on a test's thread.
const CLOSED_TRIM_READ_FIRST_DEFAULT: bool = true;

thread_local! {
    static CLOSED_TRIM_READ_FIRST_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

pub(crate) fn closed_trim_read_first_on() -> bool {
    if let Some(forced) = CLOSED_TRIM_READ_FIRST_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_CLOSED_TRIM_READ_FIRST").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => CLOSED_TRIM_READ_FIRST_DEFAULT,
    }
}


/// The check solves 31 centres per interval by continuation from the
/// neighbour, each a full station Newton with a finite-difference Jacobian
/// rebuilt at every iteration (eleven evaluations of the tangency system a
/// centre, 89 of the half torus's 169 ms of march). On the switch each centre
/// is seeded by a quadratic extrapolation through the last three and solved
/// by the chord method with the Jacobian carried from centre to centre
/// (`stations::solve_station_state_chord`), to the plain solver's own stop;
/// the plain solver from the neighbour is the fall-back for anything the
/// chord cannot finish. The centres are the same roots to the same tolerance,
/// not the same bits: the declared wall reading moves in its last digits, its
/// verdict and the body do not, which is why it landed OFF and was promoted
/// by its census. ON by default; `BREP_BLEND_CLOSED_WALL_PREDICT=0` is the
/// escape hatch back to the plain continuation solves, `=1` forces it on;
/// `set_closed_wall_predict_for_test` forces either way on a test's thread.
const CLOSED_WALL_PREDICT_DEFAULT: bool = true;

thread_local! {
    static CLOSED_WALL_PREDICT_FOR_TEST: std::cell::Cell<Option<bool>> = const { std::cell::Cell::new(None) };
}

pub(crate) fn closed_wall_predict_on() -> bool {
    if let Some(forced) = CLOSED_WALL_PREDICT_FOR_TEST.with(|forced| forced.get()) {
        return forced;
    }
    match std::env::var("BREP_BLEND_CLOSED_WALL_PREDICT").as_deref() {
        Ok("0") => false,
        Ok("1") => true,
        _ => CLOSED_WALL_PREDICT_DEFAULT,
    }
}


// BREP lane L: begin keyway_floor_open_mixed_corner_20261014
pub(crate) use corner::{open_mixed_corner_on, round_mixed_concave_open_corner};
// BREP lane L: end keyway_floor_open_mixed_corner_20261014

