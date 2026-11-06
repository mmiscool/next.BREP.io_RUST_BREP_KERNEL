//! Diagnostic, thread-local wall-time partition. No geometry decisions.
//! Enable with BREP_PROFILE=1 and BREP_PROFILE_WALL=coarse/noalloc/full/sampled.
//! Inclusive phase times overlap; exclusive times partition each outer stage.
use std::cell::Cell;
use std::sync::atomic::{AtomicU8, Ordering};
use web_time::Instant;

pub(crate) const OTHER: usize = 0;
pub(crate) const FIT: usize = 1;
pub(crate) const POINT: usize = 2;
pub(crate) const SEEDED: usize = 3;
pub(crate) const LOOKUP: usize = 4;
pub(crate) const ALLOC: usize = 5;
pub(crate) const MARCH: usize = 6;
pub(crate) const CLASSIFY: usize = 7;
const N: usize = 8;
const NAMES: [&str; N] = ["other", "fit", "point", "seeded", "lookup", "allocation", "march", "classification"];
static MODE: AtomicU8 = AtomicU8::new(0);

#[derive(Clone, Copy)]
struct State {
    active: bool,
    current: usize,
    depth: [u32; N],
    last: Option<Instant>,
    exclusive: [u64; N],
    inclusive: [u64; N],
    calls: [u64; N],
}
impl State {
    const EMPTY: Self = Self { active: false, current: OTHER, depth: [0; N], last: None,
        exclusive: [0; N], inclusive: [0; N], calls: [0; N] };
    fn tick(&mut self, now: Instant) {
        if let Some(last) = self.last {
            let ns = now.duration_since(last).as_nanos() as u64;
            self.exclusive[self.current] += ns;
            for i in 0..N { if self.depth[i] != 0 { self.inclusive[i] += ns; } }
        }
        self.last = Some(now);
    }
}
thread_local! { static STATE: Cell<State> = const { Cell::new(State::EMPTY) }; }


const SAMPLE_WEIGHT: u64 = 128;
const LEAVES: [usize; 3] = [POINT, LOOKUP, ALLOC];
static CALIBRATION: std::sync::OnceLock<[u64; 3]> = std::sync::OnceLock::new();
#[derive(Clone, Copy)]
struct Leaves {
    depth: [u32; 3], events: [u64; 3], samples: [u64; 3], raw: [u64; 3],
    corrected: [u64; 3], squares: [f64; 3], under_point: u64, under_lookup: u64,
    rng: [u64; 3], force: bool,
}
impl Leaves {
    const EMPTY: Self = Self { depth: [0; 3], events: [0; 3], samples: [0; 3], raw: [0; 3],
        corrected: [0; 3], squares: [0.; 3], under_point: 0, under_lookup: 0,
        rng: [0x9e3779b97f4a7c15, 0x243f6a8885a308d3, 0xb7e151628aed2a6b], force: false };
    fn enter(&mut self, index: usize) -> bool {
        let outer = self.depth[index] == 0;
        self.depth[index] += 1;
        if !outer { return false; }
        self.events[index] += 1;
        let mut x = self.rng[index];
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        self.rng[index] = x;
        self.force || x & (SAMPLE_WEIGHT - 1) == 0
    }
    fn leave(&mut self, index: usize, elapsed: Option<u64>, calibration: u64) {
        self.depth[index] -= 1;
        if let Some(raw) = elapsed {
            let corrected = raw.saturating_sub(calibration);
            self.samples[index] += 1; self.raw[index] += raw;
            self.corrected[index] += corrected;
            self.squares[index] += (corrected as f64).powi(2);
            if index == 2 {
                if self.depth[0] != 0 { self.under_point += corrected; }
                if self.depth[1] != 0 { self.under_lookup += corrected; }
            }
        }
    }
}
thread_local! { static LEAF_STATE: std::cell::RefCell<Leaves> = const { std::cell::RefCell::new(Leaves::EMPTY) }; }
fn take_leaves() -> Leaves {
    LEAF_STATE.with(|cell| {
        let mut state = cell.borrow_mut();
        let old = *state;
        *state = Leaves { rng: old.rng, ..Leaves::EMPTY };
        old
    })
}
fn calibrate() -> [u64; 3] {
    LEAF_STATE.with(|cell| cell.borrow_mut().force = true);
    for bucket in LEAVES {
        for _ in 0..50_000 { std::hint::black_box(Scope::sampled_enter(bucket)); }
    }
    let empty = take_leaves();
    let result = std::array::from_fn(|i| empty.raw[i] / empty.samples[i]);
    profile_line!("wall.calibration point_ns={} lookup_ns={} allocation_ns={} probes=50000", result[0], result[1], result[2]);
    result
}

pub(crate) struct Stage { name: &'static str, start: Instant }
impl Stage {
    pub(crate) fn open(name: &'static str) -> Option<Self> {
        if !crate::profile::profile_enabled() { return None; }
        static CONFIG: std::sync::OnceLock<u8> = std::sync::OnceLock::new();
        let mode = *CONFIG.get_or_init(|| match env_read!("BREP_PROFILE_WALL").as_deref() {
            Some("coarse") => 1, Some("noalloc") => 2, Some("full") => 3, Some("sampled") => 4, _ => 0,
        });
        if mode == 0 { return None; }
        MODE.store(mode, Ordering::Relaxed);
        if STATE.with(|cell| cell.get().active) { return None; }
        if mode == 4 { CALIBRATION.get_or_init(calibrate); }
        STATE.with(|cell| {
            if cell.get().active { return None; } // Nested stages belong to their outer stage.
            crate::reader_profile::begin();
            let start = Instant::now();
            cell.set(State { active: true, last: Some(start), ..State::EMPTY });
            Some(Self { name, start })
        })
    }
}
impl Drop for Stage {
    fn drop(&mut self) {
        let end = Instant::now();
        let state = STATE.with(|cell| {
            let mut state = cell.replace(State::EMPTY);
            state.tick(end);
            state
        });
        crate::reader_profile::end(self.name);
        let total = end.duration_since(self.start).as_nanos() as u64;
        let leaves = (MODE.load(Ordering::Relaxed) == 4).then(take_leaves);
        debug_assert_eq!(state.exclusive.iter().sum::<u64>(), total);
        debug_assert_eq!(state.depth, [0; N]);
        for i in 0..N {
            profile_line!("wall.profile stage={} bucket={} total_ns={} exclusive_ns={} inclusive_ns={} calls={} mode={}",
                self.name, NAMES[i], total, state.exclusive[i], state.inclusive[i], state.calls[i], MODE.load(Ordering::Relaxed));
        }
        if let Some(leaves) = leaves {
            for (i, bucket) in LEAVES.into_iter().enumerate() {
                profile_line!("wall.sample stage={} bucket={} total_ns={} events={} samples={} raw_ns={} corrected_ns={} squares_ns2={} weight={} calibration_ns={} depths_zero={}",
                    self.name, NAMES[bucket], total, leaves.events[i], leaves.samples[i], leaves.raw[i], leaves.corrected[i], leaves.squares[i],
                    SAMPLE_WEIGHT, CALIBRATION.get().unwrap()[i], leaves.depth == [0; 3]);
            }
            profile_line!("wall.sample_overlap stage={} allocation_in_point_ns={} allocation_in_lookup_ns={} weight={}",
                self.name, leaves.under_point, leaves.under_lookup, SAMPLE_WEIGHT);
        }
    }
}

pub(crate) struct Scope { bucket: usize, previous: usize, sampled: bool, started: Option<Instant> }
impl Scope {
    fn sampled_enter(bucket: usize) -> Self {
        let index = match bucket { POINT => 0, LOOKUP => 1, ALLOC => 2, _ => unreachable!() };
        let sample = LEAF_STATE.with(|cell| cell.borrow_mut().enter(index));
        Self { bucket, previous: index, sampled: true, started: sample.then(Instant::now) }
    }
    #[inline]
    pub(crate) fn enter(bucket: usize) -> Option<Self> {
        let mode = MODE.load(Ordering::Relaxed);
        if mode == 0 || (mode == 1 && matches!(bucket, POINT | LOOKUP | ALLOC)) || (bucket == ALLOC && mode != 3 && mode != 4) { return None; }
        if mode == 4 && matches!(bucket, POINT | LOOKUP | ALLOC) {
            return STATE.try_with(|cell| cell.get().active.then(|| Self::sampled_enter(bucket))).ok().flatten();
        }
        STATE.try_with(|cell| {
            let mut state = cell.get();
            if !state.active { return None; }
            let previous = state.current;
            state.tick(Instant::now());
            state.current = bucket;
            state.depth[bucket] += 1;
            state.calls[bucket] += 1;
            cell.set(state);
            Some(Self { bucket, previous, sampled: false, started: None })
        }).ok().flatten()
    }
}
impl Drop for Scope {
    fn drop(&mut self) {
        if self.sampled {
            let elapsed = self.started.map(|start| start.elapsed().as_nanos() as u64);
            let calibration = CALIBRATION.get().map_or(0, |c| c[self.previous]);
            let _ = LEAF_STATE.try_with(|cell| cell.borrow_mut().leave(self.previous, elapsed, calibration));
            return;
        }
        let _ = STATE.try_with(|cell| {
            let mut state = cell.get();
            state.tick(Instant::now());
            state.depth[self.bucket] -= 1;
            state.current = self.previous;
            cell.set(state);
        });
    }
}

/// Forward the configured allocator unchanged. Const TLS has no destructor or
/// allocating initialization; try_with also tolerates thread shutdown. No
/// logging, environment reads or heap allocations happen inside these hooks.
#[cfg(all(feature = "native-rusty-alloc", not(target_family = "wasm")))]
pub(crate) struct Allocator<A>(pub(crate) A);
#[cfg(all(feature = "native-rusty-alloc", not(target_family = "wasm")))]
unsafe impl<A: std::alloc::GlobalAlloc> std::alloc::GlobalAlloc for Allocator<A> {
    unsafe fn alloc(&self, layout: std::alloc::Layout) -> *mut u8 {
        let _wall = Scope::enter(ALLOC);
        unsafe { self.0.alloc(layout) }
    }
    unsafe fn alloc_zeroed(&self, layout: std::alloc::Layout) -> *mut u8 {
        let _wall = Scope::enter(ALLOC);
        unsafe { self.0.alloc_zeroed(layout) }
    }
    unsafe fn dealloc(&self, ptr: *mut u8, layout: std::alloc::Layout) {
        let _wall = Scope::enter(ALLOC);
        unsafe { self.0.dealloc(ptr, layout) }
    }
    unsafe fn realloc(&self, ptr: *mut u8, layout: std::alloc::Layout, size: usize) -> *mut u8 {
        let _wall = Scope::enter(ALLOC);
        unsafe { self.0.realloc(ptr, layout, size) }
    }
}
