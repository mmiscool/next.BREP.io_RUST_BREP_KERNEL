//! Optional point-reader attribution. Point and component clocks run in
//! DIFFERENT processes/modes, so inner timers cannot inflate outer samples.
use std::{cell::{Cell, RefCell}, collections::BTreeMap, panic::Location,
    sync::{OnceLock, atomic::{AtomicU8, Ordering}}, time::Instant};
pub(crate) const CURVE: u8 = 0;
pub(crate) const SURFACE: u8 = 1;
pub(crate) const PCURVE: u8 = 2;
pub(crate) const BASIS: u8 = 1;
pub(crate) const ACCUMULATE: u8 = 2;
const KINDS: [&str; 3] = ["curve", "surface", "pcurve"];
const PARTS: [&str; 3] = ["point", "basis", "accumulate"];
const WEIGHT: u64 = 128;
static MODE: AtomicU8 = AtomicU8::new(0);
static FLOOR: OnceLock<u64> = OnceLock::new();
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct Key { kind: u8, part: u8, file: &'static str, line: u32, column: u32 }
#[derive(Clone, Copy)]
struct Frame { active: bool, depth: u32, parts: [u32; 2], key: Option<Key> }
impl Frame {
    const EMPTY: Self = Self { active: false, depth: 0, parts: [0; 2], key: None };
    fn enter(&mut self, key: Key) -> bool {
        let outer = self.depth == 0;
        self.depth += 1;
        if outer { self.key = Some(key); }
        outer
    }
    fn leave(&mut self) {
        self.depth -= 1;
        if self.depth == 0 { self.key = None; }
    }
}
#[derive(Default)]
struct Stat { samples: u64, raw: u64, corrected: u64, squares: f64 }
impl Stat {
    fn add(&mut self, raw: u64, floor: u64) {
        let corrected = raw.saturating_sub(floor);
        self.samples += 1; self.raw += raw; self.corrected += corrected;
        self.squares += (corrected as f64).powi(2);
    }
}
thread_local! {
    static FRAME: Cell<Frame> = const { Cell::new(Frame::EMPTY) };
    static COUNTS: Cell<[[u64; 3]; 3]> = const { Cell::new([[0; 3]; 3]) };
    static RNG: Cell<[u64; 3]> = const { Cell::new([0x9e3779b97f4a7c15, 0x243f6a8885a308d3, 0xb7e151628aed2a6b]) };
    static STATS: RefCell<BTreeMap<Key, Stat>> = const { RefCell::new(BTreeMap::new()) };
}
fn draw(part: u8) -> bool {
    RNG.with(|cell| {
        let mut seeds = cell.get(); let x = &mut seeds[part as usize];
        *x ^= *x << 13; *x ^= *x >> 7; *x ^= *x << 17;
        let selected = *x & (WEIGHT - 1) == 0; cell.set(seeds); selected
    })
}
fn count(key: Key) {
    COUNTS.with(|cell| { let mut counts = cell.get(); counts[key.kind as usize][key.part as usize] += 1; cell.set(counts); });
}
struct Timer(Instant);
impl Timer {
    fn new() -> Self { Self(Instant::now()) }
    fn finish(self) -> u64 { self.0.elapsed().as_nanos() as u64 }
}
fn record(key: Option<Key>, elapsed: Option<u64>) {
    if let (Some(key), Some(elapsed)) = (key, elapsed) {
        STATS.with(|stats| stats.borrow_mut().entry(key).or_default().add(elapsed, *FLOOR.get().unwrap()));
    }
}
pub(crate) fn begin() {
    static CONFIG: OnceLock<u8> = OnceLock::new();
    let mode = *CONFIG.get_or_init(|| match env_read!("BREP_PROFILE_READERS").as_deref() {
        Some("points") => 1, Some("parts") => 2, _ => 0,
    });
    MODE.store(mode, Ordering::Relaxed);
    if mode == 0 { return; }
    FLOOR.get_or_init(|| {
        let mut total = 0;
        for _ in 0..50_000 { let timer = Timer::new(); std::hint::black_box(&timer); total += timer.finish(); }
        let floor = total / 50_000;
        profile_line!("reader.calibration floor_ns={} probes=50000 weight={}", floor, WEIGHT);
        floor
    });
    FRAME.with(|f| f.set(Frame { active: true, ..Frame::EMPTY }));
    COUNTS.with(|c| c.set([[0; 3]; 3]));
    STATS.with(|s| s.borrow_mut().clear());
}
pub(crate) fn end(stage: &'static str) {
    let mode = MODE.load(Ordering::Relaxed);
    if mode == 0 { return; }
    let frame = FRAME.with(|f| f.replace(Frame::EMPTY));
    let counts = COUNTS.with(Cell::get);
    let stats = STATS.with(|s| std::mem::take(&mut *s.borrow_mut()));
    profile_line!("reader.account stage={} balanced={} mode={}", stage, frame.depth == 0 && frame.parts == [0; 2], mode);
    for kind in 0..3 { for part in 0..3 {
        if counts[kind][part] != 0 {
            profile_line!("reader.events stage={} kind={} part={} events={} mode={}", stage, KINDS[kind], PARTS[part], counts[kind][part], mode);
        }
    } }
    for (key, stat) in stats {
        profile_line!("reader.sample stage={} kind={} part={} file={} line={} column={} samples={} raw_ns={} corrected_ns={} squares_ns2={} weight={} floor_ns={} mode={}",
            stage, KINDS[key.kind as usize], PARTS[key.part as usize], key.file, key.line, key.column,
            stat.samples, stat.raw, stat.corrected, stat.squares, WEIGHT, FLOOR.get().unwrap(), mode);
    }
}
pub(crate) struct Point { key: Option<Key>, timer: Option<Timer> }
impl Point {
    #[inline]
    pub(crate) fn open(kind: u8, at: &'static Location<'static>) -> Option<Self> {
        let mode = MODE.load(Ordering::Relaxed);
        if mode == 0 { return None; }
        FRAME.with(|cell| {
            let mut frame = cell.get(); if !frame.active { return None; }
            let key = Key { kind, part: 0, file: at.file(), line: at.line(), column: at.column() };
            let outer = frame.enter(key); cell.set(frame);
            if outer { count(key); }
            let timer = (outer && mode == 1 && draw(0)).then(Timer::new);
            Some(Self { key: outer.then_some(key), timer })
        })
    }
}
impl Drop for Point {
    fn drop(&mut self) {
        let elapsed = self.timer.take().map(Timer::finish);
        FRAME.with(|cell| { let mut frame = cell.get(); frame.leave(); cell.set(frame); });
        record(self.key, elapsed);
    }
}
pub(crate) struct Part { part: u8, key: Option<Key>, timer: Option<Timer> }
impl Part {
    #[inline]
    pub(crate) fn open(part: u8) -> Option<Self> {
        if MODE.load(Ordering::Relaxed) != 2 { return None; }
        FRAME.with(|cell| {
            let mut frame = cell.get(); if !frame.active || frame.depth == 0 { return None; }
            let outer = frame.parts[part as usize - 1] == 0;
            frame.parts[part as usize - 1] += 1;
            let key = Key { part, ..frame.key.unwrap() }; cell.set(frame);
            if outer { count(key); }
            let timer = (outer && draw(part)).then(Timer::new);
            Some(Self { part, key: outer.then_some(key), timer })
        })
    }
}
impl Drop for Part {
    fn drop(&mut self) {
        let elapsed = self.timer.take().map(Timer::finish);
        FRAME.with(|cell| { let mut frame = cell.get(); frame.parts[self.part as usize - 1] -= 1; cell.set(frame); });
        record(self.key, elapsed);
    }
}
