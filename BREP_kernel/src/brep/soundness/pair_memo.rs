//! A memo of face PAIRS the sweep read clean, keyed on the two faces' mesh
//! fragments and the scan's bands, kept across scans.
//!
//! The sweep tests every pair of overlapping triangles of a body's mesh for a
//! crossing. A feature history scans the whole body after every feature, and
//! a fillet changes three or four of fifty faces: every other pair of faces is
//! swept again over the same triangles with the same answer. A pair's sweep
//! is a pure function of the two fragments (which `face_memo::key` names bit
//! for bit: the face record, its coedges' shared samples, the chord), the weld
//! radius (which decides the shared-vertex skip) and the crossing length floor;
//! a pair that produced no mesh crossing under those inputs produces none
//! again. Such a pair is stored here and, on a later scan with the same
//! inputs, its triangle pairs skip the crossing test. Everything the report
//! counts is kept: a skipped pair still counts its candidates, and it had no
//! crossing, no confirmation and no fold candidate to lose.
//!
//! What is NOT memoised: a pair that crossed in the mesh (confirmed, rejected
//! or undecided), and any pair when the scan census is collecting (its tallies
//! read every candidate). Per thread, bounded (cleared whole past
//! [`CAPACITY`] entries, never evicted piecemeal), cleared with the history
//! cache. `BREP_SOUNDNESS_PAIR_MEMO=0` turns it off (read once).

use crate::mass_properties::TwoLane;
use rustc_hash::FxHashSet;
use std::cell::{Cell, RefCell};

/// Entries kept before the memo is cleared whole.
const CAPACITY: usize = 1 << 18;

thread_local! {
    static MEMO: RefCell<FxHashSet<[u64; 2]>> = RefCell::new(FxHashSet::default());
    static ENABLED: Cell<Option<bool>> = const { Cell::new(None) };
    static HITS: Cell<u64> = const { Cell::new(0) };
    static STORED: Cell<u64> = const { Cell::new(0) };
}

#[cfg(not(target_arch = "wasm32"))]
fn enabled_from_env() -> bool {
    !matches!(std::env::var("BREP_SOUNDNESS_PAIR_MEMO").as_deref(), Ok("0") | Ok("off"))
}

/// No env vars in the browser: the shipped wasm path keeps the memo on.
#[cfg(target_arch = "wasm32")]
fn enabled_from_env() -> bool {
    true
}

pub(crate) fn enabled() -> bool {
    ENABLED.with(|cell| match cell.get() {
        Some(enabled) => enabled,
        None => {
            let enabled = enabled_from_env();
            cell.set(Some(enabled));
            enabled
        }
    })
}


/// The scan inputs a pair's sweep reads besides the two fragments.
pub(crate) fn options_key(chord_tolerance: f64, weld: f64, min_crossing_length: f64) -> [u64; 2] {
    let mut lane = TwoLane::new();
    lane.f64(chord_tolerance);
    lane.f64(weld);
    lane.f64(min_crossing_length);
    lane.finish()
}

/// The key of one unordered face pair (a face against itself included).
pub(crate) fn pair_key(options: [u64; 2], a: [u64; 2], b: [u64; 2]) -> [u64; 2] {
    let (low, high) = if a <= b { (a, b) } else { (b, a) };
    let mut lane = TwoLane::new();
    lane.u64(options[0]);
    lane.u64(options[1]);
    lane.u64(low[0]);
    lane.u64(low[1]);
    lane.u64(high[0]);
    lane.u64(high[1]);
    lane.finish()
}

pub(crate) fn contains(key: [u64; 2]) -> bool {
    let hit = MEMO.with(|memo| memo.borrow().contains(&key));
    if hit {
        HITS.with(|cell| cell.set(cell.get() + 1));
    }
    hit
}

/// Store the pairs that swept clean. Past the bound the memo is cleared
/// whole first.
pub(crate) fn store(keys: impl IntoIterator<Item = [u64; 2]>) {
    MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        for key in keys {
            if memo.len() >= CAPACITY {
                memo.clear();
            }
            if memo.insert(key) {
                STORED.with(|cell| cell.set(cell.get() + 1));
            }
        }
    });
}

/// Forget every pair (the history cache's clear, and the test switch).
pub fn clear() {
    MEMO.with(|memo| memo.borrow_mut().clear());
}

/// Pairs served and pairs stored since the last call, and the entries held.
pub(crate) fn take_counts() -> (u64, u64, usize) {
    let hits = HITS.with(|cell| cell.replace(0));
    let stored = STORED.with(|cell| cell.replace(0));
    (hits, stored, MEMO.with(|memo| memo.borrow().len()))
}
