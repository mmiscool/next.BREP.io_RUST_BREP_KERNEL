//! A content-keyed memo of the per-coedge READINGS the boolean's operand
//! preparation and assembly take on every call: the inherited-trim refit's
//! detection miss (`heal::refit_inherited_trims`, every coedge of both
//! operands) and the inherited-trim challenger's old-trim tracking
//! (`boolean::assemble::inherited`, every edge of the touched scope).
//!
//! Each reading is a pure function of one coedge's use — the face's carrier,
//! the pcurve, the edge's curve and domain and the coedge's sense — so two
//! uses with the same content read the same number, bit for bit. The
//! assembly's sign check (`assemble.mass_check`, one face term per face of
//! the whole body about the shell's reference point) is memoised the same
//! way, per face: the reference point is the shell's first trim start, which
//! the composition's booleans leave where it was. The fillet's
//! cutter composition hands the same body, less a few faces, to boolean after
//! boolean (35 on the 2026-10-13 sprocket report, 240 faces each), and before
//! this memo each of them re-read every untouched coedge: 95 s of that
//! report's 243 s were the refit's detection alone, measured under load.
//!
//! The key is the 128-bit content digest of `identity.rs` (the mass cache's
//! hasher, chosen there over `FxHasher` because control-point words are
//! structured and a collision here would serve a wrong reading silently).
//! The memo is per thread, bounded (it is cleared, not evicted, past
//! [`CAPACITY`] entries) and cleared with the history cache.

use rustc_hash::FxHashMap as HashMap;
use std::cell::{Cell, RefCell};

/// Which reading a memo entry holds; part of the key.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub(crate) enum Reading {
    /// `heal::refit_inherited_trims`' detection miss (the worst same-parameter
    /// distance between the pcurve's image and the edge).
    RefitDetection,
    /// The inherited-trim challenger's old-trim tracking.
    OldTracking,
    /// One face's volume contribution about the shell's reference point
    /// under the sign reading's policy (`solid_signed_volume_for_sign`'s
    /// per-face term, the boolean assembly's inverted-shell gate).
    SignContribution,
}

/// Entries kept before the memo is cleared whole (a bound on memory, never a
/// decision input: a cleared memo re-reads, bit for bit).
const CAPACITY: usize = 1 << 16;

thread_local! {
    static MEMO: RefCell<HashMap<(Reading, [u64; 2]), Option<f64>>> = RefCell::new(HashMap::default());
    static HITS: Cell<u64> = const { Cell::new(0) };
    static MISSES: Cell<u64> = const { Cell::new(0) };
}

thread_local! {
    /// `BREP_READING_MEMO=0` turns the memo off: a DIAGNOSTIC hatch for a
    /// same-binary A/B (attribution only, never a production setting), read
    /// once per thread.
    static HATCH_OFF: std::cell::OnceCell<bool> = const { std::cell::OnceCell::new() };
}

fn memo_on() -> bool {
    !HATCH_OFF.with(|cell| *cell.get_or_init(|| std::env::var("BREP_READING_MEMO").as_deref() == Ok("0")))
}

/// `read()`'s answer for `key`, served from the memo when it holds one.
pub(crate) fn memoized(reading: Reading, key: [u64; 2], read: impl FnOnce() -> Option<f64>) -> Option<f64> {
    if !memo_on() {
        return read();
    }
    if let Some(value) = MEMO.with(|memo| memo.borrow().get(&(reading, key)).copied()) {
        HITS.with(|hits| hits.set(hits.get() + 1));
        return value;
    }
    MISSES.with(|misses| misses.set(misses.get() + 1));
    let value = read();
    MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        if memo.len() >= CAPACITY {
            memo.clear();
        }
        memo.insert((reading, key), value);
    });
    value
}

/// Drop every entry (the history cache's clear, a document switch).
pub(crate) fn clear() {
    MEMO.with(|memo| memo.borrow_mut().clear());
}

/// `(hits, misses)` since the thread started — for a profile line or a test.
pub(crate) fn counts() -> (u64, u64) {
    (HITS.with(|hits| hits.get()), MISSES.with(|misses| misses.get()))
}

