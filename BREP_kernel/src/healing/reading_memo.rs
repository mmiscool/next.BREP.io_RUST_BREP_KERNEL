//! A content-keyed memo of the per-coedge READINGS the boolean's operand
//! preparation and assembly take on every call: the inherited-trim refit's
//! detection miss (`heal::refit_inherited_trims`, every coedge of both
//! operands) and the inherited-trim challenger's old-trim tracking
//! (`boolean::assemble::inherited`, every edge of the touched scope).
//!
//! The challenger's candidate fits are served the same way
//! (`inherited::CANDIDATE_MEMO`, the whole read per use and bar): on the
//! sprocket report the same uses were fitted and thrown away on every
//! boolean of F9's cutter composition, 5.6 s of its 32 s.
//!
//! The imprint's per-face march windows (`imprint::support::WINDOW_MEMO`,
//! keyed on the face's content) are cleared here too: the same report's 35
//! imprints rebuilt ~280 unchanged windows each, 1.65 s of F9.
//!
//! The refit's two scope readings — a candidate's standoff from its own
//! carrier and its edge's claim against every incident carrier — are served
//! the same way (the claim from `heal::claim_memo`, keyed on every use of the
//! edge): on the 2026-10-13 sprocket report ("fillet takes for ever") the
//! same 140 inherited trims of the F3/F4 body were candidates on every one of
//! the composition's 35 booleans, declined every time by the same claim
//! (`EdgeOffCarrier`), at 1.9 s of scope reads per boolean — 67 s of the
//! fillet's 140 s with the detection miss already memoised.
//!
//! Each reading is a pure function of one coedge's use — the face's carrier,
//! the pcurve, the edge's curve and domain and the coedge's sense — so two
//! uses with the same content read the same number, bit for bit. The
//! assembly's sign check (`assemble.mass_check`, one face term per face of
//! the whole body about the shell's reference point) is memoised the same
//! way, per face: local volume and vector-area terms can be evaluated at
//! any shell reference, including when a union prepends the next tool's
//! faces. The fillet's
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
    /// `heal::refit_inherited_trims`' candidacy standoff: the worst standoff
    /// of a candidate's edge from the use's own carrier over the trim's
    /// stations, under the policy's model tolerance (the key carries it).
    RefitCandidacy,
    /// Assembly's worst sampled edge-to-carrier distance. The full curve,
    /// represented interval and ordered carriers determine this reading.
    CarrierResidual,
}

/// Entries kept before the memo is cleared whole (a bound on memory, never a
/// decision input: a cleared memo re-reads, bit for bit).
const CAPACITY: usize = 1 << 16;

thread_local! {
    static MEMO: RefCell<HashMap<(Reading, [u64; 2]), Option<f64>>> = RefCell::new(HashMap::default());
    // Volume and vector area / 3, keyed by the face and its local anchor.
    static SIGN_TERMS: RefCell<HashMap<[u64; 2], Option<[f64; 4]>>> = RefCell::new(HashMap::default());
    static HITS: Cell<u64> = const { Cell::new(0) };
    static MISSES: Cell<u64> = const { Cell::new(0) };
}

thread_local! {
    /// `BREP_READING_MEMO=0` turns the memo off: a DIAGNOSTIC hatch for a
    /// same-binary A/B (attribution only, never a production setting), read
    /// once per thread.
    static HATCH_OFF: std::cell::OnceCell<bool> = const { std::cell::OnceCell::new() };
}

/// Whether the memo serves readings (off under the diagnostic hatch or a
/// test's forcing). The claim memo in `heal.rs` is governed by the same answer.
pub(crate) fn memo_on() -> bool {
    !HATCH_OFF.with(|cell| *cell.get_or_init(|| std::env::var("BREP_READING_MEMO").as_deref() == Ok("0")))
}

/// Inspect an existing reading without manufacturing a clean result on a miss.
pub(crate) fn lookup(reading: Reading, key: [u64; 2]) -> Option<Option<f64>> {
    if !memo_on() { return None; }
    let value = MEMO.with(|memo| memo.borrow().get(&(reading, key)).copied());
    if value.is_some() { note(true); }
    value
}

/// Pin successful sign terms for an entire feature fold, even if the bounded
/// cross-feature cache fills. Nested folds share their parent's retention.
/// Storage is released on success, refusal, or unwinding out of the fold.
pub(crate) struct SignFoldScope(bool);
thread_local! {
    static FOLD_TERMS: RefCell<Option<HashMap<[u64; 2], [f64; 4]>>> = const { RefCell::new(None) };
    static SIGN_NEW: Cell<u64> = const { Cell::new(0) };
}
impl SignFoldScope {
    pub(crate) fn begin() -> Self {
        Self(FOLD_TERMS.with(|memo| {
            let mut memo = memo.borrow_mut();
            if memo.is_some() { false } else { *memo = Some(HashMap::default()); true }
        }))
    }
}
impl Drop for SignFoldScope {
    fn drop(&mut self) {
        if self.0 { FOLD_TERMS.with(|memo| *memo.borrow_mut() = None); }
    }
}
pub(crate) fn sign_new() -> u64 { SIGN_NEW.with(Cell::get) }
fn retain_sign(key: [u64; 2], value: Option<[f64; 4]>) {
    if let Some(value) = value {
        FOLD_TERMS.with(|memo| { if let Some(memo) = memo.borrow_mut().as_mut() { memo.insert(key, value); } });
    }
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

/// A face's local volume and vector-area terms under the sign reader's
/// correction policy. Like scalar readings, failures are recomputed by the
/// caller to preserve their error, and disabling/clearing the memo is shared.
pub(crate) fn memoized_sign_terms(key: [u64; 2], read: impl FnOnce() -> Option<[f64; 4]>) -> Option<[f64; 4]> {
    if !memo_on() {
        return read();
    }
    if let Some(value) = FOLD_TERMS.with(|memo| memo.borrow().as_ref().and_then(|m| m.get(&key).copied())) {
        note(true);
        return Some(value);
    }
    if let Some(value) = SIGN_TERMS.with(|memo| memo.borrow().get(&key).copied()) {
        note(true);
        retain_sign(key, value);
        return value;
    }
    note(false);
    SIGN_NEW.with(|n| n.set(n.get() + 1));
    let value = read();
    retain_sign(key, value);
    SIGN_TERMS.with(|memo| {
        let mut memo = memo.borrow_mut();
        if memo.len() >= CAPACITY {
            memo.clear();
        }
        memo.insert(key, value);
    });
    value
}

/// Count a hit or a miss taken by a memo of the same contract that stores
/// another value type (`heal::claim_memo`), so one profile line covers both.
pub(crate) fn note(hit: bool) {
    if hit {
        HITS.with(|hits| hits.set(hits.get() + 1));
    } else {
        MISSES.with(|misses| misses.set(misses.get() + 1));
    }
}

/// The bound every memo of this contract clears itself at.
pub(crate) const fn capacity() -> usize {
    CAPACITY
}

/// Drop every entry (the history cache's clear, a document switch).
pub(crate) fn clear() {
    MEMO.with(|memo| memo.borrow_mut().clear());
    SIGN_TERMS.with(|memo| memo.borrow_mut().clear());
    FOLD_TERMS.with(|memo| { if let Some(memo) = memo.borrow_mut().as_mut() { memo.clear(); } });
    crate::heal::clear_claim_memo();
    crate::boolean::clear_candidate_memo();
    crate::imprint::clear_window_memo();
    crate::imprint::clear_touch_memo();
}

/// `(hits, misses)` since the thread started — for a profile line or a test.
pub(crate) fn counts() -> (u64, u64) {
    (HITS.with(|hits| hits.get()), MISSES.with(|misses| misses.get()))
}


