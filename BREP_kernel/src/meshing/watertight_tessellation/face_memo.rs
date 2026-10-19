//! A content-keyed memo of one face's mesh fragment, kept across calls.
//!
//! The per-face tessellator (`tessellate_face_watertight`) is a pure function
//! of three things: the face record as `hash_face` walks it (carrier,
//! `same_sense`, loops, each coedge's sense and pcurve), the shared samples of
//! the edges its coedges name (fractions, positions, worst sag — themselves
//! deterministic functions of the edge curves, the vertex points and the chord
//! tolerance, which is what lets the worker split mesh faces on separate
//! instances), and the chord tolerance. Two faces with the same content mesh
//! to the same bits; the sequential `face_id` only stamps the fragment's
//! `face_ids`. So the fragment is stored under that content and served
//! again with the current stamp.
//!
//! Why it exists: a feature history re-scans the whole body for soundness
//! after every operation, and the scan re-meshes every face. On the 2026-10-17
//! "fillet takes for ever" document the four features after the shell each
//! re-meshed the same 34–50 faces at 0.41–0.49 s a scan, of which the bore
//! wall alone was 0.26 s, while a fillet changes three or four of them. The
//! display path reaches the same entry and benefits the same way.
//!
//! The memo is per thread, bounded by stored triangles and entries (cleared
//! whole, never evicted piecemeal: a cleared memo re-meshes, bit for bit) and
//! cleared with the history cache. The ruling-seed rule is part of the key
//! because a test may toggle it on its own thread. `BREP_TESS_MEMO=0` turns
//! the memo off (a diagnostic hatch, read once).

use super::{EdgeSamples, FaceRecord, HashMap};
use crate::mass_properties::{hash_face, TwoLane};
use crate::Mesh;
use rustc_hash::FxHashMap;
use std::cell::{Cell, RefCell};
use std::rc::Rc;

/// Stored triangles before the memo is cleared whole: about 60 bytes a
/// triangle (positions, normals, indices, face ids), so ~30 MB.
const TRIANGLE_BUDGET: usize = 500_000;
/// Entries before the memo is cleared whole.
const ENTRY_BUDGET: usize = 8_192;

thread_local! {
    static MEMO: RefCell<FxHashMap<[u64; 2], Rc<Mesh>>> = RefCell::new(FxHashMap::default());
    static STORED_TRIANGLES: Cell<usize> = const { Cell::new(0) };
    static HITS: Cell<u64> = const { Cell::new(0) };
    static MISSES: Cell<u64> = const { Cell::new(0) };
    static ENABLED: Cell<Option<bool>> = const { Cell::new(None) };
}

#[cfg(not(target_arch = "wasm32"))]
fn enabled_from_env() -> bool {
    !matches!(std::env::var("BREP_TESS_MEMO").as_deref(), Ok("0") | Ok("off"))
}

/// No env vars in the browser: the shipped wasm path keeps the memo on.
#[cfg(target_arch = "wasm32")]
fn enabled_from_env() -> bool {
    true
}

pub(in crate::watertight_tessellation) fn enabled() -> bool {
    ENABLED.with(|cell| match cell.get() {
        Some(enabled) => enabled,
        None => {
            let enabled = enabled_from_env();
            cell.set(Some(enabled));
            enabled
        }
    })
}


/// The 128-bit content key of one face's fragment: the face as `hash_face`
/// walks it, the chord tolerance, the ruling-seed rule, and per coedge in
/// order the bits of its edge's shared samples. A coedge whose edge has no
/// samples (the tessellator refuses such a face) keys as absent.
pub(in crate::watertight_tessellation) fn key(
    face: &FaceRecord,
    samples: &HashMap<u64, EdgeSamples>,
    chord_tolerance: f64,
) -> [u64; 2] {
    let mut lane = TwoLane::new();
    hash_face(&mut lane, face);
    lane.f64(chord_tolerance);
    lane.bool(super::triangulate::ruling_seed_enabled());
    for loop_record in &face.loops {
        for coedge in &loop_record.coedges {
            match samples.get(&coedge.edge_id) {
                None => lane.bool(false),
                Some(edge) => {
                    lane.bool(true);
                    lane.floats(&edge.fractions);
                    lane.usize(edge.positions.len());
                    for point in &edge.positions {
                        lane.f64(point.x);
                        lane.f64(point.y);
                        lane.f64(point.z);
                    }
                    lane.f64(edge.worst_sag);
                }
            }
        }
    }
    lane.finish()
}

/// The stored fragment under `key`, restamped with `face_id`.
pub(in crate::watertight_tessellation) fn lookup(key: [u64; 2], face_id: u32) -> Option<Mesh> {
    let stored = MEMO.with(|memo| memo.borrow().get(&key).cloned());
    match stored {
        Some(fragment) => {
            HITS.with(|cell| cell.set(cell.get() + 1));
            let mut mesh = Mesh::clone(&fragment);
            mesh.face_ids.fill(face_id);
            Some(mesh)
        }
        None => {
            MISSES.with(|cell| cell.set(cell.get() + 1));
            None
        }
    }
}

/// Store a freshly meshed fragment. Past either budget the memo is cleared
/// whole first; a fragment larger than the triangle budget is not stored.
pub(in crate::watertight_tessellation) fn store(key: [u64; 2], fragment: &Mesh) {
    let triangles = fragment.indices.len() / 3;
    if triangles > TRIANGLE_BUDGET {
        return;
    }
    MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        let stored = STORED_TRIANGLES.with(|cell| cell.get());
        if memo.len() >= ENTRY_BUDGET || stored + triangles > TRIANGLE_BUDGET {
            memo.clear();
            STORED_TRIANGLES.with(|cell| cell.set(0));
        }
        if memo.insert(key, Rc::new(fragment.clone())).is_none() {
            STORED_TRIANGLES.with(|cell| cell.set(cell.get() + triangles));
        }
    });
}

/// Forget every fragment (the history cache's clear, and the test switch).
pub fn clear() {
    MEMO.with(|memo| memo.borrow_mut().clear());
    STORED_TRIANGLES.with(|cell| cell.set(0));
}

/// Hits and misses since the last `take_counts`, for the profile line.
pub(in crate::watertight_tessellation) fn take_counts() -> (u64, u64, usize, usize) {
    let hits = HITS.with(|cell| cell.replace(0));
    let misses = MISSES.with(|cell| cell.replace(0));
    let entries = MEMO.with(|memo| memo.borrow().len());
    let triangles = STORED_TRIANGLES.with(|cell| cell.get());
    (hits, misses, entries, triangles)
}
