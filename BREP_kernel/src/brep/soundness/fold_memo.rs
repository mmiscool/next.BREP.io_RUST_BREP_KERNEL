//! Exact-input reuse of negative/undecided fold probes. The sweep still runs:
//! its selected points and triangle count are part of this key, along with
//! the complete carrier and trim and every option. Thus another face changing
//! the welded mesh cannot accidentally reuse a verdict for different probes.
//! Positive folds are read again so their current face ID/name remain current.
//! Bounded per thread, cleared on document switch; no new geometric shortcut.
use super::{confirm_fold, fold_grid, FoldVerdict, SelfIntersectionOptions};
use crate::{FaceRecord, Vec3};
use crate::mass_properties::{hash_face, TwoLane};
use std::cell::{Cell, RefCell};
use rustc_hash::FxHashMap;

thread_local! {
    static MEMO: RefCell<FxHashMap<[u64; 2], Vec<bool>>> = Default::default();
    static HITS: Cell<usize> = const { Cell::new(0) };
    static READS: Cell<usize> = const { Cell::new(0) };
}

pub(crate) fn clear() { MEMO.with(|memo| memo.borrow_mut().clear()); }

pub(super) fn counts() -> (usize, usize) { (HITS.with(Cell::get), READS.with(Cell::get)) }

fn key(face: &FaceRecord, triangles: usize, points: &[(f64, Vec3)], options: SelfIntersectionOptions) -> [u64; 2] {
    let mut hash = TwoLane::new();
    hash_face(&mut hash, face);
    hash.f64(crate::classification::lane_switches());
    // Containment distinguishes slit/seam uses of the SAME edge. Preserve
    // that equivalence relation while allowing a harmless ID renumbering.
    let mut edges = FxHashMap::default();
    for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
        let next = edges.len();
        hash.usize(*edges.entry(coedge.edge_id).or_insert(next));
    }
    hash.usize(triangles);
    hash.f64(options.chord_tolerance);
    hash.f64(options.straddle_band);
    hash.f64(options.min_crossing_length);
    hash.f64(options.weld);
    hash.usize(options.max_triangles);
    hash.usize(points.len());
    for (_, point) in points {
        hash.f64(point.x); hash.f64(point.y); hash.f64(point.z);
    }
    hash.finish()
}

pub(super) fn read(face: &FaceRecord, triangles: usize, points: &[(f64, Vec3)], options: SelfIntersectionOptions) -> Vec<FoldVerdict> {
    let enabled = std::env::var("BREP_FOLD_MEMO").as_deref() != Ok("0") && !super::fold_trace_enabled();
    let key = key(face, triangles, points, options);
    if enabled {
        if let Some(verdicts) = MEMO.with(|memo| memo.borrow().get(&key).cloned()) {
            HITS.with(|hits| hits.set(hits.get() + 1));
            return verdicts.into_iter().map(|rejected| if rejected { FoldVerdict::Rejected } else { FoldVerdict::Undecided }).collect();
        }
    }
    READS.with(|reads| reads.set(reads.get() + 1));
    let grid = fold_grid(&face.surface, triangles);
    let verdicts: Vec<_> = points.iter().map(|(_, point)| match &grid {
        Some(grid) => confirm_fold(face, grid, *point, options),
        None => FoldVerdict::Undecided,
    }).collect();
    if enabled && !verdicts.iter().any(|v| matches!(v, FoldVerdict::Confirmed(_))) {
        MEMO.with(|memo| {
            let mut memo = memo.borrow_mut();
            if memo.len() >= 4096 { memo.clear(); }
            memo.insert(key, verdicts.iter().map(|v| matches!(v, FoldVerdict::Rejected)).collect());
        });
    }
    verdicts
}

