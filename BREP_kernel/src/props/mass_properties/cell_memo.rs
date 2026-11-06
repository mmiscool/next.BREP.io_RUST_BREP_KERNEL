//! Exact tensor-station reuse across changed trims on an unchanged carrier.
//! Store geometry, never a cell sum: callers retain every weighted addition
//! and its original order, including the current sign and volume reference.
use super::{hash_surface, TwoLane};
use crate::{NurbsSurface, Vec3};
use std::cell::RefCell;
use std::rc::Rc;
use rustc_hash::FxHashMap;

type Station = (Vec3, Vec3, Vec3);
#[derive(Default)]
struct Memo {
    blocks: FxHashMap<[u64; 2], Rc<Vec<Station>>>,
    stations: usize,
    hits: u64,
    misses: u64,
}
thread_local! { static MEMO: RefCell<Memo> = RefCell::new(Memo::default()); }
const STATION_BUDGET: usize = 262_144;

pub(super) fn identity(surface: &NurbsSurface) -> Option<[u64; 2]> {
    if !switch!("BREP_MASS_CELL_REUSE", false) { return None; }
    let mut key = TwoLane::new();
    hash_surface(&mut key, surface);
    Some(key.finish())
}

pub(crate) fn clear() {
    MEMO.with(|memo| *memo.borrow_mut() = Memo::default());
}

pub(super) fn each(
    surface: &NurbsSurface,
    identity: Option<[u64; 2]>,
    us: &[f64],
    vs: &[f64],
    mut visit: impl FnMut(usize, usize, Vec3, Vec3, Vec3) -> Result<(), String>,
) -> Result<(), String> {
    let Some(identity) = identity else {
        return surface.deriv1_tensor_each(us, vs, visit);
    };
    let mut key = TwoLane::new();
    key.u64(identity[0]); key.u64(identity[1]);
    // The actual station bits also distinguish rule changes and signed zero.
    key.floats(us); key.floats(vs);
    let key = key.finish();
    let cached = MEMO.with(|memo| {
        let mut memo = memo.borrow_mut();
        let block = memo.blocks.get(&key).cloned();
        if block.is_some() { memo.hits += 1; } else { memo.misses += 1; }
        block
    });
    let block = match cached {
        Some(block) => block,
        None => {
            let mut block = Vec::with_capacity(us.len() * vs.len());
            surface.deriv1_tensor_each(us, vs, |_, _, point, su, sv| {
                block.push((point, su, sv));
                Ok(())
            })?;
            let block = Rc::new(block);
            if block.len() <= STATION_BUDGET {
                MEMO.with(|memo| {
                    let mut memo = memo.borrow_mut();
                    if memo.stations + block.len() > STATION_BUDGET {
                        memo.blocks.clear(); memo.stations = 0;
                    }
                    memo.stations += block.len();
                    memo.blocks.insert(key, block.clone());
                });
            }
            block
        }
    };
    for (index, &(point, su, sv)) in block.iter().enumerate() {
        visit(index / vs.len(), index % vs.len(), point, su, sv)?;
    }
    Ok(())
}

pub(super) fn counts() -> (u64, u64) {
    MEMO.with(|memo| { let memo = memo.borrow(); (memo.hits, memo.misses) })
}
