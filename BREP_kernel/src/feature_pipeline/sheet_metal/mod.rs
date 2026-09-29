//! Exact-BREP sheet metal.
//!
//! Sheet metal is modeled as a parametric [`tree::SheetTree`] of flats + bends and
//! turned into exact-BREP geometry by a single fold-parametric [`evaluate`]: the
//! folded 3D part at `fold = 1.0`, and the flat pattern (the "unfold") at
//! `fold = 0.0`. Everything is built from existing kernel ops — no triangle mesh.
//!
//! Sheet-metal features author or mutate the tree; unfold evaluates it flat.

pub mod evaluate;
pub mod flat_pattern;
pub mod tree;

pub use evaluate::evaluate;
pub use tree::{Bend, Edge, Flat, Hole, HoleBend, HoleBendKind, SheetTree};

use std::cell::RefCell;
use std::collections::HashMap;

/// Evaluate and register a folded tree, retaining the evaluator's face and edge
/// names. Generic solid registration also restamps names and is unsuitable here.
pub(crate) fn register_folded(tree: SheetTree, name: &str) -> Result<super::AddedSolid, String> {
    use super::features::common::{collect_edge_names, collect_face_names};
    let solid = evaluate(&tree, 1.0)?;
    let face_names = collect_face_names(&solid);
    let edge_names = collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    put_tree(handle, tree);
    Ok(super::AddedSolid {
        handle,
        name: name.to_string(),
        face_names,
        edge_names,
        ..super::AddedSolid::default()
    })
}

thread_local! {
    /// Trees keyed by resident solid handle. Chained features store each updated
    /// tree under the new handle; history-cache eviction removes it with that handle.
    static SHEET_TREES: RefCell<HashMap<u32, SheetTree>> = RefCell::new(HashMap::new());
}

/// Attach `tree` to the resident solid `handle`.
pub fn put_tree(handle: u32, tree: SheetTree) {
    SHEET_TREES.with(|trees| {
        trees.borrow_mut().insert(handle, tree);
    });
}

/// The sheet-metal tree of the resident solid `handle`, if it carries one.
pub fn get_tree(handle: u32) -> Option<SheetTree> {
    SHEET_TREES.with(|trees| trees.borrow().get(&handle).cloned())
}

/// Remove the tree when its solid handle is freed. Cached handles retain theirs.
pub fn remove_tree(handle: u32) {
    SHEET_TREES.with(|trees| {
        trees.borrow_mut().remove(&handle);
    });
}

/// Drop every stored tree — the full-reset path (`clear_history_cache`, e.g. on a
/// part/document switch).
pub fn clear_trees() {
    SHEET_TREES.with(|trees| trees.borrow_mut().clear());
}

/// Whether the resident solid `handle` carries a sheet-metal tree — the
/// SheetTree-free predicate the export lane filters resident solids with (so the
/// native engine picks the flat-pattern target without touching `SheetTree`).
pub fn is_sheet_metal_handle(handle: u32) -> bool {
    SHEET_TREES.with(|trees| trees.borrow().contains_key(&handle))
}

/// Export the flat pattern of the sheet-metal body `handle` as DXF (R12 ASCII).
/// Runs the unfold TRANSIENTLY off the resident tree — no feature, no history
/// mutation. Errors if the handle carries no sheet-metal tree.
pub fn flat_pattern_dxf(handle: u32) -> Result<String, String> {
    let tree = get_tree(handle)
        .ok_or_else(|| "flat pattern: handle is not a sheet-metal body".to_string())?;
    Ok(flat_pattern::to_dxf(&flat_pattern::build(&tree)?))
}

/// Export the flat pattern of the sheet-metal body `handle` as SVG — the DXF
/// sibling of [`flat_pattern_dxf`].
pub fn flat_pattern_svg(handle: u32) -> Result<String, String> {
    let tree = get_tree(handle)
        .ok_or_else(|| "flat pattern: handle is not a sheet-metal body".to_string())?;
    Ok(flat_pattern::to_svg(&flat_pattern::build(&tree)?))
}
