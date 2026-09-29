//! The DOCUMENT TABS seam — the outcome type the dock reports model-tab clicks
//! through, plus the verifier global describing the open models.
//!
//! # There is no strip here any more
//!
//! The tabs are drawn by `egui_tiles` itself: every open model is a
//! [`crate::panels::dock::PaneKind::Document`] pane, and they all live in one
//! `Tabs` container whose tab bar IS the model switcher. An earlier version drew
//! a hand-rolled strip INSIDE the single viewport pane, which left the pane
//! wearing two stacked bars — the tiles tab bar saying "3D View" above our own
//! row of file names. Making the pane's own bar the document bar is both what a
//! docked editor should look like and less code.
//!
//! Exclusivity — the bar holds 3D views and nothing else — is enforced in
//! `dock.rs` instead of by construction, because `egui_tiles` has no hook to
//! refuse a drop into a container: see `evict_foreign_panes_from_document_group`
//! (nothing else may land in the group) and `is_tile_draggable` (no document may
//! leave it).

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::Documents;
use eframe::egui;

/// What the user did in the document tab bar this frame. At most one of each —
/// the shell acts on them after the dock's borrows are released.
#[derive(Default)]
pub struct TabsOutcome {
    /// Activate this document (a tab click, seen as egui_tiles' active tab
    /// disagreeing with [`Documents`]).
    pub activate: Option<usize>,
    /// Close this document (its `✕`). Routed through the file dialog so a dirty
    /// model is confirmed first.
    pub close: Option<usize>,
    /// Per-tab screen rects for the headed verifier, keyed `doctab:<index>`,
    /// collected in `DockBehavior::on_tab_button`.
    pub hits: Vec<(String, egui::Rect)>,
}

/// The `__brepDocuments` verifier global: the open tabs (title, raw name, dirty)
/// and which one is active.
#[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
pub fn state_json(docs: &Documents) -> String {
    let tabs: Vec<serde_json::Value> = docs
        .iter()
        .map(|doc| {
            serde_json::json!({
                "title": doc.title(),
                "name": doc.name(),
                "dirty": doc.dirty_marker(),
            })
        })
        .collect();
    serde_json::json!({ "active": docs.active_index(), "tabs": tabs }).to_string()
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "documents", prefix: "doctab:", meaning: "activate a document tab (doctab:i) or close it (doctab:i:close)", command: None },
    HitKeyDoc { panel: "documents", prefix: "pane:", meaning: "a side pane's tab in the dock, by its title in kebab case (pane:inspector); a click brings the pane to the front", command: Some("show_pane") },
];
