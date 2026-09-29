//! Selection panel — the **selection filter** (which entity kinds a viewport
//! click may pick), rendered as a single horizontal row in the shell's bottom
//! STATUS BAR (not a side-panel section). Follows the panel pattern: a small
//! state struct + a `show_status_bar(&mut self, ui, state)` the bottom bar calls
//! once per frame in modeling mode; `EngineState` stays the single brain (it owns
//! the filter + the selection), borrowed in.
//!
//! * **Filter** — a leading "All" tristate CHECKBOX followed by a real CHECKBOX
//!   per kind (COMPONENT / SOLID / FACE / EDGE / VERTEX / PLANE; COMPONENT =
//!   promote a pick on assembly-component geometry to the whole component;
//!   PLANE = the construction datum/plane cards, pickable like any face). The
//!   engine honors the filter in
//!   `select_top_at` (via the planes-aware `pick_top_at` with the enabled
//!   kinds), so a plain click grabs only an
//!   allowed kind — a FACE-only filter selects a face, a SOLID-only filter the
//!   owning solid, a PLANE-only filter the construction plane under the cursor. Defaults to ALL kinds enabled (everything under the cursor is
//!   pickable, highest-priority kind wins). Reference-selection mode temporarily
//!   constrains it to the active field's allowed kinds — while that picker is
//!   active the row is LOCKED (greyed + non-interactive) so a click can't
//!   overwrite the constraint. The model state is the engine's `selection_filter`;
//!   this panel just reads/writes it.
//!
//! The quick actions on the current selection (Clear / Hide / Edit-owning-feature
//! + the feature-from-selection actions) live in the dedicated
//! [`crate::panels::context_bar`] (the engine-native successor to the old app's
//! floating selection action bar).
//!
//! The panel owns only the per-frame `hits` map (widget screen rects) the headed
//! verifier reads to drive real clicks, exactly like the toolbar/history panels.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::{EngineState, SelectionFilter};
use crate::icon_text::IconTextUi as _;
use eframe::egui;
use std::collections::HashMap;

/// The pickable kinds, in the order the filter row draws them: the geometry
/// kinds coarsest-first, then the CONSTRUCTION kind last. SKETCH sits beside
/// SOLID because a committed sketch is drawn as a sheet solid and picks as one;
/// the two lanes split that single pick kind so a sketch can be made pickable
/// (or not) independently of real bodies. It governs WHOLE sketches — a sketch's
/// face and its drawn segments stay under Face and Edge.
/// `(key, label)`: the `key` is the engine kind name + the `hits` map key
/// suffix; the `label` is the checkbox caption. COMPONENT is the promotion
/// kind: on, a pick landing on assembly-component geometry selects the WHOLE
/// component; off, the click reaches the sub-entity kinds. PLANE is the
/// construction kind: the drawn datum/plane cards, pickable like any face (they
/// rank right after faces in the pick list, so a plane under geometry is
/// reachable through the pick-list popup).
const KINDS: [(&str, &str); 7] = [
    ("COMPONENT", "Component"),
    ("SOLID", "Solid"),
    ("SKETCH", "Sketch"),
    ("FACE", "Face"),
    ("EDGE", "Edge"),
    ("VERTEX", "Vertex"),
    ("PLANE", "Plane"),
];

/// The selection panel's own state: the per-frame map of egui widget screen
/// rects, published to JS for the headed verifier to drive real clicks. Rebuilt
/// each frame (there is no DOM — egui is drawn on the canvas).
#[derive(Default)]
pub struct SelectionPanel {
    hits: HashMap<String, egui::Rect>,
}

impl SelectionPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the selection filter as a single horizontal row into the shell's
    /// bottom STATUS BAR: a leading "All" tristate checkbox + one CHECKBOX per
    /// pickable kind (all reflecting the LIVE engine `selection_filter`). Rebuilds
    /// `hits` as it draws. Called by the bottom bar in modeling mode.
    pub fn show_status_bar(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();
        ui.horizontal_wrapped(|ui| {
            // While the reference-selection picker is active the engine
            // temporarily CONSTRAINS the filter to the active field's allowed
            // kinds (see `begin_ref_select`). Letting a bottom-bar click edit it
            // then would clobber that constraint, so LOCK the row: it still shows
            // the constrained kinds (greyed) but is non-interactive until the
            // picker finishes/cancels and the filter is restored.
            let locked = state.ref_select_active();
            if locked {
                ui.icon_label(egui::RichText::new("\u{1f512} Reference selection — filter locked").weak())
                    .on_hover_text(
                        "The pick filter is set by the field being referenced. \
                         Finish or cancel the reference selection to change it.",
                    );
            } else {
                ui.label("Pickable:");
            }

            // `add_enabled_ui(false, …)` greys the widgets AND makes them
            // non-interactive, so no `.changed()`/`.clicked()` fires while locked
            // — the constrained filter cannot be overwritten. Rects are still laid
            // out and published so the verifier and layout stay consistent.
            ui.add_enabled_ui(!locked, |ui| {
                let mut filter = state.selection_filter();
                let before = filter;

                // Leading "All" TRISTATE CHECKBOX (mirrors the Scene tree's group
                // checkbox idiom): checked when EVERY kind is on, the indeterminate
                // dash when some-but-not-all are on, unchecked when none are.
                // Toggling it applies the toggle-all semantic — all-on → clear all;
                // partial or none → set all on — which is exactly the checkbox's
                // post-click value. Published under `filter:ALL` for the verifier.
                let all_on = KINDS.iter().all(|(k, _)| filter.get(k));
                let any_on = KINDS.iter().any(|(k, _)| filter.get(k));
                let mut all_checked = all_on;
                let resp = ui.add(
                    egui::Checkbox::new(&mut all_checked, "All").indeterminate(any_on && !all_on),
                );
                self.hits.insert("filter:ALL".into(), resp.rect);
                if resp.changed() {
                    let target = toggle_all_target(&filter);
                    for (kind, _) in KINDS {
                        filter.set(kind, target);
                    }
                }

                // One CHECKBOX per kind, reflecting the live filter. A `.changed()`
                // checkbox updates the working copy; it is written back once below.
                for (kind, label) in KINDS {
                    let mut on = filter.get(kind);
                    let resp = ui.checkbox(&mut on, label);
                    self.hits.insert(format!("filter:{kind}"), resp.rect);
                    if resp.changed() {
                        filter.set(kind, on);
                    }
                }

                if filter != before {
                    state.set_selection_filter(filter);
                }
            });
        });
    }

    /// Drop the published widget rects. The bottom bar calls this in sketch mode
    /// (when the filter row is NOT drawn) so the verifier never sees stale
    /// last-modeling-frame rects for widgets that are no longer on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// The published widget hit-rects (egui points) for the headed verifier —
    /// `filter:COMPONENT|SOLID|FACE|EDGE|VERTEX|PLANE` (the per-kind checkboxes)
    /// + `filter:ALL` (the leading "All" tristate checkbox).
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}

/// The "toggle all" target value: if EVERY pickable kind is currently on, the
/// "All" checkbox turns them all OFF; otherwise (some-but-not-all on, or none on)
/// it turns them all ON. Returns the value to write to every kind.
fn toggle_all_target(filter: &SelectionFilter) -> bool {
    !KINDS.iter().all(|(k, _)| filter.get(k))
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "selection", prefix: "filter:", meaning: "toggle which entity kinds a click may pick (filter:ALL, filter:kind)", command: None },
];
