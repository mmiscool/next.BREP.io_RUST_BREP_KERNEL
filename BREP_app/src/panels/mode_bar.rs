//! mode_bar — the special-mode EXIT controls, always pinned to the TOP-RIGHT
//! corner of the screen. Every special mode (reference-selection, sketch mode,
//! and any future mode) surfaces its Finish / Cancel here so the exit is in a
//! single, predictable place — the pattern the user asked for.
//!
//! It draws INTO a caller-owned `ui` (the shell owns the top-right `Area` and
//! stacks the context-action rail below it), and owns no model state.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// Who this card is when it drives the viewport's dialog-row hover
/// (`EngineState::hover_entity_by_name`) — the owner tag that keeps its
/// highlight independent of the form panes' and the Scene tree's.
const DIALOG_HOVER_OWNER: &str = "refsel";

/// The mode-exit card's transient UI state (the model lives in the engine).
#[derive(Default)]
pub struct ModeBar {
    /// Per-frame widget rects, published for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

impl ModeBar {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the active special mode's exit controls as a card. No-op (draws
    /// nothing) in the normal modeling environment. Called from the shell inside
    /// the shared top-right overlay `Area`.
    pub fn card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();

        if state.ref_select_active() {
            self.reference_card(ui, state);
        } else if state.sketch_mode() {
            self.sketch_card(ui, state);
        }
    }

    /// Reference-selection: the running picked-name list (each with an ✕ to drop
    /// it) + Finish / Cancel. Picking itself happens by clicking in the viewport;
    /// this card is the whole picker UI now (the side panel is hidden).
    fn reference_card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        egui::Frame::popup(ui.style()).show(ui, |ui| {
            ui.set_max_width(260.0);
            ui.label(egui::RichText::new("Select reference").strong());
            ui.label(egui::RichText::new(state.ref_select_prompt()).weak().small());
            // The same card for a pick on a drawing sheet, where the picks are
            // the anchors marked on the paper and a drag pans rather than orbits.
            let hint = if state.ref_select_is_sheet() {
                "Click an anchor marked on the sheet to pick; drag to pan."
            } else {
                "Click in the viewport to pick; drag to orbit."
            };
            ui.label(egui::RichText::new(hint).weak().small());
            ui.separator();
            // Finish / Cancel sit ABOVE the picks: a long pick list below them
            // must never push the way out of the picker off the screen.
            ui.horizontal(|ui| {
                let finish = ui.button("Finish");
                self.hits.insert("refsel:finish".into(), finish.rect);
                if finish.clicked() {
                    state.finish_ref_select();
                }
                let cancel = ui.button("Cancel");
                self.hits.insert("refsel:cancel".into(), cancel.rect);
                if cancel.clicked() {
                    state.cancel_ref_select();
                }
            });
            ui.separator();

            let names = state.ref_select_names();
            // The entity a hovered picked-name line names. The picker rolls the
            // model to BEFORE the edited feature, so these rows name geometry that
            // IS in the scene — hovering one lights the very edge/face that pick
            // grabbed.
            let mut hover: Option<String> = None;
            if names.is_empty() {
                ui.label(egui::RichText::new("(nothing picked yet)").weak());
            }
            let mut remove = None;
            // The picks scroll inside what is left of the screen below this
            // card's top-right anchor, so the card never runs off the bottom.
            let list_height =
                (ui.ctx().content_rect().bottom() - ui.cursor().top() - 24.0).max(80.0);
            egui::ScrollArea::vertical()
                .id_salt("refsel-picks")
                .max_height(list_height)
                .show(ui, |ui| {
                    for (i, name) in names.iter().enumerate() {
                        // Same order as the feature form's reference lines: the ✕ is
                        // allocated FIRST, from the RIGHT edge, and the name truncates
                        // into the remainder. A picked entity's name is unbounded, and
                        // name-then-button would push the ✕ past the card's 260 pt cap
                        // and off the screen corner this card is pinned to.
                        ui.horizontal(|ui| {
                            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                                let x = {
                                    let b = crate::icon_text::icon_button(ui, "\u{2716}").small();
                                    ui.add(b)
                                };
                                self.hits.insert(format!("refsel:x{i}"), x.rect);
                                if x.clicked() {
                                    remove = Some(i);
                                }
                                // An elided `Label` tooltips its own full text, so a
                                // truncated name still reads in full on hover.
                                ui.with_layout(egui::Layout::left_to_right(egui::Align::Center), |ui| {
                                    let line = ui.add(
                                        egui::Label::new(format!("\u{2022} {name}")).truncate(),
                                    );
                                    self.hits.insert(format!("refsel:line{i}"), line.rect);
                                    if line.hovered() {
                                        hover = Some(name.clone());
                                    }
                                });
                            });
                        });
                    }
                });
            if let Some(i) = remove {
                state.ref_select_remove(i);
            }
            // Light the hovered pick in the 3D view (the card is drawn over the
            // viewport, so the pointer is off it and the engine's one-frame yield
            // flag is what keeps the highlight alive). `dialog_hover_end` only ends
            // a hover THIS card set.
            let hover_changed = match &hover {
                Some(name) => state.hover_entity_by_name(DIALOG_HOVER_OWNER, name),
                None => state.dialog_hover_end(DIALOG_HOVER_OWNER),
            };
            if hover_changed {
                // This card is an overlay drawn in the same frame as the viewport
                // tile, which may have drawn (and consumed `state.dirty`) FIRST —
                // without this the new highlight waits for the next input event.
                ui.ctx().request_repaint();
            }

        });
    }

    /// Sketch mode: the sketch title + Finish (commit) / Cancel (discard). The
    /// drawing tools live in the main toolbar's workbench row, as shared
    /// workbench buttons (`workbench::sketch`); the selection-driven constraint
    /// actions live in the shared context rail below this card. This card is the
    /// EXIT, not a tool — which is why it stayed here when the tools moved.
    fn sketch_card(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        egui::Frame::popup(ui.style()).show(ui, |ui| {
            let id = state.sketch_edit_feature_id().unwrap_or("").to_string();
            ui.label(egui::RichText::new(format!("Sketch: {id}")).strong());
            ui.horizontal(|ui| {
                let finish = ui.button("Finish").on_hover_text("Commit the sketch");
                self.hits.insert("sketch:finish".into(), finish.rect);
                if finish.clicked() {
                    let _ = state.exit_sketch_mode(true);
                }
                let cancel = ui
                    .button("Cancel")
                    .on_hover_text("Discard changes (deletes a new sketch)");
                self.hits.insert("sketch:cancel".into(), cancel.rect);
                if cancel.clicked() {
                    let _ = state.exit_sketch_mode(false);
                }
            });
        });
    }

    /// Published widget hit-rects for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "modebar", prefix: "refsel:finish", meaning: "finish reference selection", command: None },
    HitKeyDoc { panel: "modebar", prefix: "refsel:cancel", meaning: "cancel reference selection", command: None },
    HitKeyDoc { panel: "modebar", prefix: "refsel:x", meaning: "remove the i-th picked reference (refsel:xi)", command: None },
    HitKeyDoc { panel: "modebar", prefix: "refsel:line", meaning: "the i-th picked reference's name line — hovering it lights that entity (refsel:linei)", command: None },
    HitKeyDoc { panel: "modebar", prefix: "sketch:finish", meaning: "commit the sketch", command: None },
    HitKeyDoc { panel: "modebar", prefix: "sketch:cancel", meaning: "abandon the sketch", command: None },
];
