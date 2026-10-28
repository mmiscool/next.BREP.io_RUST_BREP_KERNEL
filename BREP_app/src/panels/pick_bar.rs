//! The FOCUSED PICK bar. A form field's reference pick normally keeps its
//! dialog up, the field focused in its pane, while the user clicks in the view
//! (`EngineState::ref_select_in_form`). On a phone that dialog and the side
//! panels cover the view, so nothing can be picked: on a touch or narrow
//! window (`crate::viewport::touch::narrow`) the shell instead hides ALL of
//! its chrome — toolbar and ribbon, workbench strip, status bar, the docks and
//! the dialog in them, the action overlay — and draws the viewport alone, with
//! this bar over it: what is being picked, the running picks (each with an ✕),
//! the selection filter's one button, Finish and Cancel. Finish puts the picks
//! in the field and Cancel leaves it as it was; either brings the whole UI
//! back. Picking itself is unchanged — the same filter, the same per-field
//! validation, a list field taking picks until Finish and a single field
//! finishing on its first pick as it does on desktop.
//!
//! The bar CLAIMS the pick every frame it draws (`ref_select_claim`): the
//! shell finishes an in-form pick no form claimed, and the form is not drawn
//! while the bar is up.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// How the shell lays a frame out around a reference pick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PickLayout {
    /// No pick is running.
    Idle,
    /// A form field's pick with its dialog up (desktop).
    InForm,
    /// A form field's pick on a touch or narrow window: the dock is hidden
    /// and this bar is the picker.
    Focused,
    /// A card-based pick (a connection point's seat, a spline anchor's port):
    /// the dock is hidden and the mode bar's card is the picker.
    Modal,
}

impl PickLayout {
    pub fn of(active: bool, in_form: bool, narrow: bool) -> Self {
        match (active, in_form, narrow) {
            (false, _, _) => Self::Idle,
            (true, false, _) => Self::Modal,
            (true, true, false) => Self::InForm,
            (true, true, true) => Self::Focused,
        }
    }

    /// Whether the dock (side panels, the dialog among them) is drawn.
    pub fn shows_dock(self) -> bool {
        matches!(self, Self::Idle | Self::InForm)
    }
}

/// What a click on the bar asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Action {
    Finish,
    Cancel,
    Remove(usize),
}

#[derive(Default)]
pub struct PickBar {
    hits: HashMap<String, egui::Rect>,
    /// `(label, picks so far)` as last drawn; `None` while the bar is down.
    shown: Option<(String, usize)>,
}

impl PickBar {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the bar across the top of `view` (the viewport rect) and apply
    /// what it was asked for. Called by the shell after the viewport, every
    /// frame of a [`PickLayout::Focused`] pick.
    pub fn show(
        &mut self,
        ctx: &egui::Context,
        view: egui::Rect,
        state: &mut EngineState,
        filter: &mut crate::panels::selection::SelectionPanel,
    ) {
        self.hits.clear();
        self.shown = None;
        let Some(field) = state.ref_select_field() else {
            return;
        };
        state.ref_select_claim();
        let label = state.ref_select_label();
        let hint = if state.ref_select_is_sheet() {
            "Tap an anchor marked on the sheet to pick; drag to pan."
        } else if field.multiple {
            "Tap in the view to add picks; drag to orbit. Finish when done."
        } else {
            "Tap in the view to pick; drag to orbit."
        };
        let width = (view.width() - 16.0).max(120.0);
        let mut action = None;
        egui::Area::new(egui::Id::new("brep-pick-bar"))
            .order(egui::Order::Foreground)
            .fixed_pos(view.left_top() + egui::vec2(8.0, 8.0))
            .movable(false)
            .show(ctx, |ui| {
                let frame = egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.set_width(width - 16.0);
                    ui.horizontal_wrapped(|ui| {
                        ui.label(egui::RichText::new(format!("Select: {label}")).strong());
                        ui.label(egui::RichText::new(format!("{} picked", field.names.len())).weak());
                    });
                    ui.label(egui::RichText::new(hint).weak().small());
                    // Finish / Cancel come BEFORE the picks: a long list must
                    // never push the way out of the picker off the screen. The
                    // status bar's filter button rides here, the bar being
                    // hidden with the rest of the chrome.
                    ui.horizontal_wrapped(|ui| {
                        let finish = ui.button("Finish").on_hover_text("Put the picks in the field and return to the dialog");
                        self.hits.insert("finish".into(), finish.rect);
                        if finish.clicked() {
                            action = Some(Action::Finish);
                        }
                        let cancel = ui.button("Cancel").on_hover_text("Return to the dialog with the field unchanged");
                        self.hits.insert("cancel".into(), cancel.rect);
                        if cancel.clicked() {
                            action = Some(Action::Cancel);
                        }
                        filter.show_status_bar(ui, state);
                    });
                    if !field.names.is_empty() {
                        egui::ScrollArea::vertical().id_salt("pick-bar-picks").max_height(96.0).show(ui, |ui| {
                            ui.horizontal_wrapped(|ui| {
                                for (i, name) in field.names.iter().enumerate() {
                                    let chip = ui.add(egui::Button::new(format!("{name}  \u{2716}")).small())
                                        .on_hover_text("Remove this pick");
                                    self.hits.insert(format!("x{i}"), chip.rect);
                                    if chip.clicked() {
                                        action = Some(Action::Remove(i));
                                    }
                                }
                            });
                        });
                    }
                });
                self.hits.insert("bar".into(), frame.response.rect);
            });
        self.shown = Some((label, field.names.len()));
        match action {
            Some(Action::Finish) => state.finish_ref_select(),
            Some(Action::Cancel) => state.cancel_ref_select(),
            Some(Action::Remove(i)) => state.ref_select_remove(i),
            None => {}
        }
        if action.is_some() {
            ctx.request_repaint();
        }
    }

    /// For a frame the bar is not drawn: no rect of it may stay published.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
        self.shown = None;
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// The `__brepPickBar` verifier global: `{shown, label, count}`.
    pub fn state_json(&self) -> String {
        serde_json::json!({
            "shown": self.shown.is_some(),
            "label": self.shown.as_ref().map(|(label, _)| label),
            "count": self.shown.as_ref().map(|(_, count)| count),
        })
        .to_string()
    }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "pickbar", prefix: "bar", meaning: "the focused pick bar a touch or narrow window shows over the full-window view while a dialog's reference field picks (every other piece of chrome is hidden; the filter button rides in the bar)", command: None },
    HitKeyDoc { panel: "pickbar", prefix: "finish", meaning: "put the picks in the field and return to the dialog", command: None },
    HitKeyDoc { panel: "pickbar", prefix: "cancel", meaning: "return to the dialog with the field unchanged (Escape does the same)", command: None },
    HitKeyDoc { panel: "pickbar", prefix: "x", meaning: "remove the i-th running pick (xi)", command: None },
];

