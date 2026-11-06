//! Classic's separate workbench creation strip, sourced from the same Home
//! descriptors as Ribbon. Outcomes use the existing palette and context-bar
//! creation paths. The shell selects this strip only for Classic, and supplies
//! Ribbon's Home hit rectangles here for the existing automation export.

use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::toolbar_button;
use crate::workbench;
use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// What one frame of the strip produced for the shell to act on.
#[derive(Default)]
pub struct WorkbenchToolbarOutcome {
    /// A feature button click — the feature TYPE CODE to add (`"E"`, `"ACOMP"`).
    pub feature: Option<String>,
    /// A constraint button click — the constraint type id to add (`"parallel"`).
    pub constraint: Option<String>,
    /// An annotation button click — the PMI annotation type to add (`"linear"`).
    pub annotation: Option<String>,
}

/// The strip's own state: the per-frame hit-rects, plus the button lists,
/// which are derived from the kernel catalogues and so are rebuilt only when
/// the workbench changes rather than re-walked every frame.
#[derive(Default)]
pub struct WorkbenchToolbarPanel {
    hits: HashMap<String, egui::Rect>,
    /// The active document is read-only (a PLM revision not checked out, or
    /// released): the strip is drawn, disabled, since every button on it
    /// creates something. Set by the shell each frame.
    pub read_only: bool,
}

impl WorkbenchToolbarPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the strip is drawn this frame: the setting is on AND no special
    /// mode owns the shell (a sketch creates no features; a card-based
    /// reference pick hides the dock the new feature's form would open in). A
    /// form field's pick keeps the strip on screen, its buttons disabled
    /// (`offered_home_commands`), so the layout holds still while picking.
    pub fn visible(state: &EngineState) -> bool {
        state.settings.show_workbench_toolbar && !state.any_sketch_mode() && !state.ref_select_modal()
    }

    /// Draw the strip as a top panel — called right AFTER the primary toolbar
    /// so egui stacks it directly underneath. Draws nothing (and publishes no
    /// hit-rects) when [`Self::visible`] is false, or when the active
    /// workbench offers nothing to put on it (the placeholder workbenches).
    pub fn show(&mut self, ui: &mut egui::Ui, state: &EngineState) -> WorkbenchToolbarOutcome {
        self.hits.clear();
        let mut outcome = WorkbenchToolbarOutcome::default();
        if !Self::visible(state) {
            return outcome;
        }
        let commands = workbench::offered_home_commands(state);
        let groups = workbench::command_groups(&commands, "Home");
        if groups.is_empty() {
            return outcome;
        }
        egui::containers::panel::Panel::top("brep-workbench-toolbar")
            .resizable(false)
            .show(ui, |ui| {
                ui.add_space(2.0);
                toolbar_button::wrapped(ui, |ui| {
                    for (index, (name, members)) in groups.iter().enumerate() {
                        // A group that will not fit the rest of the line starts
                        // the next one, caption and all: the strip wrapped one
                        // button at a time, leaving "Sweep" and two of its
                        // buttons on one line and the third alone on the next
                        // (ui-polish-2026-10-28 row 15). A group wider than a
                        // whole line still wraps inside itself.
                        let starts_line = index > 0 && group_does_not_fit(ui, name, members.len());
                        if starts_line {
                            ui.end_row();
                        } else if index > 0 {
                            ui.separator();
                        }
                        caption(ui, name);
                        for command in members {
                            let mut command = (*command).clone();
                            if self.read_only {
                                command.disabled_reason = Some("Document is read-only".into());
                            }
                            if let Some(target) = super::ribbon::draw_command(
                                ui,
                                &command,
                                true,
                                false,
                                &mut self.hits,
                            ) {
                                match target {
                                    workbench::CommandTarget::Feature(id) => {
                                        outcome.feature = Some(id)
                                    }
                                    workbench::CommandTarget::Constraint(id) => {
                                        outcome.constraint = Some(id)
                                    }
                                    workbench::CommandTarget::Annotation(id) => {
                                        outcome.annotation = Some(id)
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                });
                ui.add_space(2.0);
            });
        outcome
    }

    pub fn set_ribbon_hits(&mut self, hits: HashMap<String, egui::Rect>) {
        self.hits = hits;
    }

    /// The published widget hit-rects (egui points) for the headed verifier —
    /// `wbtb:feature:<type>` / `wbtb:constraint:<type>` /
    /// `wbtb:annotation:<type>`; empty while hidden.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}

/// Whether a group (its caption and `buttons` square buttons, after a
/// separator) is wider than what is left of the current line, on a line that
/// already holds something.
fn group_does_not_fit(ui: &egui::Ui, name: &str, buttons: usize) -> bool {
    let font = egui::FontId::proportional(super::ribbon::CHROME_CAPTION_SIZE);
    let caption_w = ui.ctx().fonts_mut(|f| f.layout_no_wrap(name.to_owned(), font, egui::Color32::PLACEHOLDER).size().x);
    let gap = ui.spacing().item_spacing.x;
    let button = crate::viewport::touch::target_size(
        ui.ctx(),
        egui::vec2(super::toolbar_button::TOOLBAR_BTN, super::toolbar_button::TOOLBAR_BTN),
    )
    .x;
    let separator = ui.spacing().item_spacing.x * 2.0 + 1.0;
    let needed = separator + caption_w + gap + buttons as f32 * (button + gap);
    let remaining = ui.max_rect().right().min(ui.clip_rect().right()) - ui.cursor().left();
    needed > remaining && needed <= ui.max_rect().width() && ui.cursor().left() > ui.max_rect().left() + 0.5
}

/// A small, dim group caption ahead of its buttons.
fn caption(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).weak().size(super::ribbon::CHROME_CAPTION_SIZE));
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:feature:", meaning: "add a feature of that type (one button per feature the active workbench offers)", command: Some("feature_add") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:constraint:", meaning: "add an assembly constraint of that type, seeded from the current selection", command: Some("assembly_add_constraint") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:annotation:", meaning: "add a PMI annotation of that type to the active view, its references seeded from the current selection; its dialog opens (wbtb:annotation:<type>)", command: Some("pmi_add_annotation") },
];
