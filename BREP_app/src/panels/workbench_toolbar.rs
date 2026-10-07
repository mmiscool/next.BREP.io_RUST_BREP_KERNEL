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
    /// A constraint button click — the constraint type id to add (`"fixed"`).
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
    /// mode owns the shell (a sketch creates no features; reference selection
    /// hides the dock the new feature's form would open in).
    pub fn visible(state: &EngineState) -> bool {
        state.settings.show_workbench_toolbar && !state.sketch_mode() && !state.ref_select_active()
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
                        if index > 0 {
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

/// A small, dim group caption ahead of its buttons.
fn caption(ui: &mut egui::Ui, text: &str) {
    ui.label(egui::RichText::new(text).weak().small());
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:feature:", meaning: "add a feature of that type (one button per feature the active workbench offers)", command: Some("feature_add") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:constraint:", meaning: "add an assembly constraint of that type, seeded from the current selection", command: Some("assembly_add_constraint") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:annotation:", meaning: "add a PMI annotation of that type to the active view, its references seeded from the current selection; its dialog opens (wbtb:annotation:<type>)", command: Some("pmi_add_annotation") },
];
