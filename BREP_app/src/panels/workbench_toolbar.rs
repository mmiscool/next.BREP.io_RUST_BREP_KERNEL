//! Workbench actions toolbar — a second top strip drawn UNDER the primary
//! toolbar, listing the ACTIVE WORKBENCH's creatable features as one square
//! icon button each, then (in a workbench that shows the Constraints panel —
//! Assembly, Wire harness, and All as its union) a group with one button per
//! assembly-constraint type, and (in a workbench that shows the PMI panel —
//! PMI, and All) a group with one button per PMI annotation type.
//!
//! It is a shortcut surface over creation, nothing more: a feature button does
//! exactly what picking that entry in the **Add new feature** palette does
//! (`HistoryPanel::add_feature_of_type`, so ACOMP still routes to the
//! component selector), a constraint button does what the context bar's
//! constraint offer does (`context_bar::add_constraint_from_selection`, seeding
//! `elements` from the current selection), and an annotation button does what
//! the context bar's annotation offer does (`context_bar::add_pmi_from_selection`,
//! seeding the type's reference fields from the selection and opening the new
//! annotation's dialog). Which features appear is the SAME workbench filter
//! the palette applies ([`workbench::includes_feature`]); the constraint and
//! annotation groups are gated on the SAME panel claims the dock and the
//! context bar use ([`workbench::panel_visible`]). Nothing here decides
//! anything on its own.
//!
//! Shown only while `settings.show_workbench_toolbar` is on (the Settings
//! "Show workbench actions toolbar" checkbox), and NEVER in a special mode: a
//! sketch adds no features (its own tools are in the primary toolbar row above,
//! as workbench buttons — see `workbench::sketch`), and reference selection
//! bypasses the dock the new feature's form would open in. Follows
//! the toolbar pattern: a small state struct owning only the per-frame `hits`
//! map (widget rects) the headed verifier reads, a `show(...)` the shell calls
//! right after the primary toolbar so the strip lands below it, and clicks
//! flow OUT through a returned [`WorkbenchToolbarOutcome`] — the shell
//! dispatches, because adding a feature touches panels this strip cannot
//! borrow.

use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::toolbar_button;
use crate::workbench;
use brep_render::engine_state::EngineState;
use brep_render::features;
use eframe::egui;
use serde_json::Value;
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

/// One drawn button: its stable id, the glyph on the square, and the tooltip.
struct ActionButton {
    id: String,
    glyph: String,
    tooltip: String,
}

/// The strip's own state: the per-frame hit-rects, plus the button lists,
/// which are derived from the kernel catalogues and so are rebuilt only when
/// the workbench changes rather than re-walked every frame.
#[derive(Default)]
pub struct WorkbenchToolbarPanel {
    hits: HashMap<String, egui::Rect>,
    /// The feature buttons for `cached_workbench` (the catalogue filtered by
    /// that workbench, in catalogue order).
    features: Vec<ActionButton>,
    cached_workbench: Option<String>,
    /// The constraint buttons — the constraint catalogue is static, so this is
    /// built once.
    constraints: Vec<ActionButton>,
    /// The annotation buttons — the PMI type table is static too.
    annotations: Vec<ActionButton>,
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
        let active = state.settings.workbench.clone();
        self.refresh_buttons(&active);
        let panels = workbench::ButtonState::of(state);
        let with_constraints =
            workbench::panel_visible(&active, workbench::assembly::CONSTRAINTS_PANEL_ID, &panels);
        let with_annotations =
            workbench::panel_visible(&active, workbench::pmi::PANEL_ID, &panels);
        if self.features.is_empty() && !with_constraints && !with_annotations {
            return outcome;
        }
        egui::containers::panel::Panel::top("brep-workbench-toolbar")
            .resizable(false)
            .show(ui, |ui| {
                if self.read_only {
                    ui.disable();
                }
                ui.add_space(2.0);
                toolbar_button::wrapped(ui, |ui| {
                    if !self.features.is_empty() {
                        caption(ui, "Features");
                        outcome.feature = Self::draw_group(ui, &self.features, &mut self.hits);
                    }
                    if with_constraints {
                        if !self.features.is_empty() {
                            ui.separator();
                        }
                        caption(ui, "Constraints");
                        outcome.constraint =
                            Self::draw_group(ui, &self.constraints, &mut self.hits);
                    }
                    if with_annotations {
                        if !self.features.is_empty() || with_constraints {
                            ui.separator();
                        }
                        caption(ui, "Annotations");
                        outcome.annotation =
                            Self::draw_group(ui, &self.annotations, &mut self.hits);
                    }
                });
                ui.add_space(2.0);
            });
        outcome
    }

    /// Rebuild the button lists when the workbench changed (or on first use).
    fn refresh_buttons(&mut self, active: &str) {
        if self.cached_workbench.as_deref() != Some(active) {
            self.features = feature_buttons(active);
            self.cached_workbench = Some(active.to_string());
        }
        if self.constraints.is_empty() {
            self.constraints = constraint_buttons();
        }
        if self.annotations.is_empty() {
            self.annotations = annotation_buttons();
        }
    }

    /// Draw one group of square buttons through the shared toolbar-button
    /// helper, publishing each hit-rect under `wbtb:<id>`; returns the clicked
    /// button's payload (the part of the id after the group prefix).
    fn draw_group(
        ui: &mut egui::Ui,
        buttons: &[ActionButton],
        hits: &mut HashMap<String, egui::Rect>,
    ) -> Option<String> {
        let mut clicked = None;
        for button in buttons {
            let resp = toolbar_button::button(ui, &button.glyph, &button.tooltip);
            hits.insert(format!("wbtb:{}", button.id), resp.rect);
            if resp.clicked() {
                clicked = button.id.split_once(':').map(|(_, payload)| payload.to_string());
            }
        }
        clicked
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

/// One button per catalogue feature the workbench INCLUDES, in catalogue
/// order — the same list, same filter, as the Add-feature palette. The glyph is
/// the feature's icon (the catalogued artwork paints it); a feature with no icon
/// falls back to its short name as text.
fn feature_buttons(active: &str) -> Vec<ActionButton> {
    let catalogue = features::feature_catalogue();
    let mut buttons = Vec::new();
    if let Some(list) = catalogue.get("features").and_then(Value::as_array) {
        for feature in list {
            let ty = feature.get("type").and_then(Value::as_str).unwrap_or("");
            if ty.is_empty() || !workbench::includes_feature(active, ty) {
                continue;
            }
            let name = feature
                .get("longName")
                .and_then(Value::as_str)
                .unwrap_or(ty)
                .to_string();
            let glyph = match features::feature_icon(ty) {
                Some(icon) => icon.to_string(),
                None => feature
                    .get("shortName")
                    .and_then(Value::as_str)
                    .unwrap_or(ty)
                    .to_string(),
            };
            buttons.push(ActionButton {
                id: format!("feature:{ty}"),
                glyph,
                tooltip: format!("Add {name}"),
            });
        }
    }
    buttons
}

/// The glyph is the type's icon (`ConstraintTypeDef::icon` — the picture the
/// context bar's offer and the viewport chip show), drawn as catalogued
/// artwork; the plain name rides in the tooltip.
fn constraint_buttons() -> Vec<ActionButton> {
    brep_render::brep_kernel::CONSTRAINT_TYPES
        .iter()
        .map(|def| ActionButton {
            id: format!("constraint:{}", def.type_id),
            glyph: def.icon.to_string(),
            tooltip: format!("Add {} constraint from the selection", def.label),
        })
        .collect()
}

/// One button per PMI annotation type, in the kernel table's order (the PMI
/// panel's `+` dropdown order). The glyph is the type's icon
/// (`PmiTypeDef::icon` — the picture the tree row, the viewport chip and the
/// context bar's offer show); the plain name rides in the tooltip.
fn annotation_buttons() -> Vec<ActionButton> {
    brep_render::brep_kernel::PMI_TYPES
        .iter()
        .map(|def| ActionButton {
            id: format!("annotation:{}", def.type_id),
            glyph: def.icon.to_string(),
            tooltip: format!("Add a {} to the active view, from the selection", def.label.to_lowercase()),
        })
        .collect()
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:feature:", meaning: "add a feature of that type (one button per feature the active workbench offers)", command: Some("feature_add") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:constraint:", meaning: "add an assembly constraint of that type, seeded from the current selection", command: Some("assembly_add_constraint") },
    HitKeyDoc { panel: "wbtoolbar", prefix: "wbtb:annotation:", meaning: "add a PMI annotation of that type to the active view, its references seeded from the current selection; its dialog opens (wbtb:annotation:<type>)", command: Some("pmi_add_annotation") },
];
