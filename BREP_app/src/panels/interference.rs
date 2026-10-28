//! Interference results window — the floating window behind the Assembly
//! workbench's `∩` toolbar button.
//!
//! Follows the pinned Info-window idiom: a movable + resizable
//! [`egui::Window`] drawn at ctx level, owning its own state (the last
//! [`InterferenceReport`]). The check itself lives in the ENGINE
//! ([`EngineState::interference_check`] — main-side, non-destructive,
//! bbox-prefiltered); this panel only renders the report:
//!
//! * a Clearance field (mm, 0 = off): the near-miss threshold for the run;
//! * a row per INTERFERING pair — `ACOMP1 × ACOMP3 — 12.4 mm³` — clicking it
//!   selects/highlights BOTH components (emphasis over their member solids)
//!   and frames them; a hidden participant is noted on its row (it still
//!   participated — interference is a physical question);
//! * a row per NEAR MISS — `ACOMP1 × ACOMP2 — 1.5 mm gap` — same click;
//! * the interference volumes (red) and near-miss gaps (amber) drawn in the
//!   viewport while the window is open and the report is current;
//! * a green all-clear line when nothing interferes (a PASS state, shown
//!   positively);
//! * explicit `unverified` (boolean refused) and `skipped` (budget / no
//!   geometry) sections — nothing is ever dropped silently;
//! * a Re-run button.
//!
//! Colors come from the ONE assembly status map
//! ([`brep_render::assembly_status`]) and volumes from the ONE measurement
//! formatter (`super::info_windows::num`) — no re-rolled styling.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::assembly_status;
use brep_render::engine_state::{ClearancePair, EngineState, InterferencePair, InterferenceReport};
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The row/line label of one interfering pair: `A × B — 12.4 mm³`, with a
/// `(hidden: …)` note when a participant is currently invisible.
fn pair_label(pair: &InterferencePair) -> String {
    let mut label = format!(
        "{} \u{00d7} {} \u{2014} {} mm\u{00b3}",
        pair.a,
        pair.b,
        super::info_windows::num(pair.volume)
    );
    let hidden: Vec<&str> = [
        pair.a_hidden.then_some(pair.a.as_str()),
        pair.b_hidden.then_some(pair.b.as_str()),
    ]
    .into_iter()
    .flatten()
    .collect();
    if !hidden.is_empty() {
        label.push_str(&format!(" (hidden: {})", hidden.join(", ")));
    }
    label
}

/// The near-miss row label: `A × B — 1.5 mm gap`, plus the hidden note.
fn near_label(pair: &ClearancePair) -> String {
    let mut label = format!(
        "{} \u{00d7} {} \u{2014} {} mm gap",
        pair.a,
        pair.b,
        super::info_windows::num(pair.distance)
    );
    let hidden: Vec<&str> = [
        pair.a_hidden.then_some(pair.a.as_str()),
        pair.b_hidden.then_some(pair.b.as_str()),
    ]
    .into_iter()
    .flatten()
    .collect();
    if !hidden.is_empty() {
        label.push_str(&format!(" (hidden: {})", hidden.join(", ")));
    }
    label
}

/// An [`egui::Color32`] off the shared status palette.
fn status_color(status: &str) -> egui::Color32 {
    let [r, g, b] = assembly_status::status_color_rgb(status);
    egui::Color32::from_rgb(r, g, b)
}

/// The shell-owned interference results window. Opened (and run) by the
/// Assembly workbench's toolbar button; re-run from its own button. Owns the
/// last report — the engine holds no window state.
#[derive(Default)]
pub struct InterferenceWindow {
    open: bool,
    report: Option<InterferenceReport>,
    /// The near-miss threshold (mm) the next run uses; 0 = off.
    clearance: f64,
    /// The model moved since the report was taken: its overlay is cleared
    /// and the window asks for a re-run.
    stale: bool,
    /// Per-frame interactive-widget screen rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

impl InterferenceWindow {
    pub fn new() -> Self {
        Self::default()
    }

    /// The toolbar entry point: run the check NOW and show the window.
    pub fn open_and_run(&mut self, state: &mut EngineState) {
        self.run(state);
        self.open = true;
    }

    /// Run the check at the window's clearance (draws the overlay).
    fn run(&mut self, state: &mut EngineState) {
        self.report = Some(state.interference_check_with(self.clearance));
        self.stale = false;
    }

    /// Select both participants of a result row and frame them.
    fn focus_pair(state: &mut EngineState, a: &str, b: &str) {
        let ids = [a.to_string(), b.to_string()];
        state.select_components(&ids);
        state.zoom_to_components(&ids);
    }

    /// Draw the window (if open) at ctx level, like the Info windows.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();
        if !self.open {
            return;
        }
        // A rebuild since the run: the drawn volumes no longer match the
        // model — take them down and ask for a re-run.
        if !self.stale
            && self
                .report
                .as_ref()
                .is_some_and(|report| report.generation != state.applied_generation())
        {
            self.stale = true;
            state.clear_interference_overlay();
        }
        let mut open = true;
        egui::Window::new("Interference")
            .id(egui::Id::new("brep-interference-window"))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            .default_size([360.0, 320.0])
            .default_pos(super::below_chrome(ctx, 860.0, 0.0))
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui, state));
            });
        if !open {
            self.close(state);
        }
    }

    /// Shut the window and take its overlay down (the title-bar ✕).
    pub fn close(&mut self, state: &mut EngineState) {
        self.open = false;
        state.clear_interference_overlay();
    }

    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        // Header: the summary + Re-run.
        ui.horizontal(|ui| {
            let rerun = ui.button("Re-run");
            self.hits.insert("interference:rerun".into(), rerun.rect);
            if rerun.clicked() {
                self.run(state);
            }
            ui.label("Clearance");
            let clearance = ui.add(
                egui::DragValue::new(&mut self.clearance)
                    .range(0.0..=1.0e4)
                    .speed(0.1)
                    .suffix(" mm"),
            );
            self.hits.insert("interference:clearance".into(), clearance.rect);
            if let Some(report) = &self.report {
                ui.weak(format!(
                    "{} components \u{00b7} {} pairs \u{00b7} {} boolean{}",
                    report.component_count,
                    report.pair_total,
                    report.booleans_run,
                    if report.booleans_run == 1 { "" } else { "s" }
                ));
            }
        });
        ui.separator();

        let Some(report) = self.report.clone() else {
            ui.weak("Run the check from the toolbar.");
            return;
        };
        if report.component_count < 2 {
            ui.weak("Needs at least two components.");
            return;
        }
        if self.stale {
            ui.colored_label(
                status_color("unsupported-selection"),
                "The model changed \u{2014} Re-run to refresh.",
            );
        }

        if report.pairs.is_empty() {
            // The PASS state, shown positively (green — the shared palette's
            // `satisfied`). Refusals/skips below still temper it.
            ui.colored_label(
                status_color("satisfied"),
                format!(
                    "\u{2713} No interference \u{2014} {} pair{} checked",
                    report.pair_total,
                    if report.pair_total == 1 { "" } else { "s" }
                ),
            );
        } else {
            for pair in &report.pairs {
                // A clickable row: selecting it highlights BOTH participants
                // (emphasis over the union of their member solids).
                let row = ui.selectable_label(
                    false,
                    egui::RichText::new(pair_label(pair)).color(status_color("error")),
                );
                self.hits
                    .insert(format!("interference:pair:{}x{}", pair.a, pair.b), row.rect);
                if row.clicked() {
                    Self::focus_pair(state, &pair.a, &pair.b);
                }
            }
        }

        // Near misses (a clearance was set): amber rows, smallest gap first.
        if report.clearance > 0.0 {
            ui.add_space(4.0);
            if report.near_misses.is_empty() {
                ui.weak(format!(
                    "No gaps under {} mm",
                    super::info_windows::num(report.clearance)
                ));
            } else {
                ui.label(format!(
                    "Clearance under {} mm (gap measured on the display mesh):",
                    super::info_windows::num(report.clearance)
                ));
            }
            for pair in &report.near_misses {
                let row = ui.selectable_label(
                    false,
                    egui::RichText::new(near_label(pair))
                        .color(status_color("unsupported-selection")),
                );
                self.hits
                    .insert(format!("interference:near:{}x{}", pair.a, pair.b), row.rect);
                if row.clicked() {
                    Self::focus_pair(state, &pair.a, &pair.b);
                }
            }
        }

        // Never-silent sections: boolean refusals and skipped work.
        if !report.unverified.is_empty() {
            ui.add_space(4.0);
            ui.label("Not verified (boolean refused):");
            for line in &report.unverified {
                ui.colored_label(status_color("unsupported-selection"), line);
            }
        }
        if !report.skipped.is_empty() {
            ui.add_space(4.0);
            ui.label("Not checked:");
            for line in &report.skipped {
                ui.weak(line);
            }
        }
    }

    /// The window's logical state for the headed verifier
    /// (`__brepInterference`): `{open, report: {…} | null}`.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn state_json(&self) -> String {
        self.state_value().to_string()
    }

    /// [`Self::state_json`] plus `overlayShown`: whether `state` (the active
    /// document's engine) currently draws the interference overlay — the
    /// published blob, so a script can see the overlay follow the window.
    pub fn state_json_in(&self, state: &EngineState) -> String {
        let mut value = self.state_value();
        value["overlayShown"] = Value::Bool(
            state
                .widgets
                .overlay_group_names()
                .contains(&brep_render::engine_state::INTERFERENCE_OVERLAY_GROUP),
        );
        value.to_string()
    }

    fn state_value(&self) -> Value {
        let report = self.report.as_ref().map(|report| {
            serde_json::json!({
                "componentCount": report.component_count,
                "pairTotal": report.pair_total,
                "booleansRun": report.booleans_run,
                "pairs": report
                    .pairs
                    .iter()
                    .map(|pair| {
                        serde_json::json!({
                            "a": pair.a,
                            "b": pair.b,
                            "volume": pair.volume,
                            "aHidden": pair.a_hidden,
                            "bHidden": pair.b_hidden,
                            "label": pair_label(pair),
                        })
                    })
                    .collect::<Vec<_>>(),
                "clearance": report.clearance,
                "nearMisses": report
                    .near_misses
                    .iter()
                    .map(|pair| {
                        serde_json::json!({
                            "a": pair.a,
                            "b": pair.b,
                            "distance": pair.distance,
                            "witness": pair.witness,
                            "aHidden": pair.a_hidden,
                            "bHidden": pair.b_hidden,
                            "label": near_label(pair),
                        })
                    })
                    .collect::<Vec<_>>(),
                "skipped": report.skipped,
                "unverified": report.unverified,
            })
        });
        serde_json::json!({
            "open": self.open,
            "clearance": self.clearance,
            "stale": self.stale,
            "report": report.unwrap_or(Value::Null),
        })
    }

    /// Per-frame widget rects (`interference:rerun`, `interference:clearance`,
    /// `interference:pair:AxB`, `interference:near:AxB`)
    /// for the headed verifier.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "interference", prefix: "interference:rerun", meaning: "rerun the interference check", command: Some("interference_check") },
    HitKeyDoc { panel: "interference", prefix: "interference:", meaning: "an interference result row", command: None },
];
