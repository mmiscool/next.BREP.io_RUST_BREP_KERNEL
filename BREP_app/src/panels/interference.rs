//! Interference results window — the floating window behind the Assembly
//! workbench's `∩` toolbar button.
//!
//! Follows the pinned Info-window idiom: a movable + resizable
//! [`egui::Window`] drawn at ctx level, owning its own state (the last
//! [`InterferenceReport`]). The check itself lives in the ENGINE
//! ([`EngineState::interference_check`] — main-side, non-destructive,
//! bbox-prefiltered); this panel only renders the report:
//!
//! * a row per INTERFERING pair — `ACOMP1 × ACOMP3 — 12.4 mm³` — clicking it
//!   selects/highlights BOTH components (emphasis over their member solids);
//!   a hidden participant is noted on its row (it still participated —
//!   interference is a physical question);
//! * a green all-clear line when nothing interferes (a PASS state, shown
//!   positively);
//! * explicit `unverified` (boolean refused) and `skipped` (budget / no
//!   geometry) sections — nothing is ever dropped silently;
//! * a Re-run button.
//!
//! Colors come from the ONE assembly status map
//! ([`brep_render::assembly_status`]) and volumes from the ONE measurement
//! formatter ([`super::info_windows::num`]) — no re-rolled styling.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::assembly_status;
use brep_render::engine_state::{EngineState, InterferencePair, InterferenceReport};
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
    /// Per-frame interactive-widget screen rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

impl InterferenceWindow {
    pub fn new() -> Self {
        Self::default()
    }

    /// The toolbar entry point: run the check NOW and show the window.
    pub fn open_and_run(&mut self, state: &mut EngineState) {
        self.report = Some(state.interference_check());
        self.open = true;
    }

    /// Draw the window (if open) at ctx level, like the Info windows.
    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();
        if !self.open {
            return;
        }
        let mut open = true;
        egui::Window::new("Interference")
            .id(egui::Id::new("brep-interference-window"))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            .default_size([360.0, 320.0])
            .default_pos([860.0, 80.0])
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .auto_shrink([false, false])
                    .show(ui, |ui| self.body(ui, state));
            });
        self.open = open;
    }

    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        // Header: the summary + Re-run.
        ui.horizontal(|ui| {
            let rerun = ui.button("Re-run");
            self.hits.insert("interference:rerun".into(), rerun.rect);
            if rerun.clicked() {
                self.report = Some(state.interference_check());
            }
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
                    state.select_components(&[pair.a.clone(), pair.b.clone()]);
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
                "skipped": report.skipped,
                "unverified": report.unverified,
            })
        });
        serde_json::json!({
            "open": self.open,
            "report": report.unwrap_or(Value::Null),
        })
        .to_string()
    }

    /// Per-frame widget rects (`interference:rerun`, `interference:pair:AxB`)
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
