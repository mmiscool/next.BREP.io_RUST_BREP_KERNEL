//! The DRAFT ANALYSIS window: the floating window behind Modeling > Blend >
//! Draft Analysis (the Interference window's idiom: movable, resizable, its
//! `✕` turns the analysis off).
//!
//! The analysis itself is engine state ([`brep_render::engine_state::EngineState::draft_analysis_open`]
//! and the rest of `draft_analysis_ops`): display only, never saved, never an
//! undo step. This window draws its three inputs through the standard form
//! view (the pull is a reference field, picked like any other, with the
//! [`brep_render::engine_state::RefSelectTarget::DraftAnalysis`] arm), the pull it read with, and the
//! legend: one row per class with its colour and face count, whose click
//! selects those faces. While it is open, hovering a face in the viewport
//! shows that face's draft (and the near-limit note for a sampled face close
//! to the minimum).

use crate::automation::hit_keys::HitKeyDoc;
use crate::form_view::{form_view, FormViewSpec};
use brep_render::draft_analysis::DraftClass;
use brep_render::engine_state::{EngineState, RefSelectTarget};
use brep_render::features::form_fields_from_schema;
use eframe::egui;
use serde_json::{json, Value};
use std::collections::HashMap;

/// The window's form, in the feature schema shape.
pub fn schema() -> Value {
    json!({
        "inputParamsSchema": {
            "pullDirection": {
                "type": "reference_selection",
                "selectionFilter": ["FACE", "PLANE", "EDGE", "DATUM"],
                "multiple": false,
                "default_value": null,
                "label": "Pull direction",
                "hint": "The direction the part leaves the mould. A flat face gives its outward normal, a datum plane its normal, a straight edge its direction. Empty: the last Draft feature's pull, else +Z."
            },
            "flipPull": {
                "type": "boolean",
                "default_value": false,
                "label": "Flip pull",
                "hint": "Reverse the pull direction."
            },
            "minAngle": {
                "type": "number",
                "step": 0.5,
                "default_value": 1,
                "label": "Minimum draft (°)",
                "hint": "Faces drafted at least this much with the pull are positive, at least this much against it negative; less is Needs draft."
            }
        }
    })
}

/// The id the form's reference picks carry as their subject.
pub const SUBJECT: &str = "draftAnalysis";

#[derive(Default)]
pub struct DraftAnalysisWindow {
    /// Per-frame widget rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "draftAnalysis", prefix: "draftAnalysis:legend:", meaning: "a legend row: click selects that class's faces", command: None },
    HitKeyDoc { panel: "draftAnalysis", prefix: "draftAnalysis:", meaning: "a Draft Analysis form field", command: Some("draft_analysis") },
];

impl DraftAnalysisWindow {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    pub fn show(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.hits.clear();
        if !state.draft_analysis_active() {
            return;
        }
        let mut open = true;
        egui::Window::new("Draft Analysis")
            .id(egui::Id::new("brep-draft-analysis-window"))
            .open(&mut open)
            .movable(true)
            .resizable(true)
            .default_size([340.0, 360.0])
            .default_pos(super::below_chrome(ctx, 860.0, 0.0))
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().auto_shrink([false, false]).show(ui, |ui| self.body(ui, state));
            });
        if !open {
            if state.ref_select_field().is_some_and(|f| f.target == RefSelectTarget::DraftAnalysis) {
                state.cancel_ref_select();
            }
            state.draft_analysis_close();
        }
        // The hovered face's draft, beside the pointer.
        if let Some(text) = state.draft_analysis_hover_text() {
            if let Some(pos) = ctx.pointer_hover_pos() {
                egui::Area::new(egui::Id::new("brep-draft-analysis-hover"))
                    .order(egui::Order::Tooltip)
                    .fixed_pos(pos + egui::vec2(16.0, 16.0))
                    .interactable(false)
                    .show(ctx, |ui| {
                        egui::Frame::popup(ui.style()).show(ui, |ui| {
                            ui.label(text);
                        });
                    });
            }
        }
    }

    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        let schema = schema();
        let fields = form_fields_from_schema(&schema);
        let mut params = state.draft_analysis_params();
        let before = params.clone();
        let picking = state.ref_select_field().filter(|f| f.target == RefSelectTarget::DraftAnalysis);
        let read = state.draft_analysis_json();
        let error = read["error"].as_str().map(str::to_string);
        let banner_color = ui.visuals().error_fg_color;
        let spec = FormViewSpec {
            title: "Draft Analysis",
            subtitle: Some("Display only: nothing is added to the model."),
            fields: &fields,
            hidden: None,
            banner: error.as_deref().map(|text| crate::form_view::FormBanner::note(text, banner_color)),
            trailing: None,
            extra: None,
            exit_label: "Close",
            rollback: false,
            hits_prefix: "draftAnalysis:",
            read_only: None,
            picking: picking.as_ref().map(|f| crate::form::ActivePick { path: &f.path, names: &f.names }),
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        if let Some(activate) = out.ref_activate {
            if state.ref_select_active() {
                state.finish_ref_select();
            }
            state.begin_ref_select_for_draft_analysis(activate.path, activate.label, activate.filter, activate.seed);
        }
        if let Some(index) = out.pick_remove {
            state.ref_select_remove(index);
        }
        if out.pick_done {
            state.finish_ref_select();
        }
        if out.changed && params != before {
            let pull = (params["pullDirection"] != before["pullDirection"]).then(|| first_name(&params["pullDirection"]));
            let flip = params["flipPull"].as_bool();
            let minimum = number(&params["minAngle"]);
            if let Err(error) = state.draft_analysis_set(pull, flip, minimum) {
                state.push_notice(format!("Draft Analysis: {error}"));
            }
        }
        if out.exit_clicked {
            state.draft_analysis_close();
            return;
        }
        if picking.is_some() {
            state.ref_select_claim();
        }
        ui.separator();
        let pull = read["pull"].as_array().map(|p| p.iter().filter_map(Value::as_f64).collect::<Vec<_>>()).unwrap_or_default();
        if error.is_none() && pull.len() == 3 {
            ui.weak(format!(
                "Pull ({:+.3}, {:+.3}, {:+.3}) {}",
                pull[0],
                pull[1],
                pull[2],
                read["pullSource"].as_str().unwrap_or_default()
            ));
        }
        for class in DraftClass::ALL {
            let count = read["counts"][class.key()].as_u64().unwrap_or(0);
            let [r, g, b] = class.color().map(|v| (v * 255.0).round() as u8);
            let row = ui.horizontal(|ui| {
                let (rect, _) = ui.allocate_exact_size(egui::vec2(14.0, 14.0), egui::Sense::hover());
                ui.painter().rect_filled(rect, 2.0, egui::Color32::from_rgb(r, g, b));
                ui.add(egui::Button::new(format!("{}  {count}", class.label())).frame(false))
            });
            let response = row.inner.on_hover_text("Select these faces");
            self.hits.insert(format!("draftAnalysis:legend:{}", class.key()), response.rect);
            if response.clicked() {
                let faces = state.draft_analysis_faces_of(class);
                state.set_selection(&[], &faces, &[], &[]);
            }
        }
        ui.weak("A curved face is read at its mesh points; hover it to see its range.");
    }
}

fn first_name(value: &Value) -> Option<String> {
    match value {
        Value::String(name) => Some(name.clone()),
        Value::Array(names) => names.first().and_then(Value::as_str).map(str::to_string),
        _ => None,
    }
    .filter(|name| !name.trim().is_empty())
}

fn number(value: &Value) -> Option<f64> {
    value.as_f64().or_else(|| value.as_str().and_then(|s| s.trim().parse().ok()))
}
