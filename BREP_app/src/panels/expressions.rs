//! Expressions / parameters panel — the **variable sheet** that drives feature
//! params. The engine-owned history document carries an `expressions` source
//! string (a small `name = expr;` DSL) plus a `configurator` object; the pipeline
//! evaluates it when running features, so a numeric feature field may be the
//! expression string `"boxW"` — resolved against the variables defined here.
//!
//! Following the panel pattern, this owns NO model state: it edits + reads the
//! engine (`state.expressions_json()` / `state.set_expressions()` /
//! `state.expression_variables_json()` / `state.configurator_json()`), which write
//! into the single-source-of-truth `EngineState.history` and re-run the rolled-to
//! prefix so var-referencing params update live. The panel holds only a transient
//! editor buffer (mirrored from the engine when unfocused, committed on
//! Apply / focus-loss) and the per-frame widget hit-rects the headed verifier reads.

use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// The expressions panel's transient UI state (the model lives in the engine).
pub struct ExpressionsPanel {
    /// The multiline editor buffer. Mirrored from the engine's `expressions`
    /// whenever the editor is UNFOCUSED (so an Open / undo / redo that changed the
    /// document reflects here), and committed BACK to the engine on Apply or when
    /// the editor loses focus.
    buf: String,
    /// Per-frame widget hit-rects, published to JS for the headed verifier (wasm).
    hits: HashMap<String, egui::Rect>,
    /// The template-input editor's text buffers (label / min / max / choices),
    /// keyed by the field's hit key, held only while that field has focus.
    template_fields: HashMap<String, String>,
}

impl Default for ExpressionsPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl ExpressionsPanel {
    pub fn new() -> Self {
        Self {
            buf: String::new(),
            hits: HashMap::new(),
            template_fields: HashMap::new(),
        }
    }

    /// Draw the panel under its own collapsing header (matches the sibling panels).
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();

        egui::CollapsingHeader::new("Expressions / parameters")
            .id_salt("expressions-panel")
            .default_open(true)
            .show(ui, |ui| self.body(ui, state));
    }

    fn body(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        ui.label("Variable sheet — feature params may reference these (e.g. a size field set to `boxW`).");

        // --- the editor, bound to the engine-owned `expressions` source --------
        // Syntax highlighting via egui's BUILT-IN facility
        // (`egui_extras::syntax_highlighting`). A `layouter` re-lays every
        // frame: `highlight(...)` returns a colored `LayoutJob`, and the theme
        // is read from egui memory so it tracks the light/dark visuals. Results
        // are memoized inside `highlight`, so this is cheap per-frame.
        //
        // The language is `"c"`, not `"js"`. We build egui_extras WITHOUT its
        // `syntect` feature (16 crates natively, 20 on wasm, for this one
        // call), so `highlight` runs the crate's own fallback lexer, whose
        // `Language::new` knows only c/cpp, py, rust and toml — `"js"` returns
        // None there and falls through to unstyled monospace. The sheet is
        // `name = expr;` lines, so the C lane colours exactly what matters:
        // `//` comments, strings, numbers, identifiers and punctuation.
        let mut layouter = |ui: &egui::Ui, buf: &dyn egui::TextBuffer, wrap_width: f32| {
            let theme =
                egui_extras::syntax_highlighting::CodeTheme::from_memory(ui.ctx(), ui.style());
            let mut job = egui_extras::syntax_highlighting::highlight(
                ui.ctx(),
                ui.style(),
                &theme,
                buf.as_str(),
                "c",
            );
            job.wrap.max_width = wrap_width;
            ui.fonts_mut(|f| f.layout_job(job))
        };
        let editor = ui.add(
            egui::TextEdit::multiline(&mut self.buf)
                .id_salt("expressions-editor")
                .code_editor()
                .desired_rows(5)
                .desired_width(f32::INFINITY)
                .hint_text("boxW = 30;\nboxH = boxW / 2;")
                .layouter(&mut layouter),
        );
        self.hits.insert("expr:editor".into(), editor.rect);

        // --- commit: Apply button OR focus-loss; re-run only when text changed --
        let apply = ui.button("Apply (re-run history)");
        self.hits.insert("expr:apply".into(), apply.rect);

        let commit = editor.lost_focus() || apply.clicked();
        if commit {
            // Re-run only on a real change (each commit re-runs the rolled-to
            // prefix); a no-op commit must not thrash the kernel.
            if self.buf != state.expressions_json() {
                let _ = state.set_expressions(&self.buf);
            }
        } else if !editor.has_focus() {
            // Not editing → mirror the engine (an Open / New / undo / redo may have
            // replaced the document out from under the buffer).
            self.buf = state.expressions_json();
        }

        // --- parsed variable list (name = defining expression) -----------------
        ui.separator();
        ui.label(egui::RichText::new("Variables").strong());
        let vars: Vec<Value> =
            serde_json::from_str(&state.expression_variables_json()).unwrap_or_default();
        if vars.is_empty() {
            ui.weak("(no variables defined)");
        } else {
            egui::Grid::new("expr-vars")
                .num_columns(2)
                .striped(true)
                .show(ui, |ui| {
                    for v in &vars {
                        let name = v.get("name").and_then(Value::as_str).unwrap_or("");
                        let expr = v.get("expr").and_then(Value::as_str).unwrap_or("");
                        ui.monospace(name);
                        ui.monospace(format!("= {expr}"));
                        ui.end_row();
                    }
                });
        }

        // --- a TEMPLATE's inputs: which expressions the insert prompt asks for --
        if crate::document_class::is_template(state) {
            ui.separator();
            self.template_inputs(ui, state, &vars);
        }

        // --- configurator (typed named inputs) — read/display; editing deferred -
        ui.separator();
        ui.collapsing("Configurator (read-only)", |ui| {
            let cfg = state.configurator_json();
            let pretty = serde_json::from_str::<Value>(&cfg)
                .and_then(|v| serde_json::to_string_pretty(&v))
                .unwrap_or(cfg);
            ui.monospace(pretty);
            ui.weak("Typed named inputs — a deeper configurator editor is deferred.");
        });
    }

    /// The template-input editor (`.tbrep` only): one row per variable. Ticking
    /// **Input** marks it as a value the insert prompt asks for; a marked input
    /// takes an optional label, minimum, maximum and a fixed list of choices
    /// (comma-separated). Every change is one undoable document edit (typing in
    /// one field is one step).
    fn template_inputs(&mut self, ui: &mut egui::Ui, state: &mut EngineState, vars: &[Value]) {
        use crate::template::{apply_inputs, engine_inputs, TemplateInput};
        ui.label(egui::RichText::new("Template inputs").strong());
        ui.weak("Inserting this template asks for the ticked expressions and makes a new part with them.");
        let mut inputs = engine_inputs(state);
        let mut names: Vec<String> = vars
            .iter()
            .filter_map(|v| v.get("name").and_then(Value::as_str).map(str::to_string))
            .collect();
        for input in &inputs {
            if !names.contains(&input.name) {
                names.push(input.name.clone());
            }
        }
        if names.is_empty() {
            ui.weak("(define expressions above to mark them as inputs)");
            return;
        }
        let mut changed: Option<Option<String>> = None;
        // One block per expression: the tick and the name, and under a ticked
        // one its four fields, label beside field — a six-column grid does not
        // fit the pane's width.
        for name in &names {
            let index = inputs.iter().position(|input| input.name == *name);
            let mut marked = index.is_some();
            let defined = vars.iter().any(|v| v.get("name").and_then(Value::as_str) == Some(name));
            ui.horizontal(|ui| {
                let tick = ui.checkbox(&mut marked, "");
                self.hits.insert(format!("expr:tinput:{name}"), tick.rect);
                if tick.changed() {
                    match index {
                        Some(index) => {
                            inputs.remove(index);
                        }
                        None => inputs.push(TemplateInput { name: name.clone(), ..TemplateInput::default() }),
                    }
                    changed = Some(None);
                }
                ui.monospace(name);
                if !defined {
                    ui.weak("(not defined)");
                } else if marked {
                    ui.weak("input");
                }
            });
            let Some(index) = inputs.iter().position(|input| input.name == *name) else {
                continue;
            };
            let input = &mut inputs[index];
            let fmt = |value: Option<f64>| value.map(|v| v.to_string()).unwrap_or_default();
            let fields: [(&str, &str, String, &str); 4] = [
                ("tlabel", "Label", input.label.clone(), name.as_str()),
                ("tmin", "Min", fmt(input.min), "no limit"),
                ("tmax", "Max", fmt(input.max), "no limit"),
                ("tchoices", "Choices", input.choices.join(", "), "any value (or: 2, 3, 5)"),
            ];
            ui.indent(format!("tinput-{name}"), |ui| {
                egui::Grid::new(format!("tinput-grid-{name}")).num_columns(2).show(ui, |ui| {
                    for (field, caption, current, hint) in fields {
                        ui.weak(caption);
                        let key = format!("expr:{field}:{name}");
                        let mut text = self.template_fields.get(&key).cloned().unwrap_or(current);
                        let edit = ui.add(
                            egui::TextEdit::singleline(&mut text)
                                .hint_text(hint)
                                .desired_width(ui.available_width().min(220.0)),
                        );
                        self.hits.insert(key.clone(), edit.rect);
                        ui.end_row();
                        if edit.changed() {
                            let applied = match field {
                                "tlabel" => {
                                    input.label = text.clone();
                                    true
                                }
                                "tmin" | "tmax" => {
                                    let parsed = if text.trim().is_empty() {
                                        Some(None)
                                    } else {
                                        text.trim().parse::<f64>().ok().filter(|v| v.is_finite()).map(Some)
                                    };
                                    match parsed {
                                        Some(value) if field == "tmin" => {
                                            input.min = value;
                                            true
                                        }
                                        Some(value) => {
                                            input.max = value;
                                            true
                                        }
                                        None => false,
                                    }
                                }
                                _ => {
                                    input.choices = text
                                        .split(',')
                                        .map(str::trim)
                                        .filter(|choice| !choice.is_empty())
                                        .map(str::to_string)
                                        .collect();
                                    true
                                }
                            };
                            if applied {
                                changed = Some(Some(key.clone()));
                            }
                        }
                        if edit.has_focus() {
                            self.template_fields.insert(key, text);
                        } else {
                            self.template_fields.remove(&key);
                        }
                    }
                });
            });
        }
        if let Some(coalesce) = changed {
            apply_inputs(state, &inputs, coalesce.as_deref());
        }
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "expr", prefix: "expr:editor", meaning: "the expressions script editor", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:apply", meaning: "apply the script and rerun", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:tinput:", meaning: "a template's input tick (expr:tinput:<expression name>), shown while editing a .tbrep", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:tlabel:", meaning: "a marked template input's label field", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:tmin:", meaning: "a marked template input's minimum field", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:tmax:", meaning: "a marked template input's maximum field", command: None },
    HitKeyDoc { panel: "expr", prefix: "expr:tchoices:", meaning: "a marked template input's choices field (comma-separated)", command: None },
];
