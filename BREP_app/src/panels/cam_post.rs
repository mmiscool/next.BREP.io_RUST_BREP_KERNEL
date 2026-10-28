//! Posting an NC program (contract C19-27): the program's operations, in its
//! order, are assembled into one program with their generated toolpaths and
//! run through the program's post (C4's runtime, a built-in post by id). The
//! CAM pane's "Post…" on a program row shows the text and saves it through
//! the one export door (the Save-file chooser on desktop, a download in the
//! browser); `cam_post_text` returns the same text and file name.

use std::collections::{BTreeMap, HashMap};

use brep_cam::program::{assemble, OperationInput, ProgramInput, SetupInput};
use brep_render::engine_state::EngineState;
use egui;
use serde_json::{json, Value};

use crate::automation::hit_keys::HitKeyDoc;
use crate::document::cam::CamDocument;
use crate::panels::file::{SaveBody, SaveRequest};

/// A posted program.
#[derive(Clone, Debug, PartialEq)]
pub struct Posted {
    /// The NC program's id.
    pub program: String,
    pub post: String,
    /// The operations posted, in order.
    pub operations: Vec<String>,
    pub text: String,
    /// Where it would save: `<program name>.<post extension>`.
    pub file_name: String,
    pub warnings: Vec<String>,
}

impl Posted {
    pub fn to_json(&self) -> Value {
        json!({
            "program": self.program,
            "post": self.post,
            "operations": self.operations,
            "text": self.text,
            "fileName": self.file_name,
            "warnings": self.warnings,
            "lines": self.text.lines().count(),
        })
    }
}

/// The program input for NC program `id`: its operations in its order, each
/// with its tool and generated toolpath; consecutive operations of one
/// setup share a setup section. Refused by name when the program, an
/// operation, its tool or a usable toolpath is missing.
pub fn program_input(state: &EngineState, doc: &CamDocument, id: &str) -> Result<ProgramInput, String> {
    let program = doc.program(id).ok_or_else(|| format!("no NC program `{id}` (see cam_state ncPrograms)"))?;
    if program.operations.is_empty() {
        return Err(format!("NC program `{id}` has no operations"));
    }
    let mut setups: Vec<SetupInput> = Vec::new();
    for op_id in &program.operations {
        let op = doc.operation(op_id.as_str()).ok_or_else(|| format!("NC program `{id}` names a missing operation `{op_id}`"))?;
        let tool = doc.tool(op.tool.as_str()).ok_or_else(|| format!("operation `{op_id}` names a missing tool `{}`", op.tool))?;
        // Only a current result posts: a stale one was generated from inputs
        // that have since changed.
        match crate::panels::cam_generate::status_of(state, doc, op_id.as_str()) {
            crate::panels::cam_generate::OpStatus::Ok { .. } => {}
            other => return Err(format!("operation `{op_id}` is {}: generate it first", other.word())),
        }
        let toolpath = state
            .cam
            .results
            .get(op_id.as_str())
            .and_then(|r| r.toolpath.clone())
            .ok_or_else(|| format!("operation `{op_id}` has no generated toolpath: generate it first"))?;
        let setup = doc.setup(op.setup.as_str()).ok_or_else(|| format!("operation `{op_id}` names a missing setup `{}`", op.setup))?;
        let input = OperationInput { operation: op.clone(), tool: tool.clone(), toolpath };
        match setups.last_mut() {
            Some(last) if last.setup.id == setup.id => last.operations.push(input),
            _ => setups.push(SetupInput { setup: setup.clone(), operations: vec![input] }),
        }
    }
    let machine = doc.setup_machine(setups[0].setup.id.as_str());
    let tool_change_seconds = machine.as_ref().map_or_else(|| brep_cam::model::Machine::default().tool_change_seconds, |m| m.tool_change_seconds);
    Ok(ProgramInput { name: program.name.clone(), number: program.number, machine, setups, tool_change_seconds, ..Default::default() })
}

/// Post NC program `id` through its post.
pub fn post_program(state: &EngineState, doc: &CamDocument, id: &str) -> Result<Posted, String> {
    let input = program_input(state, doc, id)?;
    let program = doc.program(id).expect("checked by program_input");
    let operations: Vec<String> = program.operations.iter().map(|o| o.to_string()).collect();
    let assembled = assemble(&input).map_err(|e| format!("NC program `{id}`: {e}"))?;
    let output = run_post(&program.post, &program.properties, assembled)?;
    Ok(Posted { program: id.to_string(), post: program.post.clone(), operations, text: output.text, file_name: output.file_name, warnings: output.warnings })
}

fn run_post(post: &str, properties: &std::collections::BTreeMap<String, serde_json::Value>, program: brep_cam::program::Program) -> Result<brep_cam::post::PostOutput, String> {
    if brep_cam::post::builtin(post).is_none() {
        return Err(format!("post `{post}` is not a built-in post (built-ins: {})", brep_cam::post::BUILTIN_POSTS.iter().map(|p| p.id).collect::<Vec<_>>().join(", ")));
    }
    let request = brep_cam::post::PostRequest { program, properties: properties.clone(), ..Default::default() };
    brep_cam::post::run_builtin(post, &request).map_err(|e| format!("post `{post}`: {e}"))
}

/// The pane's Post section: the last posted program's preview and its Save.
#[derive(Default)]
pub struct CamPostPanel {
    hits: HashMap<String, egui::Rect>,
    pub posted: Option<Posted>,
    pub error: Option<String>,
    /// A save the shell takes ([`crate::panels::cam::CamPanel::take_shell_requests`]).
    pub pending_save: Option<SaveRequest>,
    /// The program last posted or refused: the one whose properties show.
    pub program: Option<String>,
    /// A built-in post's property list, by post id (`describe` runs the script once).
    specs: HashMap<String, Result<BTreeMap<String, brep_cam::post::PropertySpec>, String>>,
}

/// The value a property widget shows: the program's, else the post's default.
fn shown_value<'a>(props: &'a BTreeMap<String, Value>, name: &str, spec: &'a brep_cam::post::PropertySpec) -> &'a Value {
    props.get(name).unwrap_or(&spec.default)
}

impl CamPostPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Post program `id` and show the result (or why not).
    pub fn open(&mut self, state: &EngineState, doc: &CamDocument, id: &str) -> Result<&Posted, String> {
        self.program = Some(id.to_string());
        match post_program(state, doc, id) {
            Ok(p) => {
                self.error = None;
                Ok(self.posted.insert(p))
            }
            Err(e) => {
                self.posted = None;
                self.error = Some(e.clone());
                Err(e)
            }
        }
    }

    /// Hand the shown program to the shell as a file.
    pub fn save(&mut self) -> Option<&SaveRequest> {
        let p = self.posted.as_ref()?;
        self.pending_save = Some(SaveRequest { title: format!("Save NC program {}", p.file_name), file_name: p.file_name.clone(), body: SaveBody::Text(p.text.clone()) });
        self.pending_save.as_ref()
    }

    pub fn close(&mut self) {
        self.posted = None;
        self.error = None;
        self.program = None;
    }

    /// The property list of built-in post `post` (cached).
    fn specs_of(&mut self, post: &str) -> &Result<BTreeMap<String, brep_cam::post::PropertySpec>, String> {
        self.specs.entry(post.to_string()).or_insert_with(|| match brep_cam::post::builtin(post) {
            Some(b) => brep_cam::post::describe(b.source, b.id).map(|d| d.properties).map_err(|e| e.message),
            None => Err(format!("post `{post}` is not a built-in post: its properties are set by name (cam_update_program)")),
        })
    }

    pub fn hit_rects(&self) -> &HashMap<String, egui::Rect> {
        &self.hits
    }

    pub fn state_json(&self) -> Value {
        json!({
            "open": self.posted.is_some() || self.error.is_some(),
            "posted": self.posted.as_ref().map(|p| json!({ "program": p.program, "post": p.post, "operations": p.operations, "fileName": p.file_name, "lines": p.text.lines().count(), "warnings": p.warnings })),
            "error": self.error,
        })
    }

    /// Draw the preview (nothing when no program was posted) and the
    /// program's post properties. Returns a property edit `(program, patch)`
    /// for the caller to write and re-post.
    pub fn show(&mut self, ui: &mut egui::Ui, doc: &CamDocument) -> Option<(String, BTreeMap<String, Value>)> {
        self.hits.clear();
        if self.posted.is_none() && self.error.is_none() {
            return None;
        }
        let mut edit: Option<(String, BTreeMap<String, Value>)> = None;
        let program = self.program.as_deref().and_then(|id| doc.program(id)).cloned();
        ui.separator();
        let mut save = false;
        let mut close = false;
        let title = self.posted.as_ref().map_or_else(|| "Post".to_string(), |p| format!("Post \u{00b7} {} ({})", p.file_name, p.post));
        let header = egui::CollapsingHeader::new(title).id_salt("cam-post").default_open(true).show(ui, |ui| {
            if let Some(e) = &self.error {
                ui.colored_label(ui.visuals().error_fg_color, e);
            }
            // The post's properties (a mill-turn lathe post's builder codes,
            // the program number, ...): what the post runs with for this program.
            if let Some(program) = &program {
                let specs = self.specs_of(&program.post).clone();
                let props = egui::CollapsingHeader::new(format!("Post properties ({})", program.post)).id_salt("cam-post-props").default_open(self.error.is_some()).show(ui, |ui| {
                    match &specs {
                        Err(e) => {
                            ui.label(egui::RichText::new(e).weak());
                        }
                        Ok(specs) => {
                            egui::Grid::new("cam-post-props-grid").num_columns(2).show(ui, |ui| {
                                for (name, spec) in specs {
                                    ui.label(&spec.title).on_hover_text(if spec.description.is_empty() { name.as_str() } else { spec.description.as_str() });
                                    let value = shown_value(&program.properties, name, spec).clone();
                                    let mut changed: Option<Value> = None;
                                    let r = match spec.kind.as_str() {
                                        "boolean" => {
                                            let mut b = value.as_bool().unwrap_or(false);
                                            let r = ui.checkbox(&mut b, "");
                                            if r.changed() { changed = Some(Value::Bool(b)); }
                                            r
                                        }
                                        "number" | "integer" => {
                                            let mut n = value.as_f64().unwrap_or(0.0);
                                            let mut d = egui::DragValue::new(&mut n);
                                            if let Some(lo) = spec.min { d = d.range(lo..=spec.max.unwrap_or(f64::MAX)); }
                                            if spec.kind == "integer" { d = d.fixed_decimals(0); }
                                            let r = ui.add(d);
                                            if r.changed() { changed = Some(if spec.kind == "integer" { json!(n.round() as i64) } else { json!(n) }); }
                                            r
                                        }
                                        "enum" => {
                                            let current = value.as_str().unwrap_or("").to_string();
                                            let mut pick = current.clone();
                                            let r = egui::ComboBox::from_id_salt(format!("cam-post-prop-{name}")).selected_text(&current).show_ui(ui, |ui| {
                                                for v in spec.values.iter().flatten().filter_map(Value::as_str) {
                                                    ui.selectable_value(&mut pick, v.to_string(), v);
                                                }
                                            }).response;
                                            if pick != current { changed = Some(Value::String(pick)); }
                                            r
                                        }
                                        _ => {
                                            let mut text = value.as_str().map(str::to_string).unwrap_or_else(|| if value.is_null() { String::new() } else { value.to_string() });
                                            let r = ui.add(egui::TextEdit::singleline(&mut text).desired_width(160.0));
                                            if r.lost_focus() && Value::String(text.clone()) != value { changed = Some(Value::String(text)); }
                                            r
                                        }
                                    };
                                    self.hits.insert(format!("cam:post:prop:{name}"), r.rect);
                                    if let Some(v) = changed {
                                        edit = Some((program.id.clone(), BTreeMap::from([(name.clone(), v)])));
                                    }
                                    ui.end_row();
                                }
                            });
                        }
                    }
                });
                self.hits.insert("cam:post:props".into(), props.header_response.rect);
            }
            if let Some(p) = &self.posted {
                for w in &p.warnings {
                    ui.label(egui::RichText::new(w).weak());
                }
                let mut text = p.text.as_str();
                let preview = egui::ScrollArea::vertical().id_salt("cam-post-text").max_height(240.0).show(ui, |ui| {
                    ui.add(egui::TextEdit::multiline(&mut text).code_editor().desired_width(f32::INFINITY))
                });
                self.hits.insert("cam:post:text".into(), preview.inner_rect);
            }
            ui.horizontal(|ui| {
                if self.posted.is_some() {
                    let b = ui.button("Save\u{2026}").on_hover_text("Save the NC file (a download in the browser)");
                    self.hits.insert("cam:post:save".into(), b.rect);
                    save = b.clicked();
                }
                let c = ui.button("Close");
                self.hits.insert("cam:post:close".into(), c.rect);
                close = c.clicked();
            });
        });
        self.hits.insert("cam:post".into(), header.header_response.rect);
        if save {
            self.save();
        }
        if close {
            self.close();
        }
        edit
    }
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "cam", prefix: "cam:post:program:", meaning: "an NC program row's Post\u{2026} button: post it through its post and show the text", command: Some("cam_post_text") },
    HitKeyDoc { panel: "cam", prefix: "cam:post:text", meaning: "the posted NC text's preview", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:post:save", meaning: "save the posted NC file (the Save-file chooser on desktop, a download in the browser)", command: Some("cam_post_text") },
    HitKeyDoc { panel: "cam", prefix: "cam:post:close", meaning: "close the post preview", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:post:prop:", meaning: "one post property of the shown NC program (cam:post:prop:<name>): a checkbox, number, choice or text box; a change is written and the program posted again", command: Some("cam_update_program") },
    HitKeyDoc { panel: "cam", prefix: "cam:post:props", meaning: "the shown NC program's post properties section", command: None },
    HitKeyDoc { panel: "cam", prefix: "cam:post", meaning: "the post preview's header", command: None },
];

