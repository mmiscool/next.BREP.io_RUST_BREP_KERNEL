//! The Simulation pane — the Simulation workbench's
//! ([`crate::workbench::simulation`]) study list over the assembly's
//! `simulation` block, and the active study's timeline: its name, duration
//! and frame rate, the transport (play, pause, stop, loop) over a time ruler
//! that scrubs, and a row per motor and per clip. A row opens its item as
//! the app's schema-driven form ([`crate::form_view`], the one dialog engine
//! every feature, constraint and PMI form uses): standard number, option and
//! checkbox fields from [`brep_simulation::forms`], and a clip's parts as a
//! reference-selection field picked in the viewport (the engine's
//! `simulation_refs` lane). Playback poses the displays only; the engine's
//! `simulation_play` module bakes and shows the frames.
//!
//! The pane holds only transient widget state (its hit rects, an action a
//! toolbar button handed it, the last notice). Every record it draws is read
//! from the engine each frame, and every edit goes through the engine's
//! simulation methods — the same ones the `simulation_*` automation commands
//! call — so undo, save and reload come from the document. A field being
//! typed in is one undo step per field.

use crate::automation::hit_keys::HitKeyDoc;
use crate::form_view::{form_view, FormViewSpec};
use brep_render::engine_state::{EngineState, MotorMate, RefSelectField, RefSelectTarget};
use brep_render::features::form_fields_from_schema;
use brep_simulation::forms::{self, FormChoices, MateChoice};
use brep_simulation::model::{Clip, ClipBody, Drive, Study, FPS_RANGE, MAX_DURATION};
use serde_json::{json, Value};
use std::collections::{HashMap, HashSet};

/// The owner the form's hovered reference lines light entities under.
const DIALOG_HOVER_OWNER: &str = "simulation";

/// An action a toolbar button hands the pane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SimulationRequest {
    NewStudy,
    AddMotor,
}

/// The ruler's height, in points.
const RULER_HEIGHT: f32 = 22.0;

/// What the pane's Export button asks the app for (the app owns the
/// viewport's render core, so it runs the export: `take_export`).
#[derive(Debug, Clone, PartialEq)]
pub struct ExportRequest {
    pub study: String,
    /// `gif` or `mp4`.
    pub format: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub path: String,
}

#[derive(Default)]
pub struct SimulationPanel {
    hits: HashMap<String, egui::Rect>,
    request: Option<SimulationRequest>,
    notice: Option<String>,
    error: Option<String>,
    /// Whether the last frame ran in the Simulation workbench (`left_workbench`).
    in_workbench: bool,
    /// The Export row's settings (kept across frames; the path defaults to the
    /// study's name when empty).
    export_format: String,
    export_width: u32,
    export_height: u32,
    export_fps: u32,
    export_path: String,
    /// An export the Export button asked for, until the app takes it.
    export: Option<ExportRequest>,
    /// The motor or clip whose form is open (its row was clicked, or it was
    /// just added); `None` shows the rows.
    open: Option<String>,
    /// The entity a reference line of the form is hovered over this frame.
    hover: Option<String>,
}

/// One engine edit the frame asks for, applied after drawing.
enum Change {
    Activate(String),
    NewStudy,
    AddMotor { study: String, mate: Option<String> },
    Remove { study: String, item: Option<String> },
    Patch { study: String, item: Option<String>, patch: Value, coalesce: Option<String> },
    Play { study: String },
    Pause,
    Stop,
    Loop { study: String, looping: bool },
    Seek { study: String, time: f64 },
    AddClip { study: String, kind: String },
    /// Bake to the end and write the study's motion into the block, for an
    /// instructions step to attach.
    Publish { study: String },
    /// Open a motor's or clip's form (`None`: back to the rows).
    Open(Option<String>),
    /// A clip form's Parts field was activated: pick in the viewport.
    BeginRefSelect { clip: String, path: Vec<String>, label: String, filter: Vec<String>, multiple: bool, seed: Vec<String> },
    /// The focused reference field's Done: commit the running pick.
    FinishRefSelect,
    /// An `✕` on the focused reference field: drop the i-th running pick.
    RefSelectRemove(usize),
}

/// The clip kinds `+ Clip` offers (the ones playback applies).
const CLIP_KINDS: [(&str, &str); 6] = [("suppress", "Suppress a mate"), ("visibility", "Hide parts"), ("transparency", "Fade parts"), ("camera", "Camera path"), ("section", "Section"), ("explode", "Exploded view")];

impl SimulationPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Hand the pane an action; it performs it on the next frame it draws.
    pub fn request(&mut self, request: SimulationRequest) {
        self.request = Some(request);
    }

    /// The export the Export button asked for, if any (the app runs it and
    /// reports back through `report`).
    pub fn take_export(&mut self) -> Option<ExportRequest> {
        self.export.take()
    }

    /// What an export (or any app-run action) came to: a notice or an error.
    pub fn report(&mut self, result: Result<String, String>) {
        match result {
            Ok(notice) => {
                self.notice = Some(notice);
                self.error = None;
            }
            Err(error) => self.error = Some(error),
        }
    }

    /// Told each frame whether the current workbench offers the Simulation
    /// pane (Simulation, All); true on the frame that leaves it (the playback
    /// is then stopped, so every posed part is back at its document pose).
    fn left_workbench(&mut self, in_workbench: bool) -> bool {
        let left = self.in_workbench && !in_workbench;
        self.in_workbench = in_workbench;
        left
    }

    /// Drive the study's playback for this frame. The playback belongs to the
    /// Simulation pane: it ticks (bakes, advances the playhead, shows a frame,
    /// tints a clash, drives a camera clip) only while the current workbench
    /// offers the pane (`pane_shown`: Simulation or All, whether or not the
    /// pane is open in the dock), and the frame that leaves it stops it, so the
    /// displays and the camera are back as the document has them. Elsewhere
    /// the bake is kept for the return, and nothing done there reaches it: an explode
    /// step moved in the PMI workbench is a history revision, which a running
    /// tick would answer by re-baking the study and showing its first frame
    /// there — the clash tint and the camera clip over a PMI edit (owner
    /// report 2026-10-26). Returns whether another frame is wanted.
    pub fn drive_playback(&mut self, engine: &mut EngineState, pane_shown: bool, dt: f64) -> bool {
        if self.left_workbench(pane_shown) && engine.simulation_playing_or_shown() {
            engine.simulation_stop();
        }
        pane_shown && engine.simulation_tick(dt)
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// What automation reads beside the engine's `simulation_state_json`:
    /// the last notice and error, and the motor or clip whose form is open.
    pub fn state_json(&self) -> String {
        json!({ "notice": self.notice, "error": self.error, "open": self.open }).to_string()
    }

    pub fn show(&mut self, ui: &mut egui::Ui, engine: &mut EngineState) {
        self.hits.clear();
        self.hits.insert("panel:clip".into(), ui.clip_rect());
        if !engine.simulation_available() {
            ui.label("Open an assembly: a study drives its distance and angle mates as motors.");
            return;
        }
        if let Some(request) = self.request.take() {
            let study = engine.simulation().ok().flatten().and_then(|b| b.active_study().map(|s| s.id.clone()));
            let change = match request {
                SimulationRequest::NewStudy => Change::NewStudy,
                SimulationRequest::AddMotor => Change::AddMotor { study: study.unwrap_or_default(), mate: None },
            };
            self.apply(change, engine);
        }
        let block = match engine.simulation() {
            Ok(block) => block.unwrap_or_default(),
            Err(error) => {
                ui.colored_label(ui.visuals().error_fg_color, format!("The studies cannot be read: {error}"));
                return;
            }
        };
        if let Some(error) = &self.error {
            let label = ui.colored_label(ui.visuals().error_fg_color, error);
            self.hits.insert("simulation:error".into(), label.rect);
        }
        if let Some(notice) = &self.notice {
            let label = ui.weak(notice);
            self.hits.insert("simulation:notice".into(), label.rect);
        }
        let mut change = None;
        let active = block.active_study().map(|s| s.id.clone());
        ui.horizontal_wrapped(|ui| {
            ui.label("Studies");
            for study in &block.studies {
                let row = ui.selectable_label(active.as_deref() == Some(study.id.as_str()), &study.name);
                self.hits.insert(format!("simulation:study:{}", study.id), row.rect);
                if row.clicked() && active.as_deref() != Some(study.id.as_str()) {
                    change = Some(Change::Activate(study.id.clone()));
                }
            }
            let new = ui.button("+ Study");
            self.hits.insert("simulation:new_study".into(), new.rect);
            if new.clicked() {
                change = Some(Change::NewStudy);
            }
        });
        let Some(study) = block.active_study() else {
            ui.weak("No study yet: + Study (or New study on the toolbar) starts one.");
            if let Some(change) = change {
                self.apply(change, engine);
            }
            return;
        };
        ui.separator();
        self.study_fields(ui, study, &mut change);
        let playback = engine.simulation_playback_json();
        self.transport(ui, study, &playback, &mut change);
        self.ruler(ui, study, &playback, &mut change);
        ui.separator();
        let mates = engine.simulation_motor_mates();
        let pmi = engine.pmi_state();
        let choices = FormChoices {
            mates: mates.iter().map(|m| MateChoice { id: m.id.clone(), kind: m.kind.clone(), value: m.value }).collect(),
            constraints: engine.simulation_constraint_ids(),
            camera_views: pmi.views.iter().filter(|v| v.camera.is_some()).map(|v| v.id.clone()).collect(),
            section_views: pmi.views.iter().filter(|v| v.display.section.is_some()).map(|v| v.id.clone()).collect(),
            exploded_views: engine.exploded_views().into_iter().map(|v| v.id).collect(),
        };
        // An item that is gone (removed, undone) closes its form.
        if self.open.as_deref().is_some_and(|id| !study.motors.iter().any(|m| m.id == id) && !study.clips.iter().any(|c| c.id == id)) {
            self.open = None;
        }
        self.hover = None;
        match self.open.clone() {
            Some(open) => {
                // The form replaces the rows (Return brings them back); the
                // transport and ruler above it keep scrubbing.
                let picking = engine.ref_select_field().filter(|f| f.target == RefSelectTarget::Simulation && f.subject == open);
                self.item_form(ui, study, &open, &choices, picking.as_ref(), &mut change);
            }
            None => {
                self.motors(ui, study, &mates, &mut change);
                ui.separator();
                self.clips(ui, study, &mut change);
            }
        }
        ui.separator();
        self.export_row(ui, study);
        if let Some(change) = change {
            self.apply(change, engine);
        }
        // This pane drew the pick: the shell finishes an in-form pick no form
        // claims, so one never runs on with its field off screen.
        if let Some(field) = engine.ref_select_field() {
            if field.target == RefSelectTarget::Simulation && self.open.as_deref() == Some(field.subject.as_str()) {
                engine.ref_select_claim();
            }
        }
        // A hovered reference line lights the part it names in the 3D view.
        let hover_changed = match &self.hover {
            Some(name) => engine.hover_entity_by_name(DIALOG_HOVER_OWNER, name),
            None => engine.dialog_hover_end(DIALOG_HOVER_OWNER),
        };
        if hover_changed {
            ui.ctx().request_repaint();
        }
    }

    /// Export: format, size, fps, path and the button. The app renders the
    /// baked frames at that size through the viewport and writes the file.
    fn export_row(&mut self, ui: &mut egui::Ui, study: &Study) {
        if self.export_format.is_empty() {
            self.export_format = "gif".into();
        }
        if self.export_width == 0 || self.export_height == 0 {
            (self.export_width, self.export_height) = (960, 540);
        }
        if self.export_fps == 0 {
            self.export_fps = study.fps;
        }
        ui.strong("Export");
        ui.horizontal_wrapped(|ui| {
            let mut format = self.export_format.clone();
            let combo = egui::ComboBox::from_id_salt("simulation-export-format").selected_text(format.to_uppercase()).show_ui(ui, |ui| {
                for kind in ["gif", "mp4", "html"] {
                    let option = ui.selectable_value(&mut format, kind.to_string(), kind.to_uppercase());
                    self.hits.insert(format!("simulation:menuitem:export_format:{kind}"), option.rect);
                }
            });
            self.hits.insert("simulation:export:format".into(), combo.response.rect);
            if format != self.export_format {
                // A default path follows the format's extension.
                if self.export_path.ends_with(&format!(".{}", self.export_format)) {
                    self.export_path = format!("{}.{format}", self.export_path.trim_end_matches(&format!(".{}", self.export_format)));
                }
                self.export_format = format;
            }
            for (label, value, key) in [("width", &mut self.export_width, "simulation:export:width"), ("height", &mut self.export_height, "simulation:export:height")] {
                ui.label(label);
                let drag = ui.add(egui::DragValue::new(value).range(16..=4096).speed(8));
                self.hits.insert(key.into(), drag.rect);
            }
            ui.label("fps");
            let drag = ui.add(egui::DragValue::new(&mut self.export_fps).range(1..=120));
            self.hits.insert("simulation:export:fps".into(), drag.rect);
        });
        ui.horizontal_wrapped(|ui| {
            if self.export_path.is_empty() {
                self.export_path = format!("{}.{}", study.name.trim().replace(' ', "-"), self.export_format);
            }
            ui.label("file");
            let field = ui.add(egui::TextEdit::singleline(&mut self.export_path).desired_width(220.0));
            self.hits.insert("simulation:export:path".into(), field.rect);
            let run = ui.button("Export");
            self.hits.insert("simulation:export:run".into(), run.rect);
            if run.clicked() {
                self.export = Some(ExportRequest {
                    study: study.id.clone(),
                    format: self.export_format.clone(),
                    width: self.export_width,
                    height: self.export_height,
                    fps: self.export_fps,
                    path: self.export_path.clone(),
                });
                self.notice = Some("Exporting…".into());
                self.error = None;
            }
        });
    }

    /// A row per clip — its label (click: its form), its span and Remove;
    /// then + Clip.
    fn clips(&mut self, ui: &mut egui::Ui, study: &Study, change: &mut Option<Change>) {
        ui.strong("Clips");
        if study.clips.is_empty() {
            ui.weak("None: + Clip suppresses a mate, hides or fades parts, moves the camera, cuts a section or plays an exploded view over a span.");
        }
        for clip in &study.clips {
            let key = format!("simulation:clip:{}", clip.id);
            ui.horizontal_wrapped(|ui| {
                let row = ui.selectable_label(false, clip_label(clip)).on_hover_text("Open the clip's fields");
                self.hits.insert(key.clone(), row.rect);
                if row.clicked() {
                    *change = Some(Change::Open(Some(clip.id.clone())));
                }
                ui.weak(format!("{:.2}–{:.2} s", clip.from, clip.to));
                let remove = ui.button("Remove");
                self.hits.insert(format!("{key}:remove"), remove.rect);
                if remove.clicked() {
                    *change = Some(Change::Remove { study: study.id.clone(), item: Some(clip.id.clone()) });
                }
            });
        }
        let mut kind = String::new();
        let combo = egui::ComboBox::from_id_salt("simulation-add-clip").selected_text("+ Clip").show_ui(ui, |ui| {
            for (id, label) in CLIP_KINDS {
                let option = ui.selectable_value(&mut kind, id.to_string(), label);
                self.hits.insert(format!("simulation:menuitem:clip_kind:{id}"), option.rect);
            }
        });
        self.hits.insert("simulation:add_clip".into(), combo.response.rect);
        if !kind.is_empty() {
            *change = Some(Change::AddClip { study: study.id.clone(), kind });
        }
    }

    /// The open motor's or clip's form: the schema-driven dialog every pane
    /// shares, over the item's flat params ([`brep_simulation::forms`]). An
    /// edit becomes the item's patch; a clip's Parts field runs the viewport
    /// pick; Return goes back to the rows.
    fn item_form(&mut self, ui: &mut egui::Ui, study: &Study, open: &str, choices: &FormChoices, picking: Option<&RefSelectField>, change: &mut Option<Change>) {
        let motor = study.motors.iter().find(|m| m.id == open);
        let clip = study.clips.iter().find(|c| c.id == open);
        let (title, subtitle, schema, mut params, hidden) = match (motor, clip) {
            (Some(motor), _) => {
                let params = forms::motor_params(motor);
                let mate = choices.mates.iter().find(|m| m.id == motor.constraint).map_or_else(|| motor.constraint.clone(), |m| format!("{} ({})", m.id, m.kind));
                (format!("Motor {}", motor.id), mate, forms::motor_schema(motor, choices), params.clone(), forms::motor_hidden(&params))
            }
            (None, Some(clip)) => {
                let params = forms::clip_params(clip);
                (format!("Clip {}", clip.id), forms::clip_title(clip).to_string(), forms::clip_schema(clip, choices), params.clone(), forms::clip_hidden(clip, &params))
            }
            (None, None) => return,
        };
        let fields = form_fields_from_schema(&schema);
        let hidden: HashSet<String> = hidden.into_iter().collect();
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(subtitle.as_str()),
            fields: &fields,
            hidden: Some(&hidden),
            banner: None,
            trailing: None,
            exit_label: "Return to list",
            extra: None,
            rollback: false,
            hits_prefix: "simulation:item:",
            read_only: None,
            picking: picking.map(|f| crate::form::ActivePick { path: &f.path, names: &f.names }),
        };
        let before = params.clone();
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        self.hover = out.hovered_entity;
        if let Some(activate) = out.ref_activate {
            *change = Some(Change::BeginRefSelect { clip: open.to_string(), path: activate.path, label: activate.label, filter: activate.filter, multiple: activate.multiple, seed: activate.seed });
        }
        if let Some(index) = out.pick_remove {
            *change = Some(Change::RefSelectRemove(index));
        }
        if out.pick_done {
            *change = Some(Change::FinishRefSelect);
        }
        if out.exit_clicked {
            *change = Some(Change::Open(None));
        }
        if out.changed {
            // A typing run in one field is one undo step; another field is another.
            let field = params
                .as_object()
                .into_iter()
                .flatten()
                .find(|(key, value)| before.get(key.as_str()) != Some(value))
                .map(|(key, _)| key.clone())
                .unwrap_or_default();
            let patch = match (motor, clip) {
                (Some(_), _) => forms::motor_patch(&params, choices),
                (None, Some(clip)) => forms::clip_patch(clip, &params),
                (None, None) => return,
            };
            match patch {
                Ok(patch) => *change = Some(Change::Patch { study: study.id.clone(), item: Some(open.to_string()), patch, coalesce: Some(format!("simulation:{}:{open}:{field}", study.id)) }),
                Err(error) => self.error = Some(error),
            }
        }
    }

    /// The study's name, duration and frame rate, and Delete study.
    fn study_fields(&mut self, ui: &mut egui::Ui, study: &Study, change: &mut Option<Change>) {
        let id = study.id.clone();
        ui.horizontal_wrapped(|ui| {
            let mut name = study.name.clone();
            let field = ui.add(egui::TextEdit::singleline(&mut name).desired_width(140.0));
            self.hits.insert("simulation:field:name".into(), field.rect);
            if field.changed() && !name.trim().is_empty() {
                *change = Some(patch(&id, None, "name", json!(name.trim()), "name"));
            }
            ui.label("Duration");
            let mut duration = study.duration;
            let field = ui.add(egui::DragValue::new(&mut duration).range(0.1..=MAX_DURATION).speed(0.1).suffix(" s"));
            self.hits.insert("simulation:field:duration".into(), field.rect);
            if field.changed() {
                *change = Some(patch(&id, None, "duration", json!(duration), "duration"));
            }
            let mut fps = study.fps;
            let field = ui.add(egui::DragValue::new(&mut fps).range(*FPS_RANGE.start()..=*FPS_RANGE.end()).suffix(" fps"));
            self.hits.insert("simulation:field:fps".into(), field.rect);
            if field.changed() {
                *change = Some(patch(&id, None, "fps", json!(fps), "fps"));
            }
            let delete = ui.button("Delete study");
            self.hits.insert("simulation:delete_study".into(), delete.rect);
            if delete.clicked() {
                *change = Some(Change::Remove { study: id.clone(), item: None });
            }
        });
    }

    /// Play/Pause, Stop, Loop, the playhead's time and the bake's progress.
    fn transport(&mut self, ui: &mut egui::Ui, study: &Study, playback: &Value, change: &mut Option<Change>) {
        let this_study = playback["study"].as_str() == Some(study.id.as_str());
        let playing = this_study && playback["playing"] == true;
        let looping = this_study && playback["looping"] == true;
        let time = if this_study { playback["time"].as_f64().unwrap_or(0.0) } else { 0.0 };
        ui.horizontal_wrapped(|ui| {
            let play = ui.button(if playing { "Pause" } else { "▶ Play" });
            self.hits.insert("simulation:play".into(), play.rect);
            if play.clicked() {
                *change = Some(if playing { Change::Pause } else { Change::Play { study: study.id.clone() } });
            }
            let stop = ui.add_enabled(this_study, egui::Button::new("■ Stop"));
            self.hits.insert("simulation:stop".into(), stop.rect);
            if stop.clicked() {
                *change = Some(Change::Stop);
            }
            let loop_button = ui.selectable_label(looping, "Loop");
            self.hits.insert("simulation:loop".into(), loop_button.rect);
            if loop_button.clicked() {
                *change = Some(Change::Loop { study: study.id.clone(), looping: !looping });
            }
            let publish = ui
                .button(if study.motion.is_some() { "Publish again" } else { "Publish" })
                .on_hover_text("Bake the study to the end and keep its motion with the assembly, for a manufacturing-instructions step to play");
            self.hits.insert("simulation:publish".into(), publish.rect);
            if publish.clicked() {
                *change = Some(Change::Publish { study: study.id.clone() });
            }
            let frame = study.frame_at(time);
            let readout = ui.monospace(format!("{time:.2} s  frame {frame}/{}", study.frame_count().saturating_sub(1)));
            self.hits.insert("simulation:time".into(), readout.rect);
            if this_study {
                let (baked, total) = (playback["baked"].as_f64().unwrap_or(0.0), playback["total"].as_f64().unwrap_or(1.0).max(1.0));
                if baked < total {
                    let bar = ui.add(egui::ProgressBar::new((baked / total) as f32).desired_width(90.0).text(format!("baking {baked:.0}/{total:.0}")));
                    self.hits.insert("simulation:bake".into(), bar.rect);
                }
                if let Some(error) = playback["error"].as_str() {
                    let failed = playback["failed"].as_array().map_or(0, Vec::len);
                    let label = ui.colored_label(ui.visuals().warn_fg_color, format!("{failed} frame(s) the mates could not hold: {error}"));
                    self.hits.insert("simulation:failed".into(), label.rect);
                }
            }
            let mut check = study.interference.enabled;
            let box_ = ui.checkbox(&mut check, "Interference");
            self.hits.insert("simulation:field:interference".into(), box_.rect);
            if box_.changed() {
                *change = Some(patch(&study.id, None, "interference", json!({ "enabled": check }), ""));
            }
            if this_study {
                let clashing = playback["clashes"].as_array().map_or(0, Vec::len);
                if clashing > 0 {
                    let shown: Vec<String> = playback["shownClashes"]
                        .as_array()
                        .into_iter()
                        .flatten()
                        .filter_map(|pair| Some(format!("{} ↔ {}", pair.get(0)?.as_str()?, pair.get(1)?.as_str()?)))
                        .collect();
                    let text = if shown.is_empty() {
                        format!("{clashing} frame(s) with interference")
                    } else {
                        format!("{clashing} frame(s) with interference; here: {}", shown.join(", "))
                    };
                    let label = ui.colored_label(ui.visuals().error_fg_color, text);
                    self.hits.insert("simulation:interference".into(), label.rect);
                }
            }
        });
    }

    /// A time ruler over the study: a tick a second (or every 10 s on a
    /// long study), labelled; the baked extent, the playhead, a red mark on
    /// each frame where parts interfere and an amber one on each frame the
    /// mates could not hold; a press or drag on it scrubs. Then the frame
    /// count.
    fn ruler(&mut self, ui: &mut egui::Ui, study: &Study, playback: &Value, change: &mut Option<Change>) {
        let width = ui.available_width().max(120.0);
        let (rect, response) = ui.allocate_exact_size(egui::vec2(width, RULER_HEIGHT), egui::Sense::click_and_drag());
        self.hits.insert("simulation:ruler".into(), rect);
        let painter = ui.painter_at(rect);
        let visuals = ui.visuals();
        let stroke = visuals.widgets.noninteractive.fg_stroke;
        let x_of = |t: f64| rect.left() + rect.width() * (t / study.duration) as f32;
        let this_study = playback["study"].as_str() == Some(study.id.as_str());
        if this_study {
            let baked = playback["baked"].as_u64().unwrap_or(0) as usize;
            if baked > 0 {
                let baked_to = study.frame_time(baked - 1);
                painter.rect_filled(egui::Rect::from_min_max(rect.left_top(), egui::pos2(x_of(baked_to), rect.bottom())), 0.0, visuals.faint_bg_color);
            }
            for index in playback["clashes"].as_array().into_iter().flatten().filter_map(Value::as_u64) {
                let x = x_of(study.frame_time(index as usize));
                painter.line_segment([egui::pos2(x, rect.top() + 10.0), egui::pos2(x, rect.bottom())], egui::Stroke::new(2.0, visuals.error_fg_color));
            }
            for index in playback["failed"].as_array().into_iter().flatten().filter_map(Value::as_u64) {
                let x = x_of(study.frame_time(index as usize));
                painter.line_segment([egui::pos2(x, rect.top() + 4.0), egui::pos2(x, rect.bottom())], egui::Stroke::new(2.0, visuals.warn_fg_color));
            }
        }
        painter.line_segment([rect.left_bottom(), rect.right_bottom()], stroke);
        let step = if study.duration > 30.0 { 10.0 } else { 1.0 };
        let mut t = 0.0;
        while t <= study.duration + 1e-9 {
            let x = x_of(t);
            painter.line_segment([egui::pos2(x, rect.bottom() - 6.0), egui::pos2(x, rect.bottom())], stroke);
            painter.text(egui::pos2(x + 2.0, rect.top()), egui::Align2::LEFT_TOP, format!("{t:.0}s"), egui::FontId::proportional(10.0), stroke.color);
            t += step;
        }
        if this_study {
            let x = x_of(playback["time"].as_f64().unwrap_or(0.0));
            painter.line_segment([egui::pos2(x, rect.top()), egui::pos2(x, rect.bottom())], egui::Stroke::new(2.0, visuals.selection.stroke.color));
        }
        if response.is_pointer_button_down_on() || response.dragged() || response.clicked() {
            if let Some(pointer) = response.interact_pointer_pos() {
                let u = ((pointer.x - rect.left()) / rect.width()).clamp(0.0, 1.0) as f64;
                *change = Some(Change::Seek { study: study.id.clone(), time: u * study.duration });
            }
        }
        ui.weak(format!("{} frames at {} fps", study.frame_count(), study.fps));
    }

    /// A row per motor — the mate it drives and its drive (click: its form),
    /// its span and Remove; then + Motor.
    fn motors(&mut self, ui: &mut egui::Ui, study: &Study, mates: &[MotorMate], change: &mut Option<Change>) {
        ui.strong("Motors");
        if study.motors.is_empty() {
            ui.weak("None: + Motor drives a distance or angle mate.");
        }
        for motor in &study.motors {
            let key = format!("simulation:motor:{}", motor.id);
            let mate = mates.iter().find(|m| m.id == motor.constraint);
            let unit = mate.map_or("/s", |m| m.rate_unit());
            let label = match (&motor.drive, mate) {
                (Drive::Speed { speed }, Some(m)) => format!("{} ({}) at {speed} {unit}", m.id, m.kind),
                (Drive::Speed { speed }, None) => format!("{} (missing) at {speed} {unit}", motor.constraint),
                (Drive::Profile { start, end, .. }, Some(m)) => format!("{} ({}) {start} → {end} {}", m.id, m.kind, unit.trim_end_matches("/s")),
                (Drive::Profile { start, end, .. }, None) => format!("{} (missing) {start} → {end}", motor.constraint),
            };
            ui.horizontal_wrapped(|ui| {
                let row = ui.selectable_label(false, label).on_hover_text("Open the motor's fields");
                self.hits.insert(key.clone(), row.rect);
                if row.clicked() {
                    *change = Some(Change::Open(Some(motor.id.clone())));
                }
                ui.weak(format!("{:.2}–{:.2} s", motor.from, motor.to));
                let remove = ui.button("Remove");
                self.hits.insert(format!("{key}:remove"), remove.rect);
                if remove.clicked() {
                    *change = Some(Change::Remove { study: study.id.clone(), item: Some(motor.id.clone()) });
                }
            });
        }
        let add = ui.add_enabled(!mates.is_empty(), egui::Button::new("+ Motor"));
        self.hits.insert("simulation:add_motor".into(), add.rect);
        if add.clicked() {
            *change = Some(Change::AddMotor { study: study.id.clone(), mate: None });
        }
    }

    fn apply(&mut self, change: Change, engine: &mut EngineState) {
        self.notice = None;
        self.error = None;
        let result = match change {
            Change::Activate(id) => engine.simulation_set_active(&id).map(|()| None),
            Change::NewStudy => engine.simulation_add_study("").map(|id| Some(format!("New study {id}"))),
            Change::AddMotor { study, mate } => {
                let study = (!study.is_empty()).then_some(study);
                engine.simulation_add_motor(study.as_deref(), mate.as_deref()).map(|(_, id)| {
                    self.open = Some(id.clone());
                    Some(format!("Motor {id} added"))
                })
            }
            Change::Remove { study, item } => {
                // The open item's running pick goes with it.
                if item.is_some() && item == self.open && engine.ref_select_field().is_some_and(|f| f.target == RefSelectTarget::Simulation) {
                    engine.cancel_ref_select();
                }
                engine.simulation_remove(&study, item.as_deref()).map(|()| None)
            }
            Change::Patch { study, item, patch, coalesce } => engine.simulation_patch(&study, item.as_deref(), &patch, coalesce.as_deref()).map(|()| None),
            Change::Play { study } => engine.simulation_play(Some(&study), None).map(|_| None),
            Change::Pause => {
                engine.simulation_pause();
                Ok(None)
            }
            Change::Stop => {
                engine.simulation_stop();
                Ok(None)
            }
            Change::Loop { study, looping } => engine.simulation_set_looping(Some(&study), looping).map(|()| None),
            Change::Seek { study, time } => engine.simulation_seek(Some(&study), time).map(|()| None),
            Change::AddClip { study, kind } => engine.simulation_add_clip(Some(&study), &kind, None).map(|(_, id)| {
                self.open = Some(id.clone());
                Some(format!("Clip {id} added"))
            }),
            Change::Publish { study } => engine.simulation_publish(Some(&study)).map(|(_, frames)| Some(format!("Published: {frames} frames kept with the assembly"))),
            Change::Open(item) => {
                // Leaving a clip's form commits the pick running in it.
                if item.is_none() && engine.ref_select_field().is_some_and(|f| f.target == RefSelectTarget::Simulation) {
                    engine.finish_ref_select();
                }
                self.open = item;
                Ok(None)
            }
            Change::BeginRefSelect { clip, path, label, filter, multiple, seed } => {
                // Focus moves from one selection box to the next: a pick
                // already running commits first.
                if engine.ref_select_active() {
                    engine.finish_ref_select();
                }
                engine.begin_ref_select_for_simulation(&clip, path, label, filter, multiple, seed);
                Ok(None)
            }
            Change::FinishRefSelect => {
                engine.finish_ref_select();
                Ok(None)
            }
            Change::RefSelectRemove(index) => {
                engine.ref_select_remove(index);
                Ok(None)
            }
        };
        match result {
            Ok(notice) => self.notice = notice,
            Err(error) => self.error = Some(error),
        }
    }
}

/// A clip row's label: its kind and the span's reading of what it acts on.
fn clip_label(clip: &Clip) -> String {
    match &clip.body {
        ClipBody::Suppress { constraint } => format!("Suppress {constraint}"),
        ClipBody::Camera { views } => format!("Camera ({} views)", views.len()),
        ClipBody::Visibility { occurrences } => format!("Hide ({} parts)", occurrences.len()),
        ClipBody::Transparency { occurrences, .. } => format!("Transparency ({} parts)", occurrences.len()),
        ClipBody::Section { view, .. } => format!("Section {view}"),
        ClipBody::Explode { collapse, .. } => if *collapse { "Collapse".to_string() } else { "Explode".to_string() },
    }
}

/// A one-field patch; a non-empty `field` coalesces a typing run into one undo step.
fn patch(study: &str, item: Option<&str>, key: &str, value: Value, field: &str) -> Change {
    let mut fields = serde_json::Map::new();
    fields.insert(key.to_string(), value);
    Change::Patch {
        study: study.to_string(),
        item: item.map(str::to_string),
        patch: Value::Object(fields),
        coalesce: (!field.is_empty()).then(|| format!("simulation:{study}:{field}")),
    }
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "simulation", prefix: "panel:clip", meaning: "the visible region of the pane (scroll_into_view scrolls a simulation:* widget inside it)", command: None },
    HitKeyDoc { panel: "simulation", prefix: "simulation:error", meaning: "why the last action was refused", command: None },
    HitKeyDoc { panel: "simulation", prefix: "simulation:notice", meaning: "what the last action did", command: None },
    HitKeyDoc { panel: "simulation", prefix: "simulation:study:", meaning: "a study's row, by study id: make it the active one", command: Some("simulation_update") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:new_study", meaning: "+ Study: a new study, made active", command: Some("simulation_new_study") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:delete_study", meaning: "delete the active study", command: Some("simulation_remove") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:field:", meaning: "the active study's name, duration and fps fields, and the Interference checkbox (check every frame for parts that collide)", command: Some("simulation_update") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:interference", meaning: "how many baked frames have interfering parts, and which pairs clash at the shown frame", command: Some("simulation_playback") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:play", meaning: "Play the active study from its playhead, or Pause while it plays", command: Some("simulation_play") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:stop", meaning: "Stop: every part back at its document pose, the playhead at 0", command: Some("simulation_stop") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:loop", meaning: "Loop: repeat the study at its end", command: Some("simulation_play") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:time", meaning: "the playhead's time and frame", command: Some("simulation_playback") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:bake", meaning: "the bake's progress bar, while frames are still being solved", command: Some("simulation_playback") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:failed", meaning: "how many frames the mates could not hold, and the last reason", command: Some("simulation_playback") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:ruler", meaning: "the active study's time ruler (red marks: frames with interference; amber: frames the mates could not hold): press or drag on it to scrub to a time", command: Some("simulation_seek") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:motor:", meaning: "a motor's row, by motor id: its label (the mate and the drive; click opens the motor's form) and remove", command: Some("simulation_update") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:add_motor", meaning: "+ Motor: a motor on the first distance or angle mate, the whole study long; its form opens", command: Some("simulation_add_motor") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:clip:", meaning: "a clip's row, by clip id: its label (click opens the clip's form) and remove", command: Some("simulation_update") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:add_clip", meaning: "+ Clip: the menu of clip kinds (suppress, visibility, transparency, camera, section, explode), each added over the whole study on its natural target; its form opens", command: Some("simulation_add_clip") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:menuitem:clip_kind:", meaning: "+ Clip's entries, by kind", command: Some("simulation_add_clip") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:item:", meaning: "the open motor's or clip's form (the app's schema-driven dialog): form:feature (the title), form:return (back to the rows), field:<key> (a motor: constraint, from, to, mode, speed, shape, start, end, accel; a clip: from, to, and by kind constraint | occurrences | opacityFrom, opacityTo | view1..view4 | view, sweepEnabled, sweepFrom, sweepTo | view, collapse), field:<key>#<variant> (an open option menu's entries), field:occurrences#activate (Select, or Done while the pick runs), #line<i> and #x<i> (a picked part and its remove)", command: Some("simulation_update") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:publish", meaning: "Publish (or Publish again): bake the study to the end and write its motion into the assembly's simulation block, for a manufacturing-instructions step to attach", command: Some("simulation_publish") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:export:", meaning: "the Export row: format (GIF | MP4 | HTML), width, height, fps, the file path and run (a video renders every baked frame at that size; HTML writes the interactive player page)", command: Some("simulation_export") },
    HitKeyDoc { panel: "simulation", prefix: "simulation:menuitem:export_format:", meaning: "the export format menu's entries (gif, mp4, html)", command: Some("simulation_export") },
];

/// `__brepSimulation`: the engine's simulation state and the pane's.
pub fn state_json(engine: &EngineState, panel: &SimulationPanel) -> String {
    let mut state = engine.simulation_state_json();
    state["playback"] = engine.simulation_playback_json();
    state["pane"] = serde_json::from_str(&panel.state_json()).unwrap_or(Value::Null);
    state.to_string()
}

