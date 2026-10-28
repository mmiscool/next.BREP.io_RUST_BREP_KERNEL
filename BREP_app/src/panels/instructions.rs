//! The Instructions pane — the Instructions workbench's
//! ([`crate::workbench::instructions`]) step tree over the
//! document's `instructions` block, and the selected step's editor: its
//! title, parts, torque, tools, callouts, insertion arrows, notes and
//! sign-off — and the print settings the PDF follows (File ▸ Export writes it).
//! While a re-target waits (a new revision of the assembly came in through
//! Update components), the pane lists what changed and the steps it touched,
//! and Approve makes the new revision the target.
//!
//! The pane holds only transient widget state (its hit rects, the selected
//! step, an action a toolbar button handed it, the tool being typed). Every
//! record it draws is read from the engine each frame, and every edit goes
//! through the engine's instructions methods — the same ones the
//! `instructions_*` automation commands call — so undo, save and reload come
//! from the document. A field being typed in is one undo step per field.

use crate::automation::hit_keys::HitKeyDoc;
use brep_instructions::edit;
use brep_instructions::model::{Arrow, Callout, CalloutKind, Instructions, Orientation, Step, Torque, TorqueSource, ToolUse};
use brep_render::engine_state::PublishedMotion;
use brep_render::engine_state::{EngineState, InstructionsRefField, RefSelectField, RefSelectTarget};
use brep_render::style::{FieldKind, FormField};
use std::collections::HashMap;

/// The owner tag of the hover this pane sets when the pointer rests on an
/// occurrence field's line (`EngineState::hover_entity_by_name`).
const DIALOG_HOVER_OWNER: &str = "instructions";

/// An action a toolbar button hands the pane: the pane performs it because
/// it owns the selection the action reads (where a new step goes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstructionsRequest {
    Wizard,
    NewStep,
}

/// Where an insertion arrow starts, as the pane offers it: the part comes in
/// from that side, travelling the opposite way.
const ARROW_SIDES: [(&str, [f64; 3]); 6] = [
    ("from above (+Z)", [0.0, 0.0, -1.0]),
    ("from below (−Z)", [0.0, 0.0, 1.0]),
    ("from +X", [-1.0, 0.0, 0.0]),
    ("from −X", [1.0, 0.0, 0.0]),
    ("from +Y", [0.0, -1.0, 0.0]),
    ("from −Y", [0.0, 1.0, 0.0]),
];

/// The default distance an insertion arrow starts from, in model units.
const ARROW_DISTANCE: f64 = 30.0;

#[derive(Default)]
pub struct InstructionsPanel {
    /// The assembly's studies with a published motion, read each frame
    /// (`instructions_published_motions`), for the step editor's Motion row.
    published: Vec<PublishedMotion>,
    hits: HashMap<String, egui::Rect>,
    /// The pane's clip rect this frame: every published rect is cut to it,
    /// so a script's click on a widget that runs past the pane's edge (an
    /// action row wider than a narrow column) lands on the part on screen.
    clip: Option<egui::Rect>,
    selected: Option<String>,
    request: Option<InstructionsRequest>,
    /// The tool being typed before Add tool: name and spec.
    tool_draft: (String, String),
    notice: Option<String>,
    error: Option<String>,
    /// The occurrence pick running in this pane's step editor this frame, if
    /// one is (an arrow's or a callout's field), read before drawing.
    picking: Option<RefSelectField>,
    /// The occurrence (or solid) a field's line is hovered on this frame.
    hover: Option<String>,
}

/// What a BOM row, or a part of the "not yet planned" list, carries while
/// it is dragged: the occurrences it stands for. Dropped on a step's row,
/// they join that step.
#[derive(Clone, Debug)]
pub struct PartsDrag {
    pub occurrences: Vec<String>,
    pub part: String,
}

/// A step's row being dragged: dropped on another step's row, it nests
/// under that step, its children with it; dropped on the root, it returns
/// to the top level.
#[derive(Clone, Debug)]
struct StepDrag(String);

/// One engine edit the frame asks for, applied after drawing (the list and
/// the editor read the block while drawing).
enum Change {
    Show(String),
    Restructure(Restructure),
    /// Parts dropped on, or added to, a step.
    AddParts { step: String, occurrences: Vec<String> },
    /// A part taken out of its step: not planned any more.
    RemoveAdd { step: String, add: usize },
    /// A step moved under `parent` (`None`: the top level) at `index`.
    Nest { step: String, parent: Option<String>, index: usize },
    /// The viewport's camera becomes the step's, kept until reset.
    CaptureCamera(String),
    /// The step's camera is generated again.
    ResetView(String),
    Step { step: String, coalesce: Option<String>, apply: Box<dyn FnOnce(&mut Step)> },
    /// A print setting: the page template or the front matter.
    Print(Box<dyn FnOnce(&mut Instructions)>),
    /// Approve the pending re-target.
    ApproveRetarget,
    /// Reject it: the one undo that takes the revision back.
    RejectRetarget,
    /// An occurrence field's Select: pick the part in the viewport, seeded
    /// with the occurrence the field holds.
    PickOccurrence { step: String, field: InstructionsRefField, seed: Option<String> },
    /// The focused occurrence field's Done.
    FinishPick,
    /// An ✕ on the focused occurrence field's running pick.
    RemovePick(usize),
    /// The study's published motion becomes the step's.
    AttachMotion { step: String, study: String },
}

enum Restructure {
    MoveStep { step: String, index: usize },
    MergeWithNext { step: String, next: String },
    Delete(String),
    Split { step: String, add: usize },
    MoveAdd { from: String, add: usize, to: String },
}

impl InstructionsPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Hand the pane an action; it performs it on the next frame it draws.
    pub fn request(&mut self, request: InstructionsRequest) {
        self.request = Some(request);
    }

    /// The selected step.
    pub fn selected(&self) -> Option<&str> {
        self.selected.as_deref()
    }

    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }

    /// What automation reads beside the engine's `instructions_state`.
    pub fn state_json(&self) -> String {
        let picking = self.picking.as_ref().and_then(|field| match field.target {
            RefSelectTarget::Instructions { field: InstructionsRefField::Arrow(index) } => Some(format!("arrow:{index}")),
            RefSelectTarget::Instructions { field: InstructionsRefField::Callout(index) } => Some(format!("callout:{index}")),
            _ => None,
        });
        let mut state = serde_json::json!({ "selected": self.selected, "notice": self.notice, "error": self.error });
        // Absent, not null, while no pick runs: a script asks `exists`.
        if let Some(picking) = picking {
            state["picking"] = serde_json::Value::String(picking);
        }
        state.to_string()
    }

    fn perform(&mut self, request: InstructionsRequest, engine: &mut EngineState) {
        self.notice = None;
        self.error = None;
        let result = match request {
            InstructionsRequest::Wizard => engine.instructions_run_wizard_when_built().map(|()| {
                self.selected = None;
                "The wizard proposed the steps".to_string()
            }),
            InstructionsRequest::NewStep => engine.instructions_new_step(self.selected.as_deref()).map(|id| {
                let _ = engine.instructions_show_step(&id);
                self.selected = Some(id);
                "New step: move parts into it from the other steps".to_string()
            }),
        };
        match result {
            Ok(notice) => self.notice = Some(notice),
            Err(error) => self.error = Some(error),
        }
    }

    pub fn show(&mut self, ui: &mut egui::Ui, engine: &mut EngineState) {
        self.hits.clear();
        self.clip = Some(ui.clip_rect());
        self.hits.insert("panel:clip".into(), ui.clip_rect());
        if !engine.is_instructions() {
            ui.label("Open an assembly and press New from assembly on the toolbar: the instructions open in a new tab.");
            return;
        }
        if let Some(request) = self.request.take() {
            self.perform(request, engine);
        }
        let block = match engine.instructions() {
            Ok(Some(block)) => block,
            Ok(None) => return,
            Err(error) => {
                ui.colored_label(ui.visuals().error_fg_color, format!("The instructions cannot be read: {error}"));
                return;
            }
        };
        if self.selected.as_deref().is_some_and(|id| block.step(id).is_none()) {
            self.selected = None;
        }
        let shown = engine.instructions_state_json()["shownStep"].as_str().map(str::to_string);
        if self.selected.is_none() {
            self.selected = shown.clone();
        }

        ui.horizontal_wrapped(|ui| {
            ui.label("For");
            ui.strong(&block.target.name);
            if engine.instructions_wizard_pending() {
                ui.weak("— the wizard runs once the assembly is built");
            }
        });
        if let Some(error) = &self.error {
            let label = ui.colored_label(ui.visuals().error_fg_color, error);
            self.hit("instructions:error", label.rect);
        }
        if let Some(notice) = &self.notice {
            let label = ui.weak(notice);
            self.hit("instructions:notice", label.rect);
        }
        ui.separator();

        let mut change = None;
        if block.retarget.is_some() {
            self.retarget(ui, &block, shown.as_deref(), &mut change);
            ui.separator();
        }
        self.print_settings(ui, &block, &mut change);
        ui.separator();
        let unplanned = engine.instructions_unplanned();
        self.published = engine.instructions_published_motions().unwrap_or_default();
        // The occurrence pick running in the step editor, drawn by its own
        // field (the form infrastructure's reference row) like a PMI form's.
        self.picking = engine.ref_select_field().filter(|field| matches!(field.target, RefSelectTarget::Instructions { .. }));
        self.hover = None;
        self.unplanned_list(ui, &unplanned, &mut change);
        self.step_list(ui, &block, shown.as_deref(), &mut change);
        if let Some(step_id) = self.selected.clone() {
            if let Some(step) = block.step(&step_id) {
                ui.separator();
                self.step_actions(ui, &block, step, &mut change);
                self.step_editor(ui, &block, step, &mut change);
            }
        }
        if let Some(change) = change {
            self.apply(change, engine);
        }
        // This pane drew the pick's field: the shell finishes an in-form pick
        // no form claims, so one never runs on with its field off screen.
        if let Some(field) = engine.ref_select_field() {
            if matches!(field.target, RefSelectTarget::Instructions { .. }) && self.selected.as_deref() == Some(field.subject.as_str()) {
                engine.ref_select_claim();
            }
        }
        // A hovered field line lights the part it names in the viewport: a
        // running pick's line is a solid, a stored line an occurrence.
        let hovered = self.hover.clone().and_then(|name| {
            if engine.scene.solid(&name).is_some() { Some(name) } else { engine.instructions_occurrence_solid(&name) }
        });
        let hover_changed = match hovered {
            Some(name) => engine.hover_entity_by_name(DIALOG_HOVER_OWNER, &name),
            None => engine.dialog_hover_end(DIALOG_HOVER_OWNER),
        };
        if hover_changed {
            ui.ctx().request_repaint();
        }
    }

    /// A step row's occurrence field — the form infrastructure's reference
    /// row, as a PMI anchor or a constraint's element is picked: Select
    /// starts a viewport pick of ONE solid, the focused field lists the
    /// running pick and its button is Done, a stored line's ✕ clears it.
    /// `key` is the row (`arrow:0`, `callout:1`); the field's rects are
    /// published under `instructions:<key>:occurrence` (`#activate`,
    /// `#line0`, `#x0`). Returns whether the stored line was cleared.
    fn occurrence_field(&mut self, ui: &mut egui::Ui, key: &str, step: &str, field: InstructionsRefField, occurrence: Option<&str>, change: &mut Option<Change>) -> bool {
        let spec = FormField {
            path: vec!["occurrence".into()],
            label: "Part".into(),
            group: String::new(),
            kind: FieldKind::Reference { filter: vec!["SOLID".into()], multiple: false },
        };
        let mut current = serde_json::json!({ "occurrence": occurrence });
        let running: Option<Vec<String>> = self
            .picking
            .as_ref()
            .filter(|f| f.subject == step && f.target == RefSelectTarget::Instructions { field })
            .map(|f| f.names.clone());
        let picking = running.as_ref().map(|names| crate::form::ActivePick { path: &spec.path, names });
        let mut probe = HashMap::new();
        let mut actions = crate::form::FieldActions::default();
        let (changed, rect) = crate::form::field_input_with(ui, &spec, &mut current, Some(&mut probe), &mut actions, picking);
        self.hit(format!("instructions:{key}:occurrence"), rect);
        for (probe_key, probe_rect) in probe {
            self.hit(format!("instructions:{key}:{probe_key}"), probe_rect);
        }
        if actions.clicked.is_some() {
            *change = Some(Change::PickOccurrence { step: step.to_string(), field, seed: occurrence.map(str::to_string) });
        } else if actions.pick_done {
            *change = Some(Change::FinishPick);
        } else if let Some(index) = actions.pick_remove {
            *change = Some(Change::RemovePick(index));
        }
        if let Some(name) = actions.hovered_entity {
            self.hover = Some(name);
        }
        changed
    }

    /// The page template and the front matter the PDF prints: steps per
    /// page, orientation and paper as dropdowns (`form::enum_dropdown`, the
    /// combo every panel enum draws), and whether the parts and tools lists
    /// print as checkboxes.
    fn print_settings(&mut self, ui: &mut egui::Ui, block: &Instructions, change: &mut Option<Change>) {
        let template = &block.template;
        let per_page_choices: Vec<(u8, String)> = brep_instructions::model::STEPS_PER_PAGE.iter().map(|n| (*n, n.to_string())).collect();
        let per_page_choices: Vec<(u8, &str, &str)> = per_page_choices.iter().map(|(n, s)| (*n, s.as_str(), s.as_str())).collect();
        let orientation_choices = [(Orientation::Portrait, "Portrait", "Portrait"), (Orientation::Landscape, "Landscape", "Landscape")];
        let paper_choices: Vec<(String, &str, &str)> = brep_instructions::layout::PAPER_SIZES.iter().map(|(key, _, _)| (key.to_string(), *key, *key)).collect();
        ui.horizontal_wrapped(|ui| {
            ui.label("Print");
            let mut probe: HashMap<String, egui::Rect> = HashMap::new();
            ui.label("Steps a page:");
            let mut per_page = template.steps_per_page;
            if crate::form::enum_dropdown(ui, "instructions:print:per_page", &per_page_choices, &mut per_page, Some(56.0), Some(&mut probe)).0 {
                *change = Some(Change::Print(Box::new(move |block| block.template.steps_per_page = per_page)));
            }
            ui.label("Orientation:");
            let mut orientation = template.orientation;
            if crate::form::enum_dropdown(ui, "instructions:print:orientation", &orientation_choices, &mut orientation, Some(100.0), Some(&mut probe)).0 {
                *change = Some(Change::Print(Box::new(move |block| block.template.orientation = orientation)));
            }
            ui.label("Paper:");
            let mut paper = template.paper.clone();
            if crate::form::enum_dropdown(ui, "instructions:print:paper", &paper_choices, &mut paper, Some(84.0), Some(&mut probe)).0 {
                *change = Some(Change::Print(Box::new(move |block| block.template.paper = paper)));
            }
            // Through `hit`, so the rects are cut to the pane like every other.
            for (key, rect) in probe {
                self.hit(key, rect);
            }
            for (key, label) in [("parts", "Parts list"), ("tools", "Tools list")] {
                let parts = key == "parts";
                let mut on = if parts { block.front.parts_list } else { block.front.tools_list };
                let check = ui.checkbox(&mut on, label);
                self.hit(format!("instructions:print:{key}"), check.rect);
                if check.changed() {
                    *change = Some(Change::Print(Box::new(move |block| {
                        if parts { block.front.parts_list = on } else { block.front.tools_list = on }
                    })));
                }
            }
        });
    }

    /// The pending re-target: the parts added, removed and replaced in the
    /// new revision, the steps they touched (a click shows one) and the steps
    /// left with no parts, which Approve deletes.
    fn retarget(&mut self, ui: &mut egui::Ui, block: &Instructions, shown: Option<&str>, change: &mut Option<Change>) {
        let Some(retarget) = &block.retarget else { return };
        let heading = ui.strong(format!("New revision of {}: review the changes", retarget.to.name));
        self.hit("instructions:retarget:heading", heading.rect);
        let groups = [("added", "Added", &retarget.added), ("removed", "Removed", &retarget.removed), ("replaced", "Replaced", &retarget.replaced)];
        for (key, label, changes) in groups {
            for (index, occurrence) in changes.iter().enumerate() {
                let text = match &occurrence.was {
                    Some(was) => format!("{label}: {} ({was} → {})", occurrence.occurrence, occurrence.part),
                    None => format!("{label}: {} ({})", occurrence.occurrence, occurrence.part),
                };
                let row = ui.label(text);
                self.hit(format!("instructions:retarget:{key}:{index}"), row.rect);
            }
        }
        let numbers: HashMap<&str, usize> = block.steps().enumerate().map(|(i, (_, s))| (s.id.as_str(), i + 1)).collect();
        ui.horizontal_wrapped(|ui| {
            ui.label("Steps changed:");
            if retarget.touched_steps.is_empty() {
                ui.weak("none");
            }
            for id in &retarget.touched_steps {
                let Some(number) = numbers.get(id.as_str()) else { continue };
                let empty = if retarget.empty_steps.contains(id) { " (no parts: deleted on approval)" } else { "" };
                let row = ui.selectable_label(self.selected.as_deref() == Some(id.as_str()), format!("{number}{empty}"));
                self.hit(format!("instructions:retarget:step:{id}"), row.rect);
                if row.clicked() {
                    self.selected = Some(id.clone());
                    if shown != Some(id.as_str()) {
                        *change = Some(Change::Show(id.clone()));
                    }
                }
            }
        });
        ui.horizontal(|ui| {
            let approve = ui.button("Approve the new revision");
            self.hit("instructions:retarget:approve", approve.rect);
            if approve.clicked() {
                *change = Some(Change::ApproveRetarget);
            }
            let reject = ui.button("Reject: take the revision back").on_hover_text(
                "The one undo that takes the new revision and these changes back together, so the instructions keep describing the revision they did",
            );
            self.hit("instructions:retarget:reject", reject.rect);
            if reject.clicked() {
                *change = Some(Change::RejectRetarget);
            }
        });
    }

    /// The assembly's parts in no step yet, each a drag source (drop it on a
    /// step's row) with a button for the selected step.
    fn unplanned_list(&mut self, ui: &mut egui::Ui, unplanned: &[(String, String)], change: &mut Option<Change>) {
        if unplanned.is_empty() {
            return;
        }
        ui.label(egui::RichText::new("Not yet planned").strong());
        ui.weak("Drag a part onto a step (from here or from the BOM), or add it to the selected step.");
        for (occurrence, part) in unplanned {
            ui.horizontal(|ui| {
                let row = ui.add(egui::Label::new(format!("{part}  ({occurrence})")).sense(egui::Sense::click_and_drag())).on_hover_cursor(egui::CursorIcon::Grab);
                row.dnd_set_drag_payload(PartsDrag { occurrences: vec![occurrence.clone()], part: part.clone() });
                if row.dragged() {
                    drag_label(ui, part);
                }
                self.hit(format!("instructions:unplanned:{occurrence}"), row.rect);
                if let Some(selected) = self.selected.clone() {
                    let button = ui.small_button("Add to selected step");
                    self.hit(format!("instructions:unplanned:{occurrence}:add"), button.rect);
                    if button.clicked() {
                        *change = Some(Change::AddParts { step: selected, occurrences: vec![occurrence.clone()] });
                    }
                }
            });
        }
        ui.separator();
    }

    fn step_list(&mut self, ui: &mut egui::Ui, block: &Instructions, shown: Option<&str>, change: &mut Option<Change>) {
        let root = ui
            .add(egui::Button::new(egui::RichText::new("Steps (top level)").strong()).min_size(egui::vec2(ui.available_width(), 0.0)))
            .on_hover_text("Drop a step here to move it and its children to the end of the top level");
        self.hit("instructions:root", root.rect);
        if root.dnd_hover_payload::<StepDrag>().is_some() {
            ui.painter().rect_stroke(root.rect, 2.0, ui.visuals().selection.stroke, egui::StrokeKind::Outside);
        }
        // Do not consume a BOM/part payload: only steps belong at the root.
        if egui::DragAndDrop::has_payload_of_type::<StepDrag>(ui.ctx()) {
            if let Some(dragged) = root.dnd_release_payload::<StepDrag>() {
                *change = Some(Change::Nest { step: dragged.0.clone(), parent: None, index: usize::MAX });
            }
        }
        if block.steps.is_empty() {
            ui.weak("No steps yet: press Wizard on the toolbar.");
            return;
        }
        let empty = edit::empty_steps(block);
        let numbers: HashMap<&str, usize> = block.steps().enumerate().map(|(i, (_, s))| (s.id.as_str(), i + 1)).collect();
        ui.indent("instructions-root", |ui| self.step_rows(ui, &block.steps, &numbers, &empty, shown, change));
    }

    /// The step tree: a step's row, then its children indented under it. The
    /// numbers are the reading order, in which the children come first.
    fn step_rows(
        &mut self,
        ui: &mut egui::Ui,
        steps: &[Step],
        numbers: &HashMap<&str, usize>,
        empty: &[String],
        shown: Option<&str>,
        change: &mut Option<Change>,
    ) {
        for step in steps {
            let number = numbers.get(step.id.as_str()).copied().unwrap_or(0);
            let mut text = format!("{number}. {}", step.title);
            if empty.contains(&step.id) {
                text.push_str("  (no parts)");
            }
            let selected = self.selected.as_deref() == Some(step.id.as_str());
            // The row is a drag source (drop it on another step to nest it)
            // and a drop target for parts and steps. Its own response
            // carries the drag: egui's drag-source helper lays a drag-only
            // widget over the row, which took the click away from it.
            let row = ui.add(egui::Button::selectable(selected, &text).sense(egui::Sense::click_and_drag()));
            row.dnd_set_drag_payload(StepDrag(step.id.clone()));
            if row.dragged() {
                drag_label(ui, &text);
            }
            self.hit(format!("instructions:step:{}", step.id), row.rect);
            if row.dnd_hover_payload::<PartsDrag>().is_some() || row.dnd_hover_payload::<StepDrag>().is_some_and(|d| d.0 != step.id) {
                ui.painter().rect_stroke(row.rect, 2.0, ui.visuals().selection.stroke, egui::StrokeKind::Outside);
            }
            // Taking a payload of one type drops a payload of another, so
            // ask which one is carried first.
            let (parts, steps_drag) = (
                egui::DragAndDrop::has_payload_of_type::<PartsDrag>(ui.ctx()),
                egui::DragAndDrop::has_payload_of_type::<StepDrag>(ui.ctx()),
            );
            if let Some(parts) = parts.then(|| row.dnd_release_payload::<PartsDrag>()).flatten() {
                *change = Some(Change::AddParts { step: step.id.clone(), occurrences: parts.occurrences.clone() });
            } else if let Some(dragged) = steps_drag.then(|| row.dnd_release_payload::<StepDrag>()).flatten() {
                if dragged.0 != step.id {
                    *change = Some(Change::Nest { step: dragged.0.clone(), parent: Some(step.id.clone()), index: usize::MAX });
                }
            } else if row.clicked() {
                self.selected = Some(step.id.clone());
                if shown != Some(step.id.as_str()) {
                    *change = Some(Change::Show(step.id.clone()));
                }
            }
            if !step.steps.is_empty() {
                ui.indent(&step.id, |ui| self.step_rows(ui, &step.steps, numbers, empty, shown, change));
            }
        }
    }

    fn step_actions(&mut self, ui: &mut egui::Ui, block: &Instructions, step: &Step, change: &mut Option<Change>) {
        let Some((steps, s)) = block.siblings(&step.id) else { return };
        ui.horizontal_wrapped(|ui| {
            let up = ui.add_enabled(s > 0, egui::Button::new("Move up"));
            self.hit("instructions:up", up.rect);
            if up.clicked() {
                *change = Some(Change::Restructure(Restructure::MoveStep { step: step.id.clone(), index: s - 1 }));
            }
            let down = ui.add_enabled(s + 1 < steps.len(), egui::Button::new("Move down"));
            self.hit("instructions:down", down.rect);
            if down.clicked() {
                *change = Some(Change::Restructure(Restructure::MoveStep { step: step.id.clone(), index: s + 1 }));
            }
            let previous = s.checked_sub(1).map(|p| steps[p].id.clone());
            let nest = ui
                .add_enabled(previous.is_some(), egui::Button::new("Nest under previous"))
                .on_hover_text("This step, and the steps under it, build the previous step's group: they come before it and it installs them");
            self.hit("instructions:nest_previous", nest.rect);
            if nest.clicked() {
                if let Some(previous) = previous {
                    *change = Some(Change::Nest { step: step.id.clone(), parent: Some(previous), index: usize::MAX });
                }
            }
            let parent = block.parent_id(&step.id).flatten().map(str::to_string);
            let unnest = ui.add_enabled(parent.is_some(), egui::Button::new("Unnest")).on_hover_text("Out of its group: a sibling of its parent, right after it");
            self.hit("instructions:unnest", unnest.rect);
            if unnest.clicked() {
                if let Some(parent) = parent {
                    let grandparent = block.parent_id(&parent).flatten().map(str::to_string);
                    let index = block.siblings(&parent).map(|(_, i)| i + 1).unwrap_or(usize::MAX);
                    *change = Some(Change::Nest { step: step.id.clone(), parent: grandparent, index });
                }
            }
            let next = steps.get(s + 1).map(|next| next.id.clone());
            let merge = ui.add_enabled(next.is_some(), egui::Button::new("Merge with next")).on_hover_text("The next step's parts join this one");
            self.hit("instructions:merge_next", merge.rect);
            if merge.clicked() {
                if let Some(next) = next {
                    *change = Some(Change::Restructure(Restructure::MergeWithNext { step: step.id.clone(), next }));
                }
            }
            let delete = ui
                .add_enabled(step.adds.is_empty() && step.steps.is_empty(), egui::Button::new("Delete"))
                .on_disabled_hover_text("Move this step's parts, and the steps under it, elsewhere first");
            self.hit("instructions:delete", delete.rect);
            if delete.clicked() {
                *change = Some(Change::Restructure(Restructure::Delete(step.id.clone())));
            }
        });
        ui.horizontal_wrapped(|ui| {
            let capture = ui.button("Capture camera").on_hover_text("The view keeps the camera as you see it now, through every edit, until Reset view");
            self.hit("instructions:capture_camera", capture.rect);
            if capture.clicked() {
                *change = Some(Change::CaptureCamera(step.id.clone()));
            }
            let reset = ui
                .add_enabled(step.camera_captured, egui::Button::new("Reset view"))
                .on_hover_text("Back to the generated camera: everything this step shows, from its parent step's direction")
                .on_disabled_hover_text("The camera is generated from the step's parts; Capture camera keeps your own");
            self.hit("instructions:reset_view", reset.rect);
            if reset.clicked() {
                *change = Some(Change::ResetView(step.id.clone()));
            }
            if step.camera_captured {
                ui.weak("captured camera");
            }
        });
    }

    fn step_editor(&mut self, ui: &mut egui::Ui, block: &Instructions, step: &Step, change: &mut Option<Change>) {
        let id = step.id.clone();
        let (steps, s) = block.siblings(&id).expect("a listed step");

        // --- title ------------------------------------------------------------
        let mut title = step.title.clone();
        ui.horizontal(|ui| {
            ui.label("Title");
            let field = ui.text_edit_singleline(&mut title);
            self.hit("instructions:field:title", field.rect);
            if field.changed() {
                *change = Some(step_change(&id, "title", move |step| step.title = title));
            }
        });

        // --- parts ------------------------------------------------------------
        ui.label(egui::RichText::new("Parts").strong());
        if step.adds.is_empty() {
            ui.weak("No parts: move some here from a neighbouring step.");
        }
        for (index, add) in step.adds.iter().enumerate() {
            ui.horizontal_wrapped(|ui| {
                let mut text = String::new();
                if !add.find_number.is_empty() {
                    text.push_str(&format!("[{}] ", add.find_number));
                }
                text.push_str(add.display_name());
                if add.quantity() > 1 {
                    text.push_str(&format!(" × {}", add.quantity()));
                }
                if add.subassembly {
                    text.push_str(" (sub-assembly)");
                }
                let label = ui.label(text);
                self.hit(format!("instructions:add:{index}"), label.rect);
                let unplan = ui.small_button("Unplan").on_hover_text("Out of every step: the part goes back to Not yet planned");
                self.hit(format!("instructions:add:{index}:unplan"), unplan.rect);
                if unplan.clicked() {
                    *change = Some(Change::RemoveAdd { step: id.clone(), add: index });
                }
                if let Some(previous) = s.checked_sub(1).map(|p| steps[p].id.clone()) {
                    let button = ui.small_button("To previous step");
                    self.hit(format!("instructions:add:{index}:previous"), button.rect);
                    if button.clicked() {
                        *change = Some(Change::Restructure(Restructure::MoveAdd { from: id.clone(), add: index, to: previous }));
                    }
                }
                if let Some(next) = steps.get(s + 1).map(|n| n.id.clone()) {
                    let button = ui.small_button("To next step");
                    self.hit(format!("instructions:add:{index}:next"), button.rect);
                    if button.clicked() {
                        *change = Some(Change::Restructure(Restructure::MoveAdd { from: id.clone(), add: index, to: next }));
                    }
                }
                if step.adds.len() > 1 {
                    let button = ui.small_button("Split out").on_hover_text("This part gets a step of its own, right after this one");
                    self.hit(format!("instructions:add:{index}:split"), button.rect);
                    if button.clicked() {
                        *change = Some(Change::Restructure(Restructure::Split { step: id.clone(), add: index }));
                    }
                }
            });
        }

        // --- torque -----------------------------------------------------------
        let mut torque = step.torque.as_ref().map(Torque::text).unwrap_or_default();
        ui.horizontal(|ui| {
            ui.label("Torque");
            let field = ui.add(egui::TextEdit::singleline(&mut torque).hint_text("e.g. 12 N·m").desired_width(120.0));
            self.hit("instructions:field:torque", field.rect);
            match step.torque.as_ref().map(|t| t.source) {
                Some(TorqueSource::Part) => {
                    ui.weak("from the part");
                }
                Some(TorqueSource::Override) => {
                    ui.weak("set on this step");
                }
                None => {}
            }
            if field.changed() {
                *change = Some(step_change(&id, "torque", move |step| {
                    step.torque = Torque::from_attribute(&torque).map(|t| Torque { source: TorqueSource::Override, ..t });
                }));
            }
        });

        // --- tools ------------------------------------------------------------
        ui.label(egui::RichText::new("Tools").strong());
        for (index, tool) in step.tools.iter().enumerate() {
            ui.horizontal(|ui| {
                let mut text = tool.name.clone();
                if !tool.spec.is_empty() {
                    text.push_str(&format!(" — {}", tool.spec));
                }
                if tool.unlinked {
                    text.push_str(" (not in the tools library)");
                }
                ui.label(text);
                let remove = ui.small_button("✕").on_hover_text("Remove this tool from the step");
                self.hit(format!("instructions:tool:{index}:remove"), remove.rect);
                if remove.clicked() {
                    *change = Some(step_change(&id, "", move |step| {
                        step.tools.remove(index);
                    }));
                }
            });
        }
        ui.horizontal_wrapped(|ui| {
            let name = ui.add(egui::TextEdit::singleline(&mut self.tool_draft.0).hint_text("Tool").desired_width(110.0));
            self.hit("instructions:tool:name", name.rect);
            let spec = ui.add(egui::TextEdit::singleline(&mut self.tool_draft.1).hint_text("Spec").desired_width(110.0));
            self.hit("instructions:tool:spec", spec.rect);
            let add = ui.add_enabled(!self.tool_draft.0.trim().is_empty(), egui::Button::new("Add tool"));
            self.hit("instructions:tool:add", add.rect);
            if add.clicked() {
                let (name, spec) = std::mem::take(&mut self.tool_draft);
                *change = Some(step_change(&id, "", move |step| {
                    step.tools.push(ToolUse { tool_id: None, name: name.trim().into(), spec: spec.trim().into(), unlinked: true });
                }));
            }
        });

        // --- callouts ---------------------------------------------------------
        ui.label(egui::RichText::new("Callouts").strong());
        for (index, callout) in step.callouts.iter().enumerate() {
            ui.horizontal(|ui| {
                ui.label(format!("{}:", callout.kind.label()));
                let mut text = callout.text.clone();
                let field = ui.text_edit_singleline(&mut text);
                self.hit(format!("instructions:callout:{index}:text"), field.rect);
                if field.changed() {
                    *change = Some(step_change(&id, &format!("callout:{index}"), move |step| {
                        if let Some(callout) = step.callouts.get_mut(index) {
                            callout.text = text;
                        }
                    }));
                }
                let remove = ui.small_button("✕");
                self.hit(format!("instructions:callout:{index}:remove"), remove.rect);
                if remove.clicked() {
                    *change = Some(step_change(&id, "", move |step| {
                        step.callouts.remove(index);
                    }));
                }
            });
            // The part the callout points at: picked in the viewport.
            if self.occurrence_field(ui, &format!("callout:{index}"), &id, InstructionsRefField::Callout(index), callout.occurrence.as_deref(), change) {
                *change = Some(step_change(&id, "", move |step| {
                    if let Some(callout) = step.callouts.get_mut(index) {
                        callout.occurrence = None;
                    }
                }));
            }
        }
        ui.horizontal(|ui| {
            for kind in [CalloutKind::Caution, CalloutKind::Note, CalloutKind::Check] {
                let button = ui.small_button(format!("+ {}", kind.label()));
                self.hit(format!("instructions:callout:add:{}", kind.label().to_lowercase()), button.rect);
                if button.clicked() {
                    *change = Some(Change::Step {
                        step: id.clone(),
                        coalesce: None,
                        apply: Box::new(move |step: &mut Step| step.callouts.push(Callout { id: String::new(), kind, text: String::new(), occurrence: None })),
                    });
                }
            }
        });

        // --- insertion arrows -------------------------------------------------
        ui.label(egui::RichText::new("Insertion arrows").strong());
        for (index, arrow) in step.arrows.iter().enumerate() {
            ui.horizontal_wrapped(|ui| {
                ui.label(format!("Arrow {}", index + 1));
                let side = ARROW_SIDES.iter().position(|(_, d)| same_direction(*d, arrow.direction));
                let mut picked = side;
                let combo = egui::ComboBox::from_id_salt(("instructions-arrow", index))
                    .selected_text(side.map_or("custom", |i| ARROW_SIDES[i].0))
                    .show_ui(ui, |ui| {
                        for (i, (label, _)) in ARROW_SIDES.iter().enumerate() {
                            let option = ui.selectable_value(&mut picked, Some(i), *label);
                            self.hit(format!("instructions:menuitem:arrow:{index}:{i}"), option.rect);
                        }
                    });
                self.hit(format!("instructions:arrow:{index}:side"), combo.response.rect);
                if picked != side {
                    if let Some(i) = picked {
                        let direction = ARROW_SIDES[i].1;
                        *change = Some(step_change(&id, "", move |step| {
                            if let Some(arrow) = step.arrows.get_mut(index) {
                                arrow.direction = direction;
                            }
                        }));
                    }
                }
                let mut distance = arrow.distance;
                let drag = ui.add(egui::DragValue::new(&mut distance).range(0.0..=10_000.0).suffix(" mm"));
                self.hit(format!("instructions:arrow:{index}:distance"), drag.rect);
                if drag.changed() {
                    *change = Some(step_change(&id, &format!("arrow:{index}"), move |step| {
                        if let Some(arrow) = step.arrows.get_mut(index) {
                            arrow.distance = distance;
                        }
                    }));
                }
                let remove = ui.small_button("✕");
                self.hit(format!("instructions:arrow:{index}:remove"), remove.rect);
                if remove.clicked() {
                    *change = Some(step_change(&id, "", move |step| {
                        step.arrows.remove(index);
                    }));
                }
            });
            // The part the arrow moves: one of the step's own, picked in the
            // viewport; clearing it removes the arrow (an arrow moves a part).
            if self.occurrence_field(ui, &format!("arrow:{index}"), &id, InstructionsRefField::Arrow(index), Some(&arrow.occurrence), change) {
                *change = Some(step_change(&id, "", move |step| {
                    if index < step.arrows.len() {
                        step.arrows.remove(index);
                    }
                }));
            }
        }
        // An arrow for the first occurrence of this step that has none.
        let free = step
            .adds
            .iter()
            .flat_map(|add| add.occurrences.iter())
            .find(|occurrence| !step.arrows.iter().any(|arrow| &arrow.occurrence == *occurrence))
            .cloned();
        let add = ui.add_enabled(free.is_some(), egui::Button::new("+ Arrow")).on_hover_text("The part slides in from above; pick the side and distance");
        self.hit("instructions:arrow:add", add.rect);
        if add.clicked() {
            if let Some(occurrence) = free {
                *change = Some(step_change(&id, "", move |step| {
                    step.arrows.push(Arrow { occurrence, direction: ARROW_SIDES[0].1, distance: ARROW_DISTANCE });
                }));
            }
        }

        // --- motion: a study's published motion plays in place of the slide ---
        ui.label(egui::RichText::new("Motion").strong());
        ui.horizontal_wrapped(|ui| {
            let attached = step.motion.as_ref().and_then(|m| m.get("study")).and_then(|v| v.as_str()).map(str::to_string);
            let frames = step.motion.as_ref().and_then(|m| m.get("frames")).and_then(|f| f.as_array()).map_or(0, Vec::len);
            let label = match &attached {
                Some(study) => {
                    let name = self.published.iter().find(|p| &p.study == study).map_or(study.as_str(), |p| p.name.as_str());
                    format!("{name} ({frames} frames)")
                }
                None if self.published.is_empty() => "None published: Publish a study in the assembly's Simulation pane".to_string(),
                None => "None".to_string(),
            };
            let mut picked: Option<String> = None;
            let combo = egui::ComboBox::from_id_salt("instructions-motion").selected_text(label).show_ui(ui, |ui| {
                for (index, motion) in self.published.iter().enumerate() {
                    let text = if motion.current { format!("{} ({} frames)", motion.name, motion.frames) } else { format!("{} (published before the assembly changed)", motion.name) };
                    let option = ui.add_enabled(motion.current, egui::Button::selectable(attached.as_deref() == Some(motion.study.as_str()), text));
                    self.hits.insert(format!("instructions:menuitem:motion:{index}"), option.rect);
                    if option.clicked() {
                        picked = Some(motion.study.clone());
                    }
                }
            });
            self.hits.insert("instructions:motion:study".into(), combo.response.rect);
            if let Some(study) = picked {
                *change = Some(Change::AttachMotion { step: id.clone(), study });
            }
            if attached.is_some() {
                let remove = ui.small_button("✕").on_hover_text("Take the motion off: the insertion arrows play again");
                self.hits.insert("instructions:motion:remove".into(), remove.rect);
                if remove.clicked() {
                    *change = Some(step_change(&id, "", |step| step.motion = None));
                }
            }
        });

        // --- notes and sign-off -----------------------------------------------
        ui.label(egui::RichText::new("Notes").strong());
        let mut notes = step.notes.clone();
        let field = ui.add(egui::TextEdit::multiline(&mut notes).desired_rows(3).desired_width(f32::INFINITY));
        self.hit("instructions:field:notes", field.rect);
        if field.changed() {
            *change = Some(step_change(&id, "notes", move |step| step.notes = notes));
        }
        let mut required = step.signoff.required;
        let check = ui.checkbox(&mut required, "Sign-off required on the shop floor");
        self.hit("instructions:field:signoff", check.rect);
        if check.changed() {
            *change = Some(step_change(&id, "", move |step| step.signoff.required = required));
        }
    }

    fn apply(&mut self, change: Change, engine: &mut EngineState) {
        self.error = None;
        self.notice = None;
        let result = match change {
            Change::Show(step) => engine.instructions_show_step(&step),
            Change::ApproveRetarget => engine.instructions_retarget_approve().map(|target| {
                self.notice = Some(format!("The instructions now describe the new revision of {}", target.name));
            }),
            Change::PickOccurrence { step, field, seed } => {
                // Focus moves from one selection box to the next: a pick
                // already running commits first.
                if engine.ref_select_active() {
                    engine.finish_ref_select();
                }
                engine.begin_ref_select_for_instructions(&step, field, seed.as_deref());
                Ok(())
            }
            Change::FinishPick => {
                engine.finish_ref_select();
                Ok(())
            }
            Change::RemovePick(index) => {
                engine.ref_select_remove(index);
                Ok(())
            }
            Change::RejectRetarget => engine.instructions_retarget_reject().map(|target| {
                self.notice = Some(format!("The new revision was taken back: the instructions still describe {} {}", target.name, target.revision).trim_end().to_string());
            }),
            Change::Print(apply) => engine.instructions_edit(None, |block| {
                apply(block);
                Ok(())
            }),
            Change::AddParts { step, occurrences } => engine
                .instructions_add_parts(&step, &occurrences)
                .map(|report| {
                    self.selected = Some(step.clone());
                    let numbers = engine.instructions().ok().flatten();
                    let number = |id: &str| numbers.as_ref().and_then(|b| b.step_number(id)).map(|n| n.to_string()).unwrap_or_else(|| id.to_string());
                    let mut text = match report.added {
                        0 => "No part joined the step".to_string(),
                        1 => "1 part joined the step".to_string(),
                        n => format!("{n} parts joined the step"),
                    };
                    if !report.left.is_empty() {
                        let left: Vec<String> = report.left.iter().map(|l| format!("{} is in step {}", l.occurrence, number(&l.step))).collect();
                        text.push_str(&format!("; left where it is: {}", left.join(", ")));
                    }
                    self.notice = Some(text);
                })
                .and_then(|()| engine.instructions_show_step(&step)),
            Change::RemoveAdd { step, add } => engine.instructions_remove_add(&step, add).and_then(|()| engine.instructions_show_step(&step)),
            Change::Nest { step, parent, index } => engine.instructions_nest_step(&step, parent.as_deref(), index).and_then(|()| engine.instructions_show_step(&step)),
            Change::CaptureCamera(step) => engine.instructions_capture_camera(&step).and_then(|()| engine.instructions_show_step(&step)),
            Change::ResetView(step) => engine.instructions_reset_view(&step).and_then(|()| engine.instructions_show_step(&step)),
            Change::Restructure(edit) => {
                let selected = &mut self.selected;
                engine
                    .instructions_restructure(|block| match edit {
                        Restructure::MoveStep { step, index } => edit::move_step(block, &step, index),
                        Restructure::MergeWithNext { step, next } => edit::merge(block, &[&step, &next]).map(drop),
                        Restructure::Delete(step) => {
                            edit::delete_step(block, &step)?;
                            *selected = None;
                            Ok(())
                        }
                        Restructure::Split { step, add } => edit::split(block, &step, &[add]).map(|new| *selected = Some(new)),
                        Restructure::MoveAdd { from, add, to } => edit::move_add(block, &from, add, &to),
                    })
                    .and_then(|()| match self.selected.clone() {
                        Some(step) => engine.instructions_show_step(&step),
                        None => Ok(()),
                    })
            }
            Change::AttachMotion { step, study } => engine.instructions_attach_motion(&step, Some(&study)).map(|_| ()),
            Change::Step { step, coalesce, apply } => engine.instructions_edit(coalesce.as_deref(), |block| {
                let target = block.step_mut(&step).ok_or_else(|| format!("no step '{step}'"))?;
                apply(target);
                // A callout just added takes the next callout id.
                if target.callouts.iter().any(|c| c.id.is_empty()) {
                    let fresh = block.next_id("MK");
                    if let Some(callout) = block.step_mut(&step).and_then(|s| s.callouts.iter_mut().find(|c| c.id.is_empty())) {
                        callout.id = fresh;
                    }
                }
                Ok(())
            }),
        };
        if let Err(error) = result {
            self.error = Some(error);
        }
    }
}

/// Every step's picture, PNG and SVG, in one zip: `step-01-MS2.png` and
/// `step-01-MS2.svg` in reading order. File ▸ Export and `instructions_export`
/// both write this.
pub fn images_zip(engine: &mut EngineState) -> Result<Vec<u8>, String> {
    let block = engine.instructions()?.ok_or("this document is not manufacturing instructions")?;
    let mut files: Vec<(String, Vec<u8>)> = Vec::new();
    for (index, (_, step)) in block.steps().enumerate() {
        let stem = format!("step-{:02}-{}", index + 1, step.id);
        files.push((format!("{stem}.png"), engine.instructions_step_png(&step.id, brep_render::engine_state::INSTRUCTIONS_IMAGE_PX)?));
        files.push((format!("{stem}.svg"), engine.instructions_step_svg(&step.id)?.into_bytes()));
    }
    if files.is_empty() {
        return Err("no steps yet: run the wizard first".into());
    }
    Ok(brep_ecad_core::fabrication::zip(files.iter().map(|(name, bytes)| (name.as_str(), bytes.as_slice()))))
}

/// A step field edit; `field` names the typing run that coalesces into one
/// undo step (empty: a one-shot edit, its own undo step).
/// What a dragged row shows at the pointer.
fn drag_label(ui: &egui::Ui, text: &str) {
    if let Some(pos) = ui.ctx().pointer_interact_pos() {
        egui::Area::new(egui::Id::new(("instructions-drag-label", text))).order(egui::Order::Tooltip).fixed_pos(pos + egui::vec2(12.0, 12.0)).show(ui.ctx(), |ui| {
            egui::Frame::popup(ui.style()).show(ui, |ui| {
                ui.label(text);
            });
        });
    }
}

impl InstructionsPanel {
    /// Publish a widget's rect for the headed verifier, cut to the pane's clip
    /// (what a click can land on); a widget wholly off screen is not published.
    fn hit(&mut self, key: impl Into<String>, rect: egui::Rect) {
        let visible = self.clip.map_or(rect, |clip| rect.intersect(clip));
        if visible.is_positive() {
            self.hits.insert(key.into(), visible);
        }
    }
}

fn step_change(step: &str, field: &str, apply: impl FnOnce(&mut Step) + 'static) -> Change {
    Change::Step {
        step: step.to_string(),
        coalesce: (!field.is_empty()).then(|| format!("instructions:{step}:{field}")),
        apply: Box::new(apply),
    }
}

fn same_direction(a: [f64; 3], b: [f64; 3]) -> bool {
    let length = |v: [f64; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    let (la, lb) = (length(a), length(b));
    la > 1e-12 && lb > 1e-12 && (0..3).all(|i| (a[i] / la - b[i] / lb).abs() < 1e-6)
}

pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "instructions", prefix: "panel:clip", meaning: "the visible region of the pane (scroll_into_view scrolls an instructions:* widget inside it)", command: None },
    HitKeyDoc { panel: "instructions", prefix: "instructions:error", meaning: "why the last action was refused", command: None },
    HitKeyDoc { panel: "instructions", prefix: "instructions:notice", meaning: "what the last action did", command: None },
    HitKeyDoc { panel: "instructions", prefix: "instructions:print:", meaning: "the print settings as dropdowns (form::enum_dropdown): per_page (items #1, #2, #4), orientation (#Portrait, #Landscape), paper (#A4 … #Tabloid, one list), and the parts and tools list checkboxes", command: Some("instructions_update") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:retarget:", meaning: "the pending re-target: heading, added:/removed:/replaced:<index> rows, step:<id> (show a step it changed), approve and reject (instructions_retarget_reject)", command: Some("instructions_retarget_approve") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:unplanned:", meaning: "a part of the assembly in no step yet, by occurrence (drag it onto a step's row), and its Add to selected step button (:add)", command: Some("instructions_add_parts") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:step:", meaning: "a step's row, by step id (a step's children are indented under it): select it and show its view; drop a BOM row or an unplanned part on it to add the parts, drop another step's row on it to nest that step under it; drag it to nest it elsewhere", command: Some("instructions_show_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:root", meaning: "the top-level Steps node: drop a step here to move it and its children to the end of the top level", command: Some("instructions_move_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:nest_previous", meaning: "nest the selected step under its previous sibling", command: Some("instructions_move_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:unnest", meaning: "move the selected step out of its group, right after its parent", command: Some("instructions_move_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:up", meaning: "move the selected step up among its siblings", command: Some("instructions_move_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:down", meaning: "move the selected step down among its siblings", command: Some("instructions_move_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:merge_next", meaning: "merge the next step into the selected one", command: Some("instructions_merge") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:delete", meaning: "delete the selected (empty) step", command: Some("instructions_delete_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:capture_camera", meaning: "the viewport's camera becomes the selected step's view camera, kept until Reset view", command: Some("instructions_capture_camera") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:reset_view", meaning: "the selected step's camera is generated again from its parts (enabled while a captured camera is kept)", command: Some("instructions_reset_view") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:field:", meaning: "the selected step's title, torque and notes fields and the sign-off checkbox", command: Some("instructions_update_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:add:", meaning: "a part of the selected step (by index) and its Unplan, To previous step, To next step and Split out buttons", command: Some("instructions_move_add") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:tool:", meaning: "the selected step's tools (remove by index) and the Tool, Spec and Add tool fields", command: Some("instructions_update_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:callout:", meaning: "the selected step's callouts (text and remove by index, and the part it points at: `<index>:occurrence` with `#activate` to pick a solid in the viewport, `#line0` the stored part and `#x0` to clear it) and the + Caution, + Note and + Check buttons", command: Some("instructions_update_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:arrow:", meaning: "the selected step's insertion arrows (side, distance and remove by index, and the part it moves: `<index>:occurrence` with `#activate` to pick one of the step's own solids in the viewport, `#line0` the part and `#x0` to remove the arrow) and + Arrow", command: Some("instructions_update_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:menuitem:arrow:", meaning: "an insertion arrow's side menu entries (arrow index, side index)", command: Some("instructions_update_step") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:motion:", meaning: "the selected step's Motion row: the study menu (:study; a published study of the assembly plays in place of the insertion arrows) and remove (:remove)", command: Some("instructions_attach_motion") },
    HitKeyDoc { panel: "instructions", prefix: "instructions:menuitem:motion:", meaning: "the Motion menu's entries, by published study index (a stale one is greyed)", command: Some("instructions_attach_motion") },
];
