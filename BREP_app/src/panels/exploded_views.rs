//! The Exploded views section at the foot of the Assembly Constraints pane.
//!
//! An exploded view is a PMI view whose explode rows are its ordered steps
//! (`brep_render::engine_state` `exploded_ops`), so this section is a front for
//! the PMI view and annotation calls the PMI pane makes, plus the assembly
//! ones: **New exploded view** (auto-explode, one step per top-level
//! component), **Show** / **Hide** (activate the view in place, without the PMI
//! workbench), **Explode** / **Collapse** (play the steps out or home) and a
//! progress slider. A step row selects the step and arms the move gizmo on it
//! (the same gizmo the PMI pane gives an open explode annotation); its trail
//! box and delete and up / down buttons edit the PMI rows.
use crate::automation::hit_keys::HitKeyDoc;
use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// Hit keys this section publishes, into the Constraints pane's `acon:` set.
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:header", meaning: "open or close the Exploded views section", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:new", meaning: "make a new exploded view (auto-explode)", command: Some("assembly_auto_explode") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:deep", meaning: "a new exploded view opens the sub-assemblies too", command: Some("assembly_auto_explode") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:spread", meaning: "how far a new exploded view spreads the components", command: Some("assembly_auto_explode") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:show:", meaning: "show or hide an exploded view (explode:show:<view id>)", command: Some("pmi_activate_view") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:play", meaning: "play the shown view's steps out", command: Some("exploded_animate") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:collapse", meaning: "bring the shown view's steps home", command: Some("exploded_animate") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:progress", meaning: "the shown view's progress slider", command: Some("exploded_set_progress") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:add", meaning: "add a step that moves the selected components (a part inside a sub-assembly picked from the BOM is one)", command: Some("exploded_add_step") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:step:", meaning: "a step's row: select it and move it with the gizmo (explode:step:<id>)", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:trace:", meaning: "a step's trail box", command: Some("pmi_update_annotation") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:up:", meaning: "move a step earlier", command: Some("pmi_move_annotation") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "explode:del:", meaning: "delete a step", command: Some("pmi_remove_annotation") },
];

/// The section's own UI state.
pub struct ExplodedSection {
    /// A new view's spread (multiples of half the assembly diagonal).
    spread: f64,
    /// A new view opens the sub-assemblies too (their parts separate).
    deep: bool,
    /// The exploded views as last read, and the history revision they were
    /// read at. Reading them parses the PMI block and names every step's
    /// part: twice a frame, that was ~12 ms of every frame the pane showed a
    /// 499-step view on l500.
    views: Option<(u64, std::rc::Rc<Vec<brep_render::engine_state::ExplodedView>>)>,
    /// The last refusal, shown under the buttons.
    message: Option<String>,
    /// The step row under the pointer: it lights the step's parts and shows
    /// its chip (a deep view draws chips only for the selected or hovered step).
    hovered: Option<String>,
}

impl Default for ExplodedSection {
    fn default() -> Self {
        Self { spread: 1.0, deep: false, views: None, message: None, hovered: None }
    }
}

/// How deep "into sub-assemblies" opens.
const DEEP: usize = 8;

enum Action {
    New,
    Add,
    Show(String),
    Hide,
    Animate(bool),
    Progress(f64),
    Select(String),
    Trace(String, bool),
    Up(String),
    Delete(String),
}

/// How fast Explode / Collapse play, in steps per second.
const STEPS_PER_SECOND: f64 = 1.5;

impl ExplodedSection {
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState, hits: &mut HashMap<String, egui::Rect>) {
        let revision = state.history.revision();
        let views = match &self.views {
            Some((at, views)) if *at == revision => views.clone(),
            _ => {
                let views = std::rc::Rc::new(state.exploded_views());
                self.views = Some((revision, views.clone()));
                views
            }
        };
        let active = state.pmi_active_view().map(str::to_string);
        let locked = state.history.locked().is_some();
        let mut action = None;
        let mut hovered = None;
        ui.add_space(6.0);
        let header = egui::CollapsingHeader::new(format!("Exploded views ({})", views.len()))
            .id_salt("explode:section")
            .show(ui, |ui| {
                ui.horizontal(|ui| {
                    let new = ui.add_enabled(!locked, egui::Button::new("New exploded view"));
                    hits.insert("explode:new".into(), new.rect);
                    if new.on_hover_text("One step per component, each moved straight away from the base component").clicked() {
                        action = Some(Action::New);
                    }
                    let spread = ui.add(egui::DragValue::new(&mut self.spread).range(0.1..=10.0).speed(0.05).prefix("spread ×"));
                    hits.insert("explode:spread".into(), spread.rect);
                    let deep = ui.checkbox(&mut self.deep, "into sub-assemblies");
                    hits.insert("explode:deep".into(), deep.rect);
                    deep.on_hover_text("Also take each sub-assembly apart, after the assembly's own components");
                });
                if views.is_empty() {
                    ui.weak("No exploded views yet. A PMI view with explode annotations is one too.");
                }
                for view in views.iter() {
                    let shown = active.as_deref() == Some(view.id.as_str());
                    ui.horizontal(|ui| {
                        let toggle = ui.selectable_label(shown, if shown { "◉ Hide" } else { "○ Show" });
                        hits.insert(format!("explode:show:{}", view.id), toggle.rect);
                        if toggle.clicked() {
                            action = Some(if shown { Action::Hide } else { Action::Show(view.id.clone()) });
                        }
                        ui.label(format!("{} · {} step{}", view.name, view.steps.len(), if view.steps.len() == 1 { "" } else { "s" }));
                    });
                    if !shown {
                        continue;
                    }
                    ui.horizontal(|ui| {
                        let play = ui.button("▶ Explode");
                        hits.insert("explode:play".into(), play.rect);
                        if play.clicked() {
                            action = Some(Action::Animate(true));
                        }
                        let collapse = ui.button("◀ Collapse");
                        hits.insert("explode:collapse".into(), collapse.rect);
                        if collapse.clicked() {
                            action = Some(Action::Animate(false));
                        }
                        let steps = state.exploded_step_count() as f64;
                        let mut progress = state.exploded_progress();
                        let slider = ui.add(egui::Slider::new(&mut progress, 0.0..=steps.max(1e-9)).show_value(false));
                        hits.insert("explode:progress".into(), slider.rect);
                        if slider.changed() {
                            action = Some(Action::Progress(progress));
                        }
                    });
                    let add = ui.add_enabled(!locked, egui::Button::new("+ Add step from selection"));
                    hits.insert("explode:add".into(), add.rect);
                    if add
                        .on_hover_text("Move the selected components as a new step, then drag its gizmo. A part inside a sub-assembly: pick its row in the BOM")
                        .clicked()
                    {
                        action = Some(Action::Add);
                    }
                    let selected = state.pmi_open_annotation().map(str::to_string);
                    for (n, step) in view.steps.iter().enumerate() {
                        ui.horizontal(|ui| {
                            let mut trace = step.trace;
                            let trace_box = ui.add_enabled(!locked, egui::Checkbox::new(&mut trace, ""));
                            hits.insert(format!("explode:trace:{}", step.id), trace_box.rect);
                            if trace_box.on_hover_text("Draw this step's trail").changed() {
                                action = Some(Action::Trace(step.id.clone(), trace));
                            }
                            let [x, y, z] = step.translate;
                            let row = ui.selectable_label(
                                selected.as_deref() == Some(step.id.as_str()),
                                format!("{}. {}  ({x:.1}, {y:.1}, {z:.1})", n + 1, step.labels.join(", ")),
                            );
                            hits.insert(format!("explode:step:{}", step.id), row.rect);
                            if row.hovered() {
                                hovered = Some(step.id.clone());
                            }
                            if row.on_hover_text("Select, then drag the gizmo to move this step").clicked() {
                                action = Some(Action::Select(step.id.clone()));
                            }
                            if n > 0 {
                                let up = ui.add_enabled(!locked, egui::Button::new("↑").small());
                                hits.insert(format!("explode:up:{}", step.id), up.rect);
                                if up.on_hover_text("Earlier").clicked() {
                                    action = Some(Action::Up(step.id.clone()));
                                }
                            }
                            let del = ui.add_enabled(!locked, egui::Button::new("✕").small());
                            hits.insert(format!("explode:del:{}", step.id), del.rect);
                            if del.on_hover_text("Delete this step").clicked() {
                                action = Some(Action::Delete(step.id.clone()));
                            }
                        });
                    }
                }
                if let Some(message) = &self.message {
                    ui.colored_label(ui.visuals().error_fg_color, message);
                }
            });
        hits.insert("explode:header".into(), header.header_response.rect);
        if hovered != self.hovered {
            match &hovered {
                Some(id) => state.pmi_hover(id),
                None => state.pmi_hover_end(),
            }
            self.hovered = hovered;
        }
        // The gizmo follows the selected step, as it follows an open explode
        // annotation in the PMI pane.
        let open_step = state
            .pmi_open_annotation()
            .filter(|id| views.iter().any(|v| active.as_deref() == Some(v.id.as_str()) && v.steps.iter().any(|s| s.id == *id)))
            .map(str::to_string);
        if let Some(id) = open_step {
            if !state.transform_armed_for(&id) {
                state.arm_transform(&id);
            }
            state.sync_transform_gizmo();
        }
        let Some(action) = action else { return };
        let result: Result<(), String> = match action {
            Action::New => state.assembly_auto_explode_deep(None, self.spread, if self.deep { DEEP } else { 1 }).map(|_| ()),
            Action::Add => state.exploded_add_step(&[]).map(|_| ()),
            Action::Show(id) => state.pmi_activate_view(&id),
            Action::Hide => {
                state.pmi_deactivate_view();
                Ok(())
            }
            // A deep view has many steps: the whole play stays about 4 s.
            Action::Animate(explode) => {
                let steps = state.exploded_step_count() as f64;
                state.exploded_animate(explode, STEPS_PER_SECOND.max(steps / 4.0))
            }
            Action::Progress(progress) => {
                state.exploded_set_progress(progress);
                Ok(())
            }
            Action::Select(id) => {
                state.pmi_set_annotation_open(Some(&id));
                Ok(())
            }
            Action::Trace(id, trace) => match state.pmi_state().find_annotation(&id).map(|(_, a)| a.params.clone()) {
                Some(mut params) => {
                    params["showTraceLine"] = serde_json::json!(trace);
                    state.pmi_update_annotation(&id, &params.to_string())
                }
                None => Err(format!("no step '{id}'")),
            },
            Action::Up(id) => {
                // Just before the explode row ahead of it, in the view's own order.
                let pmi = state.pmi_state();
                let index = pmi.find_annotation(&id).and_then(|(view, _)| {
                    let own = view.annotations.iter().position(|a| a.id() == id)?;
                    view.annotations[..own].iter().rposition(|a| a.kind == "explode")
                });
                drop(pmi);
                match index {
                    Some(index) => state.pmi_move_annotation(&id, index),
                    None => Ok(()),
                }
            }
            Action::Delete(id) => state.pmi_remove_annotation(&id),
        };
        self.message = result.err();
    }
}
