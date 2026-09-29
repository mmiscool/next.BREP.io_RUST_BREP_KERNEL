//! The PMI panel — the view TREE on the shared [`crate::column_tree`]
//! widget: one row per PMI view, its annotations as children, and — when an
//! annotation is open — that annotation's dialog through the SHARED
//! [`crate::form_view`] (the same function the feature and constraint
//! dialogs use; the schema comes from the kernel's `pmi_schema_catalogue`).
//!
//! # Tree mode
//!
//! Header: **Capture view** (snapshot the camera + visibility into a new,
//! active view), **+ Add annotation** (the nine types, enabled while a view
//! is active — creation is gated on an active view), the active view's
//! **Text size** (points) and a summary.
//!
//! Rows: a VIEW row shows its name (editable — rename in place), whether its
//! camera is captured, its annotation count and an **active** toggle
//! (activating applies the camera / visibility / wireframe / explode poses;
//! deactivating restores the modeling state); a click SELECTS it and leaves
//! the active view alone, and a double click activates it and opens the view's
//! own dialog; its menu offers Edit view, Update camera, Update visibility,
//! Wireframe on/off, Delete. An ANNOTATION row shows the type icon, its id,
//! its resolved text (the value + tolerance, the note, the callout …) or its
//! error, a status badge and an **enabled** toggle; a click SELECTS it (the
//! row and its label in the viewport carry the selection) and a double click
//! opens its form; its menu offers Edit, Move up / down and Delete — the same
//! menu a right-click on the annotation's label chip opens in the viewport
//! ([`annotation_actions`], [`run_row_action`]), and Delete is also the Delete
//! key on a selected annotation ([`delete_annotation`], driven by the app
//! shell). Hovering an annotation row highlights the geometry it references.
//!
//! # Form mode
//!
//! Which annotation is open is the ENGINE's `pmi_open_annotation` (a mode,
//! not model state): the viewport's label click and the context bar's
//! add-from-selection open one without going through here, so the mode is
//! read, never owned. Editing is LIVE (every change re-resolves); reference
//! fields use the engine's modal picker in its PMI flavour (vertex refs in
//! world coordinates); **Return to tree** closes the form.
//!
//! A VIEW's dialog is the engine's `pmi_open_view`, exclusive with the
//! annotation's: the view's name, its text size and wireframe, and the two
//! re-capture buttons, from `pmi_view_schema` like every other object form.
//!
//! Everything the panel knows comes from the engine (`pmi_state`,
//! `pmi_report`); this struct holds the widget's transient state only.

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree::{self, CellKind, ColumnLayout, ColumnSpec, ColumnTreeSpec, RowAction, RowNode};
use crate::form_view::{form_view, FormViewSpec};
use brep_render::brep_kernel::{pmi_schema_catalogue, pmi_type, PmiReport, PmiState, PmiStatus, PMI_TYPES};
use brep_render::engine_state::{pmi_view_params, pmi_view_schema, EngineState, PmiViewPatch};
use brep_render::features::form_fields_from_schema;
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

/// Who this panel is when it drives the viewport's dialog-row hover
/// (`EngineState::hover_entity_by_name`) — the owner tag that keeps its
/// highlight independent of the history form's and the Scene tree's.
const DIALOG_HOVER_OWNER: &str = "pmi";

const NAME: &str = "name";
const KIND: &str = "kind";
const VALUE: &str = "value";
const STATUS: &str = "status";
const ON: &str = "on";
const ACTIONS: &str = "actions";

const OK_COLOR: &str = "#3fb950";
const ERROR_COLOR: &str = "#f85149";
const ACTIVE_COLOR: &str = "#58a6ff";
const MUTED_COLOR: &str = "#8b949e";

/// Row-menu action ids.
const ACT_EDIT_VIEW: &str = "edit-view";
const ACT_ACTIVATE: &str = "activate";
const ACT_DEACTIVATE: &str = "deactivate";
const ACT_UPDATE_CAMERA: &str = "update-camera";
const ACT_UPDATE_VISIBILITY: &str = "update-visibility";
const ACT_WIREFRAME: &str = "wireframe";
const ACT_DELETE_VIEW: &str = "delete-view";
const ACT_EDIT: &str = "edit";
const ACT_UP: &str = "move-up";
const ACT_DOWN: &str = "move-down";
const ACT_DELETE: &str = "delete";

/// A deferred engine mutation (one per frame).
enum Action {
    Capture,
    Add(String),
    TextSize(String, f64),
    Rename(String, String),
    Activate(String),
    Deactivate,
    UpdateCamera(String),
    UpdateVisibility(String),
    Wireframe(String, bool),
    DeleteView(String),
    SetEnabled(String, bool),
    Open(Option<String>),
    /// A single click on an annotation row: select it, open nothing.
    Select(String),
    /// A single click on a view row: select it, activate nothing.
    SelectView(String),
    /// Open (or, with `None`, close) a VIEW's dialog.
    OpenView(Option<String>),
    /// An edited view form.
    UpdateView(String, Value),
    Move(String, usize),
    Delete(String),
    UpdateParams(String, Value),
    BeginRefSelect {
        id: String,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed: Vec<String>,
    },
}

/// Drive the engine for one deferred [`Action`]. A refusal comes back as the
/// engine's own message, which the caller turns into a notice.
fn apply(state: &mut EngineState, action: Action) -> Result<(), String> {
    match action {
        Action::Capture => {
            state.pmi_capture_view(None);
            Ok(())
        }
        Action::Add(type_id) => state.pmi_add_annotation(None, &type_id, "{}").map(|_| ()),
        Action::TextSize(id, size) => state.pmi_set_view_display(&id, &PmiViewPatch { text_size_pt: Some(size), ..Default::default() }),
        Action::Rename(id, name) => state.pmi_rename_view(&id, &name),
        Action::Activate(id) => state.pmi_activate_view(&id),
        Action::Deactivate => {
            state.pmi_deactivate_view();
            Ok(())
        }
        Action::UpdateCamera(id) => state.pmi_update_view_camera(&id),
        Action::UpdateVisibility(id) => state.pmi_update_view_visibility(&id),
        Action::Wireframe(id, on) => state.pmi_set_view_display(&id, &PmiViewPatch { wireframe: Some(on), ..Default::default() }),
        Action::DeleteView(id) => state.pmi_delete_view(&id),
        Action::SetEnabled(id, on) => state.pmi_set_annotation_enabled(&id, on),
        Action::Open(id) => {
            state.pmi_set_annotation_open(id.as_deref());
            Ok(())
        }
        Action::Select(id) => {
            state.pmi_select_annotation(&id);
            Ok(())
        }
        Action::SelectView(id) => {
            state.pmi_select_view(&id);
            Ok(())
        }
        Action::OpenView(id) => {
            state.pmi_set_view_open(id.as_deref());
            Ok(())
        }
        Action::UpdateView(id, params) => state.pmi_update_view(&id, &params.to_string()),
        Action::Move(id, index) => state.pmi_move_annotation(&id, index),
        Action::Delete(id) => state.pmi_remove_annotation(&id),
        Action::UpdateParams(id, params) => state.pmi_update_annotation(&id, &params.to_string()),
        Action::BeginRefSelect { id, path, label, filter, multiple, seed } => {
            state.begin_ref_select_for_pmi(&id, path, label, filter, multiple, seed);
            Ok(())
        }
    }
}

/// An ANNOTATION row's menu.
///
/// This is what the row's `⋯` opens, and what a right-click on the
/// annotation's label chip in the 3D view opens (`viewport/labels.rs`): one
/// list, so the two cannot offer different things.
pub(crate) fn annotation_actions() -> Vec<RowAction> {
    vec![
        RowAction::new(ACT_EDIT, "Edit annotation").tooltip("Open the annotation's dialog"),
        RowAction::new(ACT_UP, "Move up").tooltip("Move before the previous annotation"),
        RowAction::new(ACT_DOWN, "Move down").tooltip("Move after the next annotation"),
        RowAction::new(ACT_DELETE, "Delete annotation").separator_above().destructive(),
    ]
}

/// What a row-menu entry does: `action` is one of the ids a view row or an
/// annotation row declares, on the row `id`.
fn menu_action(pmi: &PmiState, id: String, action: &str) -> Option<Action> {
    match action {
        ACT_EDIT_VIEW => Some(Action::OpenView(Some(id))),
        ACT_ACTIVATE => Some(Action::Activate(id)),
        ACT_DEACTIVATE => Some(Action::Deactivate),
        ACT_UPDATE_CAMERA => Some(Action::UpdateCamera(id)),
        ACT_UPDATE_VISIBILITY => Some(Action::UpdateVisibility(id)),
        ACT_WIREFRAME => pmi.find_view(&id).map(|view| Action::Wireframe(id.clone(), !view.display.wireframe)),
        ACT_DELETE_VIEW => Some(Action::DeleteView(id)),
        ACT_EDIT => Some(Action::Open(Some(id))),
        ACT_UP => pmi.locate_annotation(&id).map(|(_, index)| Action::Move(id.clone(), index.saturating_sub(1))),
        ACT_DOWN => pmi.locate_annotation(&id).map(|(_, index)| Action::Move(id.clone(), index + 1)),
        ACT_DELETE => Some(Action::Delete(id)),
        _ => None,
    }
}

/// Run the menu entry `action` of the row standing for `id` — the dispatch
/// the tree's menu and the label chip's right-click menu share. An unknown
/// entry does nothing.
pub(crate) fn run_row_action(state: &mut EngineState, id: &str, action: &str) -> Result<(), String> {
    let pmi = state.pmi_state();
    menu_action(&pmi, id.to_string(), action).map_or(Ok(()), |action| apply(state, action))
}

/// Delete an annotation through its menu's destructive entry — the Delete key
/// on a selected label.
pub(crate) fn delete_annotation(state: &mut EngineState, id: &str) -> Result<(), String> {
    run_row_action(state, id, ACT_DELETE)
}

/// The panel's transient UI state.
pub struct PmiPanel {
    hits: HashMap<String, egui::Rect>,
    layout: ColumnLayout,
    columns: Vec<ColumnSpec>,
    /// Views collapsed in the tree (every view starts expanded).
    collapsed: HashSet<String>,
    hovered: Option<String>,
    /// A hover change the tree draw recorded, applied to the engine after
    /// the draw (`Some(None)` = the pointer left the annotation rows).
    pending_hover: Option<Option<String>>,
    /// Why the document may not change this frame (its forms draw disabled).
    locked: Option<String>,
}

impl Default for PmiPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl PmiPanel {
    pub fn new() -> Self {
        Self {
            hits: HashMap::new(),
            layout: ColumnLayout::default(),
            columns: vec![
                ColumnSpec::new(NAME, "View / annotation", CellKind::Text).width(150.0),
                ColumnSpec::new(KIND, "", CellKind::Badges).width(28.0),
                ColumnSpec::new(VALUE, "Value", CellKind::ReadOnly).width(150.0),
                ColumnSpec::new(STATUS, "", CellKind::Badges).width(28.0),
                ColumnSpec::new(ON, "On", CellKind::Toggle).width(30.0),
                ColumnSpec::new(ACTIONS, "", CellKind::Actions { label: "\u{22EF}".into() }).width(30.0),
            ],
            collapsed: HashSet::new(),
            hovered: None,
            pending_hover: None,
            locked: None,
        }
    }

    /// Draw the panel: the open annotation's form, else the view tree.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.locked = state.history.locked().map(str::to_string);
        self.hits.clear();
        self.hits.insert("pmi:panel:clip".into(), ui.clip_rect());
        let pmi = state.pmi_state();
        let report = state.pmi_report().cloned().unwrap_or_default();
        let active = state.pmi_active_view().map(String::from);
        let open = state.pmi_open_annotation().map(String::from);
        let open_view = state.pmi_open_view().map(String::from);
        // The ONE selected row: an annotation or a view (ids share a counter).
        let selected = state.pmi_selected_annotation().or(state.pmi_selected_view()).map(String::from);

        let mut action: Option<Action> = None;
        let mut close = false;
        // The entity a hovered reference LINE names, kept out of `action` (which
        // carries the ONE deferred mutation) — hovering coincides with anything.
        let mut hover: Option<String> = None;
        let open_annotation = open.as_deref().and_then(|id| pmi.find_annotation(id).map(|(_, annotation)| annotation.clone()));
        let open_view = open_view.as_deref().and_then(|id| pmi.find_view(id).cloned());
        match (open_annotation, open_view) {
            (Some(annotation), _) => self.show_form(ui, &annotation, &report, &mut action, &mut close, &mut hover),
            (None, Some(view)) => self.show_view_form(ui, &view, &mut action, &mut close),
            (None, None) => self.show_tree(ui, &pmi, &report, active.as_deref(), selected.as_deref(), &mut action),
        }

        let result = action.map_or(Ok(()), |action| apply(state, action));
        if let Err(error) = result {
            state.push_notice(format!("PMI: {error}"));
        }
        if close {
            // The pane shows one dialog, so whichever is up is the one closing.
            state.pmi_set_annotation_open(None);
            state.pmi_set_view_open(None);
        }
        if let Some(hover) = self.pending_hover.take() {
            match hover {
                Some(id) => state.pmi_hover(&id),
                None => state.pmi_hover_end(),
            }
        }
        // A hovered reference line lights the ENTITY it names in the 3D view (the
        // annotation-hover above lights the ANNOTATION — different things, own
        // slots). Applied every frame, including from the tree branch where it
        // ends what the form had lit.
        let hover_changed = match &hover {
            Some(name) => state.hover_entity_by_name(DIALOG_HOVER_OWNER, name),
            None => state.dialog_hover_end(DIALOG_HOVER_OWNER),
        };
        if hover_changed {
            // The viewport tile may have drawn BEFORE this pane in the dock.
            ui.ctx().request_repaint();
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn show_tree(
        &mut self,
        ui: &mut egui::Ui,
        pmi: &PmiState,
        report: &PmiReport,
        active: Option<&str>,
        selected: Option<&str>,
        action: &mut Option<Action>,
    ) {
        // --- header -------------------------------------------------------------
        ui.horizontal_wrapped(|ui| {
            let capture = ui
                .add(crate::icon_text::icon_button(ui, "\u{1F5CE} Capture view"))
                .on_hover_text("Snapshot the current camera and visibility into a new view and activate it");
            self.hits.insert("pmi:capture".into(), capture.rect);
            if capture.clicked() {
                *action = Some(Action::Capture);
            }
            let can_add = active.is_some();
            let mut add_type: Option<String> = None;
            ui.add_enabled_ui(can_add, |ui| {
                let combo = egui::ComboBox::from_id_salt("pmi-add")
                    .selected_text("+ Add annotation")
                    .show_ui(ui, |ui| {
                        for def in PMI_TYPES.iter() {
                            let item = crate::icon_text::selectable_icon_label(ui, false, def.long_name);
                            self.hits.insert(format!("pmi:add:{}", def.type_id), item.rect);
                            if item.clicked() {
                                add_type = Some(def.type_id.to_string());
                            }
                        }
                    });
                let response = if can_add {
                    combo.response
                } else {
                    combo.response.on_disabled_hover_text("Capture or activate a view first")
                };
                self.hits.insert("pmi:add".into(), response.rect);
            });
            if let Some(type_id) = add_type {
                *action = Some(Action::Add(type_id));
            }
            if let Some(view) = active.and_then(|id| pmi.find_view(id)) {
                let mut size = view.display.text_size_pt;
                let drag = ui
                    .add(egui::DragValue::new(&mut size).range(1.0..=288.0).speed(0.5).suffix(" pt"))
                    .on_hover_text("Label text size for the active view (1–288 pt)");
                self.hits.insert("pmi:textsize".into(), drag.rect);
                if drag.changed() {
                    *action = Some(Action::TextSize(view.id.clone(), size));
                }
            }
        });
        let annotation_count: usize = pmi.views.iter().map(|view| view.annotations.len()).sum();
        ui.label(
            egui::RichText::new(match active.and_then(|id| pmi.find_view(id)) {
                Some(view) => format!(
                    "{} view{} | {annotation_count} annotation{} | active: {}",
                    pmi.views.len(),
                    if pmi.views.len() == 1 { "" } else { "s" },
                    if annotation_count == 1 { "" } else { "s" },
                    view.name
                ),
                None => format!(
                    "{} view{} | {annotation_count} annotation{} | no active view",
                    pmi.views.len(),
                    if pmi.views.len() == 1 { "" } else { "s" },
                    if annotation_count == 1 { "" } else { "s" },
                ),
            })
            .weak(),
        );
        if active.is_none() {
            ui.label(egui::RichText::new("Capture a view to start annotating").weak().italics());
        }
        ui.add_space(2.0);

        // --- the tree ---------------------------------------------------------------
        let rows: Vec<RowNode> = pmi
            .views
            .iter()
            .map(|view| {
                let is_active = active == Some(view.id.as_str());
                let view_report = report.view(&view.id);
                let count = view.annotations.len();
                let mut row = RowNode::new(&view.id)
                    .cell(NAME, Value::String(view.name.clone()))
                    .cell(KIND, serde_json::json!([{ "glyph": "\u{1F441}", "color": if is_active { ACTIVE_COLOR } else { MUTED_COLOR }, "tooltip": "PMI view" }]))
                    .cell(
                        VALUE,
                        Value::String(format!(
                            "{} · {count} annotation{}",
                            match view.camera.as_ref().map(|c| &c.projection) {
                                Some(brep_render::brep_kernel::PmiProjection::Orthographic { .. }) => "orthographic",
                                Some(brep_render::brep_kernel::PmiProjection::Perspective { .. }) => "perspective",
                                None => "no camera",
                            },
                            if count == 1 { "" } else { "s" }
                        )),
                    )
                    .cell(
                        STATUS,
                        if is_active {
                            serde_json::json!([{ "glyph": "\u{25CF}", "color": ACTIVE_COLOR, "tooltip": "active view" }])
                        } else {
                            serde_json::json!([])
                        },
                    )
                    .cell(ON, Value::Bool(is_active))
                    .actions(vec![
                        RowAction::new(ACT_EDIT_VIEW, "Edit view").tooltip("Open the view's dialog: its name, text size and wireframe"),
                        if is_active {
                            RowAction::new(ACT_DEACTIVATE, "Deactivate view").tooltip("Restore the modeling camera and visibility")
                        } else {
                            RowAction::new(ACT_ACTIVATE, "Activate view").tooltip("Apply this view's camera, visibility and wireframe")
                        },
                        RowAction::new(ACT_UPDATE_CAMERA, "Update camera").tooltip("Re-capture the camera from the current viewpoint"),
                        RowAction::new(ACT_UPDATE_VISIBILITY, "Update visibility").tooltip("Re-capture which objects are hidden"),
                        RowAction::new(ACT_WIREFRAME, if view.display.wireframe { "Wireframe off" } else { "Wireframe on" }),
                        RowAction::new(ACT_DELETE_VIEW, "Delete view").tooltip("Delete the view and its annotations").separator_above().destructive(),
                    ]);
                row.expanded = !self.collapsed.contains(&view.id);
                row.selected = selected == Some(view.id.as_str());
                row.children = view
                    .annotations
                    .iter()
                    .enumerate()
                    .map(|(index, annotation)| {
                        let id = annotation.id().to_string();
                        let resolved = view_report.and_then(|v| v.annotations.iter().find(|r| r.id == id));
                        let def = pmi_type(&annotation.kind);
                        let (status_glyph, status_color, tooltip, text) = match resolved {
                            Some(row) if row.status == PmiStatus::Ok => ("\u{2713}", OK_COLOR, "resolved".to_string(), row.text.replace('\n', " / ")),
                            Some(row) => ("\u{2715}", ERROR_COLOR, row.message.clone(), row.message.clone()),
                            None => ("\u{2013}", MUTED_COLOR, "not resolved yet".to_string(), String::new()),
                        };
                        let mut child = RowNode::new(&id)
                            .cell(NAME, Value::String(id.clone()))
                            .cell(
                                KIND,
                                serde_json::json!([{ "glyph": def.map(|d| d.icon).unwrap_or("?"), "color": if annotation.enabled { ACTIVE_COLOR } else { MUTED_COLOR }, "tooltip": def.map(|d| d.label).unwrap_or(annotation.kind.as_str()) }]),
                            )
                            .cell(VALUE, Value::String(text))
                            .cell(STATUS, serde_json::json!([{ "glyph": status_glyph, "color": status_color, "tooltip": tooltip }]))
                            .cell(ON, Value::Bool(annotation.enabled))
                            .actions(annotation_actions());
                        child.selected = selected == Some(id.as_str());
                        let _ = index;
                        child
                    })
                    .collect();
                row
            })
            .collect();
        let spec = ColumnTreeSpec {
            id: "pmi-views",
            columns: &self.columns,
            root_label: Some("PMI Views"),
            root_cells: None,
            empty_hint: Some("(no views — Capture view to snapshot the camera and start annotating)"),
            hits_prefix: "pmi:",
        };
        let out = column_tree::column_tree(ui, &spec, &mut self.layout, &rows, Some(&mut self.hits));

        // --- hover → viewport highlight (applied by `show` after the draw) ---------
        if out.hovered != self.hovered {
            self.hovered = out.hovered.clone();
            self.pending_hover = Some(out.hovered.clone().filter(|id| pmi.find_annotation(id).is_some()));
        }

        // --- act on what the widget reported (at most one mutation) ------------------
        if let Some(id) = &out.toggled {
            if pmi.find_view(id).is_some() {
                if !self.collapsed.remove(id) {
                    self.collapsed.insert(id.clone());
                }
            }
        }
        if let Some(click) = out.actions.first() {
            *action = menu_action(pmi, click.row_id.clone(), &click.action);
            return;
        }
        if let Some(edit) = out.edits.first() {
            let id = edit.row_id.clone();
            match edit.column.as_str() {
                NAME if pmi.find_view(&id).is_some() => {
                    *action = Some(Action::Rename(id, edit.value.as_str().unwrap_or("").to_string()));
                }
                ON if pmi.find_view(&id).is_some() => {
                    *action = Some(if edit.value.as_bool().unwrap_or(false) { Action::Activate(id) } else { Action::Deactivate });
                }
                ON => {
                    *action = Some(Action::SetEnabled(id, edit.value.as_bool().unwrap_or(true)));
                }
                _ => {}
            }
            return;
        }
        // A DOUBLE click opens the row's dialog — a view's opens on the view, so
        // this is what ACTIVATES one; read first, because egui reports the
        // second press of a double click as a click too.
        if let Some(id) = &out.double_clicked {
            if pmi.find_annotation(id).is_some() {
                *action = Some(Action::Open(Some(id.clone())));
            } else if pmi.find_view(id).is_some() {
                *action = Some(Action::OpenView(Some(id.clone())));
            }
            return;
        }
        // A single click SELECTS the row and activates nothing: a view keeps
        // whichever view is up (its camera would otherwise jump on every
        // click), and an annotation brings only its own view up, so its label
        // is on screen to carry the selection.
        if let Some(id) = &out.clicked {
            if pmi.find_annotation(id).is_some() {
                *action = Some(Action::Select(id.clone()));
            } else if pmi.find_view(id).is_some() {
                *action = Some(Action::SelectView(id.clone()));
            }
        }
    }

    /// Draw ONE view's dialog through the shared form view: its name, text
    /// size and wireframe from [`pmi_view_schema`], and the two re-capture
    /// buttons its row menu also offers.
    fn show_view_form(
        &mut self,
        ui: &mut egui::Ui,
        view: &brep_render::brep_kernel::PmiView,
        action: &mut Option<Action>,
        close: &mut bool,
    ) {
        let schema = pmi_view_schema();
        let fields = form_fields_from_schema(&schema);
        let mut params = pmi_view_params(view);
        // Keyed by the id, never the name: the title scopes the form's widget
        // state, and the name is one of the fields being typed into.
        let title = format!("PMI view {}", view.id);
        let spec = FormViewSpec {
            title: &title,
            subtitle: Some(view.name.as_str()),
            fields: &fields,
            hidden: None,
            banner: None,
            trailing: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "pmi:",
            read_only: self.locked.as_deref(),
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("pmi:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("pmi:form:view:{}", view.id), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        if out.changed {
            // A view needs a name: a name field emptied on the way to a new one
            // writes nothing until there is a name again.
            if let Some(object) = params.as_object_mut() {
                if object.get("name").and_then(Value::as_str).is_some_and(|name| name.trim().is_empty()) {
                    object.remove("name");
                }
            }
            *action = Some(Action::UpdateView(view.id.clone(), params));
        }
        match out.button_clicked.as_deref() {
            Some("updateCamera") => *action = Some(Action::UpdateCamera(view.id.clone())),
            Some("updateVisibility") => *action = Some(Action::UpdateVisibility(view.id.clone())),
            _ => {}
        }
        if out.exit_clicked {
            *close = true;
        }
    }

    /// Draw ONE annotation's dialog through the shared form view.
    fn show_form(
        &mut self,
        ui: &mut egui::Ui,
        annotation: &brep_render::brep_kernel::PmiAnnotation,
        report: &PmiReport,
        action: &mut Option<Action>,
        close: &mut bool,
        hover: &mut Option<String>,
    ) {
        let catalogue = pmi_schema_catalogue();
        let Some(schema) = catalogue
            .as_array()
            .and_then(|entries| entries.iter().find(|entry| entry.get("type").and_then(Value::as_str) == Some(annotation.kind.as_str())))
            .cloned()
        else {
            *close = true;
            return;
        };
        let fields = form_fields_from_schema(&schema);
        let mut params = annotation.params.clone();
        let id = annotation.id().to_string();
        let def = pmi_type(&annotation.kind);
        let title = format!("{} {}", def.map(|d| d.label).unwrap_or(&annotation.kind), id);
        let (banner_text, banner_color) = match report.annotation(&id) {
            Some(row) if row.status == PmiStatus::Ok => (row.text.replace('\n', " / "), egui::Color32::from_rgb(0x3f, 0xb9, 0x50)),
            Some(row) => (row.message.clone(), egui::Color32::from_rgb(0xf8, 0x51, 0x49)),
            None => ("not resolved yet".to_string(), egui::Color32::GRAY),
        };
        let spec = FormViewSpec {
            title: &title,
            subtitle: None,
            fields: &fields,
            banner: Some((banner_text.as_str(), banner_color)),
            trailing: None,
            hidden: None,
            exit_label: "Return to tree",
            extra: None,
            rollback: false,
            hits_prefix: "pmi:",
            read_only: self.locked.as_deref(),
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        let anchor = self.hits.get("pmi:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("pmi:form:annotation:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        if let Some(activate) = out.ref_activate {
            *action = Some(Action::BeginRefSelect {
                id: id.clone(),
                path: activate.path,
                label: activate.label,
                filter: activate.filter,
                multiple: activate.multiple,
                seed: activate.seed,
            });
        }
        if out.changed {
            *action = Some(Action::UpdateParams(id.clone(), params));
        }
        if out.exit_clicked {
            *close = true;
        }
        // The entity a hovered reference line names — applied by `show`, which
        // holds the engine.
        *hover = out.hovered_entity;
    }

    /// The per-frame widget rects for the headed verifier.
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
    HitKeyDoc { panel: "pmi", prefix: "pmi:capture", meaning: "capture the current camera as a PMI view", command: Some("pmi_capture_view") },
    HitKeyDoc { panel: "pmi", prefix: "pmi:add", meaning: "open the add-annotation menu", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:add:", meaning: "add an annotation of that type", command: Some("pmi_add_annotation") },
    HitKeyDoc { panel: "pmi", prefix: "pmi:textsize", meaning: "the text size control", command: Some("pmi_set_view_display") },
    HitKeyDoc { panel: "pmi", prefix: "pmi:row:", meaning: "a view or annotation row (pmi:row:id) \u{2014} a click SELECTS the view or annotation (a view is not activated), a double click opens the view's dialog and activates it, or opens the annotation's dialog", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:cell:", meaning: "a row cell (pmi:cell:id:column) \u{2014} `pmi:cell:<view id>:on` is the toggle that ACTIVATES a view", command: Some("pmi_activate_view") },
    // The rest of `column_tree`'s key set under this panel's prefix. Undocumented
    // until 2026-09-13, because `hit_keys_check` had never run with the PMI pane
    // DRAWN — it is a dock tab, and a pane that is not the front one publishes
    // nothing at all. Found by the pmi verifier migration, the same shape as the
    // BOM's row-action keys.
    HitKeyDoc { panel: "pmi", prefix: "pmi:box:", meaning: "a row's collapse box (pmi:box:<row id>) \u{2014} a view's box folds its annotations away", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:col:", meaning: "a table column header (pmi:col:<field>) \u{2014} click to sort", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:grip:", meaning: "a column's resize grip (pmi:grip:<field>)", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:menu:", meaning: "a row's action-menu trigger cell (pmi:menu:<row id>); a right-click on the row opens the same menu", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:menuitem:", meaning: "one entry of the OPEN row action menu (pmi:menuitem:<row id>:<action>) \u{2014} published only while the menu is up", command: None },
    HitKeyDoc { panel: "pmi", prefix: "pmi:form:", meaning: "the open annotation or view form (pmi:form:annotation:id, pmi:form:view:id, pmi:form:feature, pmi:form:return)", command: Some("pmi_update_annotation") },
    // The form's own schema FIELDS, which are `form_view`'s keys under this
    // panel's prefix rather than the `pmi:form:` chrome. Undocumented until
    // 2026-09-13 for the same reason the rest of the pane's keys were: the
    // check had never run with an annotation form open.
    HitKeyDoc { panel: "pmi", prefix: "pmi:field:", meaning: "one control of the open annotation's or view's form (pmi:field:<path>, plus #activate on a reference field) \u{2014} a note's text, an anchor or plane picker; a view's `pmi:field:name`, `pmi:field:textSizePt`, `pmi:field:wireframe` and its `pmi:field:updateCamera` / `pmi:field:updateVisibility` buttons", command: Some("pmi_update_annotation") },
    HitKeyDoc { panel: "pmi", prefix: "pmi:panel:clip", meaning: "the visible region of the pane", command: None },
];
