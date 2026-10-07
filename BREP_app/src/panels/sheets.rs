//! The Sheets panel — the drawing sheets of the document as a TREE on the
//! shared [`crate::column_tree`] widget: one row per sheet, its placed views
//! as children, and — when a sheet or a placement is open — that object's
//! dialog through the SHARED [`crate::form_view`] (the same function the
//! feature, constraint and PMI annotation dialogs use; the schema comes from
//! `brep_render::sheets::schema_catalogue`, so neither form is hand-written).
//!
//! # Tree mode
//!
//! Header: **Add sheet** (a new sheet at the default paper size, opened in the
//! sheet viewport), **+ Place view** (one entry per saved PMI view — placing
//! is gated on having a sheet AND a view to place), and a summary.
//!
//! Rows: a SHEET row shows its name (editable — rename in place), its paper
//! size in millimetres, how many views it places and an **open** toggle (the
//! sheet viewport draws the open sheet; unticking returns to the 3D view); a
//! click opens it in the sheet viewport — the open sheet is the selected one —
//! and a double click opens the sheet's dialog; its menu offers Edit sheet and
//! Delete. A PLACED VIEW row shows its id, the saved view it places with its
//! scale, its position, whether its text is `flat` on the sheet or
//! `projected`, and its status — ✓ with the count of drawn edges and
//! annotations, or ✕ with the reason it did not project (a deleted view, a
//! camera this slice cannot draw).
//!
//! Every row a sheet HOLDS — a placement, a dimension, an ordinate set — is
//! picked the same way: a click SELECTS it (the row, and the object on the
//! paper, carry the selection) and a double click opens its form.
//!
//! Every row's menu is ALSO the right-click menu of its object on the paper
//! (`viewport/sheet.rs`), bare paper opening the sheet's: [`object_actions`]
//! is the one list and [`run_row_action`] the one dispatch, so the two cannot
//! drift. The Delete key removes the selected held object through its menu's
//! destructive entry ([`delete_held_object`], driven by the app shell).
//!
//! A SHEET DIMENSION row sits under the placements of the same sheet — a
//! dimension belongs to the PAPER, not to any one placement, so it is a child
//! of the sheet — and shows its id, what it measures (its alignment or its
//! kind), how many anchors it has and the VALUE it read, with ✓ or the reason
//! it did not resolve. Its form carries the kind, the alignment, the anchors as
//! reference rows, the offset and the precision; the rows' **Select** opens the
//! reference picker — the modal every dialog's reference rows open — whose
//! picks are anchors clicked on the paper, and **Finish** writes them here.
//! Every reference row of every sheet object works that way: an ordinate
//! set's datum and members, a section's cutting line, a detail's centre and
//! rim. A SECTION and a DETAIL have forms of their own for that reason
//! (`sectionView`, `detailView`): a plain placement has nothing to pick.
//!
//! The SHEET's dialog ends with a **Revisions** section: the sheet's revision
//! rows, each a letter, a date and a description edited in place, with a ✕
//! to remove a row and **+ Add revision** under them. A list of records is a
//! surface the schema vocabulary cannot express, so it is the form's one
//! consumer-drawn section (`FormViewSpec::extra`, the spline editor's anchor
//! list is the other) — every edit still goes through the engine's revision
//! doors, and the rows keep the order they were added in.
//!
//! **Flatten text** is a checkbox of the placement's own FORM
//! (`sheets:field:flattenText`, from the schema like every other field)
//! rather than a column of the tree: the shared column tree gates its editors
//! on the whole ROW, and a placement row is not editable — its name is its
//! id. The row says which way the toggle is set; the form is where it moves.
//!
//! # Form mode
//!
//! Which object is open is the ENGINE's `sheet_open_object` (a mode, not model
//! state) — a click on an object on the sheet viewport's paper opens one without going
//! through here, so the mode is read, never owned. Editing is LIVE and
//! **Return to tree** closes the form.
//!
//! Everything the panel knows comes from the engine (`sheet_state`,
//! `sheet_drawing`); this struct holds the widget's transient state only.

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree::{self, CellKind, ColumnLayout, ColumnSpec, ColumnTreeSpec, RowAction, RowNode};
use crate::form_view::{form_view, FormViewSpec};
use brep_render::engine_state::EngineState;
use brep_render::features::form_fields_from_schema;
use brep_render::sheets::SheetState;
use eframe::egui;
use serde_json::Value;
use std::collections::{HashMap, HashSet};

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
const ACT_OPEN: &str = "open";
const ACT_CLOSE: &str = "close";
const ACT_EDIT: &str = "edit";
const ACT_DELETE_SHEET: &str = "delete-sheet";
const ACT_DELETE_VIEW: &str = "delete-view";
const ACT_DELETE_DIM: &str = "delete-dimension";
const ACT_DELETE_ORD: &str = "delete-ordinate";

/// A deferred engine mutation (one per frame).
enum Action {
    AddSheet,
    PlaceView(String),
    Rename(String, String),
    Open(Option<String>),
    Edit(Option<String>),
    /// A single click on a placement, dimension or ordinate row.
    Select(String),
    DeleteSheet(String),
    DeleteView(String),
    DeleteDimension(String),
    UpdateSheet(String, Value),
    UpdateView(String, Value),
    UpdateDimension(String, Value),
    DeleteOrdinate(String),
    UpdateOrdinate(String, Value),
    /// A reference row's **Select** — a dimension's anchors, an ordinate set's
    /// datum or members, a section's cutting line, a detail's centre or rim:
    /// the reference picker, on the paper, the modal every dialog's reference
    /// rows open.
    BeginRefSelect {
        id: String,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed: Vec<String>,
    },
    /// The sheet dialog's Revisions section: append a row…
    AddRevision(String),
    /// …edit one field of row `n`…
    UpdateRevision(String, usize, Value),
    /// …or remove row `n`.
    RemoveRevision(String, usize),
}

/// What the Revisions section asked for this frame. Recorded by the drawer
/// (which runs inside the form view and holds no engine) and turned into an
/// [`Action`] once the form has drawn.
enum RevisionIntent {
    Add,
    Update(usize, &'static str, String),
    Remove(usize),
}

/// Drive the engine for one deferred [`Action`]. A refusal comes back as the
/// engine's own message, which the caller turns into a notice.
fn apply(state: &mut EngineState, action: Action) -> Result<(), String> {
    match action {
        Action::AddSheet => {
            state.sheet_add(None);
            Ok(())
        }
        Action::PlaceView(view) => state.sheet_place_view(None, &view, None, None).map(|_| ()),
        Action::Rename(id, name) => {
            state.sheet_update(&id, &serde_json::json!({ "name": name }).to_string())
        }
        Action::Open(id) => state.sheet_set_open(id.as_deref()),
        Action::Edit(id) => {
            state.sheet_set_object_open(id.as_deref());
            Ok(())
        }
        Action::Select(id) => {
            state.sheet_select_object(Some(&id));
            Ok(())
        }
        Action::DeleteSheet(id) => state.sheet_delete(&id),
        Action::DeleteView(id) => state.sheet_remove_view(&id),
        Action::DeleteDimension(id) => state.sheet_remove_dimension(&id),
        Action::UpdateSheet(id, params) => state.sheet_update(&id, &params.to_string()),
        Action::UpdateView(id, params) => state.sheet_update_view(&id, &params.to_string()),
        Action::UpdateDimension(id, params) => {
            state.sheet_update_dimension(&id, &params.to_string())
        }
        Action::DeleteOrdinate(id) => state.sheet_remove_ordinate(&id),
        Action::UpdateOrdinate(id, params) => {
            state.sheet_update_ordinate(&id, &params.to_string())
        }
        Action::BeginRefSelect { id, path, label, filter, multiple, seed } => {
            state.begin_ref_select_for_sheet(&id, path, label, filter, multiple, seed)
        }
        Action::AddRevision(id) => state.sheet_add_revision(&id, None, None, None).map(|_| ()),
        Action::UpdateRevision(id, index, params) => {
            state.sheet_update_revision(&id, index, &params.to_string())
        }
        Action::RemoveRevision(id, index) => state.sheet_remove_revision(&id, index),
    }
}

/// A SHEET row's menu. `is_open` flips the first entry between Open and
/// Close.
fn sheet_actions(is_open: bool) -> Vec<RowAction> {
    vec![
        if is_open {
            RowAction::new(ACT_CLOSE, "Close sheet").tooltip("Return the viewport to the 3D model")
        } else {
            RowAction::new(ACT_OPEN, "Open sheet").tooltip("Draw this sheet in the viewport")
        },
        RowAction::new(ACT_EDIT, "Edit sheet").tooltip("Name and paper size"),
        RowAction::new(ACT_DELETE_SHEET, "Delete sheet").separator_above().destructive(),
    ]
}

/// A PLACED VIEW row's menu — a plain placement, a section or a detail alike.
fn placement_actions() -> Vec<RowAction> {
    vec![
        RowAction::new(ACT_EDIT, "Edit placement").tooltip("Which view, where, at what scale, whether its text lies flat on the sheet, and — on a section or a detail — its letter"),
        RowAction::new(ACT_DELETE_VIEW, "Remove from sheet").separator_above().destructive(),
    ]
}

/// A SHEET DIMENSION row's menu.
fn dimension_actions() -> Vec<RowAction> {
    vec![
        RowAction::new(ACT_EDIT, "Edit dimension").tooltip("Kind, anchors, offset, precision and tolerance"),
        RowAction::new(ACT_DELETE_DIM, "Remove dimension").separator_above().destructive(),
    ]
}

/// An ORDINATE SET row's menu.
fn ordinate_actions() -> Vec<RowAction> {
    vec![
        RowAction::new(ACT_EDIT, "Edit ordinate set").tooltip("Axis, datum, members, baseline offset and precision"),
        RowAction::new(ACT_DELETE_ORD, "Remove ordinate set").separator_above().destructive(),
    ]
}

/// The menu of the tree row standing for `id` — a sheet, a placement, a
/// dimension or an ordinate set — or nothing for an id that names none.
///
/// This is what the row's `⋯` opens, and what a right-click on the same
/// object on the PAPER opens (`viewport/sheet.rs`): one list, so the two
/// cannot offer different things.
pub(crate) fn object_actions(sheets: &SheetState, open_sheet: Option<&str>, id: &str) -> Vec<RowAction> {
    if sheets.find_sheet(id).is_some() {
        sheet_actions(open_sheet == Some(id))
    } else if sheets.locate_view(id).is_some() {
        placement_actions()
    } else if sheets.locate_dimension(id).is_some() {
        dimension_actions()
    } else if sheets.locate_ordinate(id).is_some() {
        ordinate_actions()
    } else {
        Vec::new()
    }
}

/// What a row-menu entry does: `action` is one of the ids [`object_actions`]
/// declares, on the row `id`.
fn menu_action(id: String, action: &str) -> Option<Action> {
    match action {
        ACT_OPEN => Some(Action::Open(Some(id))),
        ACT_CLOSE => Some(Action::Open(None)),
        ACT_EDIT => Some(Action::Edit(Some(id))),
        ACT_DELETE_SHEET => Some(Action::DeleteSheet(id)),
        ACT_DELETE_VIEW => Some(Action::DeleteView(id)),
        ACT_DELETE_DIM => Some(Action::DeleteDimension(id)),
        ACT_DELETE_ORD => Some(Action::DeleteOrdinate(id)),
        _ => None,
    }
}

/// Run the menu entry `action` of the row standing for `id` — the dispatch
/// the tree's menu and the paper's right-click menu share. An unknown entry
/// does nothing.
pub(crate) fn run_row_action(state: &mut EngineState, id: &str, action: &str) -> Result<(), String> {
    menu_action(id.to_string(), action).map_or(Ok(()), |action| apply(state, action))
}

/// Delete what a sheet HOLDS — a placement, a dimension or an ordinate set —
/// through its own menu's destructive entry: the Delete key on a selected
/// object. A SHEET is never deleted this way (it takes everything placed on it
/// with it), and an id that names nothing a sheet holds refuses.
pub(crate) fn delete_held_object(state: &mut EngineState, id: &str) -> Result<(), String> {
    let sheets = state.sheet_state();
    let entry = if sheets.locate_view(id).is_some() {
        ACT_DELETE_VIEW
    } else if sheets.locate_dimension(id).is_some() {
        ACT_DELETE_DIM
    } else if sheets.locate_ordinate(id).is_some() {
        ACT_DELETE_ORD
    } else {
        return Err(format!("no placed view, dimension or ordinate set '{id}'"));
    };
    run_row_action(state, id, entry)
}

/// The panel's transient UI state.
pub struct SheetsPanel {
    hits: HashMap<String, egui::Rect>,
    layout: ColumnLayout,
    columns: Vec<ColumnSpec>,
    /// Sheets collapsed in the tree (every sheet starts expanded).
    collapsed: HashSet<String>,
}

impl Default for SheetsPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl SheetsPanel {
    pub fn new() -> Self {
        Self {
            hits: HashMap::new(),
            layout: ColumnLayout::default(),
            columns: vec![
                ColumnSpec::new(NAME, "Sheet / view", CellKind::Text).width(140.0),
                ColumnSpec::new(KIND, "", CellKind::Badges).width(28.0),
                ColumnSpec::new(VALUE, "Paper / placement", CellKind::ReadOnly).width(170.0),
                ColumnSpec::new(STATUS, "", CellKind::Badges).width(28.0),
                ColumnSpec::new(ON, "Open", CellKind::Toggle).width(36.0),
                ColumnSpec::new(ACTIONS, "", CellKind::Actions { label: "\u{22EF}".into() }).width(30.0),
            ],
            collapsed: HashSet::new(),
        }
    }

    /// Draw the panel: the open object's form, else the sheet tree.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();
        self.hits.insert("sheets:panel:clip".into(), ui.clip_rect());
        let sheets = state.sheet_state();
        let open_sheet = state.sheet_open().map(String::from);
        let open_object = state.sheet_open_object().map(String::from);
        let selected = state.sheet_selected_object().map(String::from);
        let view_ids = state.sheet_pmi_view_ids();

        let mut action: Option<Action> = None;
        let mut close = false;
        match open_object.as_deref() {
            Some(id) if sheets.find_sheet(id).is_some() => {
                let sheet = sheets.find_sheet(id).expect("just matched").clone();
                self.show_form(ui, state, &sheet.id, "Sheet", &sheet.params(), 0, None, &mut action, &mut close);
            }
            Some(id) if sheets.locate_view(id).is_some() => {
                let placed = sheets
                    .sheets
                    .iter()
                    .find_map(|sheet| sheet.find_view(id))
                    .expect("just located")
                    .clone();
                // A section and a detail are placements with forms of their
                // own: their picked references are rows there, and a plain
                // placement has none.
                let (label, index) = match placed.schema_type() {
                    "sectionView" => ("Section view", 4),
                    "detailView" => ("Detail view", 5),
                    _ => ("Placed view", 1),
                };
                self.show_form(ui, state, &placed.id, label, &placed.params(), index, None, &mut action, &mut close);
            }
            Some(id) if sheets.locate_dimension(id).is_some() => {
                let dimension = sheets.find_dimension(id).expect("just located").clone();
                // A dimension that writes a PMI dimension's tolerance block says
                // so at the top of its form: its own four fields read `none`,
                // and the value on the paper does not.
                let inherited = sheets
                    .locate_dimension(id)
                    .and_then(|(sheet, _)| state.sheet_drawing_cached(&sheet))
                    .and_then(|drawing| drawing.dimensions.iter().find(|drawn| drawn.id == dimension.id))
                    .filter(|drawn| !drawn.tolerance_from.is_empty())
                    .map(|drawn| {
                        format!(
                            "Tolerance from PMI dimension {}: {} \u{2014} set one here to override it",
                            drawn.tolerance_from, drawn.text
                        )
                    });
                self.show_form(ui, state, &dimension.id, "Sheet dimension", &dimension.params(), 2, inherited.as_deref(), &mut action, &mut close);
            }
            Some(id) if sheets.locate_ordinate(id).is_some() => {
                let set = sheets.find_ordinate(id).expect("just located").clone();
                self.show_form(ui, state, &set.id, "Ordinate set", &set.params(), 3, None, &mut action, &mut close);
            }
            // A stale mode (the object was deleted elsewhere) falls back to
            // the tree rather than an empty pane.
            _ => self.show_tree(ui, state, &sheets, open_sheet.as_deref(), selected.as_deref(), &view_ids, &mut action),
        }

        let result = action.map_or(Ok(()), |action| apply(state, action));
        if let Err(error) = result {
            state.push_notice(format!("Sheets: {error}"));
        }
        if close {
            state.sheet_set_object_open(None);
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn show_tree(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        sheets: &SheetState,
        open_sheet: Option<&str>,
        selected: Option<&str>,
        view_ids: &[String],
        action: &mut Option<Action>,
    ) {
        // --- header -------------------------------------------------------------
        ui.horizontal_wrapped(|ui| {
            let add = ui
                .add(crate::icon_text::icon_button(ui, "\u{1F5CB} Add sheet"))
                .on_hover_text("Add an A3 sheet and open it in the sheet viewport");
            self.hits.insert("sheets:add".into(), add.rect);
            if add.clicked() {
                *action = Some(Action::AddSheet);
            }
            let can_place = !sheets.sheets.is_empty() && !view_ids.is_empty();
            let mut place: Option<String> = None;
            ui.add_enabled_ui(can_place, |ui| {
                let combo = egui::ComboBox::from_id_salt("sheets-place")
                    .selected_text("+ Place view")
                    .show_ui(ui, |ui| {
                        for id in view_ids {
                            let item = crate::icon_text::selectable_icon_label(ui, false, id);
                            self.hits.insert(format!("sheets:place:{id}"), item.rect);
                            if item.clicked() {
                                place = Some(id.clone());
                            }
                        }
                    });
                let response = if can_place {
                    combo.response
                } else if sheets.sheets.is_empty() {
                    combo.response.on_disabled_hover_text("Add a sheet first")
                } else {
                    combo.response.on_disabled_hover_text("Capture a PMI view first")
                };
                self.hits.insert("sheets:place".into(), response.rect);
            });
            if let Some(view) = place {
                *action = Some(Action::PlaceView(view));
            }
        });
        let placed: usize = sheets.sheets.iter().map(|sheet| sheet.views.len()).sum();
        let dimensions: usize = sheets.sheets.iter().map(|sheet| sheet.dimensions.len()).sum();
        ui.label(
            egui::RichText::new(format!(
                "{} sheet{} | {placed} placed view{} | {dimensions} dimension{} | {}",
                sheets.sheets.len(),
                if sheets.sheets.len() == 1 { "" } else { "s" },
                if placed == 1 { "" } else { "s" },
                if dimensions == 1 { "" } else { "s" },
                match open_sheet.and_then(|id| sheets.find_sheet(id)) {
                    Some(sheet) => format!("open: {}", sheet.name),
                    None => "no sheet open".to_string(),
                }
            ))
            .weak(),
        );
        if sheets.sheets.is_empty() {
            ui.label(egui::RichText::new("Add a sheet to place PMI views on paper").weak().italics());
        }
        ui.add_space(2.0);

        // --- the tree ---------------------------------------------------------------
        // The OPEN sheet's projection is already computed for the viewport;
        // the rows read it for their per-placement status, and nothing else is
        // projected — a sheet is a hidden-line pass and the tree must not
        // trigger one per row.
        let drawing = open_sheet.and_then(|id| state.sheet_drawing(id));
        let rows: Vec<RowNode> = sheets
            .sheets
            .iter()
            .map(|sheet| {
                let is_open = open_sheet == Some(sheet.id.as_str());
                let (w, h) = sheet.millimetres();
                let count = sheet.views.len();
                let mut row = RowNode::new(&sheet.id)
                    .cell(NAME, Value::String(sheet.name.clone()))
                    .cell(KIND, serde_json::json!([{ "glyph": "\u{1F5CB}", "color": if is_open { ACTIVE_COLOR } else { MUTED_COLOR }, "tooltip": "drawing sheet" }]))
                    .cell(
                        VALUE,
                        Value::String(format!(
                            "{} · {w:.0} × {h:.0} mm · {count} view{}",
                            sheet.size,
                            if count == 1 { "" } else { "s" }
                        )),
                    )
                    .cell(
                        STATUS,
                        if is_open {
                            serde_json::json!([{ "glyph": "\u{25CF}", "color": ACTIVE_COLOR, "tooltip": "shown in the sheet viewport" }])
                        } else {
                            serde_json::json!([])
                        },
                    )
                    .cell(ON, Value::Bool(is_open))
                    .actions(sheet_actions(is_open));
                row.expanded = !self.collapsed.contains(&sheet.id);
                row.selected = is_open;
                row.children = sheet
                    .views
                    .iter()
                    .map(|placed| {
                        let projected = drawing.and_then(|d| d.views.iter().find(|v| v.id == placed.id));
                        let is_section = placed.section.as_ref();
                        let is_detail = placed.detail.as_ref();
                        let (glyph, color, tooltip, status) = match projected {
                            Some(view) if view.error.is_empty() => (
                                "\u{2713}",
                                OK_COLOR,
                                format!("{} edges, {} annotations", view.edges.len(), view.annotations.len()),
                                format!("{} edges", view.edges.len()),
                            ),
                            Some(view) => ("\u{2715}", ERROR_COLOR, view.error.clone(), view.error.clone()),
                            None => ("\u{2013}", MUTED_COLOR, "open the sheet to draw it".into(), String::new()),
                        };
                        let _ = status;
                        // A placement row carries no editable cell: its name
                        // IS its id, and the Open toggle belongs to the sheet.
                        let mut child = RowNode::new(&placed.id)
                            .cell(NAME, Value::String(placed.id.clone()))
                            .cell(KIND, match (is_section, is_detail) {
                                (Some(cut), _) => serde_json::json!([{ "glyph": "\u{25E7}", "color": ACTIVE_COLOR, "tooltip": format!("section {}—{} of {}", cut.label, cut.label, cut.source) }]),
                                (_, Some(circle)) => serde_json::json!([{ "glyph": "\u{25CE}", "color": ACTIVE_COLOR, "tooltip": format!("detail {} of {}", circle.label, circle.source) }]),
                                _ => serde_json::json!([{ "glyph": "\u{1F5CE}", "color": ACTIVE_COLOR, "tooltip": "placed PMI view" }]),
                            })
                            .cell(
                                VALUE,
                                Value::String(match (is_section, is_detail) {
                                    // A section's row says what it IS a section
                                    // of; its saved view is its source's, and
                                    // saying so twice tells a reader nothing.
                                    (Some(cut), _) => format!(
                                        "SECTION {}—{} of {} · {}:1 · ({:.0}, {:.0}) mm",
                                        cut.label,
                                        cut.label,
                                        cut.source,
                                        trim(placed.scale),
                                        placed.position[0],
                                        placed.position[1]
                                    ),
                                    // …and a detail's what it is a detail of,
                                    // at the scale that is the whole point of it.
                                    (_, Some(circle)) => format!(
                                        "DETAIL {} of {} · {}:1 · ({:.0}, {:.0}) mm",
                                        circle.label,
                                        circle.source,
                                        trim(placed.scale),
                                        placed.position[0],
                                        placed.position[1]
                                    ),
                                    _ => format!(
                                        "{} · {}:1 · ({:.0}, {:.0}) mm · {}",
                                        placed.view,
                                        trim(placed.scale),
                                        placed.position[0],
                                        placed.position[1],
                                        if placed.flatten_text { "flat" } else { "projected" }
                                    ),
                                }),
                            )
                            .cell(STATUS, serde_json::json!([{ "glyph": glyph, "color": color, "tooltip": tooltip }]))
                            .cell(ON, Value::Null)
                            .actions(placement_actions());
                        child.editable = false;
                        child.selected = selected == Some(placed.id.as_str());
                        child
                    })
                    .collect();
                // …then the sheet's OWN dimensions, under the placements they
                // anchor to. A dimension belongs to the PAPER, so it is a
                // child of the sheet and not of any one placement — including
                // the unresolved ones, which is where their reason is read.
                row.children.extend(sheet.dimensions.iter().map(|dimension| {
                    let drawn = drawing.and_then(|d| d.dimensions.iter().find(|x| x.id == dimension.id));
                    let (glyph, color, tooltip) = match drawn {
                        Some(d) if d.error.is_empty() => (
                            "\u{2713}",
                            OK_COLOR,
                            if d.tolerance_from.is_empty() {
                                format!("{} \u{2014} {} mm off its anchors", d.text, trim(d.offset_mm))
                            } else {
                                format!(
                                    "{} \u{2014} {} mm off its anchors; the tolerance is PMI dimension {}'s",
                                    d.text,
                                    trim(d.offset_mm),
                                    d.tolerance_from
                                )
                            },
                        ),
                        Some(d) => ("\u{2715}", ERROR_COLOR, d.error.clone()),
                        None => ("\u{2013}", MUTED_COLOR, "open the sheet to draw it".into()),
                    };
                    let measured = drawn
                        .filter(|d| d.error.is_empty())
                        .map(|d| match d.tolerance_from.as_str() {
                            "" => d.text.clone(),
                            from => format!("{} (from {from})", d.text),
                        })
                        .unwrap_or_else(|| "\u{2014}".into());
                    let mut child = RowNode::new(&dimension.id)
                        .cell(NAME, Value::String(dimension.id.clone()))
                        .cell(KIND, serde_json::json!([{ "glyph": "\u{2194}", "color": ACTIVE_COLOR, "tooltip": "sheet dimension" }]))
                        .cell(
                            VALUE,
                            Value::String(format!(
                                "{} \u{00B7} {} \u{00B7} {measured}",
                                if dimension.kind == "linear" {
                                    dimension.alignment.clone()
                                } else {
                                    dimension.kind.clone()
                                },
                                match dimension.anchors.len() {
                                    1 => "1 anchor".to_string(),
                                    n => format!("{n} anchors"),
                                }
                            )),
                        )
                        .cell(STATUS, serde_json::json!([{ "glyph": glyph, "color": color, "tooltip": tooltip }]))
                        .cell(ON, Value::Null)
                        .actions(dimension_actions());
                    child.editable = false;
                    child.selected = selected == Some(dimension.id.as_str());
                    child
                }));
                // …and one row per ORDINATE SET, beside the dimensions: a set
                // belongs to the paper too, and its row reads how many members
                // it draws and how many it has lost.
                row.children.extend(sheet.ordinates.iter().map(|set| {
                    let drawn = drawing.and_then(|d| d.ordinates.iter().find(|x| x.id == set.id));
                    let (glyph, color, tooltip) = match drawn {
                        Some(d) if !d.error.is_empty() => ("\u{2715}", ERROR_COLOR, d.error.clone()),
                        Some(d) if d.lost() > 0 => (
                            "\u{2715}",
                            ERROR_COLOR,
                            d.stations
                                .iter()
                                .filter(|station| !station.error.is_empty())
                                .map(|station| station.error.clone())
                                .collect::<Vec<_>>()
                                .join("; "),
                        ),
                        Some(d) => (
                            "\u{2713}",
                            OK_COLOR,
                            format!(
                                "{} stations \u{2014} baseline {} mm from the datum",
                                d.stations.len(),
                                trim(d.offset_mm)
                            ),
                        ),
                        None => ("\u{2013}", MUTED_COLOR, "open the sheet to draw it".into()),
                    };
                    let mut child = RowNode::new(&set.id)
                        .cell(NAME, Value::String(set.id.clone()))
                        .cell(KIND, serde_json::json!([{ "glyph": "\u{22EE}", "color": ACTIVE_COLOR, "tooltip": "ordinate set" }]))
                        .cell(
                            VALUE,
                            Value::String(format!(
                                "{} \u{00B7} {}",
                                set.axis,
                                match set.members.len() {
                                    1 => "1 member".to_string(),
                                    n => format!("{n} members"),
                                }
                            )),
                        )
                        .cell(STATUS, serde_json::json!([{ "glyph": glyph, "color": color, "tooltip": tooltip }]))
                        .cell(ON, Value::Null)
                        .actions(ordinate_actions());
                    child.editable = false;
                    child.selected = selected == Some(set.id.as_str());
                    child
                }));
                row
            })
            .collect();
        let spec = ColumnTreeSpec {
            id: "sheets",
            columns: &self.columns,
            root_label: Some("Sheets"),
            root_cells: None,
            empty_hint: Some("(no sheets — Add sheet to start a drawing)"),
            hits_prefix: "sheets:",
        };
        let out = column_tree::column_tree(ui, &spec, &mut self.layout, &rows, Some(&mut self.hits));

        // --- act on what the widget reported (at most one mutation) ------------------
        if let Some(id) = &out.toggled {
            if sheets.find_sheet(id).is_some() && !self.collapsed.remove(id) {
                self.collapsed.insert(id.clone());
            }
        }
        if let Some(click) = out.actions.first() {
            *action = menu_action(click.row_id.clone(), &click.action);
            return;
        }
        if let Some(edit) = out.edits.first() {
            let id = edit.row_id.clone();
            match edit.column.as_str() {
                NAME if sheets.find_sheet(&id).is_some() => {
                    *action = Some(Action::Rename(id, edit.value.as_str().unwrap_or("").to_string()));
                }
                ON if sheets.find_sheet(&id).is_some() => {
                    *action = Some(Action::Open(edit.value.as_bool().unwrap_or(false).then_some(id)));
                }
                _ => {}
            }
            return;
        }
        let held_by_a_sheet = |id: &str| {
            sheets.locate_view(id).is_some()
                || sheets.locate_dimension(id).is_some()
                || sheets.locate_ordinate(id).is_some()
        };
        // A DOUBLE click opens the row's dialog — a sheet's own included. Read
        // first: egui reports the second press of a double click as a click too.
        if let Some(id) = &out.double_clicked {
            if held_by_a_sheet(id) || sheets.find_sheet(id).is_some() {
                *action = Some(Action::Edit(Some(id.clone())));
            }
            return;
        }
        // A single click SELECTS: what a sheet holds becomes the selected
        // object, and a sheet becomes the open one — which is what selecting a
        // sheet means.
        if let Some(id) = &out.clicked {
            if held_by_a_sheet(id) {
                *action = Some(Action::Select(id.clone()));
            } else if sheets.find_sheet(id).is_some() && open_sheet != Some(id.as_str()) {
                *action = Some(Action::Open(Some(id.clone())));
            }
        }
    }

    /// Draw ONE sheet object's dialog through the shared form view.
    /// `catalogue_index` picks the schema: 0 = the sheet, 1 = a placement,
    /// 2 = a dimension, 3 = an ordinate set, 4 = a section, 5 = a detail.
    #[allow(clippy::too_many_arguments)]
    fn show_form(
        &mut self,
        ui: &mut egui::Ui,
        state: &EngineState,
        id: &str,
        label: &str,
        seed: &Value,
        catalogue_index: usize,
        note: Option<&str>,
        action: &mut Option<Action>,
        close: &mut bool,
    ) {
        let catalogue = state.sheet_catalogue();
        let Some(schema) = catalogue
            .get("objects")
            .and_then(Value::as_array)
            .and_then(|objects| objects.get(catalogue_index))
            .cloned()
        else {
            *close = true;
            return;
        };
        let fields = form_fields_from_schema(&schema);
        let mut params = seed.clone();
        let title = format!("{label} {id}");
        // The SHEET's revision rows: a list of records, drawn as the form's
        // own section. Intent-out like the rest of the form — the drawer edits
        // a copy and records what changed; the engine is driven below.
        let is_sheet = catalogue_index == 0;
        let revisions: Vec<brep_render::sheets::Revision> = if is_sheet {
            serde_json::from_value(seed.get("revisions").cloned().unwrap_or(Value::Null)).unwrap_or_default()
        } else {
            Vec::new()
        };
        let revision_rows = std::cell::RefCell::new(revisions);
        let revision_intents: std::cell::RefCell<Vec<RevisionIntent>> = std::cell::RefCell::new(Vec::new());
        let revision_hits: std::cell::RefCell<Vec<(String, egui::Rect)>> = std::cell::RefCell::new(Vec::new());
        let bom = std::cell::RefCell::new(serde_json::from_value::<Option<brep_render::sheets::BomTable>>(seed.get("bomTable").cloned().unwrap_or(Value::Null)).unwrap_or_default());
        let bom_changed = std::cell::Cell::new(false);
        let mut columns = if is_sheet { state.sheet_bom_columns() } else { Vec::new() };
        let config = if state.settings.bom_columns.is_empty() { crate::panels::bom_columns::default_text() } else { state.settings.bom_columns.clone() };
        for column in crate::panels::bom_columns::parse(&config).columns { let key = column.key(); if !columns.contains(&key) { columns.push(key); } }
        if let Some(table) = bom.borrow().as_ref() { for column in &table.columns { if !columns.contains(column) { columns.push(column.clone()); } } }
        columns.sort_by_key(|column| !bom.borrow().as_ref().is_some_and(|table| table.columns.contains(column)));
        let draw_revisions = |ui: &mut egui::Ui| {
            ui.label("BOM table");
            let mut table = bom.borrow_mut();
            let mut enabled = table.is_some();
            let response = ui.checkbox(&mut enabled, "Show BOM table");
            revision_hits.borrow_mut().push(("sheets:bom:enabled".into(), response.rect));
            if response.changed() {
                *table = enabled.then(brep_render::sheets::BomTable::default); bom_changed.set(true);
            }
            if let Some(table) = table.as_mut() {
                ui.horizontal(|ui| {
                    ui.label("Position (mm)");
                    for coordinate in &mut table.position { if ui.add(egui::DragValue::new(coordinate).speed(1.)).changed() { bom_changed.set(true); } }
                });
                ui.horizontal(|ui| { ui.label("Column width (mm)"); if ui.add(egui::DragValue::new(&mut table.column_width_mm).range(8.0..=200.0)).changed() { bom_changed.set(true); } });
                ui.label("Columns");
                egui::ScrollArea::vertical().id_salt(("bom-columns", id)).max_height(180.0).show(ui, |ui| {
                for column in &columns {
                    let mut selected = table.columns.contains(column);
                    let response = ui.checkbox(&mut selected, column.replace('_', " "));
                    revision_hits.borrow_mut().push((format!("sheets:bom:column:{column}"), response.rect));
                    if response.changed() {
                        if selected { table.columns.push(column.clone()); } else { table.columns.retain(|c| c != column); }
                        bom_changed.set(true);
                    }
                }
                });
            }
            ui.separator();
            ui.label("Revisions");
            draw_revision_rows(ui, id, &revision_rows, &revision_intents, &revision_hits);
        };
        let spec = FormViewSpec {
            title: &title,
            subtitle: None,
            fields: &fields,
            banner: note.map(|note| (note, egui::Color32::from_rgb(0x58, 0xa6, 0xff))),
            trailing: None,
            exit_label: "Return to tree",
            extra: is_sheet.then_some(("Tables", &draw_revisions as &dyn Fn(&mut egui::Ui))),
            rollback: false,
            hidden: None,
            hits_prefix: "sheets:",
            read_only: state.history.locked(),
        };
        let mut out = form_view(ui, &spec, &mut params, Some(&mut self.hits));
        if is_sheet && bom_changed.get() { params["bomTable"] = serde_json::to_value(bom.into_inner()).unwrap_or(Value::Null); out.changed = true; }
        for (key, rect) in revision_hits.into_inner() {
            self.hits.insert(key, rect);
        }
        // ONE revision edit per frame, and only when the schema fields did not
        // change on the same frame (each is a document write).
        if !out.changed {
            if let Some(intent) = revision_intents.into_inner().into_iter().next() {
                *action = Some(match intent {
                    RevisionIntent::Add => Action::AddRevision(id.to_string()),
                    RevisionIntent::Update(index, field, value) => {
                        Action::UpdateRevision(id.to_string(), index, serde_json::json!({ field: value }))
                    }
                    RevisionIntent::Remove(index) => Action::RemoveRevision(id.to_string(), index),
                });
            }
        }
        let anchor = self.hits.get("sheets:form:feature").map(|rect| rect.min).unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(format!("sheets:form:object:{id}"), egui::Rect::from_min_size(anchor, egui::Vec2::ZERO));
        if out.changed {
            *action = Some(match catalogue_index {
                0 => Action::UpdateSheet(id.to_string(), params),
                2 => Action::UpdateDimension(id.to_string(), params),
                3 => Action::UpdateOrdinate(id.to_string(), params),
                // A placement, a section or a detail: one update door.
                _ => Action::UpdateView(id.to_string(), params),
            });
        }
        // Every reference row's **Select** is the same gesture a feature's
        // is: the reference picker. On a sheet the picks are anchors clicked
        // on the paper, and Finish writes them into this object's field.
        if let Some(activate) = out.ref_activate {
            *action = Some(Action::BeginRefSelect {
                id: id.to_string(),
                path: activate.path,
                label: activate.label,
                filter: activate.filter,
                multiple: activate.multiple,
                seed: activate.seed,
            });
        }
        if out.exit_clicked {
            *close = true;
        }
    }

    /// The per-frame widget rects for the automation layer.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }
}

/// The sheet dialog's **Revisions** section: one row per revision — its
/// letter, date and description as text edits and a ✕ that removes it — then
/// **+ Add revision**. Every control publishes a keyed rect
/// (`sheets:rev:<n>:rev|date|description|remove`, `sheets:rev:add`).
fn draw_revision_rows(
    ui: &mut egui::Ui,
    sheet_id: &str,
    rows: &std::cell::RefCell<Vec<brep_render::sheets::Revision>>,
    intents: &std::cell::RefCell<Vec<RevisionIntent>>,
    hits: &std::cell::RefCell<Vec<(String, egui::Rect)>>,
) {
    let mut rows = rows.borrow_mut();
    if rows.is_empty() {
        ui.label(egui::RichText::new("No revisions \u{2014} the sheet draws no revision table").weak().italics());
    }
    // Two lines per revision, because the pane is a narrow column: the letter,
    // the date and the ✕ on the first, the description the whole width of the
    // second.
    for (index, row) in rows.iter_mut().enumerate() {
        let mut edit = |ui: &mut egui::Ui, field: &'static str, value: &mut String, width: f32, hint: &str| {
            let response = ui.add(
                egui::TextEdit::singleline(value)
                    .id(egui::Id::new(("sheet-revision", sheet_id, index, field)))
                    .hint_text(hint)
                    .desired_width(width),
            );
            hits.borrow_mut().push((format!("sheets:rev:{index}:{field}"), response.rect));
            if response.changed() {
                intents.borrow_mut().push(RevisionIntent::Update(index, field, value.clone()));
            }
        };
        ui.horizontal(|ui| {
            ui.label(egui::RichText::new("Rev").weak());
            edit(ui, "rev", &mut row.rev, 36.0, "A");
            ui.label(egui::RichText::new("Date").weak());
            edit(ui, "date", &mut row.date, 88.0, "YYYY-MM-DD");
            let remove = ui
                .add(crate::icon_text::icon_button(ui, "\u{2715}"))
                .on_hover_text("Remove this revision row");
            hits.borrow_mut().push((format!("sheets:rev:{index}:remove"), remove.rect));
            if remove.clicked() {
                intents.borrow_mut().push(RevisionIntent::Remove(index));
            }
        });
        // Capped, not the whole available width: in a scrolling pane that can
        // run past the clip rect, and a rect egui will not click is a lie.
        let width = ui.available_width().min(240.0);
        edit(ui, "description", &mut row.description, width, "Description");
        ui.add_space(4.0);
    }
    let add = ui
        .add(crate::icon_text::icon_button(ui, "+ Add revision"))
        .on_hover_text("Append a revision row: the next letter, today's date, no description");
    hits.borrow_mut().push(("sheets:rev:add".into(), add.rect));
    if add.clicked() {
        intents.borrow_mut().push(RevisionIntent::Add);
    }
}

/// `4` rather than `4.0`, `2.5` as `2.5` — a scale reads as a ratio.
fn trim(value: f64) -> String {
    let text = format!("{value:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text.is_empty() { "0".into() } else { text.to_string() }
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "sheets", prefix: "sheets:panel:clip", meaning: "the pane's clip rect \u{2014} a row scrolled outside it publishes a rect egui will not accept a click on", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:add", meaning: "add a sheet at the default paper size and open it", command: Some("sheet_add") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:place", meaning: "open the place-view menu", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:place:", meaning: "place that saved PMI view on the open sheet (sheets:place:<view id>)", command: Some("sheet_place_view") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:row:", meaning: "a sheet, placement, dimension or ordinate-set row (sheets:row:<id>) \u{2014} a click opens a sheet in the sheet viewport or SELECTS what it holds, a double click opens that row's dialog", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:cell:", meaning: "a row cell (sheets:cell:<id>:<column>) \u{2014} `sheets:cell:<sheet id>:on` is the toggle that OPENS a sheet in the sheet viewport", command: Some("sheet_open") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:box:", meaning: "a row's collapse box (sheets:box:<row id>) \u{2014} a sheet's box folds its placed views away", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:col:", meaning: "a table column header (sheets:col:<field>) \u{2014} click to sort", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:grip:", meaning: "a column's resize grip (sheets:grip:<field>)", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:menu:", meaning: "a row's action-menu trigger cell (sheets:menu:<row id>); a right-click on the row opens the same menu", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:menuitem:", meaning: "one entry of the OPEN row action menu (sheets:menuitem:<row id>:<action>) \u{2014} published only while the menu is up", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:form:", meaning: "the open sheet / placement form (sheets:form:object:<id>, sheets:form:feature, sheets:form:return)", command: Some("sheet_update") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:field:", meaning: "one schema field of the open form (sheets:field:<path>) \u{2014} the paper size, the placed view, its position and scale, `sheets:field:flattenText`, the checkbox that lays that view's annotation text flat on the sheet, and a sheet dimension's kind, alignment, offset and precision and its tolerance block — `sheets:field:tolMode` (none / symmetric / deviation / limits), `sheets:field:tolUpper`, `sheets:field:tolLower` and the `sheets:field:isReference` checkbox, which an ordinate set's form carries too. `sheets:field:anchors#activate` opens the reference picker on the paper (anchors are clicked as `sheet/anchor:<ref>`, then `modebar/refsel:finish`), `sheets:field:anchors#line<n>` is one anchor row and `sheets:field:anchors#x<n>` drops it. An ORDINATE SET's form adds `sheets:field:axis` and its two reference fields `sheets:field:datum` and `sheets:field:members`; a SECTION's form its `sheets:field:cut` (the cutting line), `sheets:field:sectionLabel` and `sheets:field:sectionFlip`; a DETAIL's form its `sheets:field:centre`, `sheets:field:rim` and `sheets:field:detailLabel` \u{2014} each reference field with the same `#activate`, `#line<n>` and `#x<n>` keys", command: Some("sheet_update_dimension") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:group:", meaning: "a collapsible group header of the open form (sheets:group:<name>)", command: None },
    HitKeyDoc { panel: "sheets", prefix: "sheets:bom:", meaning: "the sheet BOM table toggle and column choices: sheets:bom:enabled, sheets:bom:column:<attribute>", command: Some("sheet_update") },
    HitKeyDoc { panel: "sheets", prefix: "sheets:rev:", meaning: "the SHEET dialog's Revisions section: `sheets:rev:add` appends a row (the next letter, today's date), and per row `sheets:rev:<n>:rev`, `sheets:rev:<n>:date` and `sheets:rev:<n>:description` are its three text edits and `sheets:rev:<n>:remove` removes it. Rows keep their order; the sheet draws them as the revision table beside its title block", command: Some("sheet_add_revision") },
];




