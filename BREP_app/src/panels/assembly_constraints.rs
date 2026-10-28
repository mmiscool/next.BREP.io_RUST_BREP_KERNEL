//! Assembly Constraints panel — the schema-driven constraint collection widget,
//! driven EXACTLY like the feature history panel: a TREE of constraints, and
//! activating one replaces the whole panel with that constraint's dialog, drawn
//! by the SHARED [`crate::form_view`]. The kernel's
//! `constraint_schema_catalogue` supplies the ten schemas and
//! [`brep_render::features::form_fields_from_schema`] maps each into the shared
//! form fields; this panel supplies the engine side and owns nothing about
//! layout.
//!
//! # The two modes, and what decides them
//!
//! Unlike the history panel, the mode is NOT panel-local state: it is READ from
//! the constraint's own `open` flag, which the kernel already maintains as an
//! ACCORDION (`EngineState::assembly_set_constraint_open` closes every other row
//! when one opens, so at most one is ever open). That matters because two other
//! surfaces open a constraint WITHOUT going through this panel — the context
//! bar's add-from-selection (`panels::context_bar`) and a viewport constraint
//! label click (`EngineState::constraint_label_clicked`). Deriving the mode from
//! `open` makes both of them open the form for free; a panel-local mode would
//! leave them setting a flag nothing renders. It also supplies the validity
//! guard for nothing: a deleted or undone constraint has no row, so there is no
//! open row, so the panel is back in the tree.
//!
//! ## Tree mode
//!
//! * Header: **Solve** (manual solve — works with auto-solve off), the
//!   **auto-solve** toggle (`settings.assembly_auto_solve`, consulted by the
//!   engine's mutation path), the **DOF readout** (dof/rank/redundant +
//!   over-/under-constrained wording), **Show Constraint Graphics**
//!   (`settings.show_constraint_graphics` — the render flag lane G consumes),
//!   and **Update components (N)** — the shell-owned [`UpdateComponents`]
//!   checker supplies N (source-signature comparison); clicking runs the batch
//!   refresh through the document-transport lane.
//! * `+` dropdown of the ten types (catalogue order), pre-seeding `elements`
//!   from the current selection filtered by the type's `selectionFilter`.
//! * Per row: enable checkbox / **edit** (`✎`) / delete / drag-reorder; status
//!   label + color from the ONE map ([`brep_render::assembly_status`]);
//!   distance/angle rows append the evaluated value (`… 12.5` / `… 90°`).
//! * A single click on the row SELECTS that constraint — the selection a
//!   viewport label click makes (`EngineState::constraint_select`: the chip's
//!   selected accent, the context bar's Delete constraint) — and the row shows
//!   it. The edit button, the `[+]` box and a DOUBLE click on the row all OPEN
//!   that constraint's form — the history tree's shape, and the same `SetOpen`
//!   engine call the expand-collapse used to make.
//!
//! ## Form mode
//!
//! One constraint's dialog fills the panel: the row's (value-free) label as the
//! title, its status as the banner in the ONE map's colour, the schema fields —
//! `elements` reference chips reuse the engine's modal ref-select in its
//! CONSTRAINT flavour (`begin_ref_select_for_constraint`), including
//! `{solid}@x,y,z` vertex refs — and ONE exit button back to the tree. Editing
//! is LIVE (every change commits and re-solves per the auto-solve setting);
//! there is no Cancel and no buffer, exactly as in the feature dialogs. Nothing
//! ROLLS: assembly constraints have no rollback, which the form view is told
//! through [`crate::form_view::FormViewSpec::rollback`].

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree::RowAction;
use crate::form_view::{form_view, FormViewSpec};
use crate::panels::tree::{self, TreeRow};
use crate::panels::update_components::UpdateComponents;
use crate::store::ModelStore;
use brep_render::assembly_status;
use brep_render::engine_state::{EngineState, RefSelectField, RefSelectTarget};
use brep_render::features::form_fields_from_schema;
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

/// Who this panel is when it drives the viewport's dialog-row hover
/// (`EngineState::hover_entity_by_name`) — the owner tag that keeps its
/// highlight independent of the history form's and the Scene tree's.
const DIALOG_HOVER_OWNER: &str = "constraints";

/// The red of the per-row delete affordance (matches the history panel).
const DELETE_RED: egui::Color32 = egui::Color32::from_rgb(0xd8, 0x54, 0x4f);

/// One constraint row's per-frame snapshot (owned, so the draw loop can defer
/// `&mut state` mutations — the shared panel pattern).
/// The constraint menu's entries (a row's right-click menu and a viewport
/// chip's): Flip, Edit, Suppress / Enable, Delete.
pub(crate) const ACT_FLIP: &str = "flip";
pub(crate) const ACT_EDIT: &str = "edit";
pub(crate) const ACT_SUPPRESS: &str = "suppress";
pub(crate) const ACT_ENABLE: &str = "enable";
pub(crate) const ACT_DELETE: &str = "delete";

/// The menu a constraint of `type_id` offers, `enabled` or suppressed.
pub(crate) fn constraint_actions(type_id: &str, enabled: bool) -> Vec<RowAction> {
    let flip = RowAction::new(ACT_FLIP, format!("{FLIP_GLYPH} Flip")).tooltip("The other solution: the other side, facing the other way, turning the other way");
    let flip = match flip_refusal(type_id) {
        Some(why) => flip.disabled(why),
        None => flip,
    };
    let toggle = if enabled {
        RowAction::new(ACT_SUPPRESS, "Suppress").tooltip("Switch this constraint off without deleting it")
    } else {
        RowAction::new(ACT_ENABLE, "Enable").tooltip("Switch this constraint back on")
    };
    vec![
        flip,
        RowAction::new(ACT_EDIT, "\u{270E} Edit").tooltip("Open the constraint's form"),
        toggle,
        RowAction::new(ACT_DELETE, "\u{2716} Delete").separator_above().destructive(),
    ]
}

/// [`constraint_actions`] for the live constraint `id` (empty when it is gone).
pub(crate) fn constraint_actions_for(state: &mut EngineState, id: &str) -> Vec<RowAction> {
    let rows = state.assembly_statuses_value();
    rows.as_array()
        .into_iter()
        .flatten()
        .find(|row| row["id"] == id)
        .map(|row| constraint_actions(row["type"].as_str().unwrap_or(""), row["enabled"].as_bool().unwrap_or(true)))
        .unwrap_or_default()
}

/// Run one menu entry on constraint `id`: the ONE dispatch the row menu and
/// the chip menu share.
pub(crate) fn run_constraint_action(state: &mut EngineState, id: &str, action: &str) -> Result<(), String> {
    match action {
        ACT_FLIP => state.assembly_flip_constraint(id).map(drop).map_err(|reason| format!("Flip {id}: {reason}")),
        ACT_EDIT => {
            state.constraint_label_clicked(id);
            Ok(())
        }
        ACT_SUPPRESS => state.assembly_set_constraint_enabled(id, false),
        ACT_ENABLE => state.assembly_set_constraint_enabled(id, true),
        ACT_DELETE => state.assembly_remove_constraint(id),
        other => Err(format!("unknown constraint menu entry '{other}'")),
    }
}

/// The Flip button's glyph (⇅).
const FLIP_GLYPH: &str = "\u{21C5}";

/// Why a constraint type cannot flip, or `None` when it can (the kernel's
/// `flip_rule` decides the rest: a Distance to a line refuses on the click).
fn flip_refusal(type_id: &str) -> Option<&'static str> {
    match type_id {
        "fixed" => Some("A fixed component has nothing to flip"),
        "coincident" => Some("Coincident elements have one solution"),
        "symmetric" => Some("A symmetric pair has one solution"),
        _ => None,
    }
}

struct ConstraintRow {
    id: String,
    type_id: String,
    /// The type's icon (`ConstraintTypeDef::icon`) — the tree row's glyph
    /// column, the same artwork as the toolbar button and the viewport chip.
    icon: String,
    /// The tree row text: `{id}  {type name}` + the evaluated value suffix
    /// (`DIST3  Distance  12.5`) — the history tree's `{id}  {name}` shape, so
    /// the two trees read alike under their glyph columns.
    label: String,
    /// `{type name} {id}` (`Distance DIST3`), WITHOUT the evaluated suffix — the
    /// form's title, and therefore the scope key for its per-field widget ids
    /// and transient view state ([`FormViewSpec::title`]). It must be STABLE:
    /// `label` changes as the solver re-evaluates the measure, and re-keying the
    /// widgets mid-edit would hand the `distance` field a fresh (empty) edit
    /// buffer and eat what the user was typing.
    title: String,
    enabled: bool,
    open: bool,
    status: String,
    /// In a named conflict, this is the member suppressing it would suggest
    /// (the most recently added).
    suppress_suggested: bool,
    input_params: Value,
}

/// A deferred engine mutation (one per frame).
enum Action {
    Add(String, Value),
    SetEnabled(String, bool),
    SetOpen(String, bool),
    /// A single click on the row: select, open nothing.
    Select(String),
    /// A double click on the row: select AND open, as a label click does.
    SelectAndOpen(String),
    Delete(String),
    /// The row's Flip: the constraint's other solution, one undo step.
    Flip(String),
    /// An entry of a row's right-click menu ([`run_constraint_action`]).
    Menu(String, String),
    Move(String, usize),
    UpdateParams(String, Value),
    Solve,
    /// Run the update-components batch refresh (the header button).
    UpdateComponents,
    BeginRefSelect {
        id: String,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed: Vec<String>,
    },
    /// The focused reference field's `Done`: commit the running pick.
    FinishRefSelect,
    /// An `✕` on the focused reference field: drop the i-th running pick.
    RefSelectRemove(usize),
}

/// The Assembly Constraints panel's transient UI state.
#[derive(Default)]
pub struct AssemblyConstraintsPanel {
    /// Per-frame widget screen rects for the headed verifier.
    hits: HashMap<String, egui::Rect>,
    /// The row index currently drag-reordered (`None` = not dragging).
    drag_src: Option<usize>,
    /// Why the document may not change this frame (the form draws disabled).
    locked: Option<String>,
    /// The "held after Replace — Review" button was clicked; the shell opens
    /// the Replace summary and clears it.
    pub review_replace_requested: bool,
    /// The open RIGHT-CLICK menu of a constraint row: the constraint and the
    /// screen point it opened at.
    row_menu: Option<(String, egui::Pos2)>,
    /// A right-click this frame, applied after the open menu has drawn.
    row_menu_request: Option<(String, egui::Pos2)>,
    /// The Exploded views section at the pane's foot (`panels::exploded_views`).
    exploded: crate::panels::exploded_views::ExplodedSection,
    /// The held-mates count and wording, per history revision
    /// (`replace_component::held_summary`).
    held: Option<(u64, usize, &'static str)>,
}

impl AssemblyConstraintsPanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the constraints panel. Every effect routes through the engine
    /// (`EngineState`); the two header toggles also persist through `model_store`
    /// (the settings key — the toolbar's wireframe-toggle pattern). `updates`
    /// is the shell-owned outdated checker (kept current by the shell's
    /// per-frame `ensure_current`); `model_store` feeds the batch refresh.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        model_store: &dyn ModelStore,
        updates: &mut UpdateComponents,
    ) {
        self.hits.clear();
        self.locked = state.history.locked().map(str::to_string);
        // The panel's VISIBLE region (the enclosing dock pane's scroll viewport),
        // published exactly as the history panel publishes `panel:clip`: every
        // other rect here is a raw LAYOUT rect, so a long constraint list — or a
        // long form — runs past the pane's bottom where egui clips it and it
        // stops being clickable even though the rect is still published. The
        // headed verifier intersects against this to know when it must scroll
        // first.
        self.hits.insert("acon:panel:clip".into(), ui.clip_rect());

        // --- snapshots (owned) -------------------------------------------------
        let catalogue = brep_render::brep_kernel::constraint_schema_catalogue();
        let schemas: Vec<Value> = catalogue.as_array().cloned().unwrap_or_default();
        let statuses = state.assembly_statuses_value();
        let overlay = state.assembly_overlay_value();
        let dof = state.assembly_dof_value();
        let constraint_state = state.assembly_state_value();
        let rows = snapshot_rows(&constraint_state, &statuses, &overlay, &schemas);
        let selected = state.selected_constraint();

        ui.spacing_mut().item_spacing.y = 2.0;

        let mut action: Option<Action> = None;
        // The form's EXIT, kept out of `action` on purpose. The panel applies one
        // `Action` per frame, and leaving the form can legitimately coincide with
        // a param commit: a focused `Scalar` commits on FOCUS LOSS, so clicking
        // "Return to tree" while one is focused produces `changed` AND
        // `exit_clicked` in the same frame. Sharing one slot would drop whatever
        // the user had just typed.
        let mut close: Option<String> = None;
        // The entity a hovered reference LINE names, kept out of `action` for the
        // same reason `close` is: hovering a line is not one of the panel's one
        // deferred mutations, and it legitimately coincides with any of them.
        let mut hover: Option<String> = None;

        // --- THE MODE SWITCH: the tree, or ONE constraint's form --------------
        // Read from the constraint's own `open` flag rather than from panel
        // state — see the module doc: the kernel keeps it an accordion (at most
        // one open) and two other surfaces set it without going through here.
        match rows.iter().find(|row| row.open) {
            Some(row) => {
                // The pick running in this constraint's form, if one is.
                let picking = state.ref_select_field().filter(|f| {
                    f.target == RefSelectTarget::AssemblyConstraint && f.subject == row.id
                });
                self.show_form(ui, row, &schemas, picking.as_ref(), &mut action, &mut close, &mut hover)
            }
            None => self.show_tree(
                ui,
                state,
                model_store,
                updates,
                &rows,
                selected.as_deref(),
                &schemas,
                &dof,
                &mut action,
            ),
        }

        // --- the open row menu: the same entries and dispatch as a chip's ------
        if let Some((id, pos)) = self.row_menu.clone() {
            match rows.iter().find(|row| row.id == id && !row.open) {
                Some(row) => {
                    let hits = &mut self.hits;
                    let (chosen, still_open) = crate::column_tree::action_menu(
                        ui.ctx(),
                        ui.layer_id(),
                        egui::Id::new("brep-acon-row-menu"),
                        pos,
                        &constraint_actions(&row.type_id, row.enabled),
                        |entry, rect| {
                            hits.insert(format!("acon:menuitem:{id}:{}", entry.id), rect);
                        },
                    );
                    if !still_open {
                        self.row_menu = None;
                    }
                    if let Some(entry) = chosen {
                        self.row_menu = None;
                        action = Some(Action::Menu(id, entry));
                    }
                }
                None => self.row_menu = None,
            }
        }
        if let Some(request) = self.row_menu_request.take() {
            self.row_menu = Some(request);
        }
        // The Exploded views section, under the tree (never under a form).
        if !rows.iter().any(|row| row.open) {
            self.exploded.show(ui, state, &mut self.hits);
        }

        // --- apply the one deferred engine mutation ----------------------------
        let result: Result<(), String> = match action {
            Some(Action::Add(type_id, params)) => state
                .assembly_add_constraint(&type_id, &params.to_string())
                .map(|_id| ()),
            Some(Action::SetEnabled(id, enabled)) => {
                state.assembly_set_constraint_enabled(&id, enabled)
            }
            Some(Action::SetOpen(id, open)) => state.assembly_set_constraint_open(&id, open),
            Some(Action::Select(id)) => {
                state.constraint_select(&id);
                Ok(())
            }
            Some(Action::SelectAndOpen(id)) => {
                state.constraint_select(&id);
                state.assembly_set_constraint_open(&id, true)
            }
            Some(Action::Delete(id)) => state.assembly_remove_constraint(&id),
            Some(Action::Flip(id)) => state.assembly_flip_constraint(&id).map(drop).map_err(|reason| format!("Flip {id}: {reason}")),
            Some(Action::Menu(id, entry)) => run_constraint_action(state, &id, &entry),
            Some(Action::Move(id, index)) => state.assembly_move_constraint(&id, index),
            Some(Action::UpdateParams(id, params)) => {
                state.assembly_update_constraint(&id, &params.to_string())
            }
            Some(Action::Solve) => state.assembly_run_solve(),
            // The batch refresh pushes its own per-entry + summary notices;
            // only a whole-batch failure routes to the shared toast below.
            Some(Action::UpdateComponents) => updates.run(state, model_store).map(|_| ()),
            Some(Action::BeginRefSelect {
                id,
                path,
                label,
                filter,
                multiple,
                seed,
            }) => {
                // Focus moves from one selection box to the next: a pick
                // already running commits first.
                if state.ref_select_active() {
                    state.finish_ref_select();
                }
                state.begin_ref_select_for_constraint(&id, path, label, filter, multiple, seed);
                Ok(())
            }
            Some(Action::FinishRefSelect) => {
                state.finish_ref_select();
                Ok(())
            }
            Some(Action::RefSelectRemove(index)) => {
                state.ref_select_remove(index);
                Ok(())
            }
            None => Ok(()),
        };
        if let Err(error) = result {
            state.push_notice(format!("Assembly constraints: {error}"));
        }
        // Leaving the form is a SECOND mutation, applied after the action so a
        // commit and an exit in the same frame both land (see `close` above).
        if let Some(id) = close {
            // Leaving the form commits the pick running in it.
            if state
                .ref_select_field()
                .is_some_and(|f| f.target == RefSelectTarget::AssemblyConstraint)
            {
                state.finish_ref_select();
            }
            if let Err(error) = state.assembly_set_constraint_open(&id, false) {
                state.push_notice(format!("Assembly constraints: {error}"));
            }
        }
        // This pane drew (or opened) the pick: the shell finishes an in-form
        // pick no form claims, so one never runs on with its field off screen.
        if let Some(field) = state.ref_select_field() {
            if field.target == RefSelectTarget::AssemblyConstraint
                && state.assembly_open_constraint().as_deref() == Some(field.subject.as_str())
            {
                state.ref_select_claim();
            }
        }

        // A hovered reference line lights the entity it names in the 3D view.
        // Applied every frame — including from the TREE branch, where `hover` is
        // None and this ends the highlight the form had lit. `dialog_hover_end`
        // only ends a hover THIS panel set, so the other panes in a split dock
        // keep theirs.
        let hover_changed = match &hover {
            Some(name) => state.hover_entity_by_name(DIALOG_HOVER_OWNER, name),
            None => state.dialog_hover_end(DIALOG_HOVER_OWNER),
        };
        if hover_changed {
            // The viewport tile may have drawn BEFORE this pane in the dock.
            ui.ctx().request_repaint();
        }

        // --- verifier hooks (wasm only) ----------------------------------------
        if crate::automation::registry::enabled() {
            let listing: Vec<Value> = rows
                .iter()
                .map(|row| {
                    serde_json::json!({
                        "id": row.id,
                        "type": row.type_id,
                        "label": row.label,
                        "icon": row.icon,
                        "enabled": row.enabled,
                        "open": row.open,
                        "selected": selected.as_deref() == Some(row.id.as_str()),
                        "status": row.status,
                        "statusLabel": assembly_status::status_label(&row.status),
                        "statusColor": assembly_status::status_color_hex(&row.status),
                    })
                })
                .collect();
            crate::automation::registry::publish("__brepAssemblyConstraints", "assembly constraint rows and DOF summary",
                &serde_json::json!({
                    "rows": listing,
                    "dof": dof,
                    "updateCount": updates.outdated_count(),
                })
                .to_string(),
            );
            self.publish_hits();
        }
    }

    /// Draw the constraint TREE — the solver header, one row per constraint, and
    /// the add-constraint dropdown.
    #[allow(clippy::too_many_arguments)]
    fn show_tree(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        model_store: &dyn ModelStore,
        updates: &mut UpdateComponents,
        rows: &[ConstraintRow],
        selected: Option<&str>,
        schemas: &[Value],
        dof: &Value,
        action: &mut Option<Action>,
    ) {

        // --- Solver accordion: solve / auto-solve / graphics / update / status.
        // Grouped above the constraint list and STACKED vertically (this used
        // to be one wrapping row of controls). Default-open so the Solve button
        // and status stay visible (and the headed-verify rects stay populated).
        egui::CollapsingHeader::new("Solver")
            .id_salt("acon-solver")
            .default_open(true)
            .show(ui, |ui| {
                ui.spacing_mut().item_spacing.y = 4.0;
                let full = egui::vec2(ui.available_width(), 0.0);

                let solve = ui
                    .add(crate::icon_text::icon_button(ui, "\u{25B6} Solve").min_size(full))
                    .on_hover_text(
                        "Solve the assembly constraints now (works with auto-solve off)",
                    );
                self.hits.insert("acon:solve".into(), solve.rect);
                if solve.clicked() {
                    *action = Some(Action::Solve);
                }

                let mut auto = state.settings.assembly_auto_solve;
                let auto_resp = ui
                    .checkbox(&mut auto, "Auto-solve")
                    .on_hover_text("Re-solve after every constraint change");
                self.hits.insert("acon:autosolve".into(), auto_resp.rect);
                if auto_resp.changed() {
                    state.settings.assembly_auto_solve = auto;
                    state.settings_generation = state.settings_generation.wrapping_add(1);
                    let _ = model_store.write(crate::store::SETTINGS_KEY, &state.settings_json());
                }

                let mut graphics = state.settings.show_constraint_graphics;
                let graphics_resp = ui
                    .checkbox(&mut graphics, "Show constraint graphics")
                    .on_hover_text("Draw per-constraint leaders + labels in the viewport");
                self.hits.insert("acon:graphics".into(), graphics_resp.rect);
                if graphics_resp.changed() {
                    state.settings.show_constraint_graphics = graphics;
                    state.settings_generation = state.settings_generation.wrapping_add(1);
                    state.dirty = true;
                    let _ = model_store.write(crate::store::SETTINGS_KEY, &state.settings_json());
                }

                // Update components: N = the checker's source-signature
                // comparison; enabled only when outdated.
                let outdated = updates.outdated_count();
                let mut hover =
                    "Refresh outdated parts from their source — every instance follows".to_string();
                if !updates.missing().is_empty() {
                    hover.push_str(&format!(
                        "; no source document for {}",
                        updates.missing().join(", ")
                    ));
                }
                // The note must survive the DISABLED state too (count 0 with
                // source-less parts is exactly when it matters): egui suppresses
                // plain hover text on disabled widgets.
                let update = ui
                    .add_enabled(
                        outdated > 0,
                        egui::Button::new(format!("Update components ({outdated})"))
                            .min_size(full),
                    )
                    .on_hover_text(hover.clone())
                    .on_disabled_hover_text(hover);
                self.hits.insert("acon:update".into(), update.rect);
                if update.clicked() {
                    *action = Some(Action::UpdateComponents);
                }

                // Mates a Replace component is holding (disabled, waiting for a
                // face): say so where the mates are, and reopen the summary.
                let (held, after) = crate::panels::replace_component::held_summary(&mut self.held, state);
                if held > 0 {
                    let review = ui
                        .add(
                            egui::Button::new(
                                egui::RichText::new(format!("\u{26A0} {held} mate(s) held after {after}: Review"))
                                    .color(ui.visuals().warn_fg_color),
                            )
                            .min_size(full),
                        )
                        .on_hover_text("These mates lost their face when a component was replaced or its part was edited; pick a face or delete them");
                    self.hits.insert("acon:replace-review".into(), review.rect);
                    if review.clicked() {
                        self.review_replace_requested = true;
                    }
                }

                // Solve status: "No constraints solved yet", the DOF summary, or
                // the last solve error.
                ui.label(egui::RichText::new(dof_summary(dof)).weak());
            });
        ui.add_space(4.0);

        // --- ROOT: `[-] Assembly Constraints (N)` ------------------------------
        tree::node(
            ui,
            TreeRow {
                guides: &[],
                is_last: true,
                expandable: true,
                expanded: true,
                root: true,
                glyph: None,
                label: "Assembly Constraints",
                selected: false,
                highlighted: false,
                draggable: false,
                tint: None,
            },
            |ui| {
                ui.label(egui::RichText::new(format!("{}", rows.len())).weak());
            },
        );
        if rows.is_empty() {
            let g = tree::child_guides(&[], true);
            tree::node(ui, TreeRow::leaf(&g, true, "(no constraints)"), |_| {});
        }

        // --- rows --------------------------------------------------------------
        let mut row_rects: Vec<(usize, egui::Rect)> = Vec::with_capacity(rows.len());
        let mut drag_move: Option<(usize, usize)> = None;
        let n = rows.len();
        for (i, row) in rows.iter().enumerate() {
            let is_selected = selected == Some(row.id.as_str());
            let rect = self.render_row(ui, row, i + 1 == n, is_selected, action);
            row_rects.push((i, rect));
        }

        // --- resolve an in-flight drag (the history panel's algorithm) ---------
        if let Some(src) = self.drag_src {
            let released = ui.input(|i| i.pointer.any_released());
            let ptr = ui.input(|i| i.pointer.interact_pos());
            match (ptr, released) {
                (Some(p), released) => {
                    let target = row_rects
                        .iter()
                        .min_by(|a, b| {
                            let da = (a.1.center().y - p.y).abs();
                            let db = (b.1.center().y - p.y).abs();
                            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
                        })
                        .map(|(index, _)| *index)
                        .unwrap_or(src);
                    if released && target == src {
                        // NOT a reorder — a press that STARTED and ENDED on the
                        // same row. egui reclassifies a press as a DRAG once it
                        // outlives `max_click_duration` (0.8 s) or drifts past
                        // `max_click_dist` (6 pt), so an ordinary human click on a
                        // label — which routinely lingers or wobbles a few pixels
                        // — fires `drag_started` and NEVER `clicked`. In the
                        // reorder arm it would do nothing at all (`src == target`
                        // is skipped below); it is the click egui refused to
                        // report, so it does what a click does — the history
                        // tree routes the same gesture to "select that row", and
                        // so does this one.
                        if let Some(row) = rows.get(src) {
                            *action = Some(Action::Select(row.id.clone()));
                        }
                        self.drag_src = None;
                    } else if released {
                        drag_move = Some((src, target));
                        self.drag_src = None;
                    } else if target != src {
                        if let Some((_, rect)) =
                            row_rects.iter().find(|(index, _)| *index == target)
                        {
                            let y = if target >= src { rect.bottom() } else { rect.top() };
                            ui.painter().hline(
                                rect.x_range(),
                                y,
                                egui::Stroke::new(2.0, ui.visuals().selection.bg_fill),
                            );
                        }
                    }
                }
                (None, true) => self.drag_src = None,
                _ => {}
            }
        }
        if let Some((src, target)) = drag_move {
            if src != target {
                if let Some(row) = rows.get(src) {
                    *action = Some(Action::Move(row.id.clone(), target));
                }
            }
        }

        // --- `+` dropdown of the ten types -------------------------------------
        ui.add_space(6.0);
        let mut add_type: Option<String> = None;
        let combo = egui::ComboBox::from_id_salt("acon-add")
            // ASCII "+", not U+FF0B: a ComboBox header is plain text and cannot
            // hold a widget, so it must use a character every font has.
            .selected_text("+ Add constraint")
            .width(ui.available_width())
            .show_ui(ui, |ui| {
                for schema in schemas {
                    let type_id = schema.get("type").and_then(Value::as_str).unwrap_or("");
                    let label = schema
                        .get("longName")
                        .and_then(Value::as_str)
                        .unwrap_or(type_id);
                    // The longName leads with the type's glyph; with no icon
                    // font it must be drawn as artwork, not as a character.
                    let item = crate::icon_text::selectable_icon_label(ui, false, label);
                    self.hits
                        .insert(format!("acon:add:{type_id}"), item.rect);
                    if item.clicked() {
                        add_type = Some(type_id.to_string());
                    }
                }
            });
        self.hits.insert("acon:add".into(), combo.response.rect);
        if let Some(type_id) = add_type {
            // Per-type selection context: pre-seed `elements` from the current
            // selection, filtered + capped by the schema's own declaration.
            let seed = seeded_elements(state, schemas, &type_id);
            *action = Some(Action::Add(type_id, seed));
        }
    }

    /// One constraint row. Returns the header row rect (drag-target
    /// hit-testing). Nothing expands here any more: activating a row swaps the
    /// whole panel to that constraint's form.
    fn render_row(
        &mut self,
        ui: &mut egui::Ui,
        row: &ConstraintRow,
        is_last: bool,
        selected: bool,
        action: &mut Option<Action>,
    ) -> egui::Rect {
        let mut enabled = row.enabled;
        let mut enabled_rect = egui::Rect::NOTHING;
        let mut enabled_clicked = false;
        let mut del_rect = egui::Rect::NOTHING;
        let mut del_clicked = false;
        let mut edit_rect = egui::Rect::NOTHING;
        let mut edit_clicked = false;
        let mut suppress_rect = egui::Rect::NOTHING;
        let mut suppress_clicked = false;
        let mut flip_rect = egui::Rect::NOTHING;
        let mut flip_clicked = false;

        let status_label = assembly_status::status_label(&row.status);
        let [r, g, b] = assembly_status::status_color_rgb(&row.status);
        let status_color = egui::Color32::from_rgb(r, g, b);

        let resp = tree::node(
            ui,
            TreeRow::branch(&[], is_last, row.open, &row.label)
                .glyph(Some(row.icon.as_str()).filter(|icon| !icon.is_empty()))
                .selected(selected)
                .draggable(true),
            |ui| {
                // right-to-left: delete X, enable checkbox, then the status.
                let del = ui.add(
                    crate::icon_text::icon_button_colored(ui, "\u{2715}", Some(DELETE_RED))
                        .stroke(egui::Stroke::new(1.0, DELETE_RED))
                        .small(),
                );
                del_rect = del.rect;
                del_clicked = del.clicked();
                ui.add_space(4.0);

                // The discoverable way into the dialog — the history tree's own
                // affordance, same glyph, same `small()` sizing, so a third
                // control costs the label as little width as possible.
                let edit = ui
                    .add(crate::icon_text::icon_button(ui, "\u{270E}").small())
                    .on_hover_text("Edit this constraint");
                edit_rect = edit.rect;
                edit_clicked = edit.clicked();
                ui.add_space(4.0);

                // Flip to the other solution; a type with only one says why.
                let refusal = flip_refusal(&row.type_id);
                let flip = ui
                    .add_enabled(refusal.is_none(), crate::icon_text::icon_button(ui, FLIP_GLYPH).small())
                    .on_hover_text("Flip: the other solution (the other side, facing the other way, turning the other way)")
                    .on_disabled_hover_text(refusal.unwrap_or_default());
                flip_rect = flip.rect;
                flip_clicked = flip.clicked();
                ui.add_space(4.0);

                let cb = ui
                    .add(egui::Checkbox::new(&mut enabled, ""))
                    .on_hover_text("Enable/disable this constraint");
                enabled_rect = cb.rect;
                enabled_clicked = cb.clicked();

                // A named conflict offers to suppress this member; the
                // suggested one (the most recently added) says so.
                if row.status == "conflict" {
                    let text = if row.suppress_suggested { "Suppress \u{2605}" } else { "Suppress" };
                    let hover = if row.suppress_suggested {
                        "Suppress this constraint to resolve the conflict (suggested: it is the most recently added)"
                    } else {
                        "Suppress this constraint to resolve the conflict"
                    };
                    let suppress = ui.add(egui::Button::new(egui::RichText::new(text).small())).on_hover_text(hover);
                    suppress_rect = suppress.rect;
                    suppress_clicked = suppress.clicked();
                }

                // Not selectable: it sits in the row's blank space, and a
                // drag-to-select label senses clicks — selectable, the status
                // would be the one spot there that does not open the dialog.
                ui.add(
                    egui::Label::new(
                        egui::RichText::new(status_label).color(status_color).small(),
                    )
                    .selectable(false),
                );
            },
        );
        // The published row rect IS the band — the whole strip is the click
        // target, so the whole strip is what a script aims at.
        // A right-click anywhere on the row opens its menu (applied after the
        // menu draws, so the click that opens it is not the click that closes it).
        let secondary = if resp.band.secondary_clicked() { Some(&resp.band) } else if resp.label.secondary_clicked() { Some(&resp.label) } else { None };
        if let Some(response) = secondary {
            let at = response.interact_pointer_pos().unwrap_or(response.rect.center());
            self.row_menu_request = Some((row.id.clone(), at));
        }
        self.hits.insert(format!("acon:row:{}", row.id), resp.band.rect);
        self.hits.insert(format!("acon:box:{}", row.id), resp.box_rect);
        self.hits.insert(format!("acon:del:{}", row.id), del_rect);
        self.hits.insert(format!("acon:edit:{}", row.id), edit_rect);
        self.hits.insert(format!("acon:flip:{}", row.id), flip_rect);
        self.hits
            .insert(format!("acon:enable:{}", row.id), enabled_rect);
        if suppress_rect != egui::Rect::NOTHING {
            self.hits.insert(format!("acon:suppress:{}", row.id), suppress_rect);
        }

        if del_clicked {
            *action = Some(Action::Delete(row.id.clone()));
        } else if flip_clicked {
            *action = Some(Action::Flip(row.id.clone()));
        } else if suppress_clicked {
            *action = Some(Action::SetEnabled(row.id.clone(), false));
        } else if enabled_clicked {
            *action = Some(Action::SetEnabled(row.id.clone(), enabled));
        } else if resp.double_clicked() {
            // A DOUBLE click anywhere on the row selects and OPENS it, exactly
            // as a label click in the viewport does. Read before `clicked()`:
            // egui reports the second press as a click too.
            *action = Some(Action::SelectAndOpen(row.id.clone()));
        } else if edit_clicked || resp.toggled {
            // The EDIT button and the `[+]` box OPEN the form — the same
            // `SetOpen` call the expand-collapse used to make, so the engine
            // sees exactly what it saw before this panel grew a form mode.
            // (`!row.open` is `true` here: the tree only draws when no row is
            // open. Left as written so the two modes cannot disagree.)
            *action = Some(Action::SetOpen(row.id.clone(), !row.open));
        } else if resp.clicked() {
            // A single click anywhere else on the row SELECTS it.
            *action = Some(Action::Select(row.id.clone()));
        }
        // The whole row arms the drag, not just its text: a press egui
        // reclassified as a drag is the click it refused to report, and the
        // resolution routes a same-row release back to selecting the row.
        if resp.drag_started() {
            self.drag_src = Some(index_of_hit(&self.hits, &row.id));
        }
        resp.row_rect
    }

    /// Draw ONE constraint's dialog filling the whole panel, through the SHARED
    /// [`form_view`] — the same function the feature dialogs use. This panel
    /// supplies the schema, the live params and the status; the form supplies
    /// every pixel of layout and hands back the intents the engine must act on.
    /// Editing is LIVE; the one exit button closes the row.
    #[allow(clippy::too_many_arguments)]
    fn show_form(
        &mut self,
        ui: &mut egui::Ui,
        row: &ConstraintRow,
        schemas: &[Value],
        picking: Option<&RefSelectField>,
        action: &mut Option<Action>,
        close: &mut Option<String>,
        hover: &mut Option<String>,
    ) {
        let Some(schema) = schemas.iter().find(|schema| {
            schema.get("type").and_then(Value::as_str) == Some(row.type_id.as_str())
        }) else {
            // A row whose type has no schema has no form to show — close it so
            // the panel can never be stranded on an empty mode.
            *close = Some(row.id.clone());
            return;
        };
        let hidden: std::collections::HashSet<String> =
            brep_render::brep_kernel::constraint_hidden_params(&row.type_id, &row.input_params)
                .into_iter()
                .map(str::to_string)
                .collect();
        let fields = form_fields_from_schema(schema);
        let mut params = row.input_params.clone();

        // The row's status is the panel's headline readout for a constraint, and
        // the row is not on screen in form mode — so it rides the BANNER, in the
        // colour the ONE map gives it ([`brep_render::assembly_status`]), never a
        // second vocabulary. The feature form uses the same slot for a run error.
        let status_label = assembly_status::status_label(&row.status);
        let [r, g, b] = assembly_status::status_color_rgb(&row.status);
        let spec = FormViewSpec {
            // The value-free title — see `ConstraintRow::title`.
            title: &row.title,
            subtitle: None,
            fields: &fields,
            banner: Some((status_label, egui::Color32::from_rgb(r, g, b))),
            // A constraint produces no named outputs — nothing to trail.
            trailing: None,
            // Q12: the constraint list is drawn by the SAME `panels::tree`
            // renderer as the feature history (a root node with connector
            // guides), so the history panel's wording is literally accurate here
            // and is kept VERBATIM — one button, one label, everywhere.
            exit_label: "Return to tree",
            extra: None,
            // The owner's call: assembly constraints have NO rollback, so an exit
            // carries no roll and this panel has no roll branch at all.
            rollback: false,
            // Conditional fields (a limit mate's range vs its target), from
            // the kernel's one rule.
            hidden: Some(&hidden),
            // The panel shows ONE form at a time, but its hit map holds the
            // tree's `acon:*` keys too AND is scanned alongside the history
            // panel's by the headed verifier — so the form's keys stay
            // namespaced.
            hits_prefix: "acon:",
            read_only: self.locked.as_deref(),
            picking: picking.map(|f| crate::form::ActivePick { path: &f.path, names: &f.names }),
        };
        let out = form_view(ui, &spec, &mut params, Some(&mut self.hits));

        // WHICH constraint the form is showing, as a presence-only zero-size rect
        // beside the header's real rect (`acon:form:feature`) — the history
        // panel's `form:feature:{id}` convention.
        let anchor = self
            .hits
            .get("acon:form:feature")
            .map(|rect| rect.min)
            .unwrap_or(egui::Pos2::ZERO);
        self.hits.insert(
            format!("acon:form:constraint:{}", row.id),
            egui::Rect::from_min_size(anchor, egui::Vec2::ZERO),
        );

        // Precedence matches the pre-form code exactly: a reference activation is
        // staged first and a param commit overrides it in the ONE action slot.
        if let Some(activate) = out.ref_activate {
            *action = Some(Action::BeginRefSelect {
                id: row.id.clone(),
                path: activate.path,
                label: activate.label,
                filter: activate.filter,
                multiple: activate.multiple,
                seed: activate.seed,
            });
        }
        // The focused reference field's own intents.
        if let Some(index) = out.pick_remove {
            *action = Some(Action::RefSelectRemove(index));
        }
        if out.pick_done {
            *action = Some(Action::FinishRefSelect);
        }
        if out.changed {
            *action = Some(Action::UpdateParams(row.id.clone(), params));
        }
        // The entity a hovered reference line names — applied by `show`, which
        // holds the engine.
        *hover = out.hovered_entity;
        // The exit rides its own slot so a focus-loss commit in the SAME frame
        // still lands (see `close` in `show`).
        if out.exit_clicked {
            *close = Some(row.id.clone());
        }
        // No constraint schema declares a `button` param (the catalogue is ids,
        // `reference_selection`s, numbers and booleans), so there is no schema
        // button to dispatch here; `roll_to_tip` is likewise always false because
        // `rollback: false` above says constraints do not roll.
        debug_assert!(
            out.button_clicked.is_none(),
            "no constraint schema declares a button param"
        );
        debug_assert!(!out.roll_to_tip, "constraints declare rollback: false");
    }

    /// The published widget hit-rects for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
        // This panel publishes its rects from `show`, which did not run.
        if crate::automation::registry::enabled() {
            self.publish_hits();
        }
    }

    fn publish_hits(&self) {
        crate::automation::registry::publish("__brepAssemblyConstraintsHit", "assembly constraints panel widget rects (acon:)", &self.hits_json());
    }
}

/// The drag source index for a row id — recovered from the ordered `acon:row:`
/// hits laid out this frame. (The row order in `hits` is not indexed; walk the
/// rects by vertical position instead.)
fn index_of_hit(hits: &HashMap<String, egui::Rect>, id: &str) -> usize {
    let Some(own) = hits.get(&format!("acon:row:{id}")) else {
        return 0;
    };
    hits.iter()
        .filter(|(key, _)| key.starts_with("acon:row:"))
        .filter(|(_, rect)| rect.center().y < own.center().y)
        .count()
}

/// Fold the constraint state + statuses + overlay values into owned rows.
fn snapshot_rows(
    constraint_state: &Value,
    statuses: &Value,
    overlay: &Value,
    schemas: &[Value],
) -> Vec<ConstraintRow> {
    let status_row = |id: &str| -> Option<&Value> {
        statuses
            .as_array()
            .and_then(|rows| rows.iter().find(|row| row.get("id").and_then(Value::as_str) == Some(id)))
    };
    let status_of = |id: &str| -> String {
        status_row(id)
            .and_then(|row| row.get("status").and_then(Value::as_str))
            .unwrap_or("")
            .to_string()
    };
    let suggested_of = |id: &str| -> bool {
        status_row(id)
            .and_then(|row| row.get("suppressSuggested").and_then(Value::as_bool))
            .unwrap_or(false)
    };
    let overlay_value = |id: &str| -> Option<f64> {
        overlay
            .as_array()
            .and_then(|rows| {
                rows.iter()
                    .find(|row| row.get("id").and_then(Value::as_str) == Some(id))
            })
            .and_then(|row| row.get("value").and_then(Value::as_f64))
    };
    // The plain name and the icon come from the schema's own `label` / `icon`
    // keys (the kernel's `ConstraintTypeDef`), never by stripping the glyph off
    // `longName`.
    let schema_of = |type_id: &str| -> Option<&Value> {
        schemas
            .iter()
            .find(|schema| schema.get("type").and_then(Value::as_str) == Some(type_id))
    };
    let label_of = |type_id: &str| -> String {
        schema_of(type_id)
            .and_then(|schema| schema.get("label").and_then(Value::as_str))
            .map(str::to_string)
            .unwrap_or_else(|| type_id.to_string())
    };
    let icon_of = |type_id: &str| -> String {
        schema_of(type_id)
            .and_then(|schema| schema.get("icon").and_then(Value::as_str))
            .map(str::to_string)
            .unwrap_or_default()
    };

    constraint_state
        .get("constraints")
        .and_then(Value::as_array)
        .map(|constraints| {
            constraints
                .iter()
                .map(|entry| {
                    let type_id = entry
                        .get("type")
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string();
                    let params = entry
                        .get("inputParams")
                        .cloned()
                        .unwrap_or_else(|| Value::Object(serde_json::Map::new()));
                    let id = params
                        .get("id")
                        .and_then(Value::as_str)
                        .unwrap_or("")
                        .to_string();
                    // Evaluated numeric suffix on distance/angle labels
                    // (expression-capable — the overlay carries the evaluated
                    // measure; fall back to the raw param for unresolved rows).
                    let suffix = match type_id.as_str() {
                        "distance" => overlay_value(&id)
                            .or_else(|| params.get("distance").and_then(Value::as_f64))
                            .map(|value| format!("  {value:.3}"))
                            .unwrap_or_default(),
                        "angle" => overlay_value(&id)
                            .or_else(|| params.get("angle").and_then(Value::as_f64))
                            .map(|value| format!("  {value:.1}\u{00B0}"))
                            .unwrap_or_default(),
                        _ => String::new(),
                    };
                    let name = label_of(&type_id);
                    let title = format!("{name} {id}");
                    ConstraintRow {
                        label: format!("{id}  {name}{suffix}"),
                        title,
                        icon: icon_of(&type_id),
                        enabled: entry.get("enabled").and_then(Value::as_bool).unwrap_or(true),
                        open: entry.get("open").and_then(Value::as_bool).unwrap_or(false),
                        status: status_of(&id),
                        suppress_suggested: suggested_of(&id),
                        input_params: params,
                        id,
                        type_id,
                    }
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The DOF readout: `dof/rank/redundant` + the over-/under-constrained wording
/// (the old app only surfaced duplicates).
fn dof_summary(dof: &Value) -> String {
    if dof.get("ok").and_then(Value::as_bool) == Some(false) {
        if let Some(ids) = dof.get("conflict").and_then(Value::as_array) {
            let ids: Vec<&str> = ids.iter().filter_map(Value::as_str).collect();
            let suggested = dof.get("suggestSuppress").and_then(Value::as_str).unwrap_or("");
            return format!(
                "Conflict: {} cannot all hold. Suppress one to solve (suggested: {suggested}).",
                ids.join(", ")
            );
        }
        let error = dof.get("error").and_then(Value::as_str).unwrap_or("solve failed");
        return format!("Solve failed: {error}");
    }
    let mates = dof.get("mates").and_then(Value::as_u64).unwrap_or(0);
    let Some(free) = dof.get("dof").and_then(Value::as_u64) else {
        return if mates == 0 {
            "No constraints solved yet".to_string()
        } else {
            format!("{mates} mate(s)")
        };
    };
    let rank = dof.get("rank").and_then(Value::as_u64).unwrap_or(0);
    let redundant = dof.get("redundant").and_then(Value::as_u64).unwrap_or(0);
    let wording = match (free, redundant) {
        (0, 0) => "fully constrained".to_string(),
        (0, r) => format!("over-constrained ({r} redundant)"),
        (d, 0) => format!("under-constrained ({d} DOF free)"),
        (d, r) => format!("under-constrained ({d} DOF free, {r} redundant)"),
    };
    format!("DOF {free} \u{00B7} rank {rank} \u{00B7} redundant {redundant} \u{2014} {wording}")
}

/// Pre-seed a new constraint's `elements` from the CURRENT selection, filtered
/// by the type's schema `selectionFilter` and capped at its `maxSelections` —
/// the per-type selection context of the `+` dropdown AND the context bar's
/// constraint-from-selection offers (`context_bar::add_constraint_from_selection`).
pub(crate) fn seeded_elements(state: &mut EngineState, schemas: &[Value], type_id: &str) -> Value {
    let Some(schema) = schemas
        .iter()
        .find(|schema| schema.get("type").and_then(Value::as_str) == Some(type_id))
    else {
        return serde_json::json!({});
    };
    let elements_spec = schema
        .get("inputParamsSchema")
        .and_then(|params| params.get("elements"));
    let filter: Vec<String> = elements_spec
        .and_then(|spec| spec.get("selectionFilter"))
        .and_then(Value::as_array)
        .map(|kinds| {
            kinds
                .iter()
                .filter_map(|kind| kind.as_str().map(String::from))
                .collect()
        })
        .unwrap_or_default();
    let cap = elements_spec
        .and_then(|spec| spec.get("maxSelections"))
        .and_then(Value::as_u64)
        .unwrap_or(2) as usize;

    let selection: Value = serde_json::from_str(&state.selection_json()).unwrap_or(Value::Null);
    let names = |key: &str| -> Vec<String> {
        selection[key]
            .as_array()
            .map(|items| {
                items
                    .iter()
                    .filter_map(|item| item.as_str().map(String::from))
                    .collect()
            })
            .unwrap_or_default()
    };
    let mut seeded: Vec<String> = Vec::new();
    let mut push = |src: Vec<String>| {
        for name in src {
            if seeded.len() < cap && !seeded.contains(&name) {
                seeded.push(name);
            }
        }
    };
    for kind in &filter {
        match kind.as_str() {
            // A COMPONENT ref is the owning component of any selected member
            // solid; FACE/EDGE map straight from the named selection.
            "COMPONENT" => {
                let owners: Vec<String> = names("solids")
                    .into_iter()
                    .filter_map(|solid| {
                        state
                            .assembly_components()
                            .iter()
                            .find(|record| record.solids.contains(&solid))
                            .map(|record| record.id.clone())
                    })
                    .collect();
                push(owners);
            }
            "FACE" => push(names("faces")),
            "EDGE" => push(names("edges")),
            // VERTEX selections are position-keyed; the ref-select picker
            // builds their `@`-refs — nothing to pre-seed here.
            _ => {}
        }
    }
    serde_json::json!({ "elements": seeded })
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:add", meaning: "add a constraint", command: Some("assembly_add_constraint") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:solve", meaning: "solve", command: Some("assembly_solve") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:autosolve", meaning: "toggle auto-solve", command: Some("settings_set") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:update", meaning: "update components", command: Some("component_update") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:replace-review", meaning: "reopen the Replace component summary for mates it holds", command: Some("assembly_replace_report") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:graphics", meaning: "toggle overlay graphics", command: Some("settings_set") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:row:", meaning: "a constraint's row (acon:row:<id>) \u{2014} a click SELECTS the constraint, a double click opens its form", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:edit:", meaning: "open a constraint's form", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:del:", meaning: "delete a constraint", command: Some("assembly_remove_constraint") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:box:", meaning: "a constraint's enable box", command: Some("assembly_set_constraint_enabled") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:menuitem:", meaning: "one entry of a constraint row's OPEN right-click menu (acon:menuitem:<id>:<flip|edit|suppress|enable|delete>), published only while the menu is up; the same entries and dispatch as the viewport chip's menu (constraintmenu/menuitem:)", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:flip:", meaning: "flip a constraint to its other solution (acon:flip:<id>): the other side, facing the other way, turning the other way; one undo step. Disabled, with the reason, for a type with one solution (Fixed, Coincident, Symmetric)", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:suppress:", meaning: "suppress a member of a named conflict (acon:suppress:<id>; the suggested one is the most recently added)", command: Some("assembly_set_constraint_enabled") },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:panel:clip", meaning: "the visible region of the pane", command: None },
    HitKeyDoc { panel: "assemblyconstraints", prefix: "acon:", meaning: "a constraint form control", command: Some("assembly_update_constraint") },
];
