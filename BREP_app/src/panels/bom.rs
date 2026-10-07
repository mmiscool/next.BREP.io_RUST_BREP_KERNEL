//! The assembly BOM panel — the parts list on the shared
//! [`crate::column_tree`] widget.
//!
//! This module is the BOM-shaped half: it turns the engine's component
//! projection into the widget's generic rows, and turns the widget's generic
//! edits back into the two attribute stores. The widget itself knows none of
//! this, which is what lets a second consumer (the wire-harness connection
//! list) reuse it untouched.
//!
//! # Packed and unpacked
//!
//! Two views of the same occurrences.
//!
//! * **Unpacked** — one row per placement. `Quantity` reads 1.
//! * **Packed** — one row per DISTINCT part **whose occurrence data matches**.
//!   Two occurrences roll up only when they agree on EVERY occurrence field;
//!   one differing Reference Designator and they stay two rows. `Quantity` is
//!   the size of the group, read-only in both views because it is derived and
//!   so can never disagree with the model.
//!
//! Editing a packed row applies to every occurrence it rolls up, and that
//! fan-out is ONE undo step, not N — `EngineState::set_occurrence_attribute`
//! takes the whole group and checkpoints once. A part-level edit is inherently
//! the same shape (the value lives on the part, so every occurrence of it sees
//! the change) and is one undo step for the same reason.
//!
//! # Nested sub-assembly rows are READ-ONLY
//!
//! A rigid sub-assembly's internal components appear as child rows, so the BOM
//! reads as the tree it is — but their part and occurrence data lives in the
//! SUB-ASSEMBLY's own document, not this one. That is the rigid-nesting model
//! (the same reason `EngineState::export_bom_csv` reports a sub-assembly as one
//! row at this level), not a limit of the widget: editing them means opening
//! that document. They are drawn weak and take no edit.
//!
//! # The row ACTION MENU
//!
//! The rightmost column's `⋯` opens a menu — and so does a right-click
//! anywhere on the row; both are the widget's ONE menu, declared here as
//! [`RowAction`]s. Its entries are the SHARED component actions
//! ([`crate::panels::component_actions`], the same dispatcher the assembly
//! structure tree's row buttons route through) plus this panel's own
//! "Edit feature" — the `✎` button the menu replaced.
//!
//! A click on a component row selects its components, and a DOUBLE click opens
//! its dialog — the component's feature, as Edit feature does — the one rule
//! every tree in the app follows.
//!
//! Availability is decided PER ROW here, because only this panel knows what
//! refuses what: a fixed component will not Move, an embedded-only part has no
//! source document to Open, and a PACKED row standing for several placements
//! refuses the per-instance actions rather than guessing which placement was
//! meant (and Delete across a group would be N undo steps, not the one this
//! panel promises). Refused entries are greyed with the reason, never hidden.
//!
//! # Wire lines
//!
//! Every harness wire is its OWN line (plan decision 6), after the component
//! rows: the connection ID in the Item column, the wire's stock part number in
//! `part.Part_Number`, Quantity 1, and in `occurrence.MF_QTY` its cut length —
//! the routed length of the current run plus the harness block's cut margin,
//! once per wire. Wire lines never go through the packed roll-up: packing keys
//! on the VISIBLE occurrence columns, so two wires of different length would
//! roll into one line the moment MF QTY is hidden, and the margin would be
//! counted once for two cuts. When the run on hand cannot speak for the model
//! (in flight, cancelled, rolled back, absent, or the wire is unrouted) the cell says so in
//! words and shows no number ([`brep_render::engine_state::WireLengthState`]).
//! A wire line is read-only here: its data lives in the Wire Harness panel,
//! and the margin is the header's **Cut margin** field.
//!
//! # The write lanes
//!
//! * occurrence field → `set_occurrence_attribute(ids, key, value)`.
//! * part field → `set_part_attribute(part, key, value)`, then the shared
//!   write-through lane ([`crate::panels::parts_library::write_through`]) so
//!   the part's file and the entry's signature keep agreeing. There is no
//!   second write path.

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree::{self, CellEdit, ColumnLayout, ColumnTreeSpec, RowAction, RowNode};
use crate::panels::parts_library;
use crate::panels::component_actions::{
    run_component_action, ComponentAction, ComponentActionRequest,
};
use crate::panels::assembly_components::{self, ChainNode, ComponentRow};
use crate::panels::update_components::UpdateComponents;
use crate::panels::bom_columns::{
    self, ParsedColumns, Scope, FLAGS_KEY, ITEM_KEY, MF_QTY_KEY, PMI_KEY, QUANTITY_KEY, VISIBLE_KEY,
};
use crate::store::ModelStore;
use brep_render::engine_state::{EngineState, WireBomLine, WireLengthState};
use eframe::egui;
use serde_json::Value;
use std::collections::{BTreeMap, HashMap, HashSet};

/// The row menu's own entry: roll to the component's feature and open it in
/// the history tree. Not a [`ComponentAction`] — the shared set is the
/// COMPONENT vocabulary and the context bar draws a button per member of it,
/// so a document-navigation entry does not belong in there. The assembly
/// structure tree keeps this action locally for the same reason.
const EDIT_FEATURE: &str = "edit-feature";

/// What a BOM frame hands back to the shell.
#[derive(Default)]
pub struct BomOutcome {
    /// Refresh saved source documents through the shared update transaction.
    pub update_components: bool,
    /// A feature id to roll to + expand in the history tree (the row menu's
    /// "Edit feature") — the structure panel's `focus` contract, verbatim, so
    /// the shell routes both the same way.
    pub focus: Option<String>,
    /// A document-level flow the SHELL owns (Edit Part → open the part's own
    /// document tab), handed
    /// back by the shared component-action dispatcher exactly as the selection
    /// context bar hands it back.
    pub component: Option<ComponentActionRequest>,
}

/// One occurrence, flattened out of the engine's projection.
#[derive(Clone)]
struct Occurrence {
    /// The owning ACOMP feature id.
    id: String,
    part_name: String,
    /// This occurrence's own attribute record.
    attributes: Value,
    selected: bool,
    /// Grounded (the ⏚ badge, and what refuses Move).
    fixed: bool,
    /// The library entry no longer matches its store source (the ↻ badge).
    outdated: bool,
    /// Worst constraint status referencing this component, if any.
    status: Option<String>,
    /// Every member solid currently visible.
    visible: bool,
    /// Member scene names, for the visibility toggle.
    solids: Vec<String>,
    /// Read-only nested component rows, from the member name chains. FULL
    /// depth: a sub-assembly inside a sub-assembly renders as such.
    children: Vec<ChainNode>,
}

/// The BOM panel's transient UI state. The data lives in the document; the
/// column arrangement lives in the settings text; this holds only what is true
/// for this session.
pub struct BomPanel {
    hits: HashMap<String, egui::Rect>,
    /// The widget's live column arrangement. Rebuilt from the settings text
    /// whenever that text changes, keeping session-only widths + sort.
    layout: ColumnLayout,
    /// The settings text `layout` was built from — the change detector.
    layout_source: String,
    /// The parsed configuration for `layout_source`.
    parsed: ParsedColumns,
    /// Packed (one row per distinct part + occurrence data) or unpacked (one
    /// row per placement).
    packed: bool,
    /// Rows explicitly collapsed, by row id (absent = open).
    collapsed: HashSet<String>,
    /// The Cut margin field's value while it is being dragged or typed. The
    /// margin is written once, when the edit ends — one undo step, not one
    /// per frame of a drag.
    margin_edit: Option<f64>,
    /// The PLM half: the server's BOM of the open revision, and part
    /// attribute edits waiting for the server (S6).
    plm: crate::panels::bom_plm::PlmBomView,
    configuration: crate::panels::bom_configuration::BomConfiguration,
    owner_key: Option<String>,
    plm_mode: bool,
}

impl Default for BomPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl BomPanel {
    pub fn new() -> Self {
        Self {
            hits: HashMap::new(),
            layout: ColumnLayout::default(),
            layout_source: String::new(),
            parsed: ParsedColumns::default(),
            // Packed is the BOM a person asks for: a parts list, not a
            // placement list.
            packed: true,
            collapsed: HashSet::new(),
            margin_edit: None,
            plm: Default::default(),
            configuration: Default::default(),
            owner_key: None,
            plm_mode: false,
        }
    }

    /// Draw the BOM. Snapshots the projection, draws the column tree, then
    /// applies at most one deferred engine mutation — the shared panel
    /// pattern, and the reason the draw can borrow `state` immutably.
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        state: &mut EngineState,
        store: &dyn ModelStore,
        updates: &UpdateComponents,
        document: Option<&str>,
    ) -> BomOutcome {
        self.hits.clear();
        // The PLM half (S6): answered edits land first, and the source
        // switch is offered only for a document that IS a PLM revision, in a
        // session signed in to that PLM.
        self.plm.settle(state);
        let plm_target = plm_target(store, document);
        self.owner_key = plm_target.as_ref().map(|(_,part,rev)|crate::plm::identity::document_key(part,rev));
        self.plm_mode = store.plm_client().is_some();
        if let Some(client) = store.plm_client() {
            let history: Value = serde_json::from_str(&state.history_request_json()).unwrap_or_default();
            let mut keys: Vec<String> = history["partsLibrary"].as_object().into_iter().flat_map(|l|l.values()).filter_map(|entry|entry["sourceKey"].as_str().filter(|key|crate::plm::bom::revision_of_document(key).is_some()).map(str::to_string)).collect();
            if let Some(key)=&self.owner_key { keys.push(key.clone()); }
            keys.sort(); keys.dedup();
            self.configuration.ensure(&client,keys);
            self.configuration.ui(ui,&client,&mut self.hits);
        }
        // What is actually VISIBLE of this pane. Every other rect below is a
        // raw LAYOUT rect, so a widget scrolled past the pane's edge is still
        // published while being unclickable — a headed verifier has to scroll
        // it into this rect first. (The constraints panel publishes
        // `acon:panel:clip` for exactly the same reason.)
        self.hits.insert("bom:panel:clip".into(), ui.clip_rect());
        let mut outcome = BomOutcome::default();

        self.sync_columns(state);
        let component_rows = assembly_components::snapshot(state, updates);
        let mut occurrences = occurrences_from(state, &component_rows);
        if let Some(owner)=&self.owner_key {
            for occurrence in &mut occurrences {if let Some(attributes)=self.configuration.occurrence(owner,&occurrence.id){occurrence.attributes=attributes;}}
        }
        let packing_fields = if self.plm_mode {occurrences.iter().flat_map(|o|o.attributes.as_object().into_iter().flat_map(|a|a.keys().cloned())).collect::<std::collections::BTreeSet<_>>().into_iter().collect()}else{self.packing_fields()};
        let groups = group(&occurrences, self.packed, &packing_fields);
        let wires = state.wire_bom_lines();
        let margin = state.wire_harness_state().cut_margin;
        let mut margin_commit: Option<f64> = None;

        // --- the source: this document's BOM, or the PLM's -------------------
        if plm_target.is_some() {
            ui.horizontal(|ui| {
                let document = ui.selectable_label(!self.plm.active, "Document").on_hover_text("This document's own BOM");
                self.hits.insert("bom:source:document".into(), document.rect);
                if document.clicked() {
                    self.plm.active = false;
                }
                let plm = ui
                    .selectable_label(self.plm.active, "PLM")
                    .on_hover_text("The PLM's BOM of this revision: every level, part numbers, catalog values, sourcing and costs");
                self.hits.insert("bom:source:plm".into(), plm.rect);
                if plm.clicked() {
                    self.plm.active = true;
                }
            });
        } else {
            self.plm.active = false;
        }
        if let (true, Some((client, part, revision))) = (self.plm.active, &plm_target) {
            self.plm.ui(ui, state, client, part, revision, &self.configuration, &mut self.hits);
            if crate::automation::registry::enabled() {
                self.publish_plm(true);
                self.publish_hits();
            }
            return outcome;
        }

        // --- header: the packed/unpacked switch ------------------------------
        ui.horizontal(|ui| {
            let packed = ui
                .selectable_label(self.packed, "Packed")
                .on_hover_text("One row per part, rolled up where every occurrence field matches");
            self.hits.insert("bom:packed".into(), packed.rect);
            if packed.clicked() {
                self.packed = true;
            }
            let unpacked = ui
                .selectable_label(!self.packed, "Unpacked")
                .on_hover_text("One row per individual instance");
            self.hits.insert("bom:unpacked".into(), unpacked.rect);
            if unpacked.clicked() {
                self.packed = false;
            }
            let expand = ui
                .button("Expand all")
                .on_hover_text("Expand every row with nested components");
            self.hits.insert("bom:expand-all".into(), expand.rect);
            if expand.clicked() {
                self.collapsed.clear();
            }
            let collapse = ui
                .button("Collapse all")
                .on_hover_text("Collapse every row with nested components");
            self.hits.insert("bom:collapse-all".into(), collapse.rect);
            if collapse.clicked() {
                // Every key the tree can hold: the group rows and, beneath
                // them, every nested chain node — collapse-all has to fold the
                // WHOLE tree, at every depth, with no key drift.
                self.collapsed = collapsible_keys(&groups);
            }
            // The cut margin every wire's MF QTY adds to its routed length.
            // Offered once there is a wire to add it to (or a margin already
            // set), and written when the drag or the typing ENDS.
            if !wires.is_empty() || margin != 0.0 {
                ui.label("Cut margin");
                let mut value = self.margin_edit.unwrap_or(margin);
                let field = ui
                    .add(egui::DragValue::new(&mut value).range(0.0..=f64::MAX).speed(0.5))
                    .on_hover_text("Added to EVERY wire's routed length to give its cut length (MF QTY) — once per wire");
                self.hits.insert("bom:cut-margin".into(), field.rect);
                if field.changed() {
                    self.margin_edit = Some(value);
                }
                let editing = field.dragged() || field.has_focus();
                if !editing {
                    if let Some(edited) = self.margin_edit.take() {
                        if edited != margin {
                            margin_commit = Some(edited);
                        }
                    }
                }
            }
            let wire_count = if wires.is_empty() {
                String::new()
            } else {
                format!(" / {} wire{}", wires.len(), if wires.len() == 1 { "" } else { "s" })
            };
            ui.label(
                egui::RichText::new(format!(
                    "{} rows / {} occurrences{wire_count}",
                    groups.len() + wires.len(),
                    occurrences.len()
                ))
                .weak(),
            );
        });
        if updates.outdated_count() > 0 {
            let refresh = ui.button(format!("Update components ({})", updates.outdated_count()))
                .on_hover_text("Load the latest saved source models and rebuild every affected instance. One undo step.");
            self.hits.insert("bom:update-components".into(), refresh.rect);
            outcome.update_components |= refresh.clicked();
        }
        ui.add_space(2.0);

        // --- the tree ---------------------------------------------------------
        let mut rows: Vec<RowNode> = groups
            .iter()
            .map(|group| {
                let mut row = self.row_for(state, group);
                plm_cells(&mut row, state, store, &group.part_name);
                row
            })
            .collect();
        rows.extend(wires.iter().map(wire_row));
        let mut specs = bom_columns::column_specs(&self.parsed);
        if self.plm_mode {for spec in &mut specs {if let Some(field)=self.configuration.field(&spec.key){spec.label=if field.unit.is_empty(){field.name.clone()}else{format!("{} ({})",field.name,field.unit)};spec.kind=field.cell_kind();}}}
        let mut root_cells: HashMap<String, Value> = HashMap::new();
        root_cells.insert(
            QUANTITY_KEY.to_string(),
            Value::from(occurrences.len() as u64),
        );
        let spec = ColumnTreeSpec {
            id: "bom",
            columns: &specs,
            root_label: Some("Assembly"),
            root_cells: Some(&root_cells),
            empty_hint: Some("(no components or harness wires — insert a component via Add new feature)"),
            hits_prefix: "",
        };
        let out = column_tree::column_tree(
            ui,
            &spec,
            &mut self.layout,
            &rows,
            Some(&mut self.hits),
        );

        // --- act on what the widget reported ---------------------------------
        if out.layout_changed {
            self.persist_layout(state, store);
        }
        if let Some(id) = &out.toggled {
            if !self.collapsed.remove(id) {
                self.collapsed.insert(id.clone());
            }
        }
        if let Some(id) = &out.clicked {
            if let Some(group) = groups.iter().find(|group| group.key == *id) {
                state.select_components(&group.ids);
            }
        }
        // A DOUBLE click opens the row's dialog: the component's feature, exactly
        // as the menu's Edit feature does. A nested sub-assembly row owns no
        // feature in this document and names no group, so it opens nothing.
        if let Some(id) = &out.double_clicked {
            if let Some(first) = groups
                .iter()
                .find(|group| group.key == *id)
                .and_then(|group| group.ids.first())
            {
                if let Some(index) = state.history.index_of(first) {
                    state.roll_to(index);
                }
                outcome.focus = Some(first.clone());
            }
        }
        // The row menu. Engine-mutating actions run in the SHARED dispatcher
        // (one truth, one undo lane, the same one the structure tree's buttons
        // and the context bar use); the two document-level flows come back as
        // a request for the shell.
        let mut acted = false;
        for click in &out.actions {
            let Some(group) = groups.iter().find(|group| group.key == click.row_id) else {
                continue;
            };
            let Some(first) = group.ids.first() else {
                continue;
            };
            acted = true;
            if click.action == "update-components" {
                outcome.update_components = true;
            } else if click.action == EDIT_FEATURE {
                if let Some(index) = state.history.index_of(first) {
                    state.roll_to(index);
                }
                outcome.focus = Some(first.clone());
            } else if let Some(action) = ComponentAction::from_id(&click.action) {
                outcome.component = run_component_action(state, action, first);
            }
        }
        if let Some(edited) = margin_commit {
            if let Err(error) = state.wire_harness_set_cut_margin(edited) {
                state.push_notice(format!("BOM: {error}"));
            }
            acted = true;
        }
        // At most ONE edit lands per frame (egui gives one widget the focus),
        // and applying it re-runs the history, so take the first and let the
        // next frame carry any other. An action that just deleted the feature
        // this edit names would make the write fail loudly, so the action wins
        // the frame and the edit comes back on the next one.
        if !acted {
            if let Some(edit) = out.edits.first() {
                if edit.column == VISIBLE_KEY {
                    // Scene state, not a stored attribute: write it straight
                    // through to every member solid the row stands for.
                    let visible = edit.value.as_bool().unwrap_or(true);
                    if let Some(group) = groups.iter().find(|g| g.key == edit.row_id) {
                        for solid in &group.solids {
                            state.set_visible(solid, visible);
                        }
                    }
                } else {
                    self.apply_edit(state, store, &groups, edit);
                }
            }
        }

        // The component oracle the headed verifiers read. Published from the
        // shared projection rather than from these rows, so it stays engine
        // truth: `verify_bom_menu` uses it to prove a menu action reached the
        // engine, and proving that against the BOM's own rendering would be
        // checking the panel against itself.
        assembly_components::publish_tree(&component_rows);

        if crate::automation::registry::enabled() {
            let listing: Vec<Value> = groups
                .iter()
                .map(|group| {
                    serde_json::json!({
                        "key": group.key,
                        "partName": group.part_name,
                        "ids": group.ids,
                        "quantity": group.ids.len(),
                        // The PLM's part number, revision and lifecycle for a
                        // component placed from a PLM revision (null otherwise).
                        "plm": plm_entry(state, store, &group.part_name).map(|e| serde_json::json!({
                            "partNumber": e.part_number, "revision": e.revision_label, "lifecycle": e.lifecycle,
                        })),
                    })
                })
                .collect();
            crate::automation::registry::publish("__brepBom", "BOM groups {key, partName, ids, quantity}", &Value::Array(listing).to_string());
            let wire_listing: Vec<Value> = wires
                .iter()
                .map(|line| {
                    serde_json::json!({
                        "key": wire_row_key(line),
                        "connectionId": line.connection_id,
                        "stockPartNumber": line.stock_part_number,
                        "length": line.length,
                        "margin": line.margin,
                        "mfQty": line.mf_qty,
                        "status": line.state.as_str(),
                    })
                })
                .collect();
            crate::automation::registry::publish("__brepBomWires", "BOM wire lines {key, connectionId, stockPartNumber, length, margin, mfQty, status}", &Value::Array(wire_listing).to_string());
            self.publish_plm(plm_target.is_some());
            self.publish_hits();
        }

        outcome
    }

    /// Rebuild the column layout when the settings text has changed. Widths
    /// and sort are session state and survive the rebuild — a re-parse must
    /// not resize the table under the user's hands.
    fn sync_columns(&mut self, state: &EngineState) {
        let text = if self.plm_mode {self.configuration.text().unwrap_or_else(||bom_columns::effective_text(&state.settings.bom_columns))}else{bom_columns::effective_text(&state.settings.bom_columns)};
        if text == self.layout_source {
            return;
        }
        self.parsed = bom_columns::parse(&text);
        self.layout = bom_columns::layout_from(&self.parsed, &self.layout);
        self.layout_source = text;
    }

    /// Fold a layout the user changed BY DRAGGING back into the settings text,
    /// so the table and the configuration can never disagree.
    fn persist_layout(&mut self, state: &mut EngineState, store: &dyn ModelStore) {
        let columns = bom_columns::columns_from_layout(&self.parsed, &self.layout);
        if self.plm_mode {self.configuration.adopt_layout(&columns);return;}
        let text = bom_columns::serialize(
            &columns,
            &self.parsed.preserved,
            // Dragging a column across the freeze boundary moves the marker,
            // exactly as dragging one across another moves its line.
            bom_columns::frozen_from_layout(&self.layout),
        );
        let mut settings: Value =
            serde_json::from_str(&state.settings_json()).unwrap_or(Value::Null);
        let Some(object) = settings.as_object_mut() else {
            return;
        };
        object.insert("bomColumns".into(), Value::String(text.clone()));
        let json = settings.to_string();
        let _ = state.apply_settings_json(&json);
        let _ = store.write(crate::store::SETTINGS_KEY, &json);
        // Adopt it as our own source so `sync_columns` does not now rebuild
        // (and discard) the very layout the user just dragged.
        self.parsed = bom_columns::parse(&text);
        self.layout_source = text;
    }

    /// The occurrence fields a packed row is keyed by: the VISIBLE
    /// occurrence-scoped columns, in the arrangement's order.
    ///
    /// Visible, not every field: a BOM row stands for what the table SHOWS, so
    /// two placements that differ only in a column nobody is looking at are one
    /// line. Part-scoped columns are identical across every placement of a part
    /// by definition, so they cannot split a row and are not consulted; the
    /// derived quantity is not a field at all.
    fn packing_fields(&self) -> Vec<String> {
        self.parsed
            .columns
            .iter()
            .filter(|column| column.scope == Scope::Occurrence)
            .filter(|column| column.key() != QUANTITY_KEY && column.key() != MF_QTY_KEY)
            .filter(|column| !self.layout.hidden.contains(&column.key()))
            .map(|column| column.field.clone())
            .collect()
    }

    /// Build one widget row for a group: the tree cell, every configured
    /// column's value, and the read-only nested component rows beneath.
    fn row_for(&self, state: &EngineState, group: &Group) -> RowNode {
        let mut cells: HashMap<String, Value> = HashMap::new();
        let label = if self.packed {
            group.part_name.clone()
        } else {
            format!("{} ({})", group.part_name, group.key)
        };
        cells.insert(ITEM_KEY.to_string(), Value::String(label));
        cells.insert(VISIBLE_KEY.to_string(), Value::Bool(group.visible));
        cells.insert(FLAGS_KEY.to_string(), Value::Array(badges(group)));
        // Quantity is DERIVED — the size of the roll-up — and never stored.
        cells.insert(
            QUANTITY_KEY.to_string(),
            Value::from(group.ids.len() as u64),
        );
        // So is PMI: the annotation count of the PART's own document, which a
        // STEP assembly import lifts onto the part rather than onto each
        // instance. Blank at zero — a column of "0"s reads as noise, and the
        // question this answers is "which parts carry PMI".
        let pmi = state.part_pmi_count(&group.part_name);
        if pmi > 0 {
            cells.insert(PMI_KEY.to_string(), Value::from(pmi as u64));
        }

        let part_attributes = state.part_attributes(&group.part_name);
        for column in &self.parsed.columns {
            let key = column.key();
            if key == QUANTITY_KEY || key == PMI_KEY || key == MF_QTY_KEY {
                continue;
            }
            if self.plm_mode {
                if let Some(field)=self.configuration.field(&key) {
                    if let Some(target)=field.edit.as_ref().filter(|target|target.resource=="part") {
                        if let Some((source_key,_))=state.part_source(&group.part_name){
                            if let Some(snapshot)=self.configuration.snapshot(&source_key){if let Some(value)=snapshot["record"].get(&target.key){cells.insert(key.clone(),value.clone());}}
                        }
                        continue;
                    }
                    if field.scope=="part" {
                        if let Some((source_key,_))=state.part_source(&group.part_name){
                            if let Some(snapshot)=self.configuration.snapshot(&source_key){
                                if snapshot["part_type"].as_str()==field.part_type.as_deref(){if let Some(value)=snapshot["attributes"].get(&field.key){cells.insert(key.clone(),value.clone());}}
                            }
                        }
                        continue;
                    }
                    if field.id=="builtin.total"{cells.insert(key.clone(),Value::from(group.ids.len() as u64));continue;}
                }
            }
            let source = match column.scope {
                Scope::Part => &part_attributes,
                Scope::Occurrence => &group.attributes,
            };
            if let Some(value) = source.get(&column.field) {
                cells.insert(key, value.clone());
            }
        }

        RowNode {
            id: group.key.clone(),
            cells,
            editable: true,
            selected: group.selected,
            expanded: !self.collapsed.contains(&group.key),
            actions: actions_for(state, group),
            // Nested components belong to the sub-assembly's own document, so
            // they show but never take an edit (rigid nesting). Rendered to
            // FULL depth: a sub-assembly inside a sub-assembly is a real thing
            // in the model and the list has to be able to show it.
            children: chain_rows(&group.key, &group.children),
        }
    }

    /// Route ONE cell edit to its store. The column's scope decides which:
    /// occurrence fields fan out across the group's ACOMPs in one undo step,
    /// part fields go to the part document and then through the shared
    /// write-through.
    fn apply_edit(
        &mut self,
        state: &mut EngineState,
        store: &dyn ModelStore,
        groups: &[Group],
        edit: &CellEdit,
    ) {
        let Some(group) = groups.iter().find(|group| group.key == edit.row_id) else {
            return; // a nested sub-assembly row — read-only, nothing to write
        };
        let Some(column) = self
            .parsed
            .columns
            .iter()
            .find(|column| column.key() == edit.column)
        else {
            return;
        };
        if self.plm_mode {
            if let Some(field)=self.configuration.field(&column.key()).cloned(){
                if !field.editable {return;}
                if let Some(client)=store.plm_client(){
                    if let Some(target)=field.edit.as_ref().filter(|target|target.resource=="part") {
                        if let Some((key,_))=state.part_source(&group.part_name){if let Some((part,_))=crate::plm::bom::revision_of_document(&key){self.plm.queue_part_record_patch(&client,&part,&group.part_name,&target.key,&field.cad_field,edit.value.clone());return;}}
                    }
                    if field.scope=="part"{
                        if let Some((key,_))=state.part_source(&group.part_name){
                            if let Some((part,rev))=crate::plm::bom::revision_of_document(&key){
                                if self.configuration.snapshot(&key).is_some_and(|s|s["part_type"].as_str()==field.part_type.as_deref()){
                                    self.plm.queue_revision_patch(&client,&part,&rev,&group.part_name,&field.key,edit.value.clone());
                                }
                                return;
                            }
                        }
                    }else if field.scope=="occurrence"{
                        if let Some(owner)=&self.owner_key{
                            if group.ids.iter().all(|id|self.configuration.occurrence(owner,id).is_some()){
                                if let Some((part,rev))=crate::plm::bom::revision_of_document(owner){self.plm.queue_occurrence_patch(&client,&part,&rev,&group.ids,&field.cad_field,edit.value.clone(),true);return;}
                            }
                        }
                    }
                }
            }
        }
        match column.scope {
            Scope::Occurrence => {
                // The fan-out: EVERY occurrence the packed row rolls up, as
                // ONE undo step.
                if let Err(error) =
                    state.set_occurrence_attribute(&group.ids, &column.field, edit.value.clone())
                {
                    state.push_notice(format!("BOM: {error}"));
                }
            }
            Scope::Part => {
                // The part's `(sourceKey, signature-as-inserted)` must be read
                // BEFORE the edit re-stamps the signature — that pair is what
                // the write-through compares the file against.
                let target = state.part_source(&group.part_name).and_then(|(key, sig)| {
                    (!key.is_empty()).then_some((key, sig))
                });
                // A part that came from the PLM: its fields are the server's
                // (D11). The edit is a PATCH of that part and changes nothing
                // here until the server accepts it.
                let plm_part = target.as_ref().and_then(|(key, _)| crate::plm::bom::revision_of_document(key));
                if let (Some((part_id, _)), Some(client)) = (plm_part, store.plm_client()) {
                    if PLM_PART_FIELDS.iter().any(|f| column.field.eq_ignore_ascii_case(f)) {
                        state.push_notice(format!(
                            "BOM: {}'s {} is the PLM's; it is not edited here",
                            group.part_name,
                            column.field.replace('_', " ").to_lowercase()
                        ));
                        return;
                    }
                    self.plm.queue_patch(&client, &part_id, &group.part_name, &column.field, edit.value.clone());
                    return;
                }
                if let Err(error) =
                    state.set_part_attribute(&group.part_name, &column.field, edit.value.clone())
                {
                    state.push_notice(format!("BOM: {error}"));
                    return;
                }
                // The part document that just changed is saved back to the file
                // it came from, through the shared write-through lane, so the
                // entry's signature and the file agree.
                if let Some(document) = state.part_document_json(&group.part_name) {
                    parts_library::write_through(
                        state,
                        store,
                        &group.part_name,
                        target.as_ref(),
                        &document,
                    );
                }
            }
        }
    }

    /// The `__brepPlmBom` state (S6).
    fn publish_plm(&self, available: bool) {
        crate::automation::registry::publish(
            "__brepPlmBom",
            "the BOM panel's PLM view: {available, active, flat, loading, error, number, revision, lines:[server lines], totals, unpriced, warnings, document:[the document's lines in the same terms], documentRefused, differences, pendingEdits, lastEdit}",
            &self.plm.state_json(available).to_string(),
        );
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
        crate::automation::registry::publish("__brepBomHit", "BOM widget rects (BOM:, cell:)", &self.hits_json());
    }
}

/// The row menu for one group: "Edit feature" (this panel's own) then the
/// SHARED component actions in bar order, each refused-with-a-reason where this
/// row cannot honour it.
///
/// The per-INSTANCE actions (Move, Fix/Unfix, Delete) are refused on a PACKED
/// row that rolls up more than one placement: acting on "the first" of four is
/// a trap, and fanning Delete or Fix out across the group would be N undo steps
/// where every other BOM edit is one. The part-level flows (Edit in place, Open
/// Part) mean the same thing for every placement, so they stay live; and
/// "Edit feature" only rolls the history, which is what the ✎ button it
/// replaced always did.
fn actions_for(state: &EngineState, group: &Group) -> Vec<RowAction> {
    if group.ids.is_empty() {
        return Vec::new();
    }
    // The row's own grounded flag — the kernel record's, rolled up across the
    // group (see `Group::fixed`). For the single-placement row it IS the
    // placement's flag; for a packed row it only words the (disabled) toggle.
    // Not `component_info(first)`: that walks every scene solid with a history
    // scan each, and this runs once per row per frame.
    let fixed = group.fixed;
    let rolled_up = group.ids.len() > 1;
    let unpack = |verb: &str| {
        format!(
            "{} placements on this row — switch to Unpacked to {verb} one",
            group.ids.len()
        )
    };
    // An embedded-only part (no `sourceKey`) has no document to open.
    let embedded = !state
        .part_source(&group.part_name)
        .is_some_and(|(key, _)| !key.is_empty());

    let mut actions = vec![RowAction::new(EDIT_FEATURE, "\u{270E} Edit feature")
        .tooltip("Roll to this component's feature and open it in the history")];
    for action in ComponentAction::ALL {
        let entry = RowAction::new(action.id(), action.label(fixed)).tooltip(action.tooltip());
        let entry = match action {
            ComponentAction::Move if fixed => {
                entry.disabled("This component is fixed — unfix it before moving it")
            }
            ComponentAction::Move if rolled_up => entry.disabled(unpack("move")),
            ComponentAction::MoveCopy if rolled_up => entry.disabled(unpack("copy")),
            ComponentAction::ToggleFixed if rolled_up => entry.disabled(unpack("fix or unfix")),
            ComponentAction::Delete if rolled_up => entry.disabled(unpack("delete")),
            ComponentAction::OpenPart if embedded => {
                entry.disabled("This part is embedded in the assembly — it has no source document")
            }
            _ => entry,
        };
        actions.push(match action {
            // The destructive tail, fenced off from the rest.
            ComponentAction::Delete => entry.separator_above().destructive(),
            _ => entry,
        });
    }
    actions
}

/// A wire line's row id: `wire:` and the harness connection id, so it can
/// never collide with a component group (`pack:…`) or an ACOMP id.
fn wire_row_key(line: &WireBomLine) -> String {
    format!("wire:{}", line.id)
}

/// One harness wire as its own BOM line (see the module doc's "Wire lines").
/// Read-only: the wire's data is the Wire Harness panel's to edit.
fn wire_row(line: &WireBomLine) -> RowNode {
    let mut cells: HashMap<String, Value> = HashMap::new();
    cells.insert(ITEM_KEY.to_string(), Value::String(line.connection_id.clone()));
    cells.insert(QUANTITY_KEY.to_string(), Value::from(1_u64));
    if !line.stock_part_number.is_empty() {
        cells.insert("part.Part_Number".to_string(), Value::String(line.stock_part_number.clone()));
    }
    let mut flags = Vec::new();
    match line.mf_qty {
        // Three decimals: a micrometre in millimetre models, and no float
        // noise in a cell a person reads off.
        Some(qty) => {
            cells.insert(MF_QTY_KEY.to_string(), Value::from((qty * 1000.0).round() / 1000.0));
        }
        None => {
            cells.insert(MF_QTY_KEY.to_string(), Value::String(line.state.describe()));
            flags.push(serde_json::json!({
                "glyph": "\u{25CF}",
                "color": color_hex(assembly_components::OUTDATED_AMBER),
                "tooltip": format!("No cut length: {}", line.state.explain()),
            }));
        }
    }
    cells.insert(FLAGS_KEY.to_string(), Value::Array(flags));
    RowNode {
        id: wire_row_key(line),
        cells,
        editable: false,
        selected: false,
        expanded: false,
        actions: Vec::new(),
        children: Vec::new(),
    }
}

/// Every key the tree can place in `collapsed`: each group row that has nested
/// components, and every nested chain node beneath it that has children of its
/// own. Collapse-all writes exactly this set.
fn collapsible_keys(groups: &[Group]) -> HashSet<String> {
    /// Does this node own a nested COMPONENT anywhere below it? Bodies do not
    /// count — they are not drawn, so a node holding only bodies has nothing to
    /// collapse and must not claim a key.
    fn owns_component(nodes: &[ChainNode]) -> bool {
        nodes
            .iter()
            .any(|node| assembly_components::is_acomp_segment(&node.label))
    }
    fn walk(parent: &str, nodes: &[ChainNode], out: &mut HashSet<String>) {
        for node in nodes
            .iter()
            .filter(|node| assembly_components::is_acomp_segment(&node.label))
        {
            let id = format!("{parent}:{}", node.label);
            if owns_component(&node.children) {
                out.insert(id.clone());
            }
            walk(&id, &node.children, out);
        }
    }
    let mut out = HashSet::new();
    for group in groups {
        if owns_component(&group.children) {
            out.insert(group.key.clone());
        }
        walk(&group.key, &group.children, &mut out);
    }
    out
}

/// The row's status glyphs: grounded, outdated, and the worst constraint
/// status referencing it. Colour carries the meaning for the last two, which is
/// why these are badges rather than text.
fn badges(group: &Group) -> Vec<Value> {
    let mut out = Vec::new();
    if group.fixed {
        out.push(serde_json::json!({
            "glyph": assembly_components::FIXED_GLYPH,
            "tooltip": "Grounded — unfix it before moving it",
        }));
    }
    if group.outdated {
        out.push(serde_json::json!({
            "glyph": assembly_components::OUTDATED_GLYPH,
            "color": color_hex(assembly_components::OUTDATED_AMBER),
            "tooltip": "Source changed — click to update all outdated components from their saved models",
            "action": "update-components",
        }));
    }
    if let Some(status) = &group.status {
        out.push(serde_json::json!({
            "glyph": "\u{25CF}",
            "color": brep_render::assembly_status::status_color_hex(status),
            "tooltip": format!("Constraint status: {status}"),
        }));
    }
    out
}

/// `Color32` → the `#rrggbb` the widget's badge cell parses. (Constraint
/// statuses have their own [`brep_render::assembly_status::status_color_hex`];
/// this is for the badge colours the app owns.)
fn color_hex(color: egui::Color32) -> String {
    crate::color::rgb_to_hex([color.r(), color.g(), color.b()])
}

/// Nested COMPONENT rows for one group, to full depth. Read-only throughout:
/// these belong to the sub-assembly's own document, so they carry no cells the
/// BOM may edit and offer no actions in THIS document.
///
/// Only `ACOMP<n>` nodes appear. A BOM lists PARTS and the sub-assemblies a
/// part contains — the bodies inside a part are that part's internals and live
/// on the Scene tree, not here. Filtering recursively also means a part whose
/// chain holds nothing but bodies ends up with no children at all, so the
/// widget draws no collapse box on a row with nothing behind it.
fn chain_rows(parent: &str, nodes: &[ChainNode]) -> Vec<RowNode> {
    nodes
        .iter()
        .filter(|node| assembly_components::is_acomp_segment(&node.label))
        .map(|node| {
            let id = format!("{parent}:{}", node.label);
            let mut cells = HashMap::new();
            cells.insert(ITEM_KEY.to_string(), Value::String(node.label.clone()));
            RowNode {
                children: chain_rows(&id, &node.children),
                id,
                cells,
                editable: false,
                selected: false,
                expanded: false,
                actions: Vec::new(),
            }
        })
        .collect()
}

/// Is `candidate` a worse constraint status than `current`? Uses the ONE status
/// map's severity ordering, so a rolled-up row shows the worst of what it
/// stands for rather than whichever placement happened to be first.
fn worse_status(current: Option<&str>, candidate: Option<&str>) -> bool {
    let Some(candidate) = candidate else {
        return false;
    };
    match current {
        None => true,
        Some(current) => {
            brep_render::assembly_status::status_severity(candidate)
                > brep_render::assembly_status::status_severity(current)
        }
    }
}

/// One BOM row's occurrences: the whole group in the packed view, exactly one
/// in the unpacked view.
struct Group {
    /// The row id. In the packed view this is a synthetic group key; in the
    /// unpacked view it is the ACOMP id itself.
    key: String,
    part_name: String,
    /// Every ACOMP this row stands for — what a packed edit fans out across.
    ids: Vec<String>,
    /// The occurrence attributes shared by the whole group (identical by
    /// construction — that is what made them one group).
    attributes: Value,
    selected: bool,
    /// Rolled up across the group: grounded only when EVERY placement is.
    fixed: bool,
    outdated: bool,
    /// Worst status across the group's placements.
    status: Option<String>,
    /// Visible only when EVERY member solid of every placement is.
    visible: bool,
    /// Every member solid the row stands for — what the toggle writes to.
    solids: Vec<String>,
    children: Vec<ChainNode>,
}

/// Flatten the engine's component projection into occurrences.
///
/// The per-component truth (fixed, outdated, constraint-status rollup,
/// visibility, the nested chain) comes from the SHARED projection in
/// [`assembly_components`] — the same rows the headed verifiers read as
/// `__brepAssemblyTree`. The BOM adds only what is its own: the attribute
/// records it edits.
fn occurrences_from(state: &mut EngineState, rows: &[ComponentRow]) -> Vec<Occurrence> {
    // ONE pass over the history for every component's attributes, not one
    // `index_of` scan per row.
    let mut attributes = state.occurrence_attributes_all();
    rows.iter()
        .map(|row| Occurrence {
            attributes: attributes
                .remove(&row.id)
                .unwrap_or_else(|| Value::Object(serde_json::Map::new())),
            selected: row.selected,
            fixed: row.fixed,
            outdated: row.outdated,
            status: row.rollup_status.clone(),
            visible: row.visible,
            solids: row.solids.clone(),
            children: row.children.clone(),
            part_name: row.part_name.clone(),
            id: row.id.clone(),
        })
        .collect()
}

/// Group occurrences into BOM rows.
///
/// PACKED rolls up by `(part name, EVERY occurrence field)` — the owner's rule:
/// occurrences that differ in ANY occurrence field stay separate rows, because
/// a rolled-up row would have to show one of two different values and an edit
/// to it would silently overwrite the other. UNPACKED is one row each.
///
/// Group order follows first appearance, which is the engine's deterministic
/// id order, so the table is stable frame to frame before any sort.
fn group(occurrences: &[Occurrence], packed: bool, fields: &[String]) -> Vec<Group> {
    if !packed {
        return occurrences
            .iter()
            .map(|occurrence| Group {
                key: occurrence.id.clone(),
                part_name: occurrence.part_name.clone(),
                ids: vec![occurrence.id.clone()],
                attributes: occurrence.attributes.clone(),
                selected: occurrence.selected,
                fixed: occurrence.fixed,
                outdated: occurrence.outdated,
                status: occurrence.status.clone(),
                visible: occurrence.visible,
                solids: occurrence.solids.clone(),
                children: occurrence.children.clone(),
            })
            .collect();
    }
    let mut order: Vec<String> = Vec::new();
    let mut groups: HashMap<String, Group> = HashMap::new();
    for occurrence in occurrences {
        let key = format!(
            "{}\u{1}{}",
            occurrence.part_name,
            canonical_over(&occurrence.attributes, fields)
        );
        match groups.get_mut(&key) {
            Some(group) => {
                group.ids.push(occurrence.id.clone());
                group.selected |= occurrence.selected;
                // A rolled-up row states what is true of EVERY placement it
                // stands for: grounded only if all are, visible only if all
                // are. Anything else would let one row claim a state a
                // placement behind it does not have.
                group.fixed &= occurrence.fixed;
                group.visible &= occurrence.visible;
                group.outdated |= occurrence.outdated;
                group.solids.extend(occurrence.solids.iter().cloned());
                if worse_status(group.status.as_deref(), occurrence.status.as_deref()) {
                    group.status = occurrence.status.clone();
                }
                for child in &occurrence.children {
                    if !group.children.iter().any(|kept| kept == child) {
                        group.children.push(child.clone());
                    }
                }
            }
            None => {
                order.push(key.clone());
                groups.insert(
                    key,
                    Group {
                        key: String::new(), // filled below, from the group order
                        part_name: occurrence.part_name.clone(),
                        ids: vec![occurrence.id.clone()],
                        attributes: occurrence.attributes.clone(),
                        selected: occurrence.selected,
                        fixed: occurrence.fixed,
                        outdated: occurrence.outdated,
                        status: occurrence.status.clone(),
                        visible: occurrence.visible,
                        solids: occurrence.solids.clone(),
                        children: occurrence.children.clone(),
                    },
                );
            }
        }
    }
    order
        .into_iter()
        .filter_map(|key| groups.remove(&key))
        .map(|mut group| {
            // The row id must be STABLE across frames (it keys collapse state
            // and every out-value) but must not be a raw attribute dump. The
            // first ACOMP of the group is both — deterministic, because the
            // projection is in id order.
            group.key = format!(
                "pack:{}",
                group.ids.first().cloned().unwrap_or_default()
            );
            group
        })
        .collect()
}

/// The packing key's value half: the named fields, in the given order, with a
/// missing field spelled explicitly. Order comes from the column arrangement
/// rather than the record, so two placements whose attributes were WRITTEN in a
/// different order still key the same. (serde_json runs with `preserve_order`
/// in this workspace, so a naive `to_string` of the record would not.)
fn canonical_over(attributes: &Value, fields: &[String]) -> String {
    fields
        .iter()
        .map(|field| {
            let value = attributes
                .get(field)
                .map(Value::to_string)
                .unwrap_or_default();
            format!("{field}={value}")
        })
        .collect::<Vec<_>>()
        .join("\u{2}")
}


/// The part fields that ARE the PLM's for a component placed from a PLM
/// revision: shown from the store index the session holds, never edited here.
pub(crate) const PLM_PART_FIELDS: [&str; 3] = ["Part_Number", "Revision", "Lifecycle_State"];

/// A component placed from a PLM revision reads its part number, revision and
/// lifecycle from the PLM (the store index the session already holds, kept
/// current by the change feed; no request per row), so the Document view
/// reads as the PLM view does. Any other component keeps its document's own.
fn plm_cells(row: &mut RowNode, state: &EngineState, store: &dyn ModelStore, part_name: &str) {
    let Some(entry) = plm_entry(state, store, part_name) else { return };
    for (field, value) in [
        ("Part_Number", entry.part_number.clone()),
        ("Revision", entry.revision_label.clone()),
        ("Lifecycle_State", entry.lifecycle.clone()),
    ] {
        row.cells.insert(format!("part.{field}"), Value::String(value));
    }
}

/// The store index row of the PLM revision `part_name` was placed from.
fn plm_entry(state: &EngineState, store: &dyn ModelStore, part_name: &str) -> Option<crate::plm::client::IndexEntry> {
    let (key, _) = state.part_source(part_name)?;
    let (part, rev) = crate::plm::bom::revision_of_document(&key)?;
    store.plm_revision(&crate::plm::identity::document_key(&part, &rev))
}

/// The PLM revision the open document is, with the session's client
/// ([`ModelStore::plm_client`]) — `None` off a PLM, or for a document that
/// is no PLM revision.
fn plm_target(store: &dyn ModelStore, document: Option<&str>) -> Option<(std::rc::Rc<crate::plm::client::PlmClient>, String, String)> {
    let connected = matches!(
        store.plm_session().map(|s| s.status),
        Some(crate::plm::connection::Status::Connected { .. })
    );
    if !connected {
        return None;
    }
    let (part, revision) = crate::plm::bom::revision_of_document(document?)?;
    Some((store.plm_client()?, part, revision))
}

/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "bom", prefix: "bom:expand-all", meaning: "expand every group", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:collapse-all", meaning: "collapse every group", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:update-components", meaning: "update all outdated components from saved sources", command: Some("component_update") },
    HitKeyDoc { panel: "bom", prefix: "badge:", meaning: "an actionable row badge (badge:<row>:update-components)", command: Some("component_update") },
    HitKeyDoc { panel: "bom", prefix: "bom:packed", meaning: "packed view", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:unpacked", meaning: "unpacked view", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:panel:clip", meaning: "the visible region of the pane", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:source:", meaning: "the BOM's source, document or plm (only for a PLM revision in a PLM session)", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:plm:", meaning: "the PLM view: indented, flat, refresh, lines", command: None },
    HitKeyDoc { panel: "bom", prefix: "bom:cut-margin", meaning: "the cut margin added to every harness wire's routed length (MF QTY), shown once the document has a wire; written when the edit ends", command: Some("wire_harness_set_cut_margin") },
    // The row ACTION MENU, `column_tree`'s own keys under this panel's empty
    // prefix. Documented as `BOM:` until 2026-09-13, which matched nothing —
    // `documented` compares case-sensitively and the keys are `menu:` /
    // `menuitem:`. Nothing had caught it because `hit_keys_check` had never run
    // with a row menu OPEN: the entries only exist for the frames the popup is
    // up, and the trigger cell is in the rightmost column, which the shipped
    // column set puts off the pane's edge.
    HitKeyDoc { panel: "bom", prefix: "menu:", meaning: "a row's action-menu trigger cell (menu:<row key>) \u{2014} the rightmost column, so it may need the pane scrolled horizontally; a RIGHT-CLICK anywhere on the row opens the same menu", command: None },
    HitKeyDoc { panel: "bom", prefix: "menuitem:", meaning: "one entry of the OPEN row action menu (menuitem:<row key>:<action>, e.g. toggle-fixed, move, delete, edit-feature, open-part) \u{2014} published only while the menu is up", command: Some("component_set_fixed") },
    HitKeyDoc { panel: "bom", prefix: "cell:", meaning: "a table cell (cell:row:column)", command: Some("bom_set_occurrence_attribute") },
    HitKeyDoc { panel: "bom", prefix: "row:", meaning: "a structure-tree row (row:node key) \u{2014} a click selects its components, a double click opens the component's feature", command: Some("component_select") },
    HitKeyDoc { panel: "bom", prefix: "box:", meaning: "a structure-tree row's expander (box:node key)", command: None },
    HitKeyDoc { panel: "bom", prefix: "col:", meaning: "a table column header (col:field) — click to sort", command: None },
    HitKeyDoc { panel: "bom", prefix: "grip:", meaning: "a column's resize grip (grip:field)", command: None },
    HitKeyDoc { panel: "bom", prefix: "freeze:divider", meaning: "the frozen-column divider", command: None },
];
