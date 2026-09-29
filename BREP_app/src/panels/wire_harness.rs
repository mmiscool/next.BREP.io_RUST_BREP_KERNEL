//! The Wire Harness panel — the connection list on the shared
//! [`crate::column_tree`] widget (its second consumer, as the widget's own
//! doc anticipated).
//!
//! One row per connection in the document's `wireHarness` block: its name,
//! its two endpoint PORTs (a dropdown of every termination port, shown by the
//! port's `portName` and stored by its feature id), its wire diameter, and —
//! from the last run's routing report — its routed length and status. Rows
//! are edited in place; every edit is a checkpointed document edit that
//! re-runs the history, whose tail re-routes (there is no Route button: the
//! model IS the router). Hovering a row highlights the two ports and the
//! bundle solids the wire runs through.
//!
//! The header carries **Add wire**, the **Build bundles** toggle (keep the
//! routing, skip the bundle solids) and the summary
//! `N connections | M endpoints | K routed`; below it the report's segment
//! problems (a spline attached at one end only) are listed so an unroutable
//! network explains itself.
//!
//! Everything the panel knows comes from the engine
//! ([`EngineState::wire_harness_state`] / [`EngineState::wire_harness_report`]);
//! this struct holds only the widget's transient state.

use crate::automation::hit_keys::HitKeyDoc;
use crate::column_tree::{self, CellKind, ColumnLayout, ColumnSpec, ColumnTreeSpec, RowAction, RowNode};
use brep_render::engine_state::{ConnectionPatch, EngineState};
use brep_render::brep_kernel::{BundleStatus, PortKind, RouteStatus, WireHarnessReport};
use eframe::egui;
use serde_json::Value;
use std::collections::HashMap;

const NAME: &str = "name";
const FROM: &str = "from";
const TO: &str = "to";
const DIAMETER: &str = "diameter";
const LENGTH: &str = "length";
const STATUS: &str = "status";
const ACTIONS: &str = "actions";
const REMOVE: &str = "remove";

/// Status colours: routed green, an authoring problem amber, a hard problem red.
const ROUTED_COLOR: &str = "#3fb950";
const WARN_COLOR: &str = "#d29922";
const ERROR_COLOR: &str = "#f85149";

/// The panel's transient UI state.
pub struct WireHarnessPanel {
    hits: HashMap<String, egui::Rect>,
    layout: ColumnLayout,
    columns: Vec<ColumnSpec>,
    /// The connection whose hover highlight is live, so leaving the rows (or
    /// moving to another row) ends it exactly once.
    hovered: Option<String>,
}

impl Default for WireHarnessPanel {
    fn default() -> Self {
        Self::new()
    }
}

impl WireHarnessPanel {
    pub fn new() -> Self {
        Self {
            hits: HashMap::new(),
            layout: ColumnLayout::default(),
            columns: Vec::new(),
            hovered: None,
        }
    }

    /// Draw the panel. Snapshots the block + report, draws the header and the
    /// column tree, then applies at most one deferred engine mutation — the
    /// shared panel pattern.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();
        self.hits.insert("wh:panel:clip".into(), ui.clip_rect());

        let harness = state.wire_harness_state();
        let empty = WireHarnessReport::default();
        let report = state.wire_harness_report().cloned().unwrap_or(empty);
        let endpoints = endpoint_choices(&report);
        let routed = report.routes.iter().filter(|route| route.feasible).count();

        // --- header -----------------------------------------------------------
        let mut add_clicked = false;
        let mut bundles = harness.build_bundles;
        let mut bundles_changed = false;
        ui.horizontal(|ui| {
            let add = ui
                .button("Add wire")
                .on_hover_text("Add a connection between two termination ports");
            self.hits.insert("wh:add".into(), add.rect);
            add_clicked = add.clicked();
            let toggle = ui
                .checkbox(&mut bundles, "Build bundles")
                .on_hover_text("Sweep a bundle solid along every routed segment (off keeps the routing only)");
            self.hits.insert("wh:bundles".into(), toggle.rect);
            bundles_changed = toggle.changed();
            ui.label(
                egui::RichText::new(format!(
                    "{} connection{} | {} endpoint{} | {routed} routed",
                    harness.connections.len(),
                    if harness.connections.len() == 1 { "" } else { "s" },
                    endpoints.len(),
                    if endpoints.len() == 1 { "" } else { "s" },
                ))
                .weak(),
            );
        });
        for problem in &report.segment_problems {
            ui.label(egui::RichText::new(problem).weak().color(hex(WARN_COLOR)));
        }
        // A bundle that is not simply there says why, on its own line, with the
        // kernel's whole sentence on hover. `bundles-off` is a choice rather than
        // a problem and says nothing; `built` has nothing to say.
        for bundle in &report.bundles {
            let (word, colour) = match bundle.status {
                BundleStatus::Built | BundleStatus::BundlesOff => continue,
                BundleStatus::TightBend => ("bend too tight for the bundle", WARN_COLOR),
                BundleStatus::BuildFailed => ("the bundle could not be swept", ERROR_COLOR),
            };
            let line = ui
                .label(
                    egui::RichText::new(format!("{}: no bundle solid — {word}", bundle.segment_id))
                        .weak()
                        .color(hex(colour)),
                )
                .on_hover_text(&bundle.error);
            self.hits
                .insert(format!("wh:bundle:{}", bundle.segment_id), line.rect);
        }
        ui.add_space(2.0);

        // --- the tree ---------------------------------------------------------
        self.sync_columns(&endpoints);
        let rows: Vec<RowNode> = harness
            .connections
            .iter()
            .map(|connection| {
                let route = report
                    .routes
                    .iter()
                    .find(|route| route.connection_id == connection.id);
                let (status_text, status_color, tooltip) = match route {
                    Some(route) if route.feasible => ("Routed", ROUTED_COLOR, String::new()),
                    Some(route) => (status_word(route.status), status_color(route.status), route.message.clone()),
                    None => ("Not routed", WARN_COLOR, "the model has not run yet".to_string()),
                };
                RowNode::new(&connection.id)
                    .cell(NAME, Value::String(connection.name.clone()))
                    .cell(FROM, Value::String(label_for(&endpoints, &connection.from)))
                    .cell(TO, Value::String(label_for(&endpoints, &connection.to)))
                    .cell(DIAMETER, Value::from(connection.diameter))
                    .cell(
                        LENGTH,
                        Value::String(
                            route
                                .and_then(|route| route.length)
                                .map(format_length)
                                .unwrap_or_else(|| "\u{2014}".to_string()),
                        ),
                    )
                    .cell(
                        STATUS,
                        serde_json::json!([{ "glyph": status_text, "color": status_color, "tooltip": tooltip }]),
                    )
                    .actions(vec![RowAction::new(REMOVE, "Remove wire")
                        .tooltip("Delete this connection")
                        .destructive()])
            })
            .collect();
        let spec = ColumnTreeSpec {
            id: "wire-harness",
            columns: &self.columns,
            root_label: Some("Wires"),
            root_cells: None,
            empty_hint: Some(
                "(no connections — add ports, attach spline ends to them, then Add wire)",
            ),
            hits_prefix: "wh:",
        };
        let out = column_tree::column_tree(ui, &spec, &mut self.layout, &rows, Some(&mut self.hits));

        // --- hover → viewport highlight -----------------------------------------
        if out.hovered != self.hovered {
            match &out.hovered {
                Some(id) => state.wire_harness_hover_connection(id),
                None => state.wire_harness_hover_end(),
            }
            self.hovered = out.hovered.clone();
        }

        // --- act on what the widget reported (at most one mutation) ------------
        if add_clicked {
            let terminations: Vec<&str> = endpoints
                .iter()
                .filter(|choice| choice.termination)
                .map(|choice| choice.id.as_str())
                .collect();
            let from = terminations.first().copied().unwrap_or("");
            let to = terminations.get(1).or(terminations.first()).copied().unwrap_or("");
            state.wire_harness_add_connection(from, to, 1.0);
            return;
        }
        if bundles_changed {
            state.wire_harness_set_build_bundles(bundles);
            return;
        }
        if let Some(click) = out.actions.first() {
            if click.action == REMOVE {
                if let Err(error) = state.wire_harness_remove_connection(&click.row_id) {
                    state.push_notice(error);
                }
            }
            return;
        }
        if let Some(edit) = out.edits.first() {
            let patch = match edit.column.as_str() {
                NAME => ConnectionPatch {
                    name: Some(edit.value.as_str().unwrap_or("").to_string()),
                    ..Default::default()
                },
                FROM => ConnectionPatch {
                    from: Some(id_for(&endpoints, edit.value.as_str().unwrap_or(""))),
                    ..Default::default()
                },
                TO => ConnectionPatch {
                    to: Some(id_for(&endpoints, edit.value.as_str().unwrap_or(""))),
                    ..Default::default()
                },
                DIAMETER => ConnectionPatch {
                    diameter: edit.value.as_f64(),
                    ..Default::default()
                },
                _ => return,
            };
            if let Err(error) = state.wire_harness_update_connection(&edit.row_id, &patch) {
                state.push_notice(error);
            }
        }
    }

    /// Rebuild the column specs when the endpoint choices change (the two
    /// port dropdowns list the current termination ports).
    fn sync_columns(&mut self, endpoints: &[EndpointChoice]) {
        let options: Vec<String> = endpoints
            .iter()
            .filter(|choice| choice.termination)
            .map(|choice| choice.label.clone())
            .collect();
        let same = self
            .columns
            .iter()
            .find(|column| column.key == FROM)
            .is_some_and(|column| column.kind == CellKind::Choice { options: options.clone() });
        if same {
            return;
        }
        self.columns = vec![
            ColumnSpec::new(NAME, "Wire", CellKind::Text).width(90.0),
            ColumnSpec::new(FROM, "From", CellKind::Choice { options: options.clone() }).width(110.0),
            ColumnSpec::new(TO, "To", CellKind::Choice { options }).width(110.0),
            ColumnSpec::new(DIAMETER, "Dia", CellKind::Numeric { step: 0.1 }).width(60.0),
            ColumnSpec::new(LENGTH, "Length", CellKind::ReadOnly).width(70.0),
            ColumnSpec::new(STATUS, "Status", CellKind::Badges).width(120.0),
            ColumnSpec::new(ACTIONS, "", CellKind::Actions { label: "\u{22EF}".into() }).width(30.0),
        ];
    }

    /// The per-frame widget rects for the headed verifier, as `[x, y, w, h]`
    /// like every other `__brep*Hit` map (`wh:add`, `wh:bundles`, `wh:panel:clip`,
    /// and the column tree's `wh:row:{id}` / `wh:cell:{id}:{column}` /
    /// `wh:menu:{id}` keys).
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
    }
}

/// One endpoint dropdown choice: the port id, the text shown for it, and
/// whether it is a termination (only those are offered as wire ends).
#[derive(Debug, Clone, PartialEq)]
struct EndpointChoice {
    id: String,
    label: String,
    termination: bool,
}

/// The endpoint choices from the report: every connection point, shown by its
/// ADDRESS. The address IS the identity now — `J1.VCC`, `ACOMP3:J1.VCC` — so
/// it is unique by construction and needs no disambiguation.
fn endpoint_choices(report: &WireHarnessReport) -> Vec<EndpointChoice> {
    report
        .endpoints
        .iter()
        .map(|endpoint| EndpointChoice {
            id: endpoint.id.clone(),
            label: endpoint.id.clone(),
            termination: endpoint.kind == PortKind::Termination,
        })
        .collect()
}

/// The dropdown text for a stored port id: its label, or the raw id when the
/// port is not in the model (so a dangling reference stays visible, and the
/// status column says why it is unrouted).
fn label_for(endpoints: &[EndpointChoice], id: &str) -> String {
    endpoints
        .iter()
        .find(|choice| choice.id == id)
        .map(|choice| choice.label.clone())
        .unwrap_or_else(|| id.to_string())
}

/// The port id behind a dropdown text (an empty choice clears the endpoint).
fn id_for(endpoints: &[EndpointChoice], label: &str) -> String {
    endpoints
        .iter()
        .find(|choice| choice.label == label)
        .map(|choice| choice.id.clone())
        .unwrap_or_else(|| label.to_string())
}

fn status_word(status: RouteStatus) -> &'static str {
    match status {
        RouteStatus::Routed => "Routed",
        RouteStatus::MissingEndpoint => "Missing port",
        RouteStatus::WaypointEndpoint => "Waypoint end",
        RouteStatus::SameEndpoint => "Same port",
        RouteStatus::NoSegments => "No splines",
        RouteStatus::NoRoute => "No route",
        RouteStatus::PortReuse => "Port reused",
    }
}

fn status_color(status: RouteStatus) -> &'static str {
    match status {
        RouteStatus::Routed => ROUTED_COLOR,
        RouteStatus::MissingEndpoint | RouteStatus::SameEndpoint | RouteStatus::NoSegments => WARN_COLOR,
        RouteStatus::WaypointEndpoint | RouteStatus::NoRoute | RouteStatus::PortReuse => ERROR_COLOR,
    }
}

/// A routed length with two decimals and no trailing zeros (`12.5`, `8`).
fn format_length(length: f64) -> String {
    brep_render::formatting::compact_decimal(length, 2)
}

fn hex(text: &str) -> egui::Color32 {
    let digits = text.trim_start_matches('#');
    let byte = |at: usize| u8::from_str_radix(&digits[at..at + 2], 16).unwrap_or(0);
    egui::Color32::from_rgb(byte(0), byte(2), byte(4))
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "wireharness", prefix: "wh:add", meaning: "add a connection", command: Some("wire_harness_add_connection") },
    HitKeyDoc { panel: "wireharness", prefix: "wh:bundles", meaning: "toggle bundle solids", command: Some("wire_harness_set_build_bundles") },
    HitKeyDoc { panel: "wireharness", prefix: "wh:bundle:", meaning: "a bundle's status line (hover for the reason)", command: None },
    HitKeyDoc { panel: "wireharness", prefix: "wh:panel:clip", meaning: "the visible region of the pane", command: None },
    HitKeyDoc { panel: "wireharness", prefix: "wh:", meaning: "a connection row control", command: Some("wire_harness_update_connection") },
];
