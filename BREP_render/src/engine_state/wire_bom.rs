//! Wire BOM lines — every harness wire as its own BOM line, carrying the cut
//! length an automatic wire-cutting machine reads (MF QTY).
//!
//! # The number is the router's
//!
//! A line's length is [`brep_kernel::RouteResult::length`] from the routing
//! report of the last APPLIED history run: the sum of the arc lengths of every
//! spline the wire crosses, measured port base point to port base point (the
//! ports' straight `extension` runs are pieces of each spline's chain). This
//! module computes no second length. MF QTY is that length plus the block's
//! [`cutMargin`](brep_kernel::WireHarnessState::cut_margin), added once to each
//! wire — never a percentage, never once per line.
//!
//! # No number unless the run is the model's
//!
//! Routing results are not saved; the report is replaced by every applied run.
//! So a length can be wrong in only these ways, and each is a state the line
//! states instead of a number ([`WireLengthState`]):
//!
//! * a run was submitted and has not landed ([`EngineState::run_pending`]) —
//!   the report on hand routed the model as it was BEFORE the edit;
//! * that run was CANCELLED ([`EngineState::cancelled_run`]): nothing is
//!   pending any more, but the model shows the last COMPLETED result, from
//!   before the cancelled edit, until the next edit submits a fresh run;
//! * no run has landed at all (a fresh engine, or a history that failed to
//!   parse, which clears the report);
//! * the history is ROLLED BACK. The tail routes the prefix it ran, so a spline
//!   past the rollback is not in the network and a wire can route over a
//!   different path and still read `routed` — a current run, a wrong length.
//!   Measured: see `a_rolled_back_history_routes_a_different_path`;
//! * the router could not route the wire: the line carries its status.
//!
//! A silently stale cut length costs a reel of wire; a blank one with a reason
//! costs a rebuild.

use super::*;
use brep_kernel::RouteStatus;

/// Why a wire line has, or has not, a cut length.
#[derive(Debug, Clone, PartialEq)]
pub enum WireLengthState {
    /// The last applied run routed the whole model, and this wire through it.
    Current,
    /// A run is in flight; the report on hand is from before the last edit.
    RunPending,
    /// The last run was cancelled: the report on hand is the last COMPLETED
    /// run's, from before the edit that was cancelled.
    Cancelled,
    /// No run has been applied (a fresh engine, or a history that failed to
    /// parse), or the run on hand has no route for this wire.
    NoRun,
    /// The history is rolled back: the run routed a prefix of the model, which
    /// can be a different path through fewer splines.
    RolledBack,
    /// The router could not route the wire; the status and its sentence are
    /// the router's own.
    Unrouted { status: RouteStatus, message: String },
}

impl WireLengthState {
    /// The kebab-case word the BOM keys on (`current`, `routing`, `no-run`,
    /// `rolled-back`, or the router's status word).
    pub fn as_str(&self) -> &'static str {
        match self {
            WireLengthState::Current => "current",
            WireLengthState::RunPending => "routing",
            WireLengthState::Cancelled => "cancelled",
            WireLengthState::NoRun => "no-run",
            WireLengthState::RolledBack => "rolled-back",
            WireLengthState::Unrouted { status, .. } => status.as_str(),
        }
    }

    /// What the MF QTY cell says in place of a number: short, and never
    /// something that reads as a length.
    pub fn describe(&self) -> String {
        match self {
            WireLengthState::Current => String::new(),
            WireLengthState::RunPending => "routing…".into(),
            WireLengthState::Cancelled => "rebuild cancelled".into(),
            WireLengthState::NoRun => "needs a rebuild".into(),
            WireLengthState::RolledBack => "rolled back".into(),
            WireLengthState::Unrouted { status, .. } => format!("not routed ({})", status.as_str()),
        }
    }

    /// The full sentence for a tooltip or an export's status column.
    pub fn explain(&self) -> String {
        match self {
            WireLengthState::Current => "routed by the current run".into(),
            WireLengthState::RunPending => {
                "a rebuild is running; the cut length appears when it lands".into()
            }
            WireLengthState::Cancelled => {
                "the last rebuild was cancelled, so the route on hand is from before your last edit; rebuild for cut lengths"
                    .into()
            }
            WireLengthState::NoRun => "no routing run has been applied; rebuild the model".into(),
            WireLengthState::RolledBack => {
                "the history is rolled back, so the route may miss later splines; roll to the end for cut lengths"
                    .into()
            }
            WireLengthState::Unrouted { message, .. } => message.clone(),
        }
    }
}

/// One wire as a BOM line.
#[derive(Debug, Clone, PartialEq)]
pub struct WireBomLine {
    /// The harness connection id (`wire-N`) — the line's stable key.
    pub id: String,
    /// The connection ID the line shows: the connection's name, else its id.
    pub connection_id: String,
    /// The stock part number of the wire. Blank for a hand-added harness wire;
    /// a wire a Diagram generates carries eCAD's.
    pub stock_part_number: String,
    /// The routed length, only when `state` is [`WireLengthState::Current`].
    pub length: Option<f64>,
    /// The block's cut margin, added once to this wire.
    pub margin: f64,
    /// The cut length: `length + margin`, only when `length` is.
    pub mf_qty: Option<f64>,
    pub state: WireLengthState,
}

impl EngineState {
    /// Every harness wire as a BOM line, in block order, each with the cut
    /// length of the last applied run — or, when that run does not speak for
    /// the model as it stands, the reason there is none (see the module doc).
    pub fn wire_bom_lines(&self) -> Vec<WireBomLine> {
        let state = self.wire_harness_state();
        let margin = state.cut_margin;
        let rolled_back = !self.history.is_empty() && self.history.rollback() + 1 < self.history.len();
        state
            .connections
            .iter()
            .map(|connection| {
                let route = self
                    .wire_harness_report
                    .as_ref()
                    .and_then(|report| report.routes.iter().find(|route| route.connection_id == connection.id));
                let (state, length) = if self.run_pending() {
                    (WireLengthState::RunPending, None)
                } else if self.cancelled_run.is_some() {
                    (WireLengthState::Cancelled, None)
                } else if self.wire_harness_report.is_none() {
                    (WireLengthState::NoRun, None)
                } else if rolled_back {
                    (WireLengthState::RolledBack, None)
                } else {
                    match route {
                        None => (WireLengthState::NoRun, None),
                        Some(route) if !route.feasible => (
                            WireLengthState::Unrouted {
                                status: route.status,
                                message: route.message.clone(),
                            },
                            None,
                        ),
                        Some(route) => match route.length {
                            Some(length) => (WireLengthState::Current, Some(length)),
                            None => (WireLengthState::NoRun, None),
                        },
                    }
                };
                let connection_id = if connection.name.trim().is_empty() {
                    connection.id.clone()
                } else {
                    connection.name.clone()
                };
                WireBomLine {
                    id: connection.id.clone(),
                    connection_id,
                    stock_part_number: connection.id.strip_prefix("diagram:")
                        .and_then(|id| self.history.diagram_block()?.get("wires")?.as_array()?.iter()
                            .find(|wire| wire["id"].as_str() == Some(id))?
                            .get("stock_part_number")?.as_str()).unwrap_or("").to_owned(),
                    length,
                    margin,
                    mf_qty: length.map(|length| length + margin),
                    state,
                }
            })
            .collect()
    }
}

