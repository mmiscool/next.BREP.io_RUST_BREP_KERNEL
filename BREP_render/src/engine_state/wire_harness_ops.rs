use super::*;
use brep_kernel::{WireHarnessConnection, WireHarnessReport, WireHarnessState};

/// A patch the panel applies to one connection: every field optional, only
/// the present ones change.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct ConnectionPatch {
    pub name: Option<String>,
    pub from: Option<String>,
    pub to: Option<String>,
    pub diameter: Option<f64>,
}

impl EngineState {
    // --- read surface ------------------------------------------------------

    /// The routing report of the last APPLIED run: endpoints (every port in
    /// the model), the harness segments, one route per connection, and the
    /// bundles. `None` before the first run.
    pub fn wire_harness_report(&self) -> Option<&WireHarnessReport> {
        self.wire_harness_report.as_ref()
    }

    /// The document's `wireHarness` block as typed state (the default — no
    /// connections — when the document carries none).
    pub fn wire_harness_state(&self) -> WireHarnessState {
        self.history
            .wire_harness_block()
            .and_then(|block| serde_json::from_value(block.clone()).ok())
            .unwrap_or_default()
    }

    /// The panel's verifier global: the block, the report, and the endpoint
    /// choices, as one JSON object.
    pub fn wire_harness_state_json(&self) -> String {
        let state = self.wire_harness_state();
        serde_json::json!({
            "connections": state.connections,
            "buildBundles": state.build_bundles,
            "cutMargin": state.cut_margin,
            "report": self.wire_harness_report,
        })
        .to_string()
    }

    // --- mutations (checkpointed document edits + re-run) -------------------

    /// Write `state` as the document's block (checkpointed) and re-run the
    /// history so the tail routes it. An empty block (no connections, a fresh
    /// counter and no cut margin) is removed from the document so a part that
    /// never had a harness saves byte-identically.
    fn write_wire_harness_state(&mut self, state: WireHarnessState) -> String {
        let block = if state.connections.is_empty() && state.id_counter == 0 && state.cut_margin == 0.0 {
            None
        } else {
            serde_json::to_value(&state).ok()
        };
        self.history.set_wire_harness_block(block);
        self.rerun_history()
    }

    /// Add a connection between two port ids (either may be empty — the panel
    /// fills them in) and return its minted id (`wire-N`, named `Wire N`).
    pub fn wire_harness_add_connection(&mut self, from: &str, to: &str, diameter: f64) -> String {
        let mut state = self.wire_harness_state();
        let id = state.next_id();
        let name = format!("Wire {}", state.id_counter);
        state.connections.push(WireHarnessConnection {
            id: id.clone(),
            name,
            from: from.to_string(),
            to: to.to_string(),
            diameter: if diameter.is_finite() && diameter > 0.0 { diameter } else { 1.0 },
        });
        self.write_wire_harness_state(state);
        id
    }

    /// Apply a patch to one connection. An unknown id is an error; a
    /// non-positive diameter is refused (the connection keeps its diameter).
    pub fn wire_harness_update_connection(
        &mut self,
        id: &str,
        patch: &ConnectionPatch,
    ) -> Result<(), String> {
        let mut state = self.wire_harness_state();
        let connection = state
            .connections
            .iter_mut()
            .find(|connection| connection.id == id)
            .ok_or_else(|| format!("no harness connection '{id}'"))?;
        if let Some(name) = &patch.name {
            connection.name = name.trim().to_string();
        }
        if let Some(from) = &patch.from {
            connection.from = from.trim().to_string();
        }
        if let Some(to) = &patch.to {
            connection.to = to.trim().to_string();
        }
        if let Some(diameter) = patch.diameter {
            if !(diameter.is_finite() && diameter > 0.0) {
                return Err(format!("wire diameter must be positive, got {diameter}"));
            }
            connection.diameter = diameter;
        }
        self.write_wire_harness_state(state);
        Ok(())
    }

    /// Remove one connection. An unknown id is an error.
    pub fn wire_harness_remove_connection(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.wire_harness_state();
        let before = state.connections.len();
        state.connections.retain(|connection| connection.id != id);
        if state.connections.len() == before {
            return Err(format!("no harness connection '{id}'"));
        }
        self.write_wire_harness_state(state);
        Ok(())
    }

    /// Switch the bundle solids on / off (the routing report stays either way).
    pub fn wire_harness_set_build_bundles(&mut self, on: bool) {
        let mut state = self.wire_harness_state();
        if state.build_bundles == on {
            return;
        }
        state.build_bundles = on;
        self.write_wire_harness_state(state);
    }

    /// Set the cut margin every wire's BOM cut length adds to its routed
    /// length (see [`WireHarnessState::cut_margin`]). A negative or non-finite
    /// margin is refused and the block is left alone; an unchanged one writes
    /// nothing, so it takes no undo step. The re-run replays the harness from
    /// its cache — the router does not read the margin.
    pub fn wire_harness_set_cut_margin(&mut self, margin: f64) -> Result<(), String> {
        if !(margin.is_finite() && margin >= 0.0) {
            return Err(format!("cut margin must be zero or more, got {margin}"));
        }
        let mut state = self.wire_harness_state();
        if state.cut_margin == margin {
            return Ok(());
        }
        state.cut_margin = margin;
        self.write_wire_harness_state(state);
        Ok(())
    }

    // --- hover ----------------------------------------------------------------

    /// Highlight a connection under the panel's pointer: its two ports (their
    /// sheet solids are keyed by the port feature id) and the bundle solids of
    /// every segment its route crosses. An unknown id highlights nothing.
    pub fn wire_harness_hover_connection(&mut self, id: &str) {
        let state = self.wire_harness_state();
        let mut names: Vec<String> = Vec::new();
        if let Some(connection) = state.connections.iter().find(|c| c.id == id) {
            for port in [&connection.from, &connection.to] {
                if !port.is_empty() && self.scene.solid(port).is_some() {
                    names.push(port.clone());
                }
            }
        }
        if let Some(report) = &self.wire_harness_report {
            if let Some(route) = report.routes.iter().find(|route| route.connection_id == id) {
                for segment in &route.segment_ids {
                    if let Some(bundle) = report.bundles.iter().find(|b| &b.segment_id == segment) {
                        if !bundle.solid_name.is_empty() {
                            names.push(bundle.solid_name.clone());
                        }
                    }
                }
            }
        }
        self.clear_hover();
        self.emphasis.hovered_solids.extend(names);
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
    }

    /// The pointer left the panel's rows: drop the highlight.
    pub fn wire_harness_hover_end(&mut self) {
        if self.clear_hover() {
            self.dirty = true;
        }
    }
}

