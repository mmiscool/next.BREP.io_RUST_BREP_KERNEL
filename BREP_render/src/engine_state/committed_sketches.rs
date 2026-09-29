use super::*;

/// The committed-sketch SHEET base color (dim cyan, matching the retired overlay
/// `COMMITTED_COLOR` 0x67c7d4) — a sketch sheet reads distinctly from a real solid.
const SKETCH_SHEET_COLOR: [f32; 3] = [
    0x67 as f32 / 255.0,
    0xc7 as f32 / 255.0,
    0xd4 as f32 / 255.0,
];

/// The wire-harness PORT sheet colour: red, so a port reads apart from every
/// sketch (cyan) and solid at a glance.
const PORT_TERMINATION_COLOR: [f32; 3] = [
    0xe5 as f32 / 255.0,
    0x32 as f32 / 255.0,
    0x2d as f32 / 255.0,
];
/// A WAYPOINT port: a lighter red, so a pass-through reads apart from an end.
const PORT_WAYPOINT_COLOR: [f32; 3] = [
    0xff as f32 / 255.0,
    0x8a as f32 / 255.0,
    0x80 as f32 / 255.0,
];

/// The feature types whose committed display is a synthesized SKETCH-LIKE
/// sheet (an `is_sketch` scene solid drawn from the paths the feature
/// published), and the prefix of the path names that are that feature's own
/// drawable segments:
///   * `S` — a sketch draws one segment per model geometry, `{id}:G{gid}`;
///   * `HX` — a helix draws its single fitted edge, `{id}:HelixEdge`;
///   * `SP` — a spline draws its exact chain, `{id}:SplineEdge`;
///   * `WP` — a routing waypoint draws its line, `{id}:PortLine`;
///   * `PORTS` — a declared connection point, which is not a feature at all:
///     its address stands in for the id and it draws the same line.
/// Anything else (a solid feature) has no sheet here. Only `S` is ever
/// ENTERABLE — the sketch-mode guards check the type separately.
fn curve_feature_segment_prefix(feature_type: &str, id: &str) -> Option<String> {
    match feature_type {
        "S" => Some(format!("{id}:G")),
        "HX" => Some(format!("{id}:HelixEdge")),
        "SP" => Some(format!("{id}:SplineEdge")),
        "WP" | brep_kernel::PORTS_FEATURE_TYPE => Some(format!("{id}:PortLine")),
        _ => None,
    }
}

/// The prefix of the published POINT names a sketch-like feature draws as
/// standalone vertices: a sketch publishes every solved point as `{id}:P{pid}`,
/// and the ones no segment covers (a points-only hole-placement sketch) are
/// otherwise invisible; a spline publishes its anchors as `{id}:P{index}`, drawn
/// so the anchors are visible and pickable; a port publishes its base point
/// as `{id}:Base`. A helix publishes only its two ends, which its edge already
/// draws, so it has no point prefix.
fn curve_feature_point_prefix(feature_type: &str, id: &str) -> Option<String> {
    match feature_type {
        "S" | "SP" => Some(format!("{id}:P")),
        "WP" | brep_kernel::PORTS_FEATURE_TYPE => Some(format!("{id}:Base")),
        _ => None,
    }
}

impl EngineState {

    /// The committed sketches present at the CURRENT rollback: every sketch-like
    /// feature (see [`curve_feature_segment_prefix`]) at index `0..=rollback`,
    /// EXCEPT the one being edited (it displays via the live editing overlay) and
    /// EXCEPT any sketch a downstream feature within the current rollback CONSUMED
    /// (see [`consumed_feature_names`](Self::consumed_feature_names)). Hidden ids
    /// are still returned (the Scene tree lists them); visibility is filtered by
    /// the caller against [`hidden_sketches`].
    fn committed_sketch_ids(&self) -> Vec<String> {
        self.committed_curve_features()
            .into_iter()
            .map(|(id, _, _)| id)
            .collect()
    }

    /// [`committed_sketch_ids`](Self::committed_sketch_ids) with each id's
    /// segment-name prefix and (for a sketch) its point-name prefix — the
    /// `(id, segment prefix, point prefix)` triples the sheet refresh draws.
    fn committed_curve_features(&self) -> Vec<(String, String, Option<String>)> {
        self.committed_curve_features_typed()
            .into_iter()
            .map(|(id, _, prefix, point_prefix)| (id, prefix, point_prefix))
            .collect()
    }

    /// [`committed_curve_features`](Self::committed_curve_features) with each
    /// feature's TYPE beside its id — the sheet refresh colours a port sheet
    /// by type.
    fn committed_curve_features_typed(&self) -> Vec<(String, String, String, Option<String>)> {
        let editing = self.sketch_edit.as_ref().map(|edit| edit.feature_id.as_str());
        let rollback = self.history.rollback();
        let mut ids = Vec::new();
        for index in 0..=rollback {
            let Some(feature_type) = self.history.feature_type(index) else {
                continue;
            };
            let Some(id) = self.history.feature_id(index) else {
                continue;
            };
            let Some(prefix) = curve_feature_segment_prefix(&feature_type, &id) else {
                continue;
            };
            if Some(id.as_str()) == editing {
                continue;
            }
            let point_prefix = curve_feature_point_prefix(&feature_type, &id);
            ids.push((id, feature_type, prefix, point_prefix));
        }
        // Drop any sketch a downstream feature CONSUMED: extrude/revolve (gated by
        // `consumeProfileSketch`) and the sheet-metal consumers push the consumed
        // sketch id into their `result.removed`, so a consumed sketch must vanish
        // from the scene — no sheet, no tree row — exactly like a consumed solid.
        // Only replay when there is a candidate to test, so a sketch-less scene pays
        // nothing for the check.
        if !ids.is_empty() {
            let consumed = self.consumed_feature_names();
            ids.retain(|(id, _, _, _)| !consumed.contains(id));
        }
        // The document's OWN declared connection points: not features, so the
        // ports tail published them under their ADDRESS with the same
        // `{id}:PortLine` / `{id}:Base` names a waypoint uses.
        let mut declare = |address: String, ids: &mut Vec<(String, String, String, Option<String>)>| {
            if ids.iter().any(|(id, _, _, _)| *id == address) {
                return;
            }
            let ty = brep_kernel::PORTS_FEATURE_TYPE;
            let Some(prefix) = curve_feature_segment_prefix(ty, &address) else {
                return;
            };
            let point_prefix = curve_feature_point_prefix(ty, &address);
            ids.push((address, ty.to_string(), prefix, point_prefix));
        };
        for point in self.history.declared_points() {
            declare(point.address(), &mut ids);
        }
        // The points placed components carry, namespaced by the ACOMP that
        // published them. Shown per workbench.
        if self.component_ports_visible {
            let addresses: Vec<String> = self
                .assembly_components()
                .iter()
                .flat_map(|component| component.ports.clone())
                .collect();
            for address in addresses {
                declare(address, &mut ids);
            }
        }
        ids
    }

    /// Whether the ports placed components carry draw their sheets.
    pub fn component_ports_visible(&self) -> bool {
        self.component_ports_visible
    }

    /// Show or hide the sheets of the ports placed components carry (the app
    /// drives this from the active workbench). A change refreshes the sheets.
    pub fn set_component_ports_visible(&mut self, visible: bool) {
        if self.component_ports_visible == visible {
            return;
        }
        self.component_ports_visible = visible;
        self.refresh_committed_sketches();
    }

    /// The sheet colour for a committed curve feature: connection points are
    /// red, routing waypoints a lighter red, everything else the sketch cyan.
    fn curve_feature_color(&self, _id: &str, feature_type: &str) -> [f32; 3] {
        match feature_type {
            "WP" => PORT_WAYPOINT_COLOR,
            ty if ty == brep_kernel::PORTS_FEATURE_TYPE => PORT_TERMINATION_COLOR,
            _ => SKETCH_SHEET_COLOR,
        }
    }

    /// The set of output names CONSUMED (removed) by some feature within the
    /// rollback prefix the last applied run executed — a solid absorbed by a
    /// boolean, or a sketch absorbed by a downstream extrude / revolve /
    /// sheet-metal consumer. The SAME per-feature `removed` signal the scene
    /// build honors, unioned by the runner and shipped in the reply
    /// ([`crate::pipeline::RunOutput::consumed`]).
    ///
    /// This used to replay the prefix on the UI thread. It was never the clean
    /// cache hit its old comment claimed: the kernel's incremental cache is per
    /// thread, so the first main-side replay of a document executed it in full,
    /// and every later one re-executed whatever the edit had dirtied — plus the
    /// uncached wire-harness tail, which swept every bundle a second time.
    ///
    /// Because [`prefix_request`](crate::history::History::prefix_request) stops
    /// at the rolled-to feature, a consumer ABOVE the rollback never runs, so its
    /// removed names are (correctly) absent — a sketch consumed above the
    /// rollback point still shows. Empty before the first run.
    fn consumed_feature_names(&self) -> &std::collections::HashSet<String> {
        &self.consumed_names
    }

    /// (Re)build the persistent committed-sketch SHEET SOLIDS. For every committed
    /// sketch that should show — [`committed_sketch_ids`](Self::committed_sketch_ids)
    /// minus [`hidden_sketches`] — synthesize its display from the run's solved
    /// profile AND its own model segments, and insert it as a scene solid (keyed by
    /// the sketch id, dim-cyan, flagged `is_sketch`); remove any sheet inserted on
    /// the PREVIOUS refresh but not this one (rolled back, deleted, hidden, or
    /// became the active edit). A sketch that closes a region gets its planar sheet;
    /// one that closes NOTHING (an open chain — the single line of the 2026-09-02
    /// report) still draws its segments as named edges, so it is visible and
    /// pickable instead of vanishing; one holding ONLY points (a hole-placement
    /// sketch) draws them as vertices, so it too is visible, listed and pickable
    /// (a vertex hit carries its owning sketch, which the `SKETCH` pick lane
    /// admits). Only a sketch with no drawable geometry at all (empty /
    /// construction-only) is skipped. Marks dirty.
    pub fn refresh_committed_sketches(&mut self) {
        let visible: Vec<(String, String, String, Option<String>)> = self
            .committed_curve_features_typed()
            .into_iter()
            .filter(|(id, _, _, _)| !self.hidden_sketches.contains(id))
            .collect();

        // Phase 1 (immutable borrow of the surfaced profiles + paths + points):
        // build a display payload per visible sketch — its closed profile's sheet
        // (when it has one), every model segment that sheet does not already
        // draw, and every model point no segment covers. A sketch with nothing to
        // draw (no face + no edges + no points) is skipped.
        let payloads: Vec<(String, [f32; 3], brep_kernel::DisplaySolidPayload)> = visible
            .iter()
            .filter_map(|(id, feature_type, prefix, point_prefix)| {
                let profile = self
                    .sketch_profiles
                    .iter()
                    .find(|(name, _)| name == id)
                    .map(|(_, profile)| profile);
                // The feature's own model segments, published under its segment
                // prefix (a sketch: one per geometry, `{id}:G{gid}`; a helix: its
                // one `{id}:HelixEdge`). The whole-chain `{id}` path duplicates
                // them and `{id}:REF:{source}` is projected reference geometry,
                // so neither is drawn here.
                let segments: Vec<(String, Vec<brep_kernel::NurbsCurve>)> = self
                    .sketch_paths
                    .iter()
                    .filter(|(name, _)| name.starts_with(prefix.as_str()))
                    .cloned()
                    .collect();
                // The sketch's own MODEL points, published under `{id}:P{pid}`.
                // Construction points constrain and never model, so — like
                // construction geometry — they do not draw.
                let points: Vec<brep_kernel::Vec3> = match point_prefix {
                    Some(point_prefix) => self
                        .sketch_points
                        .iter()
                        .filter(|(name, point)| {
                            name.starts_with(point_prefix.as_str()) && !point.construction
                        })
                        .map(|(_, point)| point.position)
                        .collect(),
                    None => Vec::new(),
                };
                let payload = brep_kernel::sketch_display_payload(profile, &segments, &points);
                if payload.mesh.indices.is_empty()
                    && payload.edges.is_empty()
                    && payload.vertices.is_empty()
                {
                    return None;
                }
                Some((id.clone(), self.curve_feature_color(id, feature_type), payload))
            })
            .collect();

        // Phase 2 (mutable borrow of the scene): insert each sheet as a scene solid.
        let mut fed: Vec<String> = Vec::with_capacity(payloads.len());
        for (id, color, payload) in payloads {
            let mut solid = crate::scene::solid_display_from_payload(&id, payload);
            solid.is_sketch = true;
            solid.color_override = Some(color);
            self.scene.insert_solid(solid);
            fed.push(id);
        }

        // Remove any sketch sheet inserted last time but not now.
        let fed_set: std::collections::HashSet<&str> = fed.iter().map(String::as_str).collect();
        let stale: Vec<String> = self
            .shown_sketch_ids
            .iter()
            .filter(|id| !fed_set.contains(id.as_str()))
            .cloned()
            .collect();
        for id in stale {
            self.scene.remove_solid(&id);
        }

        self.shown_sketch_ids = fed;
        self.dirty = true;
    }

    /// Whether the committed sketch `id`'s persistent overlay is shown (absent from
    /// [`hidden_sketches`] = visible).
    pub fn sketch_visible(&self, id: &str) -> bool {
        !self.hidden_sketches.contains(id)
    }

    /// Show/hide the committed sketch `id`'s persistent overlay (the Scene-tree
    /// checkbox). Toggles [`hidden_sketches`], rebuilds the committed overlays (so the
    /// group is fed or cleared immediately), and marks dirty.
    pub fn set_sketch_visible(&mut self, id: &str, visible: bool) {
        if visible {
            self.hidden_sketches.remove(id);
        } else {
            self.hidden_sketches.insert(id.to_string());
        }
        self.refresh_committed_sketches();
    }

    /// The committed sketches to list in the Scene tree: every `"S"` feature at the
    /// current rollback (minus the active edit), each with its live visibility — the
    /// ordered `(id, visible)` list the Scene panel snapshots.
    pub fn committed_sketches(&self) -> Vec<(String, bool)> {
        crate::visibility::named_visibility(self.committed_sketch_ids(), &self.hidden_sketches)
    }

    /// The committed sketches as JSON (`[{"name":<id>,"visible":<bool>}]`) — the
    /// sibling of [`scene_entities_json`](Self::scene_entities_json) the Scene panel
    /// publishes for the headed verifier (kept a SEPARATE method so the solids array's
    /// shape is unchanged).
    pub fn sketch_entities_json(&self) -> String {
        crate::visibility::named_visibility_json(self.committed_sketches())
    }
}


