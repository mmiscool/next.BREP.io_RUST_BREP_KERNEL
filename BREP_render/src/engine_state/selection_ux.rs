use super::*;

/// The geometry selection [`EngineState::select_feature`] made, kept beside the
/// feature id so the feature reads as selected exactly as long as that
/// selection stands.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct FeatureSelection {
    solids: crate::style::OrderedNames,
    faces: crate::style::OrderedNames,
    edges: crate::style::OrderedNames,
    datums: crate::style::OrderedNames,
}

impl FeatureSelection {
    fn of(emphasis: &crate::style::Emphasis) -> Self {
        Self {
            solids: emphasis.selected_solids.clone(),
            faces: emphasis.selected_faces.clone(),
            edges: emphasis.selected_edges.clone(),
            datums: emphasis.selected_datums.clone(),
        }
    }
}

impl EngineState {
    /// Clear the current SELECTION (Esc): drop all selected solids/faces/edges/
    /// vertices (hover is left untouched). Bumps the emphasis generation + marks
    /// dirty only when something was actually cleared. Returns whether it changed.
    pub fn clear_selection(&mut self) -> bool {
        // The viewport-selected CONSTRAINT (label click) clears with the rest.
        let had_constraint = self.selected_constraint.is_some();
        self.constraint_deselect();
        // …and so do the tree selections that have no geometry of their own: a
        // feature (whose geometry clears below), a PMI annotation and a sheet
        // object.
        let had_feature = self.selected_feature.take().is_some();
        let had_pmi_annotation = self.pmi_selected_annotation.take().is_some();
        if had_pmi_annotation {
            self.refresh_pmi_overlay();
        }
        let had_pmi = self.pmi_selected_view.take().is_some() || had_pmi_annotation;
        let had_sheet = self.sheet_selected_object.take().is_some();
        let had_datums = !self.emphasis.selected_datums.is_empty();
        let had = had_constraint
            || had_feature
            || had_pmi
            || had_sheet
            || !self.emphasis.selected_solids.is_empty()
            || !self.emphasis.selected_faces.is_empty()
            || !self.emphasis.selected_edges.is_empty()
            || !self.emphasis.selected_vertices.is_empty()
            || had_datums;
        if had {
            self.emphasis.selected_solids.clear();
            self.emphasis.selected_faces.clear();
            self.emphasis.selected_edges.clear();
            self.emphasis.selected_vertices.clear();
            self.emphasis.selected_datums.clear();
            self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
            self.dirty = true;
        }
        // A cleared datum drops its selection accent — re-feed the datum planes so
        // the highlight disappears immediately (no re-run needed).
        if had_datums {
            self.refresh_construction_datums();
        }
        had
    }

    /// REPLACE the whole selection with these named entities — the write twin of
    /// [`Self::selection_json`], and the only way to put a MULTI-entity selection
    /// back the way it was (`select_by_name` takes one name and clears the rest).
    ///
    /// Names are taken as given, IN THE ORDER GIVEN: this is the write twin of a
    /// pick order, so a selection saved from `selection_json` and put back here
    /// seeds exactly what it seeded before. An unknown name simply highlights
    /// nothing, the same as a stale name after a rebuild. VERTICES are not
    /// addressable here —
    /// they have no kernel name and emphasis keys them by position
    /// ([`Self::select_vertex_by_position`]) — so a saved selection's vertices do
    /// not come back through this call.
    pub fn set_selection(
        &mut self,
        solids: &[String],
        faces: &[String],
        edges: &[String],
        datums: &[String],
    ) {
        let had_datums = !self.emphasis.selected_datums.is_empty();
        self.emphasis.selected_solids = solids.iter().cloned().collect();
        self.emphasis.selected_faces = faces.iter().cloned().collect();
        self.emphasis.selected_edges = edges.iter().cloned().collect();
        self.emphasis.selected_datums = datums.iter().cloned().collect();
        self.emphasis.selected_vertices.clear();
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if had_datums || !datums.is_empty() {
            self.refresh_construction_datums();
        }
    }

    /// SELECT a feature, as the history tree's single click does: the feature
    /// becomes the selected one and the geometry it contributed to the model on
    /// screen becomes the viewport selection, replacing whatever was selected.
    ///
    /// What a feature "contributed" is read from the last run, first match wins:
    /// the FACES it created that are still in the scene (a fillet's blends, a
    /// hole's bore — a feature later rewritten by another keeps its own faces);
    /// else the SOLIDS it was the last to write (a boolean, which makes no face
    /// of its own); else its own sketch or datum frames. A feature past the
    /// rollback has built nothing, so it selects with an empty geometry
    /// selection — the tree row still shows it.
    ///
    /// Returns false, changing nothing, when `id` is not a feature.
    pub fn select_feature(&mut self, id: &str) -> bool {
        if self.history.index_of(id).is_none() {
            return false;
        }
        let in_scene_faces: Vec<String> = self
            .scene
            .solids()
            .iter()
            .flat_map(|solid| solid.faces.iter().map(|face| face.name.clone()))
            .filter(|name| self.entity_origin.get(name).map(String::as_str) == Some(id))
            .collect();
        let mut solids: Vec<String> = Vec::new();
        let mut datums: Vec<String> = Vec::new();
        if in_scene_faces.is_empty() {
            solids = self
                .scene
                .solids()
                .iter()
                .filter(|solid| {
                    self.provenance.get(&solid.name).map(String::as_str) == Some(id)
                        || solid.name == id
                })
                .map(|solid| solid.name.clone())
                .collect();
        }
        if in_scene_faces.is_empty() && solids.is_empty() {
            datums = self
                .construction_frames
                .iter()
                .map(|(name, _)| name)
                .filter(|name| {
                    self.datum_feature_for_name(name)
                        .is_some_and(|(feature, _)| feature == id)
                })
                .cloned()
                .collect();
        }
        self.set_selection(&solids, &in_scene_faces, &[], &datums);
        self.selected_feature = Some((id.to_string(), FeatureSelection::of(&self.emphasis)));
        true
    }

    /// The feature selected through [`Self::select_feature`], while the
    /// geometry selection it made still stands and the feature still exists.
    /// Any other selection — a viewport pick, Esc, Clear, a Scene-tree row —
    /// replaces that geometry and so reads as the feature being deselected.
    pub fn selected_feature(&self) -> Option<&str> {
        let (id, made) = self.selected_feature.as_ref()?;
        (self.history.index_of(id).is_some() && *made == FeatureSelection::of(&self.emphasis)
            && self.emphasis.selected_vertices.is_empty())
        .then_some(id.as_str())
    }

    /// Select the top-priority pick under CSS-pixel `(x, y)` that the SELECTION
    /// FILTER admits — replacing the current selection (a plain viewport click).
    /// A miss (or a click when the filter admits nothing) clears the selection.
    /// Marks dirty when the selection changed; returns whether something was
    /// selected. The by-kind honoring lives in [`select_filtered_at`] in the
    /// appended selection-filter impl block (kept separate so concurrent edits to
    /// this primary block don't conflict).
    pub fn select_top_at(&mut self, x: f64, y: f64) -> bool {
        self.select_filtered_at(x, y)
    }

    /// The current SELECTION (not hover) as JSON
    /// `{ solids:[..], faces:[..], edges:[..], vertices: n }` — lets a UI / the
    /// headed verifier read selection state (e.g. assert Esc cleared it).
    ///
    /// Each array is in PICK ORDER, oldest first ([`OrderedNames`](crate::style::
    /// OrderedNames)): the first name in `faces` is the face that was picked
    /// first, and a face toggled off and on again is last. That is the order
    /// every seeder reads — the assembly constraint's `elements` (whose first
    /// element is the Distance BASE face), the context bar's reference pre-fill,
    /// a boolean's target/tool, a fillet's edge list — so the same picks seed the
    /// same feature in every process. Order is per BUCKET: a face and an edge
    /// picked in either order still list under `faces` and `edges`, and a seeder
    /// that takes both reads them in ITS OWN kind order (the field's
    /// `selectionFilter`), which is fixed by the schema.
    pub fn selection_json(&self) -> String {
        let solids: Vec<&String> = self.emphasis.selected_solids.iter().collect();
        let faces: Vec<&String> = self.emphasis.selected_faces.iter().collect();
        let edges: Vec<&String> = self.emphasis.selected_edges.iter().collect();
        let datums: Vec<&String> = self.emphasis.selected_datums.iter().collect();
        serde_json::json!({
            "solids": solids,
            "faces": faces,
            "edges": edges,
            "datums": datums,
            "vertices": self.emphasis.selected_vertices.len(),
        })
        .to_string()
    }

    // --- Reference-selection widget (the engine-native picker, #42) --------
    //
    // A feature-dialog reference field activates this MODAL: the UI hides the
    // rest of itself and shows only the widget's list + Finish/Cancel; the engine
    // rolls to the pre-feature "before" state, highlights the running selection
    // (via `emphasis`), and each click in the viewport type-constrained-picks a
    // name into the list. Finish writes the names into the feature params (via
    // the same `update_feature_params` path) and restores; Cancel discards. The
    // list of names is the whole state — no event-on-object wiring.

    /// True while the reference-selection modal is active (the shell hides the
    /// rest of the UI and the viewport routes clicks to picking).
    pub fn ref_select_active(&self) -> bool {
        self.ref_select.is_some()
    }

    /// Enter reference-selection mode for feature `feature_id`'s param at `path`.
    /// Seeds the running list from `seed_names` (the field's current value), rolls
    /// the model to the pre-feature "before" state (the step just before the
    /// edited feature ran), and highlights the seeded names. `filter` constrains
    /// the pick kind (`["SOLID"]`, `["FACE"]`, …); `multiple` allows a list.
    pub fn begin_ref_select(
        &mut self,
        feature_id: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) {
        let restore_index = self.history.rollback();
        // "Before" = the step just before the edited feature ran, so the user
        // picks against the correct geometry. Clamp at 0 for the first feature.
        let before = self
            .history
            .index_of(feature_id)
            .map(|i| i.saturating_sub(1))
            .unwrap_or(restore_index);
        // Constrain the GLOBAL selection filter to exactly the kinds this field
        // permits: this drives BOTH click-picking (`ref_select_click`) AND
        // hover-highlighting (`hover_at`, which reads `selection_filter`), so only
        // the allowed kinds highlight/select while the picker is active. An
        // absent/construction-only field filter maps to all-enabled (see
        // `from_ref_filter`). Restored to the all-enabled default on finish/cancel
        // (`end_ref_select`).
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: feature_id.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::Feature,
        });
        // Roll to the before-state (re-runs + marks dirty), then light up the seed.
        self.history.set_rollback(before);
        self.rerun_history();
        self.sync_ref_select_emphasis();
    }

    /// The running list of picked names (empty when not active) — the modal UI
    /// reads this back to draw its one-per-line list.
    pub fn ref_select_names(&self) -> Vec<String> {
        self.ref_select
            .as_ref()
            .map(|r| r.names.clone())
            .unwrap_or_default()
    }

    /// The active field's label (for the modal heading), or empty.
    pub fn ref_select_label(&self) -> String {
        self.ref_select
            .as_ref()
            .map(|r| r.label.clone())
            .unwrap_or_default()
    }

    /// A one-line summary of the active field for the modal heading:
    /// `"Tool solids (SOLID, multiple)"`.
    pub fn ref_select_prompt(&self) -> String {
        match &self.ref_select {
            Some(r) if r.target == RefSelectTarget::Sheet => format!(
                "{} (sheet anchor{})",
                r.label,
                if r.multiple { "s" } else { "" }
            ),
            Some(r) => format!(
                "{} ({}{})",
                r.label,
                r.filter.join("/"),
                if r.multiple { ", multiple" } else { "" }
            ),
            None => String::new(),
        }
    }

    /// A viewport click while active: type-constrained-pick the nearest allowed
    /// hit under CSS-pixel `(x, y)` and add its name to the running list (single
    /// fields replace; multiple fields append, de-duplicated). Re-lights the
    /// highlight. No-op on a miss / an empty (unnamed) hit.
    pub fn ref_select_click(&mut self, x: f64, y: f64) {
        let Some(state) = self.ref_select.as_ref() else {
            return;
        };
        let filter = state.filter.clone();
        let multiple = state.multiple;
        let target = state.target;
        // A SHEET pick is an anchor on the paper, never a 3D hit: the sheet
        // viewport feeds `ref_select_pick_sheet_anchor` instead.
        if target == RefSelectTarget::Sheet {
            return;
        }
        // The field's RAW kind strings drive the pick (`pick_top_at` reads
        // `DATUM` as an alias of `PLANE`), so a `["PLANE","FACE"]` sketchPlane
        // field now picks a construction plane through the ORDINARY candidate
        // list — including one sitting under a face, which the geometry-miss
        // fallback below could never reach.
        // A spline anchor attaches to a PORT: whatever part of a port's drawn
        // sheet was hit (its line, its base vertex, the sheet itself), the
        // pick resolves to the OWNING sheet, which is keyed by the port id;
        // anything that is not a port is refused with a notice.
        if let RefSelectTarget::SplineAnchor { .. } = target {
            let Some(hit) = self.pick_top_at(x, y, &filter) else {
                return;
            };
            let owner = if hit.solid.trim().is_empty() { hit.name.clone() } else { hit.solid.clone() };
            if !self.is_port_id(&owner) {
                self.push_notice(format!("'{owner}' is not a port — pick a port to attach the anchor"));
                return;
            }
            let state = self.ref_select.as_mut().expect("active by guard above");
            state.names = vec![owner];
            self.sync_ref_select_emphasis();
            return;
        }
        let picked = match self.pick_top_at(x, y, &filter) {
            Some(hit) if hit.kind == pick::PickKind::Plane => {
                // Accept ONLY a resolved D/P frame name, the same guard the
                // fallback applies — a stray widget-fed plane never lands in a
                // reference field.
                if self.datum_feature_for_name(&hit.name).is_none() {
                    return;
                }
                hit.name
            }
            Some(hit) if !hit.name.trim().is_empty() => hit.name,
            Some(hit) => {
                // A vertex pick carries no kernel name. For an ASSEMBLY CONSTRAINT
                // field that accepts VERTEX, build the `{solidName}@x,y,z` ref with
                // COMPONENT-LOCAL coordinates (world pick · owning-component
                // pose⁻¹ — the lane-E selection contract; the kernel resolver snaps
                // to the nearest topology vertex). Everything else stays a no-op.
                if !matches!(hit.kind, pick::PickKind::Vertex) {
                    return;
                }
                match target {
                    RefSelectTarget::AssemblyConstraint => {
                        match self.component_vertex_ref(&hit.solid, hit.position) {
                            Some(vertex_ref) => vertex_ref,
                            None => return, // not component geometry — constraints reject it anyway
                        }
                    }
                    // PMI resolves against the world-posed resident solids, so
                    // its vertex refs carry WORLD coordinates.
                    RefSelectTarget::Pmi => super::pmi_ops::world_vertex_ref(&hit.solid, hit.position),
                    // A connection point's seat resolves against the finished
                    // part, which is the same world-posed scene, so its vertex
                    // refs take the same form and the kernel reads them with
                    // the same helper.
                    RefSelectTarget::PortPoint { .. } => {
                        super::pmi_ops::world_vertex_ref(&hit.solid, hit.position)
                    }
                    _ => return,
                }
            }
            None => {
                // TOTAL MISS. The plane CARDS are candidates above and they are
                // the same set `datum_pick` tests, so in the app this arm is
                // unreachable for a plane field; it is kept as a second line of
                // defense for the tested `["PLANE","FACE"]` sketchPlane flow (and
                // it is the ONLY path that would reach a datum AXIS, were one ever
                // fed). Accept ONLY a resolved D/P frame name
                // (`datum_feature_for_name`, mirroring `select_datum`'s guard) so an
                // AXIS name — which `datum_pick` may also return — never lands in a
                // plane field.
                let admits_plane = filter
                    .iter()
                    .any(|k| k.eq_ignore_ascii_case("PLANE") || k.eq_ignore_ascii_case("DATUM"));
                if !admits_plane {
                    return;
                }
                let name = self.datum_pick(x, y);
                if name.is_empty() || self.datum_feature_for_name(&name).is_none() {
                    return;
                }
                name
            }
        };
        let state = self.ref_select.as_mut().expect("active by guard above");
        if multiple {
            if !state.names.iter().any(|n| n == &picked) {
                state.names.push(picked);
            }
        } else {
            state.names = vec![picked];
        }
        self.sync_ref_select_emphasis();
    }

    /// Feed a picked NAME into the running list — what [`Self::ref_select_click`]
    /// does after its pick (single fields replace; multiple fields append,
    /// de-duplicated), for a caller that already holds the name (a test, an
    /// automation driver). No-op while no selection is running.
    pub fn ref_select_add_name(&mut self, name: String) {
        let Some(state) = self.ref_select.as_mut() else {
            return;
        };
        if state.multiple {
            if !state.names.iter().any(|n| n == &name) {
                state.names.push(name);
            }
        } else {
            state.names = vec![name];
        }
        self.sync_ref_select_emphasis();
    }

    /// Remove the name at `index` from the running list (the modal's per-line X).
    pub fn ref_select_remove(&mut self, index: usize) {
        if let Some(state) = self.ref_select.as_mut() {
            if index < state.names.len() {
                state.names.remove(index);
            }
        }
        self.sync_ref_select_emphasis();
    }

    /// Finish: write the running names into the edited feature's params at the
    /// field path, restore the rolled-to step, clear the highlight, and re-run so
    /// the feature rebuilds with the chosen references.
    pub fn finish_ref_select(&mut self) {
        let Some(state) = self.ref_select.take() else {
            return;
        };
        // An ASSEMBLY CONSTRAINT field commits through the constraint update
        // lane (kernel session + document fold), not feature params; the shared
        // end tail below still restores the roll + re-runs (which re-solves).
        if state.target == RefSelectTarget::AssemblyConstraint {
            self.assembly_commit_constraint_refs(
                &state.feature_id,
                &state.path,
                &state.names,
                state.multiple,
            );
            self.end_ref_select(state.restore_index);
            return;
        }
        // A PMI annotation field commits into the document's pmi block (no
        // history feature is involved); the shared tail restores + re-runs,
        // which resolves the annotation against the fresh scene.
        if state.target == RefSelectTarget::Pmi {
            if let Some((_, annotation)) = self.pmi_state().find_annotation(&state.feature_id) {
                if annotation.kind.contains('/') {
                    let mut params = annotation.params.clone();
                    let value = if state.multiple { serde_json::json!(state.names) }
                        else { serde_json::json!(state.names.first().cloned().unwrap_or_default()) };
                    set_json_at(&mut params, &state.path, value);
                    // Clear picking state before the transaction captures selection/revision.
                    // PMI never rolled the history; a second rerun would invalidate the worker reply.
                    let _ = self.emphasis.apply_json("{}");
                    self.selection_filter = SelectionFilter::default();
                    if let Err(error) = self.plugin_update_annotation(&state.feature_id, params) {
                        self.push_notice(format!("PMI update failed: {error}"));
                    }
                    self.dirty = true;
                    return;
                }
            }
            self.pmi_commit_refs(&state.feature_id, &state.path, &state.names, state.multiple);
            self.end_ref_select(state.restore_index);
            return;
        }
        // A SHEET object's field commits into the `sheets` block, and a sheet
        // edit never re-runs the history — so its tail is its own: no roll to
        // restore, no rebuild, just the modal gone.
        if state.target == RefSelectTarget::Sheet {
            if let Err(error) =
                self.sheet_commit_refs(&state.feature_id, &state.path, &state.names, state.multiple)
            {
                self.push_notice(format!("Sheets: {error}"));
            }
            self.dirty = true;
            return;
        }
        // A CONNECTION POINT's reference commits into the `ports` block (no
        // history feature is involved, and nothing was rolled). The shared
        // tail re-runs, which is what re-seats the point on what was picked.
        if let RefSelectTarget::PortPoint { field } = state.target {
            self.ports_commit_ref(
                &state.feature_id,
                field,
                state.names.first().map(String::as_str).unwrap_or_default(),
            );
            self.end_ref_select(state.restore_index);
            return;
        }
        // A SPLINE ANCHOR attachment commits through the anchor lane (the
        // persistent spline document), then the shared tail restores + re-runs.
        if let RefSelectTarget::SplineAnchor { index } = state.target {
            if let Some(port) = state.names.first() {
                self.attach_spline_anchor_no_rerun(&state.feature_id, index, port);
            }
            self.end_ref_select(state.restore_index);
            return;
        }
        if let Some(index) = self.history.index_of(&state.feature_id) {
            let mut params = self
                .history
                .feature_params(index)
                .unwrap_or_else(|| serde_json::json!({}));
            let value = if state.multiple {
                serde_json::Value::Array(
                    state
                        .names
                        .iter()
                        .cloned()
                        .map(serde_json::Value::String)
                        .collect(),
                )
            } else {
                serde_json::Value::String(state.names.first().cloned().unwrap_or_default())
            };
            set_json_at(&mut params, &state.path, value);
            self.stamp_face_transform_pivot(index, &mut params);
            self.history.set_feature_params(index, params);
        }
        self.end_ref_select(state.restore_index);
    }

    /// Cancel: discard the running selection, clear the highlight, restore the
    /// rolled-to step, and re-run (no param change).
    pub fn cancel_ref_select(&mut self) {
        if let Some(state) = self.ref_select.take() {
            if state.target == RefSelectTarget::Sheet {
                // Nothing rolled and nothing was highlighted: just close.
                self.dirty = true;
                return;
            }
            self.end_ref_select(state.restore_index);
        }
    }

    /// Whether the active picker is picking ANCHORS on a drawing sheet — the
    /// sheet viewport routes its clicks to [`Self::ref_select_pick_sheet_anchor`]
    /// and the modal's hint speaks of the paper rather than the 3D view.
    pub fn ref_select_is_sheet(&self) -> bool {
        self.ref_select.as_ref().is_some_and(|state| state.target == RefSelectTarget::Sheet)
    }

    /// Restore the rolled-to step + clear emphasis + re-run + reset the selection
    /// filter to the all-enabled default (shared Finish/Cancel tail).
    ///
    /// Resetting to the DEFAULT (not a saved "prior" filter) is deliberate: the
    /// spec baseline out of ref-select is "all kinds enabled", and `begin_ref_select`
    /// overwrites `ref_select` without routing through here, so a stashed prior
    /// could be a stale already-constrained filter. Living in this shared tail also
    /// means a stray `finish_ref_select()` while inactive (early return on `take`)
    /// never clobbers the filter.
    fn end_ref_select(&mut self, restore_index: usize) {
        let _ = self.emphasis.apply_json("{}");
        self.selection_filter = SelectionFilter::default();
        self.history.set_rollback(restore_index);
        self.rerun_history();
    }

    /// Drive the selection highlight (`emphasis`) from the running name list so
    /// picks light up in the viewport. A field may allow SEVERAL kinds at once
    /// (e.g. `FACE`/`EDGE`), and a pick can be any of them, so every picked name is
    /// fed to EVERY name-based bucket the filter permits — a name only ever matches
    /// its own kind's entities (edge names carry the `|…[n]` topology form, faces do
    /// not), so the cross-listing is harmless and each pick highlights correctly.
    /// (The old code bucketed ALL names by `filter.first()` only, so an EDGE pick
    /// under a `FACE`-first filter landed in `faces`, matched nothing, and never
    /// showed.) VERTEX picks are position-keyed, not name-keyed, so they can't be
    /// emphasized from a name list here.
    pub(crate) fn sync_ref_select_emphasis(&mut self) {
        // A sheet pick names paper anchors, not scene entities: the 3D
        // highlight is not its to drive (the paper draws its own picks).
        if self.ref_select_is_sheet() {
            return;
        }
        let json = match &self.ref_select {
            Some(state) => {
                let names = serde_json::json!(state.names);
                let mut selected = serde_json::Map::new();
                for kind in &state.filter {
                    // Case-INSENSITIVE match, mirroring `SelectionFilter::set` (which
                    // pick-filtering uses via `from_ref_filter`). Without this, a
                    // schema that spelled a kind non-canonically (e.g. `"Edge"`) would
                    // let the user PICK that kind but silently skip its seed HIGHLIGHT
                    // here — a lenient-pick / strict-highlight split. VERTEX is inert:
                    // vertex picks carry no kernel name, so `ref_select_click` never
                    // records one in `names` (empty-name early-return), so there is
                    // nothing to highlight by name.
                    let bucket = match kind.to_ascii_uppercase().as_str() {
                        "FACE" => "faces",
                        "EDGE" => "edges",
                        "SOLID" => "solids",
                        "PLANE" | "DATUM" => "datums",
                        _ => continue, // VERTEX (never name-seeded) / unknown
                    };
                    selected.entry(bucket.to_string()).or_insert_with(|| names.clone());
                }
                // No highlightable kind in the filter → fall back to solids (the
                // prior default) so at least solid-name picks still light up.
                if selected.is_empty() {
                    selected.insert("solids".to_string(), names);
                }
                serde_json::json!({ "selected": selected }).to_string()
            }
            None => "{}".to_string(),
        };
        let _ = self.emphasis.apply_json(&json);
        // A picked construction PLANE/DATUM highlights through the datum-plane
        // WIDGET, whose accent is baked at feed time (`refresh_construction_datums`
        // reads `emphasis.selected_datums`) — so a plain `apply_json` does not
        // re-color it. Re-feed here so a datum pick lights up (and un-lights on
        // remove) in the modal. Harmless for non-datum fields (no datum selected →
        // an ordinary calm-color re-feed).
        self.refresh_construction_datums();
        self.dirty = true;
    }
}

/// Write `value` into `root` at `path` (object-key chain), auto-vivifying
/// intermediate objects — the engine-side twin of the form's nested setter, used
/// to commit a reference field's picked names back into the feature params.
pub(crate) fn set_json_at(root: &mut serde_json::Value, path: &[String], value: serde_json::Value) {
    if path.is_empty() {
        *root = value;
        return;
    }
    if !root.is_object() {
        *root = serde_json::Value::Object(serde_json::Map::new());
    }
    let mut cur = root;
    for seg in &path[..path.len() - 1] {
        let obj = cur.as_object_mut().expect("object by construction");
        cur = obj
            .entry(seg.clone())
            .or_insert_with(|| serde_json::Value::Object(serde_json::Map::new()));
        if !cur.is_object() {
            *cur = serde_json::Value::Object(serde_json::Map::new());
        }
    }
    cur.as_object_mut()
        .expect("object by construction")
        .insert(path[path.len() - 1].clone(), value);
}

impl EngineState {
    /// The ONE candidate under CSS-pixel `(x, y)` that the hover highlight lights
    /// and a plain click takes: the filter-admitted candidate FIRST IN THE
    /// RAYCAST by depth, the shared kind order breaking ties inside the nearest
    /// depth layer ([`front_layer_winner`](super::plane_pick::front_layer_winner)).
    /// `None` on a miss, or when the filter admits nothing.
    ///
    /// Built from [`candidates_filtered_at`](Self::candidates_filtered_at), so
    /// what the pointer promises is drawn from exactly the set the PICK LIST
    /// offers — one selection rule over one list, never a second pick with its
    /// own admission rules. The list's own ordering is untouched.
    ///
    /// The layer band is the camera's world-per-pixel times the picker's
    /// [`EDGE_PICK_PX`](crate::pick::EDGE_PICK_PX): the same 6 px neighbourhood
    /// the picker already accepts *laterally* for an edge or a vertex, applied in
    /// depth. It is a screen-space quantity, so it tracks zoom and model scale
    /// without a unit of its own. (A face seen at a grazing angle changes depth
    /// faster than that across 6 px, so its bounding edge can fall outside the
    /// band and lose to the face — click nearer the edge, or dwell for the list.)
    ///
    /// COMPONENT promotion mirrors
    /// [`select_filtered_at`](Self::select_filtered_at): with the filter's
    /// COMPONENT lane on, a winner that is component geometry answers as its
    /// owning COMPONENT, so the highlight lights the same whole component the
    /// click would select.
    pub fn nearest_candidate_at(&self, x: f64, y: f64) -> Option<pick::PickCandidate> {
        let candidates = self.candidates_filtered_at(x, y);
        let band = self.camera.world_per_pixel() * crate::pick::EDGE_PICK_PX;
        let winner = super::plane_pick::front_layer_winner(&candidates, band)?;
        if self.selection_filter.component && winner.kind != pick::PickKind::Component {
            if let Some(owner) = self.hit_owning_component(winner) {
                // The COMPONENT row for that owner is already in the list (the
                // builder appends one per owning component), carrying its
                // nearest member hit — reuse it rather than synthesizing one.
                if let Some(entry) = candidates
                    .iter()
                    .find(|c| c.kind == pick::PickKind::Component && c.name == owner)
                {
                    return Some(entry.clone());
                }
            }
        }
        Some(winner.clone())
    }

    /// Hover-highlight the candidate under CSS-pixel `(x, y)` that a plain click
    /// would take — the NEAREST admitted hit by depth
    /// ([`nearest_candidate_at`](Self::nearest_candidate_at)) — setting it
    /// HOVERED in `emphasis` (the renderer tints it). A miss — or a filter
    /// admitting nothing — clears the hover. No-ops (returns `false`, no dirty)
    /// when the hovered entity is unchanged, so a stationary pointer over the
    /// same face doesn't re-render every frame. Returns whether the hover state
    /// changed — the signal the viewport's dwell clock restarts on, so "how long
    /// has THIS highlight been showing" is answered without a second pick.
    ///
    /// The highlight is the promise the click keeps: both resolve through the one
    /// query above, so what lights up is what a plain click selects.
    pub fn hover_at(&mut self, x: f64, y: f64) -> bool {
        match self.nearest_candidate_at(x, y) {
            Some(hit) => {
                if self.hover_is(&hit) {
                    return false; // unchanged — keep the frame clean.
                }
                self.set_hover_to_candidate(&hit);
                true
            }
            None => self.clear_hover(),
        }
    }

    /// Whether ANY hover highlight is lit right now. The viewport's dwell clock
    /// runs only over a lit highlight (there is nothing to open a list about over
    /// empty space), and this is the cheap read of that.
    pub fn hover_is_lit(&self) -> bool {
        !self.emphasis.hovered_solids.is_empty()
            || !self.emphasis.hovered_faces.is_empty()
            || !self.emphasis.hovered_edges.is_empty()
            || !self.emphasis.hovered_vertices.is_empty()
            || !self.emphasis.hovered_datums.is_empty()
    }

    /// Clear the hover highlight (pointer moved to empty space / off the
    /// viewport). Bumps the emphasis generation + marks dirty only when a hover
    /// was actually lit. Returns whether it changed. (Distinct from
    /// [`clear_selection`](Self::clear_selection), which leaves hover alone.)
    pub fn clear_hover(&mut self) -> bool {
        let had_datums = !self.emphasis.hovered_datums.is_empty();
        let had = !self.emphasis.hovered_solids.is_empty()
            || !self.emphasis.hovered_faces.is_empty()
            || !self.emphasis.hovered_edges.is_empty()
            || !self.emphasis.hovered_vertices.is_empty()
            || had_datums;
        if had {
            self.emphasis.hovered_solids.clear();
            self.emphasis.hovered_faces.clear();
            self.emphasis.hovered_edges.clear();
            self.emphasis.hovered_vertices.clear();
            self.emphasis.hovered_datums.clear();
            self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
            self.dirty = true;
        }
        // A hovered plane's accent is baked into the datum feed, so dropping the
        // hover needs a re-feed to un-light it (the datum twin of the selection
        // re-feed in `clear_selection`).
        if had_datums {
            self.refresh_construction_datums();
        }
        had
    }

    /// The current HOVER (not selection) as JSON
    /// `{ solids:[..], faces:[..], edges:[..], vertices: n }` — the hover twin of
    /// [`selection_json`](Self::selection_json) so a UI / the headed verifier can
    /// assert that moving the pointer over a face lit the hover emphasis.
    pub fn hovered_json(&self) -> String {
        let solids: Vec<&String> = self.emphasis.hovered_solids.iter().collect();
        let faces: Vec<&String> = self.emphasis.hovered_faces.iter().collect();
        let edges: Vec<&String> = self.emphasis.hovered_edges.iter().collect();
        let datums: Vec<&String> = self.emphasis.hovered_datums.iter().collect();
        serde_json::json!({
            "solids": solids,
            "faces": faces,
            "edges": edges,
            "datums": datums,
            "vertices": self.emphasis.hovered_vertices.len(),
        })
        .to_string()
    }

    /// TOGGLE the admitted pick under CSS-pixel `(x, y)` — the NEAREST by depth,
    /// the one the hover highlight is showing — in the current
    /// selection (a **Ctrl/Cmd+click**): add it if absent, remove it if present,
    /// leaving the rest of the selection intact (unlike [`select_top_at`], which
    /// REPLACES). With the COMPONENT filter on, a hit on component geometry
    /// toggles the whole component (all member solids as one unit). A miss — or
    /// a filter admitting nothing — leaves the selection untouched (additive
    /// mode never clears). Returns whether a hit was toggled.
    pub fn select_toggle_at(&mut self, x: f64, y: f64) -> bool {
        // The SAME hit the hover highlight is showing
        // ([`nearest_candidate_at`](Self::nearest_candidate_at)) — a modifier
        // changes whether the pick replaces or adds, never WHICH entity it is,
        // so the highlight stays the promise. Its COMPONENT promotion carries
        // the whole component through as one unit, which is what
        // [`toggle_candidate`](Self::toggle_candidate) toggles for a COMPONENT
        // row, and it answers `None` when the filter admits nothing.
        match self.nearest_candidate_at(x, y) {
            Some(hit) => {
                self.toggle_candidate(&hit);
                true
            }
            None => false,
        }
    }

    /// The RANKED, filter-respecting candidates under CSS-pixel `(x, y)` as JSON
    /// `[{kind, name, solid, depth}]` — the "candidates under the cursor" list
    /// (feeds the pick-list popup + the headed verifier).
    ///
    /// Sorted category-major in the pick-list order (VERTEX > EDGE > FACE >
    /// PLANE > SOLID > COMPONENT), nearest (smallest depth) first within each
    /// category — see [`candidates_filtered_at`](Self::candidates_filtered_at).
    pub fn candidates_at(&self, x: f64, y: f64) -> String {
        let list = self.candidates_filtered_at(x, y);
        let out: Vec<serde_json::Value> = list
            .iter()
            .map(|c| {
                serde_json::json!({
                    "kind": self.candidate_kind_label(c),
                    "name": c.name,
                    "solid": c.solid,
                    "depth": c.depth,
                })
            })
            .collect();
        serde_json::Value::Array(out).to_string()
    }

    /// What a REGULAR (non-dwelling) viewport click at CSS-pixel `(x, y)` takes,
    /// as one JSON `{kind, name, solid, depth}` — the same shape a row of
    /// [`candidates_at`](Self::candidates_at) carries — or `null` on a miss.
    ///
    /// The automation bridge reads this to answer "would a click here reach the
    /// entity I named": since the trigger moved, the top ROW of the list and what
    /// a plain click resolves to are no longer the same question, and a script
    /// that guesses wrong picks the wrong thing silently.
    pub fn nearest_candidate_json(&self, x: f64, y: f64) -> String {
        match self.nearest_candidate_at(x, y) {
            Some(c) => serde_json::json!({
                "kind": self.candidate_kind_label(&c),
                "name": c.name,
                "solid": c.solid,
                "depth": c.depth,
            })
            .to_string(),
            None => "null".to_string(),
        }
    }

    /// The same ranked, filter-respecting candidate list as typed values (the
    /// in-process egui pick-list popup consumes these directly, then re-hovers /
    /// selects a chosen one via [`hover_candidate`](Self::hover_candidate) /
    /// [`select_candidate`](Self::select_candidate) /
    /// [`toggle_candidate`](Self::toggle_candidate)). EMPTY when the filter admits
    /// nothing (not the `pick_filtered` "empty filter = any" case).
    ///
    /// The raw list is [`pick_candidates_at`](Self::pick_candidates_at), so
    /// construction PLANE cards are ordinary entries here — a plane under other
    /// geometry is listed (right after the faces) instead of being reachable only
    /// on a geometry miss.
    ///
    /// With the filter's COMPONENT lane on, one COMPONENT entry per owning
    /// assembly component of ANY raw hit is appended (name = component id, depth
    /// = the component's nearest hit) — a raw hit of a filtered-OFF kind still
    /// reaches its owning component, mirroring `select_filtered_at`'s
    /// component-only promotion (a PLANE hit owns no component). The final list is
    /// sorted category-major in the pick-list order
    /// (VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT), nearest first within
    /// each category.
    pub fn candidates_filtered_at(&self, x: f64, y: f64) -> Vec<pick::PickCandidate> {
        let kinds = self.selection_filter.enabled_kinds();
        let component_on = self.selection_filter.component;
        if kinds.is_empty() && !component_on {
            return Vec::new();
        }
        let raw = self.pick_candidates_at(x, y);
        let mut out: Vec<pick::PickCandidate> = raw
            .iter()
            .filter(|c| self.candidate_admitted(&kinds, c))
            .cloned()
            .collect();
        if component_on {
            // One entry per owning component, carrying its NEAREST member hit's
            // depth/position (raw hits are kind-major, so scan them all).
            let mut components: Vec<pick::PickCandidate> = Vec::new();
            for hit in &raw {
                let Some(owner) = self.hit_owning_component(hit) else {
                    continue;
                };
                match components.iter_mut().find(|c| c.name == owner) {
                    Some(entry) => {
                        if hit.depth < entry.depth {
                            entry.depth = hit.depth;
                            entry.position = hit.position;
                        }
                    }
                    None => components.push(pick::PickCandidate {
                        kind: pick::PickKind::Component,
                        name: owner,
                        solid: String::new(),
                        depth: hit.depth,
                        screen_dist: hit.screen_dist,
                        position: hit.position,
                    }),
                }
            }
            out.extend(components);
        }
        // Category-major (PickKind's discriminant order IS the pick-list order),
        // nearest first within a category — the shared ordering every pick path
        // uses, so the appended COMPONENT rows land in the same sort.
        super::plane_pick::sort_pick_candidates(&mut out);
        out
    }

    /// Whether a candidate is CURRENTLY selected (drives the pick-list popup's
    /// per-row selected state so click-toggling reads back visually).
    pub fn candidate_is_selected(&self, candidate: &pick::PickCandidate) -> bool {
        use crate::pick::PickKind;
        match candidate.kind {
            PickKind::Solid => self
                .emphasis
                .selected_solids
                .contains(&self.candidate_solid_name(candidate)),
            PickKind::Face => self.emphasis.selected_faces.contains(&candidate.name),
            PickKind::Edge => self.emphasis.selected_edges.contains(&candidate.name),
            PickKind::Vertex => self
                .emphasis
                .selected_vertices
                .iter()
                .any(|v| Self::vertex_ref_matches(v, candidate)),
            PickKind::Plane => self.emphasis.selected_datums.contains(&candidate.name),
            PickKind::Component => {
                let members = self.component_member_solids(&candidate.name);
                !members.is_empty()
                    && members
                        .iter()
                        .all(|m| self.emphasis.selected_solids.contains(m))
            }
        }
    }

    /// Hover a SPECIFIC candidate (the popup entry the pointer is over) — sets it
    /// HOVERED in `emphasis`, replacing any prior hover.
    pub fn hover_candidate(&mut self, candidate: &pick::PickCandidate) {
        self.set_hover_to_candidate(candidate);
    }

    /// REPLACE the selection with a specific candidate (a plain click on a popup
    /// entry) — reuses the same bucketing as a plain viewport click.
    pub fn select_candidate(&mut self, candidate: &pick::PickCandidate) {
        self.set_selection_to_candidate(candidate);
    }

    /// TOGGLE a specific candidate in the selection (a Ctrl/Cmd+click on a popup
    /// entry, or the [`select_toggle_at`](Self::select_toggle_at) hit): add if
    /// absent, remove if present. Returns whether it is NOW selected (`true` =
    /// added, `false` = removed). Bumps the emphasis generation + marks dirty.
    pub fn toggle_candidate(&mut self, candidate: &pick::PickCandidate) -> bool {
        use crate::pick::PickKind;
        let now_selected = match candidate.kind {
            PickKind::Solid => {
                let name = self.candidate_solid_name(candidate);
                if self.emphasis.selected_solids.remove(&name) {
                    false
                } else {
                    self.emphasis.selected_solids.insert(name);
                    true
                }
            }
            PickKind::Face => {
                if self.emphasis.selected_faces.remove(&candidate.name) {
                    false
                } else {
                    self.emphasis.selected_faces.insert(candidate.name.clone());
                    true
                }
            }
            PickKind::Edge => {
                if self.emphasis.selected_edges.remove(&candidate.name) {
                    false
                } else {
                    self.emphasis.selected_edges.insert(candidate.name.clone());
                    true
                }
            }
            PickKind::Vertex => {
                if let Some(index) = self
                    .emphasis
                    .selected_vertices
                    .iter()
                    .position(|v| Self::vertex_ref_matches(v, candidate))
                {
                    self.emphasis.selected_vertices.remove(index);
                    false
                } else {
                    self.emphasis.selected_vertices.push(crate::style::VertexRef {
                        solid: candidate.solid.clone(),
                        position: candidate.position,
                    });
                    true
                }
            }
            PickKind::Plane => {
                // A construction PLANE toggles by FRAME NAME, the datum bucket the
                // Scene-tree row / `select_datum` fill.
                if self.emphasis.selected_datums.remove(&candidate.name) {
                    false
                } else {
                    self.emphasis.selected_datums.insert(candidate.name.clone());
                    true
                }
            }
            PickKind::Component => {
                // The whole component toggles as ONE unit (member solids), the
                // same rule as the Ctrl/Cmd+click COMPONENT promotion.
                let was_selected = self.candidate_is_selected(candidate);
                self.toggle_component_selection(&candidate.name);
                !was_selected
            }
        };
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        // A plane's accent is baked into the datum feed, so a toggled plane only
        // lights / un-lights after a re-feed.
        if candidate.kind == PickKind::Plane {
            self.refresh_construction_datums();
        }
        now_selected
    }

    /// Set the hover emphasis to exactly one candidate (bucketed by kind), the
    /// hover twin of `set_selection_to_candidate`.
    fn set_hover_to_candidate(&mut self, candidate: &pick::PickCandidate) {
        use crate::pick::PickKind;
        // A hovered PLANE's accent lives in the datum FEED, so the re-feed below
        // is needed both when a plane becomes hovered and when one stops being.
        let touches_datums =
            !self.emphasis.hovered_datums.is_empty() || candidate.kind == PickKind::Plane;
        self.emphasis.hovered_solids.clear();
        self.emphasis.hovered_faces.clear();
        self.emphasis.hovered_edges.clear();
        self.emphasis.hovered_vertices.clear();
        self.emphasis.hovered_datums.clear();
        match candidate.kind {
            PickKind::Solid => {
                self.emphasis
                    .hovered_solids
                    .insert(self.candidate_solid_name(candidate));
            }
            PickKind::Face => {
                self.emphasis.hovered_faces.insert(candidate.name.clone());
            }
            PickKind::Edge => {
                self.emphasis.hovered_edges.insert(candidate.name.clone());
            }
            PickKind::Vertex => {
                self.emphasis.hovered_vertices.push(crate::style::VertexRef {
                    solid: candidate.solid.clone(),
                    position: candidate.position,
                });
            }
            PickKind::Plane => {
                self.emphasis.hovered_datums.insert(candidate.name.clone());
            }
            PickKind::Component => {
                // Hovering a COMPONENT entry lights every member solid.
                for member in self.component_member_solids(&candidate.name) {
                    self.emphasis.hovered_solids.insert(member);
                }
            }
        }
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if touches_datums {
            self.refresh_construction_datums();
        }
    }

    /// Whether the CURRENT hover is exactly this one candidate (the `hover_at`
    /// early-out) — a single hovered entity that matches `candidate` (for a
    /// COMPONENT candidate: exactly its member-solid set). `pub(super)` so the
    /// Scene-tree row hover (`scene_query.rs`) shares the same identity test for
    /// its dedupe + its "only clear what the tree lit" rule.
    pub(super) fn hover_is(&self, candidate: &pick::PickCandidate) -> bool {
        use crate::pick::PickKind;
        if candidate.kind == PickKind::Component {
            let members = self.component_member_solids(&candidate.name);
            return !members.is_empty()
                && self.emphasis.hovered_faces.is_empty()
                && self.emphasis.hovered_edges.is_empty()
                && self.emphasis.hovered_vertices.is_empty()
                && self.emphasis.hovered_datums.is_empty()
                && self.emphasis.hovered_solids.len() == members.len()
                && members.iter().all(|m| self.emphasis.hovered_solids.contains(m));
        }
        let total = self.emphasis.hovered_solids.len()
            + self.emphasis.hovered_faces.len()
            + self.emphasis.hovered_edges.len()
            + self.emphasis.hovered_vertices.len()
            + self.emphasis.hovered_datums.len();
        if total != 1 {
            return false;
        }
        match candidate.kind {
            PickKind::Solid => self
                .emphasis
                .hovered_solids
                .contains(&self.candidate_solid_name(candidate)),
            PickKind::Face => self.emphasis.hovered_faces.contains(&candidate.name),
            PickKind::Edge => self.emphasis.hovered_edges.contains(&candidate.name),
            PickKind::Vertex => self
                .emphasis
                .hovered_vertices
                .iter()
                .any(|v| Self::vertex_ref_matches(v, candidate)),
            PickKind::Plane => self.emphasis.hovered_datums.contains(&candidate.name),
            PickKind::Component => false, // handled by the early return above
        }
    }

    /// The scene name a SOLID candidate resolves to (its owning `solid`, falling
    /// back to `name` when the pick didn't carry one) — the same rule
    /// `set_selection_to_candidate` uses.
    fn candidate_solid_name(&self, candidate: &pick::PickCandidate) -> String {
        if candidate.solid.is_empty() {
            candidate.name.clone()
        } else {
            candidate.solid.clone()
        }
    }

    /// Vertex identity: same owning solid + position within the emphasis match
    /// tolerance (vertices carry no kernel name, so they resolve by solid+pos).
    fn vertex_ref_matches(v: &crate::style::VertexRef, candidate: &pick::PickCandidate) -> bool {
        const TOL: f64 = 1e-4;
        v.solid == candidate.solid
            && (v.position[0] - candidate.position[0]).abs() <= TOL
            && (v.position[1] - candidate.position[1]).abs() <= TOL
            && (v.position[2] - candidate.position[2]).abs() <= TOL
    }
}

// ---------------------------------------------------------------------------
// Sketch display (S0) — read-only overlay of a solved SketchSession.
//
// Additive, self-contained: a solved sketch is fed to the general `set_overlay`
// channel as the named groups `sketch-geometry` (lines) and `sketch-points`
// (billboarded points), colored by solver mobility. No interaction (the tools /
// picking / dimensions of later slices live elsewhere); this block only pushes /
// clears the display geometry.
// ---------------------------------------------------------------------------
impl EngineState {
    /// Display a solved [`crate::sketch::SketchSession`] as a read-only overlay.
    /// The plane geometry is tessellated to world space and pushed via
    /// [`set_overlay_json`](Self::set_overlay_json); construction dashes are sized
    /// against the LIVE camera so they stay screen-constant.
    pub fn set_sketch_overlay(&mut self, session: &crate::sketch::SketchSession) {
        let world_per_pixel = self.camera.world_per_pixel();
        let json = session.overlay_json(world_per_pixel);
        // The overlay channel accepts our exact `{groups:[…]}` shape; a parse
        // failure would be a programming error in the tessellator, so drop it.
        let _ = self.set_overlay_json(&json);
        // The dimension leaders ride in their own `sketch-dim-leaders` group (S5).
        let _ = self.set_overlay_json(&session.dim_leaders_overlay_json(world_per_pixel));
        // The geometric-constraint glyphs ride in `sketch-constraint-glyphs` (S6c).
        let _ = self.set_overlay_json(&session.constraint_glyphs_overlay_json(world_per_pixel));
        // The zoom this screen-constant sizing was baked at, so the per-frame
        // `ensure_sketch_overlay_current` re-bakes it when the camera zooms.
        self.sketch_overlay_wpp = if world_per_pixel > 0.0 {
            world_per_pixel
        } else {
            f64::MIN_POSITIVE
        };
    }

    /// Remove the sketch overlay groups (feeding empty same-named groups upserts
    /// them to empty, which the overlay channel treats as a removal — other
    /// overlay groups are left untouched).
    pub fn clear_sketch_overlay(&mut self) {
        let _ = self.set_overlay_json(
            "{\"groups\":[{\"name\":\"sketch-geometry\"},{\"name\":\"sketch-points\"},{\"name\":\"sketch-preview\"},{\"name\":\"sketch-dim-leaders\"},{\"name\":\"sketch-constraint-glyphs\"}]}",
        );
        self.sketch_overlay_wpp = 0.0;
    }
}

