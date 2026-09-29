use super::*;

impl EngineState {
    /// Ranked candidate list under CSS-pixel `(x, y)`, kernel names, priority
    /// VERTEX > EDGE > FACE > … > SOLID.
    ///
    /// SCENE ONLY, deliberately: this R3-boundary accessor (and its
    /// [`hover_json`](Self::hover_json) sibling) reports kernel-named GEOMETRY, and
    /// its in-tree consumer is the sketch's external-edge picker, which wants
    /// edges. The construction PLANE cards join the pick list one level up, in
    /// [`pick_candidates_at`](Self::pick_candidates_at) — that is what the
    /// selection paths and the app's pick-list popup consume.
    pub fn pick_json(&self, x: f64, y: f64) -> String {
        let candidates = pick::pick(&self.scene, &self.camera, x, y, &self.pick_options());
        pick::candidates_to_json(&candidates)
    }

    /// The single best candidate under `(x, y)` (hover), or `null`.
    pub fn hover_json(&self, x: f64, y: f64) -> String {
        let candidates = pick::pick(&self.scene, &self.camera, x, y, &self.pick_options());
        match candidates.first() {
            Some(best) => pick::candidates_to_json(std::slice::from_ref(best))
                .strip_prefix('[')
                .and_then(|s| s.strip_suffix(']'))
                .map(str::to_string)
                .unwrap_or_else(|| "null".to_string()),
            None => "null".to_string(),
        }
    }

    // --- Settings / emphasis / visibility (R11/R14/R17) -------------------

    pub fn apply_settings_json(&mut self, json: &str) -> Result<(), String> {
        let prev_lod = self.settings.lod_factor;
        let prev_override_model_colors = self.settings.override_model_colors;
        self.settings.apply_json(json)?;
        self.settings_generation = self.settings_generation.wrapping_add(1);
        self.dirty = true;
        // "Override model colors" is resolved when colours are derived, not when
        // they are drawn, so flipping it has to re-derive. Guarded on an actual
        // change: this apply path also runs on every unrelated settings edit.
        if self.settings.override_model_colors != prev_override_model_colors {
            self.sync_colors_from_metadata();
        }
        // Push the (possibly changed) ViewCube size into the widget so the rendered
        // cube AND its hit-test rect track the setting. Always re-pushed (idempotent
        // for an unchanged value) so this ONE choke point covers panel edits, boot
        // restore, and Reset-to-defaults alike.
        self.widgets.set_viewcube_size(self.settings.viewcube_size_px);
        // Projection rides in the settings JSON as `orthographic` (see `settings_json`)
        // so the toolbar toggle AND a reload both go through this ONE apply path — the
        // same way wireframe does. It is NOT a `RenderSettings` field: read it straight
        // off the JSON and drive the camera. A PARTIAL apply (the wireframe toggle's
        // `{"wireframe":true}`) omits the key and leaves the projection untouched, and
        // the panel's full-buffer apply carries the live value (so it's a no-op).
        if let Some(want_ortho) = serde_json::from_str::<serde_json::Value>(json)
            .ok()
            .and_then(|v| v.get("orthographic").and_then(|o| o.as_bool()))
        {
            let is_ortho = matches!(
                self.camera.projection,
                crate::view::Projection::Orthographic { .. }
            );
            if want_ortho != is_ortho {
                self.set_projection(if want_ortho { "orthographic" } else { "perspective" });
            }
        }
        // The LOD factor scales DISPLAY tessellation, so a change must re-run so the
        // resident meshes re-tessellate at the new chord tolerance (the runner drops
        // its reuse baseline when the lod differs). Every OTHER setting is pure
        // render state and needs no re-run. Skip the re-run when there are no
        // features (e.g. boot restores a saved `lodFactor` before any document is
        // loaded): the run would be empty, and the real doc load re-runs with the
        // lod already injected.
        if self.settings.lod_factor != prev_lod && !self.history.is_empty() {
            self.rerun_history();
        }
        // Sketch colors live in the settings too: when a sketch is being edited, push
        // the (possibly) new palette into the live session and re-push the overlay so
        // an edited color takes effect immediately (mirrors how a wireframe/lod change
        // refreshes the view). Compute the palette first to avoid a split borrow.
        if self.sketch_edit.is_some() {
            let colors = self.settings.sketch_colors();
            if let Some(edit) = self.sketch_edit.as_mut() {
                edit.session.colors = colors;
            }
            self.refresh_sketch_overlay();
        }
        Ok(())
    }

    /// The FULL current settings as JSON (the round-trip counterpart of
    /// [`apply_settings_json`]): the schema-driven form seeds its widgets from
    /// this and the storage seam persists it.
    pub fn settings_json(&self) -> String {
        // Projection is live CAMERA state surfaced to the settings layer as a boolean
        // (`orthographic`) so the toolbar toggle persists and the settings panel can
        // round-trip it without clobbering. DERIVE it from the camera here — it is
        // never a stored `RenderSettings` field — so it can NEVER drift from the
        // actual projection no matter which code path last changed it.
        let mut value: serde_json::Value =
            serde_json::from_str(&self.settings.to_json()).unwrap_or(serde_json::Value::Null);
        if let Some(obj) = value.as_object_mut() {
            obj.insert(
                "orthographic".into(),
                serde_json::Value::Bool(matches!(
                    self.camera.projection,
                    crate::view::Projection::Orthographic { .. }
                )),
            );
        }
        value.to_string()
    }

    /// The current per-solid metadata color overrides as JSON —
    /// `[{"name": "...", "override": "#rrggbb" | null}, …]`. Lets a UI list the
    /// scene's solids with their current override so the picker reflects state.
    pub fn solid_color_overrides_json(&self) -> String {
        let solids: Vec<serde_json::Value> = self
            .scene
            .solids()
            .iter()
            .map(|solid| {
                let over = solid.color_override.map(|rgb| {
                    let q = |c: f32| (c.clamp(0.0, 1.0) * 255.0).round() as u32;
                    format!("#{:02x}{:02x}{:02x}", q(rgb[0]), q(rgb[1]), q(rgb[2]))
                });
                serde_json::json!({ "name": solid.name, "override": over })
            })
            .collect();
        serde_json::Value::Array(solids).to_string()
    }

    pub fn apply_emphasis_json(&mut self, json: &str) -> Result<(), String> {
        self.emphasis.apply_json(json)?;
        self.dirty = true;
        Ok(())
    }

    pub fn set_visible(&mut self, name: &str, visible: bool) -> bool {
        let ok = self.scene.set_visible(name, visible);
        if ok {
            // A derived BOARD body is dropped by every run's scene reconcile
            // and re-seated from the KEPT display, so the hide has to reach
            // that copy or it lasts only until the next feature edit.
            self.remember_board_visibility(name, visible);
            self.dirty = true;
        }
        ok
    }

    pub fn scene_listing_json(&self) -> String {
        self.scene.listing_json()
    }

    // --- Overlay widgets --------------------------------------------------

    /// The BASE bbox the camera depth-range fit starts from: the visible SOLIDS
    /// unioned with the pushed OVERLAY groups (sketch curves/points, dimension
    /// leaders, constraint glyphs — the `set_overlay` channel). Folding in the
    /// groups stops orbiting an editing sketch from clipping it against the
    /// solids-only bounds (the reported clipping when "Lock to sketch" is off).
    /// The render path ([`Self::fit_camera_and_overlay`]) unions the FULL widget
    /// overlay's world bounds (datum planes, world axes, frames, transform
    /// gizmo — NOT in this bbox's channels) and the world origin on top of this
    /// before fitting, so construction geometry never clips. Callers must bind
    /// this to a local before `camera.fit_depth_range` (which needs
    /// `&mut self.camera`).
    pub fn depth_range_bbox(&self) -> crate::camera::Aabb {
        let mut bbox = self.scene.bbox();
        bbox.union(&self.widgets.overlay_groups_bbox());
        bbox
    }

}

impl EngineState {
    /// A RICHER scene listing than [`scene_listing_json`](Self::scene_listing_json)
    /// (which is counts only): per solid the individual face + edge kernel NAMES
    /// and vertex refs (topo id + world position), plus visibility — the shape the
    /// engine-native Scene tree lists entities from and the headed verifier asserts
    /// against. Vertices carry no kernel name, so they are keyed by topo id + world
    /// position (the same shape the emphasis vertex-ref selection uses).
    pub fn scene_entities_json(&self) -> String {
        let solids: Vec<serde_json::Value> = self
            .scene
            .solids()
            .iter()
            // Committed-sketch SHEETS are scene solids (pickable/measurable) but list
            // under "Sketches" (`committed_sketches`), not among the real solids.
            .filter(|solid| !solid.is_sketch)
            .map(|solid| {
                let faces: Vec<&str> = solid.faces.iter().map(|f| f.name.as_str()).collect();
                // Display-only (aux) edges are not entities: a board's copper
                // outline would list thousands of unnamed ones.
                let edges: Vec<&str> = solid.edges.iter().filter(|e| !e.aux).map(|e| e.name.as_str()).collect();
                let vertices: Vec<serde_json::Value> = solid
                    .vertices
                    .iter()
                    .map(|v| serde_json::json!({ "topoId": v.topo_id, "position": v.position }))
                    .collect();
                serde_json::json!({
                    "name": solid.name,
                    "visible": solid.visible,
                    "faces": faces,
                    "edges": edges,
                    "vertices": vertices,
                })
            })
            .collect();
        serde_json::Value::Array(solids).to_string()
    }

    /// Drive the engine SELECTION by kernel NAME from a UI tree (the name-based
    /// analogue of [`select_top_at`](Self::select_top_at), which picks under the
    /// cursor). Replaces the current selection with the single named `solid` /
    /// `face` / `edge` so clicking a Scene-tree row highlights that entity in the
    /// viewport (the render pass reads `emphasis`). Vertices have no kernel name —
    /// use [`select_vertex_by_position`](Self::select_vertex_by_position). Returns
    /// false for an unknown `kind` or an empty `name`.
    pub fn select_by_name(&mut self, kind: &str, name: &str) -> bool {
        // A construction datum/plane routes to its own name-keyed selection.
        if kind == "datum" {
            return self.select_datum(name);
        }
        if name.is_empty() || !matches!(kind, "solid" | "face" | "edge") {
            return false;
        }
        let had_datum = !self.emphasis.selected_datums.is_empty();
        self.emphasis.selected_solids.clear();
        self.emphasis.selected_faces.clear();
        self.emphasis.selected_edges.clear();
        self.emphasis.selected_vertices.clear();
        self.emphasis.selected_datums.clear();
        match kind {
            "solid" => {
                self.emphasis.selected_solids.insert(name.to_string());
            }
            "face" => {
                self.emphasis.selected_faces.insert(name.to_string());
            }
            "edge" => {
                self.emphasis.selected_edges.insert(name.to_string());
            }
            _ => unreachable!(),
        }
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if had_datum {
            self.refresh_construction_datums();
        }
        true
    }

    /// Select a single vertex by its owning solid + world position — vertices have
    /// no kernel name, so emphasis keys them by solid + position (matched with a
    /// tolerance in the render pass). Replaces the current selection. Returns false
    /// for an empty solid name.
    pub fn select_vertex_by_position(&mut self, solid: &str, position: [f64; 3]) -> bool {
        if solid.is_empty() {
            return false;
        }
        let had_datum = !self.emphasis.selected_datums.is_empty();
        self.emphasis.selected_solids.clear();
        self.emphasis.selected_faces.clear();
        self.emphasis.selected_edges.clear();
        self.emphasis.selected_vertices.clear();
        self.emphasis.selected_datums.clear();
        self.emphasis.selected_vertices.push(crate::style::VertexRef {
            solid: solid.to_string(),
            position,
        });
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if had_datum {
            self.refresh_construction_datums();
        }
        true
    }

    // -----------------------------------------------------------------------
    // Scene-tree row HOVER → the viewport hover highlight
    //
    // The hover twin of the name-based selection above: mousing over a Scene-tree
    // row lights the entity EXACTLY as mousing over it in the 3D view does,
    // because it feeds the SAME `emphasis` hover buckets through the SAME
    // `hover_candidate` bucketing the viewport's `hover_at` uses (so a hovered
    // datum plane gets its accent re-feed, a solid lights whole, and the render
    // pass needs no new concept). The pointer is off the viewport while it is over
    // a row, so the viewport's pointer-left-the-viewport `clear_hover` would
    // otherwise wipe it every frame — hence the one-frame yield flag, the twin of
    // `take_sketch_list_hover` / `take_constraint_label_hover`.
    // -----------------------------------------------------------------------

    /// Hover-highlight by kernel NAME from a UI tree — the hover twin of
    /// [`select_by_name`](Self::select_by_name) (`kind` is `"solid"` / `"face"` /
    /// `"edge"` / `"datum"`; vertices carry no kernel name, so they use
    /// [`hover_vertex_by_position`](Self::hover_vertex_by_position)).
    ///
    /// Deliberately does NOT consult the selection filter, matching the row's
    /// CLICK: a row selects whatever kind it is, so its hover previews the same
    /// entity. An unknown `kind`, an empty `name` or an unresolved datum frame is
    /// refused without touching the current hover.
    ///
    /// Call it EVERY frame the row is hovered: it re-arms the one-frame
    /// [`take_scene_tree_hover`](Self::take_scene_tree_hover) yield flag and
    /// re-lights nothing when the hover is already this entity. Returns whether
    /// the hover CHANGED (the [`hover_at`](Self::hover_at) convention) so the
    /// caller can request a repaint.
    pub fn hover_by_name(&mut self, kind: &str, name: &str) -> bool {
        if name.is_empty() {
            return false;
        }
        let pick_kind = match kind {
            "solid" => pick::PickKind::Solid,
            "face" => pick::PickKind::Face,
            "edge" => pick::PickKind::Edge,
            // A construction datum/plane hovers by FRAME name — the bucket
            // `select_datum` and the viewport's plane picks fill.
            "datum" => pick::PickKind::Plane,
            _ => return false,
        };
        let candidate = pick::PickCandidate {
            kind: pick_kind,
            name: name.to_string(),
            // A SOLID candidate resolves through `candidate_solid_name`, which
            // prefers `solid` — and this row IS the solid.
            solid: match pick_kind {
                pick::PickKind::Solid => name.to_string(),
                _ => String::new(),
            },
            depth: 0.0,
            screen_dist: 0.0,
            position: [0.0; 3],
        };
        self.set_scene_tree_hover(candidate)
    }

    /// Hover a single vertex by its owning solid + world position — the hover twin
    /// of [`select_vertex_by_position`](Self::select_vertex_by_position) (vertices
    /// have no kernel name, so emphasis keys them by solid + position). Same
    /// per-frame contract and return as [`hover_by_name`](Self::hover_by_name).
    pub fn hover_vertex_by_position(&mut self, solid: &str, position: [f64; 3]) -> bool {
        if solid.is_empty() {
            return false;
        }
        self.set_scene_tree_hover(pick::PickCandidate {
            kind: pick::PickKind::Vertex,
            // Vertices are unnamed: `solid` + `position` is the identity.
            name: String::new(),
            solid: solid.to_string(),
            depth: 0.0,
            screen_dist: 0.0,
            position,
        })
    }

    /// The pointer left the Scene tree's rows: drop the row-driven highlight.
    /// Clears ONLY a hover the tree itself lit and that is STILL lit — if the
    /// viewport has since hovered something else (it may draw before the panel in
    /// the dock), that hover is left alone. Safe to call every frame no row is
    /// hovered: the target is taken, so it is a no-op from the second call on.
    /// Returns whether the hover changed.
    pub fn scene_tree_hover_end(&mut self) -> bool {
        match self.scene_tree_hovered.take() {
            Some(candidate) if self.hover_is(&candidate) => self.clear_hover(),
            _ => false,
        }
    }

    /// Consume the one-frame "a Scene-tree row is hovering an entity" flag — the
    /// viewport's modeling hover branch skips its pointer-off-viewport
    /// `clear_hover` while it is set (mirrors
    /// [`take_sketch_list_hover`](Self::take_sketch_list_hover)) so the row-driven
    /// highlight survives the frame instead of being cleared and re-applied.
    pub fn take_scene_tree_hover(&mut self) -> bool {
        std::mem::take(&mut self.scene_tree_hover_active)
    }

    /// Light one Scene-tree row's candidate: arm the yield flag (every frame — the
    /// viewport consumes it every frame), record the target so
    /// [`scene_tree_hover_end`](Self::scene_tree_hover_end) knows what the tree
    /// lit, and re-emphasize only when the hover actually changed (a held row must
    /// not re-bump the emphasis generation / re-feed the datums every frame).
    fn set_scene_tree_hover(&mut self, candidate: pick::PickCandidate) -> bool {
        self.scene_tree_hover_active = true;
        match self.apply_ui_hover(&candidate) {
            // Refused (an unresolved datum frame): nothing lit, so the tree records
            // no target and has nothing to end.
            None => false,
            Some(changed) => {
                // Recorded even when unchanged: the viewport may have lit the
                // identical entity, and the tree still owns ending it.
                self.scene_tree_hovered = Some(candidate);
                changed
            }
        }
    }

    /// Apply ONE UI-driven hover candidate to the emphasis — the shared body
    /// behind every list→viewport highlight (the Scene tree's rows, the dialog
    /// rows below). `None` = REFUSED, `Some(changed)` = applied, and `changed`
    /// says whether the hover actually moved (a held row must not re-bump the
    /// emphasis generation / re-feed the datums every frame).
    ///
    /// Each caller records its OWN target: that is what keeps two panes' `_end`
    /// calls from ending each other's highlight.
    fn apply_ui_hover(&mut self, candidate: &pick::PickCandidate) -> Option<bool> {
        if self.hover_is(candidate) {
            return Some(false);
        }
        // Only a RESOLVED datum frame hovers (the `select_datum` guard); an
        // unresolved name would light nothing and churn the datum feed. Checked on
        // the change path only — the dedupe above carries the held frames.
        if candidate.kind == pick::PickKind::Plane
            && !self.construction_frames.iter().any(|(n, _)| *n == candidate.name)
        {
            return None;
        }
        self.hover_candidate(candidate);
        Some(true)
    }
}

// ---------------------------------------------------------------------------
// DIALOG row HOVER → the viewport hover highlight
//
// The Scene tree knows the KIND of every row it draws; a dialog does not. A
// feature form's reference line, its read-only `Outputs` line and the picker
// card's picked-name line all carry a bare kernel NAME, so this lane resolves the
// kind from the scene itself and then feeds the SAME hover buckets the tree and
// the viewport's `hover_at` fill.
// ---------------------------------------------------------------------------
impl EngineState {
    /// Hover-highlight the entity a DIALOG row names — the kind-less twin of
    /// [`hover_by_name`](Self::hover_by_name), for the lists that show a bare
    /// kernel name (a reference line, an `Outputs` line, the picker card).
    ///
    /// `owner` is the pane that is hovering (`"history"`, `"constraints"`,
    /// `"pmi"`, `"refsel"`): [`dialog_hover_end`](Self::dialog_hover_end) acts
    /// only for the owner that set the hover, so panes that are on screen together
    /// in a split dock cannot end each other's highlight.
    ///
    /// The name is resolved by [`resolve_entity_name`](Self::resolve_entity_name),
    /// which also covers the row whose entity the feature CONSUMED — an open
    /// fillet form is rolled to the fillet, so its `Edges` rows name edges that no
    /// longer exist and the blend faces the kernel built from them are lit
    /// instead. Call it EVERY frame the row is hovered: the resolution is memoized
    /// on the row text and the emphasis re-feed is deduped, so a held hover costs
    /// nothing. Returns whether the hover CHANGED (the
    /// [`hover_at`](Self::hover_at) convention), so the caller can repaint.
    pub fn hover_entity_by_name(&mut self, owner: &'static str, name: &str) -> bool {
        if name.is_empty() {
            return false;
        }
        // Armed even for a row that resolves to nothing: the pointer is over the
        // dialog, so the viewport's pointer-left-the-viewport `clear_hover` must
        // still yield — otherwise moving along a list of rows would strobe the
        // highlight off on every unresolvable one.
        self.dialog_hover_active = true;
        let memo = match &self.dialog_hovered {
            Some(held) if held.owner == owner && held.row == name => Some(held.candidate.clone()),
            _ => None,
        };
        let candidate = match memo {
            Some(cached) => cached,
            None => self.resolve_entity_name(name),
        };
        let applied = match &candidate {
            Some(c) => self.apply_ui_hover(c),
            None => None,
        };
        let changed = match applied {
            Some(changed) => changed,
            // Nothing to light (the scene carries no such name, or the candidate
            // was refused): end whatever the slot holds — WHOEVER set it. Leaving
            // the previous row's entity standing while the pointer sits on a
            // different row reads as "this row is that entity", and the previous
            // row is not always this owner's: moving from a History line straight
            // onto an unresolvable Constraints one (a component-local vertex ref)
            // would otherwise strand the History highlight with nobody left
            // holding the record that could end it.
            None => self.take_dialog_hover_slot(),
        };
        self.dialog_hovered = Some(super::DialogHover {
            owner,
            row: name.to_string(),
            candidate,
        });
        changed
    }

    /// The pointer left `owner`'s dialog rows: drop the row-driven highlight.
    /// Clears ONLY a hover THIS owner lit and that is STILL lit — a pane that
    /// never hovered a row (or whose hover the viewport has since replaced) is a
    /// no-op, which is what lets every consumer call it unconditionally every
    /// frame. Returns whether the hover changed.
    pub fn dialog_hover_end(&mut self, owner: &'static str) -> bool {
        let mine = matches!(&self.dialog_hovered, Some(held) if held.owner == owner);
        if mine {
            self.take_dialog_hover_slot()
        } else {
            false
        }
    }

    /// Take the dialog-hover slot and drop the highlight it recorded, when that
    /// highlight is still the live one (the viewport may have replaced it since).
    /// Owner-AGNOSTIC: [`dialog_hover_end`](Self::dialog_hover_end) checks the
    /// owner before calling this, while a row that lights nothing calls it
    /// directly — the pointer is on THAT row now, so whatever the slot still
    /// records is stale whoever set it.
    fn take_dialog_hover_slot(&mut self) -> bool {
        match self.dialog_hovered.take() {
            Some(held) => match held.candidate {
                Some(candidate) if self.hover_is(&candidate) => self.clear_hover(),
                _ => false,
            },
            None => false,
        }
    }

    /// Consume the one-frame "a dialog row is hovering an entity" flag — the
    /// viewport's modeling hover branch skips its pointer-off-viewport
    /// `clear_hover` while it is set (the twin of
    /// [`take_scene_tree_hover`](Self::take_scene_tree_hover)).
    pub fn take_dialog_hover(&mut self) -> bool {
        std::mem::take(&mut self.dialog_hover_active)
    }

    /// Resolve one bare kernel NAME to the entity it addresses, or `None` when the
    /// scene carries nothing by that name.
    ///
    /// Order — most specific identity first: a SOLID (a shown committed sketch is
    /// a scene solid keyed by its sketch id — `refresh_committed_sketches` inserts
    /// the sheet under the feature id an extrude's `profile` stores — so a
    /// `SKETCH` reference lands here whenever that sheet is shown), a
    /// construction datum FRAME, a FACE, an EDGE, then a `{solid}@x,y,z` VERTEX
    /// ref (the position-keyed form assembly constraints and PMI store, accepted
    /// only when it really is a vertex of that solid — a component-LOCAL ref lights
    /// nothing rather than lighting the wrong vertex).
    ///
    /// # The consumed row
    ///
    /// A feature's form is drawn with the model rolled TO that feature, so the
    /// feature has RUN and the entities its references name are gone: a fillet's
    /// `Edges` rows name edges the blend replaced. The kernel names what it builds
    /// after what it consumed — `{featureId}:BLEND:{originatingEdgeName}`, asserted
    /// by the kernel's own `fillet_by_face_ref_rounds_the_top_ring` /
    /// `chamfer` naming tests — so a row that resolves to nothing falls back to the
    /// FACE whose name ends with `:{row}`: the thing the feature made from it.
    /// Only when EXACTLY ONE face matches, so an ambiguous mapping lights nothing
    /// rather than an arbitrary half of it. Ends-with, never `contains`: the edges
    /// bounding that blend are named `BOX_NZ|F1:BLEND:BOX_NX|BOX_NZ[0][0]`, which
    /// CONTAINS the row but is not what it became.
    fn resolve_entity_name(&self, name: &str) -> Option<pick::PickCandidate> {
        let at = |kind: pick::PickKind, entity: &str, solid: &str| pick::PickCandidate {
            kind,
            name: entity.to_string(),
            solid: solid.to_string(),
            depth: 0.0,
            screen_dist: 0.0,
            position: [0.0; 3],
        };
        let solids = self.scene.solids();
        if solids.iter().any(|s| s.name == name) {
            // A SOLID candidate resolves through `candidate_solid_name`, which
            // prefers `solid` — and this row IS the solid.
            return Some(at(pick::PickKind::Solid, name, name));
        }
        if self.construction_frames.iter().any(|(n, _)| n.as_str() == name) {
            return Some(at(pick::PickKind::Plane, name, ""));
        }
        for solid in solids {
            if solid.faces.iter().any(|f| f.name == name) {
                return Some(at(pick::PickKind::Face, name, &solid.name));
            }
        }
        for solid in solids {
            if solid.edges.iter().any(|e| e.name == name) {
                return Some(at(pick::PickKind::Edge, name, &solid.name));
            }
        }
        if let Some(candidate) = self.resolve_vertex_ref(name) {
            return Some(candidate);
        }
        let suffix = format!(":{name}");
        let mut derived = solids
            .iter()
            .flat_map(|solid| solid.faces.iter().map(move |face| (solid, face)))
            .filter(|(_, face)| face.name.ends_with(&suffix));
        match (derived.next(), derived.next()) {
            (Some((solid, face)), None) => {
                Some(at(pick::PickKind::Face, &face.name, &solid.name))
            }
            _ => None,
        }
    }

    /// A `{solid}@x,y,z` VERTEX reference → its candidate, when the position
    /// really is a vertex of that solid within the emphasis match tolerance.
    /// Vertices carry no kernel name, so this is the only form a dialog can list
    /// one under; PMI stores WORLD coordinates (which resolve here), assembly
    /// constraints store component-LOCAL ones (which do not, and so light nothing
    /// rather than the wrong vertex).
    fn resolve_vertex_ref(&self, name: &str) -> Option<pick::PickCandidate> {
        /// The tolerance the render pass matches an emphasis vertex ref at.
        const TOL: f64 = 1e-4;
        let (solid_name, coords) = name.rsplit_once('@')?;
        let mut parts = coords.split(',').map(|p| p.trim().parse::<f64>().ok());
        let position = [parts.next()??, parts.next()??, parts.next()??];
        if parts.next().is_some() {
            return None;
        }
        let solid = self.scene.solids().iter().find(|s| s.name == solid_name)?;
        solid.vertices.iter().find(|v| {
            (v.position[0] - position[0]).abs() <= TOL
                && (v.position[1] - position[1]).abs() <= TOL
                && (v.position[2] - position[2]).abs() <= TOL
        })?;
        Some(pick::PickCandidate {
            kind: pick::PickKind::Vertex,
            name: String::new(),
            solid: solid_name.to_string(),
            depth: 0.0,
            screen_dist: 0.0,
            position,
        })
    }
}


impl EngineState {
    /// The resident kernel handle of every solid currently displayed, keyed by
    /// name. Obtained by replaying the CURRENT rolled-to history prefix through
    /// [`brep_kernel::execute_history`]: after a build the incremental cache
    /// holds exactly this prefix, so the replay is a clean cache hit — it
    /// re-tessellates nothing and hands back the SAME handles the scene was built
    /// from (mirrors the pipeline's `fold_history`: removals then additions).
    /// `pub(super)`: the interference check (`engine_state::interference`) reads
    /// the same warm main-side handle map for its non-destructive booleans.
    pub(super) fn resident_solid_handles(&self) -> HashMap<String, u32> {
        let request: HistoryRequest = match serde_json::from_value(self.run_request_value()) {
            Ok(request) => request,
            Err(_) => return HashMap::new(),
        };
        let _trace = crate::run_trace::span("resident_solid_handles");
        let result = brep_kernel::execute_history(&request);
        let mut handles: HashMap<String, u32> = HashMap::new();
        for feature in &result.results {
            for removed in &feature.removed {
                handles.remove(removed);
            }
            for added in &feature.added {
                handles.insert(added.name.clone(), added.handle);
            }
        }
        handles
    }

    /// Mass properties for the Inspector panel, from the kernel's exact
    /// (divergence-theorem) integrator. `name = Some(solid)` reports that resident
    /// solid; `None` reports the whole model. `density` (mass units per mm³; the
    /// kernel length convention is millimetres) scales `mass` and the inertia
    /// tensor — the centroid and principal axes are density-independent.
    ///
    /// Returns JSON:
    /// ```json
    /// { "ok": true, "target": "Box", "solidCount": 1, "density": 1.0,
    ///   "volume": 5738.05, "surfaceArea": 2927.79, "mass": 5738.05,
    ///   "centroid": [10.0, 10.0, 10.0],
    ///   "inertia": [[..],[..],[..]] | null,
    ///   "principalMoments": [a,b,c] | null,
    ///   "principalAxes": [[..],[..],[..]] | null }
    /// ```
    /// A single resolved solid carries the full centroidal inertia tensor +
    /// principal axes/moments; a multi-solid aggregate reports summed volume /
    /// area / mass and the volume-weighted centroid, with the tensor fields
    /// `null` (select one solid for its inertia). `ok:false` with a `message` on
    /// no solids / an unknown name / an integrator failure.
    pub fn mass_properties_json(&self, name: Option<&str>, density: f64) -> String {
        let handles = self.resident_solid_handles();

        // Resolve the target solids: a named solid (must be resident) or, for the
        // whole model, every scene solid that has resident geometry (draw order).
        let targets: Vec<String> = match name {
            Some(name) if handles.contains_key(name) => vec![name.to_string()],
            Some(name) => {
                return serde_json::json!({
                    "ok": false,
                    "message": format!("solid '{name}' has no resident geometry"),
                })
                .to_string();
            }
            None => self
                .scene
                .solids()
                .iter()
                .map(|solid| solid.name.clone())
                .filter(|name| handles.contains_key(name))
                .collect(),
        };
        if targets.is_empty() {
            return serde_json::json!({ "ok": false, "message": "no solids" }).to_string();
        }

        // Per-solid density mass properties straight from the kernel.
        let mut props = Vec::with_capacity(targets.len());
        for target in &targets {
            match brep_kernel::mass_properties_handle_native(handles[target], density) {
                Ok(properties) => props.push(properties),
                Err(error) => {
                    return serde_json::json!({
                        "ok": false,
                        "message": format!("{target}: {error}"),
                    })
                    .to_string();
                }
            }
        }

        let target_label = if targets.len() == 1 {
            targets[0].clone()
        } else {
            "(whole model)".to_string()
        };

        if props.len() == 1 {
            // Single solid: the full tensor + principal frame are meaningful.
            let p = &props[0];
            serde_json::json!({
                "ok": true,
                "target": target_label,
                "solidCount": 1,
                "density": p.density,
                "volume": p.volume,
                "surfaceArea": p.surface_area,
                "mass": p.mass,
                "centroid": [p.centroid.x, p.centroid.y, p.centroid.z],
                "inertia": p.inertia,
                "principalMoments": p.principal_moments,
                "principalAxes": p.principal_axes,
            })
            .to_string()
        } else {
            // Aggregate: additive scalars + volume-weighted centroid. Combining
            // the tensors needs a parallel-axis shift per solid; left to the
            // single-solid view rather than approximated here.
            let volume: f64 = props.iter().map(|p| p.volume).sum();
            let surface_area: f64 = props.iter().map(|p| p.surface_area).sum();
            let mass: f64 = props.iter().map(|p| p.mass).sum();
            let centroid = if volume.abs() > f64::EPSILON {
                let mut acc = [0.0f64; 3];
                for p in &props {
                    acc[0] += p.volume * p.centroid.x;
                    acc[1] += p.volume * p.centroid.y;
                    acc[2] += p.volume * p.centroid.z;
                }
                [acc[0] / volume, acc[1] / volume, acc[2] / volume]
            } else {
                [0.0, 0.0, 0.0]
            };
            serde_json::json!({
                "ok": true,
                "target": target_label,
                "solidCount": props.len(),
                "density": density,
                "volume": volume,
                "surfaceArea": surface_area,
                "mass": mass,
                "centroid": centroid,
                "inertia": serde_json::Value::Null,
                "principalMoments": serde_json::Value::Null,
                "principalAxes": serde_json::Value::Null,
            })
            .to_string()
        }
    }
}


