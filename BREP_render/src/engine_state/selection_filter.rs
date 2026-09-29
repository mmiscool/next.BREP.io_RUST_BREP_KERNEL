use super::*;

/// Which entity KINDS a plain viewport click may select.
///
/// The default enables **EVERY kind** (SOLID + FACE + EDGE + VERTEX + PLANE), so
/// out of any special mode every entity under the cursor is pickable and the
/// highest-priority kind wins (vertices/edges/faces out-priority a construction
/// plane, which in turn out-priorities the owning solid, matching the picker's
/// priority order VERTEX > EDGE > FACE > PLANE > SOLID > COMPONENT).
/// Reference-selection mode narrows this to
/// exactly the kinds the active field permits (see [`from_ref_filter`] +
/// [`EngineState::begin_ref_select`]); finishing / cancelling restores this
/// all-enabled default.
///
/// [`from_ref_filter`]: SelectionFilter::from_ref_filter
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SelectionFilter {
    pub solid: bool,
    /// Committed SKETCHES, as whole objects. A committed sketch is drawn as a
    /// synthesized sheet SOLID (`SolidDisplay::is_sketch`), so it arrives from the
    /// ray test as an ordinary [`pick::PickKind::Solid`] candidate — the two share
    /// a `PickKind` and are told apart by that flag, not by the picker (see
    /// [`EngineState::candidate_admitted`]). Splitting them into two lanes is what
    /// lets a `["SKETCH"]` reference field pick a sketch in the viewport while a
    /// `["SOLID"]` one (a boolean target, a mirror body) does not — a sketch sheet
    /// registers no KERNEL solid, so a sketch name in a `SOLID` field could never
    /// resolve anyway.
    ///
    /// It governs WHOLE-sketch picks only. A sketch's planar face
    /// (`{sketch}:FACE`) and its drawn segments (`{sketch}:G{gid}`) stay ordinary
    /// FACE and EDGE candidates under those lanes, deliberately: the profile
    /// fields pick a sketch THROUGH its face (`["SKETCH","FACE"]`, then
    /// `normalize_profile_alias`) and the path fields pick its curves through
    /// EDGE — path sweep's `path` is `["EDGE"]` alone and would lose sketch
    /// curves entirely if this lane gated them.
    pub sketch: bool,
    pub face: bool,
    pub edge: bool,
    pub vertex: bool,
    /// Construction PLANES (a `P` feature's frame, a `D` datum's three base
    /// planes). A real pickable kind: the drawn plane cards join the ordinary
    /// candidate list (see [`EngineState::pick_candidates_at`]), ranked right
    /// after faces, so a plane is reachable THROUGH other geometry via the pick
    /// list. Unchecking it excludes planes exactly as unchecking Face excludes
    /// faces. The reference field's construction kinds (`PLANE`/`DATUM`) map here.
    pub plane: bool,
    /// COMPONENT promotion: a pick landing on assembly-component geometry
    /// selects the WHOLE component (its member solids — the tree / component-
    /// action selection shape) instead of the sub-entity. Not a `pick_filtered`
    /// kind — it re-buckets the accepted hit. Default ON: in an assembly a
    /// plain click grabs the component; uncheck it to reach faces/edges of
    /// component geometry (constraint work). Non-component geometry never
    /// promotes, so part documents are unaffected.
    pub component: bool,
}

impl Default for SelectionFilter {
    fn default() -> Self {
        Self {
            solid: true,
            sketch: true,
            face: true,
            edge: true,
            vertex: true,
            plane: true,
            component: true,
        }
    }
}

impl SelectionFilter {
    /// The enabled kinds as pick-filter strings, highest pick-priority FIRST
    /// (`VERTEX > EDGE > FACE > PLANE > SOLID`) — the shape
    /// [`EngineState::pick_top_at`] / [`pick::pick_filtered`] consume.
    /// EMPTY when nothing is enabled; the caller MUST treat an empty result as
    /// "select nothing" (NOT the "empty filter = any kind" case).
    pub fn enabled_kinds(&self) -> Vec<String> {
        let mut kinds = Vec::new();
        if self.vertex {
            kinds.push("VERTEX".to_string());
        }
        if self.edge {
            kinds.push("EDGE".to_string());
        }
        if self.face {
            kinds.push("FACE".to_string());
        }
        if self.plane {
            kinds.push("PLANE".to_string());
        }
        if self.solid {
            kinds.push("SOLID".to_string());
        }
        // SKETCH is an ALIAS classification of the same `PickKind::Solid`, not a
        // rank of its own: a candidate is one or the other, never both, so where
        // it sits relative to `SOLID` cannot change which candidate wins. It is
        // listed so `candidate_admitted` can see it.
        if self.sketch {
            kinds.push("SKETCH".to_string());
        }
        kinds
    }

    /// Whether ANY kind is enabled (else a click can select nothing).
    /// COMPONENT counts: a component-only filter still picks component geometry.
    pub fn any_enabled(&self) -> bool {
        self.solid
            || self.sketch
            || self.face
            || self.edge
            || self.vertex
            || self.plane
            || self.component
    }

    /// Read one kind's pickability by name
    /// (`"SOLID"`/`"FACE"`/`"EDGE"`/`"VERTEX"`/`"PLANE"`, case-insensitive;
    /// `"DATUM"` is an alias of `"PLANE"`); unknown kinds read `false`.
    pub fn get(&self, kind: &str) -> bool {
        match kind.to_ascii_uppercase().as_str() {
            "SOLID" => self.solid,
            "SKETCH" => self.sketch,
            "FACE" => self.face,
            "EDGE" => self.edge,
            "VERTEX" => self.vertex,
            // Both construction spellings resolve to the one plane lane: the
            // drawn plane cards are the only construction entity that picks.
            "PLANE" | "DATUM" => self.plane,
            "COMPONENT" => self.component,
            _ => false,
        }
    }

    /// Set one kind's pickability by name (unknown kinds are ignored).
    pub fn set(&mut self, kind: &str, on: bool) {
        match kind.to_ascii_uppercase().as_str() {
            "SOLID" => self.solid = on,
            "SKETCH" => self.sketch = on,
            "FACE" => self.face = on,
            "EDGE" => self.edge = on,
            "VERTEX" => self.vertex = on,
            "PLANE" | "DATUM" => self.plane = on,
            "COMPONENT" => self.component = on,
            _ => {}
        }
    }

    /// Build a selection filter from a reference field's allowed-type strings
    /// (its schema `selectionFilter`, e.g. `["SOLID"]`, `["FACE","EDGE"]`,
    /// `["PLANE","FACE"]`, `["DATUM"]`). EVERY kind now maps one-to-one to a
    /// pickable lane: `SOLID`/`SKETCH`/`FACE`/`EDGE`/`VERTEX` to their own
    /// (`SKETCH` and `SOLID` split the one `PickKind::Solid` between them — see
    /// [`sketch`](Self::sketch)), and the
    /// CONSTRUCTION spellings `PLANE`/`DATUM` both to [`plane`](Self::plane) —
    /// construction planes are ordinary pick candidates (see
    /// [`EngineState::pick_candidates_at`]), no longer a `datum_pick`-only path.
    ///
    /// The result stays RESTRICTIVE: `["PLANE","FACE"]` (a sketch's `sketchPlane`)
    /// admits planes and faces and NOTHING else — never solids or edges — and
    /// `["PLANE"]`/`["DATUM"]` admits only the plane cards. The all-enabled
    /// fallback fires ONLY for a genuinely empty/unknown filter, so a field is
    /// never left in the unusable "select nothing" state. Reference-selection mode
    /// installs the result as the live `selection_filter` so BOTH
    /// hover-highlighting and click-picking follow the field's allowed kinds.
    pub fn from_ref_filter(filter: &[String]) -> Self {
        let mut f = SelectionFilter {
            solid: false,
            sketch: false,
            face: false,
            edge: false,
            vertex: false,
            plane: false,
            component: false,
        };
        for kind in filter {
            f.set(kind, true); // unknown kinds ignored by `set`.
        }
        if f.component {
            // A COMPONENT field previews/highlights through member SOLIDS.
            f.solid = true;
        }
        if f.any_enabled() {
            f
        } else {
            // Genuinely empty/unknown filter → all-enabled, so the filter is never
            // empty-and-unusable.
            Self::default()
        }
    }
}

impl EngineState {
    /// The current selection filter (a cheap `Copy`) — a panel reads it to seed
    /// its toggles, edits the copy, and writes back via [`set_selection_filter`].
    pub fn selection_filter(&self) -> SelectionFilter {
        self.selection_filter
    }

    /// Replace the whole selection filter. Purely a policy change (no geometry or
    /// camera moves), so it does NOT mark the scene dirty; the NEXT click honors it.
    pub fn set_selection_filter(&mut self, filter: SelectionFilter) {
        self.selection_filter = filter;
    }

    /// Toggle one kind's pickability by name (`"SOLID"`/`"FACE"`/`"EDGE"`/`"VERTEX"`).
    pub fn set_kind_pickable(&mut self, kind: &str, on: bool) {
        self.selection_filter.set(kind, on);
    }

    /// The filter as JSON (`{"SOLID":true,…,"PLANE":true,"COMPONENT":true}`) —
    /// lets a UI / the headed verifier read the pickable-kind state.
    pub fn selection_filter_json(&self) -> String {
        serde_json::json!({
            "SOLID": self.selection_filter.solid,
            "SKETCH": self.selection_filter.sketch,
            "FACE": self.selection_filter.face,
            "EDGE": self.selection_filter.edge,
            "VERTEX": self.selection_filter.vertex,
            "PLANE": self.selection_filter.plane,
            "COMPONENT": self.selection_filter.component,
        })
        .to_string()
    }

    /// Apply a partial filter JSON (any subset of the keys); absent keys keep
    /// their current value. The round-trip counterpart of [`selection_filter_json`].
    pub fn apply_selection_filter_json(&mut self, json: &str) -> Result<(), String> {
        let value: serde_json::Value = serde_json::from_str(json)
            .map_err(|error| format!("selection filter parse: {error}"))?;
        for kind in ["SOLID", "SKETCH", "FACE", "EDGE", "VERTEX", "PLANE", "COMPONENT"] {
            if let Some(on) = value.get(kind).and_then(|v| v.as_bool()) {
                self.selection_filter.set(kind, on);
            }
        }
        Ok(())
    }

    /// The filter-honoring plain-click selection (what [`select_top_at`] delegates
    /// to): resolve the TOP-priority candidate under `(x, y)` whose kind the filter
    /// admits (via [`pick_top_at`](Self::pick_top_at) with the enabled kinds —
    /// scene entities AND construction plane cards) and select THAT kind, replacing
    /// the current selection. A miss — or a click while the filter
    /// admits nothing — clears the selection. Returns whether something selected.
    pub fn select_filtered_at(&mut self, x: f64, y: f64) -> bool {
        let kinds = self.selection_filter.enabled_kinds();
        let component_on = self.selection_filter.component;
        if kinds.is_empty() && !component_on {
            // The filter admits no kind → a click selects nothing (and clears).
            self.clear_selection();
            return false;
        }
        // COMPONENT-only: no sub-entity kind is admitted, but component geometry
        // still is — pick against every kind and keep only component-owned hits.
        let pick_kinds = if kinds.is_empty() {
            SelectionFilter::default().enabled_kinds()
        } else {
            kinds.clone()
        };
        let Some(hit) = self.pick_top_at(x, y, &pick_kinds) else {
            self.clear_selection();
            return false;
        };
        // COMPONENT promotion: a hit on component geometry selects the WHOLE
        // component (member solids — the tree / component-action shape).
        if component_on {
            if let Some(owner) = self.hit_owning_component(&hit) {
                self.select_component(&owner);
                return true;
            }
        }
        if kinds.is_empty() {
            // COMPONENT-only, and the hit was not component geometry.
            self.clear_selection();
            return false;
        }
        self.set_selection_to_candidate(&hit);
        true
    }

    /// The assembly component owning a pick candidate's geometry, if any (the
    /// COMPONENT-promotion resolver: solid-keyed for solids/vertices,
    /// name-keyed for faces/edges — `component_of_solid` parses either).
    ///
    /// A construction PLANE never promotes: it is not component geometry, and its
    /// frame NAME (`Datum:XY`) shares the `{component}:{part}` shape a namespaced
    /// solid uses, so keying on it would conjure a phantom component from a datum
    /// id. Guarded here, at the one resolver both the click promotion and the
    /// pick-list's COMPONENT rows go through.
    pub(super) fn hit_owning_component(&self, hit: &pick::PickCandidate) -> Option<String> {
        if hit.kind == pick::PickKind::Plane {
            return None;
        }
        let key = if hit.solid.is_empty() {
            hit.name.as_str()
        } else {
            hit.solid.as_str()
        };
        if key.is_empty() {
            return None;
        }
        self.component_of_solid(key)
    }

    /// Replace the selection with a single picked candidate, bucketed by its kind
    /// (SOLID → owning solid, FACE/EDGE → kernel name, VERTEX → solid+position,
    /// PLANE → the datum FRAME name — the same bucket a Scene-tree datum row or
    /// [`select_datum`](Self::select_datum) fills).
    pub(super) fn set_selection_to_candidate(&mut self, candidate: &pick::PickCandidate) {
        use crate::pick::PickKind;
        // A real entity pick supersedes any selected datum (they share the one
        // selection); re-feed the datum planes afterward to drop its accent — and
        // likewise a PLANE pick needs the re-feed to PAINT its accent.
        let had_datum = !self.emphasis.selected_datums.is_empty()
            || candidate.kind == PickKind::Plane;
        self.emphasis.selected_solids.clear();
        self.emphasis.selected_faces.clear();
        self.emphasis.selected_edges.clear();
        self.emphasis.selected_vertices.clear();
        self.emphasis.selected_datums.clear();
        match candidate.kind {
            PickKind::Solid => {
                let name = if candidate.solid.is_empty() {
                    candidate.name.clone()
                } else {
                    candidate.solid.clone()
                };
                self.emphasis.selected_solids.insert(name);
            }
            PickKind::Face => {
                self.emphasis.selected_faces.insert(candidate.name.clone());
            }
            PickKind::Edge => {
                self.emphasis.selected_edges.insert(candidate.name.clone());
            }
            PickKind::Vertex => {
                self.emphasis.selected_vertices.push(crate::style::VertexRef {
                    solid: candidate.solid.clone(),
                    position: candidate.position,
                });
            }
            PickKind::Plane => {
                // The candidate came from the LIVE datum-plane feed, so its frame
                // name is by construction a shown D/P plane — the same name
                // `select_datum` validates before selecting.
                self.emphasis.selected_datums.insert(candidate.name.clone());
            }
            PickKind::Component => {
                // A COMPONENT pick-list entry selects the whole component: its
                // member solids as one unit (the COMPONENT-promotion shape).
                for member in self.component_member_solids(&candidate.name) {
                    self.emphasis.selected_solids.insert(member);
                }
            }
        }
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if had_datum {
            self.refresh_construction_datums();
        }
    }

    /// Whether ANYTHING is currently selected (solids/faces/edges/vertices OR a
    /// construction datum/plane) — distinct from `emphasis.is_empty()` (which also
    /// counts hover). Drives the selection action bar's visibility: a datum-only
    /// selection must show the bar so the plane can offer "Sketch".
    pub fn has_selection(&self) -> bool {
        !self.emphasis.selected_solids.is_empty()
            || !self.emphasis.selected_faces.is_empty()
            || !self.emphasis.selected_edges.is_empty()
            || !self.emphasis.selected_vertices.is_empty()
            || !self.emphasis.selected_datums.is_empty()
    }

    /// Resolve the current selection to its visibility TARGETS — exactly what the
    /// user selected, at the granularity they selected it: a selected SOLID targets
    /// the whole solid ([`HideTarget::Solid`]); a selected FACE / EDGE / VERTEX
    /// targets that single sub-entity ([`HideTarget::Entity`], the same
    /// (solid, kind, index) the Scene-tree checkboxes drive through
    /// [`set_entity_visible`](Self::set_entity_visible)).
    ///
    /// Faces/edges are name-keyed in the emphasis WITHOUT their owning solid, so we
    /// scan the scene: EVERY (solid, index) whose entity name matches a selected
    /// name becomes a target (a name is not guaranteed unique across solids —
    /// matching all mirrors the old owning-solid scan's `.any()`). Empty names are
    /// skipped (unnamed entities would otherwise all collide on `""`). Vertices
    /// carry no name, so a [`VertexRef`] resolves by owning solid + position
    /// (`TOL`-matched, the same constant the Scene panel snapshot uses).
    fn selection_hide_targets(&self) -> Vec<HideTarget> {
        use crate::visibility::EntityKind;
        const TOL: f64 = 1e-6;
        let mut targets: Vec<HideTarget> = Vec::new();

        for name in &self.emphasis.selected_solids {
            if !name.is_empty() {
                targets.push(HideTarget::Solid(name.clone()));
            }
        }
        // Faces / edges: match the selected NAMES against every solid's entities.
        let face_sel = &self.emphasis.selected_faces;
        let edge_sel = &self.emphasis.selected_edges;
        if !face_sel.is_empty() || !edge_sel.is_empty() {
            for solid in self.scene.solids() {
                for (index, f) in solid.faces.iter().enumerate() {
                    if !f.name.is_empty() && face_sel.contains(&f.name) {
                        targets.push(HideTarget::Entity {
                            solid: solid.name.clone(),
                            kind: EntityKind::Face,
                            index,
                        });
                    }
                }
                for (index, e) in solid.edges.iter().enumerate() {
                    if !e.name.is_empty() && edge_sel.contains(&e.name) {
                        targets.push(HideTarget::Entity {
                            solid: solid.name.clone(),
                            kind: EntityKind::Edge,
                            index,
                        });
                    }
                }
            }
        }
        // Vertices: resolve each ref to its index in the owning solid by position.
        for vref in &self.emphasis.selected_vertices {
            if let Some(solid) = self.scene.solid(&vref.solid) {
                if let Some(index) = solid.vertices.iter().position(|v| {
                    (v.position[0] - vref.position[0]).abs() <= TOL
                        && (v.position[1] - vref.position[1]).abs() <= TOL
                        && (v.position[2] - vref.position[2]).abs() <= TOL
                }) {
                    targets.push(HideTarget::Entity {
                        solid: vref.solid.clone(),
                        kind: EntityKind::Vertex,
                        index,
                    });
                }
            }
        }
        targets
    }

    /// Toggle the visibility of EXACTLY what is selected (the action bar's
    /// **Hide/Show**): each selected SOLID flips its whole-solid visibility
    /// ([`set_visible`](Self::set_visible)); each selected FACE / EDGE / VERTEX
    /// flips only that sub-entity ([`set_entity_visible`](Self::set_entity_visible),
    /// the same per-entity mask the Scene-tree checkboxes use — a hidden face's
    /// triangles are simply not drawn). Every target toggles INDEPENDENTLY off its
    /// own live state (hide if visible, show if hidden), so a mixed / repeat click
    /// flips each item. Returns how many targets were toggled. Leaves the selection
    /// as-is, so a second click toggles the same items back.
    pub fn hide_selected(&mut self) -> usize {
        let mut toggled = 0usize;
        for target in self.selection_hide_targets() {
            match target {
                HideTarget::Solid(name) => {
                    let visible = match self.scene.solid(&name) {
                        Some(s) => s.visible,
                        None => continue,
                    };
                    if self.set_visible(&name, !visible) {
                        toggled += 1;
                    }
                }
                HideTarget::Entity { solid, kind, index } => {
                    let visible = match self.entity_visible(&solid, kind, index) {
                        Some(v) => v,
                        None => continue,
                    };
                    if self.set_entity_visible(&solid, kind, index, !visible) {
                        toggled += 1;
                    }
                }
            }
        }
        toggled
    }
}

/// One resolved visibility target for [`EngineState::hide_selected`]: a whole
/// solid, or a single face/edge/vertex addressed by owning solid + kind + index.
enum HideTarget {
    Solid(String),
    Entity {
        solid: String,
        kind: crate::visibility::EntityKind,
        index: usize,
    },
}

