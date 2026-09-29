use super::*;

/// The calm base color of an unselected construction datum/plane (a soft blue).
const DATUM_PLANE_COLOR: &str = "#6b8fd0";
/// The selection accent for a selected datum/plane (matches `faceSelectedColor`).
const DATUM_PLANE_SELECTED_COLOR: &str = "#ffc400";
/// The HOVER accent for a datum/plane under the pointer (matches the faces'
/// default `hoverColor`) — distinct from the selection accent, so a plane reads
/// hovered-vs-selected exactly like a face does.
const DATUM_PLANE_HOVERED_COLOR: &str = "#fbff00";

impl EngineState {
    /// Map every feature id at the CURRENT rollback (`0..=rollback`) to its TYPE
    /// token — the lookup that classifies a frame name's producing feature so only
    /// DATUM (`"D"`) / PLANE (`"P"`) frames display as datum planes (a SKETCH `"S"`
    /// frame renders as curves, not a datum).
    fn feature_type_map(&self) -> HashMap<String, String> {
        let rollback = self.history.rollback();
        let mut map = HashMap::new();
        for index in 0..=rollback {
            if let (Some(id), Some(ty)) =
                (self.history.feature_id(index), self.history.feature_type(index))
            {
                map.insert(id, ty);
            }
        }
        map
    }

    /// Classify a plane-frame NAME against the feature type map: strip a trailing
    /// DATUM sub-plane suffix (`:XY`/`:XZ`/`:YZ`) to the producing feature id, look
    /// up its type, and keep only `"D"`/`"P"` producers. Returns `(producing
    /// feature id, feature type)` for a datum/plane frame, else `None` (a SKETCH
    /// `"S"` frame, or a feature past the rollback / not in the history).
    fn datum_feature_of(
        name: &str,
        type_map: &HashMap<String, String>,
    ) -> Option<(String, String)> {
        let base = [":XY", ":XZ", ":YZ"]
            .iter()
            .find_map(|suffix| name.strip_suffix(suffix))
            .unwrap_or(name);
        let ty = type_map.get(base)?;
        if ty == "D" || ty == "P" {
            Some((base.to_string(), ty.clone()))
        } else {
            None
        }
    }

    /// The construction datum/plane frame NAMES the last run resolved, filtered to
    /// the D/P producing features at the current rollback, in run order. Every
    /// DATUM contributes three (`{id}:XY|XZ|YZ`), every PLANE one (`{id}`); a
    /// SKETCH's own plane frame is excluded (it renders as curves).
    fn construction_datum_names(&self) -> Vec<String> {
        let type_map = self.feature_type_map();
        self.construction_frames
            .iter()
            .filter(|(name, _)| Self::datum_feature_of(name, &type_map).is_some())
            .map(|(name, _)| name.clone())
            .collect()
    }

    /// The producing `(feature id, feature type)` of a datum/plane frame NAME, but
    /// ONLY when the name is an actually-resolved D/P frame at the current rollback
    /// — the provenance the Properties Info tab reports for a selected datum.
    pub fn datum_feature_for_name(&self, name: &str) -> Option<(String, String)> {
        if !self.construction_frames.iter().any(|(n, _)| n == name) {
            return None;
        }
        Self::datum_feature_of(name, &self.feature_type_map())
    }

    /// (Re)build the persistent construction datum/plane overlays. Feeds every D/P
    /// frame the last run resolved (minus [`hidden_datums`]) to the datum-plane
    /// widget channel as a screen-constant NAMED plane in the calm datum color — or
    /// the selection accent when it is in `emphasis.selected_datums` / hovered when
    /// it is in `emphasis.hovered_datums` (hover wins, the `EmphasisState` order).
    /// The feed
    /// REPLACES the widget's datum set wholesale, so a departed/hidden/rolled-back
    /// plane is auto-dropped; `shown_datum_names` mirrors what was fed. Marks dirty.
    ///
    /// The fed set is also exactly what is PICKABLE: `plane_candidates_at` hit-tests
    /// these cards, so a hidden or rolled-back plane can no more be picked than it
    /// can be seen.
    pub fn refresh_construction_datums(&mut self) {
        let type_map = self.feature_type_map();
        let mut planes: Vec<serde_json::Value> = Vec::new();
        let mut fed: Vec<String> = Vec::new();
        for (name, frame) in &self.construction_frames {
            if Self::datum_feature_of(name, &type_map).is_none() {
                continue;
            }
            if self.hidden_datums.contains(name) {
                continue;
            }
            let selected = self.emphasis.selected_datums.contains(name);
            let hovered = self.emphasis.hovered_datums.contains(name);
            // Hover WINS over selected — the `EmphasisState` order faces follow.
            let color = if hovered {
                DATUM_PLANE_HOVERED_COLOR
            } else if selected {
                DATUM_PLANE_SELECTED_COLOR
            } else {
                DATUM_PLANE_COLOR
            };
            planes.push(serde_json::json!({
                "name": name,
                "origin": [frame.origin.x, frame.origin.y, frame.origin.z],
                "x": [frame.x_axis.x, frame.x_axis.y, frame.x_axis.z],
                "y": [frame.y_axis.x, frame.y_axis.y, frame.y_axis.z],
                "color": color,
                "selected": selected,
                "hovered": hovered,
            }));
            fed.push(name.clone());
        }
        // `set_datums` replaces its whole datum set, so a full re-feed each call
        // drops any plane no longer present (rolled back / deleted / hidden).
        let payload = serde_json::json!({ "planes": planes }).to_string();
        let _ = self.set_datums_json(&payload);
        self.shown_datum_names = fed;
        self.dirty = true;
    }

    /// Whether the construction datum/plane `name`'s plane is shown (absent from
    /// [`hidden_datums`] = visible).
    pub fn datum_visible(&self, name: &str) -> bool {
        !self.hidden_datums.contains(name)
    }

    /// Show/hide the construction datum/plane `name`'s plane (the Scene-tree
    /// checkbox). Toggles [`hidden_datums`] and re-feeds the datum planes so the
    /// plane appears/disappears immediately.
    pub fn set_datum_visible(&mut self, name: &str, visible: bool) {
        if visible {
            self.hidden_datums.remove(name);
        } else {
            self.hidden_datums.insert(name.to_string());
        }
        self.refresh_construction_datums();
    }

    /// The construction datums/planes to list in the Scene tree: every D/P frame at
    /// the current rollback, each with its live visibility (hidden ones included,
    /// like [`committed_sketches`](Self::committed_sketches)).
    pub fn construction_datums(&self) -> Vec<(String, bool)> {
        crate::visibility::named_visibility(self.construction_datum_names(), &self.hidden_datums)
    }

    /// The construction datums/planes as JSON (`[{"name","visible"}]`) — the datum
    /// sibling of [`sketch_entities_json`](Self::sketch_entities_json) the Scene
    /// panel publishes (`__brepDatums`) for the headed verifier.
    pub fn datum_entities_json(&self) -> String {
        crate::visibility::named_visibility_json(self.construction_datums())
    }

    /// Select a construction datum/plane by frame NAME (replacing the whole
    /// selection): a Scene-tree row click or a viewport datum pick. Only a name
    /// that is an actually-resolved D/P frame at the current rollback selects;
    /// others return false without changing the selection. Re-feeds the datum
    /// planes so the selected one shows the accent, and bumps the generation.
    pub fn select_datum(&mut self, name: &str) -> bool {
        if name.is_empty() || !self.construction_frames.iter().any(|(n, _)| n == name) {
            return false;
        }
        if Self::datum_feature_of(name, &self.feature_type_map()).is_none() {
            return false;
        }
        self.emphasis.selected_solids.clear();
        self.emphasis.selected_faces.clear();
        self.emphasis.selected_edges.clear();
        self.emphasis.selected_vertices.clear();
        self.emphasis.selected_datums.clear();
        self.emphasis.selected_datums.insert(name.to_string());
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.refresh_construction_datums();
        self.dirty = true;
        true
    }
}



