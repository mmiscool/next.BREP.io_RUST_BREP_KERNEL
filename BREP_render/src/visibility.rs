//! Visibility masks for solid faces, edges, vertices, and named overlays.
//!
//! [`EntityVisibility`] stores hidden indices; [`GroupState`] gives Scene-tree
//! checkboxes a tristate readout. Reused solids retain their masks, while newly
//! tessellated solids start visible. The renderer skips hidden geometry and
//! coalesces contiguous visible face ranges without rebuilding meshes.

use crate::engine_state::EngineState;
use crate::scene::{RenderScene, SolidDisplay};
use std::collections::HashSet;

/// Preserve scene-list order, including hidden entries for their checkboxes.
pub(crate) fn named_visibility(
    names: impl IntoIterator<Item = String>,
    hidden: &HashSet<String>,
) -> Vec<(String, bool)> {
    names.into_iter().map(|name| {
        let visible = !hidden.contains(&name);
        (name, visible)
    }).collect()
}

pub(crate) fn named_visibility_json(entries: Vec<(String, bool)>) -> String {
    let list: Vec<serde_json::Value> = entries.into_iter()
        .map(|(name, visible)| serde_json::json!({ "name": name, "visible": visible }))
        .collect();
    serde_json::Value::Array(list).to_string()
}

/// Which sub-entity list of a solid a visibility toggle addresses.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityKind {
    Face,
    Edge,
    Vertex,
}

impl EntityKind {
    /// Parse the Scene-tree / verifier kind token (`"face"` / `"edge"` /
    /// `"vertex"`, case-insensitive). `None` for anything else.
    pub fn parse(kind: &str) -> Option<Self> {
        match kind.to_ascii_lowercase().as_str() {
            "face" => Some(Self::Face),
            "edge" => Some(Self::Edge),
            "vertex" => Some(Self::Vertex),
            _ => None,
        }
    }

    /// The lowercase token (round-trips with [`parse`](Self::parse)).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Face => "face",
            Self::Edge => "edge",
            Self::Vertex => "vertex",
        }
    }
}

/// A group checkbox's tristate: every entity of the kind shown, none shown, or a
/// mix. An empty group reads [`All`](GroupState::All) — nothing to hide.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GroupState {
    All,
    Partial,
    None,
}

impl GroupState {
    /// The lowercase token the verifier asserts against (`"all"` / `"partial"` /
    /// `"none"`).
    pub fn as_str(self) -> &'static str {
        match self {
            Self::All => "all",
            Self::Partial => "partial",
            Self::None => "none",
        }
    }
}

/// The hidden-entity sets of ONE solid (empty = all visible). Indices are into
/// the owning [`SolidDisplay`]'s `faces` / `edges` / `vertices`.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct EntityVisibility {
    hidden_faces: HashSet<usize>,
    hidden_edges: HashSet<usize>,
    hidden_vertices: HashSet<usize>,
}

impl EntityVisibility {
    fn set_of(&self, kind: EntityKind) -> &HashSet<usize> {
        match kind {
            EntityKind::Face => &self.hidden_faces,
            EntityKind::Edge => &self.hidden_edges,
            EntityKind::Vertex => &self.hidden_vertices,
        }
    }

    fn set_mut(&mut self, kind: EntityKind) -> &mut HashSet<usize> {
        match kind {
            EntityKind::Face => &mut self.hidden_faces,
            EntityKind::Edge => &mut self.hidden_edges,
            EntityKind::Vertex => &mut self.hidden_vertices,
        }
    }

    /// Whether entity `index` of `kind` is currently shown.
    pub fn is_visible(&self, kind: EntityKind, index: usize) -> bool {
        !self.set_of(kind).contains(&index)
    }

    /// Whether ANY entity of `kind` is hidden — the render pass's fast-path
    /// guard (nothing hidden ⇒ keep the single whole-buffer draw).
    pub fn any_hidden(&self, kind: EntityKind) -> bool {
        !self.set_of(kind).is_empty()
    }

    /// Show/hide a single entity of `kind`.
    pub fn set_visible(&mut self, kind: EntityKind, index: usize, visible: bool) {
        if visible {
            self.set_mut(kind).remove(&index);
        } else {
            self.set_mut(kind).insert(index);
        }
    }

    /// Show/hide EVERY entity of `kind`. `count` is the number of entities of
    /// that kind, so "hide all" enumerates them (and "show all" clears).
    pub fn set_group_visible(&mut self, kind: EntityKind, count: usize, visible: bool) {
        let set = self.set_mut(kind);
        set.clear();
        if !visible {
            set.extend(0..count);
        }
    }

    /// The group tristate for `count` entities of `kind`. O(1): the hidden set
    /// satisfies `hidden ⊆ 0..count` (see [`all_hidden`](Self::all_hidden)), so its
    /// `len()` IS the hidden count — no per-entity scan. Called per solid per egui
    /// frame by the Scene panel, so the O(1) matters on point/edge-heavy models.
    pub fn group_state(&self, kind: EntityKind, count: usize) -> GroupState {
        if count == 0 {
            return GroupState::All;
        }
        let hidden = self.set_of(kind).len();
        if hidden == 0 {
            GroupState::All
        } else if hidden >= count {
            GroupState::None
        } else {
            GroupState::Partial
        }
    }

    // Render-pass conveniences (kept terse — called per solid per frame).
    pub fn is_face_visible(&self, index: usize) -> bool {
        self.is_visible(EntityKind::Face, index)
    }
    pub fn is_edge_visible(&self, index: usize) -> bool {
        self.is_visible(EntityKind::Edge, index)
    }
    pub fn is_vertex_visible(&self, index: usize) -> bool {
        self.is_visible(EntityKind::Vertex, index)
    }
    pub fn any_face_hidden(&self) -> bool {
        self.any_hidden(EntityKind::Face)
    }
    pub fn any_edge_hidden(&self) -> bool {
        self.any_hidden(EntityKind::Edge)
    }
    pub fn any_vertex_hidden(&self) -> bool {
        self.any_hidden(EntityKind::Vertex)
    }

    /// Whether EVERY entity of `kind` is hidden — O(1). A solid's hidden set is
    /// always paired with the entity counts it was built against: geometry changes
    /// replace the whole `SolidDisplay` (fresh, empty visibility), and a reused
    /// display keeps its counts AND its hidden set together — so `hidden ⊆ 0..count`
    /// holds universally and `len() == count` ⇔ the whole group is off (individual
    /// hides leave `len() < count`). `>=` rather than `==` is defensive only. The
    /// render pass uses this to skip a fully-hidden group's per-vertex draw loop.
    pub fn all_hidden(&self, kind: EntityKind, count: usize) -> bool {
        count > 0 && self.set_of(kind).len() >= count
    }
    pub fn all_vertices_hidden(&self, count: usize) -> bool {
        self.all_hidden(EntityKind::Vertex, count)
    }
}

/// The count of entities of `kind` on a solid (group toggles / tristate).
fn entity_count(solid: &SolidDisplay, kind: EntityKind) -> usize {
    match kind {
        EntityKind::Face => solid.faces.len(),
        EntityKind::Edge => solid.edges.len(),
        EntityKind::Vertex => solid.vertices.len(),
    }
}

// --- RenderScene: per-entity / group visibility (kept here, not in scene.rs, so
//     the visibility surface stays in one module) ------------------------------
impl RenderScene {
    /// Show/hide ONE face/edge/vertex of a solid. False if the solid is unknown.
    pub fn set_entity_visible(
        &mut self,
        solid: &str,
        kind: EntityKind,
        index: usize,
        visible: bool,
    ) -> bool {
        match self.solid_mut(solid) {
            Some(s) => {
                s.visibility.set_visible(kind, index, visible);
                true
            }
            None => false,
        }
    }

    /// Show/hide a whole group (all faces / all edges / all vertices) of a solid.
    /// False if the solid is unknown.
    pub fn set_group_visible(&mut self, solid: &str, kind: EntityKind, visible: bool) -> bool {
        let Some(count) = self.solid(solid).map(|s| entity_count(s, kind)) else {
            return false;
        };
        // The solid exists (count resolved above), so `solid_mut` is Some.
        if let Some(s) = self.solid_mut(solid) {
            s.visibility.set_group_visible(kind, count, visible);
        }
        true
    }

    /// Whether entity `index` of `kind` on `solid` is shown (`None` if unknown).
    pub fn entity_visible(&self, solid: &str, kind: EntityKind, index: usize) -> Option<bool> {
        self.solid(solid).map(|s| s.visibility.is_visible(kind, index))
    }

    /// The group tristate for `solid`'s `kind` (`None` if the solid is unknown).
    pub fn group_visibility(&self, solid: &str, kind: EntityKind) -> Option<GroupState> {
        self.solid(solid)
            .map(|s| s.visibility.group_state(kind, entity_count(s, kind)))
    }
}

// --- EngineState: the UI-facing visibility API (marks dirty; the Scene panel
//     drives these and reads the queries back) ---------------------------------
impl EngineState {
    /// Show/hide ONE face/edge/vertex of a solid, by its index in the solid's
    /// face/edge/vertex list — the same index the Scene tree enumerates. Marks
    /// dirty. Returns false if the solid is unknown.
    pub fn set_entity_visible(
        &mut self,
        solid: &str,
        kind: EntityKind,
        index: usize,
        visible: bool,
    ) -> bool {
        let ok = self.scene.set_entity_visible(solid, kind, index, visible);
        if ok {
            self.dirty = true;
        }
        ok
    }

    /// Show/hide a WHOLE group (all faces / all edges / all vertices of a solid).
    /// Marks dirty. Returns false if the solid is unknown.
    pub fn set_group_visible(&mut self, solid: &str, kind: EntityKind, visible: bool) -> bool {
        let ok = self.scene.set_group_visible(solid, kind, visible);
        if ok {
            self.dirty = true;
        }
        ok
    }

    /// Whether entity `index` of `kind` on `solid` is shown (`None` if unknown).
    pub fn entity_visible(&self, solid: &str, kind: EntityKind, index: usize) -> Option<bool> {
        self.scene.entity_visible(solid, kind, index)
    }

    /// The group tristate for `solid`'s `kind` (`None` if the solid is unknown).
    pub fn group_visibility(&self, solid: &str, kind: EntityKind) -> Option<GroupState> {
        self.scene.group_visibility(solid, kind)
    }

    /// Per-entity + group visibility as JSON — the readout the headed verifier
    /// asserts against (a companion to
    /// [`scene_entities_json`](EngineState::scene_entities_json), which lists the
    /// entities but not their per-entity visibility):
    /// `[{name, visible, faces:{group, states:[bool;n]}, edges:{…}, vertices:{…}}]`
    /// where `states[i]` is entity `i`'s visibility and `group` is
    /// `"all"|"partial"|"none"`.
    pub fn scene_visibility_json(&self) -> String {
        let kinds = [
            ("faces", EntityKind::Face),
            ("edges", EntityKind::Edge),
            ("vertices", EntityKind::Vertex),
        ];
        let solids: Vec<serde_json::Value> = self
            .scene
            .solids()
            .iter()
            .map(|solid| {
                let mut obj = serde_json::Map::new();
                obj.insert("name".into(), serde_json::json!(solid.name));
                obj.insert("visible".into(), serde_json::json!(solid.visible));
                for (key, kind) in kinds {
                    let count = entity_count(solid, kind);
                    let states: Vec<bool> =
                        (0..count).map(|i| solid.visibility.is_visible(kind, i)).collect();
                    obj.insert(
                        key.into(),
                        serde_json::json!({
                            "group": solid.visibility.group_state(kind, count).as_str(),
                            "states": states,
                        }),
                    );
                }
                serde_json::Value::Object(obj)
            })
            .collect();
        serde_json::Value::Array(solids).to_string()
    }
}

