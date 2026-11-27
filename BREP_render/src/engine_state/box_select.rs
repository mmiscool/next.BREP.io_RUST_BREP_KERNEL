//! BOX selection in the 3D view (Shift+left-drag): [`crate::box_pick`]'s
//! visible entities, through the bottom selection filter, into the selection.

use super::*;
use crate::box_pick::{box_pick, ScreenBox};

/// What a box drag does to the selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoxSelectMode {
    /// Shift+drag: the box's entities become the selection.
    Replace,
    /// Primary+Shift+drag: they are added.
    Add,
    /// Alt/Option+Shift+drag: they are removed.
    Remove,
}

impl EngineState {
    /// The candidates a box from `start` to `end` (viewport CSS px) takes,
    /// filter applied: WINDOW (left to right, fully inside) or CROSSING
    /// (right to left, touching), visible entities only. With the filter's
    /// COMPONENT lane on, an entity of an assembly component is taken as its
    /// component (window: every member solid inside).
    pub fn box_candidates(&self, start: (f64, f64), end: (f64, f64)) -> Vec<pick::PickCandidate> {
        let rect = ScreenBox::from_drag(start, end);
        let kinds = self.selection_filter.enabled_kinds();
        let component_on = self.selection_filter.component;
        if kinds.is_empty() && !component_on {
            return Vec::new();
        }
        let raw = box_pick(&self.scene, &self.camera, rect, &self.pick_options());
        let mut out: Vec<pick::PickCandidate> = Vec::new();
        for candidate in &raw {
            if component_on && self.hit_owning_component(candidate).is_some() {
                continue;
            }
            if self.candidate_admitted(&kinds, candidate) {
                out.push(candidate.clone());
            }
        }
        if component_on {
            let components = if rect.crossing() {
                self.component_candidates(&raw)
            } else {
                let taken: std::collections::HashSet<&str> = raw
                    .iter()
                    .filter(|c| c.kind == pick::PickKind::Solid)
                    .map(|c| c.name.as_str())
                    .collect();
                self.component_candidates(&raw)
                    .into_iter()
                    .filter(|component| {
                        let members = self.component_member_solids(&component.name);
                        !members.is_empty() && members.iter().all(|m| taken.contains(m.as_str()))
                    })
                    .collect()
            };
            out.extend(components);
        }
        out
    }

    /// Apply a box drag to the selection (see [`BoxSelectMode`]). Returns how
    /// many entities the box took.
    pub fn box_select(&mut self, start: (f64, f64), end: (f64, f64), mode: BoxSelectMode) -> usize {
        let candidates = self.box_candidates(start, end);
        if mode == BoxSelectMode::Replace {
            self.clear_selection();
        }
        for candidate in &candidates {
            let selected = self.candidate_is_selected(candidate);
            let wanted = mode != BoxSelectMode::Remove;
            if selected != wanted {
                self.toggle_candidate(candidate);
            }
        }
        self.dirty = true;
        candidates.len()
    }
}

/// The kind select-all / invert act on: the coarsest kind the filter admits
/// that has a visible entity, COMPONENT > SOLID > FACE > EDGE > VERTEX.
const SELECT_ALL_ORDER: [pick::PickKind; 5] = [
    pick::PickKind::Component,
    pick::PickKind::Solid,
    pick::PickKind::Face,
    pick::PickKind::Edge,
    pick::PickKind::Vertex,
];

impl EngineState {
    /// Every VISIBLE entity of the current filter type (primary+A): the
    /// coarsest kind the selection filter — or a held letter — admits that
    /// has anything on screen (components, else solids, faces, edges,
    /// vertices). `invert` (primary+I) flips the selection within that kind
    /// instead: the visible ones not selected become selected and the
    /// selected ones are dropped. Replaces the selection of every other kind
    /// either way. Returns how many entities are selected of that kind after.
    pub fn select_all_visible(&mut self, invert: bool) -> usize {
        let (w, h) = (self.camera.width, self.camera.height);
        let all = self.box_candidates((w, 0.0), (0.0, h));
        let Some(kind) = SELECT_ALL_ORDER.iter().copied().find(|k| all.iter().any(|c| c.kind == *k)) else {
            if !invert {
                self.clear_selection();
            }
            return 0;
        };
        let of_kind: Vec<pick::PickCandidate> = all.into_iter().filter(|c| c.kind == kind).collect();
        let selected: Vec<bool> = of_kind.iter().map(|c| self.candidate_is_selected(c)).collect();
        self.clear_selection();
        let mut count = 0;
        for (candidate, was) in of_kind.iter().zip(selected) {
            if !invert || !was {
                if !self.candidate_is_selected(candidate) {
                    self.toggle_candidate(candidate);
                }
                count += 1;
            }
        }
        self.dirty = true;
        let noun = match kind {
            pick::PickKind::Component => "component",
            pick::PickKind::Solid => "solid",
            pick::PickKind::Face => "face",
            pick::PickKind::Edge => "edge",
            _ => "vertex",
        };
        let plural = match (count, kind) {
            (1, _) => noun.to_string(),
            (_, pick::PickKind::Vertex) => "vertices".to_string(),
            _ => format!("{noun}s"),
        };
        // Say what it took (master 2026-11-26): the status readout.
        self.push_notice_as(NoticeSeverity::Info, format!("Selected {count} {plural}"));
        count
    }
}

impl EngineState {
    /// Primary+A while a MULTIPLE reference field is picking: every visible
    /// entity of the coarsest kind the field admits goes into its running
    /// list (de-duplicated, named as a click would name it). Returns how many
    /// it offered; 0 for a single field or none picking.
    pub fn ref_select_all_visible(&mut self) -> usize {
        if !self.ref_select.as_ref().is_some_and(|state| state.multiple) {
            return 0;
        }
        let (w, h) = (self.camera.width, self.camera.height);
        let all = self.box_candidates((w, 0.0), (0.0, h));
        let Some(kind) = SELECT_ALL_ORDER.iter().copied().find(|k| all.iter().any(|c| c.kind == *k)) else {
            return 0;
        };
        let of_kind: Vec<pick::PickCandidate> = all.into_iter().filter(|c| c.kind == kind).collect();
        for candidate in &of_kind {
            self.ref_select_take_candidate(candidate);
        }
        self.dirty = true;
        of_kind.len()
    }
}

impl EngineState {
    /// The LIVE box preview (owner 2026-11-26): while a box is dragged the
    /// pointer's own hover highlight is off and the hover emphasis lights
    /// exactly what releasing the box here would take — the same
    /// [`Self::box_candidates`] the release applies. Returns how many.
    pub fn hover_box_preview(&mut self, start: (f64, f64), end: (f64, f64)) -> usize {
        use crate::pick::PickKind;
        let candidates = self.box_candidates(start, end);
        let had_datums = !self.emphasis.hovered_datums.is_empty();
        self.emphasis.hovered_solids.clear();
        self.emphasis.hovered_faces.clear();
        self.emphasis.hovered_edges.clear();
        self.emphasis.hovered_vertices.clear();
        self.emphasis.hovered_datums.clear();
        for candidate in &candidates {
            match candidate.kind {
                PickKind::Solid => {
                    self.emphasis.hovered_solids.insert(self.candidate_solid_name(candidate));
                }
                PickKind::Face => {
                    self.emphasis.hovered_faces.insert(candidate.name.clone());
                }
                PickKind::Edge => {
                    self.emphasis.hovered_edges.insert(candidate.name.clone());
                }
                PickKind::Vertex => self.emphasis.hovered_vertices.push(crate::style::VertexRef {
                    solid: candidate.solid.clone(),
                    position: candidate.position,
                }),
                PickKind::Plane => {
                    self.emphasis.hovered_datums.insert(candidate.name.clone());
                }
                PickKind::Component => {
                    for member in self.component_member_solids(&candidate.name) {
                        self.emphasis.hovered_solids.insert(member);
                    }
                }
            }
        }
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.dirty = true;
        if had_datums {
            self.refresh_construction_datums();
        }
        candidates.len()
    }
}
