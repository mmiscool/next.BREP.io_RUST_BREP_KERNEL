//! Scene panel — the engine-native **Scene tree** ("Scene Manager"), the second
//! sidebar tree in the design reference. Built on the SAME reusable [`tree`] node
//! helper (connector lines + `[+]`/`[-]` collapse boxes) the history panel uses,
//! so the two trees read as one system.
//!
//! # What it draws
//!
//! Above the tree sits a **type-visibility button row** (`Faces` / `Edges` /
//! `Vertices`, plus `Sketches` / `Planes` when the scene has them): each button
//! hides/shows EVERY object of that type SCENE-WIDE in one click — the missing
//! bulk complement to the per-solid group tristate below. The toggle follows the
//! same rule as the group checkbox + selection-filter toggle-all: if EVERY object
//! of that type is currently visible → hide them all; otherwise (some or all
//! hidden) → show them all. There is deliberately no `Solids` button — the Scene
//! ROOT checkbox already toggles every solid.
//!
//! A file-tree of the engine's display scene:
//!   * `[-] Scene <☑>` — the root; its checkbox toggles ALL solids' visibility.
//!   * per solid `[-] <Name> <☑>` — the checkbox toggles that solid's visibility
//!     through the engine ([`EngineState::set_visible`]); expands to
//!   * `[+] Faces <☑>`, `[+] Edges <☑>`, `[+] Vertices <☑>` — each expands to the
//!     individual entities BY KERNEL NAME (vertices by index/position).
//!
//! # Selection sync (both ways)
//!
//! Clicking an entity row drives the engine's name-based SELECTION
//! ([`EngineState::select_by_name`] / [`select_vertex_by_position`]) so it
//! highlights (emphasis) in the viewport; and the engine's CURRENT selection
//! (`state.emphasis`) bolds the matching tree row — the same emphasis the
//! viewport reads, so a viewport pick lights up the tree and vice-versa.
//!
//! # Hover sync (row → viewport)
//!
//! MOUSING OVER a row highlights that entity in the 3D view exactly as mousing
//! over it in the viewport does: the row feeds the SAME `emphasis` hover buckets
//! the viewport's `hover_at` fills, through the name-based twins
//! ([`EngineState::hover_by_name`] / [`hover_vertex_by_position`]) — so there is
//! one hover highlight, one style, one clear. It lights exactly the rows whose
//! CLICK selects one 3D entity (solids, committed sketches, datums/planes, and
//! the face/edge/vertex leaves); the Scene root and the Faces/Edges/Vertices
//! GROUP nodes are not things in the 3D view, so they stay unlit. The pointer is
//! off the viewport while it is over a row, so the engine's one-frame
//! [`EngineState::take_scene_tree_hover`] flag makes the viewport's hover pass
//! yield instead of clearing the row's highlight.
//!
//! # This panel OWNS NO model state
//!
//! The scene + selection live in the engine ([`EngineState`], the single source
//! of truth). The panel holds only transient UI state: which nodes are expanded
//! and the per-frame `hits` map (widget screen rects) the headed verifier reads.
//! Each frame it snapshots the scene into owned rows FIRST, draws from that, and
//! applies at most one deferred engine mutation after the draw loop (so no borrow
//! of `state` is held across a `&mut` call — the history panel's pattern).
//!
//! # Two deliberate reshapes (functional-over-1:1, per the design doc)
//!
//! * The visibility checkbox is drawn in the row's RIGHT slot: the shared tree
//!   widget reserves the left columns for the collapse box + connector + glyph,
//!   and (per the constraints) it is reused verbatim, not modified. Functionally
//!   identical to the reference's left checkbox.
//! * **Per-entity visibility (live):** the engine now hides individual faces /
//!   edges / vertices and whole groups ([`EngineState::set_entity_visible`] /
//!   [`EngineState::set_group_visible`]). So each Faces/Edges/Vertices group
//!   checkbox is a live TRISTATE (all / some / none of that kind shown) and each
//!   entity leaf carries its own live checkbox — the render pass skips a hidden
//!   entity's triangles / segments / point. The whole-solid + whole-scene
//!   checkboxes still compose: a hidden solid draws nothing; re-showing it keeps
//!   any per-entity hides intact.

use crate::automation::hit_keys::HitKeyDoc;
use crate::panels::toolbar_button;
use crate::panels::tree::{self, TreeRow};
use brep_render::engine_state::EngineState;
use brep_render::style::OrderedNames;
use brep_render::visibility::{EntityKind as VisKind, GroupState};
use eframe::egui;
use std::collections::{HashMap, HashSet};

/// One entity leaf's identity — how a click maps to an engine selection call.
#[derive(Clone)]
enum EntityKind {
    /// Face, selected by kernel name (empty = unnamed → not selectable).
    Face(String),
    /// Edge, selected by kernel name (empty = unnamed → not selectable).
    Edge(String),
    /// Vertex, selected by owning-solid + world position (no kernel name).
    Vertex([f64; 3]),
}

/// One row under a Faces/Edges/Vertices group — a display label, its selection
/// identity, whether it is currently in the engine selection, and whether it is
/// currently VISIBLE in the engine (its live per-entity checkbox state).
#[derive(Clone)]
struct Entity {
    label: String,
    kind: EntityKind,
    selected: bool,
    visible: bool,
}

/// One solid's owned snapshot for the frame (decoupled from `state.scene` so the
/// draw loop can issue deferred `&mut state` mutations afterwards).
struct SolidRow {
    name: String,
    visible: bool,
    selected: bool,
    faces: Vec<Entity>,
    edges: Vec<Entity>,
    vertices: Vec<Entity>,
}

/// A deferred engine mutation, collected during the draw and applied once after
/// the loop (one per frame — the history panel's pattern).
enum Action {
    SetVisible(String, bool),
    SetAllVisible(bool),
    /// Hide/show one entity: `(solid, kind, index-in-kind-list, visible)`.
    SetEntityVisible(String, VisKind, usize, bool),
    /// Hide/show a whole group: `(solid, kind, visible)`.
    SetGroupVisible(String, VisKind, bool),
    /// Hide/show one group KIND across EVERY solid in the scene: `(kind, visible)`.
    SetAllGroupVisible(VisKind, bool),
    /// Hide/show EVERY committed sketch's overlay: `(visible)`.
    SetAllSketchVisible(bool),
    /// Hide/show EVERY construction datum/plane: `(visible)`.
    SetAllDatumVisible(bool),
    Select(&'static str, String),
    SelectVertex(String, [f64; 3]),
    /// Show/hide a committed sketch's persistent overlay: `(feature-id, visible)`.
    SetSketchVisible(String, bool),
    /// Show/hide a construction datum/plane's plane: `(frame-name, visible)`.
    SetDatumVisible(String, bool),
    /// Select a construction datum/plane by frame NAME (a row click).
    SelectDatum(String),
}

/// The row the pointer is over this frame — the hover twin of [`Action`]'s
/// select arms, and deliberately the SAME targets: a row hover previews exactly
/// what that row's click would select. Collected during the draw, applied once
/// after it (like [`Action`], so no borrow of `state` is held across the loop).
enum Hover {
    /// Solid / committed sketch / face / edge / datum — by kernel (or frame) NAME.
    Named(&'static str, String),
    /// A vertex: owning solid + world position (vertices carry no kernel name).
    Vertex(String, [f64; 3]),
}

/// The Scene tree panel's transient UI state (the scene + selection live in the
/// engine).
#[derive(Default)]
pub struct ScenePanel {
    /// Per-frame egui widget screen rects, published to JS for the headed
    /// verifier. Rebuilt every frame.
    hits: HashMap<String, egui::Rect>,
    /// The Scene ROOT is collapsed (absent/false = open — it defaults open).
    root_collapsed: bool,
    /// Solids explicitly COLLAPSED, by name (absent = open — solids default open,
    /// matching the reference showing a solid's Faces/Edges/Vertices).
    collapsed_solids: HashSet<String>,
    /// Faces/Edges/Vertices group nodes explicitly EXPANDED, keyed
    /// `"<solid>/<group>"` (absent = collapsed — groups default collapsed `[+]`).
    expanded_groups: HashSet<String>,
    /// The `Planes & Datums` group node is collapsed (absent/false = open — it
    /// defaults open so construction datums are visible in the tree).
    datums_collapsed: bool,
}

impl ScenePanel {
    pub fn new() -> Self {
        Self::default()
    }

    /// Draw the Scene tree. Snapshots the scene + current selection into owned
    /// rows, draws them via the shared [`tree`] node helper, then applies at most
    /// one deferred engine mutation.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        self.hits.clear();

        // --- snapshot scene + selection (owned) so we can mutate after drawing --
        let sel_solids = state.emphasis.selected_solids.clone();
        let sel_faces = state.emphasis.selected_faces.clone();
        let sel_edges = state.emphasis.selected_edges.clone();
        let sel_vertices = state.emphasis.selected_vertices.clone();
        let solids = snapshot(state, &sel_solids, &sel_faces, &sel_edges, &sel_vertices);
        // Committed sketches (id, visible) — listed under the solids in the tree.
        let sketches = state.committed_sketches();
        // Construction datums/planes (name, visible) + which are selected — listed as
        // the tree's LAST group.
        let sel_datums = state.emphasis.selected_datums.clone();
        let datums = state.construction_datums();

        // Tight, tree-like row spacing so connector verticals read continuously.
        ui.spacing_mut().item_spacing.y = 2.0;

        let mut action: Option<Action> = None;
        // The row under the pointer (at most one — rows don't overlap).
        let mut hover: Option<Hover> = None;

        // --- TYPE-VISIBILITY button row (scene-wide, ABOVE the tree) -----------
        // One button per display TYPE; each hides/shows EVERY object of that type
        // across the whole scene. Same toggle rule as the per-solid group tristate
        // + the selection-filter toggle-all: all-visible → hide all; otherwise
        // (some or all hidden) → show all. Buttons show a "pressed" (selected) look
        // when all of that type are currently visible. There is no `Solids` button
        // — the Scene root checkbox already toggles every solid.
        ui.horizontal(|ui| {
            let has_solids = !solids.is_empty();

            // Faces / Edges / Vertices — scene-wide across ALL solids. All-visible
            // means every solid reports GroupState::All for that kind.
            let groups: [(VisKind, &str, bool); 3] = [
                (
                    VisKind::Face,
                    "Faces",
                    has_solids
                        && solids
                            .iter()
                            .all(|s| matches!(group_state(&s.faces), GroupState::All)),
                ),
                (
                    VisKind::Edge,
                    "Edges",
                    has_solids
                        && solids
                            .iter()
                            .all(|s| matches!(group_state(&s.edges), GroupState::All)),
                ),
                (
                    VisKind::Vertex,
                    "Vertices",
                    has_solids
                        && solids
                            .iter()
                            .all(|s| matches!(group_state(&s.vertices), GroupState::All)),
                ),
            ];
            for (kind, label, all_visible) in groups {
                let tip = if all_visible {
                    format!("Hide all {}", label.to_lowercase())
                } else {
                    format!("Show all {}", label.to_lowercase())
                };
                // Disabled (greyed + non-interactive) with no solids, matching the
                // Scene root checkbox's `add_enabled(!solids.is_empty(), …)` spirit.
                let resp = ui
                    .add_enabled_ui(has_solids, |ui| {
                        toolbar_button::toggle(ui, all_visible, label, &tip)
                    })
                    .inner;
                self.hits.insert(format!("typevis:{label}"), resp.rect);
                if resp.clicked() {
                    action = Some(Action::SetAllGroupVisible(kind, !all_visible));
                }
            }

            // Sketches — shown only when the scene has committed sketches (mirrors
            // `render_sketches` being conditional). All-visible = every sketch shown.
            if !sketches.is_empty() {
                let all_visible = sketches.iter().all(|(_, v)| *v);
                let tip = if all_visible {
                    "Hide all sketches"
                } else {
                    "Show all sketches"
                };
                let resp = toolbar_button::toggle(ui, all_visible, "Sketches", tip);
                self.hits.insert("typevis:Sketches".into(), resp.rect);
                if resp.clicked() {
                    action = Some(Action::SetAllSketchVisible(!all_visible));
                }
            }

            // Planes & Datums — shown only when the scene has construction datums
            // (mirrors `render_datums` being conditional). Hit key is the literal
            // `typevis:Datums` the verifier drives; the button LABEL reads "Planes".
            if !datums.is_empty() {
                let all_visible = datums.iter().all(|(_, v)| *v);
                let tip = if all_visible {
                    "Hide all planes & datums"
                } else {
                    "Show all planes & datums"
                };
                let resp = toolbar_button::toggle(ui, all_visible, "Planes", tip);
                self.hits.insert("typevis:Datums".into(), resp.rect);
                if resp.clicked() {
                    action = Some(Action::SetAllDatumVisible(!all_visible));
                }
            }
        });

        // --- ROOT: `[-] Scene  ☑` (visibility toggles every solid) ------------
        let root_open = !self.root_collapsed;
        let all_visible = !solids.is_empty() && solids.iter().all(|s| s.visible);
        let mut root_vis = all_visible;
        let mut root_vis_rect = egui::Rect::NOTHING;
        let mut root_vis_clicked = false;
        let root_resp = tree::node(
            ui,
            TreeRow {
                guides: &[],
                is_last: true,
                expandable: true,
                expanded: root_open,
                root: true,
                glyph: None,
                label: "Scene",
                selected: false,
                highlighted: false,
                draggable: false,
                tint: None,
            },
            |ui| {
                let cb = ui.add_enabled(!solids.is_empty(), egui::Checkbox::new(&mut root_vis, ""));
                root_vis_rect = cb.rect;
                root_vis_clicked = cb.clicked();
            },
        );
        self.hits.insert("box:__scene".into(), root_resp.box_rect);
        self.hits.insert("vis:__scene".into(), root_vis_rect);
        if root_vis_clicked {
            action = Some(Action::SetAllVisible(root_vis));
        }
        if root_resp.toggled || root_resp.clicked() {
            self.root_collapsed = !self.root_collapsed;
        }

        if solids.is_empty() && sketches.is_empty() && datums.is_empty() {
            let g = tree::child_guides(&[], true);
            tree::node(ui, TreeRow::leaf(&g, true, "(scene is empty)"), |_| {});
        }

        if root_open {
            // Top-level children under the root, in order: solids, then each committed
            // sketch as its OWN top-level row (no group wrapper — a committed sketch is
            // a scene solid, listed like the solids), then the `Planes & Datums` group
            // (the last child), so nothing "below" is marked last while a later child
            // still follows.
            let has_sketches = !sketches.is_empty();
            let has_datums = !datums.is_empty();
            let n = solids.len();
            for (si, solid) in solids.iter().enumerate() {
                let is_last = !has_sketches && !has_datums && si + 1 == n;
                self.render_solid(ui, solid, is_last, &mut action, &mut hover);
            }
            let m = sketches.len();
            for (i, (id, visible)) in sketches.iter().enumerate() {
                let is_last = !has_datums && i + 1 == m;
                let selected = sel_solids.contains(id);
                self.render_sketch_row(ui, id, *visible, selected, is_last, &mut action, &mut hover);
            }
            if has_datums {
                self.render_datums(ui, &datums, &sel_datums, &mut action, &mut hover);
            }
        }

        // --- apply the one deferred engine mutation ---------------------------
        match action {
            Some(Action::SetVisible(name, v)) => {
                state.set_visible(&name, v);
            }
            Some(Action::SetAllVisible(v)) => {
                for s in &solids {
                    state.set_visible(&s.name, v);
                }
            }
            Some(Action::SetEntityVisible(name, kind, index, v)) => {
                state.set_entity_visible(&name, kind, index, v);
            }
            Some(Action::SetGroupVisible(name, kind, v)) => {
                state.set_group_visible(&name, kind, v);
            }
            Some(Action::SetAllGroupVisible(kind, v)) => {
                for s in &solids {
                    state.set_group_visible(&s.name, kind, v);
                }
            }
            Some(Action::SetAllSketchVisible(v)) => {
                for (id, _) in &sketches {
                    state.set_sketch_visible(id, v);
                }
            }
            Some(Action::SetAllDatumVisible(v)) => {
                for (name, _) in &datums {
                    state.set_datum_visible(name, v);
                }
            }
            Some(Action::Select(kind, name)) => {
                state.select_by_name(kind, &name);
            }
            Some(Action::SelectVertex(solid, pos)) => {
                state.select_vertex_by_position(&solid, pos);
            }
            Some(Action::SetSketchVisible(id, v)) => {
                state.set_sketch_visible(&id, v);
            }
            Some(Action::SetDatumVisible(name, v)) => {
                state.set_datum_visible(&name, v);
            }
            Some(Action::SelectDatum(name)) => {
                state.select_datum(&name);
            }
            None => {}
        }

        // --- apply the row HOVER (row → viewport highlight) --------------------
        // Independent of the click above (hovering and clicking legitimately land
        // on the same frame) and applied EVERY frame: the engine dedupes a held
        // hover and re-arms the one-frame flag the viewport's hover pass yields to,
        // and `scene_tree_hover_end` is a no-op once it has cleared what the tree
        // lit (it never touches a hover the viewport set).
        let hover_changed = match &hover {
            Some(Hover::Named(kind, name)) => state.hover_by_name(kind, name),
            Some(Hover::Vertex(solid, position)) => {
                state.hover_vertex_by_position(solid, *position)
            }
            None => state.scene_tree_hover_end(),
        };
        if hover_changed {
            // The viewport tile may have drawn (and consumed `state.dirty`) BEFORE
            // this pane in the dock, so without this the new highlight would wait
            // for the next pointer event to reach the screen.
            ui.ctx().request_repaint();
        }

        // --- verifier hooks (wasm only): scene listing + widget hit-rects ------
        // Published from the panel (not the shared shell) so the headed verifier
        // can assert the tree contents / visibility and drive real clicks, without
        // touching `app.rs`'s shared publish block.
        if crate::automation::registry::enabled() {
            crate::automation::registry::publish("__brepScene", "scene tree solids with faces, edges, vertices", &state.scene_entities_json());
            crate::automation::registry::publish("__brepSketches", "committed sketches", &state.sketch_entities_json());
            crate::automation::registry::publish("__brepDatums", "datum entities", &state.datum_entities_json());
            crate::automation::registry::publish("__brepSceneVis", "scene visibility map", &state.scene_visibility_json());
            self.publish_hits();
        }
    }

    /// One committed sketch as a TOP-LEVEL leaf row (no group wrapper): a
    /// visibility checkbox wired to
    /// [`EngineState::set_sketch_visible`](brep_render::engine_state::EngineState::set_sketch_visible)
    /// and a label that SELECTS the sketch's sheet solid on click (a committed
    /// sketch is a scene solid, dim-cyan / `is_sketch`-styled in the viewport, kept
    /// OUT of the plain-solid rows by the [`snapshot`] filter so it lists exactly
    /// once). Rendered under the solids, before the `Planes & Datums` group;
    /// `is_last` is set only when it is the final top-level child.
    fn render_sketch_row(
        &mut self,
        ui: &mut egui::Ui,
        id: &str,
        visible: bool,
        selected: bool,
        is_last: bool,
        action: &mut Option<Action>,
        hover: &mut Option<Hover>,
    ) {
        let mut vis = visible;
        let mut vis_rect = egui::Rect::NOTHING;
        let mut vis_clicked = false;
        let resp = tree::node(
            ui,
            TreeRow::leaf(&[], is_last, id).selected(selected),
            |ui| {
                let cb = ui.add(egui::Checkbox::new(&mut vis, ""));
                vis_rect = cb.rect;
                vis_clicked = cb.clicked();
            },
        );
        self.hits.insert(format!("vis:sketch/{id}"), vis_rect);
        self.hits.insert(format!("sel:sketch/{id}"), resp.label.rect);
        if vis_clicked {
            *action = Some(Action::SetSketchVisible(id.to_string(), vis));
        } else if resp.clicked() {
            // A committed sketch is a scene solid — select it like any solid.
            *action = Some(Action::Select("solid", id.to_string()));
        }
        if resp.label.hovered() {
            *hover = Some(Hover::Named("solid", id.to_string()));
        }
    }

    /// The `Planes & Datums` group node + (when open) one row per construction
    /// datum/plane frame, each with a visibility checkbox wired to
    /// [`EngineState::set_datum_visible`](brep_render::engine_state::EngineState::set_datum_visible)
    /// and a label that selects the datum
    /// ([`EngineState::select_datum`](brep_render::engine_state::EngineState::select_datum))
    /// on click. Rendered as the Scene root's LAST child (only when there is at
    /// least one construction datum).
    fn render_datums(
        &mut self,
        ui: &mut egui::Ui,
        datums: &[(String, bool)],
        sel_datums: &OrderedNames,
        action: &mut Option<Action>,
        hover: &mut Option<Hover>,
    ) {
        let open = !self.datums_collapsed;
        let resp = tree::node(
            ui,
            TreeRow::branch(&[], true, open, "Planes & Datums"),
            |ui| {
                ui.add_space(6.0);
                ui.label(egui::RichText::new(format!("{}", datums.len())).weak());
            },
        );
        self.hits.insert("box:__datums".into(), resp.box_rect);
        if resp.toggled || resp.clicked() {
            self.datums_collapsed = !self.datums_collapsed;
        }
        if !open {
            return;
        }

        let base = tree::child_guides(&[], true);
        let m = datums.len();
        for (i, (name, visible)) in datums.iter().enumerate() {
            let last = i + 1 == m;
            let selected = sel_datums.contains(name);
            let mut vis = *visible;
            let mut vis_rect = egui::Rect::NOTHING;
            let mut vis_clicked = false;
            let resp = tree::node(
                ui,
                TreeRow::leaf(&base, last, name).selected(selected),
                |ui| {
                    let cb = ui.add(egui::Checkbox::new(&mut vis, ""));
                    vis_rect = cb.rect;
                    vis_clicked = cb.clicked();
                },
            );
            self.hits.insert(format!("vis:datum/{name}"), vis_rect);
            self.hits.insert(format!("sel:datum/{name}"), resp.label.rect);
            if vis_clicked {
                *action = Some(Action::SetDatumVisible(name.clone(), vis));
            } else if resp.clicked() {
                *action = Some(Action::SelectDatum(name.clone()));
            }
            if resp.label.hovered() {
                *hover = Some(Hover::Named("datum", name.clone()));
            }
        }
    }

    /// One solid node + (when open) its Faces / Edges / Vertices groups.
    fn render_solid(
        &mut self,
        ui: &mut egui::Ui,
        solid: &SolidRow,
        is_last: bool,
        action: &mut Option<Action>,
        hover: &mut Option<Hover>,
    ) {
        let name = &solid.name;
        let open = !self.collapsed_solids.contains(name);

        let mut vis = solid.visible;
        let mut vis_rect = egui::Rect::NOTHING;
        let mut vis_clicked = false;
        let resp = tree::node(
            ui,
            TreeRow::branch(&[], is_last, open, name).selected(solid.selected),
            |ui| {
                let cb = ui.add(egui::Checkbox::new(&mut vis, ""));
                vis_rect = cb.rect;
                vis_clicked = cb.clicked();
            },
        );
        self.hits.insert(format!("box:{name}"), resp.box_rect);
        self.hits.insert(format!("vis:{name}"), vis_rect);
        self.hits.insert(format!("sel:{name}"), resp.label.rect);

        if vis_clicked {
            *action = Some(Action::SetVisible(name.clone(), vis));
        }
        if resp.toggled {
            if open {
                self.collapsed_solids.insert(name.clone());
            } else {
                self.collapsed_solids.remove(name);
            }
        }
        if resp.clicked() {
            *action = Some(Action::Select("solid", name.clone()));
        }
        if resp.label.hovered() {
            *hover = Some(Hover::Named("solid", name.clone()));
        }

        if open {
            let base = tree::child_guides(&[], is_last);
            self.render_group(ui, name, &base, false, "Faces", VisKind::Face, &solid.faces, action, hover);
            self.render_group(ui, name, &base, false, "Edges", VisKind::Edge, &solid.edges, action, hover);
            self.render_group(ui, name, &base, true, "Vertices", VisKind::Vertex, &solid.vertices, action, hover);
        }
    }

    /// One Faces/Edges/Vertices group node + (when open) its entity leaves. The
    /// group checkbox is a live TRISTATE that hides/shows every entity of `kind`;
    /// each leaf carries its own live checkbox that hides just that entity.
    #[allow(clippy::too_many_arguments)]
    fn render_group(
        &mut self,
        ui: &mut egui::Ui,
        solid_name: &str,
        base: &[bool],
        is_last: bool,
        group: &str,
        kind: VisKind,
        entities: &[Entity],
        action: &mut Option<Action>,
        hover: &mut Option<Hover>,
    ) {
        let key = format!("{solid_name}/{group}");
        let open = self.expanded_groups.contains(&key);

        // Tristate over the group's entities (empty group reads All → checked).
        let state = group_state(entities);
        let mut checked = matches!(state, GroupState::All);
        let indeterminate = matches!(state, GroupState::Partial);
        let mut vis_rect = egui::Rect::NOTHING;
        let mut vis_clicked = false;
        let resp = tree::node(
            ui,
            TreeRow::branch(base, is_last, open, group),
            |ui| {
                // right-to-left: the tristate group checkbox (rightmost), then count.
                let cb = ui.add(
                    egui::Checkbox::new(&mut checked, "").indeterminate(indeterminate),
                );
                vis_rect = cb.rect;
                vis_clicked = cb.clicked();
                ui.add_space(6.0);
                ui.label(egui::RichText::new(format!("{}", entities.len())).weak());
            },
        );
        self.hits.insert(format!("box:{key}"), resp.box_rect);
        self.hits.insert(format!("vis:{key}"), vis_rect);
        if vis_clicked {
            // Standard tristate: All → hide all; None/Partial → show all.
            let want_visible = !matches!(state, GroupState::All);
            *action = Some(Action::SetGroupVisible(solid_name.to_string(), kind, want_visible));
        }
        if resp.toggled || resp.clicked() {
            if open {
                self.expanded_groups.remove(&key);
            } else {
                self.expanded_groups.insert(key.clone());
            }
        }

        if !open {
            return;
        }
        let gg = tree::child_guides(base, is_last);
        if entities.is_empty() {
            tree::node(ui, TreeRow::leaf(&gg, true, "(none)"), |_| {});
            return;
        }
        let m = entities.len();
        for (ei, e) in entities.iter().enumerate() {
            let last = ei + 1 == m;
            let mut vis = e.visible;
            let mut ev_rect = egui::Rect::NOTHING;
            let mut ev_clicked = false;
            let resp = tree::node(
                ui,
                TreeRow::leaf(&gg, last, &e.label).selected(e.selected),
                |ui| {
                    let cb = ui.add(egui::Checkbox::new(&mut vis, ""));
                    ev_rect = cb.rect;
                    ev_clicked = cb.clicked();
                },
            );
            self.hits.insert(format!("sel:{key}/{ei}"), resp.label.rect);
            self.hits.insert(format!("vis:{key}/{ei}"), ev_rect);
            if ev_clicked {
                *action = Some(Action::SetEntityVisible(solid_name.to_string(), kind, ei, vis));
            } else if resp.clicked() {
                *action = Some(match &e.kind {
                    EntityKind::Face(n) => Action::Select("face", n.clone()),
                    EntityKind::Edge(n) => Action::Select("edge", n.clone()),
                    EntityKind::Vertex(p) => Action::SelectVertex(solid_name.to_string(), *p),
                });
            }
            if resp.label.hovered() {
                // An UNNAMED face/edge has nothing to highlight (it is not
                // selectable either) — leaving `hover` unset clears instead, so a
                // neighbouring row's highlight never lingers under the pointer.
                *hover = match &e.kind {
                    EntityKind::Face(n) if !n.is_empty() => {
                        Some(Hover::Named("face", n.clone()))
                    }
                    EntityKind::Edge(n) if !n.is_empty() => {
                        Some(Hover::Named("edge", n.clone()))
                    }
                    EntityKind::Vertex(p) => {
                        Some(Hover::Vertex(solid_name.to_string(), *p))
                    }
                    _ => None,
                };
            }
        }
    }

    /// The published widget hit-rects (egui points) for the headed verifier.
    pub fn hits_json(&self) -> String {
        crate::automation::hit_rects::hits_json(&self.hits)
    }

    /// Drop the published rects: the dock calls this for a pane it did not
    /// draw this frame (`DockState::ui`), whose widgets are not on screen.
    pub fn clear_hits(&mut self) {
        self.hits.clear();
        // This panel publishes its rects from `show`, which did not run.
        if crate::automation::registry::enabled() {
            self.publish_hits();
        }
    }

    fn publish_hits(&self) {
        crate::automation::registry::publish("__brepSceneHit", "scene tree widget rects (box:, sel:, vis:, typevis:)", &self.hits_json());
    }
}

/// The tristate for a group of entity rows, computed from their live per-entity
/// `visible` flags (the same the engine would report). An empty group reads
/// [`GroupState::All`] — nothing to hide, so the checkbox shows checked.
fn group_state(entities: &[Entity]) -> GroupState {
    if entities.is_empty() {
        return GroupState::All;
    }
    let visible = entities.iter().filter(|e| e.visible).count();
    if visible == entities.len() {
        GroupState::All
    } else if visible == 0 {
        GroupState::None
    } else {
        GroupState::Partial
    }
}

/// Snapshot `state.scene` into owned rows, precomputing each entity's `selected`
/// flag against the current engine selection (cloned in by the caller) and its
/// live per-entity `visible` flag. Face / edge labels fall back to `Face i` /
/// `Edge i` when the kernel left them unnamed; unnamed entities are then not
/// name-selectable (the click is a no-op) but still hideable (by index).
fn snapshot(
    state: &EngineState,
    sel_solids: &OrderedNames,
    sel_faces: &OrderedNames,
    sel_edges: &OrderedNames,
    sel_vertices: &[brep_render::style::VertexRef],
) -> Vec<SolidRow> {
    // Vertex positions are set exactly from the same source; a tiny tolerance
    // guards float round-trips.
    const TOL: f64 = 1e-6;
    state
        .scene
        .solids()
        .iter()
        // Committed-sketch SHEETS are scene solids too, but they list as their OWN
        // top-level sketch rows (`render_sketch_row`, checkbox wired to
        // `set_sketch_visible`) — never as plain solid rows, so they are dropped here
        // and never counted by the root / type-visibility toggles.
        .filter(|s| !s.is_sketch)
        .map(|s| {
            let faces = s
                .faces
                .iter()
                .enumerate()
                .map(|(i, f)| Entity {
                    label: if f.name.is_empty() {
                        format!("Face {i}")
                    } else {
                        f.name.clone()
                    },
                    selected: !f.name.is_empty() && sel_faces.contains(&f.name),
                    visible: s.visibility.is_face_visible(i),
                    kind: EntityKind::Face(f.name.clone()),
                })
                .collect();
            // AUX edges are display-only (a board's copper outline, thousands
            // of them) and are not entities. They trail the real edges, so
            // stopping at the first keeps a row's position its edge's index,
            // which is what the visibility checkbox hides by.
            let edges = s
                .edges
                .iter()
                .enumerate()
                .take_while(|(_, e)| !e.aux)
                .map(|(i, e)| Entity {
                    label: if e.name.is_empty() {
                        format!("Edge {i}")
                    } else {
                        e.name.clone()
                    },
                    selected: !e.name.is_empty() && sel_edges.contains(&e.name),
                    visible: s.visibility.is_edge_visible(i),
                    kind: EntityKind::Edge(e.name.clone()),
                })
                .collect();
            let vertices = s
                .vertices
                .iter()
                .enumerate()
                .map(|(i, v)| Entity {
                    label: format!("Vertex {i}"),
                    selected: sel_vertices.iter().any(|r| {
                        r.solid == s.name
                            && (r.position[0] - v.position[0]).abs() <= TOL
                            && (r.position[1] - v.position[1]).abs() <= TOL
                            && (r.position[2] - v.position[2]).abs() <= TOL
                    }),
                    visible: s.visibility.is_vertex_visible(i),
                    kind: EntityKind::Vertex(v.position),
                })
                .collect();
            SolidRow {
                name: s.name.clone(),
                visible: s.visible,
                selected: sel_solids.contains(&s.name),
                faces,
                edges,
                vertices,
            }
        })
        .collect()
}


/// The hit keys this panel publishes (see `automation::hit_keys`).
pub static HIT_KEYS: &[HitKeyDoc] = &[
    HitKeyDoc { panel: "scene", prefix: "box:", meaning: "expand/collapse a tree node (box:name, box:__scene, box:__datums)", command: None },
    HitKeyDoc { panel: "scene", prefix: "sel:", meaning: "select a tree row (sel:name, sel:name/ei, sel:sketch/id, sel:datum/name)", command: None },
    HitKeyDoc { panel: "scene", prefix: "vis:", meaning: "toggle a row's visibility (vis:name, vis:name/ei, vis:__scene)", command: None },
    HitKeyDoc { panel: "scene", prefix: "typevis:", meaning: "toggle visibility of a whole kind (typevis:label)", command: None },
];
