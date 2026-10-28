//! The engine-native 3D SKETCH mode: entering and leaving the edit of an
//! `S3D` feature, the cursor-to-3D mapping, the draw tools, model reference
//! linking, the constraint edits and the per-session undo.
//!
//! The planar sketch mode (`sketch_mode.rs`) is the model; what differs is
//! the geometry: there is no plane, so a click lands in 3D on the sketch's
//! own points, on model geometry under the cursor, or on the WORKING PLANE
//! (one of XY / YZ / XZ through the last placed point), and a drag moves a
//! point in the screen-parallel plane through where it was grabbed. Picking
//! is in screen space ([`crate::sketch3d::Sketch3dSession::pick_entity`]).
//!
//! A model entity linked into the sketch (the `pickRef` tool, or a snap onto a
//! vertex / edge / face while placing a point) becomes fixed construction
//! geometry — a point, a line, an arc or a reference plane — recorded in
//! `persistentData.externalRefs` by name so the `S3D` feature re-resolves it
//! against the live scene on every run
//! ([`brep_kernel::resolve_sketch3d_reference`]). The editor materializes it
//! from the DISPLAY scene (vertex positions, edge polylines, face planes), the
//! feature from the exact BREP; both describe the same entity, and the feature
//! keeps the editor's plane origin / axis point by projecting it onto the
//! resolved plane / line, so a regenerate moves nothing the user can see.

use super::*;
use crate::sketch3d::{Sketch3dDoc, Sketch3dSession, WorkPlane};
use serde_json::{json, Value};

/// One in-flight 3D sketch drag: the grabbed points and the anchor world point
/// on the drag plane (screen-parallel through the grab).
#[derive(Clone)]
pub struct Sketch3dDrag {
    /// Per grabbed point: `(id, original position, original fixed flag)`.
    pub(super) points: Vec<(Value, [f64; 3], bool)>,
    /// The drag plane through the grab point, normal to the view direction.
    pub(super) plane: crate::sketch::PlaneFrame,
    /// Where on the plane the grab started.
    pub(super) anchor: [f64; 3],
}

/// What a cursor position resolved to when placing a point.
#[derive(Clone, Debug, PartialEq)]
pub enum Sketch3dCursorHit {
    /// An existing sketch point (snap by id).
    SketchPoint(Value),
    /// A model VERTEX of `solid` at `position` (reference name `{solid}@x,y,z`).
    Vertex { solid: String, position: [f64; 3] },
    /// A point on a model EDGE (`name`), at the closest point under the cursor.
    Edge { name: String, position: [f64; 3] },
    /// A point on a planar model FACE (`name`).
    Face { name: String, position: [f64; 3] },
    /// A point on the working plane.
    WorkPlane([f64; 3]),
}

impl Sketch3dCursorHit {
    pub fn kind(&self) -> &'static str {
        match self {
            Sketch3dCursorHit::SketchPoint(_) => "point",
            Sketch3dCursorHit::Vertex { .. } => "vertex",
            Sketch3dCursorHit::Edge { .. } => "edge",
            Sketch3dCursorHit::Face { .. } => "face",
            Sketch3dCursorHit::WorkPlane(_) => "workPlane",
        }
    }
}

/// The active 3D sketch edit.
pub struct Sketch3dEdit {
    pub session: Sketch3dSession,
    pub feature_id: String,
    prev_rollback: usize,
    is_new: bool,
    creation_step: Option<u64>,
    pub(super) drag: Option<Sketch3dDrag>,
    /// The armed tool's placed point ids.
    pub(super) pending: Vec<Value>,
    /// The last cursor world point (the rubber band's free end).
    pub(super) hover_world: Option<[f64; 3]>,
    pub(super) undo_stack: Vec<Sketch3dSnapshot>,
    pub(super) redo_stack: Vec<Sketch3dSnapshot>,
    /// The reference materialization records (`persistentData.externalRefs`):
    /// `{name, kind, points, geometry}` per linked entity.
    pub(super) external_refs: Vec<Value>,
    /// Names the dialog's `references` list carried at enter that the scene
    /// could not materialize; Finish keeps them listed.
    pub(super) listed_unlinked: Vec<String>,
    /// The last cursor hit while a tool is armed, for the status readout.
    pub(super) last_hit: Option<&'static str>,
}

#[derive(Clone)]
pub(super) struct Sketch3dSnapshot {
    doc: Sketch3dDoc,
    selection: Vec<Value>,
    external_refs: Vec<Value>,
    work_plane: WorkPlane,
}

impl Sketch3dEdit {
    fn snapshot(&self) -> Sketch3dSnapshot {
        Sketch3dSnapshot {
            doc: self.session.doc.clone(),
            selection: self.session.selection.clone(),
            external_refs: self.external_refs.clone(),
            work_plane: self.session.work_plane.clone(),
        }
    }

    pub(super) fn record_undo(&mut self) {
        let snap = self.snapshot();
        self.undo_stack.push(snap);
        self.redo_stack.clear();
    }

    fn restore(&mut self, snap: Sketch3dSnapshot) {
        self.session.doc = snap.doc;
        self.session.selection = snap.selection;
        self.session.hovered = None;
        self.external_refs = snap.external_refs;
        self.session.work_plane = snap.work_plane;
        self.pending.clear();
        self.drag = None;
        let _ = self.session.resolve();
    }
}

/// The reference materialization record's shape, built here and read by the
/// kernel feature (`brep_kernel::sketch3d_reference_record` is the one author
/// of the fields; this wraps it for the editor's `Value` ids).
fn reference_record(name: &str, resolved: &brep_kernel::Sketch3dResolvedReference, points: &[Value], geometry: Option<&Value>) -> Value {
    brep_kernel::sketch3d_reference_record(name, resolved, points, geometry)
}

impl EngineState {
    /// True while a 3D sketch is being edited.
    pub fn sketch3d_mode(&self) -> bool {
        self.sketch3d_edit.is_some()
    }

    /// Either sketch editor is open — what the shell keys its sketch takeover
    /// on (the dock hides, the mode-exit card shows, undo routes to the
    /// session).
    pub fn any_sketch_mode(&self) -> bool {
        self.sketch_mode() || self.sketch3d_mode()
    }

    pub fn sketch3d_edit_session(&self) -> Option<&Sketch3dSession> {
        self.sketch3d_edit.as_ref().map(|edit| &edit.session)
    }

    pub fn sketch3d_edit_feature_id(&self) -> Option<&str> {
        self.sketch3d_edit.as_ref().map(|edit| edit.feature_id.as_str())
    }

    /// Enter 3D sketch mode on the `S3D` feature `feature_id`: snapshot the
    /// camera + rolled-to step, roll to the step before the sketch (its
    /// backdrop), load the document and the reference records, materialize
    /// every listed reference the backdrop can see, solve, and show the
    /// overlay. Returns the diagnostics JSON.
    pub fn enter_sketch3d_mode(&mut self, feature_id: &str) -> Result<String, String> {
        if self.sketch_mode() {
            return Err("a planar sketch is being edited; finish it first".into());
        }
        if self.sketch3d_mode() {
            return Err("a 3D sketch is already being edited".into());
        }
        let index = self
            .history
            .index_of(feature_id)
            .ok_or_else(|| format!("no feature with id '{feature_id}'"))?;
        let ty = self.history.feature_type(index).unwrap_or_default();
        if ty != brep_kernel::SKETCH3D_FEATURE_TYPE {
            return Err(format!("feature '{feature_id}' is not a 3D sketch (type \"S3D\")"));
        }

        // The camera is left alone on entry (there is no plane to face) and
        // therefore on exit: every orbit made while sketching stands.
        let prev_rollback = self.history.rollback();
        self.history.set_rollback(index.saturating_sub(1));
        self.rerun_history();

        let persistent = self.history.feature_persistent_data(index);
        let doc_value = persistent
            .as_ref()
            .and_then(|p| p.get(brep_kernel::SKETCH3D_DOC_KEY))
            .cloned()
            .unwrap_or_else(|| json!({ "points": [], "geometries": [], "constraints": [] }));
        let doc: Sketch3dDoc = serde_json::from_value(doc_value)
            .map_err(|error| format!("3D sketch doc parse: {error}"))?;
        let mut session = Sketch3dSession::new(doc)?;
        session.colors = self.settings.sketch_colors();
        if let Some(name) = persistent
            .as_ref()
            .and_then(|p| p.get("workPlane"))
            .and_then(Value::as_str)
        {
            if let Some(plane) = WorkPlane::new(name, [0.0; 3]) {
                session.work_plane = plane;
            }
        }
        let external_refs: Vec<Value> = persistent
            .as_ref()
            .and_then(|p| p.get(brep_kernel::SKETCH3D_REFS_KEY))
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        let listed = brep_kernel::reference_names(
            self.history
                .feature_params(index)
                .as_ref()
                .and_then(|p| p.get(brep_kernel::SKETCH3D_REFERENCES_PARAM)),
        );

        self.clear_selection();
        self.set_sketch3d_overlay(&session);
        self.sketch3d_edit = Some(Sketch3dEdit {
            session,
            feature_id: feature_id.to_string(),
            prev_rollback,
            is_new: false,
            creation_step: None,
            drag: None,
            pending: Vec::new(),
            hover_world: None,
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            external_refs,
            listed_unlinked: Vec::new(),
            last_hit: None,
        });
        // Sync the links with the dialog's list against the backdrop: every
        // record re-materializes (so a linked entity tracks an upstream edit),
        // a listed name without a record is linked, and a name the backdrop
        // cannot see is remembered for Finish.
        self.sketch3d_sync_references(&listed);
        self.refresh_committed_sketches();
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(self.sketch3d_diagnostics_json())
    }

    /// Exit 3D sketch mode. `commit` writes the document, the reference
    /// records and the working plane back; a cancel of a sketch this session
    /// created deletes it. Restores the camera + rolled-to step and re-runs.
    pub fn exit_sketch3d_mode(&mut self, commit: bool) -> String {
        let Some(edit) = self.sketch3d_edit.take() else {
            return "{}".to_string();
        };
        if commit {
            if let Some(index) = self.history.index_of(&edit.feature_id) {
                let sketch_value = serde_json::to_value(&edit.session.doc)
                    .unwrap_or_else(|_| json!({ "points": [], "geometries": [], "constraints": [] }));
                let refs_value = Value::Array(edit.external_refs.clone());
                let fields = vec![
                    (brep_kernel::SKETCH3D_DOC_KEY, sketch_value),
                    (brep_kernel::SKETCH3D_REFS_KEY, refs_value),
                    ("workPlane", Value::String(edit.session.work_plane.name.clone())),
                ];
                match edit.creation_step {
                    Some(step) => {
                        self.history.set_feature_persistent_fields_in_step(index, fields, step);
                    }
                    None => {
                        self.history.set_feature_persistent_fields(index, fields);
                    }
                }
                // The dialog's list names every link: listed order kept,
                // dropped links gone, unmaterialized names kept, new links
                // appended. Folded without a checkpoint so Finish is one step.
                let current = self.history.feature_params(index);
                let listed = brep_kernel::reference_names(
                    current.as_ref().and_then(|p| p.get(brep_kernel::SKETCH3D_REFERENCES_PARAM)),
                );
                let linked: Vec<String> = edit
                    .external_refs
                    .iter()
                    .filter_map(|r| r.get("name").and_then(Value::as_str).map(String::from))
                    .collect();
                let mut names: Vec<String> = listed
                    .iter()
                    .filter(|n| linked.contains(n) || edit.listed_unlinked.contains(n))
                    .cloned()
                    .collect();
                for name in &linked {
                    if !names.contains(name) {
                        names.push(name.clone());
                    }
                }
                if names != listed {
                    self.history.fold_param_no_undo(
                        &edit.feature_id,
                        brep_kernel::SKETCH3D_REFERENCES_PARAM,
                        Value::Array(names.into_iter().map(Value::String).collect()),
                    );
                }
            }
        } else if edit.is_new {
            if edit.creation_step.is_some_and(|step| self.history.retract(step)) {
                self.reinstall_rewound_parts_library();
            } else if let Some(index) = self.history.index_of(&edit.feature_id) {
                self.history.remove_feature(index);
            }
        }
        self.history.set_rollback(edit.prev_rollback);
        self.clear_sketch3d_overlay();
        self.dirty = true;
        self.rerun_history()
    }

    /// Create a NEW 3D sketch feature and enter its edit. A cancel exit
    /// deletes it. Returns the diagnostics JSON.
    pub fn new_sketch3d(&mut self) -> Result<String, String> {
        let id = self
            .history
            .next_feature_id(&crate::features::feature_short_name(brep_kernel::SKETCH3D_FEATURE_TYPE));
        let feature = json!({
            "type": brep_kernel::SKETCH3D_FEATURE_TYPE,
            "inputParams": { "id": id },
            "persistentData": {
                brep_kernel::SKETCH3D_DOC_KEY: { "points": [], "geometries": [], "constraints": [] }
            }
        });
        self.history.push_feature(feature);
        let creation_step = self.history.undo_top();
        let diagnostics = self.enter_sketch3d_mode(&id)?;
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.is_new = true;
            edit.creation_step = creation_step;
        }
        Ok(diagnostics)
    }

    // --- overlay -------------------------------------------------------------

    fn set_sketch3d_overlay(&mut self, session: &Sketch3dSession) {
        let wpp = self.camera.world_per_pixel();
        let _ = self.set_overlay_json(&session.overlay_json(wpp));
        let _ = self.set_overlay_json(&session.workplane_overlay_json(wpp));
        self.sketch3d_overlay_wpp = if wpp > 0.0 { wpp } else { f64::MIN_POSITIVE };
    }

    pub(super) fn refresh_sketch3d_overlay(&mut self) {
        let wpp = self.camera.world_per_pixel();
        let (geometry, preview, workplane) = match self.sketch3d_edit.as_ref() {
            Some(edit) => (
                edit.session.overlay_json(wpp),
                edit.session.preview_overlay_json(&edit.pending, edit.hover_world),
                edit.session.workplane_overlay_json(wpp),
            ),
            None => return,
        };
        let _ = self.set_overlay_json(&geometry);
        let _ = self.set_overlay_json(&preview);
        let _ = self.set_overlay_json(&workplane);
        self.sketch3d_overlay_wpp = if wpp > 0.0 { wpp } else { f64::MIN_POSITIVE };
    }

    /// Per-frame upkeep: the working-plane square and the reference-plane
    /// squares are sized in pixels, so a material zoom re-bakes them.
    pub(super) fn ensure_sketch3d_overlay_current(&mut self) {
        if !self.sketch3d_mode() {
            self.sketch3d_overlay_wpp = 0.0;
            return;
        }
        let wpp = self.camera.world_per_pixel();
        if super::overlay_wpp_stale(self.sketch3d_overlay_wpp, wpp) {
            self.refresh_sketch3d_overlay();
        }
    }

    pub fn clear_sketch3d_overlay(&mut self) {
        let groups: Vec<Value> = crate::sketch3d::OVERLAY_GROUPS
            .iter()
            .map(|name| json!({ "name": name }))
            .collect();
        let _ = self.set_overlay_json(&json!({ "groups": groups }).to_string());
        self.sketch3d_overlay_wpp = 0.0;
    }

    // --- readback ------------------------------------------------------------

    pub fn sketch3d_diagnostics_json(&self) -> String {
        self.sketch3d_edit
            .as_ref()
            .and_then(|edit| serde_json::to_string(&edit.session.diagnostics).ok())
            .unwrap_or_else(|| "{}".to_string())
    }

    /// The `__brepSketch3d` state: mode, feature, DOF readout, tool, working
    /// plane, counts, selection, the palette and the labels.
    pub fn sketch3d_state_json(&self) -> String {
        let Some(edit) = self.sketch3d_edit.as_ref() else {
            return json!({ "sketch3dMode": false }).to_string();
        };
        let session = &edit.session;
        json!({
            "sketch3dMode": true,
            "featureId": edit.feature_id,
            "dof": session.diagnostics.dof,
            "status": session.diagnostics.status,
            "conflicting": session.diagnostics.conflicting,
            "conflictingConstraints": session.diagnostics.conflicting_constraints,
            "redundant": session.diagnostics.redundant,
            "tool": session.tool.clone().unwrap_or_else(|| "select".into()),
            "workPlane": session.work_plane.name,
            "workPlaneAnchor": session.work_plane.anchor,
            "pointCount": session.doc.points.len(),
            "geometryCount": session.doc.geometries.len(),
            "constraintCount": session.doc.constraints.len(),
            "pendingLen": edit.pending.len(),
            "selection": session.selection,
            "hovered": session.hovered,
            "applicableConstraints": session.applicable_constraints(),
            "dimensions": session.dimension_labels(),
            "externalRefs": edit.external_refs,
            "lastHit": edit.last_hit,
            "canUndo": !edit.undo_stack.is_empty(),
            "canRedo": !edit.redo_stack.is_empty(),
            "entities": session.entity_rows_json(),
        })
        .to_string()
    }

    /// The dimension labels as JSON (the same shape as the planar sketch's).
    pub fn sketch3d_dimension_labels_json(&self) -> String {
        self.sketch3d_edit
            .as_ref()
            .and_then(|edit| serde_json::to_string(&edit.session.dimension_labels()).ok())
            .unwrap_or_else(|| "[]".to_string())
    }

    pub fn sketch3d_active_tool(&self) -> Option<&str> {
        self.sketch3d_edit.as_ref().and_then(|edit| edit.session.tool.as_deref())
    }

    pub fn sketch3d_selection_count(&self) -> usize {
        self.sketch3d_edit.as_ref().map_or(0, |edit| edit.session.selection.len())
    }

    pub fn sketch3d_pending_len(&self) -> usize {
        self.sketch3d_edit.as_ref().map_or(0, |edit| edit.pending.len())
    }

    pub fn sketch3d_work_plane(&self) -> Option<&str> {
        self.sketch3d_edit.as_ref().map(|edit| edit.session.work_plane.name.as_str())
    }

    // --- tools ---------------------------------------------------------------

    /// Arm a draw tool (`point`, `line`, `arc`, `bezier`, `pickRef`); `select`
    /// / `None` returns to selection. Errors on an unknown tool.
    pub fn sketch3d_set_tool(&mut self, tool: Option<&str>) -> Result<(), String> {
        let tool = match tool {
            None | Some("select") | Some("") => None,
            Some(name) if crate::sketch3d::TOOLS.contains(&name) => Some(name.to_string()),
            Some(other) => return Err(format!("unknown 3D sketch tool '{other}'")),
        };
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return Err("not in 3D sketch mode".into());
        };
        edit.session.tool = tool;
        edit.pending.clear();
        edit.drag = None;
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(())
    }

    /// Drop the in-progress placement (right-click / Escape with clicks
    /// pending) without changing the armed tool.
    pub fn sketch3d_tool_cancel(&mut self) {
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.pending.clear();
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    /// Set the working plane by name (`XY` / `YZ` / `XZ`); it keeps passing
    /// through the current anchor.
    pub fn sketch3d_set_work_plane(&mut self, name: &str) -> Result<(), String> {
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return Err("not in 3D sketch mode".into());
        };
        let anchor = edit.session.work_plane.anchor;
        edit.session.work_plane =
            WorkPlane::new(name, anchor).ok_or_else(|| format!("unknown working plane '{name}' (expected XY|YZ|XZ)"))?;
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(())
    }

    /// Cycle the working plane `XY → YZ → XZ → XY` (the Tab key).
    pub fn sketch3d_cycle_work_plane(&mut self) {
        let next = self
            .sketch3d_edit
            .as_ref()
            .map(|edit| edit.session.work_plane.next_name());
        if let Some(next) = next {
            let _ = self.sketch3d_set_work_plane(next);
        }
    }

    fn sketch3d_pick_radius_px(&self) -> f64 {
        f64::from(crate::sketch::tessellate::POINT_SIZE_PX) * 1.5
    }

    /// What the cursor at CSS-pixel `(x, y)` lands on when PLACING a point:
    /// an existing sketch point within the grab radius, else a model vertex /
    /// edge / planar face under the cursor (the pick ranking, committed
    /// sketch displays excluded), else the working plane.
    pub fn sketch3d_cursor_hit(&self, x: f64, y: f64) -> Option<Sketch3dCursorHit> {
        let edit = self.sketch3d_edit.as_ref()?;
        let radius = self.sketch3d_pick_radius_px();
        if let Some(entity) = edit.session.pick_entity(&self.camera, x, y, radius) {
            if entity.get("kind").and_then(Value::as_str) == Some("point") {
                return entity.get("id").cloned().map(Sketch3dCursorHit::SketchPoint);
            }
        }
        for candidate in crate::pick::pick_unoccluded(&self.scene, &self.context_scene, &self.camera, x, y, &self.pick_options()) {
            let Some(solid) = self.scene.solids().iter().find(|s| s.name == candidate.solid) else {
                continue;
            };
            if solid.is_sketch || !solid.visible {
                continue;
            }
            match candidate.kind {
                crate::pick::PickKind::Vertex => {
                    return Some(Sketch3dCursorHit::Vertex {
                        solid: candidate.solid,
                        position: candidate.position,
                    });
                }
                crate::pick::PickKind::Edge if !candidate.name.is_empty() => {
                    return Some(Sketch3dCursorHit::Edge {
                        name: candidate.name,
                        position: candidate.position,
                    });
                }
                crate::pick::PickKind::Face if !candidate.name.is_empty() => {
                    if self.scene.face_is_planar(&candidate.name) {
                        return Some(Sketch3dCursorHit::Face {
                            name: candidate.name,
                            position: candidate.position,
                        });
                    }
                    // A curved face is not a plane to land on; fall through
                    // to the working plane.
                    break;
                }
                _ => continue,
            }
        }
        let ray = self.camera.pick_ray(x, y);
        let frame = edit.session.work_plane.frame();
        let (u, v) = crate::sketch::ray_plane_uv(&frame, ray.origin, ray.dir)?;
        Some(Sketch3dCursorHit::WorkPlane(frame.to_world(u, v)))
    }

    /// The world point of a cursor hit.
    fn sketch3d_hit_world(&self, hit: &Sketch3dCursorHit) -> Option<[f64; 3]> {
        match hit {
            Sketch3dCursorHit::SketchPoint(id) => self.sketch3d_edit.as_ref()?.session.point_world(id),
            Sketch3dCursorHit::Vertex { position, .. }
            | Sketch3dCursorHit::Edge { position, .. }
            | Sketch3dCursorHit::Face { position, .. }
            | Sketch3dCursorHit::WorkPlane(position) => Some(*position),
        }
    }

    /// The world point the cursor would place at `(x, y)` (the readback /
    /// rubber band), `None` when the ray misses the working plane.
    pub fn sketch3d_cursor_world(&self, x: f64, y: f64) -> Option<[f64; 3]> {
        let hit = self.sketch3d_cursor_hit(x, y)?;
        self.sketch3d_hit_world(&hit)
    }

    /// Resolve the cursor into a point id for the armed tool: an existing
    /// sketch point is reused; a model vertex / edge / face hit places a point
    /// there AND links the entity (coincident / on-line / on-plane); a working
    /// plane hit places a free point. The working plane then moves to the
    /// placed point.
    fn sketch3d_place_point(&mut self, hit: Sketch3dCursorHit) -> Option<Value> {
        let world = self.sketch3d_hit_world(&hit)?;
        let id = match &hit {
            Sketch3dCursorHit::SketchPoint(id) => id.clone(),
            _ => {
                let edit = self.sketch3d_edit.as_mut()?;
                let radius = 0.0;
                edit.session.doc.snap_or_add_point(brep_kernel::Vec3::new(world[0], world[1], world[2]), radius)
            }
        };
        match &hit {
            Sketch3dCursorHit::Vertex { solid, position } => {
                let name = format!("{solid}@{},{},{}", fmt_coord(position[0]), fmt_coord(position[1]), fmt_coord(position[2]));
                if let Some(ids) = self.sketch3d_link_reference_silent(&name) {
                    if let Some(edit) = self.sketch3d_edit.as_mut() {
                        if let Some(ref_point) = ids.first() {
                            edit.session.doc.add_constraint("coincident", vec![id.clone(), ref_point.clone()], None);
                        }
                    }
                }
            }
            Sketch3dCursorHit::Edge { name, .. } => {
                if let Some(ids) = self.sketch3d_link_reference_silent(name) {
                    if let Some(edit) = self.sketch3d_edit.as_mut() {
                        match ids.len() {
                            // A line: on the infinite line through its ends.
                            2 => {
                                edit.session.doc.add_constraint("onLine", vec![id.clone(), ids[0].clone(), ids[1].clone()], None);
                            }
                            // An arc [center, start, end]: at the arc's radius
                            // from its center (the on-arc relation this sketch
                            // can state).
                            3 => {
                                let radius = edit
                                    .session
                                    .measure("distance", &[ids[0].clone(), ids[1].clone()])
                                    .unwrap_or(0.0);
                                edit.session.doc.add_constraint("distance", vec![id.clone(), ids[0].clone()], Some(radius));
                            }
                            _ => {}
                        }
                    }
                }
            }
            Sketch3dCursorHit::Face { name, .. } => {
                if let Some(ids) = self.sketch3d_link_reference_silent(name) {
                    if let Some(edit) = self.sketch3d_edit.as_mut() {
                        if ids.len() == 2 {
                            edit.session.doc.add_constraint("onPlane", vec![id.clone(), ids[0].clone(), ids[1].clone()], None);
                        }
                    }
                }
            }
            _ => {}
        }
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.session.work_plane.anchor = world;
            edit.last_hit = Some(hit.kind());
        }
        Some(id)
    }

    /// A click with a draw tool armed: place the next point of the geometry
    /// being drawn; when the tool has all its points, create the geometry.
    /// `line` chains: each click after the first lands a segment and starts
    /// the next from its end.
    pub fn sketch3d_tool_click_at(&mut self, x: f64, y: f64) {
        let Some(tool) = self.sketch3d_active_tool().map(String::from) else {
            return;
        };
        if tool == "pickRef" {
            self.sketch3d_pick_reference_at(x, y);
            return;
        }
        let Some(hit) = self.sketch3d_cursor_hit(x, y) else {
            return;
        };
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.record_undo();
        }
        let Some(id) = self.sketch3d_place_point(hit) else {
            return;
        };
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return;
        };
        // A repeat click on the pending point is a no-op.
        if edit.pending.last().map_or(false, |last| brep_kernel::sketch_id_key(last) == brep_kernel::sketch_id_key(&id)) {
            edit.undo_stack.pop();
            return;
        }
        edit.pending.push(id);
        match tool.as_str() {
            "point" => {
                let id = edit.pending.pop().unwrap();
                edit.session.doc.add_geometry("point", vec![id], false);
            }
            "line" if edit.pending.len() >= 2 => {
                let b = edit.pending.pop().unwrap();
                let a = edit.pending.pop().unwrap();
                edit.session.doc.add_geometry("line", vec![a, b.clone()], false);
                edit.pending.push(b);
            }
            "arc" if edit.pending.len() >= 3 => {
                let pts = std::mem::take(&mut edit.pending);
                edit.session.doc.add_geometry("arc", pts, false);
            }
            "bezier" if edit.pending.len() >= 4 => {
                let pts = std::mem::take(&mut edit.pending);
                edit.session.doc.add_geometry("bezier", pts, false);
            }
            _ => {}
        }
        let _ = edit.session.resolve();
        // Nothing changed beyond a pending click: the snapshot still stands so
        // the whole geometry is one undo step, which Escape or the next click
        // completes.
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    // --- references ----------------------------------------------------------

    /// The `pickRef` tool: link the model entity under the cursor (vertex,
    /// edge, planar face or datum plane) into the sketch as fixed construction
    /// geometry. Returns whether something was linked.
    pub fn sketch3d_pick_reference_at(&mut self, x: f64, y: f64) -> bool {
        let candidates = self.pick_candidates_at(x, y);
        let mut name: Option<String> = None;
        for candidate in candidates {
            match candidate.kind {
                crate::pick::PickKind::Vertex => {
                    if self.scene.solids().iter().any(|s| s.name == candidate.solid && !s.is_sketch) {
                        let p = candidate.position;
                        name = Some(format!("{}@{},{},{}", candidate.solid, fmt_coord(p[0]), fmt_coord(p[1]), fmt_coord(p[2])));
                        break;
                    }
                }
                crate::pick::PickKind::Edge | crate::pick::PickKind::Face | crate::pick::PickKind::Plane
                    if !candidate.name.is_empty() =>
                {
                    if candidate.kind == crate::pick::PickKind::Plane
                        || self.scene.solids().iter().any(|s| s.name == candidate.solid && !s.is_sketch)
                    {
                        name = Some(candidate.name);
                        break;
                    }
                }
                _ => {}
            }
        }
        let Some(name) = name else { return false };
        self.sketch3d_link_reference(&name)
    }

    /// Link a model entity BY NAME (a `{solid}@x,y,z` vertex, an edge or face
    /// name, a datum / plane frame name): materialize it as fixed construction
    /// geometry and record it. Re-linking an already linked name updates its
    /// coordinates in place. Returns whether it resolved.
    pub fn sketch3d_link_reference(&mut self, name: &str) -> bool {
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.record_undo();
        }
        let linked = self.sketch3d_link_reference_silent(name).is_some();
        if !linked {
            if let Some(edit) = self.sketch3d_edit.as_mut() {
                edit.undo_stack.pop();
            }
        }
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            let _ = edit.session.resolve();
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        linked
    }

    /// Materialize (or update) a reference without an undo snapshot or
    /// overlay refresh; returns the record's point ids.
    fn sketch3d_link_reference_silent(&mut self, name: &str) -> Option<Vec<Value>> {
        let resolved = self.sketch3d_resolve_reference_from_display(name)?;
        let edit = self.sketch3d_edit.as_mut()?;
        let positions = resolved.positions();
        // Update an existing record of the same kind in place.
        if let Some(record) = edit
            .external_refs
            .iter()
            .find(|r| r.get("name").and_then(Value::as_str) == Some(name))
            .cloned()
        {
            let ids: Vec<Value> = record.get("points").and_then(Value::as_array).cloned().unwrap_or_default();
            if record.get("kind").and_then(Value::as_str) == Some(resolved.kind()) && ids.len() == positions.len() {
                for (id, p) in ids.iter().zip(&positions) {
                    if let Some(point) = edit.session.doc.point_mut(id) {
                        point.x = p.x;
                        point.y = p.y;
                        point.z = p.z;
                        point.fixed = true;
                        point.construction = true;
                        point.external_reference = true;
                    }
                }
                return Some(ids);
            }
            // The shape changed: drop the old materialization and rebuild.
            edit.external_refs.retain(|r| r.get("name").and_then(Value::as_str) != Some(name));
            for id in &ids {
                edit.session.doc.points.retain(|p| brep_kernel::sketch_id_key(&p.id) != brep_kernel::sketch_id_key(id));
            }
            if let Some(gid) = record.get("geometry").filter(|g| !g.is_null()) {
                edit.session.doc.geometries.retain(|g| brep_kernel::sketch_id_key(&g.id) != brep_kernel::sketch_id_key(gid));
            }
        }
        let mut ids: Vec<Value> = Vec::with_capacity(positions.len());
        for p in &positions {
            let id = edit.session.doc.add_point(*p);
            if let Some(point) = edit.session.doc.point_mut(&id) {
                point.fixed = true;
                point.construction = true;
                point.external_reference = true;
            }
            ids.push(id);
        }
        let geometry = resolved
            .geometry_type()
            .map(|ty| edit.session.doc.add_geometry(ty, ids.clone(), true));
        edit.external_refs.push(reference_record(name, &resolved, &ids, geometry.as_ref()));
        Some(ids)
    }

    /// Resolve a reference name against the DISPLAY scene (the backdrop as
    /// drawn): a datum / plane frame, a planar face, an edge polyline
    /// classified as line / arc / circle axis, or a `{solid}@x,y,z` vertex.
    fn sketch3d_resolve_reference_from_display(&self, name: &str) -> Option<brep_kernel::Sketch3dResolvedReference> {
        use brep_kernel::{Sketch3dResolvedReference as R, Vec3, SKETCH3D_REFERENCE_TIP_LENGTH as TIP};
        let v = |a: [f64; 3]| Vec3::new(a[0], a[1], a[2]);
        if let Some((_, frame)) = self.construction_frames.iter().find(|(n, _)| n == name) {
            return Some(R::Plane(frame.origin, frame.origin.add(frame.z_axis.scale(TIP))));
        }
        if let Some((_, axis)) = self.sketch_axes.iter().find(|(n, _)| n == name) {
            return Some(R::Line(axis.point, axis.point.add(axis.direction.scale(TIP))));
        }
        if let Some((_, point)) = self.sketch_points.iter().find(|(n, _)| n == name) {
            return Some(R::Point(point.position));
        }
        if let Some((centroid, normal)) = self.scene.face_plane_world(name) {
            if !self.scene.face_is_planar(name) {
                return None;
            }
            return Some(R::Plane(v(centroid), v(centroid).add(v(normal).scale(TIP))));
        }
        if let Some(polyline) = self.scene.edge_polyline_world(name) {
            return classify_edge_polyline(&polyline);
        }
        if let Some((solid_name, coords)) = name.rsplit_once('@') {
            let mut parts = coords.split(',').map(|p| p.trim().parse::<f64>().ok());
            let position = [parts.next()??, parts.next()??, parts.next()??];
            let solid = self.scene.solids().iter().find(|s| s.name == solid_name)?;
            let best = solid
                .vertices
                .iter()
                .map(|vertex| vertex.position)
                .min_by(|a, b| dist(*a, position).total_cmp(&dist(*b, position)))?;
            if dist(best, position) > 1e-4 * (1.0 + best.iter().fold(0.0f64, |m, c| m.max(c.abs()))) {
                return None;
            }
            return Some(R::Point(v(best)));
        }
        None
    }

    /// Sync the links with the dialog's `references` list against the current
    /// backdrop: a record the list no longer names was removed in the dialog
    /// and is UNLINKED (its construction geometry and the constraints on it
    /// go); every remaining record re-materializes (so a linked entity tracks
    /// an upstream edit); every listed name without a record is linked; and
    /// the listed names the backdrop cannot see are remembered for Finish.
    /// The list is authoritative from the first Finish, which is what wrote
    /// it: a record can only exist because a Finish listed its name.
    fn sketch3d_sync_references(&mut self, listed: &[String]) {
        let names: Vec<String> = self
            .sketch3d_edit
            .as_ref()
            .map(|edit| {
                edit.external_refs
                    .iter()
                    .filter_map(|r| r.get("name").and_then(Value::as_str).map(String::from))
                    .collect()
            })
            .unwrap_or_default();
        for name in &names {
            if !listed.contains(name) {
                self.sketch3d_unlink_reference(name);
                if let Some(edit) = self.sketch3d_edit.as_mut() {
                    // An enter-time release is not an edit the user made.
                    edit.undo_stack.clear();
                    edit.redo_stack.clear();
                }
            }
        }
        let names: Vec<String> = names.into_iter().filter(|n| listed.contains(n)).collect();
        for name in &names {
            let _ = self.sketch3d_link_reference_silent(name);
        }
        let mut unlinked = Vec::new();
        for name in listed {
            if names.contains(name) {
                continue;
            }
            if self.sketch3d_link_reference_silent(name).is_none() {
                unlinked.push(name.clone());
            }
        }
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.listed_unlinked = unlinked;
            let _ = edit.session.resolve();
        }
    }

    /// Drop a linked reference by name: its record, its construction geometry
    /// and points, and every constraint touching them.
    pub fn sketch3d_unlink_reference(&mut self, name: &str) -> bool {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return false };
        let Some(record) = edit
            .external_refs
            .iter()
            .find(|r| r.get("name").and_then(Value::as_str) == Some(name))
            .cloned()
        else {
            return false;
        };
        edit.record_undo();
        edit.external_refs.retain(|r| r.get("name").and_then(Value::as_str) != Some(name));
        let ids: Vec<Value> = record.get("points").and_then(Value::as_array).cloned().unwrap_or_default();
        let gid = record.get("geometry").filter(|g| !g.is_null()).cloned();
        let mut refs: Vec<Value> = ids.iter().map(crate::sketch3d::entity_point_ref).collect();
        if let Some(gid) = gid {
            refs.push(crate::sketch3d::entity_geometry_ref(&gid));
        }
        edit.session.selection = refs;
        self.sketch3d_delete_selection();
        true
    }

    // --- selection / hover / drag --------------------------------------------

    pub fn sketch3d_hover_at(&mut self, x: f64, y: f64) -> bool {
        if self.sketch3d_edit.is_none() {
            return false;
        }
        let world = self.sketch3d_cursor_world(x, y);
        let radius = self.sketch3d_pick_radius_px();
        let new_hover = self
            .sketch3d_edit
            .as_ref()
            .and_then(|edit| edit.session.pick_entity(&self.camera, x, y, radius));
        let (changed, preview_live) = match self.sketch3d_edit.as_mut() {
            Some(edit) => {
                let changed = !crate::sketch::session::entity_ref_eq(edit.session.hovered.as_ref(), new_hover.as_ref());
                edit.session.hovered = new_hover;
                let moved = edit.hover_world != world;
                edit.hover_world = world;
                (changed, edit.session.tool.is_some() && moved)
            }
            None => (false, false),
        };
        if changed || preview_live {
            self.refresh_sketch3d_overlay();
            self.dirty = true;
        }
        changed
    }

    pub fn sketch3d_clear_hover(&mut self) -> bool {
        let changed = self
            .sketch3d_edit
            .as_mut()
            .map_or(false, |edit| {
                let had = edit.session.hovered.take().is_some() || edit.hover_world.take().is_some();
                had
            });
        if changed {
            self.refresh_sketch3d_overlay();
            self.dirty = true;
        }
        changed
    }

    /// Click-select: the entity under the cursor replaces (or, additive /
    /// click-toggles, toggles in) the selection; empty space clears it.
    pub fn sketch3d_click_at(&mut self, x: f64, y: f64, additive: bool) {
        let toggles = self.settings.multi_select == crate::style::MultiSelectMode::ClickToggles;
        let radius = self.sketch3d_pick_radius_px();
        let hit = self
            .sketch3d_edit
            .as_ref()
            .and_then(|edit| edit.session.pick_entity(&self.camera, x, y, radius));
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        match hit {
            None => edit.session.clear_selection(),
            Some(entity) => {
                if additive || toggles {
                    edit.session.toggle_selection(entity);
                } else {
                    edit.session.clear_selection();
                    edit.session.toggle_selection(entity);
                }
            }
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    /// Select an entity by ref from a list row (replacing the selection).
    pub fn sketch3d_select_entity(&mut self, entity_ref: Value, additive: bool) {
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            if !additive {
                edit.session.clear_selection();
            }
            edit.session.toggle_selection(entity_ref);
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    pub fn sketch3d_clear_selection(&mut self) {
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.session.clear_selection();
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    /// Begin a drag of the hovered (or picked) draggable entity. Returns
    /// whether something was grabbed; otherwise the drag is the camera's.
    pub fn sketch3d_drag_begin(&mut self, x: f64, y: f64) -> bool {
        let radius = self.sketch3d_pick_radius_px();
        let points = self.sketch3d_edit.as_ref().and_then(|edit| match edit.session.hovered.as_ref() {
            Some(entity) => edit.session.drag_points_from_ref(entity),
            None => edit
                .session
                .pick_draggable_point(&self.camera, x, y, radius)
                .and_then(|(id, fixed)| edit.session.point_world(&id).map(|p| vec![(id, p, fixed)])),
        });
        let Some(points) = points else { return false };
        // The drag plane: through the first grabbed point, facing the camera.
        let (_, _, forward) = self.camera.basis();
        let plane = crate::sketch::PlaneFrame::from_normal(points[0].1, [-forward[0], -forward[1], -forward[2]]);
        let ray = self.camera.pick_ray(x, y);
        let Some((u, v)) = crate::sketch::ray_plane_uv(&plane, ray.origin, ray.dir) else {
            return false;
        };
        let anchor = plane.to_world(u, v);
        if let Some(edit) = self.sketch3d_edit.as_mut() {
            edit.record_undo();
            edit.drag = Some(Sketch3dDrag { points, plane, anchor });
        }
        true
    }

    /// Drag the grabbed points to the cursor: each is pinned at its original
    /// position plus the cursor delta on the drag plane, the sketch re-solved,
    /// and the original fixed flags restored.
    pub fn sketch3d_drag_to(&mut self, x: f64, y: f64) {
        let ray = self.camera.pick_ray(x, y);
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        let Some(drag) = edit.drag.clone() else { return };
        let Some((u, v)) = crate::sketch::ray_plane_uv(&drag.plane, ray.origin, ray.dir) else {
            return;
        };
        let here = drag.plane.to_world(u, v);
        let delta = [here[0] - drag.anchor[0], here[1] - drag.anchor[1], here[2] - drag.anchor[2]];
        let session = &mut edit.session;
        for (id, orig, _) in &drag.points {
            if let Some(p) = session.doc.point_mut(id) {
                p.x = orig[0] + delta[0];
                p.y = orig[1] + delta[1];
                p.z = orig[2] + delta[2];
                p.fixed = true;
            }
        }
        match session.resolve() {
            Ok(()) => {
                for (id, _, fixed) in &drag.points {
                    if let Some(p) = session.doc.point_mut(id) {
                        p.fixed = *fixed;
                    }
                }
            }
            Err(_) => {
                for (id, orig, fixed) in &drag.points {
                    if let Some(p) = session.doc.point_mut(id) {
                        p.x = orig[0];
                        p.y = orig[1];
                        p.z = orig[2];
                        p.fixed = *fixed;
                    }
                }
            }
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    pub fn sketch3d_drag_end(&mut self) {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        if edit.drag.take().is_none() {
            return;
        }
        let _ = edit.session.resolve();
        if edit.undo_stack.last().map_or(false, |snap| snap.doc == edit.session.doc) {
            edit.undo_stack.pop();
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    // --- edits ---------------------------------------------------------------

    /// Add a constraint of `ctype` over the palette's point ids for the
    /// current selection (`sketch3d_applicable_constraints`); a dimensional
    /// one is seeded with the current measurement. Returns the constraint id.
    pub fn sketch3d_add_constraint(&mut self, ctype: &str) -> Result<Value, String> {
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return Err("not in 3D sketch mode".into());
        };
        let offer = edit
            .session
            .applicable_constraints()
            .into_iter()
            .find(|c| c.ctype == ctype)
            .ok_or_else(|| format!("constraint '{ctype}' does not apply to the current selection"))?;
        edit.record_undo();
        if ctype == "fixed" {
            for id in &offer.points {
                if let Some(p) = edit.session.doc.point_mut(id) {
                    p.fixed = true;
                }
            }
            let _ = edit.session.resolve();
            self.refresh_sketch3d_overlay();
            self.dirty = true;
            return Ok(Value::Null);
        }
        let value = if offer.dimensional {
            edit.session.measure(ctype, &offer.points)
        } else {
            None
        };
        let id = edit.session.doc.add_constraint(ctype, offer.points, value);
        if let Err(error) = edit.session.resolve() {
            // An unsolvable addition is taken back.
            if let Some(snap) = edit.undo_stack.pop() {
                edit.restore(snap);
            }
            return Err(error);
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(id)
    }

    /// Add a constraint over EXPLICIT point ids (automation / tests).
    pub fn sketch3d_add_constraint_over(&mut self, ctype: &str, points: Vec<Value>, value: Option<f64>) -> Result<Value, String> {
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return Err("not in 3D sketch mode".into());
        };
        edit.record_undo();
        let id = edit.session.doc.add_constraint(ctype, points, value);
        if let Err(error) = edit.session.resolve() {
            if let Some(snap) = edit.undo_stack.pop() {
                edit.restore(snap);
            }
            return Err(error);
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(id)
    }

    /// The palette for the current selection, as JSON.
    pub fn sketch3d_applicable_constraints_json(&self) -> String {
        self.sketch3d_edit
            .as_ref()
            .and_then(|edit| serde_json::to_string(&edit.session.applicable_constraints()).ok())
            .unwrap_or_else(|| "[]".to_string())
    }

    /// Set a dimensional constraint's value (a number, or an expression
    /// string kept as `valueExpr` and evaluated now against the document's
    /// expressions). Re-solves; an unsolvable value is taken back.
    pub fn sketch3d_set_dimension_value(&mut self, id: &Value, value: &str) -> Result<(), String> {
        let trimmed = value.trim();
        let number: f64 = match trimmed.parse::<f64>() {
            Ok(n) => n,
            Err(_) => brep_kernel::eval_expression(
                &self.history.expressions(),
                &self.history.configurator(),
                trimmed,
            )
            .map_err(|e| format!("dimension `{trimmed}`: {e}"))?,
        };
        if !number.is_finite() {
            return Err(format!("dimension `{trimmed}` is not finite"));
        }
        let Some(edit) = self.sketch3d_edit.as_mut() else {
            return Err("not in 3D sketch mode".into());
        };
        edit.record_undo();
        {
            let key = brep_kernel::sketch_id_key(id);
            let Some(constraint) = edit
                .session
                .doc
                .constraints
                .iter_mut()
                .find(|c| c.id().map(brep_kernel::sketch_id_key) == Some(key.clone()))
            else {
                edit.undo_stack.pop();
                return Err(format!("no constraint with id {key}"));
            };
            constraint.raw.insert("value".into(), Value::from(number));
            if trimmed.parse::<f64>().is_ok() {
                constraint.raw.remove("valueExpr");
            } else {
                constraint.raw.insert("valueExpr".into(), Value::String(trimmed.to_string()));
            }
        }
        if let Err(error) = edit.session.resolve() {
            if let Some(snap) = edit.undo_stack.pop() {
                edit.restore(snap);
            }
            return Err(error);
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        Ok(())
    }

    /// Toggle the `construction` flag of every selected geometry.
    pub fn sketch3d_toggle_construction(&mut self) {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        let (_, geometries, _) = edit.session.selection_by_kind();
        if geometries.is_empty() {
            return;
        }
        edit.record_undo();
        for gid in &geometries {
            if let Some(g) = edit.session.doc.geometry_mut(gid) {
                if g.geom_type == "plane" {
                    continue;
                }
                let now = g.construction();
                g.extra.insert("construction".into(), Value::Bool(!now));
            }
        }
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    /// Toggle the `fixed` flag of every selected point (a linked reference
    /// point stays fixed).
    pub fn sketch3d_toggle_fixed(&mut self) {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        let (points, _, _) = edit.session.selection_by_kind();
        if points.is_empty() {
            return;
        }
        edit.record_undo();
        for pid in &points {
            if let Some(p) = edit.session.doc.point_mut(pid) {
                if p.external_reference {
                    continue;
                }
                p.fixed = !p.fixed;
            }
        }
        let _ = edit.session.resolve();
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    /// Delete the selection: constraints by id; geometries with every
    /// constraint touching their points; points with their geometries and
    /// constraints. Orphaned points no geometry or constraint uses are swept
    /// away, and reference records whose entities are gone are dropped.
    pub fn sketch3d_delete_selection(&mut self) {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return };
        let (mut points, mut geometries, constraints) = edit.session.selection_by_kind();
        if points.is_empty() && geometries.is_empty() && constraints.is_empty() {
            return;
        }
        edit.record_undo();
        let key = brep_kernel::sketch_id_key;
        // A linked reference is one thing: deleting any of its points or its
        // construction geometry unlinks the whole record, the way the
        // References list's ✕ does, so no fixed reference point is left
        // behind that nothing can move.
        let mut taken: Vec<Value> = Vec::new();
        for record in &edit.external_refs {
            let ids: Vec<Value> = record.get("points").and_then(Value::as_array).cloned().unwrap_or_default();
            let gid = record.get("geometry").filter(|g| !g.is_null()).cloned();
            let hit = ids.iter().any(|id| points.iter().any(|p| key(p) == key(id)))
                || gid.as_ref().map_or(false, |g| geometries.iter().any(|s| key(s) == key(g)));
            if hit {
                for id in ids {
                    if !points.iter().any(|p| key(p) == key(&id)) {
                        points.push(id);
                    }
                }
                if let Some(g) = gid {
                    if !geometries.iter().any(|s| key(s) == key(&g)) {
                        geometries.push(g);
                    }
                }
                taken.push(record.clone());
            }
        }
        edit.external_refs.retain(|r| !taken.iter().any(|t| t == r));
        let doc = &mut edit.session.doc;
        let ckeys: Vec<String> = constraints.iter().map(key).collect();
        doc.constraints.retain(|c| c.id().map_or(true, |id| !ckeys.contains(&key(id))));
        // Points to remove: the selected ones plus every point owned ONLY by a
        // deleted geometry (its interior / endpoints no other geometry shares
        // and no constraint uses, swept below).
        let gkeys: Vec<String> = geometries.iter().map(key).collect();
        let mut pkeys: Vec<String> = points.iter().map(key).collect();
        doc.geometries.retain(|g| !gkeys.contains(&key(&g.id)));
        // A geometry touching a removed point goes with it.
        doc.geometries.retain(|g| !g.points.iter().any(|pid| pkeys.contains(&key(pid))));
        doc.points.retain(|p| !pkeys.contains(&key(&p.id)));
        // Sweep orphans: a non-reference point no geometry uses any more (a
        // deleted line's free end). Every bare point is wrapped by a `point`
        // geometry, so a point without one is nothing the user can see.
        let used: std::collections::HashSet<String> = doc
            .geometries
            .iter()
            .flat_map(|g| g.points.iter().map(key))
            .collect();
        let orphans: Vec<String> = doc
            .points
            .iter()
            .filter(|p| !p.external_reference && !used.contains(&key(&p.id)))
            .map(|p| key(&p.id))
            .collect();
        doc.points.retain(|p| !orphans.contains(&key(&p.id)));
        pkeys.extend(orphans);
        // Constraints touching a removed point go with it.
        doc.constraints.retain(|c| !c.points().iter().any(|pid| pkeys.contains(&key(pid))));
        // Reference records whose geometry or points are gone.
        edit.external_refs.retain(|r| {
            let ids: Vec<Value> = r.get("points").and_then(Value::as_array).cloned().unwrap_or_default();
            let gid = r.get("geometry").filter(|g| !g.is_null());
            let points_alive = ids.iter().all(|id| doc.point(id).is_some());
            let geometry_alive = gid.map_or(true, |g| doc.geometry(g).is_some());
            points_alive && geometry_alive
        });
        edit.session.selection.clear();
        edit.session.hovered = None;
        edit.pending.clear();
        let _ = edit.session.resolve();
        self.refresh_sketch3d_overlay();
        self.dirty = true;
    }

    pub fn sketch3d_undo(&mut self) -> bool {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return false };
        let Some(snap) = edit.undo_stack.pop() else { return false };
        let current = edit.snapshot();
        edit.redo_stack.push(current);
        edit.restore(snap);
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        true
    }

    pub fn sketch3d_redo(&mut self) -> bool {
        let Some(edit) = self.sketch3d_edit.as_mut() else { return false };
        let Some(snap) = edit.redo_stack.pop() else { return false };
        let current = edit.snapshot();
        edit.undo_stack.push(current);
        edit.restore(snap);
        self.refresh_sketch3d_overlay();
        self.dirty = true;
        true
    }

    pub fn sketch3d_can_undo(&self) -> bool {
        self.sketch3d_edit.as_ref().map_or(false, |edit| !edit.undo_stack.is_empty())
    }

    pub fn sketch3d_can_redo(&self) -> bool {
        self.sketch3d_edit.as_ref().map_or(false, |edit| !edit.redo_stack.is_empty())
    }
}

fn fmt_coord(value: f64) -> String {
    let text = format!("{value:.6}");
    let text = text.trim_end_matches('0').trim_end_matches('.').to_string();
    if text.is_empty() || text == "-0" {
        "0".to_string()
    } else {
        text
    }
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Classify a display edge polyline: a straight run → a line through its
/// ends; a run whose samples sit at one radius about a fitted center → an arc
/// `[center, start, end]`, or, closed, the circle's axis as a line; anything
/// else → its chord, the same best effort the kernel makes.
fn classify_edge_polyline(polyline: &[[f64; 3]]) -> Option<brep_kernel::Sketch3dResolvedReference> {
    use brep_kernel::{Sketch3dResolvedReference as R, Vec3, SKETCH3D_REFERENCE_TIP_LENGTH as TIP};
    let v = |a: [f64; 3]| Vec3::new(a[0], a[1], a[2]);
    if polyline.len() < 2 {
        return None;
    }
    let start = v(polyline[0]);
    let end = v(*polyline.last()?);
    let extent = polyline
        .iter()
        .map(|p| v(*p).sub(start).length())
        .fold(0.0f64, f64::max)
        .max(1e-12);
    let chord = end.sub(start);
    // Straight: every sample within tolerance of the chord.
    if chord.length() > 1e-9 * extent {
        let d = chord.normalized().ok()?;
        let off = polyline
            .iter()
            .map(|p| {
                let w = v(*p).sub(start);
                w.sub(d.scale(d.dot(w))).length()
            })
            .fold(0.0f64, f64::max);
        if off <= 1e-6 * extent {
            return Some(R::Line(start, end));
        }
    }
    // Circular: fit the circle through three well-spread samples and check
    // every sample's distance from its centre.
    if polyline.len() >= 3 {
        let a = start;
        let b = v(polyline[polyline.len() / 2]);
        let c = if chord.length() > 1e-9 * extent { end } else { v(polyline[polyline.len() / 4]) };
        if let Some((center, radius, axis)) = circle_through(a, b, c) {
            let worst = polyline
                .iter()
                .map(|p| (v(*p).sub(center).length() - radius).abs())
                .fold(0.0f64, f64::max);
            let planar = polyline
                .iter()
                .map(|p| v(*p).sub(center).dot(axis).abs())
                .fold(0.0f64, f64::max);
            if worst <= 1e-4 * radius && planar <= 1e-4 * radius {
                if chord.length() <= 1e-6 * extent {
                    return Some(R::Line(center, center.add(axis.scale(radius.max(TIP)))));
                }
                return Some(R::Arc(center, start, end));
            }
        }
    }
    (chord.length() > 1e-9 * extent).then_some(R::Line(start, end))
}

/// The circle through three points: `(center, radius, unit axis)`.
fn circle_through(a: brep_kernel::Vec3, b: brep_kernel::Vec3, c: brep_kernel::Vec3) -> Option<(brep_kernel::Vec3, f64, brep_kernel::Vec3)> {
    let ab = b.sub(a);
    let ac = c.sub(a);
    let n = ab.cross(ac);
    let n2 = n.dot(n);
    if n2 <= 1e-24 {
        return None;
    }
    // Circumcenter: a + ((|ac|² ab − |ab|² ac) × n) / (2 |n|²)
    let term = ab.scale(ac.dot(ac)).sub(ac.scale(ab.dot(ab))).cross(n).scale(1.0 / (2.0 * n2));
    let center = a.add(term);
    let radius = a.sub(center).length();
    let axis = n.normalized().ok()?;
    Some((center, radius, axis))
}
