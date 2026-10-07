//! PMI — the engine half of the PMI workbench: the document's `pmi` block
//! (checkpointed edits + re-run, like `wire_harness_ops`), the ACTIVE view
//! (applied camera / visibility / wireframe / explode poses, restored
//! exactly on deactivate), annotation CRUD, the coalesced label drag, and the
//! reference picker's PMI flavour.
//!
//! Persisted vs engine memory: the `pmi` block (views, annotations, label
//! positions) lives in the document and rides its undo stack; WHICH view is
//! active and WHICH annotation's form is open are engine memory — a mode,
//! not model state — so a rerun, a save or an undo never churns on them.
//!
//! A label drag is the one high-frequency PMI edit: it writes the block with
//! a coalesced checkpoint (`pmi:label:{id}` — one undo step per drag) and
//! NEVER re-runs the history: the cached report's label position is patched
//! locally and the overlay re-baked. Every other mutation (a view capture,
//! an annotation add / edit / remove) is its own undo step followed by a
//! re-run, whose tail re-resolves the annotations.

use super::*;
use brep_kernel::{
    PmiAnnotation, PmiCamera, PmiDisplay, PmiGeometry, PmiProjection, PmiReport, PmiState,
    PmiStatus, PmiView,
};
use crate::view::Projection;

/// The modeling state remembered while the PMI workbench is active.
#[derive(Debug, Clone)]
pub struct PmiModelingSnapshot {
    pub camera_json: String,
    pub hidden: Vec<String>,
    pub wireframe: bool,
}

/// A patch the panel applies to one view's name / display state.
#[derive(Debug, Default, Clone, PartialEq)]
pub struct PmiViewPatch {
    pub name: Option<String>,
    pub text_size_pt: Option<f64>,
    pub wireframe: Option<bool>,
    pub hidden: Option<Vec<String>>,
    pub section: Option<Option<brep_kernel::feature_pipeline::pmi::PmiSection>>,
}

/// The **PMI view** dialog's schema — what a view's own form edits: its name,
/// the text size its labels draw at and whether it shows the model in
/// wireframe, plus the two re-capture actions its row menu also offers. Shaped
/// like every other object schema so the shared form view draws it;
/// [`EngineState::pmi_update_view`] applies what it edits.
pub fn pmi_view_schema() -> serde_json::Value {
    serde_json::json!({
        "type": "pmiView",
        "shortName": "VIEW",
        "longName": "PMI view",
        "inputParamsSchema": {
            "name": { "type": "string", "label": "Name", "default_value": "View 1" },
            "textSizePt": { "type": "number", "label": "Text size (pt)", "default_value": brep_kernel::PmiDisplay::default().text_size_pt },
            "wireframe": { "type": "boolean", "label": "Wireframe", "default_value": false },
            "sectionEnabled": { "type": "boolean", "label": "Section view", "default_value": false },
            "sectionAxis": { "type": "options", "label": "Section normal", "options": ["View", "X", "Y", "Z"], "default_value": "View" },
            "sectionOffset": { "type": "number", "label": "Section offset (mm)", "default_value": 0.0 },
            "sectionFlip": { "type": "boolean", "label": "Reverse section", "default_value": false },
            "updateCamera": { "type": "button", "label": "Update camera from the viewport" },
            "updateVisibility": { "type": "button", "label": "Update visibility from the scene" }
        }
    })
}

/// A view's current values in [`pmi_view_schema`]'s shape — the form's seed.
pub fn pmi_view_params(view: &brep_kernel::PmiView) -> serde_json::Value {
    serde_json::json!({
        "name": view.name,
        "textSizePt": view.display.text_size_pt,
        "wireframe": view.display.wireframe,
        "sectionEnabled": view.display.section.is_some(),
        "sectionAxis": view.display.section.as_ref().map(|s| s.axis.as_str()).unwrap_or("View"),
        "sectionOffset": view.display.section.as_ref().map(|s| s.offset).unwrap_or(0.),
        "sectionFlip": view.display.section.as_ref().is_some_and(|s| s.flip),
    })
}

/// `{solid}@x,y,z` in WORLD coordinates (the PMI vertex-ref convention).
pub fn world_vertex_ref(solid: &str, position: [f64; 3]) -> String {
    let trim = |v: f64| {
        let rounded = (v * 1e9).round() / 1e9;
        let trimmed = crate::formatting::compact_decimal(rounded, 9);
        if trimmed == "-0" || trimmed.is_empty() { "0".to_string() } else { trimmed }
    };
    format!("{solid}@{},{},{}", trim(position[0]), trim(position[1]), trim(position[2]))
}

impl EngineState {
    // --- read surface ------------------------------------------------------

    /// The document's `pmi` block as typed state (the default — no views —
    /// when the document carries none).
    pub fn pmi_state(&self) -> PmiState {
        self.history
            .pmi_block()
            .and_then(|block| serde_json::from_value(block.clone()).ok())
            .unwrap_or_default()
    }

    /// The PMI report of the last APPLIED run.
    pub fn pmi_report(&self) -> Option<&PmiReport> {
        self.pmi_report.as_ref()
    }

    pub fn pmi_active_view(&self) -> Option<&str> {
        self.pmi_active_view.as_deref()
    }

    pub fn pmi_open_annotation(&self) -> Option<&str> {
        self.pmi_open_annotation.as_deref()
    }

    /// The view whose dialog the PMI pane shows, if one is open.
    pub fn pmi_open_view(&self) -> Option<&str> {
        self.pmi_open_view.as_deref()
    }

    /// How many PMI dialogs — annotation or view — this engine has OPENED. The
    /// app's dialog door reads it beside the open subject so a re-open of the
    /// dialog already open is still an open (the id alone does not move).
    /// Monotonic and never reset; a CLOSE does not count.
    pub fn pmi_dialog_opens(&self) -> u64 {
        self.pmi_dialog_opens
    }

    /// **The PMI half of the dialog door's counter.** Open `id`'s form and
    /// count the open. EVERY path that opens an annotation form goes through
    /// here — the panel's `+`, its row Edit and double-click, the viewport
    /// label double click, the automation add — so the door has one signal to watch
    /// and a re-open of the same annotation still reads as an open. Assigning
    /// `pmi_open_annotation = Some(..)` anywhere else is the bug this exists to
    /// prevent: the form would open with its pane still behind another tab.
    ///
    /// An open annotation is the selected one too — the dialog is about it —
    /// and it replaces an open VIEW dialog, since the pane shows one dialog.
    fn open_pmi_annotation(&mut self, id: String) {
        self.pmi_open_view = None;
        self.pmi_selected_view = None;
        self.pmi_selected_annotation = Some(id.clone());
        self.pmi_open_annotation = Some(id);
        self.pmi_dialog_opens = self.pmi_dialog_opens.wrapping_add(1);
    }

    /// The view half of [`Self::open_pmi_annotation`]: open view `id`'s dialog,
    /// closing an open annotation's, select the view and count the open.
    fn open_pmi_view(&mut self, id: String) {
        self.pmi_open_annotation = None;
        if self.pmi_selected_annotation.take().is_some() {
            self.refresh_pmi_overlay();
        }
        self.pmi_selected_view = Some(id.clone());
        self.pmi_open_view = Some(id);
        self.pmi_dialog_opens = self.pmi_dialog_opens.wrapping_add(1);
    }

    /// The annotation selected in the PMI tree (or opened), while it exists.
    pub fn pmi_selected_annotation(&self) -> Option<&str> {
        let id = self.pmi_selected_annotation.as_deref()?;
        self.pmi_state().find_annotation(id).is_some().then_some(id)
    }

    /// The view selected in the PMI tree (or opened), while it exists.
    pub fn pmi_selected_view(&self) -> Option<&str> {
        let id = self.pmi_selected_view.as_deref()?;
        self.pmi_state().find_view(id).is_some().then_some(id)
    }

    /// Whether the PMI workbench remembered a modeling state (it is "entered").
    pub fn pmi_workbench_entered(&self) -> bool {
        self.pmi_modeling.is_some()
    }

    /// The `__brepPmi` verifier global: the block, the report, the active
    /// view, the open annotation and the datum letters, as one object.
    pub fn pmi_state_json(&self) -> String {
        let state = self.pmi_state();
        serde_json::json!({
            "views": state.views,
            "idCounter": state.id_counter,
            "activeView": self.pmi_active_view,
            "openAnnotation": self.pmi_open_annotation,
            "openView": self.pmi_open_view,
            "selectedAnnotation": self.pmi_selected_annotation(),
            "selectedView": self.pmi_selected_view(),
            "entered": self.pmi_modeling.is_some(),
            "datums": state.datum_letters(),
            "report": self.pmi_report,
        })
        .to_string()
    }

    // --- block writes --------------------------------------------------------

    /// Write `state` as the document's block (checkpointed) and re-run the
    /// history so the tail resolves it. An empty block is removed so a part
    /// that never had PMI saves byte-identically.
    fn write_pmi_state(&mut self, state: PmiState) -> String {
        let block = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_pmi_block(block, None);
        self.rerun_history()
    }

    // --- views ---------------------------------------------------------------

    /// The current camera as a snapshot.
    fn snapshot_camera(&self) -> PmiCamera {
        let camera = &self.camera;
        PmiCamera {
            eye: camera.eye,
            target: camera.target,
            up: camera.up,
            projection: match camera.projection {
                Projection::Orthographic { half_height } => PmiProjection::Orthographic { half_height },
                Projection::Perspective { fov_y_deg } => PmiProjection::Perspective { fov_y_deg },
            },
            viewport: [camera.width, camera.height],
        }
    }

    /// The names of the solids currently hidden in the scene.
    fn hidden_solid_names(&self) -> Vec<String> {
        self.scene
            .solids()
            .iter()
            .filter(|solid| !solid.visible)
            .map(|solid| solid.name.clone())
            .collect()
    }

    /// Capture a view: the current camera, hidden objects and wireframe
    /// setting, named `name` (or `View N`). The new view becomes active.
    pub fn pmi_capture_view(&mut self, name: Option<&str>) -> String {
        let mut state = self.pmi_state();
        let id = state.next_id("VIEW");
        let name = name
            .map(str::trim)
            .filter(|n| !n.is_empty())
            .map(String::from)
            .unwrap_or_else(|| format!("View {}", state.views.len() + 1));
        // The capture reads the MODELING state: if another view is active its
        // applied state is what is on screen, so deactivate first (restores)
        // — a view captures what the user set up, not another view's snapshot.
        let was_active = self.pmi_active_view.is_some();
        if was_active {
            self.pmi_deactivate_view();
        }
        self.pmi_remember_modeling();
        state.views.push(PmiView {
            id: id.clone(),
            name,
            camera: Some(self.snapshot_camera()),
            display: PmiDisplay {
                text_size_pt: 12.0,
                wireframe: self.settings.wireframe,
                hidden: self.hidden_solid_names(),
                section: None,
            },
            annotations: Vec::new(),
        });
        self.write_pmi_state(state);
        let _ = self.pmi_activate_view(&id);
        id
    }

    pub fn pmi_rename_view(&mut self, id: &str, name: &str) -> Result<(), String> {
        let mut state = self.pmi_state();
        let view = state.find_view_mut(id).ok_or_else(|| format!("no PMI view '{id}'"))?;
        let name = name.trim();
        if name.is_empty() {
            return Err("a view needs a name".into());
        }
        view.name = name.to_string();
        self.write_pmi_state(state);
        Ok(())
    }

    /// Delete a view (and its annotations). An active view deactivates first.
    pub fn pmi_delete_view(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.pmi_state();
        let before = state.views.len();
        if self.pmi_active_view.as_deref() == Some(id) {
            self.pmi_deactivate_view();
        }
        state.views.retain(|view| view.id != id);
        if state.views.len() == before {
            return Err(format!("no PMI view '{id}'"));
        }
        if let Some(open) = &self.pmi_open_annotation {
            if state.find_annotation(open).is_none() {
                self.pmi_open_annotation = None;
            }
        }
        if self.pmi_open_view.as_deref() == Some(id) {
            self.pmi_open_view = None;
        }
        self.write_pmi_state(state);
        Ok(())
    }

    /// Re-capture a view's camera from the current camera (the explicit
    /// Update Camera action — orbiting never rewrites a snapshot silently).
    pub fn pmi_update_view_camera(&mut self, id: &str) -> Result<(), String> {
        let mut state = self.pmi_state();
        let camera = self.snapshot_camera();
        let view = state.find_view_mut(id).ok_or_else(|| format!("no PMI view '{id}'"))?;
        view.camera = Some(camera);
        self.write_pmi_state(state);
        Ok(())
    }

    /// Re-capture a view's hidden set from the scene (the explicit Update
    /// Visibility action).
    pub fn pmi_update_view_visibility(&mut self, id: &str) -> Result<(), String> {
        let hidden = self.hidden_solid_names();
        self.pmi_set_view_display(id, &PmiViewPatch { hidden: Some(hidden), ..Default::default() })
    }

    /// Patch a view's name / display state. The active view re-applies.
    pub fn pmi_set_view_display(&mut self, id: &str, patch: &PmiViewPatch) -> Result<(), String> {
        self.patch_pmi_view(id, patch, None)
    }

    /// Apply an edited **PMI view** form ([`pmi_view_schema`]'s params). Only
    /// the fields that differ from the view are written, and a run of edits to
    /// one view is ONE undo step — the form writes on every keystroke of the
    /// name, the way an annotation's form does.
    pub fn pmi_update_view(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let params: serde_json::Value =
            serde_json::from_str(params_json).map_err(|error| format!("view params: {error}"))?;
        let state = self.pmi_state();
        let view = state.find_view(id).ok_or_else(|| format!("no PMI view '{id}'"))?;
        let name = params
            .get("name")
            .and_then(serde_json::Value::as_str)
            .filter(|name| *name != view.name)
            .map(String::from);
        let text_size_pt = params
            .get("textSizePt")
            .and_then(serde_json::Value::as_f64)
            .filter(|size| *size != view.display.text_size_pt);
        let wireframe = params
            .get("wireframe")
            .and_then(serde_json::Value::as_bool)
            .filter(|on| *on != view.display.wireframe);
        let section = if params.get("sectionEnabled").is_some() {
            let section = if params["sectionEnabled"].as_bool().unwrap_or(false) {
                let axis = params["sectionAxis"].as_str().unwrap_or("View");
                let offset = params["sectionOffset"].as_f64().unwrap_or(0.);
                if !["View", "X", "Y", "Z"].contains(&axis) || !offset.is_finite() { return Err("invalid section plane".into()); }
                Some(brep_kernel::feature_pipeline::pmi::PmiSection { axis: axis.into(), offset, flip: params["sectionFlip"].as_bool().unwrap_or(false) })
            } else { None };
            (section != view.display.section).then_some(section)
        } else { None };
        if name.is_none() && text_size_pt.is_none() && wireframe.is_none() && section.is_none() {
            return Ok(());
        }
        let patch = PmiViewPatch { name, text_size_pt, wireframe, hidden: None, section };
        self.patch_pmi_view(id, &patch, Some(&format!("pmi:view:{id}")))
    }

    /// Open view `id`'s dialog (`None` closes it). Opening activates the view,
    /// as opening one of its annotations does: the dialog describes what is on
    /// screen. An id that names no view opens nothing and counts nothing.
    pub fn pmi_set_view_open(&mut self, id: Option<&str>) {
        match id {
            Some(id) => {
                if self.pmi_state().find_view(id).is_none() {
                    return;
                }
                if self.pmi_active_view.as_deref() != Some(id) {
                    let _ = self.pmi_activate_view(id);
                }
                self.open_pmi_view(id.to_string());
            }
            None => self.pmi_open_view = None,
        }
        self.refresh_pmi_overlay();
    }

    fn patch_pmi_view(&mut self, id: &str, patch: &PmiViewPatch, coalesce_key: Option<&str>) -> Result<(), String> {
        let mut state = self.pmi_state();
        let view = state.find_view_mut(id).ok_or_else(|| format!("no PMI view '{id}'"))?;
        if let Some(name) = &patch.name {
            let name = name.trim();
            if name.is_empty() {
                return Err("a view needs a name".into());
            }
            view.name = name.to_string();
        }
        if let Some(size) = patch.text_size_pt {
            view.display.text_size_pt = brep_kernel::clamp_text_size(size);
        }
        if let Some(wireframe) = patch.wireframe {
            view.display.wireframe = wireframe;
        }
        if let Some(hidden) = &patch.hidden {
            view.display.hidden = hidden.clone();
        }
        if let Some(section) = &patch.section { view.display.section = section.clone(); }
        let display = view.display.clone();
        let block = if state.is_empty() { None } else { serde_json::to_value(&state).ok() };
        self.history.set_pmi_block(block, coalesce_key);
        self.rerun_history();
        if self.pmi_active_view.as_deref() == Some(id) {
            self.apply_view_display(&display);
        }
        Ok(())
    }

    /// Remember the modeling state: what is on screen while NO view is
    /// active is the modeling state (hiding a body in the workbench before
    /// capturing is a modeling change, not a view's), so it is re-snapshotted
    /// whenever an activation starts from no active view; while a view is
    /// active the snapshot is kept (switching views restores to it).
    fn pmi_remember_modeling(&mut self) {
        if self.pmi_active_view.is_none() || self.pmi_modeling.is_none() {
            self.pmi_modeling = Some(PmiModelingSnapshot {
                camera_json: self.camera_state_json(),
                hidden: self.hidden_solid_names(),
                wireframe: self.settings.wireframe,
            });
        }
    }

    /// Entering the PMI workbench: remember the modeling camera, visibility
    /// and wireframe so a view activation can be undone exactly.
    pub fn pmi_enter_workbench(&mut self) {
        self.pmi_remember_modeling();
    }

    /// Leaving the PMI workbench: deactivate the view (restoring the modeling
    /// state), close its dialogs, and forget the snapshot. An open SHEET is
    /// not this workbench's to close — sheets are the Drawing workbench's, and
    /// the shell closes the paper when a workbench hides the Sheets pane.
    pub fn pmi_leave_workbench(&mut self) {
        self.pmi_deactivate_view();
        self.pmi_modeling = None;
        self.pmi_open_annotation = None;
        self.pmi_open_view = None;
        self.refresh_pmi_overlay();
    }

    fn apply_view_display(&mut self, display: &PmiDisplay) {
        let names: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
        for name in names {
            let visible = !display.hidden.contains(&name);
            self.scene.set_visible(&name, visible);
        }
        if self.settings.wireframe != display.wireframe {
            self.settings.wireframe = display.wireframe;
            self.settings_generation = self.settings_generation.wrapping_add(1);
        }
        self.dirty = true;
    }

    /// Activate a view: apply its camera (refit to the live viewport), hidden
    /// names, wireframe and explode poses; its annotations become the drawn
    /// and editable set.
    pub fn pmi_activate_view(&mut self, id: &str) -> Result<(), String> {
        let state = self.pmi_state();
        let view = state.find_view(id).ok_or_else(|| format!("no PMI view '{id}'"))?.clone();
        if self.pmi_active_view.as_deref() != Some(id) {
            // Switching views keeps the snapshot: deactivating restores the
            // modeling state, and the new view starts from it.
            let keep = self.pmi_active_view.is_some();
            self.pmi_deactivate_view();
            if !keep {
                self.pmi_remember_modeling();
            }
        }
        if self.pmi_modeling.is_none() {
            self.pmi_remember_modeling();
        }
        self.pmi_active_view = Some(id.to_string());
        if let Some(camera) = &view.camera {
            let (kind, scale) = match camera.projection {
                PmiProjection::Orthographic { half_height } => ("orthographic", half_height),
                PmiProjection::Perspective { fov_y_deg } => ("perspective", fov_y_deg),
            };
            let json = serde_json::json!({
                "kind": kind, "eye": camera.eye, "target": camera.target, "up": camera.up, "scale": scale,
            })
            .to_string();
            let _ = self.apply_camera_state_json(&json);
            self.controls_sync_after_camera_apply();
        }
        self.apply_view_display(&view.display);
        self.pmi_apply_explode();
        if let Some(open) = &self.pmi_open_annotation {
            if !view.annotations.iter().any(|a| a.id() == open) {
                self.pmi_open_annotation = None;
            }
        }
        self.refresh_pmi_overlay();
        Ok(())
    }

    /// Deactivate the active view: restore the explode poses, the modeling
    /// visibility, wireframe and camera. A no-op without an active view.
    pub fn pmi_deactivate_view(&mut self) {
        if self.pmi_active_view.take().is_none() {
            return;
        }
        if self.pmi_state().find_annotation(&self.transform_armed_feature()).is_some() { self.disarm_transform(); }
        self.pmi_restore_explode();
        if let Some(snapshot) = self.pmi_modeling.clone() {
            let names: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
            for name in names {
                let visible = !snapshot.hidden.contains(&name);
                self.scene.set_visible(&name, visible);
            }
            if self.settings.wireframe != snapshot.wireframe {
                self.settings.wireframe = snapshot.wireframe;
                self.settings_generation = self.settings_generation.wrapping_add(1);
            }
            let _ = self.apply_camera_state_json(&snapshot.camera_json);
            self.controls_sync_after_camera_apply();
        }
        self.pmi_open_annotation = None;
        self.dirty = true;
        self.refresh_pmi_overlay();
    }

    /// Keep the arcball controls in step with a camera written wholesale.
    fn controls_sync_after_camera_apply(&mut self) {
        // The controls read the camera each frame (orbit deltas are applied
        // onto `self.camera`), so writing the camera is enough; a fresh depth
        // fit happens on the next render. Nothing else to sync.
        self.dirty = true;
    }

    // --- explode (display-only poses) -----------------------------------------

    /// Pose the active view's explode targets on the CURRENT displays,
    /// keeping the un-posed copies for the restore.
    pub(crate) fn pmi_apply_explode(&mut self) {
        self.pmi_restore_explode();
        let Some(active) = self.pmi_active_view.clone() else {
            return;
        };
        let Some(report) = self.pmi_report.as_ref() else {
            return;
        };
        let Some(view) = report.view(&active) else {
            return;
        };
        let poses: Vec<(Vec<String>, [f64; 3], [f64; 3], [f64; 3], [f64; 3])> = view
            .annotations
            .iter()
            .filter(|row| row.enabled && row.status == PmiStatus::Ok)
            .filter_map(|row| match &row.geometry {
                PmiGeometry::Explode { solids, translate, rotate_deg, scale, center, .. } => {
                    Some((solids.clone(), *translate, *rotate_deg, *scale, *center))
                }
                _ => None,
            })
            .collect();
        for (solids, translate, rotate_deg, scale, center) in poses {
            for name in solids {
                let Some(display) = self.scene.solid(&name) else { continue };
                self.pmi_explode_originals
                    .entry(name.clone())
                    .or_insert_with(|| display.clone());
                if let Some(display) = self.scene.solid_mut(&name) {
                    transform_display(display, center, translate, rotate_deg, scale);
                }
            }
        }
        if let Some((point, normal)) = self.pmi_state().find_view(&active).and_then(|v| v.section_plane()) {
            let names: Vec<String> = self.scene.solids().iter().map(|s| s.name.clone()).collect();
            for name in names {
                if let Some(display) = self.scene.solid(&name) { self.pmi_explode_originals.entry(name.clone()).or_insert_with(|| display.clone()); }
                if let Some(display) = self.scene.solid_mut(&name) { super::pmi_section::clip_display(display, point, normal); }
            }
        }
        if !self.pmi_explode_originals.is_empty() {
            self.dirty = true;
        }
    }

    /// Put every exploded display back exactly.
    pub(crate) fn pmi_restore_explode(&mut self) {
        let originals = std::mem::take(&mut self.pmi_explode_originals);
        for (name, original) in originals {
            if let Some(display) = self.scene.solid_mut(&name) {
                *display = original;
            }
        }
        self.dirty = true;
    }

    /// Post-apply tail (`finish_apply`): the displays are fresh, so re-pose
    /// the active view's explode targets and re-bake the overlay.
    pub(crate) fn pmi_after_apply(&mut self) {
        // The fresh report resolved every balloon's head from its stored bubble.
        self.pmi_balloon_stale = None;
        // Normally already un-posed by `apply_run_output` (before the scene
        // reconcile); the parse-error branch of a rerun reaches here with the
        // poses still applied, so restore rather than forget them.
        self.pmi_restore_explode();
        if let Some(active) = self.pmi_active_view.clone() {
            // A view that vanished (undo of its capture) deactivates.
            let exists = self.pmi_state().find_view(&active).is_some();
            if !exists {
                self.pmi_active_view = None;
                self.pmi_open_annotation = None;
            } else {
                self.pmi_apply_explode();
            }
        }
        if let Some(open) = &self.pmi_open_annotation {
            if self.pmi_state().find_annotation(open).is_none() {
                self.pmi_open_annotation = None;
            }
        }
        // A view dialog whose view went away (undo of its capture) closes.
        if let Some(open) = &self.pmi_open_view {
            if self.pmi_state().find_view(open).is_none() {
                self.pmi_open_view = None;
            }
        }
        self.refresh_pmi_overlay();
    }

    // --- annotations -----------------------------------------------------------

    /// Add an annotation of `type_id` to view `view_id` (the active view when
    /// `None`) with `params_json` (the schema params; `id` is minted). A datum
    /// with no letter gets the next unused one. Returns the id; the new
    /// annotation's form opens.
    pub fn pmi_add_annotation(
        &mut self,
        view_id: Option<&str>,
        type_id: &str,
        params_json: &str,
    ) -> Result<String, String> {
        let def = brep_kernel::pmi_type(type_id).ok_or_else(|| format!("unknown PMI annotation type '{type_id}'"))?;
        let mut state = self.pmi_state();
        let view_id = view_id
            .map(String::from)
            .or_else(|| self.pmi_active_view.clone())
            .ok_or_else(|| "no active PMI view — capture or activate a view first".to_string())?;
        let id = state.next_id(def.short_name);
        let mut params: serde_json::Value = serde_json::from_str(params_json).unwrap_or_else(|_| serde_json::json!({}));
        if !params.is_object() {
            params = serde_json::json!({});
        }
        // Schema defaults under the given params.
        let schema = (def.schema)();
        if let (Some(fields), Some(object)) = (
            schema.get("inputParamsSchema").and_then(serde_json::Value::as_object),
            params.as_object_mut(),
        ) {
            for (key, spec) in fields {
                if !object.contains_key(key) {
                    if let Some(default) = spec.get("default_value") {
                        if !default.is_null() {
                            object.insert(key.clone(), default.clone());
                        }
                    }
                }
            }
            object.insert("id".into(), serde_json::Value::String(id.clone()));
            if type_id == "datum" {
                let letter = object.get("letter").and_then(serde_json::Value::as_str).unwrap_or("").trim().to_string();
                if letter.is_empty() {
                    if let Some(next) = state.next_datum_letter() {
                        object.insert("letter".into(), serde_json::Value::String(next));
                    }
                }
            }
        }
        let view = state.find_view_mut(&view_id).ok_or_else(|| format!("no PMI view '{view_id}'"))?;
        view.annotations.push(PmiAnnotation {
            plugin_replay: None,
            persistent_data: serde_json::Value::Null,
            kind: type_id.to_string(),
            enabled: true,
            params,
            label_world: None,
        });
        self.open_pmi_annotation(id.clone());
        self.write_pmi_state(state);
        Ok(id)
    }

    /// Replace an annotation's params (a form edit). Re-runs.
    pub fn pmi_update_annotation(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        if self.pmi_state().find_annotation(id).is_some_and(|(_, a)| a.kind.contains('/')) {
            let params = serde_json::from_str(params_json).map_err(|e| format!("annotation params: {e}"))?;
            return self.plugin_update_annotation(id, params).map(|_| ());
        }
        self.pmi_update_annotation_no_rerun(id, params_json)?;
        self.rerun_history();
        Ok(())
    }

    /// The fold half of [`Self::pmi_update_annotation`] (checkpointed, no
    /// re-run) — the reference picker's commit uses it before its shared
    /// end tail re-runs.
    pub(crate) fn pmi_update_annotation_no_rerun(&mut self, id: &str, params_json: &str) -> Result<(), String> {
        let mut state = self.pmi_state();
        let annotation = state.find_annotation_mut(id).ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        if annotation.kind.contains('/') {
            return Err("plugin annotation edits require the worker transaction path".into());
        }
        let mut params: serde_json::Value =
            serde_json::from_str(params_json).map_err(|error| format!("annotation params: {error}"))?;
        if let Some(object) = params.as_object_mut() {
            object.insert("id".into(), serde_json::Value::String(id.to_string()));
        }
        annotation.params = params;
        let block = serde_json::to_value(&state).ok();
        self.history.set_pmi_block(block, Some(&format!("pmi:params:{id}")));
        Ok(())
    }

    pub fn pmi_remove_annotation(&mut self, id: &str) -> Result<(), String> {
        if self.pmi_state().find_annotation(id).is_some_and(|(_, a)| a.kind.contains('/')) {
            return self.plugin_delete_annotation(id).map(|_| ());
        }
        let mut state = self.pmi_state();
        let Some((view_index, index)) = state.locate_annotation(id) else {
            return Err(format!("no PMI annotation '{id}'"));
        };
        state.views[view_index].annotations.remove(index);
        if self.pmi_open_annotation.as_deref() == Some(id) {
            self.pmi_open_annotation = None;
        }
        self.write_pmi_state(state);
        Ok(())
    }

    pub fn pmi_set_annotation_enabled(&mut self, id: &str, enabled: bool) -> Result<(), String> {
        if self.pmi_state().find_annotation(id).is_some_and(|(_, a)| a.kind.contains('/')) {
            return self.plugin_set_annotation_enabled(id, enabled).map(|_| ());
        }
        let mut state = self.pmi_state();
        let annotation = state.find_annotation_mut(id).ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        if annotation.enabled == enabled {
            return Ok(());
        }
        annotation.enabled = enabled;
        self.write_pmi_state(state);
        Ok(())
    }

    /// Move an annotation to `index` within its view.
    pub fn pmi_move_annotation(&mut self, id: &str, index: usize) -> Result<(), String> {
        let mut state = self.pmi_state();
        let Some((view_index, from)) = state.locate_annotation(id) else {
            return Err(format!("no PMI annotation '{id}'"));
        };
        let annotations = &mut state.views[view_index].annotations;
        let annotation = annotations.remove(from);
        let to = index.min(annotations.len());
        annotations.insert(to, annotation);
        self.write_pmi_state(state);
        Ok(())
    }

    /// Move an annotation to another view (append).
    pub fn pmi_move_annotation_to_view(&mut self, id: &str, view_id: &str) -> Result<(), String> {
        let mut state = self.pmi_state();
        let Some((view_index, from)) = state.locate_annotation(id) else {
            return Err(format!("no PMI annotation '{id}'"));
        };
        if state.find_view(view_id).is_none() {
            return Err(format!("no PMI view '{view_id}'"));
        }
        let annotation = state.views[view_index].annotations.remove(from);
        state.find_view_mut(view_id).expect("checked").annotations.push(annotation);
        self.write_pmi_state(state);
        Ok(())
    }

    /// Open one annotation's form (engine memory; `None` closes). Opening an
    /// annotation of another view activates that view. An open COUNTS
    /// ([`Self::open_pmi_annotation`]) — including a re-open of the annotation
    /// already open, which is a viewport label double click on the form the user
    /// cannot see; an id that resolves to no annotation opens nothing and
    /// counts nothing.
    pub fn pmi_set_annotation_open(&mut self, id: Option<&str>) {
        match id {
            Some(id) => {
                let state = self.pmi_state();
                if let Some((view, _)) = state.find_annotation(id) {
                    let view_id = view.id.clone();
                    if self.pmi_active_view.as_deref() != Some(view_id.as_str()) {
                        let _ = self.pmi_activate_view(&view_id);
                    }
                    self.open_pmi_annotation(id.to_string());
                }
            }
            None => {
                if self.pmi_open_annotation.as_deref() == Some(self.transform_armed_feature().as_str()) { self.disarm_transform(); }
                self.pmi_open_annotation = None;
            },
        }
        self.refresh_pmi_overlay();
    }

    // --- labels -----------------------------------------------------------------

    /// Move an annotation's label (world). Coalesced per annotation into ONE
    /// undo step, and NEVER a re-run: the cached report is patched in place
    /// and the overlay re-baked. A balloon's arrow head is re-derived from
    /// the moved bubble ([`Self::pmi_reproject_balloon_head`]).
    pub fn pmi_set_label_world(&mut self, id: &str, world: [f64; 3]) -> Result<(), String> {
        let mut state = self.pmi_state();
        let annotation = state.find_annotation_mut(id).ok_or_else(|| format!("no PMI annotation '{id}'"))?;
        annotation.label_world = Some(world);
        let block = serde_json::to_value(&state).ok();
        self.history.set_pmi_block(block, Some(&format!("pmi:label:{id}")));
        if let Some(report) = self.pmi_report.as_mut() {
            if let Some(row) = report.annotation_mut(id) {
                row.label_world = world;
                if let PmiGeometry::Note { position } = &mut row.geometry {
                    *position = world;
                }
            }
        }
        self.pmi_reproject_balloon_head(id);
        self.refresh_pmi_overlay();
        Ok(())
    }

    /// Re-derive a balloon's arrow head from its row's bubble through the
    /// recipe the kernel put on its leader (the same projection the resolver
    /// ran, on the exact-solid clones the scene holds), and patch the cached
    /// report. The head is derived, never stored, so no document edit and no
    /// run: the next run resolves the same head from the stored bubble. A
    /// solid the scene does not hold yet is asked of the runner; the head is
    /// re-derived when the reply lands ([`Self::pmi_refresh_stale_balloon`]),
    /// which for the Inline runner is before this returns. Any other
    /// annotation is left alone.
    pub(crate) fn pmi_reproject_balloon_head(&mut self, id: &str) {
        if let Some(missing) = self.pmi_balloon_head_now(id) {
            self.pmi_balloon_stale = Some(id.to_string());
            self.request_exact_solids(missing.iter().map(String::as_str));
            self.pmi_refresh_stale_balloon();
        }
    }

    /// Retry the balloon whose re-projection waited on a topology reply.
    pub(crate) fn pmi_refresh_stale_balloon(&mut self) {
        let Some(id) = self.pmi_balloon_stale.take() else {
            return;
        };
        if self.pmi_balloon_head_now(&id).is_some() {
            // Still waiting (a background runner's reply is on its way).
            self.pmi_balloon_stale = Some(id);
        } else {
            self.refresh_pmi_overlay();
        }
    }

    /// One re-projection attempt: `None` when done (or not a balloon), else
    /// the occurrence solids the scene does not hold.
    fn pmi_balloon_head_now(&mut self, id: &str) -> Option<Vec<String>> {
        let (anchor, bubble) = self
            .pmi_report
            .as_ref()
            .and_then(|report| report.annotation(id))
            .and_then(|row| match &row.geometry {
                PmiGeometry::Leader { balloon: true, anchor: Some(anchor), .. } => Some((anchor.clone(), row.label_world)),
                _ => None,
            })?;
        let mut missing = Vec::new();
        let head = brep_kernel::pmi_balloon_head(&anchor, bubble, |name| self.scene.exact_solid(name), &mut missing);
        if !missing.is_empty() {
            return Some(missing);
        }
        // A fixed (vertex) anchor answers `None`: the head never moves. A
        // projection error keeps the kernel's head rather than a wrong one.
        if let Ok(Some(head)) = head {
            if let Some(row) = self.pmi_report.as_mut().and_then(|report| report.annotation_mut(id)) {
                if let PmiGeometry::Leader { targets, .. } = &mut row.geometry {
                    if let Some(first) = targets.first_mut() {
                        *first = head;
                    }
                }
            }
        }
        None
    }

    /// Drag an annotation's label to the pointer: the new position is where
    /// the pick ray crosses the plane through the current label perpendicular
    /// to the viewing direction (so a drag never changes the label's depth).
    pub fn pmi_label_drag_to(&mut self, id: &str, x: f64, y: f64) {
        let Some((current, plane)) = self
            .pmi_report
            .as_ref()
            .and_then(|r| r.annotation(id))
            .map(|r| (r.label_world, r.plane))
        else {
            return;
        };
        let ray = self.camera.pick_ray(x, y);
        // In a picked annotation plane the label stays ON that plane; a
        // view-aligned label moves in the view-parallel plane through its
        // current position.
        let plane = plane.unwrap_or_else(|| {
            let (_, _, view) = self.camera.basis();
            brep_kernel::PmiPlane {
                origin: current,
                normal: view,
                x_axis: [0.0; 3],
            }
        });
        let Some(world) = plane.hit(ray.origin, ray.dir) else {
            return;
        };
        let _ = self.pmi_set_label_world(id, world);
    }

    /// A label drag ended: the next drag is a fresh undo step.
    pub fn pmi_label_drag_end(&mut self) {
        self.history.break_coalescing();
    }

    /// Hover a label: highlight the annotation's referenced geometry.
    pub fn pmi_hover(&mut self, id: &str) {
        self.pmi_label_hover_active = true;
        if self.pmi_hovered.as_deref() == Some(id) {
            return;
        }
        let references: Vec<String> = self
            .pmi_report
            .as_ref()
            .and_then(|report| report.annotation(id))
            .map(|row| row.references.clone())
            .unwrap_or_default();
        let mut solids: Vec<String> = Vec::new();
        let mut faces: Vec<String> = Vec::new();
        let mut edges: Vec<String> = Vec::new();
        for reference in &references {
            if let Some(at) = reference.find('@') {
                solids.push(reference[..at].to_string());
            } else if self.scene_has_face(reference) {
                faces.push(reference.clone());
            } else if self.scene_has_edge(reference) {
                edges.push(reference.clone());
            } else if self.scene.solid(reference).is_some() {
                solids.push(reference.clone());
            } else {
                let prefix = format!("{reference}:");
                solids.extend(self.scene.solids().iter().filter(|s| s.name.starts_with(&prefix)).map(|s| s.name.clone()));
            }
        }
        self.clear_hover();
        self.emphasis.hovered_solids.extend(solids);
        self.emphasis.hovered_faces.extend(faces);
        self.emphasis.hovered_edges.extend(edges);
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.pmi_hovered = Some(id.to_string());
        self.dirty = true;
    }

    pub fn pmi_hover_end(&mut self) {
        if self.pmi_hovered.take().is_some() {
            self.clear_hover();
            self.dirty = true;
        }
    }

    /// Consume the one-frame "a PMI label is hovering elements" flag (the
    /// viewport's scene-hover pass yields while set).
    pub fn take_pmi_label_hover(&mut self) -> bool {
        std::mem::take(&mut self.pmi_label_hover_active)
    }

    /// A label was clicked: SELECT that annotation, as its tree row's single
    /// click does — the label carries the accent and nothing opens.
    pub fn pmi_label_clicked(&mut self, id: &str) {
        self.pmi_select_annotation(id);
    }

    /// A label was double-clicked: open that annotation's form, as its tree
    /// row's double click does.
    pub fn pmi_label_double_clicked(&mut self, id: &str) {
        self.pmi_set_annotation_open(Some(id));
    }

    /// SELECT an annotation without opening it — the PMI tree's single click.
    /// Its view activates when it is not the active one, as opening does, so
    /// the selected label is on screen to carry the selection accent. Returns
    /// false, changing nothing, for an id that names no annotation.
    pub fn pmi_select_annotation(&mut self, id: &str) -> bool {
        let state = self.pmi_state();
        let Some((view, _)) = state.find_annotation(id) else {
            return false;
        };
        if self.pmi_active_view.as_deref() != Some(view.id.as_str()) {
            let _ = self.pmi_activate_view(&view.id);
        }
        self.pmi_selected_view = None;
        self.pmi_selected_annotation = Some(id.to_string());
        self.refresh_pmi_overlay();
        true
    }

    /// SELECT a view without activating it — the PMI tree's single click on a
    /// view row. The camera, visibility and wireframe stay as they are: a
    /// double click (which opens the view's dialog) or the row's On toggle is
    /// what activates. A selected annotation gives up the selection. Returns
    /// false, changing nothing, for an id that names no view.
    pub fn pmi_select_view(&mut self, id: &str) -> bool {
        if self.pmi_state().find_view(id).is_none() {
            return false;
        }
        if self.pmi_selected_annotation.take().is_some() {
            self.refresh_pmi_overlay();
        }
        self.pmi_selected_view = Some(id.to_string());
        self.dirty = true;
        true
    }

    // --- reference picker ---------------------------------------------------------

    /// Enter reference-selection mode for annotation `id`'s field at `path`.
    pub fn begin_ref_select_for_pmi(
        &mut self,
        id: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) {
        let restore_index = self.history.rollback();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: id.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::Pmi,
        });
        self.sync_ref_select_emphasis();
    }

    /// Commit a finished PMI ref-select into the annotation's params (fold
    /// only — the caller's shared tail re-runs).
    pub(crate) fn pmi_commit_refs(&mut self, id: &str, path: &[String], names: &[String], multiple: bool) {
        let state = self.pmi_state();
        let Some((_, annotation)) = state.find_annotation(id) else {
            self.push_notice(format!("unknown PMI annotation '{id}'"));
            return;
        };
        let mut params = annotation.params.clone();
        let value = if multiple {
            serde_json::Value::Array(names.iter().cloned().map(serde_json::Value::String).collect())
        } else {
            serde_json::Value::String(names.first().cloned().unwrap_or_default())
        };
        super::selection_ux::set_json_at(&mut params, path, value);
        if let Err(error) = self.pmi_update_annotation_no_rerun(id, &params.to_string()) {
            self.push_notice(format!("PMI update failed: {error}"));
        }
        // A pick is a discrete change (it ends with Finish): the next edit of
        // the same annotation's params is its own undo step, so picking an
        // anchor and then clearing it in the form undo one at a time.
        self.history.break_coalescing();
    }

    // --- import -------------------------------------------------------------------

    /// Merge a file's lifted PMI (`read_step_pmi`) into the document beside
    /// the import that added its geometry: views append with fresh ids, the
    /// import's undo checkpoint covers both (no second checkpoint).
    pub(crate) fn pmi_merge_imported(&mut self, lifted: PmiState) {
        let mut state = self.pmi_state();
        for view in lifted.views {
            let id = state.next_id("VIEW");
            let mut annotations = Vec::with_capacity(view.annotations.len());
            for mut annotation in view.annotations {
                let prefix = brep_kernel::pmi_type(&annotation.kind).map(|def| def.short_name).unwrap_or("PMI");
                let fresh = state.next_id(prefix);
                if let Some(object) = annotation.params.as_object_mut() {
                    object.insert("id".into(), serde_json::Value::String(fresh));
                }
                annotations.push(annotation);
            }
            state.views.push(PmiView {
                id,
                name: view.name,
                camera: view.camera,
                display: view.display,
                annotations,
            });
        }
        let block = serde_json::to_value(&state).ok();
        self.history.set_pmi_block_no_undo(block);
        self.rerun_history();
    }
}

/// Pose a display in place: `p' = R((p − c) ∘ s) + c + t` on the mesh, the
/// edge polylines and the vertices; normals rotate; the bbox is rebuilt.
pub(super) fn transform_display(
    display: &mut crate::scene::SolidDisplay,
    center: [f64; 3],
    translate: [f64; 3],
    rotate_deg: [f64; 3],
    scale: [f64; 3],
) {
    display.revision = crate::scene::next_revision();
    let rotate = |v: [f64; 3]| super::rotate_euler_xyz_f64(v, rotate_deg);
    let pose = |p: [f64; 3]| -> [f64; 3] {
        let local = [(p[0] - center[0]) * scale[0], (p[1] - center[1]) * scale[1], (p[2] - center[2]) * scale[2]];
        let rotated = rotate(local);
        [rotated[0] + center[0] + translate[0], rotated[1] + center[1] + translate[1], rotated[2] + center[2] + translate[2]]
    };
    let mut bbox = crate::camera::Aabb::empty();
    for position in &mut display.mesh.positions {
        let posed = pose([position[0] as f64, position[1] as f64, position[2] as f64]);
        *position = [posed[0] as f32, posed[1] as f32, posed[2] as f32];
        bbox.expand(posed);
    }
    for normal in &mut display.mesh.normals {
        let rotated = rotate(std::array::from_fn(|i| normal[i] as f64 / scale[i]));
        let length = rotated.iter().map(|v| v * v).sum::<f64>().sqrt().max(1e-20);
        *normal = rotated.map(|v| (v / length) as f32);
    }
    for edge in &mut display.edges {
        for point in &mut edge.polyline {
            let posed = pose([point[0] as f64, point[1] as f64, point[2] as f64]);
            *point = [posed[0] as f32, posed[1] as f32, posed[2] as f32];
            bbox.expand(posed);
        }
    }
    for vertex in &mut display.vertices {
        vertex.position = pose(vertex.position);
        bbox.expand(vertex.position);
    }
    if !bbox.is_empty() {
        display.bbox = bbox;
    }
}

