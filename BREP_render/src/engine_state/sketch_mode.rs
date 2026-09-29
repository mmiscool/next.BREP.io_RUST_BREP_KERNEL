use super::*;

/// One in-flight sketch drag (S2): the grabbed points and the anchor uv. A single
/// entry for a point grab; all of a geometry's (deduped) points for a geometry
/// grab (a rigid translate). Each point tracks the cursor by an ABSOLUTE
/// `orig + (uv - anchor)` (never incremental — the solver moves points between
/// frames, so incremental deltas would drift).
#[derive(Clone)]
pub struct SketchDrag {
    /// Per grabbed point: `(id, orig_x, orig_y, orig_fixed)` captured once at grab.
    pub(super) points: Vec<(serde_json::Value, f64, f64, bool)>,
    /// The plane `(u, v)` where the grab started — the anchor for the delta.
    pub(super) anchor: (f64, f64),
}

/// The active engine-native sketch edit — the live session plus the state to
/// restore when sketch mode exits.
pub struct SketchEdit {
    /// The live solved sketch session (the doc being edited + its plane).
    pub session: crate::sketch::SketchSession,
    /// The id of the `"S"` feature this edit belongs to.
    pub feature_id: String,
    /// The rolled-to step to restore on exit (the model state before entering).
    prev_rollback: usize,
    /// The camera to restore on exit (the vantage before the plane orient).
    prev_camera: crate::view::ViewCamera,
    /// Whether this sketch was created THIS edit (so a cancel deletes it).
    is_new: bool,
    /// For a sketch created by this session: the undo step its creation took
    /// (`History::undo_top` right after the push). Finish writes into that
    /// step and Cancel retracts it, while nothing else has taken a step since,
    /// so a new sketch is one undo step and a cancelled one is none.
    creation_step: Option<u64>,
    /// The in-flight drag (S2): the grabbed points + anchor, or `None` when no drag
    /// is live. Grabs a point OR a whole geometry (rigid translate). See [`SketchDrag`].
    pub(super) drag: Option<SketchDrag>,
    /// The in-progress draw-tool click buffer (S3a): the point ids placed so far for
    /// the geometry being drawn (a line-chain's running start, a rect/circle/arc's
    /// first clicks). Cleared when a geometry lands, the tool changes, or on cancel.
    pub(super) pending: Vec<serde_json::Value>,
    /// The last hovered plane `(u, v)` (S3a) — the anchor for the rubber-band preview
    /// so it follows the cursor while a tool has pending clicks.
    pub(super) hover_uv: Option<(f64, f64)>,
    /// Per-session UNDO stack (S6a): snapshots of the visible edit state taken at the
    /// START of each discrete mutating op. Separate from the model-level undo (which
    /// owns the FEATURE document). Cleared implicitly on enter/exit (a fresh
    /// `SketchEdit` starts empty; exit drops the whole struct).
    pub(super) undo_stack: Vec<SketchSnapshot>,
    /// Per-session REDO stack (S6a): states popped by `sketch_undo`, restored by
    /// `sketch_redo`. Cleared whenever a new mutation records an undo snapshot.
    pub(super) redo_stack: Vec<SketchSnapshot>,
    /// First-move guard for a dimension-label DRAG (S6a): set when the drag's single
    /// undo snapshot has been taken, reset at the gesture end
    /// ([`sketch_dimension_drag_end`](EngineState::sketch_dimension_drag_end)) so the
    /// whole drag is ONE undo step, not one per motion frame.
    pub(super) dim_drag_snapshotted: bool,
    /// The external-reference edge links (S6b-2): one [`crate::sketch::ExternalRef`]
    /// per linked scene edge (by name), mapping it to the materialized sketch points +
    /// construction geometry. Loaded from `persistentData.externalRefs` on enter,
    /// written back on commit, and captured in the undo snapshot so undo restores it
    /// alongside the doc.
    pub(super) external_refs: Vec<crate::sketch::ExternalRef>,
    /// The in-progress freehand stroke (S6b-3): raw plane `(u, v)` samples captured
    /// during a handdraw drag, recognized into geometry on drag-end. Empty when no
    /// stroke is live. Transient interaction state — NOT part of the undo snapshot.
    pub(super) handdraw_stroke: Vec<(f64, f64)>,
}

/// One reversible sketch-edit state (S6a): a clone of everything the visible edit
/// carries — the solved [`SketchDoc`](crate::sketch::SketchDoc), the dimension label
/// offsets, and the selection — enough to fully restore the session on undo/redo.
/// Transient interaction state (in-flight drag, draw pending buffer) is deliberately
/// NOT captured; undo/redo clears it instead.
#[derive(Clone)]
pub(super) struct SketchSnapshot {
    pub(super) doc: crate::sketch::SketchDoc,
    pub(super) dim_offsets: serde_json::Map<String, serde_json::Value>,
    pub(super) selection: Vec<serde_json::Value>,
    /// The external-reference mapping (S6b-2) — captured so undoing a pickEdges link
    /// also drops its `ExternalRef` entry, keeping the mapping consistent with the doc.
    pub(super) external_refs: Vec<crate::sketch::ExternalRef>,
}

impl SketchEdit {
    /// A snapshot of the CURRENT visible edit state (doc + dim offsets + selection +
    /// external refs).
    pub(super) fn snapshot(&self) -> SketchSnapshot {
        SketchSnapshot {
            doc: self.session.doc.clone(),
            dim_offsets: self.session.dim_offsets.clone(),
            selection: self.session.selection.clone(),
            external_refs: self.external_refs.clone(),
        }
    }

    /// Push the current state onto the undo stack and clear redo — the discrete-op
    /// undo primitive. Callers invoke this at the START of a mutation (before the doc
    /// changes) so one Ctrl+Z reverts that op.
    pub(super) fn record_undo(&mut self) {
        let snap = self.snapshot();
        self.undo_stack.push(snap);
        self.redo_stack.clear();
    }
}

/// Convert a kernel [`brep_kernel::Frame`] into the sketch module's
/// [`crate::sketch::PlaneFrame`]. Kept HERE (not in `sketch/mod.rs`, which is
/// deliberately kernel-free) so a sketch's kernel-resolved plane frame — the exact
/// frame the committed sheet is materialized against — can seed the live session.
fn plane_frame_from_kernel(frame: &brep_kernel::Frame) -> crate::sketch::PlaneFrame {
    crate::sketch::PlaneFrame {
        origin: [frame.origin.x, frame.origin.y, frame.origin.z],
        x_axis: [frame.x_axis.x, frame.x_axis.y, frame.x_axis.z],
        y_axis: [frame.y_axis.x, frame.y_axis.y, frame.y_axis.z],
        z_axis: [frame.z_axis.x, frame.z_axis.y, frame.z_axis.z],
    }
}

impl EngineState {
    /// True while a sketch is being edited (the shell hides the normal side panel
    /// and shows the sketch-mode bar).
    pub fn sketch_mode(&self) -> bool {
        self.sketch_edit.is_some()
    }

    /// The live sketch session while in sketch mode (for the DOF readout / overlay
    /// / the headless verifier), else `None`.
    pub fn sketch_edit_session(&self) -> Option<&crate::sketch::SketchSession> {
        self.sketch_edit.as_ref().map(|edit| &edit.session)
    }

    /// The id of the feature being edited while in sketch mode, else `None`.
    pub fn sketch_edit_feature_id(&self) -> Option<&str> {
        self.sketch_edit.as_ref().map(|edit| edit.feature_id.as_str())
    }

    /// Whether the sketch camera is locked flat to the plane (only panning). On by
    /// default every sketch entry; the sketch-mode bar's checkbox reflects this.
    pub fn sketch_camera_locked(&self) -> bool {
        self.sketch_camera_locked
    }

    /// Toggle the sketch camera lock. Turning it ON re-faces the camera to the
    /// current sketch plane (so "off → spin around → on" snaps back flat); turning
    /// it OFF just frees orbiting. No-op outside sketch mode.
    pub fn toggle_sketch_camera_lock(&mut self) {
        if !self.sketch_mode() {
            return;
        }
        self.sketch_camera_locked = !self.sketch_camera_locked;
        if self.sketch_camera_locked {
            // `PlaneFrame` is `Copy`, so this ends the `&self` borrow before the
            // `&mut self` re-orient.
            if let Some(plane) = self.sketch_edit_session().map(|s| s.plane) {
                self.face_camera_to_plane(&plane);
                self.dirty = true;
            }
        }
    }

    /// Orient the camera flat-on to a sketch plane: look along the NEGATIVE normal
    /// with the plane's +y as up, so the plane faces the viewer. Shared by sketch
    /// entry and the lock toggle.
    fn face_camera_to_plane(&mut self, plane: &crate::sketch::PlaneFrame) {
        let dir = [
            -(plane.z_axis[0] as f32),
            -(plane.z_axis[1] as f32),
            -(plane.z_axis[2] as f32),
        ];
        let up = [
            plane.y_axis[0] as f32,
            plane.y_axis[1] as f32,
            plane.y_axis[2] as f32,
        ];
        self.apply_look_direction(dir, up);
    }

    /// Enter sketch mode for the `"S"` feature with id `feature_id`: snapshot the
    /// camera + rolled-to step, roll to the step BEFORE the sketch (its backdrop),
    /// read the persisted plane `basis` + `sketch` doc off the history JSON, solve
    /// a live session, orient the camera onto the plane, and push the read-only
    /// overlay. Returns the session's diagnostics JSON. Errors when the feature is
    /// absent or is not a sketch.
    pub fn enter_sketch_mode(&mut self, feature_id: &str) -> Result<String, String> {
        let index = self
            .history
            .index_of(feature_id)
            .ok_or_else(|| format!("no feature with id '{feature_id}'"))?;
        if self.history.feature_type(index).as_deref() != Some("S") {
            return Err(format!("feature '{feature_id}' is not a sketch (type \"S\")"));
        }

        // The frame the kernel resolved for THIS sketch on the last full run is
        // published under the sketch's own id in `construction_frames` (see
        // `feature_pipeline::features::sketch::execute` → `result.frames`). It is, by
        // construction, the EXACT frame the committed sheet was materialized against
        // (the sheet's world curves come from the same `SketchProfile` frame). Capture
        // it NOW, before the roll below drops the sketch from `construction_frames` —
        // seeding the live session with it keeps the editing overlay on the same plane
        // the committed sheet lands on. This fixes the off-location sheet for
        // face/datum-attached sketches whose live-resolved frame differs from the
        // persisted `basis` (which the kernel only uses as a missing-reference fallback).
        let resolved_frame = self
            .construction_frames
            .iter()
            .find(|(name, _)| name == feature_id)
            .map(|(_, frame)| plane_frame_from_kernel(frame));

        // Snapshot the pre-entry view + roll, then roll to the step just before the
        // sketch so the model up to (not including) it is the sketching backdrop.
        let prev_camera = self.camera.clone();
        let prev_rollback = self.history.rollback();
        self.history.set_rollback(index.saturating_sub(1));
        self.rerun_history();

        // Read the persisted plane + doc straight off the history JSON (headless).
        // The plane is the kernel-resolved frame when available (live-first, matching
        // the SKETCH feature's `resolve_frame`), else the persisted `basis`, else XY.
        let persistent = self.history.feature_persistent_data(index);
        let plane = resolved_frame
            .or_else(|| {
                persistent
                    .as_ref()
                    .and_then(|p| p.get("basis"))
                    .map(crate::sketch::PlaneFrame::from_basis_json)
            })
            .unwrap_or_else(crate::sketch::PlaneFrame::xy);
        let doc_value = persistent
            .as_ref()
            .and_then(|p| p.get("sketch"))
            .cloned()
            .unwrap_or_else(|| {
                serde_json::json!({ "points": [], "geometries": [], "constraints": [] })
            });
        let mut doc: crate::sketch::SketchDoc = serde_json::from_value(doc_value)
            .map_err(|error| format!("sketch doc parse: {error}"))?;
        // Every sketch carries the part origin projected onto its plane, as a
        // fixed reference to constrain to (added on first entry, re-projected
        // when the plane moved).
        crate::sketch::external_ref::ensure_origin_point(&mut doc, &plane);
        let mut session = crate::sketch::SketchSession::new(doc, plane)?;
        // Seed the overlay palette from the live display settings so the sketch colors
        // are managed there like the rest of the display (kept in sync by
        // `apply_settings_json` whenever the user edits a sketch color).
        session.colors = self.settings.sketch_colors();
        // Load the persisted per-dimension label offsets (S5) — plane-space
        // `{du, dv}` keyed by constraint id, written back on commit below.
        if let Some(offsets) = persistent
            .as_ref()
            .and_then(|p| p.get("dimOffsets"))
            .and_then(serde_json::Value::as_object)
        {
            session.dim_offsets = offsets.clone();
        }
        // Load the persisted external-reference edge links (S6b-2) — the mapping from
        // linked scene edges to their materialized points/geometry, round-tripped on
        // commit below. A malformed array degrades to no refs.
        let mut external_refs: Vec<crate::sketch::ExternalRef> = persistent
            .as_ref()
            .and_then(|p| p.get("externalRefs"))
            .and_then(|v| serde_json::from_value(v.clone()).ok())
            .unwrap_or_default();
        // Heal docs poisoned by the pre-fix delete: drop any linked-edge entry whose
        // materialized points/geometry are gone, so its edge can be RE-LINKED (a stale
        // entry's dead dedup path would otherwise block the re-link forever). Reproject
        // (below) then only touches live refs.
        crate::sketch::external_ref::prune_dead_refs(&session.doc, &mut external_refs);

        // Drop any MODEL selection carried in from picking the sketch plane, so no
        // faces/edges stay highlighted inside the sketch — the only highlight the
        // user should see there is the sketch's own geometry (or an entity they
        // pick to link in). Covers the new-sketch path too (`new_sketch` enters
        // through here).
        self.clear_selection();

        // Every sketch entry starts with the camera LOCKED flat to the plane (the
        // default): face it now, and `face_camera_to_plane` re-does this when the
        // user toggles the lock back on.
        self.sketch_camera_locked = true;
        self.face_camera_to_plane(&plane);

        // Push the overlay + capture diagnostics BEFORE moving the session into the
        // edit (both are borrows of the local `session`).
        self.set_sketch_overlay(&session);
        let diagnostics =
            serde_json::to_string(&session.diagnostics).unwrap_or_else(|_| "{}".to_string());

        self.sketch_edit = Some(SketchEdit {
            session,
            feature_id: feature_id.to_string(),
            prev_rollback,
            prev_camera,
            is_new: false,
            creation_step: None,
            drag: None,
            pending: Vec::new(),
            hover_uv: None,
            // A fresh session starts with empty undo/redo history (S6a).
            undo_stack: Vec::new(),
            redo_stack: Vec::new(),
            dim_drag_snapshotted: false,
            external_refs,
            handdraw_stroke: Vec::new(),
        });
        // Re-project the loaded external refs against the CURRENT backdrop model so a
        // linked edge tracks upstream geometry edits (best-effort; a no-op when the
        // edge is absent — the persisted coords then stand as the fallback). This may
        // move the ref points + re-solve, so re-read diagnostics afterward.
        let reprojected = self.sketch_reproject_external_refs();
        let diagnostics = if reprojected {
            self.sketch_edit
                .as_ref()
                .map(|edit| {
                    serde_json::to_string(&edit.session.diagnostics)
                        .unwrap_or_else(|_| "{}".to_string())
                })
                .unwrap_or(diagnostics)
        } else {
            diagnostics
        };
        // Clear the now-active sketch's committed overlay: the mid-enter rerun above
        // (rolled to the step before the sketch, `sketch_edit` not yet set) may have
        // fed it; now `sketch_edit` is armed, so it is excluded and cleared — the live
        // editing overlay is its only display while editing (no double display).
        self.refresh_committed_sketches();
        self.dirty = true;
        Ok(diagnostics)
    }

    /// Exit sketch mode. `commit` writes the edited doc back to the feature's
    /// `persistentData.sketch`; a non-commit exit of a brand-new sketch DELETES the
    /// feature (a fresh sketch nobody kept). Restores the snapshotted camera +
    /// rolled-to step, clears the overlay, re-runs the history, and returns the
    /// build report. A no-op (returns `"{}"`) when not in sketch mode.
    pub fn exit_sketch_mode(&mut self, commit: bool) -> String {
        let Some(edit) = self.sketch_edit.take() else {
            return "{}".to_string();
        };
        if commit {
            if let Some(index) = self.history.index_of(&edit.feature_id) {
                let mut sketch_value = serde_json::to_value(&edit.session.doc).unwrap_or_else(|_| {
                    serde_json::json!({ "points": [], "geometries": [], "constraints": [] })
                });
                // Stamp each closed loop's STABLE id onto its geometries. The
                // kernel derives the same ids when it reads the sketch, so this
                // changes no name — it persists the id against the one edit
                // deriving cannot survive: deleting the edge the id came from.
                // Per-loop face names (extrude/sweep/revolve/loft caps) embed it.
                brep_kernel::assign_sketch_loop_ids(&mut sketch_value);
                // The dimension label offsets persist alongside the doc (S5) so a
                // re-enter restores each dimension's dragged label position, and
                // the external-reference edge links (S6b-2) so a re-enter reloads
                // (and re-projects) each linked scene edge.
                let refs_value = serde_json::to_value(&edit.external_refs)
                    .unwrap_or_else(|_| serde_json::Value::Array(Vec::new()));
                // One write, so Finish is ONE undo step (and none when the
                // session changed nothing). A sketch this session created
                // writes into its creation's step, so the whole new sketch is
                // one step.
                let fields = vec![
                    ("sketch", sketch_value),
                    ("dimOffsets", serde_json::Value::Object(edit.session.dim_offsets.clone())),
                    ("externalRefs", refs_value),
                ];
                match edit.creation_step {
                    Some(step) => {
                        self.history.set_feature_persistent_fields_in_step(index, fields, step);
                    }
                    None => {
                        self.history.set_feature_persistent_fields(index, fields);
                    }
                }
            }
        } else if edit.is_new {
            // Retract the creation's step, so the cancel leaves nothing to
            // undo; only if something else took a step since, delete the
            // feature as a step of its own.
            if edit.creation_step.is_some_and(|step| self.history.retract(step)) {
                self.reinstall_rewound_parts_library();
            } else if let Some(index) = self.history.index_of(&edit.feature_id) {
                self.history.remove_feature(index);
            }
        }
        self.history.set_rollback(edit.prev_rollback);
        self.clear_sketch_overlay();
        self.camera = edit.prev_camera;
        self.dirty = true;
        self.rerun_history()
    }

    /// Create a NEW engine-native sketch on a base plane (`"XY" | "XZ" | "YZ"`) and
    /// enter sketch mode on it. The feature persists an analytic `basis` (computed
    /// via [`crate::sketch::PlaneFrame`]) and an empty `sketch` doc; a cancel exit
    /// deletes it. Returns the new session's diagnostics JSON. Errors on an unknown
    /// plane name.
    pub fn new_sketch(&mut self, plane: &str) -> Result<String, String> {
        let frame = match plane {
            "XY" => crate::sketch::PlaneFrame::xy(),
            "XZ" => crate::sketch::PlaneFrame::xz(),
            "YZ" => crate::sketch::PlaneFrame::yz(),
            other => {
                return Err(format!("unknown base plane '{other}' (expected XY|XZ|YZ)"));
            }
        };
        let id = self
            .history
            .next_feature_id(&crate::features::feature_short_name("S"));
        let feature = serde_json::json!({
            "type": "S",
            "inputParams": { "id": id, "sketchPlane": plane },
            "persistentData": {
                "basis": {
                    "origin": frame.origin,
                    "x": frame.x_axis,
                    "y": frame.y_axis,
                    "z": frame.z_axis,
                },
                "sketch": { "points": [], "geometries": [], "constraints": [] }
            }
        });
        self.history.push_feature(feature);
        let creation_step = self.history.undo_top();
        let diagnostics = self.enter_sketch_mode(&id)?;
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.is_new = true;
            edit.creation_step = creation_step;
        }
        Ok(diagnostics)
    }
}
