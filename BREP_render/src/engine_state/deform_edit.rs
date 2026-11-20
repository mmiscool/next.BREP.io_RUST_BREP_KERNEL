//! The PICK HANDLES of a Deform Face (`DFF`): the engine side.
//!
//! The kernel reports every pick it resolved in the feature's
//! `persistentData.deform.resolved` (the anchor on the face, the outward
//! normal, the offset, and the point the pick was moved to). While the
//! feature's form is open each target is drawn as a dot, with a line back to
//! its anchor. A click on a dot arms the transform gizmo there; a drag moves
//! the pick ALONG ITS NORMAL: the new offset is the offset at the grab plus the
//! drag's motion projected on the normal (motion across the normal is
//! ignored, so any gizmo axis can be used). The offset is written back where
//! the pick takes it from — the pick's own `offset` in
//! `persistentData.deform.picks` when it has one, else the form's `offset` —
//! and the frames of one drag coalesce into one undo step.

use super::transform_gizmo::TransformPose;
use super::*;
use serde_json::Value;

/// The overlay group the handles draw into.
const OVERLAY_GROUP: &str = "deform-edit";
const LINE_COLOR: [f32; 3] = [0.24, 0.53, 0.77];
const DOT_PLAIN: [f32; 3] = [0.66, 0.83, 0.97];
const DOT_SELECTED: [f32; 3] = [0.435, 0.886, 0.435];
const DOT_SIZE_PX: f32 = 10.0;
/// A viewport click within this many CSS pixels of a dot picks it.
pub const DEFORM_PICK_RADIUS_PX: f64 = 10.0;

/// The gizmo's Deform Face subject: which pick, and the grab snapshot.
#[derive(Clone, Debug, Default)]
pub struct DeformArm {
    pub(super) pick: usize,
    /// At the grab: the pick's offset, its target and its unit normal.
    pub(super) grab: Option<(f64, [f64; 3], [f64; 3])>,
    /// The pose last written during the drag.
    pub(super) live: Option<TransformPose>,
}

/// One resolved pick, as the kernel reported it.
#[derive(Clone, Debug, PartialEq)]
pub struct DeformPickRow {
    pub anchor: [f64; 3],
    pub normal: [f64; 3],
    pub offset: f64,
    pub target: [f64; 3],
}

fn read3(value: &Value) -> Option<[f64; 3]> {
    let array = value.as_array()?;
    let c: Vec<f64> = array.iter().filter_map(Value::as_f64).collect();
    (c.len() == 3).then(|| [c[0], c[1], c[2]])
}

impl EngineState {
    /// Whether `feature_id` is a Deform Face.
    pub fn is_deform_feature(&self, feature_id: &str) -> bool {
        self.history
            .index_of(feature_id)
            .and_then(|index| self.history.feature_type(index))
            .is_some_and(|ty| ty.eq_ignore_ascii_case("DFF"))
    }

    /// The picks the last run of `feature_id` resolved (empty before its
    /// first successful run, or for another feature type).
    pub fn deform_picks(&self, feature_id: &str) -> Vec<DeformPickRow> {
        if !self.is_deform_feature(feature_id) {
            return Vec::new();
        }
        let Some(index) = self.history.index_of(feature_id) else {
            return Vec::new();
        };
        let persistent = self.history.feature_persistent_data(index).unwrap_or(Value::Null);
        let Some(rows) = persistent.pointer("/deform/resolved").and_then(Value::as_array) else {
            return Vec::new();
        };
        rows.iter()
            .filter_map(|row| {
                Some(DeformPickRow {
                    anchor: read3(row.get("anchor")?)?,
                    normal: read3(row.get("normal")?)?,
                    offset: row.get("offset")?.as_f64()?,
                    target: read3(row.get("target")?)?,
                })
            })
            .collect()
    }

    /// The Deform Face whose handles are shown (the history panel feeds it).
    pub fn deform_edit_feature(&self) -> Option<&str> {
        self.deform_edit_feature.as_deref()
    }

    /// The pick the gizmo is armed on, for `feature_id`.
    pub fn deform_armed_pick(&self, feature_id: &str) -> Option<usize> {
        if self.transform_gizmo.feature_id.as_deref() != Some(feature_id) {
            return None;
        }
        self.transform_gizmo.deform.as_ref().map(|arm| arm.pick)
    }

    /// Arm the gizmo on pick `pick` of `feature_id` (at its target). `false`
    /// when there is no such pick.
    pub fn deform_select_pick(&mut self, feature_id: &str, pick: usize) -> bool {
        if pick >= self.deform_picks(feature_id).len() {
            return false;
        }
        self.component_move_reset();
        self.transform_gizmo.feature_id = Some(feature_id.to_string());
        self.transform_gizmo.mode = GizmoMode::Transform;
        self.transform_gizmo.drag = None;
        self.transform_gizmo.anchor = None;
        self.transform_gizmo.port = None;
        self.transform_gizmo.cage = None;
        self.transform_gizmo.deform = Some(DeformArm { pick, grab: None, live: None });
        self.clear_feature_dimension_overlay();
        self.sync_transform_gizmo();
        true
    }

    /// When the handles are shown and one of the dots sits within
    /// [`DEFORM_PICK_RADIUS_PX`] of CSS-pixel `(x, y)`, arm the gizmo on it and
    /// report its index. `None` leaves the click to the ordinary selection.
    pub fn deform_pick_at(&mut self, x: f64, y: f64) -> Option<usize> {
        let feature_id = self.deform_edit_feature.clone()?;
        let mut best: Option<(usize, f64)> = None;
        for (index, row) in self.deform_picks(&feature_id).iter().enumerate() {
            if !self.camera.projectable(row.target) {
                continue;
            }
            let (sx, sy, _) = self.camera.project(row.target);
            let d = ((sx - x).powi(2) + (sy - y).powi(2)).sqrt();
            if d <= DEFORM_PICK_RADIUS_PX && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((index, d));
            }
        }
        let (index, _) = best?;
        self.deform_select_pick(&feature_id, index).then_some(index)
    }

    /// Every handle dot projected to VIEWPORT-LOCAL px, `(index, (x, y))`,
    /// for the host's hit blob. Empty unless handles are shown.
    pub fn deform_pick_screen_points(&self) -> Vec<(usize, (f64, f64))> {
        let Some(feature_id) = self.deform_edit_feature.as_deref() else {
            return Vec::new();
        };
        self.deform_picks(feature_id)
            .into_iter()
            .enumerate()
            .filter(|(_, row)| self.camera.projectable(row.target))
            .map(|(index, row)| {
                let (sx, sy, _) = self.camera.project(row.target);
                (index, (sx, sy))
            })
            .collect()
    }

    /// The armed pick's pose: the live drag pose while a drag is in flight,
    /// else its target with the world frame.
    pub(super) fn deform_pose(&self) -> Option<TransformPose> {
        let arm = self.transform_gizmo.deform.as_ref()?;
        if self.transform_gizmo.drag.is_some() {
            if let Some(live) = arm.live {
                return Some(live);
            }
        }
        let feature_id = self.transform_gizmo.feature_id.as_deref()?;
        let row = self.deform_picks(feature_id).get(arm.pick)?.clone();
        Some(TransformPose { position: row.target, rotation_deg: [0.0; 3], scale: [1.0; 3] })
    }

    /// Snapshot the armed pick at the grab.
    pub(super) fn deform_grab_begin(&mut self) {
        let Some(feature_id) = self.transform_gizmo.feature_id.clone() else {
            return;
        };
        let Some(pick) = self.transform_gizmo.deform.as_ref().map(|arm| arm.pick) else {
            return;
        };
        let row = self.deform_picks(&feature_id).get(pick).cloned();
        // A drag is its own undo step: a form edit of the same feature just
        // before it (same coalesce key for the form's offset) is not folded in.
        self.history.break_coalescing();
        if let (Some(arm), Some(row)) = (self.transform_gizmo.deform.as_mut(), row) {
            arm.grab = Some((row.offset, row.target, row.normal));
            arm.live = None;
        }
    }

    pub(super) fn deform_grab_end(&mut self) {
        if let Some(arm) = self.transform_gizmo.deform.as_mut() {
            arm.grab = None;
            arm.live = None;
        }
    }

    /// The gizmo's write-back: the offset at the grab plus the motion along
    /// the pick's normal.
    pub(super) fn write_deform_pose(&mut self, pose: &TransformPose) {
        let Some(feature_id) = self.transform_gizmo.feature_id.clone() else {
            return;
        };
        let Some(arm) = self.transform_gizmo.deform.clone() else {
            return;
        };
        let Some((offset, target, normal)) = arm.grab else {
            return;
        };
        let along: f64 = (0..3).map(|axis| (pose.position[axis] - target[axis]) * normal[axis]).sum();
        let moved = offset + along;
        let moved = if self.transform_dragging() { crate::formatting::snap_drag_moved(offset, moved) } else { moved };
        if let Some(live) = self.transform_gizmo.deform.as_mut() {
            // The handle follows the normal, whatever axis is dragged.
            live.live = Some(TransformPose {
                position: std::array::from_fn(|axis| target[axis] + normal[axis] * (moved - offset)),
                rotation_deg: [0.0; 3],
                scale: [1.0; 3],
            });
        }
        // Nothing to write when the stored offset already is `moved`. Compared
        // against the INPUT the last write left — not the grab's offset (a
        // drag that went out and came back must write the grab's value back
        // over the one it wrote on the way out), and not the evaluated
        // `resolved` row (an asynchronous run may not have caught up). An
        // offset still held as an expression has not been written by this
        // drag, so it is compared against the grab: an expression is only
        // replaced by a number when the drag actually changes it.
        let stored = self.deform_stored_offset(&feature_id, arm.pick).unwrap_or(offset);
        if stored == moved {
            return;
        }
        let _ = self.deform_set_offset(&feature_id, arm.pick, moved, Some(&format!("deform-drag:{feature_id}")));
    }

    /// Pick `pick`'s offset as STORED (the input, not the evaluated result),
    /// read by the rule [`Self::deform_set_offset`] writes by: its own
    /// `offset` when it carries one, else the form's. `None` when that value
    /// is not a plain number (an expression).
    fn deform_stored_offset(&self, feature_id: &str, pick: usize) -> Option<f64> {
        let index = self.history.index_of(feature_id)?;
        let persistent = self.history.feature_persistent_data(index).unwrap_or(Value::Null);
        match persistent.pointer(&format!("/deform/picks/{pick}/offset")) {
            Some(Value::Null) | None => {}
            Some(Value::String(text)) if text.trim().is_empty() => {}
            Some(own) => return own.as_f64(),
        }
        self.history.feature_params(index)?.get("offset")?.as_f64()
    }

    /// Set pick `pick`'s offset: its own `offset` when the feature lists its
    /// picks and that one carries one, else the form's `offset` (which every
    /// pick without its own follows). `coalesce` folds a drag into one undo
    /// step. Re-runs the history.
    pub fn deform_set_offset(
        &mut self,
        feature_id: &str,
        pick: usize,
        offset: f64,
        coalesce: Option<&str>,
    ) -> Result<(), String> {
        if !offset.is_finite() {
            return Err("deform offset must be finite".into());
        }
        let index = self.history.index_of(feature_id).ok_or_else(|| format!("no feature '{feature_id}'"))?;
        if !self.is_deform_feature(feature_id) {
            return Err(format!("'{feature_id}' is not a Deform Face"));
        }
        let persistent = self.history.feature_persistent_data(index).unwrap_or(Value::Null);
        // The kernel's rule: a pick's offset that is missing, null or blank
        // takes the form's.
        let own = persistent.pointer(&format!("/deform/picks/{pick}/offset")).is_some_and(|value| match value {
            Value::Null => false,
            Value::String(text) => !text.trim().is_empty(),
            _ => true,
        });
        if own {
            let mut deform = persistent.get("deform").cloned().unwrap_or(Value::Null);
            deform["picks"][pick]["offset"] = serde_json::json!(offset);
            self.history.set_feature_persistent_field_coalesced(index, "deform", deform, coalesce);
        } else {
            let mut params = self.history.feature_params(index).unwrap_or_else(|| serde_json::json!({}));
            params["offset"] = serde_json::json!(offset);
            self.history.set_feature_params(index, params);
        }
        self.rerun_history();
        Ok(())
    }

    /// Feed the handle overlay for `feature_id` — a dot per pick target and a
    /// line back to its anchor, the armed one highlighted — or clear it with
    /// `None`. Cheap to call every frame: an unchanged feed is skipped.
    pub fn refresh_deform_edit_overlay(&mut self, feature_id: Option<&str>) {
        let feature_id = feature_id.filter(|id| self.is_deform_feature(id));
        if self.deform_edit_feature.as_deref() != feature_id {
            self.deform_edit_feature = feature_id.map(str::to_string);
        }
        let armed = feature_id.and_then(|id| self.deform_armed_pick(id));
        let key = feature_id.map(|id| (id.to_string(), armed, self.applied_generation));
        if self.deform_overlay_key == key {
            return;
        }
        self.deform_overlay_key = key;
        let empty = serde_json::json!({ "groups": [{ "name": OVERLAY_GROUP }] }).to_string();
        let Some(feature_id) = feature_id else {
            let _ = self.widgets.set_overlay_json(&empty);
            self.dirty = true;
            return;
        };
        let rows = self.deform_picks(feature_id);
        let mut line_positions: Vec<f64> = Vec::new();
        let mut line_colors: Vec<f32> = Vec::new();
        let mut dot_positions: Vec<f64> = Vec::new();
        let mut dot_colors: Vec<f32> = Vec::new();
        for (index, row) in rows.iter().enumerate() {
            line_positions.extend_from_slice(&row.anchor);
            line_positions.extend_from_slice(&row.target);
            line_colors.extend_from_slice(&LINE_COLOR);
            line_colors.extend_from_slice(&LINE_COLOR);
            dot_positions.extend_from_slice(&row.target);
            dot_colors.extend_from_slice(if armed == Some(index) { &DOT_SELECTED } else { &DOT_PLAIN });
        }
        let group = serde_json::json!({
            "name": OVERLAY_GROUP,
            "renderOrder": 6,
            "lines": { "positions": line_positions, "colors": line_colors },
            "points": { "positions": dot_positions, "colors": dot_colors, "size": DOT_SIZE_PX },
        });
        let _ = self.widgets.set_overlay_json(&serde_json::json!({ "groups": [group] }).to_string());
        self.dirty = true;
    }
}

