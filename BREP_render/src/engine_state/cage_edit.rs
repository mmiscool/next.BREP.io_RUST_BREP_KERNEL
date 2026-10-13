//! The CONTROL CAGE editor of a NURBS Solid (`NS`): the engine side.
//!
//! The kernel's [`ControlCage`] is the one model: the points the editor lists,
//! draws and drags are read through `nurbs_solid::resolve_cage`, which is also
//! what the kernel builds the body from, so a handle is never somewhere the
//! surface is not controlled from. Every edit writes the whole cage back to
//! `persistentData.cage` (materialising the primitive's cage on the first
//! edit) and re-runs the history.
//!
//! SELECTION lives in the transform gizmo's arm ([`CageArm`]): the gizmo is
//! armed exactly while at least one point is selected, at the selection's
//! centroid. A drag re-derives every selected point from a snapshot taken at
//! the grab — translate adds the pose's motion, rotate turns the snapshot
//! about the grab centroid — so a long drag never accumulates error, and the
//! frames of one drag coalesce into one undo step.
//!
//! LOOPS are inserted by knot insertion ([`ControlCage::insert_ring_between`],
//! [`ControlCage::insert_column_between`]) beside the selected point, which
//! refines the cage without moving the body; the new loop is selected
//! afterwards so it can be dragged at once.

use super::transform_gizmo::{rotate_euler_xyz_f64, TransformPose};
use super::*;
use brep_kernel::feature_pipeline::nurbs_solid;
use brep_kernel::{CagePoint, ControlCage, Vec3};
use serde_json::Value;

/// The overlay group the cage draws into.
const OVERLAY_GROUP: &str = "cage-edit";
/// Cage line colour (0..1 rgb) and the dots: plain and selected.
const CAGE_LINE: [f32; 3] = [0.30, 0.48, 0.80];
const DOT_PLAIN: [f32; 3] = [0.70, 0.84, 1.0];
const DOT_SELECTED: [f32; 3] = [0.435, 0.886, 0.435];
const DOT_SIZE_PX: f32 = 9.0;
/// A viewport click within this many CSS pixels of a dot picks it.
pub const CAGE_PICK_RADIUS_PX: f64 = 10.0;

/// The gizmo's cage subject: which points, and the grab snapshot of a drag.
#[derive(Clone, Debug, Default)]
pub struct CageArm {
    /// Selected linear point indices, ascending, no repeats.
    pub(super) selected: Vec<usize>,
    /// The selected points at grab and the grab centroid, while a drag is in
    /// flight. Every drag frame re-derives the points from these.
    pub(super) grab: Option<(Vec<[f64; 3]>, [f64; 3])>,
    /// The pose last written during the drag — what the widget tracks.
    pub(super) live: Option<TransformPose>,
}

/// How a pick or a list action changes the selection.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CageSelect {
    /// The given points become the selection.
    Replace,
    /// Each given point joins the selection, or leaves it when already in.
    Toggle,
    /// The given points join the selection.
    Add,
}

/// The cage as the editor's panel describes it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CageSummary {
    pub segments: usize,
    pub rings: usize,
    pub points: usize,
    /// Whether the document carries an edited cage (the primitive params are
    /// then only what it started from; Reset drops it).
    pub persisted: bool,
}

fn array3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn vec3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn centroid(points: &[[f64; 3]]) -> [f64; 3] {
    if points.is_empty() {
        return [0.0; 3];
    }
    let mut sum = [0.0; 3];
    for p in points {
        for k in 0..3 {
            sum[k] += p[k];
        }
    }
    let n = points.len() as f64;
    [sum[0] / n, sum[1] / n, sum[2] / n]
}

impl EngineState {
    // --- reads -----------------------------------------------------------------

    /// Whether `feature_id` names a NURBS Solid.
    pub fn is_cage_feature(&self, feature_id: &str) -> bool {
        self.history
            .index_of(feature_id)
            .and_then(|index| self.history.feature_type(index))
            .is_some_and(|ty| ty.eq_ignore_ascii_case("NS"))
    }

    /// The cage the kernel builds `feature_id` from (persisted or primitive),
    /// or the reason it cannot be read.
    pub fn cage_of(&self, feature_id: &str) -> Result<ControlCage, String> {
        let index = self
            .history
            .index_of(feature_id)
            .filter(|_| self.is_cage_feature(feature_id))
            .ok_or_else(|| format!("'{feature_id}' is not a NURBS solid"))?;
        let params = self.history.feature_params(index).unwrap_or(Value::Null);
        let persistent = self.history.feature_persistent_data(index).unwrap_or(Value::Null);
        let env = brep_kernel::Env::build(&self.history.expressions(), &self.history.configurator())
            .map_err(|error| format!("expressions: {error}"))?;
        nurbs_solid::resolve_cage(&env, &params, &persistent)
    }

    /// Every control point of the cage in linear order (empty for a
    /// non-cage, or a cage the document cannot build).
    pub fn cage_points(&self, feature_id: &str) -> Vec<[f64; 3]> {
        self.cage_of(feature_id)
            .map(|cage| cage.points().into_iter().map(array3).collect())
            .unwrap_or_default()
    }

    /// The cage's shape for the panel, `None` for a non-cage.
    pub fn cage_summary(&self, feature_id: &str) -> Option<CageSummary> {
        let cage = self.cage_of(feature_id).ok()?;
        let index = self.history.index_of(feature_id)?;
        let persistent = self.history.feature_persistent_data(index).unwrap_or(Value::Null);
        Some(CageSummary {
            segments: cage.segments(),
            rings: cage.ring_count(),
            points: cage.point_count(),
            persisted: nurbs_solid::has_persisted_cage(&persistent),
        })
    }

    /// The NURBS solid whose cage editor is open, if any (the history panel
    /// feeds it with the overlay).
    pub fn cage_edit_feature(&self) -> Option<&str> {
        self.cage_edit_feature.as_deref()
    }

    /// The selected point indices of the armed cage (empty unless the gizmo
    /// is armed on a cage).
    pub fn cage_selected(&self) -> Vec<usize> {
        self.transform_gizmo
            .cage
            .as_ref()
            .map(|cage| cage.selected.clone())
            .unwrap_or_default()
    }

    /// Whether the gizmo is armed on the cage of `feature_id`.
    pub fn cage_armed_for(&self, feature_id: &str) -> bool {
        self.transform_gizmo.cage.is_some()
            && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
    }

    // --- selection and the gizmo ---------------------------------------------------

    /// Change the selection of `feature_id`'s cage and arm the gizmo at the
    /// selection's centroid (disarmed when nothing is left selected).
    /// Indices outside the cage are ignored.
    pub fn cage_select(&mut self, feature_id: &str, indices: &[usize], mode: CageSelect) {
        let count = self.cage_points(feature_id).len();
        if count == 0 {
            return;
        }
        let mut selected = if self.cage_armed_for(feature_id) {
            self.cage_selected()
        } else {
            Vec::new()
        };
        match mode {
            CageSelect::Replace => {
                selected = indices.iter().copied().filter(|i| *i < count).collect();
            }
            CageSelect::Add => {
                selected.extend(indices.iter().copied().filter(|i| *i < count));
            }
            CageSelect::Toggle => {
                for &index in indices.iter().filter(|i| **i < count) {
                    if let Some(at) = selected.iter().position(|s| *s == index) {
                        selected.remove(at);
                    } else {
                        selected.push(index);
                    }
                }
            }
        }
        selected.sort_unstable();
        selected.dedup();
        if selected.is_empty() {
            if self.cage_armed_for(feature_id) {
                self.disarm_transform();
            }
            return;
        }
        self.component_move_reset();
        self.transform_gizmo.feature_id = Some(feature_id.to_string());
        self.transform_gizmo.mode = GizmoMode::Transform;
        self.transform_gizmo.drag = None;
        self.transform_gizmo.anchor = None;
        self.transform_gizmo.port = None;
        self.transform_gizmo.cage = Some(CageArm {
            selected,
            grab: None,
            live: None,
        });
        self.clear_feature_dimension_overlay();
        self.sync_transform_gizmo();
    }

    /// Select every point of the cage.
    pub fn cage_select_all(&mut self, feature_id: &str) {
        let all: Vec<usize> = (0..self.cage_points(feature_id).len()).collect();
        self.cage_select(feature_id, &all, CageSelect::Replace);
    }

    /// Clear the selection (and so disarm the gizmo).
    pub fn cage_clear_selection(&mut self, feature_id: &str) {
        self.cage_select(feature_id, &[], CageSelect::Replace);
    }

    /// Grow the selection to the whole ring of every selected ring point (a
    /// selected pole stays as it is).
    pub fn cage_select_rings_of_selection(&mut self, feature_id: &str) {
        let Ok(cage) = self.cage_of(feature_id) else {
            return;
        };
        let mut grown = Vec::new();
        for index in self.cage_selected() {
            if let Some(CagePoint::Ring { ring, .. }) = cage.locate(index) {
                grown.extend((0..cage.segments()).map(|c| cage.index_of(ring, c)));
            }
        }
        self.cage_select(feature_id, &grown, CageSelect::Add);
    }

    /// Grow the selection to the whole column (pole to pole, poles excluded)
    /// of every selected ring point.
    pub fn cage_select_columns_of_selection(&mut self, feature_id: &str) {
        let Ok(cage) = self.cage_of(feature_id) else {
            return;
        };
        let mut grown = Vec::new();
        for index in self.cage_selected() {
            if let Some(CagePoint::Ring { column, .. }) = cage.locate(index) {
                grown.extend((0..cage.ring_count()).map(|r| cage.index_of(r, column)));
            }
        }
        self.cage_select(feature_id, &grown, CageSelect::Add);
    }

    /// A viewport click while a cage editor is open: when one of the cage's
    /// dots sits within [`CAGE_PICK_RADIUS_PX`] of CSS-pixel `(x, y)`, select
    /// it — REPLACING the selection, or TOGGLING it in with `toggle` (a
    /// modifier held) — and report its index. Dots are picked by projection,
    /// so a point behind the body is as pickable as one in front, which is
    /// what a cage wants. `None` leaves the click to the ordinary selection.
    pub fn cage_point_pick_at(&mut self, x: f64, y: f64, toggle: bool) -> Option<usize> {
        let feature_id = self.cage_edit_feature.clone()?;
        let points = self.cage_points(&feature_id);
        let mut best: Option<(usize, f64)> = None;
        for (index, point) in points.iter().enumerate() {
            if !self.camera.projectable(*point) {
                continue;
            }
            let (sx, sy, _) = self.camera.project(*point);
            let d = ((sx - x).powi(2) + (sy - y).powi(2)).sqrt();
            if d <= CAGE_PICK_RADIUS_PX && best.is_none_or(|(_, bd)| d < bd) {
                best = Some((index, d));
            }
        }
        let (index, _) = best?;
        let mode = if toggle { CageSelect::Toggle } else { CageSelect::Replace };
        self.cage_select(&feature_id, &[index], mode);
        Some(index)
    }

    /// Every cage dot projected to VIEWPORT-LOCAL px, `(index, (x, y))`, for
    /// the host's hit blob. Empty unless a cage editor is open.
    pub fn cage_point_screen_points(&self) -> Vec<(usize, (f64, f64))> {
        let Some(feature_id) = self.cage_edit_feature.as_deref() else {
            return Vec::new();
        };
        self.cage_points(feature_id)
            .into_iter()
            .enumerate()
            .filter(|(_, point)| self.camera.projectable(*point))
            .map(|(index, point)| {
                let (sx, sy, _) = self.camera.project(point);
                (index, (sx, sy))
            })
            .collect()
    }

    /// The armed cage's pose: the live drag pose while a drag is in flight,
    /// else the selection's centroid with the world frame. `None` when the
    /// selection is gone (the gizmo disarms).
    pub(super) fn cage_pose(&self) -> Option<TransformPose> {
        let arm = self.transform_gizmo.cage.as_ref()?;
        if self.transform_gizmo.drag.is_some() {
            if let Some(live) = arm.live {
                return Some(live);
            }
        }
        let feature_id = self.transform_gizmo.feature_id.as_deref()?;
        let points = self.cage_points(feature_id);
        let selected: Vec<[f64; 3]> = arm
            .selected
            .iter()
            .filter_map(|i| points.get(*i).copied())
            .collect();
        if selected.is_empty() {
            return None;
        }
        Some(TransformPose {
            position: centroid(&selected),
            rotation_deg: [0.0; 3],
            scale: [1.0; 3],
        })
    }

    /// Whether the armed cage's gizmo should draw its rotation rings: a lone
    /// point has nothing to turn about.
    pub(super) fn cage_shows_rings(&self) -> Option<bool> {
        self.transform_gizmo
            .cage
            .as_ref()
            .map(|arm| arm.selected.len() >= 2)
    }

    /// Snapshot the selection at the grab (called when a gizmo drag starts).
    pub(super) fn cage_grab_begin(&mut self) {
        let Some(feature_id) = self.transform_gizmo.feature_id.clone() else {
            return;
        };
        if self.transform_gizmo.cage.is_none() {
            return;
        }
        let points = self.cage_points(&feature_id);
        if let Some(arm) = self.transform_gizmo.cage.as_mut() {
            let snapshot: Vec<[f64; 3]> = arm
                .selected
                .iter()
                .filter_map(|i| points.get(*i).copied())
                .collect();
            let center = centroid(&snapshot);
            arm.grab = Some((snapshot, center));
            arm.live = None;
        }
    }

    /// Forget the grab snapshot (called when the drag ends).
    pub(super) fn cage_grab_end(&mut self) {
        if let Some(arm) = self.transform_gizmo.cage.as_mut() {
            arm.grab = None;
            arm.live = None;
        }
    }

    /// The gizmo's write-back for a cage: `pose` is the grab pose moved by the
    /// drag so far (position = grab centroid + translation; rotation = the
    /// drag's turn, from identity). Every selected point is re-derived from
    /// the grab snapshot: turned about the grab centroid, then moved.
    pub(super) fn write_cage_pose(&mut self, pose: &TransformPose) {
        let Some(feature_id) = self.transform_gizmo.feature_id.clone() else {
            return;
        };
        let Some(arm) = self.transform_gizmo.cage.as_ref() else {
            return;
        };
        let Some((snapshot, center)) = arm.grab.clone() else {
            return;
        };
        let selected = arm.selected.clone();
        let euler = [
            pose.rotation_deg[0].to_radians(),
            pose.rotation_deg[1].to_radians(),
            pose.rotation_deg[2].to_radians(),
        ];
        let shift = [
            pose.position[0] - center[0],
            pose.position[1] - center[1],
            pose.position[2] - center[2],
        ];
        let Ok(mut cage) = self.cage_of(&feature_id) else {
            return;
        };
        for (index, start) in selected.iter().zip(snapshot.iter()) {
            let offset = [start[0] - center[0], start[1] - center[1], start[2] - center[2]];
            let turned = rotate_euler_xyz_f64(offset, euler);
            let moved = Vec3::new(
                center[0] + turned[0] + shift[0],
                center[1] + turned[1] + shift[1],
                center[2] + turned[2] + shift[2],
            );
            let _ = cage.set_point(*index, moved);
        }
        if let Some(arm) = self.transform_gizmo.cage.as_mut() {
            arm.live = Some(*pose);
        }
        let key = format!("cage-drag:{feature_id}");
        self.write_cage(&feature_id, &cage, Some(&key));
    }

    // --- edits -------------------------------------------------------------------------

    /// Persist `cage` as the feature's edited cage (checkpointed; `coalesce`
    /// folds a run of same-key writes into one undo step) and re-run.
    fn write_cage(&mut self, feature_id: &str, cage: &ControlCage, coalesce: Option<&str>) {
        let Some(index) = self.history.index_of(feature_id) else {
            return;
        };
        self.history.set_feature_persistent_field_coalesced(
            index,
            nurbs_solid::PERSISTENT_KEY,
            cage.to_json(),
            coalesce,
        );
        self.rerun_history();
    }

    /// Move one point to `position` (a typed edit; coalesced per point).
    pub fn cage_set_point_position(
        &mut self,
        feature_id: &str,
        index: usize,
        position: [f64; 3],
    ) -> Result<(), String> {
        let mut cage = self.cage_of(feature_id)?;
        cage.set_point(index, vec3(position))?;
        let key = format!("cage-point:{feature_id}:{index}");
        self.write_cage(feature_id, &cage, Some(&key));
        self.sync_transform_gizmo();
        Ok(())
    }

    /// Insert a ring beside the selected point: `above` puts it between that
    /// point's ring and the next towards the top pole, else towards the
    /// bottom. With several points selected the first decides. The body is
    /// unchanged; the new ring is selected. Returns its ring index.
    pub fn cage_insert_ring(&mut self, feature_id: &str, above: bool) -> Result<usize, String> {
        let mut cage = self.cage_of(feature_id)?;
        let anchor = self
            .cage_selected()
            .first()
            .copied()
            .ok_or("select a point of the cage first: the ring goes beside it")?;
        // Rows count the poles: the bottom pole is row 0, ring r is row r + 1.
        let row = match cage.locate(anchor) {
            Some(CagePoint::Pole(0)) => 0,
            Some(CagePoint::Pole(_)) => cage.ring_count(),
            Some(CagePoint::Ring { ring, .. }) => {
                if above {
                    ring + 1
                } else {
                    ring
                }
            }
            None => return Err(format!("no cage point {anchor}")),
        };
        let new_ring = cage.insert_ring_between(row)?;
        self.write_cage(feature_id, &cage, None);
        let loop_points: Vec<usize> = (0..cage.segments()).map(|c| cage.index_of(new_ring, c)).collect();
        self.cage_select(feature_id, &loop_points, CageSelect::Replace);
        Ok(new_ring)
    }

    /// Insert a column (pole to pole) beside the selected point: `next` puts
    /// it between that point's column and the next around the ring, else the
    /// previous. A selected pole offers no column, so a ring point must be
    /// selected. The body is unchanged; the new column is selected. Returns
    /// its column index.
    pub fn cage_insert_column(&mut self, feature_id: &str, next: bool) -> Result<usize, String> {
        let mut cage = self.cage_of(feature_id)?;
        let selected = self.cage_selected();
        let column = selected
            .iter()
            .find_map(|index| match cage.locate(*index) {
                Some(CagePoint::Ring { column, .. }) => Some(column),
                _ => None,
            })
            .ok_or("select a ring point of the cage first: the column goes beside it")?;
        let n = cage.segments();
        let after = if next { column } else { (column + n - 1) % n };
        let new_column = cage.insert_column_between(after)?;
        self.write_cage(feature_id, &cage, None);
        let loop_points: Vec<usize> = (0..cage.ring_count()).map(|r| cage.index_of(r, new_column)).collect();
        self.cage_select(feature_id, &loop_points, CageSelect::Replace);
        Ok(new_column)
    }

    /// Drop the edited cage: the body returns to the primitive the params
    /// describe. Clears the selection.
    pub fn cage_reset(&mut self, feature_id: &str) -> Result<(), String> {
        let index = self
            .history
            .index_of(feature_id)
            .filter(|_| self.is_cage_feature(feature_id))
            .ok_or_else(|| format!("'{feature_id}' is not a NURBS solid"))?;
        if self.cage_armed_for(feature_id) {
            self.disarm_transform();
        }
        self.history
            .set_feature_persistent_field(index, nurbs_solid::PERSISTENT_KEY, Value::Null);
        self.rerun_history();
        Ok(())
    }

    // --- the overlay ------------------------------------------------------------------

    /// Feed the cage overlay for `feature_id` — its rings, its columns and a
    /// dot per point, the selected ones highlighted — or clear it with
    /// `None`. Cheap to call every frame: an unchanged feed is skipped.
    pub fn refresh_cage_edit_overlay(&mut self, feature_id: Option<&str>) {
        if self.cage_edit_feature.as_deref() != feature_id {
            self.cage_edit_feature = feature_id.map(str::to_string);
        }
        let selected = feature_id
            .filter(|id| self.cage_armed_for(id))
            .map(|_| self.cage_selected())
            .unwrap_or_default();
        let key = feature_id.map(|id| (id.to_string(), selected.clone(), self.applied_generation));
        if self.cage_overlay_key == key {
            return;
        }
        self.cage_overlay_key = key;
        let Some(feature_id) = feature_id else {
            let _ = self
                .widgets
                .set_overlay_json(&serde_json::json!({ "groups": [{ "name": OVERLAY_GROUP }] }).to_string());
            self.dirty = true;
            return;
        };
        let Ok(cage) = self.cage_of(feature_id) else {
            let _ = self
                .widgets
                .set_overlay_json(&serde_json::json!({ "groups": [{ "name": OVERLAY_GROUP }] }).to_string());
            self.dirty = true;
            return;
        };
        let mut line_positions: Vec<f64> = Vec::new();
        let mut line_colors: Vec<f32> = Vec::new();
        let mut push_line = |a: Vec3, b: Vec3| {
            line_positions.extend_from_slice(&array3(a));
            line_positions.extend_from_slice(&array3(b));
            line_colors.extend_from_slice(&CAGE_LINE);
            line_colors.extend_from_slice(&CAGE_LINE);
        };
        let n = cage.segments();
        for ring in &cage.rings {
            for c in 0..n {
                push_line(ring[c], ring[(c + 1) % n]);
            }
        }
        for c in 0..n {
            let mut previous = cage.poles[0];
            for ring in &cage.rings {
                push_line(previous, ring[c]);
                previous = ring[c];
            }
            push_line(previous, cage.poles[1]);
        }
        let mut dot_positions: Vec<f64> = Vec::new();
        let mut dot_colors: Vec<f32> = Vec::new();
        for (index, point) in cage.points().iter().enumerate() {
            dot_positions.extend_from_slice(&array3(*point));
            let color = if selected.binary_search(&index).is_ok() {
                DOT_SELECTED
            } else {
                DOT_PLAIN
            };
            dot_colors.extend_from_slice(&color);
        }
        let group = serde_json::json!({
            "name": OVERLAY_GROUP,
            "renderOrder": 6,
            "lines": { "positions": line_positions, "colors": line_colors },
            "points": { "positions": dot_positions, "colors": dot_colors, "size": DOT_SIZE_PX },
        });
        let _ = self
            .widgets
            .set_overlay_json(&serde_json::json!({ "groups": [group] }).to_string());
        self.dirty = true;
    }
}

