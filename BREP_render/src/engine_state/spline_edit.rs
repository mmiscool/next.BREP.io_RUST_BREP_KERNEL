//! Edit 3D spline anchors stored in `persistentData.spline.points`.
//!
//! List edits checkpoint the point list and rerun the history. Gizmo drags
//! coalesce into one undo step; attached anchors take their pose from the port
//! and cannot be armed. Reference selection attaches an anchor to a port, and
//! the cage overlay shows anchor points and forward/backward straight runs.
//!
//! Anchor rotations store a row-flat axis triad `[x, y, z]`, with x as the
//! travel direction. Gizmos use intrinsic XYZ Euler degrees; conversions here
//! follow the kernel's rotation order and the gizmo's matrix conventions.

use crate::json_support::vec3_or as read_vec3;
use crate::geometry3d::cross3 as cross;

use super::*;
use super::transform_gizmo::{normalize3, rotate_euler_xyz_f64};
use serde_json::Value;
use brep_kernel::PortSide;

/// The overlay group the cage draws into.
const OVERLAY_GROUP: &str = "spline-edit";
/// Cage line colours (0..1 rgb): forward run bright, backward run dim.
const CAGE_FORWARD: [f32; 3] = [0.35, 0.55, 1.0];
const CAGE_BACKWARD: [f32; 3] = [0.22, 0.34, 0.62];
/// Anchor dot colours: plain, attached to a port, and the selected one.
const DOT_PLAIN: [f32; 3] = [0.62, 0.79, 1.0];
const DOT_ATTACHED: [f32; 3] = [1.0, 0.66, 0.42];
const DOT_SELECTED: [f32; 3] = [0.435, 0.886, 0.435];
const DOT_SIZE_PX: f32 = 9.0;
/// A new anchor lands this far past its reference anchor along the direction.
const NEW_ANCHOR_GAP: f64 = 2.0;

/// One anchor as the editor lists it.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SplineAnchorRow {
    pub index: usize,
    pub id: String,
    /// The RESOLVED position (the last run's `{id}:P{index}` point; the
    /// persisted one before the first run).
    pub position: [f64; 3],
    /// The RESOLVED unit travel direction (the `{id}:P{index}` axis).
    pub direction: [f64; 3],
    pub forward: f64,
    pub backward: f64,
    pub flip: bool,
    /// `(port id, side)` when attached.
    pub attached: Option<(String, PortSide)>,
}

/// Intrinsic-XYZ Euler (degrees) whose rotation carries +X/+Y/+Z onto the
/// given orthonormal axes — the inverse of feeding each unit axis through
/// `rotate_euler_xyz_f64`. Same matrix-element naming as the gizmo's
/// quaternion extraction (`m<row><col>`, columns = the axes).
pub(super) fn euler_xyz_deg_from_axes(x: [f64; 3], y: [f64; 3], z: [f64; 3]) -> [f64; 3] {
    let (m11, m12, m13) = (x[0], y[0], z[0]);
    let (m22, m23) = (y[1], z[1]);
    let (m32, m33) = (y[2], z[2]);
    let ey = m13.clamp(-1.0, 1.0).asin();
    let (ex, ez) = if m13.abs() < 0.9999999 {
        ((-m23).atan2(m33), (-m12).atan2(m11))
    } else {
        (m32.atan2(m22), 0.0)
    };
    [ex.to_degrees(), ey.to_degrees(), ez.to_degrees()]
}

/// The axis triad an intrinsic-XYZ Euler (degrees) rotates +X/+Y/+Z onto.
pub(super) fn axes_from_euler_deg(deg: [f64; 3]) -> [[f64; 3]; 3] {
    let euler = [deg[0].to_radians(), deg[1].to_radians(), deg[2].to_radians()];
    [
        normalize3(rotate_euler_xyz_f64([1.0, 0.0, 0.0], euler)),
        normalize3(rotate_euler_xyz_f64([0.0, 1.0, 0.0], euler)),
        normalize3(rotate_euler_xyz_f64([0.0, 0.0, 1.0], euler)),
    ]
}

/// An orthonormal triad with `x` as its first axis (the frame a detached or
/// freshly added anchor is given so its stored rotation matches the direction
/// it was resolved to). Uses the same up-hint the kernel's `Frame` uses.
fn axes_from_direction(direction: [f64; 3]) -> [[f64; 3]; 3] {
    let x = normalize3(direction);
    let up = if x[2].abs() > 0.9 { [0.0, 1.0, 0.0] } else { [0.0, 0.0, 1.0] };
    let y = normalize3(cross(up, x));
    let z = normalize3(cross(x, y));
    [x, y, z]
}

fn read_axes(value: Option<&Value>) -> [[f64; 3]; 3] {
    let identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let Some(array) = value.and_then(Value::as_array) else {
        return identity;
    };
    if array.len() != 9 {
        return identity;
    }
    let numbers: Vec<f64> = array.iter().filter_map(Value::as_f64).collect();
    if numbers.len() != 9 {
        return identity;
    }
    [
        [numbers[0], numbers[1], numbers[2]],
        [numbers[3], numbers[4], numbers[5]],
        [numbers[6], numbers[7], numbers[8]],
    ]
}

fn axes_value(axes: [[f64; 3]; 3]) -> Value {
    Value::Array(axes.iter().flatten().map(|n| Value::from(*n)).collect())
}

fn distance(value: Option<&Value>) -> f64 {
    value.and_then(Value::as_f64).map(|d| d.max(0.0)).unwrap_or(1.0)
}

/// A normalized anchor: every field present in the persisted shape.
fn normalize_point(point: &Value, index: usize) -> Value {
    let id = point
        .get("id")
        .and_then(Value::as_str)
        .filter(|id| !id.is_empty())
        .map(str::to_string)
        .unwrap_or_else(|| format!("p{index}"));
    let attachment = point
        .get("attachment")
        .filter(|a| brep_kernel::SplineAttachment::parse(Some(a)).is_some())
        .cloned()
        .unwrap_or(Value::Null);
    serde_json::json!({
        "id": id,
        "position": read_vec3(point.get("position"), [0.0, 0.0, 0.0]),
        "rotation": axes_value(read_axes(point.get("rotation"))),
        "forwardDistance": distance(point.get("forwardDistance")),
        "backwardDistance": distance(point.get("backwardDistance")),
        "flipDirection": point.get("flipDirection").and_then(Value::as_bool).unwrap_or(false),
        "attachment": attachment,
    })
}

/// The persisted points of a spline, normalized — the kernel's default pair
/// `(0,0,0) → (5,0,0)` when fewer than two are stored, so the editor always
/// lists the anchors the curve is actually built from.
fn normalized_points(persistent: Option<&Value>) -> Vec<Value> {
    let stored: Vec<&Value> = persistent
        .and_then(|p| p.get("spline"))
        .and_then(|s| s.get("points"))
        .and_then(Value::as_array)
        .map(|points| points.iter().collect())
        .unwrap_or_default();
    if stored.len() < 2 {
        return vec![
            normalize_point(&serde_json::json!({ "id": "p0", "position": [0, 0, 0] }), 0),
            normalize_point(&serde_json::json!({ "id": "p1", "position": [5, 0, 0] }), 1),
        ];
    }
    stored
        .iter()
        .enumerate()
        .map(|(index, point)| normalize_point(point, index))
        .collect()
}

impl EngineState {
    // --- reads -----------------------------------------------------------------

    /// Whether `name` is a WAYPOINT feature's id.
    pub fn is_port_feature(&self, name: &str) -> bool {
        self.history
            .index_of(name)
            .and_then(|index| self.history.feature_type(index))
            .is_some_and(|ty| ty == "WP" || ty == "WAYPOINT")
    }

    /// Whether `name` addresses a harness node at all: a WAYPOINT feature of
    /// this document, a connection point it declares (`J1.VCC`), or one a
    /// placed component carries (`ACOMP1:J1.VCC`), which the harness report
    /// lists among its endpoints.
    pub fn is_port_id(&self, name: &str) -> bool {
        self.is_port_feature(name)
            || self.history.declared_points()
                .iter()
                .any(|point| point.address() == name)
            || self
                .wire_harness_report
                .as_ref()
                .is_some_and(|report| report.endpoints.iter().any(|endpoint| endpoint.id == name))
    }

    /// Whether `feature_id` names a SPLINE feature.
    pub fn is_spline_feature(&self, feature_id: &str) -> bool {
        self.history
            .index_of(feature_id)
            .and_then(|index| self.history.feature_type(index))
            .is_some_and(|ty| ty == "SP")
    }

    /// The spline's anchors, resolved (empty for a non-spline id).
    pub fn spline_anchors(&self, feature_id: &str) -> Vec<SplineAnchorRow> {
        let Some(index) = self.history.index_of(feature_id) else {
            return Vec::new();
        };
        if !self.is_spline_feature(feature_id) {
            return Vec::new();
        }
        let points = normalized_points(self.history.feature_persistent_data(index).as_ref());
        points
            .iter()
            .enumerate()
            .map(|(i, point)| {
                let name = format!("{feature_id}:P{i}");
                let axes = read_axes(point.get("rotation"));
                let flip = point.get("flipDirection").and_then(Value::as_bool).unwrap_or(false);
                let stored_direction = if flip {
                    [-axes[0][0], -axes[0][1], -axes[0][2]]
                } else {
                    axes[0]
                };
                let position = self
                    .sketch_points
                    .iter()
                    .find(|(n, _)| *n == name)
                    .map(|(_, p)| [p.position.x, p.position.y, p.position.z])
                    .unwrap_or_else(|| read_vec3(point.get("position"), [0.0; 3]));
                let direction = self
                    .sketch_axes
                    .iter()
                    .find(|(n, _)| *n == name)
                    .map(|(_, a)| [a.direction.x, a.direction.y, a.direction.z])
                    .unwrap_or_else(|| normalize3(stored_direction));
                let attached = brep_kernel::SplineAttachment::parse(point.get("attachment"))
                    .map(|a| (a.port_ref, a.side));
                SplineAnchorRow {
                    index: i,
                    id: point.get("id").and_then(Value::as_str).unwrap_or("").to_string(),
                    position,
                    direction,
                    forward: distance(point.get("forwardDistance")),
                    backward: distance(point.get("backwardDistance")),
                    flip,
                    attached,
                }
            })
            .collect()
    }

    /// The armed spline anchor `(feature id, index)`, if the gizmo is on one.
    pub fn armed_spline_anchor(&self) -> Option<(String, usize)> {
        let anchor = self.transform_gizmo.anchor?;
        let id = self.transform_gizmo.feature_id.clone()?;
        Some((id, anchor))
    }

    /// The spline whose anchor editor is open, if any (the history panel feeds
    /// it with the cage overlay).
    pub fn spline_edit_feature(&self) -> Option<&str> {
        self.spline_edit_feature.as_deref()
    }

    /// A viewport click while a spline's editor is open: when one of THAT
    /// spline's anchor dots (the sheet's vertex there) sits under CSS-pixel
    /// `(x, y)`, select the anchor — the gizmo arms on a free one — and report
    /// its index. The RAW pick is used, so the selection filter cannot hide
    /// the anchors; `None` leaves the click to the ordinary selection.
    pub fn spline_anchor_pick_at(&mut self, x: f64, y: f64) -> Option<usize> {
        let feature_id = self.spline_edit_feature.clone()?;
        let candidates = self.pick_candidates_at(x, y);
        let hit = candidates
            .iter()
            .find(|c| matches!(c.kind, crate::pick::PickKind::Vertex) && c.solid == feature_id)?;
        let rows = self.spline_anchors(&feature_id);
        let index = rows
            .iter()
            .map(|row| {
                let d = [0, 1, 2]
                    .iter()
                    .map(|&k| (row.position[k] - hit.position[k]).powi(2))
                    .sum::<f64>()
                    .sqrt();
                (row.index, d, row.position)
            })
            .filter(|(_, d, p)| {
                *d <= 1e-4 * (1.0 + p.iter().map(|v| v.abs()).fold(0.0, f64::max))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))?
            .0;
        self.arm_spline_anchor(&feature_id, index);
        self.spline_anchor_picked = Some(index);
        Some(index)
    }

    /// The anchor the last viewport click selected, once — the history panel
    /// takes it to move its row selection.
    pub fn take_spline_anchor_pick(&mut self) -> Option<usize> {
        self.spline_anchor_picked.take()
    }

    // --- the write lane ----------------------------------------------------------

    /// The normalized point list of `feature_id`, or `None` for a non-spline.
    fn spline_points(&self, feature_id: &str) -> Option<Vec<Value>> {
        let index = self.history.index_of(feature_id)?;
        if !self.is_spline_feature(feature_id) {
            return None;
        }
        Some(normalized_points(self.history.feature_persistent_data(index).as_ref()))
    }

    /// Persist `points` as the spline document (checkpointed; `coalesce`
    /// folds a run of same-key writes into one undo step) and re-run.
    fn write_spline_points(&mut self, feature_id: &str, points: Vec<Value>, coalesce: Option<&str>) {
        let Some(index) = self.history.index_of(feature_id) else {
            return;
        };
        self.history.set_feature_persistent_field_coalesced(
            index,
            "spline",
            serde_json::json!({ "points": points }),
            coalesce,
        );
        self.rerun_history();
    }

    /// Edit one anchor in place through `edit`, then persist + re-run.
    fn edit_spline_anchor(
        &mut self,
        feature_id: &str,
        anchor: usize,
        coalesce: Option<&str>,
        edit: impl FnOnce(&mut serde_json::Map<String, Value>),
    ) -> Result<(), String> {
        let mut points = self
            .spline_points(feature_id)
            .ok_or_else(|| format!("'{feature_id}' is not a spline"))?;
        let point = points
            .get_mut(anchor)
            .and_then(Value::as_object_mut)
            .ok_or_else(|| format!("spline '{feature_id}' has no anchor {anchor}"))?;
        edit(point);
        self.write_spline_points(feature_id, points, coalesce);
        Ok(())
    }

    /// Set an anchor's stored position (ignored by the kernel while attached).
    pub fn spline_set_anchor_position(
        &mut self,
        feature_id: &str,
        anchor: usize,
        position: [f64; 3],
    ) -> Result<(), String> {
        let key = format!("spline-anchor-position:{feature_id}:{anchor}");
        self.edit_spline_anchor(feature_id, anchor, Some(&key), |point| {
            point.insert("position".into(), serde_json::json!(position));
        })
    }

    /// Set an anchor's forward / backward straight-run distances (either may
    /// be left alone). Negative values clamp to zero.
    pub fn spline_set_anchor_distances(
        &mut self,
        feature_id: &str,
        anchor: usize,
        forward: Option<f64>,
        backward: Option<f64>,
    ) -> Result<(), String> {
        let key = format!("spline-anchor-distances:{feature_id}:{anchor}");
        self.edit_spline_anchor(feature_id, anchor, Some(&key), |point| {
            if let Some(forward) = forward {
                point.insert("forwardDistance".into(), Value::from(forward.max(0.0)));
            }
            if let Some(backward) = backward {
                point.insert("backwardDistance".into(), Value::from(backward.max(0.0)));
            }
        })
    }

    /// Flip an anchor's travel direction (no effect while attached — the
    /// side decides).
    pub fn spline_set_anchor_flip(&mut self, feature_id: &str, anchor: usize, flip: bool) -> Result<(), String> {
        self.edit_spline_anchor(feature_id, anchor, None, |point| {
            point.insert("flipDirection".into(), Value::Bool(flip));
        })
    }

    /// Switch an attached anchor's port side.
    pub fn spline_set_anchor_side(&mut self, feature_id: &str, anchor: usize, side: PortSide) -> Result<(), String> {
        self.edit_spline_anchor(feature_id, anchor, None, |point| {
            if let Some(attachment) = point.get_mut("attachment").and_then(Value::as_object_mut) {
                attachment.insert("side".into(), Value::String(side.letter().to_string()));
            }
        })
    }

    /// Detach an anchor from its port, keeping the curve where it is: the
    /// resolved position and direction the port gave it become its stored
    /// placement.
    pub fn spline_detach_anchor(&mut self, feature_id: &str, anchor: usize) -> Result<(), String> {
        let row = self
            .spline_anchors(feature_id)
            .into_iter()
            .nth(anchor)
            .ok_or_else(|| format!("spline '{feature_id}' has no anchor {anchor}"))?;
        self.edit_spline_anchor(feature_id, anchor, None, |point| {
            point.insert("attachment".into(), Value::Null);
            point.insert("position".into(), serde_json::json!(row.position));
            point.insert("rotation".into(), axes_value(axes_from_direction(row.direction)));
            point.insert("flipDirection".into(), Value::Bool(false));
        })
    }

    /// Add an anchor after `after` (the last anchor when `None`), placed past
    /// it along its direction by its forward run plus a gap, facing the same
    /// way. Returns the new anchor's index.
    pub fn spline_add_anchor(&mut self, feature_id: &str, after: Option<usize>) -> Result<usize, String> {
        let rows = self.spline_anchors(feature_id);
        if rows.is_empty() {
            return Err(format!("'{feature_id}' is not a spline"));
        }
        let mut points = self.spline_points(feature_id).expect("a spline by the check above");
        let at = after.unwrap_or(rows.len() - 1).min(rows.len() - 1);
        let reference = &rows[at];
        let step = reference.forward + NEW_ANCHOR_GAP;
        let position = [
            reference.position[0] + reference.direction[0] * step,
            reference.position[1] + reference.direction[1] * step,
            reference.position[2] + reference.direction[2] * step,
        ];
        let next_number = points.len();
        let point = serde_json::json!({
            "id": format!("p{next_number}"),
            "position": position,
            "rotation": axes_value(axes_from_direction(reference.direction)),
            "forwardDistance": 1.0,
            "backwardDistance": 1.0,
            "flipDirection": false,
            "attachment": Value::Null,
        });
        let index = at + 1;
        points.insert(index, point);
        self.write_spline_points(feature_id, points, None);
        Ok(index)
    }

    /// Remove an anchor. A spline keeps at least two.
    pub fn spline_remove_anchor(&mut self, feature_id: &str, anchor: usize) -> Result<(), String> {
        let mut points = self
            .spline_points(feature_id)
            .ok_or_else(|| format!("'{feature_id}' is not a spline"))?;
        if anchor >= points.len() {
            return Err(format!("spline '{feature_id}' has no anchor {anchor}"));
        }
        if points.len() <= 2 {
            return Err("a spline keeps at least two anchors".into());
        }
        points.remove(anchor);
        if self.transform_gizmo.anchor.is_some()
            && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
        {
            self.disarm_transform();
        }
        self.write_spline_points(feature_id, points, None);
        Ok(())
    }

    /// Move an anchor one place up (towards the start) or down.
    pub fn spline_move_anchor(&mut self, feature_id: &str, anchor: usize, up: bool) -> Result<(), String> {
        let mut points = self
            .spline_points(feature_id)
            .ok_or_else(|| format!("'{feature_id}' is not a spline"))?;
        let target = if up {
            anchor.checked_sub(1)
        } else {
            (anchor + 1 < points.len()).then_some(anchor + 1)
        };
        let Some(target) = target else {
            return Ok(()); // already at the end it was pushed towards
        };
        if anchor >= points.len() {
            return Err(format!("spline '{feature_id}' has no anchor {anchor}"));
        }
        points.swap(anchor, target);
        if self.transform_gizmo.anchor == Some(anchor)
            && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
        {
            self.transform_gizmo.anchor = Some(target);
        }
        self.write_spline_points(feature_id, points, None);
        Ok(())
    }

    // --- the gizmo -----------------------------------------------------------------

    /// Arm the move/rotate gizmo on one anchor. Refused (false) for an
    /// attached anchor — the port owns its pose — and for a bad index.
    pub fn arm_spline_anchor(&mut self, feature_id: &str, anchor: usize) -> bool {
        let rows = self.spline_anchors(feature_id);
        let Some(row) = rows.get(anchor) else {
            return false;
        };
        if row.attached.is_some() {
            // The port owns this anchor's pose: nothing to arm — and a gizmo
            // left on the PREVIOUSLY selected anchor would contradict the
            // selection, so it goes.
            if self.transform_gizmo.anchor.is_some() {
                self.disarm_transform();
            }
            return false;
        }
        self.component_move_reset();
        self.transform_gizmo.feature_id = Some(feature_id.to_string());
        self.transform_gizmo.mode = GizmoMode::Transform;
        self.transform_gizmo.drag = None;
        self.transform_gizmo.anchor = Some(anchor);
        self.clear_feature_dimension_overlay();
        self.sync_transform_gizmo();
        true
    }

    /// The armed anchor's pose: its stored position and the Euler of its
    /// stored axis triad. `None` when the anchor is gone (the gizmo disarms).
    pub(super) fn spline_anchor_pose(&self, feature_index: usize, anchor: usize) -> Option<([f64; 3], [f64; 3])> {
        let points = normalized_points(self.history.feature_persistent_data(feature_index).as_ref());
        let point = points.get(anchor)?;
        let axes = read_axes(point.get("rotation"));
        Some((
            read_vec3(point.get("position"), [0.0; 3]),
            euler_xyz_deg_from_axes(axes[0], axes[1], axes[2]),
        ))
    }

    /// The gizmo's write-back for an anchor: position + the axis triad the
    /// Euler rotates onto, coalesced per anchor so a drag is one undo step.
    pub(super) fn write_spline_anchor_pose(
        &mut self,
        feature_id: &str,
        anchor: usize,
        position: [f64; 3],
        rotation_deg: [f64; 3],
    ) {
        let key = format!("spline-anchor-pose:{feature_id}:{anchor}");
        let _ = self.edit_spline_anchor(feature_id, anchor, Some(&key), |point| {
            point.insert("position".into(), serde_json::json!(position));
            point.insert("rotation".into(), axes_value(axes_from_euler_deg(rotation_deg)));
        });
    }

    // --- attach (the reference picker's spline-anchor flavour) --------------------

    /// Enter the reference picker to attach `anchor` to a port: the pick
    /// admits a port's drawn sheet (its line / base vertex / the sheet), and
    /// Finish writes the attachment.
    pub fn begin_ref_select_for_spline_anchor(&mut self, feature_id: &str, anchor: usize) {
        let restore_index = self.history.rollback();
        let before = self
            .history
            .index_of(feature_id)
            .map(|i| i.saturating_sub(1))
            .unwrap_or(restore_index);
        let filter: Vec<String> = ["SKETCH", "EDGE", "VERTEX"].iter().map(|s| s.to_string()).collect();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        let seed = self
            .spline_anchors(feature_id)
            .get(anchor)
            .and_then(|row| row.attached.as_ref().map(|(port, _)| port.clone()))
            .into_iter()
            .collect();
        self.ref_select = Some(RefSelectState {
            feature_id: feature_id.to_string(),
            path: Vec::new(),
            label: format!("Anchor {anchor}: attach to port"),
            filter,
            multiple: false,
            names: seed,
            restore_index,
            target: RefSelectTarget::SplineAnchor { index: anchor },
        });
        if before != restore_index {
            self.history.set_rollback(before);
            self.rerun_history();
        }
        self.sync_ref_select_emphasis();
    }

    /// Write `port` as `anchor`'s attachment (side `A` for a new port; an
    /// existing side is kept when re-attaching to the same port) WITHOUT
    /// re-running — the picker's shared end tail re-runs.
    pub(super) fn attach_spline_anchor_no_rerun(&mut self, feature_id: &str, anchor: usize, port: &str) {
        let Some(index) = self.history.index_of(feature_id) else {
            return;
        };
        let Some(mut points) = self.spline_points(feature_id) else {
            return;
        };
        let Some(point) = points.get_mut(anchor).and_then(Value::as_object_mut) else {
            return;
        };
        let side = brep_kernel::SplineAttachment::parse(point.get("attachment"))
            .filter(|current| current.port_ref == port)
            .map(|current| current.side)
            .unwrap_or(PortSide::A);
        point.insert(
            "attachment".into(),
            serde_json::json!({ "portRef": port, "side": side.letter() }),
        );
        if self.transform_gizmo.anchor == Some(anchor)
            && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
        {
            // The port owns the pose now; the gizmo has nothing to move.
            self.transform_gizmo.anchor = None;
            self.transform_gizmo.feature_id = None;
            self.transform_gizmo.mode = GizmoMode::None;
            let _ = self.widgets.set_transform_json("null");
        }
        self.history.set_feature_persistent_field(index, "spline", serde_json::json!({ "points": points }));
    }

    // --- the cage overlay ---------------------------------------------------------

    /// Feed the anchor cage for `feature_id` (its forward / backward runs and
    /// a dot per anchor, `selected` highlighted), or clear it with `None`.
    /// Cheap to call every frame: an unchanged feed is skipped.
    pub fn refresh_spline_edit_overlay(&mut self, feature_id: Option<&str>, selected: Option<usize>) {
        if self.spline_edit_feature.as_deref() != feature_id {
            self.spline_edit_feature = feature_id.map(str::to_string);
            self.spline_anchor_picked = None;
        }
        let key = feature_id.map(|id| (id.to_string(), selected, self.applied_generation));
        if self.spline_overlay_key == key {
            return;
        }
        self.spline_overlay_key = key;
        let Some(feature_id) = feature_id else {
            let _ = self
                .widgets
                .set_overlay_json(&serde_json::json!({ "groups": [{ "name": OVERLAY_GROUP }] }).to_string());
            self.dirty = true;
            return;
        };
        let rows = self.spline_anchors(feature_id);
        let mut line_positions: Vec<f64> = Vec::new();
        let mut line_colors: Vec<f32> = Vec::new();
        let mut dot_positions: Vec<f64> = Vec::new();
        let mut dot_colors: Vec<f32> = Vec::new();
        let mut push_line = |a: [f64; 3], b: [f64; 3], color: [f32; 3]| {
            line_positions.extend_from_slice(&a);
            line_positions.extend_from_slice(&b);
            line_colors.extend_from_slice(&color);
            line_colors.extend_from_slice(&color);
        };
        for row in &rows {
            let along = |scale: f64| {
                [
                    row.position[0] + row.direction[0] * scale,
                    row.position[1] + row.direction[1] * scale,
                    row.position[2] + row.direction[2] * scale,
                ]
            };
            if row.forward > 0.0 {
                push_line(row.position, along(row.forward), CAGE_FORWARD);
            }
            if row.backward > 0.0 {
                push_line(row.position, along(-row.backward), CAGE_BACKWARD);
            }
            dot_positions.extend_from_slice(&row.position);
            let color = if Some(row.index) == selected {
                DOT_SELECTED
            } else if row.attached.is_some() {
                DOT_ATTACHED
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

