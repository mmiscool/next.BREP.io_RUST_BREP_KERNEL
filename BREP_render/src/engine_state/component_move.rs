use crate::geometry3d::{add3, cross3 as cross, sub3};
use super::transform_gizmo::{
    euler_xyz_deg_from_quat, normalize3, quat_from_axis_angle, quat_from_euler_xyz_deg, quat_mul,
    parse_drag_delta, Quat, TransformDelta,
};
use super::*;

// Component movement uses the shared gizmo at the member-bbox center. Fixed
// components cannot arm it. During a drag only the gizmo moves; release writes
// the pose once and reruns the history so assembly constraints can re-solve.
// Arming a component resets feature-transform and dimension gizmos because they
// share one widget slot. All translation and rotation handles are shown together.

/// The component-Move controller state (one per engine, like [`TransformArm`]).
#[derive(Default)]
pub struct ComponentMoveArm {
    /// The ACOMP feature id the gizmo is armed for (`None` = disarmed).
    pub(super) feature_id: Option<String>,
    /// Move Copy selects its members once the background scene rebuild lands.
    select_after_run: bool,
    /// The gizmo pivot — the component's member-bbox center, re-synced after
    /// every applied run so the gizmo follows a re-solved (snapped) component.
    pub(super) anchor: [f64; 3],
    /// The in-flight handle drag (grab snapshot + the pending composed pose).
    pub(super) drag: Option<ComponentMoveDrag>,
}

/// A grab snapshot: the handle, the grab screen point, the pose AT GRAB (the
/// frozen frame every drag move resolves against — no error accumulation), and
/// the PENDING composed pose the release commits.
#[derive(Clone)]
pub(super) struct ComponentMoveDrag {
    handle: u32,
    sx: f32,
    sy: f32,
    start: ComponentPose,
    pending: Option<ComponentPose>,
}

/// A component pose as the gizmo tracks it: the ACOMP `transform` pair plus the
/// gizmo anchor (bbox center) it pivots about.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) struct ComponentPose {
    pub translate: [f64; 3],
    pub rotate_deg: [f64; 3],
    pub anchor: [f64; 3],
}

impl EngineState {
    /// Whether the component Move gizmo is armed (for any component).
    pub fn component_move_armed(&self) -> bool {
        self.component_move.feature_id.is_some()
    }

    /// The armed component feature id (empty when disarmed).
    pub fn component_move_armed_feature(&self) -> String {
        self.component_move.feature_id.clone().unwrap_or_default()
    }

    /// The Move toggle (context bar / tree action): ARMS the full gizmo (all
    /// handle sets) for `feature_id`, or DISARMS when it is already armed
    /// (arming fresh replaces any other armed component). A FIXED component
    /// refuses with a toast and never arms.
    pub fn component_move_toggle(&mut self, feature_id: &str) {
        let Some(info) = self.component_info(feature_id) else {
            self.push_notice(format!("'{feature_id}' is not an assembly component"));
            return;
        };
        if info.fixed {
            self.push_notice(format!(
                "{} ({feature_id}) is fixed — unfix it to move",
                info.part_name
            ));
            return;
        }
        if self.component_move.feature_id.as_deref() == Some(feature_id) {
            self.disarm_transform();
        } else {
            self.component_move_arm_widget(feature_id);
        }
    }

    /// The Move Copy action (context bar / tree action): append a SECOND
    /// instance of `source_id`'s part — its `inputParams` cloned (same
    /// `partName`, same pose, any other placement keys riding along) under a
    /// fresh `ACOMP<n>` id and an explicit `isFixed: false` — then select it
    /// and arm the full Move gizmo on it, so the user drags the copy off the
    /// original. No new part is made: both instances reference one
    /// parts-library entry. A FIXED source copies fine (the copy is free).
    /// Returns the new feature id.
    pub fn component_move_copy(&mut self, source_id: &str) -> Result<String, String> {
        let Some(index) = self.history.index_of(source_id).filter(|&index| {
            matches!(
                self.history.feature_type(index).as_deref(),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        }) else {
            let error = format!("'{source_id}' is not an assembly component");
            self.push_notice(error.clone());
            return Err(error);
        };
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        let id = self.history.next_feature_id("ACOMP");
        let Some(map) = params.as_object_mut() else {
            return Err(format!("component '{source_id}': malformed inputParams"));
        };
        map.insert("id".into(), serde_json::Value::String(id.clone()));
        map.insert("isFixed".into(), serde_json::Value::Bool(false));
        let feature = serde_json::json!({
            "type": "ACOMP",
            "inputParams": params,
            "persistentData": {}
        });
        self.add_feature(&feature.to_string())?;
        // Select the copy so the context bar's actions (Move again, Delete, …)
        // target the instance the gizmo sits on, not the original.
        self.component_move_arm_widget(&id);
        if self.run_pending() {
            self.component_move.select_after_run = true;
        } else {
            self.select_component(&id);
        }
        Ok(id)
    }

    /// The `arm_transform` ROUTE for ACOMP features (history-panel expand):
    /// a FIXED component silently stays armless (the explicit Move action is
    /// the one that toasts).
    pub(super) fn component_move_arm(&mut self, feature_id: &str) {
        match self.component_info(feature_id) {
            Some(info) if !info.fixed => self.component_move_arm_widget(feature_id),
            _ => {}
        }
    }

    /// Drop the component arm STATE only (the caller owns the widget slot) —
    /// the exclusivity hook `arm_transform` / `arm_dimension` / `disarm_transform`
    /// call before taking the slot for themselves.
    pub(super) fn component_move_reset(&mut self) {
        self.component_move = ComponentMoveArm::default();
    }

    /// Arm for `feature_id`: claim the shared widget slot (clearing the
    /// feature gizmo + dimension overlay), pin the anchor at the member-bbox
    /// center, and feed the full handle set.
    fn component_move_arm_widget(&mut self, feature_id: &str) {
        // Claim the shared slot WITHOUT disarm_transform (which would also reset
        // the component state we are about to set).
        self.transform_gizmo.feature_id = None;
        self.transform_gizmo.mode = GizmoMode::None;
        self.transform_gizmo.drag = None;
        self.clear_feature_dimension_overlay();

        let anchor = self
            .component_bbox_center(feature_id)
            .or_else(|| self.component_info(feature_id).map(|info| info.translate))
            .unwrap_or([0.0; 3]);
        self.component_move.feature_id = Some(feature_id.to_string());
        self.component_move.select_after_run = false;
        self.component_move.anchor = anchor;
        self.component_move.drag = None;
        self.feed_component_widget();
        self.dirty = true;
    }

    /// (Re)feed the widget gizmo at the armed component's current pose+anchor.
    fn feed_component_widget(&mut self) {
        let Some(id) = self.component_move.feature_id.clone() else {
            return;
        };
        let Some(info) = self.component_info(&id) else {
            return;
        };
        let pose = ComponentPose {
            translate: info.translate,
            rotate_deg: info.rotate_deg,
            anchor: self.component_move.anchor,
        };
        let json = component_frame_json(&pose);
        let _ = self.widgets.set_transform_json(&json);
    }

    /// Post-run re-sync (the [`finish_apply`] hook, mirroring
    /// `sync_transform_gizmo`): re-anchor at the possibly re-solved member bbox
    /// and re-feed; auto-disarm when the component vanished or became fixed.
    pub(super) fn component_move_sync(&mut self) {
        let Some(id) = self.component_move.feature_id.clone() else {
            return;
        };
        match self.component_info(&id) {
            Some(info) if !info.fixed => {
                // Earlier replies may arrive while a newer run is pending.
                // Consume this once, only after the current scene is available.
                if self.component_move.select_after_run && !self.run_pending() {
                    self.component_move.select_after_run = false;
                    self.select_component(&id);
                }
                self.component_move.anchor =
                    self.component_bbox_center(&id).unwrap_or(info.translate);
                self.feed_component_widget();
                self.dirty = true;
            }
            _ => self.disarm_transform(),
        }
    }

    /// Begin a component-gizmo drag at viewport px `(x, y)`; `true` when a
    /// handle was grabbed (the viewport routes the drag here, not the camera).
    pub fn component_press(&mut self, x: f64, y: f64) -> bool {
        let Some(id) = self.component_move.feature_id.clone() else {
            return false;
        };
        let handle = self.transform_pick(x, y);
        if handle == 0 {
            return false;
        }
        let Some(info) = self.component_info(&id) else {
            return false;
        };
        self.widgets.set_transform_active(handle);
        self.component_move.drag = Some(ComponentMoveDrag {
            handle,
            sx: x as f32,
            sy: y as f32,
            start: ComponentPose {
                translate: info.translate,
                rotate_deg: info.rotate_deg,
                anchor: self.component_move.anchor,
            },
            pending: None,
        });
        self.dirty = true;
        true
    }

    /// Whether a component-gizmo drag is in flight.
    pub fn component_move_dragging(&self) -> bool {
        self.component_move.drag.is_some()
    }

    /// Continue the drag: resolve the world delta against the FROZEN grab frame,
    /// compose the pending pose, and move ONLY the visible gizmo (free move —
    /// the mesh follows on release, when the commit re-runs + re-solves).
    pub fn component_drag_to(&mut self, cx: f64, cy: f64) {
        let Some(drag) = self.component_move.drag.clone() else {
            return;
        };
        let cam = gizmo_camera(&self.camera);
        let frame = component_frame_json(&drag.start);
        let json = self.widgets.transform_drag_json_with_frame(
            &cam,
            &frame,
            drag.handle,
            drag.sx,
            drag.sy,
            cx as f32,
            cy as f32,
        );
        let Some(delta) = parse_drag_delta(&json) else {
            return;
        };
        let pending = compose_component_delta(&drag.start, &delta);
        // Live-follow the WIDGET at the pending pose; the gold active-handle
        // highlight survives (only a null feed clears it).
        let json = component_frame_json(&pending);
        let _ = self.widgets.set_transform_json(&json);
        if let Some(live) = self.component_move.drag.as_mut() {
            live.pending = Some(pending);
        }
        self.dirty = true;
    }

    /// End the drag: COMMIT the pending pose into the ACOMP's
    /// `inputParams.transform` (one param write → one undo entry → one rerun
    /// whose constraint tail re-solves; the post-run sync then re-glues the
    /// gizmo to wherever the solve left the component). A grab that never moved
    /// commits nothing.
    pub fn component_release(&mut self) {
        let Some(drag) = self.component_move.drag.take() else {
            return;
        };
        self.widgets.set_transform_active(0);
        self.dirty = true;
        let Some(pending) = drag.pending else {
            return;
        };
        let Some(id) = self.component_move.feature_id.clone() else {
            return;
        };
        self.component_move.anchor = pending.anchor;
        let Some(index) = self.history.index_of(&id) else {
            return;
        };
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        if !params.get("transform").map(|t| t.is_object()).unwrap_or(false) {
            if let Some(object) = params.as_object_mut() {
                object.insert("transform".into(), serde_json::json!({}));
            }
        }
        if let Some(transform) = params.get_mut("transform").and_then(|t| t.as_object_mut()) {
            transform.insert("translate".into(), serde_json::json!(pending.translate));
            transform.insert("rotateEulerDeg".into(), serde_json::json!(pending.rotate_deg));
        }
        let _ = self.update_feature_params(&id, &params.to_string());
    }

    /// The armed component gizmo's logical state for the verifier:
    /// `{armed, feature, anchor}`.
    pub fn component_move_json(&self) -> String {
        serde_json::json!({
            "armed": self.component_move_armed(),
            "feature": self.component_move_armed_feature(),
            "anchor": self.component_move.anchor,
        })
        .to_string()
    }
}

/// The widget frame feed for a component pose: origin = the ANCHOR (bbox
/// center), axes = the pose's rotated basis (intrinsic XYZ, the kernel bake).
/// EVERY handle set is shown (center free-move ball + axis arrows + rotation
/// arcs) — move and rotate coexist, no mode switch.
fn component_frame_json(pose: &ComponentPose) -> String {
    let euler = [
        pose.rotate_deg[0].to_radians(),
        pose.rotate_deg[1].to_radians(),
        pose.rotate_deg[2].to_radians(),
    ];
    let x = normalize3(rotate_euler_xyz_f64([1.0, 0.0, 0.0], euler));
    let y = normalize3(rotate_euler_xyz_f64([0.0, 1.0, 0.0], euler));
    let z = normalize3(rotate_euler_xyz_f64([0.0, 0.0, 1.0], euler));
    serde_json::json!({
        "origin": pose.anchor,
        "x": x,
        "y": y,
        "z": z,
        "showCenter": true,
        "showAxes": true,
        "showRings": true,
    })
    .to_string()
}

/// Translation shifts pose and anchor together. Rotation pivots about anchor C:
/// `R' = dR·R`, `translate' = C + dR·(translate − C)`.
pub(super) fn compose_component_delta(
    start: &ComponentPose,
    delta: &TransformDelta,
) -> ComponentPose {
    match delta {
        TransformDelta::Translate(d) => ComponentPose {
            translate: add3(start.translate, *d),
            rotate_deg: start.rotate_deg,
            anchor: add3(start.anchor, *d),
        },
        TransformDelta::Rotate { axis, radians } => {
            let dq = quat_from_axis_angle(*axis, *radians);
            let q0 = quat_from_euler_xyz_deg(start.rotate_deg);
            let rotate_deg = euler_xyz_deg_from_quat(quat_mul(dq, q0));
            let offset = sub3(start.translate, start.anchor);
            ComponentPose {
                translate: add3(start.anchor, quat_rotate(dq, offset)),
                rotate_deg,
                anchor: start.anchor,
            }
        }
    }
}

/// Rotate `v` by quaternion `q`: `v + 2·(q.xyz × (q.xyz × v + w·v))`.
fn quat_rotate(q: Quat, v: [f64; 3]) -> [f64; 3] {
    let u = [q[0], q[1], q[2]];
    let w = q[3];
    let t = cross(u, add3(cross(u, v), [w * v[0], w * v[1], w * v[2]]));
    [v[0] + 2.0 * t[0], v[1] + 2.0 * t[1], v[2] + 2.0 * t[2]]
}

