use super::*;

/// Which of the two gizmos is currently armed for the feature. `Transform` shows
/// the move/rotate gizmo; `Dimension` shows the on-canvas draggable dimension
/// annotations (feature-dimensions FD-1). Expanding a feature arms `Dimension`;
/// the viewport sphere/center toggle flips to `Transform` and back. The two modes
/// are EXCLUSIVE — the transform gizmo never renders in `Dimension` mode and
/// vice-versa.
#[derive(Default, Clone, Copy, PartialEq, Eq, Debug)]
pub enum GizmoMode {
    /// Neither gizmo armed (`feature_id` is `None`).
    #[default]
    None,
    /// The move/rotate transform gizmo is armed.
    Transform,
    /// The dimension-annotation gizmo is armed (FD-1).
    Dimension,
}

/// The transform-controls gizmo controller state (see the impl block below).
#[derive(Default)]
pub struct TransformArm {
    /// The feature id whose gizmo is armed (for EITHER mode), or `None`
    /// (disarmed → `mode == None`).
    pub(super) feature_id: Option<String>,
    /// Which gizmo is armed for `feature_id` (transform vs dimension). `None`
    /// exactly when `feature_id` is `None`.
    pub(super) mode: GizmoMode,
    /// The live handle drag, captured on pointer-down over a gizmo handle.
    pub(super) drag: Option<TransformDrag>,
    /// When the armed subject is ONE SPLINE ANCHOR rather than the feature's
    /// `transform` param: the anchor index into `persistentData.spline.points`
    /// of `feature_id` (an SP feature). The same gizmo, press / drag / release
    /// and live-follow apply; only the pose read and write-back differ (see
    /// `spline_edit.rs`).
    pub(super) anchor: Option<usize>,
    /// When the armed subject is ONE DECLARED CONNECTION POINT: its part-local
    /// address (`J1.VCC`). Exclusive with `feature_id` — a connection point is
    /// data, not a feature, so there is no history row to hold — and checked
    /// FIRST everywhere the pose is read or written, before any lookup that
    /// goes through the history. See the section at the foot of this file.
    pub(super) port: Option<String>,
}

/// A grab snapshot for an in-flight transform-gizmo drag: the grabbed handle,
/// the grab screen point, and the feature's pose AT GRAB. Every drag move
/// resolves an absolute delta from this grab (against the pinned grab-time
/// frame) and re-applies it to `start`, so the drag never accumulates error.
#[derive(Clone, Copy)]
pub(super) struct TransformDrag {
    handle: u32,
    sx: f32,
    sy: f32,
    start: TransformPose,
    /// The delta this drag has resolved SO FAR, as the handle resolved it —
    /// the readout's source. Taken from the drag rather than differenced out of
    /// the pose, because the pose is the only thing the two are agreed on: a
    /// rotation composes onto a quaternion and comes back as an Euler triple
    /// that a subtraction cannot turn into "42.5 degrees about Y" again. `None`
    /// until the pointer has actually moved off the grab point.
    last: Option<TransformDelta>,
}

/// A TRS pose read from / written to a feature's `inputParams.transform`
/// (`rotation` in DEGREES, intrinsic XYZ Euler order — the kernel `transform_bake`
/// convention, `M = T·R·S`).
#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) struct TransformPose {
    pub(super) position: [f64; 3],
    pub(super) rotation_deg: [f64; 3],
    pub(super) scale: [f64; 3],
}

/// A world-space delta shared by feature and component gizmo controllers.
#[derive(Clone, Copy, PartialEq, Debug)]
pub(super) enum TransformDelta {
    /// World-space translation, added to `position`.
    Translate([f64; 3]),
    /// World-axis rotation, pre-multiplied onto the pose's orientation.
    Rotate { axis: [f64; 3], radians: f64 },
}

/// Decode widget drag JSON; unknown kinds and invalid JSON produce no delta.
pub(super) fn parse_drag_delta(json: &str) -> Option<TransformDelta> {
    let value: serde_json::Value = serde_json::from_str(json).ok()?;
    match value.get("kind").and_then(|k| k.as_str()) {
        Some("translate") => Some(TransformDelta::Translate(
            crate::json_support::vec3_or(value.get("world"), [0.0; 3]),
        )),
        Some("rotate") => Some(TransformDelta::Rotate {
            axis: crate::json_support::vec3_or(value.get("axisWorld"), [0.0; 3]),
            radians: value.get("radians").and_then(|n| n.as_f64()).unwrap_or(0.0),
        }),
        _ => None,
    }
}

impl EngineState {
    /// Whether the TRANSFORM gizmo (move/rotate) is armed for ANY feature. False
    /// in dimension mode — the two ◎ modes are exclusive, so the transform gizmo
    /// arms/handles never render while dimensions are shown.
    pub fn transform_armed(&self) -> bool {
        matches!(self.transform_gizmo.mode, GizmoMode::Transform)
    }

    /// The transform-gizmo-armed feature id (empty unless in transform mode).
    pub fn transform_armed_feature(&self) -> String {
        if self.transform_armed() {
            self.transform_gizmo.feature_id.clone().unwrap_or_default()
        } else {
            String::new()
        }
    }

    /// The gizmo as the headed verifier sees it: the mode, the armed feature,
    /// the spline anchor index when the gizmo sits on one, and `origin` — the
    /// DRAWN move/rotate widget's frame origin (`null` when no widget is
    /// drawn), which is the proof a handle is on screen, unlike any
    /// params-derived pose.
    pub fn gizmo_state_json(&self) -> String {
        let mode = match self.transform_gizmo.mode {
            GizmoMode::None => "none",
            GizmoMode::Transform => "transform",
            GizmoMode::Dimension => "dimension",
        };
        serde_json::json!({
            "mode": mode,
            "armed": self.transform_armed(),
            "feature": self.transform_gizmo.feature_id,
            "anchor": self.transform_gizmo.anchor,
            "portPoint": self.transform_gizmo.port,
            "origin": self.widgets.transform_origin(),
            // A Transform Face turns about its stored `pivot`, and the gizmo
            // sits at `pivot + position` — so the two coincide only while the
            // motion is zero. The moment the user drags, the point the rotation
            // actually happens about is no longer under the handles and was
            // nowhere on screen at all. Published so the viewport can mark it.
            "pivot": self.armed_face_transform_pivot(),
        })
        .to_string()
    }

    /// The armed feature's Transform Face pivot in WORLD space, or `None` when
    /// the armed feature is not a Transform Face (every other transform feature
    /// turns about its own origin, which the gizmo is already on).
    pub fn armed_face_transform_pivot(&self) -> Option<[f64; 3]> {
        let id = self.transform_gizmo.feature_id.as_deref()?;
        let index = self.history.index_of(id)?;
        if !self.is_face_transform_feature(index) {
            return None;
        }
        let params = self.history.feature_params(index)?;
        let env = brep_kernel::Env::build(&self.history.expressions(), &self.history.configurator()).ok();
        self.face_transform_pivot(index, &params, env.as_ref())
    }

    /// What the gizmo drag is WORTH, for the chip the viewport draws on it —
    /// and, when the motion refused, why it refused, said about the user's
    /// model rather than about the kernel's internals.
    ///
    /// A gizmo drag used to be the one edit in the app that reported nothing:
    /// the model followed the pointer, the number went into `transform`, the
    /// form's `Transform` section is collapsed by default, and so the distance
    /// the user had just dragged appeared NOWHERE on screen. The Extrude
    /// distance arrow carries its value in a chip on the geometry; this is the
    /// same answer for the gizmo.
    ///
    /// The readout is live in two states, and silent otherwise:
    ///
    /// * **while a handle is being dragged** — the distance or angle so far.
    /// * **while the armed feature is REFUSING** — because that is the state
    ///   the user is left in after letting go of a refused drag, and it is the
    ///   state that most needs saying. The gizmo tracks the pointer while the
    ///   geometry stays at its last good answer, so on a refusal the handles
    ///   end up somewhere the model is not — measured at 15.4 mm INSIDE the
    ///   solid — and without this the only signal was a red banner in a dock on
    ///   the far side of the window from the pointer.
    ///
    /// `origin` is where to draw it: the gizmo's own frame origin, which is
    /// where the pointer is.
    pub fn transform_readout_json(&self) -> String {
        let quiet = serde_json::json!({ "active": false }).to_string();
        if !self.transform_armed() {
            return quiet;
        }
        let Some(origin) = self.widgets.transform_origin() else {
            return quiet;
        };

        // The refusal, if this feature is refusing — the armed feature's entry
        // in the last run's report, re-said around the faces it names.
        let refusal = self.armed_face_transform_refusal();
        let drag = self.transform_gizmo.drag.as_ref();
        let delta = drag.and_then(|d| d.last);

        // Silent unless something is happening: no drag in flight and no
        // refusal on the board means the model is simply sitting there.
        if delta.is_none() && refusal.is_none() {
            return quiet;
        }

        // What the handle resolved, formatted the way a dimension label is —
        // `compact_decimal(_, 4)`, so the chip and the form agree digit for
        // digit rather than rounding to different numbers.
        let (kind, label, value, unit, text) = match delta {
            Some(TransformDelta::Translate(world)) => {
                let distance = (world[0] * world[0] + world[1] * world[1] + world[2] * world[2]).sqrt();
                let axis = dominant_axis_label(&world);
                let shown = crate::formatting::compact_decimal(distance, 4);
                (
                    "translate",
                    axis.to_string(),
                    distance,
                    "mm",
                    format!("{axis}  {shown} mm"),
                )
            }
            Some(TransformDelta::Rotate { axis, radians }) => {
                let degrees = radians.to_degrees();
                let label = dominant_axis_label(&axis);
                let shown = crate::formatting::compact_decimal(degrees, 4);
                (
                    "rotate",
                    label.to_string(),
                    degrees,
                    "deg",
                    format!("{label}  {shown}\u{00b0}"),
                )
            }
            // Refusing, but not mid-drag (the user has let go, or typed the
            // value): the chip carries the refusal alone.
            None => ("none", String::new(), 0.0, "", String::new()),
        };

        let mut out = serde_json::json!({
            "active": true,
            "kind": kind,
            "axis": label,
            "value": value,
            "unit": unit,
            "text": text,
            "origin": origin,
            "dragging": drag.is_some(),
            "refused": refusal.is_some(),
        });
        if let Some(refusal) = refusal {
            out["motion"] = serde_json::json!(refusal.motion);
            out["reason"] = serde_json::json!(refusal.reason);
            out["hint"] = serde_json::json!(refusal.hint);
            out["kernel"] = serde_json::json!(refusal.kernel);
        }
        out.to_string()
    }

    /// The armed feature's refusal, re-said about the model — `None` when it is
    /// not a Transform Face, or is not refusing. The face names come from the
    /// feature's own `params.faces`, which is the thing the kernel's message
    /// does not carry and the app does.
    fn armed_face_transform_refusal(&self) -> Option<crate::face_transform_help::FaceTransformRefusal> {
        let id = self.transform_gizmo.feature_id.as_deref()?;
        let index = self.history.index_of(id)?;
        if !self.is_face_transform_feature(index) {
            return None;
        }
        let report: serde_json::Value = serde_json::from_str(&self.history_report).ok()?;
        // The kernel records each failure as `"<feature id>: <message>"`; the
        // delimiter after the exact id is what stops a shorter id matching a
        // longer one (`TF1` vs `TF10`).
        let prefix = format!("{id}: ");
        let message = report
            .get("featureErrors")?
            .as_array()?
            .iter()
            .filter_map(serde_json::Value::as_str)
            .find(|entry| entry.starts_with(&prefix))
            .map(|entry| &entry[prefix.len()..])?;
        let params = self.history.feature_params(index)?;
        let faces = brep_kernel::reference_names(params.get("faces"));
        let facts = crate::face_transform_help::RefusalFacts::from_report(&report, id);
        Some(crate::face_transform_help::explain_refusal(message, facts, &faces))
    }

    /// Whether the TRANSFORM gizmo is armed for THIS feature.
    pub fn transform_armed_for(&self, feature_id: &str) -> bool {
        self.transform_armed() && self.transform_gizmo.feature_id.as_deref() == Some(feature_id)
    }

    /// Arm the TRANSFORM gizmo for `feature_id` and feed it at the feature's
    /// transform frame. Re-arming a different feature moves the gizmo to it.
    /// Clears any dimension overlay (the modes are exclusive).
    ///
    /// An ACOMP (assembly component) feature ROUTES to the component Move gizmo
    /// instead: its `transform` is the component pose (`{translate,
    /// rotateEulerDeg}`, a different shape), its gizmo attaches at the member
    /// bbox center, commits on release, and refuses fixed instances — the
    /// generic position/rotationEuler gizmo must never write its keys into an
    /// ACOMP's params. This covers the history panel's arm-on-expand path too.
    pub fn arm_transform(&mut self, feature_id: &str) {
        let is_acomp = self
            .history
            .index_of(feature_id)
            .and_then(|index| self.history.feature_type(index))
            .is_some_and(|ty| super::components::is_acomp_feature_type(&ty));
        if is_acomp {
            self.component_move_arm(feature_id);
            return;
        }
        self.component_move_reset();
        self.transform_gizmo.feature_id = Some(feature_id.to_string());
        self.transform_gizmo.mode = GizmoMode::Transform;
        self.transform_gizmo.drag = None;
        self.transform_gizmo.anchor = None;
        self.transform_gizmo.port = None;
        self.clear_feature_dimension_overlay();
        self.sync_transform_gizmo();
    }

    /// Disarm: hide BOTH gizmos + drop any in-flight drag. Also resets the
    /// component Move gizmo (the widget slot is shared, so a disarm clears
    /// whichever controller was feeding it).
    pub fn disarm_transform(&mut self) {
        self.component_move_reset();
        self.transform_gizmo.feature_id = None;
        self.transform_gizmo.mode = GizmoMode::None;
        self.transform_gizmo.drag = None;
        self.transform_gizmo.anchor = None;
        self.transform_gizmo.port = None;
        let _ = self.widgets.set_transform_json("null");
        self.clear_feature_dimension_overlay();
        self.dirty = true;
    }

    /// The armed feature's current TRS pose (from its `inputParams.transform`),
    /// or `None` when disarmed / the feature vanished.
    fn armed_pose(&self) -> Option<TransformPose> {
        // A connection point has no history row, so this must come before
        // every lookup that needs one.
        if let Some(address) = self.transform_gizmo.port.clone() {
            return self.port_point_pose(&address);
        }
        let id = self.transform_gizmo.feature_id.as_deref()?;
        let index = self.history.index_of(id)?;
        if let Some(anchor) = self.transform_gizmo.anchor {
            // A spline anchor: its position + the intrinsic-XYZ Euler of its
            // stored axis triad (see `spline_edit.rs`).
            let (position, rotation_deg) = self.spline_anchor_pose(index, anchor)?;
            return Some(TransformPose {
                position,
                rotation_deg,
                scale: [1.0, 1.0, 1.0],
            });
        }
        let params = self.history.feature_params(index)?;
        let transform = params.get("transform");
        // Build the history's expression environment ONCE for the vectors: a
        // transform or pivot component may be an expression string.
        let env = brep_kernel::Env::build(&self.history.expressions(), &self.history.configurator())
            .ok();
        let env = env.as_ref();
        let pivot = if self.is_xform_feature(index) {
            self.xform_pivot(index, &params)
        } else if self.is_face_transform_feature(index) {
            // No faces, no pivot, no gizmo: there is nothing to put it on yet.
            self.face_transform_pivot(index, &params, env)?
        } else {
            [0.0; 3]
        };
        let position = read_pose_vec3(env, transform, "position", [0.0; 3]);
        Some(TransformPose {
            position: std::array::from_fn(|axis| position[axis] + pivot[axis]),
            rotation_deg: read_pose_vec3(env, transform, "rotationEuler", [0.0, 0.0, 0.0]),
            scale: read_pose_vec3(env, transform, "scale", [1.0, 1.0, 1.0]),
        })
    }

    fn is_xform_feature(&self, index: usize) -> bool {
        self.history.feature_type(index).is_some_and(|ty| {
            ty.eq_ignore_ascii_case("XFORM") || ty.eq_ignore_ascii_case("TRANSFORM")
        })
    }

    /// Whether the feature at `index` is a Transform Face — the feature whose
    /// motion turns about a STORED `pivot` param.
    pub fn is_face_transform_feature(&self, index: usize) -> bool {
        self.history
            .feature_type(index)
            .is_some_and(|ty| is_face_transform_type(&ty))
    }

    /// The point a Transform Face turns about: its stored `pivot`, or — while
    /// that is still null — the kernel's default, the selection's centre over
    /// the scene the feature runs against. `None` when neither exists (no face
    /// resolves), which is what keeps the gizmo off an empty selection.
    fn face_transform_pivot(
        &self,
        index: usize,
        params: &serde_json::Value,
        env: Option<&brep_kernel::Env>,
    ) -> Option<[f64; 3]> {
        match params.get("pivot") {
            Some(pivot) if !pivot.is_null() => Some(read_pose_vec3(env, Some(params), "pivot", [0.0; 3])),
            _ => self.face_transform_default_pivot(index, params),
        }
    }

    /// Whether the Transform Face `feature_id` has a pivot to put its gizmo on —
    /// a stored one, or faces that resolve to a default. The history panel arms
    /// the gizmo on the frame this first holds.
    pub fn face_transform_has_pivot(&self, feature_id: &str) -> bool {
        let Some(index) = self.history.index_of(feature_id) else {
            return false;
        };
        let Some(params) = self.history.feature_params(index) else {
            return false;
        };
        self.is_face_transform_feature(index) && self.face_transform_pivot(index, &params, None).is_some()
    }

    /// Transform Face's DEFAULT pivot for `params.faces`: the prefix BEFORE the
    /// feature is replayed (warm, like [`Self::xform_pivot`]'s) and handed to
    /// [`brep_kernel::face_transform_pivot`] — the same resolution and centre the
    /// feature itself falls back to, so the stored value and the kernel agree by
    /// construction.
    fn face_transform_default_pivot(
        &self,
        index: usize,
        params: &serde_json::Value,
    ) -> Option<[f64; 3]> {
        let names = brep_kernel::reference_names(params.get("faces"));
        if names.is_empty() {
            return None;
        }
        let request: HistoryRequest =
            serde_json::from_value(self.history.request_before(index)?).ok()?;
        let _trace = crate::run_trace::span("face_transform_pivot");
        let result = brep_kernel::execute_history(&request);
        brep_kernel::face_transform_pivot(&result.results, &names)
    }

    /// STORE a Transform Face's pivot into the params about to be written for
    /// the feature at `index`: when this write changes the face selection
    /// without also setting a pivot, or leaves the pivot null, `pivot` becomes the
    /// new selection's centre (null again when nothing resolves). A pivot the
    /// write sets itself is kept. Stored rather than derived per run, so the
    /// rotation centre is a number in the document: a re-run, a gizmo drag, or an
    /// upstream edit never moves it. Every lane that writes feature params calls
    /// this before the write (`update_feature_params`, the batch lane, the
    /// reference picker's Finish).
    pub(crate) fn stamp_face_transform_pivot(&self, index: usize, params: &mut serde_json::Value) {
        if !self.is_face_transform_feature(index) {
            return;
        }
        let before = self.history.feature_params(index).unwrap_or(serde_json::Value::Null);
        let faces_changed = brep_kernel::reference_names(before.get("faces"))
            != brep_kernel::reference_names(params.get("faces"));
        let pivot_set_here = before.get("pivot") != params.get("pivot");
        let unset = params.get("pivot").is_none_or(serde_json::Value::is_null);
        if !(unset || (faces_changed && !pivot_set_here)) {
            return;
        }
        let pivot = self.face_transform_default_pivot(index, params);
        if let Some(object) = params.as_object_mut() {
            object.insert(
                "pivot".into(),
                pivot.map_or(serde_json::Value::Null, |pivot| serde_json::json!(pivot)),
            );
        }
    }

    /// XFORM rotates each selected solid about its own source vertex-bbox center.
    /// Anchor the shared controls to the first resolved solid's pivot; the same
    /// translation/rotation delta still applies to every selected solid.
    fn xform_pivot(&self, index: usize, params: &serde_json::Value) -> [f64; 3] {
        if !params["pivot"]
            .as_str()
            .is_some_and(|p| p.eq_ignore_ascii_case("BBOX_CENTER"))
        {
            return [0.0; 3];
        }
        // Replay the warm current prefix, then fold only results BEFORE XFORM.
        // Using the transformed scene bbox would make the pivot drift on rotation.
        let Ok(request) = serde_json::from_value::<HistoryRequest>(self.run_request_value())
        else {
            return [0.0; 3];
        };
        let _trace = crate::run_trace::span("xform_pivot");
        let result = brep_kernel::execute_history(&request);
        let mut handles = HashMap::new();
        for feature in result.results.iter().take(index) {
            for name in &feature.removed {
                handles.remove(name);
            }
            for solid in &feature.added {
                handles.insert(solid.name.clone(), solid.handle);
            }
        }
        let Some(refs) = params["solids"].as_array() else {
            return [0.0; 3];
        };
        for reference in refs {
            let Some(name) = reference.as_str().or_else(|| reference["name"].as_str()) else {
                continue;
            };
            let Some(handle) = handles.get(name.trim()) else {
                continue;
            };
            if let Ok(center) = brep_kernel::transform_pivot_native(*handle) {
                return center;
            }
        }
        [0.0; 3]
    }

    /// (Re)feed the widget gizmo at the armed feature's frame: origin =
    /// `position`, axes = the feature's rotated basis (intrinsic XYZ Euler order, matching
    /// the kernel bake). Auto-disarms if the feature vanished. Called every drag
    /// frame from `transform_drag_to` so the widget tracks the moving pose live
    /// (Fix 3); the drag delta resolves against the frozen grab frame, so this
    /// re-sync never feeds back into the drag math.
    pub fn sync_transform_gizmo(&mut self) {
        // Only the TRANSFORM mode feeds the move/rotate widget; in dimension mode
        // the widget stays hidden (the annotations render as an overlay instead).
        if !matches!(self.transform_gizmo.mode, GizmoMode::Transform) {
            return;
        }
        let Some(pose) = self.armed_pose() else {
            self.disarm_transform();
            return;
        };
        let _ = self
            .widgets
            .set_transform_json(&transform_frame_json(&pose));
        self.dirty = true;
    }

    /// Whether the feature schema exposes the shared Transform controls.
    /// Capability belongs to the schema, not to which default-valued fields
    /// happen to have been serialized in the model.
    pub fn feature_has_transform(&self, feature_id: &str) -> bool {
        self.history
            .index_of(feature_id)
            .and_then(|i| self.history.feature_type(i))
            .and_then(|ty| crate::features::feature_schema(&ty))
            .is_some_and(|schema| schema["inputParamsSchema"]["transform"]["type"] == "transform")
    }

    /// The armed gizmo origin projected to VIEWPORT-LOCAL px (the center handle
    /// sits here). The history panel publishes it so the headed verifier can
    /// locate + drag the gizmo. `None` when disarmed / not projectable (the ONE
    /// [`crate::view::ViewCamera::projectable`] policy — ortho always projects).
    pub fn transform_gizmo_anchor(&self) -> Option<(f64, f64)> {
        let pose = self.armed_pose()?;
        let (sx, sy, _) = self.camera.project(pose.position);
        self.camera.projectable(pose.position).then_some((sx, sy))
    }

    /// The transform gizmo's axis-end labels as JSON:
    /// `[{ text:"XC"|"YC"|"ZC", rgb:[r,g,b], world:[x,y,z] }]`. The app projects
    /// each `world` point and draws the colored egui label just past the matching
    /// cone tip (X=red, Y=green, Z=blue). `[]` unless the TRANSFORM gizmo is armed.
    pub fn transform_axis_labels_json(&self) -> String {
        if !matches!(self.transform_gizmo.mode, GizmoMode::Transform) {
            return "[]".to_string();
        }
        let Some(pose) = self.armed_pose() else {
            return "[]".to_string();
        };
        let euler = [
            pose.rotation_deg[0].to_radians(),
            pose.rotation_deg[1].to_radians(),
            pose.rotation_deg[2].to_radians(),
        ];
        let axes = [
            normalize3(rotate_euler_xyz_f64([1.0, 0.0, 0.0], euler)),
            normalize3(rotate_euler_xyz_f64([0.0, 1.0, 0.0], euler)),
            normalize3(rotate_euler_xyz_f64([0.0, 0.0, 1.0], euler)),
        ];
        // Just past the cone tip, screen-constant.
        let gap_px = 12.0_f64;
        let dist =
            (brep_gizmos::transform::PX_AXIS_LEN as f64 + gap_px) * self.camera.world_per_pixel();
        let labels: [(&str, [f32; 3]); 3] = [
            ("XC", [0.92, 0.26, 0.28]), // red
            ("YC", [0.30, 0.78, 0.36]), // green
            ("ZC", [0.30, 0.52, 0.98]), // blue
        ];
        let out: Vec<serde_json::Value> = (0..3)
            .map(|i| {
                let o = pose.position;
                let a = axes[i];
                serde_json::json!({
                    "text": labels[i].0,
                    "rgb": labels[i].1,
                    "world": [o[0] + a[0] * dist, o[1] + a[1] * dist, o[2] + a[2] * dist],
                })
            })
            .collect();
        serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string())
    }

    /// A press point for every AXIS ARROW and ROTATION GRAB of the drawn
    /// transform gizmo, in VIEWPORT-LOCAL px, keyed `axis:x|y|z` and
    /// `ring:x|y|z` (a ring is named by the frame axis it turns about). Read off
    /// the SAME regions [`Self::transform_pick`] tests — three quarters of the way
    /// along an arrow's capsule spine, a grab ball's centre — and kept only where
    /// a press there picks THAT handle, so a published key can never grab a
    /// neighbour (an arrow foreshortened onto a ring ball, say). Empty while no
    /// transform widget is drawn. The centre handle is `gizmo/anchor`'s.
    pub fn transform_handle_points(&self) -> Vec<(&'static str, (f64, f64))> {
        use brep_gizmos::hit_region::HitShape;
        use brep_gizmos::transform as gizmo;
        let cam = gizmo_camera(&self.camera);
        self.widgets
            .transform_hit_regions(&cam)
            .into_iter()
            .filter_map(|(handle, shape)| {
                let key = match handle {
                    gizmo::HANDLE_AXIS_X => "axis:x",
                    gizmo::HANDLE_AXIS_Y => "axis:y",
                    gizmo::HANDLE_AXIS_Z => "axis:z",
                    gizmo::HANDLE_RING_X => "ring:x",
                    gizmo::HANDLE_RING_Y => "ring:y",
                    gizmo::HANDLE_RING_Z => "ring:z",
                    _ => return None,
                };
                let [x, y] = match shape {
                    HitShape::Capsule { a, b, .. } => {
                        [a[0] + 0.75 * (b[0] - a[0]), a[1] + 0.75 * (b[1] - a[1])]
                    }
                    HitShape::Circle { c, .. } => c,
                };
                let point = (f64::from(x), f64::from(y));
                (self.transform_pick(point.0, point.1) == handle).then_some((key, point))
            })
            .collect()
    }

    /// Viewport-local pick regions for the debug overlay, shared by feature and
    /// component controllers. The widget feed determines visibility; hidden
    /// widgets return `[]`. The app offsets these circles/capsules by `rect.min`.
    pub fn transform_hit_areas_json(&self) -> String {
        let cam = gizmo_camera(&self.camera);
        let regions = self.widgets.transform_hit_regions(&cam);
        super::camera_widgets::hit_shapes_json(regions.iter().map(|(_, shape)| shape))
    }

    /// Begin a gizmo drag at viewport px `(x, y)` when the gizmo is armed AND a
    /// handle is under the pointer. Returns whether a handle was grabbed — the
    /// viewport routes the drag to the gizmo (not the camera) when `true`; a press
    /// on empty space returns `false` and still orbits.
    pub fn transform_press(&mut self, x: f64, y: f64) -> bool {
        // Only grabbable in TRANSFORM mode — in dimension mode the dimension
        // handles own the pointer (routed by the app), and disarmed grabs nothing.
        if !matches!(self.transform_gizmo.mode, GizmoMode::Transform) {
            return false;
        }
        let handle = self.transform_pick(x, y);
        if handle == 0 {
            return false;
        }
        let Some(pose) = self.armed_pose() else {
            return false;
        };
        self.widgets.set_transform_active(handle);
        self.transform_gizmo.drag = Some(TransformDrag {
            handle,
            sx: x as f32,
            sy: y as f32,
            start: pose,
            last: None,
        });
        self.dirty = true;
        true
    }

    /// Whether a gizmo handle drag is in flight.
    pub fn transform_dragging(&self) -> bool {
        self.transform_gizmo.drag.is_some()
    }

    /// Continue the in-flight gizmo drag to viewport px `(cx, cy)`: resolve the
    /// world delta from the grab (against the frozen grab-time frame), apply it to
    /// the grab pose, write it back into the feature's `transform`, and re-run so
    /// the model follows live. Then re-sync the VISIBLE gizmo to the moved pose so
    /// the widget tracks the pointer in real time (Fix 3) — the delta stays
    /// anchored to `drag.start`, so this visual sync never feeds back on itself.
    pub fn transform_drag_to(&mut self, cx: f64, cy: f64) {
        let Some(drag) = self.transform_gizmo.drag else {
            return;
        };
        let Some(delta) = self.resolve_transform_delta(&drag, cx, cy) else {
            return;
        };
        let pose = apply_transform_delta(&drag.start, &delta);
        // Remember what the handle resolved, so the readout can say it. The
        // drag is re-resolved from `drag.start` every frame, so this is the
        // WHOLE motion since the grab, not an increment to accumulate.
        if let Some(live) = self.transform_gizmo.drag.as_mut() {
            live.last = Some(delta);
        }
        self.write_armed_pose(&pose);
        // Live-follow: re-feed the widget frame to the just-written pose. NB
        // `finish_apply` skips this while a drag is in flight (drag.is_some()), so
        // the sync happens here. The active-handle gold highlight survives (the
        // widget only clears it on a `null` feed).
        self.sync_transform_gizmo();
    }

    /// End the drag: clear the active-handle highlight + re-sync the gizmo to the
    /// feature's final (moved) pose (unpin the frame).
    pub fn transform_release(&mut self) {
        if self.transform_gizmo.drag.take().is_some() {
            self.widgets.set_transform_active(0);
            // ONE UNDO STEP PER DRAG. `set_feature_params` coalesces on
            // `param:{index}` so that the hundred writes a single drag makes do
            // not become a hundred undo entries — but nothing ended the run, so
            // the coalescing went on across drags too: two drags of the same
            // feature were ONE entry, and a single undo after a refused drag
            // threw away the good drag before it as well (measured
            // 2026-09-21 — 0.9254 then 1.8508, one undo, straight to zero).
            // Ending the run here is the same thing the sheet label drag does
            // on release, and it is what makes "undo the drag that refused,
            // keep the one before it" work.
            self.history.break_coalescing();
            self.sync_transform_gizmo();
        }
    }

    /// Resolve against the grab-time frame so live widget updates cannot
    /// feed back into the drag calculation.
    fn resolve_transform_delta(
        &self,
        drag: &TransformDrag,
        cx: f64,
        cy: f64,
    ) -> Option<TransformDelta> {
        let cam = gizmo_camera(&self.camera);
        let frame_json = transform_frame_json(&drag.start);
        let json = self.widgets.transform_drag_json_with_frame(
            &cam,
            &frame_json,
            drag.handle,
            drag.sx,
            drag.sy,
            cx as f32,
            cy as f32,
        );
        parse_drag_delta(&json)
    }

    /// Write `pose` into the armed feature's `inputParams.transform.{position,
    /// rotationEuler}` (preserving every other field, incl. `scale`) and re-run.
    fn write_armed_pose(&mut self, pose: &TransformPose) {
        if let Some(address) = self.transform_gizmo.port.clone() {
            self.write_port_point_pose(&address, pose);
            return;
        }
        let Some(id) = self.transform_gizmo.feature_id.clone() else {
            return;
        };
        let Some(index) = self.history.index_of(&id) else {
            return;
        };
        if let Some(anchor) = self.transform_gizmo.anchor {
            self.write_spline_anchor_pose(&id, anchor, pose.position, pose.rotation_deg);
            return;
        }
        let mut params = self
            .history
            .feature_params(index)
            .unwrap_or_else(|| serde_json::json!({}));
        let pivot = if self.is_xform_feature(index) {
            self.xform_pivot(index, &params)
        } else if self.is_face_transform_feature(index) {
            let env =
                brep_kernel::Env::build(&self.history.expressions(), &self.history.configurator()).ok();
            let Some(pivot) = self.face_transform_pivot(index, &params, env.as_ref()) else {
                return;
            };
            pivot
        } else {
            [0.0; 3]
        };
        let position: [f64; 3] = std::array::from_fn(|axis| pose.position[axis] - pivot[axis]);
        // Ensure `transform` is an object, then set the two edited vectors.
        if !params
            .get("transform")
            .map(|t| t.is_object())
            .unwrap_or(false)
        {
            if let Some(object) = params.as_object_mut() {
                object.insert("transform".into(), serde_json::json!({}));
            }
        }
        if let Some(transform) = params.get_mut("transform").and_then(|t| t.as_object_mut()) {
            transform.insert("position".into(), serde_json::json!(position));
            transform.insert("rotationEuler".into(), serde_json::json!(pose.rotation_deg));
        }
        let _ = self.update_feature_params(&id, &params.to_string());
    }
}

/// Name the axis a world vector lies along — `X`, `Y` or `Z` when it lies along
/// one to within a thousandth of its length, and the neutral `\u{0394}` when it
/// does not.
///
/// The neutral case is not a fallback, it is the honest answer: the X ARROW OF
/// A TURNED FRAME points along the turned X, which has two non-zero world
/// components, and calling that motion "X" would name a world axis the face is
/// not moving along. A zero-length vector is `\u{0394}` too — there is no axis
/// to name yet.
fn dominant_axis_label(v: &[f64; 3]) -> &'static str {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length < 1e-12 {
        return "\u{0394}";
    }
    const NAMES: [&str; 3] = ["X", "Y", "Z"];
    for axis in 0..3 {
        if v[axis].abs() >= length * 0.999 {
            return NAMES[axis];
        }
    }
    "\u{0394}"
}

/// The type strings a Transform Face answers to — the kernel dispatch's alias
/// set, case-insensitively.
pub fn is_face_transform_type(feature_type: &str) -> bool {
    matches!(
        feature_type.trim().to_ascii_uppercase().as_str(),
        "TF" | "TRANSFORM FACE" | "TRANSFORMFACE"
    )
}

/// Apply translation or world-axis rotation to the grab pose, retaining scale.
fn apply_transform_delta(start: &TransformPose, delta: &TransformDelta) -> TransformPose {
    match delta {
        TransformDelta::Translate(world) => TransformPose {
            position: [
                start.position[0] + world[0],
                start.position[1] + world[1],
                start.position[2] + world[2],
            ],
            ..*start
        },
        TransformDelta::Rotate { axis, radians } => {
            let q0 = quat_from_euler_xyz_deg(start.rotation_deg);
            let dq = quat_from_axis_angle(*axis, *radians);
            let nq = quat_mul(dq, q0);
            TransformPose {
                rotation_deg: euler_xyz_deg_from_quat(nq),
                ..*start
            }
        }
    }
}

/// Read a `[x, y, z]` from a transform sub-field (numbers only; missing / short
/// arrays keep the per-index default).
fn read_pose_vec3(
    env: Option<&brep_kernel::Env>,
    transform: Option<&serde_json::Value>,
    key: &str,
    default: [f64; 3],
) -> [f64; 3] {
    let array = transform
        .and_then(|t| t.get(key))
        .and_then(|v| v.as_array());
    let mut out = default;
    if let Some(array) = array {
        for (index, slot) in out.iter_mut().enumerate() {
            match array.get(index) {
                Some(serde_json::Value::Number(number)) => {
                    if let Some(number) = number.as_f64() {
                        *slot = number;
                    }
                }
                // A component may be an EXPRESSION (`"boxW/2"`) — the kernel
                // evaluates it when it builds, so the gizmo must arm at the same
                // place. Reading it as 0 parked the handles at the world origin
                // while the solid sat elsewhere, and the first drag wrote that
                // wrong origin back as a literal, teleporting the solid.
                Some(serde_json::Value::String(source)) => {
                    if let Some(number) = env
                        .and_then(|env| env.eval(source).ok())
                        .filter(|number| number.is_finite())
                    {
                        *slot = number;
                    }
                }
                _ => {}
            }
        }
    }
    out
}

/// The gizmo frame feed (`set_transform_json` shape) for a pose: origin =
/// `position`, axes = the feature's rotated basis (intrinsic XYZ Euler order, matching
/// the kernel bake), with the center free-move handle shown.
fn transform_frame_json(pose: &TransformPose) -> String {
    let euler = [
        pose.rotation_deg[0].to_radians(),
        pose.rotation_deg[1].to_radians(),
        pose.rotation_deg[2].to_radians(),
    ];
    let x = normalize3(rotate_euler_xyz_f64([1.0, 0.0, 0.0], euler));
    let y = normalize3(rotate_euler_xyz_f64([0.0, 1.0, 0.0], euler));
    let z = normalize3(rotate_euler_xyz_f64([0.0, 0.0, 1.0], euler));
    serde_json::json!({
        "origin": pose.position,
        "x": x,
        "y": y,
        "z": z,
        "showCenter": true,
    })
    .to_string()
}

pub(super) fn normalize3(v: [f64; 3]) -> [f64; 3] {
    let length = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if length < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / length, v[1] / length, v[2] / length]
    }
}

/// Apply an intrinsic XYZ Euler (radians) to a vector — the EXACT matrix the kernel
/// bake (`transform_bake` / `datum::rotate_euler_xyz`) uses, so the fed gizmo
/// frame aligns with the baked solid.
pub(crate) fn rotate_euler_xyz_f64(v: [f64; 3], euler: [f64; 3]) -> [f64; 3] {
    let (c1, s1) = (euler[0].cos(), euler[0].sin());
    let (c2, s2) = (euler[1].cos(), euler[1].sin());
    let (c3, s3) = (euler[2].cos(), euler[2].sin());
    let m00 = c2 * c3;
    let m01 = -c2 * s3;
    let m02 = s2;
    let m10 = c1 * s3 + c3 * s1 * s2;
    let m11 = c1 * c3 - s1 * s2 * s3;
    let m12 = -c2 * s1;
    let m20 = s1 * s3 - c1 * c3 * s2;
    let m21 = c3 * s1 + c1 * s2 * s3;
    let m22 = c1 * c2;
    [
        m00 * v[0] + m01 * v[1] + m02 * v[2],
        m10 * v[0] + m11 * v[1] + m12 * v[2],
        m20 * v[0] + m21 * v[1] + m22 * v[2],
    ]
}

// --- quaternion helpers (ported from CombinedTransformControls) -----

pub(super) type Quat = [f64; 4]; // [x, y, z, w]

pub(super) fn quat_from_axis_angle(axis: [f64; 3], angle: f64) -> Quat {
    let n = normalize3(axis);
    let half = angle * 0.5;
    let s = half.sin();
    [n[0] * s, n[1] * s, n[2] * s, half.cos()]
}

/// Quaternion from an intrinsic XYZ Euler (degrees in).
pub(super) fn quat_from_euler_xyz_deg(deg: [f64; 3]) -> Quat {
    let (c1, s1) = (
        (deg[0].to_radians() * 0.5).cos(),
        (deg[0].to_radians() * 0.5).sin(),
    );
    let (c2, s2) = (
        (deg[1].to_radians() * 0.5).cos(),
        (deg[1].to_radians() * 0.5).sin(),
    );
    let (c3, s3) = (
        (deg[2].to_radians() * 0.5).cos(),
        (deg[2].to_radians() * 0.5).sin(),
    );
    [
        s1 * c2 * c3 + c1 * s2 * s3,
        c1 * s2 * c3 - s1 * c2 * s3,
        c1 * c2 * s3 + s1 * s2 * c3,
        c1 * c2 * c3 - s1 * s2 * s3,
    ]
}

/// Quaternion product `a * b`.
pub(super) fn quat_mul(a: Quat, b: Quat) -> Quat {
    [
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    ]
}

/// Intrinsic XYZ Euler from a quaternion (via the rotation matrix) → the
/// 'XYZ' Euler in DEGREES. Uses the SAME matrix element naming as
/// `rotate_euler_xyz_f64`, so the round-trip is consistent with the kernel bake.
pub(super) fn euler_xyz_deg_from_quat(q: Quat) -> [f64; 3] {
    let [x, y, z, w] = q;
    let (x2, y2, z2) = (x + x, y + y, z + z);
    let (xx, xy, xz) = (x * x2, x * y2, x * z2);
    let (yy, yz, zz) = (y * y2, y * z2, z * z2);
    let (wx, wy, wz) = (w * x2, w * y2, w * z2);
    // Rotation matrix elements (m<row><col> naming).
    let m11 = 1.0 - (yy + zz);
    let m12 = xy - wz;
    let m13 = xz + wy;
    let m22 = 1.0 - (xx + zz);
    let m23 = yz - wx;
    let m32 = yz + wx;
    let m33 = 1.0 - (xx + yy);
    let ey = m13.clamp(-1.0, 1.0).asin();
    let (ex, ez) = if m13.abs() < 0.9999999 {
        ((-m23).atan2(m33), (-m12).atan2(m11))
    } else {
        (m32.atan2(m22), 0.0)
    };
    [ex.to_degrees(), ey.to_degrees(), ez.to_degrees()]
}

// ===========================================================================
// The gizmo on a CONNECTION POINT
// ===========================================================================
//
// A connection point is not a feature: it has no history row and no id, so
// every read and write above that goes through `history.index_of` misses it.
// What it has instead is a SEAT — the frame its declared `transform` is an
// offset in (`brep_kernel::ports`) — and the gizmo lane is exactly that seat
// applied and un-applied:
//
//   pose  = seat  o  declaration      (what the handles are drawn at)
//   write = seat⁻¹ o  dragged pose    (what the block is given back)
//
// The seat comes from the LAST APPLIED RUN's report and the declaration from
// the block as it stands NOW. That split is deliberate and it is what keeps a
// drag smooth: `set_ports_block` re-runs on the NEXT frame, so a pose read
// from the run's resolved position would lag the pointer by a frame and
// oscillate. The seat does not move while the offset is dragged — only the
// reference geometry moves it — so reading it from the last run is exact.

impl EngineState {
    /// Whether the transform gizmo is armed on a connection point, and which.
    pub fn transform_armed_port_point(&self) -> Option<&str> {
        self.transform_armed()
            .then(|| self.transform_gizmo.port.as_deref())
            .flatten()
    }

    /// Arm the TRANSFORM gizmo on the connection point at `address`, or refuse
    /// and say why ([`Self::port_point_gizmo_refusal`]).
    pub fn arm_transform_for_port_point(&mut self, address: &str) -> bool {
        if let Some(reason) = self.port_point_gizmo_refusal(address) {
            self.push_notice(format!("no handles on {address}: {reason}"));
            return false;
        }
        self.component_move_reset();
        self.transform_gizmo.feature_id = None;
        self.transform_gizmo.anchor = None;
        self.transform_gizmo.port = Some(address.to_string());
        self.transform_gizmo.mode = GizmoMode::Transform;
        self.transform_gizmo.drag = None;
        self.clear_feature_dimension_overlay();
        self.sync_transform_gizmo();
        true
    }

    /// Why the gizmo will not arm on this point, in words, or `None` when it
    /// will. There are two reasons, and both are cases where the handles would
    /// LIE — draw somewhere a drag cannot write to:
    ///
    /// * the point **maps down**. Its placement is its target's: the kernel's
    ///   `resolve_point` returns on `mapsTo` before it ever reads the
    ///   transform, so handles would sit on the descendant's geometry and a
    ///   drag would write numbers nothing reads, leaving the point where it
    ///   was while the handles walked away from it;
    /// * an **expression** places it. The gizmo writes numbers, and replacing
    ///   `width/2` with `12.5` because a user dragged something is a loss the
    ///   document cannot show them. Typing in the panel still works, which is
    ///   where an expression belongs.
    ///
    /// Public because the panel must say the same thing the arm decides rather
    /// than inferring a reason from the gizmo not being armed: it is drawn a
    /// moment BEFORE the arm runs, so an inference would flash a refusal on
    /// the frame a perfectly armable point is selected.
    pub fn port_point_gizmo_refusal(&self, address: &str) -> Option<String> {
        let point = self.declared_point_value(address)?;
        if let Some(target) = point
            .get("mapsTo")
            .and_then(serde_json::Value::as_str)
            .map(str::trim)
            .filter(|target| !target.is_empty())
        {
            return Some(format!(
                "it maps to '{target}', and that point's placement is this one's"
            ));
        }
        let transform = point.get("transform")?;
        for key in ["position", "rotationEuler"] {
            let Some(components) = transform.get(key).and_then(serde_json::Value::as_array) else {
                continue;
            };
            for component in components {
                if let Some(expression) = component.as_str() {
                    return Some(format!(
                        "'{expression}' places it, and a drag writes numbers"
                    ));
                }
            }
        }
        None
    }

    /// The seat a connection point's placement is an offset in: the last
    /// applied run's, or the WORLD seat for a point no run has resolved yet
    /// (one just added, which must still be draggable).
    fn port_point_seat(&self, address: &str) -> brep_kernel::Frame {
        self.port_point_row(address)
            .map(|row| row.seat.to_frame())
            .unwrap_or_else(brep_kernel::world_seat)
    }

    /// The armed connection point's pose in WORLD space — the seat applied to
    /// the declaration, per this section's opening note.
    pub(super) fn port_point_pose(&self, address: &str) -> Option<TransformPose> {
        let point = self.declared_point_value(address)?;
        let transform = point.get("transform");
        let local_position = read_pose_vec3(None, transform, "position", [0.0; 3]);
        let local_rotation = read_pose_vec3(None, transform, "rotationEuler", [0.0; 3]);
        let seat = self.port_point_seat(address);
        let world_axes = super::spline_edit::axes_from_euler_deg(local_rotation)
            .map(|axis| seat_axis(&seat, axis));
        Some(TransformPose {
            position: array3(brep_kernel::seat_point(&seat, vec3(local_position))),
            rotation_deg: super::spline_edit::euler_xyz_deg_from_axes(
                world_axes[0],
                world_axes[1],
                world_axes[2],
            ),
            scale: [1.0, 1.0, 1.0],
        })
    }

    /// Write a dragged WORLD pose back as the point's offset in its seat.
    /// Coalesced per address, so a whole drag is one undo step.
    pub(super) fn write_port_point_pose(&mut self, address: &str, pose: &TransformPose) {
        let seat = self.port_point_seat(address);
        let position = array3(brep_kernel::unseat_point(&seat, vec3(pose.position)));
        let local_axes = super::spline_edit::axes_from_euler_deg(pose.rotation_deg)
            .map(|axis| unseat_axis(&seat, axis));
        let rotation = super::spline_edit::euler_xyz_deg_from_axes(
            local_axes[0],
            local_axes[1],
            local_axes[2],
        );
        let key = format!("port-point-pose:{address}");
        self.edit_port_point(address, Some(&key), |point| {
            point.insert(
                "transform".into(),
                serde_json::json!({ "position": position, "rotationEuler": rotation }),
            );
        });
    }
}

fn vec3(a: [f64; 3]) -> brep_kernel::Vec3 {
    brep_kernel::Vec3::new(a[0], a[1], a[2])
}

fn array3(v: brep_kernel::Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// A local axis carried out through the seat's basis.
fn seat_axis(seat: &brep_kernel::Frame, axis: [f64; 3]) -> [f64; 3] {
    array3(brep_kernel::seat_vector(seat, vec3(axis)))
}

/// The same axis read back in the seat's basis.
fn unseat_axis(seat: &brep_kernel::Frame, axis: [f64; 3]) -> [f64; 3] {
    array3(brep_kernel::unseat_vector(seat, vec3(axis)))
}

