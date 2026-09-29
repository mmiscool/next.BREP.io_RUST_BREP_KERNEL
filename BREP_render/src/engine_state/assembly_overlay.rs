//! Assembly-constraint leaders and draggable distance/angle annotations.
//!
//! Overlays refresh after history application and constraint mutations. Camera
//! zoom changes rebake their screen-sized geometry; settings and sketch mode
//! control visibility. Drags preview locally, then commit and solve on release.
//! Plane-based distances use the signed offset along the base-face normal.
//! Solved poses update both resident display solids and the history document.
//!
//! Live-session guards are required before fallible assembly ABI calls:
//! constructing a `JsValue` error can abort on native targets.

use super::feature_dims::closest_t_on_axis;
use super::*;
use crate::constraint_overlays::{
    build_constraint_overlays, constraint_overlay_buffers, status_color, ConstraintOverlay,
    ConstraintOverlayKind,
};
use brep_gizmos::hit_region::{point_region, segment_region, HitShape};

/// The world-space overlay group carrying the constraint leaders + handles.
/// Sits just under the feature-dim gizmo group (10003) so an armed feature
/// gizmo draws over constraint graphics.
const CONSTRAINT_OVERLAY_GROUP: &str = "assembly-constraint-overlay";

impl EngineState {
    // -----------------------------------------------------------------------
    // Refresh / bake
    // -----------------------------------------------------------------------

    /// Rebuild the constraint-overlay cache from the LIVE kernel session and
    /// re-bake the drawn group. Cleared (empty group) while the Show Constraint
    /// Graphics setting is off or a sketch edit is active. A live handle drag's
    /// preview survives the rebuild (re-applied onto the fresh cache).
    pub fn refresh_constraint_overlay(&mut self) {
        if !self.settings.show_constraint_graphics || self.sketch_mode() {
            self.clear_constraint_overlay();
            return;
        }
        let rows = self.assembly_overlay_value();
        let constraints = self
            .assembly_state_value()
            .get("constraints")
            .cloned()
            .unwrap_or(serde_json::Value::Null);
        self.refresh_constraint_overlay_values(&rows, &constraints);
    }

    /// [`Self::refresh_constraint_overlay`] with explicit payloads (the parsed
    /// `assembly_overlay_json` array + `assembly_state_json` object) — the
    /// testable seam: canned payloads exercise the whole cache/pick/drag path
    /// without a kernel session.
    pub fn refresh_constraint_overlay_from(&mut self, overlay_json: &str, state_json: &str) {
        let rows: serde_json::Value =
            serde_json::from_str(overlay_json).unwrap_or(serde_json::Value::Null);
        let state: serde_json::Value =
            serde_json::from_str(state_json).unwrap_or(serde_json::Value::Null);
        let constraints = state
            .get("constraints")
            .cloned()
            .unwrap_or(serde_json::Value::Null);
        self.refresh_constraint_overlay_values(&rows, &constraints);
    }

    /// [`Self::refresh_constraint_overlay_from`] on already-parsed values — what
    /// the run's [`crate::pipeline::AssemblySync`] carries.
    fn refresh_constraint_overlay_values(
        &mut self,
        rows: &serde_json::Value,
        constraints: &serde_json::Value,
    ) {
        self.constraint_overlays = build_constraint_overlays(rows, constraints);
        self.apply_constraint_drag_preview();
        self.bake_constraint_overlay();
    }

    /// Clear the cache + the drawn group (setting off / sketch mode / no session).
    fn clear_constraint_overlay(&mut self) {
        let had = !self.constraint_overlays.is_empty() || self.constraint_overlay_wpp != 0.0;
        self.constraint_overlays.clear();
        self.constraint_overlay_wpp = 0.0;
        if had {
            let _ = self.set_overlay_json(
                &serde_json::json!({ "groups": [ { "name": CONSTRAINT_OVERLAY_GROUP } ] })
                    .to_string(),
            );
        }
    }

    /// Bake the cached overlays into the drawn triangle group at the CURRENT
    /// camera zoom (screen-constant sizing) and remember the baked
    /// `world_per_pixel` for [`Self::ensure_constraint_overlay_current`].
    fn bake_constraint_overlay(&mut self) {
        let wpp = self.camera.world_per_pixel();
        let (positions, colors) = constraint_overlay_buffers(&self.constraint_overlays, wpp);
        let _ = self.set_overlay_json(
            &serde_json::json!({
                "groups": [
                    {
                        "name": CONSTRAINT_OVERLAY_GROUP,
                        "renderOrder": 10002,
                        "tris": { "positions": positions, "colors": colors },
                    }
                ]
            })
            .to_string(),
        );
        self.constraint_overlay_wpp = if wpp > 0.0 { wpp } else { f64::MIN_POSITIVE };
    }

    /// Per-frame upkeep (the app viewport calls this once per frame): hide the
    /// group while the setting is off / sketch mode is active, restore it when
    /// they flip back, and re-bake when the camera zoom moved the
    /// `world_per_pixel` materially (>0.5%) so the screen-constant arc/rod
    /// sizing stays pixel-true. Re-bakes only on actual change, so a quiet
    /// frame stays quiet (no dirty loop).
    pub fn ensure_constraint_overlay_current(&mut self) {
        let want = self.settings.show_constraint_graphics && !self.sketch_mode();
        if !want {
            self.clear_constraint_overlay();
            return;
        }
        if self.constraint_overlay_wpp == 0.0 {
            // Hidden → shown transition (toggle flipped back on / sketch exited):
            // pull fresh session state.
            self.refresh_constraint_overlay();
            return;
        }
        if self.constraint_overlays.is_empty() {
            return;
        }
        let wpp = self.camera.world_per_pixel();
        if super::overlay_wpp_stale(self.constraint_overlay_wpp, wpp) {
            self.bake_constraint_overlay();
        }
    }

    /// The cached overlay records (read-only view for the app's label pass +
    /// tests).
    pub fn constraint_overlays(&self) -> &[ConstraintOverlay] {
        &self.constraint_overlays
    }

    /// The label feed for the app's chip pass:
    /// `[{id, type, icon, text, status, message, color:[r,g,b], world:[x,y,z],
    /// draggable, selected}]`. `text` leads with `icon` (the type's glyph), so
    /// the chip is a picture plus the measure; `id` is for the hover tooltip. One row per cached overlay with a resolvable label anchor
    /// (the leader midpoint / arc mid-sweep / anchor midpoint). `selected`
    /// marks the label-click-selected constraint (thicker chip border). Empty
    /// while hidden.
    pub fn constraint_labels_json(&self) -> String {
        let wpp = self.camera.world_per_pixel();
        let rows: Vec<serde_json::Value> = self
            .constraint_overlays
            .iter()
            .filter_map(|overlay| {
                let world = overlay.label_anchor(wpp)?;
                let color = status_color(&overlay.status);
                Some(serde_json::json!({
                    "id": overlay.id,
                    "type": overlay.constraint_type,
                    "icon": overlay.icon,
                    "text": overlay.label_text(),
                    "status": overlay.status,
                    "message": overlay.message,
                    "color": [color[0], color[1], color[2]],
                    "world": world,
                    "draggable": overlay.draggable,
                    "selected": self.selected_constraint.as_deref() == Some(overlay.id.as_str()),
                }))
            })
            .collect();
        serde_json::Value::Array(rows).to_string()
    }

    // -----------------------------------------------------------------------
    // Label hover (highlight referenced geometry) + click (expand in panel)
    // -----------------------------------------------------------------------

    /// Hover a constraint LABEL: highlight the referenced elements
    /// (`inputParams.elements`) through the existing emphasis machinery —
    /// faces/edges by kernel name, a `{solid}@x,y,z` vertex ref or a bare
    /// component id by its owning solid(s). Deduped by constraint id so a held
    /// hover never re-bumps the emphasis generation. Sets the one-frame
    /// [`Self::take_constraint_label_hover`] flag either way so the viewport's
    /// scene-hover pass yields.
    pub fn constraint_hover(&mut self, id: &str) {
        self.constraint_label_hover_active = true;
        if self.constraint_hovered.as_deref() == Some(id) {
            return;
        }
        let elements = match self.constraint_overlays.iter().find(|o| o.id == id) {
            Some(overlay) => overlay.elements.clone(),
            None => Vec::new(),
        };
        // Classify each element against the display scene FIRST (immutable
        // reads), then write the emphasis in one go.
        let mut solids: Vec<String> = Vec::new();
        let mut faces: Vec<String> = Vec::new();
        let mut edges: Vec<String> = Vec::new();
        for element in &elements {
            if let Some(at) = element.find('@') {
                // Vertex ref "{solid}@x,y,z" → highlight the owning solid.
                solids.push(element[..at].to_string());
                continue;
            }
            if self.scene_has_face(element) {
                faces.push(element.clone());
            } else if self.scene_has_edge(element) {
                edges.push(element.clone());
            } else if self.scene.solids().iter().any(|s| s.name == *element) {
                solids.push(element.clone());
            } else {
                // A bare component ref (`ACOMP2`) → its member solids
                // (`ACOMP2:…`, the namespaced-prefix convention).
                let prefix = format!("{element}:");
                solids.extend(
                    self.scene
                        .solids()
                        .iter()
                        .filter(|s| s.name.starts_with(&prefix))
                        .map(|s| s.name.clone()),
                );
            }
        }
        self.clear_hover();
        self.emphasis.hovered_solids.extend(solids);
        self.emphasis.hovered_faces.extend(faces);
        self.emphasis.hovered_edges.extend(edges);
        self.emphasis.generation = self.emphasis.generation.wrapping_add(1);
        self.constraint_hovered = Some(id.to_string());
        self.dirty = true;
    }

    /// The pointer left the constraint labels: drop the element highlight (only
    /// if a label hover set one — never clobbers an unrelated scene hover).
    pub fn constraint_hover_end(&mut self) {
        if self.constraint_hovered.take().is_some() {
            self.clear_hover();
        }
    }

    /// Consume the one-frame "a constraint label is hovering elements" flag —
    /// the viewport's modeling hover branch skips its scene re-hover while set
    /// (mirrors [`Self::take_sketch_list_hover`]) so the label-driven highlight
    /// survives the frame without a per-frame clobber/re-apply dirty loop.
    pub fn take_constraint_label_hover(&mut self) -> bool {
        std::mem::take(&mut self.constraint_label_hover_active)
    }

    /// A constraint LABEL was clicked: SELECT that constraint (the chip gains
    /// its selected accent, the context bar offers Delete constraint) and OPEN
    /// it through the ACCORDION open ([`Self::assembly_set_constraint_open`] —
    /// every other constraint closes, so the clicked one is the ONE open
    /// constraint, which is what puts the panel into its dialog). Guarded on the
    /// id existing in the live session (the fallible export's error path would
    /// abort off-wasm).
    pub fn constraint_label_clicked(&mut self, id: &str) {
        if !self.constraint_select(id) {
            return;
        }
        let _ = self.assembly_set_constraint_open(id, true);
    }

    /// SELECT a constraint without opening it — the Constraints tree's single
    /// click. The same selection a label click makes (the chip's selected
    /// accent, the context bar's Delete constraint), minus the dialog. Returns
    /// false, changing nothing, for an id the live session does not hold.
    pub fn constraint_select(&mut self, id: &str) -> bool {
        if !live_constraint_exists(id) {
            return false;
        }
        self.selected_constraint = Some(id.to_string());
        self.dirty = true;
        true
    }

    /// The constraint SELECTED via its viewport label (or `None`), pruned
    /// against the live session — a deleted / undone constraint never lingers
    /// as selected.
    pub fn selected_constraint(&self) -> Option<String> {
        let id = self.selected_constraint.as_deref()?;
        live_constraint_exists(id).then(|| id.to_string())
    }

    /// Drop the constraint selection (the Clear action, Esc, and the context
    /// bar's delete all route here — directly or via `clear_selection`).
    pub fn constraint_deselect(&mut self) {
        if self.selected_constraint.take().is_some() {
            self.dirty = true;
        }
    }

    pub(crate) fn scene_has_face(&self, name: &str) -> bool {
        self.scene
            .solids()
            .iter()
            .any(|solid| solid.faces.iter().any(|face| face.name == name))
    }

    pub(crate) fn scene_has_edge(&self, name: &str) -> bool {
        self.scene
            .solids()
            .iter()
            .any(|solid| solid.edges.iter().any(|edge| edge.name == name))
    }

    // -----------------------------------------------------------------------
    // Grabbable handles: pick + drag preview + commit
    // -----------------------------------------------------------------------

    /// The SCREEN-space pickable regions of the DRAGGABLE constraint handles,
    /// each paired with its constraint id: a distance leader's whole CAPSULE
    /// (`point_a → point_b`), an angle arc's sweep-end handle CIRCLE — the same
    /// region family (and the same generous `ARROW_HANDLE_HIT_RAD_PX`) as the
    /// feature-dimension arrows, built by the shared `brep_gizmos::hit_region`
    /// projectors. Non-draggable overlays (expression-valued params, leader-only
    /// types) contribute nothing.
    fn constraint_hit_regions(&self) -> Vec<(String, HitShape)> {
        if !self.settings.show_constraint_graphics || self.sketch_mode() {
            return Vec::new();
        }
        let wpp = self.camera.world_per_pixel();
        let radius = crate::feature_dimensions::ARROW_HANDLE_HIT_RAD_PX as f32;
        let cam = &self.camera;
        let mut out = Vec::new();
        for overlay in &self.constraint_overlays {
            if !overlay.draggable {
                continue;
            }
            let Some(annotation) = &overlay.annotation else {
                continue;
            };
            let shape = match overlay.kind {
                ConstraintOverlayKind::Distance => {
                    segment_region(cam, annotation.point_a, annotation.point_b, radius)
                }
                ConstraintOverlayKind::Angle => point_region(
                    cam,
                    crate::feature_dimensions::arrow_handle_point(annotation, wpp),
                    radius,
                ),
                ConstraintOverlayKind::Leader => None,
            };
            if let Some(shape) = shape {
                out.push((overlay.id.clone(), shape));
            }
        }
        out
    }

    /// Whether a screen-px pick lands on a draggable constraint handle; returns
    /// the grabbed constraint's id (the NEAREST containing region wins). The
    /// viewport routes a drag that starts here into the constraint drag, and
    /// swallows a bare click so the solid behind the arrow isn't selected.
    pub fn constraint_arrow_pick(&self, x: f64, y: f64) -> Option<String> {
        let p = [x as f32, y as f32];
        let mut best: Option<(f32, String)> = None;
        for (id, shape) in self.constraint_hit_regions() {
            let d = shape.spine_distance(p);
            if d <= shape.radius() && best.as_ref().map(|(bd, _)| d < *bd).unwrap_or(true) {
                best = Some((d, id));
            }
        }
        best.map(|(_, id)| id)
    }

    /// The GRAB POINT of every draggable constraint handle, as
    /// `{constraint id: [x, y]}` in VIEWPORT-LOCAL px — the point a host has to
    /// press to start the drag [`Self::constraint_drag_begin`] claims, published
    /// so the arrow is addressable BY NAME instead of by projection arithmetic.
    ///
    /// The point is [`crate::feature_dimensions::arrow_handle_point`] — a
    /// distance arrow's TIP, an angle arc's sweep-end handle — projected through
    /// the same [`point_region`] builder the hit regions come from, so the
    /// published point and the pickable region can never drift: a press here is
    /// at distance ZERO from the handle's own region.
    ///
    /// Empty while the constraint graphics are hidden (the setting is off, or a
    /// sketch edit is active) and for every overlay that has no draggable
    /// handle — a leader-only type, or a dimensional one whose value is an
    /// expression. The app publishes it as the `constraint` hit panel, in SCREEN
    /// points.
    pub fn constraint_handle_points_json(&self) -> String {
        if !self.settings.show_constraint_graphics || self.sketch_mode() {
            return "{}".to_string();
        }
        let wpp = self.camera.world_per_pixel();
        let radius = crate::feature_dimensions::ARROW_HANDLE_HIT_RAD_PX as f32;
        let cam = &self.camera;
        let mut out = serde_json::Map::new();
        for overlay in &self.constraint_overlays {
            if !overlay.draggable || matches!(overlay.kind, ConstraintOverlayKind::Leader) {
                continue;
            }
            let Some(annotation) = &overlay.annotation else {
                continue;
            };
            let point = crate::feature_dimensions::arrow_handle_point(annotation, wpp);
            if let Some(HitShape::Circle { c, .. }) = point_region(cam, point, radius) {
                out.insert(
                    overlay.id.clone(),
                    serde_json::json!([c[0] as f64, c[1] as f64]),
                );
            }
        }
        serde_json::Value::Object(out).to_string()
    }

    /// Begin a constraint-handle drag at screen `(x, y)`. Returns whether a
    /// draggable handle was grabbed (the viewport then routes the drag here
    /// instead of orbiting the camera).
    pub fn constraint_drag_begin(&mut self, x: f64, y: f64) -> bool {
        let Some(id) = self.constraint_arrow_pick(x, y) else {
            return false;
        };
        // The release commits straight into the live kernel session (no rerun —
        // the movedSolids seam refreshes the scene), so the session has to exist
        // on THIS thread before the drag starts.
        self.ensure_assembly_session();
        let Some(overlay) = self.constraint_overlays.iter().find(|o| o.id == id) else {
            return false;
        };
        let Some(field) = overlay.field_key() else {
            return false;
        };
        let preview = overlay
            .annotation
            .as_ref()
            .map(|a| a.value)
            .or(overlay.value)
            .unwrap_or(0.0);
        self.constraint_drag = Some(ConstraintDrag {
            id,
            field,
            params: overlay.input_params.clone(),
            preview,
        });
        true
    }

    /// Drag a grabbed constraint handle to screen `(x, y)`: map the pointer to a
    /// new value (distance: signed offset along the base-face normal for
    /// plane-based rows, magnitude along the leader otherwise; angle: the
    /// shared arc nearest-projection search, folded to the interior 0–180°),
    /// update the PREVIEW (annotation + label track live), and re-bake. No
    /// kernel call — the commit happens on [`Self::constraint_drag_release`].
    pub fn constraint_drag_to(&mut self, x: f64, y: f64) {
        let Some(id) = self.constraint_drag.as_ref().map(|d| d.id.clone()) else {
            return;
        };
        let Some(overlay) = self.constraint_overlays.iter().find(|o| o.id == id) else {
            return;
        };
        let Some(annotation) = overlay.annotation.clone() else {
            return;
        };
        let kind = overlay.kind;
        let new_value = match kind {
            ConstraintOverlayKind::Distance => {
                let a = annotation.point_a;
                let ray = self.camera.pick_ray(x, y);
                let rd = ray.dir;
                let rn = (rd[0] * rd[0] + rd[1] * rd[1] + rd[2] * rd[2]).sqrt();
                if rn < 1e-12 {
                    return;
                }
                let ray_dir = [rd[0] / rn, rd[1] / rn, rd[2] / rn];
                let n = annotation.axis;
                let raw = if n[0] * n[0] + n[1] * n[1] + n[2] * n[2] > 0.5 {
                    // PLANE-BASED row (the builder stashed the base-face
                    // outward unit normal in `axis`, and drew the arrow from
                    // the perpendicular foot in TRUE world scale): the new
                    // value is simply the pointer's SIGNED offset along that
                    // fixed normal from the foot — dragging through the base
                    // face crosses zero into negatives, the same
                    // `d = (P − base)·n̂` definition the kernel solves.
                    // Deliberately no length guard: a zero-distance arrow
                    // (foot == tip, just the origin sphere) still drags.
                    let Some(t) = closest_t_on_axis(a, n, ray.origin, ray_dir) else {
                        return;
                    };
                    t
                } else {
                    // Plane-less row (line/point pairing — no side to be on):
                    // project onto the anchor-to-anchor leader, map world
                    // distance → param via the measured-value / world-length
                    // ratio, and clamp to the MAGNITUDE domain. `value ≈ 0`
                    // (touching pair) has no ratio: use the world distance.
                    let b = annotation.point_b;
                    let axis = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                    let len =
                        (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
                    if len < 1e-9 {
                        return;
                    }
                    let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
                    let Some(t) = closest_t_on_axis(a, dir, ray.origin, ray_dir) else {
                        return;
                    };
                    let ratio = if annotation.value.abs() > 1e-9 {
                        annotation.value / len
                    } else {
                        1.0
                    };
                    (t * ratio).max(0.0)
                };
                (raw * 1e4).round() / 1e4
            }
            ConstraintOverlayKind::Angle => {
                let Some(degrees) = self.angular_drag_degrees(&annotation, x, y) else {
                    return;
                };
                interior_degrees(degrees)
            }
            ConstraintOverlayKind::Leader => return,
        };
        if let Some(drag) = self.constraint_drag.as_mut() {
            drag.preview = new_value;
        }
        self.apply_constraint_drag_preview();
        self.bake_constraint_overlay();
    }

    /// Patch the cached overlay with the live drag's preview value so the drawn
    /// annotation + label track the pointer: a plane-based distance arrow
    /// re-plants its tip at `foot + n̂·preview` (flipping through the base face
    /// for a negative preview), a plane-less leader stretches `point_b` along
    /// its fixed direction, and an angle arc re-sweeps.
    fn apply_constraint_drag_preview(&mut self) {
        let Some(drag) = self.constraint_drag.as_ref() else {
            return;
        };
        let preview = drag.preview;
        let id = drag.id.clone();
        let Some(overlay) = self.constraint_overlays.iter_mut().find(|o| o.id == id) else {
            return;
        };
        let kind = overlay.kind;
        overlay.value = Some(preview);
        if let Some(annotation) = overlay.annotation.as_mut() {
            match kind {
                ConstraintOverlayKind::Distance => {
                    let a = annotation.point_a;
                    let n = annotation.axis;
                    if n[0] * n[0] + n[1] * n[1] + n[2] * n[2] > 0.5 {
                        // Plane-based (base normal in `axis`): the arrow is
                        // drawn in TRUE world scale, so the tip is EXACTLY
                        // `foot + n̂·value` — a negative preview lands it on
                        // the far side of the base face, exactly where the
                        // solve will put the constrained element.
                        annotation.point_b = [
                            a[0] + n[0] * preview,
                            a[1] + n[1] * preview,
                            a[2] + n[2] * preview,
                        ];
                    } else {
                        let b = annotation.point_b;
                        let axis = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                        let len = (axis[0] * axis[0]
                            + axis[1] * axis[1]
                            + axis[2] * axis[2])
                            .sqrt();
                        if len > 1e-9 && annotation.value.abs() > 1e-9 {
                            // Keep the world-per-value scale the leader had, so
                            // the arrow tip lands where the face will end up.
                            let world_len = preview * (len / annotation.value);
                            let dir = [axis[0] / len, axis[1] / len, axis[2] / len];
                            annotation.point_b = [
                                a[0] + dir[0] * world_len,
                                a[1] + dir[1] * world_len,
                                a[2] + dir[2] * world_len,
                            ];
                        }
                    }
                    annotation.value = preview;
                }
                _ => {
                    annotation.value = preview;
                }
            }
        }
    }

    /// The pending commit `(constraint id, inputParams JSON)` a release would
    /// send — the drag's params snapshot with the dragged field set to the
    /// preview value. `None` when no drag is live. (The pure half of
    /// [`Self::constraint_drag_release`], separated for tests.)
    ///
    /// Angle display convention: the drag preview is the INTERIOR arc sweep
    /// (what the arc draws and the kernel's overlay `value` measures), but the
    /// stored `inputParams.angle` is the DISPLAY angle — exterior-remapped when
    /// the constraint's `exteriorAngle` toggle is on (`map_angle`'s contract) —
    /// so the commit remaps `interior → 180 − interior` for those.
    pub fn constraint_drag_commit_payload(&self) -> Option<(String, String)> {
        let drag = self.constraint_drag.as_ref()?;
        let mut params = drag.params.clone();
        if !params.is_object() {
            params = serde_json::json!({});
        }
        let exterior = drag.field == "angle"
            && params
                .get("exteriorAngle")
                .and_then(|v| v.as_bool())
                .unwrap_or(false);
        let committed = if exterior {
            180.0 - drag.preview
        } else {
            drag.preview
        };
        let object = params.as_object_mut().expect("object ensured above");
        object.insert(drag.field.to_string(), serde_json::json!(committed));
        object.insert("id".to_string(), serde_json::json!(drag.id));
        Some((drag.id.clone(), params.to_string()))
    }

    /// Release the constraint-handle drag: COMMIT the previewed value via
    /// `assembly_update_constraint_json` (auto-solves), consume the reply's
    /// `movedSolids` display seam (re-posed resident solids re-tessellate in
    /// place so the release shows this frame), and then run the SAME tail as
    /// every panel-side constraint edit ([`Self::after_constraint_mutation`]):
    /// a checkpointed fold — the drag is one undo entry — and the rerun, which
    /// re-keys the moved instance's cache entry to the pose it now has. A
    /// commit against a session that no longer has the constraint (or no
    /// session at all — canned-payload tests, the native thread-runner seam)
    /// drops the preview with a notice instead.
    pub fn constraint_drag_release(&mut self) {
        let Some((id, params_json)) = self.constraint_drag_commit_payload() else {
            self.constraint_drag = None;
            return;
        };
        self.constraint_drag = None;
        if !live_constraint_exists(&id) {
            self.notices
                .push(format!("Constraint {id}: no live assembly session to update"));
            self.refresh_constraint_overlay();
            return;
        }
        match brep_kernel::assembly_update_constraint_impl(&id, &params_json) {
            Ok(report) => {
                // The display seam first, so the release lands on screen in THIS
                // frame rather than when the rerun's reply does…
                self.consume_assembly_moved_solids(&report);
                // …and then the same tail every other constraint mutation runs:
                // a CHECKPOINTED fold (the drag is one undo entry, like a panel
                // edit) plus the rerun. Both are load-bearing. Without the
                // checkpoint, undo jumps past the drag to whatever edit came
                // before it — deleting the constraint the user had only resized.
                // Without the rerun, the solve's IN-PLACE re-pose leaves the
                // moved instance's geometry under its OLD fingerprint in the
                // feature cache, so a later replay of that pose (an undo, most
                // obviously) hands back the geometry at the dragged pose and the
                // document and the viewport disagree.
                self.after_constraint_mutation();
            }
            // The native door returns its error as text, so a refused update is
            // a notice on every target, not an abort off wasm.
            Err(message) => self.notices.push(format!("Constraint {id}: {message}")),
        }
        self.refresh_constraint_overlay();
        self.dirty = true;
    }

    // -----------------------------------------------------------------------
    // Solve write-backs: movedSolids display seam + the document fold
    // -----------------------------------------------------------------------

    /// Consume a solve report's `movedSolids: [{name, handle}]` display seam:
    /// those resident solids were RE-POSED IN PLACE by the solver (their
    /// producing feature replayed `reused`, so the run delta kept the stale
    /// mesh) — re-tessellate each from its live resident handle and replace its
    /// display, preserving per-solid view state (visibility, color override).
    /// The key is absent on zero-mate reports; absence is tolerated.
    pub fn consume_assembly_moved_solids(&mut self, report_json: &str) {
        let Ok(report) = serde_json::from_str::<serde_json::Value>(report_json) else {
            return;
        };
        let Some(moved) = report.get("movedSolids").and_then(|v| v.as_array()) else {
            return;
        };
        let lod = if self.settings.lod_factor.is_finite() && self.settings.lod_factor > 0.0 {
            self.settings.lod_factor
        } else {
            1.0
        };
        let mut changed = false;
        for entry in moved {
            let Some(name) = entry.get("name").and_then(|v| v.as_str()) else {
                continue;
            };
            let Some(handle) = entry
                .get("handle")
                .and_then(|v| v.as_u64())
                .map(|h| h as u32)
            else {
                continue;
            };
            match brep_kernel::display_payload_handle_native(handle, lod) {
                Ok(payload) => {
                    let mut display = crate::scene::solid_display_from_payload(name, payload);
                    display.source_handle = handle;
                    if let Some(old) = self.scene.solids().iter().find(|s| s.name == name) {
                        display.visible = old.visible;
                    }
                    self.scene.insert_solid(display);
                    // Re-posed under the SAME handle: a drawing sheet holding
                    // this solid's topology holds the pre-solve one.
                    self.scene.drop_exact_solid(name);
                    self.topology_asked.remove(&handle);
                    changed = true;
                }
                Err(error) => self
                    .notices
                    .push(format!("re-tessellate moved solid {name}: {error}")),
            }
        }
        if changed {
            // Colours are NOT hand-copied off the replaced display: this path
            // builds a fresh one per moved solid, whose per-face colours a copy
            // could not carry. Re-derive the whole scene from the metadata store
            // instead — the same seam every other path colours through.
            self.sync_colors_from_metadata();
            self.dirty = true;
        }
    }
}

/// Whether the LIVE kernel session currently holds a constraint with `id` —
/// checked through the infallible `assembly_state_json` export so the guard
/// itself can never hit a fallible export's off-wasm-aborting error path.
fn live_constraint_exists(id: &str) -> bool {
    serde_json::from_str::<serde_json::Value>(&brep_kernel::assembly_state_json())
        .ok()
        .and_then(|state| {
            state.get("constraints").and_then(|list| {
                list.as_array().map(|entries| {
                    entries.iter().any(|entry| {
                        entry
                            .get("inputParams")
                            .and_then(|p| p.get("id"))
                            .and_then(|v| v.as_str())
                            == Some(id)
                    })
                })
            })
        })
        .unwrap_or(false)
}

/// Fold an arc sweep (the shared angular drag search's [-360°, 360°] output)
/// onto the INTERIOR angle domain the constraint mate targets (0–180°,
/// unsigned — `angle_between_deg` symmetry), snapping the sub-0.5° residue of
/// the search's zero-floor to an exact 0 (parallel is a legitimate target).
fn interior_degrees(degrees: f64) -> f64 {
    let folded = degrees.abs() % 360.0;
    let interior = if folded > 180.0 { 360.0 - folded } else { folded };
    if interior < 0.5 {
        0.0
    } else {
        interior
    }
}

