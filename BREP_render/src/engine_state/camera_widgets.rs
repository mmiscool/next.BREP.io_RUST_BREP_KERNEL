use super::*;

impl EngineState {
    /// Per-frame overlay upkeep — the ONE call the app viewport makes each frame
    /// (see `BREP_app/src/viewport/interaction.rs`).
    ///
    /// Everything fed through the general `set_overlay` channel is pre-expanded
    /// into GPU vertices AT FEED TIME (see the "Why not one uniform feed" note in
    /// [`crate::widgets`]), so — unlike the specialized widgets (transform gizmo,
    /// datums, ViewCube), which are rebuilt against the LIVE camera every frame —
    /// a baked overlay group keeps whatever screen-constant sizing it was baked
    /// with. A ZOOM changes `world_per_pixel` and nothing else re-bakes them, so
    /// the draggable gizmos keep their old pixel size — and where a handle's world
    /// position is ITSELF `px × world_per_pixel` (the angular arc), their old
    /// POSITION too, drifting away from the live-computed grab region.
    ///
    /// So: re-bake on a MATERIAL `world_per_pixel` change, keyed on that ONE
    /// quantity rather than on any particular gesture. Every zoom path moves it —
    /// the wheel, [`Self::zoom_to_fit`], [`Self::standard_view`] (which fits), a
    /// viewport [`Self::resize`] — so they are all covered without a per-path
    /// hook. What does NOT move it needs no re-bake, and correctly gets none:
    /// pan and orbit hold the eye→target distance, the ViewCube (face, corner AND
    /// navigation arrow) is a fixed-pivot reorient, and
    /// [`Self::toggle_projection`] preserves apparent size by construction
    /// (`ViewCamera::toggle_projection` solves for the distance/half-height that
    /// keeps `world_per_pixel` — see `projection_toggle_preserves_apparent_size`).
    /// The baked buffers are world-space, so the GPU re-projects them for free.
    /// Re-bakes only on actual change, so a quiet frame stays quiet (no per-frame
    /// dirty loop).
    pub fn ensure_overlays_current(&mut self) {
        // Assembly-constraint leaders + grabbable distance/angle handles (§8.4).
        self.ensure_constraint_overlay_current();
        // The ◎ feature-DIMENSION gizmo: draggable leaders/arrowheads + the
        // angular sweep handle, whose arc radius is itself px × world_per_pixel.
        self.ensure_feature_dimension_overlay_current();
        // Live sketch mode: draggable dimension leaders, constraint glyphs and
        // construction dashes, all sized in pixels at bake time.
        self.ensure_sketch_overlay_current();
        // The active PMI view's annotation graphics (screen-constant arrows
        // that also face the camera).
        self.ensure_pmi_overlay_current();
    }

    /// Frame the whole scene (used right after the first history feed).
    pub fn zoom_to_fit(&mut self) {
        self.camera.zoom_to_fit(&self.scene.bbox(), 1.15);
        self.dirty = true;
    }

    // --- Sizing -----------------------------------------------------------

    /// Update the CSS viewport size (used by all camera math). The physical
    /// framebuffer size + DPR are the presentation shell's concern.
    pub fn resize(&mut self, css_width: f64, css_height: f64) {
        self.camera.width = css_width.max(1.0);
        self.camera.height = css_height.max(1.0);
        self.dirty = true;
    }

    // --- Pointer / wheel ingestion (R22) ----------------------------------

    pub fn pointer_down(&mut self, x: f64, y: f64, button: i32) -> bool {
        // Sketch camera lock: while locked, the view is held flat-on to the sketch
        // plane, so a LEFT press must NOT drive the camera at all — neither orbit
        // (which would tilt off the plane) NOR pan. Suppressing it keeps left-drag
        // free for sketch interaction and leaves pan on right/middle. Modeling mode
        // and the UNLOCKED sketch view (where left orbits) are unaffected.
        if self.sketch_mode()
            && self.sketch_camera_locked
            && button == crate::controls::BUTTON_LEFT
        {
            return false;
        }
        self.controls.pointer_down(x, y, button)
    }

    pub fn pointer_move(&mut self, x: f64, y: f64) -> bool {
        let changed = self.controls.pointer_move(&mut self.camera, x, y);
        if changed {
            self.dirty = true;
        }
        changed
    }

    pub fn pointer_up(&mut self) -> bool {
        self.controls.pointer_up()
    }

    pub fn wheel(&mut self, delta_y: f64, cursor: Option<[f64; 2]>) -> bool {
        let changed = self.controls.wheel(&mut self.camera, delta_y, cursor);
        if changed {
            self.dirty = true;
        }
        changed
    }

    pub fn set_controls_enabled(&mut self, enabled: bool) {
        self.controls.enabled = enabled;
    }

    // --- Camera commands (R21) --------------------------------------------

    pub fn toggle_projection(&mut self) -> &'static str {
        let kind = self.camera.toggle_projection();
        self.dirty = true;
        kind
    }

    pub fn set_projection(&mut self, kind: &str) {
        let is_persp = matches!(self.camera.projection, crate::view::Projection::Perspective { .. });
        let want_persp = kind.to_ascii_lowercase().starts_with("pers");
        if is_persp != want_persp {
            self.camera.toggle_projection();
            self.dirty = true;
        }
    }

    /// Snap to a named view and fit, the name read in [`Self::view_frame`] —
    /// so with a board shown TOP looks down on its component side.
    pub fn standard_view(&mut self, name: &str) -> bool {
        let ok = self.camera.standard_view_in(self.view_frame(), name);
        if ok {
            self.camera.zoom_to_fit(&self.scene.bbox(), 1.15);
            self.dirty = true;
        }
        ok
    }

    /// The frame the named views and the ViewCube are read in: Z-up
    /// ([`ViewFrame::Board`]) while the document shows a board, Y-up otherwise.
    ///
    /// Keyed on the DOCUMENT, not the workbench: the board is seen in 3D from
    /// every workbench that draws the 3D view (Modeling, Assembly…) — the PCB
    /// workbench itself draws its 2D editor there instead — so a frame the PCB
    /// workbench switched on would never be in force where the board is looked
    /// at. A document that has a board is one whose 3D view is about the board.
    pub fn view_frame(&self) -> crate::view::ViewFrame {
        if self.shown_board_ids.is_empty() {
            crate::view::ViewFrame::World
        } else {
            crate::view::ViewFrame::Board
        }
    }

    /// Put the ViewCube in the frame [`Self::view_frame`] reads now, after the
    /// board bodies were fed or removed. The first time a board appears the
    /// camera turns to the board's ISO view and frames it, so a board document
    /// opens looking down on the component side from the front right, as a
    /// board viewer does, rather than at the Y-up startup vantage that shows
    /// the board as a standing wall. Losing the board leaves the camera alone.
    pub(super) fn sync_view_frame(&mut self) {
        let frame = self.view_frame();
        if frame == self.widgets.view_frame() {
            return;
        }
        self.widgets.set_view_frame(frame);
        if frame == crate::view::ViewFrame::Board {
            self.camera.standard_view_in(frame, "ISO");
            self.camera.zoom_to_fit(&self.scene.bbox(), 1.15);
        }
        self.dirty = true;
    }

    pub fn camera_state_json(&self) -> String {
        self.camera.state_json()
    }

    pub fn apply_camera_state_json(&mut self, json: &str) -> Result<(), String> {
        self.camera.apply_state_json(json)?;
        self.dirty = true;
        Ok(())
    }

    pub fn world_per_pixel(&self) -> f64 {
        self.camera.world_per_pixel()
    }

    // --- World → screen (R25) ---------------------------------------------

    /// Project world points to CSS-pixel screen coords for host anchoring. Input
    /// is `[[x,y,z], …]`; output `[[sx, sy, depth, inFront], …]` where inFront
    /// is 1 when a LABEL anchored at the point should draw
    /// ([`crate::view::ViewCamera::label_anchor_visible`], THE one label policy:
    /// the point is projectable — ortho always, perspective unless at/behind the
    /// eye plane, near/far NEVER cull — AND its projection lands inside the
    /// viewport, so an off-screen anchor's chip vanishes instead of clamping to
    /// the viewport edge). Every app label pass (sketch dims, feature dims,
    /// constraint chips, gizmo axis text) keys its skip off THIS flag, so the
    /// policy lives in exactly one place.
    pub fn world_to_screen_json(&self, points_json: &str) -> Result<String, String> {
        let points: Vec<[f64; 3]> = serde_json::from_str(points_json)
            .map_err(|error| format!("world_to_screen points parse: {error}"))?;
        let out: Vec<[f64; 4]> = points
            .into_iter()
            .map(|p| {
                let (sx, sy, depth) = self.camera.project(p);
                let visible = self.camera.label_anchor_visible(p);
                [sx, sy, depth, if visible { 1.0 } else { 0.0 }]
            })
            .collect();
        Ok(serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string()))
    }

    /// The camera matrices for the host overlays' per-frame world→screen /
    /// screen→world hot path: `{ viewProj:[16], viewProjInverse:[16],
    /// viewport:[w,h] }`. Both matrices are column-major (index =
    /// `col*4 + row`); `viewProj` maps world → wgpu clip
    /// (x,y in −1..1, z in 0..1) and `viewport` is the CSS-pixel size. This lets
    /// dimensions + sketch drop the compat mirror camera and read the engine's
    /// own view-projection directly (see `world_to_screen_json` for one-shots).
    pub fn camera_matrices_json(&self) -> String {
        let view_proj = self.camera.view_proj_flat();
        let view_proj_inverse = self.camera.view_proj_inverse_flat();
        serde_json::json!({
            "viewProj": view_proj,
            "viewProjInverse": view_proj_inverse,
            "viewport": [self.camera.width, self.camera.height],
        })
        .to_string()
    }

    // --- Picking (R23/R24) ------------------------------------------------

}

impl EngineState {
    /// Build this frame's overlay-widget geometry, or None when nothing is
    /// enabled (skips the overlay passes entirely).
    pub fn build_widget_overlay(&self) -> Option<WidgetOverlay> {
        if !self.widgets.any_visible() {
            return None;
        }
        Some(self.widgets.build_overlay(&gizmo_camera(&self.camera)))
    }

    /// Fit the per-frame depth window to EVERYTHING drawn, then resolve the GPU
    /// camera — the ONE path both frame loops (wasm `Engine::render`, desktop
    /// `redraw`) use so they can't drift. The overlay is built FIRST, then its
    /// WORLD bounds are folded into the fit: near/far never affect the overlay
    /// geometry (it depends only on view direction + `world_per_pixel`), so
    /// building it before the fit lets construction geometry — datum planes,
    /// world axes, frames, the transform gizmo — be bracketed by the depth
    /// window instead of clipping against the solids-only bounds. The world
    /// ORIGIN is always folded in too, so the origin triad stays bracketed even
    /// when every geometry channel is momentarily empty (an all-empty frame then
    /// yields a tiny origin-centred window — harmless, re-fit next frame). The
    /// ViewCube is excluded (it draws with its own mini-camera; see
    /// [`WidgetOverlay::world_bbox`]). Returns the resolved camera + the built
    /// overlay for the frame to hand to the render core.
    pub fn fit_camera_and_overlay(&mut self) -> (crate::camera::Camera, Option<WidgetOverlay>) {
        let overlay = self.build_widget_overlay();
        let mut depth_bbox = self.depth_range_bbox();
        if let Some(overlay) = &overlay {
            depth_bbox.union(&overlay.world_bbox());
        }
        depth_bbox.expand([0.0, 0.0, 0.0]);
        self.camera.fit_depth_range(&depth_bbox);
        (self.camera.resolve(), overlay)
    }

    pub fn set_datums_json(&mut self, json: &str) -> Result<(), String> {
        self.widgets.set_datums_json(json)?;
        self.dirty = true;
        Ok(())
    }

    /// Feed the general overlay geometry channel (`set_overlay`): arbitrary named
    /// tri/line/point groups (feature-dialog previews and other display-only
    /// geometry), drawn in the widget overlay pass.
    pub fn set_overlay_json(&mut self, json: &str) -> Result<(), String> {
        self.widgets.set_overlay_json(json)?;
        self.dirty = true;
        Ok(())
    }

    pub fn set_dimensions_json(&mut self, json: &str) -> Result<(), String> {
        self.widgets.set_dimensions_json(json)?;
        self.dirty = true;
        Ok(())
    }

    pub fn set_transform_json(&mut self, json: &str) -> Result<(), String> {
        self.widgets.set_transform_json(json)?;
        self.dirty = true;
        Ok(())
    }

    pub fn set_viewcube_enabled(&mut self, enabled: bool) {
        self.widgets.set_viewcube_enabled(enabled);
        self.dirty = true;
    }

    /// The ViewCube corner rect `{x,y,w,h}` (CSS px) so the host can decide
    /// whether to forward a pointer event.
    pub fn viewcube_rect_json(&self) -> String {
        let r = self.widgets.viewcube_rect(&gizmo_camera(&self.camera));
        serde_json::json!({ "x": r[0], "y": r[1], "w": r[2], "h": r[3] }).to_string()
    }

    /// Update the ViewCube hover from cube-local pixels; returns whether it
    /// changed (a hover-out is `(None)` with local coords outside).
    pub fn viewcube_hover(&mut self, local_x: f64, local_y: f64) -> bool {
        let cam = gizmo_camera(&self.camera);
        let handle = self.widgets.viewcube_hit(&cam, local_x as f32, local_y as f32);
        let changed = self.widgets.set_viewcube_hover(handle);
        if changed {
            self.dirty = true;
        }
        changed
    }

    pub fn viewcube_clear_hover(&mut self) -> bool {
        let changed = self.widgets.set_viewcube_hover(None);
        if changed {
            self.dirty = true;
        }
        changed
    }

    /// Click the ViewCube at cube-local pixels: snap the shared camera to the
    /// region's standard view (keeping the current pivot distance). Returns
    /// true if a region was hit.
    pub fn viewcube_click(&mut self, local_x: f64, local_y: f64) -> bool {
        let cam = gizmo_camera(&self.camera);
        let Some(handle) = self.widgets.viewcube_hit(&cam, local_x as f32, local_y as f32) else {
            return false;
        };
        // Navigation arrows apply a RELATIVE camera rotation (orbit / roll) to
        // the current view instead of snapping to an absolute standard view.
        if brep_gizmos::view_cube::ViewCube::is_arrow(handle) {
            self.apply_viewcube_arrow(handle);
            self.dirty = true;
            return true;
        }
        let (dir, fallback_up) = self.widgets.viewcube_target(handle);
        // Minimal-rotation snap with a LEVELLED roll: the view direction snaps to
        // the region, and the up is snapped to the nearest member of a discrete
        // per-kind set (see [`snap_view_up`]) so a face lands flat-on with its
        // bottom edge horizontal and a corner lands on a proper top-vertex-up
        // isometric — always the orientation that rotates the camera the least.
        let kind = brep_gizmos::view_cube::ViewCube::region_kind(handle);
        let dirf = [dir[0] as f64, dir[1] as f64, dir[2] as f64];
        let up = snap_view_up(kind, dirf, self.camera.up, fallback_up);
        self.apply_look_direction(dir, up);
        self.dirty = true;
        true
    }

    /// Reorient the camera to look along `dir` (world eye→target) with `up`,
    /// preserving the current pivot distance.
    pub(super) fn apply_look_direction(&mut self, dir: [f32; 3], up: [f32; 3]) {
        let dir = [dir[0] as f64, dir[1] as f64, dir[2] as f64];
        let dist = self.camera.distance();
        self.camera.eye = [
            self.camera.target[0] - dir[0] * dist,
            self.camera.target[1] - dir[1] * dist,
            self.camera.target[2] - dir[2] * dist,
        ];
        self.camera.up = [up[0] as f64, up[1] as f64, up[2] as f64];
    }

    /// Apply a ViewCube navigation-arrow rotation to the CURRENT camera (a
    /// relative 90° orbit / roll), keeping the pivot (`target`) fixed. The
    /// rotation axes are the camera's own screen axes so the motion follows the
    /// on-screen arrow direction: the eye moves toward the pan arrow it points
    /// at, and the roll arcs spin the up vector about the view direction.
    fn apply_viewcube_arrow(&mut self, handle: u32) {
        use crate::view::{add3, rotate3, sub3};
        use brep_gizmos::view_cube::ViewCube;
        // World-space screen axes of the current view: right, up, forward(eye→target).
        let (right, up_axis, fwd) = self.camera.basis();
        let target = self.camera.target;
        let rel = sub3(self.camera.eye, target); // eye relative to pivot
        let q = std::f64::consts::FRAC_PI_2; // 90° per click
        match handle {
            // Orbit about the screen-up axis; eye moves toward the arrow side.
            ViewCube::ARROW_RIGHT => {
                self.camera.eye = add3(target, rotate3(rel, up_axis, q));
            }
            ViewCube::ARROW_LEFT => {
                self.camera.eye = add3(target, rotate3(rel, up_axis, -q));
            }
            // Orbit about the screen-right axis; carry the up vector along so the
            // view stays upright (eye moves toward the arrow side).
            ViewCube::ARROW_UP => {
                self.camera.eye = add3(target, rotate3(rel, right, -q));
                self.camera.up = rotate3(self.camera.up, right, -q);
            }
            ViewCube::ARROW_DOWN => {
                self.camera.eye = add3(target, rotate3(rel, right, q));
                self.camera.up = rotate3(self.camera.up, right, q);
            }
            // Roll about the view direction; only the up vector changes.
            ViewCube::ROLL_CCW => {
                self.camera.up = rotate3(self.camera.up, fwd, q);
            }
            ViewCube::ROLL_CW => {
                self.camera.up = rotate3(self.camera.up, fwd, -q);
            }
            _ => {}
        }
    }

    /// Pick the datum plane/axis under a screen pixel; returns its name (empty
    /// when none).
    ///
    /// NOT the selection path any more: construction planes are ordinary pick
    /// candidates ([`Self::pick_candidates_at`] → the widget's `datum_plane_hits`,
    /// which reports EVERY card the ray crosses rather than the first), so the
    /// viewport click router no longer calls this. It survives as the AXIS-aware
    /// second line of defense inside [`Self::ref_select_click`]'s total-miss arm.
    pub fn datum_pick(&self, x: f64, y: f64) -> String {
        self.widgets
            .datum_pick(&gizmo_camera(&self.camera), x as f32, y as f32)
            .unwrap_or_default()
    }

    /// Update the transform-gizmo hover from a screen pixel; returns the handle
    /// under the pointer (0 = none). Marks dirty when the highlight changed.
    pub fn transform_hover(&mut self, x: f64, y: f64) -> u32 {
        let cam = gizmo_camera(&self.camera);
        let handle = self.widgets.transform_hit(&cam, x as f32, y as f32);
        if self.widgets.set_transform_hover(handle) {
            self.dirty = true;
        }
        handle
    }

    /// The transform-gizmo handle under a screen pixel (0 = none) — the host
    /// echoes it back to start a drag.
    pub fn transform_pick(&self, x: f64, y: f64) -> u32 {
        self.widgets.transform_hit(&gizmo_camera(&self.camera), x as f32, y as f32)
    }

    /// Compute a transform drag (frame-space + world delta) as JSON for the
    /// feature-edit commit. Marks the handle active for the highlight.
    pub fn transform_drag(
        &mut self,
        handle: u32,
        sx: f64,
        sy: f64,
        cx: f64,
        cy: f64,
    ) -> String {
        let cam = gizmo_camera(&self.camera);
        self.widgets.set_transform_active(handle);
        self.dirty = true;
        self.widgets
            .transform_drag_json(&cam, handle, sx as f32, sy as f32, cx as f32, cy as f32)
    }

    pub fn transform_drag_end(&mut self) {
        self.widgets.set_transform_active(0);
        self.dirty = true;
    }

    /// Per-dimension label placement: `[{id, anchor:[x,y,z],
    /// screen:[sx,sy,inFront]}]` — the host pins each text label at `screen`.
    pub fn dimension_anchors_json(&self) -> String {
        let anchors = self.widgets.dimension_anchors(&gizmo_camera(&self.camera));
        let out: Vec<serde_json::Value> = anchors
            .into_iter()
            .map(|(id, p)| {
                let world = [p[0] as f64, p[1] as f64, p[2] as f64];
                let (sx, sy, _) = self.camera.project(world);
                // Same ONE label policy as `world_to_screen_json` — near/far
                // never cull; off-viewport anchors hide their label.
                let visible = if self.camera.label_anchor_visible(world) { 1.0 } else { 0.0 };
                serde_json::json!({
                    "id": id,
                    "anchor": p,
                    "screen": [sx, sy, visible],
                })
            })
            .collect();
        serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string())
    }

    // --- Undo / redo (engine-owned) ---------------------------------------
    //
    // The model is engine-owned, so its undo history lives in the engine core
    // too: `History` holds the stacks and snapshots itself BEFORE each model
    // mutation (edit / add / delete / reorder), while roll-to-step is view state
    // and is NOT snapshotted. The UI only TRIGGERS these; it never holds a stack.

}

/// Choose the camera `up` when the ViewCube snaps to a face / edge / corner.
///
/// `dir` is the world eye→target direction the view snaps to
/// ([`brep_gizmos::view_cube::ViewCube::target_view`]): axis-aligned for a face,
/// a 45° blend for an edge, a body-diagonal for a corner. `current_up` is the
/// live camera up; `fallback` the region's canonical up.
///
/// The roll is snapped to a DISCRETE set and the member closest to the current
/// up wins (max dot ⇒ least roll), so the reorient rotates by the smallest angle
/// while still landing "level":
/// - **face** (`kind == 1`): the 4 signed world axes lying in the face plane
///   (the two axes `dir` is perpendicular to) — the face ends flat-on with its
///   bottom edge horizontal, choosing whichever of the 4 edges was already
///   nearest the top.
/// - **corner** (`kind == 3`): the 3 cube axes that point up for this corner
///   (top-vertex-up isometric), 120° apart — the axis whose sign opposes each
///   component of `dir` (so the near vertex reads upright, not inverted).
/// - **edge / other**: no discrete set — the current up is projected onto the
///   plane ⟂ `dir` (free roll preserved), falling back to `fallback` when that
///   projection degenerates (up nearly parallel to `dir`).
pub(super) fn snap_view_up(
    kind: u8,
    dir: [f64; 3],
    current_up: [f64; 3],
    fallback: [f32; 3],
) -> [f32; 3] {
    let mut candidates: Vec<[f64; 3]> = Vec::new();
    match kind {
        // Face: both signs of each axis the (axis-aligned) view dir is ⟂ to.
        1 => {
            for a in 0..3 {
                if dir[a].abs() < 0.5 {
                    let mut p = [0.0; 3];
                    p[a] = 1.0;
                    candidates.push(p);
                    p[a] = -1.0;
                    candidates.push(p);
                }
            }
        }
        // Corner: the axis direction opposite each component of the view dir
        // (dir = -normalize(signs), so -sign(dir[a]) recovers the corner's sign).
        3 => {
            for a in 0..3 {
                let mut p = [0.0; 3];
                p[a] = -dir[a].signum();
                candidates.push(p);
            }
        }
        _ => {}
    }

    let dot = |v: &[f64; 3]| v[0] * current_up[0] + v[1] * current_up[1] + v[2] * current_up[2];
    // An EXACT tie — the current up perpendicular to two candidates, as from
    // an ISO onto its TOP face — goes to the candidate nearest the region's
    // own up, not to whichever the loop listed last. The listing order only
    // happened to give TOP's −Z in the Y-up frame; in the board frame it gave
    // the TOP view upside down (the layout's +Y down the screen).
    let score = |v: &[f64; 3]| {
        dot(v) + 1e-6 * (v[0] * fallback[0] as f64 + v[1] * fallback[1] as f64 + v[2] * fallback[2] as f64)
    };
    if let Some(best) = candidates
        .into_iter()
        .max_by(|x, y| score(x).partial_cmp(&score(y)).unwrap_or(std::cmp::Ordering::Equal))
    {
        return [best[0] as f32, best[1] as f32, best[2] as f32];
    }

    // Edge / fallback: project the current up onto the plane ⟂ dir (keep roll).
    let d = dot(&dir);
    let proj = [
        current_up[0] - dir[0] * d,
        current_up[1] - dir[1] * d,
        current_up[2] - dir[2] * d,
    ];
    let len = (proj[0] * proj[0] + proj[1] * proj[1] + proj[2] * proj[2]).sqrt();
    if len > 1e-4 {
        [
            (proj[0] / len) as f32,
            (proj[1] / len) as f32,
            (proj[2] / len) as f32,
        ]
    } else {
        fallback
    }
}

/// Serialize a screen-space [`brep_gizmos::hit_region::HitShape`] to the gizmo
/// hit-area JSON schema the app strokes: `{ kind:"capsule", a:[x,y], b:[x,y], r }`
/// or `{ kind:"circle", c:[x,y], r }` — viewport-local px, so the app only offsets
/// by `rect.min`. The engine hit-tests these SAME shapes, so the drawn outline can
/// never drift from the pickable region. Shared by both gizmo exposers.
pub(super) fn hit_shape_json(shape: &brep_gizmos::hit_region::HitShape) -> serde_json::Value {
    use brep_gizmos::hit_region::HitShape;
    match *shape {
        HitShape::Capsule { a, b, r } => {
            serde_json::json!({ "kind": "capsule", "a": a, "b": b, "r": r })
        }
        HitShape::Circle { c, r } => serde_json::json!({ "kind": "circle", "c": c, "r": r }),
    }
}

/// Serialize an iterator of [`brep_gizmos::hit_region::HitShape`]s to the app's
/// gizmo hit-area JSON array (see [`hit_shape_json`]); `"[]"` on failure.
pub(super) fn hit_shapes_json<'a>(
    shapes: impl Iterator<Item = &'a brep_gizmos::hit_region::HitShape>,
) -> String {
    let out: Vec<serde_json::Value> = shapes.map(hit_shape_json).collect();
    serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string())
}



