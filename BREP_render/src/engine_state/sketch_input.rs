use super::*;
use super::sketch_panel::{sketch_constraint_signature, sketch_perpendicular_should_swap};

// Sketch picking maps camera rays to plane-space coordinates. Hit tests prefer
// points over geometry within the grab radius. Mutators refresh the live overlay
// and mark the engine dirty; calls outside sketch mode are no-ops.
impl EngineState {
    /// Re-push the sketch overlay reflecting the live hover + selection. Reads the
    /// active `sketch_edit`'s session; a no-op when not in sketch mode. Called at
    /// the end of every S2 mutator (the reusable counterpart of the initial
    /// [`set_sketch_overlay`](Self::set_sketch_overlay) push).
    pub(super) fn refresh_sketch_overlay(&mut self) {
        let world_per_pixel = self.camera.world_per_pixel();
        let (json, preview, leaders, glyphs) = match self.sketch_edit.as_ref() {
            Some(edit) => (
                edit.session.overlay_json_with_state(world_per_pixel),
                edit.session.preview_overlay_json(
                    world_per_pixel,
                    &edit.pending,
                    edit.hover_uv,
                    &edit.handdraw_stroke,
                ),
                edit.session.dim_leaders_overlay_json_with_state(world_per_pixel),
                edit.session
                    .constraint_glyphs_overlay_json_with_state(world_per_pixel),
            ),
            None => return,
        };
        let _ = self.set_overlay_json(&json);
        // The draw-tool rubber-band rides in its own `sketch-preview` group so it
        // upserts/clears independently of the solved geometry + point groups.
        let _ = self.set_overlay_json(&preview);
        // The dimension leaders + arrows ride in `sketch-dim-leaders`, refreshed
        // alongside everything else (S5).
        let _ = self.set_overlay_json(&leaders);
        // The geometric-constraint glyphs ride in `sketch-constraint-glyphs` (S6c).
        let _ = self.set_overlay_json(&glyphs);
        // Remember the zoom these groups were baked at: their construction dashes,
        // dimension arrowheads and constraint glyphs are all screen-constant, so
        // `ensure_sketch_overlay_current` re-bakes them when it moves.
        self.sketch_overlay_wpp = if world_per_pixel > 0.0 {
            world_per_pixel
        } else {
            f64::MIN_POSITIVE
        };
    }

    /// Per-frame upkeep for the live SKETCH overlay (driven by
    /// [`Self::ensure_overlays_current`]) — the sketch-mode sibling of
    /// [`Self::ensure_feature_dimension_overlay_current`]. The dimension leaders
    /// (draggable), constraint glyphs and construction dashes are sized in PIXELS
    /// against the camera at bake time, so a zoom leaves them stale until
    /// something else mutates the sketch. Re-bakes on a material
    /// `world_per_pixel` change only, so a quiet frame stays quiet.
    pub(super) fn ensure_sketch_overlay_current(&mut self) {
        if !self.sketch_mode() {
            self.sketch_overlay_wpp = 0.0;
            return;
        }
        let wpp = self.camera.world_per_pixel();
        if super::overlay_wpp_stale(self.sketch_overlay_wpp, wpp) {
            self.refresh_sketch_overlay();
        }
    }

    /// Map CSS-pixel `(x, y)` to the active sketch plane's `(u, v)` via the camera
    /// pick ray ∩ the sketch plane. `None` when not in sketch mode or the ray misses
    /// the plane (parallel / behind).
    pub fn sketch_uv_at(&self, x: f64, y: f64) -> Option<(f64, f64)> {
        let edit = self.sketch_edit.as_ref()?;
        let ray = self.camera.pick_ray(x, y);
        crate::sketch::ray_plane_uv(&edit.session.plane, ray.origin, ray.dir)
    }

    /// The world-space pick tolerance at the current zoom — the ONE radius that
    /// drives hover-highlight, click-select, drag-grab, draw-snap AND trim, for
    /// points AND geometry, so "what highlights" is exactly "what you can grab".
    /// Sized at 1.5× the visualized point (`POINT_SIZE_PX`) for forgiving clicking.
    pub(super) fn sketch_pick_radius(&self) -> f64 {
        f64::from(crate::sketch::tessellate::POINT_SIZE_PX) * 1.5 * self.camera.world_per_pixel()
    }

    /// The entity ref under CSS-pixel `(x, y)` within the grab radius, or `None`.
    /// Priority is points > geometry > constraint: [`pick_entity`] resolves the first
    /// two, and only when neither is in range do we consult [`pick_constraint`] (a glyph
    /// or dimension leader that overlaps a point/edge never shadows it).
    ///
    /// [`pick_entity`]: crate::sketch::SketchSession::pick_entity
    /// [`pick_constraint`]: crate::sketch::SketchSession::pick_constraint
    fn sketch_entity_at(&self, x: f64, y: f64) -> Option<serde_json::Value> {
        let (u, v) = self.sketch_uv_at(x, y)?;
        let radius = self.sketch_pick_radius();
        let wpp = self.camera.world_per_pixel();
        self.sketch_edit.as_ref().and_then(|edit| {
            edit.session
                .pick_entity(u, v, radius)
                .or_else(|| edit.session.pick_constraint(u, v, radius, wpp))
        })
    }

    /// Set (or clear) the sketch hover, re-pushing the overlay + marking dirty only
    /// when it actually changed. Returns whether the hover changed.
    pub(super) fn set_sketch_hover(&mut self, new_hover: Option<serde_json::Value>) -> bool {
        let changed = match self.sketch_edit.as_ref() {
            Some(edit) => {
                !crate::sketch::entity_ref_eq(edit.session.hovered.as_ref(), new_hover.as_ref())
            }
            None => false,
        };
        if changed {
            if let Some(edit) = self.sketch_edit.as_mut() {
                edit.session.set_hover(new_hover);
            }
            self.refresh_sketch_overlay();
            self.dirty = true;
        }
        changed
    }

    /// Update the sketch hover to the entity under CSS-pixel `(x, y)` (S2). Returns
    /// whether the hover changed. A no-op returning `false` when not in sketch mode.
    pub fn sketch_hover_at(&mut self, x: f64, y: f64) -> bool {
        if self.sketch_edit.is_none() {
            return false;
        }
        // Track the live cursor uv for the S3a rubber-band. In DRAW mode with pending
        // clicks the preview follows the cursor even when the hovered ENTITY is
        // unchanged, so force an overlay refresh there.
        let uv = self.sketch_uv_at(x, y);
        let preview_live = match self.sketch_edit.as_mut() {
            Some(edit) => {
                edit.hover_uv = uv;
                edit.session.tool.is_some() && !edit.pending.is_empty()
            }
            None => false,
        };
        let new_hover = self.sketch_entity_at(x, y);
        let changed = self.set_sketch_hover(new_hover);
        if preview_live && !changed {
            self.refresh_sketch_overlay();
            self.dirty = true;
        }
        changed
    }

    /// Clear the sketch hover (pointer left the viewport / moved over the ViewCube).
    /// Returns whether a hover was cleared.
    pub fn sketch_clear_hover(&mut self) -> bool {
        self.set_sketch_hover(None)
    }

    /// Click-select in sketch mode: pick the entity under `(x, y)`; nothing → clear
    /// the selection. Otherwise honor the SAME "Multi-select" setting the 3D viewport
    /// reads (`settings.multi_select`): under `ClickToggles` a plain click toggles the
    /// hit in the set (no modifier needed for a multi-selection); under
    /// `CtrlClick` a plain click replaces the set with just the hit and `additive`
    /// (Ctrl/Cmd) toggles. Re-pushes the overlay + marks dirty.
    pub fn sketch_click_at(&mut self, x: f64, y: f64, additive: bool) {
        // Read the setting before the mutable `sketch_edit` borrow.
        let toggles = self.settings.multi_select == crate::style::MultiSelectMode::ClickToggles;
        let hit = self.sketch_entity_at(x, y);
        let Some(edit) = self.sketch_edit.as_mut() else {
            return;
        };
        match hit {
            None => edit.session.clear_selection(),
            Some(entity_ref) => {
                if additive || toggles {
                    edit.session.toggle_selection(entity_ref);
                } else {
                    edit.session.clear_selection();
                    edit.session.toggle_selection(entity_ref);
                }
            }
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// Begin a point drag if a DRAGGABLE point is under `(x, y)` (S2): remember it
    /// (id + original `fixed` flag). Returns `true` iff a point was grabbed (the
    /// viewport routes the drag to the sketch; otherwise it orbits the camera). A
    /// locked / fully-constrained point is not draggable, so an empty-space or
    /// locked-point drag falls through to a camera orbit.
    pub fn sketch_drag_begin(&mut self, x: f64, y: f64) -> bool {
        let Some((u, v)) = self.sketch_uv_at(x, y) else {
            return false;
        };
        let radius = self.sketch_pick_radius();
        // Grab EXACTLY what's HIGHLIGHTED: the hovered entity was picked at the exact
        // cursor position on the last move, so it is immune to egui reporting the
        // drag-start ~6px into the gesture (the "highlighted but won't grab"
        // intermittency — and it's what lets a whole geometry drag). A hovered LOCKED
        // point yields `None` → the drag falls through to a camera gesture; it must
        // NOT positional-fall-back there (that would grab a nearby UNhighlighted
        // point). Only an EMPTY hover (a press with no prior move) falls back to a
        // fresh positional pick.
        let points = self.sketch_edit.as_ref().and_then(|edit| {
            match edit.session.hovered.as_ref() {
                Some(entity_ref) => edit.session.drag_points_from_ref(entity_ref),
                None => edit
                    .session
                    .pick_draggable_point(u, v, radius)
                    .and_then(|(id, fixed)| {
                        edit.session
                            .doc
                            .point(&id)
                            .map(|p| vec![(id, p.x, p.y, fixed)])
                    }),
            }
        });
        let Some(points) = points else {
            return false;
        };
        if let Some(edit) = self.sketch_edit.as_mut() {
            // Snapshot ONCE at the gesture start so the whole drag is one undo step
            // (S6a); `sketch_drag_to` never snapshots. A grab that moves nothing is
            // discarded in `sketch_drag_end`.
            edit.record_undo();
            edit.drag = Some(SketchDrag { points, anchor: (u, v) });
        }
        true
    }

    /// Drag the grabbed target to `(x, y)` (S2): pin every grabbed point at its
    /// ORIGINAL position plus the cursor delta (`fixed = true`) so the solver anchors
    /// the whole shape there, re-solve, then restore each point's ORIGINAL `fixed`
    /// flag. Absolute-from-anchor (never incremental), so a rigid geometry translate
    /// tracks the cursor 1:1 without drifting as the solver nudges points between
    /// frames. A resolve error rolls every grabbed point back to its pre-drag coords
    /// (the last good state). No-op when nothing is grabbed / not in sketch mode / the
    /// ray misses the plane.
    pub fn sketch_drag_to(&mut self, x: f64, y: f64) {
        let Some((u, v)) = self.sketch_uv_at(x, y) else {
            return;
        };
        let Some(edit) = self.sketch_edit.as_mut() else {
            return;
        };
        let Some(drag) = edit.drag.clone() else {
            return;
        };
        let (du, dv) = (u - drag.anchor.0, v - drag.anchor.1);
        let session = &mut edit.session;
        for (id, ox, oy, _) in &drag.points {
            if let Some(p) = session.doc.point_mut(id) {
                p.x = *ox + du;
                p.y = *oy + dv;
                p.fixed = true;
            }
        }
        match session.resolve() {
            Ok(()) => {
                for (id, _, _, orig_fixed) in &drag.points {
                    if let Some(p) = session.doc.point_mut(id) {
                        p.fixed = *orig_fixed;
                    }
                }
            }
            Err(_) => {
                // Unsolvable target: roll every grabbed point back to its pre-drag
                // coords + flag (keep the last good state).
                for (id, ox, oy, orig_fixed) in &drag.points {
                    if let Some(p) = session.doc.point_mut(id) {
                        p.x = *ox;
                        p.y = *oy;
                        p.fixed = *orig_fixed;
                    }
                }
            }
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// End a point drag (S2): clear the grab, then one final re-solve + overlay
    /// refresh. No-op when no drag is live.
    pub fn sketch_drag_end(&mut self) {
        let had_grab = self
            .sketch_edit
            .as_ref()
            .map_or(false, |edit| edit.drag.is_some());
        if !had_grab {
            return;
        }
        // Drop radius for constraint inference == the point grab radius (12 px in
        // world units) — read before the `&mut` borrow of `sketch_edit`.
        let drop_tol = self.sketch_pick_radius();
        if let Some(edit) = self.sketch_edit.as_mut() {
            // Drop-time inference (S6c): a SINGLE-point drop snaps to a coincident
            // point / point-on-line at the release position, mirroring the previous
            // coincident-on-drop / point-on-line-on-drop inference. A whole-
            // geometry drag (multiple points) never infers — running it per
            // endpoint could glue or collapse the curve in one solve.
            let dragged_single = edit.drag.as_ref().and_then(|drag| {
                (drag.points.len() == 1).then(|| drag.points[0].0.clone())
            });
            edit.drag = None;
            if let Some(point_id) = dragged_single {
                crate::sketch::infer::infer_drop_constraint(
                    &mut edit.session.doc,
                    &point_id,
                    drop_tol,
                );
            }
            let _ = edit.session.resolve();
            // Discard the drag's undo snapshot when the doc is unchanged (a mere
            // grab-and-release, no move and no inferred constraint), so it neither
            // pollutes undo nor clobbers redo. An inferred constraint changes the
            // doc, so the snapshot is kept — one Ctrl+Z then undoes move+constraint.
            if edit
                .undo_stack
                .last()
                .map_or(false, |snap| snap.doc == edit.session.doc)
            {
                edit.undo_stack.pop();
            }
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// The number of selected sketch entities (0 when not in sketch mode) — the mode
    /// bar / verifier readout.
    pub fn sketch_selection_count(&self) -> usize {
        self.sketch_edit
            .as_ref()
            .map_or(0, |edit| edit.session.selection.len())
    }

    /// The number of selected CONSTRAINTS (refs whose `kind` is `"constraint"`; 0 when
    /// not in sketch mode) — the `__brepSketch` verifier readout for constraint
    /// selection + delete.
    pub fn sketch_selected_constraint_count(&self) -> usize {
        self.sketch_edit.as_ref().map_or(0, |edit| {
            edit.session
                .selection
                .iter()
                .filter(|r| r.get("kind").and_then(|v| v.as_str()) == Some("constraint"))
                .count()
        })
    }
}

// ===========================================================================
// Sketch draw tools (S3a) — primitive placement: point / line / rect / circle / arc.
//
// A click-state machine over the active `self.sketch_edit`. The active tool lives
// on `session.tool` ("select"/None = selection mode, S2); a DRAW tool routes clicks
// to `sketch_tool_click_at` (pixel → plane uv via the same S2 `sketch_uv_at`, then
// `sketch_tool_place_uv`). Points/geometries are minted through `SketchDoc`
// (`next_point_id`/`next_geometry_id` + `snap_or_add_point`, so shared vertices
// coincide). Each placement re-solves (swallowing solve errors), re-pushes the
// overlay (incl. the rubber-band preview), and marks dirty. The bezier tool has a
// second mode on the same click: idle, a click ON an existing spline REFINES it
// rather than starting a new one (`crate::sketch::spline`). Kept in ONE appended
// block so concurrent edits to the primary impl land clean.
// ===========================================================================
impl EngineState {
    /// Set (or clear) the active draw tool: `"select"`/`None` → selection mode (S2);
    /// `"point"|"line"|"rect"|"circle"|"arc"|"bezier"` arm the corresponding draw
    /// tool; `"handdraw"` arms the freehand stroke tool (S6b-3); `"trim"` arms the
    /// trim tool (S6b); `"pickEdges"` arms the external-edge link tool (S6b-2). Clears
    /// any in-progress click buffer + preview and refreshes the overlay. No-op when not
    /// in sketch mode.
    pub fn sketch_set_tool(&mut self, tool: Option<&str>) {
        let normalized = normalize_sketch_tool(tool);
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.session.tool = normalized;
            edit.pending.clear();
            edit.hover_uv = None;
            edit.handdraw_stroke.clear();
        } else {
            return;
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// The active draw tool (`"point"|"line"|"rect"|"circle"|"arc"`), or `None` in
    /// selection mode / when not in sketch mode.
    pub fn sketch_active_tool(&self) -> Option<&str> {
        self.sketch_edit
            .as_ref()
            .and_then(|edit| edit.session.tool.as_deref())
    }

    /// The number of in-progress draw-tool clicks buffered (0 in selection mode /
    /// when not in sketch mode) — the UI/preview + verifier readout.
    pub fn sketch_pending_len(&self) -> usize {
        self.sketch_edit.as_ref().map_or(0, |edit| edit.pending.len())
    }

    /// A draw-tool click at CSS-pixel `(x, y)`: map to plane uv (the S2 pixel→plane
    /// math) and drive the tool state machine. No-op when not in sketch mode, in
    /// selection mode, or the ray misses the plane.
    pub fn sketch_tool_click_at(&mut self, x: f64, y: f64) {
        // pickEdges (S6b-2) acts on the 3D SCENE EDGE under the cursor — it needs the
        // PIXEL coords (a scene pick), not a plane uv, so short-circuit before the
        // pixel→plane projection (which would drop clicks that miss the plane).
        if self.sketch_active_tool() == Some("pickEdges") {
            self.sketch_pick_edge_at(x, y);
            return;
        }
        let Some((u, v)) = self.sketch_uv_at(x, y) else {
            return;
        };
        self.sketch_tool_place_uv(u, v);
    }

    /// The per-tool placement logic, in plane `(u, v)` (the headless-testable core
    /// `sketch_tool_click_at` delegates to). Snaps to existing points within the grab
    /// radius so shared vertices coincide; appends geometry and re-solves when a
    /// primitive completes; carries the line chain via `pending`.
    pub fn sketch_tool_place_uv(&mut self, u: f64, v: f64) {
        let radius = self.sketch_pick_radius();
        // Selection mode (no tool / "select") never places. Read the tool without
        // holding a borrow so the trim branch can call back into `self`.
        let tool = match self.sketch_edit.as_ref() {
            Some(edit) => match edit.session.tool.clone() {
                Some(tool) => tool,
                None => return,
            },
            None => return,
        };
        // Trim (S6b) is a click tool that acts IMMEDIATELY on the geometry under the
        // cursor — it never buffers `pending` or places a point. It owns its own undo
        // snapshot (and pops it on a no-op), so short-circuit before the draw path.
        if tool == "trim" {
            self.sketch_trim_uv(u, v);
            return;
        }
        // pickEdges is NOT a uv-placement tool — it acts on a 3D scene edge (routed via
        // pixel coords in `sketch_tool_click_at`), so a stray uv place is a no-op here.
        if tool == "pickEdges" {
            return;
        }
        // handdraw (S6b-3) captures a DRAG as a stroke (routed via `sketch_handdraw_*`),
        // not a click-placed point — a plain click is a no-op (and never records a dead
        // undo step here, since we return before `record_undo`).
        if tool == "handdraw" {
            return;
        }
        let Some(edit) = self.sketch_edit.as_mut() else {
            return;
        };
        // A draw tool is active → this click WILL mutate the doc (a point and/or a
        // geometry); snapshot for undo before it does (S6a).
        edit.record_undo();
        let doc = &mut edit.session.doc;
        match tool.as_str() {
            "point" => {
                doc.snap_or_add_point(u, v, radius);
                edit.pending.clear();
            }
            "line" => {
                if edit.pending.is_empty() {
                    let a = doc.snap_or_add_point(u, v, radius);
                    edit.pending.push(a);
                } else {
                    let start = edit.pending.last().cloned().expect("pending non-empty");
                    let end = doc.snap_or_add_point(u, v, radius);
                    push_sketch_geometry(doc, "line", vec![start, end.clone()]);
                    // Continue the chain: the just-placed end is the next start.
                    edit.pending = vec![end];
                }
            }
            "rect" => {
                if edit.pending.is_empty() {
                    let a = doc.snap_or_add_point(u, v, radius);
                    edit.pending.push(a);
                } else {
                    let a_id = edit.pending[0].clone();
                    let Some((ax, ay)) = doc.point(&a_id).map(|p| (p.x, p.y)) else {
                        edit.pending.clear();
                        return;
                    };
                    let (bx, by) = (u, v);
                    // Corners A=(ax,ay), (bx,ay), (bx,by), (ax,by) → 4 closed lines.
                    let b1 = doc.snap_or_add_point(bx, ay, radius);
                    let b2 = doc.snap_or_add_point(bx, by, radius);
                    let b3 = doc.snap_or_add_point(ax, by, radius);
                    push_sketch_geometry(doc, "line", vec![a_id.clone(), b1.clone()]);
                    push_sketch_geometry(doc, "line", vec![b1.clone(), b2.clone()]);
                    push_sketch_geometry(doc, "line", vec![b2.clone(), b3.clone()]);
                    push_sketch_geometry(doc, "line", vec![b3.clone(), a_id.clone()]);
                    // Keep the rectangle rectangular under drag: three ⟂ constraints on
                    // the adjacent-edge pairs (the 4th corner's right angle follows from
                    // the closed loop). This is the minimal rigid set — it removes 3 DOF
                    // from the 8-DOF four-corner quad, leaving position (2) + rotation (1)
                    // + width + height = 5 DOF, so the sketch is neither over-constrained
                    // nor conflicting.
                    push_rect_perpendicular_constraints(doc, [a_id, b1, b2, b3]);
                    edit.pending.clear();
                }
            }
            "circle" => {
                if edit.pending.is_empty() {
                    let c = doc.snap_or_add_point(u, v, radius);
                    edit.pending.push(c);
                } else {
                    let center = edit.pending[0].clone();
                    let r = doc.snap_or_add_point(u, v, radius);
                    push_sketch_geometry(doc, "circle", vec![center, r]);
                    edit.pending.clear();
                }
            }
            "arc" => {
                // Clicks: center, start, then end completes [center, start, end].
                if edit.pending.len() < 2 {
                    let p = doc.snap_or_add_point(u, v, radius);
                    edit.pending.push(p);
                } else {
                    let center = edit.pending[0].clone();
                    let start = edit.pending[1].clone();
                    let end = doc.snap_or_add_point(u, v, radius);
                    push_sketch_geometry(doc, "arc", vec![center, start, end]);
                    edit.pending.clear();
                }
            }
            "bezier" => {
                // Cubic Bezier: 4 clicks place end0, ctrl0, ctrl1, end1 (in order).
                // The 4th click commits the span [p0, p1, p2, p3] PLUS two dashed
                // construction guide lines for the control handles (end0→ctrl0 and
                // end1→ctrl1), matching the previous basic bezier tool. One INVOCATION
                // authors one span; a chained multi-span polygon (3n+1 ids in ONE
                // geometry — the model the solver, tessellator and profile builder all
                // already read) is grown by insertion instead, below.
                //
                // IDLE + a click on an existing spline = refine it: subdivide the
                // clicked span so a new anchor lands under the cursor without the curve
                // moving (see `crate::sketch::spline`). Only while `pending` is empty —
                // mid-draw the click still means "place the next control point", so a
                // new spline can be drawn across an old one. A refusal (nothing under
                // the cursor, a point winning the pick, a degenerate span) falls
                // through to that placement, which mutates the doc too, so the
                // `record_undo` above never leaves a dead step either way.
                let refined = edit.pending.is_empty() && spline_insert_anchor(doc, u, v, radius);
                if !refined {
                    if edit.pending.len() < 3 {
                        let p = doc.snap_or_add_point(u, v, radius);
                        edit.pending.push(p);
                    } else {
                        let p0 = edit.pending[0].clone();
                        let p1 = edit.pending[1].clone();
                        let p2 = edit.pending[2].clone();
                        let p3 = doc.snap_or_add_point(u, v, radius);
                        push_sketch_geometry(
                            doc,
                            "bezier",
                            vec![p0.clone(), p1.clone(), p2.clone(), p3.clone()],
                        );
                        // Construction guide lines (dashed, non-modeling) for the two
                        // control handles — separate freshly minted geometry ids.
                        push_sketch_construction_line(doc, vec![p0, p1]);
                        push_sketch_construction_line(doc, vec![p3, p2]);
                        edit.pending.clear();
                    }
                }
            }
            _ => return,
        }
        // Every draw click mutates the doc (a new point and/or geometry); re-solve so
        // coordinates + mobility stay fresh, keeping the doc if the solve fails.
        self.resolve_active_sketch("draw-tool");
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// Abort the in-progress draw geometry (Escape / right-click): clear the pending
    /// clicks + preview and refresh. No-op when not in sketch mode.
    pub fn sketch_tool_cancel(&mut self) {
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.pending.clear();
        } else {
            return;
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }
}

/// Normalize a tool name to the stored form: `None`/`"select"`/`""` → selection mode
/// (`None`), else the tool string (`"point"|"line"|"rect"|"circle"|"arc"|"bezier"|
/// "trim"|"pickEdges"|"handdraw"`).
fn normalize_sketch_tool(tool: Option<&str>) -> Option<String> {
    match tool {
        None | Some("select") | Some("") => None,
        Some(t) => Some(t.to_string()),
    }
}

/// Append a geometry to a sketch doc with a freshly minted id (the caller passes the
/// solver `type` — `rect` corners are pushed as `line`s), carrying an explicit
/// `construction: false` so it matches the authored shape and round-trips.
fn push_sketch_geometry(
    doc: &mut crate::sketch::SketchDoc,
    geom_type: &str,
    points: Vec<serde_json::Value>,
) {
    let id = doc.next_geometry_id();
    let mut extra = serde_json::Map::new();
    extra.insert("construction".to_string(), serde_json::Value::Bool(false));
    doc.geometries.push(crate::sketch::SketchGeometry {
        id,
        geom_type: geom_type.to_string(),
        points,
        extra,
    });
}

/// Append a CONSTRUCTION `line` geometry (dashed, non-modeling — `construction: true`)
/// with a freshly minted id: the bezier tool's control-handle guide lines. Mirrors
/// [`push_sketch_geometry`] but flips the construction flag so the line renders dashed
/// and is excluded from profiles while still being constrainable.
fn push_sketch_construction_line(
    doc: &mut crate::sketch::SketchDoc,
    points: Vec<serde_json::Value>,
) {
    let id = doc.next_geometry_id();
    let mut extra = serde_json::Map::new();
    extra.insert("construction".to_string(), serde_json::Value::Bool(true));
    doc.geometries.push(crate::sketch::SketchGeometry {
        id,
        geom_type: "line".to_string(),
        points,
        extra,
    });
}

/// Subdivide the spline under the plane click `(u, v)` — the bezier tool's "refine
/// what I already drew" click. Delegates the pick + de Casteljau split to
/// [`crate::sketch::spline::insert_anchor`] (which decides whether the click is a
/// subdivision at all) and, when it lands, hangs the SAME two dashed construction
/// guides on the new anchor that the 4-click path hangs on the two drawn ends:
/// anchor→handle on each side. Consistency is the whole point — after a refine every
/// anchor of the spline, drawn or inserted, has a guide to each of its neighbouring
/// handles, so the handles stay visible and constrainable (a tangent on a guide is
/// how a spline is made to meet a line smoothly) and a refined curve is
/// indistinguishable from one drawn that way. The two ORIGINAL guides still read
/// correctly without being touched, because the split reuses the end handles' ids and
/// they remain the handles adjacent to those same two end anchors.
///
/// Returns whether the click was consumed as an insertion; `false` means the caller
/// should treat it as an ordinary control-point placement.
fn spline_insert_anchor(doc: &mut crate::sketch::SketchDoc, u: f64, v: f64, radius: f64) -> bool {
    let Some(added) = crate::sketch::spline::insert_anchor(doc, u, v, radius) else {
        return false;
    };
    push_sketch_construction_line(doc, vec![added.anchor.clone(), added.before]);
    push_sketch_construction_line(doc, vec![added.anchor, added.after]);
    true
}

/// Append the three perpendicular (`⟂`) constraints that keep a freshly drawn
/// rectangle rectangular when a corner is dragged. `corners` are the rect's four
/// points in loop order `[a, b1, b2, b3]` (edges a→b1, b1→b2, b2→b3, b3→a); the
/// constraints go on the adjacent-edge pairs sharing corners b1 / b2 / b3. The fourth
/// corner (a) is left implied — a closed quad with three right angles is a rectangle —
/// so this is the MINIMAL rigid set (3 equations, no over-constraint / redundancy).
///
/// Each `⟂` stores the two edges' endpoint pairs `[l1a, l1b, l2a, l2b]`, swap-oriented
/// exactly like a palette-added perpendicular ([`sketch_build_and_add_constraint`]), and
/// is deduped on its signature. No-op unless the four corners are all distinct (a
/// degenerate rect whose corners snapped together would otherwise carry a `⟂` on a
/// zero-length edge, which is meaningless and can wedge the solver).
fn push_rect_perpendicular_constraints(
    doc: &mut crate::sketch::SketchDoc,
    corners: [serde_json::Value; 4],
) {
    use crate::sketch::doc::id_key;
    let [a, b1, b2, b3] = corners;
    let keys = [id_key(&a), id_key(&b1), id_key(&b2), id_key(&b3)];
    for i in 0..keys.len() {
        for j in (i + 1)..keys.len() {
            if keys[i] == keys[j] {
                return; // two corners collapsed → skip (no zero-length-edge ⟂).
            }
        }
    }
    // Adjacent edge pairs sharing corner b1 / b2 / b3.
    let pairs = [
        [a.clone(), b1.clone(), b1.clone(), b2.clone()],
        [b1.clone(), b2.clone(), b2.clone(), b3.clone()],
        [b2.clone(), b3.clone(), b3.clone(), a.clone()],
    ];
    for pair in pairs {
        let mut pts = pair.to_vec();
        if sketch_perpendicular_should_swap(doc, &pts) {
            pts.swap(0, 1);
        }
        push_geometric_constraint(doc, "⟂", pts);
    }
}

/// Append a NON-dimensional geometric constraint (`type` + ordered `points`) with a
/// freshly minted id and the same base fields a palette-added constraint carries
/// (`labelX`/`labelY` = 0, `displayStyle` = "", `value` = null, `valueNeedsSetup` =
/// true — see [`sketch_build_and_add_constraint`]). Deduped on `type + sorted-points`
/// (the solver runs with `remove_implied_duplicates: false`, so this is the only
/// dedup); a no-op on a duplicate.
fn push_geometric_constraint(
    doc: &mut crate::sketch::SketchDoc,
    ctype: &str,
    points: Vec<serde_json::Value>,
) {
    let sig = sketch_constraint_signature(ctype, &points);
    let duplicate = doc.constraints.iter().any(|c| match c.ctype() {
        Some(t) => sketch_constraint_signature(t, c.points()) == sig,
        None => false,
    });
    if duplicate {
        return;
    }
    let id = doc.next_constraint_id();
    let mut raw = serde_json::Map::new();
    raw.insert("id".to_string(), id);
    raw.insert("type".to_string(), serde_json::Value::String(ctype.to_string()));
    raw.insert("points".to_string(), serde_json::Value::Array(points));
    raw.insert("labelX".to_string(), serde_json::Value::from(0));
    raw.insert("labelY".to_string(), serde_json::Value::from(0));
    raw.insert(
        "displayStyle".to_string(),
        serde_json::Value::String(String::new()),
    );
    raw.insert("value".to_string(), serde_json::Value::Null);
    raw.insert("valueNeedsSetup".to_string(), serde_json::Value::Bool(true));
    doc.constraints.push(crate::sketch::SketchConstraint { raw });
}

// ===========================================================================
// Auto-constrain — infer the constraints implied by the rough-in geometry.
//
// A one-click "constrain what I drew": snap nearly-axis-aligned lines to `━`/`│` and
// near-coincident points to `≡`, going through the same `push_geometric_constraint`
// dedup the palette uses so the solver picks them up and a re-run is idempotent.
// ===========================================================================

/// Auto-constrain tolerances (v1 constants; a future pass could surface them as
/// settings). A line whose direction is within [`AUTO_HV_ANGLE_TOL_DEG`] of an axis
/// gets a horizontal/vertical constraint — loose enough to catch a rough-in, far
/// below the slope a user clearly intended.
const AUTO_HV_ANGLE_TOL_DEG: f64 = 3.0;
/// Two points auto-snap to COINCIDENT when within this fraction of the sketch's
/// bounding-box diagonal (scale-invariant, so it works at any sketch size)…
const AUTO_COINCIDENT_FRAC: f64 = 0.01;
/// …floored at this absolute distance for a tiny or single-cluster sketch.
const AUTO_COINCIDENT_ABS: f64 = 1e-4;

/// Infer + add the constraints implied by the current geometry: `━`/`│` on
/// nearly-axis-aligned lines and `≡` between near-coincident points. Conservative —
/// never adds an unsatisfiable constraint on two already-fixed points, never collapses
/// a curve by coinciding its own endpoints, never re-adds a coincident already implied
/// (directly or transitively), and dedups H/V via [`push_geometric_constraint`]. So a
/// second pass adds nothing (idempotent). Returns the number of constraints added; the
/// caller re-solves.
pub(super) fn auto_constrain_doc(doc: &mut crate::sketch::SketchDoc) -> usize {
    use crate::sketch::doc::id_key;
    use serde_json::Value;
    use std::collections::{HashMap, HashSet};

    let before = doc.constraints.len();

    // ---- Horizontal / vertical on nearly-axis-aligned lines. ----
    // Threshold on |unit component off the axis| = |sin(angle deviation)|.
    let hv_sin_tol = AUTO_HV_ANGLE_TOL_DEG.to_radians().sin();
    let mut hv: Vec<(&str, Vec<Value>)> = Vec::new();
    for g in &doc.geometries {
        if g.geom_type != "line" || g.points.len() < 2 {
            continue;
        }
        let (Some(a), Some(b)) = (doc.point(&g.points[0]), doc.point(&g.points[1])) else {
            continue;
        };
        // Skip a line pinned at BOTH ends (no freedom → an H/V that isn't already
        // exactly true is unsatisfiable). This also excludes linked reference lines,
        // whose endpoints are all fixed.
        if a.fixed && b.fixed {
            continue;
        }
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let len = dx.hypot(dy);
        if len < 1e-9 {
            continue;
        }
        // Skip a line that already carries an H or V constraint on these endpoints
        // (never add the opposite one; keeps the pass idempotent).
        let mut keys: Vec<String> = g.points[..2].iter().map(id_key).collect();
        keys.sort();
        let already_hv = doc.constraints.iter().any(|c| {
            if !matches!(c.ctype(), Some("━") | Some("│")) {
                return false;
            }
            let mut k: Vec<String> = c.points().iter().map(id_key).collect();
            k.sort();
            k == keys
        });
        if already_hv {
            continue;
        }
        let (ux, uy) = (dx / len, dy / len);
        if uy.abs() <= hv_sin_tol {
            hv.push(("━", vec![g.points[0].clone(), g.points[1].clone()]));
        } else if ux.abs() <= hv_sin_tol {
            hv.push(("│", vec![g.points[0].clone(), g.points[1].clone()]));
        }
    }
    for (ct, pts) in hv {
        push_geometric_constraint(doc, ct, pts);
    }

    // ---- Coincident between near-coincident, mergeable point pairs. ----
    let n = doc.points.len();
    if n >= 2 {
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]]; // path halving
                x = parent[x];
            }
            x
        }
        fn union(parent: &mut [usize], a: usize, b: usize) {
            let (ra, rb) = (find(parent, a), find(parent, b));
            if ra != rb {
                parent[ra] = rb;
            }
        }

        // Union-find over point indices, SEEDED with the existing coincidents so we
        // never re-add one (directly or transitively).
        let index: HashMap<String, usize> = doc
            .points
            .iter()
            .enumerate()
            .map(|(i, p)| (id_key(&p.id), i))
            .collect();
        let mut parent: Vec<usize> = (0..n).collect();
        for c in &doc.constraints {
            if c.ctype() == Some("≡") {
                let pts = c.points();
                if let (Some(p0), Some(p1)) = (pts.first(), pts.get(1)) {
                    if let (Some(&i), Some(&j)) = (index.get(&id_key(p0)), index.get(&id_key(p1))) {
                        union(&mut parent, i, j);
                    }
                }
            }
        }

        // Distance tolerance relative to the sketch extent — the user's points only:
        // the part-origin reference may sit far from the drawing and must not widen it.
        let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for p in doc.points.iter().filter(|p| !brep_kernel::is_sketch_origin_point_id(&p.id)) {
            lo[0] = lo[0].min(p.x);
            lo[1] = lo[1].min(p.y);
            hi[0] = hi[0].max(p.x);
            hi[1] = hi[1].max(p.y);
        }
        let extent = ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt();
        let tol = (extent * AUTO_COINCIDENT_FRAC).max(AUTO_COINCIDENT_ABS);

        // Point-key set per geometry — skip a pair that are the two ends of the SAME
        // curve (coinciding them would collapse it).
        let geo_sets: Vec<HashSet<String>> = doc
            .geometries
            .iter()
            .map(|g| g.points.iter().map(id_key).collect())
            .collect();

        let mut coincidents: Vec<Vec<Value>> = Vec::new();
        for i in 0..n {
            for j in (i + 1)..n {
                let (pi, pj) = (&doc.points[i], &doc.points[j]);
                if pi.fixed && pj.fixed {
                    continue; // both pinned → a coincident is unsatisfiable
                }
                let d = ((pi.x - pj.x).powi(2) + (pi.y - pj.y).powi(2)).sqrt();
                if d > tol {
                    continue;
                }
                if find(&mut parent, i) == find(&mut parent, j) {
                    continue; // already coincident (directly or transitively)
                }
                let (ki, kj) = (id_key(&pi.id), id_key(&pj.id));
                if geo_sets.iter().any(|s| s.contains(&ki) && s.contains(&kj)) {
                    continue; // endpoints of one curve — do not collapse it
                }
                union(&mut parent, i, j);
                coincidents.push(vec![pi.id.clone(), pj.id.clone()]);
            }
        }
        for pts in coincidents {
            push_geometric_constraint(doc, "≡", pts);
        }
    }

    doc.constraints.len().saturating_sub(before)
}

impl EngineState {
    /// Auto-constrain the active sketch (the toolbar's one-click "constrain what I
    /// roughed in"): infer `━`/`│` on nearly-axis-aligned lines and `≡` between
    /// near-coincident points, then re-solve. Records ONE undo step, popped when the
    /// pass adds nothing so a dead click neither pollutes undo nor clobbers redo.
    /// Returns the number of constraints added; a no-op (0) when not in sketch mode.
    pub fn sketch_auto_constrain(&mut self) -> usize {
        let Some(edit) = self.sketch_edit.as_mut() else {
            return 0;
        };
        edit.record_undo();
        let added = auto_constrain_doc(&mut edit.session.doc);
        if added == 0 {
            edit.undo_stack.pop();
            return 0;
        }
        self.resolve_active_sketch("auto-constrain");
        self.refresh_sketch_overlay();
        self.dirty = true;
        added
    }
}

// ===========================================================================
// Sketch delete-selected (S3b) — remove the selected entities + orphan cleanup.
//
// Operates on the active `self.sketch_edit`. Rule (chosen so a remaining geometry
// NEVER references a missing point):
//   1. Partition the selection into selected geometry / point / constraint ids.
//   2. Drop every geometry that is SELECTED *or* references any selected point (the
//      remove-point cascade — deleting a vertex kills geometry that used it).
//   3. Drop the selected points.
//   4. Orphan cleanup: drop any remaining point NOT referenced by any surviving
//      geometry (a shared vertex — still referenced — stays; a deleted line's now
//      unshared endpoints vanish). Always on for this slice.
//   5. Drop any constraint that is SELECTED *or* references a removed point (selected ∪
//      orphaned) — done LAST, over the full removed-point set, so no constraint dangles
//      either. A selected constraint drops ONLY itself; the geometry/points it
//      referenced are untouched (deleting a constraint never deletes geometry).
// Then clear selection + hover, re-solve (swallowing errors), refresh, mark dirty.
// Kept in ONE appended block so concurrent edits to the primary impl land clean.
// ===========================================================================
impl EngineState {
    /// Delete the selected sketch entities (S3b): the selected geometries + points,
    /// plus any geometry orphaned by a deleted vertex, plus orphaned points and the
    /// constraints referencing any removed point. Re-solves + refreshes the overlay.
    /// Returns `true` when something was deleted; `false` when not in sketch mode or
    /// the selection is empty.
    pub fn sketch_delete_selection(&mut self) -> bool {
        use crate::sketch::doc::id_key;
        use std::collections::HashSet;

        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        if edit.session.selection.is_empty() {
            return false;
        }
        // A non-empty selection always removes something → snapshot for undo (S6a).
        edit.record_undo();

        // 1. Partition the selection into selected geometry / point / constraint ids
        //    (keyed via `id_key`, so 4 / 4.0 / "4" all match).
        let mut sel_geo: HashSet<String> = HashSet::new();
        let mut sel_pt: HashSet<String> = HashSet::new();
        let mut sel_constraint: HashSet<String> = HashSet::new();
        for r in &edit.session.selection {
            match (r.get("kind").and_then(|v| v.as_str()), r.get("id")) {
                (Some("geometry"), Some(id)) => {
                    sel_geo.insert(id_key(id));
                }
                (Some("point"), Some(id)) => {
                    sel_pt.insert(id_key(id));
                }
                (Some("constraint"), Some(id)) => {
                    sel_constraint.insert(id_key(id));
                }
                _ => {}
            }
        }

        // The part-origin reference and its ground are permanent: never deleted,
        // selected or orphaned.
        let origin_key = id_key(&serde_json::Value::String(
            brep_kernel::SKETCH_ORIGIN_POINT_ID.to_string(),
        ));
        sel_pt.remove(&origin_key);
        sel_constraint.remove(&origin_key);

        let doc = &mut edit.session.doc;

        // 2. Drop geometries that are selected OR reference any selected point (so a
        //    deleted vertex never leaves a geometry dangling).
        doc.geometries.retain(|g| {
            if sel_geo.contains(&id_key(&g.id)) {
                return false;
            }
            !g.points.iter().any(|pid| sel_pt.contains(&id_key(pid)))
        });

        // 3. Drop the explicitly-selected points.
        doc.points.retain(|p| !sel_pt.contains(&id_key(&p.id)));

        // 4. Orphan cleanup: drop points no longer referenced by any surviving
        //    geometry. Accumulate every removed point id (selected ∪ orphaned).
        let referenced: HashSet<String> = doc
            .geometries
            .iter()
            .flat_map(|g| g.points.iter().map(id_key))
            .collect();
        let mut removed_pts = sel_pt;
        doc.points.retain(|p| {
            let key = id_key(&p.id);
            if referenced.contains(&key) || key == origin_key {
                true
            } else {
                removed_pts.insert(key);
                false
            }
        });

        // 5. Drop constraints that are EXPLICITLY selected OR reference ANY removed
        //    point (done last, over the full removed set, so no constraint dangles onto
        //    a missing point). A selected constraint drops ONLY itself — the geometry /
        //    points it references are left intact (deleting a constraint never deletes
        //    geometry).
        doc.constraints.retain(|c| {
            if let Some(id) = c.raw.get("id") {
                if sel_constraint.contains(&id_key(id)) {
                    return false;
                }
            }
            !c.points().iter().any(|pid| removed_pts.contains(&id_key(pid)))
        });

        // Drop any external-reference bookkeeping whose materialized entities this
        // delete removed — otherwise its stale entry (keyed by edge name, now holding
        // dangling point ids) permanently blocks RE-LINKING the same edge.
        crate::sketch::external_ref::prune_dead_refs(&edit.session.doc, &mut edit.external_refs);

        // Clear the interaction state, re-solve (keep the doc on failure), refresh.
        edit.session.clear_selection();
        edit.session.set_hover(None);
        self.resolve_active_sketch("delete");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }
}
