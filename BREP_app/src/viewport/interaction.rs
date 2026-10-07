use super::*;

/// The RAW (unsmoothed) vertical wheel delta this frame, in UI points — summed
/// straight from the `MouseWheel` events instead of egui's `smooth_scroll_delta`.
/// egui smooths a wheel notch across ~6-10 frames (an ease-in/ease-out ramp that
/// reads as "dampening" at the start/end of a zoom); the raw events give one clean
/// discrete step per notch. Line/Page units are normalized to points the SAME way
/// egui's smoothing would (default `line_scroll_speed` = 40, private on
/// `InputState`, so mirrored here), so the calibrated `controls::wheel` step is
/// unchanged — only the ramp is gone.
pub(super) fn raw_wheel_delta_y(ctx: &egui::Context) -> f32 {
    const LINE_POINTS: f32 = 40.0;
    ctx.input(|i| {
        i.events
            .iter()
            .filter_map(|event| match event {
                egui::Event::MouseWheel { unit, delta, .. } => Some(match unit {
                    egui::MouseWheelUnit::Point => delta.y,
                    egui::MouseWheelUnit::Line => delta.y * LINE_POINTS,
                    egui::MouseWheelUnit::Page => delta.y * LINE_POINTS * 20.0,
                }),
                _ => None,
            })
            .sum()
    })
}

/// The shortest dwell that may ever open the pick list, whatever the style says.
/// A point-and-click ends with a corrective sub-movement and the button goes down
/// within roughly one to two tenths of a second of the pointer settling, so any
/// threshold inside that band would turn ordinary deliberate clicks into list
/// openings. A quarter second clears it with room to spare and is still short
/// enough to reach on purpose. It only BINDS if a theme drives `tooltip_delay`
/// below it; at the stock 0.5 s the style's value is what applies.
const DWELL_FLOOR_SECS: f64 = 0.25;

/// How long a hover highlight must stand before a plain click opens the PICK
/// LIST instead of selecting it: egui's OWN `interaction.tooltip_delay` (0.5 s by
/// default), floored at [`DWELL_FLOOR_SECS`].
///
/// Borrowed rather than re-invented, because it is the same judgement: the delay
/// is this toolkit's existing answer to "the pointer has stopped travelling and
/// is now inspecting what is under it", and that is exactly the moment the pick
/// list wants. It also means the list opens on the beat the user already knows
/// from every tooltip in the app, and that a slow-hands setting moves both
/// together instead of leaving the two disagreeing.
fn dwell_secs(ctx: &egui::Context) -> f64 {
    (f64::from(ctx.global_style().interaction.tooltip_delay)).max(DWELL_FLOOR_SECS)
}

impl Viewport {
    /// The rect (egui points) the viewport last drew into. Now that the viewport
    /// is a dock PANE (its rect moves as the user re-frames it), the shell anchors
    /// the floating context / Finish-Cancel overlays to THIS rect's right edge so
    /// they stay glued to the 3D view — see the top-right overlay in `app.rs`.
    /// `None` before the first draw (shell falls back to window-right until then).
    pub fn last_rect(&self) -> Option<egui::Rect> {
        self.last_rect
    }

    /// The last viewport rect as `{x, y, w, h}` in egui points (verification: the
    /// origin lets the verifier map engine viewport-local pick coords → page px).
    pub fn viewport_rect_json(&self) -> String {
        match self.last_rect {
            Some(r) => {
                serde_json::json!({ "x": r.min.x, "y": r.min.y, "w": r.width(), "h": r.height() })
                    .to_string()
            }
            None => "null".to_string(),
        }
    }

    /// The clean entry the shell calls: fill the central panel with the 3D
    /// viewport — size tracking, input routing, on-demand render, and the blit
    /// composite. Borrows the engine brain to draw + drive.
    pub fn show(&mut self, ui: &mut egui::Ui, state: &mut EngineState) {
        // An eCAD workbench active over a live sketch is not a state the app
        // keeps: the sketch is FINISHED, as its Finish button does, and the next
        // frame draws the dock and the editor. This is the one place every road
        // to it passes — the switcher, `settings_set`, a tab switch — because
        // in sketch mode the shell draws this and no dock. Finishing keeps the
        // drawing (discarding would lose it silently, and refusing the switch
        // leaves a workbench that cannot draw its editor), and Finish refuses
        // nothing, so neither does this.
        if crate::workbench::ecad::Target::of_workbench(&state.settings.workbench).is_some()
            && state.sketch_mode()
        {
            let _ = state.exit_sketch_mode(true);
        }
        let ppp = ui.ctx().pixels_per_point();
        // Empty frame → NO inner margin/padding: the 3D view fills the central
        // area edge-to-edge (the viewport paints the whole rect anyway).
        egui::containers::panel::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                let rect = ui.available_rect_before_wrap();
            self.last_rect = Some(rect);
            let response = ui.allocate_rect(rect, egui::Sense::click_and_drag());

            // Track viewport size in the engine (logical px) + offscreen (physical).
            let phys_w = (rect.width() * ppp).round().max(1.0) as u32;
            let phys_h = (rect.height() * ppp).round().max(1.0) as u32;
            self.ensure_offscreen(phys_w, phys_h);
            state.resize(rect.width() as f64, rect.height() as f64);

            // A document with an OPEN SHEET draws paper here instead of the
            // 3D scene: no offscreen render, no blit, and none of the 3D
            // input routing below. `show_sheet` says whether it took the
            // tile.
            if self.show_sheet(ui, rect, &response, state) {
                return;
            }

            // Feed input BEFORE rendering so a change is reflected this frame.
            self.handle_viewport_input(ui, rect, &response, state);

            // Per-frame overlay upkeep, AFTER the input + resize above so a zoom
            // is reflected in the SAME frame it happened. Re-bakes the
            // screen-constant sizing of every draggable gizmo that rides the
            // pre-expanded `set_overlay` channel — assembly-constraint handles
            // (§8.4), the ◎ feature-dimension gizmo, the live sketch overlay —
            // on a material world-per-pixel change, and hides/restores the
            // constraint graphics with their toggle + sketch mode.
            {
                let _span = crate::perf::span(crate::perf::Phase::Overlays);
                state.ensure_overlays_current();
            }

            if state.dirty {
                self.render_viewport(phys_w, phys_h, ppp, state);
                // Keep animating while a drag is live / more input pending.
                ui.ctx().request_repaint();
            }

            // Composite the offscreen 3D texture into egui's frame via the
            // wgpu paint callback.
            ui.painter().add(egui_wgpu::Callback::new_paint_callback(
                rect,
                ViewportCallback,
            ));
        });

        // The "candidates under the cursor" disambiguation popup (Alt+click)
        // floats over the viewport at ctx level, like the file dialog / palette.
        let ctx = ui.ctx().clone();
        self.show_candidate_popup(&ctx, state);

        // Editable dimension labels (S5): value text drawn at each dimension's
        // screen-projected anchor, click-to-edit + drag-to-reposition. Drawn at ctx
        // level (foreground of the viewport) so it floats over the 3D like the popup.
        // NONE of these belong on a sheet: they are 3D-anchored overlays, and
        // the sheet mode already drew everything the paper carries.
        let sheet_open = state.sheet_open().is_some();
        if let Some(rect) = self.last_rect.filter(|_| !sheet_open) {
            self.draw_dimension_labels(&ctx, rect, state);
            // Feature-dimension labels (FD-1): the ◎ dimension-gizmo mode draws the
            // primitive's param dims here, click-to-edit + drag-to-resize.
            self.draw_feature_dimension_labels(&ctx, rect, state);
            // Transform-gizmo axis labels (XC/YC/ZC): the colored cone-tip labels.
            // PAINTED, not laid out — they must never take a press from the
            // gizmo underneath them (see the function's own note).
            Self::draw_transform_axis_labels(&ctx, rect, state);
            // The gizmo's own READOUT: what the drag is worth, and — when the
            // motion refused — why, said at the gizmo rather than in a dock on
            // the far side of the window. Also the Transform Face PIVOT marker,
            // for the pivot the gizmo has moved off.
            self.draw_transform_readout(&ctx, rect, state);
            // Assembly-constraint labels (§8.4): status-colored chips at each
            // constraint's anchor — hover highlights the referenced geometry,
            // click expands the row in the Assembly Constraints panel.
            self.draw_constraint_labels(&ctx, rect, state);
            // PMI labels: the active view's annotation chips — drag to move
            // the label, click to select the annotation, double-click to open
            // it, right-click for its menu, hover to highlight its geometry.
            self.draw_pmi_labels(&ctx, rect, state);
            // DEBUG: 1px RED outline of the EXACT gizmo-arrow hit regions (axis
            // capsules + grab circles of the transform widget — feature transform
            // mode AND the component Move gizmo — or the dimension arrowheads).
            // Drawn LAST so the outlines overlay everything.
            self.draw_gizmo_hit_areas(&ctx, rect, state);
        }
        if self.candidate_popup.is_some() || state.dirty {
            // Keep animating while the popup is open (its entry hover / a fresh
            // highlight is applied AFTER this frame's render).
            ctx.request_repaint();
        }

        // Verification hooks (wasm only): the selection-UX globals the headed
        // verifier reads. Published from HERE because viewport.rs owns hover + the
        // candidate popup (keeps app.rs untouched). Purely additive.
        if crate::automation::registry::enabled() {
            crate::automation::registry::publish("__brepHover", "hovered entity", &state.hovered_json());
            // The HOVER DWELL — which of the two plain-click behaviours is armed
            // right now, and how long the current highlight has stood. A script
            // dwells by stepping frames between a `pointer_move` and a `click`
            // (the headless harness advances egui's clock by a fixed dt per
            // frame), and reads this to know which click it is about to make.
            crate::automation::registry::publish("__brepHoverDwell", "hover dwell {armed, age, threshold} — armed = a plain click opens the pick list instead of selecting the highlight", &self.hover_dwell_json(&ctx));
            crate::automation::registry::publish("__brepCandidates", "open pick-candidate popup entries [{index,kind,name,solid,depth}]", &self.candidates_json(state));
            crate::automation::registry::publish("__brepCandidateHit", "pick-candidate popup entry rects", &self.candidate_hits_json());
            // Re-publish the selection with the POST-input value: the app shell
            // publishes `__brepSelection` before this viewport draws (so its copy
            // lags a viewport click by a frame); overwrite it here with the value
            // that reflects this frame's click so the verifier reads it live.
            crate::automation::registry::publish("__brepSelection", "selection {solids, faces, edges, datums, vertices}; the name arrays are in pick order, oldest first", &state.selection_json());
            // The ◎ dimension-gizmo state (mode + annotations) for the verifier.
            crate::automation::registry::publish("__brepFeatureDim", "dimension gizmo state (mode, annotations)", &state.feature_dimension_state_json());
            // The transform gizmo's readout chip — the live drag's distance or
            // angle, and the REFUSAL when the motion is one the kernel will not
            // build. `active` is false whenever the chip is not drawn.
            crate::automation::registry::publish("__brepGizmoReadout", "transform gizmo readout {active, kind, axis, value, unit, text, origin, dragging, refused, motion, reason, hint, kernel} \u{2014} the live drag's distance/angle and, on a refusal, why it refused", &state.transform_readout_json());
            // The assembly-constraint overlay labels (id/text/status/color/world/
            // draggable) so the verifier can locate + drag a constraint handle.
            crate::automation::registry::publish("__brepConstraints", "assembly constraint overlay labels", &state.constraint_labels_json());
            // The active PMI view's label chips (id/type/text/status/world) so
            // the verifier can locate, drag and click an annotation.
            crate::automation::registry::publish("__brepPmiLabels", "PMI label chips of the active view", &state.pmi_labels_json());
            crate::automation::registry::publish("__brepPmiLabelHit", "PMI label chip rects {id: [x,y,w,h]}", &self.pmi_label_hits_json());
            // A chip's open right-click menu — `pmimenu/menuitem:<id>:<action>`.
            crate::automation::registry::publish("__brepPmiMenuHit", "open PMI label chip right-click menu entry rects (menuitem:<annotation id>:<action id>)", &crate::automation::hit_rects::hits_json(&self.pmi_menu_hits));
            // The armed gizmo's handle rects — `gizmo/…`, the VIEWPORT-hosted
            // widget panel, in SCREEN points like every other hit blob.
            crate::automation::registry::publish("__brepGizmoHit", "armed gizmo handle rects in screen points (anchor)", &gizmo_anchor_hits_json(self.last_rect, state));
            // The DRAGGABLE constraint handles' grab points — `constraint/<id>`,
            // the other VIEWPORT-hosted widget panel, in SCREEN points.
            crate::automation::registry::publish("__brepConstraintHit", "draggable assembly-constraint handle rects in screen points (by constraint id)", &constraint_handle_hits_json(self.last_rect, state));
            // The ViewCube corner rect (viewport-local `{x,y,w,h}`) so the verifier
            // can click a cube face/edge/corner by hit-rect (offset by `__brepView`).
            crate::automation::registry::publish("__brepViewCube", "ViewCube rect (viewport-local)", &state.viewcube_rect_json());
            // The SHEET mode's widgets — the paper, each placement and each
            // authored mark — in SCREEN points, empty while no sheet is open.
            // The paper carries no toolbar: the sheet's TOOLS are workbench
            // buttons, published under `toolbar/workbench:btn:`.
            crate::automation::registry::publish("__brepSheetHit", "open sheet's widget rects in screen points (paper, view:<id>, dim:<id>, ord:<id>; anchor:<reference> while the reference picker is up for a sheet object)", &self.sheet_hits_json());
        }
        // No eCAD editor is drawn this frame.
        self.publish_no_ecad();
    }

    /// The sheet mode's widget rects (`sheet/…`, screen points).
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    fn sheet_hits_json(&self) -> String {
        self.sheet.hits_json()
    }

    /// The PMI chips' screen rects (`{id: [x, y, w, h]}`, egui points) —
    /// the `__brepPmiLabelHit` verifier global.
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    fn pmi_label_hits_json(&self) -> String {
        let map: serde_json::Map<String, serde_json::Value> = self
            .pmi_label_hits
            .iter()
            .map(|(id, rect)| {
                (
                    id.clone(),
                    serde_json::json!([rect.min.x, rect.min.y, rect.width(), rect.height()]),
                )
            })
            .collect();
        serde_json::Value::Object(map).to_string()
    }

    /// The ViewCube corner rect (logical px, viewport-local), if enabled.
    fn viewcube_local(&self, state: &EngineState, x: f64, y: f64) -> Option<(f64, f64)> {
        viewcube_local(state, x, y)
    }

    /// Route pointer/wheel over the viewport into the engine's SKETCH interaction
    /// (S2), while [`EngineState::sketch_mode`]. Point drags move sketch points;
    /// empty-space drags still orbit/pan the camera; clicks select (Ctrl/Cmd adds);
    /// hover lights the entity under the cursor. The ViewCube corner + wheel zoom
    /// keep working. NONE of the modeling select/candidate/transform/ref-select
    /// branches run here.
    fn handle_sketch_input(
        &mut self,
        ui: &egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        state: &mut EngineState,
    ) {
        let local = |p: egui::Pos2| ((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64);

        // A DRAW tool (point/line/rect/circle/arc) is active vs S2 selection mode.
        // In draw mode clicks PLACE geometry and points are NOT grabbed for dragging
        // (so an empty drag still orbits the camera); hover still runs for snap + the
        // rubber-band preview.
        let draw_mode = state.sketch_active_tool().is_some();

        // Right-click aborts the in-progress draw geometry. (Escape → drop back to
        // the Select/drag tool is handled globally in `BrepApp::handle_shortcuts`,
        // the only reliable capture point: it `consume_key`s Escape before the
        // viewport runs, and it fires for EVERY armed tool, not just draw mode.)
        if draw_mode && response.secondary_clicked() {
            state.sketch_tool_cancel();
        }

        // Delete / Backspace removes the current sketch selection (S3b) — geometries +
        // points + constraints + orphan cleanup, driven by the engine. Gated on sketch
        // mode AND on no egui TEXT edit being focused, so a Backspace typed into an open
        // dimension value editor edits the number rather than deleting the selection
        // (same guard `handle_shortcuts` uses for the global keys).
        let delete = !ui.ctx().text_edit_focused()
            && ui
                .ctx()
                .input(|i| i.key_pressed(egui::Key::Delete) || i.key_pressed(egui::Key::Backspace));
        if delete {
            state.sketch_delete_selection();
        }

        if response.drag_started() {
            if let Some(pos) = response.interact_pointer_pos() {
                let (lx, ly) = local(pos);
                // ViewCube first (a view snap); then the freehand handdraw stroke
                // capture (S6b-3 — a drag IS the stroke, never a camera orbit); then a
                // sketch point grab (SELECT mode only — other draw modes never grab, so
                // an empty drag orbits); else an empty-space camera orbit/pan so
                // navigation still works.
                if let Some((cx, cy)) = self.viewcube_local(state, lx, ly) {
                    state.viewcube_click(cx, cy);
                } else if state.sketch_active_tool() == Some("handdraw") {
                    state.sketch_handdraw_begin(lx, ly);
                    self.sketch_handdrawing = true;
                } else if !draw_mode && state.sketch_drag_begin(lx, ly) {
                    self.sketch_dragging = true;
                } else {
                    let btn = if response.dragged_by(egui::PointerButton::Secondary) {
                        BUTTON_RIGHT
                    } else if response.dragged_by(egui::PointerButton::Middle) {
                        BUTTON_MIDDLE
                    } else {
                        BUTTON_LEFT
                    };
                    state.pointer_down(lx, ly, btn);
                    self.dragging = true;
                }
            }
        }
        if response.dragged() {
            if let Some(pos) = response.interact_pointer_pos() {
                let (lx, ly) = local(pos);
                if self.sketch_handdrawing {
                    state.sketch_handdraw_move(lx, ly);
                } else if self.sketch_dragging {
                    state.sketch_drag_to(lx, ly);
                } else if self.dragging {
                    state.pointer_move(lx, ly);
                }
            }
        }
        if response.drag_stopped() {
            if self.sketch_handdrawing {
                state.sketch_handdraw_end();
                self.sketch_handdrawing = false;
            }
            if self.sketch_dragging {
                state.sketch_drag_end();
                self.sketch_dragging = false;
            }
            if self.dragging {
                state.pointer_up();
                self.dragging = false;
            }
        }

        // A plain click either PLACES draw-tool geometry (draw mode) or selects the
        // entity under the cursor (SELECT mode: Ctrl/Cmd adds/toggles, empty clears).
        // A click over the ViewCube corner snaps the camera, never a sketch pick/place.
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let (lx, ly) = local(pos);
                if let Some((cx, cy)) = self.viewcube_local(state, lx, ly) {
                    // A plain click on the ViewCube corner snaps the camera. A plain
                    // click never fires drag_started (see the modeling path), so the
                    // snap must run here, not only on the drag-start branch above.
                    state.viewcube_click(cx, cy);
                } else if state.sketch_active_tool() == Some("handdraw") {
                    // handdraw (S6b-3) captures a DRAG as a stroke; a plain click (no
                    // drag) is a deliberate no-op.
                } else if state.sketch_active_tool() == Some("pickEdges") {
                    // pickEdges (S6b-2) acts on the 3D SCENE EDGE under the cursor
                    // (pixel coords), never a plane place/select.
                    state.sketch_pick_edge_at(lx, ly);
                } else if draw_mode {
                    state.sketch_tool_click_at(lx, ly);
                } else {
                    let mods = ui.ctx().input(|i| i.modifiers);
                    state.sketch_click_at(lx, ly, mods.command || mods.ctrl);
                }
            }
        }

        // Hover: the entity under the pointer (skip while dragging / over the cube).
        // Also FREEZE the entity hover while the primary button is held in SELECT
        // mode: between the press and egui's drag-start (~6px of movement later) the
        // pointer keeps moving, and re-hovering there would slide the highlight off
        // the point the user pressed — so the grab (which takes the hovered entity)
        // would miss. Draw mode keeps updating (its rubber-band preview rides the
        // live hover).
        let primary_held = ui.input(|i| i.pointer.primary_down());
        if !self.sketch_dragging
            && !self.dragging
            && !self.sketch_handdrawing
            && !(primary_held && !draw_mode)
        {
            match response.hover_pos() {
                Some(pos) => {
                    let (lx, ly) = local(pos);
                    match self.viewcube_local(state, lx, ly) {
                        Some((cx, cy)) => {
                            state.viewcube_hover(cx, cy);
                            state.sketch_clear_hover(); // over the cube, not the sketch
                        }
                        None => {
                            state.viewcube_clear_hover();
                            state.sketch_hover_at(lx, ly);
                            // pickEdges (S6b-2) targets a 3D scene edge, so also
                            // hover-highlight the edge under the cursor (modeling
                            // emphasis) as a link affordance.
                            if state.sketch_active_tool() == Some("pickEdges") {
                                state.hover_at(lx, ly);
                            }
                        }
                    }
                }
                None => {
                    state.viewcube_clear_hover();
                    // Pointer is off the viewport entirely. Don't clobber a hover the
                    // entity-LIST panel set THIS frame (it drew before us) — that is
                    // the list→canvas highlight. When the panel didn't set one, clear
                    // as usual so a stale highlight doesn't linger.
                    if !state.take_sketch_list_hover() {
                        state.sketch_clear_hover();
                    }
                }
            }
        }

        // Wheel zoom toward the cursor (same as modeling).
        if response.hovered() {
            // Raw (unsmoothed) wheel delta so each notch is a discrete step with NO
            // ease-in/out ramp (egui's smooth_scroll_delta dampens the start/end of a
            // scroll). See [`raw_wheel_delta_y`].
            let scroll_y = raw_wheel_delta_y(ui.ctx());
            if scroll_y != 0.0 {
                let cursor = response.hover_pos().map(|p| {
                    let (lx, ly) = local(p);
                    [lx, ly]
                });
                state.wheel(-(scroll_y as f64), cursor);
            }
        }
    }

    /// Route pointer/wheel over the viewport into `EngineState` (mirrors
    /// `desktop.rs`). `rect` is the viewport in egui points; coords fed to the
    /// engine are viewport-local logical px, the space `state.camera` lives in.
    fn handle_viewport_input(
        &mut self,
        ui: &egui::Ui,
        rect: egui::Rect,
        response: &egui::Response,
        state: &mut EngineState,
    ) {
        let local = |p: egui::Pos2| ((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64);

        let (events, focused, any_touches) =
            ui.ctx().input(|i| (i.events.clone(), i.focused, i.any_touches()));
        let touch = self.touch.update_for_frame(
            ui.ctx().cumulative_frame_nr(),
            &events,
            focused,
            any_touches,
            |pos| {
                ui.is_enabled()
                    && rect.contains(pos)
                    && ui.ctx().layer_id_at(pos) == Some(response.layer_id)
            },
        );
        self.touch_suppressed = touch.suppress_pointer;
        if touch.started {
            // Finish the existing single-finger interaction at its last valid
            // position before multitouch takes over, or when touch is cancelled.
            self.finish_pointer_drag(state);
            self.hover_lit_since = None;
            state.clear_hover();
            state.sketch_clear_hover();
            state.viewcube_clear_hover();
        }
        if touch.suppress_pointer {
            if let Some((from, to, scale)) = touch.motion {
                let (fx, fy) = local(from);
                let (tx, ty) = local(to);
                state.touch_navigation([fx, fy], [tx, ty], scale);
            }
            // Includes release/cancel frames: egui synthesizes mouse events for
            // touch, which must not also select, orbit, or drive a sketch tool.
            return;
        }

        // Sketch mode (S2) owns the pointer: hover/select/point-drag in plane space,
        // never the modeling select/candidate/transform/ref-select branches. Routed
        // BEFORE the modeling path, which stays byte-for-byte for `!sketch_mode()`.
        if state.sketch_mode() {
            self.handle_sketch_input(ui, rect, response, state);
            return;
        }

        if response.drag_started() {
            if let Some(pos) = response.interact_pointer_pos() {
                let (lx, ly) = local(pos);
                // Every drag-start ends the hover: the pointer is now steering
                // the camera / the cube / a gizmo handle / a dimension arrow,
                // not hovering the model — a frozen highlight riding a camera
                // snap, an orbit, or geometry that a gizmo/dim drag is reshaping
                // live reads as a stale pick. Hover re-resolves at the new pose
                // as soon as the interaction ends (the per-frame hover branch).
                state.clear_hover();
                // …and with the highlight goes the dwell clock: an orbit is not
                // an inspection, and the release must not land on an armed list.
                self.hover_lit_since = None;
                // WHERE the press landed — the point every handle hit test below is
                // taken at ([`drag_start_local`]), which is NOT `pos`: egui only calls
                // a press a drag once the pointer has left the click radius, so by
                // this frame `pos` has already drifted off whatever the user
                // pressed on.
                let (gx, gy) = drag_start_local(response, rect).unwrap_or((lx, ly));
                // A drag that STARTS over the ViewCube corner snaps/orbits via the
                // cube (a plain click — which never fires drag_started — is snapped
                // in the `clicked()` branch below); a press on an armed
                // transform-gizmo HANDLE drives the gizmo; anywhere else starts a
                // camera drag through the controls.
                match route_drag_start(state, gx, gy) {
                    // The cube already snapped the camera inside the router.
                    DragStart::ViewCube => {}
                    // Grabbed a gizmo handle → route the drag to the transform.
                    DragStart::Gizmo => self.gizmo_dragging = true,
                    // Grabbed the armed COMPONENT Move gizmo → free-move the gizmo
                    // during the drag, commit the pose (+ re-solve) on release.
                    DragStart::Component => self.component_gizmo_dragging = true,
                    // Grabbed a ◎ dimension ARROWHEAD (Fix 4) → route the drag to
                    // that param's live edit instead of orbiting the camera.
                    DragStart::Dimension(field) => self.dim_dragging = Some(field),
                    // Grabbed an ASSEMBLY-CONSTRAINT handle (a distance leader /
                    // angle-arc handle, §8.4 grabbable arrows) → the drag previews
                    // that constraint's value; release commits + auto-solves.
                    DragStart::Constraint => self.constraint_dragging = true,
                    DragStart::Camera => {
                        let btn = if response.dragged_by(egui::PointerButton::Secondary) {
                            BUTTON_RIGHT
                        } else if response.dragged_by(egui::PointerButton::Middle) {
                            BUTTON_MIDDLE
                        } else {
                            BUTTON_LEFT
                        };
                        // The camera anchors at the CURRENT pointer position, not
                        // the press origin: `pointer_move` deltas run from whatever
                        // `pointer_down` recorded, so anchoring at the (older) press
                        // origin would make the first orbit frame jump by the whole
                        // click-radius drift.
                        state.pointer_down(lx, ly, btn);
                        self.dragging = true;
                    }
                }
            }
        }
        if response.dragged() {
            if let Some(pos) = response.interact_pointer_pos() {
                let (lx, ly) = local(pos);
                if self.gizmo_dragging {
                    // Drive the transform gizmo: updates the feature's transform +
                    // re-runs the history (the model moves live).
                    state.transform_drag_to(lx, ly);
                } else if self.component_gizmo_dragging {
                    // Drive the component Move gizmo: the GIZMO follows the pointer
                    // (free move); the pose commits on release.
                    state.component_drag_to(lx, ly);
                } else if let Some(field) = self.dim_dragging.clone() {
                    // Drive the dimension arrow (Fix 4): edit the param + re-run the
                    // history live, so the geometry AND its arrow follow the pointer.
                    let feature = state.dimension_armed_feature();
                    if !feature.is_empty() {
                        state.feature_dimension_drag(&feature, &field, lx, ly);
                    }
                } else if self.constraint_dragging {
                    // Drive the constraint handle: the value PREVIEWS live (arrow +
                    // label track the pointer); nothing commits until release.
                    state.constraint_drag_to(lx, ly);
                } else if self.dragging {
                    state.pointer_move(lx, ly);
                }
            }
        }
        if response.drag_stopped() {
            if let (Some(field), Some(pos)) =
                (self.dim_dragging.as_ref(), response.interact_pointer_pos())
            {
                let (lx, ly) = local(pos);
                let feature = state.dimension_armed_feature();
                state.feature_dimension_drag(&feature, field, lx, ly);
            }
            self.finish_pointer_drag(state);
        }

        // A plain click (press+release, no drag — egui only reports `clicked()`
        // when the pointer never left the click radius, so an orbit or a pan is
        // never one of these) over the viewport: in reference-selection mode it
        // type-constrained-picks a reference under the cursor (engine drives the
        // highlight, and none of what follows applies); otherwise the modeling
        // selection UX, whose two plain-click behaviours are told apart by the
        // HOVER DWELL rather than by how much geometry happens to overlap:
        //
        //   * a click that arrives WITHOUT dwelling SELECTS the highlight — the
        //     nearest admitted hit by depth, the one already lit under the
        //     pointer (replace, or toggle in the Click-toggles multi-select
        //     mode). One click, the thing you were looking at, whether or not
        //     other geometry shares the pixel.
        //   * a click on a highlight that has been STANDING for the dwell opens
        //     the PICK LIST popup at the cursor, for the front/back faces and the
        //     obstructed geometry behind it.
        //
        // The list is the same list it has always been — same builder, same
        // ranking, same rows, same dismissal; only its TRIGGER moved, from "the
        // pixel was ambiguous" (which ambushed a click the user thought was
        // unambiguous) to "the user held still and asked for it". What makes that
        // legible is that the highlight is the promise: it shows what a quick
        // click takes, and `hover_dwell_armed` — the cursor turns into the
        // context-menu cursor — shows when a click has become a list instead.
        //
        // Ctrl/Cmd+click ADDS/TOGGLES that same highlighted pick directly (no
        // list, no dwell), and Alt+click still opens the pick list explicitly:
        // the dwell now reaches it too, but Alt remains the no-waiting path, and
        // the only path at all for a pointer that never dwells (an automation
        // script teleports and clicks in the same breath). ViewCube clicks are
        // snapped in this branch (checked right after the popup), never a
        // selection/pick.
        if response.clicked() {
            if let Some(pos) = response.interact_pointer_pos() {
                let on_popup = self.candidate_popup.is_some()
                    && self
                        .candidate_popup_rect
                        .map(|r| r.contains(pos))
                        .unwrap_or(false);
                let (lx, ly) = local(pos);
                let mods = ui.ctx().input(|i| i.modifiers);
                if on_popup {
                    // A click inside the OPEN popup belongs to the popup — its
                    // entry buttons handle it in `show_candidate_popup`. Don't let
                    // the viewport close it or pick the geometry behind it.
                } else if self.candidate_popup.is_some() {
                    // A click OUTSIDE the open pick list DISMISSES it and is
                    // SWALLOWED — it must not fall through to the selection
                    // branches below, or dismissing the list would re-pick (or, on
                    // empty space, CLEAR a multi-selection built through the
                    // list). The NEXT click acts normally.
                    self.candidate_popup = None;
                    self.candidate_popup_rect = None;
                } else if let Some((cx, cy)) = self.viewcube_local(state, lx, ly) {
                    // A click over the ViewCube corner snaps the camera to that
                    // region's standard view. Checked FIRST (after the popup) so a
                    // corner click is always a view snap, never a scene / ref-select
                    // / gizmo pick. A PLAIN click never fires `drag_started` (egui
                    // postpones the click/drag decision for a click_and_drag widget
                    // and only fires drag_started once the pointer is decidedly
                    // dragging), so the snap MUST run here — the drag-start path only
                    // catches a press that egui classifies as a drag.
                    state.viewcube_click(cx, cy);
                } else if state.ref_select_active() {
                    state.ref_select_click(lx, ly);
                } else if state.transform_center_pick(lx, ly) {
                    // The orange CENTER sphere in transform mode toggles to the
                    // DIMENSION arrows (the old app's center-handle ◎ toggle).
                    // Must precede the generic handle-swallow below so a center
                    // click flips modes instead of being swallowed; a center
                    // DRAG still free-moves (handled on drag-start, not here).
                    state.toggle_to_dimension();
                } else if state.dimension_origin_pick(lx, ly) {
                    // The orange ORIGIN sphere in dimension mode toggles back to
                    // the TRANSFORM controls (the reverse ◎ toggle).
                    state.toggle_to_transform();
                } else if state.dimension_arrow_pick(lx, ly).is_some() {
                    // A bare click on a dimension ARROWHEAD is a no-op — only a DRAG
                    // on it edits the value (Fix 4). Swallow it so the solid behind
                    // the arrow isn't selected.
                } else if state.constraint_arrow_pick(lx, ly).is_some() {
                    // Same rule for an assembly-constraint handle: only a DRAG edits
                    // the value; swallow the bare click so the geometry behind the
                    // leader/arc isn't selected.
                } else if (state.transform_armed() || state.component_move_armed())
                    && state.transform_pick(lx, ly) != 0
                {
                    // A bare click on an armed (non-center) gizmo handle — feature
                    // OR component Move gizmo — swallow it so the solid behind the
                    // gizmo is not selected.
                } else if mods.alt {
                    // Alt+click → open the pick list explicitly, even for a
                    // single candidate (the power-user inspection trigger).
                    let cands = state.candidates_filtered_at(lx, ly);
                    self.candidate_popup = (!cands.is_empty())
                        .then(|| CandidatePopup { anchor: pos, candidates: cands });
                    self.candidate_popup_fresh = self.candidate_popup.is_some();
                    // The dwell is SPENT on the list it opened: dismissing this
                    // one must not leave a clock that re-opens it on the spot.
                    self.hover_lit_since = None;
                } else if mods.command || mods.ctrl {
                    // Ctrl/Cmd+click ADDS/TOGGLES the top pick directly, no list
                    // (the classic additive shortcut, both multi-select modes).
                    state.select_toggle_at(lx, ly);
                } else if state.spline_anchor_pick_at(lx, ly).is_some() {
                    // An anchor dot of the spline whose editor is open: that
                    // anchor is selected (the gizmo arms on a free one) and the
                    // click stops here — the sheet under it is not selected.
                } else if self.hover_dwell_armed(ui.ctx()) {
                    // HOVERED, then clicked: the user held still over a highlight
                    // for the dwell and then pressed, which is how the pick list
                    // is asked for. The list itself is untouched —
                    // `candidates_filtered_at` builds the same ranked,
                    // filter-admitted rows it always did (construction planes
                    // among them as ordinary `PickKind::Plane` candidates), and
                    // `show_candidate_popup` draws and drives them. Only the
                    // question "should it open at all" is answered differently.
                    let cands = state.candidates_filtered_at(lx, ly);
                    self.candidate_popup = (!cands.is_empty())
                        .then(|| CandidatePopup { anchor: pos, candidates: cands });
                    self.candidate_popup_fresh = self.candidate_popup.is_some();
                    // Spent on the list it opened (see the Alt+click twin).
                    self.hover_lit_since = None;
                } else {
                    // A REGULAR click — no dwell — takes the highlight: the
                    // nearest admitted candidate by depth, the one already lit
                    // under the pointer. REPLACE in Ctrl+Click mode, TOGGLE in
                    // the Click-toggles mode (a second click on the same item
                    // unselects it). A miss clears.
                    //
                    // Overlapping geometry no longer diverts this click into a
                    // list: the thing in front is what the user aimed at and
                    // what the highlight promised, and the ones behind it are a
                    // dwell (or an Alt) away. `nearest_candidate_at` is the ONE
                    // query the highlight also resolves through, so the promise
                    // and the pick cannot drift apart; construction planes are
                    // ordinary candidates in it, so a plane genuinely in front of
                    // the model is now what a click takes, and a plane behind it
                    // is not (the pick list still ranks it by kind, as before).
                    let toggles =
                        state.settings.multi_select == MultiSelectMode::ClickToggles;
                    match state.nearest_candidate_at(lx, ly) {
                        Some(candidate) => {
                            if toggles {
                                state.toggle_candidate(&candidate);
                            } else {
                                state.select_candidate(&candidate);
                            }
                        }
                        None => {
                            state.clear_selection();
                        }
                    }
                    // A click SPENDS the rest that preceded it, exactly as the
                    // two list-opening branches do. Without this the clock kept
                    // running across the click, and the commonest repeat gesture
                    // there is — click to toggle an entity on, then click the
                    // same spot again to toggle it off — would find itself armed
                    // the second time and open a list instead. The next dwell
                    // starts from here.
                    self.hover_lit_since = None;
                }
            }
        }

        // Hover: the ViewCube corner, an armed transform-gizmo handle, OR the
        // top filter-admitted scene entity under the pointer (the modeling
        // hover-highlight). Suppressed while dragging / over a gizmo handle / mid
        // dimension-arrow or constraint-handle drag (feature, component-move, or
        // constraint gizmo), while the candidate popup owns the highlight, and
        // for the ONE frame a constraint LABEL applied its element highlight
        // (the label pass draws after us and re-arms the flag while hovered —
        // mirrors the sketch entity-list hover yield).
        let label_hover = state.take_constraint_label_hover() | state.take_pmi_label_hover();
        // The SCENE-TREE row hover (row → viewport highlight). Taken every frame
        // so the one-frame flag never survives a skipped one, but honored only in
        // the pointer-left-the-viewport branch below: the pointer is over the
        // sidebar while a row is hovered, so that branch — and only that branch —
        // would clear the row's highlight. Folding it into `label_hover` would also
        // skip `viewcube_clear_hover` and leave the cube lit when the pointer moves
        // from it to the tree.
        let tree_hover = state.take_scene_tree_hover();
        // The DIALOG row hover (a form's reference / `Outputs` line, the picker
        // card's picked names) — the same deal as the tree's, from a different
        // slot so the two panes cannot end each other's highlight.
        let dialog_hover = state.take_dialog_hover();
        if !self.dragging
            && !self.gizmo_dragging
            && !self.component_gizmo_dragging
            && self.dim_dragging.is_none()
            && !self.constraint_dragging
            && self.pmi_label_dragging.is_none()
            && !label_hover
        {
            match response.hover_pos() {
                Some(pos) => {
                    let (lx, ly) = local(pos);
                    match self.viewcube_local(state, lx, ly) {
                        Some((cx, cy)) => {
                            state.viewcube_hover(cx, cy);
                            state.clear_hover(); // over the cube, not the scene
                            self.hover_lit_since = None;
                        }
                        None => {
                            state.viewcube_clear_hover();
                            // Over an armed gizmo handle → highlight the handle, not
                            // the solid behind it (the previous app's "skip scene hover over
                            // the gizmo" rule). Else hover-highlight the nearest pick.
                            let over_handle = (state.transform_armed()
                                || state.component_move_armed())
                                && state.transform_hover(lx, ly) != 0;
                            if over_handle {
                                state.clear_hover();
                                self.hover_lit_since = None;
                            } else if self.candidate_popup.is_none() {
                                // THE modeling hover: lights what a regular click
                                // would take, and its return value says whether
                                // the highlight CHANGED — which is where the
                                // dwell clock restarts.
                                let changed = state.hover_at(lx, ly);
                                let lit = state.hover_is_lit();
                                self.note_hover(ui.ctx(), pos, lit, changed);
                            } else {
                                // The open pick list owns the highlight (its rows
                                // drive their own hover), so no dwell runs under
                                // it — a list cannot arm another list.
                                self.hover_lit_since = None;
                            }
                        }
                    }
                }
                None => {
                    state.viewcube_clear_hover();
                    // Pointer left the viewport (or moved onto the popup, which
                    // drives its own entry-hover) → drop the scene hover. UNLESS a
                    // Scene-tree row or a DIALOG row is hovering an entity: that
                    // highlight is the pointer's, drawn from the sidebar (mirrors
                    // the sketch entity-list hover yield).
                    if self.candidate_popup.is_none() && !tree_hover && !dialog_hover {
                        state.clear_hover();
                    }
                    // Either way no dwell is accruing: the pointer is not over
                    // the scene, and a highlight lit from a sidebar row is not
                    // one this viewport's click is about to act on.
                    self.hover_lit_since = None;
                }
            }
        } else {
            // Hover is SUPPRESSED this frame (a drag, a gizmo/arrow/handle drag,
            // or the one frame a label owns the highlight). Nothing on screen is
            // this viewport's hover promise, so the dwell cannot be accruing
            // toward a list either.
            self.hover_lit_since = None;
        }

        // Wheel zoom toward the cursor when hovering the viewport.
        if response.hovered() {
            // Raw (unsmoothed) wheel delta so each notch is a discrete step with NO
            // ease-in/out ramp (egui's smooth_scroll_delta dampens the start/end of a
            // scroll). See [`raw_wheel_delta_y`].
            let scroll_y = raw_wheel_delta_y(ui.ctx());
            if scroll_y != 0.0 {
                let cursor = response.hover_pos().map(|p| {
                    let (lx, ly) = local(p);
                    [lx, ly]
                });
                // egui scroll: +y = wheel up = zoom in; the controls treat
                // negative delta_y as zoom-in (see desktop.rs), so negate.
                state.wheel(-(scroll_y as f64), cursor);
            }
        }
    }

    /// Close captures before handing a single pointer to multitouch navigation.
    fn finish_pointer_drag(&mut self, state: &mut EngineState) {
        if self.gizmo_dragging {
            state.transform_release();
            self.gizmo_dragging = false;
        }
        if self.component_gizmo_dragging {
            state.component_release();
            self.component_gizmo_dragging = false;
        }
        if self.dim_dragging.take().is_some() {
            let feature = state.dimension_armed_feature();
            state.feature_dimension_release(&feature);
        }
        if self.constraint_dragging {
            state.constraint_drag_release();
            self.constraint_dragging = false;
        }
        if self.sketch_handdrawing {
            state.sketch_handdraw_end();
            self.sketch_handdrawing = false;
        }
        if self.sketch_dragging {
            state.sketch_drag_end();
            self.sketch_dragging = false;
        }
        if self.dragging {
            state.pointer_up();
            self.dragging = false;
        }
    }

    /// Whether a plain click RIGHT NOW opens the pick list instead of selecting
    /// the highlight — the whole discriminator, in one place.
    ///
    /// One clock, restarted by either of the two things that mean "this is not a
    /// highlight the user has been resting on":
    ///
    /// * the HIGHLIGHT changed — the pointer crossed onto another entity, or the
    ///   camera or the geometry moved under a pointer that never did. Without
    ///   this, an orbit that parks new geometry under a resting cursor would
    ///   arrive pre-armed.
    /// * the POINTER moved. Without this, travelling a long way across one big
    ///   face would accrue dwell for the whole journey, because the highlight
    ///   never changed, and the click at the end of the trip would open a list
    ///   nobody asked for.
    ///
    /// Read BEFORE this frame's hover resolve (the click branch runs first), so
    /// the answer describes the state the user could see when they pressed —
    /// never one this frame's own movement has already invalidated.
    fn hover_dwell_armed(&self, ctx: &egui::Context) -> bool {
        let Some(since) = self.hover_lit_since else {
            return false; // nothing lit → nothing to open a list about.
        };
        // `dwell_secs` reads the style, and `Context::input` holds the context
        // lock for the whole closure — calling it in there deadlocks (in release;
        // in debug egui panics on the recursion). Read it first.
        let dwell = dwell_secs(ctx);
        ctx.input(|i| i.time - since >= dwell)
    }

    /// Advance the hover DWELL clock after this frame's hover resolve, and keep
    /// the frames coming until it arms.
    ///
    /// The repaint request is not an optimization: this app renders ON DEMAND, so
    /// with the pointer at rest the frames simply stop, and a threshold that is
    /// only ever tested on input would never be crossed — the list would need a
    /// wiggle to become reachable. Asking for the frame that crosses it also
    /// paints the cursor cue at the right moment.
    fn note_hover(&mut self, ctx: &egui::Context, pos: egui::Pos2, lit: bool, changed: bool) {
        // Movement is measured against the pointer's own last position rather
        // than taken from an event, and with a couple of points of slop: a hand
        // resting on a mouse is not perfectly still, and a pixel of jitter is
        // not the user setting off somewhere.
        const SLOP: f32 = 2.0;
        let moved = self
            .hover_last_pos
            .is_none_or(|last| last.distance(pos) > SLOP);
        if moved {
            self.hover_last_pos = Some(pos);
        }
        if !lit {
            self.hover_lit_since = None;
            return;
        }
        let now = ctx.input(|i| i.time);
        if changed || moved || self.hover_lit_since.is_none() {
            self.hover_lit_since = Some(now);
        }
        if self.hover_dwell_armed(ctx) {
            // The one visible tell that this click has become a list: the
            // pointer takes the context-menu cursor, the same shape every other
            // "a click here opens a menu" in the desktop means. The rule is then
            // something the user can SEE coming instead of a hidden timer.
            ctx.set_cursor_icon(egui::CursorIcon::ContextMenu);
            return;
        }
        let dwell = dwell_secs(ctx);
        let since = self.hover_lit_since.unwrap_or(now);
        let remaining = ctx.input(|i| dwell - (i.time - since));
        if remaining > 0.0 {
            ctx.request_repaint_after(std::time::Duration::from_secs_f64(remaining.min(dwell)));
        }
    }

    /// Draw the PICK LIST popup — a semi-transparent, scrollable list of the
    /// ranked, filter-admitted candidates at the cursor, in the category order
    /// points > edges > faces > solids > components (nearest first within each).
    /// Opens on a click that arrives after the pointer has DWELLED on its hover
    /// highlight (so front + back faces and obstructed geometry are reachable
    /// behind what the highlight is promising) and on Alt+click explicitly. A
    /// click without that dwell selects the highlight instead and never comes
    /// here. Nothing below this line changed with that trigger. HOVERING an entry pre-highlights that entity in the scene
    /// (and hover-out clears it); a row draws SELECTED while its entity is in
    /// the selection, so toggling reads back visually. CLICKING an entry picks
    /// it and ALWAYS closes the list: in the Click-toggles multi-select mode
    /// (and on Ctrl/Cmd+click in either mode) the entry TOGGLES into the
    /// selection — front AND back faces join one selection across two
    /// click→entry rounds — while in Ctrl+Click mode a plain entry click
    /// REPLACES the selection with exactly it. The header's "Clear Selection"
    /// clears + closes. Also closes on Escape (routed via the app shell so it
    /// never also clears the selection) and on a click outside (swallowed by
    /// the viewport click router). Rebuilds `candidate_hits` (per-entry screen
    /// rects) each frame for the headed verifier. Engine mutations are applied
    /// AFTER the draw closure (the codebase's "no engine mutation inside the
    /// draw" rule).
    fn show_candidate_popup(&mut self, ctx: &egui::Context, state: &mut EngineState) {
        self.candidate_hits.clear();
        // A modal mode (reference-selection / sketch edit) supersedes the pick
        // list: drop a popup left open by modeling clicks so it neither draws
        // over the modal nor swallows the modal's first viewport click.
        if state.ref_select_active() || state.sketch_mode() {
            self.candidate_popup = None;
        }
        let Some(popup) = self.candidate_popup.as_ref() else {
            self.candidate_popup_rect = None;
            return;
        };
        let candidates = popup.candidates.clone();
        let anchor = popup.anchor;
        let mods = ctx.input(|i| i.modifiers);
        // Row selected-state, read BEFORE the draw (no engine borrow inside it).
        let selected_rows: Vec<bool> = candidates
            .iter()
            .map(|c| state.candidate_is_selected(c))
            .collect();

        let mut hits: Vec<egui::Rect> = Vec::with_capacity(candidates.len());
        let mut hovered_index: Option<usize> = None;
        let mut clicked_index: Option<usize> = None;
        let mut clear_clicked = false;

        let area = egui::Area::new(egui::Id::new("brep-candidate-popup"))
            .order(egui::Order::Foreground)
            .fixed_pos(anchor)
            // Keep the whole list on screen when the click lands near an edge.
            .constrain(true)
            .show(ctx, |ui| {
                // The standard popup frame at reduced opacity: the model stays
                // visible through the list while scrolling it.
                let mut frame = egui::Frame::popup(ui.style());
                frame.fill = frame.fill.gamma_multiply(0.85);
                frame.show(ui, |ui| {
                    ui.set_max_width(280.0);
                    // Header row: title + a "Clear Selection" action.
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new("Select an object").weak().small());
                        ui.with_layout(
                            egui::Layout::right_to_left(egui::Align::Center),
                            |ui| {
                                if ui.small_button("Clear Selection").clicked() {
                                    clear_clicked = true;
                                }
                            },
                        );
                    });
                    // A long candidate list scrolls instead of growing past the
                    // viewport; hover keeps re-resolving as rows slide under the
                    // pointer, so scrolling through the list highlights each
                    // entity in turn.
                    egui::ScrollArea::vertical()
                        .max_height(240.0)
                        .show(ui, |ui| {
                            ui.set_min_width(220.0);
                            for (i, candidate) in candidates.iter().enumerate() {
                                let label = format!(
                                    "{}  {}",
                                    state.candidate_kind_label(candidate),
                                    candidate_label(candidate)
                                );
                                // TRUNCATE inside the popup's cap. A candidate
                                // is labelled with the entity's own name, which
                                // the modelling history makes as long as it
                                // likes; left to extend, it grows this overlay
                                // card past the 280 pt it just asked for and out
                                // over the viewport edge. A `Button` (which is
                                // what a selectable label is) has no elided-text
                                // tooltip of its own, so the full name is spelled
                                // out on hover — the rows the user is choosing
                                // between often differ only in their tails.
                                let resp = ui
                                    .add(
                                        egui::Button::selectable(
                                            selected_rows[i],
                                            label.as_str(),
                                        )
                                        .truncate(),
                                    )
                                    .on_hover_text(&label);
                                hits.push(resp.rect);
                                if resp.hovered() {
                                    hovered_index = Some(i);
                                }
                                if resp.clicked() {
                                    clicked_index = Some(i);
                                }
                            }
                        });
                });
            });

        self.candidate_hits = hits;
        self.candidate_popup_rect = Some(area.response.rect);

        // Apply engine mutations outside the draw closure.
        if let Some(i) = hovered_index {
            state.hover_candidate(&candidates[i]);
        } else {
            // No entry under the pointer → drop the pre-highlight, so the
            // last-hovered row's entity doesn't stay lit while the pointer
            // roams elsewhere.
            state.clear_hover();
        }
        let mut close = false;
        if clear_clicked {
            state.clear_selection();
            close = true;
        }
        if let Some(i) = clicked_index {
            // Picking an entry ALWAYS dismisses the list — the pick is the
            // list's job and it is done. In Click-toggles mode (or with
            // Ctrl/Cmd held) the entry TOGGLES into the selection, so a
            // front+back multi-selection is click → front, click → back (the
            // list reopens on the next click); in Ctrl+Click mode a plain
            // entry click REPLACES the selection with exactly that entity.
            let toggles = state.settings.multi_select == MultiSelectMode::ClickToggles;
            if toggles || mods.command || mods.ctrl {
                state.toggle_candidate(&candidates[i]);
            } else {
                state.select_candidate(&candidates[i]);
            }
            close = true;
        }
        // Fallback only: the app shell's global Escape router consumes the key
        // first and closes via `close_candidate_popup` (so Escape never ALSO
        // clears the selection); this fires only when that router is skipped
        // (e.g. a text edit had focus).
        if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
            close = true;
        }
        // Ignore the OPENING Alt+click on the frame it opened; honor click-outside
        // from the next frame on.
        if self.candidate_popup_fresh {
            self.candidate_popup_fresh = false;
        } else if area.response.clicked_elsewhere() {
            close = true;
        }
        if close {
            state.clear_hover();
            self.candidate_popup = None;
            self.candidate_popup_rect = None;
        }
    }

    /// The OPEN popup's candidate list as JSON `[{index,kind,name,solid,depth}]`
    /// (empty when closed) — the headed verifier asserts the sorted list.
    fn candidates_json(&self, state: &EngineState) -> String {
        match self.candidate_popup.as_ref() {
            Some(popup) => {
                let out: Vec<serde_json::Value> = popup
                    .candidates
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        serde_json::json!({
                            "index": i,
                            "kind": state.candidate_kind_label(c),
                            "name": c.name,
                            "solid": c.solid,
                            "depth": c.depth,
                        })
                    })
                    .collect();
                serde_json::Value::Array(out).to_string()
            }
            None => "[]".to_string(),
        }
    }

    /// The OPEN popup's per-entry screen rects as the `{key: [x, y, w, h]}` map
    /// (egui points) every other panel publishes, keyed by the entry's INDEX —
    /// the same index `__brepCandidates` carries, and the rect list is built one
    /// per candidate in the same order, so row N of the list is key `"N"` here.
    ///
    /// The hover DWELL as JSON `{armed, age, threshold}` — the verification hook
    /// for the discriminator between the two plain-click behaviours: `armed` is
    /// what [`hover_dwell_armed`](Self::hover_dwell_armed) would answer for a
    /// click arriving now, `age` is how long the current highlight has stood (−1
    /// while none is lit) and `threshold` is the dwell in force.
    ///
    /// Published because the rule is a TIMING rule, and a script asserting "this
    /// click selected rather than opened a list" should be able to say WHY rather
    /// than infer it from the outcome it was testing.
    fn hover_dwell_json(&self, ctx: &egui::Context) -> String {
        let age = match self.hover_lit_since {
            Some(since) => ctx.input(|i| i.time - since),
            None => -1.0,
        };
        serde_json::json!({
            "armed": self.hover_dwell_armed(ctx),
            "age": age,
            "threshold": dwell_secs(ctx),
        })
        .to_string()
    }

    /// The map shape is what makes the popup reachable without a coordinate:
    /// `Registry::hit_rects` only collects a blob whose values are four-element
    /// rect arrays, so the `[{index, x, y, w, h}]` array this used to publish
    /// left `hit_rects candidate/` empty and `click_widget candidate/0`
    /// unresolvable, while `viewport::HIT_KEYS` documented the panel as
    /// "keyed by index" all along.
    fn candidate_hits_json(&self) -> String {
        let out: serde_json::Map<String, serde_json::Value> = self
            .candidate_hits
            .iter()
            .enumerate()
            .map(|(i, r)| {
                (
                    i.to_string(),
                    serde_json::json!([r.min.x, r.min.y, r.width(), r.height()]),
                )
            })
            .collect();
        serde_json::Value::Object(out).to_string()
    }
}

/// Which branch CLAIMED a viewport drag-start — the one dispatch that decides
/// whether a press drives a gizmo handle or the camera. Returned by
/// [`route_drag_start`], which performs the claim; the caller only records which
/// drag is now live.
#[derive(Debug, Clone, PartialEq, Eq)]
pub(super) enum DragStart {
    /// The ViewCube corner: the router already snapped the camera.
    ViewCube,
    /// A transform-gizmo handle (arrow / ring ball / center sphere).
    Gizmo,
    /// A handle of the armed assembly-component Move gizmo.
    Component,
    /// A ◎ dimension arrowhead; carries the param field key it edits.
    Dimension(String),
    /// An assembly-constraint distance/angle handle.
    Constraint,
    /// Nothing claimed it → the camera orbit/pan fallthrough.
    Camera,
}

/// The armed gizmo's frame origin as a zero-size rect in SCREEN points — the
/// `__brepGizmoHit` blob, which merges under the panel name `gizmo`
/// (`gizmo/anchor`). That point carries the transform gizmo's orange CENTRE
/// free-move handle, or — in dimension mode — the ORIGIN sphere that toggles
/// back to it, so one key addresses the ◎ toggle both ways.
///
/// ITS OWN BLOB, not the history panel's, for the two reasons that split
/// `__brepSketchHit` in two:
///   * SPACE — every rect in a hit blob is a SCREEN point, because that is what
///     a host clicks. This one is the VIEWPORT's widget, so it is the viewport
///     rect's origin (`view`) plus the engine's viewport-local projection;
///     published among the history panel's rects it was the one value in that
///     map that needed `__brepView` added before it meant anything on screen.
///   * CLIP — `click_widget` scrolls a key's panel until the rect is inside that
///     panel's `panel:clip`. The gizmo sits in the viewport, which does not
///     scroll and publishes no `panel:clip`, so the scroll-into-view step is
///     skipped and the rect is clicked where it is. Under the history panel it
///     was outside that pane's clip rect and a host wheeled the feature list
///     trying to reach it.
///
/// Beside the anchor, the transform widget's AXIS ARROWS and ROTATION GRABS are
/// published the same way (`gizmo/axis:x|y|z`, `gizmo/ring:x|y|z`), each at a
/// point [`EngineState::transform_pick`] answers with that handle — dragging
/// an arrow edits `position` along it, dragging a grab edits `rotationEuler`
/// about the axis its ring turns about. A handle whose point another handle
/// would win (an arrow looking straight at the camera) is not published.
///
/// Empty when no gizmo is armed, or before the viewport has a rect.
pub(super) fn gizmo_anchor_hits_json(view: Option<egui::Rect>, state: &EngineState) -> String {
    let Some(view) = view else {
        return "{}".to_string();
    };
    let at = |(x, y): (f64, f64)| {
        egui::Rect::from_min_size(egui::pos2(view.min.x + x as f32, view.min.y + y as f32), egui::Vec2::ZERO)
    };
    // The anchor, then each arrow and rotation grab the drawn transform widget
    // offers (`gizmo/axis:x`, `gizmo/ring:y`, …), then each ARROWHEAD the
    // DIMENSION gizmo offers (`gizmo/dim:sizeX`) — each at a point the engine's
    // own pick answers with that handle, so `drag_widget` on the key grabs it.
    //
    // The anchor is an OPTION rather than the gate it used to be: the two modes
    // are mutually exclusive but their handle lists are not the same shape, and
    // a dimension gizmo whose frame origin does not project is still a gizmo
    // with draggable arrowheads.
    let hits: Vec<(String, egui::Rect)> = state
        .transform_gizmo_anchor()
        .map(|point| ("anchor".to_string(), at(point)))
        .into_iter()
        .chain(
            state
                .transform_handle_points()
                .into_iter()
                .map(|(key, point)| (key.to_string(), at(point))),
        )
        .chain(
            state
                .dimension_handle_points()
                .into_iter()
                .map(|(key, point)| (format!("dim:{key}"), at(point))),
        )
        .chain(
            state
                .dimension_origin_points()
                .into_iter()
                .map(|(key, point)| (format!("ball:{key}"), at(point))),
        )
        .collect();
    if hits.is_empty() {
        return "{}".to_string();
    }
    crate::automation::hit_rects::hits_json(hits.iter().map(|(key, rect)| (key, rect)))
}

/// The DRAGGABLE assembly-constraint handles as zero-size rects in SCREEN
/// points — the `__brepConstraintHit` blob, which merges under the panel name
/// `constraint`, one key per constraint id (`constraint/DIST1`).
///
/// ITS OWN BLOB for the same two reasons as `gizmo/anchor`: the rects are the
/// VIEWPORT's, so they are the viewport rect's origin plus the engine's
/// viewport-local projection, and there is no `panel:clip` to scroll a handle
/// into — a host clicks it where it is. The point is the one
/// [`EngineState::constraint_arrow_pick`] matches at distance zero, so
/// `drag_widget {key: "constraint/<id>"}` presses the arrow rather than
/// near it.
///
/// Empty when no constraint publishes a draggable handle, or before the
/// viewport has a rect.
pub(super) fn constraint_handle_hits_json(view: Option<egui::Rect>, state: &EngineState) -> String {
    let Some(view) = view else {
        return "{}".to_string();
    };
    let Ok(serde_json::Value::Object(points)) =
        serde_json::from_str::<serde_json::Value>(&state.constraint_handle_points_json())
    else {
        return "{}".to_string();
    };
    let hits: Vec<(String, egui::Rect)> = points
        .iter()
        .filter_map(|(id, point)| {
            let point = point.as_array()?;
            let x = point.first()?.as_f64()? as f32;
            let y = point.get(1)?.as_f64()? as f32;
            Some((
                id.clone(),
                egui::Rect::from_min_size(
                    egui::pos2(view.min.x + x, view.min.y + y),
                    egui::Vec2::ZERO,
                ),
            ))
        })
        .collect();
    crate::automation::hit_rects::hits_json(hits.iter().map(|(key, rect)| (key, rect)))
}

/// The VIEWPORT-LOCAL point a drag-start hit test must be taken at. The ONE
/// place that choice is made — the dispatch and its tests both call THIS, so a
/// test can never pass against a call site that resolved the point differently.
///
/// NOT `response.interact_pointer_pos()`, which is the pointer's position on the
/// frame egui DECIDED the press was a drag — by then it has travelled at least
/// `max_click_dist` (6 pt) from the press, and a slow frame (the 3D viewport
/// waking from egui's on-demand repaint) coalesces the whole flick into one
/// step, so the reported point can be tens of px away. A gizmo handle carries
/// 7-9 px of grab radius (axis arrow / rotation ball / centre sphere), a
/// dimension leader 18, so hit-testing there misses the handle the user
/// actually pressed and the press falls through to the camera orbit.
///
/// `press_origin` is exactly where the button went down, so the test asks "what
/// did you press on", not "where has the cursor got to". It is `None` only when
/// no button is down — a press and release inside ONE frame — where the reported
/// interact position is all there is.
fn drag_start_local(response: &egui::Response, rect: egui::Rect) -> Option<(f64, f64)> {
    let p = response
        .ctx
        .input(|i| i.pointer.press_origin())
        .or_else(|| response.interact_pointer_pos())?;
    Some(((p.x - rect.min.x) as f64, (p.y - rect.min.y) as f64))
}

/// Dispatch ONE viewport drag-start at viewport-local px `(x, y)`: the first
/// branch whose handle is under the press CLAIMS it (and arms its drag inside
/// the engine), else the camera takes it.
///
/// Precedence is load-bearing and unchanged: ViewCube corner → transform gizmo →
/// component Move gizmo → ◎ dimension arrowhead → assembly-constraint handle →
/// camera. Split out of `handle_viewport_input` so the claim can be driven from
/// a test without a GPU-backed [`Viewport`].
pub(super) fn route_drag_start(state: &mut EngineState, x: f64, y: f64) -> DragStart {
    if let Some((cx, cy)) = viewcube_local(state, x, y) {
        state.viewcube_click(cx, cy);
        DragStart::ViewCube
    } else if state.transform_press(x, y) {
        DragStart::Gizmo
    } else if state.component_press(x, y) {
        DragStart::Component
    } else if let Some(field) = state.dimension_arrow_pick(x, y) {
        DragStart::Dimension(field)
    } else if state.constraint_drag_begin(x, y) {
        DragStart::Constraint
    } else {
        DragStart::Camera
    }
}

/// The ViewCube corner rect hit test (viewport-local logical px) — `Some` with
/// the CUBE-local coords when `(x, y)` is inside the drawn cube, else `None`.
fn viewcube_local(state: &EngineState, x: f64, y: f64) -> Option<(f64, f64)> {
    let v: serde_json::Value = serde_json::from_str(&state.viewcube_rect_json()).ok()?;
    let (rx, ry, rw, rh) = (
        v["x"].as_f64()?,
        v["y"].as_f64()?,
        v["w"].as_f64()?,
        v["h"].as_f64()?,
    );
    if rw > 0.0 && rh > 0.0 && x >= rx && x <= rx + rw && y >= ry && y <= ry + rh {
        Some((x - rx, y - ry))
    } else {
        None
    }
}

/// A human label for a pick candidate: its kernel name, or a positional tag for
/// unnamed vertices.
fn candidate_label(candidate: &PickCandidate) -> String {
    if candidate.name.trim().is_empty() {
        let p = candidate.position;
        format!("({:.2}, {:.2}, {:.2})", p[0], p[1], p[2])
    } else {
        candidate.name.clone()
    }
}

