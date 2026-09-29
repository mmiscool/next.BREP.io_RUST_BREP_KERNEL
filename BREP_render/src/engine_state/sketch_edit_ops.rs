use super::*;

// Sketch dimension editing shares geometry with crate::sketch::dimensions.
// Values may be literals or expressions evaluated against the history variables;
// dragged labels store plane-space offsets. Edits resolve the sketch and refresh
// its overlay.
impl EngineState {
    /// The dimension labels for the active sketch (S5): one entry per dimensional
    /// constraint — `[{ id, text, world:[x,y,z], value, valueExpr, mode }]`. `world`
    /// is the label anchor in world space; brep-app projects it via
    /// [`world_to_screen_json`](Self::world_to_screen_json) and draws editable text
    /// there. `[]` when not in sketch mode.
    pub fn sketch_dimension_labels_json(&self) -> String {
        let world_per_pixel = self.camera.world_per_pixel();
        let labels = match self.sketch_edit.as_ref() {
            Some(edit) => edit.session.dimension_labels(world_per_pixel),
            None => return "[]".to_string(),
        };
        let out: Vec<serde_json::Value> = labels
            .into_iter()
            .map(|l| {
                serde_json::json!({
                    "id": l.id,
                    "text": l.text,
                    "world": l.world,
                    "value": l.value,
                    "valueExpr": l.value_expr,
                    "mode": l.mode,
                    "conflicting": l.conflicting,
                })
            })
            .collect();
        serde_json::to_string(&out).unwrap_or_else(|_| "[]".to_string())
    }

    /// The current `{value, valueExpr, mode}` for a dimensional constraint (S5) — the
    /// seed for the inline edit field. `{}` when the constraint is absent / not in
    /// sketch mode. `mode` is `"distance" | "radius" | "diameter" | "angle"`. For a
    /// diameter dim `value` is the DISPLAYED diameter (twice the stored radius), so
    /// the edit field round-trips what the user sees.
    pub fn sketch_dimension_value_json(&self, constraint_id: &serde_json::Value) -> String {
        use crate::sketch::doc::id_key;
        let Some(edit) = self.sketch_edit.as_ref() else {
            return "{}".to_string();
        };
        let key = id_key(constraint_id);
        let Some(c) = edit
            .session
            .doc
            .constraints
            .iter()
            .find(|c| c.raw.get("id").map(id_key) == Some(key.clone()))
        else {
            return "{}".to_string();
        };
        let is_diameter = c.ctype() == Some("⟺")
            && c.raw.get("displayStyle").and_then(serde_json::Value::as_str) == Some("diameter");
        let mode = dimension_mode(c);
        let stored = c
            .raw
            .get("value")
            .and_then(serde_json::Value::as_f64)
            .filter(|v| v.is_finite());
        let display_value = stored.map(|v| if is_diameter { v * 2.0 } else { v });
        let value_expr = c
            .raw
            .get("valueExpr")
            .and_then(serde_json::Value::as_str)
            .filter(|s| !s.is_empty())
            .map(str::to_string);
        serde_json::json!({
            "value": display_value,
            "valueExpr": value_expr,
            "mode": mode,
        })
        .to_string()
    }

    /// Edit a dimensional constraint's value (S5), a port of the previous double-click edit:
    ///
    /// * A PLAIN NUMBER (`^-?\d*\.?\d+$`, optional exponent) → set `value` and REMOVE
    ///   `valueExpr` / `valueExprMode` (a literal dimension).
    /// * Otherwise → an EXPRESSION: set `valueExpr`, evaluate it LIVE against the
    ///   history's `expressions` + `configurator` (the kernel's `eval_expression`).
    ///   On eval success set `value`; on FAILURE keep the old value and return
    ///   `false` (never corrupt the doc). A diameter dim stores half the entered/
    ///   evaluated diameter as the solver radius and tags `valueExprMode:"diameter"`.
    ///
    /// Sets `valueNeedsSetup:false` once a real value lands, re-solves (swallowing
    /// errors), refreshes the overlay + marks dirty. Returns whether the value was
    /// applied. No-op returning `false` when the constraint is absent / not in
    /// sketch mode.
    pub fn sketch_set_dimension_value(
        &mut self,
        constraint_id: &serde_json::Value,
        input: &str,
    ) -> bool {
        use crate::sketch::doc::id_key;

        // Read the LIVE expression environment before borrowing the session mutably.
        let expressions = self.history.expressions();
        let configurator = self.history.configurator();

        let key = id_key(constraint_id);
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        // Locate the constraint + read its diameter mode.
        let is_diameter = {
            let Some(c) = edit
                .session
                .doc
                .constraints
                .iter()
                .find(|c| c.raw.get("id").map(id_key) == Some(key.clone()))
            else {
                return false;
            };
            c.ctype() == Some("⟺")
                && c.raw.get("displayStyle").and_then(serde_json::Value::as_str)
                    == Some("diameter")
        };

        let trimmed = input.trim();
        if trimmed.is_empty() {
            return false;
        }

        // Compute the DISPLAYED number the user typed/meant (a diameter for a
        // diameter dim), plus whether it came from an expression, without mutating.
        let is_plain_number = is_plain_number_literal(trimmed);
        let displayed: f64 = if is_plain_number {
            match trimmed.parse::<f64>() {
                Ok(n) if n.is_finite() => n,
                _ => return false,
            }
        } else {
            match brep_kernel::eval_expression(&expressions, &configurator, trimmed) {
                Ok(n) if n.is_finite() => n,
                // Bad expression: keep the old value, do not corrupt the doc.
                _ => return false,
            }
        };
        // The solver stores a radius for radial dims; a diameter input halves.
        let solver_value = if is_diameter { displayed * 0.5 } else { displayed };

        // Validation passed → the value WILL change; snapshot for undo now (S6a).
        edit.record_undo();

        // Apply to the constraint's raw map.
        {
            let Some(c) = edit
                .session
                .doc
                .constraints
                .iter_mut()
                .find(|c| c.raw.get("id").map(id_key) == Some(key.clone()))
            else {
                return false;
            };
            if is_plain_number {
                c.raw.remove("valueExpr");
                c.raw.remove("valueExprMode");
            } else {
                c.raw.insert(
                    "valueExpr".to_string(),
                    serde_json::Value::String(trimmed.to_string()),
                );
                if is_diameter {
                    c.raw.insert(
                        "valueExprMode".to_string(),
                        serde_json::Value::String("diameter".to_string()),
                    );
                } else {
                    c.raw.remove("valueExprMode");
                }
            }
            c.raw.insert(
                "value".to_string(),
                serde_json::Value::from(solver_value),
            );
            c.raw.insert(
                "valueNeedsSetup".to_string(),
                serde_json::Value::Bool(false),
            );
        }

        self.resolve_active_sketch("set-dimension-value");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// Drag a dimension label to CSS-pixel `(x, y)` (S5): map the pixel to plane
    /// `(u, v)` (the S2 pixel→plane math), compute the label offset `{du, dv}` =
    /// (label uv) − (the dimension's anchor uv), and store it in `session.dim_offsets`
    /// keyed by the constraint id. Refreshes the overlay (leaders + labels follow the
    /// cursor). No-op when not in sketch mode / the ray misses the plane / the
    /// constraint is not dimensional.
    pub fn sketch_dimension_drag_to(
        &mut self,
        constraint_id: &serde_json::Value,
        x: f64,
        y: f64,
    ) {
        use crate::sketch::doc::id_key;
        let Some((u, v)) = self.sketch_uv_at(x, y) else {
            return;
        };
        let key = id_key(constraint_id);
        let Some(edit) = self.sketch_edit.as_mut() else {
            return;
        };
        let anchor = {
            let Some(c) = edit
                .session
                .doc
                .constraints
                .iter()
                .find(|c| c.raw.get("id").map(id_key) == Some(key.clone()))
            else {
                return;
            };
            crate::sketch::dimensions::dimension_anchor_uv(c, &edit.session.doc)
        };
        let Some(anchor) = anchor else {
            return;
        };
        // Snapshot ONCE on the first move of this drag gesture so the whole drag is a
        // single undo step; the guard resets in `sketch_dimension_drag_end` (S6a).
        if !edit.dim_drag_snapshotted {
            edit.record_undo();
            edit.dim_drag_snapshotted = true;
        }
        edit.session.dim_offsets.insert(
            key,
            serde_json::json!({ "du": u - anchor[0], "dv": v - anchor[1] }),
        );
        self.refresh_sketch_overlay();
        self.dirty = true;
    }
}

/// Whether `s` is a plain numeric literal: an optional
/// sign, digits with an optional decimal (or a leading-dot decimal), and an optional
/// exponent. An expression (anything with a variable / operator) fails this and is
/// evaluated instead.
pub(super) fn is_plain_number_literal(s: &str) -> bool {
    let s = s.trim();
    if s.is_empty() {
        return false;
    }
    let bytes = s.as_bytes();
    let mut i = 0;
    if bytes[i] == b'+' || bytes[i] == b'-' {
        i += 1;
    }
    let mut digits_before = 0;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
        digits_before += 1;
    }
    let mut digits_after = 0;
    let mut had_dot = false;
    if i < bytes.len() && bytes[i] == b'.' {
        had_dot = true;
        i += 1;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
            digits_after += 1;
        }
    }
    // A dot must be followed by ≥1 digit (the pattern has no `\d+\.` form), and
    // the mantissa needs at least one digit overall.
    if had_dot && digits_after == 0 {
        return false;
    }
    if digits_before == 0 && digits_after == 0 {
        return false;
    }
    // Optional exponent `e[+-]?digits`.
    if i < bytes.len() && (bytes[i] == b'e' || bytes[i] == b'E') {
        i += 1;
        if i < bytes.len() && (bytes[i] == b'+' || bytes[i] == b'-') {
            i += 1;
        }
        let mut exp_digits = 0;
        while i < bytes.len() && bytes[i].is_ascii_digit() {
            i += 1;
            exp_digits += 1;
        }
        if exp_digits == 0 {
            return false;
        }
    }
    i == bytes.len()
}

/// The display mode string for a dimensional constraint (mirrors the label builder):
/// `"radius" | "diameter"` for a radial `⟺`, `"angle"` for `∠`, else `"distance"`.
fn dimension_mode(c: &crate::sketch::SketchConstraint) -> &'static str {
    match c.ctype() {
        Some("∠") => "angle",
        Some("⟺") => match c.raw.get("displayStyle").and_then(serde_json::Value::as_str) {
            Some("diameter") => "diameter",
            Some("radius") => "radius",
            _ => "distance",
        },
        _ => "distance",
    }
}

// ===========================================================================
// Sketch undo/redo (S6a) — a per-session history SEPARATE from the model-level
// undo. Each discrete mutating op snapshots the visible edit state (doc + dim
// offsets + selection) via `SketchEdit::record_undo` at its start; `sketch_undo`
// / `sketch_redo` walk those snapshots, swapping the current state onto the
// opposite stack. Continuous gestures (point drag, dimension-label drag) snapshot
// ONCE at the gesture start so one Ctrl+Z reverts the whole drag. Restores clear
// transient interaction state (pending draw chain, in-flight drag) that a doc swap
// could invalidate. Kept in ONE appended block so concurrent edits land clean.
// ===========================================================================
impl EngineState {
    /// Undo the last sketch edit (S6a) — pop the undo stack, stash the current state
    /// on redo, restore. Returns whether anything was undone. A no-op (false) when
    /// not in sketch mode or the stack is empty.
    pub fn sketch_undo(&mut self) -> bool {
        self.sketch_undo_redo(true)
    }

    /// Redo the last undone sketch edit (S6a) — the inverse of [`sketch_undo`].
    /// Returns whether anything was redone.
    pub fn sketch_redo(&mut self) -> bool {
        self.sketch_undo_redo(false)
    }

    /// Whether a sketch undo is available (drives the mode-bar Undo button).
    pub fn sketch_can_undo(&self) -> bool {
        self.sketch_edit
            .as_ref()
            .map_or(false, |edit| !edit.undo_stack.is_empty())
    }

    /// Whether a sketch redo is available (drives the mode-bar Redo button).
    pub fn sketch_can_redo(&self) -> bool {
        self.sketch_edit
            .as_ref()
            .map_or(false, |edit| !edit.redo_stack.is_empty())
    }

    /// End a dimension-label drag gesture (S6a): reset the first-move snapshot guard
    /// so the NEXT drag records its own single undo step. No-op when not in sketch
    /// mode. Called from the viewport when the label drag stops.
    pub fn sketch_dimension_drag_end(&mut self) {
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.dim_drag_snapshotted = false;
        }
    }

    /// Shared undo/redo core: move one snapshot between the undo and redo stacks
    /// (direction chosen by `undo`) and restore it into the live session, re-solving
    /// (errors swallowed + logged) and refreshing the overlay.
    fn sketch_undo_redo(&mut self, undo: bool) -> bool {
        let acted = {
            let Some(edit) = self.sketch_edit.as_mut() else {
                return false;
            };
            let popped = if undo {
                edit.undo_stack.pop()
            } else {
                edit.redo_stack.pop()
            };
            let Some(snapshot) = popped else {
                return false;
            };
            // Stash the CURRENT state on the opposite stack, then restore the pop.
            let current = edit.snapshot();
            if undo {
                edit.redo_stack.push(current);
            } else {
                edit.undo_stack.push(current);
            }
            edit.session.doc = snapshot.doc;
            edit.session.dim_offsets = snapshot.dim_offsets;
            edit.session.selection = snapshot.selection;
            edit.external_refs = snapshot.external_refs;
            // Drop transient interaction state a restore can invalidate (an in-flight
            // draw chain / drag / freehand stroke could reference points the snapshot no
            // longer holds).
            edit.pending.clear();
            edit.hover_uv = None;
            edit.drag = None;
            edit.handdraw_stroke.clear();
            edit.dim_drag_snapshotted = false;
            edit.session.hovered = None;
            self.resolve_active_sketch("undo/redo");
            true
        };
        if acted {
            self.refresh_sketch_overlay();
            self.dirty = true;
        }
        acted
    }

    /// A debug dump of a SKETCH feature's document + solved diagnostics (S6a — the
    /// `dumpSketchDiagnostics` button). Uses the LIVE session when that feature is
    /// being edited, else reads the persisted `sketch`/`basis` off the history and
    /// solves a throwaway session. Returns `{ error }` when the feature is absent /
    /// not a sketch / unparseable.
    pub fn sketch_diagnostics_dump_json(&self, feature_id: &str) -> String {
        if let Some(edit) = self.sketch_edit.as_ref() {
            if edit.feature_id == feature_id {
                return sketch_dump_value(feature_id, &edit.session).to_string();
            }
        }
        let Some(index) = self.history.index_of(feature_id) else {
            return serde_json::json!({ "error": format!("no feature '{feature_id}'") })
                .to_string();
        };
        if self.history.feature_type(index).as_deref() != Some("S") {
            return serde_json::json!({ "error": format!("'{feature_id}' is not a sketch") })
                .to_string();
        }
        let persistent = self.history.feature_persistent_data(index);
        let plane = persistent
            .as_ref()
            .and_then(|p| p.get("basis"))
            .map(crate::sketch::PlaneFrame::from_basis_json)
            .unwrap_or_else(crate::sketch::PlaneFrame::xy);
        let doc_value = persistent
            .as_ref()
            .and_then(|p| p.get("sketch"))
            .cloned()
            .unwrap_or_else(|| {
                serde_json::json!({ "points": [], "geometries": [], "constraints": [] })
            });
        let doc: crate::sketch::SketchDoc = match serde_json::from_value(doc_value) {
            Ok(doc) => doc,
            Err(error) => {
                return serde_json::json!({ "error": format!("sketch doc parse: {error}") })
                    .to_string();
            }
        };
        match crate::sketch::SketchSession::new(doc, plane) {
            Ok(session) => sketch_dump_value(feature_id, &session).to_string(),
            Err(error) => serde_json::json!({ "error": error }).to_string(),
        }
    }
}

/// Build the `dumpSketchDiagnostics` payload for one solved session: the sketch doc,
/// its solve diagnostics, the dimension offsets, and per-kind counts.
fn sketch_dump_value(
    feature_id: &str,
    session: &crate::sketch::SketchSession,
) -> serde_json::Value {
    serde_json::json!({
        "featureId": feature_id,
        "sketch": serde_json::to_value(&session.doc).unwrap_or(serde_json::Value::Null),
        "diagnostics": serde_json::to_value(&session.diagnostics)
            .unwrap_or(serde_json::Value::Null),
        "dimOffsets": serde_json::Value::Object(session.dim_offsets.clone()),
        "pointCount": session.doc.points.len(),
        "geometryCount": session.doc.geometries.len(),
        "constraintCount": session.doc.constraints.len(),
    })
}

// ===========================================================================
// Sketch trim tool (S6b) — sampled-polyline intersection split/delete.
//
// A CLICK tool that acts IMMEDIATELY on the geometry under the cursor (no `pending`
// buffer). Routed from the tool state machine (`sketch_tool_place_uv`'s `"trim"`
// arm) and directly callable headless via `sketch_trim_at`. Delegates the geometry
// math to `crate::sketch::trim` (a port of the previous trim-geometry family): sample
// the target, collect its intersections with every other geometry, bracket the
// click, split per type (line/circle/arc/bezier) or delete when unbounded. Owns its
// own undo snapshot — recorded only once a geometry is actually under the cursor,
// and popped when the trim changes nothing (no dead undo step). Kept in ONE appended
// block so concurrent edits to the primary impl land clean.
// ===========================================================================
impl EngineState {
    /// Trim the geometry under CSS-pixel `(x, y)` (S6b): map to plane uv (the S2
    /// pixel→plane math) and trim the geometry there. Returns whether the doc
    /// changed. A no-op returning `false` when not in sketch mode, the ray misses the
    /// plane, or no geometry is under the cursor.
    pub fn sketch_trim_at(&mut self, x: f64, y: f64) -> bool {
        let Some((u, v)) = self.sketch_uv_at(x, y) else {
            return false;
        };
        self.sketch_trim_uv(u, v)
    }

    /// The plane-space trim core (`sketch_trim_at` delegates here; so does the tool
    /// state machine, which already has uv). Picks the nearest geometry within the
    /// grab radius, snapshots undo, trims it (split or delete), re-solves + refreshes.
    /// Returns whether the doc changed; a no-op pops its own undo snapshot so it never
    /// leaves a dead step.
    pub fn sketch_trim_uv(&mut self, u: f64, v: f64) -> bool {
        let radius = self.sketch_pick_radius();
        // Old-trim parity: a trim click on a POINT deletes that point (and any
        // geometry/constraints depending on it), taking priority over curve trim —
        // `pick_entity` returns a point ref when one is under the cursor. Reuse the
        // full selection-delete cleanup (dependent geometry + orphan + constraint
        // pruning) by targeting just this point.
        let point_id = match self.sketch_edit.as_ref() {
            Some(edit) => edit.session.pick_entity(u, v, radius).and_then(|entity_ref| {
                (entity_ref.get("kind").and_then(|k| k.as_str()) == Some("point"))
                    .then(|| entity_ref.get("id").cloned())
                    .flatten()
            }),
            None => return false,
        };
        if let Some(point_id) = point_id {
            if let Some(edit) = self.sketch_edit.as_mut() {
                edit.session.clear_selection();
                edit.session
                    .toggle_selection(serde_json::json!({ "kind": "point", "id": point_id }));
            }
            return self.sketch_delete_selection();
        }

        // Otherwise a trim click targets a CURVE under the cursor.
        let geo_id = match self.sketch_edit.as_ref() {
            Some(edit) => crate::sketch::trim::pick_geometry_id(&edit.session.doc, u, v, radius),
            None => return false,
        };
        let Some(geo_id) = geo_id else {
            return false;
        };
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        // A geometry IS under the cursor → this click may mutate the doc; snapshot for
        // undo before it does (S6a). Popped below if the trim changed nothing.
        edit.record_undo();
        let changed = crate::sketch::trim::trim_geometry(&mut edit.session.doc, &geo_id, u, v);
        if !changed {
            edit.undo_stack.pop();
            return false;
        }
        // A trim clears the selection (the trimmed entity is gone) + hover, then
        // re-solves so coordinates + mobility stay fresh (keeping the doc on failure).
        edit.session.clear_selection();
        edit.session.set_hover(None);
        self.resolve_active_sketch("trim");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }
}

// ===========================================================================
// Sketch pickEdges tool (S6b-2) — link an external solid edge as a reference.
//
// A CLICK tool that acts on the 3D SCENE EDGE under the cursor (NOT a plane uv):
// the pixel is ranked through the modeling picker (`pick_json`), the top EDGE
// candidate's world polyline is fetched from the scene, projected into the active
// sketch plane, classified (straight `line` / fitted `circle`/`arc` / a faithful
// `line`-chain fallback) and materialized as external-reference geometry —
// `{fixed, construction, externalReference}` points, a `⏚` ground each, and the
// construction geometry — via `crate::sketch::external_ref`. A per-session
// `ExternalRef` mapping (keyed by edge name) dedups a re-pick and round-trips
// through `persistentData.externalRefs`. Kept in ONE appended block so concurrent
// edits to the primary impl land clean.
// ===========================================================================
impl EngineState {
    /// The name of the top EDGE candidate under CSS-pixel `(x, y)` (ranked by the
    /// same modeling picker), or `None` when nothing / no edge is there.
    fn sketch_top_edge_name(&self, x: f64, y: f64) -> Option<String> {
        let candidates: serde_json::Value = serde_json::from_str(&self.pick_json(x, y)).ok()?;
        candidates
            .as_array()?
            .iter()
            .find(|c| c.get("kind").and_then(|k| k.as_str()) == Some("EDGE"))
            .and_then(|c| c.get("name").and_then(|n| n.as_str()))
            .filter(|name| !name.is_empty())
            .map(str::to_string)
    }

    /// Link (as a construction reference) the 3D solid edge under CSS-pixel `(x, y)`
    /// into the active sketch (S6b-2). Ranks the pixel through the modeling picker,
    /// takes the top EDGE candidate, and delegates to [`sketch_link_edge`]. Returns
    /// whether a link was made / updated. A no-op returning `false` when not in sketch
    /// mode or no scene edge is under the cursor.
    pub fn sketch_pick_edge_at(&mut self, x: f64, y: f64) -> bool {
        if self.sketch_edit.is_none() {
            return false;
        }
        let Some(edge_name) = self.sketch_top_edge_name(x, y) else {
            return false;
        };
        self.sketch_link_edge(&edge_name)
    }

    /// Link a scene edge by kernel NAME into the active sketch as an external
    /// reference (the headless-testable core `sketch_pick_edge_at` delegates to).
    /// Fetches the edge's world polyline from the scene, projects it into the sketch
    /// plane, classifies + materializes (or updates) the reference, records one undo
    /// snapshot (popped on a no-op), re-solves, and refreshes. Returns whether the doc
    /// changed. A no-op returning `false` when not in sketch mode or the edge is
    /// absent / degenerate.
    pub fn sketch_link_edge(&mut self, edge_name: &str) -> bool {
        // Fetch the world polyline (+ owning solid) from the backdrop scene first —
        // an immutable borrow that must end before mutating the sketch edit.
        let Some(world_poly) = self.scene.edge_polyline_world(edge_name) else {
            return false;
        };
        let solid_name = self
            .scene
            .edge_solid_name(edge_name)
            .unwrap_or_default()
            .to_string();
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        // A scene edge IS under the cursor → this may mutate the doc; snapshot for undo
        // before it does (S6a). Popped below when the link changes nothing.
        edit.record_undo();
        let changed = crate::sketch::external_ref::link_or_update(
            &mut edit.session.doc,
            &mut edit.external_refs,
            edge_name,
            &solid_name,
            &world_poly,
            &edit.session.plane,
        );
        if !changed {
            edit.undo_stack.pop();
            return false;
        }
        self.resolve_active_sketch("pickEdges");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// Re-project every loaded external-reference edge against the CURRENT backdrop
    /// scene (called on enter): for each ref whose edge is still present, refresh its
    /// points' coordinates in place (via [`link_or_update`](crate::sketch::external_ref::link_or_update),
    /// which updates without duplicating). Best-effort — a ref whose edge is gone
    /// keeps its persisted coordinates as the fallback. Re-solves + refreshes when any
    /// ref moved. Returns whether anything changed. No undo snapshot (it is part of
    /// entering, not a user edit).
    pub(super) fn sketch_reproject_external_refs(&mut self) -> bool {
        // Gather the current world polylines for the loaded refs without holding a
        // borrow across the mutation.
        let jobs: Vec<(String, String, Vec<[f64; 3]>)> = match self.sketch_edit.as_ref() {
            Some(edit) if !edit.external_refs.is_empty() => edit
                .external_refs
                .iter()
                .filter_map(|r| {
                    let poly = self.scene.edge_polyline_world(&r.edge_name)?;
                    let solid = self
                        .scene
                        .edge_solid_name(&r.edge_name)
                        .unwrap_or_default()
                        .to_string();
                    Some((r.edge_name.clone(), solid, poly))
                })
                .collect(),
            _ => return false,
        };
        if jobs.is_empty() {
            return false;
        }
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        let mut changed = false;
        for (edge_name, solid_name, world_poly) in &jobs {
            changed |= crate::sketch::external_ref::link_or_update(
                &mut edit.session.doc,
                &mut edit.external_refs,
                edge_name,
                solid_name,
                world_poly,
                &edit.session.plane,
            );
        }
        if changed {
            self.resolve_active_sketch("external-ref reproject");
            self.refresh_sketch_overlay();
            self.dirty = true;
        }
        changed
    }

    /// The number of external-reference edge links in the active sketch (0 when not in
    /// sketch mode) — the `__brepSketch` verifier readout.
    pub fn sketch_external_ref_count(&self) -> usize {
        self.sketch_edit
            .as_ref()
            .map_or(0, |edit| edit.external_refs.len())
    }
}

// ===========================================================================
// Sketch handdraw tool (S6b-3) — freehand stroke → recognized primitive.
//
// A DRAG tool: `sketch_handdraw_begin` snapshots undo + starts a stroke, each
// `sketch_handdraw_move` appends a plane-`(u, v)` sample (throttled to ~2px world so
// dense pointer events don't bloat the stroke), and `sketch_handdraw_end` recognizes
// the stroke into ONE shape (line / circle / arc / bezier fallback via
// `crate::sketch::handdraw`), materializes it (endpoints snap to existing points so a
// stroke drawn onto prior geometry coincides), and re-solves. A too-short / tiny
// stroke is discarded and pops its own undo snapshot so no dead step is left. The raw
// stroke renders live through the `sketch-preview` overlay group. The pixel entry
// points map the cursor to plane uv (the S2 pixel→plane math) and delegate to the
// headless-testable uv cores. Kept in ONE appended block so concurrent edits to the
// primary impl land clean.
// ===========================================================================
impl EngineState {
    /// Begin a freehand stroke at CSS-pixel `(x, y)` (a handdraw drag start): map to
    /// plane uv and delegate. No-op when not in sketch mode or the ray misses the plane.
    pub fn sketch_handdraw_begin(&mut self, x: f64, y: f64) {
        if let Some((u, v)) = self.sketch_uv_at(x, y) {
            self.sketch_handdraw_begin_uv(u, v);
        }
    }

    /// Extend the freehand stroke toward CSS-pixel `(x, y)` (a handdraw drag move).
    pub fn sketch_handdraw_move(&mut self, x: f64, y: f64) {
        if let Some((u, v)) = self.sketch_uv_at(x, y) {
            self.sketch_handdraw_move_uv(u, v);
        }
    }

    /// Begin a freehand stroke at plane `(u, v)` (the headless-testable core): snapshot
    /// for undo (S6a), clear any prior stroke, and seed it with the start sample. The
    /// undo snapshot is popped in [`sketch_handdraw_end`] if the stroke produces nothing.
    /// No-op when not in sketch mode.
    pub fn sketch_handdraw_begin_uv(&mut self, u: f64, v: f64) {
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.record_undo();
            edit.handdraw_stroke.clear();
            edit.handdraw_stroke.push((u, v));
        } else {
            return;
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// Append plane `(u, v)` to the live stroke (the headless-testable core), throttled
    /// so a sample nearer than ~2px world to the last is skipped. No-op when not in
    /// sketch mode or no stroke is live.
    pub fn sketch_handdraw_move_uv(&mut self, u: f64, v: f64) {
        let min_step = 2.0 * self.camera.world_per_pixel();
        if let Some(edit) = self.sketch_edit.as_mut() {
            let Some(&last) = edit.handdraw_stroke.last() else {
                return; // no stroke in progress
            };
            if (u - last.0).hypot(v - last.1) < min_step {
                return;
            }
            edit.handdraw_stroke.push((u, v));
        } else {
            return;
        }
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// End the freehand stroke (a handdraw drag stop): recognize it into one shape and
    /// materialize the geometry, re-solve, and clear the stroke + preview. A stroke that
    /// is too short (fewer than 3 samples) or too tiny (extent below the grab radius) is
    /// discarded, popping the undo snapshot recorded at begin so no dead step remains.
    /// Returns whether geometry was created. No-op returning `false` when not in sketch
    /// mode / no stroke is live.
    pub fn sketch_handdraw_end(&mut self) -> bool {
        let radius = self.sketch_pick_radius();
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        if edit.handdraw_stroke.is_empty() {
            return false; // no live stroke (a plain click, or already ended)
        }
        let stroke = std::mem::take(&mut edit.handdraw_stroke);
        // Discard a stroke too short / tiny to be a deliberate shape, undoing the
        // snapshot the begin recorded so it never leaves a dead undo step.
        if stroke.len() < 3 || crate::sketch::handdraw::stroke_extent(&stroke) < radius {
            edit.undo_stack.pop();
            self.refresh_sketch_overlay();
            self.dirty = true;
            return false;
        }
        let shape = crate::sketch::handdraw::recognize(&stroke);
        crate::sketch::handdraw::emit_shape(&mut edit.session.doc, &shape, radius);
        self.resolve_active_sketch("handdraw");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// The number of samples in the live handdraw stroke (0 when none / not in sketch
    /// mode) — the `__brepSketch` verifier readout.
    pub fn sketch_handdraw_len(&self) -> usize {
        self.sketch_edit
            .as_ref()
            .map_or(0, |edit| edit.handdraw_stroke.len())
    }
}

