use super::*;

/// One row in a sketch entity-LIST panel (Points / Curves / Constraints) — the
/// display label plus the `(kind, id)` needed to select / hover / delete it, and
/// its current selected/construction state for row styling. Built by
/// [`EngineState::sketch_point_rows`] / `sketch_geometry_rows` /
/// `sketch_constraint_rows`; the panel renders them and routes clicks back through
/// [`EngineState::sketch_select_entity`] / `sketch_hover_entity`.
#[derive(Clone, Debug, PartialEq)]
pub struct SketchEntityRow {
    /// Entity ref kind: `"point"` | `"geometry"` | `"constraint"`.
    pub kind: &'static str,
    /// The entity id (opaque `Value`, passed straight back to select/hover).
    pub id: serde_json::Value,
    /// Human display label (mirrors the previous list rows).
    pub label: String,
    /// Whether this entity is in the current sketch selection (row highlight).
    pub selected: bool,
    /// Construction-only entity (dashed / de-emphasized styling).
    pub construction: bool,
    /// The solver names this entity in a CONSTRAINT CONFLICT (constraint rows
    /// only) — the panel paints the row red to match its canvas annotation.
    pub conflicting: bool,
}

/// How a notice is drawn: an [`Error`](Self::Error) (a refusal, a failure —
/// what [`EngineState::push_notice`] queues), a [`Warning`](Self::Warning)
/// (it worked, and something needs a look) or [`Info`](Self::Info) (it worked:
/// "Updated 2 part(s)", "Saved 'widget'").
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NoticeSeverity {
    #[default]
    Error,
    Warning,
    Info,
}

impl EngineState {
    /// Queue a notice of `severity`. [`Self::push_notice`] is this with
    /// [`NoticeSeverity::Error`]; bounded the same way.
    pub fn push_notice_as(&mut self, severity: NoticeSeverity, message: impl Into<String>) {
        if severity == NoticeSeverity::Error {
            return self.push_notice(message);
        }
        let message = message.into();
        #[cfg(not(target_arch = "wasm32"))]
        eprintln!("{message}");
        self.graded_notices.push((severity, message));
        if self.graded_notices.len() > 8 {
            let overflow = self.graded_notices.len() - 8;
            self.graded_notices.drain(0..overflow);
        }
    }

    /// Drain every queued notice with its severity: the errors, then the
    /// warnings and successes. The shell's toast overlay reads this.
    pub fn take_graded_notices(&mut self) -> Vec<(NoticeSeverity, String)> {
        let errors = std::mem::take(&mut self.notices).into_iter().map(|text| (NoticeSeverity::Error, text));
        errors.chain(std::mem::take(&mut self.graded_notices)).collect()
    }

    /// Queue a transient user-facing notice (shown as a toast by the shell) and,
    /// on native, also log it. Bounded so a pathological loop can't grow it.
    pub fn push_notice(&mut self, message: impl Into<String>) {
        let message = message.into();
        #[cfg(not(target_arch = "wasm32"))]
        eprintln!("{message}");
        self.notices.push(message);
        if self.notices.len() > 8 {
            let overflow = self.notices.len() - 8;
            self.notices.drain(0..overflow);
        }
    }

    /// Drain the queued notices (the shell calls this once per frame and shows
    /// each as a toast).
    pub fn take_notices(&mut self) -> Vec<String> {
        self.take_graded_notices().into_iter().map(|(_, text)| text).collect()
    }

    /// Re-solve the ACTIVE sketch session and, on failure, queue a user notice
    /// naming `context`. Replaces the scattered swallowed `eprintln!` on the
    /// interactive re-solve paths — the solver rarely fails, but when an edit
    /// leaves the sketch unsolvable the user should see why.
    pub(super) fn resolve_active_sketch(&mut self, context: &str) {
        let error = self
            .sketch_edit
            .as_mut()
            .and_then(|edit| edit.session.resolve().err());
        if let Some(error) = error {
            self.push_notice(format!("Sketch solve failed ({context}): {error}"));
        }
    }

    /// Whether `(kind, id)` is in the current sketch selection.
    fn sketch_ref_selected(session: &crate::sketch::SketchSession, kind: &str, id: &serde_json::Value) -> bool {
        use crate::sketch::doc::id_key;
        let key = id_key(id);
        session.selection.iter().any(|r| {
            r.get("kind").and_then(serde_json::Value::as_str) == Some(kind)
                && r.get("id").map(id_key).as_deref() == Some(key.as_str())
        })
    }

    /// The Points list: `P{id} (x, y)` plus ⛓ external / ◐ construction / ⏚ ground
    /// markers, mirroring the previous Points rows.
    pub fn sketch_point_rows(&self) -> Vec<SketchEntityRow> {
        use crate::sketch::doc::id_key;
        let Some(edit) = self.sketch_edit.as_ref() else {
            return Vec::new();
        };
        let session = &edit.session;
        session
            .doc
            .points
            .iter()
            .map(|p| {
                let grounded = session.doc.constraints.iter().any(|c| {
                    c.ctype() == Some("⏚")
                        && c.points().first().map(id_key).as_deref() == Some(id_key(&p.id).as_str())
                });
                let mut marks = String::new();
                if p.external_reference {
                    marks.push_str(" \u{26D3}"); // ⛓ chain-links (bundled font; 🔗 is not)
                }
                if p.construction {
                    marks.push_str(" ◐");
                }
                if grounded {
                    marks.push_str(" ⏚");
                }
                SketchEntityRow {
                    kind: "point",
                    id: p.id.clone(),
                    label: if crate::sketch::external_ref::is_origin_id(&p.id) {
                        format!("Origin ({:.1}, {:.1}){marks}", p.x, p.y)
                    } else {
                        format!("P{} ({:.1}, {:.1}){marks}", id_key(&p.id), p.x, p.y)
                    },
                    selected: Self::sketch_ref_selected(session, "point", &p.id),
                    construction: p.construction,
                    conflicting: false,
                }
            })
            .collect()
    }

    /// The Curves list: `{type}:{id} [p0,p1,…]` (◐ for construction), mirroring the
    /// previous Curves rows.
    pub fn sketch_geometry_rows(&self) -> Vec<SketchEntityRow> {
        use crate::sketch::doc::id_key;
        let Some(edit) = self.sketch_edit.as_ref() else {
            return Vec::new();
        };
        let session = &edit.session;
        session
            .doc
            .geometries
            .iter()
            .map(|g| {
                let pts = g
                    .points
                    .iter()
                    .map(id_key)
                    .collect::<Vec<_>>()
                    .join(",");
                let construction = g.construction();
                let mark = if construction { " ◐" } else { "" };
                SketchEntityRow {
                    kind: "geometry",
                    id: g.id.clone(),
                    label: format!("{}:{}{mark} [{pts}]", g.geom_type, id_key(&g.id)),
                    selected: Self::sketch_ref_selected(session, "geometry", &g.id),
                    construction,
                    conflicting: false,
                }
            })
            .collect()
    }

    /// The Constraints list: `{id} {type} {value} [points]`, mirroring the previous
    /// Constraints rows.
    pub fn sketch_constraint_rows(&self) -> Vec<SketchEntityRow> {
        use crate::sketch::doc::id_key;
        let Some(edit) = self.sketch_edit.as_ref() else {
            return Vec::new();
        };
        let session = &edit.session;
        session
            .doc
            .constraints
            .iter()
            .filter_map(|c| {
                let id = c.raw.get("id")?.clone();
                let ctype = c.ctype().unwrap_or("?");
                let value = c
                    .raw
                    .get("value")
                    .and_then(serde_json::Value::as_f64)
                    .map(|v| format!(" {v:.3}"))
                    .unwrap_or_default();
                let pts = c
                    .points()
                    .iter()
                    .map(id_key)
                    .collect::<Vec<_>>()
                    .join(",");
                Some(SketchEntityRow {
                    kind: "constraint",
                    id: id.clone(),
                    label: format!("{} {ctype}{value} [{pts}]", id_key(&id)),
                    selected: Self::sketch_ref_selected(session, "constraint", &id),
                    construction: false,
                    conflicting: session.diagnostics.constraint_conflicting(&id),
                })
            })
            .collect()
    }

    /// Select an entity BY ref from a list row (mirrors [`sketch_click_at`]): honors the
    /// SAME "Multi-select" setting — under `ClickToggles` a plain click toggles the row
    /// (no modifier needed); under `CtrlClick` a plain click replaces the
    /// selection and an additive (Ctrl/Cmd) click toggles.
    pub fn sketch_select_entity(&mut self, kind: &str, id: serde_json::Value, additive: bool) {
        let toggles = self.settings.multi_select == crate::style::MultiSelectMode::ClickToggles;
        let entity_ref = serde_json::json!({ "kind": kind, "id": id });
        let Some(edit) = self.sketch_edit.as_mut() else {
            return;
        };
        if !additive && !toggles {
            edit.session.clear_selection();
        }
        edit.session.toggle_selection(entity_ref);
        self.refresh_sketch_overlay();
        self.dirty = true;
    }

    /// Hover an entity BY ref from a list row (list→canvas highlight). Sets the
    /// one-frame guard so the viewport — which draws after the panel and would
    /// otherwise clear the hover because the pointer is off the viewport — keeps it.
    pub fn sketch_hover_entity(&mut self, kind: &str, id: serde_json::Value) {
        if self.sketch_edit.is_none() {
            return;
        }
        // `set_sketch_hover` re-tessellates only when the hovered ref actually
        // changes, so holding the pointer on one row does not churn the overlay.
        let entity_ref = serde_json::json!({ "kind": kind, "id": id });
        self.set_sketch_hover(Some(entity_ref));
        self.sketch_list_hover_active = true;
    }

    /// Consume the "list panel set the hover this frame" guard (read + reset). The
    /// viewport calls this before its off-viewport `sketch_clear_hover` so a
    /// panel-set hover survives the frame.
    pub fn take_sketch_list_hover(&mut self) -> bool {
        std::mem::take(&mut self.sketch_list_hover_active)
    }

    /// The active sketch's solver settings (the Solver Settings panel reads this).
    /// `None` when not editing a sketch.
    pub fn sketch_solver_settings(&self) -> Option<crate::sketch::SketchSolverSettings> {
        self.sketch_edit
            .as_ref()
            .map(|edit| edit.session.solver_settings.clone())
    }

    /// Replace the active sketch's solver settings and re-solve so the change
    /// takes effect immediately. No-op when not editing a sketch.
    pub fn sketch_set_solver_settings(&mut self, settings: crate::sketch::SketchSolverSettings) {
        if let Some(edit) = self.sketch_edit.as_mut() {
            edit.session.solver_settings = settings;
        } else {
            return;
        }
        self.resolve_active_sketch("solver settings");
        self.refresh_sketch_overlay();
        self.dirty = true;
    }
}

/// One applicable-constraint palette entry: the glyph passed back to
/// [`EngineState::sketch_add_constraint`] plus a human tooltip and a dimensional
/// tag (whether it opens a value the solver seeds from the current measurement).
#[derive(Clone, Debug, PartialEq)]
pub struct SketchConstraintAction {
    /// The constraint glyph (e.g. `"━"`, `"⟂"`, `"R"`) — the argument to
    /// [`EngineState::sketch_add_constraint`].
    pub symbol: String,
    /// A human tooltip label (e.g. `"Horizontal"`, `"Radius"`).
    pub label: String,
    /// Whether this is a dimensional constraint (its value is seeded from the
    /// current measurement; S5 makes it editable).
    pub dimensional: bool,
}

impl EngineState {
    /// The ordered palette of constraints applicable to the active sketch selection
    /// (a faithful port of `#refreshContextBar`). Empty when not in sketch mode or
    /// the selection surfaces no constraint. The Fix/Unfix + construction + cleanup
    /// affordances are exposed separately (see [`sketch_selection_all_grounded`] /
    /// [`sketch_selection_all_construction`] / [`sketch_cleanup_unused_points`]).
    ///
    /// [`sketch_selection_all_grounded`]: Self::sketch_selection_all_grounded
    /// [`sketch_selection_all_construction`]: Self::sketch_selection_all_construction
    /// [`sketch_cleanup_unused_points`]: Self::sketch_cleanup_unused_points
    pub fn sketch_applicable_constraints(&self) -> Vec<SketchConstraintAction> {
        match self.sketch_edit.as_ref() {
            Some(edit) => sketch_palette_for(&edit.session),
            None => Vec::new(),
        }
    }

    /// Add the constraint named by `symbol` from the current selection (a port of
    /// `createConstraint`): build the ordered point-id list, dedup on
    /// `type + sorted-points`, append (dimensional → `value:null`), then re-solve +
    /// refresh (keeping the selection). Returns whether a constraint was added.
    pub fn sketch_add_constraint(&mut self, symbol: &str) -> bool {
        // Snapshot BEFORE the (in-place) add; discard it below when the add is a
        // dedup no-op so a dead click neither pollutes undo nor clobbers redo (S6a).
        let added = match self.sketch_edit.as_mut() {
            Some(edit) => {
                edit.record_undo();
                sketch_build_and_add_constraint(&mut edit.session, symbol)
            }
            None => return false,
        };
        if added {
            self.resolve_active_sketch("add-constraint");
            self.refresh_sketch_overlay();
            self.dirty = true;
        } else if let Some(edit) = self.sketch_edit.as_mut() {
            edit.undo_stack.pop();
        }
        added
    }

    /// Toggle the ground (`⏚`) constraint on the selected points: if ALL selected
    /// points are already grounded → remove those grounds (and clear their `fixed`
    /// flag); else add a `⏚` for each ungrounded selected point (and set `fixed`).
    /// Re-solves + refreshes. Returns whether anything changed. No-op with no point
    /// selected / not in sketch mode.
    pub fn sketch_toggle_ground(&mut self) -> bool {
        use crate::sketch::doc::id_key;
        use std::collections::HashSet;

        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        let sel_ids: Vec<serde_json::Value> = edit
            .session
            .selection
            .iter()
            .filter(|r| r.get("kind").and_then(|v| v.as_str()) == Some("point"))
            .filter_map(|r| r.get("id").cloned())
            // The part-origin reference stays grounded.
            .filter(|id| !crate::sketch::external_ref::is_origin_id(id))
            .collect();
        if sel_ids.is_empty() {
            return false;
        }
        // Selected points → the toggle always adds or removes grounds; snapshot (S6a).
        edit.record_undo();
        let doc = &mut edit.session.doc;
        let has_ground = |doc: &crate::sketch::SketchDoc, id: &serde_json::Value| -> bool {
            doc.constraints.iter().any(|c| {
                c.ctype() == Some("⏚")
                    && c.points().first().map(id_key) == Some(id_key(id))
            })
        };
        let all_grounded = sel_ids.iter().all(|id| has_ground(doc, id));
        if all_grounded {
            let sel_keys: HashSet<String> = sel_ids.iter().map(id_key).collect();
            doc.constraints.retain(|c| {
                if c.ctype() != Some("⏚") {
                    return true;
                }
                match c.points().first() {
                    Some(p) => !sel_keys.contains(&id_key(p)),
                    None => true,
                }
            });
            for id in &sel_ids {
                if let Some(p) = doc.point_mut(id) {
                    p.fixed = false;
                }
            }
        } else {
            for id in &sel_ids {
                if has_ground(doc, id) {
                    continue;
                }
                let cid = doc.next_constraint_id();
                let mut raw = serde_json::Map::new();
                raw.insert("id".to_string(), cid);
                raw.insert("type".to_string(), serde_json::Value::String("⏚".to_string()));
                raw.insert(
                    "points".to_string(),
                    serde_json::Value::Array(vec![id.clone()]),
                );
                doc.constraints.push(crate::sketch::SketchConstraint { raw });
                if let Some(p) = doc.point_mut(id) {
                    p.fixed = true;
                }
            }
        }
        self.resolve_active_sketch("toggle-ground");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// Flip the `construction` flag on ALL selected points AND geometries: if every
    /// selected entity is already construction → make them regular, else make them
    /// all construction. Re-solves + refreshes. Returns whether anything was
    /// selected. No-op with nothing selected / not in sketch mode.
    pub fn sketch_toggle_construction(&mut self) -> bool {
        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        let mut pt_ids: Vec<serde_json::Value> = Vec::new();
        let mut geo_ids: Vec<serde_json::Value> = Vec::new();
        for r in &edit.session.selection {
            match r.get("kind").and_then(|v| v.as_str()) {
                Some("point") => {
                    // The part-origin reference stays construction.
                    if let Some(id) = r.get("id").filter(|id| !crate::sketch::external_ref::is_origin_id(id)) {
                        pt_ids.push(id.clone());
                    }
                }
                Some("geometry") => {
                    if let Some(id) = r.get("id") {
                        geo_ids.push(id.clone());
                    }
                }
                _ => {}
            }
        }
        if pt_ids.is_empty() && geo_ids.is_empty() {
            return false;
        }
        // Selected entities → the flip always changes something; snapshot (S6a).
        edit.record_undo();
        let doc = &mut edit.session.doc;
        let all_construction = pt_ids
            .iter()
            .all(|id| doc.point(id).map_or(false, |p| p.construction))
            && geo_ids
                .iter()
                .all(|id| doc.geometry(id).map_or(false, |g| g.construction()));
        let next = !all_construction;
        for id in &pt_ids {
            if let Some(p) = doc.point_mut(id) {
                p.construction = next;
            }
        }
        for id in &geo_ids {
            if let Some(g) = doc.geometry_mut(id) {
                g.extra
                    .insert("construction".to_string(), serde_json::Value::Bool(next));
            }
        }
        self.resolve_active_sketch("toggle-construction");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// Remove points referenced by NO geometry AND no constraint (the 🧹 action).
    /// Re-solves + refreshes only when something was dropped. Returns whether a
    /// point was removed. No-op when not in sketch mode.
    pub fn sketch_cleanup_unused_points(&mut self) -> bool {
        use crate::sketch::doc::id_key;
        use std::collections::HashSet;

        let Some(edit) = self.sketch_edit.as_mut() else {
            return false;
        };
        // Snapshot before the cleanup; discarded below when nothing is removed (S6a).
        edit.record_undo();
        let doc = &mut edit.session.doc;
        let mut used: HashSet<String> = HashSet::new();
        for g in &doc.geometries {
            for pid in &g.points {
                used.insert(id_key(pid));
            }
        }
        for c in &doc.constraints {
            for pid in c.points() {
                used.insert(id_key(pid));
            }
        }
        let before = doc.points.len();
        doc.points.retain(|p| used.contains(&id_key(&p.id)));
        let removed = doc.points.len() != before;
        if !removed {
            edit.undo_stack.pop();
            return false;
        }
        self.resolve_active_sketch("cleanup");
        self.refresh_sketch_overlay();
        self.dirty = true;
        true
    }

    /// The number of constraints in the active sketch (0 when not in sketch mode) —
    /// the verifier / palette readout.
    pub fn sketch_constraint_count(&self) -> usize {
        self.sketch_edit
            .as_ref()
            .map_or(0, |edit| edit.session.doc.constraints.len())
    }

    /// Whether the selected points are ALL grounded (`Some(true)`), NOT all grounded
    /// (`Some(false)`), or no point is selected (`None`) — labels the Fix vs Unfix
    /// button.
    pub fn sketch_selection_all_grounded(&self) -> Option<bool> {
        use crate::sketch::doc::id_key;
        let edit = self.sketch_edit.as_ref()?;
        let doc = &edit.session.doc;
        let sel: Vec<&serde_json::Value> = edit
            .session
            .selection
            .iter()
            .filter(|r| r.get("kind").and_then(|v| v.as_str()) == Some("point"))
            .filter_map(|r| r.get("id"))
            .collect();
        if sel.is_empty() {
            return None;
        }
        let all = sel.iter().all(|id| {
            doc.constraints.iter().any(|c| {
                c.ctype() == Some("⏚")
                    && c.points().first().map(id_key) == Some(id_key(id))
            })
        });
        Some(all)
    }

    /// Whether the selected points + geometries are ALL construction (`Some(true)`),
    /// NOT all construction (`Some(false)`), or nothing is selected (`None`) — labels
    /// the ◐ construction-toggle button's direction.
    pub fn sketch_selection_all_construction(&self) -> Option<bool> {
        let edit = self.sketch_edit.as_ref()?;
        let doc = &edit.session.doc;
        let mut any = false;
        let mut all = true;
        for r in &edit.session.selection {
            match r.get("kind").and_then(|v| v.as_str()) {
                Some("point") => {
                    any = true;
                    if let Some(id) = r.get("id") {
                        if !doc.point(id).map_or(false, |p| p.construction) {
                            all = false;
                        }
                    }
                }
                Some("geometry") => {
                    any = true;
                    if let Some(id) = r.get("id") {
                        if !doc.geometry(id).map_or(false, |g| g.construction()) {
                            all = false;
                        }
                    }
                }
                _ => {}
            }
        }
        if !any {
            return None;
        }
        Some(all)
    }
}

/// Build the applicable-constraint palette for a session's selection — the port of
/// `SketchMode3D.#refreshContextBar`'s branch logic (constraint buttons only; the
/// state toggles + cleanup + delete are returned via companion accessors).
fn sketch_palette_for(session: &crate::sketch::SketchSession) -> Vec<SketchConstraintAction> {
    use crate::sketch::doc::id_key;
    use std::collections::HashSet;

    let doc = &session.doc;
    let mut sel_points: Vec<serde_json::Value> = Vec::new();
    let mut geos: Vec<&crate::sketch::SketchGeometry> = Vec::new();
    for r in &session.selection {
        match r.get("kind").and_then(|v| v.as_str()) {
            Some("point") => {
                if let Some(id) = r.get("id") {
                    sel_points.push(id.clone());
                }
            }
            Some("geometry") => {
                if let Some(id) = r.get("id") {
                    if let Some(g) = doc.geometry(id) {
                        geos.push(g);
                    }
                }
            }
            _ => {}
        }
    }

    // point-coverage = selected point ids ∪ endpoints of selected geometries
    // (arc: only points[0..2] — center + start).
    let mut point_set: HashSet<String> = HashSet::new();
    for id in &sel_points {
        point_set.insert(id_key(id));
    }
    for g in &geos {
        let pts: &[serde_json::Value] = if g.geom_type == "arc" {
            &g.points[..g.points.len().min(2)]
        } else {
            &g.points
        };
        for pid in pts {
            point_set.insert(id_key(pid));
        }
    }
    let point_count = point_set.len();
    let selected_point_ids_len = sel_points.len();
    let is_radial =
        |g: &crate::sketch::SketchGeometry| g.geom_type == "arc" || g.geom_type == "circle";

    let mut out: Vec<SketchConstraintAction> = Vec::new();
    let mut push = |symbol: &str, label: &str, dimensional: bool| {
        out.push(SketchConstraintAction {
            symbol: symbol.to_string(),
            label: label.to_string(),
            dimensional,
        });
    };

    // 1 arc/circle → radial dims (defer the leader to S5).
    if geos.len() == 1 && is_radial(geos[0]) {
        push("R", "Radius", true);
        push("⌀", "Diameter", true);
        return out;
    }
    // 2 lines → parallel / perp / angle / equal-length / collinear / point-on-line,
    // plus tangent when either line is a spline HANDLE GUIDE. A guide IS a line, so
    // every offer above stays legitimate on one (its two ends are still two points
    // whose direction can be made parallel to another line's); tangency is the extra
    // question only a guide can be asked, and it is the reason a guide gets selected
    // at all. Both new pairs — guide + line and guide + guide — are pairs the
    // solver's `c_tangent` already reads as a spline tangent, at a drawn end and at
    // an inserted interior anchor alike.
    if geos.len() == 2 && geos.iter().all(|g| g.geom_type == "line") {
        push("∥", "Parallel", false);
        push("⟂", "Perpendicular", false);
        push("∠", "Angle", true);
        push("⇌", "Equal distance", false);
        push("⋰", "Collinear", false);
        push("⏛", "Point on line", false);
        if geos
            .iter()
            .any(|g| crate::sketch::spline::is_handle_guide(doc, g))
        {
            push("⌒", "Tangent", false);
        }
        // Curvature (G2) is the narrower question: the two guides have to be the
        // two sides of ONE joint — either side of one interior anchor, or two
        // ends the document already holds together. A guide paired with a plain
        // line is never one (a line has no curvature to match), and neither are
        // two guides at unrelated anchors.
        if crate::sketch::spline::is_curvature_joint(doc, geos[0], geos[1]) {
            push("ϰ", "Curvature (G2)", false);
        }
        return out;
    }
    // 2 arcs/circles → equal-radius / concentric / tangent.
    if geos.len() == 2 && geos.iter().all(|g| is_radial(g)) {
        push("⊜", "Equal radius", false);
        push("◎", "Concentric", false);
        push("⌒", "Tangent", false);
        return out;
    }
    // line + arc/circle → tangent, plus curvature when the line is a spline
    // handle guide at an END anchor: the spline's curvature there becomes the
    // circle's 1/ρ, which is how a curve is continued into an arc.
    if geos.len() == 2
        && ((geos[0].geom_type == "line" && is_radial(geos[1]))
            || (geos[1].geom_type == "line" && is_radial(geos[0])))
    {
        push("⌒", "Tangent", false);
        if geos
            .iter()
            .any(|g| crate::sketch::spline::is_end_handle_guide(doc, g))
        {
            push("ϰ", "Curvature (G2)", false);
        }
        return out;
    }
    // A spline BODY + a line, or a spline body + an arc/circle → tangency at a
    // point INTERIOR to one of the spline's spans (`∿`).
    //
    // The PICK is the whole difference from `⌒`. Selecting an anchor's handle
    // guide asks for tangency AT that anchor, where the spline's tangent has a
    // name (anchor→handle) and no touch parameter is needed — that stays `⌒`.
    // Selecting the curve ITSELF asks for tangency wherever the two geometries
    // actually touch, which is the foot-parameter lane and a different
    // constraint type. So a guide is refused here, and a body is refused there.
    if sketch_foot_tangent_pair(doc, &geos).is_some() {
        push("∿", "Tangent (span point)", false);
        return out;
    }
    // 1 line + 2 selected points → perpendicular / symmetric-about-line (no return).
    if geos.len() == 1 && geos[0].geom_type == "line" && selected_point_ids_len == 2 {
        push("⟂", "Perpendicular", false);
        push("⋈", "Symmetric about line", false);
    }

    if point_count == 1 {
        push("⏚", "Ground (fix point)", false);
    }
    if point_count == 2 {
        push("━", "Horizontal", false);
        push("│", "Vertical", false);
        push("≡", "Coincident", false);
        push("⟺", "Distance", true);
    }
    if point_count == 3 {
        push("⋯", "Midpoint", false);
        push("⏛", "Point on line", false);
        push("↥", "Line to point distance", true);
        push("∠", "Angle", true);
        // Collinear needs 3+ ACTUAL selected points (not derived from geometry).
        if selected_point_ids_len >= 3 {
            push("⋰", "Collinear", false);
        }
    }
    // 4+ free-standing points → still offer Collinear.
    if point_count > 3 && geos.is_empty() && selected_point_ids_len >= 3 {
        push("⋰", "Collinear", false);
    }

    out
}

/// The point-id list for a tangent (`⌒`) constraint
/// from two selected geometries — `[lineA, lineB, center, boundary]` for line+circle,
/// `[c1, boundary1, c2, boundary2]` for circle+circle, and `[a0, a1, b0, b1]` for two
/// lines of which at least one is a spline handle guide. `None` for any other pair.
///
/// The guide gate on the two-line case is not decoration: the solver's `c_tangent`
/// falls through to `c_tangent_line_circle` for a pair it cannot read as a spline,
/// which would take the second line's two ends for a circle's centre and rim. A pair
/// of plain lines has no tangency to state, so it is refused here rather than
/// silently stored as a nonsense circle.
fn sketch_build_tangent_points(
    doc: &crate::sketch::SketchDoc,
    geos: &[&crate::sketch::SketchGeometry],
) -> Option<Vec<serde_json::Value>> {
    if geos.len() != 2 {
        return None;
    }
    let is_radial = |g: &crate::sketch::SketchGeometry| g.geom_type == "arc" || g.geom_type == "circle";
    if geos.iter().all(|g| g.geom_type == "line") {
        let (a, b) = (geos[0], geos[1]);
        if a.points.len() < 2 || b.points.len() < 2 {
            return None;
        }
        if !crate::sketch::spline::is_handle_guide(doc, a)
            && !crate::sketch::spline::is_handle_guide(doc, b)
        {
            return None;
        }
        return Some(vec![
            a.points[0].clone(),
            a.points[1].clone(),
            b.points[0].clone(),
            b.points[1].clone(),
        ]);
    }
    let line = geos.iter().find(|g| g.geom_type == "line");
    let circ = geos.iter().find(|g| is_radial(g));
    if let (Some(line), Some(circ)) = (line, circ) {
        if line.points.len() < 2 || circ.points.len() < 2 {
            return None;
        }
        return Some(vec![
            line.points[0].clone(),
            line.points[1].clone(),
            circ.points[0].clone(),
            circ.points[1].clone(),
        ]);
    }
    if geos.iter().all(|g| is_radial(g)) {
        let (a, b) = (geos[0], geos[1]);
        if a.points.len() < 2 || b.points.len() < 2 {
            return None;
        }
        return Some(vec![
            a.points[0].clone(),
            a.points[1].clone(),
            b.points[0].clone(),
            b.points[1].clone(),
        ]);
    }
    None
}

/// The `(spline body, line-or-circle)` pair a foot-parameter tangency (`∿`) is
/// offered for, or `None` for any other selection.
///
/// ONE gate, read by both the palette offer and the point-list builder, so an
/// offered `∿` is an `∿` that builds and solves — the same contract the tangent's
/// guide gate carries. What it admits:
///
/// * exactly two geometries, one of them a spline (a `3n + 1` control polygon),
/// * the other a plain line or an arc/circle,
/// * and NOT a spline handle guide: a guide is a line whose two ends are the
///   spline's own control points, so tangency to it would be tangency of the
///   curve to its own construction artifact. Tangency at the anchor a guide
///   stands for is `⌒`'s lane, which is where that selection belongs.
fn sketch_foot_tangent_pair<'a>(
    doc: &crate::sketch::SketchDoc,
    geos: &[&'a crate::sketch::SketchGeometry],
) -> Option<(&'a crate::sketch::SketchGeometry, &'a crate::sketch::SketchGeometry)> {
    if geos.len() != 2 {
        return None;
    }
    let is_spline = |g: &crate::sketch::SketchGeometry| {
        crate::sketch::spline::is_spline_type(&g.geom_type)
    };
    let is_radial =
        |g: &crate::sketch::SketchGeometry| g.geom_type == "arc" || g.geom_type == "circle";
    let spline = *geos.iter().find(|g| is_spline(g))?;
    let target = *geos.iter().find(|g| !is_spline(g))?;
    if spline.points.len() < 4 || target.points.len() < 2 {
        return None;
    }
    let target_is_usable = if target.geom_type == "line" {
        !crate::sketch::spline::is_handle_guide(doc, target)
    } else {
        is_radial(target)
    };
    target_is_usable.then_some((spline, target))
}

/// The point-id list for a foot-parameter tangency (`∿`) —
/// `[lineA, lineB, s0, s1]` against a line, `[center, boundary, s0, s1]` against
/// an arc/circle, where `(s0, s1)` are the SPLINE's first two control ids acting
/// as a geometry handle.
///
/// The spline is named by a pair rather than by its geometry id because a
/// constraint record's only geometric field is `points` — the same reason the
/// tangent names a circle by `(centre, rim)`. Two leading controls identify one
/// spline: a document cannot hold two curves that start from the same two points
/// without them being the same curve.
fn sketch_build_foot_tangent_points(
    doc: &crate::sketch::SketchDoc,
    geos: &[&crate::sketch::SketchGeometry],
) -> Option<Vec<serde_json::Value>> {
    let (spline, target) = sketch_foot_tangent_pair(doc, geos)?;
    Some(vec![
        target.points[0].clone(),
        target.points[1].clone(),
        spline.points[0].clone(),
        spline.points[1].clone(),
    ])
}

/// The point-id list for a curvature (`ϰ`) constraint from two selected
/// geometries — the SAME two-pair shape the tangent uses, so the solver's
/// dispatch reads it the same way: `[anchorA, handleA, anchorB, handleB]` for a
/// joint, `[anchor, handle, center, boundary]` against a circle/arc.
///
/// Gated on exactly what the palette offers, for the same reason the tangent's
/// guide gate exists: an offered curvature continuity is one that solves. A
/// spline end against a circle is a join; two guides are a join only when they
/// are the two sides of one joint; anything else — two plain lines, a guide and
/// an unrelated line, two ends with nothing holding them together — has no
/// curvature continuity to state and is refused here rather than stored as a
/// constraint the solver would have to name an error on.
fn sketch_build_curvature_points(
    doc: &crate::sketch::SketchDoc,
    geos: &[&crate::sketch::SketchGeometry],
) -> Option<Vec<serde_json::Value>> {
    if geos.len() != 2 {
        return None;
    }
    let is_radial =
        |g: &crate::sketch::SketchGeometry| g.geom_type == "arc" || g.geom_type == "circle";
    if geos.iter().all(|g| g.geom_type == "line") {
        if !crate::sketch::spline::is_curvature_joint(doc, geos[0], geos[1]) {
            return None;
        }
        let (a, b) = (geos[0], geos[1]);
        return Some(vec![
            a.points[0].clone(),
            a.points[1].clone(),
            b.points[0].clone(),
            b.points[1].clone(),
        ]);
    }
    let guide = geos
        .iter()
        .find(|g| crate::sketch::spline::is_end_handle_guide(doc, g))?;
    let circ = geos.iter().find(|g| is_radial(g))?;
    if circ.points.len() < 2 {
        return None;
    }
    Some(vec![
        guide.points[0].clone(),
        guide.points[1].clone(),
        circ.points[0].clone(),
        circ.points[1].clone(),
    ])
}

/// Whether the perpendicular (`⟂`) 4-point list should SWAP its first two points to
/// orient line 1 closer to 90° against line 2 — a faithful port of the previous
/// angle-calculation block. `pts = [l1a, l1b, l2a, l2b]`.
pub(super) fn sketch_perpendicular_should_swap(
    doc: &crate::sketch::SketchDoc,
    pts: &[serde_json::Value],
) -> bool {
    let coord = |v: &serde_json::Value| doc.point(v).map(|p| (p.x, p.y));
    let (Some(p0), Some(p1), Some(p2), Some(p3)) =
        (coord(&pts[0]), coord(&pts[1]), coord(&pts[2]), coord(&pts[3]))
    else {
        return false;
    };
    // calculateAngle(a, b) = atan2(b.y-a.y, b.x-a.x) in [0, 360).
    let angle = |a: (f64, f64), b: (f64, f64)| -> f64 {
        let deg = (b.1 - a.1).atan2(b.0 - a.0) * 180.0 / std::f64::consts::PI;
        (deg + 360.0) % 360.0
    };
    // Fold into (-180, 180]: (a + 180) % 360 - 180.
    let fold = |a: f64| (a + 180.0) % 360.0 - 180.0;
    let line1_a = fold(angle(p0, p1));
    let line1_b = fold(angle(p1, p0));
    let line2 = fold(angle(p2, p3));
    let diff_a = line1_a - line2;
    let diff_b = line1_b - line2;
    (90.0 - diff_a).abs() > (90.0 - diff_b).abs()
}

/// The `type + sorted-point-ids` dedup signature (the solver runs with
/// `remove_implied_duplicates:false`, so this is the only dedup on adds).
pub(super) fn sketch_constraint_signature(ctype: &str, points: &[serde_json::Value]) -> String {
    use crate::sketch::doc::id_key;
    let mut keys: Vec<String> = points.iter().map(id_key).collect();
    keys.sort();
    format!("{ctype}|{}", keys.join(","))
}

/// The port of `ConstraintEngine.createConstraint`: from the session's selection build
/// the ordered point-id list(s) for `symbol`, dedup on `type + sorted-points`, and
/// append the constraint(s) (dimensional → `value:null` + `valueNeedsSetup:true`).
/// Returns whether at least one constraint was added. The caller re-solves.
fn sketch_build_and_add_constraint(
    session: &mut crate::sketch::SketchSession,
    symbol: &str,
) -> bool {
    /// The constraint(s) to append for a symbol: the stored solver `type`, its
    /// display style, and one or more ordered point-id lists (`⏛`-from-2-lines and
    /// the like push several).
    struct Built {
        store_type: String,
        display_style: &'static str,
        lists: Vec<Vec<serde_json::Value>>,
    }

    // --- Phase 1: read-only — assemble the constraint(s) from the selection. ---
    let built: Option<Built> = {
        let doc = &session.doc;
        // `selected` mirrors the previous behavior: point → push it; geometry → push all its
        // points, arc pops the last (center + start). `geo_items`/`point_items` are
        // the role-based lists the specials use; `geometry_type` is the LAST
        // geometry's type; `first_kind` drives the `⋯` reverse.
        let mut selected: Vec<serde_json::Value> = Vec::new();
        let mut geo_items: Vec<&crate::sketch::SketchGeometry> = Vec::new();
        let mut point_items: Vec<serde_json::Value> = Vec::new();
        let mut geometry_type: Option<String> = None;
        let mut first_kind: Option<String> = None;
        let mut has_geometry = false;
        for (i, r) in session.selection.iter().enumerate() {
            let kind = r.get("kind").and_then(|v| v.as_str());
            let Some(id) = r.get("id") else { continue };
            if i == 0 {
                first_kind = kind.map(|k| k.to_string());
            }
            match kind {
                Some("point") => {
                    if let Some(p) = doc.point(id) {
                        selected.push(p.id.clone());
                        point_items.push(p.id.clone());
                    }
                }
                Some("geometry") => {
                    if let Some(g) = doc.geometry(id) {
                        for pid in &g.points {
                            if doc.point(pid).is_some() {
                                selected.push(pid.clone());
                            }
                        }
                        if g.geom_type == "arc" {
                            selected.pop();
                        }
                        geometry_type = Some(g.geom_type.clone());
                        geo_items.push(g);
                        has_geometry = true;
                    }
                }
                _ => {}
            }
        }
        if selected.is_empty() {
            None
        } else {
            let radial = |g: &crate::sketch::SketchGeometry| {
                g.geom_type == "arc" || g.geom_type == "circle"
            };
            let simple = |t: &str, list: Vec<serde_json::Value>, ds: &'static str| Built {
                store_type: t.to_string(),
                display_style: ds,
                lists: vec![list],
            };

            // ---- Geometry-role specials (dispatched by role, not point count). ----
            match symbol {
                "◎" => {
                    if geo_items.len() == 2 && geo_items.iter().all(|g| radial(g)) {
                        Some(simple(
                            "◎",
                            vec![geo_items[0].points[0].clone(), geo_items[1].points[0].clone()],
                            "",
                        ))
                    } else {
                        None
                    }
                }
                "⊜" => {
                    if geo_items.len() == 2 && geo_items.iter().all(|g| radial(g)) {
                        let (g0, g1) = (geo_items[0], geo_items[1]);
                        Some(simple(
                            "⊜",
                            vec![
                                g0.points[0].clone(),
                                g0.points[1].clone(),
                                g1.points[0].clone(),
                                g1.points[1].clone(),
                            ],
                            "",
                        ))
                    } else {
                        None
                    }
                }
                "⌒" => sketch_build_tangent_points(doc, &geo_items).map(|pts| simple("⌒", pts, "")),
                "ϰ" => {
                    sketch_build_curvature_points(doc, &geo_items).map(|pts| simple("ϰ", pts, ""))
                }
                "∿" => sketch_build_foot_tangent_points(doc, &geo_items)
                    .map(|pts| simple("∿", pts, "")),
                "⋰" => {
                    let ids = if geo_items.len() >= 2
                        && geo_items.iter().all(|g| g.geom_type == "line")
                    {
                        let mut v = Vec::new();
                        for g in &geo_items {
                            v.push(g.points[0].clone());
                            v.push(g.points[1].clone());
                        }
                        Some(v)
                    } else if point_items.len() >= 3 {
                        Some(point_items.clone())
                    } else {
                        None
                    };
                    ids.filter(|v| v.len() >= 3).map(|v| simple("⋰", v, ""))
                }
                "⋈" => geo_items
                    .iter()
                    .find(|g| g.geom_type == "line")
                    .filter(|line| line.points.len() >= 2 && point_items.len() == 2)
                    .map(|line| {
                        simple(
                            "⋈",
                            vec![
                                line.points[0].clone(),
                                line.points[1].clone(),
                                point_items[0].clone(),
                                point_items[1].clone(),
                            ],
                            "",
                        )
                    }),
                // Radial dims: a `⟺` on [center, boundary] with a radius/diameter
                // display style (single arc/circle; `selected` is arc-popped to 2).
                "R" | "⌀" => {
                    if selected.len() == 2 {
                        let ds = if symbol == "⌀" { "diameter" } else { "radius" };
                        Some(Built {
                            store_type: "⟺".to_string(),
                            display_style: ds,
                            lists: vec![selected.clone()],
                        })
                    } else {
                        None
                    }
                }
                _ => {
                    // ---- Count-based (mirrors the previous `selected.length` blocks). ----
                    match selected.len() {
                        1 => match symbol {
                            "⏚" => Some(simple("⏚", selected.clone(), "")),
                            _ => None,
                        },
                        2 => match symbol {
                            "━" | "│" | "≡" => Some(simple(symbol, selected.clone(), "")),
                            "⟺" => {
                                let ds = if matches!(
                                    geometry_type.as_deref(),
                                    Some("arc") | Some("circle")
                                ) {
                                    "radius"
                                } else {
                                    ""
                                };
                                Some(Built {
                                    store_type: "⟺".to_string(),
                                    display_style: ds,
                                    lists: vec![selected.clone()],
                                })
                            }
                            _ => None,
                        },
                        3 => match symbol {
                            "⏛" => Some(simple("⏛", selected.clone(), "")),
                            "⋯" => {
                                let mut pts = selected.clone();
                                if has_geometry && first_kind.as_deref() == Some("point") {
                                    pts.reverse();
                                }
                                Some(simple("⋯", pts, ""))
                            }
                            "↥" => {
                                if geo_items.len() == 1 && point_items.len() == 1 {
                                    let line = geo_items[0];
                                    if line.geom_type == "line" && line.points.len() >= 2 {
                                        Some(simple(
                                            "↥",
                                            vec![
                                                line.points[0].clone(),
                                                line.points[1].clone(),
                                                point_items[0].clone(),
                                            ],
                                            "",
                                        ))
                                    } else {
                                        None
                                    }
                                } else {
                                    // 3 raw points (no geometry) → point-line-distance
                                    // over the selection as-is.
                                    Some(simple("↥", selected.clone(), ""))
                                }
                            }
                            "⇌" => Some(simple("⇌", selected.clone(), "")),
                            _ => None,
                        },
                        4 | 5 => match symbol {
                            "⏛" => {
                                // Two lines → constrain BOTH endpoints of line 2 onto
                                // line 1 (two point-on-line constraints).
                                if geo_items.len() == 2
                                    && geo_items.iter().all(|g| g.geom_type == "line")
                                    && geo_items[0].points.len() >= 2
                                    && geo_items[1].points.len() >= 2
                                {
                                    let (g0, g1) = (geo_items[0], geo_items[1]);
                                    Some(Built {
                                        store_type: "⏛".to_string(),
                                        display_style: "",
                                        lists: vec![
                                            vec![
                                                g0.points[0].clone(),
                                                g0.points[1].clone(),
                                                g1.points[0].clone(),
                                            ],
                                            vec![
                                                g0.points[0].clone(),
                                                g0.points[1].clone(),
                                                g1.points[1].clone(),
                                            ],
                                        ],
                                    })
                                } else {
                                    None
                                }
                            }
                            "⟂" => {
                                if selected.len() != 4 {
                                    None
                                } else {
                                    let mut pts = selected.clone();
                                    if sketch_perpendicular_should_swap(doc, &pts) {
                                        pts.swap(0, 1);
                                    }
                                    Some(simple("⟂", pts, ""))
                                }
                            }
                            "∥" => Some(simple("∥", selected.clone(), "")),
                            "∠" => Some(simple("∠", selected.clone(), "")),
                            "⇌" => Some(simple("⇌", selected.clone(), "")),
                            _ => None,
                        },
                        _ => None,
                    }
                }
            }
        }
    };

    let Some(built) = built else {
        return false;
    };

    // --- Phase 2: mutate — dedup + append (every constraint carries the base
    //     fields; the solver seeds a null value from the current measurement). ---
    let doc = &mut session.doc;
    let mut added_any = false;
    for pts in &built.lists {
        let sig = sketch_constraint_signature(&built.store_type, pts);
        let duplicate = doc.constraints.iter().any(|c| match c.ctype() {
            Some(t) => sketch_constraint_signature(t, c.points()) == sig,
            None => false,
        });
        if duplicate {
            continue;
        }
        let id = doc.next_constraint_id();
        let mut raw = serde_json::Map::new();
        raw.insert("id".to_string(), id);
        raw.insert(
            "type".to_string(),
            serde_json::Value::String(built.store_type.clone()),
        );
        raw.insert("points".to_string(), serde_json::Value::Array(pts.clone()));
        raw.insert("labelX".to_string(), serde_json::Value::from(0));
        raw.insert("labelY".to_string(), serde_json::Value::from(0));
        raw.insert(
            "displayStyle".to_string(),
            serde_json::Value::String(built.display_style.to_string()),
        );
        raw.insert("value".to_string(), serde_json::Value::Null);
        raw.insert("valueNeedsSetup".to_string(), serde_json::Value::Bool(true));
        doc.constraints.push(crate::sketch::SketchConstraint { raw });
        added_any = true;
    }
    added_any
}

