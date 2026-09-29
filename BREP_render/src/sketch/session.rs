//! A sketch's solved document, plane, diagnostics, and interaction state.

use crate::geometry2d::point_segment_distance;

use serde_json::{json, Value};

use super::doc::{id_key, SketchDiagnostics, SketchDoc};
use super::tessellate::{self, SketchTessellation};
use super::{solve, PlaneFrame};
use crate::style::SketchColors;

/// State for one active sketch.
pub struct SketchSession {
    /// The solved sketch (points carry solved coordinates).
    pub doc: SketchDoc,
    /// The plane the sketch's `(u, v)` coordinates are embedded in.
    pub plane: PlaneFrame,
    /// The last solve's read-only diagnostics (DOF, over/under status, mobility).
    pub diagnostics: SketchDiagnostics,

    // --- interaction state ---------------------------------------------------
    /// Selected entities — a set of entity refs
    /// (`{"kind":"point"|"geometry","id":<id>}`; see [`point_ref`]/[`geometry_ref`]).
    pub selection: Vec<Value>,
    /// The entity ref currently under the cursor, or `None`.
    pub hovered: Option<Value>,
    /// The active drawing tool.
    pub tool: Option<String>,
    /// Persisted dimension label offsets.
    pub dim_offsets: serde_json::Map<String, Value>,
    /// The overlay color palette — a live [`SketchColors`] view of the display
    /// settings (`RenderSettings::sketch_colors`). The engine sets this on entry and
    /// re-syncs it when the settings change; every overlay builder reads from here so
    /// the sketch colors are managed in the settings like the rest of the display.
    /// Defaults to [`SketchColors::default`] so a session built in a test (or before
    /// the engine syncs) renders the standard theme.
    pub colors: SketchColors,
    /// User-tunable solver knobs (the Solver Settings panel). Read by every
    /// [`resolve`](Self::resolve); default reproduces the historical solve.
    pub solver_settings: solve::SketchSolverSettings,
}

impl SketchSession {
    /// Build a session from a (possibly unsolved) doc + plane, solving it once.
    pub fn new(doc: SketchDoc, plane: PlaneFrame) -> Result<Self, String> {
        let (solved, diagnostics) = solve::solve(&doc)?;
        Ok(Self {
            doc: solved,
            plane,
            diagnostics,
            selection: Vec::new(),
            hovered: None,
            tool: None,
            dim_offsets: serde_json::Map::new(),
            colors: SketchColors::default(),
            solver_settings: solve::SketchSolverSettings::default(),
        })
    }

    /// Re-solve the current doc, refreshing coordinates + diagnostics in place,
    /// honoring the session's [`solver_settings`](Self::solver_settings).
    pub fn resolve(&mut self) -> Result<(), String> {
        let (solved, diagnostics) = solve::solve_with(&self.doc, &self.solver_settings)?;
        self.doc = solved;
        self.diagnostics = diagnostics;
        Ok(())
    }

    /// The `set_overlay` JSON for this solved sketch (see
    /// [`tessellate::overlay_json`]). `world_per_pixel` sizes construction dashes.
    pub fn overlay_json(&self, world_per_pixel: f64) -> String {
        tessellate::overlay_json(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            world_per_pixel,
            &self.colors,
        )
    }

    /// The flat overlay buffers (for tests / verification stats).
    pub fn tessellation(&self, world_per_pixel: f64) -> SketchTessellation {
        tessellate::tessellate(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            world_per_pixel,
            &self.colors,
        )
    }

    /// The `set_overlay` JSON with the live hover + selection colored in (S2). Empty
    /// hover + selection reproduce [`overlay_json`](Self::overlay_json) exactly.
    pub fn overlay_json_with_state(&self, world_per_pixel: f64) -> String {
        tessellate::overlay_json_with_state(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            world_per_pixel,
            &self.colors,
            self.hovered.as_ref(),
            &self.selection,
        )
    }

    /// The `set_overlay` JSON for the `sketch-dim-leaders` group (S5): the per-type
    /// leader/arrow segments for every dimensional constraint, offset by its stored
    /// `{du, dv}` (see [`dimensions`](super::dimensions)). Always emitted (empty when
    /// the sketch has no dimensions) so a stale group clears on the next refresh.
    pub fn dim_leaders_overlay_json(&self, world_per_pixel: f64) -> String {
        super::dimensions::dimension_leaders_overlay_json(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            &self.dim_offsets,
            world_per_pixel,
            &self.colors,
        )
    }

    /// Like [`dim_leaders_overlay_json`](Self::dim_leaders_overlay_json) but emphasizes
    /// the live hovered / selected dimensional constraint (amber selected, light-blue
    /// hovered — matching points/geometry). Used by the interactive overlay refresh.
    pub fn dim_leaders_overlay_json_with_state(&self, world_per_pixel: f64) -> String {
        super::dimensions::dimension_leaders_overlay_json_with_state(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            &self.dim_offsets,
            world_per_pixel,
            &self.colors,
            self.hovered.as_ref(),
            &self.selection,
        )
    }

    /// The `set_overlay` JSON for the `sketch-constraint-glyphs` group (S6c): the small
    /// screen-constant line-art marks for every GEOMETRIC (non-dimensional) constraint
    /// — perpendicular, parallel, horizontal/vertical, coincident, equal, … (see
    /// [`constraint_glyphs`](super::constraint_glyphs)). Painted in the shared
    /// constraint green. Always emitted (empty when the sketch has no geometric
    /// constraints) so a stale group clears on the next refresh.
    pub fn constraint_glyphs_overlay_json(&self, world_per_pixel: f64) -> String {
        super::constraint_glyphs::constraint_glyphs_overlay_json(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            world_per_pixel,
            &self.colors,
        )
    }

    /// Like [`constraint_glyphs_overlay_json`](Self::constraint_glyphs_overlay_json) but
    /// emphasizes the live hovered / selected geometric constraint (amber selected,
    /// light-blue hovered — matching points/geometry). Used by the interactive refresh.
    pub fn constraint_glyphs_overlay_json_with_state(&self, world_per_pixel: f64) -> String {
        super::constraint_glyphs::constraint_glyphs_overlay_json_with_state(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            world_per_pixel,
            &self.colors,
            self.hovered.as_ref(),
            &self.selection,
        )
    }

    /// The per-constraint dimension labels (S5): one [`DimLabel`](super::dimensions::DimLabel)
    /// per dimensional constraint, each anchored in world space (`plane.to_world` of
    /// the label uv + its stored offset). brep-app projects `world` → screen and
    /// draws the editable value text there.
    pub fn dimension_labels(&self, world_per_pixel: f64) -> Vec<super::dimensions::DimLabel> {
        super::dimensions::dimension_labels(
            &self.doc,
            &self.diagnostics,
            &self.plane,
            &self.dim_offsets,
            world_per_pixel,
        )
    }

    /// The `set_overlay` JSON for the active draw tool's in-progress rubber-band
    /// preview (S3a): the dim dashed geometry from the `pending` anchor points (their
    /// ids, resolved to uv against the live doc) toward `hover_uv`, plus the raw
    /// freehand `stroke` (S6b-3) as a dim solid polyline while a handdraw drag is live.
    /// Always a `sketch-preview` group (empty when there is nothing to preview yet).
    pub fn preview_overlay_json(
        &self,
        world_per_pixel: f64,
        pending: &[Value],
        hover_uv: Option<(f64, f64)>,
        stroke: &[(f64, f64)],
    ) -> String {
        let pending_uv: Vec<[f64; 2]> = pending
            .iter()
            .filter_map(|id| self.doc.point(id).map(|p| [p.x, p.y]))
            .collect();
        let stroke_uv: Vec<[f64; 2]> = stroke.iter().map(|&(u, v)| [u, v]).collect();
        tessellate::preview_overlay_json(
            self.tool.as_deref(),
            &pending_uv,
            hover_uv,
            &stroke_uv,
            &self.plane,
            world_per_pixel,
            &self.colors,
        )
    }

    // --- S2 hover / selection state ------------------------------------------

    /// Whether `entity_ref` is in the selection set (matched via [`refs_equal`]).
    pub fn is_selected(&self, entity_ref: &Value) -> bool {
        self.selection.iter().any(|r| refs_equal(r, entity_ref))
    }

    /// Toggle `entity_ref` in the selection set: remove it if present, else add it.
    pub fn toggle_selection(&mut self, entity_ref: Value) {
        if let Some(pos) = self.selection.iter().position(|r| refs_equal(r, &entity_ref)) {
            self.selection.remove(pos);
        } else {
            self.selection.push(entity_ref);
        }
    }

    /// Clear the selection set.
    pub fn clear_selection(&mut self) {
        self.selection.clear();
    }

    /// Set (or clear) the hovered entity ref.
    pub fn set_hover(&mut self, entity_ref: Option<Value>) {
        self.hovered = entity_ref;
    }

    // --- S2 hit-testing ------------------------------------------------------

    /// The entity ref nearest to plane coordinate `(u, v)` within `radius` — a
    /// POINT wins over geometry when one is in range (points take priority), else
    /// the nearest geometry whose polyline passes within `radius`. Construction
    /// geometry is included (it is pickable). `None` when nothing is in range.
    pub fn pick_entity(&self, u: f64, v: f64, radius: f64) -> Option<Value> {
        // Points first (priority over geometry under the cursor).
        let mut best_pt: Option<(f64, &Value)> = None;
        for p in &self.doc.points {
            let d = ((p.x - u).powi(2) + (p.y - v).powi(2)).sqrt();
            if d <= radius && best_pt.map_or(true, |(bd, _)| d < bd) {
                best_pt = Some((d, &p.id));
            }
        }
        if let Some((_, id)) = best_pt {
            return Some(point_ref(id));
        }

        // Else the nearest geometry polyline within radius.
        let mut best_geo: Option<(f64, &Value)> = None;
        for g in &self.doc.geometries {
            let poly = tessellate::geometry_polyline_uv(g, &self.doc);
            if poly.len() < 2 {
                continue;
            }
            let mut dmin = f64::INFINITY;
            for seg in poly.windows(2) {
                let d = point_segment_distance((u, v), seg[0].into(), seg[1].into()).0;
                if d < dmin {
                    dmin = d;
                }
            }
            if dmin <= radius && best_geo.map_or(true, |(bd, _)| dmin < bd) {
                best_geo = Some((dmin, &g.id));
            }
        }
        best_geo.map(|(_, id)| geometry_ref(id))
    }

    /// The CONSTRAINT nearest to `(u, v)` within `radius` — a `constraint` entity ref
    /// (`{"kind":"constraint","id":<id>}`), or `None`. A GEOMETRIC constraint is picked
    /// near its glyph line-art (the exact segments [`constraint_glyphs`] draws); a
    /// DIMENSIONAL one near its leader/arrow lines (NOT its value label — the label is
    /// the egui value-editor affordance). Solver-internal `temporary` helpers are never
    /// pickable. `world_per_pixel` sizes the screen-constant glyph/leader placement (so
    /// picking tracks what is drawn). The CALLER enforces priority: points > geometry >
    /// constraint (see `EngineState::sketch_entity_at`), so a glyph over a point never
    /// shadows the point.
    ///
    /// [`constraint_glyphs`]: super::constraint_glyphs
    pub fn pick_constraint(&self, u: f64, v: f64, radius: f64, world_per_pixel: f64) -> Option<Value> {
        let mut best: Option<(f64, &Value)> = None;
        for c in &self.doc.constraints {
            if c.temporary() {
                continue;
            }
            let Some(id) = c.raw.get("id") else { continue };
            // A constraint contributes glyph segments (geometric) XOR leader segments
            // (dimensional); measure the cursor against whichever it draws.
            let mut dmin = f64::INFINITY;
            for (a, b) in
                super::constraint_glyphs::constraint_glyph_segments(c, &self.doc, world_per_pixel)
            {
                let d = point_segment_distance((u, v), a.into(), b.into()).0;
                if d < dmin {
                    dmin = d;
                }
            }
            if let Some(segments) = super::dimensions::constraint_dim_segments(
                c,
                &self.doc,
                &self.dim_offsets,
                world_per_pixel,
            ) {
                for (a, b) in segments {
                    let d = point_segment_distance((u, v), a.into(), b.into()).0;
                    if d < dmin {
                        dmin = d;
                    }
                }
            }
            if dmin <= radius && best.map_or(true, |(bd, _)| dmin < bd) {
                best = Some((dmin, id));
            }
        }
        best.map(|(_, id)| constraint_ref(id))
    }

    /// The nearest DRAGGABLE point to `(u, v)` within `radius` (its id + authored
    /// `fixed` flag), or `None`. A point is draggable only if the solver reports it
    /// movable (a locked / grounded point has no free coordinates to drag), so a
    /// drag on a fully-constrained point falls through to a camera orbit.
    pub fn pick_draggable_point(&self, u: f64, v: f64, radius: f64) -> Option<(Value, bool)> {
        let mut best: Option<(f64, &super::doc::SketchPoint)> = None;
        for p in &self.doc.points {
            let movable = self.diagnostics.point_movable(&p.id).unwrap_or(!p.fixed);
            if !movable {
                continue;
            }
            let d = ((p.x - u).powi(2) + (p.y - v).powi(2)).sqrt();
            if d <= radius && best.map_or(true, |(bd, _)| d < bd) {
                best = Some((d, p));
            }
        }
        best.map(|(_, p)| (p.id.clone(), p.fixed))
    }

    /// The point-set to DRAG for a hovered entity ref (`{"kind","id"}`) — the grab
    /// path grabs exactly what's highlighted rather than re-picking at egui's
    /// offset drag-start position. A `"point"` ref → that point iff it is MOVABLE;
    /// a `"geometry"` ref → ALL of its (deduped) points when at least one is movable
    /// (a rigid translate). Each entry is `(id, orig_x, orig_y, orig_fixed)`. `None`
    /// for a locked point, a fully-locked geometry, or a stale/unknown ref (the
    /// caller then falls through to a camera gesture — it never drags something the
    /// user is not pointing at).
    pub fn drag_points_from_ref(&self, entity_ref: &Value) -> Option<Vec<(Value, f64, f64, bool)>> {
        let kind = entity_ref.get("kind").and_then(Value::as_str)?;
        let id = entity_ref.get("id")?;
        match kind {
            "point" => {
                let p = self.doc.point(id)?;
                let movable = self.diagnostics.point_movable(&p.id).unwrap_or(!p.fixed);
                movable.then(|| vec![(p.id.clone(), p.x, p.y, p.fixed)])
            }
            "geometry" => {
                // Raw doc ids compare via `id_key` (numeric identity) — NOT
                // `refs_equal`, which is for `{"kind","id"}` entity refs and treats
                // every bare id as equal.
                let g = self
                    .doc
                    .geometries
                    .iter()
                    .find(|g| id_key(&g.id) == id_key(id))?;
                let mut out: Vec<(Value, f64, f64, bool)> = Vec::new();
                let mut any_movable = false;
                for pid in &g.points {
                    // A closed polyline repeats its start id — dedupe so a point is
                    // pinned once.
                    if out.iter().any(|(pt_id, ..)| id_key(pt_id) == id_key(pid)) {
                        continue;
                    }
                    if let Some(p) = self.doc.point(pid) {
                        any_movable |= self.diagnostics.point_movable(&p.id).unwrap_or(!p.fixed);
                        out.push((p.id.clone(), p.x, p.y, p.fixed));
                    }
                }
                (any_movable && !out.is_empty()).then_some(out)
            }
            _ => None,
        }
    }

    /// Seed a standalone demo sketch (S0 milestone): a fully-constrained 20×12
    /// rectangle grounded at the origin PLUS a free (unconstrained) circle beside
    /// it, both on the XY plane. The mix proves the pipeline end to end — the
    /// rectangle solves to `locked` (white), the circle stays `movable` (blue),
    /// and the sketch reports `dof = 4` (the circle's four free coordinates).
    pub fn seed_rectangle_circle() -> Result<Self, String> {
        Self::new(seed_doc(), PlaneFrame::xy())
    }
}

/// The seed document consumed by [`SketchSession::seed_rectangle_circle`].
fn seed_doc() -> SketchDoc {
    let value = json!({
        "points": [
            // rectangle corners (p0 grounded at the origin)
            { "id": 0, "x": 0.0,  "y": 0.0,  "fixed": false, "construction": false, "externalReference": false },
            { "id": 1, "x": 20.0, "y": 0.0,  "fixed": false, "construction": false, "externalReference": false },
            { "id": 2, "x": 20.0, "y": 12.0, "fixed": false, "construction": false, "externalReference": false },
            { "id": 3, "x": 0.0,  "y": 12.0, "fixed": false, "construction": false, "externalReference": false },
            // free circle (center + radius point), unconstrained -> 4 DOF, movable
            { "id": 4, "x": 34.0, "y": 6.0,  "fixed": false, "construction": false, "externalReference": false },
            { "id": 5, "x": 40.0, "y": 6.0,  "fixed": false, "construction": false, "externalReference": false }
        ],
        "geometries": [
            { "id": 10, "type": "line",   "points": [0, 1], "construction": false },
            { "id": 11, "type": "line",   "points": [1, 2], "construction": false },
            { "id": 12, "type": "line",   "points": [2, 3], "construction": false },
            { "id": 13, "type": "line",   "points": [3, 0], "construction": false },
            { "id": 20, "type": "circle", "points": [4, 5], "construction": false }
        ],
        "constraints": [
            { "id": 0, "type": "⏚", "points": [0] },
            { "id": 1, "type": "━", "points": [0, 1] },
            { "id": 2, "type": "⟺", "points": [0, 1], "value": 20.0 },
            { "id": 3, "type": "│", "points": [1, 2] },
            { "id": 4, "type": "⟺", "points": [1, 2], "value": 12.0 },
            { "id": 5, "type": "━", "points": [2, 3] },
            { "id": 6, "type": "│", "points": [3, 0] }
        ]
    });
    serde_json::from_value(value).expect("seed sketch doc is valid")
}

// --- S2 entity-ref convention --------------------------------------------------
//
// Hover + selection both use ONE ref shape: `{"kind":"point"|"geometry","id":<id>}`
// where `<id>` is the raw doc id `Value` (a number in practice). Two refs are equal
// when their kinds match and their ids agree under the solver's [`id_key`] identity
// (so numeric `4` and string `"4"` are the same entity).

/// Build a `{"kind":"point","id":<id>}` entity ref.
pub fn point_ref(id: &Value) -> Value {
    json!({ "kind": "point", "id": id.clone() })
}

/// Build a `{"kind":"geometry","id":<id>}` entity ref.
pub fn geometry_ref(id: &Value) -> Value {
    json!({ "kind": "geometry", "id": id.clone() })
}

/// Build a `{"kind":"constraint","id":<id>}` entity ref — a CONSTRAINT is selectable
/// like a point/geometry (picked near its glyph or dimension leader) so it can be
/// emphasized + deleted; deleting it drops only the constraint, never its geometry.
pub fn constraint_ref(id: &Value) -> Value {
    json!({ "kind": "constraint", "id": id.clone() })
}

/// Whether two entity refs name the same entity (same `kind`, same `id` under
/// [`id_key`]).
pub fn refs_equal(a: &Value, b: &Value) -> bool {
    a.get("kind") == b.get("kind") && a.get("id").map(id_key) == b.get("id").map(id_key)
}

/// Whether two optional entity refs are equal (both absent, or both present and
/// [`refs_equal`]) — the hover-changed test.
pub fn entity_ref_eq(a: Option<&Value>, b: Option<&Value>) -> bool {
    match (a, b) {
        (None, None) => true,
        (Some(x), Some(y)) => refs_equal(x, y),
        _ => false,
    }
}

