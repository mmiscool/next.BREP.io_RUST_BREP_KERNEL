//! The engine-native 3D SKETCH editor's session: the solved document, its
//! diagnostics, the interaction state (selection, hover, armed tool, working
//! plane), screen-space picking and the overlay tessellation.
//!
//! The planar sketcher (`crate::sketch`) lives in a plane's `(u, v)` frame
//! and hit-tests there. A 3D sketch has no such frame: its points are world
//! points, so picking is done in SCREEN space (every point and polyline
//! sample projected through the live camera, distances measured in pixels)
//! and a click lands in 3D through the WORKING PLANE — one of the three world
//! planes through the sketch's current anchor (the last placed point) — or on
//! model geometry under the cursor (see `EngineState::sketch3d_cursor_world`).
//!
//! The document and the solver are the kernel's ([`brep_kernel::Sketch3dDoc`],
//! [`brep_kernel::solve_sketch3d_doc`]); the feature (`S3D`) solves the same
//! document on every regenerate, so what the editor shows is what the model
//! gets.

use serde_json::{json, Value};

pub use brep_kernel::{
    Sketch3dConstraint, Sketch3dDiagnostics, Sketch3dDoc, Sketch3dGeometry, Sketch3dPoint,
    SKETCH3D_CONSTRAINT_TYPES,
};
use brep_kernel::{sketch3d_constraint_type, Vec3};

use crate::sketch::session::{constraint_ref, geometry_ref, point_ref, refs_equal};
use crate::sketch::tessellate::{interaction_color, SketchTessellation, POINT_SIZE_PX};
use crate::sketch::PlaneFrame;
use crate::style::SketchColors;
use crate::view::ViewCamera;

/// The `set_overlay` group names. Separate from the planar sketch's so the
/// two editors never clear each other's groups.
pub const OVERLAY_GEOMETRY: &str = "sketch3d-geometry";
pub const OVERLAY_POINTS: &str = "sketch3d-points";
pub const OVERLAY_PREVIEW: &str = "sketch3d-preview";
pub const OVERLAY_WORKPLANE: &str = "sketch3d-workplane";
/// Every group, for a clear.
pub const OVERLAY_GROUPS: &[&str] =
    &[OVERLAY_GEOMETRY, OVERLAY_POINTS, OVERLAY_PREVIEW, OVERLAY_WORKPLANE];

/// Samples per arc and per bezier span when drawing / hit-testing.
const ARC_SEGMENTS: usize = 48;
const BEZIER_SEGMENTS_PER_SPAN: usize = 24;
/// Half-size of the drawn working-plane square, in pixels.
const WORKPLANE_HALF_PX: f64 = 90.0;
/// Half-size of a drawn reference plane's square, in pixels.
const REF_PLANE_HALF_PX: f64 = 40.0;

/// The draw tools the 3D editor arms (`select` is the neutral state, stored
/// as `None`).
pub const TOOLS: &[&str] = &["select", "point", "line", "arc", "bezier", "pickRef"];

/// The three working planes, by name.
pub const WORK_PLANES: &[&str] = &["XY", "YZ", "XZ"];

/// The working plane: one of the three world planes through an anchor.
#[derive(Clone, Debug, PartialEq)]
pub struct WorkPlane {
    /// `"XY"`, `"YZ"` or `"XZ"`.
    pub name: String,
    /// The point the plane passes through (the sketch's current anchor).
    pub anchor: [f64; 3],
}

impl WorkPlane {
    pub fn new(name: &str, anchor: [f64; 3]) -> Option<Self> {
        if !WORK_PLANES.contains(&name) {
            return None;
        }
        Some(Self {
            name: name.to_string(),
            anchor,
        })
    }

    /// The plane as a frame through the anchor.
    pub fn frame(&self) -> PlaneFrame {
        let mut frame = match self.name.as_str() {
            "YZ" => PlaneFrame::yz(),
            "XZ" => PlaneFrame::xz(),
            _ => PlaneFrame::xy(),
        };
        frame.origin = self.anchor;
        frame
    }

    /// The next plane in the `XY → YZ → XZ → XY` cycle.
    pub fn next_name(&self) -> &'static str {
        match self.name.as_str() {
            "XY" => "YZ",
            "YZ" => "XZ",
            _ => "XY",
        }
    }
}

impl Default for WorkPlane {
    fn default() -> Self {
        Self {
            name: "XY".into(),
            anchor: [0.0; 3],
        }
    }
}

/// One dimension label the app draws as editable text at a projected world
/// anchor — the 3D sibling of [`crate::sketch::dimensions::DimLabel`], in the
/// same JSON shape so the app's label pass draws both.
#[derive(Clone, Debug, serde::Serialize)]
pub struct Sketch3dDimLabel {
    pub id: Value,
    pub text: String,
    pub world: [f64; 3],
    pub value: Option<f64>,
    #[serde(rename = "valueExpr")]
    pub value_expr: Option<String>,
    pub mode: &'static str,
    pub conflicting: bool,
}

/// A constraint the palette can add for the current selection: the type and
/// the ordered point ids it would take.
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct ApplicableConstraint {
    /// The solver type (`distance`, `onPlane`, …).
    #[serde(rename = "type")]
    pub ctype: &'static str,
    /// The palette label (`"Normal to plane"` for a line–plane `parallel`).
    pub label: &'static str,
    /// The ordered point ids the constraint takes.
    pub points: Vec<Value>,
    /// Whether it carries a value (seeded from the current geometry).
    pub dimensional: bool,
}

/// State for one active 3D sketch.
pub struct Sketch3dSession {
    /// The solved document (points carry solved coordinates).
    pub doc: Sketch3dDoc,
    /// The last solve's diagnostics.
    pub diagnostics: Sketch3dDiagnostics,
    /// Selected entity refs (`{"kind":"point"|"geometry"|"constraint","id":…}`).
    pub selection: Vec<Value>,
    /// The entity under the cursor.
    pub hovered: Option<Value>,
    /// The armed draw tool (`None` = select).
    pub tool: Option<String>,
    /// The working plane a free click lands on.
    pub work_plane: WorkPlane,
    /// The overlay palette (the planar sketch's, shared).
    pub colors: SketchColors,
}

impl Sketch3dSession {
    /// Build a session from a (possibly unsolved) doc, solving it once.
    pub fn new(doc: Sketch3dDoc) -> Result<Self, String> {
        let mut session = Self {
            doc,
            diagnostics: Sketch3dDiagnostics::default(),
            selection: Vec::new(),
            hovered: None,
            tool: None,
            work_plane: WorkPlane::default(),
            colors: SketchColors::default(),
        };
        session.resolve()?;
        Ok(session)
    }

    /// Re-solve the document in place.
    pub fn resolve(&mut self) -> Result<(), String> {
        let diagnostics = brep_kernel::solve_sketch3d_doc(
            &mut self.doc,
            &brep_kernel::Sketch3dSolveOptions::default(),
        )?;
        self.diagnostics = diagnostics;
        Ok(())
    }

    // --- selection / hover -------------------------------------------------

    pub fn is_selected(&self, entity_ref: &Value) -> bool {
        self.selection.iter().any(|r| refs_equal(r, entity_ref))
    }

    pub fn toggle_selection(&mut self, entity_ref: Value) {
        if let Some(pos) = self.selection.iter().position(|r| refs_equal(r, &entity_ref)) {
            self.selection.remove(pos);
        } else {
            self.selection.push(entity_ref);
        }
    }

    pub fn clear_selection(&mut self) {
        self.selection.clear();
    }

    pub fn set_hover(&mut self, entity_ref: Option<Value>) {
        self.hovered = entity_ref;
    }

    /// The selected entity refs split by kind: `(point ids, geometry ids,
    /// constraint ids)`.
    pub fn selection_by_kind(&self) -> (Vec<Value>, Vec<Value>, Vec<Value>) {
        let mut points = Vec::new();
        let mut geometries = Vec::new();
        let mut constraints = Vec::new();
        for r in &self.selection {
            let Some(id) = r.get("id").cloned() else { continue };
            match r.get("kind").and_then(Value::as_str) {
                Some("point") => points.push(id),
                Some("geometry") => geometries.push(id),
                Some("constraint") => constraints.push(id),
                _ => {}
            }
        }
        (points, geometries, constraints)
    }

    // --- geometry sampling ---------------------------------------------------

    /// A point's world position.
    pub fn point_world(&self, id: &Value) -> Option<[f64; 3]> {
        self.doc.point(id).map(|p| [p.x, p.y, p.z])
    }

    /// Sample a geometry into an ordered world polyline (`None` for a missing
    /// point or a degenerate shape). A `point` geometry samples to nothing; a
    /// reference `plane` draws as a square about its origin, sized in pixels.
    pub fn sample_geometry(&self, geometry: &Sketch3dGeometry, world_per_pixel: f64) -> Option<Vec<[f64; 3]>> {
        let positions = self.doc.geometry_positions(geometry)?;
        let out: Vec<[f64; 3]> = match (geometry.geom_type.as_str(), positions.len()) {
            ("line", n) if n >= 2 => vec![arr(positions[0]), arr(positions[1])],
            ("arc", n) if n >= 3 => sample_arc(positions[0], positions[1], positions[2])?,
            ("bezier", n) if n >= 4 && (n - 1) % 3 == 0 => sample_bezier(&positions),
            ("plane", n) if n >= 2 => {
                let frame = plane_frame_of(positions[0], positions[1])?;
                plane_square(&frame, REF_PLANE_HALF_PX * world_per_pixel)
            }
            _ => return None,
        };
        (out.len() >= 2).then_some(out)
    }

    // --- screen-space picking ------------------------------------------------

    /// The entity under CSS-pixel `(x, y)` within `radius_px`: a POINT wins over
    /// geometry when one is in range, else the nearest geometry whose projected
    /// polyline passes within range. Construction geometry is pickable.
    pub fn pick_entity(&self, camera: &ViewCamera, x: f64, y: f64, radius_px: f64) -> Option<Value> {
        let mut best_pt: Option<(f64, &Value)> = None;
        for p in &self.doc.points {
            let Some((sx, sy)) = project(camera, [p.x, p.y, p.z]) else { continue };
            let d = (sx - x).hypot(sy - y);
            if d <= radius_px && best_pt.map_or(true, |(bd, _)| d < bd) {
                best_pt = Some((d, &p.id));
            }
        }
        if let Some((_, id)) = best_pt {
            return Some(point_ref(id));
        }
        let wpp = camera.world_per_pixel();
        let mut best_geo: Option<(f64, &Value)> = None;
        for g in &self.doc.geometries {
            let Some(poly) = self.sample_geometry(g, wpp) else { continue };
            let screen: Vec<Option<(f64, f64)>> = poly.iter().map(|w| project(camera, *w)).collect();
            let mut dmin = f64::INFINITY;
            for pair in screen.windows(2) {
                let (Some(a), Some(b)) = (pair[0], pair[1]) else { continue };
                let d = crate::geometry2d::point_segment_distance((x, y), a, b).0;
                if d < dmin {
                    dmin = d;
                }
            }
            if dmin <= radius_px && best_geo.map_or(true, |(bd, _)| dmin < bd) {
                best_geo = Some((dmin, &g.id));
            }
        }
        best_geo.map(|(_, id)| geometry_ref(id))
    }

    /// The nearest DRAGGABLE point under `(x, y)` (id + its authored `fixed`
    /// flag): a point the solver reports movable.
    pub fn pick_draggable_point(&self, camera: &ViewCamera, x: f64, y: f64, radius_px: f64) -> Option<(Value, bool)> {
        let mut best: Option<(f64, &Sketch3dPoint)> = None;
        for p in &self.doc.points {
            if self.diagnostics.point_movable(&p.id) != Some(true) {
                continue;
            }
            let Some((sx, sy)) = project(camera, [p.x, p.y, p.z]) else { continue };
            let d = (sx - x).hypot(sy - y);
            if d <= radius_px && best.map_or(true, |(bd, _)| d < bd) {
                best = Some((d, p));
            }
        }
        best.map(|(_, p)| (p.id.clone(), p.fixed))
    }

    /// The points a drag of `entity_ref` moves: the one point, or every
    /// (deduped, movable) point of a geometry; `None` for a locked point or a
    /// constraint.
    pub fn drag_points_from_ref(&self, entity_ref: &Value) -> Option<Vec<(Value, [f64; 3], bool)>> {
        let id = entity_ref.get("id")?;
        match entity_ref.get("kind").and_then(Value::as_str) {
            Some("point") => {
                if self.diagnostics.point_movable(id) != Some(true) {
                    return None;
                }
                let p = self.doc.point(id)?;
                Some(vec![(p.id.clone(), [p.x, p.y, p.z], p.fixed)])
            }
            Some("geometry") => {
                let g = self.doc.geometry(id)?;
                if g.geom_type == "plane" {
                    return None;
                }
                let mut out: Vec<(Value, [f64; 3], bool)> = Vec::new();
                for pid in &g.points {
                    if out.iter().any(|(seen, _, _)| refs_equal(&point_ref(seen), &point_ref(pid))) {
                        continue;
                    }
                    let Some(p) = self.doc.point(pid) else { continue };
                    if self.diagnostics.point_movable(pid) != Some(true) {
                        continue;
                    }
                    out.push((p.id.clone(), [p.x, p.y, p.z], p.fixed));
                }
                (!out.is_empty()).then_some(out)
            }
            _ => None,
        }
    }

    // --- overlay -----------------------------------------------------------

    /// Tessellate the solved sketch with hover + selection coloring.
    pub fn tessellation(&self, world_per_pixel: f64) -> SketchTessellation {
        let mut out = SketchTessellation::default();
        let colors = &self.colors;
        let hovered = self.hovered.as_ref();
        let selection = &self.selection;
        let dash_len = (8.0 * world_per_pixel).max(0.02);
        let gap_len = (6.0 * world_per_pixel).max(0.01);
        for g in &self.doc.geometries {
            let Some(pts) = self.sample_geometry(g, world_per_pixel) else { continue };
            let mobility = match self.diagnostics.geometry_movable(&g.id) {
                None => colors.geometry,
                Some(true) => colors.movable,
                Some(false) => colors.locked,
            };
            let color = interaction_color(colors, mobility, hovered, selection, "geometry", &g.id);
            let rgb = rgb_of(color);
            if g.construction() {
                push_dashed(&mut out, &pts, rgb, dash_len, gap_len);
            } else {
                push_solid(&mut out, &pts, rgb);
            }
        }
        for p in &self.doc.points {
            let base = if p.construction {
                colors.construction_point
            } else {
                match self.diagnostics.point_movable(&p.id) {
                    Some(true) => colors.movable,
                    Some(false) => colors.locked,
                    None => colors.point,
                }
            };
            let color = interaction_color(colors, base, hovered, selection, "point", &p.id);
            out.point_positions.extend_from_slice(&[p.x as f32, p.y as f32, p.z as f32]);
            out.point_colors.extend_from_slice(&rgb_of(color));
        }
        out
    }

    /// The `set_overlay` JSON for the solved geometry + points.
    pub fn overlay_json(&self, world_per_pixel: f64) -> String {
        let tess = self.tessellation(world_per_pixel);
        json!({
            "groups": [
                {
                    "name": OVERLAY_GEOMETRY,
                    "renderOrder": 10000,
                    "lines": { "positions": tess.line_positions, "colors": tess.line_colors },
                },
                {
                    "name": OVERLAY_POINTS,
                    "renderOrder": 10001,
                    "points": {
                        "positions": tess.point_positions,
                        "colors": tess.point_colors,
                        "size": POINT_SIZE_PX,
                    },
                },
            ]
        })
        .to_string()
    }

    /// The `set_overlay` JSON for the armed tool's rubber band: the pending
    /// points joined, and the last one joined to the hovered world point.
    /// Always the `sketch3d-preview` group (empty when nothing is pending).
    pub fn preview_overlay_json(&self, pending: &[Value], hover: Option<[f64; 3]>) -> String {
        let mut pts: Vec<[f64; 3]> = pending.iter().filter_map(|id| self.point_world(id)).collect();
        if let Some(h) = hover {
            pts.push(h);
        }
        let mut out = SketchTessellation::default();
        let rgb = rgb_of(self.colors.preview);
        match self.tool.as_deref() {
            Some("arc") if pts.len() >= 3 => {
                // center, start, end: preview the arc itself.
                if let Some(arc) = sample_arc(vec(pts[0]), vec(pts[1]), vec(pts[2])) {
                    push_solid(&mut out, &arc, rgb);
                }
                push_solid(&mut out, &pts[..2], rgb);
            }
            Some("bezier") if pts.len() == 4 => {
                let controls: Vec<Vec3> = pts.iter().map(|p| vec(*p)).collect();
                push_solid(&mut out, &sample_bezier(&controls), rgb);
                push_solid(&mut out, &pts, rgb);
            }
            _ if pts.len() >= 2 => push_solid(&mut out, &pts, rgb),
            _ => {}
        }
        json!({
            "groups": [{
                "name": OVERLAY_PREVIEW,
                "renderOrder": 10002,
                "lines": { "positions": out.line_positions, "colors": out.line_colors },
            }]
        })
        .to_string()
    }

    /// The `set_overlay` JSON for the working plane: a dashed square through
    /// the anchor with its two in-plane axes, sized in pixels so it reads the
    /// same at any zoom. Hidden (empty group) while the select tool is armed —
    /// it matters only where a click lands.
    pub fn workplane_overlay_json(&self, world_per_pixel: f64) -> String {
        let mut out = SketchTessellation::default();
        if self.tool.is_some() {
            let frame = self.work_plane.frame();
            let half = WORKPLANE_HALF_PX * world_per_pixel;
            let rgb = rgb_of(self.colors.preview);
            let square = plane_square(&frame, half);
            push_dashed(&mut out, &square, rgb, (8.0 * world_per_pixel).max(0.02), (6.0 * world_per_pixel).max(0.01));
            let o = frame.origin;
            push_solid(&mut out, &[frame.to_world(-half, 0.0), frame.to_world(half, 0.0)], rgb);
            push_solid(&mut out, &[frame.to_world(0.0, -half), frame.to_world(0.0, half)], rgb);
            let _ = o;
        }
        json!({
            "groups": [{
                "name": OVERLAY_WORKPLANE,
                "renderOrder": 9999,
                "lines": { "positions": out.line_positions, "colors": out.line_colors },
            }]
        })
        .to_string()
    }

    // --- dimensions ------------------------------------------------------------

    /// One label per dimensional constraint, anchored at the dimension's
    /// natural midpoint.
    pub fn dimension_labels(&self) -> Vec<Sketch3dDimLabel> {
        let mut out = Vec::new();
        for c in &self.doc.constraints {
            let (Some(ctype), Some(id)) = (c.ctype(), c.id()) else { continue };
            let Some(spec) = sketch3d_constraint_type(ctype) else { continue };
            if !spec.dimensional {
                continue;
            }
            let pts: Vec<[f64; 3]> = c.points().iter().filter_map(|pid| self.point_world(pid)).collect();
            if pts.len() < spec.arity {
                continue;
            }
            let (world, mode) = match ctype {
                "distance" => (mid(pts[0], pts[1]), "distance"),
                "angle" => (mid(mid(pts[0], pts[1]), mid(pts[2], pts[3])), "angle"),
                _ => (pts[0], "distance"),
            };
            let value = c.value();
            let text = match (mode, value) {
                ("angle", Some(v)) => format!("{v:.2}°"),
                (_, Some(v)) => format!("{v:.3}"),
                _ => "?".to_string(),
            };
            out.push(Sketch3dDimLabel {
                id: id.clone(),
                text,
                world,
                value,
                value_expr: c.raw.get("valueExpr").and_then(Value::as_str).map(String::from),
                mode,
                conflicting: self.diagnostics.constraint_conflicting(id),
            });
        }
        out
    }

    // --- the constraint palette ------------------------------------------------

    /// The constraints the palette offers for the current selection, each with
    /// the point ids it would take, in palette order.
    pub fn applicable_constraints(&self) -> Vec<ApplicableConstraint> {
        let (points, geometries, _) = self.selection_by_kind();
        let geos: Vec<&Sketch3dGeometry> = geometries.iter().filter_map(|id| self.doc.geometry(id)).collect();
        let lines: Vec<&Sketch3dGeometry> = geos.iter().copied().filter(|g| g.geom_type == "line" && g.points.len() >= 2).collect();
        let arcs: Vec<&Sketch3dGeometry> = geos.iter().copied().filter(|g| g.geom_type == "arc" && g.points.len() >= 3).collect();
        let planes: Vec<&Sketch3dGeometry> = geos.iter().copied().filter(|g| g.geom_type == "plane" && g.points.len() >= 2).collect();
        let others = geos.len() - lines.len() - arcs.len() - planes.len();
        let mut out = Vec::new();
        let mut push = |ctype: &'static str, label: &'static str, ids: Vec<Value>| {
            let dimensional = sketch3d_constraint_type(ctype).map_or(false, |t| t.dimensional);
            out.push(ApplicableConstraint { ctype, label, points: ids, dimensional });
        };
        let line_pts = |g: &Sketch3dGeometry| vec![g.points[0].clone(), g.points[1].clone()];
        let plane_pts = |g: &Sketch3dGeometry| vec![g.points[0].clone(), g.points[1].clone()];
        if others > 0 {
            return out;
        }
        match (points.len(), lines.len(), arcs.len(), planes.len()) {
            (1, 0, 0, 0) => {
                let fixed = self.doc.point(&points[0]).map_or(false, |p| p.fixed);
                if !fixed {
                    push("fixed", "Fix", vec![points[0].clone()]);
                }
            }
            (2, 0, 0, 0) => {
                push("coincident", "Coincident", vec![points[0].clone(), points[1].clone()]);
                push("distance", "Distance", vec![points[0].clone(), points[1].clone()]);
            }
            (0, 1, 0, 0) => {
                let ids = line_pts(lines[0]);
                push("distance", "Length", ids.clone());
                push("alongX", "Along X", ids.clone());
                push("alongY", "Along Y", ids.clone());
                push("alongZ", "Along Z", ids);
            }
            (0, 2, 0, 0) => {
                let mut ids = line_pts(lines[0]);
                ids.extend(line_pts(lines[1]));
                push("parallel", "Parallel", ids.clone());
                push("perpendicular", "Perpendicular", ids.clone());
                push("angle", "Angle", ids.clone());
                push("equal", "Equal length", ids);
            }
            (1, 1, 0, 0) => {
                let mut ids = vec![points[0].clone()];
                ids.extend(line_pts(lines[0]));
                push("onLine", "On line", ids.clone());
                push("lineDistance", "Distance to line", ids);
            }
            (1, 0, 0, 1) => {
                let mut ids = vec![points[0].clone()];
                ids.extend(plane_pts(planes[0]));
                push("onPlane", "On plane", ids.clone());
                push("planeDistance", "Distance to plane", ids);
            }
            (0, 1, 0, 1) => {
                let mut ids = line_pts(lines[0]);
                ids.extend(plane_pts(planes[0]));
                push("parallel", "Normal to plane", ids.clone());
                push("perpendicular", "Parallel to plane", ids);
            }
            (0, 1, 1, 0) => {
                let mut ids = line_pts(lines[0]);
                ids.extend(arcs[0].points[..3].iter().cloned());
                push("tangent", "Tangent", ids);
            }
            (0, 0, 1, 0) => {
                let a = &arcs[0].points;
                push("distance", "Radius", vec![a[0].clone(), a[1].clone()]);
            }
            (0, 0, 2, 0) => {
                let (a, b) = (&arcs[0].points, &arcs[1].points);
                push("equal", "Equal radius", vec![a[0].clone(), a[1].clone(), b[0].clone(), b[1].clone()]);
            }
            _ => {}
        }
        out
    }

    /// The value a dimensional constraint over `ids` would be SEEDED with — the
    /// current measurement, so adding a dimension changes nothing until the
    /// user edits it.
    pub fn measure(&self, ctype: &str, ids: &[Value]) -> Option<f64> {
        let p: Vec<Vec3> = ids.iter().filter_map(|id| self.point_world(id)).map(vec).collect();
        match ctype {
            "distance" if p.len() >= 2 => Some(p[1].sub(p[0]).length()),
            "lineDistance" if p.len() >= 3 => {
                let d = p[2].sub(p[1]).normalized().ok()?;
                let v = p[0].sub(p[1]);
                Some(v.sub(d.scale(d.dot(v))).length())
            }
            "planeDistance" if p.len() >= 3 => {
                let n = p[2].sub(p[1]).normalized().ok()?;
                Some(n.dot(p[0].sub(p[1])))
            }
            "angle" if p.len() >= 4 => {
                let a = p[1].sub(p[0]).normalized().ok()?;
                let b = p[3].sub(p[2]).normalized().ok()?;
                Some(a.dot(b).clamp(-1.0, 1.0).acos().to_degrees())
            }
            _ => None,
        }
    }

    /// Rows for the entity lists: `(kind, id, label)` per point, geometry and
    /// constraint, with the mobility / conflict state the list colors by.
    pub fn entity_rows_json(&self) -> Value {
        let points: Vec<Value> = self
            .doc
            .points
            .iter()
            .map(|p| {
                json!({
                    "id": p.id,
                    "fixed": p.fixed,
                    "construction": p.construction,
                    "externalReference": p.external_reference,
                    "movable": self.diagnostics.point_movable(&p.id),
                    "position": [p.x, p.y, p.z],
                })
            })
            .collect();
        let geometries: Vec<Value> = self
            .doc
            .geometries
            .iter()
            .map(|g| {
                json!({
                    "id": g.id,
                    "type": g.geom_type,
                    "construction": g.construction(),
                    "movable": self.diagnostics.geometry_movable(&g.id),
                    "points": g.points,
                })
            })
            .collect();
        let constraints: Vec<Value> = self
            .doc
            .constraints
            .iter()
            .map(|c| {
                json!({
                    "id": c.id(),
                    "type": c.ctype(),
                    "points": c.points(),
                    "value": c.value(),
                    "conflicting": c.id().map_or(false, |id| self.diagnostics.constraint_conflicting(id)),
                })
            })
            .collect();
        json!({ "points": points, "geometries": geometries, "constraints": constraints })
    }
}

// ---------------------------------------------------------------------------
// Entity refs (shared shape with the planar sketch)
// ---------------------------------------------------------------------------

pub fn entity_point_ref(id: &Value) -> Value {
    point_ref(id)
}

pub fn entity_geometry_ref(id: &Value) -> Value {
    geometry_ref(id)
}

pub fn entity_constraint_ref(id: &Value) -> Value {
    constraint_ref(id)
}

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

fn arr(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn vec(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn mid(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5]
}

/// Project a world point to CSS pixels, `None` when behind the eye.
pub fn project(camera: &ViewCamera, world: [f64; 3]) -> Option<(f64, f64)> {
    if !camera.projectable(world) {
        return None;
    }
    let (x, y, _) = camera.project(world);
    Some((x, y))
}

/// The minor arc from `start` to `end` about `center`, sampled; `None` when the
/// three points are collinear or the radius is zero.
pub fn sample_arc(center: Vec3, start: Vec3, end: Vec3) -> Option<Vec<[f64; 3]>> {
    let rs = start.sub(center);
    let re = end.sub(center);
    let radius = rs.length();
    if radius <= 1e-12 {
        return None;
    }
    let normal = rs.cross(re);
    if normal.length() <= 1e-12 * radius * radius {
        return None;
    }
    let x_axis = rs.normalized().ok()?;
    let y_axis = normal.normalized().ok()?.cross(x_axis).normalized().ok()?;
    let sweep = re.dot(y_axis).atan2(re.dot(x_axis));
    let sweep = if sweep <= 0.0 { sweep + std::f64::consts::TAU } else { sweep };
    let r_end = re.length();
    let mut out = Vec::with_capacity(ARC_SEGMENTS + 1);
    for i in 0..=ARC_SEGMENTS {
        let t = i as f64 / ARC_SEGMENTS as f64;
        let a = sweep * t;
        // Blend the radius from start to end so an arc whose implied
        // equal-radius row is not yet satisfied still draws through both ends.
        let r = radius + (r_end - radius) * t;
        out.push(arr(center.add(x_axis.scale(r * a.cos())).add(y_axis.scale(r * a.sin()))));
    }
    Some(out)
}

/// A cubic bezier chain (`3n+1` controls), sampled per span.
pub fn sample_bezier(controls: &[Vec3]) -> Vec<[f64; 3]> {
    let span_count = controls.len().saturating_sub(1) / 3;
    let mut out = Vec::with_capacity(span_count * BEZIER_SEGMENTS_PER_SPAN + 1);
    for span in 0..span_count {
        let p = &controls[span * 3..span * 3 + 4];
        let first = if span == 0 { 0 } else { 1 };
        for i in first..=BEZIER_SEGMENTS_PER_SPAN {
            let t = i as f64 / BEZIER_SEGMENTS_PER_SPAN as f64;
            let u = 1.0 - t;
            let point = p[0]
                .scale(u * u * u)
                .add(p[1].scale(3.0 * u * u * t))
                .add(p[2].scale(3.0 * u * t * t))
                .add(p[3].scale(t * t * t));
            out.push(arr(point));
        }
    }
    out
}

/// The frame of a reference plane `[origin, normalTip]`.
pub fn plane_frame_of(origin: Vec3, tip: Vec3) -> Option<PlaneFrame> {
    let normal = tip.sub(origin);
    if normal.length() <= 1e-12 {
        return None;
    }
    Some(PlaneFrame::from_normal(arr(origin), arr(normal)))
}

/// A closed square of half-size `half` in `frame`, as a 5-point polyline.
fn plane_square(frame: &PlaneFrame, half: f64) -> Vec<[f64; 3]> {
    vec![
        frame.to_world(-half, -half),
        frame.to_world(half, -half),
        frame.to_world(half, half),
        frame.to_world(-half, half),
        frame.to_world(-half, -half),
    ]
}

fn rgb_of(color: u32) -> [f32; 3] {
    [
        ((color >> 16) & 0xff) as f32 / 255.0,
        ((color >> 8) & 0xff) as f32 / 255.0,
        (color & 0xff) as f32 / 255.0,
    ]
}

fn push_seg(out: &mut SketchTessellation, a: [f64; 3], b: [f64; 3], rgb: [f32; 3]) {
    out.line_positions.extend_from_slice(&[
        a[0] as f32, a[1] as f32, a[2] as f32, b[0] as f32, b[1] as f32, b[2] as f32,
    ]);
    out.line_colors.extend_from_slice(&[rgb[0], rgb[1], rgb[2], rgb[0], rgb[1], rgb[2]]);
}

fn push_solid(out: &mut SketchTessellation, pts: &[[f64; 3]], rgb: [f32; 3]) {
    for pair in pts.windows(2) {
        push_seg(out, pair[0], pair[1], rgb);
    }
}

fn push_dashed(out: &mut SketchTessellation, pts: &[[f64; 3]], rgb: [f32; 3], dl: f64, gl: f64) {
    let mut on = true;
    let mut need = dl;
    for pair in pts.windows(2) {
        let (a, b) = (vec(pair[0]), vec(pair[1]));
        let seg = b.sub(a);
        let len = seg.length();
        if len <= 0.0 {
            continue;
        }
        let dir = seg.scale(1.0 / len);
        let mut t = 0.0;
        while t < len {
            let step = need.min(len - t);
            if on {
                push_seg(out, arr(a.add(dir.scale(t))), arr(a.add(dir.scale(t + step))), rgb);
            }
            t += step;
            need -= step;
            if need <= 1e-12 {
                on = !on;
                need = if on { dl } else { gl };
            }
        }
    }
}

