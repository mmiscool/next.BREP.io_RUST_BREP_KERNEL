//! S3D — 3D Sketch. Free-space points, lines, arcs and bezier chains, held
//! together by the 3D constraint solver (`solvers/sketch3d_solver.rs`) and
//! tied to the model through linked REFERENCES: a vertex, an edge, a face, a
//! datum plane or axis, or a point another feature published. It produces no
//! solid; it publishes its solved curves as PATHS, so the tube, sweep and
//! path-sweep features build tubular frames and routed runs along it exactly
//! as they do along a planar sketch.
//!
//! # Document (`persistentData`)
//!
//! - `sketch3d` — the [`Sketch3dDoc`]: points `{id, x, y, z, fixed?,
//!   construction?, externalReference?}`, geometries `{id, type, points}` with
//!   `point` / `line` / `arc` (center, start, end) / `bezier` (3n+1 control
//!   points) / `plane` (origin, normal tip; always construction), and
//!   constraints `{id, type, points, value?, valueExpr?}` from the solver's
//!   vocabulary ([`crate::SKETCH3D_CONSTRAINT_TYPES`]). Coordinates and
//!   dimension values may be EXPRESSION strings; they are evaluated against the
//!   document's expression sheet before every solve, exactly as the planar
//!   sketch evaluates its own.
//! - `externalRefs` — the materialization record of the dialog's `references`
//!   list: one `{name, kind, points: [ids]}` per linked model entity, saying
//!   which fixed construction points the editor made for it (`kind` is
//!   `point`, `line`, `arc` or `plane`). On every run each entry is RE-RESOLVED
//!   against the live scene and its points are moved to where the entity now
//!   is, so a 3D sketch follows the model it is constrained to. An entry whose
//!   name no longer resolves keeps its last coordinates and is reported through
//!   `unresolved` (contract rule 1: the caller repairs; nothing vanishes).
//!
//! # What it publishes (all under the feature id)
//!
//! - `scene.paths[{id}:G{gid}]` — one world curve per MODEL geometry (a
//!   construction line, arc or bezier publishes nothing); a line as a degree-1
//!   NURBS, an arc as the exact rational arc in the plane of its three points
//!   (the MINOR arc from start to end), a bezier chain as one clamped cubic
//!   B-spline with triple interior knots.
//! - `scene.paths[{id}]` — the longest head-to-tail chain of those segments,
//!   with `path_segment_names` naming each segment's `{id}:G{gid}`, so a
//!   whole-sketch `path` reference sweeps the run in one pick. A branching
//!   sketch (a frame node) still resolves per segment.
//! - `scene.axes[{id}:G{gid}]` — every line, construction included, as a
//!   revolve/sweep axis.
//! - `scene.points[{id}:P{pid}]` — every point, construction flag included.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{Axis, FeatureContext, FeatureResult, SceneMap, ScenePoint};
use crate::{
    make_arc, make_line, resolve_edge_selection, resolve_face_selection,
    resolve_vertex_selection, NurbsCurve, SelectionGeometry, Sketch3dDoc, Sketch3dSolveOptions,
    Vec3, Vec4,
};
use serde_json::{json, Map, Value};

/// The feature type code (the dispatch and the catalogue key).
pub const SKETCH3D_FEATURE_TYPE: &str = "S3D";

/// Key of the `persistentData` document.
pub const SKETCH3D_DOC_KEY: &str = "sketch3d";

/// Key of the `persistentData` reference materialization record.
pub const SKETCH3D_REFS_KEY: &str = "externalRefs";

/// The dialog's reference list param.
pub const SKETCH3D_REFERENCES_PARAM: &str = "references";

/// The schema button that opens the editor.
pub const SKETCH3D_EDIT_BUTTON: &str = "editSketch3d";

/// The length of the normal tip a resolved PLANE reference materializes
/// (`normalTip = origin + normal · this`), and of the direction tip of a
/// resolved AXIS reference. The solver normalizes directions, so the length
/// only sizes what the editor draws.
pub const REFERENCE_TIP_LENGTH: f64 = 1.0;

/// Record key of a vertex reference's topological anchor: two edge names whose
/// one shared end IS the vertex. A `{solid}@x,y,z` name is only a coordinate,
/// so without this a vertex that moves stops resolving.
pub const REFERENCE_ANCHOR_EDGES_KEY: &str = "anchorEdges";

/// Two edge ends closer than this (relative to the larger coordinate) are one
/// vertex. Loose enough for the editor's single-precision display polylines.
const ANCHOR_END_REL: f64 = 1e-5;

/// Two segment ends closer than this (relative to the sketch's length scale)
/// are the same junction when chaining.
const CHAIN_JOIN_REL: f64 = 1e-7;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(output) => {
            let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
            for entry in output.per_geometry {
                result.paths.push(entry);
            }
            if let Some((chain, names)) = output.chain {
                result.paths.push((ctx.id.clone(), chain));
                result.path_segment_names.push((ctx.id.clone(), names));
            }
            result.axes = output.axes;
            result.points = output.points;
            result.unresolved = output.unresolved;
            result.notes.extend(output.notes);
            result
        }
        Err(error) => ctx.fail(error),
    }
}

pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> Value {
    json!({
        "type": SKETCH3D_FEATURE_TYPE,
        "shortName": SKETCH3D_FEATURE_TYPE,
        "longName": "3D Sketch",
        "ribbonPath": "Home/Reference/3D Sketch",
        "commandSize": "Large",
        "displayBuilder": true,
        "inputParamsSchema": {
            "id": {
                "type": "string",
                "default_value": null,
                "hint": "unique identifier for the 3D sketch feature"
            },
            SKETCH3D_REFERENCES_PARAM: {
                "type": "reference_selection",
                "selectionFilter": ["VERTEX", "EDGE", "FACE", "PLANE"],
                "multiple": true,
                "default_value": null,
                "label": "References",
                "hint": "Model geometry the sketch is linked to: a vertex becomes a fixed point, an edge a fixed line or arc, a face or datum plane a reference plane, a cylinder its axis. Linked in the editor with the Link reference tool, or listed here; re-resolved on every regenerate."
            },
            SKETCH3D_EDIT_BUTTON: {
                "type": "button",
                "label": "Edit 3D Sketch",
                "default_value": null,
                "hint": "open the 3D sketch editor in the viewport"
            }
        }
    })
}

// ---------------------------------------------------------------------------
// Build
// ---------------------------------------------------------------------------

struct Output {
    per_geometry: Vec<(String, Vec<NurbsCurve>)>,
    chain: Option<(Vec<NurbsCurve>, Vec<Option<String>>)>,
    axes: Vec<(String, Axis)>,
    points: Vec<(String, ScenePoint)>,
    unresolved: Vec<String>,
    notes: Vec<String>,
}

fn build(ctx: &FeatureContext) -> Result<Output, String> {
    let mut unresolved: Vec<String> = Vec::new();
    let mut notes: Vec<String> = Vec::new();

    // A brand-new 3D sketch has no document yet (the app seeds an empty
    // `persistentData`), which is a legitimate state: nothing to publish.
    let mut sketch = match ctx.persistent.get(SKETCH3D_DOC_KEY) {
        Some(value) if value.is_object() => value.clone(),
        Some(value) => {
            return Err(format!(
                "3D sketch: `persistentData.{SKETCH3D_DOC_KEY}` must be an object, got {value}"
            ));
        }
        None => {
            // The dialog's references are still validated, so a listed name
            // that does not resolve is reported from the first run.
            for name in common::reference_names(ctx.param(SKETCH3D_REFERENCES_PARAM)) {
                if resolve_reference(ctx.scene, &name).is_none() {
                    unresolved.push(name);
                }
            }
            return Ok(Output {
                per_geometry: Vec::new(),
                chain: None,
                axes: Vec::new(),
                points: Vec::new(),
                unresolved,
                notes,
            });
        }
    };

    pre_evaluate_expressions(ctx, &mut sketch)?;
    let mut doc: Sketch3dDoc = serde_json::from_value(sketch)
        .map_err(|error| format!("3D sketch: invalid document: {error}"))?;

    // Re-resolve the linked references and move their points to where the
    // model entity is now.
    let refs = ctx
        .persistent
        .get(SKETCH3D_REFS_KEY)
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default();
    let mut materialized: std::collections::HashSet<String> = std::collections::HashSet::new();
    for entry in &refs {
        let Some(name) = entry.get("name").and_then(Value::as_str) else {
            continue;
        };
        materialized.insert(name.to_string());
        let kind = entry.get("kind").and_then(Value::as_str).unwrap_or("");
        let ids: Vec<Value> = entry
            .get("points")
            .and_then(Value::as_array)
            .cloned()
            .unwrap_or_default();
        // A vertex with a topological anchor is found through its edges; its
        // coordinate name is stale the moment the vertex moves, and may even
        // name a different vertex by then, so a failed anchor is a lost
        // reference and never falls back to the name.
        let resolved = match anchor_edge_names(entry) {
            Some(anchors) => resolve_anchored_vertex(ctx.scene, &anchors).map(ResolvedReference::Point),
            None => resolve_reference(ctx.scene, name),
        };
        let resolved = resolved.map(|r| r.as_recorded(kind));
        match resolved {
            Some(resolved) if resolved.kind() == kind && resolved.positions().len() == ids.len() => {
                // A plane is (origin, normal) and an axis line (point,
                // direction): any origin ON the plane / line is as good as
                // the one the exact BREP reports, so the editor's own origin
                // is kept, projected onto the resolved entity, and only its
                // tip follows the resolved direction. A regenerate then moves
                // nothing the user placed. A model EDGE (a segment, a circle)
                // is exact like a point and an arc: its ends and centre are
                // what a midpoint or centre constraint is measured from.
                let positions = resolved.positions();
                let positions = match resolved {
                    ResolvedReference::Plane(origin, tip) | ResolvedReference::Line(origin, tip) => {
                        let direction = tip.sub(origin);
                        let editor_origin = ids
                            .first()
                            .and_then(|id| doc.point(id))
                            .map(|p| p.position())
                            .unwrap_or(origin);
                        let kept = match direction.normalized() {
                            Ok(unit) => {
                                let offset = editor_origin.sub(origin);
                                if kind == "plane" {
                                    editor_origin.sub(unit.scale(offset.dot(unit)))
                                } else {
                                    origin.add(unit.scale(offset.dot(unit)))
                                }
                            }
                            Err(_) => origin,
                        };
                        vec![kept, kept.add(direction)]
                    }
                    _ => positions,
                };
                for (id, position) in ids.iter().zip(positions) {
                    if let Some(point) = doc.point_mut(id) {
                        point.x = position.x;
                        point.y = position.y;
                        point.z = position.z;
                        point.fixed = true;
                    }
                }
            }
            // Resolved to a different shape than the editor materialized (an
            // edge that became an arc after a model edit) or not at all: keep
            // the last coordinates and say so.
            _ => {
                notes.push(lost_reference_note(&doc, name, &ids));
                unresolved.push(name.to_string());
            }
        }
    }
    // Names the dialog lists that the editor has not materialized yet are
    // validated too; the editor materializes them on its next entry.
    for name in common::reference_names(ctx.param(SKETCH3D_REFERENCES_PARAM)) {
        if materialized.contains(&name) {
            continue;
        }
        if resolve_reference(ctx.scene, &name).is_none() {
            unresolved.push(name);
        }
    }

    // Solve to final coordinates.
    let diagnostics = crate::solve_sketch3d_doc(&mut doc, &Sketch3dSolveOptions::default())?;
    if diagnostics.conflicting {
        return Err(format!(
            "3D sketch: conflicting constraints ({}): the sketch has no solution near its current geometry",
            diagnostics.conflicting_constraints.join(", ")
        ));
    }

    let scale = doc
        .points
        .iter()
        .fold(1.0f64, |acc, p| acc.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));

    // Publish every point, construction flag included.
    let mut points: Vec<(String, ScenePoint)> = Vec::with_capacity(doc.points.len());
    for point in &doc.points {
        points.push((
            format!("{}:P{}", ctx.id, id_display(&point.id)),
            ScenePoint {
                position: point.position(),
                construction: point.construction,
            },
        ));
    }

    // Curves per model geometry; axes per line.
    let mut per_geometry: Vec<(String, Vec<NurbsCurve>)> = Vec::new();
    let mut axes: Vec<(String, Axis)> = Vec::new();
    let mut segments: Vec<Segment> = Vec::new();
    for geometry in &doc.geometries {
        let name = format!("{}:G{}", ctx.id, id_display(&geometry.id));
        let Some(positions) = doc.geometry_positions(geometry) else {
            return Err(format!(
                "3D sketch: geometry {} references a missing point",
                id_display(&geometry.id)
            ));
        };
        if geometry.geom_type == "line" && positions.len() == 2 {
            if let Ok(direction) = positions[1].sub(positions[0]).normalized() {
                axes.push((name.clone(), Axis { point: positions[0], direction }));
            }
        }
        if geometry.construction() {
            continue;
        }
        let curve = match (geometry.geom_type.as_str(), positions.len()) {
            ("point", _) => continue,
            ("line", 2) => {
                if positions[1].sub(positions[0]).length() <= 1e-9 * scale {
                    return Err(format!("3D sketch: line {} has zero length", id_display(&geometry.id)));
                }
                make_line(positions[0], positions[1])?
            }
            ("arc", 3) => arc_curve(positions[0], positions[1], positions[2], scale)
                .map_err(|error| format!("3D sketch: arc {}: {error}", id_display(&geometry.id)))?,
            ("bezier", n) if n >= 4 && (n - 1) % 3 == 0 => bezier_chain_curve(&positions)?,
            (other, n) => {
                return Err(format!(
                    "3D sketch: geometry {} is a '{other}' with {n} points, which is not a line, arc or bezier",
                    id_display(&geometry.id)
                ));
            }
        };
        let [t0, t1] = curve.domain()?;
        segments.push(Segment {
            name: name.clone(),
            start: curve.evaluate(t0)?,
            end: curve.evaluate(t1)?,
            curve: curve.clone(),
        });
        per_geometry.push((name, vec![curve]));
    }

    let chain = longest_chain(&segments, CHAIN_JOIN_REL * scale);

    Ok(Output {
        per_geometry,
        chain,
        axes,
        points,
        unresolved,
        notes,
    })
}

/// What the run says about a linked reference that stopped resolving: the
/// reference by name and every constraint that depends on it.
fn lost_reference_note(doc: &Sketch3dDoc, name: &str, ids: &[Value]) -> String {
    let needed: Vec<String> = doc
        .constraints
        .iter()
        .filter(|c| c.points().iter().any(|p| ids.iter().any(|id| crate::sketch_id_key(id) == crate::sketch_id_key(p))))
        .map(|c| format!("{} {}", c.ctype().unwrap_or("?"), c.id().map(id_display).unwrap_or_else(|| "?".into())))
        .collect();
    if needed.is_empty() {
        format!("3D sketch: reference '{name}' no longer resolves; it keeps its last position")
    } else {
        format!(
            "3D sketch: reference '{name}' no longer resolves; it keeps its last position, needed by constraint {}",
            needed.join(", ")
        )
    }
}

fn anchor_edge_names(entry: &Value) -> Option<Vec<String>> {
    let names: Vec<String> = entry
        .get(REFERENCE_ANCHOR_EDGES_KEY)?
        .as_array()?
        .iter()
        .filter_map(|n| n.as_str().map(String::from))
        .collect();
    (names.len() == 2).then_some(names)
}

/// The one end two edges share, `None` when they share no end or both ends
/// (two halves of a seamed rim say nothing about which end is meant).
pub fn common_edge_end(a: (Vec3, Vec3), b: (Vec3, Vec3)) -> Option<Vec3> {
    let scale = [a.0, a.1, b.0, b.1]
        .iter()
        .fold(1.0f64, |m, p| m.max(p.x.abs()).max(p.y.abs()).max(p.z.abs()));
    let tol = ANCHOR_END_REL * scale;
    let mut shared: Vec<Vec3> = Vec::new();
    for end in [a.0, a.1] {
        if [b.0, b.1].iter().any(|other| end.sub(*other).length() <= tol) {
            shared.push(end);
        }
    }
    match shared.as_slice() {
        [only] => Some(*only),
        [first, second] if first.sub(*second).length() <= tol => Some(*first),
        _ => None,
    }
}

fn resolve_anchored_vertex(scene: &SceneMap, anchors: &[String]) -> Option<Vec3> {
    let ends = |name: &String| -> Option<(Vec3, Vec3)> {
        let curve = common::edge_curve(scene.resolve_edge(name)?).ok()?;
        let [t0, t1] = curve.domain().ok()?;
        Some((curve.evaluate(t0).ok()?, curve.evaluate(t1).ok()?))
    };
    common_edge_end(ends(anchors.first()?)?, ends(anchors.get(1)?)?)
}

/// A geometry's `{id}:G{id}` / `{id}:P{id}` suffix: a number prints bare, a
/// string prints as itself.
fn id_display(id: &Value) -> String {
    match id {
        Value::String(text) => text.clone(),
        other => crate::sketch_id_key(other),
    }
}

// ---------------------------------------------------------------------------
// Expressions
// ---------------------------------------------------------------------------

fn eval_expr(ctx: &FeatureContext, what: &str, value: &Value) -> Result<Option<f64>, String> {
    let Value::String(source) = value else {
        return Ok(None);
    };
    if source.trim().is_empty() {
        return Ok(None);
    }
    let number = ctx
        .env
        .eval(source)
        .map_err(|error| format!("3D sketch: {what} = `{source}`: {error}"))?;
    if !number.is_finite() {
        return Err(format!("3D sketch: {what} = `{source}` evaluated to {number}"));
    }
    Ok(Some(number))
}

/// Substitute expression strings in point coordinates (`x`/`y`/`z`) and
/// constraint values (`valueExpr`, else a string `value`) with their numbers.
fn pre_evaluate_expressions(ctx: &FeatureContext, sketch: &mut Value) -> Result<(), String> {
    if let Some(points) = sketch.get_mut("points").and_then(Value::as_array_mut) {
        for point in points {
            let id = point.get("id").map(|id| id.to_string()).unwrap_or_else(|| "?".into());
            for key in ["x", "y", "z"] {
                let Some(value) = point.get(key).cloned() else { continue };
                if let Some(number) = eval_expr(ctx, &format!("point {id} `{key}`"), &value)? {
                    point[key] = Value::from(number);
                }
            }
        }
    }
    if let Some(constraints) = sketch.get_mut("constraints").and_then(Value::as_array_mut) {
        for constraint in constraints {
            let id = constraint.get("id").map(|id| id.to_string()).unwrap_or_else(|| "?".into());
            let expr = constraint
                .get("valueExpr")
                .cloned()
                .map(|value| eval_expr(ctx, &format!("constraint {id} `valueExpr`"), &value))
                .transpose()?
                .flatten();
            let evaluated = match expr {
                Some(number) => Some(number),
                None => constraint
                    .get("value")
                    .cloned()
                    .map(|value| eval_expr(ctx, &format!("constraint {id} `value`"), &value))
                    .transpose()?
                    .flatten(),
            };
            if let Some(number) = evaluated {
                constraint["value"] = Value::from(number);
            }
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// References
// ---------------------------------------------------------------------------

/// What a model reference resolves to, in the shapes the sketch materializes.
#[derive(Clone, Debug)]
pub enum ResolvedReference {
    /// A vertex, a published point, a sphere's centre: one fixed point.
    Point(Vec3),
    /// A datum axis or a cylinder's axis, an infinite line in all but name: a
    /// fixed construction line `[a, b]` whose origin is free to slide.
    Line(Vec3, Vec3),
    /// A model EDGE, straight or a best-effort chord: `[start, end]`, exact.
    Segment(Vec3, Vec3),
    /// A closed circular edge: `[center, center + axis * radius]`, exact, so
    /// the centre follows the model and the tip carries the radius.
    CircleAxis(Vec3, Vec3),
    /// A circular arc edge: `[center, start, end]`.
    Arc(Vec3, Vec3, Vec3),
    /// A planar face or a datum/construction plane: `[origin, normalTip]`.
    Plane(Vec3, Vec3),
    /// A free-form edge: the control points of a fitted cubic bezier chain
    /// (see [`curve_reference_controls`]).
    Curve(Vec<Vec3>),
}

/// How densely a free-form edge is sampled for its fit.
const REFERENCE_CURVE_SAMPLES: usize = 128;

/// How many cubic spans a free-form edge is fitted with. Fixed, so a rebuild
/// updates the same control points in place.
pub const REFERENCE_CURVE_SPANS: usize = 8;

/// The control points (`3 * REFERENCE_CURVE_SPANS + 1`) of a C1 cubic bezier
/// chain through samples of a curve, its anchors spaced evenly by arc length.
/// `None` for fewer than two samples or a curve of no length.
pub fn curve_reference_controls(samples: &[Vec3]) -> Option<Vec<Vec3>> {
    if samples.len() < 2 {
        return None;
    }
    let mut cumulative = Vec::with_capacity(samples.len());
    let mut total = 0.0;
    cumulative.push(0.0);
    for pair in samples.windows(2) {
        total += pair[1].sub(pair[0]).length();
        cumulative.push(total);
    }
    if total <= 0.0 {
        return None;
    }
    let spans = REFERENCE_CURVE_SPANS;
    let mut segment = 0;
    let anchors: Vec<Vec3> = (0..=spans)
        .map(|i| {
            let at = total * i as f64 / spans as f64;
            while segment + 2 < samples.len() && cumulative[segment + 1] < at {
                segment += 1;
            }
            let length = cumulative[segment + 1] - cumulative[segment];
            let t = if length > 0.0 { ((at - cumulative[segment]) / length).clamp(0.0, 1.0) } else { 0.0 };
            samples[segment].add(samples[segment + 1].sub(samples[segment]).scale(t))
        })
        .collect();
    let closed = samples[0].sub(samples[samples.len() - 1]).length() <= 1e-9 * total;
    let tangent = |i: usize| -> Vec3 {
        let (before, after) = match i {
            0 if closed => (anchors[spans - 1], anchors[1]),
            0 => (anchors[0], anchors[1]),
            n if n == spans && closed => (anchors[spans - 1], anchors[1]),
            n if n == spans => (anchors[spans - 1], anchors[spans]),
            n => (anchors[n - 1], anchors[n + 1]),
        };
        let scale = if !closed && (i == 0 || i == spans) { 1.0 / 3.0 } else { 1.0 / 6.0 };
        after.sub(before).scale(scale)
    };
    let mut controls = Vec::with_capacity(3 * spans + 1);
    for i in 0..spans {
        controls.push(anchors[i]);
        controls.push(anchors[i].add(tangent(i)));
        controls.push(anchors[i + 1].sub(tangent(i + 1)));
    }
    controls.push(anchors[spans]);
    Some(controls)
}

impl ResolvedReference {
    /// The `kind` the materialization record stores.
    pub fn kind(&self) -> &'static str {
        match self {
            ResolvedReference::Point(..) => "point",
            // A saved record says "line" for all three; the string must not
            // change or re-linking would re-mint the points under constraints.
            ResolvedReference::Line(..) | ResolvedReference::Segment(..) | ResolvedReference::CircleAxis(..) => "line",
            ResolvedReference::Arc(..) => "arc",
            ResolvedReference::Plane(..) => "plane",
            ResolvedReference::Curve(..) => "curve",
        }
    }

    /// As a saved record of `kind` holds it: a free-form edge linked before
    /// curves existed is a `line` chord, and stays one so the points under its
    /// constraints are not re-minted.
    pub fn as_recorded(self, kind: &str) -> ResolvedReference {
        match self {
            ResolvedReference::Curve(controls) if kind == "line" && controls.len() >= 2 => {
                ResolvedReference::Segment(controls[0], controls[controls.len() - 1])
            }
            other => other,
        }
    }

    /// The points it materializes, in record order.
    pub fn positions(&self) -> Vec<Vec3> {
        match *self {
            ResolvedReference::Point(p) => vec![p],
            ResolvedReference::Line(a, b) | ResolvedReference::Segment(a, b) | ResolvedReference::CircleAxis(a, b) => vec![a, b],
            ResolvedReference::Arc(c, s, e) => vec![c, s, e],
            ResolvedReference::Plane(o, n) => vec![o, n],
            ResolvedReference::Curve(ref controls) => controls.clone(),
        }
    }

    /// The construction geometry type the editor draws it with (`None` for a
    /// lone point).
    pub fn geometry_type(&self) -> Option<&'static str> {
        match self {
            ResolvedReference::Point(..) => None,
            ResolvedReference::Line(..) | ResolvedReference::Segment(..) | ResolvedReference::CircleAxis(..) => Some("line"),
            ResolvedReference::Arc(..) => Some("arc"),
            ResolvedReference::Plane(..) => Some("plane"),
            ResolvedReference::Curve(..) => Some("bezier"),
        }
    }
}

/// Resolve a reference name against the live scene: a published point
/// (`{feature}:P{n}`, a waypoint's `Base`), a datum / construction PLANE
/// frame, a published AXIS, a resident FACE (planar → plane, axis-bearing →
/// axis line, sphere → centre), a resident EDGE (straight → line, arc →
/// arc, closed circle → its axis line), or a `{solid}@x,y,z` VERTEX. `None`
/// when nothing carries the name or the entity has no analytic shape.
pub fn resolve_reference(scene: &SceneMap, name: &str) -> Option<ResolvedReference> {
    if let Some(point) = scene.resolve_point(name) {
        return Some(ResolvedReference::Point(point));
    }
    if let Some(frame) = scene.resolve_frame(name) {
        return Some(ResolvedReference::Plane(
            frame.origin,
            frame.origin.add(frame.z_axis.scale(REFERENCE_TIP_LENGTH)),
        ));
    }
    if let Some(axis) = scene.resolve_axis(name) {
        return Some(ResolvedReference::Line(
            axis.point,
            axis.point.add(axis.direction.scale(REFERENCE_TIP_LENGTH)),
        ));
    }
    if let Some(face) = scene.resolve_face(name) {
        let geometry = crate::with_registered_solid_str(face.handle, |solid| {
            resolve_face_selection(solid, face.face_id).map_err(|e| format!("{e:?}"))
        })
        .ok()?;
        return Some(match geometry {
            SelectionGeometry::Plane { origin, normal } => {
                ResolvedReference::Plane(origin, origin.add(normal.scale(REFERENCE_TIP_LENGTH)))
            }
            SelectionGeometry::Axis { origin, direction, .. } => {
                ResolvedReference::Line(origin, origin.add(direction.scale(REFERENCE_TIP_LENGTH)))
            }
            SelectionGeometry::Sphere { center, .. } => ResolvedReference::Point(center),
            _ => return None,
        });
    }
    if let Some(edge) = scene.resolve_edge(name) {
        let curve = common::edge_curve(edge).ok()?;
        let [t0, t1] = curve.domain().ok()?;
        let start = curve.evaluate(t0).ok()?;
        let end = curve.evaluate(t1).ok()?;
        let geometry = crate::with_registered_solid_str(edge.handle, |solid| {
            resolve_edge_selection(solid, edge.edge_id).map_err(|e| format!("{e:?}"))
        });
        return Some(match geometry {
            Ok(SelectionGeometry::Circle { center, axis, radius }) => {
                if end.sub(start).length() <= 1e-9 * radius.max(1.0) {
                    // A closed circle: its axis is what a pipe lines up with.
                    ResolvedReference::CircleAxis(center, center.add(axis.scale(radius)))
                } else {
                    ResolvedReference::Arc(center, start, end)
                }
            }
            Ok(SelectionGeometry::Line { .. }) => ResolvedReference::Segment(start, end),
            // A spline edge or anything without an analytic frame: a fitted
            // bezier chain, or its chord if the curve will not sample.
            _ => {
                let samples: Option<Vec<Vec3>> = (0..=REFERENCE_CURVE_SAMPLES)
                    .map(|i| curve.evaluate(t0 + (t1 - t0) * i as f64 / REFERENCE_CURVE_SAMPLES as f64).ok())
                    .collect();
                match samples.as_deref().and_then(curve_reference_controls) {
                    Some(controls) => ResolvedReference::Curve(controls),
                    None => ResolvedReference::Segment(start, end),
                }
            }
        });
    }
    if let Some((solid_name, coords)) = name.rsplit_once('@') {
        let mut parts = coords.split(',').map(|p| p.trim().parse::<f64>().ok());
        let position = Vec3::new(parts.next()??, parts.next()??, parts.next()??);
        if parts.next().is_some() {
            return None;
        }
        let handle = scene.resolve_solid(solid_name)?;
        let geometry = crate::with_registered_solid_str(handle, |solid| {
            resolve_vertex_selection(solid, position).map_err(|e| format!("{e:?}"))
        })
        .ok()?;
        if let SelectionGeometry::Point { position } = geometry {
            return Some(ResolvedReference::Point(position));
        }
    }
    None
}

// ---------------------------------------------------------------------------
// Curves
// ---------------------------------------------------------------------------

/// The exact rational arc from `start` to `end` about `center`, in the plane
/// of the three points — the MINOR arc (a sweep below π). Collinear points
/// have no plane and are refused.
pub fn arc_curve(center: Vec3, start: Vec3, end: Vec3, scale: f64) -> Result<NurbsCurve, String> {
    let rs = start.sub(center);
    let re = end.sub(center);
    let radius = rs.length();
    if radius <= 1e-9 * scale {
        return Err("zero radius".into());
    }
    let normal = rs.cross(re);
    if normal.length() <= 1e-9 * radius * radius {
        return Err("center, start and end are collinear (no arc plane)".into());
    }
    let x_axis = rs.normalized()?;
    let y_axis = normal.normalized()?.cross(x_axis).normalized()?;
    // The solver holds the radii equal; the end angle is measured on the
    // start radius so a residual difference never changes the sweep.
    let sweep = re.dot(y_axis).atan2(re.dot(x_axis));
    let sweep = if sweep <= 0.0 { sweep + std::f64::consts::TAU } else { sweep };
    make_arc(center, x_axis, y_axis, radius, 0.0, sweep)
}

/// One clamped degree-3 B-spline through a chain of cubic Bézier spans
/// (`3n+1` control points), interior knots of multiplicity 3 — the planar
/// sketch's `bezier_chain_curve`, in world space.
pub fn bezier_chain_curve(controls: &[Vec3]) -> Result<NurbsCurve, String> {
    let span_count = (controls.len().saturating_sub(1)) / 3;
    if span_count == 0 || controls.len() != span_count * 3 + 1 {
        return Err(format!(
            "3D sketch: bezier needs 3n+1 control points, got {}",
            controls.len()
        ));
    }
    let mut knots = vec![0.0; 4];
    for join in 1..span_count {
        knots.extend_from_slice(&[join as f64; 3]);
    }
    knots.extend_from_slice(&[span_count as f64; 4]);
    let points = controls.iter().map(|p| Vec4::from_point(*p, 1.0)).collect();
    NurbsCurve::new(3, knots, points)
}

// ---------------------------------------------------------------------------
// Chaining
// ---------------------------------------------------------------------------

struct Segment {
    name: String,
    start: Vec3,
    end: Vec3,
    curve: NurbsCurve,
}

/// The longest head-to-tail chain through the model segments (segments
/// reversed as needed), with each curve's source name. Segments are joined
/// where an end of one lies within `tol` of an end of another; a junction
/// where three or more ends meet (a frame node) ends the chain there. `None`
/// when there are no segments.
fn longest_chain(segments: &[Segment], tol: f64) -> Option<(Vec<NurbsCurve>, Vec<Option<String>>)> {
    if segments.is_empty() {
        return None;
    }
    let close = |a: Vec3, b: Vec3| a.sub(b).length() <= tol;
    // Ends meeting at a point: (segment, is_end) per end.
    let degree_at = |p: Vec3| -> usize {
        segments
            .iter()
            .map(|s| usize::from(close(s.start, p)) + usize::from(close(s.end, p)))
            .sum()
    };
    let mut best: Option<Vec<(usize, bool)>> = None;
    // Grow a chain from every segment in both orientations; keep the longest.
    for (seed, _) in segments.iter().enumerate() {
        for seed_reversed in [false, true] {
            let mut chain: Vec<(usize, bool)> = vec![(seed, seed_reversed)];
            let mut used = vec![false; segments.len()];
            used[seed] = true;
            // Extend from the tail.
            loop {
                let (last, reversed) = *chain.last().unwrap();
                let tail = if reversed { segments[last].start } else { segments[last].end };
                if degree_at(tail) != 2 {
                    break;
                }
                let next = segments.iter().enumerate().find_map(|(i, s)| {
                    if used[i] {
                        return None;
                    }
                    if close(s.start, tail) {
                        Some((i, false))
                    } else if close(s.end, tail) {
                        Some((i, true))
                    } else {
                        None
                    }
                });
                match next {
                    Some((i, reversed)) => {
                        used[i] = true;
                        chain.push((i, reversed));
                    }
                    None => break,
                }
            }
            // Extend from the head.
            loop {
                let (first, reversed) = chain[0];
                let head = if reversed { segments[first].end } else { segments[first].start };
                if degree_at(head) != 2 {
                    break;
                }
                let prev = segments.iter().enumerate().find_map(|(i, s)| {
                    if used[i] {
                        return None;
                    }
                    if close(s.end, head) {
                        Some((i, false))
                    } else if close(s.start, head) {
                        Some((i, true))
                    } else {
                        None
                    }
                });
                match prev {
                    Some((i, reversed)) => {
                        used[i] = true;
                        chain.insert(0, (i, reversed));
                    }
                    None => break,
                }
            }
            if best.as_ref().map_or(true, |b| chain.len() > b.len()) {
                best = Some(chain);
            }
        }
    }
    let chain = best?;
    let mut curves = Vec::with_capacity(chain.len());
    let mut names = Vec::with_capacity(chain.len());
    for (i, reversed) in chain {
        let curve = if reversed {
            segments[i].curve.reversed().ok()?
        } else {
            segments[i].curve.clone()
        };
        curves.push(curve);
        names.push(Some(segments[i].name.clone()));
    }
    Some((curves, names))
}

/// A materialization record entry for a resolved reference — the editor's
/// side of [`resolve_reference`] (kept here so the record's shape has one
/// author).
pub fn reference_record(name: &str, resolved: &ResolvedReference, point_ids: &[Value], geometry_id: Option<&Value>) -> Value {
    let mut map = Map::new();
    map.insert("name".into(), Value::String(name.to_string()));
    map.insert("kind".into(), Value::String(resolved.kind().to_string()));
    map.insert("points".into(), Value::Array(point_ids.to_vec()));
    map.insert(
        "geometry".into(),
        geometry_id.cloned().unwrap_or(Value::Null),
    );
    Value::Object(map)
}

