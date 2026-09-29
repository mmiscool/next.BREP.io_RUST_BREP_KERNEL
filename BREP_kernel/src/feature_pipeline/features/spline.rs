//! SP — Spline. The display builder
//! draws a Hermite spline through the authored anchor points and names the edge
//! `{id}:SplineEdge`; downstream features (tube/sweep/path-sweep) reference that
//! name as a path. This kernel feature is the headless twin: it reconstructs the
//! SAME curve EXACTLY and publishes it into the scene (`scene.paths` under both
//! `{id}` and `{id}:SplineEdge`, plus every anchor as `{id}:P{index}` in
//! `scene.points`, mirroring the vertex names). It produces NO solid —
//! SP stays a display builder.
//!
//! # Curve semantics (the display truth)
//!
//! `persistentData.spline.points[]` are THROUGH-points (the curve interpolates
//! them): `{ id, position:[3], rotation:[9] (row-flat 3×3, rows = x/y/z axes),
//! forwardDistance, backwardDistance, flipDirection, attachment }`. Each adjacent
//! pair `(i, i+1)` contributes three sub-segments:
//!   1. STRAIGHT `anchorᵢ → fwdExtᵢ` where `fwdExtᵢ = anchorᵢ + dirᵢ·forwardᵢ`
//!      and `dirᵢ = rotation[0..3]` (the X-axis, used AS-IS — the caller never
//!      re-normalizes it), sign-flipped when `flipDirection`.
//!   2. CUBIC HERMITE `fwdExtᵢ → backExtᵢ₊₁` (with `backExtᵢ₊₁ = anchorᵢ₊₁ −
//!      dirᵢ₊₁·backwardᵢ₊₁`) with tangents `t0 = norm(fwdExtᵢ − anchorᵢ)·s`,
//!      `t1 = norm(anchorᵢ₊₁ − backExtᵢ₊₁)·s`, where `s = max(|fwdExtᵢ −
//!      backExtᵢ₊₁|·0.3, (forwardᵢ + backwardᵢ₊₁)·0.5·0.5) · clamp(bendRadius,
//!      0.1, 5.0)` (`bendRadius` defaults to 1 when non-finite/missing). Normalizing
//!      maps the zero vector to zero (dividing by `length || 1`),
//!      so a zero extension yields a zero tangent, not NaN.
//!   3. STRAIGHT `backExtᵢ₊₁ → anchorᵢ₊₁`.
//!
//! The display polyline merely SAMPLES this piecewise curve (24 per span, split
//! 15%/70%/15%); the kernel publishes the analytic pieces instead: each straight
//! is a degree-1 NURBS line, each Hermite span is the EXACT equivalent cubic
//! Bézier `[p0, p0 + t0/3, p1 − t1/3, p1]` (identical parametrization, so
//! sampling the kernel curve at the display parameters reproduces the display points).
//! Degenerate (zero-length) pieces are dropped so the chain stays consumable.
//!
//! Normalization mirrors `normalizeSplineData`: fewer than two points falls back
//! to the default pair `(0,0,0) → (5,0,0)` (identity rotation, 1.0 extensions);
//! a malformed rotation falls back to identity; a non-number distance to 1.0.
//!
//! # Port attachments (the wire-harness network)
//!
//! A point may carry `attachment: { portRef, side }` — the ADDRESS of a
//! connection point (`J1.VCC`, `ACOMP3:J1.VCC`, or a WAYPOINT's feature id; see
//! `feature_pipeline/ports.rs`) and the side (`A` | `B`) the wire leaves it on.
//! Such an anchor is
//! LIVE-RESOLVED against the scene's port records on every run: its position
//! is the port's base point, its travel direction the port direction (negated
//! for side `B`), and its forward/backward distance the port's `extension`.
//! Move the port and the spline follows; the persisted `position` / `rotation`
//! are only what the editor last wrote (a stale snapshot, never authoritative).
//! `flipDirection` is ignored for an attached anchor — the side already fixes
//! the direction. A `portRef` naming no port is an `unresolved` entry (contract
//! rule 1) and the persisted values are used as-is.
//!
//! An anchor's attachment is what makes the spline a harness SEGMENT: a spline
//! whose FIRST and LAST anchors attach to two different ports joins those ports
//! in the routing graph (see `wire_harness.rs`).

use crate::feature_pipeline::wire_harness::Attachment;
use crate::feature_pipeline::{FeatureContext, FeatureResult, SceneMap};
use crate::{make_line, NurbsCurve, Vec3, Vec4};
use serde_json::Value;

/// Zero-length gate for dropping degenerate chain pieces (a zero `forwardDistance`
/// collapses its straight piece; the retired builder just emitted duplicated sample points there).
const DEGENERATE_EPS: f64 = 1e-12;

/// One normalized through-point (`ensurePoint` semantics, see the module doc).
pub(crate) struct SplinePoint {
    pub(crate) position: Vec3,
    /// The point's travel direction: the rotation X-axis (`rotation[0..3]`),
    /// sign-flipped when `flipDirection`; for an attached anchor the port's
    /// side direction. Used AS-IS (never re-normalized upstream).
    pub(crate) direction: Vec3,
    pub(crate) forward: f64,
    pub(crate) backward: f64,
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
    // Naming: `featureId = inputParams?.featureID ? String(...) : "Spline"`.
    let base = if ctx.id.is_empty() { "Spline" } else { ctx.id.as_str() };
    let (anchors, unresolved) = parse_spline_points(ctx.persistent, ctx.scene);
    result.unresolved = unresolved;
    match build_chain(&anchors, bend_radius(ctx)) {
        Ok(chain) if !chain.is_empty() => {
            // Publish under BOTH the feature id and the display-edge name
            // (exact-match resolution, contract rule 1) so `tube`/`sweep`
            // path references resolve either spelling.
            result.paths.push((base.to_string(), chain.clone()));
            result.paths.push((format!("{base}:SplineEdge"), chain));
        }
        Ok(_) => {} // fully degenerate spline: no path, still a pass-through
        Err(error) => return ctx.fail(format!("spline: {error}")),
    }
    // Every anchor is published twice: as the point `{id}:P{index}` (the
    // drawn vertex; a helix / hole placement can start there) and as the axis
    // `{id}:P{index}` through it along its travel direction — what the anchor
    // editor draws its direction cage from, and a revolve / sweep can reference.
    // The axis direction is the anchor's RESOLVED direction (a port's, for an
    // attached anchor), so the editor shows what the curve actually does.
    for (index, anchor) in anchors.iter().enumerate() {
        let name = format!("{base}:P{index}");
        result
            .points
            .push((name.clone(), crate::feature_pipeline::ScenePoint::model(anchor.position)));
        if let Ok(direction) = anchor.direction.normalized() {
            result.axes.push((
                name,
                crate::feature_pipeline::Axis {
                    point: anchor.position,
                    direction,
                },
            ));
        }
    }
    result
}

// ---------------------------------------------------------------------------
// persistentData.spline parsing (normalizeSplineData / ensurePoint semantics)
// ---------------------------------------------------------------------------

/// The normalized through-points plus the attachment `portRef`s that named no
/// port (reported as `unresolved`).
pub(crate) fn parse_spline_points(persistent: &Value, scene: &SceneMap) -> (Vec<SplinePoint>, Vec<String>) {
    let raw = persistent
        .get("spline")
        .and_then(|spline| spline.get("points"))
        .and_then(Value::as_array);
    let mut unresolved = Vec::new();
    let points = match raw {
        Some(list) if list.len() >= 2 => list
            .iter()
            .map(|point| parse_point(point, scene, &mut unresolved))
            .collect(),
        // `normalizeSplineData`: missing/short point list → the default pair.
        _ => vec![
            SplinePoint {
                position: Vec3::new(0.0, 0.0, 0.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
                forward: 1.0,
                backward: 1.0,
            },
            SplinePoint {
                position: Vec3::new(5.0, 0.0, 0.0),
                direction: Vec3::new(1.0, 0.0, 0.0),
                forward: 1.0,
                backward: 1.0,
            },
        ],
    };
    (points, unresolved)
}

fn parse_point(point: &Value, scene: &SceneMap, unresolved: &mut Vec<String>) -> SplinePoint {
    // An attached anchor is the connection point's placement, live (module doc).
    if let Some(attachment) = Attachment::parse(point.get("attachment")) {
        match scene.resolve_port(&attachment.port_ref) {
            Some(port) => {
                return SplinePoint {
                    position: port.point,
                    direction: port.side_direction(attachment.side),
                    forward: port.extension,
                    backward: port.extension,
                };
            }
            None => unresolved.push(attachment.port_ref.clone()),
        }
    }
    let mut direction = rotation_x_axis(point.get("rotation"));
    if point
        .get("flipDirection")
        .and_then(Value::as_bool)
        .unwrap_or(false)
    {
        direction = direction.scale(-1.0);
    }
    SplinePoint {
        position: read_vec3(point.get("position")),
        direction,
        forward: distance_or_default(point.get("forwardDistance")),
        backward: distance_or_default(point.get("backwardDistance")),
    }
}

/// `Number(v) || 0` per component (coerces numeric strings; anything else → 0).
fn read_vec3(value: Option<&Value>) -> Vec3 {
    let component = |index: usize| -> f64 {
        value
            .and_then(Value::as_array)
            .and_then(|array| array.get(index))
            .and_then(|entry| match entry {
                Value::Number(number) => number.as_f64(),
                Value::String(text) => text.trim().parse::<f64>().ok(),
                _ => None,
            })
            .filter(|number| number.is_finite())
            .unwrap_or(0.0)
    };
    Vec3::new(component(0), component(1), component(2))
}

/// The rotation matrix X-axis (`rotation[0..3]`): a 9-entry all-numeric array is
/// used as-is; anything else falls back to the identity X-axis `(1,0,0)`.
fn rotation_x_axis(value: Option<&Value>) -> Vec3 {
    let identity = Vec3::new(1.0, 0.0, 0.0);
    let Some(array) = value.and_then(Value::as_array) else {
        return identity;
    };
    if array.len() != 9 {
        return identity;
    }
    let axis: Vec<f64> = array
        .iter()
        .take(3)
        .filter_map(Value::as_f64)
        .filter(|number| number.is_finite())
        .collect();
    match axis.as_slice() {
        [x, y, z] => Vec3::new(*x, *y, *z),
        _ => identity,
    }
}

/// `typeof v === "number" ? Math.max(0, v) : 1.0` (only a JSON number counts).
fn distance_or_default(value: Option<&Value>) -> f64 {
    value
        .and_then(Value::as_f64)
        .map(|number| number.max(0.0))
        .unwrap_or(1.0)
}

/// `bendRadius`: finite → clamped to `[0.1, 5.0]`, anything else → 1.0
/// (the pipeline additionally accepts an expression
/// string, evaluated against the shared env like every numeric param).
fn bend_radius(ctx: &FeatureContext) -> f64 {
    match ctx.param("bendRadius") {
        None | Some(Value::Null) => 1.0,
        Some(_) => match ctx.number("bendRadius") {
            Ok(value) if value.is_finite() => value.clamp(0.1, 5.0),
            _ => 1.0,
        },
    }
}

// ---------------------------------------------------------------------------
// Exact chain construction (buildHermitePolyline's analytic pieces)
// ---------------------------------------------------------------------------

fn build_chain(points: &[SplinePoint], bend: f64) -> Result<Vec<NurbsCurve>, String> {
    let mut chain: Vec<NurbsCurve> = Vec::new();
    for pair in points.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let forward_ext = a.position.add(a.direction.scale(a.forward));
        let backward_ext = b.position.sub(b.direction.scale(b.backward));
        push_line(&mut chain, a.position, forward_ext)?;
        push_hermite_span(&mut chain, a, b, forward_ext, backward_ext, bend)?;
        push_line(&mut chain, backward_ext, b.position)?;
    }
    Ok(chain)
}

fn push_line(chain: &mut Vec<NurbsCurve>, start: Vec3, end: Vec3) -> Result<(), String> {
    if end.sub(start).length() <= DEGENERATE_EPS {
        return Ok(()); // zero extension distance — the retired builder emitted duplicate samples here
    }
    chain.push(make_line(start, end)?);
    Ok(())
}

/// The Hermite span `p0 → p1` as its EXACT cubic Bézier `[p0, p0 + t0/3,
/// p1 − t1/3, p1]` (same parametrization: `B(t) ≡ H(t)` for `t ∈ [0,1]`).
fn push_hermite_span(
    chain: &mut Vec<NurbsCurve>,
    a: &SplinePoint,
    b: &SplinePoint,
    p0: Vec3,
    p1: Vec3,
    bend: f64,
) -> Result<(), String> {
    let ext_distance = p0.sub(p1).length();
    let avg_ext_distance = (a.forward + b.backward) * 0.5;
    let tangent_scale = (ext_distance * 0.3).max(avg_ext_distance * 0.5) * bend;
    let t0 = normalize_or_zero(p0.sub(a.position)).scale(tangent_scale);
    let t1 = normalize_or_zero(b.position.sub(p1)).scale(tangent_scale);
    if ext_distance <= DEGENERATE_EPS
        && t0.length() <= DEGENERATE_EPS
        && t1.length() <= DEGENERATE_EPS
    {
        return Ok(()); // coincident extensions with no tangents: a zero-length span
    }
    let controls = vec![
        Vec4::from_point(p0, 1.0),
        Vec4::from_point(p0.add(t0.scale(1.0 / 3.0)), 1.0),
        Vec4::from_point(p1.sub(t1.scale(1.0 / 3.0)), 1.0),
        Vec4::from_point(p1, 1.0),
    ];
    chain.push(NurbsCurve::new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        controls,
    )?);
    Ok(())
}

/// Vector normalization dividing by `length || 1` — the zero vector
/// stays zero (this is what turns a zero extension into a zero Hermite tangent).
fn normalize_or_zero(vector: Vec3) -> Vec3 {
    let length = vector.length();
    if length == 0.0 {
        vector
    } else {
        vector.scale(1.0 / length)
    }
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered — a spline has no reference field to seed from a selection
/// (its anchors attach to ports through the anchor editor).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SP",
    "shortName": "SP",
    "longName": "Spline",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the spline feature"
        },
        "bendRadius": {
            "type": "number",
            "default_value": 1,
            "label": "Bend radius",
            "hint": "Controls the smoothness of curve transitions. Lower values create sharper bends, higher values create smoother curves.",
            "min": 0.1,
            "step": 0.5
        },
        "splinePoints": {
            "type": "spline_points",
            "label": "Anchors",
            "hint": "The through-points, edited in the anchor editor: position, travel direction, forward/backward straight run, and an optional port attachment (side A or B)."
        }
    }
})
}

