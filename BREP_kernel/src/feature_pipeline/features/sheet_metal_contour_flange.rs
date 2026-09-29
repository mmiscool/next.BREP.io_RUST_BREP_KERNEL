//! SM.CF sweeps a centered sheet cross-section along an open planar path.
//! Straight segments become flat walls; their corners become bends with
//! setback `Rm * tan(|turn| / 2)`. Circular arcs become exact rolled bends
//! whose mid-surface radius equals the path radius. The resulting [`SheetTree`]
//! uses the shared extrude/revolve/boolean evaluator.
//!
//! Walls extend along the sketch frame's +Z, or −Z with `reverseSheetSide`.
//! Paths from solid edges infer +Z from the first noncollinear turn; a straight
//! path without a sketch frame cannot determine a sheet plane and is rejected.
//!
//! The sheet tree requires a flat wall on each side of a bend:
//! - Paths must be open, planar, connected, and unbranched.
//! - Segments must be lines or circular arcs, starting and ending with lines.
//! - Consecutive arcs require an intervening straight segment, and arc
//!   junctions must be tangent; arc-corner setbacks are not representable.
//! - Arc radii must exceed half the thickness to avoid self-intersection.
//!
//! The path sketch is consumed on success.

use std::f64::consts::{PI, TAU};

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::sheet_metal::{self, Bend, Edge, Flat, SheetTree};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{NurbsCurve, Vec3};

/// Junction-turn tolerance (radians, ~0.006°): a turn within it is
/// tangent/collinear (elements join smoothly — collinear walls merge, arcs
/// count as tangent), beyond it is a genuine corner. Far above solver noise on
/// a tangency constraint, far below any deliberate corner.
const TURN_TOL: f64 = 1e-4;
/// Length floor (mm) for degenerate segments / over-trimmed walls.
const LEN_TOL: f64 = 1e-6;
/// Samples per path curve for classification (line vs circular arc vs free-form).
const SAMPLES: usize = 9;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // Resolve the flange centerline from the referenced sketch/edge path(s).
    let names = common::reference_names(ctx.param("path"));
    if names.is_empty() {
        return Err("sheet-metal contour flange: missing `path` reference selection".into());
    }
    let mut curves: Vec<NurbsCurve> = Vec::new();
    for name in &names {
        let mut chain = common::resolve_path(ctx, name)
            .map_err(|error| format!("sheet-metal contour flange: {error}"))?;
        curves.append(&mut chain);
    }
    if curves.is_empty() {
        return Err("sheet-metal contour flange: resolved path has no curves".into());
    }

    // The head-to-tail world vertex chain (verifies connectivity), the path's
    // characteristic scale for relative tolerances, and the open-path guard.
    let world = chain_world_points(&curves)?;
    let scale = world
        .iter()
        .map(|p| p.sub(world[0]).length())
        .fold(1.0_f64, f64::max);
    if world.len() >= 3 && world[0].sub(world[world.len() - 1]).length() <= 1e-6 * scale {
        return Err(
            "sheet-metal contour flange: a closed path is not supported — the wrap-around seam \
             bend's child would have to be the root flat, a cycle the rooted sheet tree cannot \
             represent; leave the profile open at the seam vertex instead"
                .into(),
        );
    }

    // Sketch plane: the published sketch frame (deterministic +Z), else the
    // first-turn fallback for frameless (solid-edge) paths. `reverseSheetSide`
    // extends the sheet along −Z instead of +Z.
    let reverse = ctx.param("reverseSheetSide").and_then(|v| v.as_bool()) == Some(true);
    let (plane_origin, base_normal) = match common::sketch_plane_frame(ctx, &names) {
        Some(frame) => (frame.origin, frame.z_axis.normalized()?),
        None => (world[0], first_turn_normal(&world)?),
    };
    let normal = if reverse {
        base_normal.scale(-1.0)
    } else {
        base_normal
    };

    // In-plane 2D basis: x along the first chord (projected), y = normal × x.
    let chord = world[1].sub(world[0]);
    let in_plane = chord.sub(normal.scale(chord.dot(normal)));
    let x_axis = in_plane.normalized().map_err(|_| {
        "sheet-metal contour flange: the first path segment is perpendicular to the sketch plane"
            .to_string()
    })?;
    let y_axis = normal.cross(x_axis);

    let distance = ctx.number("distance").unwrap_or(20.0);
    if !(distance > 0.0) {
        return Err(format!(
            "sheet-metal contour flange: distance must be positive, got {distance}"
        ));
    }
    let thickness = ctx.number("thickness")?;
    if !(thickness > 0.0) {
        return Err("sheet-metal contour flange: thickness must be positive".into());
    }
    // The FALLBACK must be the schema's declared default (2), not 0: the dialog
    // shows 2, so a document that omits the field means "the default", and a
    // 0-radius fallback silently turned that into a SHARP bend — a different
    // part, not a different tolerance. `max(0.0)` still floors a negative.
    let bend_radius = ctx.number("bendRadius").unwrap_or(2.0).max(0.0);
    let k_factor = ctx.number("neutralFactor").unwrap_or(0.5).clamp(0.0, 1.0);

    // Classify every path curve as a straight line or a circular arc (in the
    // sketch plane), merge collinear straight runs, validate the grammar, and
    // plan the walls + bends.
    let mut elems = Vec::with_capacity(curves.len());
    for (index, curve) in curves.iter().enumerate() {
        elems.push(classify_segment(
            curve,
            index,
            plane_origin,
            x_axis,
            y_axis,
            normal,
            scale,
        )?);
    }
    let elems = merge_collinear_lines(elems);
    validate_grammar(&elems, thickness)?;
    let (flat_lens, bend_specs) = plan_walls(&elems, thickness, bend_radius)?;

    // Sketch plane → the root flat's world frame: local x = first wall
    // direction, local y = plane normal (the wall-height direction), local
    // z = x × y (the through-thickness direction; the sheet is centered).
    let t0 = elems[0].start_tan;
    let seg0_world = x_axis.scale(t0[0]).add(y_axis.scale(t0[1])).normalized()?;
    let p0 = world[0];
    let w = seg0_world.cross(normal);
    let root_transform = [
        [p0.x, p0.y, p0.z],
        [seg0_world.x, seg0_world.y, seg0_world.z],
        [normal.x, normal.y, normal.z],
        [w.x, w.y, w.z],
    ];

    // Build the wall chain from the last flat backward, nesting each bend.
    let make_flat = |i: usize, len: f64| Flat {
        id: format!("{}:flat_{i}", ctx.id),
        outline: vec![[0.0, 0.0], [len, 0.0], [len, distance], [0.0, distance]],
        edges: (0..4)
            .map(|k| Edge {
                id: format!("{}:f{i}e{k}", ctx.id),
                bend: None,
            })
            .collect(),
        holes: Vec::new(),
        hole_bends: Vec::new(),
        outline_curves: Default::default(),
    };
    let last = flat_lens.len() - 1;
    let mut node = make_flat(last, flat_lens[last]);
    for i in (0..last).rev() {
        let mut flat = make_flat(i, flat_lens[i]);
        let (angle_deg, inside_radius) = bend_specs[i];
        // Edge 1 ([len,0]->[len,distance]) is the far hinge, running along the
        // wall height; the bend continues the chain into the next wall.
        flat.edges[1].bend = Some(Bend {
            id: format!("{}:bend_{}", ctx.id, i + 1),
            angle_deg,
            inside_radius,
            k_factor,
            child: Box::new(node),
        });
        node = flat;
    }

    let tree = SheetTree {
        thickness,
        root: node,
        default_inside_radius: bend_radius,
        default_k_factor: k_factor,
        root_transform,
    };

    let added = sheet_metal::register_folded(tree, &ctx.id)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.added.push(added);
    // Sheet-metal contract: the path sketch is ALWAYS consumed on success.
    for name in &names {
        common::consume_sketch_always(&common::sketch_base_name(ctx, name), &mut result);
    }
    Ok(result)
}

// ===========================================================================
// Plane + reference resolution
// ===========================================================================

/// Fallback plane normal for a FRAMELESS path (non-sketch solid edges): the
/// first non-collinear turn's cross product. A frameless STRAIGHT path errors
/// loudly — its sheet plane is genuinely unrecoverable. (Sketch paths never
/// get here: the SKETCH feature publishes its plane frame into `scene.frames`.)
fn first_turn_normal(points: &[Vec3]) -> Result<Vec3, String> {
    if points.len() >= 3 {
        let seg0 = points[1].sub(points[0]);
        for k in 1..points.len() - 1 {
            let seg = points[k + 1].sub(points[k]);
            let cross = seg0.cross(seg);
            if cross.length() > 1e-9 {
                return cross.normalized();
            }
        }
    }
    Err(
        "sheet-metal contour flange: a straight path built from non-sketch edges cannot define \
         the sheet plane (no sketch frame is published for it); pick a sketch path or include a \
         turn"
            .into(),
    )
}

/// The head-to-tail world vertex chain of an ordered curve list: the first curve's
/// start, then each curve's end. Verifies every curve starts where the previous one
/// ended (no-fallback on a branching / disconnected chain).
fn chain_world_points(curves: &[NurbsCurve]) -> Result<Vec<Vec3>, String> {
    let first = &curves[0];
    let domain = first.domain()?;
    let mut points = vec![first.evaluate(domain[0])?];
    let mut prev_end = first.evaluate(domain[1])?;
    points.push(prev_end);
    for curve in &curves[1..] {
        let domain = curve.domain()?;
        let start = curve.evaluate(domain[0])?;
        let end = curve.evaluate(domain[1])?;
        if start.sub(prev_end).length() > 1e-6 * prev_end.length().max(1.0) {
            return Err(
                "sheet-metal contour flange: path segments are not connected head-to-tail".into(),
            );
        }
        points.push(end);
        prev_end = end;
    }
    Ok(points)
}

// ===========================================================================
// Path elements — classification, grammar, wall planning
// ===========================================================================

/// One path element in sketch-plane 2D, with its endpoint TANGENTS for
/// junction classification (tangency / corner / reversal).
struct PathElement {
    start_tan: [f64; 2],
    end_tan: [f64; 2],
    kind: ElementKind,
}

enum ElementKind {
    /// Straight segment → one flat wall of this length.
    Line { len: f64 },
    /// Circular arc → a rolled bend: mid-surface radius = `radius` (the sheet
    /// is centered on the path), SIGNED `sweep` in radians (+ = CCW in the
    /// sketch (x, y) plane, which folds toward the sheet's local left — the
    /// same sign convention as a corner turn).
    Arc { radius: f64, sweep: f64 },
}

/// Signed 2D turn from direction `a` to direction `b` (radians).
fn signed_turn(a: [f64; 2], b: [f64; 2]) -> f64 {
    let cross = a[0] * b[1] - a[1] * b[0];
    let dot = a[0] * b[0] + a[1] * b[1];
    cross.atan2(dot)
}

fn free_form_error(index: usize) -> String {
    format!(
        "sheet-metal contour flange: path segment {index} is neither a straight line nor a \
         circular arc — a free-form curve cannot be swept exactly with extrude/revolve kernel \
         ops; replace it with lines and tangent arcs"
    )
}

/// Classify one path curve as a [`PathElement`] in the sketch plane: sample it,
/// verify planarity, then straight-line test first (a shallow huge-radius arc
/// within the 1e-6 exactness floor IS a line), then an exact circle fit
/// (start/mid/end circumcircle, every sample on it, monotone angular sweep).
/// Anything else is a loud free-form error.
fn classify_segment(
    curve: &NurbsCurve,
    index: usize,
    plane_origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    normal: Vec3,
    scale: f64,
) -> Result<PathElement, String> {
    let domain = curve.domain()?;
    let mut samples = [[0.0_f64; 2]; SAMPLES];
    for (i, uv) in samples.iter_mut().enumerate() {
        let t = domain[0] + (domain[1] - domain[0]) * (i as f64) / ((SAMPLES - 1) as f64);
        let point = curve.evaluate(t)?;
        let delta = point.sub(plane_origin);
        if delta.dot(normal).abs() > 1e-6 * scale {
            return Err(format!(
                "sheet-metal contour flange: path segment {index} leaves the sketch plane \
                 (non-planar paths are not supported)"
            ));
        }
        *uv = [delta.dot(x_axis), delta.dot(y_axis)];
    }
    let a = samples[0];
    let b = samples[SAMPLES - 1];
    let chord = [b[0] - a[0], b[1] - a[1]];
    let chord_len = chord[0].hypot(chord[1]);
    if chord_len <= LEN_TOL {
        return Err(format!(
            "sheet-metal contour flange: path segment {index} is degenerate (zero-length)"
        ));
    }

    // Straight? Every sample within 1e-6 (relative) of the chord line.
    let dir = [chord[0] / chord_len, chord[1] / chord_len];
    let straight = samples.iter().all(|p| {
        let dx = p[0] - a[0];
        let dy = p[1] - a[1];
        (dx * dir[1] - dy * dir[0]).abs() <= 1e-6 * chord_len.max(1.0)
    });
    if straight {
        return Ok(PathElement {
            start_tan: dir,
            end_tan: dir,
            kind: ElementKind::Line { len: chord_len },
        });
    }

    // Circular? Circumcircle of start/mid/end, verified against every sample.
    let mid = samples[SAMPLES / 2];
    let Some(center) = circumcenter(a, mid, b) else {
        return Err(free_form_error(index));
    };
    let radius = (a[0] - center[0]).hypot(a[1] - center[1]);
    if !(radius > LEN_TOL) {
        return Err(free_form_error(index));
    }
    let mut prev_angle = (a[1] - center[1]).atan2(a[0] - center[0]);
    let mut sweep = 0.0_f64;
    let mut direction = 0.0_f64;
    for p in &samples[1..] {
        let r = (p[0] - center[0]).hypot(p[1] - center[1]);
        if (r - radius).abs() > 1e-6 * radius.max(1.0) {
            return Err(free_form_error(index));
        }
        let angle = (p[1] - center[1]).atan2(p[0] - center[0]);
        let mut step = angle - prev_angle;
        while step > PI {
            step -= TAU;
        }
        while step <= -PI {
            step += TAU;
        }
        if direction == 0.0 {
            direction = step.signum();
        }
        if step * direction <= 0.0 {
            return Err(free_form_error(index)); // reverses around the fit circle
        }
        sweep += step;
        prev_angle = angle;
    }
    if sweep.abs() >= TAU - 1e-6 {
        // A (near-)full circle segment closes on itself — the open-path guard
        // upstream normally catches this; defensive here.
        return Err(free_form_error(index));
    }
    // Tangents: perpendicular to the radial, oriented along the sweep.
    let tangent_at = |p: [f64; 2]| -> [f64; 2] {
        let rx = (p[0] - center[0]) / radius;
        let ry = (p[1] - center[1]) / radius;
        if sweep >= 0.0 {
            [-ry, rx]
        } else {
            [ry, -rx]
        }
    };
    Ok(PathElement {
        start_tan: tangent_at(a),
        end_tan: tangent_at(b),
        kind: ElementKind::Arc { radius, sweep },
    })
}

/// Circumcenter of three 2D points (`None` when collinear).
fn circumcenter(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<[f64; 2]> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() <= 1e-12 {
        return None;
    }
    let aa = a[0] * a[0] + a[1] * a[1];
    let bb = b[0] * b[0] + b[1] * b[1];
    let cc = c[0] * c[0] + c[1] * c[1];
    Some([
        (aa * (b[1] - c[1]) + bb * (c[1] - a[1]) + cc * (a[1] - b[1])) / d,
        (aa * (c[0] - b[0]) + bb * (a[0] - c[0]) + cc * (b[0] - a[0])) / d,
    ])
}

/// Merge consecutive straight elements that continue within [`TURN_TOL`] into
/// one wall run — two flats with a 0° "bend" between them are one wall (the
/// retired path simplifier did the same).
fn merge_collinear_lines(elems: Vec<PathElement>) -> Vec<PathElement> {
    let mut merged: Vec<PathElement> = Vec::new();
    for elem in elems {
        if let (Some(prev), ElementKind::Line { len }) = (merged.last_mut(), &elem.kind) {
            if let ElementKind::Line { len: prev_len } = &mut prev.kind {
                if signed_turn(prev.end_tan, elem.start_tan).abs() <= TURN_TOL {
                    *prev_len += *len;
                    prev.end_tan = elem.end_tan;
                    continue;
                }
            }
        }
        merged.push(elem);
    }
    merged
}

/// Enforce the path grammar (module doc): starts/ends straight, no consecutive
/// arcs, arcs join tangently, corners don't reverse, arc radii clear t/2.
fn validate_grammar(elems: &[PathElement], thickness: f64) -> Result<(), String> {
    if !matches!(
        elems.first().map(|e| &e.kind),
        Some(ElementKind::Line { .. })
    ) {
        return Err(
            "sheet-metal contour flange: the path must START with a straight segment — a \
             terminal arc is a bend with no wall on its far side (the sheet tree's bends need a \
             flat on both sides); extend the path with a short straight segment"
                .into(),
        );
    }
    if !matches!(elems.last().map(|e| &e.kind), Some(ElementKind::Line { .. })) {
        return Err(
            "sheet-metal contour flange: the path must END with a straight segment — a terminal \
             arc is a bend with no wall on its far side (the sheet tree's bends need a flat on \
             both sides); extend the path with a short straight segment"
                .into(),
        );
    }
    for pair in elems.windows(2) {
        let turn = signed_turn(pair[0].end_tan, pair[1].start_tan);
        match (&pair[0].kind, &pair[1].kind) {
            (ElementKind::Line { .. }, ElementKind::Line { .. }) => {
                if turn.abs() >= PI - 1e-6 {
                    return Err(
                        "sheet-metal contour flange: the path reverses onto itself at a corner \
                         (≈180° turn)"
                            .into(),
                    );
                }
            }
            (ElementKind::Arc { .. }, ElementKind::Arc { .. }) => {
                return Err(
                    "sheet-metal contour flange: consecutive arc segments need a straight \
                     segment between them — a bend's child must be a flat wall; insert a short \
                     straight segment (or merge the arcs into one)"
                        .into(),
                );
            }
            _ => {
                if turn.abs() > TURN_TOL {
                    return Err(format!(
                        "sheet-metal contour flange: an arc joins its neighbor with a corner \
                         ({:.4}° turn); a corner bend at an arc junction is not representable \
                         (its setback cannot be trimmed off the arc) — add a tangent constraint \
                         or a straight segment between them",
                        turn.to_degrees()
                    ));
                }
            }
        }
    }
    for (index, elem) in elems.iter().enumerate() {
        if let ElementKind::Arc { radius, .. } = &elem.kind {
            if *radius <= thickness * 0.5 + 1e-9 {
                return Err(format!(
                    "sheet-metal contour flange: arc segment {index} radius {radius} must exceed \
                     half the sheet thickness ({}) — the inner wall would self-intersect at the \
                     arc center",
                    thickness * 0.5
                ));
            }
        }
    }
    Ok(())
}

/// Turn the validated element list into wall lengths + connecting bend specs
/// `(angle_deg, inside_radius)`:
/// * a straight run → a flat wall;
/// * a line–line corner → a corner bend of `bendRadius` whose fillet setback
///   `Rm·tan(|turn|/2)` (Rm = bendRadius + t/2, the wall-midline radius) is
///   trimmed off both adjoining walls, so their tangent lines still meet at
///   the sketch vertex;
/// * an arc → a rolled bend with mid radius = the arc radius (the sheet stays
///   centered on the path) and no trim (tangency is validated).
fn plan_walls(
    elems: &[PathElement],
    thickness: f64,
    bend_radius: f64,
) -> Result<(Vec<f64>, Vec<(f64, f64)>), String> {
    let corner_mid_radius = bend_radius + thickness * 0.5;
    let mut flat_lens: Vec<f64> = Vec::new();
    let mut bends: Vec<(f64, f64)> = Vec::new();
    let mut carry_trim = 0.0_f64;
    let mut index = 0_usize;
    while index < elems.len() {
        let ElementKind::Line { len } = elems[index].kind else {
            return Err("sheet-metal contour flange: internal: path grammar violated".into());
        };
        let mut length = len - carry_trim;
        carry_trim = 0.0;
        if index + 1 < elems.len() {
            match elems[index + 1].kind {
                ElementKind::Arc { radius, sweep } => {
                    bends.push((sweep.to_degrees(), radius - thickness * 0.5));
                    index += 2; // the arc is consumed with this junction
                }
                ElementKind::Line { .. } => {
                    let turn = signed_turn(elems[index].end_tan, elems[index + 1].start_tan);
                    let setback = corner_mid_radius * (turn.abs() * 0.5).tan();
                    length -= setback;
                    carry_trim = setback;
                    bends.push((turn.to_degrees(), bend_radius));
                    index += 1;
                }
            }
        } else {
            index += 1;
        }
        if !(length > LEN_TOL) {
            return Err(format!(
                "sheet-metal contour flange: wall {} is too short for its corner-bend setback(s) \
                 (segment length {len}, remaining {length}) — lengthen the segment or reduce \
                 bendRadius",
                flat_lens.len()
            ));
        }
        flat_lens.push(length);
    }
    Ok((flat_lens, bends))
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected sketch/edge/face drives `path`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.has_profile() || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SM.CF",
    "shortName": "SM.CF",
    "longName": "SM Contour Flange",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the contour flange feature"
        },
        "path": {
            "type": "reference_selection",
            "selectionFilter": [
                "SKETCH",
                "EDGE",
                "FACE"
            ],
            "preferAncestorSelectionTypes": [
                "SKETCH"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Open sketch path (straight lines + tangent circular arcs) defining the flange centerline."
        },
        "distance": {
            "type": "number",
            "default_value": 20,
            "min": 0,
            "hint": "How far the sheet extends from the selected path (strip width)."
        },
        "thickness": {
            "type": "number",
            "default_value": 2,
            "min": 0,
            "hint": "Sheet metal thickness (centered on the sketch path)."
        },
        "reverseSheetSide": {
            "type": "boolean",
            "default_value": false,
            "hint": "Extend the sheet on the opposite side of the sketch plane (default: the plane's +Z)."
        },
        "bendRadius": {
            "type": "number",
            "default_value": 2,
            "min": 0,
            "hint": "Default inside bend radius inserted wherever two lines meet."
        },
        "neutralFactor": {
            "type": "number",
            "default_value": 0.5,
            "min": 0,
            "max": 1,
            "step": 0.01,
            "hint": "Neutral factor used for flat pattern bend allowance (0-1)."
        }
    }
})
}

