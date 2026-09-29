//! Linear and circular patterns of named solids.
//!
//! Count-and-span mode divides the displacement or angle by `count - 1`.
//! Other modes apply the requested displacement or angle per instance. Edge
//! references supply axes from their endpoints; sketch axes resolve through
//! the scene. Missing references are reported for repair.
//!
//! Copies are named `${featureID}_${i+1}` and their faces gain `::${featureID}_${i+1}`.
//! NONE keeps separate copies and the source. UNION combines each source with
//! its copies into `${sourceName}::UNION` and removes the source.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::{transform_brep, AffineTransform, BooleanOperation, BooleanOptions, BrepSolid, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // `solids` is a reference_selection of solid NAMES (contract rule 1).
    let names = common::reference_name_array(ctx.param("solids"));
    if names.is_empty() {
        // No selection — a pass-through no-op (`noSourceResult`).
        return Ok(result);
    }

    let resolved = common::resolve_solid_names(ctx.scene, &names, &mut result.unresolved);
    if resolved.is_empty() {
        return Ok(result);
    }

    // `mode === 'LINEAR' ? linear : circular` — any non-LINEAR falls to circular.
    let mode_is_linear = ctx
        .string("mode")
        .map(|mode| mode.to_uppercase())
        .unwrap_or_else(|| "LINEAR".to_string())
        == "LINEAR";
    let count = read_count(ctx);
    let union_mode = ctx
        .string("booleanMode")
        .map(|mode| mode.to_uppercase())
        .unwrap_or_else(|| "NONE".to_string())
        == "UNION";

    // Per-instance placement matrices, shared across all sources (they depend on
    // the pattern params + shared reference, not on the source geometry). May push
    // an `unresolved` ref (→ no-op) or a deferred-FACE hard error.
    let matrices = match compute_matrices(ctx, mode_is_linear, count, &mut result)? {
        Some(matrices) => matrices,
        None => return Ok(result),
    };

    let feature_id = feature_id(ctx);

    // Build EVERYTHING as owned `BrepSolid`s first; register handles only once the
    // whole feature has succeeded, so an error mid-build leaks nothing.
    let mut owned_added: Vec<(String, BrepSolid)> = Vec::new();
    for (source_name, handle) in &resolved {
        // Each clone: transform the source, then retag its faces (`cloneInstance`).
        let mut clones: Vec<(String, BrepSolid)> = Vec::with_capacity(matrices.len());
        for (offset, matrix) in matrices.iter().enumerate() {
            // matrices[0] is i=1 → placement index i+1 = 2, matrices[1] → 3, …
            let index = offset + 2;
            let name = format!("{feature_id}_{index}");
            let transform = AffineTransform::new(*matrix)?;
            let mut solid =
                crate::with_registered_solid_str(*handle, |solid| transform_brep(solid, transform, false))?;
            // Namespace the clone's names `::${cloneName}` so faces AND edges are
            // unique across instances + the source (edges were previously kept 1:1,
            // colliding — the same bug as mirror).
            common::namespace_copy_names(&mut solid, &name);
            clones.push((name, solid));
        }

        if union_mode {
            // Fold a union over `[src, ...clones]`: base = untagged source clone.
            let base = crate::with_registered_solid_str(*handle, |solid| Ok(solid.clone()))?;
            let clone_solids: Vec<BrepSolid> = clones.into_iter().map(|(_, solid)| solid).collect();
            let union = union_all(base, clone_solids)?;
            owned_added.push((format!("{source_name}::UNION"), union));
        } else {
            owned_added.extend(clones);
        }
    }

    // All geometry built — now register handles and collect names.
    for (name, solid) in owned_added {
        let face_names = common::collect_face_names(&solid);
        let edge_names = common::collect_edge_names(&solid);
        let handle = crate::register_solid_value(solid);
        result.added.push(AddedSolid {
            handle,
            name,
            face_names,
            edge_names,
            ..AddedSolid::default()
        });
    }

    // UNION consumes each source (`removed: solids`); NONE keeps them (`[]`).
    if union_mode {
        result.removed = resolved.into_iter().map(|(name, _)| name).collect();
    }
    Ok(result)
}

/// `#featureID()`: `inputParams.featureID || PatternFeature.shortName`. The pipeline
/// carries the id in `ctx.id`; fall back to the short name `PATTERN` when empty so
/// clone names are `PATTERN_2`, never `_2`.
fn feature_id(ctx: &FeatureContext) -> String {
    if ctx.id.is_empty() {
        "PATTERN".to_string()
    } else {
        ctx.id.clone()
    }
}

/// The per-instance placement matrices for `i = 1 ..= count-1`. `Ok(None)` when a
/// named reference did not resolve (`result.unresolved` populated → the caller repairs).
fn compute_matrices(
    ctx: &FeatureContext,
    mode_is_linear: bool,
    count: i64,
    result: &mut FeatureResult,
) -> Result<Option<Vec<[f64; 16]>>, String> {
    if count <= 1 {
        // No clones (the loop `for i=1; i<=count-1` never runs).
        return Ok(Some(Vec::new()));
    }

    if mode_is_linear {
        let delta = match resolve_linear_delta(ctx, count, result)? {
            Some(delta) => delta,
            None => return Ok(None),
        };
        let mut matrices = Vec::with_capacity((count - 1) as usize);
        for i in 1..count {
            matrices.push(translation_matrix(delta.scale(i as f64)));
        }
        Ok(Some(matrices))
    } else {
        let (axis, center) = match resolve_circular_axis(ctx, result)? {
            Some(axis_center) => axis_center,
            None => return Ok(None),
        };
        let span = normalize_option(&option_str(ctx, "countMode", "count and pitch")) == "COUNT_AND_SPAN";
        let divisor = if span { (count - 1).max(1) } else { 1 } as f64;
        // count > 1 here, so `step = degToRad(totalAngleDeg)/divisor`.
        let step = total_angle_deg(ctx)?.to_radians() / divisor;
        let mut matrices = Vec::with_capacity((count - 1) as usize);
        for i in 1..count {
            matrices.push(rotation_about_axis(axis, step * (i as f64), center));
        }
        Ok(Some(matrices))
    }
}

/// `resolveLinearDelta`. `Ok(None)` when a named `directionRef` did not resolve.
fn resolve_linear_delta(
    ctx: &FeatureContext,
    count: i64,
    result: &mut FeatureResult,
) -> Result<Option<Vec3>, String> {
    let linear_mode = normalize_option(&option_str(ctx, "linearInputMode", "vector distance"));
    let span = normalize_option(&option_str(ctx, "countMode", "count and pitch")) == "COUNT_AND_SPAN";
    let span_divisor = if span { (count - 1).max(1) } else { 1 } as f64;

    if linear_mode != "VECTOR_DISTANCE" {
        // "transform" input mode → the gizmo `offset.position` (default [10,0,0]).
        let position = read_offset_position(ctx)?;
        return Ok(Some(position.scale(1.0 / span_divisor)));
    }

    // "vector distance" → direction (× distance). An EDGE reference is a resident
    // solid edge OR a sketch line axis (both are EDGE objects).
    let direction = match common::single_reference_name(ctx.param("directionRef")) {
        None => Vec3::new(1.0, 0.0, 0.0),
        Some(name) => {
            if let Some(edge_ref) = ctx.scene.resolve_edge(&name) {
                crate::with_registered_solid_str(edge_ref.handle, |solid| {
                    edge_axis(solid, edge_ref.edge_id)
                })?
                .1
            } else if let Some(axis) = ctx.scene.resolve_axis(&name) {
                axis.direction
            } else if let Some(plane_ref) = common::resolve_plane_reference(ctx, &name) {
                // A datum / construction PLANE or a planar FACE → the pattern runs
                // along its normal. Both go through the shared plane resolver, so a
                // datum plane and a face behave identically (the frame table used
                // to be missed here, no-oping a datum-plane direction; a face used
                // to error). The analytic midpoint normal is exact — no
                // tessellation winding involved.
                plane_ref.point_normal()?.1
            } else {
                result.unresolved.push(name);
                return Ok(None);
            }
        }
    };
    let distance = number_param_or(ctx, "linearDistance", 10.0)?;
    Ok(Some(direction.scale(distance / span_divisor)))
}

/// `computeAxisFromEdge` + center: `Ok(None)` when a named `axisRef` did not
/// resolve. With no `axisRef`, the axis defaults `(0,1,0)` through the origin.
fn resolve_circular_axis(
    ctx: &FeatureContext,
    result: &mut FeatureResult,
) -> Result<Option<(Vec3, Vec3)>, String> {
    let center_offset = number_param_or(ctx, "centerOffset", 0.0)?;
    match common::single_reference_name(ctx.param("axisRef")) {
        None => {
            let axis = Vec3::new(0.0, 1.0, 0.0);
            let center = Vec3::new(0.0, 0.0, 0.0).add(axis.scale(center_offset));
            Ok(Some((axis, center)))
        }
        Some(name) => {
            // A resident solid edge, or a sketch line axis (`{sketchId}:G{gid}`);
            // both are EDGE objects for `computeAxisFromEdge`.
            let resolved = if let Some(edge_ref) = ctx.scene.resolve_edge(&name) {
                Some(crate::with_registered_solid_str(edge_ref.handle, |solid| {
                    edge_axis(solid, edge_ref.edge_id)
                })?)
            } else {
                ctx.scene
                    .resolve_axis(&name)
                    .map(|axis| (axis.point, axis.direction))
            };
            match resolved {
                Some((point, dir)) => {
                    let center = point.add(dir.scale(center_offset));
                    Ok(Some((dir, center)))
                }
                None => {
                    result.unresolved.push(name);
                    Ok(None)
                }
            }
        }
    }
}

/// An edge's `(start_point, unit_direction)` from its kernel start/end vertices.
/// A zero-length (closed) edge degenerates to `(0,1,0)`, matching the `lengthSq`
/// guard on the polyline chord.
fn edge_axis(solid: &BrepSolid, edge_id: u64) -> Result<(Vec3, Vec3), String> {
    let edge = solid
        .edges
        .iter()
        .find(|edge| edge.id == edge_id)
        .ok_or_else(|| format!("Pattern reference edge id {edge_id} not found"))?;
    let point_of = |vertex_id: u64| {
        solid
            .vertices
            .iter()
            .find(|vertex| vertex.id == vertex_id)
            .map(|vertex| vertex.point)
    };
    let start = point_of(edge.start_vertex_id)
        .ok_or_else(|| format!("edge {edge_id} start vertex missing"))?;
    let end =
        point_of(edge.end_vertex_id).ok_or_else(|| format!("edge {edge_id} end vertex missing"))?;
    let delta = end.sub(start);
    let dir = if delta.length_squared() < 1e-12 {
        Vec3::new(0.0, 1.0, 0.0)
    } else {
        delta.normalized()?
    };
    Ok((start, dir))
}

/// Row-major pure translation `[ I | d ]`.
fn translation_matrix(d: Vec3) -> [f64; 16] {
    [
        1.0, 0.0, 0.0, d.x, //
        0.0, 1.0, 0.0, d.y, //
        0.0, 0.0, 1.0, d.z, //
        0.0, 0.0, 0.0, 1.0,
    ]
}

/// Row-major `T(center) · R(axis, θ) · T(-center) = [ R | center − R·center ]`,
/// with `R` the right-handed Rodrigues rotation about the UNIT `axis` (an
/// axis-angle quaternion → rotation matrix).
fn rotation_about_axis(axis: Vec3, theta: f64, center: Vec3) -> [f64; 16] {
    let (ux, uy, uz) = (axis.x, axis.y, axis.z);
    let (c, s) = (theta.cos(), theta.sin());
    let t = 1.0 - c;
    let r = [
        [t * ux * ux + c, t * ux * uy - s * uz, t * ux * uz + s * uy],
        [t * ux * uy + s * uz, t * uy * uy + c, t * uy * uz - s * ux],
        [t * ux * uz - s * uy, t * uy * uz + s * ux, t * uz * uz + c],
    ];
    let rc = [
        r[0][0] * center.x + r[0][1] * center.y + r[0][2] * center.z,
        r[1][0] * center.x + r[1][1] * center.y + r[1][2] * center.z,
        r[2][0] * center.x + r[2][1] * center.y + r[2][2] * center.z,
    ];
    [
        r[0][0], r[0][1], r[0][2], center.x - rc[0], //
        r[1][0], r[1][1], r[1][2], center.y - rc[1], //
        r[2][0], r[2][1], r[2][2], center.z - rc[2], //
        0.0, 0.0, 0.0, 1.0,
    ]
}

/// Fold a union over `[base, ...clones]`: seed = `base`, then `result =
/// Union(result, clone)` with default (merge-coplanar) options. Frees both
/// transient handles BEFORE propagating a fold error — no leaked intermediates.
fn union_all(base: BrepSolid, clones: Vec<BrepSolid>) -> Result<BrepSolid, String> {
    let options = BooleanOptions::default();
    let mut current = base;
    for clone in clones {
        let running = crate::register_solid_value(current);
        let tool = crate::register_solid_value(clone);
        let folded = crate::with_two_registered_solids(running, tool, |accumulated, next| {
            crate::boolean_operation(accumulated, next, BooleanOperation::Union, &options)
        });
        crate::free_registered_solid(running);
        crate::free_registered_solid(tool);
        current = folded.map_err(|error| format!("Pattern union failed: {error}"))?;
    }
    Ok(current)
}

// ---------------------------------------------------------------------------
// Param readers
// ---------------------------------------------------------------------------

/// `count = Math.max(1, (inputParams.count | 0))` — truncate toward zero, clamp to
/// ≥ 1 (a missing/non-numeric count → `0 | 0` → 1).
fn read_count(ctx: &FeatureContext) -> i64 {
    let raw = match ctx.param("count") {
        Some(serde_json::Value::Number(number)) => number.as_f64().unwrap_or(0.0),
        Some(serde_json::Value::String(source)) => ctx.env.eval(source).unwrap_or(0.0),
        _ => 0.0,
    };
    let truncated = if raw.is_finite() { raw.trunc() as i64 } else { 0 };
    truncated.max(1)
}

/// `Number(totalAngleDeg) || 360` — 0 / non-finite → 360.
fn total_angle_deg(ctx: &FeatureContext) -> Result<f64, String> {
    let value = number_param_or(ctx, "totalAngleDeg", 360.0)?;
    Ok(if value == 0.0 || !value.is_finite() {
        360.0
    } else {
        value
    })
}

/// `toFiniteNumber(param, default)`: literal number, expression string, or
/// missing/null/non-finite → `default`.
fn number_param_or(ctx: &FeatureContext, key: &str, default: f64) -> Result<f64, String> {
    Ok(match ctx.param(key) {
        Some(serde_json::Value::Number(number)) => {
            number.as_f64().filter(|value| value.is_finite()).unwrap_or(default)
        }
        Some(serde_json::Value::String(source)) => match ctx.env.eval(source) {
            Ok(value) if value.is_finite() => value,
            _ => default,
        },
        _ => default,
    })
}

/// An option-string param read verbatim (never expression-evaluated), or `default`.
fn option_str(ctx: &FeatureContext, key: &str, default: &str) -> String {
    ctx.string(key).unwrap_or_else(|| default.to_string())
}

/// `normalizeOptionKey`: trim, collapse runs of whitespace/`-` to a single `_`,
/// uppercase. e.g. `"vector distance"` → `"VECTOR_DISTANCE"`.
fn normalize_option(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    let mut prev_sep = false;
    for ch in value.trim().chars() {
        if ch.is_whitespace() || ch == '-' {
            if !prev_sep {
                out.push('_');
                prev_sep = true;
            }
        } else {
            out.push(ch);
            prev_sep = false;
        }
    }
    out.to_uppercase()
}

/// `toVec3(offset.position || [10,0,0], 10, 0, 0)` — the gizmo translation, with
/// per-component defaults `[10,0,0]`.
fn read_offset_position(ctx: &FeatureContext) -> Result<Vec3, String> {
    let default = [10.0, 0.0, 0.0];
    let position = ctx
        .param("offset")
        .and_then(|offset| offset.get("position"));
    let Some(position) = position else {
        return Ok(Vec3::new(default[0], default[1], default[2]));
    };
    let component = |slot: Option<&serde_json::Value>, fallback: f64| -> Result<f64, String> {
        match slot {
            None | Some(serde_json::Value::Null) => Ok(fallback),
            Some(serde_json::Value::Number(number)) => number
                .as_f64()
                .ok_or_else(|| "offset.position has a non-finite component".to_string()),
            Some(serde_json::Value::String(source)) => ctx
                .env
                .eval(source)
                .map_err(|error| format!("offset.position: {error}")),
            Some(other) => Err(format!("offset.position component must be numeric, found {other}")),
        }
    };
    if let Some(array) = position.as_array() {
        return Ok(Vec3::new(
            component(array.first(), default[0])?,
            component(array.get(1), default[1])?,
            component(array.get(2), default[2])?,
        ));
    }
    if position.is_object() {
        return Ok(Vec3::new(
            component(position.get("x"), default[0])?,
            component(position.get("y"), default[1])?,
            component(position.get("z"), default[2])?,
        ));
    }
    Err("offset.position must be a vec3 array or object".to_string())
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected solids drive `solids`, an edge/face `directionRef`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0 || probe.edges > 0 || probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "PATTERN",
    "shortName": "PATTERN",
    "longName": "Pattern",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the pattern feature"
        },
        "solids": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select solids to pattern"
        },
        "mode": {
            "type": "options",
            "options": [
                "LINEAR",
                "CIRCULAR"
            ],
            "default_value": "LINEAR",
            "hint": "Pattern type"
        },
        "linearInputMode": {
            "type": "options",
            "options": [
                "transform",
                "vector distance"
            ],
            "default_value": "vector distance",
            "label": "Linear input",
            "hint": "Use transform controls or a selected direction plus distance"
        },
        "count": {
            "type": "number",
            "default_value": 3,
            "step": 1,
            "hint": "Instance count (>= 1)"
        },
        "countMode": {
            "type": "options",
            "options": [
                "count and pitch",
                "count and span"
            ],
            "default_value": "count and pitch",
            "label": "Count mode",
            "hint": "Use the distance/angle as the per-step pitch, or divide it across the full span"
        },
        "offset": {
            "type": "transform",
            "default_value": {
                "position": [
                    10,
                    0,
                    0
                ],
                "rotationEuler": [
                    0,
                    0,
                    0
                ],
                "scale": [
                    1,
                    1,
                    1
                ]
            },
            "label": "Offset (use gizmo)",
            "hint": "Use Move gizmo to set direction and distance (position only)"
        },
        "directionRef": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE",
                "FACE",
                "PLANE"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Direction",
            "hint": "Select an EDGE direction or FACE/PLANE normal for linear spacing"
        },
        "linearDistance": {
            "type": "number",
            "default_value": 10,
            "label": "Distance",
            "hint": "Distance between linear pattern instances along the selected direction"
        },
        "axisRef": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Axis",
            "hint": "Select an EDGE to define the circular pattern axis"
        },
        "centerOffset": {
            "type": "number",
            "default_value": 0,
            "hint": "Offset along axis from reference origin to pattern center"
        },
        "totalAngleDeg": {
            "type": "number",
            "default_value": 360,
            "hint": "Angle between circular instances, or total span when Count mode is count and span"
        },
        "booleanMode": {
            "type": "options",
            "options": [
                "NONE",
                "UNION"
            ],
            "default_value": "NONE",
            "hint": "Optionally union instances together"
        }
    }
})
}

