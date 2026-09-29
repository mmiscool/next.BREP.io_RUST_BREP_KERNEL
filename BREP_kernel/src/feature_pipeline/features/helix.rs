//! Publish a helical curve as `{id}` / `{id}:HelixEdge`, its endpoints as
//! `{id}:Start` / `{id}:End`, and its axis as `{id}:Axis`. The axis also rides
//! beside the curve as its SCREW AXIS (`path_segment_axes`), so a sweep carrying
//! a profile by the path's own motion screws it up this axis instead of rolling
//! it on a rotation-minimizing frame.
//!
//! Radius varies linearly from `radius` to `endRadius`. Height always applies;
//! turns mode derives pitch as height/turns, and pitch mode derives turns as
//! height/pitch. Handedness and `startAngle` control winding. The transcendental
//! curve is approximated by a cubic through 64 stations per turn, using the same
//! sampler and fit as coil sweeps (error approximately 2e-7 times radius).
//!
//! Transform mode starts along +Z with angle zero at +X and transforms samples
//! before fitting, preserving nonuniform scale. Axis mode starts at the selected
//! edge's start; an optional `startPoint` sets both angle zero and axial height.
//! Without it, angle zero uses an arbitrary perpendicular to the axis.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{Axis, FeatureContext, FeatureResult};
use crate::{fit_helix_curve, helix_sample_points, interpolate_curve, NurbsCurve, Vec3};
use serde_json::Value;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(format!("helix: {error}")),
    }
}

/// The resolved helix parameters, placement-independent.
struct HelixParams {
    radius: f64,
    end_radius: f64,
    pitch: f64,
    turns: f64,
    start_angle: f64,
    left_handed: bool,
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let params = read_params(ctx)?;
    let base = if ctx.id.is_empty() { "Helix" } else { ctx.id.as_str() };
    let placement = ctx.string("placementMode").unwrap_or_else(|| "transform".to_string());
    let (curve, axis) = match placement.as_str() {
        "axis" => axis_placed(ctx, &params)?,
        _ => transform_placed(ctx, &params)?,
    };

    let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
    let [t0, t1] = curve.domain()?;
    let start = curve.evaluate(t0)?;
    let end = curve.evaluate(t1)?;
    let edge_name = format!("{base}:HelixEdge");
    // Publish under BOTH the feature id and the edge name (exact-match
    // resolution, contract rule 1) so a `path` reference resolves either
    // spelling; the segment name lets a sweep name what it builds after the edge.
    result.paths.push((base.to_string(), vec![curve.clone()]));
    result.paths.push((edge_name.clone(), vec![curve]));
    result
        .path_segment_names
        .push((base.to_string(), vec![Some(edge_name.clone())]));
    result
        .path_segment_names
        .push((edge_name.clone(), vec![Some(edge_name.clone())]));
    // The curve IS a helix about `axis`, and only this feature knows it — the
    // published curve is a cubic fit. A sweep carrying a profile along it by the
    // path's own motion needs that screw axis: the rotation-minimizing frame a
    // free curve gets rolls away from the screw by the helix's torsion.
    result.path_segment_axes.push((base.to_string(), vec![Some(axis)]));
    result.path_segment_axes.push((edge_name, vec![Some(axis)]));
    result
        .points
        .push((format!("{base}:Start"), crate::feature_pipeline::ScenePoint::model(start)));
    result
        .points
        .push((format!("{base}:End"), crate::feature_pipeline::ScenePoint::model(end)));
    result.axes.push((format!("{base}:Axis"), axis));
    Ok(result)
}

/// The scalar inputs: `mode` decides whether `turns` or `pitch` is authored;
/// `height` always applies. Absent / null numbers take the schema defaults.
fn read_params(ctx: &FeatureContext) -> Result<HelixParams, String> {
    let radius = number_or(ctx, "radius", 5.0)?;
    let end_radius = number_or(ctx, "endRadius", radius)?;
    let height = number_or(ctx, "height", 15.0)?;
    let mode = ctx.string("mode").unwrap_or_else(|| "turns".to_string());
    let (turns, pitch) = if mode == "pitch" {
        let pitch = number_or(ctx, "pitch", 5.0)?;
        if !(pitch.is_finite() && pitch > 0.0) {
            return Err(format!("pitch must be positive in pitch mode (got {pitch})"));
        }
        if !(height.is_finite() && height > 0.0) {
            return Err(format!(
                "height must be positive in pitch mode (turns = height / pitch; got {height})"
            ));
        }
        (height / pitch, pitch)
    } else {
        let turns = number_or(ctx, "turns", 3.0)?;
        if !(turns.is_finite() && turns > 0.0) {
            return Err(format!("turns must be positive (got {turns})"));
        }
        (turns, height / turns)
    };
    if !(height.is_finite() && height >= 0.0) {
        return Err(format!("height must not be negative (got {height})"));
    }
    if radius <= 0.0 && end_radius <= 0.0 {
        return Err("radius and end radius are both zero — nothing to wind".into());
    }
    let start_angle = number_or(ctx, "startAngle", 0.0)?.to_radians();
    let left_handed = ctx.string("handedness").as_deref() == Some("left");
    Ok(HelixParams {
        radius,
        end_radius,
        pitch,
        turns,
        start_angle,
        left_handed,
    })
}

/// A numeric param with a default for an absent OR null slot (the schema
/// defaults `endRadius` to the radius and the rest to fixed numbers); a present
/// value goes through the ordinary expression-aware `number` read.
fn number_or(ctx: &FeatureContext, key: &str, default: f64) -> Result<f64, String> {
    match ctx.param(key) {
        None | Some(Value::Null) => Ok(default),
        Some(_) => ctx.number(key),
    }
}

/// `transform` placement: author about the origin (axis +Z, angle zero at
/// +X), map every station through the feature transform, fit.
fn transform_placed(
    ctx: &FeatureContext,
    params: &HelixParams,
) -> Result<(NurbsCurve, Axis), String> {
    let origin = Vec3::new(0.0, 0.0, 0.0);
    let z = Vec3::new(0.0, 0.0, 1.0);
    let x = Vec3::new(1.0, 0.0, 0.0);
    let (mut points, parameters) = helix_sample_points(
        origin,
        z,
        Some(x),
        params.radius,
        params.end_radius,
        params.pitch,
        params.turns,
        params.start_angle,
        params.left_handed,
    )?;
    let mut axis = Axis {
        point: origin,
        direction: z,
    };
    if let Some(affine) = transform_bake::trs_transform(ctx)? {
        for point in &mut points {
            *point = affine.point(*point);
        }
        let tip = affine.point(z);
        axis.point = affine.point(origin);
        axis.direction = tip.sub(axis.point).normalized()?;
    }
    let curve = interpolate_curve(&points, 3, &parameters)?;
    Ok((curve, axis))
}

/// `axis` placement: a selected edge is the axis, an optional start point sets
/// the start height and the angle-zero direction.
fn axis_placed(ctx: &FeatureContext, params: &HelixParams) -> Result<(NurbsCurve, Axis), String> {
    let axis_name = ctx
        .param("axis")
        .and_then(common::reference_name)
        .ok_or("axis placement needs an axis edge (`axis`)")?;
    let mut axis = resolve_axis(ctx, &axis_name)?;
    let mut reference = None;
    if let Some(name) = ctx.param("startPoint").and_then(common::reference_name) {
        let start = resolve_start_point(ctx, &name)?;
        // The start point's foot on the axis is where the helix begins; its
        // perpendicular offset (if any) is where angle zero points.
        let offset = start.sub(axis.point);
        let along = offset.dot(axis.direction);
        axis.point = axis.point.add(axis.direction.scale(along));
        let radial = offset.sub(axis.direction.scale(along));
        if radial.length() > 1e-9 {
            reference = Some(radial);
        }
    }
    let curve = fit_helix_curve(
        axis.point,
        axis.direction,
        reference,
        params.radius,
        params.end_radius,
        params.pitch,
        params.turns,
        params.start_angle,
        params.left_handed,
    )?;
    Ok((curve, axis))
}

/// The axis from a published sketch line or a resident solid edge (the
/// Revolve rule).
fn resolve_axis(ctx: &FeatureContext, name: &str) -> Result<Axis, String> {
    if let Some(axis) = ctx.scene.resolve_axis(name) {
        return Ok(axis);
    }
    if let Some(edge) = ctx.scene.resolve_edge(name) {
        return common::edge_axis(edge);
    }
    Err(format!("axis '{name}' not found (no sketch line or resident edge)"))
}

/// A start point: a published scene point (`{sketch}:P{n}`, a spline anchor,
/// another helix's end) or a solid vertex reference `{solid}@x,y,z`, whose
/// coordinates are the vertex's world position.
fn resolve_start_point(ctx: &FeatureContext, name: &str) -> Result<Vec3, String> {
    if let Some(point) = ctx.scene.resolve_point(name) {
        return Ok(point);
    }
    if let Some((_, coords)) = name.split_once('@') {
        let mut parts = coords.split(',').map(str::trim);
        let mut next = || -> Option<f64> { parts.next()?.parse::<f64>().ok() };
        if let (Some(x), Some(y), Some(z)) = (next(), next(), next()) {
            return Ok(Vec3::new(x, y, z));
        }
        return Err(format!("start point '{name}': vertex position must be 'x,y,z' numbers"));
    }
    Err(format!("start point '{name}' not found (no scene point or solid vertex)"))
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected edge can seat the `axis`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "HX",
    "shortName": "HX",
    "longName": "Helix",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the helix feature"
        },
        "placementMode": {
            "type": "options",
            "options": [
                "transform",
                "axis"
            ],
            "default_value": "transform",
            "label": "Placement",
            "hint": "Use transform or align to an existing axis and start point"
        },
        "transform": {
            "type": "transform",
            "default_value": {
                "position": [
                    0,
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
            "hint": "Position, rotation, and scale to place the helix"
        },
        "axis": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Axis",
            "hint": "Select an edge to use as the helix axis"
        },
        "startPoint": {
            "type": "reference_selection",
            "selectionFilter": [
                "VERTEX"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Start point",
            "hint": "Optional start point; defaults to the axis start"
        },
        "radius": {
            "type": "number",
            "default_value": 5,
            "hint": "Base radius of the helix"
        },
        "endRadius": {
            "type": "number",
            "default_value": 5,
            "hint": "Optional end radius to taper the helix"
        },
        "handedness": {
            "type": "options",
            "options": [
                "right",
                "left"
            ],
            "default_value": "right",
            "hint": "Choose right- or left-handed helix winding"
        },
        "mode": {
            "type": "options",
            "options": [
                "turns",
                "pitch"
            ],
            "default_value": "turns",
            "label": "Mode",
            "hint": "Control helix using turn count or pitch; height is always applied"
        },
        "height": {
            "type": "number",
            "default_value": 15,
            "hint": "Total height along the axis"
        },
        "turns": {
            "type": "number",
            "default_value": 3,
            "hint": "Number of turns (used in mode 'turns'; derived in mode 'pitch')"
        },
        "pitch": {
            "type": "number",
            "default_value": 5,
            "step": 1,
            "min": 0.001,
            "hint": "Distance advanced per turn along the helix axis (editable in mode 'pitch'; derived otherwise)"
        },
        "startAngle": {
            "type": "number",
            "default_value": 0,
            "hint": "Starting angle in degrees"
        }
    }
})
}

