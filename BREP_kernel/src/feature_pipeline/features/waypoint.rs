//! WAYPOINT — a harness ROUTING waypoint: a base point and a unit direction a
//! wire passes through. Not a connection point — those are the part's declared
//! `ports` block (`feature_pipeline/ports.rs`), which is data, not history.
//!
//! A waypoint is the one half of the retired PORT feature that genuinely IS a
//! modelling step: it is placed against the part's geometry, in history order,
//! to steer a harness around a bracket. It carries exactly what routing reads —
//! a point, a direction, and the straight run a wire keeps before it may bend.
//!
//! # What it publishes (all under the feature id)
//!
//! - `scene.ports[{id}]` — the [`PortRecord`], with `kind = Waypoint`. SPLINE
//!   anchors attach to it by id and the wire-harness tail builds its sided
//!   routing graph from it. Its address is its feature id: a waypoint belongs
//!   to no port group, so it carries no `.`.
//! - `scene.points[{id}:Base]` — the base point.
//! - `scene.axes[{id}]` — the waypoint line as an axis (point + direction).
//! - `scene.frames[{id}]` — a plane frame whose normal IS the direction.
//! - `scene.paths[{id}:PortLine]` — the drawn line, centred on the base point
//!   and spanning `displayLength`, because the wire passes THROUGH.
//!
//! # Placement
//!
//! The base point is `transform.position` (world; the transform gizmo drags
//! it). The direction is the `directionRef` reference when one is set — a
//! FACE's outward normal at its midpoint, an EDGE's chord direction, a
//! PLANE / DATUM frame's normal, or a published sketch line — else the
//! `transform.rotationEuler` (degrees, intrinsic XYZ like every feature
//! transform) applied to `+X`. `reverseDirection` flips it. An unresolved
//! `directionRef` is reported through `unresolved` (contract rule 1: the caller
//! repairs; the waypoint still publishes with the transform direction so
//! nothing downstream vanishes).

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::ports;
use crate::feature_pipeline::{FeatureContext, FeatureResult, PortKind, PortRecord};
use crate::Vec3;

/// The id a WAYPOINT publishes under when its `inputParams` carry none.
pub(crate) const UNNAMED_WAYPOINT_ID: &str = "Waypoint";

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(format!("waypoint: {error}")),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let id = if ctx.id.is_empty() { UNNAMED_WAYPOINT_ID } else { ctx.id.as_str() };
    let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());

    let transform = ctx.param("transform");
    let position = common::vec3_from_value(
        ctx.env,
        transform.and_then(|t| t.get("position")),
        "transform.position",
        [0.0, 0.0, 0.0],
    )?;
    let rotation_deg = common::vec3_from_value(
        ctx.env,
        transform.and_then(|t| t.get("rotationEuler")),
        "transform.rotationEuler",
        [0.0, 0.0, 0.0],
    )?;
    let point = Vec3::new(position[0], position[1], position[2]);

    // The transform's own direction: the rotated +X axis.
    let mut direction = super::datum::rotate_euler_xyz(
        Vec3::new(1.0, 0.0, 0.0),
        [
            rotation_deg[0].to_radians(),
            rotation_deg[1].to_radians(),
            rotation_deg[2].to_radians(),
        ],
    );
    if let Some(name) = common::first_reference_name(ctx.param("directionRef")) {
        match ports::resolve_direction(ctx.scene, &name)? {
            Some(resolved) => direction = resolved,
            None => result.unresolved.push(name),
        }
    }
    let mut direction = direction
        .normalized()
        .map_err(|_| "direction is a zero vector".to_string())?;
    if ctx
        .param("reverseDirection")
        .and_then(serde_json::Value::as_bool)
        .unwrap_or(false)
    {
        direction = direction.scale(-1.0);
    }

    let extension = common::optional_number(ctx, "extension")?
        .unwrap_or(ports::DEFAULT_EXTENSION)
        .max(0.0);
    let display_length = common::optional_number(ctx, "displayLength")?
        .unwrap_or(ports::DEFAULT_DISPLAY_LENGTH)
        .max(MIN_DISPLAY_LENGTH);

    let record = PortRecord {
        point,
        direction,
        kind: PortKind::Waypoint,
        extension,
        display_length,
        port_name: String::new(),
        point_name: waypoint_name(ctx.param("waypointName"), id),
        purpose: String::new(),
    };
    ports::publish_port(&mut result, id, record)?;
    Ok(result)
}

/// The shortest drawn line — a zero-length line would publish no path.
const MIN_DISPLAY_LENGTH: f64 = 0.1;

/// The name a waypoint goes by: its trimmed `waypointName`, or its id when that
/// is empty. Routing refers to a waypoint by its id, so this is display only.
pub(crate) fn waypoint_name(name: Option<&serde_json::Value>, id: &str) -> String {
    name.and_then(serde_json::Value::as_str)
        .map(str::trim)
        .filter(|text| !text.is_empty())
        .unwrap_or(id)
        .to_string()
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected face or edge can seat `directionRef`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "WP",
    "shortName": "WP",
    "longName": "Waypoint",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the waypoint feature."
        },
        "waypointName": {
            "type": "string",
            "default_value": "Waypoint",
            "label": "Waypoint name",
            "hint": "Display name for this routing waypoint (the harness panel's label)."
        },
        "directionRef": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "EDGE",
                "PLANE",
                "DATUM"
            ],
            "multiple": false,
            "default_value": null,
            "label": "Direction reference",
            "hint": "Optional geometry that sets the waypoint direction: a face normal, an edge direction or a plane normal. Without it the placement rotation's X axis is the direction."
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
            "label": "Placement",
            "hint": "The waypoint's base point (position) and, without a direction reference, its direction (the rotated X axis)."
        },
        "reverseDirection": {
            "type": "boolean",
            "default_value": false,
            "label": "Reverse direction",
            "hint": "Flip the waypoint direction vector."
        },
        "extension": {
            "type": "number",
            "default_value": 2,
            "label": "Extension",
            "hint": "How far the wire stays straight from the base point before bending. An attached spline anchor takes it as its forward and backward distance."
        },
        "displayLength": {
            "type": "number",
            "default_value": 6,
            "label": "Display length",
            "hint": "Length of the drawn waypoint line, centred on the base point."
        }
    }
})
}

