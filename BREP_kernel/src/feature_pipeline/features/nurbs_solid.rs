//! NS — NURBS Solid: a smooth body shaped by a control cage.
//!
//! The feature wraps a [`ControlCage`] around a primitive (`primitive`,
//! `sizeX/Y/Z`, `segments`, `rings`) and builds the bicubic B-spline solid the
//! cage controls — one closed face with a seam edge and two pole edges, named
//! `{id}` like the sphere's. The editor deforms the cage by moving its points
//! and refines it by inserting loops; both write the edited cage to
//! `persistentData.cage`, and from then on THAT cage is the body — the
//! primitive params are the starting shape only, until the editor's Reset
//! drops the persisted cage. [`resolve_cage`] is the ONE place that decision
//! is made, and the editor reads its points through it, so what is drawn as
//! a handle is exactly what the kernel builds from.
//!
//! The cage lives in world space: there is no `transform` param, because a
//! point dragged in the viewport must be the point the kernel builds from,
//! not a point seen through a bake. Move the whole body by selecting every
//! point, or transform the result downstream.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{Env, FeatureContext, FeatureResult};
use crate::{CagePrimitive, ControlCage, MAX_RINGS, MAX_SEGMENTS, MIN_RINGS, MIN_SEGMENTS};
use serde_json::Value;

/// The `persistentData` key the edited cage is stored under.
pub const PERSISTENT_KEY: &str = "cage";
/// The primitive a fresh feature wraps.
pub const DEFAULT_PRIMITIVE: &str = "BOX";
pub const DEFAULT_SIZE: f64 = 10.0;
pub const DEFAULT_SEGMENTS: usize = 8;
pub const DEFAULT_RINGS: usize = 3;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let cage = resolve_cage(ctx.env, ctx.params, ctx.persistent)?;
    let solid = cage.solid(Some(ctx.id.as_str()))?;
    Ok(common::finalize_solid(ctx, solid, &ctx.id))
}

/// Whether the document carries an edited cage (the primitive params are then
/// only the shape it started from).
pub fn has_persisted_cage(persistent: &Value) -> bool {
    persistent.get(PERSISTENT_KEY).is_some_and(|cage| !cage.is_null())
}

/// The cage this feature builds from: the persisted one when the document has
/// it, otherwise the primitive the params describe. The editor reads its
/// handles through this, so a handle is always a point the kernel builds from.
pub fn resolve_cage(env: &Env, params: &Value, persistent: &Value) -> Result<ControlCage, String> {
    if let Some(stored) = persistent.get(PERSISTENT_KEY).filter(|cage| !cage.is_null()) {
        return ControlCage::from_json(stored)
            .map_err(|error| format!("nurbs solid: the stored cage is unusable: {error}"));
    }
    primitive_cage(env, params)
}

/// The initial cage the params describe, ignoring any persisted cage — what
/// Reset returns to.
pub fn primitive_cage(env: &Env, params: &Value) -> Result<ControlCage, String> {
    let kind_name = params
        .get("primitive")
        .and_then(Value::as_str)
        .unwrap_or(DEFAULT_PRIMITIVE);
    let kind = CagePrimitive::parse(kind_name)
        .ok_or_else(|| format!("nurbs solid: unknown primitive `{kind_name}` (BOX, CYLINDER or SPHERE)"))?;
    let size = [
        number(env, params, "sizeX", DEFAULT_SIZE)?,
        number(env, params, "sizeY", DEFAULT_SIZE)?,
        number(env, params, "sizeZ", DEFAULT_SIZE)?,
    ];
    let count = |key: &str, default: usize, low: usize, high: usize| -> Result<usize, String> {
        let value = number(env, params, key, default as f64)?;
        let rounded = value.round();
        if rounded < low as f64 || rounded > high as f64 {
            return Err(format!("nurbs solid: `{key}` must be between {low} and {high}, got {value}"));
        }
        Ok(rounded as usize)
    };
    let segments = count("segments", DEFAULT_SEGMENTS, MIN_SEGMENTS, MAX_SEGMENTS)?;
    let rings = count("rings", DEFAULT_RINGS, MIN_RINGS, MAX_RINGS)?;
    ControlCage::primitive(kind, size, segments, rings).map_err(|error| format!("nurbs solid: {error}"))
}

/// A numeric param by the pipeline's convention: a number passes through, a
/// string is an expression against `env`, absent / null is the default.
fn number(env: &Env, params: &Value, key: &str, default: f64) -> Result<f64, String> {
    let value = match params.get(key) {
        None | Some(Value::Null) => return Ok(default),
        Some(Value::Number(number)) => number
            .as_f64()
            .ok_or_else(|| format!("nurbs solid: param `{key}` is not a finite number"))?,
        Some(Value::String(source)) => env
            .eval(source)
            .map_err(|error| format!("nurbs solid: param `{key}`: {error}"))?,
        Some(other) => {
            return Err(format!(
                "nurbs solid: param `{key}` must be a number or expression string, found {other}"
            ))
        }
    };
    if !value.is_finite() {
        return Err(format!("nurbs solid: param `{key}` evaluated to {value}"));
    }
    Ok(value)
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered from a selection (no reference inputs).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "NS",
    "shortName": "NS",
    "longName": "NURBS Solid",
    "ribbonPath": "Home/Primitives/NURBS Solid",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "primitive": {
            "type": "options",
            "label": "Primitive",
            "options": CagePrimitive::OPTIONS,
            "default_value": DEFAULT_PRIMITIVE,
            "hint": "The primitive the control cage starts as; the smooth body lies inside the cage"
        },
        "sizeX": {
            "type": "number",
            "default_value": DEFAULT_SIZE,
            "hint": "Cage extent along X (centred on the origin)"
        },
        "sizeY": {
            "type": "number",
            "default_value": DEFAULT_SIZE,
            "hint": "Cage extent along Y, the pole-to-pole axis"
        },
        "sizeZ": {
            "type": "number",
            "default_value": DEFAULT_SIZE,
            "hint": "Cage extent along Z"
        },
        "segments": {
            "type": "number",
            "default_value": DEFAULT_SEGMENTS,
            "min": MIN_SEGMENTS,
            "step": 1,
            "hint": "Control points around each ring of the initial cage (at least 4)"
        },
        "rings": {
            "type": "number",
            "default_value": DEFAULT_RINGS,
            "min": MIN_RINGS,
            "step": 1,
            "hint": "Rings of control points between the two poles of the initial cage (at least 2)"
        },
        "boolean": common::optional_boolean_schema()
    }
})
}

