//! A box extending from the origin along +X, +Y, and +Z.
//! The solid uses the feature ID; face names append the suffixes in `FACE_SUFFIX`.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_box_brep, Vec3};

/// `make_box_brep` face order → face-name suffix (positional, verified by geometry).
const FACE_SUFFIX: [&str; 6] = ["_NZ", "_PZ", "_NY", "_PY", "_NX", "_PX"];

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // A negative size extends the box in the -axis direction: anchor the corner at
    // min(0, size) per axis and build with the magnitude, so sizeX = -5 spans
    // x ∈ [-5, 0] (the dimension arrow already points that way). Mixed signs fall
    // out per axis. (Diverges from the old `Math.abs`, which ignored the sign.)
    let size_x = ctx.number("sizeX")?;
    let size_y = ctx.number("sizeY")?;
    let size_z = ctx.number("sizeZ")?;
    let corner = Vec3::new(size_x.min(0.0), size_y.min(0.0), size_z.min(0.0));

    let mut solid = make_box_brep(corner, size_x.abs(), size_y.abs(), size_z.abs())?;

    // Stamp the six face names in `make_box_brep` order.
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("box builder produced no shell")?
        .faces;
    if faces.len() != FACE_SUFFIX.len() {
        return Err(format!("box builder produced {} faces, expected 6", faces.len()));
    }
    for (face, suffix) in faces.iter_mut().zip(FACE_SUFFIX) {
        face.name = Some(format!("{}{}", ctx.id, suffix));
    }

    // Bake the `transform` param BEFORE the boolean; the bake preserves the
    // stamped names (geometry-only rewrite).
    let solid = transform_bake::apply(ctx, solid)?;

    Ok(common::finalize_solid(ctx, solid, &ctx.id))
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered from a selection (no reference inputs).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "P.CU",
    "shortName": "P.CU",
    "longName": "Primitive Cube",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "sizeX": {
            "type": "number",
            "default_value": 10,
            "hint": "Width along X"
        },
        "sizeY": {
            "type": "number",
            "default_value": 10,
            "hint": "Height along Y"
        },
        "sizeZ": {
            "type": "number",
            "default_value": 10,
            "hint": "Depth along Z"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}

