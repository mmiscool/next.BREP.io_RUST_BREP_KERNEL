//! A sphere centered at the origin with polar axis +Y.
//! The solid and its single analytic face both use the feature ID as their name.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_sphere_brep, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // The radius is taken as its absolute value.
    let radius = ctx.number("radius")?.abs();

    let mut solid = make_sphere_brep(Vec3::new(0.0, 0.0, 0.0), radius, Vec3::new(0.0, 1.0, 0.0))?;

    // The single face carries the feature id verbatim.
    let face = solid
        .shells
        .get_mut(0)
        .and_then(|shell| shell.faces.get_mut(0))
        .ok_or("sphere builder produced no face")?;
    face.name = Some(ctx.id.clone());

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
    "type": "P.S",
    "shortName": "P.S",
    "longName": "Primitive Sphere",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "radius": {
            "type": "number",
            "default_value": 5,
            "hint": "Radius of the sphere"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}
