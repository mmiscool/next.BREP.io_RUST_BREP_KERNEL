//! P.CY — Primitive Cylinder. Ported from the retired `Primitive Cylinder` feature.
//!
//! The cylinder is aligned along +Y with its base at `y=0` and top at `y=height`
//! (built from base `(0,0,0)`, axis `(0,1,0)`).
//!
//! Name fidelity (contract rule 2): the solid is named `${featureID}`; its three
//! faces are `${featureID}` + `_S, _B, _T` (Side, Bottom, Top). This ORDER matches
//! `make_cylinder_brep`'s shell face order `[side_face, bottom_face, top_face]`
//! (topology.rs:1666, verified by geometry: `_B` cap is at `y=0`, `_T` cap at
//! `y=height`).

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_cylinder_brep, Vec3};

/// `make_cylinder_brep` face order → face-name suffix (Side, Bottom, Top).
const FACE_SUFFIX: [&str; 3] = ["_S", "_B", "_T"];

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // radius is a magnitude (|radius|); height is DIRECTIONAL — a negative height
    // extends the cylinder to the -Y side (built with the magnitude, then spun a
    // half turn below). Diverges from the old `Math.abs` on height.
    let radius = ctx.number("radius")?.abs();
    let height = ctx.number("height")?;

    let mut solid = make_cylinder_brep(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        radius,
        height.abs(),
    )?;

    // Stamp the three face names in `make_cylinder_brep` order.
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("cylinder builder produced no shell")?
        .faces;
    if faces.len() != FACE_SUFFIX.len() {
        return Err(format!(
            "cylinder builder produced {} faces, expected 3",
            faces.len()
        ));
    }
    for (face, suffix) in faces.iter_mut().zip(FACE_SUFFIX) {
        face.name = Some(format!("{}{}", ctx.id, suffix));
    }

    // A negative height spins the +Y cylinder a half turn onto the -Y side
    // (proper rotation → names + winding preserved).
    let solid = if height < 0.0 {
        common::rotate_half_turn_about_x(solid)?
    } else {
        solid
    };

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
    "type": "P.CY",
    "shortName": "P.CY",
    "longName": "Primitive Cylinder",
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
            "hint": "Radius of the cylinder"
        },
        "height": {
            "type": "number",
            "default_value": 10,
            "hint": "Height of the cylinder along Y-axis"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}

