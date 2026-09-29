//! P.PY — Primitive Pyramid. Ported from the retired `Primitive Pyramid` feature.
//!
//! The feature params map `bL=baseSideLength`, `s=sides`, `h=height`; the solid is
//! built by `make_pyramid_brep((0,0,0), sideLength, height, sides)` (a regular
//! n-gon base in the z=0 plane, apex at +z=height) and a fixed legacy-frame
//! transform maps it to be centered on the origin with its base plane at
//! `y=-height/2` and apex at `y=+height/2`.
//!
//! Name fidelity (contract rule 2): the solid is named `${featureID}`; face 0 is
//! `${featureID}_Base` and each subsequent side face `i` is `${featureID}_S[i]`.
//! `make_pyramid_brep` pushes the base face first, then
//! the `sideCount` side faces in order (topology.rs:1471-1475), and the rigid
//! legacy-frame transform preserves that face order.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_pyramid_brep, transform_brep, AffineTransform, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // sideLength is a magnitude; height is DIRECTIONAL. The pyramid is CENTERED on
    // Y (base at -h/2, apex at +h/2), so a negative height flips the apex to point
    // DOWN — realized by building the |height| pyramid, then a half-turn about X
    // (a reflection would invert winding; the half turn is a proper rotation).
    // Diverges from the old `Math.abs` on height.
    let side_length = ctx.number("baseSideLength")?.abs();
    let height_signed = ctx.number("height")?;
    let height = height_signed.abs();
    // sideCount = max(3, floor(sides)); a non-finite `sides` is rejected below.
    let sides_raw = ctx.number("sides")?;
    if !sides_raw.is_finite() {
        return Err("param `sides` must be a finite number".into());
    }
    let side_count = sides_raw.floor().max(3.0) as usize;

    let base = make_pyramid_brep(Vec3::new(0.0, 0.0, 0.0), side_length, height, side_count)?;

    // The legacy-frame transform (row-major); det = +1 (a rigid
    // rotation about X plus a translation) so `reverse_orientation` is false.
    let legacy_frame = AffineTransform::new([
        1.0, 0.0, 0.0, -side_length / 2.0,
        0.0, 0.0, 1.0, -height / 2.0,
        0.0, -1.0, 0.0, side_length / 2.0,
        0.0, 0.0, 0.0, 1.0,
    ])?;
    let mut solid = transform_brep(&base, legacy_frame, false)?;

    // Stamp face names in `make_pyramid_brep` order: face 0 → `_Base`, then each
    // side face `i` → `_S[i]`. The transform preserved this order.
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("pyramid builder produced no shell")?
        .faces;
    let expected = side_count + 1;
    if faces.len() != expected {
        return Err(format!(
            "pyramid builder produced {} faces, expected {expected}",
            faces.len()
        ));
    }
    faces[0].name = Some(format!("{}_Base", ctx.id));
    for (index, face) in faces[1..].iter_mut().enumerate() {
        face.name = Some(format!("{}_S[{index}]", ctx.id));
    }

    // A negative height flips the apex to point down (half turn about X → names +
    // winding preserved; the centered pyramid maps base↔apex side).
    let solid = if height_signed < 0.0 {
        common::rotate_half_turn_about_x(solid)?
    } else {
        solid
    };

    // Bake the `transform` param on the legacy-frame solid BEFORE the boolean;
    // the bake preserves the stamped names.
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
    "type": "P.PY",
    "shortName": "P.PY",
    "longName": "Primitive Pyramid",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "baseSideLength": {
            "type": "number",
            "default_value": 10,
            "hint": "Side length of the regular base polygon"
        },
        "sides": {
            "type": "number",
            "default_value": 4,
            "hint": "Number of sides for the base polygon (min 3)"
        },
        "height": {
            "type": "number",
            "default_value": 10,
            "hint": "Height of the pyramid along Y-axis"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}

