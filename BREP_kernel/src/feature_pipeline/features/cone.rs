//! P.CO — Primitive Cone. Ported from the retired `Primitive Cone` feature.
//!
//! The cone is aligned along +Y with its base at `y=0` and top at `y=height`. The
//! feature params `r1=radiusTop`, `r2=radiusBottom`, `h=height` map to
//! `make_cone_brep((0,0,0),(0,1,0), radius_bottom=|r2|, radius_top=max(0,|r1|),
//! height=|h|)`.
//!
//! Name fidelity (contract rule 2): the solid is named `${featureID}`; its faces
//! are `${featureID}` + `_S, _B` (Side, Bottom) — plus `_T` (Top) when a top cap
//! exists. `make_cone_brep`'s shell face order is `[side, bottom]` for the apex
//! cone and `[side, bottom, top]` for a frustum (analytic_topology.rs:316/458), so
//! we decide the naming by the ACTUAL face count rather than on `|r1| > 0`: the
//! two agree everywhere except the pathological
//! `0 < |r1| <= 1e-7` window, where `make_cone_brep` itself emits an apex cone
//! (only 2 faces) so a 3-name array would be dangling — face-count naming is the
//! geometry-consistent choice.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_cone_brep, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // radiusTop/radiusBottom are magnitudes; height is DIRECTIONAL — a negative
    // height extends the cone to the -Y side. It's built with the magnitude then
    // spun a half turn, which keeps radiusBottom at the origin anchor (y=0) and puts
    // radiusTop at the -|h| tip. Diverges from the old `Math.abs` on height.
    let radius_top = ctx.number("radiusTop")?.abs().max(0.0);
    let radius_bottom = ctx.number("radiusBottom")?.abs();
    let height = ctx.number("height")?;

    let mut solid = make_cone_brep(
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(0.0, 1.0, 0.0),
        radius_bottom,
        radius_top,
        height.abs(),
    )?;

    // Name by the actual face count: 2 → [_S, _B] (apex), 3 → [_S, _B, _T] (frustum).
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("cone builder produced no shell")?
        .faces;
    let suffixes: &[&str] = match faces.len() {
        2 => &["_S", "_B"],
        3 => &["_S", "_B", "_T"],
        other => return Err(format!("cone builder produced {other} faces, expected 2 or 3")),
    };
    for (face, suffix) in faces.iter_mut().zip(suffixes) {
        face.name = Some(format!("{}{}", ctx.id, suffix));
    }

    // A negative height spins the +Y cone a half turn onto the -Y side
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
    "type": "P.CO",
    "shortName": "P.CO",
    "longName": "Primitive Cone",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the feature"
        },
        "radiusTop": {
            "type": "number",
            "default_value": 5,
            "hint": "Top radius of the cone (tip if 0)"
        },
        "radiusBottom": {
            "type": "number",
            "default_value": 10,
            "hint": "Base radius of the cone"
        },
        "height": {
            "type": "number",
            "default_value": 10,
            "hint": "Height of the cone along Y-axis"
        },
        "transform": common::primitive_transform_schema(),
        "boolean": common::optional_boolean_schema()
    }
})
}

