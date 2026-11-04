//! P.CO — Primitive Cone. Ported from the retired `Primitive Cone` feature.
//!
//! The cone is aligned along +Y with its base at `y=0` and top at `y=height`. The
//! feature params `r1=radiusTop`, `r2=radiusBottom`, `h=height` map to
//! `make_cone_brep((0,0,0),(0,1,0), radius_bottom=|r2|, radius_top=max(0,|r1|),
//! height=|h|)`.
//!
//! A ZERO bottom radius (tip at the origin, rim at `y=height`) is the same solid
//! as the apex cone built from the far end with the axis reversed:
//! `make_cone_brep((0,h,0),(0,-1,0), radius_bottom=|r1|, radius_top=0, h)`. The
//! builder only knows the apex at its `radius_top` end, so the feature maps the
//! inverted request onto it (user bug 2026-11-02: every zero-bottom cone failed
//! with `makeConeSolid: invalid radius or height`). The side surface's `v=0` then
//! sits at the rim rather than at `y=0`.
//!
//! Name fidelity (contract rule 2): the solid is named `${featureID}`; its faces
//! are `${featureID}` + `_S` (Side) plus the caps that exist: `_B` (Bottom, at
//! `y=0`) and `_T` (Top, at `y=height`). `make_cone_brep`'s shell face order is
//! `[side, cap]` for an apex cone and `[side, bottom, top]` for a frustum
//! (analytic_topology.rs:316/458), so we decide the naming by the ACTUAL face
//! count rather than on `|r1| > 0`: the two agree everywhere except the
//! pathological `0 < |r1| <= 1e-7` window, where `make_cone_brep` itself emits an
//! apex cone (only 2 faces) so a 3-name array would be dangling — face-count
//! naming is the geometry-consistent choice. An inverted apex cone's single cap
//! is its top, so it is `_T`.

use crate::feature_pipeline::features::{common, transform_bake};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_cone_brep, Vec3};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

/// `make_cone_brep` collapses a top radius at or below this onto the apex
/// (analytic_topology.rs:357); a bottom radius in the same window is the
/// inverted apex cone.
const APEX_RADIUS: f64 = 1e-7;

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    // radiusTop/radiusBottom are magnitudes; height is DIRECTIONAL — a negative
    // height extends the cone to the -Y side. It's built with the magnitude then
    // spun a half turn, which keeps radiusBottom at the origin anchor (y=0) and puts
    // radiusTop at the -|h| tip. Diverges from the old `Math.abs` on height.
    let radius_top = ctx.number("radiusTop")?.abs().max(0.0);
    let radius_bottom = ctx.number("radiusBottom")?.abs();
    let height = ctx.number("height")?;
    if height.abs() <= 0.0 {
        return Err("cone: height must be non-zero".into());
    }
    if radius_bottom <= APEX_RADIUS && radius_top <= APEX_RADIUS {
        return Err("cone: at least one of radiusTop / radiusBottom must be positive".into());
    }

    // A zero bottom radius is the apex cone seen from the other end: build it
    // from (0,|h|,0) down the -Y axis so the tip lands on the origin anchor and
    // the rim at y=|h|, exactly where radiusTop belongs.
    let inverted = radius_bottom <= APEX_RADIUS;
    let mut solid = if inverted {
        make_cone_brep(
            Vec3::new(0.0, height.abs(), 0.0),
            Vec3::new(0.0, -1.0, 0.0),
            radius_top,
            0.0,
            height.abs(),
        )?
    } else {
        make_cone_brep(
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            radius_bottom,
            radius_top,
            height.abs(),
        )?
    };

    // Name by the actual face count: 2 → [_S, cap] (apex; the cap is `_B` at y=0,
    // or `_T` at y=height for the inverted cone), 3 → [_S, _B, _T] (frustum).
    let faces = &mut solid
        .shells
        .get_mut(0)
        .ok_or("cone builder produced no shell")?
        .faces;
    let suffixes: &[&str] = match (faces.len(), inverted) {
        (2, false) => &["_S", "_B"],
        (2, true) => &["_S", "_T"],
        (3, false) => &["_S", "_B", "_T"],
        (3, true) => return Err("cone builder produced a frustum for a zero bottom radius".into()),
        (other, _) => return Err(format!("cone builder produced {other} faces, expected 2 or 3")),
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
    "ribbonPath": "Home/Primitives/Cone",
    "commandSize": "Compact",
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
            "hint": "Base radius of the cone (tip if 0)"
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

