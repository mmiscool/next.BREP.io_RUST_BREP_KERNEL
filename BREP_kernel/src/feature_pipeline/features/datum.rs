//! D — Datum. Ported from the retired `DatiumFeature`.
//!
//! Produces no solid; it registers the three orthogonal base planes
//! `${id}:XY` / `${id}:XZ` / `${id}:YZ` as named scene FRAMES so a later sketch can
//! resolve its plane by name, fully headless. The whole datum
//! carries an optional TRS `transform` (position + `rotationEuler` in DEGREES;
//! scale is orientation-neutral for a plane and dropped, matching the retired
//! quaternion path).
//!
//! Child plane normals (local, before the group TRS): XY has mesh normal +Z; XZ
//! (`rotation.x = π/2`) → −Y;
//! YZ (`rotation.y = π/2`) → +X. The sketch frame is then `Frame::from_origin_normal`
//! (the worldUp convention) applied to each (group-origin, world-normal).

use crate::feature_pipeline::{FeatureContext, FeatureResult, Frame};
use crate::Vec3;
use serde_json::Value;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let transform = ctx.param("transform");
    let position = read_vec3(ctx, transform, "position", [0.0, 0.0, 0.0])?;
    let rotation_deg = read_vec3(ctx, transform, "rotationEuler", [0.0, 0.0, 0.0])?;
    let rotation = [
        rotation_deg[0].to_radians(),
        rotation_deg[1].to_radians(),
        rotation_deg[2].to_radians(),
    ];

    let origin = Vec3::new(position[0], position[1], position[2]);
    let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
    for (suffix, local_normal) in [
        ("XY", Vec3::new(0.0, 0.0, 1.0)),
        ("XZ", Vec3::new(0.0, -1.0, 0.0)),
        ("YZ", Vec3::new(1.0, 0.0, 0.0)),
    ] {
        // Rotate the WHOLE plane frame rigidly by the datum rotation — not just
        // the normal. Derive the canonical in-plane axes from the UNROTATED
        // local normal (worldUp convention, `from_origin_normal`), THEN rotate
        // x/y/z together. This preserves the identity conventions AND makes all
        // three planes share ONE rigid rotation about the common origin, so they
        // track the transform gizmo's triad. (Previously only the normal was
        // rotated and x/y were re-derived from worldUp per-plane, discarding the
        // datum's roll and desyncing the quads at three different rotate angles.)
        let base = Frame::from_origin_normal(origin, local_normal)?;
        let frame = Frame {
            origin,
            x_axis: rotate_euler_xyz(base.x_axis, rotation),
            y_axis: rotate_euler_xyz(base.y_axis, rotation),
            z_axis: rotate_euler_xyz(base.z_axis, rotation),
        };
        result.frames.push((format!("{}:{suffix}", ctx.id), frame));
    }
    Ok(result)
}

/// Read a `[x, y, z]` component vector from a transform sub-field; each component
/// may be a number or an expression string (evaluated against the shared env).
fn read_vec3(
    ctx: &FeatureContext,
    transform: Option<&Value>,
    key: &str,
    default: [f64; 3],
) -> Result<[f64; 3], String> {
    let array = transform.and_then(|t| t.get(key)).and_then(|v| v.as_array());
    let mut out = default;
    if let Some(array) = array {
        for (index, slot) in out.iter_mut().enumerate() {
            if let Some(value) = array.get(index) {
                *slot = match value {
                    Value::Number(number) => number.as_f64().unwrap_or(*slot),
                    Value::String(source) if !source.trim().is_empty() => ctx
                        .env
                        .eval(source)
                        .map_err(|error| format!("datum: transform `{key}`: {error}"))?,
                    _ => *slot,
                };
            }
        }
    }
    Ok(out)
}

/// Apply an intrinsic XYZ Euler rotation (default `XYZ` order) to a vector — the
/// exact rotation-matrix composition for Euler order `XYZ`, so datum orientation
/// byte-matches the editor.
pub fn rotate_euler_xyz(v: Vec3, euler: [f64; 3]) -> Vec3 {
    let (c1, s1) = (euler[0].cos(), euler[0].sin());
    let (c2, s2) = (euler[1].cos(), euler[1].sin());
    let (c3, s3) = (euler[2].cos(), euler[2].sin());
    // Column-major basis image for Euler order 'XYZ'.
    let m00 = c2 * c3;
    let m01 = -c2 * s3;
    let m02 = s2;
    let m10 = c1 * s3 + c3 * s1 * s2;
    let m11 = c1 * c3 - s1 * s2 * s3;
    let m12 = -c2 * s1;
    let m20 = s1 * s3 - c1 * c3 * s2;
    let m21 = c3 * s1 + c1 * s2 * s3;
    let m22 = c1 * c2;
    Vec3::new(
        m00 * v.x + m01 * v.y + m02 * v.z,
        m10 * v.x + m11 * v.y + m12 * v.z,
        m20 * v.x + m21 * v.y + m22 * v.z,
    )
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// never offered from a selection (no reference inputs).
pub fn context_applicable(_probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    false
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "D",
    "shortName": "D",
    "longName": "Datium",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the datium feature"
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
            "hint": "Position and rotation. Scale is ignored — a plane has no scale (it is orientation-neutral), and the Scale row the form draws for this widget shape has no effect here."
        }
    }
})
}

