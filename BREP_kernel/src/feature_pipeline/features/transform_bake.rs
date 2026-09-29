//! Primitive `transform` param bake.
//!
//! With NO `reference` set, the transform reduces to a TRS compose about the world
//! origin — `position`, `rotationEuler` (degrees, Euler order `XYZ`), and `scale`,
//! i.e.
//!
//! ```text
//!   M = T(position) · R(EulerXYZ, degrees) · S(scale)
//! ```
//!
//! applied about the WORLD ORIGIN: scale first, then rotate, then translate.
//! The rotation is the intrinsic `'XYZ'` Euler (= Rx·Ry·Rz), already ported as
//! [`rotate_euler_xyz`]; the linear part's columns are `R·e_j · scale[j]`,
//! matching `transform.rs::compose_trs_matrix` with `pivot = [0,0,0]`.
//!
//! Param reading goes through the ONE shared vec3 reader,
//! [`common::vec3_from_value`] — the same reader XFORM, the port and an assembly
//! component use. A missing component (absent / `null`) takes that index's
//! default; a number passes through; a STRING is an EXPRESSION evaluated against
//! the shared [`Env`](crate::feature_pipeline::Env), so `position: ["W/2", 0, 0]`
//! places the primitive at `W/2` exactly like `sizeX: "W/2"` sizes it. An
//! expression that does not resolve is an ERROR, never a silent fallback to the
//! default — that fallback used to drill a bore at the origin and report success.
//!
//! A `transform.reference` (start reference: face/edge/vertex/plane/datum)
//! re-bases the delta on a scene object's frame — that resolution is
//! NOT migrated, so a non-empty reference errors loudly
//! (no-fallback). NOTE this is stricter than the old
//! `require_identity_transform`, which silently passed an identity-TRS
//! transform even when a resolving reference would have moved the solid.
//!
//! Geometry: [`crate::transform_brep`] rewrites NURBS control points — EXACT
//! under any affine map, including non-uniform scale on curved primitives — and
//! never touches `face.name`/`edge.name`, so stamped names carry through
//! byte-identically. A negative-determinant scale (odd count of negative
//! components) is baked as an exact reflection (`reverse_orientation = true`,
//! the `mirror_brep` path) instead of the legacy-mesh fallback; a singular
//! scale errors loudly via [`crate::AffineTransform::new`].

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::features::datum::rotate_euler_xyz;
use crate::feature_pipeline::FeatureContext;
use crate::{transform_brep, AffineTransform, BrepSolid, Vec3};

/// Bake the feature's `transform` param onto a freshly built (and already
/// name-stamped) primitive `solid`. Identity (or absent) transform → the solid
/// passes through untouched, bit-identical.
pub fn apply(ctx: &FeatureContext, solid: BrepSolid) -> Result<BrepSolid, String> {
    let Some(affine) = trs_transform(ctx)? else {
        return Ok(solid);
    };
    // Rotation det is +1, so the sign is scale[0]·scale[1]·scale[2]: a negative
    // (reflecting) scale is baked exactly with orientation reversal.
    let reflection = affine.determinant3() < 0.0;
    transform_brep(&solid, affine, reflection)
        .map_err(|error| format!("primitive transform bake: {error}"))
}

/// The feature's `transform` param as the affine map the bake applies —
/// `None` when the param is absent or the identity. Exposed for features that
/// place CURVES rather than solids (the helix), so they read the param by the
/// one convention above. Errors exactly as the bake does on a `reference`.
pub fn trs_transform(ctx: &FeatureContext) -> Result<Option<AffineTransform>, String> {
    let Some(transform) = ctx.param("transform") else {
        return Ok(None);
    };

    // A start `reference` re-bases the TRS on a scene object's frame; that
    // resolution is not migrated. Check BEFORE the identity short-circuit: an
    // identity delta with a resolving reference still moves the solid.
    if let Some(name) = reference_name(transform.get("reference")) {
        return Err(format!(
            "primitive transform `reference` ('{name}') is not yet migrated to the Rust pipeline \
             (the base frame could not be resolved)"
        ));
    }

    let vec3 = |key: &str, default: [f64; 3]| {
        common::vec3_from_value(ctx.env, transform.get(key), &format!("transform.{key}"), default)
    };
    let position = vec3("position", [0.0, 0.0, 0.0])?;
    let rotation_deg = vec3("rotationEuler", [0.0, 0.0, 0.0])?;
    let scale = vec3("scale", [1.0, 1.0, 1.0])?;

    if position == [0.0, 0.0, 0.0] && rotation_deg == [0.0, 0.0, 0.0] && scale == [1.0, 1.0, 1.0] {
        return Ok(None);
    }

    let rotation = [
        rotation_deg[0].to_radians(),
        rotation_deg[1].to_radians(),
        rotation_deg[2].to_radians(),
    ];
    // Linear part columns: R·e_j scaled by scale[j] (TRS compose).
    let col_x = rotate_euler_xyz(Vec3::new(1.0, 0.0, 0.0), rotation).scale(scale[0]);
    let col_y = rotate_euler_xyz(Vec3::new(0.0, 1.0, 0.0), rotation).scale(scale[1]);
    let col_z = rotate_euler_xyz(Vec3::new(0.0, 0.0, 1.0), rotation).scale(scale[2]);
    let matrix = [
        col_x.x, col_y.x, col_z.x, position[0], //
        col_x.y, col_y.y, col_z.y, position[1], //
        col_x.z, col_y.z, col_z.z, position[2], //
        0.0, 0.0, 0.0, 1.0,
    ];
    AffineTransform::new(matrix)
        .map(Some)
        .map_err(|error| format!("primitive transform bake: {error}"))
}

/// `sanitizeTransformReference` name extraction: a string reference is its
/// trimmed self; an object reference is its trimmed `name` (falling back to
/// `id`, per `normalizeReferenceName`). Empty → no reference.
fn reference_name(value: Option<&serde_json::Value>) -> Option<String> {
    let raw = match value? {
        serde_json::Value::String(text) => text.as_str(),
        serde_json::Value::Object(map) => map
            .get("name")
            .and_then(|v| v.as_str())
            .filter(|s| !s.trim().is_empty())
            .or_else(|| map.get("id").and_then(|v| v.as_str()))?,
        _ => return None,
    };
    let trimmed = raw.trim();
    (!trimmed.is_empty()).then(|| trimmed.to_string())
}

