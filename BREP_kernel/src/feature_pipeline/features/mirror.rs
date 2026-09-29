//! Reflect named solids across a face, datum plane, or explicit point/normal.
//!
//! Reflection reverses handedness; `mirror_brep` reverses topology orientation
//! to keep normals outward. Negating the plane normal only affects the sign of
//! `offsetDistance`. Missing named planes are reported as unresolved; an absent
//! plane is a no-op.
//!
//! Sources are retained. Copies are named `${featureID}:${sourceName}:M`, with
//! namespaced face and authored-edge names. Topology edge names are regenerated
//! from the renamed adjacent faces.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::{mirror_brep, Vec3};

/// A resolved mirror plane: a point on it and its (unit) normal.
struct Plane {
    point: Vec3,
    normal: Vec3,
}

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, String> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // `solids` is a reference_selection of solid names (contract rule 1).
    let names = common::reference_name_array(ctx.param("solids"));
    if names.is_empty() {
        return Ok(result);
    }

    let resolved = common::resolve_solid_names(ctx.scene, &names, &mut result.unresolved);

    // Resolve the mirror plane (may add its own `unresolved` entry).
    let plane = match resolve_plane(ctx, &mut result)? {
        Some(plane) => plane,
        None => return Ok(result),
    };

    if resolved.is_empty() {
        return Ok(result);
    }

    // `featureID = inputParams.featureID || 'MIRROR'`.
    let feature_id = if ctx.id.is_empty() {
        "MIRROR".to_string()
    } else {
        ctx.id.clone()
    };
    for (source_name, handle) in &resolved {
        // Short registry borrow: reflect and hand back the owned mirrored solid.
        let mut mirrored = crate::with_registered_solid_str(*handle, |solid| {
            mirror_brep(solid, plane.point, plane.normal)
        })?;

        // Namespace the mirrored solid's names `::${featureID}` so they can't
        // collide with the (kept) source's — faces + authored edges retagged,
        // topology edges re-derived from the retagged faces (the edge-naming
        // contract). Without this the reflected edges kept the source's exact
        // names (`Box_NX|Box_NZ[0]`), so two solids shared edge names.
        common::namespace_copy_names(&mut mirrored, &feature_id);
        let name = format!("{feature_id}:{source_name}:M");
        let face_names = common::collect_face_names(&mirrored);
        let edge_names = common::collect_edge_names(&mirrored);
        let new_handle = crate::register_solid_value(mirrored);
        result.added.push(AddedSolid {
            handle: new_handle,
            name,
            face_names,
            edge_names,
            ..AddedSolid::default()
        });
    }

    // Mirror is additive — the source stays resident (removed = []).
    Ok(result)
}

/// Resolve the reflection plane per the priority in the module doc. Returns
/// `Ok(None)` when there is no usable plane (a recorded `unresolved` face
/// reference, or simply no plane params — a no-op).
fn resolve_plane(
    ctx: &FeatureContext,
    result: &mut FeatureResult,
) -> Result<Option<Plane>, String> {
    let offset = match ctx.param("offsetDistance") {
        None | Some(serde_json::Value::Null) => 0.0,
        Some(_) => ctx.number("offsetDistance")?,
    };

    // (1) `mirrorPlane` reference (schema filter `["FACE","PLANE"]`) resolved via
    // the shared plane-reference resolver — a solid FACE or a datum/construction
    // PLANE frame, treated identically (the fix for "mirror about a datum plane
    // does nothing").
    if let Some(name) = common::single_reference_name(ctx.param("mirrorPlane")) {
        return match common::resolve_plane_reference(ctx, &name) {
            Some(plane_ref) => {
                let (point, normal) = plane_ref.point_normal()?;
                Ok(Some(Plane {
                    point: point.add(normal.scale(offset)),
                    normal,
                }))
            }
            None => {
                // Neither a face nor a plane → `unresolved` (contract rule 1).
                result.unresolved.push(name);
                Ok(None)
            }
        };
    }

    // (2) explicit orientation params `normal` (+ optional `point`).
    if ctx.param("normal").is_some() {
        let n = vec3_param(ctx, "normal", [0.0, 0.0, 1.0])?;
        let p = vec3_param(ctx, "point", [0.0, 0.0, 0.0])?;
        let normal = Vec3::new(n[0], n[1], n[2]).normalized()?;
        let point = Vec3::new(p[0], p[1], p[2]).add(normal.scale(offset));
        return Ok(Some(Plane { point, normal }));
    }

    // (3) no plane — a no-op.
    Ok(None)
}

/// Read a vec3 param (array `[x,y,z]` or object `{x,y,z}`); each component may be
/// a number, an expression string, or null/missing (→ default).
fn vec3_param(
    ctx: &FeatureContext,
    key: &str,
    default: [f64; 3],
) -> Result<[f64; 3], String> {
    let Some(value) = ctx.param(key) else {
        return Ok(default);
    };
    let component = |slot: Option<&serde_json::Value>, fallback: f64| -> Result<f64, String> {
        match slot {
            None | Some(serde_json::Value::Null) => Ok(fallback),
            Some(serde_json::Value::Number(number)) => number
                .as_f64()
                .ok_or_else(|| format!("param `{key}` has a non-finite component")),
            Some(serde_json::Value::String(source)) => ctx
                .env
                .eval(source)
                .map_err(|error| format!("param `{key}`: {error}")),
            Some(other) => Err(format!(
                "param `{key}` component must be a number or expression, found {other}"
            )),
        }
    };
    if let Some(array) = value.as_array() {
        return Ok([
            component(array.first(), default[0])?,
            component(array.get(1), default[1])?,
            component(array.get(2), default[2])?,
        ]);
    }
    if value.is_object() {
        return Ok([
            component(value.get("x"), default[0])?,
            component(value.get("y"), default[1])?,
            component(value.get("z"), default[2])?,
        ]);
    }
    Err(format!("param `{key}` must be a vec3 array or object"))
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected solids drive `solids`, a face `mirrorPlane`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0 || probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "M",
    "shortName": "M",
    "longName": "Mirror",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the mirror feature"
        },
        "solids": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more solids to mirror"
        },
        "mirrorPlane": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "PLANE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Select the plane or face to mirror about"
        },
        "offsetDistance": {
            "type": "number",
            "default_value": 0,
            "hint": "Offset distance for the mirror"
        }
    }
})
}

