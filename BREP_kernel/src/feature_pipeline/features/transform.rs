//! Apply translation, intrinsic XYZ rotation, and scale about an origin or
//! vertex-AABB-center pivot: `T(translate) · T(pivot) · R · S · T(-pivot)`.
//!
//! Solids are stored in world coordinates. The transform preserves face and edge
//! names. Replacement keeps the source solid's name; copy mode keeps the source
//! and adds `${sourceName}_${id}`. Invalid transforms are rejected by the kernel.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureResult};
use crate::{transform_brep, AffineTransform, BrepSolid};

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
        // Nothing selected — a pass-through no-op (`return { added: [], removed: [] }`).
        return Ok(result);
    }

    let resolved = common::resolve_solid_names(ctx.scene, &names, &mut result.unresolved);
    if resolved.is_empty() {
        // No source resolved: return without a solid; `unresolved` signals repair.
        return Ok(result);
    }

    // TRS params (each component may be a literal or an expression string).
    let translate = vec3_param(ctx, "position", [0.0, 0.0, 0.0])?;
    let rotate_deg = vec3_param(ctx, "rotationEuler", [0.0, 0.0, 0.0])?;
    let scale = vec3_param(ctx, "scale", [1.0, 1.0, 1.0])?;
    let rotate_rad = [
        rotate_deg[0].to_radians(),
        rotate_deg[1].to_radians(),
        rotate_deg[2].to_radians(),
    ];
    let pivot_bbox = matches!(
        ctx.param("pivot").and_then(|value| value.as_str()),
        Some(mode) if mode.eq_ignore_ascii_case("BBOX_CENTER")
    );
    let copy = ctx
        .param("copy")
        .and_then(|value| value.as_bool())
        .unwrap_or(false);

    for (source_name, handle) in &resolved {
        // Short registry borrow: build + apply the transform, hand back the owned
        // transformed solid (never hold the borrow across registration).
        let mut transformed = crate::with_registered_solid_str(*handle, |solid| {
            let pivot = if pivot_bbox {
                bbox_center(solid)
            } else {
                [0.0, 0.0, 0.0]
            };
            let matrix = common::compose_trs_matrix(translate, rotate_rad, scale, pivot);
            let transform = AffineTransform::new(matrix)?;
            transform_brep(solid, transform, false)
        })?;

        // `replace = !copy`: replace keeps the SOURCE name (in-place
        // supersession — the source is removed, so its names carry through with no
        // collision); copy adds `${sourceName}_${inputParams.id}` and KEEPS the
        // source, so the copy's names must be namespaced `::${id}` (faces +
        // authored edges retagged, topology edges re-derived) or its edges would
        // share the source's exact names — the same bug mirror/pattern had.
        let name = if copy {
            common::namespace_copy_names(&mut transformed, &ctx.id);
            format!("{}_{}", source_name, ctx.id)
        } else {
            source_name.clone()
        };
        let face_names = common::collect_face_names(&transformed);
        let edge_names = common::collect_edge_names(&transformed);
        let new_handle = crate::register_solid_value(transformed);
        result.added.push(AddedSolid {
            handle: new_handle,
            name,
            face_names,
            edge_names,
            ..AddedSolid::default()
        });
    }

    // The default (`copy == false`) REPLACES: the consumed sources are removed and
    // the loop frees their handles. `copy == true` keeps them.
    if !copy {
        result.removed = resolved.into_iter().map(|(name, _)| name).collect();
    }
    Ok(result)
}

/// The centre of the solid's exact-vertex axis-aligned bounding box. This is an
/// approximation of the `bboxCenterWorld` (which spans tessellated vertices);
/// exact for straight-edged solids, slightly loose for curved ones.
pub(crate) fn bbox_center(solid: &BrepSolid) -> [f64; 3] {
    let mut min = [f64::INFINITY; 3];
    let mut max = [f64::NEG_INFINITY; 3];
    for vertex in &solid.vertices {
        let p = [vertex.point.x, vertex.point.y, vertex.point.z];
        for axis in 0..3 {
            min[axis] = min[axis].min(p[axis]);
            max[axis] = max[axis].max(p[axis]);
        }
    }
    if !min[0].is_finite() {
        return [0.0, 0.0, 0.0];
    }
    [
        0.5 * (min[0] + max[0]),
        0.5 * (min[1] + max[1]),
        0.5 * (min[2] + max[2]),
    ]
}

/// Read a canonical transform-group vec3 param via the shared value reader ([`common::vec3_from_value`]).
fn vec3_param(
    ctx: &FeatureContext,
    key: &str,
    default: [f64; 3],
) -> Result<[f64; 3], String> {
    common::vec3_from_value(ctx.env, ctx.param("transform").and_then(|t| t.get(key)), &format!("transform.{key}"), default)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected solids drive `solids`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.solids > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "XFORM",
    "shortName": "XFORM",
    "longName": "Transform",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the transform feature"
        },
        "solids": {
            "type": "reference_selection",
            "selectionFilter": [
                "SOLID"
            ],
            "multiple": true,
            "default_value": [],
            "hint": "Select solids to transform"
        },
        "pivot": {
            "type": "options",
            "options": [
                "ORIGIN",
                "BBOX_CENTER"
            ],
            "default_value": "ORIGIN",
            "hint": "Pivot point for rotation/scale"
        },
        "transform": {
            "type": "transform",
            "default_value": {
                "position": [0, 0, 0],
                "rotationEuler": [0, 0, 0],
                "scale": [1, 1, 1]
            },
            "hint": "Move, rotate and scale selected solids"
        },
        "copy": {
            "type": "boolean",
            "default_value": false,
            "label": "Copy",
            "hint": "Create a new transformed copy; keep the original"
        }
    }
})
}

