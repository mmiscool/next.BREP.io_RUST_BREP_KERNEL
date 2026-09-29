//! Edge bevels: positive `distance2` selects an asymmetric bevel; otherwise
//! positive `angle` selects a distance-angle bevel, with a symmetric fallback.
//!
//! Edge and face selections must resolve to one solid; cross-solid selections
//! are errors. Only AUTO and INSET directions are supported.
//!
//! The result retains the target's name. Generated faces use fillet's shared
//! `{featureID || 'F'}:BLEND:{edgeName}` naming scheme, including its F fallback.

use std::f64::consts::PI;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    common::require_blend_direction(ctx, "chamfer")?;

    let selection = common::resolve_blend_selection(ctx)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.unresolved = selection.unresolved;

    // A cross-solid selection is rejected → hard error.
    if selection.multi_solid {
        return Err("chamfer requires the selected edges to belong to a single solid".into());
    }
    // A selection none of whose references resolve is refused by name (see
    // the fillet feature); a partial blends what resolved and reports the
    // misses, which stay in `unresolved` for the engine's rename hint.
    let Some(target) = selection.target else {
        if result.unresolved.is_empty() {
            return Ok(result);
        }
        let mut refused =
            ctx.fail(super::fillet::unresolved_refusal("chamfer", &result.unresolved));
        refused.unresolved = result.unresolved;
        return Ok(refused);
    };
    // A partial proceeds on what resolved and reports the miss as a typed
    // fulfilment (see the fillet feature).
    result.note_partial_resolution(&common::reference_names(ctx.param("edges")));

    let distance = ctx.number("distance")?;
    // A non-positive / non-finite distance is a soft no-op.
    if !(distance.is_finite() && distance > 0.0) {
        return Ok(result);
    }

    // Asymmetric selection: `distance2 > 0` → `d1 × d2`; else `angle > 0` (degrees)
    // → distance-angle.
    let distance2 = common::optional_number(ctx, "distance2")?.unwrap_or(0.0);
    let angle_deg = common::optional_number(ctx, "angle")?.unwrap_or(0.0);
    let d2 = (distance2.is_finite() && distance2 > 0.0).then_some(distance2);
    let angle_rad = (d2.is_none() && angle_deg.is_finite() && angle_deg > 0.0)
        .then(|| angle_deg * PI / 180.0);

    // Names parallel the sampled edge points; the base also names corner patches.
    let blend_base = common::blend_face_base(&ctx.id);
    let blend_names: Vec<String> = target
        .edge_names
        .iter()
        .map(|edge_name| common::blend_face_name(&ctx.id, edge_name))
        .collect();
    let blended = crate::with_registered_solid_typed(target.handle, |solid| {
        if let Some(d2) = d2 {
            crate::chamfer_edges_asymmetric(
                solid,
                &target.edge_points,
                Some(&blend_names),
                distance,
                d2,
                Some(&blend_base),
            )
        } else if let Some(angle) = angle_rad {
            crate::chamfer_edges_angle(
                solid,
                &target.edge_points,
                Some(&blend_names),
                distance,
                angle,
                Some(&blend_base),
            )
        } else {
            crate::fillet_edges(
                solid,
                &target.edge_points,
                Some(&blend_names),
                distance,
                true,
                Some(&blend_base),
            )
        }
    })
    // The blend's refusal keeps its class through the feature's wrap.
    .map_err(|error| error.with_message(|error| format!("chamfer failed: {error}")))?;

    result.added.push(common::register_added(blended, &target.name));
    result.removed.push(target.name);
    Ok(result)
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected edges/faces drive `edges`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "CH",
    "shortName": "CH",
    "longName": "Chamfer",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the chamfer feature"
        },
        "edges": {
            "type": "reference_selection",
            "selectionFilter": [
                "EDGE",
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": null,
            "hint": "Select edges or faces to apply the chamfer"
        },
        "distance": {
            "type": "number",
            "step": 0.1,
            "default_value": 1,
            "hint": "Chamfer distance. Symmetric (distance2 and angle both 0): the ROLLING-BALL RADIUS — the bevel joins the points a ball of this radius touches, which equals the setback along each face on a 90 degree planar edge and exceeds it on curved faces. Otherwise: the setback along the first face."
        },
        "distance2": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Optional second setback for an ASYMMETRIC (two-distance) chamfer, measured along the second face. 0 = symmetric. When > 0, the edge gets a lopsided distance × distance2 bevel (kernel-exact path; straight edges on planar faces)."
        },
        "angle": {
            "type": "number",
            "step": 1,
            "default_value": 0,
            "hint": "Optional chamfer angle in DEGREES between the chamfer face and the first face, for a distance-angle chamfer. 0 = unused. Ignored when distance2 > 0. The second setback is computed from distance and this angle (kernel-exact path)."
        },
        "direction": {
            "type": "options",
            "options": [
                "AUTO",
                "INSET"
            ],
            "default_value": "AUTO",
            "hint": "Choose the chamfer side automatically (AUTO) or force INSET. There is no OUTSET: it belonged to the removed legacy mesh pipeline and the direction gate rejects it."
        }
    }
})
}

