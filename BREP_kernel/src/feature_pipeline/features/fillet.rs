//! Rolling-ball edge blends, with optional taper through `radiusEnd`.
//!
//! Edge and face selections resolve to one solid and are blended together,
//! including shared convex corners. Cross-solid selections return no changes.
//!
//! The result retains the target's name. Generated faces use the shared
//! `{featureID || 'F'}:BLEND:{edgeName}` naming scheme, with corner patches
//! under `{base}:CORNER:{adjacent edge names}`.

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{FeatureContext, FeatureRefusal, FeatureResult};
use crate::{KernelRefusal, KernelStage};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let selection = common::resolve_blend_selection(ctx)?;
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());
    result.unresolved = selection.unresolved;

    // A cross-solid selection is a SOFT abort (warn + return empty).
    if selection.multi_solid {
        return Ok(result);
    }
    // A selection in which NOTHING resolves is refused by name.  Until
    // 2026-09-26 the misses rode along in `unresolved` and the feature
    // returned success for a document that blended no edge at all
    // (`Box_PY|Box_NZ[0]`, the faces in the wrong order, on the through-hole
    // case document): a no-op reported as a result.  A selection in which SOME
    // names resolve still blends those and reports the misses — that is the
    // engine's contract (rule 1: a miss is recorded in `unresolved`, annotated
    // with the name that replaced it, never a no-op with no signal), and two
    // reported documents rely on it: the 2026-09-10 mouth-over-wall and
    // collapsed-fillet reports both select `Box_PY|Box_PZ[0]`, which their own
    // r = 18 blend consumed, beside the live mouth edge.  Whether that partial
    // should refuse too is a contract question, not this feature's alone.
    let Some(target) = selection.target else {
        if result.unresolved.is_empty() {
            return Ok(result);
        }
        let mut refused = ctx.fail(unresolved_refusal("fillet", &result.unresolved));
        refused.unresolved = result.unresolved;
        return Ok(refused);
    };
    // The feature proceeds on the references that resolved: a miss is
    // reported as a typed partial fulfilment beside `unresolved`, never
    // silently (the engine annotates both with the rename hint).
    result.note_partial_resolution(&common::reference_names(ctx.param("edges")));

    let radius = ctx.number("radius")?;
    // A non-positive / non-finite radius is a soft no-op.
    if !(radius.is_finite() && radius > 0.0) {
        return Ok(result);
    }

    // `radiusEnd > 0 && |radiusEnd − radius| > 1e-9` selects the VARIABLE
    // (tapered) fillet.
    let taper = common::optional_number(ctx, "radiusEnd")?
        .filter(|value| value.is_finite() && *value > 0.0 && (*value - radius).abs() > 1e-9);

    // Names parallel the sampled edge points; the base also names corner patches.
    let blend_base = common::blend_face_base(&ctx.id);
    let blend_names: Vec<String> = target
        .edge_names
        .iter()
        .map(|edge_name| common::blend_face_name(&ctx.id, edge_name))
        .collect();
    let blended = crate::with_registered_solid_typed(target.handle, |solid| match taper {
        Some(radius_end) => crate::fillet_edges_variable(
            solid,
            &target.edge_points,
            Some(&blend_names),
            &[(0.0, radius), (1.0, radius_end)],
            false,
            Some(&blend_base),
        ),
        None => crate::fillet_edges(
            solid,
            &target.edge_points,
            Some(&blend_names),
            radius,
            false,
            Some(&blend_base),
        ),
    })
    // The blend's refusal keeps its class through the feature's wrap.
    .map_err(|error| error.with_message(|error| format!("fillet failed: {error}")))?;

    result.added.push(common::register_added(blended, &target.name));
    result.removed.push(target.name);
    Ok(result)
}


/// The refusal for a blend selection none of whose references resolve on
/// the model, shared with the chamfer feature: an invalid input, class
/// `unresolved_reference`, typed at the feature boundary.
pub(super) fn unresolved_refusal(feature: &str, unresolved: &[String]) -> KernelRefusal {
    let names: Vec<String> = unresolved.iter().map(|name| format!("`{name}`")).collect();
    KernelRefusal::input(
        KernelStage::Collect,
        "unresolved_reference",
        format!(
            "{feature}: none of the {} selected reference{} resolves on this model ({}); a blend \
             of nothing is not a result, so the selection is refused — repair the reference{} \
             and rerun",
            unresolved.len(),
            if unresolved.len() == 1 { "" } else { "s" },
            names.join(", "),
            if unresolved.len() == 1 { "" } else { "s" },
        ),
    )
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces/edges drive `edges`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "F",
    "shortName": "F",
    "longName": "Fillet",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the fillet feature"
        },
        "edges": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "EDGE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": null,
            "hint": "Select faces (or an edge) to fillet along shared edges"
        },
        "radius": {
            "type": "number",
            "step": 0.1,
            "default_value": 1,
            "hint": "Fillet radius"
        },
        "radiusEnd": {
            "type": "number",
            "step": 0.1,
            "default_value": 0,
            "hint": "Optional end radius for a VARIABLE (tapered) fillet. 0 = constant radius. When > 0 and different from radius, each selected edge tapers from radius at its start to this value at its end (kernel-exact path; corners are not rounded in variable mode)."
        }
    }
})
}

