//! DF — Delete Face.
//!
//! Removes one or more transition faces (a chamfer, a constant-radius fillet, or
//! any simple 4-sided transition face) and heals the hole by extending and
//! re-intersecting the immediate neighbours (Golovanov §6.12). A selection that
//! is a PATCH rather than a set of strips — every face of a pocket, a boss, or
//! a through bore's wall — is healed the other way, by capping: those faces
//! have no neighbours to re-intersect, so the patch and the hole loops it was
//! sunk through are dropped and the feature is un-cut.
//! Underlying kernel fn: [`crate::delete_faces_and_heal`] takes the WHOLE
//! selection and returns a fresh solid guaranteed to `validate()` — it picks
//! the cap lane or the one-at-a-time chain from the selection's own shape, so
//! this feature hands it every resolved id at once rather than chaining here.
//! A selection that is BOTH — four counterbored holes and the block's corner
//! fillets, say — is split there into its edge-connected pieces and each piece
//! takes its own lane, which is another reason the whole selection has to
//! arrive in one call.
//!
//! Reference resolution (contract rule 1): `faces` is a `reference_selection` of
//! FACE names — each resolves against the live scene-map to `(handle, face_id)`.
//! A miss is an `unresolved` entry (the caller repairs + re-dispatches), never a panic.
//!
//! Behaviour mapping — a SOFT
//! no-op (a warn, `{ added: [], removed: [] }`) for an empty selection or
//! faces spanning multiple solids, and only THROWS when the kernel heal itself
//! fails. Matched here: multi-solid / empty selection → an empty result (no
//! error, loop continues); a `delete_faces_and_heal` `Err` → `ctx.fail` (halt).
//!
//! Name fidelity (contract rule 2): the result
//! reuses the TARGET solid's name (`result.name = targetSolid.name`) and the
//! target goes in `removed`; the healed solid keeps every OTHER face's name (only
//! the deleted face's name disappears). Names are collected, never
//! re-stamped — `delete_faces_and_heal` preserves the survivors' ids and names.

use std::collections::HashSet;

use crate::feature_pipeline::features::common;
use crate::feature_pipeline::{AddedSolid, FeatureContext, FeatureRefusal, FeatureResult};
use crate::delete_faces_and_heal;

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    match build(ctx) {
        Ok(result) => result,
        Err(error) => ctx.fail(error),
    }
}

fn build(ctx: &FeatureContext) -> Result<FeatureResult, FeatureRefusal> {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // `faces` is a reference_selection of FACE names (contract rule 1).
    let names = common::reference_name_array(ctx.param("faces"));
    if names.is_empty() {
        return Ok(result); // Soft no-op — nothing selected.
    }

    // Resolve every face by EXACT name; misses recorded (not fatal).
    let mut resolved: Vec<(u32, u64)> = Vec::new();
    for name in &names {
        match ctx.scene.resolve_face(name) {
            Some(face_ref) => resolved.push((face_ref.handle, face_ref.face_id)),
            None => result.unresolved.push(name.clone()),
        }
    }
    if resolved.is_empty() {
        return Ok(result); // Only unresolved names — the caller repairs + re-dispatches.
    }

    // All selected faces must live on ONE solid (the caller aborts with a warn otherwise).
    let handles: HashSet<u32> = resolved.iter().map(|(handle, _)| *handle).collect();
    if handles.len() > 1 {
        return Ok(result); // Soft no-op — faces from multiple solids.
    }
    let target_handle = resolved[0].0;
    // The feature proceeds on the faces that resolved: a miss is reported as
    // a typed partial fulfilment beside `unresolved`, never silently.
    result.note_partial_resolution(&names);

    // Dedup the face ids (a name listed twice, or two names for one id) preserving
    // order — the chain heals each id once.
    let mut seen: HashSet<u64> = HashSet::new();
    let face_ids: Vec<u64> = resolved
        .iter()
        .map(|(_, face_id)| *face_id)
        .filter(|face_id| seen.insert(*face_id))
        .collect();

    // The result reuses the TARGET solid's name;
    // find it by reversing the scene-map (one name per resident handle).
    let target_name = ctx
        .scene
        .solids
        .iter()
        .find(|(_, handle)| **handle == target_handle)
        .map(|(name, _)| name.clone())
        .ok_or("delete_face: target solid has no scene-map name")?;

    // Heal off a clone of the resident target (short borrow → owned). The whole
    // selection goes in one call: a pocket's walls and floor are ONE patch and
    // only cap together, and the kernel needs to see them together to say so.
    let solid = crate::with_registered_solid_str(target_handle, |solid| Ok(solid.clone()))?;
    let solid = delete_faces_and_heal(&solid, &face_ids)?;

    // Collect the healed survivors' names (the deleted face's name is gone).
    let face_names = common::collect_face_names(&solid);
    let edge_names = common::collect_edge_names(&solid);
    let handle = crate::register_solid_value(solid);
    result.added.push(AddedSolid {
        handle,
        name: target_name.clone(),
        face_names,
        edge_names,
        ..AddedSolid::default()
    });
    // The target is consumed; the loop reuses its name for the result.
    result.removed.push(target_name);
    Ok(result)
}



/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "DF",
    "shortName": "DF",
    "longName": "Delete Face",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier for the delete face feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Select the face(s) to remove: a transition face (chamfer / fillet / simple 4-sided face), whose neighbours re-intersect to heal the hole, or every face of a pocket / boss / bore, which is capped by the face it was sunk through. Both kinds can be picked together — each feature in the selection is healed the way its own shape asks for"
        }
    }
})
}

