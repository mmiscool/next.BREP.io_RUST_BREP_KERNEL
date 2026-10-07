//! O.F — Offset Face. A CONSTRUCTION feature: the record contract is EMPTY.
//!
//! This feature builds three `SKETCH` groups (the offset face sheet + its boundary
//! edges/vertices, named `${featureId}:${label}` / `...:PROFILE`) and returns
//! `{ added: [SKETCH groups], removed: [] }`. It NEVER adds or removes a SOLID —
//! the source solid is untouched (the browser volume expectations pin exactly
//! this: `test_offsetFace_preserves_individual_edges` expects the single record
//! `P.CU1` at its unmodified 125 mm³).
//!
//! In the cutover pipeline a `FeatureResult.added` entry becomes a caller SOLID record
//! (`PartHistory.#runHistoryImpl` wraps every added handle via
//! `Solid.fromKernelHandle`), so carrying the offset sheet as an `AddedSolid`
//! would fabricate a solid the established convention never had. Hence: resolve the
//! references (contract rule 1 — a name miss is `unresolved`, never a halt), add
//! nothing, remove nothing. The SKETCH display group itself is a display-side
//! artifact (like datum/plane/sketch visuals) built by the display lane, not
//! by the geometry pipeline; the kernel's curved-correct carrier op
//! ([`crate::offset_face_carrier`]) stays reachable through the one-shot wasm API
//! the retired builder called (`resolveOffsetFaceCarrier` →
//! `tryCreateRustOffsetFaceCarrier`).
//!
//! Param fidelity:
//! - `inputParams.faces` — a `reference_selection` (FACE names), `multiple`.
//!   Empty → no-op (`{ added: [], removed: [] }`).
//! - `inputParams.distance` — must be FINITE (zero accepted, unlike O.S / THK);
//!   non-finite → no-op. The value itself is consumed by the caller's display builder.

use crate::feature_pipeline::features::common;

use crate::feature_pipeline::{FeatureContext, FeatureResult};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    let mut result = FeatureResult::empty(ctx.id.clone(), ctx.feature_type.clone());

    // No faces selected → warn + no-op.
    let names = common::unique_reference_names(ctx.param("faces"));
    if names.is_empty() {
        return result;
    }

    // A non-finite distance → warn + no-op (zero IS accepted).
    match ctx.number("distance") {
        Ok(distance) if distance.is_finite() => {}
        _ => return result,
    }

    // Resolve every selected face by exact name. A miss is recorded as
    // `unresolved` (contract rule 1) so the caller's snapshot-repair pass can fix the
    // reference; a hit needs no pipeline effect (the offset sheet is a caller-side
    // SKETCH display group, never a solid record).
    for name in &names {
        if ctx.scene.resolve_face(name).is_none() {
            result.unresolved.push(name.clone());
        }
    }
    // The resolved faces are the sheet the caller displays; a miss beside
    // them is a typed partial fulfilment, never silent.
    if result.unresolved.len() < names.len() {
        result.note_partial_resolution(&names);
    }

    result
}

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "O.F",
    "shortName": "O.F",
    "longName": "Offset Face",
    "ribbonPath": "Home/Modeling/Offset Face",
    "commandSize": "Compact",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Optional identifier used for naming the offset faces"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE"
            ],
            "timestampDependency": "parentSolid",
            "multiple": true,
            "default_value": [],
            "hint": "Select one or more faces to offset"
        },
        "distance": {
            "type": "number",
            "default_value": 1,
            "hint": "Offset distance along the face normal (positive or negative)"
        }
    }
})
}

