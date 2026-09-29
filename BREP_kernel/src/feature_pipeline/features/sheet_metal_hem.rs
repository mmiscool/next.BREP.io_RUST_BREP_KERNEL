//! SM.HEM uses the flange core with a fixed 180° angle and tangent-to-bend
//! length, since mold-line setbacks diverge at that angle. A zero bend radius
//! becomes 0.0001, leaving a 0.0002 gap under the return leg. Other flange
//! parameters retain their shared behavior.

use super::sheet_metal_flange::{self, FlangeOptions};
use crate::feature_pipeline::{FeatureContext, FeatureResult};

pub fn execute(ctx: &FeatureContext) -> FeatureResult {
    sheet_metal_flange::run(
        ctx,
        FlangeOptions {
            angle_deg: Some(180.0),
            default_leg: 5.0,
            length_reference: Some("Tangent to Bend"),
            knife_zero_radius: true,
        },
    )
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces/edges drive `faces`.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.edges > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SM.HEM",
    "shortName": "SM.HEM",
    "longName": "SM Hem",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the hem feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "EDGE"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Select one or more sheet-metal edge overlays where the hem will be constructed."
        },
        "useOppositeCenterline": {
            "label": "Reverse direction",
            "type": "boolean",
            "default_value": false,
            "hint": "Flip the fold direction (up/down) for the selected hinge."
        },
        "flangeLength": {
            "type": "number",
            "default_value": 5,
            "min": 0,
            "hint": "Hem leg length extending away from the bend edge."
        },
        "edgeStartSetback": {
            "label": "Start setback",
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Distance from the selected edge start point where the hem span begins."
        },
        "edgeEndSetback": {
            "label": "End setback",
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Distance from the selected edge end point where the hem span ends."
        },
        "inset": {
            "label": "Flange position",
            "type": "options",
            "options": [
                "Material Inside",
                "Material Outside",
                "Bend Outside"
            ],
            "default_value": "Material Inside",
            "hint": "Where the hem sits relative to the reference edge (inward fold-line shift before the bend): Material Inside = thickness + bendRadius; Material Outside = bendRadius; Bend Outside = 0."
        },
        "bendRadius": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Inside hem radius. 0 (default) = knife hem — a true 0 cannot revolve, so it is interpreted as 0.0001."
        },
        "offset": {
            "type": "number",
            "default_value": 0,
            "hint": "Additional signed offset for bend-edge repositioning (positive = outward, negative = inward)."
        }
    }
})
}


