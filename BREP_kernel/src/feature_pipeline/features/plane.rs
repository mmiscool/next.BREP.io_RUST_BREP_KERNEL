//! P — Plane. Ported from the retired `PlaneFeature`.
//!
//! Produces no solid; it registers ONE named scene FRAME under `${id}` (the
//! selectable mesh name is the featureID) so a later sketch
//! can resolve its plane by name, fully headless. The frame is:
//!   - a `datum`/`face` reference's (origin, normal); or
//!   - the `orientation` (XY/XZ/YZ) base normal;
//! offset along the normal by `offset_distance`, then run through the worldUp
//! convention ([`Frame::from_origin_normal`]) — the same basis a sketch would
//! derive when it references this plane.

use crate::feature_pipeline::features::common;
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
    let offset = match ctx.param("offset_distance") {
        Some(Value::Null) | None => 0.0,
        Some(_) => ctx.number("offset_distance")?,
    };

    // A `datum` reference (a datum/construction PLANE or a FACE) takes priority
    // over `orientation`, matching `PlaneFeature.createPlaneMesh` (`datum ? … : …`).
    // The shared plane resolver treats a datum plane and a planar face identically.
    let (mut origin, normal) = match common::first_reference_name(ctx.param("datum")) {
        Some(name) => match common::resolve_plane_reference(ctx, &name) {
            Some(plane_ref) => {
                let frame = plane_ref.frame()?;
                (frame.origin, frame.z_axis)
            }
            None => {
                // Unresolved reference — register no frame; the caller repairs later.
                let mut result =
                    FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
                result.unresolved.push(name);
                return Ok(result);
            }
        },
        None => (Vec3::new(0.0, 0.0, 0.0), orientation_normal(ctx.param("orientation"))),
    };

    let unit = normal.normalized()?;
    origin = origin.add(unit.scale(offset));
    let frame = Frame::from_origin_normal(origin, normal)?;

    let mut result = FeatureResult::pass_through(ctx.id.clone(), ctx.feature_type.clone());
    result.frames.push((ctx.id.clone(), frame));
    Ok(result)
}

/// The base plane normal for an `orientation` (matching `#basisFromOrientation`:
/// XY→+Z, XZ (`rotX π/2`)→−Y, YZ (`rotY π/2`)→+X).
fn orientation_normal(param: Option<&Value>) -> Vec3 {
    match param
        .and_then(|v| v.as_str())
        .map(|text| text.trim().to_uppercase())
        .as_deref()
    {
        Some("XZ") => Vec3::new(0.0, -1.0, 0.0),
        Some("YZ") => Vec3::new(1.0, 0.0, 0.0),
        _ => Vec3::new(0.0, 0.0, 1.0),
    }
}


/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// a selected FACE or datum/construction PLANE can seat the plane (`datum`) — the
/// build path resolves both (an offset plane FROM a datum plane), so the offer
/// must appear for a plane selection too, not just a face.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.faces > 0 || probe.planes > 0
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "P",
    "shortName": "P",
    "longName": "Plane",
    "displayBuilder": true,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "unique identifier for the plane feature"
        },
        "datum": {
            "type": "reference_selection",
            "selectionFilter": [
                "PLANE",
                "FACE"
            ],
            "multiple": false,
            "default_value": null,
            "hint": "Optional reference plane or face"
        },
        "orientation": {
            "type": "options",
            "options": [
                "XY",
                "XZ",
                "YZ"
            ],
            "default_value": "XY",
            "hint": "Plane orientation"
        },
        "offset_distance": {
            "type": "number",
            "default_value": 0,
            "hint": "Plane offset distance"
        }
    }
})
}

