//! Shared native CAM player receipts and state descriptors.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, reference, nullable};

fn schema(name: &str) -> Value {
    let text=json!({"type":"string"}); let number=json!({"type":"number"});
    let uint=json!({"type":"integer","minimum":0}); let boolean=json!({"type":"boolean"});
    let xyz=json!({"type":"array","items":number,"minItems":3,"maxItems":3});
    let mut defs=json!({});
    defs["Legend"]=closed_object_optional(json!({"min":number,"max":number,"tolerance":number,"scale":number,
        "stops":array(closed_object_optional(json!({"value":number,"color":xyz}), &[])),
        "axial":closed_object_optional(json!({"withinTolerance":boolean,"tolerance":number,"machinedFromZ":number,
            "machinedExcessArea":number,"machinedMissingArea":number,"rearRemainingVolume":number,"totalRemainingVolume":number,"scope":text}), &[])}), &["axial"]);
    defs["Remaining"]=closed_object_optional(json!({"tolerance":number,"bands":array(closed_object_optional(json!({"band":text,"from":number,"to":number,"color":xyz}), &["from","to"])),
        "counts":closed_object_optional(json!({"within":uint,"excess":uint,"gouge":uint,"excessHigh":uint,"belowPart":uint}), &[]),
        "maxExcess":number,"minValue":number,"vertices":uint,"stockVersion":nullable(uint.clone())}), &[]);
    defs["Runner"]=closed_object_optional(json!({"session":uint,"loaded":boolean,"inFlight":nullable(uint.clone()),"submitted":uint,"accepted":uint,"cancelled":uint,"failed":nullable(text.clone())}), &[]);
    defs["Session"]=closed_object_optional(json!({"setup":text,"multiAxis":boolean,"playing":boolean,"speed":number,"motion":uint,"fraction":number,"time":number,
        "totalSeconds":number,"progress":number,"motions":uint,"completed":uint,"applied":uint,"caughtUp":boolean,"runner":nullable(reference("Runner")),
        "removedVolume":number,"stockVolume":number,"unsupported":array(uint.clone()),"warnings":uint,"operation":nullable(text.clone()),
        "operations":array(closed_object_optional(json!({"operation":text,"first":uint,"count":uint,"seconds":number}), &[])),
        "marks":uint,"collisionMotions":array(uint.clone()),"collisions":array(closed_object_optional(json!({"motion":uint,"part":text,"against":text}), &[])),
        "deviation":nullable(reference("Legend")),"stockVersion":uint,"remaining":nullable(reference("Remaining"))}), &[]);
    let state=closed_object_optional(json!({"loaded":boolean,"speed":number,"stopAtCollision":boolean,"cell":number,"showDeviation":boolean,"showRemaining":boolean,
        "notice":nullable(text.clone()),"error":nullable(text.clone()),"session":nullable(reference("Session")),"overlayStockBuilds":uint}), &[]);
    let mut result=match name {
        "state"=>state,
        "cancel"=>closed_object_optional(json!({"stopped":boolean,"state":state}), &[]),
        "play"=>closed_object_optional(json!({"playing":boolean}), &[]),
        "step"=>closed_object_optional(json!({"motion":uint,"fraction":number,"completed":uint}), &[]),
        "scrub"=>closed_object_optional(json!({"motion":uint,"fraction":number,"time":number,"completed":uint,
            "pose":nullable(closed_object_optional(json!({"tip":xyz,"axis":xyz}), &[]))}), &[]),
        "export"=>closed_object_optional(json!({"stl":{"type":"string","description":"Complete ASCII STL of the simulated stock in model coordinates."},"triangles":uint}), &[]),
        "deviation"=>closed_object_optional(json!({"legend":nullable(reference("Legend")),
            "faces":array(closed_object_optional(json!({"solid":text,"face":text,"samples":uint,"min":number,"max":number,"mean":number,"gouged":uint,"excess":uint}), &[])),
            "columns":uint,"belowPart":uint,"belowPartVolume":number,
            "samples":array(closed_object_optional(json!({"view":text,"p":xyz,"pModel":xyz,"excess":number,"solid":text,"face":text,"nearest":xyz}), &[])),
            "samplesTotal":uint}), &["samples","samplesTotal"]),
        _=>unreachable!("fixed CAM descriptor name"),
    };
    // Only branches using references need the shared definitions.
    if matches!(name,"state"|"cancel"|"deviation") { result["$defs"]=defs; }
    result
}
pub(super) fn state() -> Value { schema("state") }
pub(super) fn cancel() -> Value { schema("cancel") }
pub(super) fn play() -> Value { schema("play") }
pub(super) fn step() -> Value { schema("step") }
pub(super) fn scrub() -> Value { schema("scrub") }
pub(super) fn export() -> Value { schema("export") }
pub(super) fn deviation() -> Value { schema("deviation") }
