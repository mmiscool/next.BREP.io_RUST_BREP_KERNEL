//! CAM motion stream and generation-report output descriptors.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, reference};
fn variant(kind: &str, mut properties: Value, motion: bool) -> Value {
    properties["kind"]=json!({"const":kind});
    if motion {
        properties["feed"]=json!({"anyOf":[{"type":"number"},{"const":"rapid"}]});
        properties["toolAxis"]=json!({"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3});
        properties["pathId"]=json!({"type":"integer","minimum":0,"maximum":4294967295u64});
    }
    closed_object_optional(properties, if motion { &["toolAxis","pathId"] } else { &[] })
}
pub(super) fn definitions() -> Value {
    let text=json!({"type":"string"});let number=json!({"type":"number"});let boolean=json!({"type":"boolean"});
    let uint=json!({"type":"integer","minimum":0});let u32_schema=json!({"type":"integer","minimum":0,"maximum":4294967295u64});
    let xyz=json!({"type":"array","items":number,"minItems":3,"maxItems":3});let direction=json!({"enum":["cw","ccw"]});
    let mut defs=json!({});
    defs["TargetRef"]=closed_object_optional(json!({"solid":text,"face":text}), &["face"]);
    defs["WorkOffset"]=closed_object_optional(json!({"number":u32_schema,"origin":xyz}), &[]);
    defs["ProbePoint"]=closed_object_optional(json!({"id":text,"face":reference("TargetRef"),"position":xyz,"normal":xyz,"lowerTolerance":number,"upperTolerance":number}), &[]);
    defs["ProbeCycle"]=closed_object_optional(json!({"target":xyz,"approach":xyz,"nominal":reference("ProbePoint"),"stylusRadius":number,"overtravel":number,"feed":number,
        "kind":{"enum":["singleSurface","outsideCorner","insideCorner","web","pocket","bore","boss","rectangle","angle","inspection"]},
        "resultVar":text,"pointIndex":u32_schema,"pointCount":u32_schema,"wcs":{"anyOf":[reference("WorkOffset"),{"type":"null"}]}}), &[]);
    defs["RotaryWord"]=closed_object_optional(json!({"name":text,"angleDeg":number}), &[]);
    defs["TurnThreadCycle"]=closed_object_optional(json!({"start":xyz,"end":xyz,"pitch":number,"depths":array(number.clone()),"infeedOffsets":array(number.clone()),
        "clearanceX":number,"internal":boolean,"springPasses":u32_schema}), &[]);
    let mut cycles=Vec::new();
    for (kind,mut fields) in [
        ("drill",json!({"dwellSeconds":number})),("peck",json!({"peck":number})),
        ("chipBreak",json!({"peck":number,"backOff":number})),("tap",json!({"pitch":number,"rigid":boolean,"leftHand":boolean})),
        ("ream",json!({})),("bore",json!({"dwellSeconds":number,"shift":number,"feedOut":boolean})),("backBore",json!({"shift":number,"entryZ":number}))
    ] {fields["depthZ"]=number.clone();fields["retractZ"]=number.clone();cycles.push(variant(kind,fields,false));}
    defs["Cycle"]=json!({"oneOf":cycles});
    let mut motions=Vec::new();
    for kind in ["rapid","plunge","cut","retract","leadIn","leadOut","link"] {motions.push(variant(kind,json!({"to":xyz}),true));}
    for kind in ["arcCw","arcCcw"] {motions.push(variant(kind,json!({"to":xyz,"center":xyz,"plane":{"enum":["xy","xz","yz"]},"pitch":number}),true));}
    for kind in ["spindleOff","workPlaneOff"] {motions.push(variant(kind,json!({}),true));}
    for kind in ["spindleSync","tcpMode","polarInterpolation"] {motions.push(variant(kind,json!({"enabled":boolean}),true));}
    for kind in ["comment","passThrough"] {motions.push(variant(kind,json!({"text":text}),true));}
    for (kind,fields) in [
        ("probe",json!({"cycle":reference("ProbeCycle")})),("dwell",json!({"seconds":number})),("toolChange",json!({"tool":text})),
        ("cycle",json!({"at":xyz,"cycle":reference("Cycle")})),("spindleOn",json!({"rpm":number,"direction":direction})),
        ("spindleOrient",json!({"angleDeg":number})),("coolant",json!({"mode":{"enum":["off","flood","mist","through-tool","air","flood-and-through-tool","flood-and-mist"]}})),
        ("workPlane",json!({"origin":xyz,"xAxis":xyz,"zAxis":xyz})),("rotaryPosition",json!({"axes":array(reference("RotaryWord"))})),
        ("latheMode",json!({"diameterMode":boolean})),("spindleSurfaceSpeed",json!({"surfaceSpeed":number,"maxRpm":number,"direction":direction})),
        ("threadCut",json!({"to":xyz,"pitch":number})),("turnThreadCycle",json!({"cycle":reference("TurnThreadCycle")})),
        ("cylindricalInterpolation",json!({"radius":{"type":["number","null"]}})),("linearRotary",json!({"to":xyz,"axes":array(reference("RotaryWord"))})),("stop",json!({"optional":boolean}))
    ] {motions.push(variant(kind,fields,true));}
    defs["Motion"]=json!({"oneOf":motions});
    defs["Diagnostic"]=diagnostic();
    defs["EmptyRegion"]=closed_object_optional(json!({"reason":text,"target":reference("TargetRef")}), &["target"]);
    defs["Approximation"]=closed_object_optional(json!({"kind":text,"bound":number,"detail":text}), &["bound","detail"]);
    defs["OperationReport"]=closed_object_optional(json!({"status":{"enum":["pending","ok","warning","error","cancelled"]},
        "errors":array(reference("Diagnostic")),"warnings":array(reference("Diagnostic")),"emptyRegions":array(reference("EmptyRegion")),"approximations":array(reference("Approximation")),
        "counts":closed_object_optional(json!({"triangles":uint,"candidates":uint,"samples":uint,"contacts":uint,"passes":uint,"motions":uint}), &[]),"estimatedSeconds":number}),
        &["errors","warnings","emptyRegions","approximations","estimatedSeconds"]);
    defs
}
pub(super) fn toolpath() -> Value {
    let xyz=json!({"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3});
    let mut result=closed_object_optional(json!({"operation":{"type":"string"},"tool":{"type":"string"},
        "motions":{"anyOf":[array(reference("Motion")),{"type":"null"}]},"report":reference("OperationReport"),
        "cuttingLength":{"type":"number"},"segments":{"type":"integer","minimum":0},
        "cutBounds":{"anyOf":[closed_object_optional(json!({"min":xyz,"max":xyz}), &[]),{"type":"null"}]}}), &[]);
    result["$defs"]=definitions();result
}

pub(super) fn diagnostic() -> Value {
    closed_object_optional(json!({"code":{"type":"string"},"message":{"type":"string"},"detail":{"type":"string"},
        "location":{"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3}}), &["detail","location"])
}
