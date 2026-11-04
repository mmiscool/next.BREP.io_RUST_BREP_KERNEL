//! Tool/feeds DTO schemas shared by CAM inspection and library receipts.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, reference};

pub(super) fn definitions() -> Value {
    let text=json!({"type":"string"});let number=json!({"type":"number"});let boolean=json!({"type":"boolean"});
    let uint=json!({"type":"integer","minimum":0,"maximum":4294967295u64});
    let mut defs=json!({});
    defs["Feeds"]=closed_object_optional(json!({"spindleRpm":number,"spindleDirection":{"enum":["cw","ccw"]},"cuttingFeed":number,"plungeFeed":number,
        "leadFeed":number,"rampFeed":number,"retractFeed":number,"surfaceSpeed":number,"feedPerTooth":number,"surfaceSpeedLimitRpm":number,"css":boolean}),
        &["leadFeed","rampFeed","retractFeed","surfaceSpeed","feedPerTooth","surfaceSpeedLimitRpm","css"]);
    defs["ToolGeometry"]=closed_object_optional(json!({"diameter":number,"cuttingLength":number,"shaftLength":number,"shaftDiameter":number,"overallLength":number,
        "cornerRadius":number,"includedAngleDeg":number,"ballDiameter":number,"tipDiameter":number,"threadPitch":number,"fluteCount":uint,"leftHand":boolean}),
        &["shaftDiameter","overallLength","cornerRadius","includedAngleDeg","ballDiameter","tipDiameter","threadPitch","fluteCount","leftHand"]);
    defs["Holder"]=closed_object_optional(json!({"id":text,"name":text,"segments":array(closed_object_optional(json!({"height":number,"lowerDiameter":number,"upperDiameter":number}), &[]))}), &[]);
    let insert=json!({"shape":{"enum":["C","D","V","T","W","R","S"]},"edge_length":number,"nose_radius":number,
        "clearance_deg":number,"lead_angle_deg":number,"holder_angle_deg":number});
    defs["Insert"]=closed_object_optional(insert.clone(), &[]);
    let mut variants=Vec::new();
    for (kind,mut fields) in [
        ("insert",insert),
        ("blade",json!({"width":number,"reach":number,"corner_radius":number,"holder_angle_deg":number})),
        ("thread",json!({"pitch_min":number,"pitch_max":number,"angle_deg":number,"reach":number,"nose_radius":number,"holder_angle_deg":number})),
        ("boringBar",json!({"diameter":number,"reach":number,"insert":reference("Insert"),"insert_offset":{"type":"array","items":number,"minItems":3,"maxItems":3}}))
    ] { fields["kind"]=json!({"const":kind}); variants.push(closed_object_optional(fields,&[])); }
    defs["TurningGeometry"]=json!({"oneOf":variants});
    defs["Tool"]=closed_object_optional(json!({"id":text,"number":uint,"name":text,"description":text,
        "kind":{"enum":["flat","ball","bull","chamfer","v-bit","tapered-ball","tapered-flat","lollipop","face-mill","slot","t-slot","dovetail","engraver","thread-mill","drill","spot-drill","center-drill","tap","reamer","boring-bar","lathe-insert"]},
        "geometry":reference("ToolGeometry"),"turningGeometry":reference("TurningGeometry"),"holder":reference("Holder"),
        "coolant":{"enum":["off","flood","mist","through-tool","air","flood-and-through-tool","flood-and-mist"]},"feeds":reference("Feeds"),
        "lengthOffset":uint,"diameterOffset":uint,"material":text}), &["turningGeometry","holder","lengthOffset","diameterOffset","material"]);
    defs
}

pub(super) fn calculate_feeds() -> Value {
    let mut result=closed_object_optional(json!({"feeds":{"anyOf":[reference("Feeds"),{"type":"null"}]},"adjustments":array(json!({"type":"string"}))}), &[]);
    result["$defs"]=definitions();result
}
pub(super) fn library() -> Value {
    let text=json!({"type":"string"});let strings=array(text.clone());
    json!({"type":"object","oneOf":[
        closed_object_optional(json!({"name":text,"tools":array(reference("Tool"))}), &[]),
        closed_object_optional(json!({}), &[]),closed_object_optional(json!({"id":text}), &[]),
        closed_object_optional(json!({"ids":strings,"skipped":strings}), &[]),closed_object_optional(json!({"text":text}), &[])
    ],"$defs":definitions()})
}
