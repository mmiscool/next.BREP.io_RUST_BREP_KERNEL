//! Autoprogram preview and creation receipts from the application panel.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, reference};
pub(super) fn result() -> Value {
    let text=json!({"type":"string"});let boolean=json!({"type":"boolean"});
    let uint=json!({"type":"integer","minimum":0});let strings=array(text.clone());let nullable_text=json!({"type":["string","null"]});
    let mut defs=super::cam_setup_output_schema::definitions();
    defs["Diagnostic"]=super::cam_path_output_schema::diagnostic();
    defs["PlannedOperation"]=closed_object_optional(json!({"id":text,"name":text,"strategy":text,"role":text,"tool":text,"toolName":nullable_text,
        "toolSource":{"enum":["builtin","document"]},"notes":strings}), &[]);
    defs["Feature"]=closed_object_optional(json!({"id":text,"kind":text,"solid":text,"selected":boolean,
        "operations":array(reference("PlannedOperation")),"refusals":strings}), &[]);
    defs["Programmed"]=closed_object_optional(json!({"id":text,"setup":text,"feature":text,"kind":text,
        "status":{"enum":["idle","queued","running","ok","stale","error"]},"minimumCutZ":{"type":["number","null"]},
        "warnings":{"anyOf":[array(reference("Diagnostic")),{"type":"null"}]},"reRecognised":boolean}), &[]);
    let base=json!({"id":text,"name":text,"axis":nullable_text,"selected":boolean,"additional":boolean,
        "orientation":reference("RigidTransform"),"stock":reference("Stock"),"features":array(reference("Feature")),"operationCount":uint});
    let mut first=base.clone();first["additional"]=json!({"const":false});first["axis"]=json!({"const":"+Z"});
    let mut extra=base;extra["additional"]=json!({"const":true});extra["covered"]=strings.clone();
    defs["PreviewSetup"]=json!({"oneOf":[closed_object_optional(first,&[]),closed_object_optional(extra,&[])]});
    let common=json!({"open":boolean,"busy":boolean,"error":nullable_text,"allProgrammedOk":boolean,"programmed":array(reference("Programmed"))});
    let mut pending=common.clone();pending["features"]=json!({"type":"array","maxItems":0});pending["operations"]=json!({"type":"array","maxItems":0});
    let mut ready=common;ready["busy"]=json!({"const":false});
    ready.as_object_mut().unwrap().extend(json!({"stale":boolean,"setup":text,"setups":array(reference("PreviewSetup")),"setupCount":uint,
        "unreached":array(closed_object_optional(json!({"feature":text,"kind":text,"reason":text}), &[])),"features":array(reference("Feature")),"featureCount":uint,
        "selectedCount":uint,"operationCount":uint,"unassignedFaces":{"type":"array","items":{"type":"array","items":text,"minItems":2,"maxItems":2}},
        "material":{"const":"aluminium"},"notice":text}).as_object().unwrap().clone());
    json!({"type":"object","oneOf":[closed_object_optional(pending,&[]),closed_object_optional(ready,&[]),closed_object_optional(json!({"created":strings}), &[])],"$defs":defs})
}
