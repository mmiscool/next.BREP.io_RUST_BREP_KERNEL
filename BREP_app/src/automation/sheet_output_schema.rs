//! Saved sheet and projected drawing contracts, following serde output DTOs.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_optional, array, reference, vector};
fn toleranced(mut fields: Value) -> Value {
    fields.as_object_mut().unwrap().extend(json!({"tolMode":{"type":"string"},"tolUpper":{"type":"number"},"tolLower":{"type":"number"},"isReference":{"type":"boolean"}}).as_object().unwrap().clone());
    closed_object_optional(fields, &["tolMode","tolUpper","tolLower","isReference"])
}
pub(super) fn sheet_state_schema() -> Value {
    let text=json!({"type":"string"}); let number=json!({"type":"number"}); let boolean=json!({"type":"boolean"});
    let strings=array(text.clone()); let v2=vector(2); let lines=array(array(v2.clone()));
    let uint=json!({"type":"integer","minimum":0}); let precision=json!({"type":"integer","minimum":0,"maximum":4294967295u64});
    let mut defs=json!({});
    defs["SectionCut"]=closed_object_optional(json!({"source":text,"from":text,"to":text,"label":text,"flip":boolean}), &[]);
    defs["DetailCircle"]=closed_object_optional(json!({"source":text,"centre":text,"rim":text,"label":text}), &[]);
    defs["PlacedView"]=closed_object_optional(json!({"id":text,"view":text,"position":v2,"scale":number,"projection":text,"flattenText":boolean,"threeD":boolean,
        "section":reference("SectionCut"),"detail":reference("DetailCircle")}), &["threeD","section","detail"]);
    defs["Dimension"]=toleranced(json!({"id":text,"kind":text,"alignment":text,"anchors":strings,"offsetMm":number,"precision":precision}));
    defs["Ordinate"]=toleranced(json!({"id":text,"axis":text,"datum":text,"members":strings,"offsetMm":number,"precision":precision}));
    defs["Revision"]=closed_object_optional(json!({"rev":text,"date":text,"description":text}), &[]);
    defs["BomTable"]=closed_object_optional(json!({"position":v2,"columnWidthMm":number,"columns":strings}), &[]);
    defs["Sheet"]=closed_object_optional(json!({"id":text,"name":text,"size":text,"widthMm":number,"heightMm":number,"border":boolean,"borderInsetMm":number,
        "titleBlock":boolean,"notes":text,"views":array(reference("PlacedView")),"dimensions":array(reference("Dimension")),
        "ordinates":array(reference("Ordinate")),"revisions":array(reference("Revision")),"bomTable":reference("BomTable")}), &["revisions","bomTable"]);
    defs["Text"]=closed_object_optional(json!({"text":text,"anchor":v2,"height":number,"angle":number,"right":v2,"up":v2}), &[]);
    let texts=array(reference("Text"));
    defs["DetailDrawing"]=closed_object_optional(json!({"label":text,"centre":v2,"radiusMm":number,"circle":array(v2.clone()),"texts":texts}), &[]);
    defs["DetailMark"]=closed_object_optional(json!({"id":text,"label":text,"centre":v2,"radiusMm":number,"lines":lines,"texts":texts}), &[]);
    defs["SectionDrawing"]=closed_object_optional(json!({"label":text,"hatch":lines,"cut":lines,"texts":texts}), &[]);
    defs["SectionMark"]=closed_object_optional(json!({"id":text,"label":text,"lines":lines,"texts":texts}), &[]);
    defs["AnnotationDrawing"]=closed_object_optional(json!({"id":text,"type":text,"text":text,"lines":lines,"texts":texts,"note":text}), &["note"]);
    defs["Anchor"]=closed_object_optional(json!({"ref":text,"kind":text,"at":v2,"model":vector(3),"dir":v2,"radius":number,"u":v2,"v":v2}), &["dir","radius","u","v"]);
    defs["ViewDrawing"]=closed_object_optional(json!({"id":text,"view":text,"name":text,"position":v2,"scale":number,"error":text,"solids":strings,
        "edges":lines,"highlighted":lines,"annotations":array(reference("AnnotationDrawing")),"anchors":array(reference("Anchor")),"textMm":number,
        "section":reference("SectionDrawing"),"sectionMarks":array(reference("SectionMark")),"detail":reference("DetailDrawing"),
        "detailMarks":array(reference("DetailMark")),"bounds":vector(4),"hiddenLine":text}),
        &["error","solids","highlighted","section","sectionMarks","detail","detailMarks","hiddenLine"]);
    // serde_json writes non-finite f64 values as null: unresolved measurements retain their rows.
    let measured=json!({"type":["number","null"]});
    defs["DimensionDrawing"]=closed_object_optional(json!({"id":text,"type":text,"text":text,"value":measured,"lines":lines,"texts":texts,"label":v2,
        "offsetMm":number,"toleranceFrom":text,"error":text}), &["toleranceFrom","error"]);
    defs["OrdinateMember"]=closed_object_optional(json!({"ref":text,"datum":boolean,"value":measured,"text":text,"at":{"type":"array","items":measured,"minItems":2,"maxItems":2},"error":text}), &["error"]);
    defs["OrdinateDrawing"]=closed_object_optional(json!({"id":text,"axis":text,"lines":lines,"texts":texts,"label":v2,"offsetMm":number,
        "stations":array(reference("OrdinateMember")),"error":text}), &["error"]);
    defs["RevisionTableDrawing"]=closed_object_optional(json!({"lines":lines,"texts":texts,"rows":uint,"bounds":vector(4),"placement":text}), &[]);
    defs["FrameDrawing"]=closed_object_optional(json!({"lines":lines,"texts":texts,"revisionTable":reference("RevisionTableDrawing")}), &["revisionTable"]);
    defs["Drawing"]=closed_object_optional(json!({"id":text,"name":text,"widthMm":number,"heightMm":number,"views":array(reference("ViewDrawing")),
        "dimensions":array(reference("DimensionDrawing")),"ordinates":array(reference("OrdinateDrawing")),"frame":reference("FrameDrawing")}), &["frame"]);
    let nullable_text=json!({"type":["string","null"]});
    let mut result=closed_object_optional(json!({"sheets":array(reference("Sheet")),"idCounter":uint,"openSheet":nullable_text,"openObject":nullable_text,
        "selectedObject":nullable_text,"drawing":{"anyOf":[reference("Drawing"),{"type":"null"}]}}), &[]);
    result["$defs"]=defs; result
}
