//! Shared wire descriptors for plugin discovery and the workbench readback.
//! Keep authored schemas/params open; their internal languages are extensible.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_all, array};

fn pin_schema() -> Value {
    let text=json!({"type":"string"});
    closed_object_all(json!({"id":text,"version":text,"apiVersion":{"type":"integer","minimum":0},"digest":text}))
}
pub(super) fn action_schema() -> Value {
    let text=json!({"type":"string"});
    closed_object_all(json!({"id":text,"label":text,"glyph":text,"ribbonPath":text,
        "commandSize":text,"inputParamsSchema":{},"package":pin_schema()}))
}
pub(super) fn panels_schema() -> Value {
    let text=json!({"type":"string"});
    let texts=array(text.clone());
    let text_control=closed_object_all(json!({"type":{"const":"text"},"text":text}));
    let table_control=closed_object_all(json!({"type":{"const":"table"},"columns":texts,"rows":array(texts.clone())}));
    let action_control=closed_object_all(json!({"type":{"const":"action"},"id":text,"label":text,"action":text,"params":{}}));
    let form_control=closed_object_all(json!({"type":{"const":"form"},"id":text,"label":text,"action":text,"params":{}}));
    let controls=array(json!({"oneOf":[text_control,table_control,action_control,form_control]}));
    array(closed_object_all(json!({"id":text,"label":text,"controls":controls,"package":pin_schema()})))
}
pub(super) fn annotations_schema() -> Value {
    let text=json!({"type":"string"});
    array(closed_object_all(json!({"type":text,"id":text,"label":text,"longName":text,
        "shortName":{"const":"ANN"},"inputParamsSchema":{},"package":pin_schema()})))
}

pub(super) fn bundle_schema() -> Value {
    let text=json!({"type":"string"});
    closed_object_all(json!({
        "manifest":closed_object_all(json!({"id":text,"version":text,"apiVersion":{"type":"integer","minimum":0,"maximum":4294967295u64},"name":text,"entry":text})),
        "modules":{"type":"object","additionalProperties":text},
        "assets":{"type":"object","additionalProperties":{"type":"array","items":{"type":"integer","minimum":0,"maximum":255}}}
    }))
}
pub(super) fn feature_schema() -> Value {
    // Kernel definitions and plugin providers share these fields. Feature-owned
    // presentation metadata and the parameter language remain extensible.
    let text=json!({"type":"string"});
    json!({"type":"object","properties":{
        "type":text,"shortName":text,"longName":text,"id":text,"label":text,
        "ribbonPath":text,"commandSize":text,"icon":text,"displayBuilder":{"type":"boolean"},
        "inputParamsSchema":{"type":"object","additionalProperties":true},"package":pin_schema()
    },"required":["type","shortName","longName","inputParamsSchema"],"additionalProperties":true})
}
pub(super) fn feature_catalogue_schema() -> Value {
    closed_object_all(json!({"version":{"type":"integer","minimum":0},"features":array(feature_schema()),
        "assemblyConstraints":array(json!({"type":"object","additionalProperties":true}))}))
}
pub(super) fn describe_schema(action_status: Value) -> Value {
    let text=json!({"type":"string"});
    let texts=array(text.clone());
    let mut installed=pin_schema();
    installed["properties"]["name"]=text.clone();
    installed["properties"]["enabled"]=json!({"type":"boolean"});
    installed["required"].as_array_mut().unwrap().extend([json!("name"),json!("enabled")]);
    let workbench=closed_object_all(json!({"id":text,"label":text,"featureTypes":texts,"actions":texts,"panels":texts,"package":pin_schema()}));
    let catalogue=closed_object_all(json!({
        "revision":{"type":"integer","minimum":0},"documentRevision":{"type":"integer","minimum":0},
        "features":array(feature_schema()),"actions":array(action_schema()),
        "workbenches":array(workbench),"panels":panels_schema(),"annotations":annotations_schema(),"errors":texts
    }));
    closed_object_all(json!({"installed":array(installed),"catalogue":catalogue,"action":action_status}))
}
