//! CAM state uses the persisted model's serialization contracts, augmented by
//! the inspected panel/engine state producers. Post properties intentionally
//! retain arbitrary JSON values, as NcProgram permits plugin-specific values.
use serde_json::{json, Value};
use super::schema_dsl::{array, nullable, closed_object_optional};
use std::collections::BTreeMap;

#[allow(dead_code)]
#[derive(schemars::JsonSchema)]
#[serde(rename_all = "camelCase")]
struct ModelFields {
    setups: Vec<brep_cam::model::Setup>,
    tools: Vec<brep_cam::model::Tool>,
    operations: Vec<brep_cam::model::Operation>,
    nc_programs: Vec<crate::document::cam::NcProgram>,
    orientations: BTreeMap<String, crate::document::cam::SetupOrientation>,
    machines: Vec<brep_cam::model::Machine>,
    multi_axis_sources: BTreeMap<String, String>,
    id_counter: u64,
}

pub(super) fn state() -> Value {
    let generator=schemars::generate::SchemaSettings::default().for_serialize().into_generator();
    let mut result=serde_json::to_value(generator.into_root_schema_for::<ModelFields>()).unwrap();
    let text=json!({"type":"string"});let number=json!({"type":"number"});let boolean=json!({"type":"boolean"});
    let count=json!({"type":"integer","minimum":0});let strings=array(text.clone());let maybe=nullable(text.clone());
    // The panel adds the ordered operation ids to each persisted setup.
    result["$defs"]["Setup"]["properties"]["operations"]=strings.clone();
    result["$defs"]["Setup"]["required"].as_array_mut().unwrap().push(json!("operations"));
    let xyz=json!({"type":"array","items":number,"minItems":3,"maxItems":3});
    let bounds=closed_object_optional(json!({"min":xyz,"max":xyz}),&[]);
    let dictionary=|items: Value| json!({"type":"object","additionalProperties":items});
    let frame=closed_object_optional(json!({"origin":nullable(xyz.clone()),"xAxis":nullable(xyz.clone()),"zAxis":nullable(xyz),
        "frameError":maybe,"stock":nullable(bounds.clone()),"stockMachine":nullable(bounds)}),&[]);
    let mut statuses=Vec::new();
    for (word,detail) in [("idle",json!({})),("queued",json!({})),("running",json!({})),
        ("ok",json!({"motions":count,"cuttingLength":number})),("stale",json!({"motions":count})),("error",json!({"error":text}))] {
        statuses.push(closed_object_optional(json!({"status":{"const":word},"detail":closed_object_optional(detail,&[])}),&[]));
    }
    let generation=closed_object_optional(json!({"busy":boolean,"inFlight":maybe,"queue":strings,"notice":maybe,
        "progress":nullable(closed_object_optional(json!({"operation":text,"phase":text,"message":text,"current":count,"total":count}),&[])),
        "results":dictionary(closed_object_optional(json!({"ok":boolean,"error":maybe,"inputsHash":text,"workUnits":count,"motions":count,
            "cuttingLength":nullable(number.clone()),"status":maybe,"warnings":count}),&[]))}),&[]);
    let mut autoprogram=super::cam_autoprogram_output_schema::result();
    // state() never returns the command-only {created} receipt.
    autoprogram["oneOf"].as_array_mut().unwrap().pop();
    let auto_defs=autoprogram.as_object_mut().unwrap().remove("$defs").unwrap();
    // Scope these definitions to avoid collisions with persisted model DTOs.
    fn scope(value: &mut Value) {
        match value {
            Value::Object(fields) => for (key,value) in fields {
                if key=="$ref" { if let Some(s)=value.as_str() { if let Some(name)=s.strip_prefix("#/$defs/") { *value=json!(format!("#/$defs/Auto{name}")); } } }
                else { scope(value); }
            },
            Value::Array(values) => for value in values { scope(value); },
            _ => {}
        }
    }
    scope(&mut autoprogram);
    for (name,mut schema) in auto_defs.as_object().unwrap().clone() { scope(&mut schema);result["$defs"][format!("Auto{name}")]=schema; }
    let extra=json!({"strategies":strings,"problems":dictionary(text.clone()),"frames":dictionary(frame),"selected":maybe,
        "open":maybe,"notice":maybe,"error":maybe,"generation":generation,"autoprogram":autoprogram,
        "statuses":dictionary(json!({"oneOf":statuses})),"scrub":nullable(number),"builtinMachines":strings,"hiddenToolpaths":strings});
    for (key,value) in extra.as_object().unwrap() {
        result["properties"][key]=value.clone();result["required"].as_array_mut().unwrap().push(json!(key));
    }
    result["additionalProperties"]=json!(false);
    crate::automation::command::normalize_bool_subschemas(&mut result);
    result
}
