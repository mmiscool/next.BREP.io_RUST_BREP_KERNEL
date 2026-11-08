//! Diagnostic-store values produced by batch and interference execution.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_required};


pub(super) fn geometry_diagnostics_schema() -> Value {
    let text = json!({"type":"string"});
    let strings = json!({"type":"array","items":text});
    let payload = json!({"type":["object","null"]});
    let details = closed_object_required(json!({
        "errors":strings,"refusal":payload,"unresolved":{"anyOf":[strings,{"type":"null"}]},
        "fulfilment":payload,"approximations":{"type":["array","null"],"items":{"type":"object"}},
        "operation":text,"operands":{},"dependencies":{"type":"array","items":{"type":"integer","minimum":0}}
    }), &["errors","refusal","unresolved","fulfilment","approximations","operation","operands","dependencies"]);
    let interference = closed_object_required(json!({"operation":text,"category":text,"operands":strings,"detail":text}),
        &["operation","category","operands","detail"]);
    let report = closed_object_required(json!({
        "featureErrors":strings,"featureNotes":strings,"unresolved":strings,"displayErrors":strings,
        "featureFulfilment":{"type":"object","additionalProperties":{"type":"object"}},
        "featureTimings":{"type":"object","additionalProperties":{"type":"number"}},
        "featureOutputs":{"type":"object","additionalProperties":strings},
        "featureRefusals":{"type":"object","additionalProperties":{"type":"object"}},
        "featureApproximations":{"type":"object","additionalProperties":{"type":"array","items":{"type":"object"}}},
        "featureAutoTargets":{"type":"object","additionalProperties":text}
    }), &["featureErrors","featureNotes","featureFulfilment","unresolved","displayErrors","featureTimings","featureOutputs"]);
    let error = closed_object_required(json!({"error":text}), &["error"]);
    json!({"type":"object","oneOf":[details,interference,report,error]})
}
