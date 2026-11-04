//! Output envelopes for the drawing and annotation parameter catalogues.
use serde_json::{json, Value};

fn catalogue_schema(key: &str, annotation_labels: bool) -> Value {
    let mut properties = json!({
        "type": {"type":"string"},
        "shortName": {"type":"string"},
        "longName": {"type":"string"},
        "inputParamsSchema": {
            "type":"object", "additionalProperties":true,
            "description":"Parameter-name dictionary of extensible application form descriptors, not a JSON Schema document."
        }
    });
    let mut required = vec!["type", "shortName", "longName", "inputParamsSchema"];
    if annotation_labels {
        properties["label"] = json!({"type":"string"});
        properties["icon"] = json!({"type":"string"});
        required.extend(["label", "icon"]);
    }
    let entry = json!({"type":"object", "properties":properties,
        "required":required, "additionalProperties":false});
    let mut envelope = serde_json::Map::new();
    envelope.insert(key.to_owned(), json!({"type":"array", "items":entry}));
    json!({"type":"object", "properties":envelope,
        "required":[key], "additionalProperties":false})
}

pub(super) fn pmi_catalogue_schema() -> Value { catalogue_schema("annotations", true) }
pub(super) fn sheet_catalogue_schema() -> Value { catalogue_schema("objects", false) }
