//! Shared CAM setup geometry descriptors.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_all};
pub(super) fn definitions() -> Value {
    let number=json!({"type":"number"});let text=json!({"type":"string"});
    let xyz=json!({"type":"array","items":number,"minItems":3,"maxItems":3});
    let mut stocks=Vec::new();
    for (kind,mut fields) in [
        ("boundsOffset",json!({"x":number,"y":number,"top":number,"bottom":number})),
        ("box",json!({"min":xyz,"max":xyz})),("cylinder",json!({"base":xyz,"axis":xyz,"radius":number,"height":number})),
        ("fromSolid",json!({"solid":text})),("fromRest",json!({"setup":text}))
    ] {fields["kind"]=json!({"const":kind});stocks.push(closed_object_all(fields));}
    json!({"Stock":{"oneOf":stocks},"RigidTransform":closed_object_all(json!({"rotation":{"type":"array","items":xyz,"minItems":3,"maxItems":3},"translation":xyz}))})
}
