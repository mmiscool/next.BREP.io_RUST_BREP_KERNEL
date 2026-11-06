//! Serialized PMI state and report contracts from the kernel's PMI DTOs.
use serde_json::{json, Value};
use super::schema_dsl::{closed_object_required, array, reference, vector};

fn variant(kind: &str, mut fields: Value, required: &[&str]) -> Value {
    fields["kind"] = json!({"const":kind});
    let mut keys = required.to_vec(); keys.push("kind");
    closed_object_required(fields, &keys)
}

pub(super) fn pmi_state_schema() -> Value {
    let text = json!({"type":"string"});
    let number = json!({"type":"number"});
    let boolean = json!({"type":"boolean"});
    let strings = array(text.clone());
    let v3 = vector(3);
    let pair = json!({"type":"array","items":v3,"minItems":2,"maxItems":2});
    let mut defs = json!({});
    defs["Camera"] = closed_object_required(json!({"eye":v3,"target":v3,"up":v3,"viewport":vector(2),
        "projection":{"oneOf":[variant("orthographic",json!({"halfHeight":number}), &["halfHeight"]),
            variant("perspective",json!({"fovYDeg":number}), &["fovYDeg"])]}}), &["eye","target","up","projection","viewport"]);
    defs["SectionPlane"] = closed_object_required(json!({"axis":text,"offset":number,"flip":boolean,"normal":v3}), &["axis","offset","flip"]);
    defs["Section"] = closed_object_required(json!({"axis":text,"offset":number,"flip":boolean,"origin":v3,"normal":v3,
        "plane":text,"exclude":strings,"planes":array(reference("SectionPlane"))}), &["axis","offset","flip"]);
    defs["Display"] = closed_object_required(json!({"textSizePt":number,"wireframe":boolean,"section":reference("Section")}), &["textSizePt","wireframe"]);
    defs["PluginReplay"] = closed_object_required(json!({"version":{"type":"integer","minimum":0,"maximum":4294967295u64},
        "fingerprint":{},"dependencies":{"type":"object","additionalProperties":true},"inputData":{},"outputData":{}}),
        &["version","fingerprint","dependencies","inputData","outputData"]);
    defs["Annotation"] = closed_object_required(json!({"type":text,"enabled":boolean,"inputParams":{},"persistentData":{},
        "pluginReplay":reference("PluginReplay"),"labelWorld":v3}), &["type","enabled","inputParams"]);
    defs["View"] = closed_object_required(json!({"id":text,"name":text,"camera":reference("Camera"),"display":reference("Display"),
        "annotations":array(reference("Annotation"))}), &["id","name","display","annotations"]);
    defs["Pose"] = closed_object_required(json!({"center":v3,"translate":v3,"rotateDeg":v3,"scale":v3}), &["center","translate","rotateDeg","scale"]);
    defs["HalfSpace"] = closed_object_required(json!({"point":v3,"normal":v3,"also":array(pair.clone())}), &["point","normal"]);
    let anchor = json!({"camera":reference("Camera"),"solids":strings,
        "poses":array(closed_object_required(json!({"solid":text,"stages":array(reference("Pose"))}), &["solid","stages"])),"section":reference("HalfSpace")});
    let anchors: Vec<_> = ["surface","edge","point"].iter().map(|mode| {
        let mut fields = anchor.clone(); fields["mode"] = json!({"const":mode});
        let mut required = vec!["solids","mode"];
        if *mode == "edge" {
            fields["solid"] = text.clone(); fields["edgeId"] = json!({"type":"integer","minimum":0});
            required.extend(["solid","edgeId"]);
        }
        closed_object_required(fields, &required)
    }).collect();
    defs["BalloonAnchor"] = json!({"oneOf":anchors});
    defs["FcfFrame"] = closed_object_required(json!({"characteristic":text,"symbol":text,"zone":text,"datums":strings}), &["characteristic","symbol","zone","datums"]);
    defs["Plane"] = closed_object_required(json!({"origin":v3,"normal":v3,"xAxis":v3}), &["origin","normal","xAxis"]);
    defs["Geometry"] = json!({"oneOf":[
        variant("none",json!({}), &[]),
        variant("linear",json!({"a":v3,"b":v3,"component":{"type":"string","minLength":1,"maxLength":1}}), &["a","b"]),
        variant("radial",json!({"center":v3,"axis":v3,"radius":number,"diameter":boolean,"sphere":boolean}), &["center","axis","radius","diameter","sphere"]),
        variant("angular",json!({"vertex":v3,"dirA":v3,"dirB":v3,"axis":v3,"degrees":number}), &["vertex","dirA","dirB","axis","degrees"]),
        variant("leader",json!({"targets":array(v3.clone()),"dot":boolean,"balloon":boolean,"anchor":reference("BalloonAnchor")}), &["targets","dot"]),
        variant("note",json!({"position":v3}), &["position"]),
        variant("hole",json!({"anchor":v3,"normal":v3}), &["anchor","normal"]),
        variant("hide",json!({"solids":strings}), &["solids"]),
        variant("explode",json!({"solids":strings,"translate":v3,"rotateDeg":v3,"scale":v3,"center":v3,"trace":boolean}), &["solids","translate","rotateDeg","scale","center","trace"]),
        variant("datum",json!({"anchor":v3,"normal":v3,"letter":text}), &["anchor","normal","letter"]),
        variant("fcf",json!({"anchor":v3,"normal":v3,"frame":reference("FcfFrame")}), &["anchor","normal","frame"])
    ]});
    defs["AnnotationReport"] = closed_object_required(json!({"persistentData":{},"pluginReplay":reference("PluginReplay"),"id":text,"type":text,
        "enabled":boolean,"status":{"enum":["ok","error"]},"message":text,"text":text,"value":number,"unit":text,
        "references":strings,"labelWorld":v3,"plane":reference("Plane"),"geometry":reference("Geometry")}),
        &["id","type","enabled","status","message","text","unit","references","labelWorld","geometry"]);
    defs["ViewReport"] = closed_object_required(json!({"id":text,"annotations":array(reference("AnnotationReport")),
        "sectionPlane":pair,"sectionError":text,"hidden":strings}), &["id","annotations"]);
    defs["Report"] = closed_object_required(json!({"views":array(reference("ViewReport"))}), &["views"]);
    let nullable_text = json!({"type":["string","null"]});
    let mut result = closed_object_required(json!({"views":array(reference("View")),"idCounter":{"type":"integer","minimum":0},
        "activeView":nullable_text,"openAnnotation":nullable_text,"openView":nullable_text,"selectedAnnotation":nullable_text,
        "selectedView":nullable_text,"entered":boolean,
        "datums":{"type":"array","items":{"type":"array","items":text,"minItems":2,"maxItems":2}},
        "report":{"anyOf":[reference("Report"),{"type":"null"}]}}),
        &["views","idCounter","activeView","openAnnotation","openView","selectedAnnotation","selectedView","entered","datums","report"]);
    result["$defs"] = defs;
    result
}
