//! The kernel's feature catalogue (`brep_kernel::feature_schema_catalogue`,
//! reached through `brep_render::features`) rendered as JSON Schema.
//!
//! The conversion is a function of the catalogue's parameter **types**
//! (`number`, `string`, `boolean`, `options`, `reference_selection`,
//! `boolean_operation`, `transform`, …) applied to whatever the catalogue
//! reports at runtime. There is no per-feature code here: a feature added to
//! the kernel needs nothing added in this crate. The mapping mirrors the form
//! engine's (`brep_render::features::form_fields_from_schema`) so what the
//! dialog can edit and what a tool call may pass are the same set.
use brep_render::features;
use serde_json::{json, Map, Value};

/// The boolean operations the form engine offers. The catalogue stores only a
/// default operation, not the variant set; the form engine hard-codes this
/// list (`BREP_render/src/features.rs`), and the schema must agree with it.
pub const BOOLEAN_OPS: &[&str] = &["NONE", "UNION", "SUBTRACT", "INTERSECT"];

/// Parameter types the form engine does not render and a tool call may not
/// set. They are listed under `x-unmapped` so a caller sees they exist.
pub const UNMAPPED_TYPES: &[&str] = &[
    "button",
    "spline_points",
    "file",
    "textarea",
    "thread_designation",
];

/// The raw catalogue, exactly as the kernel reports it.
pub fn catalogue() -> Value {
    features::feature_catalogue()
}

/// Every feature entry of the catalogue, in the kernel's presentation order.
pub fn entries() -> Vec<Value> {
    catalogue()
        .get("features")
        .and_then(Value::as_array)
        .cloned()
        .unwrap_or_default()
}

/// One catalogue entry by `type` or `shortName`.
pub fn entry(feature_type: &str) -> Option<Value> {
    features::feature_schema(feature_type)
}

/// The identity fields of a catalogue entry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FeatureIdentity {
    pub feature_type: String,
    pub short_name: String,
    pub long_name: String,
    pub display_builder: bool,
}

pub fn identity(entry: &Value) -> FeatureIdentity {
    let s = |k: &str| entry.get(k).and_then(Value::as_str).unwrap_or("").to_string();
    FeatureIdentity {
        feature_type: s("type"),
        short_name: s("shortName"),
        long_name: s("longName"),
        display_builder: entry
            .get("displayBuilder")
            .and_then(Value::as_bool)
            .unwrap_or(false),
    }
}

/// The JSON Schema for one feature's `inputParams`, derived from its
/// `inputParamsSchema`. `additionalProperties` is `false` so an unknown key is
/// rejected before the engine sees it (see [`crate::validate`]).
pub fn to_json_schema(entry: &Value) -> Value {
    let id = identity(entry);
    let mut properties = Map::new();
    let mut unmapped = Vec::new();
    if let Some(params) = entry.get("inputParamsSchema").and_then(Value::as_object) {
        for (name, spec) in params {
            match param_schema(name, spec) {
                Some(schema) => {
                    properties.insert(name.clone(), schema);
                }
                None => unmapped.push(json!({
                    "name": name,
                    "type": spec.get("type").cloned().unwrap_or(Value::Null),
                })),
            }
        }
    }
    json!({
        "$schema": "https://json-schema.org/draft/2020-12/schema",
        "title": id.long_name,
        "description": format!("inputParams of the {} feature (type `{}`)", id.long_name, id.feature_type),
        "type": "object",
        "properties": Value::Object(properties),
        "additionalProperties": false,
        "x-feature": {
            "type": id.feature_type,
            "shortName": id.short_name,
            "longName": id.long_name,
            "displayBuilder": id.display_builder,
        },
        "x-unmapped": unmapped,
    })
}

/// The JSON Schema for one parameter, or `None` when the form engine does not
/// map its type either.
fn param_schema(name: &str, spec: &Value) -> Option<Value> {
    let ty = spec.get("type").and_then(Value::as_str).unwrap_or("");
    let hint = spec.get("hint").and_then(Value::as_str);
    let label = spec.get("label").and_then(Value::as_str);
    let mut description = String::new();
    if let Some(l) = label {
        description.push_str(l);
    }
    if let Some(h) = hint {
        if !description.is_empty() {
            description.push_str(" — ");
        }
        description.push_str(h);
    }
    let mut schema = match ty {
        "number" => {
            let mut num = Map::new();
            num.insert("type".into(), json!("number"));
            if let Some(min) = spec.get("min") {
                num.insert("minimum".into(), min.clone());
            }
            if let Some(max) = spec.get("max") {
                num.insert("maximum".into(), max.clone());
            }
            json!({
                "oneOf": [
                    Value::Object(num),
                    { "type": "string", "description": "an expression over the Expressions panel variables and `configurator.*`" }
                ]
            })
        }
        "string" => {
            let mut s = json!({ "type": "string" });
            if name == "id" {
                s["readOnly"] = json!(true);
                s["description"] = json!("the feature's unique id; assigned by the engine, never edited");
            }
            s
        }
        "boolean" => json!({ "type": "boolean" }),
        "vec3" => {
            // A bare three-component vector param (Transform Face's `pivot`).
            // Each component is a number OR an expression string, exactly as the
            // kernel's shared vec3 reader and the form's Vec3 widget accept — the
            // same `oneOf` a `number` param carries, one per slot.
            let component = json!({
                "oneOf": [
                    { "type": "number" },
                    { "type": "string", "description": "an expression over the Expressions panel variables and `configurator.*`" }
                ]
            });
            json!({
                "type": "array",
                "items": component,
                "minItems": 3,
                "maxItems": 3
            })
        }
        "options" => {
            let options = spec.get("options").cloned().unwrap_or_else(|| json!([]));
            json!({ "enum": options })
        }
        "reference_selection" => {
            let filter = spec
                .get("selectionFilter")
                .and_then(Value::as_array)
                .map(|a| {
                    a.iter()
                        .filter_map(Value::as_str)
                        .collect::<Vec<_>>()
                        .join(", ")
                })
                .unwrap_or_default();
            let multiple = spec.get("multiple").and_then(Value::as_bool).unwrap_or(false);
            // The field's own `label — hint` leads, as for every other kind;
            // the reference note follows rather than replacing them.
            let accepts = format!("a reference name; accepts: {filter}");
            let note = if description.is_empty() {
                accepts
            } else {
                format!("{description} — {accepts}")
            };
            if multiple {
                json!({ "type": "array", "items": { "type": "string" }, "x-selectionFilter": spec.get("selectionFilter").cloned().unwrap_or(json!([])), "description": note })
            } else {
                json!({ "type": ["string", "null"], "x-selectionFilter": spec.get("selectionFilter").cloned().unwrap_or(json!([])), "description": note })
            }
        }
        "boolean_operation" => json!({
            "type": "object",
            "properties": {
                "operation": { "enum": BOOLEAN_OPS },
                "targets": { "type": "array", "items": { "type": "string" }, "description": "tool solids (SOLID references)" },
                "mergeCoplanarFaces": { "type": "boolean" }
            },
            "additionalProperties": false
        }),
        "transform" => {
            let default = spec.get("default_value");
            let rigid = default
                .map(|d| d.get("translate").is_some() || d.get("rotateEulerDeg").is_some())
                .unwrap_or(false);
            let vec3 = json!({ "type": "array", "items": { "oneOf": [{"type":"number"},{"type":"string"}] }, "minItems": 3, "maxItems": 3 });
            if rigid {
                json!({
                    "type": "object",
                    "properties": { "translate": vec3, "rotateEulerDeg": vec3 },
                    "additionalProperties": false
                })
            } else {
                let properties: serde_json::Map<String, Value> = modeling_transform_keys(spec)
                    .into_iter()
                    .map(|key| (key.to_string(), vec3.clone()))
                    .collect();
                json!({
                    "type": "object",
                    "properties": properties,
                    "additionalProperties": false
                })
            }
        }
        _ => return None,
    };
    if !description.is_empty() && schema.get("description").is_none() {
        schema["description"] = json!(description);
    }
    if let Some(default) = spec.get("default_value") {
        schema["default"] = default.clone();
    }
    schema["x-paramType"] = json!(ty);
    Some(schema)
}

/// The keys a MODELING `transform` param (`{position, rotationEuler, scale}`)
/// takes: those its schema `default_value` carries, the same discriminator the
/// form engine reads. A rigid-motion feature (Transform Face) omits `scale`
/// from its default, and so has no `scale` here — neither offered in the tool
/// schema nor accepted by the validator. A default that names none of the three
/// keeps all three.
pub(crate) fn modeling_transform_keys(spec: &Value) -> Vec<&'static str> {
    const ALL: [&str; 3] = ["position", "rotationEuler", "scale"];
    let default = spec.get("default_value");
    let declared: Vec<&'static str> = ALL
        .into_iter()
        .filter(|key| default.is_some_and(|d| d.get(key).is_some()))
        .collect();
    if declared.is_empty() { ALL.to_vec() } else { declared }
}

/// A feature's default `inputParams` (one key per schema param, seeded from
/// `default_value`; `id` is `null` until the engine assigns one).
pub fn defaults(feature_type: &str) -> Value {
    features::feature_default_params(feature_type)
}

/// Defaults from a document-owned schema, including an installed plugin.
pub fn defaults_from_entry(entry: &Value) -> Value {
    let params = entry["inputParamsSchema"].as_object();
    Value::Object(params.into_iter().flatten().map(|(key, spec)| {
        (key.clone(), spec.get("default_value").cloned().unwrap_or(Value::Null))
    }).collect())
}

/// Does this feature carry a SKETCH in its `persistentData`?
///
/// The two type codes the engine's own catalogue answers to for the sketch
/// producer (`brep_render::features` matches the same pair for its icon), so
/// this asks the catalogue rather than carrying a list of sketch-ish features.
pub fn carries_sketch(feature_type: &str) -> bool {
    matches!(feature_type, "S" | "SKETCH")
}

/// The JSON Schema of a sketch feature's `persistentData` — the block a caller
/// must supply to build a profile, and the one thing `inputParamsSchema` cannot
/// describe (it covers `inputParams` only, so `sketchPlane` and `projectedEdges`
/// were all a caller could see; the geometry lived in an undocumented sibling).
///
/// Shaped after `brep_render::sketch::SketchDoc` — points, geometries and
/// constraints — and pinned to it by `the_sketch_example_round_trips`, which
/// parses [`sketch_example`] with the real type and requires an identical
/// re-serialisation. A change to the type that this schema does not follow
/// fails that test.
pub fn sketch_persistent_schema() -> Value {
    json!({
        "type": "object",
        "description": "A sketch feature's persistentData. `sketch` holds the profile geometry; `basis` gives the plane when `inputParams.sketchPlane` is null.",
        "properties": {
            "basis": {
                "type": "object",
                "description": "The sketch plane's frame in world space, used when `inputParams.sketchPlane` names nothing. Sketch coordinates are (u, v) in this frame: `origin + u*x + v*y`.",
                "properties": {
                    "origin": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                    "x": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                    "y": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 },
                    "z": { "type": "array", "items": { "type": "number" }, "minItems": 3, "maxItems": 3 }
                }
            },
            "sketch": {
                "type": "object",
                "description": "The profile. Geometry references POINT IDS, never inline coordinates, so two segments meet exactly when they name the same point id — which is also what makes a loop closed. An open chain is a path (a sweep spine), not a face: a solid needs a closed loop.",
                "properties": {
                    "points": {
                        "type": "array",
                        "description": "The sketch's points, in plane-local (u, v) millimetres.",
                        "items": {
                            "type": "object",
                            "properties": {
                                "id": { "description": "Stable id, referenced by `geometries[].points`. A number in practice; a string is legal." },
                                "x": { "type": "number", "description": "u, in mm" },
                                "y": { "type": "number", "description": "v, in mm" },
                                "fixed": { "type": "boolean", "description": "Pinned: the solver never moves it. True is the right default for authored geometry." },
                                "construction": { "type": "boolean" },
                                "externalReference": { "type": "boolean", "description": "Adopted from a picked edge endpoint." }
                            },
                            "required": ["id", "x", "y"]
                        }
                    },
                    "geometries": {
                        "type": "array",
                        "description": "The segments, each naming point ids: line = [start, end]; arc = [centre, start, end] (counter-clockwise start→end); circle = [centre, radiusPoint]; ellipse = [centre, majorEnd, minorEnd]; bezier = [p0, p1, p2, p3, …] (every three ids a further cubic span).",
                        "items": {
                            "type": "object",
                            "properties": {
                                "id": { "description": "Stable id. A revolve axis names a sketch line as `{sketchFeatureId}:G{id}`." },
                                "type": { "type": "string", "enum": ["line", "arc", "circle", "ellipse", "bezier"] },
                                "points": { "type": "array", "description": "Point ids, in the order this geometry type defines." },
                                "construction": { "type": "boolean", "description": "Constrains but never models: excluded from profiles, drawn dashed." }
                            },
                            "required": ["id", "type", "points"]
                        }
                    },
                    "constraints": {
                        "type": "array",
                        "description": "Solver constraints. `[]` is valid and is what authored-by-coordinate geometry uses — every point `fixed: true` needs no solving.",
                        "items": { "type": "object" }
                    }
                },
                "required": ["points", "geometries"]
            }
        }
    })
}

/// A minimal, real sketch `persistentData`: a closed unit square on the world
/// XY plane with no constraints. Parsed back through the engine's own
/// `SketchDoc` by the test below, so it cannot describe a shape the type does
/// not accept.
pub fn sketch_example() -> Value {
    json!({
        "basis": { "origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0], "z": [0.0, 0.0, 1.0] },
        "sketch": {
            "points": [
                { "id": 1, "x": 0.0, "y": 0.0, "fixed": true, "construction": false, "externalReference": false },
                { "id": 2, "x": 10.0, "y": 0.0, "fixed": true, "construction": false, "externalReference": false },
                { "id": 3, "x": 10.0, "y": 10.0, "fixed": true, "construction": false, "externalReference": false },
                { "id": 4, "x": 0.0, "y": 10.0, "fixed": true, "construction": false, "externalReference": false }
            ],
            "geometries": [
                { "id": 10, "type": "line", "points": [1, 2], "construction": false },
                { "id": 11, "type": "line", "points": [2, 3], "construction": false },
                { "id": 12, "type": "line", "points": [3, 4], "construction": false },
                { "id": 13, "type": "line", "points": [4, 1], "construction": false }
            ],
            "constraints": []
        },
        "dimOffsets": {},
        "externalRefs": []
    })
}

/// The whole catalogue rendered: `{version, features: [{type, shortName,
/// longName, displayBuilder, schema, defaults}]}`. What `brep://schema/features`
/// serves and what `brep-mcp schema` prints.
pub fn all() -> Value {
    let cat = catalogue();
    all_from_catalogue(&cat)
}

/// Convert a live document's owned catalogue using the same public schema
/// contract as the sessionless built-in resources.
pub fn all_from_catalogue(cat: &Value) -> Value {
    let features: Vec<Value> = cat["features"].as_array().into_iter().flatten()
        .map(|e| {
            let id = identity(e);
            let mut entry = json!({
                "type": id.feature_type,
                "shortName": id.short_name,
                "longName": id.long_name,
                "displayBuilder": id.display_builder,
                "schema": to_json_schema(e),
                "defaults": defaults_from_entry(e),
            });
            // Only the features that HAVE one carry the key: a `null` on every
            // other feature is noise in a checked-in file.
            if carries_sketch(&id.feature_type) {
                entry["persistentData"] = sketch_persistent_schema();
                entry["persistentDataExample"] = sketch_example();
            }
            entry
        })
        .collect();
    json!({
        "version": cat.get("version").cloned().unwrap_or(Value::Null),
        "features": features,
    })
}

