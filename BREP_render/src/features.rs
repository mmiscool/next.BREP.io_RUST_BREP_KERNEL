//! Map kernel feature schemas to the shared [`FormField`] representation.
//!
//! Feature definitions come from [`brep_kernel::feature_schema_catalogue`].
//! Parameters map to scalar, text, vector, choice, boolean, and reference fields;
//! a boolean operation expands into operation, tool-selection, and merge fields.
//! Transform start references are not surfaced here.

use crate::style::{FieldKind, FormField};
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::sync::OnceLock;

/// The catalogue, built ONCE per process. Every entry is static kernel data
/// (each feature's `schema()` is a literal), so the first call pays the 40-odd
/// `json!` constructions and every later read is a pointer. Before this cache
/// the history tree rebuilt the whole catalogue for EVERY ROW it drew — a
/// 633-component assembly spent ~1.7 s per frame on it, which is what made an
/// imported STEP assembly impossible to orbit.
fn cached_catalogue() -> &'static Value {
    static CATALOGUE: OnceLock<Value> = OnceLock::new();
    CATALOGUE.get_or_init(brep_kernel::feature_schema_catalogue)
}

/// `type` AND `shortName` → position in the catalogue's `features` array. Built
/// in array order with first-match-wins, so a lookup answers exactly what a
/// linear `find` over the array did.
fn schema_index() -> &'static HashMap<String, usize> {
    static INDEX: OnceLock<HashMap<String, usize>> = OnceLock::new();
    INDEX.get_or_init(|| {
        let mut index = HashMap::new();
        if let Some(list) = cached_catalogue().get("features").and_then(Value::as_array) {
            for (position, feature) in list.iter().enumerate() {
                for key in ["type", "shortName"] {
                    if let Some(name) = feature.get(key).and_then(Value::as_str) {
                        index.entry(name.to_string()).or_insert(position);
                    }
                }
            }
        }
        index
    })
}

/// One feature's schema entry, borrowed from the cached catalogue.
fn schema_entry(feature_type: &str) -> Option<&'static Value> {
    let position = *schema_index().get(feature_type)?;
    cached_catalogue().get("features")?.as_array()?.get(position)
}

/// The kernel-owned feature-schema catalogue (`{ version, features: [...] }`).
/// A clone of the process-wide cache: callers that only READ it (the palette,
/// the workbench toolbar, the context bar) pay one clone per call instead of
/// the whole construction.
pub fn feature_catalogue() -> Value {
    cached_catalogue().clone()
}

/// One feature's full schema entry, by its `type` / `shortName` (e.g. `"P.CU"`,
/// `"B"`). `None` if the type is not in the catalogue.
pub fn feature_schema(feature_type: &str) -> Option<Value> {
    schema_entry(feature_type).cloned()
}

/// The param keys this feature's dialog should HIDE for the given CURRENT
/// `values` — the kernel's field-visibility hook ([`brep_kernel::feature_hidden_params`]),
/// returned as a set so the form engine can skip a field in O(1). Empty when the
/// feature declares no hook, so every field stays visible. The form engine
/// recomputes this each frame from the live params, which is what makes fields
/// appear and disappear as their governing field changes.
pub fn feature_hidden_params(feature_type: &str, values: &Value) -> HashSet<String> {
    brep_kernel::feature_hidden_params(feature_type, values)
        .into_iter()
        .collect()
}

/// Human display name for a feature type: the schema `longName` (else the type
/// itself), with the feature's icon character PREPENDED. Every consumer that
/// shows a feature name (add-feature palette, history tree, context-bar offers,
/// dialogs) reads it through here, so the icon appears everywhere with no
/// per-caller wiring — see [`feature_icon`].
pub fn feature_long_name(feature_type: &str) -> String {
    let name = feature_plain_name(feature_type);
    match feature_icon(feature_type) {
        Some(glyph) => format!("{glyph} {name}"),
        None => name,
    }
}

/// The same display name WITHOUT the leading glyph — the schema `longName`, else
/// the type itself.
///
/// For the one kind of caller that draws the icon as ARTWORK rather than as a
/// character: the history tree gives the icon its own column
/// (`brep_app::panels::tree::TreeRow::glyph`), so its label must not carry the
/// glyph a second time. Everything that shows a feature name as plain text
/// keeps reading [`feature_long_name`], which is still where the icon is
/// prepended.
pub fn feature_plain_name(feature_type: &str) -> String {
    schema_entry(feature_type)
        .and_then(|f| f.get("longName").and_then(Value::as_str).map(String::from))
        .unwrap_or_else(|| feature_type.to_string())
}

/// A feature type's SHORT name (`shortName`, e.g. `P.CU`, `S`, `E`), else the
/// type code itself. This is the BASE for a new feature's id: the engine appends
/// the part history's persistent global counter to it (`P.CU` → `P.CU7`,
/// `S` → `S8`) in [`crate::history::History::next_feature_id`].
pub fn feature_short_name(feature_type: &str) -> String {
    schema_entry(feature_type)
        .and_then(|f| f.get("shortName").and_then(Value::as_str).map(String::from))
        .unwrap_or_else(|| feature_type.to_string())
}

/// The icon character for a feature type, if one exists. Every kernel
/// feature type has a FreeCAD-inspired monoline glyph in the font's Private-Use
/// block U+E030-E059 (edited as `BREP_app/assets/glyphs/*.svg`, which are also
/// the source of the inline SVG icon catalog — see `BREP_app/src/icons.rs`;
/// `every_feature_type_has_catalogued_artwork` there checks each of these
/// codepoints actually HAS a glyph, which this module's own test cannot). The font is
/// installed as family `"brep_icons"` at the HEAD of egui's fallback chain
/// ([`crate`]-side `fonts.rs`), so a bare returned `char` renders as the icon.
///
/// Matches the SAME alias set the kernel dispatch does (short codes AND the
/// long-/class-name strings saved files carry — e.g. `"CHAMFER"`, the misspelt
/// `"DATIUM"`, `"PUSH FACE"`), uppercased, so a type loaded from disk resolves.
pub fn feature_icon(kind: &str) -> Option<char> {
    let cp: u32 = match kind.trim().to_ascii_uppercase().as_str() {
        "D" | "DATUM" | "DATIUM" => 0xE030,
        "P" | "PLANE" => 0xE031,
        "P.CU" | "CUBE" => 0xE032,
        "P.CY" | "CYLINDER" => 0xE033,
        "P.CO" | "CONE" => 0xE034,
        "P.S" | "SPHERE" => 0xE035,
        "P.T" | "TORUS" => 0xE036,
        "P.PY" | "PYRAMID" => 0xE037,
        "IMPORT3D" => 0xE038,
        "S" | "SKETCH" => 0xE039,
        "SP" | "SPLINE" => 0xE03A,
        "WP" | "WAYPOINT" => 0xE03B,
        "HX" | "HELIX" => 0xE03C,
        "E" | "EXTRUDE" => 0xE03D,
        "B" | "BOOLEAN" => 0xE03E,
        "F" | "FILLET" => 0xE03F,
        "CH" | "CHAMFER" => 0xE040,
        "O.S" | "OFFSET SHELL" | "OFFSETSHELL" => 0xE041,
        "O.F" | "OFFSET FACE" | "OFFSETFACE" => 0xE042,
        "PF" | "PUSHFACE" | "PUSH FACE" => 0xE043,
        "DF" | "DELETE FACE" | "DELETEFACE" => 0xE044,
        "THK" | "THICKEN" => 0xE045,
        "SM.TAB" => 0xE046,
        "SM.CF" => 0xE047,
        "SM.F" => 0xE048,
        "SM.HEM" => 0xE049,
        "SM.FILLET" | "SM.CFIL" => 0xE04A,
        "SM.CHAMFER" | "SM.CCHM" => 0xE04B,
        "SM.CUTOUT" => 0xE04C,
        "LOFT" => 0xE04D,
        "M" | "MIRROR" => 0xE04E,
        "SPL" | "SPLIT" => 0xE04F,
        "R" | "REVOLVE" => 0xE050,
        "RIB" => 0xE051,
        "SW" | "SWEEP" => 0xE052,
        "SWP" | "PATH SWEEP" | "PATHSWEEP" => 0xE053,
        "H" | "HOLE" => 0xE054,
        "TU" | "TUBE" => 0xE055,
        "XFORM" | "TRANSFORM" => 0xE056,
        "PATTERN" => 0xE057,
        "ACOMP" | "ASSEMBLY COMPONENT" => 0xE058,
        "SM.UNFOLD" => 0xE059,
        "TF" | "TRANSFORM FACE" | "TRANSFORMFACE" => 0xE05D,
        "RFS" | "REFIT FACES" | "REFITFACES" => 0xE05E,
        _ => return None,
    };
    char::from_u32(cp)
}

/// A feature type's DEFAULT `inputParams`, built from its schema: one entry per
/// `inputParamsSchema` param seeded with that param's `default_value` (missing →
/// `null`). The caller assigns the unique `id` afterwards (the schema's `id`
/// default is `null`). Additive, engine-owned, and reusable by any "add feature"
/// path so a new feature's params always track the kernel schema. Unknown types
/// yield an empty object.
pub fn feature_default_params(feature_type: &str) -> Value {
    let mut params = serde_json::Map::new();
    if let Some(props) = feature_schema(feature_type)
        .as_ref()
        .and_then(|s| s.get("inputParamsSchema"))
        .and_then(Value::as_object)
    {
        for (name, spec) in props {
            let default = spec.get("default_value").cloned().unwrap_or(Value::Null);
            params.insert(name.clone(), default);
        }
    }
    Value::Object(params)
}

/// The boolean operation choices (a fixed kernel enum — the schema stores only a
/// default operation, not the variant set, so the known set rides here).
const BOOLEAN_OPS: &[&str] = &["NONE", "UNION", "SUBTRACT", "INTERSECT"];

/// Map a feature type's `inputParamsSchema` into the general form fields, in
/// schema (insertion) order — the kernel builds schemas with serde_json's
/// `preserve_order`, so grouped fields stay contiguous for the group headers.
pub fn feature_form_fields(feature_type: &str) -> Vec<FormField> {
    let Some(schema) = feature_schema(feature_type) else {
        return Vec::new();
    };
    form_fields_from_schema(&schema)
}

/// Map ANY schema entry carrying an `inputParamsSchema` object into form fields
/// — the shared engine behind [`feature_form_fields`] AND the assembly
/// constraint dialogs (whose schemas come from the kernel's
/// `constraint_schema_catalogue`, same shape, different catalogue). One mapping,
/// two catalogues (the schema-driven-dialog principle).
pub fn form_fields_from_schema(schema: &Value) -> Vec<FormField> {
    let Some(params) = schema
        .get("inputParamsSchema")
        .and_then(Value::as_object)
    else {
        return Vec::new();
    };

    let mut fields = Vec::new();
    for (name, spec) in params {
        let ty = spec.get("type").and_then(Value::as_str).unwrap_or("");
        match ty {
            "number" => fields.push(FormField {
                path: vec![name.clone()],
                label: field_label(spec, name),
                group: "Parameters".into(),
                kind: FieldKind::Scalar { step: 0.5 },
            }),
            "string" => fields.push(FormField {
                path: vec![name.clone()],
                label: field_label(spec, name),
                group: "Parameters".into(),
                // The `id` is the feature's identity; editing it must cascade to
                // references, so it is shown read-only for now.
                kind: FieldKind::Text {
                    read_only: name == "id",
                },
            }),
            "vec3" => {
                // A BARE three-component vector param (Transform Face's
                // `pivot`), as against the `transform` GROUP below: one
                // labelled row of three expression-capable components, no
                // rotation and no scale. The kernel reads it with the same
                // `common::vec3_from_value` the transform components use, so a
                // component may be a number or an expression either way.
                fields.push(FormField {
                    path: vec![name.clone()],
                    label: field_label(spec, name),
                    group: "Parameters".into(),
                    kind: FieldKind::Vec3 { step: 0.5 },
                });
            }
            "transform" => {
                // TWO transform param shapes share the `transform` schema type,
                // discriminated by the schema's own `default_value` (the shape a
                // fresh feature is seeded with, so it can never lie):
                //   * the ACOMP rigid instance pose `{translate, rotateEulerDeg}`
                //     (no scale, degrees, intrinsic XYZ);
                //   * the modeling `{position, rotationEuler, scale}` triple every
                //     modeling feature uses — WITHOUT its Scale row when the
                //     default carries no `scale`, which is how a rigid-motion
                //     feature (Transform Face) says scale means nothing to it.
                // The mapping lives HERE (the form-engine altitude) so no dialog
                // ever special-cases a feature type.
                let is_rigid_pose = spec
                    .get("default_value")
                    .map(|d| d.get("translate").is_some() || d.get("rotateEulerDeg").is_some())
                    .unwrap_or(false);
                if is_rigid_pose {
                    fields.push(vec3_field(name, "translate", "Translate", 0.5));
                    fields.push(vec3_field(name, "rotateEulerDeg", "Rotation (deg)", 1.0));
                } else {
                    fields.push(vec3_field(name, "position", "Position", 0.5));
                    fields.push(vec3_field(name, "rotationEuler", "Rotation (deg)", 1.0));
                    let scales = spec
                        .get("default_value")
                        .map(|d| d.get("scale").is_some())
                        .unwrap_or(true);
                    if scales {
                        fields.push(vec3_field(name, "scale", "Scale", 0.1));
                    }
                }
            }
            "boolean" => {
                // A plain checkbox param (e.g. ACOMP `isFixed`, constraint
                // `reverse`/`opposeNormals`/`exteriorAngle`). Label from the
                // schema's `label` when present, else the prettified key.
                fields.push(FormField {
                    path: vec![name.clone()],
                    label: field_label(spec, name),
                    group: "Parameters".into(),
                    kind: FieldKind::Bool,
                });
            }
            "boolean_operation" => {
                fields.push(FormField {
                    path: vec![name.clone(), "operation".into()],
                    label: "Operation".into(),
                    group: "Boolean".into(),
                    kind: FieldKind::Enum {
                        variants: BOOLEAN_OPS.iter().map(|s| s.to_string()).collect(),
                    },
                });
                fields.push(FormField {
                    path: vec![name.clone(), "targets".into()],
                    label: "Tool solids".into(),
                    group: "Boolean".into(),
                    kind: FieldKind::Reference {
                        filter: vec!["SOLID".into()],
                        multiple: true,
                    },
                });
                fields.push(FormField {
                    path: vec![name.clone(), "mergeCoplanarFaces".into()],
                    label: "Merge coplanar faces".into(),
                    group: "Boolean".into(),
                    kind: FieldKind::Bool,
                });
            }
            "button" => {
                // An action button (e.g. Edit Sketch): binds to no value. Its
                // `key` (the schema key) identifies the click to the host. The
                // caption is the schema `label` (falling back to the key).
                let label = field_label(spec, name);
                fields.push(FormField {
                    path: vec![name.clone()],
                    label: label.clone(),
                    group: "Parameters".into(),
                    kind: FieldKind::Button { label },
                });
            }
            "reference_selection" => {
                let filter = crate::json_support::string_values(spec.get("selectionFilter"))
                    .map(String::from)
                    .collect();
                let multiple = spec
                    .get("multiple")
                    .and_then(Value::as_bool)
                    .unwrap_or(false);
                // `"References"` is a SEMANTIC TAG, not a rendered wrapper: the
                // history tree inlines this group (each reference renders as its
                // own self-titled node — no "References" parent), and the context
                // bar keys on it to find a feature's PRE-FILL fields (top-level
                // `reference_selection`s, vs a boolean-op `targets` in `Boolean`;
                // WHETHER a feature is offered is the kernel predicate's call —
                // `feature_pipeline::context_offer`).
                // The label is the schema's own, like every other kind: it is
                // the form row's title AND, carried by the Select intent, the
                // picker's prompt.
                fields.push(FormField {
                    path: vec![name.clone()],
                    label: field_label(spec, name),
                    group: "References".into(),
                    kind: FieldKind::Reference { filter, multiple },
                });
            }
            "options" => {
                // A single-choice enum: render as a dropdown of the schema's
                // `options`, using the field's `label` when present (e.g.
                // flangeLengthReference → "Length reference", inset → "Flange
                // position") else the prettified key.
                let variants = crate::json_support::string_values(spec.get("options"))
                    .map(String::from)
                    .collect();
                fields.push(FormField {
                    path: vec![name.clone()],
                    label: field_label(spec, name),
                    group: "Parameters".into(),
                    kind: FieldKind::Enum { variants },
                });
            }
            // Unmapped kernel param types (vec3-array/etc. arrive as more
            // features are wired) are skipped rather than mis-rendered.
            _ => {}
        }
    }
    fields
}

fn vec3_field(param: &str, sub: &str, label: &str, step: f64) -> FormField {
    FormField {
        path: vec![param.to_string(), sub.to_string()],
        label: label.to_string(),
        group: "Transform".into(),
        kind: FieldKind::Vec3 { step },
    }
}

/// A field's display label: the schema `label` when set, else the prettified key.
///
/// The ONE place a form label is made, for every field kind that has a schema
/// key. The form row shows it and a reference field's Select carries it into the
/// picker's prompt. The labels the engine writes itself (a transform's
/// "Position", a boolean group's "Tool solids") have no schema key to read.
fn field_label(spec: &Value, name: &str) -> String {
    spec.get("label")
        .and_then(Value::as_str)
        .map(String::from)
        .unwrap_or_else(|| prettify(name))
}

/// `sizeX` → `Size X`, `rotationEuler` → `Rotation euler`, `id` → `Id`.
fn prettify(key: &str) -> String {
    let mut out = String::new();
    for (i, ch) in key.chars().enumerate() {
        if i == 0 {
            out.extend(ch.to_uppercase());
        } else if ch.is_ascii_uppercase() {
            out.push(' ');
            out.extend(ch.to_lowercase());
        } else {
            out.push(ch);
        }
    }
    out
}

