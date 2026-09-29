//! The PMI annotation type table — nine types, each a module exposing its
//! [`PmiTypeDef`] (`DEF`), its `schema()` and its resolver. The table is the
//! third applicability family beside the feature catalogue and the assembly
//! constraint types: an annotation predicate accepts plain AND component
//! geometry.

pub mod angle;
pub mod datum;
pub mod explode;
pub mod fcf;
pub mod hole_callout;
pub mod leader;
pub mod linear;
pub mod note;
pub mod radial;

use super::{PmiAnnotation, PmiContext, Resolved};
use crate::feature_pipeline::SelectionProbe;
use serde_json::{Map, Value};

/// One annotation type.
pub struct PmiTypeDef {
    /// Canonical `type` string (the persisted key).
    pub type_id: &'static str,
    /// Id-mint prefix (`DIM` → `DIM3`).
    pub short_name: &'static str,
    /// The type's ONE icon — a catalogued glyph character (the app's
    /// `assets/glyphs` set). Every surface that stands for the type by picture
    /// (tree row, viewport chip, context offer) reads this.
    pub icon: &'static str,
    /// `"{icon} {label}"`.
    pub long_name: &'static str,
    /// Plain label for messages and the tree.
    pub label: &'static str,
    /// Does the current selection make creating this annotation meaningful?
    pub applicable: fn(&SelectionProbe) -> bool,
    /// The annotation's schema (`{type, shortName, longName, label, icon,
    /// inputParamsSchema}`), the shape the dialog engine consumes.
    pub schema: fn() -> Value,
    /// Resolve the annotation against the scene.
    pub resolve: fn(&PmiAnnotation, &PmiContext<'_>) -> Result<Resolved, String>,
}

/// Panel / `+` dropdown order.
pub const PMI_TYPES: [PmiTypeDef; 9] = [
    linear::DEF,
    radial::DEF,
    angle::DEF,
    leader::DEF,
    note::DEF,
    hole_callout::DEF,
    datum::DEF,
    fcf::DEF,
    explode::DEF,
];

/// Look up a type by its canonical `type` string.
pub fn pmi_type(type_id: &str) -> Option<&'static PmiTypeDef> {
    PMI_TYPES.iter().find(|def| def.type_id == type_id)
}

/// The nine annotation schemas, table order.
pub fn pmi_schema_catalogue() -> Value {
    Value::Array(PMI_TYPES.iter().map(|def| (def.schema)()).collect())
}

// --- shared schema-field builders --------------------------------------------

pub(super) fn id_field() -> Value {
    serde_json::json!({
        "type": "string",
        "default_value": null,
        "hint": "Unique identifier for the annotation"
    })
}

pub(super) fn reference_field(
    label: &str,
    filter: &[&str],
    multiple: bool,
    min: usize,
    max: usize,
    hint: &str,
) -> Value {
    serde_json::json!({
        "type": "reference_selection",
        "label": label,
        "selectionFilter": filter,
        "multiple": multiple,
        "minSelections": min,
        "maxSelections": max,
        "default_value": null,
        "hint": hint
    })
}

pub(super) fn number_field(label: &str, default: f64, step: f64, hint: &str) -> Value {
    serde_json::json!({
        "type": "number",
        "label": label,
        "default_value": default,
        "step": step,
        "hint": hint
    })
}

pub(super) fn boolean_field(label: &str, default: bool, hint: &str) -> Value {
    serde_json::json!({
        "type": "boolean",
        "label": label,
        "default_value": default,
        "hint": hint
    })
}

pub(super) fn string_field(label: &str, default: &str, hint: &str) -> Value {
    serde_json::json!({
        "type": "string",
        "label": label,
        "default_value": default,
        "hint": hint
    })
}

pub(super) fn options_field(label: &str, options: &[&str], default: &str, hint: &str) -> Value {
    serde_json::json!({
        "type": "options",
        "label": label,
        "options": options,
        "default_value": default,
        "hint": hint
    })
}

/// The dimension tolerance block + decimals + reference flag, appended to a
/// dimension schema's params in this order.
pub(super) fn dimension_fields(params: &mut Map<String, Value>, decimals_default: u64) {
    params.insert(
        "decimals".into(),
        serde_json::json!({
            "type": "number",
            "label": "Decimals",
            "default_value": decimals_default,
            "step": 1,
            "hint": "Decimal places shown (0–8)"
        }),
    );
    params.insert(
        "isReference".into(),
        boolean_field("Reference", false, "A reference dimension: shown in parentheses, no tolerance"),
    );
    params.insert(
        "tolMode".into(),
        options_field(
            "Tolerance",
            &["none", "symmetric", "deviation", "limits"],
            "none",
            "none · ± symmetric · +upper/−lower deviation · upper/lower limit values",
        ),
    );
    params.insert(
        "tolUpper".into(),
        number_field("Upper (+)", 0.0, 0.01, "The upper deviation (the ± value for symmetric)"),
    );
    params.insert(
        "tolLower".into(),
        number_field("Lower (−)", 0.0, 0.01, "The lower deviation (deviation / limits modes)"),
    );
}

/// The annotation-plane field every drawn type carries (explode draws
/// nothing): a planar face or reference plane the annotation lies in;
/// empty aligns it to the view camera.
pub(super) fn plane_field() -> Value {
    reference_field(
        "Annotation plane",
        &["FACE", "PLANE"],
        false,
        0,
        1,
        "A planar face or reference plane the annotation lies in — leave empty to align it to the view camera",
    )
}

pub(super) fn schema_entry(def: &PmiTypeDef, mut params: Map<String, Value>) -> Value {
    if def.type_id != "explode" {
        params.insert("plane".into(), plane_field());
    }
    serde_json::json!({
        "type": def.type_id,
        "shortName": def.short_name,
        "longName": def.long_name,
        "label": def.label,
        "icon": def.icon,
        "inputParamsSchema": Value::Object(params),
    })
}

/// Build a params object in insertion order.
pub(super) fn params(entries: Vec<(&str, Value)>) -> Map<String, Value> {
    let mut map = Map::new();
    for (key, value) in entries {
        map.insert(key.to_string(), value);
    }
    map
}

/// The decimals param clamped to `0..=8`.
pub(super) fn decimals_of(annotation: &PmiAnnotation, context: &PmiContext<'_>, default: f64) -> usize {
    annotation
        .number("decimals", context.env, default)
        .unwrap_or(default)
        .round()
        .clamp(0.0, 8.0) as usize
}
