//! The output-schema vocabulary every `*_output_schema.rs` builds with.
//!
//! Each object constructor names what its list means. The files used to carry
//! their own `fn object(props, &[..])`, and the second argument meant the
//! REQUIRED keys in some files and the OPTIONAL keys in others, so a schema
//! line moved between two files silently inverted which fields were required
//! (duplicate-code record U4d, 2026-11-01). Every constructor here closes the
//! object (`additionalProperties: false`).
use serde_json::{json, Value};

/// A closed object whose `required` list is exactly `required`, in that order.
pub(super) fn closed_object_required(properties: Value, required: &[&str]) -> Value {
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}

/// A closed object where every property is required except those in `optional`
/// (`required` follows the properties' order).
pub(super) fn closed_object_optional(properties: Value, optional: &[&str]) -> Value {
    let required: Vec<_> = properties.as_object().unwrap().keys().filter(|k| !optional.contains(&k.as_str())).cloned().collect();
    json!({"type":"object","properties":properties,"required":required,"additionalProperties":false})
}

/// A closed object where every property is required.
pub(super) fn closed_object_all(properties: Value) -> Value {
    closed_object_optional(properties, &[])
}

pub(super) fn array(items: Value) -> Value {
    json!({"type":"array","items":items})
}

/// A `$ref` into the document's own `$defs`.
pub(super) fn reference(name: &str) -> Value {
    json!({"$ref":format!("#/$defs/{name}")})
}

/// A fixed-length number array (`vector(3)` is a point or direction).
pub(super) fn vector(n: usize) -> Value {
    json!({"type":"array","items":{"type":"number"},"minItems":n,"maxItems":n})
}

pub(super) fn nullable(value: Value) -> Value {
    json!({"anyOf":[value,{"type":"null"}]})
}
