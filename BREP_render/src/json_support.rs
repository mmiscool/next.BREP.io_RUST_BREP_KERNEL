//! Shared JSON field readers for renderer state.

use serde_json::Value;

/// Read the first three numeric coordinates, keeping each default for missing or
/// non-numeric entries. Numeric strings are not coerced; extra entries are ignored.
pub(crate) fn vec3_or(value: Option<&Value>, default: [f64; 3]) -> [f64; 3] {
    let mut out = default;
    if let Some(array) = value.and_then(Value::as_array) {
        for (slot, item) in out.iter_mut().zip(array) {
            if let Some(number) = item.as_f64() {
                *slot = number;
            }
        }
    }
    out
}

/// Iterate string entries in array order, ignoring entries of other types.
pub(crate) fn string_values(value: Option<&Value>) -> impl Iterator<Item = &str> {
    value
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter_map(Value::as_str)
}

