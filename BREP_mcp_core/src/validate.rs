//! Feature-parameter validation against the kernel catalogue, run in the
//! server before a `feature_add` / `feature_set_params` reaches the engine.
//!
//! Rules: an unknown key is an error naming the nearest known key; a value of
//! the wrong kind is an error; an `options` value outside the list is an
//! error; a reference name absent from the scene at the feature's history
//! position is a **warning** (forward references are legal while editing, and
//! the kernel reports `unresolved` at run time). Engine refusal text is never
//! rewritten here.
use crate::schema;
use serde_json::{Map, Value};
use std::collections::HashSet;

#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct Validation {
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
}

impl Validation {
    pub fn ok(&self) -> bool {
        self.errors.is_empty()
    }
}

/// Seed every schema key from its default, then overlay `params` (a shallow
/// merge for top-level keys; nested objects such as `boolean` and `transform`
/// merge one level down so a caller can set `boolean.operation` alone).
pub fn merge_params(defaults: &Value, params: &Value) -> Value {
    let mut out = defaults.as_object().cloned().unwrap_or_default();
    if let Some(overlay) = params.as_object() {
        for (k, v) in overlay {
            match (out.get_mut(k), v) {
                (Some(Value::Object(base)), Value::Object(patch)) => {
                    for (pk, pv) in patch {
                        base.insert(pk.clone(), pv.clone());
                    }
                }
                _ => {
                    out.insert(k.clone(), v.clone());
                }
            }
        }
    }
    Value::Object(out)
}

/// Validate `params` (a full `inputParams` object) against the catalogue entry
/// for `feature_type`. `known_names`, when given, is the set of reference
/// names that exist in the scene; a reference outside it is a warning.
pub fn validate(
    feature_type: &str,
    params: &Value,
    known_names: Option<&HashSet<String>>,
) -> Validation {
    let mut v = Validation::default();
    let Some(entry) = schema::entry(feature_type) else {
        v.errors.push(format!("unknown feature type `{feature_type}`"));
        return v;
    };
    validate_entry(&entry, params, known_names)
}

/// Validate against an owned catalogue entry supplied by the active document.
/// Plugin schemas must not be cached in the process-wide built-in catalogue.
pub fn validate_entry(
    entry: &Value,
    params: &Value,
    known_names: Option<&HashSet<String>>,
) -> Validation {
    let mut v = Validation::default();
    let feature_type = entry["type"].as_str().unwrap_or("");
    let Some(spec) = entry.get("inputParamsSchema").and_then(Value::as_object) else {
        v.errors.push(format!("feature `{feature_type}` has no inputParamsSchema"));
        return v;
    };
    let Some(given) = params.as_object() else {
        v.errors.push("inputParams must be a JSON object".into());
        return v;
    };
    let known: Vec<&str> = spec.keys().map(String::as_str).collect();
    for (key, value) in given {
        let Some(pspec) = spec.get(key) else {
            let near = nearest(key, &known)
                .map(|n| format!(" (did you mean `{n}`?)"))
                .unwrap_or_default();
            v.errors.push(format!("unknown parameter `{key}`{near}"));
            continue;
        };
        check_kind(key, pspec, value, known_names, &mut v);
    }
    v
}

fn check_kind(
    key: &str,
    pspec: &Value,
    value: &Value,
    known_names: Option<&HashSet<String>>,
    v: &mut Validation,
) {
    let ty = pspec.get("type").and_then(Value::as_str).unwrap_or("");
    match ty {
        "number" => {
            if !(value.is_number() || value.is_string() || value.is_null()) {
                v.errors.push(format!("`{key}` must be a number or an expression string"));
            }
        }
        "string" => {
            if !(value.is_string() || value.is_null()) {
                v.errors.push(format!("`{key}` must be a string"));
            }
        }
        "boolean" => {
            if !value.is_boolean() {
                v.errors.push(format!("`{key}` must be a boolean"));
            }
        }
        "options" => {
            let options: Vec<&str> = pspec
                .get("options")
                .and_then(Value::as_array)
                .map(|a| a.iter().filter_map(Value::as_str).collect())
                .unwrap_or_default();
            match value.as_str() {
                Some(s) if options.contains(&s) => {}
                _ => v.errors.push(format!(
                    "`{key}` must be one of {}",
                    options.iter().map(|o| format!("`{o}`")).collect::<Vec<_>>().join(", ")
                )),
            }
        }
        "reference_selection" => {
            let multiple = pspec.get("multiple").and_then(Value::as_bool).unwrap_or(false);
            let names: Vec<&str> = match value {
                Value::Null => vec![],
                Value::String(s) if !multiple => vec![s.as_str()],
                Value::Array(a) if multiple => a.iter().filter_map(Value::as_str).collect(),
                Value::String(_) => {
                    v.errors.push(format!("`{key}` takes an array of reference names"));
                    vec![]
                }
                Value::Array(_) => {
                    v.errors.push(format!("`{key}` takes a single reference name"));
                    vec![]
                }
                _ => {
                    v.errors.push(format!("`{key}` must be a reference name"));
                    vec![]
                }
            };
            if let Some(known) = known_names {
                for n in names {
                    if !known.contains(n) {
                        v.warnings.push(format!(
                            "`{key}` references `{n}`, which is not in the scene at this point in the history"
                        ));
                    }
                }
            }
        }
        "boolean_operation" => match value.as_object() {
            Some(obj) => {
                if let Some(op) = obj.get("operation") {
                    match op.as_str() {
                        Some(s) if schema::BOOLEAN_OPS.contains(&s) => {}
                        _ => v.errors.push(format!(
                            "`{key}.operation` must be one of {}",
                            schema::BOOLEAN_OPS.join(", ")
                        )),
                    }
                }
                if let Some(t) = obj.get("targets") {
                    if !t.is_array() {
                        v.errors.push(format!("`{key}.targets` must be an array of solid names"));
                    }
                }
                for k in obj.keys() {
                    if !["operation", "targets", "mergeCoplanarFaces"].contains(&k.as_str()) {
                        v.errors.push(format!("unknown key `{key}.{k}`"));
                    }
                }
            }
            None => v.errors.push(format!("`{key}` must be an object {{operation, targets, mergeCoplanarFaces}}")),
        },
        "transform" => match value.as_object() {
            Some(obj) => {
                let rigid = pspec
                    .get("default_value")
                    .map(|d| d.get("translate").is_some() || d.get("rotateEulerDeg").is_some())
                    .unwrap_or(false);
                let allowed: Vec<&str> = if rigid {
                    vec!["translate", "rotateEulerDeg"]
                } else {
                    schema::modeling_transform_keys(pspec)
                };
                for (k, val) in obj {
                    if !allowed.contains(&k.as_str()) {
                        v.errors.push(format!("unknown key `{key}.{k}` (allowed: {})", allowed.join(", ")));
                    } else if !is_vec3_or_expression(val) {
                        v.errors.push(format!("`{key}.{k}` must be three numbers or expressions"));
                    }
                }
            }
            None => v.errors.push(format!("`{key}` must be an object")),
        },
        "vec3" => {
            // A BARE three-component vector (Transform Face's `pivot`), as
            // against the `transform` GROUP above. Null is the default; a
            // component may be a number or an expression string, which is what
            // the kernel's shared vec3 reader and the form's three components
            // both accept.
            if !(value.is_null() || is_vec3_or_expression(value)) {
                v.errors.push(format!(
                    "`{key}` must be three numbers or expression strings"
                ));
            }
        }
        _ => {
            // A type the form engine does not map either (button, file, …):
            // the engine ignores it, so a value here is noise, not an error.
            v.warnings.push(format!("`{key}` has type `{ty}`, which no tool can set; it is ignored"));
        }
    }
}

/// A three-slot vector whose components may be numbers OR expression strings —
/// what a `vec3` param takes, matching the kernel's `common::vec3_from_value`
/// and the form's three expression-capable components.
fn is_vec3_or_expression(v: &Value) -> bool {
    v.as_array()
        .map(|a| a.len() == 3 && a.iter().all(|c| c.is_number() || c.is_string()))
        .unwrap_or(false)
}

/// Nearest known key by edit distance, when close enough to be a typo.
fn nearest<'a>(key: &str, known: &[&'a str]) -> Option<&'a str> {
    known
        .iter()
        .map(|k| (levenshtein(key, k), *k))
        .filter(|(d, k)| *d <= 2 || (*d <= 3 && k.len() > 6))
        .min_by_key(|(d, _)| *d)
        .map(|(_, k)| k)
}

fn levenshtein(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.chars().collect();
    let b: Vec<char> = b.chars().collect();
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut cur = vec![i + 1];
        for (j, cb) in b.iter().enumerate() {
            let cost = if ca.eq_ignore_ascii_case(cb) { 0 } else { 1 };
            cur.push((prev[j] + cost).min(prev[j + 1] + 1).min(cur[j] + 1));
        }
        prev = cur;
    }
    prev[b.len()]
}

/// Convenience: a `Map` from key/value pairs.
pub fn object(pairs: &[(&str, Value)]) -> Value {
    let mut m = Map::new();
    for (k, v) in pairs {
        m.insert((*k).to_string(), v.clone());
    }
    Value::Object(m)
}


