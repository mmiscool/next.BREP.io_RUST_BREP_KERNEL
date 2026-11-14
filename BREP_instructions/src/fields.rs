//! Step fields: what the person doing the work records at a step.
//!
//! An author adds any number of fields to a step ([`crate::model::Step::fields`]):
//! a box to tick, a choice among options, a line or a box of text, a
//! measurement with its limits, a serial number. The players show them as
//! inputs and the printed pages as a paper form (a choice prints every option
//! as its own box to mark). The values are a build's record, never the
//! document's: the players keep them, and a jobs floor stores them with the
//! step's sign-off.
//!
//! A value is plain JSON, the same in the page, the app and a server:
//!
//! | kind | value |
//! |---|---|
//! | `check` | `true` / `false` |
//! | `choice` | the option's text; with `multiple`, an array of them; with `other`, any text |
//! | `text`, `serial` | text |
//! | `number` | a number (or text that reads as one) |
//!
//! [`evaluate`] reads one value against its field and [`blocking`] says why a
//! step cannot be completed yet, so every player and every server apply the
//! same rule.

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// The most lines a text field's box holds.
pub const MAX_TEXT_LINES: u32 = 20;
/// The most options a choice offers.
pub const MAX_OPTIONS: usize = 50;
/// The most decimals a measurement shows.
pub const MAX_DECIMALS: u8 = 6;
/// The longest text a value holds, in characters.
pub const MAX_VALUE_CHARS: usize = 4000;

fn one() -> u32 {
    1
}

fn yes() -> bool {
    true
}

fn is_true(value: &bool) -> bool {
    *value
}

fn is_one(value: &u32) -> bool {
    *value == 1
}

/// One field of a step.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StepField {
    /// `MF12`: unique in the document, never reused.
    #[serde(default)]
    pub id: String,
    /// What the field asks for (`Bearing seated`, `Shaft diameter`).
    #[serde(default)]
    pub label: String,
    /// The step is not complete until the field is filled (a `check`: ticked).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub required: bool,
    /// A line of help shown with the field.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub hint: String,
    #[serde(flatten)]
    pub kind: FieldKind,
}

/// What a field records.
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum FieldKind {
    /// One box to tick.
    Check,
    /// One of `options` (a dropdown; on paper, a box per option).
    #[serde(rename_all = "camelCase")]
    Choice {
        options: Vec<String>,
        /// Any number of the options may be picked.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        multiple: bool,
        /// A free-text "Other" value is allowed beside the options.
        #[serde(default, skip_serializing_if = "std::ops::Not::not")]
        other: bool,
    },
    /// Written text: one line, or a box of `lines` lines.
    #[serde(rename_all = "camelCase")]
    Text {
        #[serde(default = "one", skip_serializing_if = "is_one")]
        lines: u32,
    },
    /// A measurement, with its unit and limits.
    #[serde(rename_all = "camelCase")]
    Number {
        #[serde(default, skip_serializing_if = "String::is_empty")]
        unit: String,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        nominal: Option<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        min: Option<f64>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        max: Option<f64>,
        /// Decimals shown for the nominal and the limits.
        #[serde(default, skip_serializing_if = "Option::is_none")]
        decimals: Option<u8>,
        /// A value outside the limits blocks the step's completion; when
        /// false it is flagged and accepted.
        #[serde(default = "yes", skip_serializing_if = "is_true")]
        required_in_limits: bool,
    },
    /// A serial number not tied to a part (the build's own, a gauge's).
    Serial,
}

impl FieldKind {
    /// Every kind's key and label, in the order a dropdown lists them.
    pub const CHOICES: [(&'static str, &'static str); 5] =
        [("check", "Checkbox"), ("choice", "Choice"), ("text", "Text"), ("number", "Measurement"), ("serial", "Serial number")];

    /// The kind's key, as written in the document.
    pub fn key(&self) -> &'static str {
        match self {
            FieldKind::Check => "check",
            FieldKind::Choice { .. } => "choice",
            FieldKind::Text { .. } => "text",
            FieldKind::Number { .. } => "number",
            FieldKind::Serial => "serial",
        }
    }

    /// A new field of the kind named `key`, as the editor adds it.
    pub fn new(key: &str) -> Option<Self> {
        Some(match key {
            "check" => FieldKind::Check,
            "choice" => FieldKind::Choice { options: vec!["Option 1".into(), "Option 2".into()], multiple: false, other: false },
            "text" => FieldKind::Text { lines: 1 },
            "number" => FieldKind::Number { unit: String::new(), nominal: None, min: None, max: None, decimals: None, required_in_limits: true },
            "serial" => FieldKind::Serial,
            _ => return None,
        })
    }
}

/// The keys a field's JSON may carry beside `id`.
const FIELD_KEYS: [&str; 12] =
    ["label", "required", "hint", "kind", "options", "multiple", "other", "lines", "unit", "nominal", "min", "max"];
const MORE_FIELD_KEYS: [&str; 2] = ["decimals", "requiredInLimits"];
/// The keys that belong to one kind: dropped when a patch changes the kind.
const KIND_KEYS: [&str; 9] = ["options", "multiple", "other", "lines", "unit", "nominal", "min", "max", "decimals"];

impl StepField {
    /// A new field of kind `key` (see [`FieldKind::new`]) labelled `label`.
    pub fn new(id: String, key: &str, label: &str) -> Option<Self> {
        Some(Self { id, label: label.to_string(), required: false, hint: String::new(), kind: FieldKind::new(key)? })
    }

    /// Read a field from JSON (`{label, kind, …}`), refusing a key no field
    /// has. A `choice` given without options, like any kind given bare, takes
    /// the kind's defaults.
    pub fn from_json(value: &Value) -> Result<Self, String> {
        let object = value.as_object().ok_or("a field is a JSON object")?;
        check_keys(object.keys().map(String::as_str), true)?;
        let key = object.get("kind").and_then(Value::as_str).ok_or("a field names its kind: check, choice, text, number or serial")?;
        let defaults = FieldKind::new(key).ok_or_else(|| format!("field kind '{key}': check, choice, text, number or serial"))?;
        let mut merged = serde_json::to_value(StepField { id: String::new(), label: String::new(), required: false, hint: String::new(), kind: defaults })
            .map_err(|e| e.to_string())?;
        for (k, v) in object {
            merged[k.as_str()] = v.clone();
        }
        let field: StepField = serde_json::from_value(merged).map_err(|e| format!("field: {e}"))?;
        field.validate()?;
        Ok(field)
    }

    /// The field with `patch`'s keys set (`null` clears an optional one).
    /// Changing `kind` drops what belonged to the old kind and takes the new
    /// kind's defaults; the id never changes.
    pub fn patched(&self, patch: &Value) -> Result<Self, String> {
        let object = patch.as_object().ok_or("a field update is a JSON object")?;
        check_keys(object.keys().map(String::as_str), false)?;
        let mut value = serde_json::to_value(self).map_err(|e| e.to_string())?;
        if let Some(kind) = object.get("kind") {
            let key = kind.as_str().ok_or("field kind: check, choice, text, number or serial")?;
            if key != self.kind.key() {
                let defaults = FieldKind::new(key).ok_or_else(|| format!("field kind '{key}': check, choice, text, number or serial"))?;
                let map = value.as_object_mut().expect("a field serializes as an object");
                for k in KIND_KEYS.iter().chain(&["requiredInLimits"]) {
                    map.remove(*k);
                }
                let fresh = serde_json::to_value(&defaults).map_err(|e| e.to_string())?;
                for (k, v) in fresh.as_object().expect("a kind serializes as an object") {
                    map.insert(k.clone(), v.clone());
                }
            }
        }
        let map = value.as_object_mut().expect("a field serializes as an object");
        for (k, v) in object {
            if v.is_null() {
                map.remove(k);
            } else {
                map.insert(k.clone(), v.clone());
            }
        }
        let field: StepField = serde_json::from_value(value).map_err(|e| format!("field: {e}"))?;
        field.validate()?;
        Ok(field)
    }

    /// The structural rules of one field.
    pub fn validate(&self) -> Result<(), String> {
        let name = if self.label.is_empty() { self.id.as_str() } else { self.label.as_str() };
        match &self.kind {
            FieldKind::Check | FieldKind::Serial => {}
            FieldKind::Choice { options, .. } => {
                if options.is_empty() {
                    return Err(format!("field '{name}': a choice needs at least one option"));
                }
                if options.len() > MAX_OPTIONS {
                    return Err(format!("field '{name}': a choice has at most {MAX_OPTIONS} options"));
                }
            }
            FieldKind::Text { lines } => {
                if !(1..=MAX_TEXT_LINES).contains(lines) {
                    return Err(format!("field '{name}': 1 to {MAX_TEXT_LINES} lines"));
                }
            }
            FieldKind::Number { nominal, min, max, decimals, .. } => {
                if [nominal, min, max].iter().any(|v| v.is_some_and(|v| !v.is_finite())) {
                    return Err(format!("field '{name}': the nominal and the limits are finite numbers"));
                }
                if let (Some(min), Some(max)) = (min, max) {
                    if min > max {
                        return Err(format!("field '{name}': the minimum ({min}) is above the maximum ({max})"));
                    }
                }
                if decimals.is_some_and(|d| d > MAX_DECIMALS) {
                    return Err(format!("field '{name}': at most {MAX_DECIMALS} decimals"));
                }
            }
        }
        Ok(())
    }

    /// The options a choice offers, as shown: blank ones left out and a
    /// repeated one listed once. Empty for every other kind.
    pub fn options(&self) -> Vec<&str> {
        let FieldKind::Choice { options, .. } = &self.kind else { return Vec::new() };
        let mut out: Vec<&str> = Vec::new();
        for option in options {
            let option = option.trim();
            if !option.is_empty() && !out.contains(&option) {
                out.push(option);
            }
        }
        out
    }

    /// A measurement's unit; empty for every other kind.
    pub fn unit(&self) -> &str {
        match &self.kind {
            FieldKind::Number { unit, .. } => unit.trim(),
            _ => "",
        }
    }

    /// A measurement's nominal and limits as the page and the paper print
    /// them: `10.00 mm, 9.95–10.05`, `min 2.0 N·m`, `max 5`; empty when the
    /// field has none (or is no measurement). Only characters every output
    /// can draw (the PDF's base font has no ≥ or ≤).
    pub fn limits_text(&self) -> String {
        let FieldKind::Number { unit, nominal, min, max, decimals, .. } = &self.kind else { return String::new() };
        let show = |v: f64| format_number(v, *decimals);
        let unit = unit.trim();
        let with_unit = |text: String| if unit.is_empty() { text } else { format!("{text} {unit}") };
        let range = match (min, max) {
            (Some(min), Some(max)) => Some(format!("{}\u{2013}{}", show(*min), show(*max))),
            (Some(min), None) => Some(format!("min {}", show(*min))),
            (None, Some(max)) => Some(format!("max {}", show(*max))),
            (None, None) => None,
        };
        match (nominal, range) {
            (Some(nominal), Some(range)) => format!("{}, {range}", with_unit(show(*nominal))),
            (Some(nominal), None) => with_unit(show(*nominal)),
            (None, Some(range)) => with_unit(range),
            (None, None) => String::new(),
        }
    }
}

fn check_keys<'a>(keys: impl Iterator<Item = &'a str>, id_allowed: bool) -> Result<(), String> {
    for key in keys {
        if key == "id" {
            if id_allowed {
                continue;
            }
            return Err("a field's id never changes".into());
        }
        if !FIELD_KEYS.contains(&key) && !MORE_FIELD_KEYS.contains(&key) {
            return Err(format!("a field has no key '{key}'"));
        }
    }
    Ok(())
}

/// `value` with `decimals` decimals, or as short as it reads when none is set.
pub fn format_number(value: f64, decimals: Option<u8>) -> String {
    match decimals {
        Some(d) => format!("{value:.*}", usize::from(d.min(MAX_DECIMALS))),
        None => format!("{value}"),
    }
}

/// One value read against its field.
#[derive(Debug, Clone, PartialEq)]
pub struct Evaluated {
    /// The value as it is stored: trimmed text, a number for a measurement,
    /// an array for a multiple choice. `None`: nothing was entered (an
    /// unticked box included).
    pub value: Option<Value>,
    /// A measurement with a limit: whether the value is inside. `None` for
    /// every other field, a measurement without limits, and an empty one.
    pub in_limits: Option<bool>,
}

impl Evaluated {
    /// The value as one line of text, for a list or a CSV cell.
    pub fn text(&self) -> String {
        match &self.value {
            None => String::new(),
            Some(Value::Bool(true)) => "yes".into(),
            Some(Value::Bool(false)) => "no".into(),
            Some(Value::String(text)) => text.clone(),
            Some(Value::Array(items)) => items.iter().filter_map(Value::as_str).collect::<Vec<_>>().join("; "),
            Some(other) => other.to_string(),
        }
    }
}

fn text_of(field: &StepField, value: &Value) -> Result<Option<String>, String> {
    let text = value.as_str().ok_or_else(|| format!("field '{}' takes text", name_of(field)))?.trim().to_string();
    if text.chars().count() > MAX_VALUE_CHARS {
        return Err(format!("field '{}': at most {MAX_VALUE_CHARS} characters", name_of(field)));
    }
    Ok((!text.is_empty()).then_some(text))
}

fn name_of(field: &StepField) -> &str {
    if field.label.trim().is_empty() { &field.id } else { field.label.trim() }
}

/// Read `value` (absent: nothing entered) against `field`. An `Err` is a
/// value the field cannot hold (text for a tick, an option the choice does
/// not offer, something that is no number): a server refuses it.
pub fn evaluate(field: &StepField, value: Option<&Value>) -> Result<Evaluated, String> {
    let empty = Evaluated { value: None, in_limits: None };
    let Some(value) = value.filter(|v| !v.is_null()) else { return Ok(empty) };
    let name = name_of(field);
    match &field.kind {
        FieldKind::Check => match value {
            Value::Bool(true) => Ok(Evaluated { value: Some(Value::Bool(true)), in_limits: None }),
            Value::Bool(false) => Ok(empty),
            _ => Err(format!("field '{name}' is a tick: true or false")),
        },
        FieldKind::Text { .. } | FieldKind::Serial => Ok(Evaluated { value: text_of(field, value)?.map(Value::String), in_limits: None }),
        FieldKind::Choice { multiple, other, .. } => {
            let options = field.options();
            let offered = |text: &str| -> Result<(), String> {
                if *other || options.contains(&text) {
                    Ok(())
                } else {
                    Err(format!("field '{name}' offers no option '{text}'"))
                }
            };
            if *multiple {
                let items: Vec<&Value> = match value {
                    Value::Array(items) => items.iter().collect(),
                    one => vec![one],
                };
                // The options and, with `other`, one free text: no more.
                if items.len() > options.len() + usize::from(*other) + MAX_OPTIONS {
                    return Err(format!("field '{name}' takes at most {} values", options.len() + usize::from(*other)));
                }
                let mut picked: Vec<String> = Vec::new();
                for item in items {
                    if let Some(text) = text_of(field, item)? {
                        offered(&text)?;
                        if !picked.contains(&text) {
                            picked.push(text);
                        }
                    }
                }
                if picked.iter().filter(|text| !options.iter().any(|option| *option == text.as_str())).count() > 1 {
                    return Err(format!("field '{name}' takes one Other value"));
                }
                if picked.is_empty() {
                    return Ok(empty);
                }
                Ok(Evaluated { value: Some(Value::Array(picked.into_iter().map(Value::String).collect())), in_limits: None })
            } else {
                match text_of(field, value)? {
                    None => Ok(empty),
                    Some(text) => {
                        offered(&text)?;
                        Ok(Evaluated { value: Some(Value::String(text)), in_limits: None })
                    }
                }
            }
        }
        FieldKind::Number { min, max, .. } => {
            let number = match value {
                Value::Number(n) => n.as_f64(),
                Value::String(text) if text.trim().is_empty() => return Ok(empty),
                Value::String(text) => text.trim().parse::<f64>().ok(),
                _ => None,
            }
            .filter(|n| n.is_finite())
            .ok_or_else(|| format!("field '{name}' takes a number"))?;
            let in_limits = (min.is_some() || max.is_some()).then(|| min.is_none_or(|m| number >= m) && max.is_none_or(|m| number <= m));
            let stored = serde_json::Number::from_f64(number).map(Value::Number).ok_or_else(|| format!("field '{name}' takes a number"))?;
            Ok(Evaluated { value: Some(stored), in_limits })
        }
    }
}

/// Why `field` keeps its step from being completed with this value, if it
/// does: a required field left empty, or a measurement outside limits that
/// must be inside.
pub fn blocking(field: &StepField, evaluated: &Evaluated) -> Option<String> {
    let name = name_of(field);
    if evaluated.value.is_none() {
        return field.required.then(|| match field.kind {
            FieldKind::Check => format!("tick '{name}'"),
            FieldKind::Choice { .. } => format!("choose '{name}'"),
            _ => format!("enter '{name}'"),
        });
    }
    if let FieldKind::Number { required_in_limits: true, .. } = field.kind {
        if evaluated.in_limits == Some(false) {
            return Some(format!("'{name}' is out of limits ({})", field.limits_text()));
        }
    }
    None
}

/// Everything that keeps a step with `fields` from being completed, given
/// the values entered (looked up by field id): one reason per field, in
/// order. A value its field cannot hold is a reason too.
pub fn missing<'a>(fields: &[StepField], value_of: impl Fn(&str) -> Option<&'a Value>) -> Vec<String> {
    fields
        .iter()
        .filter_map(|field| match evaluate(field, value_of(&field.id)) {
            Ok(evaluated) => blocking(field, &evaluated),
            Err(error) => Some(error),
        })
        .collect()
}

