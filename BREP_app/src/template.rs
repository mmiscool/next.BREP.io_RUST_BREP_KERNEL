//! A template (`.tbrep`): a seed whose MARKED expressions are the user's
//! inputs. Inserting one never places the template: it asks for a new name
//! and the input values, spins out a unique normal part with those values
//! baked into its expressions, and places that.
//!
//! The marks live in the template document as the top-level `templateInputs`
//! block — one entry per input expression, with an optional label, limits or
//! fixed list of choices. The spun-out copy KEEPS its expressions (it is an
//! ordinary part the user can keep tweaking) and records where it came from in
//! `templateSource`: the template's file name and a content hash, for
//! traceability only. Nothing links back; changing the template never touches
//! a copy.

use crate::family_table::{part_number_problem, rewrite_expressions};
use brep_render::engine_state::EngineState;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// The document key the input marks are stored under.
pub const TEMPLATE_INPUTS_KEY: &str = "templateInputs";
/// The document key a spun-out copy records its template under.
pub const TEMPLATE_SOURCE_KEY: &str = "templateSource";

/// One marked input: an expression the user sets when inserting.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TemplateInput {
    /// The expression name (`height`).
    pub name: String,
    /// What the insert prompt calls it; empty = the name.
    #[serde(default)]
    pub label: String,
    /// Inclusive limits, when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    /// A fixed list of allowed values (expression source text). Non-empty =
    /// the prompt offers exactly these.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub choices: Vec<String>,
}

impl TemplateInput {
    /// The label the prompt shows.
    pub fn shown(&self) -> &str {
        if self.label.trim().is_empty() {
            &self.name
        } else {
            &self.label
        }
    }
}

/// What a spun-out copy records about its template.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TemplateSource {
    /// The template's file name (`bracket.tbrep`).
    pub template: String,
    /// [`template_hash`] of the template when the copy was made.
    pub content_hash: String,
}

/// The inputs a template document marks (empty when none, or unreadable).
pub fn inputs_of(document: &serde_json::Value) -> Vec<TemplateInput> {
    document
        .get(TEMPLATE_INPUTS_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
        .unwrap_or_default()
}

/// The inputs the engine's document marks. One map lookup and a small parse.
pub fn engine_inputs(engine: &EngineState) -> Vec<TemplateInput> {
    engine
        .history
        .document_block(TEMPLATE_INPUTS_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
        .unwrap_or_default()
}

/// Replace the engine document's input marks as a USER edit (one undo step;
/// `coalesce` groups a typing run). An empty list removes the block.
pub fn apply_inputs(engine: &mut EngineState, inputs: &[TemplateInput], coalesce: Option<&str>) {
    let block = (!inputs.is_empty())
        .then(|| serde_json::to_value(inputs).unwrap_or(serde_json::Value::Null));
    engine
        .history
        .set_document_block(TEMPLATE_INPUTS_KEY, block, coalesce);
}

/// The copy's record of its template, if the document is a spun-out copy.
pub fn template_source(document: &serde_json::Value) -> Option<TemplateSource> {
    document
        .get(TEMPLATE_SOURCE_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
}

/// A content hash of the whole template document less the view state it
/// reopens in.
pub fn template_hash(template: &serde_json::Value) -> String {
    let mut content = template.clone();
    if let Some(object) = content.as_object_mut() {
        object.remove("workbench");
    }
    format!("{:016x}", brep_render::brep_kernel::stable_json_hash(&content))
}

/// The `name = rhs` assignments of an expressions source, in order.
pub fn assignments(expressions: &str) -> Vec<(String, String)> {
    let mut cleaned = String::with_capacity(expressions.len());
    for line in expressions.lines() {
        cleaned.push_str(line.split("//").next().unwrap_or(""));
        cleaned.push('\n');
    }
    cleaned
        .split(';')
        .filter_map(|statement| {
            let (name, rhs) = statement.split_once('=')?;
            let (name, rhs) = (name.trim(), rhs.trim());
            let identifier = name
                .chars()
                .next()
                .is_some_and(|c| c.is_ascii_alphabetic() || c == '_' || c == '$')
                && name.chars().all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$');
            (identifier && !rhs.is_empty()).then(|| (name.to_string(), rhs.to_string()))
        })
        .collect()
}

/// The template's own value for `name` (its defining expression's source),
/// what the insert prompt starts the field from.
pub fn default_value(template: &serde_json::Value, input: &TemplateInput) -> String {
    let expressions = expressions_of(template);
    if let Some(first) = input.choices.first() {
        return assignments(expressions)
            .into_iter()
            .find(|(name, _)| *name == input.name)
            .map(|(_, rhs)| rhs)
            .filter(|rhs| input.choices.iter().any(|choice| choice.trim() == rhs))
            .unwrap_or_else(|| first.clone());
    }
    assignments(expressions)
        .into_iter()
        .find(|(name, _)| *name == input.name)
        .map(|(_, rhs)| rhs)
        .unwrap_or_default()
}

fn expressions_of(document: &serde_json::Value) -> &str {
    document
        .get("expressions")
        .and_then(serde_json::Value::as_str)
        .unwrap_or_default()
}

/// Why `text` is not an acceptable value for `input`, if it is not: it must
/// evaluate (against the template's own expressions, so `width * 2` works) to
/// a finite number inside the limits, and be one of the choices when there
/// are choices.
pub fn value_problem(template: &serde_json::Value, input: &TemplateInput, text: &str) -> Option<String> {
    let text = text.trim();
    let shown = input.shown();
    if text.is_empty() {
        return Some(format!("{shown}: enter a value"));
    }
    if !input.choices.is_empty() && !input.choices.iter().any(|choice| choice.trim() == text) {
        return Some(format!("{shown}: choose one of {}", input.choices.join(", ")));
    }
    let configurator = template.get("configurator").cloned().unwrap_or(serde_json::Value::Null);
    let mut values = BTreeMap::new();
    values.insert(input.name.clone(), text.to_string());
    let source = rewrite_expressions(expressions_of(template), &values);
    let value = match brep_render::brep_kernel::eval_expression(&source, &configurator, &input.name) {
        Ok(value) if value.is_finite() => value,
        Ok(value) => return Some(format!("{shown}: evaluates to {value}")),
        Err(error) => return Some(format!("{shown}: {error}")),
    };
    if let Some(min) = input.min.filter(|min| value < *min) {
        return Some(format!("{shown}: {value} is below the minimum {min}"));
    }
    if let Some(max) = input.max.filter(|max| value > *max) {
        return Some(format!("{shown}: {value} is above the maximum {max}"));
    }
    None
}

/// The specialised copy of `template` (file `template_file`) with `values`
/// (input name -> source text) baked into its expressions: a normal part that
/// keeps its expressions, drops the input marks and the class, and records
/// its template. Every value is checked first; the first problem is the error.
pub fn spin_out(
    template: &serde_json::Value,
    template_file: &str,
    values: &BTreeMap<String, String>,
) -> Result<serde_json::Value, String> {
    let inputs = inputs_of(template);
    for input in &inputs {
        let text = values.get(&input.name).map(String::as_str).unwrap_or("");
        if let Some(problem) = value_problem(template, input, text) {
            return Err(problem);
        }
    }
    let values: BTreeMap<String, String> = inputs
        .iter()
        .filter_map(|input| {
            values
                .get(&input.name)
                .map(|text| (input.name.clone(), text.trim().to_string()))
        })
        .collect();
    let mut copy = template.clone();
    let object = copy.as_object_mut().ok_or("the template is not a JSON object")?;
    object.insert(
        "expressions".into(),
        serde_json::Value::String(rewrite_expressions(expressions_of(template), &values)),
    );
    object.remove(TEMPLATE_INPUTS_KEY);
    object.remove(crate::document_class::DOCUMENT_CLASS_KEY);
    object.remove(crate::family_table::FAMILY_TABLE_KEY);
    object.remove(crate::family_table::FAMILY_SOURCE_KEY);
    let source = TemplateSource {
        template: template_file.to_string(),
        content_hash: template_hash(template),
    };
    object.insert(
        TEMPLATE_SOURCE_KEY.into(),
        serde_json::to_value(source).map_err(|e| e.to_string())?,
    );
    Ok(copy)
}

/// Why `name` cannot be the spun-out part's name, if it cannot: the same file
/// name rules as a family member's part number.
pub fn name_problem(name: &str) -> Option<String> {
    part_number_problem(name).map(|problem| problem.replace("part number", "name"))
}

