//! Family and template SEED documents, read and written the way the CAD app
//! reads and writes them.
//!
//! The server never links the CAD app (it does link the kernel). This module
//! reads and writes just the document fields families and templates use —
//! `documentClass`, `familyTable`, `familySource`, `templateInputs` and
//! `templateSource`. The pure functions over them it shares with the CAD app
//! through the kernel (`brep_kernel::family`):
//!
//! * the expression rewrite a row or a template input applies
//!   (`rewrite_expressions`), so a member the server assembles is the member
//!   the CAD app would write;
//! * the fingerprints (`history_hash`, `row_hash`; `template_hash` here), on
//!   the kernel's `stable_json_hash` and pinned by golden values, so a stamp
//!   the server writes and a stamp the CAD app writes compare equal.
//!
//! What this module can NOT do is evaluate an expression. The CAD app refuses
//! a row whose values do not evaluate before writing it; the server writes it
//! and marks it for a bake, and the headless CAD worker that takes the bake
//! reports the problem.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

// The table, its rows, the key names, the expression rewrite, the identifier
// rule and the fingerprints are the kernel's, the very code the CAD app runs
// (I2, 2026-11-01): one copy instead of a port kept in step by tests.
pub use brep_kernel::family::{
    effective_row, history_hash, is_identifier, rewrite_expressions, row_hash, FamilyColumn, FamilyRow, FamilyTable,
    DERIVED_FROM_KEY, DOCUMENT_CLASS_KEY, FAMILY_SOURCE_KEY, FAMILY_TABLE_KEY, TEMPLATE_SOURCE_KEY,
};

/// The document key a template's input marks are stored under.
pub const TEMPLATE_INPUTS_KEY: &str = "templateInputs";
/// The document key the part attributes (the BOM's source) live under.
pub const PART_ATTRIBUTES_KEY: &str = "partAttributes";

// ===========================================================================
// The family table
// ===========================================================================

/// The table a document carries, or an empty one when it has none or the
/// block does not parse — as the CAD app reads it.
pub fn read_table(document: &Value) -> FamilyTable {
    document
        .get(FAMILY_TABLE_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
        .unwrap_or_default()
}

/// Replace `document`'s table with `table`; every other field is kept.
pub fn write_table(document: &mut Value, table: &FamilyTable) -> Result<(), String> {
    let object = document
        .as_object_mut()
        .ok_or("the family document is not a JSON object")?;
    let block = serde_json::to_value(table).map_err(|e| e.to_string())?;
    object.insert(FAMILY_TABLE_KEY.into(), block);
    Ok(())
}

/// The class a document says it is: `family`, `template`, or `None` for a
/// normal part (which writes no field).
pub fn document_class(document: &Value) -> Option<&str> {
    match document.get(DOCUMENT_CLASS_KEY).and_then(Value::as_str) {
        Some(class @ ("family" | "template")) => Some(class),
        _ => None,
    }
}

/// The `name = rhs` assignments of an expressions source, in order
/// (`BREP_app/src/template.rs` `assignments`).
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
            (is_identifier(name) && !rhs.is_empty()).then(|| (name.to_string(), rhs.to_string()))
        })
        .collect()
}

fn expressions_of(document: &Value) -> &str {
    document.get("expressions").and_then(Value::as_str).unwrap_or_default()
}

// ===========================================================================
// Fingerprints
// ===========================================================================

/// The kernel's `stable_json_hash` itself (a sorted-key walk, string leaves
/// verbatim, through std's `DefaultHasher`), so a stamp the server writes and
/// a stamp the CAD app writes compare equal. The golden values in the tests
/// below were computed by the kernel and fail loudly if it ever changes.
pub use brep_kernel::stable_json_hash;

/// A content hash of a template document less its view state.
pub fn template_hash(template: &Value) -> String {
    let mut content = template.clone();
    if let Some(object) = content.as_object_mut() {
        object.remove("workbench");
    }
    format!("{:016x}", stable_json_hash(&content))
}

// ===========================================================================
// Building a member and a copy
// ===========================================================================

/// The CAD app's `familySource` stamp.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilySource {
    /// The family's file name (`bolt.fbrep`).
    pub family: String,
    pub part_number: String,
    pub values_hash: String,
    pub history_hash: String,
}

/// The member document for `row` (already an [`effective_row`]), exactly as
/// the CAD app's `member_document` builds it: the family's model with the
/// row's values in its expressions, the table, the class and any template
/// marks dropped, the part number and description in the part attributes, and
/// the `familySource` stamp naming `family_file`.
pub fn member_document(family: &Value, family_file: &str, row: &FamilyRow) -> Value {
    let expressions = expressions_of(family);
    let mut member = family.clone();
    if let Some(object) = member.as_object_mut() {
        object.remove(FAMILY_TABLE_KEY);
        object.remove(DOCUMENT_CLASS_KEY);
        object.remove(TEMPLATE_INPUTS_KEY);
        object.remove(TEMPLATE_SOURCE_KEY);
        object.remove(DERIVED_FROM_KEY);
        object.insert(
            "expressions".into(),
            Value::String(rewrite_expressions(expressions, &row.values)),
        );
        let attributes = object
            .entry(PART_ATTRIBUTES_KEY)
            .or_insert_with(|| serde_json::json!({}));
        if !attributes.is_object() {
            *attributes = serde_json::json!({});
        }
        attributes["Part_Number"] = Value::String(row.part_number.trim().into());
        if !row.description.trim().is_empty() {
            attributes["Description"] = Value::String(row.description.trim().into());
        }
        let stamp = FamilySource {
            family: family_file.to_string(),
            part_number: row.part_number.trim().to_string(),
            values_hash: row_hash(row),
            history_hash: history_hash(family),
        };
        object.insert(
            FAMILY_SOURCE_KEY.into(),
            serde_json::to_value(stamp).unwrap_or(Value::Null),
        );
    }
    member
}

/// One marked template input (`BREP_app/src/template.rs` `TemplateInput`).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TemplateInput {
    pub name: String,
    #[serde(default)]
    pub label: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub min: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max: Option<f64>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub choices: Vec<String>,
}

impl TemplateInput {
    pub fn shown(&self) -> &str {
        if self.label.trim().is_empty() {
            &self.name
        } else {
            &self.label
        }
    }
}

/// The inputs a template marks (empty when none or unreadable).
pub fn inputs_of(document: &Value) -> Vec<TemplateInput> {
    document
        .get(TEMPLATE_INPUTS_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
        .unwrap_or_default()
}

/// The template's own value for `input`, what a form starts from.
pub fn default_value(template: &Value, input: &TemplateInput) -> String {
    let own = assignments(expressions_of(template))
        .into_iter()
        .find(|(name, _)| *name == input.name)
        .map(|(_, rhs)| rhs);
    if let Some(first) = input.choices.first() {
        return own
            .filter(|rhs| input.choices.iter().any(|choice| choice.trim() == rhs))
            .unwrap_or_else(|| first.clone());
    }
    own.unwrap_or_default()
}

/// Why `text` is not an acceptable value for `input`, as far as the server
/// can tell without an evaluator: it must be non-empty, one of the choices
/// when there are choices, and — when it is a plain number — inside the
/// limits. An expression (`width / 2`) passes here; the bake evaluates it.
pub fn value_problem(input: &TemplateInput, text: &str) -> Option<String> {
    let text = text.trim();
    let shown = input.shown();
    if text.is_empty() {
        return Some(format!("{shown}: enter a value"));
    }
    if !input.choices.is_empty() && !input.choices.iter().any(|choice| choice.trim() == text) {
        return Some(format!("{shown}: choose one of {}", input.choices.join(", ")));
    }
    if let Ok(value) = text.parse::<f64>() {
        if !value.is_finite() {
            return Some(format!("{shown}: {text} is not a finite number"));
        }
        if let Some(min) = input.min.filter(|min| value < *min) {
            return Some(format!("{shown}: {value} is below the minimum {min}"));
        }
        if let Some(max) = input.max.filter(|max| value > *max) {
            return Some(format!("{shown}: {value} is above the maximum {max}"));
        }
    }
    None
}

/// The specialised copy of `template` (file `template_file`), as the CAD
/// app's `spin_out` builds it: a normal part that keeps its expressions with
/// `values` in place of the template's, drops the input marks, the class and
/// any family blocks, and records its template in `templateSource`.
///
/// An input left out takes the template's own value. A value for a name that
/// is not an input is refused, as is any value [`value_problem`] refuses.
/// Returns the copy and the values it was made with.
pub fn spin_out(
    template: &Value,
    template_file: &str,
    values: &BTreeMap<String, String>,
) -> Result<(Value, BTreeMap<String, String>), String> {
    let inputs = inputs_of(template);
    if let Some(stray) = values.keys().find(|name| !inputs.iter().any(|i| &i.name == *name)) {
        return Err(format!("'{stray}' is not an input of this template"));
    }
    let mut used = BTreeMap::new();
    for input in &inputs {
        let given = values.get(&input.name).map(|v| v.trim()).unwrap_or("");
        let text = if given.is_empty() { default_value(template, input) } else { given.to_string() };
        if let Some(problem) = value_problem(input, &text) {
            return Err(problem);
        }
        used.insert(input.name.clone(), text.trim().to_string());
    }
    let mut copy = template.clone();
    let object = copy.as_object_mut().ok_or("the template is not a JSON object")?;
    object.insert(
        "expressions".into(),
        Value::String(rewrite_expressions(expressions_of(template), &used)),
    );
    object.remove(TEMPLATE_INPUTS_KEY);
    object.remove(DOCUMENT_CLASS_KEY);
    object.remove(FAMILY_TABLE_KEY);
    object.remove(FAMILY_SOURCE_KEY);
    object.remove(DERIVED_FROM_KEY);
    object.insert(
        TEMPLATE_SOURCE_KEY.into(),
        serde_json::json!({ "template": template_file, "contentHash": template_hash(template) }),
    );
    Ok((copy, used))
}

// ===========================================================================
// CSV rows for a family table
// ===========================================================================

/// Rows read from a CSV (or tab-separated) paste, for a family table.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CsvRows {
    /// The expression columns the header names, in order.
    pub columns: Vec<String>,
    /// Whether the header had a revision / description column. A column the
    /// CSV lacks leaves that field of an existing row alone.
    pub has_revision: bool,
    pub has_description: bool,
    pub rows: Vec<FamilyRow>,
}

/// Parse a family table CSV. The header names the columns: `part number`
/// (also `part_number`, `partnumber`, `number`) is required; `revision` (or
/// `rev`) and `description` are optional; every other header is an
/// expression name. Tab-separated text — a paste from a spreadsheet — is read
/// the same way. Quoted fields may hold the separator, quotes (`""`) and line
/// breaks. Blank lines are skipped. The reader is the kernel's
/// `text::delimited`, strict: an unclosed quote, or text after a closing
/// quote, is refused naming its line.
pub fn parse_csv(text: &str) -> Result<CsvRows, String> {
    use brep_kernel::text::delimited;
    let text = delimited::strip_bom(text);
    let records = delimited::parse(text, delimited::sniff_separator(text), delimited::Strictness::Strict)?;
    let mut records = records
        .into_iter()
        .filter(|record| !record.is_blank())
        .map(|record| (record.line, record.fields));
    let Some((_, header)) = records.next() else {
        return Err("the CSV is empty — its first line names the columns".into());
    };

    enum Col {
        Number,
        Revision,
        Description,
        Value(String),
    }
    let mut columns = Vec::new();
    let mut out = CsvRows::default();
    for raw in &header {
        let name = raw.trim();
        let key = name.to_ascii_lowercase().replace(['_', ' '], "");
        let col = match key.as_str() {
            "partnumber" | "number" => Col::Number,
            "revision" | "rev" => Col::Revision,
            "description" => Col::Description,
            _ if is_identifier(name) => Col::Value(name.to_string()),
            _ => return Err(format!("column '{name}' is not an expression name (letters, digits, _ and $, not starting with a digit)")),
        };
        let duplicate = match &col {
            Col::Number => columns.iter().any(|c| matches!(c, Col::Number)),
            Col::Revision => out.has_revision,
            Col::Description => out.has_description,
            Col::Value(n) => out.columns.contains(n),
        };
        if duplicate {
            return Err(format!("column '{name}' appears twice"));
        }
        match &col {
            Col::Revision => out.has_revision = true,
            Col::Description => out.has_description = true,
            Col::Value(n) => out.columns.push(n.clone()),
            Col::Number => {}
        }
        columns.push(col);
    }
    if !columns.iter().any(|c| matches!(c, Col::Number)) {
        return Err("the CSV needs a 'part number' column".into());
    }
    let mut seen: BTreeMap<String, usize> = BTreeMap::new();
    for (line, fields) in records {
        if fields.len() > columns.len() && fields[columns.len()..].iter().any(|f| !f.trim().is_empty()) {
            return Err(format!("line {line} has more fields than the header has columns"));
        }
        let mut row = FamilyRow::default();
        for (col, field) in columns.iter().zip(fields.iter().chain(std::iter::repeat(&String::new()))) {
            let field = field.trim();
            match col {
                Col::Number => row.part_number = field.to_string(),
                Col::Revision => row.revision = field.to_string(),
                Col::Description => row.description = field.to_string(),
                Col::Value(name) => {
                    if !field.is_empty() {
                        row.values.insert(name.clone(), field.to_string());
                    }
                }
            }
        }
        if row.part_number.is_empty() {
            return Err(format!("line {line} has no part number"));
        }
        let key = row.part_number.to_lowercase();
        if let Some(first) = seen.get(&key) {
            return Err(format!("line {line} repeats part number {} from line {first}", row.part_number));
        }
        seen.insert(key, line);
        out.rows.push(row);
    }
    Ok(out)
}

