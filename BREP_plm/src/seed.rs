//! Family and template SEED documents, read and written the way the CAD app
//! reads and writes them (§3.2, round 8).
//!
//! The server never links the kernel or the CAD app. This module is a small
//! mirror of just the document fields families and templates use —
//! `documentClass`, `familyTable`, `familySource`, `templateInputs` and
//! `templateSource` — and of the few pure functions over them:
//!
//! * the expression rewrite a row or a template input applies
//!   (`BREP_app/src/family_table.rs` `rewrite_expressions`), ported line for
//!   line so a member the server assembles is the member the CAD app would
//!   write;
//! * the fingerprints (`history_hash`, `row_hash`, `template_hash`), which use
//!   the kernel's `stable_json_hash`. That function is copied verbatim below
//!   and pinned by golden values computed with the kernel itself, so a stamp
//!   the server writes and a stamp the CAD app writes compare equal.
//!
//! What this module can NOT do is evaluate an expression. The CAD app refuses
//! a row whose values do not evaluate before writing it; the server writes it
//! and marks it for a bake, and the headless CAD worker that takes the bake
//! reports the problem.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// The document key the family table is stored under.
pub const FAMILY_TABLE_KEY: &str = "familyTable";
/// The document key a generated member records its family under.
pub const FAMILY_SOURCE_KEY: &str = "familySource";
/// The document key a template's input marks are stored under.
pub const TEMPLATE_INPUTS_KEY: &str = "templateInputs";
/// The document key a spun-out copy records its template under.
pub const TEMPLATE_SOURCE_KEY: &str = "templateSource";
/// The document key the class is stored under. A normal part writes none.
pub const DOCUMENT_CLASS_KEY: &str = "documentClass";
/// The document key the part attributes (the BOM's source) live under.
pub const PART_ATTRIBUTES_KEY: &str = "partAttributes";

// ===========================================================================
// The family table
// ===========================================================================

/// A family's table: one row per member.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyTable {
    #[serde(default)]
    pub columns: Vec<FamilyColumn>,
    #[serde(default)]
    pub rows: Vec<FamilyRow>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyColumn {
    /// The expression name this column drives (`length`).
    pub name: String,
    /// Optional display label; empty shows `name`.
    #[serde(default)]
    pub label: String,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyRow {
    /// Typed by the user: the member's part number.
    pub part_number: String,
    /// The member revision this row writes. The CAD app on the file system
    /// ignores it; the PLM honours it.
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub description: String,
    /// Column name -> expression source text ("12", "m * 2").
    #[serde(default)]
    pub values: BTreeMap<String, String>,
}

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

/// `row` as Generate uses it: only the values of the table's CURRENT columns,
/// and only non-empty ones. An empty cell keeps the family's own definition.
pub fn effective_row(table: &FamilyTable, row: &FamilyRow) -> FamilyRow {
    let mut row = row.clone();
    row.values
        .retain(|name, value| !value.trim().is_empty() && table.columns.iter().any(|c| c.name == *name));
    row
}

/// The class a document says it is: `family`, `template`, or `None` for a
/// normal part (which writes no field).
pub fn document_class(document: &Value) -> Option<&str> {
    match document.get(DOCUMENT_CLASS_KEY).and_then(Value::as_str) {
        Some(class @ ("family" | "template")) => Some(class),
        _ => None,
    }
}

// ===========================================================================
// The expression rewrite (BREP_app/src/family_table.rs, ported)
// ===========================================================================

/// `expressions` with every statement that defines a name in `values`
/// replaced, in place, by `name = <value>`; a name the source never defines
/// is appended. Comments outside the replaced statements are kept and the
/// rest of the source is untouched byte for byte.
pub fn rewrite_expressions(expressions: &str, values: &BTreeMap<String, String>) -> String {
    let mut out = expressions.to_string();
    let mut defined = std::collections::BTreeSet::new();
    // Splice back to front so earlier spans keep their offsets.
    for (start, end) in statement_spans(expressions).into_iter().rev() {
        let Some(name) = assigned_name(&expressions[start..end]) else {
            continue;
        };
        if let Some(value) = values.get(name) {
            out.replace_range(start..end, &format!("{name} = {}", value.trim()));
            defined.insert(name.to_string());
        }
    }
    // Appended, not prepended: a value may use the family's own variables,
    // and the evaluator runs statements in order.
    for (name, value) in values {
        if !defined.contains(name.as_str()) {
            if !out.is_empty() && !out.trim_end().ends_with(';') {
                out.push(';');
            }
            if !out.is_empty() && !out.ends_with('\n') {
                out.push('\n');
            }
            out.push_str(&format!("{name} = {};\n", value.trim()));
        }
    }
    out
}

/// The byte spans of the source's statements, `//` comments skipped.
fn statement_spans(source: &str) -> Vec<(usize, usize)> {
    let bytes = source.as_bytes();
    let mut spans = Vec::new();
    let mut start = None;
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'/' if bytes.get(i + 1) == Some(&b'/') => {
                while i < bytes.len() && bytes[i] != b'\n' {
                    i += 1;
                }
                continue;
            }
            b';' => {
                if let Some(s) = start.take() {
                    spans.push((s, i));
                }
            }
            c if !c.is_ascii_whitespace() && start.is_none() => start = Some(i),
            _ => {}
        }
        i += 1;
    }
    if let Some(s) = start {
        spans.push((s, source.len()));
    }
    spans
}

/// The name a statement assigns (`name = rhs`).
fn assigned_name(statement: &str) -> Option<&str> {
    let eq = statement.find('=')?;
    let name = statement[..eq].trim();
    is_identifier(name).then_some(name)
}

/// An expression name: a letter, `_` or `$`, then letters, digits, `_`, `$`.
pub fn is_identifier(name: &str) -> bool {
    let mut chars = name.chars();
    chars
        .next()
        .is_some_and(|c| c.is_ascii_alphabetic() || c == '_' || c == '$')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$')
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

/// The kernel's `stable_json_hash`, copied verbatim
/// (`BREP_kernel/src/feature_pipeline/parts_library.rs`): a sorted-key walk,
/// string leaves verbatim, through std's `DefaultHasher`. The repo pins one
/// toolchain, so this and the kernel's agree; the golden values in the tests
/// below were computed by the kernel and fail loudly if they ever stop.
pub fn stable_json_hash(value: &Value) -> u64 {
    use std::hash::{Hash, Hasher};
    fn walk(value: &Value, hasher: &mut impl Hasher) {
        match value {
            Value::Null => 0u8.hash(hasher),
            Value::Bool(flag) => {
                1u8.hash(hasher);
                flag.hash(hasher);
            }
            Value::Number(number) => {
                2u8.hash(hasher);
                number.to_string().hash(hasher);
            }
            Value::String(text) => {
                3u8.hash(hasher);
                text.hash(hasher);
            }
            Value::Array(items) => {
                4u8.hash(hasher);
                for item in items {
                    walk(item, hasher);
                }
            }
            Value::Object(map) => {
                5u8.hash(hasher);
                let mut keys: Vec<&String> = map.keys().collect();
                keys.sort();
                for key in keys {
                    key.hash(hasher);
                    walk(&map[key], hasher);
                }
            }
        }
    }
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    walk(value, &mut hasher);
    hasher.finish()
}

/// The keys that are not the family's MODEL: the table (a row edit must not
/// regenerate every member), the class, the view state, and stamps.
const NOT_THE_MODEL: [&str; 5] = [
    FAMILY_TABLE_KEY,
    DOCUMENT_CLASS_KEY,
    "workbench",
    FAMILY_SOURCE_KEY,
    TEMPLATE_SOURCE_KEY,
];

/// A content hash of the family's model: the whole document less the table,
/// the class, the view state and the stamps. Changing it regenerates every
/// member.
pub fn history_hash(family: &Value) -> String {
    let mut model = family.clone();
    if let Some(object) = model.as_object_mut() {
        for key in NOT_THE_MODEL {
            object.remove(key);
        }
    }
    format!("{:016x}", stable_json_hash(&model))
}

/// A content hash of what a row puts into its member.
pub fn row_hash(row: &FamilyRow) -> String {
    let value = serde_json::json!({
        "partNumber": row.part_number.trim(),
        "description": row.description.trim(),
        "values": row.values,
    });
    format!("{:016x}", stable_json_hash(&value))
}

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
/// breaks. Blank lines are skipped.
pub fn parse_csv(text: &str) -> Result<CsvRows, String> {
    let text = text.strip_prefix('\u{feff}').unwrap_or(text);
    let first_line = text.lines().find(|l| !l.trim().is_empty()).unwrap_or("");
    let separator = if first_line.contains('\t') && !first_line.contains(',') { '\t' } else { ',' };
    let records = split_records(text, separator)?;
    let mut records = records
        .into_iter()
        .filter(|(_, fields)| fields.iter().any(|f| !f.trim().is_empty()));
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

/// Split CSV text into records (with their 1-based starting line numbers).
fn split_records(text: &str, separator: char) -> Result<Vec<(usize, Vec<String>)>, String> {
    let mut records = Vec::new();
    let mut fields = Vec::new();
    let mut field = String::new();
    let mut quoted = false;
    let mut line = 1;
    let mut record_line = 1;
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if quoted {
            match c {
                '"' if chars.peek() == Some(&'"') => {
                    chars.next();
                    field.push('"');
                }
                '"' => quoted = false,
                '\n' => {
                    line += 1;
                    field.push('\n');
                }
                _ => field.push(c),
            }
            continue;
        }
        match c {
            '"' if field.trim().is_empty() => {
                field.clear();
                quoted = true;
            }
            c if c == separator => fields.push(std::mem::take(&mut field)),
            '\r' => {}
            '\n' => {
                fields.push(std::mem::take(&mut field));
                records.push((record_line, std::mem::take(&mut fields)));
                line += 1;
                record_line = line;
            }
            _ => field.push(c),
        }
    }
    if quoted {
        return Err(format!("line {record_line}: a quoted field is never closed"));
    }
    if !field.is_empty() || !fields.is_empty() {
        fields.push(field);
        records.push((record_line, fields));
    }
    Ok(records)
}

