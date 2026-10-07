//! A family seed's table (`.fbrep`): one row per member, and the
//! Generate that writes each row out as an ordinary `.nbrep` part next
//! to the family file.
//!
//! The table lives in the family document itself, as the top-level
//! `familyTable` block. The history keeps unknown top-level keys verbatim
//! (it holds the document as a JSON value), so the block rides save, open and
//! every undo snapshot with no engine support beyond a setter.
//!
//! A row's values are EXPRESSION SOURCE, keyed by the expression name a
//! column drives: Generate replaces that name's definition in the member's
//! `expressions` source with the row's text. The history's `configurator`
//! object was the other candidate and does not fit: it carries numbers only
//! (a row may say `m * 2`), it reaches the model only where the author wrote
//! `configurator.x` in every expression, and it has no editor.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// The document key the table is stored under.
pub const FAMILY_TABLE_KEY: &str = "familyTable";

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyTable {
    pub columns: Vec<FamilyColumn>,
    pub rows: Vec<FamilyRow>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyColumn {
    /// The expression name this column drives (e.g. `length`).
    pub name: String,
    /// Optional display label; empty = show `name`.
    #[serde(default)]
    pub label: String,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyRow {
    /// Typed by the user; the member's file name (`<partNumber>.nbrep`).
    pub part_number: String,
    /// Kept for the PLM; ignored on the file system.
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub description: String,
    /// Column name -> expression source text for this row ("12", "m * 2").
    #[serde(default)]
    pub values: BTreeMap<String, String>,
}

/// The table a document carries, or an empty one when it has none (or the
/// block does not parse).
pub fn read_table(history_json: &str) -> FamilyTable {
    serde_json::from_str::<serde_json::Value>(history_json)
        .ok()
        .and_then(|document| document.get(FAMILY_TABLE_KEY).cloned())
        .and_then(|block| serde_json::from_value(block).ok())
        .unwrap_or_default()
}

/// `history_json` with its table replaced by `table`. Every other field is
/// kept as it was.
pub fn write_table(history_json: &str, table: &FamilyTable) -> Result<String, String> {
    let mut document: serde_json::Value =
        serde_json::from_str(history_json).map_err(|e| format!("family document: {e}"))?;
    let object = document
        .as_object_mut()
        .ok_or("family document: not a JSON object")?;
    let block = serde_json::to_value(table).map_err(|e| format!("family table: {e}"))?;
    object.insert(FAMILY_TABLE_KEY.into(), block);
    Ok(document.to_string())
}

/// The table the engine's document carries (empty when it has none). The
/// cheap read for a panel: no serialization of the whole document.
pub fn engine_table(engine: &brep_render::engine_state::EngineState) -> FamilyTable {
    engine
        .history
        .document_block(FAMILY_TABLE_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
        .unwrap_or_default()
}

/// Replace the engine document's table as a USER edit: one undo step, the
/// dirty dot lit. `coalesce` groups a typing run into one step (pass the same
/// key for every keystroke into one cell, e.g. `cell:3:length`); `None` is a
/// step of its own (add or remove a row or a column).
///
/// The targeted twin of [`write_table`]: it touches the one block instead of
/// re-parsing the whole document.
pub fn apply_table(
    engine: &mut brep_render::engine_state::EngineState,
    table: &FamilyTable,
    coalesce: Option<&str>,
) {
    let block = serde_json::to_value(table).unwrap_or(serde_json::Value::Null);
    engine
        .history
        .set_document_block(FAMILY_TABLE_KEY, Some(block), coalesce);
}

/// `row` as Generate uses it: only the values of the table's CURRENT columns,
/// and only non-empty ones. A cell left empty keeps the family's own
/// definition; a value left behind by a removed column is ignored.
pub fn effective_row(table: &FamilyTable, row: &FamilyRow) -> FamilyRow {
    let mut row = row.clone();
    row.values.retain(|name, value| {
        !value.trim().is_empty() && table.columns.iter().any(|column| column.name == *name)
    });
    row
}

/// The member's `expressions` source, exactly as Generate bakes it: every
/// statement that defines a name the row sets becomes `name = <row text>`,
/// in place, so later statements that use it see the row's value. A name the
/// source never defines is added at the end. Comments outside the replaced
/// statements are kept; the rest of the source is untouched byte for byte.
///
/// Pass the row through [`effective_row`] first, as Generate does.
pub fn member_expressions(expressions: &str, row: &FamilyRow) -> String {
    rewrite_expressions(expressions, &row.values)
}

/// [`member_expressions`] for any name -> source map: the ONE rewrite, which a
/// template's spin-out uses too.
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
    // Appended, not prepended: a cell may use the family's own variables
    // (`a * 3`), and the evaluator runs statements in order.
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

/// The byte spans of the source's statements: from the first code character
/// after the previous `;` up to (not including) the next `;` or the end.
/// `//` comments are skipped, so a comment ahead of a statement is not in its
/// span and survives a replacement.
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

/// The name a statement assigns (`name = rhs`), comments stripped.
fn assigned_name(statement: &str) -> Option<&str> {
    let eq = statement.find('=')?;
    let name = statement[..eq].trim();
    let mut chars = name.chars();
    let first = chars.next()?;
    ((first.is_ascii_alphabetic() || first == '_' || first == '$')
        && chars.all(|c| c.is_ascii_alphanumeric() || c == '_' || c == '$'))
    .then_some(name)
}

/// Why `part_number` cannot be a file name, if it cannot.
pub fn part_number_problem(part_number: &str) -> Option<String> {
    let name = part_number.trim();
    if name.is_empty() {
        return Some("no part number".into());
    }
    if name == "." || name == ".." {
        return Some(format!("'{name}' is not a file name"));
    }
    if let Some(bad) = name
        .chars()
        .find(|c| matches!(c, '/' | '\\' | '<' | '>' | ':' | '"' | '|' | '?' | '*') || c.is_control())
    {
        return Some(format!("'{name}' is not a valid file name (it contains '{bad}')"));
    }
    // Windows reserves these device names with or without an extension, in
    // any case; files move between machines, so they are refused everywhere.
    let stem = name.split('.').next().unwrap_or(name).trim_end().to_ascii_uppercase();
    let reserved = matches!(stem.as_str(), "CON" | "PRN" | "AUX" | "NUL")
        || ((stem.starts_with("COM") || stem.starts_with("LPT"))
            && stem.len() == 4
            && matches!(stem.as_bytes()[3], b'1'..=b'9'));
    if reserved {
        return Some(format!("'{name}' is a reserved device name on Windows"));
    }
    match crate::document_class::DocumentClass::of_name(name) {
        Some(class) if class != crate::document_class::DocumentClass::Normal => Some(format!(
            "'{name}' names a {} file, and a member is a normal part",
            class.label().to_ascii_lowercase()
        )),
        _ => None,
    }
}

/// The store identity of `part_number`'s member: `<part number>.nbrep` in the
/// family file's own folder. A path on native, a `/models/...` path in the
/// browser, a bare file name for a family known only by a bare name (which
/// both stores resolve into their models folder). `None` when the family has
/// never been saved (there is no folder to be beside) or the part number
/// cannot be a file name ([`part_number_problem`]).
pub fn member_identity(family_identity: Option<&str>, part_number: &str) -> Option<String> {
    let family = family_identity?;
    if part_number_problem(part_number).is_some() {
        return None;
    }
    let stem = crate::document_class::strip_class_extension(part_number.trim());
    Some(crate::store::sibling_identity(
        family,
        &crate::document_class::DocumentClass::Normal.file_name(stem),
    ))
}

/// The document key a generated member records its family under.
pub const FAMILY_SOURCE_KEY: &str = "familySource";

/// What produced a family member — stamped into the member document by
/// Generate, and compared by the next Generate to skip an unchanged row.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilySource {
    /// The family's file name (`bolt.fbrep`). Members sit beside their family,
    /// so the name is enough to find it again, and it survives the folder
    /// being moved or copied.
    pub family: String,
    /// The row's part number (the member's own file stem).
    pub part_number: String,
    /// [`row_hash`] of the row that produced the member.
    pub values_hash: String,
    /// [`history_hash`] of the family at generation.
    pub history_hash: String,
}

/// The family stamp a document carries, if it is a family member.
pub fn family_source(document: &serde_json::Value) -> Option<FamilySource> {
    document
        .get(FAMILY_SOURCE_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
}

/// The family stamp of the document the engine holds.
pub fn engine_family_source(engine: &brep_render::engine_state::EngineState) -> Option<FamilySource> {
    engine
        .history
        .document_block(FAMILY_SOURCE_KEY)
        .and_then(|block| serde_json::from_value(block.clone()).ok())
}

/// The top-level keys that are NOT the family's model: the table itself (a
/// row edit must not regenerate every member), the class, the view state the
/// file reopens in, and stamps.
const NOT_THE_MODEL: [&str; 6] = [
    FAMILY_TABLE_KEY,
    crate::document_class::DOCUMENT_CLASS_KEY,
    "workbench",
    FAMILY_SOURCE_KEY,
    crate::template::TEMPLATE_SOURCE_KEY,
    // The embedded preview `history_request_json` carries since the model
    // thumbnails landed: derived from the model, rendered anew on every save,
    // and absent from the copy the server hashes.
    "thumbnail",
];

/// A content hash of the family's model: the whole document less the keys in
/// [`NOT_THE_MODEL`]. Key order does not matter (`stable_json_hash`).
pub fn history_hash(family: &serde_json::Value) -> String {
    let mut model = family.clone();
    if let Some(object) = model.as_object_mut() {
        for key in NOT_THE_MODEL {
            object.remove(key);
        }
    }
    format!("{:016x}", brep_render::brep_kernel::stable_json_hash(&model))
}

/// A content hash of what a row puts into its member: its part number,
/// description and effective values.
pub fn row_hash(row: &FamilyRow) -> String {
    let value = serde_json::json!({
        "partNumber": row.part_number.trim(),
        "description": row.description.trim(),
        "values": row.values,
    });
    format!("{:016x}", brep_render::brep_kernel::stable_json_hash(&value))
}

/// What Generate did, one entry per table row, in table order.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GenerateReport {
    pub rows: Vec<RowOutcome>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum RowOutcome {
    Written { part_number: String },
    Skipped { part_number: String, reason: String },
    Failed { part_number: String, reason: String },
}

impl RowOutcome {
    pub fn part_number(&self) -> &str {
        match self {
            Self::Written { part_number }
            | Self::Skipped { part_number, .. }
            | Self::Failed { part_number, .. } => part_number,
        }
    }

    /// `written` / `skipped` / `failed`.
    pub fn status(&self) -> &'static str {
        match self {
            Self::Written { .. } => "written",
            Self::Skipped { .. } => "skipped",
            Self::Failed { .. } => "failed",
        }
    }

    /// The reason, for a skipped or failed row.
    pub fn reason(&self) -> Option<&str> {
        match self {
            Self::Written { .. } => None,
            Self::Skipped { reason, .. } | Self::Failed { reason, .. } => Some(reason),
        }
    }
}

impl GenerateReport {
    /// The report as JSON, for the automation blob and the log.
    pub fn to_json(&self) -> serde_json::Value {
        serde_json::Value::Array(
            self.rows
                .iter()
                .map(|row| {
                    serde_json::json!({
                        "partNumber": row.part_number(),
                        "status": row.status(),
                        "reason": row.reason(),
                    })
                })
                .collect(),
        )
    }

    /// One line: how many rows were written, skipped and failed.
    pub fn summary(&self) -> String {
        let count = |status: &str| self.rows.iter().filter(|row| row.status() == status).count();
        format!(
            "{} written, {} unchanged, {} failed",
            count("written"),
            count("skipped"),
            count("failed")
        )
    }
}

/// The member document for `row` (already an [`effective_row`]): the family's
/// model with the row's values baked into its expressions, the table and the
/// class dropped (a member is a normal part), the row's part number and
/// description in its part attributes, and the family stamp.
pub fn member_document(
    family: &serde_json::Value,
    family_file: &str,
    row: &FamilyRow,
) -> serde_json::Value {
    let expressions = family
        .get("expressions")
        .and_then(serde_json::Value::as_str)
        .unwrap_or_default();
    let mut member = family.clone();
    if let Some(object) = member.as_object_mut() {
        object.remove(FAMILY_TABLE_KEY);
        object.remove(crate::document_class::DOCUMENT_CLASS_KEY);
        object.remove(crate::template::TEMPLATE_INPUTS_KEY);
        object.remove(crate::template::TEMPLATE_SOURCE_KEY);
        object.insert(
            "expressions".into(),
            serde_json::Value::String(member_expressions(expressions, row)),
        );
        let attributes = object
            .entry(brep_render::engine_state::PART_ATTRIBUTES)
            .or_insert_with(|| serde_json::json!({}));
        if !attributes.is_object() {
            *attributes = serde_json::json!({});
        }
        attributes["Part_Number"] = serde_json::Value::String(row.part_number.trim().into());
        if !row.description.trim().is_empty() {
            attributes["Description"] = serde_json::Value::String(row.description.trim().into());
        }
        let stamp = FamilySource {
            family: family_file.to_string(),
            part_number: row.part_number.trim().to_string(),
            values_hash: row_hash(row),
            history_hash: history_hash(family),
        };
        object.insert(
            FAMILY_SOURCE_KEY.into(),
            serde_json::to_value(stamp).unwrap_or(serde_json::Value::Null),
        );
    }
    member
}

/// Why a row's model does not evaluate, if it does not: the member's
/// expressions must build, and every value the row sets must come out a
/// finite number. The geometry is NOT run here — a row that only breaks a
/// feature is written, and shows its feature error when opened.
pub fn row_evaluation_problem(family: &serde_json::Value, row: &FamilyRow) -> Option<String> {
    let expressions = family
        .get("expressions")
        .and_then(serde_json::Value::as_str)
        .unwrap_or_default();
    let configurator = family.get("configurator").cloned().unwrap_or(serde_json::Value::Null);
    let source = member_expressions(expressions, row);
    let env = match brep_render::brep_kernel::Env::build(&source, &configurator) {
        Ok(env) => env,
        Err(error) => {
            let set: Vec<String> =
                row.values.iter().map(|(name, value)| format!("{name} = {}", value.trim())).collect();
            return Some(format!("with {} the expressions do not evaluate: {error}", set.join(", ")));
        }
    };
    for name in row.values.keys() {
        match env.eval(name) {
            Ok(value) if value.is_finite() => {}
            Ok(value) => return Some(format!("{name} evaluates to {value}")),
            Err(error) => return Some(format!("{name} does not evaluate: {error}")),
        }
    }
    None
}

/// Generate every row of the family `family_json` as `<part number>.nbrep`
/// beside the family file `family_identity` (its store identity: a path on
/// native, a `/models/...` path in the browser; `None` for a family that has
/// never been saved, which has nowhere to write beside).
///
/// Every row is tried; one that cannot be written is reported and the rest
/// still generate. A row is SKIPPED when its member file already carries this
/// family's stamp with the same row hash and the same history hash — so an
/// unchanged row is not rewritten, and every row is rewritten after the
/// family's model changes. A member file that exists without that stamp (made
/// by hand, or by another family) is overwritten: that is the spec's rule.
/// Nothing here deletes anything: a row removed from the table leaves its
/// member where it is.
pub fn generate_family(
    store: &dyn crate::store::ModelStore,
    family_identity: Option<&str>,
    family_json: &str,
) -> GenerateReport {
    let table = read_table(family_json);
    let fail_all = |reason: &str| GenerateReport {
        rows: table
            .rows
            .iter()
            .map(|row| RowOutcome::Failed {
                part_number: row.part_number.clone(),
                reason: reason.to_string(),
            })
            .collect(),
    };
    let Some(identity) = family_identity else {
        return fail_all("save the family first: members are written beside the family file");
    };
    let family: serde_json::Value = match serde_json::from_str(family_json) {
        Ok(value) => value,
        Err(error) => return fail_all(&format!("the family document does not parse: {error}")),
    };
    let family_file = crate::store::file_name_of(identity);
    let history = history_hash(&family);
    let mut seen: BTreeMap<String, usize> = BTreeMap::new();
    let mut report = GenerateReport::default();
    for (index, raw) in table.rows.iter().enumerate() {
        let part_number = raw.part_number.clone();
        let fail = |reason: String| RowOutcome::Failed { part_number: part_number.clone(), reason };
        if let Some(problem) = part_number_problem(&part_number) {
            report.rows.push(fail(problem));
            continue;
        }
        // Case-insensitive: two rows that differ only in case are one file on
        // Windows and macOS, and the second would silently replace the first.
        let key = crate::document_class::strip_class_extension(part_number.trim()).to_lowercase();
        if let Some(first) = seen.get(&key) {
            report.rows.push(fail(format!("the same part number as row {}", first + 1)));
            continue;
        }
        seen.insert(key, index);
        let row = effective_row(&table, raw);
        if let Some(problem) = row_evaluation_problem(&family, &row) {
            report.rows.push(fail(problem));
            continue;
        }
        let Some(member_id) = member_identity(Some(identity), &part_number) else {
            report.rows.push(fail("no file name for this row".into()));
            continue;
        };
        let values = row_hash(&row);
        let unchanged = store
            .read(&member_id)
            .and_then(|text| serde_json::from_str::<serde_json::Value>(&text).ok())
            .and_then(|document| family_source(&document))
            .is_some_and(|stamp| {
                stamp.family == family_file
                    && stamp.values_hash == values
                    && stamp.history_hash == history
            });
        if unchanged {
            report.rows.push(RowOutcome::Skipped {
                part_number,
                reason: "unchanged since it was last generated".into(),
            });
            continue;
        }
        let member = member_document(&family, &family_file, &row);
        match store.write(&member_id, &member.to_string()) {
            Ok(()) => report.rows.push(RowOutcome::Written { part_number }),
            Err(error) => report.rows.push(fail(error)),
        }
    }
    report
}

