//! A family seed's table and the pure functions over it, shared by the CAD app
//! (`BREP_app/src/family_table.rs`) and the PLM server (`BREP_plm/src/seed.rs`)
//! so a member the server assembles is the member the CAD app would write, and
//! a stamp either side writes compares equal.
//!
//! The table lives in the family document itself, as the top-level
//! `familyTable` block: one row per member. A row's values are EXPRESSION
//! SOURCE, keyed by the expression name a column drives: Generate replaces that
//! name's definition in the member's `expressions` source with the row's text
//! ([`rewrite_expressions`]).
//!
//! Pure and serde_json-only: no engine, no store, nothing native, so it builds
//! for wasm as the rest of the kernel does.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// The document key the family table is stored under.
pub const FAMILY_TABLE_KEY: &str = "familyTable";
/// The document key a generated member records its family under.
pub const FAMILY_SOURCE_KEY: &str = "familySource";
/// The document key a spun-out copy records its template under.
pub const TEMPLATE_SOURCE_KEY: &str = "templateSource";
/// The Save As lineage chain the CAD app writes (identities a part was saved
/// from). Not the model; members and template instances do not inherit it.
pub const DERIVED_FROM_KEY: &str = "derivedFrom";
/// The document key the class is stored under. A normal part writes none.
pub const DOCUMENT_CLASS_KEY: &str = "documentClass";

/// A family's table: one row per member. A block missing either list reads
/// as that list empty.
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
    /// The expression name this column drives (e.g. `length`).
    pub name: String,
    /// Optional display label; empty = show `name`.
    #[serde(default)]
    pub label: String,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FamilyRow {
    /// Typed by the user: the member's part number (on the file system, its
    /// file name `<partNumber>.nbrep`).
    pub part_number: String,
    /// The member revision this row writes. The CAD app on the file system
    /// ignores it; the PLM honours it.
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub description: String,
    /// Column name -> expression source text for this row ("12", "m * 2").
    #[serde(default)]
    pub values: BTreeMap<String, String>,
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

/// `expressions` with every statement that defines a name in `values`
/// replaced, in place, by `name = <value>`, so later statements that use it
/// see the new value; a name the source never defines is appended. Comments
/// outside the replaced statements are kept and the rest of the source is
/// untouched byte for byte. The ONE rewrite: a family row and a template's
/// spin-out both use it.
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

/// The top-level keys that are NOT the family's model: the table itself (a
/// row edit must not regenerate every member), the class, the view state the
/// file reopens in, and stamps.
pub const NOT_THE_MODEL: [&str; 7] = [
    FAMILY_TABLE_KEY,
    // The Save As lineage chain: where the document came from, not its model.
    DERIVED_FROM_KEY,
    DOCUMENT_CLASS_KEY,
    "workbench",
    FAMILY_SOURCE_KEY,
    TEMPLATE_SOURCE_KEY,
    // The embedded preview a CAD save writes into the document: derived from
    // the model and rendered anew on every save.
    "thumbnail",
];

/// A content hash of the family's model: the whole document less the keys in
/// [`NOT_THE_MODEL`]. Key order does not matter ([`stable_json_hash`]).
/// Changing it regenerates every member.
///
/// [`stable_json_hash`]: crate::stable_json_hash
pub fn history_hash(family: &Value) -> String {
    let mut model = family.clone();
    if let Some(object) = model.as_object_mut() {
        for key in NOT_THE_MODEL {
            object.remove(key);
        }
    }
    format!("{:016x}", super::parts_library::stable_json_hash(&model))
}

/// A content hash of what a row puts into its member: its part number,
/// description and effective values (not its revision).
pub fn row_hash(row: &FamilyRow) -> String {
    let value = serde_json::json!({
        "partNumber": row.part_number.trim(),
        "description": row.description.trim(),
        "values": row.values,
    });
    format!("{:016x}", super::parts_library::stable_json_hash(&value))
}

