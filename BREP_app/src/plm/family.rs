//! Families and templates on a PLM store (plm-cad-integration-todo §3 S9).
//!
//! On the file system Generate writes `<part number>.nbrep` beside the family
//! ([`crate::family_table::generate_family`]). On a PLM the members are parts,
//! and the server writes them. The app still builds and checks each member
//! itself, then sends the documents:
//!
//! 1. Each row is checked here first, with the same checks the file lane runs:
//!    a part number that cannot be one, a number used twice, and values that do
//!    not evaluate. Those rows are reported and never sent.
//! 2. Each remaining member is built ([`member_document`]), stamped with the
//!    family's file name AS THE SERVER NAMES IT (`<family number>.fbrep`), and
//!    baked on an engine of its own ([`bake_document`]). A member whose features
//!    do not build is reported with the engine's sentence and never sent.
//! 3. The rest go to `POST /api/parts/:family/generate` as documents, with each
//!    row's revision label: on a PLM the table's revision column is the revision
//!    the member is written to. The server's report for each row (written,
//!    skipped because unchanged, or refused with its sentence) is the row's
//!    outcome.
//!
//! **What "skipped" proves.** The server skips a row when ITS stamp on the
//! member's revision matches: the same family part, the same `values_hash` of
//! the row, the same `history_hash` of the family's SAVED document, and a
//! document nobody has edited since. It does not read the `familySource` the
//! app wrote into the document. So a second unchanged Generate proves the
//! server's fingerprints are stable and that the saved family is the one the
//! app generated from. That the app's stamp and the server's stamp agree is a
//! separate fact, and [`stamp_agrees`] is how a caller checks it.
//!
//! The server generates from the family revision's SAVED document. A family
//! with unsaved edits would generate from something the user is not looking at,
//! so the caller refuses it (and offers to save first).

use std::collections::BTreeMap;

use serde::Deserialize;
use serde_json::{json, Value};

use super::bake::{bake_document, verdict};
use super::client::{PlmClient, PlmError};
use crate::family_table::{
    effective_row, member_document, part_number_problem, read_table, row_evaluation_problem, FamilyTable,
    GenerateReport, RowOutcome,
};

/// Why Generate refuses a family with unsaved edits on a PLM, and what to do.
pub const UNSAVED_FAMILY: &str = "the family has unsaved changes, and the server generates from its saved \
revision: save it first, or use Save and generate";

/// The part and revision ids of a PLM store key, `part/<part>/rev/<revision>`.
pub fn key_ids(key: &str) -> Option<(String, String)> {
    match crate::store::DocumentIdentity::parse_revision_key(key)? {
        crate::store::DocumentIdentity::Revision { part, revision } => Some((part, revision)),
        _ => None,
    }
}

/// The revision key a document name means on a PLM store: the key itself
/// (`part/<p>/rev/<r>`), or the explorer's spelling of it
/// (`/models/part/<p>/rev/<r>.nbrep`). Only meaningful when the session's store
/// IS the PLM (a browser folder can be called `part/7/rev`); callers check
/// `plm_client()` first.
pub fn revision_key(name: &str) -> Option<String> {
    let relative = name.trim_start_matches('/');
    let relative = relative.strip_prefix("models/").unwrap_or(relative);
    let relative = crate::document_class::strip_class_extension(relative);
    key_ids(relative).map(|_| relative.to_string())
}

/// The file name a server-written stamp gives a family: `<family number>.fbrep`.
/// The app must stamp its members the same way, or the server's family view
/// reads every member as coming from another family.
pub fn family_file_name(number: &str) -> String {
    format!("{}.fbrep", number.trim())
}

/// The part fields this module reads from `GET /api/parts/:id`.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct PartHead {
    pub id: String,
    pub number: String,
    pub name: String,
    pub document_class: String,
    pub member_part_type: String,
    pub category: String,
    pub revisions: Vec<RevisionHead>,
}

/// A revision's fields this module reads, including the server's own family
/// stamp.
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct RevisionHead {
    pub id: String,
    pub label: String,
    pub family: Option<ServerStamp>,
}

/// The server's stamp on a generated member revision (`FamilyStamp`).
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct ServerStamp {
    pub family_part: String,
    pub family_revision: String,
    pub values_hash: String,
    pub history_hash: String,
    pub generated_hash: String,
}

/// `GET /api/parts/:id`, the fields above.
pub async fn part_head(client: &PlmClient, part: &str) -> Result<PartHead, PlmError> {
    let response = client.call("GET", &crate::plm::identity::part_path(&part), None).await?;
    serde_json::from_slice(&response.body).map_err(|e| PlmError::Malformed(format!("/api/parts/{part}: {e}")))
}

/// One row as the server reports it (`RowResult`).
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct ServerRow {
    number: String,
    status: String,
    reason: String,
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct ServerReport {
    rows: Vec<ServerRow>,
}

/// Generate the family saved at store key `key`, whose document is
/// `family_json` (the caller has checked it is saved). Every table row gets an
/// outcome, in table order. `Err` only when the server could not be asked at
/// all or refused the whole request (not an author, no such family, the family
/// has no saved document); a row the server refuses is a `Failed` row.
pub async fn generate(client: &PlmClient, key: &str, family_json: &str) -> Result<GenerateReport, PlmError> {
    let (family_part, family_revision) =
        key_ids(key).ok_or_else(|| PlmError::Malformed(format!("`{key}` is not a PLM revision")))?;
    let family: Value = serde_json::from_str(family_json)
        .map_err(|e| PlmError::Malformed(format!("the family document does not parse: {e}")))?;
    let head = part_head(client, &family_part).await?;
    let family_file = family_file_name(&head.number);
    let table = read_table(family_json);

    let (mut outcomes, sent) = prepare(&family, &family_file, &table);
    if !sent.is_empty() {
        let body = json!({ "family_revision": family_revision, "rows": sent });
        let response = client
            .call("POST", &format!("{}/generate", crate::plm::identity::part_path(&family_part)), serde_json::to_vec(&body).ok())
            .await?;
        let report: ServerReport = serde_json::from_slice(&response.body)
            .map_err(|e| PlmError::Malformed(format!("generate: {e}")))?;
        let mut by_number: BTreeMap<String, ServerRow> =
            report.rows.into_iter().map(|row| (row.number.trim().to_lowercase(), row)).collect();
        for slot in outcomes.iter_mut() {
            let Slot::Sent(part_number) = slot else { continue };
            let row = by_number.remove(&part_number.trim().to_lowercase());
            *slot = Slot::Done(match row {
                Some(row) if row.status == "written" => RowOutcome::Written { part_number: part_number.clone() },
                Some(row) if row.status == "skipped" => {
                    RowOutcome::Skipped { part_number: part_number.clone(), reason: row.reason }
                }
                Some(row) => RowOutcome::Failed { part_number: part_number.clone(), reason: row.reason },
                None => RowOutcome::Failed {
                    part_number: part_number.clone(),
                    reason: "the server's report does not mention this row".into(),
                },
            });
        }
    }
    Ok(GenerateReport {
        rows: outcomes
            .into_iter()
            .map(|slot| match slot {
                Slot::Done(outcome) => outcome,
                Slot::Sent(part_number) => RowOutcome::Failed { part_number, reason: "not generated".into() },
            })
            .collect(),
    })
}

/// A row's place in the report while the server is asked.
enum Slot {
    Done(RowOutcome),
    Sent(String),
}

/// The local half of [`generate`]: every row's local outcome, or the member
/// to send for it.
fn prepare(family: &Value, family_file: &str, table: &FamilyTable) -> (Vec<Slot>, Vec<Value>) {
    let mut slots = Vec::new();
    let mut sent = Vec::new();
    let mut seen: BTreeMap<String, usize> = BTreeMap::new();
    for (index, raw) in table.rows.iter().enumerate() {
        let part_number = raw.part_number.trim().to_string();
        let fail = |reason: String| Slot::Done(RowOutcome::Failed { part_number: part_number.clone(), reason });
        if let Some(problem) = part_number_problem(&part_number) {
            slots.push(fail(problem));
            continue;
        }
        let key = part_number.to_lowercase();
        if let Some(first) = seen.get(&key) {
            slots.push(fail(format!("the same part number as row {}", first + 1)));
            continue;
        }
        seen.insert(key, index);
        let row = effective_row(table, raw);
        if let Some(problem) = row_evaluation_problem(family, &row) {
            slots.push(fail(problem));
            continue;
        }
        let member = member_document(family, family_file, &row);
        let revision = if row.revision.trim().is_empty() { "the first revision" } else { row.revision.trim() };
        match bake_document(&member.to_string()) {
            Err(why) => {
                slots.push(fail(format!("the member does not open: {why}")));
                continue;
            }
            Ok(bake) => {
                if let Err(sentence) = verdict(&part_number, revision, &bake) {
                    slots.push(fail(sentence));
                    continue;
                }
            }
        }
        sent.push(json!({
            "number": part_number,
            "revision": row.revision.trim(),
            "description": row.description.trim(),
            "values": row.values,
            "document": member,
        }));
        slots.push(Slot::Sent(part_number));
    }
    (slots, sent)
}

/// Whether the `familySource` the app wrote into a member's `document` agrees
/// with the server's own stamp on that revision: the same row fingerprint and
/// the same model fingerprint. This is the check that the two copies of the
/// hash functions (the app's and the server's, plm-cad-integration-todo §4)
/// still compute the same thing.
pub fn stamp_agrees(document: &Value, server: &ServerStamp) -> bool {
    crate::family_table::family_source(document)
        .is_some_and(|app| app.values_hash == server.values_hash && app.history_hash == server.history_hash)
}

// ---------------------------------------------------------------------------
// The member picker, the member part type, template spin-out
// ---------------------------------------------------------------------------

/// One member as the picker lists it, from `GET /api/parts/:id/family`.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Member {
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: String,
    pub hand_edited: bool,
    pub bake: Option<String>,
}

impl Member {
    /// The store key the member's document lives at, `None` for a row whose
    /// revision does not exist yet.
    pub fn key(&self) -> Option<String> {
        (!self.part_id.is_empty() && !self.revision_id.is_empty())
            .then(|| super::identity::document_key(&self.part_id, &self.revision_id))
    }

    /// Why this member cannot be placed, or `None` when it can.
    pub fn not_placeable(&self) -> Option<String> {
        if self.key().is_none() {
            return Some(format!("{} has not been generated yet", self.number));
        }
        match self.bake.as_deref() {
            Some("pending") | Some("claimed") => Some(format!("{} is waiting for its bake", self.number)),
            Some("failed") => Some(format!("{} failed its bake", self.number)),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
struct FamilyViewRow {
    part_number: String,
    member: Option<Member>,
}

/// What the family page says about a family: its member part type and its
/// members, one per table row (a row with no member yet has an empty one).
#[derive(Debug, Clone, Default, Deserialize)]
#[serde(default)]
pub struct FamilyView {
    pub number: String,
    pub member_part_type: String,
    pub member_part_type_mode: String,
    rows: Vec<FamilyViewRow>,
}

impl FamilyView {
    /// The members in table order, the picker's list.
    pub fn members(&self) -> Vec<Member> {
        self.rows
            .iter()
            .map(|row| row.member.clone().unwrap_or_else(|| Member { number: row.part_number.clone(), ..Member::default() }))
            .collect()
    }
}

/// `GET /api/parts/:family/family`.
pub async fn family_view(client: &PlmClient, family_part: &str) -> Result<FamilyView, PlmError> {
    let response = client.call("GET", &format!("{}/family", crate::plm::identity::part_path(&family_part)), None).await?;
    serde_json::from_slice(&response.body).map_err(|e| PlmError::Malformed(format!("family view: {e}")))
}

/// Set the part type new members are created in (`PATCH /api/parts/:id`).
pub async fn set_member_part_type(client: &PlmClient, family_part: &str, part_type: &str) -> Result<(), PlmError> {
    let body = serde_json::to_vec(&json!({ "member_part_type": part_type.trim() })).ok();
    client.call("PATCH", &crate::plm::identity::part_path(&family_part), body).await.map(|_| ())
}

/// A template spin-out as the server made it: the new part and the store key
/// of its first revision, which the caller places.
#[derive(Debug, Clone, PartialEq)]
pub struct SpunOut {
    pub part_id: String,
    pub number: String,
    pub key: String,
}

/// `POST /api/parts/:template/spin-out`: a new part made from the template with
/// `values` in place of its inputs. The server assembles the copy and queues it
/// for a bake; the refusal (a value out of range, a number the part type will
/// not take) comes back as the server wrote it.
pub async fn spin_out(
    client: &PlmClient,
    template_part: &str,
    name: &str,
    number: &str,
    values: &BTreeMap<String, String>,
) -> Result<SpunOut, PlmError> {
    let body = serde_json::to_vec(&json!({ "name": name, "number": number, "values": values })).ok();
    let response = client.call("POST", &format!("{}/spin-out", crate::plm::identity::part_path(&template_part)), body).await?;
    let part: PartHead =
        serde_json::from_slice(&response.body).map_err(|e| PlmError::Malformed(format!("spin-out: {e}")))?;
    let revision = part.revisions.first().ok_or_else(|| PlmError::Malformed("spin-out: the part has no revision".into()))?;
    Ok(SpunOut { key: super::identity::document_key(&part.id, &revision.id), part_id: part.id, number: part.number })
}

// ---------------------------------------------------------------------------
// The table editor's help: the category's keys, and the columns that will fail
// ---------------------------------------------------------------------------

/// One attribute of the member category's schema
/// (`GET /api/categories/:id/schema`), as the key picker offers it.
pub type Attribute = crate::panels::plm_parts::Attribute;

/// The schema keys a family's table does not have as a column yet: what the
/// key picker offers. Family parameters are a subset of the category's
/// attributes (round 1).
pub fn keys_to_offer<'a>(schema: &'a [Attribute], table: &FamilyTable) -> Vec<&'a Attribute> {
    schema
        .iter()
        .filter(|attribute| !table.columns.iter().any(|c| c.name.trim().eq_ignore_ascii_case(&attribute.key)))
        .collect()
}

/// The cells the server will refuse, as one sentence each. A column whose name
/// is an attribute of the member category becomes that part's catalog value,
/// and the server type-checks it the way it checks any write, so a cell that
/// holds an expression (`length * 2`) where a number is wanted fails its row.
/// Text attributes take any text and are not checked.
pub fn attribute_column_problems(schema: &[Attribute], table: &FamilyTable) -> Vec<String> {
    let mut problems = Vec::new();
    for column in &table.columns {
        let Some(attribute) = schema.iter().find(|a| a.key.eq_ignore_ascii_case(column.name.trim())) else {
            continue;
        };
        for row in &table.rows {
            let Some(text) = row.values.get(&column.name).map(|v| v.trim()).filter(|v| !v.is_empty()) else {
                continue;
            };
            let fits = match attribute.kind.as_str() {
                "number" => text.parse::<f64>().is_ok_and(f64::is_finite),
                "bool" => matches!(text.to_ascii_lowercase().as_str(), "true" | "yes" | "false" | "no"),
                "enum" => attribute.values.iter().any(|v| v.eq_ignore_ascii_case(text)),
                _ => true,
            };
            if !fits {
                let wanted = match attribute.kind.as_str() {
                    "number" => "a number".to_string(),
                    "bool" => "yes or no".to_string(),
                    _ => format!("one of {}", attribute.values.join(", ")),
                };
                problems.push(format!(
                    "{}: `{}` is the catalog attribute {}, which takes {wanted}, not `{text}`; the server will fail this row",
                    row.part_number, column.name, attribute.name
                ));
            }
        }
    }
    problems
}

// ---------------------------------------------------------------------------
// Hand-editing a member
// ---------------------------------------------------------------------------

/// What saving an edited member offers, when the document being saved was
/// generated by a family.
#[derive(Debug, Clone, PartialEq)]
pub struct HandEditPrompt {
    /// The sentence the prompt shows.
    pub message: String,
    /// "Save as a new part" is always offered.
    /// "Open the family" is offered only to a user who may edit the family.
    pub offer_open_family: bool,
    /// The family's file name, from the member's stamp.
    pub family: String,
}

/// The prompt for saving `document` after edits, or `None` when the document
/// is not a family member. `may_edit_family` is whether this user may change
/// the family (an author); the server's `hand_edited` on the revision is its
/// own reading of the same thing after the save.
pub fn hand_edit_prompt(document: &Value, may_edit_family: bool) -> Option<HandEditPrompt> {
    let stamp = crate::family_table::family_source(document)?;
    let mut message = format!(
        "{} was generated from the family {}. Saving an edit here makes it differ from its row, and the next \
         Generate overwrites it. Save it as a new part instead",
        stamp.part_number, stamp.family
    );
    message.push_str(if may_edit_family { ", or change the family and generate again." } else { "." });
    Some(HandEditPrompt { message, offer_open_family: may_edit_family, family: stamp.family })
}

