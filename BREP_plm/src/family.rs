//! Families, templates and the bake queue on the server (§3.2, round
//! 8).
//!
//! A family's TABLE lives inside the family's CAD document (`familyTable`),
//! so it is revisioned with the family; the server reads it from a family
//! revision's document ([`crate::seed`]). Generate writes each row to the
//! part and the revision the row names, by the §3.2 rules:
//!
//! * the part is created when no part carries the row's number, in the
//!   family's MEMBER PART TYPE (and by that type's numbering mode);
//! * the revision is created when the part has none with the row's label;
//! * an existing revision is overwritten when it is writeable — in work, and
//!   not checked out by someone else — including one made outside the family;
//! * a row FAILS when its revision exists and is not writeable (and, as in the
//!   CAD app, when its number or document cannot be used at all). Every other
//!   row is still generated, and the report names each row's outcome;
//! * a row whose member already carries this family's stamp with the same
//!   row fingerprint, the same model fingerprint, and a document nobody has
//!   edited since, is SKIPPED;
//! * nothing is released, and nothing is deleted: a member whose row is gone
//!   is an orphan and stays;
//! * a member's CATALOG VALUES come from its family: the family part's own
//!   values, overlaid by the row's values whose column is an attribute of the
//!   member's category (a family's parameters are a subset of its category's
//!   attributes). They are type-checked like any write, and a value that does
//!   not fit fails its row. Re-generating refreshes those keys, even for a row
//!   whose document is unchanged; values set by hand under other keys are left
//!   alone.
//!
//! A row arrives one of two ways. The CAD app sends it WITH the member
//! document it baked. The web page (or a bulk setup) sends none, and the
//! server assembles the member itself — the same document the CAD app would
//! write — and queues it for a BAKE: the server never links the kernel, so a
//! headless CAD worker takes the job, checks and rebuilds the document, and
//! writes it back. A template spin-out is assembled and queued the same way.
//!
//! Each row is its own writes. Hooks (a Script-mode number, the revision
//! label) run outside the store lock through the ordinary create paths; the
//! final write re-checks writeability inside the lock.

use std::collections::BTreeMap;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::auth;
use crate::catalog;
use crate::db::{check_number, now, write_atomic, Db, PartSpec, State};
use crate::model::{
    Bake, BakeStatus, DocumentClass, FamilyStamp, Origin, Part, Revision, TemplateStamp, User,
};
use crate::seed::{self, FamilyColumn, FamilyRow};
use crate::Error;

/// How long a worker's claim on a bake job holds before another worker may
/// take the job over. A crashed worker must not strand a job forever; a
/// person can also hand a job back at any time ([`Db::retry_bake`]).
pub const BAKE_LEASE: u64 = 15 * 60;

// ===========================================================================
// Shapes
// ===========================================================================

/// One row to generate. From the CAD app it carries the member `document`;
/// without one the server assembles it and queues a bake.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct GenerateRow {
    #[serde(alias = "part_number", alias = "partNumber")]
    pub number: String,
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub values: BTreeMap<String, String>,
    #[serde(default)]
    pub document: Option<Value>,
}

/// What to generate.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct GenerateRequest {
    /// The family revision (id or label) whose document is the seed. Empty is
    /// the newest revision.
    #[serde(default)]
    pub family_revision: String,
    /// The rows. Absent: every row of the family revision's own table.
    #[serde(default)]
    pub rows: Option<Vec<GenerateRow>>,
    /// Generate only these part numbers (from `rows` or the table). Empty is
    /// all of them.
    #[serde(default)]
    pub only: Vec<String>,
}

/// What Generate did with one row.
#[derive(Debug, Clone, Serialize)]
pub struct RowResult {
    pub number: String,
    /// The revision label written (or that would have been).
    pub revision: String,
    /// `written`, `skipped` or `failed`.
    pub status: &'static str,
    pub reason: String,
    pub part_id: Option<String>,
    pub revision_id: Option<String>,
    pub created_part: bool,
    pub created_revision: bool,
    /// The member was assembled by the server and waits for a bake.
    pub bake: bool,
}

/// What Generate did, one entry per row in order.
#[derive(Debug, Clone, Serialize)]
pub struct GenerateReport {
    pub family: String,
    pub family_revision: String,
    pub rows: Vec<RowResult>,
    pub written: usize,
    pub skipped: usize,
    pub failed: usize,
}

/// The family page: the table of one family revision and what each row's
/// member looks like now.
#[derive(Debug, Clone, Serialize)]
pub struct FamilyView {
    pub family_id: String,
    pub number: String,
    pub name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub revision_state: &'static str,
    /// Whether this family revision can take a table import.
    pub revision_writeable: bool,
    pub revisions: Vec<RevisionChoice>,
    pub has_document: bool,
    /// The part type new members are created in, and its numbering mode.
    pub member_part_type: String,
    pub member_part_type_mode: String,
    pub columns: Vec<FamilyColumn>,
    pub rows: Vec<RowView>,
    /// Members this family generated whose row is gone from this table.
    pub orphans: Vec<MemberView>,
}

#[derive(Debug, Clone, Serialize)]
pub struct RevisionChoice {
    pub id: String,
    pub label: String,
    pub lifecycle: &'static str,
}

/// One table row and its member.
#[derive(Debug, Clone, Serialize)]
pub struct RowView {
    pub part_number: String,
    pub revision: String,
    pub description: String,
    pub values: BTreeMap<String, String>,
    /// `new` (Generate would create the part or the revision), `current`
    /// (generated from this row and this model, untouched), `stale` (the row
    /// or the model changed since), `unstamped` (the revision exists but this
    /// family did not write it; Generate would overwrite it), or `blocked`
    /// (Generate would write it, and cannot).
    pub status: &'static str,
    pub note: String,
    pub member: Option<MemberView>,
}

/// A member part, as the family page shows it.
#[derive(Debug, Clone, Serialize)]
pub struct MemberView {
    pub part_id: String,
    pub number: String,
    pub name: String,
    /// The revision the row targets (or, for an orphan, the newest one this
    /// family wrote). Empty when the row names a revision not yet created.
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: String,
    pub writeable: bool,
    pub locked_by: Option<String>,
    /// The label of the family revision it was generated from, when this
    /// family generated it.
    pub generated_from: Option<String>,
    pub stale_values: bool,
    pub stale_model: bool,
    /// Its document changed since Generate (or its bake) wrote it.
    pub hand_edited: bool,
    pub bake: Option<&'static str>,
    pub bake_error: String,
}

/// The result of a CSV import into a family table.
#[derive(Debug, Clone, Serialize)]
pub struct ImportReport {
    pub revision_id: String,
    pub revision_label: String,
    pub added: usize,
    pub updated: usize,
    pub columns_added: Vec<String>,
    pub rows: usize,
}

/// A template spin-out.
#[derive(Debug, Clone, Default, Deserialize)]
pub struct SpinOutRequest {
    pub name: String,
    #[serde(default)]
    pub number: String,
    /// The part type the copy is created in. Empty: the template's member
    /// part type, else the template's own.
    #[serde(default)]
    pub part_type: String,
    #[serde(default)]
    pub description: String,
    /// Input name -> expression source. An input left out takes the
    /// template's own value.
    #[serde(default)]
    pub values: BTreeMap<String, String>,
    /// The template revision (id or label). Empty: the released one, else the
    /// newest.
    #[serde(default)]
    pub template_revision: String,
    /// The creator's workspace folder the new part is linked into; empty is
    /// the top ([`crate::workspace::link_on_create`]).
    #[serde(default)]
    pub workspace_folder: String,
}

/// The template page's form: the inputs a template marks, with defaults.
#[derive(Debug, Clone, Serialize)]
pub struct TemplateView {
    pub template_id: String,
    pub number: String,
    pub revision_id: String,
    pub revision_label: String,
    pub has_document: bool,
    pub member_part_type: String,
    pub member_part_type_mode: String,
    pub inputs: Vec<TemplateInputView>,
    /// Parts spun out of this template, newest first.
    pub copies: Vec<CopyView>,
}

#[derive(Debug, Clone, Serialize)]
pub struct TemplateInputView {
    #[serde(flatten)]
    pub input: seed::TemplateInput,
    pub default: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct CopyView {
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub values: BTreeMap<String, String>,
    pub created_at: u64,
    pub bake: Option<&'static str>,
}

/// One job in the bake queue. Its id is the revision's id.
#[derive(Debug, Clone, Serialize)]
pub struct BakeJob {
    pub id: String,
    pub part_id: String,
    pub number: String,
    pub name: String,
    pub revision_label: String,
    pub lifecycle: &'static str,
    pub status: &'static str,
    pub reason: String,
    pub requested_by: String,
    pub requested_at: u64,
    pub claimed_by: Option<String>,
    pub claimed_at: Option<u64>,
    /// When another worker may take a claimed job over.
    pub lease_expires_at: Option<u64>,
    pub attempts: u32,
    pub error: String,
    pub finished_at: Option<u64>,
    pub document_key: String,
    /// `family`, `template`, or empty.
    pub source: &'static str,
    /// Who has the revision checked out. A checked-out revision is not
    /// handed to a worker until it is checked in.
    pub locked_by: Option<String>,
}

// ===========================================================================
// Reading a seed
// ===========================================================================

/// A family's or template's part, the chosen revision, and its document.
struct Seed {
    part: Part,
    revision: Revision,
    document: Option<Value>,
}

impl Db {
    /// The seed `key` (id or number) of class `class`, at `revision` (id or
    /// label; empty picks `default`).
    fn seed(
        &self,
        key: &str,
        class: DocumentClass,
        revision: &str,
        default: impl Fn(&Part) -> Option<&Revision>,
    ) -> Result<Seed, Error> {
        let (part, revision) = self.read(|state| {
            let part = state.part_by_id_or_number(key).ok_or_else(|| Error::not_found("part"))?;
            if part.document_class != class {
                return Err(Error::bad_request(format!(
                    "{} is a {} part, not a {}",
                    part.number,
                    part.document_class.as_str(),
                    class.as_str()
                )));
            }
            let revision = revision.trim();
            let chosen = if revision.is_empty() {
                default(part)
            } else {
                part.revision(revision).or_else(|| part.revision_by_label(revision))
            }
            .ok_or_else(|| Error::not_found("revision"))?;
            Ok((part.clone(), chosen.clone()))
        })?;
        let document = match self.read_document(&revision.document_key(&part.id))? {
            Some(text) => Some(
                serde_json::from_str::<Value>(&text)
                    .ok()
                    .filter(Value::is_object)
                    .ok_or_else(|| {
                        Error::conflict(format!(
                            "{} revision {} holds a document that is not a JSON object",
                            part.number, revision.label
                        ))
                    })?,
            ),
            None => None,
        };
        Ok(Seed { part, revision, document })
    }

    fn seed_document(seed: &Seed) -> Result<&Value, Error> {
        seed.document.as_ref().ok_or_else(|| {
            Error::conflict(format!(
                "{} revision {} has no document yet — save the {}'s model from the CAD app, or paste it into the revision's document",
                seed.part.number,
                seed.revision.label,
                seed.part.document_class.as_str()
            ))
        })
    }
}

/// The part type a seed's generated parts go into.
fn member_type(part: &Part) -> String {
    if part.member_part_type.is_empty() {
        part.part_type.clone()
    } else {
        part.member_part_type.clone()
    }
}

fn mode_of(state: &State, part_type: &str) -> String {
    state.part_type(part_type).map(|t| t.mode.kind().to_string()).unwrap_or_default()
}

fn user_name(state: &State, id: &str) -> String {
    state
        .user(id)
        .map(|u| if u.display_name.is_empty() { u.username.clone() } else { u.display_name.clone() })
        .unwrap_or_else(|| "another user".into())
}

/// Whether `user` may write `revision` without a checkout of their own:
/// it is in work, and nobody ELSE holds it.
fn writeable_by(revision: &Revision, user_id: &str) -> bool {
    revision.lifecycle.is_editable() && revision.lock.as_ref().is_none_or(|l| l.user_id == user_id)
}

/// Why `user` may not write `revision`, in words, if they may not.
fn not_writeable(state: &State, part: &Part, revision: &Revision, user_id: &str) -> Option<String> {
    if !revision.lifecycle.is_editable() {
        return Some(format!(
            "{} revision {} is {} and cannot be written — name a new revision in the row",
            part.number,
            revision.label,
            revision.lifecycle.as_str()
        ));
    }
    match &revision.lock {
        Some(lock) if lock.user_id != user_id => Some(format!(
            "{} revision {} is checked out by {}",
            part.number,
            revision.label,
            user_name(state, &lock.user_id)
        )),
        _ => None,
    }
}

fn bake_name(revision: &Revision) -> Option<&'static str> {
    revision.bake.as_ref().map(|b| b.status.as_str())
}

fn new_bake(user: &User, reason: String) -> Bake {
    Bake {
        status: BakeStatus::Pending,
        reason,
        requested_by: user.id.clone(),
        requested_at: now(),
        claimed_by: None,
        claimed_at: None,
        attempts: 0,
        error: String::new(),
        finished_by: None,
        finished_at: None,
    }
}

// ===========================================================================
// The family page
// ===========================================================================

impl Db {
    /// The family page for `key` at `revision` (empty: the newest).
    pub fn family_view(&self, viewer: &User, key: &str, revision: &str) -> Result<FamilyView, Error> {
        let seed = self.seed(key, DocumentClass::Family, revision, Part::latest)?;
        let table = seed.document.as_ref().map(seed::read_table).unwrap_or_default();
        let history = seed.document.as_ref().map(seed::history_hash);
        let family = &seed.part;
        self.read(|state| {
            let member_view = |part: &Part, revision: Option<&Revision>| -> MemberView {
                let stamp = revision.and_then(|r| r.family.as_ref()).filter(|s| s.family_part == family.id);
                MemberView {
                    part_id: part.id.clone(),
                    number: part.number.clone(),
                    name: part.name.clone(),
                    revision_id: revision.map(|r| r.id.clone()).unwrap_or_default(),
                    revision_label: revision.map(|r| r.label.clone()).unwrap_or_default(),
                    lifecycle: revision.map(|r| r.lifecycle.as_str().to_string()).unwrap_or_default(),
                    writeable: revision.is_some_and(|r| writeable_by(r, &viewer.id)),
                    locked_by: revision
                        .and_then(|r| r.lock.as_ref())
                        .map(|l| user_name(state, &l.user_id)),
                    generated_from: stamp.map(|s| {
                        family
                            .revision(&s.family_revision)
                            .map(|r| r.label.clone())
                            .unwrap_or_else(|| "a deleted revision".into())
                    }),
                    stale_values: false,
                    stale_model: stamp.is_some_and(|s| history.as_ref().is_some_and(|h| &s.history_hash != h)),
                    hand_edited: stamp
                        .zip(revision)
                        .is_some_and(|(s, r)| r.content_hash != s.generated_hash),
                    bake: revision.and_then(bake_name),
                    bake_error: revision
                        .and_then(|r| r.bake.as_ref())
                        .map(|b| b.error.clone())
                        .unwrap_or_default(),
                }
            };

            let mut rows = Vec::new();
            for raw in &table.rows {
                let row = seed::effective_row(&table, raw);
                let number = row.part_number.trim();
                let label = row.revision.trim();
                let part = state.part_with_number(number);
                let (status, note, member) = match part {
                    None => ("new", "Generate creates this part".to_string(), None),
                    Some(part) if part.document_class != DocumentClass::Normal => (
                        "blocked",
                        format!("{number} is a {} part — a member is a normal part", part.document_class.as_str()),
                        Some(member_view(part, part.latest())),
                    ),
                    Some(part) => {
                        let target = if label.is_empty() { part.latest() } else { part.revision_by_label(label) };
                        match target {
                            None => (
                                "new",
                                format!("Generate creates revision {label}"),
                                Some(member_view(part, None)),
                            ),
                            Some(revision) => {
                                let mut view = member_view(part, Some(revision));
                                let stamp = revision.family.as_ref().filter(|s| s.family_part == family.id);
                                view.stale_values = stamp.is_some_and(|s| s.values_hash != seed::row_hash(&row));
                                let (status, note) = match stamp {
                                    Some(_) if !view.stale_values && !view.stale_model && !view.hand_edited => {
                                        ("current", String::new())
                                    }
                                    Some(_) => {
                                        let mut why = Vec::new();
                                        if view.stale_values {
                                            why.push("the row changed");
                                        }
                                        if view.stale_model {
                                            why.push("the family's model changed");
                                        }
                                        if view.hand_edited {
                                            why.push("the member was edited by hand");
                                        }
                                        ("stale", format!("{} since it was generated", why.join(", ")))
                                    }
                                    None => (
                                        "unstamped",
                                        "this family did not write this revision — Generate overwrites it".into(),
                                    ),
                                };
                                match not_writeable(state, part, revision, &viewer.id).filter(|_| status != "current") {
                                    Some(why) => ("blocked", why, Some(view)),
                                    None => (status, note, Some(view)),
                                }
                            }
                        }
                    }
                };
                rows.push(RowView {
                    part_number: raw.part_number.clone(),
                    revision: raw.revision.clone(),
                    description: raw.description.clone(),
                    values: raw.values.clone(),
                    status,
                    note,
                    member,
                });
            }

            let in_table = |number: &str| {
                table.rows.iter().any(|r| r.part_number.trim().eq_ignore_ascii_case(number))
            };
            let orphans = state
                .parts
                .iter()
                .filter(|p| !in_table(&p.number))
                .filter_map(|p| {
                    let newest = p
                        .revisions
                        .iter()
                        .rev()
                        .find(|r| r.family.as_ref().is_some_and(|s| s.family_part == family.id))?;
                    Some(member_view(p, Some(newest)))
                })
                .collect();

            let current = family.revision(&seed.revision.id).unwrap_or(&seed.revision);
            let kind = member_type(family);
            Ok(FamilyView {
                family_id: family.id.clone(),
                number: family.number.clone(),
                name: family.name.clone(),
                revision_id: current.id.clone(),
                revision_label: current.label.clone(),
                revision_state: current.lifecycle.as_str(),
                revision_writeable: writeable_by(current, &viewer.id),
                revisions: family
                    .revisions
                    .iter()
                    .map(|r| RevisionChoice { id: r.id.clone(), label: r.label.clone(), lifecycle: r.lifecycle.as_str() })
                    .collect(),
                has_document: seed.document.is_some(),
                member_part_type_mode: mode_of(state, &kind),
                member_part_type: kind,
                columns: table.columns.clone(),
                rows,
                orphans,
            })
        })
    }
}

// ===========================================================================
// Generate
// ===========================================================================

/// What every row of one Generate shares.
struct Context<'a> {
    family: &'a Part,
    family_revision: &'a Revision,
    document: &'a Value,
    family_file: String,
    history: String,
    member_type: String,
}

impl Db {
    /// Generate a family's members (see the module header for the rules).
    pub fn generate(&self, user: &User, key: &str, request: GenerateRequest) -> Result<GenerateReport, Error> {
        let seed = self.seed(key, DocumentClass::Family, &request.family_revision, Part::latest)?;
        let document = Self::seed_document(&seed)?;
        let table = seed::read_table(document);
        let context = Context {
            family: &seed.part,
            family_revision: &seed.revision,
            document,
            family_file: format!("{}.fbrep", seed.part.number),
            history: seed::history_hash(document),
            member_type: member_type(&seed.part),
        };
        let rows: Vec<GenerateRow> = match request.rows {
            Some(rows) => rows,
            None => table
                .rows
                .iter()
                .map(|raw| {
                    let row = seed::effective_row(&table, raw);
                    GenerateRow {
                        number: row.part_number,
                        revision: row.revision,
                        description: row.description,
                        values: row.values,
                        document: None,
                    }
                })
                .collect(),
        };
        let only: Vec<String> = request.only.iter().map(|n| n.trim().to_lowercase()).collect();

        let mut report = GenerateReport {
            family: seed.part.number.clone(),
            family_revision: seed.revision.label.clone(),
            rows: Vec::new(),
            written: 0,
            skipped: 0,
            failed: 0,
        };
        let mut seen: BTreeMap<String, usize> = BTreeMap::new();
        for (index, row) in rows.into_iter().enumerate() {
            let key = row.number.trim().to_lowercase();
            if !only.is_empty() && !only.contains(&key) {
                continue;
            }
            let result = match seen.get(&key) {
                Some(first) => failed(&row, format!("the same part number as row {}", first + 1)),
                None => {
                    seen.insert(key, index);
                    self.generate_row(user, &context, row)
                }
            };
            match result.status {
                "written" => report.written += 1,
                "skipped" => report.skipped += 1,
                _ => report.failed += 1,
            }
            report.rows.push(result);
        }
        Ok(report)
    }

    fn generate_row(&self, user: &User, context: &Context, row: GenerateRow) -> RowResult {
        let number = row.number.trim().to_string();
        if let Err(error) = check_number(&number) {
            return failed(&row, error.message);
        }
        let member_row = FamilyRow {
            part_number: number.clone(),
            revision: row.revision.trim().to_string(),
            description: row.description.trim().to_string(),
            values: row
                .values
                .iter()
                .filter(|(_, v)| !v.trim().is_empty())
                .map(|(k, v)| (k.trim().to_string(), v.trim().to_string()))
                .collect(),
        };
        let values_hash = seed::row_hash(&member_row);

        // The document: the CAD app's, or assembled here and queued to bake.
        let assembled = row.document.is_none();
        let document = match &row.document {
            Some(document) => {
                if !document.is_object() {
                    return failed(&row, "the member document must be a JSON object".into());
                }
                if let Some(class) = seed::document_class(document) {
                    return failed(&row, format!("the document sent is a {class} — a member is a normal part"));
                }
                document.clone()
            }
            None => seed::member_document(context.document, &context.family_file, &member_row),
        };
        let body = document.to_string();

        // The part the row names, created if missing.
        let existing = self.read(|state| state.part_with_number(&number).cloned());
        let family_category = self.read(|state| {
            catalog::find(&state.categories, &context.family.category).map(|c| c.id.clone())
        });
        // The member's catalog values, checked BEFORE anything is created, so
        // a row whose value does not fit leaves nothing half-made.
        let member_category = match &existing {
            Some(part) => part.category.clone(),
            None => family_category.clone().unwrap_or_default(),
        };
        let values = match self.read(|state| member_values(state, context.family, &member_category, &member_row.values)) {
            Ok(values) => values,
            Err(reason) => return failed(&row, reason),
        };
        let mut result = RowResult {
            number: number.clone(),
            revision: member_row.revision.clone(),
            status: "failed",
            reason: String::new(),
            part_id: None,
            revision_id: None,
            created_part: false,
            created_revision: false,
            bake: false,
        };
        let part = match existing {
            Some(part) if part.document_class != DocumentClass::Normal => {
                result.reason = format!("{number} is a {} part — a member is a normal part", part.document_class.as_str());
                result.part_id = Some(part.id);
                return result;
            }
            Some(part) => part,
            None => {
                let category = family_category;
                let name = if member_row.description.is_empty() {
                    format!("{} {number}", context.family.name)
                } else {
                    member_row.description.clone()
                };
                let spec = PartSpec {
                    part_type: context.member_type.clone(),
                    number: number.clone(),
                    name,
                    description: member_row.description.clone(),
                    category: category.unwrap_or_default(),
                    document_class: DocumentClass::Normal,
                    label: member_row.revision.clone(),
                    tags: context.family.tags.clone(),
                    exact: true,
                    ..PartSpec::default()
                };
                match self.create_part_with(user, &spec) {
                    Ok(part) => {
                        result.created_part = true;
                        result.created_revision = true;
                        part
                    }
                    Err(error) => {
                        result.reason = error.message;
                        return result;
                    }
                }
            }
        };
        result.part_id = Some(part.id.clone());

        // The revision the row names, created if missing.
        let target = if result.created_part {
            part.revisions.first().cloned()
        } else if member_row.revision.is_empty() {
            part.latest().cloned()
        } else {
            part.revision_by_label(&member_row.revision).cloned()
        };
        let revision = match target {
            Some(revision) => revision,
            None => match self.create_revision_as(user, &part.id, &member_row.revision, true, Origin::Authored) {
                Ok(revision) => {
                    result.created_revision = true;
                    revision
                }
                Err(error) => {
                    result.reason = error.message;
                    return result;
                }
            },
        };
        result.revision = revision.label.clone();
        result.revision_id = Some(revision.id.clone());

        // Unchanged: this family's stamp, the same row, the same model, and a
        // document nobody has touched since. A baked document from the CAD
        // app still lands on a member waiting for its bake.
        if !result.created_revision {
            let unchanged = revision.family.as_ref().is_some_and(|stamp| {
                stamp.family_part == context.family.id
                    && stamp.values_hash == values_hash
                    && stamp.history_hash == context.history
                    && stamp.generated_hash == revision.content_hash
            });
            if unchanged && !(revision.needs_bake() && !assembled) {
                result.status = "skipped";
                result.reason = "unchanged since it was last generated".into();
                // The document is current; the catalog values may not be — the
                // family's own values can change without the row or the model.
                let stale = self.read(|state| {
                    state.part(&part.id).is_some_and(|p| {
                        let mut refreshed = p.attributes.clone();
                        catalog::apply_values(&state.categories, &p.category, &mut refreshed, &values).is_ok()
                            && refreshed != p.attributes
                    })
                });
                if stale {
                    match self.update_part(&part.id, &Value::Object(serde_json::Map::from_iter([(
                        "attributes".to_string(),
                        Value::Object(values),
                    )]))) {
                        Ok(_) => result.reason.push_str("; its catalog values were refreshed from the family"),
                        Err(error) => {
                            result.reason.push_str(&format!("; its catalog values were not refreshed: {}", error.message))
                        }
                    }
                }
                return result;
            }
        }

        let stamp = FamilyStamp {
            family_part: context.family.id.clone(),
            family_revision: context.family_revision.id.clone(),
            values: member_row.values.clone(),
            description: member_row.description.clone(),
            values_hash,
            history_hash: context.history.clone(),
            generated_hash: String::new(),
            generated_by: user.id.clone(),
            generated_at: now(),
        };
        let bake = assembled.then(|| {
            new_bake(
                user,
                format!("generated from family {} revision {}", context.family.number, context.family_revision.label),
            )
        });
        match self.write_generated(user, &part.id, &revision.id, &body, Written::Family(stamp), bake, &values) {
            Ok(()) => {
                result.status = "written";
                result.bake = assembled;
            }
            Err(error) => result.reason = error.message,
        }
        result
    }

    /// Write a generated document into a revision, with its stamp and its
    /// bake, in ONE locked write that re-checks the revision is writeable by
    /// `user` — someone may have released it or checked it out since the row
    /// was looked at.
    fn write_generated(
        &self,
        user: &User,
        part_id: &str,
        revision_id: &str,
        body: &str,
        written: Written,
        bake: Option<Bake>,
        values: &serde_json::Map<String, Value>,
    ) -> Result<(), Error> {
        let values = values.clone();
        let key = format!("part/{part_id}/rev/{revision_id}");
        let path = self.document_path(&key)?;
        let hash = auth::content_hash(body);
        let size = body.len() as u64;
        let user_id = user.id.clone();
        self.mutate(move |state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
            if let Some(why) = not_writeable(state, part, revision, &user_id) {
                return Err(Error::conflict(why));
            }
            // The catalog values first: they can still refuse (the tree may
            // have changed since the row was checked), and nothing may be on
            // disk when they do.
            if !values.is_empty() {
                let State { parts, categories, .. } = &mut *state;
                let part = parts.get_mut(&part_id).ok_or_else(|| Error::not_found("part"))?;
                let category = part.category.clone();
                catalog::apply_values(categories, &category, &mut part.attributes, &values)
                    .map_err(|e| Error::bad_request(format!("catalog value: {}", e.message)))?;
            }
            let revision = crate::db::find_revision_mut(state, part_id, revision_id)?.1;
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(Error::internal)?;
            }
            write_atomic(&path, body).map_err(Error::internal)?;
            revision.content_hash = hash.clone();
            revision.size = size;
            revision.modified_at = now();
            revision.origin = Origin::Generated;
            match written {
                Written::Family(mut stamp) => {
                    stamp.generated_hash = hash;
                    revision.family = Some(stamp);
                }
                Written::Template(stamp) => revision.template = Some(stamp),
            }
            match bake {
                Some(bake) => revision.bake = Some(bake),
                None => {
                    // The CAD app's own document: whatever bake was waiting
                    // is answered by it.
                    if revision.needs_bake() {
                        if let Some(old) = revision.bake.as_mut() {
                            old.status = BakeStatus::Done;
                            old.error = "superseded by a document from the CAD app".into();
                            old.finished_by = Some(user_id.clone());
                            old.finished_at = Some(now());
                        }
                    }
                }
            }
            crate::review::document_changed(state, part_id, revision_id);
            state.touch(key);
            Ok(())
        })
    }
}

/// The catalog values a member takes from its family (see the module header):
/// the family part's own values for keys the member's `category` defines,
/// overlaid by the row's values whose column names such a key. Checked here
/// on a scratch copy, so a value that does not fit is the row's failure.
fn member_values(
    state: &State,
    family: &Part,
    category: &str,
    row: &BTreeMap<String, String>,
) -> Result<serde_json::Map<String, Value>, String> {
    let mut out = serde_json::Map::new();
    let Some(found) = catalog::find(&state.categories, category) else {
        return Ok(out);
    };
    let schema = catalog::schema(&state.categories, &found.id).map_err(|e| e.message)?;
    for effective in &schema {
        if let Some(value) = family.attributes.get(&effective.def.key) {
            out.insert(effective.def.key.clone(), value.clone());
        }
    }
    for (column, value) in row {
        if let Some(effective) = schema.iter().find(|e| e.def.key.eq_ignore_ascii_case(column.trim())) {
            out.insert(effective.def.key.clone(), Value::String(value.trim().to_string()));
        }
    }
    let mut scratch = BTreeMap::new();
    catalog::apply_values(&state.categories, &found.id, &mut scratch, &out)
        .map_err(|e| format!("catalog value: {}", e.message))?;
    Ok(out)
}

/// Which stamp a generated write records.
enum Written {
    Family(FamilyStamp),
    Template(TemplateStamp),
}

fn failed(row: &GenerateRow, reason: String) -> RowResult {
    RowResult {
        number: row.number.trim().to_string(),
        revision: row.revision.trim().to_string(),
        status: "failed",
        reason,
        part_id: None,
        revision_id: None,
        created_part: false,
        created_revision: false,
        bake: false,
    }
}

// ===========================================================================
// CSV import into a family table
// ===========================================================================

impl Db {
    /// Merge CSV rows into the table of a family revision in work (`revision`,
    /// or else the newest one in work), writing the family's DOCUMENT — the
    /// table lives there. A row whose part number the table already has is
    /// updated (its revision and description when the CSV has those columns,
    /// and its value in every column the CSV has; an empty cell clears the
    /// value back to the family's own); a new number is appended. Expression
    /// columns the table lacks are added. Nothing is generated here.
    pub fn import_table(&self, user: &User, key: &str, revision: &str, csv: &str) -> Result<ImportReport, Error> {
        let parsed = seed::parse_csv(csv).map_err(Error::bad_request)?;
        let seed = self.seed(key, DocumentClass::Family, revision, |part| {
            part.revisions.iter().rev().find(|r| r.lifecycle.is_editable())
        })?;
        let mut document = Self::seed_document(&seed)?.clone();
        let mut table = seed::read_table(&document);
        let mut report = ImportReport {
            revision_id: seed.revision.id.clone(),
            revision_label: seed.revision.label.clone(),
            added: 0,
            updated: 0,
            columns_added: Vec::new(),
            rows: 0,
        };
        for column in &parsed.columns {
            if !table.columns.iter().any(|c| &c.name == column) {
                table.columns.push(FamilyColumn { name: column.clone(), label: String::new() });
                report.columns_added.push(column.clone());
            }
        }
        for incoming in parsed.rows {
            match table
                .rows
                .iter_mut()
                .find(|r| r.part_number.trim().eq_ignore_ascii_case(&incoming.part_number))
            {
                Some(existing) => {
                    if parsed.has_revision {
                        existing.revision = incoming.revision;
                    }
                    if parsed.has_description {
                        existing.description = incoming.description;
                    }
                    for column in &parsed.columns {
                        match incoming.values.get(column) {
                            Some(value) => existing.values.insert(column.clone(), value.clone()),
                            None => existing.values.remove(column),
                        };
                    }
                    report.updated += 1;
                }
                None => {
                    table.rows.push(incoming);
                    report.added += 1;
                }
            }
        }
        report.rows = table.rows.len();
        seed::write_table(&mut document, &table).map_err(Error::bad_request)?;
        self.write_seed(user, &seed.part.id, &seed.revision.id, &document.to_string())?;
        Ok(report)
    }

    /// Write a seed's own document (its table changed), re-checking inside
    /// the lock that `user` may write the revision.
    fn write_seed(&self, user: &User, part_id: &str, revision_id: &str, body: &str) -> Result<(), Error> {
        let key = format!("part/{part_id}/rev/{revision_id}");
        let path = self.document_path(&key)?;
        let hash = auth::content_hash(body);
        let size = body.len() as u64;
        let user_id = user.id.clone();
        self.mutate(move |state| {
            let part = state.part(part_id).ok_or_else(|| Error::not_found("part"))?;
            let revision = part.revision(revision_id).ok_or_else(|| Error::not_found("revision"))?;
            if let Some(why) = not_writeable(state, part, revision, &user_id) {
                return Err(Error::conflict(why.replace("name a new revision in the row", "start a new revision of the family")));
            }
            let revision = crate::db::find_revision_mut(state, part_id, revision_id)?.1;
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(Error::internal)?;
            }
            write_atomic(&path, body).map_err(Error::internal)?;
            revision.content_hash = hash;
            revision.size = size;
            revision.modified_at = now();
            crate::review::document_changed(state, part_id, revision_id);
            state.touch(key);
            Ok(())
        })
    }
}

// ===========================================================================
// Templates
// ===========================================================================

impl Db {
    /// The template page for `key` at `revision` (empty: the released one,
    /// else the newest).
    pub fn template_view(&self, key: &str, revision: &str) -> Result<TemplateView, Error> {
        let seed = self.seed(key, DocumentClass::Template, revision, |p| p.current_release().or(p.latest()))?;
        let inputs = seed
            .document
            .as_ref()
            .map(|document| {
                seed::inputs_of(document)
                    .into_iter()
                    .map(|input| TemplateInputView { default: seed::default_value(document, &input), input })
                    .collect()
            })
            .unwrap_or_default();
        self.read(|state| {
            let mut copies: Vec<CopyView> = state
                .parts
                .iter()
                .filter_map(|p| {
                    let first = p.revisions.iter().find(|r| {
                        r.template.as_ref().is_some_and(|t| t.template_part == seed.part.id)
                    })?;
                    let stamp = first.template.as_ref()?;
                    Some(CopyView {
                        part_id: p.id.clone(),
                        number: p.number.clone(),
                        name: p.name.clone(),
                        values: stamp.values.clone(),
                        created_at: stamp.created_at,
                        bake: bake_name(first),
                    })
                })
                .collect();
            copies.sort_by(|a, b| b.created_at.cmp(&a.created_at));
            let kind = member_type(&seed.part);
            Ok(TemplateView {
                template_id: seed.part.id.clone(),
                number: seed.part.number.clone(),
                revision_id: seed.revision.id.clone(),
                revision_label: seed.revision.label.clone(),
                has_document: seed.document.is_some(),
                member_part_type_mode: mode_of(state, &kind),
                member_part_type: kind,
                inputs,
                copies,
            })
        })
    }

    /// Spin a new normal part out of a template: its first revision is the
    /// template's document with `values` in its expressions (the CAD app's
    /// `spin_out`), stamped with the template, and queued for a bake.
    pub fn spin_out(&self, user: &User, key: &str, request: SpinOutRequest) -> Result<Part, Error> {
        let seed = self.seed(key, DocumentClass::Template, &request.template_revision, |p| {
            p.current_release().or(p.latest())
        })?;
        let document = Self::seed_document(&seed)?;
        let template_file = format!("{}.tbrep", seed.part.number);
        let (copy, used) = seed::spin_out(document, &template_file, &request.values).map_err(Error::bad_request)?;
        let part_type = if request.part_type.trim().is_empty() {
            member_type(&seed.part)
        } else {
            request.part_type.trim().to_string()
        };
        let category = self.read(|state| catalog::find(&state.categories, &seed.part.category).map(|c| c.id.clone()));
        let part = self.create_part_with(
            user,
            &PartSpec {
                part_type,
                number: request.number.trim().to_string(),
                name: request.name,
                description: request.description,
                category: category.unwrap_or_default(),
                document_class: DocumentClass::Normal,
                tags: seed.part.tags.clone(),
                workspace: crate::workspace::link_on_create(
                    crate::model::Origin::Authored,
                    false,
                    &request.workspace_folder,
                ),
                ..PartSpec::default()
            },
        )?;
        let revision = part.revisions.first().ok_or_else(|| Error::internal("a new part has no revision"))?;
        let stamp = TemplateStamp {
            template_part: seed.part.id.clone(),
            template_revision: seed.revision.id.clone(),
            values: used,
            content_hash: seed::template_hash(document),
            created_at: now(),
        };
        let bake = new_bake(
            user,
            format!("spun out of template {} revision {}", seed.part.number, seed.revision.label),
        );
        self.write_generated(
            user,
            &part.id,
            &revision.id,
            &copy.to_string(),
            Written::Template(stamp),
            Some(bake),
            &serde_json::Map::new(),
        )?;
        self.read(|state| state.part(&part.id).cloned()).ok_or_else(|| Error::not_found("part"))
    }
}

// ===========================================================================
// The bake queue
// ===========================================================================

impl Db {
    /// The bake jobs, oldest first. `status` narrows to one status; `all`
    /// includes finished jobs, which are otherwise left out.
    pub fn bake_jobs(&self, status: &str) -> Vec<BakeJob> {
        let status = status.trim().to_ascii_lowercase();
        let mut jobs: Vec<BakeJob> = self.read(|state| {
            state
                .parts
                .iter()
                .flat_map(|part| part.revisions.iter().map(move |r| (part, r)))
                .filter_map(|(part, revision)| {
                    let bake = revision.bake.as_ref()?;
                    let keep = match status.as_str() {
                        "" => bake.status != BakeStatus::Done,
                        "all" => true,
                        wanted => bake.status.as_str() == wanted,
                    };
                    keep.then(|| job_view(state, part, revision, bake))
                })
                .collect()
        });
        jobs.sort_by(|a, b| a.requested_at.cmp(&b.requested_at).then_with(|| a.number.cmp(&b.number)));
        jobs
    }

    /// Take a bake job: `id`, or else the oldest one free. A job is free when
    /// it is pending, or claimed longer than [`BAKE_LEASE`] ago. `None` when
    /// asked for the next and nothing is free.
    pub fn claim_bake(&self, worker: &User, id: Option<&str>) -> Result<Option<BakeJob>, Error> {
        let worker_id = worker.id.clone();
        let id = id.map(str::to_string);
        self.mutate(move |state| {
            let stamp = now();
            // A revision someone has checked out waits: a draft is written
            // only by its lock holder, and a worker is not one.
            let unlocked = |revision: &Revision| revision.lock.as_ref().is_none_or(|l| l.user_id == worker_id);
            let free = |revision: &Revision| {
                unlocked(revision)
                    && revision.bake.as_ref().is_some_and(|b| match b.status {
                        BakeStatus::Pending => true,
                        BakeStatus::Claimed => b.claimed_at.is_none_or(|at| at + BAKE_LEASE <= stamp),
                        _ => false,
                    })
            };
            let chosen = match &id {
                Some(id) => {
                    let (part, revision) = locate(state, id)?;
                    if !free(revision) {
                        let bake = revision.bake.as_ref().ok_or_else(|| Error::not_found("bake job"))?;
                        if let Some(lock) = revision.lock.as_ref().filter(|_| !unlocked(revision)) {
                            return Err(Error::conflict(format!(
                                "{} revision {} is checked out by {} — its bake waits until it is checked in",
                                part.number,
                                revision.label,
                                user_name(state, &lock.user_id)
                            )));
                        }
                        return Err(Error::conflict(match bake.status {
                            BakeStatus::Claimed => format!(
                                "{} revision {} is being baked by {} — the claim lapses {} seconds after it was taken",
                                part.number,
                                revision.label,
                                bake.claimed_by.as_deref().map(|u| user_name(state, u)).unwrap_or_default(),
                                BAKE_LEASE
                            ),
                            BakeStatus::Failed => format!(
                                "{} revision {} failed its bake — retry it first",
                                part.number, revision.label
                            ),
                            _ => format!("{} revision {} has nothing to bake", part.number, revision.label),
                        }));
                    }
                    Some((part.id.clone(), revision.id.clone()))
                }
                None => state
                    .parts
                    .iter()
                    .flat_map(|p| p.revisions.iter().map(move |r| (p, r)))
                    .filter(|(_, r)| free(r))
                    .min_by_key(|(p, r)| (r.bake.as_ref().map(|b| b.requested_at), p.number.clone()))
                    .map(|(p, r)| (p.id.clone(), r.id.clone())),
            };
            let Some((part_id, revision_id)) = chosen else {
                return Ok(None);
            };
            let revision = crate::db::find_revision_mut(state, &part_id, &revision_id)?.1;
            let bake = revision.bake.as_mut().expect("chosen as free");
            bake.status = BakeStatus::Claimed;
            bake.claimed_by = Some(worker_id.clone());
            bake.claimed_at = Some(stamp);
            bake.attempts += 1;
            let part = state.part(&part_id).expect("found above");
            let revision = part.revision(&revision_id).expect("found above");
            Ok(Some(job_view(state, part, revision, revision.bake.as_ref().expect("just set"))))
        })
    }

    /// Extend a claimed job's lease: the claim holder says "still baking", and
    /// the claim runs [`BAKE_LEASE`] from now. A long bake renews every so
    /// often instead of being handed to another worker at 15 minutes. Only the
    /// holder may renew ([`held_by`]): a job another worker took over after
    /// the lease lapsed, or one nobody has claimed, is refused by name.
    pub fn renew_bake(&self, worker: &User, id: &str) -> Result<BakeJob, Error> {
        let worker_id = worker.id.clone();
        let id = id.to_string();
        self.mutate(move |state| {
            let (part, revision) = locate(state, &id)?;
            held_by(state, part, revision, &worker_id)?;
            let (part_id, revision_id) = (part.id.clone(), revision.id.clone());
            let revision = crate::db::find_revision_mut(state, &part_id, &revision_id)?.1;
            let bake = revision.bake.as_mut().expect("held above");
            bake.claimed_at = Some(now());
            let part = state.part(&part_id).expect("found above");
            let revision = part.revision(&revision_id).expect("found above");
            Ok(job_view(state, part, revision, revision.bake.as_ref().expect("held above")))
        })
    }

    /// A worker's result: the baked document replaces the assembled one, the
    /// job is done, and a family member's stamp takes the new hash (a baked
    /// member is not "edited by hand").
    pub fn finish_bake(&self, worker: &User, id: &str, body: &str) -> Result<(), Error> {
        let parsed: Value = serde_json::from_str(body).map_err(|_| Error::bad_request("a document must be JSON"))?;
        if !parsed.is_object() {
            return Err(Error::bad_request("a document must be a JSON object"));
        }
        let hash = auth::content_hash(body);
        let size = body.len() as u64;
        let worker_id = worker.id.clone();
        let id = id.to_string();
        let path_of = |part_id: &str| self.document_path(&format!("part/{part_id}/rev/{id}"));
        let part_id = self.read(|state| locate(state, &id).map(|(p, _)| p.id.clone()))?;
        let path = path_of(&part_id)?;
        self.mutate(move |state| {
            let (part, revision) = locate(state, &id)?;
            held_by(state, part, revision, &worker_id)?;
            if let Some(lock) = revision.lock.as_ref().filter(|l| l.user_id != worker_id) {
                return Err(Error::conflict(format!(
                    "{} revision {} was checked out by {} after the bake was claimed — report the bake failed, or wait and retry",
                    part.number,
                    revision.label,
                    user_name(state, &lock.user_id)
                )));
            }
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(format!(
                    "{} revision {} is {} — its document can no longer change",
                    part.number,
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            let key = revision.document_key(&part.id);
            let revision = crate::db::find_revision_mut(state, &part_id, &id)?.1;
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent).map_err(Error::internal)?;
            }
            write_atomic(&path, body).map_err(Error::internal)?;
            revision.content_hash = hash.clone();
            revision.size = size;
            revision.modified_at = now();
            if let Some(stamp) = revision.family.as_mut() {
                stamp.generated_hash = hash;
            }
            let bake = revision.bake.as_mut().expect("held_by checked it");
            bake.status = BakeStatus::Done;
            bake.error.clear();
            bake.finished_by = Some(worker_id);
            bake.finished_at = Some(now());
            crate::review::document_changed(state, &part_id, &id);
            state.touch(key);
            Ok(())
        })
    }

    /// A worker reports that a bake failed (an expression that does not
    /// evaluate, a feature that does not build). The job waits for a person.
    pub fn fail_bake(&self, worker: &User, id: &str, error: &str) -> Result<(), Error> {
        let error = error.trim();
        if error.is_empty() {
            return Err(Error::bad_request("say why the bake failed"));
        }
        let worker_id = worker.id.clone();
        let error = error.to_string();
        let id = id.to_string();
        self.mutate(move |state| {
            let (part, revision) = locate(state, &id)?;
            held_by(state, part, revision, &worker_id)?;
            let part_id = part.id.clone();
            let revision = crate::db::find_revision_mut(state, &part_id, &id)?.1;
            let bake = revision.bake.as_mut().expect("held_by checked it");
            bake.status = BakeStatus::Failed;
            bake.error = error;
            bake.finished_by = Some(worker_id);
            bake.finished_at = Some(now());
            Ok(())
        })
    }

    /// Put a failed or claimed job back in the queue — after fixing the row
    /// that failed, or to take a job off a worker that went away.
    pub fn retry_bake(&self, id: &str) -> Result<(), Error> {
        let id = id.to_string();
        self.mutate(move |state| {
            let (part, revision) = locate(state, &id)?;
            let (number, label) = (part.number.clone(), revision.label.clone());
            let part_id = part.id.clone();
            let revision = crate::db::find_revision_mut(state, &part_id, &id)?.1;
            if !revision.lifecycle.is_editable() {
                return Err(Error::conflict(format!("{number} revision {label} is no longer in work")));
            }
            let bake = revision.bake.as_mut().ok_or_else(|| Error::not_found("bake job"))?;
            if !matches!(bake.status, BakeStatus::Failed | BakeStatus::Claimed) {
                return Err(Error::conflict(format!(
                    "{number} revision {label}'s bake is {} — only a failed or claimed one goes back in the queue",
                    bake.status.as_str()
                )));
            }
            bake.status = BakeStatus::Pending;
            bake.claimed_by = None;
            bake.claimed_at = None;
            Ok(())
        })
    }
}

/// The part and revision a bake job id (a revision id) names.
fn locate<'a>(state: &'a State, id: &str) -> Result<(&'a Part, &'a Revision), Error> {
    state
        .parts
        .iter()
        .find_map(|p| p.revision(id).map(|r| (p, r)))
        .filter(|(_, r)| r.bake.is_some())
        .ok_or_else(|| Error::not_found("bake job"))
}

/// Refuse unless `worker` holds the job's claim.
fn held_by(state: &State, part: &Part, revision: &Revision, worker_id: &str) -> Result<(), Error> {
    let bake = revision.bake.as_ref().ok_or_else(|| Error::not_found("bake job"))?;
    if bake.status != BakeStatus::Claimed {
        return Err(Error::conflict(format!(
            "{} revision {}'s bake is {}{} — claim it first",
            part.number,
            revision.label,
            bake.status.as_str(),
            if bake.error.is_empty() { String::new() } else { format!(" ({})", bake.error) }
        )));
    }
    if bake.claimed_by.as_deref() != Some(worker_id) {
        return Err(Error::conflict(format!(
            "{} revision {} is being baked by {}",
            part.number,
            revision.label,
            bake.claimed_by.as_deref().map(|u| user_name(state, u)).unwrap_or_default()
        )));
    }
    Ok(())
}

fn job_view(state: &State, part: &Part, revision: &Revision, bake: &Bake) -> BakeJob {
    BakeJob {
        id: revision.id.clone(),
        part_id: part.id.clone(),
        number: part.number.clone(),
        name: part.name.clone(),
        revision_label: revision.label.clone(),
        lifecycle: revision.lifecycle.as_str(),
        status: bake.status.as_str(),
        reason: bake.reason.clone(),
        requested_by: user_name(state, &bake.requested_by),
        requested_at: bake.requested_at,
        claimed_by: bake.claimed_by.as_deref().map(|u| user_name(state, u)),
        claimed_at: bake.claimed_at,
        lease_expires_at: (bake.status == BakeStatus::Claimed)
            .then(|| bake.claimed_at.map(|at| at + BAKE_LEASE))
            .flatten(),
        attempts: bake.attempts,
        error: bake.error.clone(),
        finished_at: bake.finished_at,
        document_key: revision.document_key(&part.id),
        source: if revision.family.is_some() {
            "family"
        } else if revision.template.is_some() {
            "template"
        } else {
            ""
        },
        locked_by: revision.lock.as_ref().map(|l| user_name(state, &l.user_id)),
    }
}
