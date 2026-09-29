//! An assembly revision's USES LIST, read off its document (S5, the pure half).
//!
//! The server keeps, per revision, the list of child parts it uses
//! (`PUT /api/parts/:id/revisions/:rev/uses`). The CAD app is that list's
//! publisher, and the document is its only source: every top-level `ACOMP`
//! occurrence names a `partsLibrary` entry, whose `sourceKey` is the identity
//! the part was inserted from. On a PLM store that identity is a revision key,
//! so an occurrence resolves to a part and a concrete revision (D3: the CAD app
//! has no floating occurrences).
//!
//! * [`occurrences`] walks the document.
//! * [`uses_lines`] folds them into lines, one per part and revision, and
//!   REFUSES every occurrence that does not resolve rather than dropping it: a
//!   uses list one child short is a wrong BOM that validates.
//! * [`disagreements`] compares a uses list the server holds against the
//!   document, which is what an assembly must check on open after someone ran
//!   Replace everywhere (the server swaps the child in the LIST, never in the
//!   document).
//!
//! Nothing here talks to a server or a store. Identity comes in as a resolver
//! (`ModelStore::identity` in the app), so a file store, whose identities are
//! all paths, publishes nothing.

use crate::store::DocumentIdentity;
use serde_json::Value;

/// A use line's fields, EXACTLY: the server answers `400` naming any other key
/// (`BREP_plm/src/bom.rs` `check_uses`).
pub const USE_FIELDS: [&str; 7] = [
    "part",
    "revision",
    "quantity",
    "unit",
    "find_number",
    "reference",
    "notes",
];

/// One placement of a part in an assembly document.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Occurrence {
    /// The owning feature id (`ACOMP1`).
    pub id: String,
    /// What a person calls this placement: its `Reference_Designator` when set,
    /// else its id — the engine's own relabel rule
    /// (`BREP_render` `set_occurrence_attribute`).
    pub label: String,
    /// The `partsLibrary` entry it places.
    pub part_name: String,
    /// That entry's `sourceKey`: empty for an embedded-only part, `None` when
    /// the document has no entry of that name at all.
    pub source_key: Option<String>,
}

/// Every top-level occurrence of `document`, in feature order.
///
/// Nested occurrences are NOT here: a sub-assembly is one child of this
/// revision, and its own children are its own revision's uses list. A saved
/// document carries no rollback point, so every component feature counts.
pub fn occurrences(document: &Value) -> Vec<Occurrence> {
    let library = document.get("partsLibrary");
    document
        .get("features")
        .and_then(Value::as_array)
        .into_iter()
        .flatten()
        .filter(|feature| {
            matches!(
                feature.get("type").and_then(Value::as_str),
                Some("ACOMP") | Some("ASSEMBLY COMPONENT")
            )
        })
        .map(|feature| {
            let params = &feature["inputParams"];
            let id = params["id"].as_str().unwrap_or_default().to_string();
            let designator = params["bom"]["Reference_Designator"]
                .as_str()
                .map(str::trim)
                .unwrap_or_default();
            let part_name = params["partName"].as_str().unwrap_or_default().to_string();
            let source_key = library
                .and_then(|library| library.get(&part_name))
                .map(|entry| entry["sourceKey"].as_str().unwrap_or_default().to_string());
            Occurrence {
                label: if designator.is_empty() { id.clone() } else { designator.to_string() },
                id,
                part_name,
                source_key,
            }
        })
        .collect()
}

/// One line of a uses list, as the server takes it — these seven fields and no
/// other ([`USE_FIELDS`]).
#[derive(Clone, Debug, PartialEq, serde::Serialize)]
pub struct UseLine {
    /// The child part's id.
    pub part: String,
    /// The child revision's id — never empty (D3).
    pub revision: String,
    /// How many occurrences place this part and revision.
    pub quantity: u32,
    pub unit: String,
    pub find_number: String,
    /// The occurrences' labels, `", "`-joined in feature order.
    pub reference: String,
    pub notes: String,
}

/// A line and the occurrences (by feature id) it stands for.
#[derive(Clone, Debug, PartialEq)]
pub struct PlacedLine {
    pub line: UseLine,
    pub occurrences: Vec<String>,
}

/// An occurrence that names no PLM revision, and why. The publisher refuses to
/// publish while any exist.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Refusal {
    pub occurrence: String,
    pub reason: String,
}

/// A document's uses list: the lines, and the occurrences that could not be
/// put on one.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct UsesList {
    pub lines: Vec<PlacedLine>,
    pub refusals: Vec<Refusal>,
}

impl UsesList {
    /// The `PUT …/uses` body, `{"uses": [...]}`. Refuses, naming every
    /// occurrence, while any occurrence is unresolved.
    pub fn body(&self) -> Result<Value, String> {
        if !self.refusals.is_empty() {
            let named: Vec<String> = self
                .refusals
                .iter()
                .map(|refusal| format!("{}: {}", refusal.occurrence, refusal.reason))
                .collect();
            return Err(format!(
                "the uses list was not published — {} occurrence{} name{} no PLM revision: {}",
                self.refusals.len(),
                if self.refusals.len() == 1 { "" } else { "s" },
                if self.refusals.len() == 1 { "s" } else { "" },
                named.join("; ")
            ));
        }
        let lines: Vec<&UseLine> = self.lines.iter().map(|placed| &placed.line).collect();
        Ok(serde_json::json!({ "uses": lines }))
    }
}

/// Fold `occurrences` into uses lines: one per (part, revision), in the order
/// each first appears; `quantity` the count, `reference` the labels. `resolve`
/// reads a `sourceKey` as an identity (`ModelStore::identity`).
///
/// The fold key is part and revision ONLY. Two placements of one revision with
/// different designators are one line with both in `reference`: a second line
/// for the same part and revision would need a find number to tell it apart,
/// and the server refuses the repeat.
pub fn uses_lines(
    occurrences: &[Occurrence],
    resolve: &dyn Fn(&str) -> DocumentIdentity,
) -> UsesList {
    let mut list = UsesList::default();
    let mut labels: Vec<Vec<String>> = Vec::new();
    for occurrence in occurrences {
        let refuse = |reason: String| Refusal {
            occurrence: occurrence.label.clone(),
            reason,
        };
        let key = match &occurrence.source_key {
            None => {
                list.refusals.push(refuse(format!(
                    "the document has no parts-library entry '{}'",
                    occurrence.part_name
                )));
                continue;
            }
            Some(key) if key.is_empty() => {
                list.refusals.push(refuse(format!(
                    "'{}' is embedded in this document and was never a part — insert it from the PLM",
                    occurrence.part_name
                )));
                continue;
            }
            Some(key) => key,
        };
        let (part, revision) = match resolve(key) {
            DocumentIdentity::Revision { part, revision } => (part, revision),
            DocumentIdentity::Path(path) => {
                list.refusals.push(refuse(format!(
                    "'{}' was inserted from the file '{path}', not a PLM revision — import it first",
                    occurrence.part_name
                )));
                continue;
            }
        };
        match list
            .lines
            .iter()
            .position(|placed| placed.line.part == part && placed.line.revision == revision)
        {
            Some(index) => {
                list.lines[index].line.quantity += 1;
                list.lines[index].occurrences.push(occurrence.id.clone());
                labels[index].push(occurrence.label.clone());
            }
            None => {
                list.lines.push(PlacedLine {
                    line: UseLine {
                        part,
                        revision,
                        quantity: 1,
                        unit: "each".into(),
                        find_number: String::new(),
                        reference: String::new(),
                        notes: String::new(),
                    },
                    occurrences: vec![occurrence.id.clone()],
                });
                labels.push(vec![occurrence.label.clone()]);
            }
        }
    }
    for (placed, labels) in list.lines.iter_mut().zip(labels) {
        placed.line.reference = labels.join(", ");
    }
    list
}

/// One line of the uses list the SERVER holds for a revision, reduced to what
/// the comparison needs. `revision` is a revision ID (the server's
/// `resolved_uses` answers ids); empty is a floating line, which the CAD app
/// never writes (D3) but an import or another client may have.
#[derive(Clone, Debug, PartialEq)]
pub struct ListedUse {
    pub part: String,
    pub revision: String,
    pub quantity: f64,
}

/// Occurrences to move from one part revision to another so the document
/// matches its uses list again.
#[derive(Clone, Debug, PartialEq)]
pub struct Repoint {
    /// Feature ids, in feature order.
    pub occurrences: Vec<String>,
    /// `(part, revision)` the document places now.
    pub from: (String, String),
    /// `(part, revision)` the uses list names instead.
    pub to: (String, String),
}

/// Where a document and its revision's uses list disagree.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Disagreement {
    /// Proposed re-points: only the pairings that are unambiguous (see
    /// [`disagreements`]).
    pub repoints: Vec<Repoint>,
    /// What the document places that the list does not name, left unpaired.
    pub unlisted: Vec<PlacedLine>,
    /// What the list names that the document does not place, left unpaired.
    pub unplaced: Vec<ListedUse>,
}

impl Disagreement {
    pub fn is_empty(&self) -> bool {
        self.repoints.is_empty() && self.unlisted.is_empty() && self.unplaced.is_empty()
    }
}

/// Compare the server's uses list for a revision with the lines its document
/// yields. Matching is on part and revision ids, and a line that matches on
/// both is agreement even if the quantities differ: a different COUNT is the
/// document changing, which the next save republishes, not a swap to undo.
///
/// Re-points are proposed only where the pairing cannot be a guess:
/// * the same part on both sides, once each — a revision swap;
/// * after that, exactly one line left on each side with the same quantity —
///   the part swap Replace everywhere makes.
///
/// Anything else is reported unpaired, for a person to decide.
pub fn disagreements(listed: &[ListedUse], document: &UsesList) -> Disagreement {
    let mut unplaced: Vec<ListedUse> = listed
        .iter()
        .filter(|listed| {
            !document
                .lines
                .iter()
                .any(|placed| placed.line.part == listed.part && placed.line.revision == listed.revision)
        })
        .cloned()
        .collect();
    let mut unlisted: Vec<PlacedLine> = document
        .lines
        .iter()
        .filter(|placed| {
            !listed
                .iter()
                .any(|listed| listed.part == placed.line.part && listed.revision == placed.line.revision)
        })
        .cloned()
        .collect();
    let mut repoints = Vec::new();

    let repoint = |placed: &PlacedLine, listed: &ListedUse| Repoint {
        occurrences: placed.occurrences.clone(),
        from: (placed.line.part.clone(), placed.line.revision.clone()),
        to: (listed.part.clone(), listed.revision.clone()),
    };

    // A revision swap: the part appears exactly once on each side.
    let mut index = 0;
    while index < unlisted.len() {
        let part = unlisted[index].line.part.clone();
        let same_part_listed: Vec<usize> = (0..unplaced.len())
            .filter(|&i| unplaced[i].part == part)
            .collect();
        let same_part_placed = unlisted.iter().filter(|placed| placed.line.part == part).count();
        if same_part_listed.len() == 1 && same_part_placed == 1 {
            let listed = unplaced.remove(same_part_listed[0]);
            let placed = unlisted.remove(index);
            repoints.push(repoint(&placed, &listed));
        } else {
            index += 1;
        }
    }
    // A part swap: one line left on each side, the same count.
    if unlisted.len() == 1
        && unplaced.len() == 1
        && unplaced[0].quantity == f64::from(unlisted[0].line.quantity)
    {
        let listed = unplaced.remove(0);
        let placed = unlisted.remove(0);
        repoints.push(repoint(&placed, &listed));
    }
    Disagreement {
        repoints,
        unlisted,
        unplaced,
    }
}

/// Point the document's occurrences in `repoint` at `to_key`
/// (`part/<part>/rev/<revision>`): every `partsLibrary` entry whose
/// `sourceKey` names the revision they place now gets the new key.
///
/// Only the KEY moves. The entry's embedded document is still the old part's,
/// and its `sourceSignature` no longer matches what the key reads, so Update
/// Components lights and brings the new part's geometry in — the same path
/// every other outdated component takes, not a second one. Returns how many
/// entries were moved.
pub fn repoint(document: &mut Value, repoint: &Repoint, resolve: &dyn Fn(&str) -> DocumentIdentity) -> usize {
    let to_key = DocumentIdentity::Revision {
        part: repoint.to.0.clone(),
        revision: repoint.to.1.clone(),
    }
    .key();
    let Some(library) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return 0;
    };
    let mut moved = 0;
    for entry in library.values_mut() {
        let key = entry.get("sourceKey").and_then(Value::as_str).unwrap_or_default().to_string();
        if key.is_empty() {
            continue;
        }
        if let DocumentIdentity::Revision { part, revision } = resolve(&key) {
            if (part, revision) == repoint.from {
                entry["sourceKey"] = Value::String(to_key.clone());
                moved += 1;
            }
        }
    }
    moved
}

// --- the publisher, over the PLM client --------------------------------------------

/// `part/<part>/rev/<revision>` split into the route halves, or the sentence
/// saying the document is not a PLM revision.
fn revision_of(key: &str) -> Result<(String, String), String> {
    match DocumentIdentity::parse_revision_key(key) {
        Some(DocumentIdentity::Revision { part, revision }) => Ok((part, revision)),
        _ => Err(format!("'{key}' is not a PLM revision — nothing to publish")),
    }
}

/// The revision key a PLM store's name for a document carries: the bare key
/// (`part/<p>/rev/<r>`), or the explorer's spelling of it
/// (`/models/part/<p>/rev/<r>.nbrep`) — which is what a component inserted
/// through the app records as its `sourceKey`. `None` for anything else.
pub fn revision_key_in(name: &str) -> Option<String> {
    let key = name.strip_prefix("/models/").unwrap_or(name);
    if !key.starts_with("part/") {
        return None;
    }
    let key = match key.rsplit_once('.') {
        Some((stem, extension)) if !extension.contains('/') => stem,
        _ => key,
    };
    DocumentIdentity::parse_revision_key(key).map(|identity| identity.key())
}

/// How a PLM store reads a `sourceKey`: a revision key (either spelling,
/// [`revision_key_in`]) is a revision, and anything else is a path, which
/// publishes nothing.
pub fn plm_identity(key: &str) -> DocumentIdentity {
    revision_key_in(key)
        .and_then(|key| DocumentIdentity::parse_revision_key(&key))
        .unwrap_or_else(|| DocumentIdentity::Path(key.to_string()))
}

/// Publish the uses list of the assembly revision `key` from its `document`,
/// right after its document was written, under the same checkout (S5). An
/// occurrence that names no PLM revision refuses the whole list, naming it;
/// the server's refusal (no lock, a released revision, a cycle) comes back as
/// its sentence. Returns the list sent.
pub async fn publish_uses(
    client: &crate::plm::client::PlmClient,
    key: &str,
    document: &Value,
) -> Result<UsesList, String> {
    let (part, revision) = revision_of(key)?;
    let list = uses_lines(&occurrences(document), &plm_identity);
    let body = list.body()?;
    client
        .call(
            "PUT",
            &format!("/api/parts/{part}/revisions/{revision}/uses"),
            Some(serde_json::to_vec(&body).unwrap_or_default()),
        )
        .await
        .map_err(|error| error.to_string())?;
    Ok(list)
}

/// Whether saving `document` must (re)publish its revision's uses list: it
/// places components, or it still has a parts library (an assembly whose
/// last component was just deleted must publish its now-empty list, or the
/// server keeps the old children). A plain part sends nothing.
pub fn publishes_on_save(document: &Value) -> bool {
    !occurrences(document).is_empty() || document.get("partsLibrary").is_some_and(|l| l.as_object().is_some_and(|m| !m.is_empty()))
}

/// The hook the PLM documents backend runs after every document write the
/// server accepted (`plm::backend::SavedHook`): publish the uses list right
/// after the document, under the same checkout (S5). Its refusal says the
/// document itself WAS saved.
pub(crate) fn publish_on_save() -> crate::plm::backend::SavedHook {
    std::rc::Rc::new(|client, key, value| {
        Box::pin(async move {
            let Ok(document) = serde_json::from_str::<Value>(&value) else {
                return Ok(());
            };
            if !publishes_on_save(&document) {
                return Ok(());
            }
            publish_uses(&client, &key, &document)
                .await
                .map(|_| ())
                .map_err(|refusal| format!("the document was saved, but its parts list was not published: {refusal}"))
        })
    })
}

/// Release the revision `key`, which the caller holds checked out, publishing
/// its uses list FIRST: the server freezes the list with the revision, and with
/// `require_released_children` on refuses the release naming every unreleased
/// child. With it off, the release answers `warnings` naming them, which are
/// returned for the app to show.
///
/// The order is the server's: a uses list is written only under the lock, and
/// a revision is released only once it is checked in. So: publish, check in,
/// release. A refused release leaves the revision checked in with its list
/// published — both true of it, and the user checks out again to change either.
pub async fn release(
    client: &crate::plm::client::PlmClient,
    key: &str,
    document: &Value,
) -> Result<Vec<String>, String> {
    let (part, revision) = revision_of(key)?;
    if !occurrences(document).is_empty() {
        publish_uses(client, key, document).await?;
    }
    client
        .call(
            "POST",
            &format!("/api/parts/{part}/revisions/{revision}/checkin"),
            Some(b"{}".to_vec()),
        )
        .await
        .map_err(|error| error.to_string())?;
    let response = client
        .call(
            "POST",
            &format!("/api/parts/{part}/revisions/{revision}/state"),
            Some(serde_json::to_vec(&serde_json::json!({ "to": "released" })).unwrap_or_default()),
        )
        .await
        .map_err(|error| error.to_string())?;
    let answer: Value = serde_json::from_slice(&response.body).unwrap_or_default();
    Ok(answer["warnings"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|warning| match warning {
            Value::String(text) => text.clone(),
            other => other.to_string(),
        })
        .collect())
}

/// The uses list the server holds for the revision `key`, as the comparison
/// reads it: part and revision ids (a floating line reads as the revision it
/// resolves to), and the quantity.
pub async fn listed_uses(client: &crate::plm::client::PlmClient, key: &str) -> Result<Vec<ListedUse>, String> {
    let (part, revision) = revision_of(key)?;
    let response = client
        .call("GET", &format!("/api/parts/{part}/revisions/{revision}/uses"), None)
        .await
        .map_err(|error| error.to_string())?;
    let answer: Value = serde_json::from_slice(&response.body).map_err(|error| error.to_string())?;
    Ok(answer["uses"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|line| ListedUse {
            part: line["part"].as_str().unwrap_or_default().to_string(),
            revision: line["revision"].as_str().unwrap_or_default().to_string(),
            quantity: line["quantity"].as_f64().unwrap_or(0.0),
        })
        .collect())
}

/// On open: where the revision's uses list and its document disagree — what
/// Replace everywhere leaves behind (it swaps the child in the LIST only). An
/// empty answer means the next save may republish from the document safely.
pub async fn check_on_open(
    client: &crate::plm::client::PlmClient,
    key: &str,
    document: &Value,
) -> Result<Disagreement, String> {
    let listed = listed_uses(client, key).await?;
    Ok(disagreements(&listed, &uses_lines(&occurrences(document), &plm_identity)))
}

