//! KiCad library parts become real PLM parts (S13's KiCad half, over P8).
//!
//! The KiCad import (`panels::kicad_bulk`) already turns each symbol into a
//! part document, keyed by the symbol's `library_id` (`Timer:NE555D`). On a
//! PLM those documents become Parts of their own part type:
//!
//! * **Deduped on `external_ref`** = the `library_id`. The server holds it
//!   unique per part type (P8), so a re-import finds the part it made before,
//!   and two imports of one symbol at once cannot make two parts. The second
//!   one's `409` makes it re-read and adopt the first one's part.
//! * **Minted as imported**, written, then **released**: a library part is
//!   something people place, not something they draft.
//! * **Revised on re-import only when it should be.** A part whose latest
//!   revision is `imported` and whose document hash differs gets a new
//!   `imported` revision. An unchanged one is skipped without downloading
//!   anything: the hash is SHA-256 of the exact bytes this app would upload,
//!   compared with the server's `latest_content_hash`, and those bytes are
//!   deterministic (pinned by
//!   `kicad_import::one_symbol_imports_to_the_same_bytes_every_time`). A part
//!   someone has since revised BY HAND (latest revision `authored`) is left
//!   alone. The import would otherwise overwrite their work.
//!
//! There is no separate resume ledger. The server is the record: every part
//! is found by its `external_ref` in one cheap lookup, and a finished part
//! compares equal and is skipped. A run stopped midway loses at most the part
//! in flight. A part stopped between its write and its release is an
//! `imported` draft, and the next run finishes it.

use crate::plm::client::PlmClient;
use serde_json::Value;

/// Lowercase hex SHA-256 of `bytes` — the server's `content_hash` of a
/// document written with exactly these bytes.
pub fn content_hash(bytes: &[u8]) -> String {
    use sha2::{Digest, Sha256};
    Sha256::digest(bytes).iter().map(|byte| format!("{byte:02x}")).collect()
}

/// What the server says about the part a `library_id` already names (a row of
/// `GET /api/parts?external_ref=`, P8).
#[derive(Clone, Debug, Default, PartialEq, serde::Deserialize)]
pub struct ImportedRow {
    pub id: String,
    #[serde(default)]
    pub number: String,
    #[serde(default)]
    pub latest_revision_id: String,
    /// `imported`, `authored` or `generated`; `None` for a part with no
    /// revision.
    #[serde(default)]
    pub latest_origin: Option<String>,
    /// `""` when the latest revision has no document yet.
    #[serde(default)]
    pub latest_content_hash: String,
    /// The latest revision's lifecycle (`draft`, `released`, …).
    #[serde(default)]
    pub latest_state: String,
}

/// What to do with one KiCad part.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Action {
    /// No part names this `library_id`: mint one.
    Mint,
    /// The latest revision is an imported DRAFT — a run stopped between its
    /// write and its release. Write it and release it.
    Finish { part: String, revision: String },
    /// The latest revision is imported, released and different: a new one.
    Revise { part: String },
    /// Nothing to send, and why.
    Skip { part: String, reason: String },
}

/// Decide from the server's row (`None`: no part yet) and the hash of the
/// document this import would write.
pub fn decide(existing: Option<&ImportedRow>, hash: &str) -> Action {
    let Some(row) = existing else {
        return Action::Mint;
    };
    let skip = |reason: String| Action::Skip { part: row.id.clone(), reason };
    match row.latest_origin.as_deref() {
        Some("imported") => {}
        Some(origin) => {
            return skip(format!(
                "{} was revised by hand (its latest revision is {origin}) — the library is not written over it",
                row.number
            ))
        }
        None => {
            return Action::Finish { part: row.id.clone(), revision: row.latest_revision_id.clone() };
        }
    }
    if row.latest_state == "draft" {
        return Action::Finish { part: row.id.clone(), revision: row.latest_revision_id.clone() };
    }
    if row.latest_content_hash == hash {
        return skip(format!("{} is unchanged", row.number));
    }
    Action::Revise { part: row.id.clone() }
}

/// What happened to one part.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Written and released: the part and its new revision.
    Written { part: String, revision: String, created_part: bool },
    Skipped { part: String, reason: String },
}

fn body(value: Value) -> Option<Vec<u8>> {
    Some(serde_json::to_vec(&value).unwrap_or_default())
}

async fn call(client: &PlmClient, method: &'static str, path: &str, payload: Option<Vec<u8>>) -> Result<Value, String> {
    let response = client.call(method, path, payload).await.map_err(|error| error.to_string())?;
    Ok(serde_json::from_slice(&response.body).unwrap_or(Value::Null))
}

fn encode(text: &str) -> String {
    text.bytes()
        .map(|byte| match byte {
            b'A'..=b'Z' | b'a'..=b'z' | b'0'..=b'9' | b'-' | b'.' | b'_' | b'~' => (byte as char).to_string(),
            _ => format!("%{byte:02X}"),
        })
        .collect()
}

/// The part `library_id` names in `part_type`, if any.
pub async fn find(client: &PlmClient, part_type: &str, library_id: &str) -> Result<Option<ImportedRow>, String> {
    let page = call(
        client,
        "GET",
        &format!("/api/parts?limit=2&part_type={}&external_ref={}", encode(part_type), encode(library_id)),
        None,
    )
    .await?;
    let rows = page["parts"].as_array().cloned().unwrap_or_default();
    Ok(rows.into_iter().next().and_then(|row| serde_json::from_value(row).ok()))
}

/// Write `document` into a revision and release it: checkout, write, check
/// in, release — the server's order.
async fn write_and_release(client: &PlmClient, part: &str, revision: &str, document: &[u8]) -> Result<(), String> {
    let base = format!("/api/parts/{part}/revisions/{revision}");
    call(client, "POST", &format!("{base}/checkout"), body(serde_json::json!({ "client_id": "brep-app kicad" }))).await?;
    let written = call(client, "PUT", &format!("/api/store/doc/part/{part}/rev/{revision}"), Some(document.to_vec())).await;
    call(client, "POST", &format!("{base}/checkin"), body(serde_json::json!({}))).await?;
    written?;
    call(client, "POST", &format!("{base}/state"), body(serde_json::json!({ "to": "released" }))).await?;
    Ok(())
}

/// Publish one KiCad part: find it by `library_id`, then mint, finish, revise
/// or skip ([`decide`]). `document` is the exact bytes the KiCad import wrote
/// for it. A mint refused because another import minted the same
/// `library_id` meanwhile (`409`) re-reads and carries on with that part.
pub async fn publish_part(
    client: &PlmClient,
    part_type: &str,
    library_id: &str,
    name: &str,
    document: &[u8],
) -> Result<Outcome, String> {
    let hash = content_hash(document);
    for attempt in 0..2 {
        let existing = find(client, part_type, library_id).await?;
        match decide(existing.as_ref(), &hash) {
            Action::Skip { part, reason } => return Ok(Outcome::Skipped { part, reason }),
            Action::Finish { part, revision } => {
                write_and_release(client, &part, &revision, document).await?;
                return Ok(Outcome::Written { part, revision, created_part: false });
            }
            Action::Revise { part } => {
                let revision = call(
                    client,
                    "POST",
                    &format!("/api/parts/{part}/revisions"),
                    body(serde_json::json!({ "origin": "imported" })),
                )
                .await?;
                let revision = revision["id"].as_str().unwrap_or_default().to_string();
                write_and_release(client, &part, &revision, document).await?;
                return Ok(Outcome::Written { part, revision, created_part: false });
            }
            Action::Mint => {
                let request = serde_json::json!({
                    "part_type": part_type,
                    "name": name,
                    "external_ref": library_id,
                    "origin": "imported",
                    // A library import: the server links none of these parts
                    // into the workspace (`brep_plm::workspace::link_on_create`).
                    "bulk": true,
                });
                match client.call("POST", "/api/parts", body(request)).await {
                    Ok(response) => {
                        let created: Value = serde_json::from_slice(&response.body).unwrap_or(Value::Null);
                        let part = created["id"].as_str().unwrap_or_default().to_string();
                        let revision = created["revisions"][0]["id"].as_str().unwrap_or_default().to_string();
                        write_and_release(client, &part, &revision, document).await?;
                        return Ok(Outcome::Written { part, revision, created_part: true });
                    }
                    // Someone minted this library_id first: adopt theirs.
                    Err(error) if error.status() == Some(409) && attempt == 0 => continue,
                    Err(error) => return Err(error.to_string()),
                }
            }
        }
    }
    Err(format!("'{library_id}' could not be minted or found"))
}

/// What a library pass did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct LibraryReport {
    pub written: usize,
    pub created: usize,
    pub skipped: usize,
    /// `(library_id, sentence)` for every part that could not be published.
    /// Unlike the folder import, one refused part does not stop the pass: a
    /// library is thousands of independent parts.
    pub failed: Vec<(String, String)>,
}

/// Publish a KiCad library — the documents a sweep wrote, as
/// `(library_id, document name)` pairs from its ledger — to the PLM, one part
/// at a time, reading each document's bytes from `store` (exactly what the
/// sweep wrote, so an unchanged part hashes equal and is skipped). The name
/// a new part gets is the symbol's own (`NE555D` of `Timer:NE555D`).
/// `progress` is called after every part with the report so far and
/// answers whether to go on (`false` is Stop; the next pass skips what this
/// one finished, by the server's own record).
pub async fn publish_library(
    client: &PlmClient,
    store: &dyn crate::store::ModelStore,
    part_type: &str,
    parts: &[(String, String)],
    progress: &mut dyn FnMut(&LibraryReport) -> bool,
) -> LibraryReport {
    let mut report = LibraryReport::default();
    for (library_id, document) in parts {
        let outcome = match store.read(document) {
            None => Err(format!("its document '{document}' is not in the store")),
            Some(bytes) => {
                let name = library_id.rsplit(':').next().unwrap_or(library_id);
                publish_part(client, part_type, library_id, name, bytes.as_bytes()).await
            }
        };
        match outcome {
            Ok(Outcome::Written { created_part, .. }) => {
                report.written += 1;
                report.created += usize::from(created_part);
            }
            Ok(Outcome::Skipped { .. }) => report.skipped += 1,
            Err(sentence) => report.failed.push((library_id.clone(), sentence)),
        }
        if !progress(&report) {
            break;
        }
    }
    report
}

