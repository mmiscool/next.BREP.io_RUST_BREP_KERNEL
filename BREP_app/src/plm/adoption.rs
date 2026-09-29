//! Adoption: turning a folder of documents into PLM parts (S13, the pure half).
//!
//! Without this there is no way from a tree of `.nbrep` files to a populated
//! PLM (D10). The importer mints one part per document, uploads each document
//! to its part's first revision, and publishes every assembly's uses list. An
//! imported assembly only resolves if its occurrences' `sourceKey`s have been
//! rewritten from file names to `part/<part>/rev/<revision>`.
//!
//! This module decides WHAT to do and in what order; the importer (wired once
//! the PLM client lands) does it. It never writes: the source tree is read
//! into [`SourceDocument`]s and every rewrite is made on a copy, so the same
//! folder still opens file-based afterwards.
//!
//! * [`plan_adoption`]: one [`PlannedPart`] per document, with its class
//!   (from the extension, as the file stores read it) and every `sourceKey` it
//!   names that the tree does not hold.
//! * [`Ledger`]: `identity → (part, revision)` and how far each document has
//!   got. It is what makes the import restartable. [`next_step`] reads the
//!   plan against the ledger and answers the first thing not yet done, so a
//!   run killed at any point resumes where it stopped. An import of a tree the
//!   ledger has already finished answers `None`: it writes nothing.
//! * [`rewrite_source_keys`]: every `sourceKey`, at EVERY depth. A
//!   sub-assembly's library entry carries that sub-assembly's whole document,
//!   whose own entries have `sourceKey`s too; a stale nested key is what Update
//!   Components would later "fix" by fetching a file that is not there.
//!
//! The order is three phases over the whole tree: mint every part, then
//! upload every document, then publish every assembly's uses list. A document
//! can only be uploaded rewritten once every part it names has been minted,
//! and a uses list can only name parts that exist. Part NUMBERS are the
//! server's to give (counter, pattern or script numbering cannot run here),
//! so a plan holds intents, and the ledger holds what the server answered.

use crate::document_class::DocumentClass;
use crate::plm::uses;
use crate::store::DocumentIdentity;
use serde_json::Value;
use std::collections::{BTreeMap, BTreeSet};

/// One document of the tree being adopted, as read — never written back.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SourceDocument {
    /// Its canonical identity (`ModelStore::canonical_identity`), the key
    /// every `sourceKey` is matched against.
    pub identity: String,
    pub contents: String,
}

/// A part the import will mint for one document.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PlannedPart {
    pub identity: String,
    /// From the file's extension: a `.fbrep` is a family, a `.tbrep` a
    /// template, anything else a normal part.
    pub class: DocumentClass,
    /// The name a person knew it by: its file name without the class
    /// extension.
    pub name: String,
    /// The document's content signature, so a later import can tell a
    /// changed file from an adopted one.
    pub signature: String,
    /// Whether it places components, which is what earns it a uses list.
    pub assembly: bool,
}

/// What an import of a tree will do.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct AdoptionPlan {
    /// One per document, in identity order.
    pub parts: Vec<PlannedPart>,
    /// Documents that will not be imported, and why.
    pub refused: Vec<(String, String)>,
    /// `(document, sourceKey)` for every key a document names that the tree
    /// does not hold. The key is left as it is; an assembly with one cannot
    /// publish a whole uses list, so the importer reports it before it starts.
    pub external: Vec<(String, String)>,
}

/// Plan the adoption of `documents`. `canonical` spells a `sourceKey` the way
/// the walker spelled the documents (`ModelStore::canonical_identity`).
pub fn plan_adoption(
    documents: &[SourceDocument],
    canonical: &dyn Fn(&str) -> String,
) -> AdoptionPlan {
    let mut plan = AdoptionPlan::default();
    let held: BTreeSet<&str> = documents.iter().map(|d| d.identity.as_str()).collect();
    let mut sorted: Vec<&SourceDocument> = documents.iter().collect();
    sorted.sort_by(|a, b| a.identity.cmp(&b.identity));
    for document in sorted {
        let parsed: Value = match serde_json::from_str(&document.contents) {
            Ok(parsed @ Value::Object(_)) => parsed,
            Ok(_) => {
                plan.refused
                    .push((document.identity.clone(), "not a model document (not a JSON object)".into()));
                continue;
            }
            Err(error) => {
                plan.refused
                    .push((document.identity.clone(), format!("does not parse: {error}")));
                continue;
            }
        };
        let mut keys = Vec::new();
        source_keys(&parsed, &mut keys);
        for key in keys {
            if DocumentIdentity::parse_revision_key(&key).is_some() {
                continue; // already a PLM part
            }
            if !held.contains(canonical(&key).as_str()) {
                plan.external.push((document.identity.clone(), key));
            }
        }
        let file = crate::store::file_name_of(&document.identity);
        plan.parts.push(PlannedPart {
            class: DocumentClass::of_name(&file).unwrap_or_default(),
            name: crate::document_class::strip_class_extension(&file).to_string(),
            signature: crate::panels::parts_library::document_signature(&document.contents),
            assembly: !uses::occurrences(&parsed).is_empty(),
            identity: document.identity.clone(),
        });
    }
    plan.external.dedup();
    plan
}

/// Every non-empty `sourceKey` in `document`'s parts library, at every depth,
/// in document order.
fn source_keys(document: &Value, out: &mut Vec<String>) {
    let Some(library) = document.get("partsLibrary").and_then(Value::as_object) else {
        return;
    };
    for entry in library.values() {
        if let Some(key) = entry.get("sourceKey").and_then(Value::as_str) {
            if !key.is_empty() {
                out.push(key.to_string());
            }
        }
        if let Some(nested) = entry.get("document") {
            source_keys(nested, out);
        }
    }
}

/// Rewrite every `sourceKey` in `document`, at every depth, through `to_key`
/// (a raw key → its `part/…/rev/…` key, or `None` when the import did not
/// mint it). Returns the keys `to_key` did not know, which are left exactly
/// as they were: an unknown key is reported, never blanked, because an empty
/// `sourceKey` means "embedded, never a part", which is a different claim.
/// An empty key or a key that is already a revision key is left alone.
pub fn rewrite_source_keys(
    document: &mut Value,
    to_key: &dyn Fn(&str) -> Option<String>,
) -> Vec<String> {
    let mut unknown = Vec::new();
    rewrite(document, to_key, &mut unknown);
    unknown
}

fn rewrite(document: &mut Value, to_key: &dyn Fn(&str) -> Option<String>, unknown: &mut Vec<String>) {
    let Some(library) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return;
    };
    for entry in library.values_mut() {
        let key = entry
            .get("sourceKey")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_string();
        if !key.is_empty() && DocumentIdentity::parse_revision_key(&key).is_none() {
            match to_key(&key) {
                Some(revision_key) => entry["sourceKey"] = Value::String(revision_key),
                None => unknown.push(key),
            }
        }
        if let Some(nested) = entry.get_mut("document") {
            rewrite(nested, to_key, unknown);
        }
    }
}

/// How far one document has got. Persisted between runs; this is what makes
/// the import restartable.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LedgerEntry {
    /// What the server minted for it.
    pub part: String,
    pub revision: String,
    /// The content signature it was minted from.
    pub signature: String,
    #[serde(default)]
    pub uploaded: bool,
    #[serde(default)]
    pub published: bool,
}

/// The import's memory, by canonical identity.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct Ledger {
    pub entries: BTreeMap<String, LedgerEntry>,
}

impl Ledger {
    /// The revision key the import minted for a document.
    pub fn revision_key(&self, identity: &str) -> Option<String> {
        self.entries.get(identity).map(|entry| {
            DocumentIdentity::Revision {
                part: entry.part.clone(),
                revision: entry.revision.clone(),
            }
            .key()
        })
    }

    pub fn record_minted(&mut self, part: &PlannedPart, part_id: &str, revision_id: &str) {
        self.entries.insert(
            part.identity.clone(),
            LedgerEntry {
                part: part_id.into(),
                revision: revision_id.into(),
                signature: part.signature.clone(),
                uploaded: false,
                published: false,
            },
        );
    }

    pub fn record_uploaded(&mut self, identity: &str) {
        if let Some(entry) = self.entries.get_mut(identity) {
            entry.uploaded = true;
        }
    }

    pub fn record_published(&mut self, identity: &str) {
        if let Some(entry) = self.entries.get_mut(identity) {
            entry.published = true;
        }
    }
}

/// Every import's ledger on this machine, under the reserved Local key
/// `@plm_import`: by PLM server, then by the folder imported. A ledger names
/// part and revision ids that exist on ONE server, so an import of the same
/// folder against another server starts from nothing rather than "resuming"
/// with ids that server never minted.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct ImportLedgers {
    /// Server (normalised by [`server_key`]) → folder → ledger.
    pub servers: BTreeMap<String, BTreeMap<String, Ledger>>,
}

/// A server URL as a ledger key: scheme and host lowercased, no trailing
/// slash, so `HTTPS://PLM.example.com/` and `https://plm.example.com` are one
/// server. The path is kept: two PLMs may be served under one host.
pub fn server_key(url: &str) -> String {
    let url = url.trim().trim_end_matches('/');
    match url.split_once("://") {
        Some((scheme, rest)) => {
            let (host, path) = rest.split_once('/').map_or((rest, ""), |(h, p)| (h, p));
            let mut key = format!("{}://{}", scheme.to_ascii_lowercase(), host.to_ascii_lowercase());
            if !path.is_empty() {
                key.push('/');
                key.push_str(path);
            }
            key
        }
        None => url.to_ascii_lowercase(),
    }
}

impl ImportLedgers {
    /// What the store holds; empty when nothing (or something unreadable) is.
    pub fn load(store: &dyn crate::store::ModelStore) -> Self {
        store
            .read(crate::store::PLM_IMPORT_KEY)
            .and_then(|text| serde_json::from_str(&text).ok())
            .unwrap_or_default()
    }

    /// The ledger for importing `folder` into `server`; empty if never run.
    pub fn ledger(&self, server: &str, folder: &str) -> Ledger {
        self.servers
            .get(&server_key(server))
            .and_then(|folders| folders.get(folder))
            .cloned()
            .unwrap_or_default()
    }

    /// Record `ledger` for `folder` on `server` and write the whole record.
    pub fn save(
        &mut self,
        store: &dyn crate::store::ModelStore,
        server: &str,
        folder: &str,
        ledger: &Ledger,
    ) -> Result<(), String> {
        self.servers
            .entry(server_key(server))
            .or_default()
            .insert(folder.to_string(), ledger.clone());
        let text = serde_json::to_string(self).map_err(|error| error.to_string())?;
        store.write(crate::store::PLM_IMPORT_KEY, &text)
    }
}

/// The next thing an import has to do.
#[derive(Clone, Debug, PartialEq)]
pub enum Step {
    /// Create a part (and its first revision) for this document.
    Mint(PlannedPart),
    /// Upload the document, rewritten, to the revision the ledger names.
    Upload {
        identity: String,
        key: String,
        document: String,
        /// `sourceKey`s the rewrite could not map (all of them in
        /// [`AdoptionPlan::external`]).
        unknown: Vec<String>,
    },
    /// Publish an assembly's uses list: the `PUT …/uses` body, or why there
    /// is none (an occurrence naming no minted part).
    PublishUses {
        identity: String,
        key: String,
        body: Result<Value, String>,
    },
}

/// The first step `plan` still needs, given `ledger`; `None` when the import
/// is complete. `documents` are the same ones the plan was made from.
///
/// A document whose signature has changed since it was minted is NOT
/// re-uploaded: that would be a new revision of a part that may already be
/// released, which is the ordinary revision workflow and not adoption. It is
/// reported by [`changed_since`].
pub fn next_step(
    plan: &AdoptionPlan,
    documents: &[SourceDocument],
    ledger: &Ledger,
    canonical: &dyn Fn(&str) -> String,
) -> Option<Step> {
    if let Some(part) = plan.parts.iter().find(|p| !ledger.entries.contains_key(&p.identity)) {
        return Some(Step::Mint(part.clone()));
    }
    let contents = |identity: &str| {
        documents
            .iter()
            .find(|d| d.identity == identity)
            .map(|d| d.contents.as_str())
    };
    let to_key = |key: &str| ledger.revision_key(&canonical(key));
    for part in &plan.parts {
        let entry = &ledger.entries[&part.identity];
        if entry.uploaded {
            continue;
        }
        let mut document: Value = serde_json::from_str(contents(&part.identity)?).ok()?;
        let unknown = rewrite_source_keys(&mut document, &to_key);
        return Some(Step::Upload {
            identity: part.identity.clone(),
            key: ledger.revision_key(&part.identity)?,
            document: document.to_string(),
            unknown,
        });
    }
    for part in plan.parts.iter().filter(|p| p.assembly) {
        if ledger.entries[&part.identity].published {
            continue;
        }
        let mut document: Value = serde_json::from_str(contents(&part.identity)?).ok()?;
        rewrite_source_keys(&mut document, &to_key);
        let resolve = |key: &str| {
            DocumentIdentity::parse_revision_key(key)
                .unwrap_or_else(|| DocumentIdentity::Path(key.to_string()))
        };
        return Some(Step::PublishUses {
            identity: part.identity.clone(),
            key: ledger.revision_key(&part.identity)?,
            body: uses::uses_lines(&uses::occurrences(&document), &resolve).body(),
        });
    }
    None
}

/// Documents whose content changed after the import minted them.
pub fn changed_since(plan: &AdoptionPlan, ledger: &Ledger) -> Vec<String> {
    plan.parts
        .iter()
        .filter(|part| {
            ledger
                .entries
                .get(&part.identity)
                .is_some_and(|entry| entry.signature != part.signature)
        })
        .map(|part| part.identity.clone())
        .collect()
}

// --- reading the tree -------------------------------------------------------------

/// Every model document under the explorer location `root`, folders
/// included, read through the store the user browses — the same identities
/// the explorer shows, so a `sourceKey` picked there matches. The explorer's
/// location is restored afterwards. Nothing is written.
pub fn walk_tree(store: &dyn crate::store::ModelStore, root: &str) -> Result<Vec<SourceDocument>, String> {
    let before = store.browser_location();
    let walked = walk(store, root);
    let _ = store.browser_navigate(&before);
    walked
}

fn walk(store: &dyn crate::store::ModelStore, root: &str) -> Result<Vec<SourceDocument>, String> {
    let mut out = Vec::new();
    let mut folders = vec![root.to_string()];
    let mut seen = BTreeSet::new();
    while let Some(folder) = folders.pop() {
        if !seen.insert(folder.clone()) {
            continue;
        }
        store.browser_navigate(&folder)?;
        for entry in store.browser_entries(crate::store::MODEL_EXTENSIONS) {
            if entry.is_dir {
                folders.push(entry.identity);
            } else {
                let contents = store
                    .read(&entry.identity)
                    .ok_or_else(|| format!("'{}' could not be read", entry.identity))?;
                out.push(SourceDocument { identity: entry.identity, contents });
            }
        }
    }
    out.sort_by(|a, b| a.identity.cmp(&b.identity));
    Ok(out)
}

// --- the importer, over the PLM client ----------------------------------------------

/// How an imported part is numbered.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ImportNumbering {
    /// The part type every document is minted as.
    pub part_type: String,
    /// Send the file's name as the number: for a free or pattern type, whose
    /// numbers must be typed. A counter type refuses one, so this is `false`
    /// for it and the counter numbers each part.
    pub number_from_name: bool,
}

/// What an import run did.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ImportReport {
    pub minted: usize,
    pub uploaded: usize,
    pub published: usize,
    /// `(document, sentence)` for every step the server refused. The run stops
    /// at the first; the ledger holds everything before it, so fixing the
    /// cause and running again resumes there.
    pub refused: Vec<(String, String)>,
}

/// The name this import checks revisions out under, so a lock left by a run
/// killed mid-step says what holds it.
const CLIENT_ID: &str = "brep-app import";

async fn with_checkout(
    client: &crate::plm::client::PlmClient,
    key: &str,
    write: impl std::future::Future<Output = Result<(), String>>,
) -> Result<(), String> {
    let Some(DocumentIdentity::Revision { part, revision }) = DocumentIdentity::parse_revision_key(key) else {
        return Err(format!("'{key}' is not a PLM revision"));
    };
    let base = format!("/api/parts/{part}/revisions/{revision}");
    let body = |value: Value| Some(serde_json::to_vec(&value).unwrap_or_default());
    // Checking out a revision this user already holds succeeds, so a run
    // killed between checkout and check-in resumes cleanly.
    client
        .call("POST", &format!("{base}/checkout"), body(serde_json::json!({ "client_id": CLIENT_ID })))
        .await
        .map_err(|error| error.to_string())?;
    let written = write.await;
    // Check in whatever happened: a refused write must not leave the draft
    // locked against the person who will fix it.
    let checked_in = client
        .call("POST", &format!("{base}/checkin"), body(serde_json::json!({})))
        .await
        .map_err(|error| error.to_string());
    written.and(checked_in.map(|_| ()))
}

/// Load `contents` — a file family (`.fbrep`), template (`.tbrep`) or any
/// model document — into the existing revision `key` (`part/…/rev/…`): check
/// it out, write it, check it in. This is what the web page's "load a file
/// into this revision" did; the CAD app does it from the file it has open.
/// The server's refusal (a released revision, someone else's lock, a document
/// that is not JSON) comes back as its sentence.
pub async fn load_into_revision(
    client: &crate::plm::client::PlmClient,
    key: &str,
    contents: &str,
) -> Result<(), String> {
    if serde_json::from_str::<Value>(contents).is_err() {
        return Err("the file is not a model document (it does not parse as JSON)".into());
    }
    let path = format!("/api/store/doc/{key}");
    let bytes = contents.as_bytes().to_vec();
    with_checkout(client, key, async {
        client.call("PUT", &path, Some(bytes)).await.map(|_| ()).map_err(|e| e.to_string())
    })
    .await
}

/// Run an adoption against the PLM `client` is signed in to, from wherever
/// `ledger` says it got to. `persist` is called after every step that moved
/// the ledger, so a run killed at any point loses at most the step in flight
/// (and that step is safe to repeat, except a mint whose answer never arrived,
/// which leaves one unused part — the one thing a restart cannot know). It
/// answers whether to go on: `false` is the user's Stop, and the next run
/// resumes from the ledger it was just handed.
pub async fn run_import(
    client: &crate::plm::client::PlmClient,
    plan: &AdoptionPlan,
    documents: &[SourceDocument],
    ledger: &mut Ledger,
    canonical: &dyn Fn(&str) -> String,
    numbering: &ImportNumbering,
    persist: &mut dyn FnMut(&Ledger) -> bool,
) -> ImportReport {
    let mut report = ImportReport::default();
    while let Some(step) = next_step(plan, documents, ledger, canonical) {
        let outcome: Result<(), String> = match &step {
            Step::Mint(part) => {
                let request = serde_json::json!({
                    "part_type": numbering.part_type,
                    "name": part.name,
                    "number": if numbering.number_from_name { part.name.clone() } else { String::new() },
                    "document_class": part.class.slug(),
                    // A bulk import: the server links none of these parts into
                    // the workspace (`brep_plm::workspace::link_on_create`).
                    "bulk": true,
                });
                match client
                    .call("POST", "/api/parts", Some(serde_json::to_vec(&request).unwrap_or_default()))
                    .await
                {
                    Ok(response) => {
                        let created: Value = serde_json::from_slice(&response.body).unwrap_or_default();
                        let id = created["id"].as_str().unwrap_or_default();
                        let revision = created["revisions"][0]["id"].as_str().unwrap_or_default();
                        if id.is_empty() || revision.is_empty() {
                            Err("the PLM created the part but did not say its revision".to_string())
                        } else {
                            ledger.record_minted(part, id, revision);
                            report.minted += 1;
                            Ok(())
                        }
                    }
                    Err(error) => Err(error.to_string()),
                }
            }
            Step::Upload { identity, key, document, .. } => {
                let bytes = document.clone().into_bytes();
                let path = format!("/api/store/doc/{key}");
                let written = with_checkout(client, key, async {
                    client.call("PUT", &path, Some(bytes)).await.map(|_| ()).map_err(|e| e.to_string())
                })
                .await;
                written.map(|()| {
                    ledger.record_uploaded(identity);
                    report.uploaded += 1;
                })
            }
            Step::PublishUses { identity, key, body } => match body {
                Err(refusal) => Err(refusal.clone()),
                Ok(body) => {
                    let Some(DocumentIdentity::Revision { part, revision }) = DocumentIdentity::parse_revision_key(key)
                    else {
                        return report;
                    };
                    let path = format!("/api/parts/{part}/revisions/{revision}/uses");
                    let bytes = serde_json::to_vec(body).unwrap_or_default();
                    let written = with_checkout(client, key, async {
                        client.call("PUT", &path, Some(bytes)).await.map(|_| ()).map_err(|e| e.to_string())
                    })
                    .await;
                    written.map(|()| {
                        ledger.record_published(identity);
                        report.published += 1;
                    })
                }
            },
        };
        match outcome {
            Ok(()) => {
                if !persist(ledger) {
                    return report;
                }
            }
            Err(sentence) => {
                let identity = match &step {
                    Step::Mint(part) => part.identity.clone(),
                    Step::Upload { identity, .. } | Step::PublishUses { identity, .. } => identity.clone(),
                };
                report.refused.push((identity, sentence));
                return report;
            }
        }
    }
    report
}

