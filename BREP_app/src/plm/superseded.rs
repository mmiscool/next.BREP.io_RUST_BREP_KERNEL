//! Embedded copies pinned to a PLM revision that is no longer current: the
//! revision a copy names was superseded (a later one was released) or made
//! obsolete. A released assembly pinning its revisions is correct by design;
//! an assembly still being worked on is offered the current release
//! (`panels::plm_import::SupersededCheck`). These are the pure halves: which
//! revisions a document names, where they are placed, and which revision of a
//! part is current.
use crate::plm::uses::revision_key_in;
use serde_json::Value;

/// How deep the walks follow nested copies (the import planner's bound).
const MAX_DEPTH: usize = 64;

/// Whether a revision in `lifecycle` is no longer the one to build with.
pub fn is_superseded(lifecycle: &str) -> bool {
    matches!(lifecycle, "superseded" | "obsolete")
}

/// Every PLM revision key a copy in `document` names, at any depth, each once,
/// in the order first met.
pub fn pinned_keys(document: &Value) -> Vec<String> {
    fn walk(document: &Value, depth: usize, out: &mut Vec<String>) {
        let Some(entries) = document["partsLibrary"].as_object() else { return };
        if depth > MAX_DEPTH {
            return;
        }
        for entry in entries.values() {
            if let Some(key) = entry["sourceKey"].as_str().and_then(revision_key_in) {
                if !out.contains(&key) {
                    out.push(key);
                }
            }
            walk(&entry["document"], depth + 1, out);
        }
    }
    let mut out = Vec::new();
    walk(document, 0, &mut out);
    out
}

/// One placement of a revision: its occurrence path from the top
/// (`ACOMP1`, or `ACOMP3:ACOMP1` inside the sub-assembly ACOMP3 shows) and the
/// chain of part names that defines it (empty for the top's own placements;
/// `["sub-b"]` for one inside sub-b's definition, wherever sub-b is placed).
#[derive(Debug, Clone, PartialEq)]
pub struct Placement {
    pub path: String,
    pub owners: Vec<String>,
    /// The library entry it shows, in its owner's library.
    pub part: String,
}

impl Placement {
    /// As a library path a person reads (`sub-b/plate`).
    pub fn name(&self) -> String {
        self.owners.iter().chain(std::iter::once(&self.part)).cloned().collect::<Vec<_>>().join("/")
    }
}

/// Every placement (ACOMP occurrence) in `document` that shows the PLM
/// revision `key`, at any depth. Only PLACED copies: a library entry no
/// occurrence shows (one a Replace inside a sub-assembly left behind) is not
/// what the assembly uses.
pub fn occurrences_of(document: &Value, key: &str) -> Vec<Placement> {
    fn walk(document: &Value, key: &str, prefix: &str, owners: &[String], depth: usize, out: &mut Vec<Placement>) {
        if depth > MAX_DEPTH {
            return;
        }
        let library = &document["partsLibrary"];
        for feature in document["features"].as_array().into_iter().flatten() {
            if !matches!(feature["type"].as_str(), Some("ACOMP" | "ASSEMBLY COMPONENT")) {
                continue;
            }
            let (Some(id), Some(part)) = (feature["inputParams"]["id"].as_str(), feature["inputParams"]["partName"].as_str()) else {
                continue;
            };
            let path = if prefix.is_empty() { id.to_string() } else { format!("{prefix}:{id}") };
            let entry = &library[part];
            if entry["sourceKey"].as_str().and_then(revision_key_in).as_deref() == Some(key) {
                out.push(Placement { path, owners: owners.to_vec(), part: part.to_string() });
            } else if entry["document"].is_object() {
                let mut chain = owners.to_vec();
                chain.push(part.to_string());
                walk(&entry["document"], key, &path, &chain, depth + 1, out);
            }
        }
    }
    let mut out = Vec::new();
    walk(document, key, "", &[], 0, &mut out);
    out
}

/// The part's current release in a `GET /api/parts/:id` answer: the newest
/// released revision, as `(revision id, label)`.
pub fn current_release(part: &Value) -> Option<(String, String)> {
    part["revision_views"]
        .as_array()?
        .iter()
        .filter(|r| r["lifecycle"] == "released")
        .next_back()
        .and_then(|r| Some((r["id"].as_str()?.to_string(), r["label"].as_str()?.to_string())))
}

