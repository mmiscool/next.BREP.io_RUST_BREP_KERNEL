//! LIVE LINKS between open documents (owner, 2026-10-20): a part edited in its
//! own tab — saved or not — and the same part edited inside an assembly are one
//! part. Every open document that holds a source (a tab named by it, or an
//! assembly embedding a copy of it at any depth) sees the newest state of that
//! source when the user switches to it.
//!
//! The session keeps ONE current state per source key, in memory only (no
//! file is written and nothing new is saved in a document):
//! * LEAVING a tab PUBLISHES every source it changed since it last synced:
//!   its own document when it has a store name, and each embedded copy with a
//!   `sourceKey`. "Changed" is measured against what the tab last pulled or
//!   published, or — before that — its own Open/Save baseline, so a stale copy
//!   in an assembly nobody touched never wins over a newer part.
//! * ENTERING a tab PULLS every source whose current state it does not show.
//!   An assembly takes it the way a Work part edit does (every copy, at every
//!   depth; one undo step; the assembly re-runs and its mates re-solve; lost
//!   references are named); a part tab takes it as one undoable edit. Either
//!   tab is then dirty.
//!
//! Only the active tab can change anything, so two tabs never hold two
//! different states of one source: the one left last is the newest. Undo is
//! per tab and is an edit like any other — an undo of a pulled change is
//! published back on the next switch.
use super::owned_save::{source_hash, writable};
use super::Documents;
use serde_json::Value;
use std::collections::HashMap;

/// The document blocks that make a document an eCAD sheet or board. Such a
/// document takes a part's newer state like any assembly (owner,
/// 2026-10-21: one behaviour everywhere), and says so in a notice: a pin
/// renumbered in another tab re-pins the board, which must not be silent.
const ECAD_BLOCKS: [&str; 2] = ["diagram", "pcb"];

/// The newest state of each source, and what each tab last synced.
#[derive(Default)]
pub(super) struct LiveLinks {
    /// key → (hash, document text).
    current: HashMap<String, (u64, String)>,
    /// (tab id, key) → the hash that tab last published or pulled.
    seen: HashMap<(u64, String), u64>,
    /// Tab id → its [`PublishKey`] when it last published: a tab left again
    /// unchanged has nothing new to publish, and is not walked or hashed.
    published: HashMap<u64, PublishKey>,
    /// How many publishes walked a document (the work count the tests read).
    walks: u64,
}

/// Everything [`Documents::publish_live`] reads from a tab: its document (run
/// write-backs, edits, undo and redo, object metadata), the Open/Save baseline
/// its copies are measured against, and the name its own copy is keyed by.
/// What it reads from the session, the tab's `seen` hashes, is moved only by
/// this tab's own publish and pull, and a pull that moves one forgets the key.
#[derive(Clone, PartialEq)]
struct PublishKey {
    edit: (u64, u64),
    metadata: u64,
    baseline: u64,
    name: Option<String>,
}

fn publish_key(doc: &super::Document) -> PublishKey {
    PublishKey {
        edit: doc.edit_key(),
        metadata: doc.engine.metadata.stamp(),
        baseline: doc.baseline_generation,
        name: doc.name().map(str::to_string),
    }
}

/// One source a document holds: `(key, current document, baseline hash)`.
struct Copy {
    /// The store's spelling of the source (the comparison key).
    key: String,
    /// The `sourceKey` as this document spells it.
    raw: String,
    document: Value,
    baseline: Option<u64>,
}

/// Every source `current` holds as an embedded copy (first copy of each key,
/// outermost first), with the hash of the same copy in `baseline`.
fn embedded_copies(canon: &dyn Fn(&str) -> String, baseline: Option<&Value>, current: &Value, out: &mut Vec<Copy>) {
    let Some(entries) = current["partsLibrary"].as_object() else {
        return;
    };
    for (name, entry) in entries {
        let base = baseline.map(|b| &b["partsLibrary"][name]).filter(|b| b.is_object());
        let raw = entry["sourceKey"].as_str().unwrap_or("");
        let key = canon(raw);
        if writable(raw) && entry["document"].is_object() && !out.iter().any(|c| c.key == key) {
            out.push(Copy {
                raw: raw.to_string(),
                key,
                document: entry["document"].clone(),
                baseline: base.map(|b| source_hash(&b["document"])),
            });
        }
        embedded_copies(canon, base.map(|b| &b["document"]), &entry["document"], out);
    }
}

/// Point every copy of `old` (compared through `canon`) at `new_key` with the
/// body `body` and the signature `signature`, at any depth. The count changed.
fn retarget(document: &mut Value, canon: &dyn Fn(&str) -> String, old: &str, new_key: &str, body: &Value, signature: &str) -> usize {
    let Some(entries) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return 0;
    };
    let mut changed = 0;
    for entry in entries.values_mut() {
        if let Some(child) = entry.get_mut("document") {
            changed += retarget(child, canon, old, new_key, body, signature);
        }
        let key = entry["sourceKey"].as_str().unwrap_or("");
        if writable(key) && canon(key) == canon(old) {
            entry["sourceKey"] = Value::String(new_key.to_string());
            entry["sourceSignature"] = Value::String(signature.to_string());
            entry["document"] = body.clone();
            changed += 1;
        }
    }
    changed
}

impl Documents {
    /// The open tabs other than `except` whose documents place a copy of
    /// `key` at any depth: `(tab id, title)`. A part tab saved under a new
    /// name asks whether these should follow it.
    pub fn tabs_holding(&self, key: &str, except: u64) -> Vec<(u64, String)> {
        let canon = self.canon.clone();
        self.open
            .iter()
            .filter(|doc| doc.id() != except)
            .filter(|doc| {
                let mut document: Value = serde_json::from_str(&doc.engine.history_request_json()).unwrap_or(Value::Null);
                retarget(&mut document, &*canon, key, "", &Value::Null, "") > 0
            })
            .map(|doc| (doc.id(), doc.title()))
            .collect()
    }

    /// Point every copy of `old` in tab `id` at the document `text` saved as
    /// `new_key`: one undoable edit, after which the tab is unsaved and its
    /// next Save writes the new reference. The number of copies moved.
    pub fn retarget_copies(&mut self, id: u64, old: &str, new_key: &str, text: &str) -> Result<usize, String> {
        let canon = self.canon.clone();
        let doc = self.open.iter_mut().find(|doc| doc.id() == id).ok_or("that tab is closed")?;
        let mut document: Value = serde_json::from_str(&doc.engine.history_request_json()).map_err(|e| e.to_string())?;
        let body: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        let signature = crate::panels::parts_library::document_signature(text);
        let moved = retarget(&mut document, &*canon, old, new_key, &body, &signature);
        if moved > 0 {
            doc.engine.edit_document_json(&document.to_string())?;
        }
        Ok(moved)
    }
}

impl Documents {
    /// The sources tab `index` holds: its own document (when it has a store
    /// name) and its embedded copies.
    fn sources_of(&self, index: usize) -> (Vec<Copy>, Value) {
        let doc = &self.open[index];
        let current: Value = serde_json::from_str(&doc.engine.history_request_json()).unwrap_or(Value::Null);
        let mut baseline: Option<Value> = doc.saved_signature.as_deref().and_then(|s| serde_json::from_str(s).ok());
        // The baseline is the SAVED form, which carries `metadata` beside the
        // request document; the request document does not.
        if let Some(object) = baseline.as_mut().and_then(Value::as_object_mut) {
            if current.get("metadata").is_none() {
                object.remove("metadata");
            }
        }
        let mut copies = Vec::new();
        if let Some(name) = doc.name().filter(|name| writable(name)) {
            copies.push(Copy {
                key: (self.canon)(name),
                raw: name.to_string(),
                document: current.clone(),
                baseline: baseline.as_ref().map(source_hash),
            });
        }
        embedded_copies(self.canon.as_ref(), baseline.as_ref(), &current, &mut copies);
        (copies, current)
    }

    /// Publish every source tab `index` changed since it last synced.
    pub(super) fn publish_live(&mut self, index: usize) {
        if index >= self.open.len() {
            return;
        }
        let id = self.open[index].id();
        let key = publish_key(&self.open[index]);
        if self.live.published.get(&id) == Some(&key) {
            return;
        }
        // A tab that never synced measures every copy against its baseline;
        // while it still IS that baseline, nothing differs.
        if self.open[index].is_its_baseline() && !self.live.seen.keys().any(|(tab, _)| *tab == id) {
            self.live.published.insert(id, key);
            return;
        }
        self.live.walks += 1;
        let (copies, _) = self.sources_of(index);
        self.live.published.insert(id, key);
        for copy in copies {
            let hash = source_hash(&copy.document);
            let reference = self.live.seen.get(&(id, copy.key.clone())).copied().or(copy.baseline);
            if reference != Some(hash) {
                self.live.current.insert(copy.key.clone(), (hash, copy.document.to_string()));
                self.live.seen.insert((id, copy.key), hash);
            }
        }
    }

    /// Bring tab `index` up to the current state of every source it holds.
    pub(super) fn pull_live(&mut self, index: usize) {
        if index >= self.open.len() || self.live.current.is_empty() {
            return;
        }
        let id = self.open[index].id();
        let own = self.open[index].name().map(|name| (self.canon)(name));
        let (copies, mut document) = self.sources_of(index);
        let ecad = ECAD_BLOCKS.iter().any(|key| document.get(*key).is_some_and(|block| !block.is_null()));
        let mut pulled_parts = Vec::new();
        let mut own_state = None;
        let mut embedded_changed = false;
        let mut notes = Vec::new();
        for copy in copies {
            let Some((hash, text)) = self.live.current.get(&copy.key).cloned() else {
                continue;
            };
            let is_own = own.as_deref() == Some(copy.key.as_str());
            // A pull that moves this tab's `seen` walks its next publish.
            if self.live.seen.insert((id, copy.key.clone()), hash) != Some(hash) {
                self.live.published.remove(&id);
            }
            if source_hash(&copy.document) == hash {
                continue;
            }
            if is_own {
                own_state = Some(text);
                continue;
            }
            pulled_parts.push(crate::store::model_display_name(&copy.raw));
            let Ok(state) = serde_json::from_str::<Value>(&text) else {
                continue;
            };
            match super::work_part::refresh_parts(&mut document, &[], Some(&copy.raw), &state, &mut notes) {
                Ok(changed) => embedded_changed |= changed,
                Err(error) => notes.push(format!("'{}' not updated: {error}", copy.key)),
            }
        }
        let engine = &mut self.open[index].engine;
        // A tab IS the source it is named by; its own state, when newer, is
        // the whole document (its embedded copies come with it).
        let result = match (own_state, embedded_changed) {
            (Some(text), _) => engine.edit_document_json(&text).map(drop),
            (None, true) => engine.edit_document_json(&document.to_string()).map(drop),
            (None, false) => Ok(()),
        };
        if let Err(error) = result {
            engine.push_notice(format!("Linked documents: could not take the newer state: {error}"));
        }
        if ecad && !pulled_parts.is_empty() {
            // A board or sheet changed by an edit made in another tab: never
            // silently. The refresh notes name what it carried over (point
            // renames by position) and what it could not keep bound.
            engine.push_notice_as(
                brep_render::engine_state::NoticeSeverity::Warning,
                format!(
                    "Linked documents: this board took the newer {} from its open tab{} — Undo takes it back",
                    pulled_parts.join(", "),
                    if notes.is_empty() { String::new() } else { format!(": {}", notes.join("; ")) }
                ),
            );
            notes.clear();
        }
        for note in notes {
            engine.push_notice(format!("Linked documents: {note}"));
        }
    }

    /// Forget what a closed tab synced.
    pub(super) fn forget_live(&mut self, id: u64) {
        self.live.seen.retain(|(tab, _), _| *tab != id);
        self.live.published.remove(&id);
    }

    /// A new canonicalizer can spell a source key differently: walk every tab
    /// on its next publish.
    pub(super) fn forget_published(&mut self) {
        self.live.published.clear();
    }
}

