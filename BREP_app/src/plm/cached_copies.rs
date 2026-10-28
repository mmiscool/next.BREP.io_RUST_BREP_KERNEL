//! An assembly's embedded copy of a PLM part is a CACHE of that part's
//! revision (integrator ruling 2026-10-21, nested-assembly import): when a
//! sub-assembly's draft is rewritten in place (same revision key), the parents
//! that place it are not rewritten, so their copies can be older than the
//! item. A reader resolves the keyed revision first and uses the copy only when
//! the revision cannot be read.
//!
//! This is the CAD reader: on opening a document from the PLM, every embedded
//! copy keyed to a PLM revision, at every depth, is compared with that
//! revision's document and replaced when they differ ([`refresh`](crate::plm::cached_copies::refresh)), as one edit
//! with one notice. File-keyed copies are left to Update Components and its
//! outdated badge, as before.
//!
//! A copy whose text is the very bytes the PLM stores for its revision (its
//! SHA-256 is the `content_hash` the resident index lists) is that revision,
//! so it stands in for it and the open does not read it ([`unchanged`](crate::plm::cached_copies::unchanged)).
use crate::plm::client::PlmClient;
use crate::plm::uses::revision_key_in;
use serde_json::Value;
use std::collections::HashMap;

/// How deep a refresh follows nested copies (the import planner's bound).
const MAX_DEPTH: usize = 64;
/// How many distinct revisions one open may read.
const MAX_SOURCES: usize = 4096;

/// The PLM revision keys of every embedded copy in `document`, at any depth.
fn keys_in(document: &Value, depth: usize, out: &mut Vec<String>) {
    let Some(entries) = document["partsLibrary"].as_object() else {
        return;
    };
    if depth > MAX_DEPTH {
        return;
    }
    for entry in entries.values() {
        if let Some(key) = entry["sourceKey"].as_str().and_then(revision_key_in) {
            out.push(key);
        }
        keys_in(&entry["document"], depth + 1, out);
    }
}

/// The revisions EVERY copy of which in `document`, at any depth, is the
/// very bytes the PLM stores for it (`hash_of`: the index's `content_hash`,
/// `None` when the index does not list it), each with one of its copies: an
/// open takes the copy for the revision instead of reading it. A copy laid
/// out otherwise (another writer's bytes, a server normalisation) is read.
pub fn unchanged(document: &Value, hash_of: &dyn Fn(&str) -> Option<String>) -> HashMap<String, Value> {
    fn walk(
        document: &Value,
        hash_of: &dyn Fn(&str) -> Option<String>,
        depth: usize,
        good: &mut HashMap<String, Value>,
        bad: &mut std::collections::HashSet<String>,
    ) {
        let Some(entries) = document["partsLibrary"].as_object() else {
            return;
        };
        if depth > MAX_DEPTH {
            return;
        }
        for entry in entries.values() {
            if let Some(key) = entry["sourceKey"].as_str().and_then(revision_key_in) {
                if !bad.contains(&key) {
                    let copy = &entry["document"];
                    let same = copy["features"].is_array()
                        && hash_of(&key).is_some_and(|hash| crate::plm::kicad::content_hash(copy.to_string().as_bytes()) == hash);
                    if same {
                        good.entry(key).or_insert_with(|| copy.clone());
                    } else {
                        good.remove(&key);
                        bad.insert(key);
                    }
                }
            }
            walk(&entry["document"], hash_of, depth + 1, good, bad);
        }
    }
    let (mut good, mut bad) = (HashMap::new(), std::collections::HashSet::new());
    walk(document, hash_of, 0, &mut good, &mut bad);
    good
}

/// How many revision reads one open keeps in flight at once.
const IN_FLIGHT: usize = 16;

/// Every future of `futures`, polled together, in order. (The app has no
/// futures crate; this is the one join it needs.)
type Read<'a, T> = std::pin::Pin<Box<dyn std::future::Future<Output = T> + 'a>>;

struct JoinAll<'a, T> {
    futures: Vec<Option<Read<'a, T>>>,
    out: Vec<Option<T>>,
}

// The outputs are moved, never pinned; the futures are boxed.
impl<T> Unpin for JoinAll<'_, T> {}

impl<T> std::future::Future for JoinAll<'_, T> {
    type Output = Vec<T>;
    fn poll(self: std::pin::Pin<&mut Self>, cx: &mut std::task::Context<'_>) -> std::task::Poll<Vec<T>> {
        let this = self.get_mut();
        let mut pending = false;
        for (slot, out) in this.futures.iter_mut().zip(this.out.iter_mut()) {
            if let Some(future) = slot {
                match future.as_mut().poll(cx) {
                    std::task::Poll::Ready(value) => {
                        *out = Some(value);
                        *slot = None;
                    }
                    std::task::Poll::Pending => pending = true,
                }
            }
        }
        if pending {
            std::task::Poll::Pending
        } else {
            std::task::Poll::Ready(this.out.iter_mut().map(|o| o.take().expect("every future answered")).collect())
        }
    }
}

fn join_all<'a, T>(futures: Vec<Read<'a, T>>) -> JoinAll<'a, T> {
    let out = futures.iter().map(|_| None).collect();
    JoinAll { futures: futures.into_iter().map(Some).collect(), out }
}

/// Read the revision documents `document`'s copies name, and those THEIR
/// documents name. A revision with no document, or one that cannot be read,
/// is left out: its copy is then the fallback. Reads go out up to
/// `IN_FLIGHT` at a time, level by level, and each revision is read once
/// however many copies name it, so a 386-item tree costs one read per item
/// in a few round-trip times rather than 386 in a row.
pub async fn fetch_sources(client: &PlmClient, document: &Value) -> HashMap<String, Value> {
    fetch_changed(client, document, HashMap::new()).await
}

/// [`fetch_sources`], taking each of the `unchanged` revisions' copies for it
/// instead of reading it ([`unchanged`]). A copy that stands in this way is
/// also what a parent read from its revision takes at that revision, so the
/// parent's older embedded copy of it does not come back.
pub async fn fetch_changed(client: &PlmClient, document: &Value, unchanged: HashMap<String, Value>) -> HashMap<String, Value> {
    let mut queue = Vec::new();
    keys_in(document, 0, &mut queue);
    let mut tried: std::collections::HashSet<String> = unchanged.keys().cloned().collect();
    let mut sources = unchanged;
    while !queue.is_empty() && tried.len() < MAX_SOURCES {
        let mut wave = Vec::new();
        while let Some(key) = queue.pop() {
            if wave.len() == IN_FLIGHT || tried.len() >= MAX_SOURCES {
                queue.push(key);
                break;
            }
            if tried.insert(key.clone()) {
                wave.push(key);
            }
        }
        let reads: Vec<Read<'_, Result<Option<Vec<u8>>, crate::plm::client::PlmError>>> =
            wave.iter().map(|key| Box::pin(client.get_document(key)) as Read<'_, _>).collect();
        let answers = join_all(reads).await;
        for (key, answer) in wave.into_iter().zip(answers) {
            let Ok(Some(bytes)) = answer else {
                continue;
            };
            let Ok(source) = serde_json::from_slice::<Value>(&bytes) else {
                continue;
            };
            if !source["features"].is_array() {
                continue;
            }
            keys_in(&source, 0, &mut queue);
            sources.insert(key, source);
        }
    }
    sources
}

/// `value` without the fields a copy carries for itself (`sourceSignature`,
/// `snapshot`), so a copy and its revision compare on content alone.
fn content(value: &Value) -> Value {
    match value {
        Value::Object(map) => Value::Object(
            map.iter()
                .filter(|(k, _)| *k != "sourceSignature" && *k != "snapshot")
                .map(|(k, v)| (k.clone(), content(v)))
                .collect(),
        ),
        Value::Array(items) => Value::Array(items.iter().map(content).collect()),
        other => other.clone(),
    }
}

/// `source` with its own copies refreshed the same way.
fn resolved(source: &Value, sources: &HashMap<String, Value>, depth: usize) -> Value {
    let mut source = source.clone();
    refresh_at(&mut source, sources, depth + 1, "", &mut Vec::new());
    source
}

/// Put `current`'s copies of `held` revisions, at any depth, into `fresh` (a
/// copy rebuilt from its revision) where `fresh` places the same entry.
fn graft_held(fresh: &mut Value, current: &Value, held: &std::collections::BTreeSet<String>, depth: usize) {
    if depth > MAX_DEPTH || held.is_empty() {
        return;
    }
    let Some(entries) = fresh.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return;
    };
    for (name, entry) in entries.iter_mut() {
        let mine = &current["partsLibrary"][name];
        if !mine.is_object() {
            continue;
        }
        if entry["sourceKey"].as_str().and_then(revision_key_in).is_some_and(|key| held.contains(&key)) {
            *entry = mine.clone();
        } else {
            graft_held(&mut entry["document"], &mine["document"], held, depth + 1);
        }
    }
}

fn refresh_at(document: &mut Value, sources: &HashMap<String, Value>, depth: usize, trail: &str, out: &mut Vec<String>) {
    refresh_held(document, sources, depth, trail, out, &std::collections::BTreeSet::new());
}

/// [`refresh_at`], leaving alone the copies of `held` revisions: a Save could
/// not write their change, which lives only in this assembly
/// (`document::owned_save::UNSAVED_EDITS_BLOCK`).
fn refresh_held(
    document: &mut Value,
    sources: &HashMap<String, Value>,
    depth: usize,
    trail: &str,
    out: &mut Vec<String>,
    held: &std::collections::BTreeSet<String>,
) {
    if depth > MAX_DEPTH {
        return;
    }
    let Some(entries) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return;
    };
    for (name, entry) in entries.iter_mut() {
        let here = if trail.is_empty() { name.clone() } else { format!("{trail}/{name}") };
        let key = entry["sourceKey"].as_str().and_then(revision_key_in);
        if key.as_ref().is_some_and(|key| held.contains(key)) {
            // Its change is not in its revision yet: the copy IS the user's work.
            if entry["document"].is_object() {
                refresh_held(&mut entry["document"], sources, depth + 1, &here, out, held);
            }
            continue;
        }
        let source = key.and_then(|key| sources.get(&key));
        match source {
            Some(source) => {
                let mut fresh = resolved(source, sources, depth);
                // A held copy inside stays the user's: a parent taken from its
                // revision must not bring back the revision's older copy of it.
                graft_held(&mut fresh, &entry["document"], held, depth + 1);
                if content(&entry["document"]) != content(&fresh) {
                    entry["sourceSignature"] =
                        Value::String(brep_render::engine_state::document_signature(&fresh.to_string()));
                    entry["document"] = fresh;
                    entry["snapshot"] = Value::String(String::new());
                    out.push(here);
                }
            }
            // An unreadable or file-keyed copy stays, and copies inside it are
            // still refreshed: a deeper part may be on the PLM.
            None => {
                if entry["document"].is_object() {
                    refresh_held(&mut entry["document"], sources, depth + 1, &here, out, held);
                }
            }
        }
    }
}

/// Replace every copy in `document` that differs from its PLM revision in
/// `sources`, at any depth. Returns the copies replaced, as library paths
/// (`sub-b`, `sub-a/washer`); empty means the document is current.
pub fn refresh(document: &mut Value, sources: &HashMap<String, Value>) -> Vec<String> {
    let held = held_revisions(document);
    let mut out = Vec::new();
    refresh_held(document, sources, 0, "", &mut out, &held);
    out
}

/// [`refresh`], also leaving alone the copies of `edited` revisions: copies the
/// user changed since the tab was last clean, whose change no Save has written
/// yet. A refresh while the document is open must not take their work.
pub fn refresh_except(document: &mut Value, sources: &HashMap<String, Value>, edited: &std::collections::BTreeSet<String>) -> Vec<String> {
    let mut held = held_revisions(document);
    held.extend(edited.iter().cloned());
    let mut out = Vec::new();
    refresh_held(document, sources, 0, "", &mut out, &held);
    out
}

/// The PLM revision keys of every embedded copy in `document`, at any depth.
pub fn revision_keys(document: &Value) -> Vec<String> {
    let mut out = Vec::new();
    keys_in(document, 0, &mut out);
    out
}

/// The revisions in `sources` whose own content (their nested copies left
/// out) differs from their copies in `document`, at any depth: the parts that
/// changed on the PLM since `document` (a tab's clean baseline) was taken, as
/// opposed to the holders a refresh replaces because of them.
pub fn moved_revisions(document: &Value, sources: &HashMap<String, Value>) -> std::collections::BTreeSet<String> {
    fn own(document: &Value) -> Value {
        let mut own = content(document);
        if let Some(map) = own.as_object_mut() {
            map.remove("partsLibrary");
        }
        own
    }
    fn walk(document: &Value, sources: &HashMap<String, Value>, depth: usize, out: &mut std::collections::BTreeSet<String>) {
        let Some(entries) = document["partsLibrary"].as_object() else {
            return;
        };
        if depth > MAX_DEPTH {
            return;
        }
        for entry in entries.values() {
            if let Some(key) = entry["sourceKey"].as_str().and_then(revision_key_in) {
                if sources.get(&key).is_some_and(|source| own(source) != own(&entry["document"])) {
                    out.insert(key);
                }
            }
            walk(&entry["document"], sources, depth + 1, out);
        }
    }
    let mut out = std::collections::BTreeSet::new();
    walk(document, sources, 0, &mut out);
    out
}

/// The library paths (`sub-b/plate`) of the copies in `document` keyed to one
/// of the revisions `keys`, at any depth: what a notice names as changed.
pub fn paths_of(document: &Value, keys: &std::collections::BTreeSet<String>) -> Vec<String> {
    fn walk(document: &Value, keys: &std::collections::BTreeSet<String>, trail: &str, depth: usize, out: &mut Vec<String>) {
        let Some(entries) = document["partsLibrary"].as_object() else {
            return;
        };
        if depth > MAX_DEPTH {
            return;
        }
        for (name, entry) in entries {
            let here = if trail.is_empty() { name.clone() } else { format!("{trail}/{name}") };
            if entry["sourceKey"].as_str().and_then(revision_key_in).is_some_and(|key| keys.contains(&key)) {
                out.push(here.clone());
            }
            walk(&entry["document"], keys, &here, depth + 1, out);
        }
    }
    let mut out = Vec::new();
    walk(document, keys, "", 0, &mut out);
    out
}

/// The revisions whose copies in `document` hold a change a Save could not
/// write (`document::owned_save::UNSAVED_EDITS_BLOCK`), as store keys.
pub fn held_revisions(document: &Value) -> std::collections::BTreeSet<String> {
    crate::document::owned_save::unsaved_edits(document).into_keys().filter_map(|key| revision_key_in(&key)).collect()
}

/// What a person can do about a change kept in the assembly because `why`.
pub fn way_forward(why: &str) -> String {
    if let Some(user) = why.strip_prefix("checked out by ") {
        format!("ask {user} to check it in, then Save again")
    } else if why.contains("released") {
        "it is released: make a new revision of it, then Save again".into()
    } else if why.contains("still loading") {
        "Save again once it has loaded".into()
    } else {
        "Save again to write it".into()
    }
}

/// `text` with its PLM copies refreshed from their revisions, for a reader
/// that builds the document without keeping it (the bake worker). A document
/// that does not parse, or is current, comes back as it was.
pub async fn with_current_copies(client: &PlmClient, text: &str) -> String {
    let Ok(mut document) = serde_json::from_str::<Value>(text) else {
        return text.to_string();
    };
    let sources = fetch_sources(client, &document).await;
    if refresh(&mut document, &sources).is_empty() {
        text.to_string()
    } else {
        document.to_string()
    }
}

