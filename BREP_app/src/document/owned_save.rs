//! Save an assembly INTO the documents it owns (NX "Save" in an assembly).
//!
//! An assembly embeds a copy of every part and sub-assembly it places
//! (`partsLibrary.<name>.document`); a copy with a `sourceKey` is a copy of a
//! document of its own. Every edit made from inside the assembly changes the
//! embedded copy only: a Work part edit (at any depth, every copy with the
//! same source), a BOM `part.` cell, a BOM `occurrence.` cell of a NESTED
//! placement (which lives in the sub-assembly's definition), and an edit made
//! in the part's own open tab, which reaches the assembly through the live
//! link ([`super::live_link`]). The assembly's Save then writes every copy
//! that CHANGED back to the document it came from, so opening that document
//! alone shows the change.
//!
//! WHAT changed is one rule, not a list each edit path must remember to feed:
//! the assembly's CURRENT document against its clean BASELINE (the request JSON
//! as of the last Open/Save, the same baseline the dirty dot compares). A copy
//! whose document differs from the baseline's copy at the same place is
//! modified. That makes the written state the CURRENT one — an edit that was
//! undone back to the original writes nothing — and catches a sub-assembly
//! whose embedded part changed, so its own file does not keep a stale part.
//!
//! WRITE ORDER: deepest documents first (parts before the sub-assemblies that
//! embed them), then the assembly itself (the caller). Each written document's
//! `sourceSignature` is stamped on every copy of it before its parents are
//! read, so a parent's file and the assembly agree with what was written.
//!
//! NOTHING IS BLOCKED THAT THE USER CAN SAY YES TO (owner, 2026-10-20). A Save
//! is first PLANNED ([`Documents::plan_owned_save`]); a document that needs a
//! decision is a row of one Save dialog, every row defaulting to proceed:
//! * changed on disk since the assembly loaded it → overwrite it;
//! * a PLM item not checked out to this user → check it out, then save it
//!   through the same store write the PLM's own Save uses;
//! * gone from storage, or a family member → save it as a NEW file next to
//!   the old one; every copy in the assembly moves to the new file;
//! * two copies changed to different documents (only a hand-edited file can
//!   do that now: every in-app edit changes every copy of a source) → keep
//!   one (default: the deepest copy); every copy becomes that one.
//!
//! A row the user unticks is not written; its change stays in the assembly.
//! Only what nobody can say yes to is reported by name and skipped: a PLM item
//! checked out by someone else or not editable (released), or one still
//! loading. An open tab of a written document is marked clean when it shows
//! what was written, and reloaded when not.
//!
//! EVERY PLACED DOCUMENT HAS A FILE AFTER A SAVE (owner, 2026-10-21: "if a part
//! does not exist it should save to a new part file; never fall back to saving
//! in the assembly"). Changed or not, a copy is PROMOTED ([`Promotion`]):
//! * with no `sourceKey` (an import, a part made in context, an old document)
//!   → a new `<part name>.nbrep` in the assembly's folder, `-2`, `-3`, … when
//!   the name is taken;
//! * whose file is gone → re-created at its old key, so every reference to it
//!   stays valid; when that write fails, a new file in the assembly's folder.
//!
//! Each promotion relinks every copy and is written deepest first with the
//! rows, so the sub-assembly that holds a promoted part is written after it
//! and names its file (a row is added for it when it had no change of its
//! own). Nothing asks: the summary names each file made.
use super::Documents;
use crate::panels::parts_library::{document_signature, WriteOutcome};
use crate::store::{ModelStore, ReadNow};
use brep_render::brep_kernel::stable_json_hash;
use brep_render::engine_state::{EngineState, NoticeSeverity};
use serde_json::Value;

/// One owned document the assembly's Save will write.
#[derive(Debug, Clone, PartialEq)]
pub struct OwnedWrite {
    /// The store identity (`sourceKey`).
    pub key: String,
    /// The part-name chain to its deepest copy in the assembly.
    pub names: Vec<String>,
    /// How deep that copy sits (1 = a top-level component's document).
    pub depth: usize,
    /// The copy's CURRENT document (at plan time).
    pub text: String,
    /// The `sourceSignature` the baseline copy carried: what the assembly last
    /// synced with the file. A file with other content changed on disk.
    pub synced: String,
    /// Every DIFFERENT changed copy, deepest first, as `(names, document)`;
    /// more than one is a conflict.
    pub variants: Vec<(Vec<String>, String)>,
}

/// `(tab id, edit key, baseline generation)` of a cached answer, and the keys
/// ([`Documents::owned_modified_keys_of`]).
pub type OwnedCacheKey = ((u64, (u64, u64), u64), Vec<String>);

/// `((tab id, baseline generation), entry → synced signature)`
/// ([`Documents::synced_signatures`]).
pub type SyncedCache = ((u64, u64), std::collections::HashMap<String, String>);

/// What the assembly's Save would write: the changed owned documents,
/// deepest first.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct OwnedChanges {
    pub writes: Vec<OwnedWrite>,
}

/// A placed document the Save gives a file of its own ([module docs](self)).
#[derive(Debug, Clone, PartialEq)]
pub struct Promotion {
    /// The part-name chain to the copy (outermost first).
    pub names: Vec<String>,
    /// The key it had: `""` when it never had a file, else its gone file.
    pub old_key: String,
    /// Where it is written.
    pub new_key: String,
}

impl Promotion {
    /// Re-creating a missing file rather than making a new one.
    pub fn recreates(&self) -> bool {
        !self.old_key.is_empty() && self.old_key == self.new_key
    }
}

/// The decision one owned document needs before it is written.
#[derive(Debug, Clone, PartialEq)]
pub enum Issue {
    /// The file changed on disk since the assembly synced with it: write
    /// over it.
    Overwrite,
    /// A PLM revision not checked out to this user: check it out, then save.
    /// `newer`: the PLM holds a document saved since this assembly last
    /// synced with it (another user's work), which the save would overwrite;
    /// asked as an overwrite, even when checkouts are remembered.
    CheckOut { part: String, revision: String, newer: bool },
    /// The source cannot take the change (`why`): save it as `new_key`.
    SaveAsNew { why: String, new_key: String },
    /// Copies changed differently: keep [`Planned::choice`] of
    /// [`OwnedWrite::variants`].
    Conflict,
    /// A released (not editable) PLM revision: make the part's next
    /// revision, check it out and save the change into it; every copy then
    /// names the new revision and the documents that hold one are saved
    /// again. `from` is the revision's label and state, for the question.
    NewRevision { part: String, from: String },
    /// Nobody can say yes here (checked out by someone else, still loading).
    Blocked(String),
}

/// One row of a planned Save.
#[derive(Debug, Clone, PartialEq)]
pub struct Planned {
    pub write: OwnedWrite,
    pub issue: Option<Issue>,
    /// The user's answer; every question defaults to yes.
    pub proceed: bool,
    /// For [`Issue::Conflict`]: which variant to keep.
    pub choice: usize,
    /// For [`Issue::Conflict`]: the variant edited most recently, which is
    /// also the default `choice` (`None` when the undo history cannot tell).
    pub edited_last: Option<usize>,
}

/// How a Save names one document to the user: a PLM revision by its part's
/// name with its number and revision (`plate (CPART000000003 rev A)`, not
/// its storage key), a file by its key.
pub fn shown(key: &str, names: &[String]) -> String {
    match (crate::plm::bom::revision_of_document(key), names.last().filter(|name| !name.is_empty())) {
        (Some((part, revision)), Some(name)) => format!("{name} ({part} rev {revision})"),
        (Some((part, revision)), None) => format!("{part} rev {revision}"),
        (None, _) => key.to_string(),
    }
}

/// The notice for `part` written to `key`: a PLM revision named as [`shown`]
/// names it (`Saved sub-b (CPART000000004 rev A)`), a file by its key as before.
pub fn saved_notice(part: &str, key: &str) -> String {
    if crate::plm::bom::revision_of_document(key).is_some() {
        format!("Saved {}", shown(key, &[part.to_string()]))
    } else {
        format!("Saved '{part}' to '{key}'")
    }
}

impl Planned {
    /// The dialog's line for this row.
    pub fn question(&self) -> String {
        let key = &shown(&self.write.key, &self.write.names);
        match &self.issue {
            None => format!("Save {key}"),
            Some(Issue::Overwrite) if crate::plm::bom::revision_of_document(&self.write.key).is_some() => format!(
                "{key} was saved again on the PLM since this assembly last wrote it — overwrite it with the copy here (untick to keep the PLM's version; the change stays in this assembly)"
            ),
            Some(Issue::Overwrite) => format!("{key} changed on disk since this assembly loaded it — overwrite it"),
            Some(Issue::CheckOut { newer: true, .. }) => format!(
                "{key} was saved again on the PLM since this assembly read it, and is not checked out to you — check it out and overwrite it with the copy here (untick to keep the PLM's version; the change stays in this assembly)"
            ),
            Some(Issue::CheckOut { .. }) => format!("{key} is not checked out to you — check it out and save it"),
            Some(Issue::SaveAsNew { why, new_key }) => format!("{key} {why} — save the change as a new file {new_key}"),
            Some(Issue::Conflict) => {
                let places: Vec<String> = self.write.variants.iter().map(|(names, _)| names.join(" > ")).collect();
                let newest = self
                    .edited_last
                    .and_then(|i| places.get(i))
                    .map(|place| format!("; the copy in {place} was edited last"))
                    .unwrap_or_default();
                format!("{key} was changed differently in {} ({}){newest} — keep one", places.len(), places.join(", "))
            }
            Some(Issue::NewRevision { from, .. }) => format!("{key} is {from} — make a new revision of it and save the change there"),
            Some(Issue::Blocked(why)) => format!("{key}: {why} — not saved, kept in the assembly"),
        }
    }

    /// Whether this row is a question for the user.
    pub fn asks(&self) -> bool {
        matches!(&self.issue, Some(issue) if !matches!(issue, Issue::Blocked(_)))
    }
}

/// A planned Save of the active tab's owned documents.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct OwnedSavePlan {
    /// The tab being saved (a plan never applies to another one).
    pub doc_id: u64,
    pub rows: Vec<Planned>,
    /// Deepest first.
    pub promotions: Vec<Promotion>,
    /// The name the assembly is being saved under (a failed re-create falls
    /// back to a new file beside it).
    pub saving_as: String,
}

impl OwnedSavePlan {
    /// Whether the Save dialog must ask before writing.
    pub fn needs_asking(&self) -> bool {
        self.rows.iter().any(Planned::asks)
    }

    pub fn to_json(&self) -> Value {
        serde_json::json!({
            "rows": self.rows.iter().map(|row| serde_json::json!({
                "key": row.write.key,
                "issue": match &row.issue {
                    None => "none",
                    Some(Issue::Overwrite) => "overwrite",
                    Some(Issue::CheckOut { .. }) => "checkout",
                    Some(Issue::SaveAsNew { .. }) => "saveasnew",
                    Some(Issue::Conflict) => "conflict",
                    Some(Issue::NewRevision { .. }) => "newrevision",
                    Some(Issue::Blocked(_)) => "blocked",
                },
                "question": row.question(),
                "proceed": row.proceed,
                "choice": row.choice,
                "editedLast": row.edited_last,
                "variants": row.write.variants.iter().map(|(names, _)| names.join(" > ")).collect::<Vec<_>>(),
            })).collect::<Vec<_>>(),
            "promotions": self.promotions.iter().map(|p| serde_json::json!({
                "names": p.names,
                "oldKey": p.old_key,
                "newKey": p.new_key,
            })).collect::<Vec<_>>(),
        })
    }
}

/// A PLM item to write once its checkout lands.
#[derive(Debug, Clone, PartialEq)]
pub struct CheckoutRequest {
    pub key: String,
    pub part: String,
    pub revision: String,
    pub text: String,
    /// How the notices name it ([`shown`]).
    pub shown: String,
    /// Not a checkout of `revision` but a NEW revision of `part`
    /// ([`Issue::NewRevision`]): made, checked out and written with `text`;
    /// its key replaces `key` in every copy ([`Documents::retarget_copies`]).
    pub new_revision: bool,
}

/// What one Save of an assembly did with its owned documents.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct OwnedSaveReport {
    pub written: Vec<String>,
    /// `(key, reason)`: not written, the change is kept in the assembly.
    pub skipped: Vec<(String, String)>,
    /// New files made for documents that had none.
    pub created: Vec<String>,
    /// Missing files written again at their old key.
    pub recreated: Vec<String>,
    /// Parts that had no file and could not be given one: the caller must not
    /// write the assembly, which would keep them only inside it.
    pub failed_promotions: Vec<String>,
    /// PLM items being checked out; each is written when its checkout lands.
    pub checking_out: Vec<String>,
    /// Released PLM items whose change goes into a new revision, made when
    /// the Save's requests land.
    pub revising: Vec<String>,
    /// How the summary names each key above ([`shown`]); a key missing here
    /// is named by itself.
    pub shown: std::collections::BTreeMap<String, String>,
}

impl OwnedSaveReport {
    pub fn to_json(&self) -> Value {
        serde_json::json!({
            "written": self.written,
            "skipped": self.skipped.iter().map(|(key, reason)| serde_json::json!({"key": key, "reason": reason})).collect::<Vec<_>>(),
            "created": self.created,
            "recreated": self.recreated,
            "failedPromotions": self.failed_promotions,
            "checkingOut": self.checking_out,
            "revising": self.revising,
        })
    }

    /// The Save summary line ("" when the assembly owns nothing that changed).
    pub fn summary(&self) -> String {
        let named = |keys: &[String]| keys.iter().map(|key| self.shown.get(key).unwrap_or(key).as_str()).collect::<Vec<_>>().join(", ");
        let mut parts = Vec::new();
        if !self.written.is_empty() {
            parts.push(format!("also saved {}", named(&self.written)));
        }
        if !self.revising.is_empty() {
            parts.push(format!("making new revisions of {}", named(&self.revising)));
        }
        if !self.checking_out.is_empty() {
            parts.push(format!("checking out and saving {}", named(&self.checking_out)));
        }
        if !self.created.is_empty() {
            parts.push(format!("new part files {}", self.created.join(", ")));
        }
        if !self.recreated.is_empty() {
            parts.push(format!("re-created {} (the file was missing)", self.recreated.join(", ")));
        }
        if !self.skipped.is_empty() {
            let names: Vec<String> = self.skipped.iter().map(|(key, why)| format!("{} ({why})", self.shown.get(key).unwrap_or(key))).collect();
            parts.push(format!("not saved: {} — the change is kept in the assembly", names.join("; ")));
        }
        parts.join("; ")
    }
}

/// A `sourceKey` that names a document of its own: not embedded-only, not a
/// reserved key.
pub(super) fn writable(key: &str) -> bool {
    !key.is_empty() && !key.starts_with('@')
}

pub(super) fn text_hash(text: &str) -> u64 {
    serde_json::from_str::<Value>(text).map_or(0, |value| source_hash(&value))
}

/// What a document IS, for telling two states of one source apart: its
/// stable hash without the fields a save stamps from the window it was saved
/// in (`thumbnail`, `workbench`) and without the mate solver's run record
/// ([`super::without_run_record`]). A part's own tab and its copy in an
/// assembly differ in exactly those, and are still the same part.
pub(crate) fn source_hash(document: &Value) -> u64 {
    let Some(object) = document.as_object() else {
        return stable_json_hash(document);
    };
    let mut bare = object.clone();
    bare.remove("thumbnail");
    bare.remove("workbench");
    bare.remove(INSERTED_COPIES_BLOCK);
    let mut bare = Value::Object(bare);
    super::without_run_record(&mut bare);
    stable_json_hash(&bare)
}

/// Fill in every copy `current` holds that `baseline` lacks (inserted after
/// the last Open/Save) with the copy as it was inserted, so an edit made to it
/// since is owed like any other. `libraries` is the `partsLibrary` block before
/// each undoable edit, newest first: the oldest of the newest run that holds
/// the copy is the state right after the insert (an insert, then a delete and a
/// re-insert, finds the second). No snapshot holds it when nothing was edited
/// since the insert, so the current copy is its own baseline. A top-level copy
/// `current`'s [`INSERTED_COPIES_BLOCK`] noted takes `noted_baseline` instead,
/// which neither the undo limit nor a crash recovery loses.
pub fn fill_inserted(baseline: &mut Value, current: &Value, libraries: &[&Value], store: Option<&dyn ModelStore>) {
    let block = current[INSERTED_COPIES_BLOCK].clone();
    fill_inserted_at(baseline, current, &mut Vec::new(), libraries, &block, store);
}

fn fill_inserted_at(
    baseline: &mut Value,
    current: &Value,
    names: &mut Vec<String>,
    libraries: &[&Value],
    block: &Value,
    store: Option<&dyn ModelStore>,
) {
    let Some(entries) = current["partsLibrary"].as_object() else {
        return;
    };
    for (name, entry) in entries {
        names.push(name.clone());
        if !baseline["partsLibrary"][name].is_object() && baseline.is_object() {
            let mut inserted = None;
            for library in libraries {
                match library_entry(library, names) {
                    Some(state) => inserted = Some(state),
                    None => break,
                }
            }
            let noted = (names.len() == 1).then(|| noted_baseline(entry, block, store)).flatten();
            baseline["partsLibrary"][name] = noted.unwrap_or_else(|| inserted.unwrap_or(entry).clone());
        }
        if let Some(base) = baseline.get_mut("partsLibrary").and_then(|l| l.get_mut(name)) {
            if let Some(document) = base.get_mut("document").filter(|d| d.is_object()) {
                fill_inserted_at(document, &entry["document"], names, libraries, block, store);
            }
        }
        names.pop();
    }
}

/// The entry at `names` (outermost first) in a top-level `partsLibrary` block.
fn library_entry<'a>(library: &'a Value, names: &[String]) -> Option<&'a Value> {
    let (first, rest) = names.split_first()?;
    let entry = &library[first];
    if rest.is_empty() {
        return entry.is_object().then_some(entry);
    }
    super::work_part::entry_at(&entry["document"], rest)
}

/// Every changed copy in `current` against `baseline` (both whole assembly
/// documents), deepest first. A copy absent from the baseline is not counted:
/// [`fill_inserted`] gives each inserted copy its inserted state first.
pub fn owned_changes(baseline: &Value, current: &Value) -> OwnedChanges {
    let mut found: Vec<OwnedWrite> = Vec::new();
    walk(Some(baseline), current, &mut Vec::new(), &mut found);
    // Deepest first, so the first copy of each document is its deepest.
    found.sort_by(|a, b| b.depth.cmp(&a.depth));
    let mut writes: Vec<OwnedWrite> = Vec::new();
    for item in found {
        match writes.iter_mut().find(|w| w.key == item.key) {
            Some(existing) => {
                if !existing.variants.iter().any(|(_, text)| text_hash(text) == text_hash(&item.text)) {
                    existing.variants.push((item.names, item.text));
                }
            }
            None => writes.push(item),
        }
    }
    OwnedChanges { writes }
}

fn walk(
    baseline: Option<&Value>,
    current: &Value,
    names: &mut Vec<String>,
    found: &mut Vec<OwnedWrite>,
) {
    let Some(entries) = current["partsLibrary"].as_object() else {
        return;
    };
    for (name, entry) in entries {
        let base = baseline
            .map(|b| &b["partsLibrary"][name])
            .filter(|b| b.is_object());
        // The same document as the baseline's, down to its last copy: nothing
        // in it changed. One comparison, which stops at the first difference,
        // instead of hashing (and cloning) the subtree at every level of it —
        // on l500 that was ~50 ms of every edit and every Save.
        if base.is_some_and(|base| base["document"] == entry["document"]) {
            continue;
        }
        names.push(name.clone());
        let before = found.len();
        walk(base.map(|b| &b["document"]), &entry["document"], names, found);
        if let Some(base) = base {
            let document = &entry["document"];
            // A copy inside it changed, so it did too: its hash covers
            // everything a nested copy's does. Only where nothing below moved
            // are the two (cloned, hashed) subtrees compared.
            let below = found.len() > before;
            if document.is_object() && (below || source_hash(document) != source_hash(&base["document"])) {
                let key = entry["sourceKey"].as_str().unwrap_or("");
                if writable(key) {
                    let text = part_text(document);
                    found.push(OwnedWrite {
                        key: key.to_string(),
                        names: names.clone(),
                        depth: names.len(),
                        variants: vec![(names.clone(), text.clone())],
                        text,
                        synced: base["sourceSignature"].as_str().unwrap_or("").to_string(),
                    });
                }
            }
        }
        names.pop();
    }
}

/// A new file next to `key` for a change its source cannot take:
/// `bolt.nbrep` → `bolt-edited.nbrep` (then `bolt-edited-2.nbrep`, …), one
/// the store does not hold yet.
pub fn new_file_beside(store: &dyn ModelStore, key: &str) -> String {
    let file = crate::store::file_name_of(key);
    let (stem, extension) = match file.rfind('.') {
        Some(dot) if dot > 0 => (&file[..dot], &file[dot..]),
        _ => (file.as_str(), ".nbrep"),
    };
    (1..)
        .map(|n| {
            let name = if n == 1 { format!("{stem}-edited{extension}") } else { format!("{stem}-edited-{n}{extension}") };
            store.sibling_of(key, &name).unwrap_or(name)
        })
        .find(|candidate| store.read(candidate).is_none())
        .unwrap_or_default()
}

/// A key for a new part file named after `part`, beside `saving_as`: the
/// first of `<part>.nbrep`, `<assembly>-<part>.nbrep` (so two imports' `Body`
/// parts say which assembly they came from), `<assembly>-<part>-2.nbrep`, …
/// that the store does not hold and this Save has not given out (`taken`).
pub fn new_part_key(store: &dyn ModelStore, saving_as: &str, part: &str, taken: &mut Vec<String>) -> String {
    let stem = crate::panels::file::sanitize_file_stem(part);
    let assembly = crate::panels::file::sanitize_file_stem(&crate::store::model_display_name(saving_as));
    let key = (0..)
        .map(|n| {
            let name = match n {
                0 => format!("{stem}.nbrep"),
                1 => format!("{assembly}-{stem}.nbrep"),
                n => format!("{assembly}-{stem}-{n}.nbrep"),
            };
            store.sibling_of(saving_as, &name).unwrap_or(name)
        })
        .find(|candidate| !taken.contains(candidate) && store.residency(candidate) == crate::store::Residency::Absent)
        .unwrap_or_default();
    taken.push(key.clone());
    key
}

/// Every copy in `current` with no file of its own, deepest first: no
/// `sourceKey`, or one the store no longer holds. Copies with the same
/// content share one new file; copies of one gone file are re-created once.
pub fn plan_promotions(store: &dyn ModelStore, current: &Value, saving_as: &str) -> Vec<Promotion> {
    let mut found = Vec::new();
    collect_promotions(store, current, &mut Vec::new(), &mut found);
    found.sort_by(|a: &(Vec<String>, String, u64), b| b.0.len().cmp(&a.0.len()));
    let mut taken = Vec::new();
    let mut by_content: Vec<(u64, String)> = Vec::new();
    let mut promotions: Vec<Promotion> = Vec::new();
    for (names, old_key, hash) in found {
        let new_key = if !old_key.is_empty() {
            if promotions.iter().any(|p| p.old_key == old_key) {
                continue; // one re-create relinks every copy
            }
            taken.push(old_key.clone());
            old_key.clone()
        } else if let Some((_, key)) = by_content.iter().find(|(h, _)| *h == hash) {
            key.clone()
        } else {
            let key = new_part_key(store, saving_as, names.last().map_or("part", String::as_str), &mut taken);
            by_content.push((hash, key.clone()));
            key
        };
        promotions.push(Promotion { names, old_key, new_key });
    }
    promotions
}

fn collect_promotions(store: &dyn ModelStore, document: &Value, names: &mut Vec<String>, found: &mut Vec<(Vec<String>, String, u64)>) {
    let Some(entries) = document["partsLibrary"].as_object() else {
        return;
    };
    for (name, entry) in entries {
        names.push(name.clone());
        collect_promotions(store, &entry["document"], names, found);
        let key = entry["sourceKey"].as_str().unwrap_or("");
        let document = &entry["document"];
        if document.is_object() {
            if key.is_empty() {
                found.push((names.clone(), String::new(), source_hash(document)));
            } else if writable(key) && store.residency(key) == crate::store::Residency::Absent {
                found.push((names.clone(), key.to_string(), 0));
            }
        }
        names.pop();
    }
}

/// Which variant of a conflicting `write` was edited most recently: the one
/// whose copy differs from the NEWEST undo snapshot (`libraries`, newest
/// first, each the `partsLibrary` block before an edit). Ties — copies changed
/// by the same edit, or a change the undo history no longer holds — keep the
/// deepest (the lowest index); `None` when no variant changed within it.
fn edited_last(write: &OwnedWrite, libraries: &[&Value]) -> Option<usize> {
    let mut best: Option<(usize, usize)> = None; // (snapshot age, variant)
    for (index, (names, text)) in write.variants.iter().enumerate() {
        let now = text_hash(text);
        let age = libraries.iter().position(|library| {
            library_entry(library, names).is_none_or(|entry| source_hash(&entry["document"]) != now)
        });
        if let Some(age) = age {
            if best.is_none_or(|(best_age, _)| age < best_age) {
                best = Some((age, index));
            }
        }
    }
    best.map(|(_, index)| index)
}

/// Every copy of `key` in `document`, at any depth, as its part-name chain.
fn copies_of(document: &Value, key: &str) -> Vec<Vec<String>> {
    fn walk(document: &Value, key: &str, names: &mut Vec<String>, out: &mut Vec<Vec<String>>) {
        let Some(entries) = document["partsLibrary"].as_object() else { return };
        for (name, entry) in entries {
            names.push(name.clone());
            if entry["sourceKey"].as_str() == Some(key) {
                out.push(names.clone());
            }
            walk(&entry["document"], key, names, out);
            names.pop();
        }
    }
    let mut out = Vec::new();
    walk(document, key, &mut Vec::new(), &mut out);
    out
}

/// Point the copy at `names` at `key` (and its signature). `true` when it
/// changed.
fn set_entry_source(document: &mut Value, names: &[String], key: &str, signature: &str) -> bool {
    let Some((last, parents)) = names.split_last() else {
        return false;
    };
    let mut owner = document;
    for name in parents {
        let Some(child) = owner.get_mut("partsLibrary").and_then(|l| l.get_mut(name)).and_then(|e| e.get_mut("document")) else {
            return false;
        };
        owner = child;
    }
    let Some(entry) = owner.get_mut("partsLibrary").and_then(|l| l.get_mut(last)).filter(|e| e.is_object()) else {
        return false;
    };
    let before = (entry["sourceKey"].clone(), entry["sourceSignature"].clone());
    entry["sourceKey"] = Value::String(key.to_string());
    entry["sourceSignature"] = Value::String(signature.to_string());
    before != (entry["sourceKey"].clone(), entry["sourceSignature"].clone())
}

/// The decision `write` needs (`None` = write it as is), and whether it needs
/// writing at all (`false`: the file already holds it).
/// Why a Save to the PLM cannot happen for an account without the author
/// group, in the words the Save's notice and the pane use.
pub const VIEW_ONLY: &str = "your PLM account can view it but not save to it (saving needs the author group)";

fn issue_of(store: &dyn ModelStore, write: &OwnedWrite) -> (Option<Issue>, bool) {
    if write.variants.len() > 1 {
        return (Some(Issue::Conflict), true);
    }
    if let Some(client) = store.plm_client() {
        if let Some((part, revision)) = crate::plm::bom::revision_of_document(&write.key) {
            // A viewer's checkout, new revision or write can only be refused:
            // say so before anyone confirms one.
            let viewer = client.can_author() == Some(false);
            match store.plm_revision(&write.key) {
                Some(entry) if entry.editable && entry.locked_by_me => {}
                Some(_) if viewer => return (Some(Issue::Blocked(VIEW_ONLY.into())), true),
                Some(entry) if !entry.editable => {
                    // Released: the change goes into the part's next revision
                    // (owner 2026-10-21: prompts that default to proceed).
                    let from = format!("revision {} ({})", entry.revision_label, entry.lifecycle);
                    return (Some(Issue::NewRevision { part, from }), true);
                }
                Some(entry) if entry.locked_by.is_some() => {
                    let why = format!("checked out by {}", entry.locked_by.unwrap_or_default());
                    return (Some(Issue::Blocked(why)), true);
                }
                _ => {
                    // The checkout writes over whatever the PLM holds now: a
                    // document someone saved since this assembly synced with
                    // it is their work, and the question has to say so.
                    let newer = match crate::store::read_now(store, &write.key) {
                        ReadNow::Ready(current) => {
                            text_hash(&current) != text_hash(&write.text)
                                && document_signature(&current) != write.synced
                                && !store.write_failed(&write.key)
                        }
                        ReadNow::Loading => return (Some(Issue::Blocked("still loading".into())), true),
                        ReadNow::Absent => false,
                    };
                    return (Some(Issue::CheckOut { part, revision, newer }), true);
                }
            }
        }
    }
    let current = match crate::store::read_now(store, &write.key) {
        ReadNow::Ready(text) => text,
        ReadNow::Loading => return (Some(Issue::Blocked("still loading".into())), true),
        ReadNow::Absent => {
            let new_key = new_file_beside(store, &write.key);
            return (Some(Issue::SaveAsNew { why: "is no longer in storage".into(), new_key }), true);
        }
    };
    if let Some(source) = serde_json::from_str::<Value>(&current)
        .ok()
        .as_ref()
        .and_then(crate::family_table::family_source)
    {
        let new_key = new_file_beside(store, &write.key);
        let why = format!("is member '{}' of the family '{}'", source.part_number, source.family);
        return (Some(Issue::SaveAsNew { why, new_key }), true);
    }
    if text_hash(&current) == text_hash(&write.text) {
        return (None, false);
    }
    // The stored copy differs from what this assembly last synced with. Not
    // a newer document when it is only that this session's own last write of
    // it never reached the store: then nothing to ask, the Save writes it.
    let changed_elsewhere = document_signature(&current) != write.synced && !store.write_failed(&write.key);
    (changed_elsewhere.then_some(Issue::Overwrite), true)
}

/// The document block naming the sources whose copies in this assembly hold
/// a change their own documents did not take (checked out by someone else,
/// released, the user said no, a checkout that failed): `{ "<sourceKey>":
/// "<why>" }`. The change lives only in this assembly until a Save writes it,
/// so the next Save asks again ([`Documents::plan_owned_save`]), a successful
/// write clears the key (`clear_unsaved`), and a reader that refreshes
/// copies from their sources (`plm::cached_copies`) leaves those copies alone.
/// It is saved with the assembly, file or PLM alike.
pub const UNSAVED_EDITS_BLOCK: &str = "unsavedEdits";

/// The document block naming each source a copy was inserted from since the
/// last Save, with the signature that copy had at the insert: `{ "<sourceKey>":
/// "<signature>" }`. `sourceSignature` cannot stand in, as every in-app edit
/// re-stamps it with the edited copy's own signature, and the undo history
/// loses the insert past `MAX_UNDO` steps or after a crash recovery. Kept up to
/// date per frame (`note_inserted_copies`), without undo; it never makes a
/// document unsaved (the clean signature leaves it out) and is never written
/// into a part's own file (`part_text`).
pub const INSERTED_COPIES_BLOCK: &str = "insertedCopies";

/// Bring `doc`'s [`INSERTED_COPIES_BLOCK`] up to date: a top-level copy whose
/// source the clean baseline does not hold is noted with its signature the
/// first time it is seen, so before any edit to it; a source the baseline
/// holds, or no copy uses any more, is dropped.
pub(crate) fn note_inserted_copies(doc: &mut super::Document) {
    let seen = (doc.edit_key(), doc.baseline_generation);
    if doc.inserted_noted == Some(seen) {
        return;
    }
    doc.inserted_noted = Some(seen);
    let saved: std::collections::HashSet<String> = doc.saved_sources().into_values().map(|(key, _)| key).collect();
    let old = doc.engine.history.document_block(INSERTED_COPIES_BLOCK).cloned();
    let mut block = serde_json::Map::new();
    if let Some(entries) = doc.engine.history.parts_library().as_object() {
        for entry in entries.values() {
            let Some(key) = entry["sourceKey"].as_str().filter(|key| writable(key)) else { continue };
            if saved.contains(key) || block.contains_key(key) {
                continue;
            }
            let noted = old.as_ref().and_then(|old| old.get(key)).cloned().or_else(|| {
                entry["sourceSignature"].as_str().filter(|s| !s.is_empty()).map(|s| Value::String(s.to_string()))
            });
            if let Some(signature) = noted {
                block.insert(key.to_string(), signature);
            }
        }
    }
    let block = (!block.is_empty()).then_some(Value::Object(block));
    if block != old {
        doc.engine.history.set_document_block_no_undo(INSERTED_COPIES_BLOCK, block);
        doc.inserted_noted = Some((doc.edit_key(), doc.baseline_generation));
    }
}

/// The baseline of a top-level copy inserted since the last Save, from the
/// signature [`INSERTED_COPIES_BLOCK`] noted at its insert: the source file
/// itself while the store still holds that version; the copy as it is when
/// its signature is still that one (untouched, or edited back); otherwise
/// nothing, so the copy is owed, and asked about if its file moved on.
fn noted_baseline(entry: &Value, block: &Value, store: Option<&dyn ModelStore>) -> Option<Value> {
    let key = entry["sourceKey"].as_str()?;
    let noted = block.get(key)?.as_str()?;
    if let Some(ReadNow::Ready(text)) = store.map(|store| crate::store::read_now(store, key)) {
        if document_signature(&text) == noted {
            let mut base = entry.clone();
            base["document"] = serde_json::from_str(&text).ok()?;
            base["sourceSignature"] = Value::String(noted.to_string());
            return Some(base);
        }
    }
    if document_signature(&entry["document"].to_string()) == noted {
        return Some(entry.clone());
    }
    let mut base = entry.clone();
    base["document"] = Value::Null;
    base["sourceSignature"] = Value::String(noted.to_string());
    Some(base)
}

/// The text a part's own file is written with: its copy, without the
/// assembly's own [`INSERTED_COPIES_BLOCK`] should it hold one.
fn part_text(document: &Value) -> String {
    match document.as_object() {
        Some(object) if object.contains_key(INSERTED_COPIES_BLOCK) => {
            let mut object = object.clone();
            object.shift_remove(INSERTED_COPIES_BLOCK);
            Value::Object(object).to_string()
        }
        _ => document.to_string(),
    }
}

/// The keys marked in `document`'s [`UNSAVED_EDITS_BLOCK`], with why.
pub fn unsaved_edits(document: &Value) -> std::collections::BTreeMap<String, String> {
    document[UNSAVED_EDITS_BLOCK]
        .as_object()
        .map(|block| block.iter().map(|(k, v)| (k.clone(), v.as_str().unwrap_or_default().to_string())).collect())
        .unwrap_or_default()
}

/// Mark `key` in `document` as holding a change its source did not take,
/// because `why`. `true` when anything changed.
pub(crate) fn mark_unsaved(document: &mut Value, key: &str, why: &str) -> bool {
    let Some(object) = document.as_object_mut() else { return false };
    let block = object.entry(UNSAVED_EDITS_BLOCK).or_insert_with(|| Value::Object(Default::default()));
    if !block.is_object() {
        *block = Value::Object(Default::default());
    }
    if block[key].as_str() == Some(why) {
        return false;
    }
    block[key] = Value::String(why.to_string());
    true
}

/// `key` was written: drop its mark. `true` when anything changed.
pub(crate) fn clear_unsaved(document: &mut Value, key: &str) -> bool {
    let Some(block) = document.get_mut(UNSAVED_EDITS_BLOCK).and_then(Value::as_object_mut) else { return false };
    let removed = block.remove(key).is_some();
    if block.is_empty() {
        document.as_object_mut().map(|o| o.remove(UNSAVED_EDITS_BLOCK));
    }
    removed
}

/// The first copy (`names` path) of `key` in `document`, any depth.
pub(crate) fn first_copy(document: &Value, key: &str, names: &mut Vec<String>) -> Option<(Vec<String>, Value)> {
    let entries = document["partsLibrary"].as_object()?;
    for (name, entry) in entries {
        names.push(name.clone());
        if entry["sourceKey"].as_str() == Some(key) {
            return Some((names.clone(), entry.clone()));
        }
        if let Some(found) = first_copy(&entry["document"], key, names) {
            return Some(found);
        }
        names.pop();
    }
    None
}

/// Put `document`'s [`UNSAVED_EDITS_BLOCK`] into the tab's history as it stands
/// (no undo step: it is the Save's own bookkeeping).
fn store_unsaved_block(engine: &mut EngineState, document: &Value) {
    let block = document.get(UNSAVED_EDITS_BLOCK).cloned();
    if engine.history.document_block(UNSAVED_EDITS_BLOCK) != block.as_ref() {
        engine.history.set_document_block_no_undo(UNSAVED_EDITS_BLOCK, block);
    }
}

/// Change the tab's [`UNSAVED_EDITS_BLOCK`] with `edit` (handed a document
/// holding that block alone) and put it back when `edit` says it changed. The
/// block is all the marks are: an l500 assembly is megabytes of text, and a
/// Save whose ten checkouts land in one frame paid ten round trips of it.
fn edit_unsaved_block(engine: &mut EngineState, edit: impl FnOnce(&mut Value) -> bool) -> bool {
    let mut marks = serde_json::json!({});
    if let Some(block) = engine.history.document_block(UNSAVED_EDITS_BLOCK) {
        marks[UNSAVED_EDITS_BLOCK] = block.clone();
    }
    if edit(&mut marks) {
        store_unsaved_block(engine, &marks);
        return true;
    }
    false
}

/// Stamp every copy of `key` in `document` (any depth) with `signature`, and
/// with `retarget` move it to a new key and body. `true` when anything
/// changed.
fn stamp_copies(document: &mut Value, key: &str, signature: &str, retarget: Option<(&str, &Value)>) -> bool {
    // `get_mut`, never `document["partsLibrary"]`: indexing a document that
    // has no library INSERTS a null one, which no part document parses.
    let Some(entries) = document.get_mut("partsLibrary").and_then(Value::as_object_mut) else {
        return false;
    };
    let mut changed = false;
    for entry in entries.values_mut() {
        if let Some(child) = entry.get_mut("document") {
            changed |= stamp_copies(child, key, signature, retarget);
        }
        if entry["sourceKey"].as_str() == Some(key) {
            if let Some((new_key, body)) = retarget {
                entry["sourceKey"] = Value::String(new_key.to_string());
                entry["document"] = body.clone();
                changed = true;
            }
            if entry["sourceSignature"].as_str() != Some(signature) {
                entry["sourceSignature"] = Value::String(signature.to_string());
                changed = true;
            }
        }
    }
    changed
}

/// `text` (a document) with every copy of each `(old key, new key, body)` in
/// `done` re-pointed to the new key and body: a new revision made earlier in
/// the same Save, deeper in the tree, so a holder's new revision names it.
pub fn retarget_text(text: &str, done: &[(String, String, String)]) -> String {
    let Ok(mut document) = serde_json::from_str::<Value>(text) else { return text.to_string() };
    let mut changed = false;
    for (old, new, body) in done {
        if let Ok(body_value) = serde_json::from_str::<Value>(body) {
            changed |= stamp_copies(&mut document, old, &document_signature(body), Some((new, &body_value)));
        }
    }
    if changed { document.to_string() } else { text.to_string() }
}

impl Documents {
    /// The clean baseline and current document of the tab at `index`. A tab
    /// with no clean baseline (restored after a crash) is measured, on the
    /// Save's path (`store`), against its file as stored; untitled, against
    /// nothing, so its copies count as inserted.
    fn baseline_and_current(&self, index: usize, store: Option<&dyn ModelStore>) -> Option<(Value, Value)> {
        let doc = self.open.get(index)?;
        let mut baseline = match doc.saved_signature.as_deref() {
            Some(saved) => serde_json::from_str(saved).ok()?,
            None => store
                .zip(doc.name())
                .and_then(|(store, name)| match crate::store::read_now(store, name) {
                    ReadNow::Ready(text) => serde_json::from_str(&text).ok(),
                    _ => None,
                })
                .unwrap_or_else(|| serde_json::json!({})),
        };
        let current = doc.engine.history_request_value();
        let libraries: Vec<&Value> = doc.engine.history.undo_parts_libraries().collect();
        fill_inserted(&mut baseline, &current, &libraries, store);
        Some((baseline, current))
    }

    /// What the active tab's next Save writes besides itself.
    pub fn owned_changes(&self) -> OwnedChanges {
        self.owned_changes_at(self.active, None)
    }

    fn owned_changes_at(&self, index: usize, store: Option<&dyn ModelStore>) -> OwnedChanges {
        self.baseline_and_current(index, store)
            .map(|(baseline, current)| owned_changes(&baseline, &current))
            .unwrap_or_default()
    }

    /// Each top-level parts-library entry's `sourceSignature` as of the
    /// active tab's last Open/Save: the file version it last synced with
    /// (`panels::update_components` judges "outdated" against it). An entry
    /// added since (an insert) is synced with the version it was first seen
    /// at, which is the insert's. Cached per baseline.
    pub fn synced_signatures(&self) -> std::collections::HashMap<String, String> {
        let doc = &self.open[self.active];
        let key = (doc.id(), doc.baseline_generation);
        let mut map = self.baseline_signatures(key);
        let library = doc.engine.history.parts_library();
        let mut learned = self.learned_signatures.borrow_mut();
        if let Some(entries) = library.as_object() {
            for (name, entry) in entries {
                if map.contains_key(name) {
                    continue;
                }
                let Some(signature) = entry["sourceSignature"].as_str() else { continue };
                let first = learned.entry((key, name.clone())).or_insert_with(|| signature.to_string());
                map.insert(name.clone(), first.clone());
            }
        }
        // A nested entry, by its chain: one added after the baseline is
        // learned at first sight, like a top-level one.
        for (chain, entry) in nested_entries(library) {
            if map.contains_key(&chain) {
                continue;
            }
            let Some(signature) = entry["sourceSignature"].as_str() else { continue };
            let first = learned.entry((key, chain.clone())).or_insert_with(|| signature.to_string());
            map.insert(chain, first.clone());
        }
        map
    }

    fn baseline_signatures(&self, key: (u64, u64)) -> std::collections::HashMap<String, String> {
        if let Some((cached, map)) = self.synced_cache.borrow().as_ref() {
            if *cached == key {
                return map.clone();
            }
        }
        let doc = &self.open[self.active];
        let mut map: std::collections::HashMap<String, String> =
            doc.saved_sources().into_iter().filter(|(_, (_, signature))| !signature.is_empty()).map(|(name, (_, signature))| (name, signature)).collect();
        // The nested entries' synced versions, by chain (`module/part`): one
        // parse per baseline, cached with the rest.
        if let Some(saved) = doc.saved_document() {
            for (chain, entry) in nested_entries(&saved["partsLibrary"]) {
                if let Some(signature) = entry["sourceSignature"].as_str().filter(|s| !s.is_empty()) {
                    map.insert(chain, signature.to_string());
                }
            }
        }
        *self.synced_cache.borrow_mut() = Some((key, map.clone()));
        map
    }

    /// The source documents the active tab's next Save writes (`[key]`).
    pub fn owned_modified_keys(&self) -> Vec<String> {
        self.owned_modified_keys_of(self.active_id())
    }

    /// The source documents the next Save of tab `id` writes, cached per edit
    /// (the Work part bar asks every frame; the diff serializes the assembly).
    pub fn owned_modified_keys_of(&self, id: u64) -> Vec<String> {
        let Some(index) = self.open.iter().position(|d| d.id() == id) else {
            return Vec::new();
        };
        let key = (id, self.open[index].edit_key(), self.open[index].baseline_generation);
        if let Some((cached, keys)) = self.owned_cache.borrow().as_ref() {
            if *cached == key {
                return keys.clone();
            }
        }
        let keys: Vec<String> = self.owned_changes_at(index, None).writes.into_iter().map(|w| w.key).collect();
        *self.owned_cache.borrow_mut() = Some((key, keys.clone()));
        keys
    }

    /// Save step one: what the active tab's Save would write besides itself,
    /// and the decisions it needs. `saving_as` is the name the tab is about
    /// to be written under.
    pub fn plan_owned_save(&self, store: &dyn ModelStore, saving_as: &str) -> OwnedSavePlan {
        let changes = self.owned_changes_at(self.active, Some(store));
        let current: Value = self.open[self.active].engine.history_request_value();
        let promotions = plan_promotions(store, &current, saving_as);
        let mut rows = Vec::new();
        for write in changes.writes {
            if (self.canon)(&write.key) == (self.canon)(saving_as) {
                continue; // the tab itself is the caller's write
            }
            if promotions.iter().any(|p| p.old_key == write.key) {
                continue; // its file is gone: re-created by the promotion
            }
            // `needed == false`: the file already holds it (saved from its
            // own tab); the row still stamps the signature and settles the tab.
            let (issue, _needed) = issue_of(store, &write);
            let edited_last = (issue == Some(Issue::Conflict))
                .then(|| edited_last(&write, &self.open[self.active].engine.history.undo_parts_libraries().collect::<Vec<_>>()))
                .flatten();
            rows.push(Planned { write, issue, proceed: true, choice: edited_last.unwrap_or(0), edited_last });
        }
        // A copy an earlier Save could not write is still owed, changed again
        // or not: ask again.
        for key in unsaved_edits(&current).into_keys() {
            if !writable(&key) || rows.iter().any(|r| r.write.key == key) || promotions.iter().any(|p| p.old_key == key) {
                continue;
            }
            // Every copy, deepest first, one variant per distinct text: copies
            // that differ are asked about like any changed row's.
            let mut copies: Vec<(Vec<String>, &Value)> = copies_of(&current, &key)
                .into_iter()
                .filter_map(|names| super::work_part::entry_at(&current, &names).map(|entry| (names, entry)))
                .collect();
            copies.sort_by(|a, b| b.0.len().cmp(&a.0.len()));
            // A copy that is its file as stored holds no kept change.
            if let ReadNow::Ready(file) = crate::store::read_now(store, &key) {
                let file = text_hash(&file);
                let changed: Vec<_> = copies.iter().filter(|(_, entry)| source_hash(&entry["document"]) != file).cloned().collect();
                if !changed.is_empty() {
                    copies = changed;
                }
            }
            let Some((names, entry)) = copies.first().cloned() else { continue };
            let mut variants: Vec<(Vec<String>, String)> = Vec::new();
            for (names, entry) in &copies {
                let text = part_text(&entry["document"]);
                if !variants.iter().any(|(_, known)| text_hash(known) == text_hash(&text)) {
                    variants.push((names.clone(), text));
                }
            }
            let write = OwnedWrite {
                key,
                depth: names.len(),
                text: variants[0].1.clone(),
                variants,
                names,
                synced: entry["sourceSignature"].as_str().unwrap_or("").to_string(),
            };
            let (issue, _) = issue_of(store, &write);
            let edited_last = (issue == Some(Issue::Conflict))
                .then(|| edited_last(&write, &self.open[self.active].engine.history.undo_parts_libraries().collect::<Vec<_>>()))
                .flatten();
            rows.push(Planned { write, issue, proceed: true, choice: edited_last.unwrap_or(0), edited_last });
        }
        // A document holding a copy that moves to another file names that
        // file: it is written too, even when nothing else in it changed. A
        // copy moves when it is promoted, or saved as a new file (a family
        // member's change), at whatever depth it is held.
        let mut moved: Vec<Vec<String>> = promotions.iter().map(|p| p.names.clone()).collect();
        for row in &rows {
            if matches!(row.issue, Some(Issue::SaveAsNew { .. })) {
                moved.extend(copies_of(&current, &row.write.key));
            }
        }
        for names in &moved {
            for depth in 1..names.len() {
                let names = &names[..depth];
                let Some(entry) = super::work_part::entry_at(&current, names) else { continue };
                let key = entry["sourceKey"].as_str().unwrap_or("");
                if !writable(key)
                    || promotions.iter().any(|p| p.names == names || p.old_key == key)
                    || rows.iter().any(|r| r.write.key == key)
                {
                    continue;
                }
                let text = part_text(&entry["document"]);
                let write = OwnedWrite {
                    key: key.to_string(),
                    names: names.to_vec(),
                    depth,
                    variants: vec![(names.to_vec(), text.clone())],
                    text,
                    synced: entry["sourceSignature"].as_str().unwrap_or("").to_string(),
                };
                let (issue, _) = issue_of(store, &write);
                rows.push(Planned { write, issue, proceed: true, choice: 0, edited_last: None });
            }
        }
        rows.sort_by(|a, b| b.write.depth.cmp(&a.write.depth));
        OwnedSavePlan { doc_id: self.active_id(), rows, promotions, saving_as: saving_as.to_string() }
    }

    /// "Save part as…": the file-less part placed at `component_path` in the
    /// tab at `index` (it lives only inside that assembly) gets the new part
    /// file `key` now, under the name the user chose, and every file-less copy
    /// of the same part, at any depth, is linked to it — what the Save's own
    /// promotion does under the library entry's name. Nothing stays embedded.
    /// The assembly is then unsaved: its next Save keeps the reference. A part
    /// that has a file of its own is refused; a name already taken only with
    /// `overwrite` (the dialog asks first).
    pub fn save_part_as(
        &mut self,
        index: usize,
        store: &dyn ModelStore,
        component_path: &str,
        key: &str,
        overwrite: bool,
    ) -> Result<(), String> {
        let doc = self.open.get(index).ok_or("the assembly's tab was closed")?;
        let mut current: Value = serde_json::from_str(&doc.engine.history_request_json()).map_err(|e| e.to_string())?;
        let (_, names, source, document) = super::work_part::resolve_occurrence(&doc.engine, &current, component_path)?;
        if let Some(source) = source.filter(|source| !source.is_empty()) {
            let part = names.last().map(String::as_str).unwrap_or("");
            return Err(format!("'{part}' already has a file of its own ({source}): open it and use Save As"));
        }
        if key.trim().is_empty() {
            return Err("enter a name for the part file".into());
        }
        if !overwrite && store.residency(key) != crate::store::Residency::Absent {
            return Err(format!("'{key}' already exists: choose another name"));
        }
        let same = source_hash(&document);
        let copies: Vec<Vec<String>> = plan_promotions(store, &current, doc.name().unwrap_or_default())
            .into_iter()
            .filter(|p| p.old_key.is_empty())
            .filter(|p| super::work_part::entry_at(&current, &p.names).is_some_and(|e| source_hash(&e["document"]) == same))
            .map(|p| p.names)
            .collect();
        if copies.is_empty() {
            return Err("this part is not one the assembly holds without a file".into());
        }
        let text = part_text(&document);
        store.write(key, &text)?;
        let signature = document_signature(&text);
        for names in &copies {
            set_entry_source(&mut current, names, key, &signature);
        }
        self.open[index]
            .engine
            .edit_document_json(&current.to_string())
            .map_err(|e| format!("'{key}' was written, but the assembly could not be linked to it: {e}"))?;
        Ok(())
    }

    /// Save step two, BEFORE the active tab itself is written: carry out
    /// `plan` (with the user's answers), deepest first. The assembly's
    /// own document takes the signatures (and any new file names) of what was
    /// written. PLM checkouts come back to the caller, which writes each one
    /// when its checkout lands.
    pub fn execute_owned_save(&mut self, store: &dyn ModelStore, plan: OwnedSavePlan) -> (OwnedSaveReport, Vec<CheckoutRequest>) {
        let mut report = OwnedSaveReport::default();
        let mut checkouts = Vec::new();
        if plan.doc_id != self.active_id() {
            return (report, checkouts);
        }
        let active = self.active;
        let mut document = self.open[active].engine.history_request_value();
        // `changed`: signatures stamped (no geometry moved); `rebuilt`: a copy
        // took another document or file name, which the assembly must re-run.
        let mut changed = false;
        let mut rebuilt = false;
        // The [`UNSAVED_EDITS_BLOCK`] moved (a copy kept, or one now written).
        let mut marks = false;
        let mut notes = Vec::new();
        // A conflict resolved: every copy becomes the kept one, before any
        // parent is read.
        for row in plan.rows.iter().filter(|r| r.proceed && r.issue == Some(Issue::Conflict)) {
            if let Some((names, text)) = row.write.variants.get(row.choice) {
                if let Ok(kept) = serde_json::from_str::<Value>(text) {
                    rebuilt |= super::work_part::refresh_parts(&mut document, names, Some(&row.write.key), &kept, &mut notes)
                        .unwrap_or(false);
                }
            }
        }
        // Rows and promotions, deepest first (a promotion before a row at the
        // same depth: neither holds the other).
        let mut steps: Vec<(usize, Option<Planned>, Option<Promotion>)> = plan
            .promotions
            .into_iter()
            .map(|p| (p.names.len(), None, Some(p)))
            .chain(plan.rows.into_iter().map(|r| (r.write.depth, Some(r), None)))
            .collect();
        steps.sort_by(|a, b| b.0.cmp(&a.0));
        let mut taken: Vec<String> = Vec::new();
        for (_, row, promotion) in steps {
            if let Some(promotion) = promotion {
                let engine = &mut self.open[active].engine;
                rebuilt |= promote(engine, store, &mut document, &promotion, &plan.saving_as, &mut taken, &mut report);
                continue;
            }
            let Some(row) = row else { continue };
            let key = row.write.key.clone();
            let part = row.write.names.last().cloned().unwrap_or_else(|| key.clone());
            report.shown.insert(key.clone(), shown(&key, &row.write.names));
            // The CURRENT text of this copy: a child written (or moved to a
            // new file) before it is already stamped into it.
            let Some(text) = super::work_part::entry_at(&document, &row.write.names).map(|e| e["document"].to_string()) else {
                continue;
            };
            let engine = &mut self.open[active].engine;
            let outcome = match (&row.issue, row.proceed) {
                (Some(Issue::Blocked(why)), _) => {
                    marks |= mark_unsaved(&mut document, &key, why);
                    report.skipped.push((key, why.clone()));
                    continue;
                }
                (Some(_), false) => {
                    marks |= mark_unsaved(&mut document, &key, "you chose not to save it");
                    report.skipped.push((key, "you chose not to save it".into()));
                    continue;
                }
                (Some(Issue::CheckOut { part, revision, .. }), true) => {
                    report.checking_out.push(key.clone());
                    checkouts.push(CheckoutRequest { shown: shown(&key, &row.write.names), key, part: part.clone(), revision: revision.clone(), text, new_revision: false });
                    continue;
                }
                (Some(Issue::NewRevision { part, .. }), true) => {
                    // Kept (and marked) until the new revision holds it: an app
                    // closed before the answer lands loses nothing.
                    marks |= mark_unsaved(&mut document, &key, "a new revision is being made for it");
                    report.revising.push(key.clone());
                    checkouts.push(CheckoutRequest { shown: shown(&key, &row.write.names), key, part: part.clone(), revision: String::new(), text, new_revision: true });
                    continue;
                }
                (Some(Issue::SaveAsNew { new_key, .. }), true) => {
                    // A copy of its own: a family member's stamp does not
                    // come with it (as Save As drops it).
                    let mut body: Value = serde_json::from_str(&text).unwrap_or(Value::Null);
                    if let Some(object) = body.as_object_mut() {
                        object.remove(crate::family_table::FAMILY_SOURCE_KEY);
                    }
                    let body_text = body.to_string();
                    match store.write(new_key, &body_text) {
                        Ok(()) => {
                            engine.push_notice_as(NoticeSeverity::Info, format!("Saved '{part}' as a new file '{new_key}' (was '{key}')"));
                            rebuilt |= stamp_copies(&mut document, &key, &document_signature(&body_text), Some((new_key, &body)));
                            marks |= clear_unsaved(&mut document, &key);
                            report.written.push(new_key.clone());
                        }
                        Err(error) => {
                            engine.push_notice(format!("Could not save '{new_key}': {error} — '{part}' is kept in the assembly"));
                            report.skipped.push((key, format!("write failed: {error}")));
                        }
                    }
                    continue;
                }
                (Some(Issue::Overwrite), true) => match store.write(&key, &text) {
                    Ok(()) => {
                        engine.push_notice_as(NoticeSeverity::Info, format!("{} (overwrote the newer document)", saved_notice(&part, &key)));
                        WriteOutcome::Written
                    }
                    Err(error) => {
                        engine.push_notice(format!("Could not save {}: {error} — the change is kept in the assembly", shown(&key, std::slice::from_ref(&part))));
                        WriteOutcome::Refused(format!("write failed: {error}"))
                    }
                },
                (None | Some(Issue::Conflict), _) => crate::panels::parts_library::write_through(
                    engine,
                    store,
                    &part,
                    Some(&(key.clone(), row.write.synced.clone())),
                    &text,
                ),
            };
            match outcome {
                WriteOutcome::Refused(why) => report.skipped.push((key, why)),
                outcome => {
                    // The FILE's signature: what the file holds may differ from
                    // the copy in its thumbnail alone (saved from its own tab).
                    let signature = store.read(&key).map_or_else(|| document_signature(&text), |file| document_signature(&file));
                    changed |= stamp_copies(&mut document, &key, &signature, None);
                    marks |= clear_unsaved(&mut document, &key);
                    self.settle_tab(&key, &text);
                    if outcome == WriteOutcome::Written {
                        report.written.push(key);
                    }
                }
            }
        }
        let engine = &mut self.open[active].engine;
        if rebuilt {
            if let Err(error) = engine.edit_document_json(&document.to_string()) {
                engine.push_notice(format!("Save: the assembly could not take the saved documents: {error}"));
            }
        } else if changed {
            stamp_library(engine, &document);
        }
        if marks && !rebuilt {
            store_unsaved_block(engine, &document);
        }
        for note in notes {
            engine.push_notice(format!("Save: {note}"));
        }
        (report, checkouts)
    }

    /// A PLM checkout the Save asked for failed after the assembly was written:
    /// mark `key`'s copies in the tab `doc_id` as holding a change not saved to
    /// it, so the tab reads unsaved, the next Save asks again and no refresh
    /// replaces the copy. `true` when the mark is new: the assembly as stored
    /// was written without it, so it must be saved again.
    pub fn mark_unsaved_copies(&mut self, doc_id: u64, key: &str, why: &str) -> bool {
        let Some(doc) = self.open.iter_mut().find(|d| d.id() == doc_id) else { return false };
        edit_unsaved_block(&mut doc.engine, |marks| mark_unsaved(marks, key, why))
    }

    /// The write of the stored document `key` failed after the Save that made
    /// it returned (a write-behind store: the server down, a refusal). Every
    /// open document holding a copy of it marks that copy unsaved, as a
    /// failed checkout does, and the mark goes into its stored document too:
    /// the copy was stamped as written, so without the mark the next Save
    /// would skip it and the next open would refresh it away. Returns the
    /// tabs marked.
    pub fn mark_failed_copies(&mut self, store: &dyn ModelStore, key: &str) -> Vec<u64> {
        let canonical = store.canonical_identity(key);
        let holders: Vec<(u64, bool)> = self
            .open
            .iter()
            .filter(|doc| {
                serde_json::from_str::<Value>(&doc.engine.history_request_json())
                    .is_ok_and(|document| !copies_of(&document, key).is_empty() || !copies_of(&document, &canonical).is_empty())
            })
            .map(|doc| (doc.id(), !doc.is_dirty()))
            .collect();
        let mut marked = Vec::new();
        for (id, was_clean) in holders {
            if self.mark_unsaved_copies(id, key, "its last save did not reach the PLM") {
                // A clean tab is its stored document: mark that too. An
                // unsaved one (its own write failed as well) is written with
                // the mark by its next Save.
                if was_clean {
                    let _ = self.write_unsaved_marks(store, id, true);
                }
                marked.push(id);
            }
        }
        marked
    }

    /// A mark [`Self::mark_unsaved_copies`] added after the assembly `doc_id`
    /// was written: write its stored document again with the tab's marks and
    /// nothing else of the tab, so the stored assembly that holds the change
    /// also says it is not saved to its item, and no open refreshes it away.
    /// `was_clean`: the tab held nothing unsaved before the mark, so it is
    /// clean again once written.
    pub fn write_unsaved_marks(&mut self, store: &dyn ModelStore, doc_id: u64, was_clean: bool) -> Result<(), String> {
        let doc = self.open.iter_mut().find(|d| d.id() == doc_id).ok_or("the assembly's tab was closed")?;
        let name = doc.name().ok_or("the assembly has no name")?.to_string();
        let stored = store.read(&name).ok_or_else(|| format!("'{name}' is not in the store"))?;
        let mut stored: Value = serde_json::from_str(&stored).map_err(|e| e.to_string())?;
        let current: Value = serde_json::from_str(&doc.engine.history_request_json()).map_err(|e| e.to_string())?;
        let object = stored.as_object_mut().ok_or_else(|| format!("'{name}' is not a document"))?;
        match current.get(UNSAVED_EDITS_BLOCK) {
            Some(block) => object.insert(UNSAVED_EDITS_BLOCK.to_string(), block.clone()),
            None => object.remove(UNSAVED_EDITS_BLOCK),
        };
        store.write(&name, &stored.to_string())?;
        if was_clean {
            doc.mark_clean();
        }
        Ok(())
    }

    /// A new revision the Save made holds the change of `old_key`'s copies as
    /// `new_key` (`text`): every copy in the tab `doc_id` now names it, and the
    /// mark goes. The documents that hold a copy changed with it (they name
    /// another revision), so the caller saves the tab again.
    pub fn retarget_revised_copies(&mut self, doc_id: u64, old_key: &str, new_key: &str, text: &str) -> Result<(), String> {
        let doc = self.open.iter_mut().find(|d| d.id() == doc_id).ok_or("the assembly's tab was closed")?;
        let mut document: Value = serde_json::from_str(&doc.engine.history_request_json()).map_err(|e| e.to_string())?;
        let body: Value = serde_json::from_str(text).map_err(|e| e.to_string())?;
        stamp_copies(&mut document, old_key, &document_signature(text), Some((new_key, &body)));
        clear_unsaved(&mut document, old_key);
        doc.engine.edit_document_json(&document.to_string()).map(|_| ())
    }

    /// A PLM checkout the Save asked for landed and `key` was written: drop
    /// its mark in the tab `doc_id`. `true` when there was one: the assembly
    /// as stored still carries it, so it must be saved again.
    pub fn clear_unsaved_copies(&mut self, doc_id: u64, key: &str) -> bool {
        let Some(doc) = self.open.iter_mut().find(|d| d.id() == doc_id) else { return false };
        edit_unsaved_block(&mut doc.engine, |marks| clear_unsaved(marks, key))
    }

    /// The open tab (other than the active one) of `key`, after `text` was
    /// written to it: clean when it shows `text`, else reloaded from it.
    pub fn settle_tab(&mut self, key: &str, text: &str) {
        let Some(index) = self.index_of_source(key).filter(|&index| index != self.active) else {
            return;
        };
        let doc = &mut self.open[index];
        let same = serde_json::from_str::<Value>(&doc.engine.history_request_json())
            .is_ok_and(|v| source_hash(&v) == text_hash(text));
        if same || doc.engine.set_history_json(text).is_ok() {
            doc.mark_clean();
        }
    }
}

/// Write one promoted copy (its CURRENT body in `document`, children already
/// relinked) and relink every copy of it. A failed re-create falls back to a
/// new file beside the assembly. `true` when the assembly changed.
fn promote(
    engine: &mut EngineState,
    store: &dyn ModelStore,
    document: &mut Value,
    promotion: &Promotion,
    saving_as: &str,
    taken: &mut Vec<String>,
    report: &mut OwnedSaveReport,
) -> bool {
    let part = promotion.names.last().cloned().unwrap_or_default();
    let Some(body) = super::work_part::entry_at(document, &promotion.names).map(|e| e["document"].clone()) else {
        return false;
    };
    let text = body.to_string();
    let mut key = promotion.new_key.clone();
    let mut result = store.write(&key, &text);
    let mut recreated = promotion.recreates();
    if let (Err(error), true) = (&result, recreated) {
        let fallback = new_part_key(store, saving_as, &part, taken);
        engine.push_notice(format!("Could not re-create '{key}': {error} — saving '{part}' as '{fallback}'"));
        key = fallback;
        recreated = false;
        result = store.write(&key, &text);
    }
    if let Err(error) = result {
        engine.push_notice(format!("Could not create '{key}' for '{part}': {error} — try Save again"));
        report.skipped.push((key.clone(), format!("could not create the file: {error}")));
        report.failed_promotions.push(key);
        return false;
    }
    let signature = document_signature(&text);
    let changed = if promotion.old_key.is_empty() {
        set_entry_source(document, &promotion.names, &key, &signature)
    } else {
        let retarget = (key != promotion.old_key).then_some((key.as_str(), &body));
        stamp_copies(document, &promotion.old_key, &signature, retarget)
    };
    if recreated {
        engine.push_notice_as(NoticeSeverity::Info, format!("Re-created the missing file '{key}' for '{part}'"));
        report.recreated.push(key);
    } else {
        engine.push_notice_as(NoticeSeverity::Info, format!("Saved '{part}' as a new part file '{key}'"));
        if !report.created.contains(&key) {
            report.created.push(key);
        }
    }
    changed
}

/// Put the stamped `partsLibrary` of `document` into `engine` WITHOUT a run
/// or an undo step: only signatures moved (a nested copy's, or the
/// `sourceSignature` of a document just written), nothing that builds. The
/// kernel's resident library takes each changed top-level entry too, or the
/// next run would mirror the old one back over it.
fn stamp_library(engine: &mut EngineState, document: &Value) {
    let library = document["partsLibrary"].clone();
    if let Some(entries) = library.as_object() {
        let before = engine.history.parts_library().clone();
        for (name, entry) in entries {
            if before.get(name) != Some(entry) {
                let signature = entry["sourceSignature"].as_str().unwrap_or("");
                if let Err(error) = brep_render::brep_kernel::refresh_library_entry_impl(name, signature, &entry["document"].to_string()) {
                    engine.push_notice(format!("Save: {error}"));
                }
            }
        }
    }
    engine.history.set_parts_library(library);
}

/// Write one PLM item whose checkout just landed (`result`), through the same
/// store write the PLM's own Save uses; the notice names it either way.
pub fn finish_checkout(state: &mut EngineState, store: &dyn ModelStore, request: &CheckoutRequest, result: Result<(), String>) -> bool {
    if result.is_ok() {
        // Ours from now on: a Save planned before the change feed says so
        // (the one a new revision's retarget starts) must not ask again.
        store.note_checked_out(&request.key);
    }
    match result.and_then(|()| store.write(&request.key, &request.text)) {
        Ok(()) => {
            state.push_notice_as(NoticeSeverity::Info, format!("Checked out and saved {}", request.shown));
            true
        }
        Err(error) => {
            state.push_notice(format!("{} NOT saved: {error} — the change is kept in the assembly", request.shown));
            false
        }
    }
}

/// Every NESTED parts-library entry under `library` (depth two and below), by
/// its chain of entry names joined with `/` (`module-1/part-3`): the key
/// Update components judges a nested part's file by.
pub(crate) fn nested_entries(library: &Value) -> Vec<(String, &Value)> {
    fn walk<'a>(library: &'a Value, prefix: &str, out: &mut Vec<(String, &'a Value)>) {
        for (name, entry) in library.as_object().into_iter().flatten() {
            let chain = format!("{prefix}/{name}");
            out.push((chain.clone(), entry));
            walk(&entry["document"]["partsLibrary"], &chain, out);
        }
    }
    let mut out = Vec::new();
    for (name, entry) in library.as_object().into_iter().flatten() {
        walk(&entry["document"]["partsLibrary"], name, &mut out);
    }
    out
}

/// The entry at a `/`-joined chain of names under `library`.
pub(crate) fn entry_at_chain<'a>(library: &'a Value, chain: &str) -> Option<&'a Value> {
    let mut parts = chain.split('/');
    let mut entry = library.get(parts.next()?)?;
    for name in parts {
        entry = entry.get("document")?.get("partsLibrary")?.get(name)?;
    }
    Some(entry)
}

