//! # Staleness (recompute on a cheap trigger, never per frame)
//!
//! The comparison reads the store once per entry — too hot to recompute on
//! every panel draw. [`UpdateComponents::ensure_current`] caches the result
//! keyed on `(EngineState::applied_generation, History::revision,
//! FileDialog::save_generation, ModelStore::mutation_generation)`:
//!
//! * an applied history run bumps the first half (covers opening an assembly,
//!   inserting components, an update-components refresh, constraint solves);
//! * a successful store save bumps the third (covers Edit Part → edit → save
//!   in the part's own tab → back to the assembly tab: the badge lights
//!   without a restart);
//! * ANY other write or delete through the store bumps the fourth.
//!
//! The fourth is not redundant with the third. `save_generation` counts what
//! the Save dialog did; the comparison depends on what the STORE holds, and
//! the dialog is not the only thing that writes to it. `kicad_bulk::write_part`
//! re-imports a part onto a document name an open assembly may already
//! reference, and the explorer's delete row (`panels::file`'s `del:` handler)
//! takes a source away — neither touches the assembly document, so neither
//! moved any of the other three, and the badge stayed dark until something
//! unrelated moved. Keying on the store's own mutation counter covers every
//! door into it by construction, including doors not written yet.
//!
//! Entries with an EMPTY `sourceKey` (embedded-only parts) are skipped;
//! entries whose key has no store document are NOTED (surfaced in the header
//! hover + a run-time notice) but never counted — they cannot be refreshed.
//!
//! The comparison only means "outdated" because every writer keeps the entry's
//! `sourceSignature` equal to what the FILE holds: the insert lane hashes the
//! file it read, [`UpdateComponents::run`] hashes the file it just pulled in,
//! and an in-document part edit SAVES the part back to its `sourceKey` before
//! stamping the signature (`panels::parts_library`'s write-through lane).
//! Without that last one an in-context edit read as outdated against a file it
//! was newer than — the badge meaning the opposite of what happened.

use crate::panels::parts_library::{document_signature, refresh_library_entry};
use crate::store::ModelStore;
use brep_render::engine_state::{EngineState, NoticeSeverity};
use crate::store::{ReadNow, Residency};
use eframe::egui;
use serde_json::Value;

/// The shell-owned outdated-parts checker + batch refresher. One instance on
/// the app; the constraints panel reads the count (and runs the refresh), the
/// structure tree reads per-part flags.
#[derive(Default)]
pub struct UpdateComponents {
    /// Parts-library entry names whose store content no longer matches their
    /// `sourceSignature` — the badge count.
    outdated: Vec<String>,
    /// Entry names with a `sourceKey` but NO store document under it (noted,
    /// never counted — nothing to refresh from).
    missing: Vec<String>,
    /// The `(applied_generation, history_revision, save_generation,
    /// store_mutation_generation)` key the cache was computed against; `None`
    /// forces a recompute on the next ensure.
    checked: Option<(u64, u64, u64, u64)>,
    /// The tab's synced signatures last passed to [`Self::ensure_current`]
    /// ([`compute`]); [`Self::run`] judges with the same ones.
    synced: std::collections::HashMap<String, String>,
    /// The signatures this tab took from the files with Update since that
    /// Open/Save, by entry. Such an entry has moved off its synced version,
    /// but by taking the file's, not by an edit made here: a second Update
    /// before a Save must not ask about it.
    taken: std::collections::HashMap<String, String>,
    /// The file version (`document_signature`) each entry's synced copy was
    /// judged to be, since that Open/Save. An entry whose file is still that
    /// version is not outdated whatever its copy became since: a
    /// sub-assembly re-signed by a nested part's refresh, or edited here. (Judged by
    /// signature and content alone, such a sub-assembly read as outdated
    /// once its child was refreshed, and a second Update would have put the
    /// old part back from the sub-assembly's file.)
    seen_files: std::collections::HashMap<String, std::collections::BTreeSet<String>>,
    /// Outdated parts that were ALSO edited in this document since it was
    /// opened or saved: Update would discard that edit, so [`Self::run`] asks
    /// first ([`Self::show_conflict`]).
    asking: Option<Vec<String>>,
    /// The answer to that question for the run it resumes: `true` takes the
    /// files' versions, `false` keeps this document's.
    answer: Option<bool>,
    /// Each entry's last verdict, with exactly what it was judged from
    /// ([`Verdict`]). The key above moves on every edit and every landed run,
    /// but an entry whose signature, file text and synced version are all
    /// unchanged is judged the same again: reading, parsing and hashing a
    /// 160 KB definition per entry per recompute was ~80 ms at 500 parts.
    verdicts: std::collections::HashMap<String, Verdict>,
    /// How many entries were judged from scratch (the work the cache saves).
    judged: usize,
}

/// One entry's "has this tab seen its file's version" answer, and its inputs.
#[derive(Debug, Clone)]
struct Verdict {
    signature: String,
    synced: Option<String>,
    contents: String,
    seen: bool,
}

impl UpdateComponents {
    pub fn new() -> Self {
        Self::default()
    }

    /// Recompute the outdated/missing sets when the staleness key moved (or
    /// after [`Self::invalidate`]); otherwise a cheap compare. The shell calls
    /// this once per frame BEFORE the assembly panels draw.
    ///
    /// `synced`: each top-level entry's `sourceSignature` as of the tab's last
    /// Open/Save ([`crate::document::Documents::synced_signatures`]); empty
    /// when unknown, and then the entry's own signature stands alone.
    pub fn ensure_current(
        &mut self,
        state: &mut EngineState,
        model_store: &dyn ModelStore,
        save_generation: u64,
        synced: &std::collections::HashMap<String, String>,
    ) {
        if self.synced != *synced {
            self.synced = synced.clone();
            self.taken.clear();
            self.seen_files.clear();
            self.checked = None;
        }
        let key = (
            state.applied_generation(),
            state.history.revision(),
            save_generation,
            model_store.mutation_generation(),
        );
        if self.checked == Some(key) {
            return;
        }
        let (outdated, missing) = compute(state, model_store, &self.synced, &mut self.verdicts, &mut self.seen_files, &mut self.judged);
        self.outdated = outdated;
        self.missing = missing;
        self.checked = Some(key);
    }

    /// The parts an Update is waiting to ask about (automation readback).
    pub fn asking(&self) -> Option<&[String]> {
        self.asking.as_deref()
    }

    /// Answer the question [`Self::run`] asked and run the update: `true`
    /// takes the files' versions, `false` keeps this document's edits (the
    /// other outdated parts are updated either way).
    pub fn answer(&mut self, state: &mut EngineState, model_store: &dyn ModelStore, take_files: bool) -> Result<usize, String> {
        self.asking = None;
        self.answer = Some(take_files);
        let result = self.run(state, model_store);
        self.answer = None;
        result
    }

    /// The question, while one is pending: one dialog, never a block. The
    /// default (Enter) takes the files' versions.
    pub fn show_conflict(&mut self, ctx: &egui::Context, state: &mut EngineState, model_store: &dyn ModelStore) {
        let Some(names) = self.asking.clone() else {
            if crate::automation::registry::enabled() {
                crate::automation::registry::publish("__brepUpdateComponentsHit", HIT_BLOB, "{}");
            }
            return;
        };
        let mut choice = None;
        let mut hits = serde_json::json!({});
        let rect = |r: egui::Rect| serde_json::json!([r.min.x, r.min.y, r.width(), r.height()]);
        let modal = egui::Modal::new(egui::Id::new("brep-update-components-conflict")).show(ctx, |ui| {
            ui.set_width(460.0);
            ui.heading("Update components");
            ui.label(format!(
                "{} changed in its file and was also edited in this document. Updating takes the file's version and discards the edit made here.",
                names.join(", ")
            ));
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                let file = ui.button("Take the file's version");
                hits["update:file"] = rect(file.rect);
                if file.clicked() || ui.input(|i| i.key_pressed(egui::Key::Enter)) {
                    choice = Some(true);
                }
                let mine = ui.button("Keep mine");
                hits["update:mine"] = rect(mine.rect);
                if mine.clicked() {
                    choice = Some(false);
                }
            });
        });
        if crate::automation::registry::enabled() {
            crate::automation::registry::publish("__brepUpdateComponentsHit", HIT_BLOB, &hits.to_string());
        }
        let choice = choice.or_else(|| modal.should_close().then_some(false));
        if let Some(take_files) = choice {
            if let Err(error) = self.answer(state, model_store, take_files) {
                state.push_notice(format!("Update components: {error}"));
            }
        }
    }

    /// Drop the cache so the next [`Self::ensure_current`] recomputes even on
    /// an unmoved key (after a run, or an external store change).
    pub fn invalidate(&mut self) {
        self.checked = None;
    }

    /// The outdated-entry count — the constraints-header badge number.
    pub fn outdated_count(&self) -> usize {
        self.outdated.len()
    }

    /// The outdated entry names — what the Diagram and PCB workbenches mark on
    /// the components placed from them.
    pub fn outdated(&self) -> &[String] {
        &self.outdated
    }

    /// Whether `part_name`'s library entry is outdated — the structure tree's
    /// per-node badge (every instance of the part lights).
    pub fn is_outdated(&self, part_name: &str) -> bool {
        self.outdated.iter().any(|part| part == part_name)
    }

    /// Entry names noted as source-less (header hover text).
    pub fn missing(&self) -> &[String] {
        &self.missing
    }

    /// RUN the update (the header button): recompute against the LIVE document
    /// (never a stale cache), rewrite every outdated entry from its store
    /// content through [`refresh_library_entry`], then reload ONCE — the
    /// kernel self-heals every instance + re-solves, and the main-side sync
    /// captures healed snapshots back. Per-entry failures (missing store key,
    /// unreadable content) toast and SKIP — the batch never aborts. Returns
    /// the number of entries refreshed.
    pub fn run(
        &mut self,
        state: &mut EngineState,
        model_store: &dyn ModelStore,
    ) -> Result<usize, String> {
        let (mut outdated, missing) = compute(state, model_store, &self.synced, &mut self.verdicts, &mut self.seen_files, &mut self.judged);
        // Edited HERE as well as changed in the file: ask before discarding the
        // edit (default: take the file's version). Edited here = the entry's
        // signature moved off the version the tab synced with.
        let edited_here: Vec<String> = edited_here(state, &outdated, &self.synced, &self.taken);
        if !edited_here.is_empty() {
            match self.answer.take() {
                None => {
                    self.asking = Some(edited_here);
                    return Ok(0);
                }
                Some(true) => {}
                Some(false) => outdated.retain(|name| !edited_here.contains(name)),
            }
        }
        if !missing.is_empty() {
            state.push_notice_as(NoticeSeverity::Warning, format!(
                "Update components: no source document for {} — skipped",
                missing.join(", ")
            ));
        }
        if outdated.is_empty() {
            self.invalidate();
            return Ok(0);
        }
        // Instructions take one new revision at a time: the author approves
        // the pending re-target first (the engine refuses it as well).
        if state.instructions().ok().flatten().is_some_and(|block| block.retarget.is_some()) {
            return Err("the instructions' re-target waits for approval: approve it in the Instructions pane, then update again".into());
        }
        let mut document = state.history_request_value();
        let mut refreshed = 0usize;
        let mut refreshed_parts: Vec<String> = Vec::new();
        // Every name refreshed, a nested chain included (what `taken` records).
        let mut refreshed_names: Vec<String> = Vec::new();
        for part_name in &outdated {
            let source_key = crate::document::owned_save::entry_at_chain(&document["partsLibrary"], part_name)
                .and_then(|entry| entry["sourceKey"].as_str())
                .unwrap_or_default()
                .to_string();
            let contents = match crate::store::read_now(model_store, &source_key) {
                ReadNow::Ready(contents) => contents,
                ReadNow::Loading => {
                    state.push_notice_as(NoticeSeverity::Info, format!(
                        "Update components: '{part_name}' is still loading from the store — run the update again in a moment"
                    ));
                    continue;
                }
                ReadNow::Absent => {
                    state.push_notice_as(NoticeSeverity::Warning, format!(
                        "Update components: '{part_name}' source '{source_key}' missing — skipped"
                    ));
                    continue;
                }
            };
            // A nested part: every copy of its file, at every depth, takes
            // the new version, and each sub-assembly above it is re-signed
            // and loses its snapshot (the Work part push's refresh). Its
            // mates are captured on the top-level occurrences holding it.
            if let Some((top, _)) = part_name.split_once('/') {
                let refreshed_nested = serde_json::from_str::<Value>(&contents).map_err(|error| error.to_string()).and_then(|edited| {
                    let mut notes = Vec::new();
                    crate::document::work_part::refresh_parts(&mut document, &[], Some(&source_key), &edited, &mut notes).map(|changed| (changed, notes))
                });
                match refreshed_nested {
                    Ok((true, notes)) => {
                        refreshed += 1;
                        refreshed_names.push(part_name.clone());
                        if !refreshed_parts.iter().any(|name| name == top) {
                            refreshed_parts.push(top.to_string());
                        }
                        for line in notes {
                            state.push_notice_as(NoticeSeverity::Warning, format!("Update components: '{part_name}': {line}"));
                        }
                    }
                    Ok((false, _)) => {}
                    Err(error) => state.push_notice(format!("Update components: '{part_name}': {error} — skipped")),
                }
                continue;
            }
            match refresh_library_entry(&mut document, part_name, &contents) {
                Ok(unlinked) => {
                    refreshed += 1;
                    refreshed_parts.push(part_name.clone());
                    refreshed_names.push(part_name.clone());
                    // Per-component, not per-refresh: a sheet component the
                    // assembly can no longer place is named and the rest of the
                    // batch still lands.
                    for line in unlinked {
                        state.push_notice_as(NoticeSeverity::Warning, format!("Update components: '{part_name}': {line}"));
                    }
                }
                Err(error) => state.push_notice(format!(
                    "Update components: '{part_name}': {error} — skipped"
                )),
            }
        }
        if refreshed > 0 {
            // The ONE reload for the whole batch (no zoom — the camera stays).
            // The mates on the updated parts' occurrences are checked once it
            // lands: a face the update renamed on the same plane is
            // re-attached, anything ambiguous is held for Repair.
            // Another open document's replay may own the thread's session.
            state.forget_assembly_session();
            for part_name in &refreshed_names {
                let entry = crate::document::owned_save::entry_at_chain(&document["partsLibrary"], part_name);
                if let Some(signature) = entry.and_then(|entry| entry["sourceSignature"].as_str()) {
                    self.taken.insert(part_name.clone(), signature.to_string());
                }
            }
            let capture = state.capture_part_edit(&state.components_of_parts(&refreshed_parts));
            state
                .commit_part_edit(capture, document, &refreshed_parts)
                .map_err(|error| format!("assembly reload failed: {error}"))?;
            state.push_notice_as(NoticeSeverity::Info, format!(
                "Updated {refreshed} part(s) from source — instances and eCAD snapshots refreshed"
            ));
        }
        self.invalidate();
        Ok(refreshed)
    }
}

const HIT_BLOB: &str = "Update components' question when a part was edited here AND changed in its file: update:file (default), update:mine";

pub static HIT_KEYS: &[crate::automation::hit_keys::HitKeyDoc] = &[crate::automation::hit_keys::HitKeyDoc {
    panel: "updatecomponents",
    prefix: "update:",
    meaning: "Update components' question for a part edited here AND changed in its file: update:file (take the file's version, the default), update:mine (keep this document's edit)",
    command: None,
}];

/// Of `outdated`, the parts this document ALSO edited since its last
/// Open/Save: the entry's signature is no longer the one it synced with, nor
/// one an Update took from the file (`taken`).
fn edited_here(
    state: &EngineState,
    outdated: &[String],
    synced: &std::collections::HashMap<String, String>,
    taken: &std::collections::HashMap<String, String>,
) -> Vec<String> {
    let library = state.history.parts_library();
    outdated
        .iter()
        .filter(|name| {
            synced.get(*name).is_some_and(|synced| {
                crate::document::owned_save::entry_at_chain(library, name)
                    .and_then(|entry| entry["sourceSignature"].as_str())
                    .is_some_and(|now| now != synced && taken.get(*name).is_none_or(|taken| taken != now))
            })
        })
        .cloned()
        .collect()
}

/// Whether the copy `document` of the PLM revision `source_key` is the very
/// bytes the PLM stores for it, as the store's index lists them: then it is up
/// to date, judged without reading the revision (the open's cached-copy check
/// takes such a copy for its revision the same way). Reading every source of
/// a 500-part PLM assembly cost 36 requests on every open.
fn copy_is_revision(store: &dyn ModelStore, source_key: &str, document: &Value) -> bool {
    let Some((part, revision)) = crate::plm::bom::revision_of_document(source_key) else { return false };
    document["features"].is_array()
        && store
            .plm_revision(&crate::plm::identity::document_key(&part, &revision))
            .is_some_and(|row| crate::plm::kicad::content_hash(document.to_string().as_bytes()) == row.content_hash)
}

/// The signature comparison over the LIVE document's `partsLibrary` block →
/// `(outdated, missing)` entry-name sets. Componentless documents short-circuit
/// to empty (zero cost for modeling files).
///
/// OUTDATED = the file holds a version this tab has never seen. The tab has
/// seen two: the one it synced with at its last Open/Save (`synced`) and its
/// own current copy. An edit made in the assembly (a Work part, a BOM cell, a
/// live link from the part's open tab) re-stamps the entry's signature and
/// leaves the file alone; judged against the entry's signature alone it read
/// as "outdated" and offered an Update that would REVERT it.
fn compute(
    state: &mut EngineState,
    model_store: &dyn ModelStore,
    synced: &std::collections::HashMap<String, String>,
    verdicts: &mut std::collections::HashMap<String, Verdict>,
    seen_files: &mut std::collections::HashMap<String, std::collections::BTreeSet<String>>,
    judged: &mut usize,
) -> (Vec<String>, Vec<String>) {
    let mut outdated = Vec::new();
    let mut missing = Vec::new();
    if !state.history_has_assembly() {
        return (outdated, missing);
    }
    // The document's partsLibrary block mirrors the kernel store (heals + GC
    // included); `apply_run_output` installs and mirrors the store of any run
    // that moved it, so the block is already current for this generation.
    // Read in place: serializing the whole document (and its thumbnail) to
    // parse it back cost ~40 ms per recompute at 500 parts.
    let history = &state.history;
    let Some(library) = history.parts_library().as_object() else {
        return (outdated, missing);
    };
    let sheets = [history.diagram_block(), history.pcb_block()];
    for (part_name, entry) in library {
        let Some(source_key) = entry
            .get("sourceKey")
            .and_then(Value::as_str)
            .filter(|key| !key.is_empty())
        else {
            continue; // embedded-only part: no source to compare against
        };
        if sheets.iter().all(Option::is_none) && copy_is_revision(model_store, source_key, &entry["document"]) {
            continue;
        }
        match crate::store::read_now(model_store, source_key) {
            // Listed but not loaded yet (an index-hydrated store, S0): neither
            // missing nor outdated — there is nothing to judge until the bytes
            // land, and their landing moves the store generation this cache is
            // keyed on, so the entry is judged then.
            ReadNow::Loading => {}
            ReadNow::Absent => missing.push(part_name.clone()),
            ReadNow::Ready(contents) => {
                let signature = entry
                    .get("sourceSignature")
                    .and_then(Value::as_str)
                    .unwrap_or_default();
                // A component's version belongs to the DEVICE it was placed
                // from, and a nested device's key is a key of ITS OWN parent's
                // library — the same spelling in a different namespace. Judging
                // one against a top-level entry read it as stale against a part
                // it was never placed from, and NOTHING could clear the badge:
                // `parts_library::refresh_library_entry` binds by occurrence
                // CHAIN and correctly declines to touch a device that is not
                // this entry's. So a chain (`ACOMP9:ACOMP1`) is not this
                // entry's to judge. A top-level occurrence is, and so is a
                // legacy component with no occurrence at all.
                let stale_sheet = sheets.iter().flatten().any(|block| {
                    block["components"].as_array().is_some_and(|components| {
                        components.iter().any(|component| component["part"]["key"].as_str() == Some(part_name.as_str())
                            && !component["part"]["instance"].as_str().is_some_and(|id| id.contains(':'))
                            && component["part"]["signature"].as_str() != Some(signature))
                    })
                });
                let synced_now = synced.get(part_name).cloned();
                let cached = verdicts.get(part_name).filter(|v| {
                    v.signature == signature && v.synced == synced_now && v.contents == contents
                });
                let seen = match cached {
                    Some(verdict) => verdict.seen,
                    None => {
                        *judged += 1;
                        let on_disk = document_signature(&contents);
                        let seen = on_disk == signature
                            || synced_now.as_ref().is_some_and(|synced| *synced == on_disk)
                            || seen_files.get(part_name).is_some_and(|files| files.contains(&on_disk))
                            || serde_json::from_str::<Value>(&contents).is_ok_and(|file| {
                                crate::document::owned_save::source_hash(&file)
                                    == crate::document::owned_save::source_hash(&entry["document"])
                            });
                        // Remembered only while the copy is still the synced one:
                        // the file version the baseline is. A version an
                        // Update took is not (undoing that Update makes the
                        // entry outdated again).
                        if seen && synced_now.as_deref() == Some(signature) {
                            seen_files.entry(part_name.clone()).or_default().insert(on_disk);
                        }
                        verdicts.insert(
                            part_name.clone(),
                            Verdict { signature: signature.to_string(), synced: synced_now, contents, seen },
                        );
                        seen
                    }
                };
                if !seen || stale_sheet {
                    outdated.push(part_name.clone());
                }
            }
        }
    }
    // A part that exists only INSIDE a sub-assembly has a file too: judged
    // the same way, by its chain (`module-1/part-3`). A sub-assembly's own
    // edit of it (a Work part push) stays "seen" through the synced version.
    // (Only top-level entries were judged, so a changed nested part's file
    // read as up to date and Update left the old geometry in place.)
    for (chain, entry) in crate::document::owned_save::nested_entries(history.parts_library()) {
        let Some(source_key) = entry.get("sourceKey").and_then(Value::as_str).filter(|key| !key.is_empty()) else {
            continue;
        };
        if copy_is_revision(model_store, source_key, &entry["document"]) {
            continue;
        }
        let ReadNow::Ready(contents) = crate::store::read_now(model_store, source_key) else {
            continue; // a nested source that is absent or loading is not reported
        };
        let signature = entry.get("sourceSignature").and_then(Value::as_str).unwrap_or_default();
        let synced_now = synced.get(&chain).cloned();
        let cached = verdicts
            .get(&chain)
            .filter(|v| v.signature == signature && v.synced == synced_now && v.contents == contents);
        let seen = match cached {
            Some(verdict) => verdict.seen,
            None => {
                *judged += 1;
                let on_disk = document_signature(&contents);
                let seen = on_disk == signature
                    || synced_now.as_ref().is_some_and(|synced| *synced == on_disk)
                    || seen_files.get(&chain).is_some_and(|files| files.contains(&on_disk))
                    || serde_json::from_str::<Value>(&contents).is_ok_and(|file| {
                        crate::document::owned_save::source_hash(&file)
                            == crate::document::owned_save::source_hash(&entry["document"])
                    });
                if seen && synced_now.as_deref() == Some(signature) {
                    seen_files.entry(chain.clone()).or_default().insert(on_disk);
                }
                verdicts.insert(chain.clone(), Verdict { signature: signature.to_string(), synced: synced_now, contents, seen });
                seen
            }
        };
        // One file placed at several depths is one update.
        if !seen && !outdated.iter().any(|name| {
            crate::document::owned_save::entry_at_chain(history.parts_library(), name)
                .is_some_and(|other| other["sourceKey"].as_str() == Some(source_key))
        }) {
            outdated.push(chain);
        }
    }
    (outdated, missing)
}

