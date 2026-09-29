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
}

impl UpdateComponents {
    pub fn new() -> Self {
        Self::default()
    }

    /// Recompute the outdated/missing sets when the staleness key moved (or
    /// after [`Self::invalidate`]); otherwise a cheap compare. The shell calls
    /// this once per frame BEFORE the assembly panels draw.
    pub fn ensure_current(
        &mut self,
        state: &mut EngineState,
        model_store: &dyn ModelStore,
        save_generation: u64,
    ) {
        let key = (
            state.applied_generation(),
            state.history.revision(),
            save_generation,
            model_store.mutation_generation(),
        );
        if self.checked == Some(key) {
            return;
        }
        let (outdated, missing) = compute(state, model_store);
        self.outdated = outdated;
        self.missing = missing;
        self.checked = Some(key);
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
        let (outdated, missing) = compute(state, model_store);
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
        let mut document: Value = serde_json::from_str(&state.history_request_json())
            .map_err(|error| format!("assembly document unreadable: {error}"))?;
        let mut refreshed = 0usize;
        for part_name in &outdated {
            let source_key = document["partsLibrary"][part_name]["sourceKey"]
                .as_str()
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
            match refresh_library_entry(&mut document, part_name, &contents) {
                Ok(unlinked) => {
                    refreshed += 1;
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
            state
                .edit_document_json(&document.to_string())
                .map_err(|error| format!("assembly reload failed: {error}"))?;
            state.push_notice_as(NoticeSeverity::Info, format!(
                "Updated {refreshed} part(s) from source — instances and eCAD snapshots refreshed"
            ));
        }
        self.invalidate();
        Ok(refreshed)
    }
}

/// The signature comparison over the LIVE document's `partsLibrary` block →
/// `(outdated, missing)` entry-name sets. Componentless documents short-circuit
/// to empty (zero cost for modeling files).
fn compute(state: &mut EngineState, model_store: &dyn ModelStore) -> (Vec<String>, Vec<String>) {
    let mut outdated = Vec::new();
    let mut missing = Vec::new();
    if !state.history_has_assembly() {
        return (outdated, missing);
    }
    // The document's partsLibrary block mirrors the kernel store (heals + GC
    // included); `apply_run_output` installs and mirrors the store of any run
    // that moved it, so the block is already current for this generation.
    let Ok(document) = serde_json::from_str::<Value>(&state.history_request_json()) else {
        return (outdated, missing);
    };
    let Some(library) = document.get("partsLibrary").and_then(Value::as_object) else {
        return (outdated, missing);
    };
    for (part_name, entry) in library {
        let Some(source_key) = entry
            .get("sourceKey")
            .and_then(Value::as_str)
            .filter(|key| !key.is_empty())
        else {
            continue; // embedded-only part: no source to compare against
        };
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
                let stale_sheet = ["diagram", "pcb"].iter().any(|block| {
                    document[*block]["components"].as_array().is_some_and(|components| {
                        components.iter().any(|component| component["part"]["key"].as_str() == Some(part_name.as_str())
                            && !component["part"]["instance"].as_str().is_some_and(|id| id.contains(':'))
                            && component["part"]["signature"].as_str() != Some(signature))
                    })
                });
                if document_signature(&contents) != signature || stale_sheet {
                    outdated.push(part_name.clone());
                }
            }
        }
    }
    (outdated, missing)
}

