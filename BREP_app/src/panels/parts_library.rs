//! The PARTS-LIBRARY document lane — the one place the app rewrites a
//! parts-library entry from a part document, and the one place it writes a part
//! document back to the file it came from.
//!
//! Two callers ride it: the update-components batch
//! ([`crate::panels::update_components`]), which pulls a changed source file
//! into every instance, and the BOM's part-attribute edits
//! ([`crate::panels::bom`]), which change a part document in place and must
//! reach that part's file. A second copy of either rule is exactly the drift
//! this module exists to prevent.
//!
//! # The refresh lane — document transport, not a UI-side kernel call
//!
//! The kernel's `refresh_library_entry` mutates the RUNNER-side thread-local
//! library (the thread/worker that runs `execute_history`); calling it from the
//! UI thread under the Thread/Worker runner would mutate a cold store nobody
//! executes against (it only *looks* right under the test-time Inline runner).
//! A refresh therefore rides the DOCUMENT: rewrite the assembly's
//! `partsLibrary[part]` entry — new `document`, a fresh content-hash
//! `sourceSignature`, and a BLANKED `snapshot` when the new document builds a
//! different part — and reload.
//! `set_history_json` resets the runner (clearing its resident library), the
//! next run ingests the block, and the empty snapshot forces the kernel's
//! designed SELF-HEAL lane: re-execute the embedded document, re-snapshot,
//! rebuild every instance, re-solve. Correct under every runner.
//!
//! # WRITE-THROUGH — saving an edited part back to its SOURCE file
//!
//! Changing a part document in the assembly is an IN-MEMORY update; on its own
//! it persists only if the user then saves the ASSEMBLY, leaving the part's
//! `.nbrep` silently behind (and — because [`crate::panels::update_components`]
//! compares the store's content against the entry's `sourceSignature` — badging
//! the component "outdated" against a file it is actually NEWER than).
//! [`write_through`] writes the edited part document back to the `sourceKey` it
//! came from, so the entry's signature and the file agree and the badge stays
//! dark. Six lanes:
//!
//! * **embedded-only** (`sourceKey: ""`): no file exists to write back to.
//!   Skipped SILENTLY. This used to be EVERY STEP-imported part; a STEP import
//!   now writes its unique parts to the store and stamps a real key, so the
//!   empty key is what a part gets when there was no store to write to — a
//!   headless import, or a write that failed.
//! * **source gone** (the file was deleted or renamed since insertion): NOT
//!   recreated — resurrecting a file the user removed is a worse surprise than
//!   a note. The assembly keeps the edit; a notice says the file was not
//!   updated.
//! * **source moved on** (the file was edited + saved elsewhere since the
//!   assembly last synced with it — the very case the outdated badge catches):
//!   NOT overwritten. Clobbering it would destroy work that lives nowhere
//!   else; the assembly keeps the edit, the notice names the file, and the
//!   badge (now genuinely divergent) stays lit.
//! * **family member** (the file carries a `familySource` stamp): NOT
//!   written. A member is generated from its family's table, so an edit
//!   written into it would be a hand edit the next Generate undoes; the
//!   assembly keeps the edit and a notice says where the change belongs.
//! * **write failed**: the edit still lands in the assembly — losing it is the
//!   worst outcome, a stale file is recoverable — with a notice carrying the
//!   backend's error. (A WRITE-BEHIND backend can also fail LATER; that lands
//!   in `ModelStore::take_persistence_errors`, which the shell already drains
//!   into the toast overlay every frame.)
//! * **written**: a notice naming the part AND the file, so the side effect on
//!   a document shared by other assemblies is visible rather than hidden.

use crate::store::ModelStore;
use brep_render::engine_state::{EngineState, NoticeSeverity};

/// Rewrite `partsLibrary[part_name]` in a parsed assembly document with a NEW
/// part document text: the embedded `document`, a fresh [`document_signature`],
/// and — when the new document builds differently — a BLANKED `snapshot`. The
/// blank snapshot is the heal trigger: the entry's resident `dirty` flag cannot
/// ride the document (serde-skipped), but an unreadable snapshot routes the
/// ACOMP feature into the SAME self-heal lane (re-execute `document`,
/// re-snapshot, rebuild every instance, re-solve).
///
/// A new version that builds the SAME part (`brep_kernel::same_build`: only
/// its symbol, pads or BOM attributes moved) keeps the snapshot, which still
/// describes it. Blanking it there rebuilt the part for an edit that changed no
/// geometry: 24 s on a herringbone gear whose reload from its snapshot takes
/// 0.12 s.
///
/// The ONE "refresh a library entry through the document" step (the
/// document-transport lane in this module's header). The caller reloads the
/// rewritten document (`set_history_json`) once — batch callers rewrite every
/// entry first, then reload.
///
/// Returns what the user should look at afterwards, one line each: the sheet
/// components the refresh could not keep bound, and the point renames it
/// carried across on POSITION rather than on evidence. Neither is a refusal —
/// the entry is rewritten and the rest of the sheet folds, because one dangling
/// instance costing the user an entire refresh is the worse outcome.
pub fn refresh_library_entry(
    assembly: &mut serde_json::Value,
    part_name: &str,
    document_json: &str,
) -> Result<Vec<String>, String> {
    let document: serde_json::Value = serde_json::from_str(document_json)
        .map_err(|error| format!("part document unreadable: {error}"))?;
    if !document.is_object() {
        return Err("part document unreadable: not a JSON object".to_string());
    }
    let mut draft = assembly.clone();
    let old = draft["partsLibrary"][part_name]["document"].clone();
    let signature = document_signature(document_json);
    // The entry goes in FIRST: the snapshot refresh walks the assembly to find
    // what version each component's own device now carries, and at depth that
    // device lives inside this very entry. A walk of the pre-edit tree would
    // read every nested part back at the version it is replacing.
    match draft["partsLibrary"].get_mut(part_name) {
        Some(entry) if entry.is_object() => {
            let rebuild = !brep_render::brep_kernel::same_build(&old, &document);
            entry["document"] = document;
            entry["sourceSignature"] = serde_json::Value::String(signature);
            if rebuild {
                entry["snapshot"] = serde_json::Value::String(String::new());
            }
        }
        _ => return Err(format!("no parts-library entry '{part_name}'")),
    }
    let mut notes = super::ecad_parts::refresh_snapshots(&mut draft, assembly, part_name)?;
    // Refresh can remove pins and therefore diagram connections. Carry that
    // removal into the harness in this same transaction, before rebuilding.
    // Components the assembly can no longer place come back as UNLINKED lines
    // rather than failing the transaction: one dangling instance used to cost
    // the user the whole refresh.
    if let Some(value) = draft.get("diagram").filter(|v| !v.is_null()) {
        let sheet = brep_ecad_core::Document::from_value(value.clone())?;
        let endpoints = super::ecad_parts::declared_endpoints(&draft);
        notes.extend(super::ecad_parts::follow_sheet(&mut draft, "diagram", &sheet, &endpoints)?);
    }
    *assembly = draft;
    Ok(notes)
}

/// WRITE-THROUGH: save `edited` (the part document the caller just changed) back
/// to the store document it came from, so the file and the assembly's embedded
/// copy stay one thing — and the update-components badge, which compares
/// [`document_signature`] of the file against the entry's `sourceSignature`,
/// reads a freshly edited part as up-to-date instead of backwards.
///
/// `target` is the entry's `(sourceKey, sourceSignature)` as it stood BEFORE the
/// edit re-stamped the signature: the key names the file, the signature is the
/// content the assembly last SYNCED with that file. `None` for an EMBEDDED-ONLY
/// entry (an empty `sourceKey`) — there is no file to write back to.
///
/// The module header lists the lanes. EVERY refusal leaves the caller's
/// in-memory change standing and only pushes a notice: a stale file is
/// recoverable, a dropped edit is not.
pub(crate) fn write_through(
    state: &mut EngineState,
    store: &dyn ModelStore,
    part_name: &str,
    target: Option<&(String, String)>,
    edited: &str,
) {
    // Embedded-only (no source document): no write, no notice — nothing changed
    // for it, so nothing to report.
    let Some((source_key, inserted_signature)) = target else {
        return;
    };
    let current = crate::store::read_now(store, source_key);
    let crate::store::ReadNow::Ready(current) = current else {
        // Listed but not loaded yet (an index-hydrated PLM store): the
        // signature check cannot run without its bytes, and it is not gone.
        if current == crate::store::ReadNow::Loading {
            state.push_notice_as(NoticeSeverity::Info, format!(
                "Source '{source_key}' is still loading — '{part_name}' updated in this document only; \
                 edit it again once it has loaded to write the part"
            ));
            return;
        }
        state.push_notice_as(NoticeSeverity::Warning, format!(
            "Source '{source_key}' is no longer in storage — \
             '{part_name}' updated in this document only, the file was NOT written"
        ));
        return;
    };
    // A FAMILY MEMBER is the family's output: writing an edit into it is a
    // hand edit the next Generate would silently undo, so it is refused here
    // as the member's own tab refuses it (the hand-edit prompt).
    if let Some(source) = serde_json::from_str::<serde_json::Value>(&current)
        .ok()
        .as_ref()
        .and_then(crate::family_table::family_source)
    {
        state.push_notice_as(NoticeSeverity::Warning, format!(
            "'{source_key}' is member '{}' of the family '{}' — NOT overwritten; \
             '{part_name}' updated in this document only (change the family, or save the part as a new part)",
            source.part_number, source.family
        ));
        return;
    }
    if document_signature(&current) != *inserted_signature {
        state.push_notice_as(NoticeSeverity::Warning, format!(
            "Source '{source_key}' changed since this document last synced '{part_name}' — \
             NOT overwritten; your edit is held here (Update Components takes the file's version instead)"
        ));
        return;
    }
    match store.write(source_key, edited) {
        Ok(()) => state.push_notice_as(NoticeSeverity::Info, format!("Saved '{part_name}' to '{source_key}'")),
        Err(error) => state.push_notice(format!(
            "Could not save '{source_key}': {error} — \
             '{part_name}' updated in this document only"
        )),
    }
}

/// The ONE signature fn, re-exported from its engine-altitude home
/// ([`brep_render::engine_state::document_signature`]) so the app's writers —
/// the insert flow (`panels::file`), [`refresh_library_entry`], the
/// update-components comparison (`panels::update_components`) — and any
/// engine-side writer hash a part document identically. Kept on this path
/// because this module is the app's documented signature/refresh home.
pub use brep_render::engine_state::document_signature;

