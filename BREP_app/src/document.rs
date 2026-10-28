//! Open documents and their active tab. Each document owns an engine, store
//! identity, and clean baseline for dirty tracking. Keeping engine access on
//! `Documents` allows separate mutable borrows of other app fields.
//!
//! Every engine has its own history runner and resident kernel state. All open
//! documents are pumped each frame, so switching tabs cannot redirect a pending
//! result to another document. Each open document retains its runner and scene.
//!
//! Display settings are shared across tabs by `carry_settings`; workbench
//! selection belongs to each document. A new engine initially inherits the
//! current workbench, which a loaded document can override.

use brep_render::engine_state::EngineState;

pub mod cam;
pub mod live_link;
pub mod owned_save;
pub mod work_part;
pub mod ecad;

/// Builds a fresh engine for a new document — the platform runner, the viewcube,
/// and the persisted display settings, all applied before anything is loaded
/// into it. Injected by the shell so tests (which need the SYNCHRONOUS inline
/// runner) can construct documents without a background thread.
pub type EngineFactory = Box<dyn Fn() -> EngineState>;

/// Whether a document may be edited. Every file-based document is
/// [`Access::Editable`]. PLM permissions apply to [`Document::save_access`],
/// independently of editing the session copy.
///
/// Held by the engine's history ([`brep_render::history::History::lock`]),
/// which refuses every change to the saved document while it is read-only —
/// the one door every edit passes, so no panel has to remember to ask.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Access {
    Editable,
    /// `reason` explains the edit or save restriction to the user.
    ReadOnly { reason: String },
}

impl Access {
    pub fn read_only(reason: impl Into<String>) -> Self {
        Access::ReadOnly { reason: reason.into() }
    }

    pub fn is_editable(&self) -> bool {
        matches!(self, Access::Editable)
    }
}

/// Empty history for loading fixtures and creating nonpart documents.
pub const EMPTY_DOCUMENT: &str = r#"{"expressions":"","configurator":{},"features":[]}"#;

/// Workbenches share a document format. Only part workspaces seed geometry
/// references; assembly, drawing, harness and eCAD workspaces start empty.
fn is_part_workbench(id: &str) -> bool {
    !matches!(crate::workbench::resolve(id).id,
        "assembly" | "drawing" | "wireHarness" | "diagram" | "pcb" | "symbol" | "pads")
}

/// Seed only a feature-empty part when explicitly creating a new part. PLM
/// adoption uses this too, preserving the source's expressions and document blocks.
pub fn seed_new_part_datum(engine: &mut EngineState) {
    if engine.history_len() == 0 && is_part_workbench(&engine.settings.workbench) {
        let id = engine.next_feature_id(&brep_render::features::feature_short_name("D"));
        let mut params = engine.feature_default_params("D");
        params["id"] = serde_json::json!(id);
        engine.add_feature(&serde_json::json!({
            "type": "D", "inputParams": params, "persistentData": {}
        }).to_string()).expect("valid default datum");
    }
}

/// ONE open model: the engine that owns it plus the identity the file lane needs
/// — the store name it was opened from / saved to, and the clean baseline.
pub struct Document {
    /// The windowing-agnostic brain for THIS document (scene, camera, history,
    /// settings, selection). Panels borrow it; it is never forked.
    pub engine: EngineState,
    /// The store identity (`None` = a never-saved "untitled" model). The RAW
    /// identity, not the display name — a native dialog hands back a full path
    /// and a plain Save must write back to it.
    name: Option<String>,
    /// Which untitled document this is while it has no `name`: 1 is plain
    /// "untitled", 2 and up read "untitled 2", … — assigned by
    /// [`Documents::open_document`], the lowest number no other open untitled
    /// tab holds, so two new tabs never share a label (lane U, ruling P2).
    untitled: u32,
    /// PLM write permission is independent of editing the session copy.
    plm_save_access: Option<Access>,
    /// The PLM workspace folder `(id, name)` this tab was opened from (a link
    /// in the workspace panel), for this session: the parts its Save makes
    /// PLM items for are linked there. `None`: opened another way, so the
    /// items go to the top of the workspace.
    plm_folder: Option<(String, String)>,
    /// A hash of the stored text this tab was opened from (or last reloaded
    /// from): while the store still holds those bytes, nobody has saved it
    /// since, and there is nothing to take — see [`Self::holds_stored`].
    opened_from: Option<u64>,
    /// The model request JSON as of the last New / Open / Save — the baseline
    /// the dirty flag compares the live history against.
    saved_signature: Option<String>,
    /// The baseline's top-level parts library as `{entry: (sourceKey,
    /// sourceSignature)}`, taken from the engine whenever the baseline is (a
    /// handful of entries, where parsing `saved_signature` is the whole
    /// document); `None` when the baseline came from elsewhere, so
    /// [`Self::saved_sources`] parses it.
    saved_sources: Option<SavedSources>,
    /// Moves every time `saved_signature` is replaced (a cache key for what
    /// is derived from the baseline).
    baseline_generation: u64,
    /// [`Self::content_key`] when the baseline was taken from the document
    /// itself (`None` when it came from elsewhere): while the key is unmoved,
    /// the document IS its baseline.
    baseline_content: Option<ContentKey>,
    /// While the document is as it was opened, what the runs of the open write
    /// back is part of the open: see [`Document::settle_open_runs`]. `None`
    /// once the user edits, or once anything else writes the document.
    settle: Option<Settle>,
    /// After a write that discarded the user's edits (`Take their version`),
    /// the runs it causes re-solve the document back towards its clean
    /// baseline, but not to the same bits: see [`Document::converge_runs`].
    /// `None` once the user edits, once it converged, or after
    /// [`CONVERGE_RUNS`] runs.
    converge: Option<(Settle, u32)>,
    /// The cached tab-strip dirty dot. See [`Document::dirty_marker`] for why
    /// this is not simply `is_dirty()`.
    dirty_marker: bool,
    /// The [`Document::edit_key`] `dirty_marker` was last refreshed at.
    marker_key: Option<(u64, u64)>,
    /// How many times the marker has serialized the document to recompute
    /// itself — the per-frame cost [`Document::refresh_dirty_marker`] bounds.
    dirty_checks: u64,
    /// A process-unique handle, so the shell can notice "the active document is
    /// a different one now" without comparing indices (closing a tab BEFORE the
    /// active one moves the index without changing the document).
    id: u64,
    /// This document's eCAD editors — Diagram, PCB, Symbol and Pads — kept
    /// beside its engine, so each tab has its own and a tab switch keeps their
    /// pan, zoom and view: a switch rebuilds the PANELS
    /// (`reset_document_scoped_state`), never a document. Built with the
    /// document; all four measured 0.65 ms (release), so no document waits for
    /// its first eCAD workbench to have them.
    pub ecad: crate::workbench::ecad::Editors,
    /// Each editor's standing with its block of this document's history: what
    /// takes BREP's undo and redo to the editor, and the editor's edits into the
    /// document ([`ecad::BlockSync`]).
    pub ecad_blocks: [ecad::BlockSync; 4],
    pub ecad_parts_revision: Option<u64>,
    /// The broken-reference lines `crate::panels::ecad_parts::sync` has
    /// already put in front of the user. A device whose part is missing is a
    /// STANDING state and the revision bumps on every edit, so without this it
    /// would toast on every keystroke — the same rule
    /// [`ecad::BlockSync::unreported`] keeps for an UNLINKED component.
    pub ecad_parts_reported: Vec<String>,
    /// What `crate::panels::ecad_parts::follow_occurrences` last saw of the
    /// assembly's occurrences; `None` until the first frame looks.
    pub ecad_occurrences: Option<crate::panels::ecad_parts::OccurrenceMark>,
    /// The parts-library entries whose saved part has moved on since this
    /// document took it — the shell's `UpdateComponents` list for the active
    /// document, handed over each frame so the Diagram and PCB workbenches can
    /// mark the components placed from them and offer the update where the
    /// user is looking (`viewport::ecad`).
    pub ecad_outdated: Vec<String>,
    /// Set by the eCAD host's Update parts button; the shell runs the update
    /// (`UpdateComponents::run`, one undo step) on its next frame.
    pub ecad_update_requested: bool,
    /// The history's refused-edit count as of the last
    /// [`Document::take_refused`], so each refusal is told once.
    refused_seen: u64,
    /// The [`Document::edit_key`] and baseline generation the
    /// `insertedCopies` block was last brought up to date at
    /// (`owned_save::note_inserted_copies`).
    pub(crate) inserted_noted: Option<((u64, u64), u64)>,
}

/// What the dirty flag compares: the saved document ([`EngineState::history_request_json`])
/// WITHOUT its `workbench` and derived `thumbnail`.
///
/// The workbench is saved, so a file reopens where it was left, but it is view state
/// and not work. Counting it made Close ask to discard documents nobody had edited:
/// open a board saved in PCB, look at its Diagram, and the saved document differed
/// from the file in exactly `/workbench` (the eCAD re-audit's B5, measured in lane X).
/// The tab's dot never counted it ([`Document::edit_key`] does not move for it), so
/// the prompt came with no dot. Now the two agree, and Save still writes the field.
///
/// Built as `history_request_json` builds the saved document, less the one field,
/// rather than by parsing that again: the dot recomputes this on every run-moving
/// edit, and a second parse of a megabyte assembly doubles it. A test holds the two
/// together (`the_signature_is_the_saved_document_less_its_workbench`).
/// A parts library's top-level entries as `{entry: (sourceKey, sourceSignature)}`.
pub(crate) type SavedSources = std::collections::HashMap<String, (String, String)>;

fn sources_of(library: &serde_json::Value) -> SavedSources {
    library
        .as_object()
        .map(|entries| {
            entries
                .iter()
                .map(|(name, entry)| {
                    let field = |key: &str| entry[key].as_str().unwrap_or_default().to_string();
                    (name.clone(), (field("sourceKey"), field("sourceSignature")))
                })
                .collect()
        })
        .unwrap_or_default()
}

fn raw_hash(text: &str) -> u64 {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    text.hash(&mut hasher);
    hasher.finish()
}

fn signature(engine: &EngineState) -> String {
    signature_without(engine, None)
}

/// What [`Document::settle_open_runs`] last saw: the user's edit serial at the
/// clean baseline, and the applied-run generation and history revision at the
/// last check.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Settle {
    serial: u64,
    generation: u64,
    revision: u64,
}

impl Settle {
    fn of(engine: &EngineState) -> Self {
        Self {
            serial: engine.history.edit_serial(),
            generation: engine.applied_generation(),
            revision: engine.history.revision(),
        }
    }
}

/// What the document holds: its runs and edits ([`Document::edit_key`]) and
/// its object metadata, which moves neither.
type ContentKey = ((u64, u64), u64);

fn content_key(engine: &EngineState) -> ContentKey {
    ((engine.applied_generation(), engine.history_revision()), engine.metadata.stamp())
}

/// How many applied runs [`Document::converge_runs`] watches.
const CONVERGE_RUNS: u32 = 16;
/// Relative tolerance of [`nearly_equal`] (absolute below 1).
const CONVERGE_TOLERANCE: f64 = 1e-6;

/// The document keys [`Document::converge_runs`] leaves out of its compare:
/// ONLY keys whose value in the history never reaches a saved file, because
/// the saved form (`EngineState::history_request_json`) overwrites them from
/// the session: the rendered preview, and the active workbench (stamped into
/// the history by any write-back of that form, e.g. a Work part's return).
/// Adding a key here lets a difference in it read clean, so a new entry needs
/// the same argument.
const CONVERGE_DERIVED_DOCUMENT_KEYS: &[&str] = &["thumbnail", "workbench"];
/// The embedded-copy keys it leaves out, at every depth, on the same terms:
/// the copy's cached geometry, which a refresh blanks and the next run re-heals.
const CONVERGE_DERIVED_COPY_KEYS: &[&str] = &["snapshot"];

/// `document` without [`CONVERGE_DERIVED_DOCUMENT_KEYS`] and, on every
/// embedded copy, [`CONVERGE_DERIVED_COPY_KEYS`]. Everything else is compared.
fn without_derived(mut document: serde_json::Value) -> serde_json::Value {
    fn strip(document: &mut serde_json::Value) {
        if let Some(entries) = document.get_mut("partsLibrary").and_then(serde_json::Value::as_object_mut) {
            for entry in entries.values_mut() {
                if let Some(entry) = entry.as_object_mut() {
                    for key in CONVERGE_DERIVED_COPY_KEYS {
                        entry.remove(*key);
                    }
                }
                strip(&mut entry["document"]);
            }
        }
    }
    if let Some(object) = document.as_object_mut() {
        for key in CONVERGE_DERIVED_DOCUMENT_KEYS {
            object.remove(*key);
        }
    }
    strip(&mut document);
    document
}

/// `a` and `b` alike but for floating-point numbers within
/// [`CONVERGE_TOLERANCE`]. Keys, strings, booleans and integers (counts, ids)
/// compare exactly.
fn nearly_equal(a: &serde_json::Value, b: &serde_json::Value) -> bool {
    use serde_json::Value;
    match (a, b) {
        (Value::Number(x), Value::Number(y)) if x.is_f64() && y.is_f64() => match (x.as_f64(), y.as_f64()) {
            (Some(x), Some(y)) => (x - y).abs() <= CONVERGE_TOLERANCE * x.abs().max(y.abs()).max(1.0),
            _ => false,
        },
        (Value::Array(x), Value::Array(y)) => x.len() == y.len() && x.iter().zip(y).all(|(x, y)| nearly_equal(x, y)),
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len() && x.iter().all(|(k, v)| y.get(k).is_some_and(|w| nearly_equal(v, w)))
        }
        _ => a == b,
    }
}

/// The saved-content signature with one document block left out — the
/// baseline of a copy saved WITHOUT that block (a family member saved as a new
/// part drops `familySource` on the copy), in the same canonical form.
fn signature_without(engine: &EngineState, without: Option<&str>) -> String {
    let mut document: serde_json::Value =
        serde_json::from_str(&engine.history.request_json()).unwrap_or_else(|_| serde_json::json!({}));
    if let Some(object) = document.as_object_mut() {
        if let Some(key) = without {
            // In order, as the document drops it (`set_document_block_no_undo`):
            // `remove` would move the last key into the gap.
            object.shift_remove(key);
        }
        if !engine.metadata.is_empty() {
            object.insert("metadata".into(), engine.metadata.to_json());
        }
    }
    without_run_record(&mut document);
    if let Some(object) = document.as_object_mut() {
        object.shift_remove(owned_save::INSERTED_COPIES_BLOCK);
    }
    document.to_string()
}

/// Drop what the mate solver writes into each assembly constraint as the
/// record of its last RUN — `persistentData.error`, `.kernel` (residual,
/// iterations, rank, …) and `.message`, which quotes the residual — so an
/// edit and its undo compare equal although the re-solve after the undo
/// started from the solved pose and took 0 iterations instead of 4. What a
/// solve decides (`status`, `satisfied`, first-solve choices) still counts.
/// The library's saved rows go too ([`without_saved_rows`]).
/// Comparison only: the saved file keeps the record.
pub(crate) fn without_run_record(document: &mut serde_json::Value) {
    without_saved_rows(document);
    let Some(constraints) = document
        .get_mut("assembly")
        .and_then(|assembly| assembly.get_mut("constraints"))
        .and_then(serde_json::Value::as_array_mut)
    else {
        return;
    };
    for constraint in constraints {
        if let Some(record) = constraint.get_mut("persistentData").and_then(serde_json::Value::as_object_mut) {
            for key in ["error", "kernel", "message"] {
                record.remove(key);
            }
        }
    }
}

/// Drop each library entry's `nestedRows`, at every depth: the rows of the
/// run that made the entry's snapshot, which a run writes back and re-stamps
/// (the kernel's `SavedRows`), the same kind of record as the solver's.
fn without_saved_rows(document: &mut serde_json::Value) {
    let Some(entries) = document.get_mut("partsLibrary").and_then(serde_json::Value::as_object_mut) else {
        return;
    };
    for entry in entries.values_mut() {
        if let Some(object) = entry.as_object_mut() {
            object.shift_remove("nestedRows");
        }
        if let Some(child) = entry.get_mut("document") {
            without_saved_rows(child);
        }
    }
}

/// One [`ecad::BlockSync`] per eCAD block, none of them pulled yet.
fn ecad_blocks() -> [ecad::BlockSync; 4] {
    use ecad::Block;
    [Block::Diagram, Block::Pcb, Block::Symbol, Block::Pads].map(ecad::BlockSync::new)
}

impl Document {
    /// Create a fresh document. Part workbenches start with the same ordinary,
    /// schema-seeded datum a user adds from the palette. Existing content must
    /// use `new` instead: opening a file never changes its feature history.
    pub fn fresh(mut engine: EngineState, class: crate::document_class::DocumentClass) -> Self {
        engine.set_history_json(EMPTY_DOCUMENT).expect("valid empty document");
        crate::document_class::set_document_class(&mut engine, class);
        seed_new_part_datum(&mut engine);
        Self::new(engine)
    }

    /// Wrap `engine` as a document, taking its CURRENT model as the clean
    /// baseline — so a freshly loaded (or freshly emptied) document is clean and
    /// the first edit marks it dirty.
    pub fn new(engine: EngineState) -> Self {
        let saved_signature = Some(signature(&engine));
        let baseline_content = Some(content_key(&engine));
        let saved_sources = Some(sources_of(engine.history.parts_library()));
        let settle = Some(Settle::of(&engine));
        Self {
            engine,
            name: None,
            plm_save_access: None,
            plm_folder: None,
            opened_from: None,
            baseline_content,
            saved_signature,
            saved_sources,
            baseline_generation: 0,
            settle,
            converge: None,
            dirty_marker: false,
            marker_key: None,
            dirty_checks: 0,
            id: next_document_id(),
            untitled: 1,
            ecad: crate::workbench::ecad::Editors::new(),
            ecad_blocks: ecad_blocks(),
            ecad_parts_revision: None,
            ecad_parts_reported: Vec::new(),
            ecad_occurrences: None,
            ecad_outdated: Vec::new(),
            ecad_update_requested: false,
            refused_seen: 0,
            inserted_noted: None,
        }
    }

    /// Wrap `engine` as a document RESTORED from the autosave blob
    /// (`crate::recovery`): it carries the store identity it had (`None` for
    /// an untitled one) and NO clean baseline, so it is dirty from its first
    /// frame — the work it holds is exactly what was never saved, and the close
    /// guard and the tab dot must say so until a Save writes it somewhere.
    pub fn recovered(engine: EngineState, name: Option<String>) -> Self {
        Self {
            engine,
            name,
            plm_save_access: None,
            plm_folder: None,
            opened_from: None,
            saved_signature: None,
            saved_sources: None,
            baseline_generation: 0,
            baseline_content: None,
            settle: None,
            converge: None,
            dirty_marker: true,
            marker_key: None,
            dirty_checks: 0,
            id: next_document_id(),
            untitled: 1,
            ecad: crate::workbench::ecad::Editors::new(),
            ecad_blocks: ecad_blocks(),
            ecad_parts_revision: None,
            ecad_parts_reported: Vec::new(),
            ecad_occurrences: None,
            ecad_outdated: Vec::new(),
            ecad_update_requested: false,
            refused_seen: 0,
            inserted_noted: None,
        }
    }

    /// Whether this document may be edited ([`Access`]).
    pub fn access(&self) -> Access {
        match self.engine.history.locked() {
            None => Access::Editable,
            Some(reason) => Access::read_only(reason),
        }
    }

    /// Make the document editable or read-only. Read-only locks the engine's
    /// history, so its saved bytes cannot change until it is editable again.
    pub fn set_access(&mut self, access: Access) {
        match access {
            Access::Editable => self.engine.history.unlock(),
            Access::ReadOnly { reason } => self.engine.history.lock(reason),
        }
    }

    /// Saving to the current PLM revision may be restricted while session
    /// editing remains available. A local Save As clears this restriction.
    pub fn save_access(&self) -> Access {
        self.plm_save_access.clone().unwrap_or_else(|| self.access())
    }

    pub(crate) fn has_plm_save_access(&self) -> bool { self.plm_save_access.is_some() }

    pub fn set_plm_save_access(&mut self, access: Access) {
        self.plm_save_access = Some(access);
        self.engine.history.unlock();
    }

    /// The sentence to show when an edit was refused since the last call —
    /// `None` when nothing was. Called once a frame by the shell, which toasts it.
    pub fn take_refused(&mut self) -> Option<String> {
        let refused = self.engine.history.refused_user_edits();
        if refused == self.refused_seen {
            return None;
        }
        self.refused_seen = refused;
        let reason = self.engine.history.locked()?;
        Some(format!("This document is read-only: {reason}."))
    }

    /// The raw store identity, or `None` for a never-saved document.
    pub fn name(&self) -> Option<&str> {
        self.name.as_deref()
    }

    pub fn set_name(&mut self, name: Option<String>) {
        if self.name != name { self.plm_save_access = None; }
        self.name = name;
    }

    /// The tab label: PLM part/revision, the bare file name, or `untitled`
    /// (`untitled 2`, … for the second and later untitled tabs).
    pub fn title(&self) -> String {
        match &self.name {
            Some(name) => crate::panels::plm_host::revision_of(name)
                .map(|(part, revision)| format!("{part}/{revision}"))
                .unwrap_or_else(|| crate::store::model_display_name(name)),
            None => self.untitled_title(),
        }
    }

    /// What an untitled document is called, and what Save As offers for it.
    pub fn untitled_title(&self) -> String {
        match self.untitled {
            0 | 1 => "untitled".to_string(),
            n => format!("untitled {n}"),
        }
    }

    /// Snapshot the current model as the clean baseline (after New / Open /
    /// Save, or a `?loadModel=` boot-load).
    pub fn mark_clean(&mut self) {
        self.saved_signature = Some(signature(&self.engine));
        self.saved_sources = Some(sources_of(self.engine.history.parts_library()));
        self.baseline_generation += 1;
        self.baseline_content = Some(content_key(&self.engine));
        self.dirty_marker = false;
        self.marker_key = Some(self.edit_key());
        self.settle = Some(Settle::of(&self.engine));
    }

    /// Keep taking what the open's runs write back as part of the open, from
    /// here (a check on open that wrote the document itself and absorbed that
    /// write, [`Self::absorb_open_write`], re-arms so the run it causes is
    /// absorbed too).
    pub(crate) fn arm_open_settle(&mut self) {
        self.settle = Some(Settle::of(&self.engine));
    }

    /// A document's first runs after it is opened write back into it: the
    /// first solve of a mate stored unsolved (the pose and the solve's
    /// status), and the snapshots of embedded parts stored without one (the
    /// PLM import blanks them). None of that is the user's change, so closing
    /// the document must not ask to save it.
    ///
    /// Called once a frame. While the user has not edited the document since
    /// it was opened (its edit serial is unchanged), a run that has APPLIED
    /// moves the clean baseline to the document as that run left it. The rule
    /// is conservative where it cannot tell: the document changing while no
    /// run applied (a write that took no undo step, which could be the user's)
    /// stops the settling for good, and the document stays as dirty as it is.
    /// The worst case is one needless save prompt, never an unsaved edit that
    /// reads saved.
    pub fn settle_open_runs(&mut self) {
        let Some(last) = self.settle else {
            return;
        };
        let now = Settle::of(&self.engine);
        if now.serial != last.serial || (now.generation == last.generation && now.revision != last.revision) {
            self.settle = None;
            return;
        }
        if now.generation == last.generation || self.engine.run_pending() {
            return;
        }
        if now.revision != last.revision {
            self.saved_signature = Some(signature(&self.engine));
            self.saved_sources = Some(sources_of(self.engine.history.parts_library()));
            self.baseline_generation += 1;
            self.baseline_content = Some(content_key(&self.engine));
            self.marker_key = None;
        }
        self.settle = Some(now);
    }

    /// Watch the runs a write that discarded the user's edits causes: see
    /// [`Self::converge_runs`].
    pub(crate) fn arm_converge(&mut self) {
        self.converge = Some((Settle::of(&self.engine), 0));
    }

    /// Called once a frame. After `Self::arm_converge`, once a run has
    /// applied and the user has not edited since: when the document equals its
    /// clean baseline to within `CONVERGE_TOLERANCE` in every number (mates
    /// re-solved the poses the discarded edit had moved, to ~1e-9), the solved
    /// document becomes the baseline, as a Save's would. A real change is far
    /// outside the tolerance and leaves the tab changed.
    pub fn converge_runs(&mut self) {
        let Some((last, runs)) = self.converge else {
            return;
        };
        let now = Settle::of(&self.engine);
        if now.serial != last.serial || runs >= CONVERGE_RUNS {
            self.converge = None;
            return;
        }
        if now.generation == last.generation || self.engine.run_pending() {
            return;
        }
        let current = signature(&self.engine);
        let near = self.saved_signature.as_deref().is_some_and(|saved| {
            match (serde_json::from_str::<serde_json::Value>(saved), serde_json::from_str::<serde_json::Value>(&current)) {
                (Ok(saved), Ok(current)) => nearly_equal(&without_derived(saved), &without_derived(current)),
                _ => false,
            }
        });
        if near {
            if self.saved_signature.as_deref() != Some(current.as_str()) {
                self.saved_signature = Some(current);
                self.saved_sources = Some(sources_of(self.engine.history.parts_library()));
                self.baseline_generation += 1;
                self.baseline_content = Some(content_key(&self.engine));
                self.marker_key = None;
            }
            self.converge = None;
        } else {
            self.converge = Some((now, runs + 1));
        }
    }

    pub(crate) fn save_signature(&self) -> String { signature(&self.engine) }

    /// The workspace folder this tab was opened from ([`Self::plm_folder`]'s field).
    pub(crate) fn plm_folder(&self) -> Option<&(String, String)> {
        self.plm_folder.as_ref()
    }

    /// The tab was opened (or reloaded) from the stored `text`.
    pub(crate) fn note_opened_from(&mut self, text: &str) {
        self.opened_from = Some(raw_hash(text));
    }

    /// Whether the store's `text` is still the text this tab was opened from:
    /// a byte compare, not the document's, which the open's own runs have
    /// written back into (snapshots, a first mate solve).
    pub(crate) fn holds_stored(&self, text: &str) -> bool {
        self.opened_from == Some(raw_hash(text))
    }

    pub(crate) fn set_plm_folder(&mut self, folder: Option<(String, String)>) {
        self.plm_folder = folder;
    }

    /// The clean baseline's top-level parts library, `{entry: (sourceKey,
    /// sourceSignature)}` (empty for a recovered, never-saved tab), without
    /// parsing the saved document when the baseline was taken from the engine.
    pub(crate) fn saved_sources(&self) -> SavedSources {
        match &self.saved_sources {
            Some(sources) => sources.clone(),
            None => self
                .saved_signature
                .as_deref()
                .and_then(|saved| serde_json::from_str::<serde_json::Value>(saved).ok())
                .map(|saved| sources_of(&saved["partsLibrary"]))
                .unwrap_or_default(),
        }
    }

    /// The clean baseline as a document (`None` for a recovered, never-saved
    /// one): what this tab last opened or saved.
    pub(crate) fn saved_document(&self) -> Option<serde_json::Value> {
        serde_json::from_str(self.saved_signature.as_deref()?).ok()
    }

    /// Take `baseline` (what [`Self::saved_document`] returned, changed the
    /// way the document itself just was) as the clean baseline: a write that
    /// is not the user's change adds no unsaved change, and the user's own
    /// unsaved changes stay unsaved. `key` is [`Self::edit_key`] before the write.
    pub(crate) fn rebase_saved_document(&mut self, baseline: &serde_json::Value, key: (u64, u64)) {
        self.saved_signature = Some(baseline.to_string());
        // Not the engine's own state: the sources are read from it on demand.
        self.saved_sources = None;
        self.baseline_generation += 1;
        self.baseline_content = None;
        if self.marker_key == Some(key) {
            self.marker_key = Some(self.edit_key());
        }
    }

    /// [`Self::save_signature`] of the document with one block left out: what a
    /// copy saved without that block has as its clean baseline.
    pub(crate) fn save_signature_without(&self, block: &str) -> String {
        signature_without(&self.engine, Some(block))
    }

    pub(crate) fn mark_saved_signature(&mut self, saved: String) {
        self.saved_signature = Some(saved);
        self.saved_sources = None;
        self.baseline_generation += 1;
        self.baseline_content = None;
        self.marker_key = None;
        self.refresh_dirty_marker();
    }

    /// Restored from crash recovery and not saved since: it has no clean
    /// baseline, and what differs from its stored document is the user's
    /// unsaved work, not an older copy to refresh.
    pub fn is_recovered(&self) -> bool {
        self.saved_signature.is_none()
    }

    /// Its Save did not reach the store after all (a write-behind backend
    /// reported the failure once `write` had returned `Ok`): unsaved again, so
    /// the close prompt asks and Save writes it. A later Save that lands takes
    /// a new baseline as any Save does.
    pub(crate) fn mark_write_failed(&mut self) {
        self.mark_saved_signature(String::new());
    }

    /// Dirty = the live model differs from the last saved/opened baseline.
    /// (Rolling the history does NOT change the request document, so navigating
    /// steps never marks dirty — only real edits/add/delete/reorder do.)
    /// The workbench the document is in is not part of it (`signature`).
    ///
    /// HONEST but not free: it serializes the whole request document, which for
    /// an assembly carrying an embedded parts library is megabytes. Every place
    /// where being wrong would cost the user work — the close prompt, Save —
    /// calls THIS. The per-frame tab dot calls [`Self::dirty_marker`] instead.
    pub fn is_dirty(&self) -> bool {
        match &self.saved_signature {
            Some(saved) => *saved != signature(&self.engine),
            None => true,
        }
    }

    /// What moves when the saved document can have changed: the APPLIED RUN
    /// generation and the history's revision.
    ///
    /// Both, because each misses edits the other sees. An ordinary edit (a
    /// param change, add, delete, reorder, undo) re-runs the history, but a
    /// block write runs nothing — a drawing sheet, a PMI label move, an eCAD
    /// edit, a BOM part-attribute edit — and moves only the revision, which
    /// every write to the SAVED document moves, the `partsLibrary` block
    /// included (it is held in a field beside the document and both its setters
    /// tick the revision themselves). An object-metadata change (a workbench
    /// change is not an unsaved change at all, `signature`) moves neither and can
    /// leave the dot one edit behind: a marker briefly optimistic, never a lost
    /// edit, since [`Self::is_dirty`] is recomputed honestly at the close prompt.
    pub fn edit_key(&self) -> (u64, u64) {
        (self.engine.applied_generation(), self.engine.history_revision())
    }

    /// Whether the document is still the baseline it took from itself (an
    /// Open, a Save, or the open's runs settling).
    pub(crate) fn is_its_baseline(&self) -> bool {
        self.baseline_content == Some(content_key(&self.engine))
    }

    /// The cached dirty flag the tab strip draws. Recomputing it serializes
    /// the whole document, so it is recomputed only when the
    /// [`Self::edit_key`] moved, and, once the tab is marked unsaved, only when
    /// a RUN moved it (an ordinary edit, an undo, a redo).
    ///
    /// The second rule is for drags. Dragging a block (a PMI label, a view on
    /// a sheet) writes the document every frame and moves only the revision;
    /// recomputing there serialized the document on every frame, 2.6 ms in a
    /// release build on a 1.3 MB assembly. An unsaved tab stays unsaved while
    /// only the revision moves, so a drag pays one serialization, on the frame
    /// the dot lights. Param-slider drags re-run the history each frame and
    /// still recompute each frame, as they always did.
    ///
    /// The residue is deliberate: a block edit that lands exactly back on the
    /// saved bytes keeps the dot on until the next run or Save. Showing unsaved
    /// when clean costs one needless Save; the opposite loses work, which is
    /// the bug the revision key fixed. [`Self::is_dirty`] stays honest at the
    /// close prompt either way.
    pub fn refresh_dirty_marker(&mut self) {
        let key = self.edit_key();
        let previous = self.marker_key.replace(key);
        if previous == Some(key) {
            return;
        }
        let run_moved = previous.map_or(true, |(generation, _)| generation != key.0);
        if self.dirty_marker && !run_moved {
            return;
        }
        self.dirty_checks += 1;
        self.dirty_marker = self.is_dirty();
    }

    /// Take a write that OPENING the document made — `before` and `after` are
    /// the saved document either side of it, `key` the [`Self::edit_key`] before
    /// it — as though the file had been saved that way: every top-level block
    /// the write changed, and that was still as last saved, is moved in the
    /// clean baseline too, and a tab dot that was current stays as it was.
    ///
    /// For an eCAD editor's first sight of an older board (`viewport::ecad`'s
    /// `store`). Not [`Self::mark_clean`]: whatever else was unsaved before the
    /// open stays unsaved. So the open changes nothing a document that needed no
    /// bringing up would show — the honest [`Self::is_dirty`] and the dot alike.
    pub fn absorb_open_write(&mut self, before: &serde_json::Value, after: &serde_json::Value, key: (u64, u64)) {
        // Compared as the baseline was taken ([`signature`]): without the
        // run's records, or a block holding saved rows never matches.
        let compared = |document: &serde_json::Value| {
            let mut document = document.clone();
            without_run_record(&mut document);
            document
        };
        let (before, after) = (&compared(before), &compared(after));
        let rebased = self.saved_signature.as_deref().and_then(|saved| {
            let mut base: serde_json::Value = serde_json::from_str(saved).ok()?;
            let (base_map, before, after) = (base.as_object_mut()?, before.as_object()?, after.as_object()?);
            for name in before.keys().chain(after.keys()) {
                let (was, is) = (before.get(name), after.get(name));
                if was == is || base_map.get(name) != was {
                    continue;
                }
                match is {
                    Some(value) => base_map.insert(name.clone(), value.clone()),
                    None => base_map.shift_remove(name),
                };
            }
            Some(base.to_string())
        });
        if let Some(rebased) = rebased {
            self.saved_signature = Some(rebased);
            self.saved_sources = None;
            self.baseline_generation += 1;
            self.baseline_content = None;
        }
        if self.marker_key == Some(key) {
            self.marker_key = Some(self.edit_key());
        }
    }

    /// How many times [`Self::refresh_dirty_marker`] has serialized the
    /// document.
    pub fn dirty_checks(&self) -> u64 {
        self.dirty_checks
    }

    pub fn dirty_marker(&self) -> bool {
        self.dirty_marker
    }

    /// The document's process-unique handle (identity across index shuffles).
    pub fn id(&self) -> u64 {
        self.id
    }
}

/// Every open document + which one is active. Always holds AT LEAST ONE
/// document: closing the last tab leaves a fresh untitled one in its place, so
/// [`Documents::engine_mut`] is infallible and the shell never has to render a
/// "no document" state that would be an empty viewport with extra steps.
pub struct Documents {
    work_part: Option<work_part::WorkPart>,
    /// An ancestor sub-assembly to make the Work part once its assembly's run
    /// lands: `(assembly tab id, component path, mode)`
    /// ([`Documents::work_part_up_to`]).
    work_part_next: Option<(u64, String, work_part::WorkPartMode)>,
    /// The Work part readback's `context` map (every assembly solid outside
    /// the work occurrence, in assembly coordinates), by (component path,
    /// host scene fingerprint): automation reads it every frame.
    work_part_context: std::cell::RefCell<Option<((String, u64), serde_json::Value)>>,
    /// [`Documents::owned_modified_keys_of`]'s last answer: `(tab id, edit
    /// key, baseline length)` and the keys.
    owned_cache: std::cell::RefCell<Option<owned_save::OwnedCacheKey>>,
    /// [`Documents::synced_signatures`]'s last answer, by (tab id, baseline).
    synced_cache: std::cell::RefCell<Option<owned_save::SyncedCache>>,
    /// The signature each entry added after a tab's baseline was first seen
    /// with, by ((tab id, baseline generation), entry name).
    learned_signatures: std::cell::RefCell<std::collections::HashMap<((u64, u64), String), String>>,
    /// The newest state of every source open documents hold ([`live_link`]).
    live: live_link::LiveLinks,
    /// The store's spelling of a document identity
    /// ([`crate::store::ModelStore::canonicalizer`]): a tab and a `sourceKey`
    /// naming one file the same way.
    canon: crate::store::Canonicalizer,
    open: Vec<Document>,
    active: usize,
    /// How a new tab's engine is built (platform runner + persisted settings).
    new_engine: EngineFactory,
}

impl Documents {
    /// A session holding one fresh document built by `new_engine`.
    pub fn new(new_engine: EngineFactory) -> Self {
        let first = Document::fresh(new_engine(), crate::document_class::DocumentClass::Normal);
        Self {
            open: vec![first],
            work_part: None,
            work_part_next: None,
            work_part_context: Default::default(),
            owned_cache: Default::default(),
            synced_cache: Default::default(),
            learned_signatures: Default::default(),
            live: Default::default(),
            canon: std::rc::Rc::new(str::to_string),
            active: 0,
            new_engine,
        }
    }

    /// A fresh engine for a document about to be opened — the caller loads into
    /// it and hands the result to [`Self::open_document`].
    ///
    /// Seeded with the WHOLE of the session's settings, workbench included, so
    /// a new tab continues what you were doing. The load that follows overrides
    /// the workbench if the document names one (see the module header).
    /// Build every later tab's engine through `wrap` as well: the app's live
    /// PLM reconnect puts the new store's settings over the factory's.
    pub fn wrap_engine_factory(&mut self, wrap: impl Fn(EngineState) -> EngineState + 'static) {
        let old = std::mem::replace(&mut self.new_engine, Box::new(EngineState::new));
        self.new_engine = Box::new(move || wrap(old()));
    }

    pub fn spawn_engine(&self) -> EngineState {
        let mut engine = (self.new_engine)();
        carry_settings(&self.engine().settings_json(), &mut engine, true);
        // ...and the LIVE VIEWPORT SIZE, which decides how the document about to
        // be loaded is FRAMED.
        //
        // A brand-new engine's camera is `ViewCamera::default()` — 800x600 —
        // until the viewport tile draws it (`Viewport::show` is the only
        // production `EngineState::resize`), and that is a whole frame after the
        // load. But `load_model_and_fit` arms `pending_fit`, and the app pumps
        // every open document at the TOP of the frame (`BrepApp::ui`), before
        // the dock lays out and before the viewport draws. So when the run
        // lands quickly — a warm process, a small document — the deferred
        // `zoom_to_fit` fires against 800x600 and frames the model to the wrong
        // ASPECT; when the run lands a frame later it fires against the real
        // viewport. Same document, same window, two different cameras decided by
        // a race with the background runner. Handing the engine the session's
        // viewport size HERE, before anything is loaded into it, makes both
        // orderings frame the same way. (The app-level one-shot at
        // `BrepApp::first_run_framed` sits in the same place in the frame and is
        // fixed by the same line.)
        let (width, height) = self.viewport_size();
        engine.resize(width, height);
        engine
    }

    /// The session's 3D viewport in logical px, as the active engine last had it
    /// from [`crate::viewport::Viewport::show`]. Every document in a session
    /// draws into the SAME tile, so this is the size any new one will get.
    fn viewport_size(&self) -> (f64, f64) {
        let camera = &self.engine().camera;
        (camera.width, camera.height)
    }

    /// The engine the user is editing: the active tab's, or — while a Work
    /// part is edited in place in that tab — the hosted part's
    /// ([`work_part`]). Save, the dirty dot and the tab identity stay the
    /// tab's own ([`Self::active`], [`Self::tab_engine`]).
    pub fn engine(&self) -> &EngineState {
        match self.hosted() {
            Some(doc) => &doc.engine,
            None => &self.open[self.active].engine,
        }
    }

    pub fn engine_mut(&mut self) -> &mut EngineState {
        if self.work_part_in_place() {
            return &mut self.hosted_mut().unwrap().engine;
        }
        &mut self.open[self.active].engine
    }

    /// The active TAB's own engine — the document Save writes. The same as
    /// [`Self::engine`] except while a Work part is edited in place, when this
    /// is the assembly.
    pub fn tab_engine(&self) -> &EngineState {
        &self.open[self.active].engine
    }

    pub fn tab_engine_mut(&mut self) -> &mut EngineState {
        &mut self.open[self.active].engine
    }

    /// The identity of what the panels and viewport are drawing: the hosted
    /// Work part's document while one is edited in place, else the active
    /// tab's. The shell resets its document-scoped state when this changes.
    pub fn view_id(&self) -> u64 {
        self.hosted().map_or_else(|| self.active_id(), Document::id)
    }

    pub fn active(&self) -> &Document {
        &self.open[self.active]
    }

    pub fn active_mut(&mut self) -> &mut Document {
        &mut self.open[self.active]
    }

    /// The document being DRAWN and edited: the hosted Work part while one is
    /// edited in place, else the active tab ([`Self::active`] stays the tab,
    /// for its name, dirty state and Save).
    pub fn view(&self) -> &Document {
        self.hosted().unwrap_or(&self.open[self.active])
    }

    pub fn view_mut(&mut self) -> &mut Document {
        if self.work_part_in_place() {
            return self.hosted_mut().unwrap();
        }
        &mut self.open[self.active]
    }

    pub fn active_index(&self) -> usize {
        self.active
    }

    /// The ACTIVE document's handle — the shell compares it frame to frame to
    /// notice a switch from ANY source (a tab click, a close, New, Open, Open
    /// Part, a crash-recovery restore) with one check instead of a hook per site.
    pub fn active_id(&self) -> u64 {
        self.open[self.active].id
    }

    pub fn len(&self) -> usize {
        self.open.len()
    }

    pub fn get(&self, index: usize) -> Option<&Document> {
        self.open.get(index)
    }

    pub fn get_mut(&mut self, index: usize) -> Option<&mut Document> {
        self.open.get_mut(index)
    }

    pub fn iter(&self) -> std::slice::Iter<'_, Document> {
        self.open.iter()
    }

    pub fn iter_mut(&mut self) -> std::slice::IterMut<'_, Document> {
        self.open.iter_mut()
    }

    /// Land every open document's finished runs; `true` while any of them still
    /// has work in flight, so the frame loop keeps pumping.
    ///
    /// EVERY open document is pumped, not just the active one: a run belongs to
    /// the engine that submitted it (each document owns its own runner), so a
    /// run still in flight when the user switches tabs must land in ITS
    /// document rather than be dropped or, worse, applied to whatever is on
    /// screen. An idle document's pump is a couple of empty `try_recv`s.
    ///
    /// The kernel's parts library is one store for every tab, and a pump can
    /// SUBMIT a run as well as land one: `EngineState::pump` re-runs after its
    /// runner refused a run for a library it no longer holds, after a ports
    /// edit moved a point, and when a mesh reconstruction lands (its STEP is
    /// imported, which writes the store into the document and runs). A submit hands the runner whatever library is
    /// in the store, and a run that lands mirrors the store it ran with into
    /// its document's own `partsLibrary` block. So a tab in the BACKGROUND that
    /// may do either — a run in flight, or a point move waiting — is pumped with
    /// ITS library installed, and the active document's is put back right after.
    /// (A STEP probe's reply only stashes what it found, and landing a run
    /// submits nothing, so those need no more than `run_pending`.)
    /// Pumped with the active one's, it resolved its components against another
    /// document's parts, and the run that landed wrote that into its own file.
    /// An idle background tab submits and lands nothing, so it costs nothing.
    pub fn pump_all(&mut self) -> bool {
        use brep_render::brep_kernel::parts_library_revision;
        let mut work_in_flight = false;
        let in_place = self.work_part_in_place();
        for index in 0..self.open.len() {
            // While a Work part is edited in place, its assembly tab is pumped
            // as a background document: the store holds the PART's library.
            let background = index != self.active || in_place;
            // Only a frame where something lands needs the document's own
            // library: a run in flight is otherwise polled and left, and
            // swapping the libraries both ways on every frame of it cost
            // ~30 ms a frame on l500 (a Work part edit in place).
            let doc = &mut self.open[index];
            let own_library = background
                && (doc.engine.run_landing()
                    || doc.engine.mesh_imports_pending()
                    || doc.engine.history.ports_follow_pending());
            let doc = &self.open[index];
            if own_library {
                install_library_of(doc);
            }
            let revision = parts_library_revision();
            let doc = &mut self.open[index];
            if background && !own_library && doc.engine.run_pending() {
                doc.engine.pump_holding_runs();
            } else {
                doc.engine.pump();
            }
            work_in_flight |= doc.engine.run_pending()
                || doc.engine.queries_pending()
                || doc.engine.mesh_imports_pending()
                || doc.engine.step_probes_pending()
                || doc.engine.topology_pending()
                || doc.engine.sheet_lines_pending();
            if own_library || (background && parts_library_revision() != revision) {
                self.install_active_library();
            }
        }
        // The hosted Work part (edited in place) is the foreground engine: the
        // store already holds its library.
        if let Some(doc) = self.hosted_mut() {
            doc.engine.pump();
            let engine = &doc.engine;
            work_in_flight |= engine.run_pending()
                || engine.queries_pending()
                || engine.mesh_imports_pending()
                || engine.step_probes_pending()
                || engine.topology_pending()
                || engine.sheet_lines_pending();
        }
        self.sync_work_part();
        self.begin_next_work_part();
        work_in_flight
    }

    /// Compare identities the way the store does (the shell installs it).
    pub fn set_canonicalizer(&mut self, canon: crate::store::Canonicalizer) {
        self.canon = canon;
        self.forget_published();
    }

    /// The tab holding the document `key` names, in the store's spelling.
    pub fn index_of_source(&self, key: &str) -> Option<usize> {
        let wanted = (self.canon)(key);
        self.open.iter().position(|doc| doc.name.as_deref().is_some_and(|name| (self.canon)(name) == wanted))
    }

    /// The tab holding the document stored under `name`, if any.
    pub fn index_of(&self, name: &str) -> Option<usize> {
        self.open
            .iter()
            .position(|doc| doc.name.as_deref() == Some(name))
    }

    /// Activate the tab already holding `name`; `false` when it is not open.
    /// The "focus the existing tab" half of open-or-focus.
    pub fn focus_named(&mut self, name: &str) -> bool {
        match self.index_of(name) {
            Some(index) => {
                self.activate(index);
                true
            }
            None => false,
        }
    }

    /// Add `doc` as a new tab and make it active.
    pub fn open_document(&mut self, mut doc: Document) -> usize {
        if doc.name.is_none() {
            let taken: Vec<u32> = self.open.iter().filter(|d| d.name.is_none()).map(|d| d.untitled).collect();
            doc.untitled = (1..).find(|n| !taken.contains(n)).unwrap_or(1);
        }
        self.open.push(doc);
        let index = self.open.len() - 1;
        self.activate(index);
        index
    }

    /// Number the untitled tabs 1, 2, … in tab order. Only for a session being
    /// built before anyone has seen its tabs — the autosave restore, which adds
    /// the recovered documents beside the boot seed and then closes the seed,
    /// so the seed's "untitled" must not push a recovered tab to "untitled 2".
    pub(crate) fn renumber_untitled(&mut self) {
        for (n, doc) in self.open.iter_mut().filter(|d| d.name.is_none()).enumerate() {
            doc.untitled = n as u32 + 1;
        }
    }

    /// Make tab `index` active, carrying the session's display settings over to
    /// it. Out-of-range indices are ignored (a stale click never panics).
    pub fn activate(&mut self, index: usize) {
        if index >= self.open.len() || index == self.active {
            return;
        }
        if self.work_part.is_some() {
            if let Err(error) = self.finish_work_part() {
                self.engine_mut().push_notice(format!("Work part: {error}"));
                return;
            }
        }
        let settings = self.open[self.active].engine.settings_json();
        self.publish_live(self.active);
        self.active = index;
        carry_settings(&settings, &mut self.open[index].engine, false);
        self.pull_live(index);
        self.install_active_library();
    }

    /// Put the ACTIVE document's parts library into the kernel's store.
    ///
    /// That store is one per thread, not one per document: every tab's engine
    /// resolves its ACOMP features against it and hands it to its runner, and a
    /// document seeds it only as it loads. So opening a second document — a New
    /// tab is enough — replaced it, and switching back left the assembly running
    /// against the other document's parts. Its components lost every connection
    /// point, and the next Diagram wire read "has no connection point named '2'
    /// in port 'Pins'" and took the harness connections already drawn with it
    /// (the eCAD workflow audit, issue 13). The block in the document's own
    /// history is the library that document last ran with, so it is the one to
    /// put back; an entry the store already holds unchanged is kept as it is,
    /// heal included (`install_parts_library`).
    fn install_active_library(&self) {
        install_library_of(self.hosted().unwrap_or(&self.open[self.active]));
    }

    /// Close tab `index`. Closing the LAST document leaves a fresh untitled one
    /// (the always-one-document invariant); closing a tab before the active one
    /// keeps the same document active at its new index.
    ///
    /// The caller is responsible for the unsaved-changes prompt — this is the
    /// mechanical close (`panels::file` owns the confirmation).
    pub fn close(&mut self, index: usize) {
        if index >= self.open.len() {
            return;
        }
        if self.work_part.as_ref().is_some_and(|session| {
            let id = self.open[index].id();
            id == session.assembly_id || id == session.part_id
        }) {
            self.engine_mut().push_notice("Return to assembly before closing a Work part session document");
            return;
        }
        // Taken BEFORE the removal: if the closing tab was the active one, its
        // settings are the session's and must survive into whatever is shown next.
        let settings = self.open[self.active].engine.settings_json();
        // Same reason as `spawn_engine`: a refill built from the factory starts
        // on the 800x600 default camera, so carry the live viewport size too.
        let viewport = self.viewport_size();
        let closed_active = index == self.active;
        // A tab closed with its changes SAVED leaves them to the linked
        // documents; one closed with them discarded does not.
        if closed_active && !self.open[index].is_dirty() {
            self.publish_live(index);
        }
        let closed = self.open.remove(index);
        self.forget_live(closed.id());
        // The replacement for a session closed down to nothing is a brand-new
        // tab, so it takes the WHOLE of the session's settings — workbench
        // included — exactly as `New` does through `spawn_engine`.
        let refilled = self.open.is_empty();
        if refilled {
            let mut engine = (self.new_engine)();
            carry_settings(&settings, &mut engine, true);
            self.open.push(Document::fresh(engine, crate::document_class::DocumentClass::Normal));
            self.active = 0;
            self.open[0].engine.resize(viewport.0, viewport.1);
        } else if closed_active {
            self.active = index.min(self.open.len() - 1);
        } else if index < self.active {
            self.active -= 1;
        }
        if closed_active {
            let active = self.active;
            carry_settings(&settings, &mut self.open[active].engine, refilled);
            self.pull_live(active);
            self.install_active_library();
        }
    }

    /// Refresh every tab's dirty dot (see [`Document::refresh_dirty_marker`]).
    /// The write of document `name` failed after its Save returned: the tabs
    /// showing it (any spelling of the same identity) read unsaved again.
    /// Whether one did.
    pub fn mark_write_failed(&mut self, store: &dyn crate::store::ModelStore, name: &str) -> bool {
        let failed = store.canonical_identity(name);
        let mut marked = false;
        for doc in &mut self.open {
            if doc.name().is_some_and(|n| store.canonical_identity(n) == failed) {
                doc.mark_write_failed();
                marked = true;
            }
        }
        marked
    }

    pub fn refresh_dirty_markers(&mut self) {
        for doc in &mut self.open {
            owned_save::note_inserted_copies(doc);
            doc.settle_open_runs();
            doc.converge_runs();
            doc.refresh_dirty_marker();
        }
    }
}

/// Put `doc`'s own parts library — the block in its history, the library it
/// last ran with — into the kernel's store (`Documents::install_active_library`).
fn install_library_of(doc: &Document) {
    let library = serde_json::from_value(doc.engine.history.parts_library().clone()).unwrap_or_default();
    brep_render::brep_kernel::install_parts_library(&library);
}

/// Copy the SESSION-scoped display settings (theme, UI scale, colors, wireframe,
/// projection, lod…) from a `settings_json()` snapshot into `target`.
/// `with_workbench` is true ONLY when seeding a brand-new engine — see the
/// module header for why activation must never carry it.
fn carry_settings(settings_json: &str, target: &mut EngineState, with_workbench: bool) {
    let Ok(mut settings) = serde_json::from_str::<serde_json::Value>(settings_json) else {
        return;
    };
    if !with_workbench {
        if let Some(object) = settings.as_object_mut() {
            object.remove("workbench");
        }
    }
    let _ = target.apply_settings_json(&settings.to_string());
}

/// Hand out the next document handle. A plain process counter: handles only ever
/// need to be distinct WITHIN a session, and they never reach storage.
fn next_document_id() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NEXT: AtomicU64 = AtomicU64::new(1);
    NEXT.fetch_add(1, Ordering::Relaxed)
}

