//! Open documents and their active tab. Each document owns an engine, store
//! identity, and clean baseline for dirty tracking. Keeping engine access on
//! `Documents` allows separate mutable borrows of other app fields.
//!
//! Every engine has its own history runner and resident kernel state. All open
//! documents are pumped each frame, so switching tabs cannot redirect a pending
//! result to another document. Each open document retains its runner and scene.
//!
//! Display settings are shared across tabs by [`carry_settings`]; workbench
//! selection belongs to each document. A new engine initially inherits the
//! current workbench, which a loaded document can override.

use brep_render::engine_state::EngineState;

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

/// The empty model a **New** document starts from.
pub const EMPTY_DOCUMENT: &str = r#"{"expressions":"","configurator":{},"features":[]}"#;

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
    /// PLM write permission is independent of editing the session copy.
    plm_save_access: Option<Access>,
    /// The model request JSON as of the last New / Open / Save — the baseline
    /// the dirty flag compares the live history against.
    saved_signature: Option<String>,
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
    /// The broken-reference lines [`crate::panels::ecad_parts::sync`] has
    /// already put in front of the user. A device whose part is missing is a
    /// STANDING state and the revision bumps on every edit, so without this it
    /// would toast on every keystroke — the same rule
    /// [`ecad::BlockSync::unreported`] keeps for an UNLINKED component.
    pub ecad_parts_reported: Vec<String>,
    /// What [`crate::panels::ecad_parts::follow_occurrences`] last saw of the
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
fn signature(engine: &EngineState) -> String {
    signature_without(engine, None)
}

/// The saved-content signature with one document block left out — the
/// baseline of a copy saved WITHOUT that block (a family member saved as a new
/// part drops `familySource` on the copy), in the same canonical form.
fn signature_without(engine: &EngineState, without: Option<&str>) -> String {
    let mut document: serde_json::Value =
        serde_json::from_str(&engine.history.request_json()).unwrap_or_else(|_| serde_json::json!({}));
    if let Some(object) = document.as_object_mut() {
        if let Some(key) = without {
            object.remove(key);
        }
        if !engine.metadata.is_empty() {
            object.insert("metadata".into(), engine.metadata.to_json());
        }
    }
    document.to_string()
}

/// One [`ecad::BlockSync`] per eCAD block, none of them pulled yet.
fn ecad_blocks() -> [ecad::BlockSync; 4] {
    use ecad::Block;
    [Block::Diagram, Block::Pcb, Block::Symbol, Block::Pads].map(ecad::BlockSync::new)
}

impl Document {
    /// Wrap `engine` as a document, taking its CURRENT model as the clean
    /// baseline — so a freshly loaded (or freshly emptied) document is clean and
    /// the first edit marks it dirty.
    pub fn new(engine: EngineState) -> Self {
        let saved_signature = Some(signature(&engine));
        Self {
            engine,
            name: None,
            plm_save_access: None,
            saved_signature,
            dirty_marker: false,
            marker_key: None,
            dirty_checks: 0,
            id: next_document_id(),
            ecad: crate::workbench::ecad::Editors::new(),
            ecad_blocks: ecad_blocks(),
            ecad_parts_revision: None,
            ecad_parts_reported: Vec::new(),
            ecad_occurrences: None,
            ecad_outdated: Vec::new(),
            ecad_update_requested: false,
            refused_seen: 0,
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
            saved_signature: None,
            dirty_marker: true,
            marker_key: None,
            dirty_checks: 0,
            id: next_document_id(),
            ecad: crate::workbench::ecad::Editors::new(),
            ecad_blocks: ecad_blocks(),
            ecad_parts_revision: None,
            ecad_parts_reported: Vec::new(),
            ecad_occurrences: None,
            ecad_outdated: Vec::new(),
            ecad_update_requested: false,
            refused_seen: 0,
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

    /// The tab label: PLM part/revision, the bare file name, or `untitled`.
    pub fn title(&self) -> String {
        match &self.name {
            Some(name) => crate::panels::plm_host::revision_of(name)
                .map(|(part, revision)| format!("{part}/{revision}"))
                .unwrap_or_else(|| crate::store::model_display_name(name)),
            None => "untitled".to_string(),
        }
    }

    /// Snapshot the current model as the clean baseline (after New / Open /
    /// Save, or a `?loadModel=` boot-load).
    pub fn mark_clean(&mut self) {
        self.saved_signature = Some(signature(&self.engine));
        self.dirty_marker = false;
        self.marker_key = Some(self.edit_key());
    }

    pub(crate) fn save_signature(&self) -> String { signature(&self.engine) }

    /// [`Self::save_signature`] of the document with one block left out: what a
    /// copy saved without that block has as its clean baseline.
    pub(crate) fn save_signature_without(&self, block: &str) -> String {
        signature_without(&self.engine, Some(block))
    }

    pub(crate) fn mark_saved_signature(&mut self, saved: String) {
        self.saved_signature = Some(saved);
        self.marker_key = None;
        self.refresh_dirty_marker();
    }

    /// Dirty = the live model differs from the last saved/opened baseline.
    /// (Rolling the history does NOT change the request document, so navigating
    /// steps never marks dirty — only real edits/add/delete/reorder do.)
    /// The workbench the document is in is not part of it ([`signature`]).
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
    /// change is not an unsaved change at all, [`signature`]) moves neither and can
    /// leave the dot one edit behind: a marker briefly optimistic, never a lost
    /// edit, since [`Self::is_dirty`] is recomputed honestly at the close prompt.
    pub fn edit_key(&self) -> (u64, u64) {
        (self.engine.applied_generation(), self.engine.history_revision())
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
                    None => base_map.remove(name),
                };
            }
            Some(base.to_string())
        });
        if let Some(rebased) = rebased {
            self.saved_signature = Some(rebased);
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
    open: Vec<Document>,
    active: usize,
    /// How a new tab's engine is built (platform runner + persisted settings).
    new_engine: EngineFactory,
}

impl Documents {
    /// A session holding ONE empty document built by `new_engine`.
    pub fn new(new_engine: EngineFactory) -> Self {
        let first = Document::new(new_engine());
        Self {
            open: vec![first],
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

    pub fn engine(&self) -> &EngineState {
        &self.open[self.active].engine
    }

    pub fn engine_mut(&mut self) -> &mut EngineState {
        &mut self.open[self.active].engine
    }

    pub fn active(&self) -> &Document {
        &self.open[self.active]
    }

    pub fn active_mut(&mut self) -> &mut Document {
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
        for index in 0..self.open.len() {
            let background = index != self.active;
            let doc = &self.open[index];
            let own_library = background
                && (doc.engine.run_pending()
                    || doc.engine.mesh_imports_pending()
                    || doc.engine.history.ports_follow_pending());
            if own_library {
                install_library_of(doc);
            }
            let revision = parts_library_revision();
            let doc = &mut self.open[index];
            doc.engine.pump();
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
        work_in_flight
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
    pub fn open_document(&mut self, doc: Document) -> usize {
        self.open.push(doc);
        let index = self.open.len() - 1;
        self.activate(index);
        index
    }

    /// Make tab `index` active, carrying the session's display settings over to
    /// it. Out-of-range indices are ignored (a stale click never panics).
    pub fn activate(&mut self, index: usize) {
        if index >= self.open.len() || index == self.active {
            return;
        }
        let settings = self.open[self.active].engine.settings_json();
        self.active = index;
        carry_settings(&settings, &mut self.open[index].engine, false);
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
        install_library_of(&self.open[self.active]);
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
        // Taken BEFORE the removal: if the closing tab was the active one, its
        // settings are the session's and must survive into whatever is shown next.
        let settings = self.open[self.active].engine.settings_json();
        // Same reason as `spawn_engine`: a refill built from the factory starts
        // on the 800x600 default camera, so carry the live viewport size too.
        let viewport = self.viewport_size();
        let closed_active = index == self.active;
        self.open.remove(index);
        // The replacement for a session closed down to nothing is a brand-new
        // tab, so it takes the WHOLE of the session's settings — workbench
        // included — exactly as `New` does through `spawn_engine`.
        let refilled = self.open.is_empty();
        if refilled {
            self.open.push(Document::new((self.new_engine)()));
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
            self.install_active_library();
        }
    }

    /// Refresh every tab's dirty dot (see [`Document::refresh_dirty_marker`]).
    pub fn refresh_dirty_markers(&mut self) {
        for doc in &mut self.open {
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

