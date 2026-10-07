//! `BrepApp` — the THIN shell of the engine-native UI.
//!
//! One eframe [`App`] that hosts the EXISTING `brep-render` engine and lays out
//! the panels. The heavy lifting lives in focused modules; this file only owns
//! the shell:
//!
//! * [`Documents`] (`crate::document`) holds the OPEN MODELS — one
//!   [`EngineState`] per document plus its identity, exactly one active. Panels
//!   borrow the active engine through `self.docs.engine_mut()`, which borrows
//!   only that FIELD and so still composes with the disjoint panel borrows
//!   beside it. [`EngineState`] (`brep-render`) is still the single
//!   windowing-agnostic BRAIN per document (scene / camera / controls / settings
//!   / widgets + pointer/wheel/viewcube/pick); we do NOT fork it.
//! * [`crate::viewport::Viewport`] draws + drives the central 3D viewport (the
//!   offscreen texture, the `egui_wgpu` blit callback, input routing).
//! * [`crate::panels`] — one module per left-panel section, each a small state
//!   struct + a `show(&mut self, ui, state, …)` method. Adding a panel = add
//!   `panels/<name>.rs`, one field here, one `self.<name>.show(…)` call in
//!   [`eframe::App::ui`] below (see `README.md` → "Adding a panel").
//!
//! Native (`run_native`) and wasm (`WebRunner`) run this SAME code.

use crate::document::{Documents, EngineFactory};
use crate::panels::assembly_constraints::AssemblyConstraintsPanel;
use crate::panels::bug_report::BugReportPanel;
use crate::panels::wire_harness::WireHarnessPanel;
use crate::panels::component_actions::ComponentActionRequest;
use crate::panels::context_bar::ContextBarPanel;
use crate::panels::mode_bar::ModeBar;
use crate::panels::expressions::ExpressionsPanel;
use crate::panels::file::{FileAction, FileDialog};
use crate::panels::history::HistoryPanel;
use crate::panels::info_windows::InfoWindows;
use crate::panels::scene::ScenePanel;
use crate::panels::selection::SelectionPanel;
use crate::panels::sketch::SketchPanel;
use crate::panels::part_properties::PartPropertiesPanel;
use crate::panels::settings::SettingsPanel;
use crate::panels::toasts::Toasts;
use crate::panels::toolbar::ToolbarPanel;
use crate::panels::workbench_toolbar::WorkbenchToolbarPanel;
use crate::panels::update_components::UpdateComponents;
use crate::panels::bom::BomPanel;
use crate::panels::dock::{DockContext, DockState, PaneKind};
use crate::panels::document_tabs;
use crate::store::{default_model_store, ModelStore, SETTINGS_KEY};
use crate::viewport::Viewport;
use brep_render::engine_state::EngineState;
use brep_render::style::ThemeMode;
use eframe::egui;

/// A measured camera spin: an orbit drag the app steps ITSELF, one pointer move
/// per frame, from `perf_spin`. See [`BrepApp::spin`] for why it is not driven
/// from outside.
///
/// The steps run through the same `EngineState::pointer_down/move/up` the
/// viewport's real drag uses (see `viewport::interaction`), so a spin costs
/// exactly what dragging the mouse costs — no shortcut past the controls.
#[cfg(feature = "automation")]
pub struct Spin {
    /// Pointer steps still to make; the drag ends (pointer_up) on the last one.
    pub left: u32,
    /// Where the drag started, in viewport-local logical px.
    pub origin: (f64, f64),
    /// Pointer step, in viewport-local logical px per frame.
    pub step: (f64, f64),
    /// Steps already made — the drag is a straight sweep from `origin`, and
    /// reversing at the halfway mark keeps it inside the viewport however many
    /// frames are asked for.
    pub made: u32,
    /// False until the first frame has pressed the button.
    pub pressed: bool,
}

pub struct BrepApp {
    /// Every OPEN MODEL and which one is active. Each document owns a full
    /// `EngineState` (the shared viewer brain `desktop.rs` / the wasm shell
    /// wrap) plus its file identity; panels borrow the active one.
    pub(crate) docs: Documents,
    /// The central 3D viewport: engine render core + offscreen texture + blit +
    /// input routing.
    pub(crate) viewport: Viewport,
    /// The single persistence seam for settings, layout, and model documents
    /// (native filesystem / wasm IndexedDB + download/upload).
    pub(crate) model_store: Box<dyn ModelStore>,

    // --- one small state value per panel --------------------------------------
    /// Top toolbar: undo/redo, wireframe toggle, zoom-to-fit + standard views,
    /// and the File-actions seam (owned by the concurrent file panel).
    pub(crate) toolbar: ToolbarPanel,
    /// New / Open / Save / Save As of the model document (the `.nbrep`
    /// recipe) — a reusable modal file dialog opened from the toolbar.
    file: FileDialog,
    stl_import: Option<crate::panels::stl_import::StlImportPreview>,
    /// The "Submit Bug" flow: on the toolbar bug button it screenshots the app
    /// (UI + 3D model) BEFORE its own dialog opens, then collects a description
    /// (+ optional email) and POSTs the model + screenshot to the public reports
    /// endpoint. Native + wasm, one path.
    bug_report: BugReportPanel,
    /// The Info window: the licences and this session's diagnostics, toggled
    /// from the toolbar's info button.
    info: crate::panels::info::InfoPanel,
    /// What this session is running on — captured ONCE, from the adapter eframe
    /// actually gave us (never re-probed), and read by BOTH the Info window and
    /// the problem report so the two cannot disagree. See
    /// [`crate::diagnostics`].
    diagnostics: crate::diagnostics::Diagnostics,
    /// Workbench actions toolbar: the second top strip under the primary
    /// toolbar — one button per feature the active workbench offers, plus the
    /// constraint types where the Constraints panel is shown. Gated by the
    /// `showWorkbenchToolbar` setting and hidden in the special modes. A click
    /// flows out as the type to add; the shell adds it through the SAME paths
    /// the palette / context bar use.
    workbench_toolbar: WorkbenchToolbarPanel,
    /// Display-settings + per-solid color panel (Phase 1): a FLOATING window
    /// (movable + resizable, toggled from the toolbar gear button), no longer a
    /// left-panel section.
    settings: SettingsPanel,
    pub(crate) plugins: crate::plugins::PluginsPanel,
    pub(crate) javascript: crate::javascript::JavaScriptEditor,
    /// The active document's OWN BOM attributes (Part Number, Material, Mass,
    /// …) as a floating window, toggled from the toolbar's Properties button.
    /// Document-level, not selection-level: entity inspection is the context
    /// bar's `info_windows`.
    part_properties: PartPropertiesPanel,
    /// History feature-tree + schema-driven feature dialog panel (Phase 2).
    /// NOTE: the editable history is NOT owned here — it lives in the engine core
    /// (`EngineState.history`), the single source of truth; this panel only reads
    /// it back to draw and calls the engine's `history_*` methods to mutate.
    history: HistoryPanel,
    /// Scene tree ("Scene Manager"): the display scene as a file-tree — per-solid
    /// visibility + Faces/Edges/Vertices with two-way selection sync. Reads the
    /// engine scene/emphasis; owns only transient expand + hit state.
    scene: ScenePanel,
    /// Assembly Structure tree (claimed by the Assembly workbench): a VIEW over
    /// the scene's component records — per-instance fixed/visibility/status
    /// adornments, actions routed to the owning ACOMP feature.
    /// The BOM panel (claimed by the Assembly workbench): the parts list on
    /// the shared column-tree widget, with the editable part/occurrence
    /// attribute columns the Settings "Assemblies" section configures.
    bom: BomPanel,
    /// Assembly Constraints panel (claimed by the Assembly workbench): the
    /// schema-driven constraint collection widget + Solve/auto-solve/DOF header.
    assembly_constraints: AssemblyConstraintsPanel,
    /// The wire-harness connection list (a Wire harness workbench pane): add /
    /// edit / remove wires, read their routed length + status, hover to
    /// highlight. Document data lives in the engine; this holds the widget's
    /// transient state.
    wire_harness: WireHarnessPanel,
    /// The PMI view tree + annotation forms (the PMI workbench's pane).
    pmi: crate::panels::pmi::PmiPanel,
    /// The drawing sheets + their forms (the Drawing workbench's pane).
    sheets: crate::panels::sheets::SheetsPanel,
    /// Update-components checker: compares each parts-library entry's
    /// `sourceSignature` against the model store's current content. Kept
    /// current once per frame (cheap generation key: applied run + store
    /// save); the constraints header reads the count + runs the batch refresh,
    /// the structure tree reads per-part badges.
    update_components: UpdateComponents,
    /// Expressions / parameters panel: the variable sheet (engine-owned history
    /// `expressions`) feature params reference. Owns only its editor buffer.
    expressions: ExpressionsPanel,
    /// Qualify: the part's declared CONNECTION POINTS, on whichever surface is
    /// on screen (symbol, pads or 3D). Holds only the selected address; the
    /// block and both consistency reports come from the engine.
    qualify: crate::panels::qualify::QualifyPanel,
    /// The family table editor: a `.fbrep` family seed's member table. Holds
    /// the selection, the cell being typed and the last Generate report; the
    /// table itself is the document's `familyTable` block.
    family_table: crate::panels::family_table_editor::FamilyTableEditor,
    /// Every PLM panel (`panels::plm_host`): empty in a file-based session.
    pub(crate) plm: crate::panels::plm_host::PlmHost,
    /// Info windows: MULTIPLE pinned per-entity inspector windows opened from the
    /// selection-driven context bar's Info action. Each floating (movable +
    /// resizable) window is PINNED to one object name at open time — a Metadata
    /// (editable attribute) tab + a read-only Info (measurements + provenance) tab —
    /// and keeps showing that entity regardless of later selection changes. Replaces
    /// the old single Properties window.
    info_windows: InfoWindows,
    /// Interference results window: opened by the Assembly workbench's `∩`
    /// toolbar button, which runs the engine's pairwise-intersect check; a
    /// floating window like the Info windows with a row per interfering pair
    /// (click = select both components), a green all-clear pass line, and a
    /// Re-run button.
    interference: crate::panels::interference::InterferenceWindow,
    /// Auto Constraints window (Assembly workbench): opened by the `⚿` toolbar
    /// button, it lists the constraint types the kernel's inference lane can
    /// read out of the components' current placement, with what a scan found
    /// for each, and creates the whole accepted set as one undo step.
    auto_constraints: crate::panels::auto_constraints::AutoConstraintsWindow,
    /// step.parts online model library browser (Assembly workbench): a ctx-level
    /// window (opened by the library toolbar button) that searches the public
    /// step.parts v1 API, shows results with thumbnails, and imports a chosen
    /// STEP model as a new part document + adds it to the assembly as an ACOMP.
    step_parts: crate::panels::step_parts::StepPartsPanel,
    /// Selection panel: the pickable-kinds filter (which entity kinds a viewport
    /// click may select — honored by the engine's `select_top_at`). The filter +
    /// selection live in `EngineState`; this panel only reads/writes them.
    selection: SelectionPanel,
    /// Context action toolbar: the selection-driven action bar (Clear / Hide /
    /// Edit-owning-feature + the feature-from-selection actions whose primary
    /// reference accepts the selected kind). Shown only while something is
    /// selected; drives the engine directly and returns a feature id for the shell
    /// to expand in the history tree.
    context_bar: ContextBarPanel,
    /// Sketch (S0): a seeded, read-only sketch preview — pushes a solved rectangle
    /// + circle to the `set_overlay` channel colored by solver mobility, and shows
    /// the DOF status readout. The engine-native sketcher's foundation surface.
    sketch: SketchPanel,
    /// Special-mode EXIT controls (Finish/Cancel), always pinned to the top-right
    /// corner — reference-selection, sketch mode, and any future special mode.
    mode_bar: ModeBar,
    /// Transient toast overlay: drains the engine's queued notices each frame
    /// (e.g. a sketch solve that failed after an edit) and shows each briefly.
    pub(crate) toasts: Toasts,
    /// The status bar's app-wide working indicator (`panels::busy`): what is
    /// in flight, collected from every source each frame.
    busy: crate::panels::busy::BusyIndicator,
    /// Dockable / tabbed side-panel layout (egui_tiles): the shared, persisted
    /// tree that hosts every side-panel section AND the 3D viewport as tiles the
    /// user can split, tab, resize, and drag-rearrange. Owns the layout; borrows
    /// each panel + the engine per frame through [`DockContext`]. Drawn in normal
    /// modeling mode; sketch / ref-select mode bypasses it (viewport drawn direct).
    dock: DockState,

    /// Whether the ONE-SHOT first-model framing has fired. The seed run is async
    /// under a background runner (native thread / wasm worker), so the boot
    /// `zoom_to_fit` can run before the first solids exist → an unframed first
    /// model. Once the seed run has landed (`has_solids() && !run_pending()`), the
    /// `ui` loop frames it once and sets this. Under the synchronous Inline runner
    /// (tests) the scene is already populated, so this fires on the very first frame.
    first_run_framed: bool,

    /// A model fetch kicked off at boot from a `?loadModel=<url>` query param
    /// (wasm only). When the fetch lands it REPLACES the seed model. `None` on
    /// native and once applied.
    pending_boot_load: Option<std::sync::mpsc::Receiver<Result<String, String>>>,

    /// "Open in CAD" from the PLM's page: the `?open=part/…/rev/…` this page
    /// was started with (wasm only), driven once a frame until it settles.
    plm_launch: Option<crate::panels::plm_launch::Launch>,

    /// The document handle the shared panels were last reset for. Compared to
    /// `docs.active_id()` at the top of every frame: ONE check catches a switch
    /// from any source (a tab click, a close, New, Open, Edit Part) instead of a
    /// hook per call site, and it runs BEFORE any panel draws this frame.
    active_document: u64,

    /// Whether the active workbench showed the Sheets pane LAST frame — the
    /// edge the paper is closed on. Closing is a transition, not a condition:
    /// a sheet opened while another workbench is active (a command, a script)
    /// stays open until the Drawing workbench is LEFT, exactly as PMI's view
    /// deactivates on leaving rather than on every frame outside it.
    sheets_pane_shown: bool,

    /// The DOCUMENT TAB STRIP's per-tab hit-rects from the last dock frame,
    /// published for the headed verifier. The strip is drawn inside the dock's
    /// viewport pane, so its rects have to ride back out through the outcome.
    document_tab_hits: Vec<(String, egui::Rect)>,

    /// The `__brepParams` blob and the document state it was built from:
    /// `(document, rolled-to step, history revision, json)`.
    ///
    /// The blob is the rolled-to feature's whole `inputParams`, and for an
    /// `IMPORT3D` those params carry the embedded model text — 681 KB on the
    /// heaviest corpus model. It was rebuilt EVERY frame, which on the browser
    /// build (where the state registry is always on) also transcoded it into a
    /// fresh JS string every frame, on the one thread that also has to answer
    /// the pointer. Nothing reads it per frame; it only has to be correct when
    /// someone asks. The revision is a mutation counter the document hands out
    /// (`EngineState::history_revision`), so this cannot go stale without the
    /// borrow checker having let a mutation past the door that bumps it.
    params_blob: (u64, usize, u64, String),

    /// A measured camera spin in flight (`perf_spin`): the orbit drag the app
    /// drives ITSELF, one pointer step per frame, so the frames run back to back.
    /// Driving it from a host instead would put a round trip between every pair
    /// of frames, and `dt` — the frame period, the number a user feels as a
    /// freeze — would measure the host's latency rather than the app's.
    #[cfg(feature = "automation")]
    spin: Option<Spin>,

    /// The debounced autosave of every dirty document (`crate::recovery`), and
    /// the boot-time **Recover unsaved work?** prompt its blob feeds. The
    /// autosave is held while the prompt is open: the clean seed tab would
    /// otherwise remove the very blob being offered.
    autosave: crate::recovery::Autosave,
    recovery: crate::recovery::RecoveryPanel,

    /// The UI zoom scale CURRENTLY applied to the egui context. Tracks
    /// `settings.ui_scale` but is only synced to it while the pointer is up, so
    /// dragging the Settings "UI scale" slider doesn't rescale the whole UI under
    /// the cursor mid-drag — the settled value is committed on release. See the
    /// zoom-apply block in `ui`.
    applied_ui_scale: f32,
    /// The automation channel (hosts submit commands; the frame drains them at
    /// three fixed points — see `automation::queue`).
    #[cfg(feature = "automation")]
    pub(crate) automation: std::sync::Arc<crate::automation::queue::AutomationQueue>,
}

impl BrepApp {
    /// The automation queue a host submits commands to.
    #[cfg(feature = "automation")]
    pub fn automation(&self) -> &std::sync::Arc<crate::automation::queue::AutomationQueue> {
        &self.automation
    }

    /// Open a `.nbrep` file named on the command line — `brep-app model.nbrep`,
    /// which is also what a desktop file association hands us (`Exec=… %f`).
    ///
    /// The path goes through the same `FileDialog::open_document` door as
    /// File>Open, so the document keeps the full path as its identity and a
    /// later plain Save writes back to the file the user double-clicked, rather
    /// than to a copy in the app's own models directory. A path that does not
    /// parse leaves the status line saying so and the seed document in place —
    /// a bad argument must not cost the user a window.
    ///
    /// Native only: called once, before the first frame.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn open_path_from_command_line(&mut self, path: &std::path::Path) {
        // Absolute, so the store resolves it as a real file rather than as a
        // name inside the models directory (`store::resolve` routes anything
        // with a separator straight to the filesystem).
        let path = std::fs::canonicalize(path).unwrap_or_else(|_| path.to_path_buf());
        let Some(name) = path.to_str() else {
            log::warn!("brep-app: {} is not valid UTF-8; not opening it", path.display());
            return;
        };
        // Field-by-field, because `open_document` borrows the documents and the
        // store while `self.file` is borrowed mutably.
        let Self { file, docs, model_store, .. } = self;
        file.open_document(docs, model_store.as_ref(), name);
    }

    /// The 3D viewport rect of the last frame, in egui points.
    pub fn view_rect(&self) -> Option<egui::Rect> {
        self.viewport.last_rect()
    }

    /// The session's store, read-only (the automation layer's `frame_info`
    /// counts its writes still on their way).
    pub(crate) fn model_store(&self) -> &dyn ModelStore {
        self.model_store.as_ref()
    }

    /// Why this session cannot switch stores live, or `None` when it can: every
    /// open document untitled and clean, the state a fresh start has. A named
    /// document's identity belongs to the store it came from, and a dirty one's
    /// work has nowhere to go in a new store.
    pub(crate) fn reconnect_blocker(&self) -> Option<String> {
        self.reconnect_blocker_with(crate::document::Document::is_dirty)
    }

    fn reconnect_blocker_with(&self, is_dirty: impl Fn(&crate::document::Document) -> bool) -> Option<String> {
        let named = self.docs.iter().filter(|d| d.name().is_some()).count();
        let dirty = self.docs.iter().filter(|d| is_dirty(d)).count();
        match (named, dirty) {
            (0, 0) => None,
            (_, 0) => Some(format!("{named} document(s) from this store are open: close them, then Connect now (or restart)")),
            _ => Some(format!("{dirty} document(s) have unsaved work: save or close them, then Connect now (or restart)")),
        }
    }

    /// Switch this session to the PLM configured at `root`, live (S1's named
    /// limit): the native boot S2 runs at start, then everything the app read
    /// from the store when it was built is read again from the new one. Only
    /// while [`Self::reconnect_blocker`] is `None`. A refused boot changes
    /// nothing and says why.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn reconnect_plm(&mut self, root: std::path::PathBuf) -> Result<String, String> {
        use crate::plm::connection::Status;
        if let Some(why) = self.reconnect_blocker() {
            return Err(why);
        }
        let config = crate::plm::config::resolve(&root, &|_| None, &Default::default())?
            .ok_or_else(|| format!("no PLM is configured in {}", root.display()))?;
        let store = crate::store::boot_native_store(root, config);
        let Some(Status::Connected { url, username }) = store.plm_session().map(|s| s.status) else {
            let why = store.take_persistence_errors().join("; ");
            return Err(if why.is_empty() { "the PLM did not take this session".into() } else { why });
        };
        self.model_store = store;
        // Settings: every open engine, and every later tab, starts from the
        // defaults with the new store's settings over them, as a fresh boot
        // would; nothing of the old store's carries over.
        let saved = self.model_store.read(SETTINGS_KEY);
        let fresh = move |mut state: EngineState| {
            state.settings = brep_render::style::RenderSettings::default();
            let defaults = state.settings.to_json();
            let _ = state.apply_settings_json(&defaults);
            if let Some(saved) = &saved {
                let _ = state.apply_settings_json(saved);
            }
            state
        };
        for doc in self.docs.iter_mut() {
            let engine = std::mem::replace(&mut doc.engine, EngineState::new());
            doc.engine = fresh(engine);
        }
        self.docs.wrap_engine_factory(fresh);
        self.dock = DockState::new(self.model_store.as_ref());
        self.recovery.arm_from_store(
            self.model_store.as_ref(),
            self.docs.engine().settings.disable_recovery_prompt,
        );
        self.file = FileDialog::new();
        self.update_components = UpdateComponents::new();
        Ok(format!("connected to {url} as {username}"))
    }

    /// The file dialog, the documents and the store together — for the
    /// automation commands that drive a document-class flow (Generate, a
    /// store read-back) exactly as the panels do.
    pub(crate) fn file_docs_store(&mut self) -> (&mut FileDialog, &mut Documents, &dyn ModelStore) {
        (&mut self.file, &mut self.docs, self.model_store.as_ref())
    }

    /// The active document's engine.
    pub fn docs_engine(&self) -> &EngineState {
        self.docs.engine()
    }

    /// Open the mesh import preview for the file `name` (`.stl` or `.3mf`)
    /// against the active document — what activating it in the Import explorer
    /// does. The preview replaces the shell until it is accepted or cancelled.
    pub fn open_mesh_preview(&mut self, name: String, bytes: Vec<u8>) -> Result<(), String> {
        use brep_render::runner::MeshImportFormat;
        let format = match name.rsplit('.').next().map(str::to_ascii_lowercase).as_deref() {
            Some("stl") => MeshImportFormat::Stl,
            Some("3mf") => MeshImportFormat::ThreeMf,
            _ => return Err(format!("{name} is not an STL or 3MF file")),
        };
        self.start_mesh_preview(format, name, bytes);
        Ok(())
    }

    fn start_mesh_preview(&mut self, format: brep_render::runner::MeshImportFormat, name: String, bytes: Vec<u8>) {
        self.stl_import = Some(crate::panels::stl_import::StlImportPreview::new(
            self.docs.active_id(), format, name, bytes, self.docs.spawn_engine(),
        ));
        self.viewport.forget_document();
    }

    /// Whether the mesh import preview is open.
    pub fn mesh_preview_open(&self) -> bool {
        self.stl_import.is_some()
    }

    pub fn new(cc: &eframe::CreationContext<'_>) -> Result<Self, String> {
        Self::new_with(cc, crate::automation::AppOptions::default())
    }

    /// Build the app with host-supplied options: an isolated store (a host
    /// MUST pass one) and whether to start on the seed model.
    pub fn new_with(cc: &eframe::CreationContext<'_>, opts: crate::automation::AppOptions) -> Result<Self, String> {
        // Keep overflow discoverable without hovering or scrolling first, in
        // both themes and every child UI (including eCAD and plugin panels).
        // ScrollArea's default VisibleWhenNeeded still hides bars that aren't needed.
        cc.egui_ctx.all_styles_mut(|style| {
            style.spacing.scroll = egui::style::ScrollStyle::solid();
        });
        let crate::automation::AppOptions { store: opt_store, seed } = opts;
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .ok_or_else(|| "eframe was not created with a wgpu render state".to_string())?;

        // The viewport owns the render core + blit pipeline, built from eframe's
        // SHARED device/queue/format.
        let viewport = Viewport::new(render_state);

        // The session's diagnostics, taken from that same render state: the
        // adapter about to draw every frame is the one a report must name. The
        // WebGPU-vs-WebGL2 decision has already been made by the time we get
        // here (eframe drops `BROWSER_WEBGPU` from the backend set when the
        // browser offers no WebGPU adapter), so this READS the outcome — asking
        // again later would be a second question with its own answer.
        let diagnostics = crate::diagnostics::Diagnostics::from_render_state(render_state);

        // --- storage seam: load the persisted settings ------------------------
        let model_store = opt_store.unwrap_or_else(default_model_store);
        // wasm: hand the store the egui context so an async file-upload load
        // callback can wake the reactive frame loop (see `store::set_repaint_ctx`).
        #[cfg(target_arch = "wasm32")]
        crate::store::set_repaint_ctx(cc.egui_ctx.clone());
        let saved_settings = model_store.read(SETTINGS_KEY);

        // --- how a document's engine is built --------------------------------
        // Every tab gets its OWN engine, and therefore its own history runner —
        // a runner owns the resident kernel state of the document it executes,
        // so one shared between documents would apply a background run against
        // the wrong registry. See `crate::document`.
        let plugins = crate::plugins::PluginsPanel::new(model_store.as_ref());
        let javascript = crate::javascript::JavaScriptEditor::new(model_store.as_ref());
        let plugin_packages = plugins.packages.clone();
        let engine_factory: EngineFactory = Box::new(move || {
            let mut state = EngineState::new();
            // Native: run the whole history — and per-object measurement queries — on a
            // persistent background thread so the UI never freezes during a run or a
            // selection (M2b). Installed BEFORE anything loads so it builds through it.
            #[cfg(not(target_arch = "wasm32"))]
            state.set_runner(Box::new(brep_render::runner::ThreadRunner::new()));
            // wasm: the browser-thread analogue — a dedicated web worker (M3b) so the
            // single-threaded wasm UI stays responsive during a run. Same seam. Tests
            // (which never hit this wasm path) keep the default synchronous InlineRunner.
            #[cfg(target_arch = "wasm32")]
            state.set_runner(Box::new(crate::worker::WorkerRunner::new()));
            if !plugin_packages.borrow().is_empty() {
                if let Err(error) = crate::plugins::restore(&mut state, &plugin_packages.borrow()) {
                    log::error!("Plugin restore failed: {error}");
                }
            }
            state.set_viewcube_enabled(true);
            // Partial-override apply: unknown/absent keys keep their defaults.
            if let Some(saved) = &saved_settings {
                let _ = state.apply_settings_json(saved);
            }
            state
        });

        // Start with the seed model on every launch (7626103c2). Reopening the
        // previous session can immediately rerun a problematic document and
        // prevent the user from recovering by restarting the app, so which
        // documents were open is not persisted at all: saved models come back
        // through the file dialog, and work that was never saved comes back as
        // the crash-recovery OFFER below — a prompt, never an automatic load.
        let mut docs = Documents::new(engine_factory);
        let _ = docs.engine_mut().set_history_json(if seed { seed_history_json() } else { crate::document::EMPTY_DOCUMENT.to_string() }.as_str());
        docs.engine_mut().zoom_to_fit();
        docs.active_mut().mark_clean();

        // The autosave blob from a session that ended with unsaved work: offer
        // it back (a prompt, never an automatic restore — see `crate::recovery`).
        let mut recovery = crate::recovery::RecoveryPanel::new();
        recovery.arm_from_store(model_store.as_ref(), docs.engine().settings.disable_recovery_prompt);

        // The settings panel seeds its working JSON from the (post-load) engine
        // settings so the widgets reflect the persisted state on first paint.
        let settings = SettingsPanel::new();
        let part_properties = PartPropertiesPanel::new();

        // New / Open / Save / Save As. Holds no document identity — that lives
        // on each `Document`.
        let file = FileDialog::new();

        // Boot at the saved UI scale.
        let applied_ui_scale = docs.engine().settings.ui_scale;
        let active_document = docs.active_id();

        // The dock layout (loads the persisted tree, or the default). Built before
        // `model_store` is moved into `Self`.
        let dock = DockState::new(model_store.as_ref());

        // Boot-load: if the page URL carries `?loadModel=<url>` (wasm only), start
        // fetching that model NOW; the seed still loads this frame and the fetched
        // model REPLACES it when it lands (drained in `ui`). See the drain block.
        crate::offsite::prefetch();
        #[cfg(target_arch = "wasm32")]
        let pending_boot_load = web_sys::window()
            .and_then(|w| w.location().search().ok())
            .and_then(|search| web_sys::UrlSearchParams::new_with_str(&search).ok())
            .and_then(|params| params.get("loadModel"))
            .filter(|url| !url.is_empty())
            .map(|url| crate::offsite::fetch_text(&cc.egui_ctx, "Opening a model from a link", url));
        #[cfg(not(target_arch = "wasm32"))]
        let pending_boot_load: Option<std::sync::mpsc::Receiver<Result<String, String>>> = None;
        #[cfg(target_arch = "wasm32")]
        let plm_launch = crate::panels::plm_launch::Launch::from_page();
        #[cfg(not(target_arch = "wasm32"))]
        let plm_launch = None;

        Ok(Self {
            docs,
            viewport,
            toolbar: ToolbarPanel::new(),
            workbench_toolbar: WorkbenchToolbarPanel::new(),
            model_store,
            file,
            stl_import: None,
            bug_report: BugReportPanel::new(),
            info: crate::panels::info::InfoPanel::new(),
            diagnostics,
            settings,
            plugins,
            javascript,
            part_properties,
            history: HistoryPanel::new(),
            scene: ScenePanel::new(),
            bom: BomPanel::new(),
            assembly_constraints: AssemblyConstraintsPanel::new(),
            wire_harness: WireHarnessPanel::new(),
            pmi: crate::panels::pmi::PmiPanel::new(),
            sheets: crate::panels::sheets::SheetsPanel::new(),
            update_components: UpdateComponents::new(),
            expressions: ExpressionsPanel::new(),
            qualify: crate::panels::qualify::QualifyPanel::new(),
            family_table: crate::panels::family_table_editor::FamilyTableEditor::new(),
            plm: crate::panels::plm_host::PlmHost::new(),
            info_windows: InfoWindows::new(),
            interference: crate::panels::interference::InterferenceWindow::new(),
            auto_constraints: crate::panels::auto_constraints::AutoConstraintsWindow::new(),
            step_parts: crate::panels::step_parts::StepPartsPanel::new(),
            selection: SelectionPanel::new(),
            context_bar: ContextBarPanel::new(),
            sketch: SketchPanel::new(),
            mode_bar: ModeBar::new(),
            toasts: Toasts::new(),
            busy: crate::panels::busy::BusyIndicator::new(),
            dock,
            first_run_framed: false,
            pending_boot_load,
            plm_launch,
            active_document,
            sheets_pane_shown: false,
            document_tab_hits: Vec::new(),
            params_blob: (u64::MAX, usize::MAX, u64::MAX, String::new()),
            #[cfg(feature = "automation")]
            spin: None,
            autosave: crate::recovery::Autosave::new(),
            recovery,
            applied_ui_scale,
            #[cfg(feature = "automation")]
            automation: {
                let q = crate::automation::queue::AutomationQueue::new();
                q.attach(&cc.egui_ctx);
                q
            },
        })
    }

    /// Arm a measured camera spin: `frames` pointer steps of `step` logical px
    /// each, starting at the centre of the 3D viewport. Replaces any spin in
    /// flight. See [`Spin`].
    #[cfg(feature = "automation")]
    pub fn start_spin(&mut self, frames: u32, step: (f64, f64)) {
        let rect = self.viewport.last_rect();
        let origin = rect
            .map(|r| (r.width() as f64 * 0.5, r.height() as f64 * 0.5))
            .unwrap_or((320.0, 240.0));
        self.spin = Some(Spin { left: frames.max(1), origin, step, made: 0, pressed: false });
    }

    /// Is a measured spin still running?
    #[cfg(feature = "automation")]
    pub fn spinning(&self) -> bool {
        self.spin.is_some()
    }

    /// Advance an in-flight spin by ONE pointer step and keep the frame loop
    /// awake. The drag sweeps out and back over a fixed span so it never leaves
    /// the viewport however many frames are asked for; the camera keeps turning
    /// either way, which is all the measurement needs.
    #[cfg(feature = "automation")]
    fn step_spin(&mut self, ctx: &egui::Context) {
        let Some(mut spin) = self.spin.take() else { return };
        {
            let state = self.docs.engine_mut();
            if !spin.pressed {
                state.pointer_down(spin.origin.0, spin.origin.1, brep_render::controls::BUTTON_LEFT);
                spin.pressed = true;
            }
            const SPAN: f64 = 60.0;
            let phase = f64::from(spin.made % (2 * SPAN as u32));
            let t = if phase <= SPAN { phase } else { 2.0 * SPAN - phase };
            state.pointer_move(spin.origin.0 + spin.step.0 * t, spin.origin.1 + spin.step.1 * t);
            spin.made += 1;
            spin.left -= 1;
            if spin.left == 0 {
                state.pointer_up();
            }
        }
        if spin.left > 0 {
            self.spin = Some(spin);
        }
        ctx.request_repaint();
    }

    /// The `__brepParams` JSON, rebuilt only when the document it describes can
    /// have changed. The `params_blob` FIELD carries the reason this is not
    /// simply built afresh each frame.
    fn params_blob(&mut self) -> &str {
        let key = (
            self.docs.active_id(),
            self.docs.engine().history_rollback(),
            self.docs.engine().history_revision(),
        );
        if (self.params_blob.0, self.params_blob.1, self.params_blob.2) != key {
            self.params_blob = (
                key.0,
                key.1,
                key.2,
                self.docs.engine().feature_params_json(key.1),
            );
        }
        &self.params_blob.3
    }

    /// Global keyboard shortcuts (egui input): **Ctrl/Cmd+Z** undo,
    /// **Ctrl/Cmd+Shift+Z** or **Ctrl/Cmd+Y** redo, **Esc** clears the selection
    /// — unless a popup or menu is up, which the Escape closes instead.
    ///
    /// `Modifiers::COMMAND` is Ctrl on Windows/Linux and ⌘ on macOS, so one map
    /// covers both. Skipped entirely while an egui TEXT edit is focused so typing
    /// (and text-field Ctrl+Z / Esc-to-defocus) is never hijacked. Redo is
    /// consumed BEFORE undo because egui's `consume_key` matches modifiers
    /// logically (a plain `COMMAND+Z` pattern would also swallow `COMMAND+Shift+Z`).
    fn handle_shortcuts(&mut self, ctx: &egui::Context) {
        // A modal owns every key: its Escape closes it, and nothing under it —
        // undo, a Delete, the selection — is the key's to change.
        if ctx.text_edit_focused() || crate::viewport::modal_open_last_pass(ctx) {
            return;
        }
        use egui::{Key, Modifiers};
        // A popup or menu that was up last frame owns this Escape: it closes on
        // `key_pressed(Escape)`, which a consumed key never is, so the key is
        // left in the input for it and nothing below acts on it — not the
        // sketch tool, not a pick, and not the selection its menu is about.
        let popup_open = crate::viewport::popup_open_last_pass(ctx);
        // On an eCAD workbench, Delete and Escape are the editor's keys, and
        // whether they reach it is the editors' ONE rule, `keys_reach_editor`
        // (a text field, a menu or combo box, a modal), not a copy of it here.
        // The watch above is the host's own addition: egui's popup memory does
        // not see the host's `open_bool` menus (a tree row's), and those own
        // their Escape on every workbench.
        let ecad_target = crate::workbench::ecad::Target::of_workbench(&self.docs.engine().settings.workbench);
        let keys_free = match ecad_target {
            Some(_) => !popup_open && brep_ecad_egui::keys_reach_editor(ctx),
            None => !popup_open,
        };
        if crate::javascript::JavaScriptEditor::editing_source(ctx) {
            return;
        }
        let (redo, undo, esc) = ctx.input_mut(|i| {
            let redo = i.consume_key(Modifiers::COMMAND | Modifiers::SHIFT, Key::Z)
                || i.consume_key(Modifiers::COMMAND, Key::Y);
            let undo = i.consume_key(Modifiers::COMMAND, Key::Z);
            let esc = keys_free && i.consume_key(Modifiers::NONE, Key::Escape);
            (redo, undo, esc)
        });
        // While editing a sketch, Ctrl+Z / Ctrl+Shift+Z drive the PER-SESSION sketch
        // history (S6a), not the model-level undo — this global router consumes the
        // keys first (before the viewport), so it must intercept here. Esc drops the
        // active draw/trim/pick tool back to Select/drag (clearing any in-progress
        // placement): this is the ONLY reliable capture point, since `consume_key`
        // above already swallowed the Escape before the viewport can see it.
        if self.docs.engine().sketch_mode() {
            if redo {
                self.docs.engine_mut().sketch_redo();
            }
            if undo {
                self.docs.engine_mut().sketch_undo();
            }
            if esc {
                self.docs.engine_mut().sketch_set_tool(Some("select"));
            }
            return;
        }
        if redo {
            self.docs.engine_mut().redo();
        }
        if undo {
            self.docs.engine_mut().undo();
        }
        if let Some(target) = ecad_target {
            if keys_free && !self.docs.engine().ref_select_active() {
                let delete = ctx.input_mut(|i| i.consume_key(Modifiers::NONE, Key::Delete)
                    || i.consume_key(Modifiers::NONE, Key::Backspace));
                if delete {
                    self.docs.active_mut().ecad.selection_key(target, true);
                }
            }
        } else {
            self.delete_selected_annotation(ctx);
        }
        if esc {
            // An open pick-list popup owns the first Escape: close it WITHOUT
            // clearing the selection (a popup-built multi-selection must survive
            // dismissing the list); the next Escape clears as before.
            if self.viewport.close_candidate_popup() {
                return;
            }
            // A REFERENCE PICK owns the next one — a 3D pick or anchors on a
            // drawing sheet alike — and Escape has to CANCEL it
            // rather than merely clear the selection. Beginning a pick ROLLS THE
            // MODEL BACK to the step before the edited feature so the picker sees
            // the geometry that feature is about to consume; Finish and Cancel
            // both restore that roll through `end_ref_select`. Escape used to do
            // neither: it cleared the selection, left the picker open and left the
            // history PARKED before the feature, so the feature stopped running
            // and the viewport quietly showed the model without it. Measured on
            // the retired `rotate-face.json`, which read the seed volume back at its
            // closed-form step with the feature never re-run.
            if self.docs.engine().ref_select_active() {
                self.docs.engine_mut().cancel_ref_select();
                return;
            }
            // A family-row preview owns the next one: Escape leaves the
            // preview (the family's own values again) and keeps the selection.
            if self.family_table.escape_preview(self.docs.engine_mut()) {
                return;
            }
            // The global router owns Escape before the viewport runs its keys.
            if let Some(target) = ecad_target {
                self.docs.active_mut().ecad.selection_key(target, false);
                return;
            }
            self.docs.engine_mut().clear_selection();
        }
    }

    /// DELETE (or Backspace) removes the selected object of the OPEN drawing
    /// sheet — a placed view, a sheet dimension or an ordinate set, picked on
    /// the paper or in the Sheets tree — or, with no sheet open, the selected
    /// PMI annotation while the PMI workbench is up. Either goes through the
    /// destructive entry of the object's own row menu, so the key and the menu
    /// do the same thing (and it is one undo step, like the menu's).
    ///
    /// The key is consumed only when there IS something to delete. Never a
    /// SHEET or a PMI VIEW: each takes everything placed on it with it, and
    /// both stay behind their menus' Delete. Nothing while a reference pick is
    /// up — deleting the object whose references are being picked would
    /// strand the picker. (Called after the text-edit and sketch-mode returns,
    /// so a Backspace typed into a field, or a sketch's own Delete, never gets
    /// here.)
    fn delete_selected_annotation(&mut self, ctx: &egui::Context) {
        use egui::{Key, Modifiers};
        let engine = self.docs.engine();
        if engine.ref_select_active() {
            return;
        }
        let target = if engine.sheet_open().is_some() {
            engine.sheet_selected_object().map(|id| (true, id.to_string()))
        } else if engine.pmi_workbench_entered() {
            engine.pmi_selected_annotation().map(|id| (false, id.to_string()))
        } else {
            None
        };
        let Some((on_sheet, id)) = target else {
            return;
        };
        let pressed = ctx.input_mut(|i| {
            i.consume_key(Modifiers::NONE, Key::Delete) || i.consume_key(Modifiers::NONE, Key::Backspace)
        });
        if !pressed {
            return;
        }
        let engine = self.docs.engine_mut();
        let result = if on_sheet {
            crate::panels::sheets::delete_held_object(engine, &id)
        } else {
            crate::panels::pmi::delete_annotation(engine, &id)
        };
        if let Err(error) = result {
            engine.push_notice(format!("{}: {error}", if on_sheet { "Sheets" } else { "PMI" }));
        }
    }

    /// Open the component source, offering to recover a missing file from the
    /// assembly's embedded part definition.
    fn edit_part(&mut self, component_id: &str) {
        self.file.edit_component_part(
            &mut self.docs,
            self.model_store.as_ref(),
            component_id,
        );
    }

    /// Draw an isolated preview while the destination document remains untouched.
    /// Hand the working indicator this frame's work, most important first:
    /// the active document's runner queues, a boot load, the STEP parts
    /// library's downloads, then every background document's queues.
    fn collect_busy(&mut self, ctx: &egui::Context) {
        use crate::panels::busy::{engine_activities, Activity};
        let active = self.docs.active_index();
        let mut activities = engine_activities(self.docs.engine(), self.docs.active_id(), None);
        if self.pending_boot_load.is_some() {
            activities.push(Activity::new("boot", "boot", "Loading document"));
        }
        activities.extend(self.step_parts.busy_activities());
        activities.extend(self.file.busy_activities());
        // A write-behind store (the browser's IndexedDB) still settling saves.
        let saving = self.model_store.pending_writes();
        if saving > 0 {
            let label = match saving {
                1 => "Saving to browser storage".to_string(),
                n => format!("Saving to browser storage ({n} writes)"),
            };
            activities.push(Activity::new("storeWrite", "storeWrite", label));
        }
        for (index, doc) in self.docs.iter().enumerate() {
            if index != active {
                activities.extend(engine_activities(&doc.engine, doc.id(), Some(&doc.title())));
            }
        }
        self.busy.update(activities, ctx.input(|i| i.time));
        self.busy.request_repaint(ctx);
    }

    fn show_stl_preview(&mut self, ui: &mut egui::Ui) -> bool {
        use crate::panels::stl_import::PreviewAction;
        let Some(preview) = self.stl_import.as_mut() else {
            return false;
        };
        let action = if preview.destination != self.docs.active_id() {
            PreviewAction::Cancel
        } else {
            preview.show(ui, &mut self.viewport)
        };
        if crate::automation::registry::enabled() {
            let _publish_span = crate::perf::span(crate::perf::Phase::Publish);
            crate::automation::registry::publish("__brepImportPreview", "STL/OBJ import preview state (tolerances, counts, accept readiness); null when no preview is open", &preview.state_json());
            crate::automation::registry::publish("__brepImportPreviewHit", "import preview dialog widget rects", &preview.hits_json());
            crate::automation::registry::publish("__brepCamera", "camera state: kind, eye, target, up, near/far, projection block, worldPerPixel", &preview.engine.camera_state_json());
            crate::automation::registry::publish("__brepPpp", "pixels per point of the surface", &format!("{}", ui.ctx().pixels_per_point()));
            crate::automation::registry::publish("__brepHistory", "history listing {step, features:[{index,type,id}]}", &self.docs.engine().history_listing_json());
            crate::automation::registry::publish("__brepDocuments", "open document tabs {active, tabs:[{title,name,dirty}]}", &document_tabs::state_json(&self.docs));
        }
        let close = match action {
            PreviewAction::Accept => {
                match preview.accept_into(self.docs.active_id(), self.docs.engine_mut()) {
                    Ok(()) => true,
                    Err(error) => {
                        self.docs.engine_mut().push_notice(error);
                        false
                    }
                }
            }
            PreviewAction::Cancel => true,
            PreviewAction::None => false,
        };
        if close {
            self.stl_import = None;
            self.viewport.forget_document();
            ui.ctx().request_repaint();
        }
        true
    }

    /// Phase 3 of the automation frame (§4.4): read-only commands answer from
    /// THIS frame's registry and layout. A method rather than a block at the
    /// end of `ui` because the import preview returns from that function early
    /// and still owes the frame its reads.
    #[cfg(feature = "automation")]
    fn drain_reads(&mut self, ctx: &egui::Context) {
        let queue = self.automation.clone();
        let frame = ctx.cumulative_frame_nr();
        queue.drain_app(
            crate::automation::command::Phase::Read,
            &mut crate::automation::command::Ctx { app: self, egui: ctx },
            frame,
        );
    }

    /// Reset everything the shared panels and the viewport hold ABOUT ONE
    /// DOCUMENT, run at the top of the first frame that sees a different active
    /// document.
    ///
    /// Panel state is deliberately NOT per-document (one History panel, one
    /// Scene tree, …): a second copy per tab would double every panel's state
    /// for a benefit — remembering which feature form was open in a background
    /// tab — nobody asked for. The price is that the transient state has to be
    /// dropped on a switch, because every bit of it (expansion sets, hit maps,
    /// an open feature form, a pinned Info window's object name) refers to the
    /// document that just went away.
    fn reset_document_scoped_state(&mut self) {
        self.history = HistoryPanel::new();
        self.scene = ScenePanel::new();
        self.bom = BomPanel::new();
        self.assembly_constraints = AssemblyConstraintsPanel::new();
        self.wire_harness = WireHarnessPanel::new();
        self.pmi = crate::panels::pmi::PmiPanel::new();
        self.sheets = crate::panels::sheets::SheetsPanel::new();
        self.expressions = ExpressionsPanel::new();
        self.qualify = crate::panels::qualify::QualifyPanel::new();
        self.family_table = crate::panels::family_table_editor::FamilyTableEditor::new();
        // A family-row preview left on in a document the user comes back to
        // would have no pane state to show or end it: switching ends it.
        self.docs.engine_mut().set_expression_preview(None);
        // Pinned to object NAMES of the old document ("Box" exists in most of
        // them), so these would silently retarget rather than go blank.
        self.info_windows = InfoWindows::new();
        self.interference = crate::panels::interference::InterferenceWindow::new();
        self.auto_constraints = crate::panels::auto_constraints::AutoConstraintsWindow::new();
        // The outdated-parts cache keys on `(applied_generation, save_generation)`,
        // and two documents' generations are unrelated — a switch can land on the
        // same key with an entirely different parts library.
        self.update_components.invalidate();
        self.viewport.forget_document();
    }

    /// A signature of the CURRENT rendered model (rolled-to step) — solid count,
    /// per-solid triangle count + bbox, and total triangles. Published to JS so
    /// the headed verifier can prove each roll / edit produced different geometry
    /// (names alone don't: a SUBTRACT reuses the target's name).
    #[cfg_attr(not(target_arch = "wasm32"), allow(dead_code))]
    fn model_signature_json(&self) -> String {
        let solids: Vec<serde_json::Value> = self
            .docs
            .engine()
            .scene
            .solids()
            .iter()
            .map(|s| {
                serde_json::json!({
                    "name": s.name,
                    "tris": s.mesh.indices.len() / 3,
                    "min": s.bbox.min,
                    "max": s.bbox.max,
                })
            })
            .collect();
        let total_tris: usize = self
            .docs
            .engine()
            .scene
            .solids()
            .iter()
            .map(|s| s.mesh.indices.len() / 3)
            .sum();
        serde_json::json!({
            "step": self.docs.engine().history_rollback(),
            "solidCount": solids.len(),
            "totalTris": total_tris,
            "solids": solids,
        })
        .to_string()
    }

    /// Open or close the floating Settings window — the toolbar gear's flag,
    /// reachable from the automation layer (`settings_window`).
    pub fn set_settings_window_open(&mut self, open: bool) {
        self.settings.open = open;
    }

    /// Open or close the floating Part Properties window — the toolbar tag
    /// button's flag, reachable from the automation layer
    /// (`part_properties_window`).
    pub fn set_part_properties_window_open(&mut self, open: bool) {
        self.part_properties.open = open;
    }

    /// Open or close the floating Info window — the toolbar info button's flag,
    /// reachable from the automation layer (`info_window`).
    pub fn set_info_window_open(&mut self, open: bool) {
        self.info.open = open;
    }

    /// What this session is running on. The app's ONE record of it: the Info
    /// window draws it, a problem report carries it, the `diagnostics` command
    /// returns it and the MCP banner names its adapter — all from here.
    pub fn diagnostics(&self) -> &crate::diagnostics::Diagnostics {
        &self.diagnostics
    }

    /// Begin the in-app problem report: the same call the Submit Bug button
    /// makes, and for the same reason it makes it THIS frame — `request`
    /// captures the current frame before its own dialog exists.
    pub fn begin_bug_report(&mut self, ctx: &egui::Context) {
        self.bug_report.request(ctx, self.docs.engine(), &self.diagnostics);
    }

    /// Activate an available declarative panel using its stable package ID.
    pub(crate) fn show_plugin_panel(&mut self, id: &str) -> Result<(), String> {
        self.dock.show_plugin_panel(self.docs.engine(), id)
    }

    /// Bring a built-in dock pane to the front (`show_pane`).
    pub fn show_pane(&mut self, kind: PaneKind) {
        self.dock.show_pane(kind);
    }

    /// The parts-library staleness check and its refresh — the Constraints
    /// header's "Update components (N)" button, reachable as the
    /// `component_update` command. Returns `(outdated, missing, refreshed)`;
    /// `run` is skipped when nothing is outdated.
    pub fn update_components(&mut self, run: bool) -> Result<(usize, Vec<String>, usize), String> {
        let store = self.model_store.as_ref();
        self.update_components.ensure_current(self.docs.engine_mut(), store, self.file.save_generation());
        let outdated = self.update_components.outdated_count();
        let missing = self.update_components.missing().to_vec();
        let refreshed = if run { self.update_components.run(self.docs.engine_mut(), store)? } else { 0 };
        Ok((outdated, missing, refreshed))
    }

    /// Frame what the central tile is actually DRAWING: the OPEN SHEET's paper
    /// when a sheet is open, the 3D scene otherwise.
    ///
    /// This is where the fit branch lives, and it is here rather than in the
    /// toolbar because the two halves sit in different places — the 3D fit is an
    /// engine camera op (`EngineState::zoom_to_fit`) while the sheet's pan/zoom
    /// is viewport-transient state, refitted by clearing the marker the sheet
    /// paint compares against. One method, so the toolbar's Zoom-to-fit button
    /// and the `zoom_to_fit` command can never mean different things — and so
    /// the paper needs no Fit button of its own.
    ///
    /// The toolbar draws BEFORE the dock in `ui`, so a fit requested from the
    /// button lands on this very frame.
    pub fn zoom_to_fit(&mut self) {
        if self.docs.engine().sheet_open().is_some() {
            self.viewport.request_sheet_fit();
        } else {
            self.docs.engine_mut().zoom_to_fit();
        }
    }

    /// Run what a WORKBENCH TOOLBAR button does, by its `WorkbenchButton::id`.
    ///
    /// Split out of `ui` so a toolbar click and the `workbench_button` command
    /// run the SAME arms: the workbench registry declares a button, this
    /// dispatches it, and the automation surface owns no second copy of the
    /// list. Returns whether the id was known.
    pub fn dispatch_workbench_button(&mut self, id: &str) -> bool {
        // The eCAD workbenches' buttons (Diagram, PCB, Symbol, Pads) are
        // eCAD's own actions: run on the active document's editors with
        // eCAD's `run_action`, so a press does exactly what eCAD's toolbar
        // button does. `None` is any other id, which falls through to the
        // arms below.
        if crate::workbench::ecad::dispatch(&mut self.docs.active_mut().ecad, id).is_some() {
            return true;
        }
        // The SHEET's constructions are the Drawing workbench's two tables plus
        // the section and the detail, and each press is the PMI annotation
        // buttons' workflow: the object is created on the open sheet with
        // nothing picked yet, and its dialog opens (through the dialog door,
        // which surfaces the Sheets pane) — where its reference rows are picked
        // on the paper with the reference picker.
        if let Some((kind, alignment)) = crate::workbench::drawing::dim_tool(id) {
            let engine = self.docs.engine_mut();
            if let Err(error) = engine.sheet_add_dimension(None, kind, Some(alignment), Vec::new()) {
                engine.push_notice(format!("Sheet dimension: {error}"));
            }
            return true;
        }
        if let Some(axis) = crate::workbench::drawing::ord_tool(id) {
            let engine = self.docs.engine_mut();
            if let Err(error) = engine.sheet_add_ordinate(None, axis, None, Vec::new()) {
                engine.push_notice(format!("Ordinate set: {error}"));
            }
            return true;
        }
        if id == crate::workbench::drawing::BOM_BUTTON_ID {
            let engine = self.docs.engine_mut();
            if let Err(error) = engine.sheet_insert_bom() { engine.push_notice(format!("BOM table: {error}")); }
            return true;
        }
        if id == crate::workbench::drawing::SECTION_BUTTON_ID {
            let engine = self.docs.engine_mut();
            if let Err(error) = engine.sheet_new_section(None) {
                engine.push_notice(format!("Section view: {error}"));
            }
            return true;
        }
        if id == crate::workbench::drawing::DETAIL_BUTTON_ID {
            let engine = self.docs.engine_mut();
            if let Err(error) = engine.sheet_new_detail(None) {
                engine.push_notice(format!("Detail view: {error}"));
            }
            return true;
        }
        // The SKETCH mode's draw tools are one table too, and they are SHARED —
        // every workbench's row carries them while a sketch is being edited, so
        // this arm answers for all of them whatever workbench is active.
        // Pressing the LIT tool puts it down (back to Select), matching the
        // sheet constructions above; pressing the lit Select is already Select.
        if let Some(tool) = crate::workbench::sketch::draw_tool(id) {
            let engine = self.docs.engine_mut();
            let armed = engine.sketch_active_tool().unwrap_or("select") == tool;
            engine.sketch_set_tool(Some(if armed { "select" } else { tool }));
            return true;
        }
        match id {
            // The sketch strip's one-shot action, now the row's: infer the
            // constraints the rough-in geometry already implies.
            crate::workbench::sketch::AUTOCONSTRAIN_BUTTON_ID => {
                self.docs.engine_mut().sketch_auto_constrain();
            }
            // Sheet Metal's flat pattern: open the export modal in its DXF /
            // SVG mode; the engine reports "no sheet-metal body in the part"
            // as a toast on export.
            "sheetmetal.flat_pattern" => {
                self.file.dispatch(FileAction::ExportFlatPattern, &mut self.docs, self.model_store.as_ref());
            }
            // Assembly's Add Component: open the insert-component modal (the
            // same flow as the ACOMP palette pick). ONE arm for both rows that
            // carry it — PCB BORROWS this very button (`workbench::BORROWED`)
            // rather than declaring a second id for the same action.
            crate::workbench::assembly::ADD_COMPONENT_BUTTON_ID => {
                self.file.dispatch(FileAction::InsertComponent, &mut self.docs, self.model_store.as_ref());
            }
            // Assembly's interference check: run the engine's pairwise
            // intersect sweep NOW and open the results window.
            "assembly.interference" => {
                self.interference.open_and_run(self.docs.engine_mut());
            }
            // Assembly's Auto Constraints: open the inference window and scan
            // the current placement NOW, so it opens showing real counts.
            "assembly.auto_constrain" => {
                self.auto_constraints.open_and_scan(self.docs.engine_mut());
            }
            // Assembly's step.parts library: open the online-library browser
            // (search → thumbnails → import a STEP part → add as an ACOMP).
            "assembly.step_parts_library" => {
                self.step_parts.open();
            }
            // Wire harness's Declare connection point: the part's FIRST port
            // group, with one point at the origin. It exists because the
            // Qualify pane is on screen only while the part declares some — so
            // without it a 3D-only part could never get its first one from the
            // UI at all. The pane is surfaced by the dock's own ports door on
            // the next frame, once the visibility pass has let it back in.
            crate::workbench::wire_harness::DECLARE_POINT_BUTTON_ID => {
                let engine = self.docs.engine_mut();
                engine.history.set_ports_block(
                    Some(serde_json::json!([{
                        "name": crate::workbench::wire_harness::FIRST_GROUP,
                        "purpose": crate::workbench::wire_harness::FIRST_PURPOSE,
                        "points": [{
                            "name": crate::workbench::wire_harness::FIRST_POINT,
                            "transform": {
                                "position": [0.0, 0.0, 0.0],
                                "rotationEuler": [0.0, 0.0, 0.0],
                            },
                        }],
                    }])),
                    None,
                );
            }
            // PMI's Capture view: snapshot the camera + visibility into a new
            // active view and surface the PMI pane so its row is seen.
            crate::workbench::pmi::CAPTURE_BUTTON_ID => {
                self.docs.engine_mut().pmi_capture_view(None);
                self.dock.show_pane(PaneKind::Pmi);
            }
            // Drawing's Back to 3D: leave the open sheet. The paper's own
            // widgets go with it, and the sheet keeps its contents — the same
            // edit the Sheets pane's Close sheet makes.
            crate::workbench::drawing::SHEET_CLOSE_BUTTON_ID => {
                let _ = self.docs.engine_mut().sheet_set_open(None);
                self.viewport.forget_sheet();
            }
            // Drawing's Add sheet: a new sheet, opened on the paper, and the
            // Sheets pane brought forward so its row is seen.
            crate::workbench::drawing::ADD_SHEET_BUTTON_ID => {
                self.docs.engine_mut().sheet_add(None);
                self.dock.show_pane(PaneKind::Sheets);
            }
            // Drawing's Place view: the first saved PMI view on the open sheet;
            // the placement's form opens (the door surfaces the pane), and its
            // `view` field is where a different view is chosen.
            crate::workbench::drawing::PLACE_VIEW_BUTTON_ID => {
                let engine = self.docs.engine_mut();
                let placed = match engine.sheet_pmi_view_ids().first().cloned() {
                    Some(view) => engine.sheet_place_view(None, &view, None, None).map(|_| ()),
                    None => Err("capture a PMI view in the PMI workbench first".to_string()),
                };
                if let Err(error) = placed {
                    engine.push_notice(format!("Place view: {error}"));
                }
            }
            _ => return false,
        }
        true
    }

}

impl eframe::App for BrepApp {
    /// Phase 1 of the automation frame (§4.4): queued input commands become
    /// egui events; a screenshot that landed completes its reply. Called by
    /// every eframe runner before `ui`; the headless host calls it itself.
    #[cfg(feature = "automation")]
    fn raw_input_hook(&mut self, ctx: &egui::Context, raw_input: &mut egui::RawInput) {
        let view = self.viewport.last_rect();
        self.automation.drain_input(raw_input, ctx.cumulative_frame_nr(), ctx.pixels_per_point(), view, ctx);
    }


    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // Open this frame's timing window (banks the previous frame's phase
        // totals and its period) and bracket the whole body. See `perf`: on a
        // single-threaded shell a long frame IS a frozen UI, so what the frame
        // spends is the whole diagnosis.
        crate::perf::tick();
        let _ui_span = crate::perf::span(crate::perf::Phase::Ui);

        // The native store and PLM queue (`plm::native`): every task whose I/O
        // answered since the last frame finishes here, on this thread, FIRST —
        // before any early return below (the mesh preview's replaces the whole
        // shell) can skip it. The browser drains its microtasks between events
        // the same way, whatever the page is showing.
        #[cfg(not(target_arch = "wasm32"))]
        crate::plm::native::run_pending();
        // A PLM save made mid-rebuild is pictured once its run lands.
        crate::plm::thumbnail::tick(|id| self.docs.iter().find(|d| d.id() == id).map(|d| &d.engine));

        // --- global keyboard shortcuts (undo/redo/clear-selection) ------------
        // Handled before any panel draws so a Ctrl+Z etc. this frame takes effect
        // this frame. `ctx` is a cheap Arc clone (avoids borrowing `ui` across the
        // `&mut self` call).
        let ctx = ui.ctx().clone();

        self.plugins.finish_installations(&mut self.docs, self.model_store.as_ref());
        self.plugins.sync_documents(&mut self.docs);
        self.javascript.poll(&ctx, &mut self.docs, self.model_store.as_ref());
        self.plugins.poll(&ctx, self.docs.engine_mut(), self.model_store.as_ref());

        // Phase 2 of the automation frame (§4.4): mutations run before any
        // panel draws, so this frame shows their effect.
        #[cfg(feature = "automation")]
        {
            let queue = self.automation.clone();
            let frame = ctx.cumulative_frame_nr();
            queue.drain_app(
                crate::automation::command::Phase::Mutate,
                &mut crate::automation::command::Ctx { app: self, egui: &ctx },
                frame,
            );
        }

        // A measured spin (`perf_spin`) advances one pointer step per frame, here
        // — after the mutation drain that may have armed it, so the spin starts
        // on the frame it was asked for.
        #[cfg(feature = "automation")]
        self.step_spin(&ctx);

        // --- a different document is active than the panels were drawn for ----
        // Checked FIRST, before anything draws: the switch itself happened late
        // in some earlier frame (a tab click, a close, an Open), and every
        // shared panel is still holding the previous document's transient state.
        if self.active_document != self.docs.active_id() {
            if !self.javascript.running_in(self.active_document) {
                for doc in self.docs.iter_mut() { if doc.id() == self.active_document { doc.engine.cancel_plugin_action(); } }
            }
            self.plugins.close_action();
            self.active_document = self.docs.active_id();
            self.reset_document_scoped_state();
        }

        // --- the workbench decides whether placed parts show their ports -------
        // A workbench that shows the Wire Harness panel (Wire harness, All)
        // draws the ports components carry; the others — Modeling included,
        // though it offers the Port feature — keep the assembly clean. A no-op
        // when nothing changed.
        {
            // The document's name lives in the SHELL (the tab, the file), and a
            // drawing sheet's title block prints it. Told to the engine here,
            // every frame, because that is where the active tab is known; a
            // no-op when it has not changed. It is the DISPLAY name, the one the
            // tab shows: a stamp reading `bracket.nbrep` would be naming our
            // file format rather than the drawing.
            let name = self
                .docs
                .active()
                .name()
                .map(crate::store::model_display_name)
                .unwrap_or_default();
            let engine = self.docs.engine_mut();
            engine.set_document_name(&name);
            let show = crate::workbench::panel_visible(
                &engine.settings.workbench,
                crate::workbench::wire_harness::PANEL_ID,
                &crate::workbench::ButtonState::of(engine),
            );
            engine.set_component_ports_visible(show);
            // The PMI workbench IS the PMI editing mode: a workbench that shows
            // the PMI panel (PMI, All) enters it — the modeling camera /
            // visibility / wireframe are remembered — and one that hides it
            // leaves it, deactivating the view and restoring them.
            let pmi_shown = crate::workbench::panel_visible(
                &engine.settings.workbench,
                crate::workbench::pmi::PANEL_ID,
                &crate::workbench::ButtonState::of(engine),
            );
            if pmi_shown && !engine.pmi_workbench_entered() {
                engine.pmi_enter_workbench();
            } else if !pmi_shown && engine.pmi_workbench_entered() {
                engine.pmi_leave_workbench();
            }
            // The DRAWING workbench owns the paper: LEAVING it for a workbench
            // that hides the Sheets pane closes an open sheet, so the central
            // tile is not left drawing paper whose tools, pane and way back are
            // all hidden. It is a MODE change — the sheets and their contents
            // are untouched — and an edge, see `sheets_pane_shown`.
            let sheets_shown = crate::workbench::panel_visible(
                &engine.settings.workbench,
                crate::workbench::drawing::SHEETS_PANEL_ID,
                &crate::workbench::ButtonState::of(engine),
            );
            if self.sheets_pane_shown && !sheets_shown && engine.sheet_open().is_some() {
                let _ = engine.sheet_set_open(None);
                self.viewport.forget_sheet();
            }
            self.sheets_pane_shown = sheets_shown;
        }

        // --- GUI chrome theme -------------------------------------------------
        // Apply the user's theme preference to the egui chrome every frame
        // (idempotent: `set_theme` just stores the preference). Auto follows the
        // OS/system theme (prefers-color-scheme on web); egui falls back to dark
        // when no OS signal is available. This controls panels/windows/toolbar/
        // text only — the 3D viewport `background` is a separate setting.
        ctx.set_theme(match self.docs.engine().settings.theme {
            ThemeMode::Auto => egui::ThemePreference::System,
            ThemeMode::Light => egui::ThemePreference::Light,
            ThemeMode::Dark => egui::ThemePreference::Dark,
        });

        // --- global UI size scale --------------------------------------------
        // Apply the user's "UI scale" to the whole egui chrome every frame. This
        // is idempotent when unchanged (`set_zoom_factor` only repaints on an
        // actual change) and composes with the native device pixel ratio
        // (pixels_per_point = zoom_factor * native_pixels_per_point).
        //
        // Defer live UI rescale while the user drags the Settings "UI scale" slider:
        // the slider value updates continuously, but only commit it to the actual egui
        // zoom once the pointer is released, so the whole UI doesn't rescale under the
        // cursor mid-drag.
        let pointer_down = ctx.input(|i| i.pointer.any_down());
        if !pointer_down {
            self.applied_ui_scale = self.docs.engine().settings.ui_scale;
        }
        ctx.set_zoom_factor(self.applied_ui_scale);

        // --- history-runner pump ---------------------------------------------
        // Apply any completed background history run BEFORE panels read the scene.
        // For the synchronous InlineRunner this is a no-op (`rerun_history` already
        // pumped its own submit), so nothing changes today; it is the seam a future
        // native-thread / wasm-worker runner lands its reply through. While a run is
        // still in flight, keep the frame loop alive so its reply gets pumped — for
        // Inline `run_pending()` is always false, so this never fires.
        //
        // EVERY open document is pumped, not just the active one — see
        // `Documents::pump_all`, which also keeps the active document's parts
        // library in the kernel's store when a background tab's run lands.
        let work_in_flight = self.docs.pump_all();
        if work_in_flight {
            ctx.request_repaint();
        }
        if self.show_stl_preview(ui) {
            // The preview REPLACES the shell for the frame, and this early
            // return used to take the frame's READ drain with it: while a
            // preview was open every read-only automation command — `ping`,
            // `state_get`, `hit_rects`, a screenshot — went unanswered until
            // the host gave up on it, so no script could observe the preview it
            // had just opened. The preview publishes its own state and rects
            // above; this answers the reads that ask for them.
            #[cfg(feature = "automation")]
            self.drain_reads(&ctx);
            return;
        }
        if crate::automation::registry::enabled() {
            let _publish_span = crate::perf::span(crate::perf::Phase::Publish);
            crate::automation::registry::publish("__brepImportPreview", "STL/OBJ import preview state (tolerances, counts, accept readiness); null when no preview is open", "null");
        }

        // The tab strip's dirty dots, refreshed once per frame (cheap — see
        // `Document::refresh_dirty_marker`).
        self.docs.refresh_dirty_markers();

        // --- boot-load (?loadModel=): apply the fetched model once it lands ----
        // Replaces the seed with the URL-specified document (armed in `new`). The
        // ehttp callback wakes the frame loop, so a plain per-frame drain suffices.
        // `load_model_and_fit` arms deferred framing; the pump above reframes it
        // next frame. `mark_clean` opens it as a non-dirty document.
        if self.pending_boot_load.is_some() {
            let received = self
                .pending_boot_load
                .as_ref()
                .and_then(|rx| rx.try_recv().ok());
            if let Some(result) = received {
                self.pending_boot_load = None;
                match result {
                    Ok(json) => {
                        // REPLACES the seed in place rather than adding a tab.
                        // It lands on the document that is already active,
                        // whatever the session restored.
                        let _ = self.docs.engine_mut().load_model_and_fit(&json);
                        self.docs.active_mut().mark_clean();
                    }
                    Err(e) => self
                        .docs
                        .engine_mut()
                        .push_notice(format!("Could not load model from URL: {e}")),
                }
            }
        }

        // --- "Open in CAD" (?open=part/…/rev/…) --------------------------------
        if let Some(launch) = self.plm_launch.as_mut() {
            use crate::panels::plm_launch::Step;
            let step = launch.step(self.model_store.as_ref(), &mut self.docs, &mut self.file);
            match &step {
                Step::SignIn(sentence) => {
                    self.settings.show_plm_tab();
                    self.docs.engine_mut().push_notice(sentence.clone());
                }
                Step::Refused(sentence) => self.docs.engine_mut().push_notice(sentence.clone()),
                _ => {}
            }
            if step.settles() {
                crate::panels::plm_launch::forget_query();
            }
            if launch.is_done() {
                self.plm_launch = None;
            } else {
                ui.ctx().request_repaint();
            }
        }

        crate::panels::ecad_parts::sync(self.docs.active_mut());
        let requests = crate::panels::ecad_parts::take_open_requests(self.docs.active_mut());
        for key in requests {
            self.file.open_document(&mut self.docs, self.model_store.as_ref(), &key);
        }
        // The parts pane's Add part button presses Assembly's Add Component: one arm.
        if crate::panels::ecad_parts::take_add_part_request(self.docs.active_mut()) {
            self.dispatch_workbench_button(crate::workbench::assembly::ADD_COMPONENT_BUTTON_ID);
        }
        // The eCAD host's Update parts button is Constraints' Update components.
        if std::mem::take(&mut self.docs.active_mut().ecad_update_requested) {
            if let Err(error) = self.update_components.run(self.docs.engine_mut(), self.model_store.as_ref()) {
                self.docs.engine_mut().push_notice(format!("Update components: {error}"));
            }
        }

        // --- update-components badge freshness ---------------------------------
        // Keep the outdated-parts checker current BEFORE any assembly panel draws
        // (the structure tree renders per-node badges ahead of the constraints
        // header). Cheap: a real recompute happens only when an applied run or a
        // successful store save moved the generation key.
        self.update_components.ensure_current(
            self.docs.engine_mut(),
            self.model_store.as_ref(),
            self.file.save_generation(),
        );
        // …and the eCAD host marks the components placed from those parts.
        self.docs.active_mut().ecad_outdated = self.update_components.outdated().to_vec();

        // --- async-safe first-model framing -----------------------------------
        // The seed run is async under a background runner (native thread / wasm
        // worker), so the boot `zoom_to_fit` may have run before any solid existed.
        // Frame the model ONCE, the first frame the seed run has fully landed (solids
        // present AND no run still in flight). Under the synchronous Inline runner
        // (tests) both hold on the very first frame, so this is identical to today.
        if !self.first_run_framed
            && !self.docs.engine().run_pending()
            && self.docs.engine().has_solids()
        {
            self.docs.engine_mut().zoom_to_fit();
            self.first_run_framed = true;
        }

        self.handle_shortcuts(&ctx);

        // --- top toolbar: primary actions, drawn FIRST so its top strip is
        // reserved above the left panel + central viewport. A clicked File button
        // returns an action the file dialog acts on (open its modal / save / new).
        self.toolbar.read_only = !self.docs.active().access().is_editable();
        let doc = self.docs.active_mut();
        let toolbar_outcome = self.toolbar.show(
            ui,
            &mut doc.engine,
            Some(&doc.ecad),
            self.model_store.as_ref(),
            &mut self.settings.open,
            &mut self.part_properties.open,
            &mut self.info.open,
        );
        if toolbar_outcome.plugins { self.plugins.open = !self.plugins.open; }
        if toolbar_outcome.javascript { self.javascript.open = !self.javascript.open; }
        if let Some(id) = toolbar_outcome.plugin_action {
            if let Err(error) = self.plugins.open_action(self.docs.engine(), &id) { self.plugins.error = Some(error); }
        }
        self.plugins.active_document = self.docs.active_id();
        self.plugins.show(&ctx, self.docs.engine_mut(), self.model_store.as_ref());
        self.javascript.show(&ctx, &mut self.docs, self.model_store.as_ref());
        // An inbox row opens its review or change order in the PLM pane.
        for event in toolbar_outcome.plm_review {
            self.plm.open_review(event);
        }
        if let Some(name) = toolbar_outcome.recent_document {
            self.file
                .open_document(&mut self.docs, self.model_store.as_ref(), &name);
        }
        if let Some(action) = toolbar_outcome.file {
            self.file
                .dispatch(action, &mut self.docs, self.model_store.as_ref());
        }
        // Submit Bug: begin the screenshot-capture + report flow. `request`
        // grabs the current frame (before its dialog exists) and the model, so
        // it must run THIS frame while the shot is still dialog-free.
        if toolbar_outcome.bug_report {
            self.bug_report.request(&ctx, self.docs.engine(), &self.diagnostics);
        }
        // A workbench toolbar button click surfaces its id here; the dispatch
        // itself is a METHOD so the `workbench_button` command runs the very
        // same arms (a button an agent can only reach by clicking is a button
        // an agent cannot reach when something covers it).
        if let Some(id) = toolbar_outcome.workbench_button {
            self.dispatch_workbench_button(id);
        }
        // Zoom-to-fit: the ONE button frames whatever the tile draws — the open
        // sheet's paper, or the 3D scene. See `BrepApp::zoom_to_fit`.
        if toolbar_outcome.zoom_to_fit {
            self.zoom_to_fit();
        }

        // --- workbench actions toolbar: a second strip directly UNDER the
        // primary toolbar (egui stacks top panels in call order) listing the
        // active workbench's creatable features + the constraint types. Off via
        // the Settings checkbox, and never in sketch / reference-selection mode
        // (the panel decides — see `WorkbenchToolbarPanel::visible`). A feature
        // click adds exactly what the palette pick would, so ACOMP still routes
        // to the component selector; a constraint click is the context bar's
        // constraint offer, seeded from the current selection.
        // A read-only document (a PLM revision not checked out, or released)
        // draws the strip disabled: every button on it creates something.
        self.workbench_toolbar.read_only = !self.docs.active().access().is_editable();
        let actions = if self.docs.engine().settings.toolbar_style
            == brep_render::style::ToolbarStyle::Classic
        {
            self.workbench_toolbar.show(ui, self.docs.engine())
        } else {
            self.workbench_toolbar
                .set_ribbon_hits(self.toolbar.home_hits());
            toolbar_outcome.creation
        };
        if let Some(type_code) = actions.feature {
            self.history
                .add_feature_of_type(self.docs.engine_mut(), &type_code);
            // No `show_pane` here: `HistoryPanel::open_form` moved the target and
            // the dialog door surfaces History for it (`surface_open_dialogs`
            // below). ACOMP opens the component-selector MODAL instead of a
            // feature dialog, so it moves no target and surfaces no pane.
            if self.history.take_insert_component_request() {
                self.file.dispatch(
                    FileAction::InsertComponent,
                    &mut self.docs,
                    self.model_store.as_ref(),
                );
            }
        }
        if let Some(type_id) = actions.constraint {
            // The strip offers every type regardless of the selection (unlike the
            // context bar's gated offers), so a refusal has to be SAID: a toast,
            // never a silent no-op. A document with no components has no assembly
            // session to add to (the kernel's own message names the session, not
            // the cause) — say what is actually missing.
            let engine = self.docs.engine_mut();
            if !engine.history_has_assembly() {
                engine.push_notice(
                    "Add constraint: the document has no components — insert a component first"
                        .to_string(),
                );
            } else {
                // The add leaves the new constraint OPEN (the kernel mints it
                // that way), so the dialog door surfaces the Constraints pane.
                if let Err(error) =
                    crate::panels::context_bar::add_constraint_from_selection(engine, &type_id)
                {
                    engine.push_notice(format!("Add constraint: {error}"));
                }
            }
        }
        if let Some(type_id) = actions.annotation {
            // The context bar's annotation offer, from the strip: seeded from
            // the selection, and the new annotation's form opens through the
            // dialog door. The strip offers every type whatever is selected, so
            // a refusal — no active view to add to — is said, not swallowed.
            let engine = self.docs.engine_mut();
            if let Err(error) = crate::panels::context_bar::add_pmi_from_selection(engine, &type_id) {
                engine.push_notice(format!("Add annotation: {error}"));
            }
        }

        // --- sketch mode has NO strip of its own: its draw tools are workbench
        // buttons in the row above (`workbench::sketch`), offered while a sketch
        // is being edited, so the 3D viewport is the full-width sketching
        // surface with nothing between it and the one toolbar.

        // --- bottom STATUS BAR: a persistent, full-width strip whose CONTENT is
        // chosen by context each frame. Drawn AFTER the top bars but BEFORE the
        // left panel(s) so it reserves the FULL bottom width and the left column
        // stops above it (egui resolves reserved space by call order). It is a
        // HOST: the branch below picks what to draw. A NEW context is added by
        // extending this branch (e.g. `else if engine.some_mode() { … }`) and
        // routing through the owning panel's `show_status_bar` for DRY styling.
        // Its top is the floor the toasts stack up from (`Toasts::show`).
        //
        // Its RIGHT END is the app-wide working indicator (`panels::busy`),
        // whatever the context: collected here from every source, drawn first
        // in a right-to-left row so the context content gets what is left of
        // the width and never runs under it.
        self.collect_busy(&ctx);
        let status_bar_top = egui::containers::panel::Panel::bottom("brep-status-bar")
            .resizable(false)
            .min_size(30.0)
            .show(ui, |ui| {
                ui.add_space(2.0);
                // `Align::Min`, not `Center`: the bar's height is not known
                // until its content is laid out, and centring against the
                // unbounded space below doubled the bar with the indicator on
                // a row of its own.
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Min), |ui| {
                    self.busy.show(ui);
                    ui.with_layout(egui::Layout::top_down(egui::Align::Min), |ui| {
                        if self.docs.engine().sketch_mode() {
                            // Sketch context: the status row (title / DOF / N selected /
                            // undo-redo / Lock). The selection-filter row is NOT drawn
                            // now, so drop its stale hit-rects (the verifier must never
                            // click a phantom rect for an off-screen widget).
                            self.selection.clear_hits();
                            self.sketch.show_status_bar(ui, self.docs.engine_mut());
                        } else if crate::workbench::ecad::Target::of_workbench(&self.docs.engine().settings.workbench).is_some() {
                            // An eCAD editor owns the central tile, and a click there
                            // picks nothing 3D: the pickable-kinds filter would be a
                            // row of dead checkboxes. The strip carries the editor's
                            // own line instead: where the pointer is and what is under it.
                            self.selection.clear_hits();
                            self.viewport.show_ecad_status(ui);
                        } else {
                            // Modeling context: the selection filter (pickable kinds).
                            self.selection.show_status_bar(ui, self.docs.engine_mut());
                        }
                    });
                });
                ui.add_space(2.0);
            })
            .response
            .rect
            .top();
        // Cancel on the working indicator: sent to the document whose run it
        // named, which a background tab's run need not be the active one.
        if let Some(activity) = self.busy.take_cancel() {
            if let Some(doc) = self.docs.iter_mut().find(|doc| Some(doc.id()) == activity.document) {
                doc.engine.cancel_run();
            }
        }

        // --- central region: the dock tree, OR (special modes) the bare 3D view
        // ---------------------------------------------------------------------
        // Normal modeling mode: ONE egui_tiles tree fills the whole remaining
        // area between the top toolbar and the bottom status bar. Every side-panel
        // section AND the 3D viewport are tiles the user can split / tab / resize /
        // drag-rearrange, and the layout persists. Which side panes are visible is
        // filtered per-workbench inside the dock (`workbench::panel_visible`).
        //
        // Sketch mode and reference-selection are "special modes" that take over
        // the shell: they BYPASS the tree and draw the viewport directly, so the
        // modeling side panes don't appear (sketch's own entity-list panel + the
        // top-right mode card own those flows). Drawing the viewport HERE — before
        // the top-right overlay below — keeps `viewport.last_rect()` current-frame
        // so the overlay anchors to the live 3D-view rect with no lag.
        let sketch = self.docs.engine().sketch_mode();
        let ref_select = self.docs.engine().ref_select_active();

        if sketch {
            // Sketch entity lists (Curves / Points / Constraints) + solver
            // settings — a dedicated left panel, drawn BEFORE the viewport so it
            // reserves the left and the viewport fills the rest.
            egui::containers::panel::Panel::left("sketch-entities")
                .resizable(true)
                .default_size(300.0)
                .size_range(200.0..=560.0)
                .show(ui, |ui| {
                    self.sketch.show_entity_lists(ui, self.docs.engine_mut());
                });
        }

        self.history.sync_palette_display(self.model_store.as_ref());
        // The PLM panels (a PLM session only): each PLM document's access from
        // its revision, and the revisions the user asked to open.
        let plm = self.plm.sync(self.model_store.as_ref(), &mut self.docs, self.file.save_generation());
        for name in plm.open {
            self.file.open_document(&mut self.docs, self.model_store.as_ref(), &name);
        }
        // A PLM section asked for a file (Attach a file…, Add file…): the file
        // chooser answers, and the file goes back to the section that asked.
        if let Some((tag, title)) = plm.pick_file {
            if let Err(error) = self.file.request_pick_file(self.model_store.as_ref(), &tag, &title) {
                self.file.status = error;
            }
        }
        for tag in self.plm.pick_tags() {
            if let Some(file) = self.file.take_picked_file(&tag) {
                self.plm.deliver_picked(&tag, file);
            }
        }
        if plm.refresh_inbox {
            self.toolbar.refresh_inbox();
        }
        // Save As → "A new part" (D9): the PLM pane's New part form, shown.
        if self.file.take_plm_new_part() {
            self.plm.open_new_part();
            self.show_pane(PaneKind::Plm);
        }
        if self.plm.busy() {
            ui.ctx().request_repaint();
        }
        // Following other clients (`panels::plm_follow`): when the feed moved,
        // the inbox asks again too; and the next question gets a frame.
        if self.plm.take_followed() {
            self.toolbar.refresh_inbox();
        }
        if let Some(next) = self.plm.next_follow_in() {
            ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(next.max(0.1)));
        }
        if sketch || ref_select {
            // Special mode: the viewport fills the remaining central area; no
            // dock, no modeling side panes — and therefore no DOCUMENT TAB
            // STRIP either, which is the guard that keeps a live sketch /
            // reference-pick session from having its document swapped out from
            // under it.
            self.viewport.show(ui, self.docs.engine_mut());
            DockState::retract_all(DockContext {
                docs: &mut self.docs,
                plugins: &mut self.plugins,
                viewport: &mut self.viewport,
                history: &mut self.history,
                bom: &mut self.bom,
                assembly_constraints: &mut self.assembly_constraints,
                wire_harness: &mut self.wire_harness,
                pmi: &mut self.pmi,
                sheets: &mut self.sheets,
                scene: &mut self.scene,
                expressions: &mut self.expressions,
                qualify: &mut self.qualify,
                family_table: &mut self.family_table,
                update_components: &mut self.update_components,
                model_store: self.model_store.as_ref(),
                plm: &mut self.plm,
            });
        } else {
            // Normal mode: the dock owns the whole central area (the viewport is a
            // pane). Cross-panel requests the panels can't act on while their
            // borrows are held bubble OUT via the returned outcome — the SAME
            // requests the old left-panel closure produced.
            let outcome = self.dock.ui(
                ui,
                DockContext {
                    docs: &mut self.docs,
                    plugins: &mut self.plugins,
                    viewport: &mut self.viewport,
                    history: &mut self.history,
                    bom: &mut self.bom,
                    assembly_constraints: &mut self.assembly_constraints,
                    wire_harness: &mut self.wire_harness,
                    pmi: &mut self.pmi,
                    sheets: &mut self.sheets,
                    scene: &mut self.scene,
                    expressions: &mut self.expressions,
                    qualify: &mut self.qualify,
                    family_table: &mut self.family_table,
                    update_components: &mut self.update_components,
                    model_store: self.model_store.as_ref(),
                    plm: &mut self.plm,
                },
            );

            // The ACOMP palette pick must open the COMPONENT SELECTOR, never a
            // bare feature dialog — the file dialog is shell-owned.
            if outcome.insert_component_requested {
                self.file.dispatch(
                    FileAction::InsertComponent,
                    &mut self.docs,
                    self.model_store.as_ref(),
                );
            }
            // The DOCUMENT TAB STRIP inside the viewport tile. Activation is
            // immediate; a close routes through the file dialog because a dirty
            // document has to be confirmed first, and that prompt lives there.
            if let Some(index) = outcome.document_tabs.activate {
                self.docs.activate(index);
            }
            if let Some(index) = outcome.document_tabs.close {
                self.file.request_close(&mut self.docs, index);
            }
            self.document_tab_hits = outcome.document_tabs.hits;
            // A structure-tree Edit — or a BOM row's action button, which
            // reports through the same outcome field so there is one arm and
            // not two — expands its feature in the history tree.
            if let Some(focus) = outcome.feature_focus {
                self.history.focus_feature(focus);
            }
            // Structure-tree interaction hooks route through the SAME dispatcher
            // as the context bar (one truth per action); document-level flows
            // (edit-in-place / open-part) come back as requests the shell runs.
            // A BOM row menu's document-level flow: its engine-mutating half
            // already ran inside the panel, through the same dispatcher.
            match outcome.component_request {
                Some(ComponentActionRequest::OpenPart { component_id }) => {
                    self.edit_part(&component_id);
                }
                None => {}
            }
        }

        self.history.sync_palette_display(self.model_store.as_ref());

        // --- file dialog: a ctx-level modal (like the command palette), drawn
        // after the panels so its backdrop dims the whole shell. Idempotent when
        // closed; also polls for a completed async import each frame.
        self.file
            .show(&ctx, &mut self.docs, self.model_store.as_ref());
        self.plm.save_prompt(&ctx, &mut self.docs, &mut self.file, self.model_store.as_ref());

        // --- crash recovery: the boot prompt, then the debounced autosave -----
        // Drawn with the same ctx-level modal treatment as the file dialog. The
        // autosave ticks only once the prompt has resolved (or never existed).
        if self.recovery.is_open() {
            if let Some(resolution) =
                self.recovery.show(&ctx, &mut self.docs, self.model_store.as_ref())
            {
                self.autosave.note_cleared();
                if let crate::recovery::Resolution::Restored(count) = resolution {
                    self.docs.engine_mut().push_notice_as(NoticeSeverity::Info, format!(
                        "Restored {count} unsaved document{}",
                        if count == 1 { "" } else { "s" }
                    ));
                }
            }
        } else {
            self.recovery.clear_hits();
            let now = ctx.input(|i| i.time);
            if let Some(due) = self.autosave.tick(&self.docs, self.model_store.as_ref(), now) {
                // The frame loop idles between inputs; wake it when the write is due.
                ctx.request_repaint_after(std::time::Duration::from_secs_f64(due.max(0.05)));
            }
        }

        if let Some((format, name, bytes)) = self.file.take_stl_import() {
            self.start_mesh_preview(format, name, bytes);
            ctx.request_repaint();
        }

        // --- Submit Bug: the screenshot-capture state machine + report modal.
        // Drawn at ctx level like the file dialog; idempotent while idle. Draws
        // NOTHING during capture, so the screenshot it requested never contains
        // this dialog.
        self.bug_report.show(&ctx, self.docs.engine_mut());

        // --- Info: the licences + this session's diagnostics + the MCP start
        // button, a floating window toggled from the toolbar's info button.
        // Idempotent when closed. The server's state is read here, each frame
        // the window is open, and a click is answered here: the shell owns the
        // queue the server attaches to and the adapter line its session names.
        #[cfg(all(not(target_arch = "wasm32"), feature = "mcp"))]
        let mcp_view = if self.info.open { crate::mcp::info_view() } else { crate::panels::info::McpView::Unavailable };
        #[cfg(not(all(not(target_arch = "wasm32"), feature = "mcp")))]
        let mcp_view = crate::panels::info::McpView::Unavailable;
        self.info.show(&ctx, &self.diagnostics, &mcp_view);
        #[cfg(all(not(target_arch = "wasm32"), feature = "mcp"))]
        if self.info.take_mcp_start() {
            // The failure is already in `mcp::status()`, which the window
            // shows next frame, and on stderr in the same words.
            let _ = crate::mcp::start_from_window(self.automation.clone(), self.diagnostics.adapter_line());
        }

        // --- Settings: a floating (movable + resizable) window, toggled from the
        // toolbar gear button, drawn at ctx level like Properties so it floats
        // over the shell. Idempotent when closed. Replaces the old sidebar section.
        // This preview runs every frame, including when Settings is closed.
        // Exact dirty checks serialize the whole assembly; use the tab's cached
        // marker here. reconnect_plm still checks the exact saved bytes before
        // switching stores, including metadata edits the marker may not see.
        self.settings.reconnect_blocker =
            self.reconnect_blocker_with(crate::document::Document::dirty_marker);
        self.settings
            .show(&ctx, self.docs.engine_mut(), self.model_store.as_ref());
        // The PLM tab's Connect now: switch this session to the PLM live.
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(root) = self.settings.take_reconnect_request() {
            let outcome = self.reconnect_plm(root);
            self.settings.reconnected(outcome);
        }

        // --- Part Properties: the active document's own BOM attribute record,
        // in a floating window beside Settings. The title is passed in because
        // the panel takes only the engine, and a user with several tabs open
        // must be able to see WHICH part they are annotating.
        let part_title = self.docs.active().title();
        let part_document = self.docs.active().id();
        self.part_properties.configure_plm(self.model_store.plm_client(), self.docs.active().name());
        self.part_properties
            .show(&ctx, self.docs.engine_mut(), &part_title, part_document);

        // --- top-right overlay column: the special-mode EXIT card (Finish/Cancel
        // for reference-selection / sketch mode) stacked ABOVE the selection-driven
        // CONTEXT ACTION rail. Both cards live in ONE ctx-level Area anchored
        // top-right so they never overlap, and the context rail uses the SAME
        // renderer whether it is showing modeling actions or sketch actions
        // (`panels::action_rail`). A modeling create/edit action returns a feature
        // id to expand in the history tree.
        {
            let mut focus: Option<String> = None;
            let mut info_targets: Vec<String> = Vec::new();
            let mut plugin_action = None;
            let mut component_request: Option<ComponentActionRequest> = None;
            // Anchor the overlay to the RIGHT edge of the 3D VIEW (the viewport
            // tile), not the window — so it stays glued to the viewport wherever
            // docking frames it. The viewport was drawn earlier THIS frame, so its
            // rect is current. Before the first draw (`None`) fall back to the
            // window's top-right.
            let mut overlay = egui::Area::new(egui::Id::new("brep-top-right-overlay"))
                .order(egui::Order::Foreground);
            overlay = match self.viewport.last_rect() {
                Some(rect) => overlay
                    .fixed_pos(rect.right_top() + egui::vec2(-12.0, 8.0))
                    .pivot(egui::Align2::RIGHT_TOP),
                None => overlay.anchor(egui::Align2::RIGHT_TOP, egui::vec2(-12.0, 56.0)),
            };
            overlay
                .show(&ctx, |ui| {
                    // 0. The family-row preview's card: which row, and Exit.
                    self.family_table.preview_card(ui, self.docs.engine_mut());
                    // 1. Exit controls for whatever special mode is active.
                    self.mode_bar.card(ui, self.docs.engine_mut());
                    // 2. Context actions: sketch actions in sketch mode, else the
                    // modeling selection actions. Same rail, mode-appropriate items.
                    if self.docs.engine().sketch_mode() {
                        self.sketch.context_card(ui, self.docs.engine_mut());
                    } else {
                        let outcome = self.context_bar.card(ui, self.docs.engine_mut());
                        focus = outcome.focus;
                        info_targets = outcome.info_targets;
                        plugin_action = outcome.plugin_action;
                        component_request = outcome.component;
                    }
                });
            if let Some(focus) = focus {
                self.history.focus_feature(focus);
            }
            // The Info action returns one target per selected entity — open (or, on
            // dedup, keep) a pinned Info window for each. Drawn below.
            if let Some(id) = plugin_action {
                if let Err(e) = self.plugins.open_action(self.docs.engine(), &id) { self.plugins.error = Some(e); }
            }
            if !info_targets.is_empty() {
                self.info_windows.open_for(&info_targets, self.viewport.last_rect());
            }
            // Component document-level flows (the engine-mutating component
            // actions already ran inside the bar).
            match component_request {
                Some(ComponentActionRequest::OpenPart { component_id }) => {
                    self.edit_part(&component_id);
                }
                None => {}
            }
        }

        // --- THE DIALOG DOOR: every dynamically generated dialog that opened
        // this frame makes its host pane's tab current.
        //
        // ONE call, placed after everything that can open one — the dock's panes
        // and viewport, and the context bar's overlay above — so a form opened
        // anywhere is surfaced in the same frame, before `__brepDock` publishes
        // the active tab below. It watches the TARGETS rather than the callers
        // because most of them set the target inside the engine or inside a
        // panel's draw, with no dock in reach; `panels::dock::DialogTargets`
        // carries that reasoning and the openers it covers.
        let targets = crate::panels::dock::DialogTargets::read(
            &self.history,
            self.docs.engine(),
            self.docs.active_id(),
        );
        self.dock.surface_opened_dialogs(targets);

        // --- Info windows: the pinned per-entity inspector windows, drawn at ctx
        // level like the file dialog so they float over the shell. Each is pinned to
        // its open-time object name (selection changes never retarget them); closed
        // windows (their `×`) are pruned here. Drawn AFTER the context bar so a
        // window opened THIS frame paints this frame.
        self.info_windows.show(&ctx, self.docs.engine_mut());

        // --- interference results window: same floating idiom, owned report;
        // its Re-run button re-drives the engine check.
        self.interference.show(&ctx, self.docs.engine_mut());

        // --- auto-constraints window: the inference scan + its Create button.
        self.auto_constraints.show(&ctx, self.docs.engine_mut());
        self.step_parts
            .show(&ctx, self.docs.engine_mut(), self.model_store.as_ref());

        // --- transient toasts: drain the engine's queued notices (e.g. a sketch
        // solve that failed after an edit) and show each briefly. Drawn last so
        // the cards float over the whole shell.
        let now = ctx.input(|i| i.time);
        // All three sources gathered FIRST, because the automation queue must see
        // every one of them: `Toasts::extend` keeps only the last few on screen,
        // and a message dropped for space is exactly the one a host wants told.
        use brep_render::engine_state::NoticeSeverity;
        let mut raised = self.docs.engine_mut().take_graded_notices();
        // Same lane for STORAGE failures the store could only discover after its
        // synchronous `write` returned `Ok` (the browser backend writes behind an
        // in-memory mirror). A save that did not persist must never be silent.
        let errors = self.model_store.take_persistence_errors().into_iter().chain(self.autosave.take_errors());
        raised.extend(errors.map(|text| (NoticeSeverity::Error, text)));
        // An edit a read-only document refused (a PLM revision not checked out,
        // or released): its history kept the saved bytes, and this says why.
        for doc in self.docs.iter_mut() {
            if let Some(text) = doc.take_refused() {
                raised.push((NoticeSeverity::Warning, text));
            }
        }
        // The automation channel's `notices` drain: what the user would have SEEN.
        // Without this the drain has no producers at all and every script's
        // `{"notices": []}` asserts nothing.
        #[cfg(feature = "automation")]
        for (_, text) in &raised {
            self.automation.push_notice(
                crate::automation::command::NoticeKind::Toast,
                text.clone(),
            );
        }
        self.toasts.extend(raised, now);
        // An edit after a card was raised supersedes it (`Toasts::note_document`).
        self.toasts
            .note_document(self.docs.active_id(), self.docs.engine().history.undo_top(), now);
        self.toasts.show(&ctx, self.viewport.last_rect(), status_bar_top);

        // The state registry (`automation::registry`): the live app + engine
        // state, published by name with a one-line doc so a host can read it
        // (and, on wasm, mirrored to `window.__brep*` for the verify scripts).
        // Purely additive; no render effect. Published AFTER the panels draw so
        // the hit-rects are for THIS frame's layout. Off unless a host enabled it.
        if crate::automation::registry::enabled() {
            let _publish_span = crate::perf::span(crate::perf::Phase::Publish);
            let ppp = ui.ctx().pixels_per_point();
            crate::automation::registry::publish("__brepCamera", "camera state: kind, eye, target, up, near/far, projection block, worldPerPixel", &self.docs.engine().camera_state_json());
            crate::automation::registry::publish("__brepSettings", "render and UI settings", &self.docs.engine().settings_json());
            crate::automation::registry::publish("__brepSolidColors", "per-solid colour overrides", &self.docs.engine().solid_color_overrides_json());
            crate::automation::registry::publish("__brepHistory", "history listing {step, features:[{index,type,id}]}", &self.docs.engine().history_listing_json());
            crate::automation::registry::publish("__brepGizmo", "transform gizmo state", &self.docs.engine().gizmo_state_json());
            crate::automation::registry::publish("__brepFile", "file dialog state (mode, entries, current name)",
                &self.file.file_state_json(&self.docs, self.model_store.as_ref()),
            );
            crate::automation::registry::publish("__brepFileHit", "file dialog widget rects", &self.file.hits_json());
            crate::automation::registry::publish("__brepModel", "model signature: solid count, triangle count, per-solid bounds (change detection)", &self.model_signature_json());
            crate::automation::registry::publish("__brepReport", "last run report {featureErrors, featureNotes, featureFulfilment, featureRefusals (only when a feature failed with a typed refusal), featureApproximations (only when a successful feature carries a measured approximation: feature id → [{code, body, measured, bar, volume_bound, edges, message, summary}]), unresolved, displayErrors, featureTimings, featureOutputs}", &self.docs.engine().history_report_json());
            // The in-flight run: whether one is pending, the feature the runner
            // says it is executing, and the feature a cancelled run was stuck on.
            crate::automation::registry::publish("__brepRun", "in-flight run {pending, progress:{generation,index,total,featureId,featureType}|null, cancelled}",
                &serde_json::json!({
                    "pending": self.docs.engine().run_pending(),
                    "progress": self.docs.engine().run_progress().map(|p| serde_json::json!({
                        "generation": p.generation,
                        "index": p.index,
                        "total": p.total,
                        "featureId": p.feature_id,
                        "featureType": p.feature_type,
                    })),
                    "cancelled": self.docs.engine().cancelled_run(),
                })
                .to_string(),
            );
            crate::automation::registry::publish("__brepBusy", "the status bar's working indicator {shown, activities:[{kind, label, document, cancellable, elapsed}]} — everything in flight, most important first; shown once work has lasted 0.2 s", &self.busy.state_json());
            crate::automation::registry::publish("__brepStatusHit", "status bar widget rects: busy, busy:more, busy:cancel (present only while the working indicator shows)", &self.busy.hits_json());
            crate::automation::registry::publish("__brepHit", "history panel widget rects: step:i edit:i del:i box:i add:menu form:* field:* panel:clip", &self.history.hits_json());
            crate::automation::registry::publish("__brepPaletteHit", "add-feature palette widget rects in screen points: input top item:type display display:mode (empty while closed)", &self.history.palette_hits_json());
            crate::automation::registry::publish("__brepExprHit", "expressions panel widget rects", &self.expressions.hits_json());
            crate::automation::registry::publish("__brepExpr", "expressions script, its variables and the configurator",
                &serde_json::json!({
                    "expressions": self.docs.engine().expressions_json(),
                    "variables": serde_json::from_str::<serde_json::Value>(
                        &self.docs.engine().expression_variables_json()
                    )
                    .unwrap_or(serde_json::Value::Null),
                    "configurator": serde_json::from_str::<serde_json::Value>(
                        &self.docs.engine().configurator_json()
                    )
                    .unwrap_or(serde_json::Value::Null),
                })
                .to_string(),
            );
            crate::automation::registry::publish("__brepToolbar", "toolbar rects: shared File menu file:* and workbench:item:<id>; Ribbon tabs/overflow ribbon:* and Home creation wbtb:*; command keys undo redo fit projection wireframe show:* help info bug workbench:btn:<id>. Classic wraps its controls; Ribbon compacts Large commands before whole-group overflow. Keys appear when their widgets are rendered; menu entries appear while open", &self.toolbar.hits_json());
            // The workbench actions strip's button rects (`wbtb:feature:<type>` /
            // `wbtb:constraint:<type>` / `wbtb:annotation:<type>`); an empty map
            // while the strip is hidden.
            crate::automation::registry::publish("__brepWorkbenchToolbar", "creation rects: wbtb:* feature:type constraint:type annotation:type, from the Classic workbench strip or Ribbon Home; empty while hidden or another Ribbon tab is selected", &self.workbench_toolbar.hits_json());
            // The PLM pane (a PLM session only): its sections' widget rects,
            // `plm/section:<id>` and `plm/<section>:<widget>`, and the active
            // document's access and revision.
            crate::automation::registry::publish("__brepPlmHit", "PLM pane rects: section:lifecycle, lifecycle:revision:<label>, lifecycle:verb:check-out, save:<yes|no|remember>, web:<summary|revisions|attachments|structure|history|parts|workspace|reviews|ecos>", &self.plm.hits_json());
            crate::automation::registry::publish("__brepPlmDoc", "the active document on the PLM: {access:{editable, reason}, plmDocument, revision:{id,label,lifecycle,lockedBy,lockedByMe}, busy, message, offered:[verb key], files:{staged:{name,mediaType,kind,size,target}, revision:[name], part:[name], problem, notice, status, busy}}; null with no PLM session", &self.plm.state_json(&self.docs));
            // The queued toast texts — the only trace of a refusal the app shows
            // as a transient card (e.g. a constraint the strip could not add).
            crate::automation::registry::publish("__brepNotices", "queued toast texts", &self.toasts.texts_json());
            crate::automation::registry::publish("__brepToasts", "toast cards {cards: [{text, severity (error|warning|info), count, shown, superseded}], shown, folded}", &self.toasts.state_json());
            crate::automation::registry::publish("__brepToastsHit", "toast card rects (toast:<i> top to bottom, toast:more)", &self.toasts.hits_json());
            crate::automation::registry::publish("__brepBug", "bug report panel state", &self.bug_report.state_json());
            crate::automation::registry::publish("__brepDiagnostics", "what this session is running on: renderer, adapter, texture ceiling, version, platform", &self.diagnostics.json().to_string());
            // Where the last ~120 frames' milliseconds went. `__brepDiagnostics`
            // says WHICH renderer is drawing; this says what it costs, in the
            // same split the Info window's Performance rows show.
            crate::automation::registry::publish("__brepPerf", "rolling per-frame timings in ms {frames, window, dt, ui, publish, sync, fit, overlays, draw} each {avg,p95,max}", &crate::perf::json());
            crate::automation::registry::publish("__brepBugHit", "bug report panel widget rects", &self.bug_report.hits_json());
            // The JavaScript editor window: its outer rect against the surface
            // it may fill, and its widget rects (`javascript/panel:clip`,
            // `javascript/source`, `javascript/output`, the buttons).
            crate::automation::registry::publish("__brepJavascript", "JavaScript editor window {open, rect:[x,y,w,h] or null while closed, surface:[w,h], running, output}", &self.javascript.state_json());
            crate::automation::registry::publish("__brepJavascriptHit", "JavaScript editor window widget rects (panel:clip, run, cancel, save, open, export, help, example, source, output); empty while the window is closed", &self.javascript.hits_json());
            // The wire harness: the document's connections + the last run's
            // routing report (engine truth), and the panel's widget rects.
            crate::automation::registry::publish("__brepWireHarness", "wire harness connections and the last routing report", &self.docs.engine().wire_harness_state_json());
            crate::automation::registry::publish("__brepWireHarnessHit", "wire harness panel widget rects", &self.wire_harness.hits_json());
            // The part's declared connection points (the `ports` block) and the
            // ports tail's report, beside the Qualify panel's own selection and
            // its rects (`qualify:*`).
            crate::automation::registry::publish("__brepPorts", "the part's declared connection points (the `ports` block) and the ports tail's report of the last run", &self.docs.engine().ports_state_json());
            crate::automation::registry::publish("__brepQualify", "Qualify panel state {selected: the connection-point address it has selected}", &self.qualify.state_json());
            crate::automation::registry::publish("__brepQualifyHit", "Qualify panel widget rects (qualify:*)", &self.qualify.hits_json());
            // The family table as the Family table pane last read it, its
            // problems, selection and the last Generate report; and its rects.
            crate::automation::registry::publish("__brepFamily", "Family table pane state {table, expressions, notColumns, selected, editing, problems, columnProblems, values, generateCalls, report, status, preview: {row, partNumber, applied, showing, message, featureErrors} or null, runsReplied}", &self.family_table.state_json());
            crate::automation::registry::publish("__brepFamilyHit", "Family table pane widget rects (family:*)", &self.family_table.hits_json());
            // The PMI block + report + active view / open annotation, and the
            // PMI panel's rects (`pmi:capture`, `pmi:add`, `pmi:add:<type>`,
            // `pmi:row:<id>`, `pmi:cell:<id>:<column>`, `pmi:menu:<id>`, the
            // form's `pmi:` keys).
            crate::automation::registry::publish("__brepPmi", "PMI block, report, active view, open annotation or view dialog, and the selected annotation or view", &self.docs.engine().pmi_state_json());
            crate::automation::registry::publish("__brepPluginsHit", "Declarative plugin panel action controls", &self.plugins.panel_hits_json());
            crate::automation::registry::publish("__brepPmiHit", "PMI panel widget rects (pmi:*)", &self.pmi.hits_json());
            // The `sheets` block, which sheet the viewport draws, and the OPEN
            // sheet's whole projection — the paper, the visible edge runs and
            // the annotations that passed the parallel rule, in millimetres.
            // The panel's rects (`sheets:add`, `sheets:place:<view>`,
            // `sheets:row:<id>`, the form's `sheets:` keys) ride beside it.
            crate::automation::registry::publish("__brepSheets", "drawing sheets, the open sheet and its projection, the open and the selected sheet object", &self.docs.engine_mut().sheet_state_json());
            crate::automation::registry::publish("__brepSheetsHit", "Sheets panel widget rects (sheets:*)", &self.sheets.hits_json());
            // The workbench logical state (resolved current id + available ids) so
            // the verifier can drive the dropdown and confirm the active workbench.
            // Hit-rects for the dropdown ride in `__brepToolbar` (self.toolbar.hits).
            crate::automation::registry::publish("__brepWorkbench", "active workbench id and the available ids",
                &crate::workbench::workbench_state_scoped(self.docs.engine()).to_string(),
            );
            crate::automation::registry::publish("__brepSelection", "selection {solids, faces, edges, datums, vertices}; the name arrays are in pick order, oldest first", &self.docs.engine().selection_json());
            crate::automation::registry::publish("__brepInfoWindows", "open info windows (mass properties, topology, metadata)",
                &self.info_windows.published_json(self.docs.engine_mut()),
            );
            crate::automation::registry::publish("__brepInfoWindowsHit", "info window widget rects", &self.info_windows.hits_json());
            crate::automation::registry::publish("__brepInterference", "interference check state", &self.interference.state_json());
            crate::automation::registry::publish("__brepInterferenceHit", "interference panel widget rects", &self.interference.hits_json());
            crate::automation::registry::publish("__brepAutoConstraints", "auto-constraint inference state", &self.auto_constraints.state_json());
            crate::automation::registry::publish("__brepAutoConstraintsHit", "auto-constraints widget rects", &self.auto_constraints.hits_json());
            crate::automation::registry::publish("__brepStepParts", "STEP parts library panel state", &self.step_parts.state_json());
            crate::automation::registry::publish("__brepStepPartsHit", "STEP parts library widget rects", &self.step_parts.hits_json());
            crate::automation::registry::publish("__brepSelectionFilter", "which entity kinds a viewport click may pick", &self.docs.engine().selection_filter_json());
            crate::automation::registry::publish("__brepSelectionHit", "selection bar widget rects (filter:kind, clear, hide)", &self.selection.hits_json());
            crate::automation::registry::publish("__brepContext", "context action bar state (offers for the selection)", &self.context_bar.state_json());
            crate::automation::registry::publish("__brepContextHit", "context action bar widget rects", &self.context_bar.hits_json());
            crate::automation::registry::publish("__brepModeBarHit", "mode bar widget rects (refsel:*, Sketch:*)", &self.mode_bar.hits_json());
            // The DOCUMENT TABS: the open models, which one is active, and the
            // strip's per-tab hit-rects, so an e2e script can switch and close
            // documents the way a user does.
            crate::automation::registry::publish("__brepDocuments", "open document tabs {active, tabs:[{title,name,dirty}]}", &document_tabs::state_json(&self.docs));
            crate::automation::registry::publish(
                "__brepDocumentsHit",
                "document tab strip widget rects",
                &crate::automation::hit_rects::hits_json(
                    self.document_tab_hits.iter().map(|(key, rect)| (key, rect)),
                ),
            );
            // The boot-time recovery prompt: its entries and its two buttons.
            crate::automation::registry::publish("__brepRecovery", "boot-time recovery prompt entries", &self.recovery.state_json());
            crate::automation::registry::publish("__brepRecoveryHit", "recovery prompt widget rects", &self.recovery.hits_json());
            crate::automation::registry::publish("__brepComponentMove", "assembly component move state", &self.docs.engine().component_move_json());
            crate::automation::registry::publish("__brepSketch", "sketch mode state (session, tool, selection, constraints)", &self.sketch.published_json(self.docs.engine()));
            crate::automation::registry::publish("__brepSketchHit", "sketch-mode overlay widget rects: the SKETCH ACTIONS card (constraint:glyph fix construction cleanup delete) \u{2014} empty out of sketch mode. The draw tools are NOT here: they are workbench buttons in the toolbar row (toolbar/workbench:btn:sketch.tool.* and sketch.autoconstrain)", &self.sketch.hits_json(self.docs.engine()));
            crate::automation::registry::publish("__brepSketchListHit", "sketch entity-list widget rects: section:* geometry:id point:id constraint:id del:kind:id panel:clip (empty out of sketch mode)", &self.sketch.list_hits_json(self.docs.engine()));
            crate::automation::registry::publish("__brepWireframe", "wireframe toggle", &format!("{}", self.docs.engine().settings.wireframe));
            crate::automation::registry::publish("__brepRefSelect", "reference-selection picker {active, prompt, names}",
                &serde_json::json!({
                    "active": self.docs.engine().ref_select_active(),
                    "prompt": self.docs.engine().ref_select_prompt(),
                    "names": self.docs.engine().ref_select_names(),
                })
                .to_string(),
            );
            // Viewport origin + projected probe points (viewport-local logical
            // px) so the verifier can click precise spots ON the Box and ON the
            // Pin during ref-select mode. Index 0 is a Box top-corner clear of the
            // pin; indices 1..4 are points on the Pin's cylindrical stub that
            // protrudes above the Box top (y=20), on the camera-facing sides — the
            // verifier tries them until one picks "Pin".
            crate::automation::registry::publish("__brepView", "the 3D viewport rect {x,y,w,h} in egui points", &self.viewport.viewport_rect_json());
            // Dock layout snapshot (per-pane visible / rendered) so the verifier
            // can see which side panels are on-screen and, once a user tabs panels
            // together, activate the right tab before asserting on its widgets.
            // `active=false` in sketch / ref-select (the dock is bypassed).
            crate::automation::registry::publish("__brepDock", "dock layout in tab-strip order: per-pane visible/activeTab/rendered", &self.dock.state_json(!sketch && !ref_select));
            crate::automation::registry::publish("__brepProbe", "projected seed-model probe points (viewport-local px) for the verifier",
                &self
                    .docs
                    .engine()
                    .world_to_screen_json(
                        "[[2.0,20.0,2.0],[14.243,22.5,14.243],[16.0,22.5,10.0],\
                          [10.0,22.5,16.0],[10.0,25.0,10.0]]",
                    )
                    .unwrap_or_else(|_| "[]".to_string()),
            );
            crate::automation::registry::publish("__brepPpp", "pixels per point of the surface", &format!("{ppp}"));
            crate::automation::registry::publish("__brepStep", "the rolled-to feature index", &format!("{}", self.docs.engine().history_rollback()));
            crate::automation::registry::publish("__brepParams", "inputParams of the rolled-to feature", self.params_blob());
        }

        // Phase 3 of the automation frame (§4.4): reads answer from THIS frame's
        // registry and layout.
        #[cfg(feature = "automation")]
        self.drain_reads(&ctx);

        // NOTE: the 3D viewport is no longer drawn here — it is a dock tile drawn
        // earlier this frame (normal mode) or drawn directly in the sketch /
        // ref-select branch above. Drawing it before the top-right overlay is what
        // keeps that overlay anchored to the live viewport rect.
    }
}


/// Mirror an engine JSON string to `window.<name>` (wasm/verification only).

/// The seed model handed to the engine at startup: a 3-feature history so the
/// tree / roll / edit are real —
///   0. `P.CU` "Box"  — a 20 mm cube at the origin (spans `[0,20]³`).
///   1. `P.CY` "Pin"  — a r=6, h=30 cylinder (axis +Y) positioned to pierce the
///      cube through its centre in XZ (x=10, z=10) from below (y=-5) to above.
///   2. `B`    "Cut"  — SUBTRACT: `targetSolid = Box`, tools `[Pin]` → the cube
///      with a cylindrical through-hole (the ref-select field is visible for the
///      next slice). Roll-to-step shows: cube → cube+cylinder → subtracted cube.
///
/// This is just the INITIAL document — once handed to `EngineState`, the engine
/// OWNS the mutable history; the app keeps no copy.
pub(crate) fn seed_history_json() -> String {
    serde_json::json!({
        "expressions": "",
        "configurator": {},
        "features": [
            {
                "type": "P.CU",
                "inputParams": {
                    "id": "Box",
                    "sizeX": 20.0, "sizeY": 20.0, "sizeZ": 20.0,
                    "transform": {
                        "position": [0.0, 0.0, 0.0],
                        "rotationEuler": [0.0, 0.0, 0.0],
                        "scale": [1.0, 1.0, 1.0]
                    },
                    "boolean": { "targets": [], "operation": "NONE", "mergeCoplanarFaces": true }
                },
                "persistentData": {}
            },
            {
                "type": "P.CY",
                "inputParams": {
                    "id": "Pin",
                    "radius": 6.0, "height": 30.0,
                    "transform": {
                        "position": [10.0, -5.0, 10.0],
                        "rotationEuler": [0.0, 0.0, 0.0],
                        "scale": [1.0, 1.0, 1.0]
                    },
                    "boolean": { "targets": [], "operation": "NONE", "mergeCoplanarFaces": true }
                },
                "persistentData": {}
            },
            {
                "type": "B",
                "inputParams": {
                    "id": "Cut",
                    "targetSolid": "Box",
                    "boolean": { "operation": "SUBTRACT", "targets": ["Pin"], "mergeCoplanarFaces": true }
                },
                "persistentData": {}
            }
        ]
    })
    .to_string()
}
