//! [`EngineState`] — the windowing-agnostic viewer state machine the host UI
//! programs against (R3): scene + camera + controls + settings + emphasis, plus
//! the whole event/command/query surface (run-history feed, pointer/wheel
//! ingestion, camera commands, picking, world→screen, visibility). No GPU, no
//! canvas — the wasm `Engine` and the winit desktop shell both wrap this; it is
//! fully unit-testable on native.
//!
//! Everything crosses the R3 boundary as plain JSON/scalars: the host never holds a
//! renderer object, only names, ids, and JSON.

use crate::controls::ArcballControls;
use crate::history::History;
use crate::pick::{self, PickOptions};
use crate::scene::RenderScene;
use crate::style::{Emphasis, RenderSettings};
use crate::view::ViewCamera;
use crate::widgets::{gizmo_camera, WidgetOverlay, WidgetRegistry};
use brep_kernel::HistoryRequest;
use std::collections::HashMap;

/// One DIALOG row's hover state — which panel lit it, the row's own TEXT, and
/// what that text resolved to in the scene.
///
/// The row text is the memo key rather than the candidate's name because the two
/// differ whenever the derived-face fallback fires (a fillet's `Edges` row reads
/// `A|B[0]` while the face it lights is `F1:BLEND:A|B[0]` — see
/// [`EngineState::hover_entity_by_name`]). An unresolvable row memoizes as
/// `candidate: None`, so a held hover over a name the scene does not carry scans
/// the scene ONCE, not once per frame.
#[derive(Clone)]
pub(crate) struct DialogHover {
    /// Which panel set it (`"history"`, `"constraints"`, `"pmi"`, `"refsel"`) —
    /// [`EngineState::dialog_hover_end`] acts only for the owner that set it, so a
    /// pane that never hovered a row cannot end another pane's hover.
    pub(crate) owner: &'static str,
    /// The row's TEXT, exactly as the dialog listed it.
    pub(crate) row: String,
    /// What it resolved to, or `None` for a row the scene does not carry.
    pub(crate) candidate: Option<pick::PickCandidate>,
}

pub struct EngineState {
    pub scene: RenderScene,
    pub camera: ViewCamera,
    pub controls: ArcballControls,
    pub settings: RenderSettings,
    pub emphasis: Emphasis,
    /// In-scene overlay widgets: datums/dimensions/curves, transform
    /// gizmo, ViewCube — fed as JSON, drawn by the render core's overlay pass.
    pub widgets: WidgetRegistry,
    /// The engine-owned editable model recipe (ordered features + rollback
    /// index) — the SINGLE source of truth for the model. The UI never keeps its
    /// own copy; it mutates/reads this through the `history_*` / feature methods.
    pub history: History,
    interference_sessions: std::collections::BTreeMap<String, interference::InterferenceCursor>,
    interference_sequence: u64,
    illustration_snapshot: Option<illustration::IllustrationSnapshot>,
    geometry_diagnostics: std::collections::BTreeMap<String, serde_json::Value>,
    /// The build report (`{featureErrors, unresolved, displayErrors}`) of the
    /// last history run, so the UI can show it without re-running.
    history_report: String,
    /// The Properties-panel metadata store: user attributes keyed by OBJECT NAME
    /// (solid / face / edge kernel name), NOT feature id — so a record survives
    /// feature edits as long as the object's name persists. Persisted with the
    /// model (a top-level `metadata` field in the history document); see the
    /// [`crate::metadata`] module for the store + the object-info/measurement API.
    pub metadata: crate::metadata::MetadataStore,
    /// Derived preview data stays outside the recipe and undo history: the
    /// document's own picture at a history edit serial (opened with it, or the
    /// last one a save rendered), and the last picture a save produced by key.
    document_thumbnail: std::cell::RefCell<Option<(u64, serde_json::Value)>>,
    thumbnail_cache: std::cell::RefCell<Option<(Vec<u64>, serde_json::Value)>>,
    /// Bumped whenever settings change so the renderer re-derives per-solid
    /// base styles (a cheap key, not a per-frame diff).
    pub settings_generation: u64,
    /// The engine sets this whenever the camera/scene/emphasis changed; the
    /// presentation shell renders only when it is set (R22 on-demand render —
    /// the OrthoCameraIdle matrix-compare analogue, made explicit).
    pub dirty: bool,
    /// The modal reference-selection state (the ref-select widget). `Some` while
    /// the user is picking references for a feature-dialog field; `None`
    /// otherwise. The picked-name list here is the SINGLE source of truth while
    /// active (the UI reads it back; the viewport appends to it on a pick).
    pub ref_select: Option<RefSelectState>,
    /// Which entity KINDS a plain viewport click may select (the selection
    /// filter, mirroring the earlier `SelectionFilter.allowedSelectionTypes`). `select_top_at`
    /// consults it via `pick_filtered`; see the appended `SelectionFilter` impl
    /// block near the end of this file for the state + honoring logic.
    pub selection_filter: SelectionFilter,
    /// The transform-controls gizmo controller: which feature (if any) has the
    /// move/rotate gizmo armed (via the in-viewport center-sphere toggle), plus the
    /// in-flight handle drag. All the arm/drag/apply logic lives in the appended
    /// transform-gizmo impl block near the end of this file.
    pub transform_gizmo: TransformArm,
    /// The COMPONENT Move gizmo controller (assemblies §8.5): which ACOMP
    /// instance has the bbox-center move/rotate gizmo armed, the translate/
    /// rotate cycle mode, and the commit-on-release drag. Exclusive with
    /// `transform_gizmo` (shared widget slot). See `component_move.rs`.
    pub component_move: ComponentMoveArm,
    /// The active engine-native sketch edit (`Some` while in sketch mode). Holds
    /// the live [`crate::sketch::SketchSession`] plus the pre-entry camera + roll
    /// to restore on exit. All the enter/exit/new logic lives in the appended
    /// sketch-mode impl block at the END of this file.
    sketch_edit: Option<SketchEdit>,
    /// Sketch-mode camera lock. When true (the default on every sketch entry), the
    /// camera is held flat-on to the sketch plane and an empty drag only PANS —
    /// no orbit. Toggling it back on re-faces the camera to the plane. Meaningful
    /// only while `sketch_edit` is `Some`. See the sketch-mode impl block.
    sketch_camera_locked: bool,
    /// Set for ONE frame when the sketch entity-LIST panel hovers a row (it calls
    /// [`sketch_hover_entity`](Self::sketch_hover_entity)). The viewport, which
    /// draws AFTER the panel and would otherwise `sketch_clear_hover` because the
    /// pointer is off the viewport, consumes this flag via
    /// [`take_sketch_list_hover`](Self::take_sketch_list_hover) and keeps the
    /// panel-set hover so list→canvas highlight survives the frame.
    sketch_list_hover_active: bool,
    /// Transient user-facing notices (e.g. a sketch solve that failed after an
    /// edit). The shell drains them each frame via [`take_notices`](Self::take_notices)
    /// into a toast overlay; the engine only queues. Replaces the swallowed
    /// `eprintln!` on the interactive re-solve paths.
    notices: Vec<String>,
    /// Notices that are NOT errors — a warning, or a success such as "Updated 2
    /// part(s)" — with their severity, queued by
    /// [`push_notice_as`](Self::push_notice_as). Beside [`Self::notices`] rather
    /// than in it, so every existing `push_notice` stays an error unchanged.
    graded_notices: Vec<(NoticeSeverity, String)>,
    /// Committed-sketch visibility: the feature ids whose persistent committed-sketch
    /// overlay is HIDDEN (its Scene-tree checkbox off). Absent = visible. See the
    /// committed-sketch impl block appended at the END of this file.
    hidden_sketches: std::collections::HashSet<String>,
    /// The committed-sketch feature ids whose overlay groups were fed on the LAST
    /// [`refresh_committed_sketches`](Self::refresh_committed_sketches), so an id that
    /// is no longer shown (rolled back, deleted, hidden, or became the active edit)
    /// can have its now-stale groups cleared.
    shown_sketch_ids: Vec<String>,
    /// The board bodies fed on the LAST
    /// [`refresh_board_geometry`](Self::refresh_board_geometry), in stack order,
    /// so a body this build did not produce can be swept out of the scene.
    /// Parallels [`shown_sketch_ids`].
    shown_board_ids: Vec<String>,
    /// The eCAD board those bodies were built from — the rebuild test. A
    /// SCHEMATIC edit rewrites the whole `pcb` block with the same board inside
    /// it, and this is what makes that edit free.
    board_built_from: Option<brep_ecad_core::board::Board>,
    /// The pin-to-net map those bodies' NET COLOURS were built from. The board
    /// alone does not decide them — a copper island carries the net of the pads
    /// it touches, and which net a pad carries is the SCHEMATIC's answer — so
    /// this is the second half of the rebuild test. Empty for a board with no
    /// placements, which can carry no net at all.
    board_pin_nets: std::collections::BTreeMap<(brep_ecad_core::Uuid, String), String>,
    /// What that build cost, for the Scene tree and the tests.
    board_build: board_geometry::BoardBuild,
    /// The built bodies themselves, KEPT across runs. The scene reconcile drops
    /// every derived body, so without this a dense board would be re-meshed on
    /// every history run; a clone of these is what goes back in instead.
    board_displays: Vec<crate::scene::SolidDisplay>,
    /// The named plane FRAMES the last history run resolved `(frame name, frame)`.
    /// Stored so [`refresh_construction_datums`](Self::refresh_construction_datums)
    /// (and datum selection) can display/pick the construction datum/plane frames
    /// without re-running. Filled from the run's
    /// [`crate::pipeline::SceneBuildReport::frames`]; the D/P filter is applied at
    /// display time (a frame name maps to its producing feature TYPE via the
    /// history). See the appended construction-datum impl block at the END.
    construction_frames: Vec<(String, brep_kernel::Frame)>,
    /// The solved sketch PROFILES the last history run produced `(sketch id,
    /// profile)`. Stored so [`refresh_committed_sketches`](Self::refresh_committed_sketches)
    /// can synthesize each committed sketch's SHEET SOLID (planar face + named
    /// boundary edges + corner vertices) without re-running. Filled from the run's
    /// [`crate::pipeline::SceneBuildReport::profiles`]; rollback / active-edit /
    /// visibility gating is applied at display time.
    sketch_profiles: Vec<(String, brep_kernel::SketchProfile)>,
    /// The named PATH chains the last history run produced `(path name, curves)`.
    /// Stored alongside [`sketch_profiles`](Self::sketch_profiles) so
    /// [`refresh_committed_sketches`](Self::refresh_committed_sketches) can draw the
    /// sketch geometry no closed profile covers: a sketch's OPEN chain publishes no
    /// profile, so the sheet builder had nothing to draw it from and an open sketch
    /// was invisible in 3D. Filled from the run's
    /// [`crate::pipeline::SceneBuildReport::paths`]; the per-segment `{id}:G{gid}`
    /// entries are the display input (the whole-chain `{id}` entry duplicates them
    /// and `{id}:REF:{source}` is projected reference geometry).
    sketch_paths: Vec<(String, Vec<brep_kernel::NurbsCurve>)>,
    /// The named world POINTS the last history run produced `(point name, point)`.
    /// Stored alongside [`sketch_paths`](Self::sketch_paths) so
    /// [`refresh_committed_sketches`](Self::refresh_committed_sketches) can draw the
    /// sketch points no segment covers: a points-only sketch (a hole-placement
    /// sketch) publishes no profile and no path, so it had nothing to draw from
    /// and was invisible in 3D. Filled from the run's
    /// [`crate::pipeline::SceneBuildReport::points`]; the per-point `{id}:P{pid}`
    /// entries are the display input (construction points are skipped at draw
    /// time, like construction geometry).
    sketch_points: Vec<(String, brep_kernel::ScenePoint)>,
    /// The named axis LINES the last history run produced `(axis name, line)`.
    /// Stored so the feature-dimension angle gizmo can resolve a revolve `axis`
    /// reference to a world line without re-running. Filled from the run's
    /// [`crate::pipeline::SceneBuildReport::axes`].
    sketch_axes: Vec<(String, brep_kernel::Axis)>,
    /// The wire-harness routing report of the last APPLIED run (`None` before
    /// the first run). Filled from [`crate::pipeline::SceneBuildReport::wire_harness`];
    /// the harness panel reads it, the committed-curve display reads the port
    /// kinds off it. See the appended wire-harness impl block.
    wire_harness_report: Option<brep_kernel::WireHarnessReport>,
    /// The PORTS tail's report of the last APPLIED run (`None` before the first
    /// run, and for a document that declares no `ports` block). Filled from
    /// [`crate::pipeline::SceneBuildReport::ports`]; the Qualify panel reads it,
    /// and it is the only place the tail's refused names and unresolved
    /// references surface, the tail not being a feature.
    ports_report: Option<brep_kernel::PortsReport>,
    /// The PMI tail's report of the last APPLIED run (`pmi_ops.rs`).
    pub(crate) pmi_report: Option<brep_kernel::PmiReport>,
    /// The ACTIVE PMI view (engine memory, never persisted): its camera,
    /// display state and explode poses are applied; its annotations drawn.
    pub(crate) pmi_active_view: Option<String>,
    /// The annotation whose form the PMI panel shows (engine memory). Never
    /// assign `Some` here: go through [`EngineState::open_pmi_annotation`], the
    /// one place that counts the open.
    pub(crate) pmi_open_annotation: Option<String>,
    /// The VIEW whose form the PMI panel shows (engine memory) — exclusive
    /// with [`Self::pmi_open_annotation`], since the pane shows one dialog.
    /// Never assign `Some` here: go through [`EngineState::open_pmi_view`].
    pub(crate) pmi_open_view: Option<String>,
    /// The annotation SELECTED in the PMI tree or by its viewport label: the
    /// tree row's highlight and the label's accent. Cleared with the selection
    /// (Esc / Clear / a click on empty space); read through
    /// [`EngineState::pmi_selected_annotation`], which prunes a stale id.
    pub(crate) pmi_selected_annotation: Option<String>,
    /// The VIEW selected in the PMI tree — its row's highlight. Selecting a
    /// view never ACTIVATES it (a double click does); exclusive with
    /// [`Self::pmi_selected_annotation`], since the tree selects one row.
    /// Read through [`EngineState::pmi_selected_view`], which prunes a stale id.
    pub(crate) pmi_selected_view: Option<String>,
    /// How many times a PMI dialog — an annotation's or a view's — has been
    /// OPENED: monotonic, never reset, closes do not count. The app's dialog
    /// door keys on it beside the subject so a re-open of the dialog that is
    /// ALREADY open is still one open event (a viewport label double click on the
    /// annotation whose form is up, which a user makes precisely because they
    /// are on another tab).
    pub(crate) pmi_dialog_opens: u64,
    /// The modeling camera / visibility / wireframe remembered on entering
    /// the PMI workbench, restored when a view deactivates or the workbench
    /// is left.
    pub(crate) pmi_modeling: Option<pmi_ops::PmiModelingSnapshot>,
    /// The un-exploded displays of the solids the active view's explode
    /// annotations posed, for an exact restore.
    pub(crate) pmi_explode_originals: std::collections::HashMap<String, crate::scene::SolidDisplay>,
    /// A balloon whose dragged bubble could not be re-projected yet because
    /// the scene did not hold its occurrence's exact solids: re-derived when
    /// the topology reply lands (`pmi_refresh_stale_balloon`).
    pub(crate) pmi_balloon_stale: Option<String>,
    /// `(world_per_pixel, view direction)` the PMI overlay was last baked at.
    pub(crate) pmi_overlay_key: Option<(f64, [f64; 3])>,
    pub(crate) pmi_hovered: Option<String>,
    pub(crate) pmi_label_hover_active: bool,
    /// The sheet the sheet viewport shows (engine memory, never persisted).
    /// `None` = the 3D view (`sheet_ops.rs`).
    pub(crate) sheet_open: Option<String>,
    /// The sheet or placed view whose form the Sheets pane shows. Never assign
    /// `Some` here: go through [`EngineState::open_sheet_object`], the one
    /// place that counts the open.
    pub(crate) sheet_open_object: Option<String>,
    /// How many times a sheet object's form has been OPENED — the sheet half
    /// of the same counter as [`Self::pmi_dialog_opens`], read by the
    /// app's dialog door so a re-open of the object already open still counts.
    pub(crate) sheet_object_opens: u64,
    /// The placed view, sheet dimension or ordinate set SELECTED in the Sheets
    /// tree or on the paper: the tree row's highlight and the object's accent
    /// on the sheet. Cleared with the selection; read through
    /// [`EngineState::sheet_selected_object`], which prunes a stale id.
    pub(crate) sheet_selected_object: Option<String>,
    /// The last projected sheet, kept until the model or the block moves: a
    /// sheet is a hidden-line pass over the whole model and must never run
    /// per frame.
    pub(crate) sheet_cache: Option<sheet_ops::SheetCache>,
    /// The sheet's exact passes on the runner and waiting for it.
    pub(crate) sheet_lines: sheet_ops::SheetLinesWork,
    /// The open document's name, as the shell means it (the tab, the file, the
    /// root PRODUCT of a STEP export). Engine memory, never persisted: the
    /// document does not carry its own name, the SHELL owns it, and it is here
    /// because a drawing sheet's TITLE BLOCK prints it. Set every frame by the
    /// app from the active tab (a no-op when unchanged).
    pub(crate) document_name: String,
    /// What the spline-anchor cage overlay was last fed for — `(spline id,
    /// selected anchor, applied run generation)` — so the per-frame refresh
    /// the history panel makes while an SP form is open is a no-op until one
    /// of those moves. See `spline_edit.rs`.
    spline_overlay_key: Option<(String, Option<usize>, u64)>,
    /// Whether the ports placed components carry (`ACOMP1:PORT1`) draw their
    /// sheets. The app sets it from the active workbench — one that shows the
    /// Wire Harness panel; a document's own PORT features always draw.
    component_ports_visible: bool,
    /// The spline whose anchor editor is open (fed by the history panel with
    /// the cage overlay), so a viewport click on one of its anchor dots selects
    /// that anchor; the index of the last such pick, until the panel takes it.
    spline_edit_feature: Option<String>,
    spline_anchor_picked: Option<usize>,
    /// Construction datum/plane visibility: the frame NAMES whose datum plane is
    /// HIDDEN (its Scene-tree checkbox off). Absent = visible. Mirrors
    /// [`hidden_sketches`].
    hidden_datums: std::collections::HashSet<String>,
    /// The datum frame NAMES fed to the widget on the LAST
    /// [`refresh_construction_datums`](Self::refresh_construction_datums). The datum
    /// feed REPLACES its set wholesale each call, so this is a bookkeeping mirror of
    /// what is currently shown (parallels [`shown_sketch_ids`]).
    shown_datum_names: Vec<String>,
    /// The history-run seam (M2a of the off-thread runner). Owns the scene-free
    /// runner + its delta baseline (`name → last-emitted handle`) ACROSS reruns:
    /// [`rerun_history`](Self::rerun_history) SUBMITS a run tagged with
    /// [`run_generation`](Self::run_generation), and [`pump`](Self::pump) drains
    /// the completed reply and [`apply_run_output`](Self::apply_run_output)s its
    /// delta to [`Self::scene`]. The [`InlineRunner`](crate::runner::InlineRunner)
    /// default runs synchronously (submit → immediate poll, byte-identical to the
    /// old in-place reconcile); a background thread/worker impl slots in behind the
    /// same trait in M2b/M3. Reset on a document switch
    /// ([`set_history_json`](Self::set_history_json)) so a new model rebuilds fully.
    pub(crate) runner: Box<dyn crate::runner::HistoryRunner>,
    plugins: plugins::Plugins,
    /// Monotonic run counter: bumped each time [`rerun_history`](Self::rerun_history)
    /// SUBMITS a run, and stamped onto the reply so a stale reply (a newer run that
    /// finished first) can be dropped. `run_generation != applied_generation` means
    /// a run is in flight (always equal for the synchronous Inline runner).
    run_generation: u64,
    /// The generation of the last reply [`pump`](Self::pump) APPLIED — the high-water
    /// mark that gates stale replies.
    applied_generation: u64,
    /// The latest [`crate::runner::RunProgress`] of the run in flight (a
    /// background runner posts one before each feature it executes), or
    /// `None` when nothing is running. Names what the spinner is waiting on.
    run_progress: Option<crate::runner::RunProgress>,
    /// The feature the last CANCELLED run was executing (`Some("")` when the
    /// run was cancelled before any progress arrived), cleared by the next
    /// submit. The history header shows it until the model is rebuilt.
    cancelled_run: Option<String>,
    /// A TRANSIENT expressions source the runs evaluate instead of the
    /// document's own (the family table's row preview). Never written into
    /// the history: not saved, not undone, not dirty. See
    /// [`Self::set_expression_preview`].
    expression_preview: Option<ExpressionPreview>,
    /// How many run replies came back from the runner (applied or dropped as
    /// stale). A run the runner coalesced away never replies, so this counts
    /// the runs actually EXECUTED.
    runs_replied: u64,
    /// Displays shipped by replies [`pump`](Self::pump) DROPPED as superseded,
    /// by solid name, latest wins. A background runner executes every run it
    /// is handed and its delta baseline (`name → last-emitted handle`) advances
    /// on each, so a later reply can say "unchanged, keep yours" about a
    /// display that only ever travelled in a dropped reply (the reporter's
    /// arrow-head drag: the frame that built the new radius was superseded by
    /// the still-pointer frames that replayed its handle). The keep is served
    /// from here when the scene's display is not the handle it names. Cleared
    /// with the runner's baseline (document switch, cancel) and whenever a
    /// fresh display for the name is applied.
    superseded_displays: HashMap<String, crate::scene::SolidDisplay>,
    /// A pending one-shot "frame the scene once the in-flight run lands" request.
    /// Import / Open SUBMIT an async run (native [`ThreadRunner`], wasm worker) and
    /// want to `zoom_to_fit` the RESULT — but the scene is still empty when they
    /// return, so an immediate fit frames nothing (bbox empty → no-op). Instead they
    /// set this flag and [`pump`](Self::pump) performs the fit on the first apply that
    /// leaves the run no longer pending. Under the synchronous Inline runner the run
    /// applies inside the submitting call's own `pump`, so the fit is still immediate.
    pending_fit: bool,
    /// EAGER provenance `name → creating-feature id` for the current resident
    /// solids, shipped with each run ([`crate::pipeline::RunOutput::provenance`]) and
    /// replaced wholesale in [`apply_run_output`](Self::apply_run_output). Answers
    /// `creating_feature` + the Info tab's `creatingFeature` WITHOUT a cold
    /// `execute_history` — the freeze side-door once the run lives off-thread.
    pub(crate) provenance: std::collections::HashMap<String, String>,
    /// EAGER ENTITY ORIGIN `face/edge NAME → ORIGINATING feature id` (FIRST writer in
    /// timeline order), shipped with each run
    /// ([`crate::pipeline::RunOutput::entity_origin`]) and replaced wholesale in
    /// [`apply_run_output`](Self::apply_run_output). Unlike `provenance` (the SOLID's
    /// LAST writer) this is the feature that gave the face/edge its NAME — the answer
    /// `creating_feature` returns for a face/edge (the "Edit owning feature" action +
    /// the Info tab's `creatingFeature`), so it rolls to the entity's origin, not the
    /// owning solid's last producer.
    pub(crate) entity_origin: std::collections::HashMap<String, String>,
    /// Object-info MEASUREMENT cache `name → merged object-info JSON`, filled by
    /// [`pump_queries`](crate::metadata) from the runner's replies and served every
    /// frame a selection persists (so `object_info_json` fires ONE query per
    /// selection, not one per frame). Invalidated on any geometry change
    /// (`apply_run_output`) or metadata edit (`set_metadata_attribute`).
    pub(crate) info_cache: std::collections::HashMap<String, String>,
    /// In-flight measurement queries `id → object NAME` — the key needed to MERGE a
    /// reply back into the info cache (inject `name` + `creatingFeature`, the latter
    /// now resolved by the entity name itself). Cleared alongside `info_cache` on a
    /// geometry change so a stale reply is dropped rather than caching a superseded
    /// measurement.
    pub(crate) pending_query: std::collections::HashMap<u64, String>,
    /// Monotonic measurement-query id (pairs a [`crate::runner::MeasureReply`] back
    /// with its `pending_query` entry).
    pub(crate) next_query_id: u64,
    /// Off-thread mesh reconstructions awaiting a runner reply.
    pub(crate) pending_mesh_imports: std::collections::HashMap<u64, MeshImportDestination>,
    pub(crate) mesh_preview_results: std::collections::VecDeque<crate::runner::MeshImportReply>,
    /// Monotonic id for pairing mesh import replies with submissions.
    pub(crate) next_mesh_import_id: u64,
    /// Topology requests in flight (see [`crate::runner::TopologyRequest`]).
    pub(crate) pending_topology: std::collections::HashSet<u64>,
    /// Every resident handle asked for since the last applied run, answered or
    /// not: a handle the runner could not clone is not asked for again until a
    /// run brings new handles.
    pub(crate) topology_asked: std::collections::HashSet<u32>,
    /// Monotonic id for pairing topology replies with requests.
    pub(crate) next_topology_id: u64,
    /// The run's ASSEMBLY tail as the runner read it
    /// ([`crate::pipeline::AssemblySync`]) — the constraint state, statuses, DOF
    /// and overlay rows every assembly panel reads, adopted from the reply of the
    /// last applied run. `None` for a componentless document. This is what
    /// replaced the second, main-side `execute_history`: see `assembly_ops`.
    pub(crate) assembly_sync: Option<crate::pipeline::AssemblySync>,
    /// The scene's assembly COMPONENT records (deterministic id order), adopted
    /// from [`Self::assembly_sync`] — the Assembly Structure tree's source.
    /// Empty for componentless documents.
    pub(crate) assembly_components: Vec<crate::pipeline::ComponentSnapshot>,
    /// The [`Self::applied_generation`] the main-side kernel assembly SESSION was
    /// last installed for (`None` = never). Only a constraint MUTATION needs one
    /// — its solve runs against the resident geometry of the thread that asks —
    /// so only `ensure_assembly_session` sets this. See `assembly_ops`.
    pub(crate) assembly_session_generation: Option<u64>,
    /// Every output name the last applied run CONSUMED
    /// ([`crate::pipeline::RunOutput::consumed`]) — what the committed-sketch
    /// refresh hides a consumed sketch by, and what it used to re-execute the
    /// whole history on the UI thread to learn.
    pub(crate) consumed_names: std::collections::HashSet<String>,
    /// Re-entrancy guard for the parts-library resync in [`Self::pump`] (a
    /// refused run re-runs, and `rerun_history` pumps).
    pub(crate) library_resync: bool,
    /// The assembly-constraint viewport overlays — the cached
    /// [`crate::constraint_overlays::ConstraintOverlay`] records the engine last
    /// built from the kernel session (`assembly_overlay_json` + state), the source
    /// of the drawn leader/arrow group, the label feed, and the grabbable-handle
    /// hit regions. Refreshed after every history apply and every constraint
    /// mutation; see the appended assembly-overlay impl block.
    pub(crate) constraint_overlays: Vec<crate::constraint_overlays::ConstraintOverlay>,
    /// A live GRAB on a constraint's distance-arrow / angle-arc handle (`Some`
    /// between `constraint_drag_begin` and `constraint_drag_release`): the drag
    /// previews the value locally and COMMITS on release via
    /// `assembly_update_constraint_json` (which auto-solves).
    pub(crate) constraint_drag: Option<ConstraintDrag>,
    /// The `world_per_pixel` the constraint overlay buffers were last baked at
    /// (screen-constant arc/rod sizing): `ensure_constraint_overlay_current`
    /// re-bakes when the camera zoom moves it materially. `0.0` = never baked.
    constraint_overlay_wpp: f64,
    /// The `world_per_pixel` the FEATURE-DIMENSION gizmo group
    /// (`feature-dim-leaders`) was last baked at. The leaders' rod/cone/origin
    /// sphere — and the angular arc's whole world RADIUS — are
    /// `pixels × world_per_pixel`, so a zoom makes the drawn gizmo stale;
    /// `ensure_feature_dimension_overlay_current` re-bakes on a material move.
    /// `0.0` = nothing baked (gizmo disarmed / not in dimension mode).
    /// See `feature_dims`.
    pub(crate) feature_dim_overlay_wpp: f64,
    /// The `world_per_pixel` the live SKETCH overlay groups (geometry, points,
    /// preview, `sketch-dim-leaders`, `sketch-constraint-glyphs`) were last baked
    /// at — construction dashes, dimension arrowheads and constraint glyphs are
    /// all screen-constant. `ensure_sketch_overlay_current` re-bakes on a material
    /// zoom. `0.0` = nothing baked (not in sketch mode). See `sketch_input`.
    pub(crate) sketch_overlay_wpp: f64,
    /// The constraint id whose referenced ELEMENTS the panel/label hover is
    /// currently highlighting (dedupe key so a held hover doesn't re-bump the
    /// emphasis generation every frame).
    constraint_hovered: Option<String>,
    /// Set for ONE frame when a constraint LABEL hover applied the element
    /// highlight; the viewport's modeling hover branch consumes it (mirrors
    /// [`Self::take_sketch_list_hover`]) so the scene hover doesn't clobber it.
    constraint_label_hover_active: bool,
    /// The SCENE-TREE row the pointer is over, as the pick candidate its hover
    /// lit (the row→viewport highlight). Kept so
    /// [`scene_tree_hover_end`](Self::scene_tree_hover_end) clears ONLY a hover
    /// the tree itself set — never one the viewport lit in the meantime — and as
    /// the dedupe key that keeps a held row hover from re-bumping the emphasis
    /// generation every frame. See `scene_query.rs`.
    scene_tree_hovered: Option<pick::PickCandidate>,
    /// Set for ONE frame while a Scene-tree row is hovered; the viewport's
    /// modeling hover branch consumes it via
    /// [`take_scene_tree_hover`](Self::take_scene_tree_hover) and skips the
    /// pointer-off-viewport `clear_hover` (mirrors
    /// [`Self::take_sketch_list_hover`]) so the row-set highlight survives.
    scene_tree_hover_active: bool,
    /// The DIALOG row the pointer is over — a feature form's reference line, its
    /// read-only `Outputs` line, or the reference picker card's picked-name line
    /// (the row→viewport highlight, the twin of the Scene tree's above). Its OWN
    /// slot, not the tree's: History, Constraints, PMI and the Scene tree can all
    /// be on screen at once in a split dock, and a shared slot would let the pane
    /// that draws second end the hover the pane that drew first had just set. See
    /// [`DialogHover`] for what it memoizes and `scene_query.rs` for the API.
    dialog_hovered: Option<DialogHover>,
    /// Set for ONE frame while a dialog row is hovered; the viewport's modeling
    /// hover branch consumes it via [`take_dialog_hover`](Self::take_dialog_hover)
    /// and skips the pointer-off-viewport `clear_hover` (mirrors
    /// [`Self::take_scene_tree_hover`]) so the row-set highlight survives.
    dialog_hover_active: bool,
    /// The constraint SELECTED via its viewport label chip (`Some` = selected):
    /// drives the chip's selected accent, the context bar's Delete-constraint
    /// action, and clears with the selection (Esc / Clear / delete). Read
    /// through [`Self::selected_constraint`], which prunes a stale id.
    selected_constraint: Option<String>,
    /// The feature SELECTED in the history tree, beside the geometry selection
    /// that selecting it made. It stays selected only while that geometry
    /// selection is unchanged — a viewport pick, Esc or Clear replaces the
    /// geometry and so deselects the feature. Read through
    /// [`Self::selected_feature`].
    selected_feature: Option<(String, selection_ux::FeatureSelection)>,
    /// How many times a constraint's dialog has been OPENED through
    /// [`Self::assembly_set_constraint_open`] — the constraint half of the same
    /// counter as [`Self::pmi_dialog_opens`], read by the app's dialog door
    /// so a re-open of the constraint already open still counts. The flag
    /// itself lives in the kernel (it is part of the saved document), so this
    /// counts the OPENS this engine performed, not the flag's history.
    pub(crate) constraint_opens: u64,
    /// The STEP product structure a [`Self::probe_step_assembly`] parsed, held
    /// until the import dialog's button decides its fate: consumed by
    /// [`Self::import_probed_step_assembly`], dropped by
    /// [`Self::discard_probed_step_assembly`] (Cancel), by the next probe, or by
    /// a document switch. The ONE parse of a structured STEP import lives here —
    /// the dialog needs the counts BEFORE the user chooses, and re-parsing
    /// multi-MB Part-21 text on the way back would double the most expensive step
    /// of the import. See `model_io`'s import block.
    pub(crate) pending_step_assembly: Option<brep_kernel::StepAssembly>,
    /// STEP probes submitted to the runner and not yet answered (their ids).
    pub(crate) pending_step_probes: std::collections::HashSet<u64>,
    /// Answered probes awaiting [`Self::take_step_probe`], oldest first.
    pub(crate) step_probe_results: std::collections::VecDeque<(u64, StepProbeOutcome)>,
    /// Monotonic id pairing a probe reply with its submission.
    pub(crate) next_step_probe_id: u64,
}

/// What a STEP probe found, as the file panel consumes it (see
/// [`EngineState::submit_step_probe`]).
#[derive(Debug, Clone, PartialEq)]
pub enum StepProbeOutcome {
    /// Product structure: the parsed assembly is stashed for
    /// [`EngineState::import_probed_step_assembly`]; these are its counts.
    Structure(model_io::StepAssemblyProbe),
    /// No structure worth keeping — take the flat lane.
    Flat,
    /// The text did not parse as Part 21; the flat lane refuses it with its
    /// own wording.
    Failed(String),
}

/// A live constraint-handle drag: which constraint + which `inputParams` field
/// (`distance` / `angle`), the params snapshot the commit mutates, and the live
/// preview value the overlay/label show while dragging.
#[derive(Debug, Clone)]
pub struct ConstraintDrag {
    pub id: String,
    pub field: &'static str,
    pub params: serde_json::Value,
    pub preview: f64,
}

/// WHO a finished reference-selection commits to: a history FEATURE's params
/// (the original widget) or an ASSEMBLY CONSTRAINT's `inputParams` (same modal,
/// different commit lane — see `assembly_ops::begin_ref_select_for_constraint`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum RefSelectTarget {
    #[default]
    Feature,
    AssemblyConstraint,
    /// ONE SPLINE ANCHOR's port attachment: `path` is unused, the picked name
    /// must be a PORT feature id (a click on a port's drawn sheet), and Finish
    /// writes `persistentData.spline.points[index].attachment` (see
    /// `spline_edit.rs`).
    SplineAnchor { index: usize },
    /// A PMI annotation's reference field: `feature_id` is the annotation id;
    /// Finish writes the picked names into its params through the PMI block
    /// (`pmi_ops.rs`). Vertex picks are `{solid}@x,y,z` in WORLD coordinates.
    Pmi,
    /// ONE CONNECTION POINT's reference field: `feature_id` is the point's
    /// part-local ADDRESS (`J1.VCC`) — a connection point is data, not a
    /// feature, so there is no id to carry — and `path` is the block key the
    /// pick writes (`pointRef` / `directionRef`). Finish writes it through
    /// `ports_ops::ports_commit_ref`, and NOTHING ROLLS: a point resolves at
    /// the tail of the run, so the geometry it may name is what is on screen.
    PortPoint { field: ports_ops::PortRefField },
    /// A drawing-sheet object's reference field — a dimension's anchors, an
    /// ordinate set's datum or members, a section's cutting line, a detail's
    /// centre or rim: `feature_id` is the object id, the picks are ANCHORS
    /// clicked on the open sheet's paper (`sheet_ops.rs`), and nothing rolls.
    Sheet,
}

/// The modal state of the reference-selection widget while it is ACTIVE: which
/// feature-dialog field is being filled, what it accepts, and the running list
/// of picked kernel names (the source of truth — added by picking in the view,
/// removed via the per-line X). See the ref-select methods below.
#[derive(Debug, Clone, Default)]
pub struct RefSelectState {
    /// The id of the feature whose param is being edited.
    pub feature_id: String,
    /// The JSON path into that feature's `inputParams` the names write to
    /// (`["targetSolid"]`, `["boolean","targets"]`, `["faceRef"]`, …).
    pub path: Vec<String>,
    /// A human label for the modal heading (the field's label).
    pub label: String,
    /// The allowed pick kinds (`["SOLID"]`, `["FACE"]`, …) — the type constraint.
    pub filter: Vec<String>,
    /// Whether the field takes a LIST of references (else a single one).
    pub multiple: bool,
    /// The picked kernel names — the running selection (source of truth).
    pub names: Vec<String>,
    /// The rollback step to restore on Finish/Cancel (the edited feature's own
    /// step): entering the mode rolls to the pre-feature "before" state, so we
    /// remember where to return.
    pub restore_index: usize,
    /// Which surface Finish commits the names to (feature params vs an
    /// assembly constraint's `inputParams`). Defaults to [`RefSelectTarget::Feature`].
    pub target: RefSelectTarget,
}

impl Default for EngineState {
    fn default() -> Self {
        Self {
            scene: RenderScene::new(),
            camera: ViewCamera::default(),
            controls: ArcballControls::new(),
            settings: RenderSettings::default(),
            emphasis: Emphasis::default(),
            widgets: WidgetRegistry::new(),
            history: History::default(),
            history_report: String::new(),
            metadata: crate::metadata::MetadataStore::new(),
            document_thumbnail: std::cell::RefCell::new(None),
            thumbnail_cache: std::cell::RefCell::new(None),
            settings_generation: 1,
            dirty: true,
            ref_select: None,
            selection_filter: SelectionFilter::default(),
            transform_gizmo: TransformArm::default(),
            component_move: ComponentMoveArm::default(),
            sketch_edit: None,
            sketch_camera_locked: true,
            sketch_list_hover_active: false,
            notices: Vec::new(),
            graded_notices: Vec::new(),
            hidden_sketches: std::collections::HashSet::new(),
            shown_sketch_ids: Vec::new(),
            shown_board_ids: Vec::new(),
            board_built_from: None,
            board_pin_nets: Default::default(),
            board_build: board_geometry::BoardBuild::default(),
            board_displays: Vec::new(),
            construction_frames: Vec::new(),
            sketch_profiles: Vec::new(),
            sketch_paths: Vec::new(),
            sketch_points: Vec::new(),
            sketch_axes: Vec::new(),
            wire_harness_report: None,
            ports_report: None,
            sheet_open: None,
            sheet_open_object: None,
            sheet_object_opens: 0,
            sheet_selected_object: None,
            sheet_cache: None,
            sheet_lines: Default::default(),
            document_name: String::new(),
            pmi_report: None,
            pmi_active_view: None,
            pmi_open_annotation: None,
            pmi_open_view: None,
            pmi_selected_annotation: None,
            pmi_selected_view: None,
            pmi_dialog_opens: 0,
            pmi_modeling: None,
            pmi_explode_originals: std::collections::HashMap::new(),
            pmi_balloon_stale: None,
            pmi_overlay_key: None,
            pmi_hovered: None,
            pmi_label_hover_active: false,
            spline_overlay_key: None,
            component_ports_visible: false,
            spline_edit_feature: None,
            spline_anchor_picked: None,
            hidden_datums: std::collections::HashSet::new(),
            shown_datum_names: Vec::new(),
            runner: Box::new(crate::runner::InlineRunner::new()),
            plugins: plugins::Plugins::default(),
            interference_sessions: Default::default(),
            interference_sequence: 0,
            illustration_snapshot: None,
            geometry_diagnostics: Default::default(),
            run_generation: 0,
            applied_generation: 0,
            run_progress: None,
            cancelled_run: None,
            expression_preview: None,
            runs_replied: 0,
            superseded_displays: HashMap::new(),
            pending_fit: false,
            provenance: std::collections::HashMap::new(),
            entity_origin: std::collections::HashMap::new(),
            info_cache: std::collections::HashMap::new(),
            pending_query: std::collections::HashMap::new(),
            next_query_id: 0,
            pending_mesh_imports: std::collections::HashMap::new(),
            mesh_preview_results: std::collections::VecDeque::new(),
            next_mesh_import_id: 0,
            pending_topology: std::collections::HashSet::new(),
            topology_asked: std::collections::HashSet::new(),
            next_topology_id: 0,
            assembly_components: Vec::new(),
            assembly_sync: None,
            assembly_session_generation: None,
            consumed_names: std::collections::HashSet::new(),
            library_resync: false,
            constraint_overlays: Vec::new(),
            constraint_drag: None,
            constraint_overlay_wpp: 0.0,
            feature_dim_overlay_wpp: 0.0,
            sketch_overlay_wpp: 0.0,
            constraint_hovered: None,
            constraint_label_hover_active: false,
            scene_tree_hovered: None,
            scene_tree_hover_active: false,
            dialog_hovered: None,
            dialog_hover_active: false,
            selected_constraint: None,
            selected_feature: None,
            constraint_opens: 0,
            pending_step_assembly: None,
            pending_step_probes: std::collections::HashSet::new(),
            step_probe_results: std::collections::VecDeque::new(),
            next_step_probe_id: 1,
        }
    }
}

impl EngineState {
    pub fn new() -> Self {
        Self::default()
    }

    fn pick_options(&self) -> PickOptions {
        PickOptions {
            double_sided: self.settings.pick_double_sided,
            ..PickOptions::default()
        }
    }
}

/// Has the camera's `world_per_pixel` moved MATERIALLY (>0.5%) away from the
/// value an overlay group's screen-constant sizing was baked at?
///
/// The ONE judgement every camera-keyed overlay re-bake shares
/// (`ensure_overlays_current` and the per-group ensures it drives), so
/// "what counts as a zoom" cannot drift between them. `baked == 0.0` means
/// "never baked" — a state each caller handles itself, so it reads as NOT
/// stale here. The band is what keeps a quiet frame quiet: float jitter in the
/// camera never trips it, so there is no per-frame re-bake loop.
pub(crate) fn overlay_wpp_stale(baked: f64, now: f64) -> bool {
    baked > 0.0 && (now - baked).abs() > baked * 0.005
}

// ============================================================================
// MODULE MAP — engine_state is split into topic children. THIS file is the
// module root: it keeps the EngineState/RefSelectState structs, Default, the
// constructor + shared pick_options, the child `mod` declarations, and the
// re-exports that preserve the original `engine_state::*` public paths.
// Append new work to the matching child (or add a new child + re-export here).
// ============================================================================

/// The assembly surface (Wave-3): main-side session sync + component
/// projection, constraint CRUD with the document fold (pose-authority
/// contract), the insert-component flow, component fix/select actions, the
/// constraint flavor of the reference picker, and `document_signature` — the
/// ONE parts-library `sourceSignature` hash every writer shares.
mod assembly_ops;
/// Assembly-constraint viewport overlays: overlay refresh from the kernel
/// session, the grabbable distance/angle handle pick + drag preview +
/// `assembly_update_constraint_json` commit, movedSolids re-tessellation, the
/// drag-path document fold, label feed + element hover, and tests.
mod assembly_overlay;
/// BOM export: the `{partName, sourceKey, quantity}` parts list off the
/// main-side parts library + live component projection, serialized as CSV /
/// JSON for the file dialog's Export modal.
mod bom;
/// Camera & view commands (zoom-to-fit, resize, pointer/wheel ingestion,
/// projection, standard views, camera state/matrices, world→screen) plus the
/// widget-overlay feeds (datums/overlay/dimensions/transform JSON), the
/// ViewCube (incl. `apply_look_direction`), `datum_pick`, the widget
/// transform-handle hover/pick/drag path, and `dimension_anchors_json`.
mod camera_widgets;
/// Committed-sketch persistent overlays: sheet-solid synthesis from stored
/// profiles (`refresh_committed_sketches`), per-sketch visibility, the
/// Scene-tree rows + `sketch_entities_json`, and the committed-sketch tests.
/// The board's 3D bodies (substrate + copper + via barrels), derived from the
/// document's `pcb` block after every run — see the module notes.
mod board_geometry;
mod committed_sketches;
/// The COMPONENT Move gizmo (assemblies §8.5): translate→rotate→off cycle at
/// the member-bbox center, fixed-refusal, free-move drag with the pose commit
/// (and re-solve) on release. See `ComponentMoveArm`.
mod component_move;
/// Assembly COMPONENT read surface: the app-side component projection derived
/// from the history's ACOMP features + the scene's namespaced solid names
/// (`component_of_solid` / `component_info` / `component_bbox_center`), and its
/// tests + shared assembly fixtures.
mod components;
/// Construction datum/plane display: frame→feature mapping,
/// `refresh_construction_datums`, datum visibility + selection + entity rows,
/// and the construction-datum tests.
mod construction_datums;
/// Expressions & configurator surface (`expressions_json`, `set_expressions`,
/// `configurator_json`, `expression_variables_json`) + parsing helpers + tests.
mod expressions;
pub use expressions::ExpressionPreview;
/// The feature-dimension gizmo (dimension arrows / angle arc / center-sphere
/// toggle): arm state, `__brep`-style annotation JSON, overlay publishing,
/// drag + set-value writeback, `FEATURE_DIM_OVERLAY`, fd_* math helpers, tests.
mod feature_dims;
/// The assembly interference check: the bbox-prefiltered pairwise
/// non-destructive INTERSECT sweep over component instances
/// (`interference_check`), its report types, and the pure pair planner.
mod interference;
mod batch;
mod plugins;
mod illustration;
/// History runs & the feature CRUD surface: `run_history_json`, the
/// runner/pump/apply seam, history JSON accessors, roll/update/add/delete/
/// reorder, engine undo/redo, `load_model_and_fit`,
/// and the history-cache rollback tests.
mod history_ops;
/// Model I/O: `import_step_feature`, `export_step_text`, `export_stl_text`,
/// `triangle_normal`, and the io tests.
mod model_io;
/// Construction PLANES as ordinary pick candidates: the combined
/// scene+plane-card candidate list (`pick_candidates_at`), the planes-aware
/// single-hit pick (`pick_top_at`), the shared candidate ordering, and the
/// plane-pick tests. Read its header for how a plane competes for a pick.
mod plane_pick;
/// Scene & object queries: `pick_json`/`hover_json`, settings + emphasis +
/// visibility + color overrides, `scene_listing_json`, `depth_range_bbox`,
/// `scene_entities_json`/select-by-name, mass properties + object info, tests.
mod scene_query;
/// The selection-filter state + kind gating (`SelectionFilter`,
/// `select_filtered_at`, hide-selected) and its tests.
mod selection_filter;
/// Selection & hover UX: clear/select-top/selection JSON, the modal
/// ref-select widget (incl. `set_json_at`), hover/candidate cycling, and the
/// live sketch overlay feed (`set_sketch_overlay`/`clear_sketch_overlay`) +
/// selection UX tests.
mod selection_ux;
/// Sketch editing ops appended after dimensions: dimension labels/value/drag,
/// sketch undo/redo + diagnostics dump, trim, external edge refs
/// (pick/link/reproject), and hand-draw strokes.
mod sketch_edit_ops;
/// Sketch-mode viewport input: overlay refresh, uv picking, hover/click/drag,
/// the draw tools + pending-geometry helpers, and delete-selection.
mod sketch_input;
/// Sketch mode session: `SketchDrag`/`SketchEdit` state, plane-frame helpers,
/// enter/exit/new sketch, and the camera lock.
mod sketch_mode;
/// Sketch entity-list panel rows + notices + solver settings
/// (`SketchEntityRow`), and the constraint palette/actions
/// (`SketchConstraintAction`, add-constraint builders, ground/construction
/// toggles, cleanup).
mod sketch_panel;
/// The transform-controls gizmo: `GizmoMode`/`TransformArm`, arm/drag/apply,
/// pose ⇄ params JSON, quaternion helpers (`rotate_euler_xyz_f64`), tests.
mod transform_gizmo;
/// The wire-harness surface: the document's `wireHarness` block (connection
/// add / edit / remove / bundles toggle — checkpointed document edits that
/// re-run the history, whose tail routes them), the applied run's routing
/// report, the endpoint list, and the panel's hover highlight.
mod wire_harness_ops;
/// The PORTS surface: the ports tail's report of the last applied run — the
/// one place the tail's resolved points, refused names and unresolved
/// references reach a caller, the tail not being a feature.
mod ports_ops;
pub use ports_ops::PortRefField;
// Every harness wire as a BOM line with its cut length (MF QTY), or the reason
// the last applied run cannot give one.
mod wire_bom;
mod pmi_ops;
mod pmi_section;
pub use pmi_ops::{pmi_view_params, pmi_view_schema, world_vertex_ref, PmiModelingSnapshot, PmiViewPatch};
// Drawing sheets: the `sheets` block, the open sheet and the projection cache.
mod sheet_ops;
pub(crate) mod pmi_overlay;
/// The 3D spline-anchor editor: the resolved anchor list, the value edits
/// (position / distances / flip / side / add / remove / reorder / detach),
/// the anchor flavour of the transform gizmo, the port-attach flavour of the
/// reference picker, and the direction-cage overlay.
mod spline_edit;

// Re-exports preserving the original `engine_state::*` public surface.
pub use assembly_ops::{document_signature, ComponentInsert};
pub use board_geometry::{
    default_color as board_default_color, BoardBuild, BOARD_SOLID_PREFIX, SUBSTRATE_SOLID,
    VIAS_SOLID,
};
pub use bom::PART_ATTRIBUTES;
pub use component_move::ComponentMoveArm;
pub use components::ComponentInfo;
pub use interference::{InterferencePair, InterferenceReport};
pub use model_io::{
    EmbeddedOnly, PartSink, StepAssemblyImport, StepAssemblyProbe, StepAssemblyReport,
};
pub use selection_filter::SelectionFilter;
pub use sketch_mode::{SketchDrag, SketchEdit};
pub use sketch_panel::{NoticeSeverity, SketchConstraintAction, SketchEntityRow};
pub use wire_harness_ops::ConnectionPatch;
pub use wire_bom::{WireBomLine, WireLengthState};
pub use spline_edit::SplineAnchorRow;
pub use transform_gizmo::{GizmoMode, TransformArm};
pub(crate) use transform_gizmo::rotate_euler_xyz_f64;




/// Reconstruction results either append to a document or remain uncommitted.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum MeshImportDestination {
    Document,
    Preview,
}
