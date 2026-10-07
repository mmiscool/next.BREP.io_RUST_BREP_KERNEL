//! The history-run seam — the async RUN machine behind a trait, mirroring the
//! platform-seam shape of `brep-app/src/store.rs`'s `ModelStore` (a trait with
//! impls behind it, async surfaced via a poll idiom).
//!
//! A history run is split SUBMIT → POLL/APPLY: [`HistoryRunner::submit_run`]
//! kicks off a run tagged with a monotonic generation, and the completed
//! [`RunReply`] is drained later via [`HistoryRunner::poll_run`]. The runner OWNS
//! the [`SceneRunner`](crate::pipeline::SceneRunner) — so a future thread/worker
//! impl owns the resident registry that `execute_history` populates — and holds
//! the delta baseline across reruns.
//!
//! This slice ships the DEFAULT [`InlineRunner`]: it runs on `submit_run` and
//! stashes the reply for an immediate `poll_run`, so the run stays synchronous
//! and byte-identical to the pre-seam in-process run. A native-thread impl (M2b)
//! and a wasm-worker impl (M3) slot in behind the SAME trait — `submit_run`
//! defers the work and `poll_run` surfaces it a frame (or many) later, so the
//! `EngineState::pump` caller never changes.

/// A completed history run, tagged with the [`generation`](Self::generation) it
/// was submitted under so the applier can drop stale replies (a newer run that
/// finished first). The [`output`](Self::output) is the [`SceneRunner`] delta to
/// apply to the display scene.
///
/// [`SceneRunner`]: crate::pipeline::SceneRunner
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct RunReply {
    /// The monotonic generation this run was submitted under.
    pub generation: u64,
    /// The delta snapshot + report the run produced.
    pub output: crate::pipeline::RunOutput,
}

/// A feature about to EXECUTE inside an in-flight run (cached replays are
/// instant and are not reported): which one, of how many, under which
/// generation. Posted by the runner before the kernel starts the feature, so
/// the UI can name what it is waiting on — and, after a cancel, what it was
/// waiting on.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct RunProgress {
    /// The generation of the run this belongs to (see [`RunReply::generation`]).
    pub generation: u64,
    /// Zero-based position of the feature in the request.
    pub index: usize,
    /// The request's feature count.
    pub total: usize,
    pub feature_id: String,
    pub feature_type: String,
}

/// A STEP text to PROBE for product structure on the runner — the parse that
/// `brep_kernel::read_step_assembly` performs, which builds every product's
/// bodies and takes seconds on a real assembly (8.3 s natively for a 5 MB,
/// 140-product file), so it must not run on the browser's main thread.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StepProbeRequest {
    pub id: u64,
    pub text: String,
}

/// The probe's answer: the parsed assembly (`Some`), no structure (`None`),
/// or a parse failure. The assembly crosses back to the main side whole — its
/// JSON is large (74 MB for the file above) but the trip costs a third of a
/// second against the seconds the parse took.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StepProbeReply {
    pub id: u64,
    pub result: Result<Option<brep_kernel::StepAssembly>, String>,
}

/// Which exact measurement a [`MeasureQuery`] wants, mirroring the object-info
/// kinds ([`crate::metadata`]): a whole solid, one named face, or one named edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum MeasureKind {
    Solid,
    Face,
    Edge,
}

/// A per-object MEASUREMENT request routed to the runner (which owns the warm
/// registry). The runner resolves `owner` to its resident handle and measures by
/// [`kind`](Self::kind); the reply is the object-info JSON fragment MINUS the
/// main-injected `name`/`creatingFeature` fields. Tagged with a monotonic
/// [`id`](Self::id) so the main side can pair the reply with its pending request.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MeasureQuery {
    /// Monotonic request id (main pairs the reply back by it).
    pub id: u64,
    /// The measurement to run.
    pub kind: MeasureKind,
    /// The OWNING solid name (a solid is its own owner; a face/edge names its solid).
    pub owner: String,
    /// The face/edge NAME to measure (ignored for a whole-solid query).
    pub entity: String,
    /// The density (mass per mm³) to scale a solid's weight (ignored for face/edge).
    pub density: f64,
}

/// A completed [`MeasureQuery`]: the object-info measurement fields as a JSON
/// fragment (WITHOUT `name`/`creatingFeature`, which the main thread injects from
/// its eager provenance), tagged with the request [`id`](Self::id).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MeasureReply {
    /// The request id this answers.
    pub id: u64,
    /// The measurement JSON fragment (see [`measure_json`]).
    pub result: String,
}

/// A request for the B-rep TOPOLOGY of resident solids, by the handle each
/// display was tessellated from. The drawing sheet's exact hidden-line pass
/// reads faces and edges, and the resident registry that holds them is
/// thread-local to the runner, so the main side asks for exactly the solids a
/// placement draws and does not already hold. Nothing else ever asks, so a
/// document with no sheet never pays for the topology crossing.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TopologyRequest {
    /// Monotonic request id (main pairs the reply back by it).
    pub id: u64,
    /// `(solid name, resident handle)` per solid wanted.
    pub solids: Vec<(String, u32)>,
}

/// A completed [`TopologyRequest`]: a clone of every requested solid the
/// registry still holds, `(name, handle, solid)`. A handle the registry no
/// longer holds (a later run freed it) is left out.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct TopologyReply {
    /// The request id this answers.
    pub id: u64,
    pub solids: Vec<(String, u32, brep_kernel::BrepSolid)>,
}

/// Answer a topology request against the resident registry of the thread this
/// runs on. Shared by every runner.
fn topology_reply(request: TopologyRequest) -> TopologyReply {
    let solids = request
        .solids
        .into_iter()
        .filter_map(|(name, handle)| {
            brep_kernel::registered_solid_clone(handle)
                .ok()
                .map(|solid| (name, handle, solid))
        })
        .collect();
    TopologyReply { id: request.id, solids }
}

/// A drawing sheet's exact hidden-line pass for ONE placement, handed to the
/// runner so the UI thread never runs it (see
/// [`crate::sheets::project::LinesJob`]). The job names the runner's own
/// resident solids by handle and content fingerprint, so nothing crosses the
/// seam but the question and the lines.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SheetLinesRequest {
    /// Monotonic request id (main pairs the reply back by it).
    pub id: u64,
    pub job: crate::sheets::project::LinesJob,
}

/// A completed [`SheetLinesRequest`]: the placement's lines relative to its
/// position, or why the runner could not draw them (a handle its registry no
/// longer holds, or holds different geometry under).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SheetLinesReply {
    /// The request id this answers.
    pub id: u64,
    pub key: String,
    pub lines: Result<crate::sheets::project::ExactLines, String>,
}

/// Answer a sheet-lines request against the resident registry of the thread
/// this runs on. Shared by every runner.
fn sheet_lines_reply(request: SheetLinesRequest) -> SheetLinesReply {
    SheetLinesReply {
        id: request.id,
        lines: crate::sheets::project::run_lines_job(&request.job),
        key: request.job.key,
    }
}

pub use brep_reconstruction::stl_conversion::{
    ConversionPolicy, StlConversionOptions, StlConversionOutput,
};

/// Mesh encoding accepted by the off-thread reconstruction channel.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub enum MeshImportFormat {
    Stl,
    Obj,
    /// 3MF (core specification): a package whose build instances are flattened
    /// into one millimetre triangle soup before this common chain runs.
    ThreeMf,
}

/// A mesh reconstruction request. The byte payload moves to the native thread
/// or is serialized to the browser worker; no parsing or fitting happens on UI.
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct MeshImportRequest {
    pub id: u64,
    pub format: MeshImportFormat,
    pub bytes: Vec<u8>,
    pub options: StlConversionOptions,
}

/// Completed off-thread reconstruction. Success carries validated STEP and
/// diagnostics for a preview or an ordinary IMPORT3D history insertion.
#[derive(Debug, serde::Serialize, serde::Deserialize)]
pub struct MeshImportReply {
    pub id: u64,
    pub result: Result<StlConversionOutput, String>,
}

/// Validate explicitly installed packages away from the UI thread.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PluginInstallRequest {
    pub id: u64,
    pub packages: serde_json::Value,
}
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PluginInstallReply {
    pub id: u64,
    pub result: Result<brep_plugins::Registry, String>,
}
fn validate_plugins(request: PluginInstallRequest) -> PluginInstallReply {
    let result = serde_json::from_value(request.packages)
        .map_err(|e| format!("plugin packages: {e}"))
        .and_then(brep_plugins::Runtime::new)
        .map(|runtime| runtime.registry().clone());
    PluginInstallReply { id: request.id, result }
}

/// A callback request against an immutable document/selection snapshot.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PluginActionRequest {
    pub id: u64,
    pub pins: Vec<brep_plugins::PluginPin>,
    pub action: String,
    pub input: serde_json::Value,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub script: Option<String>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PluginActionReply {
    pub id: u64,
    pub result: Result<brep_plugins::ActionPlan, String>,
}

/// The history-run seam: SUBMIT a run, POLL for its completed reply, RESET the
/// delta baseline, plus a QUERY channel for per-object measurements (routed to the
/// runner so the warm registry answers them, never the cold main-side one). The
/// Inline impl runs everything synchronously; a later thread/worker impl defers
/// the work and surfaces the replies through the same poll idiom.
pub trait HistoryRunner {
    fn submit_plugin_install(&mut self, _request: PluginInstallRequest) -> bool { false }
    fn poll_plugin_install(&mut self) -> Option<PluginInstallReply> { None }
    /// Trusted installed content, separate from saved document dependency pins.
    fn sync_plugins(&mut self, _revision: u64, _fetch: &mut dyn FnMut() -> serde_json::Value) {}
    /// False explicitly reports a runner without action support.
    fn submit_plugin_action(&mut self, _request: PluginActionRequest) -> bool { false }
    fn poll_plugin_action(&mut self) -> Option<PluginActionReply> { None }

    /// Submit a history run tagged with a monotonic generation. The runner executes
    /// it (immediately for Inline; on a background thread later) and makes the reply
    /// available via [`poll_run`](Self::poll_run).
    fn submit_run(&mut self, request: brep_kernel::HistoryRequest, generation: u64);
    /// Non-blocking: the next completed run reply, if any (drained each frame).
    fn poll_run(&mut self) -> Option<RunReply>;
    /// Submit a per-object measurement query (answered against the runner's warm
    /// registry). Inline computes it immediately; a thread impl computes it on the
    /// runner thread and surfaces it via [`poll_query`](Self::poll_query).
    fn submit_query(&mut self, query: MeasureQuery);
    /// Non-blocking: the next completed measurement reply, if any.
    fn poll_query(&mut self) -> Option<MeasureReply>;
    /// Submit RANSAC mesh reconstruction to the runner's thread/worker.
    fn submit_mesh_import(&mut self, request: MeshImportRequest);
    /// Non-blocking: the next completed mesh reconstruction, if any.
    fn poll_mesh_import(&mut self) -> Option<MeshImportReply>;
    /// Submit a STEP product-structure probe (see [`StepProbeRequest`]).
    fn submit_step_probe(&mut self, request: StepProbeRequest);
    /// Non-blocking: the next completed STEP probe, if any.
    fn poll_step_probe(&mut self) -> Option<StepProbeReply>;
    /// Submit a topology request (see [`TopologyRequest`]). The default drops
    /// it: a runner that never answers leaves a sheet on its mesh
    /// approximation, which the drawing reports.
    fn submit_topology(&mut self, _request: TopologyRequest) {}
    /// Non-blocking: the next completed topology reply, if any.
    fn poll_topology(&mut self) -> Option<TopologyReply> {
        None
    }
    /// Submit a sheet placement's exact pass (see [`SheetLinesRequest`]).
    /// `false` when this runner cannot run one — the engine then runs it on
    /// its own thread, as every projection did before the pass moved off it.
    fn submit_sheet_lines(&mut self, _request: SheetLinesRequest) -> bool {
        false
    }
    /// Non-blocking: the next completed sheet-lines reply, if any.
    fn poll_sheet_lines(&mut self) -> Option<SheetLinesReply> {
        None
    }
    /// Drop the delta baseline (document switch → full rebuild).
    fn reset(&mut self);

    /// Bring the runner's resident PARTS LIBRARY up to `revision`, calling
    /// `fetch` ONLY when it is not already there. Called immediately before
    /// every [`submit_run`](Self::submit_run).
    ///
    /// This is the whole point of the library channel: the library is sent
    /// when it CHANGES (insert, document load, refresh), not on every run.
    /// Cheap to call — the revision comparison is an integer, and `fetch`
    /// (which clones the store) never runs in the steady state. The default is
    /// a no-op for [`InlineRunner`], which shares the caller's kernel store.
    fn sync_parts_library(
        &mut self,
        _revision: u64,
        _fetch: &mut dyn FnMut() -> brep_kernel::PartsLibraryMap,
    ) {
    }

    /// True when the runner REFUSED a run because its resident parts library
    /// could not serve it (see [`Reply::NeedPartsLibrary`]). It has already
    /// forgotten its copy, so the next `sync_parts_library` reinstalls; the
    /// caller must re-submit the run. Never true for a runner that shares the
    /// caller's store.
    fn poll_library_request(&mut self) -> bool {
        false
    }

    /// Non-blocking: the MOST RECENT progress report of the in-flight run, with
    /// any older ones discarded (only the latest feature matters). `None` for
    /// a synchronous runner — Inline has finished before anyone could ask.
    fn poll_progress(&mut self) -> Option<RunProgress> {
        None
    }

    /// ABANDON the in-flight work and start over with an EMPTY resident
    /// registry. Returns `false` when there is nothing this runner can abandon
    /// (Inline: the run already completed on the caller's thread). A `true`
    /// means every handle the caller holds is now invalid, no reply is coming
    /// for anything submitted so far, and the parts library must be re-sent —
    /// the caller reconciles its generations and pending sets accordingly
    /// (`EngineState::cancel_run`). Nothing inside a feature is interruptible:
    /// the thread runner lets its old thread finish the feature it is on and
    /// stop at the next boundary; the browser worker is terminated outright.
    fn cancel(&mut self) -> bool {
        false
    }
}

/// Measure `query` against `runner`'s resident geometry and emit the object-info
/// JSON FRAGMENT — EXACTLY the fields [`crate::metadata::EngineState::object_info_json`]
/// emits for that kind EXCEPT `name` and `creatingFeature` (the main thread injects
/// those from its eager provenance, so the merged output is byte-identical to the
/// pre-seam in-process result). Shared verbatim by the Inline and thread runners so
/// both produce the identical fragment. A missing handle / kernel error yields
/// `{ "ok": false, "message": .. }` (main injects `name`).
fn measure_json(runner: &crate::pipeline::SceneRunner, query: &MeasureQuery) -> String {
    let Some(handle) = runner.handle_of(&query.owner) else {
        return serde_json::json!({
            "ok": false,
            "message": format!("solid '{}' has no resident geometry", query.owner),
        })
        .to_string();
    };
    match query.kind {
        MeasureKind::Solid => {
            match brep_kernel::mass_properties_handle_native(handle, query.density) {
                Ok(properties) => {
                    let edge_total =
                        brep_kernel::solid_edge_length_total_native(handle).unwrap_or(0.0);
                    serde_json::json!({
                        "ok": true,
                        "kind": "solid",
                        "volume": properties.volume,
                        "surfaceArea": properties.surface_area,
                        "edgeLengthTotal": edge_total,
                        "density": properties.density,
                        "weight": properties.mass,
                    })
                    .to_string()
                }
                Err(error) => serde_json::json!({ "ok": false, "message": error }).to_string(),
            }
        }
        MeasureKind::Face => match brep_kernel::face_measurements_native(handle, &query.entity) {
            Ok((area, edge_total, surface_type)) => serde_json::json!({
                "ok": true,
                "kind": "face",
                "solid": query.owner,
                "surfaceType": surface_type,
                "area": area,
                "edgeLengthTotal": edge_total,
            })
            .to_string(),
            Err(error) => serde_json::json!({ "ok": false, "message": error }).to_string(),
        },
        MeasureKind::Edge => match brep_kernel::edge_length_native(handle, &query.entity) {
            Ok(length) => serde_json::json!({
                "ok": true,
                "kind": "edge",
                "solid": query.owner,
                "length": length,
            })
            .to_string(),
            Err(error) => serde_json::json!({ "ok": false, "message": error }).to_string(),
        },
    }
}

/// The default, SYNCHRONOUS runner: runs on submit, stashes the reply for an
/// immediate poll. Behavior-identical to the pre-seam in-process run.
pub struct InlineRunner {
    /// The scene-free history runner it owns (delta baseline lives here).
    runner: crate::pipeline::SceneRunner,
    /// Completed replies awaiting a poll. For Inline this holds exactly one entry
    /// between a `submit_run` and the immediately-following `poll_run`.
    pending: std::collections::VecDeque<RunReply>,
    /// Completed measurement replies awaiting a poll (computed synchronously on
    /// `submit_query`, popped on `poll_query`) — the synchronous mirror of the
    /// thread runner's query buffer, so the object-info path resolves same-call.
    query_pending: std::collections::VecDeque<MeasureReply>,
    mesh_import_pending: std::collections::VecDeque<MeshImportReply>,
    step_probe_pending: std::collections::VecDeque<StepProbeReply>,
    topology_pending: std::collections::VecDeque<TopologyReply>,
    sheet_lines_pending: std::collections::VecDeque<SheetLinesReply>,
    plugin_action_pending: std::collections::VecDeque<PluginActionReply>,
    plugin_install_pending: std::collections::VecDeque<PluginInstallReply>,
}

impl InlineRunner {
    pub fn new() -> Self {
        Self {
            runner: crate::pipeline::SceneRunner::new(),
            pending: std::collections::VecDeque::new(),
            query_pending: std::collections::VecDeque::new(),
            mesh_import_pending: std::collections::VecDeque::new(),
            step_probe_pending: std::collections::VecDeque::new(),
            topology_pending: std::collections::VecDeque::new(),
            sheet_lines_pending: std::collections::VecDeque::new(),
            plugin_action_pending: std::collections::VecDeque::new(),
            plugin_install_pending: std::collections::VecDeque::new(),
        }
    }
}

impl HistoryRunner for InlineRunner {
    fn submit_plugin_install(&mut self, request: PluginInstallRequest) -> bool {
        self.plugin_install_pending.push_back(validate_plugins(request));
        true
    }
    fn poll_plugin_install(&mut self) -> Option<PluginInstallReply> {
        self.plugin_install_pending.pop_front()
    }
    fn sync_plugins(&mut self, revision: u64, fetch: &mut dyn FnMut() -> serde_json::Value) {
        if self.runner.plugin_revision != Some(revision) {
            self.runner.set_plugins(revision, fetch());
        }
    }
    fn submit_plugin_action(&mut self, request: PluginActionRequest) -> bool {
        self.plugin_action_pending.push_back(self.runner.plugin_action(request));
        true
    }
    fn poll_plugin_action(&mut self) -> Option<PluginActionReply> {
        self.plugin_action_pending.pop_front()
    }

    fn submit_run(&mut self, request: brep_kernel::HistoryRequest, generation: u64) {
        let output = self.runner.run(&request);
        self.pending.push_back(RunReply { generation, output });
    }

    /// Nothing to send: Inline runs on the CALLER's thread against the CALLER's
    /// kernel store, so the library the caller would hand over is already the
    /// one this run resolves against. (Which is also why Inline never needs the
    /// preflight — there is only one store, and the caller seeds it directly.)
    fn sync_parts_library(
        &mut self,
        _revision: u64,
        _fetch: &mut dyn FnMut() -> brep_kernel::PartsLibraryMap,
    ) {
    }

    fn poll_run(&mut self) -> Option<RunReply> {
        self.pending.pop_front()
    }

    fn submit_query(&mut self, query: MeasureQuery) {
        let result = measure_json(&self.runner, &query);
        self.query_pending.push_back(MeasureReply { id: query.id, result });
    }

    fn poll_query(&mut self) -> Option<MeasureReply> {
        self.query_pending.pop_front()
    }

    fn submit_mesh_import(&mut self, request: MeshImportRequest) {
        self.mesh_import_pending
            .push_back(reconstruct_mesh(request));
    }

    fn poll_mesh_import(&mut self) -> Option<MeshImportReply> {
        self.mesh_import_pending.pop_front()
    }

    fn submit_step_probe(&mut self, request: StepProbeRequest) {
        self.step_probe_pending.push_back(probe_step(request));
    }

    fn poll_step_probe(&mut self) -> Option<StepProbeReply> {
        self.step_probe_pending.pop_front()
    }

    fn submit_topology(&mut self, request: TopologyRequest) {
        self.topology_pending.push_back(topology_reply(request));
    }

    fn poll_topology(&mut self) -> Option<TopologyReply> {
        self.topology_pending.pop_front()
    }

    /// Runs the pass at once, on the caller's thread: an inline runner's
    /// registry IS the caller's, so the answer is ready for the next poll.
    fn submit_sheet_lines(&mut self, request: SheetLinesRequest) -> bool {
        self.sheet_lines_pending.push_back(sheet_lines_reply(request));
        true
    }

    fn poll_sheet_lines(&mut self) -> Option<SheetLinesReply> {
        self.sheet_lines_pending.pop_front()
    }

    fn reset(&mut self) {
        self.runner.reset();
    }
}

impl Default for InlineRunner {
    fn default() -> Self {
        Self::new()
    }
}

// ===========================================================================
// Shared run/query PROTOCOL (M3b): the `Command`/`Reply` message pair and the
// `process_command` step. Both async runners speak it — the native `ThreadRunner`
// ships the enums over `mpsc` (no serialization), and the wasm `WorkerRunner`
// serializes them to JSON for `postMessage`. Hence the serde derives (the enums
// carry serde types: `HistoryRequest`/`MeasureQuery`/`RunReply`/`MeasureReply`)
// and `process_command` being un-gated + shared so both drivers do the SAME work.
// ===========================================================================

/// A command sent main → runner (thread channel OR worker `postMessage`) over one
/// ordered stream (so a `Reset` before a `Run` stays before it). `Run` carries the
/// whole request; the driver coalesces consecutive `Run`s to shed a slider drag's
/// backlog (the thread in `thread_main`, the worker's main side in `WorkerRunner`).
#[derive(serde::Serialize, serde::Deserialize)]
pub enum Command {
    ValidatePlugins(PluginInstallRequest),
    SetPlugins { revision: u64, packages: serde_json::Value },
    PluginAction(PluginActionRequest),
    Run {
        request: brep_kernel::HistoryRequest,
        generation: u64,
        /// The [`brep_kernel::parts_library_revision`] this run was built
        /// against. The runner refuses to execute a run stamped for a library
        /// it does not hold — see [`process_command`].
        #[serde(default)]
        parts_library_revision: u64,
    },
    /// Install the parts library on the runner's OWN kernel store. Sent only
    /// when the library actually changes (insert, document load, refresh),
    /// never per run: it carries the whole embedded-part payload, which for an
    /// imported STEP assembly is megabytes, and stringifying that on the
    /// browser's main thread once per edit is what froze the UI.
    ///
    /// Deliberately its OWN command rather than an optional field on `Run`:
    /// both drivers COALESCE consecutive runs (see [`thread_main`] and
    /// `WorkerRunner::submit_run`), so a library riding on a run could be
    /// dropped with it. A `SetPartsLibrary` is never coalesced away, and the
    /// stream is ordered, so it always lands before the run that needs it.
    SetPartsLibrary {
        library: brep_kernel::PartsLibraryMap,
        revision: u64,
    },
    Query(MeasureQuery),
    MeshImport(MeshImportRequest),
    StepProbe(StepProbeRequest),
    Topology(TopologyRequest),
    SheetLines(SheetLinesRequest),
    Reset,
}

/// A reply sent runner → main; the main side demuxes it into per-kind buffers.
#[derive(serde::Serialize, serde::Deserialize)]
pub enum Reply {
    ValidatedPlugins(PluginInstallReply),
    PluginAction(PluginActionReply),
    Run(RunReply),
    Query(MeasureReply),
    MeshImport(MeshImportReply),
    StepProbe(StepProbeReply),
    Topology(TopologyReply),
    SheetLines(SheetLinesReply),
    /// The runner REFUSED a run because its resident parts library could not
    /// serve it (a part the request references is missing, or the run was
    /// stamped for a different library revision). No geometry was touched; the
    /// main side re-sends the library and re-submits. Refusing is the whole
    /// point — running with the wrong library would be silently wrong
    /// geometry, which is far worse than one extra round trip.
    NeedPartsLibrary,
    /// A feature of the in-flight run is about to execute (see
    /// [`RunProgress`]). Posted from INSIDE `process_command`, before its
    /// `Reply::Run`; the main side keeps only the latest.
    Progress(RunProgress),
}

/// The probe itself — the one parse of a structured STEP import (see
/// `EngineState::submit_step_probe`), run wherever the runner runs.
fn probe_step(request: StepProbeRequest) -> StepProbeReply {
    StepProbeReply {
        id: request.id,
        result: brep_kernel::read_step_assembly(&request.text),
    }
}

fn reconstruct_mesh(request: MeshImportRequest) -> MeshImportReply {
    let result = (|| {
        use brep_reconstruction::stl_conversion::{
            binary_stl_coordinate_precision_tolerance, convert_stl_mesh_to_step,
        };
        use brep_reconstruction::{Mesh, Vec3};

        // A 3MF build's placements, kept only to name each body afterwards.
        let mut instances = Vec::new();
        let (mesh, positions, indices, coordinate_precision_tolerance) = match request.format {
            MeshImportFormat::Stl => {
                use brep_reconstruction::stl::{parse_stl_bytes, StlFormat, StlReadOptions};
                let read_options = StlReadOptions {
                    weld_tolerance: (request.options.weld_tolerance >= 0.0)
                        .then_some(request.options.weld_tolerance),
                };
                let imported = parse_stl_bytes(&request.bytes, &read_options)
                    .map_err(|error| format!("STL import failed: {error}"))?;
                let positions = imported
                    .mesh
                    .vertices
                    .iter()
                    .flat_map(|point| [point.x, point.y, point.z])
                    .collect::<Vec<_>>();
                let indices = imported
                    .mesh
                    .triangles
                    .iter()
                    .flatten()
                    .copied()
                    .collect::<Vec<_>>();
                let precision = if imported.format == StlFormat::Binary {
                    binary_stl_coordinate_precision_tolerance(&imported.mesh)
                } else {
                    0.0
                };
                (imported.mesh, positions, indices, precision)
            }
            MeshImportFormat::Obj => {
                let text = std::str::from_utf8(&request.bytes)
                    .map_err(|_| "OBJ import failed: file is not UTF-8 text".to_string())?;
                let obj = brep_kernel::read_obj(text)
                    .map_err(|error| format!("OBJ import failed: {error}"))?;
                let vertices = obj
                    .positions
                    .chunks_exact(3)
                    .map(|point| Vec3::new(point[0], point[1], point[2]))
                    .collect::<Vec<_>>();
                let triangles = obj
                    .indices
                    .chunks_exact(3)
                    .map(|triangle| [triangle[0], triangle[1], triangle[2]])
                    .collect::<Vec<_>>();
                (
                    Mesh::new(vertices, triangles),
                    obj.positions,
                    obj.indices,
                    0.0,
                )
            }
            // The 3MF reader has already applied the file's unit and every
            // build/component transform, so what arrives here is the same kind
            // of millimetre triangle soup an STL carries — including a build
            // that places SEVERAL instances, which is several bodies in that
            // one buffer. The chain splits a soup into its vertex-connected
            // components and reconstructs each as its own body, so the whole
            // build goes through in ONE call and the poses are already in the
            // coordinates.
            MeshImportFormat::ThreeMf => {
                let model = brep_kernel::read_3mf(&request.bytes)
                    .map_err(|error| format!("3MF import failed: {error}"))?;
                instances = model.instances;
                let vertices = model
                    .positions
                    .chunks_exact(3)
                    .map(|point| Vec3::new(point[0], point[1], point[2]))
                    .collect::<Vec<_>>();
                let triangles = model
                    .indices
                    .chunks_exact(3)
                    .map(|triangle| [triangle[0], triangle[1], triangle[2]])
                    .collect::<Vec<_>>();
                (
                    Mesh::new(vertices, triangles),
                    model.positions,
                    model.indices,
                    0.0,
                )
            }
        };
        let mut options = request.options;
        options.coordinate_precision_tolerance = options.coordinate_precision_tolerance
            .max(coordinate_precision_tolerance);
        convert_stl_mesh_to_step(
            &mesh,
            &positions,
            Some(&indices),
            &options,
            "Imported mesh",
            "MM",
            "",
        )
        .map(|mut output| {
            name_bodies(&mut output.report, &instances);
            output
        })
        .map_err(|error| format!("RANSAC reconstruction failed: {error}"))
    })();
    MeshImportReply {
        id: request.id,
        result,
    }
}

/// Give each body the name of the 3MF object its first source triangle came
/// from, so a refusal can say WHICH object is missing. Bodies are found in the
/// flattened soup the reader laid out instance by instance.
fn name_bodies(
    report: &mut brep_reconstruction::stl_conversion::StlConversionReport,
    instances: &[brep_kernel::ThreeMfInstance],
) {
    for component in &mut report.components {
        component.source_name = instances
            .iter()
            .find(|instance| {
                (instance.first_triangle..instance.first_triangle + instance.triangle_count)
                    .contains(&component.first_triangle)
            })
            .and_then(|instance| instance.name.clone());
    }
}

/// The notice for bodies a multi-body import left out, one clause per body,
/// named by the source's own name when it has one; `None` when every body was
/// built.
pub(crate) fn refused_bodies_notice(
    report: &brep_reconstruction::stl_conversion::StlConversionReport,
) -> Option<String> {
    let refused = report
        .components
        .iter()
        .filter_map(|component| {
            component.refusal.as_ref().map(|reason| {
                let body = match &component.source_name {
                    Some(name) => format!("body {} \"{name}\"", component.index + 1),
                    None => format!(
                        "body {} (from source triangle {})",
                        component.index + 1,
                        component.first_triangle
                    ),
                };
                format!("{body}: {reason}")
            })
        })
        .collect::<Vec<_>>();
    (!refused.is_empty()).then(|| {
        format!(
            "mesh import: {} of {} bodies could not be reconstructed and are NOT in the model — {}",
            refused.len(),
            report.components.len(),
            refused.join("; ")
        )
    })
}

/// Execute ONE [`Command`] against `runner` and return the [`Reply`] it produces,
/// if any. The shared step both async drivers run: the native [`thread_main`]
/// calls it for each (post-coalescing) command on the runner thread; the wasm
/// `WorkerRunner`'s `worker_entry` calls it per `postMessage` on the worker.
/// `Run` → a [`Reply::Run`]; `Query` → a [`Reply::Query`]; `Reset` drops the delta
/// baseline AND the runner's OWN kernel history cache (the resident registry lives
/// with the runner — thread or worker — not on main) and yields no reply.
///
/// `progress` is called before every feature a `Run` actually executes, with
/// the report the driver should ship as [`Reply::Progress`]; returning `false`
/// stops the run at that boundary (the native thread's cooperative cancel —
/// the worker is terminated instead and always returns `true`).
pub fn process_command(
    runner: &mut crate::pipeline::SceneRunner,
    command: Command,
    progress: &mut dyn FnMut(RunProgress) -> bool,
) -> Option<Reply> {
    match command {
        Command::ValidatePlugins(request) => Some(Reply::ValidatedPlugins(validate_plugins(request))),
        Command::SetPlugins { revision, packages } => {
            runner.set_plugins(revision, packages);
            None
        }
        Command::PluginAction(request) => Some(Reply::PluginAction(runner.plugin_action(request))),
        Command::Run {
            request,
            generation,
            parts_library_revision,
        } => {
            // PREFLIGHT (see `Reply::NeedPartsLibrary`). Two independent
            // checks, because neither alone is enough:
            //
            // * the revision stamp catches CHANGED content the sender knows
            //   about but this store has not received;
            // * `missing_library_parts` catches content that is simply GONE
            //   here — the orphan GC at the end of every run drops entries no
            //   ACOMP in THAT run referenced, so an undo to zero components
            //   empties this store while the sender's (which never ran) keeps
            //   everything and its revision never moves. Only looking at the
            //   actual content sees that.
            let stale = match runner.parts_library_revision {
                Some(installed) => installed != parts_library_revision,
                // Nothing installed yet. Accept a run stamped 0 — that is
                // either an empty library or a caller driving `submit_run`
                // directly (the runner tests); the content preflight below is
                // what actually protects the run. A non-zero stamp with nothing
                // installed IS a gap: refuse it.
                None => parts_library_revision != 0,
            };
            // Only a part the sender DID send and this store has since lost is
            // worth asking for again. One it never sent is a dangling reference
            // in the document itself: run it, so the ACOMP feature reports it
            // the way it always has. (An install inserts every incoming name,
            // so right after one this set can only be empty — which is what
            // makes the ask-and-retry terminate.)
            let recoverable = brep_kernel::missing_library_parts(&request)
                .iter()
                .any(|name| runner.parts_library_names.contains(name));
            if stale || recoverable {
                runner.parts_library_revision = None;
                return Some(Reply::NeedPartsLibrary);
            }
            let output = runner.run_observed(&request, &mut |event| {
                progress(RunProgress {
                    generation,
                    index: event.index,
                    total: event.total,
                    feature_id: event.id.to_string(),
                    feature_type: event.feature_type.to_string(),
                })
            });
            Some(Reply::Run(RunReply { generation, output }))
        }
        Command::SetPartsLibrary { library, revision } => {
            runner.parts_library_names = library.keys().cloned().collect();
            brep_kernel::install_parts_library(&library);
            runner.parts_library_revision = Some(revision);
            None
        }
        Command::Query(query) => {
            let result = measure_json(runner, &query);
            Some(Reply::Query(MeasureReply { id: query.id, result }))
        }
        Command::MeshImport(request) => Some(Reply::MeshImport(reconstruct_mesh(request))),
        Command::StepProbe(request) => Some(Reply::StepProbe(probe_step(request))),
        Command::Topology(request) => Some(Reply::Topology(topology_reply(request))),
        Command::SheetLines(request) => Some(Reply::SheetLines(sheet_lines_reply(request))),
        Command::Reset => {
            // A document switch: drop the delta baseline AND this runner's OWN kernel
            // history cache (the resident registry lives here, not on main), mirroring
            // `set_history_json`'s main-thread clear so a new model rebuilds fully and
            // the old model's handles are freed.
            runner.reset();
            brep_kernel::clear_history_cache();
            None
        }
    }
}

// ===========================================================================
// ThreadRunner (M2b): a persistent std::thread that OWNS the SceneRunner, so a
// history run — and per-object measurement queries — execute OFF the main thread
// and the native UI stays responsive during a run AND during selection. Native
// only: `std::thread` + `std::sync::mpsc` do not exist on wasm32 (M3b lands a
// worker impl behind this same trait), so the whole thing is cfg-gated out there.
// ===========================================================================

/// The persistent-thread runner. `submit_*`/`reset` push [`Command`]s down the
/// channel; `poll_*` first DRAIN every ready [`Reply`] into the two demux buffers,
/// then pop the matching one. The `SceneRunner` (and thus the kernel's resident
/// registry it warms) lives ENTIRELY on the thread — it is never shared — so the
/// only cross-thread traffic is the `Send` command/reply payloads.
#[cfg(not(target_arch = "wasm32"))]
pub struct ThreadRunner {
    /// Main → thread. `Option` so [`Drop`] can take + drop it, ending the thread's
    /// blocking `recv` after already-submitted work finishes.
    tx: Option<std::sync::mpsc::Sender<Command>>,
    /// Thread → main.
    rx: std::sync::mpsc::Receiver<Reply>,
    /// Dropped without joining so closing a busy preview never stalls the UI.
    handle: Option<std::thread::JoinHandle<()>>,
    /// Demuxed completed run replies awaiting `poll_run`.
    run_buf: std::collections::VecDeque<RunReply>,
    /// Demuxed completed measurement replies awaiting `poll_query`.
    query_buf: std::collections::VecDeque<MeasureReply>,
    mesh_import_buf: std::collections::VecDeque<MeshImportReply>,
    step_probe_buf: std::collections::VecDeque<StepProbeReply>,
    topology_buf: std::collections::VecDeque<TopologyReply>,
    sheet_lines_buf: std::collections::VecDeque<SheetLinesReply>,
    plugin_action_buf: std::collections::VecDeque<PluginActionReply>,
    plugin_install_buf: std::collections::VecDeque<PluginInstallReply>,
    sent_plugin_revision: Option<u64>,
    /// Progress reports of the in-flight run, oldest first (`poll_progress`
    /// keeps the newest).
    progress_buf: std::collections::VecDeque<RunProgress>,
    /// The parts-library revision last SENT down the channel (`None` = never,
    /// or the thread dropped it). Lives here rather than on the caller so
    /// "reset forgets the library" is a local property of this object.
    sent_library_revision: Option<u64>,
    /// The thread refused a run for want of its library (drained by
    /// [`HistoryRunner::poll_library_request`]).
    library_requested: bool,
    /// The cooperative stop flag THIS thread checks between features. Each
    /// spawn gets its own, so a cancelled thread keeps its raised flag while
    /// the replacement starts clean.
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
}

#[cfg(not(target_arch = "wasm32"))]
impl ThreadRunner {
    pub fn new() -> Self {
        let (tx, cmd_rx) = std::sync::mpsc::channel::<Command>();
        let (reply_tx, rx) = std::sync::mpsc::channel::<Reply>();
        let stop = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        let thread_stop = stop.clone();
        let handle = std::thread::Builder::new()
            .name("brep-history-runner".to_string())
            .spawn(move || thread_main(cmd_rx, reply_tx, thread_stop))
            .expect("spawn brep-history-runner thread");
        Self {
            tx: Some(tx),
            rx,
            handle: Some(handle),
            run_buf: std::collections::VecDeque::new(),
            query_buf: std::collections::VecDeque::new(),
            mesh_import_buf: std::collections::VecDeque::new(),
            step_probe_buf: std::collections::VecDeque::new(),
            topology_buf: std::collections::VecDeque::new(),
            sheet_lines_buf: std::collections::VecDeque::new(),
            plugin_action_buf: std::collections::VecDeque::new(),
            plugin_install_buf: std::collections::VecDeque::new(),
            sent_plugin_revision: None,
            progress_buf: std::collections::VecDeque::new(),
            sent_library_revision: None,
            library_requested: false,
            stop,
        }
    }

    /// Pull every ready reply off the channel and demux it into the run/query
    /// buffers (so a `poll_run` never swallows a query reply and vice versa).
    fn drain(&mut self) {
        while let Ok(reply) = self.rx.try_recv() {
            match reply {
                Reply::ValidatedPlugins(reply) => self.plugin_install_buf.push_back(reply),
                Reply::PluginAction(reply) => self.plugin_action_buf.push_back(reply),
                Reply::Run(run) => self.run_buf.push_back(run),
                Reply::Query(query) => self.query_buf.push_back(query),
                Reply::MeshImport(reply) => self.mesh_import_buf.push_back(reply),
                Reply::StepProbe(reply) => self.step_probe_buf.push_back(reply),
                Reply::Topology(reply) => self.topology_buf.push_back(reply),
                Reply::SheetLines(reply) => self.sheet_lines_buf.push_back(reply),
                Reply::Progress(progress) => self.progress_buf.push_back(progress),
                Reply::NeedPartsLibrary => {
                    // The thread dropped (or never had) the library this run
                    // needs. Forget what we believe it holds so the next
                    // `sync_parts_library` re-installs, and flag the refused
                    // run for the caller to re-submit.
                    self.sent_library_revision = None;
                    self.library_requested = true;
                }
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Default for ThreadRunner {
    fn default() -> Self {
        Self::new()
    }
}

/// The runner thread's whole life: block on the next command, batch it with every
/// other command already queued, COALESCE consecutive `Run`s (a `Run` immediately
/// followed by another `Run` — with no `Query`/`Reset` between — is dropped; only
/// the last of each consecutive group runs), then process the batch IN ORDER so a
/// `Reset` or a `Query` interleaved between two runs keeps its place. Exits when
/// the command sender is dropped (`recv` errors) or the reply receiver is gone (a
/// `send` errors — the main side went away).
///
/// `stop` is the cooperative cancel: raised by [`ThreadRunner::cancel`] on a
/// thread that has already been abandoned, it is checked before every feature
/// a run executes, so the abandoned thread finishes the feature it is on and
/// exits at the next boundary instead of running the rest of the history for
/// a receiver that is gone.
#[cfg(not(target_arch = "wasm32"))]
fn thread_main(
    cmd_rx: std::sync::mpsc::Receiver<Command>,
    reply_tx: std::sync::mpsc::Sender<Reply>,
    stop: std::sync::Arc<std::sync::atomic::AtomicBool>,
) {
    let mut runner = crate::pipeline::SceneRunner::new();
    while let Ok(first) = cmd_rx.recv() {
        if stop.load(std::sync::atomic::Ordering::Relaxed) {
            return;
        }
        // Gather this command plus everything else already waiting.
        let mut batch = vec![first];
        loop {
            match cmd_rx.try_recv() {
                Ok(command) => batch.push(command),
                Err(_) => break, // Empty or Disconnected — process what we have.
            }
        }
        // A Run immediately followed by another Run is coalesced away (its result
        // would be overwritten before anything observed it); a Run followed by a
        // Query/Reset/end still runs, so interleaved work sees the right geometry.
        let mut run_here: Vec<bool> = vec![true; batch.len()];
        for i in 0..batch.len() {
            if matches!(batch[i], Command::Run { .. })
                && matches!(batch.get(i + 1), Some(Command::Run { .. }))
            {
                run_here[i] = false;
            }
        }
        // Process the KEPT commands in order through the SHARED `process_command`
        // (byte-identical work to the wasm worker's per-message step), sending each
        // reply it produces; a coalesced-away Run is skipped without touching the
        // runner, so interleaved Query/Reset still see the right geometry.
        for (i, command) in batch.into_iter().enumerate() {
            if !run_here[i] {
                continue;
            }
            let mut progress = |report: RunProgress| {
                // A lost receiver means the main side abandoned this thread:
                // stop at this boundary rather than finishing for nobody.
                reply_tx.send(Reply::Progress(report)).is_ok()
                    && !stop.load(std::sync::atomic::Ordering::Relaxed)
            };
            if let Some(reply) = process_command(&mut runner, command, &mut progress) {
                if reply_tx.send(reply).is_err() {
                    return; // The reply receiver is gone — the main side went away.
                }
            }
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl HistoryRunner for ThreadRunner {
    fn submit_plugin_install(&mut self, request: PluginInstallRequest) -> bool {
        self.tx.as_ref().is_some_and(|tx| tx.send(Command::ValidatePlugins(request)).is_ok())
    }
    fn poll_plugin_install(&mut self) -> Option<PluginInstallReply> {
        self.drain();
        self.plugin_install_buf.pop_front()
    }
    fn sync_plugins(&mut self, revision: u64, fetch: &mut dyn FnMut() -> serde_json::Value) {
        if self.sent_plugin_revision != Some(revision) {
            if let Some(tx) = &self.tx {
                if tx.send(Command::SetPlugins { revision, packages: fetch() }).is_ok() {
                    self.sent_plugin_revision = Some(revision);
                }
            }
        }
    }
    fn submit_plugin_action(&mut self, request: PluginActionRequest) -> bool {
        self.tx.as_ref().is_some_and(|tx| tx.send(Command::PluginAction(request)).is_ok())
    }
    fn poll_plugin_action(&mut self) -> Option<PluginActionReply> {
        self.drain();
        self.plugin_action_buf.pop_front()
    }

    fn submit_run(&mut self, request: brep_kernel::HistoryRequest, generation: u64) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::Run {
                request,
                generation,
                parts_library_revision: self.sent_library_revision.unwrap_or(0),
            });
        }
    }

    fn sync_parts_library(
        &mut self,
        revision: u64,
        fetch: &mut dyn FnMut() -> brep_kernel::PartsLibraryMap,
    ) {
        if self.sent_library_revision == Some(revision) {
            return;
        }
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::SetPartsLibrary {
                library: fetch(),
                revision,
            });
            self.sent_library_revision = Some(revision);
        }
    }

    fn poll_library_request(&mut self) -> bool {
        self.drain();
        std::mem::take(&mut self.library_requested)
    }

    fn poll_run(&mut self) -> Option<RunReply> {
        self.drain();
        self.run_buf.pop_front()
    }

    fn submit_query(&mut self, query: MeasureQuery) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::Query(query));
        }
    }

    fn poll_query(&mut self) -> Option<MeasureReply> {
        self.drain();
        self.query_buf.pop_front()
    }

    fn submit_mesh_import(&mut self, request: MeshImportRequest) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::MeshImport(request));
        }
    }

    fn poll_mesh_import(&mut self) -> Option<MeshImportReply> {
        self.drain();
        self.mesh_import_buf.pop_front()
    }

    fn submit_step_probe(&mut self, request: StepProbeRequest) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::StepProbe(request));
        }
    }

    fn poll_step_probe(&mut self) -> Option<StepProbeReply> {
        self.drain();
        self.step_probe_buf.pop_front()
    }

    fn submit_topology(&mut self, request: TopologyRequest) {
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::Topology(request));
        }
    }

    fn poll_topology(&mut self) -> Option<TopologyReply> {
        self.drain();
        self.topology_buf.pop_front()
    }

    fn submit_sheet_lines(&mut self, request: SheetLinesRequest) -> bool {
        match &self.tx {
            Some(tx) => tx.send(Command::SheetLines(request)).is_ok(),
            None => false,
        }
    }

    fn poll_sheet_lines(&mut self) -> Option<SheetLinesReply> {
        self.drain();
        self.sheet_lines_buf.pop_front()
    }

    fn poll_progress(&mut self) -> Option<RunProgress> {
        self.drain();
        let latest = self.progress_buf.pop_back();
        self.progress_buf.clear();
        latest
    }

    /// Abandon the thread: raise its stop flag, close its channels, and spawn a
    /// fresh thread with a fresh registry. The old thread cannot be interrupted
    /// inside a feature — it finishes the one it is on (burning CPU until
    /// then), sees the flag, and exits. Always `true`: there is always a
    /// thread to replace.
    fn cancel(&mut self) -> bool {
        self.stop.store(true, std::sync::atomic::Ordering::Relaxed);
        self.tx.take();
        self.handle.take();
        *self = Self::new();
        true
    }

    fn reset(&mut self) {
        self.sent_plugin_revision = None;
        self.plugin_action_buf.clear();
        if let Some(tx) = &self.tx {
            let _ = tx.send(Command::Reset);
        }
        // A reset is a wholesale document switch: any replies still buffered from
        // the old model are stale — drop them (a fresh run/query supersedes).
        self.run_buf.clear();
        self.query_buf.clear();
        self.mesh_import_buf.clear();
        self.step_probe_buf.clear();
        self.topology_buf.clear();
        self.sheet_lines_buf.clear();
        self.progress_buf.clear();
        // `Command::Reset` clears the thread's kernel store, library included,
        // so what we believe it holds is void.
        self.sent_library_revision = None;
        self.library_requested = false;
    }
}

#[cfg(not(target_arch = "wasm32"))]
impl Drop for ThreadRunner {
    fn drop(&mut self) {
        // Closing a document or cancelling an import preview must not block
        // the UI on reconstruction. Closing the channel lets the worker exit
        // and release its registry once already-submitted work finishes.
        self.tx.take();
        self.handle.take();
    }
}

