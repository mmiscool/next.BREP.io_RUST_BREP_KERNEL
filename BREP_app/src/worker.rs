//! `WorkerRunner` (M3b) — the wasm history runner that executes a history OFF the
//! browser main thread in a dedicated WEB WORKER, so the single-threaded wasm UI
//! stays responsive during a run (and during per-object measurement queries). It
//! is the wasm sibling of `brep-render`'s native `ThreadRunner`, behind the SAME
//! [`HistoryRunner`] trait: `submit_*` / `reset` serialize a
//! [`Command`](brep_render::runner::Command) to JSON and `postMessage` it to the
//! worker; `poll_*` drain the JSON [`Reply`](brep_render::runner::Reply)s the
//! worker's `onmessage` pushes into a shared inbox.
//!
//! Everything here runs on the MAIN thread (single-threaded wasm) — the worker is a
//! separate agent reached only through `postMessage` — so `Rc<RefCell<..>>` is the
//! right shared-state primitive (no `Send`; the `HistoryRunner` trait imposes none).
//! The worker's CPU-only side is [`worker_entry`] at the bottom of this file.

use std::cell::RefCell;
use std::collections::VecDeque;
use std::rc::Rc;

use brep_render::brep_kernel::HistoryRequest;
use brep_render::runner::{
    Command, HistoryRunner, MeasureQuery, MeasureReply, MeshImportReply, MeshImportRequest, Reply,
    RunProgress, RunReply, SheetLinesReply, SheetLinesRequest, StepProbeReply, StepProbeRequest,
    TopologyReply, TopologyRequest, PluginActionRequest, PluginActionReply, PluginInstallRequest, PluginInstallReply,
};
use wasm_bindgen::prelude::*;
use wasm_bindgen::JsCast;
use web_sys::MessageEvent;

/// A [`HistoryRunner`] that offloads runs + measurement queries to a dedicated
/// MODULE web worker (`worker.js` → [`worker_entry`]). Holds the worker, a shared
/// `inbox` its `onmessage` fills, the two demux buffers, the kept `onmessage`
/// closure, and MAIN-SIDE run coalescing (`in_flight` + `pending_run`).
pub struct WorkerRunner {
    /// The dedicated worker running [`worker_entry`] (it owns the resident registry).
    worker: web_sys::Worker,
    /// Replies the worker's `onmessage` has pushed, awaiting [`Self::drain`]. Shared
    /// with the kept closure (both on the main thread → `Rc<RefCell<..>>`, no `Send`).
    inbox: Rc<RefCell<VecDeque<Reply>>>,
    /// The kept `onmessage` closure — dropping it detaches the handler, so it lives
    /// as long as the runner.
    _onmessage: Closure<dyn FnMut(MessageEvent)>,
    /// Demuxed completed run replies awaiting `poll_run`.
    run_buf: VecDeque<RunReply>,
    /// Demuxed completed measurement replies awaiting `poll_query`.
    query_buf: VecDeque<MeasureReply>,
    /// Completed RANSAC reconstruction replies awaiting the engine pump.
    mesh_import_buf: VecDeque<MeshImportReply>,
    /// Answered STEP structure probes awaiting the engine pump.
    step_probe_buf: VecDeque<StepProbeReply>,
    /// Answered topology requests awaiting the engine pump.
    topology_buf: VecDeque<TopologyReply>,
    /// Answered sheet exact passes awaiting the engine pump.
    sheet_lines_buf: VecDeque<SheetLinesReply>,
    /// Progress reports of the in-flight run, oldest first (`poll_progress`
    /// keeps the newest).
    progress_buf: VecDeque<RunProgress>,
    /// MAIN-SIDE run coalescing: a run is on the worker (posted, reply not yet drained).
    in_flight: bool,
    /// The latest run submitted WHILE one was in flight — REPLACES any earlier parked
    /// run (latest wins), so a slider drag never queues stale intermediates; posted
    /// when the in-flight run's reply drains (see [`Self::drain`]). Carries the
    /// parts-library revision STAMPED AT SUBMIT TIME, so a run parked before a
    /// library change never posts claiming the newer one.
    pending_run: Option<(HistoryRequest, u64, u64)>,
    /// The parts-library revision last POSTED to the worker (`None` = never, or
    /// the worker dropped it). Kept here rather than on the engine so "reset
    /// forgets the library" is a local property of this object.
    sent_library_revision: Option<u64>,
    /// The worker refused a run for want of its library (drained by
    /// [`HistoryRunner::poll_library_request`]).
    library_requested: bool,
    sent_plugin_revision: Option<u64>,
    plugin_action_buf: VecDeque<PluginActionReply>,
    plugin_install_buf: VecDeque<PluginInstallReply>,
    pending_install: Option<PluginInstallRequest>,
}

impl WorkerRunner {
    /// Spawn the module worker and wire its reply pump. Panics with a clear message
    /// if the worker cannot be created (the browser could not load `./worker.js`).
    pub fn new() -> Self {
        let worker = spawn_worker();

        // The worker posts a JSON `Reply` string per completed command; parse it back
        // and queue it for the next `poll_*` drain.
        let inbox: Rc<RefCell<VecDeque<Reply>>> = Rc::new(RefCell::new(VecDeque::new()));
        let inbox_cb = inbox.clone();
        let onmessage = Closure::wrap(Box::new(move |event: MessageEvent| {
            if let Some(text) = event.data().as_string() {
                match serde_json::from_str::<Reply>(&text) {
                    Ok(reply) => inbox_cb.borrow_mut().push_back(reply),
                    Err(error) => log::error!("worker reply parse failed: {error}"),
                }
            }
        }) as Box<dyn FnMut(MessageEvent)>);
        worker.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));

        Self {
            worker,
            inbox,
            _onmessage: onmessage,
            run_buf: VecDeque::new(),
            query_buf: VecDeque::new(),
            mesh_import_buf: VecDeque::new(),
            step_probe_buf: VecDeque::new(),
            topology_buf: VecDeque::new(),
            sheet_lines_buf: VecDeque::new(),
            progress_buf: VecDeque::new(),
            in_flight: false,
            pending_run: None,
            sent_library_revision: None,
            library_requested: false,
            sent_plugin_revision: None,
            plugin_action_buf: VecDeque::new(),
            plugin_install_buf: VecDeque::new(),
            pending_install: None,
        }
    }

    /// Serialize `command` and `postMessage` it to the worker as a JSON string.
    fn post(&self, command: &Command) -> Result<(), JsValue> {
        let json = serde_json::to_string(command).expect("serialize worker command");
        self.worker.post_message(&JsValue::from_str(&json))
    }

    /// Post a `Run` and mark a run in flight. A failed post is FATAL: leaving
    /// `in_flight = true` after a swallowed error would wedge coalescing forever (the
    /// parked `pending_run` would never post), so panic loudly instead.
    fn post_run(&mut self, request: HistoryRequest, generation: u64, library_revision: u64) {
        self.post(&Command::Run {
            request,
            generation,
            parts_library_revision: library_revision,
        })
        .unwrap_or_else(|e| panic!("post run to history worker failed: {e:?}"));
        self.in_flight = true;
    }

    /// Move every queued inbox `Reply` into the per-kind buffers. On a `Run` reply the
    /// in-flight run is done: clear the flag and, if a newer run is parked in
    /// `pending_run`, post it now — this is the one-in-flight coalescing.
    fn drain(&mut self) {
        loop {
            // Bind the pop result before matching so the inbox borrow is released
            // before `post_run` (which does not touch the inbox, but keep it tight).
            let next = self.inbox.borrow_mut().pop_front();
            let Some(reply) = next else { break };
            match reply {
                Reply::PluginAction(reply) => self.plugin_action_buf.push_back(reply),
                Reply::ValidatedPlugins(reply) => {
                    if self.pending_install.as_ref().is_some_and(|p| p.id == reply.id) { self.pending_install = None; }
                    self.plugin_install_buf.push_back(reply);
                }
                Reply::Run(run) => {
                    self.run_buf.push_back(run);
                    self.in_flight = false;
                    if let Some((request, generation, revision)) = self.pending_run.take() {
                        self.post_run(request, generation, revision);
                    }
                }
                Reply::Query(query) => self.query_buf.push_back(query),
                Reply::MeshImport(reply) => self.mesh_import_buf.push_back(reply),
                Reply::StepProbe(reply) => self.step_probe_buf.push_back(reply),
                Reply::Topology(reply) => self.topology_buf.push_back(reply),
                Reply::SheetLines(reply) => self.sheet_lines_buf.push_back(reply),
                Reply::Progress(progress) => self.progress_buf.push_back(progress),
                Reply::NeedPartsLibrary => {
                    // The worker dropped (or never had) the library this run
                    // needs and did NOT run. Clear the in-flight flag (no run
                    // reply is coming for it), forget what we believe the
                    // worker holds, and flag the refusal for the engine to
                    // re-drive — running against the wrong library would be
                    // silently wrong geometry.
                    self.in_flight = false;
                    self.sent_library_revision = None;
                    self.library_requested = true;
                    // Any parked run carries the SAME rejected stamp, so posting
                    // it would only be refused again. Drop it — the engine
                    // re-runs from the current document once the library lands.
                    self.pending_run = None;
                }
            }
        }
    }
}

impl Default for WorkerRunner {
    fn default() -> Self {
        Self::new()
    }
}

/// Spawn the module worker (`./worker.js` → [`worker_entry`]). Panics with a
/// clear message if the browser cannot create it. `worker.js` buffers every
/// message posted before its wasm has initialised, so the caller may post to
/// the returned worker immediately.
fn spawn_worker() -> web_sys::Worker {
    let options = web_sys::WorkerOptions::new();
    // 0.3.104 deprecates the builder `type_()`; `set_type` is the current setter.
    options.set_type(web_sys::WorkerType::Module);
    web_sys::Worker::new_with_options("./worker.js", &options)
        .unwrap_or_else(|e| panic!("failed to spawn history worker (./worker.js): {e:?}"))
}

/// A closed document TAKES ITS WORKER WITH IT. Nothing else terminates a
/// `Worker` — dropping the handle leaves the thread (and its instantiated wasm
/// module, which for this kernel is the expensive part) alive for the life of
/// the page. Since every document owns its own runner (`crate::document`),
/// closing tabs without this would leak one worker per close.
impl Drop for WorkerRunner {
    fn drop(&mut self) {
        self.worker.terminate();
    }
}

impl HistoryRunner for WorkerRunner {
    fn submit_plugin_install(&mut self, request: PluginInstallRequest) -> bool {
        if self.post(&Command::ValidatePlugins(request.clone())).is_err() { return false; }
        self.pending_install = Some(request);
        true
    }

    fn poll_plugin_install(&mut self) -> Option<PluginInstallReply> {
        self.drain();
        self.plugin_install_buf.pop_front()
    }

    fn sync_plugins(&mut self, revision: u64, fetch: &mut dyn FnMut() -> serde_json::Value) {
        if self.sent_plugin_revision != Some(revision) {
            self.post(&Command::SetPlugins { revision, packages: fetch() })
                .unwrap_or_else(|e| panic!("Failed to provision plugin worker: {e:?}"));
            self.sent_plugin_revision = Some(revision);
        }
    }

    fn submit_plugin_action(&mut self, request: PluginActionRequest) -> bool {
        self.post(&Command::PluginAction(request)).is_ok()
    }

    fn poll_plugin_action(&mut self) -> Option<PluginActionReply> {
        self.drain();
        self.plugin_action_buf.pop_front()
    }

    fn submit_run(&mut self, request: HistoryRequest, generation: u64) {
        // Stamp the revision NOW: `sync_parts_library` ran immediately before
        // this call, so this is the library the worker is known to hold for
        // this run — a park must not later post claiming a newer one.
        let revision = self.sent_library_revision.unwrap_or(0);
        if self.in_flight {
            // A run is already on the worker — PARK this one (latest wins) so the
            // worker never runs a stale intermediate from a drag backlog. `in_flight`
            // false always implies `pending_run` is `None` (drain / reset keep this).
            self.pending_run = Some((request, generation, revision));
        } else {
            self.post_run(request, generation, revision);
        }
    }

    fn sync_parts_library(
        &mut self,
        revision: u64,
        fetch: &mut dyn FnMut() -> brep_render::brep_kernel::PartsLibraryMap,
    ) {
        if self.sent_library_revision == Some(revision) {
            return;
        }
        // The one place the whole library crosses to the worker. It is posted
        // only when the library CHANGES (insert, document load, refresh) — the
        // per-run post this replaces stringified every embedded part payload on
        // the main thread for every edit.
        if self
            .post(&Command::SetPartsLibrary {
                library: fetch(),
                revision,
            })
            .is_ok()
        {
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
        // Queries are not coalesced; post immediately. (A query posted while a run is
        // parked in `pending_run` answers against the worker's CURRENT geometry; the
        // next applied run clears any pending query main-side, so this is benign.)
        let _ = self.post(&Command::Query(query));
    }

    fn poll_query(&mut self) -> Option<MeasureReply> {
        self.drain();
        self.query_buf.pop_front()
    }

    fn submit_mesh_import(&mut self, request: MeshImportRequest) {
        let _ = self.post(&Command::MeshImport(request));
    }

    fn poll_mesh_import(&mut self) -> Option<MeshImportReply> {
        self.drain();
        self.mesh_import_buf.pop_front()
    }

    fn submit_step_probe(&mut self, request: StepProbeRequest) {
        let _ = self.post(&Command::StepProbe(request));
    }

    fn poll_step_probe(&mut self) -> Option<StepProbeReply> {
        self.drain();
        self.step_probe_buf.pop_front()
    }

    fn submit_topology(&mut self, request: TopologyRequest) {
        // Not coalesced, like a query. One posted while a run is parked answers
        // against the worker's CURRENT registry; a handle that run frees is
        // simply absent from the reply, and the engine asks again after the
        // run lands.
        let _ = self.post(&Command::Topology(request));
    }

    fn poll_topology(&mut self) -> Option<TopologyReply> {
        self.drain();
        self.topology_buf.pop_front()
    }

    fn submit_sheet_lines(&mut self, request: SheetLinesRequest) -> bool {
        // The engine keeps ONE pass on the worker at a time and parks the
        // rest (latest per placement), so nothing is coalesced here. A pass
        // posted behind a run answers against the registry that run leaves;
        // a handle it freed, or re-posed, comes back refused and the engine
        // draws that pass on the main thread.
        self.post(&Command::SheetLines(request)).is_ok()
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

    /// TERMINATE the worker mid-computation — the only way to stop a
    /// single-threaded agent that is busy inside a feature (a posted message
    /// would queue behind the run) — and spawn a fresh one on the SAME reply
    /// pump. Everything the old worker owned is gone with it: the resident
    /// registry, the warm history cache, the installed parts library, the
    /// parked run. The bookkeeping is reset exactly as `reset` does it; the
    /// inbox is emptied too, since a reply the old worker posted just before
    /// dying may already be queued there.
    fn cancel(&mut self) -> bool {
        self.worker.terminate();
        self.worker = spawn_worker();
        self.worker
            .set_onmessage(Some(self._onmessage.as_ref().unchecked_ref()));
        self.inbox.borrow_mut().clear();
        self.run_buf.clear();
        self.query_buf.clear();
        self.mesh_import_buf.clear();
        self.step_probe_buf.clear();
        self.topology_buf.clear();
        self.sheet_lines_buf.clear();
        self.progress_buf.clear();
        self.pending_run = None;
        self.in_flight = false;
        self.sent_library_revision = None;
        self.library_requested = false;
        self.sent_plugin_revision = None;
        self.plugin_action_buf.clear();
        // Validation belongs to the installed library, not the cancelled document run.
        if let Some(request) = self.pending_install.clone() {
            if self.post(&Command::ValidatePlugins(request.clone())).is_err() {
                self.pending_install = None;
                self.plugin_install_buf.push_back(PluginInstallReply { id: request.id, result: Err("Worker restart could not resume plugin validation".into()) });
            }
        }
        true
    }

    fn reset(&mut self) {
        let _ = self.post(&Command::Reset);
        // A wholesale document switch: drop any buffered/parked work from the old
        // model (the generation gate in `EngineState::pump` also protects) and clear
        // the in-flight flag so the next run posts immediately.
        self.run_buf.clear();
        self.query_buf.clear();
        self.mesh_import_buf.clear();
        self.step_probe_buf.clear();
        self.topology_buf.clear();
        self.sheet_lines_buf.clear();
        self.progress_buf.clear();
        self.pending_run = None;
        self.in_flight = false;
        // `Command::Reset` clears the worker's kernel store, library included.
        self.sent_library_revision = None;
        self.library_requested = false;
        self.sent_plugin_revision = None;
        self.plugin_action_buf.clear();
    }
}

// ===========================================================================
// worker_entry — the WORKER side (CPU only; never touches wgpu/canvas). Owns a
// PERSISTENT SceneRunner so its delta baseline + this worker's thread-local kernel
// registry survive across messages, and answers each posted `Command` through the
// SHARED `process_command`, posting back the JSON `Reply` it produces.
// ===========================================================================

/// The web worker's entry point — invoked once from `worker.js` after `init()`.
/// Installs an `onmessage` loop over the worker's global scope: each posted
/// [`Command`] (a JSON string) runs through
/// [`brep_render::runner::process_command`] against a persistent
/// [`SceneRunner`](brep_render::pipeline::SceneRunner), and any [`Reply`] is posted
/// back as JSON. The closure is `forget()`-ten so it lives for the worker's life.
#[wasm_bindgen]
pub fn worker_entry() {
    brep_render::brep_kernel::panic_hook::set_once();

    let scope: web_sys::DedicatedWorkerGlobalScope = js_sys::global().unchecked_into();
    let scope_reply = scope.clone();

    // The resident runner: its `last_sent` delta baseline AND this worker's
    // thread-local kernel history cache persist across messages, so incremental
    // reruns replay exactly as the native thread runner does.
    let runner = Rc::new(RefCell::new(brep_render::pipeline::SceneRunner::new()));

    let onmessage = Closure::wrap(Box::new(move |event: MessageEvent| {
        let Some(text) = event.data().as_string() else {
            return;
        };
        let command = match serde_json::from_str::<Command>(&text) {
            Ok(command) => command,
            Err(error) => {
                log::error!("worker command parse failed: {error}");
                return;
            }
        };
        let reply = {
            let mut resident = runner.borrow_mut();
            // Progress goes out from INSIDE the run: a worker's `postMessage`
            // is delivered to the main thread while the worker stays busy, so
            // the UI can name the feature it is waiting on. Never asks the
            // run to stop — cancel terminates this worker instead.
            let mut progress = |report: RunProgress| {
                let json = serde_json::to_string(&Reply::Progress(report))
                    .expect("serialize worker progress");
                let _ = scope_reply.post_message(&JsValue::from_str(&json));
                true
            };
            brep_render::runner::process_command(&mut resident, command, &mut progress)
        };
        if let Some(reply) = reply {
            let json = serde_json::to_string(&reply).expect("serialize worker reply");
            let _ = scope_reply.post_message(&JsValue::from_str(&json));
        }
    }) as Box<dyn FnMut(MessageEvent)>);
    scope.set_onmessage(Some(onmessage.as_ref().unchecked_ref()));
    // Keep the handler alive for the worker's whole life (the worker owns it now).
    onmessage.forget();
}
