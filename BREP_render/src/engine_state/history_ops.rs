use super::*;

// --- Scene feed (R10) -------------------------------------------------

impl EngineState {
    /// Run a whole history and reconcile the display scene (R10 incremental):
    /// reused solids keep their buffers, the rest re-tessellate. Returns the build
    /// report JSON (`{featureErrors, unresolved, displayErrors}`). Marks dirty.
    ///
    /// A SCENE-ONLY one-shot: it does not touch the engine-owned history document
    /// and does not run the [`Self::finish_apply`] tail, so it also does not paint
    /// model colours. Callers that need those use [`Self::set_history_json`].
    pub fn run_history_json(&mut self, request_json: &str) -> Result<String, String> {
        let request: HistoryRequest = serde_json::from_str(request_json)
            .map_err(|error| format!("history request parse: {error}"))?;
        let report = self.run_plugin_scene(&request)?;
        self.dirty = true;
        Ok(with_approximations(
            with_refusals(
                serde_json::json!({
                    "featureErrors": report.feature_errors,
                    "featureNotes": report.feature_notes,
                    "featureFulfilment": fulfilment_map(&report),
                    "unresolved": report.unresolved,
                    "displayErrors": report.display_errors,
                }),
                &report,
            ),
            &report,
        )
        .to_string())
    }

    /// (Re)run the current rolled-to prefix of the engine's history through the
    /// SAME kernel pipeline and reconcile the display scene. Stores + returns the
    /// build report JSON.
    pub(super) fn rerun_history(&mut self) -> String {
        self.rerun_history_forcing(None)
    }

    pub(super) fn rerun_history_forcing(&mut self, feature_id: Option<&str>) -> String {
        if self.defer_plugin_run() { return self.history_report.clone(); }
        // This run publishes whatever a symbol edit did to the PORT features,
        // so `pump` need not run again for it.
        self.history.take_ports_followed();
        // Keep the incremental cache when rolling or editing: producer entries
        // own their handles, so rolling before a consuming boolean can reuse its
        // inputs. Document switches clear the cache in `set_history_json`.
        // Tag each submission with a generation; `pump` applies completed deltas
        // immediately for the inline runner or on a later frame for workers.
        // A transient preview replaces what the run evaluates, in the request
        // only: the history keeps its own source.
        self.drop_stale_expression_preview();
        let request_value = self.run_request_value();
        match serde_json::from_value::<HistoryRequest>(request_value) {
            Ok(mut request) => {
                if let Some(id) = feature_id {
                    request.force_rebuild.push(id.to_string());
                }
                // Carry the live display LOD to the runner (the request is the run
                // boundary the thread/worker receives). The runner re-tessellates
                // every resident mesh when this differs from its last run's lod.
                request.display_lod = self.settings.lod_factor;
                self.run_generation += 1;
                // A new run supersedes a cancelled one's notice.
                self.cancelled_run = None;
                // The PARTS-LIBRARY CHANNEL. A background runner (native
                // thread / browser worker) owns its own kernel store, so it
                // needs the library — but sending it WITH every run meant
                // stringifying every embedded part payload on the UI thread
                // for every edit, which froze the browser on an imported STEP
                // assembly. It is sent only when it CHANGES; `fetch` does not
                // run in the steady state. Inline shares this thread's store
                // and no-ops.
                self.runner.sync_parts_library(
                    brep_kernel::parts_library_revision(),
                    &mut brep_kernel::parts_library_map,
                );
                self.sync_plugin_runner();
                self.runner.submit_run(request, self.run_generation);
            }
            // A parse failure runs nothing: clear the surfaced frames/profiles (so
            // stale construction geometry does not linger — the scene keeps its
            // previous solids) and set an error report, then run the shared
            // post-apply tail synchronously (no kernel work), so this branch shares
            // the dirty/gizmo/overlay continuation verbatim with a real apply.
            Err(error) => {
                self.construction_frames.clear();
                self.sketch_profiles.clear();
                self.sketch_paths.clear();
                self.sketch_points.clear();
                self.sketch_axes.clear();
                self.wire_harness_report = None;
                self.ports_report = None;
                self.finish_apply(
                    serde_json::json!({ "error": format!("history request: {error}") }).to_string(),
                );
            }
        }
        // Inline applies the submitted run NOW; a thread impl would defer it to a
        // later frame's `pump`. Either way `history_report` is fresh once the reply
        // is applied — for Inline that is before this call returns.
        self.pump();
        self.history_report.clone()
    }

    /// Drain every completed run reply and APPLY it — the POLL/APPLY half of the
    /// M2a seam. Called from [`rerun_history`](Self::rerun_history) for the Inline
    /// runner's immediate apply, and once per frame from the app so a future async
    /// runner's completed runs land on the main thread. A reply older than
    /// [`applied_generation`](Self::applied_generation) (a newer run that finished
    /// first) is dropped.
    pub fn pump(&mut self) {
        self.pump_plugin_installations();
        // The runner REFUSED a run because its resident parts library could not
        // serve it (see `Reply::NeedPartsLibrary`). It has already forgotten
        // its copy, so re-running re-installs the library and re-submits. This
        // is a real path, not just a tripwire: the kernel's orphan GC drops
        // entries at the end of every run, so an undo to zero components empties
        // the RUNNER's store while this side's (which never ran) keeps
        // everything — no revision bookkeeping can see that, only the runner's
        // content preflight can. `library_resync` breaks the rerun→pump→rerun
        // recursion (the reinstall makes the second attempt succeed, but a
        // guard beats relying on that). The guard is tested FIRST because
        // `poll_library_request` CONSUMES the flag — polling it while a resync
        // is already in flight would swallow a second refusal.
        if !self.library_resync && self.runner.poll_library_request() {
            self.library_resync = true;
            self.rerun_history();
            self.library_resync = false;
        }
        // A symbol edit relabelled, added or removed a PORT feature. The eCAD
        // editors write their blocks straight into the history, which does not
        // run it, so the run happens here, on the next frame, for every host.
        if self.history.take_ports_followed() {
            self.rerun_history();
        }
        while let Some(reply) = self.runner.poll_mesh_import() {
            // A document switch clears this set. Ignore any older reconstruction
            // reply that was already running when its Reset crossed the queue.
            let Some(destination) = self.pending_mesh_imports.remove(&reply.id) else {
                continue;
            };
            if destination == MeshImportDestination::Preview {
                self.mesh_preview_results.push_back(reply);
                continue;
            }
            match reply.result {
                Ok(output) => match self.import_step_feature(&output.step_text) {
                    Ok(_) => {
                        self.push_notice(
                            "RANSAC reconstruction complete; building imported CAD body",
                        );
                        // A body the chain refused is left out of the model;
                        // say so, by name, rather than let it vanish.
                        if let Some(notice) = crate::runner::refused_bodies_notice(&output.report)
                        {
                            self.push_notice(notice);
                        }
                    }
                    Err(error) => self.push_notice(format!("mesh import failed: {error}")),
                },
                Err(error) => self.push_notice(format!("mesh import failed: {error}")),
            }
        }
        // STEP probes: the parse ran on the runner; stash the structure it
        // found (the import consumes it) and queue the outcome for the panel.
        while let Some(reply) = self.runner.poll_step_probe() {
            if !self.pending_step_probes.remove(&reply.id) {
                continue; // cancelled, or a document switch — nobody is waiting
            }
            let outcome = match reply.result {
                Ok(Some(assembly)) => {
                    let probe = super::model_io::probe_counts(&assembly);
                    // The kernel already refuses a structure that reaches no
                    // geometry, so this is belt-and-braces: an assembly with
                    // zero instances would import as zero components, which
                    // is the silent failure the structured lane forbids.
                    if probe.instances == 0 {
                        super::StepProbeOutcome::Flat
                    } else {
                        self.pending_step_assembly = Some(assembly);
                        super::StepProbeOutcome::Structure(probe)
                    }
                }
                Ok(None) => super::StepProbeOutcome::Flat,
                Err(error) => super::StepProbeOutcome::Failed(error),
            };
            self.step_probe_results.push_back((reply.id, outcome));
        }
        // The in-flight run's latest progress report (a background runner
        // posts one before each feature it executes). Kept only for a run
        // newer than the applied one: a report from a superseded run — or one
        // still arriving after a cancel bumped the generations — is stale.
        if let Some(progress) = self.runner.poll_progress() {
            if progress.generation > self.applied_generation {
                self.run_progress = Some(progress);
            }
        }
        self.pump_plugin_callbacks();
        while let Some(reply) = self.runner.poll_run() {
            self.runs_replied += 1;
            if reply.generation >= self.run_generation && self.accept_plugin_run(&reply) {
                self.applied_generation = reply.generation;
                self.apply_run_output(reply.output);
            } else {
                // Superseded — but the runner's baseline has moved past every
                // display in it, so a later keep can name one of them. Hold
                // them for that keep (see `superseded_displays`).
                for (name, _handle, maybe) in reply.output.snapshot {
                    if let Some(display) = maybe {
                        self.superseded_displays.insert(name, display);
                    }
                }
            }
        }
        if !self.run_pending() {
            self.run_progress = None;
        }
        // Deferred one-shot framing (Import / Open): frame the scene the moment the
        // run they submitted has fully landed. Consumed unconditionally once the run
        // is no longer pending — even when it produced no solids (bbox empty →
        // `zoom_to_fit` no-ops) — so a later unrelated run never inherits a stale fit.
        if self.pending_fit && !self.run_pending() {
            self.pending_fit = false;
            self.zoom_to_fit();
        }
        // Drain any completed measurement replies too (a background runner surfaces
        // them a frame after selection); for Inline this is a no-op each frame since
        // `object_info_json` already pumped its own query same-call.
        self.pump_queries();
        self.pump_topology();
        self.pump_sheet_lines();
    }

    /// Ask the runner for the B-rep topology of the displayed solids `names`
    /// that the scene does not hold for their current handle and that have not
    /// been asked for since the last applied run, then drain whatever has
    /// already answered (all of it, for the synchronous Inline runner). The
    /// drawing sheet's door, and the balloon bubble drag's (which re-projects
    /// the arrow head on the exact solid, `pmi_reproject_balloon_head`):
    /// nothing else needs topology on this side, so a document with neither a
    /// sheet nor a dragged balloon never asks.
    pub fn request_exact_solids<'a>(&mut self, names: impl IntoIterator<Item = &'a str>) {
        let mut wanted: Vec<(String, u32)> = Vec::new();
        for name in names {
            let Some(display) = self.scene.solid(name) else { continue };
            let handle = display.source_handle;
            if handle == 0 || display.is_sketch || self.scene.exact_solid(name).is_some() {
                continue;
            }
            if self.topology_asked.insert(handle) {
                wanted.push((name.to_string(), handle));
            }
        }
        if !wanted.is_empty() {
            self.next_topology_id += 1;
            let id = self.next_topology_id;
            self.pending_topology.insert(id);
            self.runner.submit_topology(crate::runner::TopologyRequest { id, solids: wanted });
        }
        self.pump_topology();
    }

    /// Fold every answered topology request into the scene. A reply for a
    /// display that has moved on to another handle is dropped; the next
    /// projection asks for the new one.
    pub fn pump_topology(&mut self) {
        let mut landed = false;
        while let Some(reply) = self.runner.poll_topology() {
            if !self.pending_topology.remove(&reply.id) {
                continue; // cancelled, or a document switch
            }
            for (name, handle, solid) in reply.solids {
                self.scene.set_exact_solid(&name, handle, solid);
            }
            landed = true;
        }
        if landed {
            self.pmi_refresh_stale_balloon();
        }
    }

    /// Whether a topology request is still in flight — the frame loop and the
    /// idle contract wait on it like a measurement query.
    pub fn topology_pending(&self) -> bool {
        !self.pending_topology.is_empty()
    }

    /// Whether a measurement query is still in flight (its reply not yet drained) —
    /// the query analogue of [`run_pending`](Self::run_pending), so the app keeps the
    /// frame loop alive until a background runner's measurement lands and displays.
    /// Always `false` for the synchronous Inline runner.
    pub fn queries_pending(&self) -> bool {
        !self.pending_query.is_empty()
    }

    /// Whether RANSAC reconstruction is still executing on the native runner
    /// thread or browser worker.
    pub fn mesh_imports_pending(&self) -> bool {
        !self.pending_mesh_imports.is_empty()
    }

    /// Whether a submitted run has not yet been applied (`run_generation !=
    /// applied_generation`). Always `false` for the synchronous Inline runner
    /// (submit → immediate `pump` keeps the two in lockstep); a background runner
    /// uses it to keep the frame loop alive until its reply lands.
    pub fn run_pending(&self) -> bool {
        self.run_generation != self.applied_generation || self.plugin_work_pending()
    }

    /// The generation of the last APPLIED run — bumps once per applied history
    /// run (document loads, edits, constraint mutations, solves). A cheap
    /// staleness key for app-side caches derived from the applied document
    /// (the update-components outdated badge keys on it).
    pub fn applied_generation(&self) -> u64 {
        self.applied_generation
    }

    /// What the in-flight run is executing right now, as far as the runner
    /// has reported (see [`crate::runner::RunProgress`]); `None` when nothing
    /// is running or the run has not reached its first executed feature.
    pub fn run_progress(&self) -> Option<&crate::runner::RunProgress> {
        self.run_progress.as_ref()
    }

    /// The feature id the last cancelled run was executing (empty when it was
    /// cancelled before any progress arrived), until the next submit.
    pub fn cancelled_run(&self) -> Option<&str> {
        self.cancelled_run.as_deref()
    }

    /// CANCEL the in-flight run. The runner abandons its work and comes back
    /// with an EMPTY resident registry (a fresh thread / a fresh worker — see
    /// [`crate::runner::HistoryRunner::cancel`]), so this side forgets
    /// everything that was waiting on it: the run itself (generations are
    /// bumped past it, so a straggling reply from the old runner is dropped
    /// as stale), pending measurement queries, document-bound mesh imports
    /// and a deferred fit. The DISPLAY SCENE is left as the last applied run
    /// built it — the document is ahead of it now, which the notice says; the
    /// next edit re-runs the whole history through the new runner (a cold
    /// run: the warm cache went with the old one). Nothing inside a feature
    /// is interruptible, so the native thread keeps burning CPU until the
    /// feature it is on finishes; the browser worker is terminated outright.
    ///
    /// `false` when nothing was running, or the runner cannot abandon (the
    /// synchronous Inline runner, whose runs are over before anyone can ask).
    pub fn cancel_run(&mut self) -> bool {
        self.cancel_run_with_notice(true)
    }

    pub(super) fn cancel_run_with_notice(&mut self, notify: bool) -> bool {
        if self.plugin_installation_pending() { return false; }
        if self.cancel_plugin_action() { return true; }
        if !self.run_pending() || !self.runner.cancel() {
            return false;
        }
        let stalled_on = self.run_progress.take().map(|progress| progress.feature_id);
        // Past every generation submitted so far: a reply the old runner
        // already posted (sitting in the main event loop on wasm) carries an
        // older number than this and is dropped by `pump`'s gate.
        self.run_generation += 1;
        self.applied_generation = self.run_generation;
        self.pending_query.clear();
        // The new runner's registry counts handles from one again, so every
        // topology keyed by an old handle could alias a new display.
        self.pending_topology.clear();
        self.topology_asked.clear();
        self.scene.clear_exact_solids();
        self.sheet_lines.forget();
        // The old runner's held displays name ITS handles; the new one's
        // first run emits every display afresh.
        self.superseded_displays.clear();
        let dropped_imports = self.pending_mesh_imports.len();
        self.pending_mesh_imports.clear();
        let dropped_probes = self.pending_step_probes.len();
        self.pending_step_probes.clear();
        self.pending_fit = false;
        self.library_resync = false;
        self.cancelled_run = Some(stalled_on.clone().unwrap_or_default());
        if notify {
            self.push_notice(match stalled_on {
                Some(id) if !id.is_empty() => format!(
                    "Run cancelled while executing {id}. The model shows the last completed \
                     result; edit or delete the feature to rebuild."
                ),
                _ => "Run cancelled. The model shows the last completed result; the next edit \
                      rebuilds it."
                    .to_string(),
            });
        }
        if dropped_imports > 0 {
            self.push_notice(format!(
                "cancelled {dropped_imports} pending mesh import{}",
                if dropped_imports == 1 { "" } else { "s" }
            ));
        }
        if dropped_probes > 0 {
            self.push_notice("cancelled the STEP file being read — upload it again to import it");
        }
        true
    }

    /// How many run replies the runner has sent back (see `runs_replied`).
    pub fn runs_replied(&self) -> u64 {
        self.runs_replied
    }

    /// Whether the display scene currently holds at least one solid. Used by the
    /// app's async-safe first-frame framing: under a background runner (thread /
    /// worker) the seed run lands a frame (or many) after boot, so the shell waits
    /// for `has_solids() && !run_pending()` before its one-shot `zoom_to_fit`.
    pub fn has_solids(&self) -> bool {
        !self.scene.solids().is_empty()
    }

    /// Swap in a different history runner (the platform injects its own — the native
    /// app installs a [`ThreadRunner`](crate::runner::ThreadRunner); wasm keeps the
    /// default Inline until M3's worker). Resets the new runner's delta baseline so
    /// the next run rebuilds fully. Call BEFORE seeding a document so the seed builds
    /// through the installed runner.
    pub fn set_runner(&mut self, runner: Box<dyn crate::runner::HistoryRunner>) {
        self.runner = runner;
        self.plugin_runner_replaced();
        self.runner.reset();
        self.superseded_displays.clear();
        self.pending_mesh_imports.clear();
        self.mesh_preview_results.clear();
        self.pending_topology.clear();
        self.topology_asked.clear();
        self.scene.clear_exact_solids();
        self.sheet_lines.forget();
    }

    /// Apply a [`SceneRunner`](crate::pipeline::SceneRunner) delta to the display
    /// scene and build the history report JSON — the APPLY half of the M2a seam.
    ///
    /// Reconcile preserving ORDER + reuse: MOVE every current display out of the
    /// scene ([`RenderScene::drain`](crate::scene::RenderScene::drain)) into a
    /// name-keyed `kept` map, then reinsert in snapshot order — a fresh entry
    /// (`Some`) replaces, an UNCHANGED entry (`None`) reuses its moved-out display
    /// (stable `revision` ⇒ GPU-buffer reuse, Task-1; its `source_handle` equals
    /// the run's handle by the monotonic-handle reuse invariant). Leftovers in
    /// `kept` — departed kernel solids AND the previous run's sketch sheets — are
    /// dropped; `refresh_committed_sketches` (run in the shared continuation after)
    /// re-adds the sheets, so dropping them here is correct.
    ///
    /// Then the report continuation (identical to the pre-seam run): keep the run's
    /// resolved frames + solved sketch profiles and fold the per-feature timings /
    /// output-names into the id-keyed report JSON, then hand it to
    /// [`finish_apply`](Self::finish_apply) — the shared dirty/gizmo/overlay tail
    /// that the parse-error branch in [`rerun_history`](Self::rerun_history) also
    /// calls, so both paths share the continuation verbatim.
    pub(super) fn apply_run_output(&mut self, output: crate::pipeline::RunOutput) {
        let crate::pipeline::RunOutput {
            snapshot,
            mut report,
            provenance,
            entity_origin,
            assembly_poses,
            assembly_fixed,
            moved_solids,
            imported_colors,
            consumed,
            assembly,
        } = output;
        for (id, data) in &report.plugin_persistent_data {
            self.history.fold_plugin_persistent_data(id, data.clone());
        }
        // The runner already forced fresh displays for the solver-moved solids
        // (their snapshot entries arrive `Some`); nothing extra to do main-side.
        let _ = moved_solids;
        // Un-pose the active PMI view's exploded displays BEFORE the reconcile
        // keeps them, so the re-pose after apply starts from the modeling
        // pose and never compounds.
        self.pmi_restore_explode();

        // The names this run consumed, for the committed-sketch refresh in the
        // tail below — the runner folded them, so the UI thread does not execute
        // the history a second time to fold them again.
        self.consumed_names = consumed.into_iter().collect();
        // Adopt the run's assembly tail. The PARTS LIBRARY comes out of it here
        // rather than in `adopt_assembly_sync`, so the (potentially megabyte)
        // store is installed and mirrored into the document exactly once, on the
        // run that moved it, and never kept in the snapshot afterwards.
        self.assembly_sync = assembly.map(|mut sync| {
            if let Some(library) = sync.parts_library.take() {
                // CLEAR, then install. `install_parts_library` keeps a resident
                // entry whose identity (source key + signature + document hash)
                // matches the incoming one VERBATIM — the right rule for the
                // main → runner direction it was written for, because the
                // runner may hold a heal the sender never saw. This is the
                // other direction: the run that just healed an entry (a heal
                // rewrites the snapshot and deliberately does NOT change the
                // identity) is the one holding the newer copy, so keeping the
                // resident one would silently drop it and the document would
                // re-heal on every open. Emptying first makes every incoming
                // entry a fresh insert, i.e. "the store IS what the run left".
                brep_kernel::install_parts_library(&Default::default());
                brep_kernel::install_parts_library(&library);
                if let Ok(block) = serde_json::to_value(&library) {
                    self.history.set_parts_library(block);
                }
            }
            sync
        });

        // --- assembly pose-authority fold -------- Adopt the solver's pose /
        // isFixed write-backs into the owning ACOMP features by
        // `inputParams.id` BEFORE anything persists or re-runs this document.
        // Deliberately NOT `update_feature_params`: the fold must not mint an
        // undo entry nor trigger a rerun (rerun → solve → fold → rerun would
        // loop); `fold_param_no_undo` writes the param silently, and the next
        // run's request simply carries the solved pose (a no-motion solve emits
        // no updates, so fingerprints never churn).
        for (id, pose) in assembly_poses {
            let expression_driven = self.history.index_of(&id).and_then(|i| self.history.feature_params(i))
                .is_some_and(|p| super::batch::contains_expression(&p["transform"]));
            if !expression_driven { self.history.fold_param_no_undo(&id, "transform", pose); }
        }
        for (id, fixed) in assembly_fixed {
            self.history
                .fold_param_no_undo(&id, "isFixed", serde_json::Value::Bool(fixed));
        }

        // Adopt the run's eager provenance (SOLID last-writer) + entity origin
        // (face/edge first-writer) wholesale (both drive `creating_feature` + the
        // Info tab's `creatingFeature` with no cold re-run), and INVALIDATE the
        // object-info measurement cache + any in-flight query: the geometry changed,
        // so cached measurements are stale and a pending reply is superseded (a
        // re-selection re-queries against the fresh geometry).
        self.provenance = provenance.into_iter().collect();
        self.entity_origin = entity_origin.into_iter().collect();
        self.info_cache.clear();
        self.pending_query.clear();

        // Fold the run's IMPORTED COLOURS into the engine's own metadata store —
        // the seam between the kernel's (thread-local, never persisted by us)
        // name-keyed store and the one the Info window edits and the document
        // saves. NON-overwriting on purpose: an import re-stamps its colour on
        // every replay, and a colour the user changed in the panel must win.
        for (name, hex) in imported_colors {
            if self.metadata.attribute(&name, "color").is_none() {
                self.metadata.set_attribute(&name, "color", &hex);
            }
        }

        // Reconcile the scene: move current displays out, reinsert in order.
        let mut kept: std::collections::HashMap<String, crate::scene::SolidDisplay> = self
            .scene
            .drain()
            .into_iter()
            .map(|solid| (solid.name.clone(), solid))
            .collect();
        for (name, handle, maybe) in snapshot {
            match maybe {
                Some(display) => {
                    self.superseded_displays.remove(&name);
                    self.scene.insert_solid(display);
                }
                // "Unchanged": the runner last emitted `handle` for this name.
                // That emit is the scene's display when its reply was applied,
                // and a held superseded display when it was dropped — the
                // handle says which, since handles are monotonic and never
                // recycled within one runner. Neither holding it is a protocol
                // breach (a reply lost between the runner and `pump`), and the
                // app is not the place to crash on one, in any build: the
                // solid is left out of this frame and the report names it.
                None => {
                    let kept_display = kept.remove(&name);
                    let display = match kept_display {
                        Some(display) if display.source_handle == handle => Some(display),
                        _ => self
                            .superseded_displays
                            .remove(&name)
                            .filter(|display| display.source_handle == handle),
                    };
                    match display {
                        Some(display) => self.scene.insert_solid(display),
                        None => {
                            report.display_errors.push(format!(
                                "{name}: the run kept resident handle {handle}, which no reply has displayed"
                            ));
                        }
                    }
                }
            }
        }
        // Topology is kept only for a display whose handle survived the run,
        // and so is the record of what was asked: a surviving handle is in
        // flight, answered or not in the registry, and every other is gone.
        self.scene.prune_exact_solids();
        let live: std::collections::HashSet<u32> =
            self.scene.solids().iter().map(|solid| solid.source_handle).collect();
        self.topology_asked.retain(|handle| live.contains(handle));

        // Keep every plane frame the run resolved (DATUM/PLANE/SKETCH);
        // `refresh_construction_datums` filters to the D/P producers.
        self.construction_frames = report.frames.clone();
        // Keep every solved sketch profile so `refresh_committed_sketches` can
        // synthesize the committed sketch sheet solids.
        self.sketch_profiles = report.profiles.clone();
        // ...and every path chain, so a sketch whose geometry closes NO region (an
        // open chain — the reported single line) still has something to draw.
        self.sketch_paths = report.paths.clone();
        // ...and every published point, so a sketch holding ONLY points (a
        // hole-placement sketch) still has something to draw.
        self.sketch_points = report.points.clone();
        // Keep every axis line the run published so the angle gizmo can resolve a
        // revolve `axis` reference to a world line without re-running.
        self.sketch_axes = report.axes.clone();
        // ...and the wire-harness routing report, for the harness panel.
        self.wire_harness_report = report.wire_harness.clone();
        // ...and the ports tail's report, for the Qualify panel: the tail is not
        // a feature, so nothing else carries what it refused or could not resolve.
        self.ports_report = report.ports.clone();
        // ...and the PMI report (every view's resolved annotations).
        self.pmi_report = report.pmi.clone();
        self.fold_plugin_annotation_data();
        // Fold the per-feature timing / output-name pairs into id-keyed maps so the
        // history-tree UI can look them up by feature id.
        let timings: serde_json::Map<String, serde_json::Value> = report
            .feature_timings
            .iter()
            .map(|(id, ms)| (id.clone(), serde_json::json!(ms)))
            .collect();
        let outputs: serde_json::Map<String, serde_json::Value> = report
            .feature_outputs
            .iter()
            .map(|(id, names)| (id.clone(), serde_json::json!(names)))
            .collect();
        let report_json = with_approximations(
            with_refusals(
                serde_json::json!({
                    "featureErrors": report.feature_errors,
                    "featureNotes": report.feature_notes,
                    "featureFulfilment": fulfilment_map(&report),
                    "unresolved": report.unresolved,
                    "displayErrors": report.display_errors,
                    "featureTimings": timings,
                    "featureOutputs": outputs,
                }),
                &report,
            ),
            &report,
        )
        .to_string();
        self.finish_apply(report_json);
    }

    /// The shared post-apply TAIL: mark dirty, store the report JSON, re-sync an
    /// armed gizmo, and rebuild the persistent committed-sketch + construction-datum
    /// overlays. Called after a real run's [`apply_run_output`](Self::apply_run_output)
    /// AND from [`rerun_history`](Self::rerun_history)'s parse-error branch, so both
    /// paths run the identical continuation. Callers read the result via
    /// [`Self::history_report`](Self::history_report_json).
    fn finish_apply(&mut self, report_json: String) {
        self.dirty = true;
        self.history_report = report_json;
        // Keep an armed gizmo glued to its feature as the model rebuilds. During a
        // transform drag the re-sync is driven by `transform_drag_to` itself (which
        // resolves the delta against the frozen grab frame first, then syncs), so
        // skip it here to avoid a redundant double-feed per drag frame. Transform
        // mode re-feeds the widget frame; dimension mode re-projects the annotation
        // leaders onto the rebuilt (param-changed) geometry.
        if self.transform_gizmo.drag.is_none() {
            match self.transform_gizmo.mode {
                GizmoMode::Transform => self.sync_transform_gizmo(),
                GizmoMode::Dimension => self.refresh_feature_dimension_overlay(),
                GizmoMode::None => {}
            }
        }
        // Rebuild the persistent committed-sketch overlays against the reconciled
        // scene (also covers `set_history_json`, which returns this call's result).
        self.refresh_committed_sketches();
        // Rebuild the persistent construction datum/plane overlays from the frames
        // the run just surfaced (D/P features only; sketches render as curves).
        self.refresh_construction_datums();
        // Rebuild the BOARD's 3D bodies from the `pcb` block, for the same reason
        // the sketch sheets are rebuilt here: the reconcile above kept only the
        // run's own outputs. A document with no `pcb` block returns immediately
        // inside.
        self.refresh_board_geometry();
        // Assembly documents: adopt the component projection and the solved
        // constraint state the runner shipped, and fold that state back into the
        // document (the pose-authority contract — see `assembly_ops`).
        // Componentless documents return immediately inside.
        self.adopt_assembly_sync();
        // Component selection needs the new member projection above, not the
        // previous run's assembly. This also completes a deferred Move Copy.
        // Keep an armed COMPONENT Move gizmo glued to its (possibly re-solved)
        // component: re-anchor at the fresh member bbox. Never during its own
        // drag — the drag feed owns the widget frame (free-move live-follow).
        if self.component_move.drag.is_none() {
            self.component_move_sync();
        }
        // Rebuild the assembly-constraint viewport overlays from the rows the
        // same reply carried (an inert no-op — empty group — for a document with
        // no assembly state). ORDER MATTERS: the overlay read must follow the
        // adopt.
        self.refresh_constraint_overlay();
        // PMI: re-pose the active view's exploded solids on the fresh displays
        // and re-bake its annotation overlay from the run's report.
        self.pmi_after_apply();
        // Re-derive every display colour from the metadata store, LAST — after
        // the sketch sheets, the datum overlays and the assembly sync have all
        // settled the scene, so no display inserted above is missed. This is why
        // a model colour now survives a feature edit: the freshly tessellated
        // display arrives colourless and is re-coloured from the store, instead
        // of the colour living only on the display that was just thrown away.
        // A no-op when nothing changed, which is the common case.
        self.sync_colors_from_metadata();
    }

    /// Load a whole history document (a saved part file parses as one); the
    /// engine now OWNS this recipe. Rolls to the last feature and builds it.
    ///
    /// The document's top-level `metadata` field (the Properties-panel
    /// name-keyed store) is lifted out into [`Self::metadata`] before the feature
    /// list is handed to the kernel — loading a part REPLACES the store wholesale
    /// (a document with no `metadata` clears it), mirroring the previous metadata
    /// manager's load semantics. Round-trips with [`Self::history_request_json`].
    ///
    /// The top-level `workbench` field (the ACTIVE-WORKBENCH id the save embedded
    /// — see [`Self::history_request_json`]) is lifted off the kernel recipe the
    /// same way and applied through [`Self::apply_settings_json`] — the SAME seam
    /// the toolbar's workbench dropdown writes through — so the palette / context
    /// offers / workbench buttons react to a restored workbench exactly as they
    /// do to a manual switch (settings generation bump included). Tolerances:
    ///
    /// * a legacy document WITHOUT the field (or with a non-string value) leaves
    ///   the current workbench untouched — opening an old file never yanks the
    ///   user out of their workbench;
    /// * an unknown/stale id is stored RAW (never an error): the settings layer
    ///   deliberately doesn't validate ids, and every consumer resolves through
    ///   the app-side registry's `resolve()`, which falls back to the default
    ///   workbench — so a file saved by a build with a workbench this build
    ///   doesn't know still opens cleanly;
    /// * the restored id is deliberately NOT persisted to the settings blob —
    ///   that blob stays the user's boot preference; a document's workbench is
    ///   session-scoped (the next explicit dropdown change persists as usual).
    pub fn set_history_json(&mut self, request_json: &str) -> Result<String, String> {
        // A locked history (a PLM revision this user may not change) is not
        // replaced either: a fresh `History` would arrive unlocked, which is the
        // one door `History::lock` cannot hold by itself. Opening another
        // document is a new engine, and a reload after checkout unlocks first.
        if let Some(reason) = self.history.locked() {
            return Err(format!("this document is read-only: {reason}"));
        }
        // A document switch is a wholesale model replacement: drop the incremental
        // cache so the new model starts from a clean slate (no cross-document
        // staleness, no unbounded cache growth across many opens). The roll/edit
        // hot path (`rerun_history`) deliberately KEEPS the cache for instant
        // rollback; this is the ONE place the full clear belongs.
        brep_kernel::clear_history_cache();
        // Reset the delta runner's baseline in lockstep with the cache clear so the
        // new document is a FULL rebuild (no reuse against the prior model's names).
        self.runner.reset();
        self.superseded_displays.clear();
        self.pending_mesh_imports.clear();
        self.mesh_preview_results.clear();
        self.pending_topology.clear();
        self.topology_asked.clear();
        self.scene.clear_exact_solids();
        // A probed-but-unconsumed STEP assembly belongs to the document being
        // replaced: importing it into the NEW one would land a file the user never
        // chose here (and hold its solids resident until they did). A probe
        // still running is likewise the old document's: its answer is dropped.
        self.pending_step_assembly = None;
        self.pending_step_probes.clear();
        self.step_probe_results.clear();
        // Everything the previous document's run shipped belongs to THAT
        // document: the assembly tail, the component projection, its consumed
        // names, and the main-side session's generation stamp. The new
        // document's first reply replaces them; until it lands, nothing must
        // read the old one's.
        self.assembly_sync = None;
        self.assembly_components.clear();
        self.assembly_session_generation = None;
        self.consumed_names.clear();
        // The open sheet, the open sheet form and the projection cache all
        // name objects of the document being replaced.
        self.forget_sheets();
        // A preview belongs to the document being replaced.
        self.expression_preview = None;
        let mut document: serde_json::Value = serde_json::from_str(request_json)
            .map_err(|error| format!("history parse: {error}"))?;
        let thumbnail = document.as_object_mut().and_then(|o| o.remove("thumbnail"));
        self.thumbnail_cache.replace(None);
        // Pull `metadata` out of the document so the engine holds the single copy
        // (kept off the History recipe the kernel executes).
        let metadata_value = document
            .as_object_mut()
            .and_then(|object| object.remove("metadata"));
        self.metadata.load_json(metadata_value.as_ref());
        // Lift the saved active-workbench id off the kernel recipe (`metadata`'s
        // sibling — the kernel would ignore the extra field, but the engine owns
        // it) and apply it through the shared settings seam; see the doc comment
        // for the legacy/unknown-id tolerances. Applied BEFORE the rebuild below
        // so anything reading the settings post-run already sees the restored id.
        if let Some(workbench_id) = document
            .as_object_mut()
            .and_then(|object| object.remove("workbench"))
            .as_ref()
            .and_then(serde_json::Value::as_str)
        {
            let _ = self.apply_settings_json(
                &serde_json::json!({ "workbench": workbench_id }).to_string(),
            );
        }
        // Stamp each sketch's per-loop ids onto its geometries before the model
        // is built. Deriving already yields the right ids, so this renames
        // nothing — it PERSISTS them, which is what lets a loop keep its identity
        // when the edge the id came from is deleted. Doing it here (rather than
        // only on sketch commit) covers every document, including one built by a
        // script that never enters sketch mode. See the kernel's
        // `features/sketch/loop_ids`.
        stamp_sketch_loop_ids(&mut document);
        // SEED this thread's kernel parts library from the document's block.
        // The per-run request no longer carries the block (see
        // `History::parts_library`), so this explicit install is the ONE door
        // that seeds a loaded document — it is what `parts_library_json()`
        // (SAVE), the mutation-path session re-run and the export lanes all
        // resolve ACOMPs against, and it bumps the revision so the next run
        // hands the fresh library to the background runner. A run that heals or
        // GCs it ships the result back (`apply_run_output`).
        let library = document
            .get("partsLibrary")
            .and_then(|block| serde_json::from_value(block.clone()).ok())
            .unwrap_or_default();
        brep_kernel::install_parts_library(&library);
        self.history = History::from_request_json(&document.to_string())?;
        self.document_thumbnail.replace(thumbnail.map(|image| (self.history.edit_serial(), image)));
        Ok(self.rerun_history())
    }

    /// The whole history request document (persistence / debugging), with the
    /// engine-owned extras folded back in on top of the kernel recipe so
    /// save→open round-trips them:
    ///
    /// * `metadata` — the Properties-panel store, written only when non-empty so
    ///   an un-annotated model persists as before;
    /// * `workbench` — the CURRENT active-workbench id
    ///   (`self.settings.workbench`), ALWAYS written so a saved part reopens in
    ///   the workbench it was saved from (restored by
    ///   [`Self::set_history_json`]). Always-embed keeps the invariant simple:
    ///   the serialized field tracks the LIVE setting, never a stale stored copy
    ///   — the load lifts it off the kernel recipe entirely, so this is the ONE
    ///   place it is (re)written. Note the deliberate consequence: switching
    ///   workbench changes this document, so the file panel's dirty flag flips —
    ///   consistent with the `metadata` precedent, and semantically true now
    ///   that the workbench is part of the saved file.
    pub fn history_request_json(&self) -> String {
        let mut document: serde_json::Value =
            serde_json::from_str(&self.history.request_json())
                .unwrap_or_else(|_| serde_json::json!({}));
        if let Some(object) = document.as_object_mut() {
            if !self.metadata.is_empty() {
                object.insert("metadata".into(), self.metadata.to_json());
            }
            object.insert("thumbnail".into(), self.model_thumbnail());
            object.insert(
                "workbench".into(),
                serde_json::Value::String(self.settings.workbench.clone()),
            );
        }
        document.to_string()
    }

    /// Cache the small portable PNG by scene content, not by camera position.
    /// The picture is the DOCUMENT's, so it is rendered only while the scene is
    /// the document's model as built. A run the scene has not caught up with
    /// (an edit whose rebuild is still to come) gets an explicitly pending,
    /// empty image rather than a picture of the previous geometry; plugin work
    /// in flight is not that, since a candidate keeps its own scene until it
    /// commits. A transient presentation — an expression preview (a family
    /// member shown in the document's place) or an illustration — gets the
    /// document's own last picture (the one it was opened with, or the one its
    /// last save rendered) rather than what is on screen. A just-opened file
    /// keeps its embedded image while its unchanged recipe is being rebuilt.
    /// The document's picture is keyed by the history's edit serial, which
    /// moves on user edits, undo and redo only (a run's write-back ticks the
    /// revision). The same built model renders the same bytes every time.
    fn model_thumbnail(&self) -> serde_json::Value {
        let pending = self.run_generation != self.applied_generation();
        let transient = self.expression_preview.is_some() || self.illustration_snapshot.is_some();
        let mut key = vec![self.history.revision(), self.applied_generation(),
            self.scene.visibility_revision(), self.settings_generation, pending as u64, transient as u64];
        key.extend(self.scene.solids().iter().map(|s| s.revision));
        if let Some((old, image)) = self.thumbnail_cache.borrow().as_ref() {
            if *old == key { return image.clone(); }
        }
        let image = if pending || transient {
            self.document_thumbnail.borrow().as_ref().filter(|(serial, _)| *serial == self.history.edit_serial())
                .map(|(_, image)| image.clone())
                .unwrap_or_else(|| crate::thumbnail::empty_embedded("pending"))
        } else {
            let capture = crate::thumbnail::capture(&self.scene);
            let image = match crate::thumbnail::render_png(&capture, crate::thumbnail::SIZE) {
                Some(png) => crate::thumbnail::embedded_png(&png, crate::thumbnail::SIZE, "ready"),
                None => crate::thumbnail::empty_embedded("empty"),
            };
            self.document_thumbnail.replace(Some((self.history.edit_serial(), image.clone())));
            image
        };
        self.thumbnail_cache.replace(Some((key, image.clone())));
        image
    }

    /// The tree listing `{ step, features:[{index,type,id}] }` for the UI panel.
    pub fn history_listing_json(&self) -> String {
        self.history.listing_json()
    }

    /// The last build report JSON.
    pub fn history_report_json(&self) -> String {
        self.history_report.clone()
    }

    pub fn history_len(&self) -> usize {
        self.history.len()
    }

    /// The rolled-to (selected) feature index.
    pub fn history_rollback(&self) -> usize {
        self.history.rollback()
    }

    /// A monotonic counter of mutation opportunities on the history document —
    /// the cache key for anything derived from it. See [`crate::history::History::revision`].
    pub fn history_revision(&self) -> u64 {
        self.history.revision()
    }

    pub fn feature_type_at(&self, index: usize) -> Option<String> {
        self.history.feature_type(index)
    }

    pub fn feature_id_at(&self, index: usize) -> Option<String> {
        self.history.feature_id(index)
    }

    /// The `inputParams` document of feature `index` (`"null"` if none) — the
    /// dialog's editing-buffer source.
    pub fn feature_params_json(&self, index: usize) -> String {
        self.history
            .feature_params(index)
            .map(|v| v.to_string())
            .unwrap_or_else(|| "null".to_string())
    }

    /// Mint the id for a NEW feature: `{base}{N}` where `base` is the feature's
    /// shortName ([`crate::features::feature_short_name`]) and `N` is the part
    /// history's persistent GLOBAL counter (monotonic, never reused, round-trips
    /// save/load — see [`History::next_feature_id`]). `&mut` because the counter
    /// advances; if the caller's `add_feature` then fails the number is simply
    /// skipped (monotonic-with-gaps is the contract, not an error).
    pub fn next_feature_id(&mut self, base: &str) -> String {
        self.history.next_feature_id(base)
    }

    /// Adopt an edited document as one undo step and rebuild through the normal
    /// runner, preserving the tab's scene settings and history stack.
    pub fn edit_document_json(&mut self, document: &str) -> Result<String, String> {
        let value: serde_json::Value = serde_json::from_str(document).map_err(|e| e.to_string())?;
        let library = value.get("partsLibrary").map(|v| serde_json::from_value(v.clone()))
            .transpose().map_err(|e| format!("parts library: {e}"))?;
        self.history.adopt_document_checkpointed(document)?;
        // The runner's library channel reads the resident store. Updating only
        // History left it serving the old snapshot after Update Components.
        if let Some(library) = library {
            brep_kernel::install_parts_library(&library);
        }
        Ok(self.rerun_history())
    }

    /// Publish derived changes folded into the current user edit, without adding
    /// another undo step (for example PCB placement and its assembly pose).
    pub fn rebuild_current_history(&mut self) -> String {
        self.rerun_history()
    }

    /// Roll the model to feature `index`: re-run `features[0..=index]`.
    pub fn roll_to(&mut self, index: usize) -> String {
        self.history.set_rollback(index);
        self.rerun_history()
    }

    /// Replace feature `id`'s input params and re-run at the current rollback →
    /// the viewport updates live.
    pub fn update_feature_params(
        &mut self,
        id: &str,
        input_params_json: &str,
    ) -> Result<String, String> {
        let mut params: serde_json::Value = serde_json::from_str(input_params_json)
            .map_err(|e| format!("feature params parse: {e}"))?;
        let index = self
            .history
            .index_of(id)
            .ok_or_else(|| format!("no feature with id '{id}'"))?;
        self.stamp_face_transform_pivot(index, &mut params);
        self.history.set_feature_params(index, params);
        Ok(self.rerun_history())
    }

    /// Replace the `inputParams` of MANY features as ONE model edit — the
    /// [`Self::update_feature_params`] batch sibling ([`Self::add_features`] is
    /// the append-only one). ONE undo checkpoint and ONE history re-run for the
    /// whole set.
    ///
    /// The lane that needs it: a PACKED BOM row rolls up every occurrence whose
    /// occurrence data matches, so editing one of its cells writes the same key
    /// into N ACOMP features. Looping `update_feature_params` would cost N
    /// re-runs and — worse — N undo entries, so taking back one visible edit
    /// would need N presses of undo.
    ///
    /// Unknown ids are reported (the whole batch is refused before anything is
    /// written, so a typo can never half-apply); an empty batch is a no-op that
    /// neither checkpoints nor runs.
    pub fn update_many_feature_params(
        &mut self,
        edits: &[(String, serde_json::Value)],
    ) -> Result<String, String> {
        if edits.is_empty() {
            return Ok(self.history_report.clone());
        }
        let mut resolved: Vec<(usize, serde_json::Value)> = Vec::with_capacity(edits.len());
        for (id, params) in edits {
            let index = self
                .history
                .index_of(id)
                .ok_or_else(|| format!("no feature with id '{id}'"))?;
            let mut params = params.clone();
            self.stamp_face_transform_pivot(index, &mut params);
            resolved.push((index, params));
        }
        self.history.set_many_feature_params(&resolved);
        Ok(self.rerun_history())
    }

    /// Append a feature (a full `{type, inputParams, …}` descriptor) and roll to
    /// it. The caller assigns a unique `id` (see [`Self::next_feature_id`]).
    pub fn add_feature(&mut self, feature_json: &str) -> Result<String, String> {
        let feature: serde_json::Value =
            serde_json::from_str(feature_json).map_err(|e| format!("feature parse: {e}"))?;
        if !self.append_plugin_features(std::slice::from_ref(&feature))? {
            self.history.push_feature(feature);
        }
        let last = self.history.len().saturating_sub(1);
        self.history.set_rollback(last);
        Ok(self.rerun_history())
    }

    /// Append MANY features and roll to the last — [`Self::add_feature`]'s batch
    /// sibling, and the reason it exists: `add_feature` re-runs the WHOLE history
    /// per call, so a lane that appends N features by looping it costs N rebuilds
    /// (O(N²) work on an N-part STEP-assembly import). This pushes all of them,
    /// then re-runs ONCE — one rebuild, one undo checkpoint, one
    /// [`applied_generation`](Self::applied_generation) bump.
    ///
    /// Deliberately NOT [`Self::set_history_json`]: that is the document-SWITCH
    /// path (it clears the kernel history cache and resets the runner's delta
    /// baseline, forcing a full cold rebuild), which is the wrong mechanism for an
    /// append onto the live document.
    ///
    /// An empty batch is a no-op — no checkpoint, no run, no generation bump —
    /// and returns the standing report. The caller assigns each feature's unique
    /// `id` (see [`Self::next_feature_id`]).
    pub fn add_features(&mut self, features: &[serde_json::Value]) -> String {
        if features.is_empty() {
            return self.history_report.clone();
        }
        match self.append_plugin_features(features) {
            Ok(true) => {},
            Ok(false) => self.history.push_features(features.to_vec()),
            Err(error) => return serde_json::json!({"error":error}).to_string(),
        }
        let last = self.history.len().saturating_sub(1);
        self.history.set_rollback(last);
        self.rerun_history()
    }

    /// Delete the feature with id `id` (no-op if absent) and re-run, clamping the
    /// rolled-to step.
    pub fn delete_feature(&mut self, id: &str) -> String {
        match self.history.index_of(id) {
            Some(index) => self.delete_feature_at(index),
            None => self.rerun_history(),
        }
    }

    /// Delete the feature at `index` (no-op when out of range) and re-run,
    /// clamping the rolled-to step.
    ///
    /// The POSITIONAL twin of [`Self::delete_feature`], and the only way to
    /// remove a feature whose `inputParams` carry no `id` — a shape a hand-built
    /// history JSON can still contain, and one an id-keyed delete can never
    /// address.
    pub fn delete_feature_at(&mut self, index: usize) -> String {
        if index < self.history.len() {
            self.history.remove_feature(index);
            let step = self
                .history
                .rollback()
                .min(self.history.len().saturating_sub(1));
            self.history.set_rollback(step);
        }
        self.rerun_history()
    }

    /// Move feature `index` one slot up/down (reorder), keeping it selected.
    pub fn reorder_feature(&mut self, index: usize, up: bool) -> String {
        let len = self.history.len();
        if len >= 2 {
            let target = if up {
                index.checked_sub(1)
            } else if index + 1 < len {
                Some(index + 1)
            } else {
                None
            };
            if let Some(target) = target {
                self.history.swap(index, target);
                self.history.set_rollback(target);
            }
        }
        self.rerun_history()
    }

}

impl EngineState {
    /// Whether an undo step is available (to enable the toolbar's Undo button).
    pub fn can_undo(&self) -> bool {
        self.history.can_undo()
    }

    /// Whether a redo step is available.
    pub fn can_redo(&self) -> bool {
        self.history.can_redo()
    }

    /// Undo the last model mutation: restore the previous document + rolled-to
    /// step, then re-run + reconcile the scene. Returns the build report; a no-op
    /// (empty undo stack) returns the last report unchanged.
    pub fn undo(&mut self) -> String {
        if self.history.undo() {
            self.reinstall_rewound_parts_library();
            self.rerun_history()
        } else {
            self.history_report.clone()
        }
    }

    /// Redo the last undone model mutation (symmetric with [`Self::undo`]).
    pub fn redo(&mut self) -> String {
        if self.history.redo() {
            self.reinstall_rewound_parts_library();
            self.rerun_history()
        } else {
            self.history_report.clone()
        }
    }

    /// Push the just-rewound `partsLibrary` block back into the kernel's
    /// main-side store, so time travel moves the LIBRARY with the document.
    ///
    /// Without this, undo only half-works on anything that edits a library
    /// entry. The undo snapshot carries the block (`History::Snapshot`), but the
    /// block is a MIRROR: the per-run request does not carry it
    /// ([`History::prefix_request`] omits it) and a run that heals or GCs the
    /// store writes its result back over the block (`apply_run_output`). So a
    /// rewound block that was never pushed into the store is simply overwritten
    /// again, and the undo silently does nothing — which is what
    /// [`Self::set_part_attribute`](crate::engine_state::EngineState::set_part_attribute)
    /// would hit on its first undo.
    ///
    /// `install_parts_library` matches the store to the block BY CONTENT
    /// IDENTITY: an identical entry is kept verbatim (so a heal this side
    /// derived is not clobbered), a changed one is replaced and marked dirty
    /// (so the ACOMP self-heal re-derives every instance), and one absent from
    /// the block is dropped. An unchanged block is therefore a no-op, which is
    /// the overwhelmingly common undo.
    pub(super) fn reinstall_rewound_parts_library(&mut self) {
        let library = serde_json::from_value(self.history.parts_library().clone())
            .unwrap_or_default();
        brep_kernel::install_parts_library(&library);
    }

    // --- Selection (Esc clears / viewport click selects) ------------------

}

/// Stamp per-loop ids onto every SKETCH feature's geometries in a history
/// document, in place.
///
/// The persistence half of per-loop face naming. The kernel DERIVES a loop's id
/// the same way every run, so this renames nothing; what it adds is durability —
/// a stored id survives deleting the edge it was originally derived from, which
/// derivation alone cannot. Running it on document LOAD (not only on sketch
/// commit) means a model built by a script, an import, or any other path that
/// never opens the sketch editor still gets its ids written down.
///
/// Assemblies: a parts-library entry embeds a FULL sub-part history
/// (`partsLibrary[*].document`), whose features can include sketches of its own,
/// so the walk recurses into each one. Without that, a sketch inside an imported
/// assembly part would derive correct names but carry no stored ids — exactly the
/// case (deleting the edge an id came from) that persisting exists to cover.
/// Depth is bounded: a sub-document's own library is seeded from the kernel store
/// rather than nested inside the entry, so one level of recursion reaches all of
/// them, and `MAX_LIBRARY_DEPTH` stops a malformed self-referential document.
///
/// Malformed features are skipped rather than rejected: this is a best-effort
/// enrichment on the way to the kernel, which validates the document itself.
fn stamp_sketch_loop_ids(document: &mut serde_json::Value) {
    /// Depth cap for the embedded sub-document walk — a guard against a
    /// hand-edited or corrupt document that nests libraries into each other.
    const MAX_LIBRARY_DEPTH: usize = 8;
    stamp_sketch_loop_ids_to_depth(document, MAX_LIBRARY_DEPTH);
}

/// [`stamp_sketch_loop_ids`] with the remaining recursion budget.
fn stamp_sketch_loop_ids_to_depth(document: &mut serde_json::Value, depth: usize) {
    if let Some(features) = document
        .get_mut("features")
        .and_then(serde_json::Value::as_array_mut)
    {
        for feature in features {
            if feature.get("type").and_then(serde_json::Value::as_str) != Some("S") {
                continue;
            }
            let Some(sketch) = feature
                .get_mut("persistentData")
                .and_then(|data| data.get_mut("sketch"))
            else {
                continue;
            };
            brep_kernel::assign_sketch_loop_ids(sketch);
        }
    }
    if depth == 0 {
        return;
    }
    // Each library entry's embedded sub-part history gets the same treatment.
    let Some(library) = document
        .get_mut("partsLibrary")
        .and_then(serde_json::Value::as_object_mut)
    else {
        return;
    };
    for (_, entry) in library.iter_mut() {
        let Some(embedded) = entry.get_mut("document") else {
            continue;
        };
        stamp_sketch_loop_ids_to_depth(embedded, depth - 1);
    }
}

impl EngineState {
    /// Load a whole model document (a saved `.nbrep` recipe) and FRAME it:
    /// [`set_history_json`](Self::set_history_json) (which rolls to the last
    /// feature) followed by [`zoom_to_fit`](Self::zoom_to_fit). The one call the
    /// file panel's **Open** needs — the model IS the engine-owned history, so
    /// opening a file is loading its request JSON and reframing. Returns the
    /// build-report JSON.
    pub fn load_model_and_fit(&mut self, request_json: &str) -> Result<String, String> {
        // Frame once the run lands, not now: under a background runner (native
        // thread / wasm worker) the freshly loaded model is not resident yet when
        // `set_history_json` returns, so an immediate `zoom_to_fit` would frame the
        // OLD scene. Set BEFORE the submit so the Inline runner's in-call `pump`
        // still frames synchronously. See [`EngineState::pending_fit`].
        self.pending_fit = true;
        let result = self.set_history_json(request_json);
        if result.is_err() {
            // A rejected document submits no run, so the armed fit would otherwise
            // fire on the OLD scene next frame — disarm it.
            self.pending_fit = false;
        }
        result
    }
}






/// The run report's `featureRefusals` object, added only when a feature failed
/// with a TYPED refusal: feature id → the kernel's refusal as it serializes
/// (`class` and its payload, `stage`, `message` — the text `featureErrors`
/// carries for the same feature) plus `step`, the motion step that refused,
/// when the feature records one (Transform Face: `rotation` / `translation`).
/// Absent otherwise, so a report with no typed refusal is byte-for-byte what it
/// was before this key existed.
fn with_refusals(
    mut value: serde_json::Value,
    report: &crate::pipeline::SceneBuildReport,
) -> serde_json::Value {
    if report.feature_refusals.is_empty() {
        return value;
    }
    let refusals: serde_json::Map<String, serde_json::Value> = report
        .feature_refusals
        .iter()
        .map(|(id, refusal, step)| {
            let mut value = serde_json::to_value(refusal).unwrap_or(serde_json::Value::Null);
            if let (Some(step), serde_json::Value::Object(object)) = (step, &mut value) {
                object.insert("step".into(), serde_json::Value::String(step.clone()));
            }
            (id.clone(), value)
        })
        .collect();
    if let serde_json::Value::Object(object) = &mut value {
        object.insert("featureRefusals".into(), serde_json::Value::Object(refusals));
    }
    value
}

/// The run report's `featureApproximations` object, added only when a feature
/// SUCCEEDED carrying a MEASURED approximation: feature id → the kernel's
/// approximations as they serialize (`code`, `body`, `measured`, `bar`,
/// `volume_bound`, `edges[{edge_id, curve_ref, face_ref, surface_ref,
/// off_carrier_mm, origin}]`, `message`), each plus a one-line `summary` the
/// history tree shows as the feature's leaf. Absent otherwise, so a report with
/// no approximation is byte-for-byte what it was before this key existed — the
/// same contract as `featureRefusals`. Distinct from `featureRefusals` (the
/// result does not stand) and `featureFulfilment` (what was done of what was
/// asked): an approximation's result stands, and this says by how much it is
/// not exact.
fn with_approximations(
    mut value: serde_json::Value,
    report: &crate::pipeline::SceneBuildReport,
) -> serde_json::Value {
    if report.feature_approximations.is_empty() {
        return value;
    }
    let approximations: serde_json::Map<String, serde_json::Value> = report
        .feature_approximations
        .iter()
        .map(|(id, approximations)| {
            let entries: Vec<serde_json::Value> = approximations
                .iter()
                .map(|approximation| {
                    let mut value =
                        serde_json::to_value(approximation).unwrap_or(serde_json::Value::Null);
                    if let serde_json::Value::Object(object) = &mut value {
                        object.insert(
                            "summary".into(),
                            serde_json::Value::String(approximation_summary(approximation)),
                        );
                    }
                    value
                })
                .collect();
            (id.clone(), serde_json::Value::Array(entries))
        })
        .collect();
    if let serde_json::Value::Object(object) = &mut value {
        object.insert("featureApproximations".into(), serde_json::Value::Object(approximations));
    }
    value
}

/// One line for an approximation, for the history tree's leaf: the marker
/// word, the code, the body, the measurement against its bar and the volume
/// bound when the code implies one. Code-agnostic on purpose — the units of
/// `measured` are the code's (mm² for `import.shell_closure`), so only the
/// volume bound, which every code states in mm³, carries a unit here; the
/// kernel's `message` has the full text with units, faces and edges.
pub fn approximation_summary(approximation: &brep_kernel::Approximation) -> String {
    let ratio = if approximation.bar > 0.0 {
        format!(" ({:.1}x)", approximation.measured / approximation.bar)
    } else {
        String::new()
    };
    let bound = approximation
        .volume_bound
        .map(|bound| format!("; volume determined to ±{bound:.3e} mm³"))
        .unwrap_or_default();
    format!(
        "approximate ({}): {} measured {:.3e} against a bar of {:.3e}{ratio}{bound}",
        approximation.code, approximation.body, approximation.measured, approximation.bar
    )
}

/// The run report's `featureFulfilment` object: feature id → the typed
/// fulfilment (`requested`, `applied`, `rejected[{name, kind, reason}]`) plus a
/// one-line `summary` the tree and the automation report show verbatim. Only the
/// features that reported one appear, so an absent key means "did everything".
fn fulfilment_map(report: &crate::pipeline::SceneBuildReport) -> serde_json::Map<String, serde_json::Value> {
    report
        .feature_fulfilment
        .iter()
        .map(|(id, fulfilment)| {
            let mut value = serde_json::to_value(fulfilment).unwrap_or(serde_json::Value::Null);
            if let serde_json::Value::Object(object) = &mut value {
                object.insert("summary".into(), serde_json::Value::String(fulfilment.summary()));
            }
            (id.clone(), value)
        })
        .collect()
}
