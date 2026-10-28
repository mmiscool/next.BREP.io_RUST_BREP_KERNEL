//! The CAM workbench's engine-side seams: the reference picker for a CAM
//! record's field, committing into the document's `cam` block.
//!
//! The engine never interprets the block — the application's CAM document
//! module owns its shape — so a pick is committed by JSON PATH from the
//! block's root, with one convention that keeps a path stable while the
//! picker runs: a `#<id>` segment selects, in the array the path is at, the
//! object whose `id` is `<id>` (`["setups", "#SETUP1", "targets"]`), so a
//! record is addressed by identity rather than by an index another edit
//! could move. Nothing rolls and nothing re-runs: a CAM reference names the
//! finished model, which is what is on screen.

use super::{EngineState, RefSelectState, RefSelectTarget};
use crate::engine_state::selection_filter::SelectionFilter;
use serde_json::Value;

/// Write `value` at `path` under `root`, creating objects on the way; a
/// `#<id>` segment resolves to the element of the array at that point whose
/// `id` is `<id>`. An `#<id>` segment on a non-array, or one that matches no
/// element, is an error naming the segment — never a silent new record.
pub fn set_cam_json_at(root: &mut Value, path: &[String], value: Value) -> Result<(), String> {
    let Some((last, parents)) = path.split_last() else {
        *root = value;
        return Ok(());
    };
    let mut cur = root;
    for seg in parents {
        cur = step_into(cur, seg)?;
    }
    if let Some(id) = last.strip_prefix('#') {
        let slot = select_by_id(cur, id)?;
        *slot = value;
        return Ok(());
    }
    if !cur.is_object() {
        *cur = Value::Object(serde_json::Map::new());
    }
    cur.as_object_mut().expect("object by construction").insert(last.clone(), value);
    Ok(())
}

fn step_into<'a>(cur: &'a mut Value, seg: &str) -> Result<&'a mut Value, String> {
    if let Some(id) = seg.strip_prefix('#') {
        return select_by_id(cur, id);
    }
    if !cur.is_object() {
        *cur = Value::Object(serde_json::Map::new());
    }
    Ok(cur
        .as_object_mut()
        .expect("object by construction")
        .entry(seg.to_string())
        .or_insert(Value::Object(serde_json::Map::new())))
}

fn select_by_id<'a>(cur: &'a mut Value, id: &str) -> Result<&'a mut Value, String> {
    let Some(array) = cur.as_array_mut() else {
        return Err(format!("`#{id}` addresses a record, but the path is not at a list"));
    };
    array
        .iter_mut()
        .find(|record| record.get("id").and_then(Value::as_str) == Some(id))
        .ok_or_else(|| format!("no CAM record `{id}`"))
}

impl EngineState {
    /// Enter reference-selection mode for CAM record `id`'s field at `path`
    /// (from the block's root, see the module doc). Nothing rolls.
    #[allow(clippy::too_many_arguments)]
    pub fn begin_ref_select_for_cam(
        &mut self,
        id: &str,
        path: Vec<String>,
        label: String,
        filter: Vec<String>,
        multiple: bool,
        seed_names: Vec<String>,
    ) {
        let restore_index = self.history.rollback();
        self.selection_filter = SelectionFilter::from_ref_filter(&filter);
        self.ref_select = Some(RefSelectState {
            feature_id: id.to_string(),
            path,
            label,
            filter,
            multiple,
            names: seed_names,
            restore_index,
            target: RefSelectTarget::Cam,
            picks: 0,
            claimed: false,
            finish_requested: false,
        });
        self.sync_ref_select_emphasis();
    }

    /// Commit a finished CAM pick: the names (a list, or the first one)
    /// written at `path` under the `cam` block as ONE undo step. A block that
    /// is missing is an error (there is no record to write into); a path
    /// that resolves to no record is the setter's error.
    pub(crate) fn cam_commit_refs(&mut self, path: &[String], names: &[String], multiple: bool) -> Result<(), String> {
        let mut block = self.history.cam_block().cloned().ok_or_else(|| "the document has no CAM data".to_string())?;
        let value = if multiple {
            Value::Array(names.iter().cloned().map(Value::String).collect())
        } else {
            Value::String(names.first().cloned().unwrap_or_default())
        };
        set_cam_json_at(&mut block, path, value)?;
        self.history.set_cam_block(Some(block), None);
        // A pick is a discrete change (it ends with Finish): the next edit of
        // the same record's field is its own undo step.
        self.history.break_coalescing();
        Ok(())
    }
}


// ============================================================================
// Generation: the CAM jobs on the runner, their results and their progress.
// ============================================================================

/// One operation's generation outcome, kept by operation id. Derived data:
/// never written to the document; dropped on a document switch; STALE when
/// the inputs it was computed from (`inputs_hash`) have moved.
#[derive(Debug, Clone)]
pub struct CamOutcome {
    pub turning_profile: Option<brep_cam::turning::profile::TurningProfile>,
    /// The stock the operation's setup starts from, as generation resolved it.
    pub setup_start: Option<brep_cam::SetupStart>,
    pub toolpath: Option<brep_cam::model::Toolpath>,
    pub error: Option<String>,
    /// The hash of the inputs the job carried (the application computes it
    /// over the operation, its setup, its tool and the bodies' revisions).
    pub inputs_hash: u64,
    pub work_units: u64,
}

/// The generation queue and its results. ONE job is on the runner at a
/// time; the next is submitted when its reply lands, so a cancel between
/// operations is immediate on every platform (see `runner::CamJob`).
#[derive(Debug, Default)]
pub struct CamGeneration {
    queue: std::collections::VecDeque<(crate::runner::CamJob, u64)>,
    in_flight: Option<(u64, String, u64)>,
    next_id: u64,
    /// Job ids cancelled while in flight: their reply is dropped.
    cancelled: std::collections::BTreeSet<u64>,
    pub results: std::collections::BTreeMap<String, CamOutcome>,
    /// Bumped whenever `results` changes, so a viewer can key a cache on it
    /// instead of comparing toolpaths.
    pub results_stamp: u64,
    pub progress: Option<crate::runner::CamProgress>,
    /// What the last cancel or refusal said, for the pane.
    pub notice: Option<String>,
    /// Rebuilds an operation's job from the document as it stands, at the
    /// moment it is submitted (the application's `job_for`): the job then
    /// carries the results of the operations generated before it in the same
    /// queue (lane C7 pre-operation stock) and any edit made while it waited.
    /// `None`: the job is submitted as queued.
    pub job_builder: Option<CamJobBuilder>,
}

/// See [`CamGeneration::job_builder`].
pub type CamJobBuilder = fn(&EngineState, &str) -> Result<(crate::runner::CamJob, u64), String>;

impl CamGeneration {
    /// Whether a job is on the runner or waiting.
    pub fn busy(&self) -> bool {
        self.in_flight.is_some() || !self.queue.is_empty()
    }

    /// The operation ids waiting, in order, after the one in flight.
    pub fn queued(&self) -> Vec<String> {
        self.queue.iter().map(|(job, _)| job.operation.id.as_str().to_string()).collect()
    }

    /// The operation in flight.
    pub fn in_flight(&self) -> Option<&str> {
        self.in_flight.as_ref().map(|(_, op, _)| op.as_str())
    }

    /// Forget every result and job (a document switch).
    pub fn clear(&mut self) {
        if let Some((id, _, _)) = self.in_flight.take() {
            self.cancelled.insert(id);
        }
        self.queue.clear();
        self.results.clear();
        self.results_stamp += 1;
        self.progress = None;
        self.notice = None;
    }
}

impl EngineState {
    /// A feature's published axis by name (`AxisFixture:Axis`, from the last
    /// history run): a point on it and its direction, for a CAM setup's Z.
    pub fn cam_named_axis(&self, name: &str) -> Option<([f64; 3], [f64; 3])> {
        self.sketch_axes
            .iter()
            .find(|(n, _)| n == name)
            .map(|(_, a)| ([a.point.x, a.point.y, a.point.z], [a.direction.x, a.direction.y, a.direction.z]))
    }

    /// The sketch curves and profiles `chains` name, from the last history
    /// run: `{sketch}:G{id}` segments and each sketch's world profile. A name
    /// the run did not produce is refused by name (the sketch was deleted,
    /// rolled back past, or the id is stale), so the job never reaches the
    /// generator with a hole in it.
    #[allow(clippy::type_complexity)]
    pub fn cam_chain_geometry(
        &self,
        chains: &[brep_cam::model::ChainRef],
    ) -> Result<(Vec<(String, String, brep_kernel::NurbsCurve)>, Vec<(String, brep_kernel::SketchProfile)>), String> {
        use brep_cam::model::ChainSource;
        let mut curves: Vec<(String, String, brep_kernel::NurbsCurve)> = Vec::new();
        let mut profiles: Vec<(String, brep_kernel::SketchProfile)> = Vec::new();
        for chain in chains {
            match &chain.source {
                ChainSource::SketchCurves { sketch, curves: ids } => {
                    for id in ids {
                        if curves.iter().any(|(s, g, _)| s == sketch && g == id) {
                            continue;
                        }
                        let name = format!("{sketch}:G{id}");
                        let entry = self.sketch_paths.iter().find(|(n, _)| *n == name).ok_or_else(|| format!("sketch curve `{name}` is not in the model (run the history, or pick the chain again)"))?;
                        match entry.1.as_slice() {
                            [curve] => curves.push((sketch.clone(), id.clone(), curve.clone())),
                            other => return Err(format!("sketch curve `{name}` is {} curves, not one", other.len())),
                        }
                    }
                }
                ChainSource::SketchProfile { sketch, .. } => {
                    if profiles.iter().any(|(s, _)| s == sketch) {
                        continue;
                    }
                    let profile = self.sketch_profiles.iter().find(|(n, _)| n == sketch).ok_or_else(|| format!("sketch `{sketch}` has no closed profile in the model"))?;
                    profiles.push((sketch.clone(), profile.1.clone()));
                }
                ChainSource::Edges { .. } => {}
            }
        }
        Ok((curves, profiles))
    }
}

impl EngineState {
    /// Queue operations for generation: each `(job, inputs_hash)` is one
    /// runner job; an operation already queued or in flight is not queued
    /// twice. The jobs' ids are minted here.
    pub fn cam_enqueue(&mut self, jobs: Vec<(crate::runner::CamJob, u64)>) -> Vec<String> {
        let mut queued = Vec::new();
        for (mut job, hash) in jobs {
            let op = job.operation.id.as_str().to_string();
            let already = self.cam.in_flight().is_some_and(|f| f == op) || self.cam.queue.iter().any(|(j, _)| j.operation.id.as_str() == op);
            if already {
                continue;
            }
            self.cam.next_id += 1;
            job.id = self.cam.next_id;
            // A result for this operation is superseded the moment it is
            // queued again: the pane shows it as regenerating.
            queued.push(op);
            self.cam.queue.push_back((job, hash));
        }
        self.cam.notice = None;
        self.cam_pump();
        self.dirty = true;
        queued
    }

    /// Cancel: the queue is dropped at once; the job in flight is told to stop
    /// at its next chunk where the runner can (the native thread) and its
    /// reply is dropped either way; every result already landed stays.
    pub fn cam_cancel(&mut self) -> usize {
        let dropped = self.cam.queue.len() + usize::from(self.cam.in_flight.is_some());
        self.cam.queue.clear();
        if let Some((id, op, _)) = self.cam.in_flight.take() {
            self.cam.cancelled.insert(id);
            self.runner.cancel_cam();
            self.cam.notice = Some(format!("cancelled while generating {op}"));
        } else if dropped > 0 {
            self.cam.notice = Some("cancelled".to_string());
        }
        self.cam.progress = None;
        self.dirty = true;
        dropped
    }

    /// Submit the next job when none is in flight and fold every landed reply
    /// into the results. Called every frame with the other pumps.
    pub fn cam_pump(&mut self) {
        while let Some(progress) = self.runner.poll_cam_progress() {
            if self.cam.in_flight.as_ref().is_some_and(|(id, _, _)| *id == progress.id) {
                self.cam.progress = Some(progress);
                self.dirty = true;
            }
        }
        while let Some(reply) = self.runner.poll_cam() {
            let debug = std::env::var_os("BREP_CAM_DEBUG").is_some();
            if self.cam.cancelled.remove(&reply.id) {
                if debug {
                    eprintln!("cam reply {} ({}): dropped, cancelled", reply.id, reply.operation);
                }
                continue;
            }
            let Some((id, op, hash)) = self.cam.in_flight.clone() else {
                if debug {
                    eprintln!("cam reply {} ({}): dropped, nothing in flight", reply.id, reply.operation);
                }
                continue;
            };
            if id != reply.id {
                if debug {
                    eprintln!("cam reply {} ({}): dropped, in flight is {id} ({op})", reply.id, reply.operation);
                }
                continue;
            }
            if debug {
                eprintln!("cam reply {} ({}): landed, ok {}", reply.id, reply.operation, reply.toolpath.is_ok());
            }
            self.cam.in_flight = None;
            self.cam.progress = None;
            let (toolpath, error) = match reply.toolpath {
                Ok(path) => (Some(path), None),
                Err(message) => (None, Some(message)),
            };
            self.cam.results_stamp += 1;
            self.cam.results.insert(op, CamOutcome { turning_profile: reply.turning_profile, setup_start: reply.setup_start, toolpath, error, inputs_hash: hash, work_units: reply.work_units });
            self.dirty = true;
        }
        while self.cam.in_flight.is_none() {
            let Some((job, hash)) = self.cam.queue.pop_front() else { break };
            let id = job.id;
            let op = job.operation.id.as_str().to_string();
            // Built again at dispatch: an operation that cannot be built now
            // (its inputs went bad while it waited) records why, by name.
            let rebuilt = match self.cam.job_builder {
                Some(build) => build(self, &op).map(|(mut job, hash)| {
                    job.id = id;
                    (job, hash)
                }),
                None => Ok((job, hash)),
            };
            let (job, hash) = match rebuilt {
                Ok(built) => built,
                Err(why) => {
                    self.cam.results_stamp += 1;
                    self.cam.results.insert(op, CamOutcome { toolpath: None, error: Some(why), inputs_hash: hash, work_units: 0, turning_profile: None, setup_start: None });
                    self.dirty = true;
                    continue;
                }
            };
            {
                if self.runner.submit_cam(job) {
                    self.cam.in_flight = Some((id, op, hash));
                } else {
                    self.cam.results_stamp += 1;
            self.cam.results.insert(op, CamOutcome { turning_profile: None, setup_start: None, toolpath: None, error: Some("this runner cannot generate toolpaths".to_string()), inputs_hash: hash, work_units: 0 });
                }
                self.dirty = true;
            }
        }
    }

    /// The generation state for the pane and the automation.
    /// TEST HOLD for automation scripts (see `HistoryRunner::hold_cam_replies`):
    /// while held, a generation job stays in flight on this side however fast
    /// the runner finished it, so a scripted cancel always meets it; releasing
    /// lets the kept replies land (a cancelled one is dropped as usual).
    pub fn cam_hold_replies(&mut self, hold: bool) {
        self.runner.hold_cam_replies(hold);
    }

    pub fn cam_generation_json(&self) -> serde_json::Value {
        serde_json::json!({
            "busy": self.cam.busy(),
            "inFlight": self.cam.in_flight(),
            "queue": self.cam.queued(),
            "progress": self.cam.progress.as_ref().map(|p| serde_json::json!({ "operation": p.operation, "phase": p.phase, "message": p.message, "current": p.current, "total": p.total })),
            "notice": self.cam.notice,
            "results": self.cam.results.iter().map(|(op, r)| (op.clone(), serde_json::json!({
                "ok": r.error.is_none(),
                "error": r.error,
                "inputsHash": r.inputs_hash.to_string(),
                "workUnits": r.work_units,
                "motions": r.toolpath.as_ref().map(|t| t.motions.len()).unwrap_or(0),
                "cuttingLength": r.toolpath.as_ref().map(|t| t.cutting_length()),
                "status": r.toolpath.as_ref().map(|t| format!("{:?}", t.report.status)),
                "warnings": r.toolpath.as_ref().map(|t| t.report.warnings.len()).unwrap_or(0),
            }))).collect::<serde_json::Map<String, serde_json::Value>>(),
        })
    }
}

