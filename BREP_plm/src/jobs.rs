//! The jobs floor queue: serialized builds of released manufacturing
//! instructions, signed off step by step on the shop floor.
//!
//! One job is one build of one released instructions revision. Its steps are
//! a snapshot of that revision's step order (reading order: a step's children
//! before it), taken when the job is made; a released document never changes, so
//! the snapshot is what the floor builds. Each step is signed once, in order,
//! by an operator (or an author or admin), who records the serial of every
//! unit the step adds of a part marked `Serial_Tracked`. A serial of a part
//! is in at most one job that is not cancelled: `GET /api/serials` answers
//! which build took it. Only the check-in group reopens a signed step (and every step after
//! it), and the audit keeps both.
//!
//! No inventory: no stock, no locations, no consumption. The queue is a step
//! towards an ERP system, not one.
//!
//! Every write below is one read-modify-write inside [`Db::mutate`], which
//! holds the store's write lock from the first read to the commit: two
//! tablets never both read a step unsigned, and no write is lost. Jobs are
//! journaled ([`crate::journal::Journaled`]): a write stores and audits only
//! the jobs it touched, one `meta` row each.

use std::collections::BTreeSet;

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::db::{now, Db, State};
use crate::model::{groups, DocumentClass, Instructs, Lifecycle, Timestamp, User};
use crate::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum JobStatus {
    Queued,
    InProgress,
    Done,
    Cancelled,
}

impl JobStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            JobStatus::Queued => "queued",
            JobStatus::InProgress => "in_progress",
            JobStatus::Done => "done",
            JobStatus::Cancelled => "cancelled",
        }
    }

    /// Queued or in progress: its serials are taken.
    pub fn is_live(self) -> bool {
        matches!(self, JobStatus::Queued | JobStatus::InProgress)
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Job {
    /// `JOB000001`: handed out in order, never reused.
    pub id: String,
    pub instructions: Instructs,
    pub assembly: Instructs,
    /// The serial of the unit being built, when it has one.
    #[serde(default)]
    pub serial: String,
    pub status: JobStatus,
    /// The queue's order: lowest first, ties by number.
    pub position: i64,
    /// A user id, or empty for anyone.
    #[serde(default)]
    pub assigned_to: String,
    pub created_by: String,
    pub created_at: Timestamp,
    #[serde(default)]
    pub started_at: Option<Timestamp>,
    #[serde(default)]
    pub finished_at: Option<Timestamp>,
    /// The store sequence of the job's last change: a floor tablet polls
    /// the queue and re-reads what moved.
    #[serde(default)]
    pub modified_seq: u64,
    pub steps: Vec<JobStep>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JobStep {
    /// The instructions' step id.
    pub step: String,
    /// The sub-assembly the step helps build (the occurrence its parent step
    /// places): empty at the top level. Named for the chapters that held a
    /// sub-assembly's steps before the step tree.
    #[serde(default)]
    pub chapter: String,
    #[serde(default)]
    pub title: String,
    /// What the step adds, with the units that need a serial.
    #[serde(default)]
    pub adds: Vec<JobAdd>,
    #[serde(default)]
    pub signed_by: Option<String>,
    #[serde(default)]
    pub signed_at: Option<Timestamp>,
    #[serde(default)]
    pub note: String,
    #[serde(default)]
    pub serials: Vec<SerialEntry>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JobAdd {
    /// One occurrence path per unit (`ACOMP1:ACOMP3`).
    pub occurrences: Vec<String>,
    /// The PLM part id when the add names a part this server knows, else
    /// the document's text.
    #[serde(default)]
    pub part: String,
    /// One serial per occurrence at sign-off: the part's `Serial_Tracked`
    /// attribute, or the document marks the add.
    pub serial_tracked: bool,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SerialEntry {
    pub occurrence: String,
    /// The PLM part. A sign-off need not send it: the server takes it from
    /// the step's add for that occurrence.
    #[serde(default)]
    pub part: String,
    pub serial: String,
}

/// Whether a part (its newest revision's type attributes, then its catalog
/// attributes) is marked `Serial_Tracked`, the attribute name the app and
/// server agreed (matched without regard to case).
pub fn serial_tracked(state: &State, part_id: &str) -> bool {
    let Some(part) = state.part(part_id) else { return false };
    let truthy = |map: &std::collections::BTreeMap<String, Value>| {
        map.iter().find(|(k, _)| k.eq_ignore_ascii_case("serial_tracked")).is_some_and(|(_, v)| {
            v.as_bool().unwrap_or(false) || v.as_str().is_some_and(|s| matches!(s.trim().to_ascii_lowercase().as_str(), "true" | "yes" | "1"))
        })
    };
    part.latest().is_some_and(|r| truthy(&r.attributes)) || truthy(&part.attributes)
}

/// The sub-assembly a step helps build: the occurrence its parent step
/// places; empty at the top level.
fn group_of(instructions: &brep_instructions::model::Instructions, step_id: &str) -> String {
    instructions
        .parent_id(step_id)
        .flatten()
        .and_then(|parent| instructions.step(parent))
        .and_then(|parent| parent.adds.iter().find(|add| add.subassembly).and_then(|add| add.occurrences.first().cloned()))
        .unwrap_or_default()
}

/// The steps a job signs, read with the instructions crate's own model (a
/// schema-1 document's chapters read as the tree): every step in reading
/// order (a step's children before it, as the wizard orders them), leaving
/// out a step that needs no sign-off.
pub fn snapshot_steps(state: &State, document: &Value) -> Result<Vec<JobStep>, Error> {
    let block = document.get(brep_instructions::model::BLOCK_KEY).ok_or_else(|| Error::conflict("the document holds no instructions"))?;
    let instructions = brep_instructions::model::Instructions::from_value(block)
        .map_err(|e| Error::conflict(format!("the instructions do not read: {e}")))?;
    let steps: Vec<JobStep> = instructions
        .steps()
        // A step that needs no sign-off is not on the floor's checklist.
        .filter(|(_, step)| step.signoff.required)
        .map(|(_, step)| JobStep {
            step: step.id.clone(),
            chapter: group_of(&instructions, &step.id),
            title: step.title.clone(),
            adds: step
                .adds
                .iter()
                .map(|add| {
                    let known = state.part_by_id_or_number(&add.part).map(|p| p.id.clone());
                    JobAdd {
                        occurrences: add.occurrences.clone(),
                        serial_tracked: add.serial_tracked || known.as_deref().is_some_and(|id| serial_tracked(state, id)),
                        part: known.unwrap_or_else(|| add.part.clone()),
                    }
                })
                .collect(),
            signed_by: None,
            signed_at: None,
            note: String::new(),
            serials: Vec::new(),
        })
        .collect();
    if steps.is_empty() {
        return Err(Error::conflict("the instructions have no steps to build"));
    }
    let mut seen = BTreeSet::new();
    if let Some(twice) = steps.iter().find(|s| !seen.insert(s.step.clone())) {
        return Err(Error::conflict(format!("step id {} is used twice", twice.step)));
    }
    Ok(steps)
}

/// Who may sign a step: operators, authors and admins.
pub fn can_sign(user: &User) -> bool {
    [groups::OPERATOR, groups::AUTHOR, groups::ADMIN].iter().any(|g| user.groups.iter().any(|u| u == g))
}

/// Who may reopen a signed step: the check-in group and admins.
pub fn can_reopen(user: &User) -> bool {
    [groups::CHECKIN, groups::ADMIN].iter().any(|g| user.groups.iter().any(|u| u == g))
}

/// Where a part's serial is taken: the job (live or done; a cancelled job
/// frees its serials) and the step that took it.
pub fn serial_holder<'a>(state: &'a State, part: &str, serial: &str) -> Option<(&'a Job, &'a JobStep)> {
    state.jobs.iter().filter(|j| j.status != JobStatus::Cancelled).find_map(|job| {
        job.steps
            .iter()
            .find(|s| s.serials.iter().any(|e| e.part == part && e.serial.eq_ignore_ascii_case(serial)))
            .map(|s| (job, s))
    })
}

#[derive(Debug, Clone, Deserialize)]
pub struct NewJob {
    /// The instructions part, by id or number.
    pub instructions: String,
    /// Its revision, by id or label: a released one.
    pub revision: String,
    #[serde(default)]
    pub serial: String,
    /// How many jobs to queue (each its own build). One by default.
    #[serde(default)]
    pub quantity: Option<u32>,
}

#[derive(Debug, Clone, Deserialize)]
pub struct SignOff {
    #[serde(default)]
    pub serials: Vec<SerialEntry>,
    #[serde(default)]
    pub note: String,
}

impl Db {
    /// Queue `quantity` jobs building one released instructions revision.
    pub fn create_jobs(&self, user: &User, request: &NewJob) -> Result<Vec<Job>, Error> {
        let quantity = request.quantity.unwrap_or(1);
        if !(1..=500).contains(&quantity) {
            return Err(Error::bad_request("a request queues 1 to 500 jobs"));
        }
        let serial = request.serial.trim().to_string();
        if quantity > 1 && !serial.is_empty() {
            return Err(Error::bad_request("a serial names one build: queue one job for it"));
        }
        // The document is read outside the lock: a released revision's bytes
        // never change, and the lifecycle is checked again inside it.
        let (part_id, revision_id, path) = self.read(|state| {
            let part = state.part_by_id_or_number(&request.instructions).ok_or_else(|| Error::not_found("instructions"))?;
            if part.document_class != DocumentClass::Instructions {
                return Err(Error::bad_request(format!("{} is not manufacturing instructions", part.number)));
            }
            let revision = crate::bom::find_revision(part, &request.revision).ok_or_else(|| Error::not_found("revision"))?;
            Ok((part.id.clone(), revision.id.clone(), crate::identity::model_path(&part.id, &revision.id)))
        })?;
        let document: Value = std::fs::read_to_string(self.root().join(path))
            .ok()
            .and_then(|text| serde_json::from_str(&text).ok())
            .ok_or_else(|| Error::conflict("the instructions revision has no readable document"))?;
        let user_id = user.id.clone();
        self.mutate(move |state| {
            let part = state.part(&part_id).ok_or_else(|| Error::not_found("instructions"))?;
            let revision = part.revision(&revision_id).ok_or_else(|| Error::not_found("revision"))?;
            if revision.lifecycle != Lifecycle::Released {
                return Err(Error::conflict(format!(
                    "{} rev {} is {}: the floor builds released instructions only",
                    part.number,
                    revision.label,
                    revision.lifecycle.as_str()
                )));
            }
            let assembly = revision.instructs.clone().ok_or_else(|| Error::conflict("the instructions instruct no PLM assembly"))?;
            let steps = snapshot_steps(state, &document)?;
            if !serial.is_empty() {
                if let Some(job) = state.jobs.iter().find(|j| j.status != JobStatus::Cancelled && j.assembly.part == assembly.part && j.serial.eq_ignore_ascii_case(&serial)) {
                    return Err(Error::conflict(format!("serial {serial} is {} by {}", if job.status == JobStatus::Done { "already built" } else { "already being built" }, job.id)));
                }
            }
            let mut position = state.jobs.iter().filter(|j| j.status.is_live()).map(|j| j.position).max().unwrap_or(0);
            let stamp = now();
            let mut made = Vec::new();
            for _ in 0..quantity {
                state.next_job += 1;
                position += 1;
                let job = Job {
                    id: format!("JOB{:06}", state.next_job),
                    instructions: Instructs { part: part_id.clone(), revision: revision_id.clone() },
                    assembly: assembly.clone(),
                    serial: serial.clone(),
                    status: JobStatus::Queued,
                    position,
                    assigned_to: String::new(),
                    created_by: user_id.clone(),
                    created_at: stamp,
                    started_at: None,
                    finished_at: None,
                    modified_seq: state.seq,
                    steps: steps.clone(),
                };
                state.jobs.push(job.clone());
                made.push(job);
            }
            Ok(made)
        })
    }

    /// Change a job's place in the queue, who it is assigned to, or cancel
    /// it (`status: "cancelled"`; a cancelled job frees its serials).
    pub fn update_job(&self, id: &str, fields: &Value) -> Result<Job, Error> {
        let fields = fields.as_object().cloned().ok_or_else(|| Error::bad_request("a job change is a JSON object"))?;
        if let Some(key) = fields.keys().find(|k| !["position", "assigned_to", "status"].contains(&k.as_str())) {
            return Err(Error::bad_request(format!("a job has no field '{key}' to change")));
        }
        let id = id.to_string();
        self.mutate(move |state| {
            if let Some(user) = fields.get("assigned_to").and_then(Value::as_str).filter(|u| !u.is_empty()) {
                if state.user(user).is_none() {
                    return Err(Error::bad_request(format!("no user '{user}'")));
                }
            }
            let seq = state.seq;
            let job = state.jobs.get_mut(&id).ok_or_else(|| Error::not_found("job"))?;
            if !job.status.is_live() {
                return Err(Error::conflict(format!("{} is {}", job.id, job.status.as_str())));
            }
            if let Some(position) = fields.get("position") {
                job.position = position.as_i64().ok_or_else(|| Error::bad_request("position is a whole number"))?;
            }
            if let Some(user) = fields.get("assigned_to") {
                job.assigned_to = user.as_str().ok_or_else(|| Error::bad_request("assigned_to is a user id"))?.to_string();
            }
            match fields.get("status").map(|s| s.as_str()) {
                None => {}
                Some(Some("cancelled")) => {
                    job.status = JobStatus::Cancelled;
                    job.finished_at = Some(now());
                }
                Some(_) => return Err(Error::bad_request("a job's status changes only to cancelled; sign-off moves the rest")),
            }
            job.modified_seq = seq;
            Ok(job.clone())
        })
    }

    /// Sign one step of a job, in order, with the serials it captures.
    pub fn sign_step(&self, user: &User, id: &str, step: &str, signoff: &SignOff) -> Result<JobStep, Error> {
        if !can_sign(user) {
            return Err(Error::forbidden("signing a step needs the operator, author or admin group"));
        }
        let (id, step, user_id) = (id.to_string(), step.to_string(), user.id.clone());
        let mut signoff = signoff.clone();
        for entry in &mut signoff.serials {
            entry.serial = entry.serial.trim().to_string();
            if entry.serial.is_empty() || entry.serial.chars().count() > 120 {
                return Err(Error::bad_request("a serial is 1 to 120 characters"));
            }
        }
        if signoff.note.chars().count() > 2000 {
            return Err(Error::bad_request("a sign-off note is at most 2000 characters"));
        }
        self.mutate(move |state| {
            let job = state.jobs.get(&id).ok_or_else(|| Error::not_found("job"))?;
            if !job.status.is_live() {
                return Err(Error::conflict(format!("{} is {}: nothing more is signed", job.id, job.status.as_str())));
            }
            let at = job.steps.iter().position(|s| s.step == step).ok_or_else(|| Error::not_found("step"))?;
            let this = &job.steps[at];
            if let (Some(by), Some(when)) = (&this.signed_by, this.signed_at) {
                let name = state.user(by).map_or_else(|| "someone".to_string(), |u| u.username.clone());
                return Err(Error::conflict(format!("step {} of {} was signed by {name} at {when}", this.step, job.id)));
            }
            if let Some(earlier) = job.steps[..at].iter().find(|s| s.signed_at.is_none()) {
                return Err(Error::conflict(format!("step {} of {} is not signed yet: steps are signed in order", earlier.step, job.id)));
            }
            // Exactly one serial per occurrence of each serial-tracked add,
            // and no serial for anything else.
            let add_of = |occurrence: &str| this.adds.iter().find(|a| a.occurrences.iter().any(|o| o == occurrence));
            if let Some(stray) = signoff.serials.iter().find(|e| add_of(&e.occurrence).is_none_or(|a| !a.serial_tracked)) {
                return Err(Error::conflict(format!("step {} takes no serial for '{}'", this.step, stray.occurrence)));
            }
            for add in this.adds.iter().filter(|a| a.serial_tracked) {
                for occurrence in &add.occurrences {
                    let given = signoff.serials.iter().filter(|e| &e.occurrence == occurrence).count();
                    if given != 1 {
                        return Err(Error::conflict(format!("{occurrence} takes one serial at step {}; {given} given", this.step)));
                    }
                }
            }
            let mut serials = Vec::new();
            for entry in &signoff.serials {
                let part = add_of(&entry.occurrence).map(|a| a.part.clone()).unwrap_or_default();
                if serials.iter().any(|e: &SerialEntry| e.part == part && e.serial.eq_ignore_ascii_case(&entry.serial)) {
                    return Err(Error::conflict(format!("serial {} is given twice", entry.serial)));
                }
                if let Some((holder, held_at)) = serial_holder(state, &part, &entry.serial) {
                    return Err(Error::conflict(format!("serial {} of {part} was taken by {} at step {}", entry.serial, holder.id, held_at.step)));
                }
                serials.push(SerialEntry { occurrence: entry.occurrence.clone(), part, serial: entry.serial.clone() });
            }
            let (seq, stamp) = (state.seq, now());
            let job = state.jobs.get_mut(&id).expect("the job was read under this lock");
            let signed = &mut job.steps[at];
            signed.signed_by = Some(user_id);
            signed.signed_at = Some(stamp);
            signed.note = signoff.note.trim().to_string();
            signed.serials = serials;
            let signed = signed.clone();
            if job.status == JobStatus::Queued {
                job.status = JobStatus::InProgress;
                job.started_at = Some(stamp);
            }
            if job.steps.iter().all(|s| s.signed_at.is_some()) {
                job.status = JobStatus::Done;
                job.finished_at = Some(stamp);
            }
            job.modified_seq = seq;
            Ok(signed)
        })
    }

    /// Reopen a signed step and every step after it: the check-in group, with
    /// a reason the audit keeps. A done job is in progress again.
    pub fn reopen_step(&self, user: &User, id: &str, step: &str, reason: &str) -> Result<Job, Error> {
        if !can_reopen(user) {
            return Err(Error::forbidden("reopening a signed step needs the check-in group"));
        }
        let reason = reason.trim().to_string();
        if reason.is_empty() || reason.chars().count() > 2000 {
            return Err(Error::bad_request("a reopen gives its reason (at most 2000 characters)"));
        }
        let (id, step) = (id.to_string(), step.to_string());
        self.mutate(move |state| {
            let seq = state.seq;
            let job = state.jobs.get_mut(&id).ok_or_else(|| Error::not_found("job"))?;
            if job.status == JobStatus::Cancelled {
                return Err(Error::conflict(format!("{} is cancelled", job.id)));
            }
            let at = job.steps.iter().position(|s| s.step == step).ok_or_else(|| Error::not_found("step"))?;
            if job.steps[at].signed_at.is_none() {
                return Err(Error::conflict(format!("step {step} of {} is not signed", job.id)));
            }
            for later in &mut job.steps[at..] {
                later.signed_by = None;
                later.signed_at = None;
                later.serials.clear();
                later.note = String::new();
            }
            job.steps[at].note = format!("reopened: {reason}");
            job.status = if job.steps.iter().any(|s| s.signed_at.is_some()) { JobStatus::InProgress } else { JobStatus::Queued };
            job.finished_at = None;
            job.modified_seq = seq;
            Ok(job.clone())
        })
    }
}
