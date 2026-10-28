//! The jobs floor queue over HTTP ([`crate::jobs`]).

use axum::extract::{Path, Query, State};
use axum::http::HeaderMap;
use axum::Json;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use super::{blocking, require_author, require_user, Shared};
use crate::jobs::{serial_holder, Job, JobStep, NewJob, SignOff};
use crate::Error;

#[derive(Debug, Deserialize, Default)]
pub struct QueueQuery {
    /// Comma-separated statuses. Empty: the live queue (queued,
    /// in_progress), or every status for a poll with `since`.
    #[serde(default)]
    pub status: String,
    /// Only jobs changed after this store sequence (a floor tablet's poll).
    #[serde(default)]
    pub since: u64,
}

#[derive(Debug, Serialize)]
pub struct Queue {
    /// The store sequence the answer is current at: the next poll's `since`.
    pub seq: u64,
    pub jobs: Vec<Job>,
}

/// `GET /api/jobs`: the queue, by position then number.
pub async fn list(State(db): State<Shared>, headers: HeaderMap, Query(query): Query<QueueQuery>) -> Result<Json<Queue>, Error> {
    require_user(&db, &headers)?;
    let wanted: Vec<String> = match query.status.trim() {
        // A poll (`since`) sees every status: a job that finished or was
        // cancelled since leaves the tablet's queue.
        "" if query.since > 0 => vec!["queued".into(), "in_progress".into(), "done".into(), "cancelled".into()],
        "" => vec!["queued".into(), "in_progress".into()],
        list => list.split(',').map(|s| s.trim().to_string()).filter(|s| !s.is_empty()).collect(),
    };
    if let Some(bad) = wanted.iter().find(|s| !["queued", "in_progress", "done", "cancelled"].contains(&s.as_str())) {
        return Err(Error::bad_request(format!("'{bad}' is not a job status")));
    }
    Ok(Json(db.read(|state| {
        let mut jobs: Vec<Job> = state
            .jobs
            .iter()
            // A job's `modified_seq` is at least 1 (a change bumps the
            // sequence before it runs), so `since=0` is every job.
            .filter(|j| wanted.iter().any(|s| s == j.status.as_str()) && j.modified_seq > query.since)
            .cloned()
            .collect();
        jobs.sort_by(|a, b| (a.position, &a.id).cmp(&(b.position, &b.id)));
        Queue { seq: state.seq, jobs }
    })))
}

pub async fn get(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Json<Job>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| state.jobs.iter().find(|j| j.id == id).cloned()).map(Json).ok_or_else(|| Error::not_found("job"))
}

/// `POST /api/jobs {instructions, revision, serial?, quantity?}`.
pub async fn create(State(db): State<Shared>, headers: HeaderMap, Json(request): Json<NewJob>) -> Result<Json<Vec<Job>>, Error> {
    let user = require_author(&db, &headers)?;
    Ok(Json(blocking(move || db.create_jobs(&user, &request)).await?))
}

/// `PATCH /api/jobs/:id {position?, assigned_to?, status?: "cancelled"}`.
pub async fn update(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>, Json(fields): Json<Value>) -> Result<Json<Job>, Error> {
    require_author(&db, &headers)?;
    Ok(Json(blocking(move || db.update_job(&id, &fields)).await?))
}

/// `POST /api/jobs/:id/steps/:step/signoff {serials, note?}`.
pub async fn sign(State(db): State<Shared>, headers: HeaderMap, Path((id, step)): Path<(String, String)>, Json(signoff): Json<SignOff>) -> Result<Json<JobStep>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(blocking(move || db.sign_step(&user, &id, &step, &signoff)).await?))
}

#[derive(Debug, Deserialize)]
pub struct Reopen {
    pub reason: String,
}

/// `POST /api/jobs/:id/steps/:step/reopen {reason}`.
pub async fn reopen(State(db): State<Shared>, headers: HeaderMap, Path((id, step)): Path<(String, String)>, Json(body): Json<Reopen>) -> Result<Json<Job>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(blocking(move || db.reopen_step(&user, &id, &step, &body.reason)).await?))
}

#[derive(Debug, Deserialize)]
pub struct SerialQuery {
    /// A part id or number.
    pub part: String,
    pub serial: String,
}

/// `GET /api/serials?part=&serial=`: which build took this serial.
pub async fn serial(State(db): State<Shared>, headers: HeaderMap, Query(query): Query<SerialQuery>) -> Result<Json<Value>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let part = state.part_by_id_or_number(&query.part).ok_or_else(|| Error::not_found("part"))?;
        let (job, step) = serial_holder(state, &part.id, query.serial.trim()).ok_or_else(|| Error::not_found("serial"))?;
        let signer = step.signed_by.as_deref().and_then(|id| state.user(id)).map(|u| u.username.clone());
        Ok(Json(json!({
            "job": job.id,
            "status": job.status.as_str(),
            "serial": job.serial,
            "step": step.step,
            "signed_by": signer,
            "signed_at": step.signed_at,
        })))
    })
}
