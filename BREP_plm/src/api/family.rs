//! Families, templates and the bake queue over HTTP. The rules are in
//! [`crate::family`]; these handlers decide only who is asking.
//!
//! * `GET  /api/parts/:id/family?revision=` — the family page.
//! * `POST /api/parts/:id/generate` — Generate (author). With `rows` carrying
//!   documents it is the CAD app's Generate; without, the server assembles
//!   each member and queues it for a bake.
//! * `POST /api/parts/:id/family/import` — merge CSV rows into the table of a
//!   family revision in work (author), and optionally Generate.
//! * `GET  /api/parts/:id/template?revision=` — the template page.
//! * `POST /api/parts/:id/spin-out` — a new part from a template (author).
//! * `/api/bake/...` — the queue a headless CAD worker (the `worker` group)
//!   takes jobs from.

use axum::extract::{Path, Query, State};
use axum::http::{HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;

use super::{blocking, require_author, require_user, Shared};
use crate::family::{BakeJob, FamilyView, GenerateReport, GenerateRequest, ImportReport, SpinOutRequest, TemplateView};
use crate::model::User;
use crate::Error;

#[derive(Debug, Deserialize)]
pub struct Which {
    #[serde(default)]
    pub revision: String,
}

pub async fn family(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(which): Query<Which>,
) -> Result<Json<FamilyView>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.family_view(&user, &id, &which.revision)?))
}

pub async fn generate(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    body: Option<Json<GenerateRequest>>,
) -> Result<Json<GenerateReport>, Error> {
    let user = require_author(&db, &headers)?;
    let request = body.map(|Json(b)| b).unwrap_or_default();
    Ok(Json(blocking(move || db.generate(&user, &id, request)).await?))
}

#[derive(Debug, Deserialize)]
pub struct Import {
    pub csv: String,
    /// The family revision (id or label) whose table is edited. Empty: the
    /// newest one in work.
    #[serde(default)]
    pub revision: String,
    /// Generate from that revision right after the import.
    #[serde(default)]
    pub generate: bool,
}

#[derive(Debug, serde::Serialize)]
pub struct Imported {
    pub import: ImportReport,
    pub generate: Option<GenerateReport>,
}

pub async fn import(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Import>,
) -> Result<Json<Imported>, Error> {
    let user = require_author(&db, &headers)?;
    let result = blocking(move || {
        let import = db.import_table(&user, &id, &body.revision, &body.csv)?;
        let generate = if body.generate {
            Some(db.generate(
                &user,
                &id,
                GenerateRequest { family_revision: import.revision_id.clone(), ..GenerateRequest::default() },
            )?)
        } else {
            None
        };
        Ok(Imported { import, generate })
    })
    .await?;
    Ok(Json(result))
}

pub async fn template(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(which): Query<Which>,
) -> Result<Json<TemplateView>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.template_view(&id, &which.revision)?))
}

pub async fn spin_out(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<SpinOutRequest>,
) -> Result<Response, Error> {
    let user = require_author(&db, &headers)?;
    let part = blocking(move || db.spin_out(&user, &id, body)).await?;
    Ok(Json(part).into_response())
}

// ===========================================================================
// The bake queue
// ===========================================================================

/// A signed-in user in the `worker` group (or an admin).
fn require_worker(db: &crate::db::Db, headers: &HeaderMap) -> Result<User, Error> {
    let user = require_user(db, headers)?;
    if !user.can_bake() {
        return Err(Error::forbidden("this needs the worker group"));
    }
    Ok(user)
}

#[derive(Debug, Deserialize)]
pub struct Status {
    #[serde(default)]
    pub status: String,
}

/// The queue. Anyone signed in may look; `status` narrows it, `all` includes
/// finished jobs.
pub async fn jobs(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(status): Query<Status>,
) -> Result<Json<Vec<BakeJob>>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.bake_jobs(&status.status)))
}

/// Claim the oldest free job; `204 No Content` when there is none.
pub async fn next(State(db): State<Shared>, headers: HeaderMap) -> Result<Response, Error> {
    let worker = require_worker(&db, &headers)?;
    Ok(match db.claim_bake(&worker, None)? {
        Some(job) => Json(job).into_response(),
        None => StatusCode::NO_CONTENT.into_response(),
    })
}

pub async fn claim(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    let worker = require_worker(&db, &headers)?;
    let job = db.claim_bake(&worker, Some(&id))?.ok_or_else(|| Error::not_found("bake job"))?;
    Ok(Json(job).into_response())
}

/// Extend the caller's claim on a job by a full lease ([`crate::family::BAKE_LEASE`]
/// from now): what a worker whose bake runs long sends every few minutes.
/// Answers the job, with its new `lease_expires_at`.
pub async fn renew(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    let worker = require_worker(&db, &headers)?;
    Ok(Json(db.renew_bake(&worker, &id)?).into_response())
}

/// The baked document, as the request body.
pub async fn result(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    body: String,
) -> Result<Response, Error> {
    let worker = require_worker(&db, &headers)?;
    db.finish_bake(&worker, &id, &body)?;
    Ok(super::ok_seq(&db))
}

#[derive(Debug, Deserialize)]
pub struct Failure {
    pub error: String,
}

pub async fn fail(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Failure>,
) -> Result<Response, Error> {
    let worker = require_worker(&db, &headers)?;
    db.fail_bake(&worker, &id, &body.error)?;
    Ok(super::ok_seq(&db))
}

/// Put a failed or claimed job back in the queue. Authors — the people who
/// fix the row that failed — and admins.
pub async fn retry(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    require_author(&db, &headers)?;
    db.retry_bake(&id)?;
    Ok(super::ok_seq(&db))
}
