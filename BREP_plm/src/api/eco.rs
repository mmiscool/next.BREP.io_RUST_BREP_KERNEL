//! Change orders over HTTP ([`crate::eco`]).
//!
//! Routes that can run a script (numbering, submission, release, and every
//! review event) run on the blocking pool; mutating routes answer with the
//! change order, or `{ ok, seq, warnings }`.

use axum::extract::{Path, Query, State};
use axum::http::HeaderMap;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::review::{verdict, ChangeBody, CommentBody, DecisionBody};
use super::{blocking, require_admin, require_user, Shared};
use crate::eco::{EcoChange, EcoRow, EcoView, NewEco, NewItem, NumberingChange};
use crate::model::{ChangeOrder, PartType};
use crate::review::RoundChange;
use crate::Error;

fn done(db: &crate::db::Db, warnings: Vec<String>) -> Response {
    let seq = db.read(|state| state.seq);
    Json(json!({ "ok": true, "seq": seq, "warnings": warnings })).into_response()
}

#[derive(Debug, Default, Deserialize)]
pub struct ListQuery {
    /// `draft`, `inreview`, `approved`, `released`, `cancelled`, or `open`.
    #[serde(default)]
    pub state: String,
}

/// `GET /api/ecos?state=`
pub async fn list(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<ListQuery>,
) -> Result<Json<Vec<EcoRow>>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.list_ecos(&query.state)))
}

/// `POST /api/ecos`
pub async fn create(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewEco>,
) -> Result<Json<ChangeOrder>, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    Ok(Json(blocking(move || handle.create_eco(&user, &body)).await?))
}

/// `GET /api/ecos/numbering`
pub async fn numbering(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<PartType>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.read(|state| state.eco_numbering.0.clone())))
}

/// `PATCH /api/ecos/numbering` — administrators.
pub async fn set_numbering(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NumberingChange>,
) -> Result<Json<PartType>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.update_eco_numbering(&body)?))
}

/// `GET /api/ecos/:id` — by id or number.
pub async fn get(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Json<EcoView>, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    Ok(Json(blocking(move || handle.eco_view(&user, &id)).await?))
}

/// `PATCH /api/ecos/:id`
pub async fn update(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<EcoChange>,
) -> Result<Json<ChangeOrder>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.update_eco(&user, &id, &body)?))
}

/// `POST /api/ecos/:id/items` — `{ part, revision, action, note }`.
pub async fn add_item(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<NewItem>,
) -> Result<Json<ChangeOrder>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.add_eco_item(&user, &id, &body)?))
}

#[derive(Default, Deserialize)]
pub struct ItemOwner { pub part: Option<String> }

/// `DELETE /api/ecos/:id/items/:rev?part=<part id>`
pub async fn remove_item(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Query(owner): Query<ItemOwner>,
) -> Result<Json<ChangeOrder>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(match owner.part {
        Some(part) => db.remove_eco_part_item(&user, &id, &part, &rev)?,
        None => db.remove_eco_item(&user, &id, &rev)?,
    }))
}

#[derive(Debug, Default, Deserialize)]
pub struct SubmitBody {
    #[serde(default)]
    pub reviewers: Option<Value>,
    #[serde(default)]
    pub note: String,
}

/// `POST /api/ecos/:id/submit`
pub async fn submit(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    body: Option<Json<SubmitBody>>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let body = body.map(|Json(b)| b).unwrap_or_default();
    let handle = db.clone();
    let warnings = blocking(move || handle.submit_eco(&user, &id, body.reviewers.as_ref(), &body.note)).await?;
    Ok(done(&db, warnings))
}

/// `POST /api/ecos/:id/withdraw`
pub async fn withdraw(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    let warnings = blocking(move || handle.withdraw_eco(&user, &id)).await?;
    Ok(done(&db, warnings))
}

/// `POST /api/ecos/:id/cancel`
pub async fn cancel(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    let warnings = blocking(move || handle.cancel_eco(&user, &id)).await?;
    Ok(done(&db, warnings))
}

/// `POST /api/ecos/:id/release` — the check-in group.
pub async fn release(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    let warnings = blocking(move || handle.release_eco(&user, &id)).await?;
    Ok(done(&db, warnings))
}

/// `POST /api/ecos/:id/review/decision`
pub async fn decide(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<DecisionBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let verdict = verdict(&body.verdict)?;
    let handle = db.clone();
    let warnings = blocking(move || handle.decide_eco(&user, &id, verdict, &body.comment)).await?;
    Ok(done(&db, warnings))
}

/// `PATCH /api/ecos/:id/review`
pub async fn change_review(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<ChangeBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let change: RoundChange = body.into();
    let handle = db.clone();
    let warnings = blocking(move || handle.change_eco_review(&user, &id, &change)).await?;
    Ok(done(&db, warnings))
}

/// `POST /api/ecos/:id/comments`
pub async fn comment(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<CommentBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    let (comment, warnings) = blocking(move || handle.comment_on_eco(&user, &id, &body.body, &body.parent)).await?;
    let seq = db.read(|state| state.seq);
    Ok(Json(json!({ "ok": true, "seq": seq, "comment": comment, "warnings": warnings })).into_response())
}
