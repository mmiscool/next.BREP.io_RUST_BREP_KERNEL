//! Review and approval over HTTP ([`crate::review`]): a revision's review
//! panel, submitting with reviewers, decisions, a round's changes, comments,
//! and each person's inbox.
//!
//! Every route that can run a script (`reviewers.js` on submit,
//! `review-event.js` after every event) runs on the blocking pool, and every
//! mutating one answers `{ ok, seq, warnings }` — a warning is a
//! `review-event` hook that failed after the event committed.

use axum::extract::{Path, State};
use axum::http::HeaderMap;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{blocking, require_user, Shared};
use crate::review::{Inbox, RevisionReview, RoundChange, Submission};
use crate::model::Verdict;
use crate::Error;

fn done(db: &crate::db::Db, warnings: Vec<String>) -> Response {
    let seq = db.read(|state| state.seq);
    Json(json!({ "ok": true, "seq": seq, "warnings": warnings })).into_response()
}

/// `GET /api/parts/:id/revisions/:rev/review`
pub async fn get_review(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
) -> Result<Json<RevisionReview>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.revision_review(&user, &id, &rev)?))
}

#[derive(Debug, Default, Deserialize)]
pub struct SubmitBody {
    /// Reviewers added to the rule's: `["user:ada", "group:quality"]`.
    #[serde(default)]
    pub reviewers: Option<Value>,
    /// Unix seconds.
    #[serde(default)]
    pub due: Option<u64>,
    /// A comment posted with the submission.
    #[serde(default)]
    pub note: String,
}

/// `POST /api/parts/:id/revisions/:rev/submit` — submit for review with
/// extra reviewers, a due date or a note. `state` with `to: inreview` does the
/// same with none of them.
pub async fn submit(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    body: Option<Json<SubmitBody>>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    if !user.can_author() {
        return Err(Error::forbidden("submitting for review needs the author group"));
    }
    let body = body.map(|Json(b)| b).unwrap_or_default();
    let submission = Submission { reviewers: body.reviewers, due: body.due, note: body.note };
    let handle = db.clone();
    let warnings = blocking(move || handle.submit_for_review(&user, &id, &rev, &submission)).await?;
    Ok(done(&db, warnings))
}

#[derive(Debug, Deserialize)]
pub struct DecisionBody {
    /// `approve` or `reject`.
    pub verdict: String,
    #[serde(default)]
    pub comment: String,
}

pub fn verdict(text: &str) -> Result<Verdict, Error> {
    match text.trim().to_ascii_lowercase().as_str() {
        "approve" | "approved" => Ok(Verdict::Approve),
        "reject" | "rejected" => Ok(Verdict::Reject),
        other => Err(Error::bad_request(format!("'{other}' is not a verdict — approve or reject"))),
    }
}

/// `POST /api/parts/:id/revisions/:rev/review/decision`
pub async fn decide(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Json(body): Json<DecisionBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let verdict = verdict(&body.verdict)?;
    let handle = db.clone();
    let warnings = blocking(move || handle.decide_revision(&user, &id, &rev, verdict, &body.comment)).await?;
    Ok(done(&db, warnings))
}

#[derive(Debug, Default, Deserialize)]
pub struct ChangeBody {
    #[serde(default)]
    pub reviewers: Option<Value>,
    #[serde(default)]
    pub required_approvals: Option<u32>,
    /// Unix seconds; 0 clears it.
    #[serde(default)]
    pub due: Option<u64>,
}

impl From<ChangeBody> for RoundChange {
    fn from(body: ChangeBody) -> Self {
        RoundChange { reviewers: body.reviewers, required_approvals: body.required_approvals, due: body.due }
    }
}

/// `PATCH /api/parts/:id/revisions/:rev/review`
pub async fn change(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Json(body): Json<ChangeBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let change: RoundChange = body.into();
    let handle = db.clone();
    let warnings = blocking(move || handle.change_revision_review(&user, &id, &rev, &change)).await?;
    Ok(done(&db, warnings))
}

#[derive(Debug, Deserialize)]
pub struct CommentBody {
    pub body: String,
    #[serde(default)]
    pub parent: String,
}

/// `POST /api/parts/:id/revisions/:rev/comments`
pub async fn comment(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Json(body): Json<CommentBody>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    let (comment, warnings) =
        blocking(move || handle.comment_on_revision(&user, &id, &rev, &body.body, &body.parent)).await?;
    let seq = db.read(|state| state.seq);
    Ok(Json(json!({ "ok": true, "seq": seq, "comment": comment, "warnings": warnings })).into_response())
}

/// `GET /api/inbox` — what is waiting on the caller, and what they submitted.
pub async fn inbox(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Inbox>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.inbox(&user)))
}
