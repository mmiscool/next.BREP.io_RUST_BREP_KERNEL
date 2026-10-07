//! The DOCUMENT surface — the contract the CAD app's `StoreBackend` will
//! consume, shaped to the five obligations a remote backend carries.
//!
//! | Obligation | Route |
//! | --- | --- |
//! | Metadata-only hydration (never pull the corpus) | `GET /api/store/index` |
//! | Read one key, seeing other clients' writes | `GET /api/store/doc/*key` |
//! | Write one key | `PUT /api/store/doc/*key` |
//! | Delete one key | `DELETE /api/store/doc/*key` |
//! | A monotonic change sequence + what moved | `GET /api/store/changes?since=` |
//!
//! `seq` rides every response so a client's cache-invalidation key never needs
//! a second request, and `stale` tells a client that has been away too long to
//! re-read the index rather than trust a short answer.
//!
//! # Why a write can be refused
//!
//! A `PUT` needs the revision to be EDITABLE and the caller to hold its lock.
//! Both refusals are `409` with a sentence explaining which, because the CAD
//! app's write-behind lane surfaces the message to the user after its own
//! `write()` has already returned — a bare status would leave them with "it
//! did not save" and nothing else.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::{Deserialize, Serialize};

use super::{require_user, Shared};
use crate::Error;

/// One row of the index: everything a client needs to list, sort and decide
/// staleness — and no document bytes.
#[derive(Debug, Serialize)]
pub struct IndexEntry {
    pub key: String,
    pub part_id: String,
    pub part_number: String,
    pub part_name: String,
    /// What the document is — `normal`, `family` or `template` — fixed when
    /// the part was made (plm-cad-integration-todo D4).
    pub document_class: crate::model::DocumentClass,
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: &'static str,
    pub editable: bool,
    pub content_hash: String,
    pub size: u64,
    pub modified: u64,
    pub locked_by: Option<String>,
    pub locked_by_me: bool,
}

#[derive(Debug, Serialize)]
pub struct Index {
    pub seq: u64,
    pub entries: Vec<IndexEntry>,
}

#[derive(Debug, Default, Deserialize)]
pub struct IndexQuery {
    /// Comma-separated document keys: only their rows (a key that names no
    /// revision is left out). Absent: the whole index.
    #[serde(default)]
    pub keys: Option<String>,
}

/// The whole key space as METADATA. This is `load_index`.
///
/// `?keys=part/a/rev/b,part/c/rev/d` answers only those rows, looked up by
/// part: what a client re-reads after the change feed named them, at a cost
/// of the keys asked and not of the catalog.
pub async fn index(
    State(db): State<Shared>,
    headers: HeaderMap,
    query: Option<Query<IndexQuery>>,
) -> Result<Json<Index>, Error> {
    let user = require_user(&db, &headers)?;
    let wanted: Option<Vec<(String, String)>> = query.and_then(|Query(q)| q.keys).map(|keys| {
        keys.split(',').filter_map(|key| split_key(key.trim()).ok()).collect()
    });
    Ok(Json(db.read(|state| {
        let row = |part: &crate::model::Part, revision: &crate::model::Revision| IndexEntry {
            key: revision.document_key(&part.id),
            part_id: part.id.clone(),
            part_number: part.number.clone(),
            part_name: part.name.clone(),
            document_class: part.document_class,
            revision_id: revision.id.clone(),
            revision_label: revision.label.clone(),
            lifecycle: revision.lifecycle.as_str(),
            editable: revision.lifecycle.is_editable(),
            content_hash: revision.content_hash.clone(),
            size: revision.size,
            modified: revision.modified_at,
            locked_by: revision.lock.as_ref().and_then(|l| state.user(&l.user_id).map(|u| u.username.clone())),
            locked_by_me: revision.lock.as_ref().is_some_and(|l| l.user_id == user.id),
        };
        let mut entries = Vec::new();
        match &wanted {
            Some(keys) => {
                for (part_id, revision_id) in keys {
                    if let Some(part) = state.part(part_id) {
                        if let Some(revision) = part.revision(revision_id) {
                            entries.push(row(part, revision));
                        }
                    }
                }
            }
            None => {
                for part in &state.parts {
                    for revision in &part.revisions {
                        entries.push(row(part, revision));
                    }
                }
            }
        }
        Index { seq: state.seq, entries }
    })))
}

#[derive(Debug, Deserialize)]
pub struct Since {
    #[serde(default)]
    pub since: u64,
}

#[derive(Debug, Serialize)]
pub struct Changes {
    pub seq: u64,
    /// The client asked from further back than the log reaches. Its correct
    /// response is to re-read the index, NOT to treat `keys` as complete.
    pub stale: bool,
    pub keys: Vec<String>,
}

pub async fn changes(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<Since>,
) -> Result<Json<Changes>, Error> {
    require_user(&db, &headers)?;
    let (seq, stale, keys) = db.changes_since(query.since);
    Ok(Json(Changes { seq, stale, keys }))
}

/// `part/<part id>/rev/<revision id>` split back into its halves.
fn split_key(key: &str) -> Result<(String, String), Error> {
    crate::identity::split_key(key).ok_or_else(|| Error::bad_request(format!(
        "'{key}' is not a document key — expected part/<part>/rev/<revision>"
    )))
}

/// Read one document. This is the LIVE read: it goes to the file every time,
/// so a client re-reading a key after someone else wrote it sees the new
/// bytes, which is the whole reason the backend trait requires a `get`.
pub async fn read_doc(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let (part_id, revision_id) = split_key(&key)?;
    let exists = db.read(|state| {
        state
            .part(&part_id)
            .and_then(|p| p.revision(&revision_id))
            .is_some()
    });
    if !exists {
        return Err(Error::not_found("revision"));
    }
    match db.read_document(&key)? {
        Some(body) => Ok((
            StatusCode::OK,
            [(header::CONTENT_TYPE, "application/json; charset=utf-8")],
            body,
        )
            .into_response()),
        // A revision with no document yet is a real state, not an error — an
        // imported part with no 3D model will be exactly this. 204 says "the
        // revision is there and empty"; 404 would say "no such revision".
        None => Ok(StatusCode::NO_CONTENT.into_response()),
    }
}

/// Write one document.
///
/// Refused unless the revision is editable AND the caller holds its lock.
/// Together those two checks are the immutability rule: there is no path by
/// which a released revision's bytes change.
pub async fn write_doc(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
    body: Body,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let (part_id, revision_id) = split_key(&key)?;
    guard_writable(&db, &user, &part_id, &revision_id)?;
    let limit = db.security().config.document_limit();
    let body = read_bounded(&headers, body, limit, || {
        format!("the document is over the {} document limit", megabytes(limit))
    })
    .await?;

    // The CAD app writes history JSON. Refusing a body that is not JSON keeps
    // a truncated upload from becoming a revision's content.
    if serde_json::from_str::<serde_json::Value>(&body).is_err() {
        return Err(Error::bad_request("a document must be JSON"));
    }

    let seq = db.write_document(&part_id, &revision_id, &body)?;
    Ok(Json(serde_json::json!({
        "ok": true,
        "seq": seq,
        "content_hash": db.read(|state|state.part(&part_id).and_then(|p|p.revision(&revision_id)).map(|r|r.content_hash.clone()).unwrap_or_default()),
        "size": body.len(),
    }))
    .into_response())
}

pub async fn delete_doc(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(key): Path<String>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let (part_id, revision_id) = split_key(&key)?;
    guard_writable(&db, &user, &part_id, &revision_id)?;
    db.remove_document(&key)?;
    db.mutate(|state| {
        if let Some(revision) = state.parts.get_mut(&part_id)
            .and_then(|p| p.revision_mut(&revision_id))
        {
            revision.content_hash.clear();
            revision.size = 0;
            revision.modified_at = crate::db::now();
        }
        crate::review::document_changed(state, &part_id, &revision_id);
        Ok(())
    })?;
    Ok(super::ok_seq(&db))
}

/// The two conditions a document write rests on, checked together so the
/// message can say which one failed ([`crate::db::check_writable`], which the
/// uses list is written under too).
fn guard_writable(
    db: &crate::db::Db,
    user: &crate::model::User,
    part_id: &str,
    revision_id: &str,
) -> Result<(), Error> {
    db.read(|state| crate::db::check_writable(state, user, part_id, revision_id))
}

/// A request body as text, refused with a `413` and `refusal()`'s sentence
/// past `limit` bytes — read here rather than through axum's `String`
/// extractor, whose 2 MB default answers a real model document with its own
/// plain-text 413. A declared `Content-Length` over the limit is refused
/// before anything is read. A body that is not UTF-8 cannot be JSON.
pub(crate) async fn read_bounded(
    headers: &HeaderMap,
    body: Body,
    limit: u64,
    refusal: impl Fn() -> String,
) -> Result<String, Error> {
    use futures_util::StreamExt;
    let too_large = || Error { status: StatusCode::PAYLOAD_TOO_LARGE, message: refusal() };
    let declared = headers
        .get(header::CONTENT_LENGTH)
        .and_then(|v| v.to_str().ok())
        .and_then(|v| v.parse::<u64>().ok());
    if declared.is_some_and(|length| length > limit) {
        return Err(too_large());
    }
    let mut bytes = Vec::with_capacity(declared.unwrap_or(0) as usize);
    let mut stream = body.into_data_stream();
    while let Some(chunk) = stream.next().await {
        let chunk = chunk.map_err(|e| Error::bad_request(format!("the request body broke off: {e}")))?;
        if bytes.len() as u64 + chunk.len() as u64 > limit {
            return Err(too_large());
        }
        bytes.extend_from_slice(&chunk);
    }
    String::from_utf8(bytes).map_err(|_| Error::bad_request("the body must be JSON, and it is not UTF-8 text"))
}

/// `limit` as a person reads it: whole megabytes, else kilobytes.
pub(crate) fn megabytes(limit: u64) -> String {
    const MB: u64 = 1024 * 1024;
    if limit >= MB && limit % MB == 0 { format!("{} MB", limit / MB) } else { format!("{} KB", limit.div_ceil(1024)) }
}
