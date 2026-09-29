//! Revision thumbnails over HTTP. The rules are in [`crate::thumbnail`].
//!
//! - `PUT /api/parts/:id/revisions/:rev/thumbnail?content_hash=&renderer=`
//!   with the PNG as the body: `{ ok, thumbnail }`. `409` when the revision's
//!   document is no longer the one pictured.
//! - `GET /api/parts/:id/revisions/:rev/thumbnail` and
//!   `GET /api/parts/:id/thumbnail` (the newest revision's that is current):
//!   the PNG, or `204` when there is none or it is stale; `404` for an unknown
//!   part or revision. The ETag is the blob's hash, and `If-None-Match`
//!   answers `304`. Asked with `?v=<that hash>` — the URL the listings hand
//!   out — the answer may be cached for good; without it, revalidated.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, HeaderValue, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{blocking, require_user, Shared};
use crate::thumbnail::{self, Claim};
use crate::Error;

#[derive(Debug, Deserialize, Default)]
pub struct PutQuery {
    #[serde(default)]
    pub content_hash: String,
    #[serde(default)]
    pub renderer: String,
}

/// Keep a revision's thumbnail.
pub async fn put(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Query(query): Query<PutQuery>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    if !user.can_author() && !user.can_bake() {
        return Err(Error::forbidden("a thumbnail is uploaded by an author or the bake worker"));
    }
    let bytes = axum::body::to_bytes(body, thumbnail::MAX_BYTES as usize).await.map_err(|_| Error {
        status: StatusCode::PAYLOAD_TOO_LARGE,
        message: format!("a thumbnail is at most {} KB", thumbnail::MAX_BYTES / 1024),
    })?;
    let size = thumbnail::png_size(&bytes)?;
    let mut upload = db.start_upload()?;
    upload.write(&bytes)?;
    let staged = upload.finish()?;
    let claim = Claim { content_hash: query.content_hash, renderer: query.renderer };
    let kept = blocking(move || db.put_thumbnail(&user, &id, &rev, staged, size, &claim)).await?;
    Ok(Json(json!({ "ok": true, "thumbnail": kept })))
}

#[derive(Debug, Deserialize, Default)]
pub struct GetQuery {
    /// The blob hash the caller expects; when it is the one served, the
    /// answer is immutable.
    #[serde(default)]
    pub v: String,
}

/// One revision's thumbnail.
pub async fn get_revision(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Query(query): Query<GetQuery>,
) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let found = db.thumbnail(&id, &rev)?;
    serve(&db, &headers, found, &query.v).await
}

/// A part's thumbnail: its newest revision's that is current.
pub async fn get_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<GetQuery>,
) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let found = db.thumbnail(&id, "")?;
    serve(&db, &headers, found, &query.v).await
}

async fn serve(
    db: &Shared,
    headers: &HeaderMap,
    found: Option<(String, crate::model::Thumbnail)>,
    asked: &str,
) -> Result<Response, Error> {
    let Some((label, t)) = found else {
        let mut none = StatusCode::NO_CONTENT.into_response();
        none.headers_mut().insert(header::CACHE_CONTROL, HeaderValue::from_static("no-cache"));
        return Ok(none);
    };
    let etag = format!("\"{}\"", t.sha256);
    let cache = if asked == t.sha256 { "private, max-age=31536000, immutable" } else { "private, no-cache" };
    let matches = headers
        .get(header::IF_NONE_MATCH)
        .and_then(|v| v.to_str().ok())
        .is_some_and(|v| v.split(',').any(|tag| tag.trim() == etag || tag.trim() == "*"));
    let mut response = if matches {
        StatusCode::NOT_MODIFIED.into_response()
    } else {
        let bytes = tokio::fs::read(db.blob_path(&t.sha256))
            .await
            .map_err(|e| Error::internal(format!("the thumbnail's file is missing: {e}")))?;
        let mut r = Body::from(bytes).into_response();
        r.headers_mut().insert(header::CONTENT_TYPE, HeaderValue::from_static(thumbnail::MEDIA_TYPE));
        r
    };
    let h = response.headers_mut();
    h.insert(header::CACHE_CONTROL, HeaderValue::from_static(cache));
    if let Ok(v) = HeaderValue::from_str(&etag) {
        h.insert(header::ETAG, v);
    }
    // Which revision it pictures, and from which document.
    if let Ok(v) = HeaderValue::from_str(&ascii(&label)) {
        h.insert("x-thumbnail-revision", v);
    }
    if let Ok(v) = HeaderValue::from_str(&t.content_hash) {
        h.insert("x-thumbnail-content-hash", v);
    }
    if let Ok(v) = HeaderValue::from_str(&t.renderer) {
        h.insert("x-thumbnail-renderer", v);
    }
    Ok(response)
}

/// A free-text label as a header value: printable ASCII kept, the rest `_`.
fn ascii(text: &str) -> String {
    text.chars().map(|c| if c.is_ascii_graphic() || c == ' ' { c } else { '_' }).collect()
}
