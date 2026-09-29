//! Attachments over HTTP. The rules are in [`crate::attach`].
//!
//! An upload is the file's bytes as the request body — not a multipart form —
//! with its media type as `Content-Type` and its name, kind and note in the
//! query string. It streams to disk as it arrives and is refused with `413`
//! the moment it passes the limit. A download streams from disk.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, HeaderValue};
use axum::response::{IntoResponse, Response};
use axum::Json;
use futures_util::StreamExt;
use serde::Deserialize;
use serde_json::{json, Value};

use super::{blocking, require_author, require_user, Shared};
use crate::attach::{self, Describe};
use crate::Error;

#[derive(Debug, Deserialize, Default)]
pub struct UploadQuery {
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub kind: String,
    #[serde(default)]
    pub note: String,
}

/// Stream a request body into the staging directory under the attachment
/// limit. Workspace files arrive the same way.
pub(crate) async fn receive(db: &Shared, headers: &HeaderMap, body: Body) -> Result<attach::Staged, Error> {
    let mut upload = db.start_upload()?;
    let limit = db.security().config.attachment_limit();
    if let Some(length) = headers.get(header::CONTENT_LENGTH).and_then(|v| v.to_str().ok()).and_then(|v| v.parse::<u64>().ok()) {
        if length > limit {
            return Err(Error {
                status: axum::http::StatusCode::PAYLOAD_TOO_LARGE,
                message: format!("the file is over the {} MB attachment limit", limit / (1024 * 1024)),
            });
        }
    }
    let mut stream = body.into_data_stream();
    while let Some(chunk) = stream.next().await {
        let chunk = chunk.map_err(|e| Error::bad_request(format!("the upload broke off: {e}")))?;
        upload.write(&chunk)?;
    }
    upload.finish()
}

fn describe(headers: &HeaderMap, query: UploadQuery) -> Describe {
    Describe {
        name: query.name,
        media_type: headers
            .get(header::CONTENT_TYPE)
            .and_then(|v| v.to_str().ok())
            .unwrap_or("")
            .to_string(),
        kind: query.kind,
        note: query.note,
    }
}

/// Attach a file to a part.
pub async fn upload_to_part(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<UploadQuery>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let staged = receive(&db, &headers, body).await?;
    let what = describe(&headers, query);
    let done = blocking(move || db.attach(&user, &id, "", staged, &what)).await?;
    Ok(Json(json!({ "ok": true, "attachment": done })))
}

/// Attach a file to one revision.
pub async fn upload_to_revision(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, rev)): Path<(String, String)>,
    Query(query): Query<UploadQuery>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let staged = receive(&db, &headers, body).await?;
    let what = describe(&headers, query);
    let done = blocking(move || db.attach(&user, &id, &rev, staged, &what)).await?;
    Ok(Json(json!({ "ok": true, "attachment": done })))
}

#[derive(Debug, Deserialize, Default)]
pub struct ListQuery {
    /// A revision id or label: its files too. Empty: the part's only.
    #[serde(default)]
    pub revision: String,
}

/// A part's files, and one revision's.
pub async fn list(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<ListQuery>,
) -> Result<Json<Value>, Error> {
    require_user(&db, &headers)?;
    db.read(|state| {
        let part = state.part_by_id_or_number(&id).ok_or_else(|| Error::not_found("part"))?;
        let revision = match query.revision.trim() {
            "" => None,
            key => Some(crate::bom::find_revision(part, key).ok_or_else(|| Error::not_found("revision"))?),
        };
        let name = |uid: &str| state.user(uid).map(|u| u.username.clone()).unwrap_or_default();
        let show = |list: &[crate::model::Attachment]| -> Vec<Value> {
            list.iter()
                .map(|a| {
                    let mut v = serde_json::to_value(a).unwrap_or(Value::Null);
                    v["uploaded_by_name"] = json!(name(&a.uploaded_by));
                    v["inline"] = json!(attach::inline_ok(&a.media_type));
                    v
                })
                .collect()
        };
        Ok(Json(json!({
            "part": show(&part.attachments),
            "revision": revision.map(|r| show(&r.attachments)).unwrap_or_default(),
            "revision_label": revision.map(|r| r.label.clone()).unwrap_or_default(),
            "revision_editable": revision.map(|r| r.lifecycle.is_editable()).unwrap_or(false),
            "kinds": attach::KINDS,
        })))
    })
}

#[derive(Debug, Deserialize, Default)]
pub struct DownloadQuery {
    /// Ask to see it in the browser; honoured only for the safe types.
    #[serde(default)]
    pub inline: bool,
}

/// `attachment; filename=...` with an ASCII fallback and the UTF-8 name.
fn disposition(inline: bool, name: &str) -> String {
    let ascii: String = name
        .chars()
        .map(|c| if c.is_ascii_graphic() && c != '"' && c != '\\' || c == ' ' { c } else { '_' })
        .collect();
    let mut encoded = String::new();
    for byte in name.as_bytes() {
        if byte.is_ascii_alphanumeric() || b"-._~".contains(byte) {
            encoded.push(*byte as char);
        } else {
            encoded.push_str(&format!("%{byte:02X}"));
        }
    }
    format!(
        "{}; filename=\"{ascii}\"; filename*=UTF-8''{encoded}",
        if inline { "inline" } else { "attachment" }
    )
}

/// Stream a file.
pub async fn download(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<DownloadQuery>,
) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let found = db.attachment(&id).ok_or_else(|| Error::not_found("attachment"))?;
    let a = found.attachment;
    stream_blob(&db, &a.name, &a.media_type, &a.sha256, a.size, query.inline).await
}

/// Stream the blob `sha256` as a download named `name`: its media type,
/// length, disposition (inline only when asked AND safe) and an ETag.
/// Workspace files download the same way.
pub(crate) async fn stream_blob(
    db: &Shared,
    name: &str,
    media_type: &str,
    sha256: &str,
    size: u64,
    inline: bool,
) -> Result<Response, Error> {
    if !attach::is_sha256(sha256) {
        return Err(Error::internal("a file names no valid blob"));
    }
    let file = tokio::fs::File::open(db.blob_path(sha256))
        .await
        .map_err(|e| Error::internal(format!("the file for {name} is missing: {e}")))?;
    let stream = futures_util::stream::unfold(file, |mut file| async move {
        use tokio::io::AsyncReadExt;
        let mut buffer = vec![0u8; 64 * 1024];
        match file.read(&mut buffer).await {
            Ok(0) => None,
            Ok(n) => {
                buffer.truncate(n);
                Some((Ok::<_, std::io::Error>(buffer), file))
            }
            Err(e) => Some((Err(e), file)),
        }
    });
    let inline = inline && attach::inline_ok(media_type);
    let mut response = Body::from_stream(stream).into_response();
    let h = response.headers_mut();
    h.insert(
        header::CONTENT_TYPE,
        HeaderValue::from_str(media_type).unwrap_or(HeaderValue::from_static("application/octet-stream")),
    );
    h.insert(header::CONTENT_LENGTH, HeaderValue::from(size));
    h.insert(
        header::CONTENT_DISPOSITION,
        HeaderValue::from_str(&disposition(inline, name)).unwrap_or(HeaderValue::from_static("attachment")),
    );
    if let Ok(etag) = HeaderValue::from_str(&format!("\"{sha256}\"")) {
        h.insert(header::ETAG, etag);
    }
    Ok(response)
}

/// Remove an attachment.
pub async fn remove(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let gone = blocking(move || db.detach(&user, &id)).await?;
    Ok(Json(json!({ "ok": true, "removed": gone.attachment.id })))
}
