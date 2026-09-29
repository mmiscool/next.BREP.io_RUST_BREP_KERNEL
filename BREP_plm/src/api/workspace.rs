//! Workspaces over HTTP. The rules are in [`crate::workspace`].
//!
//! A file upload is the raw bytes as the body, as for attachments
//! ([`super::attachments`]): its media type as `Content-Type`, its folder and
//! name in the query string. It streams to disk and is refused with `413` the
//! moment it passes the attachment limit.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::HeaderMap;
use axum::response::Response;
use axum::Json;
use serde::Deserialize;
use serde_json::{json, Value};

use super::attachments::{receive, stream_blob};
use super::{blocking, require_user, Shared};
use crate::workspace::Promote;
use crate::Error;

fn media_type(headers: &HeaderMap) -> String {
    headers
        .get(axum::http::header::CONTENT_TYPE)
        .and_then(|v| v.to_str().ok())
        .unwrap_or("")
        .to_string()
}

fn seq(db: &Shared) -> u64 {
    db.read(|state| state.seq)
}

/// The workspaces the caller may open, their own first.
pub async fn list_workspaces(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let browsable = db.read(|state| state.settings.workspaces_browsable);
    Ok(Json(json!({ "workspaces": db.workspaces(&user), "browsable": browsable })))
}

#[derive(Debug, Deserialize, Default)]
pub struct FolderQuery {
    /// A user id or username; empty is the caller.
    #[serde(default)]
    pub owner: String,
    /// A folder id; empty is the top.
    #[serde(default)]
    pub parent: String,
}

/// The entries in one folder.
pub async fn list_folder(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<FolderQuery>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let entries = db.workspace_folder(&user, &query.owner, &query.parent)?;
    Ok(Json(json!({ "entries": entries, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct NewFolder {
    #[serde(default)]
    pub parent: String,
    pub name: String,
}

pub async fn create_folder(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewFolder>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let made = blocking({
        let db = db.clone();
        move || db.create_folder(&user, &body.parent, &body.name)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": made, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct NewLink {
    #[serde(default)]
    pub parent: String,
    /// Empty: the part's number.
    #[serde(default)]
    pub name: String,
    /// A part id or number.
    pub part: String,
    /// A revision id or label to pin to; empty follows the newest.
    #[serde(default)]
    pub revision: String,
}

pub async fn create_link(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewLink>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let made = blocking({
        let db = db.clone();
        move || db.create_link(&user, &body.parent, &body.name, &body.part, &body.revision)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": made, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct NewFile {
    #[serde(default)]
    pub parent: String,
    #[serde(default)]
    pub name: String,
}

/// Upload a new file: version 1.
pub async fn create_file(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<NewFile>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    // Refuse a bad name before reading the bytes.
    crate::workspace::clean_name(&query.name)?;
    let staged = receive(&db, &headers, body).await?;
    let media = media_type(&headers);
    let made = blocking({
        let db = db.clone();
        move || db.create_file(&user, &query.parent, &query.name, &media, staged)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": made, "seq": seq(&db) })))
}

/// One entry.
pub async fn get_entry(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(json!({ "entry": db.workspace_entry(&user, &id)? })))
}

/// Rename, move, or pin / unpin a link.
pub async fn update_entry(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Value>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let changed = blocking({
        let db = db.clone();
        move || db.update_entry(&user, &id, &body)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": changed, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct DeleteQuery {
    /// Delete a folder with everything in it.
    #[serde(default)]
    pub recursive: bool,
}

/// Remove an entry. Never a part or a revision.
pub async fn delete_entry(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<DeleteQuery>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let removed = blocking({
        let db = db.clone();
        move || db.delete_entry(&user, &id, query.recursive)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "removed": removed, "seq": seq(&db) })))
}

/// Replace a file's bytes; the old version is kept.
pub async fn replace_file(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    body: Body,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    // Refuse a caller who may not write before reading the bytes.
    db.read(|state| {
        let e = state.workspace.iter().find(|e| e.id == id).ok_or_else(|| Error::not_found("workspace entry"))?;
        if e.owner != user.id {
            return Err(Error::forbidden("only its owner changes a workspace"));
        }
        Ok(())
    })?;
    let staged = receive(&db, &headers, body).await?;
    let media = media_type(&headers);
    let changed = blocking({
        let db = db.clone();
        move || db.replace_file(&user, &id, &media, staged)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": changed, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct ContentQuery {
    /// Which version; absent or 0 is the current one.
    #[serde(default)]
    pub version: u32,
    #[serde(default)]
    pub inline: bool,
}

/// Download a file, or one of its versions.
pub async fn download(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Query(query): Query<ContentQuery>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let (name, version) = db.workspace_file(&user, &id, query.version)?;
    stream_blob(&db, &name, &version.media_type, &version.sha256, version.size, query.inline).await
}

/// A file's versions, oldest first.
pub async fn versions(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(json!({ "versions": db.workspace_versions(&user, &id)? })))
}

/// Make an old version current again (as a new version).
pub async fn restore(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path((id, version)): Path<(String, u32)>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let changed = blocking({
        let db = db.clone();
        move || db.restore_version(&user, &id, version)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "entry": changed, "seq": seq(&db) })))
}

#[derive(Debug, Deserialize, Default)]
pub struct PromoteBody {
    /// A part id or number.
    pub part: String,
    /// A revision id or label; empty attaches to the part.
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub version: u32,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub kind: String,
    #[serde(default)]
    pub note: String,
}

/// Promote a file to an attachment on a part or revision.
pub async fn promote(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<PromoteBody>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let ask = Promote {
        part: body.part,
        revision: body.revision,
        version: body.version,
        name: body.name,
        kind: body.kind,
        note: body.note,
    };
    let made = blocking({
        let db = db.clone();
        move || db.promote(&user, &id, &ask)
    })
    .await?;
    Ok(Json(json!({ "ok": true, "attachment": made, "seq": seq(&db) })))
}
