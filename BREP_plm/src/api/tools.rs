//! The tools library over HTTP ([`crate::tools`]): anyone signed in reads
//! it; the author group adds, edits, retires and pictures tools.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap};
use axum::response::Response;
use axum::Json;
use serde::{Deserialize, Serialize};
use serde_json::Value;

use super::attachments::{receive, stream_blob};
use super::{blocking, require_author, require_user, Shared};
use crate::tools::Tool;
use crate::Error;

/// A tool as the API shows it: its fields, and where its picture is.
#[derive(Debug, Serialize)]
pub struct ToolView {
    #[serde(flatten)]
    pub tool: Tool,
    pub picture_url: Option<String>,
}

fn view(tool: Tool) -> ToolView {
    // A bare path: the instructions player accepts exactly this form.
    let picture_url = tool.picture.as_ref().map(|_| format!("/api/tools/{}/picture", tool.id));
    ToolView { tool, picture_url }
}

#[derive(Debug, Deserialize, Default)]
pub struct ListQuery {
    /// Retired tools too (an admin page); the picker leaves them out.
    #[serde(default)]
    pub retired: bool,
}

/// `GET /api/tools`: the library by name.
pub async fn list(State(db): State<Shared>, headers: HeaderMap, Query(query): Query<ListQuery>) -> Result<Json<Vec<ToolView>>, Error> {
    require_user(&db, &headers)?;
    let mut tools = db.read(|state| state.tools.iter().filter(|t| query.retired || !t.retired).cloned().collect::<Vec<_>>());
    tools.sort_by_key(|t| (t.name.to_lowercase(), t.id.clone()));
    Ok(Json(tools.into_iter().map(view).collect()))
}

pub async fn get(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Json<ToolView>, Error> {
    require_user(&db, &headers)?;
    let tool = db.read(|state| state.tools.iter().find(|t| t.id == id).cloned()).ok_or_else(|| Error::not_found("tool"))?;
    Ok(Json(view(tool)))
}

/// `POST /api/tools {name, spec?, kind?}`.
pub async fn create(State(db): State<Shared>, headers: HeaderMap, Json(fields): Json<Value>) -> Result<Json<ToolView>, Error> {
    let user = require_author(&db, &headers)?;
    Ok(Json(view(blocking(move || db.create_tool(&user, &fields)).await?)))
}

/// `PATCH /api/tools/:id {name?, spec?, kind?, retired?}`: a tool is retired,
/// never deleted.
pub async fn update(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>, Json(fields): Json<Value>) -> Result<Json<ToolView>, Error> {
    require_author(&db, &headers)?;
    Ok(Json(view(blocking(move || db.update_tool(&id, &fields)).await?)))
}

/// `PUT /api/tools/:id/picture`: the body is the image, its type the
/// `Content-Type`.
pub async fn set_picture(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>, body: Body) -> Result<Json<ToolView>, Error> {
    require_author(&db, &headers)?;
    if db.read(|state| !state.tools.iter().any(|t| t.id == id)) {
        return Err(Error::not_found("tool"));
    }
    let staged = receive(&db, &headers, body).await?;
    let media_type = headers.get(header::CONTENT_TYPE).and_then(|v| v.to_str().ok()).unwrap_or("").to_string();
    Ok(Json(view(blocking(move || db.set_tool_picture(&id, staged, &media_type)).await?)))
}

pub async fn picture(State(db): State<Shared>, headers: HeaderMap, Path(id): Path<String>) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let (tool, picture) = db
        .read(|state| state.tools.iter().find(|t| t.id == id).and_then(|t| Some((t.id.clone(), t.picture.clone()?))))
        .ok_or_else(|| Error::not_found("tool picture"))?;
    stream_blob(&db, &format!("{tool}-picture"), &picture.media_type, &picture.sha256, picture.size, true).await
}
