use super::{blocking, require_admin, require_user, Shared};
use crate::{
    workflow::{self, Definition},
    Error,
};
use axum::{
    extract::{Path, State},
    http::HeaderMap,
    Json,
};
use serde::Deserialize;
use serde_json::{json, Map, Value};
pub async fn definitions(
    State(db): State<Shared>,
    headers: HeaderMap,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.read(|s| {
        json!(s
            .workflows
            .iter()
            .filter(|d| user.is_admin() || d.active)
            .map(|d| workflow::definition_view(d, &user))
            .collect::<Vec<_>>())
    })))
}
pub async fn save(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<Definition>,
) -> Result<Json<Definition>, Error> {
    let user = require_admin(&db, &headers)?;
    Ok(Json(db.save_workflow(&user, body)?))
}
pub async fn runs(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.read(|s| json!(workflow::run_list(s, &user)))))
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Start {
    pub workflow: String,
    #[serde(default)]
    pub title: String,
    #[serde(default)]
    pub part: String,
    #[serde(default)]
    pub revision: String,
    #[serde(default)]
    pub fields: Map<String, Value>,
}
pub async fn start(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<Start>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let id = blocking(move || {
        db.start_workflow(
            &user,
            &body.workflow,
            &body.title,
            &body.part,
            &body.revision,
            body.fields,
        )
    })
    .await?;
    Ok(Json(json!({"id":id})))
}
pub async fn detail(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    Ok(Json(db.workflow_view(&user, &id)?))
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Action {
    pub node: String,
    pub outcome: String,
    #[serde(default)]
    pub comment: String,
    #[serde(default)]
    pub fields: Map<String, Value>,
}
pub async fn act(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Action>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let target = id.clone();
    let handle = db.clone();
    blocking(move || {
        handle.workflow_action(
            &user,
            &target,
            &body.node,
            &body.outcome,
            &body.comment,
            body.fields,
        )
    })
    .await?;
    Ok(Json(json!({"ok":true,"seq":db.read(|s|s.seq)})))
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Control {
    pub action: String,
    #[serde(default)]
    pub node: String,
}
pub async fn control(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<Control>,
) -> Result<Json<Value>, Error> {
    let user = require_user(&db, &headers)?;
    let handle = db.clone();
    blocking(move || handle.control_workflow(&user, &id, &body.action, &body.node)).await?;
    Ok(Json(json!({"ok":true,"seq":db.read(|s|s.seq)})))
}
