//! Server-owned worker controls; all three routes require an administrator.
use super::{require_admin, Shared};
use crate::{bake_worker::Status, Error};
use axum::{extract::State, http::HeaderMap, Json};

pub async fn status(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Status>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(
        db.bake_worker
            .status(db.security().config.bake_worker.as_ref())?,
    ))
}

pub async fn start(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Status>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.bake_worker.start(
        db.security().config.bake_worker.as_ref(),
        db.root(),
    )?))
}

pub async fn stop(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Status>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(
        db.bake_worker
            .stop(db.security().config.bake_worker.as_ref())?,
    ))
}
