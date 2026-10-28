//! The instructions player over HTTP ([`crate::player`]): its data, its page
//! per revision, and customer links.

use axum::body::Body;
use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, HeaderValue, StatusCode};
use axum::response::{IntoResponse, Redirect, Response};
use axum::Json;
use brep_instructions::player::{served_html, Mode};
use serde::Deserialize;
use serde_json::{json, Value};

use super::attachments::stream_blob;
use super::{blocking, require_author, require_user, Shared};
use crate::Error;

#[derive(Debug, Deserialize, Default)]
pub struct PutQuery {
    /// The revision's document the data was made from.
    #[serde(default)]
    pub content_hash: String,
}

/// `PUT /api/parts/:id/revisions/:rev/player-data?content_hash=`: the body is
/// the crate's `PlayerData` JSON, checked before it is kept.
pub async fn put(State(db): State<Shared>, headers: HeaderMap, Path((id, rev)): Path<(String, String)>, Query(query): Query<PutQuery>, body: Body) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let bytes = axum::body::to_bytes(body, crate::player::MAX_BYTES as usize).await.map_err(|_| Error {
        status: StatusCode::PAYLOAD_TOO_LARGE,
        message: format!("player data is at most {} MB", crate::player::MAX_BYTES / (1024 * 1024)),
    })?;
    let text = std::str::from_utf8(&bytes).map_err(|_| Error::bad_request("player data is UTF-8 JSON"))?;
    brep_instructions::player::PlayerData::from_json(text).map_err(Error::bad_request)?;
    let mut upload = db.start_upload()?;
    upload.write(&bytes)?;
    let staged = upload.finish()?;
    let kept = blocking(move || db.put_player_data(&user, &id, &rev, staged, &query.content_hash)).await?;
    Ok(Json(json!({ "ok": true, "player": kept })))
}

/// `GET /api/parts/:id/revisions/:rev/player-data`: the current data, or
/// `404` when there is none or the document moved since.
pub async fn get(State(db): State<Shared>, headers: HeaderMap, Path((id, rev)): Path<(String, String)>) -> Result<Response, Error> {
    require_user(&db, &headers)?;
    let (_, _, data) = db.player_data(&id, &rev)?;
    let data = data.ok_or_else(|| Error::not_found("current player data (the CAD app makes it when it saves the instructions)"))?;
    stream_blob(&db, "player-data.json", "application/json", &data.sha256, data.size, false).await
}

#[derive(Debug, Deserialize, Default)]
pub struct PageQuery {
    #[serde(default)]
    pub mode: String,
    #[serde(default)]
    pub job: String,
}

/// The page: the crate's template under its own policy, a nonce per response.
fn page(data_url: &str, mode: Mode, job: Option<&str>) -> Result<Response, Error> {
    let nonce = crate::auth::random_token(18);
    let html = served_html(&nonce, data_url, mode, job).map_err(Error::internal)?;
    let mut response = (StatusCode::OK, html).into_response();
    let headers = response.headers_mut();
    headers.insert(header::CONTENT_TYPE, HeaderValue::from_static("text/html; charset=utf-8"));
    headers.insert(header::CONTENT_SECURITY_POLICY, HeaderValue::from_str(&crate::player::policy(&nonce)).map_err(Error::internal)?);
    headers.insert(header::CACHE_CONTROL, HeaderValue::from_static("no-store"));
    Ok(response)
}

/// `GET /instructions/:number/:rev?mode=shop|customer&job=JOB…`: the player
/// for anyone signed in; manufacturing instructions only (an ordinary part's
/// study plays at `/motion/…`). Signed out, the PLM's sign-in page.
pub async fn view(State(db): State<Shared>, headers: HeaderMap, Path((number, rev)): Path<(String, String)>, Query(query): Query<PageQuery>) -> Result<Response, Error> {
    if require_user(&db, &headers).is_err() {
        return Ok(Redirect::to("/").into_response());
    }
    let mode = match query.mode.trim() {
        "" => Mode::Shop,
        text => Mode::parse(text).ok_or_else(|| Error::bad_request("mode is shop or customer"))?,
    };
    let (part, revision, _) = db.instructions_player_data(&number, &rev)?;
    let job = match query.job.trim() {
        "" => None,
        job => {
            let builds = db.read(|state| state.jobs.get(job).map(|j| j.instructions.part == part && j.instructions.revision == revision));
            match builds {
                None => return Err(Error::not_found("job")),
                Some(false) => return Err(Error::bad_request(format!("{job} builds other instructions"))),
                Some(true) => Some(job.to_string()),
            }
        }
    };
    let data_url = format!("/api/parts/{}/revisions/{}/player-data", crate::identity::segment(&part), crate::identity::segment(&revision));
    page(&data_url, mode, job.as_deref())
}

/// `GET /motion/:number/:rev`: an ordinary part's motion-study player (the
/// same template with the study's frames on a timeline, customer mode, no
/// job) for anyone signed in. Signed out, the PLM's sign-in page.
pub async fn motion_view(State(db): State<Shared>, headers: HeaderMap, Path((number, rev)): Path<(String, String)>) -> Result<Response, Error> {
    if require_user(&db, &headers).is_err() {
        return Ok(Redirect::to("/").into_response());
    }
    let (part, revision, _) = db.player_data(&number, &rev)?;
    let data_url = format!("/api/parts/{}/revisions/{}/player-data", crate::identity::segment(&part), crate::identity::segment(&revision));
    page(&data_url, Mode::Customer, None)
}

#[derive(Debug, Deserialize, Default)]
pub struct NewShare {
    #[serde(default)]
    pub label: String,
}

/// `POST /api/parts/:id/revisions/:rev/shares {label?}`: a customer link to a
/// released revision. The token is in this answer only.
pub async fn create_share(State(db): State<Shared>, headers: HeaderMap, Path((id, rev)): Path<(String, String)>, Json(body): Json<NewShare>) -> Result<Json<Value>, Error> {
    let user = require_author(&db, &headers)?;
    let (share, token) = blocking(move || db.create_share(&user, &id, &rev, &body.label)).await?;
    Ok(Json(json!({ "id": share.id, "label": share.label, "created_at": share.created_at, "url": format!("/s/{token}") })))
}

/// `GET /api/parts/:id/revisions/:rev/shares`: the links, without their tokens.
pub async fn list_shares(State(db): State<Shared>, headers: HeaderMap, Path((id, rev)): Path<(String, String)>) -> Result<Json<Value>, Error> {
    require_author(&db, &headers)?;
    db.read(|state| {
        let part = state.part_by_id_or_number(&id).ok_or_else(|| Error::not_found("part"))?;
        let revision = crate::bom::find_revision(part, &rev).ok_or_else(|| Error::not_found("revision"))?;
        let name = |uid: &str| state.user(uid).map(|u| u.username.clone()).unwrap_or_default();
        Ok(Json(Value::Array(
            revision.shares.iter().map(|s| json!({ "id": s.id, "label": s.label, "created_by": name(&s.created_by), "created_at": s.created_at })).collect(),
        )))
    })
}

/// `DELETE /api/parts/:id/revisions/:rev/shares/:share`: the link stops working.
pub async fn revoke_share(State(db): State<Shared>, headers: HeaderMap, Path((id, rev, share)): Path<(String, String, String)>) -> Result<Json<Value>, Error> {
    require_author(&db, &headers)?;
    blocking(move || db.revoke_share(&id, &rev, &share)).await?;
    Ok(Json(json!({ "ok": true })))
}

/// `GET /s/:token`: a customer link, in customer mode. No sign-in.
pub async fn shared(State(db): State<Shared>, Path(token): Path<String>) -> Result<Response, Error> {
    db.shared_player(&token).ok_or_else(|| Error::not_found("link"))?;
    page(&format!("/s/{token}/data"), Mode::Customer, None)
}

/// `GET /s/:token/data`: that link's player data, and nothing else.
pub async fn shared_data(State(db): State<Shared>, Path(token): Path<String>) -> Result<Response, Error> {
    let (what, data) = db.shared_player(&token).ok_or_else(|| Error::not_found("link"))?;
    let data = data.ok_or_else(|| Error::not_found(format!("player data for {what}")))?;
    stream_blob(&db, "player-data.json", "application/json", &data.sha256, data.size, false).await
}
