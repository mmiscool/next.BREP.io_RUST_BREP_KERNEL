//! The desktop app's browser sign-in ([`crate::desktop_link`]) over HTTP.
//!
//! * `POST /api/desktop/link` — no credential. Registers a link; rate-limited
//!   per address.
//! * `POST /api/desktop/link/poll` — no credential; the poll secret in the body.
//! * `POST /api/desktop/link/inspect|approve|deny` — a signed-in BROWSER (cookie
//!   and CSRF token); an API token is refused by the request guard
//!   ([`crate::security::is_credential_route`]).
//!
//! No handler here logs a token or a secret, and neither travels in a path.

use std::net::SocketAddr;

use axum::extract::{ConnectInfo, State};
use axum::http::HeaderMap;
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::Deserialize;

use super::{identify, Shared};
use crate::db::now;
use crate::desktop_link::{gone, Asker, LinkView, Poll, Registered};
use crate::model::{ApiToken, DesktopDevice, EntityRef, TokenScope};
use crate::{auth, security, Error};

pub async fn register(
    State(db): State<Shared>,
    headers: HeaderMap,
    peer: Option<ConnectInfo<SocketAddr>>,
    Json(asker): Json<Asker>,
) -> Result<Json<Registered>, Error> {
    let security = db.security();
    let ip = security.config.client_ip(&headers, peer.map(|ConnectInfo(addr)| addr.ip()));
    Ok(Json(security.desktop_links.register(asker, ip.map(|ip| ip.to_string()), now())?))
}

#[derive(Debug, Deserialize)]
pub struct LinkToken {
    #[serde(default)]
    pub token: String,
}

/// The browser must be a signed-in person's session, not a token.
fn require_browser(db: &crate::db::Db, headers: &HeaderMap) -> Result<super::Identity, Error> {
    let identity = identify(db, headers)?;
    if identity.session.is_none() {
        return Err(Error::forbidden("approve a desktop sign-in in a signed-in browser"));
    }
    Ok(identity)
}

pub async fn inspect(State(db): State<Shared>, headers: HeaderMap, Json(body): Json<LinkToken>) -> Result<Json<LinkView>, Error> {
    require_browser(&db, &headers)?;
    db.security().desktop_links.inspect(&body.token, now()).map(Json).ok_or_else(gone)
}

pub async fn approve(State(db): State<Shared>, headers: HeaderMap, Json(body): Json<LinkToken>) -> Result<Response, Error> {
    decide(&db, &headers, &body.token, true)
}

pub async fn deny(State(db): State<Shared>, headers: HeaderMap, Json(body): Json<LinkToken>) -> Result<Response, Error> {
    decide(&db, &headers, &body.token, false)
}

fn decide(db: &crate::db::Db, headers: &HeaderMap, token: &str, approve: bool) -> Result<Response, Error> {
    let identity = require_browser(db, headers)?;
    let actor = crate::audit::current_actor();
    let asker = db.security().desktop_links.decide(token, approve.then(|| actor.clone()), now())?;
    let (action, verb) = if approve { ("desktop-link-approved", "connected") } else { ("desktop-link-denied", "refused") };
    db.record(action, link_entity(&identity.user.id, &asker), format!("{verb} BREP desktop on {}", describe(&asker)));
    Ok(Json(serde_json::json!({ "ok": true, "approved": approve })).into_response())
}

#[derive(Debug, Deserialize)]
pub struct PollBody {
    #[serde(default)]
    pub poll_secret: String,
}

/// The app's poll. `approved` carries a full API token for the approver,
/// named for the machine, minted now and handed out this once.
pub async fn poll(State(db): State<Shared>, Json(body): Json<PollBody>) -> Result<Response, Error> {
    let status = match db.security().desktop_links.poll(&body.poll_secret, now()) {
        Poll::Unknown => return Err(gone()),
        Poll::Pending => "pending",
        Poll::Denied => "denied",
        Poll::Approved { by, asker } => {
            let name = format!("brep-app {}", if asker.host.is_empty() { "desktop" } else { &asker.host });
            let user_id = by.user_id.clone();
            let device = DesktopDevice { host: asker.host, os: asker.os, app_version: asker.app_version };
            // The token is the approver's, and the audit log says so: the
            // poll itself arrives with no one signed in.
            let (secret, username) = crate::audit::as_actor(by, || mint(&db, &user_id, name, device))?;
            return Ok(Json(serde_json::json!({ "status": "approved", "token": secret, "username": username })).into_response());
        }
    };
    Ok(Json(serde_json::json!({ "status": status })).into_response())
}

/// The desktop app signing out (its "Forget this machine's sign-in"): the
/// Linked desktop token it called with is revoked, so the sign-in ends on the
/// server too and leaves the account page's list. A token a person made and
/// pasted is left alone (it may serve other machines); a browser session has
/// no desktop to sign out.
pub async fn sign_out(State(db): State<Shared>, headers: HeaderMap) -> Result<Response, Error> {
    let identity = identify(&db, &headers)?;
    let Some((id, _)) = identity.token else {
        return Err(Error::bad_request("a browser signs out with /api/logout"));
    };
    let revoked = db.mutate(move |state| {
        let before = state.api_tokens.len();
        state.api_tokens.retain(|t| !(t.id == id && t.desktop.is_some()));
        Ok(state.api_tokens.len() != before)
    })?;
    Ok(Json(serde_json::json!({ "ok": true, "revoked": revoked })).into_response())
}

/// A full token for `user_id`, who must still be an active user.
fn mint(db: &crate::db::Db, user_id: &str, name: String, device: DesktopDevice) -> Result<(String, String), Error> {
    let secret = security::new_token_secret();
    let token = ApiToken {
        id: auth::new_id(),
        user_id: user_id.to_string(),
        name,
        scope: TokenScope::Full,
        hash: security::token_hash(&secret),
        prefix: secret.chars().take(security::TOKEN_PREFIX.len() + 4).collect(),
        created_at: now(),
        last_used_at: None,
        expires_at: None,
        desktop: Some(device),
    };
    let username = db.mutate(move |state| {
        let owner = state.user(&token.user_id).ok_or_else(|| Error::not_found("user"))?;
        if !owner.active {
            return Err(Error::forbidden("this account is disabled"));
        }
        let username = owner.username.clone();
        state.api_tokens.push(token);
        Ok(username)
    })?;
    Ok((secret, username))
}

fn link_entity(user_id: &str, asker: &Asker) -> EntityRef {
    EntityRef { kind: "desktop-link".into(), id: user_id.to_string(), label: describe(asker), part_id: String::new() }
}

fn describe(asker: &Asker) -> String {
    let host = if asker.host.is_empty() { "an unnamed machine" } else { &asker.host };
    if asker.os.is_empty() { host.to_string() } else { format!("{host} ({})", asker.os) }
}
