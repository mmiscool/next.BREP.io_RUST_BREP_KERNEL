//! Credentials: changing a password, signing out everywhere, API tokens, and
//! the administrator's view of the sign-in throttle ([`crate::security`]).
//!
//! None of these routes accepts an API token ([`security::is_credential_route`]
//! and the request guard): a leaked token must not be able to mint another,
//! change its owner's password, or revoke the others to hide.

use axum::extract::{Path, Query, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::{Deserialize, Serialize};

use super::{identify, ok_seq, require_admin, require_user, Shared};
use crate::db::now;
use crate::model::{ApiToken, TokenScope, Timestamp};
use crate::security::{self, Lockout, ThrottleKind};
use crate::{auth, Error};

// ===========================================================================
// Passwords and sessions
// ===========================================================================

#[derive(Debug, Deserialize)]
pub struct PasswordChange {
    pub current: String,
    pub new: String,
}

/// Change your own password. The current one is required, and a wrong one
/// counts against the sign-in throttle like a failed sign-in. Every OTHER
/// session of the account ends; this one stays signed in. API tokens are left
/// alone — they are revoked on their own.
pub async fn change_password(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<PasswordChange>,
) -> Result<Response, Error> {
    let identity = identify(&db, &headers)?;
    let Some(session) = identity.session else {
        return Err(Error::forbidden("sign in with a password to change it"));
    };
    let user = identity.user;
    let throttle = &db.security().throttle;
    if let Some(seconds) = throttle.locked_for(ThrottleKind::User, &user.username, now()) {
        return Err(Error::too_many(format!("too many wrong passwords — try again in {seconds} s")));
    }
    if !auth::verify_password(&body.current, &user.password) {
        throttle.fail(ThrottleKind::User, &user.username, now());
        return Err(Error::forbidden("the current password is wrong"));
    }
    if body.new.len() < 8 {
        return Err(Error::bad_request("a password needs at least 8 characters"));
    }
    let verifier = auth::hash_password(&body.new);
    let user_id = user.id.clone();
    let ended = db.mutate(move |state| {
        let stored = state
            .users
            .iter_mut()
            .find(|u| u.id == user_id)
            .ok_or_else(|| Error::not_found("user"))?;
        stored.password = verifier;
        let before = state.sessions.len();
        state.sessions.retain(|s| s.user_id != user_id || s.token == session);
        Ok(before - state.sessions.len())
    })?;
    throttle.clear(ThrottleKind::User, &user.username);
    Ok(Json(serde_json::json!({ "ok": true, "other_sessions_ended": ended })).into_response())
}

/// Sign out everywhere: every session of this account ends, this one too.
pub async fn logout_all(State(db): State<Shared>, headers: HeaderMap) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    let ended = end_sessions(&db, &user.id)?;
    let cookie = super::clear_cookie(&db, &headers);
    Ok((
        StatusCode::OK,
        [(header::SET_COOKIE, cookie)],
        Json(serde_json::json!({ "ok": true, "sessions_ended": ended })),
    )
        .into_response())
}

/// An administrator signs a user out of every browser.
pub async fn sign_out_user(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    if db.read(|state| state.user(&id).is_none()) {
        return Err(Error::not_found("user"));
    }
    let ended = end_sessions(&db, &id)?;
    Ok(Json(serde_json::json!({ "ok": true, "sessions_ended": ended })).into_response())
}

fn end_sessions(db: &crate::db::Db, user_id: &str) -> Result<usize, Error> {
    let user_id = user_id.to_string();
    db.mutate(move |state| {
        let before = state.sessions.len();
        state.sessions.retain(|s| s.user_id != user_id);
        Ok(before - state.sessions.len())
    })
}

// ===========================================================================
// API tokens
// ===========================================================================

/// A token as the API reports it — never its secret or its hash.
#[derive(Debug, Serialize)]
pub struct TokenView {
    pub id: String,
    pub user_id: String,
    pub username: String,
    pub name: String,
    pub scope: TokenScope,
    pub prefix: String,
    pub created_at: Timestamp,
    pub last_used_at: Option<Timestamp>,
    pub expires_at: Option<Timestamp>,
    pub expired: bool,
}

fn view(token: &ApiToken, username: &str, now: Timestamp) -> TokenView {
    TokenView {
        id: token.id.clone(),
        user_id: token.user_id.clone(),
        username: username.to_string(),
        name: token.name.clone(),
        scope: token.scope,
        prefix: token.prefix.clone(),
        created_at: token.created_at,
        last_used_at: token.last_used_at,
        expires_at: token.expires_at,
        expired: token.expires_at.is_some_and(|at| at <= now),
    }
}

#[derive(Debug, Deserialize, Default)]
pub struct TokenQuery {
    /// An administrator's `?all=true`: every user's tokens.
    #[serde(default)]
    pub all: bool,
}

/// Your tokens; an administrator's `?all=true` lists everyone's.
pub async fn list_tokens(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<TokenQuery>,
) -> Result<Json<Vec<TokenView>>, Error> {
    let user = require_user(&db, &headers)?;
    let everyone = query.all && user.is_admin();
    let t = now();
    Ok(Json(db.read(|state| {
        state
            .api_tokens
            .iter()
            .filter(|token| everyone || token.user_id == user.id)
            .map(|token| {
                let owner = state.user(&token.user_id).map(|u| u.username.as_str()).unwrap_or("");
                view(token, owner, t)
            })
            .collect()
    })))
}

#[derive(Debug, Deserialize)]
pub struct NewToken {
    pub name: String,
    #[serde(default)]
    pub scope: TokenScope,
    /// Days until it stops working; absent or 0 for never.
    #[serde(default)]
    pub expires_days: Option<u64>,
    /// An administrator may make a token for another user — a worker account
    /// that never signs in with a password, say.
    #[serde(default)]
    pub user_id: Option<String>,
}

#[derive(Debug, Serialize)]
pub struct CreatedToken {
    /// The secret. Shown this once; the server keeps only its hash.
    pub token: String,
    #[serde(flatten)]
    pub view: TokenView,
}

pub async fn create_token(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewToken>,
) -> Result<Json<CreatedToken>, Error> {
    let user = require_user(&db, &headers)?;
    let name = body.name.trim().to_string();
    if name.is_empty() {
        return Err(Error::bad_request("a token needs a name, so you can tell it apart later"));
    }
    let owner_id = match body.user_id.as_deref().map(str::trim).filter(|id| !id.is_empty()) {
        Some(id) if id != user.id => {
            if !user.is_admin() {
                return Err(Error::forbidden("only an administrator makes tokens for someone else"));
            }
            id.to_string()
        }
        _ => user.id.clone(),
    };
    let secret = security::new_token_secret();
    let created = now();
    let token = ApiToken {
        id: auth::new_id(),
        user_id: owner_id.clone(),
        name,
        scope: body.scope,
        hash: security::token_hash(&secret),
        prefix: secret.chars().take(security::TOKEN_PREFIX.len() + 4).collect(),
        created_at: created,
        last_used_at: None,
        expires_at: body.expires_days.filter(|d| *d > 0).map(|d| created + d * 86_400),
    };
    let stored = token.clone();
    let username = db.mutate(move |state| {
        let owner = state.user(&owner_id).ok_or_else(|| Error::not_found("user"))?.username.clone();
        state.api_tokens.push(stored);
        Ok(owner)
    })?;
    Ok(Json(CreatedToken { token: secret, view: view(&token, &username, created) }))
}

/// Revoke a token: its owner, or an administrator.
pub async fn revoke_token(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
) -> Result<Response, Error> {
    let user = require_user(&db, &headers)?;
    db.mutate(move |state| {
        let token = state.api_tokens.iter().find(|t| t.id == id).ok_or_else(|| Error::not_found("token"))?;
        if token.user_id != user.id && !user.is_admin() {
            return Err(Error::forbidden("that token is someone else's"));
        }
        state.api_tokens.retain(|t| t.id != id);
        Ok(())
    })?;
    Ok(ok_seq(&db))
}

// ===========================================================================
// The sign-in throttle, for an administrator
// ===========================================================================

pub async fn list_lockouts(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<Vec<Lockout>>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.security().throttle.list(now())))
}

#[derive(Debug, Deserialize, Default)]
pub struct ClearQuery {
    /// `user` or `ip`; with `key`, clear just that one. Without, clear all.
    #[serde(default)]
    pub kind: Option<ThrottleKind>,
    #[serde(default)]
    pub key: Option<String>,
}

pub async fn clear_lockouts(
    State(db): State<Shared>,
    headers: HeaderMap,
    Query(query): Query<ClearQuery>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    let throttle = &db.security().throttle;
    let cleared = match (query.kind, query.key) {
        (Some(kind), Some(key)) => usize::from(throttle.clear(kind, &key)),
        (None, None) => throttle.clear_all(),
        _ => return Err(Error::bad_request("give both kind and key, or neither to clear everything")),
    };
    Ok(Json(serde_json::json!({ "ok": true, "cleared": cleared })).into_response())
}
