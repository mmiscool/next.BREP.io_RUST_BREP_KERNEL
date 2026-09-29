//! Sign-in, user administration and part types.
//!
//! Part types live here rather than with parts because they are an
//! ADMINISTRATIVE object: they define the numbering scheme an operator
//! configures once, and creating one is not part of designing anything.

use std::net::SocketAddr;

use axum::extract::{ConnectInfo, Path, State};
use axum::http::{header, HeaderMap, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::Json;
use serde::{Deserialize, Serialize};

use super::{ok_seq, require_admin, require_user, Shared};
use crate::db::now;
use crate::model::{groups, NumberMode, PartType, Session, User};
use crate::security::ThrottleKind;
use crate::{auth, Error};

/// A user as the API reports them — never with the password verifier.
#[derive(Debug, Serialize)]
pub struct PublicUser {
    pub id: String,
    pub username: String,
    pub display_name: String,
    pub email: String,
    pub groups: Vec<String>,
    pub active: bool,
    pub created_at: u64,
    /// What this account may do, resolved by the server so the page does not
    /// re-implement the group rules and drift from them.
    pub can_author: bool,
    pub can_checkin: bool,
    pub is_admin: bool,
    pub can_bake: bool,
    /// The session's CSRF token, on `/api/login` and `/api/me` only: the page
    /// sends it back in `X-CSRF-Token` with every change.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub csrf: Option<String>,
    /// How this request signed in: `session` or `token`, on `/api/me` only.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub via: Option<&'static str>,
    /// The version handshake, on `/api/me` only ([`crate::version`]): this
    /// server's version and the oldest CAD app it serves. The app refuses to
    /// connect when its own version is older than `min_client_version`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub server_version: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_client_version: Option<String>,
    /// What this server can do, by name ([`crate::version::FEATURES`]).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub features: Option<&'static [&'static str]>,
}

impl From<&User> for PublicUser {
    fn from(user: &User) -> Self {
        Self {
            id: user.id.clone(),
            username: user.username.clone(),
            display_name: user.display_name.clone(),
            email: user.email.clone(),
            groups: user.groups.clone(),
            active: user.active,
            created_at: user.created_at,
            can_author: user.can_author(),
            can_checkin: user.can_checkin(),
            is_admin: user.is_admin(),
            can_bake: user.can_bake(),
            csrf: None,
            via: None,
            server_version: None,
            min_client_version: None,
            features: None,
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct Credentials {
    pub username: String,
    pub password: String,
}

/// Sign in. A wrong username and a wrong password answer identically — the
/// difference would tell an attacker which half to keep working on.
///
/// Failures are counted per username and per client address
/// ([`crate::security::Throttle`]). A locked-out key is refused with `429`
/// BEFORE the password is checked, so guessing during a lockout learns
/// nothing; the refusal names neither which key is locked nor whether the
/// account exists.
pub async fn login(
    State(db): State<Shared>,
    headers: HeaderMap,
    peer: Option<ConnectInfo<SocketAddr>>,
    Json(body): Json<Credentials>,
) -> Result<Response, Error> {
    let security = db.security();
    let ip = security.config.client_ip(&headers, peer.map(|ConnectInfo(addr)| addr.ip()));
    let ip_key = ip.map(|ip| ip.to_string());
    let username = body.username.trim().to_ascii_lowercase();
    let t = now();
    let locked = security
        .throttle
        .locked_for(ThrottleKind::User, &username, t)
        .into_iter()
        .chain(ip_key.as_deref().and_then(|ip| security.throttle.locked_for(ThrottleKind::Ip, ip, t)))
        .max();
    let failed = |db: &crate::db::Db, user_id: &str, reason: &str| {
        let who = crate::model::EntityRef {
            kind: "user".into(),
            id: user_id.to_string(),
            label: body.username.trim().to_string(),
            part_id: String::new(),
        };
        db.record("sign-in-failed", who, reason);
    };
    if let Some(seconds) = locked {
        failed(&db, "", "locked out by the sign-in throttle");
        return Err(Error::too_many(format!(
            "too many failed sign-ins — try again in {seconds} s, or ask an administrator to clear the lockout"
        )));
    }

    let found = db.read(|state| state.user_by_name(&body.username).cloned());
    let refusal = || {
        security.throttle.fail(ThrottleKind::User, &username, t);
        if let Some(ip) = &ip_key {
            security.throttle.fail(ThrottleKind::Ip, ip, t);
        }
        Error::unauthorized("wrong username or password")
    };

    let user = match found {
        Some(user) if auth::verify_password(&body.password, &user.password) => user,
        Some(user) => {
            failed(&db, &user.id, "wrong password");
            return Err(refusal());
        }
        None => {
            failed(&db, "", "no such username");
            // Spend the same work on an unknown user as on a known one, so the
            // response time does not reveal which it was.
            static DECOY: std::sync::OnceLock<String> = std::sync::OnceLock::new();
            let decoy = DECOY.get_or_init(|| auth::hash_password("decoy"));
            let _ = auth::verify_password(&body.password, decoy);
            return Err(refusal());
        }
    };
    if !user.active {
        failed(&db, &user.id, "the account is disabled");
        return Err(Error::forbidden("this account is disabled"));
    }
    security.throttle.clear(ThrottleKind::User, &username);

    let token = auth::random_token(32);
    let csrf = auth::random_token(32);
    let session = Session {
        token: token.clone(),
        user_id: user.id.clone(),
        created_at: now(),
        last_seen: now(),
        csrf: csrf.clone(),
    };
    // The new session is logged as a sign-in BY this user: until now the
    // request had no one signed in.
    let actor = crate::model::Actor {
        user_id: user.id.clone(),
        username: user.username.clone(),
        via: "session".into(),
        ..crate::audit::current_actor()
    };
    crate::audit::as_actor(actor, || {
        db.mutate(move |state| {
            state.sessions.push(session);
            Ok(())
        })
    })?;
    // Sessions nobody can use any more go as someone signs in, so the list
    // never grows past the ones that could still be used.
    if let Err(error) = db.prune_sessions() {
        eprintln!("brep-plm: could not remove expired sessions: {}", error.message);
    }

    let cookie = super::session_cookie(&db, &headers, &token);
    let mut public = PublicUser::from(&user);
    public.csrf = Some(csrf);
    Ok((StatusCode::OK, [(header::SET_COOKIE, cookie)], Json(public)).into_response())
}

pub async fn logout(State(db): State<Shared>, headers: HeaderMap) -> Result<Response, Error> {
    if let Some(token) = super::session_token(&headers) {
        db.mutate(move |state| {
            state.sessions.retain(|s| s.token != token);
            Ok(())
        })?;
    }
    let cookie = super::clear_cookie(&db, &headers);
    Ok((StatusCode::OK, [(header::SET_COOKIE, cookie)], Json(serde_json::json!({"ok": true})))
        .into_response())
}

/// Who am I, and what may I do. For a browser this also hands out the
/// session's CSRF token, issuing one for a session from before hardening.
pub async fn me(State(db): State<Shared>, headers: HeaderMap) -> Result<Json<PublicUser>, Error> {
    let identity = super::identify(&db, &headers)?;
    let mut public = PublicUser::from(&identity.user);
    public.server_version = Some(crate::version::SERVER_VERSION);
    public.min_client_version = Some(db.security().config.min_client_version().to_string());
    public.features = Some(crate::version::FEATURES);
    if let Some(token) = identity.session {
        let csrf = db.mutate(move |state| {
            let session = state
                .sessions
                .iter_mut()
                .find(|s| s.token == token)
                .ok_or_else(|| Error::unauthorized("session has expired"))?;
            if session.csrf.is_empty() {
                session.csrf = auth::random_token(32);
            }
            Ok(session.csrf.clone())
        })?;
        public.csrf = Some(csrf);
        public.via = Some("session");
    } else {
        public.via = Some("token");
    }
    Ok(Json(public))
}

// ===========================================================================
// Users
// ===========================================================================

pub async fn list_users(
    State(db): State<Shared>,
    headers: HeaderMap,
) -> Result<Json<Vec<PublicUser>>, Error> {
    require_admin(&db, &headers)?;
    Ok(Json(db.read(|state| {
        state.users.iter().map(PublicUser::from).collect()
    })))
}

#[derive(Debug, Deserialize)]
pub struct NewUser {
    pub username: String,
    pub password: String,
    #[serde(default)]
    pub display_name: String,
    #[serde(default)]
    pub email: String,
    #[serde(default)]
    pub groups: Vec<String>,
}

pub async fn create_user(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewUser>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    let username = body.username.trim().to_string();
    if username.is_empty() {
        return Err(Error::bad_request("a user needs a username"));
    }
    if body.password.len() < 8 {
        return Err(Error::bad_request("a password needs at least 8 characters"));
    }
    let user = User {
        id: auth::new_id(),
        username,
        display_name: body.display_name.trim().to_string(),
        email: body.email.trim().to_string(),
        password: auth::hash_password(&body.password),
        groups: normalize_groups(body.groups),
        active: true,
        created_at: now(),
    };
    db.mutate(move |state| {
        if state.user_by_name(&user.username).is_some() {
            return Err(Error::conflict(format!(
                "username '{}' is taken",
                user.username
            )));
        }
        state.users.push(user);
        Ok(())
    })?;
    Ok(ok_seq(&db))
}

#[derive(Debug, Deserialize)]
pub struct UserPatch {
    pub display_name: Option<String>,
    pub email: Option<String>,
    pub groups: Option<Vec<String>>,
    pub active: Option<bool>,
    pub password: Option<String>,
}

pub async fn update_user(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<UserPatch>,
) -> Result<Response, Error> {
    let actor = require_admin(&db, &headers)?;
    if let Some(password) = &body.password {
        if password.len() < 8 {
            return Err(Error::bad_request("a password needs at least 8 characters"));
        }
    }
    let mut reset_for: Option<String> = None;
    let reset = &mut reset_for;
    db.mutate(move |state| {
        let reset_for = reset;
        // Locking yourself out of the only admin account is unrecoverable
        // without editing the file by hand, so it is refused rather than
        // warned about.
        let losing_admin = body.active == Some(false)
            || body
                .groups
                .as_ref()
                .is_some_and(|g| !g.iter().any(|name| name == groups::ADMIN));
        if id == actor.id && losing_admin {
            return Err(Error::conflict(
                "an administrator cannot remove their own admin access".to_string(),
            ));
        }
        let user = state
            .users
            .iter_mut()
            .find(|u| u.id == id)
            .ok_or_else(|| Error::not_found("user"))?;
        if let Some(name) = body.display_name {
            user.display_name = name.trim().to_string();
        }
        if let Some(email) = body.email {
            user.email = email.trim().to_string();
        }
        if let Some(list) = body.groups {
            user.groups = normalize_groups(list);
        }
        if let Some(active) = body.active {
            user.active = active;
        }
        if let Some(password) = body.password {
            user.password = auth::hash_password(&password);
            *reset_for = Some(user.username.clone());
            // A password change ends every other session of that account.
            let user_id = user.id.clone();
            state.sessions.retain(|s| s.user_id != user_id);
        }
        Ok(())
    })?;
    // A reset password is a fresh start for that account's sign-in throttle.
    if let Some(username) = reset_for {
        db.security().throttle.clear(ThrottleKind::User, &username);
    }
    Ok(ok_seq(&db))
}

/// Trim, lower-case, drop blanks and duplicates. Group names are compared
/// exactly everywhere else, so they are normalized exactly once — here.
fn normalize_groups(list: Vec<String>) -> Vec<String> {
    let mut out: Vec<String> = Vec::new();
    for name in list {
        let name = name.trim().to_ascii_lowercase();
        if !name.is_empty() && !out.contains(&name) {
            out.push(name);
        }
    }
    out
}

// ===========================================================================
// Part types — the numbering schemes
// ===========================================================================

pub async fn list_part_types(
    State(db): State<Shared>,
    headers: HeaderMap,
) -> Result<Json<Vec<PartTypeView>>, Error> {
    require_user(&db, &headers)?;
    Ok(Json(db.read(|state| {
        state.part_types.iter().map(PartTypeView::from).collect()
    })))
}

/// A part type plus what its NEXT number would look like — the page shows it
/// so an operator can see the scheme before committing to it.
#[derive(Debug, Serialize)]
pub struct PartTypeView {
    pub id: String,
    pub name: String,
    pub prefix: String,
    pub digits: u32,
    pub next: u64,
    /// What the counter would hand out next. Meaningful only in counter mode;
    /// the page shows it only then.
    pub next_number: String,
    pub capacity: u64,
    pub mode: NumberMode,
}

impl From<&PartType> for PartTypeView {
    fn from(kind: &PartType) -> Self {
        Self {
            id: kind.id.clone(),
            name: kind.name.clone(),
            prefix: kind.prefix.clone(),
            digits: kind.digits,
            next: kind.next,
            next_number: kind.format_number(kind.next),
            capacity: kind.capacity(),
            mode: kind.mode.clone(),
        }
    }
}

#[derive(Debug, Deserialize)]
pub struct NewPartType {
    pub id: String,
    pub name: String,
    /// Counter mode only; the other modes keep it for a later switch back.
    #[serde(default)]
    pub prefix: String,
    #[serde(default = "nine")]
    pub digits: u32,
    #[serde(default)]
    pub start: Option<u64>,
    /// `{ "kind": "counter" | "free" | "pattern" | "script", ... }`. Absent is
    /// a counter, as every type was before modes existed.
    #[serde(default)]
    pub mode: NumberMode,
}

fn nine() -> u32 {
    9
}

/// A mode an administrator may save: a pattern that compiles, a script path
/// inside the scripts directory. The script file need not exist yet — a part
/// created before it does is refused with a message naming it.
fn check_mode(db: &crate::db::Db, mode: &NumberMode) -> Result<(), Error> {
    match mode {
        NumberMode::Counter | NumberMode::Free => Ok(()),
        NumberMode::Pattern { regex } => {
            if regex.trim().is_empty() {
                return Err(Error::bad_request("a pattern type needs a pattern"));
            }
            crate::db::full_match(regex).map(|_| ())
        }
        NumberMode::Script { script } => db.scripts().resolve(script).map(|_| ()),
    }
}

/// Two COUNTER types sharing a prefix can mint the same string. (Any clash
/// that still happens — a typed number landing in a counter's range — is
/// caught by the store's one-number-one-part check.)
fn prefix_clash<'a>(state: &'a crate::db::State, kind: &PartType) -> Option<&'a PartType> {
    if kind.mode != NumberMode::Counter {
        return None;
    }
    state.part_types.iter().find(|t| {
        t.id != kind.id && t.mode == NumberMode::Counter && t.prefix.eq_ignore_ascii_case(&kind.prefix)
    })
}

pub async fn create_part_type(
    State(db): State<Shared>,
    headers: HeaderMap,
    Json(body): Json<NewPartType>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    let id = body.id.trim().to_ascii_lowercase();
    if id.is_empty() || !id.chars().all(|c| c.is_ascii_alphanumeric() || c == '-' || c == '_') {
        return Err(Error::bad_request(
            "a part type id is letters, digits, '-' and '_'",
        ));
    }
    // 18 digits is where u64 stops being able to count the sequence; 1 is the
    // smallest scheme that can spell anything at all.
    if !(1..=18).contains(&body.digits) {
        return Err(Error::bad_request("digits must be between 1 and 18"));
    }
    let kind = PartType {
        id,
        name: body.name.trim().to_string(),
        prefix: body.prefix.trim().to_string(),
        digits: body.digits,
        next: body.start.unwrap_or(1).max(1),
        created_at: now(),
        mode: body.mode,
    };
    check_mode(&db, &kind.mode)?;
    if kind.name.is_empty() {
        return Err(Error::bad_request("a part type needs a name"));
    }
    if kind.next > kind.capacity() {
        return Err(Error::bad_request(format!(
            "a start of {} does not fit in {} digits",
            kind.next, kind.digits
        )));
    }
    db.mutate(move |state| {
        if state.part_type(&kind.id).is_some() {
            return Err(Error::conflict(format!("part type '{}' exists", kind.id)));
        }
        if let Some(clash) = prefix_clash(state, &kind) {
            return Err(Error::conflict(format!(
                "prefix '{}' is already used by part type '{}'",
                kind.prefix, clash.id
            )));
        }
        state.part_types.push(kind);
        Ok(())
    })?;
    Ok(ok_seq(&db))
}

#[derive(Debug, Deserialize)]
pub struct PartTypeChange {
    #[serde(default)]
    pub name: Option<String>,
    #[serde(default)]
    pub mode: Option<NumberMode>,
}

/// Rename a part type or change how it numbers. Parts already numbered keep
/// their numbers; the change applies to the next part created.
pub async fn update_part_type(
    State(db): State<Shared>,
    headers: HeaderMap,
    Path(id): Path<String>,
    Json(body): Json<PartTypeChange>,
) -> Result<Response, Error> {
    require_admin(&db, &headers)?;
    if let Some(mode) = &body.mode {
        check_mode(&db, mode)?;
    }
    db.mutate(move |state| {
        let index = state
            .part_types
            .iter()
            .position(|t| t.id == id)
            .ok_or_else(|| Error::not_found("part type"))?;
        let mut kind = state.part_types[index].clone();
        if let Some(name) = body.name {
            let name = name.trim().to_string();
            if name.is_empty() {
                return Err(Error::bad_request("a part type needs a name"));
            }
            kind.name = name;
        }
        if let Some(mode) = body.mode {
            kind.mode = mode;
        }
        if let Some(clash) = prefix_clash(state, &kind) {
            return Err(Error::conflict(format!(
                "prefix '{}' is already used by counter type '{}'",
                kind.prefix, clash.id
            )));
        }
        state.part_types[index] = kind;
        Ok(())
    })?;
    Ok(ok_seq(&db))
}
