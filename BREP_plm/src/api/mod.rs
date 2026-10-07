//! The HTTP surface: the router, the session cookie, and the helpers every
//! handler shares.
//!
//! Two families of route live here and they answer different callers:
//!
//! * `/api/...` — the PLM itself: accounts, part types, the catalog, parts,
//!   revisions, uses lists, BOMs and where-used, families and templates,
//!   the bake queue, sourcing and settings.
//!   The browser page in `web/` is its only client today.
//! * `/api/store/...` — the DOCUMENT surface, shaped to what the CAD app's
//!   `StoreBackend` needs from a remote backend (see [`store`]).
//!
//! Every handler that changes anything returns the new `seq`, so a client
//! never has to ask a second question to find out whether its cache is stale.

pub mod accounts;
pub mod attachments;
pub mod audit_log;
pub mod backups;
pub mod bake_worker;
pub mod bom;
mod bom_config;
mod frontend;
pub mod cad;
pub mod catalog;
pub mod family;
pub mod parts;
pub mod preferences;
pub mod review;
pub mod scripts;
pub mod credentials;
pub mod eco;
pub mod settings;
pub mod setup;
pub mod workflow;
pub mod thumbnails;
pub mod sourcing;
pub mod store;
pub mod workspace;
pub mod native_import;

use std::sync::Arc;

use std::net::SocketAddr;

use axum::extract::{ConnectInfo, Request, State};
use axum::http::{header, HeaderMap, Method, StatusCode};
use axum::middleware::Next;
use axum::response::{IntoResponse, Response};
use axum::routing::{delete, get, patch, post, put};
use axum::Router;

use crate::db::{self, Db};
use crate::audit;
use crate::model::{Actor, TokenScope, User};
use crate::security::{self, Credential};
use crate::Error;

/// The cookie the browser session rides in.
pub const SESSION_COOKIE: &str = "plm_session";

/// The header a browser request that changes anything carries its session's
/// CSRF token in ([`crate::security`]).
pub const CSRF_HEADER: &str = "x-csrf-token";

/// Shared handler state.
pub type Shared = Arc<Db>;

/// Build the whole router.
pub fn router(db: Shared) -> Router {
    Router::new()
        // -- the page ------------------------------------------------------
        .route("/", get(frontend::index))
        .route("/*path", get(frontend::file))
        // -- the hosted CAD app and the server describing itself -----------
        .route("/cad", get(cad::entry))
        .route("/cad/", get(cad::entry))
        .route("/cad/app", get(cad::entry))
        .route("/cad/app/", get(cad::entry))
        .route("/cad/app/*path", get(cad::file))
        .route("/cad/config", get(cad::config))
        .route("/api/setup/import", post(setup::import))
        .route("/api/setup/kicad-libraries", get(setup::kicad_libraries))
        .route("/api/setup/kicad-part-type", post(setup::kicad_part_type))
        .route("/api/setup/kicad-taxonomy", post(setup::kicad_taxonomy))
        .route("/api/workflows", get(workflow::definitions).post(workflow::save))
        .route("/api/workflow-runs", get(workflow::runs).post(workflow::start))
        .route("/api/workflow-runs/:id", get(workflow::detail))
        .route("/api/workflow-runs/:id/actions", post(workflow::act))
        .route("/api/workflow-runs/:id/control", post(workflow::control))
        // -- accounts ------------------------------------------------------
        .route("/api/login", post(accounts::login))
        .route("/api/logout", post(accounts::logout))
        .route("/api/me", get(accounts::me))
        .route("/api/me/preferences", get(preferences::list))
        .route(
            "/api/me/preferences/:key",
            get(preferences::read).put(preferences::write).delete(preferences::remove),
        )
        .route("/api/users", get(accounts::list_users).post(accounts::create_user))
        .route("/api/users/:id", patch(accounts::update_user))
        .route("/api/users/:id/sign-out", post(credentials::sign_out_user))
        // -- credentials ---------------------------------------------------
        .route("/api/me/password", post(credentials::change_password))
        .route("/api/logout-all", post(credentials::logout_all))
        .route("/api/tokens", get(credentials::list_tokens).post(credentials::create_token))
        .route("/api/tokens/:id", delete(credentials::revoke_token))
        .route(
            "/api/security/lockouts",
            get(credentials::list_lockouts).delete(credentials::clear_lockouts),
        )
        .route(
            "/api/part-types",
            get(accounts::list_part_types).post(accounts::create_part_type),
        )
        .route("/api/part-types/:id", patch(accounts::update_part_type))
        .route("/api/part-types/:id/fields", get(bom_config::type_fields).put(bom_config::set_type_fields))
        .route("/api/bom/configuration", get(bom_config::configuration))
        .route("/api/bom/occurrence-fields", axum::routing::put(bom_config::set_occurrence_fields))
        .route("/api/bom/layouts", post(bom_config::save_layout))
        .route("/api/bom/layouts/:id", delete(bom_config::delete_layout))
        .route("/api/bom/selection", axum::routing::put(bom_config::select_layout))
        .route("/api/parts/:id/revisions/:rev/attributes", get(bom_config::revision_attributes).patch(bom_config::patch_revision_attributes))
        .route("/api/parts/:id/revisions/:rev/occurrences", patch(bom_config::patch_occurrences))
        // -- parts and revisions -------------------------------------------
        .route("/api/parts", get(parts::list_parts).post(parts::create_part))
        .route("/api/parts/:id", get(parts::get_part).patch(parts::update_part))
        .route("/api/parts/:id/revisions", post(parts::create_revision))
        .route("/api/parts/:id/revisions/:rev/checkout", post(parts::checkout))
        .route("/api/parts/:id/revisions/:rev/checkin", post(parts::checkin))
        .route("/api/parts/:id/revisions/:rev/state", post(parts::set_state))
        .route("/api/parts/:id/revisions/:rev", delete(parts::delete_revision))
        // -- review and approval -------------------------------------------
        .route("/api/parts/:id/revisions/:rev/submit", post(review::submit))
        .route(
            "/api/parts/:id/revisions/:rev/review",
            get(review::get_review).patch(review::change),
        )
        .route("/api/parts/:id/revisions/:rev/review/decision", post(review::decide))
        .route("/api/parts/:id/revisions/:rev/comments", post(review::comment))
        .route("/api/inbox", get(review::inbox))
        // -- change orders -------------------------------------------------
        .route("/api/ecos", get(eco::list).post(eco::create))
        .route("/api/ecos/numbering", get(eco::numbering).patch(eco::set_numbering))
        .route("/api/ecos/:id", get(eco::get).patch(eco::update))
        .route("/api/ecos/:id/items", post(eco::add_item))
        .route("/api/ecos/:id/items/:rev", delete(eco::remove_item))
        .route("/api/ecos/:id/submit", post(eco::submit))
        .route("/api/ecos/:id/withdraw", post(eco::withdraw))
        .route("/api/ecos/:id/cancel", post(eco::cancel))
        .route("/api/ecos/:id/release", post(eco::release))
        .route("/api/ecos/:id/review", patch(eco::change_review))
        .route("/api/ecos/:id/review/decision", post(eco::decide))
        .route("/api/ecos/:id/comments", post(eco::comment))
        // -- assembly structure: uses lists, BOMs and where-used -----------
        .route(
            "/api/parts/:id/revisions/:rev/uses",
            get(bom::get_uses).put(bom::put_uses),
        )
        .route("/api/parts/:id/revisions/:rev/bom", get(bom::get_bom))
        .route("/api/parts/:id/revisions/:rev/import", put(native_import::write))
        .route("/api/import/validate", post(native_import::validate))
        .route("/api/parts/:id/bom/diff", get(bom::diff))
        .route("/api/parts/:id/where-used", get(bom::where_used))
        .route("/api/parts/:id/replace", post(bom::replace))
        // -- attachments ------------------------------------------------------
        .route(
            "/api/parts/:id/attachments",
            get(attachments::list).post(attachments::upload_to_part),
        )
        .route("/api/parts/:id/revisions/:rev/attachments", post(attachments::upload_to_revision))
        // -- thumbnails -------------------------------------------------------
        .route(
            "/api/parts/:id/revisions/:rev/thumbnail",
            get(thumbnails::get_revision).put(thumbnails::put),
        )
        .route("/api/parts/:id/thumbnail", get(thumbnails::get_part))
        .route(
            "/api/attachments/:id",
            get(attachments::download).put(attachments::replace).delete(attachments::remove),
        )
        .route("/api/attachments/:id/info", get(attachments::info))
        // -- backups and exports (administrators) ---------------------------
        .route("/api/admin/backups", get(backups::status).post(backups::run))
        .route("/api/admin/backups/file/:name", get(backups::saved_file))
        .route("/api/admin/backup", get(backups::download))
        .route("/api/admin/export", get(backups::export))
        // -- families, templates and the bake queue ------------------------
        .route("/api/parts/:id/family", get(family::family))
        .route("/api/parts/:id/family/import", post(family::import))
        .route("/api/parts/:id/generate", post(family::generate))
        .route("/api/parts/:id/template", get(family::template))
        .route("/api/parts/:id/spin-out", post(family::spin_out))
        .route("/api/bake/jobs", get(family::jobs))
        .route("/api/bake/jobs/progress", post(family::progress))
        .route("/api/bake/worker", get(bake_worker::status))
        .route("/api/bake/worker/start", post(bake_worker::start))
        .route("/api/bake/worker/stop", post(bake_worker::stop))
        .route("/api/bake/resave", post(family::resave))
        .route("/api/bake/next", post(family::next))
        .route("/api/bake/jobs/:id/claim", post(family::claim))
        .route("/api/bake/jobs/:id/renew", post(family::renew))
        .route("/api/bake/jobs/:id/result", put(family::result))
        .route("/api/bake/jobs/:id/fail", post(family::fail))
        .route("/api/bake/jobs/:id/retry", post(family::retry))
        .route("/api/parts/:id/sourcing", post(sourcing::add_manufacturer_part))
        .route(
            "/api/parts/:id/sourcing/:mp",
            patch(sourcing::update_manufacturer_part).delete(sourcing::delete_manufacturer_part),
        )
        .route("/api/parts/:id/sourcing/:mp/offers", post(sourcing::add_offer))
        .route(
            "/api/parts/:id/sourcing/:mp/offers/:offer",
            patch(sourcing::update_offer).delete(sourcing::delete_offer),
        )
        // -- sourcing: who makes parts and who sells them -----------------
        .route(
            "/api/manufacturers",
            get(sourcing::list_manufacturers).post(sourcing::create_manufacturer),
        )
        .route(
            "/api/manufacturers/:id",
            patch(sourcing::update_manufacturer).delete(sourcing::delete_manufacturer),
        )
        .route("/api/suppliers", get(sourcing::list_suppliers).post(sourcing::create_supplier))
        .route(
            "/api/suppliers/:id",
            patch(sourcing::update_supplier).delete(sourcing::delete_supplier),
        )
        // -- the catalog ---------------------------------------------------
        .route("/api/categories", get(catalog::list).post(catalog::create))
        .route(
            "/api/categories/:id",
            patch(catalog::update).delete(catalog::remove),
        )
        .route("/api/categories/:id/schema", get(catalog::schema))
        // -- the administrator's settings ----------------------------------
        .route("/api/settings", get(settings::get).patch(settings::update))
        // -- the administrator's scripts -----------------------------------
        .route("/api/scripts", get(scripts::list))
        .route("/api/scripts/run", post(scripts::run))
        .route(
            "/api/scripts/file/*path",
            get(scripts::read).put(scripts::write).delete(scripts::remove),
        )
        // -- the document surface the CAD app will consume -----------------
        .route("/api/store/index", get(store::index))
        .route("/api/store/changes", get(store::changes))
        .route("/api/store/doc/*key", get(store::read_doc))
        .route("/api/store/doc/*key", put(store::write_doc))
        .route("/api/store/doc/*key", delete(store::delete_doc))
        // -- the audit log -------------------------------------------------
        .route("/api/audit", get(audit_log::query))
        .route("/api/parts/:id/history", get(audit_log::part_history))
        // -- workspaces: each user's folders of links and files -------------
        .route("/api/workspaces", get(workspace::list_workspaces))
        .route("/api/workspace/entries", get(workspace::list_folder))
        .route("/api/workspace/folders", post(workspace::create_folder))
        .route("/api/workspace/links", post(workspace::create_link))
        .route("/api/workspace/files", post(workspace::create_file))
        .route(
            "/api/workspace/entries/:id",
            get(workspace::get_entry).patch(workspace::update_entry).delete(workspace::delete_entry),
        )
        .route(
            "/api/workspace/entries/:id/content",
            get(workspace::download).put(workspace::replace_file),
        )
        .route("/api/workspace/entries/:id/versions", get(workspace::versions))
        .route("/api/workspace/entries/:id/versions/:version/restore", post(workspace::restore))
        .route("/api/workspace/entries/:id/promote", post(workspace::promote))
        .layer(axum::middleware::from_fn_with_state(Arc::clone(&db), guard))
        .with_state(db)
}

// ===========================================================================
// Sessions
// ===========================================================================

/// The session token the request carries, if any.
pub fn session_token(headers: &HeaderMap) -> Option<String> {
    let raw = headers.get(header::COOKIE)?.to_str().ok()?;
    raw.split(';').find_map(|pair| {
        let (name, value) = pair.split_once('=')?;
        (name.trim() == SESSION_COOKIE).then(|| value.trim().to_string())
    })
}

/// Who a request is, once its credential has been checked.
#[derive(Debug, Clone)]
pub struct Identity {
    pub user: User,
    /// The session token, for a browser.
    pub session: Option<String>,
    /// The API token's id and scope, for anything else.
    pub token: Option<(String, TokenScope)>,
}

/// The signed-in user, or a 401. A browser session or an API token
/// ([`crate::security`]) — the handler does not care which.
pub fn require_user(db: &Db, headers: &HeaderMap) -> Result<User, Error> {
    identify(db, headers).map(|identity| identity.user)
}

/// Check the request's credential and say who it is.
///
/// A session expires after [`crate::model::Settings::session_idle_minutes`]
/// without a request, and [`crate::model::Settings::session_max_hours`] after
/// sign-in however busy it is. Both are checked on every request, so nothing
/// has to sweep. Touching `last_seen` costs a metadata write, so it is
/// re-stamped at most once a minute.
pub fn identify(db: &Db, headers: &HeaderMap) -> Result<Identity, Error> {
    match security::credential(headers) {
        None => Err(Error::unauthorized("not signed in")),
        Some(Credential::Bearer(secret)) => identify_token(db, &secret),
        Some(Credential::Session(token)) => identify_session(db, &token),
    }
}

fn identify_session(db: &Db, token: &str) -> Result<Identity, Error> {
    let now = db::now();
    let outcome = db.read(|state| {
        let session = state.sessions.get(token)?;
        let user = state.user(&session.user_id)?;
        Some((session.last_seen, db::session_expired(&state.settings, session, now), user.clone()))
    });
    let Some((last_seen, expired, user)) = outcome else {
        return Err(Error::unauthorized("session has expired"));
    };
    if expired {
        let token = token.to_string();
        let _ = db.mutate(move |state| {
            state.sessions.retain(|s| s.token != token);
            Ok(())
        });
        return Err(Error::unauthorized("session has expired"));
    }
    if !user.active {
        return Err(Error::forbidden("this account is disabled"));
    }
    if now.saturating_sub(last_seen) > 60 {
        let token = token.to_string();
        let _ = db.mutate(move |state| {
            if let Some(session) = state.sessions.get_mut(&token) {
                session.last_seen = now;
            }
            Ok(())
        });
    }
    Ok(Identity { user, session: Some(token.to_string()), token: None })
}

fn identify_token(db: &Db, secret: &str) -> Result<Identity, Error> {
    let hash = security::token_hash(secret);
    let found = db.read(|state| {
        let token = state.api_tokens.iter().find(|t| security::same_secret(&t.hash, &hash))?;
        let user = state.user(&token.user_id)?;
        Some((token.clone(), user.clone()))
    });
    let Some((token, user)) = found else {
        return Err(Error::unauthorized("that API token is not valid"));
    };
    let now = db::now();
    if token.expires_at.is_some_and(|at| at <= now) {
        return Err(Error::unauthorized("that API token has expired"));
    }
    if !user.active {
        return Err(Error::forbidden("this account is disabled"));
    }
    if token.last_used_at.is_none_or(|at| now.saturating_sub(at) > 60) {
        let id = token.id.clone();
        let _ = db.mutate(move |state| {
            if let Some(t) = state.api_tokens.iter_mut().find(|t| t.id == id) {
                t.last_used_at = Some(now);
            }
            Ok(())
        });
    }
    Ok(Identity { user, session: None, token: Some((token.id, token.scope)) })
}

/// The `Set-Cookie` value that starts a browser session.
pub fn session_cookie(db: &Db, headers: &HeaderMap, token: &str) -> String {
    let settings = db.settings();
    let max_age = if settings.session_max_hours > 0 {
        settings.session_max_hours * 3600
    } else {
        settings.session_idle_minutes * 60
    };
    let secure = if db.security().config.cookie_is_secure(headers) { "; Secure" } else { "" };
    // HttpOnly so script cannot read it; SameSite=Lax so another site's form
    // post does not carry it. Secure per `--secure-cookies`.
    format!("{SESSION_COOKIE}={token}; HttpOnly; SameSite=Lax; Path=/; Max-Age={max_age}{secure}")
}

/// The `Set-Cookie` value that ends one.
pub fn clear_cookie(db: &Db, headers: &HeaderMap) -> String {
    let secure = if db.security().config.cookie_is_secure(headers) { "; Secure" } else { "" };
    format!("{SESSION_COOKIE}=; HttpOnly; SameSite=Lax; Path=/; Max-Age=0{secure}")
}

/// Every request passes through here before its handler.
///
/// * A request with an API token is refused if the token is not valid, or if
///   its scope does not allow the request ([`security::scope_refusal`]).
/// * A request that changes anything and rides a session cookie must carry the
///   session's CSRF token in `X-CSRF-Token`. `/api/login` is exempt — there is
///   no session yet, and it only takes a JSON body, which another site's form
///   cannot send.
/// * Every response gets the security headers ([`security::security_headers`]).
///
/// The request then runs as its [`Actor`], so the audit log can say who made
/// every change it causes ([`crate::audit`]).
pub async fn guard(State(db): State<Shared>, request: Request, next: Next) -> Response {
    let path = request.uri().path().to_string();
    let api = path.starts_with("/api/");
    let secure = db.security().config.is_secure_request(request.headers());
    if api {
        if let Err(error) = check_request(&db, request.method(), &path, request.headers()) {
            let mut response = error.into_response();
            security::security_headers(response.headers_mut(), secure, true);
            return response;
        }
    }
    let peer = request.extensions().get::<ConnectInfo<SocketAddr>>().map(|ConnectInfo(addr)| addr.ip());
    let actor = actor_of(&db, request.headers(), peer);
    let mut response = audit::scope(actor, next.run(request)).await;
    // The hosted CAD app answers with its own policy ([`crate::cad`]); every
    // other response gets the PLM's, whatever its handler set.
    let own_policy = if path.starts_with(crate::cad::PREFIX) {
        response.headers_mut().remove(header::CONTENT_SECURITY_POLICY)
    } else {
        None
    };
    let inline_document = response.status().is_success()
        && (path.starts_with("/api/attachments/") || path.starts_with("/api/workspace/entries/"))
        && response.headers().get(header::CONTENT_DISPOSITION).and_then(|v| v.to_str().ok()).is_some_and(|v| v.starts_with("inline;"))
        && response.headers().get(header::CONTENT_TYPE).and_then(|v| v.to_str().ok()).is_some_and(|mime| mime == "application/pdf");
    security::security_headers(response.headers_mut(), secure, api);
    if inline_document {
        response.headers_mut().insert(header::CONTENT_SECURITY_POLICY, axum::http::HeaderValue::from_static("default-src 'none'; frame-ancestors 'self'; base-uri 'none'"));
        response.headers_mut().insert("x-frame-options", axum::http::HeaderValue::from_static("SAMEORIGIN"));
    }
    if let Some(policy) = own_policy {
        response.headers_mut().insert(header::CONTENT_SECURITY_POLICY, policy);
    }
    response
}

/// Who a request is, for the audit log: read-only, and without the expiry
/// checks — the handler still decides whether the credential is good, and a
/// session that expires during this request is logged as ended by its owner.
pub fn actor_of(db: &Db, headers: &HeaderMap, peer: Option<std::net::IpAddr>) -> Actor {
    let ip = db.security().config.client_ip(headers, peer).map(|ip| ip.to_string()).unwrap_or_default();
    let found = match security::credential(headers) {
        Some(Credential::Session(token)) => db.read(|state| {
            let session = state.sessions.iter().find(|s| s.token == token)?;
            let user = state.user(&session.user_id)?;
            Some((user.id.clone(), user.username.clone(), "session"))
        }),
        Some(Credential::Bearer(secret)) => {
            let hash = security::token_hash(&secret);
            db.read(|state| {
                let token = state.api_tokens.iter().find(|t| security::same_secret(&t.hash, &hash))?;
                let user = state.user(&token.user_id)?;
                Some((user.id.clone(), user.username.clone(), "token"))
            })
        }
        None => None,
    };
    match found {
        Some((user_id, username, via)) => Actor { user_id, username, via: via.into(), ip },
        None => Actor { ip, ..Actor::system() },
    }
}

fn check_request(db: &Db, method: &Method, path: &str, headers: &HeaderMap) -> Result<(), Error> {
    match security::credential(headers) {
        Some(Credential::Bearer(secret)) => {
            let identity = identify_token(db, &secret)?;
            if let Some((_, scope)) = identity.token {
                if let Some(refusal) = security::scope_refusal(scope, method, path) {
                    return Err(Error::forbidden(refusal));
                }
            }
            Ok(())
        }
        Some(Credential::Session(token)) if security::is_mutating(method) && path != "/api/login" => {
            let expected = db.read(|state| {
                state.sessions.iter().find(|s| s.token == token).map(|s| s.csrf.clone())
            });
            // No such session: the handler answers 401 as it always has.
            let Some(expected) = expected else { return Ok(()) };
            let sent = headers.get(CSRF_HEADER).and_then(|v| v.to_str().ok()).unwrap_or("");
            if expected.is_empty() || !security::same_secret(&expected, sent) {
                return Err(Error::forbidden(
                    "this request is missing its CSRF token — reload the page and try again",
                ));
            }
            Ok(())
        }
        _ => Ok(()),
    }
}

/// The signed-in user, who must be an administrator.
pub fn require_admin(db: &Db, headers: &HeaderMap) -> Result<User, Error> {
    let user = require_user(db, headers)?;
    if !user.is_admin() {
        return Err(Error::forbidden("this needs the admin group"));
    }
    Ok(user)
}

/// The signed-in user, who must be able to create and edit work.
pub fn require_author(db: &Db, headers: &HeaderMap) -> Result<User, Error> {
    let user = require_user(db, headers)?;
    if !user.can_author() {
        return Err(Error::forbidden("this needs the author group"));
    }
    Ok(user)
}

/// Run `work` on the blocking pool. Anything that may call a script goes
/// through here: a hook can wait on the network or a child process, and that
/// wait must not stall the async runtime's worker threads.
///
/// The request's actor goes with the work, so a change made on the blocking
/// pool is logged as the caller's.
pub async fn blocking<R: Send + 'static>(
    work: impl FnOnce() -> Result<R, Error> + Send + 'static,
) -> Result<R, Error> {
    let actor = audit::current_actor();
    tokio::task::spawn_blocking(move || audit::as_actor(actor, work))
        .await
        .map_err(Error::internal)?
}

/// `{ "ok": true, "seq": n }` — the shape every mutating handler answers with.
pub fn ok_seq(db: &Db) -> Response {
    let seq = db.read(|state| state.seq);
    axum::Json(serde_json::json!({ "ok": true, "seq": seq })).into_response()
}

/// A tiny health probe, useful before the page exists.
pub async fn health(State(db): State<Shared>) -> Response {
    let (users, parts, seq) = db.read(|s| (s.users.len(), s.parts.len(), s.seq));
    (
        StatusCode::OK,
        axum::Json(serde_json::json!({
            "ok": true, "users": users, "parts": parts, "seq": seq,
        })),
    )
        .into_response()
}
