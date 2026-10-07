//! Hardening: how the server is configured to face a network, the sign-in
//! throttle, API tokens, and the pieces of request checking every route shares
//! (CSRF, token scopes, response headers).
//!
//! # Two ways to sign in
//!
//! * **A browser** signs in with a password and gets a session cookie
//!   (`HttpOnly`, `SameSite=Lax`, `Secure` when [`ServerConfig`] says so). A
//!   cookie is sent by the browser on its own, so every request that changes
//!   anything must ALSO carry the session's CSRF token in `X-CSRF-Token`. That
//!   is the synchronizer-token pattern: the token lives in the session record
//!   on the server, the page receives it from `/api/login` and `/api/me`, and
//!   another site's page can neither read it nor guess it.
//! * **Anything else** — the CAD app, a headless bake worker, an organization's
//!   own tooling — sends `Authorization: Bearer plm_…`, an API token its owner
//!   created. A bearer header is never sent by a browser on its own, so a token
//!   request needs no CSRF token. A token acts as its owner, narrowed by its
//!   [`TokenScope`].
//!
//! # The sign-in throttle
//!
//! Failed sign-ins are counted per username and per client address, in memory.
//! Past a threshold the key is locked out for a period that doubles with each
//! further failure, up to a cap; while locked out the server refuses without
//! even checking the password, so a guesser learns nothing. A success clears
//! the username's count. An administrator can see and clear every lockout. The
//! counts live in memory on purpose: a restart forgets them, and nothing an
//! attacker does can lock an account out permanently.

use std::collections::HashMap;
use std::net::IpAddr;
use std::sync::Mutex;

use axum::http::{HeaderMap, Method};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use subtle::ConstantTimeEq;

use crate::model::{Timestamp, TokenScope};

/// When the session cookie carries the `Secure` flag.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum CookieSecurity {
    /// Never. The default, so plain-HTTP loopback development works.
    #[default]
    Off,
    /// Always. Right when every client reaches the server over HTTPS.
    On,
    /// When the request arrived over TLS: natively, or through a trusted
    /// proxy that says `X-Forwarded-Proto: https`.
    Auto,
}

impl std::str::FromStr for CookieSecurity {
    type Err = String;
    fn from_str(text: &str) -> Result<Self, String> {
        match text {
            "off" => Ok(Self::Off),
            "on" => Ok(Self::On),
            "auto" => Ok(Self::Auto),
            other => Err(format!("'{other}' is not off, on or auto")),
        }
    }
}

/// How the server faces the network. Set from the command line at start-up;
/// nothing in the browser can change it.
#[derive(Debug, Clone, Default)]
pub struct ServerConfig {
    /// Native worker pinned by the server operator, controlled by admins.
    pub bake_worker: Option<crate::bake_worker::Config>,
    pub secure_cookies: CookieSecurity,
    /// Believe `X-Forwarded-For` and `X-Forwarded-Proto`. Only right behind a
    /// reverse proxy that sets them; otherwise any client could claim any
    /// address and dodge the per-address throttle.
    pub trust_proxy: bool,
    /// The server terminates TLS itself (`--tls-cert` / `--tls-key`).
    pub tls: bool,
    /// `--lock-script-editor`: the in-browser script editor is off whatever
    /// the settings say. The one protection an admin login cannot undo.
    pub lock_script_editor: bool,
    /// `--max-attachment-mb`: the largest attachment an upload may carry, in
    /// bytes. 0 means [`DEFAULT_MAX_ATTACHMENT`].
    pub max_attachment_bytes: u64,
    /// `--max-document-mb`: the largest body a document write — a revision's
    /// document, or the `@recovery` preference that mirrors unsaved ones — may
    /// carry, in bytes. 0 means [`DEFAULT_MAX_DOCUMENT`].
    pub max_document_bytes: u64,
    /// `--min-client-version`: a floor on the CAD app's version ABOVE
    /// [`crate::version::MIN_CLIENT_VERSION`], for an administrator retiring
    /// old installs — and for a test runner provoking the app's refusal. None
    /// means the built-in floor.
    pub min_client_version: Option<String>,
    /// `--backup-dir`: where scheduled and "Back up now" backups are saved.
    /// None: backups are only downloaded.
    pub backup_dir: Option<std::path::PathBuf>,
    /// `--backup-every`: minutes between scheduled backups; 0 is none.
    pub backup_every_minutes: u64,
    /// `--backup-keep`: how many saved backups to keep; 0 means 7.
    pub backup_keep: usize,
    /// `--cad-app`: the directory the wasm CAD app is served from
    /// ([`crate::cad`]). None: no app is hosted.
    pub cad_app_dir: Option<std::path::PathBuf>,
    /// `--web-dir`: live PLM frontend files overriding the embedded bundle.
    pub web_dir: Option<std::path::PathBuf>,
    /// `--cad-connect-src`: origins the hosted CAD app may call besides this
    /// server ([`crate::cad::policy`]).
    pub cad_connect_src: Vec<String>,
}

/// 100 MB: a scanned drawing set or a long datasheet, not a video.
pub const DEFAULT_MAX_ATTACHMENT: u64 = 100 * 1024 * 1024;

/// 64 MB: a large assembly's history JSON, with room. Far above axum's 2 MB
/// default for a buffered body, which real model documents exceed.
pub const DEFAULT_MAX_DOCUMENT: u64 = 64 * 1024 * 1024;

impl ServerConfig {
    /// The upload limit in force.
    pub fn attachment_limit(&self) -> u64 {
        if self.max_attachment_bytes == 0 { DEFAULT_MAX_ATTACHMENT } else { self.max_attachment_bytes }
    }

    /// The oldest CAD app this server serves: `--min-client-version` when
    /// given, else the built-in [`crate::version::MIN_CLIENT_VERSION`].
    pub fn min_client_version(&self) -> &str {
        self.min_client_version.as_deref().unwrap_or(crate::version::MIN_CLIENT_VERSION)
    }

    /// The document body limit in force.
    pub fn document_limit(&self) -> u64 {
        if self.max_document_bytes == 0 { DEFAULT_MAX_DOCUMENT } else { self.max_document_bytes }
    }
}

impl ServerConfig {
    /// Whether this request reached the server over TLS, as far as the server
    /// can know.
    pub fn is_secure_request(&self, headers: &HeaderMap) -> bool {
        self.tls
            || (self.trust_proxy
                && headers
                    .get("x-forwarded-proto")
                    .and_then(|v| v.to_str().ok())
                    .is_some_and(|v| v.split(',').next().unwrap_or("").trim().eq_ignore_ascii_case("https")))
    }

    /// Whether a cookie set in answer to this request is `Secure`.
    pub fn cookie_is_secure(&self, headers: &HeaderMap) -> bool {
        match self.secure_cookies {
            CookieSecurity::Off => false,
            CookieSecurity::On => true,
            CookieSecurity::Auto => self.is_secure_request(headers),
        }
    }

    /// The client address: the socket peer, or with [`Self::trust_proxy`] the
    /// first address in `X-Forwarded-For`.
    pub fn client_ip(&self, headers: &HeaderMap, peer: Option<IpAddr>) -> Option<IpAddr> {
        if self.trust_proxy {
            let forwarded = headers
                .get("x-forwarded-for")
                .and_then(|v| v.to_str().ok())
                .and_then(|v| v.split(',').next())
                .and_then(|v| v.trim().parse().ok());
            if forwarded.is_some() {
                return forwarded;
            }
        }
        peer
    }
}

// ===========================================================================
// The sign-in throttle
// ===========================================================================

/// Failures on one username before it is locked out.
pub const USER_THRESHOLD: u32 = 5;
/// Failures from one address before it is locked out. Higher, because one
/// address may be a whole office behind NAT.
pub const IP_THRESHOLD: u32 = 20;
/// The first lockout; each further failure doubles it.
pub const FIRST_LOCKOUT: u64 = 30;
/// The longest lockout.
pub const MAX_LOCKOUT: u64 = 15 * 60;
/// A count with no failure for this long starts again from zero.
pub const FORGET_AFTER: u64 = 60 * 60;
/// The most keys remembered; past it the stalest are forgotten, so a flood of
/// made-up usernames cannot grow memory without bound.
const MAX_KEYS: usize = 10_000;

/// Which kind of key a throttle entry counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ThrottleKind {
    User,
    Ip,
}

impl ThrottleKind {
    fn threshold(self) -> u32 {
        match self {
            ThrottleKind::User => USER_THRESHOLD,
            ThrottleKind::Ip => IP_THRESHOLD,
        }
    }
}

#[derive(Debug, Clone, Default)]
struct Count {
    failures: u32,
    last_failure: Timestamp,
    locked_until: Timestamp,
}

/// One entry as an administrator sees it.
#[derive(Debug, Clone, Serialize)]
pub struct Lockout {
    pub kind: ThrottleKind,
    pub key: String,
    pub failures: u32,
    pub last_failure: Timestamp,
    /// When the lockout ends; 0 or in the past when the key is only counting.
    pub locked_until: Timestamp,
    pub locked: bool,
}

/// Failed sign-ins per username and per address.
#[derive(Debug, Default)]
pub struct Throttle {
    counts: Mutex<HashMap<(ThrottleKind, String), Count>>,
}

impl Throttle {
    fn key(kind: ThrottleKind, key: &str) -> (ThrottleKind, String) {
        (kind, key.trim().to_ascii_lowercase())
    }

    /// How many seconds `key` is still locked out for, if it is.
    pub fn locked_for(&self, kind: ThrottleKind, key: &str, now: Timestamp) -> Option<u64> {
        let counts = self.counts.lock().unwrap_or_else(|e| e.into_inner());
        let count = counts.get(&Self::key(kind, key))?;
        (count.locked_until > now).then(|| count.locked_until - now)
    }

    /// Count a failure against `key`.
    pub fn fail(&self, kind: ThrottleKind, key: &str, now: Timestamp) {
        let mut counts = self.counts.lock().unwrap_or_else(|e| e.into_inner());
        if counts.len() >= MAX_KEYS {
            // Forget the stalest tenth rather than one at a time.
            let mut ages: Vec<_> = counts.iter().map(|(k, c)| (c.last_failure, k.clone())).collect();
            ages.sort();
            for (_, stale) in ages.into_iter().take(MAX_KEYS / 10) {
                counts.remove(&stale);
            }
        }
        let count = counts.entry(Self::key(kind, key)).or_default();
        if now.saturating_sub(count.last_failure) > FORGET_AFTER {
            *count = Count::default();
        }
        count.failures += 1;
        count.last_failure = now;
        let threshold = kind.threshold();
        if count.failures >= threshold {
            let doublings = (count.failures - threshold).min(16);
            count.locked_until = now + (FIRST_LOCKOUT << doublings).min(MAX_LOCKOUT);
        }
    }

    /// Forget `key` — a successful sign-in, or an administrator's clear.
    pub fn clear(&self, kind: ThrottleKind, key: &str) -> bool {
        let mut counts = self.counts.lock().unwrap_or_else(|e| e.into_inner());
        counts.remove(&Self::key(kind, key)).is_some()
    }

    /// Forget everything.
    pub fn clear_all(&self) -> usize {
        let mut counts = self.counts.lock().unwrap_or_else(|e| e.into_inner());
        let n = counts.len();
        counts.clear();
        n
    }

    /// Every key with a failure on record, locked-out ones first.
    pub fn list(&self, now: Timestamp) -> Vec<Lockout> {
        let counts = self.counts.lock().unwrap_or_else(|e| e.into_inner());
        let mut out: Vec<Lockout> = counts
            .iter()
            .filter(|(_, c)| now.saturating_sub(c.last_failure) <= FORGET_AFTER)
            .map(|((kind, key), c)| Lockout {
                kind: *kind,
                key: key.clone(),
                failures: c.failures,
                last_failure: c.last_failure,
                locked_until: c.locked_until,
                locked: c.locked_until > now,
            })
            .collect();
        out.sort_by(|a, b| b.locked.cmp(&a.locked).then(b.last_failure.cmp(&a.last_failure)));
        out
    }
}

/// Everything the hardening layer keeps for the life of the process.
#[derive(Debug, Default)]
pub struct Security {
    pub config: ServerConfig,
    pub throttle: Throttle,
}

// ===========================================================================
// API tokens
// ===========================================================================

/// Every token secret starts with this, so a leaked one is recognizable in a
/// log or a repository scan.
pub const TOKEN_PREFIX: &str = "plm_";

/// A fresh token secret: the prefix and 256 random bits.
pub fn new_token_secret() -> String {
    format!("{TOKEN_PREFIX}{}", crate::auth::random_token(32))
}

/// The stored form of a secret: hex SHA-256.
pub fn token_hash(secret: &str) -> String {
    let digest = Sha256::digest(secret.as_bytes());
    digest.iter().map(|b| format!("{b:02x}")).collect()
}

/// Whether two secrets are equal, in constant time.
pub fn same_secret(a: &str, b: &str) -> bool {
    a.len() == b.len() && bool::from(a.as_bytes().ct_eq(b.as_bytes()))
}

/// How a request says who it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Credential {
    Session(String),
    Bearer(String),
}

/// The credential a request carries. A bearer header wins over a cookie: a
/// tool that sends one means it.
pub fn credential(headers: &HeaderMap) -> Option<Credential> {
    if let Some(value) = headers.get(axum::http::header::AUTHORIZATION).and_then(|v| v.to_str().ok()) {
        let value = value.trim();
        if let Some(rest) = value.strip_prefix("Bearer ").or_else(|| value.strip_prefix("bearer ")) {
            return Some(Credential::Bearer(rest.trim().to_string()));
        }
    }
    crate::api::session_token(headers).map(Credential::Session)
}

/// Whether a method changes anything. Everything but a read does.
pub fn is_mutating(method: &Method) -> bool {
    !matches!(*method, Method::GET | Method::HEAD | Method::OPTIONS)
}

/// Routes a token may never call, whatever its scope: the ones that mint or
/// revoke credentials. A leaked token must not be able to make itself a new
/// one, change its owner's password, or hide by revoking the others.
pub fn is_credential_route(path: &str) -> bool {
    path == "/api/tokens"
        || path.starts_with("/api/tokens/")
        || path == "/api/me/password"
        || path == "/api/logout-all"
}

/// `/api/parts/<id>/revisions/<rev>/thumbnail`: the bake worker pictures
/// what it bakes ([`crate::thumbnail`]).
fn is_thumbnail_route(path: &str) -> bool {
    let parts: Vec<&str> = path.split('/').collect();
    matches!(parts.as_slice(), ["", "api", "parts", p, "revisions", r, "thumbnail"] if !p.is_empty() && !r.is_empty())
}

/// Why a token of `scope` may not make this request, or `None` when it may.
pub fn scope_refusal(scope: TokenScope, method: &Method, path: &str) -> Option<String> {
    if is_credential_route(path) {
        return Some("an API token cannot manage tokens or passwords — sign in with a password".into());
    }
    if !is_mutating(method) {
        return None;
    }
    match scope {
        TokenScope::Full => None,
        TokenScope::Read => Some("this API token may only read".into()),
        TokenScope::Worker => {
            let allowed = (path.starts_with("/api/bake/") && !path.starts_with("/api/bake/worker"))
                || path.starts_with("/api/store/doc/")
                || (*method == Method::PUT && is_thumbnail_route(path));
            (!allowed).then(|| "this worker token may only read, work the bake queue, write documents and their thumbnails".into())
        }
    }
}

// ===========================================================================
// Response headers
// ===========================================================================

/// The page's Content Security Policy. The page is three same-origin files
/// with no inline script or style and no third-party anything, so everything
/// is `'self'` and nothing may frame it.
pub const CONTENT_SECURITY_POLICY: &str = "default-src 'none'; script-src 'self'; style-src 'self'; \
img-src 'self' data:; connect-src 'self'; font-src 'self'; frame-src 'self'; form-action 'self'; \
frame-ancestors 'none'; base-uri 'none'";

/// Headers every response carries.
pub fn security_headers(headers: &mut HeaderMap, secure: bool, api: bool) {
    use axum::http::HeaderValue;
    let chose_private = headers.get("cache-control").and_then(|v| v.to_str().ok()).is_some_and(|v| v.starts_with("private"));
    let mut set = |name: &'static str, value: &'static str| {
        headers.insert(name, HeaderValue::from_static(value));
    };
    set("content-security-policy", CONTENT_SECURITY_POLICY);
    set("x-frame-options", "DENY");
    set("x-content-type-options", "nosniff");
    set("referrer-policy", "no-referrer");
    set("cross-origin-opener-policy", "same-origin");
    set("permissions-policy", "camera=(), microphone=(), geolocation=()");
    // Part data, documents and tokens must not sit in a shared cache. A
    // handler that chose a `private` policy itself keeps it: a thumbnail
    // (`api::thumbnails`) is revalidated by its ETag, or cached for good at a
    // URL versioned by its hash.
    if api && !chose_private {
        set("cache-control", "no-store");
    }
    if secure {
        set("strict-transport-security", "max-age=31536000");
    }
}

