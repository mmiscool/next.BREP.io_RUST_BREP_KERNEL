//! Sign-in, the refusals, the change feed and the version check (plan S1).
//!
//! [`PlmClient`] speaks the PLM's API over any [`PlmTransport`]. It owns the
//! credential, so a transport only carries bytes:
//!
//! - **A token** (`Authorization: Bearer plm_…`) for the native app, the bake
//!   worker and tooling. A user pastes one, or [`PlmClient::sign_in_for_token`]
//!   trades a username and password for one: it signs in, mints a token through
//!   that session, and signs the session out again, so the machine keeps only
//!   the token (D5).
//! - **The session cookie** when the page is served by the PLM itself (D6). The
//!   browser holds the cookie; the client sends `X-CSRF-Token` with every change,
//!   read from `/api/me`'s `csrf`.
//!
//! Every refusal carries the sentence the server sent ([`PlmError`]). `204` is
//! not one: [`PlmClient::get_document`] answers `None` for a revision with no
//! document yet.
//!
//! **The version check** runs on every sign-in, before the client counts as
//! connected. On a mismatch the sign-in fails with [`PlmError::Version`], which
//! names both versions, and the credential is dropped. The file stores are
//! untouched either way; nothing here knows they exist.
use super::{PlmRequest, PlmResponse, PlmTransport};
use serde::Deserialize;
use serde_json::{json, Value};
use std::cell::RefCell;
use std::rc::Rc;

/// Sign in with a configured token and compose the session's store backend
/// (`plm::backend`): the native boot's one call. (The web boot signs in with
/// the browser's session instead: `plm::backend::open_web_session`.)
#[cfg(not(target_arch = "wasm32"))]
pub(crate) use super::backend::open_session;

/// This app's version, as the PLM's `min_client_version` is compared against.
pub const CLIENT_VERSION: &str = env!("CARGO_PKG_VERSION");

/// Why a call to the PLM did not do what was asked. Each variant keeps the
/// server's own sentence; `Display` shows it as the user should read it.
#[derive(Debug, Clone, PartialEq)]
pub enum PlmError {
    /// No answer at all: the server is down, the address is wrong, the network
    /// dropped.
    Unreachable(String),
    /// `401`: the credential is missing, expired or revoked. Sign in again.
    SignIn(String),
    /// `403`: signed in, but the account or the token's scope may not do this.
    Forbidden(String),
    /// `404`: the part, revision or route does not exist.
    NotFound(String),
    /// `409`: locked by someone else, not checked out, or not editable. For a
    /// document write it arrives after `write()` returned, through the store's
    /// write-behind error lane.
    Conflict(String),
    /// `413`: over the server's attachment limit.
    TooLarge(String),
    /// `429`: the sign-in throttle.
    Throttled(String),
    /// Any other refusal (`400`, `5xx`), with its status.
    Refused { status: u16, message: String },
    /// The answer was not what the contract says (unparseable JSON, a missing
    /// field).
    Malformed(String),
    /// Signed in, but this app and this server do not serve each other. Both
    /// versions are named. The file stores stay usable; the boot that falls
    /// back to them says so after this sentence.
    Version { server: String, min_client: String, client: String, reason: String },
}

impl PlmError {
    /// The HTTP status behind a refusal, if there was one.
    pub fn status(&self) -> Option<u16> {
        match self {
            PlmError::SignIn(_) => Some(401),
            PlmError::Forbidden(_) => Some(403),
            PlmError::NotFound(_) => Some(404),
            PlmError::Conflict(_) => Some(409),
            PlmError::TooLarge(_) => Some(413),
            PlmError::Throttled(_) => Some(429),
            PlmError::Refused { status, .. } => Some(*status),
            PlmError::Unreachable(_) | PlmError::Malformed(_) | PlmError::Version { .. } => None,
        }
    }

    /// Map a non-success answer to its refusal, keeping the server's sentence
    /// (`{"error": "..."}`, else the body text, else a sentence of our own).
    pub fn from_response(response: &PlmResponse) -> Self {
        let sentence = server_sentence(response);
        let or = |fallback: &str| if sentence.is_empty() { fallback.to_string() } else { sentence.clone() };
        match response.status {
            401 => PlmError::SignIn(or("sign in again")),
            403 => PlmError::Forbidden(or("you may not do that")),
            404 => PlmError::NotFound(or("not found")),
            409 => PlmError::Conflict(or("refused: the revision is locked or not editable")),
            413 => PlmError::TooLarge(or("too large for the server's limit")),
            429 => PlmError::Throttled(or("too many sign-in attempts; wait and try again")),
            status => PlmError::Refused { status, message: or(&format!("the server answered {status}")) },
        }
    }
}

impl std::fmt::Display for PlmError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlmError::Unreachable(why) => write!(f, "the PLM server did not answer: {why}"),
            PlmError::SignIn(s) => write!(f, "sign in again: {s}"),
            PlmError::Forbidden(s)
            | PlmError::NotFound(s)
            | PlmError::Conflict(s)
            | PlmError::TooLarge(s)
            | PlmError::Throttled(s) => f.write_str(s),
            PlmError::Refused { message, .. } => f.write_str(message),
            PlmError::Malformed(why) => write!(f, "the PLM server's answer was not understood: {why}"),
            PlmError::Version { server, min_client, client, reason } => write!(
                f,
                "this CAD app ({client}) cannot use this PLM server ({server}, which serves CAD {min_client} and later): {reason}."
            ),
        }
    }
}

/// `Accept` and the credential's headers: the bearer token, or the session
/// cookie with the CSRF token on anything but a `GET`.
fn credential_headers(credential: &Credential, method: &str) -> Vec<(String, String)> {
    let mut headers = vec![("Accept".to_string(), "application/json".to_string())];
    match credential {
        Credential::Anonymous => {}
        Credential::Token(token) => headers.push(("Authorization".into(), format!("Bearer {token}"))),
        Credential::Session { cookie, csrf } => {
            if let Some(cookie) = cookie {
                headers.push(("Cookie".into(), cookie.clone()));
            }
            if method != "GET" {
                if let Some(csrf) = csrf {
                    headers.push(("X-CSRF-Token".into(), csrf.clone()));
                }
            }
        }
    }
    headers
}

fn server_sentence(response: &PlmResponse) -> String {
    if let Ok(v) = serde_json::from_slice::<Value>(&response.body) {
        if let Some(s) = v.get("error").and_then(Value::as_str) {
            return s.to_string();
        }
    }
    String::from_utf8_lossy(&response.body).trim().chars().take(500).collect()
}

/// The PLM's `/api/me`, read tolerantly: fields this client does not know are
/// ignored and a missing one takes its default.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Me {
    pub id: String,
    pub username: String,
    pub display_name: String,
    pub groups: Vec<String>,
    /// `"token"` or `"session"`.
    pub via: Option<String>,
    /// A session's CSRF token (session sign-ins only).
    pub csrf: Option<String>,
    /// P1: the server's version (`BREP_plm`'s own).
    pub server_version: Option<String>,
    /// P1: the oldest CAD app (`BREP_app` version) the server serves.
    pub min_client_version: Option<String>,
    /// What the server has beyond the base contract (`workspaces`,
    /// `attribute-filters`, `external-ref`, …). Absent on a server that
    /// predates the list: then it has none of them. Names are only added.
    #[serde(default)]
    pub features: Vec<String>,
}

impl Me {
    /// Whether the server says it has `feature`.
    pub fn has(&self, feature: &str) -> bool {
        self.features.iter().any(|f| f == feature)
    }
}

/// How the client proves who it is.
#[derive(Debug, Clone, PartialEq)]
pub enum Credential {
    Anonymous,
    /// `Authorization: Bearer <token>`.
    Token(String),
    /// A session: the browser holds the cookie on the web (`cookie: None`); a
    /// native password sign-in carries it itself, only long enough to mint a
    /// token. `csrf` goes on every change.
    Session { cookie: Option<String>, csrf: Option<String> },
}

/// `GET /api/store/index`: the store's metadata, one row per revision with a
/// document key. This is the seam's `load_index`.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct StoreIndex {
    pub seq: u64,
    pub entries: Vec<IndexEntry>,
}

#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct IndexEntry {
    pub key: String,
    pub part_id: String,
    pub part_number: String,
    pub part_name: String,
    pub revision_id: String,
    pub revision_label: String,
    pub lifecycle: String,
    pub editable: bool,
    pub content_hash: String,
    pub size: u64,
    pub modified: i64,
    pub locked_by: Option<String>,
    pub locked_by_me: bool,
    /// P3; absent until the server sends it.
    pub document_class: Option<String>,
}

/// `PUT /api/store/doc/*key`'s answer.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct PutAck {
    pub seq: u64,
    pub content_hash: String,
    pub size: u64,
}

/// `GET /api/store/changes?since=`.
#[derive(Debug, Clone, Default, PartialEq, Deserialize)]
#[serde(default)]
pub struct Changes {
    pub seq: u64,
    /// The client asked from further back than the log reaches: `keys` is NOT
    /// complete, and the index must be re-read.
    pub stale: bool,
    pub keys: Vec<String>,
}

/// One PLM server, and who the app is on it.
pub struct PlmClient {
    transport: Rc<dyn PlmTransport>,
    credential: RefCell<Credential>,
    me: RefCell<Option<Me>>,
}

impl PlmClient {
    pub fn new(transport: Rc<dyn PlmTransport>) -> Self {
        Self { transport, credential: RefCell::new(Credential::Anonymous), me: RefCell::new(None) }
    }

    /// Who the client is signed in as, once a sign-in has passed the version
    /// check.
    pub fn me(&self) -> Option<Me> {
        self.me.borrow().clone()
    }

    pub fn credential(&self) -> Credential {
        self.credential.borrow().clone()
    }

    /// Forget the credential locally. (A token stays valid on the server; the
    /// user revokes it on the PLM's page.)
    pub fn forget(&self) {
        *self.credential.borrow_mut() = Credential::Anonymous;
        *self.me.borrow_mut() = None;
    }

    /// One call with the current credential. Every status comes back `Ok`;
    /// only a request that got no answer is an `Err`.
    pub async fn raw(&self, method: &'static str, path: &str, body: Option<Vec<u8>>) -> Result<PlmResponse, PlmError> {
        let credential = self.credential.borrow().clone();
        self.send_as(&credential, method, path, body).await
    }

    async fn send_as(
        &self,
        credential: &Credential,
        method: &'static str,
        path: &str,
        body: Option<Vec<u8>>,
    ) -> Result<PlmResponse, PlmError> {
        let mut headers = credential_headers(credential, method);
        if body.as_deref().is_some_and(|b| serde_json::from_slice::<Value>(b).is_ok()) {
            headers.push(("Content-Type".into(), "application/json".into()));
        }
        let request = PlmRequest { method, path: path.to_string(), headers, body: body.unwrap_or_default() };
        self.transport.send(request).await.map_err(PlmError::Unreachable)
    }

    /// A 2xx-or-refusal call carrying `body` as `content_type`: a file
    /// upload, whose media type the server keeps from this header (an
    /// attachment sent without one is stored as `application/octet-stream`).
    pub(crate) async fn call_typed(
        &self,
        method: &'static str,
        path: &str,
        body: Vec<u8>,
        content_type: &str,
    ) -> Result<PlmResponse, PlmError> {
        let credential = self.credential.borrow().clone();
        let mut headers = credential_headers(&credential, method);
        headers.push(("Content-Type".into(), content_type.to_string()));
        let request = PlmRequest { method, path: path.to_string(), headers, body };
        let response = self.transport.send(request).await.map_err(PlmError::Unreachable)?;
        if (200..300).contains(&response.status) {
            Ok(response)
        } else {
            Err(PlmError::from_response(&response))
        }
    }

    /// A call whose answer must be 2xx.
    pub(crate) async fn call(&self, method: &'static str, path: &str, body: Option<Vec<u8>>) -> Result<PlmResponse, PlmError> {
        let response = self.raw(method, path, body).await?;
        if (200..300).contains(&response.status) {
            Ok(response)
        } else {
            Err(PlmError::from_response(&response))
        }
    }

    async fn call_json<T: for<'de> Deserialize<'de>>(
        &self,
        method: &'static str,
        path: &str,
        body: Option<Value>,
    ) -> Result<T, PlmError> {
        let response = self.call(method, path, body.map(|b| serde_json::to_vec(&b).unwrap_or_default())).await?;
        parse(&response, path)
    }

    // ---------------------------------------------------------------- sign-in

    /// Sign in with a token (pasted, or read from `plm-token`). Passes only if
    /// the server accepts the token AND its version serves this app.
    pub async fn sign_in_with_token(&self, token: &str) -> Result<Me, PlmError> {
        let token = token.trim();
        if token.is_empty() {
            return Err(PlmError::SignIn("paste a token (it starts with plm_)".into()));
        }
        self.adopt(Credential::Token(token.to_string())).await
    }

    /// The page is served by the PLM and the browser already holds a session
    /// cookie (D6): adopt it, and read the CSRF token changes must carry.
    pub async fn sign_in_with_browser_session(&self) -> Result<Me, PlmError> {
        self.adopt(Credential::Session { cookie: None, csrf: None }).await
    }

    /// Sign in with a username and password as a SESSION (the web lane, D6).
    pub async fn sign_in_with_password(&self, username: &str, password: &str) -> Result<Me, PlmError> {
        let cookie = self.log_in(username, password).await?;
        self.adopt(Credential::Session { cookie, csrf: None }).await
    }

    /// Trade a username and password for a token (the native lane, D5): sign
    /// in, mint a `full` token named `token_name` through that session, sign
    /// the session out, and sign in with the token. Returns the token for the
    /// caller to keep (`plm::config::write_token`); the password is not kept.
    pub async fn sign_in_for_token(&self, username: &str, password: &str, token_name: &str) -> Result<(Me, String), PlmError> {
        let cookie = self.log_in(username, password).await?;
        let session = Credential::Session { cookie, csrf: None };
        let me: Me = parse(&self.checked(&session, "GET", "/api/me", None).await?, "/api/me")?;
        let session = Credential::Session { cookie: session_cookie(&session), csrf: me.csrf.clone() };
        // Refuse BEFORE minting: a server that does not serve this app must not
        // be left holding a fresh token nobody will use.
        if let Err(refused) = check_version(&me) {
            let _ = self.send_as(&session, "POST", "/api/logout", Some(b"{}".to_vec())).await;
            return Err(refused);
        }
        let body = serde_json::to_vec(&json!({ "name": token_name, "scope": "full" })).unwrap_or_default();
        let minted: Value = parse(&self.checked(&session, "POST", "/api/tokens", Some(body)).await?, "/api/tokens")?;
        let token = minted
            .get("token")
            .and_then(Value::as_str)
            .ok_or_else(|| PlmError::Malformed("/api/tokens answered without a token".into()))?
            .to_string();
        // The session was only a way to mint the token; do not leave it open.
        let _ = self.send_as(&session, "POST", "/api/logout", Some(b"{}".to_vec())).await;
        let me = self.sign_in_with_token(&token).await?;
        Ok((me, token))
    }

    /// `POST /api/login`. The cookie comes back when the transport can see
    /// `Set-Cookie` (native); in a browser it cannot, and need not — the
    /// browser keeps it.
    async fn log_in(&self, username: &str, password: &str) -> Result<Option<String>, PlmError> {
        let body = serde_json::to_vec(&json!({ "username": username, "password": password })).unwrap_or_default();
        let response = self.checked(&Credential::Anonymous, "POST", "/api/login", Some(body)).await?;
        Ok(response
            .headers
            .iter()
            .find(|(k, _)| k.eq_ignore_ascii_case("set-cookie"))
            .and_then(|(_, v)| v.split(';').next())
            .map(|c| c.trim().to_string()))
    }

    /// A call with an explicit credential whose answer must be 2xx.
    async fn checked(&self, credential: &Credential, method: &'static str, path: &str, body: Option<Vec<u8>>) -> Result<PlmResponse, PlmError> {
        let response = self.send_as(credential, method, path, body).await?;
        if (200..300).contains(&response.status) {
            Ok(response)
        } else {
            Err(PlmError::from_response(&response))
        }
    }

    /// Read `/api/me` under `credential`, run the version check, and only then
    /// keep the credential.
    async fn adopt(&self, credential: Credential) -> Result<Me, PlmError> {
        let me: Me = parse(&self.checked(&credential, "GET", "/api/me", None).await?, "/api/me")?;
        check_version(&me)?;
        let credential = match credential {
            Credential::Session { cookie, .. } => Credential::Session { cookie, csrf: me.csrf.clone() },
            other => other,
        };
        *self.credential.borrow_mut() = credential;
        *self.me.borrow_mut() = Some(me.clone());
        Ok(me)
    }

    // ------------------------------------------------------------ the store

    /// `GET /api/store/index`.
    pub async fn index(&self) -> Result<StoreIndex, PlmError> {
        self.call_json("GET", "/api/store/index", None).await
    }

    /// `GET /api/store/index?keys=…`: only those revisions' rows (a key that
    /// names no revision any more is absent from the answer).
    pub async fn index_rows(&self, keys: &[String]) -> Result<StoreIndex, PlmError> {
        self.call_json("GET", &format!("/api/store/index?keys={}", keys.iter().map(|k| crate::plm::identity::segment(k)).collect::<Vec<_>>().join(",")), None).await
    }

    /// `GET /api/store/doc/<key>`: the bytes, or `None` when the revision has
    /// no document yet (`204`). A revision that does not exist is
    /// [`PlmError::NotFound`].
    pub async fn get_document(&self, key: &str) -> Result<Option<Vec<u8>>, PlmError> {
        let response = self.call("GET", &doc_path(key), None).await?;
        Ok(if response.status == 204 { None } else { Some(response.body) })
    }

    /// `PUT /api/store/doc/<key>`. Needs the lock and an editable revision;
    /// otherwise [`PlmError::Conflict`] with the server's sentence.
    pub async fn put_document(&self, key: &str, bytes: Vec<u8>) -> Result<PutAck, PlmError> {
        let path = doc_path(key);
        let response = self.call("PUT", &path, Some(bytes)).await?;
        parse(&response, &path)
    }

    /// `DELETE /api/store/doc/<key>`, under the same two conditions as a write.
    pub async fn delete_document(&self, key: &str) -> Result<(), PlmError> {
        self.call("DELETE", &doc_path(key), None).await.map(|_| ())
    }

    // ------------------------------------------------------ preferences (P5)

    /// `GET /api/me/preferences/<name>` (`@settings`, …): the value, or `None`
    /// when the user has never set it. Unset is `204 No Content` (server B,
    /// 2026-09-25: a hosted page's console logs every 404) and was `404`
    /// before; both read as unset.
    pub async fn get_preference(&self, name: &str) -> Result<Option<String>, PlmError> {
        match self.call("GET", &preference_path(name), None).await {
            Ok(response) if response.status == 204 => Ok(None),
            Ok(response) => String::from_utf8(response.body)
                .map(Some)
                .map_err(|_| PlmError::Malformed(format!("preference {name} is not UTF-8"))),
            Err(PlmError::NotFound(_)) => Ok(None),
            Err(e) => Err(e),
        }
    }

    /// `PUT /api/me/preferences/<name>` with the value, which must be JSON.
    pub async fn put_preference(&self, name: &str, value: String) -> Result<(), PlmError> {
        self.call("PUT", &preference_path(name), Some(value.into_bytes())).await.map(|_| ())
    }

    /// `DELETE /api/me/preferences/<name>`; deleting an unset one is fine.
    pub async fn delete_preference(&self, name: &str) -> Result<(), PlmError> {
        self.call("DELETE", &preference_path(name), None).await.map(|_| ())
    }

    /// `GET /api/store/changes?since=<seq>`.
    pub async fn changes(&self, since: u64) -> Result<Changes, PlmError> {
        self.call_json("GET", &format!("/api/store/changes?since={since}"), None).await
    }
}

/// What one poll of the change feed found.
#[derive(Debug, Clone, PartialEq)]
pub enum FeedUpdate {
    /// Nothing changed since the last poll.
    Quiet,
    /// These keys changed (in the server's order, each once).
    Keys(Vec<String>),
    /// The feed could not say what changed (the first poll, or the client
    /// fell behind the log): here is the whole index instead.
    Reindexed(StoreIndex),
}

/// The store's change feed: poll `?since=`, and re-read the index whenever
/// the answer is `stale`. The first poll reads the index, which is also where
/// the feed's position starts.
#[derive(Debug, Clone, Default)]
pub struct ChangeFeed {
    since: Option<u64>,
}

impl ChangeFeed {
    pub fn new() -> Self {
        Self::default()
    }

    /// Start from a position the caller already holds (an index it read).
    pub fn starting_at(seq: u64) -> Self {
        Self { since: Some(seq) }
    }

    pub fn position(&self) -> Option<u64> {
        self.since
    }

    pub async fn poll(&mut self, client: &PlmClient) -> Result<FeedUpdate, PlmError> {
        let Some(since) = self.since else {
            let index = client.index().await?;
            self.since = Some(index.seq);
            return Ok(FeedUpdate::Reindexed(index));
        };
        let changes = client.changes(since).await?;
        if changes.stale {
            let index = client.index().await?;
            self.since = Some(index.seq);
            return Ok(FeedUpdate::Reindexed(index));
        }
        self.since = Some(changes.seq.max(since));
        let mut keys = Vec::new();
        for key in changes.keys {
            if !keys.contains(&key) {
                keys.push(key);
            }
        }
        Ok(if keys.is_empty() { FeedUpdate::Quiet } else { FeedUpdate::Keys(keys) })
    }
}

/// P1: refuse a server that does not serve this app. The rule is the
/// server's own (`brep_plm::version::serves`, mirrored exactly and pinned
/// against it by a test): this app's version must parse and be at least
/// `min_client_version`, and a version that does not parse is not served. A
/// server that reports no version predates the handshake and is refused too:
/// it cannot promise the contracts this client reads.
pub fn check_version(me: &Me) -> Result<(), PlmError> {
    check_version_of(me, CLIENT_VERSION)
}

fn check_version_of(me: &Me, client: &str) -> Result<(), PlmError> {
    let server = me.server_version.as_deref().unwrap_or("a version it does not report");
    let refuse = |min: &str, reason: String| PlmError::Version {
        server: server.to_string(),
        min_client: min.to_string(),
        client: client.to_string(),
        reason,
    };
    let Some(min) = me.min_client_version.as_deref() else {
        return Err(refuse("?", "the server does not say which CAD versions it serves; it is older than this app".into()));
    };
    if !serves(min, client) {
        return Err(refuse(min, format!("update this CAD app to {min} or later")));
    }
    Ok(())
}

/// `MAJOR.MINOR.PATCH` as numbers, a `-pre` / `+build` suffix ignored; exactly
/// three fields. The same parse as `brep_plm::version::parse`.
pub fn parse_version(version: &str) -> Option<(u64, u64, u64)> {
    let core = version.trim().split(['-', '+']).next()?;
    let mut fields = core.split('.').map(|f| f.parse::<u64>().ok());
    let triple = (fields.next()??, fields.next()??, fields.next()??);
    fields.next().is_none().then_some(triple)
}

/// Whether a server whose oldest served client is `min_client_version` serves
/// a client at `client_version` (`brep_plm::version::serves`).
pub fn serves(min_client_version: &str, client_version: &str) -> bool {
    match (parse_version(min_client_version), parse_version(client_version)) {
        (Some(min), Some(client)) => client >= min,
        _ => false,
    }
}

fn preference_path(name: &str) -> String {
    format!("/api/me/preferences/{name}")
}

fn doc_path(key: &str) -> String {
    format!("/api/store/doc/{}", key.trim_start_matches('/').replace('%', "%25"))
}

fn session_cookie(credential: &Credential) -> Option<String> {
    match credential {
        Credential::Session { cookie, .. } => cookie.clone(),
        _ => None,
    }
}

fn parse<T: for<'de> Deserialize<'de>>(response: &PlmResponse, what: &str) -> Result<T, PlmError> {
    serde_json::from_slice(&response.body).map_err(|e| PlmError::Malformed(format!("{what}: {e}")))
}

