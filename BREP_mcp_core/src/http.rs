//! The server over MCP's streamable HTTP transport, for an app that is already
//! running (`brep-app --mcp`): an agent connects to `http://127.0.0.1:<port>/mcp`
//! instead of launching a process. rmcp's transport is a tower service; hyper
//! serves it connection by connection on the listener the caller bound.
//!
//! Every request shares one [`ServerState`](crate::server::ServerState): the
//! session slot and the tool set are the same whichever connection asks, so a
//! client that lists tools sees the attached app's commands from its first
//! request. rmcp validates `Host` against loopback names by default, which is
//! the DNS-rebinding guard a local server needs; the listener itself is bound
//! by the caller (to 127.0.0.1 in the app).
//!
//! Every request also carries the loopback credential (D3, [`Credential`]):
//! `Authorization: Bearer <token>`, the token read from a 0600 file in the
//! app's config directory (`mcp-token`, created on first use), so another
//! local user — or a page in a browser — cannot drive the app. A request
//! with an `Origin` header must come from a loopback origin.
use crate::server::BrepServer;
use hyper_util::rt::TokioIo;
use hyper_util::service::TowerToHyperService;
use rmcp::transport::streamable_http_server::session::local::LocalSessionManager;
use rmcp::transport::{StreamableHttpServerConfig, StreamableHttpService};
use std::sync::Arc;
use tokio::net::TcpListener;

/// The URL agents are told to connect to for a server on `port`.
pub fn url(port: u16) -> String {
    format!("http://127.0.0.1:{port}/mcp")
}

/// Serve until the listener fails. Each accepted connection runs on its own
/// task; a connection error ends that connection only.
/// The largest request body the HTTP transport reads (contract §12).
pub const MAX_BODY_BYTES: usize = 1 << 20;

/// The loopback HTTP credential: a bearer token kept in a file only its
/// owner can read. The file outlives the server, so an agent configured once
/// keeps working; delete it to mint a new one.
#[derive(Clone)]
pub struct Credential {
    token: Arc<str>,
}

impl std::fmt::Debug for Credential {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("Credential(..)")
    }
}

/// The app's config directory, by the crate's one resolver
/// ([`crate::launch_state::config_dir`]: `brep_app::plm::config::app_config_dir`'s
/// rule), so `brep-app --mcp` and `brep-mcp serve --http` share one token.
///
/// With no `XDG_CONFIG_HOME`, `APPDATA` (Windows) or `HOME`, the token lives
/// in `./brep-app`, where the app's own config falls back to as well: a
/// server started without a home directory still starts, its token file is
/// owner-only, and `BREP_MCP_TOKEN_FILE` names another place. (A launch
/// profile, which `launch_state` refuses to write in the working directory, is
/// not a secret every start needs.)
fn config_dir() -> std::path::PathBuf {
    crate::launch_state::config_dir().unwrap_or_else(|| std::path::PathBuf::from(".").join("brep-app"))
}

impl Credential {
    /// Where the token lives: `BREP_MCP_TOKEN_FILE`, else `mcp-token` in the
    /// app's config directory.
    pub fn path() -> std::path::PathBuf {
        std::env::var_os("BREP_MCP_TOKEN_FILE").map(std::path::PathBuf::from).unwrap_or_else(|| config_dir().join("mcp-token"))
    }

    /// The token at [`Self::path`], minted (256 random bits, owner-only file)
    /// when there is none. A file others may read is narrowed to its owner.
    pub fn load_or_create() -> Result<Credential, String> {
        Self::load_or_create_at(&Self::path())
    }

    /// [`Self::load_or_create`] for the token file at `path`.
    pub fn load_or_create_at(path: &std::path::Path) -> Result<Credential, String> {
        match std::fs::read_to_string(&path) {
            Ok(text) => {
                let token = text.trim();
                if token.len() < 32 {
                    return Err(format!("{}: the MCP token is shorter than 32 characters; delete the file to mint a new one", path.display()));
                }
                #[cfg(unix)]
                {
                    use std::os::unix::fs::PermissionsExt;
                    let mode = std::fs::metadata(&path).map_err(|e| format!("{}: {e}", path.display()))?.permissions().mode();
                    if mode & 0o077 != 0 {
                        std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600)).map_err(|e| format!("{}: {e}", path.display()))?;
                    }
                }
                Ok(Credential { token: token.into() })
            }
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                let mut bytes = crate::cad::wire::random_128().to_vec();
                bytes.extend(crate::cad::wire::random_128());
                let token = crate::cad::wire::b64url(&bytes);
                if let Some(dir) = path.parent() {
                    std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
                }
                let mut options = std::fs::OpenOptions::new();
                options.write(true).create_new(true);
                #[cfg(unix)]
                {
                    use std::os::unix::fs::OpenOptionsExt;
                    options.mode(0o600);
                }
                use std::io::Write as _;
                options.open(&path).and_then(|mut f| f.write_all(format!("{token}\n").as_bytes())).map_err(|e| format!("{}: {e}", path.display()))?;
                Ok(Credential { token: token.into() })
            }
            Err(e) => Err(format!("{}: {e}", path.display())),
        }
    }

    /// A credential with this token (a test's own).
    pub fn from_token(token: &str) -> Credential {
        Credential { token: token.into() }
    }

    pub fn token(&self) -> &str {
        &self.token
    }

    /// Why a request is refused, or `None` when it may proceed: the bearer
    /// token must match (compared in constant time), and an `Origin`, when a
    /// browser sends one, must be a loopback origin (§11).
    fn refusal(&self, headers: &hyper::HeaderMap) -> Option<(hyper::StatusCode, &'static str)> {
        if let Some(origin) = headers.get(hyper::header::ORIGIN) {
            let origin = origin.to_str().unwrap_or("");
            let allowed = origin.parse::<hyper::Uri>().ok().is_some_and(|uri| {
                matches!(uri.scheme_str(), Some("http" | "https"))
                    && matches!(uri.host(), Some("127.0.0.1" | "localhost" | "[::1]"))
                    && uri.authority().is_some_and(|a| {
                        let host = uri.host().unwrap_or("");
                        a.as_str() == host || a.port_u16().is_some_and(|port| a.as_str() == format!("{host}:{port}"))
                    })
                    && matches!(uri.path(), "" | "/")
                    && uri.query().is_none()
            });
            if !allowed {
                return Some((hyper::StatusCode::FORBIDDEN, "origin not allowed: the BREP MCP server answers loopback origins only"));
            }
        }
        let given = headers.get(hyper::header::AUTHORIZATION).and_then(|v| v.to_str().ok()).and_then(|v| v.strip_prefix("Bearer ")).unwrap_or("");
        let want = self.token.as_bytes();
        let same = given.len() == want.len() && given.bytes().zip(want).fold(0u8, |acc, (a, b)| acc | (a ^ b)) == 0;
        (!same).then_some((hyper::StatusCode::UNAUTHORIZED, "missing or wrong bearer token: send `Authorization: Bearer <token>`, the token in the server's mcp-token file"))
    }
}

pub async fn serve_http(server: BrepServer, listener: TcpListener, credential: Credential) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let state = server.state.clone();
    let service = StreamableHttpService::new(
        move || Ok(BrepServer { state: state.clone() }),
        Arc::new(LocalSessionManager::default()),
        // No SEP-1699 priming event. rmcp's default opens every reply stream
        // with an empty `data:` / `id: 0` / `retry: 3000` frame before the
        // JSON-RPC message, and a client that reads the first `data:` as the
        // reply parses "". There is no event store to resume from, so the
        // message is the first event of its stream.
        //
        // Request bodies are capped at MAX_BODY_BYTES (contract §12: 1 MiB,
        // refused before deserialisation — rmcp answers 413). A model bigger
        // than that goes in by path (`document_open`, `document_import`) or as
        // a STEP artifact, never inline.
        StreamableHttpServerConfig::default().with_sse_retry(None).with_max_request_body_bytes(MAX_BODY_BYTES),
    );
    loop {
        let (stream, _peer) = listener.accept().await?;
        let io = TokioIo::new(stream);
        let svc = TowerToHyperService::new(service.clone());
        let credential = credential.clone();
        let guarded = hyper::service::service_fn(move |req: hyper::Request<hyper::body::Incoming>| {
            let refused = credential.refusal(req.headers());
            let admitted = refused.is_none().then(|| hyper::service::Service::call(&svc, req));
            async move {
                match (refused, admitted) {
                    (_, Some(reply)) => reply.await,
                    (Some((status, why)), None) => {
                        use http_body_util::BodyExt as _;
                        let body = http_body_util::Full::new(hyper::body::Bytes::from(why)).boxed();
                        let mut reply = hyper::Response::new(body);
                        *reply.status_mut() = status;
                        if status == hyper::StatusCode::UNAUTHORIZED {
                            reply.headers_mut().insert(hyper::header::WWW_AUTHENTICATE, hyper::header::HeaderValue::from_static("Bearer"));
                        }
                        Ok(reply)
                    }
                    (None, None) => unreachable!("admitted or refused"),
                }
            }
        });
        tokio::spawn(async move {
            if let Err(e) = hyper::server::conn::http1::Builder::new().serve_connection(io, guarded).await {
                eprintln!("brep-mcp http: {e}");
            }
        });
    }
}

