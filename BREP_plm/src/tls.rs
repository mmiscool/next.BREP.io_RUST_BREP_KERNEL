//! Native TLS: `brep-plm serve --tls-cert <pem> --tls-key <pem>`.
//!
//! The usual deployment puts the server behind a reverse proxy that
//! terminates TLS (then use `--trust-proxy` and `--secure-cookies auto`). This
//! is for a small install that has no proxy: rustls with the ring provider,
//! HTTP/1.1 over each TLS stream, the same router. The certificate and key
//! are read once, at start-up; renewing them means a restart.

use std::net::SocketAddr;
use std::path::Path;
use std::sync::Arc;

use axum::extract::ConnectInfo;
use axum::Router;
use hyper_util::rt::TokioIo;
use hyper_util::service::TowerToHyperService;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::TcpListener;
use tokio_rustls::rustls::pki_types::pem::PemObject;
use tokio_rustls::rustls::pki_types::{CertificateDer, PrivateKeyDer};
use tokio_rustls::rustls::{self, ServerConfig};
use tokio_rustls::TlsAcceptor;
use tower::ServiceExt;

/// The rustls configuration for a PEM certificate chain and private key.
pub fn load(cert: &Path, key: &Path) -> Result<ServerConfig, String> {
    let chain: Vec<CertificateDer<'static>> = CertificateDer::pem_file_iter(cert)
        .map_err(|e| format!("{}: {e}", cert.display()))?
        .collect::<Result<_, _>>()
        .map_err(|e| format!("{}: {e}", cert.display()))?;
    if chain.is_empty() {
        return Err(format!("{}: no certificate in the file", cert.display()));
    }
    let key = PrivateKeyDer::from_pem_file(key).map_err(|e| format!("{}: {e}", key.display()))?;
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    let mut config = ServerConfig::builder_with_provider(provider)
        .with_safe_default_protocol_versions()
        .map_err(|e| e.to_string())?
        .with_no_client_auth()
        .with_single_cert(chain, key)
        .map_err(|e| format!("the certificate and key do not make a usable pair: {e}"))?;
    config.alpn_protocols = vec![b"http/1.1".to_vec()];
    Ok(config)
}

/// Serve `app` over TLS on `listener` until `shutdown` resolves. HTTP/1.1
/// without upgrades: the PLM's router has no upgrade route, and each stream is
/// kept by its task so it can be closed cleanly (see `close_gracefully`).
pub async fn serve(
    listener: TcpListener,
    app: Router,
    config: ServerConfig,
    shutdown: impl std::future::Future<Output = ()>,
) {
    let acceptor = TlsAcceptor::from(Arc::new(config));
    tokio::pin!(shutdown);
    loop {
        let (stream, peer) = tokio::select! {
            _ = &mut shutdown => return,
            accepted = listener.accept() => match accepted {
                Ok(pair) => pair,
                Err(error) => {
                    eprintln!("brep-plm: accept: {error}");
                    continue;
                }
            },
        };
        let acceptor = acceptor.clone();
        let app = app.clone();
        tokio::spawn(async move {
            // A client that never finishes the handshake must not hold a task
            // forever.
            let handshake = tokio::time::timeout(std::time::Duration::from_secs(15), acceptor.accept(stream)).await;
            let Ok(Ok(mut tls)) = handshake else { return };
            let service = app.map_request(move |mut request: axum::http::Request<hyper::body::Incoming>| {
                request.extensions_mut().insert(ConnectInfo::<SocketAddr>(peer));
                request
            });
            // Served over a borrow of the stream, so it is this task's to
            // close however the connection ends: after the last response, and
            // after an error hyper answered itself (a malformed request's 400).
            let _ = hyper::server::conn::http1::Builder::new()
                .serve_connection(TokioIo::new(&mut tls), TowerToHyperService::new(service))
                .await;
            close_gracefully(tls).await;
        });
    }
}

/// How long, and how much, a closing connection reads of what its client
/// still sends.
const LINGER: std::time::Duration = std::time::Duration::from_secs(3);
const LINGER_BYTES: usize = 8 << 20;

/// Close a TLS connection so its last response reaches the client. A handler
/// may answer before reading the request's body (a refusal); dropping the
/// stream then closes the socket with the client's unread body in it, the
/// kernel answers with a reset, and the client's kernel discards the response
/// it had not read yet (curl's 000, 28 of 30 refused 300 KB uploads). So the
/// write side is shut first (close_notify, then a TCP half-close the client
/// can still send into), and the rest of what it sends is read and dropped
/// until it closes, for at most `LINGER` and `LINGER_BYTES`.
async fn close_gracefully<S: tokio::io::AsyncRead + tokio::io::AsyncWrite + Unpin>(mut tls: S) {
    if tls.shutdown().await.is_err() {
        return;
    }
    let _ = tokio::time::timeout(LINGER, async {
        let mut buf = vec![0u8; 16 << 10];
        let mut total = 0usize;
        loop {
            match tls.read(&mut buf).await {
                Ok(0) | Err(_) => break,
                Ok(n) => {
                    total += n;
                    if total >= LINGER_BYTES {
                        break;
                    }
                }
            }
        }
    })
    .await;
}

