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

/// Serve `app` over TLS on `listener` until `shutdown` resolves.
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
            let Ok(Ok(tls)) = handshake else { return };
            let service = app.map_request(move |mut request: axum::http::Request<hyper::body::Incoming>| {
                request.extensions_mut().insert(ConnectInfo::<SocketAddr>(peer));
                request
            });
            let _ = hyper::server::conn::http1::Builder::new()
                .serve_connection(TokioIo::new(tls), TowerToHyperService::new(service))
                .with_upgrades()
                .await;
        });
    }
}

