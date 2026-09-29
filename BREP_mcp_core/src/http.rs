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
pub async fn serve_http(server: BrepServer, listener: TcpListener) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let state = server.state.clone();
    let service = StreamableHttpService::new(
        move || Ok(BrepServer { state: state.clone() }),
        Arc::new(LocalSessionManager::default()),
        // No SEP-1699 priming event. rmcp's default opens every reply stream
        // with an empty `data:` / `id: 0` / `retry: 3000` frame before the
        // JSON-RPC message, and a client that reads the first `data:` as the
        // reply parses "". There is no event store to resume from, so the
        // message is the first event of its stream.
        StreamableHttpServerConfig::default().with_sse_retry(None),
    );
    loop {
        let (stream, _peer) = listener.accept().await?;
        let io = TokioIo::new(stream);
        let svc = TowerToHyperService::new(service.clone());
        tokio::spawn(async move {
            if let Err(e) = hyper::server::conn::http1::Builder::new().serve_connection(io, svc).await {
                eprintln!("brep-mcp http: {e}");
            }
        });
    }
}

