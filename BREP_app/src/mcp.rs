//! The embedded MCP server — `brep-app --mcp`.
//!
//! The window you launched is the session: the server (`brep_mcp_core`, the
//! same one `brep-mcp serve` runs headlessly) lives on its own thread inside
//! this process and reaches the app through the automation queue, exactly as
//! the headless host does. Agents connect over MCP's streamable HTTP
//! transport on a loopback port instead of launching a process, so Claude
//! Code or Codex can drive the app a person is looking at.
//!
//! Sequence (main.rs): bind the listener → print how to configure the agent →
//! open the window → once the app exists, [`start`] attaches the server to its
//! queue and serves. The store is the user's own (this is their app, not a
//! test host), which is what the tools' docs say.
use crate::automation::command::{Envelope as AppEnvelope, Reply as AppReply};
use crate::automation::queue::AutomationQueue;
use brep_mcp_core::host::{Backend, Envelope, HostInfo, HostRequest, Reply};
use brep_mcp_core::server::BrepServer;
use serde_json::{json, Value};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

/// The port `--mcp` listens on unless `--mcp-port` says otherwise.
pub const DEFAULT_PORT: u16 = 8765;

/// How long one command may wait for the window to answer. A window paints
/// on demand (every submit requests a repaint), so a reply normally arrives
/// within a frame or two; this bounds a hung or minimised app.
const REPLY_TIMEOUT: Duration = Duration::from_secs(120);

/// What main.rs hands to [`start`] once the app exists.
pub struct Launch {
    pub listener: TcpListener,
    pub session_root: PathBuf,
}

/// Bind the loopback listener. Done before the window opens so a port in use
/// is a launch error, never a window silently running without its server.
pub fn bind(port: u16) -> Result<TcpListener, String> {
    TcpListener::bind(("127.0.0.1", port)).map_err(|e| format!("cannot listen on 127.0.0.1:{port}: {e}"))
}

pub fn url(listener: &TcpListener) -> String {
    let port = listener.local_addr().map(|a| a.port()).unwrap_or(DEFAULT_PORT);
    brep_mcp_core::http::url(port)
}

/// Where sessions keep screenshots and call logs unless `--session-root`
/// or `BREP_MCP_SESSION_ROOT` says otherwise (the same default as brep-mcp).
pub fn default_session_root() -> PathBuf {
    std::env::var_os("BREP_MCP_SESSION_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| std::env::temp_dir().join("brep-mcp"))
}

/// The text printed in the terminal at launch: how to point an agent at this
/// window. Both forms each agent accepts — its CLI and its config file.
pub fn agent_instructions(url: &str, session_root: &Path) -> String {
    format!(
        "BREP MCP server listening on {url}\n\
         This window is the session: an agent's tools act on the documents you see here.\n\
         \n\
         Claude Code:\n\
         \x20 claude mcp add --transport http brep {url}\n\
         \x20 or in a project's .mcp.json:\n\
         \x20   {{ \"mcpServers\": {{ \"brep\": {{ \"type\": \"http\", \"url\": \"{url}\" }} }} }}\n\
         \n\
         Codex:\n\
         \x20 codex mcp add brep --url {url}\n\
         \x20 or in ~/.codex/config.toml:\n\
         \x20   [mcp_servers.brep]\n\
         \x20   url = \"{url}\"\n\
         \n\
         Then ask the agent to call session_start; screenshots and call logs land under\n\
         \x20 {root}\n",
        root = session_root.display(),
    )
}

/// The app's reply in the server's shape: the same JSON, the blob beside it.
fn convert(mut r: AppReply) -> Reply {
    let blob = r.blob.take();
    let (id, frame) = (r.id, r.frame);
    match serde_json::to_value(&r).map_err(|e| e.to_string()).and_then(|v| Reply::from_app(v, blob)) {
        Ok(reply) => reply,
        Err(e) => Reply::err(id, frame, e),
    }
}

/// Submit one command to the window and wait for its reply off the runtime.
async fn forward(queue: &Arc<AutomationQueue>, envelope: Envelope) -> Reply {
    let id = envelope.id;
    let rx = queue.submit(AppEnvelope { id, cmd: envelope.cmd, args: envelope.args });
    match tokio::task::spawn_blocking(move || rx.recv_timeout(REPLY_TIMEOUT)).await {
        Ok(Ok(reply)) => convert(reply),
        Ok(Err(std::sync::mpsc::RecvTimeoutError::Timeout)) => Reply::err(
            id,
            queue.frame(),
            format!("no reply from the app within {} s (is the window responding?)", REPLY_TIMEOUT.as_secs()),
        ),
        Ok(Err(std::sync::mpsc::RecvTimeoutError::Disconnected)) => Reply::err(id, queue.frame(), "the app dropped the command"),
        Err(e) => Reply::err(id, queue.frame(), e.to_string()),
    }
}

/// A capture is the app's own `screenshot` command (egui's viewport
/// screenshot, completed on a later frame); the PNG rides in the reply blob.
async fn capture(queue: &Arc<AutomationQueue>, region: Value) -> Result<(Value, Vec<u8>), String> {
    let reply = forward(queue, Envelope { id: 0, cmd: "screenshot".into(), args: json!({ "region": region }) }).await;
    if !reply.ok {
        return Err(reply.error.unwrap_or_else(|| "screenshot failed".into()));
    }
    let png = reply.blob.ok_or("the screenshot reply carried no image")?;
    Ok((reply.result.unwrap_or(Value::Null), png))
}

/// Start the server thread over the app's queue. Called from the eframe
/// creation closure, once the app (and its queue) exists; returns as soon as
/// the thread is spawned. `adapter` names the GPU for the session info.
pub fn start(queue: Arc<AutomationQueue>, adapter: String, launch: Launch) -> Result<(), String> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .thread_name("brep-mcp-worker")
        .enable_all()
        .build()
        .map_err(|e| format!("tokio runtime: {e}"))?;
    let url = url(&launch.listener);
    std::thread::Builder::new()
        .name("brep-mcp".into())
        .spawn(move || {
            rt.block_on(async move {
                let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<HostRequest>();
                let q = queue.clone();
                tokio::spawn(async move {
                    while let Some(req) = rx.recv().await {
                        let q = q.clone();
                        match req {
                            HostRequest::Command { envelope, reply } => {
                                tokio::spawn(async move {
                                    let _ = reply.send(forward(&q, envelope).await);
                                });
                            }
                            HostRequest::Screenshot { region, reply } => {
                                tokio::spawn(async move {
                                    let _ = reply.send(capture(&q, region).await);
                                });
                            }
                            // The DOCS ANNOTATION overlay belongs to the
                            // headless harness, which owns its egui context and
                            // paints on top of the app's frame for the
                            // walkthrough GIFs. A running window draws the
                            // product, not a caption band over it, so this says
                            // so rather than capturing an unannotated frame and
                            // letting a silent GIF claim otherwise.
                            HostRequest::Annotate { reply, .. } => {
                                let _ = reply.send(Err(
                                    "the window host draws no annotation overlay: docs walkthroughs are recorded by the headless host (brep-mcp)".to_string(),
                                ));
                            }
                            // The app is not the server's to stop.
                            HostRequest::Stop => {}
                        }
                    }
                });
                let info = HostInfo {
                    backend: "window",
                    adapter,
                    platform: std::env::consts::OS,
                    width: 0.0,
                    height: 0.0,
                    ppp: 1.0,
                };
                let server = BrepServer::new(launch.session_root, Backend::Attached { tx, info });
                // Attach now: the first `tools/list` must already carry the
                // app's commands (the HTTP transport may answer statelessly).
                match server.attach(true).await {
                    Ok(info) => log::info!("brep-app --mcp: attached session {} ({}x{} @ {})", info.id, info.host.width, info.host.height, info.host.ppp),
                    Err(e) => {
                        eprintln!("brep-app --mcp: cannot attach the server to the app: {e}");
                        return;
                    }
                }
                let listener = match launch.listener.set_nonblocking(true).and_then(|_| tokio::net::TcpListener::from_std(launch.listener)) {
                    Ok(l) => l,
                    Err(e) => {
                        eprintln!("brep-app --mcp: listener: {e}");
                        return;
                    }
                };
                if let Err(e) = brep_mcp_core::http::serve_http(server, listener).await {
                    eprintln!("brep-app --mcp: server on {url} stopped: {e}");
                }
            });
        })
        .map_err(|e| format!("spawn the MCP server thread: {e}"))?;
    Ok(())
}

