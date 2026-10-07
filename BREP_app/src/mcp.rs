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
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The port `--mcp` listens on unless `--mcp-port` says otherwise.
pub const DEFAULT_PORT: u16 = 8765;

/// How long one command may wait for the window to answer. A window paints
/// on demand (every submit requests a repaint), so a reply normally arrives
/// within a frame or two; this bounds a hung or minimised app.
const REPLY_TIMEOUT: Duration = Duration::from_secs(120);

/// The ports the Info window's **Start MCP server** button tries, in order:
/// the default, then the next few above it. The flag tries ONE port (the
/// default or `--mcp-port`) and exits when it is taken, because a scripted
/// launch wants the address it asked for; a person clicking a button wants a
/// server, and is told which port it got.
pub const WINDOW_PORTS: std::ops::RangeInclusive<u16> = DEFAULT_PORT..=DEFAULT_PORT + 9;

/// A bound listener and where its sessions go: what [`start`] serves. Built by
/// [`launch`] — the ONE place the sequence "bind, choose the session root,
/// word the instructions" lives, for the flag and the button alike.
#[derive(Debug)]
pub struct Launch {
    pub listener: TcpListener,
    pub session_root: PathBuf,
    /// Set when the first port asked for was taken and a later one answered:
    /// the sentence that says so, shown above the instructions.
    pub note: Option<String>,
}

impl Launch {
    pub fn url(&self) -> String {
        url(&self.listener)
    }

    /// The text the console prints and the Info window shows: one function,
    /// so what a user copies from the window is what a terminal would show.
    pub fn instructions(&self) -> String {
        agent_instructions(&self.url(), &self.session_root)
    }
}

/// Bind the loopback listener. Done before the window opens so a port in use
/// is a launch error, never a window silently running without its server.
pub fn bind(port: u16) -> Result<TcpListener, String> {
    TcpListener::bind(("127.0.0.1", port)).map_err(|e| format!("cannot listen on 127.0.0.1:{port}: {e}"))
}

/// Bind the first free port of `ports` and settle the session root: the
/// start sequence shared by `--mcp` (one port) and the window's button (the
/// [`WINDOW_PORTS`] range). `Ok` carries a [`Launch`] whose `note` names the
/// fallback when one happened; `Err` is the bind error of the first port,
/// with the count of the others tried, in the console's own words.
pub fn launch(ports: impl IntoIterator<Item = u16>, session_root: Option<PathBuf>) -> Result<Launch, String> {
    let mut failures: Vec<(u16, String)> = Vec::new();
    for port in ports {
        match bind(port) {
            Ok(listener) => {
                // The port actually bound, not the one asked for: `0` asks
                // the OS for any free port, and a note must name a real one.
                let bound = listener.local_addr().map(|a| a.port()).unwrap_or(port);
                let note = failures
                    .first()
                    .map(|(first, _)| format!("port {first} was in use; listening on {bound} instead"));
                let session_root = session_root.unwrap_or_else(default_session_root);
                return Ok(Launch { listener, session_root, note });
            }
            Err(e) => failures.push((port, e)),
        }
    }
    match failures.as_slice() {
        [] => Err("no port to listen on".to_string()),
        [(_, only)] => Err(only.clone()),
        [(_, first), .., (last, _)] => Err(format!("{first} (and every port up to {last} is in use too)")),
    }
}

/// Whether the server is up, for the Info window: written by [`start`] and by
/// the server thread when it stops, read each frame the window is open.
/// Process-global like the registry it serves: one app per process is the
/// only configuration that exists, and `start` runs at most once in it.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Status {
    /// Never started, or stopped: `error` is the line the console printed when
    /// something went wrong (`None` for a plain "not started").
    Stopped { error: Option<String> },
    /// Serving on `url`; `note` is [`Launch::note`].
    Running { url: String, session_root: PathBuf, note: Option<String> },
}

static STATUS: Mutex<Status> = Mutex::new(Status::Stopped { error: None });

pub fn status() -> Status {
    STATUS.lock().unwrap_or_else(|p| p.into_inner()).clone()
}

fn set_status(status: Status) {
    *STATUS.lock().unwrap_or_else(|p| p.into_inner()) = status;
}

/// Report a failure the way the console always has — `brep-app --mcp: …` on
/// stderr — AND record the same line for the window, so the panel and the
/// terminal cannot word it differently.
fn fail(text: impl std::fmt::Display) -> String {
    let line = format!("brep-app --mcp: {text}");
    eprintln!("{line}");
    set_status(Status::Stopped { error: Some(line.clone()) });
    line
}

/// What the Info window's "Connect an agent" section draws, from [`status`].
pub fn info_view() -> crate::panels::info::McpView {
    use crate::panels::info::McpView;
    match status() {
        Status::Stopped { error } => McpView::Stopped { error },
        Status::Running { url, session_root, note } => McpView::Running {
            instructions: agent_instructions(&url, &session_root),
            url,
            note,
        },
    }
}

/// The button's path: [`launch`] over [`WINDOW_PORTS`] with the default
/// session root, then [`start`] — the flag's sequence, at runtime. The
/// instructions still go to the console too, for a window launched from a
/// terminal. A failure is recorded in [`status`] (the window shows it) and
/// returned.
pub fn start_from_window(queue: Arc<AutomationQueue>, adapter: String) -> Result<(), String> {
    start_over(queue, adapter, WINDOW_PORTS)
}

fn start_over(queue: Arc<AutomationQueue>, adapter: String, ports: impl IntoIterator<Item = u16>) -> Result<(), String> {
    // A refusal is not a failure: it must not touch the status (the button
    // is disabled while the server runs, so this is belt and braces), and it
    // must not bind a port it would only drop.
    if let Status::Running { url, .. } = status() {
        return Err(format!("the MCP server is already running on {url}"));
    }
    let launch = match launch(ports, None) {
        Ok(launch) => launch,
        // main.rs's wording for the same failure, which also goes to stderr.
        Err(e) => {
            let line = format!("brep-app: {e}");
            eprintln!("{line}");
            set_status(Status::Stopped { error: Some(line.clone()) });
            return Err(line);
        }
    };
    if let Some(note) = &launch.note {
        println!("brep-app --mcp: {note}");
    }
    println!("{}", launch.instructions());
    start(queue, adapter, launch).map_err(|e| fail(e))
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
/// creation closure, once the app (and its queue) exists — or from the Info
/// window's button, any time later; returns as soon as the thread is spawned.
/// `adapter` names the GPU for the session info. At most ONCE per process: a
/// second call while the server runs is refused with its address, so neither
/// path can stack a second server on the first.
pub fn start(queue: Arc<AutomationQueue>, adapter: String, launch: Launch) -> Result<(), String> {
    {
        let mut status = STATUS.lock().unwrap_or_else(|p| p.into_inner());
        if let Status::Running { url, .. } = &*status {
            return Err(format!("the MCP server is already running on {url}"));
        }
        *status = Status::Running { url: launch.url(), session_root: launch.session_root.clone(), note: launch.note.clone() };
    }
    // The state registry publishes only for a host; this window has one now.
    // Part of the start sequence, not of either caller, so an agent connected
    // through the button sees `hit_rects` and `state_get` like one connected
    // at launch.
    crate::automation::registry::set_enabled(true);
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .thread_name("brep-mcp-worker")
        .enable_all()
        .build()
        .map_err(|e| {
            set_status(Status::Stopped { error: None });
            format!("tokio runtime: {e}")
        })?;
    let url = launch.url();
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
                        fail(format!("cannot attach the server to the app: {e}"));
                        return;
                    }
                }
                let listener = match launch.listener.set_nonblocking(true).and_then(|_| tokio::net::TcpListener::from_std(launch.listener)) {
                    Ok(l) => l,
                    Err(e) => {
                        fail(format!("listener: {e}"));
                        return;
                    }
                };
                if let Err(e) = brep_mcp_core::http::serve_http(server, listener).await {
                    fail(format!("server on {url} stopped: {e}"));
                }
            });
        })
        .map_err(|e| {
            set_status(Status::Stopped { error: None });
            format!("spawn the MCP server thread: {e}")
        })?;
    Ok(())
}

