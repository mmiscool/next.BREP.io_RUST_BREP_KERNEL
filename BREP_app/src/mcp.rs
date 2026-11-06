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
//! open the window → once the app exists, [`start`](crate::mcp::start) attaches the server to its
//! queue and serves. The store is the user's own (this is their app, not a
//! test host), which is what the tools' docs say.
use crate::automation::command::Envelope as AppEnvelope;
use crate::automation::queue::AutomationQueue;
use brep_mcp_core::host::{Backend, Envelope, HostInfo, HostRequest, NativePlatform, Reply};
use brep_mcp_core::server::BrepServer;
use serde_json::{json, Value};
use std::net::TcpListener;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// The port `--mcp` listens on unless `--mcp-port` says otherwise.
pub const DEFAULT_PORT: u16 = 8765;

/// How long one command may wait for the window to answer. A submit wakes the
/// desktop loop, which dispatches app-state commands at once and runs an egui
/// pass itself for input, layout reads and captures (`desktop_loop`), so no
/// command waits for the window to be drawn. This bounds a hung app.
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
    /// Port reserved; application attachment has not completed yet.
    Starting { url: String, session_root: PathBuf, note: Option<String> },
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
        Status::Starting { url, session_root, note } => McpView::Running {
            instructions: agent_instructions(&url, &session_root),
            url,
            note: Some(format!("Application host not ready; waiting for command dispatch. {}", note.unwrap_or_default())),
        },
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
    if let Status::Running { url, .. } | Status::Starting { url, .. } = status() {
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
/// window. Both forms each agent accepts — its CLI and its config file —
/// each sending the server's token (D3), which stays in its file: the text
/// names the file, never the secret.
pub fn agent_instructions(url: &str, session_root: &Path) -> String {
    instructions_naming(url, session_root, &brep_mcp_core::http::Credential::path())
}

/// [`agent_instructions`] naming this token file.
fn instructions_naming(url: &str, session_root: &Path, token: &Path) -> String {
    format!(
        "BREP MCP endpoint {url} (serving after application host readiness)\n\
         Headless automation: use brep-mcp serve --http PORT with its own free port.\n\
         HTTP uses bearer authentication, not OAuth or Dynamic Client Registration.\n\
         This window is the session: an agent's tools act on the documents you see here.\n\
         Every request needs the server's token, `Authorization: Bearer <token>`; the token is in\n\
         \x20 {token} (readable by you only).\n\
         \n\
         Claude Code:\n\
         \x20 In a project's .mcp.json, with BREP_MCP_TOKEN loaded privately from the token file:\n\
         \x20   {{ \"mcpServers\": {{ \"brep\": {{ \"type\": \"http\", \"url\": \"{url}\", \"headers\": {{ \"Authorization\": \"Bearer ${{BREP_MCP_TOKEN}}\" }} }} }} }}\n\
         \n\
         Codex, with BREP_MCP_TOKEN loaded privately from the token file:\n\
         \x20 codex mcp add brep --url {url} --bearer-token-env-var BREP_MCP_TOKEN\n\
         \x20 or in ~/.codex/config.toml:\n\
         \x20   [mcp_servers.brep]\n\
         \x20   url = \"{url}\"\n\
         \x20   bearer_token_env_var = \"BREP_MCP_TOKEN\"\n\
         \n\
         Then ask the agent to use run_script; screenshots and call logs land under\n\
         \x20 {root}\n",
        token = token.display(),
        root = session_root.display(),
    )
}

/// Submit one command to the window and wait for its reply off the runtime.
async fn forward(queue: &Arc<AutomationQueue>, envelope: Envelope) -> Reply {
    let id = envelope.id;
    let deadline = std::time::Instant::now() + REPLY_TIMEOUT;
    let rx = queue.submit_before(AppEnvelope { id, cmd: envelope.cmd, args: envelope.args }, Some(deadline));
    match tokio::task::spawn_blocking(move || (rx.recv_timeout(REPLY_TIMEOUT), rx)).await {
        Ok((Ok(reply), _)) => reply.into_host(),
        Ok((Err(std::sync::mpsc::RecvTimeoutError::Timeout), rx)) => {
            // Say which: a command still queued is withdrawn and never runs;
            // one a handler accepted may have had (or still have) effects.
            // A withdrawal answers on `rx`, as does work that finished (or
            // expired in the queue) in the meantime: that reply is the truth.
            let _ = queue.withdraw(id);
            match rx.try_recv() {
                Ok(reply) => reply.into_host(),
                Err(_) => Reply::err(
                    id,
                    queue.frame(),
                    format!("host_deadline_exceeded: no reply within {} s; the app accepted the command and it may have had effects; host frame {}; reconcile before retrying", REPLY_TIMEOUT.as_secs(), queue.frame()),
                ),
            }
        }
        Ok((Err(std::sync::mpsc::RecvTimeoutError::Disconnected), _)) => Reply::err(id, queue.frame(), "the app dropped the command"),
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

/// The app's one server, attached to its queue: created by the first of
/// [`start`] (which adds the HTTP listener) and the JavaScript editor's Run
/// (which needs none), and shared by both. It and its runtime live for the
/// process.
pub struct Embedded {
    pub server: BrepServer,
    pub runtime: tokio::runtime::Runtime,
    /// The attach's outcome once it has finished.
    attached: Arc<std::sync::OnceLock<Result<(), String>>>,
    /// Signalled once `attached` is set.
    attached_signal: Arc<tokio::sync::Notify>,
}

static SESSION_ROOT: std::sync::OnceLock<PathBuf> = std::sync::OnceLock::new();

static EMBEDDED: std::sync::OnceLock<Result<Embedded, String>> = std::sync::OnceLock::new();

static STATE_CONFIGURATION: std::sync::OnceLock<Result<brep_mcp_core::launch_state::Resolved, String>> = std::sync::OnceLock::new();

/// Freeze the desktop policy before either the editor or HTTP starts the singleton.
/// Failed resolution is sticky too: never substitute an ephemeral identity.
pub fn configure_state(policy: &brep_mcp_core::launch_state::Policy) -> Result<(), String> {
    let resolved = brep_mcp_core::launch_state::resolve(brep_mcp_core::launch_state::Host::Desktop, policy);
    STATE_CONFIGURATION.set(resolved.clone()).map_err(|_| "MCP state configuration is already fixed for this launch".to_string())?;
    resolved.map(|_| ()).map_err(fail)
}

/// Read-only UI snapshot; looking at settings must not initialize the server.
pub fn state_configuration() -> Option<&'static Result<brep_mcp_core::launch_state::Resolved, String>> {
    STATE_CONFIGURATION.get()
}

pub fn state_settings_path() -> Result<PathBuf, String> {
    brep_mcp_core::launch_state::config_dir().map(|p| p.join("mcp-state.json"))
        .ok_or_else(|| "cannot locate the machine-local MCP settings directory".into())
}

pub fn save_desktop_policy(policy: brep_mcp_core::launch_state::Policy) -> Result<(), String> {
    save_desktop_policy_at(&state_settings_path()?, policy)
}

fn save_desktop_policy_at(path: &Path, policy: brep_mcp_core::launch_state::Policy) -> Result<(), String> {
    policy.validate()?;
    let mut settings = brep_mcp_core::launch_state::Settings::load(path)?;
    settings.desktop = Some(policy);
    settings.save(path)
}


impl Embedded {
    /// `None` while the attach is in flight; then whether it succeeded.
    pub fn attached(&self) -> Option<Result<(), String>> {
        self.attached.get().cloned()
    }
}

/// The app's server, created on the first call (`record`: whether its
/// session keeps a call log; the first caller decides). Returns at once: the
/// attach completes on the server's runtime, because it is answered by the
/// app's frame, which may be the caller.
pub fn embedded(queue: &Arc<AutomationQueue>, adapter: &str, record: bool) -> Result<&'static Embedded, String> {
    EMBEDDED
        .get_or_init(|| {
            let policy = STATE_CONFIGURATION.get_or_init(|| {
                brep_mcp_core::launch_state::resolve(brep_mcp_core::launch_state::Host::Desktop, &Default::default())
            }).as_ref().map_err(Clone::clone)?;
            policy.prepare_named_parent().map_err(fail)?;
            // The state registry publishes only for a host; this window has
            // one now (`hit_rects`, `state_get` read it).
            crate::automation::registry::set_enabled(true);
            let runtime = tokio::runtime::Builder::new_multi_thread()
                .worker_threads(2)
                .thread_name("brep-mcp-worker")
                .enable_all()
                .build()
                .map_err(|e| format!("tokio runtime: {e}"))?;
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel::<HostRequest>();
            let q = queue.clone();
            runtime.spawn(async move {
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
            let info = HostInfo { backend: "window", adapter: adapter.to_string(), platform: std::env::consts::OS, width: 0.0, height: 0.0, ppp: 1.0 };
            let server = BrepServer::new(SESSION_ROOT.get_or_init(default_session_root).clone(), Backend::Attached { tx, info, platform: Arc::new(NativePlatform) });
            let server = match &policy.state_dir {
                Some(path) => server.with_state_dir(path).map_err(fail)?,
                None => server,
            };
            let attached = Arc::new(std::sync::OnceLock::new());
            let attached_signal = Arc::new(tokio::sync::Notify::new());
            let (s, done, signal) = (server.clone(), attached.clone(), attached_signal.clone());
            runtime.spawn(async move {
                let outcome = match s.attach(record).await {
                    Ok(info) => {
                        log::info!("brep-app: attached session {} ({}x{} @ {})", info.id, info.host.width, info.host.height, info.host.ppp);
                        Ok(())
                    }
                    Err(e) => Err(format!("cannot attach the server to the app: {e}")),
                };
                let _ = done.set(outcome);
                signal.notify_waiters();
            });
            Ok(Embedded { server, runtime, attached, attached_signal })
        })
        .as_ref()
        .map_err(Clone::clone)
}

/// Serve the app's server over a loopback listener. Called from the eframe
/// creation closure, once the app (and its queue) exists — or from the Info
/// window's button, any time later; returns as soon as the listener is
/// handed to the server. `adapter` names the GPU for the session info. At
/// most ONCE per process: a second call while the server runs is refused
/// with its address, so neither path can stack a second listener on the
/// first. The JavaScript editor may have created the server already; the
/// listener then serves that one.
pub fn start(queue: Arc<AutomationQueue>, adapter: String, launch: Launch) -> Result<(), String> {
    let root = SESSION_ROOT.get_or_init(|| launch.session_root.clone());
    if root != &launch.session_root {
        return Err(format!("the embedded session already uses {}; cannot relocate live artifacts to {}", root.display(), launch.session_root.display()));
    }
    {
        let mut status = STATUS.lock().unwrap_or_else(|p| p.into_inner());
        if let Status::Running { url, .. } | Status::Starting { url, .. } = &*status {
            return Err(format!("the MCP server is already running on {url}"));
        }
        *status = Status::Starting { url: launch.url(), session_root: launch.session_root.clone(), note: launch.note.clone() };
    }
    // The loopback credential (D3): read or minted before anything listens.
    let credential = brep_mcp_core::http::Credential::load_or_create().map_err(|e| {
        set_status(Status::Stopped { error: None });
        format!("the MCP token: {e}")
    })?;
    let embedded = embedded(&queue, &adapter, false).map_err(|e| {
        set_status(Status::Stopped { error: None });
        e
    })?;
    launch.listener.set_nonblocking(true).map_err(|e| {
        set_status(Status::Stopped { error: None });
        format!("listener: {e}")
    })?;
    let url = launch.url();
    let (server, attached, signal) = (embedded.server.clone(), embedded.attached.clone(), embedded.attached_signal.clone());
    let ready_status = Status::Running { url: url.clone(), session_root: launch.session_root.clone(), note: launch.note.clone() };
    let std_listener = launch.listener;
    embedded.runtime.spawn(async move {
        // The first `tools/list` must already carry the app's commands (the
        // HTTP transport may answer statelessly): serve once attached. The
        // attach is itself an app command, so "Running" means dispatch works.
        loop {
            // Registered before the check, so a set in between still wakes it.
            let notified = signal.notified();
            let mut notified = std::pin::pin!(notified);
            notified.as_mut().enable();
            match attached.get() {
                Some(Ok(())) => break,
                Some(Err(e)) => {
                    fail(e);
                    return;
                }
                None => notified.await,
            }
        }
        let listener = match tokio::net::TcpListener::from_std(std_listener) {
            Ok(l) => l,
            Err(e) => {
                fail(format!("listener: {e}"));
                return;
            }
        };
        set_status(ready_status);
        eprintln!("BREP MCP application host ready; serving on {url}");
        if let Err(e) = brep_mcp_core::http::serve_http(server, listener, credential).await {
            fail(format!("server on {url} stopped: {e}"));
        }
    });
    Ok(())
}

/// The JavaScript editor's runner: the app's server's code mode, the same
/// realm, limits and `brep.*` functions an agent's `run_script` gets.
pub struct EditorHost {
    queue: Arc<AutomationQueue>,
    adapter: String,
}

impl EditorHost {
    pub fn new(queue: Arc<AutomationQueue>, adapter: String) -> Self {
        EditorHost { queue, adapter }
    }

    fn service(&self) -> Result<(&'static Embedded, Arc<brep_mcp_core::cad::CadService>), String> {
        let e = embedded(&self.queue, &self.adapter, false)?;
        Ok((e, e.server.state.cad.clone()))
    }
}

impl crate::javascript::ScriptHost for EditorHost {
    fn ready(&self) -> Result<bool, String> {
        let (e, _) = self.service()?;
        e.attached().transpose().map(|r| r.is_some())
    }
    fn run(&self, source: &str) -> Result<String, String> {
        let (e, cad) = self.service()?;
        let _enter = e.runtime.enter();
        cad.run_local(source, "javascript")
    }
    fn result(&self, id: &str) -> Result<Option<Value>, String> {
        self.service()?.1.result_local(id)
    }
    fn halt(&self, id: &str) -> Result<(), String> {
        let (e, cad) = self.service()?;
        let _enter = e.runtime.enter();
        cad.halt_local(id)
    }
}

