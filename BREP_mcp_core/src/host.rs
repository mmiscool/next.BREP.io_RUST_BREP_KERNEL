//! The host protocol. A host owns an app instance and answers commands; the
//! server talks to it through [`HostHandle`] — a tokio channel in, a oneshot
//! per reply out — so async tool handlers never block.
//!
//! Two kinds of host exist, and [`Backend`] is how a server is told which one
//! it has: a *spawning* backend builds an app per session (BREP_mcp's headless
//! harness on its own thread), an *attached* backend is a running app the
//! server lives inside (`brep-app --mcp`), which every session shares and no
//! session may stop.
//!
//! The wire types mirror `brep_app::automation::command::{Envelope, Reply}`
//! field for field. They are repeated here rather than imported because this
//! crate must not depend on the app (the app depends on it); a host converts
//! with [`Reply::from_app`], which reads the app's own serialisation.
//!
//! A host also says where the server's tools touch the world OUTSIDE the app
//! — the clock, a pause, files, the pictures a session keeps — through the
//! [`Platform`] it hands a [`HostHandle`]. The tools are written against that
//! trait and nothing else, which is what lets one tool set run under a native
//! process (the [`NativePlatform`], feature `server`) and inside a browser
//! page, where the same calls are a `setTimeout`, the page's document store
//! and a blob URL.
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::future::Future;
use std::path::PathBuf;
use std::pin::Pin;
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot};

/// A future a [`Platform`] answers with. `Send`, like every tool future: the
/// native server runs them on a multi-threaded runtime.
pub type PlatformFuture<'a, T> = Pin<Box<dyn Future<Output = T> + Send + 'a>>;

/// What a session keeps of one screenshot, by number: where the full
/// picture went (a path, a URL) and where its manifest went when it had one.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KeptShot {
    pub path: String,
    pub manifest: Option<String>,
}

/// The world outside the app, as the tools see it.
///
/// Every method is what a tool needs and nothing about HOW the platform does
/// it: `document_open` reads text at a path, a screenshot is kept and answers
/// with where it went, a wait sleeps. A path is whatever the platform calls
/// one — a filesystem path natively, a name in the page's document store in
/// the browser — and the tool docs say so.
pub trait Platform: Send + Sync {
    /// Milliseconds on a clock that only ever moves forward; only
    /// differences are read.
    fn now_ms(&self) -> f64;

    /// Pause for `ms` without holding the thread.
    fn sleep(&self, ms: u64) -> PlatformFuture<'_, ()>;

    /// Run `task` to completion whether or not whoever spawned it keeps
    /// waiting — a capture's cleanup must finish after its caller is gone.
    fn spawn(&self, task: Pin<Box<dyn Future<Output = ()> + Send + 'static>>);

    /// A stamp unique enough to name a session: the native platform's is
    /// `<unix seconds>-<pid>`.
    fn stamp(&self) -> String;

    /// Make the directory of session `id` under `root` (with its `shots`,
    /// `exports` and `config` subdirectories, where directories exist) and
    /// return it.
    fn session_dir(&self, root: &std::path::Path, id: &str) -> Result<PathBuf, String>;

    /// Append `line` to the session's call log under `dir`. Best effort.
    fn append_log(&self, dir: &std::path::Path, line: &str);

    /// The text at `path`.
    fn read_text(&self, path: String) -> PlatformFuture<'_, Result<String, String>>;

    /// The bytes at `path`.
    fn read_bytes(&self, path: String) -> PlatformFuture<'_, Result<Vec<u8>, String>>;

    /// Write `bytes` at `path` — refusing one that exists unless `overwrite`
    /// — and answer with where it went.
    fn write_file(&self, path: String, bytes: Vec<u8>, overwrite: bool) -> PlatformFuture<'_, Result<String, String>>;

    /// Keep shot number `n` of the session under `dir`: the full-resolution
    /// PNG, and its manifest when the capture made one.
    fn keep_shot(&self, dir: &std::path::Path, n: u64, png: &[u8], manifest: Option<&Value>) -> Result<KeptShot, String>;
}

/// The platform of a native process: the filesystem, `Instant`, tokio's
/// timers and tasks. What every server host had before the seam existed.
#[cfg(feature = "server")]
#[derive(Debug, Default, Clone, Copy)]
pub struct NativePlatform;

#[cfg(feature = "server")]
impl NativePlatform {
    fn epoch() -> &'static std::time::Instant {
        static EPOCH: std::sync::OnceLock<std::time::Instant> = std::sync::OnceLock::new();
        EPOCH.get_or_init(std::time::Instant::now)
    }
}

#[cfg(feature = "server")]
impl Platform for NativePlatform {
    fn now_ms(&self) -> f64 {
        Self::epoch().elapsed().as_secs_f64() * 1000.0
    }

    fn sleep(&self, ms: u64) -> PlatformFuture<'_, ()> {
        Box::pin(tokio::time::sleep(std::time::Duration::from_millis(ms)))
    }

    fn spawn(&self, task: Pin<Box<dyn Future<Output = ()> + Send + 'static>>) {
        tokio::spawn(task);
    }

    fn stamp(&self) -> String {
        let secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        format!("{secs}-{}", std::process::id())
    }

    fn session_dir(&self, root: &std::path::Path, id: &str) -> Result<PathBuf, String> {
        let dir = root.join(id);
        for sub in ["shots", "exports", "config"] {
            std::fs::create_dir_all(dir.join(sub)).map_err(|e| format!("session dir {}: {e}", dir.display()))?;
        }
        Ok(dir)
    }

    fn append_log(&self, dir: &std::path::Path, line: &str) {
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(dir.join("log.jsonl")) {
            use std::io::Write;
            let _ = writeln!(f, "{line}");
        }
    }

    fn read_text(&self, path: String) -> PlatformFuture<'_, Result<String, String>> {
        Box::pin(std::future::ready(std::fs::read_to_string(&path).map_err(|e| format!("{path}: {e}"))))
    }

    fn read_bytes(&self, path: String) -> PlatformFuture<'_, Result<Vec<u8>, String>> {
        Box::pin(std::future::ready(std::fs::read(&path).map_err(|e| format!("{path}: {e}"))))
    }

    fn write_file(&self, path: String, bytes: Vec<u8>, overwrite: bool) -> PlatformFuture<'_, Result<String, String>> {
        let result = (|| {
            if !overwrite && std::path::Path::new(&path).exists() {
                return Err(format!("{path} exists; pass overwrite: true"));
            }
            if let Some(parent) = std::path::Path::new(&path).parent() {
                let _ = std::fs::create_dir_all(parent);
            }
            std::fs::write(&path, &bytes).map_err(|e| format!("{path}: {e}"))?;
            Ok(path.clone())
        })();
        Box::pin(std::future::ready(result))
    }

    fn keep_shot(&self, dir: &std::path::Path, n: u64, png: &[u8], manifest: Option<&Value>) -> Result<KeptShot, String> {
        let path = dir.join("shots").join(format!("{n:04}.png"));
        std::fs::write(&path, png).map_err(|e| format!("{}: {e}", path.display()))?;
        let manifest = match manifest {
            Some(m) => {
                let mpath = path.with_extension("json");
                std::fs::write(&mpath, serde_json::to_vec_pretty(m).map_err(|e| e.to_string())?)
                    .map_err(|e| format!("{}: {e}", mpath.display()))?;
                Some(mpath.display().to_string())
            }
            None => None,
        };
        Ok(KeptShot { path: path.display().to_string(), manifest })
    }
}

/// One command on its way to the app.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Envelope {
    pub id: u64,
    pub cmd: String,
    #[serde(default)]
    pub args: Value,
}

/// The app's answer. `notices` are the app's `Notice` records as JSON
/// (`{kind, frame, text}`); `blob` is the binary payload the app carries
/// beside the JSON (a PNG), never serialised.
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Reply {
    pub id: u64,
    pub frame: u64,
    pub ok: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub notices: Vec<Value>,
    #[serde(skip)]
    pub blob: Option<Vec<u8>>,
}

impl Reply {
    pub fn err(id: u64, frame: u64, error: impl Into<String>) -> Self {
        Self { id, frame, ok: false, result: None, error: Some(error.into()), notices: Vec::new(), blob: None }
    }

    /// From the app's serialised reply (`serde_json::to_value(&reply)`) plus
    /// the blob it carried beside it.
    pub fn from_app(value: Value, blob: Option<Vec<u8>>) -> Result<Self, String> {
        let mut reply: Reply = serde_json::from_value(value).map_err(|e| format!("app reply: {e}"))?;
        reply.blob = blob;
        Ok(reply)
    }
}

/// What the server asks a host to do.
pub enum HostRequest {
    /// Run one command through the app's automation queue.
    Command { envelope: Envelope, reply: oneshot::Sender<Reply> },
    /// Capture the frame (headless renders directly; the window host turns
    /// this into the app's `screenshot` command).
    Screenshot { region: Value, reply: oneshot::Sender<Result<(Value, Vec<u8>), String>> },
    /// Set (or clear) the DOCS ANNOTATION the host paints over the app's own
    /// frame: a caption band and a ring around named widgets, for the
    /// walkthrough GIFs.
    ///
    /// It is a host request and not an app command for the same reason
    /// [`HostRequest::Screenshot`] is one under the headless backend: the app
    /// does not do the work. Nothing about the document changes, no widget is
    /// published, and the app's own registries are untouched — the overlay is
    /// painted on top of whatever the app drew, by the harness that owns the
    /// egui context. A host that has no such overlay answers with an error
    /// rather than silently capturing an unannotated frame.
    Annotate { spec: Value, reply: oneshot::Sender<Result<Value, String>> },
    Stop,
}

#[derive(Debug, Clone, serde::Serialize)]
pub struct HostInfo {
    pub backend: &'static str,
    pub adapter: String,
    pub platform: &'static str,
    pub width: f32,
    pub height: f32,
    pub ppp: f32,
}

/// What a spawning backend builds an app from (`session_start`'s arguments).
#[derive(Debug, Clone)]
pub struct HostConfig {
    pub width: f32,
    pub height: f32,
    pub ppp: f32,
    pub seed: bool,
    /// The session's private store directory.
    pub store_dir: PathBuf,
}

pub type SpawnFn = Arc<dyn Fn(HostConfig) -> Result<HostHandle, String> + Send + Sync>;

/// Where a server's sessions run.
#[derive(Clone)]
pub enum Backend {
    /// The server builds an app per session through `spawn`, on the host's
    /// own thread, and stops it at `session_stop`.
    Spawn { name: &'static str, spawn: SpawnFn },
    /// The server lives inside a running app. Every session attaches to it;
    /// `session_stop` detaches and the app stays open. `platform` is where
    /// that app runs: the process's filesystem and clock, or the page's.
    Attached { tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo, platform: Arc<dyn Platform> },
}

impl Backend {
    pub fn name(&self) -> &'static str {
        match self {
            Backend::Spawn { name, .. } => name,
            Backend::Attached { info, .. } => info.backend,
        }
    }

    pub fn is_attached(&self) -> bool {
        matches!(self, Backend::Attached { .. })
    }

    /// A handle on the attached app (None for a spawning backend).
    pub fn attached_handle(&self) -> Option<HostHandle> {
        match self {
            Backend::Attached { tx, info, platform } => Some(HostHandle::attached_on(tx.clone(), info.clone(), platform.clone())),
            Backend::Spawn { .. } => None,
        }
    }
}

pub struct HostHandle {
    pub tx: mpsc::UnboundedSender<HostRequest>,
    pub info: HostInfo,
    /// The world outside the app: see [`Platform`].
    pub platform: Arc<dyn Platform>,
    join: Option<std::thread::JoinHandle<()>>,
    /// Owned hosts are stopped with the session; an attached host is someone
    /// else's app and `stop` sends it nothing.
    owned: bool,
    next_id: std::sync::atomic::AtomicU64,
}

impl HostHandle {
    /// A host this handle owns: `stop` ends it and joins its thread. On the
    /// native platform.
    #[cfg(feature = "server")]
    pub fn new(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo, join: std::thread::JoinHandle<()>) -> Self {
        Self::new_on(tx, info, join, Arc::new(NativePlatform))
    }

    /// A host this handle owns, on `platform`.
    pub fn new_on(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo, join: std::thread::JoinHandle<()>, platform: Arc<dyn Platform>) -> Self {
        Self { tx, info, platform, join: Some(join), owned: true, next_id: std::sync::atomic::AtomicU64::new(1) }
    }

    /// A handle on a running app the server does not own. On the native
    /// platform.
    #[cfg(feature = "server")]
    pub fn attached(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo) -> Self {
        Self::attached_on(tx, info, Arc::new(NativePlatform))
    }

    /// A handle on a running app the server does not own, on `platform`.
    pub fn attached_on(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo, platform: Arc<dyn Platform>) -> Self {
        Self { tx, info, platform, join: None, owned: false, next_id: std::sync::atomic::AtomicU64::new(1) }
    }

    pub fn is_owned(&self) -> bool {
        self.owned
    }

    /// Run a command and await its reply.
    pub async fn call(&self, cmd: &str, args: Value) -> Result<Reply, String> {
        let id = self.next_id.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        let (tx, rx) = oneshot::channel();
        self.tx
            .send(HostRequest::Command { envelope: Envelope { id, cmd: cmd.to_string(), args }, reply: tx })
            .map_err(|_| "the app host has stopped".to_string())?;
        rx.await.map_err(|_| "the app host dropped the reply (did it panic?)".to_string())
    }

    /// Run a command; an `ok: false` reply becomes `Err(error)`.
    pub async fn call_ok(&self, cmd: &str, args: Value) -> Result<Reply, String> {
        let r = self.call(cmd, args).await?;
        if r.ok {
            Ok(r)
        } else {
            Err(r.error.unwrap_or_else(|| "command failed".into()))
        }
    }

    pub async fn screenshot(&self, region: Value) -> Result<(Value, Vec<u8>), String> {
        let (tx, rx) = oneshot::channel();
        self.tx
            .send(HostRequest::Screenshot { region, reply: tx })
            .map_err(|_| "the app host has stopped".to_string())?;
        rx.await.map_err(|_| "the app host dropped the screenshot reply".to_string())?
    }

    /// Set or clear the host's docs annotation overlay (see
    /// [`HostRequest::Annotate`]). `spec` is `{caption?, title?, highlight?,
    /// step?}`; an empty object clears it.
    pub async fn annotate(&self, spec: Value) -> Result<Value, String> {
        let (tx, rx) = oneshot::channel();
        self.tx
            .send(HostRequest::Annotate { spec, reply: tx })
            .map_err(|_| "the app host has stopped".to_string())?;
        rx.await.map_err(|_| "the app host dropped the annotate reply".to_string())?
    }

    /// Stop an owned host and wait for its thread; a no-op for an attached one.
    pub fn stop(mut self) {
        if !self.owned {
            return;
        }
        let _ = self.tx.send(HostRequest::Stop);
        if let Some(j) = self.join.take() {
            let _ = j.join();
        }
    }
}

