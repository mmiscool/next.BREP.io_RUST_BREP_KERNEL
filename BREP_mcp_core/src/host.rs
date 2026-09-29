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
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::{mpsc, oneshot};

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
    /// `session_stop` detaches and the app stays open.
    Attached { tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo },
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
            Backend::Attached { tx, info } => Some(HostHandle::attached(tx.clone(), info.clone())),
            Backend::Spawn { .. } => None,
        }
    }
}

pub struct HostHandle {
    pub tx: mpsc::UnboundedSender<HostRequest>,
    pub info: HostInfo,
    join: Option<std::thread::JoinHandle<()>>,
    /// Owned hosts are stopped with the session; an attached host is someone
    /// else's app and `stop` sends it nothing.
    owned: bool,
    next_id: std::sync::atomic::AtomicU64,
}

impl HostHandle {
    /// A host this handle owns: `stop` ends it and joins its thread.
    pub fn new(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo, join: std::thread::JoinHandle<()>) -> Self {
        Self { tx, info, join: Some(join), owned: true, next_id: std::sync::atomic::AtomicU64::new(1) }
    }

    /// A handle on a running app the server does not own.
    pub fn attached(tx: mpsc::UnboundedSender<HostRequest>, info: HostInfo) -> Self {
        Self { tx, info, join: None, owned: false, next_id: std::sync::atomic::AtomicU64::new(1) }
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

