//! A session: one app instance under one host, plus the session directory
//! (store, shots, exports, log) and the recorder. Where the directory and
//! the log ARE is the host's platform's business
//! (`crate::host::Platform`): a directory tree natively, a name and a blob
//! URL in a browser page.
use crate::host::{HostConfig, HostHandle, HostInfo, KeptShot};
use serde::Serialize;
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;

pub struct Session {
    pub id: String,
    pub dir: PathBuf,
    pub host: HostHandle,
    /// What the host was built from, so the session can be started again on the
    /// SAME store (`session_restart`). `None` for an attached window: that app
    /// is the user's and the server never built it.
    pub config: Option<HostConfig>,
    /// The platform clock's reading when the session started (`now_ms`).
    pub started_ms: f64,
    shots: AtomicU64,
    recorder: Mutex<Recorder>,
}

#[derive(Default)]
struct Recorder {
    on: bool,
    steps: Vec<Value>,
}

#[derive(Serialize)]
pub struct SessionInfo {
    pub id: String,
    pub dir: String,
    pub host: HostInfo,
    pub uptime_ms: u128,
    pub shots: u64,
    pub recording: bool,
}

impl Session {
    pub fn new(root: &std::path::Path, host: HostHandle, config: Option<HostConfig>, record: bool) -> Result<Self, String> {
        let id = host.platform.stamp();
        let dir = host.platform.session_dir(root, &id)?;
        let started_ms = host.platform.now_ms();
        Ok(Self {
            id,
            dir,
            host,
            config,
            started_ms,
            shots: AtomicU64::new(0),
            recorder: Mutex::new(Recorder { on: record, steps: Vec::new() }),
        })
    }

    pub fn info(&self) -> SessionInfo {
        SessionInfo {
            id: self.id.clone(),
            dir: self.dir.display().to_string(),
            host: self.host.info.clone(),
            uptime_ms: (self.host.platform.now_ms() - self.started_ms).max(0.0) as u128,
            shots: self.shots.load(Ordering::Relaxed),
            recording: self.recorder.lock().unwrap().on,
        }
    }

    /// Keep the next shot: reserve its number, hand the PNG (and the
    /// manifest, when the capture made one) to the platform, and answer with
    /// the number and where it went.
    pub fn keep_shot(&self, png: &[u8], manifest: Option<&Value>) -> Result<(u64, KeptShot), String> {
        let n = self.shots.fetch_add(1, Ordering::Relaxed) + 1;
        let kept = self.host.platform.keep_shot(&self.dir, n, png, manifest)?;
        Ok((n, kept))
    }

    pub fn record(&self, tool: &str, args: &Value, ok: bool, digest: Value) {
        let mut r = self.recorder.lock().unwrap();
        if !r.on {
            return;
        }
        let entry = json!({ "tool": tool, "args": args, "ok": ok, "digest": digest });
        r.steps.push(entry.clone());
        self.host.platform.append_log(&self.dir, &entry.to_string());
    }

    pub fn set_recording(&self, on: bool) {
        self.recorder.lock().unwrap().on = on;
    }

    /// The recorded calls as a test-mcp script (expectations left empty).
    pub fn script(&self, name: &str, since: usize) -> Value {
        let r = self.recorder.lock().unwrap();
        let steps: Vec<Value> = r
            .steps
            .iter()
            .skip(since)
            .filter(|s| !matches!(s["tool"].as_str(), Some("session_script") | Some("session_info") | Some("session_record")))
            .map(|s| json!({ "tool": s["tool"], "args": s["args"] }))
            .collect();
        json!({
            "name": name,
            "note": "recorded by session_script; add `expect` entries before committing",
            "backend": self.host.info.backend,
            "size": [self.host.info.width, self.host.info.height],
            "ppp": self.host.info.ppp,
            "steps": steps,
        })
    }
}

