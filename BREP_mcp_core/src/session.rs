//! A session: one app instance under one host, plus the session directory
//! (store, shots, exports, log) and the recorder.
use crate::host::{HostConfig, HostHandle, HostInfo};
use serde::Serialize;
use serde_json::{json, Value};
use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::time::Instant;

pub struct Session {
    pub id: String,
    pub dir: PathBuf,
    pub host: HostHandle,
    /// What the host was built from, so the session can be started again on the
    /// SAME store (`session_restart`). `None` for an attached window: that app
    /// is the user's and the server never built it.
    pub config: Option<HostConfig>,
    pub started: Instant,
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
        let id = format!("{}-{}", chrono_free_stamp(), std::process::id());
        let dir = root.join(&id);
        for sub in ["shots", "exports", "config"] {
            std::fs::create_dir_all(dir.join(sub)).map_err(|e| format!("session dir {}: {e}", dir.display()))?;
        }
        Ok(Self {
            id,
            dir,
            host,
            config,
            started: Instant::now(),
            shots: AtomicU64::new(0),
            recorder: Mutex::new(Recorder { on: record, steps: Vec::new() }),
        })
    }

    pub fn info(&self) -> SessionInfo {
        SessionInfo {
            id: self.id.clone(),
            dir: self.dir.display().to_string(),
            host: self.host.info.clone(),
            uptime_ms: self.started.elapsed().as_millis(),
            shots: self.shots.load(Ordering::Relaxed),
            recording: self.recorder.lock().unwrap().on,
        }
    }

    /// Reserve the next shot number and its path.
    pub fn next_shot(&self) -> (u64, PathBuf) {
        let n = self.shots.fetch_add(1, Ordering::Relaxed) + 1;
        (n, self.dir.join("shots").join(format!("{n:04}.png")))
    }

    pub fn record(&self, tool: &str, args: &Value, ok: bool, digest: Value) {
        let mut r = self.recorder.lock().unwrap();
        if !r.on {
            return;
        }
        let entry = json!({ "tool": tool, "args": args, "ok": ok, "digest": digest });
        r.steps.push(entry.clone());
        if let Ok(mut f) = std::fs::OpenOptions::new().create(true).append(true).open(self.dir.join("log.jsonl")) {
            use std::io::Write;
            let _ = writeln!(f, "{entry}");
        }
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

/// A sortable timestamp without pulling in a date crate.
fn chrono_free_stamp() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or(0);
    format!("{secs}")
}
