//! Cancellation and progress at bounded work chunks.
//!
//! Long-running generation runs off the UI thread. It polls a [`CancelToken`]
//! and publishes [`Progress`] through a [`ProgressSink`] only at chunk
//! boundaries counted in WORK (samples, fibers, passes), never in seconds:
//! there is no clock here, so the same code runs identically on native and
//! `wasm32-unknown-unknown`.

use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

/// Shared cancellation flag. Clones observe the same flag.
#[derive(Clone, Debug, Default)]
pub struct CancelToken {
    flag: Arc<AtomicBool>,
}

impl CancelToken {
    pub fn new() -> Self {
        Self::default()
    }

    /// Request cancellation; every clone reads it at its next checkpoint.
    pub fn cancel(&self) {
        self.flag.store(true, Ordering::SeqCst);
    }

    pub fn is_cancelled(&self) -> bool {
        self.flag.load(Ordering::SeqCst)
    }

    /// `Err(Cancelled)` once cancellation was requested.
    pub fn check(&self) -> Result<(), Cancelled> {
        if self.is_cancelled() {
            Err(Cancelled)
        } else {
            Ok(())
        }
    }
}

/// The generation was cancelled at a chunk boundary. The caller keeps the
/// previous accepted result; nothing partial is published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cancelled;

impl std::fmt::Display for Cancelled {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "cancelled")
    }
}

impl std::error::Error for Cancelled {}

/// A progress event, published at stage and chunk boundaries.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Progress {
    /// Stable stage identifier, e.g. `index-build`, `drop-cutter`, `linking`.
    pub phase: String,
    /// Human-readable one-liner.
    pub message: String,
    /// Optional detail; a fallback path names itself here.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
    /// Work units done so far in this phase.
    pub current: u64,
    /// Total work units in this phase (`0` when unknown).
    pub total: u64,
    /// The operation this progress belongs to, when there is one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub operation_id: Option<String>,
}

/// Receives progress events. Implemented by the worker boundary.
pub trait ProgressSink {
    fn progress(&mut self, progress: &Progress);
}

/// A sink that discards everything.
#[derive(Clone, Copy, Debug, Default)]
pub struct NullSink;

impl ProgressSink for NullSink {
    fn progress(&mut self, _progress: &Progress) {}
}

/// A sink that records every event, for tests and for batch logs.
#[derive(Clone, Debug, Default)]
pub struct RecordingSink {
    pub events: Vec<Progress>,
}

impl ProgressSink for RecordingSink {
    fn progress(&mut self, progress: &Progress) {
        self.events.push(progress.clone());
    }
}

/// Counts work units and tells the caller when a chunk boundary was crossed.
///
/// `chunk` is the number of work units between two cancellation checks and
/// progress publications. The default suits per-sample loops; index builds
/// use a larger chunk, per-pass loops a smaller one.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkBudget {
    chunk: u64,
    used: u64,
    next_boundary: u64,
}

impl Default for WorkBudget {
    fn default() -> Self {
        Self::new(Self::DEFAULT_CHUNK)
    }
}

impl WorkBudget {
    pub const DEFAULT_CHUNK: u64 = 1024;

    /// A budget that reports a boundary every `chunk` units (`0` is treated as 1).
    pub fn new(chunk: u64) -> Self {
        let chunk = chunk.max(1);
        Self { chunk, used: 0, next_boundary: chunk }
    }

    pub fn chunk(&self) -> u64 {
        self.chunk
    }

    /// Work units consumed since construction.
    pub fn used(&self) -> u64 {
        self.used
    }

    /// Record `units` of work. Returns `true` when a chunk boundary was crossed,
    /// i.e. the caller should checkpoint now.
    pub fn step(&mut self, units: u64) -> bool {
        self.used = self.used.saturating_add(units);
        if self.used >= self.next_boundary {
            let crossed = (self.used - self.next_boundary) / self.chunk + 1;
            self.next_boundary = self.next_boundary.saturating_add(crossed * self.chunk);
            true
        } else {
            false
        }
    }
}

/// Everything an algorithm needs to cooperate with the runner: the token,
/// the sink and a budget. One per generation call; algorithms borrow it.
pub struct Runner<'a> {
    token: CancelToken,
    sink: &'a mut dyn ProgressSink,
    budget: WorkBudget,
    operation_id: Option<String>,
}

impl<'a> Runner<'a> {
    pub fn new(token: CancelToken, sink: &'a mut dyn ProgressSink, budget: WorkBudget) -> Self {
        Self { token, sink, budget, operation_id: None }
    }

    /// A runner for synchronous callers: never cancels, discards progress.
    pub fn detached(sink: &'a mut dyn ProgressSink) -> Self {
        Self::new(CancelToken::new(), sink, WorkBudget::default())
    }

    pub fn with_operation(mut self, operation_id: impl Into<String>) -> Self {
        self.operation_id = Some(operation_id.into());
        self
    }

    pub fn token(&self) -> &CancelToken {
        &self.token
    }

    pub fn budget(&self) -> &WorkBudget {
        &self.budget
    }

    /// Publish a stage boundary (always) and check cancellation.
    pub fn stage(&mut self, phase: &str, message: &str, current: u64, total: u64) -> Result<(), Cancelled> {
        self.publish(phase, message, None, current, total);
        self.token.check()
    }

    /// Publish a stage boundary carrying a detail line (e.g. the fallback used).
    pub fn stage_detail(
        &mut self,
        phase: &str,
        message: &str,
        detail: &str,
        current: u64,
        total: u64,
    ) -> Result<(), Cancelled> {
        self.publish(phase, message, Some(detail.to_string()), current, total);
        self.token.check()
    }

    /// Record `units` of work; at a chunk boundary publish progress and check
    /// cancellation. Cheap between boundaries: one add and one compare.
    pub fn work(&mut self, units: u64, phase: &str, current: u64, total: u64) -> Result<(), Cancelled> {
        if self.budget.step(units) {
            self.publish(phase, phase, None, current, total);
            self.token.check()?;
        }
        Ok(())
    }

    fn publish(&mut self, phase: &str, message: &str, detail: Option<String>, current: u64, total: u64) {
        let progress = Progress {
            phase: phase.to_string(),
            message: message.to_string(),
            detail,
            current,
            total,
            operation_id: self.operation_id.clone(),
        };
        self.sink.progress(&progress);
    }
}

