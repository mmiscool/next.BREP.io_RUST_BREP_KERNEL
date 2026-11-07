//! Bounded durable execution evidence. Admission is synced before returning.
//! This primitive does not launch workers or automatically replay mutations.
//!
//! Durability per frame: an admission and a terminal outcome are synced
//! before their call returns. A host call's intent ([`Journal::start_call`])
//! is synced before dispatch, by the caller, through the [`IntentSync`] it is
//! handed, so the sync runs after the caller has released whatever lock
//! guards the journal. A host call's completion is an observation and is not
//! synced on its own: it reaches the disk with the next synced frame (the
//! next intent or the outcome). Lost to a power cut, it reads back as an
//! unacknowledged call, the conservative answer. One sync per host call, not
//! two.
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};
use std::collections::{BTreeMap, HashMap};
use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom, Write};
use std::path::Path;
use std::sync::Arc;

const MAX_RECORD: usize = 1 << 20;
const MAX_BYTES: u64 = 128 << 20;
const MAX_ADMISSIONS: usize = 4096;
const CALL_COMPLETION_RESERVE: u64 = 4096;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Admission {
    pub binding: String,
    pub submission: String,
    pub execution_id: String,
    pub scope: String,
    pub actor: String,
    pub status: Value,
    #[serde(default)]
    pub bridge_evidence: bool,
}

/// Metadata only: never persist call arguments, source, or response bodies.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CallEvidence {
    pub call_id: u64,
    pub method: String,
    pub completion: Option<CallCompletion>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CallCompletion {
    pub reply_received: bool,
    pub ok: Option<bool>,
    #[serde(default)]
    pub operation: Option<OperationObservation>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OperationObservation {
    pub operation_id: String,
    pub state: Option<String>,
    pub effects: Option<String>,
}

#[derive(Clone, Debug)]
pub struct Record {
    pub admission: Admission,
    pub outcome: Option<Value>,
    pub calls: BTreeMap<u64, CallEvidence>,
}

impl Record {
    pub fn recovery_evidence(&self) -> Value {
        json!({
            "hostCallRecordingEnabled":self.admission.bridge_evidence,
            "operationsEvidenceComplete":false,"automaticReplay":false,
            "calls":self.calls.values().collect::<Vec<_>>(),
            "unacknowledgedCallIds":self.calls.values().filter(|c| !c.completion.as_ref().is_some_and(|v| v.reply_received)).map(|c| c.call_id).collect::<Vec<_>>(),
            "observationsMayBeStale":true
        })
    }
}

#[derive(Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
enum Entry {
    Admit { admission: Admission },
    Finish { execution_id: String, outcome: Value },
    CallStarted { execution_id: String, call_id: u64, method: String },
    CallFinished { execution_id: String, call_id: u64, completion: CallCompletion },
}

#[derive(Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct Frame {
    version: u32,
    sha256: String,
    payload: Value,
}

pub struct Journal {
    file: File, // the exclusive OS lock lives as long as this file
    /// A second handle on the same file, for syncs made outside the lock
    /// that guards this journal ([`IntentSync`]).
    syncer: Arc<File>,
    bytes: u64,
    poisoned: bool,
    records: HashMap<String, Record>,
    bindings: HashMap<String, String>,
}

impl Journal {
    pub fn open(directory: &Path) -> Result<Self, String> {
        let parent = directory.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
        if !parent.is_dir() { return Err("create the persistent state directory's parent before opening the journal".into()); }
        crate::host::private_dir(directory)?;
        #[cfg(unix)]
        File::open(parent).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?;
        let path = directory.join("executions.jsonl");
        if std::fs::symlink_metadata(&path).is_ok_and(|m| !m.is_file() || m.file_type().is_symlink()) {
            return Err("execution journal must be a regular file, not a symlink".into());
        }
        let mut options = OpenOptions::new();
        options.read(true).append(true).create(true);
        #[cfg(unix)] {
            use std::os::unix::fs::OpenOptionsExt;
            options.mode(0o600);
        }
        let file = options.open(&path).map_err(|e| e.to_string())?;
        file.try_lock().map_err(|_| "persistent state directory is already in use or cannot be locked".to_string())?;
        #[cfg(unix)] {
            use std::os::unix::fs::PermissionsExt;
            file.set_permissions(std::fs::Permissions::from_mode(0o600)).map_err(|e| e.to_string())?;
        }
        file.sync_all().map_err(|e| e.to_string())?;
        #[cfg(unix)]
        File::open(directory).and_then(|f| f.sync_all()).map_err(|e| e.to_string())?;
        let bytes = file.metadata().map_err(|e| e.to_string())?.len();
        if bytes > MAX_BYTES { return Err("execution journal exceeds its storage budget".into()); }
        let syncer = Arc::new(file.try_clone().map_err(|e| e.to_string())?);
        let mut journal = Self { file, syncer, bytes, poisoned: false, records: HashMap::new(), bindings: HashMap::new() };
        let mut reader = BufReader::new(journal.file.try_clone().map_err(|e| e.to_string())?);
        reader.seek(SeekFrom::Start(0)).map_err(|e| e.to_string())?;
        loop {
            let mut line = Vec::new();
            let count = Read::by_ref(&mut reader).take((MAX_RECORD + 1) as u64).read_until(b'\n', &mut line).map_err(|e| e.to_string())?;
            if count == 0 { break; }
            if count > MAX_RECORD || line.last() != Some(&b'\n') {
                return Err("execution journal has an oversized or truncated record; reconciliation required".into());
            }
            let frame: Frame = serde_json::from_slice(&line).map_err(|_| "invalid execution journal frame")?;
            let canonical = super::canon::canonical(&frame.payload)?;
            if frame.version != 1 || super::canon::sha256_hex(canonical.as_bytes()) != frame.sha256 {
                return Err("execution journal checksum/version mismatch; reconciliation required".into());
            }
            let entry: Entry = serde_json::from_value(frame.payload).map_err(|_| "invalid execution journal entry")?;
            journal.validate(&entry)?;
            journal.apply(entry);
        }
        Ok(journal)
    }

    /// Only digests of caller-supplied keys/programs need to reach storage.
    pub fn binding(scope: &str, actor: &str, namespace: &str, key: &str) -> String {
        super::canon::sha256_hex(json!([scope, actor, namespace, key]).to_string().as_bytes())
    }

    pub fn by_binding(&self, binding: &str, submission: &str) -> Result<Option<&Record>, String> {
        let Some(id) = self.bindings.get(binding) else { return Ok(None) };
        let record = &self.records[id];
        if record.admission.submission != submission { return Err("idempotency_conflict".into()); }
        Ok(Some(record))
    }

    pub fn by_id(&self, id: &str, scope: &str, actor: &str) -> Option<&Record> {
        self.records.get(id).filter(|r| r.admission.scope == scope && r.admission.actor == actor)
    }

    pub fn unfinished(&self) -> Vec<Admission> {
        self.records.values().filter(|r| r.outcome.is_none()).map(|r| r.admission.clone()).collect()
    }

    pub fn admit(&mut self, admission: Admission) -> Result<(), String> {
        self.append(Entry::Admit { admission }, Durability::Synced)
    }

    pub fn finish(&mut self, execution_id: &str, outcome: Value) -> Result<(), String> {
        self.append(Entry::Finish { execution_id: execution_id.into(), outcome }, Durability::Synced)
    }

    /// Record intent before dispatch; rejection means the caller must not
    /// dispatch. The frame is written, not yet synced: the caller syncs it
    /// with the returned [`IntentSync`] (outside its lock) and must not
    /// dispatch unless that succeeds, and [`Self::poison`] the journal if it
    /// fails.
    pub fn start_call(&mut self, execution_id: &str, call_id: u64, method: &str) -> Result<IntentSync, String> {
        self.append(Entry::CallStarted { execution_id: execution_id.into(), call_id, method: method.into() }, Durability::Caller)?;
        Ok(IntentSync(self.syncer.clone()))
    }

    /// A reply is an observation, not proof of downstream transaction
    /// completion. Written, not synced: the next synced frame carries it.
    pub fn finish_call(&mut self, execution_id: &str, call_id: u64, completion: CallCompletion) -> Result<(), String> {
        self.append(Entry::CallFinished { execution_id: execution_id.into(), call_id, completion }, Durability::Deferred)
    }

    /// A sync made through an [`IntentSync`] failed: what reached the disk is
    /// unknown, so no further frame is accepted.
    pub fn poison(&mut self) {
        self.poisoned = true;
    }

    fn validate(&self, entry: &Entry) -> Result<(), String> {
        match entry {
            Entry::Admit { admission: a } => {
                if self.records.len() >= MAX_ADMISSIONS { return Err("execution journal admission capacity reached".into()); }
                if self.bindings.contains_key(&a.binding) || self.records.contains_key(&a.execution_id) {
                    return Err("duplicate durable admission".into());
                }
                let digest = |s: &str| s.len() == 64 && s.bytes().all(|b| b.is_ascii_hexdigit());
                if a.execution_id.len() > 128 || a.status["executionId"] != a.execution_id || !digest(&a.binding) || !digest(&a.submission) {
                    return Err("invalid durable admission identity".into());
                }
            }
            Entry::CallStarted { execution_id, call_id, method } => {
                let record = self.records.get(execution_id).ok_or("call without admission")?;
                if record.outcome.is_some() || record.calls.len() >= 1024 || record.calls.contains_key(call_id) {
                    return Err("call admission refused: terminal execution, duplicate call, or call capacity reached".into());
                }
                if method.is_empty() || method.len() > 128 || !method.bytes().all(|b| b.is_ascii_alphanumeric() || b == b'.' || b == b'_') {
                    return Err("invalid call method metadata".into());
                }
            }
            Entry::CallFinished { execution_id, call_id, completion } => {
                let call = self.records.get(execution_id).and_then(|r| r.calls.get(call_id)).ok_or("completion without call admission")?;
                if let Some(op) = &completion.operation {
                    if (!op.operation_id.is_ascii() || super::wire::handle_shape("op", &op.operation_id) == super::wire::HandleShape::Malformed)
                        || op.state.as_deref().is_some_and(|s| !matches!(s, "queued" | "running" | "evaluating" | "committing" | "succeeded" | "failed" | "cancelled"))
                        || op.effects.as_deref().is_some_and(|s| !matches!(s, "none" | "committed" | "unknown" | "partial")) {
                        return Err("invalid operation observation".into());
                    }
                }
                if call.completion.is_some() || (!completion.reply_received && (completion.ok.is_some() || completion.operation.is_some())) {
                    return Err("invalid or duplicate call completion".into());
                }
            }
            Entry::Finish { execution_id, outcome } => {
                let record = self.records.get(execution_id).ok_or("terminal record without admission")?;
                if record.outcome.is_some() { return Err("terminal execution outcome is immutable".into()); }
                if outcome["executionId"] != *execution_id || !matches!(outcome["state"].as_str(), Some("succeeded" | "failed" | "cancelled")) {
                    return Err("invalid terminal execution outcome".into());
                }
            }
        }
        Ok(())
    }

    fn append(&mut self, entry: Entry, durability: Durability) -> Result<(), String> {
        if self.poisoned { return Err("execution journal unavailable after a write failure; reconciliation required".into()); }
        self.validate(&entry)?;
        let payload = serde_json::to_value(&entry).map_err(|e| e.to_string())?;
        let sha256 = super::canon::sha256_hex(super::canon::canonical(&payload)?.as_bytes());
        let mut bytes = serde_json::to_vec(&Frame { version: 1, sha256, payload }).map_err(|e| e.to_string())?;
        bytes.push(b'\n');
        if bytes.len() > MAX_RECORD || self.bytes + bytes.len() as u64 > MAX_BYTES {
            return Err("execution journal storage budget reached; existing admission evidence is preserved".into());
        }
        // Reserve one maximum-sized terminal frame per unfinished execution
        // and one small completion frame per outstanding call. Every append
        // respects both reservations, including late replies after cancellation.
        let mut unfinished = self.records.values().filter(|r| r.outcome.is_none()).count() as u64;
        let mut pending = self.records.values().flat_map(|r| r.calls.values()).filter(|c| c.completion.is_none()).count() as u64;
        match &entry {
            Entry::Admit { .. } => unfinished += 1,
            Entry::Finish { .. } => unfinished -= 1,
            Entry::CallStarted { .. } => pending += 1,
            Entry::CallFinished { .. } => pending -= 1,
        }
        if self.bytes + bytes.len() as u64 + unfinished * MAX_RECORD as u64 + pending * CALL_COMPLETION_RESERVE > MAX_BYTES {
            return Err("execution journal has no unreserved outcome capacity".into());
        }
        // `sync_data`: an append moves the length, which it syncs; the
        // timestamps it skips are not evidence.
        let written = self.file.write_all(&bytes).and_then(|()| match durability {
            Durability::Synced => self.file.sync_data(),
            Durability::Caller | Durability::Deferred => Ok(()),
        });
        if let Err(e) = written {
            self.poisoned = true;
            return Err(format!("execution journal write failed; reconciliation required: {e}"));
        }
        self.bytes += bytes.len() as u64;
        self.apply(entry);
        Ok(())
    }

    fn apply(&mut self, entry: Entry) {
        match entry {
            Entry::Admit { admission } => {
                self.bindings.insert(admission.binding.clone(), admission.execution_id.clone());
                self.records.insert(admission.execution_id.clone(), Record { admission, outcome: None, calls: BTreeMap::new() });
            }
            Entry::CallStarted { execution_id, call_id, method } => {
                self.records.get_mut(&execution_id).unwrap().calls.insert(call_id, CallEvidence { call_id, method, completion: None });
            }
            Entry::CallFinished { execution_id, call_id, completion } => {
                self.records.get_mut(&execution_id).unwrap().calls.get_mut(&call_id).unwrap().completion = Some(completion);
            }
            Entry::Finish { execution_id, outcome } => self.records.get_mut(&execution_id).unwrap().outcome = Some(outcome),
        }
    }
}

/// When an appended frame reaches stable storage.
#[derive(Clone, Copy)]
enum Durability {
    /// Before `append` returns (admission, terminal outcome).
    Synced,
    /// By the caller, through an [`IntentSync`] (host-call intent).
    Caller,
    /// With the next synced frame (host-call completion).
    Deferred,
}

/// The sync a host call's intent still owes before the call may dispatch.
/// It holds its own handle on the journal file, so it is made after the
/// caller has released the lock that guards the journal: one call's disk
/// wait does not hold up another execution, or a status read.
#[must_use = "the intent is not durable until it is synced; do not dispatch without it"]
pub struct IntentSync(Arc<File>);

impl IntentSync {
    /// Make the intent (and every completion written before it) durable.
    pub fn sync(self) -> Result<(), String> {
        self.0.sync_data().map_err(|e| format!("execution journal write failed; reconciliation required: {e}"))
    }
}

