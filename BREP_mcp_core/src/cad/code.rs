//! Code mode (contract §9–§12): a restricted JavaScript / TypeScript program
//! with an exported async `main()`, run on Boa — the app's own interpreter,
//! unchanged — in a WORKER PROCESS of its own (`brep_script_core::worker`), talking to the CAD service only by message
//! passing.
//!
//! The worker is the host's OWN executable started with a hidden flag
//! ([`WORKER_FLAG`], [`worker_entry`]), so a shipped app — one binary — needs
//! nothing beside it. The host that starts the worker owns it: it relays each host call to the
//! service and the answer back, and it KILLS the worker on an agent's halt
//! (`cad.code.cancel`), on the CPU budget (the worker's own CPU time — it
//! sleeps while it waits on the host), on the memory limit (its resident
//! set) or on the wall deadline. A kill stops any script however it loops;
//! nothing of the host is reachable from it. Accepted operations belong to
//! the service: an execution that ends, throws, hits a limit or is halted
//! detaches from them and lists them; it never cancels them.
//!
//! TypeScript is type-checked before it runs, by the pinned TypeScript
//! compiler ([`TSC_VERSION`], `tsc --strict --noEmitOnError` against the
//! generated `@brep/cad` declarations with every current tool). Node starts
//! the compiler's launcher; one bounded subprocess checks and emits JavaScript.
//! Only the emitted JavaScript is sent to the existing Boa worker.
//! A type error refuses the program with tsc's diagnostics, and so does a
//! missing or wrong checker: unchecked TypeScript never runs.
use super::keys::{self, Purpose};
use super::ops::{Caller, Grants};
use super::wire::{self, ApiError, Effects};
use super::{canon, CadService};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::io::{BufRead, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{mpsc, Arc, Mutex, Weak};
use std::time::{Duration, Instant};

pub const MAX_ACTIVE: usize = 4;
pub const MAX_ACTIVE_PER_SCOPE: usize = 2;
pub const MAX_RETAINED: usize = 128;

/// The §12 limits, as configured for one execution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Limits {
    pub source_bytes: u64,
    pub input_bytes: u64,
    pub compile_ms: u64,
    pub compile_heap_bytes: u64,
    pub bridge_bytes: u64,
    pub js_cpu_ms: u64,
    pub js_heap_bytes: u64,
    pub wall_ms: u64,
    pub outstanding_host_calls: u64,
    pub log_bytes: u64,
    pub result_bytes: u64,
    pub artifact_count: u64,
    pub artifact_bytes: u64,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            source_bytes: 256 << 10,
            input_bytes: 1 << 20,
            compile_ms: 5_000,
            compile_heap_bytes: 256 << 20,
            bridge_bytes: 16 << 20,
            js_cpu_ms: 5_000,
            js_heap_bytes: 64 << 20,
            wall_ms: 15 * 60 * 1000,
            outstanding_host_calls: 8,
            log_bytes: 64 << 10,
            result_bytes: 64 << 10,
            artifact_count: 16,
            artifact_bytes: 128 << 20,
        }
    }
}

const LIMIT_KEYS: &[&str] = &[
    "sourceBytes", "inputBytes", "compileMs", "compileHeapBytes", "bridgeBytes", "jsCpuMs", "jsHeapBytes", "wallMs",
    "outstandingHostCalls", "logBytes", "resultBytes", "artifactCount", "artifactBytes",
];

impl Limits {
    fn fields(&mut self) -> [&mut u64; 13] {
        [
            &mut self.source_bytes, &mut self.input_bytes, &mut self.compile_ms, &mut self.compile_heap_bytes, &mut self.bridge_bytes,
            &mut self.js_cpu_ms, &mut self.js_heap_bytes, &mut self.wall_ms, &mut self.outstanding_host_calls, &mut self.log_bytes,
            &mut self.result_bytes, &mut self.artifact_count, &mut self.artifact_bytes,
        ]
    }

    /// Requested limits may LOWER the defaults; raising one needs host policy,
    /// which this server does not grant.
    pub fn lowered(requested: Option<&Value>) -> Result<Limits, ApiError> {
        let mut limits = Limits::default();
        let Some(req) = requested.filter(|v| !v.is_null()) else { return Ok(limits) };
        let obj = req.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "limits must be an object"))?;
        if let Some(k) = obj.keys().find(|k| !LIMIT_KEYS.contains(&k.as_str())) {
            return Err(ApiError::new("invalid_arguments", "validation", format!("unknown limit `{k}`")));
        }
        for (key, slot) in LIMIT_KEYS.iter().zip(limits.fields()) {
            if let Some(v) = obj.get(*key) {
                let n = v.as_u64().filter(|n| *n > 0 && *n <= (1 << 53)).ok_or_else(|| ApiError::new("invalid_arguments", "validation", format!("{key} must be a positive safe integer")))?;
                if n > *slot {
                    return Err(ApiError::new("capability_denied", "authorization", format!("raising {key} above {} needs host policy", *slot)));
                }
                *slot = n;
            }
        }
        Ok(limits)
    }

    pub fn to_json(&self) -> Value {
        let mut copy = *self;
        let mut out = serde_json::Map::new();
        for (key, v) in LIMIT_KEYS.iter().zip(copy.fields()) {
            out.insert(key.to_string(), json!(*v));
        }
        Value::Object(out)
    }
}

/// The host policy for code mode: what an execution may be granted at most.
/// `document.*` / `cad.*` / `artifact.*` are bounded to the documents and
/// artifacts the execution creates (or explicitly names); record controls are
/// `own` only. No filesystem, network, process, environment or plugin
/// capability exists to request.
fn effective_grants(requested: &Value, initial: &[(String, String)]) -> Result<(Grants, Value), ApiError> {
    const ALLOWED: &[&str] = &[
        "catalogue.read", "document.create", "document.read", "document.write", "document.close", "artifact.read", "artifact.write",
        "cad.export", "cad.import", "operation.read", "operation.cancel",
    ];
    let list = requested.as_array().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "requestedCapabilities must be an array"))?;
    let mut names = Vec::new();
    let mut effective = Vec::new();
    let mut documents: Vec<String> = Vec::new();
    let mut document_grants = HashMap::<String, Vec<String>>::new();
    let mut created_capabilities = Vec::new();
    for grant in list {
        let obj = grant.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "a capability grant must be an object"))?;
        const FIELDS: &[&str] = &["name", "documentIds", "allowCreatedDocuments", "allowExecutionArtifacts", "formats", "maxBytes", "controlScope"];
        if let Some(k) = obj.keys().find(|k| !FIELDS.contains(&k.as_str())) {
            return Err(ApiError::new("invalid_arguments", "validation", format!("unknown grant field `{k}`")));
        }
        let name = obj.get("name").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "a grant needs a name"))?;
        if !super::ops::CAPABILITIES.contains(&name) {
            return Err(ApiError::new("invalid_arguments", "validation", format!("unknown capability `{name}`")));
        }
        let attributes: &[&str] = match name {
            "document.read" | "document.write" | "document.close" => &["name", "documentIds", "allowCreatedDocuments"],
            "cad.import" | "cad.export" => &["name", "documentIds", "allowCreatedDocuments", "formats", "maxBytes"],
            "artifact.read" | "artifact.write" => &["name", "allowExecutionArtifacts", "maxBytes"],
            "operation.read" | "operation.cancel" => &["name", "controlScope"],
            _ => &["name"],
        };
        if let Some(key) = obj.keys().find(|k| !attributes.contains(&k.as_str())) {
            return Err(ApiError::new("invalid_arguments", "validation", format!("{key} is not an attribute of {name}")));
        }
        if name.starts_with("operation.") && obj.get("controlScope").is_some_and(|v| v.as_str() != Some("own")) {
            return Err(ApiError::new("capability_denied", "authorization", "code mode grants only `own` record control"));
        }
        if !ALLOWED.contains(&name) {
            // execution.*, code.execute and view.* are not grantable to a program:
            // it cannot inspect other executions or start new ones.
            return Err(ApiError::new("capability_denied", "authorization", format!("`{name}` cannot be granted to a code-mode program")));
        }
        if obj.get("formats").is_some_and(|v| *v != json!(["step"])) || obj.get("maxBytes").is_some_and(|v| v.as_u64() != Some(super::artifacts::MAX_ARTIFACT as u64)) {
            return Err(ApiError::new("capability_denied", "authorization", "only STEP with the fixed per-artifact byte limit is currently supported; use execution artifact limits to lower the budget"));
        }
        if obj.get("allowExecutionArtifacts").is_some_and(|v| *v != json!(true)) {
            return Err(ApiError::new("capability_denied", "authorization", "artifact capabilities require allowExecutionArtifacts=true"));
        }
        if obj.get("allowCreatedDocuments").is_some_and(|v| !v.is_boolean()) || obj.get("documentIds").is_some_and(|v| !v.is_array()) {
            return Err(ApiError::new("invalid_arguments", "validation", "invalid document grant"));
        }
        if obj.get("allowCreatedDocuments") == Some(&json!(true)) { created_capabilities.push(name.to_string()); }
        if let Some(ids) = obj.get("documentIds").and_then(Value::as_array) {
            for id in ids {
                let id = id.as_str().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "documentIds holds strings"))?;
                // An existing document is granted only if the submission named it.
                if !initial.iter().any(|(d, _)| d == id) {
                    return Err(ApiError::new("capability_denied", "authorization", "a granted document must be one of initialDocuments"));
                }
                documents.push(id.to_string());
                document_grants.entry(name.to_string()).or_default().push(id.to_string());
            }
        }
        if !names.iter().any(|n| n == name) {
            names.push(name.to_string());
        }
        effective.push(grant.clone());
    }
    let grants = Grants { names, documents: Some(Arc::new(Mutex::new(documents))), created_documents: !created_capabilities.is_empty(), document_grants: Some(Arc::new(Mutex::new(document_grants))), created_capabilities };
    Ok((grants, Value::Array(effective)))
}

/// A host call the realm made, owned JSON both ways.
struct BridgeCall {
    call: u64,
    method: String,
    body: String,
}

enum Wake {
    Reply { call: u64, body: Result<String, String> },
    Stop,
    /// A line the worker wrote.
    Line(String),
    /// The worker's stdout closed.
    Eof,
}

struct Exec {
    id: String,
    scope: String,
    actor: String,
    envelope: String,
    manifest: Value,
    accepted_at: Instant,
    updated_at: Instant,
    key_deadline: Instant,
    state: &'static str,
    sequence: u64,
    /// `not_applicable` (JavaScript), `pending`, `passed` or `failed`.
    type_checking: &'static str,
    capabilities: Value,
    /// Snapshot of the originating request; never changed by another client.
    task_form_supported: bool,
    effects_possible: bool,
    input: Option<InputWaiter>,
    limits: Limits,
    cancel: Arc<AtomicBool>,
    wake: Option<mpsc::SyncSender<Wake>>,
    result: Option<Value>,
    retain_until: Option<Instant>,
}

struct InputWaiter {
    form: super::inputs::PendingExport,
    reply: tokio::sync::oneshot::Sender<super::inputs::Resolution>,
}

impl Exec {
    fn terminal(&self) -> bool {
        matches!(self.state, "succeeded" | "failed" | "cancelled")
    }
    fn status(&self, ops: &[Value]) -> Value {
        let pending: Vec<&Value> = ops.iter().filter(|o| !matches!(o["state"].as_str(), Some("succeeded" | "failed" | "cancelled"))).map(|o| &o["operationId"]).collect();
        json!({
            "executionId": self.id,
            "manifest": self.manifest,
            "acceptedAt": wire::timestamp(self.accepted_at),
            "updatedAt": wire::timestamp(self.updated_at),
            "state": self.state,
            "sequence": self.sequence.to_string(),
            "effectiveCapabilities": self.capabilities,
            "effectiveLimits": self.limits.to_json(),
            "typeChecking": self.type_checking,
            "cancellationRequested": self.cancel.load(Ordering::Relaxed),
            "acceptedOperations": ops.iter().map(|o| &o["operationId"]).collect::<Vec<_>>(),
            "pendingOperations": pending,
            "retainUntil": self.retain_until.map(wire::timestamp),
        })
    }
}

#[derive(Default)]
struct State {
    execs: HashMap<String, Exec>,
    bindings: HashMap<(String, String), String>,
    journal: Option<super::journal::Journal>,
}

pub struct Executions {
    service: Weak<CadService>,
    state: Mutex<State>,
}

fn reject_unknown(req: &Value, allowed: &[&str]) -> Result<(), ApiError> {
    let obj = req.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "the request must be an object"))?;
    if let Some(k) = obj.keys().find(|k| !allowed.contains(&k.as_str())) {
        return Err(ApiError::new("invalid_arguments", "validation", format!("unknown field `{k}`")));
    }
    Ok(())
}

impl Executions {
    pub fn new(service: Weak<CadService>) -> Self {
        Executions { service, state: Mutex::new(State::default()) }
    }

    /// Opt in before serving requests. Reopening never starts a worker: an
    /// incomplete admission becomes a terminal reconciliation failure.
    pub fn configure_persistence(&self, directory: &std::path::Path) -> Result<(), String> {
        let mut state = self.state.lock().unwrap();
        if state.journal.is_some() || !state.execs.is_empty() { return Err("configure persistence before admitting executions".into()); }
        let mut journal = super::journal::Journal::open(directory)?;
        for admission in journal.unfinished() {
            let id = &admission.execution_id;
            let error = ApiError::new("reconciliation_required", "recovery", "the server stopped before recording a terminal outcome; inspect document and external state before submitting new work")
                .effects(Effects::unknown()).exec(id);
            let mut recovery = journal.by_id(id, &admission.scope, &admission.actor).unwrap().recovery_evidence();
            recovery["interrupted"] = json!(true);
            journal.finish(id, json!({
                "executionId":id,"state":"failed","sequence":"2","finishedAt":wire::timestamp(Instant::now()),
                "typeChecking":admission.status["typeChecking"],"output":null,"error":error.to_json(),
                "operations":[],"pendingOperationIds":[],"artifacts":[],"logArtifact":null,"retainUntil":null,
                "recovery":recovery
            }))?;
        }
        state.journal = Some(journal);
        Ok(())
    }

    fn start_host_call(&self, caller: &Caller, call_id: u64, method: &str) -> Result<bool, String> {
        let mut state = self.state.lock().unwrap();
        if state.journal.is_none() { return Ok(false); }
        let id = caller.execution.as_deref().ok_or("bridge call without execution")?;
        // The dispatcher owns caller identity. Re-check cancellation while
        // holding the same lock used by cancellation and terminal completion.
        let execution = state.execs.get(id).ok_or("execution no longer active")?;
        if execution.terminal() || execution.cancel.load(Ordering::Relaxed) { return Err("execution stopped before call admission".into()); }
        let intent = state.journal.as_mut().unwrap().start_call(id, call_id, method)?;
        // The intent's sync runs with the lock released: another execution's
        // admission, call or status read does not wait on this disk write.
        drop(state);
        if let Err(error) = intent.sync() {
            if let Some(journal) = self.state.lock().unwrap().journal.as_mut() {
                journal.poison();
            }
            return Err(error);
        }
        Ok(true)
    }

    fn finish_host_call(&self, caller: &Caller, call_id: u64, completion: super::journal::CallCompletion) -> Result<(), String> {
        let mut state = self.state.lock().unwrap();
        let id = caller.execution.as_deref().ok_or("bridge call without execution")?;
        state.journal.as_mut().ok_or("execution journal unavailable")?.finish_call(id, call_id, completion)
    }

    pub fn persistent(&self) -> bool { self.state.lock().unwrap().journal.is_some() }

    fn envelope(req: &Value, trusted: bool) -> Result<String, ApiError> {
        if req.to_string().len() > (1 << 20) + (256 << 10) {
            return Err(ApiError::new("resource_limit", "validation", "the submission is larger than the source and input limits"));
        }
        canon::canonical(&json!({
            "manifest":req.get("manifest"),"language":req.get("language"),"source":req.get("source"),
            "initialDocuments":req.get("initialDocuments"),"requestedCapabilities":req.get("requestedCapabilities"),
            "limits":req.get("limits"),"trusted":trusted,
        })).map_err(|e| ApiError::new("invalid_arguments", "validation", e))
    }

    fn durable_binding(caller: &Caller, issued: &str, retry: Option<&str>) -> String {
        super::journal::Journal::binding(&caller.scope, &caller.actor, if retry.is_some() { "front-door" } else { "issued" }, retry.unwrap_or(issued))
    }

    fn stored_status(record: &super::journal::Record) -> Value {
        let mut status = record.admission.status.clone();
        if let Some(outcome) = &record.outcome {
            status["state"] = outcome["state"].clone();
            status["sequence"] = outcome["sequence"].clone();
            status["updatedAt"] = outcome["finishedAt"].clone();
            status["typeChecking"] = outcome["typeChecking"].clone();
            status["pendingOperations"] = outcome["pendingOperationIds"].clone();
            status["acceptedOperations"] = outcome["operations"].as_array().into_iter().flatten().map(|op| op["operationId"].clone()).collect();
            status["retainUntil"] = Value::Null;
        }
        status["persistence"] = json!({"durable":true,"automaticReplay":false});
        status
    }

    fn stored_result(record: &super::journal::Record) -> Result<Value, ApiError> {
        let mut result = record.outcome.clone().ok_or_else(|| ApiError::new("reconciliation_required", "recovery", "durable admission has no terminal outcome").effects(Effects::unknown()))?;
        // Result JSON survives. Artifacts and app handles do not become durable
        // merely because their references occur in a durable result.
        result["hostCallEvidence"] = record.recovery_evidence();
        result["persistence"] = json!({"durable":true,"referencedArtifactsAndHandlesMayBeUnavailable":true,"automaticReplay":false});
        Ok(result)
    }

    fn durable_record<'a>(state: &'a State, caller: &Caller, id: &str) -> Result<Option<&'a super::journal::Record>, ApiError> {
        caller.require(&["execution.read"])?;
        if caller.execution.is_some() { return Ok(None); }
        Ok(state.journal.as_ref().and_then(|journal| journal.by_id(id, &caller.scope, &caller.actor)))
    }

    /// Check durable retries before starting an app session. Admission repeats
    /// this check under its lock, so simultaneous retries cannot race it.
    pub fn replay(&self, caller: &Caller, req: &Value, trusted: bool, retry: Option<&str>) -> Result<Option<Value>, ApiError> {
        let envelope = Self::envelope(req, trusted)?;
        let binding = Self::durable_binding(caller, req["idempotencyKey"].as_str().unwrap_or(""), retry);
        let state = self.state.lock().unwrap();
        self.replay_locked(&state, caller, &binding, &envelope)
    }

    fn replay_locked(&self, state: &State, caller: &Caller, binding: &str, envelope: &str) -> Result<Option<Value>, ApiError> {
        let Some(journal) = &state.journal else { return Ok(None) };
        caller.require(&["execution.read"])?;
        if caller.execution.is_some() { return Err(ApiError::new("capability_denied", "authorization", "programs cannot inspect executions")); }
        let record = journal.by_binding(binding, &canon::sha256_hex(envelope.as_bytes()))
            .map_err(|e| ApiError::new("idempotency_conflict", "admission", e))?;
        let Some(record) = record else { return Ok(None) };
        if let Some(exec) = state.execs.get(&record.admission.execution_id) {
            let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "lookup", "the service is stopping"))?;
            return Ok(Some(exec.status(&service.ops.of_execution(&exec.id))));
        }
        Ok(Some(Self::stored_status(record)))
    }

    pub async fn shutdown(&self) {
        {
            for execution in self.state.lock().unwrap().execs.values() {
                if !execution.terminal() {
                    execution.cancel.store(true, Ordering::Relaxed);
                    if let Some(wake) = &execution.wake { let _ = wake.try_send(Wake::Stop); }
                }
            }
        }
        // Let supervisors reap their child processes before the CLI runtime
        // exits; merely setting a flag can otherwise orphan a computing worker.
        let deadline = Instant::now() + Duration::from_secs(5);
        while Instant::now() < deadline {
            if self.state.lock().unwrap().execs.values().all(Exec::terminal) { break; }
            tokio::time::sleep(Duration::from_millis(10)).await;
        }
    }

    fn sweep(st: &mut State) {
        let now = Instant::now();
        let gone: Vec<String> = st.execs.values().filter(|e| e.retain_until.is_some_and(|t| t <= now)).map(|e| e.id.clone()).collect();
        for id in &gone {
            st.execs.remove(id);
        }
        st.bindings.retain(|_, e| !gone.contains(e));
    }

    pub fn submit(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        self.submit_as(caller, req, false)
    }

    /// `trusted`: the MCP's own `run_script` — the program gets the local
    /// caller's whole authority over documents and every app tool, but never
    /// the right to start or inspect executions. Otherwise the requested
    /// grants are intersected with code mode's policy.
    pub fn submit_as(&self, caller: &Caller, req: &Value, trusted: bool) -> Result<Value, ApiError> {
        self.admit(caller, req, trusted, None, None)
    }

    /// The host's own policy: a trusted submission whose limits the HOST sets
    /// (they may exceed the defaults a client can only lower). The test-mcp
    /// runner's script mode, whose gate scripts run up to an hour and drive
    /// hundreds of tool calls.
    pub fn submit_with_limits(&self, caller: &Caller, req: &Value, limits: Limits) -> Result<Value, ApiError> {
        self.admit(caller, req, true, Some(limits), None)
    }

    /// Drop a terminal execution now instead of at its retention deadline,
    /// with the artifacts it made: a host that has read the result and plays
    /// hundreds of programs in one process (the test-mcp runner) would
    /// otherwise reach `MAX_RETAINED`.
    pub fn forget(&self, id: &str) {
        let mut st = self.state.lock().unwrap();
        if st.execs.get(id).is_some_and(Exec::terminal) {
            st.execs.remove(id);
            st.bindings.retain(|_, e| e != id);
            if let Some(service) = self.service.upgrade() {
                service.artifacts.drop_execution(id);
            }
        }
    }

    pub fn submit_with_retry(&self, caller: &Caller, req: &Value, trusted: bool, retry: Option<&str>) -> Result<Value, ApiError> {
        self.admit(caller, req, trusted, None, retry)
    }

    fn admit(&self, caller: &Caller, req: &Value, trusted: bool, host_limits: Option<Limits>, retry: Option<&str>) -> Result<Value, ApiError> {
        reject_unknown(req, &["manifest", "language", "source", "initialDocuments", "requestedCapabilities", "limits", "idempotencyKey"])?;
        let key_text = req.get("idempotencyKey").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "idempotencyKey is required (cad_issue_key with purpose execution)"))?;
        let envelope = Self::envelope(req, trusted)?;
        let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "admission", "the service is stopping"))?;
        let mut st = self.state.lock().unwrap();
        Self::sweep(&mut st);
        let durable_binding = Self::durable_binding(caller, key_text, retry);
        if let Some(status) = self.replay_locked(&st, caller, &durable_binding, &envelope)? { return Ok(status); }
        let key = keys::verify(key_text, &caller.scope, Purpose::Execution)?;
        let binding = (caller.scope.clone(), key.nonce.clone());
        if let Some(id) = st.bindings.get(&binding).cloned() {
            caller.require(&["execution.read"])?;
            let e = &st.execs[&id];
            if e.envelope != envelope {
                return Err(ApiError::new("idempotency_conflict", "admission", "this key was used for a different submission").exec(&id));
            }
            let ops = service.ops.of_execution(&id);
            return Ok(e.status(&ops));
        }
        if key.expired() {
            return Err(ApiError::new("idempotency_expired", "admission", "the key's first-admission window has closed").effects(Effects::unknown()));
        }
        caller.require(&["code.execute", "execution.read"])?;
        let manifest = req.get("manifest").cloned().unwrap_or(Value::Null);
        service.catalogue.check_manifest(&manifest)?;
        let typescript = match req.get("language").and_then(Value::as_str) {
            Some("javascript") => false,
            Some("typescript") => true,
            _ => return Err(ApiError::new("invalid_arguments", "validation", "language must be javascript or typescript")),
        };
        let limits = match host_limits {
            Some(limits) => limits,
            None => Limits::lowered(req.get("limits"))?,
        };
        if req.get("initialDocuments").map_or(0, |v| v.to_string().len()) as u64 > limits.input_bytes {
            return Err(ApiError::new("resource_limit", "validation", "initialDocuments exceeds inputBytes"));
        }
        let source = req.get("source").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "source is required"))?.to_string();
        if source.len() as u64 > limits.source_bytes {
            return Err(ApiError::new("resource_limit", "validation", format!("source is larger than {} bytes", limits.source_bytes)));
        }
        let mut initial = Vec::new();
        for c in req.get("initialDocuments").and_then(Value::as_array).cloned().unwrap_or_default() {
            initial.push(wire::document_context(&c)?);
        }
        let (grants, capabilities) = if trusted {
            let names: Vec<String> = super::ops::CAPABILITIES
                .iter()
                .filter(|n| !matches!(**n, "code.execute" | "execution.read" | "execution.cancel"))
                .map(|n| n.to_string())
                .collect();
            let listed = Value::Array(names.iter().map(|n| json!({ "name": n })).collect());
            (Grants { names, documents: None, created_documents: true, document_grants: None, created_capabilities: vec![] }, listed)
        } else {
            effective_grants(req.get("requestedCapabilities").unwrap_or(&json!([])), &initial)?
        };
        let active = st.execs.values().filter(|e| !e.terminal());
        let (total, scoped) = active.fold((0, 0), |(t, s), e| (t + 1, s + usize::from(e.scope == caller.scope)));
        if total >= MAX_ACTIVE || scoped >= MAX_ACTIVE_PER_SCOPE || st.execs.len() >= MAX_RETAINED {
            return Err(ApiError::new("overloaded", "admission", "too many executions are running; retry the same key shortly"));
        }
        let id = wire::mint("exec");
        let cancel = Arc::new(AtomicBool::new(false));
        let (wake_tx, wake_rx) = mpsc::sync_channel(32);
        let exec = Exec {
            effects_possible: false,
            id: id.clone(),
            scope: caller.scope.clone(),
            actor: caller.actor.clone(),
            envelope: envelope.clone(),
            manifest: manifest.clone(),
            accepted_at: Instant::now(),
            updated_at: Instant::now(),
            key_deadline: key.deadline,
            state: "queued",
            sequence: 1,
            type_checking: if typescript { "pending" } else { "not_applicable" },
            capabilities,
            task_form_supported: trusted && super::inputs::task_form_supported(),
            input: None,
            limits,
            cancel: cancel.clone(),
            wake: Some(wake_tx.clone()),
            result: None,
            retain_until: None,
        };
        let status = exec.status(&[]);
        if let Some(journal) = &mut st.journal {
            journal.admit(super::journal::Admission { binding: durable_binding, submission: canon::sha256_hex(envelope.as_bytes()),
                execution_id:id.clone(),scope:caller.scope.clone(),actor:caller.actor.clone(),status:status.clone(),bridge_evidence:true })
                .map_err(|e| ApiError::new("persistence_failed", "admission", e).exec(&id).effects(Effects::unknown()))?;
        }
        service.artifacts.set_execution_budget(&id, limits.artifact_count, limits.artifact_bytes);
        st.execs.insert(id.clone(), exec);
        st.bindings.insert(binding, id.clone());
        drop(st);

        let program_caller = Caller { scope: caller.scope.clone(), actor: caller.actor.clone(), execution: Some(id.clone()), grants };
        let runtime = tokio::runtime::Handle::current();
        let (bridge_tx, bridge_rx) = tokio::sync::mpsc::channel::<BridgeCall>(limits.outstanding_host_calls as usize);
        runtime.spawn(dispatch(service.clone(), program_caller, bridge_rx, wake_tx.clone()));
        let svc = service.clone();
        let exec_id = id.clone();
        let spawned = std::thread::Builder::new()
            .name(format!("brep-code-{}", &id[id.len() - 6..]))
            .spawn(move || {
                let outcome = realm(&svc, &exec_id, typescript, source, limits, cancel, bridge_tx, wake_tx, wake_rx);
                svc.code.finish(&svc, &exec_id, outcome);
            });
        if let Err(error) = spawned {
            self.finish(&service, &id, Outcome { result: Err(ApiError::new("internal_error", "admission", error.to_string())), logs: vec![] });
            return Err(ApiError::new("internal_error", "admission", error.to_string()).exec(&id));
        }
        Ok(status)
    }

    fn set_type_checking(&self, id: &str, verdict: &'static str) {
        if let Some(e) = self.state.lock().unwrap().execs.get_mut(id) {
            e.type_checking = verdict;
            e.updated_at = Instant::now();
            e.sequence += 1;
        }
    }

    fn set_state(&self, id: &str, state: &'static str) {
        if let Some(e) = self.state.lock().unwrap().execs.get_mut(id) {
            let state = if e.input.is_some() { "input_required" } else { state };
            if !e.terminal() && e.state != state {
                e.state = state;
                e.updated_at = Instant::now();
                e.sequence += 1;
            }
        }
    }

    fn finish(&self, service: &CadService, id: &str, outcome: Outcome) {
        let ops = service.ops.of_execution(id);
        let pending: Vec<Value> = ops.iter().filter(|o| !matches!(o["state"].as_str(), Some("succeeded" | "failed" | "cancelled"))).map(|o| o["operationId"].clone()).collect();
        let (state, output, mut error) = match outcome.result {
            Ok(v) => ("succeeded", Some(v), None),
            Err(e) => (if e.code == "cancelled" { "cancelled" } else { "failed" }, None, Some(e)),
        };
        // Success requires every accepted operation to be terminal: unawaited
        // work is a failed execution that names it, never a hidden success.
        let (mut state, mut output) = if state == "succeeded" && !pending.is_empty() {
            error = Some(ApiError::new("invalid_arguments", "execution", format!("main returned with {} accepted operation(s) still running; await them", pending.len())));
            ("failed", None)
        } else {
            (state, output)
        };
        let mut st = self.state.lock().unwrap();
        let Some(e) = st.execs.get_mut(id) else { return };
        if let Some(input) = e.input.take() {
            let _ = input.reply.send(super::inputs::Resolution::Cancelled);
            if state == "succeeded" {
                state = "failed"; output = None;
                error = Some(ApiError::new("invalid_arguments", "execution", "main returned with an input request still pending; await exportOptions"));
            }
        }
        // Script failure is not a rollback. Raw tool calls and accepted operations
        // may have committed before the worker failed or lost its reply.
        if e.effects_possible {
            if let Some(error) = &mut error { error.effects = Effects::unknown(); error.retry = "inspect_then_new_submission"; }
        }
        service.event("execution_terminal", Some(id), Some(state), error.as_ref().map(|e| e.code), None);
        let now = Instant::now();
        e.state = state;
        e.updated_at = now;
        e.sequence += 1;
        e.wake = None;
        e.retain_until = Some((now + super::ops::RETENTION).max(e.key_deadline));
        let log = if outcome.logs.is_empty() {
            Value::Null
        } else {
            service
                .artifacts
                .create(super::artifacts::Owner { scope: e.scope.clone(), execution: Some(id.to_string()) }, "text/plain", outcome.logs.join("\n").into_bytes())
                .unwrap_or(Value::Null)
        };
        e.result = Some(json!({
            "executionId": id,
            "state": state,
            "sequence": e.sequence.to_string(),
            "finishedAt": wire::timestamp(now),
            "typeChecking": e.type_checking,
            "output": output,
            "error": error.map(|x| x.exec(id).to_json()),
            "operations": ops,
            "pendingOperationIds": pending,
            "artifacts": service.artifacts.owned_by(&e.scope, id),
            "logArtifact": log,
            "retainUntil": e.retain_until.map(wire::timestamp),
        }));
        let result = e.result.clone().unwrap();
        if let Some(journal) = &mut st.journal {
            if let Err(error) = journal.finish(id, result) {
                let e = st.execs.get_mut(id).unwrap();
                e.state = "failed";
                let result = e.result.as_mut().unwrap();
                result["state"] = json!("failed");
                result["error"] = ApiError::new("persistence_failed", "completion", error).effects(Effects::unknown()).exec(id).to_json();
                result["output"] = Value::Null;
            }
        }
    }

    fn lookup<'a>(st: &'a mut State, caller: &Caller, id: &str) -> Result<&'a mut Exec, ApiError> {
        caller.require(&["execution.read"])?;
        match wire::handle_shape("exec", id) {
            wire::HandleShape::Current => {}
            wire::HandleShape::OtherEpoch => return Err(ApiError::new("handle_epoch_mismatch", "lookup", "the execution is from an earlier run of the server").effects(Effects::unknown())),
            wire::HandleShape::Malformed => return Err(ApiError::new("invalid_arguments", "validation", "executionId is not an execution handle")),
        }
        Self::sweep(st);
        let unknown = || ApiError::new("execution_unknown", "lookup", "no retained execution has that id").effects(Effects::unknown());
        let e = st.execs.get_mut(id).ok_or_else(unknown)?;
        if e.scope != caller.scope || e.actor != caller.actor || caller.execution.is_some() {
            return Err(unknown());
        }
        Ok(e)
    }

    pub fn status(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["executionId"])?;
        let id = req.get("executionId").and_then(Value::as_str).unwrap_or_default();
        let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "lookup", "the service is stopping"))?;
        let mut st = self.state.lock().unwrap();
        Self::sweep(&mut st);
        if !st.execs.contains_key(id) {
            if let Some(record) = Self::durable_record(&st, caller, id)? { return Ok(Self::stored_status(record)); }
        }
        let e = Self::lookup(&mut st, caller, id)?;
        // A terminal status is the immutable snapshot taken when it ended.
        if let Some(r) = &e.result {
            let mut s = e.status(r["operations"].as_array().map(Vec::as_slice).unwrap_or(&[]));
            s["pendingOperations"] = r["pendingOperationIds"].clone();
            return Ok(s);
        }
        let ops = service.ops.of_execution(id);
        Ok(e.status(&ops))
    }

    pub fn result(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["executionId"])?;
        let id = req.get("executionId").and_then(Value::as_str).unwrap_or_default();
        let mut st = self.state.lock().unwrap();
        Self::sweep(&mut st);
        if !st.execs.contains_key(id) {
            if let Some(record) = Self::durable_record(&st, caller, id)? { return Self::stored_result(record); }
        }
        let e = Self::lookup(&mut st, caller, id)?;
        e.result.clone().ok_or_else(|| ApiError::new("not_ready", "lookup", format!("the execution is {}", e.state)).exec(id))
    }

    /// Execution-only cancellation: closes the bridge and stops the program
    /// at its next interrupt check. It never cancels accepted operations.
    pub fn cancel(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["executionId"])?;
        caller.require(&["execution.cancel"])?;
        let id = req.get("executionId").and_then(Value::as_str).unwrap_or_default();
        let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "lookup", "the service is stopping"))?;
        let mut st = self.state.lock().unwrap();
        Self::sweep(&mut st);
        if !st.execs.contains_key(id) {
            if let Some(record) = Self::durable_record(&st, caller, id)? { return Ok(Self::stored_status(record)); }
        }
        let e = Self::lookup(&mut st, caller, id)?;
        if e.terminal() {
            drop(st);
            return self.status(caller, req);
        }
        if !e.cancel.swap(true, Ordering::Relaxed) {
            if let Some(mut input) = e.input.take() {
                let resolution = input.form.cancel(Instant::now()).clone();
                let _ = input.reply.send(resolution);
            }
            e.updated_at = Instant::now();
            e.sequence += 1;
            service.event("cancel_requested", Some(id), Some(e.state), None, Some(e.sequence));
            if let Some(w) = &e.wake {
                let _ = w.try_send(Wake::Stop); // the atomic flag remains authoritative when the mailbox is full
            }
        }
        let ops = service.ops.of_execution(id);
        Ok(e.status(&ops))
    }

    /// Exposes only the requests owned by an externally addressable execution.
    pub(crate) fn input_requests(&self, caller: &Caller, id: &str) -> Result<rmcp::model::InputRequests, ApiError> {
        let mut state = self.state.lock().unwrap();
        let e = Self::lookup(&mut state, caller, id)?;
        Self::expire_input(e, Instant::now());
        match &mut e.input {
            Some(input) => input.form.input_requests(Instant::now()).map_err(|message| ApiError::new("internal_error", "input", message)),
            None => Ok(Default::default()),
        }
    }

    fn expire_input(e: &mut Exec, now: Instant) {
        let resolved = e.input.as_mut().and_then(|input| input.form.resolution(now).cloned());
        if let Some(resolved) = resolved {
            let input = e.input.take().unwrap();
            let _ = input.reply.send(resolved);
            e.state = "waiting"; e.updated_at = now; e.sequence += 1;
        }
    }

    pub(crate) fn update_input(&self, caller: &Caller, id: &str, responses: &rmcp::model::InputResponses) -> Result<(), ApiError> {
        let mut state = self.state.lock().unwrap();
        let e = Self::lookup(&mut state, caller, id)?;
        Self::expire_input(e, Instant::now());
        if e.cancel.load(Ordering::Relaxed) || e.terminal() || responses.len() != 1 {
            return Err(ApiError::new("invalid_arguments", "input", "update requires exactly one outstanding input request on a live execution"));
        }
        let input = e.input.as_mut().ok_or_else(|| ApiError::new("invalid_arguments", "input", "execution has no outstanding input request"))?;
        let (request_id, response) = responses.iter().next().unwrap();
        let resolved = input.form.respond(request_id, response, Instant::now())
            .map_err(|message| ApiError::new("invalid_arguments", "input", message))?.clone();
        let input = e.input.take().unwrap();
        let _ = input.reply.send(resolved);
        e.state = "waiting"; e.updated_at = Instant::now(); e.sequence += 1;
        Ok(())
    }

    pub(crate) async fn export_options(&self, caller: &Caller, request: &Value) -> Result<Value, ApiError> {
        use super::inputs::{ExportForm, PendingExport, Resolution};
        let invalid = |message| ApiError::new("invalid_arguments", "input", message);
        let service = self.service.upgrade().ok_or_else(|| invalid("service is stopping".into()))?;
        let descriptor = service.describe(&["brep.document.document_export".into()]).await?;
        let formats = descriptor["methods"][0]["inputSchema"]["properties"]["format"]["enum"].as_array()
            .ok_or_else(|| invalid("export format catalogue is unavailable".into()))?
            .iter().map(|value| value.as_str().map(str::to_owned).ok_or_else(|| invalid("invalid export format catalogue".into())))
            .collect::<Result<Vec<_>, _>>()?;
        let form = ExportForm::new(request, formats).map_err(invalid)?;
        if let Some(options) = form.complete() { return Ok(json!({"format":options.format,"path":options.path})); }
        let id = caller.execution.as_deref().ok_or_else(|| invalid("exportOptions requires a script execution".into()))?;
        let (mut receiver, deadline) = {
            let mut state = self.state.lock().unwrap();
            let e = state.execs.get_mut(id).filter(|e| e.scope == caller.scope && e.actor == caller.actor)
                .ok_or_else(|| invalid("execution is unavailable".into()))?;
            if !e.task_form_supported {
                return Err(ApiError::new("input_required", "input", "Supply format and path explicitly, or submit run_script with wait_ms: 0 from a client supporting modern Tasks and form elicitation to request missing export options"));
            }
            if e.terminal() || e.cancel.load(Ordering::Relaxed) || e.input.is_some() {
                return Err(invalid("execution is terminal, cancelled or already waiting for input".into()));
            }
            let now = Instant::now();
            let pending = PendingExport::new(wire::mint("input"), form, now, e.accepted_at + Duration::from_millis(e.limits.wall_ms)).map_err(invalid)?;
            let deadline = pending.deadline();
            let (reply, receiver) = tokio::sync::oneshot::channel();
            e.input = Some(InputWaiter { form: pending, reply });
            e.state = "input_required"; e.updated_at = now; e.sequence += 1;
            (receiver, deadline)
        };
        let result = tokio::select! {
            result = &mut receiver => result,
            _ = tokio::time::sleep_until(tokio::time::Instant::from_std(deadline)) => {
                {
                    let mut state = self.state.lock().unwrap();
                    if let Some(e) = state.execs.get_mut(id) { Self::expire_input(e, Instant::now()); }
                }
                receiver.await
            }
        };
        match result.unwrap_or(Resolution::Cancelled) {
            Resolution::Accepted(options) => Ok(json!({"format":options.format,"path":options.path})),
            Resolution::Declined => Err(ApiError::new("input_declined", "input", "export choices were declined; no export was performed")),
            Resolution::Cancelled => Err(ApiError::new("cancelled", "input", "export input was cancelled; no export was performed")),
            Resolution::Expired => Err(ApiError::new("input_expired", "input", "export input expired; supply format and path explicitly or start a new request")),
        }
    }
}

/// What the realm thread hands back.
struct Outcome {
    result: Result<Value, ApiError>,
    logs: Vec<String>,
}

/// The bridge's host side: each call is served on the tokio runtime under
/// the execution's narrowed caller, and its reply is posted to the realm's
/// mailbox as owned JSON.
async fn dispatch(service: Arc<CadService>, caller: Caller, mut rx: tokio::sync::mpsc::Receiver<BridgeCall>, wake: mpsc::SyncSender<Wake>) {
    while let Some(call) = rx.recv().await {
        let service = service.clone();
        let caller = caller.clone();
        let wake = wake.clone();
        // The execution is the client: its calls keep their order in the
        // app's queue, and another execution's need not wait behind them.
        let client = caller.execution.as_deref().map(crate::host::client_of);
        tokio::spawn(crate::host::with_client(client, async move {
            let body = serve_recorded(service.clone(), caller, &call).await;
            // A realm that has gone away simply never reads this.
            let mut reply = Wake::Reply { call: call.call, body };
            loop {
                match wake.try_send(reply) {
                    Ok(()) => break,
                    Err(mpsc::TrySendError::Full(value)) => {
                        reply = value;
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    }
                    Err(mpsc::TrySendError::Disconnected(_)) => {
                        service.event("late_reply_discarded", None, None, None, None);
                        break;
                    }
                }
            }
        }));
    }
}

/// Journal IO runs off the async executor. Once admitted, a host call may
/// finish after cancellation; its reply is still recorded as an observation.
async fn serve_recorded(service: Arc<CadService>, caller: Caller, call: &BridgeCall) -> Result<String, String> {
    if call.method == "cad.internal.pause" || !service.code.persistent() {
        return serve(&service, &caller, &call.method, &call.body).await;
    }
    // Persist only known method names, never arbitrary worker-controlled text.
    let method = if let Some(name) = call.method.strip_prefix("tool.") {
        if service.script_tool(name).await.is_some() { call.method.clone() } else { "unknown".into() }
    } else if matches!(call.method.as_str(),
        "cad.internal.awaitOperation" | "cad.discovery.search" | "cad.discovery.describe" | "cad.inputs.exportOptions" |
        "cad.documents.list" | "cad.documents.get" | "cad.operations.issueKey" |
        "cad.operations.submit" | "cad.operations.status" | "cad.operations.result" |
        "cad.operations.cancel" | "cad.commands.query" | "cad.artifacts.read" |
        "cad.artifacts.create" | "cad.artifacts.begin" | "cad.artifacts.append" | "cad.artifacts.commit") {
        call.method.clone()
    } else { "unknown".into() };
    let recording_service = service.clone(); let recording_caller = caller.clone(); let call_id = call.call;
    tokio::task::spawn_blocking(move || recording_service.code.start_host_call(&recording_caller, call_id, &method))
        .await.map_err(|_| "host-call admission recorder failed")??;
    let body = serve(&service, &caller, &call.method, &call.body).await;
    let completion = call_observation(&body);
    tokio::task::spawn_blocking(move || service.code.finish_host_call(&caller, call_id, completion))
        .await.map_err(|_| "host-call completion recorder failed; effects unknown")??;
    body
}

fn call_observation(body: &Result<String, String>) -> super::journal::CallCompletion {
    let value = body.as_ref().ok().and_then(|s| serde_json::from_str::<Value>(s).ok());
    let ok = value.as_ref().and_then(|v| v["ok"].as_bool());
    let operation = value.as_ref().and_then(|v| {
        let observation = if ok == Some(true) { &v["value"] } else { &v["error"] };
        let id = observation["operationId"].as_str()?;
        if !id.is_ascii() || wire::handle_shape("op", id) == wire::HandleShape::Malformed { return None; }
        let state = observation["state"].as_str().filter(|s| matches!(*s, "queued" | "running" | "evaluating" | "committing" | "succeeded" | "failed" | "cancelled")).map(str::to_owned);
        let effects = observation["effects"]["disposition"].as_str().filter(|s| matches!(*s, "none" | "committed" | "unknown" | "partial")).map(str::to_owned);
        Some(super::journal::OperationObservation { operation_id:id.into(),state,effects })
    });
    super::journal::CallCompletion {reply_received:body.is_ok(),ok,operation}
}

async fn serve(service: &CadService, caller: &Caller, method: &str, body: &str) -> Result<String, String> {
    let req: Value = serde_json::from_str(body).map_err(|e| e.to_string())?;
    let respond = |r: Result<Value, ApiError>| Ok(wire::respond(r).to_string());
    match method {
        "cad.internal.pause" => {
            let ms = req["ms"].as_u64().unwrap_or(100).min(1000);
            tokio::time::sleep(Duration::from_millis(ms)).await;
            Ok("{}".into())
        }
        "cad.internal.awaitOperation" => {
            // Waits host-side for the terminal result (no transport request
            // is held: this is the in-process bridge).
            let mut delay = 25;
            loop {
                match service.ops.result(caller, &req) {
                    Err(e) if e.code == "not_ready" => {
                        tokio::time::sleep(Duration::from_millis(delay)).await;
                        delay = (delay * 2).min(250);
                    }
                    other => return respond(other),
                }
            }
        }
        // A registry tool, called as a script function (`brep.<group>.<tool>`):
        // the tool's own handler, the very code path an MCP call takes.
        m if m.starts_with("tool.") => {
            let name = &m["tool.".len()..];
            if let Err(e) = caller.require(&["app.tools"]) {
                return respond(Err(e));
            }
            let Some(spec) = service.script_tool(name).await else {
                return respond(Err(ApiError::new("method_unknown", "lookup", format!("no script function for tool `{name}`"))));
            };
            if let Some(message) = crate::script_sdk::undeclared_context(name, &spec.input_schema, &req) {
                return respond(Err(ApiError::new("invalid_arguments", "validation", message)));
            }
            let result = if spec.group == "files" {
                match name.strip_prefix("files_").ok_or_else(|| "invalid file method".to_string())
                    .and_then(|op| crate::files::operation_request(op, req)) {
                    Ok(args) => service.files.call(service, caller, args).await,
                    Err(error) => Err(error),
                }
            } else { (spec.handler)(req).await };
            match result {
                Ok(out) => {
                    let mut value = out.json;
                    if !out.images.is_empty() {
                        let mut refs = Vec::new();
                        for image in out.images {
                            match service.artifacts.create(caller.owner(), &image.mime, image.png) {
                                Ok(r) => refs.push(r),
                                Err(e) => return respond(Err(e)),
                            }
                        }
                        match value.as_object_mut() {
                            Some(o) => {
                                o.insert("images".into(), Value::Array(refs));
                            }
                            None => value = json!({ "value": value, "images": refs }),
                        }
                    }
                    let reply = wire::respond(Ok(value)).to_string();
                    // A reply this large would exhaust the program's worker
                    // (64 MiB) when parsed: refuse it softly, by size.
                    if reply.len() > MAX_TOOL_REPLY {
                        return respond(Err(ApiError::new("resource_limit", "evaluation", format!("{name}'s result is {} bytes, more than a script can take ({MAX_TOOL_REPLY}); ask for less (a filter, a page, one entity)", reply.len()))));
                    }
                    Ok(reply)
                }
                Err(e) => {
                    let e = with_signature(&e, &spec);
                    let mut error = ApiError::from_app(&e, "evaluation");
                    // An unclassified tool failure may follow a partial mutation.
                    if error.code == "evaluation_failed" { error.effects = super::wire::Effects::unknown(); }
                    respond(Err(error))
                },
            }
        }
        "cad.code.submit" | "cad.code.status" | "cad.code.result" | "cad.code.cancel" => {
            respond(Err(ApiError::new("capability_denied", "authorization", "a program cannot start or inspect executions")))
        }
        _ => respond(super::tools::call_service(service, caller, method, &req).await),
    }
}

/// The largest script-function reply handed to a program: well under the
/// worker's 64 MiB once parsed into JavaScript values.
const MAX_TOOL_REPLY: usize = 12 << 20;

/// An argument error from a script function, completed with the fields the
/// function takes: `unknown field `kind`, there are no fields` alone sent
/// agents guessing one round trip per spelling.
fn with_signature(error: &str, spec: &crate::tools::ToolSpec) -> String {
    let argument_error = ["unknown field", "missing field", "invalid type", "unknown variant", "there are no fields"]
        .iter().any(|m| error.contains(m));
    if !argument_error || error.contains(" takes {") { return error.to_string(); }
    let path = format!("brep.{}.{}", crate::script_sdk::ident(spec.group), crate::script_sdk::method_name(spec.group, &spec.name));
    format!("{error} — {path} takes {}", signature(&spec.input_schema))
}

/// `{id: string, patch?: object}` from an input schema, at most ~600 bytes.
pub(crate) fn signature(schema: &Value) -> String {
    let Some(props) = schema.get("properties").and_then(Value::as_object).filter(|p| !p.is_empty()) else { return "{} (no arguments)".into() };
    let required: Vec<&str> = schema["required"].as_array().into_iter().flatten().filter_map(Value::as_str).collect();
    let mut fields = Vec::new();
    let mut len = 0;
    for (name, p) in props {
        let ty = match &p["type"] {
            Value::String(t) => t.clone(),
            Value::Array(ts) => ts.iter().filter_map(Value::as_str).collect::<Vec<_>>().join("|"),
            _ if p.get("enum").is_some() => p["enum"].as_array().into_iter().flatten().take(6).map(|v| v.to_string()).collect::<Vec<_>>().join("|"),
            _ => "any".into(),
        };
        let field = format!("{name}{}: {ty}", if required.contains(&name.as_str()) { "" } else { "?" });
        len += field.len() + 2;
        if len > 600 { fields.push("…".to_string()); break; }
        fields.push(field);
    }
    format!("{{{}}}", fields.join(", "))
}

/// The TypeScript compiler a TypeScript program is checked with: exactly
/// this version, or no TypeScript runs.
pub const TSC_VERSION: &str = "7.0.2";
/// Resolve a tool before the compiler enters its scratch directory and clears
/// its environment. Explicit paths remain explicit; bare names use the host's
/// PATH. Node on Windows has the usual executable suffix.
fn resolve_tool_path(requested: &std::path::Path, search_path: Option<&std::ffi::OsStr>, executable: bool) -> Option<std::path::PathBuf> {
    let usable = |path: std::path::PathBuf| {
        if !path.is_file() { return None; }
        #[cfg(unix)]
        if executable {
            use std::os::unix::fs::PermissionsExt;
            if std::fs::metadata(&path).ok()?.permissions().mode() & 0o111 == 0 { return None; }
        }
        #[cfg(not(unix))]
        let _ = executable;
        path.canonicalize().ok()
    };
    if requested.is_absolute() || requested.components().count() > 1 {
        return usable(requested.to_path_buf());
    }
    for directory in std::env::split_paths(search_path?) {
        let path = directory.join(requested);
        if let Some(path) = usable(path.clone()) { return Some(path); }
        #[cfg(windows)]
        if executable && path.extension().is_none() {
            if let Some(path) = usable(path.with_extension("exe")) { return Some(path); }
        }
    }
    None
}

/// Only search installation locations controlled by the launching user. In
/// particular, never discover/execute a compiler from a document's directory.
fn user_node_prefixes() -> Vec<std::path::PathBuf> {
    use std::path::PathBuf;
    let mut prefixes = Vec::new();
    for key in ["npm_config_prefix", "NPM_CONFIG_PREFIX"] {
        if let Some(value) = std::env::var_os(key).filter(|v| !v.is_empty()) { prefixes.push(PathBuf::from(value)); }
    }
    if let Some(home) = std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE")).map(PathBuf::from) {
        prefixes.extend([home.join(".local"), home.join(".npm-global")]);
        let nvm = std::env::var_os("NVM_DIR").map(PathBuf::from).unwrap_or_else(|| home.join(".nvm"));
        // Bounded, one-level discovery: no recursive home-directory scan and
        // no shell startup scripts or package-manager commands at runtime.
        if let Some(entries) = nvm.is_absolute().then(|| std::fs::read_dir(nvm.join("versions/node")).ok()).flatten() {
            let mut versions: Vec<_> = entries.filter_map(Result::ok)
                .filter(|entry| entry.file_name().to_string_lossy().starts_with('v'))
                .map(|entry| entry.path()).collect();
            versions.sort();
            prefixes.extend(versions.into_iter().rev().take(32));
        }
    }
    #[cfg(windows)]
    if let Some(appdata) = std::env::var_os("APPDATA") { prefixes.push(PathBuf::from(appdata).join("npm")); }
    prefixes.retain(|p| p.is_absolute());
    prefixes
}

fn compiler_candidates() -> Vec<std::path::PathBuf> {
    use std::path::PathBuf;
    let mut candidates = Vec::new();
    if let Some(path) = std::env::var_os("PATH") {
        for dir in std::env::split_paths(&path).filter(|p| p.is_absolute()) {
            candidates.push(dir.join("tsc"));
            // npm on Windows exposes tsc.cmd, a shell wrapper, beside the
            // package. Execute its real JS launcher through Node instead.
            candidates.push(dir.join("node_modules/typescript/bin/tsc"));
            candidates.push(dir.join("../lib/node_modules/typescript/bin/tsc"));
        }
    }
    let home = std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE")).map(PathBuf::from);
    if let Some(prefix) = typescript_prefix() { candidates.push(prefix.join(PREFIX_LAUNCHER)); }
    if let Some(home) = home { candidates.push(home.join(".local/share/brep-typescript").join(PREFIX_LAUNCHER)); }
    for prefix in user_node_prefixes() {
        candidates.push(prefix.join("lib/node_modules/typescript/bin/tsc"));
        candidates.push(prefix.join("node_modules/typescript/bin/tsc"));
    }
    candidates.retain(|p| p.is_absolute());
    candidates
}

/// The launcher inside an npm `--prefix` installation of TypeScript.
const PREFIX_LAUNCHER: &str = "node_modules/typescript/bin/tsc";

/// The user-local prefix `provision_typescript` installs into and discovery
/// checks first after PATH: `${XDG_DATA_HOME:-$HOME/.local/share}/brep-typescript`.
pub fn typescript_prefix() -> Option<std::path::PathBuf> {
    use std::path::PathBuf;
    let home = std::env::var_os("HOME").or_else(|| std::env::var_os("USERPROFILE")).map(PathBuf::from);
    std::env::var_os("XDG_DATA_HOME").map(PathBuf::from).filter(|p| p.is_absolute())
        .or_else(|| home.filter(|p| p.is_absolute()).map(|p| p.join(".local/share")))
        .map(|data| data.join("brep-typescript"))
}

/// The provisioning command a diagnostic names.
pub const PROVISION_HINT: &str = "run `brep-mcp typescript provision` (or `brep-app --provision-typescript`) once, outside any tool call";

fn checked_tsc(path: &std::path::Path) -> Result<std::path::PathBuf, ApiError> {
    let tsc = resolve_tool_path(path, std::env::var_os("PATH").as_deref(), false)
        .ok_or_else(|| ApiError::new("compile_failed", "compilation", format!("TypeScript compiler missing at {}; code is not executed", path.display())))?;
    let package = tsc.parent().and_then(|p| p.parent()).map(|p| p.join("package.json"));
    let version = package.and_then(|p| std::fs::read_to_string(p).ok()).and_then(|t| serde_json::from_str::<Value>(&t).ok()).and_then(|v| v["version"].as_str().map(str::to_string));
    if version.as_deref() != Some(TSC_VERSION) {
        return Err(ApiError::new("compile_failed", "compilation", format!("TypeScript compiler incompatible: the checker at {} is version {}, not the pinned {TSC_VERSION}", tsc.display(), version.as_deref().unwrap_or("unknown"))));
    }
    Ok(tsc)
}

fn discover_tsc(candidates: impl IntoIterator<Item = std::path::PathBuf>) -> Result<std::path::PathBuf, ApiError> {
    let mut incompatible = None;
    for candidate in candidates {
        if !candidate.is_file() { continue; }
        match checked_tsc(&candidate) {
            Ok(tsc) => return Ok(tsc),
            Err(error) => { if incompatible.is_none() { incompatible = Some(error); } }
        }
    }
    Err(incompatible.unwrap_or_else(|| ApiError::new("compile_failed", "compilation",
        format!("TypeScript compiler missing; code is not executed. JavaScript needs no compiler. To use TypeScript, {PROVISION_HINT}; it installs TypeScript {TSC_VERSION} into {}. Or set BREP_TSC to a TypeScript {TSC_VERSION} bin/tsc launcher.",
            typescript_prefix().map(|p| p.display().to_string()).unwrap_or_else(|| "${XDG_DATA_HOME:-$HOME/.local/share}/brep-typescript".into())))))
}

/// Explicit configuration is authoritative, including an invalid override.
/// Otherwise skip incompatible installations and find a pinned user-local one.
fn tsc_path() -> Result<std::path::PathBuf, ApiError> {
    if let Some(requested) = std::env::var_os("BREP_TSC") { return checked_tsc(std::path::Path::new(&requested)); }
    discover_tsc(compiler_candidates())
}

/// Node runs the trusted compiler only. User JavaScript still runs in Boa.
fn node_path(tsc: &std::path::Path) -> Result<std::path::PathBuf, ApiError> {
    let path = std::env::var_os("PATH");
    let resolve = |p: &std::path::Path| resolve_tool_path(p, path.as_deref(), true);
    if let Some(requested) = std::env::var_os("BREP_NODE") {
        return resolve(std::path::Path::new(&requested)).ok_or_else(|| ApiError::new("compile_failed", "compilation", "TypeScript runtime unavailable: BREP_NODE must name a usable Node executable"));
    }
    if let Some(path) = &path {
        for dir in std::env::split_paths(path).filter(|p| p.is_absolute()) {
            if let Some(node) = resolve(&dir.join("node")) { return Ok(node); }
        }
    }
    // An npm-global compiler can belong to an nvm prefix absent from a
    // desktop launcher's PATH. Prefer that installation's actual Node binary.
    let mut prefixes = Vec::new();
    if let Some(lib) = tsc.ancestors().nth(4).filter(|p| p.file_name().is_some_and(|n| n == "lib")) {
        if let Some(prefix) = lib.parent() { prefixes.push(prefix.to_path_buf()); }
    }
    prefixes.extend(user_node_prefixes());
    for prefix in prefixes {
        for candidate in [prefix.join("bin/node"), prefix.join("node.exe")] {
            if let Some(node) = resolve(&candidate) { return Ok(node); }
        }
    }
    Err(ApiError::new("compile_failed", "compilation", "TypeScript runtime unavailable: install Node for your user, set BREP_NODE to its executable, or put node on PATH"))
}

/// Discover local provisioning without executing a compiler or downloading
/// anything. Compilation verifies the actual launcher before checking code.
pub fn typescript_toolchain() -> Result<(std::path::PathBuf, std::path::PathBuf), ApiError> {
    let tsc = tsc_path()?;
    Ok((node_path(&tsc)?, tsc))
}

/// Run a compiler step in `dir`, killed at `compile_ms`: its status, stdout
/// and stderr.
fn compile_step(mut command: std::process::Command, dir: &std::path::Path, what: &str, limits: &Limits, deadline: Instant, cancel: &AtomicBool) -> Result<(std::process::ExitStatus, Vec<u8>, Vec<u8>), ApiError> {
    if cancel.load(Ordering::Relaxed) { return Err(ApiError::new("cancelled", "compilation", "compilation was cancelled")); }
    if Instant::now() >= deadline { return Err(ApiError::new("resource_limit", "compilation", "the compilation deadline has expired")); }
    #[cfg(unix)]
    { use std::os::unix::process::CommandExt; command.process_group(0); }
    let child = command
        .env_clear()
        .env("PATH", "/usr/bin:/bin")
        .current_dir(dir)
        .stdin(std::process::Stdio::null())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::piped())
        .spawn();
    let mut child = child.map_err(|e| ApiError::new("compile_failed", "compilation", format!("TypeScript compiler unusable: {what} did not start: {e}")))?;
    // Drain both pipes on their own threads: a program whose output exceeds
    // the pipe buffer would otherwise block the step on write and read as a
    // timeout.
    let drain = |pipe: Option<Box<dyn std::io::Read + Send>>| {
        std::thread::spawn(move || {
            let mut bytes = Vec::new();
            if let Some(mut p) = pipe {
                let _ = std::io::Read::read_to_end(&mut std::io::Read::take(&mut p, 1 << 20), &mut bytes);
            }
            bytes
        })
    };
    let out = drain(child.stdout.take().map(|p| Box::new(p) as Box<dyn std::io::Read + Send>));
    let err = drain(child.stderr.take().map(|p| Box::new(p) as Box<dyn std::io::Read + Send>));
    let status = loop {
        match child.try_wait() {
            Ok(Some(s)) => break s,
            Ok(None) if !cancel.load(Ordering::Relaxed) && Instant::now() < deadline && compiler_rss(child.id()).is_none_or(|rss| rss <= limits.compile_heap_bytes) => std::thread::sleep(Duration::from_millis(10)),
            _ => {
                // The pinned tsc launcher can spawn its native compiler. End
                // the process group before joining pipe readers, otherwise an
                // inherited stdout can keep those joins blocked indefinitely.
                #[cfg(unix)]
                let _ = std::process::Command::new("/bin/kill").args(["-KILL", "--", &format!("-{}", child.id())])
                    .stdin(std::process::Stdio::null()).stdout(std::process::Stdio::null()).stderr(std::process::Stdio::null()).status();
                let _ = child.kill();
                let _ = child.wait();
                let _ = (out.join(), err.join());
                if cancel.load(Ordering::Relaxed) { return Err(ApiError::new("cancelled", "compilation", "compilation was cancelled")); }
                return Err(ApiError::new("resource_limit", "compilation", format!("{what} exceeded its time or memory budget ({} ms, {} bytes) and was stopped", limits.compile_ms, limits.compile_heap_bytes)));
            }
        }
    };
    Ok((status, out.join().unwrap_or_default(), err.join().unwrap_or_default()))
}

/// A scratch directory for one compiler step, removed when dropped.
struct Scratch(std::path::PathBuf);

impl Scratch {
    fn new() -> Result<Scratch, ApiError> {
        let dir = std::env::temp_dir().join(format!("brep-code-{}", wire::b64url(&wire::random_128()[..9])));
        crate::host::private_dir(&dir).map_err(|e| ApiError::new("internal_error", "compilation", e))?;
        Ok(Scratch(dir))
    }
    fn write(&self, name: &str, text: &str) -> Result<std::path::PathBuf, ApiError> {
        let path = self.0.join(name);
        std::fs::write(&path, text).map_err(|e| ApiError::new("internal_error", "compilation", e.to_string()))?;
        Ok(path)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// Check and emit in ONE pinned TypeScript invocation. `noEmitOnError` keeps
/// failed programs from producing executable output. The returned scratch
/// directory is retained until its emitted program has been read.
fn type_check_and_emit(source: &str, declarations: &str, limits: &Limits, deadline: Instant, cancel: &AtomicBool) -> Result<Scratch, ApiError> {
    let (node, tsc) = typescript_toolchain()?;
    let dir = Scratch::new()?;
    probe_launcher(&node, &tsc, &dir, limits, deadline, cancel)?;
    let decls = dir.write("brep-cad.d.ts", declarations)?;
    let main = dir.write("main.ts", source)?;
    let mut command = std::process::Command::new(&node);
    command
        .arg(&tsc)
        .args(["--noEmitOnError", "--strict", "--pretty", "false", "--target", "es2022", "--module", "esnext", "--moduleResolution", "bundler", "--lib", "es2022", "--types", "", "--outDir"])
        .arg(dir.0.join("emitted"))
        .arg(&decls)
        .arg(&main);
    let (status, stdout, stderr) = compile_step(command, &dir.0, "TypeScript checking and emission", limits, deadline, cancel)?;
    if status.success() {
        return Ok(dir);
    }
    let prefix = format!("{}/", dir.0.display());
    let text = format!("{}{}", String::from_utf8_lossy(&stdout), String::from_utf8_lossy(&stderr)).replace(&prefix, "");
    let diagnostics: Vec<String> = text.lines().filter(|l| !l.trim().is_empty()).map(|l| wire::sanitize(l.trim())).take(50).collect();
    let message = brep_script_core::worker::syntax_help(source, &format!("type errors:\n{}", diagnostics.join("\n")));
    Err(ApiError::new("compile_failed", "compilation", message).context(json!({ "diagnostics": diagnostics })))
}

/// Run the resolved launcher the way compilation will (`node tsc --version`,
/// cleared environment) and require the pinned version. Distinguishes an
/// unusable launcher (it fails to run) from an incompatible one.
fn probe_launcher(node: &std::path::Path, tsc: &std::path::Path, dir: &Scratch, limits: &Limits, deadline: Instant, cancel: &AtomicBool) -> Result<(), ApiError> {
    let mut probe = std::process::Command::new(node);
    probe.arg(tsc).arg("--version");
    let (status, stdout, stderr) = compile_step(probe, &dir.0, "TypeScript version probe", limits, deadline, cancel)?;
    if !status.success() {
        return Err(ApiError::new("compile_failed", "compilation", format!("TypeScript compiler unusable: {} {} --version exited with {status}; check BREP_NODE and BREP_TSC: {}", node.display(), tsc.display(), wire::sanitize(&String::from_utf8_lossy(&stderr)))));
    }
    let reported = String::from_utf8_lossy(&stdout).trim().to_string();
    if reported != format!("Version {TSC_VERSION}") {
        return Err(ApiError::new("compile_failed", "compilation", format!("TypeScript compiler incompatible: {} reports `{}`, not Version {TSC_VERSION}", tsc.display(), wire::sanitize(&reported))));
    }
    Ok(())
}

/// What `tools/list` advertises: the toolchain execution would use, AND its
/// launcher probe passing, so "available" never names a compiler that would
/// fail before checking code (a wrong Node, a broken install). The probe runs
/// once per distinct toolchain (paths plus launcher, package and Node
/// modification times); discovery itself re-runs every time, so provisioning
/// or removing a compiler shows on the next listing.
pub fn typescript_status() -> Result<(std::path::PathBuf, std::path::PathBuf), ApiError> {
    type Key = (std::path::PathBuf, std::path::PathBuf, Vec<Option<std::time::SystemTime>>);
    static PROBED: std::sync::Mutex<Option<(Key, Result<(), ApiError>)>> = std::sync::Mutex::new(None);
    let (node, tsc) = typescript_toolchain()?;
    let modified = |p: &std::path::Path| std::fs::metadata(p).and_then(|m| m.modified()).ok();
    let package = tsc.parent().and_then(|p| p.parent()).map(|p| p.join("package.json"));
    let key: Key = (node.clone(), tsc.clone(), vec![modified(&node), modified(&tsc), package.as_deref().and_then(modified)]);
    let mut probed = PROBED.lock().unwrap_or_else(|p| p.into_inner());
    if let Some((cached, outcome)) = probed.as_ref() {
        if *cached == key { return outcome.clone().map(|()| (node, tsc)); }
    }
    let limits = Limits::default();
    let outcome = Scratch::new().and_then(|dir| {
        probe_launcher(&node, &tsc, &dir, &limits, Instant::now() + Duration::from_millis(limits.compile_ms), &AtomicBool::new(false))
    });
    // A deadline or cancellation says nothing about the toolchain: not cached.
    if outcome.as_ref().err().is_none_or(|e| e.code == "compile_failed") {
        *probed = Some((key, outcome.clone()));
    }
    outcome.map(|()| (node, tsc))
}

/// Install the pinned compiler into [`typescript_prefix`] with npm — the one
/// provisioning path, run by a person (`brep-mcp typescript provision`,
/// `brep-app --provision-typescript`), never by a tool call. `npm` is
/// `BREP_NPM`, else `npm` on PATH. Returns what discovery then resolves.
pub fn provision_typescript() -> Result<String, String> {
    let prefix = typescript_prefix().ok_or("no user data directory: set XDG_DATA_HOME or HOME")?;
    let path = std::env::var_os("PATH");
    let npm = match std::env::var_os("BREP_NPM") {
        Some(requested) => resolve_tool_path(std::path::Path::new(&requested), path.as_deref(), true)
            .ok_or_else(|| format!("BREP_NPM does not name a usable npm executable: {}", std::path::Path::new(&requested).display()))?,
        None => resolve_tool_path(std::path::Path::new("npm"), path.as_deref(), true)
            .ok_or("npm not found on PATH: install Node.js (which provides npm), or set BREP_NPM to npm's path")?,
    };
    std::fs::create_dir_all(&prefix).map_err(|e| format!("{}: {e}", prefix.display()))?;
    let status = std::process::Command::new(&npm)
        .args(["install", "--prefix"]).arg(&prefix)
        .args(["--no-audit", "--no-fund", "--save-exact", "--ignore-scripts", &format!("typescript@{TSC_VERSION}")])
        .stdin(std::process::Stdio::null())
        .status()
        .map_err(|e| format!("{} did not start: {e}", npm.display()))?;
    if !status.success() {
        return Err(format!("{} install exited with {status}; nothing was changed in BREP's configuration", npm.display()));
    }
    let installed = prefix.join(PREFIX_LAUNCHER);
    checked_tsc(&installed).map_err(|e| e.message)?;
    let mut report = format!("installed TypeScript {TSC_VERSION} at {}", installed.display());
    match typescript_status() {
        Ok((node, tsc)) => report.push_str(&format!("\nTypeScript available: node {} runs {}", node.display(), tsc.display())),
        Err(e) => report.push_str(&format!("\nTypeScript still unavailable: {}", e.message)),
    }
    if std::env::var_os("BREP_TSC").is_some() {
        report.push_str("\nnote: BREP_TSC is set and overrides discovery; unset it to use this installation");
    }
    Ok(report)
}

/// Read only the entry point emitted after a successful type check. Retain the
/// previous four-times-source output budget, without reading an unbounded file.
fn emitted_program(dir: &Scratch, limits: &Limits, deadline: Instant, cancel: &AtomicBool) -> Result<String, ApiError> {
    if cancel.load(Ordering::SeqCst) { return Err(ApiError::new("cancelled", "compilation", "compilation cancelled")); }
    if Instant::now() >= deadline { return Err(ApiError::new("resource_limit", "compilation", "compilation deadline exceeded")); }
    let maximum = limits.source_bytes.saturating_mul(4);
    let file = std::fs::File::open(dir.0.join("emitted/main.js"))
        .map_err(|e| ApiError::new("compile_failed", "compilation", format!("TypeScript did not produce main.js: {e}")))?;
    let mut bytes = Vec::new();
    std::io::Read::read_to_end(&mut std::io::Read::take(file, maximum.saturating_add(1)), &mut bytes)
        .map_err(|e| ApiError::new("compile_failed", "compilation", format!("the emitted program could not be read: {e}")))?;
    if bytes.len() as u64 > maximum {
        return Err(ApiError::new("resource_limit", "compilation", "the emitted program is too large"));
    }
    String::from_utf8(bytes).map_err(|_| ApiError::new("compile_failed", "compilation", "TypeScript's emitted program is not UTF-8"))
}

/// The hidden argument a host binary starts ITSELF with to be a code
/// worker: every shipped app is one executable, so it is its own worker.
pub const WORKER_FLAG: &str = "--brep-code-worker";

static SELF_WORKER: AtomicBool = AtomicBool::new(false);

/// Call first thing in a host binary's `main`. Started with [`WORKER_FLAG`],
/// this process IS a code worker: it runs the execution and answers true
/// (return from `main`). Otherwise it registers this executable as the
/// worker the code-mode host spawns, and answers false.
pub fn worker_entry() -> bool {
    if std::env::args().nth(1).as_deref() == Some(WORKER_FLAG) {
        brep_script_core::worker::main();
        return true;
    }
    SELF_WORKER.store(true, Ordering::Relaxed);
    false
}

/// The executable a code worker is spawned from (with [`WORKER_FLAG`]):
/// `BREP_CODE_WORKER` (any binary that calls [`worker_entry`]), else this
/// process's own executable when its `main` called [`worker_entry`].
fn worker_path() -> Option<std::path::PathBuf> {
    if let Some(p) = std::env::var_os("BREP_CODE_WORKER").map(std::path::PathBuf::from).filter(|p| p.is_file()) {
        return Some(p);
    }
    if SELF_WORKER.load(Ordering::Relaxed) {
        return std::env::current_exe().ok();
    }
    None
}

/// The worker's CPU time and resident memory, from `/proc` (Linux).
/// `None` where the platform does not say — then only the halt and the wall
/// deadline bound the worker.
fn usage(pid: u32) -> Option<(Duration, u64)> {
    let stat = std::fs::read_to_string(format!("/proc/{pid}/stat")).ok()?;
    let fields: Vec<&str> = stat.rsplit(')').next()?.split_whitespace().collect();
    let ticks = |i: usize| fields.get(i).and_then(|v| v.parse::<u64>().ok());
    let cpu = Duration::from_millis((ticks(11)? + ticks(12)?) * 10);
    let status = std::fs::read_to_string(format!("/proc/{pid}/status")).ok()?;
    let rss = status.lines().find(|l| l.starts_with("VmRSS:"))?.split_whitespace().nth(1)?.parse::<u64>().ok()? * 1024;
    Some((cpu, rss))
}

/// Include the native TypeScript compiler spawned by its trusted launcher.
fn compiler_rss(pid: u32) -> Option<u64> {
    let mut total = usage(pid)?.1;
    let mut pending = vec![pid];
    let mut seen = std::collections::HashSet::new();
    while let Some(pid) = pending.pop() {
        if !seen.insert(pid) { continue; }
        if seen.len() > 32 { return Some(u64::MAX); }
        if let Ok(children) = std::fs::read_to_string(format!("/proc/{pid}/task/{pid}/children")) {
            for child in children.split_whitespace().filter_map(|p| p.parse::<u32>().ok()) {
                total = total.saturating_add(usage(child).map_or(0, |(_, rss)| rss));
                pending.push(child);
            }
        }
    }
    Some(total)
}

#[allow(clippy::too_many_arguments)]
fn realm(
    service: &CadService,
    id: &str,
    typescript: bool,
    source: String,
    limits: Limits,
    cancel: Arc<AtomicBool>,
    bridge: tokio::sync::mpsc::Sender<BridgeCall>,
    wake_tx: mpsc::SyncSender<Wake>,
    wake: mpsc::Receiver<Wake>,
) -> Outcome {
    let wall_deadline = Instant::now() + Duration::from_millis(limits.wall_ms);
    service.code.set_state(id, "compiling");
    let program = if typescript {
        let declarations = super::declarations::module(&service.catalogue, &service.script_tools());
        let compile_deadline = (Instant::now() + Duration::from_millis(limits.compile_ms)).min(wall_deadline);
        let checked = type_check_and_emit(&source, &declarations, &limits, compile_deadline, &cancel);
        service.code.set_type_checking(id, if checked.is_ok() { "passed" } else { "failed" });
        match checked.and_then(|dir| emitted_program(&dir, &limits, compile_deadline, &cancel)) {
            Ok(js) => js,
            Err(e) => return Outcome { result: Err(e), logs: vec![] },
        }
    } else {
        source
    };
    let fail = |e: ApiError| Outcome { result: Err(e), logs: vec![] };
    let Some(exe) = worker_path() else {
        return fail(ApiError::new("internal_error", "execution", "this host cannot start a code worker: its main did not call cad::code::worker_entry (or set BREP_CODE_WORKER)"));
    };
    let mut child = match std::process::Command::new(&exe)
        .arg(WORKER_FLAG)
        .env_clear()
        .stdin(std::process::Stdio::piped())
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .spawn()
    {
        Ok(c) => c,
        Err(e) => return fail(ApiError::new("internal_error", "execution", format!("the code worker did not start: {e}"))),
    };
    let pid = child.id();
    let mut stdin = child.stdin.take().expect("piped");
    let stdout = child.stdout.take().expect("piped");
    // Pipe writes must never run on the supervisor: a worker doing compute
    // need not read stdin, but its deadline and halt must still be enforced.
    let (write_tx, write_rx) = mpsc::sync_channel::<String>(8);
    let writer = std::thread::spawn(move || {
        while let Ok(line) = write_rx.recv() {
            if writeln!(stdin, "{line}").and_then(|()| stdin.flush()).is_err() { break; }
        }
    });
    let reader = {
        let tx = wake_tx.clone();
        std::thread::spawn(move || {
            let mut input = std::io::BufReader::new(stdout);
            loop {
                let mut line = String::new();
                let read = std::io::Read::take(&mut input, (2 << 20) + 1).read_line(&mut line);
                if !matches!(read, Ok(n) if n > 0) { break; }
                if line.len() > 2 << 20 || !line.ends_with('\n') { break; }
                if tx.send(Wake::Line(line)).is_err() { break; }
            }
            let _ = tx.send(Wake::Eof);
        })
    };
    let start = json!({ "source": program, "sdk": super::sdk::module(&service.catalogue, &service.script_tools()), "manifest": service.catalogue.identity().to_string() });
    if write_tx.try_send(start.to_string()).is_err() {
        let _ = child.kill();
        let _ = child.wait();
        return fail(ApiError::new("internal_error", "execution", "the code worker closed its input"));
    }
    service.code.set_state(id, "running");

    let mut logs: Vec<String> = Vec::new();
    let (mut log_bytes, mut truncated) = (0usize, false);
    let mut outstanding: HashMap<u64, usize> = HashMap::new();
    let mut in_flight_bytes = 0u64;
    let kill = |child: &mut std::process::Child| {
        let _ = child.kill();
        let _ = child.wait();
    };
    let result = loop {
        let w = match wake.recv_timeout(Duration::from_millis(20)) {
            Ok(w) => Some(w),
            Err(mpsc::RecvTimeoutError::Timeout) => None,
            Err(mpsc::RecvTimeoutError::Disconnected) => break Err(ApiError::new("internal_error", "execution", "the host bridge closed")),
        };
        // The host's own checks, every turn: halt, wall, CPU, memory.
        if matches!(w, Some(Wake::Stop)) || cancel.load(Ordering::Relaxed) {
            kill(&mut child);
            break Err(ApiError::new("cancelled", "execution", "the execution was halted; its accepted operations continue"));
        }
        if Instant::now() > wall_deadline {
            kill(&mut child);
            break Err(ApiError::new("deadline_exceeded", "execution", format!("the execution ran longer than {} ms", limits.wall_ms)));
        }
        if let Some((cpu, rss)) = usage(pid) {
            if cpu > Duration::from_millis(limits.js_cpu_ms) {
                kill(&mut child);
                break Err(ApiError::new("resource_limit", "execution", format!("the program used more than {} ms of JS compute", limits.js_cpu_ms)));
            }
            if rss > limits.js_heap_bytes {
                kill(&mut child);
                break Err(ApiError::new("resource_limit", "execution", format!("out of memory: the program's worker passed {} bytes", limits.js_heap_bytes)));
            }
        }
        let Some(w) = w else { continue };
        match w {
            Wake::Line(line) => {
                let Ok(msg) = serde_json::from_str::<Value>(&line) else { continue };
                if let Some(text) = msg.get("log").and_then(Value::as_str) {
                    let text = wire::bounded(text.to_string(), 2048);
                    if log_bytes + text.len() <= limits.log_bytes as usize {
                        log_bytes += text.len();
                        logs.push(text);
                    } else if !truncated {
                        truncated = true;
                        logs.push("[console output truncated at the log limit]".into());
                    }
                } else if let Some(done) = msg.get("done") {
                    let _ = child.wait();
                    break match done.get("ok") {
                        Some(v) if v.to_string().len() as u64 > limits.result_bytes => {
                            Err(ApiError::new("resource_limit", "execution", format!("main's result is larger than {} bytes", limits.result_bytes)))
                        }
                        Some(v) => Ok(v.clone()),
                        None => {
                            let code = done["error"]["code"].as_str().unwrap_or("script_failed");
                            let code = match code {
                                "compile_failed" => "compile_failed",
                                "invalid_arguments" => "invalid_arguments",
                                "job_failed" => "job_failed",
                                "internal_error" => "internal_error",
                                "resource_limit" => "resource_limit",
                                _ => "script_failed",
                            };
                            let phase = if code == "compile_failed" { "compilation" } else { "execution" };
                            Err(ApiError::new(code, phase, wire::sanitize(done["error"]["message"].as_str().unwrap_or("the program failed"))))
                        }
                    };
                } else if let (Some(call), Some(method), Some(body)) =
                    (msg.get("call").and_then(Value::as_u64), msg.get("method").and_then(Value::as_str), msg.get("body").and_then(Value::as_str))
                {
                    // The bridge's bounds, before the host sees the call.
                    let refusal = if body.len() > 1 << 20 {
                        Some("a host message may be at most 1 MiB")
                    } else if in_flight_bytes + body.len() as u64 > limits.bridge_bytes {
                        Some("the bridge's resident payload limit is reached")
                    } else if outstanding.len() as u64 >= limits.outstanding_host_calls {
                        Some("too many host calls are outstanding")
                    } else {
                        None
                    };
                    let refused = match refusal {
                        Some(why) => Some(wire::fail(&ApiError::new("resource_limit", "execution", why)).to_string()),
                        None => match bridge.try_send(BridgeCall { call, method: method.to_string(), body: body.to_string() }) {
                            Ok(()) => {
                                // Mark before consuming a terminal worker message: queued
                                // host work can outlive that worker and its response.
                                if method.starts_with("tool.") || matches!(method, "cad.operations.submit" | "cad.artifacts.create" | "cad.artifacts.begin" | "cad.artifacts.append" | "cad.artifacts.commit") {
                                    if let Some(execution) = service.code.state.lock().unwrap().execs.get_mut(id) {
                                        execution.effects_possible = true;
                                    }
                                }
                                outstanding.insert(call, body.len());
                                in_flight_bytes += body.len() as u64;
                                None
                            }
                            Err(_) => Some(wire::fail(&ApiError::new("resource_limit", "execution", "the host bridge is full")).to_string()),
                        },
                    };
                    if let Some(reply) = refused {
                        if write_tx.try_send(json!({ "reply": call, "body": reply }).to_string()).is_err() {
                            break Err(ApiError::new("resource_limit", "execution", "worker reply queue is full"));
                        }
                    } else if outstanding.len() == 1 {
                        service.code.set_state(id, "waiting");
                    }
                }
            }
            Wake::Reply { call, body } => {
                if let Some(sent) = outstanding.remove(&call) {
                    in_flight_bytes = in_flight_bytes.saturating_sub(sent as u64);
                }
                let body = body.unwrap_or_else(|e| wire::fail(&ApiError::new("internal_error", "execution", e)).to_string());
                if body.len() as u64 > limits.bridge_bytes || write_tx.try_send(json!({ "reply": call, "body": body }).to_string()).is_err() {
                    break Err(ApiError::new("resource_limit", "execution", "worker reply exceeds the bridge budget or its queue is full"));
                }
                service.code.set_state(id, "running");
            }
            Wake::Eof => {
                let status = child.wait().ok();
                break Err(ApiError::new("internal_error", "execution", format!("the code worker ended without an answer ({status:?})")));
            }
            Wake::Stop => unreachable!("handled above"),
        }
    };
    // The bridge closes first: unaccepted calls are dropped, accepted
    // operations stay the service's; a late reply lands in a mailbox nobody reads.
    drop(bridge);
    kill(&mut child);
    drop(write_tx);
    drop(wake);
    let _ = writer.join();
    let _ = reader.join();
    Outcome { result, logs }
}

