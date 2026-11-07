//! The operation service (contract §7): admission, idempotency, a lifetime
//! that belongs to the service rather than to the request or realm that
//! submitted it, cooperative cancellation, bounded retention and overload.
//!
//! An accepted operation is a tokio task the SERVICE owns: a caller that
//! disconnects, a transport request that times out, or a code-mode realm
//! that is torn down never cancels it. Its outcome is retained for 30
//! minutes after it ends (or until its key's first-admission window closes,
//! whichever is later) and is then forgotten honestly: `operation_unknown`
//! with unknown effects.
use super::artifacts::Owner;
use super::canon;
use super::commands;
use super::keys::{self, Purpose};
use super::wire::{self, ApiError, Effects};
use super::CadService;
use crate::tools::app::{current, SessionSlot};
use serde_json::{json, Value};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, Weak};
use std::time::{Duration, Instant};

pub const MAX_NONTERMINAL: usize = 64;
pub const MAX_PER_SCOPE: usize = 16;
pub const MAX_PER_DOCUMENT: usize = 4;
pub const MAX_RETAINED: usize = 1024;
pub const RETENTION: Duration = Duration::from_secs(30 * 60);
pub const QUEUE_DEADLINE: Duration = Duration::from_secs(30);
const MAX_RETAINED_BYTES: usize = 64 << 20;

/// Every capability name of the contract (§11).
pub const CAPABILITIES: &[&str] = &[
    "catalogue.read", "document.create", "document.read", "document.write", "document.close", "view.read", "view.write",
    "artifact.read", "artifact.write", "cad.export", "cad.import", "operation.read", "operation.cancel", "execution.read",
    "execution.cancel", "code.execute",
    // Call the app's tools (the whole MCP tool registry) as script functions.
    "app.tools",
    // Read a file on the host into an artifact (`cad.artifacts.create` with
    // `path`): the local user's own files, as `document_import` reads them.
    // Never granted to a restricted program.
    "host.files.read", "host.files.write",
];

/// What a caller may do: the effective grants after intersection with host
/// policy. `documents` is `None` for "any document" (the trusted local
/// profile) or the explicit set a restricted execution may touch.
#[derive(Debug, Clone)]
pub struct Grants {
    pub names: Vec<String>,
    /// Existing documents this caller may name (None = any).
    pub documents: Option<Arc<Mutex<Vec<String>>>>,
    /// Whether documents the caller creates join `documents`.
    pub created_documents: bool,
    pub document_grants: Option<Arc<Mutex<HashMap<String, Vec<String>>>>>,
    pub created_capabilities: Vec<String>,
}

impl Grants {
    /// The trusted local profile of a direct MCP client: every capability,
    /// every document. (The transport admits only the local user, D3: the one
    /// place they would narrow this is `Caller::local`.)
    pub fn trusted() -> Self {
        Grants { names: CAPABILITIES.iter().map(|s| s.to_string()).collect(), documents: None, created_documents: true, document_grants: None, created_capabilities: vec![] }
    }
    pub fn has(&self, name: &str) -> bool {
        self.names.iter().any(|n| n == name)
    }
    pub fn may_touch(&self, document: &str) -> bool {
        match &self.documents {
            None => true,
            Some(list) => list.lock().unwrap().iter().any(|d| d == document),
        }
    }
    pub fn may_use(&self, capability: &str, document: &str) -> bool {
        // Artifact and operation grants have their own ownership scope; they
        // do not carry documentIds and must not be interpreted as empty lists.
        if !matches!(capability, "document.read" | "document.write" | "document.close" | "cad.import" | "cad.export") { return self.has(capability); }
        self.document_grants.as_ref().map_or_else(|| self.may_touch(document), |grants| grants.lock().unwrap().get(capability).is_some_and(|ids| ids.iter().any(|id| id == document)))
    }
    pub fn add_created(&self, document: &str) {
        if let Some(grants) = &self.document_grants {
            let mut grants = grants.lock().unwrap();
            for cap in &self.created_capabilities { grants.entry(cap.clone()).or_default().push(document.into()); }
        }
        if self.created_documents {
            if let Some(list) = &self.documents {
                list.lock().unwrap().push(document.to_string());
            }
        }
    }
}

/// Who is calling: the authorization scope and actor (host-established,
/// never from the request), the execution if the call comes through a
/// code-mode bridge, and the effective grants.
#[derive(Debug, Clone)]
pub struct Caller {
    pub scope: String,
    pub actor: String,
    pub execution: Option<String>,
    pub grants: Grants,
}

impl Caller {
    /// The direct MCP client of this local server (one scope, one actor).
    pub fn local() -> Self {
        Caller { scope: "local".into(), actor: "local".into(), execution: None, grants: Grants::trusted() }
    }
    pub fn owner(&self) -> Owner {
        Owner { scope: self.scope.clone(), execution: self.execution.clone() }
    }
    pub fn require(&self, names: &[&str]) -> Result<(), ApiError> {
        match names.iter().find(|n| !self.grants.has(n)) {
            Some(missing) => Err(ApiError::new("capability_denied", "authorization", format!("this caller lacks `{missing}`"))),
            None => Ok(()),
        }
    }
}

struct Op {
    id: String,
    scope: String,
    actor: String,
    execution: Option<String>,
    command: String,
    envelope: String,
    document: Option<String>,
    context: Option<Value>,
    state: &'static str,
    sequence: u64,
    stage: String,
    summary: String,
    cancel: Arc<AtomicBool>,
    accepted_at: Instant,
    key_deadline: Instant,
    finished_at: Option<Instant>,
    retain_until: Option<Instant>,
    result: Option<Value>,
    bytes: usize,
}

impl Op {
    fn terminal(&self) -> bool {
        matches!(self.state, "succeeded" | "failed" | "cancelled")
    }

    fn retention(&self) -> Value {
        json!({
            "scope": "process_epoch",
            "epoch": wire::epoch(),
            "retainUntil": self.retain_until.map(wire::timestamp),
            "terminalRetentionMs": RETENTION.as_millis() as u64,
            "artifactRetentionMs": super::artifacts::AGE.as_millis() as u64,
        })
    }

    fn admission(&self) -> Value {
        json!({
            "operationId": self.id,
            "acceptedAt": wire::timestamp(self.accepted_at),
            "context": self.context,
            "state": self.state,
            "sequence": self.sequence.to_string(),
            "retention": self.retention(),
        })
    }

    fn status(&self) -> Value {
        let mut v = self.admission();
        let requested = self.cancel.load(Ordering::Relaxed);
        v["command"] = json!(self.command);
        v["cancellation"] = json!({
            "requested": requested,
            "effectiveNow": requested && matches!(self.state, "queued"),
            // An evaluation is cancelled through the app's run cancel; only a
            // commit in progress is past stopping.
            "nonPreemptible": matches!(self.state, "committing"),
        });
        v["progress"] = json!({ "stage": self.stage, "completed": null, "total": null, "summary": self.summary });
        v
    }
}

#[derive(Default)]
struct State {
    ops: HashMap<String, Op>,
    /// (scope, purpose, key nonce) → operation id.
    bindings: HashMap<(String, Purpose, String), String>,
    /// One stage lease per document: operations on a document run in order.
    leases: HashMap<String, Arc<tokio::sync::Mutex<()>>>,
}

impl State {
    fn sweep(&mut self) {
        let now = Instant::now();
        let gone: Vec<String> = self.ops.values().filter(|o| o.retain_until.is_some_and(|t| t <= now)).map(|o| o.id.clone()).collect();
        for id in &gone {
            self.ops.remove(id);
        }
        if !gone.is_empty() {
            self.bindings.retain(|_, op| !gone.contains(op));
        }
    }
    fn nonterminal(&self) -> impl Iterator<Item = &Op> {
        self.ops.values().filter(|o| !o.terminal())
    }
}

pub struct Operations {
    slot: SessionSlot,
    service: Weak<CadService>,
    state: Mutex<State>,
}

fn field<'a>(req: &'a Value, key: &str) -> Option<&'a Value> {
    req.get(key).filter(|v| !v.is_null())
}

fn reject_unknown(req: &Value, allowed: &[&str]) -> Result<(), ApiError> {
    let obj = req.as_object().ok_or_else(|| ApiError::new("invalid_arguments", "validation", "the request must be an object"))?;
    if let Some(k) = obj.keys().find(|k| !allowed.contains(&k.as_str())) {
        return Err(ApiError::new("invalid_arguments", "validation", format!("unknown field `{k}`")));
    }
    Ok(())
}

impl Operations {
    fn event(&self, kind: &'static str, id: Option<&str>, state: Option<&str>, code: Option<&str>, sequence: Option<u64>) {
        if let Some(service) = self.service.upgrade() { service.event(kind, id, state, code, sequence); }
    }

    pub fn new(slot: SessionSlot, service: Weak<CadService>) -> Self {
        Operations { slot, service, state: Mutex::new(State::default()) }
    }

    pub fn issue_key(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["purpose"])?;
        let purpose = req.get("purpose").and_then(Value::as_str).and_then(Purpose::parse).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "purpose must be \"command\" or \"execution\""))?;
        match purpose {
            Purpose::Command => caller.require(&["operation.read"])?,
            Purpose::Execution => caller.require(&["code.execute", "execution.read"])?,
        }
        let (key, expires) = keys::issue(&caller.scope, purpose);
        Ok(json!({ "key": key, "firstAdmissionExpiresAt": expires }))
    }

    /// `cad.operations.submit` (§7.1, in its mandatory order).
    pub fn submit(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        self.event("admission_received", None, None, None, None);
        let r = self.submit_inner(caller, req);
        match &r {
            Ok(a) => self.event("operation_accepted", a["operationId"].as_str(), a["state"].as_str(), None, None),
            Err(e) => self.event("admission_rejected", e.operation_id.as_deref(), Some(e.phase), Some(e.code), None),
        }
        r
    }

    fn submit_inner(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["manifest", "command", "args", "context", "targets", "idempotencyKey"])?;
        let key_text = req.get("idempotencyKey").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "idempotencyKey is required (cad_issue_key)"))?;
        // 1. authenticate the key for this scope and purpose.
        let key = keys::verify(key_text, &caller.scope, Purpose::Command)?;
        let command = req.get("command").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "command is required"))?.to_string();
        if field(req, "targets").is_some_and(|t| t.as_array().is_none_or(|a| !a.is_empty())) {
            return Err(ApiError::new("invalid_arguments", "validation", "no version-1 command declares explicit_many targets"));
        }
        let args = req.get("args").cloned().unwrap_or(json!({}));
        let context = field(req, "context").cloned();
        // The canonical original envelope, defaulted by the command's own
        // (static) contract: what a retry is compared against.
        let defaulted = commands::validate(&command, &args).unwrap_or(args.clone());
        let envelope = canon::canonical(&json!({
            "manifest": req.get("manifest"),
            "command": command,
            "args": defaulted,
            "context": context,
        }))
        .map_err(|e| ApiError::new("invalid_arguments", "validation", e))?;

        let mut st = self.state.lock().unwrap();
        st.sweep();
        // 2–3. a retained binding answers the retry, whatever has changed since.
        let binding = (caller.scope.clone(), Purpose::Command, key.nonce.clone());
        if let Some(op_id) = st.bindings.get(&binding).cloned() {
            caller.require(&["operation.read"])?;
            let op = st.ops.get(&op_id).expect("a binding names a retained operation");
            if op.actor != caller.actor && !caller.grants.has("operation.read") {
                return Err(ApiError::new("capability_denied", "authorization", "not this caller's operation"));
            }
            if op.envelope != envelope {
                return Err(ApiError::new("idempotency_conflict", "admission", "this key was used for a different request").op(&op_id));
            }
            return Ok(op.admission());
        }
        // 4. first admission only.
        if key.expired() {
            return Err(ApiError::new("idempotency_expired", "admission", "the key's first-admission window has closed; issue a new key after inspecting what happened").effects(Effects::unknown()));
        }
        let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "admission", "the service is stopping"))?;
        service.catalogue.check_manifest(req.get("manifest").unwrap_or(&Value::Null))?;
        let descriptor = service.catalogue.get(&command).ok_or_else(|| ApiError::new("method_unknown", "lookup", format!("no method `{command}`")))?;
        if descriptor.execution != "operation" {
            return Err(ApiError::new("invalid_arguments", "validation", format!("`{command}` is a query: use cad_query")));
        }
        caller.require(descriptor.required_capabilities)?;
        caller.require(&["operation.read"])?;
        let defaulted = commands::validate(&command, &args)?;
        let bound = match (&context, descriptor.document_scope) {
            (None, "none" | "creates_document") => None,
            (Some(_), "none" | "creates_document") => return Err(ApiError::new("invalid_arguments", "validation", format!("`{command}` takes a null context"))),
            (None, _) => return Err(ApiError::new("invalid_arguments", "validation", format!("`{command}` needs a document context"))),
            (Some(c), _) => Some(wire::document_context(c)?),
        };
        if let Some((doc, _)) = &bound {
            if !descriptor.required_capabilities.iter().all(|cap| caller.grants.may_use(cap, doc)) {
                return Err(ApiError::new("capability_denied", "authorization", "this caller may not touch that document"));
            }
        }
        // One outstanding mutation per document per execution (§8.1, §12):
        // a program's second write to a document before the first has ended is
        // refused here, whether it came through the SDK or `service` directly.
        if let (Some(exec), Some((doc, _))) = (&caller.execution, &bound) {
            if descriptor.effect_classes.contains(&"document_write") || descriptor.effect_classes.contains(&"document_lifecycle") {
                let busy = st.nonterminal().any(|o| o.execution.as_deref() == Some(exec.as_str()) && o.document.as_deref() == Some(doc.as_str()));
                if busy {
                    return Err(ApiError::new("document_busy", "admission", "this execution already has a write to that document outstanding; await it first"));
                }
            }
        }
        // A close with work in flight on its document answers at once and the
        // document stays open (§5.2): the caller cancels or waits, then retries.
        if command == "cad.documents.close" {
            if let Some((doc, _)) = &bound {
                if st.nonterminal().any(|o| o.document.as_deref() == Some(doc.as_str())) {
                    return Err(ApiError::new("document_busy", "admission", "an operation on that document is still in flight; cancel or await it, then close"));
                }
            }
        }
        // No live app session (stopped, restarting or never started): refused
        // before acceptance, so nothing is queued against a host that is gone.
        if self.slot.try_read().map(|g| g.is_none()).unwrap_or(false) {
            return Err(ApiError::new("host_stopped", "admission", "no app session is running: run_script starts one, or attach the app"));
        }
        // Capacity: reserve before acceptance, never evict nonterminal work.
        let total = st.nonterminal().count();
        let scoped = st.nonterminal().filter(|o| o.scope == caller.scope).count();
        let per_doc = bound.as_ref().map_or(0, |(d, _)| st.nonterminal().filter(|o| o.document.as_deref() == Some(d)).count());
        let retained = st.ops.len();
        let bytes: usize = st.ops.values().map(|o| o.bytes).sum();
        let reserve = envelope.len() + (64 << 10);
        if total >= MAX_NONTERMINAL || scoped >= MAX_PER_SCOPE || per_doc >= MAX_PER_DOCUMENT || retained >= MAX_RETAINED || bytes + reserve > MAX_RETAINED_BYTES {
            return Err(ApiError::new("overloaded", "admission", "the service is at capacity; retry the same key shortly").retry("retry_same_key_within_retention"));
        }
        let id = wire::mint("op");
        let cancel = Arc::new(AtomicBool::new(false));
        let op = Op {
            id: id.clone(),
            scope: caller.scope.clone(),
            actor: caller.actor.clone(),
            execution: caller.execution.clone(),
            command: command.clone(),
            envelope,
            document: bound.as_ref().map(|(d, _)| d.clone()),
            context: context.clone(),
            state: "queued",
            sequence: 1,
            stage: "queued".into(),
            summary: String::new(),
            cancel: cancel.clone(),
            accepted_at: Instant::now(),
            key_deadline: key.deadline,
            finished_at: None,
            retain_until: None,
            result: None,
            bytes: reserve,
        };
        let admission = op.admission();
        let lease = bound.as_ref().map(|(d, _)| st.leases.entry(d.clone()).or_default().clone());
        st.ops.insert(id.clone(), op);
        st.bindings.insert(binding, id.clone());
        drop(st);
        // Published before dispatch: a lost reply is recovered by the same key.
        let me = service.clone();
        let caller = caller.clone();
        // An execution's operation is its client's: see `host::with_client`.
        let client = caller.execution.as_deref().map(crate::host::client_of);
        tokio::spawn(crate::host::with_client(client, async move { me.ops.drive(me.clone(), id, command, defaulted, bound, lease, cancel, caller).await }));
        Ok(admission)
    }

    /// Cancellation and admission use the same state lock: a queued cancellation
    /// cannot be acknowledged after the runner has passed its cancellation check.
    fn start(&self, id: &str) -> bool {
        let mut state = self.state.lock().unwrap();
        let Some(op) = state.ops.get_mut(id) else { return false };
        if op.state != "queued" || op.cancel.load(Ordering::Relaxed) { return false; }
        op.state = "running"; op.stage = "running".into(); op.sequence += 1;
        true
    }

    /// Cross the last cancellable boundary for nonpreemptible host commands.
    fn begin_commit(&self, id: &str) -> Result<(), ApiError> {
        let mut state = self.state.lock().unwrap();
        let op = state.ops.get_mut(id).ok_or_else(|| ApiError::new("operation_unknown", "commit", "the operation is no longer available").effects(Effects::unknown()))?;
        if op.terminal() { return Err(ApiError::new("operation_unknown", "commit", "the operation is already terminal").effects(Effects::unknown())); }
        if op.cancel.load(Ordering::Relaxed) { return Err(ApiError::new("cancelled", "commit", "cancelled before host dispatch")); }
        op.state = "committing"; op.stage = "committing".into(); op.sequence += 1;
        Ok(())
    }

    fn update(&self, id: &str, f: impl FnOnce(&mut Op)) {
        if let Some(op) = self.state.lock().unwrap().ops.get_mut(id) {
            if !op.terminal() {
                f(op);
                op.sequence += 1;
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    async fn drive(
        &self,
        service: Arc<CadService>,
        id: String,
        command: String,
        args: Value,
        bound: Option<(String, String)>,
        lease: Option<Arc<tokio::sync::Mutex<()>>>,
        cancel: Arc<AtomicBool>,
        caller: Caller,
    ) {
        // Queue: wait for the document's stage lease, at most QUEUE_DEADLINE.
        self.event("queue_enter", Some(&id), Some("queued"), None, None);
        let _guard = match &lease {
            Some(lease) => match tokio::time::timeout(QUEUE_DEADLINE, lease.clone().lock_owned()).await {
                Ok(guard) => Some(guard),
                Err(_) => {
                    let e = ApiError::new("deadline_exceeded", "queue", "the operation waited 30 s for its document and never started");
                    return self.finish(&id, "failed", None, Effects::none(), json!({ "status": "not_applicable", "report": null, "summary": "never started" }), Some(e));
                }
            },
            None => None,
        };
        if !self.start(&id) {
            return self.finish(&id, "cancelled", None, Effects::none(), json!({ "status": "not_applicable", "report": null, "summary": "cancelled before it started" }), Some(ApiError::new("cancelled", "queue", "cancelled before it started")));
        }
        self.event("queue_leave", Some(&id), Some("running"), None, None);
        let session = match current(&self.slot).await {
            Ok(s) => s,
            Err(e) => {
                return self.finish(&id, "failed", None, Effects::none(), json!({ "status": "not_applicable", "report": null, "summary": "no session" }), Some(ApiError::new("host_stopped", "running", e)));
            }
        };
        let progress_id = id.clone();
        let svc = service.clone();
        let progress = move |stage: &str| {
            if stage == "evaluating" {
                svc.event("evaluation_begin", Some(&progress_id), Some("evaluating"), None, None);
            }
            svc.ops.update(&progress_id, |op| {
                if stage == "evaluating" {
                    op.state = "evaluating";
                }
                op.stage = stage.to_string();
                op.summary = stage.to_string();
            })
        };
        let begin_commit = || service.ops.begin_commit(&id);
        let created_document = |id: &str| caller.grants.add_created(id);
        let run = commands::Run { session, owner: caller.owner(), artifacts: &service.artifacts, cancel: &cancel, progress: &progress, begin_commit: &begin_commit, created_document: &created_document };
        match commands::execute(&command, &args, bound, &run).await {
            Ok(done) => {
                self.finish(&id, "succeeded", Some(done.output), done.effects, done.geometry, None)
            }
            Err(e) => {
                let state = if e.code == "cancelled" { "cancelled" } else { "failed" };
                let geometry = json!({
                    "status": if e.code == "geometry_refused" { "refused" } else if e.effects.disposition == "unknown" { "unknown" } else { "not_applicable" },
                    "report": null,
                    "summary": e.message,
                });
                self.finish(&id, state, None, e.effects.clone(), geometry, Some(e))
            }
        }
    }

    fn finish(&self, id: &str, state: &'static str, output: Option<Value>, effects: Effects, geometry: Value, error: Option<ApiError>) {
        let mut st = self.state.lock().unwrap();
        let Some(op) = st.ops.get_mut(id) else { return };
        if op.terminal() {
            return;
        }
        let now = Instant::now();
        op.state = state;
        op.sequence += 1;
        op.stage = state.to_string();
        op.finished_at = Some(now);
        op.retain_until = Some((now + RETENTION).max(op.key_deadline));
        let error = error.map(|e| e.op(id).to_json());
        let result = json!({
            "operationId": id,
            "state": state,
            "sequence": op.sequence.to_string(),
            "accepted": true,
            "finishedAt": wire::timestamp(now),
            "effects": effects.to_json(),
            "geometry": geometry,
            "output": output,
            "error": error,
            "retention": op.retention(),
        });
        op.bytes = op.envelope.len() + result.to_string().len().min(64 << 10);
        self.event("operation_terminal", Some(id), Some(state), result["error"]["code"].as_str(), Some(op.sequence));
        op.result = Some(result);
    }

    fn lookup<'a>(st: &'a mut State, caller: &Caller, id: &str) -> Result<&'a mut Op, ApiError> {
        caller.require(&["operation.read"])?;
        match wire::handle_shape("op", id) {
            wire::HandleShape::Current => {}
            wire::HandleShape::OtherEpoch => {
                return Err(ApiError::new("handle_epoch_mismatch", "lookup", "the operation is from an earlier run of the server; its effects are unknown").effects(Effects::unknown()))
            }
            wire::HandleShape::Malformed => return Err(ApiError::new("invalid_arguments", "validation", "operationId is not an operation handle")),
        }
        st.sweep();
        let unknown = || ApiError::new("operation_unknown", "lookup", "no retained operation has that id (never accepted, or its record expired)").effects(Effects::unknown());
        let op = st.ops.get_mut(id).ok_or_else(unknown)?;
        // `own` control: the actor's own records in its scope; a restricted
        // execution sees only the operations its own bridge accepted.
        if op.scope != caller.scope || op.actor != caller.actor || (caller.execution.is_some() && op.execution != caller.execution) {
            return Err(unknown());
        }
        Ok(op)
    }

    pub fn status(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["operationId", "afterSequence"])?;
        let id = req.get("operationId").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "operationId is required"))?;
        let after = match field(req, "afterSequence") {
            None => None,
            Some(v) => Some(v.as_str().and_then(wire::parse_revision).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "afterSequence must be a decimal string"))?),
        };
        let mut st = self.state.lock().unwrap();
        let op = Self::lookup(&mut st, caller, id)?;
        Ok(json!({ "changed": after.is_none_or(|a| op.sequence > a), "status": op.status() }))
    }

    pub fn result(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["operationId"])?;
        let id = req.get("operationId").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "operationId is required"))?;
        let mut st = self.state.lock().unwrap();
        let op = Self::lookup(&mut st, caller, id)?;
        op.result.clone().ok_or_else(|| ApiError::new("not_ready", "lookup", format!("the operation is {}", op.state)).op(id))
    }

    pub fn cancel(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["operationId"])?;
        caller.require(&["operation.cancel"])?;
        let id = req.get("operationId").and_then(Value::as_str).ok_or_else(|| ApiError::new("invalid_arguments", "validation", "operationId is required"))?;
        let mut st = self.state.lock().unwrap();
        let op = Self::lookup(&mut st, caller, id)?;
        let disposition = match op.state {
            "succeeded" | "failed" | "cancelled" => "already_terminal",
            "committing" => "too_late",
            "queued" => {
                if !op.cancel.swap(true, Ordering::Relaxed) { op.sequence += 1; }
                // The task notices before it starts and records the terminal
                // outcome itself (no effects).
                "cancelled_before_start"
            }
            _ => {
                if !op.cancel.swap(true, Ordering::Relaxed) {
                    op.sequence += 1;
                }
                "requested"
            }
        };
        self.event("cancel_requested", Some(id), Some(op.state), Some(disposition), Some(op.sequence));
        Ok(json!({ "operationId": id, "disposition": disposition, "status": op.status() }))
    }

    /// `cad.commands.query`: a query command, run inline (an ordered read).
    pub async fn query(&self, caller: &Caller, req: &Value) -> Result<Value, ApiError> {
        reject_unknown(req, &["manifest", "command", "args", "context"])?;
        let service = self.service.upgrade().ok_or_else(|| ApiError::new("host_stopping", "admission", "the service is stopping"))?;
        service.catalogue.check_manifest(req.get("manifest").unwrap_or(&Value::Null))?;
        let command = req.get("command").and_then(Value::as_str).unwrap_or_default();
        let descriptor = service.catalogue.get(command).ok_or_else(|| ApiError::new("method_unknown", "lookup", format!("no method `{command}`")))?;
        if descriptor.execution != "query" {
            return Err(ApiError::new("invalid_arguments", "validation", format!("`{command}` is an operation: submit it with cad.operations.submit")));
        }
        caller.require(descriptor.required_capabilities)?;
        commands::validate(command, &req.get("args").cloned().unwrap_or(json!({})))?;
        let context = wire::document_context(field(req, "context").ok_or_else(|| ApiError::new("invalid_arguments", "validation", "a document context is required"))?)?;
        if !caller.grants.may_use("document.read", &context.0) {
            return Err(ApiError::new("capability_denied", "authorization", "this caller may not read that document"));
        }
        let session = current(&self.slot).await.map_err(|e| ApiError::new("host_stopped", "running", e))?;
        let output = commands::inspect(context, &session).await?;
        Ok(json!({ "context": output["context"], "output": output }))
    }

    /// `cad.documents.list` / `get`: current ordered state, filtered by read access.
    pub async fn documents(&self, caller: &Caller, only: Option<&str>) -> Result<Value, ApiError> {
        caller.require(&["document.read"])?;
        let session = current(&self.slot).await.map_err(|e| ApiError::new("host_stopped", "running", e))?;
        let r = session.host.call_ok("docs_list", json!({})).await.map_err(|e| ApiError::new("internal_error", "running", wire::sanitize(&e)))?;
        let tabs = r.result.unwrap_or(Value::Null);
        let busy: Vec<String> = self.state.lock().unwrap().nonterminal().filter_map(|o| o.document.clone()).collect();
        let infos: Vec<Value> = tabs["tabs"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|t| t["id"].as_str().is_some_and(|d| caller.grants.may_use("document.read", d)))
            .filter(|t| only.is_none_or(|id| t["id"] == id))
            .map(|t| {
                let id = t["id"].as_str().unwrap_or_default();
                json!({
                    "context": wire::context_json(id, t["revision"].as_str().unwrap_or("0")),
                    "title": t["title"],
                    "lifecycle": "open",
                    "dirty": t["dirty"],
                    "busy": busy.iter().any(|b| b == id),
                })
            })
            .collect();
        match only {
            Some(_) => infos.into_iter().next().ok_or_else(|| ApiError::new("document_unknown", "lookup", "no open document with that handle is visible to this caller")),
            None => Ok(json!({ "documents": infos })),
        }
    }

    /// Accepted operation ids of an execution (for its summary), with states.
    pub fn of_execution(&self, execution: &str) -> Vec<Value> {
        let st = self.state.lock().unwrap();
        let mut ops: Vec<&Op> = st.ops.values().filter(|o| o.execution.as_deref() == Some(execution)).collect();
        ops.sort_by_key(|o| o.accepted_at);
        ops.iter()
            .map(|o| {
                json!({
                    "operationId": o.id,
                    "state": o.state,
                    "effects": o.result.as_ref().map_or(Effects::unknown().to_json(), |r| r["effects"].clone()),
                    "retainUntil": o.retain_until.map(wire::timestamp),
                })
            })
            .collect()
    }
}

