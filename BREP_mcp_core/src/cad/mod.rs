//! The CAD operation service: the `cad.*` contract (owner brief 2026-10-28).
//!
//! One semantic CAD API, one operation lifecycle and one code-mode executor,
//! served to MCP clients as the `cad_*` tools and to JavaScript / TypeScript
//! programs as the `@brep/cad` module. Everything here runs over the host
//! protocol a live session already speaks, so the headless host and the
//! attached desktop window get identical semantics.
//!
//! - [`wire`]      — handles, timestamps, `ApiError`, `Effects`, `Response`.
//! - [`canon`]     — RFC 8785 canonical JSON and SHA-256.
//! - [`keys`]      — authenticated, expiring admission keys.
//! - [`catalogue`] — the command descriptors, the manifest and its hash, search/describe.
//! - [`commands`]  — what each semantic command does to the app.
//! - [`ops`]       — admission, idempotency, the request-independent operation lifetime,
//!   cancellation, retention and overload.
//! - [`artifacts`] — opaque, scoped, expiring byte objects (a STEP export, an upload).
//! - [`code`]      — code mode: Boa in a host-supervised worker process.
//! - [`sdk`]       — the `@brep/cad` module, generated from the catalogue.
//! - [`declarations`] — the SDK's TypeScript declarations, generated from the catalogue.
//! - [`diag`]      — bounded, payload-free diagnostic events.
//! - [`tools`]     — the `cad_*` MCP tools over all of the above.
pub mod artifacts;
pub mod canon;
pub mod catalogue;
pub mod code;
pub mod commands;
pub mod declarations;
pub mod diag;
pub mod event_journal;
pub mod event_archive;
pub mod journal;
pub(crate) mod inputs;
pub mod keys;
pub mod ops;
pub mod output_schema;
pub mod sdk;
pub mod tools;
pub mod wire;

use std::sync::Arc;

/// Everything the `cad_*` tools share for the life of the server process:
/// the service is process-scoped, not connection- or session-scoped, so an
/// accepted operation outlives the request that submitted it.
mod files;

pub struct CadService {
    pub files: files::Files,
    pub event_archive: std::sync::OnceLock<event_archive::Archive>,
    pub ops: ops::Operations,
    pub artifacts: artifacts::Artifacts,
    pub code: code::Executions,
    pub catalogue: catalogue::Catalogue,
    /// The server whose tool registry scripts call as `brep.<group>.<tool>`
    /// (set once by `BrepServer::new`).
    pub server: std::sync::OnceLock<std::sync::Weak<crate::server::ServerState>>,
    /// `run_script`'s retry keys: a client's key → the issued key it stands
    /// for. Here, not in the tool, because the tool set is rebuilt whenever
    /// a session starts — the first submission's own session among them.
    pub retry_keys: std::sync::Mutex<std::collections::HashMap<String, String>>,
}

pub use crate::script_sdk::{script_callable, ScriptTool};

impl CadService {
    pub fn new(slot: crate::tools::app::SessionSlot) -> Arc<Self> {
        let catalogue = catalogue::Catalogue::new();
        Arc::new_cyclic(|weak| CadService {
            files: files::Files::new(),
            event_archive: Default::default(),
            ops: ops::Operations::new(slot, weak.clone()),
            artifacts: artifacts::Artifacts::new(),
            code: code::Executions::new(weak.clone()),
            catalogue,
            server: std::sync::OnceLock::new(),
            retry_keys: Default::default(),
        })
    }

    pub fn event(&self, kind: &'static str, id: Option<&str>, state: Option<&str>, code: Option<&str>, sequence: Option<u64>) {
        let value = diag::event(kind, id, state, code, sequence);
        if let Some(archive) = self.event_archive.get() { archive.publish(value); }
    }

    pub fn archive_resource(&self, uri: &str) -> Result<serde_json::Value, String> {
        match self.event_archive.get() {
            Some(archive) => archive.resource(uri),
            None if uri == "brep://events/archive" => Ok(serde_json::json!({"enabled":false,"records":[],"persistence":{"status":"disabled"}})),
            None => Err("diagnostic archive is disabled".into()),
        }
    }

    fn server_state(&self) -> Option<Arc<crate::server::ServerState>> {
        self.server.get().and_then(std::sync::Weak::upgrade)
    }

    /// The registry tools a script may call, as of now (a live session adds
    /// the app's). Blocking: called from a code worker's host thread.
    pub fn script_tools(&self) -> Vec<ScriptTool> {
        let Some(state) = self.server_state() else { return vec![] };
        let set = state.tools.blocking_read();
        crate::script_sdk::script_tools(&set.specs)
    }

    /// One script-callable registry tool by name.
    pub async fn script_tool(&self, name: &str) -> Option<crate::tools::ToolSpec> {
        let state = self.server_state()?;
        // By name AND a script-callable group: the front door's `screenshot`
        // shares its name with the capture tool a script calls.
        let set = state.tools.read().await;
        set.specs.iter().find(|t| t.name == name && script_callable(t.group, &t.name)).cloned()
    }

    /// Search both semantic commands and application functions. The cursor
    /// binds to the exact live metadata and query, so registry changes cannot
    /// silently mix pages.
    pub async fn search(&self, req: &serde_json::Value) -> Result<serde_json::Value, wire::ApiError> {
        use serde_json::{json, Value};
        let query = req["query"].as_str().ok_or_else(|| wire::ApiError::new("invalid_arguments", "validation", "query must be a string"))?;
        if query.len() > 512 { return Err(wire::ApiError::new("invalid_arguments", "validation", "query exceeds 512 bytes")); }
        let limit = req.get("limit").map_or(Some(20), Value::as_u64).filter(|n| *n > 0 && *n <= 50).ok_or_else(|| wire::ApiError::new("invalid_arguments", "validation", "limit must be 1..50"))? as usize;
        let mut entries: Vec<Value> = self.catalogue.descriptors.iter().map(|d| d.to_json()).collect();
        if let Some(state) = self.server_state() {
            let set = state.tools.read().await;
            for t in &set.specs {
                if script_callable(t.group, &t.name) { entries.push(json!({"path":format!("brep.{}.{}", sdk::ident(t.group), crate::script_sdk::method_name(t.group, &t.name)),"summary":t.doc,"inputSchema":t.input_schema,"outputSchema":t.output_schema})); }
            }
        }
        entries.sort_by(|a,b| a["path"].as_str().cmp(&b["path"].as_str()));
        let binding = canon::sha256_hex(json!({"query":query,"entries":entries}).to_string().as_bytes());
        let start = match req.get("cursor") {
            None | Some(Value::Null) => 0,
            Some(Value::String(c)) => c.split_once(':').filter(|(_, hash)| *hash == binding).and_then(|(n,_)| n.parse::<usize>().ok()).ok_or_else(|| wire::ApiError::new("invalid_arguments", "validation", "cursor expired or belongs to another query"))?,
            _ => return Err(wire::ApiError::new("invalid_arguments", "validation", "cursor must be a string")),
        };
        let words: Vec<String> = query.split_whitespace().map(str::to_lowercase).collect();
        let mut hits: Vec<_> = entries.into_iter().filter(|v| { let text = format!("{} {}",v["path"],v["summary"]).to_lowercase(); words.iter().all(|w| text.contains(w)) }).collect();
        hits.sort_by_key(|v| {
            let path = v["path"].as_str().unwrap_or_default().to_string();
            let lower = path.to_lowercase();
            let leaf = lower.rsplit('.').next().unwrap_or_default();
            let rank = if words.iter().all(|w| leaf.contains(w)) { 0 } else if words.iter().all(|w| lower.contains(w)) { 1 } else { 2 };
            (rank, path)
        });
        let methods: Vec<_> = hits.iter().skip(start).take(limit).map(|v| json!({"path":v["path"],"summary":v["summary"]})).collect();
        let next = start + methods.len();
        Ok(json!({"manifest":self.catalogue.identity(),"methods":methods,"nextCursor":if next < hits.len() { json!(format!("{next}:{binding}")) } else { Value::Null }}))
    }

    pub async fn describe(&self, paths: &[String]) -> Result<serde_json::Value, wire::ApiError> {
        use serde_json::json;
        if paths.is_empty() || paths.len() > 8 { return Err(wire::ApiError::new("invalid_arguments", "validation", "describe takes 1..8 paths")); }
        let mut methods = Vec::new();
        for path in paths {
            if let Some(d) = self.catalogue.get(path) { methods.push(d.to_json()); continue; }
            let state = self.server_state().ok_or_else(|| wire::ApiError::new("method_unknown", "lookup", "no application registry"))?;
            let set = state.tools.read().await;
            let t = set.specs.iter().find(|t| script_callable(t.group, &t.name) && format!("brep.{}.{}",sdk::ident(t.group),crate::script_sdk::method_name(t.group, &t.name)) == *path)
                .ok_or_else(|| wire::ApiError::new("method_unknown", "lookup", format!("no function {path}")))?;
            methods.push(json!({"path":path,"summary":t.doc,"documentation":t.doc,"inputSchema":t.input_schema,"outputSchema":t.output_schema,"readOnly":t.annotations.read_only,"destructive":t.annotations.destructive}));
        }
        let result = json!({"manifest":self.catalogue.identity(),"methods":methods});
        if result.to_string().len() > 64 << 10 { return Err(wire::ApiError::new("resource_limit", "validation", "describe fewer paths")); }
        Ok(result)
    }

    /// Run a program for the app itself (its JavaScript editor): the local
    /// user, trusted as the MCP's `run_script` is — every app tool, no
    /// execution control. Must be called inside the server's tokio runtime
    /// (`Handle::enter`); returns at once with the execution id.
    pub fn run_local(&self, source: &str, language: &str) -> Result<String, String> {
        let caller = tools::caller_of_transport();
        let (key, _) = keys::issue(&caller.scope, keys::Purpose::Execution);
        let req = serde_json::json!({
            "manifest": self.catalogue.identity(),
            "language": language,
            "source": source,
            "initialDocuments": [],
            "requestedCapabilities": [],
            "idempotencyKey": key,
        });
        let status = self.code.submit_as(&caller, &req, true).map_err(|e| e.message)?;
        Ok(status["executionId"].as_str().unwrap_or_default().to_string())
    }

    /// A [`Self::run_local`] execution's result once it has ended (`None`
    /// while it runs), with its console lines as `logText`.
    pub fn result_local(&self, id: &str) -> Result<Option<serde_json::Value>, String> {
        let caller = tools::caller_of_transport();
        match self.code.result(&caller, &serde_json::json!({ "executionId": id })) {
            Ok(mut result) => {
                let log = result["logArtifact"]["artifactId"].as_str().map(str::to_string);
                if let Some(log) = log {
                    if let Ok((bytes, _)) = self.artifacts.lease(&caller.owner(), &log) {
                        self.artifacts.release(&log);
                        result["logText"] = String::from_utf8_lossy(&bytes).into_owned().into();
                    }
                }
                Ok(Some(result))
            }
            Err(e) if e.code == "not_ready" => Ok(None),
            Err(e) => Err(e.message),
        }
    }

    /// Halt a [`Self::run_local`] execution: its worker is killed.
    pub fn halt_local(&self, id: &str) -> Result<(), String> {
        let caller = tools::caller_of_transport();
        self.code.cancel(&caller, &serde_json::json!({ "executionId": id })).map(|_| ()).map_err(|e| e.message)
    }
}

/// The app's own handle on code mode, for its JavaScript editor: the
/// server's [`CadService`] and the runtime it runs on, so a frame can submit,
/// poll and halt without awaiting anything.
#[derive(Clone)]
pub struct LocalScripts {
    cad: std::sync::Weak<CadService>,
    runtime: tokio::runtime::Handle,
}

impl std::fmt::Debug for LocalScripts {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("LocalScripts")
    }
}

impl LocalScripts {
    pub fn new(cad: &Arc<CadService>, runtime: tokio::runtime::Handle) -> Self {
        LocalScripts { cad: Arc::downgrade(cad), runtime }
    }

    fn cad(&self) -> Result<Arc<CadService>, String> {
        self.cad.upgrade().ok_or_else(|| "the script runner has stopped".to_string())
    }

    /// Whether the app's tools are registered (a program's `brep.*` is
    /// generated from them at submit). Never blocks.
    pub fn ready(&self) -> Result<bool, String> {
        let state = self.cad()?.server_state().ok_or("the server is stopping")?;
        Ok(state.tools.try_read().is_ok_and(|set| set.specs.iter().any(|t| t.name == "docs_list")))
    }

    pub fn run(&self, source: &str) -> Result<String, String> {
        let _enter = self.runtime.enter();
        self.cad()?.run_local(source, "javascript")
    }

    pub fn result(&self, id: &str) -> Result<Option<serde_json::Value>, String> {
        self.cad()?.result_local(id)
    }

    pub fn halt(&self, id: &str) -> Result<(), String> {
        let _enter = self.runtime.enter();
        self.cad()?.halt_local(id)
    }
}
