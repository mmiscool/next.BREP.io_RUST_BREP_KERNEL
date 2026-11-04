//! The dynamic `rmcp::ServerHandler`.
//!
//! rmcp's `#[tool]` / `#[tool_router]` macros build a static tool table at
//! compile time, which is exactly what this server must not have: its tools
//! come from registries read at session start. So the handler implements the
//! dynamic side of the trait by hand — `tools/list` and `tools/call` walk a
//! [`ToolSet`], `resources/list` and `resources/read` walk a [`ResourceSet`] —
//! and advertises `tools.listChanged` so a session can swap the set.
use crate::host::Backend;
use crate::tools::{ToolSet, ToolSpec};
use base64::Engine as _;
use rmcp::model::{
    CacheScope, CallToolRequestParams, CallToolResponse, CallToolResult, ContentBlock, Implementation,
    ListResourcesResult, ListToolsResult, PaginatedRequestParams, ProtocolVersion, ReadResourceRequestParams,
    ReadResourceResponse, ReadResourceResult, Resource, ResourceContents, ServerCapabilities, ServerInfo,
    SubscriptionFilter, Tool, ToolAnnotations,
};
use rmcp::service::{NotificationContext, RequestContext, RoleServer, SubscriptionContext, SubscriptionSink};
use rmcp::{ErrorData as McpError, ServerHandler, ServiceExt};
use serde_json::{Map, Value};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Mutex;
use std::{future::Future, sync::Arc};
use tokio::sync::RwLock;

/// Delegate version negotiation to the SDK while narrowing legacy discovery.
struct InitializationInfo(ServerInfo);
impl ServerHandler for InitializationInfo {
    fn get_info(&self) -> ServerInfo { self.0.clone() }
}

/// A resource the server can list and read. Like tools, resources are produced
/// by walking registries; this is only the shape they arrive in.
#[derive(Clone)]
pub struct ResourceSpec {
    pub uri: String,
    pub name: String,
    pub description: String,
    pub mime: &'static str,
    pub read: Arc<dyn Fn() -> Result<String, String> + Send + Sync>,
}

#[derive(Default, Clone)]
pub struct ResourceSet {
    pub specs: Vec<ResourceSpec>,
}

impl ResourceSet {
    pub fn get(&self, uri: &str) -> Option<&ResourceSpec> {
        self.specs.iter().find(|r| r.uri == uri)
    }
}

/// Resources that need no app session: the feature catalogue as JSON Schema
/// and the script format. Everything else arrives with a session's registries.
pub fn engine_resources() -> ResourceSet {
    let mut specs = vec![
        ResourceSpec {
            uri: "brep://api/commands".into(), name: "application-functions".into(),
            description: "Live application functions: groups, descriptions, argument and result schemas. Read sdk.d.ts for typed bindings.".into(),
            mime: "application/json", read: Arc::new(|| Ok("{}".into())),
        },
        ResourceSpec {
            uri: "brep://schema/features".into(),
            name: "feature-schemas".into(),
            description: "Every feature in the kernel catalogue with its inputParams JSON Schema and defaults".into(),
            mime: "application/json",
            read: Arc::new(|| serde_json::to_string_pretty(&crate::schema::all()).map_err(|e| e.to_string())),
        },
        ResourceSpec {
            uri: "brep://schema/sketch".into(),
            name: "sketch-persistent-data".into(),
            description: "The persistentData block a sketch feature carries — the profile geometry Extrude and Revolve consume — with a working example".into(),
            mime: "application/json",
            read: Arc::new(|| {
                serde_json::to_string_pretty(&serde_json::json!({
                    "schema": crate::schema::sketch_persistent_schema(),
                    "example": crate::schema::sketch_example(),
                }))
                .map_err(|e| e.to_string())
            }),
        },
        ResourceSpec {
            uri: "brep://script/format".into(),
            name: "script-format".into(),
            description: "JSON Schema of the test-mcp script format".into(),
            mime: "application/json",
            read: Arc::new(|| serde_json::to_string_pretty(&crate::script::Script::json_schema()).map_err(|e| e.to_string())),
        },
    ];
    // The CAD API's manifest and one descriptor per method (contract §4),
    // for focused access without listing the whole catalogue in context.
    let catalogue = crate::cad::catalogue::Catalogue::new();
    let manifest = catalogue.manifest_json.clone();
    let identity = catalogue.identity();
    specs.push(ResourceSpec {
        uri: "brep://cad/manifest".into(),
        name: "cad-manifest".into(),
        description: "The cad.* API manifest identity {apiVersion, sha256} and its method paths".into(),
        mime: "application/json",
        read: Arc::new(move || {
            let paths: Vec<&Value> = manifest["methods"].as_array().into_iter().flatten().map(|m| &m["path"]).collect();
            serde_json::to_string_pretty(&serde_json::json!({ "manifest": identity, "methods": paths })).map_err(|e| e.to_string())
        }),
    });
    let declarations = crate::cad::declarations::module(&catalogue, &[]);
    specs.push(ResourceSpec {
        uri: "brep://cad/sdk.d.ts".into(),
        name: "cad-sdk-declarations".into(),
        description: "TypeScript declarations of the @brep/cad module, generated from the catalogue".into(),
        mime: "text/plain",
        read: Arc::new(move || Ok(declarations.clone())),
    });
    for descriptor in &catalogue.descriptors {
        let json = descriptor.to_json();
        specs.push(ResourceSpec {
            uri: format!("brep://cad/method/{}", descriptor.path.replace('%', "%25")),
            name: format!("cad-method-{}", descriptor.path),
            description: descriptor.summary.to_string(),
            mime: "application/json",
            read: Arc::new(move || serde_json::to_string_pretty(&json).map_err(|e| e.to_string())),
        });
    }
    for entry in crate::schema::entries() {
        let id = crate::schema::identity(&entry);
        let ty = id.feature_type.clone();
        specs.push(ResourceSpec {
            uri: format!("brep://schema/features/{}", id.feature_type),
            name: format!("feature-schema-{}", id.feature_type),
            description: format!("inputParams JSON Schema and defaults of {}", id.long_name),
            mime: "application/json",
            read: Arc::new(move || {
                let entry = crate::schema::entry(&ty).ok_or_else(|| format!("no feature `{ty}`"))?;
                serde_json::to_string_pretty(&serde_json::json!({
                    "schema": crate::schema::to_json_schema(&entry),
                    "defaults": crate::schema::defaults(&ty),
                }))
                .map_err(|e| e.to_string())
            }),
        });
    }
    for (uri, name, description) in [("brep://events/archive", "diagnostic-archive", "Opt-in bounded metadata history across restarts; reports disabled or persistence health"), ("brep://events", "diagnostic-events", "Bounded payload-free events; reconcile state after a sequence gap"), ("brep://documents", "open-documents", "Open document identities and revisions; subscribable") ] {
        specs.push(ResourceSpec { uri: uri.into(), name: name.into(), description: description.into(), mime:"application/json", read: Arc::new(|| Ok("{}".into())) });
    }
    for skill in crate::skills::SKILLS {
        specs.push(ResourceSpec { uri: skill.uri(), name: skill.name.into(), description: skill.description.into(), mime: "text/markdown", read: Arc::new(move || Ok(skill.text.into())) });
    }
    ResourceSet { specs }
}

pub const INSTRUCTIONS: &str = "BREP CAD automation. Use run_script with an ES module: \
import { brep, cad } from '@brep/cad'; export default async function main() { return await brep.document.docs_list({}); } \
Discover functions with cad.discovery.search/describe or brep://api/group/{group}; load brep://cad/sdk.d.ts when authoring TypeScript. \
Use skills/list to discover optional workflows and resources/read to load a selected skill. \
All application functions run inside scripts; only run_script, script_result, script_halt, screenshot, input and files are direct tools. \
Use document identities and expected revisions when editing; await writes sequentially and inspect the result before saving. \
Supply a unique retry key for each program and reuse it after a lost response. Scripts may have partial effects; halting a script does not undo changes or cancel accepted CAD operations. \
The local credential grants application and file access. Coordinates are egui points, origin top-left. \
Return compact summaries and artifact references; use screenshot for inline images.";

/// Everything the handler shares across requests: the tool and resource sets
/// (rebuilt from the registries whenever a session starts or stops), the
/// session slot, and who to tell when the tool list changes.
pub struct ServerState {
    pub tools: RwLock<ToolSet>,
    pub resources: RwLock<ResourceSet>,
    pub slot: crate::tools::app::SessionSlot,
    pub session_root: std::path::PathBuf,
    pub session_start: tokio::sync::Mutex<()>,
    /// Where sessions run: spawned per session, or the app this server is in.
    pub backend: Backend,
    /// The last client to initialise: where a *legacy* client (protocol before
    /// `2026-07-28`) receives a bare `tools/list_changed`.
    pub peer: Mutex<Option<rmcp::service::Peer<RoleServer>>>,
    /// The open `subscriptions/listen` streams: where a modern client receives
    /// `tools/list_changed`, tagged with its subscription. One per connection
    /// that asked — the HTTP host serves several at once.
    pub listeners: Mutex<Vec<Listener>>,
    next_listener: AtomicU64,
    /// The `cad.*` operation service: process-scoped, so an accepted
    /// operation outlives the connection and the request that submitted it.
    pub cad: Arc<crate::cad::CadService>,
    /// Tools a HOST adds to every tool set this server composes, whatever the
    /// session does: the test-mcp runner's own `runner` group (its PLM
    /// fixture, step records, `session_restart`), which a script reaches as
    /// `brep.runner.<tool>` and which must survive the rebuild a session
    /// restart makes. Empty unless a host sets it.
    pub extra_tools: Mutex<Vec<ToolSpec>>,
}

/// One open `subscriptions/listen` stream. `seq` tells two apart: request ids
/// (`listen:1`) repeat across connections.
pub struct Listener {
    seq: u64,
    sink: SubscriptionSink,
}

/// Removes a listener from the state when its `listen` future ends — by
/// cancellation, or by being dropped with the connection.
struct ListenerGuard {
    state: Arc<ServerState>,
    seq: u64,
}

impl Drop for ListenerGuard {
    fn drop(&mut self) {
        self.state.listeners.lock().unwrap().retain(|l| l.seq != self.seq);
    }
}

#[derive(Clone)]
pub struct BrepServer {
    pub state: Arc<ServerState>,
}

impl BrepServer {
    /// A server that runs sessions on `backend`, keeping their directories
    /// under `session_root`. Usable from inside or outside a runtime: nothing
    /// here blocks.
    pub fn new(session_root: std::path::PathBuf, backend: Backend) -> Self {
        let slot = Arc::new(RwLock::new(None));
        let state = Arc::new(ServerState {
            tools: RwLock::new(ToolSet::default()),
            resources: RwLock::new(engine_resources()),
            cad: crate::cad::CadService::new(slot.clone()),
            extra_tools: Mutex::new(Vec::new()),
            slot,
            session_root,
            session_start: tokio::sync::Mutex::new(()),
            backend,
            peer: Mutex::new(None),
            listeners: Mutex::new(Vec::new()),
            next_listener: AtomicU64::new(0),
        });
        let _ = state.cad.server.set(Arc::downgrade(&state));
        let server = Self { state: state.clone() };
        // The initial tool set: catalogue tools + session tools. Built
        // synchronously so `tools/list` is right from the first request.
        let initial = server.compose_tools(None);
        *state.tools.try_write().expect("a freshly built server has no readers") = ToolSet::new(initial);
        server
    }

    /// Seed discovery from the host's static command registry without starting
    /// an application, renderer or GPU. Execution still starts a session lazily.
    pub fn with_commands(self, describe: &Value) -> Self {
        *self.state.tools.try_write().expect("unserved server") = ToolSet::new(self.compose_tools(Some(describe)));
        self
    }

    /// Enable restart-safe admission/outcomes for a named local server. The
    /// directory must be private and must not be shared by live servers.
    pub fn with_state_dir(self, directory: &std::path::Path) -> Result<Self, String> {
        self.state.cad.code.configure_persistence(directory)?;
        Ok(self)
    }

    /// Standard Tasks adapts the existing process-scoped execution records.
    /// A task outlives its HTTP request; a server restart invalidates its epoch.
    fn execution_task(&self, id: &str) -> Result<rmcp::model::DetailedTask, McpError> {
        use rmcp::model::{Task, TaskStatus, TaskPayload, DetailedTask};
        let caller = crate::cad::tools::caller_of_transport();
        let req = serde_json::json!({"executionId":id});
        let status = self.state.cad.code.status(&caller, &req).map_err(|e| McpError::invalid_params(e.message, None))?;
        let created = status["acceptedAt"].as_str().unwrap_or_default();
        let result = self.state.cad.code.result(&caller, &req);
        let updated = status["updatedAt"].as_str().unwrap_or(created);
        let task = Task::new(id, TaskStatus::Working, created, updated)
            .with_ttl_ms(30 * 60 * 1000).with_poll_interval_ms(500)
            .with_status_message(format!("{}; {}; accepted CAD operations survive script cancellation", status["state"].as_str().unwrap_or("working"),
                if self.state.cad.code.persistent() { "durable record; interrupted work requires reconciliation" } else { "process-scoped" }));
        let payload = match result {
            Ok(r) if r["state"] == "cancelled" => TaskPayload::Cancelled,
            Ok(r) => {
                let failed = r["state"] == "failed";
                let value = crate::cad::wire::ok(r);
                let response = if failed { CallToolResult::structured_error(value) } else { CallToolResult::structured(value) };
                TaskPayload::Completed { result: serde_json::to_value(response).unwrap().as_object().unwrap().clone() }
            }
            Err(e) if e.code == "not_ready" => {
                let input_requests = self.state.cad.code.input_requests(&caller, id).map_err(|e| McpError::invalid_params(e.message, None))?;
                if input_requests.is_empty() { TaskPayload::Working } else { TaskPayload::InputRequired { input_requests } }
            },
            Err(e) => return Err(McpError::invalid_params(e.message, None)),
        };
        Ok(DetailedTask::new(task, payload))
    }

    /// Enable the independent metadata-only diagnostic archive before serving.
    /// This performs startup disk I/O; call outside the async request path.
    pub fn with_event_journal(self, directory: &std::path::Path) -> Result<Self, String> {
        let archive = crate::cad::event_archive::Archive::open(directory)?;
        self.state.cad.event_archive.set(archive).map_err(|_| "diagnostic archive already configured")?;
        Ok(self)
    }

    async fn observed_resource(&self, uri: &str) -> Result<Value, String> {
        if uri == "brep://events/archive" || uri.starts_with("brep://events/archive/") { return self.state.cad.archive_resource(uri); }
        let caller = crate::cad::tools::caller_of_transport();
        if uri == "brep://events" || uri.starts_with("brep://events/replay/") { return crate::cad::diag::resource(uri); }
        if uri == "brep://documents" {
            if self.state.slot.read().await.is_none() { return Ok(serde_json::json!({"documents":[]})); }
            return self.state.cad.ops.documents(&caller, None).await.map_err(|e| e.message);
        }
        if let Some(id) = uri.strip_prefix("brep://execution/") {
            return self.state.cad.code.status(&caller, &serde_json::json!({"executionId":id})).map_err(|e| e.message);
        }
        Err("unknown observable resource".into())
    }

    /// The registry-independent server: catalogue tools and resources only
    /// (what `brep-mcp schema` and the unit tests use).
    pub fn engine_only() -> Self {
        let slot = Arc::new(RwLock::new(None));
        let state = Arc::new(ServerState {
            tools: RwLock::new(ToolSet::new(crate::tools::engine_tools())),
            resources: RwLock::new(engine_resources()),
            cad: crate::cad::CadService::new(slot.clone()),
            extra_tools: Mutex::new(Vec::new()),
            slot,
            session_root: std::env::temp_dir(),
            session_start: tokio::sync::Mutex::new(()),
            backend: Backend::Spawn { name: "none", spawn: Arc::new(|_| Err("this server hosts no app".into())) },
            peer: Mutex::new(None),
            listeners: Mutex::new(Vec::new()),
            next_listener: AtomicU64::new(0),
        });
        let _ = state.cad.server.set(Arc::downgrade(&state));
        Self { state }
    }

    /// For an attached backend: make the running app the live session now, so
    /// the tool list carries its commands from the first `tools/list` (the
    /// HTTP transport may serve requests statelessly, where a later
    /// `tools/list_changed` reaches nobody). Returns the session info.
    pub async fn attach(&self, record: bool) -> Result<crate::session::SessionInfo, String> {
        let cx = self.context();
        let session = crate::tools::compose::attach_session(&cx, record).await?;
        self.rebuild_tools().await;
        Ok(session.info())
    }

    fn context(&self) -> Arc<crate::tools::compose::ServerContext> {
        let weak = Arc::downgrade(&self.state);
        Arc::new(crate::tools::compose::ServerContext {
            slot: self.state.slot.clone(),
            session_root: self.state.session_root.clone(),
            backend: self.state.backend.clone(),
            cad: Arc::downgrade(&self.state.cad),
            on_tools_changed: Arc::new(move || {
                if let Some(state) = weak.upgrade() {
                    let server = BrepServer { state };
                    tokio::spawn(async move { server.rebuild_tools().await });
                }
            }),
        })
    }

    /// The whole tool list for the current state: catalogue + session tools,
    /// and — when a session is live — every command the app describes plus
    /// the server-side compositions.
    fn compose_tools(&self, describe: Option<&Value>) -> Vec<ToolSpec> {
        let cx = self.context();
        let slot = self.state.slot.clone();
        let mut tools = crate::tools::session_engine_tools(slot.clone());
        tools.extend(crate::tools::compose::described_session_tools(cx, describe));
        let front_door = crate::cad::tools::front_door_tools(self.state.cad.clone(), slot.clone());
        if let Some(describe) = describe {
            tools.extend(crate::tools::app::app_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::described_pointer_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::described_capture_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::document_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::feature_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::registry_tools(slot));
        }
        let service = self.state.cad.clone();
        tools.extend(crate::files::script_tools(Arc::new(move |args| {
            let service = service.clone();
            Box::pin(async move { service.files.call(&service, &crate::cad::ops::Caller::local(), args).await })
        })));
        tools.extend(self.state.extra_tools.lock().unwrap().iter().cloned());
        // Last: the front door's `screenshot` shares its name with the
        // capture tool, which a lookup by name must find first.
        tools.extend(front_door);
        tools
    }

    /// Regenerate the tool set from the live session's command registry (or
    /// back to the sessionless set) and tell the client the list changed.
    pub async fn rebuild_tools(&self) {
        let describe = {
            let session = self.state.slot.read().await.clone();
            match session {
                Some(s) => s.host.call_ok("describe_commands", serde_json::json!({})).await.ok().and_then(|r| r.result),
                None => None,
            }
        };
        let tools = self.compose_tools(describe.as_ref());
        *self.state.tools.write().await = ToolSet::new(tools);
        self.notify_tool_list_changed().await;
        let sinks: Vec<_> = self.state.listeners.lock().unwrap().iter().map(|l| l.sink.clone()).collect();
        for sink in sinks { if sink.accepted().resources_list_changed == Some(true) { let _ = tokio::time::timeout(std::time::Duration::from_secs(5), sink.notify_resource_list_changed()).await; } }
    }

    /// Tell every client the tool list changed. A modern client hears it on
    /// its `subscriptions/listen` stream (and only there: a bare notification
    /// is not addressed to a subscription, so it ignores one); a legacy client
    /// hears it on the connection itself. A stream that cannot take the
    /// notification is finished, and is forgotten.
    async fn notify_tool_list_changed(&self) {
        let sinks: Vec<(u64, SubscriptionSink)> = {
            let listeners = self.state.listeners.lock().unwrap();
            listeners
                .iter()
                .filter(|l| l.sink.accepted().tools_list_changed == Some(true))
                .map(|l| (l.seq, l.sink.clone()))
                .collect()
        };
        for (seq, sink) in sinks {
            if !matches!(tokio::time::timeout(std::time::Duration::from_secs(5), sink.notify_tool_list_changed()).await, Ok(Ok(()))) {
                self.state.listeners.lock().unwrap().retain(|l| l.seq != seq);
            }
        }
        let peer = self.state.peer.lock().unwrap().clone();
        if let Some(peer) = peer {
            let legacy = peer.peer_info().is_none_or(|info| info.protocol_version < ProtocolVersion::V_2026_07_28);
            if legacy {
                let _ = tokio::time::timeout(std::time::Duration::from_secs(5), peer.notify_tool_list_changed()).await;
            }
        }
    }
}

fn to_rmcp_tool(spec: &ToolSpec) -> Tool {
    let mut schema: Map<String, Value> = spec.input_schema.as_object().cloned().unwrap_or_default();
    // Host provisioning belongs to live discovery, not deterministic generated docs.
    if spec.name == "run_script" {
        // The same resolution AND launcher probe execution uses (cached per
        // toolchain), so "available" means a TypeScript program will be checked.
        let toolchain = match crate::cad::code::typescript_status() {
            Ok((node, tsc)) => serde_json::json!({"available":true,"requiredVersion":crate::cad::code::TSC_VERSION,"node":node,"tsc":tsc,"check":"the resolved launcher reported the pinned version; every TypeScript program is strictly type-checked before it runs"}),
            Err(error) => serde_json::json!({"available":false,"requiredVersion":crate::cad::code::TSC_VERSION,"diagnostic":error.message}),
        };
        schema.get_mut("properties").unwrap()["language"]["description"] = serde_json::json!(if toolchain["available"] == true {
            "JavaScript is available. TypeScript requires the resolved pinned compiler to pass strict checking before execution.".to_string()
        } else { format!("JavaScript is available. TypeScript is unavailable until explicitly provisioned: {}", toolchain["diagnostic"].as_str().unwrap_or_default()) });
        schema.get_mut("properties").unwrap()["language"]["x-brep-typescript"] = toolchain;
    }
    let mut tool = Tool::new(spec.name.clone(), spec.doc.clone(), Arc::new(schema));
    let mut ann = ToolAnnotations::default();
    ann.read_only_hint = Some(spec.annotations.read_only);
    ann.destructive_hint = Some(spec.annotations.destructive);
    ann.idempotent_hint = Some(spec.annotations.idempotent);
    ann.open_world_hint = Some(matches!(spec.name.as_str(), "run_script" | "input" | "screenshot"));
    tool.annotations = Some(ann);
    tool.output_schema = Some(Arc::new(spec.output_schema.as_object().cloned().unwrap_or_else(|| serde_json::json!({"type":"object"}).as_object().unwrap().clone())));
    tool
}

/// How long a client may keep a list (SEP-2549). Clients on protocol
/// `2026-07-28` (Claude Code, 2026-09) reject a list result without `ttlMs`
/// and `cacheScope`. Lists are per session — private — and change when a
/// session starts or stops, which `tools/list_changed` announces; a minute
/// bounds a client that missed the notification.
const LIST_TTL_MS: u64 = 60_000;

fn cacheable(result: ListToolsResult) -> ListToolsResult {
    result.with_ttl_ms(LIST_TTL_MS).with_cache_scope(CacheScope::Private)
}


fn to_rmcp_resource(spec: &ResourceSpec) -> Resource {
    let mut r = Resource::new(spec.uri.clone(), spec.name.clone());
    r.description = Some(spec.description.clone());
    r.mime_type = Some(spec.mime.to_string());
    r
}

impl ServerHandler for BrepServer {
    fn initialize(&self, request: rmcp::model::InitializeRequestParams, context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::InitializeResult, McpError>> + Send + '_ {
        async move {
            let mut info = self.get_info();
            if request.protocol_version.as_str() < ProtocolVersion::V_2026_07_28.as_str() {
                // Notifications use subscriptions/listen, not the deprecated
                // resources/subscribe lifecycle. Legacy clients poll resources
                // and use script_result/script_halt rather than Tasks.
                if let Some(resources) = &mut info.capabilities.resources {
                    resources.subscribe = None;
                    resources.list_changed = None;
                }
                if let Some(extensions) = &mut info.capabilities.extensions {
                    extensions.remove(rmcp::model::TASKS_EXTENSION_ID);
                    extensions.remove(crate::skills::EXTENSION);
                }
            }
            InitializationInfo(info).initialize(request, context).await
        }
    }
    fn get_info(&self) -> ServerInfo {
        let mut caps = ServerCapabilities::builder()
            .enable_prompts()
            .enable_tools()
            .enable_tool_list_changed()
            .enable_resources()
            .enable_resources_subscribe()
            .enable_resources_list_changed()
            .enable_tasks()
            .build();
        caps.extensions.get_or_insert_with(Default::default).insert(crate::skills::EXTENSION.into(), Default::default());
        ServerInfo::new(caps)
            .with_instructions(INSTRUCTIONS)
            .with_server_info(Implementation::new("brep-mcp", env!("CARGO_PKG_VERSION")))
    }

    fn list_prompts(&self, request: Option<PaginatedRequestParams>, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::ListPromptsResult, McpError>> + Send + '_ {
        std::future::ready(crate::prompts::list(request.as_ref().and_then(|p| p.cursor.as_deref())))
    }

    fn get_prompt(&self, request: rmcp::model::GetPromptRequestParams, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::GetPromptResponse, McpError>> + Send + '_ {
        std::future::ready(crate::prompts::get(request).map(Into::into))
    }

    fn on_custom_request(&self, request: rmcp::model::CustomRequest, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::CustomResult, McpError>> + Send + '_ {
        async move {
            if !matches!(request.method.as_str(), "skills/list" | "skills/get") {
                return Err(McpError::new(rmcp::model::ErrorCode::METHOD_NOT_FOUND, "unknown method", None));
            }
            crate::skills::request(&request.method, &request.params.unwrap_or(serde_json::json!({})))
                .map(rmcp::model::CustomResult::new).map_err(|e| McpError::invalid_params(e, None))
        }
    }

    /// The client that just initialised is the one `tools/list_changed` goes to.
    fn on_initialized(&self, context: NotificationContext<RoleServer>) -> impl Future<Output = ()> + Send + '_ {
        *self.state.peer.lock().unwrap() = Some(context.peer);
        std::future::ready(())
    }

    /// `subscriptions/listen` (protocol `2026-07-28`): the stream a modern
    /// client opens right after `server/discover`, and where
    /// `tools/list_changed` reaches it. Accept whatever it asks for that the capabilities
    /// advertise; a server that leaves this unimplemented answers
    /// method-not-found, which the HTTP transport maps to a 404 and the
    /// client reads as a dropped connection.
    fn accepted_subscription_filter(&self, requested: &SubscriptionFilter) -> Option<SubscriptionFilter> {
        let mut accepted = requested.supported_by(&self.get_info().capabilities);
        if let Some(uris) = &mut accepted.resource_subscriptions {
            uris.retain(|uri| matches!(uri.as_str(), "brep://events" | "brep://documents") || uri.starts_with("brep://execution/"));
            uris.truncate(32);
        }
        Some(accepted)
    }

    /// Hold the stream open until the client cancels it or the connection
    /// goes; returning early would end it gracefully and send the client
    /// into a re-listen loop.
    fn listen(&self, context: SubscriptionContext) -> impl Future<Output = Result<(), McpError>> + Send + '_ {
        async move {
            let seq = self.state.next_listener.fetch_add(1, Ordering::Relaxed);
            {
                let mut listeners = self.state.listeners.lock().unwrap();
                if listeners.len() >= 128 { return Err(McpError::internal_error("too many notification subscriptions", None)); }
                listeners.push(Listener { seq, sink: context.sink().clone() });
            }
            let _guard = ListenerGuard { state: self.state.clone(), seq };
            let uris = context.sink().accepted().resource_subscriptions.clone().unwrap_or_default();
            let mut last = std::collections::HashMap::new();
            let mut interval = tokio::time::interval(std::time::Duration::from_millis(500));
            loop {
                tokio::select! {
                    _ = context.cancelled() => break,
                    _ = interval.tick(), if !uris.is_empty() => {
                        for uri in &uris {
                            let observed = tokio::select! {
                                _ = context.cancelled() => return Ok(()),
                                result = tokio::time::timeout(std::time::Duration::from_secs(5), self.observed_resource(uri)) => result,
                            };
                            let value = match observed {
                                Ok(Ok(value)) => value,
                                Ok(Err(error)) => serde_json::json!({"error":error}),
                                Err(_) => continue, // a busy app is not a resource change
                            };
                            let digest = crate::cad::canon::sha256_hex(value.to_string().as_bytes());
                            if last.insert(uri.clone(), digest.clone()).is_some_and(|old| old != digest) {
                                if !matches!(tokio::time::timeout(std::time::Duration::from_secs(5), context.sink().notify_resource_updated(uri)).await, Ok(Ok(()))) { return Ok(()); }
                            }
                        }
                    }
                }
            }
            Ok(())
        }
    }

    fn list_tools(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<ListToolsResult, McpError>> + Send + '_ {
        async move {
            let set = self.state.tools.read().await;
            // The front door (`cad::tools::FRONT_DOOR`): everything else is
            // a script function of `run_script`.
            let listed = set.specs.iter().filter(|t| t.group == "script");
            Ok(cacheable(ListToolsResult::with_all_items(listed.map(to_rmcp_tool).collect())))
        }
    }

    fn call_tool(
        &self,
        request: CallToolRequestParams,
        context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<CallToolResponse, McpError>> + Send + '_ {
        async move {
            let spec = {
                let set = self.state.tools.read().await;
                let front = set.specs.iter().find(|t| t.name == *request.name && t.group == "script");
                front.or_else(|| set.get(&request.name)).cloned()
            };
            let Some(spec) = spec else {
                return Err(McpError::invalid_params(format!("unknown tool `{}`", request.name), None));
            };
            if spec.group != "script" {
                let script = if crate::cad::script_callable(spec.group, &spec.name) {
                    format!("call it from run_script as `await brep.{}.{}(args)`", crate::cad::sdk::ident(spec.group), crate::script_sdk::method_name(spec.group, &spec.name))
                } else {
                    "it is not exposed over MCP".into()
                };
                return Ok(CallToolResult::error(vec![ContentBlock::text(format!("`{}` is not a front-door tool: {script}", request.name))]).into());
            }
            let mut args = Value::Object(request.arguments.unwrap_or_default());
            if let Err(e) = crate::script_sdk::validate_front_door(&request.name, &args) {
                return Ok(CallToolResult::error(vec![ContentBlock::text(e)]).into());
            }
            let task_mode = request.name.as_ref() == "run_script" && args.get("wait_ms").and_then(Value::as_u64) == Some(0) && context.client_capabilities().is_some_and(|c| c.supports_tasks());
            if task_mode { args["wait_ms"] = serde_json::json!(0); }
            let task_form = task_mode && crate::cad::inputs::supports_task_form(&context);
            let progress = crate::progress::waits(&request.name, &args);
            let operation = crate::cad::inputs::with_task_form_support(task_form, (spec.handler)(args));
            match crate::progress::waiting(&context, progress, operation).await {
                Ok(out) => {
                    if task_mode && out.json["value"]["running"] == true {
                        let id = out.json["value"]["status"]["executionId"].as_str().ok_or_else(|| McpError::internal_error("missing execution id", None))?;
                        return Ok(rmcp::model::CreateTaskResult::new(self.execution_task(id)?.task).into());
                    }
                    let text = serde_json::to_string(&out.json)
                        .map_err(|e| McpError::internal_error(format!("serialize result: {e}"), None))?;
                    let mut content = vec![ContentBlock::text(text)];
                    for im in out.images {
                        let data = base64::engine::general_purpose::STANDARD.encode(&im.png);
                        content.push(ContentBlock::image(data, im.mime));
                    }
                    let failed = out.json.get("ok") == Some(&Value::Bool(false))
                        || (request.name.as_ref() == "run_script" && out.json["value"]["state"] == "failed");
                    let mut result = if failed { CallToolResult::error(content) } else { CallToolResult::success(content) };
                    result.structured_content = Some(out.json);
                    Ok(result.into())
                }
                // A tool that ran and refused is a tool *result* with isError, not a
                // protocol error: the model reads the text and adapts.
                Err(e) => Ok(CallToolResult::error(vec![ContentBlock::text(e)]).into()),
            }
        }
    }

    fn get_task(&self, request: rmcp::model::GetTaskParams, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::GetTaskResult, McpError>> + Send + '_ {
        async move { self.execution_task(&request.task_id).map(rmcp::model::GetTaskResult::new) }
    }

    fn cancel_task(&self, request: rmcp::model::CancelTaskParams, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<(), McpError>> + Send + '_ {
        async move {
            self.state.cad.code.cancel(&crate::cad::tools::caller_of_transport(), &serde_json::json!({"executionId":request.task_id}))
                .map(|_| ()).map_err(|e| McpError::invalid_params(e.message, None))
        }
    }

    fn update_task(&self, request: rmcp::model::UpdateTaskParams, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<(), McpError>> + Send + '_ {
        async move { self.state.cad.code.update_input(&crate::cad::tools::caller_of_transport(), &request.task_id, &request.input_responses)
            .map_err(|e| McpError::invalid_params(e.message, None)) }
    }

    fn list_resource_templates(&self, _request: Option<PaginatedRequestParams>, _context: RequestContext<RoleServer>) -> impl Future<Output = Result<rmcp::model::ListResourceTemplatesResult, McpError>> + Send + '_ {
        async move {
            let templates = [
                ("brep://events/archive/{archiveId}/{sequence}/{limit}", "diagnostic-archive-page", "Synced metadata across restarts; bounded pages, explicit gap records and cursor gaps; limit 1..128"),
                ("brep://events/replay/{epoch}/{sequence}/{limit}", "event-replay", "Bounded event pages; sequence is the next wanted event, limit 1..128; explicit gaps require state reconciliation"),
                ("brep://api/function/{path}", "function-contract", "Input/output schemas and effect annotations for a discovered function"),
                ("brep://api/group/{group}", "function-group", "Function contracts for a discovered group"),
                ("brep://execution/{executionId}", "execution-status", "Process-scoped execution status; subscribe for invalidation and reread to reconcile"),
                ("brep://artifact/{artifactId}", "artifact-manifest", "Metadata for an expiring artifact owned by this local transport scope"),
                ("brep://artifact/{artifactId}/chunks/{offset}", "artifact-chunk", "At most 64 KiB of artifact bytes as base64 JSON; offset is a decimal byte offset"),
            ].into_iter().map(|(uri, name, description)| rmcp::model::ResourceTemplate::new(uri, name).with_description(description).with_mime_type("application/json")).collect();
            Ok(rmcp::model::ListResourceTemplatesResult::with_all_items(templates).with_ttl_ms(LIST_TTL_MS).with_cache_scope(CacheScope::Private))
        }
    }

    fn list_resources(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<ListResourcesResult, McpError>> + Send + '_ {
        async move {
            let mut resources: Vec<Resource> = self.state.resources.read().await.specs.iter().map(to_rmcp_resource).collect();
            let groups: std::collections::BTreeSet<String> = self.state.tools.read().await.specs.iter().filter(|t| crate::script_sdk::script_callable(t.group, &t.name)).map(|t| t.group.to_string()).collect();
            for group in groups { resources.push(Resource::new(format!("brep://api/group/{group}"), format!("functions-{group}"))); }
            let session = self.state.slot.read().await.clone();
            if let Some(session) = session {
                let catalogue = session.host.call_ok("plugin_feature_catalogue", serde_json::json!({})).await
                    .map_err(|error| McpError::internal_error(error, None))?.result.unwrap_or(Value::Null);
                for entry in catalogue["features"].as_array().into_iter().flatten() {
                    let id = crate::schema::identity(entry);
                    if crate::schema::entry(&id.feature_type).is_some() { continue; }
                    let mut resource = Resource::new(format!("brep://schema/features/{}", id.feature_type), format!("feature-schema-{}", id.feature_type));
                    resource.description = Some(format!("inputParams JSON Schema and defaults of {}", id.long_name));
                    resource.mime_type = Some("application/json".into());
                    resources.push(resource);
                }
            }
            Ok(ListResourcesResult::with_all_items(resources)
                .with_ttl_ms(LIST_TTL_MS)
                .with_cache_scope(CacheScope::Private))
        }
    }

    fn read_resource(
        &self,
        request: ReadResourceRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<ReadResourceResponse, McpError>> + Send + '_ {
        async move {
            if let Some(path) = request.uri.strip_prefix("brep://artifact/") {
                let caller = crate::cad::tools::caller_of_transport();
                let owner = crate::cad::artifacts::Owner { scope: caller.scope.clone(), execution: None };
                let value = if let Some((id, offset)) = path.split_once("/chunks/") {
                    let offset = offset.parse::<usize>().map_err(|_| McpError::invalid_params("artifact offset must be a decimal byte offset", None))?;
                    self.state.cad.artifacts.read(&owner, id, offset, None)
                } else { self.state.cad.artifacts.reference_of(&owner, path) }.map_err(|e| McpError::resource_not_found(e.message, None))?;
                let mut content = ResourceContents::text(value.to_string(), request.uri.clone());
                if let ResourceContents::TextResourceContents { mime_type, .. } = &mut content { *mime_type = Some("application/json".into()); }
                return Ok(ReadResourceResult::new(vec![content]).with_ttl_ms(0).with_cache_scope(CacheScope::Private).into());
            }
            if matches!(request.uri.as_str(), "brep://events" | "brep://documents") || request.uri == "brep://events/archive" || request.uri.starts_with("brep://events/archive/") || request.uri.starts_with("brep://events/replay/") || request.uri.starts_with("brep://execution/") {
                let value = self.observed_resource(&request.uri).await.map_err(|e| McpError::resource_not_found(e, None))?;
                return Ok(ReadResourceResult::new(vec![ResourceContents::text(value.to_string(), request.uri.clone())]).with_ttl_ms(0).with_cache_scope(CacheScope::Private).into());
            }
            let dynamic = if let Some(path) = request.uri.strip_prefix("brep://api/function/") {
                let value = self.state.cad.describe(&[path.to_string()]).await.map_err(|e| McpError::resource_not_found(e.message, None))?;
                Some((value.to_string(), "application/json"))
            } else if let Some(group) = request.uri.strip_prefix("brep://api/group/") {
                let set = self.state.tools.read().await;
                let specs = set.specs.iter().filter(|t| t.group == group && crate::script_sdk::script_callable(t.group, &t.name)).cloned().collect::<Vec<_>>();
                if specs.is_empty() { return Err(McpError::resource_not_found("unknown function group", None)); }
                Some((crate::generate::commands_json(&ToolSet::new(specs)).to_string(), "application/json"))
            } else if request.uri == "brep://cad/sdk.d.ts" {
                let set = self.state.tools.read().await;
                Some((crate::cad::declarations::module(&self.state.cad.catalogue, &crate::script_sdk::script_tools(&set.specs)), "text/plain"))
            } else if request.uri == "brep://api/commands" {
                let set = self.state.tools.read().await;
                Some((crate::generate::commands_json(&set).to_string(), "application/json"))
            } else { None };
            if let Some((text, mime)) = dynamic {
                let mut contents = ResourceContents::text(text, request.uri.clone());
                if let ResourceContents::TextResourceContents { mime_type, .. } = &mut contents { *mime_type = Some(mime.into()); }
                return Ok(ReadResourceResult::new(vec![contents]).with_ttl_ms(LIST_TTL_MS).with_cache_scope(CacheScope::Private).into());
            }
            let session = self.state.slot.read().await.clone();
            if let Some(session) = session {
                let value = if request.uri == "brep://schema/features" {
                    let catalogue = session.host.call_ok("plugin_feature_catalogue", serde_json::json!({})).await
                        .map_err(|error| McpError::internal_error(error, None))?.result.unwrap_or(Value::Null);
                    Some(crate::schema::all_from_catalogue(&catalogue))
                } else if let Some(ty) = request.uri.strip_prefix("brep://schema/features/").filter(|ty| ty.contains('/')) {
                    let entry = crate::tools::session_feature_entry(&session, ty).await
                        .map_err(|error| McpError::resource_not_found(error, None))?;
                    Some(serde_json::json!({"schema":crate::schema::to_json_schema(&entry),"defaults":crate::schema::defaults_from_entry(&entry)}))
                } else { None };
                if let Some(value) = value {
                    let text = serde_json::to_string_pretty(&value).map_err(|error| McpError::internal_error(error.to_string(), None))?;
                    let mut contents = ResourceContents::text(text, request.uri.clone());
                    if let ResourceContents::TextResourceContents { mime_type, .. } = &mut contents {
                        *mime_type = Some("application/json".into());
                    }
                    return Ok(ReadResourceResult::new(vec![contents]).with_ttl_ms(LIST_TTL_MS).with_cache_scope(CacheScope::Private).into());
                }
            }
            let spec = self.state.resources.read().await.get(&request.uri).cloned();
            let Some(spec) = spec else {
                return Err(McpError::resource_not_found(format!("no resource `{}`", request.uri), None));
            };
            let text = (spec.read)().map_err(|e| McpError::internal_error(e, None))?;
            let mut contents = ResourceContents::text(text, spec.uri.clone());
            if let ResourceContents::TextResourceContents { mime_type, .. } = &mut contents {
                *mime_type = Some(spec.mime.to_string());
            }
            Ok(ReadResourceResult::new(vec![contents]).with_ttl_ms(LIST_TTL_MS).with_cache_scope(CacheScope::Private).into())
        }
    }
}

/// Serve over stdin/stdout until the client disconnects.
pub async fn serve_stdio(server: BrepServer) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let state = server.state.clone();
    let running = server.serve(rmcp::transport::io::stdio()).await?;
    *state.peer.lock().unwrap() = Some(running.peer().clone());
    let result = running.waiting().await;
    state.cad.code.shutdown().await;
    *state.peer.lock().unwrap() = None;
    // A client that disconnects mid-session must not leave an app thread behind.
    if let Some(session) = state.slot.write().await.take() {
        match Arc::try_unwrap(session) {
            Ok(s) => { tokio::task::spawn_blocking(move || s.host.stop()).await.ok(); }
            Err(shared) => shared.host.request_stop(),
        }
    }
    result?;
    Ok(())
}

