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
    ResourceSet { specs }
}

pub const INSTRUCTIONS: &str = "BREP CAD automation. Tools are generated from the engine's registries: \
`feature_catalogue` / `feature_schema` describe features; session tools drive the running app. \
Coordinates are egui points with the origin at the top-left of the app surface; every reply carries `frame` and `ppp`.";

/// Everything the handler shares across requests: the tool and resource sets
/// (rebuilt from the registries whenever a session starts or stops), the
/// session slot, and who to tell when the tool list changes.
pub struct ServerState {
    pub tools: RwLock<ToolSet>,
    pub resources: RwLock<ResourceSet>,
    pub slot: crate::tools::app::SessionSlot,
    pub session_root: std::path::PathBuf,
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
        let state = Arc::new(ServerState {
            tools: RwLock::new(ToolSet::default()),
            resources: RwLock::new(engine_resources()),
            slot: Arc::new(RwLock::new(None)),
            session_root,
            backend,
            peer: Mutex::new(None),
            listeners: Mutex::new(Vec::new()),
            next_listener: AtomicU64::new(0),
        });
        let server = Self { state: state.clone() };
        // The initial tool set: catalogue tools + session tools. Built
        // synchronously so `tools/list` is right from the first request.
        let initial = server.compose_tools(None);
        *state.tools.try_write().expect("a freshly built server has no readers") = ToolSet::new(initial);
        server
    }

    /// The registry-independent server: catalogue tools and resources only
    /// (what `brep-mcp schema` and the unit tests use).
    pub fn engine_only() -> Self {
        let state = Arc::new(ServerState {
            tools: RwLock::new(ToolSet::new(crate::tools::engine_tools())),
            resources: RwLock::new(engine_resources()),
            slot: Arc::new(RwLock::new(None)),
            session_root: std::env::temp_dir(),
            backend: Backend::Spawn { name: "none", spawn: Arc::new(|_| Err("this server hosts no app".into())) },
            peer: Mutex::new(None),
            listeners: Mutex::new(Vec::new()),
            next_listener: AtomicU64::new(0),
        });
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
        tools.extend(crate::tools::compose::session_tools(cx));
        if let Some(describe) = describe {
            tools.extend(crate::tools::app::app_tools(slot.clone(), describe));
            tools.extend(crate::tools::compose::pointer_tools(slot.clone()));
            tools.extend(crate::tools::compose::capture_tools(slot.clone()));
            tools.extend(crate::tools::compose::document_tools(slot.clone()));
            tools.extend(crate::tools::compose::feature_tools(slot.clone()));
            tools.extend(crate::tools::compose::registry_tools(slot));
        }
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
            if sink.notify_tool_list_changed().await.is_err() {
                self.state.listeners.lock().unwrap().retain(|l| l.seq != seq);
            }
        }
        let peer = self.state.peer.lock().unwrap().clone();
        if let Some(peer) = peer {
            let legacy = peer.peer_info().is_none_or(|info| info.protocol_version < ProtocolVersion::V_2026_07_28);
            if legacy {
                let _ = peer.notify_tool_list_changed().await;
            }
        }
    }
}

fn to_rmcp_tool(spec: &ToolSpec) -> Tool {
    let schema: Map<String, Value> = spec.input_schema.as_object().cloned().unwrap_or_default();
    let mut tool = Tool::new(spec.name.clone(), spec.doc.clone(), Arc::new(schema));
    let mut ann = ToolAnnotations::default();
    ann.read_only_hint = Some(spec.annotations.read_only);
    ann.destructive_hint = Some(spec.annotations.destructive);
    ann.idempotent_hint = Some(spec.annotations.idempotent);
    ann.open_world_hint = Some(false);
    tool.annotations = Some(ann);
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
    fn get_info(&self) -> ServerInfo {
        let caps = ServerCapabilities::builder()
            .enable_tools()
            .enable_tool_list_changed()
            .enable_resources()
            .build();
        ServerInfo::new(caps)
            .with_instructions(INSTRUCTIONS)
            .with_server_info(Implementation::new("brep-mcp", env!("CARGO_PKG_VERSION")))
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
        Some(requested.supported_by(&self.get_info().capabilities))
    }

    /// Hold the stream open until the client cancels it or the connection
    /// goes; returning early would end it gracefully and send the client
    /// into a re-listen loop.
    fn listen(&self, context: SubscriptionContext) -> impl Future<Output = Result<(), McpError>> + Send + '_ {
        async move {
            let seq = self.state.next_listener.fetch_add(1, Ordering::Relaxed);
            self.state.listeners.lock().unwrap().push(Listener { seq, sink: context.sink().clone() });
            let _guard = ListenerGuard { state: self.state.clone(), seq };
            context.cancelled().await;
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
            Ok(cacheable(ListToolsResult::with_all_items(set.specs.iter().map(to_rmcp_tool).collect())))
        }
    }

    fn call_tool(
        &self,
        request: CallToolRequestParams,
        _context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<CallToolResponse, McpError>> + Send + '_ {
        async move {
            let spec = self.state.tools.read().await.get(&request.name).cloned();
            let Some(spec) = spec else {
                return Err(McpError::invalid_params(format!("unknown tool `{}`", request.name), None));
            };
            let args = Value::Object(request.arguments.unwrap_or_default());
            match (spec.handler)(args).await {
                Ok(out) => {
                    let text = serde_json::to_string(&out.json)
                        .map_err(|e| McpError::internal_error(format!("serialize result: {e}"), None))?;
                    let mut content = vec![ContentBlock::text(text)];
                    for im in out.images {
                        let data = base64::engine::general_purpose::STANDARD.encode(&im.png);
                        content.push(ContentBlock::image(data, im.mime));
                    }
                    Ok(CallToolResult::success(content).into())
                }
                // A tool that ran and refused is a tool *result* with isError, not a
                // protocol error: the model reads the text and adapts.
                Err(e) => Ok(CallToolResult::error(vec![ContentBlock::text(e)]).into()),
            }
        }
    }

    fn list_resources(
        &self,
        _request: Option<PaginatedRequestParams>,
        _context: RequestContext<RoleServer>,
    ) -> impl Future<Output = Result<ListResourcesResult, McpError>> + Send + '_ {
        async move {
            let mut resources: Vec<Resource> = self.state.resources.read().await.specs.iter().map(to_rmcp_resource).collect();
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
                    return Ok(ReadResourceResult::new(vec![contents]).into());
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
            Ok(ReadResourceResult::new(vec![contents]).into())
        }
    }
}

/// Serve over stdin/stdout until the client disconnects.
pub async fn serve_stdio(server: BrepServer) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let state = server.state.clone();
    let running = server.serve(rmcp::transport::io::stdio()).await?;
    *state.peer.lock().unwrap() = Some(running.peer().clone());
    running.waiting().await?;
    // A client that disconnects mid-session must not leave an app thread behind.
    if let Some(session) = state.slot.write().await.take() {
        if let Ok(s) = Arc::try_unwrap(session) {
            tokio::task::spawn_blocking(move || s.host.stop()).await.ok();
        }
    }
    Ok(())
}

