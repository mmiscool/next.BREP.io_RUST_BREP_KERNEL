//! The command registry and the wire envelope (Appendix A).
//!
//! A command is a [`CommandSpec`]: name, group, doc, the JSON Schemas of its
//! argument and result types (derived with `schemars`), the frame phase it
//! runs in, MCP-style annotations, and a handler. Each `cmd_*` module owns a
//! `pub static COMMANDS: &[CommandSpec]`; [`registry`] gathers them. A host
//! generates its tool list from [`describe`]; nothing lists commands by hand.
use serde::{Deserialize, Serialize};
use serde_json::Value;
use brep_render::engine_state::EngineState;

/// Where in the frame a command runs (§4.4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Phase {
    /// `raw_input_hook`: the handler emits `egui::Event`s.
    Input,
    /// Top of `ui`, before any panel draws.
    Mutate,
    /// Bottom of `ui`, after the state registry is rebuilt.
    Read,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, schemars::JsonSchema)]
pub struct Annotations {
    pub read_only: bool,
    pub destructive: bool,
    pub idempotent: bool,
    /// The host applies the idle contract (§6) before returning.
    pub waits: bool,
}

impl Annotations {
    pub const READ: Annotations = Annotations { read_only: true, destructive: false, idempotent: true, waits: false };
    pub const INPUT: Annotations = Annotations { read_only: false, destructive: false, idempotent: false, waits: false };
    pub const MUTATE: Annotations = Annotations { read_only: false, destructive: false, idempotent: false, waits: true };
    pub const MUTATE_NOWAIT: Annotations = Annotations { read_only: false, destructive: false, idempotent: true, waits: false };
    pub const DESTRUCTIVE: Annotations = Annotations { read_only: false, destructive: true, idempotent: false, waits: true };
}

/// What an app-phase handler sees.
pub struct Ctx<'a> {
    pub app: &'a mut crate::app::BrepApp,
    pub egui: &'a egui::Context,
}

/// What a handler returns.
pub enum Outcome {
    Done(Value),
    /// A JSON result plus a binary payload (a PNG) carried beside it.
    Blob { json: Value, bytes: Vec<u8> },
    /// The reply completes when `Event::Screenshot` with this token arrives.
    /// `document` is the visible document's handle when the capture was asked
    /// for; the capture is refused if another one is showing when it lands.
    AwaitScreenshot { token: u64, region: crate::automation::cmd_capture::Region, document: Option<String> },
    /// Long work the command has STARTED without waiting for it: the queue
    /// calls the poller once per frame in the command's phase, from the frame
    /// thread, until it answers. A handler never waits inside a frame —
    /// geometry, a PLM answer or a store write lands on later frames, and
    /// the app keeps drawing and answering other commands meanwhile.
    Poll(Poller),
}

/// See [`Outcome::Poll`]: `None` = not yet, `Some(result)` = the reply.
pub type Poller = Box<dyn FnMut(&mut Ctx<'_>) -> Option<Result<Outcome, String>> + Send>;

pub type InputHandler = fn(&mut crate::automation::pointer::Pointer, Value, &mut Vec<egui::Event>) -> Result<Value, String>;
pub type AppHandler = fn(&mut Ctx<'_>, Value) -> Result<Outcome, String>;

pub enum Handler {
    Input(InputHandler),
    App(AppHandler),
}

pub struct CommandSpec {
    pub name: &'static str,
    pub group: &'static str,
    pub doc: &'static str,
    pub phase: Phase,
    pub annotations: Annotations,
    pub args_schema: fn() -> Value,
    pub result_schema: fn() -> Value,
    pub handler: Handler,
}

/// One app-handled [`CommandSpec`] on one line:
/// `app_command!(group, name, doc, Args, handler, Phase, ANNOTATIONS, result_schema)`.
/// `Args` is the argument type (its schema is `schema_of::<Args>`), `Phase` and
/// `ANNOTATIONS` name the [`Phase`] and [`Annotations`] constants. Leaving out
/// `result_schema` declares the exactly-empty result ([`empty_result_schema`]).
/// Input-phase commands, and any spec whose annotations or argument schema is
/// not one of those shapes, stay written out as literals.
macro_rules! app_command {
    ($group:literal, $name:literal, $doc:expr, $args:ty, $handler:path, $phase:ident, $annotation:ident, $output:expr) => {
        $crate::automation::command::CommandSpec {
            name: $name,
            group: $group,
            doc: $doc,
            phase: $crate::automation::command::Phase::$phase,
            annotations: $crate::automation::command::Annotations::$annotation,
            args_schema: $crate::automation::command::schema_of::<$args>,
            result_schema: $output,
            handler: $crate::automation::command::Handler::App($handler),
        }
    };
    ($group:literal, $name:literal, $doc:expr, $args:ty, $handler:path, $phase:ident, $annotation:ident) => {
        $crate::automation::command::app_command!($group, $name, $doc, $args, $handler, $phase, $annotation, $crate::automation::command::empty_result_schema)
    };
}
pub(crate) use app_command;

/// A mutate handler that parses its arguments, makes one engine call and
/// answers the empty result: `engine_call(ctx, args, |e, a: Args| e.m(&a.id))`.
/// The call's own value is dropped, its error is the command's error.
pub fn engine_call<A, R, E>(ctx: &mut Ctx<'_>, args: Value, f: impl FnOnce(&mut EngineState, A) -> Result<R, E>) -> Result<Outcome, String>
where
    A: serde::de::DeserializeOwned,
    String: From<E>,
{
    let a: A = parse_args(args)?;
    f(ctx.app.docs.engine_mut(), a)?;
    Ok(Outcome::Done(serde_json::json!({})))
}

/// A reader that takes no arguments and answers what `f` reads from the
/// engine: `read_engine(ctx, args, |e| e.feature_catalogue())`.
pub fn read_engine(ctx: &mut Ctx<'_>, args: Value, f: impl FnOnce(&EngineState) -> Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(f(ctx.app.docs.engine())))
}

/// The JSON Schema of a type, as a `serde_json::Value`.
///
/// `schemars` renders an unconstrained value (e.g. a `serde_json::Value` field)
/// as the boolean schema `true`. Strict MCP tool-schema validators (such as
/// Claude Code's) reject a bare boolean where a schema object is expected, and
/// that rejects the *whole* `tools/list` — one such field zeroes out every tool.
/// So we normalize boolean subschemas to their object form before publishing.
pub fn schema_of<T: schemars::JsonSchema>() -> Value {
    let mut schema = serde_json::to_value(schemars::schema_for!(T)).unwrap_or(Value::Null);
    normalize_bool_subschemas(&mut schema);
    schema
}

/// A source-reviewed command whose successful native result is exactly `{}`.
/// Use this only for genuinely empty payloads, never as a placeholder for an
/// undeclared result. The MCP adapter adds its own envelope fields separately.
pub fn empty_result_schema() -> Value {
    serde_json::json!({
        "type": "object",
        "properties": {},
        "additionalProperties": false,
        "description": "No native application payload. The MCP adapter adds frame, optional notices and settlement information."
    })
}

/// Replace boolean subschemas (`true`/`false` used where a schema *object* is
/// expected — a `properties` member or `items`) with their object equivalents
/// (`true` → `{}`, `false` → `{"not": {}}`). Legitimate booleans such as
/// `additionalProperties` and `default` are left untouched.
pub(super) fn normalize_bool_subschemas(node: &mut Value) {
    match node {
        Value::Object(map) => {
            if let Some(Value::Object(props)) = map.get_mut("properties") {
                for value in props.values_mut() {
                    if let Some(b) = value.as_bool() {
                        *value = if b { serde_json::json!({}) } else { serde_json::json!({"not": {}}) };
                    }
                }
            }
            if let Some(items) = map.get_mut("items") {
                if let Some(b) = items.as_bool() {
                    *items = if b { serde_json::json!({}) } else { serde_json::json!({"not": {}}) };
                }
            }
            for value in map.values_mut() {
                normalize_bool_subschemas(value);
            }
        }
        Value::Array(arr) => {
            for value in arr.iter_mut() {
                normalize_bool_subschemas(value);
            }
        }
        _ => {}
    }
}

pub fn parse_args<T: serde::de::DeserializeOwned>(args: Value) -> Result<T, String> {
    // A call with no arguments arrives as `null` or `{}`; both mean "none".
    let args = if args.is_null() { Value::Object(Default::default()) } else { args };
    serde_json::from_value(args).map_err(|e| format!("arguments: {e}"))
}

/// A command that takes nothing.
#[derive(Debug, Default, Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NoArgs {}

/// A command that returns nothing beyond the envelope.
#[derive(Debug, Default, Serialize, schemars::JsonSchema)]
pub struct Empty {}

/// Every registered command, in module order. Each `cmd_*` module contributes
/// its `COMMANDS` static; `tests/automation_registry.rs` walks the source tree
/// for `static COMMANDS` declarations and fails if one is missing here, so this
/// function cannot silently become a hand-kept subset.
pub fn registry() -> Vec<&'static CommandSpec> {
    use crate::automation::*;
    let sets: &[&[CommandSpec]] = &[
        cmd_frame::COMMANDS,
        cmd_files::COMMANDS,
        cmd_perf::COMMANDS,
        cmd_input::COMMANDS,
        cmd_capture::COMMANDS,
        cmd_state::COMMANDS,
        cmd_document::COMMANDS,
        cmd_classes::COMMANDS,
        cmd_history::COMMANDS,
        cmd_scene::COMMANDS,
        cmd_camera::COMMANDS,
        cmd_settings::COMMANDS,
        cmd_shell::COMMANDS,
        cmd_plugins::COMMANDS,
        cmd_metadata::COMMANDS,
        cmd_assembly::COMMANDS,
        cmd_pmi::COMMANDS,
        cmd_sheets::COMMANDS,
        cmd_wire_harness::COMMANDS,
        cmd_cam::COMMANDS,
        cmd_exploded::COMMANDS,
        cmd_instructions::COMMANDS,
        cmd_instructions_motion::COMMANDS,
        cmd_simulation::COMMANDS,
        cmd_simulation_play::COMMANDS,
        cmd_cam_sim::COMMANDS,
        cmd_ecad::COMMANDS,
        #[cfg(not(target_arch = "wasm32"))]
        cmd_plm::COMMANDS,
        #[cfg(not(target_arch = "wasm32"))]
        cmd_kicad::COMMANDS,
    ];
    sets.iter().flat_map(|s| s.iter()).collect()
}

pub fn lookup(name: &str) -> Option<&'static CommandSpec> {
    registry().into_iter().find(|c| c.name == name)
}

/// The registry as JSON — what a host turns into `tools/list` and what the
/// generated docs render: `[{name, group, doc, phase, annotations, argsSchema, resultSchema}]`.
pub fn describe() -> Value {
    Value::Array(
        registry()
            .into_iter()
            .map(|c| {
                serde_json::json!({
                    "name": c.name,
                    "group": c.group,
                    "doc": c.doc,
                    "phase": c.phase,
                    "annotations": c.annotations,
                    "argsSchema": (c.args_schema)(),
                    "resultSchema": (c.result_schema)(),
                })
            })
            .collect(),
    )
}

// --- wire -------------------------------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Envelope {
    pub id: u64,
    pub cmd: String,
    #[serde(default)]
    pub args: Value,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, schemars::JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum NoticeKind {
    Panic,
    LogError,
    RunnerRefusal,
    Toast,
    Console,
}

#[derive(Debug, Clone, Serialize, Deserialize, schemars::JsonSchema)]
pub struct Notice {
    pub kind: NoticeKind,
    pub frame: u64,
    pub text: String,
}

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
    pub notices: Vec<Notice>,
    /// A binary payload beside the JSON (a PNG). In-process hosts pass it
    /// through; the dial-in adapter sends it as a binary frame.
    #[serde(skip)]
    pub blob: Option<Vec<u8>>,
}

impl Reply {
    pub fn ok(id: u64, frame: u64, result: Value, notices: Vec<Notice>) -> Self {
        Self { id, frame, ok: true, result: Some(result), error: None, notices, blob: None }
    }
    pub fn err(id: u64, frame: u64, error: impl Into<String>, notices: Vec<Notice>) -> Self {
        Self { id, frame, ok: false, result: None, error: Some(error.into()), notices, blob: None }
    }

    /// This reply in the MCP host protocol's shape: the same JSON, the blob
    /// beside it. The one conversion the window embed (`mcp.rs`), the WebMCP
    /// host (`webmcp.rs`) and BREP_mcp's headless host all make; the two
    /// `Reply` types stay mirrored because `brep_mcp_core` must not depend on
    /// the app (`BREP_mcp_core/src/host.rs`).
    pub fn into_host(mut self) -> brep_mcp_core::host::Reply {
        use brep_mcp_core::host::Reply as HostReply;
        let blob = self.blob.take();
        let (id, frame) = (self.id, self.frame);
        match serde_json::to_value(&self).map_err(|e| e.to_string()).and_then(|v| HostReply::from_app(v, blob)) {
            Ok(reply) => reply,
            Err(e) => HostReply::err(id, frame, e),
        }
    }
}

