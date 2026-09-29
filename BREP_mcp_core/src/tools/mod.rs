//! The server's view of a tool.
//!
//! A [`ToolSpec`] is what every registry entry becomes on its way to
//! `tools/list`: a name, a group, a doc string, a JSON Schema for its input, MCP
//! annotations, and an async handler. Nothing in this module enumerates engine
//! facts; the sets at the bottom *read* them. When the app's command registry
//! lands (Appendix A) an adapter turns each `CommandSpec` into a `ToolSpec` in
//! exactly this form, so `tools/list` stays one uniform walk.
pub mod app;
pub mod compose;

use crate::schema;
use serde_json::{json, Map, Value};
use std::{future::Future, pin::Pin, sync::Arc};

pub type ToolFuture = Pin<Box<dyn Future<Output = Result<ToolOutput, String>> + Send>>;

/// What a tool call produced: a JSON object (always) and zero or more images.
#[derive(Default)]
pub struct ToolOutput {
    pub json: Value,
    pub images: Vec<ToolImage>,
}

pub struct ToolImage {
    pub png: Vec<u8>,
    pub mime: &'static str,
}

impl ToolOutput {
    pub fn json(v: Value) -> Self {
        Self { json: v, images: Vec::new() }
    }
}

/// MCP tool annotations plus the server's own `waits` flag (the tool applies
/// the idle contract before returning).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Annotations {
    pub read_only: bool,
    pub destructive: bool,
    pub idempotent: bool,
    pub waits: bool,
}

impl Annotations {
    pub const READ: Annotations = Annotations { read_only: true, destructive: false, idempotent: true, waits: false };
    pub const MUTATE: Annotations = Annotations { read_only: false, destructive: false, idempotent: false, waits: true };
    pub const DESTRUCTIVE: Annotations = Annotations { read_only: false, destructive: true, idempotent: false, waits: true };
}

#[derive(Clone)]
pub struct ToolSpec {
    pub name: String,
    pub group: &'static str,
    pub doc: String,
    pub input_schema: Value,
    pub annotations: Annotations,
    pub handler: Arc<dyn Fn(Value) -> ToolFuture + Send + Sync>,
}

impl ToolSpec {
    pub fn new(
        name: impl Into<String>,
        group: &'static str,
        doc: impl Into<String>,
        input_schema: Value,
        annotations: Annotations,
        handler: impl Fn(Value) -> ToolFuture + Send + Sync + 'static,
    ) -> Self {
        Self {
            name: name.into(),
            group,
            doc: doc.into(),
            input_schema,
            annotations,
            handler: Arc::new(handler),
        }
    }
}

impl std::fmt::Debug for ToolSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ToolSpec")
            .field("name", &self.name)
            .field("group", &self.group)
            .finish()
    }
}

/// The tools a session currently exposes. Rebuilt whenever a registry changes;
/// the server announces `tools/list_changed` when it does.
#[derive(Debug, Default)]
pub struct ToolSet {
    pub specs: Vec<ToolSpec>,
}

impl ToolSet {
    pub fn new(specs: Vec<ToolSpec>) -> Self {
        Self { specs }
    }
    pub fn extend(&mut self, more: Vec<ToolSpec>) {
        self.specs.extend(more);
    }
    pub fn get(&self, name: &str) -> Option<&ToolSpec> {
        self.specs.iter().find(|s| s.name == name)
    }
    pub fn names(&self) -> Vec<&str> {
        self.specs.iter().map(|s| s.name.as_str()).collect()
    }
}

/// `{"type":"object","properties":…,"required":[…],"additionalProperties":false}`.
pub fn object_schema(properties: Value, required: &[&str]) -> Value {
    json!({
        "type": "object",
        "properties": properties,
        "required": required,
        "additionalProperties": false,
    })
}

fn arg_str(args: &Value, key: &str) -> Result<String, String> {
    args.get(key)
        .and_then(Value::as_str)
        .map(str::to_string)
        .ok_or_else(|| format!("missing string argument `{key}`"))
}

/// Tools that need no app session: they read the kernel's feature catalogue
/// directly. Everything else arrives through the app's command registry.
pub fn engine_tools() -> Vec<ToolSpec> {
    vec![
        ToolSpec::new(
            "feature_catalogue",
            "features",
            "List every feature the kernel catalogue exposes: type, shortName, longName, displayBuilder. \
             Read `brep://schema/features/{type}` or call `feature_schema` for a feature's parameter schema.",
            object_schema(json!({}), &[]),
            Annotations::READ,
            |_args| {
                Box::pin(async move {
                    let features: Vec<Value> = schema::entries()
                        .iter()
                        .map(|e| {
                            let id = schema::identity(e);
                            json!({
                                "type": id.feature_type,
                                "shortName": id.short_name,
                                "longName": id.long_name,
                                "displayBuilder": id.display_builder,
                            })
                        })
                        .collect();
                    Ok(ToolOutput::json(json!({ "features": features })))
                })
            },
        ),
        ToolSpec::new(
            "feature_schema",
            "features",
            "The JSON Schema of one feature's inputParams (derived from the kernel's inputParamsSchema) \
             and its default parameter values. `type` is the catalogue type or shortName, e.g. `E`, `P.CU`, `B`. \
             A feature that carries a `persistentData` block (the sketch profile Extrude and Revolve consume) also \
             returns `persistentData` — its schema — and `persistentDataExample`, a complete working block.",
            object_schema(json!({ "type": { "type": "string", "description": "feature type or shortName" } }), &["type"]),
            Annotations::READ,
            |args| {
                Box::pin(async move {
                    let ty = arg_str(&args, "type")?;
                    let entry = schema::entry(&ty).ok_or_else(|| format!("unknown feature type `{ty}`"))?;
                    let id = schema::identity(&entry);
                    let mut out = json!({
                        "type": id.feature_type,
                        "longName": id.long_name,
                        "schema": schema::to_json_schema(&entry),
                        "defaults": schema::defaults(&id.feature_type),
                    });
                    // `inputParamsSchema` describes inputParams and nothing else,
                    // so a sketch's geometry — the part a caller cannot guess —
                    // has to be published beside it.
                    if schema::carries_sketch(&id.feature_type) {
                        out["persistentData"] = schema::sketch_persistent_schema();
                        out["persistentDataExample"] = schema::sketch_example();
                    }
                    Ok(ToolOutput::json(out))
                })
            },
        ),
    ]
}

/// Helper for handlers: the arguments as an object map.
pub fn args_object(args: &Value) -> Map<String, Value> {
    args.as_object().cloned().unwrap_or_default()
}

