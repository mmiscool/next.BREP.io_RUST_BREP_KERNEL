//! The name-keyed METADATA store — the Properties/Info window's model.
//!
//! Every solid, face and edge can carry a record of `key -> value` string
//! attributes that live in the document (they are saved under its top-level
//! `metadata` block and restored with it), survive a rebuild, and drive two
//! well-known behaviours the engine reads back:
//!
//! - `color` — the object's shaded colour, as a CSS hex string (`#rrggbb`).
//!   Writing it repaints immediately; the display's colour override is a
//!   derived cache of THIS store, re-derived after every history run, so a
//!   colour set once outlives feature edits and re-tessellation.
//! - `density` — the object's mass in `mass_properties`.
//!
//! Nothing here decides what an attribute means: these commands are the store's
//! read/write surface, the same one `panels::info_windows` drives, so a colour
//! an agent writes and a colour a person picks are the same edit.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NameArgs {
    /// A solid, face or edge name (the names `scene_entities` lists).
    pub name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AttributeArgs {
    /// A solid, face or edge name.
    pub name: String,
    /// The attribute key. `color` (a CSS hex `#rrggbb`) shades the object;
    /// `density` weighs it. Any other key is carried verbatim.
    pub key: String,
    /// The value. Non-strings are stored as their JSON text.
    pub value: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AttributeManyArgs {
    /// The writes to apply, in order. One rebuild-free edit each — this is how
    /// a whole model gets coloured in one call.
    pub items: Vec<AttributeArgs>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RemoveArgs {
    pub name: String,
    pub key: String,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct Written {
    /// How many attributes were written.
    pub written: usize,
}

/// Coerce an attribute value to the string the store holds: strings verbatim,
/// everything else as its JSON text (matching the store's own load path).
fn text_of(value: &Value) -> String {
    match value {
        Value::String(text) => text.clone(),
        Value::Null => String::new(),
        other => other.to_string(),
    }
}

fn metadata_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: NameArgs = parse_args(args)?;
    let record = parse(&ctx.app.docs.engine().object_metadata_json(&a.name));
    Ok(Outcome::Done(json!({ "name": a.name, "attributes": record })))
}

fn metadata_all(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "metadata": parse(&ctx.app.docs.engine().metadata_json()) })))
}

fn metadata_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AttributeArgs = parse_args(args)?;
    if a.key.trim().is_empty() {
        return Err("metadata key is empty".into());
    }
    ctx.app.docs.engine_mut().set_metadata_attribute(&a.name, &a.key, &text_of(&a.value));
    serde_json::to_value(Written { written: 1 }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn metadata_set_many(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AttributeManyArgs = parse_args(args)?;
    // Refuse the whole batch on a malformed key rather than half-applying it.
    if let Some(bad) = a.items.iter().position(|item| item.key.trim().is_empty()) {
        return Err(format!("items[{bad}]: metadata key is empty (nothing was written)"));
    }
    for item in &a.items {
        ctx.app.docs.engine_mut().set_metadata_attribute(&item.name, &item.key, &text_of(&item.value));
    }
    serde_json::to_value(Written { written: a.items.len() }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn metadata_remove(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: RemoveArgs = parse_args(args)?;
    let existed = ctx.app.docs.engine_mut().remove_metadata_attribute(&a.name, &a.key);
    Ok(Outcome::Done(json!({ "removed": existed })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "metadata_get", group: "metadata", doc: "One object's metadata record `{key: value}` (empty when it has none). Names come from `scene_entities`.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NameArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(metadata_get) },
    CommandSpec { name: "metadata_all", group: "metadata", doc: "The whole metadata store, `{name: {key: value}}` — what the document saves and reloads.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(metadata_all) },
    CommandSpec { name: "metadata_set", group: "metadata", doc: "Set one durable metadata attribute on a solid, face or edge. `color` (`#rrggbb`) shades it at once and survives rebuilds; `density` drives `mass_properties`; any other key is carried with the document.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<AttributeArgs>, result_schema: schema_of::<Written>, handler: Handler::App(metadata_set) },
    CommandSpec { name: "metadata_set_many", group: "metadata", doc: "Set many metadata attributes in one call (colouring a whole model is one call, not one per solid). Refused whole if any item has an empty key.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<AttributeManyArgs>, result_schema: schema_of::<Written>, handler: Handler::App(metadata_set_many) },
    CommandSpec { name: "metadata_remove", group: "metadata", doc: "Remove one metadata attribute; returns whether it existed. Removing `color` returns the object to the display's own colouring.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<RemoveArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(metadata_remove) },
];
