//! Render/UI settings: the same JSON the Settings panel edits.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SettingsSetArgs {
    /// A partial settings object; unknown or absent keys keep their values.
    pub patch: Value,
}

fn settings_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "settings": parse(&ctx.app.docs.engine().settings_json()) })))
}

fn settings_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SettingsSetArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.patch).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().apply_settings_json(&text)?;
    Ok(Outcome::Done(json!({ "settings": parse(&ctx.app.docs.engine().settings_json()) })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "settings_get", group: "settings", doc: "The render and UI settings (theme, colours, edge width, workbench, …).", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(settings_get) },
    CommandSpec { name: "settings_set", group: "settings", doc: "Apply a partial settings patch (the keys `settings_get` returns).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<SettingsSetArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(settings_set) },
];
