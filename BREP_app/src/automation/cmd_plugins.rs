//! MCP/app automation uses the same registry and transaction path as plugin UI.
use crate::automation::command::{
    parse_args, schema_of, Annotations, CommandSpec, Ctx, Handler, NoArgs, Outcome, Phase,
};
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct InstallArgs {
    bundle: Value,
}
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct VersionArgs {
    id: String,
    digest: String,
}
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct EnableArgs {
    id: String,
    digest: String,
    enabled: bool,
}
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct ActionArgs {
    id: String,
    #[serde(default = "empty_params")]
    params: Value,
}
fn empty_params() -> Value {
    json!({})
}
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct SchemaArgs {
    r#type: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct WindowArgs {
    open: bool,
}
fn window(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: WindowArgs = parse_args(args)?;
    ctx.app.plugins.open = a.open;
    Ok(Outcome::Done(json!({})))
}

fn javascript_window(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: WindowArgs = parse_args(args)?;
    ctx.app.javascript.open = a.open;
    Ok(Outcome::Done(json!({})))
}

fn catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(ctx.app.docs.engine().feature_catalogue()))
}
fn schema(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SchemaArgs = parse_args(args)?;
    ctx.app
        .docs
        .engine()
        .feature_schema(&a.r#type)
        .map(Outcome::Done)
        .ok_or_else(|| format!("Unavailable feature type {}", a.r#type))
}
fn describe(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(
        json!({ "installed": ctx.app.docs.engine().installed_plugins(), "catalogue": ctx.app.docs.engine().plugin_catalogue(), "action": ctx.app.docs.engine().plugin_action_status() }),
    ))
}
fn install(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: InstallArgs = parse_args(args)?;
    let app = &mut ctx.app;
    app.plugins.active_document = app.docs.active_id();
    let id = app.plugins.install(
        app.docs.engine_mut(),
        app.model_store.as_ref(),
        &serde_json::to_vec(&a.bundle).map_err(|e| e.to_string())?,
        None,
    )?;
    Ok(Outcome::Done(json!({"id":id,"pending":true})))
}
fn select(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VersionArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().select_plugin(&a.id, &a.digest)?;
    Ok(Outcome::Done(json!({})))
}
fn migrate(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VersionArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().migrate_plugin(&a.id, &a.digest)?;
    Ok(Outcome::Done(json!({"id": id})))
}
fn export(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VersionArgs = parse_args(args)?;
    Ok(Outcome::Done(
        ctx.app.docs.engine().export_plugin(&a.id, &a.digest)?,
    ))
}
fn change(
    ctx: &mut Ctx<'_>,
    id: &str,
    digest: &str,
    enabled: Option<bool>,
) -> Result<Outcome, String> {
    let app = &mut ctx.app;
    let package = app
        .plugins
        .packages
        .borrow()
        .iter()
        .find(|p| p.id == id && p.digest == digest)
        .cloned()
        .ok_or("Package is not installed")?;
    app.plugins.active_document = app.docs.active_id();
    app.plugins.error = None;
    app.plugins.change(
        app.docs.engine_mut(),
        app.model_store.as_ref(),
        &package,
        enabled,
    );
    if let Some(error) = &app.plugins.error {
        return Err(error.clone());
    }
    Ok(Outcome::Done(
        app.docs.engine().plugin_installation_status(),
    ))
}
fn enable(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: EnableArgs = parse_args(args)?;
    change(ctx, &a.id, &a.digest, Some(a.enabled))
}
fn remove(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: VersionArgs = parse_args(args)?;
    change(ctx, &a.id, &a.digest, None)
}
fn action(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ActionArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine_mut();
    let selection = serde_json::from_str(&engine.selection_json()).unwrap_or(Value::Null);
    let id = engine.plugin_action(&a.id, a.params, selection)?;
    Ok(Outcome::Done(json!({"id": id, "pending": true})))
}
fn status(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(ctx.app.docs.engine().plugin_action_status()))
}
fn installation_status(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app
        .plugins
        .finish_installations(&mut ctx.app.docs, ctx.app.model_store.as_ref());
    let mut status = ctx.app.docs.engine().plugin_installation_status();
    if status.is_object() {
        status["installed"] = ctx.app.docs.engine().installed_plugins();
    }
    if let Some(error) = &ctx.app.plugins.storage_error {
        status["storageError"] = json!(error);
    }
    Ok(Outcome::Done(status))
}

fn cancel(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(
        json!({"cancelled": ctx.app.docs.engine_mut().cancel_plugin_action()}),
    ))
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
struct AnnotationCreateArgs {
    view_id: String,
    r#type: String,
    #[serde(default = "empty_params")]
    params: Value,
}
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct IdArgs { id: String }
fn panels(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let engine = ctx.app.docs.engine();
    Ok(Outcome::Done(json!(crate::workbench::plugin_panels(engine, &engine.settings.workbench))))
}
fn panel_show(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    ctx.app.show_plugin_panel(&a.id)?;
    Ok(Outcome::Done(json!({})))
}
fn annotation_catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(ctx.app.docs.engine().plugin_annotation_catalogue()))
}
fn annotation_create(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnnotationCreateArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().plugin_add_annotation(&a.view_id, &a.r#type, a.params)?;
    Ok(Outcome::Done(json!({"id": id, "pending": true})))
}
fn annotation_update(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ActionArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().plugin_update_annotation(&a.id, a.params)?;
    Ok(Outcome::Done(json!({"id": id, "pending": true})))
}
fn annotation_delete(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IdArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().plugin_delete_annotation(&a.id)?;
    Ok(Outcome::Done(json!({"id": id, "pending": true})))
}

macro_rules! command {
    ($name:literal, $doc:literal, $args:ty, $handler:ident, $phase:ident, $annotation:ident) => {
        CommandSpec {
            name: $name,
            group: "plugins",
            doc: $doc,
            phase: Phase::$phase,
            annotations: Annotations::$annotation,
            args_schema: schema_of::<$args>,
            result_schema: schema_of::<Value>,
            handler: Handler::App($handler),
        }
    };
}
pub static COMMANDS: &[CommandSpec] = &[
    command!("javascript_window", "Open or close the in-app JavaScript editor.", WindowArgs, javascript_window, Mutate, MUTATE_NOWAIT),
    command!("plugin_panel_show", "Activate a claimed plugin panel tab by its stable ID without changing the dock arrangement.", IdArgs, panel_show, Mutate, MUTATE_NOWAIT),
    command!("plugin_panels", "Validated declarative panels claimed by the active workbench, with text/table/form/action controls. Forms use the named action schema; dispatch via plugin_action.", NoArgs, panels, Read, READ),
    command!("plugin_annotation_catalogue", "Document-scoped typed plugin annotation schemas used by PMI authoring forms.", NoArgs, annotation_catalogue, Read, READ),
    command!("plugin_annotation_create", "Queue typed annotation creation in an existing PMI view with schema params. Poll plugin_action_status; successful replay commits one undo checkpoint.", AnnotationCreateArgs, annotation_create, Mutate, MUTATE_NOWAIT),
    command!("plugin_annotation_update", "Queue parameter replacement for a typed plugin annotation. Poll plugin_action_status; failures preserve the prior annotation.", ActionArgs, annotation_update, Mutate, MUTATE_NOWAIT),
    command!("plugin_annotation_delete", "Queue deletion of a typed plugin annotation, including an unavailable provider's preserved record. Poll plugin_action_status.", IdArgs, annotation_delete, Mutate, MUTATE_NOWAIT),
    command!("plugins_window", "Open or close the installed plugins manager.", WindowArgs, window, Mutate, MUTATE_NOWAIT),
    command!("plugin_installation_status", "Read async package validation status, installed versions and persistence errors. Installation is complete only after state is complete with no storageError.", NoArgs, installation_status, Read, READ),
    command!("plugin_feature_catalogue", "Document-scoped feature catalogue, including enabled JavaScript contributions, in normal feature catalogue format.", NoArgs, catalogue, Read, READ),
    command!("plugin_feature_schema", "Read the normal feature catalogue entry for a built-in or document-scoped plugin type.", SchemaArgs, schema, Read, READ),
    command!("plugin_describe", "Installed exact versions, scoped feature/action/workbench registrations and async action status.", NoArgs, describe, Read, READ),
    command!("plugin_install", "Explicitly install a portable bundle {manifest,modules,assets}; persists exact package content locally. Does not migrate document pins.", InstallArgs, install, Mutate, MUTATE_NOWAIT),
    command!("plugin_enable", "Enable or disable one installed digest; disabled packages preserve history and block affected recompute.", EnableArgs, enable, Mutate, MUTATE_NOWAIT),
    command!("plugin_remove", "Remove an installed digest without removing document history.", VersionArgs, remove, Mutate, MUTATE_NOWAIT),
    command!("plugin_select", "Explicitly select an installed exact package version for the current document.", VersionArgs, select, Mutate, MUTATE_NOWAIT),
    command!("plugin_migrate", "Queue an undoable document migration to an installed exact version; poll plugin_action_status.", VersionArgs, migrate, Mutate, MUTATE_NOWAIT),
    command!("plugin_export", "Export the portable bundle for an installed exact version.", VersionArgs, export, Read, READ),
    command!("plugin_action", "Submit a named action with schema parameters and current selection to the shared async transaction runner. Poll plugin_action_status for completion or error.", ActionArgs, action, Mutate, MUTATE_NOWAIT),
    command!("plugin_action_status", "Read the queued action result, pending state or refusal without consuming it.", NoArgs, status, Read, READ),
    command!("plugin_action_cancel", "Cancel the pending plugin action without committing staged edits.", NoArgs, cancel, Mutate, MUTATE_NOWAIT),
];
