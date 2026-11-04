//! MCP/app automation uses the same registry and transaction path as plugin UI.
use crate::automation::command::{app_command, engine_call, read_engine, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
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

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
struct SourceArgs {
    source: String,
}
fn javascript_source(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SourceArgs = parse_args(args)?;
    ctx.app.javascript.set_source(a.source);
    Ok(Outcome::Done(json!({})))
}

fn catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    read_engine(ctx, args, |e| e.feature_catalogue())
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
fn plugin_describe_result_schema() -> Value {
    super::plugin_output_schema::describe_schema(plugin_action_result_schema())
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
    engine_call(ctx, args, |e, a: VersionArgs| e.select_plugin(&a.id, &a.digest))
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
    let selection = engine.selection_value();
    let id = engine.plugin_action(&a.id, a.params, selection)?;
    Ok(Outcome::Done(json!({"id": id, "pending": true})))
}
fn status(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    read_engine(ctx, args, |e| e.plugin_action_status())
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
    read_engine(ctx, args, |e| e.plugin_annotation_catalogue())
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

fn plugin_pending_result_schema() -> Value {
    super::schema_dsl::closed_object_all(json!({"id":{"type":"integer","minimum":0},"pending":{"const":true}}))
}
fn plugin_id_result_schema() -> Value {
    super::schema_dsl::closed_object_all(json!({"id":{"type":"integer","minimum":0}}))
}
fn plugin_cancel_result_schema() -> Value {
    super::schema_dsl::closed_object_all(json!({"cancelled":{"type":"boolean"}}))
}
fn plugin_action_result_schema() -> Value {
    json!({"oneOf":[
        {"type":"null"},
        super::schema_dsl::closed_object_all(json!({"state":{"const":"pending"},"id":{"type":"integer","minimum":0}})),
        super::schema_dsl::closed_object_all(json!({"state":{"const":"complete"},"result":{}})),
        super::schema_dsl::closed_object_all(json!({"state":{"const":"error"},"error":{"type":"string"}}))
    ]})
}

fn plugin_install_change_result_schema() -> Value {
    let id = json!({"type":"integer","minimum":0});
    let mut error = super::schema_dsl::closed_object_all(json!({"state":{"const":"error"},"error":{"type":"string"}}));
    // Replacing a runner can refuse an installation without retaining its id.
    error["properties"]["id"] = id.clone();
    json!({"oneOf":[
        super::schema_dsl::closed_object_all(json!({"state":{"const":"pending"},"id":id})),
        super::schema_dsl::closed_object_all(json!({"state":{"const":"complete"},"id":id})),
        error
    ]})
}

fn plugin_installation_result_schema() -> Value {
    let installed = super::schema_dsl::closed_object_all(json!({
        "id":{"type":"string"},"version":{"type":"string"},
        "apiVersion":{"type":"integer","minimum":0,"maximum":4294967295u64},
        "digest":{"type":"string"},"name":{"type":"string"},"enabled":{"type":"boolean"}
    }));
    let mut schema = plugin_install_change_result_schema();
    let branches = schema["oneOf"].as_array_mut().unwrap();
    for branch in branches.iter_mut() {
        branch["properties"]["installed"] = json!({"type":"array","items":installed});
        branch["required"].as_array_mut().unwrap().push(json!("installed"));
        branch["properties"]["storageError"] = json!({"type":"string"});
    }
    // The initial engine status is null. A storage error assigned to that
    // value creates an object only after the installed-list enrichment.
    branches.push(json!({"type":"null"}));
    branches.push(super::schema_dsl::closed_object_all(json!({"storageError":{"type":"string"}})));
    schema
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("plugins", "javascript_window", "Open or close the in-app JavaScript editor.", WindowArgs, javascript_window, Mutate, MUTATE_NOWAIT),
    app_command!("plugins", "javascript_source", "Replace the JavaScript editor's source, as typing it would (not saved as the draft).", SourceArgs, javascript_source, Mutate, MUTATE_NOWAIT),
    app_command!("plugins", "plugin_panel_show", "Activate a claimed plugin panel tab by its stable ID without changing the dock arrangement.", IdArgs, panel_show, Mutate, MUTATE_NOWAIT),
    app_command!("plugins", "plugin_panels", "Validated declarative panels claimed by the active workbench, with text/table/form/action controls. Forms use the named action schema; dispatch via plugin_action.", NoArgs, panels, Read, READ, super::plugin_output_schema::panels_schema),
    app_command!("plugins", "plugin_annotation_catalogue", "Document-scoped typed plugin annotation schemas used by PMI authoring forms.", NoArgs, annotation_catalogue, Read, READ, super::plugin_output_schema::annotations_schema),
    app_command!("plugins", "plugin_annotation_create", "Queue typed annotation creation in an existing PMI view with schema params. Poll plugin_action_status; successful replay commits one undo checkpoint.", AnnotationCreateArgs, annotation_create, Mutate, MUTATE_NOWAIT, plugin_pending_result_schema),
    app_command!("plugins", "plugin_annotation_update", "Queue parameter replacement for a typed plugin annotation. Poll plugin_action_status; failures preserve the prior annotation.", ActionArgs, annotation_update, Mutate, MUTATE_NOWAIT, plugin_pending_result_schema),
    app_command!("plugins", "plugin_annotation_delete", "Queue deletion of a typed plugin annotation, including an unavailable provider's preserved record. Poll plugin_action_status.", IdArgs, annotation_delete, Mutate, MUTATE_NOWAIT, plugin_pending_result_schema),
    app_command!("plugins", "plugins_window", "Open or close the installed plugins manager.", WindowArgs, window, Mutate, MUTATE_NOWAIT),
    app_command!("plugins", "plugin_installation_status", "Read async package validation status, installed versions and persistence errors. Installation is complete only after state is complete with no storageError.", NoArgs, installation_status, Read, READ, plugin_installation_result_schema),
    app_command!("plugins", "plugin_feature_catalogue", "Document-scoped feature catalogue, including enabled JavaScript contributions, in normal feature catalogue format.", NoArgs, catalogue, Read, READ, super::plugin_output_schema::feature_catalogue_schema),
    app_command!("plugins", "plugin_feature_schema", "Read the normal feature catalogue entry for a built-in or document-scoped plugin type.", SchemaArgs, schema, Read, READ, super::plugin_output_schema::feature_schema),
    app_command!("plugins", "plugin_describe", "Installed exact versions, scoped feature/action/workbench registrations and async action status.", NoArgs, describe, Read, READ, plugin_describe_result_schema),
    app_command!("plugins", "plugin_install", "Explicitly install a portable bundle {manifest,modules,assets}; persists exact package content locally. Does not migrate document pins.", InstallArgs, install, Mutate, MUTATE_NOWAIT, plugin_pending_result_schema),
    app_command!("plugins", "plugin_enable", "Enable or disable one installed digest; disabled packages preserve history and block affected recompute.", EnableArgs, enable, Mutate, MUTATE_NOWAIT, plugin_install_change_result_schema),
    app_command!("plugins", "plugin_remove", "Remove an installed digest without removing document history.", VersionArgs, remove, Mutate, MUTATE_NOWAIT, plugin_install_change_result_schema),
    app_command!("plugins", "plugin_select", "Explicitly select an installed exact package version for the current document.", VersionArgs, select, Mutate, MUTATE_NOWAIT),
    app_command!("plugins", "plugin_migrate", "Queue an undoable document migration to an installed exact version; poll plugin_action_status.", VersionArgs, migrate, Mutate, MUTATE_NOWAIT, plugin_id_result_schema),
    app_command!("plugins", "plugin_export", "Export the portable bundle for an installed exact version.", VersionArgs, export, Read, READ, super::plugin_output_schema::bundle_schema),
    app_command!("plugins", "plugin_action", "Submit a named action with schema parameters and current selection to the shared async transaction runner. Poll plugin_action_status for completion or error.", ActionArgs, action, Mutate, MUTATE_NOWAIT, plugin_pending_result_schema),
    app_command!("plugins", "plugin_action_status", "Read the queued action result, pending state or refusal without consuming it.", NoArgs, status, Read, READ, plugin_action_result_schema),
    app_command!("plugins", "plugin_action_cancel", "Cancel the pending plugin action without committing staged edits.", NoArgs, cancel, Mutate, MUTATE_NOWAIT, plugin_cancel_result_schema),
];
