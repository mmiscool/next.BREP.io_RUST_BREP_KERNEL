//! The PMI workbench: annotation VIEWS and the annotations inside them.
//!
//! A PMI view captures a camera, a visibility set and a display style; every
//! annotation belongs to one. These commands are the operations
//! `panels::pmi` performs, on the same engine methods, so an agent authors the
//! same document a person does. The annotation TYPES are not listed here —
//! `pmi_catalogue` returns the kernel's own schema catalogue, which is what
//! `pmi_add_annotation` takes.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use brep_render::engine_state::PmiViewPatch;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CaptureViewArgs {
    /// The view's name; a default (`View N`) is minted when absent.
    #[serde(default)]
    pub name: Option<String>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ViewIdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RenameViewArgs {
    pub id: String,
    pub name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ViewDisplayArgs {
    pub id: String,
    /// Rename the view.
    #[serde(default)]
    pub name: Option<String>,
    /// Annotation text height in points.
    #[serde(default)]
    pub text_size_pt: Option<f64>,
    /// Show the view's model as wireframe.
    #[serde(default)]
    pub wireframe: Option<bool>,
    /// The solid names hidden in this view.
    #[serde(default)]
    pub hidden: Option<Vec<String>>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddAnnotationArgs {
    /// The view to add to; the active view when absent.
    #[serde(default)]
    pub view_id: Option<String>,
    /// The annotation type id from `pmi_catalogue`.
    #[serde(rename = "type")]
    pub annotation_type: String,
    /// The annotation's params; every other key is seeded from its schema default.
    #[serde(default)]
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct UpdateAnnotationArgs {
    pub id: String,
    /// The annotation's complete new params.
    pub params: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnnotationIdArgs {
    pub id: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnnotationEnabledArgs {
    pub id: String,
    pub enabled: bool,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnnotationMoveArgs {
    pub id: String,
    /// The annotation's new position within its view.
    pub index: usize,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AnnotationToViewArgs {
    pub id: String,
    /// The view to move the annotation into.
    pub view_id: String,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct PmiAdded {
    /// The id the engine minted.
    pub id: String,
}

fn pmi_state(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(parse(&ctx.app.docs.engine().pmi_state_json())))
}

fn pmi_catalogue(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    let _ = ctx;
    Ok(Outcome::Done(json!({ "annotations": brep_render::brep_kernel::pmi_schema_catalogue() })))
}

fn pmi_capture_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: CaptureViewArgs = parse_args(args)?;
    let id = ctx.app.docs.engine_mut().pmi_capture_view(a.name.as_deref());
    serde_json::to_value(PmiAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn pmi_activate_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_activate_view(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_deactivate_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_deactivate_view();
    Ok(Outcome::Done(json!({})))
}

fn pmi_delete_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_delete_view(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_rename_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: RenameViewArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_rename_view(&a.id, &a.name)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_update_view_camera(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_update_view_camera(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_update_view_visibility(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_update_view_visibility(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_set_view_display(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewDisplayArgs = parse_args(args)?;
    let patch = PmiViewPatch {
        section: None,
        name: a.name,
        text_size_pt: a.text_size_pt,
        wireframe: a.wireframe,
        hidden: a.hidden,
    };
    ctx.app.docs.engine_mut().pmi_set_view_display(&a.id, &patch)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_add_annotation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AddAnnotationArgs = parse_args(args)?;
    let params = serde_json::to_string(&a.params).map_err(|e| e.to_string())?;
    let id = ctx
        .app
        .docs
        .engine_mut()
        .pmi_add_annotation(a.view_id.as_deref(), &a.annotation_type, &params)?;
    serde_json::to_value(PmiAdded { id }).map(Outcome::Done).map_err(|e| e.to_string())
}

fn pmi_update_annotation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: UpdateAnnotationArgs = parse_args(args)?;
    let params = serde_json::to_string(&a.params).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().pmi_update_annotation(&a.id, &params)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_remove_annotation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnnotationIdArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_remove_annotation(&a.id)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_set_annotation_enabled(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnnotationEnabledArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_set_annotation_enabled(&a.id, a.enabled)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_move_annotation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnnotationMoveArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_move_annotation(&a.id, a.index)?;
    Ok(Outcome::Done(json!({})))
}

fn pmi_move_annotation_to_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: AnnotationToViewArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_move_annotation_to_view(&a.id, &a.view_id)?;
    Ok(Outcome::Done(json!({})))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "pmi_state", group: "pmi", doc: "The document's PMI block: every view with its annotations, the active view, the datum letters in use and the last run's PMI report.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_state) },
    CommandSpec { name: "pmi_catalogue", group: "pmi", doc: "Every PMI annotation type the kernel offers, with its parameter schema — the types `pmi_add_annotation` takes.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_catalogue) },
    CommandSpec { name: "pmi_capture_view", group: "pmi", doc: "Capture the CURRENT camera, visibility and display style as a new PMI view (point the camera first with `standard_view` / `camera_set`). Returns the view id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<CaptureViewArgs>, result_schema: schema_of::<PmiAdded>, handler: Handler::App(pmi_capture_view) },
    CommandSpec { name: "pmi_activate_view", group: "pmi", doc: "Enter a PMI view: the camera, visibility and style become the view's, and new annotations land in it.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ViewIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_activate_view) },
    CommandSpec { name: "pmi_deactivate_view", group: "pmi", doc: "Leave the active PMI view and restore the modeling camera and visibility.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_deactivate_view) },
    CommandSpec { name: "pmi_delete_view", group: "pmi", doc: "Delete a PMI view and every annotation in it.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<ViewIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_delete_view) },
    CommandSpec { name: "pmi_rename_view", group: "pmi", doc: "Rename a PMI view.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<RenameViewArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_rename_view) },
    CommandSpec { name: "pmi_update_view_camera", group: "pmi", doc: "Re-capture the live camera into a view (the panel's Update Camera).", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ViewIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_update_view_camera) },
    CommandSpec { name: "pmi_update_view_visibility", group: "pmi", doc: "Re-capture the live visibility set into a view (the panel's Update Visibility).", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ViewIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_update_view_visibility) },
    CommandSpec { name: "pmi_set_view_display", group: "pmi", doc: "Patch a view's name, annotation text size, wireframe flag or hidden-solid set; absent fields are unchanged.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<ViewDisplayArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_set_view_display) },
    CommandSpec { name: "pmi_add_annotation", group: "pmi", doc: "Add an annotation to a view (the active one when `view_id` is absent) with the params its catalogue schema defines; every other key is seeded from the schema default. Returns the annotation id.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AddAnnotationArgs>, result_schema: schema_of::<PmiAdded>, handler: Handler::App(pmi_add_annotation) },
    CommandSpec { name: "pmi_update_annotation", group: "pmi", doc: "Replace an annotation's params and re-bake the overlay.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<UpdateAnnotationArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_update_annotation) },
    CommandSpec { name: "pmi_remove_annotation", group: "pmi", doc: "Delete one annotation.", phase: Phase::Mutate, annotations: Annotations::DESTRUCTIVE, args_schema: schema_of::<AnnotationIdArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_remove_annotation) },
    CommandSpec { name: "pmi_set_annotation_enabled", group: "pmi", doc: "Suppress or re-enable one annotation.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AnnotationEnabledArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_set_annotation_enabled) },
    CommandSpec { name: "pmi_move_annotation", group: "pmi", doc: "Reorder an annotation within its view.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AnnotationMoveArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_move_annotation) },
    CommandSpec { name: "pmi_move_annotation_to_view", group: "pmi", doc: "Move an annotation into another view.", phase: Phase::Mutate, annotations: Annotations::MUTATE, args_schema: schema_of::<AnnotationToViewArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(pmi_move_annotation_to_view) },
];
