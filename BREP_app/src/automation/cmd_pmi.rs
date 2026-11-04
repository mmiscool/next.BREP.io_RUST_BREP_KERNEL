//! The PMI workbench: annotation VIEWS and the annotations inside them.
//!
//! A PMI view captures a camera, a visibility set and a display style; every
//! annotation belongs to one. These commands are the operations
//! `panels::pmi` performs, on the same engine methods, so an agent authors the
//! same document a person does. The annotation TYPES are not listed here —
//! `pmi_catalogue` returns the kernel's own schema catalogue, which is what
//! `pmi_add_annotation` takes.
use crate::automation::command::{app_command, engine_call, read_engine, parse_args, schema_of, CommandSpec, Ctx, NoArgs, Outcome};
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
    /// The solid names hidden in this view at capture (its baseline; the view's `hide` steps add to it).
    #[serde(default)]
    pub hidden: Option<Vec<String>>,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ViewSectionArgs {
    pub id: String,
    /// Turn the section on or off; off removes it and ignores the rest.
    pub enabled: bool,
    /// The plane's normal: `View` (the captured camera's direction), `X`, `Y`,
    /// `Z`, or `Plane` (the face or plane named by `plane`).
    #[serde(default)]
    pub axis: Option<String>,
    /// How far the cut lies from its anchor, mm: the captured camera target,
    /// or along a `Plane`'s outward normal.
    #[serde(default)]
    pub offset: Option<f64>,
    /// Keep the other side.
    #[serde(default)]
    pub flip: Option<bool>,
    /// A planar face or a datum or sketch plane to cut on, by name; with no
    /// `axis` it sets `Plane`.
    #[serde(default)]
    pub plane: Option<String>,
    /// Solids or components the cut leaves whole (the form's Not cut).
    #[serde(default)]
    pub exclude: Option<Vec<String>>,
    /// Further planes, replacing the ones the section has (`[]` removes
    /// them): material is removed only where every plane removes it, so one
    /// plane more is a half section (a quarter cut away), two a corner cut away.
    #[serde(default)]
    pub planes: Option<Vec<SectionPlaneArgs>>,
}

/// One further plane of a section: an axis through the section's origin.
#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SectionPlaneArgs {
    /// `View`, `X`, `Y` or `Z`.
    pub axis: String,
    /// mm from the section's origin, along the axis.
    #[serde(default)]
    pub offset: f64,
    /// Keep the other side of this plane.
    #[serde(default)]
    pub flip: bool,
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
    read_engine(ctx, args, |e| e.pmi_state_value())
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
    engine_call(ctx, args, |e, a: ViewIdArgs| e.pmi_activate_view(&a.id))
}

fn pmi_deactivate_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_deactivate_view();
    Ok(Outcome::Done(json!({})))
}

fn pmi_delete_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: ViewIdArgs| e.pmi_delete_view(&a.id))
}

fn pmi_rename_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: RenameViewArgs| e.pmi_rename_view(&a.id, &a.name))
}

fn pmi_update_view_camera(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: ViewIdArgs| e.pmi_update_view_camera(&a.id))
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

fn pmi_set_view_section(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewSectionArgs = parse_args(args)?;
    ctx.app.docs.engine_mut().pmi_set_view_section(
        &a.id,
        a.enabled,
        a.axis.as_deref(),
        a.offset,
        a.flip,
        a.plane.as_deref(),
        a.exclude,
        a.planes.map(|planes| {
            planes
                .into_iter()
                .map(|plane| brep_render::brep_kernel::feature_pipeline::pmi::PmiSectionPlane { axis: plane.axis, offset: plane.offset, flip: plane.flip, normal: None })
                .collect()
        }),
    )?;
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
    engine_call(ctx, args, |e, a: AnnotationIdArgs| e.pmi_remove_annotation(&a.id))
}

fn pmi_set_annotation_enabled(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: AnnotationEnabledArgs| e.pmi_set_annotation_enabled(&a.id, a.enabled))
}

fn pmi_move_annotation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: AnnotationMoveArgs| e.pmi_move_annotation(&a.id, a.index))
}

fn pmi_move_annotation_to_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    engine_call(ctx, args, |e, a: AnnotationToViewArgs| e.pmi_move_annotation_to_view(&a.id, &a.view_id))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("pmi", "pmi_state", "The document's PMI block: every view with its annotations, the active view, the datum letters in use and the last run's PMI report.", NoArgs, pmi_state, Read, READ, super::pmi_output_schema::pmi_state_schema),
    app_command!("pmi", "pmi_catalogue", "Every PMI annotation type the kernel offers, with its parameter schema — the types `pmi_add_annotation` takes.", NoArgs, pmi_catalogue, Read, READ, super::catalogue_output_schema::pmi_catalogue_schema),
    app_command!("pmi", "pmi_capture_view", "Capture the CURRENT camera, visibility and display style as a new PMI view (point the camera first with `standard_view` / `camera_set`). Returns the view id.", CaptureViewArgs, pmi_capture_view, Mutate, MUTATE, schema_of::<PmiAdded>),
    app_command!("pmi", "pmi_activate_view", "Enter a PMI view: the camera, visibility and style become the view's, and new annotations land in it.", ViewIdArgs, pmi_activate_view, Mutate, MUTATE),
    app_command!("pmi", "pmi_deactivate_view", "Leave the active PMI view and restore the modeling camera and visibility.", NoArgs, pmi_deactivate_view, Mutate, MUTATE),
    app_command!("pmi", "pmi_delete_view", "Delete a PMI view and every annotation in it.", ViewIdArgs, pmi_delete_view, Mutate, DESTRUCTIVE),
    app_command!("pmi", "pmi_rename_view", "Rename a PMI view.", RenameViewArgs, pmi_rename_view, Mutate, MUTATE),
    app_command!("pmi", "pmi_update_view_camera", "Re-capture the live camera into a view (the panel's Update Camera).", ViewIdArgs, pmi_update_view_camera, Mutate, MUTATE),
    app_command!("pmi", "pmi_set_view_display", "Patch a view's name, annotation text size, wireframe flag or captured hidden-solid baseline (`hidden`; what a view hides is this list plus its `hide` steps — add those with `pmi_add_annotation` type `hide`); absent fields are unchanged.", ViewDisplayArgs, pmi_set_view_display, Mutate, MUTATE),
    app_command!("pmi", "pmi_set_view_section", "Set a view's section as its form's section rows do, as one undo step: on or off, the normal (View, X, Y, Z or Plane), the offset in mm, which side is kept, the face or plane a Plane section cuts on, the solids or components it leaves whole, and further planes (`planes`: a half or corner cut away; material is removed only where every plane removes it). Absent fields keep the section's current value.", ViewSectionArgs, pmi_set_view_section, Mutate, MUTATE),
    app_command!("pmi", "pmi_add_annotation", "Add an annotation to a view (the active one when `view_id` is absent) with the params its catalogue schema defines; every other key is seeded from the schema default. Returns the annotation id.", AddAnnotationArgs, pmi_add_annotation, Mutate, MUTATE, schema_of::<PmiAdded>),
    app_command!("pmi", "pmi_update_annotation", "Replace an annotation's params and re-bake the overlay.", UpdateAnnotationArgs, pmi_update_annotation, Mutate, MUTATE),
    app_command!("pmi", "pmi_remove_annotation", "Delete one annotation.", AnnotationIdArgs, pmi_remove_annotation, Mutate, DESTRUCTIVE),
    app_command!("pmi", "pmi_set_annotation_enabled", "Suppress or re-enable one annotation.", AnnotationEnabledArgs, pmi_set_annotation_enabled, Mutate, MUTATE),
    app_command!("pmi", "pmi_move_annotation", "Reorder an annotation within its view.", AnnotationMoveArgs, pmi_move_annotation, Mutate, MUTATE),
    app_command!("pmi", "pmi_move_annotation_to_view", "Move an annotation into another view.", AnnotationToViewArgs, pmi_move_annotation_to_view, Mutate, MUTATE),
];
