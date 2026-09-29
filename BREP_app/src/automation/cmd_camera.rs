//! Camera: read / set the state, fit, standard views, projection.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, NoArgs, Outcome, Phase};
use crate::automation::cmd_document::parse;
use serde::Deserialize;
use serde_json::{json, Value};

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CameraSetArgs {
    /// A camera state as returned by `camera_get` (partial fields allowed where the engine accepts them).
    pub state: Value,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ViewArgs {
    /// FRONT | BACK | LEFT | RIGHT | TOP | BOTTOM | ISO
    pub name: String,
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ProjectionArgs {
    /// `perspective` | `orthographic`
    pub kind: String,
}

fn camera_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(json!({ "camera": parse(&ctx.app.docs.engine().camera_state_json()) })))
}

fn camera_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: CameraSetArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.state).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().apply_camera_state_json(&text)?;
    Ok(Outcome::Done(json!({ "camera": parse(&ctx.app.docs.engine().camera_state_json()) })))
}

fn zoom_to_fit(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    // The app's own branch, so the command frames what the tile is DRAWING —
    // the open sheet's paper, or the 3D scene — exactly as the toolbar button
    // beside it does.
    ctx.app.zoom_to_fit();
    Ok(Outcome::Done(json!({ "camera": parse(&ctx.app.docs.engine().camera_state_json()) })))
}

fn standard_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewArgs = parse_args(args)?;
    if !ctx.app.docs.engine_mut().standard_view(&a.name.to_ascii_uppercase()) {
        return Err(format!("unknown view `{}` (FRONT BACK LEFT RIGHT TOP BOTTOM ISO)", a.name));
    }
    Ok(Outcome::Done(json!({ "camera": parse(&ctx.app.docs.engine().camera_state_json()) })))
}

fn set_projection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ProjectionArgs = parse_args(args)?;
    let k = a.kind.to_ascii_lowercase();
    if !(k.starts_with("pers") || k.starts_with("orth")) {
        return Err("kind must be `perspective` or `orthographic`".into());
    }
    ctx.app.docs.engine_mut().set_projection(&k);
    Ok(Outcome::Done(json!({ "camera": parse(&ctx.app.docs.engine().camera_state_json()) })))
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "camera_get", group: "camera", doc: "The camera state: kind, eye, target, up, near/far, projection block, worldPerPixel.", phase: Phase::Read, annotations: Annotations::READ, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(camera_get) },
    CommandSpec { name: "camera_set", group: "camera", doc: "Apply a camera state (the shape `camera_get` returns).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<CameraSetArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(camera_set) },
    CommandSpec { name: "zoom_to_fit", group: "camera", doc: "Frame the whole model \u{2014} or, while a drawing sheet is open, the sheet's whole paper (the toolbar's Zoom-to-fit button is this same branch).", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<NoArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(zoom_to_fit) },
    CommandSpec { name: "standard_view", group: "camera", doc: "Snap to a standard view and fit: FRONT BACK LEFT RIGHT TOP BOTTOM ISO.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ViewArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(standard_view) },
    CommandSpec { name: "set_projection", group: "camera", doc: "Switch between perspective and orthographic projection.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<ProjectionArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(set_projection) },
];
