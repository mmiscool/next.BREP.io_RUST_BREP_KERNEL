//! Camera: read / set the state, fit, standard views, projection.
use crate::automation::command::{app_command, read_engine, parse_args, CommandSpec, Ctx, NoArgs, Outcome};
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

// Shared by every camera command: ViewCamera::state_json is the producer
// (BREP_render/src/view.rs). Its two projection kinds share this flat shape;
// `scale` means half-height for orthographic and vertical FOV in degrees for
// perspective. Near may be negative; these are not visibility bounds.
fn camera_result_schema() -> Value {
    let vector = json!({"type":"array","items":{"type":"number"},"minItems":3,"maxItems":3});
    let camera = json!({
        "type":"object",
        "properties":{
            "kind":{"enum":["orthographic","perspective"]},
            "eye":vector,"target":vector,"up":vector,
            "scale":{"type":"number","description":"Orthographic half-height or perspective vertical field of view in degrees, selected by kind."},
            "near":{"type":"number"},"far":{"type":"number"},
            "width":{"type":"number"},"height":{"type":"number"},
            "worldPerPixel":{"type":"number"}
        },
        "required":["kind","eye","target","up","scale","near","far","width","height","worldPerPixel"],
        "additionalProperties":false
    });
    json!({"type":"object","properties":{"camera":camera},"required":["camera"],"additionalProperties":false})
}

fn camera_get(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    read_engine(ctx, args, |e| json!({ "camera": e.camera_state_value() }))
}

fn camera_set(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: CameraSetArgs = parse_args(args)?;
    let text = serde_json::to_string(&a.state).map_err(|e| e.to_string())?;
    ctx.app.docs.engine_mut().apply_camera_state_json(&text)?;
    Ok(Outcome::Done(json!({ "camera": ctx.app.docs.engine().camera_state_value() })))
}

fn zoom_to_fit(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    // The app's own branch, so the command frames what the tile is DRAWING —
    // the open sheet's paper, or the 3D scene — exactly as the toolbar button
    // beside it does.
    ctx.app.zoom_to_fit();
    Ok(Outcome::Done(json!({ "camera": ctx.app.docs.engine().camera_state_value() })))
}

fn standard_view(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ViewArgs = parse_args(args)?;
    if !ctx.app.docs.engine_mut().standard_view(&a.name.to_ascii_uppercase()) {
        return Err(format!("unknown view `{}` (FRONT BACK LEFT RIGHT TOP BOTTOM ISO)", a.name));
    }
    Ok(Outcome::Done(json!({ "camera": ctx.app.docs.engine().camera_state_value() })))
}

fn set_projection(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ProjectionArgs = parse_args(args)?;
    let k = a.kind.to_ascii_lowercase();
    if !(k.starts_with("pers") || k.starts_with("orth")) {
        return Err("kind must be `perspective` or `orthographic`".into());
    }
    ctx.app.docs.engine_mut().set_projection(&k);
    Ok(Outcome::Done(json!({ "camera": ctx.app.docs.engine().camera_state_value() })))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("camera", "camera_get", "The camera state: projection kind/scale, eye, target, up, near/far, viewport width/height and worldPerPixel.", NoArgs, camera_get, Read, READ, camera_result_schema),
    app_command!("camera", "camera_set", "Apply a camera state (the shape `camera_get` returns).", CameraSetArgs, camera_set, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "zoom_to_fit", "Frame the whole model \u{2014} or, while a drawing sheet is open, the sheet's whole paper (the toolbar's Zoom-to-fit button is this same branch).", NoArgs, zoom_to_fit, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "standard_view", "Snap to a standard view and fit: FRONT BACK LEFT RIGHT TOP BOTTOM ISO.", ViewArgs, standard_view, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "set_projection", "Switch between perspective and orthographic projection.", ProjectionArgs, set_projection, Mutate, MUTATE_NOWAIT, camera_result_schema),
];
