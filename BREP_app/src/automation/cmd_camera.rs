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

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields, rename_all = "camelCase")]
pub struct SpaceMouseReportArgs {
    /// The HID report ID: 1 translation (3 axes) or motion (6 axes), 2 rotation, 3 buttons.
    pub report_id: u8,
    /// The report payload WITHOUT the ID byte, as WebHID delivers it: little-endian i16 axes (x right, y toward the user, z down; about ±350 at full deflection), or the button bitmask.
    pub data: Vec<u8>,
}

fn spacemouse_json(parsed: Option<bool>) -> Value {
    let hub = crate::spacemouse::snapshot();
    let mut out = json!({
        "axes": hub.axes,
        "buttons": hub.buttons,
        "device": hub.device,
        "reports": hub.reports,
        "status": crate::spacemouse::status_line(),
    });
    if let Some(parsed) = parsed {
        out["parsed"] = json!(parsed);
    }
    out
}

fn spacemouse_result_schema() -> Value {
    json!({
        "type":"object",
        "properties":{
            "axes":{"type":"array","items":{"type":"integer"},"minItems":6,"maxItems":6,"description":"tx, ty, tz, rx, ry, rz as last reported."},
            "buttons":{"type":"integer","description":"The held-button bitmask."},
            "device":{"type":["string","null"],"description":"The open device's name; null while none is open (injected reports need none)."},
            "reports":{"type":"integer","description":"Reports received since start, from any transport."},
            "status":{"type":"string","description":"The settings window's SpaceMouse status sentence."},
            "parsed":{"type":"boolean","description":"spacemouse_report only: whether the report was understood."}
        },
        "required":["axes","buttons","device","reports","status"],
        "additionalProperties":false
    })
}

/// Inject one raw report through the same entry the WebHID shim and the
/// native reader use; the viewport applies it on its next frames.
fn spacemouse_report(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: SpaceMouseReportArgs = parse_args(args)?;
    let parsed = crate::spacemouse::feed(a.report_id, &a.data);
    ctx.egui.request_repaint();
    Ok(Outcome::Done(spacemouse_json(Some(parsed))))
}

fn spacemouse_state(_ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let _: NoArgs = parse_args(args)?;
    Ok(Outcome::Done(spacemouse_json(None)))
}

pub static COMMANDS: &[CommandSpec] = &[
    app_command!("camera", "camera_get", "The camera state: projection kind/scale, eye, target, up, near/far, viewport width/height and worldPerPixel.", NoArgs, camera_get, Read, READ, camera_result_schema),
    app_command!("camera", "camera_set", "Apply a camera state (the shape `camera_get` returns).", CameraSetArgs, camera_set, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "zoom_to_fit", "Frame the whole model \u{2014} or, while a drawing sheet is open, the sheet's whole paper (the toolbar's Zoom-to-fit button is this same branch).", NoArgs, zoom_to_fit, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "standard_view", "Snap to a standard view and fit: FRONT BACK LEFT RIGHT TOP BOTTOM ISO.", ViewArgs, standard_view, Mutate, MUTATE_NOWAIT, camera_result_schema),
    app_command!("camera", "spacemouse_report", "Inject one SpaceMouse / Spaceball HID input report (report ID + payload), as the device would send it; the 3D view moves on the following frames while the axes are off centre (send an all-zero report to release).", SpaceMouseReportArgs, spacemouse_report, Mutate, INPUT, spacemouse_result_schema),
    app_command!("camera", "spacemouse_state", "The SpaceMouse state: last axes, held buttons, open device and status sentence.", NoArgs, spacemouse_state, Read, READ, spacemouse_result_schema),
    app_command!("camera", "set_projection", "Switch between perspective and orthographic projection.", ProjectionArgs, set_projection, Mutate, MUTATE_NOWAIT, camera_result_schema),
];
