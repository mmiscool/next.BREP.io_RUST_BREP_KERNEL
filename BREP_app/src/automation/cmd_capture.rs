//! Screen capture through egui's own screenshot path: `ViewportCommand::Screenshot`
//! in the mutate phase, completed by the matching `Event::Screenshot` in a later
//! frame's input phase (§5). The headless host renders directly instead and
//! answers this command itself; the window and dial-in hosts go through here.
use crate::automation::command::{parse_args, schema_of, Annotations, CommandSpec, Ctx, Empty, Handler, Outcome, Phase};
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

/// What to capture: the whole surface, the 3D viewport, or a rect in points.
#[derive(Debug, Clone, Deserialize, schemars::JsonSchema)]
#[serde(untagged)]
pub enum Region {
    /// `"full"` or `"viewport"`
    Named(String),
    Rect { x: f32, y: f32, w: f32, h: f32 },
}

impl Default for Region {
    fn default() -> Self {
        Region::Named("full".into())
    }
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ScreenshotArgs {
    #[serde(default)]
    pub region: Region,
}

#[derive(Serialize, schemars::JsonSchema)]
pub struct ScreenshotInfo {
    /// Pixel size of the PNG carried beside this reply.
    pub width: u32,
    pub height: u32,
    pub ppp: f32,
    /// The captured rect in egui points.
    pub region: [f32; 4],
    pub png_bytes: usize,
}

/// Resolve a region to a pixel rect on a `w`×`h` image at `ppp`.
pub fn region_px(region: &Region, ppp: f32, view: Option<egui::Rect>, w: u32, h: u32) -> Result<(u32, u32, u32, u32, [f32; 4]), String> {
    let pts = match region {
        Region::Named(n) if n == "full" => [0.0, 0.0, w as f32 / ppp, h as f32 / ppp],
        Region::Named(n) if n == "viewport" => {
            let r = view.ok_or("no viewport rect: the 3D view was not drawn last frame")?;
            [r.min.x, r.min.y, r.width(), r.height()]
        }
        Region::Named(n) => return Err(format!("unknown region `{n}` (full | viewport | {{x,y,w,h}})")),
        Region::Rect { x, y, w, h } => [*x, *y, *w, *h],
    };
    let px = |v: f32| (v * ppp).round().max(0.0) as u32;
    let x0 = px(pts[0]).min(w.saturating_sub(1));
    let y0 = px(pts[1]).min(h.saturating_sub(1));
    let cw = px(pts[2]).clamp(1, w - x0);
    let ch = px(pts[3]).clamp(1, h - y0);
    Ok((x0, y0, cw, ch, pts))
}

/// Encode a captured frame (cropped to `region`) as PNG.
pub fn encode_capture(image: &egui::ColorImage, ppp: f32, view: Option<egui::Rect>, region: &Region) -> Result<(Value, Vec<u8>), String> {
    let (w, h) = (image.width() as u32, image.height() as u32);
    let (x0, y0, cw, ch, pts) = region_px(region, ppp, view, w, h)?;
    let full = image::RgbaImage::from_raw(w, h, image.as_raw().to_vec()).ok_or("frame buffer size mismatch")?;
    let cropped = image::imageops::crop_imm(&full, x0, y0, cw, ch).to_image();
    let mut out = std::io::Cursor::new(Vec::new());
    cropped.write_to(&mut out, image::ImageFormat::Png).map_err(|e| format!("png encode: {e}"))?;
    let png = out.into_inner();
    let info = ScreenshotInfo { width: cw, height: ch, ppp, region: pts, png_bytes: png.len() };
    Ok((serde_json::to_value(info).map_err(|e| e.to_string())?, png))
}

fn screenshot(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: ScreenshotArgs = parse_args(args)?;
    let token = ctx.app.automation.next_token();
    ctx.egui.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::new(token)));
    Ok(Outcome::AwaitScreenshot { token, region: a.region })
}

#[derive(Deserialize, schemars::JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IllustrationArgs {
    /// begin | view | end. Normally orchestrated by illustration_capture_many.
    pub action: String,
    pub document_id: Option<u64>,
    pub view: Option<Value>,
}

fn illustration_presentation(ctx: &mut Ctx<'_>, args: Value) -> Result<Outcome, String> {
    let a: IllustrationArgs = parse_args(args)?;
    if let Some(id) = a.document_id {
        let index = ctx.app.docs.iter().position(|d| d.id() == id).ok_or("illustration document is no longer open")?;
        ctx.app.docs.activate(index);
    }
    let id = ctx.app.docs.active_id();
    let engine = ctx.app.docs.engine_mut();
    match a.action.as_str() {
        "begin" => { engine.illustration_begin()?; Ok(Outcome::Done(json!({"document_id":id}))) },
        "view" => engine.illustration_apply(&a.view.unwrap_or(json!({}))).map(Outcome::Done),
        "end" => { engine.illustration_end()?; Ok(Outcome::Done(json!({"restored":true,"document_id":id}))) },
        _ => Err("action must be begin, view or end".into()),
    }
}

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec { name: "illustration_presentation", group: "capture", doc: "Internal presentation loan for illustration_capture_many: preserves camera, full selection, display settings, saved-view state and display transforms without editing history or creating documents.", phase: Phase::Mutate, annotations: Annotations::MUTATE_NOWAIT, args_schema: schema_of::<IllustrationArgs>, result_schema: schema_of::<Empty>, handler: Handler::App(illustration_presentation) },
    CommandSpec { name: "screenshot", group: "capture", doc: "Capture the composited frame (egui chrome and the 3D view) as PNG: region `full` (default), `viewport`, or `{x,y,w,h}` in egui points. The PNG rides beside the JSON reply.", phase: Phase::Mutate, annotations: Annotations::READ, args_schema: schema_of::<ScreenshotArgs>, result_schema: schema_of::<ScreenshotInfo>, handler: Handler::App(screenshot) },
];

#[allow(dead_code)]
fn _unused(_: Value) -> Value {
    json!({})
}
