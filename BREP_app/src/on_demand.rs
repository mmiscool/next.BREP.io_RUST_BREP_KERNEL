//! One egui pass the app runs itself, when nothing is drawing it.
//!
//! A desktop window on a hidden Wayland workspace gets no `RedrawRequested`
//! (`desktop_loop`), and a browser tab in the background gets no animation
//! frames (`web_pump`). An automation command that needs a pass (input, a
//! capture, a current layout) gets one from here: the app's own
//! `raw_input_hook`, `logic` and `ui`, in `Context::run_ui`, never presented.
//! The caller builds the pass's input (its host knows the surface) and
//! finishes its captures (the desktop waits for the GPU; the page cannot).
use crate::app::BrepApp;
use eframe::egui_wgpu;

/// What a pass leaves its caller.
pub struct Pass {
    /// `ViewportCommand::Screenshot`s the pass asked for: the caller draws
    /// them ([`crate::offscreen`]) and returns `Event::Screenshot`s in a
    /// later pass's input, as eframe delivers one.
    pub captures: Vec<egui::UserData>,
    pub shapes: Vec<egui::epaint::ClippedShape>,
    pub pixels_per_point: f32,
    /// Applied ([`crate::offscreen::apply_texture_sets`]); its frees are the
    /// caller's, once anything that draws with them is done.
    pub textures_delta: egui::TexturesDelta,
}

/// Run one pass of `app` with `raw`. Its texture delta goes to the shared
/// renderer (the window's next real frame never sees it); viewport and
/// platform commands go back onto the context for eframe's next real frame,
/// except the root viewport's captures, which the caller draws.
pub fn run(app: &mut BrepApp, ctx: &egui::Context, render: Option<&egui_wgpu::RenderState>, mut raw: egui::RawInput) -> Pass {
    // This pass draws its `ui` (offscreen), whatever the window last said
    // (occluded, minimised, a hidden tab): automation input is applied here.
    let root = raw.viewports.entry(egui::ViewportId::ROOT).or_default();
    root.occluded = Some(false);
    root.minimized = Some(false);
    eframe::App::raw_input_hook(app, ctx, &mut raw);
    let mut frame = eframe::Frame::_new_kittest();
    let output = ctx.run_ui(raw, |ui| {
        eframe::App::logic(app, ui.ctx(), &mut frame);
        eframe::App::ui(app, ui, &mut frame);
    });
    let egui::FullOutput { platform_output, textures_delta, shapes, pixels_per_point, mut viewport_output } = output;
    if let Some(render) = render {
        crate::offscreen::apply_texture_sets(render, &textures_delta);
    }
    let mut captures = Vec::new();
    for (id, viewport) in viewport_output.iter_mut() {
        for command in std::mem::take(&mut viewport.commands) {
            match command {
                egui::ViewportCommand::Screenshot(user_data) if *id == egui::ViewportId::ROOT => captures.push(user_data),
                other => ctx.send_viewport_cmd_to(*id, other),
            }
        }
    }
    // What the user would have been given (a copy, an opened link) still
    // happens, on eframe's next real frame.
    if !platform_output.commands.is_empty() {
        ctx.output_mut(|o| o.commands.extend(platform_output.commands));
    }
    // A surface that IS drawn shows the pass's effect on its next frame.
    ctx.request_repaint();
    Pass { captures, shapes, pixels_per_point, textures_delta }
}

/// The pixel size and clear colour a capture of this pass is drawn at.
pub fn capture_target(app: &BrepApp, ctx: &egui::Context, pixels_per_point: f32) -> ([u32; 2], [f32; 4]) {
    let size = ctx.content_rect().size() * pixels_per_point;
    let clear = eframe::App::clear_color(app, &ctx.global_style().visuals);
    ([size.x.round() as u32, size.y.round() as u32], clear)
}

/// The `Event::Screenshot`s that deliver `image` for `captures`.
pub fn screenshot_events(captures: Vec<egui::UserData>, image: egui::ColorImage) -> Vec<egui::Event> {
    let image = std::sync::Arc::new(image);
    captures
        .into_iter()
        .map(|user_data| egui::Event::Screenshot { viewport_id: egui::ViewportId::ROOT, user_data, image: image.clone() })
        .collect()
}
