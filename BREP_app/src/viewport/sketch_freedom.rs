//! The sketch HOVER READOUT: how many ways the hovered point or curve can still
//! move ("2 DOF free", "Fully constrained"), painted beside the pointer while the
//! canvas draws that entity's free directions as arrows. Painted, never
//! hit-tested, so it cannot take a press from the geometry under it.

use eframe::egui;

use brep_render::engine_state::EngineState;

use super::Viewport;

impl Viewport {
    pub(super) fn draw_sketch_hover_freedom(ctx: &egui::Context, rect: egui::Rect, state: &EngineState) {
        if !state.sketch_mode() {
            return;
        }
        let Some(label) = state.sketch_hover_freedom_label() else { return };
        let Some(pointer) = ctx.pointer_hover_pos().filter(|p| rect.contains(*p)) else { return };
        let mut painter =
            ctx.layer_painter(egui::LayerId::new(egui::Order::Foreground, egui::Id::new("brep-sketch-hover-freedom")));
        painter.set_clip_rect(rect);
        let font = egui::FontId::proportional(12.0);
        let text = ctx.global_style().visuals.text_color();
        let galley = painter.layout_no_wrap(label, font, text);
        let at = pointer + egui::vec2(16.0, 14.0);
        let bg = egui::Rect::from_min_size(at, galley.size()).expand(4.0);
        painter.rect_filled(bg, 4.0, ctx.global_style().visuals.extreme_bg_color.gamma_multiply(0.9));
        painter.galley(at, galley, text);
    }
}
