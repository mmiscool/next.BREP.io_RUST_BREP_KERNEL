//! The Move gizmo's typed-value card: while the component Move gizmo is armed,
//! a small card in the viewport's top-left corner takes exact values — mm
//! along, or degrees about, each of the gizmo's axes. Enter in a field applies
//! that one value ([`EngineState::component_move_typed`]): one solve against the
//! mates, one undo step, the same commit as a drag release. The field then
//! clears, and the card's last line says what the mates let through.
//!
//! It sits in a CORNER, never at the gizmo: the card takes presses, and an
//! interactable overlay at the gizmo would take the press meant for a handle
//! (the rule the readout chip and the constraint chips follow). The top-left
//! is free: toasts take the bottom-left, the selection rail and the ViewCube
//! the right.

use brep_render::engine_state::EngineState;
use eframe::egui;
use std::collections::HashMap;

/// The six fields in card order: `(field id, axis label)`, three moves then
/// three turns.
pub(super) const FIELDS: [(&str, &str); 6] =
    [("x", "X"), ("y", "Y"), ("z", "Z"), ("rx", "X"), ("ry", "Y"), ("rz", "Z")];

/// The card's text and its rects this frame (`movecard/…`).
#[derive(Default)]
pub(crate) struct MoveCard {
    /// What is typed in each field, in [`FIELDS`] order (empty = nothing).
    fields: [String; 6],
    /// `field:<id>` per field and `card` for the whole card, in screen points.
    pub(super) hits: HashMap<String, egui::Rect>,
}

impl MoveCard {
    /// Draw the card for the armed Move gizmo (nothing, and the fields
    /// cleared, while it is disarmed), and apply a field committed with Enter.
    pub(super) fn draw(&mut self, ctx: &egui::Context, rect: egui::Rect, state: &mut EngineState) {
        self.hits.clear();
        if !state.component_move_armed() {
            self.fields = Default::default();
            return;
        }
        let group = state.component_move_group();
        let title = match group.as_slice() {
            [one] => {
                let name = state.component_info(one).map(|info| info.part_name).filter(|name| !name.is_empty());
                format!("Move {}", name.unwrap_or_else(|| one.clone()))
            }
            many => format!("Move {} components", many.len()),
        };
        let axes = if group.len() > 1 { "World axes, about the group's centre" } else { "Along the gizmo's axes, about its centre" };
        let dragging = state.component_move_dragging();
        let report = state.component_move_typed_report();
        let mut commit: Option<(usize, String)> = None;
        let hits = &mut self.hits;
        let fields = &mut self.fields;
        let shown = egui::Area::new(egui::Id::new("brep-move-card"))
            .order(egui::Order::Foreground)
            .pivot(egui::Align2::LEFT_TOP)
            .fixed_pos(rect.left_top() + egui::vec2(8.0, 8.0))
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.strong(title);
                    ui.weak(axes);
                    egui::Grid::new("brep-move-card-grid").num_columns(5).spacing([6.0, 4.0]).show(ui, |ui| {
                        for (row, (label, unit)) in [("Move", "mm"), ("Turn", "\u{00b0}")].into_iter().enumerate() {
                            ui.label(label);
                            for column in 0..3 {
                                let index = row * 3 + column;
                                let (id, axis) = FIELDS[index];
                                let field = ui.add_enabled(
                                    !dragging,
                                    egui::TextEdit::singleline(&mut fields[index]).hint_text(axis).desired_width(56.0),
                                );
                                if field.lost_focus() && ui.input(|input| input.key_pressed(egui::Key::Enter)) {
                                    commit = Some((index, fields[index].clone()));
                                }
                                hits.insert(format!("field:{id}"), field.rect);
                            }
                            ui.label(unit);
                            ui.end_row();
                        }
                    });
                    if let Some(report) = &report {
                        let color = if !report.written {
                            ui.visuals().error_fg_color
                        } else if report.complete() {
                            ui.visuals().text_color()
                        } else {
                            ui.visuals().warn_fg_color
                        };
                        ui.label(egui::RichText::new(report.summary()).color(color));
                    }
                });
            });
        self.hits.insert("card".into(), shown.response.rect);
        if let Some((index, text)) = commit {
            self.fields[index].clear();
            apply(state, FIELDS[index].0, &text);
        }
    }
}

/// Apply one committed field: blank or zero does nothing, a number moves.
fn apply(state: &mut EngineState, field: &str, text: &str) {
    let text = text.trim();
    if text.is_empty() {
        return;
    }
    let Ok(value) = text.replace(',', ".").parse::<f64>() else {
        state.push_notice(format!("Move: {text:?} is not a number."));
        return;
    };
    if value == 0.0 {
        return;
    }
    if let Err(error) = state.component_move_typed(field, value) {
        state.push_notice(format!("Move: {error}."));
    }
}

