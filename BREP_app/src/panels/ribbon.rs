//! Ribbon layout only. All command state and dispatch are shared with Classic.
use super::toolbar_button;
use crate::workbench::{command_groups, CommandSize, CommandTarget, OfferedCommand, RIBBON_TABS};
use eframe::egui;
use std::collections::HashMap;

pub use super::toolbar_button::LARGE_BTN;

/// How a tab presents its commands, chosen from the measured free width.
#[derive(Clone, Copy, Debug, Hash, PartialEq, Eq)]
pub enum SizeMode {
    /// The tab has room for every command Large, so every command is Large.
    ForceLarge,
    /// Each command at its registered size; Compact runs wrap into rows.
    Mixed,
    /// Every command Compact and wrapped.
    AllCompact,
}

#[derive(Debug, PartialEq)]
pub struct GroupLayout {
    pub mode: SizeMode,
    pub visible: usize,
}

impl Default for GroupLayout {
    fn default() -> Self {
        GroupLayout {
            mode: SizeMode::ForceLarge,
            visible: 0,
        }
    }
}

#[derive(Default)]
pub struct RibbonPanel {
    selected: usize,
    layout: GroupLayout,
}

/// Everything Large while the tab has room; otherwise the registered sizes with
/// Compact runs wrapped; then every command Compact; and only then do whole
/// groups move into overflow. The widths are the measured group widths in each
/// mode, in tab order.
pub fn fit_groups(
    force_large: &[f32],
    mixed: &[f32],
    compact: &[f32],
    available: f32,
    overflow_width: f32,
    gap: f32,
) -> GroupLayout {
    let total =
        |widths: &[f32]| widths.iter().sum::<f32>() + gap * widths.len().saturating_sub(1) as f32;
    for (mode, widths) in [
        (SizeMode::ForceLarge, force_large),
        (SizeMode::Mixed, mixed),
        (SizeMode::AllCompact, compact),
    ] {
        if total(widths) <= available {
            return GroupLayout {
                mode,
                visible: widths.len(),
            };
        }
    }
    let mut used = overflow_width;
    let mut visible = 0;
    for width in compact {
        if used + gap + width > available {
            break;
        }
        used += gap + width;
        visible += 1;
    }
    GroupLayout {
        mode: SizeMode::AllCompact,
        visible,
    }
}

/// Rows of Compact commands that stack inside the Large row height: a run of
/// small icons fills these top to bottom before it adds a column.
pub fn compact_rows(ui: &egui::Ui) -> usize {
    let spacing = ui.spacing().item_spacing.y;
    (((LARGE_BTN + spacing) / (toolbar_button::TOOLBAR_BTN + spacing)).floor() as usize).max(1)
}

pub fn hit_key(command: &OfferedCommand) -> String {
    match &command.target {
        CommandTarget::Workbench(id) => format!("workbench:btn:{id}"),
        CommandTarget::Plugin(id) => format!("plugin:action:{id}"),
        CommandTarget::Feature(_) | CommandTarget::Constraint(_) | CommandTarget::Annotation(_) => {
            format!("wbtb:{}", command.id)
        }
        CommandTarget::Shell(id) => (*id).into(),
    }
}

pub fn group_key(name: &str) -> String {
    format!("ribbon:group:{name}")
}

/// Shared button implementation for Classic, Ribbon, explicit menus and overflow.
pub fn draw_command(
    ui: &mut egui::Ui,
    command: &OfferedCommand,
    compact: bool,
    menu: bool,
    hits: &mut HashMap<String, egui::Rect>,
) -> Option<CommandTarget> {
    let enabled = command.disabled_reason.is_none();
    let response = if menu {
        ui.add_enabled(
            enabled,
            toolbar_button::menu_row_button(&command.glyph, command.label())
                .selected(command.pressed)
                .min_size(egui::vec2(
                    ui.available_width(),
                    toolbar_button::TOOLBAR_BTN,
                )),
        )
    } else if compact {
        if command.toggle {
            toolbar_button::toggle_enabled(
                ui,
                enabled,
                command.pressed,
                &command.glyph,
                command.label(),
            )
        } else {
            toolbar_button::button_enabled(ui, enabled, &command.glyph, command.label())
        }
    } else {
        toolbar_button::large_button(
            ui,
            &command.glyph,
            command.label(),
            command.pressed,
            command.toggle,
            enabled,
        )
    };
    let tip = if command.detail.is_empty() || command.detail == command.label() {
        command.label().to_owned()
    } else {
        format!("{}\n{}", command.label(), command.detail)
    };
    let response = response.on_hover_text(&tip).on_disabled_hover_text(format!(
        "{tip}\n{}",
        command.disabled_reason.as_deref().unwrap_or("")
    ));
    if !menu || ui.is_rect_visible(response.rect) {
        hits.insert(hit_key(command), response.rect);
    }
    let mut target = None;
    if !command.menu.is_empty() {
        egui::Popup::menu(&response).show(|ui| {
            for child in &command.menu {
                if let Some(t) = draw_command(ui, child, true, true, hits) {
                    target = Some(t);
                    ui.close();
                }
            }
        });
    } else if response.clicked() {
        target = Some(command.target.clone());
        if menu {
            ui.close();
        }
    }
    target
}

/// A run of Compact commands: columns of `rows` buttons, filled top to bottom.
fn draw_compact_run(
    ui: &mut egui::Ui,
    run: &[&OfferedCommand],
    rows: usize,
    hits: &mut HashMap<String, egui::Rect>,
) -> Option<CommandTarget> {
    let mut target = None;
    for column in run.chunks(rows) {
        ui.vertical(|ui| {
            for c in column {
                if let Some(t) = draw_command(ui, c, true, false, hits) {
                    target = Some(t);
                }
            }
        });
    }
    target
}

/// A bordered group: its commands packed from the left in source order, then
/// its label. Large commands stand alone; Compact runs wrap into rows.
fn draw_group(
    ui: &mut egui::Ui,
    name: &str,
    members: &[&OfferedCommand],
    mode: SizeMode,
    hits: &mut HashMap<String, egui::Rect>,
) -> Option<CommandTarget> {
    let mut target = None;
    let frame = egui::Frame::group(ui.style()).show(ui, |ui| {
        ui.vertical(|ui| {
            ui.with_layout(egui::Layout::left_to_right(egui::Align::TOP), |ui| {
                ui.set_min_height(LARGE_BTN);
                let rows = compact_rows(ui);
                let mut run: Vec<&OfferedCommand> = vec![];
                for c in members {
                    let compact = match mode {
                        SizeMode::ForceLarge => false,
                        SizeMode::Mixed => c.size == CommandSize::Compact,
                        SizeMode::AllCompact => true,
                    };
                    if compact {
                        run.push(c);
                        continue;
                    }
                    if let Some(t) = draw_compact_run(ui, &std::mem::take(&mut run), rows, hits) {
                        target = Some(t);
                    }
                    if let Some(t) = draw_command(ui, c, false, false, hits) {
                        target = Some(t);
                    }
                }
                if let Some(t) = draw_compact_run(ui, &run, rows, hits) {
                    target = Some(t);
                }
            });
            ui.label(egui::RichText::new(name).weak().small());
        });
    });
    hits.insert(group_key(name), frame.response.rect);
    target
}

impl RibbonPanel {
    /// The layout the last `show` chose for the selected tab.
    pub fn layout(&self) -> &GroupLayout {
        &self.layout
    }
    pub fn tabs(&mut self, ui: &mut egui::Ui, hits: &mut HashMap<String, egui::Rect>) {
        for (index, tab) in RIBBON_TABS.iter().enumerate() {
            let response = ui.selectable_label(self.selected == index, *tab);
            hits.insert(format!("ribbon:tab:{tab}"), response.rect);
            if response.clicked() {
                self.selected = index;
            }
        }
    }
    pub fn show(
        &mut self,
        ui: &mut egui::Ui,
        commands: &[OfferedCommand],
        hits: &mut HashMap<String, egui::Rect>,
    ) -> Option<CommandTarget> {
        let groups = command_groups(commands, RIBBON_TABS[self.selected]);
        let measure = |mode: SizeMode| {
            let mut probe = egui::Ui::new(
                ui.ctx().clone(),
                ui.id().with(("ribbon-measure", mode)),
                egui::UiBuilder::new()
                    .sizing_pass()
                    .invisible()
                    .style(ui.style().clone())
                    .layer_id(ui.layer_id())
                    .max_rect(egui::Rect::from_min_size(
                        egui::pos2(1.0e5, 1.0e5),
                        egui::Vec2::splat(1.0e4),
                    )),
            );
            let mut widths = vec![];
            for (name, members) in &groups {
                widths.push(
                    probe
                        .scope(|ui| {
                            draw_group(ui, name, members, mode, &mut HashMap::new());
                        })
                        .response
                        .rect
                        .width(),
                );
            }
            widths
        };
        let force_large = measure(SizeMode::ForceLarge);
        let mixed = measure(SizeMode::Mixed);
        let compact = measure(SizeMode::AllCompact);
        let mut overflow_probe = egui::Ui::new(
            ui.ctx().clone(),
            ui.id().with("overflow-measure"),
            egui::UiBuilder::new()
                .sizing_pass()
                .invisible()
                .style(ui.style().clone())
                .layer_id(ui.layer_id())
                .max_rect(egui::Rect::from_min_size(
                    egui::pos2(1.0e5, 1.0e5),
                    egui::Vec2::splat(1.0e4),
                )),
        );
        let overflow_width = overflow_probe.button("More ▾").rect.width();
        let layout = fit_groups(
            &force_large,
            &mixed,
            &compact,
            ui.available_width(),
            overflow_width,
            ui.spacing().item_spacing.x,
        );
        let mut target = None;
        ui.horizontal(|ui| {
            for (name, members) in groups.iter().take(layout.visible) {
                if let Some(t) = draw_group(ui, name, members, layout.mode, hits) {
                    target = Some(t);
                }
            }
            if layout.visible < groups.len() {
                let more = ui.button("More ▾");
                hits.insert("ribbon:overflow".into(), more.rect);
                egui::Popup::menu(&more).show(|ui| {
                    egui::ScrollArea::vertical()
                        .max_height(ui.ctx().content_rect().height() * 0.75)
                        .show(ui, |ui| {
                            for (name, members) in groups.iter().skip(layout.visible) {
                                let frame = egui::Frame::group(ui.style()).show(ui, |ui| {
                                    ui.weak(*name);
                                    for c in members {
                                        if let Some(t) = draw_command(ui, c, true, true, hits) {
                                            target = Some(t);
                                        }
                                    }
                                });
                                if ui.is_rect_visible(frame.response.rect) {
                                    hits.insert(group_key(name), frame.response.rect);
                                }
                            }
                        });
                });
            }
        });
        self.layout = layout;
        target
    }
}

