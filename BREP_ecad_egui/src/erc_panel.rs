//! The Connectivity panel's electrical rule check, and what it draws on the sheet:
//! the no-connect crosses and power flags its fixes place, and a ring on each
//! finding.
//!
//! The check runs on the first press of its button, as KiCad's does. From then on it
//! follows every edit, so a fix or a new wire takes its row away at once. A click on
//! a row zooms the sheet to the finding and rings it there. The row's selection does
//! not select the part: selecting it would put the part's properties above this list
//! and move the row away from the pointer.
use crate::{Editor, accent};
use brep_ecad_core::Point;
use brep_ecad_core::erc::{ErcFinding, ErcFix, ErcKind, ErcSeverity};
use egui::{Color32, Rect, Stroke, Vec2};

/// Error and warning on the canvas's own dark ground.
const ERROR: Color32 = Color32::from_rgb(240, 90, 90);
const WARNING: Color32 = Color32::from_rgb(232, 182, 64);
/// KiCad draws the no-connect cross in blue.
const NO_CONNECT: Color32 = Color32::from_rgb(110, 140, 255);
/// The most rows drawn; the count above them still says how many there are.
const MAX_ROWS: usize = 200;

#[derive(Default)]
pub(crate) struct ErcView {
    /// Set once the check has been run on this sheet.
    run: bool,
    /// The finding last clicked, by kind and place, so it stays picked when a fix
    /// above it in the list takes a row away.
    focused: Option<(ErcKind, Point)>,
    /// The section's buttons and rows as last drawn, for automation.
    pub(crate) hits: crate::symbol_editor::InspectorHits,
}

/// A row: the severity as a chip in its colour, then the message. A chip and not
/// coloured text, so the word still reads on a selected row's own highlight (the
/// eCAD re-audit found DRC's red word on the blue selection hard to read).
fn finding_row(style: &egui::Style, f: &ErcFinding) -> egui::text::LayoutJob {
    let body = egui::TextStyle::Body.resolve(style);
    let (word, color) = match f.severity {
        ErcSeverity::Error => (" Error ", style.visuals.error_fg_color),
        ErcSeverity::Warning => (" Warning ", style.visuals.warn_fg_color),
    };
    let mut row = egui::text::LayoutJob::default();
    row.append(
        word,
        0.,
        egui::TextFormat {
            font_id: body.clone(),
            color: style.visuals.panel_fill,
            background: color,
            ..Default::default()
        },
    );
    row.append(&f.message, 8., egui::TextFormat::simple(body, style.visuals.text_color()));
    row
}
/// What a fix's button says when hovered.
fn fix_hint(fix: &ErcFix) -> &'static str {
    match fix {
        ErcFix::MarkNoConnect(_) => "Leaving it open is meant: put a no-connect cross on the pin, as KiCad does, and the check stops asking for it.",
        ErcFix::RemoveNoConnect(_) => "The pin is joined after all: take its no-connect cross off.",
        ErcFix::AddPowerFlag(_) => "The supply comes from off the board, through a connector or a bench supply: a power flag on the pin says so, as KiCad's PWR_FLAG does.",
    }
}

impl Editor {
    /// The check's section of the Connectivity panel: its button, the count, one row
    /// per finding and the fix a finding offers under it.
    pub(crate) fn erc_section(&mut self, ui: &mut egui::Ui) {
        self.erc.hits.begin(ui.ctx());
        ui.add_space(8.);
        ui.label(egui::RichText::new("Electrical rules").strong());
        // Once run, the findings follow every edit, so the button's second press
        // is not "again" but "done": it takes the rows and the rings away.
        let button = if self.erc.run {
            ui.button("Hide the findings")
                .on_hover_text("Take the findings and their rings off the sheet. They follow every edit while shown.")
        } else {
            ui.button("Run electrical rules check")
                .on_hover_text("Check how the pins are joined: pins left open, outputs joined to outputs, power inputs nothing drives, no-connect pins that are joined, unused units, repeated references and wires that end on nothing.")
        };
        self.erc.hits.mark("erc:run", &button);
        if button.clicked() {
            self.erc.run = !self.erc.run;
            self.erc.focused = None;
        }
        if !self.erc.run {
            return;
        }
        let findings = self.document.erc();
        if findings.is_empty() {
            ui.label(egui::RichText::new("No findings.").color(accent(ui)));
            return;
        }
        let errors = findings.iter().filter(|f| f.severity == ErcSeverity::Error).count();
        let warnings = findings.len() - errors;
        let plural = |n: usize, one: &str| match n {
            1 => format!("1 {one}"),
            n => format!("{n} {one}s"),
        };
        ui.label(format!(
            "{}, {}. Click one to zoom to it on the sheet.",
            plural(errors, "error"),
            plural(warnings, "warning")
        ));
        let mut focus = None;
        let mut fix = None;
        egui::ScrollArea::vertical()
            .id_salt("erc")
            .max_height(260.)
            .auto_shrink([false, true])
            .show(ui, |ui| {
                ui.style_mut().wrap_mode = Some(egui::TextWrapMode::Wrap);
                for (i, f) in findings.iter().enumerate().take(MAX_ROWS) {
                    let focused = self.erc.focused == Some((f.kind, f.at));
                    let row = ui
                        .selectable_label(focused, finding_row(ui.style(), f))
                        .on_hover_text("Zoom to it on the sheet");
                    self.erc.hits.mark(format!("erc:row:{i}"), &row);
                    if row.clicked() {
                        focus = Some(i);
                    }
                    if let Some(offered) = &f.fix {
                        ui.horizontal(|ui| {
                            ui.add_space(16.);
                            let button = ui.small_button(offered.label()).on_hover_text(fix_hint(offered));
                            self.erc.hits.mark(format!("erc:fix:{i}"), &button);
                            if button.clicked() {
                                fix = Some(offered.clone());
                            }
                        });
                    }
                }
            });
        if findings.len() > MAX_ROWS {
            ui.small(format!("The first {MAX_ROWS} are listed."));
        }
        if let Some(i) = focus {
            let f = &findings[i];
            self.erc.focused = Some((f.kind, f.at));
            self.focus_sheet(f.at);
        }
        if let Some(fix) = fix {
            self.transaction(|d| d.apply_erc_fix(&fix));
        }
    }
    /// Centre the sheet on `at`, zoomed in far enough to read a pin number there.
    fn focus_sheet(&mut self, at: Point) {
        self.zoom = self.zoom.max(0.024);
        self.pan = -Vec2::new(at.x as f32, at.y as f32) * self.zoom;
    }
    /// The markers the check's fixes set, and, once the check has run, a ring on each
    /// finding: a heavier one on the finding last clicked.
    pub(crate) fn paint_erc(&self, painter: &egui::Painter, rect: Rect) {
        let size = (self.zoom * 800.).clamp(3., 9.);
        for t in &self.document.no_connects {
            if let Some(at) = self.document.terminal_position(t) {
                let c = self.screen(at, rect);
                let stroke = Stroke::new(2., NO_CONNECT);
                painter.line_segment([c + Vec2::new(-size, -size), c + Vec2::new(size, size)], stroke);
                painter.line_segment([c + Vec2::new(-size, size), c + Vec2::new(size, -size)], stroke);
            }
        }
        for t in &self.document.power_flags {
            if let Some(at) = self.document.terminal_position(t) {
                // KiCad's PWR_FLAG: a stem from the pin and a diamond on it. Down,
                // since a pin's own net flag runs sideways from its tip and the
                // part's reference sits above its body.
                let c = self.screen(at, rect);
                let top = c + Vec2::new(0., 2.5 * size);
                let stroke = Stroke::new(1.5, ERROR);
                painter.line_segment([c, top], stroke);
                let d = size * 0.8;
                painter.add(egui::Shape::closed_line(
                    vec![
                        top,
                        top + Vec2::new(-d, d),
                        top + Vec2::new(0., 2. * d),
                        top + Vec2::new(d, d),
                    ],
                    stroke,
                ));
                if self.zoom > 0.006 {
                    painter.text(
                        top + Vec2::new(d + 3., d),
                        egui::Align2::LEFT_CENTER,
                        "PWR_FLAG",
                        egui::FontId::monospace(10.),
                        ERROR,
                    );
                }
            }
        }
        if !self.erc.run {
            return;
        }
        for f in self.document.erc() {
            let color = match f.severity {
                ErcSeverity::Error => ERROR,
                ErcSeverity::Warning => WARNING,
            };
            let c = self.screen(f.at, rect);
            if self.erc.focused == Some((f.kind, f.at)) {
                painter.circle_stroke(c, size * 2.4, Stroke::new(3., color));
            } else {
                painter.circle_stroke(c, size * 1.4, Stroke::new(1.5, color));
            }
        }
    }
    /// What `ecad_state` reports of the check: whether it has run, and its findings
    /// (the same list whether or not it has).
    pub fn erc_state(&self) -> (bool, Vec<ErcFinding>) {
        (self.erc.run, self.document.erc())
    }
}

