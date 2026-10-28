//! Manufacturing instructions on paper: the printed pages and the per-step
//! pictures, drawn as sheets.
//!
//! [`brep_instructions::layout`] says WHERE everything goes on each page; this
//! module fills those places with what the sheet writers already draw —
//! polylines and centred text runs in paper millimetres, plus each step's view
//! projected as a placed view ([`ViewDrawing`]) — so the PDF and the SVG are
//! [`super::pdf::write_pdf`] and [`super::svg::to_svg`] unchanged. The
//! projections come from the engine, which owns the scene: it asks
//! [`picture_rects`] where each picture goes, fits a view into each with
//! [`fit`], and hands the drawings back to [`pages`] or [`step_image`].

use super::frame::FrameDrawing;
use super::pdf::text_width;
use super::project::{SheetDrawing, SheetText, ViewDrawing};
use super::svg::CAP_PER_EM;
use brep_instructions::layout::{Page, PageKind, Rect, HEADER_MM, MARGIN_MM, PARTS_ROW_MM, TOOLS_ROW_MM};
use brep_instructions::model::{Instructions, Step};
use std::collections::HashMap;

/// Body text's cap height, in millimetres.
const BODY_MM: f64 = 2.8;
/// A heading's cap height.
const HEADING_MM: f64 = 4.2;
/// Line pitch over cap height.
const LEADING: f64 = 1.75;
/// The share of a picture rectangle a fitted view fills.
const FILL: f64 = 0.9;
/// The cover's title block above its picture.
const COVER_TEXT_MM: f64 = 46.0;
/// A per-step picture: its paper and the caption band above the view.
pub const IMAGE_MM: (f64, f64) = (160.0, 120.0);
const IMAGE_CAPTION_MM: f64 = 10.0;

/// A run's width in millimetres at cap height `cap`.
fn width_mm(text: &str, cap: f64) -> f64 {
    text_width(text) * cap / CAP_PER_EM
}

/// A run reading left to right whose line box starts at `(x, y)` (top-left).
/// The writers centre a run on its anchor, so the anchor is the box's middle.
fn left(text: impl Into<String>, x: f64, y: f64, cap: f64) -> SheetText {
    let text = text.into();
    let w = width_mm(&text, cap);
    SheetText::flat(text, [x + w / 2.0, y + cap / 2.0], cap)
}

/// [`left`], ending at `x`.
fn right(text: impl Into<String>, x: f64, y: f64, cap: f64) -> SheetText {
    let text = text.into();
    let w = width_mm(&text, cap);
    SheetText::flat(text, [x - w / 2.0, y + cap / 2.0], cap)
}

/// `text` broken into lines no wider than `width` at cap height `cap`, at
/// spaces; a word wider than the line stands alone on one.
pub fn wrap(text: &str, width: f64, cap: f64) -> Vec<String> {
    let mut lines = Vec::new();
    for paragraph in text.lines() {
        let mut line = String::new();
        for word in paragraph.split_whitespace() {
            let candidate = if line.is_empty() { word.to_string() } else { format!("{line} {word}") };
            if !line.is_empty() && width_mm(&candidate, cap) > width {
                lines.push(std::mem::replace(&mut line, word.to_string()));
            } else {
                line = candidate;
            }
        }
        lines.push(line);
    }
    lines
}

fn rect_lines(r: Rect) -> Vec<[f64; 2]> {
    vec![[r.x, r.y], [r.x + r.w, r.y], [r.x + r.w, r.y + r.h], [r.x, r.y + r.h], [r.x, r.y]]
}

/// A column of text lines from the top of `area` down; a line that would
/// run past its bottom is left out and the last one kept reads `…`.
struct Column {
    area: Rect,
    y: f64,
    texts: Vec<SheetText>,
    full: bool,
}

impl Column {
    fn new(area: Rect) -> Self {
        Column { area, y: area.y, texts: Vec::new(), full: false }
    }

    fn line(&mut self, text: impl Into<String>, cap: f64, indent: f64) {
        if self.full {
            return;
        }
        let pitch = cap * LEADING;
        if self.y + pitch > self.area.y + self.area.h + 1e-9 {
            self.full = true;
            if let Some(last) = self.texts.pop() {
                let x = last.anchor[0] - width_mm(&last.text, last.height) / 2.0;
                let y = last.anchor[1] - last.height / 2.0;
                self.texts.push(left(format!("{}\u{2026}", last.text), x, y, last.height));
            }
            return;
        }
        self.texts.push(left(text, self.area.x + indent, self.y, cap));
        self.y += pitch;
    }

    fn wrapped(&mut self, text: &str, cap: f64, indent: f64) {
        for line in wrap(text, self.area.w - indent, cap) {
            self.line(line, cap, indent);
        }
    }

    fn gap(&mut self) {
        self.y += BODY_MM * 0.6;
    }
}

/// What a step's text area says: its number and title, the parts it adds,
/// its tools and torque, its callouts and its notes.
fn step_text(step: &Step, number: usize, area: Rect) -> Vec<SheetText> {
    let mut column = Column::new(area);
    column.line(format!("Step {number}"), HEADING_MM, 0.0);
    if !step.title.is_empty() {
        column.wrapped(&step.title, BODY_MM * 1.15, 0.0);
    }
    column.gap();
    if !step.adds.is_empty() {
        column.line("Parts", BODY_MM, 0.0);
        for add in &step.adds {
            let find = if add.find_number.is_empty() { String::new() } else { format!("({}) ", add.find_number) };
            column.wrapped(&format!("{find}{} \u{d7} {}", add.quantity(), add.display_name()), BODY_MM, 3.0);
        }
        column.gap();
    }
    if !step.tools.is_empty() {
        column.line("Tools", BODY_MM, 0.0);
        for tool in &step.tools {
            let text = if tool.spec.is_empty() { tool.name.clone() } else { format!("{} \u{2014} {}", tool.name, tool.spec) };
            column.wrapped(&text, BODY_MM, 3.0);
        }
        column.gap();
    }
    if let Some(torque) = &step.torque {
        column.wrapped(&format!("Torque: {}", torque.text()), BODY_MM, 0.0);
        column.gap();
    }
    for callout in &step.callouts {
        column.wrapped(&format!("{}: {}", callout.kind.label().to_uppercase(), callout.text), BODY_MM, 0.0);
    }
    if !step.callouts.is_empty() {
        column.gap();
    }
    if !step.notes.is_empty() {
        column.wrapped(&step.notes, BODY_MM, 0.0);
    }
    column.texts
}

/// The document's title as the pages print it.
pub fn title(block: &Instructions) -> String {
    let title = block.front.title.trim();
    if !title.is_empty() {
        return title.to_string();
    }
    let name = block.target.name.trim();
    if name.is_empty() { "Assembly instructions".into() } else { format!("{name} assembly instructions") }
}

/// The step a page's cover shows: the front matter's, else the last.
pub fn cover_step(block: &Instructions) -> Option<String> {
    block.front.cover_step.clone().or_else(|| block.steps().last().map(|(_, step)| step.id.clone()))
}

/// Where a picture goes, keyed by what it shows: `cover` for the cover's,
/// else `(page index, slot index)` for a step slot's.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum PictureAt {
    Cover,
    Slot(usize, usize),
}

/// Every picture the pages draw, with the step whose view it is and the
/// rectangle it is fitted into.
pub fn picture_rects(block: &Instructions, pages: &[Page]) -> Vec<(PictureAt, String, Rect)> {
    let mut out = Vec::new();
    for (index, page) in pages.iter().enumerate() {
        match page.kind {
            PageKind::Cover => {
                if let Some(step) = cover_step(block) {
                    let c = page.content;
                    out.push((PictureAt::Cover, step, Rect { x: c.x, y: c.y + COVER_TEXT_MM, w: c.w, h: c.h - COVER_TEXT_MM }));
                }
            }
            PageKind::Steps => {
                for (slot_index, slot) in page.slots.iter().enumerate() {
                    out.push((PictureAt::Slot(index, slot_index), slot.step_id.clone(), slot.picture));
                }
            }
            _ => {}
        }
    }
    out
}

/// The placement `(position, scale)` that fits a view into `rect`, from its
/// drawing at position `[0, 0]` and scale 1 (`bounds` = that drawing's
/// `[x0, y0, x1, y1]`). Text keeps its paper size, so this is the fit of the
/// lines; the margin `FILL` leaves takes up the balloons' slack.
pub fn fit(bounds: [f64; 4], rect: Rect) -> ([f64; 2], f64) {
    let (w, h) = (bounds[2] - bounds[0], bounds[3] - bounds[1]);
    let scale = if w > 1e-9 && h > 1e-9 {
        (rect.w / w).min(rect.h / h) * FILL
    } else if w > 1e-9 {
        rect.w / w * FILL
    } else if h > 1e-9 {
        rect.h / h * FILL
    } else {
        1.0
    };
    let centre = [(bounds[0] + bounds[2]) / 2.0, (bounds[1] + bounds[3]) / 2.0];
    ([rect.x + rect.w / 2.0 - centre[0] * scale, rect.y + rect.h / 2.0 - centre[1] * scale], scale)
}

/// The printed pages as sheets, in print order: each page's header (the
/// title, and the page number of the count), then its own content — the
/// cover's title, assembly and picture; a parts or tools list's rows; the
/// step slots, each framed, its picture in `pictures` and its text beside or
/// below. A picture missing from `pictures` leaves its place empty.
pub fn pages(block: &Instructions, layout: &[Page], pictures: &HashMap<PictureAt, ViewDrawing>) -> Vec<SheetDrawing> {
    let title = title(block);
    let count = layout.len();
    let parts = block.parts_list();
    let tools = block.tools_list();
    layout
        .iter()
        .enumerate()
        .map(|(index, page)| {
            let mut lines: Vec<Vec<[f64; 2]>> = Vec::new();
            let mut texts: Vec<SheetText> = Vec::new();
            let mut views: Vec<ViewDrawing> = Vec::new();
            let c = page.content;
            // The header band: title left, page number right, a rule below.
            let header_y = MARGIN_MM + (HEADER_MM - BODY_MM * 1.2) / 2.0 - 1.0;
            let section = match page.kind {
                PageKind::Cover => String::new(),
                PageKind::PartsList { .. } => " \u{2014} Parts list".into(),
                PageKind::ToolsList { .. } => " \u{2014} Tools".into(),
                PageKind::Steps => String::new(),
            };
            texts.push(left(format!("{title}{section}"), c.x, header_y, BODY_MM * 1.2));
            texts.push(right(format!("Page {} of {count}", page.number), c.x + c.w, header_y, BODY_MM * 1.2));
            lines.push(vec![[c.x, c.y - 2.0], [c.x + c.w, c.y - 2.0]]);
            match page.kind {
                PageKind::Cover => {
                    let mut column = Column::new(Rect { h: COVER_TEXT_MM, ..c });
                    for line in wrap(&title, c.w, 8.0) {
                        column.line(line, 8.0, 0.0);
                    }
                    if !block.front.subtitle.is_empty() {
                        column.wrapped(&block.front.subtitle, 4.5, 0.0);
                    }
                    column.gap();
                    let target = &block.target;
                    let mut assembly = format!("Assembly: {}", if target.name.is_empty() { "-" } else { &target.name });
                    if !target.part_number.is_empty() {
                        assembly.push_str(&format!(" {}", target.part_number));
                    }
                    if !target.revision.is_empty() {
                        assembly.push_str(&format!(" rev {}", target.revision));
                    }
                    column.wrapped(&assembly, BODY_MM * 1.2, 0.0);
                    column.line(format!("{} steps", block.step_count()), BODY_MM * 1.2, 0.0);
                    texts.extend(column.texts);
                    if let Some(view) = pictures.get(&PictureAt::Cover) {
                        views.push(view.clone());
                    }
                }
                PageKind::PartsList { first, count } => {
                    let columns = [c.x, c.x + 24.0, c.x + c.w - 48.0, c.x + c.w - 30.0];
                    let row = |r: usize| c.y + r as f64 * PARTS_ROW_MM + (PARTS_ROW_MM - BODY_MM) / 2.0;
                    for (x, label) in columns.iter().zip(["Find No.", "Part", "Qty", "First step"]) {
                        texts.push(left(label, *x, row(0), BODY_MM));
                    }
                    for (r, item) in parts.iter().skip(first).take(count).enumerate() {
                        let step = block.step_number(&item.first_step).map(|n| n.to_string()).unwrap_or_default();
                        let name = if item.subassembly { format!("{} (sub-assembly)", item.name) } else { item.name.clone() };
                        let cells = [item.find_number.clone(), name, item.quantity.to_string(), step];
                        for (x, cell) in columns.iter().zip(cells) {
                            if !cell.is_empty() {
                                texts.push(left(cell, *x, row(r + 1), BODY_MM));
                            }
                        }
                    }
                    for r in 1..=count + 1 {
                        let y = c.y + r as f64 * PARTS_ROW_MM;
                        lines.push(vec![[c.x, y], [c.x + c.w, y]]);
                    }
                }
                PageKind::ToolsList { first, count } => {
                    let columns = [c.x, c.x + c.w * 0.4, c.x + c.w - 30.0];
                    let row = |r: usize| c.y + r as f64 * TOOLS_ROW_MM + (TOOLS_ROW_MM - BODY_MM) / 2.0;
                    for (x, label) in columns.iter().zip(["Tool", "Spec", "Steps"]) {
                        texts.push(left(label, *x, row(0), BODY_MM));
                    }
                    for (r, (tool, steps)) in tools.iter().skip(first).take(count).enumerate() {
                        let numbers: Vec<String> =
                            steps.iter().filter_map(|id| block.step_number(id)).map(|n| n.to_string()).collect();
                        let cells = [tool.name.clone(), tool.spec.clone(), numbers.join(", ")];
                        for (x, cell) in columns.iter().zip(cells) {
                            if !cell.is_empty() {
                                texts.push(left(cell, *x, row(r + 1), BODY_MM));
                            }
                        }
                    }
                    for r in 1..=count + 1 {
                        let y = c.y + r as f64 * TOOLS_ROW_MM;
                        lines.push(vec![[c.x, y], [c.x + c.w, y]]);
                    }
                }
                PageKind::Steps => {
                    for (slot_index, slot) in page.slots.iter().enumerate() {
                        lines.push(rect_lines(slot.rect));
                        if let Some(view) = pictures.get(&PictureAt::Slot(index, slot_index)) {
                            views.push(view.clone());
                        }
                        if let Some(step) = block.step(&slot.step_id) {
                            let inset = Rect { x: slot.text.x + 3.0, y: slot.text.y + 3.0, w: slot.text.w - 6.0, h: slot.text.h - 6.0 };
                            texts.extend(step_text(step, slot.step_number, inset));
                        }
                    }
                }
            }
            SheetDrawing {
                id: format!("P{}", page.number),
                name: format!("Page {}", page.number),
                width_mm: page.width_mm,
                height_mm: page.height_mm,
                views,
                dimensions: Vec::new(),
                ordinates: Vec::new(),
                frame: Some(FrameDrawing { lines, texts, revision_table: None }),
            }
        })
        .collect()
}

/// Where a per-step picture's view goes on its [`IMAGE_MM`] paper.
pub fn image_picture_rect() -> Rect {
    let m = 4.0;
    Rect { x: m, y: IMAGE_CAPTION_MM, w: IMAGE_MM.0 - 2.0 * m, h: IMAGE_MM.1 - IMAGE_CAPTION_MM - m }
}

/// One step's picture as a sheet: its number and title over its view.
pub fn step_image(block: &Instructions, step_id: &str, picture: Option<ViewDrawing>) -> Result<SheetDrawing, String> {
    let step = block.step(step_id).ok_or_else(|| format!("no step '{step_id}'"))?;
    let number = block.step_number(step_id).unwrap_or(0);
    let caption = if step.title.is_empty() { format!("Step {number}") } else { format!("Step {number}  {}", step.title) };
    let texts = vec![left(caption, 4.0, (IMAGE_CAPTION_MM - HEADING_MM) / 2.0, HEADING_MM)];
    Ok(SheetDrawing {
        id: step.id.clone(),
        name: format!("Step {number}"),
        width_mm: IMAGE_MM.0,
        height_mm: IMAGE_MM.1,
        views: picture.into_iter().collect(),
        dimensions: Vec::new(),
        ordinates: Vec::new(),
        frame: Some(FrameDrawing { lines: Vec::new(), texts, revision_table: None }),
    })
}

