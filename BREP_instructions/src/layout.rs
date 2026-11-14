//! Printed pages from the document's template: a cover, the parts list, the
//! tools list, then the steps at 1, 2 or 4 per page, portrait or landscape.
//!
//! Everything is in paper millimetres from the page's top-left corner. The
//! layout only says WHERE each thing goes; the renderer draws the step's view
//! into a slot's `picture` and its text into `text`.
//!
//! A step's Record block ([`crate::record`]: its fields as a paper form, a
//! serial line per serial-tracked unit, the sign-off lines) goes under the
//! step's text. Its height is known here, measured through a
//! [`TextMeasure`], so the paging accounts for it: a step with a block may
//! take more of a tall cell from its picture (down to [`PICTURE_MIN_SHARE`]
//! of the cell and [`PICTURE_MIN_MM`]),
//! and what still does not fit continues on Record pages right after the
//! steps page, never over the picture and never cut off. A document whose
//! steps record nothing and need no sign-off lays out exactly as before.

use crate::model::{Instructions, Orientation, Step};
use crate::record::{self, Item};
use serde::Serialize;

/// Body text's cap height, in millimetres.
pub const BODY_MM: f64 = 2.8;
/// A heading's cap height.
pub const HEADING_MM: f64 = 4.2;
/// Line pitch over cap height.
pub const LEADING: f64 = 1.75;
/// The inset of a step's text inside its text area.
pub const TEXT_INSET_MM: f64 = 3.0;
/// The share of a tall cell the text area takes.
pub const TEXT_SHARE: f64 = 0.3;
/// What the picture of a step with a Record block keeps when the text area
/// grows into it: this share of the cell's height, and never less than
/// [`PICTURE_MIN_MM`] (a picture smaller than that says little).
pub const PICTURE_MIN_SHARE: f64 = 0.3;
pub const PICTURE_MIN_MM: f64 = 60.0;
/// The widest a Record page's column is: a line longer than this is hard
/// to write along.
pub const RECORD_COLUMN_MM: f64 = 180.0;

/// How wide a run of text is on paper: the renderer's font metrics, so the
/// layout and the drawing agree.
pub trait TextMeasure {
    /// The width in millimetres of `text` at cap height `cap`.
    fn width_mm(&self, text: &str, cap: f64) -> f64;
}

/// A measure for a caller without font metrics (a page count on a server, a
/// test): every character half an em wide, the em 1.39 cap heights. Close
/// to a sans-serif's average; a renderer passes its own.
pub struct Estimate;

impl TextMeasure for Estimate {
    fn width_mm(&self, text: &str, cap: f64) -> f64 {
        text.chars().count() as f64 * 0.5 * cap * 1.39
    }
}

/// `text` broken into lines no wider than `width` at cap height `cap`, at
/// spaces; a word wider than the line stands alone on one.
pub fn wrap(text: &str, width: f64, cap: f64, m: &dyn TextMeasure) -> Vec<String> {
    let mut lines = Vec::new();
    for paragraph in text.lines() {
        let mut line = String::new();
        for word in paragraph.split_whitespace() {
            let candidate = if line.is_empty() { word.to_string() } else { format!("{line} {word}") };
            if !line.is_empty() && m.width_mm(&candidate, cap) > width {
                lines.push(std::mem::replace(&mut line, word.to_string()));
            } else {
                line = candidate;
            }
        }
        lines.push(line);
    }
    lines
}

/// One line of a step's text column.
#[derive(Debug, Clone, PartialEq)]
pub enum TextLine {
    /// A run at cap height `cap`, `indent` millimetres in.
    Text { text: String, cap: f64, indent: f64 },
    /// A little space between two groups of lines.
    Gap,
}

impl TextLine {
    pub fn height(&self) -> f64 {
        match self {
            TextLine::Text { cap, .. } => cap * LEADING,
            TextLine::Gap => BODY_MM * 0.6,
        }
    }
}

/// What a step's text area says, top to bottom, wrapped to `width`: its
/// number and title, the parts it adds, its tools and torque, its callouts
/// and its notes.
pub fn step_text(step: &Step, number: usize, width: f64, m: &dyn TextMeasure) -> Vec<TextLine> {
    let mut out = Vec::new();
    let wrapped = |out: &mut Vec<TextLine>, text: &str, cap: f64, indent: f64| {
        out.extend(wrap(text, width - indent, cap, m).into_iter().map(|text| TextLine::Text { text, cap, indent }));
    };
    out.push(TextLine::Text { text: format!("Step {number}"), cap: HEADING_MM, indent: 0.0 });
    if !step.title.is_empty() {
        wrapped(&mut out, &step.title, BODY_MM * 1.15, 0.0);
    }
    out.push(TextLine::Gap);
    if !step.adds.is_empty() {
        out.push(TextLine::Text { text: "Parts".into(), cap: BODY_MM, indent: 0.0 });
        for add in &step.adds {
            let find = if add.find_number.is_empty() { String::new() } else { format!("({}) ", add.find_number) };
            wrapped(&mut out, &format!("{find}{} \u{d7} {}", add.quantity(), add.display_name()), BODY_MM, 3.0);
        }
        out.push(TextLine::Gap);
    }
    if !step.tools.is_empty() {
        out.push(TextLine::Text { text: "Tools".into(), cap: BODY_MM, indent: 0.0 });
        for tool in &step.tools {
            let text = if tool.spec.is_empty() { tool.name.clone() } else { format!("{} \u{2014} {}", tool.name, tool.spec) };
            wrapped(&mut out, &text, BODY_MM, 3.0);
        }
        out.push(TextLine::Gap);
    }
    if let Some(torque) = &step.torque {
        wrapped(&mut out, &format!("Torque: {}", torque.text()), BODY_MM, 0.0);
        out.push(TextLine::Gap);
    }
    for callout in &step.callouts {
        wrapped(&mut out, &format!("{}: {}", callout.kind.label().to_uppercase(), callout.text), BODY_MM, 0.0);
    }
    if !step.callouts.is_empty() {
        out.push(TextLine::Gap);
    }
    if !step.notes.is_empty() {
        wrapped(&mut out, &step.notes, BODY_MM, 0.0);
    }
    out
}

/// The height of [`step_text`]: trailing space not counted.
pub fn step_text_height(step: &Step, number: usize, width: f64, m: &dyn TextMeasure) -> f64 {
    let lines = step_text(step, number, width, m);
    let end = lines.iter().rposition(|line| matches!(line, TextLine::Text { .. })).map_or(0, |i| i + 1);
    lines[..end].iter().map(TextLine::height).sum()
}

/// Paper sizes in millimetres, landscape (the same keys and sizes as drawing
/// sheets; portrait swaps them).
pub const PAPER_SIZES: &[(&str, f64, f64)] = &[
    ("A4", 297.0, 210.0),
    ("A3", 420.0, 297.0),
    ("A2", 594.0, 420.0),
    ("A1", 841.0, 594.0),
    ("A0", 1189.0, 841.0),
    ("Letter", 279.4, 215.9),
    ("Tabloid", 431.8, 279.4),
];

pub const MARGIN_MM: f64 = 12.0;
/// The band at the top of every page: document title and page number.
pub const HEADER_MM: f64 = 12.0;
pub const GUTTER_MM: f64 = 6.0;
pub const PARTS_ROW_MM: f64 = 12.0;
pub const TOOLS_ROW_MM: f64 = 16.0;

#[derive(Debug, Clone, Copy, PartialEq, Serialize)]
pub struct Rect {
    pub x: f64,
    pub y: f64,
    pub w: f64,
    pub h: f64,
}

impl Rect {
    pub fn contains(&self, other: &Rect) -> bool {
        const E: f64 = 1e-9;
        other.x >= self.x - E && other.y >= self.y - E && other.x + other.w <= self.x + self.w + E && other.y + other.h <= self.y + self.h + E
    }

    pub fn overlaps(&self, other: &Rect) -> bool {
        const E: f64 = 1e-9;
        self.x + E < other.x + other.w && other.x + E < self.x + self.w && self.y + E < other.y + other.h && other.y + E < self.y + self.h
    }
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum PageKind {
    Cover,
    /// Rows `first .. first + count` of the parts list.
    PartsList { first: usize, count: usize },
    /// Rows `first .. first + count` of the tools list.
    ToolsList { first: usize, count: usize },
    Steps,
    /// Record blocks carried over from the steps page before it: the rows
    /// are the page's `records`.
    Record,
}

/// One band of a step's Record block, placed: the item it belongs to, which
/// of the item's bands it is, and where it goes. The renderer draws
/// [`record::bands`] of the item at `rect.w` and takes band `band`.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RecordRow {
    pub step_id: String,
    pub step_number: usize,
    #[serde(flatten)]
    pub item: Item,
    pub band: usize,
    /// The heading of a block carried over to a Record page (it names its
    /// step).
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub continued: bool,
    pub rect: Rect,
}

/// One step's place on a page.
#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Slot {
    pub step_id: String,
    pub step_number: usize,
    pub rect: Rect,
    /// Where the step's view is drawn.
    pub picture: Rect,
    /// Where the step's number, parts, tools, torque, callouts and notes go.
    pub text: Rect,
    /// The bands of the step's Record block that fit under its text, placed
    /// inside `text`; the rest are on the Record page(s) that follow.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub record: Vec<RecordRow>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Page {
    /// 1-based.
    pub number: usize,
    pub width_mm: f64,
    pub height_mm: f64,
    #[serde(flatten)]
    pub kind: PageKind,
    /// The area below the header and inside the margins.
    pub content: Rect,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub slots: Vec<Slot>,
    /// A Record page's rows.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub records: Vec<RecordRow>,
}

/// A paper key's millimetres in `orientation`.
pub fn paper_mm(paper: &str, orientation: Orientation) -> Result<(f64, f64), String> {
    let &(_, w, h) = PAPER_SIZES
        .iter()
        .find(|(key, _, _)| key.eq_ignore_ascii_case(paper))
        .ok_or_else(|| format!("unknown paper size '{paper}'"))?;
    Ok(match orientation {
        Orientation::Landscape => (w, h),
        Orientation::Portrait => (h, w),
    })
}

/// The block's pages, in print order, measured with [`Estimate`]: for a
/// caller without font metrics. A renderer calls [`pages_with`] with its own
/// measure, so its pages are the ones it draws.
pub fn pages(block: &Instructions) -> Result<Vec<Page>, String> {
    pages_with(block, &Estimate)
}

/// The block's pages, in print order, text measured by `m`.
pub fn pages_with(block: &Instructions, m: &dyn TextMeasure) -> Result<Vec<Page>, String> {
    let template = &block.template;
    if !crate::model::STEPS_PER_PAGE.contains(&template.steps_per_page) {
        return Err(format!("steps per page must be 1, 2 or 4, not {}", template.steps_per_page));
    }
    let (width, height) = paper_mm(&template.paper, template.orientation)?;
    let content = Rect {
        x: MARGIN_MM,
        y: MARGIN_MM + HEADER_MM,
        w: width - 2.0 * MARGIN_MM,
        h: height - 2.0 * MARGIN_MM - HEADER_MM,
    };
    let mut pages = Vec::new();
    let push = |kind: PageKind, slots: Vec<Slot>, pages: &mut Vec<Page>| {
        let number = pages.len() + 1;
        pages.push(Page { number, width_mm: width, height_mm: height, kind, content, slots, records: Vec::new() });
    };
    push(PageKind::Cover, Vec::new(), &mut pages);
    if block.front.parts_list {
        let rows = block.parts_list().len();
        for (first, count) in chunks(rows, rows_per_page(content.h, PARTS_ROW_MM)) {
            push(PageKind::PartsList { first, count }, Vec::new(), &mut pages);
        }
    }
    if block.front.tools_list {
        let rows = block.tools_list().len();
        if rows > 0 {
            for (first, count) in chunks(rows, rows_per_page(content.h, TOOLS_ROW_MM)) {
                push(PageKind::ToolsList { first, count }, Vec::new(), &mut pages);
            }
        }
    }
    let cells = grid(content, template.steps_per_page, template.orientation);
    let steps: Vec<&Step> = block.steps().map(|(_, s)| s).collect();
    for (page_index, chunk) in steps.chunks(cells.len()).enumerate() {
        // What did not fit under a step's text: the step, its number and the
        // items left, in order.
        let mut carried: Vec<(&Step, usize, Vec<Item>)> = Vec::new();
        let slots = chunk
            .iter()
            .zip(&cells)
            .enumerate()
            .map(|(i, (step, cell))| {
                let step_number = page_index * cells.len() + i + 1;
                let items = record::items(step);
                let (picture, text) = if items.is_empty() { split_cell(*cell) } else { split_cell_for(*cell, step, step_number, &items, m) };
                let (rows, left) = place_record(step, step_number, &items, text, m);
                if !left.is_empty() {
                    carried.push((step, step_number, left));
                }
                Slot { step_id: step.id.clone(), step_number, rect: *cell, picture, text, record: rows }
            })
            .collect();
        push(PageKind::Steps, slots, &mut pages);
        for records in record_pages(&carried, content, m) {
            push(PageKind::Record, Vec::new(), &mut pages);
            pages.last_mut().expect("just pushed").records = records;
        }
    }
    Ok(pages)
}

/// A cell's picture and text areas for a step with a Record block: in a
/// tall cell (text below the picture) the text area grows from its usual
/// share to what the step's text and block need, while the picture keeps
/// [`PICTURE_MIN_SHARE`] of the cell and at least [`PICTURE_MIN_MM`]. A wide
/// cell's text column already runs the cell's height.
fn split_cell_for(cell: Rect, step: &Step, number: usize, items: &[Item], m: &dyn TextMeasure) -> (Rect, Rect) {
    if cell.w > 1.3 * cell.h {
        return split_cell(cell);
    }
    let width = cell.w - 2.0 * TEXT_INSET_MM;
    let block: f64 = items.iter().flat_map(|item| record::bands(step, number, item, width, false, m)).map(|band| band.h).sum();
    let needed = step_text_height(step, number, width, m) + RECORD_GAP_MM + block + 2.0 * TEXT_INSET_MM;
    let usual = cell.h * TEXT_SHARE;
    let most = (cell.h - GUTTER_MM / 2.0 - (cell.h * PICTURE_MIN_SHARE).max(PICTURE_MIN_MM)).max(usual);
    let text_h = needed.clamp(usual, most);
    let picture = Rect { x: cell.x, y: cell.y, w: cell.w, h: cell.h - text_h - GUTTER_MM / 2.0 };
    let text = Rect { x: cell.x, y: cell.y + cell.h - text_h, w: cell.w, h: text_h };
    (picture, text)
}

/// The space between a step's text and its Record block.
const RECORD_GAP_MM: f64 = 2.0;

/// Place the step's Record block under its text inside the text area: whole
/// items, in order, while they fit (the heading only with the item after
/// it). Returns the placed rows and the items left for a Record page.
fn place_record(step: &Step, number: usize, items: &[Item], text: Rect, m: &dyn TextMeasure) -> (Vec<RecordRow>, Vec<Item>) {
    if items.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let area = Rect { x: text.x + TEXT_INSET_MM, y: text.y + TEXT_INSET_MM, w: text.w - 2.0 * TEXT_INSET_MM, h: text.h - 2.0 * TEXT_INSET_MM };
    let bottom = area.y + area.h + 1e-9;
    let mut y = area.y + step_text_height(step, number, area.w, m) + RECORD_GAP_MM;
    let heights = |item: &Item| -> Vec<f64> { record::bands(step, number, item, area.w, false, m).iter().map(|band| band.h).collect() };
    let mut rows = Vec::new();
    for (i, item) in items.iter().enumerate() {
        let own = heights(item);
        let mut needed: f64 = own.iter().sum();
        if *item == Item::Heading {
            needed += items.get(i + 1).map_or(0.0, |next| heights(next).iter().sum());
        }
        if y + needed > bottom {
            return (rows, items[i..].to_vec());
        }
        for (band, h) in own.into_iter().enumerate() {
            rows.push(RecordRow { step_id: step.id.clone(), step_number: number, item: *item, band, continued: false, rect: Rect { x: area.x, y, w: area.w, h } });
            y += h;
        }
    }
    (rows, Vec::new())
}

/// The Record pages after a steps page: each carried step's heading (naming
/// the step) and the bands of its remaining items, flowing down one column
/// and onto as many pages as they need. A heading stays with the band after
/// it; a band taller than a page still gets a page of its own.
fn record_pages(carried: &[(&Step, usize, Vec<Item>)], content: Rect, m: &dyn TextMeasure) -> Vec<Vec<RecordRow>> {
    let width = content.w.min(RECORD_COLUMN_MM);
    let bottom = content.y + content.h + 1e-9;
    let mut pages: Vec<Vec<RecordRow>> = Vec::new();
    let mut y = content.y;
    for (step, number, items) in carried {
        // The bands in order, the block's own heading replaced by the
        // carried one.
        let mut bands: Vec<(Item, usize, bool, f64)> = Vec::new();
        for (band, b) in record::bands(step, *number, &Item::Heading, width, true, m).iter().enumerate() {
            bands.push((Item::Heading, band, true, b.h));
        }
        for item in items.iter().filter(|item| **item != Item::Heading) {
            for (band, b) in record::bands(step, *number, item, width, false, m).iter().enumerate() {
                bands.push((*item, band, false, b.h));
            }
        }
        if !pages.is_empty() {
            y += 4.0;
        }
        for (i, (item, band, continued, h)) in bands.iter().enumerate() {
            let with_next = if *continued { bands.get(i + 1).map_or(0.0, |next| next.3) } else { 0.0 };
            if pages.is_empty() || (y + h + with_next > bottom && y > content.y + 1e-9) {
                pages.push(Vec::new());
                y = content.y;
            }
            pages.last_mut().expect("a page was just opened").push(RecordRow {
                step_id: step.id.clone(),
                step_number: *number,
                item: *item,
                band: *band,
                continued: *continued,
                rect: Rect { x: content.x, y, w: width, h: *h },
            });
            y += h;
        }
    }
    pages
}

/// Rows a list page holds: the content height in rows, less one for the
/// column headings, at least one.
fn rows_per_page(content_h: f64, row_mm: f64) -> usize {
    ((content_h / row_mm).floor() as usize).saturating_sub(1).max(1)
}

/// `(first, count)` chunks of `rows`; one empty chunk for no rows, so the
/// list still prints its headings.
fn chunks(rows: usize, per_page: usize) -> Vec<(usize, usize)> {
    if rows == 0 {
        return vec![(0, 0)];
    }
    (0..rows).step_by(per_page).map(|first| (first, per_page.min(rows - first))).collect()
}

/// The step cells of a page: 1 fills it; 2 stack (portrait) or sit side by
/// side (landscape); 4 make a 2 × 2 grid. Reading order: left to right, top
/// to bottom.
fn grid(content: Rect, per_page: u8, orientation: Orientation) -> Vec<Rect> {
    let (columns, rows) = match (per_page, orientation) {
        (1, _) => (1, 1),
        (2, Orientation::Portrait) => (1, 2),
        (2, Orientation::Landscape) => (2, 1),
        _ => (2, 2),
    };
    let w = (content.w - GUTTER_MM * (columns - 1) as f64) / columns as f64;
    let h = (content.h - GUTTER_MM * (rows - 1) as f64) / rows as f64;
    let mut cells = Vec::new();
    for r in 0..rows {
        for c in 0..columns {
            cells.push(Rect { x: content.x + c as f64 * (w + GUTTER_MM), y: content.y + r as f64 * (h + GUTTER_MM), w, h });
        }
    }
    cells
}

/// A cell's picture and text areas: text on the right of a wide cell, below
/// a tall one.
fn split_cell(cell: Rect) -> (Rect, Rect) {
    if cell.w > 1.3 * cell.h {
        let text_w = (cell.w * 0.34).max(40.0f64.min(cell.w * 0.5));
        let picture = Rect { x: cell.x, y: cell.y, w: cell.w - text_w - GUTTER_MM / 2.0, h: cell.h };
        let text = Rect { x: cell.x + cell.w - text_w, y: cell.y, w: text_w, h: cell.h };
        (picture, text)
    } else {
        let text_h = cell.h * TEXT_SHARE;
        let picture = Rect { x: cell.x, y: cell.y, w: cell.w, h: cell.h - text_h - GUTTER_MM / 2.0 };
        let text = Rect { x: cell.x, y: cell.y + cell.h - text_h, w: cell.w, h: text_h };
        (picture, text)
    }
}

