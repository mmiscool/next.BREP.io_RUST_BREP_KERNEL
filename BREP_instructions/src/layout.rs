//! Printed pages from the document's template: a cover, the parts list, the
//! tools list, then the steps at 1, 2 or 4 per page, portrait or landscape.
//!
//! Everything is in paper millimetres from the page's top-left corner. The
//! layout only says WHERE each thing goes; the renderer draws the step's view
//! into a slot's `picture` and its text into `text`.

use crate::model::{Instructions, Orientation};
use serde::Serialize;

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

/// The block's pages, in print order.
pub fn pages(block: &Instructions) -> Result<Vec<Page>, String> {
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
        pages.push(Page { number, width_mm: width, height_mm: height, kind, content, slots });
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
    let steps: Vec<&str> = block.steps().map(|(_, s)| s.id.as_str()).collect();
    for (page_index, chunk) in steps.chunks(cells.len()).enumerate() {
        let slots = chunk
            .iter()
            .zip(&cells)
            .enumerate()
            .map(|(i, (id, cell))| {
                let (picture, text) = split_cell(*cell);
                Slot {
                    step_id: id.to_string(),
                    step_number: page_index * cells.len() + i + 1,
                    rect: *cell,
                    picture,
                    text,
                }
            })
            .collect();
        push(PageKind::Steps, slots, &mut pages);
    }
    Ok(pages)
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
        let text_h = cell.h * 0.3;
        let picture = Rect { x: cell.x, y: cell.y, w: cell.w, h: cell.h - text_h - GUTTER_MM / 2.0 };
        let text = Rect { x: cell.x, y: cell.y + cell.h - text_h, w: cell.w, h: text_h };
        (picture, text)
    }
}

