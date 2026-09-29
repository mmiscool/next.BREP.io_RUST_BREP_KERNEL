//! The paper's own furniture: the drawing BORDER and the TITLE BLOCK.
//!
//! Both are the sheet's ([`crate::sheets::Sheet::border`],
//! `border_inset_mm`, `title_block`, `notes`), both are on by default, and both
//! come out of here as the same thing a placed view does — polylines and
//! centred text runs in paper millimetres, measured from the paper's TOP-LEFT
//! corner with y running down. That is what lets the SVG writer, the PDF writer
//! and the sheet viewport draw the frame with the code they already have for
//! the drawing: **lines and text only**, nothing filled.
//!
//! # The title block
//!
//! A fixed stamp in the bottom-right corner, inside the border: five cells, each
//! a small LABEL over its VALUE.
//!
//! ```text
//! +----------------------------------+
//! | SHEET                            |
//! | Page 1                           |
//! +----------------------------------+
//! | DOCUMENT                         |
//! | bracket                          |
//! +----------------+-----------------+
//! | SCALE          | DATE            |
//! | 4:1            | 2026-09-13      |
//! +----------------+-----------------+
//! | NOTES                            |
//! | first article                    |
//! +----------------------------------+
//! ```
//!
//! Fixed, because a title block that scaled with the paper would be unreadable
//! on an A0 and off the page on a small custom sheet; what DOES scale is the
//! whole stamp on a sheet too small to carry it ([`SCALE_FLOOR`]), so a 60 × 40
//! mm sheet gets a proportionally smaller one rather than a block hanging over
//! its own border.
//!
//! `SCALE` is the FIRST placed view's — a sheet whose views are at different
//! scales says so per view, and the stamp names the one the drawing is read at.
//! `DATE` is the day the sheet was drawn or exported, UTC (see [`today_iso`]);
//! it is part of the projection cache's key, so a session open across midnight
//! re-projects once rather than printing yesterday.
//!
//! # The revision table
//!
//! The sheet's [`crate::sheets::Revision`] rows, as a table of three columns
//! beside the title block — immediately to its LEFT, at the stamp's own scale,
//! its bottom on the same margin — with a header row on top and the revisions
//! under it in the order the list keeps them:
//!
//! ```text
//! +-----+----------+----------------------------+----------------------------------+
//! | REV |   DATE   |        DESCRIPTION         | SHEET                            |
//! +-----+----------+----------------------------+ Page 1                           |
//! |  A  |2026-09-01|       first release        +----------------------------------+
//! +-----+----------+----------------------------+ …                                |
//! |  B  |2026-09-14|       bore opened out      |                                  |
//! +-----+----------+----------------------------+----------------------------------+
//! ```
//!
//! Its width is the stamp's, 110 mm, so every preset carries the two side by
//! side at full size. A sheet whose drawing area is too narrow for both puts
//! the table ON TOP of the stamp instead, right-aligned with it; a sheet with
//! the title block switched off puts it in the corner the stamp would have
//! taken. One too short for the table in any of those places draws none, and
//! an empty list never draws one. The text is the title block's: flat runs of
//! the same cap heights, centred in their cells.

use super::project::SheetText;
use super::Sheet;

/// The title block's size at full scale, millimetres.
const TITLE_WIDTH_MM: f64 = 110.0;
/// Its rows, top to bottom: sheet, document, scale|date, notes.
const ROW_HEIGHTS_MM: [f64; 4] = [11.0, 8.0, 8.0, 8.0];
/// Where the `SCALE | DATE` row divides, as a fraction of the width.
const SPLIT: f64 = 0.5;
/// The cap height of a cell's LABEL and of its VALUE, millimetres.
const LABEL_HEIGHT_MM: f64 = 1.8;
const VALUE_HEIGHT_MM: f64 = 3.2;
/// The first row's value is the sheet's name, and reads as the title.
const TITLE_HEIGHT_MM: f64 = 4.4;
/// How much of a cell the label strip takes.
const LABEL_BAND: f64 = 0.42;
/// A cell's text is inset this far from its own edges, millimetres.
const CELL_PAD_MM: f64 = 1.2;
/// Below this scale factor the stamp is not drawn at all: a sheet that cannot
/// carry a legible title block is better off with none.
pub const SCALE_FLOOR: f64 = 0.35;
/// The revision table's three columns at full scale, millimetres: REV, DATE
/// and DESCRIPTION. They sum to [`TITLE_WIDTH_MM`], so the table stacks flush
/// on the stamp when it has to go above it.
const REVISION_COLUMNS_MM: [f64; 3] = [12.0, 24.0, 74.0];
/// Its header row and each revision row, at full scale.
const REVISION_HEADER_MM: f64 = 6.0;
const REVISION_ROW_MM: f64 = 7.0;
/// A revision cell's cap height — a size between the title block's label and
/// its value, because a row is shorter than a stamp cell.
const REVISION_TEXT_MM: f64 = 2.6;

/// The whole title-block height at full scale.
fn title_height_mm() -> f64 {
    ROW_HEIGHTS_MM.iter().sum()
}

/// What the frame knows that the sheet does not: the document it belongs to and
/// the day it is drawn on.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct FrameContext {
    /// The open document's name, as the app's tab and every other export mean
    /// it; empty when the document has never been named.
    pub document: String,
    /// `YYYY-MM-DD`, from [`today_iso`].
    pub date: String,
}

/// The border, title block and revision table, drawn — polylines and centred
/// text runs in paper millimetres. Absent from a sheet that draws none of them.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct FrameDrawing {
    /// The border rectangle and the title block's cell rules, each a polyline.
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
    /// The revision table, when the sheet has revisions and room for them. A
    /// piece of its own rather than more `lines` and `texts`, so the SVG can
    /// group it and a reader can find its bounds.
    #[serde(rename = "revisionTable", skip_serializing_if = "Option::is_none")]
    pub revision_table: Option<RevisionTableDrawing>,
}

/// The revision table: its rules, its header and row text, and where it sits.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct RevisionTableDrawing {
    pub lines: Vec<Vec<[f64; 2]>>,
    /// The header's three labels, then each row's cells, in row order. A blank
    /// cell writes no run, as a blank title-block value does.
    pub texts: Vec<SheetText>,
    /// How many revision rows it draws, header not counted.
    pub rows: usize,
    /// `[x0, y0, x1, y1]`, paper millimetres.
    pub bounds: [f64; 4],
    /// Where it went: `beside` the title block, `above` it, or in the `corner`
    /// of a sheet with no title block.
    pub placement: String,
}

impl FrameDrawing {
    fn is_empty(&self) -> bool {
        self.lines.is_empty() && self.texts.is_empty() && self.revision_table.is_none()
    }
}

/// Build the frame of `sheet`. `None` when the sheet draws none of its pieces.
pub fn frame(sheet: &Sheet, context: &FrameContext) -> Option<FrameDrawing> {
    let (w, h) = sheet.millimetres();
    let inset = sheet.border_inset();
    let mut out = FrameDrawing { lines: Vec::new(), texts: Vec::new(), revision_table: None };
    if sheet.border {
        // A closed rectangle as ONE polyline, back to its first corner: the
        // frame is lines like everything else on the paper, not a `<rect>` the
        // PDF writer would have to special-case.
        let (x0, y0, x1, y1) = (inset, inset, w - inset, h - inset);
        if x1 > x0 && y1 > y0 {
            out.lines.push(vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]);
        }
    }
    if sheet.title_block {
        title_block(sheet, context, w, h, inset, &mut out);
    }
    out.revision_table = revision_table(sheet, w, h, inset);
    (!out.is_empty()).then_some(out)
}

/// The stamp's scale factor on a `w × h` sheet at `inset`: 1 wherever it fits,
/// less on a sheet too small to carry it. The revision table is drawn at the
/// SAME factor, so the two read as one piece of furniture.
fn stamp_scale(w: f64, h: f64, inset: f64) -> f64 {
    let usable_w = (w - 2.0 * inset).max(0.0);
    let usable_h = (h - 2.0 * inset).max(0.0);
    (usable_w / TITLE_WIDTH_MM).min(usable_h / title_height_mm()).min(1.0)
}

/// The revision table, placed. `None` for an empty list, or a sheet with no
/// room for it — see the module header for where it goes.
fn revision_table(sheet: &Sheet, w: f64, h: f64, inset: f64) -> Option<RevisionTableDrawing> {
    if sheet.revisions.is_empty() {
        return None;
    }
    let scale = stamp_scale(w, h, inset);
    if scale < SCALE_FLOOR {
        return None;
    }
    let table_w = TITLE_WIDTH_MM * scale;
    let table_h = (REVISION_HEADER_MM + REVISION_ROW_MM * sheet.revisions.len() as f64) * scale;
    let (left, top) = (inset, inset);
    let (right, bottom) = (w - inset, h - inset);
    // Where it can go, in order: to the stamp's left on the bottom margin; on
    // top of the stamp; or, with no stamp at all, in the corner.
    let (x1, y1, placement) = if sheet.title_block {
        let stamp_left = right - TITLE_WIDTH_MM * scale;
        let stamp_top = bottom - title_height_mm() * scale;
        if stamp_left - table_w >= left - 1e-9 && bottom - table_h >= top - 1e-9 {
            (stamp_left, bottom, "beside")
        } else if stamp_top - table_h >= top - 1e-9 {
            (right, stamp_top, "above")
        } else {
            return None;
        }
    } else if bottom - table_h >= top - 1e-9 {
        (right, bottom, "corner")
    } else {
        return None;
    };
    let (x0, y0) = (x1 - table_w, y1 - table_h);

    let mut lines = vec![vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]];
    // A rule under the header and one between each pair of rows…
    let header = REVISION_HEADER_MM * scale;
    let row = REVISION_ROW_MM * scale;
    for k in 0..sheet.revisions.len() {
        let y = y0 + header + row * k as f64;
        lines.push(vec![[x0, y], [x1, y]]);
    }
    // …and the two column rules, the table's whole height.
    let mut columns = vec![x0];
    for width in REVISION_COLUMNS_MM {
        columns.push(columns[columns.len() - 1] + width * scale);
    }
    for x in &columns[1..3] {
        lines.push(vec![[*x, y0], [*x, y1]]);
    }

    let pad = CELL_PAD_MM * scale;
    let mut table = FrameDrawing { lines: Vec::new(), texts: Vec::new(), revision_table: None };
    for (k, label) in ["REV", "DATE", "DESCRIPTION"].into_iter().enumerate() {
        push_text(&mut table, label, columns[k] + pad, columns[k + 1] - pad, y0, y0 + header, LABEL_HEIGHT_MM * scale);
    }
    for (index, revision) in sheet.revisions.iter().enumerate() {
        let (top, bottom) = (y0 + header + row * index as f64, y0 + header + row * (index + 1) as f64);
        for (k, value) in [&revision.rev, &revision.date, &revision.description].into_iter().enumerate() {
            push_text(&mut table, value, columns[k] + pad, columns[k + 1] - pad, top, bottom, REVISION_TEXT_MM * scale);
        }
    }
    Some(RevisionTableDrawing {
        lines,
        texts: table.texts,
        rows: sheet.revisions.len(),
        bounds: [x0, y0, x1, y1],
        placement: placement.to_string(),
    })
}

/// The stamp in the bottom-right corner, inside the border.
fn title_block(
    sheet: &Sheet,
    context: &FrameContext,
    w: f64,
    h: f64,
    inset: f64,
    out: &mut FrameDrawing,
) {
    let scale = stamp_scale(w, h, inset);
    if scale < SCALE_FLOOR {
        return;
    }
    let block_w = TITLE_WIDTH_MM * scale;
    let rows: Vec<f64> = ROW_HEIGHTS_MM.iter().map(|mm| mm * scale).collect();
    let block_h: f64 = rows.iter().sum();
    // Bottom-right, against the border.
    let x0 = w - inset - block_w;
    let y0 = h - inset - block_h;
    let x1 = x0 + block_w;
    let y1 = y0 + block_h;

    out.lines.push(vec![[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]]);
    let mut y = y0;
    let mut edges = Vec::new();
    for row in &rows {
        edges.push(y);
        y += row;
        if y < y1 - 1e-9 {
            out.lines.push(vec![[x0, y], [x1, y]]);
        }
    }
    edges.push(y1);
    // The one vertical rule: SCALE | DATE.
    let split = x0 + block_w * SPLIT;
    out.lines.push(vec![[split, edges[2]], [split, edges[3]]]);

    let pad = CELL_PAD_MM * scale;
    let mut cell = |x_min: f64, x_max: f64, y_min: f64, y_max: f64, label: &str, value: &str, value_height: f64| {
        let band = (y_max - y_min) * LABEL_BAND;
        push_text(out, label, x_min + pad, x_max - pad, y_min, y_min + band, LABEL_HEIGHT_MM * scale);
        push_text(out, value, x_min + pad, x_max - pad, y_min + band, y_max, value_height * scale);
    };
    cell(x0, x1, edges[0], edges[1], "SHEET", &sheet.name, TITLE_HEIGHT_MM);
    cell(x0, x1, edges[1], edges[2], "DOCUMENT", &context.document, VALUE_HEIGHT_MM);
    cell(x0, split, edges[2], edges[3], "SCALE", &scale_text(sheet), VALUE_HEIGHT_MM);
    cell(split, x1, edges[2], edges[3], "DATE", &context.date, VALUE_HEIGHT_MM);
    cell(x0, x1, edges[3], edges[4], "NOTES", &sheet.notes, VALUE_HEIGHT_MM);
}

/// One text run, centred in the box `[x_min, x_max] × [y_min, y_max]`. Nothing
/// is pushed for an empty string: an unnamed document or an empty note leaves
/// its cell blank rather than carrying a run with no characters in it.
fn push_text(
    out: &mut FrameDrawing,
    text: &str,
    x_min: f64,
    x_max: f64,
    y_min: f64,
    y_max: f64,
    height: f64,
) {
    let text = text.trim();
    if text.is_empty() || x_max <= x_min {
        return;
    }
    out.texts.push(SheetText::flat(
        text.to_string(),
        [(x_min + x_max) * 0.5, (y_min + y_max) * 0.5],
        height,
    ));
}

/// The FIRST placed view's scale as a ratio — `4:1`, `1:2`, `2.5:1` — or an em
/// dash on a sheet with nothing placed on it yet.
pub fn scale_text(sheet: &Sheet) -> String {
    match sheet.views.first() {
        Some(view) => ratio_text(view.scale),
        None => "\u{2014}".to_string(),
    }
}

/// A scale in paper millimetres per model unit as a drawing ratio — `4:1`,
/// `1:2`, `2.5:1` — or an em dash for one that is not a scale. The title
/// block's SCALE cell and a detail view's caption both read it.
pub fn ratio_text(scale: f64) -> String {
    if !scale.is_finite() || scale <= 0.0 {
        return "\u{2014}".to_string();
    }
    if scale >= 1.0 {
        format!("{}:1", trim(scale))
    } else {
        format!("1:{}", trim(1.0 / scale))
    }
}

/// `4` rather than `4.000`, `2.5` as `2.5`.
fn trim(value: f64) -> String {
    let text = format!("{value:.3}");
    let text = text.trim_end_matches('0').trim_end_matches('.');
    if text.is_empty() { "0".to_string() } else { text.to_string() }
}

/// Today, `YYYY-MM-DD`, in UTC.
///
/// UTC on both shells rather than local time on one of them: the same sheet
/// exported from the browser and from a headless host must carry the same date,
/// and the alternative is a native build with no time zone database pretending
/// to know one.
pub fn today_iso() -> String {
    iso_date(now_unix_seconds())
}

/// A unix timestamp as `YYYY-MM-DD` (UTC).
pub fn iso_date(unix_seconds: i64) -> String {
    let (year, month, day) = civil_from_days(unix_seconds.div_euclid(86_400));
    format!("{year:04}-{month:02}-{day:02}")
}

/// Days since 1970-01-01 as a civil date — Howard Hinnant's `civil_from_days`,
/// exact for every day in a proleptic Gregorian calendar and the reason this
/// file needs no date crate.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    // Shift the era so the leap day lands at the END of a 400-year cycle.
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let day_of_era = (z - era * 146_097) as i64; // [0, 146_096]
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365; // [0, 399]
    let year = year_of_era + era * 400;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100); // [0, 365]
    let month_prime = (5 * day_of_year + 2) / 153; // [0, 11] with March = 0
    let day = (day_of_year - (153 * month_prime + 2) / 5 + 1) as u32; // [1, 31]
    let month = if month_prime < 10 { month_prime + 3 } else { month_prime - 9 } as u32;
    (year + i64::from(month <= 2), month, day)
}

#[cfg(not(target_arch = "wasm32"))]
fn now_unix_seconds() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|since| since.as_secs() as i64)
        .unwrap_or(0)
}

/// The browser's clock. `std::time::SystemTime::now()` panics on
/// `wasm32-unknown-unknown`, which is why this is the one place in the crate
/// that asks JS for the time.
#[cfg(target_arch = "wasm32")]
fn now_unix_seconds() -> i64 {
    (js_sys::Date::now() / 1000.0) as i64
}


