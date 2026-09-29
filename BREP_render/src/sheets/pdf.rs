//! The sheet's PDF output contract — a minimal PDF 1.4 writer, no crate (1.7
//! when it carries a 3D model; see the last section).
//!
//! The same [`SheetDrawing`] the SVG writer serialises, written as pages of
//! **stroked paths and text**: ONE FILE for a list of sheets, one page per
//! sheet in the order given, each page's `MediaBox` its own paper size in
//! points (the paper is millimetres, a point is 1/72 inch), each drawing in a
//! single uncompressed content stream, and text set in the standard Helvetica
//! base font with WinAnsi encoding. No compression, no embedded font, no
//! transparency, no xobject — a file a text editor can read, whose xref table
//! points at real byte offsets. Every byte is ASCII: a character outside
//! printable ASCII is written as an octal escape inside its string literal, so
//! the file has no binary section and needs no binary marker.
//!
//! # The object graph
//!
//! ```text
//! 1  Catalog   /Pages 2 0 R
//! 2  Pages     /Count n  /Kids [5 0 R 7 0 R … (5 + 2(n−1)) 0 R]
//! 3  Font      Helvetica, WinAnsiEncoding — shared by every page
//! 4  Info      /Title (the document) /Producer (BREP) /HiddenLine (…)
//! 5  Page 1    /Parent 2 0 R  /MediaBox [0 0 w₁ h₁]  /Contents 6 0 R
//! 6  stream    page 1's marks
//! 7  Page 2    /Parent 2 0 R  /MediaBox [0 0 w₂ h₂]  /Contents 8 0 R
//! 8  stream    page 2's marks
//! …
//! ```
//!
//! The page tree is FLAT: one `Pages` node whose kids are every page. A
//! balanced tree only matters to a reader seeking into thousands of pages,
//! and a drawing set is not that; flat is what the specification's own
//! minimal example is. A single sheet is the one-page case of the same writer
//! ([`to_pdf`]), not a second one.
//!
//! # The frame
//!
//! PDF's origin is the page's BOTTOM-left with y running UP; a sheet is measured
//! from the TOP-left with y running down. Every coordinate therefore goes
//! through [`point`], which flips y about the page height — and a text run's
//! rotation flips with it, which is why the text matrix turns by the NEGATIVE of
//! the drawing's angle ([`text_matrix`]).
//!
//! # What the PDF cannot carry that the SVG does
//!
//! Two things, both about what a file says ABOUT itself rather than what it
//! draws:
//!
//! - **The grouping and the ids.** An SVG placement is a `<g id="SV2"
//!   data-view="VIEW1" data-scale="4">` a reader can select, script or style,
//!   and a placement that could not project carries its reason in a `<desc>`.
//!   This writer draws the marks only: the geometry is identical, the labelling
//!   is not there. A refused placement is therefore INVISIBLE in the PDF where
//!   the SVG says why it is blank — the panel and `sheet_state` are where that
//!   answer lives for both.
//! - **Characters outside WinAnsi.** The SVG hands any Unicode string to
//!   whatever renders it; a Helvetica base font is a 256-glyph encoding. What is
//!   drawn is what [`encode_winansi`] can map: Latin-1 plus the CP1252
//!   punctuation, with `⌀` written as `Ø` (the substitution every drawing
//!   standard already treats as the diameter sign) and `−` U+2212 as a hyphen.
//!   Anything else — the fourteen GD&T characteristic symbols an FCF cell draws,
//!   CJK in a note — becomes `?`. Everything the dimension formatter itself
//!   emits (digits, `.`, `±`, `°`, `⌀`, parentheses, `/`) IS carried.
//!
//! # Which pass drew the lines
//!
//! The Info dictionary's `/HiddenLine` says it for the whole file: `(analytic)`
//! when every placement that projected was drawn by the exact pass, and
//! `(mesh: Sheet 1 SV2, …)` naming each one the mesh approximation drew — a
//! placement whose B-rep had not reached the scene when the file was written
//! ([`ViewDrawing::hidden_line`]). An export from a background runner can
//! write a sheet no one has shown yet, and this is how that file says so.
//!
//! Everything else the SVG has is here: the same millimetres, the same visible
//! edge runs, the same annotation lines and text, the same border and title
//! block, the same stroke widths, and the same real text (selectable and
//! searchable, not stroked outlines).
//!
//! # The one exception: a live 3D model
//!
//! [`write_pdf_3d`] can carry the model in 3D ([`super::pdf3d`]) — over a
//! placement marked `threeD`, and on a last page given over to it. That file
//! is PDF **1.7**, its pages carry `/Annots` (the 3D annotations and the view
//! buttons), and each 3D annotation has a form xobject as its (empty)
//! appearance. It is still all ASCII: the U3D stream is ASCII85-encoded. A
//! file that asks for no 3D is not touched by any of this — [`write_pdf`] is
//! the no-3D case of the same writer, and its bytes are what they were.

use super::frame::FrameDrawing;
use super::project::{SheetDrawing, SheetText, ViewDrawing};
use super::svg::{ANNOTATION_WIDTH, CAP_PER_EM, EDGE_WIDTH, FRAME_WIDTH};

/// PDF user-space units per millimetre — a point is 1/72 of an inch.
pub const PT_PER_MM: f64 = 72.0 / 25.4;

/// The one font every PDF reader has without embedding.
const BASE_FONT: &str = "Helvetica";

/// Serialise one sheet as a one-page PDF titled with the sheet's name — the
/// one-page case of [`write_pdf`].
pub fn to_pdf(sheet: &SheetDrawing) -> Vec<u8> {
    write_pdf(&sheet.name, &[sheet])
}

/// Serialise `pages` as one PDF titled `title`, **one page per sheet**, in
/// order, each at its own paper size.
pub fn write_pdf(title: &str, pages: &[&SheetDrawing]) -> Vec<u8> {
    write_pdf_3d(title, pages, None)
}

/// [`write_pdf`], carrying a live 3D model ([`super::pdf3d`]) when `three_d`
/// is given: a 3D annotation over each placement it names, and — when it asks
/// for one — a last page given over to the model with a button per saved
/// view.
///
/// With 3D the file is PDF **1.7** (per-view node visibility is a 1.7 feature)
/// and pages carry `/Annots`; the rest of this module's contract holds — the
/// U3D stream is ASCII85-encoded, so every byte is still ASCII. Without it the
/// output is byte for byte [`write_pdf`]'s.
///
/// ```text
/// 5 … 4+2n      the sheet pages, (page, contents) pairs as above
/// 5+2n, 6+2n    the 3D page and its contents, when asked for
/// next          the 3D stream (the U3D, and every saved view in /VA)
/// then          per placement: its 3D annotation, its (empty) appearance
/// then          the 3D page's annotation, its appearance, a link per view
/// ```
pub fn write_pdf_3d(title: &str, pages: &[&SheetDrawing], three_d: Option<&super::pdf3d::Pdf3d>) -> Vec<u8> {
    let mut pdf = Writer::default();
    pdf.header(if three_d.is_some() { "1.7" } else { "1.4" });
    // Object numbers are assigned up front so the catalogue, the page tree and
    // the pages can reference each other: 1 catalogue, 2 page tree, 3 font,
    // 4 info, then a (page, contents) pair per sheet.
    let first_page = 5;
    let extra_page = three_d.and_then(|three_d| three_d.page.as_ref());
    let page_count = pages.len() + usize::from(extra_page.is_some());
    let plan = three_d.map(|three_d| super::pdf3d::Plan::new(three_d, first_page + page_count * 2, pages.len()));
    let kids: Vec<String> =
        (0..page_count).map(|index| format!("{} 0 R", first_page + index * 2)).collect();
    pdf.object(1, "<< /Type /Catalog /Pages 2 0 R >>");
    pdf.object(
        2,
        &format!("<< /Type /Pages /Count {} /Kids [{}] >>", page_count, kids.join(" ")),
    );
    pdf.object(
        3,
        &format!(
            "<< /Type /Font /Subtype /Type1 /BaseFont /{BASE_FONT} /Encoding /WinAnsiEncoding >>"
        ),
    );
    pdf.object(
        4,
        &format!(
            "<< /Title ({}) /Producer (BREP) /HiddenLine ({}) >>",
            string_literal(title),
            string_literal(&hidden_line(pages))
        ),
    );
    for (index, sheet) in pages.iter().enumerate() {
        let page = first_page + index * 2;
        let contents = page + 1;
        let (w, h) = (sheet.width_mm.max(1.0), sheet.height_mm.max(1.0));
        let annots = plan.as_ref().map(|plan| plan.annots_on(index)).unwrap_or_default();
        pdf.object(
            page,
            &format!(
                "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {:.4} {:.4}] \
                 /Resources << /Font << /F1 3 0 R >> >> /Contents {contents} 0 R{annots} >>",
                w * PT_PER_MM,
                h * PT_PER_MM
            ),
        );
        pdf.stream(contents, content_stream(sheet).as_bytes());
    }
    if let (Some(three_d), Some(plan)) = (three_d, plan.as_ref()) {
        if let Some(extra) = extra_page {
            let page = first_page + pages.len() * 2;
            let (w, h) = (extra.width_mm.max(1.0), extra.height_mm.max(1.0));
            pdf.object(
                page,
                &format!(
                    "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {:.4} {:.4}] \
                     /Resources << /Font << /F1 3 0 R >> >> /Contents {} 0 R{} >>",
                    w * PT_PER_MM,
                    h * PT_PER_MM,
                    page + 1,
                    plan.annots_on(pages.len())
                ),
            );
            pdf.stream(page + 1, model_page_stream(extra).as_bytes());
        }
        for (number, body) in plan.objects(three_d, pages, first_page) {
            pdf.object(number, &body);
        }
    }
    pdf.xref_and_trailer(1);
    pdf.out
}

/// The Info dictionary's `/HiddenLine`: `analytic`, or `mesh: ` and the sheet
/// and placement of every view the mesh approximation drew.
fn hidden_line(pages: &[&SheetDrawing]) -> String {
    let approximate: Vec<String> = pages
        .iter()
        .flat_map(|sheet| {
            sheet
                .views
                .iter()
                .filter(|view| view.hidden_line == super::project::HiddenLine::Mesh.label())
                .map(move |view| format!("{} {}", sheet.name, view.id))
        })
        .collect();
    if approximate.is_empty() {
        super::project::HiddenLine::Analytic.label().to_string()
    } else {
        format!("{}: {}", super::project::HiddenLine::Mesh.label(), approximate.join(", "))
    }
}

// ---------------------------------------------------------------------------
// The content stream
// ---------------------------------------------------------------------------

/// One page's marks: the paper's furniture, then every placement's model lines
/// and annotations — the SVG writer's order, so the two files layer alike.
fn content_stream(sheet: &SheetDrawing) -> String {
    let h = sheet.height_mm.max(1.0);
    let mut out = String::new();
    // Black, butt-free joins: the round caps the SVG asks for, so a thin
    // polyline reads the same in both files.
    out.push_str("0 G\n0 g\n1 J\n1 j\n");
    if let Some(frame) = &sheet.frame {
        frame_marks(frame, h, &mut out);
    }
    for view in &sheet.views {
        view_marks(view, h, &mut out);
    }
    // The SHEET's own dimensions, last: they are drawn over the placements
    // exactly as the SVG layers them.
    for dimension in &sheet.dimensions {
        if !dimension.lines.is_empty() {
            out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
            for line in &dimension.lines {
                out.push_str(&path(line, h));
            }
        }
        for text in &dimension.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    // …and the ordinate sets beside them, in the SVG's own order.
    for set in &sheet.ordinates {
        if !set.lines.is_empty() {
            out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
            for line in &set.lines {
                out.push_str(&path(line, h));
            }
        }
        for text in &set.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    out
}

/// The 3D page's marks: its title, the default view drawn as the sheets draw
/// a placement (what every reader without 3D shows, and what the 3D box
/// covers once active), the view buttons' boxes and labels, and the caption.
fn model_page_stream(page: &super::pdf3d::Page3d) -> String {
    let h = page.height_mm.max(1.0);
    let mut out = String::from("0 G\n0 g\n1 J\n1 j\n");
    out.push_str(&text_marks(&SheetText::flat(page.title.clone(), page.title_at, page.title_mm), h));
    match &page.poster {
        Some(poster) => view_marks(poster, h, &mut out),
        None => {
            // No drawing to show: the box outlined, so the page is not blank
            // where the model will be.
            let [x0, y0, x1, y1] = page.box_mm;
            out.push_str(&format!("{:.4} w\n", FRAME_WIDTH * PT_PER_MM));
            out.push_str(&path(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]], h));
        }
    }
    if !page.buttons.is_empty() {
        out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
        for button in &page.buttons {
            let [x0, y0, x1, y1] = button.rect_mm;
            out.push_str(&path(&[[x0, y0], [x1, y0], [x1, y1], [x0, y1], [x0, y0]], h));
            let centre = [(x0 + x1) * 0.5, (y0 + y1) * 0.5];
            out.push_str(&text_marks(&SheetText::flat(button.label.clone(), centre, page.button_text_mm), h));
        }
    }
    for (line, at) in page.caption.iter().zip(&page.caption_at) {
        out.push_str(&text_marks(&SheetText::flat(line.clone(), *at, page.caption_mm), h));
    }
    out
}

fn frame_marks(frame: &FrameDrawing, h: f64, out: &mut String) {
    if !frame.lines.is_empty() {
        out.push_str(&format!("{:.4} w\n", FRAME_WIDTH * PT_PER_MM));
        for line in &frame.lines {
            out.push_str(&path(line, h));
        }
    }
    for text in &frame.texts {
        out.push_str(&text_marks(text, h));
    }
    // The revision table in the frame's own stroke, after the stamp — the
    // SVG's order.
    if let Some(table) = &frame.revision_table {
        out.push_str(&format!("{:.4} w\n", FRAME_WIDTH * PT_PER_MM));
        for line in &table.lines {
            out.push_str(&path(line, h));
        }
        for text in &table.texts {
            out.push_str(&text_marks(text, h));
        }
    }
}

fn view_marks(view: &ViewDrawing, h: f64, out: &mut String) {
    if !view.edges.is_empty() {
        out.push_str(&format!("{:.4} w\n", EDGE_WIDTH * PT_PER_MM));
        for run in &view.edges {
            out.push_str(&path(run, h));
        }
    }
    // The SECTION's hatch and caption, and the section LINES this placement
    // carries — the SVG's own layering, so the two files read alike.
    if let Some(section) = &view.section {
        if !section.hatch.is_empty() {
            out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
            for line in &section.hatch {
                out.push_str(&path(line, h));
            }
        }
        for text in &section.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    for mark in &view.section_marks {
        if !mark.lines.is_empty() {
            out.push_str(&format!("{:.4} w\n", FRAME_WIDTH * PT_PER_MM));
            for line in &mark.lines {
                out.push_str(&path(line, h));
            }
        }
        for text in &mark.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    // A DETAIL's boundary and caption, then the detail circles drawn on this
    // placement — again the SVG's order.
    if let Some(detail) = &view.detail {
        out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
        out.push_str(&path(&detail.circle, h));
        for text in &detail.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    for mark in &view.detail_marks {
        if !mark.lines.is_empty() {
            out.push_str(&format!("{:.4} w\n", FRAME_WIDTH * PT_PER_MM));
            for line in &mark.lines {
                out.push_str(&path(line, h));
            }
        }
        for text in &mark.texts {
            out.push_str(&text_marks(text, h));
        }
    }
    for annotation in &view.annotations {
        if !annotation.lines.is_empty() {
            out.push_str(&format!("{:.4} w\n", ANNOTATION_WIDTH * PT_PER_MM));
            for line in &annotation.lines {
                out.push_str(&path(line, h));
            }
        }
        for text in &annotation.texts {
            out.push_str(&text_marks(text, h));
        }
    }
}

/// A polyline as one stroked path: `m`, `l`…, `S`.
fn path(points: &[[f64; 2]], h: f64) -> String {
    let mut out = String::new();
    for (index, p) in points.iter().enumerate() {
        let (x, y) = point(*p, h);
        out.push_str(&format!("{x:.4} {y:.4} {}\n", if index == 0 { "m" } else { "l" }));
    }
    if !out.is_empty() {
        out.push_str("S\n");
    }
    out
}

/// A text run as `BT … Tm (…) Tj ET`, centred on its anchor.
///
/// The run's own basis becomes the text matrix. `Tf` still carries the size
/// the cap height asks for, so `a b c d` is that basis divided by the height —
/// a rotation for a flat or merely rotated run, and the sheared pair an
/// oblique annotation plane projects to, which a rotation could not express.
fn text_marks(text: &SheetText, h: f64) -> String {
    let literal = string_literal(&text.text);
    if literal.is_empty() || !(text.height > 1e-12) {
        return String::new();
    }
    let size = (text.height / CAP_PER_EM) * PT_PER_MM;
    let width = text_width(&text.text);
    let (cx, cy) = point(text.anchor, h);
    // `Tf` carries the size, so the matrix is the run's basis NORMALISED by
    // its cap height — a rotation when the run is merely turned, the sheared
    // pair when an oblique annotation plane projected it. The page's y runs
    // UP and the drawing's runs DOWN, so a paper vector flips its second
    // component on the way in.
    let unit = |v: [f64; 2]| [v[0] / text.height, -v[1] / text.height];
    let (along, up) = (unit(text.right), unit(text.up));
    // Back off half the string along the baseline and half a cap height down
    // the run's own up direction, so it is centred on the anchor either way.
    let x = cx - (along[0] * width + up[0] * CAP_PER_EM) * size * 0.5;
    let y = cy - (along[1] * width + up[1] * CAP_PER_EM) * size * 0.5;
    format!(
        "BT /F1 {size:.4} Tf {} ({literal}) Tj ET\n",
        text_matrix(along, up, x, y)
    )
}

/// `a b c d e f Tm` from the run's basis normalised by its cap height:
/// `along` the baseline and `up` its own up direction.
fn text_matrix(along: [f64; 2], up: [f64; 2], x: f64, y: f64) -> String {
    format!(
        "{:.6} {:.6} {:.6} {:.6} {x:.4} {y:.4} Tm",
        along[0], along[1], up[0], up[1]
    )
}

/// A sheet millimetre point in PDF user space: y flips about the page height.
fn point(p: [f64; 2], height_mm: f64) -> (f64, f64) {
    (p[0] * PT_PER_MM, (height_mm - p[1]) * PT_PER_MM)
}

// ---------------------------------------------------------------------------
// Text: WinAnsi encoding and Helvetica metrics
// ---------------------------------------------------------------------------

/// `text` as a PDF string literal's CONTENTS (no parentheses): WinAnsi bytes,
/// with `(`, `)` and `\` escaped and everything outside printable ASCII written
/// as a three-digit octal escape, so a content stream stays readable ASCII.
pub fn string_literal(text: &str) -> String {
    let mut out = String::new();
    for byte in encode_winansi(text) {
        match byte {
            b'(' | b')' | b'\\' => {
                out.push('\\');
                out.push(byte as char);
            }
            0x20..=0x7e => out.push(byte as char),
            _ => out.push_str(&format!("\\{byte:03o}")),
        }
    }
    out
}

/// `text` in WinAnsi (CP1252) bytes. A character the encoding has no glyph for
/// becomes `?` — see this module's header for what that costs.
pub fn encode_winansi(text: &str) -> Vec<u8> {
    text.chars().map(|ch| winansi(ch).unwrap_or(b'?')).collect()
}

/// The 0x80-0x9f band of WinAnsi (CP1252), the one place it differs from ISO
/// 8859-1: `(character, byte)`.
const CP1252_BAND: [(char, u8); 27] = [
    ('\u{20ac}', 0x80), // euro
    ('\u{201a}', 0x82),
    ('\u{192}', 0x83),
    ('\u{201e}', 0x84),
    ('\u{2026}', 0x85), // ellipsis
    ('\u{2020}', 0x86),
    ('\u{2021}', 0x87),
    ('\u{2c6}', 0x88),
    ('\u{2030}', 0x89),
    ('\u{160}', 0x8a),
    ('\u{2039}', 0x8b),
    ('\u{152}', 0x8c),
    ('\u{17d}', 0x8e),
    ('\u{2018}', 0x91),
    ('\u{2019}', 0x92),
    ('\u{201c}', 0x93),
    ('\u{201d}', 0x94),
    ('\u{2022}', 0x95), // bullet
    ('\u{2013}', 0x96), // en dash
    ('\u{2014}', 0x97), // em dash
    ('\u{2dc}', 0x98),
    ('\u{2122}', 0x99), // trade mark
    ('\u{161}', 0x9a),
    ('\u{203a}', 0x9b),
    ('\u{153}', 0x9c),
    ('\u{17e}', 0x9e),
    ('\u{178}', 0x9f),
];

/// One character's WinAnsi code point.
fn winansi(ch: char) -> Option<u8> {
    if let Some((_, byte)) = CP1252_BAND.iter().find(|(band, _)| *band == ch) {
        return Some(*byte);
    }
    Some(match ch {
        // ASCII and the Latin-1 upper half are WinAnsi's own, with the 0x80-0x9f
        // band the one place CP1252 differs from ISO 8859-1.
        '\u{20}'..='\u{7e}' | '\u{a0}'..='\u{ff}' => ch as u8,
        // The two substitutions a drawing needs. `⌀` U+2300 DIAMETER SIGN is
        // what the kernel formats a diameter with and is not in WinAnsi; `Ø` is
        // the glyph every drawing standard already accepts for it. U+2212 MINUS
        // SIGN is the formatter's minus, and a hyphen is what Helvetica has.
        '\u{2300}' => 0xd8,
        '\u{2212}' => b'-',
        _ => return None,
    })
}

/// Every text string a PDF this module wrote shows, in page order: each
/// `(…) Tj` literal read back from the file's own bytes — escapes undone and
/// WinAnsi decoded, so a `⌀` the writer substituted reads `Ø` and a U+2212 a
/// hyphen, which is what the file says. What the export door reports ABOUT the
/// file, so a script can assert the drawing's text without a PDF reader.
pub fn text_strings(pdf: &[u8]) -> Vec<String> {
    let mut out = Vec::new();
    let mut at = 0;
    while let Some(open) = pdf[at..].iter().position(|b| *b == b'(').map(|i| at + i) {
        let mut bytes = Vec::new();
        let mut i = open + 1;
        while i < pdf.len() && pdf[i] != b')' {
            if pdf[i] == b'\\' && i + 1 < pdf.len() {
                let digits: Vec<u8> = pdf[i + 1..].iter().take(3).take_while(|b| (b'0'..=b'7').contains(b)).copied().collect();
                if digits.is_empty() {
                    bytes.push(pdf[i + 1]);
                    i += 2;
                } else {
                    bytes.push(digits.iter().fold(0u32, |n, d| n * 8 + (d - b'0') as u32) as u8);
                    i += 1 + digits.len();
                }
            } else {
                bytes.push(pdf[i]);
                i += 1;
            }
        }
        at = i + 1;
        if pdf[at.min(pdf.len())..].starts_with(b" Tj") {
            out.push(bytes.iter().map(|byte| from_winansi(*byte)).collect());
        }
    }
    out
}

/// One WinAnsi byte's character.
fn from_winansi(byte: u8) -> char {
    CP1252_BAND.iter().find(|(_, band)| *band == byte).map(|(ch, _)| *ch).unwrap_or(byte as char)
}

/// A string's advance in ems, from Helvetica's own metrics.
pub fn text_width(text: &str) -> f64 {
    encode_winansi(text).iter().map(|byte| advance(*byte)).sum::<f64>() / 1000.0
}

/// One WinAnsi byte's Helvetica advance in 1/1000 em. The printable ASCII range
/// is the AFM's; the accented Latin-1 letters take their base letter's, which
/// they share in this face, and anything else takes the average — a width is
/// only used to CENTRE a run, so being a few thousandths out moves a title
/// block's text by less than a stroke.
fn advance(byte: u8) -> f64 {
    const ASCII: [u16; 95] = [
        278, 278, 355, 556, 556, 889, 667, 191, 333, 333, 389, 584, 278, 333, 278, 278, // 32..47
        556, 556, 556, 556, 556, 556, 556, 556, 556, 556, 278, 278, 584, 584, 584, 556, // 48..63
        1015, 667, 667, 722, 722, 667, 611, 778, 722, 278, 500, 667, 556, 833, 722, 778, // 64..79
        667, 778, 722, 667, 611, 722, 667, 944, 667, 667, 611, 278, 278, 278, 469, 556, // 80..95
        333, 556, 556, 500, 556, 556, 278, 556, 556, 222, 222, 500, 222, 833, 556, 556, // 96..111
        556, 556, 333, 500, 278, 556, 500, 722, 500, 500, 500, 334, 260, 334, 584, // 112..126
    ];
    match byte {
        0x20..=0x7e => f64::from(ASCII[(byte - 0x20) as usize]),
        0xb0 => 400.0, // degree
        0xb1 => 584.0, // plus-minus
        0xd7 => 584.0, // multiply
        0xd8 => 778.0, // O with stroke — the diameter sign
        0xa0 => 278.0, // no-break space
        // The accented capitals, which carry their base letter's width in this
        // face; everything else (the accented lower case, the symbols) takes the
        // average, which only moves a centred run by a fraction of a stroke.
        0xc0..=0xdd => 722.0,
        _ => 556.0,
    }
}

// ---------------------------------------------------------------------------
// The file
// ---------------------------------------------------------------------------

/// The bytes, and where each object started — the xref table IS those offsets,
/// so they are recorded as the objects are written rather than counted twice.
#[derive(Default)]
struct Writer {
    out: Vec<u8>,
    /// `offsets[n - 1]` is object `n`'s byte offset.
    offsets: Vec<usize>,
}

impl Writer {
    fn header(&mut self, version: &str) {
        self.out.extend_from_slice(format!("%PDF-{version}\n").as_bytes());
        // No binary marker comment after it: this writer emits no byte above
        // 0x7e (every string literal is octal-escaped and no stream is
        // compressed), so the whole file is ASCII and the marker would be the
        // only thing in it claiming otherwise.
    }

    fn begin(&mut self, number: usize) {
        assert_eq!(number, self.offsets.len() + 1, "objects are written in order");
        self.offsets.push(self.out.len());
        self.out.extend_from_slice(format!("{number} 0 obj\n").as_bytes());
    }

    fn object(&mut self, number: usize, body: &str) {
        self.begin(number);
        self.out.extend_from_slice(body.as_bytes());
        self.out.extend_from_slice(b"\nendobj\n");
    }

    fn stream(&mut self, number: usize, content: &[u8]) {
        self.begin(number);
        self.out
            .extend_from_slice(format!("<< /Length {} >>\nstream\n", content.len()).as_bytes());
        self.out.extend_from_slice(content);
        self.out.extend_from_slice(b"endstream\nendobj\n");
    }

    /// The cross-reference table and the trailer. Entries are exactly twenty
    /// bytes, free object 0 first, which is what lets a reader seek by object
    /// number.
    fn xref_and_trailer(&mut self, root: usize) {
        let start = self.out.len();
        let count = self.offsets.len() + 1;
        self.out.extend_from_slice(format!("xref\n0 {count}\n").as_bytes());
        self.out.extend_from_slice(b"0000000000 65535 f \n");
        for offset in &self.offsets {
            self.out.extend_from_slice(format!("{offset:010} 00000 n \n").as_bytes());
        }
        self.out.extend_from_slice(
            format!("trailer\n<< /Size {count} /Root {root} 0 R /Info 4 0 R >>\nstartxref\n{start}\n%%EOF\n")
                .as_bytes(),
        );
    }
}




