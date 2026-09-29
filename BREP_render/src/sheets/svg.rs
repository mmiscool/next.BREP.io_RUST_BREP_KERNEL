//! The sheet's SVG output contract.
//!
//! One file per sheet. The paper is the `viewBox` — `0 0 <width> <height>` in
//! millimetres, with `width` / `height` carrying the `mm` unit, so the file
//! prints at its real size. The paper's furniture comes first as one
//! `<g class="sheet-frame">` (the border, the title block's rules and its text),
//! then each placed view is one `<g>` carrying its placed-view id, its saved
//! view's id, its scale and which pass drew its model lines (`data-hidden-line`:
//! `analytic`, or `mesh` for the approximation drawn before the B-rep arrived). Inside a view group the model is `<path>` elements
//! (the visible edge runs, one path per run) and each annotation is its own
//! `<g>` of paths plus `<text>` elements holding the kernel's own display
//! string, so the value in the file is the value in the viewport and a reader
//! can select it.
//!
//! **Lines and text only.** There are no fills anywhere: an arrowhead is its
//! closed outline, a datum triangle and an FCF frame are their borders, and the
//! border and title block are polylines. The same drawing goes out as PDF
//! through [`super::pdf`], which shares the stroke widths and the cap-height
//! ratio below so the two files cannot drift.

use super::dimension::DimensionDrawing;
use super::ordinate::OrdinateDrawing;
use super::frame::FrameDrawing;
use super::project::{AnnotationDrawing, SheetDrawing, SheetText, ViewDrawing};

/// Model edge stroke width, millimetres.
pub(super) const EDGE_WIDTH: f64 = 0.35;
/// Annotation stroke width, millimetres.
pub(super) const ANNOTATION_WIDTH: f64 = 0.18;
/// The border and title-block stroke width, millimetres.
pub(super) const FRAME_WIDTH: f64 = 0.5;
/// A text run's CAP height as a fraction of the font's em. The layout sizes a
/// cap; an SVG `font-size` and a PDF `Tf` size are both the em, so this is the
/// one conversion both writers (and the viewport) have to agree on.
pub(super) const CAP_PER_EM: f64 = 0.72;

/// Serialise a projected sheet.
pub fn to_svg(sheet: &SheetDrawing) -> String {
    let (w, h) = (sheet.width_mm.max(1.0), sheet.height_mm.max(1.0));
    let mut out = String::new();
    out.push_str(&format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{w:.6}mm\" height=\"{h:.6}mm\" viewBox=\"0 0 {w:.6} {h:.6}\">\n"
    ));
    out.push_str(&format!("<title>{}</title>\n", escape(&sheet.name)));
    if let Some(frame) = &sheet.frame {
        out.push_str(&frame_group(frame));
    }
    for view in &sheet.views {
        out.push_str(&view_group(view));
    }
    if !sheet.dimensions.is_empty() {
        out.push_str(&dimension_group(&sheet.dimensions));
    }
    if !sheet.ordinates.is_empty() {
        out.push_str(&ordinate_group(&sheet.ordinates));
    }
    out.push_str("</svg>\n");
    out
}

/// The paper's furniture: the border and the title block, lines and text, in
/// one group a reader can turn off.
fn frame_group(frame: &FrameDrawing) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "<g class=\"sheet-frame\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{FRAME_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n"
    ));
    for line in &frame.lines {
        out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
    }
    for text in &frame.texts {
        out.push_str(&text_element(text));
    }
    // The REVISION TABLE, as its own group inside the frame's: it IS paper
    // furniture, and a drawing-office tool looking for the revisions finds
    // them by class with the row count and the place the table went.
    if let Some(table) = &frame.revision_table {
        out.push_str(&format!(
            "<g class=\"revision-table\" data-rows=\"{}\" data-placement=\"{}\">\n",
            table.rows,
            escape(&table.placement)
        ));
        for line in &table.lines {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &table.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    out.push_str("</g>\n");
    out
}

fn view_group(view: &ViewDrawing) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "<g class=\"sheet-view\" id=\"{}\" data-view=\"{}\" data-name=\"{}\" data-scale=\"{:.6}\"{}>\n",
        escape(&view.id),
        escape(&view.view),
        escape(&view.name),
        view.scale,
        if view.hidden_line.is_empty() { String::new() } else { format!(" data-hidden-line=\"{}\"", escape(&view.hidden_line)) }
    ));
    if !view.error.is_empty() {
        // A placement that could not project says so IN the file rather than
        // leaving a silent blank on the paper.
        out.push_str(&format!("<desc>{}</desc>\n", escape(&view.error)));
    }
    if !view.edges.is_empty() {
        out.push_str(&format!(
            "<g class=\"model\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{EDGE_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n"
        ));
        for run in &view.edges {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(run)));
        }
        out.push_str("</g>\n");
    }
    // A SECTION's cut face, under the annotations: the hatch is a fill drawn
    // as parallel LINES, because the sheet's whole contract is lines and text
    // and a `fill` would be the one mark in the file a plotter could not draw
    // the same way twice.
    if let Some(section) = &view.section {
        out.push_str(&format!(
            "<g class=\"section-hatch\" data-label=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{ANNOTATION_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&section.label)
        ));
        for line in &section.hatch {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &section.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    // …and the section LINES this placement carries for the sections cut
    // through it, each naming the view it is the cut of.
    for mark in &view.section_marks {
        out.push_str(&format!(
            "<g class=\"section-line\" data-section=\"{}\" data-label=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{FRAME_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&mark.id),
            escape(&mark.label)
        ));
        for line in &mark.lines {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &mark.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    // A DETAIL's boundary circle and caption, inside the detail's own group
    // and carrying its letter and its scale…
    if let Some(detail) = &view.detail {
        out.push_str(&format!(
            "<g class=\"detail-view\" data-label=\"{}\" data-scale=\"{:.6}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{ANNOTATION_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&detail.label),
            view.scale
        ));
        out.push_str(&format!("<path d=\"{}\" />\n", path_data(&detail.circle)));
        for text in &detail.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    // …and the detail CIRCLES this placement carries for the details drawn
    // from it, each naming the view it is the region of — the section's two
    // halves, in the same two places.
    for mark in &view.detail_marks {
        out.push_str(&format!(
            "<g class=\"detail-circle\" data-detail=\"{}\" data-label=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{FRAME_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&mark.id),
            escape(&mark.label)
        ));
        for line in &mark.lines {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &mark.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    for annotation in &view.annotations {
        out.push_str(&annotation_group(annotation));
    }
    out.push_str("</g>\n");
    out
}

/// The SHEET's own dimensions, in one group of their own — they belong to the
/// paper rather than to any placement, so they sit beside the view groups and
/// not inside one. Each is a `<g>` carrying its id, its kind and its value, so
/// a drawing-office tool can read what the sheet measured without re-deriving
/// it from the paths.
///
/// A dimension that did NOT resolve writes its reason as a `<desc>` and no
/// geometry: a lost anchor is said out loud, the way a refused placement is,
/// rather than leaving a silent blank. A dimension whose tolerance block is a
/// PMI dimension's carries that dimension's id as `data-tolerance-from`.
fn dimension_group(dimensions: &[DimensionDrawing]) -> String {
    let mut out = String::from("<g class=\"sheet-dimensions\">\n");
    for dimension in dimensions {
        let from = if dimension.tolerance_from.is_empty() {
            String::new()
        } else {
            format!(" data-tolerance-from=\"{}\"", escape(&dimension.tolerance_from))
        };
        out.push_str(&format!(
            "<g class=\"sheet-dimension\" id=\"{}\" data-type=\"{}\" data-value=\"{}\"{from} fill=\"none\" stroke=\"#000000\" stroke-width=\"{ANNOTATION_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&dimension.id),
            escape(&dimension.kind),
            escape(&dimension.text)
        ));
        if !dimension.error.is_empty() {
            out.push_str(&format!("<desc>{}</desc>\n", escape(&dimension.error)));
        }
        for line in &dimension.lines {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &dimension.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    out.push_str("</g>\n");
    out
}

/// The sheet's ORDINATE SETS, in their own group beside the dimensions.
///
/// Each set is a `<g>` carrying its id and its axis, and each STATION is a
/// nested `<g>` carrying the anchor it reads, its value and whether it is the
/// datum — so a drawing-office tool can read the run without re-deriving which
/// leader belongs to which point. A station that did not resolve writes its
/// reason as a `<desc>` and no geometry, exactly as a refused dimension does;
/// the rest of the set still draws, because losing one point of five is not
/// losing the set.
fn ordinate_group(ordinates: &[OrdinateDrawing]) -> String {
    let mut out = String::from("<g class=\"sheet-ordinates\">\n");
    for set in ordinates {
        out.push_str(&format!(
            "<g class=\"sheet-ordinate\" id=\"{}\" data-axis=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{ANNOTATION_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
            escape(&set.id),
            escape(&set.axis)
        ));
        if !set.error.is_empty() {
            out.push_str(&format!("<desc>{}</desc>\n", escape(&set.error)));
        }
        for station in &set.stations {
            out.push_str(&format!(
                "<g class=\"{}\" data-anchor=\"{}\" data-value=\"{}\">\n",
                if station.datum { "ordinate-datum" } else { "ordinate-member" },
                escape(&station.anchor),
                escape(&station.text)
            ));
            if !station.error.is_empty() {
                out.push_str(&format!("<desc>{}</desc>\n", escape(&station.error)));
            }
            out.push_str("</g>\n");
        }
        // The leaders, the origin circle and the values, once for the set: the
        // stations above say WHAT was measured, these are the marks.
        for line in &set.lines {
            out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
        }
        for text in &set.texts {
            out.push_str(&text_element(text));
        }
        out.push_str("</g>\n");
    }
    out.push_str("</g>\n");
    out
}

fn annotation_group(annotation: &AnnotationDrawing) -> String {
    let mut out = String::new();
    out.push_str(&format!(
        "<g class=\"annotation\" id=\"{}\" data-type=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{ANNOTATION_WIDTH:.6}\" stroke-linecap=\"round\" stroke-linejoin=\"round\">\n",
        escape(&annotation.id),
        escape(&annotation.kind)
    ));
    for line in &annotation.lines {
        out.push_str(&format!("<path d=\"{}\" />\n", path_data(line)));
    }
    for text in &annotation.texts {
        out.push_str(&text_element(text));
    }
    out.push_str("</g>\n");
    out
}

/// One placed text run as a `<text>` element: centred on its anchor, and
/// selectable, because the file hands the string to whatever renders it rather
/// than stroking it.
///
/// The run's own basis becomes the element's TRANSFORM. The element is placed
/// at the origin of a local frame whose x axis is the run's reading direction
/// and whose y axis runs DOWN its up direction, each one cap height long and
/// divided by the height so a flat run's matrix is the identity and is left
/// off. An oblique annotation plane gives a basis that is sheared, not merely
/// rotated, and `matrix` is what carries it — a `rotate` cannot.
fn text_element(text: &SheetText) -> String {
    let (x, y) = (text.anchor[0], text.anchor[1]);
    // The cap height is what the layout sizes; an SVG `font-size` is the em,
    // and a cap is about [`CAP_PER_EM`] of it for the sans fallbacks this asks
    // for — the one place a sheet has to guess. The PDF writer converts with the
    // same ratio, so the two files agree.
    let font = text.height / CAP_PER_EM;
    let transform = match text_basis(text) {
        None => String::new(),
        Some([a, b, c, d]) => {
            format!(" transform=\"matrix({a:.6} {b:.6} {c:.6} {d:.6} {x:.6} {y:.6})\"")
        }
    };
    let (x, y) = if transform.is_empty() { (x, y) } else { (0.0, 0.0) };
    format!(
        "<text x=\"{x:.6}\" y=\"{y:.6}\"{transform} font-family=\"sans-serif\" font-size=\"{font:.6}\" text-anchor=\"middle\" dominant-baseline=\"central\" fill=\"#000000\" stroke=\"none\">{}</text>\n",
        escape(&text.text)
    )
}

/// The run's basis as `a b c d` of an SVG matrix, normalised by its cap
/// height (x along the reading direction, y DOWN the run's up direction), or
/// `None` when that is the identity — a run laid flat on the paper, which is
/// the whole title block and every run of a placement with `flattenText` on.
/// A run with no height left to normalise by is not drawn at all.
fn text_basis(text: &SheetText) -> Option<[f64; 4]> {
    if !(text.height > 1e-12) {
        return None;
    }
    let m = [
        text.right[0] / text.height,
        text.right[1] / text.height,
        -text.up[0] / text.height,
        -text.up[1] / text.height,
    ];
    let identity = [1.0, 0.0, 0.0, 1.0];
    (0..4).any(|i| (m[i] - identity[i]).abs() > 1e-9).then_some(m)
}

/// A polyline as `M x y L x y …` with `{:.6}` coordinates.
fn path_data(points: &[[f64; 2]]) -> String {
    let mut out = String::new();
    for (i, p) in points.iter().enumerate() {
        out.push_str(if i == 0 { "M " } else { " L " });
        out.push_str(&format!("{:.6} {:.6}", p[0], p[1]));
    }
    out
}

/// XML text escaping — the five predefined entities. A note's text is the
/// user's own, so this is the boundary that keeps it from being markup.
/// Every text string an SVG this module wrote shows, in document order: each
/// `<text>` element's content read back from the file, entities undone. What
/// the export door reports ABOUT the file, so a script can assert the drawing's
/// text without an XML reader.
pub fn text_strings(svg: &str) -> Vec<String> {
    let mut out = Vec::new();
    let mut rest = svg;
    while let Some(open) = rest.find("<text ") {
        let after = &rest[open..];
        let (Some(start), Some(end)) = (after.find('>'), after.find("</text>")) else { break };
        if start < end {
            out.push(
                after[start + 1..end]
                    .replace("&lt;", "<")
                    .replace("&gt;", ">")
                    .replace("&quot;", "\"")
                    .replace("&apos;", "'")
                    .replace("&amp;", "&"),
            );
        }
        rest = &after[end + "</text>".len()..];
    }
    out
}

fn escape(text: &str) -> String {
    let mut out = String::with_capacity(text.len());
    for ch in text.chars() {
        match ch {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(ch),
        }
    }
    out
}





