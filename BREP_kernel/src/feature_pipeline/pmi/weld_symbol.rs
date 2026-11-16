//! Weld symbols for the `weldCallout` PMI type: AWS A2.4 by default, ISO 2553
//! as an option, the text leader as the fallback (`style: TEXT`).
//!
//! The callout resolves to a text leader (its seam anchor and the weld's
//! kind and size). The symbol is built here from the callout's fields and the
//! weld feature's own parameters ([`WeldSymbol::from_params`]) and carried on
//! the report row, so every consumer draws the same symbol from it
//! ([`present`], [`present_row`]): the 3D view, the 3D PDF, the drawing
//! sheets and the STEP presentation. It is laid out in the view plane (or the
//! annotation's picked plane) as the primitives every PMI type is drawn with:
//! polylines, filled triangles and text runs.
//!
//! Each weld kind draws its standard basic symbol: the fillet triangle (tack
//! welds too), the groove symbols (square, V, bevel, U, J), the plug or slot
//! rectangle, the spot circle, the seam circle crossed by two lines, and the
//! surfacing arcs. Edge and corner welds keep the text leader.
use super::layout::{LayoutStyle, Presentation, TextRun};
use super::PmiGeometry;
use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

/// The symbol fields of a `weldCallout`, in form order, as schema entries.
pub fn symbol_fields() -> Vec<(&'static str, Value)> {
    vec![
        ("style", json!({"type": "options", "label": "Style", "options": ["SYMBOL", "TEXT"], "default_value": "SYMBOL",
            "hint": "SYMBOL draws the standard weld symbol; TEXT keeps the plain text leader."})),
        ("standard", json!({"type": "options", "label": "Standard", "options": ["AWS", "ISO"], "default_value": "AWS",
            "hint": "AWS A2.4 (the arrow side's symbol below the reference line) or ISO 2553 (solid and dashed reference lines, z leg size)."})),
        ("side", json!({"type": "options", "label": "Weld side", "options": ["ARROW", "OTHER", "BOTH"], "default_value": "ARROW",
            "hint": "Which side of the joint the weld is on, relative to the arrow. A weld on both sides (a fillet's Sides BOTH, a double groove) draws both."})),
        ("length", json!({"type": "number", "label": "Length", "default_value": 0,
            "hint": "Weld length (each segment of an intermittent weld). 0 = from the weld feature (its segment length), else continuous."})),
        ("pitch", json!({"type": "number", "label": "Pitch", "default_value": 0,
            "hint": "Centre-to-centre spacing of an intermittent weld. 0 = from the weld feature, else not intermittent."})),
        ("allAround", json!({"type": "boolean", "label": "All around", "default_value": false,
            "hint": "Weld all around the joint (circle at the elbow); a weld feature welded all around draws it too."})),
        ("fieldWeld", json!({"type": "boolean", "label": "Field weld", "default_value": false,
            "hint": "Weld made in the field (flag at the elbow)."})),
    ]
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Standard {
    Aws,
    Iso,
}

#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Side {
    Arrow,
    Other,
    Both,
}

/// A weld kind's basic symbol.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Glyph {
    Fillet,
    Square,
    V,
    Bevel,
    U,
    J,
    Plug,
    Spot,
    Seam,
    Surfacing,
}

/// What one weld symbol says.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WeldSymbol {
    pub standard: Standard,
    pub side: Side,
    pub glyph: Glyph,
    /// The size text on the elbow side of the symbol (`6`, `6x8`, `z6`, `⌀5`).
    pub size: String,
    /// The length / pitch text beyond the symbol (`50-150`, `3 x 50 (100)`).
    pub length: String,
    pub all_around: bool,
    pub field_weld: bool,
    /// Process / specification and the annotation's extra note, tail text.
    pub tail: String,
    /// Intermittent on both sides, the far side's segments half a pitch
    /// along: the other side's symbol is drawn offset along the line.
    #[serde(default)]
    pub staggered: bool,
}

/// The length of a weld's seam path, measured along its curves.
pub fn path_length(curves: &[crate::NurbsCurve]) -> f64 {
    let mut length = 0.0;
    for curve in curves {
        let Ok([t0, t1]) = curve.domain() else { continue };
        let mut previous: Option<crate::Vec3> = None;
        for k in 0..=512 {
            let Ok(point) = curve.evaluate(t0 + (t1 - t0) * k as f64 / 512.0) else { continue };
            if let Some(last) = previous { length += point.sub(last).length(); }
            previous = Some(point);
        }
    }
    length
}

/// How many segments, tacks or spots one side of a weld lays, from its
/// parameters and its seam's length, as the weld feature lays them (`None`
/// for a continuous weld, or a number that does not evaluate).
pub fn segments(kind: &str, weld: &Value, seam_length: f64, evaluate: Evaluate<'_>) -> Option<usize> {
    let get = |key: &str, default: f64| -> Option<f64> {
        match weld.get(key) { None => Some(default), value => number(key, value, evaluate).ok() }
    };
    let count = |span: f64, pitch: f64| -> Option<usize> {
        (span >= -1e-9 && pitch > 0.0).then(|| ((span.max(0.0) / pitch + 1e-9).floor() + 1.0) as usize)
    };
    match kind {
        "WELD" if matches!(text(weld.get("intermittent")), "CHAIN" | "STAGGERED") => {
            let (segment, pitch) = (get("segmentLength", 10.0)?, get("pitch", 25.0)?);
            count(seam_length - segment, pitch)
        }
        "WELD.TACK" => get("count", 3.0).map(|n| n.max(0.0) as usize),
        "WELD.SPOT" => {
            let (pitch, end) = (get("pitch", 25.0)?, get("endDistance", 10.0)?);
            count(seam_length - 2.0 * end, pitch)
        }
        _ => None,
    }
}

/// A numeric parameter: a number, or an expression the caller's evaluator reads
/// (the document's Expressions, as features are evaluated).
pub type Evaluate<'a> = &'a dyn Fn(&str) -> Option<f64>;

/// A number, an expression that evaluates, or absent (0). An expression that
/// does not evaluate is an error naming the field, never a silent 0.
fn number(field: &str, value: Option<&Value>, evaluate: Evaluate<'_>) -> Result<f64, String> {
    match value {
        Some(Value::Number(n)) => Ok(n.as_f64().unwrap_or(0.0)),
        Some(Value::String(s)) if s.trim().is_empty() => Ok(0.0),
        Some(Value::String(s)) => s.trim().parse().ok().or_else(|| evaluate(s))
            .ok_or_else(|| format!("{field}: cannot evaluate `{}`", s.trim())),
        _ => Ok(0.0),
    }
}

/// Literal numbers only: for callers with no expression environment.
pub fn literal(_: &str) -> Option<f64> {
    None
}

fn text<'a>(value: Option<&'a Value>) -> &'a str {
    value.and_then(Value::as_str).unwrap_or("")
}

/// A dimension number without a trailing `.0`, at most two decimals.
fn fmt(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    let text = format!("{rounded:.2}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
}

impl WeldSymbol {
    /// The symbol a callout asks for. `kind` is the weld feature's type and
    /// `weld` its `inputParams`; `segments` is how many segments (or spots,
    /// or tacks) the weld laid on one side, for ISO's `n`. Where the feature
    /// says how the weld runs (intermittent segments, all around, both sides)
    /// it wins over the callout's own fields, which stay for a weld feature
    /// that does not say. `Ok(None)` for the text-leader style and for a kind
    /// with no symbol here (edge, corner); `Err` names a number that does not
    /// evaluate.
    pub fn from_params(annotation: &Value, kind: &str, weld: &Value, evaluate: Evaluate<'_>, segments: Option<usize>) -> Result<Option<Self>, String> {
        if text(annotation.get("style")) == "TEXT" {
            return Ok(None);
        }
        let standard = if text(annotation.get("standard")) == "ISO" { Standard::Iso } else { Standard::Aws };
        let mut side = match text(annotation.get("side")) {
            "OTHER" => Side::Other,
            "BOTH" => Side::Both,
            _ => Side::Arrow,
        };
        let iso = standard == Standard::Iso;
        let feature = |key: &str| number(&format!("weld {key}"), weld.get(key), evaluate);
        let flag = |key: &str| weld.get(key).and_then(Value::as_bool).unwrap_or(false);
        let mut length = number("length", annotation.get("length"), evaluate)?.max(0.0);
        let mut pitch = number("pitch", annotation.get("pitch"), evaluate)?.max(0.0);
        let mut all_around = annotation.get("allAround").and_then(Value::as_bool).unwrap_or(false) || flag("allAround");
        let mut tail_kind: Option<&str> = None;
        let staggered = kind == "WELD" && text(weld.get("intermittent")) == "STAGGERED" && text(weld.get("sides")) == "BOTH";
        let fillet_size = || -> Result<String, String> {
            let first = match weld.get("legSize") { None => 3.0, value => number("weld leg size", value, evaluate)? };
            let second = feature("legSize2")?;
            let base = if second > 0.0 && (second - first).abs() > 1e-12 { format!("{}x{}", fmt(first), fmt(second)) } else { fmt(first) };
            Ok(if iso { format!("z{base}") } else { base })
        };
        let (glyph, size) = match kind {
            "WELD" => {
                if text(weld.get("sides")) == "BOTH" { side = Side::Both; }
                if matches!(text(weld.get("intermittent")), "CHAIN" | "STAGGERED") {
                    length = match weld.get("segmentLength") { None => 10.0, value => number("weld segment length", value, evaluate)? };
                    pitch = match weld.get("pitch") { None => 25.0, value => number("weld pitch", value, evaluate)? };
                }
                (Glyph::Fillet, fillet_size()?)
            }
            "WELD.TACK" => {
                if text(weld.get("sides")) == "BOTH" { side = Side::Both; }
                length = match weld.get("tackLength") { None => 5.0, value => number("weld tack length", value, evaluate)? };
                pitch = 0.0;
                tail_kind = Some("TACK");
                (Glyph::Fillet, fillet_size()?)
            }
            "WELD.GROOVE" => {
                if flag("double") { side = Side::Both; }
                let shape = match text(weld.get("shape")) {
                    "SQUARE" => Glyph::Square,
                    "BEVEL" => Glyph::Bevel,
                    "U" => Glyph::U,
                    "J" => Glyph::J,
                    _ => Glyph::V,
                };
                let angle = match weld.get("grooveAngle") { None => 60.0, value => number("weld groove angle", value, evaluate)? };
                (shape, if shape == Glyph::Square { String::new() } else { format!("{}°", fmt(angle)) })
            }
            "WELD.SPOT" => {
                pitch = match weld.get("pitch") { None => 25.0, value => number("weld pitch", value, evaluate)? };
                let diameter = match weld.get("diameter") { None => 5.0, value => number("weld nugget diameter", value, evaluate)? };
                length = 0.0;
                (Glyph::Spot, format!("\u{2300}{}", fmt(diameter)))
            }
            "WELD.RSEAM" => {
                let width = match weld.get("width") { None => 5.0, value => number("weld width", value, evaluate)? };
                (Glyph::Seam, fmt(width))
            }
            "WELD.PLUG" => (Glyph::Plug, String::new()),
            "WELD.SURF" => {
                let thickness = match weld.get("thickness") { None => 2.0, value => number("weld thickness", value, evaluate)? };
                (Glyph::Surfacing, fmt(thickness))
            }
            _ => return Ok(None),
        };
        // A spot weld's text: the pitch with the spot count; an intermittent
        // weld's: length-pitch (AWS), n x l (e) (ISO, n the segment count).
        let length_text = match glyph {
            Glyph::Spot => match segments {
                Some(n) if n > 0 => format!("{} ({n})", fmt(pitch)),
                _ => fmt(pitch),
            },
            _ if length <= 0.0 => String::new(),
            _ => match (standard, pitch > 0.0, segments) {
                (_, false, Some(n)) if tail_kind.is_some() => format!("{} ({n})", fmt(length)),
                (_, false, _) => fmt(length),
                (Standard::Aws, true, _) => format!("{}-{}", fmt(length), fmt(pitch)),
                (Standard::Iso, true, Some(n)) => format!("{n} x {} ({})", fmt(length), fmt((pitch - length).max(0.0))),
                (Standard::Iso, true, None) => format!("{} ({})", fmt(length), fmt((pitch - length).max(0.0))),
            },
        };
        if matches!(glyph, Glyph::Spot | Glyph::Seam | Glyph::Plug | Glyph::Surfacing) && side == Side::Both {
            side = Side::Arrow;
        }
        all_around = all_around && !matches!(glyph, Glyph::Spot);
        let mut tail: Vec<&str> = Vec::new();
        if let Some(kind) = tail_kind { tail.push(kind); }
        for note in [text(weld.get("process")).trim(), text(annotation.get("afterText")).trim()] {
            if !note.is_empty() {
                tail.push(note);
            }
        }
        Ok(Some(WeldSymbol {
            standard,
            side,
            glyph,
            size,
            length: length_text,
            all_around,
            field_weld: annotation.get("fieldWeld").and_then(Value::as_bool).unwrap_or(false),
            tail: tail.join(" "),
            staggered: staggered && side == Side::Both,
        }))
    }
}

/// The view plane the symbol is drawn in: screen right and screen up.
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub right: [f64; 3],
    pub up: [f64; 3],
    /// PMI text cap height, world units.
    pub height: f64,
    /// Arrowhead length, world units.
    pub arrow: f64,
}

impl Frame {
    /// In a picked annotation plane: its in-plane text direction and the up
    /// direction `normal × x_axis`, as planar PMI is laid out.
    pub fn from_plane(normal: [f64; 3], x_axis: [f64; 3], height: f64, arrow: f64) -> Self {
        let right = normalize(x_axis);
        Frame { right, up: normalize(cross(normal, right)), height, arrow }
    }

    /// From the camera's forward and up (`forward × up` is screen right).
    pub fn from_view(forward: [f64; 3], up: [f64; 3], height: f64, arrow: f64) -> Self {
        let right = normalize(cross(forward, up));
        let up = normalize(cross(right, forward));
        Frame { right, up, height, arrow }
    }
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn scale(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn normalize(a: [f64; 3]) -> [f64; 3] {
    let length = dot(a, a).sqrt();
    if length > 1e-12 { scale(a, 1.0 / length) } else { [1.0, 0.0, 0.0] }
}

/// Where the elbow sits when the user has not dragged the label: up and to the
/// right of the seam point on screen, far enough for the arrow to read.
pub fn default_elbow(anchor: [f64; 3], frame: &Frame) -> [f64; 3] {
    add(anchor, add(scale(frame.right, 3.0 * frame.height), scale(frame.up, 4.0 * frame.height)))
}

/// The all-around ring's radius in text heights: it rings the label chip at the
/// elbow (whose corners reach about 1.7 text heights), and the symbol, its texts
/// and the field-weld flag stand clear of it.
pub const RING: f64 = 2.2;

/// Approximate width of a run in the PMI stroke font (used only to place texts
/// beside the symbol; the strokes themselves come from the PMI font).
pub fn text_width(text: &str, height: f64) -> f64 {
    text.chars().count() as f64 * height * 0.8
}

/// A row's weld symbol laid out for `style`: from the leader's seam point to
/// the row's label (the elbow), in the row's picked plane or the view plane.
/// `None` for a row with no symbol (another type, the text style, an edge or
/// corner weld), which is laid out by [`super::layout::present`].
pub fn present_row(row: &super::PmiAnnotationReport, style: &LayoutStyle) -> Option<Presentation> {
    let symbol = row.weld_symbol.as_ref()?;
    let PmiGeometry::Leader { targets, .. } = &row.geometry else { return None };
    let anchor = *targets.first()?;
    let plane = row.plane.or(style.plane);
    let frame = match plane {
        Some(plane) => Frame::from_plane(plane.normal, plane.x_axis, style.text_height, style.arrow),
        None => Frame::from_view(style.view_dir, style.view_up, style.text_height, style.arrow),
    };
    let elbow = plane.map_or(row.label_world, |plane| plane.project(row.label_world));
    Some(present(symbol, anchor, elbow, &frame))
}

/// Lay out the symbol: the arrow from `elbow` to the seam point `anchor`, the
/// reference line from the elbow away from the arrow, and everything on it.
pub fn present(symbol: &WeldSymbol, anchor: [f64; 3], elbow: [f64; 3], frame: &Frame) -> Presentation {
    let h = frame.height;
    let (u, v) = (frame.right, frame.up);
    // The reference line runs away from the arrow along screen right or left.
    let s = if dot(sub(elbow, anchor), u) >= 0.0 { 1.0 } else { -1.0 };
    let at = |x: f64, y: f64| add(elbow, add(scale(u, s * x), scale(v, y)));
    let size = symbol.size.clone();
    let length = symbol.length.clone();
    let tail = symbol.tail.clone();
    let label_room = text_width(&size, h) + text_width(&length, h) + 4.0 * h;
    // The symbol stands clear of the label chip at the elbow and of the
    // all-around ring; the text on the elbow side of the symbol (the size
    // when the line runs right, the length when it runs left) needs its room.
    let near_text = if s > 0.0 { text_width(&size, h) } else { text_width(&length, h) };
    let centre = RING * h + 0.9 * h + near_text + 0.5 * h;
    let line_length = (centre / h + 4.5).max(label_room / h + 2.0 + RING) * h;
    let mut drawn = Presentation::default();
    let mut texts: Vec<TextRun> = Vec::new();

    // Arrow from the elbow to the seam, head at the seam.
    drawn.polylines.push(vec![elbow, anchor]);
    let to_anchor = sub(anchor, elbow);
    let distance = dot(to_anchor, to_anchor).sqrt();
    if distance > 1e-9 {
        let d = scale(to_anchor, 1.0 / distance);
        let head = frame.arrow.min(distance * 0.5);
        let side = normalize(cross(cross(u, v), d));
        let base = sub(anchor, scale(d, head));
        drawn.arrows.push([anchor, add(base, scale(side, head * 0.3)), sub(base, scale(side, head * 0.3))]);
    }

    // Reference line.
    drawn.polylines.push(vec![at(0.0, 0.0), at(line_length, 0.0)]);

    // ISO identification line: dashed, below the solid line, unless symmetric.
    let iso = symbol.standard == Standard::Iso;
    let dashed_y = -0.6 * h;
    if iso && symbol.side != Side::Both {
        let (dash, gap) = (0.5 * h, 0.25 * h);
        let mut x = 0.0;
        while x < line_length {
            let end = (x + dash).min(line_length);
            drawn.polylines.push(vec![at(x, dashed_y), at(end, dashed_y)]);
            x += dash + gap;
        }
    }

    // The basic symbol in an h-wide box. `left` is its left edge in screen
    // terms: a perpendicular leg is always on the left, whichever way the
    // line runs. `sx(k)` is k box-widths right of it on screen.
    let left = if s > 0.0 { centre - 0.5 * h } else { centre + 0.5 * h };
    let screen_x = |k: f64| if s > 0.0 { left + k * h } else { left - k * h };
    // `dx` moves the whole symbol along the line (box widths).
    let mut glyph = |base_y: f64, toward: f64, dx: f64| {
        let sx = |k: f64| screen_x(k + dx);
        let y = |k: f64| base_y + toward * k * h;
        let pt = |kx: f64, ky: f64| at(sx(kx), y(ky));
        // An arc about (cx, cy) of radius r (box units), from a0 to a1
        // radians, measured in screen terms (x right, y toward the symbol).
        let arc = |cx: f64, cy: f64, r: f64, a0: f64, a1: f64| -> Vec<[f64; 3]> {
            (0..=16).map(|k| {
                let a = a0 + (a1 - a0) * k as f64 / 16.0;
                pt(cx + r * a.cos(), cy + r * a.sin())
            }).collect()
        };
        match symbol.glyph {
            Glyph::Fillet => drawn.polylines.push(vec![pt(0.0, 0.0), pt(0.0, 1.0), pt(1.0, 0.0), pt(0.0, 0.0)]),
            Glyph::Square => {
                drawn.polylines.push(vec![pt(0.3, 0.0), pt(0.3, 1.0)]);
                drawn.polylines.push(vec![pt(0.7, 0.0), pt(0.7, 1.0)]);
            }
            Glyph::V => drawn.polylines.push(vec![pt(0.0, 1.0), pt(0.5, 0.0), pt(1.0, 1.0)]),
            Glyph::Bevel => {
                drawn.polylines.push(vec![pt(0.2, 0.0), pt(0.2, 1.0)]);
                drawn.polylines.push(vec![pt(0.2, 0.0), pt(0.9, 1.0)]);
            }
            Glyph::U => {
                let mut line = vec![pt(0.0, 1.0), pt(0.25, 0.55)];
                line.extend(arc(0.5, 0.55, 0.25, std::f64::consts::PI, 2.0 * std::f64::consts::PI).into_iter().skip(1));
                line.push(pt(1.0, 1.0));
                drawn.polylines.push(vec![pt(0.5, 0.0), pt(0.5, 0.3)]);
                drawn.polylines.push(line);
            }
            Glyph::J => {
                drawn.polylines.push(vec![pt(0.2, 0.0), pt(0.2, 1.0)]);
                let mut line = arc(0.2, 0.55, 0.35, -std::f64::consts::FRAC_PI_2, 0.0);
                line.push(pt(0.9, 1.0));
                drawn.polylines.push(line);
            }
            Glyph::Plug => drawn.polylines.push(vec![pt(0.0, 0.0), pt(0.0, 0.6), pt(1.0, 0.6), pt(1.0, 0.0)]),
            Glyph::Spot => drawn.polylines.push(arc(0.5, 0.4, 0.4, 0.0, std::f64::consts::TAU)),
            Glyph::Seam => {
                drawn.polylines.push(arc(0.5, 0.4, 0.4, 0.0, std::f64::consts::TAU));
                drawn.polylines.push(vec![pt(-0.1, 0.3), pt(1.1, 0.3)]);
                drawn.polylines.push(vec![pt(-0.1, 0.5), pt(1.1, 0.5)]);
            }
            Glyph::Surfacing => {
                drawn.polylines.push(arc(0.25, 0.0, 0.25, 0.0, std::f64::consts::PI));
                drawn.polylines.push(arc(0.75, 0.0, 0.25, 0.0, std::f64::consts::PI));
            }
        }
        // Size to the screen-left of the symbol, length to the screen-right,
        // a little clear of the line they stand on.
        let text_y = base_y + toward * 0.7 * h;
        let (lx, rx) = (sx(0.0), sx(1.0));
        if !size.is_empty() {
            let offset = 0.3 * h + text_width(&size, h) * 0.5;
            let x = if s > 0.0 { lx - offset } else { lx + offset };
            texts.push(run(&size, at(x, text_y), u, v, h));
        }
        if !length.is_empty() {
            let offset = 0.3 * h + text_width(&length, h) * 0.5;
            let x = if s > 0.0 { rx + offset } else { rx - offset };
            texts.push(run(&length, at(x, text_y), u, v, h));
        }
    };
    // AWS: arrow side below the line, other side above. ISO: arrow side on the
    // solid line (above it), other side on the dashed line (below it).
    let (arrow_side, other_side) = if iso { ((0.0, 1.0), (dashed_y, -1.0)) } else { ((0.0, -1.0), (0.0, 1.0)) };
    match symbol.side {
        Side::Arrow => glyph(arrow_side.0, arrow_side.1, 0.0),
        Side::Other => glyph(other_side.0, other_side.1, 0.0),
        Side::Both => {
            glyph(0.0, -1.0, 0.0);
            // A staggered weld's other side stands a symbol along (AWS A2.4).
            glyph(0.0, 1.0, if symbol.staggered { 1.0 } else { 0.0 });
        }
    }

    // All around: a circle at the elbow, ringing the label chip there so the
    // chip never hides it.
    if symbol.all_around {
        let r = RING * h;
        drawn.polylines.push((0..=32).map(|k| {
            let a = k as f64 / 32.0 * std::f64::consts::TAU;
            add(elbow, add(scale(u, r * a.cos()), scale(v, r * a.sin())))
        }).collect());
    }
    // Field weld: a staff up from the elbow with a pennant toward the tail,
    // tall enough to clear the label chip at the elbow.
    if symbol.field_weld {
        let top = at(0.0, (RING + 1.0) * h);
        drawn.polylines.push(vec![elbow, top]);
        drawn.arrows.push([top, at(0.0, (RING + 0.4) * h), at(0.9 * h, (RING + 0.7) * h)]);
    }
    // Tail: a fork at the far end, with the process text beyond it.
    if !tail.is_empty() {
        let end = at(line_length, 0.0);
        let k = 0.8 * h * std::f64::consts::FRAC_1_SQRT_2;
        drawn.polylines.push(vec![at(line_length + k, k), end, at(line_length + k, -k)]);
        let x = line_length + k + 0.3 * h + text_width(&tail, h) * 0.5;
        texts.push(run(&tail, at(x, 0.0), u, v, h));
    }

    drawn.text_anchor = elbow;
    drawn.texts = texts;
    drawn
}

fn run(text: &str, anchor: [f64; 3], right: [f64; 3], up: [f64; 3], height: f64) -> TextRun {
    TextRun { text: text.to_string(), anchor, right, up, height }
}

