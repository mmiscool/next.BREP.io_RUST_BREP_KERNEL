//! Weld symbols for the `weldCallout` PMI type: AWS A2.4 by default, ISO 2553
//! as an option, the kernel's text leader as the fallback (`style: TEXT`).
//!
//! The kernel resolves the callout (its seam anchor, the weld's size and process)
//! into a text leader. The symbol is drawn here from that anchor, the label
//! position (the elbow) and the weld feature's own parameters, in the view plane,
//! as the same primitives every other PMI type is drawn with (polylines, filled
//! triangles, text runs). The 3D overlay draws it now; the 3D PDF, the
//! instructions player and STEP read the kernel's presentation, so they show
//! the symbol once the kernel lays it out (until then, the text leader).
//!
//! The symbol's own fields (`style`, `standard`, `side`, `length`, `pitch`,
//! `allAround`, `fieldWeld`) extend the kernel's `weldCallout` schema through
//! [`pmi_schema_catalogue`] until the kernel schema carries them; a field the
//! kernel already defines is never overridden.
use brep_kernel::feature_pipeline::pmi::layout::TextRun;
use brep_kernel::PmiPresentation;
use serde_json::{json, Value};

/// The PMI type this module draws.
pub const WELD_CALLOUT: &str = "weldCallout";

/// The symbol fields, in form order, as kernel-style schema entries.
fn symbol_fields() -> Vec<(&'static str, Value)> {
    vec![
        ("style", json!({"type": "options", "label": "Style", "options": ["SYMBOL", "TEXT"], "default_value": "SYMBOL",
            "hint": "SYMBOL draws the standard weld symbol; TEXT keeps the plain text leader."})),
        ("standard", json!({"type": "options", "label": "Standard", "options": ["AWS", "ISO"], "default_value": "AWS",
            "hint": "AWS A2.4 (fillet below the reference line for the arrow side) or ISO 2553 (solid and dashed reference lines, z leg size)."})),
        ("side", json!({"type": "options", "label": "Weld side", "options": ["ARROW", "OTHER", "BOTH"], "default_value": "ARROW",
            "hint": "Which side of the joint the weld is on, relative to the arrow."})),
        ("length", json!({"type": "number", "label": "Length", "default_value": 0,
            "hint": "Weld length (each segment of an intermittent weld). 0 = continuous; no length is shown."})),
        ("pitch", json!({"type": "number", "label": "Pitch", "default_value": 0,
            "hint": "Centre-to-centre spacing of an intermittent weld. 0 = not intermittent."})),
        ("allAround", json!({"type": "boolean", "label": "All around", "default_value": false,
            "hint": "Weld all around the joint (circle at the elbow)."})),
        ("fieldWeld", json!({"type": "boolean", "label": "Field weld", "default_value": false,
            "hint": "Weld made in the field (flag at the elbow)."})),
    ]
}

/// The kernel's PMI schema catalogue with the weld symbol fields added to the
/// `weldCallout` entry (after its existing fields; the kernel's own wins).
pub fn pmi_schema_catalogue() -> Value {
    let mut catalogue = brep_kernel::pmi_schema_catalogue();
    if let Some(entries) = catalogue.as_array_mut() {
        for entry in entries.iter_mut().filter(|e| e.get("type").and_then(Value::as_str) == Some(WELD_CALLOUT)) {
            if let Some(fields) = entry.get_mut("inputParamsSchema").and_then(Value::as_object_mut) {
                for (key, spec) in symbol_fields() {
                    fields.entry(key.to_string()).or_insert(spec);
                }
            }
        }
    }
    catalogue
}

/// Defaults of the symbol fields for a new `weldCallout`, for keys absent from `params`.
pub fn seed_defaults(type_id: &str, params: &mut serde_json::Map<String, Value>) {
    if type_id != WELD_CALLOUT {
        return;
    }
    for (key, spec) in symbol_fields() {
        if let Some(default) = spec.get("default_value") {
            params.entry(key.to_string()).or_insert_with(|| default.clone());
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Standard {
    Aws,
    Iso,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Side {
    Arrow,
    Other,
    Both,
}

/// What one weld symbol says.
#[derive(Debug, Clone, PartialEq)]
pub struct WeldSymbol {
    pub standard: Standard,
    pub side: Side,
    /// Leg size(s) as numbers; the second only when the legs differ.
    pub legs: (f64, Option<f64>),
    pub length: f64,
    pub pitch: f64,
    pub all_around: bool,
    pub field_weld: bool,
    /// Process / specification and the annotation's extra note, tail text.
    pub tail: String,
}

/// A numeric parameter: a number, or an expression the caller's evaluator reads
/// (the document's Expressions, as the kernel evaluates feature parameters).
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

impl WeldSymbol {
    /// The symbol an annotation asks for, or `None` for the text-leader style.
    /// `weld` is the weld feature's `inputParams`; expressions in it and in the
    /// annotation's numbers are read through `evaluate`.
    /// `Ok(None)` for the text-leader style; `Err` names a number that does not
    /// evaluate.
    pub fn from_params(annotation: &Value, weld: &Value, evaluate: Evaluate<'_>) -> Result<Option<Self>, String> {
        if text(annotation.get("style")) == "TEXT" {
            return Ok(None);
        }
        let standard = if text(annotation.get("standard")) == "ISO" { Standard::Iso } else { Standard::Aws };
        let side = match text(annotation.get("side")) {
            "OTHER" => Side::Other,
            "BOTH" => Side::Both,
            _ => Side::Arrow,
        };
        let first = match weld.get("legSize") {
            None => 3.0,
            value => number("weld leg size", value, evaluate)?,
        };
        let second = number("weld second leg", weld.get("legSize2"), evaluate)?;
        let second = (second > 0.0 && (second - first).abs() > 1e-12).then_some(second);
        let mut tail: Vec<&str> = Vec::new();
        for note in [text(weld.get("process")).trim(), text(annotation.get("afterText")).trim()] {
            if !note.is_empty() {
                tail.push(note);
            }
        }
        Ok(Some(WeldSymbol {
            standard,
            side,
            legs: (first, second),
            length: number("length", annotation.get("length"), evaluate)?.max(0.0),
            pitch: number("pitch", annotation.get("pitch"), evaluate)?.max(0.0),
            all_around: annotation.get("allAround").and_then(Value::as_bool).unwrap_or(false),
            field_weld: annotation.get("fieldWeld").and_then(Value::as_bool).unwrap_or(false),
            tail: tail.join(" "),
        }))
    }

    /// The size text: `6`, `6x8`; ISO prefixes the leg length with `z`.
    pub fn size_text(&self) -> String {
        let base = match self.legs.1 {
            Some(second) => format!("{}x{}", fmt(self.legs.0), fmt(second)),
            None => fmt(self.legs.0),
        };
        match self.standard {
            Standard::Aws => base,
            Standard::Iso => format!("z{base}"),
        }
    }

    /// The length / pitch text: AWS `50-150`, ISO `50 (100)` (length and the gap
    /// between segments); empty for a continuous weld.
    pub fn length_text(&self) -> String {
        if self.length <= 0.0 {
            return String::new();
        }
        match (self.standard, self.pitch > 0.0) {
            (_, false) => fmt(self.length),
            (Standard::Aws, true) => format!("{}-{}", fmt(self.length), fmt(self.pitch)),
            (Standard::Iso, true) => format!("{} ({})", fmt(self.length), fmt((self.pitch - self.length).max(0.0))),
        }
    }
}

/// A dimension number without a trailing `.0`, at most two decimals.
fn fmt(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    let text = format!("{rounded:.2}");
    text.trim_end_matches('0').trim_end_matches('.').to_string()
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
    /// direction `normal × x_axis`, as the kernel lays planar PMI out.
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
const RING: f64 = 2.2;

/// Approximate width of a run in the PMI stroke font (used only to place texts
/// beside the symbol; the strokes themselves come from the kernel font).
fn text_width(text: &str, height: f64) -> f64 {
    text.chars().count() as f64 * height * 0.8
}

/// Lay out the symbol: the arrow from `elbow` to the seam point `anchor`, the
/// reference line from the elbow away from the arrow, and everything on it.
pub fn present(symbol: &WeldSymbol, anchor: [f64; 3], elbow: [f64; 3], frame: &Frame) -> PmiPresentation {
    let h = frame.height;
    let (u, v) = (frame.right, frame.up);
    // The reference line runs away from the arrow along screen right or left.
    let s = if dot(sub(elbow, anchor), u) >= 0.0 { 1.0 } else { -1.0 };
    let at = |x: f64, y: f64| add(elbow, add(scale(u, s * x), scale(v, y)));
    // Screen-space point (x to the right, unaffected by the line's direction).
    let size = symbol.size_text();
    let length = symbol.length_text();
    let tail = symbol.tail.clone();
    let label_room = text_width(&size, h) + text_width(&length, h) + 4.0 * h;
    // The symbol stands clear of the label chip at the elbow and of the
    // all-around ring.
    // The text on the elbow side of the triangle (the size when the line runs
    // right, the length when it runs left) also needs its room.
    let near_text = if s > 0.0 { text_width(&size, h) } else { text_width(&length, h) };
    let centre = RING * h + 0.9 * h + near_text + 0.5 * h;
    let line_length = (centre / h + 4.5).max(label_room / h + 2.0 + RING) * h;
    let mut drawn = PmiPresentation::default();
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

    // Fillet triangles. `x0` is the symbol's left edge in screen terms: the
    // vertical leg is always on the left, whichever way the line runs.
    let screen_left = |c: f64| if s > 0.0 { c - 0.5 * h } else { c + 0.5 * h };
    let left = screen_left(centre);
    let mut triangle = |base_y: f64, toward: f64| {
        // In screen x: left edge, then h to the right.
        let lx = left;
        let rx = if s > 0.0 { left + h } else { left - h };
        drawn.polylines.push(vec![at(lx, base_y), at(lx, base_y + toward * h), at(rx, base_y), at(lx, base_y)]);
        // Size to the screen-left of the symbol, length to the screen-right,
        // a little clear of the line they stand on.
        let y = base_y + toward * 0.7 * h;
        if !size.is_empty() {
            let offset = 0.3 * h + text_width(&size, h) * 0.5;
            let x = if s > 0.0 { lx - offset } else { lx + offset };
            texts.push(run(&size, at(x, y), u, v, h));
        }
        if !length.is_empty() {
            let offset = 0.3 * h + text_width(&length, h) * 0.5;
            let x = if s > 0.0 { rx + offset } else { rx - offset };
            texts.push(run(&length, at(x, y), u, v, h));
        }
    };
    // AWS: arrow side below the line, other side above. ISO: arrow side on the
    // solid line (above it), other side on the dashed line (below it).
    let (arrow_side, other_side) = if iso { ((0.0, 1.0), (dashed_y, -1.0)) } else { ((0.0, -1.0), (0.0, 1.0)) };
    match symbol.side {
        Side::Arrow => triangle(arrow_side.0, arrow_side.1),
        Side::Other => triangle(other_side.0, other_side.1),
        Side::Both => {
            triangle(0.0, -1.0);
            triangle(0.0, 1.0);
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

    // The texts are runs, as for every PMI type: the 3D PDF and STEP stroke
    // them, the viewport strokes them for a symbol (it has no chip text).
    drawn.text_anchor = elbow;
    drawn.texts = texts;
    drawn
}

fn run(text: &str, anchor: [f64; 3], right: [f64; 3], up: [f64; 3], height: f64) -> TextRun {
    TextRun { text: text.to_string(), anchor, right, up, height }
}

