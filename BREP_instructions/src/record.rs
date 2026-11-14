//! The printed pages' Record block: a step's fields as a paper form.
//!
//! What the players show as inputs, the paper shows as places to write: an
//! empty box to tick, a choice's options each as its own box to mark (a
//! dropdown cannot be opened on paper), ruled lines for text, a ruled line
//! with its unit and limits for a measurement, and a ruled line per serial
//! number — the step's own fields and one line per unit of each
//! serial-tracked part it adds. A step that needs a sign-off ends with a
//! Signed and a Date line.
//!
//! The block is a list of [`Item`]s, each drawn as one or more [`Band`]s:
//! full-width strips stacked down the page, the unit the page layout flows
//! (a band is never split; an item may continue on the next page between
//! two bands). A band is made of [`Mark`]s in millimetres from its own
//! top-left corner: text runs and strokes. Boxes and rules are strokes the
//! sheet writers draw as vector lines, never characters of a font.
//!
//! Everything here is measured through a [`TextMeasure`], so the layout that
//! pages the bands and the renderer that draws them agree to the millimetre.

use crate::fields::{FieldKind, StepField};
use crate::layout::{wrap, TextMeasure, BODY_MM, LEADING};
use crate::model::Step;
use serde::Serialize;

/// Small print's cap height: a hint, a measurement's limits, "(mark one)".
pub const SMALL_MM: f64 = 2.2;
/// A box to tick: its side.
pub const BOX_MM: f64 = 3.4;
/// The pitch of a line to write on by hand.
pub const WRITE_MM: f64 = 8.0;
/// The shortest ruled line worth writing on.
const MIN_RULE_MM: f64 = 30.0;
/// A measurement's ruled line.
const NUMBER_RULE_MM: f64 = 26.0;

/// One piece of a step's Record block.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(tag = "item", rename_all = "camelCase")]
pub enum Item {
    /// The block's heading.
    Heading,
    /// The serial of one unit of a serial-tracked part the step adds: the
    /// add's index and the unit's (its occurrence).
    Serial { add: usize, unit: usize },
    /// One of the step's fields, by index.
    Field { index: usize },
    /// The Signed and Date lines of a step that needs a sign-off.
    Signoff,
}

/// One mark of a band, in millimetres from the band's top-left corner.
#[derive(Debug, Clone, PartialEq)]
pub enum Mark {
    /// A run of text whose line box starts at `(x, y)`, `cap` its cap height.
    Text { text: String, x: f64, y: f64, cap: f64 },
    /// An empty square to tick or mark, its top-left corner at `(x, y)`.
    Box { x: f64, y: f64, size: f64 },
    /// A line to write on, from `x0` to `x1` at height `y`.
    Rule { x0: f64, x1: f64, y: f64 },
    /// Any other stroke: a write-in box's sides and top, the heading's rule.
    Line(Vec<[f64; 2]>),
}

/// One full-width strip of the block.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Band {
    pub h: f64,
    pub marks: Vec<Mark>,
}

impl Band {
    fn new(h: f64) -> Self {
        Band { h, marks: Vec::new() }
    }

    fn text(mut self, text: impl Into<String>, x: f64, y: f64, cap: f64) -> Self {
        self.marks.push(Mark::Text { text: text.into(), x, y, cap });
        self
    }

    /// How many boxes to tick the band draws.
    pub fn boxes(&self) -> usize {
        self.marks.iter().filter(|m| matches!(m, Mark::Box { .. })).count()
    }

    /// How many lines to write on the band draws.
    pub fn rules(&self) -> usize {
        self.marks.iter().filter(|m| matches!(m, Mark::Rule { .. })).count()
    }

    /// The band's text runs, in drawing order.
    pub fn texts(&self) -> Vec<&str> {
        self.marks.iter().filter_map(|m| if let Mark::Text { text, .. } = m { Some(text.as_str()) } else { None }).collect()
    }
}

/// The Record block of `step`, in print order: the heading, a serial line per
/// unit of each serial-tracked add, the fields, then the sign-off lines.
/// Empty for a step that records nothing and needs no sign-off; a step that
/// only needs a sign-off gets its Signed line alone, without a heading.
pub fn items(step: &Step) -> Vec<Item> {
    let mut out = Vec::new();
    for (add, part) in step.adds.iter().enumerate() {
        if part.serial_tracked {
            out.extend((0..part.occurrences.len()).map(|unit| Item::Serial { add, unit }));
        }
    }
    out.extend((0..step.fields.len()).map(|index| Item::Field { index }));
    if !out.is_empty() {
        out.insert(0, Item::Heading);
    }
    if step.signoff.required {
        out.push(Item::Signoff);
    }
    out
}

const PITCH: f64 = BODY_MM * LEADING;

/// A label over (or before) one line to write on: on one band when the
/// label leaves room for a line worth writing on, else the label's lines and
/// then a full-width line.
fn write_line(label: &str, width: f64, m: &dyn TextMeasure) -> Vec<Band> {
    let w = m.width_mm(label, BODY_MM);
    let baseline = WRITE_MM - 1.2;
    if w + 3.0 + MIN_RULE_MM <= width {
        let mut band = Band::new(WRITE_MM).text(label, 0.0, baseline - BODY_MM - 0.6, BODY_MM);
        band.marks.push(Mark::Rule { x0: w + 3.0, x1: width, y: baseline });
        return vec![band];
    }
    let mut bands: Vec<Band> = wrap(label, width, BODY_MM, m).into_iter().map(|line| Band::new(PITCH).text(line, 0.0, (PITCH - BODY_MM) / 2.0, BODY_MM)).collect();
    let mut rule = Band::new(WRITE_MM);
    rule.marks.push(Mark::Rule { x0: 0.0, x1: width, y: baseline });
    bands.push(rule);
    bands
}

fn label_bands(text: &str, width: f64, m: &dyn TextMeasure) -> Vec<Band> {
    wrap(text, width, BODY_MM, m).into_iter().map(|line| Band::new(PITCH).text(line, 0.0, (PITCH - BODY_MM) / 2.0, BODY_MM)).collect()
}

fn field_bands(field: &StepField, width: f64, m: &dyn TextMeasure) -> Vec<Band> {
    let star = if field.required { " *" } else { "" };
    let name = if field.label.trim().is_empty() { "Field" } else { field.label.trim() };
    let label = format!("{name}{star}");
    let mut bands = match &field.kind {
        FieldKind::Check => {
            let h = PITCH.max(BOX_MM + 1.6);
            wrap(&label, width - BOX_MM - 2.0, BODY_MM, m)
                .into_iter()
                .enumerate()
                .map(|(i, line)| {
                    let mut band = Band::new(h).text(line, BOX_MM + 2.0, (h - BODY_MM) / 2.0, BODY_MM);
                    if i == 0 {
                        band.marks.insert(0, Mark::Box { x: 0.0, y: (h - BOX_MM) / 2.0, size: BOX_MM });
                    }
                    band
                })
                .collect()
        }
        FieldKind::Choice { multiple, other, .. } => {
            let how = if *multiple { "(mark all that apply)" } else { "(mark one)" };
            let mut bands = label_bands(&format!("{label} {how}"), width, m);
            // Every option is its own box, flowing left to right and
            // wrapping; an option too long for a line wraps under its box.
            let h = PITCH.max(BOX_MM + 2.4);
            let (gap, after) = (1.5, 5.0);
            let mut row = Band::new(h);
            let mut x = 0.0;
            let flush = |row: &mut Band, x: &mut f64, bands: &mut Vec<Band>| {
                if !row.marks.is_empty() {
                    bands.push(std::mem::replace(row, Band::new(h)));
                }
                *x = 0.0;
            };
            for option in field.options() {
                let w = m.width_mm(option, BODY_MM);
                if BOX_MM + gap + w > width {
                    flush(&mut row, &mut x, &mut bands);
                    for (i, line) in wrap(option, width - BOX_MM - gap, BODY_MM, m).into_iter().enumerate() {
                        let mut band = Band::new(h).text(line, BOX_MM + gap, (h - BODY_MM) / 2.0, BODY_MM);
                        if i == 0 {
                            band.marks.insert(0, Mark::Box { x: 0.0, y: (h - BOX_MM) / 2.0, size: BOX_MM });
                        }
                        bands.push(band);
                    }
                    continue;
                }
                if x > 0.0 && x + BOX_MM + gap + w > width {
                    flush(&mut row, &mut x, &mut bands);
                }
                row.marks.push(Mark::Box { x, y: (h - BOX_MM) / 2.0, size: BOX_MM });
                row.marks.push(Mark::Text { text: option.to_string(), x: x + BOX_MM + gap, y: (h - BODY_MM) / 2.0, cap: BODY_MM });
                x += BOX_MM + gap + w + after;
            }
            if *other {
                let text = "Other:";
                let w = m.width_mm(text, BODY_MM);
                if x > 0.0 && x + BOX_MM + gap + w + 2.0 + MIN_RULE_MM > width {
                    flush(&mut row, &mut x, &mut bands);
                }
                row.marks.push(Mark::Box { x, y: (h - BOX_MM) / 2.0, size: BOX_MM });
                row.marks.push(Mark::Text { text: text.into(), x: x + BOX_MM + gap, y: (h - BODY_MM) / 2.0, cap: BODY_MM });
                row.marks.push(Mark::Rule { x0: x + BOX_MM + gap + w + 2.0, x1: width, y: h - 1.0 });
            }
            flush(&mut row, &mut x, &mut bands);
            bands
        }
        FieldKind::Text { lines } if *lines > 1 => {
            // A ruled box: `lines` lines to write on, framed.
            let mut bands = label_bands(&label, width, m);
            for i in 0..*lines {
                let mut band = Band::new(WRITE_MM);
                if i == 0 {
                    band.marks.push(Mark::Line(vec![[0.0, 0.0], [width, 0.0]]));
                }
                band.marks.push(Mark::Line(vec![[0.0, 0.0], [0.0, WRITE_MM]]));
                band.marks.push(Mark::Line(vec![[width, 0.0], [width, WRITE_MM]]));
                band.marks.push(Mark::Rule { x0: 0.0, x1: width, y: WRITE_MM });
                bands.push(band);
            }
            bands
        }
        FieldKind::Text { .. } | FieldKind::Serial => write_line(&label, width, m),
        FieldKind::Number { .. } => {
            // Label, a line for the value, the unit, and the nominal and
            // limits in small print (on a line of their own when the first
            // is full).
            let unit = field.unit();
            let limits = field.limits_text();
            let label_w = m.width_mm(&label, BODY_MM);
            let unit_w = if unit.is_empty() { 0.0 } else { m.width_mm(unit, BODY_MM) + 1.5 };
            let baseline = WRITE_MM - 1.2;
            let text_y = baseline - BODY_MM - 0.6;
            let mut bands = Vec::new();
            let mut band = Band::new(WRITE_MM);
            let mut x = 0.0;
            if label_w + 3.0 + NUMBER_RULE_MM + unit_w <= width {
                band = band.text(label.as_str(), 0.0, text_y, BODY_MM);
                x = label_w + 3.0;
            } else {
                bands.extend(label_bands(&label, width, m));
            }
            let rule_end = (x + NUMBER_RULE_MM).min(width - unit_w).max(x);
            band.marks.push(Mark::Rule { x0: x, x1: rule_end, y: baseline });
            x = rule_end;
            if !unit.is_empty() {
                band = band.text(unit, x + 1.5, text_y, BODY_MM);
                x += unit_w;
            }
            if !limits.is_empty() {
                let small = format!("({limits})");
                let small_w = m.width_mm(&small, SMALL_MM);
                if x + 3.0 + small_w <= width {
                    band = band.text(small, x + 3.0, baseline - SMALL_MM - 0.6, SMALL_MM);
                    bands.push(band);
                } else {
                    bands.push(band);
                    for line in wrap(&small, width, SMALL_MM, m) {
                        bands.push(Band::new(SMALL_MM * LEADING).text(line, 0.0, (SMALL_MM * LEADING - SMALL_MM) / 2.0, SMALL_MM));
                    }
                }
            } else {
                bands.push(band);
            }
            bands
        }
    };
    let hint = field.hint.trim();
    if !hint.is_empty() {
        for line in wrap(hint, width, SMALL_MM, m) {
            bands.push(Band::new(SMALL_MM * LEADING).text(line, 0.0, (SMALL_MM * LEADING - SMALL_MM) / 2.0, SMALL_MM));
        }
    }
    // A little air before the next field.
    if let Some(last) = bands.last_mut() {
        last.h += 1.2;
    }
    bands
}

/// The bands of one item of `step`'s block (its `number` in reading order)
/// in a column `width` wide. `continued`: the heading of a block carried
/// over to a Record page, which names its step.
pub fn bands(step: &Step, number: usize, item: &Item, width: f64, continued: bool, m: &dyn TextMeasure) -> Vec<Band> {
    match item {
        Item::Heading => {
            let h = PITCH + 2.0;
            let text = if continued { format!("Step {number} record (continued)") } else { "Record".to_string() };
            let mut band = Band::new(h).text(text, 0.0, 0.6, BODY_MM);
            band.marks.push(Mark::Line(vec![[0.0, h - 1.4], [width, h - 1.4]]));
            vec![band]
        }
        Item::Serial { add, unit } => {
            let Some(part) = step.adds.get(*add) else { return Vec::new() };
            let which = if part.occurrences.len() > 1 { format!(" #{}", unit + 1) } else { String::new() };
            let mut bands = write_line(&format!("Serial: {}{which} *", part.display_name()), width, m);
            if let Some(last) = bands.last_mut() {
                last.h += 1.2;
            }
            bands
        }
        Item::Field { index } => step.fields.get(*index).map(|field| field_bands(field, width, m)).unwrap_or_default(),
        Item::Signoff => {
            let h = WRITE_MM + 1.5;
            let baseline = h - 1.2;
            let text_y = baseline - BODY_MM - 0.6;
            let signed_w = m.width_mm("Signed", BODY_MM);
            let date_w = m.width_mm("Date", BODY_MM);
            let free = (width - signed_w - date_w - 8.0).max(0.0);
            let signed_end = signed_w + 2.0 + free * 0.62;
            let mut band = Band::new(h).text("Signed", 0.0, text_y, BODY_MM).text("Date", signed_end + 4.0, text_y, BODY_MM);
            band.marks.push(Mark::Rule { x0: signed_w + 2.0, x1: signed_end, y: baseline });
            band.marks.push(Mark::Rule { x0: signed_end + 4.0 + date_w + 2.0, x1: width, y: baseline });
            vec![band]
        }
    }
}

