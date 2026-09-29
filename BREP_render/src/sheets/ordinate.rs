//! Ordinate sets — a DATUM and a run of members, each reading its signed
//! distance from the datum along one axis of the paper.
//!
//! # Why this is not a fifth dimension kind
//!
//! A [`super::dimension::SheetDimension`] measures ONE thing and draws one
//! dimension line for it. An ordinate set measures any number of points
//! against a shared origin and draws one leader per point to a shared
//! baseline, and there is no dimension line at all. Its anchors are not a
//! fixed-length list, its value is not one number, and deleting its datum
//! deletes every value it defines. That is a different object, so it is one:
//! `Sheet::ordinates` beside `Sheet::dimensions`, with its own id prefix
//! (`OD{n}` from the one counter) and its own form.
//!
//! # The drawing
//!
//! The `axis` picks which paper component is measured and therefore which way
//! the leaders run:
//!
//! | axis | value | leader | baseline |
//! |---|---|---|---|
//! | `horizontal` | paper x from the datum, right positive | vertical | a paper y |
//! | `vertical` | paper y from the datum, **UP** positive | horizontal | a paper x |
//!
//! A vertical set negates the paper component on purpose: paper y runs DOWN
//! (SVG's frame) and a drawing's vertical ordinates count UP, so the sign the
//! reader sees is the model's and not the file format's.
//!
//! `offsetMm` is the baseline's signed distance from the DATUM along the
//! leader direction — one number, so the whole set moves with one drag, and
//! its sign picks which side of the datum the values are written on.
//!
//! Every station draws a leader from its anchor to the baseline and its value
//! just past the end, flat on the sheet. The DATUM draws too, reading `0.000`
//! and carrying the small circle that is the ordinate origin's own symbol —
//! without it a reader cannot see which of the values the others are measured
//! from.
//!
//! # What a lost anchor does
//!
//! A lost DATUM takes the set: there is nothing left to measure from, so the
//! row carries the reason and draws nothing ([`OrdinateDrawing::error`]). A
//! lost MEMBER takes only itself — its own station carries the reason
//! ([`OrdinateMember::error`]) and the rest of the set still draws, because a
//! set that vanished when one of five points moved would be worse than one
//! that says which point went. That is the same rule a dimension follows,
//! read at the right granularity.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use super::dimension::{SheetAnchor, HORIZONTAL, VERTICAL};
use super::project::{SheetText, ViewDrawing};
use brep_kernel::{format_dimension, format_number};

/// The axes a set can be measured along — a linear dimension's own two, minus
/// `aligned`: an ordinate run has no arbitrary direction, because the whole
/// point is that every value shares one.
pub const AXES: &[&str] = &[HORIZONTAL, VERTICAL];

/// A new set's baseline offset from its datum, paper millimetres.
pub const DEFAULT_OFFSET_MM: f64 = 16.0;

/// Decimals in an ordinate value, the linear default.
pub const DEFAULT_PRECISION: u32 = 3;

/// The clear space between a leader's end and its value, as a multiple of the
/// cap height.
const VALUE_GAP: f64 = 0.5;

/// The ordinate ORIGIN symbol's radius at the datum, as a multiple of the cap
/// height.
const ORIGIN_RADIUS: f64 = 0.35;

/// Segments the origin circle is drawn with — the sheet's contract is lines,
/// so a circle is a closed polyline exactly as an arrowhead is.
const ORIGIN_STEPS: usize = 24;

/// How many rows a crowded value may be staggered away from the baseline
/// before the run gives up and lets it print where it falls. A set dense
/// enough to need more than this is telling the reader to raise the scale.
const MAX_STAGGER_ROWS: usize = 8;

/// The clear space two values have to leave each other, as a multiple of the
/// cap height: ONE character advance — two numbers with less than a space
/// between them read as one longer number, which is the failure this is here
/// to stop, not merely ink overlapping ink.
const VALUE_PAD: f64 = super::project::TEXT_ADVANCE;

// ===========================================================================
// The block
// ===========================================================================

/// One ordinate set on a sheet.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SheetOrdinate {
    pub id: String,
    /// [`HORIZONTAL`] or [`VERTICAL`].
    #[serde(default = "default_axis")]
    pub axis: String,
    /// The anchor reference every member is measured FROM. Empty is a set
    /// with no datum — what the form leaves behind when its datum row is
    /// cleared, and the engine deletes the set rather than keeping it.
    #[serde(default)]
    pub datum: String,
    /// The measured anchors, in the order they read.
    #[serde(default)]
    pub members: Vec<String>,
    /// The baseline's signed distance from the datum, paper millimetres.
    #[serde(default = "default_offset", rename = "offsetMm")]
    pub offset_mm: f64,
    #[serde(default = "default_precision")]
    pub precision: u32,
    /// The tolerance block every MEMBER's value is written with. The datum
    /// reads a plain `0.000`: it is the origin the others are measured from,
    /// exact by definition.
    #[serde(flatten)]
    pub tolerance: super::dimension::SheetTolerance,
}

fn default_axis() -> String {
    HORIZONTAL.to_string()
}

fn default_offset() -> f64 {
    DEFAULT_OFFSET_MM
}

fn default_precision() -> u32 {
    DEFAULT_PRECISION
}

impl SheetOrdinate {
    pub fn new(id: String, axis: &str, datum: String) -> SheetOrdinate {
        SheetOrdinate {
            id,
            axis: axis.to_string(),
            datum,
            members: Vec::new(),
            offset_mm: DEFAULT_OFFSET_MM,
            precision: DEFAULT_PRECISION,
            tolerance: Default::default(),
        }
    }

    /// The params the **Ordinate set** form edits.
    pub fn params(&self) -> Value {
        let mut params = json!({
            "id": self.id,
            "axis": self.axis,
            "datum": if self.datum.is_empty() { Value::Null } else { Value::String(self.datum.clone()) },
            "members": self.members,
            "offsetMm": self.offset_mm,
            "precision": self.precision,
        });
        self.tolerance.write_params(&mut params);
        params
    }

    /// Apply an edited **Ordinate set** form. `id` is identity and is never
    /// taken from the form.
    pub fn apply(&mut self, params: &Value) -> Result<(), String> {
        if let Some(axis) = params.get("axis").and_then(Value::as_str) {
            let axis = axis.trim();
            let Some(known) = AXES.iter().find(|a| a.eq_ignore_ascii_case(axis)) else {
                return Err(format!("no ordinate axis '{axis}'"));
            };
            self.axis = (*known).to_string();
        }
        // The DATUM is a single-reference field, so the form hands back either
        // a string or a one-element list; both mean the same thing and an
        // empty one means the row was cleared.
        if let Some(datum) = params.get("datum") {
            let picked: Vec<String> = match datum.as_str() {
                Some(one) => vec![one.to_string()],
                None => crate::json_support::string_values(Some(datum)).map(str::to_string).collect(),
            }
            .into_iter()
            .filter(|anchor| !anchor.is_empty())
            .collect();
            if let Some(first) = picked.first() {
                SheetAnchor::parse(first)?;
            }
            self.datum = picked.into_iter().next().unwrap_or_default();
        }
        if let Some(members) = params.get("members") {
            let list: Vec<String> = crate::json_support::string_values(Some(members))
                .map(str::to_string)
                .collect();
            for anchor in &list {
                SheetAnchor::parse(anchor)?;
            }
            self.members = list;
        }
        super::dimension::apply_offset_precision(params, &mut self.offset_mm, &mut self.precision)?;
        self.tolerance.apply(params)
    }
}

/// The **Ordinate set** dialog's schema, in the `inputParamsSchema` vocabulary
/// the shared form engine renders.
///
/// TWO reference fields rather than one list with a magic first row: the datum
/// and the members answer different questions, and `datum`'s ✕ has to mean
/// "this set has no origin any more" while a member's ✕ means "drop that one
/// value". A single list could not tell those apart without counting rows.
pub fn sheet_ordinate_schema() -> Value {
    let mut fields = json!({
        "id": { "type": "string", "default_value": null },
        "axis": { "type": "options", "label": "Axis", "options": AXES, "default_value": HORIZONTAL },
        "datum": { "type": "reference_selection", "selectionFilter": ["sheetAnchor"], "multiple": false },
        "members": { "type": "reference_selection", "selectionFilter": ["sheetAnchor"], "multiple": true },
        "offsetMm": { "type": "number", "label": "Baseline offset (mm)", "default_value": DEFAULT_OFFSET_MM },
        "precision": { "type": "number", "label": "Precision (decimals)", "default_value": DEFAULT_PRECISION },
    });
    // The tolerance block, the PMI dimension form's own four fields.
    super::dimension::SheetTolerance::schema_fields(fields.as_object_mut().expect("an object"));
    json!({
        "type": "sheetOrdinate",
        "shortName": "OD",
        "longName": "Ordinate set",
        "inputParamsSchema": fields,
    })
}

// ===========================================================================
// The drawing
// ===========================================================================

/// One ordinate set, laid out in paper millimetres.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct OrdinateDrawing {
    pub id: String,
    pub axis: String,
    /// Every station's leader, and the datum's origin circle.
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
    /// The datum's own value position — the point the baseline drag grabs, and
    /// where a set with no datum left writes its reason.
    pub label: [f64; 2],
    #[serde(rename = "offsetMm")]
    pub offset_mm: f64,
    /// The datum and every member, in reading order.
    pub stations: Vec<OrdinateMember>,
    /// Empty when the set resolved; why the whole set did not otherwise — a
    /// lost DATUM, a member on another placement, a placement that did not
    /// project. A lost MEMBER is on its own station instead.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub error: String,
}

/// One station of a set — the datum, or one member.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct OrdinateMember {
    #[serde(rename = "ref")]
    pub anchor: String,
    /// Whether this station IS the datum (value 0, and the origin symbol).
    pub datum: bool,
    /// The signed model distance from the datum along the set's axis (`NaN`
    /// when this station did not resolve).
    pub value: f64,
    /// The formatted value, empty when it did not resolve.
    pub text: String,
    /// The anchor's paper point.
    pub at: [f64; 2],
    /// Empty when this station resolved; why it did not otherwise.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub error: String,
}

impl OrdinateDrawing {
    fn refused(set: &SheetOrdinate, label: [f64; 2], error: String) -> OrdinateDrawing {
        OrdinateDrawing {
            id: set.id.clone(),
            axis: set.axis.clone(),
            lines: Vec::new(),
            texts: Vec::new(),
            label,
            offset_mm: set.offset_mm,
            stations: Vec::new(),
            error,
        }
    }

    /// How many stations carry their own reason — what the pane's row and the
    /// viewport's note count.
    pub fn lost(&self) -> usize {
        self.stations.iter().filter(|s| !s.error.is_empty()).count()
    }
}

/// Lay out every ordinate set of a sheet against the placements already
/// projected into `views`. `centre` is where a set with no datum left writes
/// its reason.
pub fn layout_ordinates(
    ordinates: &[SheetOrdinate],
    views: &[ViewDrawing],
    centre: [f64; 2],
) -> Vec<OrdinateDrawing> {
    ordinates
        .iter()
        .map(|set| layout_one(set, views, centre))
        .collect()
}

fn layout_one(
    set: &SheetOrdinate,
    views: &[ViewDrawing],
    centre: [f64; 2],
) -> OrdinateDrawing {
    if set.datum.trim().is_empty() {
        return OrdinateDrawing::refused(
            set,
            centre,
            "an ordinate set has no datum; every value is measured from one".into(),
        );
    }
    let (view, datum) = match super::dimension::resolve_anchor(&set.datum, views) {
        Ok(found) => found,
        Err(error) => return OrdinateDrawing::refused(set, centre, error),
    };
    if view.scale <= 0.0 {
        return OrdinateDrawing::refused(
            set,
            datum.at,
            format!("placement '{}' has no scale", view.id),
        );
    }
    let vertical = set.axis.eq_ignore_ascii_case(VERTICAL);
    // The leader direction — which way a station reaches its baseline — and
    // the level the baseline sits at, both from the ONE offset.
    let sign = if set.offset_mm < 0.0 { -1.0 } else { 1.0 };
    let level = if vertical { datum.at[0] } else { datum.at[1] } + set.offset_mm;
    let cap = super::dimension::text_height(view);

    let mut lines: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut texts: Vec<SheetText> = Vec::new();
    let mut stations: Vec<OrdinateMember> = Vec::new();
    // Every value box already laid, so the next one can be jogged clear of it.
    let mut placed: Vec<[f64; 4]> = Vec::new();
    let mut label = datum.at;

    let references = std::iter::once((&set.datum, true))
        .chain(set.members.iter().map(|member| (member, false)));
    for (reference, is_datum) in references {
        let point = if is_datum {
            Ok(datum)
        } else {
            match super::dimension::resolve_anchor(reference, views) {
                // A member on ANOTHER placement measures against a different
                // projection, which is the linear dimension's refusal read at
                // the station rather than at the set.
                Ok((other, _)) if other.id != view.id => Err(format!(
                    "'{reference}' is on placement '{}', not on the datum's '{}'",
                    other.id, view.id
                )),
                Ok((_, found)) => Ok(found),
                Err(error) => Err(error),
            }
        };
        let found = match point {
            Ok(found) => found,
            Err(error) => {
                stations.push(OrdinateMember {
                    anchor: reference.clone(),
                    datum: is_datum,
                    value: f64::NAN,
                    text: String::new(),
                    at: [f64::NAN, f64::NAN],
                    error,
                });
                continue;
            }
        };
        // The signed MODEL distance along the axis. A vertical set counts UP,
        // so the paper's downward y is negated before it is scaled.
        let value = if vertical {
            (datum.at[1] - found.at[1]) / view.scale
        } else {
            (found.at[0] - datum.at[0]) / view.scale
        };
        // Every member through the kernel's own dimension text, with the set's
        // block; the datum is the origin and reads a plain zero.
        let text = if is_datum {
            format_number(value, set.precision as usize)
        } else {
            let block = set.tolerance.block();
            format_dimension(value, set.precision as usize, &block, set.tolerance.reference, "", "")
        };
        // The leader, from the anchor to the baseline.
        let foot = if vertical {
            [level, found.at[1]]
        } else {
            [found.at[0], level]
        };
        // …and the value just past it, clear of the leader's end. A flat run
        // grows along x, so a VERTICAL set has to clear half a text WIDTH
        // where a horizontal one clears half a cap height.
        let half_width = 0.5 * text.chars().count() as f64 * super::project::TEXT_ADVANCE * cap;
        let clear = cap * VALUE_GAP + if vertical { half_width } else { 0.5 * cap };
        let away = |distance: f64| {
            if vertical {
                [foot[0] + sign * distance, foot[1]]
            } else {
                [foot[0], foot[1] + sign * distance]
            }
        };
        // Two stations closer together than their values are wide would print
        // one number through the other, which is the one way a run of numbers
        // stops being readable — and a pair of stations over ONE paper
        // position (two points differing only ACROSS the measured axis) is not
        // an edge case, it is what an ordinate set is for. A value that lands
        // on one already placed is STAGGERED a row further from the baseline,
        // which is what a drafter does with a crowded run. The leaders do not
        // move: every one of them still ends on the common baseline, and a
        // staggered number reads under the one above it.
        //
        // A row is the value's own extent plus TWICE the pad, so one step
        // always clears — a row of exactly the pad lands the two boxes edge to
        // edge, where the collision test's own arithmetic decides it, and the
        // value is stepped a second time for nothing.
        let row = 2.0 * cap * VALUE_PAD + if vertical { 2.0 * half_width } else { cap };
        let mut anchor = away(clear);
        for step in 1..=MAX_STAGGER_ROWS {
            if !placed
                .iter()
                .any(|other| boxes_collide(other, &value_box(anchor, half_width, cap), cap))
            {
                break;
            }
            anchor = away(clear + step as f64 * row);
        }
        placed.push(value_box(anchor, half_width, cap));
        lines.push(vec![found.at, foot]);
        texts.push(SheetText::flat(text.clone(), anchor, cap));
        if is_datum {
            lines.push(origin_circle(found.at, cap * ORIGIN_RADIUS));
            label = anchor;
        }
        stations.push(OrdinateMember {
            anchor: reference.clone(),
            datum: is_datum,
            value,
            text,
            at: found.at,
            error: String::new(),
        });
    }
    // The sheet's own text pass, exactly as a dimension's: a leader that ran
    // into its own value is broken around it rather than struck through it.
    let lines = super::project::break_for_text_boxes(lines, &texts);
    OrdinateDrawing {
        id: set.id.clone(),
        axis: set.axis.clone(),
        lines,
        texts,
        label,
        offset_mm: set.offset_mm,
        stations,
        error: String::new(),
    }
}

/// A value's footprint on the paper as `[x0, x1, y0, y1]`, centred on the
/// anchor the text is drawn from: the text is centred and vertically centred,
/// so the box is symmetric about it in both directions.
fn value_box(anchor: [f64; 2], half_width: f64, cap: f64) -> [f64; 4] {
    [
        anchor[0] - half_width,
        anchor[0] + half_width,
        anchor[1] - 0.5 * cap,
        anchor[1] + 0.5 * cap,
    ]
}

/// Whether two value boxes are closer than `VALUE_PAD` of a cap height — the
/// test the stagger is made on. Nearly touching counts: two numbers printed a
/// hair apart read as one longer number.
fn boxes_collide(a: &[f64; 4], b: &[f64; 4], cap: f64) -> bool {
    let pad = cap * VALUE_PAD;
    a[0] < b[1] + pad && b[0] < a[1] + pad && a[2] < b[3] + pad && b[2] < a[3] + pad
}

/// The ordinate ORIGIN symbol: the small circle at the datum, as a closed
/// polyline — the sheet draws lines, so a circle is a polygon here exactly as
/// an arrowhead is a closed outline.
fn origin_circle(centre: [f64; 2], radius: f64) -> Vec<[f64; 2]> {
    (0..=ORIGIN_STEPS)
        .map(|step| {
            let angle = std::f64::consts::TAU * step as f64 / ORIGIN_STEPS as f64;
            [centre[0] + radius * angle.cos(), centre[1] + radius * angle.sin()]
        })
        .collect()
}

