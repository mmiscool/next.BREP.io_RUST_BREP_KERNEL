//! Sheet dimensions — a dimension drawn ON the paper, anchored to the
//! projected geometry of a placed view rather than to a PMI annotation.
//!
//! # The anchor space
//!
//! Every annotation a sheet drew before this module came from a saved PMI
//! view: it was authored in 3D, resolved by the kernel, and the sheet only
//! projected it. A SHEET dimension is authored on the paper, so it needs an
//! anchor space of its own — and the one thing it must NOT be is a paper
//! position, because the paper position of everything a placement draws moves
//! whenever the camera, the scale, the placement or the model moves.
//!
//! A [`SheetAnchor`] therefore names a point of a placed view's projection by
//! MODEL IDENTITY:
//!
//! | kind | names | the point |
//! |---|---|---|
//! | `vertex` | a placement, a solid and a display vertex's topology id | the vertex, projected |
//! | `edge` | a placement, a solid, an edge NAME and a fraction | that fraction along the edge by arc length, projected |
//! | `circle` | a placement, a solid and a CIRCULAR edge's name | the circle's centre, projected |
//!
//! An ANGULAR dimension names two `edge` anchors and measures between the two
//! EDGES rather than between the two points: the point picks which of the four
//! angles at the crossing is meant (each ray leaves the vertex toward its own
//! anchor) and the direction comes from the edge. So the same three candidates
//! per edge serve both, and no fifth anchor kind is needed.
//!
//! and its reference string is `{placement}:{kind}:{solid}#{entity}`, with the
//! fraction after an `@` on an edge (`SV2:edge:Box#Box_NX|Box_NZ[0]@0.5`).
//! Edge names are the kernel's own pipeline names and a display vertex's
//! topology id is stable across a parameter edit, so an anchor FOLLOWS its
//! geometry: the projection re-runs on every camera, scale, placement and
//! model change, and the anchor is re-resolved against it each time.
//!
//! An anchor whose geometry is GONE — the edge no longer exists after a model
//! edit, the placement was removed — is not dropped. The dimension keeps its
//! row, [`DimensionDrawing::error`] carries the reason, and the sheet draws it
//! in the unresolved style the 3D overlay uses for an annotation that will not
//! resolve. A drawing that silently loses a dimension is worse than one that
//! says which dimension it lost.
//!
//! **A HIDDEN edge is still an anchor.** The hidden-line pass decides which
//! model LINES are inked; it has no vote on the anchor space, which is the
//! model's topology. An anchor on an edge the pass removed resolves exactly as
//! one on a visible edge, and the dimension DRAWS to it — extension line,
//! arrowhead and all — landing on a part of the paper where no model line was
//! drawn. That is what dimensioning a hidden feature looks like on a drawing.
//! The viewport's picker offers every anchor of a placement for the same
//! reason: it is picking model geometry, not ink.
//!
//! # The drawing
//!
//! The layout is the SHARED one — `pmi_present`, the same function the 3D
//! overlay, the STEP presentation and every placed annotation go through — run
//! in a frame that IS the paper: x to the right, y UP, z out of the page, in
//! paper millimetres. So a sheet dimension's extension lines, dimension line,
//! arrowheads, value run and line breaking are the ones every other dimension
//! in the product has, and its text is flat on the sheet by construction —
//! there is no annotation plane to project out of.
//!
//! Two things this module does that the shared layout cannot:
//!
//! - **The component pair.** A HORIZONTAL or VERTICAL dimension measures one
//!   component, and the layout draws the dimension line along `b - a`. The
//!   kernel's own X/Y/Z rule answers that by replacing the second point with
//!   its projection onto the measured axis, and this reads the same rule in
//!   the sheet's own axes.
//! - **Re-seating the extension lines.** Because the layout measured that
//!   PROJECTED pair, one of its two extension lines starts at the projected
//!   point rather than at the anchor it belongs to. Each is re-seated on its
//!   true anchor and given the layout's own overshoot past the dimension line,
//!   so a horizontal dimension between two points at different heights reaches
//!   both of them.

use serde::{Deserialize, Serialize};
use serde_json::{json, Value};

use super::project::{AnchorPoint, Measures, SheetText, ViewDrawing};
use brep_kernel::{format_dimension, pmi_present, PmiGeometry, PmiLayoutStyle, ToleranceBlock, ToleranceMode};

/// The dimension kinds.
pub const LINEAR: &str = "linear";
pub const RADIAL: &str = "radial";
pub const DIAMETRAL: &str = "diametral";
pub const ANGULAR: &str = "angular";
pub const KINDS: &[&str] = &[LINEAR, RADIAL, DIAMETRAL, ANGULAR];

/// How many anchors a dimension of `kind` takes: TWO for a linear one (the two
/// measured points) and for an angular one (the two measured EDGES), one for a
/// radial or diametral one (the circle).
///
/// The ONE reading of that count. `SheetDimension::anchor_count` is the layout
/// asking it and the sheet reference picker is the other asker (a full list
/// makes room for a new pick), and a kind whose two answers disagreed would be
/// a picker that collects what the layout refuses.
pub fn anchor_count(kind: &str) -> usize {
    if kind.eq_ignore_ascii_case(LINEAR) || kind.eq_ignore_ascii_case(ANGULAR) {
        2
    } else {
        1
    }
}

/// A linear dimension's alignment: which direction it measures along.
pub const HORIZONTAL: &str = "horizontal";
pub const VERTICAL: &str = "vertical";
pub const ALIGNED: &str = "aligned";
pub const ALIGNMENTS: &[&str] = &[HORIZONTAL, VERTICAL, ALIGNED];

/// The anchor kinds.
pub const VERTEX: &str = "vertex";
pub const EDGE: &str = "edge";
pub const CIRCLE: &str = "circle";

/// A new dimension's offset from its anchors, paper millimetres.
pub const DEFAULT_OFFSET_MM: f64 = 12.0;

/// Decimals in a sheet dimension's value, matching the kernel's own linear
/// default.
pub const DEFAULT_PRECISION: u32 = 3;

/// The direction a RADIAL or DIAMETRAL dimension's leader leaves the centre,
/// on the paper: up and to the right at 45°. A radial dimension's offset is
/// one number (the distance from the circle outward), so the direction is a
/// convention rather than a stored value.
const RADIAL_DIR: [f64; 2] = [std::f64::consts::FRAC_1_SQRT_2, std::f64::consts::FRAC_1_SQRT_2];

// ===========================================================================
// The tolerance block
// ===========================================================================

/// The tolerance block a sheet dimension or an ordinate set carries: the PMI
/// dimension's own four params under the PMI's own keys (`tolMode`,
/// `tolUpper`, `tolLower`, `isReference`), read into the kernel's
/// [`ToleranceBlock`] and written by the kernel's [`format_dimension`] — so a
/// sheet value reads `10.000 ±0.050` exactly as the viewport's label and the
/// STEP presentation of a PMI dimension do. Every key is omitted while it
/// holds its default, so a sheet saved before the block re-serializes
/// byte-identically.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SheetTolerance {
    /// One of [`tolerance_modes`].
    #[serde(default = "default_tol_mode", rename = "tolMode", skip_serializing_if = "is_no_tolerance")]
    pub mode: String,
    /// The upper deviation — the ± value of a symmetric block.
    #[serde(default, rename = "tolUpper", skip_serializing_if = "is_zero")]
    pub upper: f64,
    /// The lower deviation (deviation and limits blocks).
    #[serde(default, rename = "tolLower", skip_serializing_if = "is_zero")]
    pub lower: f64,
    /// A REFERENCE dimension: parenthesized, and no tolerance shown.
    #[serde(default, rename = "isReference", skip_serializing_if = "is_false")]
    pub reference: bool,
}

impl Default for SheetTolerance {
    fn default() -> SheetTolerance {
        SheetTolerance { mode: default_tol_mode(), upper: 0.0, lower: 0.0, reference: false }
    }
}

fn default_tol_mode() -> String {
    ToleranceMode::None.as_str().to_string()
}

fn is_no_tolerance(mode: &String) -> bool {
    ToleranceMode::parse(mode) == ToleranceMode::None
}

fn is_zero(value: &f64) -> bool {
    *value == 0.0
}

fn is_false(flag: &bool) -> bool {
    !*flag
}

/// The tolerance modes, in the PMI form's order, read off the kernel's own
/// [`ToleranceMode`] so the two dialogs cannot offer different lists.
pub fn tolerance_modes() -> Vec<&'static str> {
    [ToleranceMode::None, ToleranceMode::Symmetric, ToleranceMode::Deviation, ToleranceMode::Limits]
        .iter()
        .map(|mode| mode.as_str())
        .collect()
}

impl SheetTolerance {
    /// The kernel's block — magnitudes, as `ToleranceBlock::read` takes them.
    pub fn block(&self) -> ToleranceBlock {
        ToleranceBlock { mode: ToleranceMode::parse(&self.mode), upper: self.upper.abs(), lower: self.lower.abs() }
    }

    /// No tolerance and not a reference: the plain formatted number, and the
    /// one state in which a sheet dimension may inherit a PMI dimension's.
    pub fn is_plain(&self) -> bool {
        self.block().mode == ToleranceMode::None && !self.reference
    }

    /// Write the four params into a form's params object.
    pub fn write_params(&self, params: &mut Value) {
        params["tolMode"] = json!(ToleranceMode::parse(&self.mode).as_str());
        params["tolUpper"] = json!(self.upper);
        params["tolLower"] = json!(self.lower);
        params["isReference"] = json!(self.reference);
    }

    /// Apply an edited form's four params, refusing a mode the kernel does not
    /// know rather than reading it as none.
    pub fn apply(&mut self, params: &Value) -> Result<(), String> {
        if let Some(mode) = params.get("tolMode").and_then(Value::as_str) {
            let mode = mode.trim();
            let Some(known) = tolerance_modes().into_iter().find(|known| known.eq_ignore_ascii_case(mode)) else {
                return Err(format!("no tolerance mode '{mode}' (one of {})", tolerance_modes().join(", ")));
            };
            self.mode = known.to_string();
        }
        for (key, slot) in [("tolUpper", &mut self.upper), ("tolLower", &mut self.lower)] {
            if let Some(value) = params.get(key).and_then(Value::as_f64) {
                if !value.is_finite() {
                    return Err(format!("{key}: {value} is not a deviation"));
                }
                *slot = value.abs();
            }
        }
        if let Some(reference) = params.get("isReference").and_then(Value::as_bool) {
            self.reference = reference;
        }
        Ok(())
    }

    /// The four fields of a dialog schema, labelled and hinted as the PMI
    /// dimension form labels them.
    pub fn schema_fields(schema: &mut serde_json::Map<String, Value>) {
        schema.insert(
            "isReference".into(),
            json!({ "type": "boolean", "label": "Reference", "default_value": false, "hint": "A reference dimension: shown in parentheses, no tolerance" }),
        );
        schema.insert(
            "tolMode".into(),
            json!({
                "type": "options", "label": "Tolerance", "options": tolerance_modes(), "default_value": ToleranceMode::None.as_str(),
                "hint": "none \u{00B7} \u{00B1} symmetric \u{00B7} +upper/\u{2212}lower deviation \u{00B7} upper/lower limit values"
            }),
        );
        schema.insert(
            "tolUpper".into(),
            json!({ "type": "number", "label": "Upper (+)", "default_value": 0.0, "step": 0.01, "hint": "The upper deviation (the \u{00B1} value for symmetric)" }),
        );
        schema.insert(
            "tolLower".into(),
            json!({ "type": "number", "label": "Lower (\u{2212})", "default_value": 0.0, "step": 0.01, "hint": "The lower deviation (deviation / limits modes)" }),
        );
    }
}

// ===========================================================================
// The block
// ===========================================================================

/// One dimension drawn on the sheet.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SheetDimension {
    pub id: String,
    /// [`LINEAR`], [`RADIAL`], [`DIAMETRAL`] or [`ANGULAR`].
    #[serde(default = "default_kind")]
    pub kind: String,
    /// A linear dimension's measured direction ([`ALIGNMENTS`]); unread by a
    /// radial or diametral one, which measures a circle, and by an angular
    /// one, whose direction is the two edges'.
    #[serde(default = "default_alignment")]
    pub alignment: String,
    /// The anchor references — two for a linear dimension, one for a radial or
    /// diametral one. Stored as their reference STRINGS, the convention every
    /// other reference in the document follows.
    #[serde(default)]
    pub anchors: Vec<String>,
    /// How far the dimension line sits from the anchors, paper millimetres.
    /// Signed: the sign picks the side.
    #[serde(default = "default_offset", rename = "offsetMm")]
    pub offset_mm: f64,
    /// Decimals in the value.
    #[serde(default = "default_precision")]
    pub precision: u32,
    /// The tolerance block the value is written with.
    #[serde(flatten)]
    pub tolerance: SheetTolerance,
}

fn default_kind() -> String {
    LINEAR.to_string()
}

fn default_alignment() -> String {
    ALIGNED.to_string()
}

fn default_offset() -> f64 {
    DEFAULT_OFFSET_MM
}

fn default_precision() -> u32 {
    DEFAULT_PRECISION
}

impl SheetDimension {
    /// A new dimension of `kind` / `alignment` over `anchors`.
    pub fn new(id: String, kind: &str, alignment: &str, anchors: Vec<String>) -> SheetDimension {
        SheetDimension {
            id,
            kind: kind.to_string(),
            alignment: alignment.to_string(),
            anchors,
            offset_mm: DEFAULT_OFFSET_MM,
            precision: DEFAULT_PRECISION,
            tolerance: SheetTolerance::default(),
        }
    }

    /// How many anchors this kind takes — [`anchor_count`], which the picker
    /// reads too.
    pub fn anchor_count(&self) -> usize {
        anchor_count(&self.kind)
    }

    /// The params the **Sheet dimension** form edits.
    pub fn params(&self) -> Value {
        let mut params = json!({
            "id": self.id,
            "kind": self.kind,
            "alignment": self.alignment,
            "anchors": self.anchors,
            "offsetMm": self.offset_mm,
            "precision": self.precision,
        });
        self.tolerance.write_params(&mut params);
        params
    }

    /// Apply an edited **Sheet dimension** form. `id` is identity and is never
    /// taken from the form.
    pub fn apply(&mut self, params: &Value) -> Result<(), String> {
        if let Some(kind) = params.get("kind").and_then(Value::as_str) {
            let kind = kind.trim();
            let Some(known) = KINDS.iter().find(|k| k.eq_ignore_ascii_case(kind)) else {
                return Err(format!("no dimension kind '{kind}'"));
            };
            self.kind = (*known).to_string();
        }
        if let Some(alignment) = params.get("alignment").and_then(Value::as_str) {
            let alignment = alignment.trim();
            let Some(known) = ALIGNMENTS.iter().find(|a| a.eq_ignore_ascii_case(alignment)) else {
                return Err(format!("no alignment '{alignment}'"));
            };
            self.alignment = (*known).to_string();
        }
        if let Some(anchors) = params.get("anchors") {
            let list: Vec<String> = crate::json_support::string_values(Some(anchors))
                .map(str::to_string)
                .collect();
            // Every anchor the form keeps has to BE an anchor: a reference row
            // is free text, and a dimension that stored one it cannot parse
            // would report the same "lost anchor" a real model edit reports.
            for anchor in &list {
                SheetAnchor::parse(anchor)?;
            }
            self.anchors = list;
        }
        apply_offset_precision(params, &mut self.offset_mm, &mut self.precision)?;
        self.tolerance.apply(params)
    }
}

/// Apply the numeric layout fields shared by dimensions and ordinate sets.
/// Missing or non-numeric values leave the current setting unchanged.
pub(super) fn apply_offset_precision(
    params: &Value,
    offset_mm: &mut f64,
    precision: &mut u32,
) -> Result<(), String> {
    if let Some(offset) = params.get("offsetMm").and_then(Value::as_f64) {
        if !offset.is_finite() {
            return Err(format!("offsetMm: {offset} is not a length"));
        }
        *offset_mm = offset;
    }
    if let Some(value) = params.get("precision").and_then(Value::as_f64) {
        if !value.is_finite() || !(0.0..=8.0).contains(&value) {
            return Err(format!("precision: {value} is not 0 to 8 decimals"));
        }
        *precision = value.round() as u32;
    }
    Ok(())
}

/// The **Sheet dimension** dialog's schema, in the `inputParamsSchema`
/// vocabulary the shared form engine renders — so the dialog is not
/// hand-written, exactly like the sheet's and the placement's.
///
/// `anchors` is a `reference_selection`: the form draws it as the reference
/// rows every feature's references are drawn as, its **Select** button opens
/// the reference picker on the sheet's paper, and each row's ✕ drops that
/// anchor.
/// With no `label` its row reads the prettified key, "Anchors".
///
/// The last four fields are the tolerance block ([`SheetTolerance`]), the PMI
/// dimension form's own.
pub fn sheet_dimension_schema() -> Value {
    let mut fields = json!({
        "id": { "type": "string", "default_value": null },
        "kind": { "type": "options", "label": "Kind", "options": KINDS, "default_value": LINEAR },
        "alignment": { "type": "options", "label": "Alignment", "options": ALIGNMENTS, "default_value": ALIGNED },
        "anchors": { "type": "reference_selection", "selectionFilter": ["sheetAnchor"], "multiple": true },
        "offsetMm": { "type": "number", "label": "Offset (mm)", "default_value": DEFAULT_OFFSET_MM },
        "precision": { "type": "number", "label": "Precision (decimals)", "default_value": DEFAULT_PRECISION },
    });
    SheetTolerance::schema_fields(fields.as_object_mut().expect("an object"));
    json!({
        "type": "sheetDimension",
        "shortName": "SD",
        "longName": "Sheet dimension",
        "inputParamsSchema": fields,
    })
}

// ===========================================================================
// Anchors
// ===========================================================================

/// A parsed anchor reference.
#[derive(Debug, Clone, PartialEq)]
pub struct SheetAnchor {
    /// The PLACED VIEW's id (`SV2`) — which projection the point belongs to.
    pub view: String,
    /// [`VERTEX`], [`EDGE`] or [`CIRCLE`].
    pub kind: String,
    /// The solid the geometry belongs to.
    pub solid: String,
    /// A display vertex's topology id ([`VERTEX`]).
    pub vertex: u64,
    /// The kernel edge name ([`EDGE`], [`CIRCLE`]).
    pub edge: String,
    /// Arc-length fraction along the edge ([`EDGE`]): 0 and 1 are its ends,
    /// 0.5 its midpoint.
    pub fraction: f64,
}

impl SheetAnchor {
    pub fn vertex(view: &str, solid: &str, topo_id: u64) -> SheetAnchor {
        SheetAnchor {
            view: view.into(),
            kind: VERTEX.into(),
            solid: solid.into(),
            vertex: topo_id,
            edge: String::new(),
            fraction: 0.0,
        }
    }

    pub fn edge(view: &str, solid: &str, edge: &str, fraction: f64) -> SheetAnchor {
        SheetAnchor {
            view: view.into(),
            kind: EDGE.into(),
            solid: solid.into(),
            vertex: 0,
            edge: edge.into(),
            fraction,
        }
    }

    pub fn circle(view: &str, solid: &str, edge: &str) -> SheetAnchor {
        SheetAnchor {
            view: view.into(),
            kind: CIRCLE.into(),
            solid: solid.into(),
            vertex: 0,
            edge: edge.into(),
            fraction: 0.0,
        }
    }

    /// The reference string: `{placement}:{kind}:{solid}#{entity}`, with an
    /// edge's fraction after an `@`.
    ///
    /// The two separators are chosen so the parse cannot be ambiguous: a
    /// kernel edge name carries `|` and `[n]` but never a `#` or an `@`, and
    /// the placement id and the kind are this module's own vocabulary.
    pub fn to_ref(&self) -> String {
        match self.kind.as_str() {
            EDGE => format!(
                "{}:{}:{}#{}@{}",
                self.view, self.kind, self.solid, self.edge, self.fraction
            ),
            CIRCLE => format!("{}:{}:{}#{}", self.view, self.kind, self.solid, self.edge),
            _ => format!("{}:{}:{}#{}", self.view, VERTEX, self.solid, self.vertex),
        }
    }

    /// Parse a reference string, or say what is wrong with it.
    pub fn parse(text: &str) -> Result<SheetAnchor, String> {
        let bad = || format!("'{text}' is not a sheet anchor (expected SV2:vertex:Solid#7)");
        let mut parts = text.splitn(3, ':');
        let view = parts.next().unwrap_or_default().trim();
        let kind = parts.next().ok_or_else(bad)?.trim();
        let rest = parts.next().ok_or_else(bad)?;
        if view.is_empty() {
            return Err(bad());
        }
        let (solid, entity) = rest.split_once('#').ok_or_else(bad)?;
        if solid.is_empty() || entity.is_empty() {
            return Err(bad());
        }
        match kind {
            VERTEX => {
                let id: u64 = entity
                    .parse()
                    .map_err(|_| format!("'{entity}' is not a vertex id in '{text}'"))?;
                Ok(SheetAnchor::vertex(view, solid, id))
            }
            EDGE => {
                let (edge, fraction) = entity
                    .rsplit_once('@')
                    .ok_or_else(|| format!("'{text}' is an edge anchor with no fraction"))?;
                let fraction: f64 = fraction
                    .parse()
                    .map_err(|_| format!("'{fraction}' is not a fraction in '{text}'"))?;
                if !fraction.is_finite() || !(0.0..=1.0).contains(&fraction) {
                    return Err(format!("'{text}': a fraction runs 0 to 1"));
                }
                Ok(SheetAnchor::edge(view, solid, edge, fraction))
            }
            CIRCLE => Ok(SheetAnchor::circle(view, solid, entity)),
            other => Err(format!("no anchor kind '{other}' in '{text}'")),
        }
    }
}

// ===========================================================================
// The drawing
// ===========================================================================

/// One sheet dimension, laid out in paper millimetres — the sibling of
/// [`crate::sheets::project::AnnotationDrawing`], drawn in its own group by
/// both writers.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct DimensionDrawing {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    /// The kernel-formatted value string, in MODEL units — never divided by
    /// the sheet scale, exactly as a placed annotation's is.
    pub text: String,
    /// The measured value in model units (`NaN` when the dimension did not
    /// resolve).
    pub value: f64,
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
    /// Where the value sits — the point the offset drag grabs, and where an
    /// UNRESOLVED dimension's reason is written.
    pub label: [f64; 2],
    #[serde(rename = "offsetMm")]
    pub offset_mm: f64,
    /// The PMI dimension whose tolerance block the value is written with —
    /// empty unless the dimension carries none of its own and its anchors are
    /// the points a PMI dimension of the placement's view measures (see
    /// [`inherited`]).
    #[serde(rename = "toleranceFrom", skip_serializing_if = "String::is_empty")]
    pub tolerance_from: String,
    /// Empty when the dimension resolved; why it did not otherwise. A
    /// dimension with an error is drawn in the unresolved style rather than
    /// dropped.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub error: String,
}

impl DimensionDrawing {
    fn refused(dimension: &SheetDimension, label: [f64; 2], error: String) -> DimensionDrawing {
        DimensionDrawing {
            id: dimension.id.clone(),
            kind: dimension.kind.clone(),
            text: String::new(),
            value: f64::NAN,
            lines: Vec::new(),
            texts: Vec::new(),
            label,
            offset_mm: dimension.offset_mm,
            tolerance_from: String::new(),
            error,
        }
    }
}

/// Lay out every dimension of `sheet` against the placements already
/// projected into `views`. `centre` is where an unresolved dimension writes
/// its reason when it has no anchor left to write it beside.
pub fn layout_dimensions(
    dimensions: &[SheetDimension],
    views: &[ViewDrawing],
    centre: [f64; 2],
) -> Vec<DimensionDrawing> {
    dimensions
        .iter()
        .map(|dimension| layout_one(dimension, views, centre))
        .collect()
}

fn layout_one(
    dimension: &SheetDimension,
    views: &[ViewDrawing],
    centre: [f64; 2],
) -> DimensionDrawing {
    let wanted = dimension.anchor_count();
    if dimension.anchors.len() != wanted {
        return DimensionDrawing::refused(
            dimension,
            centre,
            format!(
                "a {} dimension takes {wanted} anchor{}; it has {}",
                dimension.kind,
                if wanted == 1 { "" } else { "s" },
                dimension.anchors.len()
            ),
        );
    }
    // Resolve every anchor against the projection, keeping the FIRST failure's
    // reason — the point of the unresolved lane is to say which anchor went.
    let mut resolved: Vec<(&ViewDrawing, &AnchorPoint)> = Vec::new();
    for reference in &dimension.anchors {
        match resolve_anchor(reference, views) {
            Ok(found) => resolved.push(found),
            Err(error) => {
                let label = resolved.first().map(|(_, a)| a.at).unwrap_or(centre);
                return DimensionDrawing::refused(dimension, label, error);
            }
        }
    }
    // This slice dimensions WITHIN one placement: a dimension across two views
    // measures between two different projections, which is a different
    // construction and is named as out of scope rather than guessed at.
    if let [(first, _), rest @ ..] = resolved.as_slice() {
        if let Some((other, _)) = rest.iter().find(|(view, _)| view.id != first.id) {
            return DimensionDrawing::refused(
                dimension,
                resolved[0].1.at,
                format!(
                    "anchors on two placements ('{}' and '{}'); a sheet dimension measures within one view",
                    first.id, other.id
                ),
            );
        }
    }
    let view = resolved[0].0;
    if view.scale <= 0.0 {
        return DimensionDrawing::refused(
            dimension,
            resolved[0].1.at,
            format!("placement '{}' has no scale", view.id),
        );
    }
    if dimension.kind.eq_ignore_ascii_case(LINEAR) {
        linear(dimension, view, resolved[0].1, resolved[1].1)
    } else if dimension.kind.eq_ignore_ascii_case(ANGULAR) {
        angular(dimension, view, resolved[0].1, resolved[1].1)
    } else {
        radial(dimension, view, resolved[0].1)
    }
}

/// The placement and the anchor point a reference names, or why it is lost.
pub(super) fn resolve_anchor<'a>(
    reference: &str,
    views: &'a [ViewDrawing],
) -> Result<(&'a ViewDrawing, &'a AnchorPoint), String> {
    let anchor = SheetAnchor::parse(reference)?;
    let Some(view) = views.iter().find(|view| view.id == anchor.view) else {
        return Err(format!("'{reference}': no placed view '{}' on this sheet", anchor.view));
    };
    if !view.error.is_empty() {
        return Err(format!("'{reference}': placement '{}' did not project", view.id));
    }
    view.anchors
        .iter()
        .find(|candidate| candidate.anchor == reference)
        .map(|candidate| (view, candidate))
        .ok_or_else(|| match anchor.kind.as_str() {
            VERTEX => format!(
                "'{reference}': '{}' has no vertex {} any more",
                anchor.solid, anchor.vertex
            ),
            CIRCLE => format!("'{reference}': '{}' is not a circular edge any more", anchor.edge),
            _ => format!("'{reference}': '{}' has no edge '{}' any more", anchor.solid, anchor.edge),
        })
}

// ---------------------------------------------------------------------------
// The paper as a layout frame
// ---------------------------------------------------------------------------

/// Paper millimetres (y DOWN, SVG's frame) → the layout's own 3D frame: x to
/// the right, y UP, z out of the page.
fn lift(p: [f64; 2]) -> [f64; 3] {
    [p[0], -p[1], 0.0]
}

/// …and back.
fn drop_to_paper(q: [f64; 3]) -> [f64; 2] {
    [q[0], -q[1]]
}

/// A paper DIRECTION into the layout's frame — the same flip as [`lift`], with
/// no translation to carry.
fn lift_dir(d: [f64; 2]) -> [f64; 3] {
    [d[0], -d[1], 0.0]
}

/// The cap height everything a sheet AUTHORS is drawn at: the placement's own
/// text size in paper millimetres, so a sheet dimension and an ordinate value
/// read at the size of the annotations they sit among. A placement with no
/// text size of its own falls back to 12 pt, the PMI default.
pub(super) fn text_height(view: &ViewDrawing) -> f64 {
    if view.text_mm > 1e-9 {
        view.text_mm
    } else {
        12.0 * super::project::MM_PER_POINT
    }
}

/// The style a sheet dimension is laid out with: [`text_height`], the
/// overlay's arrow proportion, and a camera looking straight at the paper.
fn style(view: &ViewDrawing) -> PmiLayoutStyle {
    let text_height = text_height(view);
    PmiLayoutStyle {
        arrow: text_height * super::project::ARROW_PER_TEXT,
        text_height,
        view_dir: [0.0, 0.0, -1.0],
        view_up: [0.0, 1.0, 0.0],
        plane: None,
    }
}

/// Finish a laid-out dimension: project every piece back onto the paper,
/// close the arrowheads into outlines (the sheet's contract is lines and
/// text), and break the lines around the value.
fn finish(
    dimension: &SheetDimension,
    drawn: brep_kernel::PmiPresentation,
    value: f64,
    (text, tolerance_from): (String, String),
    reseat: &[(([f64; 3], [f64; 3]), [f64; 3])],
) -> DimensionDrawing {
    let mut lines: Vec<Vec<[f64; 2]>> = Vec::new();
    for polyline in &drawn.polylines {
        // An extension line the layout seated on the PROJECTED point is moved
        // onto the anchor it belongs to and given the layout's own overshoot
        // past the dimension line — which is the one thing a component
        // dimension's shared layout cannot know.
        let seated = reseat.iter().find(|((from, _), _)| {
            polyline.len() == 2 && near(polyline[0], *from)
        });
        match seated {
            Some(((_, to), true_point)) => lines.push(vec![drop_to_paper(*true_point), drop_to_paper(*to)]),
            None => lines.push(polyline.iter().map(|p| drop_to_paper(*p)).collect()),
        }
    }
    for arrow in &drawn.arrows {
        let mut closed: Vec<[f64; 2]> = arrow.iter().map(|p| drop_to_paper(*p)).collect();
        closed.push(closed[0]);
        lines.push(closed);
    }
    let texts: Vec<SheetText> = drawn
        .texts
        .iter()
        .filter(|run| !run.text.trim().is_empty())
        .map(|run| SheetText::flat(run.text.clone(), drop_to_paper(run.anchor), run.height))
        .collect();
    let label = texts.first().map(|run| run.anchor).unwrap_or(drop_to_paper(drawn.text_anchor));
    let lines = super::project::break_for_text_boxes(lines, &texts);
    DimensionDrawing {
        id: dimension.id.clone(),
        kind: dimension.kind.clone(),
        text,
        value,
        lines,
        texts,
        label,
        offset_mm: dimension.offset_mm,
        tolerance_from,
        error: String::new(),
    }
}

// ---------------------------------------------------------------------------
// The value's text, and the tolerance a dimension inherits
// ---------------------------------------------------------------------------

/// The dimension's value text — the kernel's [`format_dimension`] with the
/// block the dimension carries, or else the one it inherits — and the PMI
/// dimension it inherited from (empty when it did not).
fn value_text(
    dimension: &SheetDimension,
    view: &ViewDrawing,
    value: f64,
    (prefix, suffix): (&str, &str),
    source: impl Fn(&Measures) -> bool,
) -> (String, String) {
    let (block, reference, from) = match inherited(dimension, view, source) {
        Some(found) => (found.block, found.reference, found.id.clone()),
        None => (dimension.tolerance.block(), dimension.tolerance.reference, String::new()),
    };
    (format_dimension(value, dimension.precision as usize, &block, reference, prefix, suffix), from)
}

/// The PMI dimension of the placement's view a sheet dimension INHERITS its
/// tolerance from: none when the sheet dimension carries a block or the
/// reference flag of its own (its own always wins), else the first of the
/// view's [`super::project::ToleranceSource`]s that measures what `source`
/// accepts — the same points, in this view.
///
/// "The same point" is the CROSSING FLOOR (`pcurve_consistency` times the
/// placement's scale, [`super::hlr::Floors`]): two projected points closer
/// than it are one point to the hidden-line pass, and so they are here.
pub fn inherited<'a>(
    dimension: &SheetDimension,
    view: &'a ViewDrawing,
    source: impl Fn(&Measures) -> bool,
) -> Option<&'a super::project::ToleranceSource> {
    if !dimension.tolerance.is_plain() {
        return None;
    }
    view.tolerances.iter().find(|candidate| source(&candidate.measures))
}

/// Whether two paper points are one to the placement's crossing floor.
fn same_point(view: &ViewDrawing, a: [f64; 2], b: [f64; 2]) -> bool {
    length2([a[0] - b[0], a[1] - b[1]]) <= super::hlr::Floors::of(view.scale).crossing_mm
}

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < 1e-9)
}

fn length2(v: [f64; 2]) -> f64 {
    (v[0] * v[0] + v[1] * v[1]).sqrt()
}

/// The 2D cross product — the signed area of the parallelogram `a`, `b` span.
fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

// ---------------------------------------------------------------------------
// Linear
// ---------------------------------------------------------------------------

fn linear(
    dimension: &SheetDimension,
    view: &ViewDrawing,
    first: &AnchorPoint,
    second: &AnchorPoint,
) -> DimensionDrawing {
    let (a, b) = (lift(first.at), lift(second.at));
    let alignment = dimension.alignment.as_str();
    // The measured direction, canonicalised so the offset's SIDE does not
    // depend on the order the two anchors were picked in: rightward, or
    // downward when the measurement is vertical.
    let raw = [b[0] - a[0], b[1] - a[1]];
    let flip = if alignment.eq_ignore_ascii_case(HORIZONTAL) {
        raw[0] < 0.0
    } else if alignment.eq_ignore_ascii_case(VERTICAL) {
        raw[1] > 0.0
    } else {
        raw[0] < 0.0 || (raw[0] == 0.0 && raw[1] > 0.0)
    };
    let (a, b) = if flip { (b, a) } else { (a, b) };
    let span = [b[0] - a[0], b[1] - a[1]];
    let d = match alignment {
        _ if alignment.eq_ignore_ascii_case(HORIZONTAL) => [1.0, 0.0],
        _ if alignment.eq_ignore_ascii_case(VERTICAL) => [0.0, -1.0],
        _ => {
            let len = length2(span);
            if len < 1e-9 {
                return DimensionDrawing::refused(
                    dimension,
                    first.at,
                    "the two anchors project to the same point".into(),
                );
            }
            [span[0] / len, span[1] / len]
        }
    };
    // The dimension line's own normal, the side the offset moves it to.
    let n = [-d[1], d[0]];
    let value_mm = (span[0] * d[0] + span[1] * d[1]).abs();
    if value_mm < 1e-9 {
        return DimensionDrawing::refused(
            dimension,
            first.at,
            format!("the two anchors have no {} separation", dimension.alignment),
        );
    }
    // The kernel's own X / Y / Z component rule, read in the sheet's axes: the
    // layout draws the dimension line along `b - a`, so a component dimension
    // measures a PROJECTED pair. The common cross-coordinate is the anchor
    // FARTHER from the side the offset moves to, so the dimension line clears
    // both anchors rather than landing between them.
    let (ca, cb) = (a[0] * n[0] + a[1] * n[1], b[0] * n[0] + b[1] * n[1]);
    let base = if dimension.offset_mm >= 0.0 { ca.min(cb) } else { ca.max(cb) };
    let onto = |p: [f64; 3], c: f64| -> [f64; 3] {
        let along = p[0] * d[0] + p[1] * d[1];
        [along * d[0] + c * n[0], along * d[1] + c * n[1], 0.0]
    };
    let (pa, pb) = (onto(a, base), onto(b, base));
    // Paper distance determines layout, but dividing it by scale cannot
    // recover the depth lost to projection. Aligned dimensions measure the
    // original 3D pair; horizontal/vertical dimensions measure view components.
    let aligned = alignment.eq_ignore_ascii_case(ALIGNED);
    let value = if aligned {
        first.model.iter().zip(second.model).map(|(a, b)| (b - a).powi(2)).sum::<f64>().sqrt()
    } else {
        value_mm / view.scale
    };
    // A free PMI dimension between these very two points hands its block on —
    // to an ALIGNED sheet dimension only: a horizontal or vertical one measures
    // a component, which is a different number.
    let text = value_text(dimension, view, value, ("", ""), |measures| match measures {
        Measures::Linear { a, b } => {
            aligned
                && ((same_point(view, *a, first.at) && same_point(view, *b, second.at))
                    || (same_point(view, *a, second.at) && same_point(view, *b, first.at)))
        }
        _ => false,
    });
    let style = style(view);
    // Where the value sits along the line. Centred when its box and both
    // arrowheads fit between the extension lines; otherwise just PAST the far
    // one, one arrowhead clear of it — the layout runs the dimension line on
    // to the value's station — because a value centred on a line too short for
    // it would cover the line and break both arrowheads out of the drawing. A
    // toleranced value is two or three times the plain number's width, which
    // is when this starts to matter.
    let half = super::project::text_half_width(&text.0, style.text_height);
    let along = if 2.0 * (half + style.arrow) <= value_mm {
        [(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5]
    } else {
        let past = style.arrow + half;
        [pb[0] + d[0] * past, pb[1] + d[1] * past]
    };
    let label = [along[0] + n[0] * dimension.offset_mm, along[1] + n[1] * dimension.offset_mm, 0.0];
    let drawn = pmi_present(
        &PmiGeometry::Linear { a: pa, b: pb, component: None },
        label,
        &text.0,
        &style,
    );
    // The layout's extension lines start at the pair it MEASURED. Re-seat each
    // on its true anchor and run it to the layout's own overshoot past the
    // dimension line — computed from the anchor's own side, so an anchor
    // BEYOND the line still reaches past it rather than stopping short.
    let overshoot = style.arrow * 0.6;
    let reseat: Vec<(([f64; 3], [f64; 3]), [f64; 3])> = [(pa, a), (pb, b)]
        .iter()
        .filter_map(|(projected, truth)| {
            let foot = [
                projected[0] + n[0] * dimension.offset_mm,
                projected[1] + n[1] * dimension.offset_mm,
                0.0,
            ];
            let away = [foot[0] - truth[0], foot[1] - truth[1]];
            let len = length2(away);
            if len < 1e-9 {
                return None;
            }
            let end = [
                foot[0] + away[0] / len * overshoot,
                foot[1] + away[1] / len * overshoot,
                0.0,
            ];
            Some(((*projected, end), *truth))
        })
        .collect();
    finish(dimension, drawn, value, text, &reseat)
}

// ---------------------------------------------------------------------------
// Radial and diametral
// ---------------------------------------------------------------------------

fn radial(dimension: &SheetDimension, view: &ViewDrawing, anchor: &AnchorPoint) -> DimensionDrawing {
    let (Some(radius), Some(u), Some(v)) = (anchor.radius, anchor.u, anchor.v) else {
        return DimensionDrawing::refused(
            dimension,
            anchor.at,
            format!("'{}' is not a circular edge", anchor.anchor),
        );
    };
    let diameter = dimension.kind.eq_ignore_ascii_case(DIAMETRAL);
    // The projected circle is an ELLIPSE — `u` and `v` are the paper images of
    // its two conjugate radii — so "the radius" the leader reaches is the
    // ellipse's extent along the leader's own direction. An ellipse is centrally
    // symmetric, so the diameter line through the centre reaches the same
    // distance on both sides, which is exactly what the shared layout draws.
    let dir = RADIAL_DIR;
    let (u, v) = ([u[0], -u[1]], [v[0], -v[1]]);
    let theta = (-cross2(u, dir)).atan2(cross2(v, dir));
    let at = |t: f64| [u[0] * t.cos() + v[0] * t.sin(), u[1] * t.cos() + v[1] * t.sin()];
    let mut point = at(theta);
    if point[0] * dir[0] + point[1] * dir[1] < 0.0 {
        point = at(theta + std::f64::consts::PI);
    }
    let paper_radius = length2(point);
    if !(paper_radius > 1e-9) {
        return DimensionDrawing::refused(
            dimension,
            anchor.at,
            format!("'{}' projects edge-on: its circle has no radius on the paper", anchor.anchor),
        );
    }
    let centre = lift(anchor.at);
    let reach = paper_radius + dimension.offset_mm;
    let label = [centre[0] + dir[0] * reach, centre[1] + dir[1] * reach, 0.0];
    let value = if diameter { radius * 2.0 } else { radius };
    // A PMI radial (or diametral) dimension of this circle — the same centre on
    // the paper and the same MODEL radius to the B-rep bar — hands its block on
    // to the sheet dimension of the same kind.
    let bar = super::hlr::Floors::of(view.scale).bar;
    let text = value_text(
        dimension,
        view,
        value,
        (if diameter { "\u{2300}" } else { "R" }, ""),
        |measures| match measures {
            Measures::Radial { centre, radius: measured, diameter: pmi_diameter } => {
                *pmi_diameter == diameter && same_point(view, *centre, anchor.at) && (measured - radius).abs() <= bar
            }
            _ => false,
        },
    );
    let style = style(view);
    let drawn = pmi_present(
        &PmiGeometry::Radial {
            center: centre,
            axis: [0.0, 0.0, 1.0],
            radius: paper_radius,
            diameter,
            sphere: false,
        },
        label,
        &text.0,
        &style,
    );
    finish(dimension, drawn, value, text, &[])
}


// ---------------------------------------------------------------------------
// Angular
// ---------------------------------------------------------------------------

/// The angle between two projected STRAIGHT edges, as an arc with tangent
/// arrowheads and the value in degrees.
///
/// Both anchors are `edge` candidates, and each carries its edge's own unit
/// PAPER direction ([`AnchorPoint::dir`]) — the direction as THIS view projects
/// it, so a foreshortened angle reads its projected value rather than the
/// model's. That is the drawing convention and it is what an orthographic view
/// can honestly say: the true angle between two skew or tilted edges is not a
/// property of this projection.
///
/// **Which angle.** Two lines crossing make four angles, and the pair of RAYS
/// this measures is chosen by the anchors themselves: each ray leaves the
/// vertex toward the anchor POINT that was picked, so picking an edge's near
/// end and its far end pick supplementary angles. The value is therefore always
/// the angle between the two picked rays, strictly between 0° and 180°.
///
/// `offsetMm` is the arc's radius from the vertex, measured along the bisector
/// of the two rays — the one number a single drag can move, exactly as a radial
/// dimension's offset is.
fn angular(
    dimension: &SheetDimension,
    view: &ViewDrawing,
    first: &AnchorPoint,
    second: &AnchorPoint,
) -> DimensionDrawing {
    let direction = |anchor: &AnchorPoint| -> Result<[f64; 2], String> {
        anchor.dir.ok_or_else(|| {
            if anchor.kind == EDGE {
                format!(
                    "'{}' is not a straight edge in this view; an angular dimension measures between two straight edges",
                    anchor.anchor
                )
            } else {
                format!("'{}' is a {} anchor; an angular dimension measures between two EDGES", anchor.anchor, anchor.kind)
            }
        })
    };
    let (da, db) = match (direction(first), direction(second)) {
        (Ok(a), Ok(b)) => (a, b),
        (Err(error), _) | (_, Err(error)) => {
            return DimensionDrawing::refused(dimension, first.at, error)
        }
    };
    // Where the two lines cross — the arc's centre and the vertex the
    // extension lines run out from. Parallel lines never do, and there is no
    // angle to draw rather than a very distant one.
    let denominator = cross2(da, db);
    if denominator.abs() < 1e-9 {
        return DimensionDrawing::refused(
            dimension,
            first.at,
            format!(
                "'{}' and '{}' are parallel in this view; they make no angle",
                first.anchor, second.anchor
            ),
        );
    }
    let between = [second.at[0] - first.at[0], second.at[1] - first.at[1]];
    let t = cross2(between, db) / denominator;
    let vertex = [first.at[0] + da[0] * t, first.at[1] + da[1] * t];
    // Each ray points from the vertex at the anchor that was picked. An anchor
    // sitting ON the vertex — an edge picked at the very end the other one
    // meets — has no side to name, so the edge's own direction stands in.
    let ray = |anchor: &AnchorPoint, fallback: [f64; 2]| -> [f64; 2] {
        let away = [anchor.at[0] - vertex[0], anchor.at[1] - vertex[1]];
        let length = length2(away);
        if length < 1e-9 {
            fallback
        } else {
            [away[0] / length, away[1] / length]
        }
    };
    let (ra, rb) = (ray(first, da), ray(second, db));
    let degrees = cross2(ra, rb).abs().atan2(ra[0] * rb[0] + ra[1] * rb[1]).to_degrees();
    if !(degrees > 1e-6) {
        return DimensionDrawing::refused(
            dimension,
            first.at,
            format!(
                "'{}' and '{}' leave the vertex the same way; they make no angle",
                first.anchor, second.anchor
            ),
        );
    }
    // Into the layout's own frame (x right, y UP, z out of the page), where the
    // arc sweeps `degrees` from `dir_a` about +z RIGHT-HANDED — so the pair is
    // ordered by that sweep rather than by the order the anchors were picked.
    let (la, lb) = (lift_dir(ra), lift_dir(rb));
    let (dir_a, dir_b) = if cross2([la[0], la[1]], [lb[0], lb[1]]) >= 0.0 {
        (la, lb)
    } else {
        (lb, la)
    };
    let bisector = [dir_a[0] + dir_b[0], dir_a[1] + dir_b[1]];
    let length = length2([bisector[0], bisector[1]]);
    // `degrees` is strictly below 180, so the two rays never cancel.
    let bisector = [bisector[0] / length, bisector[1] / length, 0.0];
    let centre = lift(vertex);
    let label = [
        centre[0] + bisector[0] * dimension.offset_mm,
        centre[1] + bisector[1] * dimension.offset_mm,
        0.0,
    ];
    // The kernel's own angle string: the number at this sheet's precision with
    // a degree sign, the same text a PMI angle annotation carries.
    // An angle inherits nothing: the sheet reads the angle AS THIS VIEW
    // PROJECTS IT, which is the model's angle only when the view looks down
    // the angle's axis, and a tolerance on one is not a tolerance on the other.
    let text = value_text(dimension, view, degrees, ("", "\u{00B0}"), |_| false);
    let style = style(view);
    let drawn = pmi_present(
        &PmiGeometry::Angular {
            vertex: centre,
            dir_a,
            dir_b,
            axis: [0.0, 0.0, 1.0],
            degrees,
        },
        label,
        &text.0,
        &style,
    );
    finish(dimension, drawn, degrees, text, &[])
}


