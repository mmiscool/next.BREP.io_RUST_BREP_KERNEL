//! PMI — Product and Manufacturing Information: the document's `pmi` block
//! (views that own annotations), the annotation type table with its schemas
//! and selection predicates, and the history tail that resolves every
//! annotation against the just-built scene into a typed [`PmiReport`].
//!
//! # The `pmi` block
//!
//! `{ views: [{ id, name, camera?, display, annotations: [...] }], idCounter }`
//! on the history request, round-tripped through the saved document and
//! ABSENT on every document without PMI (so those files re-serialize
//! byte-identically). A **view** is a named camera snapshot plus display
//! state (text size, wireframe, hidden object names) plus an ordered list of
//! annotations; a saved view maps to an AP242 `DRAUGHTING_MODEL` +
//! `CAMERA_MODEL_D3` on export and is the unit a future drawing sheet places.
//!
//! An **annotation** is `{ type, enabled, inputParams, labelWorld? }`: its
//! params follow a kernel schema exactly like a feature's or an assembly
//! constraint's (`inputParams.id` is its id, `{SHORT}{n}` minted from the
//! block's counter), so the app's dialog engine draws every annotation form
//! from [`pmi_schema_catalogue`] and nothing is hand-written. Geometry is
//! referenced by kernel entity NAME (faces / edges, `{solid}@x,y,z` vertices
//! in WORLD coordinates, D/P frame names) — plain and component-owned
//! geometry alike — so a rebuilt model re-resolves every value and a renamed
//! or deleted entity puts the annotation into an error status without ever
//! aborting the view or the run.
//!
//! Every drawn annotation has an **annotation plane**: its optional `plane`
//! reference names a planar face or a reference plane the annotation lies
//! IN (label projected onto it, drags stay in it, the dimension / extension
//! lines and frames drawn in it, the AP242 `ANNOTATION_PLANE` placed on it);
//! left empty, the annotation aligns to the view camera. A picked plane must
//! be parallel to what it carries — a linear dimension's direction, an
//! angle's or a circle's plane — or the annotation reports an error rather
//! than a foreshortened value. Drawing sheets will show, per sheet view, the
//! annotations whose plane is parallel to the view.
//!
//! # The tail
//!
//! [`finish_history_run`] runs after the wire-harness tail on every run and
//! resolves EVERY view's annotations (a few name lookups each) into a
//! [`PmiReport`]: per annotation the status, the display text (value +
//! tolerance block, kernel-formatted so the viewport and the STEP file agree),
//! the measured value, the resolved analytic geometry the presentation is laid
//! out from ([`layout`]) and the label position (stored, or a default derived
//! from the geometry). The report rides `HistoryResult.pmi` (never the JSON
//! ABI) and the render crate's scene report across the runner seam. Nothing
//! here touches the incremental feature cache: PMI is annotation state, not
//! history, and a label drag never re-executes a feature.
//!
//! # Measurement contracts (kernel queries only)
//!
//! Every value is read from the exact BREP through [`crate::assembly_resolve`]
//! (planes, carrier lines, circles, axes, points) — never from tessellation.
//! Linear: point–point, point–line (perpendicular), line–line (carrier lines:
//! parallel spacing or closest points), point–plane, parallel plane spacing,
//! single straight edge length, optional X/Y/Z component. Radial: cylinder /
//! sphere / circular edge radius from the face metadata. Angle: the two
//! elements' directions folded to acute / obtuse / reflex, optionally
//! reversed. Hole callout: the owning Hole feature's parameters. Datum letters
//! are unique per part; a feature control frame's datum references must name
//! defined datums.

pub mod annotations;
pub mod font;
pub mod layout;
pub mod resolve;

use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

use crate::feature_pipeline::{Env, HistoryRequest, SceneMap};

pub use annotations::{pmi_schema_catalogue, pmi_type, PmiTypeDef, PMI_TYPES};

// ===========================================================================
// The persisted block
// ===========================================================================

/// The document's `pmi` block.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PmiState {
    #[serde(default)]
    pub views: Vec<PmiView>,
    /// Mints every view AND annotation id (`VIEW{n}`, `DIM{n}`, …); monotonic,
    /// never reused, re-seeded from the largest numeric suffix on load.
    #[serde(default, rename = "idCounter")]
    pub id_counter: u64,
}

/// A PMI view: a camera snapshot, display state and the annotations it owns.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PmiView {
    pub id: String,
    #[serde(default)]
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub camera: Option<PmiCamera>,
    #[serde(default)]
    pub display: PmiDisplay,
    #[serde(default)]
    pub annotations: Vec<PmiAnnotation>,
}

/// The camera snapshot: eye / target / up / projection / viewport size — the
/// render crate's `ViewCamera` minus near/far, which are a per-frame depth
/// window and never persisted. Applying a snapshot under another viewport
/// aspect refits the frustum, preserving the apparent size at the target.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PmiCamera {
    pub eye: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    pub projection: PmiProjection,
    /// `[width, height]` of the viewport the snapshot was taken in (CSS px).
    #[serde(default = "default_viewport")]
    pub viewport: [f64; 2],
}

fn default_viewport() -> [f64; 2] {
    [1280.0, 800.0]
}

impl PmiCamera {
    /// The unit viewing direction (eye → target).
    pub fn view_direction(&self) -> [f64; 3] {
        let d = [
            self.target[0] - self.eye[0],
            self.target[1] - self.eye[1],
            self.target[2] - self.eye[2],
        ];
        let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if len < 1e-12 {
            [0.0, 0.0, -1.0]
        } else {
            [d[0] / len, d[1] / len, d[2] / len]
        }
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum PmiProjection {
    Orthographic {
        #[serde(rename = "halfHeight")]
        half_height: f64,
    },
    Perspective {
        #[serde(rename = "fovYDeg")]
        fov_y_deg: f64,
    },
}

/// Per-view display state.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PmiDisplay {
    /// Label text size in points, clamped to `[1, 288]`.
    #[serde(default = "default_text_size", rename = "textSizePt")]
    pub text_size_pt: f64,
    #[serde(default)]
    pub wireframe: bool,
    /// Scene object NAMES hidden in this view (resolved to runtime visibility
    /// on apply; a name that no longer exists is ignored).
    #[serde(default)]
    pub hidden: Vec<String>,
}

fn default_text_size() -> f64 {
    12.0
}

impl Default for PmiDisplay {
    fn default() -> Self {
        Self {
            text_size_pt: default_text_size(),
            wireframe: false,
            hidden: Vec::new(),
        }
    }
}

/// The text size clamp: `[1, 288]` points.
pub fn clamp_text_size(size: f64) -> f64 {
    if !size.is_finite() {
        return default_text_size();
    }
    size.clamp(1.0, 288.0)
}

/// One annotation: schema-driven params keyed by its `type`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PmiAnnotation {
    #[serde(rename = "type")]
    pub kind: String,
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default, rename = "inputParams")]
    pub params: Value,
    /// The draggable label's world anchor. Absent until the user moves it —
    /// the report then carries a default derived from the geometry.
    #[serde(default, rename = "labelWorld", skip_serializing_if = "Option::is_none")]
    pub label_world: Option<[f64; 3]>,
}

fn default_true() -> bool {
    true
}

impl PmiAnnotation {
    /// `inputParams.id` (empty when absent).
    pub fn id(&self) -> &str {
        self.params
            .get("id")
            .and_then(Value::as_str)
            .unwrap_or("")
    }

    /// A string param, trimmed (`""` when absent or not a string).
    pub fn text(&self, key: &str) -> &str {
        self.params
            .get(key)
            .and_then(Value::as_str)
            .map(str::trim)
            .unwrap_or("")
    }

    /// A boolean param (`false` when absent).
    pub fn flag(&self, key: &str) -> bool {
        self.params
            .get(key)
            .and_then(Value::as_bool)
            .unwrap_or(false)
    }

    /// A numeric param: a JSON number, or a string evaluated as an expression
    /// against the document's variable sheet. `default` when absent / empty.
    pub fn number(&self, key: &str, env: &Env, default: f64) -> Result<f64, String> {
        match self.params.get(key) {
            None | Some(Value::Null) => Ok(default),
            Some(Value::Number(number)) => Ok(number.as_f64().unwrap_or(default)),
            Some(Value::Bool(flag)) => Ok(if *flag { 1.0 } else { 0.0 }),
            Some(Value::String(source)) => {
                let source = source.trim();
                if source.is_empty() {
                    return Ok(default);
                }
                env.eval(source)
                    .map_err(|error| format!("{key}: {error}"))
            }
            Some(other) => Err(format!("{key}: expected a number, got {other}")),
        }
    }

    /// The `plane` reference (a planar face / reference plane name), when
    /// the annotation is laid out in a picked plane rather than the view.
    pub fn plane_ref(&self) -> Option<&str> {
        let name = self.text("plane").trim();
        (!name.is_empty()).then_some(name)
    }

    /// A reference param as a name list: a string, or an array of strings
    /// (empty strings dropped).
    pub fn references(&self, key: &str) -> Vec<String> {
        match self.params.get(key) {
            Some(Value::String(name)) => {
                let name = name.trim();
                if name.is_empty() {
                    Vec::new()
                } else {
                    vec![name.to_string()]
                }
            }
            Some(Value::Array(items)) => items
                .iter()
                .filter_map(Value::as_str)
                .map(str::trim)
                .filter(|name| !name.is_empty())
                .map(String::from)
                .collect(),
            _ => Vec::new(),
        }
    }
}

impl PmiState {
    /// Mint the next id with `prefix` (`VIEW` → `VIEW3`). The counter is
    /// re-seeded first from the largest numeric suffix already in the block,
    /// so an edited or merged document never hands out a colliding id.
    pub fn next_id(&mut self, prefix: &str) -> String {
        let seen = self.max_numeric_suffix();
        if seen > self.id_counter {
            self.id_counter = seen;
        }
        loop {
            self.id_counter += 1;
            let candidate = format!("{prefix}{}", self.id_counter);
            if self.find_view(&candidate).is_none() && self.find_annotation(&candidate).is_none() {
                return candidate;
            }
        }
    }

    fn max_numeric_suffix(&self) -> u64 {
        let mut best = 0u64;
        let mut consider = |id: &str| {
            let digits = id
                .bytes()
                .rev()
                .take_while(u8::is_ascii_digit)
                .count();
            if digits > 0 {
                if let Ok(value) = id[id.len() - digits..].parse::<u64>() {
                    best = best.max(value);
                }
            }
        };
        for view in &self.views {
            consider(&view.id);
            for annotation in &view.annotations {
                consider(annotation.id());
            }
        }
        best
    }

    pub fn find_view(&self, id: &str) -> Option<&PmiView> {
        self.views.iter().find(|view| view.id == id)
    }

    pub fn find_view_mut(&mut self, id: &str) -> Option<&mut PmiView> {
        self.views.iter_mut().find(|view| view.id == id)
    }

    /// The annotation with `id` and its owning view.
    pub fn find_annotation(&self, id: &str) -> Option<(&PmiView, &PmiAnnotation)> {
        self.views.iter().find_map(|view| {
            view.annotations
                .iter()
                .find(|annotation| annotation.id() == id)
                .map(|annotation| (view, annotation))
        })
    }

    /// The owning view id and index of annotation `id`.
    pub fn locate_annotation(&self, id: &str) -> Option<(usize, usize)> {
        self.views.iter().enumerate().find_map(|(view_index, view)| {
            view.annotations
                .iter()
                .position(|annotation| annotation.id() == id)
                .map(|index| (view_index, index))
        })
    }

    pub fn find_annotation_mut(&mut self, id: &str) -> Option<&mut PmiAnnotation> {
        self.views.iter_mut().find_map(|view| {
            view.annotations
                .iter_mut()
                .find(|annotation| annotation.id() == id)
        })
    }

    /// Every datum letter defined in the part (any view), in definition
    /// order, with the defining annotation's id — the part-level datum
    /// registry a feature control frame validates its references against.
    pub fn datum_letters(&self) -> Vec<(String, String)> {
        let mut out = Vec::new();
        for view in &self.views {
            for annotation in &view.annotations {
                if annotation.kind == annotations::datum::DEF.type_id {
                    let letter = annotation.text("letter").to_uppercase();
                    if !letter.is_empty() {
                        out.push((letter, annotation.id().to_string()));
                    }
                }
            }
        }
        out
    }

    /// The next unused datum letter: A–Z skipping I, O and Q (ASME Y14.5),
    /// then AA, AB, … (`None` only past ZZ).
    pub fn next_datum_letter(&self) -> Option<String> {
        let used: Vec<String> = self.datum_letters().into_iter().map(|(l, _)| l).collect();
        let alphabet: Vec<char> = ('A'..='Z').filter(|c| !matches!(c, 'I' | 'O' | 'Q')).collect();
        for letter in &alphabet {
            let candidate = letter.to_string();
            if !used.contains(&candidate) {
                return Some(candidate);
            }
        }
        for first in &alphabet {
            for second in &alphabet {
                let candidate = format!("{first}{second}");
                if !used.contains(&candidate) {
                    return Some(candidate);
                }
            }
        }
        None
    }

    /// Whether the block carries anything worth persisting.
    pub fn is_empty(&self) -> bool {
        self.views.is_empty() && self.id_counter == 0
    }
}

// ===========================================================================
// The resolved report
// ===========================================================================

/// The tail's resolution of every view's annotations.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PmiReport {
    pub views: Vec<PmiViewReport>,
}

impl PmiReport {
    pub fn view(&self, id: &str) -> Option<&PmiViewReport> {
        self.views.iter().find(|view| view.id == id)
    }

    pub fn annotation(&self, id: &str) -> Option<&PmiAnnotationReport> {
        self.views
            .iter()
            .find_map(|view| view.annotations.iter().find(|a| a.id == id))
    }

    pub fn annotation_mut(&mut self, id: &str) -> Option<&mut PmiAnnotationReport> {
        self.views
            .iter_mut()
            .find_map(|view| view.annotations.iter_mut().find(|a| a.id == id))
    }
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PmiViewReport {
    pub id: String,
    pub annotations: Vec<PmiAnnotationReport>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PmiStatus {
    /// Resolved: `text` / `value` / `geometry` are live.
    Ok,
    /// An unresolved anchor, a dangling datum reference, an unsupported
    /// pairing …: `message` says which. The annotation stays listed.
    Error,
}

/// One resolved annotation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PmiAnnotationReport {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    pub enabled: bool,
    pub status: PmiStatus,
    #[serde(default)]
    pub message: String,
    /// The display text (value + tolerance block, note / leader text, hole
    /// callout, datum letter, FCF cells joined).
    #[serde(default)]
    pub text: String,
    /// The measured value (length in model units, angle in degrees).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub value: Option<f64>,
    /// `"mm"` / `"deg"` / `""`.
    #[serde(default)]
    pub unit: String,
    /// The scene entity names the annotation resolved (for hover
    /// highlighting and the STEP shape aspects), in reference order.
    #[serde(default)]
    pub references: Vec<String>,
    /// The label anchor: the stored `labelWorld`, else the geometry's default
    /// — projected onto the annotation plane when one is picked.
    #[serde(rename = "labelWorld")]
    pub label_world: [f64; 3],
    /// The picked annotation plane; `None` aligns to the view camera.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plane: Option<PmiPlane>,
    pub geometry: PmiGeometry,
}

/// A resolved annotation plane: the plane the annotation lies in (its
/// `plane` reference — a planar face or a reference plane), `normal`
/// oriented toward the view's captured camera, `x_axis` the in-plane text
/// direction (the camera's right projected into the plane). The viewport
/// overlay and the AP242 `ANNOTATION_PLANE` both lay the annotation out in
/// this frame.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PmiPlane {
    pub origin: [f64; 3],
    pub normal: [f64; 3],
    #[serde(rename = "xAxis")]
    pub x_axis: [f64; 3],
}

impl PmiPlane {
    /// `point` projected onto the plane along its normal.
    pub fn project(&self, point: [f64; 3]) -> [f64; 3] {
        let n = self.normal;
        let d = (point[0] - self.origin[0]) * n[0] + (point[1] - self.origin[1]) * n[1] + (point[2] - self.origin[2]) * n[2];
        [point[0] - n[0] * d, point[1] - n[1] * d, point[2] - n[2] * d]
    }

    /// The in-plane up direction (`normal × x_axis`).
    pub fn y_axis(&self) -> [f64; 3] {
        let n = self.normal;
        let x = self.x_axis;
        [n[1] * x[2] - n[2] * x[1], n[2] * x[0] - n[0] * x[2], n[0] * x[1] - n[1] * x[0]]
    }

    /// The ray `origin + t·dir` hit on the plane, if not parallel.
    pub fn hit(&self, origin: [f64; 3], dir: [f64; 3]) -> Option<[f64; 3]> {
        let n = self.normal;
        let denominator = dir[0] * n[0] + dir[1] * n[1] + dir[2] * n[2];
        if denominator.abs() < 1e-12 {
            return None;
        }
        let diff = [self.origin[0] - origin[0], self.origin[1] - origin[1], self.origin[2] - origin[2]];
        let t = (diff[0] * n[0] + diff[1] * n[1] + diff[2] * n[2]) / denominator;
        Some([origin[0] + dir[0] * t, origin[1] + dir[1] * t, origin[2] + dir[2] * t])
    }
}

/// The analytic geometry an annotation resolved to — what the presentation
/// ([`layout`]) and the STEP semantic export are built from.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum PmiGeometry {
    None,
    /// Measured from `a` to `b` (world). `component` names the X/Y/Z axis
    /// when the dimension is an aligned component, else `None`.
    Linear {
        a: [f64; 3],
        b: [f64; 3],
        #[serde(default, skip_serializing_if = "Option::is_none")]
        component: Option<char>,
    },
    Radial {
        center: [f64; 3],
        axis: [f64; 3],
        radius: f64,
        diameter: bool,
        sphere: bool,
    },
    /// The arc sweeps `degrees` from `dir_a` about `axis` (right-handed) at
    /// `vertex`.
    Angular {
        vertex: [f64; 3],
        #[serde(rename = "dirA")]
        dir_a: [f64; 3],
        #[serde(rename = "dirB")]
        dir_b: [f64; 3],
        axis: [f64; 3],
        degrees: f64,
    },
    Leader {
        targets: Vec<[f64; 3]>,
        dot: bool,
    },
    Note {
        position: [f64; 3],
    },
    Hole {
        anchor: [f64; 3],
        normal: [f64; 3],
    },
    /// Display-only: pose `solids` by the delta while the view is active.
    Explode {
        solids: Vec<String>,
        translate: [f64; 3],
        #[serde(rename = "rotateDeg")]
        rotate_deg: [f64; 3],
        scale: [f64; 3],
        /// The rotation / scale pivot: the targets' aggregate bbox center.
        center: [f64; 3],
        trace: bool,
    },
    Datum {
        anchor: [f64; 3],
        normal: [f64; 3],
        letter: String,
    },
    Fcf {
        anchor: [f64; 3],
        normal: [f64; 3],
        frame: FcfFrame,
    },
}

/// The cells of a feature control frame.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FcfFrame {
    /// The characteristic id (`flatness`, `position`, …).
    pub characteristic: String,
    /// Its Unicode symbol (the glyph key).
    pub symbol: String,
    /// The tolerance cell text (`⌀0.1 Ⓜ`).
    pub zone: String,
    /// The datum reference cells (`A`, `B Ⓜ`).
    pub datums: Vec<String>,
}

// ===========================================================================
// Value formatting (shared by the viewport and the STEP presentation)
// ===========================================================================

/// The tolerance block of a dimension: `tolMode` none / symmetric / deviation
/// / limits with `tolUpper` / `tolLower`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceBlock {
    pub mode: ToleranceMode,
    pub upper: f64,
    pub lower: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ToleranceMode {
    None,
    Symmetric,
    Deviation,
    Limits,
}

impl ToleranceMode {
    pub fn parse(text: &str) -> Self {
        match text.trim().to_ascii_lowercase().as_str() {
            "symmetric" => ToleranceMode::Symmetric,
            "deviation" => ToleranceMode::Deviation,
            "limits" => ToleranceMode::Limits,
            _ => ToleranceMode::None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ToleranceMode::None => "none",
            ToleranceMode::Symmetric => "symmetric",
            ToleranceMode::Deviation => "deviation",
            ToleranceMode::Limits => "limits",
        }
    }
}

impl ToleranceBlock {
    /// Read the block off an annotation's params.
    pub fn read(annotation: &PmiAnnotation, env: &Env) -> Result<Self, String> {
        Ok(Self {
            mode: ToleranceMode::parse(annotation.text("tolMode")),
            upper: annotation.number("tolUpper", env, 0.0)?.abs(),
            lower: annotation.number("tolLower", env, 0.0)?.abs(),
        })
    }

    /// The signed bounds as offsets from the nominal (`lower ≤ 0 ≤ upper`),
    /// `None` when the mode carries no tolerance.
    pub fn bounds(&self) -> Option<(f64, f64)> {
        match self.mode {
            ToleranceMode::None => None,
            ToleranceMode::Symmetric => Some((-self.upper, self.upper)),
            ToleranceMode::Deviation | ToleranceMode::Limits => Some((-self.lower, self.upper)),
        }
    }
}

/// `value` with `decimals` places (`0..=8`), no exponent.
pub fn format_number(value: f64, decimals: usize) -> String {
    let decimals = decimals.min(8);
    let text = format!("{:.*}", decimals, value);
    // `-0.000` reads as a sign error on a dimension.
    if text.starts_with('-') && text[1..].bytes().all(|b| b == b'0' || b == b'.') {
        text[1..].to_string()
    } else {
        text
    }
}

/// The dimension text: `prefix` (`⌀` / `R` / `""`) + the nominal (+ the
/// tolerance block) + `suffix` (`°` / `""`); a reference dimension is
/// parenthesized and shows no tolerance.
pub fn format_dimension(
    value: f64,
    decimals: usize,
    tolerance: &ToleranceBlock,
    is_reference: bool,
    prefix: &str,
    suffix: &str,
) -> String {
    let nominal = format!("{prefix}{}{suffix}", format_number(value, decimals));
    if is_reference {
        return format!("({nominal})");
    }
    match tolerance.mode {
        ToleranceMode::None => nominal,
        ToleranceMode::Symmetric => format!(
            "{nominal} \u{00B1}{}{suffix}",
            format_number(tolerance.upper, decimals)
        ),
        ToleranceMode::Deviation => format!(
            "{nominal} +{}{suffix}/\u{2212}{}{suffix}",
            format_number(tolerance.upper, decimals),
            format_number(tolerance.lower, decimals)
        ),
        ToleranceMode::Limits => format!(
            "{prefix}{}{suffix} / {prefix}{}{suffix}",
            format_number(value + tolerance.upper, decimals),
            format_number(value - tolerance.lower, decimals)
        ),
    }
}

// ===========================================================================
// The history tail
// ===========================================================================

/// What the annotation resolvers read: the live scene, the request (hole
/// features for callouts, the datum registry) and the expression sheet.
pub struct PmiContext<'a> {
    pub scene: &'a SceneMap,
    pub request: &'a HistoryRequest,
    pub env: &'a Env,
    /// The part-level datum registry (letter → annotation id).
    pub datums: BTreeMap<String, String>,
}

/// The outcome of one annotation's resolver.
pub struct Resolved {
    pub text: String,
    pub value: Option<f64>,
    pub unit: &'static str,
    pub references: Vec<String>,
    pub geometry: PmiGeometry,
    /// The default label anchor when the annotation stores none.
    pub default_label: [f64; 3],
}

/// Resolve every view's annotations against `scene`. `None` when the request
/// carries no `pmi` block (a document without PMI reports nothing).
pub(crate) fn finish_history_run(
    request: &HistoryRequest,
    scene: &SceneMap,
    env: &Env,
) -> Option<PmiReport> {
    let state = request.pmi.as_ref()?;
    Some(resolve_state(state, scene, request, env))
}

/// Resolve `state` (the tail's body, callable on any scene).
pub fn resolve_state(
    state: &PmiState,
    scene: &SceneMap,
    request: &HistoryRequest,
    env: &Env,
) -> PmiReport {
    // The datum registry: first definition of a letter wins; a duplicate is
    // reported on the later annotation by the datum resolver.
    let mut datums: BTreeMap<String, String> = BTreeMap::new();
    for (letter, id) in state.datum_letters() {
        datums.entry(letter).or_insert(id);
    }
    let context = PmiContext {
        scene,
        request,
        env,
        datums,
    };
    PmiReport {
        views: state
            .views
            .iter()
            .map(|view| PmiViewReport {
                id: view.id.clone(),
                annotations: view
                    .annotations
                    .iter()
                    .map(|annotation| resolve_annotation(annotation, &context, view.camera.as_ref()))
                    .collect(),
            })
            .collect(),
    }
}

/// Resolve one annotation through its type's resolver into a report row:
/// the type's measurement, then its annotation plane (`camera` orients the
/// plane toward the view; `None` for a view without a camera).
pub fn resolve_annotation(
    annotation: &PmiAnnotation,
    context: &PmiContext<'_>,
    camera: Option<&PmiCamera>,
) -> PmiAnnotationReport {
    let outcome = match pmi_type(&annotation.kind) {
        Some(def) => (def.resolve)(annotation, context),
        None => Err(format!("unknown PMI annotation type '{}'", annotation.kind)),
    }
    .and_then(|resolved| {
        let plane = annotation_plane(annotation, context, camera, &resolved.geometry)?;
        Ok((resolved, plane))
    });
    match outcome {
        Ok((resolved, plane)) => {
            let label = annotation.label_world.unwrap_or(resolved.default_label);
            PmiAnnotationReport {
                id: annotation.id().to_string(),
                kind: annotation.kind.clone(),
                enabled: annotation.enabled,
                status: PmiStatus::Ok,
                message: String::new(),
                text: resolved.text,
                value: resolved.value,
                unit: resolved.unit.to_string(),
                references: resolved.references,
                label_world: plane.map_or(label, |plane| plane.project(label)),
                plane,
                geometry: resolved.geometry,
            }
        }
        Err(message) => PmiAnnotationReport {
            id: annotation.id().to_string(),
            kind: annotation.kind.clone(),
            enabled: annotation.enabled,
            status: PmiStatus::Error,
            message,
            text: String::new(),
            value: None,
            unit: String::new(),
            references: Vec::new(),
            label_world: annotation.label_world.unwrap_or([0.0; 3]),
            plane: None,
            geometry: PmiGeometry::None,
        },
    }
}

/// Resolve an annotation's `plane` reference into its [`PmiPlane`]. The
/// reference must be a planar face or a reference plane, and the plane must
/// be parallel to what it carries — a linear dimension's direction, an
/// angle's plane, a circle's plane — since a dimension line drawn in a
/// non-parallel plane would be foreshortened and misstate the value. The
/// normal faces the view's camera and the text direction follows the
/// camera's right. `Ok(None)` when the annotation aligns to the view.
fn annotation_plane(
    annotation: &PmiAnnotation,
    context: &PmiContext<'_>,
    camera: Option<&PmiCamera>,
    geometry: &PmiGeometry,
) -> Result<Option<PmiPlane>, String> {
    use crate::SelectionGeometry;
    use resolve::{a3, perpendicular_in_plane, v3};
    let Some(name) = annotation.plane_ref() else {
        return Ok(None);
    };
    let (origin, normal) = match resolve::resolve_reference(context.scene, name)? {
        SelectionGeometry::Plane { origin, normal } => (origin, normal),
        _ => return Err(format!("annotation plane '{name}' must be a planar face or a reference plane")),
    };
    let mut normal = normal
        .normalized()
        .map_err(|_| format!("annotation plane '{name}' has no normal"))?;
    // sin of the largest tolerated tilt (~0.06°).
    const PARALLEL: f64 = 1e-3;
    match geometry {
        PmiGeometry::Linear { a, b, .. } => {
            let span = v3(*b).sub(v3(*a));
            if span.length() > 1e-9 && span.normalized().map(|d| d.dot(normal).abs()).unwrap_or(0.0) > PARALLEL {
                return Err(format!("annotation plane '{name}' is not parallel to the measured direction"));
            }
        }
        PmiGeometry::Angular { axis, .. } => {
            if v3(*axis).cross(normal).length() > PARALLEL {
                return Err(format!("annotation plane '{name}' is not parallel to the angle's plane"));
            }
        }
        PmiGeometry::Radial { axis, sphere: false, .. } => {
            if v3(*axis).cross(normal).length() > PARALLEL {
                return Err(format!("annotation plane '{name}' is not parallel to the circle's plane"));
            }
        }
        _ => {}
    }
    let x_axis = match camera {
        Some(camera) => {
            let view = v3(camera.view_direction());
            if normal.dot(view) > 0.0 {
                normal = normal.scale(-1.0);
            }
            perpendicular_in_plane(normal, view.cross(v3(camera.up)))
        }
        None => perpendicular_in_plane(normal, crate::Vec3::new(1.0, 0.0, 0.0)),
    };
    Ok(Some(PmiPlane {
        origin: a3(origin),
        normal: a3(normal),
        x_axis: a3(x_axis),
    }))
}

