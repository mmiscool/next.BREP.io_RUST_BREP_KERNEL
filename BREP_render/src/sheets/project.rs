//! Projecting a placed PMI view onto its sheet: the model as edge LINES with
//! hidden lines removed, plus EVERY annotation of the saved view, all in
//! paper millimetres.
//!
//! # Annotations
//!
//! An annotation is laid out through the same [`pmi_present`] the 3D overlay
//! and the STEP presentation use — in its own annotation plane when it has
//! one — and the result is projected through the view camera exactly as the
//! overlay draws it. There is no plane filter: a plane oblique to the view
//! draws foreshortened, a plane EDGE-ON to it draws as a line and the row
//! says so ([`AnnotationDrawing::note`]). Circles and arcs in the plane are
//! polylines before they are projected, so they come out as the ellipses any
//! parallel projection gives. Annotations are NOT hidden-line tested: they
//! are drawn on top of the model, which is what the viewport overlay does and
//! what a drawing wants — a dimension behind the part is still a dimension.
//!
//! # The model lines
//!
//! Drawn by the ANALYTIC hidden-line pass, [`super::hlr`]: candidate curves
//! from the B-rep (edges and the silhouettes of curved faces), split where
//! they cross in the projection plane, each piece shown or hidden by an exact
//! depth comparison against the faces in front of it. A section cuts the
//! B-rep with its plane exactly ([`super::hlr::section_curves`]) and a detail
//! decides visibility inside its circle's cylinder at its own scale. The
//! result is cached per placement by what it depends on — the geometry, the
//! camera, the scale and the clip — and never by where the placement sits on
//! the paper, so a drag does not re-run it.
//!
//! # Before the B-rep arrives
//!
//! The exact pass needs every drawn solid's B-rep, which the scene PULLS from
//! the runner when a sheet first shows it. Until it lands — and for a handle
//! the registry could not clone — the placement is drawn by the same pass on
//! the DISPLAY instead ([`super::hlr::visible_mesh_lines`]): the display edges
//! and the mesh's silhouettes, split and decided against the display
//! triangles. That is a stated approximation, never a silent one:
//! [`ViewDrawing::hidden_line`] says `mesh`, and the PDF and SVG writers carry
//! it into the files they write.

use crate::scene::RenderScene;
use brep_kernel::{
    pmi_present, PmiAnnotationReport, PmiCamera, PmiLayoutStyle, PmiProjection, PmiStatus,
    PmiView, PmiViewReport,
};

use super::{PlacedView, ORTHOGRAPHIC};

/// Dihedral threshold for a drawn edge, degrees — `EdgesGeometry(geo, 30)`,
/// the value `BREP_mcp_core::render_mesh` matches three.js on. The analytic
/// pass reads the same bar off the exact face normals along a B-rep edge.
pub const EDGE_ANGLE_DEG: f64 = 30.0;

/// Points per millimetre: a PMI view's text size is in POINTS, and a sheet is
/// in millimetres.
pub const MM_PER_POINT: f64 = 25.4 / 72.0;

/// The arrowhead length as a multiple of the text cap height — the viewport
/// overlay's own proportion (12 px arrow against an 11 px cap at 12 pt), so an
/// annotation on paper has the arrowheads it has on screen.
pub(super) const ARROW_PER_TEXT: f64 = 12.0 / 11.0;

/// The PMI stroke font's monospace advance as a multiple of the cap height
/// (the kernel's `font::ADVANCE`, mirrored because the kernel does not publish
/// its font metrics). The sheet renders the text with a real proportional
/// face, which is narrower, so a box measured with this is slightly generous —
/// which is the right way round for a clearance.
pub(super) const TEXT_ADVANCE: f64 = 0.75;

/// Clear space kept around a text run before a line is broken for it, as a
/// multiple of the cap height.
const TEXT_PAD: f64 = 0.3;

/// A section's HATCH spacing, PAPER millimetres. A hatch is a drawing
/// convention rather than a property of the model, so it is a paper distance
/// at every sheet scale — a 2:1 section is hatched at the same pitch as a 1:1
/// one, which is what makes two sections of a drawing read as one drawing.
pub const HATCH_SPACING_MM: f64 = 2.5;

/// The most hatch lines one cut face may draw. A section of a part a metre
/// across at 1:1 would otherwise ask for four hundred thousand of them; past
/// this the hatch is THINNED (the spacing doubles until it fits) rather than
/// refused, because a thinned hatch still says "this is cut material" and an
/// absent one says nothing.
const MAX_HATCH_LINES: usize = 2000;

/// When an annotation plane is EDGE-ON to the view: the cosine of the angle
/// between the plane's normal and the viewing direction. Below this the
/// plane projects to a line, and so does everything laid out in it. The bar
/// is the kernel's own (`annotation_plane` accepts a plane against its
/// geometry at `sin < 1e-3`), read on the other axis.
pub const EDGE_ON_COS: f64 = 1e-3;

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------

/// A whole sheet, projected — the sheet viewport draws this and the SVG and PDF
/// writers serialise it.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SheetDrawing {
    pub id: String,
    pub name: String,
    #[serde(rename = "widthMm")]
    pub width_mm: f64,
    #[serde(rename = "heightMm")]
    pub height_mm: f64,
    pub views: Vec<ViewDrawing>,
    /// The sheet's OWN dimensions — authored on the paper, anchored to the
    /// placements' projected geometry rather than to a saved PMI view. They
    /// live beside the views rather than inside one because a dimension is the
    /// SHEET's (see [`super::dimension`]).
    pub dimensions: Vec<super::dimension::DimensionDrawing>,
    /// The sheet's ORDINATE SETS, laid out ([`super::ordinate`]). Beside the
    /// dimensions for the same reason they are beside them in the block: a set
    /// is its own object, not a dimension of a fifth kind.
    pub ordinates: Vec<super::ordinate::OrdinateDrawing>,
    /// The paper's own furniture — the border and the title block — in the same
    /// millimetres and the same lines-and-text as the views. `None` on a sheet
    /// that draws neither. It is built by the ENGINE rather than here
    /// ([`crate::sheets::frame::frame`]): a title block carries the document's
    /// name and the date, which a projection knows nothing about.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub frame: Option<super::frame::FrameDrawing>,
}

/// One placed view, projected. Paper millimetres throughout, measured from the
/// paper's TOP-LEFT corner with y running DOWN — SVG's own frame, so the
/// writer transforms nothing.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct ViewDrawing {
    /// The placed view's id (`SV2`).
    pub id: String,
    /// The saved PMI view it places (`VIEW1`).
    pub view: String,
    /// The saved view's name, for the panel and the SVG group label.
    pub name: String,
    pub position: [f64; 2],
    pub scale: f64,
    /// Empty when the view projected; the reason it did not otherwise (no such
    /// saved view, no captured camera, a perspective camera).
    #[serde(skip_serializing_if = "String::is_empty")]
    pub error: String,
    /// The model as VISIBLE edge lines, one polyline per visible run.
    pub edges: Vec<Vec<[f64; 2]>>,
    /// Every enabled, resolved annotation of the saved view, projected.
    pub annotations: Vec<AnnotationDrawing>,
    /// Every point of this projection a SHEET dimension can be anchored to —
    /// each named vertex, each named edge's two ends and its midpoint, and the
    /// centre of each circular edge. The anchor space is the MODEL's topology,
    /// so the hidden-line pass has no vote on it: a candidate whose ink the
    /// pass removed is still here, and a dimension anchored to it still draws.
    pub anchors: Vec<AnchorPoint>,
    /// The saved view's text size in PAPER millimetres — what a sheet
    /// dimension on this placement is drawn at, so it reads at the size of the
    /// annotations it sits among.
    #[serde(rename = "textMm")]
    pub text_mm: f64,
    /// This placement IS a section view: the cut face's hatch and its caption
    /// ([`SectionDrawing`]). `None` on a plain placement.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub section: Option<SectionDrawing>,
    /// The section LINES drawn on THIS placement — one per section view cut
    /// through it. A plain placement that nothing sections has none.
    #[serde(rename = "sectionMarks", skip_serializing_if = "Vec::is_empty")]
    pub section_marks: Vec<SectionMark>,
    /// This placement IS a detail view: its boundary circle and its caption
    /// ([`DetailDrawing`]). `None` on every other placement.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub detail: Option<DetailDrawing>,
    /// The detail CIRCLES drawn on THIS placement — one per detail view drawn
    /// from it.
    #[serde(rename = "detailMarks", skip_serializing_if = "Vec::is_empty")]
    pub detail_marks: Vec<DetailMark>,
    /// `[x0, y0, x1, y1]` over everything drawn — empty geometry gives the
    /// placement point twice.
    pub bounds: [f64; 4],
    /// Which hidden-line pass drew [`Self::edges`]: `analytic` (the exact
    /// pass on the B-rep, [`super::hlr::visible_edges`]) or `mesh` (the same
    /// pass on the display, [`super::hlr::visible_mesh_lines`] — the stated
    /// approximation while a drawn solid's B-rep has not reached the scene).
    /// Empty on a placement that did not project.
    #[serde(rename = "hiddenLine", skip_serializing_if = "String::is_empty")]
    pub hidden_line: String,
    /// The saved view's PMI dimensions that carry a tolerance block (or read
    /// as a reference dimension), by what they MEASURE on this paper: what a
    /// sheet dimension anchored to the same points inherits
    /// ([`super::dimension`]). Engine-side only — the drawing does not
    /// publish them; a dimension that inherits says so on its own row.
    #[serde(skip)]
    pub tolerances: Vec<ToleranceSource>,
}

/// A PMI dimension of a placement's view with a tolerance block to hand on, by
/// what it measures on the paper.
#[derive(Debug, Clone, PartialEq)]
pub struct ToleranceSource {
    /// The annotation's id (`DIM4`).
    pub id: String,
    pub measures: Measures,
    pub block: brep_kernel::ToleranceBlock,
    pub reference: bool,
}

/// What a [`ToleranceSource`] measures, projected.
#[derive(Debug, Clone, PartialEq)]
pub enum Measures {
    /// A free linear dimension between the paper images of its two measured
    /// points. A component (X / Y / Z) row measures a different number than
    /// any sheet dimension between the same two points can.
    Linear { a: [f64; 2], b: [f64; 2] },
    /// A radial or diametral dimension of the circle centred at the paper
    /// image of `centre`, of MODEL radius `radius`.
    Radial { centre: [f64; 2], radius: f64, diameter: bool },
}

impl ViewDrawing {
    /// A placement with nothing drawn yet: its id, its position and a bounds
    /// box of the placement point, so a view that draws nothing still has a
    /// row and a grab rect on the paper.
    fn empty(placed: &PlacedView, name: String) -> ViewDrawing {
        ViewDrawing {
            id: placed.id.clone(),
            view: placed.view.clone(),
            name,
            position: placed.position,
            scale: placed.scale,
            error: String::new(),
            edges: Vec::new(),
            annotations: Vec::new(),
            anchors: Vec::new(),
            text_mm: 0.0,
            section: None,
            section_marks: Vec::new(),
            detail: None,
            detail_marks: Vec::new(),
            bounds: [placed.position[0], placed.position[1], placed.position[0], placed.position[1]],
            hidden_line: String::new(),
            tolerances: Vec::new(),
        }
    }
}

/// A detail placement's own marks: the circle its drawing is clipped to and
/// the `DETAIL B` / `SCALE 4:1` caption under it.
///
/// The model lines are not here — they are in [`ViewDrawing::edges`], already
/// clipped to [`Self::circle`].
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct DetailDrawing {
    pub label: String,
    /// The circle's centre on the paper — the placement's own position.
    pub centre: [f64; 2],
    /// The circle's radius on the paper: the model radius at this placement's
    /// scale.
    #[serde(rename = "radiusMm")]
    pub radius_mm: f64,
    /// The boundary, as a closed polyline.
    pub circle: Vec<[f64; 2]>,
    /// The caption: the letter, then the detail's own scale.
    pub texts: Vec<SheetText>,
}

/// One detail CIRCLE, drawn on the placement it was picked on: the circle
/// round the region the detail redraws, and its letter beside it.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct DetailMark {
    /// The DETAIL placement's id — which view this circle is the region of.
    pub id: String,
    pub label: String,
    pub centre: [f64; 2],
    #[serde(rename = "radiusMm")]
    pub radius_mm: f64,
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
}

/// A section placement's own marks: the hatch over its cut faces and the
/// `SECTION A-A` caption under the drawing.
///
/// The cut OUTLINE is not here — it is in [`ViewDrawing::edges`] with every
/// other model line, because the clip's new boundary edges go through the very
/// same weld, classify and chain pass the rest of the model does. One
/// extractor, so the ink and the hatch cannot disagree about where the cut is.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SectionDrawing {
    /// The section's letter.
    pub label: String,
    /// The cut face's hatch: parallel 45\u{00B0} lines clipped to the cut region by
    /// an even-odd crossing count against [`Self::cut`].
    pub hatch: Vec<Vec<[f64; 2]>>,
    /// The cut region's boundary, in paper millimetres: one polyline per
    /// trimmed piece of the plane's curve with a face, sampled from the exact
    /// curve and oriented consistently round the region (their shoelace sum
    /// is the cut area, the holes taken out). In the mesh approximation, one
    /// segment per display triangle the plane cuts instead. What the hatch was
    /// clipped to, and what a reader can check the hatched region against.
    pub cut: Vec<Vec<[f64; 2]>>,
    /// The `SECTION A-A` caption.
    pub texts: Vec<SheetText>,
}

/// One section LINE, drawn on the placement the cut was picked in: the line
/// through the two anchors, an arrow at each end pointing the way the section
/// looks, and the letter beside each arrow.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SectionMark {
    /// The SECTION placement's id — which view this line is the cut of.
    pub id: String,
    pub label: String,
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
}

/// One annotation, laid out at sheet scale.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct AnnotationDrawing {
    pub id: String,
    #[serde(rename = "type")]
    pub kind: String,
    /// The kernel-formatted value — in MODEL units, never divided by the
    /// sheet scale: a 10 mm feature drawn at 2:1 still reads `10.000`.
    pub text: String,
    /// Dimension / extension / leader lines, arrowhead outlines and frames.
    pub lines: Vec<Vec<[f64; 2]>>,
    pub texts: Vec<SheetText>,
    /// What the projection has to say about this row, empty when it has
    /// nothing: an annotation plane EDGE-ON to the view projects to a LINE,
    /// so the row's geometry is a line and — with `flattenText` off — its
    /// text falls on that line. Said out loud rather than dropped, which is
    /// what the first sheets slice did with every non-parallel plane.
    #[serde(skip_serializing_if = "String::is_empty")]
    pub note: String,
}

/// One point of a placement's projection a sheet dimension can anchor to.
///
/// `anchor` is the reference string ([`super::dimension::SheetAnchor`]) — the
/// name that survives a re-projection — and `at` is where it landed on the
/// paper THIS time. The sheet viewport publishes one hit rect per candidate
/// while the reference picker is up for a sheet object, and a dimension resolves its anchors by
/// looking its references up in this list: an anchor that is not here is a
/// LOST anchor, which is what marks its dimension unresolved.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct AnchorPoint {
    #[serde(rename = "ref")]
    pub anchor: String,
    /// `vertex`, `edge` or `circle`.
    pub kind: String,
    /// Paper millimetres.
    pub at: [f64; 2],
    /// The 3D point before projection, in model units, with this view's poses
    /// applied. Aligned dimensions measure here so an oblique camera cannot
    /// foreshorten their values; `at` remains the point used for layout.
    pub model: [f64; 3],
    /// The unit PAPER direction of a STRAIGHT display edge — an `edge`
    /// candidate only, and `None` on a curved edge or one that projects to a
    /// point (an edge parallel to the viewing direction). It is what an
    /// ANGULAR dimension measures between: the edge's direction as this view
    /// projects it, so a foreshortened angle reads its projected value.
    ///
    /// Straightness is judged in the MODEL, not on the paper: a circular edge
    /// seen edge-on projects to a straight segment and is not a straight edge.
    /// All three fractions of one edge carry the same direction — it is the
    /// EDGE's, not the point's.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dir: Option<[f64; 2]>,
    /// The circle's MODEL radius — a `circle` candidate only.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub radius: Option<f64>,
    /// The paper images of the circle's two conjugate radii: the projected
    /// circle is the ellipse `at + u cos t + v sin t`. A `circle` candidate
    /// only, and what a radial dimension's leader measures its reach along.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub u: Option<[f64; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub v: Option<[f64; 2]>,
}

/// A placed text run, centred on `anchor`.
///
/// The run is drawn in its OWN basis: [`SheetText::right`] and
/// [`SheetText::up`] are the paper images of ONE CAP HEIGHT along the run's
/// reading and up directions, in the drawing's millimetres (y DOWN, SVG's
/// frame). A run laid flat on the sheet has `right = [h, 0]` and
/// `up = [0, -h]`; a run projected out of an oblique annotation plane has a
/// pair that is neither perpendicular nor equal in length, because the
/// projection foreshortens and shears it, and the SVG and PDF writers carry
/// the pair as a text matrix.
///
/// `height` and `angle` are that basis SUMMARISED — the cap height the run
/// draws at (`|up|`) and the angle its baseline reads along, degrees
/// CLOCKWISE (SVG's positive direction). They describe a flat or merely
/// rotated run completely; the sheet viewport draws every run from them
/// because egui cannot shear a text shape, so an oblique run previews as its
/// rotation and its foreshortened height while the two files carry it exactly.
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct SheetText {
    pub text: String,
    pub anchor: [f64; 2],
    pub height: f64,
    pub angle: f64,
    pub right: [f64; 2],
    pub up: [f64; 2],
}

impl SheetText {
    /// A run from its projected basis; `height` and `angle` are derived here
    /// and nowhere else, so the summary cannot disagree with the basis.
    pub fn new(text: String, anchor: [f64; 2], right: [f64; 2], up: [f64; 2]) -> SheetText {
        SheetText {
            text,
            anchor,
            height: (up[0] * up[0] + up[1] * up[1]).sqrt(),
            // atan2(0, 0) is 0: an EDGE-ON run has no reading direction left
            // to report, and its row carries the note that says so.
            angle: right[1].atan2(right[0]).to_degrees(),
            right,
            up,
        }
    }

    /// A run laid FLAT on the sheet: reading left to right, `height`
    /// millimetres of cap height. The title block's cells and every run of a
    /// placement with `flattenText` on.
    pub fn flat(text: String, anchor: [f64; 2], height: f64) -> SheetText {
        SheetText::new(text, anchor, [height, 0.0], [0.0, -height])
    }
}

// ---------------------------------------------------------------------------
// The view frame
// ---------------------------------------------------------------------------

/// The saved camera as an orthonormal frame: `right` / `up` span the
/// projection plane, `forward` runs eye → target. Points are measured from the
/// camera's TARGET, which is the point the placement positions on the paper.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ViewFrame {
    pub target: [f64; 3],
    pub right: [f64; 3],
    pub up: [f64; 3],
    pub forward: [f64; 3],
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm(v: [f64; 3]) -> [f64; 3] {
    let len = dot(v, v).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

impl ViewFrame {
    /// The frame of a saved camera. Mirrors [`crate::view::ViewCamera::basis`]
    /// exactly (right = forward × up, true up = right × forward), so a sheet
    /// view is oriented the way the viewport showed it.
    pub fn of(camera: &PmiCamera) -> ViewFrame {
        let forward = norm(sub(camera.target, camera.eye));
        let forward = if forward == [0.0, 0.0, 0.0] { [0.0, 0.0, -1.0] } else { forward };
        let mut right = norm(cross(forward, camera.up));
        if right == [0.0, 0.0, 0.0] {
            // The up vector is parallel to the view direction: any in-plane
            // axis will do, and this one is continuous in the other cases.
            right = norm(cross(forward, [0.0, 0.0, 1.0]));
            if right == [0.0, 0.0, 0.0] {
                right = norm(cross(forward, [0.0, 1.0, 0.0]));
            }
        }
        let up = cross(right, forward);
        ViewFrame { target: camera.target, right, up, forward }
    }

    /// `(x, y, depth)` in view space: x right, y UP, depth along the viewing
    /// direction from the target plane (larger = further from the eye).
    pub fn to_view(&self, p: [f64; 3]) -> [f64; 3] {
        let d = sub(p, self.target);
        [dot(d, self.right), dot(d, self.up), dot(d, self.forward)]
    }
}

/// View-space `(x, y)` → paper millimetres for a placement.
fn to_paper(placed: &PlacedView, x: f64, y: f64) -> [f64; 2] {
    [placed.position[0] + x * placed.scale, placed.position[1] - y * placed.scale]
}

// ---------------------------------------------------------------------------
// Entry
// ---------------------------------------------------------------------------

/// Project one placed view. `view` is the saved PMI view it names and `report`
/// that view's resolved annotations (both `None` when the block no longer
/// carries it, which is an error row rather than a dropped placement).
pub fn project_view(
    scene: &RenderScene,
    placed: &PlacedView,
    view: Option<&PmiView>,
    report: Option<&PmiViewReport>,
) -> ViewDrawing {
    let (unposed, env) = (Unposed::new(), brep_kernel::Env::default());
    project_view_through(&Model { scene, unposed: &unposed, env: &env }, placed, view, report, None, &mut LinesMode::Compute)
}

/// What a projection reads beside the sheet and its PMI views: the scene, the
/// displays the viewport's active view posed in place (un-posed, see
/// [`Unposed`]), and the document's expression sheet — a PMI row's tolerance
/// may be an expression, and a sheet dimension that inherits it needs its
/// value.
pub struct Model<'a> {
    pub scene: &'a RenderScene,
    pub unposed: &'a Unposed,
    pub env: &'a brep_kernel::Env,
}

/// The displays the viewport's ACTIVE PMI view has posed in place, un-posed —
/// the engine's `pmi_explode_originals`. A sheet draws every placement with
/// its OWN view's poses, so it reads a display through this first.
pub type Unposed = std::collections::HashMap<String, crate::scene::SolidDisplay>;

/// Which pass drew a placement's model lines ([`ViewDrawing::hidden_line`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HiddenLine {
    /// The exact pass on the B-rep ([`super::hlr::visible_edges`]).
    Analytic,
    /// The same pass on the display ([`super::hlr::visible_mesh_lines`]): the
    /// stated approximation while a drawn solid's B-rep is not in the scene.
    Mesh,
}

impl HiddenLine {
    pub fn label(self) -> &'static str {
        match self {
            HiddenLine::Analytic => "analytic",
            HiddenLine::Mesh => "mesh",
        }
    }
}

/// What a SECTION placement projects through, once its cut has been resolved
/// against the source placement's projection: the derived camera frame, the
/// half-space to keep, and the letter.
pub struct SectionSetup {
    pub frame: ViewFrame,
    pub plane: ClipPlane,
    pub label: String,
}

/// What a DETAIL placement projects through, once its circle has been
/// resolved against the source placement's projection: the source's own
/// camera frame re-targeted on the circle's centre, and the circle's radius
/// in MODEL units.
pub struct DetailSetup {
    pub frame: ViewFrame,
    pub radius: f64,
    pub label: String,
}

/// A placement whose camera is DERIVED from another placement rather than
/// read off its saved view.
pub enum Derived {
    Section(SectionSetup),
    Detail(DetailSetup),
}

/// Project one placed view, optionally through a DERIVED camera — a
/// section's or a detail's — instead of its saved one.
///
/// With a SECTION, the frame is the derived one, the model is cut by the
/// section plane exactly ([`super::hlr::section_curves`]), and the pass keeps
/// only what is at or beyond the plane with the cut region as a cap in front
/// of what lies behind it. The candidate, split and visibility code and the
/// ANCHOR space are the very same as a plain placement's.
///
/// With a DETAIL, the frame is the source's camera re-targeted on the circle's
/// centre (so this placement's `position` is where the centre lands), and
/// visibility is decided inside the circle's cylinder at the detail's own
/// scale; [`clip_to_circle`] then trims only a chord that bows past the rim.
///
/// Both draw the model ONLY: the saved view's annotations were laid out
/// against THAT view's camera and scale, and a section's camera is a different
/// one, while a detail's circle would cut a dimension or a frame in half.
pub fn project_view_through(
    model: &Model,
    placed: &PlacedView,
    view: Option<&PmiView>,
    report: Option<&PmiViewReport>,
    derived: Option<Derived>,
    mode: &mut LinesMode,
) -> ViewDrawing {
    let (scene, unposed) = (model.scene, model.unposed);
    let mut out = ViewDrawing::empty(placed, view.map(|v| v.name.clone()).unwrap_or_default());
    let Some(view) = view else {
        out.error = format!("no PMI view '{}'", placed.view);
        return out;
    };
    let Some(camera) = view.camera.as_ref() else {
        out.error = format!("PMI view '{}' has no captured camera", placed.view);
        return out;
    };
    if !matches!(camera.projection, PmiProjection::Orthographic { .. }) {
        out.error = format!(
            "PMI view '{}' is a perspective camera; a sheet draws {ORTHOGRAPHIC} views only",
            placed.view
        );
        return out;
    }
    if !placed.projection.eq_ignore_ascii_case(ORTHOGRAPHIC) {
        out.error = format!("projection '{}' is not drawn on a sheet", placed.projection);
        return out;
    }

    let frame = match &derived {
        Some(Derived::Section(setup)) => setup.frame,
        Some(Derived::Detail(setup)) => setup.frame,
        None => ViewFrame::of(camera),
    };
    out.text_mm = view.display.text_size_pt * MM_PER_POINT;
    let cap = if out.text_mm > 1e-9 { out.text_mm } else { 12.0 * MM_PER_POINT };
    // The exact pass needs every drawn solid's B-rep; a solid whose topology
    // did not reach the scene sends the placement to the mesh approximation,
    // and so does a pass handed to the runner that has not come back yet. The
    // placement says which drew it rather than drawing that solid as nothing.
    let plan = exact_solids(scene, view, report);
    let clip = match &derived {
        None => LinesClip::Whole,
        Some(Derived::Detail(setup)) => LinesClip::Circle { radius: setup.radius },
        Some(Derived::Section(_)) => LinesClip::Section,
    };
    let exact = plan.as_ref().and_then(|plan| exact_lines_for(plan, &frame, placed, clip, mode));
    out.hidden_line = if exact.is_some() { HiddenLine::Analytic } else { HiddenLine::Mesh }.label().into();
    match &derived {
        None => {
            out.edges = match exact {
                Some(lines) => lines.runs,
                None => mesh_lines_for(scene, unposed, view, report, &frame, placed, LinesClip::Whole, None).runs,
            };
            out.annotations = annotations(report, view, &frame, placed);
            out.tolerances = tolerance_sources(report, view, &frame, placed, model.env);
        }
        Some(Derived::Detail(setup)) => {
            let radius_mm = setup.radius * placed.scale;
            // Analytic: visibility is decided on the model inside the
            // circle's cylinder, sampled at THIS placement's scale. The
            // circle clip after it only trims a chord that bows past the rim.
            let lines = match exact {
                Some(lines) => lines.runs,
                None => mesh_lines_for(scene, unposed, view, report, &frame, placed, clip, None).runs,
            };
            out.edges = clip_to_circle(lines, placed.position, radius_mm);
            out.detail = Some(DetailDrawing {
                label: setup.label.clone(),
                centre: placed.position,
                radius_mm,
                circle: circle_polyline(placed.position, radius_mm),
                texts: Vec::new(),
            });
        }
        Some(Derived::Section(setup)) => match exact {
            Some(ExactLines { runs, cut }) => {
                // The cut curves are model lines in their own right: nothing
                // is in front of a cut face to hide them.
                let mut edges = runs;
                edges.extend(cut.iter().cloned());
                out.edges = edges;
                out.section = Some(SectionDrawing {
                    label: setup.label.clone(),
                    hatch: hatch(&segments_of(&cut)),
                    cut,
                    texts: Vec::new(),
                });
            }
            None => {
                let ExactLines { runs, cut: cut_paper } =
                    mesh_lines_for(scene, unposed, view, report, &frame, placed, clip, Some(&setup.plane));
                let mut edges = runs;
                edges.extend(cut_paper.iter().cloned());
                out.edges = edges;
                out.section = Some(SectionDrawing {
                    label: setup.label.clone(),
                    hatch: hatch(&cut_paper),
                    cut: cut_paper,
                    texts: Vec::new(),
                });
            }
        },
    }
    out.anchors = anchor_points(scene, unposed, view, report, &frame, placed);
    // A detail offers the anchors INSIDE its circle and no others. The anchor
    // space is still the model's topology — the hidden-line pass has no vote
    // on it, here as everywhere — but a point outside the circle is not on
    // this placement at all: nothing of the detail is drawn there, and at the
    // detail's scale it would land anywhere on the paper, over other views,
    // where the sheet picker's markers would offer it as a point of theirs.
    if let Some(detail) = &out.detail {
        let reach = detail.radius_mm * (1.0 + 1e-9) + 1e-9;
        out.anchors.retain(|anchor| {
            let (dx, dy) = (anchor.at[0] - detail.centre[0], anchor.at[1] - detail.centre[1]);
            (dx * dx + dy * dy).sqrt() <= reach
        });
    }
    out.bounds = bounds_of(&out, placed);
    // The caption goes UNDER the finished drawing, so it is placed once the
    // bounds are known — and the bounds are then re-read to take it in, or a
    // placement drag would grab a rect the caption hangs out of.
    let middle = (out.bounds[0] + out.bounds[2]) * 0.5;
    let below = out.bounds[3] + cap * 2.0;
    if let Some(section) = out.section.as_mut() {
        section.texts.push(SheetText::flat(
            format!("SECTION {}\u{2014}{}", section.label, section.label),
            [middle, below],
            cap,
        ));
        out.bounds = bounds_of(&out, placed);
    }
    // A detail's caption says which circle it is and — the one thing that
    // distinguishes it from its source — the scale it is drawn at.
    if let Some(detail) = out.detail.as_mut() {
        detail.texts.push(SheetText::flat(format!("DETAIL {}", detail.label), [middle, below], cap));
        detail.texts.push(SheetText::flat(
            format!("SCALE {}", super::frame::ratio_text(placed.scale)),
            [middle, below + cap * 1.8],
            cap * 0.8,
        ));
        out.bounds = bounds_of(&out, placed);
    }
    out
}

/// The whole sheet: the paper's furniture, then every placement projected, in
/// order. `context` is what the border and title block need and a projection
/// cannot know — the document's name and the day (see
/// [`super::frame::FrameContext`]). Every exact pass runs HERE, on the calling
/// thread; [`project_sheet_with`] can hand them over instead.
pub fn project_sheet(
    scene: &RenderScene,
    sheet: &super::Sheet,
    views: &[PmiView],
    report: Option<&brep_kernel::PmiReport>,
    context: &super::frame::FrameContext,
) -> SheetDrawing {
    let (unposed, env) = (Unposed::new(), brep_kernel::Env::default());
    project_sheet_with(&Model { scene, unposed: &unposed, env: &env }, sheet, views, report, context, &mut LinesMode::Compute)
}

/// [`project_sheet`], with what an exact-lines miss does chosen by `mode`: the
/// engine's viewport hands the pass to the runner and draws the mesh
/// approximation until the answer lands; an export computes it here.
pub fn project_sheet_with(
    model: &Model,
    sheet: &super::Sheet,
    views: &[PmiView],
    report: Option<&brep_kernel::PmiReport>,
    context: &super::frame::FrameContext,
    mode: &mut LinesMode,
) -> SheetDrawing {
    let (width_mm, height_mm) = sheet.millimetres();
    // PLAIN placements first, then the DERIVED ones: a section's camera and a
    // detail's circle are both read off where their anchors landed in the
    // source's projection, so the source has to be drawn before either can
    // know where it is looking from. Every derived placement's source must be
    // a PLAIN one — a section or a detail OF a section or a detail is refused
    // by name rather than ordered, because this pass is one pass and a chain
    // would need as many.
    let mut drawn: Vec<ViewDrawing> = sheet
        .views
        .iter()
        .map(|placed| {
            let saved = views.iter().find(|view| view.id == placed.view);
            if placed.section.is_some() || placed.detail.is_some() {
                // A stand-in until the second pass: it holds the id and the
                // position, so a derived view that cannot resolve still has a
                // row and a grab rect on the paper.
                ViewDrawing::empty(placed, saved.map(|v| v.name.clone()).unwrap_or_default())
            } else {
                project_view_through(model, placed, saved, report.and_then(|report| report.view(&placed.view)), None, mode)
            }
        })
        .collect();
    for (index, placed) in sheet.views.iter().enumerate() {
        let Some(circle) = placed.detail.as_ref() else { continue };
        match detail_setup(sheet, &drawn, circle, views) {
            Err(error) => drawn[index].error = error,
            Ok((setup, source, mark)) => {
                drawn[index] = project_view_through(
                    model,
                    placed,
                    views.iter().find(|view| view.id == placed.view),
                    report.and_then(|report| report.view(&placed.view)),
                    Some(Derived::Detail(setup)),
                    mode,
                );
                let mark = DetailMark { id: placed.id.clone(), ..mark };
                drawn[source].detail_marks.push(mark);
                let source_placed = &sheet.views[source];
                drawn[source].bounds = bounds_of(&drawn[source], source_placed);
            }
        }
    }
    for (index, placed) in sheet.views.iter().enumerate() {
        let Some(cut) = placed.section.as_ref() else { continue };
        match section_setup(sheet, &drawn, placed, cut, views) {
            Err(error) => drawn[index].error = error,
            Ok((setup, source, mark)) => {
                drawn[index] = project_view_through(
                    model,
                    placed,
                    views.iter().find(|view| view.id == placed.view),
                    report.and_then(|report| report.view(&placed.view)),
                    Some(Derived::Section(setup)),
                    mode,
                );
                // The LINE goes on the source, where it was picked. Its own
                // bounds grow to take it in, so dragging that placement still
                // grabs everything it draws.
                drawn[source].section_marks.push(mark);
                let source_placed = &sheet.views[source];
                drawn[source].bounds = bounds_of(&drawn[source], source_placed);
            }
        }
    }
    // The sheet's own dimensions come LAST: they anchor to the placements'
    // projections, so they are laid out over the finished drawing rather than
    // inside any one placement.
    let dimensions = super::dimension::layout_dimensions(
        &sheet.dimensions,
        &drawn,
        [width_mm * 0.5, height_mm * 0.5],
    );
    let ordinates = super::ordinate::layout_ordinates(
        &sheet.ordinates,
        &drawn,
        [width_mm * 0.5, height_mm * 0.5],
    );
    SheetDrawing {
        id: sheet.id.clone(),
        name: sheet.name.clone(),
        width_mm,
        height_mm,
        frame: super::frame::frame(sheet, context),
        views: drawn,
        dimensions,
        ordinates,
    }
}

/// Resolve a section's cut against the SOURCE placement's finished projection:
/// the derived camera frame, the half-space to keep, which placement carries
/// the line, and the line itself.
///
/// # The camera
///
/// The two anchors are paper points of the source's projection. Lifted back
/// into the source camera's own plane they give two 3D points, and the CUTTING
/// PLANE is the plane through them containing the source's viewing direction —
/// which is what "a line drawn on a view" means: every point of the paper line
/// is a whole ray of the model.
///
/// The section camera then looks along that plane's NORMAL, and its up is the
/// source's viewing direction REVERSED. Those two are perpendicular for free,
/// because the plane contains the source's viewing direction and its normal is
/// perpendicular to everything in it — so the frame is orthonormal by
/// construction rather than by a re-orthogonalisation that could quietly tilt
/// the section. It is also the drawing convention: section a TOP view and the
/// section is an elevation with model up ON the page.
///
/// # Which way it looks
///
/// 90\u{00B0} clockwise from `from` \u{2192} `to` on the SOURCE's paper — the side a reader
/// walking from the first arrow to the second has on their right. The pick
/// ORDER is therefore the choice, and `flip` is the form's way to change its
/// mind without re-picking.
fn section_setup(
    sheet: &super::Sheet,
    drawn: &[ViewDrawing],
    placed: &PlacedView,
    cut: &super::SectionCut,
    views: &[PmiView],
) -> Result<(SectionSetup, usize, SectionMark), String> {
    // A section the Section view button made has no line until its form's
    // Cutting line row is picked — say what it is waiting for.
    if cut.from.is_empty() || cut.to.is_empty() {
        return Err("no cutting line yet \u{2014} pick its two anchors on a placed view".into());
    }
    let Some(source_index) = sheet.views.iter().position(|v| v.id == cut.source) else {
        return Err(format!("no placement '{}' to cut through on this sheet", cut.source));
    };
    let source_placed = &sheet.views[source_index];
    if source_placed.section.is_some() {
        return Err(format!(
            "'{}' is itself a section; a section of a section is not drawn",
            cut.source
        ));
    }
    if source_placed.detail.is_some() {
        return Err(format!(
            "'{}' is a detail; a section of a detail is not drawn",
            cut.source
        ));
    }
    let source = &drawn[source_index];
    if !source.error.is_empty() {
        return Err(format!("placement '{}' did not project", cut.source));
    }
    let (from, to) = (point_of(source, &cut.from)?, point_of(source, &cut.to)?);
    let span = [to[0] - from[0], to[1] - from[1]];
    let length = (span[0] * span[0] + span[1] * span[1]).sqrt();
    if !(length > 1e-9) {
        return Err(format!(
            "'{}' and '{}' are the same point; a cutting line needs two",
            cut.from, cut.to
        ));
    }
    let Some(camera) = views
        .iter()
        .find(|view| view.id == source_placed.view)
        .and_then(|view| view.camera.as_ref())
    else {
        return Err(format!("placement '{}' has no captured camera to cut in", cut.source));
    };
    let source_frame = ViewFrame::of(camera);
    if source_placed.scale <= 0.0 {
        return Err(format!("placement '{}' has no scale", cut.source));
    }
    // Paper -> the source camera's own plane. The depth along the source's
    // viewing direction is free — the cutting plane contains that direction,
    // so every choice names the same plane — and the target plane is the one
    // choice with no arbitrary number in it.
    let (p0, p1) = (lift(&source_frame, source_placed, from), lift(&source_frame, source_placed, to));
    let along = norm(sub(p1, p0));
    let mut forward = norm(cross(source_frame.forward, along));
    if forward == [0.0, 0.0, 0.0] {
        return Err("the cutting line has no direction in this view".into());
    }
    if cut.flip {
        forward = [-forward[0], -forward[1], -forward[2]];
    }
    let up = [-source_frame.forward[0], -source_frame.forward[1], -source_frame.forward[2]];
    let right = norm(cross(forward, up));
    let target = [(p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5];
    let setup = SectionSetup {
        frame: ViewFrame { target, right, up: cross(right, forward), forward },
        plane: ClipPlane { point: target, normal: forward },
        label: cut.label.clone(),
    };
    let cap = if source.text_mm > 1e-9 { source.text_mm } else { 12.0 * MM_PER_POINT };
    Ok((setup, source_index, section_mark(placed, cut, from, to, cap)))
}

/// The section LINE on the source's paper: the line through the two anchors
/// with a tail past each, an arrow at each end pointing the way the section
/// looks, and the letter beside each arrow.
fn section_mark(
    placed: &PlacedView,
    cut: &super::SectionCut,
    from: [f64; 2],
    to: [f64; 2],
    cap: f64,
) -> SectionMark {
    let span = [to[0] - from[0], to[1] - from[1]];
    let length = (span[0] * span[0] + span[1] * span[1]).sqrt().max(1e-9);
    let along = [span[0] / length, span[1] / length];
    // The viewing direction, as the SOURCE's paper sees it: 90 degrees
    // clockwise from the travel direction (paper y runs DOWN, so that is
    // `(-dy, dx)`), turned round by `flip`. Derived here from the same pick
    // order the camera is, so the arrows point where the section looks.
    let sign = if cut.flip { -1.0 } else { 1.0 };
    let look = [-along[1] * sign, along[0] * sign];
    let tail = cap * 1.5;
    let arrow = cap * 1.2;
    let mut lines = vec![vec![
        [from[0] - along[0] * tail, from[1] - along[1] * tail],
        [to[0] + along[0] * tail, to[1] + along[1] * tail],
    ]];
    let mut texts = Vec::new();
    for (end, outward) in [(from, [-along[0], -along[1]]), (to, along)] {
        let root = [end[0] + outward[0] * tail, end[1] + outward[1] * tail];
        let tip = [root[0] + look[0] * arrow * 2.0, root[1] + look[1] * arrow * 2.0];
        // The stem, then the head as a closed outline — the sheet's contract
        // is lines, so an arrowhead is its own boundary here exactly as it is
        // in the shared dimension layout.
        lines.push(vec![root, tip]);
        let across = [-look[1] * arrow * 0.35, look[0] * arrow * 0.35];
        lines.push(vec![
            tip,
            [tip[0] - look[0] * arrow + across[0], tip[1] - look[1] * arrow + across[1]],
            [tip[0] - look[0] * arrow - across[0], tip[1] - look[1] * arrow - across[1]],
            tip,
        ]);
        texts.push(SheetText::flat(
            cut.label.clone(),
            [
                root[0] + outward[0] * cap * 1.2 + look[0] * cap * 0.6,
                root[1] + outward[1] * cap * 1.2 + look[1] * cap * 0.6,
            ],
            cap,
        ));
    }
    SectionMark { id: placed.id.clone(), label: cut.label.clone(), lines, texts }
}

/// Where anchor `reference` landed on a finished placement's paper, or the
/// reason a derived view built on it cannot resolve.
fn point_of(source: &ViewDrawing, reference: &str) -> Result<[f64; 2], String> {
    source
        .anchors
        .iter()
        .find(|a| a.anchor == reference)
        .map(|a| a.at)
        .ok_or_else(|| format!("'{reference}' is not a point of '{}' any more", source.id))
}

/// A paper point of `placed` lifted back into its camera's TARGET plane: the
/// inverse of [`to_paper`] with the depth set to zero.
fn lift(frame: &ViewFrame, placed: &PlacedView, p: [f64; 2]) -> [f64; 3] {
    let x = (p[0] - placed.position[0]) / placed.scale;
    let y = -(p[1] - placed.position[1]) / placed.scale;
    [
        frame.target[0] + frame.right[0] * x + frame.up[0] * y,
        frame.target[1] + frame.right[1] * x + frame.up[1] * y,
        frame.target[2] + frame.right[2] * x + frame.up[2] * y,
    ]
}

/// Resolve a detail's circle against the SOURCE placement's finished
/// projection: the camera the detail looks through, which placement carries
/// the circle, and the circle itself as drawn there.
///
/// # The camera
///
/// The SOURCE's, unchanged in every direction: the detail is the same view at
/// a larger scale, so its right, up and viewing direction are the source's.
/// Only the TARGET moves — to the circle's centre, lifted back into the source
/// camera's own plane — which is what makes this placement's `position` the
/// point the circle's centre lands on. The depth of that target is free (a
/// parallel projection does not care where along the viewing direction the
/// target plane is), and the source's own target plane is the choice with no
/// arbitrary number in it.
///
/// # The radius
///
/// The distance between the two anchors on the SOURCE's paper over the
/// source's scale: a model length, so the circle holds the same region of the
/// model at any detail scale. Measured in the projection plane, which is where
/// the circle is.
fn detail_setup(
    sheet: &super::Sheet,
    drawn: &[ViewDrawing],
    circle: &super::DetailCircle,
    views: &[PmiView],
) -> Result<(DetailSetup, usize, DetailMark), String> {
    // A detail the Detail view button made has no circle until its form's
    // Centre and rim rows are picked.
    if circle.centre.is_empty() || circle.rim.is_empty() {
        return Err("no detail circle yet \u{2014} pick its centre and a point on its rim on a placed view".into());
    }
    let Some(source_index) = sheet.views.iter().position(|v| v.id == circle.source) else {
        return Err(format!("no placement '{}' to draw a detail of on this sheet", circle.source));
    };
    let source_placed = &sheet.views[source_index];
    if source_placed.detail.is_some() {
        return Err(format!(
            "'{}' is itself a detail; a detail of a detail is not drawn",
            circle.source
        ));
    }
    if source_placed.section.is_some() {
        return Err(format!(
            "'{}' is a section; a detail of a section is not drawn",
            circle.source
        ));
    }
    let source = &drawn[source_index];
    if !source.error.is_empty() {
        return Err(format!("placement '{}' did not project", circle.source));
    }
    let (centre, rim) = (point_of(source, &circle.centre)?, point_of(source, &circle.rim)?);
    let radius_mm = ((rim[0] - centre[0]).powi(2) + (rim[1] - centre[1]).powi(2)).sqrt();
    if !(radius_mm > 1e-9) {
        return Err(format!(
            "'{}' and '{}' land on the same point of '{}'; a detail circle needs a radius",
            circle.centre, circle.rim, circle.source
        ));
    }
    let Some(camera) = views
        .iter()
        .find(|view| view.id == source_placed.view)
        .and_then(|view| view.camera.as_ref())
    else {
        return Err(format!("placement '{}' has no captured camera to draw a detail of", circle.source));
    };
    if source_placed.scale <= 0.0 {
        return Err(format!("placement '{}' has no scale", circle.source));
    }
    let source_frame = ViewFrame::of(camera);
    let setup = DetailSetup {
        frame: ViewFrame { target: lift(&source_frame, source_placed, centre), ..source_frame },
        radius: radius_mm / source_placed.scale,
        label: circle.label.clone(),
    };
    let cap = if source.text_mm > 1e-9 { source.text_mm } else { 12.0 * MM_PER_POINT };
    // The letter sits outside the circle up and to the right, a cap and a
    // half clear of the rim, so it names the circle without touching what the
    // circle encloses.
    let d = std::f64::consts::FRAC_1_SQRT_2;
    let reach = radius_mm + cap * 1.5;
    let mark = DetailMark {
        id: String::new(),
        label: circle.label.clone(),
        centre,
        radius_mm,
        lines: vec![circle_polyline(centre, radius_mm)],
        texts: vec![SheetText::flat(
            circle.label.clone(),
            [centre[0] + reach * d, centre[1] - reach * d],
            cap,
        )],
    };
    Ok((setup, source_index, mark))
}

/// How many straight pieces a detail circle is drawn with. A circle is a
/// polyline on a sheet like every other curve; at this count a 100 mm circle
/// misses the true one by 0.05 mm, under a stroke width.
const CIRCLE_SEGMENTS: usize = 96;

/// A closed circle polyline in paper millimetres, back to its first point.
fn circle_polyline(centre: [f64; 2], radius: f64) -> Vec<[f64; 2]> {
    (0..=CIRCLE_SEGMENTS)
        .map(|k| {
            let t = std::f64::consts::TAU * k as f64 / CIRCLE_SEGMENTS as f64;
            [centre[0] + radius * t.cos(), centre[1] + radius * t.sin()]
        })
        .collect()
}

/// Keep the parts of `lines` inside the circle, splitting a polyline wherever
/// it leaves and re-enters.
///
/// Exact per segment — each piece is cut where it crosses the TRUE circle, not
/// the polyline it is drawn with — so a line leaving the detail ends on the
/// boundary a reader sees rather than a stroke short of it. A segment lying
/// wholly inside keeps its own two ends, so a straight edge inside the circle
/// comes back as the two points it went in as.
fn clip_to_circle(lines: Vec<Vec<[f64; 2]>>, centre: [f64; 2], radius: f64) -> Vec<Vec<[f64; 2]>> {
    let mut out: Vec<Vec<[f64; 2]>> = Vec::new();
    let flush = |current: &mut Vec<[f64; 2]>, out: &mut Vec<Vec<[f64; 2]>>| {
        if current.len() >= 2 {
            out.push(std::mem::take(current));
        } else {
            current.clear();
        }
    };
    for line in lines {
        let mut current: Vec<[f64; 2]> = Vec::new();
        for pair in line.windows(2) {
            let (p, q) = (pair[0], pair[1]);
            let d = [q[0] - p[0], q[1] - p[1]];
            let f = [p[0] - centre[0], p[1] - centre[1]];
            let a = d[0] * d[0] + d[1] * d[1];
            let c = f[0] * f[0] + f[1] * f[1] - radius * radius;
            // The parameter span of the segment inside the circle, if any.
            let span = if a < 1e-24 {
                (c <= 0.0).then_some((0.0, 1.0))
            } else {
                let b = 2.0 * (f[0] * d[0] + f[1] * d[1]);
                let disc = b * b - 4.0 * a * c;
                if disc < 0.0 {
                    None
                } else {
                    let root = disc.sqrt();
                    let (t0, t1) = (((-b - root) / (2.0 * a)).max(0.0), ((-b + root) / (2.0 * a)).min(1.0));
                    (t1 > t0).then_some((t0, t1))
                }
            };
            let at = |t: f64| [p[0] + d[0] * t, p[1] + d[1] * t];
            match span {
                None => flush(&mut current, &mut out),
                Some((t0, t1)) => {
                    if t0 <= 0.0 {
                        if current.is_empty() {
                            current.push(p);
                        }
                        if t1 >= 1.0 {
                            current.push(q);
                        } else {
                            current.push(at(t1));
                            flush(&mut current, &mut out);
                        }
                    } else {
                        flush(&mut current, &mut out);
                        current.push(at(t0));
                        if t1 >= 1.0 {
                            current.push(q);
                        } else {
                            current.push(at(t1));
                            flush(&mut current, &mut out);
                        }
                    }
                }
            }
        }
        flush(&mut current, &mut out);
    }
    out
}

fn bounds_of(view: &ViewDrawing, placed: &PlacedView) -> [f64; 4] {
    let mut bounds = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut add = |p: [f64; 2]| {
        bounds[0] = bounds[0].min(p[0]);
        bounds[1] = bounds[1].min(p[1]);
        bounds[2] = bounds[2].max(p[0]);
        bounds[3] = bounds[3].max(p[1]);
    };
    for polyline in &view.edges {
        for point in polyline {
            add(*point);
        }
    }
    for annotation in &view.annotations {
        for polyline in &annotation.lines {
            for point in polyline {
                add(*point);
            }
        }
        for text in &annotation.texts {
            add(text.anchor);
        }
    }
    if let Some(section) = &view.section {
        for polyline in section.hatch.iter().chain(section.cut.iter()) {
            for point in polyline {
                add(*point);
            }
        }
        for text in &section.texts {
            add(text.anchor);
        }
    }
    for mark in &view.section_marks {
        for polyline in &mark.lines {
            for point in polyline {
                add(*point);
            }
        }
        for text in &mark.texts {
            add(text.anchor);
        }
    }
    if let Some(detail) = &view.detail {
        for point in &detail.circle {
            add(*point);
        }
        for text in &detail.texts {
            add(text.anchor);
        }
    }
    for mark in &view.detail_marks {
        for polyline in &mark.lines {
            for point in polyline {
                add(*point);
            }
        }
        for text in &mark.texts {
            add(text.anchor);
        }
    }
    if bounds[0] > bounds[2] {
        [placed.position[0], placed.position[1], placed.position[0], placed.position[1]]
    } else {
        bounds
    }
}

// ---------------------------------------------------------------------------
// The exact solids
// ---------------------------------------------------------------------------

/// One explode row's pose of a drawn solid: `p' = R((p − c) ∘ s) + c + t`, the
/// viewport's own formula (`pmi_ops`), read off the view's report.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SolidPose {
    pub translate: [f64; 3],
    #[serde(rename = "rotateDeg")]
    pub rotate_deg: [f64; 3],
    pub scale: [f64; 3],
    pub center: [f64; 3],
}

impl SolidPose {
    pub fn apply(&self, p: [f64; 3]) -> [f64; 3] {
        let (center, scale, translate) = (self.center, self.scale, self.translate);
        let local = [(p[0] - center[0]) * scale[0], (p[1] - center[1]) * scale[1], (p[2] - center[2]) * scale[2]];
        let rotated = crate::engine_state::rotate_euler_xyz_f64(local, self.rotate_deg);
        [
            rotated[0] + center[0] + translate[0],
            rotated[1] + center[1] + translate[1],
            rotated[2] + center[2] + translate[2],
        ]
    }

    /// The pose to the bit, as it enters an identity.
    fn identity(&self) -> String {
        let bits = |v: &[f64; 3]| format!("{:x},{:x},{:x}", v[0].to_bits(), v[1].to_bits(), v[2].to_bits());
        format!(
            "pose[{}|{}|{}|{}];",
            bits(&self.translate),
            bits(&self.rotate_deg),
            bits(&self.scale),
            bits(&self.center)
        )
    }
}

/// Every pose the view's enabled, resolved explode rows apply to `name`, in
/// row order — what the exact pass and the mesh approximation both draw a
/// solid with.
pub(crate) fn poses_of(report: Option<&PmiViewReport>, name: &str) -> Vec<SolidPose> {
    let Some(report) = report else { return Vec::new() };
    report
        .annotations
        .iter()
        .filter(|row| row.enabled && row.status == PmiStatus::Ok)
        .filter_map(|row| match &row.geometry {
            brep_kernel::PmiGeometry::Explode { solids, translate, rotate_deg, scale, center, .. }
                if solids.iter().any(|target| target == name) =>
            {
                Some(SolidPose { translate: *translate, rotate_deg: *rotate_deg, scale: *scale, center: *center })
            }
            _ => None,
        })
        .collect()
}

/// One solid of an exact pass, by what the RUNNER holds it as: its name, the
/// resident handle, the content fingerprint the main side read off its copy,
/// and the poses the view applies to it.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LinesSolid {
    pub name: String,
    pub handle: u32,
    pub fingerprint: u64,
    pub poses: Vec<SolidPose>,
}

impl LinesSolid {
    /// What the drawn geometry IS: the content fingerprint and every pose
    /// applied to it, to the bit. Not the resident handle: a handle is unique
    /// only within one runner's registry, and a restarted runner or a second
    /// document tab counts from one again.
    fn identity(&self) -> String {
        let mut identity = format!("{}#{:016x};", self.name, self.fingerprint);
        for pose in &self.poses {
            identity.push_str(&pose.identity());
        }
        identity
    }
}

/// The exact solids a saved view draws, as a PLAN: which B-rep each is and how
/// it is posed, without posing anything — a projection whose lines are cached,
/// or computed on the runner, never needs the posed copies on this side.
struct ExactPlan<'a> {
    solids: Vec<LinesSolid>,
    /// The main side's copy of each, unposed.
    sources: Vec<&'a brep_kernel::BrepSolid>,
    /// Each solid's identity, and all of them together.
    each: Vec<String>,
    identity: String,
}

impl ExactPlan<'_> {
    /// The B-rep of every solid, POSED by the view's own explode rows.
    ///
    /// The pose is the view's, not the viewport's: the display meshes are posed
    /// only while a view is ACTIVE, and a sheet draws every placement's view at
    /// once. A pose is affine, and a NURBS carrier is invariant under an affine
    /// map of its control points, so the posed solid is exact.
    fn posed(&self) -> Vec<std::borrow::Cow<'_, brep_kernel::BrepSolid>> {
        self.solids
            .iter()
            .zip(&self.sources)
            .map(|(solid, source)| posed_solid(std::borrow::Cow::Borrowed(*source), &solid.poses))
            .collect()
    }
}

fn posed_solid<'a>(
    solid: std::borrow::Cow<'a, brep_kernel::BrepSolid>,
    poses: &[SolidPose],
) -> std::borrow::Cow<'a, brep_kernel::BrepSolid> {
    poses.iter().fold(solid, |solid, pose| std::borrow::Cow::Owned(pose_solid(&solid, &|p| pose.apply(p))))
}

/// The plan of every solid a saved view draws, or `None` when any of them has
/// no topology in the scene.
fn exact_solids<'a>(scene: &'a RenderScene, view: &PmiView, report: Option<&PmiViewReport>) -> Option<ExactPlan<'a>> {
    let mut plan = ExactPlan { solids: Vec::new(), sources: Vec::new(), each: Vec::new(), identity: String::new() };
    for solid in drawn_solids(scene, view) {
        let (brep, fingerprint) = scene.exact_solid_fingerprinted(&solid.name)?;
        let planned = LinesSolid {
            name: solid.name.clone(),
            handle: solid.source_handle,
            fingerprint,
            poses: poses_of(report, &solid.name),
        };
        plan.each.push(planned.identity());
        plan.solids.push(planned);
        plan.sources.push(brep);
    }
    plan.identity = plan.each.concat();
    Some(plan)
}

// ---------------------------------------------------------------------------
// The exact lines, and the job that computes them off the UI thread
// ---------------------------------------------------------------------------

/// What an exact pass is restricted to, as a job carries it: a SECTION's cap
/// is the cut the job computes itself, so it has no data to carry.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum LinesClip {
    Whole,
    Circle { radius: f64 },
    Section,
}

impl LinesClip {
    /// The clip's part of an exact-lines key.
    fn key(&self) -> String {
        match self {
            LinesClip::Whole => "whole".into(),
            LinesClip::Circle { radius } => format!("circle {:x}", radius.to_bits()),
            LinesClip::Section => "section".into(),
        }
    }
}

/// ONE placement's analytic pass, as the runner is asked to run it: everything
/// [`exact_lines`] would compute on a miss, by reference to the runner's own
/// resident solids rather than a copy of them.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct LinesJob {
    /// The exact-lines key the answer is cached under.
    pub key: String,
    /// The placement it draws: a later job for the same placement SUPERSEDES
    /// this one (the engine drops a superseded answer).
    pub placement: String,
    pub solids: Vec<LinesSolid>,
    pub frame: ViewFrame,
    /// The placement at the paper origin — the lines are relative to it.
    pub placed: PlacedView,
    pub clip: LinesClip,
}

/// What a projection does on an exact-lines MISS.
pub enum LinesMode<'a> {
    /// Run the pass here and now, on this thread — an export, a test, a runner
    /// that cannot run the job.
    Compute,
    /// Hand the job over. The answer comes back as the lines (a runner that
    /// answered at once), `None` while it is being computed elsewhere — the
    /// placement then draws its mesh approximation — or [`Deferred::Here`] to
    /// run it on this thread after all.
    Defer(&'a mut dyn FnMut(LinesJob) -> Deferred),
}

/// A deferred job's answer.
pub enum Deferred {
    Lines(ExactLines),
    Pending,
    Here,
}

/// Run `job` against the resident registry of THIS thread — the runner's side
/// of [`LinesMode::Defer`]. Refused when the registry no longer holds a handle,
/// or holds different geometry under it than the main side fingerprinted (an
/// assembly solve re-poses a resident solid in place): lines of the wrong
/// geometry cached under this key would be a silent error.
pub fn run_lines_job(job: &LinesJob) -> Result<ExactLines, String> {
    let mut posed = Vec::with_capacity(job.solids.len());
    for solid in &job.solids {
        let brep = brep_kernel::registered_solid_clone(solid.handle)
            .map_err(|error| format!("'{}' (handle {}): {error}", solid.name, solid.handle))?;
        if crate::scene::brep_fingerprint(&brep) != solid.fingerprint {
            return Err(format!("'{}' changed under handle {}", solid.name, solid.handle));
        }
        posed.push(posed_solid(std::borrow::Cow::Owned(brep), &solid.poses).into_owned());
    }
    let solids: Vec<&brep_kernel::BrepSolid> = posed.iter().collect();
    let each: Vec<String> = job.solids.iter().map(LinesSolid::identity).collect();
    Ok(compute_exact_lines(&solids, &each, &job.frame, &job.placed, job.clip))
}

/// The analytic pass itself, relative to `origin` (a placement at the paper
/// origin): the visible runs and, for a section, the cut curves.
fn compute_exact_lines(
    solids: &[&brep_kernel::BrepSolid],
    each: &[String],
    frame: &ViewFrame,
    origin: &PlacedView,
    clip: LinesClip,
) -> ExactLines {
    let trims = solid_trims(each, solids);
    let trims: Vec<&super::hlr::SolidTrims> = trims.iter().map(|t| t.as_ref()).collect();
    match clip {
        LinesClip::Whole => ExactLines {
            runs: super::hlr::visible_edges_with(solids, &trims, frame, origin, &super::hlr::Clip::Whole).runs,
            cut: Vec::new(),
        },
        LinesClip::Circle { radius } => ExactLines {
            runs: super::hlr::visible_edges_with(solids, &trims, frame, origin, &super::hlr::Clip::Circle { radius }).runs,
            cut: Vec::new(),
        },
        LinesClip::Section => {
            // The exact cut: the plane against every face, trimmed. Its curves
            // are the hatch's boundary, the cap that hides what is behind the
            // cut face, and model lines in their own right.
            let cut = super::hlr::section_curves_with(solids, &trims, frame, origin);
            let clip = super::hlr::Clip::Beyond { cap: &cut };
            ExactLines { runs: super::hlr::visible_edges_with(solids, &trims, frame, origin, &clip).runs, cut }
        }
    }
}

/// A placement's exact lines ON the paper at its position: from the cache,
/// else as `mode` says. `None` when the job was handed over and has not come
/// back — the caller draws the mesh approximation.
fn exact_lines_for(
    plan: &ExactPlan,
    frame: &ViewFrame,
    placed: &PlacedView,
    clip: LinesClip,
    mode: &mut LinesMode,
) -> Option<ExactLines> {
    let key = exact_key(&plan.identity, frame, placed.scale, &clip.key());
    let origin = PlacedView { position: [0.0, 0.0], ..placed.clone() };
    let relative = match cached_exact_lines(&key) {
        Some(lines) => lines,
        None => {
            let deferred = match mode {
                LinesMode::Compute => Deferred::Here,
                LinesMode::Defer(defer) => defer(LinesJob {
                    key: key.clone(),
                    placement: placed.id.clone(),
                    solids: plan.solids.clone(),
                    frame: *frame,
                    placed: origin.clone(),
                    clip,
                }),
            };
            let lines = match deferred {
                Deferred::Lines(lines) => lines,
                Deferred::Pending => return None,
                Deferred::Here => {
                    let posed = plan.posed();
                    let solids: Vec<&brep_kernel::BrepSolid> = posed.iter().map(|solid| solid.as_ref()).collect();
                    compute_exact_lines(&solids, &plan.each, frame, &origin, clip)
                }
            };
            store_exact_lines(key, lines.clone());
            lines
        }
    };
    let shift = |lines: Vec<Vec<[f64; 2]>>| -> Vec<Vec<[f64; 2]>> {
        lines
            .into_iter()
            .map(|line| line.into_iter().map(|p| [p[0] + placed.position[0], p[1] + placed.position[1]]).collect())
            .collect()
    };
    Some(ExactLines { runs: shift(relative.runs), cut: shift(relative.cut) })
}

/// How many solids' trims the projection keeps: one for every placement the
/// exact-lines cache keeps.
const TRIM_CACHE_SOLIDS: usize = EXACT_CACHE_ENTRIES;

thread_local! {
    /// Each drawn solid's [`super::hlr::SolidTrims`] by its identity (content
    /// fingerprint and pose): what the pass learns about a trim holds for any
    /// camera, scale or clip, so a new scale, a section or a detail of a part
    /// already drawn builds none of it again. Most recent last.
    static TRIM_CACHE: std::cell::RefCell<std::collections::VecDeque<(String, std::rc::Rc<super::hlr::SolidTrims>)>> =
        const { std::cell::RefCell::new(std::collections::VecDeque::new()) };
}



/// The cached trims of each solid, built for any the cache does not hold.
fn solid_trims(identities: &[String], solids: &[&brep_kernel::BrepSolid]) -> Vec<std::rc::Rc<super::hlr::SolidTrims>> {
    TRIM_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        identities
            .iter()
            .zip(solids)
            .map(|(identity, solid)| {
                let trims = match cache.iter().position(|(key, _)| key == identity) {
                    Some(index) => cache.remove(index).map(|(_, trims)| trims).unwrap(),
                    None => std::rc::Rc::new(super::hlr::SolidTrims::of(solid)),
                };
                cache.push_back((identity.clone(), trims.clone()));
                while cache.len() > TRIM_CACHE_SOLIDS {
                    cache.pop_front();
                }
                trims
            })
            .collect()
    })
}

/// The analytic pass's lines for one placement, RELATIVE to its position
/// (paper millimetres from the placement point): the visible runs and, for a
/// section, the cut curves. What a [`LinesJob`] answers with.
#[derive(Debug, Clone, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct ExactLines {
    pub runs: Vec<Vec<[f64; 2]>>,
    pub cut: Vec<Vec<[f64; 2]>>,
}

/// How many placements' exact lines are kept.
const EXACT_CACHE_ENTRIES: usize = 64;

thread_local! {
    /// The exact lines by what they depend on — the geometry's identity, the
    /// camera frame, the scale and the clip, never the placement's POSITION —
    /// so dragging a placement, dragging a PMI label or editing a dimension
    /// re-projects the sheet without re-running the pass. Most recent last.
    /// On the UI thread it is also where a job's answer lands
    /// ([`store_exact_lines`]).
    static EXACT_CACHE: std::cell::RefCell<std::collections::VecDeque<(String, ExactLines)>> =
        const { std::cell::RefCell::new(std::collections::VecDeque::new()) };
}

/// The cached exact lines for `key`, relative to the placement point.
pub(crate) fn cached_exact_lines(key: &str) -> Option<ExactLines> {
    EXACT_CACHE.with(|cache| cache.borrow().iter().find(|(k, _)| k == key).map(|(_, lines)| lines.clone()))
}

/// Keep `lines` under `key`, most recent last — a pass run here, or a job's
/// answer from the runner.
pub(crate) fn store_exact_lines(key: String, lines: ExactLines) {
    EXACT_CACHE.with(|cache| {
        let mut cache = cache.borrow_mut();
        cache.retain(|(k, _)| *k != key);
        cache.push_back((key, lines));
        while cache.len() > EXACT_CACHE_ENTRIES {
            cache.pop_front();
        }
    });
}

/// The part of an exact-lines key a placement's frame, scale and clip give.
fn exact_key(identity: &str, frame: &ViewFrame, scale: f64, clip: &str) -> String {
    let bits = |v: [f64; 3]| format!("{:x},{:x},{:x}", v[0].to_bits(), v[1].to_bits(), v[2].to_bits());
    format!(
        "{identity}|{}|{}|{}|{}|{:x}|{clip}",
        bits(frame.target),
        bits(frame.right),
        bits(frame.up),
        bits(frame.forward),
        scale.to_bits()
    )
}

/// `solid` with every model point mapped through the affine `pose`: vertices,
/// edge curves' and carriers' control points (a rational control point is
/// mapped as its Euclidean point and re-weighted). Carriers are REBUILT
/// rather than cloned, so no analytic recognition cached on the unposed
/// surface survives onto the posed one. Pcurves live in parameter space and
/// are unchanged.
fn pose_solid(solid: &brep_kernel::BrepSolid, pose: &dyn Fn([f64; 3]) -> [f64; 3]) -> brep_kernel::BrepSolid {
    let map = |cp: &brep_kernel::Vec4| -> brep_kernel::Vec4 {
        let w = if cp.w != 0.0 { cp.w } else { 1.0 };
        let p = pose([cp.x / w, cp.y / w, cp.z / w]);
        brep_kernel::Vec4 { x: p[0] * w, y: p[1] * w, z: p[2] * w, w: cp.w }
    };
    let mut out = solid.clone();
    for vertex in &mut out.vertices {
        let p = pose([vertex.point.x, vertex.point.y, vertex.point.z]);
        vertex.point = brep_kernel::Vec3::new(p[0], p[1], p[2]);
    }
    for edge in &mut out.edges {
        let points = edge.curve.control_points.iter().map(map).collect();
        if let Ok(curve) = brep_kernel::NurbsCurve::new(edge.curve.degree, edge.curve.knots.clone(), points) {
            edge.curve = curve;
        }
    }
    for shell in &mut out.shells {
        for face in &mut shell.faces {
            let s = &face.surface;
            let points = s.control_points.iter().map(|row| row.iter().map(map).collect()).collect();
            if let Ok(surface) =
                brep_kernel::NurbsSurface::new(s.degree_u, s.degree_v, s.knots_u.clone(), s.knots_v.clone(), points)
            {
                face.surface = surface;
            }
        }
    }
    out
}

/// Polylines as the two-point segments [`hatch`] counts crossings against.
fn segments_of(lines: &[Vec<[f64; 2]>]) -> Vec<Vec<[f64; 2]>> {
    lines.iter().flat_map(|line| line.windows(2).map(|w| vec![w[0], w[1]])).collect()
}

// ---------------------------------------------------------------------------
// The drawn meshes
// ---------------------------------------------------------------------------

/// The solids a saved view draws: visible in the scene, not hidden by the
/// view's own `display.hidden` set, and not a committed SKETCH (a drawing
/// shows the model, not the sketches it was built from).
fn drawn_solids<'a>(scene: &'a RenderScene, view: &PmiView) -> Vec<&'a crate::scene::SolidDisplay> {
    scene
        .solids()
        .iter()
        .filter(|solid| solid.visible && !solid.is_sketch)
        .filter(|solid| !view.display.hidden.iter().any(|name| name == &solid.name))
        .filter(|solid| solid.mesh.indices.len() >= 3)
        .collect()
}

/// The names of every solid some placement of `sheet` draws: the solids whose
/// topology its exact pass needs.
pub fn sheet_solid_names(scene: &RenderScene, sheet: &super::Sheet, views: &[PmiView]) -> Vec<String> {
    let mut names: Vec<String> = Vec::new();
    for placed in &sheet.views {
        let Some(view) = views.iter().find(|view| view.id == placed.view) else { continue };
        for solid in drawn_solids(scene, view) {
            if !names.contains(&solid.name) {
                names.push(solid.name.clone());
            }
        }
    }
    names
}

/// The DISPLAY each drawn solid is read from: the un-posed one where the
/// viewport's active view posed it in place, so every placement starts from
/// the model as built and applies its own view's poses.
fn drawn_displays<'a>(scene: &'a RenderScene, unposed: &'a Unposed, view: &PmiView) -> Vec<&'a crate::scene::SolidDisplay> {
    drawn_solids(scene, view).into_iter().map(|solid| unposed.get(&solid.name).unwrap_or(solid)).collect()
}


/// Every drawn solid's DISPLAY as the mesh approximation reads it
/// ([`super::hlr::visible_mesh_lines`]), POSED by the view's own explode rows
/// exactly as the exact pass poses the B-rep: the triangles with their faces,
/// and every display edge that is a boundary of the model — not an auxiliary
/// or centreline edge — with the two topology VERTICES it runs between.
///
/// **The ends are the kernel's adjacency, recovered exactly.** A display edge
/// carries no vertex ids, but the tessellator writes an edge's first and last
/// samples AS its start and end vertex's own points
/// (`watertight_tessellation::edge_sampling`), and the display narrows both to
/// `f32` the same way. So an end whose `f32` bits are one display vertex's is
/// that vertex, with no distance and no bar; an end shared by two vertices
/// narrowed to one `f32` point is no vertex (nothing chains through it), and
/// so is an end that is none. The ends are read BEFORE the pose, off the
/// display's own numbers.
fn mesh_solids(displays: &[&crate::scene::SolidDisplay], report: Option<&PmiViewReport>) -> Vec<super::hlr::MeshSolid> {
    let world = |p: &[f32; 3]| [p[0] as f64, p[1] as f64, p[2] as f64];
    displays
        .iter()
        .map(|solid| {
            let poses = poses_of(report, &solid.name);
            let pose = |p: [f64; 3]| poses.iter().fold(p, |p, pose| pose.apply(p));
            let count = solid.mesh.positions.len();
            let (triangles, faces) = solid
                .mesh
                .indices
                .chunks_exact(3)
                .enumerate()
                .filter(|(_, c)| c.iter().all(|i| (*i as usize) < count))
                .map(|(t, c)| ([c[0], c[1], c[2]], solid.mesh.face_ids.get(t).copied().unwrap_or(u32::MAX)))
                .unzip();
            let bits = |p: [f32; 3]| [p[0].to_bits(), p[1].to_bits(), p[2].to_bits()];
            let mut at: std::collections::HashMap<[u32; 3], Option<u64>> = std::collections::HashMap::new();
            for vertex in &solid.vertices {
                let p = vertex.position;
                at.entry(bits([p[0] as f32, p[1] as f32, p[2] as f32]))
                    .and_modify(|id| *id = None)
                    .or_insert(Some(vertex.topo_id));
            }
            let vertex_of = |p: &[f32; 3]| at.get(&bits(*p)).copied().flatten();
            super::hlr::MeshSolid {
                positions: solid.mesh.positions.iter().map(|p| pose(world(p))).collect(),
                triangles,
                faces,
                edges: solid
                    .edges
                    .iter()
                    .filter(|edge| !edge.aux && !edge.centerline && edge.polyline.len() >= 2)
                    .map(|edge| super::hlr::MeshEdge {
                        points: edge.polyline.iter().map(|p| pose(world(p))).collect(),
                        ends: vertex_of(&edge.polyline[0]).zip(vertex_of(edge.polyline.last().unwrap())),
                    })
                    .collect(),
            }
        })
        .collect()
}

/// How many placements' mesh approximations are kept.
const MESH_CACHE_ENTRIES: usize = 16;

thread_local! {
    /// The mesh approximation's lines by what they depend on — each drawn
    /// display's content revision and poses, the camera frame, the scale, the
    /// clip and a section's plane — never the placement's position, exactly as
    /// the exact lines are kept. A placement waiting for its exact pass is
    /// re-projected by every edit on the sheet, and the approximation of a
    /// large part is tens of milliseconds a time. Most recent last.
    static MESH_CACHE: std::cell::RefCell<std::collections::VecDeque<(String, ExactLines)>> =
        const { std::cell::RefCell::new(std::collections::VecDeque::new()) };
}

/// A placement's mesh approximation ON the paper at its position: the runs
/// and, for a section, the cut through the display triangles.
#[allow(clippy::too_many_arguments)]
fn mesh_lines_for(
    scene: &RenderScene,
    unposed: &Unposed,
    view: &PmiView,
    report: Option<&PmiViewReport>,
    frame: &ViewFrame,
    placed: &PlacedView,
    clip: LinesClip,
    plane: Option<&ClipPlane>,
) -> ExactLines {
    let displays = drawn_displays(scene, unposed, view);
    let mut identity = String::from("mesh|");
    for display in &displays {
        identity.push_str(&format!("{}@{};", display.name, display.revision));
        for pose in poses_of(report, &display.name) {
            identity.push_str(&pose.identity());
        }
    }
    let bits = |v: [f64; 3]| format!("{:x},{:x},{:x}", v[0].to_bits(), v[1].to_bits(), v[2].to_bits());
    let plane_key = plane.map(|plane| format!("{}|{}", bits(plane.point), bits(plane.normal))).unwrap_or_default();
    let key = exact_key(&identity, frame, placed.scale, &format!("{}{plane_key}", clip.key()));
    let hit = MESH_CACHE.with(|cache| cache.borrow().iter().find(|(k, _)| *k == key).map(|(_, lines)| lines.clone()));
    let relative = match hit {
        Some(lines) => lines,
        None => {
            let origin = PlacedView { position: [0.0, 0.0], ..placed.clone() };
            let meshes = mesh_solids(&displays, report);
            let lines = match (clip, plane) {
                (LinesClip::Section, Some(plane)) => {
                    let cut: Vec<Vec<[f64; 2]>> = mesh_cut(&meshes, plane)
                        .iter()
                        .map(|segment| {
                            segment
                                .iter()
                                .map(|p| {
                                    let v = frame.to_view(*p);
                                    to_paper(&origin, v[0], v[1])
                                })
                                .collect()
                        })
                        .collect();
                    let beyond = super::hlr::Clip::Beyond { cap: &cut };
                    ExactLines { runs: super::hlr::visible_mesh_lines(&meshes, frame, &origin, &beyond).runs, cut }
                }
                (LinesClip::Circle { radius }, _) => ExactLines {
                    runs: super::hlr::visible_mesh_lines(&meshes, frame, &origin, &super::hlr::Clip::Circle { radius }).runs,
                    cut: Vec::new(),
                },
                _ => ExactLines {
                    runs: super::hlr::visible_mesh_lines(&meshes, frame, &origin, &super::hlr::Clip::Whole).runs,
                    cut: Vec::new(),
                },
            };
            MESH_CACHE.with(|cache| {
                let mut cache = cache.borrow_mut();
                cache.push_back((key, lines.clone()));
                while cache.len() > MESH_CACHE_ENTRIES {
                    cache.pop_front();
                }
            });
            lines
        }
    };
    let shift = |lines: Vec<Vec<[f64; 2]>>| -> Vec<Vec<[f64; 2]>> {
        lines
            .into_iter()
            .map(|line| line.into_iter().map(|p| [p[0] + placed.position[0], p[1] + placed.position[1]]).collect())
            .collect()
    };
    ExactLines { runs: shift(relative.runs), cut: shift(relative.cut) }
}

// ---------------------------------------------------------------------------
// The cutting plane
// ---------------------------------------------------------------------------

/// The half-space a SECTION placement draws: everything with
/// `(p \u{2212} point) \u{00B7} normal >= 0` is KEPT, and `normal` is the direction the
/// section camera looks.
///
/// Because the camera looks ALONG the normal, the cutting plane is a plane of
/// CONSTANT view depth — and the plane passes through the camera's target, so
/// that depth is exactly zero. Everything that follows leans on it: the pass
/// keeps what is at or beyond depth zero, the cut region is a cap at one depth,
/// and the hatch is a plain two-dimensional fill. A section view is defined as
/// the one that looks straight down its own cutting plane's normal.
#[derive(Debug, Clone, Copy)]
pub struct ClipPlane {
    pub point: [f64; 3],
    pub normal: [f64; 3],
}

/// The mesh approximation's cut: the segment where the plane crosses each
/// display triangle that keeps area on the kept side — the section's outline
/// and the region its hatch fills and its cap hides behind, while the exact
/// cut ([`super::hlr::section_curves`]) waits for the B-rep.
fn mesh_cut(meshes: &[super::hlr::MeshSolid], plane: &ClipPlane) -> Vec<[[f64; 3]; 2]> {
    // The on-plane bar, relative to the model's own extent: a vertex this
    // close to the plane IS on it, and treating it as one side or the other
    // would open the cut boundary by one triangle.
    let mut extent: f64 = 0.0;
    for mesh in meshes {
        for p in &mesh.positions {
            extent = extent.max(dot(sub(*p, plane.point), plane.normal).abs());
        }
    }
    let eps = (extent * 1e-9).max(f64::MIN_POSITIVE);

    let mut cut: Vec<[[f64; 3]; 2]> = Vec::new();
    for mesh in meshes {
        let side: Vec<f64> = mesh
            .positions
            .iter()
            .map(|p| dot(sub(*p, plane.point), plane.normal))
            .collect();
        for tri in &mesh.triangles {
            let corners = [tri[0] as usize, tri[1] as usize, tri[2] as usize];
            let d = [side[corners[0]], side[corners[1]], side[corners[2]]];
            if d.iter().all(|v| *v < -eps) || d.iter().all(|v| *v > eps) {
                continue; // wholly cut away, or wholly kept
            }
            // Straddling (or touching): walk the edges, counting what is on or
            // in front of the plane and collecting the crossing points.
            let mut kept = 0;
            let mut on_plane: Vec<[f64; 3]> = Vec::with_capacity(2);
            for k in 0..3 {
                let (i, j) = (k, (k + 1) % 3);
                let (pi, pj) = (mesh.positions[corners[i]], mesh.positions[corners[j]]);
                let (di, dj) = (d[i], d[j]);
                if di >= -eps {
                    kept += 1;
                    if di.abs() <= eps {
                        on_plane.push(pi);
                    }
                }
                if (di > eps && dj < -eps) || (di < -eps && dj > eps) {
                    let t = di / (di - dj);
                    kept += 1;
                    on_plane.push([
                        pi[0] + (pj[0] - pi[0]) * t,
                        pi[1] + (pj[1] - pi[1]) * t,
                        pi[2] + (pj[2] - pi[2]) * t,
                    ]);
                }
            }
            // A cut edge belongs to the triangle that KEPT AREA on it, and to
            // exactly that one. Two on-plane points is the edge; three is a
            // triangle lying IN the plane, whose own edges already bound the
            // region, and one is a triangle touching the plane at a corner.
            //
            // The kept-area half is not a nicety. A cutting plane through a
            // mesh VERTEX RING — which is what a plane through the middle of a
            // tessellated cylinder is, and hardly a rare accident — gives the
            // ring's edge to two triangles at once: the one above it, which
            // keeps a real triangle, and the one below, which is cut away
            // entirely and keeps only that edge. Counting both would put the
            // boundary in twice, and an even-odd fill crossing a boundary
            // twice reads the inside as the outside.
            if kept >= 3 && on_plane.len() == 2 {
                cut.push([on_plane[0], on_plane[1]]);
            }
        }
    }
    cut
}

/// The hatch over a cut region: parallel 45\u{00B0} paper lines, each kept only
/// where an EVEN-ODD crossing count against the cut boundary says it is inside
/// the material.
///
/// Even-odd against the SEGMENTS rather than against chained loops, on purpose:
/// the cut of a closed solid is a closed set of curves however the segments
/// happen to group, so the parity is right without a chaining pass that could
/// break a loop at a vertex three edges meet in — and a section through a bore
/// is exactly such a region, an outer loop with a hole in it.
fn hatch(cut: &[Vec<[f64; 2]>]) -> Vec<Vec<[f64; 2]>> {
    if cut.is_empty() {
        return Vec::new();
    }
    // The hatch runs up and to the right; `n` is its normal, so a line of the
    // family is `p \u{00B7} n = c` and a point's place ALONG it is `p \u{00B7} u`.
    let d = std::f64::consts::FRAC_1_SQRT_2;
    let u = [d, -d];
    let n = [d, d];
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for segment in cut {
        for p in segment {
            let c = p[0] * n[0] + p[1] * n[1];
            lo = lo.min(c);
            hi = hi.max(c);
        }
    }
    if !lo.is_finite() || hi <= lo {
        return Vec::new();
    }
    let mut spacing = HATCH_SPACING_MM;
    while (hi - lo) / spacing > MAX_HATCH_LINES as f64 {
        spacing *= 2.0;
    }
    // Start on a multiple of the spacing, so two sections of the same drawing
    // hatch in step rather than each from its own corner.
    let first = (lo / spacing).ceil() * spacing;
    let mut out = Vec::new();
    let mut c = first;
    while c <= hi {
        let mut crossings: Vec<f64> = Vec::new();
        for segment in cut {
            let (a, b) = (segment[0], segment[segment.len() - 1]);
            let (ca, cb) = (a[0] * n[0] + a[1] * n[1], b[0] * n[0] + b[1] * n[1]);
            // Half-open in `c`, so a vertex two segments share is counted once
            // and the parity survives it.
            if (ca <= c) == (cb <= c) {
                continue;
            }
            let t = (c - ca) / (cb - ca);
            let hit = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            crossings.push(hit[0] * u[0] + hit[1] * u[1]);
        }
        crossings.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        for pair in crossings.chunks_exact(2) {
            if pair[1] - pair[0] < 1e-9 {
                continue;
            }
            out.push(vec![
                [u[0] * pair[0] + n[0] * c, u[1] * pair[0] + n[1] * c],
                [u[0] * pair[1] + n[0] * c, u[1] * pair[1] + n[1] * c],
            ]);
        }
        c += spacing;
    }
    out
}

// ---------------------------------------------------------------------------
// Anchor candidates
// ---------------------------------------------------------------------------

/// The angular span a circular edge must cover before it is offered as a
/// CIRCLE anchor. Three samples fix a circle exactly, but three samples a
/// degree apart fix it badly: a short fillet rim would hand a radial dimension
/// a centre with no precision behind it.
const CIRCLE_MIN_SPAN_DEG: f64 = 20.0;

/// How far a sampled point may sit off the fitted circle, as a fraction of the
/// radius, before the edge is not a circle. The display polyline is SAMPLED on
/// the curve, so a real circular edge misses by f32 rounding alone; anything
/// looser than this is a spline that happens to start out round.
const CIRCLE_FIT_TOLERANCE: f64 = 1e-4;

/// How far a sampled point may sit off the CHORD, as a fraction of the chord's
/// length, before the display edge is not STRAIGHT. The same bar as the circle
/// fit and for the same reason: the polyline is sampled on the curve, so a real
/// straight edge misses its own chord by f32 rounding alone.
const STRAIGHT_TOLERANCE: f64 = 1e-4;

/// Every point of this placement's projection a sheet dimension can anchor to.
///
/// One candidate per display VERTEX, three per named display EDGE (its two
/// ends and its midpoint by arc length) and one more per CIRCULAR edge, at the
/// circle's projected centre. Visibility plays no part: the hidden-line pass
/// decides what ink the model leaves on the paper, and the anchor space is the
/// model's topology.
fn anchor_points(
    scene: &RenderScene,
    unposed: &Unposed,
    view: &PmiView,
    report: Option<&PmiViewReport>,
    frame: &ViewFrame,
    placed: &PlacedView,
) -> Vec<AnchorPoint> {
    let project = |p: [f64; 3]| -> [f64; 2] {
        let v = frame.to_view(p);
        to_paper(placed, v[0], v[1])
    };
    let mut out = Vec::new();
    for solid in drawn_displays(scene, unposed, view) {
        // The anchors are where the placement DRAWS the model: posed by its
        // own view's explode rows, as the lines are.
        let poses = poses_of(report, &solid.name);
        let pose = |p: [f64; 3]| poses.iter().fold(p, |p, pose| pose.apply(p));
        for vertex in &solid.vertices {
            out.push(AnchorPoint {
                anchor: super::dimension::SheetAnchor::vertex(&placed.id, &solid.name, vertex.topo_id)
                    .to_ref(),
                kind: super::dimension::VERTEX.to_string(),
                at: project(pose(vertex.position)),
                model: pose(vertex.position),
                dir: None,
                radius: None,
                u: None,
                v: None,
            });
        }
        for edge in &solid.edges {
            // An edge with no name cannot be REFERRED to, and an auxiliary or
            // centreline display edge is not a BREP boundary — neither is a
            // point of the model a drawing may dimension.
            if edge.name.is_empty() || edge.aux || edge.centerline || edge.polyline.len() < 2 {
                continue;
            }
            let points: Vec<[f64; 3]> = edge
                .polyline
                .iter()
                .map(|p| pose([p[0] as f64, p[1] as f64, p[2] as f64]))
                .collect();
            // The EDGE's own paper direction, once — every fraction of it
            // carries the same one, because it is the edge's and not the
            // point's. `None` on a curved edge, which an angular dimension
            // refuses by name.
            let dir = straight_chord(&points).and_then(|d| {
                let paper = [dot(d, frame.right), -dot(d, frame.up)];
                let length = (paper[0] * paper[0] + paper[1] * paper[1]).sqrt();
                (length > 1e-9).then(|| [paper[0] / length, paper[1] / length])
            });
            for fraction in [0.0, 0.5, 1.0] {
                let Some(point) = along(&points, fraction) else { continue };
                out.push(AnchorPoint {
                    anchor: super::dimension::SheetAnchor::edge(
                        &placed.id,
                        &solid.name,
                        &edge.name,
                        fraction,
                    )
                    .to_ref(),
                    kind: super::dimension::EDGE.to_string(),
                    at: project(point),
                    model: point,
                    dir,
                    radius: None,
                    u: None,
                    v: None,
                });
            }
            if let Some((centre, axis, radius)) = fit_circle(&points) {
                // The projected circle is an ellipse: `u` and `v` are the paper
                // images of one pair of conjugate radii, which is everything a
                // radial dimension needs to find where its leader meets the rim.
                let u3 = norm(sub(points[0], centre));
                let v3 = cross(axis, u3);
                let image = |d: [f64; 3]| -> [f64; 2] {
                    [
                        dot(d, frame.right) * radius * placed.scale,
                        -dot(d, frame.up) * radius * placed.scale,
                    ]
                };
                out.push(AnchorPoint {
                    anchor: super::dimension::SheetAnchor::circle(&placed.id, &solid.name, &edge.name)
                        .to_ref(),
                    kind: super::dimension::CIRCLE.to_string(),
                    at: project(centre),
                    model: centre,
                    dir: None,
                    radius: Some(radius),
                    u: Some(image(u3)),
                    v: Some(image(v3)),
                });
            }
        }
    }
    out
}

/// The unit MODEL direction of a straight display edge, or `None` when the
/// polyline is curved or closed on itself.
///
/// Straight means every SAMPLE lies on the chord between the two ends, within
/// [`STRAIGHT_TOLERANCE`] of its length. Judged in the model rather than on the
/// paper on purpose: a circular edge seen edge-on projects to a straight
/// segment, and an angle measured to it would be an angle to nothing.
fn straight_chord(points: &[[f64; 3]]) -> Option<[f64; 3]> {
    let (first, last) = (*points.first()?, *points.last()?);
    let span = sub(last, first);
    let length = dot(span, span).sqrt();
    if length < 1e-12 {
        // A closed edge (a full circle) has no chord at all.
        return None;
    }
    let direction = [span[0] / length, span[1] / length, span[2] / length];
    let bar = length * STRAIGHT_TOLERANCE;
    for point in points {
        let offset = sub(*point, first);
        let along = dot(offset, direction);
        let away = [
            offset[0] - direction[0] * along,
            offset[1] - direction[1] * along,
            offset[2] - direction[2] * along,
        ];
        if dot(away, away).sqrt() > bar {
            return None;
        }
    }
    Some(direction)
}

/// The point `fraction` of the way along a polyline BY ARC LENGTH.
fn along(points: &[[f64; 3]], fraction: f64) -> Option<[f64; 3]> {
    if points.len() < 2 {
        return None;
    }
    let steps: Vec<f64> = points
        .windows(2)
        .map(|pair| dot(sub(pair[1], pair[0]), sub(pair[1], pair[0])).sqrt())
        .collect();
    let total: f64 = steps.iter().sum();
    if !(total > 0.0) {
        return Some(points[0]);
    }
    let target = total * fraction.clamp(0.0, 1.0);
    let mut walked = 0.0;
    for (index, step) in steps.iter().enumerate() {
        if walked + step >= target || index + 1 == steps.len() {
            let t = if *step > 0.0 { ((target - walked) / step).clamp(0.0, 1.0) } else { 0.0 };
            let (a, b) = (points[index], points[index + 1]);
            return Some([
                a[0] + (b[0] - a[0]) * t,
                a[1] + (b[1] - a[1]) * t,
                a[2] + (b[2] - a[2]) * t,
            ]);
        }
        walked += step;
    }
    points.last().copied()
}

/// `(centre, unit axis, radius)` when every sampled point of `points` lies on
/// one circle, else `None`.
///
/// The circle through three of the samples, then VERIFIED against all of them:
/// a display polyline is sampled ON its curve, so a real circular edge's
/// samples sit on that circle to f32 rounding while a spline's do not. The
/// axis follows the right-hand rule along the polyline's own direction, the
/// kernel's `SelectionGeometry::Circle` convention.
fn fit_circle(points: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3], f64)> {
    if points.len() < 4 {
        return None;
    }
    let last = points.len() - 1;
    let (a, b, c) = (points[0], points[last / 3], points[last * 2 / 3]);
    let (ab, ac) = (sub(b, a), sub(c, a));
    let n = cross(ab, ac);
    let n2 = dot(n, n);
    if n2 < 1e-24 {
        return None;
    }
    // The circumcentre of a triangle, in the triangle's own plane.
    let centre = {
        let (ab2, ac2) = (dot(ab, ab), dot(ac, ac));
        let to = cross(
            [
                ab2 * ac[0] - ac2 * ab[0],
                ab2 * ac[1] - ac2 * ab[1],
                ab2 * ac[2] - ac2 * ab[2],
            ],
            n,
        );
        [
            a[0] + to[0] / (2.0 * n2),
            a[1] + to[1] / (2.0 * n2),
            a[2] + to[2] / (2.0 * n2),
        ]
    };
    let axis = norm(n);
    let radius = dot(sub(a, centre), sub(a, centre)).sqrt();
    if !(radius > 1e-12) {
        return None;
    }
    let bar = radius * CIRCLE_FIT_TOLERANCE;
    for p in points {
        let d = sub(*p, centre);
        if (dot(d, d).sqrt() - radius).abs() > bar || dot(d, axis).abs() > bar {
            return None;
        }
    }
    // A rim too short to fit reliably is not offered: three samples a degree
    // apart give a centre with no precision behind it.
    let first = norm(sub(points[0], centre));
    let mut span = 0.0;
    for p in points {
        let d = norm(sub(*p, centre));
        let angle = dot(d, first).clamp(-1.0, 1.0).acos();
        span = f64::max(span, angle);
    }
    if span.to_degrees() < CIRCLE_MIN_SPAN_DEG {
        return None;
    }
    Some((centre, axis, radius))
}

// ---------------------------------------------------------------------------
// Annotations
// ---------------------------------------------------------------------------

/// A blocking box in paper millimetres: `centre`, the paper images of one
/// LOCAL unit along the box's own two axes, and the half extents in those
/// units. The axes are a text run's own — under an oblique projection they
/// are sheared, so the box is a parallelogram and not a rotated rectangle.
#[derive(Debug, Clone, Copy)]
struct Box2 {
    centre: [f64; 2],
    right: [f64; 2],
    up: [f64; 2],
    half: [f64; 2],
}

impl Box2 {
    /// `p` in the box's own units, or `None` when the basis is DEGENERATE —
    /// an edge-on plane projects its two axes onto one line, and a box with
    /// no area blocks nothing.
    fn local(&self, p: [f64; 2]) -> Option<[f64; 2]> {
        let det = self.right[0] * self.up[1] - self.right[1] * self.up[0];
        if det.abs() < 1e-12 {
            return None;
        }
        let d = [p[0] - self.centre[0], p[1] - self.centre[1]];
        Some([
            (d[0] * self.up[1] - d[1] * self.up[0]) / det,
            (d[1] * self.right[0] - d[0] * self.right[1]) / det,
        ])
    }

    /// The box a text run occupies: its own basis, its character extent in
    /// cap heights, and [`TEXT_PAD`] of clearance on every side.
    fn of_text(run: &SheetText) -> Box2 {
        let rows = run.text.lines().count().max(1) as f64;
        Box2 {
            centre: run.anchor,
            right: run.right,
            up: run.up,
            half: [text_half_width(&run.text, 1.0), rows * 0.5 + TEXT_PAD],
        }
    }
}

/// How far a FLAT run of `text` at cap height `height` blocks a line either
/// side of its anchor along its reading direction: its character extent at
/// [`TEXT_ADVANCE`] plus [`TEXT_PAD`] of clearance — the box
/// [`break_for_text_boxes`] breaks a line with.
pub(super) fn text_half_width(text: &str, height: f64) -> f64 {
    let columns = text.lines().map(|line| line.chars().count()).max().unwrap_or(0) as f64;
    (columns * TEXT_ADVANCE * 0.5 + TEXT_PAD) * height
}

/// Break `lines` where a text run — or a framed symbol's outline — sits.
///
/// The kernel's layout runs the dimension line THROUGH the label — the
/// viewport hides that under the label's chip and the STEP presentation leaves
/// it — but a sheet has no chip and a value struck through by its own
/// dimension line is not a drawing. Breaking the line is a SHEET-layer
/// decision, so it is made here rather than in the shared layout: the gap is
/// the text's own box plus a clearance, subtracted from every line of the same
/// annotation. A framed symbol (an FCF, a datum) adds its FRAME as a box with
/// no clearance, which is what re-attaches its leader to the box's nearest
/// edge when the frame has been flattened out from under it. The frame's own
/// outline is never offered to this pass — it would erase itself.
fn break_for_text(lines: Vec<Vec<[f64; 2]>>, boxes: &[Box2]) -> Vec<Vec<[f64; 2]>> {
    if boxes.is_empty() {
        return lines;
    }
    let mut out: Vec<Vec<[f64; 2]>> = Vec::new();
    for line in lines {
        // Each run of consecutive UNBROKEN segments becomes one polyline.
        let mut current: Vec<[f64; 2]> = Vec::new();
        for pair in line.windows(2) {
            let (p, q) = (pair[0], pair[1]);
            let mut kept = visible_spans(p, q, boxes);
            if kept.is_empty() {
                if current.len() >= 2 {
                    out.push(std::mem::take(&mut current));
                } else {
                    current.clear();
                }
                continue;
            }
            let full = kept.len() == 1 && kept[0].0 <= 0.0 && kept[0].1 >= 1.0;
            let at = |t: f64| [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t];
            if full {
                if current.is_empty() {
                    current.push(p);
                }
                current.push(q);
                continue;
            }
            // A broken segment ends whatever was running and contributes its
            // own pieces.
            if !kept.is_empty() && kept[0].0 <= 0.0 {
                let (_, end) = kept.remove(0);
                if current.is_empty() {
                    current.push(p);
                }
                current.push(at(end));
            }
            if current.len() >= 2 {
                out.push(std::mem::take(&mut current));
            } else {
                current.clear();
            }
            for (start, end) in kept {
                if end >= 1.0 {
                    current = vec![at(start), q];
                } else {
                    out.push(vec![at(start), at(end)]);
                }
            }
        }
        if current.len() >= 2 {
            out.push(current);
        }
    }
    out
}

/// Break `lines` around the text `runs` — the door [`super::dimension`] uses,
/// so a sheet dimension's value is cleared out of its own dimension line by
/// the very same pass that clears a placed annotation's.
pub(super) fn break_for_text_boxes(
    lines: Vec<Vec<[f64; 2]>>,
    runs: &[SheetText],
) -> Vec<Vec<[f64; 2]>> {
    let boxes: Vec<Box2> = runs.iter().map(Box2::of_text).collect();
    break_for_text(lines, &boxes)
}

/// The parameter spans of segment `p`→`q` that lie OUTSIDE every box, as
/// `(start, end)` pairs in increasing order.
fn visible_spans(p: [f64; 2], q: [f64; 2], boxes: &[Box2]) -> Vec<(f64, f64)> {
    let mut blocked: Vec<(f64, f64)> = Vec::new();
    for box2 in boxes {
        let (Some(a), Some(b)) = (box2.local(p), box2.local(q)) else {
            continue;
        };
        // Liang-Barsky against the box's two slabs.
        let (mut t0, mut t1) = (0.0f64, 1.0f64);
        let mut inside = true;
        for (axis, half) in [(0usize, box2.half[0]), (1usize, box2.half[1])] {
            let delta = b[axis] - a[axis];
            if delta.abs() < 1e-12 {
                if a[axis].abs() > half {
                    inside = false;
                    break;
                }
                continue;
            }
            let (lo, hi) = ((-half - a[axis]) / delta, (half - a[axis]) / delta);
            let (lo, hi) = if lo <= hi { (lo, hi) } else { (hi, lo) };
            t0 = t0.max(lo);
            t1 = t1.min(hi);
            if t0 > t1 {
                inside = false;
                break;
            }
        }
        if inside && t1 > t0 {
            blocked.push((t0, t1));
        }
    }
    if blocked.is_empty() {
        return vec![(0.0, 1.0)];
    }
    blocked.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
    let mut spans = Vec::new();
    let mut cursor = 0.0f64;
    for (start, end) in blocked {
        if start > cursor + 1e-9 {
            spans.push((cursor, start.min(1.0)));
        }
        cursor = cursor.max(end);
        if cursor >= 1.0 {
            break;
        }
    }
    if cursor < 1.0 - 1e-9 {
        spans.push((cursor, 1.0));
    }
    spans
}

/// Whether an annotation plane is EDGE-ON to a view looking along `forward`:
/// its normal perpendicular to the viewing direction, so the plane and
/// everything laid out in it project onto one line. An annotation with no
/// picked plane is laid out in the view's own plane and is never edge-on.
pub fn plane_is_edge_on(row: &PmiAnnotationReport, forward: [f64; 3]) -> bool {
    match row.plane.as_ref() {
        None => false,
        Some(plane) => dot(norm(plane.normal), forward).abs() < EDGE_ON_COS,
    }
}

/// The view's PMI dimensions a sheet dimension can inherit a tolerance from:
/// every enabled, resolved free linear, radial or diametral row whose params
/// carry a tolerance block or the reference flag (the kernel's own
/// `ToleranceBlock::read`, against the document's expressions), with its
/// measured points through the view camera.
fn tolerance_sources(
    report: Option<&PmiViewReport>,
    view: &PmiView,
    frame: &ViewFrame,
    placed: &PlacedView,
    env: &brep_kernel::Env,
) -> Vec<ToleranceSource> {
    let Some(report) = report else { return Vec::new() };
    let project = |p: [f64; 3]| -> [f64; 2] {
        let v = frame.to_view(p);
        to_paper(placed, v[0], v[1])
    };
    let mut out = Vec::new();
    for row in report.annotations.iter().filter(|row| row.enabled && row.status == PmiStatus::Ok) {
        let Some(annotation) = view.annotations.iter().find(|annotation| annotation.id() == row.id) else { continue };
        let Ok(block) = brep_kernel::ToleranceBlock::read(annotation, env) else { continue };
        let reference = annotation.flag("isReference");
        if block.mode == brep_kernel::ToleranceMode::None && !reference {
            continue;
        }
        let measures = match &row.geometry {
            brep_kernel::PmiGeometry::Linear { a, b, component: None } => Measures::Linear { a: project(*a), b: project(*b) },
            brep_kernel::PmiGeometry::Radial { center, radius, diameter, sphere: false, .. } => {
                Measures::Radial { centre: project(*center), radius: *radius, diameter: *diameter }
            }
            _ => continue,
        };
        out.push(ToleranceSource { id: row.id.clone(), measures, block, reference });
    }
    out
}

fn annotations(
    report: Option<&PmiViewReport>,
    view: &PmiView,
    frame: &ViewFrame,
    placed: &PlacedView,
) -> Vec<AnnotationDrawing> {
    let Some(report) = report else {
        return Vec::new();
    };
    // The text is sized on PAPER: a view's text size is in points, so the
    // model-unit cap height the layout needs is the paper height over the
    // placement's scale.
    let paper_text = view.display.text_size_pt * MM_PER_POINT;
    let text_height = paper_text / placed.scale.max(1e-9);
    let style = PmiLayoutStyle {
        arrow: text_height * ARROW_PER_TEXT,
        text_height,
        view_dir: frame.forward,
        view_up: frame.up,
        plane: None,
    };
    let flatten = placed.flatten_text;
    let mut out = Vec::new();
    for row in &report.annotations {
        if !row.enabled || row.status != PmiStatus::Ok {
            continue;
        }
        let row_style = PmiLayoutStyle { plane: row.plane, ..style };
        let drawn = pmi_present(&row.geometry, row.label_world, &row.text, &row_style);
        // Paper millimetres, through the view camera — the ONE map every
        // piece of an annotation's geometry goes through.
        let project = |p: [f64; 3]| -> [f64; 2] {
            let v = frame.to_view(p);
            to_paper(placed, v[0], v[1])
        };
        let project_line = |polyline: &Vec<[f64; 3]>| -> Vec<[f64; 2]> {
            polyline.iter().map(|p| project(*p)).collect()
        };
        let mut lines: Vec<Vec<[f64; 2]>> = drawn.polylines.iter().map(project_line).collect();
        // Arrowheads are CLOSED outlines here: the sheet's output contract is
        // lines and text, so a filled head is drawn as its border.
        for arrow in &drawn.arrows {
            let mut closed: Vec<[f64; 3]> = arrow.to_vec();
            closed.push(arrow[0]);
            lines.push(project_line(&closed));
        }

        // Every run of one annotation is laid out in the SAME in-plane basis
        // (the layout's `right` / `up`), so the first run carries it — read
        // rather than re-derived, because the basis is the kernel's. A FRAMED
        // symbol — an FCF's cells, a datum's box — is then ONE unit about the
        // annotation's anchor: flattening rotates the frame and its cells
        // together, so the cells keep their order and their spacing.
        let unit = drawn
            .texts
            .first()
            .filter(|_| !drawn.frames.is_empty())
            .map(|run| Unit {
                anchor: drawn.text_anchor,
                right: run.right,
                up: run.up,
                cap: run.height,
            });
        // A point of a FLATTENED piece: its offset from the anchor measured
        // in the layout's own in-plane basis, laid straight onto the paper at
        // sheet scale — the anchor's PROJECTED position kept, everything
        // about it turned flat and back to its unforeshortened size.
        let flat_about = |anchor: [f64; 3], right3: [f64; 3], up3: [f64; 3], p: [f64; 3]| -> [f64; 2] {
            let d = sub(p, anchor);
            let a = project(anchor);
            [a[0] + dot(d, right3) * placed.scale, a[1] - dot(d, up3) * placed.scale]
        };

        let texts: Vec<SheetText> = drawn
            .texts
            .iter()
            .filter(|run| !run.text.trim().is_empty())
            .map(|run| {
                let cap = run.height * placed.scale;
                if flatten {
                    // Reading left to right at the run's sheet-scale size,
                    // about its own anchor — or about the unit's, when it is
                    // one cell of a framed symbol that turns as a whole.
                    let anchor = match unit {
                        Some(unit) => flat_about(unit.anchor, unit.right, unit.up, run.anchor),
                        None => project(run.anchor),
                    };
                    return SheetText::flat(run.text.clone(), anchor, cap);
                }
                // The paper image of one cap height along a unit direction.
                let image = |v: [f64; 3]| -> [f64; 2] {
                    [dot(v, frame.right) * cap, -dot(v, frame.up) * cap]
                };
                let mut right = image(run.right);
                // A plane seen from BEHIND mirrors its geometry — that is
                // what a projection does — but never its text: negating the
                // reading direction spans the very same parallelogram, so the
                // box stays where it lies and the glyphs still read forward.
                if dot(cross(run.right, run.up), frame.forward) > 0.0 {
                    right = [-right[0], -right[1]];
                }
                SheetText::new(run.text.clone(), project(run.anchor), right, image(run.up))
            })
            .collect();

        // The frames follow their unit; everything else — leaders, dimension
        // and extension lines, arrowheads — is projected either way.
        let frames: Vec<Vec<[f64; 2]>> = match (flatten, unit) {
            (true, Some(unit)) => drawn
                .frames
                .iter()
                .map(|polyline| {
                    polyline
                        .iter()
                        .map(|p| flat_about(unit.anchor, unit.right, unit.up, *p))
                        .collect()
                })
                .collect(),
            _ => drawn.frames.iter().map(project_line).collect(),
        };
        // A framed symbol's outline is exempt from the line breaking (it
        // would erase itself) and instead becomes a box of its own, which is
        // what re-attaches the leader to the flattened frame's nearest edge.
        let mut boxes: Vec<Box2> = texts.iter().map(Box2::of_text).collect();
        if let Some(unit) = unit {
            let scaled = unit.cap * placed.scale;
            let (right, up) = if flatten {
                ([scaled, 0.0], [0.0, -scaled])
            } else {
                let image = |v: [f64; 3]| [dot(v, frame.right) * scaled, -dot(v, frame.up) * scaled];
                (image(unit.right), image(unit.up))
            };
            if let Some(box2) = unit.box_of(&drawn.frames, project(unit.anchor), right, up) {
                boxes.push(box2);
            }
        }
        let mut lines = break_for_text(lines, &boxes);
        lines.extend(frames);

        out.push(AnnotationDrawing {
            id: row.id.clone(),
            kind: row.kind.clone(),
            text: row.text.clone(),
            lines,
            texts,
            note: if !plane_is_edge_on(row, frame.forward) {
                String::new()
            } else if flatten {
                format!(
                    "annotation plane edge-on to '{}': its geometry projects to a line, its text is flattened onto the sheet",
                    view.name
                )
            } else {
                format!(
                    "annotation plane edge-on to '{}': it and its text project to a line",
                    view.name
                )
            },
        });
    }
    out
}

/// A FRAMED symbol — an FCF's row of cells, a datum's lettered box — as one
/// piece: the annotation's anchor and the layout's own in-plane basis, with
/// the cap height its sizes are measured in. The frame and its cells flatten
/// together about `anchor`, which is what keeps the cells in order and
/// evenly spaced whatever the annotation plane does to the view.
#[derive(Debug, Clone, Copy)]
struct Unit {
    anchor: [f64; 3],
    right: [f64; 3],
    up: [f64; 3],
    cap: f64,
}

impl Unit {
    /// The box the unit's outline occupies, measured in the layout's own
    /// basis about the anchor and mapped to paper through the basis it was
    /// DRAWN with — so an oblique plane with the toggle off gives the
    /// projected parallelogram and not its bounding rectangle. No clearance:
    /// a leader is meant to TOUCH the frame.
    fn box_of(
        &self,
        frames: &[Vec<[f64; 3]>],
        anchor: [f64; 2],
        right: [f64; 2],
        up: [f64; 2],
    ) -> Option<Box2> {
        if self.cap <= 0.0 {
            return None;
        }
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for polyline in frames {
            for p in polyline {
                let d = sub(*p, self.anchor);
                let local = [dot(d, self.right) / self.cap, dot(d, self.up) / self.cap];
                for axis in 0..2 {
                    lo[axis] = lo[axis].min(local[axis]);
                    hi[axis] = hi[axis].max(local[axis]);
                }
            }
        }
        if lo[0] > hi[0] {
            return None;
        }
        let mid = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
        Some(Box2 {
            centre: [
                anchor[0] + mid[0] * right[0] + mid[1] * up[0],
                anchor[1] + mid[0] * right[1] + mid[1] * up[1],
            ],
            right,
            up,
            half: [(hi[0] - lo[0]) * 0.5, (hi[1] - lo[1]) * 0.5],
        })
    }
}



