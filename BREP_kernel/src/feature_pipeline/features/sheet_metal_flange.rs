//! Sheet-metal flange and the shared core used by hems. Each selected rim
//! receives a bend and child flat; evaluation replaces the body under its name.
//! Picks may name side faces, hole segments, or their rim edges, all on one body.
//!
//! The straight leg is `flangeLength - refSetback`: inner-virtual-sharp uses
//! `R*tan(|angle|/2)`, outer-virtual-sharp (default) uses `(R+t)*tan(|angle|/2)`,
//! and tangent-to-bend uses zero. Tangent-to-bend requires at least 90°.
//! Hems force tangent-to-bend and 180°. Zero bend radius inherits the tree
//! default for flanges, or uses `1e-4` for hems because zero cannot revolve.
//! `useOppositeCenterline` mirrors the fold through the sheet plane.
//!
//! The fold line shifts inward by `insetShift - offset`, where material-inside
//! uses `t+R`, material-outside uses `R`, and bend-outside uses zero. Circular
//! hole radii grow by this shift. Edge setbacks restrict the bend to
//! `[start, length-end]`; circular-hole setbacks are unsupported.
//! Straight hole segments form flaps; full circles form revolved collars.
//!
//! Relief cuts are holes in the parent flat: rectangular, obround, or a
//! triangular tear notch. Relief defaults to none; default width is thickness
//! and default depth past the fold line is the neutral-fiber bend allowance.
//!
//! Adjacent picks in one feature trim at intersecting inset lines. Open corners
//! remove the overlapping corner square. Closed corners extend the earlier
//! wall by `R+t` and the other by `R`; miter corners cut both at 45°.
//! Closed/miter treatments require perpendicular corners and 90° folds.
//! Acute/reflex corners, over-folds, and reshaping an edge already flanged by
//! another feature are rejected. Finite-width rip slots use SM.CUTOUT.

use crate::feature_pipeline::sheet_metal::{self, Bend, Edge, Flat, Hole, HoleBend, HoleBendKind};
use crate::feature_pipeline::{FeatureContext, FeatureResult};
use crate::{make_arc, make_line, NurbsCurve, Vec3};

/// Geometric epsilon for outline/setback surgery decisions (model units).
const GEOM_EPS: f64 = 1e-9;
/// Smallest bend span the setbacks may leave.
const MIN_SPAN: f64 = 1e-6;
/// A 0 inside radius cannot revolve; it is interpreted as this knife radius.
const KNIFE_RADIUS: f64 = 1e-4;

/// Preset knobs so SM.HEM can reuse this core.
pub struct FlangeOptions {
    /// Fixed fold angle in degrees (SM.HEM = 180); `None` reads `angle` (default 90).
    pub angle_deg: Option<f64>,
    /// Default leg length when `flangeLength` is absent.
    pub default_leg: f64,
    /// Fixed `flangeLengthReference` (SM.HEM forces `Tangent to Bend` — at 180°
    /// the inner/outer mold-line setbacks diverge); `None` reads the param.
    pub length_reference: Option<&'static str>,
    /// `bendRadius <= 0` fallback: the knife radius (SM.HEM) instead of the
    /// tree default (SM.F).
    pub knife_zero_radius: bool,
}

impl Default for FlangeOptions {
    fn default() -> Self {
        FlangeOptions {
            angle_deg: None,
            default_leg: 10.0,
            length_reference: None,
            knife_zero_radius: false,
        }
    }
}

/// The per-edge geometric parameters shared by every pick of one feature.
struct FlangeParams {
    signed_angle: f64,
    inside_radius: f64,
    k_factor: f64,
    thickness: f64,
    /// Straight leg length (already length-reference adjusted).
    leg: f64,
    /// Net INWARD fold-line shift `d = insetShift − offset`.
    shift: f64,
    setback_start: f64,
    setback_end: f64,
    /// Bend-relief slot cut at setback band ends (`none` = retired-engine behavior).
    relief: ReliefType,
    /// Relief slot width along the edge (defaulted to the thickness).
    relief_width: f64,
    /// Relief reach PAST the fold line into the web (defaulted to the bend
    /// allowance).
    relief_depth: f64,
    /// Treatment where two picks of this feature meet at a flat corner.
    corner: CornerType,
}

/// Bend-relief slot shape at a flange band end (see [`add_relief_slot`]).
#[derive(Clone, Copy, PartialEq)]
enum ReliefType {
    None,
    Rectangular,
    Obround,
    Tear,
}

/// Corner treatment where two flanges of ONE feature meet at a convex corner.
#[derive(Clone, Copy, PartialEq)]
enum CornerType {
    Open,
    Closed,
    Miter,
}

/// How a corner treatment shapes one hinge-parallel end of a child wall.
#[derive(Clone, Copy)]
enum WallEnd {
    /// Plain square end at the band boundary.
    Flat,
    /// Stretch the wall past the band boundary by this much (closed corners).
    Extend(f64),
    /// 45° cut from the band boundary at the attach seam toward the free rim.
    Miter,
}

/// One resolved pick.
#[derive(PartialEq)]
enum Target {
    Outline {
        flat_id: String,
        edge_id: String,
    },
    Hole {
        flat_id: String,
        hole: usize,
        segment: usize,
    },
}

mod build;
mod hole;
mod outline;

pub use build::{execute, run};
use build::corner_child_flat;
use hole::graft_hole_flange;
use outline::{add_relief_slot, graft_outline_group, OutlinePick, ReliefSide};

/// Context-bar applicability ([`crate::feature_pipeline::context_offer`]):
/// selected faces/edges drive `faces`, but a flange folds a wall off an EXISTING
/// sheet-metal body — so the selection must sit on one (`all_sheet_metal`); a
/// plain face/edge does not offer it.
pub fn context_applicable(probe: &crate::feature_pipeline::SelectionProbe) -> bool {
    probe.all_sheet_metal && (probe.faces > 0 || probe.edges > 0)
}

pub fn schema() -> serde_json::Value {
    serde_json::json!({
    "type": "SM.F",
    "shortName": "SM.F",
    "longName": "SM Flange",
    "displayBuilder": false,
    "inputParamsSchema": {
        "id": {
            "type": "string",
            "default_value": null,
            "hint": "Unique identifier for the flange feature"
        },
        "faces": {
            "type": "reference_selection",
            "selectionFilter": [
                "FACE",
                "EDGE"
            ],
            "multiple": true,
            "default_value": null,
            "hint": "Select one or more sheet-metal edges (side faces) or hole bore segments where flanges will be constructed. All picks must be on one sheet-metal body."
        },
        "useOppositeCenterline": {
            "label": "Reverse direction",
            "type": "boolean",
            "default_value": false,
            "hint": "Flip the fold direction (up/down) for the selected hinge."
        },
        "flangeLength": {
            "type": "number",
            "default_value": 10,
            "min": 0,
            "hint": "Flange length, measured per the length reference (Outer Virtual Sharp by default)."
        },
        "edgeStartSetback": {
            "label": "Start setback",
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Distance from the selected edge start point where the flange span begins (the rest stays straight rim)."
        },
        "edgeEndSetback": {
            "label": "End setback",
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Distance from the selected edge end point where the flange span ends (the rest stays straight rim)."
        },
        "flangeLengthReference": {
            "label": "Length reference",
            "type": "options",
            "options": [
                "Inner Virtual Sharp",
                "Outer Virtual Sharp",
                "Tangent to Bend"
            ],
            "default_value": "Outer Virtual Sharp",
            "hint": "Datum flangeLength is measured from: Inner Virtual Sharp = inner mold-line corner (leg = length − R·tan(|angle|/2)); Outer Virtual Sharp = outer mold-line corner (leg = length − (R+thickness)·tan(|angle|/2)); Tangent to Bend = the bend-tangent line, so leg = length (bends ≥ 90° only)."
        },
        "angle": {
            "type": "number",
            "default_value": 90,
            "min": 0,
            "max": 180,
            "hint": "Flange angle relative to the parent sheet (0° = flat, 90° = perpendicular)."
        },
        "inset": {
            "label": "Flange position",
            "type": "options",
            "options": [
                "Material Inside",
                "Material Outside",
                "Bend Outside"
            ],
            "default_value": "Material Inside",
            "hint": "Where the flange sits relative to the reference edge (inward fold-line shift before the bend): Material Inside = thickness + bendRadius (flange stays within the original footprint at 90°); Material Outside = bendRadius; Bend Outside = 0 (the whole bend is added outside the edge)."
        },
        "bendRadius": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Inside bend radius override. 0 inherits the model’s stored value (a stored 0 is treated as 0.0001)."
        },
        "offset": {
            "type": "number",
            "default_value": 0,
            "hint": "Additional signed offset for bend-edge repositioning (positive = outward, negative = inward)."
        },
        "reliefType": {
            "type": "options",
            "options": [
                "none",
                "rectangular",
                "obround",
                "tear"
            ],
            "default_value": "none",
            "hint": "Bend relief cut at each band end that abuts remaining rim (setback ends), baked into the parent flat so it shows in the flat pattern. none = no cut (the default); rectangular = square slot; obround = stadium slot (rounded deep end, exact arcs); tear = triangular notch flush with the band end (a zero-width tear is not manifold)."
        },
        "reliefWidth": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "Relief slot width along the edge, cut into the rim flush with the band end. 0 = default: the sheet thickness."
        },
        "reliefDepth": {
            "type": "number",
            "default_value": 0,
            "min": 0,
            "hint": "How far the relief reaches past the fold line into the web. 0 = default: the neutral-fiber bend allowance |angle|·(R + k·t)."
        },
        "cornerType": {
            "type": "options",
            "options": [
                "open",
                "closed",
                "miter"
            ],
            "default_value": "open",
            "hint": "Where two flanges of THIS feature meet at a convex flat corner: open = each band trimmed to the inset fold-line intersection, corner square removed (natural gap; the bends can never overlap); closed = the earlier-selected wall extends across the corner and the other butts against it; miter = both walls cut at 45°. closed/miter need a perpendicular corner and a 90° fold."
        }
    }
})
}
