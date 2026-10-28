//! Hole recognition, grouping, and operation suggestions in machine coordinates.
use crate::Point3;
use serde::{Serialize, Deserialize};

/// Analytic carrier from the kernel adapter; this is not yet a recognized hole.
/// Trimmed mesh topology supplies extent, inward sense, and cap evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevolvedFace {
    pub solid: String,
    pub face: String,
    pub origin: Point3,
    pub axis: Point3,
    pub radius0: f64,
    pub radius1: f64,
    pub height: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum HoleKind { Plain, Counterbore, Countersink, SpotFace }

/// One coaxial hole, with complete feature extent. `top` is the entry point;
/// depth is measured along `axis` from bottom to top. Nonvertical holes remain
/// in the report, with machinable=false, rather than silently disappearing.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Hole {
    pub solid: String,
    pub faces: Vec<String>,
    pub top: Point3,
    pub axis: Point3,
    pub depth: f64,
    pub diameter: f64,
    pub through: bool,
    pub kind: HoleKind,
    pub recess: Option<HoleRecess>,
    pub machinable: bool,
    /// Diameter-table inference only. An unthreaded pilot hole can match too.
    pub thread_candidate: Option<ThreadCandidate>,
}

/// Top recess dimensions measured on the trimmed face, not its untrimmed carrier.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct HoleRecess { pub diameter: f64, pub depth: f64, pub included_angle_deg: Option<f64> }

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThreadCandidate {
    pub designation: String,
    pub pitch: f64,
    pub nominal_diameter: f64,
    pub from_pmi: bool,
}

mod recognize;
pub use recognize::{recognize, Recognition};


mod mapping;
pub use mapping::{group, annotate_threads, mark_spot_faces, suggest, HoleGroup, ThreadSize, Suggestion};
