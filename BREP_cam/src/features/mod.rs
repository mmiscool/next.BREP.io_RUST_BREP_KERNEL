//! 2.5D feature recognition (lane C13 row 1).
//!
//! Recognition is kernel-free: it reads the machine-space mesh snapshot for
//! adjacency, convexity and extents, the exact analytic carriers the kernel
//! adapter reports per face ([`FaceCarrier`], `kernel_adapter::features::
//! face_carriers`), the exact planar regions of horizontal faces
//! (`kernel_adapter::planar::horizontal_faces`) and lane C8's recognised
//! holes. Every feature is reported in machine space with its machining
//! direction; a feature the setup cannot machine along `+Z` stays in the
//! result with `machinable: false` and the reason.
//!
//! Definitions (the rules the tests pin):
//! - A *floor* is a planar face. Its *walls* are the neighbours across
//!   concave edges that rise along the floor's normal; *drops* are the
//!   neighbours across convex edges.
//! - *Face to face*: the highest upward planar face(s) of a body.
//! - *Closed pocket*: an upward floor whose outer boundary is entirely walls;
//!   *open pocket*: walls and drops; *step*: an open pocket with exactly one
//!   connected wall chain; *slot*: a closed pocket whose floor outline is a
//!   stadium (two parallel lines joined by two half circles).
//! - *Boss*: an island loop of a floor (the material standing on it), or an
//!   upward planar face below the top whose outer boundary is entirely drops.
//! - *Chamfer*: a planar face inclined to the tool axis between a horizontal
//!   face above and a steeper face below, across convex edges. *Fillet*: a
//!   cylinder (any revolution with a straight generatrix parallel to its
//!   axis) in the same position; a vertical concave cylinder beside a floor
//!   is that pocket's corner radius instead.
//! - *Hole*: lane C8's recognition; a floor whose walls all belong to one
//!   recognised hole is that hole's bottom, not a pocket.

use crate::model::Point3;
use serde::{Deserialize, Serialize};

pub mod graph;
pub mod recognize;
pub mod techdb;

pub use recognize::{recognize, Feature, FeatureKind, Recognized, RecognitionInput, WallLine, ProfileWalls};
pub use techdb::TechDb;

/// The exact analytic surface of one face, in machine space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
pub enum Carrier {
    /// `normal` is the face's OUTWARD normal (material behind it).
    Plane { point: Point3, normal: Point3 },
    /// Full or partial cylinder about `axis` through `origin`.
    Cylinder { origin: Point3, axis: Point3, radius: f64 },
    Cone { origin: Point3, axis: Point3, radius0: f64, radius1: f64, height: f64 },
    Torus { origin: Point3, axis: Point3, major_radius: f64, minor_radius: f64 },
    Sphere { center: Point3, radius: f64 },
    /// Anything else, with the kernel's label.
    Other { label: String },
}

/// A face's carrier by persistent body/face name.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FaceCarrier {
    pub solid: String,
    pub face: String,
    pub carrier: Carrier,
}

