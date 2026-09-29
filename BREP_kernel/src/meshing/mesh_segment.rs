//! Mesh face segmentation — stage 2 of the Rust mesh-import pipeline (the
//! rust-first port of the app's `buildImportSolidsFromGeometry`: deflection-
//! angle face grouping + analytic-primitive recognition; stage 1 is
//! `mesh_to_faceted_brep`).
//!
//! `segment_mesh_faces` groups the triangles of an indexed mesh (or a raw
//! STL-style soup when `indices` is empty) into smooth regions by dihedral-
//! angle region growing, then recognizes each region's analytic carrier by
//! least-squares fitting in the order plane → cylinder → cone → sphere →
//! torus.  A fit is accepted only when every region vertex sits within
//! `fit_tolerance · region-scale` of the carrier AND the triangle normals
//! agree with the carrier normal within `normal_tolerance_deg` (area-
//! trimmed: the worst slivers up to 0.1% of the region area are exempt);
//! regions with no accepted carrier stay `Freeform`.
//!
//! Tangent-smooth compounds defeat pure dihedral growing (a fillet blend
//! runs tangentially into its walls, so the walls and the blend merge into
//! one smooth region).  Mirroring the app's planar-extraction semantics, a
//! refinement pass splits such regions: seed-plane-anchored coplanar groups
//! are peeled off first (gated by `planar_extraction_angle_deg` and
//! `planar_min_area_percent`) and the leftover connected components are
//! re-fitted through the same cascade.  A component the cascade still
//! rejects is then peeled into cylinders — a rounded outline of tangent
//! arcs with different radii, a fillet running out into a boss — by
//! seed-and-grow over its planar strips at the mesh's own precision, so the
//! first strip past a tangent junction is refused where the acceptance
//! tolerance would have admitted it.
//!
//! `segment_mesh_faces` is geometry-only analysis: no `BrepSolid` is built
//! there.  The output — per-triangle region ids plus per-region carrier
//! records, all serde-serializable — is the input contract for stage 3,
//! `mesh_regions_to_brep`, which rebuilds a validated `BrepSolid` whose
//! faces are the segmented regions (exact planes, full revolve walls and
//! partial cylinder patches in v1) instead of one face per triangle.

use crate::fit::solve_small;
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{
    interpolate_curve, make_line, make_plane, make_revolution,
    mesh_to_faceted_brep, solid_signed_volume, NurbsCurve, Vec3, Vec4,
};
use rustc_hash::FxHashMap as HashMap;
use serde::{Deserialize, Serialize};

/// Region id given to triangles that could not be assigned to any region
/// (degenerate triangles with no non-degenerate neighbor).
pub const UNASSIGNED_REGION: u32 = u32::MAX;

/// Options for `segment_mesh_faces`.  All tolerances are dimensionless or in
/// degrees; distances derive from the region/mesh scale so the segmentation
/// is size-invariant.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct SegmentOptions {
    /// Region-growing gate: triangles on either side of an edge join the
    /// same smooth region when their dihedral (normal-to-normal) angle is
    /// below this threshold, in degrees.
    pub deflection_angle_deg: f64,
    /// Regions with fewer triangles than this are not fitted (they stay
    /// `Freeform`).  The app used 8 for noisy scanned meshes; the default 1
    /// fits everything.
    pub min_region_triangles: usize,
    /// Carrier acceptance: maximum vertex deviation from the fitted carrier
    /// as a fraction of the region's bounding-box diagonal.
    pub fit_tolerance: f64,
    /// Carrier acceptance: maximum angle between a triangle normal and the
    /// carrier normal at the triangle centroid, in degrees.
    pub normal_tolerance_deg: f64,
    /// Refinement pass: a triangle joins a seed plane only when its normal
    /// is within this many degrees of the seed normal (and its vertices lie
    /// within the distance gate of the seed plane).
    pub planar_extraction_angle_deg: f64,
    /// Refinement pass: an extracted planar group is kept only when its
    /// area is at least this percentage of its parent region's area
    /// (mirrors the app's planar minimum-area percent).
    pub planar_min_area_percent: f64,
    /// Vertex weld tolerance; `<= 0` derives it from the bounding-box
    /// diagonal (`diagonal · 1e-6`), matching `mesh_to_faceted_brep`.
    pub weld_tolerance: f64,
}

impl Default for SegmentOptions {
    fn default() -> Self {
        Self {
            deflection_angle_deg: 30.0,
            min_region_triangles: 1,
            fit_tolerance: 1e-3,
            normal_tolerance_deg: 15.0,
            planar_extraction_angle_deg: 1.0,
            planar_min_area_percent: 1.0,
            weld_tolerance: 0.0,
        }
    }
}

/// Recognized analytic carrier of a region.  Axis directions are unit
/// vectors; `sense` is `+1` when the mesh normals point along the carrier's
/// outward normal (away from the axis/center) and `-1` for a cavity.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(tag = "type")]
pub enum RegionCarrier {
    Plane {
        /// A point on the plane (the region's vertex centroid).
        origin: Vec3,
        /// Unit normal oriented with the mesh triangle normals.
        normal: Vec3,
    },
    Cylinder {
        /// Point on the axis closest to the region's vertex centroid.
        axis_point: Vec3,
        axis_dir: Vec3,
        radius: f64,
        sense: i8,
    },
    Cone {
        apex: Vec3,
        /// Unit axis pointing from the apex into the region.
        axis_dir: Vec3,
        half_angle_rad: f64,
        sense: i8,
    },
    Sphere {
        center: Vec3,
        radius: f64,
        sense: i8,
    },
    Torus {
        /// Center of the spine circle.
        center: Vec3,
        axis_dir: Vec3,
        major_radius: f64,
        minor_radius: f64,
        sense: i8,
    },
    Freeform,
}

impl RegionCarrier {
    pub fn kind(&self) -> &'static str {
        match self {
            RegionCarrier::Plane { .. } => "plane",
            RegionCarrier::Cylinder { .. } => "cylinder",
            RegionCarrier::Cone { .. } => "cone",
            RegionCarrier::Sphere { .. } => "sphere",
            RegionCarrier::Torus { .. } => "torus",
            RegionCarrier::Freeform => "freeform",
        }
    }
}

/// One smooth region of the mesh with its recognized carrier and fit
/// residuals (residuals are zero for `Freeform` regions — no fit).
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MeshRegion {
    pub id: u32,
    pub triangle_count: usize,
    pub area: f64,
    pub bbox_min: Vec3,
    pub bbox_max: Vec3,
    pub carrier: RegionCarrier,
    /// Maximum absolute vertex deviation from the accepted carrier.
    pub max_deviation: f64,
    /// Root-mean-square vertex deviation from the accepted carrier.
    pub rms_deviation: f64,
    /// Maximum angle between a triangle normal and the carrier normal, deg.
    pub max_normal_angle_deg: f64,
}

/// Segmentation result: `triangle_region_ids[t]` is the region id of input
/// triangle `t` (index into `regions`; `UNASSIGNED_REGION` for degenerate
/// triangles with no assignable neighbor).
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MeshSegmentation {
    pub triangle_region_ids: Vec<u32>,
    pub regions: Vec<MeshRegion>,
    pub triangle_count: usize,
    pub welded_vertex_count: usize,
}

#[path = "mesh_segment/mesh_data.rs"]
mod mesh_data;
#[path = "mesh_segment/carrier_fit.rs"]
mod carrier_fit;
#[path = "mesh_segment/segmentation.rs"]
mod segmentation;
#[path = "mesh_segment/brep_builder.rs"]
mod brep_builder;
#[path = "mesh_segment/face_build.rs"]
mod face_build;

use brep_builder::*;
use carrier_fit::*;
use face_build::*;
use mesh_data::*;
use segmentation::*;

pub use brep_builder::mesh_regions_to_brep;
pub use segmentation::segment_mesh_faces;

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

