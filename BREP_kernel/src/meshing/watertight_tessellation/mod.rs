//! Watertight, chord-tolerance-driven BREP tessellation.
//!
//! The classic per-face tessellator samples every face independently, so
//! adjacent faces disagree along their shared edge and the mesh has cracks.
//! Here every EDGE is sampled once — adaptively, until the 3D chord sag is
//! below the chord tolerance — and both adjacent faces consume the exact
//! same sample positions (endpoints come from the shared vertex records), so
//! coincidence along shared edges holds bit-for-bit by construction.  Each
//! face is then triangulated in parameter space: trim loops built from the
//! shared samples (holes bridged to the outer loop), ear clipping, then
//! conforming interior-edge splits until interior chords also meet the
//! tolerance.  Boundary edges are never split, so no T-junctions can appear.

use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord};
use crate::{KnotVector, Mesh, Vec3};
use rustc_hash::FxHashMap;
use std::collections::{HashMap, HashSet};



mod profile;
mod triangulate;
mod periodic;
mod edge_sampling;
mod face_tess;
mod cone;
mod orient;
mod sphere_atlas;
mod pole_patch;
mod rectangle;
mod stride_encode;
mod face_memo;
mod scan_mesh;

use profile::*;
use triangulate::*;
use periodic::*;
use edge_sampling::*;
use face_tess::*;
use cone::tessellate_pointed_cone_fan;
use sphere_atlas::*;
use pole_patch::*;
use rectangle::*;

/// `BREP_TESS_PINCH_CUSP` (default on, `=0` restores): the watertight mesher meets a face whose
/// boundary points touch at zero angle (a cusp) or a shell pinched to a point (
/// brep-bench tangent/midsphere-common and tangent/viviani-cut). Three parts:
/// - [`edge_sampling`]: a sphere chart plane an edge only TOUCHES at a sample (its
///   function is rounding noise there) is not crossed beside that sample, and a span
///   whose chord and midpoint are under 1e-6(1 + |p|) is not split (a hook at the
///   end of a tangent-touch edge);
/// - [`orient`]: two faces that emit the same triangle (the tip of a sliver between
///   two surfaces tangent at a pinch, spanned by the same three shared samples) are a
///   zero-thickness slab, and both copies are dropped before orientation.
pub(crate) fn pinch_cusp_enabled() -> bool {
    crate::switch!("BREP_TESS_PINCH_CUSP", true)
}

pub use face_tess::triangulate_planar_region;
pub use orient::tessellate_brep_watertight;
pub use face_memo::clear as clear_face_memo;
pub(crate) use scan_mesh::{enabled as scan_mesh_enabled, take_counts as take_scan_mesh_counts, ScanMeshScope};
pub(crate) use stride_encode::tessellate_brep_watertight_face_stride_keyed;
pub use stride_encode::{
    sample_edge_polylines, sample_edges_encoded, tessellate_brep_watertight_face_stride,
    tessellate_brep_watertight_face_stride_with_samples,
};

