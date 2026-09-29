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
mod orient;
mod sphere_atlas;
mod stride_encode;

use profile::*;
use triangulate::*;
use periodic::*;
use edge_sampling::*;
use face_tess::*;
use sphere_atlas::*;

pub use face_tess::triangulate_planar_region;
pub use orient::tessellate_brep_watertight;
pub use stride_encode::{
    sample_edge_polylines, sample_edges_encoded, tessellate_brep_watertight_face_stride,
    tessellate_brep_watertight_face_stride_with_samples,
};

