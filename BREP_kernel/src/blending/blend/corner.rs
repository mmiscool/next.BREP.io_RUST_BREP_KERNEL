//! Corner closures for the rolling-ball blend.
//!
//! Mixed corners use [`crate::offset_analytic_pair`] to intersect offset
//! planes and cylinders as complete loci (lines or quadratic roots). These
//! solves accept carrier data rather than NURBS surfaces. Root selection and
//! geometry-specific refusals remain in each corner implementation.

use crate::offset_analytic_pair::{
    axial_plane_meets_offset_cylinder, diag as offset_pair_diag, line_meets_offset_cylinder,
    offset_cylinder_radius, offset_plane_pair, offset_plane_triple, OffsetPairDegeneracy,
};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{fit, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};
use super::stations::*;
use super::edge::*;

mod ball;
mod convex;
mod general_star;
mod keep_open;
mod mixed_concave;
mod mixed_curved;
mod mixed_support;
mod patch;

pub use convex::round_convex_corner;
#[allow(unused_imports)] // consumed by the network orchestrator and its cfg(test) siblings
pub(in crate::blend) use ball::{corner_faces, solve_corner_ball, CornerBall, CornerContact, CornerFace};
pub(super) use keep_open::build_keep_open_surgery;
pub(crate) use mixed_concave::round_concave_chain_corner;
#[allow(unused_imports)] // blend-surface re-export; consumed only by cfg(test) siblings today
pub(super) use mixed_curved::{detect_mixed_concave_curved_corner, MixedCurvedCorner};

use general_star::round_general_star;
use mixed_concave::round_mixed_concave_corner;
use mixed_curved::round_mixed_concave_curved_corner;
use mixed_support::{
    build_mixed_sector_patch, coedge_sense_on, curved_wall_ball_candidates, cylinder_wall_carrier,
    line_closest_points, locate_mixed_junction_arc, rebuild_mixed_corner_run,
};
pub(in crate::blend) use patch::{build_chamfer_corner_facet, build_general_corner_patch};
use patch::build_octant_face;

/// Partition a cyclic loop from a retained anchor, preserving traversal order.
/// Selected coedges are returned as indices; retained coedges are cloned.
fn partition_corner_loop(
    coedges: &[CoedgeRecord],
    selected: &[bool],
    anchor: usize,
) -> (Vec<CoedgeRecord>, Vec<usize>, Vec<CoedgeRecord>) {
    let mut prefix = Vec::new();
    let mut run = Vec::new();
    let mut suffix = Vec::new();
    for offset in 0..coedges.len() {
        let index = (anchor + offset) % coedges.len();
        if selected[index] {
            run.push(index);
        } else if run.is_empty() {
            prefix.push(coedges[index].clone());
        } else {
            suffix.push(coedges[index].clone());
        }
    }
    (prefix, run, suffix)
}
