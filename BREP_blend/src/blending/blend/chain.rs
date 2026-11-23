use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{fit, NurbsCurve, NurbsSurface, Vec3, Vec4};
use super::stations::*;
pub use super::edge::*;
pub use super::fold::check_wall_fold;

// ====================================================================
// Smooth-edge chains (§6.9.5): conjugated edges blended as ONE unit
// ====================================================================

mod closed;
mod closed_surgery;
mod collect;
mod march;
mod open;

pub use closed::{blend_smooth_chain, blend_smooth_chain_if_closed};

use closed::{cross_edge_at, pcurve_portion, ChainRows, RimPiece};
use closed_surgery::chain_surgery;
pub use closed_surgery::signed_area;
// The marches' own tests drive the chain march directly (`blend/tests`).
pub use collect::collect_smooth_chain;
use collect::{ChainSegment, SmoothChain};
pub use march::{march_chain, ChainBudget, ChainSample, FoldPolicy, CHAIN_PER_SEGMENT};
pub use march::{march_chain_typed, ChainMarchError};
pub use march::{assign_chain_parameters, chain_dense_contacts, insert_chain_stations, DenseContacts};
use march::{dense_contact_scope, ladder_trend, ChainStationFailure, CHAIN_CARVE_MAX_PER_SEGMENT};
use open::blend_open_smooth_chain;

/// The edges of the smooth (G1) chain through `edge_id`, in chain order, and
/// whether it closes: the selection a chamfer that walks whole chains reads.
pub fn smooth_chain_members(solid: &BrepSolid, edge_id: u64) -> Result<(Vec<u64>, bool), crate::KernelRefusal> {
    let chain = collect_smooth_chain(solid, edge_id)?;
    Ok((chain.segments.iter().map(|segment| segment.edge.id).collect(), chain.closed))
}



/// The declared wall reading, shared with the closed-edge lane's wall verdict.
pub use closed::{
    declared_wall_probe_at_why, declared_wall_probe_located, declared_wall_probe_located_why,
    declared_wall_probe_why, PreparedWall,
};
/// The support-piece pcurve fitter, shared with the closed-edge lane.
pub use closed::{pcurve_guide, project_piece_pcurve, project_piece_pcurve_within, project_piece_pcurve_within_knot_retry};
pub use closed::CHAIN_PCURVE_FLOOR_WHAT;






