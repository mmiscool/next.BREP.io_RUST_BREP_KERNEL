use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, VertexRecord};
use crate::{fit, NurbsCurve, NurbsSurface, Vec3, Vec4};
use super::stations::*;
use super::edge::*;
use super::fold::check_wall_fold;

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
pub(in crate::blend) use closed_surgery::signed_area;
// The marches' own tests drive the chain march directly (`blend/tests`).
pub(in crate::blend) use collect::collect_smooth_chain;
use collect::{ChainSegment, SmoothChain};
pub(in crate::blend) use march::{march_chain, ChainSample, FoldPolicy, CHAIN_PER_SEGMENT};
pub(in crate::blend) use march::{march_chain_typed, ChainMarchError};
pub(in crate::blend) use march::{assign_chain_parameters, chain_dense_contacts, insert_chain_stations, DenseContacts};
use march::{ChainStationFailure, CHAIN_CARVE_MAX_PER_SEGMENT};
use open::blend_open_smooth_chain;


/// The declared wall reading, shared with the closed-edge lane's wall verdict.
pub(in crate::blend) use closed::{
    centre_path_node, declared_wall_probe, declared_wall_probe_at_why, declared_wall_probe_located, declared_wall_probe_located_why,
    declared_wall_probe_why,
};
/// The support-piece pcurve fitter, shared with the closed-edge lane.
pub(in crate::blend) use closed::{pcurve_guide, project_piece_pcurve, project_piece_pcurve_within};






