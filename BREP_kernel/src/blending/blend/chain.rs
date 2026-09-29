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

use closed::{cross_edge_at, pcurve_portion, project_piece_pcurve, ChainRows, RimPiece};
use closed_surgery::chain_surgery;
// The marches' own tests drive the chain march directly (`blend/tests`).
pub(in crate::blend) use collect::collect_smooth_chain;
use collect::{ChainSegment, SmoothChain};
pub(in crate::blend) use march::{march_chain, ChainSample, FoldPolicy, CHAIN_PER_SEGMENT};
use march::CHAIN_CARVE_MAX_PER_SEGMENT;
use open::blend_open_smooth_chain;
