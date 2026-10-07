use super::*;

mod builder;
mod clearance;
mod edge_conform;
mod edges;
mod finalize;
mod joints;
mod image;
mod inherited;
mod pipeline;
mod polish;
mod refusal_welds;
mod vertices;

pub(super) use builder::{Assembler, SourceEdge};
pub(crate) use edge_conform::edge_interior_lies_on;
pub(crate) use finalize::{
    apply_assembly_heal_chain, commit_nearby_edge_endpoints, finalize_assembled_solid,
};
pub(super) use finalize::repair_open_assembly_via_heal_chain;
pub(crate) use pipeline::{assemble_fragments, assemble_open_fragments};
pub(super) use edges::{reconstruct_edges_from_carriers, restore_edges};
pub(super) use polish::polish_triple_junction_vertices;
pub(super) use vertices::{restore_vertices, settle_vertices_onto_carriers};
pub(super) use refusal_welds::conform_unmatched_one_use_edges;

pub(super) use joints::{close_trim_loop_joints, restore_trims};
