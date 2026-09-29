use super::*;

mod builder;
mod edge_conform;
mod finalize;
mod pipeline;
mod polish;
mod refusal_welds;

pub(super) use builder::{Assembler, SourceEdge};
pub(crate) use edge_conform::edge_interior_lies_on;
pub(crate) use finalize::{
    apply_assembly_heal_chain, commit_nearby_edge_endpoints, finalize_assembled_solid,
};
pub(super) use finalize::repair_open_assembly_via_heal_chain;
pub(crate) use pipeline::{assemble_fragments, assemble_open_fragments};
pub(super) use polish::polish_triple_junction_vertices;
pub(super) use refusal_welds::conform_unmatched_one_use_edges;
