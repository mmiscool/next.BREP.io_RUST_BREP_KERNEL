//! A closed rim written as a CHAIN of edges between one face and the halves
//! of a split carrier (refusal census 2026-10-31 row 2).
//!
//! The ball that blends that base rolls over a tangent round of the plane
//! (the reach check admits it only through the sided neighbour rule,
//! `analyze::support_extent`), and only the closed-rim rollover follows a
//! rolled contact (`edge/closed.rs` → `rollover.rs`). That lane takes ONE
//! closed edge; the open network's stripes do not roll, and the exact cutter
//! rides the plane's extension past the round and fills over air (+1.096 on
//! lCount_409 against the input-only quadrature), so the selection is refused
//! by name (`tangent_rollover_only`).
//!
//! The same shape built whole (one boss face, one closed base edge) builds to
//! that quadrature (`fillet::analyze::sided_ball_neighbour_tests`). So when the
//! ordinary path refuses a selection by that name (and only then: a split rim it
//! builds, keeps its body, its faces and its per-edge blend names), the body is
//! CANONICALISED: the halves the chain's far side runs along are merged into the
//! one face they are (`merge_faces_within`, whose shared-seam ring lane exists
//! for exactly the halves of a bore written as two faces), which joins the rim's
//! pieces into one closed edge where they continue each other, and the chain is
//! selected as that edge. The result keeps the merged face (the surviving half's
//! id and name): it is the body a face built whole would give.
//!
//! The class is narrow by construction: two or more selected OPEN edges that
//! close one cycle through shared vertices, every one with the SAME face on
//! one side and pairwise distinct faces on the other, which merge into one
//! face; after the merge every selected point must resolve to one CLOSED
//! edge. Anything else (the merge declines, the pieces stay apart, the merged
//! body refuses too) keeps the ordinary path's refusal unchanged.
//!
//! Naming: the merged rim is ONE selection, so its blend face and pieces are
//! named after the FIRST selected edge (`SuiteOperation:BLEND:<first>`, then
//! `_1`, `_2` ..); a later history that names another piece's wall by its own
//! edge does not resolve. When the merged body refuses too, the refusal keeps
//! the ordinary path's text and appends the merged attempt's reason.
//!
//! Default ON since the promotion 2026-11-05, (lane F, 2026-10-31, behind a switch
//! until then): `BREP_SPLIT_RIM_MERGE=0` (or `off`) turns it off, and
//! `with_split_rim_merge_for_test` forces either on a test's thread.

use crate::topology::BrepSolid;
use crate::{KernelRefusal, Vec3};

/// Whether the split-rim canonicalisation runs: ON unless
/// `BREP_SPLIT_RIM_MERGE=0` (or `off`), or a test's forced value.
pub(crate) fn split_rim_merge_on() -> bool {
    switch!("BREP_SPLIT_RIM_MERGE", true)
}


/// The canonicalised body and the selection on it: one point (the first
/// selected) and its name.
pub(crate) struct SplitRimMerge {
    pub(crate) solid: BrepSolid,
    pub(crate) points: Vec<Vec3>,
    pub(crate) names: Option<Vec<String>>,
}

/// The faces whose loops use `edge_id`, in shell order (a manifold edge has two).
fn faces_of_edge(solid: &BrepSolid, edge_id: u64) -> Vec<u64> {
    let mut faces = Vec::with_capacity(2);
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let uses = face.loops.iter().flat_map(|loop_record| &loop_record.coedges).filter(|coedge| coedge.edge_id == edge_id).count();
        for _ in 0..uses {
            faces.push(face.id);
        }
    }
    faces
}

/// Do these open edges close ONE cycle: every end vertex used by exactly two
/// of them, and all of them connected through those vertices?
fn closes_one_cycle(solid: &BrepSolid, edge_ids: &[u64]) -> bool {
    let mut ends: Vec<(u64, u64)> = Vec::with_capacity(edge_ids.len());
    for id in edge_ids {
        let Some(edge) = solid.edges.iter().find(|edge| edge.id == *id) else { return false };
        if edge.degenerate || edge.start_vertex_id == edge.end_vertex_id {
            return false;
        }
        ends.push((edge.start_vertex_id, edge.end_vertex_id));
    }
    let mut uses: rustc_hash::FxHashMap<u64, usize> = Default::default();
    for (a, b) in &ends {
        *uses.entry(*a).or_default() += 1;
        *uses.entry(*b).or_default() += 1;
    }
    if uses.values().any(|count| *count != 2) {
        return false;
    }
    // Walk the cycle from the first edge; it must visit every edge.
    let mut visited = vec![false; ends.len()];
    visited[0] = true;
    let mut at = ends[0].1;
    for _ in 1..ends.len() {
        let Some(next) = (0..ends.len()).find(|index| !visited[*index] && (ends[*index].0 == at || ends[*index].1 == at)) else {
            return false;
        };
        visited[next] = true;
        at = if ends[next].0 == at { ends[next].1 } else { ends[next].0 };
    }
    at == ends[0].0 && visited.iter().all(|seen| *seen)
}

/// The body with the split carrier under a selected rim chain merged into one
/// face and the chain into one closed edge, or `None` when the selection is
/// not that class or the merge does not give one closed edge.
pub(crate) fn merge_split_rims(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    resolve: &dyn Fn(&BrepSolid, Vec3) -> Result<u64, KernelRefusal>,
) -> Option<SplitRimMerge> {
    if edge_points.len() < 2 {
        return None;
    }
    let mut selected: Vec<u64> = Vec::with_capacity(edge_points.len());
    for point in edge_points {
        let edge_id = resolve(solid, *point).ok()?;
        if !selected.contains(&edge_id) {
            selected.push(edge_id);
        }
    }
    if selected.len() < 2 || !closes_one_cycle(solid, &selected) {
        return None;
    }
    // One face common to every edge of the chain; the far faces pairwise distinct.
    let pairs: Vec<Vec<u64>> = selected.iter().map(|id| faces_of_edge(solid, *id)).collect();
    if pairs.iter().any(|faces| faces.len() != 2 || faces[0] == faces[1]) {
        return None;
    }
    let common = [pairs[0][0], pairs[0][1]].into_iter().find(|face| pairs.iter().all(|faces| faces.contains(face)))?;
    let far: Vec<u64> = pairs.iter().map(|faces| if faces[0] == common { faces[1] } else { faces[0] }).collect();
    if far.iter().enumerate().any(|(index, face)| far[..index].contains(face)) {
        return None;
    }
    let tolerance = crate::KernelTolerances::for_solid(solid, crate::BooleanOptions::default().tolerance).model;
    let (merged, carried_by) = crate::merge_faces_within(solid, tolerance, &far).ok()?;
    let survivor = carried_by.first()?.1;
    if carried_by.iter().any(|(_, carrier)| *carrier != survivor) {
        return None;
    }
    // Every selected point now on ONE closed edge.
    let mut joined: Option<u64> = None;
    for point in edge_points {
        let edge_id = resolve(&merged, *point).ok()?;
        if joined.is_some_and(|id| id != edge_id) {
            return None;
        }
        joined = Some(edge_id);
    }
    let edge = merged.edges.iter().find(|edge| Some(edge.id) == joined)?;
    if edge.degenerate || edge.start_vertex_id != edge.end_vertex_id {
        return None;
    }
    if !merged.validate().is_empty() {
        return None;
    }
    crate::blend::record_count("count.split_rim.merged", 1);
    Some(SplitRimMerge {
        solid: merged,
        points: vec![edge_points[0]],
        names: edge_names.and_then(|names| names.first()).map(|name| vec![name.clone()]),
    })
}

