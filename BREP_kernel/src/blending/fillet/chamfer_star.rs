//! The chamfer MIXED STAR (refusal census 2026-10-26 row 1, lane R
//! 2026-10-28): one selected CONCAVE edge Q between two walls and two selected
//! CONVEX edges A and B, each on one of those walls and on the cap they share,
//! meeting at one vertex. The stripe network has a fillet closure for this
//! class at planar orthogonal corners (`VertexKind::MixedStar`, the ball
//! riding Q's rim as a torus sector) and none for chamfers; it refuses ("the
//! stripes at vertex N disagree on which side of face M the ball sits").
//!
//! The chamfer closure is the fillet's read with straight sections: the
//! convex chamfers continue round the top of Q's chamfer strip.  Built as a
//! composition of two constructions the network already has:
//!
//! 1. every selected concave edge at such a star is chamfered first (Q's
//!    strip fills the air wedge between the walls; its top edge S lies in
//!    the cap);
//! 2. on that body the remaining selected edges are chamfered together with
//!    each S, a convex cap edge whose corners with A and B are two-edge
//!    corners whose third edge (the strip's edge with the wall) stays sharp,
//!    which the network closes with its own corner construction.
//!
//! No tolerance of its own: each phase is an ordinary network build. Where the
//! strip meets each wall the bevel chain turns a REFLEX corner, which the
//! network closes with the chamfer section turned about the corner's vertical
//! edge (a cone sector). On lane L's L prism the result is the closed form 750
//! + 5d² − (10 − 2d + d√2)·d²/2 − 2·(π/4)·d³/6 to 1e-9.
//!
//! Default ON since lane R's promotion (2026-10-29, owner decision: this
//! corner); `BREP_CHAMFER_MIXED_STAR=0` restores the refusal. A phase that
//! refuses hands the selection back to the ordinary path unchanged.

use crate::topology::BrepSolid;
use crate::{KernelRefusal, Vec3};

/// Whether the two-phase chamfer mixed star runs: ON unless
/// `BREP_CHAMFER_MIXED_STAR=0` (or `off`), or a test's forced value.
pub(super) fn chamfer_mixed_star_on() -> bool {
    switch!("BREP_CHAMFER_MIXED_STAR", true)
}


/// A star found on the selection: the concave edge's index in the selection
/// and the cap face's id.
struct Star {
    concave: usize,
    cap: u64,
}

fn faces_of(solid: &BrepSolid, edge_id: u64) -> Vec<u64> {
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter(|face| face.loops.iter().flat_map(|l| &l.coedges).any(|coedge| coedge.edge_id == edge_id))
        .map(|face| face.id)
        .collect()
}

/// Concavity of a selected edge by its dihedral scan: `Some(true)` convex
/// along its whole length, `Some(false)` concave, `None` mixed or unreadable.
fn convexity(solid: &BrepSolid, edge_id: u64) -> Option<bool> {
    let profile = super::analyze::scan_dihedral(solid, edge_id).ok()?;
    match (profile.any_convex, profile.any_concave) {
        (true, false) => Some(true),
        (false, true) => Some(false),
        _ => None,
    }
}

/// Every vertex where exactly three selected edges meet — one concave, two
/// convex — the convex two sharing a cap that the concave one does not
/// border, and each convex sharing one wall with the concave one.
fn stars(solid: &BrepSolid, ids: &[u64]) -> Vec<Star> {
    let mut found: Vec<Star> = Vec::new();
    for vertex in &solid.vertices {
        let at: Vec<usize> = (0..ids.len())
            .filter(|&index| {
                solid.edges.iter().find(|edge| edge.id == ids[index]).is_some_and(|edge| {
                    edge.start_vertex_id != edge.end_vertex_id
                        && (edge.start_vertex_id == vertex.id || edge.end_vertex_id == vertex.id)
                })
            })
            .collect();
        if at.len() != 3 {
            continue;
        }
        let senses: Option<Vec<bool>> = at.iter().map(|&index| convexity(solid, ids[index])).collect();
        let Some(senses) = senses else { continue };
        let concave: Vec<usize> = at.iter().zip(&senses).filter(|(_, convex)| !**convex).map(|(index, _)| *index).collect();
        let convex: Vec<usize> = at.iter().zip(&senses).filter(|(_, convex)| **convex).map(|(index, _)| *index).collect();
        let ([q], [a, b]) = (concave.as_slice(), convex.as_slice()) else { continue };
        let (fq, fa, fb) = (faces_of(solid, ids[*q]), faces_of(solid, ids[*a]), faces_of(solid, ids[*b]));
        if fq.len() != 2 || fa.len() != 2 || fb.len() != 2 {
            continue;
        }
        let cap: Vec<u64> = fa.iter().copied().filter(|face| fb.contains(face) && !fq.contains(face)).collect();
        let wall_a = fa.iter().filter(|face| fq.contains(face)).count();
        let wall_b = fb.iter().filter(|face| fq.contains(face)).count();
        if let ([cap], 1, 1) = (cap.as_slice(), wall_a, wall_b) {
            if !found.iter().any(|star| star.concave == *q && star.cap == *cap) {
                found.push(Star { concave: *q, cap: *cap });
            }
        }
    }
    found
}

/// The two-phase chamfer when the selection holds a chamfer mixed star;
/// `None` when it holds none, or when either phase refuses (the ordinary path
/// then answers the selection as before).
pub(super) fn chamfer_mixed_stars(
    solid: &BrepSolid,
    edge_points: &[Vec3],
    edge_names: Option<&[String]>,
    build: &dyn Fn(&BrepSolid, &[Vec3], Option<&[String]>) -> Result<BrepSolid, KernelRefusal>,
) -> Option<BrepSolid> {
    let ids: Vec<u64> = edge_points
        .iter()
        .map(|point| super::edges::resolve_edge_by_point(solid, *point).ok())
        .collect::<Option<Vec<_>>>()?;
    let found = stars(solid, &ids);
    if found.is_empty() {
        return None;
    }
    let debug = std::env::var("BREP_DEBUG_NETWORK").is_ok();
    let mut first_indices: Vec<usize> = found.iter().map(|star| star.concave).collect();
    first_indices.sort_unstable();
    first_indices.dedup();
    let pick = |indices: &[usize]| -> (Vec<Vec3>, Option<Vec<String>>) {
        (
            indices.iter().map(|&index| edge_points[index]).collect(),
            edge_names.map(|names| indices.iter().map(|&index| names.get(index).cloned().unwrap_or_default()).collect()),
        )
    };
    let (first_points, first_names) = pick(&first_indices);
    let first = match build(solid, &first_points, first_names.as_deref()) {
        Ok(body) => body,
        Err(refusal) => {
            if debug {
                eprintln!("chamfer mixed star: phase 1 refused: {}", refusal.message);
            }
            return None;
        }
    };
    // Phase 2: the other selected edges, by their midpoints on the input (a
    // phase-1 strip does not reach an edge it does not border), and each S —
    // an edge of a phase-1 face that the star's cap also borders.
    let original: std::collections::HashSet<u64> = solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).collect();
    let new_faces: std::collections::HashSet<u64> =
        first.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).filter(|id| !original.contains(id)).collect();
    let caps: Vec<u64> = found.iter().map(|star| star.cap).collect();
    let rest: Vec<usize> = (0..edge_points.len()).filter(|index| !first_indices.contains(index)).collect();
    let (mut second_points, mut second_names) = pick(&rest);
    let mut tops = 0usize;
    for edge in &first.edges {
        let faces = faces_of(&first, edge.id);
        if faces.len() != 2 {
            continue;
        }
        let strip = faces.iter().any(|face| new_faces.contains(face));
        let cap = faces.iter().any(|face| caps.contains(face));
        if strip && cap {
            let Ok(midpoint) = edge.curve.evaluate(0.5 * (edge.t0 + edge.t1)) else { return None };
            second_points.push(midpoint);
            if let Some(names) = second_names.as_mut() {
                let owner = found.iter().find(|star| faces.contains(&star.cap)).map(|star| star.concave).unwrap_or(first_indices[0]);
                names.push(edge_names.and_then(|names| names.get(owner).cloned()).unwrap_or_default());
            }
            tops += 1;
        }
    }
    if tops == 0 {
        return None;
    }
    if debug {
        eprintln!(
            "chamfer mixed star: {} star(s); phase 1 chamfers {} concave edge(s), phase 2 {} edge(s) incl. {tops} strip top(s)",
            found.len(),
            first_points.len(),
            second_points.len()
        );
    }
    match build(&first, &second_points, second_names.as_deref()) {
        Ok(body) => Some(body),
        Err(refusal) => {
            if debug {
                eprintln!("chamfer mixed star: phase 2 refused: {}", refusal.message);
            }
            None
        }
    }
}
