//! T-junction stitching before the faceted repair.
//!
//! A mesh whose triangle was split on ONE side of a shared edge (a vertex
//! sitting on the interior of a neighbour's edge) is not a two-manifold:
//! the long edge is used once, its two halves once each, and the repair's
//! boundary capping can only fan a degenerate three-vertex loop over them.
//! Every T-junction variant of the lane's degraded corpus (200 of 1 600
//! rows) failed exactly there, "faceted shell repair produced invalid
//! topology: edge N used 1 times (expected 2)".
//!
//! Stitching splits the long edge at every boundary vertex that lies on its
//! interior (within the weld tolerance of the segment, strictly between its
//! ends), so both sides carry the same chain and the mesh closes. Only
//! boundary edges and boundary vertices take part: a closed mesh is left
//! byte-identical, and nothing is moved — the split vertex is the one the
//! file already had.
use std::collections::{BTreeMap, HashMap};

use crate::Vec3;

/// What the stitch did, for the conversion report.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StitchReport {
    /// Boundary edges split at a vertex on their interior.
    pub stitched_edges: usize,
    /// Triangles added by those splits (one per inserted vertex).
    pub added_triangles: usize,
    /// Passes the stitch took (a split can expose a nested junction).
    pub passes: usize,
}

fn point(positions: &[f64], index: u32) -> Vec3 {
    let at = index as usize * 3;
    Vec3::new(positions[at], positions[at + 1], positions[at + 2])
}

/// Split every boundary edge at the boundary vertices on its interior.
/// `tolerance` is the distance within which a vertex counts as on the
/// segment; returns the (possibly longer) index buffer and the report.
/// Indices must already be welded (one index per coordinate).
pub fn stitch_t_junctions(positions: &[f64], indices: &[u32], tolerance: f64) -> (Vec<u32>, StitchReport) {
    let mut triangles = indices.to_vec();
    let mut report = StitchReport::default();
    if positions.len() % 3 != 0 || triangles.len() % 3 != 0 || !(tolerance >= 0.0) || !tolerance.is_finite() {
        return (triangles, report);
    }
    let vertex_count = (positions.len() / 3) as u32;
    if triangles.iter().any(|&i| i >= vertex_count) {
        return (triangles, report);
    }
    for _ in 0..8 {
        // Undirected edge use counts, and the owning (triangle, corner) of
        // each directed edge.
        let mut uses = HashMap::<(u32, u32), u32>::new();
        for triangle in triangles.chunks_exact(3) {
            for corner in 0..3 {
                let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                *uses.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let uses = &uses;
        let boundary_edges: Vec<(usize, usize)> = triangles
            .chunks_exact(3)
            .enumerate()
            .flat_map(|(t, triangle)| {
                (0..3).filter_map(move |corner| {
                    let (a, b) = (triangle[corner], triangle[(corner + 1) % 3]);
                    (uses[&(a.min(b), a.max(b))] == 1).then_some((t, corner))
                })
            })
            .collect();
        if boundary_edges.is_empty() {
            break;
        }
        let mut boundary_vertices: Vec<u32> = boundary_edges
            .iter()
            .flat_map(|&(t, corner)| [triangles[3 * t + corner], triangles[3 * t + (corner + 1) % 3]])
            .collect();
        boundary_vertices.sort_unstable();
        boundary_vertices.dedup();
        // Per boundary edge, the interior vertices sorted along it.
        let mut splits = BTreeMap::<(usize, usize), Vec<(f64, u32)>>::new();
        for &(t, corner) in &boundary_edges {
            let (a, b) = (triangles[3 * t + corner], triangles[3 * t + (corner + 1) % 3]);
            let (pa, pb) = (point(positions, a), point(positions, b));
            let ab = pb - pa;
            let length_squared = ab.length_squared();
            if length_squared <= 0.0 {
                continue;
            }
            for &v in &boundary_vertices {
                if v == a || v == b {
                    continue;
                }
                let pv = point(positions, v);
                let s = (pv - pa).dot(ab) / length_squared;
                if !(s > 0.0 && s < 1.0) {
                    continue;
                }
                let foot = pa + ab * s;
                let along = ab.length();
                // Strictly inside: further than the tolerance from both ends.
                if (pv - foot).length() <= tolerance
                    && s * along > tolerance
                    && (1.0 - s) * along > tolerance
                {
                    splits.entry((t, corner)).or_default().push((s, v));
                }
            }
        }
        if splits.is_empty() {
            break;
        }
        report.passes += 1;
        // Apply the splits: a triangle with several split edges is handled
        // one edge per pass (the triangle index changes once it is rewritten),
        // so take the first split edge of each triangle this pass.
        let mut seen = std::collections::HashSet::new();
        for ((t, corner), mut chain) in splits {
            if !seen.insert(t) {
                continue;
            }
            chain.sort_by(|x, y| x.0.total_cmp(&y.0));
            chain.dedup_by_key(|x| x.1);
            let (a, b, c) = (
                triangles[3 * t + corner],
                triangles[3 * t + (corner + 1) % 3],
                triangles[3 * t + (corner + 2) % 3],
            );
            // (a, b, c) becomes (a, v1, c), (v1, v2, c), ..., (vk, b, c).
            let mut previous = a;
            let mut first = true;
            for &(_, v) in &chain {
                if first {
                    triangles[3 * t] = previous;
                    triangles[3 * t + 1] = v;
                    triangles[3 * t + 2] = c;
                    first = false;
                } else {
                    triangles.extend_from_slice(&[previous, v, c]);
                    report.added_triangles += 1;
                }
                previous = v;
            }
            triangles.extend_from_slice(&[previous, b, c]);
            report.added_triangles += 1;
            report.stitched_edges += 1;
        }
    }
    (triangles, report)
}

