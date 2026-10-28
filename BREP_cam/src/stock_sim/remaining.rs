//! Remaining-stock display: the in-process stock as a mesh coloured by how
//! much material is left over the part.
//!
//! The mesh is the exact dexel surface of [`stock_mesh`](super::mesh::stock_mesh)
//! (watertight, outward-wound, the grid's own resolution), with its wall
//! quads cut into strips no taller than a cell: a wall from the stock floor
//! to its top would otherwise carry one colour ramp over its whole height,
//! where the part's side face and the excess above it need a band edge
//! between them. The strips cover the same quads, so volume and tiling are
//! unchanged. Every vertex
//! carries the SIGNED distance from that point of the stock surface to the
//! part (machine coordinates): positive where the stock surface is outside the
//! part (remaining material, its thickness measured along the normal, so a
//! slope reads its true stock-to-leave), negative where it is inside the part
//! (a gouge), and `+∞` where the nearest part triangle is farther than
//! `max_distance` (raw stock far from the part). The sign comes from the
//! part's vertical-ray parity, taken as the majority of three rays offset
//! slightly in XY: a stock vertex often lies exactly on a part face plane
//! (stock flush with the part, a wall across a hole's opening), where one ray
//! grazes the part's edges and miscounts. The part must be the closed target bodies'
//! snapshot (protected bodies excluded), as for [`crate::verify::deviation_map`].
//! Values within `tolerance` classify as [`DeviationClass::Within`]; the
//! colour ramp itself is the viewer's. Stock below the part (no higher than
//! its lowest point and not touching it) is flagged per vertex and counted
//! apart: it is never excess on the nearest wall.
//!
//! A REGION variant extracts and evaluates only the columns a step touched,
//! so a player rebuilds the part of the mesh that changed; region meshes of a
//! partition tile the full mesh (see [`stock_mesh_region`]).

use super::mesh::{stock_mesh_region, StockMesh};
use super::DexelGrid;
use crate::index::TriangleIndex;
use crate::mesh::MeshSnapshot;
use crate::verify::{inside_part, nearest_triangle, DeviationClass, DeviationOptions};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// The stock surface with the remaining material at every vertex.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RemainingStockMesh {
    /// The stock surface, walls in strips at most a cell tall.
    pub mesh: StockMesh,
    /// Per vertex of `mesh.positions`: signed distance to the part (+ remaining
    /// material, − gouge, `+∞` beyond `max_distance`).
    pub excess: Vec<f64>,
    /// Smallest and largest finite value (0 when there is none).
    pub min: f64,
    pub max: f64,
    pub tolerance: f64,
    /// Per vertex: stock below the part (no higher than the part's lowest
    /// point and not touching it), e.g. the block under a flange exit. Such
    /// vertices are counted apart, never as excess on the nearest wall.
    #[serde(default)]
    pub below_part: Vec<bool>,
}

impl RemainingStockMesh {
    /// Class of vertex `i` against the tolerance (`+∞` is excess).
    pub fn class(&self, i: usize) -> DeviationClass {
        let v = self.excess[i];
        if v.abs() <= self.tolerance {
            DeviationClass::Within
        } else if v > 0.0 {
            DeviationClass::Excess
        } else {
            DeviationClass::Gouge
        }
    }

    /// Whether vertex `i` is stock below the part.
    pub fn is_below_part(&self, i: usize) -> bool {
        self.below_part.get(i).copied().unwrap_or(false)
    }

    /// Vertices below the part.
    pub fn below_part_count(&self) -> usize {
        self.below_part.iter().filter(|b| **b).count()
    }

    /// Vertices in each class, below-part vertices left out: `(within, excess, gouge)`.
    pub fn counts(&self) -> (usize, usize, usize) {
        let mut out = (0, 0, 0);
        for i in 0..self.excess.len() {
            if self.is_below_part(i) {
                continue;
            }
            match self.class(i) {
                DeviationClass::Within => out.0 += 1,
                DeviationClass::Excess => out.1 += 1,
                DeviationClass::Gouge => out.2 += 1,
            }
        }
        out
    }
}

/// The whole stock with its remaining material.
pub fn remaining_stock_mesh(grid: &DexelGrid, part: &MeshSnapshot, index: &dyn TriangleIndex, options: &DeviationOptions) -> RemainingStockMesh {
    if grid.nx == 0 || grid.ny == 0 {
        return RemainingStockMesh { tolerance: options.tolerance, ..Default::default() };
    }
    remaining_stock_mesh_region(grid, [0, grid.nx - 1, 0, grid.ny - 1], part, index, options)
}

/// The columns in `region` (`[ix0, ix1, iy0, iy1]`, inclusive) with their
/// remaining material: the faces [`stock_mesh_region`] gives that region.
pub fn remaining_stock_mesh_region(
    grid: &DexelGrid,
    region: [u32; 4],
    part: &MeshSnapshot,
    index: &dyn TriangleIndex,
    options: &DeviationOptions,
) -> RemainingStockMesh {
    let mesh = strips(stock_mesh_region(grid, region), grid.cell_x.min(grid.cell_y));
    let start_radius = grid.cell_x.max(grid.cell_y);
    // Ray offsets: far below the cell, far above round-off.
    let jitter = 1e-4 * grid.cell_x.min(grid.cell_y);
    // The quads repeat their corners: evaluate each position once.
    let mut seen: HashMap<[u64; 3], f64> = HashMap::new();
    let mut scratch = Vec::new();
    let mut excess = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
        let value = *seen.entry(key).or_insert_with(|| match nearest_triangle(part, index, *p, start_radius, options.max_distance) {
            Some((_, d)) => {
                if d > options.tolerance && inside_by_majority(part, index, *p, jitter, &mut scratch) {
                    -d
                } else {
                    d
                }
            }
            None => f64::INFINITY,
        });
        excess.push(value);
    }
    // The part's lowest point: stock no higher than it, and not touching the
    // part, is below the part.
    // Not touching the part and no higher than the part's bottom beside it
    // (next to the vertex's nearest part point, else the part's lowest point).
    let mut below_seen: HashMap<[u64; 3], bool> = HashMap::new();
    let below_part: Vec<bool> = mesh
        .positions
        .iter()
        .zip(&excess)
        .map(|(p, v)| {
            if !(*v > options.tolerance) {
                return false;
            }
            let key = [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()];
            *below_seen.entry(key).or_insert_with(|| {
                let beside = nearest_triangle(part, index, *p, start_radius, options.max_distance).and_then(|(tri, _)| {
                    let t = part.triangle(tri);
                    let q = crate::verify::closest_point_on_triangle(*p, t.a, t.b, t.c);
                    crate::verify::part_bottom_beside(part, index, *p, q, jitter * 10.0, &mut scratch)
                });
                p.z <= beside.unwrap_or(part.bounds.min.z) + options.tolerance
            })
        })
        .collect();
    let finite = excess.iter().zip(&below_part).filter(|(_, b)| !**b).map(|(v, _)| *v).filter(|v| v.is_finite());
    let (min, max) = finite.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| (lo.min(v), hi.max(v)));
    let (min, max) = if min.is_finite() { (min, max) } else { (0.0, 0.0) };
    RemainingStockMesh { mesh, excess, min, max, tolerance: options.tolerance, below_part }
}

/// Inside the part by the majority of three vertical-ray parities, cast at
/// fixed offsets of about `jitter` around `p` (none on a symmetry line).
fn inside_by_majority(part: &MeshSnapshot, index: &dyn TriangleIndex, p: crate::model::Point3, jitter: f64, scratch: &mut Vec<u32>) -> bool {
    let offsets = [(1.0, 0.37), (-0.61, 1.0), (0.29, -1.0)];
    let votes = offsets
        .iter()
        .filter(|(dx, dy)| inside_part(part, index, crate::model::Point3::new(p.x + dx * jitter, p.y + dy * jitter, p.z), scratch))
        .count();
    votes >= 2
}

/// Cut every quad of `mesh` taller than `step` into horizontal strips. A
/// dexel quad is either horizontal (one z) or a wall whose corners sit at two
/// heights; each strip keeps the corners' order (so the winding) and moves
/// the low and high corners to the strip's own heights.
fn strips(mesh: StockMesh, step: f64) -> StockMesh {
    let mut out = StockMesh { region: mesh.region, ..Default::default() };
    for q in mesh.positions.chunks_exact(4) {
        let lo = q.iter().map(|p| p.z).fold(f64::INFINITY, f64::min);
        let hi = q.iter().map(|p| p.z).fold(f64::NEG_INFINITY, f64::max);
        let n = if step > 0.0 && hi - lo > step { ((hi - lo) / step).ceil() as usize } else { 1 };
        for k in 0..n {
            let (z0, z1) = (lo + (hi - lo) * k as f64 / n as f64, if k + 1 == n { hi } else { lo + (hi - lo) * (k + 1) as f64 / n as f64 });
            let base = out.positions.len() as u32;
            out.positions.extend(q.iter().map(|p| crate::model::Point3::new(p.x, p.y, if n == 1 { p.z } else if p.z == lo { z0 } else { z1 })));
            out.indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        }
    }
    out
}

