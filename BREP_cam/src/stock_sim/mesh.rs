//! Viewport mesh of the Z-dexel stock.
//!
//! The dexel surface is extracted exactly: every material interval of every
//! column contributes its top and bottom cell-sized quads and, towards each of
//! its four neighbours, the wall pieces where this column has material and the
//! neighbour has none (grid borders count as empty). The result is the
//! watertight, outward-wound boundary of the union of all columns; its signed
//! volume equals the grid's volume exactly. Where neighbouring columns differ
//! in height the walls meet at T-junctions along vertical lines (one tall wall
//! quad against two shorter ones), so the mesh is watertight but not
//! edge-conforming; [`StockMesh::is_watertight`] checks exactly that. Extracting a column REGION gives the faces owned
//! by that region's columns only, and regions tile: the full mesh is the union
//! of any partition's region meshes, so a player rebuilds just the columns a
//! step touched. The display resolution is the grid's cell.

use super::{Column, DexelGrid};
use crate::model::{Bounds3, Point3};
use serde::{Deserialize, Serialize};

/// A plain triangle mesh of the stock surface, counter-clockwise from outside.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct StockMesh {
    pub positions: Vec<Point3>,
    pub indices: Vec<u32>,
    /// Column index range `[ix0, ix1, iy0, iy1]` this mesh covers.
    pub region: [u32; 4],
}

impl StockMesh {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn triangle(&self, i: usize) -> [Point3; 3] {
        let t = &self.indices[3 * i..3 * i + 3];
        [self.positions[t[0] as usize], self.positions[t[1] as usize], self.positions[t[2] as usize]]
    }

    /// Signed volume by the divergence theorem (meaningful for a full extraction).
    pub fn signed_volume(&self) -> f64 {
        (0..self.triangle_count())
            .map(|i| {
                let [a, b, c] = self.triangle(i);
                a.dot(b.cross(c)) / 6.0
            })
            .sum()
    }

    pub fn bounds(&self) -> Bounds3 {
        Bounds3::from_points(self.positions.iter().copied())
    }

    /// Every geometric edge used exactly twice in opposite directions
    /// (vertices welded by exact position).
    pub fn is_closed(&self) -> bool {
        let mut keyed: Vec<([u64; 3], u32)> = self
            .positions
            .iter()
            .enumerate()
            .map(|(i, p)| ([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()], i as u32))
            .collect();
        keyed.sort_unstable();
        let mut canon = vec![0u32; self.positions.len()];
        let mut i = 0;
        while i < keyed.len() {
            let mut j = i;
            while j < keyed.len() && keyed[j].0 == keyed[i].0 {
                canon[keyed[j].1 as usize] = keyed[i].1;
                j += 1;
            }
            i = j;
        }
        let mut edges: Vec<(u32, u32)> = Vec::with_capacity(self.indices.len());
        for t in self.indices.chunks_exact(3) {
            let (a, b, c) = (canon[t[0] as usize], canon[t[1] as usize], canon[t[2] as usize]);
            edges.push((a, b));
            edges.push((b, c));
            edges.push((c, a));
        }
        edges.sort_unstable();
        if edges.windows(2).any(|w| w[0] == w[1]) {
            return false;
        }
        edges.iter().all(|&(a, b)| edges.binary_search(&(b, a)).is_ok())
    }

    /// Watertight: every directed edge is either paired by an opposite edge, or
    /// is a vertical edge whose span is covered by opposite vertical edges on the
    /// same (x, y) line (a T-junction between walls of different heights). The
    /// signed coverage along every vertical line must cancel exactly.
    pub fn is_watertight(&self) -> bool {
        let key = |p: Point3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
        let mut edges: Vec<((u64, u64, u64), (u64, u64, u64))> = Vec::with_capacity(self.indices.len());
        for i in 0..self.triangle_count() {
            let [a, b, c] = self.triangle(i);
            edges.push((key(a), key(b)));
            edges.push((key(b), key(c)));
            edges.push((key(c), key(a)));
        }
        edges.sort_unstable();
        if edges.windows(2).any(|w| w[0] == w[1]) {
            return false;
        }
        // Unpaired edges, grouped by their vertical line.
        let mut spans: Vec<((u64, u64), f64, f64, i8)> = Vec::new();
        for &(a, b) in &edges {
            if edges.binary_search(&(b, a)).is_ok() {
                continue;
            }
            if a.0 != b.0 || a.1 != b.1 {
                return false; // an unpaired non-vertical edge is a real hole
            }
            let (za, zb) = (f64::from_bits(a.2), f64::from_bits(b.2));
            let (lo, hi, sign) = if zb > za { (za, zb, 1i8) } else { (zb, za, -1i8) };
            spans.push(((a.0, a.1), lo, hi, sign));
        }
        spans.sort_by(|p, q| p.0.cmp(&q.0).then(p.1.partial_cmp(&q.1).unwrap_or(std::cmp::Ordering::Equal)));
        let mut i = 0;
        while i < spans.len() {
            let line = spans[i].0;
            let mut j = i;
            while j < spans.len() && spans[j].0 == line {
                j += 1;
            }
            let group = &spans[i..j];
            let mut breaks: Vec<f64> = group.iter().flat_map(|s| [s.1, s.2]).collect();
            breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            breaks.dedup();
            for w in breaks.windows(2) {
                let (lo, hi) = (w[0], w[1]);
                let net: i32 = group.iter().filter(|s| s.1 <= lo && s.2 >= hi).map(|s| s.3 as i32).sum();
                if net != 0 {
                    return false;
                }
            }
            i = j;
        }
        true
    }

    fn quad(&mut self, a: Point3, b: Point3, c: Point3, d: Point3) {
        let base = self.positions.len() as u32;
        self.positions.extend([a, b, c, d]);
        self.indices.extend([base, base + 1, base + 2, base, base + 2, base + 3]);
    }
}

/// Parts of `[a, b]` not covered by `other`'s intervals, in order.
fn uncovered(a: f64, b: f64, other: &Column) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    let mut cursor = a;
    for &[lo, hi] in &other.intervals {
        if hi <= cursor {
            continue;
        }
        if lo >= b {
            break;
        }
        if lo > cursor {
            out.push([cursor, lo]);
        }
        cursor = cursor.max(hi);
        if cursor >= b {
            break;
        }
    }
    if cursor < b {
        out.push([cursor, b]);
    }
    out
}

/// Extract the stock surface of the whole grid.
pub fn stock_mesh(grid: &DexelGrid) -> StockMesh {
    if grid.nx == 0 || grid.ny == 0 {
        return StockMesh::default();
    }
    stock_mesh_region(grid, [0, grid.nx - 1, 0, grid.ny - 1])
}

/// Extract the surface owned by the columns in `region` (`[ix0, ix1, iy0, iy1]`,
/// inclusive, clipped to the grid). Region meshes of a partition tile without
/// overlap: a wall between two columns belongs to the one that has material.
pub fn stock_mesh_region(grid: &DexelGrid, region: [u32; 4]) -> StockMesh {
    let empty = Column::default();
    let [ix0, ix1, iy0, iy1] = region;
    let ix1 = ix1.min(grid.nx.saturating_sub(1));
    let iy1 = iy1.min(grid.ny.saturating_sub(1));
    let mut mesh = StockMesh { region: [ix0, ix1, iy0, iy1], ..Default::default() };
    if ix0 > ix1 || iy0 > iy1 {
        return mesh;
    }
    let neighbour = |ix: i64, iy: i64| -> &Column {
        if ix < 0 || iy < 0 || ix >= grid.nx as i64 || iy >= grid.ny as i64 {
            &empty
        } else {
            grid.column(ix as u32, iy as u32)
        }
    };
    for iy in iy0..=iy1 {
        for ix in ix0..=ix1 {
            let column = grid.column(ix, iy);
            if column.is_empty() {
                continue;
            }
            let x0 = grid.origin.x + ix as f64 * grid.cell_x;
            let x1 = x0 + grid.cell_x;
            let y0 = grid.origin.y + iy as f64 * grid.cell_y;
            let y1 = y0 + grid.cell_y;
            for &[a, b] in &column.intervals {
                // Top (normal +z) and bottom (normal -z).
                mesh.quad(Point3::new(x0, y0, b), Point3::new(x1, y0, b), Point3::new(x1, y1, b), Point3::new(x0, y1, b));
                mesh.quad(Point3::new(x0, y0, a), Point3::new(x0, y1, a), Point3::new(x1, y1, a), Point3::new(x1, y0, a));
                // Walls towards +x, -x, +y, -y where the neighbour has no material.
                for [lo, hi] in uncovered(a, b, neighbour(ix as i64 + 1, iy as i64)) {
                    mesh.quad(Point3::new(x1, y0, lo), Point3::new(x1, y1, lo), Point3::new(x1, y1, hi), Point3::new(x1, y0, hi));
                }
                for [lo, hi] in uncovered(a, b, neighbour(ix as i64 - 1, iy as i64)) {
                    mesh.quad(Point3::new(x0, y0, lo), Point3::new(x0, y0, hi), Point3::new(x0, y1, hi), Point3::new(x0, y1, lo));
                }
                for [lo, hi] in uncovered(a, b, neighbour(ix as i64, iy as i64 + 1)) {
                    mesh.quad(Point3::new(x0, y1, lo), Point3::new(x0, y1, hi), Point3::new(x1, y1, hi), Point3::new(x1, y1, lo));
                }
                for [lo, hi] in uncovered(a, b, neighbour(ix as i64, iy as i64 - 1)) {
                    mesh.quad(Point3::new(x0, y0, lo), Point3::new(x1, y0, lo), Point3::new(x1, y0, hi), Point3::new(x0, y0, hi));
                }
            }
        }
    }
    mesh
}

/// Column region a move's footprint touched (for incremental rebuilds):
/// the grid columns whose centres lie within `radius` of the segment's box.
pub fn touched_region(grid: &DexelGrid, start: Point3, end: Point3, radius: f64) -> Option<[u32; 4]> {
    grid.columns_in(&Bounds3::from_points([start, end]).expanded(radius))
}
