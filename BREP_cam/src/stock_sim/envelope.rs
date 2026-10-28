//! Highest-material queries over a frozen stock state, for linking.
//!
//! The linker proves its moves against the final protected mesh; material
//! that is still uncut while an operation runs is invisible to it. A
//! [`StockEnvelope`] is the stock as it stands at the start of the operation
//! (the initial stock, or the replay of the previous operations), reduced to
//! one top Z per column, and answers "how high is the material anywhere under
//! a disc swept along this segment". The answer is CONSERVATIVE: a column
//! counts as soon as its cell square — not just its sample centre — meets the
//! swept disc, so material between sample centres is never missed, and an
//! unknowable query (non-finite input) answers with the highest material in
//! the whole stock. Material removed by the operation being linked is not
//! credited: links retract higher than strictly needed, never lower.
//!
//! Queries are in the LINK frame (the setup's machine coordinates). The
//! stock grid may be in another frame related by a transform that keeps +Z
//! vertical (a rotation about Z, e.g. an indexed C section, plus a
//! translation); anything that tilts Z is refused by name.

use super::{DexelGrid, StockSimError};
use crate::model::{Point3, RigidTransform};

/// Per-column material tops of a stock state, queried in the link frame.
#[derive(Clone, Debug, PartialEq)]
pub struct StockEnvelope {
    origin_x: f64,
    origin_y: f64,
    cell_x: f64,
    cell_y: f64,
    nx: u32,
    ny: u32,
    /// Row-major top Z per column (stock frame), `None` where the column is empty.
    tops: Vec<Option<f64>>,
    /// Highest top anywhere (stock frame).
    max_top: Option<f64>,
    /// Link frame -> stock frame.
    to_stock: RigidTransform,
}

impl StockEnvelope {
    /// The envelope of `grid`, queried through `to_stock` (link frame to the
    /// grid's frame; the identity for a 3-axis setup).
    pub fn new(grid: &DexelGrid, to_stock: RigidTransform) -> Result<Self, StockSimError> {
        to_stock.validate().map_err(|e| StockSimError::InvalidInput(format!("stock envelope frame: {e}")))?;
        if to_stock.apply_vector(Point3::Z).distance(Point3::Z) > 1e-9 {
            return Err(StockSimError::InvalidInput(
                "stock envelope frame must keep +Z vertical (a rotation about Z and a translation)".into(),
            ));
        }
        let tops: Vec<Option<f64>> = grid.columns.iter().map(|c| c.top()).collect();
        let max_top = tops.iter().flatten().copied().fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.max(t))));
        Ok(Self {
            origin_x: grid.origin.x,
            origin_y: grid.origin.y,
            cell_x: grid.cell_x,
            cell_y: grid.cell_y,
            nx: grid.nx,
            ny: grid.ny,
            tops,
            max_top,
            to_stock,
        })
    }

    /// Stock-frame Z to link-frame Z (the frame keeps Z vertical).
    fn to_link_z(&self, z: f64) -> f64 {
        z - self.to_stock.translation.z
    }

    /// Highest material anywhere in the stock, in link Z.
    pub fn max_top(&self) -> Option<f64> {
        self.max_top.map(|z| self.to_link_z(z))
    }

    /// A copy keeping only the columns `keep` accepts; `keep` gets each
    /// column's top (link Z) and its cell footprint's four corners in the
    /// link frame (Z 0). Used to reduce the stock to the material above the
    /// part (lane C16's shank and holder certification of finishing cuts).
    pub fn filtered(&self, mut keep: impl FnMut(f64, [Point3; 4]) -> bool) -> Self {
        let to_link = self.to_stock.inverse();
        let mut out = self.clone();
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                let k = (iy * self.nx + ix) as usize;
                let Some(top) = self.tops[k] else { continue };
                let (x0, y0) = (self.origin_x + ix as f64 * self.cell_x, self.origin_y + iy as f64 * self.cell_y);
                let corner = |x: f64, y: f64| {
                    let p = to_link.apply_point(Point3::new(x, y, 0.0));
                    Point3::new(p.x, p.y, 0.0)
                };
                let corners = [corner(x0, y0), corner(x0 + self.cell_x, y0), corner(x0 + self.cell_x, y0 + self.cell_y), corner(x0, y0 + self.cell_y)];
                if !keep(self.to_link_z(top), corners) {
                    out.tops[k] = None;
                }
            }
        }
        out.max_top = out.tops.iter().flatten().copied().fold(None, |m: Option<f64>, t| Some(m.map_or(t, |m| m.max(t))));
        out
    }

    /// Cell size of the columns (stock frame).
    pub fn cell(&self) -> (f64, f64) {
        (self.cell_x, self.cell_y)
    }

    /// Highest material under a disc of `radius` at `p` (its Z is ignored).
    pub fn top_at(&self, p: Point3, radius: f64) -> Option<f64> {
        self.top_over_segment(p, p, radius)
    }

    /// Highest material Z (link frame) under the footprint swept by a disc of
    /// `radius` whose centre moves along the XY projection of `a`→`b` (their Z
    /// is ignored). `None`: no material under it. A negative radius is a
    /// point query; non-finite input answers with [`max_top`](Self::max_top).
    pub fn top_over_segment(&self, a: Point3, b: Point3, radius: f64) -> Option<f64> {
        if !a.is_finite() || !b.is_finite() || radius.is_nan() || radius == f64::INFINITY {
            return self.max_top();
        }
        let r = radius.max(0.0);
        let sa = self.to_stock.apply_point(a);
        let sb = self.to_stock.apply_point(b);
        let (p, q) = ((sa.x, sa.y), (sb.x, sb.y));
        // Columns whose cell square can meet the capsule: its bounding box.
        let fx = |x: f64| ((x - self.origin_x) / self.cell_x).floor();
        let fy = |y: f64| ((y - self.origin_y) / self.cell_y).floor();
        // One extra column each side: a disc ending exactly on a cell boundary
        // touches the neighbour's edge (the exact test below decides).
        let ix0 = (fx(p.0.min(q.0) - r) - 1.0).max(0.0);
        let ix1 = (fx(p.0.max(q.0) + r) + 1.0).min(self.nx as f64 - 1.0);
        let iy0 = (fy(p.1.min(q.1) - r) - 1.0).max(0.0);
        let iy1 = (fy(p.1.max(q.1) + r) + 1.0).min(self.ny as f64 - 1.0);
        if ix0 > ix1 || iy0 > iy1 {
            return None;
        }
        let mut best: Option<f64> = None;
        for iy in iy0 as u32..=iy1 as u32 {
            for ix in ix0 as u32..=ix1 as u32 {
                let Some(top) = self.tops[(iy * self.nx + ix) as usize] else { continue };
                if best.is_some_and(|b| b >= top) {
                    continue;
                }
                let x0 = self.origin_x + ix as f64 * self.cell_x;
                let y0 = self.origin_y + iy as f64 * self.cell_y;
                if segment_rect_distance(p, q, (x0, y0), (x0 + self.cell_x, y0 + self.cell_y)) <= r {
                    best = Some(top);
                }
            }
        }
        best.map(|z| self.to_link_z(z))
    }
}

fn point_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    ((p.0 - a.0 - t * dx).powi(2) + (p.1 - a.1 - t * dy).powi(2)).sqrt()
}

fn point_rect_distance(p: (f64, f64), lo: (f64, f64), hi: (f64, f64)) -> f64 {
    let dx = (lo.0 - p.0).max(0.0).max(p.0 - hi.0);
    let dy = (lo.1 - p.1).max(0.0).max(p.1 - hi.1);
    (dx * dx + dy * dy).sqrt()
}

/// Whether segment `a`-`b` meets the closed rectangle `[lo, hi]` (Liang–Barsky clip).
fn segment_meets_rect(a: (f64, f64), b: (f64, f64), lo: (f64, f64), hi: (f64, f64)) -> bool {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let (mut t0, mut t1) = (0.0f64, 1.0f64);
    for (pk, qk) in [(-dx, a.0 - lo.0), (dx, hi.0 - a.0), (-dy, a.1 - lo.1), (dy, hi.1 - a.1)] {
        if pk == 0.0 {
            if qk < 0.0 {
                return false;
            }
        } else {
            let t = qk / pk;
            if pk < 0.0 {
                t0 = t0.max(t);
            } else {
                t1 = t1.min(t);
            }
            if t0 > t1 {
                return false;
            }
        }
    }
    true
}

/// Exact distance between a segment and a closed axis-aligned rectangle: zero
/// when they meet, else the least of the segment ends to the rectangle and the
/// rectangle corners to the segment (the minimum of two disjoint convex sets
/// in the plane is attained at a vertex of one of them).
fn segment_rect_distance(a: (f64, f64), b: (f64, f64), lo: (f64, f64), hi: (f64, f64)) -> f64 {
    if segment_meets_rect(a, b, lo, hi) {
        return 0.0;
    }
    let ends = point_rect_distance(a, lo, hi).min(point_rect_distance(b, lo, hi));
    [(lo.0, lo.1), (hi.0, lo.1), (hi.0, hi.1), (lo.0, hi.1)]
        .into_iter()
        .map(|c| point_segment_distance(c, a, b))
        .fold(ends, f64::min)
}

