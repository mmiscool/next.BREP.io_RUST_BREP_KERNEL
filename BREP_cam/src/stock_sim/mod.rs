//! Stock model for material-removal simulation: a Z-dexel grid.
//!
//! The stock is a grid of vertical columns in MACHINE XY; every column holds
//! its material as a sorted list of disjoint `[low, high]` Z intervals. This is
//! the data structure the exact per-column floor of a 3-axis move writes into
//! directly ([`removal`]), so removal never approximates the floor: only the XY
//! sampling is discrete, and the cell size is a reported output, never a
//! silent coarsening. Limits, stated: undercuts (T-slot undersides, lollipop
//! upper surfaces) are not simulated by a Z-dexel; moves that would need them
//! are reported as unsupported by the removal pass, never simulated wrongly.

pub mod carry;
pub mod envelope;
pub mod frame;
pub mod job;
pub mod mesh;
pub mod preop;
pub mod remaining;
pub mod removal;

use crate::index::{ProjectionMode, TriangleIndex};
use crate::mesh::MeshSnapshot;
use crate::model::{Bounds3, Point3};
use crate::stock::ResolvedStock;
use serde::{Deserialize, Serialize};

/// How the grid is laid out and how much it may cost.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct DexelOptions {
    /// Requested cell size in model units (both axes are at most this; a box
    /// stock gets cells that divide its sides exactly so its volume is exact).
    pub cell: f64,
    /// Work budget: the grid may not exceed this many columns.
    pub max_columns: u64,
}

impl Default for DexelOptions {
    fn default() -> Self {
        Self { cell: 0.5, max_columns: 4_000_000 }
    }
}

/// Why a stock could not become a grid.
#[derive(Clone, Debug, PartialEq)]
pub enum StockSimError {
    /// The requested cell would need more columns than the budget allows.
    /// `cell_that_fits` is the smallest cell within budget.
    BudgetExceeded { requested_cell: f64, columns: u64, max_columns: u64, cell_that_fits: f64 },
    /// `Stock::FromRest` needs the previous setup's simulated stock.
    RestStockMissing,
    /// A `FromSolid` stock mesh is open at some column (odd crossing count on
    /// the centre ray and on every jittered ray).
    OpenStockMesh { column: [u32; 2] },
    /// Non-finite or non-positive cell / empty stock bounds.
    InvalidInput(String),
}

impl std::fmt::Display for StockSimError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StockSimError::BudgetExceeded { requested_cell, columns, max_columns, cell_that_fits } => write!(
                f,
                "stock grid at cell {requested_cell} needs {columns} columns, over the budget of {max_columns}; a cell of {cell_that_fits} fits"
            ),
            StockSimError::RestStockMissing => write!(f, "rest stock needs the previous setup's simulated stock"),
            StockSimError::OpenStockMesh { column } => {
                write!(f, "stock mesh is not closed at column ({}, {})", column[0], column[1])
            }
            StockSimError::InvalidInput(m) => write!(f, "stock grid: {m}"),
        }
    }
}

impl std::error::Error for StockSimError {}

/// Material of one column: sorted, disjoint, non-empty `[low, high]` intervals.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct Column {
    pub intervals: Vec<[f64; 2]>,
}

impl Column {
    pub fn full(low: f64, high: f64) -> Self {
        Self { intervals: vec![[low, high]] }
    }

    pub fn is_empty(&self) -> bool {
        self.intervals.is_empty()
    }

    /// Total material height.
    pub fn height(&self) -> f64 {
        self.intervals.iter().map(|[a, b]| b - a).sum()
    }

    /// Highest material Z, if any.
    pub fn top(&self) -> Option<f64> {
        self.intervals.last().map(|[_, b]| *b)
    }

    /// Remove material in `[low, high]`; returns the height removed. Raw float
    /// compares, never snapped, so commuting cuts give identical columns.
    pub fn cut(&mut self, low: f64, high: f64) -> f64 {
        if !(high > low) {
            return 0.0;
        }
        let mut removed = 0.0;
        let mut out: Vec<[f64; 2]> = Vec::with_capacity(self.intervals.len() + 1);
        for &[a, b] in &self.intervals {
            if b <= low || a >= high {
                out.push([a, b]);
                continue;
            }
            // Overlap [max(a, low), min(b, high)].
            let cut_lo = a.max(low);
            let cut_hi = b.min(high);
            removed += cut_hi - cut_lo;
            if a < cut_lo {
                out.push([a, cut_lo]);
            }
            if cut_hi < b {
                out.push([cut_hi, b]);
            }
        }
        self.intervals = out;
        removed
    }

    /// Material present anywhere in `[low, high]`.
    pub fn has_material(&self, low: f64, high: f64) -> bool {
        self.intervals.iter().any(|[a, b]| *b > low && *a < high)
    }
}

/// The Z-dexel stock. Column `(ix, iy)` is centred at
/// `(origin.x + (ix + 0.5)·cell_x, origin.y + (iy + 0.5)·cell_y)`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DexelGrid {
    pub origin: Point3,
    pub cell_x: f64,
    pub cell_y: f64,
    pub nx: u32,
    pub ny: u32,
    /// Row-major: `columns[iy * nx + ix]`.
    pub columns: Vec<Column>,
    /// Z range of the initial stock (for display and for the top-of-stock).
    pub z_range: [f64; 2],
}

impl DexelGrid {
    /// Lay a grid over `bounds` with cells at most `cell` on each side,
    /// dividing the sides exactly.
    fn layout(bounds: &Bounds3, options: &DexelOptions) -> Result<(Point3, f64, f64, u32, u32), StockSimError> {
        if !(options.cell > 0.0) || !options.cell.is_finite() {
            return Err(StockSimError::InvalidInput(format!("cell {} must be finite and positive", options.cell)));
        }
        if bounds.is_empty() || !bounds.is_finite() {
            return Err(StockSimError::InvalidInput("stock bounds are empty or not finite".into()));
        }
        let size = bounds.size();
        let nx = (size.x / options.cell).ceil().max(1.0);
        let ny = (size.y / options.cell).ceil().max(1.0);
        let columns = nx * ny;
        if columns > options.max_columns as f64 {
            let area = size.x * size.y;
            let cell_that_fits = (area / options.max_columns as f64).sqrt() * 1.001;
            return Err(StockSimError::BudgetExceeded {
                requested_cell: options.cell,
                columns: columns as u64,
                max_columns: options.max_columns,
                cell_that_fits,
            });
        }
        Ok((bounds.min, size.x / nx, size.y / ny, nx as u32, ny as u32))
    }

    /// Seed the grid from a resolved stock. `rest` supplies the previous
    /// setup's simulated stock for `ResolvedStock::Rest`; `index` speeds up
    /// mesh stock seeding (brute force without it).
    pub fn from_stock(
        stock: &ResolvedStock,
        options: &DexelOptions,
        rest: Option<&DexelGrid>,
        index: Option<&dyn TriangleIndex>,
    ) -> Result<DexelGrid, StockSimError> {
        match stock {
            ResolvedStock::Box(b) => {
                let bounds = b.machine_bounds();
                let (origin, cell_x, cell_y, nx, ny) = Self::layout(&bounds, options)?;
                let is_aligned = b.to_machine.rotation.iter().flatten().all(|v| v.abs() < 1e-12 || (v.abs() - 1.0).abs() < 1e-12);
                let mut grid = Self::empty(origin, cell_x, cell_y, nx, ny, [bounds.min.z, bounds.max.z]);
                for iy in 0..ny {
                    for ix in 0..nx {
                        let (x, y) = grid.center(ix, iy);
                        let inside = is_aligned || b.contains(Point3::new(x, y, 0.5 * (bounds.min.z + bounds.max.z)));
                        if inside {
                            grid.columns[(iy * nx + ix) as usize] = Column::full(bounds.min.z, bounds.max.z);
                        }
                    }
                }
                Ok(grid)
            }
            ResolvedStock::Cylinder(c) => {
                let bounds = c.machine_bounds();
                let (origin, cell_x, cell_y, nx, ny) = Self::layout(&bounds, options)?;
                let mut grid = Self::empty(origin, cell_x, cell_y, nx, ny, [bounds.min.z, bounds.max.z]);
                let vertical = (c.axis.z.abs() - 1.0).abs() < 1e-12;
                for iy in 0..ny {
                    for ix in 0..nx {
                        let (x, y) = grid.center(ix, iy);
                        if vertical {
                            let d = ((x - c.base.x).powi(2) + (y - c.base.y).powi(2)).sqrt();
                            if d <= c.radius {
                                grid.columns[(iy * nx + ix) as usize] = Column::full(bounds.min.z, bounds.max.z);
                            }
                        } else {
                            // Tilted cylinder: intersect the vertical line with the
                            // infinite cylinder, clip by the end planes.
                            if let Some(iv) = cylinder_column(c, x, y) {
                                grid.columns[(iy * nx + ix) as usize] = Column::full(iv[0], iv[1]);
                            }
                        }
                    }
                }
                Ok(grid)
            }
            ResolvedStock::Mesh(mesh) => Self::from_mesh(mesh, options, index),
            ResolvedStock::Rest(_) => rest.cloned().ok_or(StockSimError::RestStockMissing),
        }
    }

    /// Seed from a closed mesh in machine coordinates by casting one vertical
    /// ray per column and pairing crossings. A ray through a shared edge or
    /// vertex meets every triangle there, and their plane evaluations differ
    /// by round-off, so an odd count is first re-cast through points jittered
    /// inside the cell; a column whose every ray stays odd is an open mesh.
    pub fn from_mesh(mesh: &MeshSnapshot, options: &DexelOptions, index: Option<&dyn TriangleIndex>) -> Result<DexelGrid, StockSimError> {
        Self::seed_from_mesh(mesh, options, index, false).map(|(grid, _)| grid)
    }

    /// A PROTECTION grid from a target mesh: [`from_mesh`](Self::from_mesh),
    /// except that a column where the mesh is open (odd crossings on every
    /// jittered ray) is filled conservatively over the mesh's whole height
    /// (a missing face may be the top or the bottom, so no crossing bounds
    /// it), and no part material can be missing from it. The fill
    /// is named (`open-mesh-protection`, with the column count) rather than
    /// refused: a checked replay must never run with no protection because
    /// the target mesh has a hole.
    pub fn protected_from_mesh(
        mesh: &MeshSnapshot,
        options: &DexelOptions,
        index: Option<&dyn TriangleIndex>,
    ) -> Result<(DexelGrid, Option<crate::model::ApproximationLimit>), StockSimError> {
        let (grid, open) = Self::seed_from_mesh(mesh, options, index, true)?;
        let limit = (open > 0).then(|| crate::model::ApproximationLimit {
            kind: "open-mesh-protection".into(),
            bound: None,
            detail: Some(format!(
                "the protected mesh is open at {open} column(s); each is protected over the mesh's whole height"
            )),
        });
        Ok((grid, limit))
    }

    fn seed_from_mesh(
        mesh: &MeshSnapshot,
        options: &DexelOptions,
        index: Option<&dyn TriangleIndex>,
        fill_open: bool,
    ) -> Result<(DexelGrid, usize), StockSimError> {
        let bounds = mesh.bounds;
        let (origin, cell_x, cell_y, nx, ny) = Self::layout(&bounds, options)?;
        let mut grid = Self::empty(origin, cell_x, cell_y, nx, ny, [bounds.min.z, bounds.max.z]);
        let brute = crate::index::BruteForceIndex::from_snapshot(mesh);
        let idx: &dyn TriangleIndex = index.unwrap_or(&brute);
        let mut candidates = Vec::new();
        let mut hits: Vec<f64> = Vec::new();
        // Fixed, deterministic offsets inside the cell (fractions of the
        // smaller cell side), none on a symmetry line of the cell.
        const JITTER: [(f64, f64); 4] = [(1.0e-3, 0.37e-3), (-0.61e-3, 1.0e-3), (0.29e-3, -1.0e-3), (-1.0e-3, -0.53e-3)];
        let side = cell_x.min(cell_y);
        let mut open = 0usize;
        for iy in 0..ny {
            for ix in 0..nx {
                let (cx, cy) = grid.center(ix, iy);
                let mut cast = |x: f64, y: f64, hits: &mut Vec<f64>| {
                    candidates.clear();
                    let q = Bounds3::new(Point3::new(x, y, f64::NEG_INFINITY), Point3::new(x, y, f64::INFINITY));
                    idx.query_aabb(&q, ProjectionMode::Xy, &mut candidates);
                    hits.clear();
                    for &id in &candidates {
                        let t = mesh.triangle(id);
                        if let Some(z) = vertical_ray_hit(t.a, t.b, t.c, x, y) {
                            hits.push(z);
                        }
                    }
                    hits.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                    // Exact duplicates come from a ray through a shared edge: keep one.
                    hits.dedup();
                };
                cast(cx, cy, &mut hits);
                if hits.is_empty() {
                    continue;
                }
                let mut k = 0;
                while hits.len() % 2 == 1 && k < JITTER.len() {
                    let (jx, jy) = JITTER[k];
                    cast(cx + jx * side, cy + jy * side, &mut hits);
                    k += 1;
                }
                let column = &mut grid.columns[(iy * nx + ix) as usize];
                if hits.len() % 2 == 1 {
                    if !fill_open {
                        return Err(StockSimError::OpenStockMesh { column: [ix, iy] });
                    }
                    open += 1;
                    column.intervals.push([bounds.min.z, bounds.max.z]);
                    continue;
                }
                for pair in hits.chunks_exact(2) {
                    if pair[1] > pair[0] {
                        column.intervals.push([pair[0], pair[1]]);
                    }
                }
            }
        }
        Ok((grid, open))
    }

    fn empty(origin: Point3, cell_x: f64, cell_y: f64, nx: u32, ny: u32, z_range: [f64; 2]) -> Self {
        Self {
            origin,
            cell_x,
            cell_y,
            nx,
            ny,
            columns: vec![Column::default(); (nx as usize) * (ny as usize)],
            z_range,
        }
    }

    /// Centre of column `(ix, iy)`.
    pub fn center(&self, ix: u32, iy: u32) -> (f64, f64) {
        (self.origin.x + (ix as f64 + 0.5) * self.cell_x, self.origin.y + (iy as f64 + 0.5) * self.cell_y)
    }

    pub fn column(&self, ix: u32, iy: u32) -> &Column {
        &self.columns[(iy * self.nx + ix) as usize]
    }

    pub fn column_mut(&mut self, ix: u32, iy: u32) -> &mut Column {
        &mut self.columns[(iy * self.nx + ix) as usize]
    }

    /// Column index range `[ix0, ix1] × [iy0, iy1]` whose centres fall inside
    /// `bounds` (XY), clipped to the grid; `None` when nothing overlaps.
    pub fn columns_in(&self, bounds: &Bounds3) -> Option<[u32; 4]> {
        let fx = |x: f64| (x - self.origin.x) / self.cell_x - 0.5;
        let fy = |y: f64| (y - self.origin.y) / self.cell_y - 0.5;
        let ix0 = fx(bounds.min.x).ceil().max(0.0);
        let ix1 = fx(bounds.max.x).floor().min(self.nx as f64 - 1.0);
        let iy0 = fy(bounds.min.y).ceil().max(0.0);
        let iy1 = fy(bounds.max.y).floor().min(self.ny as f64 - 1.0);
        if ix0 > ix1 || iy0 > iy1 {
            return None;
        }
        Some([ix0 as u32, ix1 as u32, iy0 as u32, iy1 as u32])
    }

    pub fn cell_area(&self) -> f64 {
        self.cell_x * self.cell_y
    }

    /// Material volume.
    pub fn volume(&self) -> f64 {
        self.columns.iter().map(|c| c.height()).sum::<f64>() * self.cell_area()
    }

    /// Columns plus intervals: the memory this grid costs, in work units.
    pub fn work_units(&self) -> u64 {
        self.columns.len() as u64 + self.columns.iter().map(|c| c.intervals.len() as u64).sum::<u64>()
    }

    /// Top of material at the column containing `(x, y)`, if any.
    pub fn top_at(&self, x: f64, y: f64) -> Option<f64> {
        let ix = ((x - self.origin.x) / self.cell_x).floor();
        let iy = ((y - self.origin.y) / self.cell_y).floor();
        if ix < 0.0 || iy < 0.0 || ix >= self.nx as f64 || iy >= self.ny as f64 {
            return None;
        }
        self.column(ix as u32, iy as u32).top()
    }

    /// Machine-space bounds of the grid footprint and the initial Z range.
    pub fn bounds(&self) -> Bounds3 {
        Bounds3::new(
            Point3::new(self.origin.x, self.origin.y, self.z_range[0]),
            Point3::new(self.origin.x + self.nx as f64 * self.cell_x, self.origin.y + self.ny as f64 * self.cell_y, self.z_range[1]),
        )
    }
}

/// Z where the vertical line through `(x, y)` crosses triangle `abc`
/// (closed test; a hit on a shared edge is reported by both triangles and
/// de-duplicated by the caller). `None` for vertical triangles.
pub(crate) fn vertical_ray_hit(a: Point3, b: Point3, c: Point3, x: f64, y: f64) -> Option<f64> {
    let side = |p: Point3, q: Point3| (q.x - p.x) * (y - p.y) - (q.y - p.y) * (x - p.x);
    let (d1, d2, d3) = (side(a, b), side(b, c), side(c, a));
    let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    if has_neg && has_pos {
        return None;
    }
    let n = b.sub(a).cross(c.sub(a));
    if n.z.abs() <= f64::EPSILON * (n.length() + 1.0) {
        return None;
    }
    Some(a.z - (n.x * (x - a.x) + n.y * (y - a.y)) / n.z)
}

/// Z interval of a tilted cylinder along the vertical line through `(x, y)`.
fn cylinder_column(c: &crate::stock::StockCylinder, x: f64, y: f64) -> Option<[f64; 2]> {
    // Points p(z) = (x, y, z); radial distance to the axis line (base, axis):
    // |(p - base) - ((p - base)·a) a|² = r² is a quadratic in z.
    let a = c.axis;
    let d0 = Point3::new(x - c.base.x, y - c.base.y, -c.base.z);
    let dz = Point3::Z;
    // v(z) = d0 + z dz; radial = v - (v·a) a.
    let r0 = d0.sub(a.scale(d0.dot(a)));
    let r1 = dz.sub(a.scale(dz.dot(a)));
    let qa = r1.dot(r1);
    let qb = 2.0 * r0.dot(r1);
    let qc = r0.dot(r0) - c.radius * c.radius;
    let (mut lo, mut hi) = if qa.abs() < 1e-15 {
        if qc > 0.0 {
            return None;
        }
        (f64::NEG_INFINITY, f64::INFINITY)
    } else {
        let disc = qb * qb - 4.0 * qa * qc;
        if disc < 0.0 {
            return None;
        }
        let s = disc.sqrt();
        ((-qb - s) / (2.0 * qa), (-qb + s) / (2.0 * qa))
    };
    // End planes: 0 <= (p - base)·a <= height, i.e. d0·a + z (a·z) in [0, h].
    let s0 = d0.dot(a);
    let s1 = dz.dot(a);
    if s1.abs() < 1e-15 {
        if s0 < 0.0 || s0 > c.height {
            return None;
        }
    } else {
        let z_a = -s0 / s1;
        let z_b = (c.height - s0) / s1;
        lo = lo.max(z_a.min(z_b));
        hi = hi.min(z_a.max(z_b));
    }
    (hi > lo).then_some([lo, hi])
}


