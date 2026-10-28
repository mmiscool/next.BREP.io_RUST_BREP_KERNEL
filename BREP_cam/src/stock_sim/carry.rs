//! Stock carried from one setup to the next ("continue from the previous setup").
//!
//! The next setup's initial stock is the previous setup's final in-process
//! stock, moved into the next setup's machine frame. Both setups place the
//! same model: `machine_k = M_k(model)`, so a point moves between them by
//! `T = M_{k+1} ∘ M_k⁻¹`. The Z-dexel grid is resampled in the new frame:
//! every material box of the old grid (a cell square × an interval) is moved
//! by `T`, and every new column whose cell square overlaps the moved box's
//! bounding box (with positive area) receives the box's Z range. That is a
//! superset of the true material, so no column ever reads below material.
//!
//! When `T` maps grid axes onto grid axes (flips, quarter turns, translations
//! with aligned cells) every box lands exactly on whole cells and the result
//! is exact, including voids now lying under material: dexel columns hold
//! several intervals, so a top pocket flipped to the bottom stays a gap below
//! the material. An oblique turn cannot be held exactly by columns on the new
//! grid; the result is then a named conservative superset (`oblique-resample`)
//! with a bound on how far material may have grown.

use super::envelope::StockEnvelope;
use super::preop::{PlannedOp, PreOpStockCache, PreOpStockError};
use super::removal::RemovalOptions;
use super::{Column, DexelGrid, DexelOptions, StockSimError};
use crate::cancel::Runner;
use crate::model::{ApproximationLimit, Bounds3, Machine, Point3, RigidTransform, Setup};
use crate::numeric::Tolerance;
use crate::stock::ResolvedStock;

/// The previous setup's stock in the next setup's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct CarriedStock {
    pub grid: DexelGrid,
    /// Envelope over `grid` in the new setup's machine frame.
    pub envelope: StockEnvelope,
    /// `true` when the resample is exact (axis-aligned transform on aligned cells).
    pub exact: bool,
    /// Named approximations when not exact.
    pub limits: Vec<ApproximationLimit>,
}

/// Move `grid` (in `from`'s machine frame) into `to`'s machine frame.
/// `options` sets the new cell size where the transform does not map grid
/// axes onto grid axes, and the column budget in every case.
pub fn carry_stock(grid: &DexelGrid, from: &Setup, to: &Setup, options: &DexelOptions) -> Result<CarriedStock, StockSimError> {
    let bad = |e: crate::model::TransformError| StockSimError::InvalidInput(format!("setup transform: {e}"));
    from.model_to_machine.validate().map_err(bad)?;
    to.model_to_machine.validate().map_err(bad)?;
    let t = from.model_to_machine.inverse().then(&to.model_to_machine);
    let (grid, exact, limits) = transform_grid(grid, &t, options)?;
    let envelope = StockEnvelope::new(&grid, RigidTransform::identity())?;
    Ok(CarriedStock { grid, envelope, exact, limits })
}

/// The stock `to` starts from when it continues from `from`: `from`'s stock
/// after all of `ops` (through the pre-operation cache, so an unchanged
/// previous setup is not replayed again), carried into `to`'s frame. The
/// arguments before `to` are those of [`PreOpStockCache::stock_before`].
#[allow(clippy::too_many_arguments)]
pub fn carry_from_setup(
    cache: &mut PreOpStockCache,
    from: &Setup,
    machine: Option<&Machine>,
    initial: &ResolvedStock,
    rest: Option<&DexelGrid>,
    options: &DexelOptions,
    ops: &[PlannedOp<'_>],
    tol: &Tolerance,
    removal: &RemovalOptions,
    runner: &mut Runner<'_>,
    to: &Setup,
) -> Result<CarriedStock, PreOpStockError> {
    let after = cache.stock_before(from, machine, initial, rest, options, ops, ops.len(), tol, removal, runner)?;
    carry_stock(&after.grid, from, to, options).map_err(PreOpStockError::Grid)
}

/// For an axis-permuting rotation, the old axis (0, 1, 2) each new axis comes from.
fn axis_map(t: &RigidTransform) -> Option<[usize; 3]> {
    let r = &t.rotation;
    let mut map = [0usize; 3];
    for (i, row) in r.iter().enumerate() {
        let mut found = None;
        for (j, v) in row.iter().enumerate() {
            if (v.abs() - 1.0).abs() < 1e-12 {
                found = Some(j);
            } else if v.abs() > 1e-12 {
                return None;
            }
        }
        map[i] = found?;
    }
    Some(map)
}

/// Resample `grid` through the rigid transform `t`.
pub fn transform_grid(grid: &DexelGrid, t: &RigidTransform, options: &DexelOptions) -> Result<(DexelGrid, bool, Vec<ApproximationLimit>), StockSimError> {
    t.validate().map_err(|e| StockSimError::InvalidInput(format!("stock transform: {e}")))?;
    let old_cells = [grid.cell_x, grid.cell_y];
    let map = axis_map(t);
    // An axis-permuting rotation built from angles carries round-off (sin(pi)
    // is 1.2e-16): snap it to its exact signed permutation so flips and
    // quarter turns move interval ends without drift.
    let snapped;
    let t = match map {
        Some(_) => {
            let mut e = *t;
            for row in e.rotation.iter_mut() {
                for v in row.iter_mut() {
                    *v = v.round();
                }
            }
            snapped = e;
            &snapped
        }
        None => t,
    };
    // New cell sizes: inherited along axes that come from old grid axes.
    let cell_of = |new_axis: usize| -> f64 {
        match map {
            Some(m) if m[new_axis] < 2 => old_cells[m[new_axis]],
            _ => options.cell,
        }
    };
    let (cx, cy) = (cell_of(0), cell_of(1));
    if !(cx > 0.0) || !(cy > 0.0) || !cx.is_finite() || !cy.is_finite() {
        return Err(StockSimError::InvalidInput(format!("cell {} must be finite and positive", options.cell)));
    }
    // A Z-dexel column knows its material along its centre line only: beside
    // a wall that passes between two column centres, material reaches from
    // the full column's centre up to (not past) the empty neighbour's centre,
    // half a cell beyond the full column's own cell. Carried onto the same
    // column lines (z stays vertical, axes onto axes) that is the source's own
    // representation and is kept exactly; carried anywhere else that lateral
    // shortfall becomes a shortfall of the new columns, possibly a vertical
    // one (a 90° turn makes an old wall a new top, C20 plate setup 4: 0.34 =
    // half the 0.68 cell under the part). There every box is widened by half
    // a cell on each side, so no column reads below the source's material.
    // A quarter turn widens only the old axis that becomes the new Z: the
    // other old axis stays a column axis on the same lattice (its cells are
    // inherited and its moved edges are the new origin), so its walls keep
    // the source's own representation. Widening it too added half a cell to
    // every such wall pair (the blade's three quarter turns, about 2,000 mm³
    // each).
    let same_columns = matches!(map, Some(m) if m[2] == 2);
    let (wx, wy) = match map {
        _ if same_columns => (0.0, 0.0),
        Some(m) if m[2] == 0 => (0.5 * grid.cell_x, 0.0),
        Some(_) => (0.0, 0.5 * grid.cell_y),
        None => (0.5 * grid.cell_x, 0.5 * grid.cell_y),
    };
    // Moved boxes and the new bounds.
    let mut boxes: Vec<Bounds3> = Vec::new();
    let mut bounds = Bounds3::empty();
    for iy in 0..grid.ny {
        for ix in 0..grid.nx {
            let x0 = grid.origin.x + ix as f64 * grid.cell_x;
            let y0 = grid.origin.y + iy as f64 * grid.cell_y;
            for &[a, b] in &grid.column(ix, iy).intervals {
                let mut bb = Bounds3::empty();
                for k in 0..8 {
                    let p = Point3::new(
                        if k & 1 == 0 { x0 - wx } else { x0 + grid.cell_x + wx },
                        if k & 2 == 0 { y0 - wy } else { y0 + grid.cell_y + wy },
                        if k & 4 == 0 { a } else { b },
                    );
                    bb.include(t.apply_point(p));
                }
                bounds = bounds.union(&bb);
                boxes.push(bb);
            }
        }
    }
    // The new grid spans the moved old grid footprint (empty stock keeps its extent).
    let old_extent = grid.bounds();
    let moved_extent = t.apply_bounds(&old_extent);
    let footprint = if bounds.is_empty() { moved_extent } else { bounds.union(&moved_extent) };
    let nx = (((footprint.max.x - footprint.min.x) / cx) - 1e-9).ceil().max(1.0);
    let ny = (((footprint.max.y - footprint.min.y) / cy) - 1e-9).ceil().max(1.0);
    if nx * ny > options.max_columns as f64 {
        let area = (footprint.max.x - footprint.min.x) * (footprint.max.y - footprint.min.y);
        return Err(StockSimError::BudgetExceeded {
            requested_cell: cx.max(cy),
            columns: (nx * ny) as u64,
            max_columns: options.max_columns,
            cell_that_fits: (area / options.max_columns as f64).sqrt() * 1.001,
        });
    }
    let (nx, ny) = (nx as u32, ny as u32);
    let origin = Point3::new(footprint.min.x, footprint.min.y, footprint.min.z);
    let mut columns = vec![Column::default(); (nx as usize) * (ny as usize)];
    // Rasterize: a box reaches a column when they overlap with positive area
    // (a shared edge is not overlap; the tolerance absorbs round-off).
    let eps_x = 1e-9 * cx;
    let eps_y = 1e-9 * cy;
    for bb in &boxes {
        let ix0 = (((bb.min.x - origin.x) / cx + 1e-9).floor()).max(0.0) as u32;
        let ix1 = ((((bb.max.x - origin.x) / cx - 1e-9).ceil()) as i64 - 1).clamp(0, nx as i64 - 1) as u32;
        let iy0 = (((bb.min.y - origin.y) / cy + 1e-9).floor()).max(0.0) as u32;
        let iy1 = ((((bb.max.y - origin.y) / cy - 1e-9).ceil()) as i64 - 1).clamp(0, ny as i64 - 1) as u32;
        for iy in iy0..=iy1.max(iy0) {
            for ix in ix0..=ix1.max(ix0) {
                if ix >= nx || iy >= ny {
                    continue;
                }
                let x0 = origin.x + ix as f64 * cx;
                let y0 = origin.y + iy as f64 * cy;
                if bb.max.x <= x0 + eps_x || bb.min.x >= x0 + cx - eps_x || bb.max.y <= y0 + eps_y || bb.min.y >= y0 + cy - eps_y {
                    continue;
                }
                columns[(iy * nx + ix) as usize].intervals.push([bb.min.z, bb.max.z]);
            }
        }
    }
    for c in &mut columns {
        merge(&mut c.intervals);
    }
    let new = DexelGrid { origin, cell_x: cx, cell_y: cy, nx, ny, columns, z_range: [footprint.min.z, footprint.max.z] };
    // Exact when grid axes map to grid axes, old Z stays vertical, and the
    // moved cells land on whole new cells.
    let aligned = match map {
        Some(m) => {
            m[2] == 2 && {
                let ox = (t.apply_point(Point3::new(grid.origin.x, grid.origin.y, 0.0)).x - origin.x) / cx;
                let oy = (t.apply_point(Point3::new(grid.origin.x, grid.origin.y, 0.0)).y - origin.y) / cy;
                (ox - ox.round()).abs() < 1e-9 && (oy - oy.round()).abs() < 1e-9
            }
        }
        None => false,
    };
    let mut limits = Vec::new();
    if !aligned {
        let old_diag = (grid.cell_x.powi(2) + grid.cell_y.powi(2)).sqrt();
        let new_diag = (cx * cx + cy * cy).sqrt();
        let keeps_z = t.apply_vector(Point3::Z).distance(Point3::Z) < 1e-12;
        // The boxes were widened by half a cell (half the old diagonal).
        let widened = if same_columns { 0.0 } else { 0.5 * old_diag };
        let bound = match map {
            // A quarter turn: half a cell along the widened axis, and the old
            // interval ends rounded out to whole cells of the new axis they
            // run along; the inherited axis lands on whole cells.
            Some(m) if !same_columns => 0.5 * [grid.cell_x, grid.cell_y][m[2]] + if m[0] == 2 { cx } else { cy },
            _ if keeps_z => widened + old_diag + new_diag,
            _ => {
                // A tilted box's bounding box can grow by its whole diagonal.
                let tallest = grid.columns.iter().flat_map(|c| c.intervals.iter()).map(|[a, b]| b - a).fold(0.0, f64::max);
                widened + (old_diag * old_diag + tallest * tallest).sqrt() + new_diag
            }
        };
        let detail = if matches!(map, Some(_)) && !same_columns {
            "the setup change turns old dexel columns sideways; the carried stock is a conservative superset (the old axis that becomes the new Z widened by half a cell, so a wall between column centres is never carried short)"
        } else {
            "the setup change does not map dexel columns onto columns; the carried stock is a conservative superset (each old column widened by half a cell, so a wall between column centres is never carried short)"
        };
        limits.push(ApproximationLimit { kind: "oblique-resample".into(), bound: Some(bound), detail: Some(detail.into()) });
    }
    Ok((new, aligned, limits))
}

/// Sort and merge overlapping or touching intervals.
fn merge(v: &mut Vec<[f64; 2]>) {
    v.sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap_or(std::cmp::Ordering::Equal));
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(v.len());
    for &[a, b] in v.iter() {
        match out.last_mut() {
            Some(last) if a <= last[1] => last[1] = last[1].max(b),
            _ => out.push([a, b]),
        }
    }
    *v = out;
}

