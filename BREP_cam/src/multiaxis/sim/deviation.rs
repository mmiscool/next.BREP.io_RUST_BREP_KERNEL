//! Deviation of multi-axis results (lane C10c, row 3) through C7's verify
//! module, not a fork of it: each tri-dexel grid is a Z-dexel in a frame
//! rotated by a cyclic axis permutation (a proper rotation), so
//! `verify::deviation_map` runs three times — on the Z grid as is, and on
//! the X and Y grids with the part mesh rotated into their frames — and
//! returns one map per viewing axis (+Z, +X, +Y faces of the stock).

use crate::index::BruteForceIndex;
use crate::model::{Point3, RigidTransform};
use crate::multiaxis::sim::tridexel::TriDexel;
use crate::stock_sim::DexelGrid;
use crate::verify::{deviation_map, excess_samples, DeviationMap, DeviationOptions, ExcessSample};
use crate::{CamError, MeshSnapshot};

/// The proper rotation that maps a grid's `(u, v, axis)` coordinates onto
/// `(x, y, z)`: identity for the Z grid, cyclic permutations otherwise.
pub fn axis_rotation(axis: usize) -> RigidTransform {
    let rotation = match axis {
        0 => [[0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]],
        1 => [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
        _ => [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
    };
    RigidTransform::new(rotation, Point3::ZERO).expect("a cyclic permutation is a proper rotation")
}

/// One grid of the tri-dexel as a Z-dexel in its rotated frame.
pub fn dexel_grid_of(td: &TriDexel, axis: usize) -> DexelGrid {
    let g = &td.grids[axis.min(2)];
    let o = axis_rotation(g.axis).apply_point(g.origin);
    DexelGrid {
        origin: o,
        cell_x: g.cell[0],
        cell_y: g.cell[1],
        nx: g.n[0],
        ny: g.n[1],
        columns: g.columns.clone(),
        z_range: g.range,
    }
}

/// Deviation maps of the stock against `part` seen along +Z, +X and +Y
/// (`[z, x, y]` order matches `TriDexel::grids` reversed: index 0 is the Z
/// map, 1 the X map, 2 the Y map). The X and Y maps are in their rotated
/// frames: `axis_rotation(axis)` maps part coordinates into them.
pub fn deviation_maps(td: &TriDexel, part: &MeshSnapshot, options: &DeviationOptions) -> Result<[DeviationMap; 3], CamError> {
    if part.is_empty() {
        return Err(CamError::InvalidInput("deviation: the part mesh is empty".into()));
    }
    let mut maps = Vec::with_capacity(3);
    for axis in [2usize, 0, 1] {
        let grid = dexel_grid_of(td, axis);
        let rotated = part.transformed(&axis_rotation(axis));
        let index = BruteForceIndex::build(&rotated);
        maps.push(deviation_map(&grid, &rotated, &index, options));
    }
    let y = maps.pop().unwrap();
    let x = maps.pop().unwrap();
    let z = maps.pop().unwrap();
    Ok([z, x, y])
}

/// 180° about the grid frame's y: the grid's columns seen from the other end of
/// its axis (their bottoms become tops).
fn flip_rotation() -> RigidTransform {
    RigidTransform::new([[-1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]], Point3::ZERO).expect("a half turn is a proper rotation")
}

/// `grid` in the frame [`flip_rotation`] maps its frame to: columns mirrored
/// in x and each interval negated, so a column's lowest material is its top.
fn flipped(grid: &DexelGrid) -> DexelGrid {
    let mut columns = Vec::with_capacity(grid.columns.len());
    for iy in 0..grid.ny {
        for ix in (0..grid.nx).rev() {
            let mut c = grid.column(ix, iy).clone();
            c.intervals = c.intervals.iter().rev().map(|[a, b]| [-b, -a]).collect();
            columns.push(c);
        }
    }
    DexelGrid {
        origin: Point3::new(-(grid.origin.x + grid.nx as f64 * grid.cell_x), grid.origin.y, -grid.z_range[1]),
        cell_x: grid.cell_x,
        cell_y: grid.cell_y,
        nx: grid.nx,
        ny: grid.ny,
        columns,
        z_range: [-grid.z_range[1], -grid.z_range[0]],
    }
}

/// One excess sample and the view (`+Z`, `-Z`, `+X`, `-X`, `+Y`, `-Y`) that
/// read it, with the point and nearest point in the part's frame.
#[derive(Clone, Debug, PartialEq)]
pub struct ViewSample {
    pub view: &'static str,
    pub sample: ExcessSample,
}

/// The excess samples of the stock against `part` read from all six sides:
/// each tri-dexel grid seen from both ends of its axis, so the material left
/// on faces turned to -X, -Y and -Z is listed too (the three maps of
/// [`deviation_maps`] see only the + ends). A face is read by every view that
/// sees it; callers that need one entry per place deduplicate by point.
pub fn excess_samples_all_views(td: &TriDexel, part: &MeshSnapshot, options: &DeviationOptions) -> Result<Vec<ViewSample>, CamError> {
    if part.is_empty() {
        return Err(CamError::InvalidInput("deviation: the part mesh is empty".into()));
    }
    let mut out = Vec::new();
    for (axis, views) in [(2usize, ["+Z", "-Z"]), (0, ["+X", "-X"]), (1, ["+Y", "-Y"])] {
        let grid = dexel_grid_of(td, axis);
        for (view, flip) in views.into_iter().zip([false, true]) {
            let (grid, frame) = if flip {
                (flipped(&grid), axis_rotation(axis).then(&flip_rotation()))
            } else {
                (grid.clone(), axis_rotation(axis))
            };
            let rotated = part.transformed(&frame);
            let index = BruteForceIndex::build(&rotated);
            let map = deviation_map(&grid, &rotated, &index, options);
            let back = frame.inverse();
            out.extend(excess_samples(&map, &rotated, &index, options).into_iter().map(|mut sample| {
                sample.point = back.apply_point(sample.point);
                sample.nearest = back.apply_point(sample.nearest);
                ViewSample { view, sample }
            }));
        }
    }
    Ok(out)
}
