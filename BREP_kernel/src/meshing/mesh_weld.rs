//! Vertex welding and undirected edge keys for triangle meshes.
//!
//! The one tolerance weld (record item G3, 2026-11-01): `floor` cells of
//! twice the tolerance, the 27-cell neighbourhood, a match at
//! `distance <= tolerance`, and the LOWEST-index matching vertex as the
//! representative, so the result does not depend on bucket or scan order.

use crate::Vec3;
use rustc_hash::FxHashMap as HashMap;

/// Bits of `value` with negative zero read as positive zero, for exact
/// (bitwise) point keys: `-0.0` and `0.0` are the same coordinate.
pub fn canonical_bits(value: f64) -> u64 {
    if value == 0.0 {
        0
    } else {
        value.to_bits()
    }
}

/// The weld cell of `point`: `floor(p / (2 tol))`. At a cell of `tol` a pair
/// exactly `tol` apart can land two cells apart once the division rounds
/// (and `round` puts ±0.5 two cells apart outright), outside the
/// neighbourhood; at `2 tol` every match is at most one cell away (the
/// sew.rs `EndpointGrid` reasoning).
fn cell(point: Vec3, tolerance: f64) -> [i64; 3] {
    let size = 2.0 * tolerance;
    [
        (point.x / size).floor() as i64,
        (point.y / size).floor() as i64,
        (point.z / size).floor() as i64,
    ]
}

/// The index of the lowest-index vertex within `tolerance` of `point`, or
/// `point` appended as a new vertex. `buckets` maps a cell to its vertices
/// in insertion (= index) order.
pub(crate) fn weld_vertex(
    point: Vec3,
    tolerance: f64,
    vertices: &mut Vec<Vec3>,
    buckets: &mut HashMap<[i64; 3], Vec<usize>>,
) -> usize {
    let key = cell(point, tolerance);
    let mut matched: Option<usize> = None;
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if let Some(indices) = buckets.get(&[key[0] + dx, key[1] + dy, key[2] + dz]) {
                    // Ascending within a bucket: the first match is its lowest.
                    if let Some(index) = indices
                        .iter()
                        .copied()
                        .find(|index| vertices[*index].sub(point).length() <= tolerance)
                    {
                        matched = Some(matched.map_or(index, |current| current.min(index)));
                    }
                }
            }
        }
    }
    if let Some(index) = matched {
        return index;
    }
    let index = vertices.len();
    vertices.push(point);
    buckets.entry(key).or_default().push(index);
    index
}

/// A tolerance weld over points given one at a time (`weld_vertex` with
/// its own vertex list and cells), for callers outside the kernel.
#[derive(Clone, Debug, Default)]
pub struct VertexWeld {
    tolerance: f64,
    vertices: Vec<Vec3>,
    buckets: HashMap<[i64; 3], Vec<usize>>,
}

impl VertexWeld {
    /// An empty weld that merges points within `tolerance` (> 0).
    pub fn new(tolerance: f64) -> Self {
        Self {
            tolerance,
            ..Self::default()
        }
    }

    /// The index of the lowest-index welded vertex within the tolerance of
    /// `point`, or the index `point` is appended at.
    pub fn weld(&mut self, point: [f64; 3]) -> usize {
        weld_vertex(
            Vec3::new(point[0], point[1], point[2]),
            self.tolerance,
            &mut self.vertices,
            &mut self.buckets,
        )
    }

    /// The welded vertex at `index` (the first point that created it).
    pub fn vertex(&self, index: usize) -> [f64; 3] {
        let v = self.vertices[index];
        [v.x, v.y, v.z]
    }

    /// The number of welded vertices.
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Whether no point has been welded yet.
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }
}

pub(crate) fn edge_key(first: usize, second: usize) -> (usize, usize) {
    if first < second {
        (first, second)
    } else {
        (second, first)
    }
}

