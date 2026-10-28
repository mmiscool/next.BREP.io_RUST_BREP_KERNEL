//! Broad-phase triangle queries.
//!
//! Drop-cutter and push-cutter ask the same question many times: which
//! triangles can a cutter at this XY (or this fiber) touch? A
//! [`TriangleIndex`] answers with a candidate list that never misses a true
//! overlap (false positives are allowed; the exact cutter/triangle test is the
//! caller's). Results are ascending triangle ids, so every implementation
//! returns the same list for the same query and output is deterministic.

use crate::mesh::{MeshSnapshot, TriangleSource};
use crate::model::Bounds3;
use serde::{Deserialize, Serialize};

mod tree;

pub use tree::{AnyIndex, TreeIndex, TreeOptions};

/// Which axes a query compares. A projected mode ignores the dropped axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ProjectionMode {
    /// Compare X and Y only (the drop-cutter's view down the tool axis).
    #[default]
    Xy,
    /// Compare X and Z only (push-cutter fibers along Y).
    Xz,
    /// Compare Y and Z only (push-cutter fibers along X).
    Yz,
    /// Compare all three axes.
    Xyz,
}

impl ProjectionMode {
    /// Closed-interval overlap of two boxes on the compared axes.
    pub fn overlaps(self, a: &Bounds3, b: &Bounds3) -> bool {
        let x = a.min.x <= b.max.x && b.min.x <= a.max.x;
        let y = a.min.y <= b.max.y && b.min.y <= a.max.y;
        let z = a.min.z <= b.max.z && b.min.z <= a.max.z;
        match self {
            ProjectionMode::Xy => x && y,
            ProjectionMode::Xz => x && z,
            ProjectionMode::Yz => y && z,
            ProjectionMode::Xyz => x && y && z,
        }
    }
}

/// Size and shape of an index, for reports and tests.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct IndexStats {
    pub triangle_count: usize,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub node_count: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub leaf_count: Option<usize>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub max_depth: Option<usize>,
    /// Implementation name: `brute-force`, `median-split-tree`, ...
    pub implementation: String,
}

/// Broad-phase candidate queries over one mesh snapshot.
/// `Sync`: an index is shared read-only by parallel fiber and span workers
/// (feature `parallel`); every implementor is plain data.
pub trait TriangleIndex: Sync {
    /// Append every triangle id whose bounds overlap `query` on the axes `mode`
    /// compares. Touching counts. Ids are appended in ascending order and
    /// `out` is NOT cleared first.
    fn query_aabb(&self, query: &Bounds3, mode: ProjectionMode, out: &mut Vec<u32>);

    fn stats(&self) -> IndexStats;

    /// Convenience: a fresh ascending candidate list.
    fn query(&self, query: &Bounds3, mode: ProjectionMode) -> Vec<u32> {
        let mut out = Vec::new();
        self.query_aabb(query, mode, &mut out);
        out
    }

    /// Candidates for a cutter of `radius` at `(x, y)`: the XY disc's box,
    /// unbounded in Z (the drop-cutter's query).
    fn query_cutter_xy(&self, x: f64, y: f64, radius: f64, out: &mut Vec<u32>) {
        let query = Bounds3::new(
            crate::model::Point3::new(x - radius, y - radius, f64::NEG_INFINITY),
            crate::model::Point3::new(x + radius, y + radius, f64::INFINITY),
        );
        self.query_aabb(&query, ProjectionMode::Xy, out);
    }
}

/// Scans every triangle. The reference every other index is checked against,
/// and the right choice for small meshes.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BruteForceIndex {
    bounds: Vec<Bounds3>,
}

impl BruteForceIndex {
    pub fn build(mesh: &dyn TriangleSource) -> Self {
        let bounds = (0..mesh.triangle_count()).map(|i| mesh.triangle(i as u32).bounds).collect();
        Self { bounds }
    }

    pub fn from_snapshot(mesh: &MeshSnapshot) -> Self {
        Self::build(mesh)
    }
}

impl TriangleIndex for BruteForceIndex {
    fn query_aabb(&self, query: &Bounds3, mode: ProjectionMode, out: &mut Vec<u32>) {
        for (i, b) in self.bounds.iter().enumerate() {
            if mode.overlaps(b, query) {
                out.push(i as u32);
            }
        }
    }

    fn stats(&self) -> IndexStats {
        IndexStats {
            triangle_count: self.bounds.len(),
            node_count: None,
            leaf_count: None,
            max_depth: None,
            implementation: "brute-force".to_string(),
        }
    }
}

