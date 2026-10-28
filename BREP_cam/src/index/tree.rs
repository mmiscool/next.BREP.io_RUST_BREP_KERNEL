//! Build: each node owns a contiguous slice of `order`; a node with more than
//! `bucket_size` triangles is split on its longest axis at the median
//! centroid (ties broken by triangle id), so the same mesh always builds the
//! same tree. Query: descend only into nodes whose projected bounds overlap
//! the projected query, test triangle bounds at the leaves, and return the
//! ids ASCENDING, so a tree and the brute-force scan return identical lists.

use super::{IndexStats, ProjectionMode, TriangleIndex};
use crate::cancel::{Cancelled, Runner};
use crate::mesh::TriangleSource;
use crate::model::{Bounds3, Point3};
use serde::{Deserialize, Serialize};

/// Build parameters. Defaults follow the spec.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TreeOptions {
    /// Largest leaf.
    pub bucket_size: usize,
    /// Deepest node (root is depth 0).
    pub max_depth: usize,
    /// A node whose longest extent is below this is not split further.
    pub min_extent: f64,
}

impl Default for TreeOptions {
    fn default() -> Self {
        Self { bucket_size: 16, max_depth: 32, min_extent: 1e-6 }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct Node {
    bounds: Bounds3,
    /// Leaf: start into `order`. Internal: index of the left child.
    first: u32,
    /// Leaf: end (exclusive) into `order`. Internal: index of the right child.
    second: u32,
    leaf: bool,
}

/// Static median-split tree over one mesh's triangle bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct TreeIndex {
    nodes: Vec<Node>,
    order: Vec<u32>,
    bounds: Vec<Bounds3>,
    max_depth: usize,
    leaf_count: usize,
}

struct Item {
    id: u32,
    bounds: Bounds3,
    centroid: Point3,
}

impl TreeIndex {
    pub fn build(mesh: &dyn TriangleSource, options: TreeOptions) -> Self {
        let mut sink = crate::cancel::NullSink;
        let mut runner = Runner::detached(&mut sink);
        Self::build_with_runner(mesh, options, &mut runner).expect("a detached runner never cancels")
    }

    /// Build, publishing `index-build-start`, `index-build-partition` (at
    /// chunk boundaries counted in triangles partitioned) and
    /// `index-build-complete`, and stopping at a partition boundary on cancel.
    pub fn build_with_runner(
        mesh: &dyn TriangleSource,
        options: TreeOptions,
        runner: &mut Runner<'_>,
    ) -> Result<Self, Cancelled> {
        let n = mesh.triangle_count();
        runner.stage("index-build-start", "building triangle index", 0, n as u64)?;
        let mut items: Vec<Item> = (0..n)
            .map(|i| {
                let t = mesh.triangle(i as u32);
                Item { id: t.id, bounds: t.bounds, centroid: t.centroid() }
            })
            .collect();
        let bounds: Vec<Bounds3> = items.iter().map(|it| it.bounds).collect();
        let mut tree = Self { nodes: Vec::new(), order: Vec::with_capacity(n), bounds, max_depth: 0, leaf_count: 0 };
        if n > 0 {
            let bucket = options.bucket_size.max(1);
            tree.build_node(&mut items, 0, bucket, &options, runner)?;
        }
        runner.stage("index-build-complete", "triangle index built", n as u64, n as u64)?;
        Ok(tree)
    }

    fn build_node(
        &mut self,
        items: &mut [Item],
        depth: usize,
        bucket: usize,
        options: &TreeOptions,
        runner: &mut Runner<'_>,
    ) -> Result<u32, Cancelled> {
        let bounds = items.iter().fold(Bounds3::empty(), |b, it| b.union(&it.bounds));
        self.max_depth = self.max_depth.max(depth);
        let index = self.nodes.len() as u32;
        let axis = bounds.longest_axis();
        let extent = bounds.size().axis(axis);
        if items.len() <= bucket || depth >= options.max_depth || extent < options.min_extent {
            let first = self.order.len() as u32;
            // Leaves keep ascending ids so a leaf scan appends in order.
            items.sort_by_key(|it| it.id);
            self.order.extend(items.iter().map(|it| it.id));
            self.nodes.push(Node { bounds, first, second: self.order.len() as u32, leaf: true });
            self.leaf_count += 1;
            runner.work(items.len() as u64, "index-build-partition", self.order.len() as u64, self.bounds.len() as u64)?;
            return Ok(index);
        }
        // Median split on the longest axis; equal centroids order by id so the
        // partition never depends on input order.
        items.sort_by(|a, b| {
            a.centroid
                .axis(axis)
                .partial_cmp(&b.centroid.axis(axis))
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.id.cmp(&b.id))
        });
        let mid = items.len() / 2;
        self.nodes.push(Node { bounds, first: 0, second: 0, leaf: false });
        let (left_items, right_items) = items.split_at_mut(mid);
        let left = self.build_node(left_items, depth + 1, bucket, options, runner)?;
        let right = self.build_node(right_items, depth + 1, bucket, options, runner)?;
        self.nodes[index as usize].first = left;
        self.nodes[index as usize].second = right;
        Ok(index)
    }

    pub fn node_count(&self) -> usize {
        self.nodes.len()
    }
}

impl TriangleIndex for TreeIndex {
    fn query_aabb(&self, query: &Bounds3, mode: ProjectionMode, out: &mut Vec<u32>) {
        if self.nodes.is_empty() {
            return;
        }
        let start = out.len();
        let mut stack: Vec<u32> = vec![0];
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index as usize];
            if !mode.overlaps(&node.bounds, query) {
                continue;
            }
            if node.leaf {
                for &id in &self.order[node.first as usize..node.second as usize] {
                    if mode.overlaps(&self.bounds[id as usize], query) {
                        out.push(id);
                    }
                }
            } else {
                // Right first so the left child is visited first (stable order).
                stack.push(node.second);
                stack.push(node.first);
            }
        }
        out[start..].sort_unstable();
    }

    fn stats(&self) -> IndexStats {
        IndexStats {
            triangle_count: self.bounds.len(),
            node_count: Some(self.nodes.len()),
            leaf_count: Some(self.leaf_count),
            max_depth: Some(self.max_depth),
            implementation: "median-split-tree".to_string(),
        }
    }
}

/// Brute force for small meshes, the tree above the threshold.
#[derive(Clone, Debug, PartialEq)]
pub enum AnyIndex {
    Brute(super::BruteForceIndex),
    Tree(TreeIndex),
}

impl AnyIndex {
    /// Meshes with fewer triangles than this scan linearly.
    pub const TREE_THRESHOLD: usize = 256;

    pub fn build(mesh: &dyn TriangleSource, options: TreeOptions) -> Self {
        if mesh.triangle_count() < Self::TREE_THRESHOLD {
            AnyIndex::Brute(super::BruteForceIndex::build(mesh))
        } else {
            AnyIndex::Tree(TreeIndex::build(mesh, options))
        }
    }
}

impl TriangleIndex for AnyIndex {
    fn query_aabb(&self, query: &Bounds3, mode: ProjectionMode, out: &mut Vec<u32>) {
        match self {
            AnyIndex::Brute(i) => i.query_aabb(query, mode, out),
            AnyIndex::Tree(i) => i.query_aabb(query, mode, out),
        }
    }

    fn stats(&self) -> IndexStats {
        match self {
            AnyIndex::Brute(i) => i.stats(),
            AnyIndex::Tree(i) => i.stats(),
        }
    }
}
