//! Shared axis-aligned bounding boxes and a static bounding-volume
//! hierarchy over per-face boxes.
//!
//! Rational surfaces and curves with positive weights lie inside the convex
//! hull of their control points, so a control-point box is a conservative
//! bound for every point on the exact geometry. Queries expand boxes by the
//! caller's tolerance, which keeps BVH pruning conservative for the same
//! tolerance the narrow phase uses.

use crate::surface::NurbsSurface;
use crate::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Aabb {
    pub minimum: Vec3,
    pub maximum: Vec3,
}

fn axis(point: Vec3, index: usize) -> f64 {
    match index {
        0 => point.x,
        1 => point.y,
        _ => point.z,
    }
}

impl Aabb {
    pub fn empty() -> Self {
        Self {
            minimum: Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            maximum: Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    pub(crate) fn from_points(points: impl IntoIterator<Item = Vec3>) -> Self {
        let mut bounds = Self::empty();
        for point in points {
            bounds.include_point(point);
        }
        bounds
    }

    pub fn include_point(&mut self, point: Vec3) {
        self.minimum.x = self.minimum.x.min(point.x);
        self.minimum.y = self.minimum.y.min(point.y);
        self.minimum.z = self.minimum.z.min(point.z);
        self.maximum.x = self.maximum.x.max(point.x);
        self.maximum.y = self.maximum.y.max(point.y);
        self.maximum.z = self.maximum.z.max(point.z);
    }

    pub fn include(&mut self, other: Self) {
        self.include_point(other.minimum);
        self.include_point(other.maximum);
    }

    pub fn from_surface_controls(surface: &NurbsSurface) -> Result<Self, String> {
        let mut bounds = Self::empty();
        for control in surface.control_points.iter().flatten() {
            bounds.include_point(control.point()?);
        }
        Ok(bounds)
    }

    pub fn expanded(self, amount: f64) -> Self {
        let delta = Vec3::new(amount, amount, amount);
        Self {
            minimum: self.minimum.sub(delta),
            maximum: self.maximum.add(delta),
        }
    }

    pub fn contains(self, point: Vec3) -> bool {
        point.x >= self.minimum.x
            && point.x <= self.maximum.x
            && point.y >= self.minimum.y
            && point.y <= self.maximum.y
            && point.z >= self.minimum.z
            && point.z <= self.maximum.z
    }

    pub fn intersects(self, other: Self, tolerance: f64) -> bool {
        self.minimum.x - tolerance <= other.maximum.x
            && self.maximum.x + tolerance >= other.minimum.x
            && self.minimum.y - tolerance <= other.maximum.y
            && self.maximum.y + tolerance >= other.minimum.y
            && self.minimum.z - tolerance <= other.maximum.z
            && self.maximum.z + tolerance >= other.minimum.z
    }

    pub fn diagonal(self) -> f64 {
        self.maximum.sub(self.minimum).length()
    }

    fn center_along(self, index: usize) -> f64 {
        (axis(self.minimum, index) + axis(self.maximum, index)) / 2.0
    }

    fn intersects_segment(self, start: Vec3, delta: Vec3, tolerance: f64) -> bool {
        let mut enter = 0.0f64;
        let mut exit = 1.0f64;
        for index in 0..3 {
            let origin = axis(start, index);
            let direction = axis(delta, index);
            let minimum = axis(self.minimum, index) - tolerance;
            let maximum = axis(self.maximum, index) + tolerance;
            if direction.abs() < 1e-300 {
                if origin < minimum || origin > maximum {
                    return false;
                }
                continue;
            }
            let inverse = 1.0 / direction;
            let mut near = (minimum - origin) * inverse;
            let mut far = (maximum - origin) * inverse;
            if near > far {
                std::mem::swap(&mut near, &mut far);
            }
            enter = enter.max(near);
            exit = exit.min(far);
            if enter > exit {
                return false;
            }
        }
        true
    }
}

const LEAF_SIZE: usize = 4;

struct Node {
    bounds: Aabb,
    /// Leaf: start index into `order`. Internal: index of the left child
    /// node (the right child immediately follows the left subtree).
    left: u32,
    /// Leaf: end index (exclusive) into `order`. Internal: index of the
    /// right child node.
    right: u32,
    leaf: bool,
}

/// Static BVH over per-item boxes; queries return indices into the slice
/// the tree was built from.
pub struct Bvh {
    nodes: Vec<Node>,
    order: Vec<u32>,
    boxes: Vec<Aabb>,
}

impl Bvh {
    pub fn build(boxes: &[Aabb]) -> Self {
        let mut order: Vec<u32> = (0..boxes.len() as u32).collect();
        let mut nodes = Vec::new();
        if !boxes.is_empty() {
            let count = order.len();
            build_node(boxes, &mut order, 0, count, &mut nodes);
        }
        Self {
            nodes,
            order,
            boxes: boxes.to_vec(),
        }
    }

    /// Collect indices whose box overlaps `query` expanded by `tolerance`.
    pub fn overlapping(&self, query: Aabb, tolerance: f64, out: &mut Vec<usize>) {
        self.visit(|bounds| bounds.intersects(query, tolerance), out);
    }

    /// Collect indices whose box (expanded by `tolerance`) intersects the
    /// segment from `start` to `end`.
    pub fn intersecting_segment(
        &self,
        start: Vec3,
        end: Vec3,
        tolerance: f64,
        out: &mut Vec<usize>,
    ) {
        let delta = end.sub(start);
        self.visit(
            |bounds| bounds.intersects_segment(start, delta, tolerance),
            out,
        );
    }

    /// Collect indices whose box (expanded by `tolerance`) contains `point`.
    pub fn containing_point(&self, point: Vec3, tolerance: f64, out: &mut Vec<usize>) {
        self.visit(|bounds| bounds.expanded(tolerance).contains(point), out);
    }

    fn visit(&self, hit: impl Fn(Aabb) -> bool, out: &mut Vec<usize>) {
        if self.nodes.is_empty() {
            return;
        }
        let mut stack = vec![0usize];
        while let Some(index) = stack.pop() {
            let node = &self.nodes[index];
            if !hit(node.bounds) {
                continue;
            }
            if node.leaf {
                for &item in &self.order[node.left as usize..node.right as usize] {
                    if hit(self.boxes[item as usize]) {
                        out.push(item as usize);
                    }
                }
            } else {
                stack.push(node.left as usize);
                stack.push(node.right as usize);
            }
        }
    }
}

fn build_node(
    boxes: &[Aabb],
    order: &mut [u32],
    start: usize,
    end: usize,
    nodes: &mut Vec<Node>,
) -> usize {
    let mut bounds = Aabb::empty();
    for &item in &order[start..end] {
        bounds.include(boxes[item as usize]);
    }
    let index = nodes.len();
    if end - start <= LEAF_SIZE {
        nodes.push(Node {
            bounds,
            left: start as u32,
            right: end as u32,
            leaf: true,
        });
        return index;
    }
    let extent = bounds.maximum.sub(bounds.minimum);
    let split_axis = if extent.x >= extent.y && extent.x >= extent.z {
        0
    } else if extent.y >= extent.z {
        1
    } else {
        2
    };
    let middle = start + (end - start) / 2;
    order[start..end].select_nth_unstable_by(middle - start, |&a, &b| {
        boxes[a as usize]
            .center_along(split_axis)
            .total_cmp(&boxes[b as usize].center_along(split_axis))
    });
    nodes.push(Node {
        bounds,
        left: 0,
        right: 0,
        leaf: false,
    });
    let left = build_node(boxes, order, start, middle, nodes);
    let right = build_node(boxes, order, middle, end, nodes);
    nodes[index].left = left as u32;
    nodes[index].right = right as u32;
    index
}

