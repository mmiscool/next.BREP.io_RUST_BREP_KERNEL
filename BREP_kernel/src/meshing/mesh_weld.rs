//! Vertex welding and undirected edge keys for triangle meshes.

use crate::Vec3;
use rustc_hash::FxHashMap as HashMap;

fn quantize(point: Vec3, tolerance: f64) -> [i64; 3] {
    [
        (point.x / tolerance).round() as i64,
        (point.y / tolerance).round() as i64,
        (point.z / tolerance).round() as i64,
    ]
}

pub(crate) fn weld_vertex(
    point: Vec3,
    tolerance: f64,
    vertices: &mut Vec<Vec3>,
    buckets: &mut HashMap<[i64; 3], Vec<usize>>,
) -> usize {
    let key = quantize(point, tolerance);
    for dx in -1..=1 {
        for dy in -1..=1 {
            for dz in -1..=1 {
                if let Some(indices) = buckets.get(&[key[0] + dx, key[1] + dy, key[2] + dz]) {
                    if let Some(index) = indices
                        .iter()
                        .copied()
                        .find(|index| vertices[*index].sub(point).length() <= tolerance)
                    {
                        return index;
                    }
                }
            }
        }
    }
    let index = vertices.len();
    vertices.push(point);
    buckets.entry(key).or_default().push(index);
    index
}

pub(crate) fn edge_key(first: usize, second: usize) -> (usize, usize) {
    if first < second {
        (first, second)
    } else {
        (second, first)
    }
}
