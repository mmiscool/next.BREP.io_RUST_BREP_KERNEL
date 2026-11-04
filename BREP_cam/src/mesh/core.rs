//! Shared operations on indexed triangle positions, without owning mesh metadata.
use crate::model::{Bounds3, Point3};

pub(crate) struct MeshCore<'a> {
    pub positions: &'a [Point3],
    pub indices: &'a [u32],
}

impl MeshCore<'_> {
    pub fn triangle_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn triangle(&self, i: usize) -> [Point3; 3] {
        let t = &self.indices[3 * i..3 * i + 3];
        [self.positions[t[0] as usize], self.positions[t[1] as usize], self.positions[t[2] as usize]]
    }

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

    pub fn is_closed(&self) -> bool {
        let mut keyed: Vec<([u64; 3], u32)> = self
            .positions
            .iter()
            .enumerate()
            .map(|(i, p)| ([crate::geom::canonical_bits(p.x), crate::geom::canonical_bits(p.y), crate::geom::canonical_bits(p.z)], i as u32))
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
}
