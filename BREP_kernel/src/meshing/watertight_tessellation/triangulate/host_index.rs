//! Candidate index for the seed's unchanged, reverse-order barycentric test.
//!
//! A strict host has three positive computed subareas. For coordinates bounded
//! by M, E = 64*EPSILON*M² bounds the absolute rounding error of triangle_area
//! (subtractions, two products, subtraction and halving), with ample slack.
//! We index only A > 256*E, so the exact area is positive. Each exact
//! barycentric coordinate is then >= -E/(A-E). At most two can be negative;
//! an accepted point is outside the vertex box by at most
//! 2*E/(A-E)*axis_range. The padding below is larger, including box rounding.
//! Ill-conditioned triangles, extreme coordinates and out-of-domain queries
//! use the original scan. The index never supplies a geometric verdict.
use super::{triangle_area, FaceVertex};

const MAX_MEMBERS: usize = 4_000_000;
const MAX_ENTRIES: usize = 1 << 18;
#[derive(Default)]
struct Entry {
    bounds: Option<[f64; 4]>,
    cover: Option<[usize; 4]>,
    in_overflow: bool,
}
pub(super) struct HostIndex {
    domain: [f64; 4],
    magnitude: f64,
    error: f64,
    side: usize,
    buckets: Vec<Vec<usize>>,
    entries: Vec<Entry>,
    overflow: Vec<usize>,
    members: usize,
}
impl HostIndex {
    pub(super) fn new(vertices: &[FaceVertex], triangles: &[[usize; 3]], domain: [f64; 4], stations: usize) -> Self {
        let magnitude = vertices.iter().flat_map(|v| v.uv).chain(domain).map(f64::abs).fold(0.0, f64::max);
        let finite = domain.iter().all(|x| x.is_finite()) && vertices.iter().all(|v| v.uv.iter().all(|x| x.is_finite()));
        let enabled = stations >= 64 && finite && domain[1] > domain[0] && domain[3] > domain[2]
            && (1e-12..=1e100).contains(&magnitude);
        let mut result = Self { domain, magnitude, error: 64.0 * f64::EPSILON * magnitude * magnitude,
            side: if enabled { ((stations as f64 / 4.0).sqrt() as usize).clamp(4, 32) } else { 0 },
            buckets: Vec::new(), entries: Vec::new(), overflow: Vec::new(), members: 0 };
        result.buckets.resize_with(result.side * result.side, Vec::new);
        if result.side != 0 {
            for (id, &triangle) in triangles.iter().enumerate() { result.update(id, triangle, vertices); }
        }
        result
    }
    fn cell(&self, x: f64, axis: usize) -> usize {
        let f = (x - self.domain[axis]) / (self.domain[axis + 1] - self.domain[axis]);
        ((f * self.side as f64).floor().max(0.0) as usize).min(self.side - 1)
    }
    fn bounds(&self, triangle: [usize; 3], vertices: &[FaceVertex]) -> Option<[f64; 4]> {
        let p = triangle.map(|i| vertices[i].uv);
        if p.iter().flatten().any(|x| !x.is_finite() || x.abs() > self.magnitude) { return None; }
        let area = triangle_area(p[0], p[1], p[2]);
        if !(area > 256.0 * self.error) { return None; }
        let mut b = [0.0; 4];
        for axis in 0..2 {
            let lo = p.iter().map(|x| x[axis]).fold(f64::INFINITY, f64::min);
            let hi = p.iter().map(|x| x[axis]).fold(f64::NEG_INFINITY, f64::max);
            let padding = (4.0 * self.error / area) * (hi - lo) + 64.0 * f64::EPSILON * self.magnitude;
            b[axis * 2] = lo - padding;
            b[axis * 2 + 1] = hi + padding;
        }
        Some(b)
    }
    /// Triangles only split during seeding. Retain old bucket memberships and
    /// filter with the current box; expand coverage if rounding padding grows.
    pub(super) fn update(&mut self, id: usize, triangle: [usize; 3], vertices: &[FaceVertex]) {
        if self.side == 0 { return; }
        if id >= MAX_ENTRIES {
            self.side = 0; self.buckets.clear(); self.entries.clear(); self.overflow.clear();
            return;
        }
        while self.entries.len() <= id { self.entries.push(Entry::default()); }
        let bounds = self.bounds(triangle, vertices);
        self.entries[id].bounds = bounds;
        let Some(b) = bounds else {
            if !self.entries[id].in_overflow {
                self.entries[id].in_overflow = true;
                self.overflow.push(id);
            }
            return;
        };
        let mut cover = [self.cell(b[0], 0), self.cell(b[1], 0), self.cell(b[2], 2), self.cell(b[3], 2)];
        let old = self.entries[id].cover;
        if let Some(o) = old {
            cover = [cover[0].min(o[0]), cover[1].max(o[1]), cover[2].min(o[2]), cover[3].max(o[3])];
        }
        for y in cover[2]..=cover[3] {
            for x in cover[0]..=cover[1] {
                if old.is_some_and(|o| x >= o[0] && x <= o[1] && y >= o[2] && y <= o[3]) { continue; }
                if self.members >= MAX_MEMBERS {
                    self.side = 0;
                    self.buckets.clear(); self.entries.clear(); self.overflow.clear();
                    return;
                }
                self.buckets[y * self.side + x].push(id);
                self.members += 1;
            }
        }
        self.entries[id].cover = Some(cover);
    }
    /// Sorted ids: the caller reverses them to preserve the original first
    /// accepted host, even for overlapping or nearly degenerate triangles.
    /// False requests the complete original scan, never an empty candidate set.
    pub(super) fn candidates(&self, uv: [f64; 2], out: &mut Vec<usize>) -> bool {
        out.clear();
        if self.side == 0 || uv.iter().any(|x| !x.is_finite() || x.abs() > self.magnitude)
            || uv[0] < self.domain[0] || uv[0] > self.domain[1] || uv[1] < self.domain[2] || uv[1] > self.domain[3] { return false; }
        let cell = self.cell(uv[1], 2) * self.side + self.cell(uv[0], 0);
        for &id in self.buckets[cell].iter().chain(&self.overflow) {
            if self.entries[id].bounds.is_none_or(|b| uv[0] >= b[0] && uv[0] <= b[1] && uv[1] >= b[2] && uv[1] <= b[3]) { out.push(id); }
        }
        out.sort_unstable(); out.dedup();
        true
    }
}

