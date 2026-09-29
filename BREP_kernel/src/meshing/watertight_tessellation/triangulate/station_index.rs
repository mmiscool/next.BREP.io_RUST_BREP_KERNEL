use super::*;

/// Bucket indexes for the interior seed's two per-station rejection tests.
///
/// [`seed_interior_grid`] offers `nu`×`nv` stations and rejects most of them:
/// first anything within `margin` of a boundary segment, then anything the
/// trim ring does not enclose. Both tests were written as a scan of the WHOLE
/// boundary, so the seed cost `stations × boundary` — on the
/// `onshape-unclamped-periodic` wall (1679 boundary vertices) the 2026-09-13
/// profile read 63.9 M segment-distance tests and 59.4 M ring-crossing tests
/// for 77 860 stations, 47.6 s of a 68.5 s tessellation.
///
/// Both answers are decided by geometry that is LOCAL to the query point:
///
/// * a segment can only be within `margin` of the point if it touches a cell
///   within one cell of the point's own, when cells are at least `margin`
///   across (they are, by construction);
/// * a ring edge can only contribute a crossing to a `+u` ray at `v` if its
///   own `v`-span contains `v`.
///
/// So both become bucket lookups. The tests each candidate then receives are
/// the ORIGINAL ones, unchanged and in the original order, and the buckets are
/// conservative — every segment or edge the linear scan would have accepted is
/// still offered. The boolean the seed reads is therefore identical, not
/// merely equivalent: this is a pure index, never a tolerance.
pub(in crate::watertight_tessellation) struct BoundaryIndex {
    /// Segment endpoints in the caller's order and the caller's RAW uv: the
    /// distance test below must be the same floating-point expression the scan
    /// evaluated, so only the bucketing works in scaled coordinates.
    segments: Vec<([f64; 2], [f64; 2])>,
    scale: [f64; 2],
    margin: f64,
    origin: [f64; 2],
    cell: [f64; 2],
    dims: [usize; 2],
    /// CSR buckets over `dims[0] * dims[1]` cells.
    starts: Vec<u32>,
    items: Vec<u32>,
    /// Segments whose cell footprint was too large to bucket; always tested.
    wide: Vec<u32>,
    /// The closed trim ring, raw uv, or empty when the caller decides
    /// containment by host-triangle search instead.
    ring: Vec<[f64; 2]>,
    ring_v0: f64,
    ring_cell: f64,
    ring_dims: usize,
    ring_starts: Vec<u32>,
    ring_items: Vec<u32>,
    ring_wide: Vec<u32>,
}

/// Cells one segment (or `v`-buckets one ring edge) may occupy before it is
/// moved to the always-tested overflow list. Bounds the index build at a small
/// multiple of the input regardless of how one long segment lies.
const MAX_FOOTPRINT: usize = 16;

/// Buckets per axis, before the `margin` floor raises the cell size. One
/// segment per cell on average; 256 caps a pathological boundary's memory.
const MAX_AXIS_BUCKETS: f64 = 256.0;

impl BoundaryIndex {
    pub(in crate::watertight_tessellation) fn new(
        segments: &[([f64; 2], [f64; 2])],
        ring: &[[f64; 2]],
        scale: [f64; 2],
        margin: f64,
    ) -> Self {
        let [su, sv] = scale;
        let scaled: Vec<([f64; 2], [f64; 2])> = segments
            .iter()
            .map(|&(a, b)| ([a[0] * su, a[1] * sv], [b[0] * su, b[1] * sv]))
            .collect();
        let raw = segments.to_vec();
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for (a, b) in &scaled {
            for point in [a, b] {
                for axis in 0..2 {
                    lo[axis] = lo[axis].min(point[axis]);
                    hi[axis] = hi[axis].max(point[axis]);
                }
            }
        }
        if !lo[0].is_finite() {
            lo = [0.0, 0.0];
            hi = [0.0, 0.0];
        }
        let target = (scaled.len() as f64).sqrt().ceil().clamp(1.0, MAX_AXIS_BUCKETS);
        let mut cell = [0.0f64; 2];
        let mut dims = [1usize; 2];
        for axis in 0..2 {
            let extent = (hi[axis] - lo[axis]).max(0.0);
            // At least `margin` across, so a query only ever needs the 3×3
            // neighbourhood of its own cell — and strictly positive, so the
            // floor division below is always defined.
            cell[axis] = (extent / target).max(margin).max(f64::MIN_POSITIVE);
            dims[axis] = (((extent / cell[axis]).floor() as usize) + 1).min(512);
        }
        let cell_of = |point: [f64; 2]| -> [usize; 2] {
            [
                (((point[0] - lo[0]) / cell[0]).floor().max(0.0) as usize).min(dims[0] - 1),
                (((point[1] - lo[1]) / cell[1]).floor().max(0.0) as usize).min(dims[1] - 1),
            ]
        };
        let footprint = |a: &[f64; 2], b: &[f64; 2]| -> ([usize; 2], [usize; 2]) {
            let low = cell_of([a[0].min(b[0]), a[1].min(b[1])]);
            let high = cell_of([a[0].max(b[0]), a[1].max(b[1])]);
            (low, high)
        };
        let cells = dims[0] * dims[1];
        let mut counts = vec![0u32; cells + 1];
        let mut wide = Vec::new();
        for (index, (a, b)) in scaled.iter().enumerate() {
            let (low, high) = footprint(a, b);
            let span = (high[0] - low[0] + 1) * (high[1] - low[1] + 1);
            if span > MAX_FOOTPRINT {
                wide.push(index as u32);
                continue;
            }
            for iy in low[1]..=high[1] {
                for ix in low[0]..=high[0] {
                    counts[iy * dims[0] + ix + 1] += 1;
                }
            }
        }
        for index in 1..counts.len() {
            counts[index] += counts[index - 1];
        }
        let starts = counts.clone();
        let mut cursor = counts;
        let mut items = vec![0u32; starts[cells] as usize];
        for (index, (a, b)) in scaled.iter().enumerate() {
            let (low, high) = footprint(a, b);
            let span = (high[0] - low[0] + 1) * (high[1] - low[1] + 1);
            if span > MAX_FOOTPRINT {
                continue;
            }
            for iy in low[1]..=high[1] {
                for ix in low[0]..=high[0] {
                    let cell_index = iy * dims[0] + ix;
                    items[cursor[cell_index] as usize] = index as u32;
                    cursor[cell_index] += 1;
                }
            }
        }

        let mut index = Self {
            segments: raw,
            scale,
            margin,
            origin: lo,
            cell,
            dims,
            starts,
            items,
            wide,
            ring: Vec::new(),
            ring_v0: 0.0,
            ring_cell: 1.0,
            ring_dims: 1,
            ring_starts: Vec::new(),
            ring_items: Vec::new(),
            ring_wide: Vec::new(),
        };
        if !ring.is_empty() {
            index.build_ring(ring);
        }
        index
    }

    /// `v`-buckets over the closed trim ring, so a crossing-parity query tests
    /// only the edges whose own `v`-span can contain the query level.
    fn build_ring(&mut self, ring: &[[f64; 2]]) {
        let count = ring.len();
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for point in ring {
            lo = lo.min(point[1]);
            hi = hi.max(point[1]);
        }
        let buckets = count.clamp(1, 4096);
        let extent = (hi - lo).max(0.0);
        let cell = (extent / buckets as f64).max(f64::MIN_POSITIVE);
        let dims = (((extent / cell).floor() as usize) + 1).min(4096);
        let bucket_of = |v: f64| -> usize {
            (((v - lo) / cell).floor().max(0.0) as usize).min(dims - 1)
        };
        let mut counts = vec![0u32; dims + 1];
        let mut wide = Vec::new();
        let span_of = |edge: usize| -> (usize, usize) {
            let a = ring[edge][1];
            let b = ring[(edge + 1) % count][1];
            (bucket_of(a.min(b)), bucket_of(a.max(b)))
        };
        for edge in 0..count {
            let (low, high) = span_of(edge);
            if high - low + 1 > MAX_FOOTPRINT {
                wide.push(edge as u32);
                continue;
            }
            for bucket in low..=high {
                counts[bucket + 1] += 1;
            }
        }
        for index in 1..counts.len() {
            counts[index] += counts[index - 1];
        }
        let starts = counts.clone();
        let mut cursor = counts;
        let mut items = vec![0u32; starts[dims] as usize];
        for edge in 0..count {
            let (low, high) = span_of(edge);
            if high - low + 1 > MAX_FOOTPRINT {
                continue;
            }
            for bucket in low..=high {
                items[cursor[bucket] as usize] = edge as u32;
                cursor[bucket] += 1;
            }
        }
        self.ring = ring.to_vec();
        self.ring_v0 = lo;
        self.ring_cell = cell;
        self.ring_dims = dims;
        self.ring_starts = starts;
        self.ring_items = items;
        self.ring_wide = wide;
    }

    /// Whether any boundary segment is closer than `margin` to `uv` — the
    /// crowding test [`seed_interior_grid`] rejects a station on.
    pub(in crate::watertight_tessellation) fn near_segment(&self, uv: [f64; 2]) -> bool {
        let [su, sv] = self.scale;
        let point = [uv[0] * su, uv[1] * sv];
        let centre = [
            (((point[0] - self.origin[0]) / self.cell[0]).floor().max(0.0) as usize)
                .min(self.dims[0] - 1),
            (((point[1] - self.origin[1]) / self.cell[1]).floor().max(0.0) as usize)
                .min(self.dims[1] - 1),
        ];
        let mut tested = 0u64;
        let mut hit = false;
        {
            let mut consider = |index: u32| {
                if hit {
                    return;
                }
                tested += 1;
                let (a, b) = self.segments[index as usize];
                // The original scaled point-to-segment distance, expression for
                // expression: `(uv - a) * s`, never `uv * s - a * s`.
                let ax = (uv[0] - a[0]) * su;
                let ay = (uv[1] - a[1]) * sv;
                let bx = (b[0] - a[0]) * su;
                let by = (b[1] - a[1]) * sv;
                let dot = ax * bx + ay * by;
                let len2 = (bx * bx + by * by).max(1e-30);
                let t = (dot / len2).clamp(0.0, 1.0);
                let dx = ax - bx * t;
                let dy = ay - by * t;
                if (dx * dx + dy * dy).sqrt() < self.margin {
                    hit = true;
                }
            };
            for iy in centre[1].saturating_sub(1)..=(centre[1] + 1).min(self.dims[1] - 1) {
                for ix in centre[0].saturating_sub(1)..=(centre[0] + 1).min(self.dims[0] - 1) {
                    let cell_index = iy * self.dims[0] + ix;
                    let from = self.starts[cell_index] as usize;
                    let to = self.starts[cell_index + 1] as usize;
                    for &item in &self.items[from..to] {
                        consider(item);
                    }
                }
            }
            for &item in &self.wide {
                consider(item);
            }
        }
        tess_profile(|p| p.margin_tests += tested);
        hit
    }

    /// Crossing parity of a `+u` ray at `uv` against the closed trim ring.
    pub(in crate::watertight_tessellation) fn ring_encloses(&self, uv: [f64; 2]) -> bool {
        let count = self.ring.len();
        let bucket = (((uv[1] - self.ring_v0) / self.ring_cell).floor().max(0.0) as usize)
            .min(self.ring_dims - 1);
        let mut crossings = 0usize;
        let mut tested = 0u64;
        let mut consider = |crossings: &mut usize, edge: usize| {
            tested += 1;
            let a = self.ring[edge];
            let b = self.ring[(edge + 1) % count];
            if (a[1] > uv[1]) != (b[1] > uv[1]) {
                let x = a[0] + (uv[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
                if x > uv[0] {
                    *crossings += 1;
                }
            }
        };
        let from = self.ring_starts[bucket] as usize;
        let to = self.ring_starts[bucket + 1] as usize;
        for &edge in &self.ring_items[from..to] {
            consider(&mut crossings, edge as usize);
        }
        for &edge in &self.ring_wide {
            consider(&mut crossings, edge as usize);
        }
        tess_profile(|p| p.parity_tests += tested);
        crossings % 2 == 1
    }
}

/// Uniform bucket grid over the PROTECTED boundary segments, answering "does
/// this candidate diagonal properly cross any of them" without scanning them
/// all.
///
/// [`lawson_flips_guarded`] asks that of every flip candidate that has already
/// passed the Delaunay and orientation gates, and asked it as
/// `boundary_pairs.iter().any(..)` — 2.55 billion segment-crossing tests on the
/// `onshape-unclamped-periodic` wall at chord 1e-3, the largest single term
/// left in the 2026-09-13 tessellation profile once the per-round edge maps
/// went.
///
/// A PROPER crossing (both cross products strictly opposite on both sides)
/// implies the two segments meet at a point interior to both, so their closed
/// uv bounding boxes overlap. Only segments whose box overlaps the candidate's
/// can therefore answer yes, and the query box is grown by a whole cell before
/// the lookup, so no rounding in the bucketing can lose one. Every candidate
/// the scan would have tested that could possibly answer yes is still tested,
/// with the identical predicate: the boolean is the same.
pub(in crate::watertight_tessellation) struct ProtectedSegments {
    pairs: Vec<(usize, usize)>,
    boxes: Vec<[f64; 4]>,
    origin: [f64; 2],
    cell: [f64; 2],
    dims: [usize; 2],
    starts: Vec<u32>,
    items: Vec<u32>,
    wide: Vec<u32>,
}

impl ProtectedSegments {
    pub(in crate::watertight_tessellation) fn new(
        pairs: impl Iterator<Item = (usize, usize)>,
        uv_of: impl Fn(usize) -> [f64; 2],
    ) -> Self {
        let pairs: Vec<(usize, usize)> = pairs.collect();
        let boxes: Vec<[f64; 4]> = pairs
            .iter()
            .map(|&(first, second)| {
                let a = uv_of(first);
                let b = uv_of(second);
                [
                    a[0].min(b[0]),
                    a[1].min(b[1]),
                    a[0].max(b[0]),
                    a[1].max(b[1]),
                ]
            })
            .collect();
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for bounds in &boxes {
            lo[0] = lo[0].min(bounds[0]);
            lo[1] = lo[1].min(bounds[1]);
            hi[0] = hi[0].max(bounds[2]);
            hi[1] = hi[1].max(bounds[3]);
        }
        if !lo[0].is_finite() {
            lo = [0.0, 0.0];
            hi = [0.0, 0.0];
        }
        let target = (boxes.len() as f64).sqrt().ceil().clamp(1.0, MAX_AXIS_BUCKETS);
        let mut cell = [0.0f64; 2];
        let mut dims = [1usize; 2];
        for axis in 0..2 {
            let extent = (hi[axis] - lo[axis]).max(0.0);
            cell[axis] = (extent / target).max(f64::MIN_POSITIVE);
            dims[axis] = (((extent / cell[axis]).floor() as usize) + 1).min(512);
        }
        let span = |bounds: &[f64; 4]| -> ([usize; 2], [usize; 2]) {
            let index = |value: f64, axis: usize| -> usize {
                (((value - lo[axis]) / cell[axis]).floor().max(0.0) as usize).min(dims[axis] - 1)
            };
            (
                [index(bounds[0], 0), index(bounds[1], 1)],
                [index(bounds[2], 0), index(bounds[3], 1)],
            )
        };
        let cells = dims[0] * dims[1];
        let mut counts = vec![0u32; cells + 1];
        let mut wide = Vec::new();
        for (index, bounds) in boxes.iter().enumerate() {
            let (low, high) = span(bounds);
            if (high[0] - low[0] + 1) * (high[1] - low[1] + 1) > MAX_FOOTPRINT {
                wide.push(index as u32);
                continue;
            }
            for iy in low[1]..=high[1] {
                for ix in low[0]..=high[0] {
                    counts[iy * dims[0] + ix + 1] += 1;
                }
            }
        }
        for index in 1..counts.len() {
            counts[index] += counts[index - 1];
        }
        let starts = counts.clone();
        let mut cursor = counts;
        let mut items = vec![0u32; starts[cells] as usize];
        for (index, bounds) in boxes.iter().enumerate() {
            let (low, high) = span(bounds);
            if (high[0] - low[0] + 1) * (high[1] - low[1] + 1) > MAX_FOOTPRINT {
                continue;
            }
            for iy in low[1]..=high[1] {
                for ix in low[0]..=high[0] {
                    let cell_index = iy * dims[0] + ix;
                    items[cursor[cell_index] as usize] = index as u32;
                    cursor[cell_index] += 1;
                }
            }
        }
        Self {
            pairs,
            boxes,
            origin: lo,
            cell,
            dims,
            starts,
            items,
            wide,
        }
    }

    pub(in crate::watertight_tessellation) fn is_empty(&self) -> bool {
        self.pairs.is_empty()
    }

    /// Whether the segment `c`–`d` properly crosses any protected segment that
    /// shares neither endpoint with it.
    pub(in crate::watertight_tessellation) fn crosses(
        &self,
        c: usize,
        d: usize,
        uv_of: impl Fn(usize) -> [f64; 2],
        epsilon: f64,
    ) -> bool {
        let (p, q) = (uv_of(c), uv_of(d));
        // Query box grown by a whole cell: a candidate lost to rounding in the
        // bucketing is a wrong answer, a candidate gained is only a test.
        let query = [
            p[0].min(q[0]) - self.cell[0],
            p[1].min(q[1]) - self.cell[1],
            p[0].max(q[0]) + self.cell[0],
            p[1].max(q[1]) + self.cell[1],
        ];
        let index = |value: f64, axis: usize| -> usize {
            (((value - self.origin[axis]) / self.cell[axis]).floor().max(0.0) as usize)
                .min(self.dims[axis] - 1)
        };
        let low = [index(query[0], 0), index(query[1], 1)];
        let high = [index(query[2], 0), index(query[3], 1)];
        let mut tested = 0u64;
        let mut hit = false;
        {
            let mut consider = |item: u32| {
                if hit {
                    return;
                }
                let bounds = &self.boxes[item as usize];
                if bounds[2] < query[0]
                    || bounds[0] > query[2]
                    || bounds[3] < query[1]
                    || bounds[1] > query[3]
                {
                    return;
                }
                let (first, second) = self.pairs[item as usize];
                if first == c || first == d || second == c || second == d {
                    return;
                }
                tested += 1;
                if segments_properly_cross(p, q, uv_of(first), uv_of(second), epsilon) {
                    hit = true;
                }
            };
            for iy in low[1]..=high[1] {
                for ix in low[0]..=high[0] {
                    let cell_index = iy * self.dims[0] + ix;
                    let from = self.starts[cell_index] as usize;
                    let to = self.starts[cell_index + 1] as usize;
                    for &item in &self.items[from..to] {
                        consider(item);
                    }
                }
            }
            for &item in &self.wide {
                consider(item);
            }
        }
        tess_profile(|p| p.boundary_cross_tests += tested);
        hit
    }
}
