use crate::spatial::{Aabb, Bvh};
use crate::Vec3;
use rustc_hash::FxHashMap;
use serde::{Deserialize, Serialize};

const PARALLEL_THRESHOLD: f64 = 1e-12;

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct Vec2 {
    pub x: f64,
    pub y: f64,
}

impl Vec2 {
    /// Distance to a segment. `guard` is the caller's squared-length threshold.
    #[inline]
    pub fn distance_to_segment(self, start: Self, end: Self, guard: f64) -> f64 {
        let segment = end.sub(start);
        let length_squared = segment.dot(segment);
        if length_squared <= guard {
            return self.sub(start).length();
        }
        let parameter = (self.sub(start).dot(segment) / length_squared).clamp(0.0, 1.0);
        self.sub(start.add(segment.scale(parameter))).length()
    }

    /// Squared distance, retaining the seed sweep's projection on zero-length segments.
    #[inline]
    pub fn distance_to_segment_squared(self, start: Self, end: Self) -> f64 {
        let segment = end.sub(start);
        let length_squared = segment.dot(segment);
        let parameter = if length_squared > 0.0 {
            (self.sub(start).dot(segment) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let residual = self.sub(start.add(segment.scale(parameter)));
        residual.dot(residual)
    }

    pub(crate) fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
        }
    }
    pub(crate) fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
        }
    }
    pub(crate) fn scale(self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
        }
    }
    fn distance(self, other: Self) -> f64 {
        self.sub(other).length()
    }
    pub(crate) fn length(self) -> f64 {
        (self.x * self.x + self.y * self.y).sqrt()
    }
    fn cross(self, other: Self) -> f64 {
        self.x * other.y - self.y * other.x
    }
    pub(crate) fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Segment2 {
    pub a: Vec2,
    pub b: Vec2,
    #[serde(default)]
    pub tag: serde_json::Value,
}

#[derive(Clone, Debug, Serialize)]
pub struct ArrangementPiece {
    pub a: Vec2,
    pub b: Vec2,
    pub parent: Segment2,
}

#[derive(Clone, Debug, Serialize)]
pub struct CycleUse {
    pub piece: ArrangementPiece,
    pub forward: bool,
}

#[derive(Clone, Debug, Serialize)]
pub struct ArrangementRegion {
    pub outer: Vec<CycleUse>,
    pub holes: Vec<Vec<CycleUse>>,
    pub area: f64,
    /// The outer cycle's area alone, the key regions are returned by
    /// (ascending); `area` nets the holes out.
    pub outer_area: f64,
}

#[derive(Clone)]
struct Node {
    point: Vec2,
    outgoing: Vec<usize>,
}

/// Append-only node lookup. A match always selects the lowest existing node
/// index, exactly like the original linear `.position` scan, not the nearest
/// point or the first occupied neighboring cell.
struct NodeLookup {
    tolerance: f64,
    width: f64,
    cells: Option<FxHashMap<[i64; 2], Vec<usize>>>,
}

impl NodeLookup {
    fn new(tolerance: f64, indexed: bool) -> Self {
        let width = tolerance * 2.0;
        Self {
            tolerance,
            width,
            // Squared distances can underflow and accept farther points at
            // tiny tolerances. Preserve that legacy behavior with a scan.
            cells: (indexed && tolerance >= 1e-150 && width.is_finite()).then(FxHashMap::default),
        }
    }

    fn key(&self, point: Vec2) -> Option<[i64; 2]> {
        let q = [point.x / self.width, point.y / self.width];
        // Two-radius cells leave half a cell of rounding headroom. Bound the
        // quotient so division rounding cannot consume it, or integer neighbors
        // overflow. This also rejects non-finite coordinates.
        q.iter()
            .all(|v| v.is_finite() && v.abs() <= (1u64 << 48) as f64)
            .then(|| q.map(|v| v.floor() as i64))
    }

    fn find_or_insert(&mut self, point: Vec2, nodes: &mut Vec<Node>) -> usize {
        let key = self.cells.as_ref().and_then(|_| self.key(point));
        if key.is_none() {
            // Once a point cannot be indexed, retain the scan for the rest of
            // the stream, including comparisons against this newly added node.
            self.cells = None;
        }
        let found = if let (Some(cells), Some([x, y])) = (&self.cells, key) {
            let mut first = None;
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(indices) = cells.get(&[x + dx, y + dy]) {
                        // Each bucket is in insertion order. Later nodes in
                        // this bucket cannot beat an already earlier match.
                        for &index in indices {
                            if first.is_some_and(|first| index >= first) {
                                break;
                            }
                            if nodes[index].point.distance(point) <= self.tolerance {
                                first = Some(index);
                                break;
                            }
                        }
                    }
                }
            }
            first
        } else {
            nodes
                .iter()
                .position(|node| node.point.distance(point) <= self.tolerance)
        };
        if let Some(index) = found {
            return index;
        }
        let index = nodes.len();
        nodes.push(Node {
            point,
            outgoing: Vec::new(),
        });
        if let (Some(cells), Some(key)) = (&mut self.cells, key) {
            cells.entry(key).or_default().push(index);
        }
        index
    }
}

fn tolerance_parameter(segment: &Segment2, tolerance: f64) -> f64 {
    let length = segment.a.distance(segment.b);
    if length <= tolerance {
        0.5
    } else {
        tolerance / length
    }
}

fn interpolate(a: Vec2, b: Vec2, parameter: f64) -> Vec2 {
    Vec2 {
        x: a.x + (b.x - a.x) * parameter,
        y: a.y + (b.y - a.y) * parameter,
    }
}

pub fn segment_intersection(
    a1: Vec2,
    b1: Vec2,
    a2: Vec2,
    b2: Vec2,
    tolerance: f64,
) -> Option<[f64; 2]> {
    let d1 = b1.sub(a1);
    let d2 = b2.sub(a2);
    let denominator = d1.cross(d2);
    let length1 = d1.length();
    let length2 = d2.length();
    if denominator.abs() <= PARALLEL_THRESHOLD * length1 * length2 {
        return None;
    }
    let offset = a2.sub(a1);
    let t1 = offset.cross(d2) / denominator;
    let t2 = offset.cross(d1) / denominator;
    let epsilon1 = tolerance / length1.max(tolerance);
    let epsilon2 = tolerance / length2.max(tolerance);
    if t1 < -epsilon1 || t1 > 1.0 + epsilon1 || t2 < -epsilon2 || t2 > 1.0 + epsilon2 {
        None
    } else {
        Some([t1.clamp(0.0, 1.0), t2.clamp(0.0, 1.0)])
    }
}

pub use crate::polygon::PolygonClass;

fn point_segment_distance(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    point.distance_to_segment(a, b, 1e-300)
}

/// The hole assignment's box rejection (2026-11-02): a point
/// beyond a region polygon's vertex box by more than the classifier's
/// tolerance plus a rounding pad is classified Outside without the O(n) test,
/// and the full classifier would say the same. Its boundary band cannot fire
/// (every computed segment distance exceeds the tolerance: the clamped foot
/// lies in the box up to rounding the pad covers). Its form-K parity reads
/// Outside: above or below the box no edge straddles the ray; to the right no
/// crossing lies right of the point; to the left every straddling edge crosses
/// right of it, and a closed loop has an even number of them. The hole loop
/// was 60-70% of the arrangement's time on the package models (one negative
/// cycle tested node by node against every positive region's polygon).
/// Returns `[min_x, min_y, max_x, max_y, reach]`; `reach` is NaN (never
/// reject) for an empty or non-finite polygon.
fn polygon_reject_box(polygon: &[Vec2], tolerance: f64) -> [f64; 5] {
    let mut bounds = [f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY];
    let mut magnitude = 1.0f64;
    for point in polygon {
        bounds[0] = bounds[0].min(point.x);
        bounds[1] = bounds[1].min(point.y);
        bounds[2] = bounds[2].max(point.x);
        bounds[3] = bounds[3].max(point.y);
        magnitude = magnitude.max(point.x.abs()).max(point.y.abs());
    }
    let finite = !polygon.is_empty() && bounds.iter().all(|value| value.is_finite()) && tolerance.is_finite() && tolerance >= 0.0;
    let reach = if finite { tolerance * (1.0 + 1e-9) + 64.0 * f64::EPSILON * magnitude } else { f64::NAN };
    [bounds[0], bounds[1], bounds[2], bounds[3], reach]
}

/// The classifier's boundary band, indexed (2026-11-02): a
/// uniform grid over the polygon's vertex box whose cells list every edge whose
/// box, widened by the tolerance plus a rounding pad, overlaps them. An edge
/// whose computed distance to a point is within the tolerance has that point
/// inside its widened box (the clamped foot lies in the edge's box up to
/// rounding the pad covers), so the point's cell lists it; the band is an
/// existence test, so testing the listed edges with the classifier's own
/// distance gives the classifier's own answer. Parity is the original full
/// scan. Why: a negative cycle that runs along a region's own boundary
/// classifies Boundary at every node and goes on to the next, so the hole loop
/// paid one full O(n) band scan per node.
struct BoundaryBand {
    origin: [f64; 2],
    cell: [f64; 2],
    side: usize,
    reach: f64,
    starts: Vec<u32>,
    items: Vec<u32>,
    usable: bool,
}

impl BoundaryBand {
    fn new(polygon: &[Vec2], tolerance: f64) -> Self {
        let reject = polygon_reject_box(polygon, tolerance);
        let reach = reject[4];
        let count = polygon.len();
        let usable = reach.is_finite() && count >= 16 && count <= (1 << 24);
        let side = if usable { ((count as f64).sqrt() as usize).clamp(4, 512) } else { 0 };
        let origin = [reject[0] - reach, reject[1] - reach];
        let extent = [(reject[2] + reach) - origin[0], (reject[3] + reach) - origin[1]];
        let cell = [(extent[0] / side.max(1) as f64).max(f64::MIN_POSITIVE), (extent[1] / side.max(1) as f64).max(f64::MIN_POSITIVE)];
        let mut band = Self { origin, cell, side, reach, starts: Vec::new(), items: Vec::new(), usable };
        if !band.usable {
            return band;
        }
        let mut counts = vec![0u32; side * side + 1];
        let ranges: Vec<[usize; 4]> = (0..count)
            .map(|index| {
                let (a, b) = (polygon[index], polygon[(index + 1) % count]);
                let pad = reach + 64.0 * f64::EPSILON * a.x.abs().max(a.y.abs()).max(b.x.abs()).max(b.y.abs());
                [band.column(a.x.min(b.x) - pad), band.column(a.x.max(b.x) + pad), band.row(a.y.min(b.y) - pad), band.row(a.y.max(b.y) + pad)]
            })
            .collect();
        for range in &ranges {
            for y in range[2]..=range[3] {
                for x in range[0]..=range[1] {
                    counts[y * side + x + 1] += 1;
                }
            }
        }
        for index in 1..counts.len() {
            counts[index] += counts[index - 1];
        }
        // Long edges cover many cells; past a budget the original scan is cheaper.
        if counts[side * side] as usize > 64 * count + 4096 {
            band.usable = false;
            return band;
        }
        let mut cursor = counts.clone();
        band.items = vec![0u32; counts[side * side] as usize];
        for (edge, range) in ranges.iter().enumerate() {
            for y in range[2]..=range[3] {
                for x in range[0]..=range[1] {
                    let slot = &mut cursor[y * side + x];
                    band.items[*slot as usize] = edge as u32;
                    *slot += 1;
                }
            }
        }
        band.starts = counts;
        band
    }
    fn column(&self, x: f64) -> usize {
        (((x - self.origin[0]) / self.cell[0]).floor().max(0.0) as usize).min(self.side - 1)
    }
    fn row(&self, y: f64) -> usize {
        (((y - self.origin[1]) / self.cell[1]).floor().max(0.0) as usize).min(self.side - 1)
    }
    fn classify(&self, point: Vec2, polygon: &[Vec2], tolerance: f64) -> PolygonClass {
        if !self.usable || !point.x.is_finite() || !point.y.is_finite() {
            return point_in_polygon_class(point, polygon, tolerance);
        }
        let cell = self.row(point.y) * self.side + self.column(point.x);
        let count = polygon.len();
        for &edge in &self.items[self.starts[cell] as usize..self.starts[cell + 1] as usize] {
            let edge = edge as usize;
            if point.distance_to_segment(polygon[edge], polygon[(edge + 1) % count], 1e-300) <= tolerance {
                return PolygonClass::Boundary;
            }
        }
        if crate::polygon::parity_inside([point.x, point.y], count, |i| [polygon[i].x, polygon[i].y]) {
            PolygonClass::Inside
        } else {
            PolygonClass::Outside
        }
    }
}

fn outside_reject_box(point: Vec2, reject: &[f64; 5]) -> bool {
    let reach = reject[4] + 64.0 * f64::EPSILON * point.x.abs().max(point.y.abs());
    // NaN `reach` (or a NaN point) makes every comparison false: never reject.
    point.x < reject[0] - reach || point.y < reject[1] - reach || point.x > reject[2] + reach || point.y > reject[3] + reach
}

/// `BREP_ARRANGE_DIGON_ORDER` (default on; `=0` restores insertion order for
/// equal angles): see DIGON ORDER in [`arrange_segments`].
fn digon_order_enabled() -> bool {
    switch!("BREP_ARRANGE_DIGON_ORDER", true)
}

fn hole_box_reject_enabled() -> bool {
    switch!("BREP_ARRANGE_HOLE_BOX", true)
}


fn point_in_polygon_class(point: Vec2, polygon: &[Vec2], tolerance: f64) -> PolygonClass {
    crate::polygon::classify(point, polygon, tolerance, 1e-300)
}

pub fn point_in_polygon(point: Vec2, polygon: &[Vec2], tolerance: f64) -> &'static str {
    match point_in_polygon_class(point, polygon, tolerance) {
        PolygonClass::Inside => "in",
        PolygonClass::Outside => "out",
        PolygonClass::Boundary => "boundary",
    }
}

// Arrangement decisions read endpoints and topology, never parent tags. Keep
// the immutable source segment borrowed until a retained cycle needs an owned
// output. The eager arm preserves the original allocation path for A/B.
struct PieceInput<'a> {
    a: Vec2,
    b: Vec2,
    parent: std::borrow::Cow<'a, Segment2>,
}
impl PieceInput<'_> {
    fn materialize(&self, forward: bool) -> CycleUse {
        CycleUse { piece: ArrangementPiece { a: self.a, b: self.b, parent: self.parent.as_ref().clone() }, forward }
    }
}

#[derive(Clone)]
enum CycleUses {
    Eager(Vec<CycleUse>),
    Deferred(Vec<usize>),
}
impl CycleUses {
    fn push(&mut self, half: usize, pieces: &[PieceInput<'_>]) {
        match self {
            Self::Eager(uses) => uses.push(pieces[half >> 1].materialize(half % 2 == 0)),
            Self::Deferred(halves) => halves.push(half),
        }
    }
    fn into_uses(self, pieces: &[PieceInput<'_>]) -> Vec<CycleUse> {
        match self {
            Self::Eager(uses) => uses,
            Self::Deferred(halves) => halves.into_iter().map(|h| pieces[h >> 1].materialize(h % 2 == 0)).collect(),
        }
    }
    fn copy_uses(&self, pieces: &[PieceInput<'_>]) -> Vec<CycleUse> {
        match self {
            Self::Eager(uses) => uses.clone(),
            Self::Deferred(halves) => halves.iter().map(|&h| pieces[h >> 1].materialize(h % 2 == 0)).collect(),
        }
    }
}

#[derive(Clone)]
struct Cycle {
    uses: CycleUses,
    area: f64,
    node_indices: Vec<usize>,
}

/// Visit a conservative subset of pairs in original input order. The narrow
/// phase remains authoritative, including its endpoint tolerance and rounding.
fn visit_segment_pairs(segments: &[Segment2], tolerance: f64, mut visit: impl FnMut(usize, usize)) {
    let all_pairs = |visit: &mut dyn FnMut(usize, usize)| {
        for first in 0..segments.len() {
            for second in first + 1..segments.len() {
                visit(first, second);
            }
        }
    };
    if segments.len() <= 32
        || segments.len() > u32::MAX as usize
        || !tolerance.is_finite()
        || tolerance < 0.0
        || tolerance > 1e100
    {
        all_pairs(&mut visit);
        return;
    }
    let mut scale = 0.0f64;
    let mut boxes = Vec::with_capacity(segments.len());
    let mut extent = Aabb::empty();
    for segment in segments {
        let coordinates = [segment.a.x, segment.a.y, segment.b.x, segment.b.y];
        let length = segment.a.distance(segment.b);
        // Keep the legacy scan for numeric extremes: underflow/overflow or NaN
        // in the narrow phase need not behave like geometric intersection.
        if coordinates
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 1e100)
            || !(1e-100..=1e100).contains(&length)
        {
            all_pairs(&mut visit);
            return;
        }
        for coordinate in coordinates {
            scale = scale.max(coordinate.abs());
        }
        let bounds = Aabb {
            minimum: Vec3::new(
                segment.a.x.min(segment.b.x),
                segment.a.y.min(segment.b.y),
                0.0,
            ),
            maximum: Vec3::new(
                segment.a.x.max(segment.b.x),
                segment.a.y.max(segment.b.y),
                0.0,
            ),
        };
        extent.include(bounds);
        boxes.push(bounds);
    }
    // Each accepted parameter may extend its segment by up to tolerance.
    // Cross-product roundoff is amplified by up to 1/PARALLEL_THRESHOLD when
    // solving the parameters. Bound it using the global coordinate magnitude
    // (including translation), with headroom for subtraction, products, length
    // and division rounding. This deliberately admits extra pairs near parallel
    // lines instead of treating exact endpoint boxes as a bound on an inexact
    // solve. The numeric guards above keep these operations in the normal range.
    let padding = 2.0 * tolerance + 128.0 * f64::EPSILON / PARALLEL_THRESHOLD * scale;
    // A large translation or tolerance can swallow the entire arrangement.
    // An index cannot reject anything then, so avoid its query/sort overhead.
    if padding >= extent.diagonal() {
        all_pairs(&mut visit);
        return;
    }
    let direction_tiers = direction_index_on();
    // Row 29: the direction tiers' small-padding query is answered by a
    // one-axis sweep instead of the tree, which is then not built (`None`
    // from the sweep keeps the tree).
    let small_padding = 2.0 * tolerance + 128.0 * f64::EPSILON / DIRECTION_SINE * scale;
    let swept = if direction_tiers && arrange_sweep_on() { swept_pairs(&boxes, small_padding) } else { None };
    let tree = swept.is_none().then(|| Bvh::build(&boxes));
    let mut candidates = Vec::new();
    if direction_tiers {
        // DIRECTION TIERS (lane O, 2026-10-28). The padding above is what a
        // pair as near-parallel as the narrow phase admits (`sin θ` down to
        // PARALLEL_THRESHOLD) can need, and on a face whose uv sit far from
        // the origin it reaches across most of the arrangement: the filleted
        // box's outward offset sends a 252-segment quarter-arc chain through
        // here twice per attempt and the index visited nearly all 37 000
        // pairs (0.7 ms a face). The same rounding model bounds a pair whose
        // directions differ by `sin θ ≥ s` with the padding at `s` in place of
        // the threshold. So every pair is visited under that small padding,
        // and a pair whose directions lie within one bucket of each other
        // (width `2s`, wrapping at π) — the only pairs that can be nearer
        // parallel than `s` — under the full padding as before. Visit order
        // and the narrow phase are unchanged; the visited set still contains
        // every pair the full padding could accept.
        let width = 2.0 * DIRECTION_SINE;
        let bucket_count = ((std::f64::consts::PI / width).ceil() as usize).max(1);
        let mut buckets: FxHashMap<usize, Vec<usize>> = FxHashMap::default();
        let bucket_of: Vec<usize> = segments
            .iter()
            .map(|segment| {
                let angle = (segment.b.y - segment.a.y).atan2(segment.b.x - segment.a.x).rem_euclid(std::f64::consts::PI);
                ((angle / width) as usize).min(bucket_count - 1)
            })
            .collect();
        for (index, bucket) in bucket_of.iter().enumerate() {
            buckets.entry(*bucket).or_default().push(index);
        }
        let near = |a: &Aabb, b: &Aabb| {
            a.minimum.x - padding <= b.maximum.x
                && b.minimum.x - padding <= a.maximum.x
                && a.minimum.y - padding <= b.maximum.y
                && b.minimum.y - padding <= a.maximum.y
        };
        for (first, &bounds) in boxes.iter().enumerate() {
            candidates.clear();
            match (&swept, &tree) {
                (Some(lists), _) => candidates.extend_from_slice(&lists[first]),
                (None, Some(tree)) => tree.overlapping(bounds, small_padding, &mut candidates),
                (None, None) => unreachable!("the tree is built whenever the sweep is not"),
            }
            let bucket = bucket_of[first];
            for neighbour in [(bucket + bucket_count - 1) % bucket_count, bucket, (bucket + 1) % bucket_count] {
                if let Some(members) = buckets.get(&neighbour) {
                    candidates.extend(members.iter().copied().filter(|&second| second > first && near(&bounds, &boxes[second])));
                }
                if bucket_count < 3 {
                    break;
                }
            }
            candidates.retain(|&second| second > first);
            candidates.sort_unstable();
            candidates.dedup();
            for &second in &candidates {
                visit(first, second);
            }
        }
        return;
    }
    let Some(tree) = tree else { unreachable!("the sweep runs only under the direction tiers") };
    for (first, &bounds) in boxes.iter().enumerate() {
        candidates.clear();
        tree.overlapping(bounds, padding, &mut candidates);
        candidates.retain(|&second| second > first);
        candidates.sort_unstable();
        for &second in &candidates {
            visit(first, second);
        }
    }
}

/// The direction sine above which [`visit_segment_pairs`] pads a pair with
/// the small padding (see DIRECTION TIERS there).
const DIRECTION_SINE: f64 = 1e-3;

/// Lane O row 29 (2026-10-30, `BREP_ARRANGE_SWEEP`, default on since batch 12;
/// `0`/`off` restores the tree): for each box `first`, every `second > first` whose box passes
/// the tree's own leaf test `boxes[second].intersects(boxes[first], padding)`
/// — the set [`Bvh::overlapping`] returns past `second > first`, since a node
/// box contains its items' boxes — in ascending order. The candidates come
/// from a sweep over the boxes sorted by their minimum on one axis, each
/// scanning forward while the next minimum is within its maximum plus twice
/// the padding: a pair the leaf test admits lies within the padding plus one
/// rounding of either subtraction there, and the padding (at least
/// 128 ε / DIRECTION_SINE of the largest coordinate) exceeds that rounding.
///
/// The axis is the one whose sweep windows (counted by binary search before
/// any test runs) are fewer in total; when even those exceed 32 a box, the
/// boxes overlap along both axes and `None` hands the query back to the tree,
/// which prunes on both at once (a guard: of the D1, inward opposed and
/// outward 3-adjacent shell cases probed, only D1 reaches the sweep, and
/// never the fallback). D1's two 272-segment faces spent 0.38 ms a run
/// building and querying the tree for 570 narrow-phase pairs that cost
/// 0.04 ms.
fn swept_pairs(boxes: &[Aabb], padding: f64) -> Option<Vec<Vec<usize>>> {
    let reach = 2.0 * padding;
    let low = |index: usize, axis: usize| if axis == 0 { boxes[index].minimum.x } else { boxes[index].minimum.y };
    let high = |index: usize, axis: usize| if axis == 0 { boxes[index].maximum.x } else { boxes[index].maximum.y };
    let mut best: Option<(usize, Vec<usize>, usize)> = None;
    for axis in [0, 1] {
        let mut order: Vec<usize> = (0..boxes.len()).collect();
        order.sort_by(|&a, &b| low(a, axis).total_cmp(&low(b, axis)));
        let lows: Vec<f64> = order.iter().map(|&index| low(index, axis)).collect();
        let windows: usize = order
            .iter()
            .enumerate()
            .map(|(position, &a)| {
                let limit = high(a, axis) + reach;
                lows.partition_point(|&value| value <= limit).saturating_sub(position + 1)
            })
            .sum();
        if best.as_ref().is_none_or(|(least, _, _)| windows < *least) {
            best = Some((windows, order, axis));
        }
    }
    let (windows, order, axis) = best?;
    if windows > 32 * boxes.len() {
        return None;
    }
    let mut lists = vec![Vec::new(); boxes.len()];
    for (position, &a) in order.iter().enumerate() {
        let limit = high(a, axis) + reach;
        for &b in &order[position + 1..] {
            if low(b, axis) > limit {
                break;
            }
            let (first, second) = if a < b { (a, b) } else { (b, a) };
            if boxes[second].intersects(boxes[first], padding) {
                lists[first].push(second);
            }
        }
    }
    for list in &mut lists {
        list.sort_unstable();
    }
    Some(lists)
}

/// Row 29's switch ([`swept_pairs`]), read once.
fn arrange_sweep_on() -> bool {
    switch!("BREP_ARRANGE_SWEEP", true)
}


/// The direction tiers of [`visit_segment_pairs`], on unless
/// `BREP_ARRANGE_DIRECTION_INDEX` is `0`/`off` (the A/B lever). Read once.
fn direction_index_on() -> bool {
    switch!("BREP_ARRANGE_DIRECTION_INDEX", true)
}

pub fn arrange_segments(
    segments: &[Segment2],
    tolerance: f64,
) -> Result<Vec<ArrangementRegion>, String> {
    arrange_segments_impl(segments, tolerance, true, true)
}

fn arrange_segments_impl(
    segments: &[Segment2],
    tolerance: f64,
    indexed: bool,
    indexed_nodes: bool,
) -> Result<Vec<ArrangementRegion>, String> {
    let defer_tags = switch!("BREP_ARRANGE_DEFER_TAGS", true);
    let mut cuts = vec![Vec::<f64>::new(); segments.len()];
    let mut intersect_pair = |first: usize, second: usize| {
        let Some([first_parameter, second_parameter]) = segment_intersection(
            segments[first].a,
            segments[first].b,
            segments[second].a,
            segments[second].b,
            tolerance,
        ) else {
            return;
        };
        let first_tolerance = tolerance_parameter(&segments[first], tolerance);
        let second_tolerance = tolerance_parameter(&segments[second], tolerance);
        if first_parameter > first_tolerance && first_parameter < 1.0 - first_tolerance {
            cuts[first].push(first_parameter);
        }
        if second_parameter > second_tolerance && second_parameter < 1.0 - second_tolerance {
            cuts[second].push(second_parameter);
        }
    };
    if indexed {
        visit_segment_pairs(segments, tolerance, &mut intersect_pair);
    } else {
        for first in 0..segments.len() {
            for second in first + 1..segments.len() {
                intersect_pair(first, second);
            }
        }
    }

    let mut pieces = Vec::new();
    for (index, segment) in segments.iter().enumerate() {
        let mut parameters = vec![0.0];
        cuts[index].sort_by(f64::total_cmp);
        parameters.extend(cuts[index].iter().copied());
        parameters.push(1.0);
        let parameter_tolerance = tolerance_parameter(segment, tolerance);
        let mut deduplicated: Vec<f64> = Vec::new();
        for parameter in parameters {
            if deduplicated
                .last()
                .is_none_or(|previous| parameter - *previous > parameter_tolerance)
            {
                deduplicated.push(parameter);
            } else if parameter == 1.0 {
                *deduplicated.last_mut().unwrap() = 1.0;
            }
        }
        for pair in deduplicated.windows(2) {
            let a = interpolate(segment.a, segment.b, pair[0]);
            let b = interpolate(segment.a, segment.b, pair[1]);
            if a.distance(b) <= tolerance {
                continue;
            }
            pieces.push(PieceInput {
                a,
                b,
                parent: if defer_tags { std::borrow::Cow::Borrowed(segment) } else { std::borrow::Cow::Owned(segment.clone()) },
            });
        }
    }

    let mut nodes: Vec<Node> = Vec::new();
    let mut lookup = NodeLookup::new(tolerance, indexed_nodes && pieces.len() > 64);
    let mut piece_start = Vec::with_capacity(pieces.len());
    let mut piece_end = Vec::with_capacity(pieces.len());
    for piece in &pieces {
        piece_start.push(lookup.find_or_insert(piece.a, &mut nodes));
        piece_end.push(lookup.find_or_insert(piece.b, &mut nodes));
    }

    let mut alive = vec![true; pieces.len()];
    loop {
        let mut pruned = false;
        let mut degree = vec![0usize; nodes.len()];
        for index in 0..pieces.len() {
            if !alive[index] {
                continue;
            }
            if piece_start[index] == piece_end[index] {
                alive[index] = false;
                pruned = true;
                continue;
            }
            degree[piece_start[index]] += 1;
            degree[piece_end[index]] += 1;
        }
        for index in 0..pieces.len() {
            if alive[index] && (degree[piece_start[index]] == 1 || degree[piece_end[index]] == 1) {
                alive[index] = false;
                pruned = true;
            }
        }
        if !pruned {
            break;
        }
    }

    let half_tail = |half_edge: usize| {
        if half_edge % 2 == 0 {
            piece_start[half_edge >> 1]
        } else {
            piece_end[half_edge >> 1]
        }
    };
    let half_head = |half_edge: usize| {
        if half_edge % 2 == 0 {
            piece_end[half_edge >> 1]
        } else {
            piece_start[half_edge >> 1]
        }
    };
    for node in &mut nodes {
        node.outgoing.clear();
    }
    for index in 0..pieces.len() {
        if !alive[index] {
            continue;
        }
        nodes[piece_start[index]].outgoing.push(index * 2);
        nodes[piece_end[index]].outgoing.push(index * 2 + 1);
    }
    let angles: Vec<f64> = (0..pieces.len() * 2)
        .map(|half_edge| {
            let tail = nodes[half_tail(half_edge)].point;
            let head = nodes[half_head(half_edge)].point;
            (head.y - tail.y).atan2(head.x - tail.x)
        })
        .collect();
    // DIGON ORDER: two pieces joining the SAME two nodes (a cut lying along a
    // trim edge for its last stretch, merged onto the edge's nodes) leave
    // the same angle at both ends, and a sort by angle alone keeps them in
    // insertion order at BOTH nodes, which no planar embedding has: the walk
    // then crosses between the two sides of the pair and stitches the region
    // beside it into the outer cycle, so the region is lost (an offset
    // fillet's skin vanished at a miter corner, 2026-11-07). Equal
    // angles are ordered by piece index, reversed at the higher node, so the
    // pair bounds an empty digon and both neighbours close.
    let digon_order = digon_order_enabled();
    for node in &mut nodes {
        node.outgoing.sort_by(|a, b| {
            angles[*a].total_cmp(&angles[*b]).then_with(|| {
                if !digon_order {
                    return std::cmp::Ordering::Equal;
                }
                let rank = |half_edge: usize| {
                    let piece = (half_edge >> 1) as i64;
                    if half_tail(half_edge) < half_head(half_edge) {
                        piece
                    } else {
                        -piece
                    }
                };
                rank(*a).cmp(&rank(*b))
            })
        });
    }

    let mut visited = rustc_hash::FxHashSet::default();
    let mut positive = Vec::new();
    let mut negative = Vec::new();
    for index in 0..pieces.len() {
        if !alive[index] {
            continue;
        }
        for start in [index * 2, index * 2 + 1] {
            if visited.contains(&start) {
                continue;
            }
            let mut uses = if defer_tags { CycleUses::Deferred(Vec::new()) } else { CycleUses::Eager(Vec::new()) };
            let mut node_indices = Vec::new();
            let mut area = 0.0;
            let mut half_edge = start;
            let mut guard = 0;
            loop {
                visited.insert(half_edge);
                uses.push(half_edge, &pieces);
                let tail_index = half_tail(half_edge);
                let head_index = half_head(half_edge);
                let tail = nodes[tail_index].point;
                let head = nodes[head_index].point;
                area += 0.5 * (tail.x * head.y - head.x * tail.y);
                node_indices.push(tail_index);
                let reverse = half_edge ^ 1;
                let outgoing = &nodes[head_index].outgoing;
                let reverse_index = outgoing
                    .iter()
                    .position(|candidate| *candidate == reverse)
                    .ok_or_else(|| "arrangeSegments: reverse half-edge missing".to_string())?;
                half_edge = outgoing[(reverse_index + outgoing.len() - 1) % outgoing.len()];
                guard += 1;
                if guard > pieces.len() * 4 + 8 {
                    return Err("arrangeSegments: face walk did not terminate".into());
                }
                if half_edge == start {
                    break;
                }
            }
            let cycle = Cycle {
                uses,
                area,
                node_indices,
            };
            if area > tolerance * tolerance {
                positive.push(cycle);
            } else if area < -tolerance * tolerance {
                negative.push(cycle);
            }
        }
    }

    positive.sort_by(|a, b| a.area.total_cmp(&b.area));
    let mut regions: Vec<(ArrangementRegion, Vec<Vec2>)> = positive
        .into_iter()
        .map(|cycle| {
            let polygon = cycle
                .node_indices
                .iter()
                .map(|index| nodes[*index].point)
                .collect();
            (
                ArrangementRegion {
                    outer: cycle.uses.into_uses(&pieces),
                    holes: Vec::new(),
                    area: cycle.area,
                    outer_area: cycle.area,
                },
                polygon,
            )
        })
        .collect();
    let hole_boxes = hole_box_reject_enabled();
    let boxes: Vec<[f64; 5]> = if hole_boxes {
        regions.iter().map(|(_, polygon)| polygon_reject_box(polygon, tolerance)).collect()
    } else {
        Vec::new()
    };
    let mut bands: Vec<Option<BoundaryBand>> = (0..regions.len()).map(|_| None).collect();
    for cycle in negative {
        for (region_index, (region, polygon)) in regions.iter_mut().enumerate() {
            let mut inside = false;
            let mut on_boundary = false;
            for node_index in &cycle.node_indices {
                let point = nodes[*node_index].point;
                let class = if hole_boxes && outside_reject_box(point, &boxes[region_index]) {
                    PolygonClass::Outside
                } else if hole_boxes {
                    let band = bands[region_index].get_or_insert_with(|| BoundaryBand::new(polygon, tolerance));
                    band.classify(point, polygon, tolerance)
                } else {
                    point_in_polygon_class(point, polygon, tolerance)
                };
                match class {
                    PolygonClass::Boundary => {
                        on_boundary = true;
                        continue;
                    }
                    PolygonClass::Inside => {
                        inside = true;
                        on_boundary = false;
                        break;
                    }
                    PolygonClass::Outside => {
                        inside = false;
                        on_boundary = false;
                        break;
                    }
                }
            }
            if on_boundary {
                continue;
            }
            if inside {
                region.holes.push(cycle.uses.copy_uses(&pieces));
                region.area += cycle.area;
                break;
            }
        }
    }
    if crate::profile::profile_enabled() {
        let holes: usize = regions.iter().flat_map(|(r, _)| r.holes.iter()).map(Vec::len).sum();
        let returned: usize = holes + regions.iter().map(|(r, _)| r.outer.len()).sum::<usize>();
        let clones = if defer_tags { returned } else { pieces.len() + visited.len() + holes };
        crate::imprint::profile_emit(format_args!("arrange.tags segments={} pieces={} walked={} returned={returned} parent_clones={clones} deferred={defer_tags}", segments.len(), pieces.len(), visited.len()));
    }
    Ok(regions.into_iter().map(|(region, _)| region).collect())
}


