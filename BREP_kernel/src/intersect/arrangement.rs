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

#[derive(Clone, Copy, PartialEq)]
enum PolygonClass {
    Inside,
    Outside,
    Boundary,
}

fn point_segment_distance(point: Vec2, a: Vec2, b: Vec2) -> f64 {
    let segment = b.sub(a);
    let length_squared = segment.dot(segment);
    if length_squared <= 1e-300 {
        return point.distance(a);
    }
    let parameter = (point.sub(a).dot(segment) / length_squared).clamp(0.0, 1.0);
    point.distance(Vec2 {
        x: a.x + segment.x * parameter,
        y: a.y + segment.y * parameter,
    })
}

fn point_in_polygon_class(point: Vec2, polygon: &[Vec2], tolerance: f64) -> PolygonClass {
    for index in 0..polygon.len() {
        if point_segment_distance(point, polygon[index], polygon[(index + 1) % polygon.len()])
            <= tolerance
        {
            return PolygonClass::Boundary;
        }
    }
    let mut inside = false;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        if (a.y > point.y) != (b.y > point.y) {
            let crossing = a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x);
            if crossing > point.x {
                inside = !inside;
            }
        }
    }
    if inside {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    }
}

pub fn point_in_polygon(point: Vec2, polygon: &[Vec2], tolerance: f64) -> &'static str {
    match point_in_polygon_class(point, polygon, tolerance) {
        PolygonClass::Inside => "in",
        PolygonClass::Outside => "out",
        PolygonClass::Boundary => "boundary",
    }
}

#[derive(Clone)]
struct Cycle {
    uses: Vec<CycleUse>,
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
    let tree = Bvh::build(&boxes);
    let mut candidates = Vec::new();
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
            pieces.push(ArrangementPiece {
                a,
                b,
                parent: segment.clone(),
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
    for node in &mut nodes {
        node.outgoing
            .sort_by(|a, b| angles[*a].total_cmp(&angles[*b]));
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
            let mut uses = Vec::new();
            let mut node_indices = Vec::new();
            let mut area = 0.0;
            let mut half_edge = start;
            let mut guard = 0;
            loop {
                visited.insert(half_edge);
                let piece_index = half_edge >> 1;
                uses.push(CycleUse {
                    piece: pieces[piece_index].clone(),
                    forward: half_edge % 2 == 0,
                });
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
                    outer: cycle.uses,
                    holes: Vec::new(),
                    area: cycle.area,
                },
                polygon,
            )
        })
        .collect();
    for cycle in negative {
        for (region, polygon) in &mut regions {
            let mut inside = false;
            let mut on_boundary = false;
            for node_index in &cycle.node_indices {
                match point_in_polygon_class(nodes[*node_index].point, polygon, tolerance) {
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
                region.holes.push(cycle.uses.clone());
                region.area += cycle.area;
                break;
            }
        }
    }
    Ok(regions.into_iter().map(|(region, _)| region).collect())
}

