use crate::curve::interior_knot_count;
use super::*;

/// How many chords the containment lanes sample ONE pcurve at:
/// `max(2, (k + 1)(p + 1)·4)`, `k` the pcurve's distinct interior knots (two
/// knots are one when they differ by no more than
/// [`KNOT_DEDUP_EPS`](crate::KNOT_DEDUP_EPS), and a knot that close to an end
/// of the active domain is not interior) and `p` its degree. Knot- and
/// degree-aware, so a spline trim is resolved wherever its shape can bend and a
/// line is two chords.
///
/// Nothing compares the count; it IS the polygon [`parameter_point_in_face`]
/// classifies against. The generic scan, the wrapped-horizon, sphere-cap and
/// covering-rim-strip lanes sample every pcurve at
/// [`trim_station`]`(start, end, i, count)` for `i` in `0..count` (the rim
/// strip `0..=count`), the pcurve's domain being `[start, end]`.
///
/// Published rather than copied: [`crate::loop_self_crossings`] and the
/// soundness detectors test EXACTLY these chords, or the kernel could hold that
/// a loop does not cross itself while classifying points against a polygon
/// that does; and a caller that answers containment questions ahead of the
/// kernel (the sheets' hidden-line pass) must build its polygon from the same
/// stations, or it stops answering as the kernel does.
pub fn trim_sample_count(curve: &NurbsCurve) -> usize {
    2usize.max((interior_knot_count(&curve.knots, curve.degree) + 1) * (curve.degree + 1) * 4)
}

/// The pcurve parameter of trim station `index` of `count` over `[start, end]`:
/// `start + (end − start)·index / count`, evaluated in exactly that order.
///
/// The comparand is bit identity with the stations the lanes sampled: the
/// prepared-face cache, the self-crossing detector and any caller mirroring a
/// lane rely on the same floats. At `index == count` this is NOT `end` to the
/// bit; the generic scan closes a pcurve's last chord at `end` itself.
pub fn trim_station(start: f64, end: f64, index: usize, count: usize) -> f64 {
    start + (end - start) * index as f64 / count as f64
}

/// `BREP_SEAM_BAND_MERGED`: set to `"0"` (the only value compared), the
/// doubly-periodic seam-band lane stops unwrapping a single merged seam-carrying
/// loop, and that face goes on to the two-loop analyses or the generic scan.
/// Read when a face is prepared; part of the prepared-face cache key.
pub const SEAM_BAND_MERGED_SWITCH: &str = "BREP_SEAM_BAND_MERGED";

/// `BREP_HORIZON_CONTAINMENT`: set to `"0"`, the wrapped-horizon lane declines
/// every face. Read when a face is prepared; part of the cache key.
pub const HORIZON_CONTAINMENT_SWITCH: &str = "BREP_HORIZON_CONTAINMENT";

/// `BREP_HORIZON_SINGLE_LOOP`: set to `"0"`, the wrapped-horizon lane declines a
/// face of one or two loops (compared: `face.loops.len() <= 2`). Read when a
/// face is prepared; part of the cache key.
pub const HORIZON_SINGLE_LOOP_SWITCH: &str = "BREP_HORIZON_SINGLE_LOOP";

/// `BREP_HORIZON_CROSS_FRAME`: set to `"0"`, the wrapped-horizon lane combines
/// its period images per image (Inside when any image counts odd) instead of
/// summing each loop's parity over the images. Read per QUERY, not when the
/// face is prepared; see [`horizon_cross_frame`].
pub const HORIZON_CROSS_FRAME_SWITCH: &str = "BREP_HORIZON_CROSS_FRAME";

/// `BREP_COVERING_RIM_STRIP`: set to `"0"`, the covering-rim-strip lane declines
/// every face. Read when a face is prepared; part of the cache key.
pub const COVERING_RIM_STRIP_SWITCH: &str = "BREP_COVERING_RIM_STRIP";

/// `BREP_NO_SPHERE_CHART_TRIM`: SET to any value (compared: present in the
/// environment), the sphere-chart lane stops substituting for the generic scan's
/// parity count. `BREP_NO_SPHERE_CHARTS`, which turns every sphere-chart path
/// off, does the same here. Read per query.
pub const NO_SPHERE_CHART_TRIM_SWITCH: &str = "BREP_NO_SPHERE_CHART_TRIM";

/// Whether a `"0"`-style lane switch is off: its environment value is exactly
/// `"0"`.
fn switched_off(name: &str) -> bool {
    std::env::var(name).as_deref() == Ok("0")
}

/// Whether the wrapped-horizon lane sums each loop's parity over the query's
/// period images (`true`, the default) or keeps the per-image combining
/// (`false`, [`HORIZON_CROSS_FRAME_SWITCH`] = `"0"`). Read from the environment
/// on every call, as the lane reads it on every query.
pub fn horizon_cross_frame() -> bool {
    !switched_off(HORIZON_CROSS_FRAME_SWITCH)
}

/// Whether the sphere-chart lane may substitute for the generic scan's parity
/// count: neither `BREP_NO_SPHERE_CHARTS` nor [`NO_SPHERE_CHART_TRIM_SWITCH`]
/// is set. Read on every call.
fn sphere_chart_trim_enabled() -> bool {
    std::env::var("BREP_NO_SPHERE_CHARTS").is_err()
        && std::env::var(NO_SPHERE_CHART_TRIM_SWITCH).is_err()
}

fn point_segment_distance(point: Vec2, start: Vec2, end: Vec2) -> f64 {
    let segment = end.sub(start);
    let length_squared = segment.dot(segment);
    if length_squared <= 1e-30 {
        return point.sub(start).length();
    }
    let parameter = (point.sub(start).dot(segment) / length_squared).clamp(0.0, 1.0);
    point.sub(start.add(segment.scale(parameter))).length()
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum PolygonClass {
    Inside,
    Outside,
    Boundary,
}

fn point_in_polygon(point: Vec2, polygon: &[Vec2], tolerance: f64) -> PolygonClass {
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


/// One chord of a trim loop as the generic scan samples it: the loop and
/// coedge it belongs to (the pcurve is looked up on the face at query time —
/// the prepared data is cached across faces and must own nothing borrowed)
/// and the pcurve parameters at its ends.
#[derive(Clone, Copy)]
struct SegmentPrep {
    coedge_index: usize,
    parameter_start: f64,
    parameter_end: f64,
}

/// Which of the environment switches the lanes read were set when a face was
/// prepared; part of the cache signature so a test that flips one between two
/// queries on one thread never answers from data prepared under the other.
fn lane_switches() -> f64 {
    let mut bits = 0u32;
    if switched_off(SEAM_BAND_MERGED_SWITCH) {
        bits |= 1;
    }
    if switched_off(HORIZON_CONTAINMENT_SWITCH) {
        bits |= 2;
    }
    if switched_off(HORIZON_SINGLE_LOOP_SWITCH) {
        bits |= 4;
    }
    if switched_off(COVERING_RIM_STRIP_SWITCH) {
        bits |= 8;
    }
    bits as f64
}

/// Doubly-periodic seam-band lane, prepared.
enum SeamBandPrep {
    /// The surface is not closed in both directions: the lane declines.
    NotApplicable,
    /// Every loop is a collapsed puncture: the whole torus is material.
    AlwaysInside,
    /// One seam-crossing loop unwrapped onto the covering plane.
    Merged { polygon: Vec<Vec2>, u_span: f64, v_span: f64 },
    /// Two loops the seam-band analysis reassembled into one polygon.
    Analyzed { polygon: Vec<Vec2> },
    /// Two clean full-wrap rims at constant cross levels.
    Rings { p_is_u: bool, q_lo: f64, q_hi: f64, complement: bool },
    /// None of the shapes above: keep the general path.
    Declined,
}

/// Winding sphere-cap lane, prepared: `None` when the face is not a cap.
struct SphereCapPrep {
    rim: Vec<Vec2>,
    pole_v: f64,
    period: f64,
}

/// Wrapped-horizon lane, prepared: `None` when no loop straddles the seam.
struct HorizonPrep {
    loops: Vec<Vec<Vec2>>,
    u_period: f64,
    /// The loops' chords, indexed the first time a query asks.
    index: std::cell::OnceCell<Option<ChordIndex>>,
}

/// Covering rim-strip lane, prepared: `None` when the face is not a strip.
struct RimStripPrep {
    lower: Vec<Vec2>,
    upper: Vec<Vec2>,
    period: f64,
}

/// The generic scan's samples: per loop the parity polygon, the chord
/// parameters behind it and the per-coedge runs the spherical lane reads.
struct GenericPrep {
    loops: Vec<LoopSamples>,
    segments: Vec<Vec<SegmentPrep>>,
    /// Every loop's chords, counted.
    chord_count: usize,
    /// The chords indexed ([`ChordIndex`]), built the first time a query asks
    /// for the index; `None` inside when a sample is too large or not finite.
    index: std::cell::OnceCell<Option<ChordIndex>>,
}

/// The fewest chords a lane must carry before a query goes through its
/// [`ChordIndex`] rather than scanning every chord: below it the scan costs
/// about what a descent does. Either answers the same.
const CHORD_INDEX_MIN_CHORDS: usize = 64;

/// Chords per leaf of a [`ChordIndex`].
const CHORD_INDEX_LEAF: usize = 6;

/// The largest coordinate, of a chord end or of a query point, the index is
/// used for. Squares of anything smaller stay finite, so every distance the
/// scan computes is a finite number the index can bound; beyond it the scan
/// answers.
const CHORD_INDEX_MAX_COORDINATE: f64 = 1e100;

/// Whether [`parameter_point_in_face`] may answer through a lane's
/// [`ChordIndex`]: `BREP_CONTAINMENT_INDEX=0` keeps the linear scans
/// everywhere, for an A/B of the two in one build. Read once per process.
fn chord_index_enabled() -> bool {
    static ENABLED: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ENABLED.get_or_init(|| !switched_off("BREP_CONTAINMENT_INDEX"))
}

/// A node of a [`ChordIndex`]: the exact box (the min and max of the samples,
/// not a computed value) of a run of consecutive chords of one loop, chords
/// `first..last` of loop `loop_index`, chord `i` running from `polygon[i]` to
/// `polygon[(i + 1) % len]`. A leaf reads its chords; an inner node has two
/// children, the runs before and after its middle.
#[derive(Clone, Copy)]
struct ChordNode {
    low: Vec2,
    high: Vec2,
    loop_index: u32,
    first: u32,
    last: u32,
    children: Option<(u32, u32)>,
}

/// A hierarchy of boxes over every chord of a face's generic scan, or of its
/// wrapped-horizon lane's unwrapped loops, so a query reads the chords near it
/// rather than all of them — and gives the scan's answer to the bit.
///
/// Consecutive chords of a trim lie together, so each loop's chords are cut
/// into runs of [`CHORD_INDEX_LEAF`] and the runs paired, and paired again, up
/// to one box per loop: built in one pass over the samples, holding no copy of
/// them. (A median-split tree over the chords of `gear-hex-bore-push`'s face
/// 8307, 557 504 of them, took 200 ms to build — more than the questions it
/// answered saved.)
///
/// The scan asks three things of its chords, and the index asks each of the
/// same chords with the same arithmetic (the wrapped horizon asks the second
/// and third, at each of its three period images):
///
/// - the NEAREST chord, `point_segment_distance` to every chord with a strict
///   `<` in loop-then-chord order, so the first minimum is kept. The index
///   keeps the least `(distance, loop, chord)`, which is that chord, and skips a
///   box only when its distance exceeds the best by more than `slack`;
/// - whether any chord is within the tolerance (`Boundary`): exactly when the
///   nearest chord's distance is, both being that same float;
/// - each loop's even-odd count along +x, the test `(a.y > p.y) != (b.y > p.y)`
///   and `crossing > p.x`. A chord that passes the first has one sample above
///   `p.y` and one at or below it, which its box holds exactly, and its
///   crossing is within rounding of its samples' largest `x`; a box is skipped
///   only when its `y` range cannot straddle `p.y` or its largest `x` is short
///   of `p.x` by more than `slack`.
///
/// `slack` is `1e-12` of the largest coordinate in play, some four thousand
/// times the few units in the last place a computed foot or crossing can stray
/// outside its chord's box. Distances and crossings are only COMPARED against
/// boxes with that slack; every value that decides an answer is the scan's own.
struct ChordIndex {
    nodes: Vec<ChordNode>,
    /// One per loop with chords.
    roots: Vec<u32>,
    /// `1e-12 × (1 + the largest |coordinate| of a sample)`.
    slack: f64,
}

/// The closed polygons a [`ChordIndex`] is built over and read against: the
/// generic scan's loops, or the wrapped-horizon lane's unwrapped ones.
trait ChordLoops {
    fn loop_count(&self) -> usize;
    fn polygon(&self, index: usize) -> &[Vec2];
}

impl ChordLoops for [LoopSamples] {
    fn loop_count(&self) -> usize {
        self.len()
    }

    fn polygon(&self, index: usize) -> &[Vec2] {
        &self[index].polygon
    }
}

impl ChordLoops for [Vec<Vec2>] {
    fn loop_count(&self) -> usize {
        self.len()
    }

    fn polygon(&self, index: usize) -> &[Vec2] {
        &self[index]
    }
}

impl ChordIndex {
    /// The index over `loops`, or `None` when a sample is not finite, a
    /// coordinate exceeds [`CHORD_INDEX_MAX_COORDINATE`], a count does not fit
    /// in `u32`, or there is no chord.
    fn build<L: ChordLoops + ?Sized>(loops: &L) -> Option<ChordIndex> {
        let mut nodes: Vec<ChordNode> = Vec::new();
        let mut roots = Vec::new();
        let mut largest: f64 = 0.0;
        for loop_index in 0..loops.loop_count() {
            let polygon = loops.polygon(loop_index);
            let len = polygon.len();
            if len == 0 {
                continue;
            }
            u32::try_from(len).ok()?;
            for sample in polygon {
                if !(sample.x.abs() <= CHORD_INDEX_MAX_COORDINATE && sample.y.abs() <= CHORD_INDEX_MAX_COORDINATE) {
                    return None;
                }
                largest = largest.max(sample.x.abs()).max(sample.y.abs());
            }
            let loop_index = u32::try_from(loop_index).ok()?;
            let mut level: Vec<u32> = Vec::with_capacity(len / CHORD_INDEX_LEAF + 1);
            for first in (0..len).step_by(CHORD_INDEX_LEAF) {
                let last = (first + CHORD_INDEX_LEAF).min(len);
                let (mut low, mut high) = (polygon[first], polygon[first]);
                for index in first..last {
                    let end = polygon[(index + 1) % len];
                    low = Vec2 { x: low.x.min(end.x), y: low.y.min(end.y) };
                    high = Vec2 { x: high.x.max(end.x), y: high.y.max(end.y) };
                    let start = polygon[index];
                    low = Vec2 { x: low.x.min(start.x), y: low.y.min(start.y) };
                    high = Vec2 { x: high.x.max(start.x), y: high.y.max(start.y) };
                }
                level.push(u32::try_from(nodes.len()).ok()?);
                nodes.push(ChordNode {
                    low,
                    high,
                    loop_index,
                    first: first as u32,
                    last: last as u32,
                    children: None,
                });
            }
            while level.len() > 1 {
                let mut next = Vec::with_capacity(level.len() / 2 + 1);
                for pair in level.chunks(2) {
                    let &[left, right] = pair else {
                        next.push(pair[0]);
                        continue;
                    };
                    let (a, b) = (nodes[left as usize], nodes[right as usize]);
                    next.push(u32::try_from(nodes.len()).ok()?);
                    nodes.push(ChordNode {
                        low: Vec2 { x: a.low.x.min(b.low.x), y: a.low.y.min(b.low.y) },
                        high: Vec2 { x: a.high.x.max(b.high.x), y: a.high.y.max(b.high.y) },
                        loop_index,
                        first: a.first,
                        last: b.last,
                        children: Some((left, right)),
                    });
                }
                level = next;
            }
            roots.push(level[0]);
        }
        if roots.is_empty() {
            return None;
        }
        Some(ChordIndex {
            nodes,
            roots,
            slack: 1e-12 * (1.0 + largest),
        })
    }

    /// The slack for a query at `point`: the index's own plus `1e-12` of the
    /// point's largest coordinate.
    fn slack_at(&self, point: Vec2) -> f64 {
        self.slack + 1e-12 * point.x.abs().max(point.y.abs())
    }

    fn box_distance(node: &ChordNode, point: Vec2) -> f64 {
        let dx = (node.low.x - point.x).max(point.x - node.high.x).max(0.0);
        let dy = (node.low.y - point.y).max(point.y - node.high.y).max(0.0);
        (dx * dx + dy * dy).sqrt()
    }

    /// The scan's nearest chord of `loops` to `point` as `(loop, chord,
    /// distance)`: the least distance, and of equal distances the first in
    /// loop-then-chord order.
    fn nearest<L: ChordLoops + ?Sized>(&self, loops: &L, point: Vec2) -> Option<(usize, usize, f64)> {
        let slack = self.slack_at(point);
        let mut best: Option<(f64, u32, u32)> = None;
        let mut stack: Vec<(u32, f64)> = Vec::with_capacity(64);
        for &root in &self.roots {
            stack.push((root, Self::box_distance(&self.nodes[root as usize], point)));
        }
        while let Some((node_index, bound)) = stack.pop() {
            if best.is_some_and(|(distance, _, _)| bound > distance + slack + 1e-12 * distance) {
                continue;
            }
            let node = self.nodes[node_index as usize];
            match node.children {
                None => {
                    let polygon = loops.polygon(node.loop_index as usize);
                    for index in node.first..node.last {
                        let start = polygon[index as usize];
                        let end = polygon[(index as usize + 1) % polygon.len()];
                        let distance = point_segment_distance(point, start, end);
                        let better = best.is_none_or(|(least, loop_index, chord_index)| {
                            distance < least
                                || (distance == least && (node.loop_index, index) < (loop_index, chord_index))
                        });
                        if better {
                            best = Some((distance, node.loop_index, index));
                        }
                    }
                }
                Some((left, right)) => {
                    let left_bound = Self::box_distance(&self.nodes[left as usize], point);
                    let right_bound = Self::box_distance(&self.nodes[right as usize], point);
                    // The nearer box is read first, so the best tightens early.
                    if left_bound <= right_bound {
                        stack.push((right, right_bound));
                        stack.push((left, left_bound));
                    } else {
                        stack.push((left, left_bound));
                        stack.push((right, right_bound));
                    }
                }
            }
        }
        best.map(|(distance, loop_index, chord_index)| {
            (loop_index as usize, chord_index as usize, distance)
        })
    }

    /// Whether a chord of `loops` lies within `tolerance` of `point`, by
    /// `point_segment_distance`, the scan's own test.
    fn any_within<L: ChordLoops + ?Sized>(&self, loops: &L, point: Vec2, tolerance: f64) -> bool {
        let reach = tolerance + self.slack_at(point) + 1e-12 * tolerance.abs();
        let mut stack: Vec<u32> = self.roots.clone();
        while let Some(node_index) = stack.pop() {
            let node = self.nodes[node_index as usize];
            if Self::box_distance(&node, point) > reach {
                continue;
            }
            match node.children {
                None => {
                    let polygon = loops.polygon(node.loop_index as usize);
                    for index in node.first as usize..node.last as usize {
                        let (start, end) = (polygon[index], polygon[(index + 1) % polygon.len()]);
                        if point_segment_distance(point, start, end) <= tolerance {
                            return true;
                        }
                    }
                }
                Some((left, right)) => {
                    stack.push(right);
                    stack.push(left);
                }
            }
        }
        false
    }

    /// Flips `odd[loop]` once for every chord of that loop the +x ray from
    /// `point` crosses, with `point_in_polygon`'s crossing test.
    fn flip_crossings<L: ChordLoops + ?Sized>(&self, loops: &L, point: Vec2, odd: &mut [bool]) {
        let slack = self.slack_at(point);
        let mut stack: Vec<u32> = self.roots.clone();
        while let Some(node_index) = stack.pop() {
            let node = self.nodes[node_index as usize];
            if !(node.low.y <= point.y && node.high.y > point.y) || node.high.x + slack < point.x {
                continue;
            }
            match node.children {
                None => {
                    let polygon = loops.polygon(node.loop_index as usize);
                    for index in node.first as usize..node.last as usize {
                        let a = polygon[index];
                        let b = polygon[(index + 1) % polygon.len()];
                        if (a.y > point.y) != (b.y > point.y) {
                            let crossing = a.x + (point.y - a.y) / (b.y - a.y) * (b.x - a.x);
                            if crossing > point.x {
                                let parity = &mut odd[node.loop_index as usize];
                                *parity = !*parity;
                            }
                        }
                    }
                }
                Some((left, right)) => {
                    stack.push(right);
                    stack.push(left);
                }
            }
        }
    }
}

/// How a query reads a lane's chords: the generic scan's, or the wrapped
/// horizon's.
#[derive(Clone, Copy, PartialEq, Eq)]
enum ChordScan {
    /// Through the index on a face of [`CHORD_INDEX_MIN_CHORDS`] or more,
    /// unless `BREP_CONTAINMENT_INDEX=0`.
    Auto,
    /// Through the index whenever it can be built.
    Indexed,
    /// Every chord.
    Linear,
}

impl ChordScan {
    /// Whether a lane of `chords` chords is read through its index.
    fn indexes(self, chords: usize) -> bool {
        match self {
            ChordScan::Linear => false,
            ChordScan::Indexed => true,
            ChordScan::Auto => chord_index_enabled() && chords >= CHORD_INDEX_MIN_CHORDS,
        }
    }
}

/// Whether a query point is one the index is used for
/// ([`CHORD_INDEX_MAX_COORDINATE`]).
fn in_index_range(point: Vec2) -> bool {
    point.x.abs() <= CHORD_INDEX_MAX_COORDINATE && point.y.abs() <= CHORD_INDEX_MAX_COORDINATE
}

/// The generic scan over every chord: `(odd loops, boundary, nearest chord as
/// (loop, chord, distance))`. The reference the index answers as.
fn linear_scan(
    generic: &GenericPrep,
    point: Vec2,
    tolerance: f64,
) -> (usize, bool, Option<(usize, usize, f64)>) {
    let mut crossings = 0;
    let mut boundary = false;
    // `(loop index, chord index, distance)` of the nearest chord, scanned in
    // loop order then chord order so a tie resolves as it always has.
    let mut nearest: Option<(usize, usize, f64)> = None;
    for (loop_index, (samples, chords)) in generic.loops.iter().zip(&generic.segments).enumerate() {
        let polygon = &samples.polygon;
        match point_in_polygon(point, polygon, tolerance) {
            PolygonClass::Boundary => boundary = true,
            PolygonClass::Inside => crossings += 1,
            PolygonClass::Outside => {}
        }
        for index in 0..chords.len() {
            let start = polygon[index];
            let end = polygon[(index + 1) % polygon.len()];
            let distance = point_segment_distance(point, start, end);
            if nearest
                .as_ref()
                .is_none_or(|(_, _, nearest_distance)| distance < *nearest_distance)
            {
                nearest = Some((loop_index, index, distance));
            }
        }
    }
    (crossings, boundary, nearest)
}

/// The generic scan's `(odd loops, boundary, nearest chord)` for `point`, read
/// as `scan` says. When the index answers, `odd loops` is not counted on a
/// boundary point, where nothing reads it.
fn generic_scan(
    generic: &GenericPrep,
    point: Vec2,
    tolerance: f64,
    scan: ChordScan,
) -> (usize, bool, Option<(usize, usize, f64)>) {
    if scan.indexes(generic.chord_count) && in_index_range(point) {
        if let Some(index) = generic.index.get_or_init(|| ChordIndex::build(generic.loops.as_slice())) {
            let loops = generic.loops.as_slice();
            let nearest = index.nearest(loops, point);
            let boundary = nearest.is_some_and(|(_, _, distance)| distance <= tolerance);
            let crossings = if boundary {
                0
            } else {
                let mut odd = vec![false; loops.len()];
                index.flip_crossings(loops, point, &mut odd);
                odd.into_iter().filter(|&parity| parity).count()
            };
            return (crossings, boundary, nearest);
        }
    }
    linear_scan(generic, point, tolerance)
}

/// Everything [`parameter_point_in_face`] derives from the FACE alone, so a
/// face queried many times — the imprint clip asks once per marched sample,
/// the fragment builder and the classifier's rays once per crossing — samples
/// its trim once. Each lane is prepared the first time a query REACHES it, so
/// an error in a lane's sampling surfaces on exactly the faces and queries it
/// did when the lanes sampled inline.
struct PreparedFace {
    seam_band: std::cell::OnceCell<Result<SeamBandPrep, String>>,
    sphere_cap: std::cell::OnceCell<Result<Option<SphereCapPrep>, String>>,
    horizon: std::cell::OnceCell<Result<Option<HorizonPrep>, String>>,
    rim_strip: std::cell::OnceCell<Result<Option<RimStripPrep>, String>>,
    generic: std::cell::OnceCell<Result<GenericPrep, String>>,
    /// The spherical trim lane's region, built from `generic`'s samples.
    sphere_region: std::cell::OnceCell<Result<crate::sphere_chart::SphericalRegion, String>>,
}

impl PreparedFace {
    fn new() -> Self {
        Self {
            seam_band: std::cell::OnceCell::new(),
            sphere_cap: std::cell::OnceCell::new(),
            horizon: std::cell::OnceCell::new(),
            rim_strip: std::cell::OnceCell::new(),
            generic: std::cell::OnceCell::new(),
            sphere_region: std::cell::OnceCell::new(),
        }
    }
}

fn lane<'a, T>(
    cell: &'a std::cell::OnceCell<Result<T, String>>,
    build: impl FnOnce() -> Result<T, String>,
) -> Result<&'a T, String> {
    match cell.get_or_init(build) {
        Ok(value) => Ok(value),
        Err(error) => Err(error.clone()),
    }
}

/// The prepared faces this thread queried most recently, keyed by the EXACT
/// data the lanes read: the environment switches, the face sense, the surface's
/// degrees, knots and control points, and per loop each coedge's edge id,
/// direction, pcurve degree, knots and control points. A hit compares the whole
/// signature, value by value to the bit (and never a NaN); the hash only
/// chooses which entries to compare. It also holds the spherical lane's region
/// (`PreparedFace::sphere_region`, [`build_sphere_region`]). Sampling a
/// 16-coedge sphere trim at 2 600 chords per coedge four times per query was 1.5 s of
/// `anotherBooleanFail`'s imprint clip and most of its fragment selection, for
/// samples that never change between one query and the next.
///
/// The lookup is paid on every query, so it walks the face once against the
/// stored signature and copies nothing: the hash reads the signature's shape
/// and a few values, not all of it. Building, hashing and comparing the whole
/// signature per query was 0.68 ms on `gear-hex-bore-push`'s face 8307 (562
/// pcurves, 36 530 control points), which is all a query there costs once its
/// chords are indexed.
struct PreparedFaceCache {
    entries: Vec<(u64, Vec<f64>, std::rc::Rc<PreparedFace>)>,
}

const PREPARED_FACE_CACHE_ENTRIES: usize = 64;

thread_local! {
    static PREPARED_FACES: std::cell::RefCell<PreparedFaceCache> = const {
        std::cell::RefCell::new(PreparedFaceCache {
            entries: Vec::new(),
        })
    };
}

/// Walks a face's signature — `switches` ([`lane_switches`], read once by the
/// caller), then the face's data in a fixed order, every count before the
/// values it counts — handing each value to `visit` until it returns false.
/// Returns whether every value was visited.
fn walk_face_signature<F: FnMut(f64) -> bool>(face: &FaceRecord, switches: f64, visit: &mut F) -> bool {
    let surface = &face.surface;
    let point = |visit: &mut F, p: &crate::Vec4| visit(p.x) && visit(p.y) && visit(p.z) && visit(p.w);
    if !(visit(switches)
        && visit(if face.same_sense { 1.0 } else { 0.0 })
        && visit(surface.degree_u as f64)
        && visit(surface.degree_v as f64)
        && visit(surface.knots_u.len() as f64))
    {
        return false;
    }
    if !(surface.knots_u.iter().all(|&knot| visit(knot)) && visit(surface.knots_v.len() as f64)) {
        return false;
    }
    if !(surface.knots_v.iter().all(|&knot| visit(knot)) && visit(surface.control_points.len() as f64)) {
        return false;
    }
    for row in &surface.control_points {
        if !(visit(row.len() as f64) && row.iter().all(|p| point(visit, p))) {
            return false;
        }
    }
    if !visit(face.loops.len() as f64) {
        return false;
    }
    for loop_record in &face.loops {
        if !visit(loop_record.coedges.len() as f64) {
            return false;
        }
        for coedge in &loop_record.coedges {
            let curve = &coedge.pcurve;
            if !(visit(coedge.edge_id as f64)
                && visit(if coedge.forward { 1.0 } else { 0.0 })
                && visit(curve.degree as f64)
                && visit(curve.knots.len() as f64))
            {
                return false;
            }
            if !(curve.knots.iter().all(|&knot| visit(knot)) && visit(curve.control_points.len() as f64)) {
                return false;
            }
            if !curve.control_points.iter().all(|p| point(visit, p)) {
                return false;
            }
        }
    }
    true
}

/// The cache's hash of a face: its signature's every count, the switches and
/// sense, the surface's first and last control point, and each pcurve's
/// first and last knot and control point, each value by its bits. Two faces
/// whose signatures agree to the bit hash the same.
fn face_key_hash(face: &FaceRecord, switches: f64) -> u64 {
    use std::hash::Hasher;
    let mut hasher = rustc_hash::FxHasher::default();
    let mut write = |value: f64| hasher.write_u64(value.to_bits());
    let surface = &face.surface;
    write(switches);
    write(if face.same_sense { 1.0 } else { 0.0 });
    write(surface.degree_u as f64);
    write(surface.degree_v as f64);
    write(surface.knots_u.len() as f64);
    write(surface.knots_v.len() as f64);
    write(surface.control_points.len() as f64);
    let corners = [surface.control_points.first().and_then(|row| row.first()), surface.control_points.last().and_then(|row| row.last())];
    for p in corners.into_iter().flatten() {
        [p.x, p.y, p.z, p.w].into_iter().for_each(&mut write);
    }
    write(face.loops.len() as f64);
    for loop_record in &face.loops {
        write(loop_record.coedges.len() as f64);
        for coedge in &loop_record.coedges {
            let curve = &coedge.pcurve;
            write(coedge.edge_id as f64);
            write(if coedge.forward { 1.0 } else { 0.0 });
            write(curve.degree as f64);
            write(curve.knots.len() as f64);
            write(curve.control_points.len() as f64);
            for knot in [curve.knots.first(), curve.knots.last()].into_iter().flatten() {
                write(*knot);
            }
            for p in [curve.control_points.first(), curve.control_points.last()].into_iter().flatten() {
                [p.x, p.y, p.z, p.w].into_iter().for_each(&mut write);
            }
        }
    }
    hasher.finish()
}

/// Where `face` is in the cache's `entries`, compared exactly.
fn cached_entry(
    entries: &[(u64, Vec<f64>, std::rc::Rc<PreparedFace>)],
    face: &FaceRecord,
    switches: f64,
    hash: u64,
) -> Option<usize> {
    entries.iter().position(|(entry_hash, signature, _)| {
        if *entry_hash != hash {
            return false;
        }
        let mut at = 0;
        let walked = walk_face_signature(face, switches, &mut |value: f64| {
            let same = signature
                .get(at)
                .is_some_and(|stored| stored.to_bits() == value.to_bits() && !value.is_nan());
            at += 1;
            same
        });
        walked && at == signature.len()
    })
}

/// Empties this thread's prepared-face cache, spherical regions included. For
/// the boolean's operand-state instrument (`BREP_DEBUG_OPERAND_STATE=tls`) only.
pub(crate) fn forget_prepared_faces() {
    PREPARED_FACES.with(|cache| cache.borrow_mut().entries.clear());
}

/// `face`'s prepared data if a query on this thread prepared it and it is
/// still cached, without touching the cache: a caller that only wants the
/// lane ([`containment_lane`]) must not push out the faces queries are using.
fn cached_prepared_face(face: &FaceRecord) -> Option<std::rc::Rc<PreparedFace>> {
    PREPARED_FACES.with(|cache| {
        let PreparedFaceCache { entries } = &*cache.borrow();
        let switches = lane_switches();
        let hash = face_key_hash(face, switches);
        cached_entry(entries, face, switches, hash).map(|index| entries[index].2.clone())
    })
}

fn prepared_face(face: &FaceRecord) -> std::rc::Rc<PreparedFace> {
    PREPARED_FACES.with(|cache| {
        let PreparedFaceCache { entries } = &mut *cache.borrow_mut();
        let switches = lane_switches();
        let hash = face_key_hash(face, switches);
        if let Some(index) = cached_entry(entries, face, switches, hash) {
            if index != 0 {
                let entry = entries.remove(index);
                entries.insert(0, entry);
            }
            return entries[0].2.clone();
        }
        let mut signature = Vec::new();
        walk_face_signature(face, switches, &mut |value: f64| {
            signature.push(value);
            true
        });
        let prepared = std::rc::Rc::new(PreparedFace::new());
        entries.insert(0, (hash, signature, prepared.clone()));
        entries.truncate(PREPARED_FACE_CACHE_ENTRIES);
        prepared
    })
}

/// Point-in-face for a DOUBLY-periodic (torus) seam-band face, whose material
/// region the raw even-odd/tangent test below gets wrong: the constant-level
/// seam rim is a zero-area iso line and the wavy cut's own polygon encloses only
/// the thin strip it bounds, so a point in the band reads Outside (or flips on
/// the fragile nearest-tangent refinement). Reconstruct the band as one simple
/// (u,v) polygon — identical topology to the mass integrator and tessellator —
/// and test the point against it, so all three subsystems agree on the material
/// side. Returns None (keep the general path) for every other face.
fn prepare_seam_band(face: &FaceRecord) -> Result<SeamBandPrep, String> {
    if face.surface.closed_directions()? != (true, true) {
        return Ok(SeamBandPrep::NotApplicable);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    if crate::topology::doubly_periodic_has_only_collapsed_loops(face)? {
        return Ok(SeamBandPrep::AlwaysInside);
    }
    let u_span = (u1 - u0).abs().max(1e-30);
    let v_span = (v1 - v0).abs().max(1e-30);
    let mut loops_uv: Vec<Vec<[f64; 2]>> = Vec::with_capacity(face.loops.len());
    // Which face loop each retained entry came from: the collapsed punctures
    // dropped below make the two lists disagree, and the merged-band lane needs
    // the ORIGINAL coedges of the loop it is about to unwrap.
    let mut retained: Vec<usize> = Vec::with_capacity(face.loops.len());
    for (loop_index, loop_record) in face.loops.iter().enumerate() {
        let mut points: Vec<[f64; 2]> = Vec::new();
        for coedge in &loop_record.coedges {
            let [d0, d1] = coedge.pcurve.domain()?;
            let samples = 24;
            for k in 0..=samples {
                let t = trim_station(d0, d1, k, samples);
                let p = coedge.pcurve.evaluate(t)?;
                points.push([p.x, p.y]);
            }
        }
        let (mut umin, mut umax, mut vmin, mut vmax) = (
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
        );
        for p in &points {
            umin = umin.min(p[0]);
            umax = umax.max(p[0]);
            vmin = vmin.min(p[1]);
            vmax = vmax.max(p[1]);
        }
        // Mirror `mass_properties::biperiodic_band_range`: a torus has no
        // geometric pole, so a VERTEX_LOOP whose whole pcurve trace collapses
        // to one parameter point is a zero-area puncture, not a band boundary.
        //
        // This used to drop the puncture only on faces with MORE than two loops,
        // to leave "ordinary two-loop torus classification" alone. That gate was
        // wrong, and mass integration never had it: one merged seam-carrying band
        // plus one puncture IS a two-loop face, and keeping the puncture put it at
        // `loops_uv.len() == 2`, where the two-loop analyses both decline (a
        // collapsed loop never full-wraps) and the face fell through to the plain
        // even-odd — whose verdict across a seam-wrapped band is the COMPLEMENT.
        // Measured on `anotherBooleanFail`'s imported body: face 280 read its
        // material band as v ∈ [0.125, 0.875] where its puncture-free twin face
        // 166 reads v ∈ [0.875, 1] ∪ [0, 0.125], and a ray crossing there was
        // silently lost, so points OpenCASCADE calls interior classified `Out`.
        // A zero-area point removes no material from any face, so drop it
        // wherever it appears; the all-collapsed case (an untrimmed torus written
        // as a sole VERTEX_LOOP) is already answered above, so at least one real
        // loop always survives.
        if (umax - umin) <= 1e-3 * u_span && (vmax - vmin) <= 1e-3 * v_span {
            continue;
        }
        retained.push(loop_index);
        loops_uv.push(points);
    }
    // MERGED SEAM-CARRYING BAND (single loop): `insert_periodic_band_seam_edges`
    // fuses a wrapped band's two rims into ONE loop joined by seam columns, so
    // the two-loop paths below never see it, and the plain even-odd polygon
    // carries a period-magnitude hop at each rim→column junction (a phantom
    // diagonal across the domain) that misclassifies the whole middle zone.
    // Unwrap the loop onto the covering plane (`loop_seam_offsets`, the same
    // fold mass integration and tessellation use), where it IS a simple
    // polygon, and even-odd the query's period images against it. Gated on a
    // genuinely seam-crossing single loop on a doubly-periodic surface; every
    // other face falls through unchanged. Hatch: BREP_SEAM_BAND_MERGED=0.
    if loops_uv.len() == 1 && !switched_off(SEAM_BAND_MERGED_SWITCH) {
        let coedges = &face.loops[retained[0]].coedges;
        let offsets = crate::topology::loop_seam_offsets(coedges, true, true, u_span, v_span)?;
        if offsets.iter().any(|o| o[0] != 0.0 || o[1] != 0.0) {
            let mut polygon: Vec<Vec2> = Vec::new();
            for (coedge_index, coedge) in coedges.iter().enumerate() {
                let [d0, d1] = coedge.pcurve.domain()?;
                let samples = 24;
                for k in 0..samples {
                    let t = trim_station(d0, d1, k, samples);
                    let p = coedge.pcurve.evaluate(t)?;
                    polygon.push(Vec2 {
                        x: p.x + offsets[coedge_index][0],
                        y: p.y + offsets[coedge_index][1],
                    });
                }
            }
            return Ok(SeamBandPrep::Merged { polygon, u_span, v_span });
        }
    }
    if loops_uv.len() != 2 {
        return Ok(SeamBandPrep::Declined);
    }
    if let Some(band) = crate::topology::analyze_doubly_periodic_seam_band(
        &loops_uv,
        [u0, u1, v0, v1],
        face.same_sense,
    ) {
        let polygon: Vec<Vec2> =
            crate::topology::seam_band_uv_polygon(&loops_uv, [u0, u1, v0, v1], &band)
                .into_iter()
                .map(|p| Vec2 { x: p[0], y: p[1] })
                .collect();
        return Ok(SeamBandPrep::Analyzed { polygon });
    }
    // TWO clean full-wrap rims are the companion `biperiodic_band_range` case,
    // with or without an extra collapsed puncture. Classify only the cross
    // parameter: the material is either the strip between the rims or its
    // periodic complement. The exact two-loop/full-wrap/constant-cross-level
    // gates below mirror mass integration and tessellation; every other torus
    // face retains the general classifier.
    for p_is_u in [true, false] {
        let (period, q_extent) = if p_is_u {
            (u1 - u0, v_span)
        } else {
            (v1 - v0, u_span)
        };
        if !(period > 0.0) {
            continue;
        }
        let coord = |p: &[f64; 2]| if p_is_u { (p[0], p[1]) } else { (p[1], p[0]) };
        let mut rings = Vec::with_capacity(2);
        for points in &loops_uv {
            let (mut pmin, mut pmax, mut qmin, mut qmax) = (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            );
            for p in points {
                let (periodic, cross) = coord(p);
                pmin = pmin.min(periodic);
                pmax = pmax.max(periodic);
                qmin = qmin.min(cross);
                qmax = qmax.max(cross);
            }
            if (pmax - pmin) < 0.6 * period || (qmax - qmin) > 0.05 * q_extent {
                rings.clear();
                break;
            }
            let mut net = 0.0;
            for pair in points.windows(2) {
                let mut delta = coord(&pair[1]).0 - coord(&pair[0]).0;
                if delta > 0.5 * period {
                    delta -= period;
                } else if delta < -0.5 * period {
                    delta += period;
                }
                net += delta;
            }
            let direction = if net > 0.25 * period {
                1
            } else if net < -0.25 * period {
                -1
            } else {
                0
            };
            rings.push((0.5 * (qmin + qmax), direction));
        }
        if rings.len() != 2 {
            continue;
        }
        rings.sort_by(|a, b| a.0.total_cmp(&b.0));
        let [(q_lo, lower_direction), (q_hi, upper_direction)] = rings.as_slice() else {
            unreachable!()
        };
        let inconclusive =
            *lower_direction == 0 || *upper_direction == 0 || lower_direction == upper_direction;
        let between_is_ccw_uv = if p_is_u {
            *lower_direction > 0
        } else {
            *lower_direction < 0
        };
        let complement = !inconclusive && between_is_ccw_uv != face.same_sense;
        return Ok(SeamBandPrep::Rings {
            p_is_u,
            q_lo: *q_lo,
            q_hi: *q_hi,
            complement,
        });
    }
    Ok(SeamBandPrep::Declined)
}

fn seam_band_point_in_face(
    prepared: &SeamBandPrep,
    point: Vec2,
    tolerance: f64,
) -> Option<PolygonClass> {
    match prepared {
        SeamBandPrep::NotApplicable | SeamBandPrep::Declined => None,
        SeamBandPrep::AlwaysInside => Some(PolygonClass::Inside),
        SeamBandPrep::Merged { polygon, u_span, v_span } => {
            let mut best = PolygonClass::Outside;
            'images: for du in [-1.0, 0.0, 1.0] {
                for dv in [-1.0, 0.0, 1.0] {
                    let image = Vec2 {
                        x: point.x + du * u_span,
                        y: point.y + dv * v_span,
                    };
                    match point_in_polygon(image, polygon, tolerance) {
                        PolygonClass::Boundary => {
                            best = PolygonClass::Boundary;
                            break 'images;
                        }
                        PolygonClass::Inside => best = PolygonClass::Inside,
                        PolygonClass::Outside => {}
                    }
                }
            }
            Some(best)
        }
        SeamBandPrep::Analyzed { polygon } => Some(point_in_polygon(point, polygon, tolerance)),
        SeamBandPrep::Rings { p_is_u, q_lo, q_hi, complement } => {
            let q = if *p_is_u { point.y } else { point.x };
            if (q - q_lo).abs() <= tolerance || (q - q_hi).abs() <= tolerance {
                return Some(PolygonClass::Boundary);
            }
            let between = q > *q_lo && q < *q_hi;
            Some(if between != *complement {
                PolygonClass::Inside
            } else {
                PolygonClass::Outside
            })
        }
    }
}

/// Point-in-face for a SINGLY-PERIODIC face with a SEAM-STRADDLING loop: a
/// trim loop crossing the u seam is stored with its samples folded into the
/// domain, so its flat polygon is TORN at the seam and plain even-odd
/// misclassifies near the tear (STEP 00000585-family: face 519's big inner
/// loop straddles the seam and a REAL section sub-segment between two accepted
/// pieces read Outside in `process_curve`'s midpoint gate — trial 174's
/// missing middle segment). Mirror fragment.rs's proven seam-straddling
/// fallback: UNWRAP every loop by accumulating period-folded deltas (a
/// zero-winding loop closes in the covering plane), then even-odd the query at
/// its −P/0/+P period images across all unwrapped loops. Fires only when the
/// face is closed in exactly u, has >2 loops, and at least one loop straddles
/// the seam (≥1 fold jump); every other face returns None and keeps the plain
/// classifier bit-identically. Loops that WIND the period (net winding ≠ 0)
/// do not close under unwrapping — bail to the plain path rather than guess.
/// Escape hatch `BREP_HORIZON_CONTAINMENT=0`.
fn prepare_wrapped_horizon(face: &FaceRecord) -> Result<Option<HorizonPrep>, String> {
    if face.surface.closed_directions()? != (true, false) || face.loops.is_empty() {
        return Ok(None);
    }
    if switched_off(HORIZON_CONTAINMENT_SWITCH) {
        return Ok(None);
    }
    // SINGLE/DOUBLE-LOOP straddlers (t91's face 675: ONE loop drawn in-domain
    // that hops the u-seam twice, trimming a sliver strip AT the seam) used
    // to be excluded by a `loops.len() <= 2` gate and fell to the plain
    // even-odd, whose verdict on the hopped polygon is INVERTED (the strip
    // interior read Outside, far azimuths read Inside) — minting bogus
    // far-azimuth section pieces and discarding the real ones. The unwrap +
    // period-image even-odd below is loop-count-agnostic, so the gate now
    // only requires a non-empty loop set; non-straddling loops still bail to
    // the plain path unchanged. Escape hatch: BREP_HORIZON_SINGLE_LOOP=0
    // restores the old minimum.
    if face.loops.len() <= 2 && switched_off(HORIZON_SINGLE_LOOP_SWITCH) {
        return Ok(None);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let u_period = u1 - u0;
    if u_period <= 0.0 {
        return Ok(None);
    }
    let debug = std::env::var("BREP_DEBUG_HORIZON").is_ok();
    let mut loops: Vec<Vec<Vec2>> = Vec::new();
    let mut straddling = false;
    for loop_record in &face.loops {
        let mut points: Vec<Vec2> = Vec::new();
        for coedge in &loop_record.coedges {
            let curve = &coedge.pcurve;
            let [start, end] = curve.domain()?;
            let sample_count = trim_sample_count(curve);
            for index in 0..sample_count {
                let parameter = trim_station(start, end, index, sample_count);
                let evaluated = curve.evaluate(parameter)?;
                points.push(Vec2 {
                    x: evaluated.x,
                    y: evaluated.y,
                });
            }
        }
        if points.len() < 3 {
            continue;
        }
        // Unwrap into the covering plane: place each sample at the previous
        // one plus the period-folded delta. A seam-straddling loop becomes a
        // continuous closed polygon; a WINDING loop does not close — bail.
        let mut unwrapped = Vec::with_capacity(points.len());
        let mut jumps = 0usize;
        let mut cursor = points[0];
        unwrapped.push(cursor);
        for pair in points.windows(2) {
            let mut du = pair[1].x - pair[0].x;
            let folded = du - u_period * (du / u_period).round();
            if (du - folded).abs() > 0.25 * u_period {
                jumps += 1;
            }
            du = folded;
            cursor = Vec2 {
                x: cursor.x + du,
                y: pair[1].y,
            };
            unwrapped.push(cursor);
        }
        let closure = (unwrapped[0].x - unwrapped[unwrapped.len() - 1].x).abs();
        if closure > 0.25 * u_period {
            if debug {
                eprintln!(
                    "horizon: face {} loop winds the period (closure {closure:.3}) — bail",
                    face.id
                );
            }
            return Ok(None); // winding loop: unwrapping cannot close it
        }
        if jumps > 0 {
            straddling = true;
        }
        loops.push(unwrapped);
    }
    if !straddling || loops.is_empty() {
        return Ok(None);
    }
    Ok(Some(HorizonPrep {
        loops,
        u_period,
        index: std::cell::OnceCell::new(),
    }))
}

fn wrapped_horizon_point_in_face(
    face: &FaceRecord,
    prepared: &HorizonPrep,
    point: Vec2,
    tolerance: f64,
    scan: ChordScan,
) -> Option<PolygonClass> {
    let HorizonPrep { loops, u_period, index } = prepared;
    let u_period = *u_period;
    let debug = std::env::var("BREP_DEBUG_HORIZON").is_ok();
    // Even-odd on the QUOTIENT cylinder: each unwrapped loop is tested at
    // every period image of the query and contributes its own parity. The old
    // combining ("any single image with an odd crossing count ⇒ Inside") is
    // wrong whenever two loops live in DIFFERENT period frames after
    // unwrapping — t181/00000312#p7: face 519's seam-straddling HOLE unwraps
    // to u∈[−0.039, 0.039] while the outer rectangle spans [0, 1], so a query
    // inside the hole at u≈0.986 is contained by the outer loop at shift 0
    // (odd ⇒ Inside) and by the hole at shift −P (odd ⇒ Inside again); the
    // true even count (outer + hole = 2 ⇒ Outside) is never assembled at any
    // single image. Fragment selection then kept a PHANTOM half-hole region
    // whose boundary (half the hole loop + a derived chord) minted same-sense
    // coedges at assembly. Summing per-loop parities across images restores
    // the covering-space even-odd. Escape hatch: BREP_HORIZON_CROSS_FRAME=0
    // restores the old per-image combining.
    if !horizon_cross_frame() {
        let mut best: Option<PolygonClass> = None;
        for shift in [-u_period, 0.0, u_period] {
            let image = Vec2 {
                x: point.x + shift,
                y: point.y,
            };
            let mut crossings = 0usize;
            for polygon in loops {
                match point_in_polygon(image, polygon, tolerance) {
                    PolygonClass::Boundary => return Some(PolygonClass::Boundary),
                    PolygonClass::Inside => crossings += 1,
                    PolygonClass::Outside => {}
                }
            }
            if crossings % 2 == 1 {
                best = Some(PolygonClass::Inside);
            } else if best.is_none() {
                best = Some(PolygonClass::Outside);
            }
        }
        if debug {
            eprintln!(
                "horizon: face {} loops={} straddling (legacy per-image) -> {:?}",
                face.id,
                loops.len(),
                best
            );
        }
        return best;
    }
    let images = [-u_period, 0.0, u_period].map(|shift| Vec2 {
        x: point.x + shift,
        y: point.y,
    });
    let chords: usize = loops.iter().map(Vec::len).sum();
    let indexed = (scan.indexes(chords) && images.iter().all(|image| in_index_range(*image)))
        .then(|| index.get_or_init(|| ChordIndex::build(loops.as_slice())).as_ref())
        .flatten();
    let mut crossings = 0usize;
    if let Some(index) = indexed {
        // The per-loop parities summed over the images, as the scan below
        // sums them: `Boundary` if any image is within the band of any chord,
        // otherwise each loop's crossings flipped across all three images.
        let loops = loops.as_slice();
        if images.iter().any(|image| index.any_within(loops, *image, tolerance)) {
            return Some(PolygonClass::Boundary);
        }
        let mut odd = vec![false; loops.len()];
        for image in images {
            index.flip_crossings(loops, image, &mut odd);
        }
        crossings = odd.into_iter().filter(|&parity| parity).count();
    } else {
        for polygon in loops {
            let mut image_hits = 0usize;
            for image in images {
                match point_in_polygon(image, polygon, tolerance) {
                    PolygonClass::Boundary => return Some(PolygonClass::Boundary),
                    PolygonClass::Inside => image_hits += 1,
                    PolygonClass::Outside => {}
                }
            }
            crossings += image_hits % 2;
        }
    }
    let class = if crossings % 2 == 1 {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    };
    if debug {
        eprintln!(
            "horizon: face {} loops={} straddling -> {:?}",
            face.id,
            loops.len(),
            class
        );
    }
    Some(class)
}

/// Classify a spherical cap represented by a varying full-wrap contact rim and
/// the oppositely wound collapsed rim at one sphere pole.  This is the natural
/// topology when a circle about an oblique axis encloses a pole of the sphere's
/// stored parameter frame.
fn prepare_winding_sphere_cap(face: &FaceRecord) -> Result<Option<SphereCapPrep>, String> {
    if !matches!(
        face.surface.analytic(),
        Some(crate::AnalyticSurface::Sphere { .. })
    ) || face.surface.closed_directions()? != (true, false)
        || face.loops.len() != 2
    {
        return Ok(None);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let period = u1 - u0;
    let v_span = v1 - v0;
    if !(period > 0.0 && v_span > 0.0) {
        return Ok(None);
    }
    struct WindingLoop {
        points: Vec<Vec2>,
        winding: f64,
        vmin: f64,
        vmax: f64,
    }
    let mut loops = Vec::with_capacity(2);
    for loop_record in &face.loops {
        let mut points = Vec::new();
        for coedge in &loop_record.coedges {
            let [start, end] = coedge.pcurve.domain()?;
            let samples = trim_sample_count(&coedge.pcurve);
            for index in 0..samples {
                let parameter = trim_station(start, end, index, samples);
                let p = coedge.pcurve.evaluate(parameter)?;
                points.push(Vec2 { x: p.x, y: p.y });
            }
        }
        if points.len() < 2 {
            return Ok(None);
        }
        let first = points[0];
        let mut cursor = first;
        let mut unwrapped = vec![cursor];
        for next in points.iter().skip(1) {
            let du = next.x - cursor.x;
            let folded = du - period * (du / period).round();
            cursor = Vec2 {
                x: cursor.x + folded,
                y: next.y,
            };
            unwrapped.push(cursor);
        }
        let last_raw = *points.last().unwrap();
        let closing_du = first.x - last_raw.x;
        let closing_folded = closing_du - period * (closing_du / period).round();
        let winding = cursor.x + closing_folded - first.x;
        let (mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &unwrapped {
            vmin = vmin.min(p.y);
            vmax = vmax.max(p.y);
        }
        loops.push(WindingLoop {
            points: unwrapped,
            winding,
            vmin,
            vmax,
        });
    }
    if loops
        .iter()
        .any(|loop_data| (loop_data.winding.abs() - period).abs() > 0.05 * period)
        || loops[0].winding * loops[1].winding >= 0.0
    {
        return Ok(None);
    }
    let flat = |loop_data: &WindingLoop| loop_data.vmax - loop_data.vmin <= 1e-6 * v_span;
    let (pole, rim) = match (flat(&loops[0]), flat(&loops[1])) {
        (true, false) => (&loops[0], &loops[1]),
        (false, true) => (&loops[1], &loops[0]),
        _ => return Ok(None),
    };
    let pole_v = 0.5 * (pole.vmin + pole.vmax);
    if (pole_v - v0).abs() > 1e-6 * v_span && (pole_v - v1).abs() > 1e-6 * v_span {
        return Ok(None);
    }
    Ok(Some(SphereCapPrep {
        rim: rim.points.clone(),
        pole_v,
        period,
    }))
}

fn winding_sphere_cap_point_in_face(
    prepared: &SphereCapPrep,
    point: Vec2,
    tolerance: f64,
) -> Option<PolygonClass> {
    let SphereCapPrep { rim, pole_v, period } = prepared;
    let (pole_v, period) = (*pole_v, *period);
    let window_start = rim[0].x.min(rim[rim.len() - 1].x);
    let query_u = window_start + (point.x - window_start).rem_euclid(period);
    let mut crossings = Vec::new();
    for pair in rim.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if (a.x > query_u) != (b.x > query_u) {
            crossings.push(a.y + (query_u - a.x) / (b.x - a.x) * (b.y - a.y));
        }
        for image in [-period, 0.0, period] {
            if point_segment_distance(
                Vec2 {
                    x: point.x + image,
                    y: point.y,
                },
                a,
                b,
            ) <= tolerance
            {
                return Some(PolygonClass::Boundary);
            }
        }
    }
    let rim_v = crossings
        .into_iter()
        .min_by(|a, b| (a - point.y).abs().total_cmp(&(b - point.y).abs()))?;
    let inside = if pole_v < rim_v {
        point.y <= rim_v + tolerance
    } else {
        point.y >= rim_v - tolerance
    };
    Some(if inside {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    })
}

/// Point-in-face for a SINGLY-PERIODIC (closed-u) FULL-BAND STRIP whose two
/// full-wrap rim loops are drawn as COVERING-PLANE pcurves running OUT OF the
/// u-domain by up to a whole period (case 08:10-genus face 1513, the 4th seam
/// representation after helmet-unwrapped / band-complement / 675-hops): the
/// raw even-odd sees u-winding loop polygons whose parity is meaningless, so
/// nearly the whole material strip reads Outside, the marched clip crumbles
/// the section curves to sub-mm bits, and the neighbours strand one-use.
///
/// The material region is the strip BETWEEN the two rims: on an open-v
/// surface a 2-loop face whose loops both wrap the full period has no other
/// representable region. The verdict is a sampled per-u hull — each rim is a
/// single periodic polyline v = rim(u) in the covering plane, and the query
/// point is between the rims iff an upward (+v) vertical ray at the query's
/// u (folded modulo the period into each rim's own one-period window) crosses
/// the two rim polylines an odd number of times in total. NO flat-level
/// approximation is used (the rims are scalloped: 1513's upper rim is flat at
/// v=1 on parts of u but dips to v=0.745 elsewhere — a v-between-extremes
/// rule is provably wrong on it).
///
/// STRUCTURAL gates (all must hold; anything else returns None and keeps the
/// previous classification bit-identically):
///   - surface closed in exactly u; exactly 2 loops;
///   - at least one loop's samples run OUT of the u-domain by >1e-3 period
///     (the covering-plane discriminator — in-domain 2-rim bands keep their
///     current path);
///   - each loop's unwrapped net u-travel is exactly ±one period and its v
///     closes (a true full-wrap rim, not a partial arc);
///   - opposite travel directions, disjoint v-hulls, and the material-left
///     winding rule confirming the BETWEEN strip (a complement verdict cannot
///     be represented on an open-v surface — decline, never guess).
/// Escape hatch: BREP_COVERING_RIM_STRIP=0.
fn prepare_covering_rim_strip(face: &FaceRecord) -> Result<Option<RimStripPrep>, String> {
    if face.surface.closed_directions()? != (true, false) || face.loops.len() != 2 {
        return Ok(None);
    }
    if switched_off(COVERING_RIM_STRIP_SWITCH) {
        return Ok(None);
    }
    let [u0, u1] = face.surface.domain_u()?;
    let [v0, v1] = face.surface.domain_v()?;
    let period = u1 - u0;
    if !(period > 0.0) {
        return Ok(None);
    }
    let v_span = (v1 - v0).abs().max(1e-30);
    struct Rim {
        points: Vec<Vec2>,
        vmin: f64,
        vmax: f64,
        ascending: bool,
    }
    let mut rims: Vec<Rim> = Vec::with_capacity(2);
    let mut out_of_domain = false;
    for loop_record in &face.loops {
        let mut points: Vec<Vec2> = Vec::new();
        for coedge in &loop_record.coedges {
            let curve = &coedge.pcurve;
            let [start, end] = curve.domain()?;
            let sample_count = trim_sample_count(curve);
            for index in 0..=sample_count {
                let parameter = trim_station(start, end, index, sample_count);
                let evaluated = curve.evaluate(parameter)?;
                points.push(Vec2 {
                    x: evaluated.x,
                    y: evaluated.y,
                });
            }
        }
        if points.len() < 3 {
            return Ok(None);
        }
        // Covering-plane discriminator: this lane exists for loops drawn PAST
        // the domain; everything in-domain keeps the existing classifiers.
        for p in &points {
            if p.x < u0 - 1e-3 * period || p.x > u1 + 1e-3 * period {
                out_of_domain = true;
            }
        }
        // Unwrap into the covering plane (period-folded deltas — a no-op for
        // an already-continuous covering-plane representation, and the same
        // fold the horizon lane uses for in-domain seam hops).
        let mut unwrapped: Vec<Vec2> = Vec::with_capacity(points.len());
        let mut cursor = points[0];
        unwrapped.push(cursor);
        for pair in points.windows(2) {
            let du = pair[1].x - pair[0].x;
            let folded = du - period * (du / period).round();
            cursor = Vec2 {
                x: cursor.x + folded,
                y: pair[1].y,
            };
            unwrapped.push(cursor);
        }
        let net = unwrapped[unwrapped.len() - 1].x - unwrapped[0].x;
        if (net.abs() - period).abs() > 2e-2 * period {
            return Ok(None); // not a clean single full wrap
        }
        if (unwrapped[unwrapped.len() - 1].y - unwrapped[0].y).abs() > 1e-3 * v_span {
            return Ok(None); // rim does not close in v
        }
        let (mut vmin, mut vmax) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &unwrapped {
            vmin = vmin.min(p.y);
            vmax = vmax.max(p.y);
        }
        rims.push(Rim {
            points: unwrapped,
            vmin,
            vmax,
            ascending: net > 0.0,
        });
    }
    if !out_of_domain {
        return Ok(None);
    }
    if rims[0].ascending == rims[1].ascending {
        return Ok(None); // rims must traverse opposite u directions
    }
    let (lower, upper) = if rims[0].vmax <= rims[1].vmin {
        (&rims[0], &rims[1])
    } else if rims[1].vmax <= rims[0].vmin {
        (&rims[1], &rims[0])
    } else {
        return Ok(None); // interleaved v-hulls: not a clean strip
    };
    if upper.vmin - lower.vmax <= 1e-6 * v_span {
        return Ok(None);
    }
    // Material-left rule (same convention as the doubly-periodic band lanes):
    // the between strip is CCW in uv iff the LOWER rim travels +u; that must
    // match the face sense, else the material would be the complement — which
    // an open-v surface cannot represent. Decline rather than guess.
    if (lower.ascending) != face.same_sense {
        return Ok(None);
    }
    Ok(Some(RimStripPrep {
        lower: lower.points.clone(),
        upper: upper.points.clone(),
        period,
    }))
}

fn covering_rim_strip_point_in_face(
    prepared: &RimStripPrep,
    point: Vec2,
    tolerance: f64,
) -> Option<PolygonClass> {
    let RimStripPrep { lower, upper, period } = prepared;
    let period = *period;
    // Boundary: proximity to either rim polyline at the query's period images.
    for rim in [lower, upper] {
        for shift in [-period, 0.0, period] {
            let image = Vec2 {
                x: point.x + shift,
                y: point.y,
            };
            for pair in rim.windows(2) {
                if point_segment_distance(image, pair[0], pair[1]) <= tolerance {
                    return Some(PolygonClass::Boundary);
                }
            }
        }
    }
    // Sampled per-u hull: fold the query into each rim's own one-period
    // window and count upward-ray crossings; odd total = between the rims.
    let mut crossings = 0usize;
    for rim in [lower, upper] {
        let window_base = rim[0].x.min(rim[rim.len() - 1].x);
        let x = window_base + (point.x - window_base).rem_euclid(period);
        for pair in rim.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            if (a.x > x) != (b.x > x) {
                let v_cross = a.y + (x - a.x) / (b.x - a.x) * (b.y - a.y);
                if v_cross > point.y {
                    crossings += 1;
                }
            }
        }
    }
    Some(if crossings % 2 == 1 {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    })
}

/// The generic scan's trim samples: every pcurve at `(interior knots + 1) ×
/// (degree + 1) × 4` chords, the closed polygon per loop, and the chord
/// parameters behind every polygon edge.
fn prepare_generic(face: &FaceRecord) -> Result<GenericPrep, String> {
    let mut loops: Vec<LoopSamples> = Vec::with_capacity(face.loops.len());
    let mut segments: Vec<Vec<SegmentPrep>> = Vec::with_capacity(face.loops.len());
    let mut chord_count = 0;
    for loop_record in &face.loops {
        let mut polygon = Vec::new();
        let mut chords = Vec::new();
        let mut coedges = Vec::with_capacity(loop_record.coedges.len());
        for (coedge_index, coedge) in loop_record.coedges.iter().enumerate() {
            let first = polygon.len();
            let curve = &coedge.pcurve;
            let [start, end] = curve.domain()?;
            let sample_count = trim_sample_count(curve);
            for index in 0..sample_count {
                let parameter = trim_station(start, end, index, sample_count);
                let evaluated = curve.evaluate(parameter)?;
                polygon.push(Vec2 {
                    x: evaluated.x,
                    y: evaluated.y,
                });
                let parameter_end = if index + 1 < sample_count {
                    trim_station(start, end, index + 1, sample_count)
                } else {
                    end
                };
                chords.push(SegmentPrep {
                    coedge_index,
                    parameter_start: parameter,
                    parameter_end,
                });
            }
            coedges.push((coedge.edge_id, first, polygon.len() - first));
        }
        chord_count += chords.len();
        loops.push(LoopSamples { polygon, coedges });
        segments.push(chords);
    }
    Ok(GenericPrep {
        loops,
        segments,
        chord_count,
        index: std::cell::OnceCell::new(),
    })
}

/// One trim loop as [`parameter_point_in_face`] already sampled it: the closed
/// polygon it counts parity against, plus `(edge id, first sample, sample count)`
/// per coedge in loop order, which is all the spherical lane needs to drop a slit
/// and to rebuild the boundary as SEGMENTS rather than as one ring.
struct LoopSamples {
    polygon: Vec<Vec2>,
    coedges: Vec<(u64, usize, usize)>,
}

/// Point-in-face for a SPHERICAL carrier, decided on the ball itself instead of
/// in its polar parameter domain — but ONLY in the far field, where the generic
/// path has nothing better than a parity count.
///
/// The generic classifier below does not really answer by parity.  Whenever the
/// query has a nearest trim segment whose foot is interior to that pcurve, it
/// answers by which SIDE of that curve the point is on, which is locally exact
/// and is what the imprint and the fragment builder are tuned against.  Parity is
/// its fallback for points further from every trim segment than that segment is
/// long — and parity is exactly what the polar domain breaks: a loop enclosing a
/// pole does not enclose it in `uv` (it wraps the domain instead), and a region
/// straddling the seam is not one polygon there at all.  Three separate special
/// cases in this file exist because of that.
///
/// So this lane engages on the complement of the refinement's condition: it
/// replaces the parity count and nothing else.  Material is the intersection of
/// the regions to the LEFT of each trim loop, and the side a point falls on is
/// the sign of the signed solid angle that loop subtends there — a question with
/// no parameter domain in it, so pole and seam need no mention.  A collapsed pole
/// loop has no area and a seam is traversed once each way; both cancel exactly.
///
/// [`crate::sphere_chart::SphericalRegion`] is the same classifier the chart
/// tessellation uses, so the trim query and the mesh cannot disagree about where
/// the material is.
fn sphere_chart_point_in_face(
    face: &FaceRecord,
    point: Vec2,
    prepared: &PreparedFace,
    sampled: &[LoopSamples],
) -> Result<Option<PolygonClass>, String> {
    let Some(atlas) = sphere_chart_atlas(face) else {
        return Ok(None);
    };
    let region = lane(&prepared.sphere_region, || build_sphere_region(face, &atlas, sampled))?;
    if region.is_whole_sphere() {
        return Ok(Some(PolygonClass::Inside));
    }
    if !region.is_decidable() {
        // No seed: decline to the generic path rather than answer Inside for
        // the whole ball.
        return Ok(None);
    }
    let probe = face.surface.evaluate(point.x, point.y)?;
    Ok(Some(if region.contains(atlas.centre, probe) {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    }))
}

/// The spherical region [`sphere_chart_point_in_face`] reads `face` against,
/// built ONCE per prepared face.
///
/// A region depends on the face's sphere, its orientation and its trim samples
/// — never on the query point — but the trim query is called once PER POINT
/// from the innermost loops of the imprint, the fragment builder and the
/// classifier's ray casts, and building one costs a surface evaluation per
/// boundary sample, a 27-cell canonicalization pass, two hash maps and a seed
/// search that can run two dozen crossing counts. Rebuilding that per query is
/// the whole cost of this lane; the query itself is one crossing count.
///
/// It lives on the [`PreparedFace`], whose cache key already holds everything
/// the region is built from. It used to have a four-entry cache of its own, and
/// a ray cast visits every face its segment's box touches in the same order
/// every time: a ball cut into seven faces by a helical tube (the 2026-09-26
/// coil fillet report) cycled that cache and missed on every query, 20-35 ms a
/// trim test, and the fillet's interference check ran 147 s instead of 30.
fn build_sphere_region(
    face: &FaceRecord,
    atlas: &crate::sphere_chart::SphereAtlas,
    sampled: &[LoopSamples],
) -> Result<crate::sphere_chart::SphericalRegion, String> {
    // An edge used TWICE by one face is a slit — the parametric seam of a ball
    // the trim never cut. It bounds no material, so it is left out of the region
    // rather than cancelled numerically afterwards.
    let mut uses: std::collections::HashMap<u64, usize> = std::collections::HashMap::new();
    for coedge in face.loops.iter().flat_map(|record| record.coedges.iter()) {
        *uses.entry(coedge.edge_id).or_insert(0) += 1;
    }
    // Boundary SEGMENTS, not one polyline per loop.
    //
    // A ball drilled through carries BOTH rims in a single loop, joined by the
    // seam traversed once each way — a keyhole. Concatenating that loop's
    // surviving coedges and closing the ring would bridge rim to rim with two
    // chords that are not reverses of each other, and the winding of that figure
    // is not the winding of the two rims: on a through-drilled ball it comes out
    // exactly inverted. Handing the classifier the segments themselves lets it
    // cancel the seam and recover the two real rims.
    //
    // The samples are the CALLER's, taken once for its own parity count and its
    // own nearest-segment scan. Sampling the pcurves again here doubled the cost
    // of every query on a spherical face, and the caller only reaches this lane
    // when it would otherwise answer by parity — so that second pass was paid on
    // nearly every query and used on almost none of them.
    //
    // They are sampled in each coedge's own traversal direction, which is the
    // direction its pcurve is parameterized in: `forward` selects which end of
    // the EDGE's samples a traversal starts from, it does not reverse the
    // pcurve. A loop walked backwards puts the material on the wrong side of
    // every boundary.
    // The material-left convention is stated against the FACE normal, which the
    // generic path encodes as `(cross > 0) == same_sense` in uv: material is left
    // of the boundary seen from `same_sense ? Su x Sv : -(Su x Sv)`.
    let outward_face_normal =
        face.same_sense == atlas.parameterization_is_outward(&face.surface)?;
    let mut points: Vec<Vec3> = Vec::new();
    let mut spans: Vec<(usize, usize)> = Vec::new();
    for samples in sampled {
        let count = samples.polygon.len();
        if count < 2 {
            continue;
        }
        for &(edge_id, first, span) in &samples.coedges {
            if span == 0 || uses.get(&edge_id).copied().unwrap_or(0) >= 2 {
                continue;
            }
            // `span + 1` points: the coedge's own samples plus the first
            // sample of the next coedge, which is this one's END point. The
            // loop is closed, so the last coedge wraps back to `polygon[0]`.
            let start = points.len();
            for offset in 0..=span {
                let uv = samples.polygon[(first + offset) % count];
                points.push(face.surface.evaluate(uv.x, uv.y)?);
            }
            spans.push((start, points.len()));
        }
    }
    // One canonicalization over ALL the face's points: a vertex named by
    // two coedges is evaluated twice and the two answers agree only to
    // rounding, which the exact-reverse cancellation and the loop walk
    // both need collapsed.
    crate::sphere_chart::canonicalize_points(&mut points, 1e-9);
    let mut boundary: Vec<(Vec3, Vec3)> = Vec::new();
    for (first, last) in spans {
        for index in first..last.saturating_sub(1) {
            boundary.push((points[index], points[index + 1]));
        }
    }
    Ok(crate::sphere_chart::SphericalRegion::from_segments(
        atlas.centre,
        &boundary,
        outward_face_normal,
    ))
}


/// The sphere the sphere-chart lane reads a face on: an analytic sphere, with
/// the lane's switches on ([`sphere_chart_trim_enabled`]).
fn sphere_chart_atlas(face: &FaceRecord) -> Option<crate::sphere_chart::SphereAtlas> {
    if !sphere_chart_trim_enabled() {
        return None;
    }
    crate::sphere_chart::SphereAtlas::of_surface(&face.surface)
}

/// Whether `BREP_DEBUG_CONTAINMENT` is set, read once.  `parameter_point_in_face`
/// runs per ray-face intersection inside every boolean, so the cost when the
/// trace is off must be one `OnceLock` read, not an environment lookup per exit.
fn containment_trace_enabled() -> bool {
    static ON: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *ON.get_or_init(|| std::env::var("BREP_DEBUG_CONTAINMENT").is_ok())
}

/// Name the lane that answered, under `BREP_DEBUG_CONTAINMENT`.  This function
/// has seven exits and which one fired is the first thing any investigation of a
/// wrong containment verdict needs; reconstructing it from the outside means
/// re-deriving each lane's gate by hand.
fn trace_lane(face: &FaceRecord, point: Vec2, lane: &str, class: PolygonClass) {
    if containment_trace_enabled() {
        eprintln!(
            "containment face={} uv=({:.8},{:.8}) lane={lane} -> {class:?}",
            face.id, point.x, point.y
        );
    }
}


/// A lane [`parameter_point_in_face`] answers in, with what it prepared.
enum Lane<'a> {
    SeamBand(&'a SeamBandPrep),
    SphereCap(&'a SphereCapPrep),
    Horizon(&'a HorizonPrep),
    RimStrip(&'a RimStripPrep),
    Generic,
}

/// The first lane at dispatch position `from` or later that is prepared to
/// answer `face`, and its position: 0 the seam band, 1 the sphere cap, 2 the
/// wrapped horizon, 3 the covering rim strip, 4 the generic scan. Each lane is
/// prepared (and its preparation's error surfaces) only when the walk reaches
/// it. [`parameter_point_in_face`] walks on from the next position when a lane
/// declines a query; [`containment_lane`] reports where the walk starts, so the
/// two cannot disagree about which lane a face is in.
fn lane_from<'a>(
    face: &FaceRecord,
    prepared: &'a PreparedFace,
    from: usize,
) -> Result<(usize, Lane<'a>), String> {
    if from == 0 {
        let band = lane(&prepared.seam_band, || prepare_seam_band(face))?;
        if !matches!(band, SeamBandPrep::NotApplicable | SeamBandPrep::Declined) {
            return Ok((0, Lane::SeamBand(band)));
        }
    }
    if from <= 1 {
        if let Some(cap) = lane(&prepared.sphere_cap, || prepare_winding_sphere_cap(face))? {
            return Ok((1, Lane::SphereCap(cap)));
        }
    }
    if from <= 2 {
        if let Some(horizon) = lane(&prepared.horizon, || prepare_wrapped_horizon(face))? {
            return Ok((2, Lane::Horizon(horizon)));
        }
    }
    if from <= 3 {
        if let Some(strip) = lane(&prepared.rim_strip, || prepare_covering_rim_strip(face))? {
            return Ok((3, Lane::RimStrip(strip)));
        }
    }
    Ok((4, Lane::Generic))
}

/// Which lane of [`parameter_point_in_face`] a face's questions go to, read off
/// the face alone — the kernel's own lane selection, through the same prepared
/// data the query uses, for a caller that answers some containment questions
/// ahead of the kernel and must know which rule the kernel applies.
///
/// The lanes are tried in this order, and a face is in the first whose gates it
/// passes (each comparand is in the carrier's parameters unless stated):
///
/// 1. [`SeamBand`](Self::SeamBand): `closed_directions() == (true, true)`, and
///    the seam-band preparation reads the face — every loop a collapsed
///    puncture (its trace within `1e-3` of the domain span in u and in v), one
///    seam-crossing loop unwrapped (unless [`SEAM_BAND_MERGED_SWITCH`]), two
///    loops the seam-band analysis reassembles, or two clean full-wrap rims (a
///    periodic extent of at least `0.6` period and a cross extent of at most
///    `0.05` of the cross span each).
/// 2. [`SphereCap`](Self::SphereCap): an analytic sphere closed in exactly u
///    with two loops, each winding one period to within `0.05` period and the
///    two opposite, one flat (a v extent within `1e-6` of the v span) at a pole
///    (within `1e-6` of the v span of a v end).
/// 3. [`WrappedHorizon`](Self::WrappedHorizon): closed in exactly u, at least
///    one loop, neither [`HORIZON_CONTAINMENT_SWITCH`] nor (for one or two
///    loops) [`HORIZON_SINGLE_LOOP_SWITCH`], some loop of three or more samples
///    stepping over the seam (a sample-to-sample u step more than a quarter
///    period from its period-folded value), and no loop winding the period (its
///    unwrapped first and last samples more than a quarter period apart).
/// 4. [`CoveringRimStrip`](Self::CoveringRimStrip): closed in exactly u, two
///    loops, not [`COVERING_RIM_STRIP_SWITCH`], a sample beyond the u domain by
///    more than `1e-3` period, each loop travelling one period to within `2e-2`
///    period and closing in v to within `1e-3` of the v span, opposite
///    directions, v hulls apart by more than `1e-6` of the v span, and the lower
///    rim travelling +u exactly when the face is `same_sense`.
/// 5. [`Generic`](Self::Generic): everything else.
///
/// Sampling is [`trim_sample_count`] chords per pcurve at [`trim_station`]
/// (the seam band: 24). `Err` is the error the face's first query would
/// return.
///
/// It reads the prepared data of a face a query has prepared, and otherwise
/// prepares what the walk reaches on the side, leaving the thread's
/// prepared-face cache as it was: asked for every face of a part, it would
/// otherwise push out the faces the queries are using.
pub fn containment_lane(face: &FaceRecord) -> Result<ContainmentLane, String> {
    let prepared = cached_prepared_face(face).unwrap_or_else(|| std::rc::Rc::new(PreparedFace::new()));
    Ok(match lane_from(face, &prepared, 0)?.1 {
        Lane::SeamBand(_) => ContainmentLane::SeamBand,
        Lane::SphereCap(_) => ContainmentLane::SphereCap,
        Lane::Horizon(horizon) => ContainmentLane::WrappedHorizon {
            loops: horizon.loops.clone(),
            u_period: horizon.u_period,
            cross_frame: horizon_cross_frame(),
        },
        Lane::RimStrip(_) => ContainmentLane::CoveringRimStrip,
        Lane::Generic => ContainmentLane::Generic {
            sphere_chart: sphere_chart_atlas(face).is_some(),
        },
    })
}

/// The lane [`containment_lane`] finds a face in.
#[derive(Clone, Debug)]
pub enum ContainmentLane {
    /// The doubly-periodic seam-band lane: answers every point of the face.
    SeamBand,
    /// The winding sphere-cap lane: answers every point within the query's
    /// tolerance of a chord of the rim or whose u meets one; any other point
    /// goes on to the lanes after it.
    SphereCap,
    /// The wrapped-horizon lane: answers every point. Within the query's
    /// tolerance of a chord of `loops` at `u − u_period`, `u` or `u + u_period`
    /// it is `Boundary`; otherwise, with `cross_frame`, each loop's even-odd
    /// count (crossings of +u, half-open in v) at the three images summed mod
    /// 2, odd being `Inside`; without it ([`HORIZON_CROSS_FRAME_SWITCH`] = `"0"`
    /// when this was called), `Inside` when some image counts an odd total.
    WrappedHorizon {
        /// Each loop's samples, unwrapped sample to sample across the seam:
        /// the lane's own polygons, closed from last sample to first.
        loops: Vec<Vec<Vec2>>,
        u_period: f64,
        cross_frame: bool,
    },
    /// The covering-rim-strip lane: answers every point.
    CoveringRimStrip,
    /// The generic scan: `Boundary` within the tolerance of a chord; otherwise
    /// the side of the nearest pcurve where the nearest chord's foot lands
    /// inside it and the point is no farther from that chord than the chord is
    /// long; otherwise the even-odd count over every loop — for which, with
    /// `sphere_chart`, the sphere-chart lane substitutes where it can decide.
    Generic {
        /// An analytic sphere with the sphere-chart trim lane on
        /// ([`NO_SPHERE_CHART_TRIM_SWITCH`]), read when this was called.
        sphere_chart: bool,
    },
}

pub fn parameter_point_in_face(
    face: &FaceRecord,
    point: Vec2,
    tolerance: f64,
) -> Result<PolygonClass, String> {
    point_in_face_scanned(face, point, tolerance, ChordScan::Auto)
}

/// [`parameter_point_in_face`] with the generic scan and the wrapped-horizon
/// lane read through their chord index (`indexed`, on a face of any size) or
/// over every chord. The two give the same answer on every point; this exists
/// so a test can show it.
#[doc(hidden)]
pub fn parameter_point_in_face_scan(
    face: &FaceRecord,
    point: Vec2,
    tolerance: f64,
    indexed: bool,
) -> Result<PolygonClass, String> {
    let scan = if indexed {
        ChordScan::Indexed
    } else {
        ChordScan::Linear
    };
    point_in_face_scanned(face, point, tolerance, scan)
}

fn point_in_face_scanned(
    face: &FaceRecord,
    point: Vec2,
    tolerance: f64,
    scan: ChordScan,
) -> Result<PolygonClass, String> {
    let prepared = prepared_face(face);
    let mut from = 0;
    loop {
        let (position, lane) = lane_from(face, &prepared, from)?;
        let answer = match lane {
            Lane::SeamBand(band) => {
                seam_band_point_in_face(band, point, tolerance).map(|class| ("seam_band", class))
            }
            Lane::SphereCap(cap) => winding_sphere_cap_point_in_face(cap, point, tolerance)
                .map(|class| ("winding_sphere_cap", class)),
            Lane::Horizon(horizon) => wrapped_horizon_point_in_face(face, horizon, point, tolerance, scan)
                .map(|class| ("wrapped_horizon", class)),
            Lane::RimStrip(strip) => covering_rim_strip_point_in_face(strip, point, tolerance)
                .map(|class| ("covering_rim_strip", class)),
            Lane::Generic => break,
        };
        if let Some((name, class)) = answer {
            trace_lane(face, point, name, class);
            return Ok(class);
        }
        from = position + 1;
    }
    let generic = lane(&prepared.generic, || prepare_generic(face))?;
    let (crossings, boundary, nearest) = generic_scan(generic, point, tolerance, scan);
    if boundary {
        trace_lane(face, point, "polygon_boundary", PolygonClass::Boundary);
        return Ok(PolygonClass::Boundary);
    }
    let parity = if crossings % 2 == 1 {
        PolygonClass::Inside
    } else {
        PolygonClass::Outside
    };
    let chord = nearest.map(|(loop_index, index, distance)| {
        let polygon = &generic.loops[loop_index].polygon;
        let prep = generic.segments[loop_index][index];
        (
            polygon[index],
            polygon[(index + 1) % polygon.len()],
            &face.loops[loop_index].coedges[prep.coedge_index].pcurve,
            prep,
            distance,
        )
    });
    if containment_trace_enabled() {
        let (distance, length) = match chord.as_ref() {
            Some((start, end, _, _, distance)) => (*distance, end.sub(*start).length()),
            None => (f64::NAN, f64::NAN),
        };
        eprintln!(
            "containment face={} uv=({:.8},{:.8}) crossings={crossings} parity={parity:?} \
             nearest distance={distance:.3e} segment length={length:.3e}",
            face.id, point.x, point.y
        );
    }
    // The LAST of the special lanes, and the only one that engages here rather
    // than ahead of the scan: it substitutes for the parity count and for nothing
    // else. The four lanes above were each written for a shape the parity gets
    // wrong and are calibrated against real documents; pre-empting them would move
    // decisions this change has no business moving. Whenever the nearest-curve
    // refinement below can answer, it is more accurate than any global rule and is
    // what the imprint and the fragment builder are tuned against — so this asks
    // the sphere only on the complement of the refinement's own condition.
    let parity_decides = match chord.as_ref() {
        None => true,
        Some((start, end, _, _, distance)) => {
            let length = end.sub(*start).length();
            *distance > length || length <= 0.0
        }
    };
    if parity_decides {
        if let Some(class) = sphere_chart_point_in_face(face, point, &prepared, &generic.loops)? {
            trace_lane(face, point, "sphere_chart", class);
            return Ok(class);
        }
    }
    let Some((segment_start, segment_end, curve, prep, distance)) = chord else {
        trace_lane(face, point, "parity (no segment)", parity);
        return Ok(parity);
    };
    let segment_length = segment_end.sub(segment_start).length();
    if distance > segment_length || segment_length <= 0.0 {
        trace_lane(face, point, "parity (far from every segment)", parity);
        return Ok(parity);
    }
    let chord_parameter = prep.parameter_start
        + (prep.parameter_end - prep.parameter_start)
            * (point.sub(segment_start).dot(segment_end.sub(segment_start))
                / (segment_length * segment_length))
                .clamp(0.0, 1.0);
    let [domain_start, domain_end] = curve.domain()?;
    let mut parameter = chord_parameter;
    for _ in 0..12 {
        let derivatives = curve.derivatives(parameter, 2)?;
        let on_curve = Vec2 {
            x: derivatives[0].x,
            y: derivatives[0].y,
        };
        let tangent = Vec2 {
            x: derivatives[1].x,
            y: derivatives[1].y,
        };
        let second = Vec2 {
            x: derivatives[2].x,
            y: derivatives[2].y,
        };
        let residual = on_curve.sub(point);
        let denominator = tangent.dot(tangent) + residual.dot(second);
        if denominator.abs() < 1e-30 {
            break;
        }
        let step = -residual.dot(tangent) / denominator;
        parameter = (parameter + step).clamp(domain_start, domain_end);
        if step.abs() < 1e-14 * (domain_end - domain_start + 1.0) {
            break;
        }
    }
    let margin = 1e-9 * (domain_end - domain_start);
    if parameter > domain_start + margin && parameter < domain_end - margin {
        let derivatives = curve.derivatives(parameter, 1)?;
        let on_curve = Vec2 {
            x: derivatives[0].x,
            y: derivatives[0].y,
        };
        let tangent = Vec2 {
            x: derivatives[1].x,
            y: derivatives[1].y,
        };
        let offset = point.sub(on_curve);
        if offset.length() <= tolerance {
            trace_lane(face, point, "nearest curve (on it)", PolygonClass::Boundary);
            return Ok(PolygonClass::Boundary);
        }
        let cross = tangent.x * offset.y - tangent.y * offset.x;
        if cross.abs() > 1e-30 {
            let class = if (cross > 0.0) == face.same_sense {
                PolygonClass::Inside
            } else {
                PolygonClass::Outside
            };
            trace_lane(face, point, "nearest curve (side)", class);
            return Ok(class);
        }
    }
    trace_lane(face, point, "parity (nearest curve declined)", parity);
    Ok(parity)
}
