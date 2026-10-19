//! The 2-D arrangement (`face_split.rs`) samples every loop and every cut of a
//! face into chains, intersects the chains pairwise, extracts the cycles and
//! rebuilds the loops from them. This module does the same for a face whose
//! pieces are SIMPLE, and declines every other face to the arrangement:
//!
//! * every open piece ends on a vertex of the face's loops (after
//!   `apply_edge_splits` the coedges already break there), both in 3-D (the
//!   split vertex is the imprint junction within the weld floor) and in the
//!   chart (the piece's pcurve end lies on the loop junction);
//! * every closed piece (one vertex, both ends) lies in the face interior;
//! * no two piece ends meet at one loop junction, no piece shares an interior
//!   junction with another, no piece is a duplicate of another, and no piece
//!   rides an existing edge (`shared_edge`);
//! * no piece's interior comes within the chart band of a loop or of another
//!   piece (a graze or an unminted crossing is the arrangement's: it pinches
//!   regions apart where a split loop would touch itself);
//! * the face's loops are drawn in the carrier's domain without a seam hop,
//!   and the face has no cosurface partner.
//!
//! Boundary coedges keep their pcurves byte for byte and their traversal
//! direction; a cut coedge carries the piece's pcurve, reversed when the loop
//! traverses the piece backwards, exactly as `build_loop` emits them, so a
//! differential run can compare the two paths' records. Fragments are emitted
//! in ascending outer-loop area, the arrangement's region order; a region's
//! holes in the order the arrangement's walk finds their cycles; every loop
//! started at the coedge `build_loop` starts it at (the chain the walk reaches
//! first), so a reader that walks a loop from its first coedge sees the same
//! face from either path. The 2026-10-11 reds that reverted the first
//! promotion were a graze (a frustum's flat tangent to its top rim) and a
//! loop that started at a seam coedge instead of the bottom rim.
//!
//! Behind `BREP_LOOP_SPLIT`: unset or `on` (the default since the second
//! promotion, 2026-10-11, after the whole kernel battery ran green with it
//! on) returns the loop split's fragments where the face is simple and the
//! arrangement's elsewhere; `off` or `0` is the arrangement alone, the A/B
//! hatch; `check` runs both paths on every eligible face, returns the
//! arrangement's fragments and appends one line per face to
//! `BREP_LOOP_SPLIT_CENSUS=<file>` (`loop-split census:`, with
//! `BREP_CENSUS_LABEL`).

use super::face_split::FragmentIndex;
use super::loops::{edge_for, interior_points_in_polygons, pcurve_for, point_in_polygon, vertex_point};
use super::sampling::sample_chain;
use super::*;
use crate::tolerance::WELD_FLOOR;
use crate::{KernelRefusal, KernelStage, OrRefuse};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum LoopSplitMode {
    Off,
    Check,
    On,
}

/// `BREP_LOOP_SPLIT`, read once per process.
pub(super) fn loop_split_mode() -> LoopSplitMode {
    static MODE: std::sync::OnceLock<LoopSplitMode> = std::sync::OnceLock::new();
    *MODE.get_or_init(|| match std::env::var("BREP_LOOP_SPLIT").as_deref() {
        Ok("0") | Ok("off") => LoopSplitMode::Off,
        Ok("check") => LoopSplitMode::Check,
        _ => LoopSplitMode::On,
    })
}

/// Why the loop split left a face to the arrangement. Named so the census
/// can count them and a test can pin the fallback.
#[derive(Clone, Debug, PartialEq)]
pub(super) enum Decline {
    /// The face pairs with a cosurface partner: selection reads those faces
    /// against each other and the arrangement path owns that class.
    Cosurface,
    /// A piece rides an existing edge (`shared_edge`).
    SharedEdge,
    /// Two pieces are one section minted twice (the arrangement's key drop).
    DuplicatePiece,
    /// A loop or piece pcurve leaves the carrier's domain.
    OutOfDomain,
    /// Consecutive coedges of a loop do not meet in the chart.
    SeamHop,
    /// An open piece's end is on no loop junction (3-D vertex and chart).
    EndNotOnLoop(u64),
    /// Two piece ends share one loop junction.
    EndTwiceUsed(u64),
    /// A closed piece's vertex lies on a loop or is shared with another piece.
    RingTouches(u64),
    /// A piece's two ends resolve to loops of different regions.
    CrossRegion(u64),
    /// A piece's two ends resolve to the same junction.
    SameJunction(u64),
    /// A piece's interior comes within the chart band of a loop or of another
    /// piece away from its own ends: a graze or a crossing the imprint minted
    /// no vertex for (a frustum's wall cut by a flat whose plane is tangent to
    /// the top rim: the hyperbola touches the rim between its feet and the
    /// arrangement pinches the outside into two regions).
    Crosses(u64),
    /// A closed piece encloses no measurable area in the chart.
    RingTooSmall(u64),
    /// A closed piece lies in no region of the face.
    RingOutside(u64),
    /// A region has no interior point, or its point reads outside the face.
    NoInterior,
    /// The imprint lacks a vertex or pcurve the face needs.
    Incomplete,
}

impl std::fmt::Display for Decline {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}

#[derive(Clone)]
enum Side<'a> {
    Boundary { coedge: &'a CoedgeRecord },
    Cut { piece_id: u64, forward: bool, pcurve: &'a NurbsCurve, shared_ring: Option<(u8, u64, bool)> },
}

/// One coedge of a loop under construction: its side, its chart samples in
/// traversal order (the first is the junction the entry starts at), the 3-D
/// vertex at that junction and its chain key: the position the arrangement
/// gives the same chain (the face's boundary coedges in loop order, then the
/// pieces in `piece_ids` order), which decides where an emitted loop starts.
#[derive(Clone)]
struct Entry<'a> {
    side: Side<'a>,
    samples: Vec<Vec2>,
    start_point: Vec3,
    end_point: Vec3,
    key: usize,
}

impl<'a> Entry<'a> {
    fn start(&self) -> Vec2 {
        self.samples[0]
    }
    fn end(&self) -> Vec2 {
        self.samples[self.samples.len() - 1]
    }
    fn reversed(&self) -> Self {
        let mut samples = self.samples.clone();
        samples.reverse();
        Entry {
            side: match &self.side {
                Side::Boundary { coedge } => Side::Boundary { coedge },
                Side::Cut { piece_id, forward, pcurve, shared_ring } => Side::Cut { piece_id: *piece_id, forward: !forward, pcurve, shared_ring: *shared_ring },
            },
            samples,
            start_point: self.end_point,
            end_point: self.start_point,
            key: self.key,
        }
    }
    /// Whether the entry runs along its chain: a boundary coedge always does,
    /// a cut when it follows its piece.
    fn along_chain(&self) -> bool {
        match &self.side {
            Side::Boundary { .. } => true,
            Side::Cut { forward, .. } => *forward,
        }
    }
}

#[derive(Clone)]
struct LoopUnderConstruction<'a> {
    entries: Vec<Entry<'a>>,
}

impl<'a> LoopUnderConstruction<'a> {
    /// The least chain key among the entries.
    fn min_key(&self) -> usize {
        self.entries.iter().map(|entry| entry.key).min().unwrap_or(0)
    }
    /// Start the loop where the arrangement starts the same cycle. Its walk
    /// begins at the cycle's lowest-indexed segment, in the direction the
    /// cycle uses it, and `build_loop` then rotates to the first chain
    /// boundary and reverses the whole cycle when the face is drawn against
    /// its positive winding. Read in the emitted direction: the entry with the
    /// least key comes FIRST when the face winds positive and the loop runs
    /// along that chain, and LAST otherwise (the walk met its lowest segment
    /// at the end of the run, or the reversal put the run's start at the end).
    fn rotate_like_arrangement(&mut self, reverse_loops: bool) {
        let n = self.entries.len();
        if n < 2 {
            return;
        }
        let min_index = (0..n).min_by_key(|&k| self.entries[k].key).unwrap_or(0);
        let start = if !reverse_loops && self.entries[min_index].along_chain() { min_index } else { (min_index + 1) % n };
        self.entries.rotate_left(start);
    }
    fn polygon(&self) -> Vec<Vec2> {
        let mut polygon = Vec::new();
        for entry in &self.entries {
            polygon.extend_from_slice(&entry.samples[..entry.samples.len() - 1]);
        }
        polygon
    }
    fn area(&self) -> f64 {
        signed_area(&self.polygon())
    }
    /// The entries from junction `from` forward to junction `to` (exclusive),
    /// cyclically.
    fn arc(&self, from: usize, to: usize) -> Vec<Entry<'a>> {
        let n = self.entries.len();
        let mut out = Vec::new();
        let mut index = from;
        loop {
            out.push(self.entries[index].clone());
            index = (index + 1) % n;
            if index == to {
                break;
            }
        }
        out
    }
}

struct Region<'a> {
    outer: LoopUnderConstruction<'a>,
    holes: Vec<LoopUnderConstruction<'a>>,
}

/// Insert a chord (`forward`, its reverse `backward`) from junction `a` to
/// junction `b` of region `ra`: a split when both are on one loop, a join of
/// two loops otherwise.
fn insert_chord<'a>(
    regions: &mut Vec<Region<'a>>,
    ra: usize,
    (la, ka): (usize, usize),
    (lb, kb): (usize, usize),
    forward: Vec<Entry<'a>>,
    backward: Vec<Entry<'a>>,
    outer_sign: bool,
) {
    let region = regions.swap_remove(ra);
    let Region { outer, mut holes } = region;
    fn take<'b>(l: usize, outer: &LoopUnderConstruction<'b>, holes: &[LoopUnderConstruction<'b>]) -> LoopUnderConstruction<'b> {
        if l == 0 { outer.clone() } else { holes[l - 1].clone() }
    }
    if la == lb {
        // Split: from a forward to b, back along the piece; from b
        // forward to a, along the piece.
        let loop_a = take(la, &outer, &holes);
        let mut first = loop_a.arc(ka, kb);
        first.extend(backward);
        let mut second = loop_a.arc(kb, ka);
        second.extend(forward);
        let first = LoopUnderConstruction { entries: first };
        let second = LoopUnderConstruction { entries: second };
        if la == 0 {
            // The outer split into two outers; the holes go to the one
            // that contains them.
            let first_polygon = first.polygon();
            let mut first_holes = Vec::new();
            let mut second_holes = Vec::new();
            for hole in holes {
                if point_in_polygon(hole.entries[0].start(), &first_polygon) {
                    first_holes.push(hole);
                } else {
                    second_holes.push(hole);
                }
            }
            regions.push(Region { outer: first, holes: first_holes });
            regions.push(Region { outer: second, holes: second_holes });
        } else {
            // A hole split by a chord: one side is a hole of the region,
            // the other is a new region (its loop winds against the
            // hole's, hence like the outer).
            holes.remove(la - 1);
            let (hole_side, region_side) = if (first.area() >= 0.0) == outer_sign { (second, first) } else { (first, second) };
            holes.push(hole_side);
            regions.push(Region { outer, holes });
            regions.push(Region { outer: region_side, holes: Vec::new() });
        }
    } else {
        // Join: around a's loop from a, along the piece to b, around b's
        // loop from b, back along the piece.
        let loop_a = take(la, &outer, &holes);
        let loop_b = take(lb, &outer, &holes);
        let mut joined = loop_a.arc(ka, ka);
        joined.extend(forward);
        joined.extend(loop_b.arc(kb, kb));
        joined.extend(backward);
        let joined = LoopUnderConstruction { entries: joined };
        let mut remaining: Vec<LoopUnderConstruction<'_>> = Vec::new();
        for (l, hole) in holes.into_iter().enumerate() {
            if l + 1 != la && l + 1 != lb {
                remaining.push(hole);
            }
        }
        if la == 0 || lb == 0 {
            regions.push(Region { outer: joined, holes: remaining });
        } else {
            remaining.push(joined);
            regions.push(Region { outer, holes: remaining });
        }
    }
}

fn signed_area(polygon: &[Vec2]) -> f64 {
    let mut area = 0.0;
    for index in 0..polygon.len() {
        let a = polygon[index];
        let b = polygon[(index + 1) % polygon.len()];
        area += 0.5 * (a.x * b.y - b.x * a.y);
    }
    area
}

fn uv(point: Vec3) -> Vec2 {
    Vec2 { x: point.x, y: point.y }
}

/// A cut entry traversed along its piece (`forward`), with the piece's start
/// as the entry's start.
fn cut_entry<'a>(piece_id: u64, pcurve: &'a NurbsCurve, samples: &[Vec2], start_point: Vec3, end_point: Vec3, key: usize) -> Entry<'a> {
    Entry {
        side: Side::Cut { piece_id, forward: true, pcurve, shared_ring: None },
        samples: samples.to_vec(),
        start_point,
        end_point,
        key,
    }
}

/// The chart box of a sample chain, grown by `band`.
fn chain_box(samples: &[Vec2], band: f64) -> [Vec2; 2] {
    let mut low = Vec2 { x: f64::INFINITY, y: f64::INFINITY };
    let mut high = Vec2 { x: f64::NEG_INFINITY, y: f64::NEG_INFINITY };
    for point in samples {
        low.x = low.x.min(point.x - band);
        low.y = low.y.min(point.y - band);
        high.x = high.x.max(point.x + band);
        high.y = high.y.max(point.y + band);
    }
    [low, high]
}

fn boxes_meet(a: &[Vec2; 2], b: &[Vec2; 2]) -> bool {
    a[0].x <= b[1].x && b[0].x <= a[1].x && a[0].y <= b[1].y && b[0].y <= a[1].y
}

/// Where a piece's chain meets another chain within `band`, away from the
/// points in `clear` by more than `clear_radius`: a sample of either chain on
/// a segment of the other (which a chain riding along the other shows, and
/// the parallel-rejecting segment test does not), or two segments crossing.
/// Returns the meeting point, or `None` when the chains stay apart.
fn chains_meet(piece: &[Vec2], other: &[Vec2], band: f64, clear: &[Vec2], clear_radius: f64) -> Option<Vec2> {
    use super::sampling::point_segment_distance;
    let is_clear = |point: Vec2| clear.iter().all(|centre| centre.sub(point).length() > clear_radius);
    let other_segments: Vec<([Vec2; 2], [Vec2; 2])> =
        other.windows(2).map(|pair| ([pair[0], pair[1]], chain_box(&pair[..], band))).collect();
    for window in piece.windows(2) {
        let (a, b) = (window[0], window[1]);
        let segment_box = chain_box(window, band);
        for (segment, other_box) in &other_segments {
            if !boxes_meet(&segment_box, other_box) {
                continue;
            }
            for point in [a, b] {
                if point_segment_distance(point, segment[0], segment[1]) <= band && is_clear(point) {
                    return Some(point);
                }
            }
            for point in *segment {
                if point_segment_distance(point, a, b) <= band && is_clear(point) {
                    return Some(point);
                }
            }
            if let Some([t, _]) = crate::arrangement::segment_intersection(a, b, segment[0], segment[1], band) {
                let hit = a.add(b.sub(a).scale(t));
                if is_clear(hit) {
                    return Some(hit);
                }
            }
        }
    }
    None
}

/// Split the face by its simple pieces, or say why it is not simple.
pub(super) fn loop_split_face(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    face_key: FaceKey,
    piece_ids: &[u64],
    index: &FragmentIndex,
) -> Result<Result<Vec<FaceFragmentRecord>, Decline>, KernelRefusal> {
    let cycles = !matches!(std::env::var("BREP_LOOP_SPLIT_CYCLES").as_deref(), Ok("0") | Ok("off"));
    let shared_rings = !matches!(std::env::var("BREP_LOOP_SPLIT_SHARED_RINGS").as_deref(), Ok("0") | Ok("off"));
    let touch_rings = touch_rings_enabled();
    loop_split_face_impl(solid, operand, face, face_key, piece_ids, index, cycles, shared_rings, touch_rings, touch_rings && junction_chords_enabled())
}

/// `BREP_LOOP_SPLIT_JUNCTION_CHORDS` (lane X row 6): the touching pieces of a
/// face reach TWO or more loop junctions through vertices inside the face (the
/// wall of a bore tangent to the box wall: each cap circle crosses the chart
/// from seam to seam through its tangent vertex, and the tangent line joins
/// the two tangent vertices). Each path of pieces from a loop junction through
/// interior vertices to a loop junction is a chord, inserted as an open piece
/// is; its interior vertices are junctions for the next. A piece left with an
/// end on no loop declines. On by default: `0`/`off` declines the class (it
/// needs `BREP_LOOP_SPLIT_TOUCH_RINGS`).
pub(super) fn junction_chords_enabled() -> bool {
    let enabled = !matches!(std::env::var("BREP_LOOP_SPLIT_JUNCTION_CHORDS").as_deref(), Ok("0") | Ok("off"));
    enabled
}


/// `BREP_LOOP_SPLIT_TOUCH_RINGS` (lane X row 1): a closed section that touches
/// a loop at ONE minted junction and is otherwise interior (a bore tangent to
/// the box wall: the cap circle meets the wall edge at the tangent vertex and
/// is split at the cylinder's seam vertex inside the face). The arrangement
/// splices that ring into the loop at the junction, a pinched loop, and makes
/// the disc its own region; so does the split. On by default: `0`/`off`
/// declines the class.
pub(super) fn touch_rings_enabled() -> bool {
    let enabled = !matches!(std::env::var("BREP_LOOP_SPLIT_TOUCH_RINGS").as_deref(), Ok("0") | Ok("off"));
    enabled
}


/// A measured extension for two interior arcs on an affine carrier. The
/// extension passed the kernel, manifest and HLR census; `off` retains
/// the original splitter for differential measurements.
#[allow(clippy::too_many_arguments)]
pub(super) fn loop_split_face_with_cycles(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    face_key: FaceKey,
    piece_ids: &[u64],
    index: &FragmentIndex,
    cycles: bool,
) -> Result<Result<Vec<FaceFragmentRecord>, Decline>, KernelRefusal> {
    loop_split_face_with_options(solid, operand, face, face_key, piece_ids, index, cycles, false)
}

#[allow(clippy::too_many_arguments)]
pub(super) fn loop_split_face_with_options(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    face_key: FaceKey,
    piece_ids: &[u64],
    index: &FragmentIndex,
    cycles: bool,
    shared_rings: bool,
) -> Result<Result<Vec<FaceFragmentRecord>, Decline>, KernelRefusal> {
    loop_split_face_impl(solid, operand, face, face_key, piece_ids, index, cycles, shared_rings, false, false)
}

#[allow(clippy::too_many_arguments)]
fn loop_split_face_impl(
    solid: &BrepSolid,
    operand: u8,
    face: &FaceRecord,
    face_key: FaceKey,
    piece_ids: &[u64],
    index: &FragmentIndex,
    cycles: bool,
    shared_rings: bool,
    touch_rings: bool,
    junction_chords: bool,
) -> Result<Result<Vec<FaceFragmentRecord>, Decline>, KernelRefusal> {
    if index.cosurface_pairs.iter().any(|(a, b)| *a == face_key || *b == face_key) {
        return Ok(Err(Decline::Cosurface));
    }
    let u_domain = KnotVector::new(face.surface.knots_u.clone(), face.surface.degree_u).or_refuse(KernelStage::Fragment, "new")?.domain();
    let v_domain = KnotVector::new(face.surface.knots_v.clone(), face.surface.degree_v).or_refuse(KernelStage::Fragment, "new")?.domain();
    let diagonal = ((u_domain[1] - u_domain[0]).powi(2) + (v_domain[1] - v_domain[0]).powi(2)).sqrt();
    // The arrangement's own resolution, and the band a pcurve end may miss its
    // loop junction by in the chart: both ends are projections of one 3-D
    // point onto one carrier, so the band is generous for them and still far
    // under any feature of the chart.
    let arrangement_tolerance = 1e-7f64.max(diagonal * 1e-6);
    let uv_match = (arrangement_tolerance * 100.0).max(diagonal * 1e-5);
    let domain_slack = arrangement_tolerance * 100.0;
    let in_domain = |samples: &[Vec2]| {
        samples.iter().all(|point| {
            point.x >= u_domain[0] - domain_slack
                && point.x <= u_domain[1] + domain_slack
                && point.y >= v_domain[0] - domain_slack
                && point.y <= v_domain[1] + domain_slack
        })
    };
    let vertex_band = WELD_FLOOR;

    // The face's loops, as drawn.
    let mut loops: Vec<LoopUnderConstruction<'_>> = Vec::with_capacity(face.loops.len());
    let mut next_key = 0usize;
    for loop_record in &face.loops {
        let mut entries = Vec::with_capacity(loop_record.coedges.len());
        for coedge in &loop_record.coedges {
            let key = next_key;
            next_key += 1;
            let edge = edge_for(solid, coedge.edge_id)?;
            let (start_vertex, end_vertex) = if coedge.forward {
                (edge.start_vertex_id, edge.end_vertex_id)
            } else {
                (edge.end_vertex_id, edge.start_vertex_id)
            };
            let samples = sample_chain(&coedge.pcurve)?;
            if samples.len() < 2 {
                return Ok(Err(Decline::Incomplete));
            }
            if !in_domain(&samples) {
                return Ok(Err(Decline::OutOfDomain));
            }
            entries.push(Entry {
                side: Side::Boundary { coedge },
                samples,
                start_point: vertex_point(solid, start_vertex)?,
                end_point: vertex_point(solid, end_vertex)?,
                key,
            });
        }
        if entries.is_empty() {
            return Ok(Err(Decline::Incomplete));
        }
        for k in 0..entries.len() {
            let next = (k + 1) % entries.len();
            if entries[k].end().sub(entries[next].start()).length() > uv_match {
                return Ok(Err(Decline::SeamHop));
            }
        }
        loops.push(LoopUnderConstruction { entries });
    }
    if loops.is_empty() {
        return Ok(Err(Decline::Incomplete));
    }
    // The outer loop is the one of largest area; its sign is the chart's
    // material winding, which every new outer loop follows and every hole
    // opposes.
    let areas: Vec<f64> = loops.iter().map(|l| l.area()).collect();
    let outer_index = (0..loops.len()).max_by(|a, b| areas[*a].abs().total_cmp(&areas[*b].abs())).unwrap_or(0);
    let outer_sign = areas[outer_index] >= 0.0;
    // The arrangement reverses its (positive) cycles when the face is drawn
    // against them: by the outer loop's winding on an open carrier, by
    // `same_sense` on a closed one (`source_loop_winding_negative`).
    let closed = face.surface.closed_directions().or_refuse(KernelStage::Fragment, "closed_directions")?;
    let reverse_loops = if closed.0 || closed.1 { !face.same_sense } else { !outer_sign };
    let outer = loops.swap_remove(outer_index);
    let mut regions = vec![Region { outer, holes: loops }];
    let boundary_keys = next_key;

    // The pieces: open ones ending on loop junctions, closed ones in the
    // interior.
    struct OpenPiece<'a> {
        id: u64,
        pcurve: &'a NurbsCurve,
        samples: Vec<Vec2>,
        start_point: Vec3,
        end_point: Vec3,
        key: usize,
    }
    struct Ring<'a> {
        id: u64,
        entries: Vec<Entry<'a>>,
        samples: Vec<Vec2>,
        point: Vec3,
        area: f64,
    }
    let mut interior_arcs = Vec::new();
    // Pieces of a ring touching a loop at one junction (`touch_rings`), with
    // their vertex ids, chained below.
    let mut touch_arcs: Vec<(Entry<'_>, u64, u64)> = Vec::new();
    let mut open_pieces: Vec<OpenPiece<'_>> = Vec::new();
    let mut rings: Vec<Ring<'_>> = Vec::new();
    let mut keys: HashSet<((i64, i64), (i64, i64), (i64, i64))> = HashSet::default();
    let quantize = |point: Vec2| ((point.x / arrangement_tolerance).round() as i64, (point.y / arrangement_tolerance).round() as i64);
    let on_a_loop = |point: Vec3, regions: &[Region<'_>]| {
        regions.iter().any(|region| {
            std::iter::once(&region.outer)
                .chain(region.holes.iter())
                .any(|l| l.entries.iter().any(|entry| entry.start_point.sub(point).length() <= vertex_band))
        })
    };
    for (position, piece_id) in piece_ids.iter().enumerate() {
        let key = boundary_keys + position;
        let Some(piece) = index.pieces_by_id.get(piece_id) else {
            return Ok(Err(Decline::Incomplete));
        };
        let shared_ring = if shared_rings && piece_ids.len() == 1
            && face.surface.is_affine().unwrap_or(false)
            && piece.start_vertex_id == piece.end_vertex_id
            && piece.curve.evaluate(piece.t0).ok().zip(piece.curve.evaluate(piece.t1).ok())
                .is_some_and(|(a, b)| a.sub(b).length() <= 1e-6)
        {
            piece.shared_edge
        } else {
            None
        };
        if piece.shared_edge.is_some() && shared_ring.is_none() {
            return Ok(Err(Decline::SharedEdge));
        }
        let pcurve = pcurve_for(piece, face_key)?;
        let samples = sample_chain(pcurve)?;
        if samples.len() < 2 {
            return Ok(Err(Decline::Incomplete));
        }
        if !in_domain(&samples) {
            return Ok(Err(Decline::OutOfDomain));
        }
        let [t0, t1] = pcurve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let middle = uv(pcurve.evaluate(0.5 * (t0 + t1)).or_refuse(KernelStage::Fragment, "evaluate")?);
        let mut ends = [quantize(samples[0]), quantize(samples[samples.len() - 1])];
        ends.sort();
        if !keys.insert((ends[0], quantize(middle), ends[1])) {
            return Ok(Err(Decline::DuplicatePiece));
        }
        let (Some(&start_point), Some(&end_point)) =
            (index.vertex_points.get(&piece.start_vertex_id), index.vertex_points.get(&piece.end_vertex_id))
        else {
            return Ok(Err(Decline::Incomplete));
        };
        let closed = piece.start_vertex_id == piece.end_vertex_id
            && samples[0].sub(samples[samples.len() - 1]).length() <= uv_match;
        if std::env::var_os("BREP_DEBUG_LOOP_SPLIT").is_some() {
            let nearest = |point: Vec3, at: Vec2| {
                regions
                    .iter()
                    .flat_map(|region| std::iter::once(&region.outer).chain(region.holes.iter()))
                    .flat_map(|l| l.entries.iter())
                    .map(|entry| (entry.start_point.sub(point).length(), entry.start().sub(at).length()))
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .unwrap_or((f64::NAN, f64::NAN))
            };
            let (ds3, dsuv) = nearest(start_point, samples[0]);
            let (de3, deuv) = nearest(end_point, samples[samples.len() - 1]);
            eprintln!(
                "loop-split face {} piece {} vertices {}->{} closed={closed} uv_start=({:.6},{:.6}) uv_end=({:.6},{:.6}) nearest_junction start 3d={ds3:.3e} uv={dsuv:.3e} end 3d={de3:.3e} uv={deuv:.3e} uv_match={uv_match:.3e} domain u={u_domain:?} v={v_domain:?}",
                face.id, piece_id, piece.start_vertex_id, piece.end_vertex_id, samples[0].x, samples[0].y, samples[samples.len() - 1].x, samples[samples.len() - 1].y
            );
        }
        if closed {
            if touch_rings && shared_ring.is_none() && on_a_loop(start_point, &regions) {
                touch_arcs.push((cut_entry(*piece_id, pcurve, &samples, start_point, end_point, key), piece.start_vertex_id, piece.end_vertex_id));
                continue;
            }
            if on_a_loop(start_point, &regions) {
                return Ok(Err(Decline::RingTouches(*piece_id)));
            }
            let area = signed_area(&samples[..samples.len() - 1]);
            if area.abs() <= (arrangement_tolerance * arrangement_tolerance).max(1e-24) {
                return Ok(Err(Decline::RingTooSmall(*piece_id)));
            }
            let mut entry = cut_entry(*piece_id, pcurve, &samples, start_point, end_point, key);
            if let Side::Cut { shared_ring: target, .. } = &mut entry.side {
                *target = shared_ring;
            }
            rings.push(Ring { id: *piece_id, entries: vec![entry], samples, point: start_point, area });
        } else {
            if !on_a_loop(start_point, &regions) || !on_a_loop(end_point, &regions) {
                if cycles && piece_ids.len() == 2 && face.surface.is_affine().unwrap_or(false)
                    && !on_a_loop(start_point, &regions) && !on_a_loop(end_point, &regions)
                {
                    interior_arcs.push(cut_entry(*piece_id, pcurve, &samples, start_point, end_point, key));
                    continue;
                }
                if touch_rings {
                    touch_arcs.push((cut_entry(*piece_id, pcurve, &samples, start_point, end_point, key), piece.start_vertex_id, piece.end_vertex_id));
                    continue;
                }
                return Ok(Err(Decline::EndNotOnLoop(*piece_id)));
            }
            open_pieces.push(OpenPiece { id: *piece_id, pcurve, samples, start_point, end_point, key });
        }
    }
    if !interior_arcs.is_empty() {
        let first_id = piece_ids[0];
        if interior_arcs.len() != 2 || !open_pieces.is_empty() || !rings.is_empty() {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        }
        let first = interior_arcs.remove(0);
        let second = interior_arcs.remove(0);
        // Pair by minted topology, not proximity. Both joins must also meet
        // in the chart; an atlas hop must still take the arrangement.
        let a = index.pieces_by_id[&piece_ids[0]];
        let b = index.pieces_by_id[&piece_ids[1]];
        let second = if a.end_vertex_id == b.start_vertex_id && a.start_vertex_id == b.end_vertex_id {
            second
        } else if a.end_vertex_id == b.end_vertex_id && a.start_vertex_id == b.start_vertex_id {
            second.reversed()
        } else {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        };
        if first.end().sub(second.start()).length() > uv_match
            || second.end().sub(first.start()).length() > uv_match
        {
            return Ok(Err(Decline::SeamHop));
        }
        // The two arcs may meet only at their minted joins, with the same
        // clearance radius used by the original splitter's endpoint guard.
        let clear_radius = (arrangement_tolerance * 50.0).max(diagonal * 1.5e-3);
        if chains_meet(&first.samples, &second.samples, uv_match,
            &[first.start(), first.end()], clear_radius).is_some()
        {
            return Ok(Err(Decline::Crosses(first_id)));
        }
        let mut samples = first.samples[..first.samples.len() - 1].to_vec();
        samples.extend_from_slice(&second.samples);
        let area = signed_area(&samples[..samples.len() - 1]);
        if area.abs() <= (arrangement_tolerance * arrangement_tolerance).max(1e-24) {
            return Ok(Err(Decline::RingTooSmall(first_id)));
        }
        let point = first.start_point;
        rings.push(Ring { id: first_id, entries: vec![first, second], samples, point, area });
    }
    // A touching ring: every piece chains into ONE cycle through exactly one
    // loop junction J (by minted vertex id, never by proximity), no other
    // piece on the face. Its chain starts at J.
    // Junction chords (`junction_chords`): touching pieces that reach two or
    // more loop junctions, with no other piece on the face, leave the
    // one-junction ring path for the chord insertion below.
    let mut chord_arcs: Vec<(Entry<'_>, u64, u64)> = Vec::new();
    if junction_chords && !touch_arcs.is_empty() && open_pieces.is_empty() && rings.is_empty() && interior_arcs.is_empty() {
        let mut junctions: Vec<u64> = Vec::new();
        for (entry, start_id, end_id) in &touch_arcs {
            for (id, point) in [(*start_id, entry.start_point), (*end_id, entry.end_point)] {
                if on_a_loop(point, &regions) && !junctions.contains(&id) {
                    junctions.push(id);
                }
            }
        }
        if junctions.len() >= 2 {
            chord_arcs = std::mem::take(&mut touch_arcs);
        }
    }
    let mut touch: Option<Ring<'_>> = None;
    if !touch_arcs.is_empty() {
        let first_id = match &touch_arcs[0].0.side { Side::Cut { piece_id, .. } => *piece_id, Side::Boundary { .. } => 0 };
        if !open_pieces.is_empty() || !rings.is_empty() || !interior_arcs.is_empty() {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        }
        let mut junctions: Vec<u64> = Vec::new();
        for (entry, start_id, end_id) in &touch_arcs {
            for (id, point) in [(*start_id, entry.start_point), (*end_id, entry.end_point)] {
                if on_a_loop(point, &regions) && !junctions.contains(&id) {
                    junctions.push(id);
                }
            }
        }
        if junctions.len() != 1 {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        }
        let junction = junctions[0];
        // Every vertex of the cycle is used by exactly two piece ends.
        let mut uses: HashMap<u64, usize> = HashMap::default();
        for (_, start_id, end_id) in &touch_arcs {
            *uses.entry(*start_id).or_default() += 1;
            *uses.entry(*end_id).or_default() += 1;
        }
        if uses.values().any(|&count| count != 2) {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        }
        let mut remaining: Vec<(Entry<'_>, u64, u64)> = std::mem::take(&mut touch_arcs);
        let mut entries: Vec<Entry<'_>> = Vec::with_capacity(remaining.len());
        let mut at = junction;
        while !remaining.is_empty() {
            let Some(position) = remaining.iter().position(|(_, start_id, end_id)| *start_id == at || *end_id == at) else {
                return Ok(Err(Decline::EndNotOnLoop(first_id)));
            };
            let (entry, start_id, end_id) = remaining.remove(position);
            if start_id == at {
                at = end_id;
                entries.push(entry);
            } else {
                at = start_id;
                entries.push(entry.reversed());
            }
            if at == junction && !remaining.is_empty() {
                return Ok(Err(Decline::EndNotOnLoop(first_id)));
            }
        }
        if at != junction {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        }
        for k in 0..entries.len() {
            let next = (k + 1) % entries.len();
            if entries[k].end().sub(entries[next].start()).length() > uv_match {
                return Ok(Err(Decline::SeamHop));
            }
        }
        let mut samples: Vec<Vec2> = Vec::new();
        for entry in &entries {
            samples.extend_from_slice(&entry.samples[..entry.samples.len() - 1]);
        }
        samples.push(entries[0].start());
        let area = signed_area(&samples[..samples.len() - 1]);
        if area.abs() <= (arrangement_tolerance * arrangement_tolerance).max(1e-24) {
            return Ok(Err(Decline::RingTooSmall(first_id)));
        }
        let point = entries[0].start_point;
        touch = Some(Ring { id: first_id, entries, samples, point, area });
    }
    // Every piece's interior stays clear of the loops and of the other
    // pieces. The band is the chart band a piece end may miss its junction
    // by; an open piece is read clear of its own two junctions by the radius
    // the arrangement drops chain samples around a junction, so leaving the
    // loop at a shallow angle is not a graze. Chains whose boxes stay apart
    // are never compared (the plate's 36 rings against its four sides).
    {
        let clear_radius = (arrangement_tolerance * 50.0).max(diagonal * 1.5e-3);
        let band = uv_match;
        let loop_chains: Vec<&[Vec2]> = regions
            .iter()
            .flat_map(|region| std::iter::once(&region.outer).chain(region.holes.iter()))
            .flat_map(|l| l.entries.iter().map(|entry| entry.samples.as_slice()))
            .collect();
        let loop_boxes: Vec<[Vec2; 2]> = loop_chains.iter().map(|chain| chain_box(chain, band)).collect();
        let piece_chains: Vec<(u64, &[Vec2], Vec<Vec2>)> = open_pieces
            .iter()
            .map(|piece| (piece.id, piece.samples.as_slice(), vec![piece.samples[0], piece.samples[piece.samples.len() - 1]]))
            .chain(rings.iter().map(|ring| (ring.id, ring.samples.as_slice(), Vec::new())))
            // A touching ring leaves its junction tangentially or at an angle;
            // it is read clear of that one junction like an open piece's end.
            .chain(touch.iter().map(|ring| (ring.id, ring.samples.as_slice(), vec![ring.samples[0]])))
            // A chord arc is read clear only of its ends ON a loop: an interior
            // junction near the loop is a graze the arrangement resolves.
            .chain(chord_arcs.iter().map(|(entry, _, _)| {
                let id = match &entry.side { Side::Cut { piece_id, .. } => *piece_id, Side::Boundary { .. } => 0 };
                let on_loop_ends = [(entry.start_point, entry.start()), (entry.end_point, entry.end())]
                    .into_iter()
                    .filter(|(point, _)| on_a_loop(*point, &regions))
                    .map(|(_, at)| at)
                    .collect();
                (id, entry.samples.as_slice(), on_loop_ends)
            }))
            .collect();
        let chord_start = piece_chains.len() - chord_arcs.len();
        let chord_ends: Vec<Vec<Vec2>> = chord_arcs.iter().map(|(entry, _, _)| vec![entry.start(), entry.end()]).collect();
        let piece_boxes: Vec<[Vec2; 2]> = piece_chains.iter().map(|(_, chain, _)| chain_box(chain, band)).collect();
        for (i, (id, chain, clear)) in piece_chains.iter().enumerate() {
            for (loop_chain, loop_box) in loop_chains.iter().zip(&loop_boxes) {
                if boxes_meet(&piece_boxes[i], loop_box) && chains_meet(chain, loop_chain, band, clear, clear_radius).is_some() {
                    return Ok(Err(Decline::Crosses(*id)));
                }
            }
            for (j, (_, other, _)) in piece_chains.iter().enumerate().skip(i + 1) {
                // Two chord arcs meet only at the junctions they share.
                let shared: Vec<Vec2> = if i >= chord_start {
                    let (mine, theirs) = (&chord_ends[i - chord_start], &chord_ends[j - chord_start]);
                    mine.iter().copied().filter(|end| theirs.iter().any(|o| o.sub(*end).length() <= uv_match)).collect()
                } else {
                    Vec::new()
                };
                let radius = if shared.is_empty() { 0.0 } else { clear_radius };
                if boxes_meet(&piece_boxes[i], &piece_boxes[j]) && chains_meet(chain, other, band, &shared, radius).is_some() {
                    return Ok(Err(Decline::Crosses(*id)));
                }
            }
        }
    }
    // A ring's vertex shared with any other piece is a junction the ring
    // meets another section at: not simple.
    for ring in &rings {
        let shared = open_pieces
            .iter()
            .any(|piece| piece.start_point.sub(ring.point).length() <= vertex_band || piece.end_point.sub(ring.point).length() <= vertex_band)
            || rings.iter().filter(|other| other.id != ring.id).any(|other| other.point.sub(ring.point).length() <= vertex_band);
        if shared {
            return Ok(Err(Decline::RingTouches(ring.id)));
        }
    }
    // No two open-piece ends on one junction.
    for (i, piece) in open_pieces.iter().enumerate() {
        for end in [(piece.start_point, piece.samples[0]), (piece.end_point, piece.samples[piece.samples.len() - 1])] {
            let twice = open_pieces.iter().enumerate().any(|(j, other)| {
                let others = if i == j {
                    vec![]
                } else {
                    vec![(other.start_point, other.samples[0]), (other.end_point, other.samples[other.samples.len() - 1])]
                };
                others.iter().any(|(p, q)| p.sub(end.0).length() <= vertex_band && q.sub(end.1).length() <= uv_match)
            });
            if twice {
                return Ok(Err(Decline::EndTwiceUsed(piece.id)));
            }
        }
    }

    // Open pieces: split or join.
    for piece in &open_pieces {
        let locate = |point: Vec3, at: Vec2, regions: &[Region<'_>]| -> Option<(usize, usize, usize)> {
            let mut best: Option<((usize, usize, usize), f64)> = None;
            for (r, region) in regions.iter().enumerate() {
                for (l, l_record) in std::iter::once(&region.outer).chain(region.holes.iter()).enumerate() {
                    for (k, entry) in l_record.entries.iter().enumerate() {
                        if entry.start_point.sub(point).length() > vertex_band {
                            continue;
                        }
                        let distance = entry.start().sub(at).length();
                        if distance <= uv_match && best.is_none_or(|(_, d)| distance < d) {
                            best = Some(((r, l, k), distance));
                        }
                    }
                }
            }
            best.map(|(found, _)| found)
        };
        let Some((ra, la, ka)) = locate(piece.start_point, piece.samples[0], &regions) else {
            return Ok(Err(Decline::EndNotOnLoop(piece.id)));
        };
        let Some((rb, lb, kb)) = locate(piece.end_point, piece.samples[piece.samples.len() - 1], &regions) else {
            return Ok(Err(Decline::EndNotOnLoop(piece.id)));
        };
        if ra != rb {
            return Ok(Err(Decline::CrossRegion(piece.id)));
        }
        if la == lb && ka == kb {
            return Ok(Err(Decline::SameJunction(piece.id)));
        }
        let forward = cut_entry(piece.id, piece.pcurve, &piece.samples, piece.start_point, piece.end_point, piece.key);
        let backward = forward.reversed();
        insert_chord(&mut regions, ra, (la, ka), (lb, kb), vec![forward], vec![backward], outer_sign);
    }

    // Junction chords: a path of the remaining arcs from a loop junction
    // through vertices on no loop to a loop junction (minted vertex ids),
    // inserted as one chord into the region its interior lies in; its interior
    // vertices are loop junctions from then on.
    let mut remaining = chord_arcs;
    while !remaining.is_empty() {
        let first_id = match &remaining[0].0.side { Side::Cut { piece_id, .. } => *piece_id, Side::Boundary { .. } => 0 };
        let mut path: Option<Vec<(usize, bool)>> = None;
        'search: for start in 0..remaining.len() {
            for forward in [true, false] {
                let (entry, start_id, end_id) = &remaining[start];
                let (from_point, from_id, mut at_id, mut at_point) = if forward {
                    (entry.start_point, *start_id, *end_id, entry.end_point)
                } else {
                    (entry.end_point, *end_id, *start_id, entry.start_point)
                };
                if !on_a_loop(from_point, &regions) {
                    continue;
                }
                let mut steps = vec![(start, forward)];
                let mut visited = vec![from_id];
                loop {
                    if on_a_loop(at_point, &regions) {
                        path = Some(steps);
                        break 'search;
                    }
                    if visited.contains(&at_id) {
                        break;
                    }
                    visited.push(at_id);
                    let next = remaining.iter().enumerate().find(|(k, (_, s, e))| {
                        !steps.iter().any(|(used, _)| used == k) && (*s == at_id || *e == at_id)
                    });
                    let Some((k, (next_entry, s, e))) = next else {
                        break;
                    };
                    let along = *s == at_id;
                    steps.push((k, along));
                    (at_id, at_point) = if along { (*e, next_entry.end_point) } else { (*s, next_entry.start_point) };
                }
            }
        }
        let Some(steps) = path else {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        };
        let forward: Vec<Entry<'_>> = steps.iter().map(|(k, along)| if *along { remaining[*k].0.clone() } else { remaining[*k].0.reversed() }).collect();
        let backward: Vec<Entry<'_>> = forward.iter().rev().map(Entry::reversed).collect();
        let mut used: Vec<usize> = steps.iter().map(|(k, _)| *k).collect();
        used.sort_unstable_by(|a, b| b.cmp(a));
        for k in used {
            remaining.remove(k);
        }
        // The region: the one whose interior holds the chord's first arc.
        let first_samples = &forward[0].samples;
        let middle = first_samples.len() / 2;
        let probe = first_samples[middle - 1].add(first_samples[middle]).scale(0.5);
        let Some(r) = regions.iter().position(|region| {
            point_in_polygon(probe, &region.outer.polygon()) && !region.holes.iter().any(|hole| point_in_polygon(probe, &hole.polygon()))
        }) else {
            return Ok(Err(Decline::RingOutside(first_id)));
        };
        // Its two junctions in that region, each matched exactly once.
        let locate = |point: Vec3, at: Vec2| -> Option<(usize, usize)> {
            let mut found = None;
            let region = &regions[r];
            for (l, l_record) in std::iter::once(&region.outer).chain(region.holes.iter()).enumerate() {
                for (k, entry) in l_record.entries.iter().enumerate() {
                    if entry.start_point.sub(point).length() <= vertex_band && entry.start().sub(at).length() <= uv_match {
                        if found.is_some() {
                            return None;
                        }
                        found = Some((l, k));
                    }
                }
            }
            found
        };
        let last = &forward[forward.len() - 1];
        let (Some((la, ka)), Some((lb, kb))) = (locate(forward[0].start_point, forward[0].start()), locate(last.end_point, last.end())) else {
            return Ok(Err(Decline::EndNotOnLoop(first_id)));
        };
        if la == lb && ka == kb {
            return Ok(Err(Decline::SameJunction(first_id)));
        }
        insert_chord(&mut regions, r, (la, ka), (lb, kb), forward, backward, outer_sign);
    }

    // The touching ring: spliced into its loop at the junction against the
    // material winding (the pinched loop the arrangement walks), and the disc
    // inside it a new region; a hole inside the disc moves into it.
    if let Some(ring) = &touch {
        let mut found: Option<(usize, usize, usize)> = None;
        let mut best = f64::INFINITY;
        for (r, region) in regions.iter().enumerate() {
            for (l, l_record) in std::iter::once(&region.outer).chain(region.holes.iter()).enumerate() {
                for (k, entry) in l_record.entries.iter().enumerate() {
                    if entry.start_point.sub(ring.point).length() > vertex_band {
                        continue;
                    }
                    let distance = entry.start().sub(ring.samples[0]).length();
                    if distance <= uv_match && distance < best {
                        best = distance;
                        found = Some((r, l, k));
                    }
                }
            }
        }
        let Some((r, l, k)) = found else {
            return Ok(Err(Decline::EndNotOnLoop(ring.id)));
        };
        let probe = ring.samples[ring.samples.len() / 2];
        let region = &regions[r];
        let inside_outer = point_in_polygon(probe, &region.outer.polygon());
        let in_a_hole = region.holes.iter().any(|hole| point_in_polygon(probe, &hole.polygon()));
        if !inside_outer || in_a_hole {
            return Ok(Err(Decline::RingOutside(ring.id)));
        }
        let along = LoopUnderConstruction { entries: ring.entries.clone() };
        let against = LoopUnderConstruction { entries: ring.entries.iter().rev().map(Entry::reversed).collect() };
        let (as_outer, as_hole) = if (ring.area >= 0.0) == outer_sign { (along, against) } else { (against, along) };
        let Region { outer, holes } = regions.swap_remove(r);
        let ring_polygon = as_outer.polygon();
        let host = if l == 0 { outer.clone() } else { holes[l - 1].clone() };
        let mut pinched = host.arc(k, k);
        pinched.extend(as_hole.entries);
        let pinched = LoopUnderConstruction { entries: pinched };
        let (mut outer, holes) = (outer, holes);
        let mut inside_ring = Vec::new();
        let mut outside_ring = Vec::new();
        for (index, hole) in holes.into_iter().enumerate() {
            if index + 1 == l {
                outside_ring.push(pinched.clone());
            } else if point_in_polygon(hole.entries[0].start(), &ring_polygon) {
                inside_ring.push(hole);
            } else {
                outside_ring.push(hole);
            }
        }
        if l == 0 {
            outer = pinched;
        }
        regions.push(Region { outer, holes: outside_ring });
        regions.push(Region { outer: as_outer, holes: inside_ring });
    }
    // Rings, largest first, each a new region and a hole of the region that
    // contains it; a hole already inside it moves into it.
    rings.sort_by(|a, b| b.area.abs().total_cmp(&a.area.abs()));
    for ring in &rings {
        let probe = ring.samples[0];
        let containing = regions.iter().position(|region| {
            point_in_polygon(probe, &region.outer.polygon()) && !region.holes.iter().any(|hole| point_in_polygon(probe, &hole.polygon()))
        });
        let Some(r) = containing else {
            return Ok(Err(Decline::RingOutside(ring.id)));
        };
        let along = LoopUnderConstruction { entries: ring.entries.clone() };
        let against = LoopUnderConstruction { entries: ring.entries.iter().rev().map(Entry::reversed).collect() };
        let (as_outer, as_hole) = if (ring.area >= 0.0) == outer_sign {
            (along, against)
        } else {
            (against, along)
        };
        let ring_polygon = as_outer.polygon();
        let mut inside_ring = Vec::new();
        let mut outside_ring = Vec::new();
        for hole in std::mem::take(&mut regions[r].holes) {
            if point_in_polygon(hole.entries[0].start(), &ring_polygon) {
                inside_ring.push(hole);
            } else {
                outside_ring.push(hole);
            }
        }
        outside_ring.push(as_hole);
        regions[r].holes = outside_ring;
        regions.push(Region { outer: as_outer, holes: inside_ring });
    }

    // Records, in the arrangement's order: ascending outer area, each
    // region's holes in the order its walk finds their cycles (ascending least
    // chain key), each loop started where `build_loop` starts it.
    let mut ordered: Vec<(f64, Region<'_>)> = regions.into_iter().map(|region| (region.outer.area().abs(), region)).collect();
    ordered.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut fragments = Vec::with_capacity(ordered.len());
    for (outer_area, mut region) in ordered {
        // The arrangement's key ([`super::types::FragmentOrder`]): every chain
        // here is used whole, so a loop's sorted chain keys order regions as
        // the arrangement's sorted (chain, segment) lists do.
        let loop_key = |l: &LoopUnderConstruction<'_>| {
            let mut keys: Vec<(u64, u64)> = l.entries.iter().map(|entry| (entry.key as u64, 0)).collect();
            keys.sort_unstable();
            keys
        };
        let mut hole_keys: Vec<Vec<(u64, u64)>> = region.holes.iter().map(loop_key).collect();
        hole_keys.sort_unstable();
        let order = FragmentOrder { area: outer_area, key: (loop_key(&region.outer), hole_keys) };
        region.holes.sort_by_key(|hole| hole.min_key());
        region.outer.rotate_like_arrangement(reverse_loops);
        for hole in &mut region.holes {
            hole.rotate_like_arrangement(reverse_loops);
        }
        // The interior-point search offsets each outer edge to its LEFT, which
        // is inward only on a counter-clockwise polygon (the arrangement's
        // cycles always are); a face drawn clockwise in its chart is read
        // reversed here, for the search alone.
        let mut outer_polygon = region.outer.polygon();
        let mut hole_polygons: Vec<Vec<Vec2>> = region.holes.iter().map(|hole| hole.polygon()).collect();
        if signed_area(&outer_polygon) < 0.0 {
            outer_polygon.reverse();
            for hole in &mut hole_polygons {
                hole.reverse();
            }
        }
        let area = region.outer.area() + region.holes.iter().map(|hole| hole.area()).sum::<f64>();
        let candidates = interior_points_in_polygons(&outer_polygon, &hole_polygons, area, 3);
        let debug_no_interior = |candidates: &[Vec2], class: Option<PolygonClass>| {
            if std::env::var_os("BREP_DEBUG_LOOP_SPLIT").is_some() {
                eprintln!(
                    "loop-split face {} NoInterior: outer area {:.4e} ({} points) holes {:?} area {area:.4e} candidates {:?} class {class:?} outer_sign {outer_sign}",
                    face.id,
                    signed_area(&outer_polygon),
                    outer_polygon.len(),
                    hole_polygons.iter().map(|h| format!("{:.4e}/{}", signed_area(h), h.len())).collect::<Vec<_>>(),
                    candidates.iter().map(|c| format!("({:.4},{:.4})", c.x, c.y)).collect::<Vec<_>>(),
                );
            }
        };
        let Some(&test_uv) = candidates.first() else {
            debug_no_interior(&candidates, None);
            return Ok(Err(Decline::NoInterior));
        };
        let class = parameter_point_in_face(face, test_uv, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")?;
        if class != PolygonClass::Inside {
            debug_no_interior(&candidates, Some(class));
            return Ok(Err(Decline::NoInterior));
        }
        let mut extra_test_points = Vec::new();
        for &candidate in candidates.iter().skip(1) {
            if parameter_point_in_face(face, candidate, 1e-9).or_refuse(KernelStage::Fragment, "parameter_point_in_face")? == PolygonClass::Inside {
                extra_test_points.push(face.surface.evaluate(candidate.x, candidate.y).or_refuse(KernelStage::Fragment, "evaluate")?);
            }
        }
        let mut record_loops = Vec::with_capacity(1 + region.holes.len());
        for l in std::iter::once(&region.outer).chain(region.holes.iter()) {
            let mut coedges = Vec::with_capacity(l.entries.len());
            for entry in &l.entries {
                coedges.push(match &entry.side {
                    Side::Boundary { coedge } => FragmentCoedge {
                        source: FragmentEdgeSource::Boundary { operand, edge_id: coedge.edge_id },
                        forward: coedge.forward,
                        pcurve: coedge.pcurve.clone(),
                    },
                    Side::Cut { piece_id, forward, pcurve, shared_ring } => FragmentCoedge {
                        source: match shared_ring {
                            Some((operand, edge_id, _)) => FragmentEdgeSource::SharedBoundary { operand: *operand, edge_id: *edge_id },
                            None => FragmentEdgeSource::Imprint { piece_id: *piece_id },
                        },
                        forward: shared_ring.map_or(*forward, |(_, _, aligned)| *forward == aligned),
                        pcurve: if *forward { (*pcurve).clone() } else { pcurve.reversed().or_refuse(KernelStage::Fragment, "reversed")? },
                    },
                });
            }
            record_loops.push(FragmentLoop { coedges });
        }
        fragments.push(FaceFragmentRecord {
            operand,
            source_face_id: face.id,
            surface: face.surface.clone(),
            same_sense: face.same_sense,
            loops: record_loops,
            test_point: face.surface.evaluate(test_uv.x, test_uv.y).or_refuse(KernelStage::Fragment, "evaluate")?,
            test_uv,
            extra_test_points,
            order: Some(order),
        });
    }
    Ok(Ok(fragments))
}

/// One coedge as a comparison key: its source, direction and pcurve bytes.
fn coedge_key(coedge: &FragmentCoedge) -> String {
    let source = match &coedge.source {
        FragmentEdgeSource::Boundary { operand, edge_id } => format!("B{operand}:{edge_id}"),
        FragmentEdgeSource::SharedBoundary { operand, edge_id } => format!("S{operand}:{edge_id}"),
        FragmentEdgeSource::Imprint { piece_id } => format!("I{piece_id}"),
        FragmentEdgeSource::Derived { .. } => "D".to_string(),
    };
    format!("{source}/{}/{}", coedge.forward, serde_json::to_string(&coedge.pcurve).unwrap_or_default())
}

/// A loop as a rotation-invariant key: the lexicographically least rotation
/// of its coedge keys.
fn loop_key(loop_record: &FragmentLoop) -> String {
    let keys: Vec<String> = loop_record.coedges.iter().map(coedge_key).collect();
    let n = keys.len();
    (0..n)
        .map(|start| (0..n).map(|k| keys[(start + k) % n].as_str()).collect::<Vec<_>>().join("|"))
        .min()
        .unwrap_or_default()
}

fn fragment_key(fragment: &FaceFragmentRecord) -> String {
    let mut loops: Vec<String> = fragment.loops.iter().map(loop_key).collect();
    loops.sort();
    format!("{}/{}", fragment.same_sense, loops.join("||"))
}

/// The chart polygons of a record's loops: the outer (largest area) first.
fn record_polygons(fragment: &FaceFragmentRecord) -> Result<(Vec<Vec2>, Vec<Vec<Vec2>>), KernelRefusal> {
    let mut polygons = Vec::with_capacity(fragment.loops.len());
    for loop_record in &fragment.loops {
        let mut polygon = Vec::new();
        for coedge in &loop_record.coedges {
            let samples = sample_chain(&coedge.pcurve)?;
            polygon.extend_from_slice(&samples[..samples.len().saturating_sub(1)]);
        }
        polygons.push(polygon);
    }
    let outer = (0..polygons.len())
        .max_by(|a, b| signed_area(&polygons[*a]).abs().total_cmp(&signed_area(&polygons[*b]).abs()))
        .unwrap_or(0);
    let outer_polygon = polygons.swap_remove(outer);
    Ok((outer_polygon, polygons))
}

fn record_contains(fragment: &FaceFragmentRecord, point: Vec2) -> Result<bool, KernelRefusal> {
    let (outer, holes) = record_polygons(fragment)?;
    Ok(point_in_polygon(point, &outer) && !holes.iter().any(|hole| point_in_polygon(point, hole)))
}

/// What [`fragments_agree`] found beyond agreement: how many of the split's
/// fragments the arrangement wrote byte for byte in the same place (the same
/// position, loops in the same order, every loop started at the same coedge)
/// and how many carry its test point exactly. Agreement does not need either;
/// a face where both counts equal `fragments` leaves the pipeline the same
/// records whichever path built them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Agreement {
    pub(super) fragments: usize,
    pub(super) same_order: usize,
    pub(super) same_test_uv: usize,
}

impl std::fmt::Display for Agreement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "agree {} fragments ({} in the arrangement's loop order, {} with its test point)",
            self.fragments, self.same_order, self.same_test_uv
        )
    }
}

/// A fragment's loops as written: every coedge key in record order.
fn ordered_loop_keys(fragment: &FaceFragmentRecord) -> Vec<Vec<String>> {
    fragment.loops.iter().map(|l| l.coedges.iter().map(coedge_key).collect()).collect()
}

/// The differential check: the loop split's records against the
/// arrangement's. Same count; every loop-split fragment has exactly one
/// arrangement fragment with the same loops (cyclic sequences of source,
/// direction and pcurve bytes) and the same sense; each path's test point lies
/// in the other's region. `Err` names the first disagreement; `Ok` counts how
/// closely the records match beyond that.
pub(super) fn fragments_agree(split: &[FaceFragmentRecord], arranged: &[FaceFragmentRecord]) -> Result<Agreement, String> {
    if split.len() != arranged.len() {
        return Err(format!("fragment count {} vs arrangement {}", split.len(), arranged.len()));
    }
    let mut agreement = Agreement { fragments: split.len(), same_order: 0, same_test_uv: 0 };
    let mut unmatched: Vec<(usize, String, &FaceFragmentRecord)> =
        arranged.iter().enumerate().map(|(at, fragment)| (at, fragment_key(fragment), fragment)).collect();
    for (position, fragment) in split.iter().enumerate() {
        let key = fragment_key(fragment);
        let Some(found) = unmatched.iter().position(|(_, other, _)| *other == key) else {
            let loops: Vec<String> = fragment
                .loops
                .iter()
                .map(|l| {
                    l.coedges
                        .iter()
                        .map(|c| match &c.source {
                            FragmentEdgeSource::Boundary { edge_id, .. } => format!("e{edge_id}{}", if c.forward { "+" } else { "-" }),
                            FragmentEdgeSource::SharedBoundary { edge_id, .. } => format!("s{edge_id}{}", if c.forward { "+" } else { "-" }),
                            FragmentEdgeSource::Imprint { piece_id } => format!("p{piece_id}{}", if c.forward { "+" } else { "-" }),
                            FragmentEdgeSource::Derived { .. } => "derived".to_string(),
                        })
                        .collect::<Vec<_>>()
                        .join(",")
                })
                .collect();
            let arranged_loops: Vec<String> = arranged
                .iter()
                .map(|f| {
                    f.loops
                        .iter()
                        .map(|l| {
                            l.coedges
                                .iter()
                                .map(|c| match &c.source {
                                    FragmentEdgeSource::Boundary { edge_id, .. } => format!("e{edge_id}{}", if c.forward { "+" } else { "-" }),
                                    FragmentEdgeSource::SharedBoundary { edge_id, .. } => format!("s{edge_id}{}", if c.forward { "+" } else { "-" }),
                                    FragmentEdgeSource::Imprint { piece_id } => format!("p{piece_id}{}", if c.forward { "+" } else { "-" }),
                                    FragmentEdgeSource::Derived { .. } => "derived".to_string(),
                                })
                                .collect::<Vec<_>>()
                                .join(",")
                        })
                        .collect::<Vec<_>>()
                        .join(" ; ")
                })
                .collect();
            return Err(format!(
                "split fragment {position} [{}] has no arrangement twin among [{}]",
                loops.join(" ; "),
                arranged_loops.join(" // ")
            ));
        };
        let (twin_position, _, twin) = unmatched.swap_remove(found);
        let split_in_twin = record_contains(twin, fragment.test_uv).map_err(|e| e.to_string())?;
        let twin_in_split = record_contains(fragment, twin.test_uv).map_err(|e| e.to_string())?;
        if !split_in_twin || !twin_in_split {
            return Err(format!(
                "fragment {position}: test points disagree (split ({:.6},{:.6}) in arrangement region: {split_in_twin}; arrangement ({:.6},{:.6}) in split region: {twin_in_split})",
                fragment.test_uv.x, fragment.test_uv.y, twin.test_uv.x, twin.test_uv.y
            ));
        }
        if twin_position == position && ordered_loop_keys(fragment) == ordered_loop_keys(twin) {
            agreement.same_order += 1;
        }
        if fragment.test_uv.x == twin.test_uv.x && fragment.test_uv.y == twin.test_uv.y {
            agreement.same_test_uv += 1;
        }
    }
    Ok(agreement)
}

/// `BREP_LOOP_SPLIT_CENSUS=<file>`: one line per face the check mode read.
pub(super) fn loop_split_census(line: impl FnOnce() -> String) {
    if let Ok(path) = std::env::var("BREP_LOOP_SPLIT_CENSUS") {
        use std::io::Write;
        if let Ok(mut file) = std::fs::OpenOptions::new().create(true).append(true).open(path) {
            let label = std::env::var("BREP_CENSUS_LABEL").unwrap_or_default();
            let _ = writeln!(file, "loop-split census: label {label} {}", line());
        }
    }
}
