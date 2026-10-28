//! Booleans and offsets on one arrangement.
//!
//! Booleans classify every piece of the arrangement of both operands by the
//! winding numbers on its two sides: a horizontal ray from a probe point a
//! few point epsilons off the piece (left along its normal, right against
//! it) counts the crossings of every segment of an operand EXCEPT the
//! piece's own coincidence group, and the group's members add the crossing
//! of the group's curve as seen from that probe, times their sense. The
//! probes stand off the piece because the arrangement perturbs piece
//! geometry by up to an epsilon (merged vertices, endpoint incidences), so a
//! ray from the piece itself can meet a neighbour's vertex on both sides.
//!
//! Offsets use the level set: the boundary of the offset region is the set of
//! points at distance exactly |d| from the input boundary on the requested
//! side. Every such point lies on a capsule boundary of one input segment
//! (the parallel curves and the end circles), so the raw curve is the union of
//! all capsule boundaries, arranged; a piece is kept when its midpoint's
//! distance to the input boundary is |d| within the arrangement's slack and
//! it lies on the right side, and it is oriented from the direction to its
//! nearest boundary point.
//!
//! Both operations check their result against invariants it cannot violate
//! (area bounds; witness points an offset cannot reach) and report an
//! [`PlanarError::Inconsistent`] instead of returning a wrong region.

use super::arrange::{arrange, Dsu, Input, Piece};
use super::{crossings, dir, Loop, PlanarError, Region, Segment, Vec2};
use crate::model::Point2;
use crate::numeric::Tolerance;
use std::f64::consts::{PI, TAU};

/// The set operation of [`boolean`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BoolOp {
    Union,
    Intersection,
    /// `a` minus `b`.
    Difference,
    /// Symmetric difference.
    Xor,
}

impl BoolOp {
    fn keep(self, in_a: bool, in_b: bool) -> bool {
        match self {
            BoolOp::Union => in_a || in_b,
            BoolOp::Intersection => in_a && in_b,
            BoolOp::Difference => in_a && !in_b,
            BoolOp::Xor => in_a != in_b,
        }
    }
}

/// A kept, oriented piece ready for stitching.
#[derive(Clone, Debug)]
struct Kept {
    seg: Segment,
    v0: usize,
    v1: usize,
    /// How far the piece's midpoint misses the level set it was kept for
    /// (zero for a boolean piece): the better of two near-coincident pieces
    /// is the one with the smaller miss.
    miss: f64,
    /// On the level set within rounding (every boolean piece is): a strict
    /// piece left without a continuation is an arrangement failure.
    strict: bool,
}

/// Pieces shorter than this many point epsilons are stubs between vertices
/// the arrangement kept apart by rounding (a few epsilons, so merging did
/// not take them); stitching identifies their ends and bridges the gap.
const STUB_EPS: f64 = 16.0;

/// How far off an offset piece its two sides are probed, in point
/// epsilons: the region side must move the distance by at least half of
/// it, which input noise of an epsilon cannot fake.
const GAIN_EPS: f64 = 8.0;

/// The arrangement's slack in point epsilons: how far a piece's geometry
/// may be perturbed by vertex merging and by endpoint incidences taken
/// within the distance epsilon, and the input's own slack (arc end radii
/// may differ by the distance epsilon). Booleans probe a piece's sides this
/// far off it (a ray cast from the piece itself can meet a neighbour's
/// vertex on both sides; a probe off the piece sees the sides as the exact
/// limit points would, and coincident pieces are grouped within one
/// epsilon, so nothing but the group's own curve lies closer than this to
/// the midpoint); offsets keep a piece whose midpoint misses the distance
/// by less than this; the result invariants allow it along the boundary.
const SLACK_EPS: f64 = 4.0;

/// Boolean of two regions.
pub fn boolean(a: &Region, b: &Region, op: BoolOp, tol: &Tolerance) -> Result<Region, PlanarError> {
    tol.validate().map_err(|e| PlanarError::InvalidInput(e.to_string()))?;
    let mut inputs: Vec<Input> = Vec::new();
    for (operand, r) in [(0u8, a), (1u8, b)] {
        for (i, seg) in r.canonical_segments(tol).into_iter().enumerate() {
            seg.validate(tol)?;
            inputs.push(Input { seg, operand, source: i });
        }
    }
    let arr = arrange(&inputs, tol);
    let mut kept: Vec<Kept> = Vec::new();
    let mut set_area = [0.0f64; 2];
    let input_segments: Vec<Segment> = inputs.iter().map(|i| i.seg).collect();
    let near_inputs = NearestGrid::new(&input_segments);
    // Per-operand row buckets for the ray casts.
    let rows = [Rows::build(&inputs, 0), Rows::build(&inputs, 1)];
    for group in &arr.groups {
        let rep = &arr.pieces[group[0]];
        let m = rep.seg.midpoint();
        // The two sides are probed off the piece's SOURCE curve (the input
        // segment it was cut from, of which it is an exact sub-curve up to
        // the merged end vertices), at its point nearest the piece midpoint,
        // along its normal; and each operand's winding number there is the
        // crossing count of ALL its input segments, the piece's own source
        // included. The probe distance is half the distance to the nearest
        // curve of another source, at most the arrangement's slack: a probe
        // never crosses a neighbour, however close (lane C6's capsule unions:
        // a 6.75e-3 line tangent to another operand's arc within 1.9e-6 was
        // counted as that operand's boundary by a 4e-6 probe), and probing
        // the source rather than the piece makes a piece tilted by vertex
        // merging irrelevant. (A ray through the piece's midpoint needed the
        // group excluded and its contribution added; from a probe the ray can
        // cross the source beside the piece — the sibling of a nearly
        // horizontal piece — and excluding the source lost that crossing:
        // lane C20's C19-08 swept union.)
        let source = &inputs[rep.source].seg;
        let (q, u) = source.nearest(m);
        let normal = source.tangent_at(u).perp();
        let members: Vec<usize> = group.iter().map(|&k| arr.pieces[k].source).collect();
        let clearance = near_inputs.distance_excluding(q, &|i| members.contains(&i)).unwrap_or(f64::INFINITY);
        let probe = (0.5 * clearance).min(SLACK_EPS * tol.point_eps);
        let p_l = q.add(normal.scale(probe));
        let p_r = q.sub(normal.scale(probe));
        let mut wn_l = [0i32; 2];
        let mut wn_r = [0i32; 2];
        for operand in 0..2usize {
            wn_l[operand] = crossings(rows[operand].row(p_l.y).iter().map(|&i| &inputs[i].seg), p_l, true);
            wn_r[operand] = crossings(rows[operand].row(p_r.y).iter().map(|&i| &inputs[i].seg), p_r, true);
        }
        // The operands' own set areas, from the same classification.
        for operand in 0..2usize {
            match (wn_l[operand] > 0, wn_r[operand] > 0) {
                (true, false) => set_area[operand] += piece_area(&rep.seg),
                (false, true) => set_area[operand] -= piece_area(&rep.seg),
                _ => {}
            }
        }
        let keep_l = op.keep(wn_l[0] > 0, wn_l[1] > 0);
        let keep_r = op.keep(wn_r[0] > 0, wn_r[1] > 0);
        if keep_l == keep_r {
            continue;
        }
        if keep_l {
            kept.push(Kept { seg: rep.seg, v0: rep.v0, v1: rep.v1, miss: 0.0, strict: true });
        } else {
            kept.push(Kept { seg: rep.seg.reversed(), v0: rep.v1, v1: rep.v0, miss: 0.0, strict: true });
        }
    }
    let loops = stitch(kept, &arr.vertices, tol)?;
    let result = Region { loops };
    check_boolean(a, b, op, set_area, &result, tol)?;
    Ok(result)
}

/// The signed area a boundary piece contributes to the region on its left
/// (the shoelace term of its chord plus the arc bulge); a closed loop's
/// pieces sum to its signed area.
fn piece_area(seg: &Segment) -> f64 {
    0.5 * seg.start.cross(seg.end) + seg.bulge_area()
}

/// What a boolean cannot do: produce an area outside the bounds set by the
/// operands' set areas, measured by the same classification (a union holds
/// each operand and at most both, an intersection at most the smaller, a
/// difference at most `a` and at least `a` minus `b`, a symmetric
/// difference between their difference and their sum). A violation beyond
/// the boundary slack times the perimeters is reported, never returned as a
/// region; a boundary piece left without continuation is reported by the
/// stitcher before that.
pub(crate) fn check_boolean(a: &Region, b: &Region, op: BoolOp, set_area: [f64; 2], result: &Region, tol: &Tolerance) -> Result<(), PlanarError> {
    let (aa, ab, ar) = (set_area[0], set_area[1], result.area());
    let perimeter: f64 = a.loops.iter().chain(b.loops.iter()).map(Loop::length).sum();
    let slack = tol.area_eps + 2.0 * SLACK_EPS * tol.point_eps * perimeter;
    let (lo, hi) = match op {
        BoolOp::Union => (aa.max(ab), aa + ab),
        BoolOp::Intersection => (0.0, aa.min(ab)),
        BoolOp::Difference => ((aa - ab).max(0.0), aa),
        BoolOp::Xor => ((aa - ab).abs(), aa + ab),
    };
    if ar < lo - slack || ar > hi + slack {
        return Err(PlanarError::Inconsistent(format!(
            "{op:?} of regions with areas {aa} and {ab} produced area {ar}, outside [{lo}, {hi}]"
        )));
    }
    Ok(())
}

/// Offset a region by `distance` (positive grows it).
pub fn offset_region(region: &Region, distance: f64, tol: &Tolerance) -> Result<Region, PlanarError> {
    tol.validate().map_err(|e| PlanarError::InvalidInput(e.to_string()))?;
    if !distance.is_finite() {
        return Err(PlanarError::NonFinite { what: "offset distance".into() });
    }
    let mut normalized = region.clone();
    for l in &mut normalized.loops {
        l.validate(tol)?;
    }
    if distance.abs() <= tol.distance_eps {
        return Ok(normalized);
    }
    let d = distance.abs();
    let boundary = normalized.canonical_segments(tol);
    // Raw curve: capsule boundaries of every segment, one circle per vertex
    // (collinear vertices carry no circle: they are merged away first).
    let mut raw: Vec<Segment> = Vec::new();
    for l in &normalized.loops {
        let l = super::simplified(l, tol);
        for s in &l.segments {
            for k in 0..4 {
                let p0 = s.start.add(dir(k as f64 * PI / 2.0).scale(d));
                let p1 = s.start.add(dir((k + 1) as f64 * PI / 2.0).scale(d));
                raw.push(Segment::arc(p0, p1, s.start, true));
            }
            match s.arc {
                None => {
                    let n = s.end.sub(s.start).normalized().expect("validated").perp().scale(d);
                    raw.push(Segment::line(s.start.add(n), s.end.add(n)));
                    raw.push(Segment::line(s.start.sub(n), s.end.sub(n)));
                }
                Some(a) => {
                    let r = s.radius();
                    let (a0, sweep) = s.angles();
                    let a1 = a0 + sweep;
                    let on = |rad: f64, ang: f64| a.center.add(dir(ang).scale(rad));
                    raw.extend(Segment::arc(on(r + d, a0), on(r + d, a1), a.center, a.ccw).canonical(tol));
                    let inner = r - d;
                    if inner > tol.point_eps {
                        raw.extend(Segment::arc(on(inner, a0), on(inner, a1), a.center, a.ccw).canonical(tol));
                    } else if inner < -tol.point_eps {
                        // Points at distance d past the centre: the inverted arc.
                        raw.extend(Segment::arc(on(-inner, a0 + PI), on(-inner, a1 + PI), a.center, a.ccw).canonical(tol));
                    }
                }
            }
        }
    }
    let inputs: Vec<Input> = raw.into_iter().enumerate().map(|(i, seg)| Input { seg, operand: 0, source: i }).collect();
    let arr = arrange(&inputs, tol);
    let keep_eps = keep_tolerance(d, tol);
    let boundary_inputs: Vec<Input> = boundary.iter().enumerate().map(|(i, &seg)| Input { seg, operand: 0, source: i }).collect();
    let boundary_rows = Rows::build(&boundary_inputs, 0);
    let nearest = NearestGrid::new(&boundary);
    let mut kept: Vec<Kept> = Vec::new();
    for group in &arr.groups {
        let rep = &arr.pieces[group[0]];
        let m = rep.seg.midpoint();
        let Some(dist) = nearest.distance(m) else { continue };
        if (dist - d).abs() > keep_eps {
            continue;
        }
        let inside = crossings(boundary_rows.row(m.y).iter().map(|&i| &boundary[i]), m, true) > 0;
        if (distance < 0.0) != inside {
            continue;
        }
        // The region is on the side where the distance moves the right way
        // (up for an inward offset, down for an outward one) and not on the
        // other: where two sides of the input meet at exactly 2|d| (a slot
        // shrunk by its radius, two bodies grown to touch) the level set is
        // a curve with the region on neither or both sides, not a boundary.
        let sense = if distance < 0.0 { 1.0 } else { -1.0 };
        let gain_probe = GAIN_EPS * tol.point_eps;
        let normal = rep.seg.tangent_at(0.5).perp();
        let gain = |p: Point2| nearest.distance(p).map(|dp| sense * (dp - dist) >= 0.5 * gain_probe).unwrap_or(false);
        let (left_in, right_in) = (gain(m.add(normal.scale(gain_probe))), gain(m.sub(normal.scale(gain_probe))));
        let miss = (dist - d).abs();
        let strict = miss <= strict_tolerance(d);
        match (left_in, right_in) {
            (true, false) => kept.push(Kept { seg: rep.seg, v0: rep.v0, v1: rep.v1, miss, strict }),
            (false, true) => kept.push(Kept { seg: rep.seg.reversed(), v0: rep.v1, v1: rep.v0, miss, strict }),
            _ => {}
        }
    }
    let loops = stitch(kept, &arr.vertices, tol)?;
    let result = Region { loops };
    check_offset(&normalized, distance, &result, tol)?;
    Ok(result)
}

/// What an offset cannot do: lose a point of the region farther than the
/// shrink distance from its boundary, gain a point of a hole farther than
/// the growth distance from it, shrink while growing or grow while
/// shrinking (a level-set piece left without continuation is reported by
/// the stitcher before that).
/// Every input loop is witnessed by a point inside it (its centroid, else its
/// box centre) when that point is farther from the whole boundary than the
/// distance the offset can reach; a result that contradicts a witness is an
/// arrangement failure and is reported, never returned as a region.
pub(crate) fn check_offset(input: &Region, distance: f64, result: &Region, tol: &Tolerance) -> Result<(), PlanarError> {
    let d = distance.abs();
    let slack = SLACK_EPS * tol.point_eps.max(tol.distance_eps);
    let perimeter: f64 = input.loops.iter().map(Loop::length).sum();
    let (ai, ar) = (input.area(), result.area());
    let area_slack = tol.area_eps + 2.0 * slack * perimeter;
    if (distance > 0.0 && ar < ai - area_slack) || (distance < 0.0 && ar > ai + area_slack) {
        return Err(PlanarError::Inconsistent(format!("offset by {distance} took the area from {ai} to {ar}")));
    }
    for l in &input.loops {
        let Some((w, dist)) = loop_witness(l, input, tol) else { continue };
        let reach = if l.is_ccw() == (distance > 0.0) { slack } else { d + slack };
        if dist <= reach {
            continue;
        }
        let inside = result.contains(w, tol);
        if inside != l.is_ccw() {
            return Err(PlanarError::Inconsistent(format!(
                "offset by {distance} {} ({}, {}) at distance {dist} from the boundary",
                if l.is_ccw() { "lost the region point" } else { "filled the hole point" },
                w.x,
                w.y
            )));
        }
    }
    Ok(())
}

/// A point inside loop `l` (its polygon centroid, else its box centre) and
/// its distance to the whole boundary of `region`.
fn loop_witness(l: &Loop, region: &Region, tol: &Tolerance) -> Option<(Point2, f64)> {
    let pts = l.discretize(tol.sample_eps);
    let mut area = 0.0;
    let mut cx = 0.0;
    let mut cy = 0.0;
    for i in 0..pts.len() {
        let (p, q) = (pts[i], pts[(i + 1) % pts.len()]);
        let c = p.cross(q);
        area += c;
        cx += (p.x + q.x) * c;
        cy += (p.y + q.y) * c;
    }
    let (lo, hi) = l.bounds();
    let mut candidates = Vec::new();
    if area.abs() > tol.area_eps {
        candidates.push(Point2::new(cx / (3.0 * area), cy / (3.0 * area)));
    }
    candidates.push(Point2::new((lo.x + hi.x) * 0.5, (lo.y + hi.y) * 0.5));
    // Inside the loop and on the loop's side of the region: an outer loop's
    // centroid can fall in a hole of the region, a hole's in an island.
    candidates
        .into_iter()
        .find(|&w| l.winding(w, tol) != 0 && region.contains(w, tol) == l.is_ccw())
        .and_then(|w| region.nearest_boundary(w).map(|(_, dist)| (w, dist)))
}

/// Segments of one operand bucketed by the horizontal rows their Y range
/// touches, so a horizontal ray cast scans one row instead of every segment.
/// Row lists keep input order, so the count is deterministic.
pub(crate) struct Rows {
    y0: f64,
    dy: f64,
    rows: Vec<Vec<usize>>,
}

impl Rows {
    pub(crate) fn build(inputs: &[Input], operand: u8) -> Self {
        let mine: Vec<usize> = (0..inputs.len()).filter(|&i| inputs[i].operand == operand).collect();
        let mut y0 = f64::INFINITY;
        let mut y1 = f64::NEG_INFINITY;
        for &i in &mine {
            let (lo, hi) = inputs[i].seg.bounds();
            y0 = y0.min(lo.y);
            y1 = y1.max(hi.y);
        }
        let n = ((mine.len() as f64).sqrt().ceil() as usize).clamp(1, 4096);
        let dy = if y1 > y0 { (y1 - y0) / n as f64 } else { 1.0 };
        let mut rows = vec![Vec::new(); n];
        for &i in &mine {
            let (lo, hi) = inputs[i].seg.bounds();
            let a = (((lo.y - y0) / dy).floor().max(0.0) as usize).min(n - 1);
            let b = (((hi.y - y0) / dy).floor().max(0.0) as usize).min(n - 1);
            for row in rows.iter_mut().take(b + 1).skip(a) {
                row.push(i);
            }
        }
        Self { y0, dy, rows }
    }

    pub(crate) fn row(&self, y: f64) -> &[usize] {
        let n = self.rows.len();
        let top = self.y0 + self.dy * n as f64;
        if !(y >= self.y0) || y > top {
            return &[];
        }
        let k = (((y - self.y0) / self.dy).floor() as usize).min(n - 1);
        &self.rows[k]
    }
}

/// How far a raw piece's midpoint may miss the level set distance and still
/// be kept: the arrangement's own slack. A piece's geometry is perturbed by
/// up to the point epsilon by vertex merging and by endpoint incidences
/// taken within the distance epsilon (a vertex circle's quadrant point a
/// fraction of an epsilon off a tangent parallel splits both there), and an
/// input arc's end radii may differ by the distance epsilon; a true piece
/// can therefore miss the distance by a few epsilons, while a piece inside a
/// neighbouring capsule misses it by a geometric amount. Below the slack the
/// two cannot be told apart, so the slack is the smallest tolerance that
/// keeps every true piece (lane C6's reproducers offset-fine-boundary.json
/// and offset-equal-radius.json lost whole loops under a 1e-9 rule).
fn keep_tolerance(d: f64, tol: &Tolerance) -> f64 {
    strict_tolerance(d).max(SLACK_EPS * tol.point_eps.max(tol.distance_eps))
}

/// Rounding alone: a piece this close to the level set distance is on it.
fn strict_tolerance(d: f64) -> f64 {
    1e-9 * d.max(1.0)
}

fn nearest_on(segments: &[Segment], p: Point2) -> Option<(Point2, f64)> {
    let mut best: Option<(Point2, f64)> = None;
    for s in segments {
        let (q, _) = s.nearest(p);
        let d = q.distance(p);
        if best.map_or(true, |(_, bd)| d < bd) {
            best = Some((q, d));
        }
    }
    best
}

/// The distance from a point to the nearest of a fixed set of segments,
/// through a uniform grid of the segments' boxes: cells are visited in
/// rings around the point's cell until the ring's nearest edge is farther
/// than the best distance found. The distance is the exact minimum over
/// the segments (a minimum does not depend on the order it is taken in),
/// so a grid query equals the linear scan of [`nearest_on`] to the bit.
pub(crate) struct NearestGrid<'a> {
    segments: &'a [Segment],
    lo: Point2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
}

impl<'a> NearestGrid<'a> {
    /// Below this many segments the linear scan is cheaper than the grid.
    const LINEAR_BELOW: usize = 24;

    pub(crate) fn new(segments: &'a [Segment]) -> Self {
        let (lo, hi) = super::bounds_of(segments.iter());
        let (lo, hi) = if segments.is_empty() { (Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)) } else { (lo, hi) };
        // About four segments per cell for evenly spread input.
        let n = (segments.len() as f64 / 4.0).sqrt().ceil().max(1.0);
        let cell = (((hi.x - lo.x).max(hi.y - lo.y)) / n).max(1e-9);
        let nx = (((hi.x - lo.x) / cell).floor() as usize + 1).min(2048);
        let ny = (((hi.y - lo.y) / cell).floor() as usize + 1).min(2048);
        let mut cells = vec![Vec::new(); nx * ny];
        let index = |p: Point2| -> (usize, usize) {
            let ix = (((p.x - lo.x) / cell).floor().max(0.0) as usize).min(nx - 1);
            let iy = (((p.y - lo.y) / cell).floor().max(0.0) as usize).min(ny - 1);
            (ix, iy)
        };
        for (i, s) in segments.iter().enumerate() {
            let (slo, shi) = s.bounds();
            let (x0, y0) = index(slo);
            let (x1, y1) = index(shi);
            for iy in y0..=y1 {
                for ix in x0..=x1 {
                    cells[iy * nx + ix].push(i as u32);
                }
            }
        }
        Self { segments, lo, cell, nx, ny, cells }
    }

    /// The exact nearest distance (`None` for no segments).
    pub(crate) fn distance(&self, p: Point2) -> Option<f64> {
        self.distance_excluding(p, &|_| false)
    }

    /// The exact nearest distance to the segments `skip` does not reject.
    pub(crate) fn distance_excluding(&self, p: Point2, skip: &dyn Fn(usize) -> bool) -> Option<f64> {
        if self.segments.len() < Self::LINEAR_BELOW {
            let d = self.segments.iter().enumerate().filter(|(i, _)| !skip(*i)).map(|(_, s)| s.distance(p)).fold(f64::INFINITY, f64::min);
            return d.is_finite().then_some(d);
        }
        let fx = (p.x - self.lo.x) / self.cell;
        let fy = (p.y - self.lo.y) / self.cell;
        let ix = (fx.floor().max(0.0) as usize).min(self.nx - 1) as i64;
        let iy = (fy.floor().max(0.0) as usize).min(self.ny - 1) as i64;
        let mut best = f64::INFINITY;
        let rings = (self.nx.max(self.ny)) as i64;
        for r in 0..=rings {
            // Every cell of ring r lies outside the (r-1)-square about the
            // point's cell; nothing in it is nearer than the point's
            // distance to that square's edges (zero when the point is outside).
            if r > 0 {
                let inner = (r - 1) as f64;
                let x_lo = self.lo.x + (ix as f64 - inner) * self.cell;
                let x_hi = self.lo.x + (ix as f64 + inner + 1.0) * self.cell;
                let y_lo = self.lo.y + (iy as f64 - inner) * self.cell;
                let y_hi = self.lo.y + (iy as f64 + inner + 1.0) * self.cell;
                let edge = (p.x - x_lo).min(x_hi - p.x).min(p.y - y_lo).min(y_hi - p.y).max(0.0);
                if edge >= best {
                    break;
                }
            }
            let mut any = false;
            for cy in (iy - r)..=(iy + r) {
                if cy < 0 || cy >= self.ny as i64 {
                    continue;
                }
                for cx in (ix - r)..=(ix + r) {
                    if cx < 0 || cx >= self.nx as i64 {
                        continue;
                    }
                    if r > 0 && (cx - ix).abs() < r && (cy - iy).abs() < r {
                        continue;
                    }
                    any = true;
                    for &i in &self.cells[cy as usize * self.nx + cx as usize] {
                        if skip(i as usize) {
                            continue;
                        }
                        let d = self.segments[i as usize].distance(p);
                        if d < best {
                            best = d;
                        }
                    }
                }
            }
            if !any && r > 0 && (ix - r < 0 && ix + r >= self.nx as i64 && iy - r < 0 && iy + r >= self.ny as i64) {
                break;
            }
        }
        best.is_finite().then_some(best)
    }
}

/// Stitch oriented pieces into closed loops. Vertices joined by stub pieces
/// are identified and the stubs dropped; of two pieces within the slack of
/// each other running the same way (a vertex circle's last fraction beside
/// the parallel it is tangent to, the vertex circles over a tiny input
/// segment beside its parallel) only the better one survives; dangling pieces (an endpoint with no
/// continuation) are pruned, a strict one being an arrangement failure; at
/// a vertex with several continuations the sharpest left turn is taken,
/// which keeps touching regions as separate loops. Gaps left by identified
/// vertices are bridged. Consecutive pieces on one line or circle are merged
/// back into one segment; loops thinner than the point epsilon are dropped.
fn stitch(pieces: Vec<Kept>, vertices: &[Point2], tol: &Tolerance) -> Result<Vec<Loop>, PlanarError> {
    let nv = vertices.len();
    let stub = STUB_EPS * tol.point_eps;
    // Stubs identify their ends.
    let mut dsu = Dsu::new(nv);
    for p in &pieces {
        if p.seg.length() <= stub {
            dsu.union(p.v0, p.v1);
        }
    }
    let mut pieces: Vec<Kept> = pieces
        .into_iter()
        .filter(|p| p.seg.length() > stub)
        .map(|mut p| {
            p.v0 = dsu.find(p.v0);
            p.v1 = dsu.find(p.v1);
            p
        })
        .filter(|p| p.v0 != p.v1)
        .collect();
    // Near-coincident pieces: the better (smaller miss, then longer, then
    // earlier) of two pieces within the arrangement's slack of each other
    // running the same way covers the other, which is dropped (the keep
    // rules admit pieces up to the slack off their curve, so duplicates
    // can be that far apart: a vertex circle's two halves over a 5e-3
    // chord beside the chord's parallel). Pieces are taken best first, so
    // a piece is only ever dropped for a survivor.
    let mut order: Vec<usize> = (0..pieces.len()).collect();
    order.sort_by(|&a, &b| {
        pieces[a].miss.total_cmp(&pieces[b].miss).then(pieces[b].seg.length().total_cmp(&pieces[a].seg.length())).then(a.cmp(&b))
    });
    let mut grid = PieceGrid::new(&pieces, tol);
    let mut survives = vec![false; pieces.len()];
    for &i in &order {
        let m = pieces[i].seg.midpoint();
        let t = pieces[i].seg.tangent_at(0.5);
        let covered = grid.near(m).any(|j| {
            if j == i || !survives[j] {
                return false;
            }
            let (q, u) = pieces[j].seg.nearest(m);
            q.distance(m) <= SLACK_EPS * tol.point_eps && pieces[j].seg.tangent_at(u).dot(t) > 0.0
        });
        if !covered {
            survives[i] = true;
            grid.insert(i, &pieces[i].seg, tol);
        }
    }
    let mut pieces: Vec<Kept> = pieces.into_iter().zip(survives).filter(|(_, s)| *s).map(|(p, _)| p).collect();
    // Unbalanced vertices within a stub length of each other: one lacking a
    // continuation and one with a continuation to spare are the same point
    // kept apart by rounding without a stub piece between them (two operands'
    // co-circular arcs ending 1.07e-6 apart, just past the merge epsilon:
    // lane C6's slot). Identify each such pair, nearest first; a balanced
    // arrangement has no unbalanced vertex, so a result that stitched before
    // is unchanged.
    {
        let mut net = vec![0i64; nv];
        for p in &pieces {
            net[p.v0] += 1;
            net[p.v1] -= 1;
        }
        let needs: Vec<usize> = (0..nv).filter(|&v| net[v] < 0).collect();
        let spares: Vec<usize> = (0..nv).filter(|&v| net[v] > 0).collect();
        if !needs.is_empty() && !spares.is_empty() {
            let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
            for &v in &needs {
                for &w in &spares {
                    let d = vertices[v].distance(vertices[w]);
                    if d <= stub {
                        pairs.push((d, v, w));
                    }
                }
            }
            pairs.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)).then(a.2.cmp(&b.2)));
            let mut joined = Dsu::new(nv);
            let mut any = false;
            for (_, v, w) in pairs {
                if net[v] < 0 && net[w] > 0 {
                    let k = (-net[v]).min(net[w]);
                    net[v] += k;
                    net[w] -= k;
                    joined.union(v, w);
                    any = true;
                }
            }
            if any {
                pieces = pieces
                    .into_iter()
                    .map(|mut p| {
                        p.v0 = joined.find(p.v0);
                        p.v1 = joined.find(p.v1);
                        p
                    })
                    .filter(|p| p.v0 != p.v1)
                    .collect();
            }
        }
    }
    // Prune dangling pieces until every piece has a predecessor and a
    // successor. A strict piece (on its level set, or any boolean piece)
    // longer than a stub cannot dangle in a sound arrangement.
    loop {
        let mut indeg = vec![0usize; nv];
        let mut outdeg = vec![0usize; nv];
        for p in &pieces {
            outdeg[p.v0] += 1;
            indeg[p.v1] += 1;
        }
        let before = pieces.len();
        if let Some(p) = pieces.iter().find(|p| (indeg[p.v0] == 0 || outdeg[p.v1] == 0) && p.strict && p.seg.length() > stub) {
            let (a, b) = (p.seg.start, p.seg.end);
            return Err(PlanarError::Inconsistent(format!(
                "a boundary piece of length {} from ({}, {}) to ({}, {}) has no continuation",
                p.seg.length(),
                a.x,
                a.y,
                b.x,
                b.y
            )));
        }
        pieces.retain(|p| indeg[p.v0] > 0 && outdeg[p.v1] > 0);
        if pieces.len() == before {
            break;
        }
    }
    let mut outgoing: Vec<Vec<usize>> = vec![Vec::new(); nv];
    for (i, p) in pieces.iter().enumerate() {
        outgoing[p.v0].push(i);
    }
    let mut used = vec![false; pieces.len()];
    let mut loops = Vec::new();
    for first in 0..pieces.len() {
        if used[first] {
            continue;
        }
        let start_vertex = pieces[first].v0;
        let mut segs = Vec::new();
        let mut current = first;
        loop {
            used[current] = true;
            segs.push(pieces[current].seg);
            let v = pieces[current].v1;
            if v == start_vertex {
                break;
            }
            let back = pieces[current].seg.tangent_at(1.0).scale(-1.0).angle();
            let mut best: Option<(f64, usize)> = None;
            for &k in &outgoing[v] {
                if used[k] {
                    continue;
                }
                // Counter-clockwise angle from the way back: right turns are
                // small, left turns large; going straight back is last.
                let mut ang = (pieces[k].seg.tangent_at(0.0).angle() - back).rem_euclid(TAU);
                if ang >= TAU - 1e-12 {
                    ang = 0.0;
                }
                if best.map_or(true, |(ba, bk)| ang > ba + 1e-12 || ((ang - ba).abs() <= 1e-12 && k < bk)) {
                    best = Some((ang, k));
                }
            }
            match best {
                Some((_, k)) => current = k,
                None => return Err(PlanarError::OpenBoundary { at: vertices[v] }),
            }
        }
        let l = merge_consecutive(bridged(segs, tol), tol);
        // A loop thinner than the point epsilon on average is arrangement
        // noise (two near-coincident curves between the same vertices), not
        // a region.
        if l.signed_area().abs() > tol.area_eps.max(tol.point_eps * l.length()) {
            loops.push(l);
        }
    }
    Ok(loops)
}

/// Consecutive segments whose ends were identified through stubs meet
/// within the stub length: a gap within the point epsilon is snapped, a
/// larger one bridged by a line.
fn bridged(segs: Vec<Segment>, tol: &Tolerance) -> Vec<Segment> {
    let n = segs.len();
    let mut out: Vec<Segment> = Vec::with_capacity(n + 2);
    let snap = tol.point_eps.min(tol.distance_eps);
    for (i, mut s) in segs.into_iter().enumerate() {
        if let Some(prev) = out.last() {
            let gap = prev.end.distance(s.start);
            if gap <= snap {
                s.start = prev.end;
            } else if gap > 0.0 {
                out.push(Segment::line(prev.end, s.start));
            }
        }
        out.push(s);
        if i + 1 == n {
            let (a, b) = (out[out.len() - 1].end, out[0].start);
            let gap = a.distance(b);
            if gap <= snap {
                out[0].start = a;
            } else if gap > 0.0 {
                out.push(Segment::line(a, b));
            }
        }
    }
    out
}

/// A uniform grid over the pieces' bounds for the near-coincidence query.
struct PieceGrid {
    lo: Point2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<usize>>,
}

impl PieceGrid {
    fn new(pieces: &[Kept], tol: &Tolerance) -> Self {
        let (lo, hi) = super::bounds_of(pieces.iter().map(|p| &p.seg));
        let (lo, hi) = if pieces.is_empty() { (Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)) } else { (lo, hi) };
        let diag = hi.sub(lo).length().max(tol.point_eps);
        let n = (pieces.len() as f64).sqrt().ceil().max(1.0);
        let cell = (diag / n).max(16.0 * tol.point_eps);
        let nx = (((hi.x - lo.x) / cell).floor() as usize + 1).min(4096);
        let ny = (((hi.y - lo.y) / cell).floor() as usize + 1).min(4096);
        Self { lo, cell, nx, ny, cells: vec![Vec::new(); nx * ny] }
    }

    fn index(&self, p: Point2) -> (usize, usize) {
        let ix = (((p.x - self.lo.x) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let iy = (((p.y - self.lo.y) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        (ix, iy)
    }

    fn insert(&mut self, i: usize, seg: &Segment, tol: &Tolerance) {
        let (lo, hi) = seg.bounds();
        let pad = SLACK_EPS * tol.point_eps;
        let (x0, y0) = self.index(lo.sub(Point2::new(pad, pad)));
        let (x1, y1) = self.index(hi.add(Point2::new(pad, pad)));
        for iy in y0..=y1 {
            for ix in x0..=x1 {
                self.cells[iy * self.nx + ix].push(i);
            }
        }
    }

    fn near(&self, p: Point2) -> impl Iterator<Item = usize> + '_ {
        let (ix, iy) = self.index(p);
        self.cells[iy * self.nx + ix].iter().copied()
    }
}

/// Merge consecutive segments that share a support (collinear lines, arcs
/// about the same centre with the same sense up to a half turn).
fn merge_consecutive(segs: Vec<Segment>, tol: &Tolerance) -> Loop {
    let same_support = |a: &Segment, b: &Segment| -> bool {
        match (a.arc, b.arc) {
            (None, None) => {
                let (Some(u), Some(v)) = (a.end.sub(a.start).normalized(), b.end.sub(b.start).normalized()) else { return false };
                u.cross(v).abs() <= tol.angle_eps && u.dot(v) > 0.0
            }
            (Some(x), Some(y)) => x.ccw == y.ccw && x.center.distance(y.center) <= tol.point_eps,
            _ => false,
        }
    };
    let mut out: Vec<Segment> = Vec::with_capacity(segs.len());
    for s in segs {
        if let Some(last) = out.last_mut() {
            if same_support(last, &s) && s.end.distance(last.start) > tol.point_eps {
                let merged = Segment { start: last.start, end: s.end, arc: last.arc };
                // Merged arcs stay at or below a half turn (a circle is two arcs).
                if merged.arc.is_none() || (merged.length() >= last.length() + s.length() - tol.distance_eps && merged.length() <= PI * merged.radius() + tol.distance_eps) {
                    *last = merged;
                    continue;
                }
            }
        }
        out.push(s);
    }
    // The seam between the last and the first segment.
    if out.len() > 1 && same_support(&out[out.len() - 1], &out[0]) && out[out.len() - 1].start.distance(out[0].end) > tol.point_eps {
        let last = out.pop().unwrap();
        let merged = Segment { start: last.start, end: out[0].end, arc: last.arc };
        if merged.arc.is_none() || (merged.length() >= last.length() + out[0].length() - tol.distance_eps && merged.length() <= PI * merged.radius() + tol.distance_eps) {
            out[0] = merged;
        } else {
            out.push(last);
        }
    }
    Loop::new(out)
}

// ---------------------------------------------------------------------------
// Open chains: side offsets and capsules; line clipping
// ---------------------------------------------------------------------------

/// Nearest point of a chain to `p`: the point, the chain's unit tangent
/// there, and whether `p` lies beyond an open end (its foot is an endpoint
/// and `p` projects past it).
fn nearest_on_chain(chain: &super::Chain, p: Point2) -> Option<(Point2, Point2, bool)> {
    let mut best: Option<(f64, Point2, Point2, bool)> = None;
    let n = chain.segments.len();
    for (i, s) in chain.segments.iter().enumerate() {
        let (q, t) = s.nearest(p);
        let d = q.distance(p);
        if best.as_ref().map_or(true, |b| d < b.0) {
            let tangent = s.tangent_at(t);
            let beyond = !chain.closed
                && ((i == 0 && t == 0.0 && p.sub(q).dot(tangent) < 0.0) || (i == n - 1 && t == 1.0 && p.sub(q).dot(tangent) > 0.0));
            best = Some((d, q, tangent, beyond));
        }
    }
    best.map(|(_, q, t, b)| (q, t, b))
}

/// The raw curve of a chain's capsules at radius `d`: both parallel curves
/// of every segment and a circle at every vertex, canonicalised.
fn chain_raw(chain: &super::Chain, d: f64, tol: &Tolerance) -> Vec<Segment> {
    let simplified = super::simplified(&super::Loop::new(chain.segments.clone()), tol);
    // simplified() may merge the seam; an open chain keeps its ends apart.
    let chain = if chain.closed || simplified.segments.len() == chain.segments.len() || simplified.segments.first().map(|s| s.start) == chain.segments.first().map(|s| s.start) {
        super::Chain { segments: simplified.segments, closed: chain.closed }
    } else {
        chain.clone()
    };
    let chain = &chain;
    let mut raw: Vec<Segment> = Vec::new();
    let mut circle = |c: Point2| {
        for k in 0..4 {
            let p0 = c.add(dir(k as f64 * PI / 2.0).scale(d));
            let p1 = c.add(dir((k + 1) as f64 * PI / 2.0).scale(d));
            raw.push(Segment::arc(p0, p1, c, true));
        }
    };
    for (i, s) in chain.segments.iter().enumerate() {
        circle(s.start);
        if i == chain.segments.len() - 1 && !chain.closed {
            circle(s.end);
        }
    }
    for s in &chain.segments {
        match s.arc {
            None => {
                let n = s.end.sub(s.start).normalized().expect("validated").perp().scale(d);
                raw.push(Segment::line(s.start.add(n), s.end.add(n)));
                raw.push(Segment::line(s.start.sub(n), s.end.sub(n)));
            }
            Some(a) => {
                let r = s.radius();
                let (a0, sweep) = s.angles();
                let a1 = a0 + sweep;
                let on = |rad: f64, ang: f64| a.center.add(dir(ang).scale(rad));
                raw.extend(Segment::arc(on(r + d, a0), on(r + d, a1), a.center, a.ccw).canonical(tol));
                let inner = r - d;
                if inner > tol.point_eps {
                    raw.extend(Segment::arc(on(inner, a0), on(inner, a1), a.center, a.ccw).canonical(tol));
                } else if inner < -tol.point_eps {
                    raw.extend(Segment::arc(on(-inner, a0 + PI), on(-inner, a1 + PI), a.center, a.ccw).canonical(tol));
                }
            }
        }
    }
    raw
}

/// Multiple of the tangent-intersection error the offset's joins tolerate.
const TANGENT_JOIN_FACTOR: f64 = 10.0;

/// The tolerance an offset of `chain` by `d` joins and merges its pieces
/// with. Each offset line is tangent to the round join about its vertex, and
/// intersecting a line with a circle it is tangent to is ill-conditioned: a
/// rounding error `delta` in the distance moves the intersection by about
/// `sqrt(2 d delta)`, with `delta` about `f64::EPSILON * M` for coordinates of
/// magnitude `M`. The point epsilon is floored at TANGENT_JOIN_FACTOR times
/// that bound (C19's ABC49 mill-turn envelope: d = 0.525, M about 60, a bound
/// of 1.1e-7 against the app's scaled point epsilon of 6.1e-8, and the offset
/// came back as five chains whose ends were 1e-7 apart). The floor, 1.2e-6
/// there, is four orders below the mesh chord (0.0125) and the machining
/// tolerance (0.05): a genuine break of an offset removes the stretch closer
/// than `d` to the chain, never a gap that small.
fn offset_join_tolerance(chain: &super::Chain, d: f64, tol: &Tolerance) -> Tolerance {
    let m = chain.segments.iter().flat_map(|s| [s.start, s.end]).map(|p| p.x.abs().max(p.y.abs())).fold(0.0f64, f64::max) + d;
    let floor = TANGENT_JOIN_FACTOR * (2.0 * d * f64::EPSILON * m).sqrt();
    if floor <= tol.point_eps {
        return *tol;
    }
    let s = floor / tol.point_eps;
    Tolerance { point_eps: floor, distance_eps: tol.distance_eps.max(floor), area_eps: tol.area_eps * s * s, ..*tol }
}

/// Which side of a chain an offset lies on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Side {
    Left,
    Right,
}

/// Offset an open or closed chain to one side by `distance`: the parallel
/// curve at that distance (round joins at convex corners, trimmed at
/// concave ones), without end caps. The result keeps the chain's direction.
/// Several chains come back when the offset breaks apart.
pub fn offset_chain(chain: &super::Chain, distance: f64, side: Side, tol: &Tolerance) -> Result<Vec<super::Chain>, PlanarError> {
    tol.validate().map_err(|e| PlanarError::InvalidInput(e.to_string()))?;
    let mut chain = chain.clone();
    chain.validate(tol)?;
    if !distance.is_finite() || distance <= 0.0 {
        return Err(PlanarError::InvalidInput("chain offset distance must be positive".into()));
    }
    let tol = &offset_join_tolerance(&chain, distance, tol);
    let raw = chain_raw(&chain, distance, tol);
    let inputs: Vec<Input> = raw.into_iter().enumerate().map(|(i, seg)| Input { seg, operand: 0, source: i }).collect();
    let arr = arrange(&inputs, tol);
    let keep_eps = keep_tolerance(distance, tol);
    let mut kept: Vec<Segment> = Vec::new();
    for group in &arr.groups {
        let rep = &arr.pieces[group[0]];
        let m = rep.seg.midpoint();
        let Some((q, t, beyond)) = nearest_on_chain(&chain, m) else { continue };
        if beyond || (q.distance(m) - distance).abs() > keep_eps {
            continue;
        }
        let s = t.cross(m.sub(q));
        if (side == Side::Left) != (s > 0.0) {
            continue;
        }
        // Keep the chain's direction of travel.
        kept.push(if rep.seg.tangent_at(0.5).dot(t) >= 0.0 { rep.seg } else { rep.seg.reversed() });
    }
    let mut chains = super::extract::chain_undirected(kept, tol);
    for c in &mut chains {
        // chain_undirected may have reversed a chain as a whole; realign it.
        if let Some(s) = c.segments.first() {
            if let Some((_, t, _)) = nearest_on_chain(&chain, s.midpoint()) {
                if s.tangent_at(0.5).dot(t) < 0.0 {
                    *c = c.reversed();
                }
            }
        }
        *c = super::Chain { segments: merge_consecutive(std::mem::take(&mut c.segments), tol).segments, closed: c.closed };
    }
    chains.sort_by(|a, b| b.length().total_cmp(&a.length()));
    Ok(chains)
}

/// The region within `radius` of a chain (its capsule / slot outline).
pub fn chain_capsule(chain: &super::Chain, radius: f64, tol: &Tolerance) -> Result<Region, PlanarError> {
    tol.validate().map_err(|e| PlanarError::InvalidInput(e.to_string()))?;
    let mut chain = chain.clone();
    chain.validate(tol)?;
    if !radius.is_finite() || radius <= 0.0 {
        return Err(PlanarError::InvalidInput("capsule radius must be positive".into()));
    }
    let raw = chain_raw(&chain, radius, tol);
    let inputs: Vec<Input> = raw.into_iter().enumerate().map(|(i, seg)| Input { seg, operand: 0, source: i }).collect();
    let arr = arrange(&inputs, tol);
    let keep_eps = keep_tolerance(radius, tol);
    let mut kept: Vec<Kept> = Vec::new();
    for group in &arr.groups {
        let rep = &arr.pieces[group[0]];
        let m = rep.seg.midpoint();
        let Some((q, _, _)) = nearest_on_chain(&chain, m) else { continue };
        let miss = (q.distance(m) - radius).abs();
        if miss > keep_eps {
            continue;
        }
        let strict = miss <= strict_tolerance(radius);
        // The capsule interior is towards the chain.
        let towards = q.sub(m).normalized().unwrap_or(Point2::new(1.0, 0.0));
        if rep.seg.tangent_at(0.5).perp().dot(towards) >= 0.0 {
            kept.push(Kept { seg: rep.seg, v0: rep.v0, v1: rep.v1, miss, strict });
        } else {
            kept.push(Kept { seg: rep.seg.reversed(), v0: rep.v1, v1: rep.v0, miss, strict });
        }
    }
    let loops = stitch(kept, &arr.vertices, tol)?;
    Ok(Region { loops })
}

/// The parts of the straight segment `a`-`b` inside `region`, in order from
/// `a` to `b`, as `(t0, t1)` parameter pairs.
pub fn clip_line(region: &Region, a: Point2, b: Point2, tol: &Tolerance) -> Vec<(f64, f64)> {
    let line = Segment::line(a, b);
    if a.distance(b) <= tol.point_eps {
        return Vec::new();
    }
    let canon = region.canonical_segments(tol);
    let mut ts: Vec<f64> = vec![0.0, 1.0];
    for s in &canon {
        for h in line.intersect(s, tol) {
            ts.push(h.ta);
        }
    }
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|x, y| (*x - *y).abs() <= tol.point_eps / a.distance(b));
    let mut out: Vec<(f64, f64)> = Vec::new();
    for w in ts.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        if t1 - t0 <= 0.0 {
            continue;
        }
        let m = line.point_at(0.5 * (t0 + t1));
        if crossings(canon.iter(), m, true) > 0 {
            if let Some(last) = out.last_mut() {
                if (last.1 - t0).abs() <= 1e-12 {
                    last.1 = t1;
                    continue;
                }
            }
            out.push((t0, t1));
        }
    }
    out
}
