//! The engagement model: how much of the tool circle is in material.
//!
//! The material at a Z level is a planar region with its directed boundary
//! (material on the left of every segment, the planar engine's convention).
//! The radial engagement of a tool of radius `r` at centre `c` moving in
//! direction `d` is the angular measure of the FRONT half of the circle
//! `|p - c| = r` that lies in material not yet removed. The circle is
//! intersected with every nearby boundary segment in closed form; each
//! crossing is classified as entering or leaving material from the sign of
//! the cross product of the segment's tangent with the circle's
//! counter-clockwise tangent; the arcs between crossings follow. Material
//! the current pass has already removed (the stadiums of its earlier chords)
//! is subtracted as angular intervals, so a tool looping back over its own
//! track sees air there. Everything behind the tool was removed by the
//! chord that brought it here (the trailing half of the circle lies inside
//! that chord's stadium for any step length), which is why only the front
//! half counts.
//!
//! Closed forms this reproduces (tests): a straight wall cut with radial
//! depth `a` gives `arccos(1 - a / r)`; a wall and a corner wall ahead give
//! the union of two arcs; a disc of material gives the lens angle.

use crate::model::Point2;
use crate::planar::{Loop, Segment, Vec2};
#[allow(unused_imports)]
use crate::planar::Chain;
use crate::Tolerance;

pub const TAU: f64 = std::f64::consts::TAU;
use std::f64::consts::PI;

/// Sorted, disjoint angular intervals within `[0, TAU)`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Intervals {
    pub arcs: Vec<(f64, f64)>,
}

fn wrap(a: f64) -> f64 {
    a.rem_euclid(TAU)
}

impl Intervals {
    pub fn empty() -> Self {
        Self { arcs: Vec::new() }
    }

    pub fn full() -> Self {
        Self { arcs: vec![(0.0, TAU)] }
    }

    /// From raw `(start, end)` pairs measured counter-clockwise; `end` may be
    /// below `start` or exceed `TAU` (wrapping arcs are split).
    pub fn from_arcs(raw: impl IntoIterator<Item = (f64, f64)>) -> Self {
        let mut arcs: Vec<(f64, f64)> = Vec::new();
        for (a, b) in raw {
            let a0 = wrap(a);
            let mut len = (b - a).rem_euclid(TAU);
            if len == 0.0 && b != a {
                len = TAU;
            }
            if len <= 0.0 {
                continue;
            }
            if a0 + len <= TAU {
                arcs.push((a0, a0 + len));
            } else {
                arcs.push((a0, TAU));
                arcs.push((0.0, a0 + len - TAU));
            }
        }
        let mut out = Self { arcs };
        out.normalize();
        out
    }

    fn normalize(&mut self) {
        self.arcs.retain(|(a, b)| b > a);
        self.arcs.sort_by(|x, y| x.0.total_cmp(&y.0).then(x.1.total_cmp(&y.1)));
        let mut merged: Vec<(f64, f64)> = Vec::with_capacity(self.arcs.len());
        for &(a, b) in &self.arcs {
            if let Some(last) = merged.last_mut() {
                if a <= last.1 {
                    last.1 = last.1.max(b);
                    continue;
                }
            }
            merged.push((a, b));
        }
        self.arcs = merged;
    }

    /// Total angle covered.
    pub fn measure(&self) -> f64 {
        self.arcs.iter().map(|(a, b)| b - a).sum()
    }

    pub fn is_empty(&self) -> bool {
        self.arcs.is_empty()
    }

    /// `self` minus `other`.
    pub fn subtract(&self, other: &Intervals) -> Intervals {
        let mut out: Vec<(f64, f64)> = Vec::with_capacity(self.arcs.len() + other.arcs.len());
        for &(a, b) in &self.arcs {
            let mut cur = a;
            for &(c, d) in &other.arcs {
                if d <= cur {
                    continue;
                }
                if c >= b {
                    break;
                }
                if c > cur {
                    out.push((cur, c));
                }
                cur = cur.max(d);
                if cur >= b {
                    break;
                }
            }
            if cur < b {
                out.push((cur, b));
            }
        }
        let mut r = Intervals { arcs: out };
        r.normalize();
        r
    }

    /// `self` intersected with `other`.
    pub fn intersect(&self, other: &Intervals) -> Intervals {
        let mut out = Vec::new();
        for &(a, b) in &self.arcs {
            for &(c, d) in &other.arcs {
                let lo = a.max(c);
                let hi = b.min(d);
                if hi > lo {
                    out.push((lo, hi));
                }
            }
        }
        let mut r = Intervals { arcs: out };
        r.normalize();
        r
    }

    /// The open half circle of directions within 90 degrees of `direction`.
    pub fn front_half(direction: Point2) -> Intervals {
        let phi = direction.angle();
        Intervals::from_arcs([(phi - 0.5 * PI, phi + 0.5 * PI)])
    }

    /// Largest connected angle (wrapping arcs joined across `TAU`).
    pub fn largest_arc(&self) -> f64 {
        if self.arcs.is_empty() {
            return 0.0;
        }
        let mut best = self.arcs.iter().map(|(a, b)| b - a).fold(0.0, f64::max);
        let n = self.arcs.len();
        if n >= 2 && self.arcs[0].0 <= 0.0 && self.arcs[n - 1].1 >= TAU {
            best = best.max(self.arcs[0].1 + TAU - self.arcs[n - 1].0);
        }
        best.min(TAU)
    }
}

/// A uniform grid over directed boundary segments (material on their left),
/// for the circle queries of the engagement model and the clearance checks
/// of repositioning chords. Segments are stored in canonical form (arcs split
/// per quadrant), so every segment is monotone in both axes.
#[derive(Clone, Debug)]
pub struct SegmentGrid {
    pub segments: Vec<Segment>,
    origin: Point2,
    cell: f64,
    nx: usize,
    ny: usize,
    cells: Vec<Vec<u32>>,
    // Which (lo, hi) cell box each segment occupies, for de-duplication.
    seg_cells: Vec<[usize; 4]>,
}

impl SegmentGrid {
    /// Index `segments` (already canonical or not; they are canonicalised)
    /// with cells of about `cell` on a side.
    pub fn new(segments: &[Segment], cell: f64, tol: &Tolerance) -> Self {
        let canon: Vec<Segment> = segments.iter().flat_map(|s| s.canonical(tol)).collect();
        let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
        let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
        for s in &canon {
            let (a, b) = s.bounds();
            lo = Point2::new(lo.x.min(a.x), lo.y.min(a.y));
            hi = Point2::new(hi.x.max(b.x), hi.y.max(b.y));
        }
        if canon.is_empty() {
            lo = Point2::new(0.0, 0.0);
            hi = Point2::new(1.0, 1.0);
        }
        let cell = if cell.is_finite() && cell > 0.0 { cell } else { 1.0 };
        let nx = (((hi.x - lo.x) / cell).ceil() as usize).clamp(1, 4096);
        let ny = (((hi.y - lo.y) / cell).ceil() as usize).clamp(1, 4096);
        // Stretch the cell so the clamp never leaves segments outside the grid.
        let cell_x = ((hi.x - lo.x) / nx as f64).max(cell);
        let cell_y = ((hi.y - lo.y) / ny as f64).max(cell);
        let cell = cell_x.max(cell_y);
        let mut cells: Vec<Vec<u32>> = vec![Vec::new(); nx * ny];
        let mut seg_cells = Vec::with_capacity(canon.len());
        let mut g = Self { segments: Vec::new(), origin: lo, cell, nx, ny, cells: Vec::new(), seg_cells: Vec::new() };
        for (i, s) in canon.iter().enumerate() {
            let (a, b) = s.bounds();
            let box_ = g.cell_box(a, b);
            for cy in box_[1]..=box_[3] {
                for cx in box_[0]..=box_[2] {
                    cells[cy * nx + cx].push(i as u32);
                }
            }
            seg_cells.push(box_);
        }
        g.segments = canon;
        g.cells = cells;
        g.seg_cells = seg_cells;
        g
    }

    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    fn cell_box(&self, lo: Point2, hi: Point2) -> [usize; 4] {
        let fx = |x: f64| (((x - self.origin.x) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let fy = |y: f64| (((y - self.origin.y) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        [fx(lo.x), fy(lo.y), fx(hi.x), fy(hi.y)]
    }

    /// Indices of the segments whose cells overlap the box `lo..hi`, each
    /// once, in increasing order (deterministic).
    pub fn query(&self, lo: Point2, hi: Point2, out: &mut Vec<u32>) {
        out.clear();
        if self.segments.is_empty() {
            return;
        }
        if hi.x < self.origin.x || hi.y < self.origin.y || lo.x > self.origin.x + self.nx as f64 * self.cell || lo.y > self.origin.y + self.ny as f64 * self.cell {
            return;
        }
        let b = self.cell_box(lo, hi);
        for cy in b[1]..=b[3] {
            for cx in b[0]..=b[2] {
                out.extend_from_slice(&self.cells[cy * self.nx + cx]);
            }
        }
        out.sort_unstable();
        out.dedup();
    }

    /// Nearest segment to `p` within `max_distance`: `(index, point, distance)`.
    pub fn nearest(&self, p: Point2, max_distance: f64) -> Option<(usize, Point2, f64)> {
        let mut best: Option<(usize, Point2, f64)> = None;
        let mut ids = Vec::new();
        // Grow the search box in rings of cells until a hit is closer than
        // the ring's inner radius.
        let mut radius = self.cell;
        loop {
            let lo = Point2::new(p.x - radius, p.y - radius);
            let hi = Point2::new(p.x + radius, p.y + radius);
            self.query(lo, hi, &mut ids);
            for &i in &ids {
                let s = &self.segments[i as usize];
                let (q, _) = s.nearest(p);
                let d = q.distance(p);
                if best.map_or(true, |b| d < b.2 || (d == b.2 && (i as usize) < b.0)) {
                    best = Some((i as usize, q, d));
                }
            }
            if let Some(b) = best {
                if b.2 <= radius {
                    return if b.2 <= max_distance { Some(b) } else { None };
                }
            }
            if radius >= max_distance {
                return best.filter(|b| b.2 <= max_distance);
            }
            radius = (radius * 2.0).min(max_distance.max(self.cell));
            if radius >= max_distance && best.is_none() {
                // One final full query at the maximum.
                let lo = Point2::new(p.x - max_distance, p.y - max_distance);
                let hi = Point2::new(p.x + max_distance, p.y + max_distance);
                self.query(lo, hi, &mut ids);
                for &i in &ids {
                    let s = &self.segments[i as usize];
                    let (q, _) = s.nearest(p);
                    let d = q.distance(p);
                    if best.map_or(true, |b| d < b.2) {
                        best = Some((i as usize, q, d));
                    }
                }
                return best.filter(|b| b.2 <= max_distance);
            }
        }
    }

    /// Is `p` inside the region (material on the left of the boundary)? Exact
    /// by the side of the nearest boundary point: on a segment's interior the
    /// side of its tangent; at a vertex the wedge rule (a convex corner holds
    /// material to the left of both edges, a reflex corner to the left of
    /// either). `None` when no segment lies within `max_distance` (the caller
    /// knows the ambient state) or the vertex is shared by more than two
    /// segments.
    pub fn inside(&self, p: Point2, max_distance: f64) -> Option<bool> {
        let (i, q, d) = self.nearest(p, max_distance)?;
        if d <= 0.0 {
            return Some(true);
        }
        let v = p.sub(q);
        // Every segment whose nearest point to p is (within rounding) q.
        let mut ids = Vec::new();
        self.query(Point2::new(q.x - 1e-7, q.y - 1e-7), Point2::new(q.x + 1e-7, q.y + 1e-7), &mut ids);
        let mut at_vertex: Vec<(usize, bool)> = Vec::new(); // (segment, q is its end)
        let mut interior: Option<usize> = None;
        for &j in &ids {
            let o = &self.segments[j as usize];
            let (oq, ot) = o.nearest(p);
            if oq.distance(q) > 1e-9 {
                continue;
            }
            let eps_t = 1e-9;
            if ot <= eps_t {
                at_vertex.push((j as usize, false));
            } else if ot >= 1.0 - eps_t {
                at_vertex.push((j as usize, true));
            } else {
                interior = Some(j as usize);
            }
        }
        if let Some(j) = interior {
            let o = &self.segments[j];
            let (_, t) = o.nearest(p);
            return Some(o.tangent_at(t).cross(v) > 0.0);
        }
        if at_vertex.is_empty() {
            let s = &self.segments[i];
            let (_, t) = s.nearest(p);
            return Some(s.tangent_at(t).cross(v) > 0.0);
        }
        let incoming: Vec<usize> = at_vertex.iter().filter(|(_, end)| *end).map(|(j, _)| *j).collect();
        let outgoing: Vec<usize> = at_vertex.iter().filter(|(_, end)| !*end).map(|(j, _)| *j).collect();
        if incoming.len() != 1 || outgoing.len() != 1 {
            return None;
        }
        let s_in = &self.segments[incoming[0]];
        let s_out = &self.segments[outgoing[0]];
        let t_in = s_in.tangent_at(1.0);
        let t_out = s_out.tangent_at(0.0);
        let left_in = t_in.cross(v) > 0.0;
        let left_out = t_out.cross(v) > 0.0;
        let convex = t_in.cross(t_out) >= 0.0;
        Some(if convex { left_in && left_out } else { left_in || left_out })
    }

    /// Smallest distance from any segment to the chord `a`-`b` (XY), or
    /// `None` when no segment comes within `reach` of the chord's box.
    pub fn chord_clearance(&self, a: Point2, b: Point2, reach: f64) -> Option<f64> {
        let lo = Point2::new(a.x.min(b.x) - reach, a.y.min(b.y) - reach);
        let hi = Point2::new(a.x.max(b.x) + reach, a.y.max(b.y) + reach);
        let mut ids = Vec::new();
        self.query(lo, hi, &mut ids);
        let chord = Segment::line(a, b);
        let mut best: Option<f64> = None;
        for &i in &ids {
            let d = segment_distance(&self.segments[i as usize], &chord);
            if best.map_or(true, |b| d < b) {
                best = Some(d);
            }
        }
        best.filter(|d| *d <= reach)
    }
}

/// Distance between two segments (lines or arcs), zero when they cross.
pub fn segment_distance(s: &Segment, t: &Segment) -> f64 {
    let tol = Tolerance::default();
    if !s.intersect(t, &tol).is_empty() {
        return 0.0;
    }
    // Endpoint-to-segment distances plus, for arcs, the radial extremes.
    let mut best = f64::INFINITY;
    for (p, q) in [(s, t), (t, s)] {
        best = best.min(q.distance(p.start)).min(q.distance(p.end));
        if let Some(a) = p.arc {
            // Nearest point of q to the arc's centre, projected onto the arc.
            let (n, _) = q.nearest(a.center);
            let (on_arc, _) = p.nearest(n);
            best = best.min(on_arc.distance(n));
            // And the other way: q's nearest point to that arc point.
            let (back, _) = q.nearest(on_arc);
            best = best.min(back.distance(on_arc));
        }
    }
    best
}

/// A chord's swept stadium of radius `radius` (the material one chord of a
/// pass removes); with `a == b` it is a disc (a trochoid loop's envelope).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stadium {
    pub a: Point2,
    pub b: Point2,
    pub radius: f64,
}

/// Angles `psi` with `lo <= cos(psi - phi) <= hi`.
fn cos_range(phi: f64, lo: f64, hi: f64) -> Vec<(f64, f64)> {
    if hi < -1.0 || lo > 1.0 || hi < lo {
        return Vec::new();
    }
    let lo = lo.max(-1.0);
    let hi = hi.min(1.0);
    let a_hi = hi.acos(); // smallest |delta|
    let a_lo = lo.acos(); // largest |delta|
    if a_hi <= 0.0 {
        if a_lo >= PI {
            return vec![(0.0, TAU)];
        }
        return vec![(phi - a_lo, phi + a_lo)];
    }
    if a_lo >= PI {
        // |delta| in [a_hi, pi]: one arc through the back.
        return vec![(phi + a_hi, phi + TAU - a_hi)];
    }
    vec![(phi + a_hi, phi + a_lo), (phi - a_lo, phi - a_hi)]
}

/// Angles of the circle `|p - c| = r` inside the disc of radius `big` about `e`.
fn lens(c: Point2, r: f64, e: Point2, big: f64) -> Vec<(f64, f64)> {
    let v = e.sub(c);
    let d = v.length();
    if d + r <= big {
        return vec![(0.0, TAU)];
    }
    if d >= big + r || d + big <= r {
        return Vec::new();
    }
    let cos_half = ((d * d + r * r - big * big) / (2.0 * d * r)).clamp(-1.0, 1.0);
    let half = cos_half.acos();
    let mid = v.angle();
    vec![(mid - half, mid + half)]
}

impl Stadium {
    pub fn new(a: Point2, b: Point2, radius: f64) -> Self {
        Self { a, b, radius }
    }

    pub fn bounds(&self) -> (Point2, Point2) {
        let r = self.radius;
        (
            Point2::new(self.a.x.min(self.b.x) - r, self.a.y.min(self.b.y) - r),
            Point2::new(self.a.x.max(self.b.x) + r, self.a.y.max(self.b.y) + r),
        )
    }

    pub fn contains(&self, p: Point2) -> bool {
        Segment::line(self.a, self.b).distance(p) <= self.radius
    }

    /// Angular intervals of the circle `|p - c| = r` inside this stadium, in
    /// closed form: the lens of each end disc plus the slab between them.
    pub fn circle_inside(&self, c: Point2, r: f64) -> Intervals {
        let big = self.radius;
        let mut arcs: Vec<(f64, f64)> = lens(c, r, self.a, big);
        let ab = self.b.sub(self.a);
        let len = ab.length();
        if len > 0.0 {
            arcs.extend(lens(c, r, self.b, big));
            let u = ab.scale(1.0 / len);
            let n = u.perp();
            let w = c.sub(self.a);
            let a_n = w.dot(n);
            let b_u = w.dot(u);
            // |a_n + r cos(psi - phi_n)| <= big  and  0 <= b_u + r cos(psi - phi_u) <= len
            let across = Intervals::from_arcs(cos_range(n.angle(), (-big - a_n) / r, (big - a_n) / r));
            let along = Intervals::from_arcs(cos_range(u.angle(), -b_u / r, (len - b_u) / r));
            arcs.extend(across.intersect(&along).arcs);
        }
        Intervals::from_arcs(arcs)
    }
}

/// One crossing of the circle with a boundary.
#[derive(Clone, Copy, Debug)]
struct Crossing {
    angle: f64,
    /// `true`: moving counter-clockwise along the circle enters material here.
    entering: bool,
}

/// The circle's four counter-clockwise quadrant arcs.
fn circle_arcs(center: Point2, radius: f64) -> Vec<Segment> {
    Loop::circle(center, radius, true).segments
}

/// Crossings of the circle with `segments` (directed, material on the left).
fn crossings<'a>(center: Point2, radius: f64, segments: impl Iterator<Item = &'a Segment>, tol: &Tolerance) -> Vec<Crossing> {
    let circle = circle_arcs(center, radius);
    let mut out: Vec<Crossing> = Vec::new();
    for s in segments {
        let (lo, hi) = s.bounds();
        if lo.x > center.x + radius || hi.x < center.x - radius || lo.y > center.y + radius || hi.y < center.y - radius {
            continue;
        }
        for c in &circle {
            for h in c.intersect(s, tol) {
                let radial = h.point.sub(center);
                let angle = wrap(radial.angle());
                let circle_tangent = radial.perp();
                let seg_tangent = s.tangent_at(h.tb);
                let cross = seg_tangent.cross(circle_tangent);
                // Tangential contact (within the angle tolerance): not a crossing.
                if cross.abs() <= 1e-7 * circle_tangent.length() {
                    continue;
                }
                out.push(Crossing { angle, entering: cross > 0.0 });
            }
        }
    }
    out.sort_by(|a, b| a.angle.total_cmp(&b.angle).then(a.entering.cmp(&b.entering)));
    // Coincident crossings: a vertex the circle passes through yields one
    // hit per adjacent segment (same sense: keep one; opposite senses: the
    // boundary touches the circle there and turns back: drop both).
    let eps = (tol.point_eps / radius.max(tol.point_eps)).max(1e-9) * 4.0;
    let mut dedup: Vec<Crossing> = Vec::with_capacity(out.len());
    let mut i = 0;
    while i < out.len() {
        let mut j = i + 1;
        while j < out.len() && out[j].angle - out[i].angle <= eps {
            j += 1;
        }
        let group = &out[i..j];
        let enters = group.iter().filter(|c| c.entering).count();
        let leaves = group.len() - enters;
        if enters > 0 && leaves == 0 {
            dedup.push(group[0]);
        } else if leaves > 0 && enters == 0 {
            dedup.push(group[0]);
        }
        // Mixed: touching, dropped.
        i = j;
    }
    // Wrap-around coincidence (angles near 0 and near TAU).
    if dedup.len() >= 2 {
        let first = dedup[0];
        let last = dedup[dedup.len() - 1];
        if first.angle + TAU - last.angle <= eps {
            if first.entering == last.entering {
                dedup.pop();
            } else {
                dedup.pop();
                dedup.remove(0);
            }
        }
    }
    dedup
}

/// Angular intervals of the circle `|p - center| = radius` inside the region
/// bounded by `segments` (directed, material on the left). `ambient` decides
/// the state when the circle meets no boundary at all (or the crossings are
/// inconsistent and a midpoint has no boundary within reach): it is given a
/// point of the circle and must say whether it lies in material.
pub fn circle_inside<'a>(
    center: Point2,
    radius: f64,
    segments: impl Iterator<Item = &'a Segment>,
    tol: &Tolerance,
    ambient: &dyn Fn(Point2) -> bool,
) -> Intervals {
    let xs = crossings(center, radius, segments, tol);
    if xs.is_empty() {
        // The circle is wholly in or out. A single probe can sit exactly on a
        // boundary the circle only touches (a previous pass's edge at the tool
        // radius); a majority of seven probes at irregular angles cannot all.
        let votes = (0..7)
            .filter(|k| {
                let a = 0.37 + 0.9 * *k as f64;
                ambient(center.add(Point2::new(radius * a.cos(), radius * a.sin())))
            })
            .count();
        return if votes >= 4 { Intervals::full() } else { Intervals::empty() };
    }
    // Each arc between consecutive crossings is classified by the material
    // test at its midpoint, not by the crossings' orientation: a material
    // boundary tangent to the tool circle (the edge a previous pass left at
    // exactly the tool radius, which every later position on that pass sees)
    // yields near-tangent crossing pairs whose orientation is rounding noise.
    let n = xs.len();
    let mut arcs: Vec<(f64, f64)> = Vec::new();
    for i in 0..n {
        let a = xs[i].angle;
        let b = if i + 1 < n { xs[i + 1].angle } else { xs[0].angle + TAU };
        let mid = 0.5 * (a + b);
        let inside = ambient(center.add(Point2::new(radius * mid.cos(), radius * mid.sin())));
        if inside && b > a {
            arcs.push((a, b));
        }
    }
    Intervals::from_arcs(arcs)
}

/// The engagement of a tool of radius `radius` at `center` moving in
/// `direction`: the angular measure (radians) of the front half of the tool
/// circle that lies in material. `material` is the grid over the remaining
/// material's boundary, `inside_material` the ambient classifier for points
/// the grid cannot decide locally, `cleared` the stadiums this pass has
/// already swept (chords before the one that ends at `center`).
/// Returns the angle and the intervals it was measured over.
pub fn engagement(
    material: &SegmentGrid,
    inside_material: &dyn Fn(Point2) -> bool,
    cleared: &[&Stadium],
    center: Point2,
    direction: Point2,
    radius: f64,
    tol: &Tolerance,
) -> (f64, Intervals) {
    let lo = Point2::new(center.x - radius, center.y - radius);
    let hi = Point2::new(center.x + radius, center.y + radius);
    let mut ids = Vec::new();
    material.query(lo, hi, &mut ids);
    let ambient = |p: Point2| -> bool {
        match material.inside(p, 4.0 * radius) {
            Some(b) => b,
            None => inside_material(p),
        }
    };
    let mut inside = circle_inside(center, radius, ids.iter().map(|&i| &material.segments[i as usize]), tol, &ambient);
    if inside.is_empty() {
        return (0.0, inside);
    }
    for &s in cleared {
        let (slo, shi) = s.bounds();
        if slo.x > hi.x || shi.x < lo.x || slo.y > hi.y || shi.y < lo.y {
            continue;
        }
        let swept = s.circle_inside(center, radius);
        if !swept.is_empty() {
            inside = inside.subtract(&swept);
            if inside.is_empty() {
                return (0.0, inside);
            }
        }
    }
    let front = inside.intersect(&Intervals::front_half(direction));
    (front.measure(), front)
}

/// Radial depth of cut `a` for an engagement angle on a tool of radius `r`
/// (straight cut): `a = r (1 - cos theta)`.
pub fn depth_for_engagement(theta: f64, r: f64) -> f64 {
    r * (1.0 - theta.cos())
}

/// Engagement angle of a straight cut at radial depth `a` on a tool of
/// radius `r`: `arccos(1 - a / r)`, clamped to `[0, pi]`.
pub fn engagement_for_depth(a: f64, r: f64) -> f64 {
    (1.0 - a / r).clamp(-1.0, 1.0).acos()
}
