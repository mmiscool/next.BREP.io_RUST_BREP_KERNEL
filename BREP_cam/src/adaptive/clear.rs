//! The level clearing solver: constant-engagement roughing of one Z level.
//!
//! Sets, all in the plane of the level:
//! - `T`: the tool-centre feasible set (positions where the tool touches
//!   neither the part plus its allowance nor a fixture); the caller builds
//!   it from the stock slice and the part shadow.
//! - `M`: the material still to remove (exact region); `M ∩ (T ⊕ r)` is the
//!   part of it the tool can reach.
//! - `Q = T \ (M ⊕ r)`: the cleared tool-centre set, positions where the tool
//!   cuts nothing (air outside the stock, the entry's helix envelope, what
//!   earlier passes removed).
//!
//! A round grows the cleared set by the optimal load `s`: the front
//! `∂((Q ⊕ s) ∩ T)` is where a tool cutting a strip of radial depth `s`
//! runs; its straight stretches have exactly the limit engagement
//! `arccos(1 - s / r)`. Where the front bends around a convex corner of `Q`
//! the engagement rises above the limit (for a sharp corner it exceeds 90
//! degrees for any offset), so the marcher evaluates the exact engagement
//! ([`super::engagement`](mod@super::engagement)) at every step and, where the limit would be
//! exceeded, cuts trochoidal loops instead: circles on the cleared side,
//! tangent to the front, advanced by a step found by a bounded search so
//! that the exact engagement over the whole loop stays within the limit.
//! After each pass the material is updated exactly (`M := M \ capsule`), the
//! front is recomputed, and the next front piece is reached through cleared
//! material when a straight chord is clear of material and inside `T`, else
//! a new pass starts (the linker retracts). When no front piece touches
//! material and reachable material remains, the solver seeds a new entry at
//! the deepest point of the uncleared feasible set. Everything is counted
//! as work, nothing depends on time or iteration order of a hash map.

use super::dilate::{check_difference, checked, disc, grow, swept_region};
use super::engagement::{engagement, segment_distance, SegmentGrid, Stadium, TAU};
use crate::model::{Diagnostic, Point2};
use crate::planar::{Chain, Loop, Region, Segment, Vec2};
use crate::{CamError, Runner, Tolerance};
use std::collections::BTreeMap;
use std::f64::consts::PI;

fn invalid(message: impl std::fmt::Display) -> CamError {
    CamError::InvalidInput(format!("adaptive: {message}"))
}

/// Solver parameters (tool-centre units).
#[derive(Clone, Debug, PartialEq)]
pub struct ClearParams {
    /// Tool radius `r`.
    pub radius: f64,
    /// Optimal load `s` (radial stepover of a straight cut), `0 < s <= r`.
    pub load: f64,
    /// Smallest trochoid loop radius the machine should run.
    pub min_loop_radius: f64,
    /// Largest trochoid loop radius tried (`0`: the tool radius).
    pub max_loop_radius: f64,
    /// Chord tolerance of the emitted polylines.
    pub chord_tolerance: f64,
    /// Climb milling: the uncut material on the right of travel.
    pub climb: bool,
    /// Helix diameter of a fresh entry (`0`: plunge). Its envelope
    /// (`diameter / 2 + r`) is the material the entry removes.
    pub helix_diameter: f64,
    /// Longest straight repositioning chord kept at the cut height (`0`: unlimited).
    pub max_connector: f64,
    /// Budgets: rounds per level, engagement samples per level.
    pub max_rounds: usize,
    pub max_samples: u64,
}

impl Default for ClearParams {
    fn default() -> Self {
        Self {
            radius: 3.0,
            load: 1.0,
            min_loop_radius: 0.25,
            max_loop_radius: 0.0,
            chord_tolerance: 0.01,
            climb: true,
            helix_diameter: 0.0,
            max_connector: 0.0,
            max_rounds: 100_000,
            max_samples: 50_000_000,
        }
    }
}

impl ClearParams {
    pub fn validate(&self) -> Result<(), CamError> {
        let fin = |v: f64| v.is_finite();
        if !fin(self.radius) || self.radius <= 0.0 {
            return Err(invalid("tool radius must be positive"));
        }
        if !fin(self.load) || self.load <= 0.0 || self.load > self.radius + 1e-12 {
            return Err(invalid("optimal load must be positive and at most the tool radius"));
        }
        if !fin(self.min_loop_radius) || self.min_loop_radius < 0.0 || !fin(self.max_loop_radius) || self.max_loop_radius < 0.0 {
            return Err(invalid("loop radii must be finite and non-negative"));
        }
        if !fin(self.chord_tolerance) || self.chord_tolerance <= 0.0 {
            return Err(invalid("chord tolerance must be positive"));
        }
        if !fin(self.helix_diameter) || self.helix_diameter < 0.0 || !fin(self.max_connector) || self.max_connector < 0.0 {
            return Err(invalid("helix diameter and connector length must be finite and non-negative"));
        }
        if self.max_rounds == 0 || self.max_samples == 0 {
            return Err(invalid("budgets must be positive"));
        }
        Ok(())
    }

    /// The engagement limit: a straight cut at the optimal load.
    pub fn limit(&self) -> f64 {
        super::engagement::engagement_for_depth(self.load, self.radius)
    }

    /// Marching step along fronts and loops.
    pub fn step(&self) -> f64 {
        (self.load / 5.0).max(self.chord_tolerance).min(self.radius / 4.0)
    }
}

/// Material already removed at an entry point before the passes start.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Entry {
    /// Centre of the removed disc (the helix centre or the predrilled hole).
    pub center: Point2,
    /// Radius of the removed disc (`0`: a plunge removed only the tool's own
    /// disc).
    pub cleared_radius: f64,
    /// `true` when this solver chose the entry (the pass starting here needs
    /// the linker's helix of `helix_diameter`), `false` for a predrilled hole.
    pub helix: bool,
}

/// What one level needs.
pub struct ClearInput<'a> {
    pub feasible: &'a Region,
    pub material: &'a Region,
    /// Predrilled holes or user entry points.
    pub entries: Vec<Entry>,
    /// Tool position before the level (orders the first front piece).
    pub start: Option<Point2>,
}

/// One tool-down sequence at the level.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelPass {
    /// Exact geometry: front pieces, trochoid arcs, repositioning chords.
    pub chain: Chain,
    /// The polyline the pass is cut along (chords within the chord tolerance
    /// of `chain`, never on the material side of it).
    pub points: Vec<Point2>,
    /// The entry whose descent this pass starts with, when it is fresh.
    pub entry: Option<Entry>,
    pub max_engagement: f64,
    pub loops: usize,
    /// Engagement samples above the limit (reported, never silent).
    pub exceeded: u64,
    /// Area of the material left after this pass (the solver's bookkeeping).
    pub material_after: f64,
}

/// The level's result.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelResult {
    pub passes: Vec<LevelPass>,
    pub limit: f64,
    pub max_engagement: f64,
    pub exceeded: u64,
    pub loops: usize,
    pub rounds: usize,
    /// Reachable material left (exact), and its area.
    pub remaining: Region,
    pub remaining_area: f64,
    pub entries: Vec<Entry>,
    pub samples: u64,
    pub warnings: Vec<Diagnostic>,
    /// Work of the feasibility checks of pulled stretches: the stretches
    /// checked, and the containment tests (a winding count over the whole
    /// feasible boundary each) they took. One per stretch at most; a test per
    /// chord point made a level eight times slower and two sweep runs never
    /// finish (h35).
    pub stretches_checked: u64,
    pub stretch_containment_tests: u64,
}

thread_local! {
    /// `LevelResult`'s stretch-check counters for the level on this thread
    /// (the marcher runs on the calling thread).
    static STRETCH_WORK: std::cell::Cell<(u64, u64)> = const { std::cell::Cell::new((0, 0)) };
}

/// The deepest point of `region`: a point whose distance to the boundary is
/// within `resolution` of the largest inscribed radius (bisection on inward
/// offsets). Returns the point and a lower bound of its clearance.
pub fn deepest_point(region: &Region, resolution: f64, tol: &Tolerance) -> Result<Option<(Point2, f64)>, CamError> {
    if region.is_empty() {
        return Ok(None);
    }
    let (lo_b, hi_b) = region.bounds();
    let mut lo = 0.0;
    let mut hi = 0.5 * lo_b.distance(hi_b);
    let mut best = region.clone();
    let mut guard = 0;
    while hi - lo > resolution.max(tol.distance_eps) && guard < 64 {
        guard += 1;
        let mid = 0.5 * (lo + hi);
        let r = super::dilate::shrink(region, mid, resolution.max(tol.distance_eps), tol)?;
        if r.is_empty() {
            hi = mid;
        } else {
            lo = mid;
            best = r;
        }
    }
    // The largest loop of the deepest offset; its first vertex.
    let l = best
        .loops
        .iter()
        .filter(|l| l.is_ccw())
        .max_by(|a, b| a.signed_area().total_cmp(&b.signed_area()))
        .or(best.loops.first());
    Ok(l.map(|l| (l.segments[0].start, lo)))
}

/// Positions along a chain of segments by arc length.
struct Cursor {
    segments: Vec<Segment>,
    cumulative: Vec<f64>,
}

impl Cursor {
    fn new(segments: Vec<Segment>) -> Self {
        let mut cumulative = Vec::with_capacity(segments.len() + 1);
        let mut acc = 0.0;
        cumulative.push(0.0);
        for s in &segments {
            acc += s.length();
            cumulative.push(acc);
        }
        Self { segments, cumulative }
    }

    fn length(&self) -> f64 {
        *self.cumulative.last().unwrap_or(&0.0)
    }

    fn locate(&self, s: f64) -> (usize, f64) {
        let n = self.segments.len();
        if n == 0 {
            return (0, 0.0);
        }
        let s = s.clamp(0.0, self.length());
        // Binary search for the segment containing s.
        let mut i = match self.cumulative.binary_search_by(|c| c.total_cmp(&s)) {
            Ok(i) => i,
            Err(i) => i.saturating_sub(1),
        };
        if i >= n {
            i = n - 1;
        }
        let len = self.cumulative[i + 1] - self.cumulative[i];
        let t = if len > 0.0 { ((s - self.cumulative[i]) / len).clamp(0.0, 1.0) } else { 0.0 };
        (i, t)
    }

    fn point(&self, s: f64) -> Point2 {
        let (i, t) = self.locate(s);
        if s >= self.length() {
            return self.segments[i].end;
        }
        self.segments[i].point_at(t)
    }

    fn tangent(&self, s: f64) -> Point2 {
        let (i, t) = self.locate(s);
        self.segments[i].tangent_at(t)
    }

    /// Exact pieces between arc lengths `s0 < s1` with the given end points.
    fn pieces(&self, s0: f64, s1: f64, p0: Point2, p1: Point2) -> Vec<Segment> {
        let (i0, _) = self.locate(s0);
        let (mut i1, t1) = self.locate(s1);
        if t1 <= 0.0 && i1 > i0 {
            i1 -= 1;
        }
        let mut out = Vec::new();
        let mut from = p0;
        for i in i0..=i1 {
            let seg = &self.segments[i];
            let to = if i == i1 { p1 } else { seg.end };
            if from.distance(to) > 1e-6 {
                out.push(seg.sub_segment(from, to));
                from = to;
            }
        }
        // The last piece must end exactly at p1.
        if let Some(last) = out.last_mut() {
            if last.end != p1 {
                if last.start.distance(p1) > 1e-6 {
                    last.end = p1;
                } else {
                    out.pop();
                }
            }
        }
        out
    }
}

/// Spatial grid over the stadiums of the current pass.
struct StadiumGrid {
    cell: f64,
    cells: BTreeMap<(i64, i64), Vec<u32>>,
    stadiums: Vec<Stadium>,
}

impl StadiumGrid {
    fn new(cell: f64) -> Self {
        Self { cell, cells: BTreeMap::new(), stadiums: Vec::new() }
    }

    fn key(&self, x: f64, y: f64) -> (i64, i64) {
        ((x / self.cell).floor() as i64, (y / self.cell).floor() as i64)
    }

    fn push(&mut self, s: Stadium) {
        let (lo, hi) = s.bounds();
        let (x0, y0) = self.key(lo.x, lo.y);
        let (x1, y1) = self.key(hi.x, hi.y);
        let id = self.stadiums.len() as u32;
        for y in y0..=y1 {
            for x in x0..=x1 {
                self.cells.entry((x, y)).or_default().push(id);
            }
        }
        self.stadiums.push(s);
    }

    /// Remove the last stadium (from its cells only).
    fn pop(&mut self) {
        let Some(s) = self.stadiums.pop() else { return };
        let id = self.stadiums.len() as u32;
        let (lo, hi) = s.bounds();
        let (x0, y0) = self.key(lo.x, lo.y);
        let (x1, y1) = self.key(hi.x, hi.y);
        for y in y0..=y1 {
            for x in x0..=x1 {
                if let Some(v) = self.cells.get_mut(&(x, y)) {
                    v.retain(|&i| i != id);
                }
            }
        }
    }

    /// Indices of the stadiums near the disc of `radius` about `c` (each
    /// once, increasing).
    fn near(&self, c: Point2, radius: f64, ids: &mut Vec<u32>) {
        ids.clear();
        let (x0, y0) = self.key(c.x - radius, c.y - radius);
        let (x1, y1) = self.key(c.x + radius, c.y + radius);
        for y in y0..=y1 {
            for x in x0..=x1 {
                if let Some(v) = self.cells.get(&(x, y)) {
                    ids.extend_from_slice(v);
                }
            }
        }
        ids.sort_unstable();
        ids.dedup();
    }
}

/// The per-pass marcher.
struct Marcher<'a> {
    params: &'a ClearParams,
    tol: &'a Tolerance,
    side: f64,
    material: &'a SegmentGrid,
    material_region: &'a Region,
    feasible: &'a SegmentGrid,
    feasible_region: &'a Region,
    chain: Vec<Segment>,
    points: Vec<Point2>,
    stadiums: StadiumGrid,
    scratch: Vec<u32>,
    /// The limit plus the allowance for sampling a chord at a quarter step
    /// (the independent replay's slack).
    chord_limit: f64,
    max_theta: f64,
    exceeded: u64,
    loops: usize,
    samples: u64,
    limit: f64,
    /// The pass starts at the end of a helix entry: the chord from there to
    /// the first front point is the lead into the material, cut and measured,
    /// never a connector that must be clear.
    entry_lead: bool,
}

impl<'a> Marcher<'a> {
    fn theta(&mut self, c: Point2, d: Point2) -> f64 {
        self.theta_with(&[], c, d)
    }

    /// Engagement with extra stadiums (a loop under evaluation).
    fn theta_with(&mut self, extra: &[Stadium], c: Point2, d: Point2) -> f64 {
        self.samples += 1;
        let r = self.params.radius;
        let mut ids = std::mem::take(&mut self.scratch);
        self.stadiums.near(c, r, &mut ids);
        let cleared: Vec<&Stadium> = ids.iter().map(|&i| &self.stadiums.stadiums[i as usize]).chain(extra.iter()).collect();
        let region = self.material_region;
        let tol = self.tol;
        let inside = |p: Point2| region.contains(p, tol);
        let t = engagement(self.material, &inside, &cleared, c, d, r, self.tol).0;
        drop(cleared);
        self.scratch = ids;
        t
    }

    fn record(&mut self, theta: f64) {
        if theta > self.max_theta {
            self.max_theta = theta;
        }
        if theta > self.limit + 1e-9 {
            self.exceeded += 1;
        }
    }

    fn push_point(&mut self, p: Point2) {
        if let Some(&last) = self.points.last() {
            if last.distance(p) <= self.tol.point_eps {
                return;
            }
            // Consecutive collinear chords share one stadium.
            let n = self.stadiums.stadiums.len();
            if n > 0 {
                let prev = &self.stadiums.stadiums[n - 1];
                if prev.b == last && prev.a.distance(prev.b) > 0.0 {
                    let u = prev.b.sub(prev.a).normalized().unwrap();
                    let v = p.sub(last);
                    if u.cross(v).abs() <= 1e-9 * v.length() && u.dot(v) > 0.0 {
                        let a = prev.a;
                        self.stadiums.pop();
                        self.stadiums.push(Stadium::new(a, p, self.params.radius));
                        self.points.push(p);
                        return;
                    }
                }
            }
            self.stadiums.push(Stadium::new(last, p, self.params.radius));
        }
        self.points.push(p);
    }

    /// Append the loop's points without per-chord stadiums; the loop's swept
    /// area is exactly the disc of `rho + r` about its centre.
    fn push_loop_points(&mut self, pts: &[Point2], o: Point2, rho: f64) {
        for &p in pts {
            if self.points.last().map_or(true, |l| l.distance(p) > self.tol.point_eps) {
                self.points.push(p);
            }
        }
        self.stadiums.push(Stadium::new(o, o, rho + self.params.radius));
    }

    /// Circle of `rho` about `o` entirely inside the feasible set.
    fn loop_fits(&self, o: Point2, rho: f64) -> bool {
        let tol = self.tol;
        let region = self.feasible_region;
        if !region.contains(o, tol) {
            return false;
        }
        let mut ids = Vec::new();
        self.feasible.query(Point2::new(o.x - rho, o.y - rho), Point2::new(o.x + rho, o.y + rho), &mut ids);
        for i in ids {
            let s = &self.feasible.segments[i as usize];
            // Any boundary point within rho of o means the circle crosses or
            // touches the boundary.
            // Touching the boundary is allowed (a loop tangent to a wall);
            // crossing it is not.
            if s.distance(o) < rho - tol.distance_eps {
                return false;
            }
        }
        true
    }

    /// Simulate a loop of radius `rho` tangent at `x` (travel direction `t`)
    /// on the cleared side: the sampled centre points and the largest
    /// engagement over them.
    fn eval_loop(&mut self, x: Point2, t: Point2, rho: f64) -> (f64, Vec<Point2>) {
        let n_dir = t.perp().scale(self.side);
        let o = x.add(n_dir.scale(rho));
        let h = self.params.step();
        let n = ((TAU * rho / h).ceil() as usize).clamp(8, 100_000);
        let a0 = x.sub(o).angle();
        let mut pts = Vec::with_capacity(n);
        let mut extra: Vec<Stadium> = Vec::new();
        let mut worst = 0.0f64;
        let mut prev = x;
        for k in 1..=n {
            let a = a0 + self.side * TAU * k as f64 / n as f64;
            let p = if k == n { x } else { o.add(Point2::new(rho * a.cos(), rho * a.sin())) };
            let radial = p.sub(o);
            let d = radial.perp().scale(self.side);
            let th = self.theta_with(&extra, p, d);
            worst = worst.max(th);
            extra.push(Stadium::new(prev, p, self.params.radius));
            pts.push(p);
            prev = p;
        }
        (worst, pts)
    }

    fn commit_loop(&mut self, x: Point2, t: Point2, rho: f64, pts: &[Point2], worst: f64) {
        let n_dir = t.perp().scale(self.side);
        let o = x.add(n_dir.scale(rho));
        let ccw = self.side > 0.0;
        // Four quadrant arcs from x around o.
        let a0 = x.sub(o).angle();
        let mut from = x;
        for k in 1..=4 {
            let a = a0 + self.side * k as f64 * PI / 2.0;
            let to = if k == 4 { x } else { o.add(Point2::new(rho * a.cos(), rho * a.sin())) };
            self.chain.push(Segment::arc(from, to, o, ccw));
            from = to;
        }
        self.push_loop_points(pts, o, rho);
        self.loops += 1;
        self.record(worst);
    }

    /// Straight-advance step of a trochoid of radius `rho` at load `s`
    /// (closed form for a straight front; the exact check decides).
    fn advance_step(&self, rho: f64) -> f64 {
        let (r, s) = (self.params.radius, self.params.load);
        let a = rho + r - s;
        let p = -a + (a * a + 2.0 * s * rho).sqrt();
        p.max(self.params.step())
    }

    /// Loop radius candidates, largest first.
    fn loop_radii(&self, x: Point2) -> Vec<f64> {
        // Every radius down to the minimum is tried; `loop_fits` decides
        // against the feasible set from the loop's own centre, so a loop
        // tangent to a wall the path runs along is allowed.
        let mut rho = if self.params.max_loop_radius > 0.0 { self.params.max_loop_radius } else { self.params.radius };
        let _ = x;
        let min = self.params.min_loop_radius.max(self.params.step());
        let mut out = Vec::new();
        while rho >= min - 1e-12 {
            out.push(rho);
            rho *= 0.7;
        }
        if out.is_empty() && min <= 0.5 * self.params.radius {
            out.push(min);
        }
        out
    }

    /// Try a loop at `x`; commit the first within the limit, else the best
    /// fitting one with the overshoot recorded. Returns the loop radius used.
    fn try_loop(&mut self, x: Point2, t: Point2, warnings: &mut Vec<Diagnostic>) -> Option<f64> {
        let n_dir = t.perp().scale(self.side);
        let mut best: Option<(f64, f64, Vec<Point2>)> = None;
        for rho in self.loop_radii(x) {
            let o = x.add(n_dir.scale(rho));
            if !self.loop_fits(o, rho) {
                continue;
            }
            let (worst, pts) = self.eval_loop(x, t, rho);
            if worst <= self.limit + 1e-9 {
                self.commit_loop(x, t, rho, &pts, worst);
                return Some(rho);
            }
            if best.as_ref().map_or(true, |b| worst < b.1) {
                best = Some((rho, worst, pts));
            }
        }
        if let Some((rho, worst, pts)) = best {
            if warnings.iter().all(|w| w.code != "adaptive-engagement-exceeded") {
                warnings.push(Diagnostic::new(
                    "adaptive-engagement-exceeded",
                    format!("no trochoid loop at ({:.3}, {:.3}) keeps the engagement within {:.1} degrees (best {:.1}); the loop is cut with the overshoot reported", x.x, x.y, self.limit.to_degrees(), worst.to_degrees()),
                ));
            }
            self.commit_loop(x, t, rho, &pts, worst);
            return Some(rho);
        }
        None
    }

    /// The pull a convex front arc of tool-centre radius `front_radius` needs
    /// so that a concentric cut keeps the limit engagement: the cleared set
    /// behind it has radius `rho = front_radius - load`, and a cut at offset
    /// `delta` from it engages `arccos(1 - delta/r - delta/rho + delta^2/(2 rho r))`,
    /// which equals the straight limit at
    /// `delta = (rho + r) - sqrt((rho + r)^2 - 2 s rho)`.
    fn curvature_pull(&self, front_radius: f64) -> f64 {
        let (r, s) = (self.params.radius, self.params.load);
        let rho = (front_radius - s).max(0.0);
        let a = rho + r;
        let delta = a - (a * a - 2.0 * s * rho).max(0.0).sqrt();
        (s - delta).clamp(0.0, (s - self.params.step()).max(0.0))
    }

    /// Pull a front sample `x` (travel tangent `t`) towards the cleared set
    /// until the engagement meets the limit: the curvature-corrected
    /// stepover. `eta_start` is the pull to try first. Returns the accepted
    /// point, its pull and its engagement, or `None` when even the largest
    /// pull (`load - step`) exceeds the limit.
    fn pulled(&mut self, x: Point2, t: Point2, eta_start: f64, eta_cap: f64) -> Option<(Point2, f64, f64)> {
        let n = t.perp().scale(self.side);
        let eta_max = (self.params.load - self.params.step()).max(0.0).min(eta_cap.max(0.0));
        let eta0 = eta_start.clamp(0.0, eta_max);
        let q0 = if eta0 > 0.0 { x.add(n.scale(eta0)) } else { x };
        if eta0 == 0.0 || self.feasible_region.contains(q0, self.tol) {
            let th0 = self.theta(q0, t);
            if th0 <= self.limit + 1e-9 {
                return Some((q0, eta0, th0));
            }
        }
        if eta_max <= eta0 {
            return None;
        }
        let far = x.add(n.scale(eta_max));
        if !self.feasible_region.contains(far, self.tol) {
            return None;
        }
        let th_far = self.theta(far, t);
        if th_far > self.limit + 1e-9 {
            return None;
        }
        let (mut lo, mut hi) = (eta0, eta_max);
        let mut best = (far, eta_max, th_far);
        for _ in 0..12 {
            let mid = 0.5 * (lo + hi);
            let q = x.add(n.scale(mid));
            let th = self.theta(q, t);
            if th <= self.limit + 1e-9 && self.feasible_region.contains(q, self.tol) {
                hi = mid;
                best = (q, mid, th);
            } else {
                lo = mid;
            }
        }
        Some(best)
    }

    /// A front piece shifted by `eta` towards the cleared side: a parallel
    /// line, or the concentric arc (an exact radial shift of both ends).
    fn offset_piece(&self, seg: &Segment, eta: f64) -> Segment {
        if eta <= 0.0 {
            return *seg;
        }
        let shift = |p: Point2, t: Point2| p.add(t.perp().scale(self.side * eta));
        let a = shift(seg.start, seg.tangent_at(0.0));
        let b = shift(seg.end, seg.tangent_at(1.0));
        match seg.arc {
            None => Segment::line(a, b),
            Some(arc) => Segment::arc(a, b, arc.center, arc.ccw),
        }
    }

    /// Whether the chord `a`-`b` of a shifted stretch lies in the (closed)
    /// feasible set. A chord clear of the boundary does when its start is
    /// inside (the caller chains them). A stretch pulled off a front that
    /// runs along the boundary starts on it (a front along the top wall,
    /// pulled down the side wall at the corner): such a chord touches the
    /// boundary only at an end, and `chord_feasible` alone refused it, so
    /// the whole stretch was cut unpulled, far over the engagement limit.
    /// Only when that strict test fails: the chord trimmed by
    /// `100 distance_eps` at both ends clears the boundary, so it lies on
    /// one side, and its start is on the inner side of the nearest boundary
    /// piece (inside is on the left: outer loops counter-clockwise, holes
    /// clockwise). Where the nearest boundary point is a piece's end the side
    /// is decided by a whole-boundary containment test. The chord's own start
    /// is the chain's previous point (or the stretch's first, tested). A
    /// chord that crosses the boundary between its ends is still refused.
    fn stretch_chord_feasible(&self, a: Point2, b: Point2) -> bool {
        if chord_feasible(a, b, self.feasible, self.tol) {
            return true;
        }
        let len = a.distance(b);
        let eps = self.tol.distance_eps;
        let trim = (100.0 * eps).min(0.25 * len);
        if len <= self.tol.point_eps || trim <= 4.0 * eps {
            return false;
        }
        let u = b.sub(a).scale(1.0 / len);
        let (a2, b2) = (a.add(u.scale(trim)), b.sub(u.scale(trim)));
        if !chord_feasible(a2, b2, self.feasible, self.tol) {
            return false;
        }
        let reach = 2.0 * trim;
        let mut ids = Vec::new();
        self.feasible.query(Point2::new(a2.x - reach, a2.y - reach), Point2::new(a2.x + reach, a2.y + reach), &mut ids);
        let nearest = ids
            .iter()
            .map(|&i| {
                let seg = &self.feasible.segments[i as usize];
                let (q, t) = seg.nearest(a2);
                (q.distance(a2), seg, q, t)
            })
            .min_by(|x, y| x.0.total_cmp(&y.0));
        match nearest {
            Some((_, seg, q, t)) if t > 0.0 && t < 1.0 => seg.tangent_at(t).cross(a2.sub(q)) > 0.0,
            _ => {
                STRETCH_WORK.with(|w| w.set((w.get().0, w.get().1 + 1)));
                self.feasible_region.contains(a2, self.tol)
            }
        }
    }

    /// Whether every chord the cut polyline takes along the shifted pieces
    /// (and the bridges between them) lies in the feasible set: the first
    /// point is inside, and no chord touches the boundary but at an end on
    /// it (`stretch_chord_feasible`), so the connected polyline never leaves
    /// it. One containment test per stretch (a winding count over the whole
    /// boundary) unless a chord touches the boundary; every chord on the
    /// boundary grid.
    fn stretch_feasible(&self, pieces: &[Segment]) -> bool {
        STRETCH_WORK.with(|w| w.set((w.get().0 + 1, w.get().1)));
        let mut prev: Option<Point2> = None;
        for seg in pieces {
            let pts = self.piece_points(seg);
            let Some(&first) = pts.first() else { continue };
            match prev {
                None => {
                    STRETCH_WORK.with(|w| w.set((w.get().0, w.get().1 + 1)));
                    // On the boundary counts as inside (the set is closed).
                    let on_boundary = self.feasible.chord_clearance(first, first, 4.0 * self.tol.distance_eps).is_some_and(|d| d <= self.tol.distance_eps);
                    if !on_boundary && !self.feasible_region.contains(first, self.tol) {
                        return false;
                    }
                }
                Some(p) => {
                    if p.distance(first) > self.tol.point_eps && !self.stretch_chord_feasible(p, first) {
                        return false;
                    }
                }
            }
            if !pts.windows(2).all(|w| self.stretch_chord_feasible(w[0], w[1])) {
                return false;
            }
            prev = pts.last().copied();
        }
        true
    }

    /// Chords of an exact piece for the cut polyline: arcs whose centre side
    /// holds the material are discretised outward (the chords never dip into
    /// it), the others inward.
    fn piece_points(&self, seg: &Segment) -> Vec<Point2> {
        let climb = self.side > 0.0;
        match seg.arc {
            None => vec![seg.start, seg.end],
            Some(a) if a.ccw == climb => seg.discretize(self.params.chord_tolerance),
            Some(_) => seg.discretize_outward(self.params.chord_tolerance),
        }
    }

    /// Measure the engagement along a chord about to be cut, four samples per
    /// step as the independent replay does, against the limit plus the
    /// sampling allowance. The front samples alone never saw the chords
    /// emitted between them: the bridge between two stretches pulled by
    /// different amounts, the corner where a pass starts.
    fn measure_chord(&mut self, a: Point2, b: Point2) {
        let len = a.distance(b);
        if len <= self.tol.point_eps {
            return;
        }
        let d = b.sub(a).scale(1.0 / len);
        let n = ((4.0 * len / self.params.step()).ceil() as usize).max(1);
        for k in 1..=n {
            let q = a.add(b.sub(a).scale(k as f64 / n as f64));
            let th = self.theta(q, d);
            if th > self.max_theta {
                self.max_theta = th;
            }
            if th > self.chord_limit + 1e-9 {
                self.exceeded += 1;
                if std::env::var("BREP_CAM_ADAPTIVE_TRACE").is_ok() {
                    eprintln!("adaptive over-limit: chord at ({:.3}, {:.3}): {:.1} deg (allowance {:.1})", q.x, q.y, th.to_degrees(), self.chord_limit.to_degrees());
                }
            }
        }
    }

    /// Append exact pieces to the chain and their chords to the points, each
    /// chord measured.
    fn emit(&mut self, pieces: &[Segment]) {
        for seg in pieces {
            if seg.length() <= self.tol.point_eps {
                continue;
            }
            if let Some(&last) = self.points.last() {
                if last.distance(seg.start) > self.tol.point_eps {
                    self.measure_chord(last, seg.start);
                    self.chain.push(Segment::line(last, seg.start));
                    self.push_point(seg.start);
                }
            }
            self.chain.push(*seg);
            for q in self.piece_points(seg).into_iter().skip(1) {
                if let Some(&last) = self.points.last() {
                    self.measure_chord(last, q);
                }
                self.push_point(q);
            }
        }
    }

    /// A one-turn Archimedean spiral lead from radius `rho0` about `center`
    /// (the tool-centre radius the hole leaves in air) out to `target`, in the
    /// cutting sense, so the radial depth grows from zero to the load over the
    /// turn and the material is met from the side. Chords within the chord
    /// tolerance; the tool first moves radially to the spiral's start in air.
    fn spiral_lead(&mut self, center: Point2, rho0: f64, target: Point2) {
        let v = target.sub(center);
        let rho1 = v.length();
        if rho1 <= rho0 + self.tol.point_eps {
            if let Some(&last) = self.points.last() {
                if last.distance(target) > self.tol.point_eps {
                    self.measure_chord(last, target);
                    self.chain.push(Segment::line(last, target));
                    self.push_point(target);
                }
            }
            return;
        }
        let theta1 = v.angle();
        let turn = self.side * TAU;
        let theta0 = theta1 - turn;
        let at = |t: f64| {
            let rho = rho0 + (rho1 - rho0) * t;
            let a = theta0 + turn * t;
            center.add(Point2::new(rho * a.cos(), rho * a.sin()))
        };
        let max_step = 2.0 * (1.0 - (self.params.chord_tolerance / rho1).min(1.0)).acos();
        let n = ((TAU / max_step.max(1e-3)).ceil() as usize).clamp(16, 20_000);
        let mut prev = *self.points.last().unwrap_or(&center);
        for k in 0..=n {
            let q = if k == n { target } else { at(k as f64 / n as f64) };
            if prev.distance(q) > self.tol.point_eps {
                self.measure_chord(prev, q);
                self.chain.push(Segment::line(prev, q));
                self.push_point(q);
                prev = q;
            }
        }
    }

    /// The lead out of a helix entry of tool-centre radius `rho0` about
    /// `center` (the pass's last point is the helix end, on that circle in the
    /// direction of `target`) to `target`: one straight step along the
    /// helix's own tangent, so the linker's helix (centred from the pass's
    /// start tangent) bores the disc the solver removed, then a one-turn
    /// Archimedean spiral out to `target`, the radial depth growing from zero
    /// to the load over the turn as in `spiral_lead`. Every chord is measured.
    /// The spiral is wider than the helix, so beside a wall it can leave the
    /// feasible set: the lead is emitted only when every chord stays inside,
    /// and `false` (nothing emitted) otherwise.
    fn helix_lead(&mut self, center: Point2, rho0: f64, target: Point2) -> bool {
        let Some(&x0) = self.points.last() else { return false };
        let v = target.sub(center);
        let rho1 = v.length();
        let mut lead_points = Vec::new();
        if rho1 <= rho0 + self.tol.point_eps || rho0 <= self.tol.point_eps {
            if x0.distance(target) > self.tol.point_eps {
                lead_points.push(target);
            }
        } else {
            let theta1 = v.angle();
            let turn = self.side * TAU;
            let tangent = Point2::new(theta1.cos(), theta1.sin()).perp().scale(self.side);
            let lead = self.params.step().min(0.5 * rho0);
            let q1 = x0.add(tangent.scale(lead));
            lead_points.push(q1);
            let t_start = lead / (TAU * rho0);
            let at = |t: f64| {
                let rho = rho0 + (rho1 - rho0) * t;
                let a = theta1 - turn + turn * t;
                center.add(Point2::new(rho * a.cos(), rho * a.sin()))
            };
            let max_step = 2.0 * (1.0 - (self.params.chord_tolerance / rho1).min(1.0)).acos();
            let n = ((TAU / max_step.max(1e-3)).ceil() as usize).clamp(16, 20_000);
            let mut prev = q1;
            for k in 1..=n {
                let q = if k == n { target } else { at(t_start + (1.0 - t_start) * k as f64 / n as f64) };
                if prev.distance(q) > self.tol.point_eps {
                    lead_points.push(q);
                    prev = q;
                }
            }
        }
        if !self.feasible_region.contains(x0, self.tol) {
            return false;
        }
        let mut prev = x0;
        for &q in &lead_points {
            if !chord_feasible(prev, q, self.feasible, self.tol) {
                return false;
            }
            prev = q;
        }
        let mut prev = x0;
        for q in lead_points {
            self.measure_chord(prev, q);
            self.chain.push(Segment::line(prev, q));
            self.push_point(q);
            prev = q;
        }
        true
    }

    /// Whether the chord from the pass's last point to the front's first cut
    /// may join them: a connector must be clear of material and stay
    /// feasible; after a helix lead the chord is part of the lead (it cuts,
    /// and it is measured), so only feasibility is required.
    fn joins(&mut self, a: Point2, b: Point2) -> bool {
        if self.entry_lead {
            if !chord_feasible(a, b, self.feasible, self.tol) {
                return false;
            }
            self.measure_chord(a, b);
            true
        } else {
            connector_clear(a, b, self.material, self.material_region, self.feasible, self.params.radius, self.tol)
        }
    }

    /// The first point of the pass along `segments` (the first sample pulled
    /// by the curvature's pull), for the caller's connector check.
    fn first_point(&mut self, segments: &[Segment]) -> Point2 {
        let x0 = segments[0].start;
        let t0 = segments[0].tangent_at(0.0);
        let climb = self.side > 0.0;
        let (pull, cap) = match segments[0].arc {
            Some(a) if a.ccw == climb => (self.curvature_pull(segments[0].radius()), segments[0].radius() - self.params.step()),
            _ => (0.0, f64::INFINITY),
        };
        self.pulled(x0, t0, pull, cap).map_or(x0, |(q, _, _)| q)
    }

    /// March along `segments` (a front piece). Samples are pulled towards the
    /// cleared set where the full load exceeds the limit; a run of samples on
    /// one exact front piece is emitted as that piece shifted by the largest
    /// pull its samples needed (an exact concentric arc or parallel line), so
    /// consecutive passes stay exactly concentric; trochoids are cut where no
    /// pull suffices. Returns the chain index of the first cutting segment,
    /// or `None` when the connector from the current position to the pass
    /// start is not clear (the caller starts a fresh pass).
    fn run_front(&mut self, segments: Vec<Segment>, warnings: &mut Vec<Diagnostic>, runner: &mut Runner<'_>) -> Result<Option<usize>, CamError> {
        let cursor = Cursor::new(segments);
        let length = cursor.length();
        if length <= self.tol.point_eps {
            return Ok(Some(self.chain.len()));
        }
        let h = self.params.step();
        let climb = self.side > 0.0;
        let connector_from = self.points.last().copied();
        let mut first_cut: Option<usize> = None;
        // The open stretch: raw front interval and the pull it will be shifted by.
        struct Stretch {
            s_start: f64,
            piece: usize,
            eta: f64,
            provisional: usize,
        }
        let mut stretch: Option<Stretch> = None;
        let mut s_cur = 0.0;
        let mut trochoid = false;
        let mut rho_cur = 0.0;
        let mut eta_prev = 0.0;
        let mut guard: u64 = 0;
        // Where trochoid mode last found the front clear and handed back
        // without advancing: if no pull fits there either, the two modes
        // would hand over forever (seed 74 run 40).
        let mut clear_at: Option<f64> = None;
        // Flush the open stretch up to `s_end`: exact pieces shifted by its pull.
        macro_rules! flush {
            ($st:expr, $s_end:expr) => {{
                let st: Stretch = $st;
                for _ in 0..st.provisional {
                    self.stadiums.pop();
                    self.points.pop();
                }
                if $s_end > st.s_start + 1e-9 {
                    let raw = cursor.pieces(st.s_start, $s_end, cursor.point(st.s_start), cursor.point($s_end));
                    let mut shifted: Vec<Segment> = raw.iter().map(|seg| self.offset_piece(seg, st.eta)).collect();
                    // The pull was proven at the stretch's samples only; a
                    // shifted piece beside the feasible boundary (a front
                    // along a containment edge) can leave it between them.
                    // The raw front lies in the feasible set, so such a
                    // stretch is cut unshifted; its chords are measured.
                    if st.eta > 0.0 && !self.stretch_feasible(&shifted) {
                        shifted = raw.clone();
                    }
                    if first_cut.is_none() {
                        if let (Some(from), Some(first)) = (connector_from, shifted.first()) {
                            if from.distance(first.start) > self.tol.point_eps {
                                if !self.joins(from, first.start) {
                                    return Ok(None);
                                }
                                self.chain.push(Segment::line(from, first.start));
                            }
                        }
                        if self.points.is_empty() {
                            if let Some(first) = shifted.first() {
                                self.points.push(first.start);
                            }
                        }
                        first_cut = Some(self.chain.len());
                    }
                    self.emit(&shifted);
                }
            }};
        }
        while s_cur < length - 1e-9 {
            guard += 1;
            if guard > 10_000_000 || self.samples > self.params.max_samples {
                return Err(invalid("engagement sample budget exceeded"));
            }
            runner.work(1, "adaptive-march", 0, 0)?;
            if !trochoid {
                let s_next = (s_cur + h).min(length);
                let x = cursor.point(s_next);
                let t = cursor.tangent(s_next);
                let (idx, _) = cursor.locate(s_next);
                let seg = &cursor.segments[idx];
                // A convex arc piece is pulled at most to a radius of one step,
                // so the shifted arc never collapses.
                let (convex_pull, eta_cap) = match seg.arc {
                    Some(a) if a.ccw == climb => (self.curvature_pull(seg.radius()), seg.radius() - h),
                    _ => (0.0, f64::INFINITY),
                };
                let same_piece = stretch.as_ref().map_or(false, |st| st.piece == idx);
                let floor = if same_piece { stretch.as_ref().unwrap().eta } else { (eta_prev - 0.25 * h).max(0.0) };
                let eta_start = convex_pull.max(floor).min(eta_cap.max(0.0));
                match self.pulled(x, t, eta_start, eta_cap) {
                    Some((q, eta, th)) => {
                        self.record(th);
                        if !same_piece {
                            if let Some(st) = stretch.take() {
                                // The previous piece ends at its own end.
                                let s_end = cursor.cumulative[idx].min(length);
                                flush!(st, s_end);
                                stretch = Some(Stretch { s_start: s_end, piece: idx, eta, provisional: 0 });
                            } else {
                                stretch = Some(Stretch { s_start: s_cur, piece: idx, eta, provisional: 0 });
                            }
                        } else if let Some(st) = stretch.as_mut() {
                            st.eta = st.eta.max(eta);
                        }
                        // Provisional point for the stretch's own trailing stadiums.
                        let before = self.points.len();
                        if first_cut.is_some() || self.points.len() > 0 || stretch.as_ref().map_or(false, |st| st.provisional > 0) {
                            self.push_point(q);
                        } else {
                            self.points.push(q);
                        }
                        if let Some(st) = stretch.as_mut() {
                            st.provisional += self.points.len() - before;
                        }
                        s_cur = s_next;
                        eta_prev = eta;
                        continue;
                    }
                    None => {}
                }
                // No pull suffices: flush and enter trochoid mode at the raw front.
                if let Some(st) = stretch.take() {
                    flush!(st, s_cur);
                }
                let p_cur = cursor.point(s_cur);
                let t_cur = cursor.tangent(s_cur);
                if first_cut.is_none() {
                    if let Some(from) = connector_from {
                        if from.distance(p_cur) > self.tol.point_eps {
                            if !self.joins(from, p_cur) {
                                return Ok(None);
                            }
                            self.chain.push(Segment::line(from, p_cur));
                        }
                    }
                    first_cut = Some(self.chain.len());
                }
                if self.points.last().map_or(true, |l| l.distance(p_cur) > self.tol.point_eps) {
                    if let Some(&last) = self.points.last() {
                        self.measure_chord(last, p_cur);
                        self.chain.push(Segment::line(last, p_cur));
                    }
                    self.push_point(p_cur);
                }
                let th_here = self.theta(x, t);
                runner.stage_detail("adaptive-trochoid", "Trochoid mode", &format!("at ({:.3}, {:.3}) s={:.3}/{:.3}: front {:.1} deg", p_cur.x, p_cur.y, s_cur, length, th_here.to_degrees()), 0, 0)?;
                // Trochoid mode just found this front clear, so no loop is
                // needed: the raw front is cut (no pull beside the
                // containment edge is feasible).
                let handed_back = clear_at == Some(s_cur);
                match if handed_back { None } else { self.try_loop(p_cur, t_cur, warnings) } {
                    Some(rho) => {
                        trochoid = true;
                        rho_cur = rho;
                    }
                    None => {
                        // No loop fits: cut the raw front with the overshoot recorded.
                        self.record(th_here);
                        let pieces = cursor.pieces(s_cur, s_next, p_cur, x);
                        self.emit(&pieces);
                        s_cur = s_next;
                        eta_prev = 0.0;
                        if (!handed_back || th_here > self.limit + 1e-9) && warnings.iter().all(|w| w.code != "adaptive-engagement-exceeded") {
                            warnings.push(Diagnostic::new(
                                "adaptive-engagement-exceeded",
                                format!("no trochoid loop fits at ({:.3}, {:.3}); the front is cut with engagement {:.1} degrees over the {:.1} degree limit", p_cur.x, p_cur.y, th_here.to_degrees(), self.limit.to_degrees()),
                            ));
                        }
                    }
                }
                continue;
            }
            // Trochoid mode: probe whether the front (pulled or not) is clear ahead.
            let probes = ((self.params.load / h).ceil() as usize).max(1);
            let mut clear = true;
            for j in 1..=probes {
                let s_p = (s_cur + j as f64 * h).min(length);
                let x = cursor.point(s_p);
                let t = cursor.tangent(s_p);
                let n = t.perp().scale(self.side);
                let th0 = self.theta(x, t);
                let ok = th0 <= self.limit + 1e-9 || {
                    let far = x.add(n.scale((self.params.load - h).max(0.0)));
                    self.feasible_region.contains(far, self.tol) && self.theta(far, t) <= self.limit + 1e-9
                };
                if !ok {
                    clear = false;
                    break;
                }
                if s_p >= length {
                    break;
                }
            }
            if clear {
                trochoid = false;
                eta_prev = 0.0;
                clear_at = Some(s_cur);
                continue;
            }
            // Advance by the trochoid step and loop again, shrinking the step
            // until the loop's exact engagement is within the limit.
            let p_cur = cursor.point(s_cur);
            let mut p = self.advance_step(rho_cur);
            let mut committed = false;
            let mut attempts = 0;
            while !committed {
                attempts += 1;
                let s_next = (s_cur + p).min(length);
                let p_next = cursor.point(s_next);
                let t_next = cursor.tangent(s_next);
                let th_adv = self.theta(p_next, t_next);
                let n_dir = t_next.perp().scale(self.side);
                let o = p_next.add(n_dir.scale(rho_cur));
                let fits = self.loop_fits(o, rho_cur);
                let (worst, pts) = if fits { self.eval_loop(p_next, t_next, rho_cur) } else { (f64::INFINITY, Vec::new()) };
                let ok = fits && worst.max(th_adv) <= self.limit + 1e-9;
                if ok || p <= h + 1e-12 || attempts >= 12 || s_next >= length {
                    let pieces = cursor.pieces(s_cur, s_next, p_cur, p_next);
                    self.emit(&pieces);
                    self.record(th_adv);
                    s_cur = s_next;
                    if s_cur < length - 1e-9 {
                        if ok {
                            self.commit_loop(p_next, t_next, rho_cur, &pts, worst);
                        } else {
                            match self.try_loop(p_next, t_next, warnings) {
                                Some(rho) => rho_cur = rho,
                                None => trochoid = false,
                            }
                        }
                    }
                    committed = true;
                } else {
                    p *= 0.7;
                }
            }
        }
        if let Some(st) = stretch.take() {
            flush!(st, length);
        }
        Ok(Some(first_cut.unwrap_or(self.chain.len())))
    }
}

/// Pieces of the front `∂F` whose tool capsule meets material, chained.
fn front_chains(front: &Region, material: &SegmentGrid, material_region: &Region, radius: f64, climb: bool, tol: &Tolerance) -> Vec<(Vec<Segment>, bool)> {
    let mut out: Vec<(Vec<Segment>, bool)> = Vec::new();
    let mut ids = Vec::new();
    for l in &front.loops {
        let n = l.segments.len();
        if n == 0 {
            continue;
        }
        let keep: Vec<bool> = l
            .segments
            .iter()
            .map(|s| {
                let (lo, hi) = s.bounds();
                material.query(Point2::new(lo.x - radius, lo.y - radius), Point2::new(hi.x + radius, hi.y + radius), &mut ids);
                let mut touches = ids.iter().any(|&i| segment_distance(&material.segments[i as usize], s) < radius - tol.distance_eps);
                if !touches && ids.is_empty() {
                    // No boundary near: buried or in air; buried is material.
                    touches = material_region.contains(s.midpoint(), tol);
                }
                touches
            })
            .collect();
        if keep.iter().all(|k| *k) {
            out.push((l.segments.clone(), true));
            continue;
        }
        if !keep.iter().any(|k| *k) {
            continue;
        }
        // Rotate so index 0 is a dropped segment, then collect runs.
        let first_drop = keep.iter().position(|k| !*k).unwrap();
        let mut run: Vec<Segment> = Vec::new();
        for k in 0..n {
            let i = (first_drop + k) % n;
            if keep[i] {
                run.push(l.segments[i]);
            } else if !run.is_empty() {
                out.push((std::mem::take(&mut run), false));
            }
        }
        if !run.is_empty() {
            out.push((run, false));
        }
    }
    if !climb {
        for (segs, _) in &mut out {
            segs.reverse();
            for s in segs.iter_mut() {
                *s = s.reversed();
            }
        }
    }
    out
}

/// Rotate a closed chain so it starts at the vertex nearest `p`.
fn rotate_to_nearest(segs: &mut Vec<Segment>, p: Point2) {
    if segs.len() < 2 {
        return;
    }
    let mut best = 0;
    let mut bd = f64::INFINITY;
    for (i, s) in segs.iter().enumerate() {
        let d = s.start.distance(p);
        if d < bd {
            bd = d;
            best = i;
        }
    }
    segs.rotate_left(best);
}

fn grid_of(region: &Region, cell: f64, tol: &Tolerance) -> SegmentGrid {
    let segs: Vec<Segment> = region.loops.iter().flat_map(|l| l.segments.iter().copied()).collect();
    SegmentGrid::new(&segs, cell, tol)
}

/// A straight repositioning chord `a`-`b` at the cut height is clear when the
/// tool along it touches no material and stays inside the feasible set.
fn connector_clear(a: Point2, b: Point2, material: &SegmentGrid, material_region: &Region, feasible: &SegmentGrid, radius: f64, tol: &Tolerance) -> bool {
    if a.distance(b) <= tol.point_eps {
        return true;
    }
    if let Some(d) = material.chord_clearance(a, b, radius + tol.distance_eps) {
        if d < radius + tol.distance_eps {
            return false;
        }
    } else if material_region.contains(a.add(b).scale(0.5), tol) {
        return false;
    }
    chord_feasible(a, b, feasible, tol)
}

/// The chord `a`–`b` (both feasible) does not touch the feasible set's
/// boundary, so every tool-centre point on it is feasible.
fn chord_feasible(a: Point2, b: Point2, feasible: &SegmentGrid, tol: &Tolerance) -> bool {
    if let Some(d) = feasible.chord_clearance(a, b, 4.0 * tol.distance_eps) {
        if d <= tol.distance_eps {
            return false;
        }
    }
    true
}

/// Clear one level. See the module documentation.
pub fn clear_level(input: &ClearInput<'_>, params: &ClearParams, tol: &Tolerance, runner: &mut Runner<'_>) -> Result<LevelResult, CamError> {
    STRETCH_WORK.with(|w| w.set((0, 0)));
    params.validate()?;
    tol.validate()?;
    let r = params.radius;
    let s = params.load;
    let limit = params.limit();
    let chord_limit = super::engagement::engagement_for_depth(params.load + params.step() + 2.0 * params.chord_tolerance, params.radius).max(limit);
    let feasible = input.feasible;
    if feasible.is_empty() {
        return Err(invalid("the tool-centre feasible set is empty (nothing the tool can reach)"));
    }
    let ct = params.chord_tolerance;
    let reach = grow(feasible, r, ct, tol)?;
    let feasible_grid = grid_of(feasible, 2.0 * r, tol);
    let mut material = input.material.clone();
    let mut entries: Vec<Entry> = Vec::new();
    let mut pending: Vec<Entry> = Vec::new();
    for e in &input.entries {
        if !feasible.contains(e.center, tol) {
            return Err(invalid(format!("entry point ({:.3}, {:.3}) is not a feasible tool position", e.center.x, e.center.y)));
        }
        if e.cleared_radius > 0.0 {
            let hole = disc(e.center, e.cleared_radius, tol)?;
            material = checked("entry disc", &material, &hole, || material.difference(&hole, tol))?;
        }
        entries.push(*e);
        pending.push(*e);
    }
    let mut warnings: Vec<Diagnostic> = Vec::new();
    let mut passes: Vec<LevelPass> = Vec::new();
    let mut position: Option<Point2> = input.start;
    let mut max_theta = 0.0f64;
    let mut exceeded = 0u64;
    let mut loops = 0usize;
    let mut samples = 0u64;
    let mut rounds = 0usize;
    let mut remaining = material.intersection(&reach, tol)?;
    let mut last_area = f64::INFINITY;
    let mut stalls = 0;
    let helix_envelope = if params.helix_diameter > 0.0 { 0.5 * params.helix_diameter + r } else { 0.0 };
    // Done when what is left is thinner than the chord tolerance everywhere:
    // a sliver of width w has area at most w times half its perimeter.
    let done = |rem: &Region| -> bool {
        if rem.is_empty() {
            return true;
        }
        let perimeter: f64 = rem.loops.iter().map(Loop::length).sum();
        rem.area() <= params.chord_tolerance * 0.5 * perimeter + tol.area_eps
    };
    while !done(&remaining) {
        rounds += 1;
        if rounds > params.max_rounds {
            return Err(invalid("round budget exceeded"));
        }
        runner.stage_detail("adaptive-round", "Adaptive clearing round", &format!("{:.3} area left", remaining.area()), rounds as u64, 0)?;
        let area = remaining.area();
        if area >= last_area - tol.area_eps {
            stalls += 1;
            if stalls >= 3 {
                warnings.push(Diagnostic::new("adaptive-unreachable-material", format!("{area:.4} of reachable material could not be cleared within the engagement model; it is left and reported")));
                break;
            }
        } else {
            stalls = 0;
        }
        last_area = area;
        // Cleared tool-centre set and the next front.
        let grown = grow(&material, r, ct, tol)?;
        if std::env::var("BREP_CAM_ADAPTIVE_CHECK").is_ok() {
            // Diagnostics only: every vertex of the dilated material lies at
            // most r from the material, and every material vertex inside it.
            let mut worst: Option<(Point2, f64)> = None;
            for l in &grown.loops {
                for seg in &l.segments {
                    let q = seg.midpoint();
                    let d = -material.signed_distance(q, tol);
                    if (d - r).abs() > 1e-3 && d > 0.0 && worst.map_or(true, |w| (d - r).abs() > (w.1 - r).abs()) {
                        worst = Some((q, d));
                    }
                }
            }
            let probe_points: Vec<Point2> = material.loops.iter().flat_map(|l| l.segments.iter().map(|s| s.midpoint())).collect();
            let missing = probe_points.iter().filter(|q| !grown.contains(**q, tol) && grown.nearest_boundary(**q).map_or(true, |b| b.1 > 1e-6)).count();
            if worst.is_some() || missing > 0 {
                eprintln!("adaptive check round {rounds}: grown boundary point at distance {:?} from material (want {r}), {missing} material boundary midpoints outside grown", worst);
                if let Ok(path) = std::env::var("BREP_CAM_ADAPTIVE_DUMP") {
                    let dump = serde_json::json!({ "material": material, "grown": grown, "round": rounds, "radius": r });
                    let _ = std::fs::write(format!("{path}.grow{rounds}.json"), serde_json::to_string(&dump).unwrap_or_default());
                }
            }
        }
        let mut cleared = checked("cleared = feasible - grown", feasible, &grown, || feasible.difference(&grown, tol))?;
        check_difference("cleared = feasible - grown", feasible, &grown, &cleared, tol)?;
        for e in &pending {
            if !e.helix {
                // A plunge or a predrilled hole clears a disc the tool centre
                // can barely enter: seed the front with a small disc about the
                // entry (no shorter than the dilation's short-piece floor).
                let seed = disc(e.center, (e.cleared_radius - r).max(0.0).max(2.0 * ct), tol)?;
                cleared = checked("seed union", &cleared, &seed, || cleared.union(&seed, tol))?;
            }
        }
        let widened = grow(&cleared, s, ct, tol)?;
        let front = checked("front = widened & feasible", &widened, feasible, || widened.intersection(feasible, tol))?;
        let material_grid = grid_of(&material, 2.0 * r, tol);
        let mut chains = front_chains(&front, &material_grid, &material, r, params.climb, tol);
        let seg_count = |g: &Region| g.loops.iter().map(|l| l.segments.len()).sum::<usize>();
        runner.stage_detail(
            "adaptive-geometry",
            "Adaptive front",
            &format!("material {} segs / {} loops, cleared {} segs, front {} segs, {} chains", seg_count(&material), material.loops.len(), seg_count(&cleared), seg_count(&front), chains.len()),
            rounds as u64,
            0,
        )?;
        if chains.is_empty() {
            if !cleared.is_empty() && front.is_empty() {
                // An empty offset of a non-empty cleared set: dump the round for
                // a reproducer when asked (test diagnostics only).
                if let Ok(path) = std::env::var("BREP_CAM_ADAPTIVE_DUMP") {
                    let dump = serde_json::json!({ "material": material, "feasible": feasible, "cleared": cleared, "front": front, "round": rounds, "load": s, "radius": r });
                    let _ = std::fs::write(format!("{path}.round{rounds}.json"), serde_json::to_string(&dump).unwrap_or_default());
                }
            }
            // Nothing reachable from the cleared set: seed a new entry at the
            // deepest point of the uncleared feasible set.
            let uncleared = checked("uncleared = feasible - cleared", feasible, &cleared, || feasible.difference(&cleared, tol))?;
            let Some((seed, clearance)) = deepest_point(&uncleared, 0.25 * params.step(), tol)? else {
                warnings.push(Diagnostic::new("adaptive-unreachable-material", format!("{area:.4} of reachable material has no feasible tool position; it is left and reported")));
                break;
            };
            if entries.iter().any(|e| e.center.distance(seed) <= params.step()) {
                // A second entry at the same point means the cleared set was
                // lost: a geometry failure, never silently retried.
                if let Ok(path) = std::env::var("BREP_CAM_ADAPTIVE_DUMP") {
                    let dump = serde_json::json!({ "material": material, "feasible": feasible, "cleared": cleared, "front": front, "round": rounds, "radius": r, "load": s });
                    let _ = std::fs::write(&path, serde_json::to_string(&dump).unwrap_or_default());
                }
                return Err(invalid(format!(
                    "the cleared tool-centre set vanished at round {rounds} (material {} segments, cleared {} segments, {area:.4} area left); the level is refused",
                    seg_count(&material),
                    seg_count(&cleared)
                )));
            }
            // The helix must fit the feasible set; otherwise plunge.
            let helix_ok = helix_envelope > 0.0 && clearance >= 0.5 * params.helix_diameter;
            // A plunge removes the tool's own disc.
            let entry = Entry { center: seed, cleared_radius: if helix_ok { helix_envelope } else { r }, helix: helix_ok };
            if !helix_ok {
                let code = if params.helix_diameter > 0.0 { "adaptive-helix-does-not-fit" } else { "adaptive-plunge-entry" };
                warnings.push(Diagnostic::new(code, format!("entry at ({:.3}, {:.3}) is a plunge: the first ring out of a plunged hole is a full-width slot and exceeds the engagement limit; use a helix or a predrilled hole where it fits", seed.x, seed.y)));
            }
            if entry.cleared_radius > 0.0 {
                let hole = disc(seed, entry.cleared_radius, tol)?;
                material = checked("entry disc", &material, &hole, || material.difference(&hole, tol))?;
                remaining = checked("remaining", &material, &reach, || material.intersection(&reach, tol))?;
            }
            entries.push(entry);
            pending.push(entry);
            position = Some(seed);
            continue;
        }
        // Cut every front piece, nearest first.
        let mut cursor_pos = position.unwrap_or_else(|| chains[0].0[0].start);
        while !chains.is_empty() {
            // Nearest chain (by start, closed chains by their nearest vertex).
            let mut best = 0;
            let mut bd = f64::INFINITY;
            for (i, (segs, closed)) in chains.iter().enumerate() {
                let d = if *closed { segs.iter().map(|s| s.start.distance(cursor_pos)).fold(f64::INFINITY, f64::min) } else { segs[0].start.distance(cursor_pos) };
                if d < bd {
                    bd = d;
                    best = i;
                }
            }
            let (mut segs, closed) = chains.remove(best);
            if closed {
                rotate_to_nearest(&mut segs, cursor_pos);
            }
            let material_grid = grid_of(&material, 2.0 * r, tol);
            let side = if params.climb { 1.0 } else { -1.0 };
            let mut m = Marcher {
                params,
                tol,
                side,
                material: &material_grid,
                material_region: &material,
                feasible: &feasible_grid,
                feasible_region: feasible,
                chain: Vec::new(),
                points: Vec::new(),
                stadiums: StadiumGrid::new(2.0 * r),
                scratch: Vec::new(),
                chord_limit,
                max_theta: 0.0,
                exceeded: 0,
                loops: 0,
                samples: 0,
                limit,
                entry_lead: false,
            };
            let start = m.first_point(&segs);
            // Continue the previous pass when the straight connector is clear.
            let continue_pass = match passes.last() {
                Some(last) if position.is_some() && pending.is_empty() => {
                    let from = *last.points.last().unwrap();
                    let within = params.max_connector <= 0.0 || from.distance(start) <= params.max_connector;
                    within && connector_clear(from, start, &material_grid, &material, &feasible_grid, r, tol)
                }
                _ => false,
            };
            let mut entry_for_pass: Option<Entry> = None;
            let mut lead_from: Option<usize> = None;
            let mut popped: Option<LevelPass> = None;
            if continue_pass {
                let last = passes.pop().unwrap();
                popped = Some(last.clone());
                m.chain = last.chain.segments;
                for p in &last.points {
                    m.push_point(*p);
                }
                m.max_theta = last.max_engagement;
                m.exceeded = last.exceeded;
                m.loops = last.loops;
                entry_for_pass = last.entry;
            } else if let Some(k) = pending.iter().position(|e| e.center.distance(start) <= e.cleared_radius.max(r) + s + 1e-6) {
                // The pass starts with the entry nearest its first front
                // point: at the plunge point, or at the end of the helix (its
                // centre is the seed, offset from the start by half the
                // diameter on the cleared side).
                let e = pending.remove(k);
                let helix = e.helix && e.cleared_radius > 0.0;
                let rho_h = 0.5 * params.helix_diameter;
                let x0 = if helix {
                    let u = start.sub(e.center).normalized().unwrap_or(Point2::new(1.0, 0.0));
                    e.center.add(u.scale(rho_h))
                } else {
                    e.center
                };
                m.push_point(x0);
                // Whatever joins the helix lead to the front's first stretch
                // is part of the lead: cut and measured, never refused as a
                // connector for touching material (a refusal would drop the
                // entry from the pass). A lead that would leave the feasible
                // set is not emitted, and the pass joins as before it existed.
                let chain_before = m.chain.len();
                if helix && m.helix_lead(e.center, rho_h, start) {
                    m.entry_lead = true;
                    lead_from = Some(chain_before);
                } else if !e.helix {
                    // A plunge or predrilled hole: spiral out to the first front
                    // point instead of moving radially into the hole's wall.
                    lead_from = Some(m.chain.len());
                    m.spiral_lead(e.center, (e.cleared_radius - r).max(0.0), start);
                }
                entry_for_pass = Some(e);
            }
            #[allow(unused_assignments)]
            let mut material_before = Region::empty();
            let first_cut = match m.run_front(segs.clone(), &mut warnings, runner)? {
                // The spiral lead cuts: it belongs to the swept run.
                Some(i) => lead_from.map_or(i, |l| l.min(i)),
                None => {
                    // The connector was not clear after all: a fresh pass; the
                    // previous one stays as it was.
                    if let Some(prev) = popped.take() {
                        passes.push(prev);
                    }
                    let mut fresh = Marcher {
                        params,
                        tol,
                        side,
                        material: &material_grid,
                        material_region: &material,
                        feasible: &feasible_grid,
                        feasible_region: feasible,
                        chain: Vec::new(),
                        points: Vec::new(),
                        stadiums: StadiumGrid::new(2.0 * r),
                        scratch: Vec::new(),
                        chord_limit,
                        max_theta: 0.0,
                        exceeded: 0,
                        loops: 0,
                        samples: m.samples,
                        limit,
                        entry_lead: false,
                    };
                    let i = fresh.run_front(segs, &mut warnings, runner)?.unwrap_or(0);
                    m = fresh;
                    entry_for_pass = None;
                    i
                }
            };
            samples += m.samples;
            max_theta = max_theta.max(m.max_theta);
            exceeded += m.exceeded;
            loops += m.loops;
            let (chain_segments, points, pass_max, pass_loops, pass_exceeded) = (m.chain, m.points, m.max_theta, m.loops, m.exceeded);
            if points.len() >= 2 && first_cut < chain_segments.len() {
                // The earlier part of a continued pass is already fitted and
                // already removed from the material; only the new cutting run
                // is fitted and swept. Connectors cut nothing.
                let mut segments: Vec<Segment> = chain_segments[..first_cut].to_vec();
                let fitted_run: Vec<Segment> = chain_segments[first_cut..].to_vec();
                let mut run_chain = Chain { segments: fitted_run.clone(), closed: false };
                run_chain.validate(tol).map_err(|e| invalid(format!("pass chain invalid: {e}")))?;
                let swept = swept_region(&run_chain.segments, r, params.chord_tolerance, tol)?;
                material_before = material.clone();
                let before = material.area();
                let after = checked("material - swept", &material, &swept, || material.difference(&swept, tol))?;
                let removed = checked("material & swept", &material, &swept, || material.intersection(&swept, tol))?.area();
                let residual = (after.area() + removed - before).abs();
                if residual > 1e-6 * before.max(1.0) {
                    if let Ok(path) = std::env::var("BREP_CAM_ADAPTIVE_DUMP") {
                        let dump = serde_json::json!({ "material": material, "run": run_chain, "swept": swept, "after": after, "round": rounds, "radius": r });
                        let _ = std::fs::write(format!("{path}.stall{rounds}.json"), serde_json::to_string(&dump).unwrap_or_default());
                    }
                    return Err(invalid(format!("material bookkeeping failed at round {rounds}: {before:.6} - {removed:.6} removed gives {:.6} (residual {residual:.3e}); the level is refused", after.area())));
                }
                material = after;
                segments.extend(fitted_run);
                let chain = Chain { segments, closed: false };
                position = points.last().copied();
                cursor_pos = position.unwrap();
                if let (Ok(path), Ok(want)) = (std::env::var("BREP_CAM_ADAPTIVE_DUMP"), std::env::var("BREP_CAM_ADAPTIVE_PASS")) {
                    if want.parse::<usize>().ok() == Some(passes.len()) {
                        let dump = serde_json::json!({ "material_before": material_before, "front": front, "cleared": cleared, "feasible": feasible, "chain": chain, "points": points.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(), "first_cut": first_cut, "round": rounds, "radius": r, "load": s });
                        let _ = std::fs::write(format!("{path}.pass{}.json", passes.len()), serde_json::to_string(&dump).unwrap_or_default());
                    }
                }
                passes.push(LevelPass { chain, points, entry: entry_for_pass, max_engagement: pass_max, loops: pass_loops, exceeded: pass_exceeded, material_after: material.area() });
            } else if continue_pass {
                // Nothing was cut: keep the previous pass as it was.
                let chain = Chain { segments: chain_segments[..first_cut.min(chain_segments.len())].to_vec(), closed: false };
                if let (Ok(path), Ok(want)) = (std::env::var("BREP_CAM_ADAPTIVE_DUMP"), std::env::var("BREP_CAM_ADAPTIVE_PASS")) {
                    if want.parse::<usize>().ok() == Some(passes.len()) {
                        let dump = serde_json::json!({ "material_before": material_before, "front": front, "cleared": cleared, "feasible": feasible, "chain": chain, "points": points.iter().map(|q| [q.x, q.y]).collect::<Vec<_>>(), "first_cut": first_cut, "round": rounds, "radius": r, "load": s });
                        let _ = std::fs::write(format!("{path}.pass{}.json", passes.len()), serde_json::to_string(&dump).unwrap_or_default());
                    }
                }
                passes.push(LevelPass { chain, points, entry: entry_for_pass, max_engagement: pass_max, loops: pass_loops, exceeded: pass_exceeded, material_after: material.area() });
            }
            if let Some(last) = passes.last() {
                runner.stage_detail("adaptive-pass", "Adaptive pass", &format!("{} points, {} loops, {} chain segments, max {:.1} deg", last.points.len(), last.loops, last.chain.segments.len(), last.max_engagement.to_degrees()), passes.len() as u64, 0)?;
            }
        }
        remaining = checked("remaining = material & reach", &material, &reach, || material.intersection(&reach, tol))?;
    }
    let remaining_area = remaining.area();
    let (stretches_checked, stretch_containment_tests) = STRETCH_WORK.with(|w| w.get());
    Ok(LevelResult { passes, limit, max_engagement: max_theta, exceeded, loops, rounds, remaining, remaining_area, entries, samples, warnings, stretches_checked, stretch_containment_tests })
}
