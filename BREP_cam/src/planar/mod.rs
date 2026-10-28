//! The 2D geometry engine (lane C5): regions bounded by lines and EXACT arcs,
//! robust booleans and offsets, point containment and distances.
//!
//! A [`Region`] is a set of oriented closed [`Loop`]s with the positive-winding
//! rule: a point is inside when its winding number is positive, so outer loops
//! run counter-clockwise and holes clockwise. Every arc is kept exact (centre,
//! sense) through booleans and offsets; discretisation happens only when a
//! consumer asks for chords at a tolerance.
//!
//! Robustness comes from one arrangement step shared by every operation
//! ([`arrange`]): all pairwise intersections are computed in closed form,
//! vertices within `point_eps` are merged, coincident pieces are grouped, and
//! classification then works on the pieces of that arrangement with exact
//! winding bookkeeping ([`ops`]). Arcs are canonicalised into quadrant pieces
//! (split at the axis extremes, sweep at most 90 degrees) so every piece is
//! monotone in both axes and a horizontal ray meets it at most once.
//!
//! Everything is deterministic: no hashing, every sort has an index tie-break.

use crate::model::Point2;
use crate::numeric::Tolerance;
use serde::{Deserialize, Serialize};
use std::f64::consts::{FRAC_PI_2, TAU};

pub mod arrange;
pub mod extract;
pub mod machining;
pub mod ops;

pub use ops::{boolean, chain_capsule, clip_line, offset_chain, offset_region, BoolOp, Side};

/// Why a planar operation refused its input or could not close its output.
#[derive(Clone, Debug, PartialEq)]
pub enum PlanarError {
    /// A coordinate was NaN or infinite.
    NonFinite { what: String },
    /// A segment or loop is degenerate (zero length, unequal arc radii, open loop, ...).
    Degenerate(String),
    /// The arrangement's kept pieces do not close into loops at `at`.
    OpenBoundary { at: Point2 },
    /// A user-facing refusal.
    InvalidInput(String),
    /// A result that contradicts an invariant of its operation (an offset
    /// that loses a loop it cannot reach, a boolean outside its area bounds).
    Inconsistent(String),
}

impl std::fmt::Display for PlanarError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlanarError::NonFinite { what } => write!(f, "planar: {what} is not finite"),
            PlanarError::Degenerate(m) => write!(f, "planar: degenerate input: {m}"),
            PlanarError::OpenBoundary { at } => {
                write!(f, "planar: the result boundary does not close at ({}, {})", at.x, at.y)
            }
            PlanarError::InvalidInput(m) => write!(f, "planar: {m}"),
            PlanarError::Inconsistent(m) => write!(f, "planar: inconsistent result: {m}"),
        }
    }
}

impl std::error::Error for PlanarError {}

impl From<PlanarError> for crate::CamError {
    fn from(e: PlanarError) -> Self {
        crate::CamError::InvalidInput(e.to_string())
    }
}

// ---------------------------------------------------------------------------
// 2D vector helpers (Point2 is C0's plain value type; the arithmetic lives here)
// ---------------------------------------------------------------------------

pub(crate) trait Vec2 {
    fn add(self, o: Point2) -> Point2;
    fn sub(self, o: Point2) -> Point2;
    fn scale(self, s: f64) -> Point2;
    fn dot(self, o: Point2) -> f64;
    fn cross(self, o: Point2) -> f64;
    fn length(self) -> f64;
    fn distance(self, o: Point2) -> f64;
    /// Rotated +90 degrees (the left normal of a direction).
    fn perp(self) -> Point2;
    fn normalized(self) -> Option<Point2>;
    fn angle(self) -> f64;
}

impl Vec2 for Point2 {
    fn add(self, o: Point2) -> Point2 {
        Point2::new(self.x + o.x, self.y + o.y)
    }
    fn sub(self, o: Point2) -> Point2 {
        Point2::new(self.x - o.x, self.y - o.y)
    }
    fn scale(self, s: f64) -> Point2 {
        Point2::new(self.x * s, self.y * s)
    }
    fn dot(self, o: Point2) -> f64 {
        self.x * o.x + self.y * o.y
    }
    fn cross(self, o: Point2) -> f64 {
        self.x * o.y - self.y * o.x
    }
    fn length(self) -> f64 {
        self.x.hypot(self.y)
    }
    fn distance(self, o: Point2) -> f64 {
        self.sub(o).length()
    }
    fn perp(self) -> Point2 {
        Point2::new(-self.y, self.x)
    }
    fn normalized(self) -> Option<Point2> {
        let l = self.length();
        if l > 0.0 && l.is_finite() {
            Some(self.scale(1.0 / l))
        } else {
            None
        }
    }
    fn angle(self) -> f64 {
        self.y.atan2(self.x)
    }
}

pub(crate) fn dir(a: f64) -> Point2 {
    Point2::new(a.cos(), a.sin())
}

// ---------------------------------------------------------------------------
// Segments
// ---------------------------------------------------------------------------

/// The circular part of an arc segment: its centre and sense.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Arc {
    pub center: Point2,
    /// Counter-clockwise from the segment's start to its end.
    pub ccw: bool,
}

/// A directed line or arc. An arc runs from `start` to `end` about
/// `arc.center` in the sense `arc.ccw`; full circles are two arcs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Segment {
    pub start: Point2,
    pub end: Point2,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub arc: Option<Arc>,
}

/// Where two segments meet.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Hit {
    pub point: Point2,
    /// Parameter on the first segment, `0..=1`.
    pub ta: f64,
    /// Parameter on the second segment, `0..=1`.
    pub tb: f64,
}

impl Segment {
    pub fn line(start: Point2, end: Point2) -> Self {
        Self { start, end, arc: None }
    }

    pub fn arc(start: Point2, end: Point2, center: Point2, ccw: bool) -> Self {
        Self { start, end, arc: Some(Arc { center, ccw }) }
    }

    pub fn is_arc(&self) -> bool {
        self.arc.is_some()
    }

    pub fn reversed(&self) -> Self {
        Self { start: self.end, end: self.start, arc: self.arc.map(|a| Arc { center: a.center, ccw: !a.ccw }) }
    }

    /// Radius of an arc (from its start), `0` for a line.
    pub fn radius(&self) -> f64 {
        self.arc.map_or(0.0, |a| self.start.distance(a.center))
    }

    /// Start angle and SIGNED sweep (positive counter-clockwise) of an arc.
    /// For a line both are zero.
    pub fn angles(&self) -> (f64, f64) {
        match self.arc {
            None => (0.0, 0.0),
            Some(a) => {
                let a0 = self.start.sub(a.center).angle();
                let a1 = self.end.sub(a.center).angle();
                let ccw = (a1 - a0).rem_euclid(TAU);
                if a.ccw {
                    (a0, ccw)
                } else {
                    (a0, ccw - TAU)
                }
            }
        }
    }

    /// Arc length (chord length for a line).
    pub fn length(&self) -> f64 {
        match self.arc {
            None => self.start.distance(self.end),
            Some(_) => self.radius() * self.angles().1.abs(),
        }
    }

    pub fn point_at(&self, t: f64) -> Point2 {
        let t = t.clamp(0.0, 1.0);
        if t == 0.0 {
            return self.start;
        }
        if t == 1.0 {
            return self.end;
        }
        match self.arc {
            None => self.start.add(self.end.sub(self.start).scale(t)),
            Some(a) => {
                let (a0, sweep) = self.angles();
                a.center.add(dir(a0 + sweep * t).scale(self.radius()))
            }
        }
    }

    /// Unit tangent in the direction of travel at parameter `t`.
    pub fn tangent_at(&self, t: f64) -> Point2 {
        match self.arc {
            None => self.end.sub(self.start).normalized().unwrap_or(Point2::new(1.0, 0.0)),
            Some(a) => {
                let (a0, sweep) = self.angles();
                let radial = dir(a0 + sweep * t.clamp(0.0, 1.0));
                if a.ccw {
                    radial.perp()
                } else {
                    radial.perp().scale(-1.0)
                }
            }
        }
    }

    pub fn midpoint(&self) -> Point2 {
        self.point_at(0.5)
    }

    /// Axis-aligned bounds `(min, max)`.
    pub fn bounds(&self) -> (Point2, Point2) {
        let mut lo = Point2::new(self.start.x.min(self.end.x), self.start.y.min(self.end.y));
        let mut hi = Point2::new(self.start.x.max(self.end.x), self.start.y.max(self.end.y));
        if let Some(a) = self.arc {
            let r = self.radius();
            let (a0, sweep) = self.angles();
            let (lo_a, hi_a) = if sweep >= 0.0 { (a0, a0 + sweep) } else { (a0 + sweep, a0) };
            // Every axis extreme angle inside the sweep extends the box.
            let k0 = (lo_a / FRAC_PI_2).floor() as i64;
            for k in k0..=k0 + 5 {
                let ang = k as f64 * FRAC_PI_2;
                if ang > lo_a && ang < hi_a {
                    let p = a.center.add(dir(ang).scale(r));
                    lo = Point2::new(lo.x.min(p.x), lo.y.min(p.y));
                    hi = Point2::new(hi.x.max(p.x), hi.y.max(p.y));
                }
            }
        }
        (lo, hi)
    }

    /// The part of the segment between parameters `t0 < t1`, with the given
    /// exact endpoint coordinates (so split pieces share vertices bit for bit).
    pub fn sub_segment(&self, start: Point2, end: Point2) -> Segment {
        Segment { start, end, arc: self.arc }
    }

    /// Parameter of the nearest point on the supporting curve to `p`, clamped
    /// to the segment, and that point.
    pub fn nearest(&self, p: Point2) -> (Point2, f64) {
        match self.arc {
            None => {
                let d = self.end.sub(self.start);
                let l2 = d.dot(d);
                let t = if l2 > 0.0 { (p.sub(self.start).dot(d) / l2).clamp(0.0, 1.0) } else { 0.0 };
                (self.point_at(t), t)
            }
            Some(a) => {
                let (a0, sweep) = self.angles();
                let v = p.sub(a.center);
                if v.length() == 0.0 {
                    return (self.start, 0.0);
                }
                let ang = v.angle();
                // Fraction of the sweep at which the radial direction lies.
                let rel = if sweep >= 0.0 { (ang - a0).rem_euclid(TAU) } else { -((a0 - ang).rem_euclid(TAU)) };
                let t = rel / sweep;
                if t.is_finite() && (0.0..=1.0).contains(&t) {
                    (a.center.add(dir(ang).scale(self.radius())), t)
                } else if p.distance(self.start) <= p.distance(self.end) {
                    (self.start, 0.0)
                } else {
                    (self.end, 1.0)
                }
            }
        }
    }

    pub fn distance(&self, p: Point2) -> f64 {
        self.nearest(p).0.distance(p)
    }

    /// Signed area between the chord and the arc (positive for a
    /// counter-clockwise arc, which bulges to the right of its chord).
    pub fn bulge_area(&self) -> f64 {
        match self.arc {
            None => 0.0,
            Some(_) => {
                let (_, s) = self.angles();
                let r = self.radius();
                0.5 * r * r * (s - s.sin())
            }
        }
    }

    /// Chord points from `start` to `end` with sagitta at most `tol`.
    pub fn discretize(&self, tol: f64) -> Vec<Point2> {
        match self.arc {
            None => vec![self.start, self.end],
            Some(_) => {
                let r = self.radius();
                let (_, sweep) = self.angles();
                let max_step = if tol >= r || tol <= 0.0 { FRAC_PI_2 } else { 2.0 * (1.0 - tol / r).acos() };
                let n = ((sweep.abs() / max_step.max(1e-9)).ceil() as usize).clamp(1, 100_000);
                let mut pts = Vec::with_capacity(n + 1);
                pts.push(self.start);
                for i in 1..n {
                    pts.push(self.point_at(i as f64 / n as f64));
                }
                pts.push(self.end);
                pts
            }
        }
    }

    /// Chord points whose chords never come closer to the arc's centre than
    /// the arc itself: the circumscribed polygon. Its vertices sit at the
    /// half-step angles at `radius / cos(step / 2)`, so every chord lies on
    /// a tangent line of the arc (the first and last chords on the tangents
    /// at the arc's ends) and the polyline deviates outward by at most
    /// `tol`. Lines are unchanged. Used for tool-centre arcs whose centre
    /// side holds material.
    pub fn discretize_outward(&self, tol: f64) -> Vec<Point2> {
        let Some(a) = self.arc else { return vec![self.start, self.end] };
        let r = self.radius();
        let (a0, sweep) = self.angles();
        // 1/cos(s/2) - 1 <= tol/r bounds the outward deviation.
        let max_step = if tol >= r || tol <= 0.0 { FRAC_PI_2 } else { 2.0 * (r / (r + tol)).acos() };
        let n = ((sweep.abs() / max_step.max(1e-9)).ceil() as usize).clamp(1, 100_000);
        let step = sweep / n as f64;
        let outer = r / (0.5 * step).cos();
        let mut pts = Vec::with_capacity(n + 2);
        pts.push(self.start);
        for i in 0..n {
            pts.push(a.center.add(dir(a0 + step * (i as f64 + 0.5)).scale(outer)));
        }
        pts.push(self.end);
        pts
    }

    pub fn validate(&self, tol: &Tolerance) -> Result<(), PlanarError> {
        if !self.start.is_finite() || !self.end.is_finite() {
            return Err(PlanarError::NonFinite { what: "segment endpoint".into() });
        }
        if self.start.distance(self.end) <= tol.point_eps {
            return Err(PlanarError::Degenerate(format!("segment of zero length at ({}, {})", self.start.x, self.start.y)));
        }
        if let Some(a) = self.arc {
            if !a.center.is_finite() {
                return Err(PlanarError::NonFinite { what: "arc centre".into() });
            }
            let r0 = self.start.distance(a.center);
            let r1 = self.end.distance(a.center);
            if r0 <= tol.sample_eps {
                return Err(PlanarError::Degenerate("arc radius is below the sample epsilon".into()));
            }
            // Each end vertex may sit up to a point epsilon off the circle
            // (the arrangement cuts arcs between merged vertices), so the end
            // radii of an arc the engine itself produced differ by up to two;
            // more than that is a malformed arc.
            if (r0 - r1).abs() > (2.0 * tol.point_eps).max(tol.distance_eps).max(1e-9 * r0) {
                return Err(PlanarError::Degenerate(format!("arc endpoints have radii {r0} and {r1}")));
            }
        }
        Ok(())
    }

    /// The canonical pieces: lines unchanged, arcs split at every axis
    /// extreme so each piece lies in one quadrant about its centre.
    pub fn canonical(&self, tol: &Tolerance) -> Vec<Segment> {
        let Some(a) = self.arc else { return vec![*self] };
        let r = self.radius();
        let (a0, sweep) = self.angles();
        let (lo, hi) = if sweep >= 0.0 { (a0, a0 + sweep) } else { (a0 + sweep, a0) };
        let mut cuts: Vec<f64> = Vec::new();
        let k0 = (lo / FRAC_PI_2).floor() as i64;
        for k in k0..=k0 + 5 {
            let ang = k as f64 * FRAC_PI_2;
            // Skip cuts that would leave a piece shorter than the point epsilon.
            if ang - lo > tol.point_eps / r && hi - ang > tol.point_eps / r {
                cuts.push(ang);
            }
        }
        if cuts.is_empty() {
            return vec![*self];
        }
        if sweep < 0.0 {
            cuts.reverse();
        }
        let mut out = Vec::with_capacity(cuts.len() + 1);
        let mut from = self.start;
        for c in cuts {
            let p = a.center.add(dir(c).scale(r));
            out.push(Segment::arc(from, p, a.center, a.ccw));
            from = p;
        }
        out.push(Segment::arc(from, self.end, a.center, a.ccw));
        out
    }

    /// Closed-form intersections of two segments, including the endpoints of
    /// overlapping collinear / co-circular pairs (each endpoint of one that
    /// lies on the other is a hit, so both get split there).
    pub fn intersect(&self, other: &Segment, tol: &Tolerance) -> Vec<Hit> {
        let eps = tol.distance_eps;
        let mut hits = Vec::new();
        // Quick reject on bounds.
        let (alo, ahi) = self.bounds();
        let (blo, bhi) = other.bounds();
        if alo.x > bhi.x + eps || blo.x > ahi.x + eps || alo.y > bhi.y + eps || blo.y > ahi.y + eps {
            return hits;
        }
        match (self.arc, other.arc) {
            (None, None) => {
                let d1 = self.end.sub(self.start);
                let d2 = other.end.sub(other.start);
                let denom = d1.cross(d2);
                let w = other.start.sub(self.start);
                let l1 = d1.length();
                let l2 = d2.length();
                if denom.abs() <= tol.angle_eps * l1 * l2 {
                    // Parallel: overlapping only when collinear.
                    if w.cross(d1).abs() <= eps * l1 {
                        overlap_hits(self, other, &mut hits);
                    }
                    return hits;
                }
                let ta = w.cross(d2) / denom;
                let tb = w.cross(d1) / denom;
                let slack_a = eps / l1;
                let slack_b = eps / l2;
                if (-slack_a..=1.0 + slack_a).contains(&ta) && (-slack_b..=1.0 + slack_b).contains(&tb) {
                    let p = self.start.add(d1.scale(ta.clamp(0.0, 1.0)));
                    hits.push(Hit { point: p, ta: ta.clamp(0.0, 1.0), tb: tb.clamp(0.0, 1.0) });
                }
            }
            (None, Some(_)) => {
                for h in other.intersect(self, tol) {
                    hits.push(Hit { point: h.point, ta: h.tb, tb: h.ta });
                }
            }
            (Some(a), None) => {
                // Endpoints lying on the other curve are exact hits; roots
                // within the tangency uncertainty of one are its numerical
                // shadow (a line through a point of a circle that meets the
                // circle again nearby is nearly tangent) and are dropped.
                let r = self.radius();
                self.incidences(other, tol, &mut hits);
                let shadow = 2.0 * (2.0 * r * eps).sqrt();
                // |s + t d - c|^2 = r^2
                let d = other.end.sub(other.start);
                let f = other.start.sub(a.center);
                let qa = d.dot(d);
                let qb = 2.0 * f.dot(d);
                let qc = f.dot(f) - r * r;
                let disc = qb * qb - 4.0 * qa * qc;
                let l = d.length();
                // Tangency within tolerance counts as one touch.
                let tangent_slack = 4.0 * qa * (2.0 * r * eps);
                if disc < -tangent_slack {
                    return hits;
                }
                let roots: Vec<f64> = if disc <= tangent_slack {
                    vec![-qb / (2.0 * qa)]
                } else {
                    let s = disc.sqrt();
                    vec![(-qb - s) / (2.0 * qa), (-qb + s) / (2.0 * qa)]
                };
                let known: Vec<Point2> = hits.iter().map(|h| h.point).collect();
                for tb in roots {
                    let slack = eps / l;
                    if !(-slack..=1.0 + slack).contains(&tb) {
                        continue;
                    }
                    let p = other.start.add(d.scale(tb.clamp(0.0, 1.0)));
                    if known.iter().any(|k| k.distance(p) <= shadow) {
                        continue;
                    }
                    if let Some(ta) = self.arc_param(p, tol) {
                        hits.push(Hit { point: p, ta, tb: tb.clamp(0.0, 1.0) });
                    }
                }
            }
            (Some(a), Some(b)) => {
                let r0 = self.radius();
                let r1 = other.radius();
                let cd = b.center.sub(a.center);
                let dist = cd.length();
                if dist <= tol.point_eps && (r0 - r1).abs() <= eps {
                    overlap_hits(self, other, &mut hits);
                    return hits;
                }
                if dist <= tol.point_eps {
                    return hits; // concentric, different radii
                }
                if dist > r0 + r1 + eps || dist < (r0 - r1).abs() - eps {
                    return hits;
                }
                self.incidences(other, tol, &mut hits);
                let shadow = 2.0 * (2.0 * r0.max(r1) * eps).sqrt();
                let known: Vec<Point2> = hits.iter().map(|h| h.point).collect();
                let x = (dist * dist + r0 * r0 - r1 * r1) / (2.0 * dist);
                let h2 = r0 * r0 - x * x;
                let u = cd.scale(1.0 / dist);
                let base = a.center.add(u.scale(x));
                // Tangency within tolerance counts as one touch: the circles'
                // gap (outside each other or one inside the other) is at most
                // the distance epsilon. Deciding it on `h2` against `2 r0 eps`
                // took circles overlapping by up to about three epsilons for
                // tangent (`h2` is `2 g r0 r1 / (r0 + r1)` for an overlap `g`),
                // and the single touch point between two real crossings can
                // lie off both arcs: a 0.69 and a 0.35 arc overlapping by
                // 2.3e-6 lost the crossing one of them ends past, and the
                // union left a piece without continuation (sweep 51 run 1).
                let gap = (r0 + r1 - dist).min(dist - (r0 - r1).abs());
                let candidates: Vec<Point2> = if gap <= eps {
                    vec![base]
                } else {
                    let h = h2.max(0.0).sqrt();
                    vec![base.sub(u.perp().scale(h)), base.add(u.perp().scale(h))]
                };
                for p in candidates {
                    if known.iter().any(|k| k.distance(p) <= shadow) {
                        continue;
                    }
                    if let (Some(ta), Some(tb)) = (self.arc_param(p, tol), other.arc_param(p, tol)) {
                        hits.push(Hit { point: p, ta, tb });
                    }
                }
            }
        }
        hits
    }

    /// Hits at endpoints of either segment that lie on the other segment
    /// (within the distance epsilon), with exact parameters 0 or 1 on the
    /// owning side.
    fn incidences(&self, other: &Segment, tol: &Tolerance, hits: &mut Vec<Hit>) {
        let eps = tol.distance_eps;
        for (tb, p) in [(0.0, other.start), (1.0, other.end)] {
            let (q, ta) = self.nearest(p);
            if q.distance(p) <= eps && !hits.iter().any(|h| h.point.distance(p) <= tol.point_eps) {
                hits.push(Hit { point: p, ta, tb });
            }
        }
        for (ta, p) in [(0.0, self.start), (1.0, self.end)] {
            let (q, tb) = other.nearest(p);
            if q.distance(p) <= eps && !hits.iter().any(|h| h.point.distance(p) <= tol.point_eps) {
                hits.push(Hit { point: p, ta, tb });
            }
        }
    }

    /// Parameter of a point known to lie on the arc's circle, when it lies
    /// within the arc's sweep (with endpoint slack of the distance epsilon).
    fn arc_param(&self, p: Point2, tol: &Tolerance) -> Option<f64> {
        let a = self.arc?;
        let (a0, sweep) = self.angles();
        let r = self.radius();
        let ang = p.sub(a.center).angle();
        let rel = if sweep >= 0.0 { (ang - a0).rem_euclid(TAU) } else { -((a0 - ang).rem_euclid(TAU)) };
        let slack = tol.distance_eps / r;
        let t = rel / sweep;
        if (0.0..=1.0).contains(&t) {
            return Some(t);
        }
        // Just before the start (wrapped around) or just past the end.
        if (rel.abs() - TAU).abs() <= slack || rel.abs() <= slack {
            return Some(0.0);
        }
        if (rel.abs() - sweep.abs()).abs() <= slack {
            return Some(1.0);
        }
        None
    }
}

/// Endpoint hits for overlapping segments on the same line or circle.
fn overlap_hits(a: &Segment, b: &Segment, hits: &mut Vec<Hit>) {
    for (tb, p) in [(0.0, b.start), (1.0, b.end)] {
        let (q, ta) = a.nearest(p);
        if q.distance(p) <= 1e-9 * (1.0 + a.length()) {
            hits.push(Hit { point: p, ta, tb });
        }
    }
    for (ta, p) in [(0.0, a.start), (1.0, a.end)] {
        let (q, tb) = b.nearest(p);
        if q.distance(p) <= 1e-9 * (1.0 + b.length()) {
            hits.push(Hit { point: p, ta, tb });
        }
    }
}

// ---------------------------------------------------------------------------
// Loops, chains and regions
// ---------------------------------------------------------------------------

/// A closed, directed sequence of segments (each ends where the next starts).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Loop {
    pub segments: Vec<Segment>,
}

impl Loop {
    pub fn new(segments: Vec<Segment>) -> Self {
        Self { segments }
    }

    /// A closed polygon through `points` (last joins back to first).
    pub fn polygon(points: &[Point2]) -> Self {
        let n = points.len();
        Self::new((0..n).map(|i| Segment::line(points[i], points[(i + 1) % n])).collect())
    }

    /// An axis-aligned rectangle, counter-clockwise.
    pub fn rectangle(min: Point2, max: Point2) -> Self {
        Self::polygon(&[min, Point2::new(max.x, min.y), max, Point2::new(min.x, max.y)])
    }

    /// A full circle as four quadrant arcs, counter-clockwise when `ccw`.
    pub fn circle(center: Point2, radius: f64, ccw: bool) -> Self {
        let p = |k: usize| center.add(dir(k as f64 * FRAC_PI_2).scale(radius));
        let mut segs: Vec<Segment> = (0..4).map(|k| Segment::arc(p(k), p(k + 1), center, true)).collect();
        if !ccw {
            segs.reverse();
            for s in &mut segs {
                *s = s.reversed();
            }
        }
        Self::new(segs)
    }

    /// A stadium (slot) around the centre line `a`-`b` with the given
    /// radius, counter-clockwise.
    pub fn stadium(a: Point2, b: Point2, radius: f64) -> Self {
        let u = b.sub(a).normalized().unwrap_or(Point2::new(1.0, 0.0));
        let n = u.perp().scale(radius);
        let mut segs = vec![Segment::line(a.sub(n), b.sub(n))];
        segs.extend(Segment::arc(b.sub(n), b.add(n), b, true).canonical(&Tolerance::default()));
        segs.push(Segment::line(b.add(n), a.add(n)));
        segs.extend(Segment::arc(a.add(n), a.sub(n), a, true).canonical(&Tolerance::default()));
        Self::new(segs)
    }

    pub fn reversed(&self) -> Self {
        Self::new(self.segments.iter().rev().map(Segment::reversed).collect())
    }

    /// Shoelace area of the chords plus the arc bulges; positive counter-clockwise.
    pub fn signed_area(&self) -> f64 {
        let mut a = 0.0;
        for s in &self.segments {
            a += 0.5 * s.start.cross(s.end) + s.bulge_area();
        }
        a
    }

    pub fn is_ccw(&self) -> bool {
        self.signed_area() > 0.0
    }

    pub fn length(&self) -> f64 {
        self.segments.iter().map(Segment::length).sum()
    }

    pub fn bounds(&self) -> (Point2, Point2) {
        bounds_of(self.segments.iter())
    }

    /// Chord points around the loop (first point not repeated at the end).
    pub fn discretize(&self, tol: f64) -> Vec<Point2> {
        let mut out = Vec::new();
        for s in &self.segments {
            let pts = s.discretize(tol);
            out.extend_from_slice(&pts[..pts.len() - 1]);
        }
        out
    }

    /// Winding number of `p` about this loop alone.
    pub fn winding(&self, p: Point2, tol: &Tolerance) -> i32 {
        let canon: Vec<Segment> = self.segments.iter().flat_map(|s| s.canonical(tol)).collect();
        crossings(canon.iter(), p, true)
    }

    /// Closed (consecutive endpoints coincide within `point_eps`), every
    /// segment valid, at least two segments. Snaps consecutive endpoints to
    /// the same coordinates.
    pub fn validate(&mut self, tol: &Tolerance) -> Result<(), PlanarError> {
        if self.segments.len() < 2 {
            return Err(PlanarError::Degenerate("a loop needs at least two segments".into()));
        }
        for s in &self.segments {
            s.validate(tol)?;
        }
        let n = self.segments.len();
        for i in 0..n {
            let a = self.segments[i].end;
            let b = self.segments[(i + 1) % n].start;
            if a.distance(b) > tol.point_eps {
                return Err(PlanarError::Degenerate(format!(
                    "loop is open between ({}, {}) and ({}, {})",
                    a.x, a.y, b.x, b.y
                )));
            }
            self.segments[(i + 1) % n].start = a;
        }
        Ok(())
    }

    /// The loop as canonical pieces (arcs split per quadrant).
    pub fn canonical(&self, tol: &Tolerance) -> Loop {
        Loop::new(self.segments.iter().flat_map(|s| s.canonical(tol)).collect())
    }
}

/// An open or closed directed curve (edge chains, trace paths).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Chain {
    pub segments: Vec<Segment>,
    pub closed: bool,
}

impl Chain {
    pub fn open(segments: Vec<Segment>) -> Self {
        Self { segments, closed: false }
    }

    pub fn from_loop(l: &Loop) -> Self {
        Self { segments: l.segments.clone(), closed: true }
    }

    pub fn length(&self) -> f64 {
        self.segments.iter().map(Segment::length).sum()
    }

    pub fn start(&self) -> Option<Point2> {
        self.segments.first().map(|s| s.start)
    }

    pub fn end(&self) -> Option<Point2> {
        self.segments.last().map(|s| s.end)
    }

    pub fn reversed(&self) -> Self {
        Self { segments: self.segments.iter().rev().map(Segment::reversed).collect(), closed: self.closed }
    }

    /// Chord points along the chain; a closed chain repeats its first point last.
    pub fn discretize(&self, tol: f64) -> Vec<Point2> {
        let mut out = Vec::new();
        for (i, s) in self.segments.iter().enumerate() {
            let pts = s.discretize(tol);
            if i == 0 {
                out.push(pts[0]);
            }
            out.extend_from_slice(&pts[1..]);
        }
        out
    }

    pub fn validate(&mut self, tol: &Tolerance) -> Result<(), PlanarError> {
        if self.segments.is_empty() {
            return Err(PlanarError::Degenerate("a chain needs at least one segment".into()));
        }
        for s in &self.segments {
            s.validate(tol)?;
        }
        let n = self.segments.len();
        let last = if self.closed { n } else { n - 1 };
        for i in 0..last {
            let a = self.segments[i].end;
            let b = self.segments[(i + 1) % n].start;
            if a.distance(b) > tol.point_eps {
                return Err(PlanarError::Degenerate(format!("chain is broken between ({}, {}) and ({}, {})", a.x, a.y, b.x, b.y)));
            }
            self.segments[(i + 1) % n].start = a;
        }
        Ok(())
    }
}

/// One outer loop with its holes, from [`Region::shells`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Shell {
    pub outer: Loop,
    pub holes: Vec<Loop>,
}

impl Shell {
    pub fn region(&self) -> Region {
        let mut loops = vec![self.outer.clone()];
        loops.extend(self.holes.iter().cloned());
        Region { loops }
    }
}

/// A planar region: oriented loops under the positive-winding rule.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Region {
    pub loops: Vec<Loop>,
}

impl Region {
    /// A region from already oriented loops (outer CCW, holes CW), validated.
    pub fn new(loops: Vec<Loop>, tol: &Tolerance) -> Result<Self, PlanarError> {
        let mut loops = loops;
        for l in &mut loops {
            l.validate(tol)?;
        }
        Ok(Self { loops })
    }

    /// A region from loops of unknown orientation, assuming they do not cross:
    /// each loop is oriented by its nesting depth (even depth outer, odd hole).
    pub fn from_unoriented(loops: Vec<Loop>, tol: &Tolerance) -> Result<Self, PlanarError> {
        let mut loops = loops;
        for l in &mut loops {
            l.validate(tol)?;
        }
        let canon: Vec<Loop> = loops.iter().map(|l| l.canonical(tol)).collect();
        let mut out = Vec::with_capacity(loops.len());
        for (i, l) in loops.iter().enumerate() {
            let probe = l.segments[0].midpoint();
            let depth = canon
                .iter()
                .enumerate()
                .filter(|(j, _)| *j != i)
                .filter(|(_, other)| crossings(other.segments.iter(), probe, true) != 0)
                .count();
            let want_ccw = depth % 2 == 0;
            out.push(if l.is_ccw() == want_ccw { l.clone() } else { l.reversed() });
        }
        Ok(Self { loops: out })
    }

    pub fn empty() -> Self {
        Self { loops: Vec::new() }
    }

    pub fn is_empty(&self) -> bool {
        self.loops.is_empty()
    }

    pub fn area(&self) -> f64 {
        self.loops.iter().map(Loop::signed_area).sum()
    }

    pub fn bounds(&self) -> (Point2, Point2) {
        bounds_of(self.loops.iter().flat_map(|l| l.segments.iter()))
    }

    /// Every segment of every loop in canonical form.
    pub fn canonical_segments(&self, tol: &Tolerance) -> Vec<Segment> {
        self.loops.iter().flat_map(|l| l.segments.iter().flat_map(|s| s.canonical(tol))).collect()
    }

    /// Winding number of `p`; positive inside.
    pub fn winding(&self, p: Point2, tol: &Tolerance) -> i32 {
        let canon = self.canonical_segments(tol);
        crossings(canon.iter(), p, true)
    }

    pub fn contains(&self, p: Point2, tol: &Tolerance) -> bool {
        self.winding(p, tol) > 0
    }

    /// Nearest boundary point to `p` and the distance to it.
    pub fn nearest_boundary(&self, p: Point2) -> Option<(Point2, f64)> {
        let mut best: Option<(Point2, f64)> = None;
        for s in self.loops.iter().flat_map(|l| l.segments.iter()) {
            let (q, _) = s.nearest(p);
            let d = q.distance(p);
            if best.map_or(true, |(_, bd)| d < bd) {
                best = Some((q, d));
            }
        }
        best
    }

    /// Signed distance: positive inside, negative outside, zero on the boundary.
    pub fn signed_distance(&self, p: Point2, tol: &Tolerance) -> f64 {
        let d = self.nearest_boundary(p).map_or(f64::INFINITY, |(_, d)| d);
        if self.contains(p, tol) {
            d
        } else {
            -d
        }
    }

    /// Group loops into outer loops with their holes. Loops are assumed not to
    /// cross (true for every operation output). An island inside a hole is its
    /// own shell.
    pub fn shells(&self, tol: &Tolerance) -> Vec<Shell> {
        let outers: Vec<usize> = (0..self.loops.len()).filter(|&i| self.loops[i].is_ccw()).collect();
        let canon: Vec<Loop> = self.loops.iter().map(|l| l.canonical(tol)).collect();
        let mut shells: Vec<Shell> = outers.iter().map(|&i| Shell { outer: self.loops[i].clone(), holes: Vec::new() }).collect();
        for l in &self.loops {
            if l.is_ccw() {
                continue;
            }
            let probe = l.segments[0].midpoint();
            // The smallest outer containing the hole is its parent.
            let mut parent: Option<(usize, f64)> = None;
            for (k, &o) in outers.iter().enumerate() {
                if crossings(canon[o].segments.iter(), probe, true) != 0 {
                    let a = self.loops[o].signed_area();
                    if parent.map_or(true, |(_, pa)| a < pa) {
                        parent = Some((k, a));
                    }
                }
            }
            if let Some((k, _)) = parent {
                shells[k].holes.push(l.clone());
            }
        }
        shells
    }

    /// Offset by `distance`: positive grows the region, negative shrinks it.
    pub fn offset(&self, distance: f64, tol: &Tolerance) -> Result<Region, PlanarError> {
        offset_region(self, distance, tol)
    }

    pub fn union(&self, other: &Region, tol: &Tolerance) -> Result<Region, PlanarError> {
        boolean(self, other, BoolOp::Union, tol)
    }

    pub fn difference(&self, other: &Region, tol: &Tolerance) -> Result<Region, PlanarError> {
        boolean(self, other, BoolOp::Difference, tol)
    }

    pub fn intersection(&self, other: &Region, tol: &Tolerance) -> Result<Region, PlanarError> {
        boolean(self, other, BoolOp::Intersection, tol)
    }

    /// Chord loops at `tol`, for consumers that take polylines
    /// (e.g. [`crate::model::Containment::boundary`]).
    pub fn to_polylines(&self, tol: f64) -> Vec<Vec<Point2>> {
        self.loops.iter().map(|l| l.discretize(tol)).collect()
    }
}

/// A loop with collinear consecutive lines merged (co-circular consecutive
/// arcs too), so raw offset curves carry no degenerate vertex circles.
pub(crate) fn simplified(l: &Loop, tol: &Tolerance) -> Loop {
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
    let mut out: Vec<Segment> = Vec::with_capacity(l.segments.len());
    for s in &l.segments {
        if let Some(last) = out.last_mut() {
            if same_support(last, s) && s.end.distance(last.start) > tol.point_eps {
                let merged = Segment { start: last.start, end: s.end, arc: last.arc };
                if merged.arc.is_none() || (merged.length() >= last.length() + s.length() - tol.distance_eps && merged.length() <= std::f64::consts::PI * merged.radius() + tol.distance_eps) {
                    *last = merged;
                    continue;
                }
            }
        }
        out.push(*s);
    }
    if out.len() > 2 {
        let n = out.len();
        if same_support(&out[n - 1], &out[0]) && out[n - 1].start.distance(out[0].end) > tol.point_eps {
            let last = out[n - 1];
            let merged = Segment { start: last.start, end: out[0].end, arc: last.arc };
            if merged.arc.is_none() || (merged.length() >= last.length() + out[0].length() - tol.distance_eps && merged.length() <= std::f64::consts::PI * merged.radius() + tol.distance_eps) {
                out.pop();
                out[0] = merged;
            }
        }
    }
    Loop::new(out)
}

pub(crate) fn bounds_of<'a>(segments: impl Iterator<Item = &'a Segment>) -> (Point2, Point2) {
    let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for s in segments {
        let (a, b) = s.bounds();
        lo = Point2::new(lo.x.min(a.x), lo.y.min(a.y));
        hi = Point2::new(hi.x.max(b.x), hi.y.max(b.y));
    }
    (lo, hi)
}

/// Signed crossings of the horizontal ray from `p` towards +X with CANONICAL
/// segments (lines, or arcs within one quadrant about their centre, so each
/// crosses a horizontal line at most once). `upper` selects the half-open
/// convention: `true` counts a segment when `y_low <= p.y < y_high` (the ray
/// taken infinitesimally above `p`), `false` when `y_low < p.y <= y_high`.
/// Upward segments count +1, downward -1. The result is the winding number
/// of `p` about the loops the segments form, for any `p` off the boundary.
pub(crate) fn crossings<'a>(segments: impl Iterator<Item = &'a Segment>, p: Point2, upper: bool) -> i32 {
    let mut w = 0;
    for s in segments {
        let (ylo, yhi) = if s.start.y <= s.end.y { (s.start.y, s.end.y) } else { (s.end.y, s.start.y) };
        let inside = if upper { ylo <= p.y && p.y < yhi } else { ylo < p.y && p.y <= yhi };
        if !inside {
            continue;
        }
        let x = match s.arc {
            None => s.start.x + (p.y - s.start.y) * (s.end.x - s.start.x) / (s.end.y - s.start.y),
            Some(a) => {
                let r = s.radius();
                let dy = p.y - a.center.y;
                let h = (r * r - dy * dy).max(0.0).sqrt();
                if s.midpoint().x >= a.center.x {
                    a.center.x + h
                } else {
                    a.center.x - h
                }
            }
        };
        if x > p.x {
            w += if s.end.y > s.start.y { 1 } else { -1 };
        }
    }
    w
}

