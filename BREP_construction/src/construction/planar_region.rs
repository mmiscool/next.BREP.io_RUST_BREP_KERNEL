//! The exact region a polygon covers under a planar rigid motion (turns about
//! axes along the prism's own direction), extruded: the sweep of a straight-
//! sided prism within a clip, built from its boundary curves rather than read
//! level by level (behind `BREP_SWEEP_PLANAR_REGION`, default on; `=0` restores the level engine).
//!
//! In the plane square to the turns the boundary of the region lies on
//! - the paths of the polygon's corners;
//! - the envelopes of its sides: at each time the point of a side whose
//!   velocity runs along it (the foot of the perpendicular from the
//!   instantaneous centre), where that point lies on the side;
//! - the sides at the two end positions (none when the motion ends where it
//!   began, as a set);
//! - the clip's circles (and its angle's rays).
//!
//! These curves are cut at their crossings (found on samples, then solved
//! on the exact curves), and an arc is kept where exact membership differs on
//! its two sides; membership walks the motion's time by the polygon's signed
//! distance over a bound on the preimage's speed. The kept arcs close into
//! loops, which are extruded over the clip's axial range.
use crate::solid_sweep::{OutlinePiece, ScrewStep, SolidMotion, SweepClip, SweptTool};
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{make_line, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};
use std::f64::consts::TAU;

/// The switch (default on; `BREP_SWEEP_PLANAR_REGION=0` restores the level engine).
pub fn planar_region_on() -> bool {
    crate::switch!("BREP_SWEEP_PLANAR_REGION", true)
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("sweepSolidWithin: {message}"))
}

fn frag<T>(result: Result<T, String>, what: &'static str) -> Result<T, KernelRefusal> {
    result.or_refuse(KernelStage::Fragment, what)
}

type P2 = [f64; 2];

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P2, b: P2) -> P2 {
    [a[0] + b[0], a[1] + b[1]]
}
fn mul(a: P2, s: f64) -> P2 {
    [a[0] * s, a[1] * s]
}
fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}
fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn norm(a: P2) -> f64 {
    a[0].hypot(a[1])
}
fn rot(a: P2, angle: f64) -> P2 {
    let (s, c) = angle.sin_cos();
    [c * a[0] - s * a[1], s * a[0] + c * a[1]]
}

/// The motion in the plane: steps applied in order, each a turn (centre,
/// rate) and a slide (velocity) beside it, `slides[k]` with `turns[k]` (a
/// turn's slide zero, a slide's turn of rate zero).
struct Motion {
    turns: Vec<(P2, f64)>,
    slides: Vec<P2>,
    t0: f64,
    t1: f64,
}

impl Motion {
    /// The pose at t: `x ↦ R(θ) x + b`, as (θ, b), and their rates.
    fn pose(&self, t: f64) -> (f64, P2, f64, P2) {
        let (mut theta, mut b, mut dtheta, mut db) = (0.0, [0.0, 0.0], 0.0, [0.0, 0.0]);
        for (k, &(c, w)) in self.turns.iter().enumerate() {
            // b ← R(wt)(b − c) + c; db ← R(wt) db + w J R(wt)(b − c)
            let rel = rot(sub(b, c), w * t);
            db = add(rot(db, w * t), mul([-rel[1], rel[0]], w));
            b = add(rel, c);
            theta += w * t;
            dtheta += w;
            // A slide: b ← b + s t; db ← db + s.
            let slide = self.slides[k];
            if slide != [0.0, 0.0] {
                b = add(b, mul(slide, t));
                db = add(db, slide);
            }
        }
        (theta, b, dtheta, db)
    }
    fn apply(&self, t: f64, x: P2) -> P2 {
        let (theta, b, _, _) = self.pose(t);
        add(rot(x, theta), b)
    }
    fn invert(&self, t: f64, y: P2) -> P2 {
        let (theta, b, _, _) = self.pose(t);
        rot(sub(y, b), -theta)
    }
    /// The velocity of the material point at world `y` at time t.
    fn velocity(&self, t: f64, y: P2) -> P2 {
        let (_, b, dtheta, db) = self.pose(t);
        let r = sub(y, b);
        add(mul([-r[1], r[0]], dtheta), db)
    }
    /// A bound on the preimage's acceleration for a world point `y`, at
    /// every time: with x = R(−θ)(y − b), |x''| <= |ω|·|v| + |ω|·|b'| + |b''|
    /// and |v| <= |ω|·|y − b| + |b'|, from bounds on |b|, |b'| and |b''|
    /// carried through the turns (b ← R(b − c) + c: b' ← R b' + wJR(b − c),
    /// b'' ← R b'' + 2wJR b' + w²J²R(b − c)).
    fn accel_bound(&self, y: P2) -> f64 {
        let (mut b, mut d1, mut d2, mut omega) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let span = self.t0.abs().max(self.t1.abs());
        for (k, &(c, w)) in self.turns.iter().enumerate() {
            let arm = b + norm(c);
            d2 += 2.0 * w.abs() * d1 + w * w * arm;
            d1 += w.abs() * arm;
            b += 2.0 * norm(c);
            omega += w;
            // A slide adds s t to b and s to b' (nothing to b'').
            let slide = norm(self.slides[k]);
            d1 += slide;
            b += slide * span;
        }
        let omega = omega.abs();
        let v = omega * (norm(y) + b) + d1;
        (omega * v + omega * d1 + d2) * (1.0 + 1e-9) + 1e-300
    }

    /// Bounds on a material point's world speed and on the rate of its
    /// velocity, for a point `x` of the tool: with y = R x + b, |y'| <=
    /// |ω|·|x| + |b'| and |y''| <= ω²·|x| + |b''| (|b'| and |b''| bounded as
    /// in [`Motion::accel_bound`]).
    fn point_rate_bounds(&self, x: P2) -> (f64, f64) {
        let (mut b, mut d1, mut d2, mut omega) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
        let span = self.t0.abs().max(self.t1.abs());
        for (k, &(c, w)) in self.turns.iter().enumerate() {
            let arm = b + norm(c);
            d2 += 2.0 * w.abs() * d1 + w * w * arm;
            d1 += w.abs() * arm;
            b += 2.0 * norm(c);
            omega += w;
            // A slide adds s t to b and s to b' (nothing to b'').
            let slide = norm(self.slides[k]);
            d1 += slide;
            b += slide * span;
        }
        let omega = omega.abs();
        ((omega * norm(x) + d1) * (1.0 + 1e-9), (omega * omega * norm(x) + d2) * (1.0 + 1e-9))
    }

    /// A bound on the preimage's speed for a world point `y`.
    fn speed_bound(&self, y: P2) -> f64 {
        let mut reach = norm(y);
        for &(c, _) in &self.turns {
            reach += 2.0 * norm(c);
        }
        // Slides carry points out by |s| t and move them at |s|.
        let span = self.t0.abs().max(self.t1.abs());
        let slid: f64 = self.slides.iter().map(|s| norm(*s)).sum();
        if slid > 0.0 {
            reach += slid * span;
        }
        let turning = self.turns.iter().map(|&(c, w)| w.abs() * (reach + norm(c))).sum::<f64>();
        (if slid > 0.0 { turning + slid } else { turning }).max(1e-300)
    }

    /// Does the motion slide anywhere?
    fn slides(&self) -> bool {
        self.slides.iter().any(|s| *s != [0.0, 0.0])
    }

    /// How far its slides can carry a point (0 without slides).
    fn slide_reach(&self) -> f64 {
        let span = self.t0.abs().max(self.t1.abs());
        self.slides.iter().map(|s| norm(*s) * span).sum()
    }
}

/// The polygon (tool frame, counter-clockwise or not) and its signed distance.
struct Polygon {
    corners: Vec<P2>,
}

impl Polygon {
    fn signed_distance(&self, p: P2) -> f64 {
        let n = self.corners.len();
        let mut best = f64::INFINITY;
        let mut inside = false;
        for k in 0..n {
            let (a, b) = (self.corners[k], self.corners[(k + 1) % n]);
            let e = sub(b, a);
            let s = (dot(sub(p, a), e) / dot(e, e)).clamp(0.0, 1.0);
            best = best.min(norm(sub(p, add(a, mul(e, s)))));
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
                inside = !inside;
            }
        }
        if inside {
            -best
        } else {
            best
        }
    }
}

/// The clip in the plane: an annulus (and an angle range) about the origin.
struct Clip {
    r: [f64; 2],
    angle: Option<[f64; 2]>,
}

impl Clip {
    fn holds(&self, y: P2) -> bool {
        let r = norm(y);
        if !(r > self.r[0] && r < self.r[1]) {
            return false;
        }
        match self.angle {
            None => true,
            Some([a0, a1]) => {
                let span = if a1 - a0 >= TAU { TAU } else { (a1 - a0).rem_euclid(TAU) };
                let rel = (y[1].atan2(y[0]) - a0).rem_euclid(TAU);
                rel > 0.0 && rel < span
            }
        }
    }
}

/// A boundary candidate: an exact curve over [lo, hi].
#[derive(Clone, Copy, Debug)]
enum Kind {
    /// The path of corner k.
    Corner(usize),
    /// The envelope of side k (from corner k to k + 1).
    Envelope(usize),
    /// Side k at the start (false) or end (true) position: parameter along it.
    End(usize, bool),
    /// A clip circle of radius r: parameter the angle.
    Circle(f64),
    /// A clip ray at angle a: parameter the radius.
    Ray(f64),
    /// Under turns about one centre: the circle of radius r about it that
    /// corners and side feet at that distance run on (parameter the angle).
    Orbit(f64),
    /// Piece k of the target's outline (a line's fraction, an arc's angle).
    Target(usize),
}

struct Curve {
    kind: Kind,
    lo: f64,
    hi: f64,
}

struct Region<'a> {
    motion: &'a Motion,
    polygon: &'a Polygon,
    clip: &'a Clip,
    scale: f64,
    /// A prism the region is cut from: the region is then what it holds
    /// outside the sweep.
    target: Option<&'a Target>,
}

impl Region<'_> {
    /// Where on side k the velocity runs along it, at time t (a fraction of
    /// the side; outside [0, 1] where the foot misses it).
    fn foot(&self, k: usize, t: f64) -> f64 {
        let n = self.polygon.corners.len();
        let (a, b) = (self.motion.apply(t, self.polygon.corners[k]), self.motion.apply(t, self.polygon.corners[(k + 1) % n]));
        let e = sub(b, a);
        let normal = [e[1], -e[0]];
        let (g0, g1) = (dot(normal, self.motion.velocity(t, a)), dot(normal, self.motion.velocity(t, b)));
        if g0 == g1 {
            f64::NAN
        } else {
            g0 / (g0 - g1)
        }
    }

    fn eval(&self, kind: Kind, s: f64) -> P2 {
        let n = self.polygon.corners.len();
        let corner = |k: usize| self.polygon.corners[k % n];
        match kind {
            Kind::Corner(k) => self.motion.apply(s, corner(k)),
            Kind::Envelope(k) => {
                let f = self.foot(k, s).clamp(0.0, 1.0);
                self.motion.apply(s, add(corner(k), mul(sub(corner(k + 1), corner(k)), f)))
            }
            Kind::End(k, end) => {
                let t = if end { self.motion.t1 } else { self.motion.t0 };
                self.motion.apply(t, add(corner(k), mul(sub(corner(k + 1), corner(k)), s)))
            }
            Kind::Circle(r) => [r * s.cos(), r * s.sin()],
            Kind::Orbit(r) => {
                let c = self.motion.turns[0].0;
                [c[0] + r * s.cos(), c[1] + r * s.sin()]
            }
            Kind::Ray(a) => [s * a.cos(), s * a.sin()],
            Kind::Target(k) => self.target.expect("a target's piece").eval(k, s),
        }
    }

    /// Is `y` in the region: in the sweep (within the clip), or, cut from a
    /// target, in the target and not in the sweep.
    fn member(&self, y: P2) -> bool {
        match self.target {
            None => self.inside(y, 0.0),
            Some(target) => target.holds(y) && !self.inside(y, 0.0),
        }
    }

    /// Is `y` inside the swept region and the clip? The walk steps the time
    /// by the polygon's signed distance over the preimage's speed bound, so it
    /// skips no time at which the preimage is deeper than `margin`.
    fn inside(&self, y: P2, margin: f64) -> bool {
        if !self.clip.holds(y) {
            return false;
        }
        if margin == 0.0 && region_walk_on() {
            return self.inside_curved(y);
        }
        let speed = self.motion.speed_bound(y);
        let (t0, t1) = (self.motion.t0, self.motion.t1);
        let floor = (t1 - t0) * 1e-10;
        let mut t = t0;
        for _ in 0..5_000_000 {
            let d = self.polygon.signed_distance(self.motion.invert(t, y));
            if d < -margin {
                return true;
            }
            if t >= t1 {
                return false;
            }
            t = (t + ((d + margin) / speed).max(floor)).min(t1);
        }
        true
    }
}

impl Region<'_> {
    /// [`Region::inside`] (margin 0) by steps of second order: while the
    /// preimage x is outside the polygon its distance to each side's segment
    /// is convex in x, so over a step s it stays at least
    /// d_k + (∇d_k·x')s − ½As² (A bounding |x''|); the step is the least s
    /// at which any of these could reach zero. Near a tangential approach
    /// (∇d_k·x' ≈ 0) that is √(2d/A), where a step of d over the speed bound
    /// crawls.
    fn inside_curved(&self, y: P2) -> bool {
        let (t0, t1) = (self.motion.t0, self.motion.t1);
        let accel = self.motion.accel_bound(y);
        let floor = (t1 - t0) * 1e-10;
        let corners = &self.polygon.corners;
        let n = corners.len();
        let mut t = t0;
        for _ in 0..5_000_000 {
            let (theta, b, dtheta, db) = self.motion.pose(t);
            let r = sub(y, b);
            let x = rot(r, -theta);
            // x' = −R(−θ)·v, v the world velocity of the material point at y.
            let xv = mul(rot(add(mul([-r[1], r[0]], dtheta), db), -theta), -1.0);
            let mut inside = false;
            let mut step = f64::INFINITY;
            let mut touches = false;
            for k in 0..n {
                let (a, c) = (corners[k], corners[(k + 1) % n]);
                let e = sub(c, a);
                let length = norm(e);
                let w = sub(x, a);
                let f = dot(w, e) / (length * length);
                if (a[1] > x[1]) != (c[1] > x[1]) && x[0] < a[0] + (x[1] - a[1]) / (c[1] - a[1]) * (c[0] - a[0]) {
                    inside = !inside;
                }
                // The distance to the side's segment and a subgradient of it:
                // off the side's middle, the side's own unit normal (the
                // foot's offset there is all rounding near the side, and
                // would point along it); off an end, from that corner.
                let (d, gradient) = if f > 0.0 && f < 1.0 {
                    let h = cross(e, w) / length;
                    (h.abs(), mul([-e[1], e[0]], h.signum() / length))
                } else {
                    let off = if f <= 0.0 { w } else { sub(x, c) };
                    let d = norm(off);
                    (d, if d > 0.0 { mul(off, 1.0 / d) } else { [0.0, 0.0] })
                };
                // On the side exactly: no step but the floor's. (Near it the
                // normal above is still the side's own, so a point that
                // lingers close outside steps on by the curvature bound.)
                if !(d > 0.0) {
                    touches = true;
                    continue;
                }
                let g = dot(gradient, xv);
                // The positive root of d + g s − A s²/2; approaching (g < 0)
                // in the form without cancellation (2d over the root's
                // conjugate), which a near-zero A would round to nothing.
                let root = (g * g + 2.0 * accel * d).sqrt();
                step = step.min(if g >= 0.0 { (g + root) / accel } else { 2.0 * d / (root - g) });
            }
            if inside && !touches {
                return true;
            }
            if t >= t1 {
                return false;
            }
            let step = if touches || !step.is_finite() { floor } else { step.max(floor) };
            t = (t + step).min(t1);
        }
        true
    }
}

/// The switch for slides in the region engine's motions (default on;
/// `BREP_SWEEP_REGION_SLIDE=0` leaves them to the level engine): a slide
/// square to the clip's axis beside the turns (a rack rolled round its blank).
pub fn region_slide_on() -> bool {
    crate::switch!("BREP_SWEEP_REGION_SLIDE", true)
}

/// The switch for [`Region::inside_curved`] (default on;
/// `BREP_SWEEP_REGION_WALK=0` restores the first-order walk).
pub fn region_walk_on() -> bool {
    crate::switch!("BREP_SWEEP_REGION_WALK", true)
}

/// The switch for the side sampling bound in [`region_loops`] (default on;
/// `BREP_SWEEP_REGION_SIDE_BOUND=0` samples every side on the common grid
/// and never declines, as before).
pub fn region_side_bound_on() -> bool {
    crate::switch!("BREP_SWEEP_REGION_SIDE_BOUND", true)
}

/// One kept arc: its curve, its parameter range (run with the region on its
/// left), and its end points.
struct Arc {
    kind: Kind,
    a: f64,
    b: f64,
    start: P2,
    end: P2,
}

/// The sweep of `tool` along `motion` within `clip` as exact planar regions,
/// extruded; `None` where this construction does not apply (not a prism of
/// straight sides, a step that is not a turn about an axis along the prism,
/// a clip not along it).
pub fn sweep_prism_planar_region(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    let Some(prepared) = prepare(tool, motion, clip)? else { return Ok(None) };
    let region = Region { motion: &prepared.motion, polygon: &prepared.polygon, clip: &prepared.clip, scale: prepared.scale, target: None };
    let Some(loops) = region_loops(&region)? else { return Ok(None) };
    if loops.is_empty() {
        return Ok(Some(Vec::new()));
    }
    let solids = extrude(&region, &loops, clip.axial, &|p, z| prepared.lift(p, z))?;
    Ok(Some(solids))
}

/// The region engine's inputs in the plane square to the clip's axis.
struct Prepared {
    motion: Motion,
    polygon: Polygon,
    clip: Clip,
    scale: f64,
    origin: Vec3,
    frame: [Vec3; 3],
}

impl Prepared {
    fn plane(&self, x: Vec3) -> P2 {
        let d = x.sub(self.origin);
        [d.dot(self.frame[0]), d.dot(self.frame[1])]
    }
    fn lift(&self, p: P2, z: f64) -> Vec3 {
        self.origin.add(self.frame[0].scale(p[0])).add(self.frame[1].scale(p[1])).add(self.frame[2].scale(z))
    }
}

fn prepare(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Prepared>, KernelRefusal> {
    let SweptTool::Prism { origin, normal, x_axis, outline, from, to } = tool else { return Ok(None) };
    if !outline.iter().all(|p| matches!(p, OutlinePiece::Line { .. })) || outline.len() < 3 {
        return Ok(None);
    }
    // Each line must end where the next begins (else the level engine refuses
    // the outline in its own words).
    let size = outline.iter().map(|p| match p {
        OutlinePiece::Line { from, .. } => from[0].abs().max(from[1].abs()),
        _ => 0.0,
    }).fold(1.0f64, f64::max);
    let closes = (0..outline.len()).all(|k| match (&outline[k], &outline[(k + 1) % outline.len()]) {
        (OutlinePiece::Line { to, .. }, OutlinePiece::Line { from, .. }) => (to[0] - from[0]).hypot(to[1] - from[1]) <= 1e-12 * size,
        _ => false,
    });
    if !closes || motion.steps.iter().all(|step| step.turn == 0.0) {
        return Ok(None);
    }
    // An invalid clip or range is the level engine's to refuse, in its words.
    let finite = clip.radius.iter().chain(&clip.axial).all(|x| x.is_finite()) && motion.from.is_finite() && motion.to.is_finite();
    if !(finite && clip.radius[0] >= 0.0 && clip.radius[1] > clip.radius[0] && clip.axial[1] > clip.axial[0] && motion.to > motion.from) {
        return Ok(None);
    }
    let ez = clip.axis_direction.normalized().or_refuse(KernelStage::Collect, "clip_axis")?;
    let n = normal.normalized().or_refuse(KernelStage::Collect, "prism_normal")?;
    if n.cross(ez).length() > 1e-12 || motion.steps.is_empty() || !(motion.to > motion.from) {
        return Ok(None);
    }
    let ex = clip.reference.sub(ez.scale(clip.reference.dot(ez))).normalized().or_refuse(KernelStage::Collect, "clip_reference")?;
    let ey = ez.cross(ex);
    let plane = |x: Vec3| -> P2 {
        let d = x.sub(clip.axis_origin);
        [d.dot(ex), d.dot(ey)]
    };
    let mut turns = Vec::new();
    let mut slides = Vec::new();
    for ScrewStep { axis_origin, axis_direction, turn, advance } in &motion.steps {
        let axis = axis_direction.normalized().or_refuse(KernelStage::Collect, "step_axis")?;
        // A slide square to the clip's axis (behind its switch): it moves
        // the plane in itself.
        if *turn == 0.0 && *advance != 0.0 && axis.dot(ez).abs() <= 1e-12 && region_slide_on() {
            turns.push(([0.0, 0.0], 0.0));
            slides.push([axis.dot(ex) * advance, axis.dot(ey) * advance]);
            continue;
        }
        if *advance != 0.0 || axis.cross(ez).length() > 1e-12 {
            return Ok(None);
        }
        turns.push((plane(*axis_origin), turn * axis.dot(ez).signum()));
        slides.push([0.0, 0.0]);
    }
    // The clip's axial range must lie within the prism (as for the level engine).
    let base = clip.axis_origin.sub(*origin).dot(n);
    let sense = ez.dot(n);
    let (h0, h1) = (base + sense * clip.axial[0], base + sense * clip.axial[1]);
    if h0.min(h1) < *from || h0.max(h1) > *to {
        return Ok(None);
    }
    let xa = x_axis.sub(n.scale(x_axis.dot(n))).normalized().or_refuse(KernelStage::Collect, "prism_x_axis")?;
    let ya = n.cross(xa);
    let mut corners: Vec<P2> = Vec::with_capacity(outline.len());
    for piece in outline {
        let OutlinePiece::Line { from: q, .. } = piece else { unreachable!() };
        corners.push(plane(origin.add(xa.scale(q[0])).add(ya.scale(q[1]))));
    }
    let polygon = Polygon { corners };
    let motion2 = Motion { turns, slides, t0: motion.from, t1: motion.to };
    // An angle range of a whole turn or more is no range.
    let angle = clip.angle.filter(|[a0, a1]| a1 - a0 < TAU);
    let clip2 = Clip { r: clip.radius, angle };
    // Turns about one centre that cancel leave the tool still: not a sweep
    // this construction draws.
    let turns = &motion2.turns;
    if let Some(&(c0, _)) = turns.first() {
        let one = turns.iter().all(|(c, _)| norm(sub(*c, c0)) <= 1e-12 * norm(c0).max(1.0));
        if one && !motion2.slides() && turns.iter().map(|(_, w)| w).sum::<f64>().abs() <= 1e-15 * turns.iter().map(|(_, w)| w.abs()).sum::<f64>() {
            return Ok(None);
        }
    }
    // The geometry's own size: the corners, the turns' centres, and the clip
    // only as far as the motion can reach (an oversized clip must not widen
    // the probes past the tool).
    let corner_reach = polygon.corners.iter().map(|c| norm(*c)).fold(0.0f64, f64::max);
    let centres: f64 = motion2.turns.iter().map(|(c, _)| norm(*c)).sum();
    let reach = if motion2.slides() { corner_reach + 2.0 * centres + motion2.slide_reach() } else { corner_reach + 2.0 * centres };
    let scale = corner_reach.max(motion2.turns.iter().map(|(c, _)| norm(*c)).fold(0.0f64, f64::max)).max(clip.radius[1].min(reach));
    if clip.radius[1] - clip.radius[0] < 1e-6 * scale {
        return Ok(None);
    }
    Ok(Some(Prepared { motion: motion2, polygon, clip: clip2, scale, origin: clip.axis_origin, frame: [ex, ey, ez] }))
}

/// The switch for [`cut_planar_sweep_from_prism`] (default on;
/// `BREP_SWEEP_REGION_CUT=0` turns it off).
pub fn region_cut_on() -> bool {
    crate::switch!("BREP_SWEEP_REGION_CUT", true)
}

/// A prism along the clip's axis, its outline in the plane square to it
/// (exact: lines and circular arcs, in that plane's coordinates).
struct Target {
    pieces: Vec<TargetPiece>,
    /// The outline as y-monotone pieces (for membership).
    monotone: Vec<(P2, P2, Option<(P2, f64, f64)>)>,
    /// Radii of clip circles the target's outline covers whole (left out).
    shadowed: Vec<f64>,
}

#[derive(Clone, Copy, Debug)]
enum TargetPiece {
    Line(P2, P2),
    /// Centre, radius, and the angles from and to (either way round).
    Arc(P2, f64, f64, f64),
}

impl Target {
    fn eval(&self, k: usize, s: f64) -> P2 {
        match self.pieces[k] {
            TargetPiece::Line(a, b) => add(a, mul(sub(b, a), s)),
            TargetPiece::Arc(c, r, _, _) => [c[0] + r * s.cos(), c[1] + r * s.sin()],
        }
    }
    fn span(&self, k: usize) -> (f64, f64) {
        match self.pieces[k] {
            TargetPiece::Line(..) => (0.0, 1.0),
            TargetPiece::Arc(_, _, a, b) => (a.min(b), a.max(b)),
        }
    }
    /// Is `p` inside the outline? Crossings of the ray to +x, counted as a
    /// polygon's sides are (a piece crosses where one end lies above p and
    /// the other not), over y-monotone pieces: the lines, and the arcs split
    /// at their highest and lowest points, each ending at the outline's own
    /// shared corners so neighbours agree on which side of p a corner lies.
    fn holds(&self, p: P2) -> bool {
        let mut inside = false;
        for &(a, b, arc) in &self.monotone {
            if (a[1] > p[1]) == (b[1] > p[1]) {
                continue;
            }
            let x = match arc {
                None => a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]),
                Some((c, r, side)) => {
                    let dy = p[1] - c[1];
                    c[0] + side * (r * r - dy * dy).max(0.0).sqrt()
                }
            };
            if p[0] < x {
                inside = !inside;
            }
        }
        inside
    }

    /// The outline's y-monotone pieces: (from, to, for an arc its centre,
    /// radius and the side of the centre it runs on); `corners[k]` is where
    /// piece k begins (and piece k − 1 ends).
    fn monotone_pieces(pieces: &[TargetPiece], corners: &[P2]) -> Vec<(P2, P2, Option<(P2, f64, f64)>)> {
        let n = pieces.len();
        let mut out = Vec::new();
        for k in 0..n {
            let (start, end) = (corners[k], corners[(k + 1) % n]);
            match pieces[k] {
                TargetPiece::Line(..) => out.push((start, end, None)),
                TargetPiece::Arc(c, r, a0, a1) => {
                    // Angles of the arc's extremes in y, strictly inside its run.
                    let (lo, hi) = (a0.min(a1), a0.max(a1));
                    let mut breaks: Vec<f64> = Vec::new();
                    let mut m = ((lo - std::f64::consts::FRAC_PI_2) / std::f64::consts::PI).floor() + 1.0;
                    loop {
                        let angle = std::f64::consts::FRAC_PI_2 + m * std::f64::consts::PI;
                        if angle >= hi {
                            break;
                        }
                        if angle > lo {
                            breaks.push(angle);
                        }
                        m += 1.0;
                    }
                    if a1 < a0 {
                        breaks.reverse();
                    }
                    let mut points = vec![(start, a0)];
                    points.extend(breaks.iter().map(|&t| ([c[0] + r * t.cos(), c[1] + r * t.sin()], t)));
                    points.push((end, a1));
                    for w in points.windows(2) {
                        let mid = 0.5 * (w[0].1 + w[1].1);
                        out.push((w[0].0, w[1].0, Some((c, r, if mid.cos() >= 0.0 { 1.0 } else { -1.0 }))));
                    }
                }
            }
        }
        out
    }
}

/// Does a target piece run along one of the sweep's own boundary curves
/// (where the two would both be kept, the boundary traced twice)? Under
/// turns about one centre: an arc about it on a corner's or a side foot's
/// orbit; at the motion's ends: a line along a side's end position.
fn on_sweep_curve(prepared: &Prepared, pieces: &[TargetPiece], tolerance: f64) -> bool {
    let motion = &prepared.motion;
    let corners = &prepared.polygon.corners;
    let n = corners.len();
    let centre = motion.turns[0].0;
    let one_centre = !motion.slides() && motion.turns.iter().all(|(c, _)| norm(sub(*c, centre)) <= 1e-12 * prepared.scale);
    let mut orbits = Vec::new();
    if one_centre {
        for k in 0..n {
            let (a, b) = (corners[k], corners[(k + 1) % n]);
            orbits.push(norm(sub(motion.apply(motion.t0, a), centre)));
            // The side's nearest point to the centre runs an orbit too.
            let (pa, pb) = (motion.apply(motion.t0, a), motion.apply(motion.t0, b));
            let e = sub(pb, pa);
            let f = (dot(sub(centre, pa), e) / dot(e, e)).clamp(0.0, 1.0);
            orbits.push(norm(sub(add(pa, mul(e, f)), centre)));
        }
    }
    let mut sides = Vec::new();
    for t in [motion.t0, motion.t1] {
        for k in 0..n {
            sides.push((motion.apply(t, corners[k]), motion.apply(t, corners[(k + 1) % n])));
        }
    }
    pieces.iter().any(|piece| match *piece {
        TargetPiece::Arc(c, r, ..) => one_centre && norm(sub(c, centre)) <= tolerance && orbits.iter().any(|&o| (o - r).abs() <= tolerance),
        TargetPiece::Line(a, b) => sides.iter().any(|&(p, q)| {
            let e = sub(q, p);
            let length = norm(e);
            if !(length > 0.0) {
                return false;
            }
            let along = |x: P2| (cross(e, sub(x, p)) / length).abs() <= tolerance;
            if !(along(a) && along(b)) {
                return false;
            }
            // Collinear: overlapping by more than the tolerance?
            let s = |x: P2| dot(sub(x, p), e) / length;
            let (lo, hi) = (s(a).min(s(b)), s(a).max(s(b)));
            hi.min(length) - lo.max(0.0) > tolerance
        }),
    })
}

/// Whether the clip-plane point `p` lies in what `tool` sweeps under
/// `motion` within `clip`, by the region engine's membership walk; `None`
/// where the engine does not take the sweep. (For tests.)
#[doc(hidden)]
pub fn swept_region_holds(tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip, p: [f64; 2]) -> Result<Option<bool>, KernelRefusal> {
    let Some(prepared) = prepare(tool, motion, clip)? else { return Ok(None) };
    let region = Region { motion: &prepared.motion, polygon: &prepared.polygon, clip: &prepared.clip, scale: prepared.scale, target: None };
    Ok(Some(region.inside(p, 0.0)))
}

/// Whether `p` lies inside a closed outline of lines and circular arcs (in
/// the outline's own coordinates), as the region cut reads its target;
/// `None` for an outline it does not take. (For tests.)
#[doc(hidden)]
pub fn target_outline_holds(outline: &[OutlinePiece], p: [f64; 2]) -> Option<bool> {
    let mut pieces = Vec::new();
    let mut corners = Vec::new();
    for piece in outline {
        match *piece {
            OutlinePiece::Line { from, to } => {
                pieces.push(TargetPiece::Line(from, to));
                corners.push(from);
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                pieces.push(TargetPiece::Arc(centre, radius, start, end));
                corners.push([centre[0] + radius * start.cos(), centre[1] + radius * start.sin()]);
            }
            OutlinePiece::Ellipse { .. } => return None,
        }
    }
    let monotone = Target::monotone_pieces(&pieces, &corners);
    Some(Target { pieces, monotone, shadowed: Vec::new() }.holds(p))
}

/// The prism `target` less what `tool` sweeps under `motion` within `clip`,
/// as one extrusion of the plane region the target's outline holds outside
/// the swept region (exact where the sweep's own region engine is; behind
/// [`region_cut_on`], default on). The same solid as subtracting
/// [`sweep_prism_planar_region`]'s bodies from the target, without the 3D
/// boolean: the sweep is taken within the clip only, as there.
///
/// `None` where it does not apply: the switch off, the sweep not one the
/// region engine draws, a target that is not a prism along the clip's axis
/// with an outline of lines and circular arcs end to end, or a target that
/// runs past the clip's axial range.
pub fn cut_planar_sweep_from_prism(target: &SweptTool, tool: &SweptTool, motion: &SolidMotion, clip: &SweepClip) -> Result<Option<Vec<BrepSolid>>, KernelRefusal> {
    if !region_cut_on() {
        return Ok(None);
    }
    let SweptTool::Prism { origin, normal, x_axis, outline, from, to } = target else { return Ok(None) };
    if outline.is_empty() || !(from.is_finite() && to.is_finite() && to > from) {
        return Ok(None);
    }
    let Some(prepared) = prepare(tool, motion, clip)? else { return Ok(None) };
    let ez = prepared.frame[2];
    let Ok(n) = normal.normalized() else { return Ok(None) };
    if n.cross(ez).length() > 1e-12 {
        return Ok(None);
    }
    // The target's heights along the clip's axis, within the clip's range.
    let base = origin.sub(prepared.origin).dot(ez);
    let sense = n.dot(ez).signum();
    let (h0, h1) = (base + sense * from, base + sense * to);
    let axial = [h0.min(h1), h0.max(h1)];
    let slack = 1e-12 * prepared.scale;
    if axial[0] < clip.axial[0] - slack || axial[1] > clip.axial[1] + slack {
        return Ok(None);
    }
    let Ok(xa) = x_axis.sub(n.scale(x_axis.dot(n))).normalized() else { return Ok(None) };
    let ya = n.cross(xa);
    let at = |q: [f64; 2]| prepared.plane(origin.add(xa.scale(q[0])).add(ya.scale(q[1])));
    // The outline's angles in the plane: turned by the target's x axis, and
    // mirrored where its normal runs against the clip's axis.
    let offset = xa.dot(prepared.frame[1]).atan2(xa.dot(prepared.frame[0]));
    let mut pieces = Vec::with_capacity(outline.len());
    let mut ends: Vec<(P2, P2)> = Vec::with_capacity(outline.len());
    for piece in outline {
        match *piece {
            OutlinePiece::Line { from: a, to: b } => {
                let (a, b) = (at(a), at(b));
                pieces.push(TargetPiece::Line(a, b));
                ends.push((a, b));
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                if !(radius > 0.0 && radius.is_finite() && start.is_finite() && end.is_finite()) || (end - start).abs() > TAU * (1.0 + 1e-12) || end == start {
                    return Ok(None);
                }
                let c = at(centre);
                let (a0, a1) = (offset + sense * start, offset + sense * end);
                pieces.push(TargetPiece::Arc(c, radius, a0, a1));
                ends.push(([c[0] + radius * a0.cos(), c[1] + radius * a0.sin()], [c[0] + radius * a1.cos(), c[1] + radius * a1.sin()]));
            }
            OutlinePiece::Ellipse { .. } => return Ok(None),
        }
    }
    let size = ends.iter().flat_map(|(a, b)| [norm(*a), norm(*b)]).fold(prepared.scale, f64::max);
    let closes = (0..ends.len()).all(|k| norm(sub(ends[k].1, ends[(k + 1) % ends.len()].0)) <= 1e-9 * size);
    if !closes {
        return Ok(None);
    }
    // The region's tolerances and probes follow the sweep's scale: a target
    // far smaller or larger than it is not drawn here.
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (k, piece) in pieces.iter().enumerate() {
        let mut take = |q: P2| {
            for i in 0..2 {
                lo[i] = lo[i].min(q[i]);
                hi[i] = hi[i].max(q[i]);
            }
        };
        take(ends[k].0);
        if let TargetPiece::Arc(c, r, ..) = piece {
            take([c[0] - r, c[1] - r]);
            take([c[0] + r, c[1] + r]);
        }
    }
    let extent = (hi[0] - lo[0]).hypot(hi[1] - lo[1]);
    if !(extent >= 1e-2 * prepared.scale && extent <= 1e2 * prepared.scale) {
        return Ok(None);
    }
    // A target piece along a clip curve: a whole circle about the clip's
    // axis on a clip circle stands for it (the clip's circle is left out);
    // any other overlap (part of a circle, a side along a clip ray) is not
    // drawn here.
    let tolerance = 1e-9 * prepared.scale;
    let mut shadowed = Vec::new();
    for piece in &pieces {
        match *piece {
            TargetPiece::Arc(c, r, a0, a1) => {
                if norm(c) <= tolerance {
                    if let Some(&radius) = prepared.clip.r.iter().find(|&&radius| (radius - r).abs() <= tolerance) {
                        if prepared.clip.angle.is_none() && (a1 - a0).abs() >= TAU * (1.0 - 1e-12) {
                            shadowed.push(radius);
                        } else {
                            return Ok(None);
                        }
                    }
                }
            }
            TargetPiece::Line(a, b) => {
                if let Some([r0, r1]) = prepared.clip.angle {
                    for ray in [r0, r1] {
                        let d = [ray.cos(), ray.sin()];
                        if cross(d, a).abs() <= tolerance && cross(d, b).abs() <= tolerance {
                            return Ok(None);
                        }
                    }
                }
            }
        }
    }
    if on_sweep_curve(&prepared, &pieces, tolerance) {
        return Ok(None);
    }
    let corners: Vec<P2> = ends.iter().map(|e| e.0).collect();
    let monotone = Target::monotone_pieces(&pieces, &corners);
    let target = Target { pieces, monotone, shadowed };
    let region = Region { motion: &prepared.motion, polygon: &prepared.polygon, clip: &prepared.clip, scale: prepared.scale, target: Some(&target) };
    let Some(loops) = region_loops(&region)? else { return Ok(None) };
    if loops.is_empty() {
        return Ok(Some(Vec::new()));
    }
    Ok(Some(extrude(&region, &loops, axial, &|p, z| prepared.lift(p, z))?))
}

/// The region's boundary loops (each counter-clockwise about its inside).
///
/// A side's foot lies on it while its normal velocity summed at its two ends,
/// G, is within ω·|e|² of zero (ω the net turn rate, e the side), and G moves
/// at most L = |e|·(ω(|a'| + |b'|) + |a''| + |b''|): a run about any zero of G
/// lasts ω|e|²/L to either side, so samples that far apart see every run
/// through a zero and no sign change of G slips between two samples. A side
/// whose bound asks for more than a million samples (ω all but zero: turns
/// that cancel leave a translation along a curve, whose boundary takes a
/// whole side at an instant, no envelope this engine draws) is declined:
/// `None`.
fn region_loops(region: &Region) -> Result<Option<Vec<Vec<Arc>>>, KernelRefusal> {
    let motion = region.motion;
    let polygon = region.polygon;
    let n = polygon.corners.len();
    let (t0, t1) = (motion.t0, motion.t1);
    let scale = region.scale;
    // Does the motion end where it began, as a set (every corner at the end on
    // some corner at the start)? Then there are no end positions.
    let closed = (0..n).all(|k| {
        let end = motion.apply(t1, polygon.corners[k]);
        (0..n).any(|j| norm(sub(end, motion.apply(t0, polygon.corners[j]))) <= 1e-9 * scale)
    });
    // Sample density: by the fastest point's travel.
    let travel = motion.speed_bound([0.0, 0.0]) * (t1 - t0);
    let samples = ((travel / (1e-3 * scale)).ceil() as usize).clamp(2000, 200_000);
    let mut curves: Vec<Curve> = Vec::new();
    // Turns about one centre: every corner path and side envelope is an arc
    // of a circle about it, and those at one distance coincide; each distance
    // gets one curve over the union of its arcs.
    let centre = motion.turns[0].0;
    let one_centre = !motion.slides() && motion.turns.iter().all(|(c, _)| norm(sub(*c, centre)) <= 1e-12 * scale);
    if one_centre {
        let rate: f64 = motion.turns.iter().map(|(_, w)| w).sum();
        let mut arcs_at: Vec<(f64, f64, f64)> = Vec::new(); // (radius, angle from, angle to)
        let mut push = |x: P2| {
            let at = motion.apply(t0, x);
            let r = norm(sub(at, centre));
            let a = (at[1] - centre[1]).atan2(at[0] - centre[0]);
            let (a0, a1) = (a, a + rate * (t1 - t0));
            arcs_at.push((r, a0.min(a1), a0.max(a1)));
        };
        for k in 0..n {
            push(polygon.corners[k]);
            // A side's foot is fixed on it: inside the side, it runs a circle.
            let f = region.foot(k, t0);
            if f > 0.0 && f < 1.0 {
                let (a, b) = (polygon.corners[k], polygon.corners[(k + 1) % n]);
                push(add(a, mul(sub(b, a), f)));
            }
        }
        arcs_at.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut i = 0;
        while i < arcs_at.len() {
            let mut j = i;
            while j + 1 < arcs_at.len() && (arcs_at[j + 1].0 - arcs_at[i].0).abs() <= 1e-12 * scale {
                j += 1;
            }
            let r = arcs_at[i].0;
            // Union of the angle intervals on the circle.
            let mut spans: Vec<(f64, f64)> = arcs_at[i..=j].iter().map(|x| {
                let lo = x.1.rem_euclid(TAU);
                (lo, lo + (x.2 - x.1))
            }).collect();
            spans.sort_by(|x, y| x.0.total_cmp(&y.0));
            let mut merged: Vec<(f64, f64)> = Vec::new();
            for (lo, hi) in spans {
                match merged.last_mut() {
                    Some(last) if lo <= last.1 => last.1 = last.1.max(hi),
                    _ => merged.push((lo, hi)),
                }
            }
            // The last may wrap onto the first.
            if merged.len() > 1 {
                let first = merged[0];
                let last = *merged.last().expect("merged");
                if last.1 >= first.0 + TAU {
                    merged.remove(0);
                    let l = merged.last_mut().expect("merged");
                    l.1 = l.1.max(first.1 + TAU);
                }
            }
            for (lo, hi) in merged {
                let hi = hi.min(lo + TAU);
                if r > 0.0 && hi > lo {
                    curves.push(Curve { kind: Kind::Orbit(r), lo, hi });
                }
            }
            i = j + 1;
        }
    }
    for k in 0..n {
        if one_centre {
            if !closed {
                curves.push(Curve { kind: Kind::End(k, false), lo: 0.0, hi: 1.0 });
                curves.push(Curve { kind: Kind::End(k, true), lo: 0.0, hi: 1.0 });
            }
            continue;
        }
        curves.push(Curve { kind: Kind::Corner(k), lo: t0, hi: t1 });
        // Envelope runs: where the foot lies on the side, sampled at most the
        // bound's spacing apart (the common grid where that is finer).
        let mut side_samples = samples;
        if region_side_bound_on() {
            let (a, b) = (polygon.corners[k], polygon.corners[(k + 1) % n]);
            let omega = motion.turns.iter().map(|(_, w)| w).sum::<f64>().abs();
            let length = norm(sub(b, a));
            let ((va, aa), (vb, ab)) = (motion.point_rate_bounds(a), motion.point_rate_bounds(b));
            let reach = omega * length / (omega * (va + vb) + aa + ab).max(1e-300);
            let need = (t1 - t0) / reach;
            if !(need <= 1e6) {
                return Ok(None);
            }
            if need > samples as f64 {
                side_samples = need.ceil() as usize + 1;
            }
        }
        let ts: Vec<f64> = (0..=side_samples).map(|i| t0 + (t1 - t0) * i as f64 / side_samples as f64).collect();
        let on = |t: f64| {
            let f = region.foot(k, t);
            (0.0..=1.0).contains(&f)
        };
        let flags: Vec<bool> = ts.iter().map(|&t| on(t)).collect();
        let edge = |mut tin: f64, mut tout: f64| {
            for _ in 0..80 {
                let m = 0.5 * (tin + tout);
                if on(m) {
                    tin = m;
                } else {
                    tout = m;
                }
            }
            tin
        };
        let mut i = 0;
        while i < ts.len() {
            if !flags[i] {
                i += 1;
                continue;
            }
            let mut j = i;
            while j + 1 < ts.len() && flags[j + 1] {
                j += 1;
            }
            let lo = if i == 0 { ts[0] } else { edge(ts[i], ts[i - 1]) };
            let hi = if j == ts.len() - 1 { ts[j] } else { edge(ts[j], ts[j + 1]) };
            if hi > lo {
                curves.push(Curve { kind: Kind::Envelope(k), lo, hi });
            }
            i = j + 1;
        }
        if !closed {
            curves.push(Curve { kind: Kind::End(k, false), lo: 0.0, hi: 1.0 });
            curves.push(Curve { kind: Kind::End(k, true), lo: 0.0, hi: 1.0 });
        }
    }
    let clip = region.clip;
    // Every pose keeps the polygon within this distance of the clip's axis
    // (a turn about c moves a point by at most 2|c| further out): clip curves
    // beyond it meet nothing and are left out.
    let turned = polygon.corners.iter().map(|c| norm(*c)).fold(0.0f64, f64::max) + 2.0 * motion.turns.iter().map(|(c, _)| norm(*c)).sum::<f64>();
    let reach = if motion.slides() { turned + motion.slide_reach() } else { turned };
    let shadowed = |r: f64| region.target.is_some_and(|t| t.shadowed.contains(&r));
    let within = |r: f64| r > 0.0 && r <= reach * (1.0 + 1e-12) && !shadowed(r);
    match clip.angle {
        None => {
            for r in clip.r {
                if within(r) {
                    curves.push(Curve { kind: Kind::Circle(r), lo: 0.0, hi: TAU });
                }
            }
        }
        Some([a0, a1]) => {
            let a1 = a0 + (a1 - a0).rem_euclid(TAU);
            for r in clip.r {
                if within(r) {
                    curves.push(Curve { kind: Kind::Circle(r), lo: a0, hi: a1 });
                }
            }
            // Past the reach by a margin: a ray must not end on a curve it crosses.
            let hi = clip.r[1].min(1.1 * reach);
            if hi > clip.r[0] {
                for a in [a0, a1] {
                    curves.push(Curve { kind: Kind::Ray(a), lo: clip.r[0], hi });
                }
            }
        }
    }
    if let Some(target) = region.target {
        for k in 0..target.pieces.len() {
            let (lo, hi) = target.span(k);
            curves.push(Curve { kind: Kind::Target(k), lo, hi });
        }
    }
    // Sample every curve.
    let per = |c: &Curve| -> usize {
        match c.kind {
            Kind::Target(k) => match region.target.expect("a target's piece").pieces[k] {
                TargetPiece::Arc(_, r, ..) => ((r * (c.hi - c.lo) / (1e-3 * scale)).ceil() as usize).clamp(64, 200_000),
                // Segments no longer than the least crossing cell (a long
                // side's segments would each fill a box of cells).
                TargetPiece::Line(a, b) => ((norm(sub(b, a)) / (1e-3 * scale)).ceil() as usize).clamp(64, 200_000),
            },
            Kind::Corner(_) | Kind::Envelope(_) => (((c.hi - c.lo) / (t1 - t0) * samples as f64).ceil() as usize).max(64),
            Kind::Circle(r) | Kind::Orbit(r) => ((r * (c.hi - c.lo) / (1e-3 * scale)).ceil() as usize).clamp(64, 200_000),
            _ => 64,
        }
    };
    let sampled: Vec<(Vec<f64>, Vec<P2>)> = curves
        .iter()
        .map(|c| {
            let m = per(c);
            let ts: Vec<f64> = (0..=m).map(|i| c.lo + (c.hi - c.lo) * i as f64 / m as f64).collect();
            let ps = ts.iter().map(|&t| region.eval(c.kind, t)).collect();
            (ts, ps)
        })
        .collect();
    // Crossings: segments bucketed on a grid, each hit solved on the exact curves.
    // Cells about two segments long (between a thousandth and a hundredth of
    // the scale): a crossing is found in the cell holding it whatever the
    // size, and cells much longer than the segments fill with pairs that
    // cannot meet (where many paths run close, 20M pairs for 504 crossings).
    let (count, length) = sampled.iter().flat_map(|(_, ps)| ps.windows(2)).fold((0usize, 0.0f64), |(n, l), w| (n + 1, l + norm(sub(w[1], w[0]))));
    let mean = if count > 0 { length / count as f64 } else { 0.0 };
    let cell = (2.0 * mean).clamp(1e-3 * scale, 1e-2 * scale);
    // The cell index of a coordinate: floor(x / cell) (in integers, as
    // f64::floor would give it, without the library's floor).
    let index = |x: f64| -> i64 {
        let q = x / cell;
        let i = q as i64;
        if (i as f64) > q {
            i - 1
        } else {
            i
        }
    };
    // Segments by cell, sorted by cell (each cell's run is its bucket).
    let mut grid: Vec<((i64, i64), usize, usize)> = Vec::new();
    for (ci, (_, ps)) in sampled.iter().enumerate() {
        for k in 0..ps.len() - 1 {
            let (p, q) = (ps[k], ps[k + 1]);
            let (gx0, gx1) = (index(p[0].min(q[0])), index(p[0].max(q[0])));
            let (gy0, gy1) = (index(p[1].min(q[1])), index(p[1].max(q[1])));
            for gx in gx0..=gx1 {
                for gy in gy0..=gy1 {
                    grid.push(((gx, gy), ci, k));
                }
            }
        }
    }
    grid.sort_unstable_by(|a, b| a.0.cmp(&b.0));
    let buckets = grid.chunk_by(|a, b| a.0 == b.0);
    let mut splits: Vec<Vec<f64>> = vec![Vec::new(); curves.len()];
    // A segment's cell range; a pair is tested once, in the lowest cell both share.
    let cells = |ci: usize, k: usize| -> (i64, i64) {
        let (p, q) = (sampled[ci].1[k], sampled[ci].1[k + 1]);
        (index(p[0].min(q[0])), index(p[1].min(q[1])))
    };
    for items in buckets {
        let (gx, gy) = items[0].0;
        for x in 0..items.len() {
            for y in x + 1..items.len() {
                let ((_, ci, k), (_, cj, l)) = (items[x], items[y]);
                if ci == cj && k.abs_diff(l) <= 1 {
                    continue;
                }
                let ((ax, ay), (bx, by)) = (cells(ci, k), cells(cj, l));
                if ax.max(bx) != gx || ay.max(by) != gy {
                    continue;
                }
                let (ts_i, ps_i) = &sampled[ci];
                let (ts_j, ps_j) = &sampled[cj];
                let (p, q, r, s) = (ps_i[k], ps_i[k + 1], ps_j[l], ps_j[l + 1]);
                let (d1, d2) = (sub(q, p), sub(s, r));
                let den = cross(d1, d2);
                if den.abs() < 1e-300 {
                    continue;
                }
                let w = sub(r, p);
                let (u, v) = (cross(w, d2) / den, cross(w, d1) / den);
                if !((0.0..1.0).contains(&u) && (0.0..1.0).contains(&v)) {
                    continue;
                }
                let mut ta = ts_i[k] + u * (ts_i[k + 1] - ts_i[k]);
                let mut tb = ts_j[l] + v * (ts_j[l + 1] - ts_j[l]);
                let (ka, kb) = (curves[ci].kind, curves[cj].kind);
                for _ in 0..40 {
                    let f = sub(region.eval(ka, ta), region.eval(kb, tb));
                    let (ha, hb) = (1e-7 * (curves[ci].hi - curves[ci].lo), 1e-7 * (curves[cj].hi - curves[cj].lo));
                    let ja = scale_p(sub(region.eval(ka, ta + ha), region.eval(ka, ta - ha)), 0.5 / ha);
                    let jb = scale_p(sub(region.eval(kb, tb + hb), region.eval(kb, tb - hb)), 0.5 / hb);
                    let det = cross(ja, mul(jb, -1.0));
                    if det.abs() < 1e-300 {
                        break;
                    }
                    // [ja, −jb] (da, db) = −f
                    let da = cross(mul(f, -1.0), mul(jb, -1.0)) / det;
                    let db = cross(ja, mul(f, -1.0)) / det;
                    ta += da;
                    tb += db;
                    if da.abs() + db.abs() < 1e-15 * (1.0 + ta.abs() + tb.abs()) {
                        break;
                    }
                }
                let gap = norm(sub(region.eval(ka, ta), region.eval(kb, tb)));
                // An envelope touches its side's corner paths tangentially at
                // its ends (added below, exactly): a near-touch there is that.
                let end_touch = |e: usize, other: Kind, te: f64| -> bool {
                    let Kind::Envelope(k) = curves[e].kind else { return false };
                    let own = matches!(other, Kind::Corner(j) if j == k || j == (k + 1) % n);
                    let span = curves[e].hi - curves[e].lo;
                    own && ((te - curves[e].lo).abs() < 1e-2 * span || (curves[e].hi - te).abs() < 1e-2 * span)
                };
                if gap > 1e-9 * scale && (end_touch(ci, kb, ta) || end_touch(cj, ka, tb)) {
                    continue;
                }
                if gap > 1e-9 * scale {
                    return Err(refuse("planar_crossing", format!("two boundary curves' crossing did not solve: {ka:?} at {ta} and {kb:?} at {tb}, gap {gap:.3e} (from samples {} / {})", ts_i[k], ts_j[l])));
                }
                splits[ci].push(ta);
                splits[cj].push(tb);
            }
        }
    }
    // Where the motion has ends, a side's envelope (or, about one centre, its
    // foot's orbit) meets that side's end position tangentially, at its foot.
    if !closed {
        for k in 0..n {
            for (end, t) in [(false, t0), (true, t1)] {
                let f = region.foot(k, t);
                let reaches = one_centre || curves.iter().any(|c| matches!(c.kind, Kind::Envelope(j) if j == k) && ((t - c.lo).abs() <= 1e-12 * (t1 - t0) || (c.hi - t).abs() <= 1e-12 * (t1 - t0)));
                if reaches && f > 0.0 && f < 1.0 {
                    if let Some(cj) = curves.iter().position(|c| matches!(c.kind, Kind::End(j, e) if j == k && e == end)) {
                        splits[cj].push(f);
                    }
                }
            }
        }
    }
    // An envelope's ends lie on its side's corner paths (tangent there).
    for ci in 0..curves.len() {
        if let Kind::Envelope(k) = curves[ci].kind {
            for t in [curves[ci].lo, curves[ci].hi] {
                let corner = if region.foot(k, t) < 0.5 { k } else { (k + 1) % n };
                if let Some(cj) = curves.iter().position(|c| matches!(c.kind, Kind::Corner(j) if j == corner)) {
                    splits[cj].push(t);
                }
            }
        }
    }
    // Arcs: kept where membership differs across them.
    // Probes beside an arc: well inside any feature the clip allows (a clip
    // thinner than a millionth of the scale is declined above).
    let eps = (1e-7 * scale).min(1e-3 * (region.clip.r[1] - region.clip.r[0]));
    let mut arcs = Vec::new();
    for (ci, c) in curves.iter().enumerate() {
        let mut ps: Vec<f64> = splits[ci].iter().copied().filter(|t| *t > c.lo && *t < c.hi).collect();
        ps.push(c.lo);
        ps.push(c.hi);
        ps.sort_by(|a, b| a.total_cmp(b));
        ps.dedup_by(|b, a| (*b - *a).abs() <= 1e-13 * (c.hi - c.lo));
        for w in ps.windows(2) {
            let (a, b) = (w[0], w[1]);
            let m = 0.5 * (a + b);
            let y = region.eval(c.kind, m);
            let h = 1e-6 * (b - a);
            let d = sub(region.eval(c.kind, m + h), region.eval(c.kind, m - h));
            let nd = norm(d);
            if !(nd > 0.0) {
                continue;
            }
            let left = [-d[1] / nd, d[0] / nd];
            let (l, r) = (region.member(add(y, mul(left, eps))), region.member(add(y, mul(left, -eps))));
            if l == r {
                continue;
            }
            let (a, b) = if l { (a, b) } else { (b, a) };
            arcs.push(Arc { kind: c.kind, a, b, start: region.eval(c.kind, a), end: region.eval(c.kind, b) });
        }
    }
    // Chain the arcs into loops by their end points.
    let tolerance = 1e-7 * scale;
    let mut used = vec![false; arcs.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    for first in 0..arcs.len() {
        if used[first] {
            continue;
        }
        used[first] = true;
        let mut chain = vec![first];
        loop {
            let tail = arcs[*chain.last().expect("chain")].end;
            if norm(sub(tail, arcs[first].start)) <= tolerance && chain.len() > 1 {
                break;
            }
            let next = (0..arcs.len()).filter(|&k| !used[k]).min_by(|&x, &y| norm(sub(arcs[x].start, tail)).total_cmp(&norm(sub(arcs[y].start, tail))));
            match next {
                Some(k) if norm(sub(arcs[k].start, tail)) <= tolerance => {
                    used[k] = true;
                    chain.push(k);
                }
                _ => {
                    if norm(sub(tail, arcs[first].start)) <= tolerance {
                        break;
                    }
                    let open: Vec<String> = chain.iter().map(|&k| format!("{:?} {:.6}..{:.6} {:?}->{:?}", arcs[k].kind, arcs[k].a, arcs[k].b, arcs[k].start, arcs[k].end)).collect();
                    let nearest = (0..arcs.len()).filter(|&k| !used[k]).map(|k| (norm(sub(arcs[k].start, tail)), k)).min_by(|a, b| a.0.total_cmp(&b.0));
                    return Err(refuse("planar_loops", format!("the region's boundary arcs do not close into loops: {} arcs, chain {open:?}, nearest start {:?}", arcs.len(), nearest.map(|(d, k)| (d, arcs[k].kind, arcs[k].start)))));
                }
            }
        }
        loops.push(chain);
    }
    let mut arcs: Vec<Option<Arc>> = arcs.into_iter().map(Some).collect();
    Ok(Some(loops.into_iter().map(|chain| chain.into_iter().map(|k| arcs[k].take().expect("once")).collect()).collect()))
}

fn scale_p(a: P2, s: f64) -> P2 {
    mul(a, s)
}

/// An arc as a planar curve (in the plane's coordinates, z = 0) over [0, 1]:
/// a line or circle exactly, any other interpolated (quintic) through exact
/// points, doubled until its midpoints miss by under 1e-10 of the scale.
fn arc_curve(region: &Region, arc: &Arc) -> Result<NurbsCurve, KernelRefusal> {
    let p3 = |p: P2| Vec3::new(p[0], p[1], 0.0);
    match arc.kind {
        Kind::End(..) | Kind::Ray(_) => return frag(make_line(p3(arc.start), p3(arc.end)), "planar_line"),
        Kind::Target(k) => match region.target.expect("a target's piece").pieces[k] {
            TargetPiece::Line(..) => return frag(make_line(p3(arc.start), p3(arc.end)), "planar_line"),
            TargetPiece::Arc(c, r, ..) => {
                let (x, y) = (Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
                let (lo, hi) = (arc.a.min(arc.b), arc.a.max(arc.b));
                let curve = frag(crate::make_arc(p3(c), x, y, r, lo, hi), "planar_arc")?;
                return if arc.a <= arc.b { Ok(curve) } else { frag(curve.reversed(), "planar_arc") };
            }
        },
        _ => {}
    }
    let mut count = 32usize;
    loop {
        let params: Vec<f64> = (0..=count).map(|i| i as f64 / count as f64).collect();
        let points: Vec<Vec3> = params.iter().map(|&u| p3(region.eval(arc.kind, arc.a + (arc.b - arc.a) * u))).collect();
        let curve = frag(crate::interpolate_curve(&points, 5, &params), "planar_curve")?;
        let mut worst = 0.0f64;
        for i in 0..count {
            let u = (i as f64 + 0.5) / count as f64;
            let exact = p3(region.eval(arc.kind, arc.a + (arc.b - arc.a) * u));
            worst = worst.max(frag(curve.evaluate(u), "planar_curve")?.sub(exact).length());
        }
        if worst <= 1e-10 * region.scale || count >= 8192 {
            if worst > 1e-7 * region.scale {
                return Err(refuse("planar_fit", format!("a boundary arc's fit misses by {worst:.2e}")));
            }
            return Ok(curve);
        }
        count *= 2;
    }
}

/// Green's area of one loop (counter-clockwise positive), by exact samples.
fn loop_area(region: &Region, arcs: &[Arc]) -> f64 {
    let mut area = 0.0;
    for arc in arcs {
        let m = 4096;
        let mut previous = region.eval(arc.kind, arc.a);
        for i in 1..=m {
            // Simpson on x dy − y dx per pair of steps would be finer; the
            // samples are dense enough for the check this serves.
            let p = region.eval(arc.kind, arc.a + (arc.b - arc.a) * i as f64 / m as f64);
            area += 0.5 * cross(previous, p);
            previous = p;
        }
    }
    area
}

/// Extrude the loops (outer loops with the holes inside them) over the clip's
/// axial range: one solid per outer loop.
fn extrude(region: &Region, loops: &[Vec<Arc>], axial: [f64; 2], lift: &dyn Fn(P2, f64) -> Vec3) -> Result<Vec<BrepSolid>, KernelRefusal> {
    let areas: Vec<f64> = loops.iter().map(|l| loop_area(region, l)).collect();
    let outers: Vec<usize> = (0..loops.len()).filter(|&k| areas[k] > 0.0).collect();
    // Each hole goes to the smallest outer loop holding its first point.
    let holds = |outer: usize, p: P2| -> bool {
        let mut inside = false;
        let pts: Vec<P2> = loops[outer].iter().flat_map(|arc| (0..64).map(move |i| region.eval(arc.kind, arc.a + (arc.b - arc.a) * i as f64 / 64.0))).collect();
        let n = pts.len();
        for k in 0..n {
            let (a, b) = (pts[k], pts[(k + 1) % n]);
            if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
                inside = !inside;
            }
        }
        inside
    };
    let mut groups: Vec<(usize, Vec<usize>)> = outers.iter().map(|&o| (o, Vec::new())).collect();
    for h in (0..loops.len()).filter(|&k| areas[k] <= 0.0) {
        let p = loops[h][0].start;
        let owner = groups.iter_mut().filter(|g| holds(g.0, p)).min_by(|a, b| areas[a.0].total_cmp(&areas[b.0]));
        match owner {
            Some(g) => g.1.push(h),
            None => return Err(refuse("planar_loops", "a hole of the region lies in no outer loop".into())),
        }
    }
    let (z0, z1) = (axial[0], axial[1]);
    let height = z1 - z0;
    let mut solids = Vec::new();
    for (outer, holes) in groups {
        let mut next = 1u64;
        let mut id = || {
            next += 1;
            next
        };
        let members: Vec<usize> = std::iter::once(outer).chain(holes.iter().copied()).collect();
        let mut vertices = Vec::new();
        let mut edges = Vec::new();
        let mut faces = Vec::new();
        let mut bottom_loops = Vec::new();
        let mut top_loops = Vec::new();
        let mut expected = 0.0;
        for &l in &members {
            expected += areas[l] * height;
            let arcs = &loops[l];
            let m = arcs.len();
            // Corner vertices (bottom, top) and their upright edges.
            let mut low = Vec::with_capacity(m);
            let mut high = Vec::with_capacity(m);
            let mut uprights = Vec::with_capacity(m);
            for arc in arcs {
                let (a, b) = (lift(arc.start, z0), lift(arc.start, z1));
                let (va, vb) = (id(), id());
                vertices.push(VertexRecord { id: va, point: a });
                vertices.push(VertexRecord { id: vb, point: b });
                let e = id();
                edges.push(EdgeRecord { id: e, curve: frag(make_line(a, b), "planar_upright")?, t0: 0.0, t1: 1.0, start_vertex_id: va, end_vertex_id: vb, degenerate: false, name: None });
                low.push(va);
                high.push(vb);
                uprights.push(e);
            }
            let mut bottom = Vec::new();
            let mut top = Vec::new();
            for (i, arc) in arcs.iter().enumerate() {
                let j = (i + 1) % m;
                let plane = arc_curve(region, arc)?;
                let at = |z: f64| -> Result<NurbsCurve, KernelRefusal> {
                    let points: Vec<Vec4> = plane
                        .control_points
                        .iter()
                        .map(|c| {
                            let p = c.point().unwrap_or(Vec3::new(0.0, 0.0, 0.0));
                            Vec4::from_point(lift([p.x, p.y], z), c.w)
                        })
                        .collect();
                    frag(NurbsCurve::new(plane.degree, plane.knots.clone(), points), "planar_lift")
                };
                let (cb, ct) = (at(z0)?, at(z1)?);
                let (eb, et) = (id(), id());
                edges.push(EdgeRecord { id: eb, curve: cb.clone(), t0: 0.0, t1: 1.0, start_vertex_id: low[i], end_vertex_id: low[j], degenerate: false, name: None });
                edges.push(EdgeRecord { id: et, curve: ct.clone(), t0: 0.0, t1: 1.0, start_vertex_id: high[i], end_vertex_id: high[j], degenerate: false, name: None });
                // The side: ruled between the bottom and top curves (u along
                // the arc, v up); outward is to the arc's right (the region
                // lies on its left).
                let net = vec![cb.control_points.clone(), ct.control_points.clone()];
                let side = frag(NurbsSurface::new(1, plane.degree, vec![0.0, 0.0, 1.0, 1.0], plane.knots.clone(), net), "planar_side")?;
                // Surface parameters (v along the arc, u up); the loop runs
                // bottom forward, upright j up, top backward, upright i down.
                let uv = |a: [f64; 2], b: [f64; 2]| frag(make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0)), "planar_pcurve");
                let coedges = vec![
                    (eb, true, uv([0.0, 0.0], [0.0, 1.0])?),
                    (uprights[j], true, uv([0.0, 1.0], [1.0, 1.0])?),
                    (et, false, uv([1.0, 1.0], [1.0, 0.0])?),
                    (uprights[i], false, uv([1.0, 0.0], [0.0, 0.0])?),
                ];
                let records: Vec<CoedgeRecord> = coedges.into_iter().map(|(edge, forward, pcurve)| CoedgeRecord { id: id(), edge_id: edge, forward, pcurve }).collect();
                // (u up, v along) × … : the surface's normal is ∂u × ∂v = up × along,
                // which points to the arc's left (inside): flip the sense.
                let loop_id = id();
                faces.push(FaceRecord { id: id(), surface: side, same_sense: false, loops: vec![LoopRecord { id: loop_id, coedges: records }], name: None });
                bottom.push((eb, plane.clone()));
                top.push((et, plane));
            }
            bottom_loops.push(bottom);
            top_loops.push(top);
        }
        // Caps: the plane's own coordinates as parameters; the bottom faces
        // down (its loops run backward), the top up.
        let corner = |z: f64| -> Result<NurbsSurface, KernelRefusal> {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for &l in &members {
                for arc in &loops[l] {
                    for i in 0..=64 {
                        let p = region.eval(arc.kind, arc.a + (arc.b - arc.a) * i as f64 / 64.0);
                        for k in 0..2 {
                            lo[k] = lo[k].min(p[k]);
                            hi[k] = hi[k].max(p[k]);
                        }
                    }
                }
            }
            let pad = 0.01 * region.scale;
            let (lo, hi) = ([lo[0] - pad, lo[1] - pad], [hi[0] + pad, hi[1] + pad]);
            let at = |x: f64, y: f64| Vec4::from_point(lift([x, y], z), 1.0);
            frag(
                NurbsSurface::new(1, 1, vec![lo[0], lo[0], hi[0], hi[0]], vec![lo[1], lo[1], hi[1], hi[1]], vec![vec![at(lo[0], lo[1]), at(lo[0], hi[1])], vec![at(hi[0], lo[1]), at(hi[0], hi[1])]]),
                "planar_cap",
            )
        };
        for (z, cap_loops, up) in [(z0, &bottom_loops, false), (z1, &top_loops, true)] {
            let surface = corner(z)?;
            let mut records = Vec::new();
            for coedges in cap_loops {
                let mut list: Vec<(u64, bool, NurbsCurve)> = coedges.iter().map(|(e, plane)| (*e, true, plane.clone())).collect();
                if !up {
                    list.reverse();
                    for c in &mut list {
                        c.1 = false;
                        c.2 = frag(c.2.reversed(), "planar_pcurve")?;
                    }
                }
                let coedge_records: Vec<CoedgeRecord> = list.into_iter().map(|(edge, forward, pcurve)| CoedgeRecord { id: id(), edge_id: edge, forward, pcurve }).collect();
                records.push(LoopRecord { id: id(), coedges: coedge_records });
            }
            faces.push(FaceRecord { id: id(), surface, same_sense: up, loops: records, name: None });
        }
        let edges_count = edges.len() as i64;
        // Each hole is a loop of both caps.
        let euler = vertices.len() as i64 - edges_count + faces.len() as i64 - 2 * holes.len() as i64;
        let shell = id();
        let solid_id = id();
        let body = BrepSolid {
            mass_properties_cache: Default::default(),
            id: solid_id,
            vertices,
            edges,
            shells: vec![ShellRecord { id: shell, faces }],
            genus: (2 - euler) / 2,
            kind: Default::default(),
            cells: Default::default(),
        };
        let issues = body.validate();
        if !issues.is_empty() {
            return Err(refuse("planar_assembly", format!("the extruded region does not validate: {:?}", issues.first())));
        }
        let built = frag(crate::solid_mass_properties(&body).map(|p| p.volume), "planar_volume")?;
        if (built - expected).abs() > 1e-6 * expected.abs().max(region.scale.powi(3) * 1e-6) {
            return Err(refuse("planar_volume", format!("the extruded region's volume {built:.9} disagrees with its area's {expected:.9}")));
        }
        solids.push(body);
    }
    Ok(solids)
}

/// The screw section (default on; `BREP_SWEEP_SCREW_SECTION=0` restores the level engine): under a screw about the clip's axis, a
/// straight-sided prism along that axis has as its reduced section the
/// region its outline covers turned by −z/c over its height (each slice at
/// height z lands on the reduced plane turned back by z/c), built here as a
/// planar region.
pub fn screw_section_on() -> bool {
    crate::switch!("BREP_SWEEP_SCREW_SECTION", true)
}

/// One boundary arc of the screw section: its points in the clip's plane, and
/// whether it is an arc about the axis (a constant level).
pub type SectionArc = (Vec<[f64; 2]>, bool);

/// The reduced section of the prism with `corners` (clip plane) between
/// heights `z` (along the clip's axis) under a screw of advance `c` per
/// radian, within `radius` of the axis: its loops of arcs. `None` where it is
/// not one region without holes (the level engine reads those).
pub fn screw_section_loops(corners: Vec<[f64; 2]>, z: [f64; 2], c: f64, radius: [f64; 2]) -> Result<Option<Vec<Vec<SectionArc>>>, KernelRefusal> {
    if !(c.is_finite() && c != 0.0 && z[1] > z[0] && radius[1] > radius[0] && radius[0] >= 0.0) || corners.len() < 3 {
        return Ok(None);
    }
    let polygon = Polygon { corners };
    let motion = Motion { turns: vec![([0.0, 0.0], -1.0 / c)], slides: vec![[0.0, 0.0]], t0: z[0], t1: z[1] };
    let clip = Clip { r: radius, angle: None };
    let corner_reach = polygon.corners.iter().map(|q| norm(*q)).fold(0.0f64, f64::max);
    let scale = corner_reach.max(radius[1].min(corner_reach));
    if scale <= 0.0 || radius[1] - radius[0] < 1e-6 * scale {
        return Ok(None);
    }
    let region = Region { motion: &motion, polygon: &polygon, clip: &clip, scale, target: None };
    let Some(loops) = region_loops(&region)? else { return Ok(None) };
    if loops.is_empty() || loops.iter().any(|l| loop_area(&region, l) <= 0.0) {
        return Ok(None);
    }
    let mut out = Vec::with_capacity(loops.len());
    for arcs in &loops {
        // A loft takes at least two curves a section: a loop of one arc (a
        // whole circle) is split at its middle.
        let halves: Vec<Arc>;
        let arcs: &[Arc] = if let [one] = arcs.as_slice() {
            let m = 0.5 * (one.a + one.b);
            let middle = region.eval(one.kind, m);
            halves = vec![
                Arc { kind: one.kind, a: one.a, b: m, start: one.start, end: middle },
                Arc { kind: one.kind, a: m, b: one.b, start: middle, end: one.end },
            ];
            &halves
        } else {
            arcs
        };
        let mut list = Vec::with_capacity(arcs.len());
        for arc in arcs {
            let level = matches!(arc.kind, Kind::Circle(_) | Kind::Orbit(_));
            // A level arc's samples step under a quarter turn, so the angle
            // unwrapped from them keeps its winding (a whole circle too).
            let count = if level { (((arc.b - arc.a).abs() / (0.25 * std::f64::consts::PI)).ceil() as usize).max(2) } else { 16 };
            let points: Vec<[f64; 2]> = (0..=count).map(|i| region.eval(arc.kind, arc.a + (arc.b - arc.a) * i as f64 / count as f64)).collect();
            list.push((points, level));
        }
        out.push(list);
    }
    Ok(Some(out))
}
