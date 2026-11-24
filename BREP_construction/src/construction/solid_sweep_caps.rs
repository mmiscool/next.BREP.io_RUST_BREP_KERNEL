//! The WHOLE sweep of a solid along a one-parameter motion, ends included:
//! caps and wall.
//!
//! Along a screw (a turn is a screw of zero advance, a slide one of zero turn)
//! the velocity field, read in the tool's own frame, does not change with time:
//! at a boundary point x with outward normal n it is
//! `g(x, n) = (ω e × (x − a) + c e) · n`. So the tool's boundary splits once
//! and for all into FRONT (g > 0) and BACK (g < 0) parts, and the GRAZING LOOP
//! K between them (g = 0 inside a face, or a sharp edge whose two faces have
//! opposite signs) is fixed on the tool. Where every point of space is inside
//! the moving tool for ONE interval of time, the sweep over [t0, t1] is bounded
//! by exactly
//!
//! - the back of the tool at t0 (where points start to be covered),
//! - the front of the tool at t1 (where they stop),
//! - the WALL: K carried by the motion from t0 to t1.
//!
//! The body is assembled directly from those faces with shared edges, never as
//! a boolean of the tool with a tube (the two meet tangentially along K).
//!
//! The one-interval condition is CHECKED, not assumed: points just outside
//! every built face are walked through the whole motion, and a sweep that
//! covers any of them (a fold, a buried cap, a turn running into itself) is
//! refused by name.
//!
//! Tools: a convex polyhedron (K a polygon of edges and face chords); a solid
//! of revolution whose faces are cylinders or cones (split along standing
//! generators, or along two grazing curves moving round them, fitted), flat
//! ends wholly front or back, a hemisphere from a pole to an equator or a whole
//! ball (split along a great circle), or a ring (torus, split along its two
//! grazing curves round it). Caps keep the tool's exact surfaces; walls are
//! the K curves carried by quintic splines through the exact motion and its
//! first two derivatives (error h⁶, 48 spans a turn), a row per quarter turn.
//!
//! Moving grazing loops: [`sweep_solid_along`] (a solid of revolution carried
//! by a frame along a planar curve) and [`sweep_solid_tumbled`] (two turns
//! about skew axes) split the tool at sample times, sample each piece of K
//! exactly and loft the walls through time; a tumble that passes over a point
//! twice is built from one-pass spans of its time, joined.
//! [`sweep_prism_along`] sweeps a prism along a frame by the plane region the
//! moving outline covers (an arrangement of its corners', grazing points' and
//! end positions' paths), extruded. [`sweep_solid_parts`] returns a folding
//! polyhedron's sweep as the sweeps of its halves through the motion's axis.
//!
//! [`sweep_solid_clipped`] adds the case the caller only needs inside a
//! region: a polyhedron TURNED about the region's axis with both end positions
//! outside the region's wedge fills the wedge with its (ρ, z) shadow turned,
//! bounded by exact conic arcs (edges) and segments (face chords).
use crate::solid_sweep::outline_signed_distance;
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
pub use crate::{make_line, make_revolution, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, OutlinePiece, SolidMotion, Vec3, Vec4};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// A tool the whole sweep takes.
#[derive(Clone, Debug)]
pub enum SolidTool {
    /// A convex polyhedron: its corners, and its faces as loops of corner
    /// indices (either winding; outward is found from the centroid).
    Polyhedron { vertices: Vec<Vec3>, faces: Vec<Vec<usize>> },
    /// A solid of revolution: the region `meridian` bounds in the
    /// half-plane spanned by `radial` (first coordinate, the distance from the
    /// axis, ≥ 0) and `axis_direction` (second coordinate), turned about the
    /// axis through `axis_origin` (as [`crate::SweptTool::Revolved`]).
    Revolved { axis_origin: Vec3, axis_direction: Vec3, radial: Vec3, meridian: Vec<OutlinePiece> },
}

/// Samples of a fitted pcurve: the image of its edge through the surface's
/// exact inverse, interpolated (quintic). A rational chart's parameter has a
/// second-derivative jump at its span joints, so the interpolant errs there
/// as h³; at 576 that is below 1e-10 of a face's area.
const PCURVE_SAMPLES: usize = 576;

/// The prefix of every refusal of this operation.
pub const SOLID_SWEEP_WHOLE_REFUSAL: &str = "sweepSolid";

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{SOLID_SWEEP_WHOLE_REFUSAL}: {message}"))
}

/// The stable slug of an unsupported-geometry refusal of this module.
fn refused_as(error: &KernelRefusal) -> Option<&str> {
    match &error.class {
        crate::RefusalClass::UnsupportedGeometry { what } => Some(what.as_str()),
        _ => None,
    }
}

fn input(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::input(KernelStage::Collect, what, format!("{SOLID_SWEEP_WHOLE_REFUSAL}: {message}"))
}

fn frag<T>(result: Result<T, String>, what: &'static str) -> Result<T, KernelRefusal> {
    result.or_refuse(KernelStage::Fragment, what)
}

/// The motion `x ↦ a + R(ω t)(x − a) + (c t + c₂ t²) e`: a screw (c₂ = 0;
/// a turn has c = 0, a slide ω = 0), or a turn whose rise grows as it turns.
#[derive(Clone, Copy, Debug)]
pub struct Screw {
    pub a: Vec3,
    pub e: Vec3,
    pub omega: f64,
    pub c: f64,
    pub c2: f64,
    pub t0: f64,
    pub t1: f64,
    /// A frame riding a curve instead (then `a` is the curve's start, `e` the
    /// frame's fixed up direction, `omega` a bound on its turning rate and `c`
    /// on its speed).
    pub frame: Option<Frame>,
    /// Two turns about fixed axes instead (then `a` is the first axis' point).
    pub turns: Option<Turns>,
    /// The time whose velocity field splits the tool (default: the middle).
    pub frozen: Option<f64>,
    /// How many spans a ring's grazing loop is sampled over, per half of the
    /// ring (`LOFT_SAMPLES` unless a loft found its rows too coarse).
    pub loop_samples: usize,
    /// How far the frame's origin travels from `a` (for speed bounds).
    pub extent: f64,
}

/// Two turns driven by one parameter: first `rates[0]·t` about the first
/// axis, then `rates[1]·t` about the second; both axes fixed in the world.
#[derive(Clone, Copy, Debug)]
pub struct Turns {
    points: [Vec3; 2],
    directions: [Vec3; 2],
    rates: [f64; 2],
}

impl Turns {
    fn apply(&self, t: f64, x: Vec3) -> Vec3 {
        let y = self.points[0].add(x.sub(self.points[0]).rotate_about(self.directions[0], self.rates[0] * t));
        self.points[1].add(y.sub(self.points[1]).rotate_about(self.directions[1], self.rates[1] * t))
    }
    fn invert(&self, t: f64, y: Vec3) -> Vec3 {
        let x = self.points[1].add(y.sub(self.points[1]).rotate_about(self.directions[1], -self.rates[1] * t));
        self.points[0].add(x.sub(self.points[0]).rotate_about(self.directions[0], -self.rates[0] * t))
    }
    /// Velocity of the tool point x at t, read back in the tool's frame.
    fn velocity(&self, t: f64, x: Vec3) -> Vec3 {
        let first = x.sub(self.points[0]).rotate_about(self.directions[0], self.rates[0] * t);
        let y = self.points[0].add(first);
        let moved = y.sub(self.points[1]).rotate_about(self.directions[1], self.rates[1] * t);
        let inner = self.directions[0].cross(first).scale(self.rates[0]);
        let world = self.directions[1].cross(moved).scale(self.rates[1]).add(inner.rotate_about(self.directions[1], self.rates[1] * t));
        world.rotate_about(self.directions[1], -self.rates[1] * t).rotate_about(self.directions[0], -self.rates[0] * t)
    }
}

/// A frame riding a planar cubic Bézier curve: origin on the curve, first
/// axis along its tangent, a fixed up direction square to its plane.
#[derive(Clone, Copy, Debug)]
pub struct Frame {
    pub control: [Vec3; 4],
    pub up: Vec3,
    /// The tangent's direction at the start.
    pub start_tangent: Vec3,
}

impl Frame {
    fn point(&self, s: f64) -> Vec3 {
        let [p0, p1, p2, p3] = self.control;
        let r = 1.0 - s;
        p0.scale(r * r * r).add(p1.scale(3.0 * r * r * s)).add(p2.scale(3.0 * r * s * s)).add(p3.scale(s * s * s))
    }
    fn first(&self, s: f64) -> Vec3 {
        let [p0, p1, p2, p3] = self.control;
        let r = 1.0 - s;
        p1.sub(p0).scale(3.0 * r * r).add(p2.sub(p1).scale(6.0 * r * s)).add(p3.sub(p2).scale(3.0 * s * s))
    }
    fn second(&self, s: f64) -> Vec3 {
        let [p0, p1, p2, p3] = self.control;
        p2.sub(p1.scale(2.0)).add(p0).scale(6.0 * (1.0 - s)).add(p3.sub(p2.scale(2.0)).add(p1).scale(6.0 * s))
    }
    /// The frame's turn from its start, about `up`.
    fn angle(&self, s: f64) -> f64 {
        let t = self.first(s);
        self.up.dot(self.start_tangent.cross(t)).atan2(self.start_tangent.dot(t))
    }
    fn angle_rate(&self, s: f64) -> f64 {
        let (d1, d2) = (self.first(s), self.second(s));
        self.up.dot(d1.cross(d2)) / d1.dot(d1)
    }
}

/// A turn about an axis whose rise along it grows as it turns: turned by t
/// (radians, right-handed about `axis_direction`) the tool has risen
/// `rise[0] t + rise[1] t²`, for t from `from` to `to` (a variable-lead
/// screw).
#[derive(Clone, Copy, Debug)]
pub struct RisingTurn {
    pub axis_origin: Vec3,
    pub axis_direction: Vec3,
    pub rise: [f64; 2],
    pub from: f64,
    pub to: f64,
}

impl Screw {
    pub fn from_motion(motion: &SolidMotion) -> Result<Self, KernelRefusal> {
        let [step] = motion.steps.as_slice() else {
            return Err(refuse("motion_steps", "only a single screw, turn or slide is swept whole here".into()));
        };
        let e = step
            .axis_direction
            .normalized()
            .map_err(|_| input("axis", "the motion's axis has no direction".into()))?;
        if !(motion.from.is_finite() && motion.to.is_finite() && motion.to > motion.from) {
            return Err(input("range", "the motion's range must run forward".into()));
        }
        if step.turn == 0.0 && step.advance == 0.0 {
            return Err(input("still", "the motion does not move".into()));
        }
        Ok(Self { a: step.axis_origin, e, omega: step.turn, c: step.advance, c2: 0.0, t0: motion.from, t1: motion.to, frame: None, turns: None, frozen: None,
        loop_samples: LOFT_SAMPLES, extent: 0.0 })
    }

    fn from_rising(motion: &RisingTurn) -> Result<Self, KernelRefusal> {
        let e = motion.axis_direction.normalized().map_err(|_| input("axis", "the motion's axis has no direction".into()))?;
        if !(motion.from.is_finite() && motion.to.is_finite() && motion.to > motion.from) {
            return Err(input("range", "the motion's range must run forward".into()));
        }
        // The rise's rate must keep one sign, or the tool's front and back swap.
        let rate = |t: f64| motion.rise[0] + 2.0 * motion.rise[1] * t;
        if rate(motion.from) * rate(motion.to) <= 0.0 {
            return Err(refuse("rise_turns", "the rise stops or turns back within the motion".into()));
        }
        Ok(Self { a: motion.axis_origin, e, omega: 1.0, c: motion.rise[0], c2: motion.rise[1], t0: motion.from, t1: motion.to, frame: None, turns: None, frozen: None,
        loop_samples: LOFT_SAMPLES, extent: 0.0 })
    }

    fn advance(&self, t: f64) -> f64 {
        self.c * t + self.c2 * t * t
    }

    /// The fastest the motion advances along its axis.
    fn rate_bound(&self) -> f64 {
        (self.c + 2.0 * self.c2 * self.t0).abs().max((self.c + 2.0 * self.c2 * self.t1).abs())
    }

    fn apply(&self, t: f64, x: Vec3) -> Vec3 {
        if let Some(turns) = &self.turns {
            return turns.apply(t, x);
        }
        if let Some(frame) = &self.frame {
            return frame.point(t).add(x.sub(self.a).rotate_about(frame.up, frame.angle(t)));
        }
        self.a.add(x.sub(self.a).rotate_about(self.e, self.omega * t)).add(self.e.scale(self.advance(t)))
    }

    fn invert(&self, t: f64, y: Vec3) -> Vec3 {
        if let Some(turns) = &self.turns {
            return turns.invert(t, y);
        }
        if let Some(frame) = &self.frame {
            return self.a.add(y.sub(frame.point(t)).rotate_about(frame.up, -frame.angle(t)));
        }
        self.a.add(y.sub(self.e.scale(self.advance(t))).sub(self.a).rotate_about(self.e, -self.omega * t))
    }

    /// Velocity, in the tool's frame, of the tool point x (the same at every t
    /// for a screw; for a growing rise, at the middle of the motion; the split
    /// it gives is checked against both ends).
    pub fn velocity(&self, x: Vec3) -> Vec3 {
        self.velocity_at(self.frozen.unwrap_or(0.5 * (self.t0 + self.t1)), x)
    }

    fn velocity_at(&self, t: f64, x: Vec3) -> Vec3 {
        if let Some(turns) = &self.turns {
            return turns.velocity(t, x);
        }
        if let Some(frame) = &self.frame {
            // In the riding frame: the speed along the start tangent and the
            // turn about up through the frame's origin.
            return frame.start_tangent.scale(frame.first(t).length()).add(frame.up.cross(x.sub(self.a)).scale(frame.angle_rate(t)));
        }
        self.e.cross(x.sub(self.a)).scale(self.omega).add(self.e.scale(self.c + 2.0 * self.c2 * t))
    }

    /// Value, first and second t-derivative of the path of the tool point x.
    fn jet(&self, t: f64, x: Vec3) -> (Vec3, Vec3, Vec3) {
        let r = x.sub(self.a).rotate_about(self.e, self.omega * t);
        let axial = self.e.scale(r.dot(self.e));
        let radial = r.sub(axial);
        let value = self.a.add(r).add(self.e.scale(self.advance(t)));
        let first = self.e.cross(radial).scale(self.omega).add(self.e.scale(self.c + 2.0 * self.c2 * t));
        let second = radial.scale(-self.omega * self.omega).add(self.e.scale(2.0 * self.c2));
        (value, first, second)
    }

    fn move_point(&self, t: f64, p: &Vec4) -> Result<Vec4, KernelRefusal> {
        let x = frag(p.point(), "control_point")?;
        Ok(Vec4::from_point(self.apply(t, x), p.w))
    }

    fn move_curve(&self, t: f64, curve: &NurbsCurve) -> Result<NurbsCurve, KernelRefusal> {
        let points = curve.control_points.iter().map(|p| self.move_point(t, p)).collect::<Result<Vec<_>, _>>()?;
        frag(NurbsCurve::new(curve.degree, curve.knots.clone(), points), "moved_curve")
    }

    fn move_surface(&self, t: f64, surface: &NurbsSurface) -> Result<NurbsSurface, KernelRefusal> {
        let mut rows = Vec::with_capacity(surface.control_points.len());
        for row in &surface.control_points {
            rows.push(row.iter().map(|p| self.move_point(t, p)).collect::<Result<Vec<_>, _>>()?);
        }
        frag(
            NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), surface.knots_v.clone(), rows),
            "moved_surface",
        )
    }
}

/// The motion's times at the ends of its rows: each row of wall turns at most
/// a quarter turn (long thin faces cost every later reading, and the cut).
fn row_times(screw: &Screw) -> Vec<f64> {
    let turned = (screw.omega * (screw.t1 - screw.t0)).abs();
    let rows = ((turned / (0.5 * PI)).ceil() as usize).max(1);
    // Breaks off the round fractions of the range, where a caller's workpiece
    // faces tend to lie (a precaution: an edge lying in a face makes a
    // degenerate cut).
    let at = |k: usize| match k {
        0 => screw.t0,
        k if k == rows => screw.t1,
        k => screw.t0 + (screw.t1 - screw.t0) * (k as f64 - 0.381966011250105) / rows as f64,
    };
    (0..=rows).map(at).collect()
}

/// Spans of the quintic carry over [ta, tb]: 48 per turn (error ≈ r·h⁶/46080
/// ≈ 1e-10·r), at least 4.
fn carry_segments(screw: &Screw, ta: f64, tb: f64) -> usize {
    let turned = (screw.omega * (tb - ta)).abs();
    ((turned / TAU * 48.0).ceil() as usize).max(4)
}

/// Knots of the C² quintic carry on [0, 1].
fn carry_knots(segments: usize) -> Vec<f64> {
    let mut knots = vec![0.0; 6];
    for k in 1..segments {
        knots.extend([k as f64 / segments as f64; 3]);
    }
    knots.extend([1.0; 6]);
    knots
}

/// Control points of the C² quintic spline through the exact path of the tool
/// point x, its first and second derivatives at `segments + 1` times. Affine
/// in x, so carrying a rational curve's control points carries the curve.
fn carry_points(screw: &Screw, x: Vec3, ta: f64, tb: f64, segments: usize) -> Vec<Vec3> {
    let h = (tb - ta) / segments as f64;
    let at = |k: usize| screw.jet(ta + h * k as f64, x);
    let piece = |k: usize| {
        let ((f0, d0, a0), (f1, d1, a1)) = (at(k), at(k + 1));
        [
            f0,
            f0.add(d0.scale(h / 5.0)),
            f0.add(d0.scale(2.0 * h / 5.0)).add(a0.scale(h * h / 20.0)),
            f1.sub(d1.scale(2.0 * h / 5.0)).add(a1.scale(h * h / 20.0)),
            f1.sub(d1.scale(h / 5.0)),
            f1,
        ]
    };
    let mut points = Vec::with_capacity(3 * segments + 3);
    let first = piece(0);
    points.extend_from_slice(&first[..3]);
    for k in 0..segments - 1 {
        let (here, next) = (piece(k), piece(k + 1));
        points.push(here[3]);
        points.push(here[4].scale(2.0).sub(here[3]));
        points.push(next[2]);
    }
    let last = piece(segments - 1);
    points.extend_from_slice(&last[3..]);
    points
}

fn carry_curve(screw: &Screw, x: Vec3, ta: f64, tb: f64, segments: usize) -> Result<NurbsCurve, KernelRefusal> {
    frag(
        NurbsCurve::new(5, carry_knots(segments), carry_points(screw, x, ta, tb, segments).into_iter().map(|p| Vec4::from_point(p, 1.0)).collect()),
        "carry_curve",
    )
}

/// A boundary curve of the tool (in its own frame), from point `start` to
/// point `end` along its parameter.
struct ToolCurve {
    curve: NurbsCurve,
    start: usize,
    end: usize,
    degenerate: bool,
    /// Exact points at `LOFT_SAMPLES + 1` even steps of a natural parameter
    /// from start to end (meridian place, angle), where a grazing curve
    /// moves on the tool and its wall is lofted through time.
    samples: Option<Vec<Vec3>>,
}

/// Steps of a grazing curve's exact samples for a lofted wall.
pub const LOFT_SAMPLES: usize = 128;

/// How near a lofted wall keeps to its exact grazing loop, as a fraction of
/// the sweep's size: every exact sample of the loop, in the middle of every
/// time span, is within this of the wall.
///
/// It was 5e-10. A wall that tight is heavy (a quintic's miss falls as the
/// sixth power of its span, so each tenfold costs half again as many spans
/// each way), and whoever reads the solid pays for every span. The measures
/// do not need it: an interpolant's miss changes sign from span to span and
/// all but cancels in a volume or an area. Over this hundredfold the lofted
/// cases' volumes and areas moved by 2e-10 or less.
const LOFT_TARGET: f64 = 5e-8;

/// A face of the tool after the split along K: its exact surface (normal
/// outward), its side, and its loop (curve, traversed forward?, pcurve in the
/// direction of traversal).
struct Cap {
    surface: NurbsSurface,
    front: bool,
    coedges: Vec<(usize, bool, NurbsCurve)>,
    /// The surface's exact parameters of a point (in the tool's frame), where
    /// known: a lofted edge's pcurve is then its exact image.
    chart: Option<Chart>,
}

/// How a cap's surface is parameterized, to invert it exactly.
#[derive(Clone, Copy, Debug)]
enum Chart {
    /// `plane_patch(origin, u, v, …)`: (x − origin)·u, (x − origin)·v.
    Plane { origin: Vec3, u: Vec3, v: Vec3 },
    /// `make_revolution(origin, w, generatrix, theta)` with the generatrix in the
    /// meridian half-plane at angle `alpha` (from u1 towards u2).
    Revolution { origin: Vec3, w: Vec3, u1: Vec3, u2: Vec3, alpha: f64, theta: f64, spans_u: usize, meridian: MeridianChart },
}

#[derive(Clone, Copy, Debug)]
enum MeridianChart {
    /// A segment from m0 to m1 in (ρ, z), parameter 0 -> 1.
    Line { m0: [f64; 2], m1: [f64; 2] },
    /// An arc about `centre` in (ρ, z) from angle `start` through `sweep`, as
    /// `make_revolution` of a point makes it (`spans` arcs).
    Arc { centre: [f64; 2], start: f64, sweep: f64, spans: usize },
}

impl Chart {
    /// Is `x` on the chart's own axis, where its first parameter (the angle
    /// about the axis) is not defined? A ball's pole.
    fn at_pole(&self, x: Vec3) -> bool {
        match *self {
            Chart::Plane { .. } => false,
            Chart::Revolution { origin, w, .. } => {
                let r = x.sub(origin);
                r.sub(w.scale(r.dot(w))).length() <= 1e-6 * r.length()
            }
        }
    }

    /// The chart's span joints in each parameter (`make_revolution`'s arcs
    /// in equal spans of [0, 1]), where its parameter's second derivative
    /// jumps along the surface.
    fn joints(&self) -> (Vec<f64>, Vec<f64>) {
        let inner = |spans: usize| (1..spans).map(|k| k as f64 / spans as f64).collect::<Vec<f64>>();
        match *self {
            Chart::Plane { .. } => (Vec::new(), Vec::new()),
            Chart::Revolution { spans_u, meridian, .. } => (
                inner(spans_u),
                match meridian {
                    MeridianChart::Arc { spans, .. } => inner(spans),
                    MeridianChart::Line { .. } => Vec::new(),
                },
            ),
        }
    }

    fn invert(&self, x: Vec3) -> [f64; 2] {
        match *self {
            Chart::Plane { origin, u, v } => {
                let d = x.sub(origin);
                [d.dot(u), d.dot(v)]
            }
            Chart::Revolution { origin, w, u1, u2, alpha, theta, spans_u, meridian } => {
                let r = x.sub(origin);
                let z = r.dot(w);
                let rho = r.sub(w.scale(z)).length();
                let mid = alpha + 0.5 * theta;
                let psi = mid + (r.dot(u2).atan2(r.dot(u1)) - mid + PI).rem_euclid(TAU) - PI;
                let u = crate::circle_angle_to_parameter(spans_u, theta, psi - alpha);
                let v = match meridian {
                    MeridianChart::Line { m0, m1 } => {
                        let (a, b) = (m1[0] - m0[0], m1[1] - m0[1]);
                        ((rho - m0[0]) * a + (z - m0[1]) * b) / (a * a + b * b)
                    }
                    MeridianChart::Arc { centre, start, sweep, spans } => {
                        let mid = start + 0.5 * sweep;
                        let angle = mid + ((z - centre[1]).atan2(rho - centre[0]) - mid + PI).rem_euclid(TAU) - PI;
                        crate::circle_angle_to_parameter(spans, sweep, angle - start)
                    }
                };
                [u, v]
            }
        }
    }
}

/// The spans `make_revolution` gives a sweep of `theta`.
fn spans_of(surface_knots_u: &[f64]) -> usize {
    (surface_knots_u.len() - 6) / 2 + 1
}

/// The tool's boundary split along K, in the tool's own frame.
struct Split {
    points: Vec<Vec3>,
    curves: Vec<ToolCurve>,
    caps: Vec<Cap>,
    /// Flat ends that slide within their own plane all through the motion.
    slides: Vec<SlideDisc>,
    /// Cylinders that slide along their own generators all through it.
    slide_sides: Vec<SlideSide>,
    scale: f64,
    shape: Shape,
}

/// A flat end about the tool's axis whose every point moves within the
/// end's own plane (a turn about a parallel axis, a slide across the tool's
/// axis): neither front nor back. What it sweeps is one face of the sweep in
/// that plane, bounded by the rim's arcs beside back faces where the motion
/// starts, its arcs beside front faces where it stops, and the paths of the
/// points between them. `rim`: the rim's arcs counter-clockwise about the
/// outward normal `u × v`, each with its direction of traversal.
struct SlideDisc {
    centre: Vec3,
    u: Vec3,
    v: Vec3,
    rim: Vec<(usize, bool)>,
}

/// A cylinder about the tool's axis slid along that axis: every point moves
/// along its own generator. What it sweeps is the cylinder from one rim where
/// that rim's neighbour sits (back at the start, front at the end) to the
/// other rim where its neighbour sits. `patches`: per stretch between two
/// seam angles, (the angle it starts at, its sweep, the start rim's arc, the
/// end rim's arc, the rim points at the start angle: start rim, end rim).
struct SlideSide {
    origin: Vec3,
    w: Vec3,
    patches: Vec<(f64, f64, usize, usize, [usize; 2])>,
}

/// What the membership oracle measures.
enum Shape {
    Planes(Vec<(Vec3, f64)>),
    Revolved { origin: Vec3, axis: Vec3, meridian: Vec<OutlinePiece> },
}

impl Split {
    /// Signed distance to the tool in its own frame (negative inside).
    fn distance(&self, x: Vec3) -> f64 {
        match &self.shape {
            Shape::Planes(planes) => planes.iter().map(|(n, d)| n.dot(x) - d).fold(f64::NEG_INFINITY, f64::max),
            Shape::Revolved { origin, axis, meridian } => {
                let r = x.sub(*origin);
                let z = r.dot(*axis);
                let rho = r.sub(axis.scale(z)).length();
                outline_signed_distance(meridian, [rho, z])
            }
        }
    }

    /// K: each curve with a front cap on one side and a back cap on the
    /// other, with the direction in which the back cap runs along it (the back
    /// on its left seen from outside).
    fn grazing(&self) -> Result<Vec<(usize, bool)>, KernelRefusal> {
        let mut uses: HashMap<usize, Vec<(bool, bool)>> = HashMap::new();
        for cap in &self.caps {
            for &(curve, forward, _) in &cap.coedges {
                uses.entry(curve).or_default().push((forward, cap.front));
            }
        }
        let mut grazing = Vec::new();
        for (curve, list) in uses {
            if self.curves[curve].degenerate {
                continue;
            }
            // A sliding end's rim carries no wall: it moves within the end's plane.
            if list.len() == 1
                && (self.slides.iter().any(|slide| slide.rim.iter().any(|&(c, _)| c == curve))
                    || self.slide_sides.iter().any(|side| side.patches.iter().any(|patch| patch.2 == curve || patch.3 == curve)))
            {
                continue;
            }
            match list.as_slice() {
                [(fa, front_a), (fb, front_b)] => {
                    if fa == fb {
                        return Err(refuse("split_topology", "two caps run one boundary curve the same way".into()));
                    }
                    if front_a != front_b {
                        grazing.push((curve, if *front_a { *fb } else { *fa }));
                    }
                }
                _ => return Err(refuse("split_topology", format!("a boundary curve is used {} times", list.len()))),
            }
        }
        grazing.sort_unstable();
        Ok(grazing)
    }
}

fn uv_line(a: [f64; 2], b: [f64; 2]) -> Result<NurbsCurve, KernelRefusal> {
    frag(make_line(Vec3::new(a[0], a[1], 0.0), Vec3::new(b[0], b[1], 0.0)), "pcurve")
}

/// The plane through `origin` with in-plane axes u, v (u × v outward), as a
/// patch covering `points`; and the map to its parameters.
fn plane_patch(origin: Vec3, u: Vec3, v: Vec3, points: &[Vec3]) -> Result<NurbsSurface, KernelRefusal> {
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in points {
        let q = [p.sub(origin).dot(u), p.sub(origin).dot(v)];
        for k in 0..2 {
            lo[k] = lo[k].min(q[k]);
            hi[k] = hi[k].max(q[k]);
        }
    }
    let at = |s: f64, t: f64| Vec4::from_point(origin.add(u.scale(s)).add(v.scale(t)), 1.0);
    frag(
        NurbsSurface::new(
            1,
            1,
            vec![lo[0], lo[0], hi[0], hi[0]],
            vec![lo[1], lo[1], hi[1], hi[1]],
            vec![vec![at(lo[0], lo[1]), at(lo[0], hi[1])], vec![at(hi[0], lo[1]), at(hi[0], hi[1])]],
        ),
        "plane_surface",
    )
}

/// A curve lying in the plane (origin, u, v), as a pcurve of that plane:
/// its control points projected, weights kept (exact, the map is affine).
fn plane_pcurve(origin: Vec3, u: Vec3, v: Vec3, curve: &NurbsCurve, forward: bool) -> Result<NurbsCurve, KernelRefusal> {
    let mut points = Vec::with_capacity(curve.control_points.len());
    for p in &curve.control_points {
        let x = frag(p.point(), "control_point")?.sub(origin);
        points.push(Vec4::from_point(Vec3::new(x.dot(u), x.dot(v), 0.0), p.w));
    }
    let pcurve = frag(NurbsCurve::new(curve.degree, curve.knots.clone(), points), "plane_pcurve")?;
    if forward { Ok(pcurve) } else { frag(pcurve.reversed(), "plane_pcurve") }
}

fn split_polyhedron(screw: &Screw, vertices: &[Vec3], faces: &[Vec<usize>]) -> Result<Split, KernelRefusal> {
    if vertices.len() < 4 || faces.len() < 4 {
        return Err(input("polyhedron", "a polyhedron needs at least four corners and four faces".into()));
    }
    let centroid = vertices.iter().fold(Vec3::new(0.0, 0.0, 0.0), |s, &p| s.add(p)).scale(1.0 / vertices.len() as f64);
    let scale = vertices.iter().map(|p| p.sub(centroid).length()).fold(0.0f64, f64::max).max(1e-300);
    let tolerance = 1e-12 * scale.max(screw.a.sub(centroid).length());
    let mut loops: Vec<Vec<usize>> = Vec::with_capacity(faces.len());
    let mut planes = Vec::with_capacity(faces.len());
    for face in faces {
        if face.len() < 3 || face.iter().any(|&i| i >= vertices.len()) {
            return Err(input("face", "a face needs three or more existing corners".into()));
        }
        let mut normal = Vec3::new(0.0, 0.0, 0.0);
        for k in 0..face.len() {
            let (p, q) = (vertices[face[k]], vertices[face[(k + 1) % face.len()]]);
            normal = normal.add(p.sub(centroid).cross(q.sub(centroid)));
        }
        let mut normal = normal.normalized().map_err(|_| input("face", "a face has no area".into()))?;
        let mut face = face.clone();
        if normal.dot(vertices[face[0]].sub(centroid)) < 0.0 {
            normal = normal.scale(-1.0);
            face.reverse();
        }
        let offset = normal.dot(vertices[face[0]]);
        for p in vertices {
            if normal.dot(*p) - offset > 1e-9 * scale {
                return Err(refuse("not_convex", "the polyhedron is not convex".into()));
            }
        }
        for &i in &face {
            if (normal.dot(vertices[i]) - offset).abs() > 1e-9 * scale {
                return Err(input("face", "a face's corners are not in one plane".into()));
            }
        }
        loops.push(face);
        planes.push((normal, offset));
    }
    // Each face's grazing value (affine along the face).
    let g = |face: usize, x: Vec3| -> f64 {
        let value = screw.velocity(x).dot(planes[face].0);
        if value.abs() <= tolerance * screw.velocity(x).length().max(1.0) { 0.0 } else { value }
    };
    let mut owner: HashMap<(usize, usize), usize> = HashMap::new();
    for (index, face) in loops.iter().enumerate() {
        for k in 0..face.len() {
            owner.insert((face[k], face[(k + 1) % face.len()]), index);
        }
    }
    for &(i, j) in owner.keys() {
        if !owner.contains_key(&(j, i)) {
            return Err(input("closed", "the polyhedron's faces do not close".into()));
        }
    }
    // Split points on edges: where either neighbour's grazing value crosses zero.
    let mut points: Vec<Vec3> = vertices.to_vec();
    let mut edge_splits: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    let mut keys: Vec<(usize, usize)> = owner.keys().copied().filter(|&(i, j)| i < j).collect();
    keys.sort_unstable();
    for (i, j) in keys {
        let mut params: Vec<f64> = Vec::new();
        for f in [owner[&(i, j)], owner[&(j, i)]] {
            let (gi, gj) = (g(f, vertices[i]), g(f, vertices[j]));
            if gi * gj < 0.0 {
                params.push(gi / (gi - gj));
            }
        }
        params.sort_by(|a, b| a.total_cmp(b));
        params.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
        let mut list = Vec::new();
        for s in params {
            points.push(vertices[i].add(vertices[j].sub(vertices[i]).scale(s)));
            list.push(points.len() - 1);
        }
        edge_splits.insert((i, j), list);
    }
    let splits_along = |i: usize, j: usize| -> Vec<usize> {
        if i < j {
            edge_splits[&(i, j)].clone()
        } else {
            edge_splits[&(j, i)].iter().rev().copied().collect()
        }
    };
    // Sub-faces as point rings, counter-clockwise from outside.
    let mut rings: Vec<(Vec<usize>, usize, bool)> = Vec::new();
    for (index, face) in loops.iter().enumerate() {
        let mut ring: Vec<usize> = Vec::new();
        for k in 0..face.len() {
            ring.push(face[k]);
            ring.extend(splits_along(face[k], face[(k + 1) % face.len()]));
        }
        let values: Vec<f64> = ring.iter().map(|&p| g(index, points[p])).collect();
        let positive = values.iter().any(|&v| v > 0.0);
        let negative = values.iter().any(|&v| v < 0.0);
        if !positive && !negative {
            return Err(refuse("face_in_own_plane", format!("face {index} moves in its own plane (its sweep is flat)")));
        }
        if !(positive && negative) {
            rings.push((ring, index, positive));
            continue;
        }
        let zeros: Vec<usize> = (0..ring.len()).filter(|&k| values[k] == 0.0).collect();
        if zeros.len() != 2 {
            return Err(refuse("grazing_split", format!("face {index} meets its grazing line {} times", zeros.len())));
        }
        let (za, zb) = (zeros[0], zeros[1]);
        if za + 1 == zb || (zb + 1) % ring.len() == za {
            return Err(refuse("grazing_split", format!("face {index} is split along one of its own edges")));
        }
        let first: Vec<usize> = (za..=zb).map(|k| ring[k]).collect();
        let second: Vec<usize> = (zb..ring.len()).chain(0..=za).map(|k| ring[k]).collect();
        let first_front = values[za + 1] > 0.0;
        rings.push((first, index, first_front));
        rings.push((second, index, !first_front));
    }
    // A growing rise moves the grazing lines: every sub-face must keep its side
    // at both ends of the motion.
    if screw.c2 != 0.0 {
        for (ring, face, front) in &rings {
            for &p in ring {
                for t in [screw.t0, screw.t1] {
                    let value = screw.velocity_at(t, points[p]).dot(planes[*face].0);
                    let slack = tolerance * screw.velocity_at(t, points[p]).length().max(1.0);
                    if (*front && value < -slack) || (!*front && value > slack) {
                        return Err(refuse("rising_sides", format!("face {face} changes side as the rise grows")));
                    }
                }
            }
        }
    }
    // Curves: one straight segment per point pair.
    let mut curves: Vec<ToolCurve> = Vec::new();
    let mut by_pair: HashMap<(usize, usize), usize> = HashMap::new();
    let mut caps = Vec::with_capacity(rings.len());
    for (ring, face, front) in rings {
        let normal = planes[face].0;
        let origin = points[ring[0]];
        let u = frag(points[ring[1]].sub(origin).normalized(), "plane_axis")?;
        let v = normal.cross(u);
        let corners: Vec<Vec3> = ring.iter().map(|&p| points[p]).collect();
        let surface = plane_patch(origin, u, v, &corners)?;
        let mut coedges = Vec::with_capacity(ring.len());
        for k in 0..ring.len() {
            let (p, q) = (ring[k], ring[(k + 1) % ring.len()]);
            let key = (p.min(q), p.max(q));
            let index = match by_pair.get(&key) {
                Some(&index) => index,
                None => {
                    curves.push(ToolCurve {
                        curve: frag(make_line(points[key.0], points[key.1]), "line")?,
                        start: key.0,
                        end: key.1,
                        degenerate: false,
                        samples: None,
                    });
                    by_pair.insert(key, curves.len() - 1);
                    curves.len() - 1
                }
            };
            let forward = p == key.0;
            coedges.push((index, forward, plane_pcurve(origin, u, v, &curves[index].curve, forward)?));
        }
        caps.push(Cap { surface, front, coedges, chart: Some(Chart::Plane { origin, u, v }) });
    }
    Ok(Split { points, curves, caps, slides: Vec::new(), slide_sides: Vec::new(), scale, shape: Shape::Planes(planes) })
}

/// The grazing value `A cos ψ + B sin ψ + C` of a face of revolution at one
/// meridian point, from three samples.
pub fn harmonic(g: impl Fn(f64) -> f64) -> (f64, f64, f64) {
    let (g0, g90, g180) = (g(0.0), g(0.5 * PI), g(PI));
    let c = 0.5 * (g0 + g180);
    (0.5 * (g0 - g180), g90 - c, c)
}

pub fn piece_point(piece: &OutlinePiece, s: f64) -> [f64; 2] {
    match *piece {
        OutlinePiece::Line { from, to } => [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s],
        OutlinePiece::Arc { centre, radius, start, end } => {
            let angle = start + (end - start) * s;
            [centre[0] + radius * angle.cos(), centre[1] + radius * angle.sin()]
        }
        OutlinePiece::Ellipse { centre, semi, rotation, start, end } => crate::solid_sweep::ellipse_point(centre, semi, rotation, start + (end - start) * s),
    }
}

/// The whole sweep takes meridians of lines and circular arcs; an elliptic
/// piece is swept only by [`crate::sweep_solid_within`]'s region engine.
fn refuse_elliptic(meridian: &[OutlinePiece]) -> Result<(), KernelRefusal> {
    if meridian.iter().any(|piece| matches!(piece, OutlinePiece::Ellipse { .. })) {
        return Err(refuse("revolved_ellipse", "a meridian with an elliptic piece is not swept whole here".into()));
    }
    Ok(())
}

fn split_revolved(
    screw: &Screw,
    origin: Vec3,
    axis: Vec3,
    radial: Vec3,
    meridian: &[OutlinePiece],
) -> Result<Split, KernelRefusal> {
    let w = axis.normalized().map_err(|_| input("axis", "the tool's axis has no direction".into()))?;
    let u1 = radial.sub(w.scale(radial.dot(w))).normalized().map_err(|_| input("radial", "the radial direction lies along the axis".into()))?;
    let u2 = w.cross(u1);
    refuse_elliptic(meridian)?;
    if let [OutlinePiece::Arc { centre, radius, start, end }] = meridian {
        if ((end - start).abs() - TAU).abs() < 1e-12 && centre[0] > *radius {
            return split_torus(screw, origin, w, u1, u2, *centre, *radius, meridian);
        }
    }
    let at = |rho: f64, z: f64, psi: f64| origin.add(w.scale(z)).add(u1.scale(rho * psi.cos())).add(u2.scale(rho * psi.sin()));
    let scale = meridian
        .iter()
        .flat_map(|p| [piece_point(p, 0.0), piece_point(p, 1.0)])
        .map(|q| q[0].abs().max(q[1].abs()))
        .fold(0.0f64, f64::max)
        .max(1e-300);
    let on_axis = |q: [f64; 2]| q[0].abs() <= 1e-12 * scale;
    // Orientation of the meridian outline (+1 counter-clockwise in (ρ, z)).
    let mut area = 0.0;
    for piece in meridian {
        for k in 0..16 {
            let (p, q) = (piece_point(piece, k as f64 / 16.0), piece_point(piece, (k + 1) as f64 / 16.0));
            area += p[0] * q[1] - q[0] * p[1];
        }
    }
    // Work counter-clockwise in (ρ, z): a revolved patch's u × v is then outward.
    let reversed_meridian: Vec<OutlinePiece>;
    let meridian: &[OutlinePiece] = if area > 0.0 {
        meridian
    } else {
        reversed_meridian = meridian
            .iter()
            .rev()
            .map(|piece| match *piece {
                OutlinePiece::Line { from, to } => OutlinePiece::Line { from: to, to: from },
                OutlinePiece::Arc { centre, radius, start, end } => OutlinePiece::Arc { centre, radius, start: end, end: start },
                OutlinePiece::Ellipse { centre, semi, rotation, start, end } => OutlinePiece::Ellipse { centre, semi, rotation, start: end, end: start },
            })
            .collect();
        &reversed_meridian
    };
    let ccw = true;
    // Outward normal in (ρ, z) of a piece at s.
    let normal = |piece: &OutlinePiece, s: f64| -> [f64; 2] {
        let (p, q) = (piece_point(piece, (s - 1e-6).max(0.0)), piece_point(piece, (s + 1e-6).min(1.0)));
        let (dx, dz) = (q[0] - p[0], q[1] - p[1]);
        let length = dx.hypot(dz);
        if ccw { [dz / length, -dx / length] } else { [-dz / length, dx / length] }
    };
    let g = |piece: &OutlinePiece, s: f64, psi: f64| -> f64 {
        let m = piece_point(piece, s);
        let n = normal(piece, s);
        let x = at(m[0], m[1], psi);
        let normal = w.scale(n[1]).add(u1.scale(n[0] * psi.cos())).add(u2.scale(n[0] * psi.sin()));
        screw.velocity(x).dot(normal)
    };
    // The faces: pieces off the axis.
    let faces: Vec<usize> = (0..meridian.len())
        .filter(|&i| !(matches!(meridian[i], OutlinePiece::Line { .. }) && on_axis(piece_point(&meridian[i], 0.0)) && on_axis(piece_point(&meridian[i], 1.0))))
        .collect();
    // Each face's split angles (where K meets it, constant along it), or its
    // great circle.
    enum Kind {
        /// A cylinder or cone (or a flat end) split along generators at these
        /// angles (empty: one side).
        Ruled(Vec<f64>),
        /// A cylinder or cone whose two grazing curves move round it along
        /// the meridian (`branches`), from rim to rim.
        Moving,
        /// A hemisphere from a pole to an equator rim, split by the great
        /// circle square to its centre's velocity.
        Sphere { centre: Vec3, pole_at_start: bool },
        /// A whole ball (a half circle from pole to pole), split by that
        /// great circle into two hemispheres.
        Ball { centre: Vec3, radius: f64 },
        /// A rounded corner (a zone of a torus: an arc whose centre is off the
        /// axis), crossed by K as a graph v(ψ) of the tube angle for ψ from
        /// `psi_a` to `psi_b`, entering and leaving at the same rim (the end
        /// rim when `at_end`).
        TorusZone { psi_a: f64, psi_b: f64, at_end: bool },
        /// A flat end that slides within its own plane (`SlideDisc`).
        Slide,
        /// A cylinder slid along its own axis (`SlideSide`).
        SlideSide,
        /// A rounded corner split along meridian arcs at these angles (the
        /// grazing curve keeps its angle about the axis: a turn about a
        /// parallel axis, a slide across the tool's axis).
        RuledArc(Vec<f64>),
    }
    // The two grazing angles of a cylinder or cone at s, the lower and the
    // upper, unwrapped against s = 0: the front lies between them.
    let branches = |piece: &OutlinePiece, s: f64| -> Option<(f64, f64)> {
        let reference = {
            let (a, b, _) = harmonic(|psi| g(piece, 0.0, psi));
            b.atan2(a)
        };
        let (a, b, c) = harmonic(|psi| g(piece, s, psi));
        let r = a.hypot(b);
        if !(c.abs() < r * (1.0 - 1e-9)) {
            return None;
        }
        let base = reference + (b.atan2(a) - reference + PI).rem_euclid(TAU) - PI;
        let spread = (-c / r).acos();
        Some((base - spread, base + spread))
    };
    let mut kinds: Vec<Kind> = Vec::with_capacity(faces.len());
    for &i in &faces {
        let piece = &meridian[i];
        match *piece {
            OutlinePiece::Ellipse { .. } => return Err(refuse("revolved_ellipse", "a meridian with an elliptic piece is not swept whole here".into())),
            OutlinePiece::Line { .. } => {
                let roots = |s: f64| -> Vec<f64> {
                    let (a, b, c) = harmonic(|psi| g(piece, s, psi));
                    let r = a.hypot(b);
                    if c.abs() >= r * (1.0 - 1e-12) {
                        return Vec::new();
                    }
                    let base = b.atan2(a);
                    let spread = (-c / r).acos();
                    let mut list = vec![(base - spread).rem_euclid(TAU), (base + spread).rem_euclid(TAU)];
                    list.sort_by(|x, y| x.total_cmp(y));
                    list
                };
                // Where the grazing value vanishes all round (a rim through
                // the motion's axis turning in its own plane) it names no
                // angle; read the roots where it does not.
                let alive = |s: f64| {
                    let (a, b, c) = harmonic(|psi| g(piece, s, psi));
                    a.hypot(b).max(c.abs()) > 1e-12 * scale * screw.velocity(origin).length().max(screw.omega.abs() * scale).max(screw.rate_bound())
                };
                let readings: Vec<Vec<f64>> = [0.0, 0.5, 1.0].into_iter().filter(|&t| alive(t)).map(roots).collect();
                if readings.is_empty() {
                    let (m0, m1) = (piece_point(piece, 0.0), piece_point(piece, 1.0));
                    let flat = (m0[1] - m1[1]).abs() <= 1e-12 * scale;
                    if flat && (on_axis(m0) || on_axis(m1)) {
                        kinds.push(Kind::Slide);
                        continue;
                    }
                    let upright = (m0[0] - m1[0]).abs() <= 1e-12 * scale;
                    if upright && screw.omega == 0.0 && screw.turns.is_none() && screw.frame.is_none() {
                        kinds.push(Kind::SlideSide);
                        continue;
                    }
                    return Err(refuse("revolved_grazing", format!("meridian piece {i} grazes all over")));
                }
                let r0 = readings[0].clone();
                let still = readings.iter().all(|r| r.len() == r0.len() && r.iter().zip(&r0).all(|(x, y)| (x - y).abs() < 1e-10));
                if still {
                    // A face wholly on one side that is not a disc about the
                    // axis (a cone to its point, an annulus, a whole side):
                    // two halves of the same side, seamed at 0 and π.
                    let (m0, m1) = (piece_point(piece, 0.0), piece_point(piece, 1.0));
                    let disc = (m0[1] - m1[1]).abs() <= 1e-12 * scale && (on_axis(m0) || on_axis(m1));
                    kinds.push(Kind::Ruled(if r0.is_empty() && !disc { vec![0.0, PI] } else { r0 }));
                    continue;
                }
                let mut previous: Option<(f64, f64)> = None;
                for k in 0..=64 {
                    let Some(pair) = branches(piece, k as f64 / 64.0) else {
                        return Err(refuse("revolved_grazing", format!("the grazing curves on meridian piece {i} do not run rim to rim")));
                    };
                    if previous.is_some_and(|p| (p.0 - pair.0).abs() > 0.5 || (p.1 - pair.1).abs() > 0.5) {
                        return Err(refuse("revolved_grazing", format!("the grazing curves on meridian piece {i} turn too fast")));
                    }
                    previous = Some(pair);
                }
                kinds.push(Kind::Moving);
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                let (p0, p1) = (piece_point(piece, 0.0), piece_point(piece, 1.0));
                let span = (end - start).abs();
                let centre_on_axis = on_axis(centre);
                let pole_at_start = on_axis(p0);
                let equator = |p: [f64; 2]| (p[1] - centre[1]).abs() <= 1e-12 * scale && (p[0] - radius).abs() <= 1e-12 * scale;
                if centre_on_axis && (span - PI).abs() < 1e-12 && pole_at_start && on_axis(p1) {
                    kinds.push(Kind::Ball { centre: origin.add(w.scale(centre[1])), radius });
                    continue;
                }
                if centre[0] > radius * (1.0 + 1e-9) && end > start && span <= PI + 1e-12 {
                    let at_rim = |v: f64| [centre[0] + radius * v.cos(), centre[1] + radius * v.sin()];
                    let g_tube = |psi: f64, v: f64| -> f64 {
                        let m = at_rim(v);
                        let x = at(m[0], m[1], psi);
                        let normal = w.scale(v.sin()).add(u1.scale(v.cos() * psi.cos())).add(u2.scale(v.cos() * psi.sin()));
                        screw.velocity(x).dot(normal)
                    };
                    // K at the same angles about the axis all along the arc?
                    {
                        let roots_at = |v: f64| -> Option<Vec<f64>> {
                            let (a, b, c) = harmonic(|psi| g_tube(psi, v));
                            let r = a.hypot(b);
                            let floor = 1e-12 * scale * screw.velocity(origin).length().max(screw.omega.abs() * scale).max(screw.rate_bound());
                            if r.max(c.abs()) <= floor || c.abs() >= r * (1.0 - 1e-12) {
                                return None;
                            }
                            let (base, spread) = (b.atan2(a), (-c / r).acos());
                            let mut list = vec![(base - spread).rem_euclid(TAU), (base + spread).rem_euclid(TAU)];
                            list.sort_by(|x, y| x.total_cmp(y));
                            Some(list)
                        };
                        let readings: Vec<Option<Vec<f64>>> = [0.25, 0.5, 0.75].into_iter().map(|t| roots_at(start + (end - start) * t)).collect();
                        if let [Some(r0), Some(r1), Some(r2)] = readings.as_slice() {
                            if r0.iter().zip(r1).chain(r0.iter().zip(r2)).all(|(x, y)| (x - y).abs() < 1e-10) {
                                kinds.push(Kind::RuledArc(r0.clone()));
                                continue;
                            }
                        }
                    }
                    if span >= PI {
                        return Err(refuse("revolved_arc", format!("meridian piece {i}: a half-round rim is swept only where its grazing curves keep their angle")));
                    }
                    let in_zone = |psi: f64| -> Result<Option<f64>, KernelRefusal> {
                        let (a, b, c) = harmonic(|v| g_tube(psi, v));
                        let r = a.hypot(b);
                        if !(c.abs() < r) {
                            return Ok(None);
                        }
                        let base = b.atan2(a);
                        let spread = (-c / r).acos();
                        let inside: Vec<f64> = [base - spread, base + spread]
                            .into_iter()
                            .map(|v| start + (v - start).rem_euclid(TAU))
                            .filter(|&v| v <= end)
                            .collect();
                        match inside.as_slice() {
                            [] => Ok(None),
                            [v] => Ok(Some(*v)),
                            _ => Err(refuse("torus_zone", format!("meridian piece {i}: the grazing curve crosses the rounded corner twice at one angle"))),
                        }
                    };
                    let count = 720;
                    let has: Vec<bool> = (0..count).map(|k| in_zone(TAU * k as f64 / count as f64).map(|r| r.is_some())).collect::<Result<_, _>>()?;
                    let rises: Vec<usize> = (0..count).filter(|&k| has[k] && !has[(k + count - 1) % count]).collect();
                    let falls: Vec<usize> = (0..count).filter(|&k| !has[k] && has[(k + count - 1) % count]).collect();
                    let ([rise], [fall]) = (rises.as_slice(), falls.as_slice()) else {
                        return Err(refuse("torus_zone", format!("meridian piece {i}: the grazing curve does not cross the rounded corner once")));
                    };
                    // The crossing's ends: roots of the grazing value along the rim it meets.
                    let estimate = |k: usize| TAU * (k as f64 - 0.5) / count as f64;
                    let (ea, eb) = (estimate(*rise), estimate(*fall));
                    let sample = |k: usize| TAU * (k % count) as f64 / count as f64;
                    let probe_v = in_zone(sample(*rise))?.ok_or_else(|| refuse("torus_zone", "the grazing curve's crossing is not found".into()))?;
                    let at_end = (probe_v - end).abs() < (probe_v - start).abs();
                    let rim_v = if at_end { end } else { start };
                    let (a, b, c) = harmonic(|psi| g_tube(psi, rim_v));
                    let r = a.hypot(b);
                    let base = b.atan2(a);
                    let spread = (-c / r).acos();
                    let roots = [base - spread, base + spread];
                    let nearest = |e: f64| roots.iter().copied().min_by(|x, y| ((x - e + PI).rem_euclid(TAU) - PI).abs().total_cmp(&((y - e + PI).rem_euclid(TAU) - PI).abs())).expect("two");
                    let (psi_a, psi_b) = (nearest(ea).rem_euclid(TAU), nearest(eb).rem_euclid(TAU));
                    let psi_b = if psi_b <= psi_a { psi_b + TAU } else { psi_b };
                    let leave_v = in_zone(sample(*fall + count - 1))?.ok_or_else(|| refuse("torus_zone", "the grazing curve's crossing is not found".into()))?;
                    if ((leave_v - end).abs() < (leave_v - start).abs()) != at_end {
                        return Err(refuse("torus_zone", format!("meridian piece {i}: the grazing curve crosses the rounded corner from rim to rim")));
                    }
                    kinds.push(Kind::TorusZone { psi_a, psi_b, at_end });
                    continue;
                }
                if !(centre_on_axis && (span - 0.5 * PI).abs() < 1e-12 && (pole_at_start && equator(p1) || on_axis(p1) && equator(p0))) {
                    // A cap of a sphere about the axis, from its pole to a rim
                    // short of (or past) the equator: swept where its grazing
                    // curves keep their angle about the axis (meridian arcs
                    // through the pole).
                    if centre_on_axis && end > start && span < PI && (pole_at_start != on_axis(p1)) {
                        let roots_at = |s: f64| -> Option<Vec<f64>> {
                            let (a, b, c) = harmonic(|psi| g(piece, s, psi));
                            let r = a.hypot(b);
                            let floor = 1e-12 * scale * screw.velocity(origin).length().max(screw.omega.abs() * scale).max(screw.rate_bound());
                            if r.max(c.abs()) <= floor || c.abs() >= r * (1.0 - 1e-12) {
                                return None;
                            }
                            let (base, spread) = (b.atan2(a), (-c / r).acos());
                            let mut list = vec![(base - spread).rem_euclid(TAU), (base + spread).rem_euclid(TAU)];
                            list.sort_by(|x, y| x.total_cmp(y));
                            Some(list)
                        };
                        if let [Some(r0), Some(r1), Some(r2)] = [0.25, 0.5, 0.75].map(roots_at) {
                            if r0.iter().zip(&r1).chain(r0.iter().zip(&r2)).all(|(x, y)| (x - y).abs() < 1e-10) {
                                kinds.push(Kind::RuledArc(r0));
                                continue;
                            }
                        }
                    }
                    return Err(refuse("revolved_arc", format!("meridian piece {i}: only a quarter circle from a pole to an equator is swept here")));
                }
                kinds.push(Kind::Sphere { centre: origin.add(w.scale(centre[1])), pole_at_start });
            }
        }
    }
    // Rims: junctions off the axis, with the angles they are split at.
    let count = meridian.len();
    let junction = |k: usize| piece_point(&meridian[k], 1.0); // between piece k and k + 1
    let face_of = |piece: usize| faces.iter().position(|&f| f == piece);
    let mut rim_angles: HashMap<usize, Vec<f64>> = HashMap::new();
    for k in 0..count {
        if on_axis(junction(k)) {
            continue;
        }
        let mut angles: Vec<f64> = Vec::new();
        for (piece, s_end) in [(k, 1.0), ((k + 1) % count, 0.0)] {
            let Some(f) = face_of(piece) else { continue };
            match &kinds[f] {
                Kind::Ruled(roots) | Kind::RuledArc(roots) => angles.extend(roots.iter().copied()),
                Kind::Ball { .. } | Kind::Slide => {}
                Kind::SlideSide => angles.extend([0.0, PI]),
                Kind::TorusZone { psi_a, psi_b, .. } => angles.extend([psi_a.rem_euclid(TAU), psi_b.rem_euclid(TAU)]),
                Kind::Moving => {
                    let (lower, upper) = branches(&meridian[piece], s_end).expect("checked");
                    angles.extend([lower.rem_euclid(TAU), upper.rem_euclid(TAU)]);
                }
                Kind::Sphere { centre, .. } => {
                    let d = screw.velocity(*centre).cross(w);
                    let base = d.dot(u2).atan2(d.dot(u1)).rem_euclid(TAU);
                    angles.extend([base, (base + PI).rem_euclid(TAU)]);
                }
            }
        }
        angles.sort_by(|a, b| a.total_cmp(b));
        let mut merged: Vec<f64> = Vec::new();
        for a in angles {
            if merged.last().is_some_and(|&b| (a - b).abs() < 1e-9) || merged.first().is_some_and(|&b| (b + TAU - a).abs() < 1e-9) {
                continue;
            }
            merged.push(a);
        }
        rim_angles.insert(k, merged);
    }
    // Every face's own split angles must be its rims' (one arc per patch side).
    for (f, &piece) in faces.iter().enumerate() {
        for (k, s_end) in [((piece + count - 1) % count, 0.0), (piece, 1.0)] {
            let own: Vec<f64> = match &kinds[f] {
                Kind::Ruled(roots) | Kind::RuledArc(roots) => roots.clone(),
                Kind::Moving => {
                    let (lower, upper) = branches(&meridian[piece], s_end).expect("checked");
                    let mut list = vec![lower.rem_euclid(TAU), upper.rem_euclid(TAU)];
                    list.sort_by(|a, b| a.total_cmp(b));
                    list
                }
                Kind::TorusZone { psi_a, psi_b, .. } => {
                    let mut list = vec![psi_a.rem_euclid(TAU), psi_b.rem_euclid(TAU)];
                    list.sort_by(|a, b| a.total_cmp(b));
                    list
                }
                Kind::Sphere { .. } | Kind::Ball { .. } | Kind::Slide => continue,
                Kind::SlideSide => vec![0.0, PI],
            };
            if let Some(angles) = rim_angles.get(&k) {
                if own.is_empty() {
                    continue;
                }
                if angles.len() != own.len() || angles.iter().zip(&own).any(|(a, b)| (a - b).abs() > 1e-9) {
                    return Err(refuse("revolved_rims", format!("meridian piece {piece} splits at other angles than its rim")));
                }
            }
        }
    }
    let mut points: Vec<Vec3> = Vec::new();
    let mut curves: Vec<ToolCurve> = Vec::new();
    let mut caps: Vec<Cap> = Vec::new();
    let mut slides: Vec<SlideDisc> = Vec::new();
    let mut slide_sides: Vec<SlideSide> = Vec::new();
    // Rim points and arcs: arc j of rim k runs from angle j to angle j + 1.
    let mut rim_points: HashMap<(usize, usize), usize> = HashMap::new();
    let mut rim_arcs: HashMap<(usize, usize), usize> = HashMap::new();
    let mut rim_keys: Vec<usize> = rim_angles.keys().copied().collect();
    rim_keys.sort_unstable();
    for &k in &rim_keys {
        let q = junction(k);
        let angles = rim_angles[&k].clone();
        let angles = if angles.is_empty() { vec![0.0] } else { angles };
        for (j, &a) in angles.iter().enumerate() {
            points.push(at(q[0], q[1], a));
            rim_points.insert((k, j), points.len() - 1);
        }
        for j in 0..angles.len() {
            let a = angles[j];
            let b = if j + 1 < angles.len() { angles[j + 1] } else { angles[0] + TAU };
            let base = at(q[0], q[1], a);
            let surface = frag(make_revolution(origin, w, &frag(make_line(base, base.add(w)), "line")?, b - a), "rim")?;
            curves.push(ToolCurve {
                curve: frag(surface.iso_curve_v(0.0), "rim_arc")?,
                start: rim_points[&(k, j)],
                end: rim_points[&(k, (j + 1) % angles.len())],
                degenerate: false,
                samples: Some((0..=LOFT_SAMPLES).map(|i| at(q[0], q[1], a + (b - a) * i as f64 / LOFT_SAMPLES as f64)).collect()),
            });
            rim_arcs.insert((k, j), curves.len() - 1);
        }
        rim_angles.insert(k, angles);
    }
    let arc_index = |k: usize, angle: f64| -> Result<usize, KernelRefusal> {
        rim_angles[&k]
            .iter()
            .position(|&a| ((a - angle).rem_euclid(TAU)).min((angle - a).rem_euclid(TAU)) < 1e-9)
            .ok_or_else(|| refuse("revolved_rims", "a patch corner is not on its rim's split".into()))
    };
    let sign_at = |piece: &OutlinePiece, psi: f64| g(piece, 0.5, psi) > 0.0;
    for (f, &piece_index) in faces.iter().enumerate() {
        let piece = &meridian[piece_index];
        let below = (piece_index + count - 1) % count; // rim at the piece's start
        let above = piece_index; // rim at its end
        let (m0, m1) = (piece_point(piece, 0.0), piece_point(piece, 1.0));
        match &kinds[f] {
            Kind::SlideSide => {
                if on_axis(m0) || on_axis(m1) {
                    return Err(refuse("revolved_grazing", format!("meridian piece {piece_index} grazes all over")));
                }
                let seams = [0.0, PI];
                let mut patches = Vec::with_capacity(2);
                for j in 0..2 {
                    let (alpha, theta) = (seams[j], PI);
                    let (ib, ia) = (arc_index(below, alpha)?, arc_index(above, alpha)?);
                    patches.push((alpha, theta, rim_arcs[&(below, ib)], rim_arcs[&(above, ia)], [rim_points[&(below, ib)], rim_points[&(above, ia)]]));
                }
                slide_sides.push(SlideSide { origin, w, patches });
            }
            Kind::RuledArc(roots) => {
                let OutlinePiece::Arc { centre, radius, start, end } = *piece else { unreachable!() };
                let tube_centre = |psi: f64| at(centre[0], centre[1], psi);
                let tube_axis = |psi: f64| u1.scale(psi.cos()).add(u2.scale(psi.sin())).cross(w);
                let tube_point = |psi: f64, v: f64| at(centre[0] + radius * v.cos(), centre[1] + radius * v.sin(), psi);
                // The tube's arc (start -> end) in the meridian half-plane at ψ.
                let generatrix = |psi: f64| -> Result<NurbsCurve, KernelRefusal> {
                    let from = tube_point(psi, start);
                    let surface = frag(make_revolution(tube_centre(psi), tube_axis(psi), &frag(make_line(from, from.add(tube_axis(psi))), "line")?, end - start), "tube")?;
                    frag(surface.iso_curve_v(0.0), "tube")
                };
                // A cap to its pole: the meridian arcs start (or end) there,
                // and each patch has a pole in place of a rim arc.
                let apex = if on_axis(m0) || on_axis(m1) {
                    let m = if on_axis(m0) { m0 } else { m1 };
                    points.push(at(0.0, m[1], 0.0));
                    Some(points.len() - 1)
                } else {
                    None
                };
                let rim_point = |rim: usize, on: bool, angle: f64| -> Result<usize, KernelRefusal> {
                    match (on, apex) {
                        (true, Some(p)) => Ok(p),
                        _ => Ok(rim_points[&(rim, arc_index(rim, angle)?)]),
                    }
                };
                let mut generators = Vec::with_capacity(roots.len());
                for &root in roots {
                    let (a, b) = (rim_point(below, on_axis(m0), root)?, rim_point(above, on_axis(m1), root)?);
                    let mut exact: Vec<Vec3> = (0..=LOFT_SAMPLES).map(|i| tube_point(root, start + (end - start) * i as f64 / LOFT_SAMPLES as f64)).collect();
                    exact[0] = points[a];
                    exact[LOFT_SAMPLES] = points[b];
                    curves.push(ToolCurve { curve: generatrix(root)?, start: a, end: b, degenerate: false, samples: Some(exact) });
                    generators.push(curves.len() - 1);
                }
                for j in 0..roots.len() {
                    let (alpha, beta) = (roots[j], if j + 1 < roots.len() { roots[j + 1] } else { roots[0] + TAU });
                    let surface = frag(make_revolution(origin, w, &generatrix(alpha)?, beta - alpha), "patch")?;
                    let (mid_psi, mid_v) = (0.5 * (alpha + beta), 0.5 * (start + end));
                    let mid_normal = w.scale(mid_v.sin()).add(u1.scale(mid_v.cos() * mid_psi.cos())).add(u2.scale(mid_v.cos() * mid_psi.sin()));
                    let front = screw.velocity(tube_point(mid_psi, mid_v)).dot(mid_normal) > 0.0;
                    let pole = |curves: &mut Vec<ToolCurve>| -> usize {
                        let p = apex.expect("apex");
                        curves.push(ToolCurve { curve: make_line(points[p], points[p]).expect("point"), start: p, end: p, degenerate: true, samples: None });
                        curves.len() - 1
                    };
                    let lower = if on_axis(m0) { pole(&mut curves) } else { rim_arcs[&(below, arc_index(below, alpha)?)] };
                    let upper = if on_axis(m1) { pole(&mut curves) } else { rim_arcs[&(above, arc_index(above, alpha)?)] };
                    let (g_alpha, g_beta) = (generators[j], generators[(j + 1) % roots.len()]);
                    let coedges = vec![
                        (lower, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                        (g_beta, true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (upper, on_axis(m1), uv_line([1.0, 1.0], [0.0, 1.0])?),
                        (g_alpha, false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                    ];
                    let chart = Chart::Revolution {
                        origin,
                        w,
                        u1,
                        u2,
                        alpha,
                        theta: beta - alpha,
                        spans_u: spans_of(&surface.knots_u),
                        meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: spans_of(&surface.knots_v) },
                    };
                    caps.push(Cap { surface, front, coedges, chart: Some(chart) });
                }
            }
            Kind::Slide => {
                let rim = if on_axis(m0) { above } else { below };
                let n = normal(piece, 0.5);
                let outward = w.scale(n[1].signum());
                let arcs = rim_angles[&rim].len();
                // Counter-clockwise about the outward normal: increasing ψ about +w.
                let order: Vec<usize> = if n[1] > 0.0 { (0..arcs).collect() } else { (0..arcs).rev().collect() };
                slides.push(SlideDisc {
                    centre: origin.add(w.scale(m0[1])),
                    u: u1,
                    v: outward.cross(u1),
                    rim: order.into_iter().map(|j| (rim_arcs[&(rim, j)], n[1] > 0.0)).collect(),
                });
            }
            Kind::Ruled(roots) if roots.is_empty() => {
                // A flat end wholly on one side: a disc about the axis.
                let flat = (m0[1] - m1[1]).abs() <= 1e-12 * scale;
                if !flat || !(on_axis(m0) || on_axis(m1)) {
                    return Err(refuse("revolved_side", format!("meridian piece {piece_index}: a whole side or ring is not swept here")));
                }
                let rim = if on_axis(m0) { above } else { below };
                let front = sign_at(piece, 0.0);
                if (0..64).any(|k| sign_at(piece, TAU * k as f64 / 64.0) != front) {
                    return Err(refuse("revolved_side", format!("meridian piece {piece_index}: a flat end meets its grazing line")));
                }
                let n = normal(piece, 0.5);
                let outward = w.scale(n[1].signum());
                let centre = origin.add(w.scale(m0[1]));
                let (u, v) = (u1, outward.cross(u1));
                let arcs = rim_angles[&rim].len();
                let mut coedges = Vec::with_capacity(arcs);
                let mut corners = Vec::new();
                // Counter-clockwise about the outward normal: increasing ψ about +w.
                let order: Vec<usize> = if n[1] > 0.0 { (0..arcs).collect() } else { (0..arcs).rev().collect() };
                for j in order {
                    let c = rim_arcs[&(rim, j)];
                    let forward = n[1] > 0.0;
                    coedges.push((c, forward, plane_pcurve(centre, u, v, &curves[c].curve, forward)?));
                    corners.push(points[curves[c].start]);
                }
                let r = junction(rim)[0];
                corners.extend([centre.add(u.scale(r)).add(v.scale(r)), centre.sub(u.scale(r)).sub(v.scale(r))]);
                caps.push(Cap { surface: plane_patch(centre, u, v, &corners)?, front, coedges, chart: Some(Chart::Plane { origin: centre, u, v }) });
            }
            Kind::Ruled(roots) => {
                // A cone to a point: its generators start (or end) at the
                // apex, and each patch has a pole there in place of a rim arc.
                let apex = if on_axis(m0) || on_axis(m1) {
                    let m = if on_axis(m0) { m0 } else { m1 };
                    points.push(at(0.0, m[1], 0.0));
                    Some(points.len() - 1)
                } else {
                    None
                };
                let rim_point = |rim: usize, on: bool, angle: f64| -> Result<usize, KernelRefusal> {
                    match (on, apex) {
                        (true, Some(p)) => Ok(p),
                        _ => Ok(rim_points[&(rim, arc_index(rim, angle)?)]),
                    }
                };
                // Generators at each root, from the start rim to the end rim.
                let mut generators = Vec::with_capacity(roots.len());
                for &root in roots {
                    let (a, b) = (rim_point(below, on_axis(m0), root)?, rim_point(above, on_axis(m1), root)?);
                    let line_samples = (0..=LOFT_SAMPLES).map(|i| points[a].add(points[b].sub(points[a]).scale(i as f64 / LOFT_SAMPLES as f64))).collect();
                    curves.push(ToolCurve { curve: frag(make_line(points[a], points[b]), "generator")?, start: a, end: b, degenerate: false, samples: Some(line_samples) });
                    generators.push(curves.len() - 1);
                }
                for j in 0..roots.len() {
                    let (alpha, beta) = (roots[j], if j + 1 < roots.len() { roots[j + 1] } else { roots[0] + TAU });
                    let generatrix = frag(make_line(at(m0[0], m0[1], alpha), at(m1[0], m1[1], alpha)), "generatrix")?;
                    let surface = frag(make_revolution(origin, w, &generatrix, beta - alpha), "patch")?;
                    let front = sign_at(piece, 0.5 * (alpha + beta));
                    let pole = |curves: &mut Vec<ToolCurve>| -> usize {
                        let p = apex.expect("apex");
                        curves.push(ToolCurve { curve: make_line(points[p], points[p]).expect("point"), start: p, end: p, degenerate: true, samples: None });
                        curves.len() - 1
                    };
                    let lower = if on_axis(m0) { pole(&mut curves) } else { rim_arcs[&(below, arc_index(below, alpha)?)] };
                    let upper = if on_axis(m1) { pole(&mut curves) } else { rim_arcs[&(above, arc_index(above, alpha)?)] };
                    let (g_alpha, g_beta) = (generators[j], generators[(j + 1) % roots.len()]);
                    let coedges = vec![
                        (lower, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                        (g_beta, true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (upper, on_axis(m1), uv_line([1.0, 1.0], [0.0, 1.0])?),
                        (g_alpha, false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                    ];
                    let chart = Chart::Revolution { origin, w, u1, u2, alpha, theta: beta - alpha, spans_u: spans_of(&surface.knots_u), meridian: MeridianChart::Line { m0, m1 } };
                    caps.push(Cap { surface, front, coedges, chart: Some(chart) });
                }
            }
            Kind::Moving => {
                if on_axis(m0) || on_axis(m1) {
                    return Err(refuse("revolved_side", format!("meridian piece {piece_index}: a split cone to a point is not swept here")));
                }
                let n = 128;
                let params: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
                let pairs: Vec<(f64, f64)> = params.iter().map(|&t| branches(piece, t).expect("checked")).collect();
                let pick = |pair: (f64, f64), which: usize| if which == 0 { pair.0 } else { pair.1 };
                // K: the lower and upper branch, each from the start rim to the end rim.
                let mut branch_curves = [0usize; 2];
                for which in 0..2 {
                    let (psi0, psi1) = (pick(pairs[0], which), pick(pairs[n], which));
                    let start = rim_points[&(below, arc_index(below, psi0)?)];
                    let end = rim_points[&(above, arc_index(above, psi1)?)];
                    let mut samples: Vec<Vec3> = params
                        .iter()
                        .zip(&pairs)
                        .map(|(&t, &pair)| {
                            let m = piece_point(piece, t);
                            at(m[0], m[1], pick(pair, which))
                        })
                        .collect();
                    samples[0] = points[start];
                    samples[n] = points[end];
                    let mut exact: Vec<Vec3> = (0..=LOFT_SAMPLES)
                        .map(|i| {
                            let t = i as f64 / LOFT_SAMPLES as f64;
                            let m = piece_point(piece, t);
                            at(m[0], m[1], pick(branches(piece, t).expect("checked"), which))
                        })
                        .collect();
                    exact[0] = points[start];
                    exact[LOFT_SAMPLES] = points[end];
                    curves.push(ToolCurve { curve: frag(fit_tool_curve(&samples, &params), "branch")?, start, end, degenerate: false, samples: Some(exact) });
                    branch_curves[which] = curves.len() - 1;
                }
                // Front between lower and upper; back from upper to lower + 2π.
                for (lo_which, hi_which, hi_turn) in [(0usize, 1usize, 0.0), (1, 0, TAU)] {
                    let lo = |i: usize| pick(pairs[i], lo_which);
                    let hi = |i: usize| pick(pairs[i], hi_which) + hi_turn;
                    let alpha = (0..=n).map(lo).fold(f64::INFINITY, f64::min) - 1e-3;
                    let theta = (0..=n).map(hi).fold(f64::NEG_INFINITY, f64::max) + 1e-3 - alpha;
                    if theta >= TAU {
                        return Err(refuse("revolved_side", format!("meridian piece {piece_index}: a side patch wraps all the way round")));
                    }
                    let generatrix = frag(make_line(at(m0[0], m0[1], alpha), at(m1[0], m1[1], alpha)), "generatrix")?;
                    let surface = frag(make_revolution(origin, w, &generatrix, theta), "patch")?;
                    let spans = (surface.knots_u.len() - 6) / 2 + 1;
                    let u_of = |psi: f64| crate::circle_angle_to_parameter(spans, theta, psi - alpha);
                    let angle_near = |x: Vec3, guess: f64| -> f64 {
                        let r = x.sub(origin);
                        let a = r.dot(u2).atan2(r.dot(u1));
                        guess + (a - guess + PI).rem_euclid(TAU) - PI
                    };
                    let front = sign_at(piece, 0.5 * (lo(n / 2) + hi(n / 2)));
                    // A rim arc's pcurve on this patch, through its own parameter.
                    let rim_pcurve = |arc: usize, v: f64| -> Result<NurbsCurve, KernelRefusal> {
                        let curve = &curves[arc].curve;
                        let [d0, d1] = frag(curve.domain(), "domain")?;
                        let ts: Vec<f64> = pcurve_stations(d0, d1);
                        let mut uv = Vec::with_capacity(ts.len());
                        for &t in &ts {
                            let x = frag(curve.evaluate(t), "rim")?;
                            uv.push(Vec3::new(u_of(angle_near(x, alpha + 0.5 * theta)), v, 0.0));
                        }
                        if pcurve_joints_on() {
                            let (ju, jv) = surface_joints(&surface);
                            let at = |t: f64| -> Result<[f64; 2], KernelRefusal> { Ok([u_of(angle_near(frag(curve.evaluate(t), "rim")?, alpha + 0.5 * theta)), v]) };
                            if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &ju, &jv, curve)? {
                                return Ok(fitted);
                            }
                        }
                        frag(fit_pcurve(&uv, &ts), "rim_pcurve")
                    };
                    // A grazing branch's pcurve: the image of the built edge,
                    // through its own parameter (angle about the axis; place
                    // along the meridian line).
                    let branch_pcurve = |which: usize, _turn: f64| -> Result<NurbsCurve, KernelRefusal> {
                        let curve = &curves[branch_curves[which]].curve;
                        let [d0, d1] = frag(curve.domain(), "domain")?;
                        let (span_rho, span_z) = (m1[0] - m0[0], m1[1] - m0[1]);
                        let length2 = span_rho * span_rho + span_z * span_z;
                        let ts: Vec<f64> = pcurve_stations(d0, d1);
                        let mut uv = Vec::with_capacity(ts.len());
                        let read = |x: Vec3| -> [f64; 2] {
                            let r = x.sub(origin);
                            let z = r.dot(w);
                            let rho = r.sub(w.scale(z)).length();
                            let along = ((rho - m0[0]) * span_rho + (z - m0[1]) * span_z) / length2;
                            [u_of(angle_near(x, alpha + 0.5 * theta)), along]
                        };
                        for &t in &ts {
                            let [u, along] = read(frag(curve.evaluate(t), "branch")?);
                            uv.push(Vec3::new(u, along, 0.0));
                        }
                        if pcurve_joints_on() {
                            let (ju, jv) = surface_joints(&surface);
                            let at = |t: f64| -> Result<[f64; 2], KernelRefusal> { Ok(read(frag(curve.evaluate(t), "branch")?)) };
                            if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &ju, &jv, curve)? {
                                return Ok(fitted);
                            }
                        }
                        frag(fit_pcurve(&uv, &ts), "branch_pcurve")
                    };
                    let bottom = rim_arcs[&(below, arc_index(below, lo(0))?)];
                    let top = rim_arcs[&(above, arc_index(above, lo(n))?)];
                    let coedges = vec![
                        (bottom, true, rim_pcurve(bottom, 0.0)?),
                        (branch_curves[hi_which], true, branch_pcurve(hi_which, hi_turn)?),
                        (top, false, frag(rim_pcurve(top, 1.0)?.reversed(), "reversed")?),
                        (branch_curves[lo_which], false, frag(branch_pcurve(lo_which, 0.0)?.reversed(), "reversed")?),
                    ];
                    let chart_spans = spans_of(&surface.knots_u);
                    caps.push(Cap { surface, front, coedges, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha, theta, spans_u: chart_spans, meridian: MeridianChart::Line { m0, m1 } }) });
                }
            }
            Kind::TorusZone { psi_a, psi_b, at_end } => {
                let OutlinePiece::Arc { centre, radius, start, end } = *piece else { unreachable!() };
                let (psi_a, psi_b, at_end) = (*psi_a, *psi_b, *at_end);
                let tube_centre = |psi: f64| at(centre[0], centre[1], psi);
                let tube_axis = |psi: f64| u1.scale(psi.cos()).add(u2.scale(psi.sin())).cross(w);
                let tube_point = |psi: f64, v: f64| at(centre[0] + radius * v.cos(), centre[1] + radius * v.sin(), psi);
                let normal_at = |psi: f64, v: f64| w.scale(v.sin()).add(u1.scale(v.cos() * psi.cos())).add(u2.scale(v.cos() * psi.sin()));
                let root = |psi: f64| -> Option<f64> {
                    let (a, b, c) = harmonic(|v| screw.velocity(tube_point(psi, v)).dot(normal_at(psi, v)));
                    let r = a.hypot(b);
                    if !(c.abs() < r) {
                        return None;
                    }
                    let base = b.atan2(a);
                    let spread = (-c / r).acos();
                    [base - spread, base + spread].into_iter().map(|v| start + (v - start).rem_euclid(TAU)).find(|&v| v <= end + 1e-12)
                };
                // The tube's arc (start -> end) in the meridian half-plane at ψ.
                let generatrix = |psi: f64| -> Result<NurbsCurve, KernelRefusal> {
                    let from = tube_point(psi, start);
                    let surface = frag(make_revolution(tube_centre(psi), tube_axis(psi), &frag(make_line(from, from.add(tube_axis(psi))), "line")?, end - start), "tube")?;
                    frag(surface.iso_curve_v(0.0), "tube")
                };
                let (rim_k, other_k) = if at_end { (above, below) } else { (below, above) };
                let ka = rim_points[&(rim_k, arc_index(rim_k, psi_a)?)];
                let kb = rim_points[&(rim_k, arc_index(rim_k, psi_b)?)];
                // K: the graph over ψ ∈ [ψa, ψb].
                let n = 256;
                let params: Vec<f64> = (0..=n).map(|j| j as f64 / n as f64).collect();
                // Spaced as 1 − cos: where K meets its rim it runs along the
                // meridian, so its tube angle goes as √(ψ − ψa); in this
                // parameter it is regular.
                let psi_of = |t: f64| psi_a + (psi_b - psi_a) * 0.5 * (1.0 - (PI * t).cos());
                let mut xyz = Vec::with_capacity(n + 1);
                for j in 0..=n {
                    let psi = psi_of(params[j]);
                    let v = if j == 0 || j == n { if at_end { end } else { start } } else {
                        root(psi).ok_or_else(|| refuse("torus_zone", "the grazing curve leaves the rounded corner".into()))?
                    };
                    xyz.push(tube_point(psi, v));
                }
                xyz[0] = points[ka];
                xyz[n] = points[kb];
                let mut exact = Vec::with_capacity(LOFT_SAMPLES + 1);
                for i in 0..=LOFT_SAMPLES {
                    let psi = psi_of(i as f64 / LOFT_SAMPLES as f64);
                    let v = if i == 0 || i == LOFT_SAMPLES { if at_end { end } else { start } } else {
                        root(psi).ok_or_else(|| refuse("torus_zone", "the grazing curve leaves the rounded corner".into()))?
                    };
                    exact.push(tube_point(psi, v));
                }
                exact[0] = points[ka];
                exact[LOFT_SAMPLES] = points[kb];
                curves.push(ToolCurve { curve: frag(fit_tool_curve(&xyz, &params), "torus_k")?, start: ka, end: kb, degenerate: false, samples: Some(exact) });
                let k_curve = curves.len() - 1;
                // Cut meridians at ψa and ψb, from the start rim to the end rim.
                let mut cut = [0usize; 2];
                for (slot, psi) in [(0, psi_a), (1, psi_b)] {
                    let (lo, hi) = (rim_points[&(below, arc_index(below, psi)?)], rim_points[&(above, arc_index(above, psi)?)]);
                    curves.push(ToolCurve { curve: generatrix(psi)?, start: lo, end: hi, degenerate: false, samples: None });
                    cut[slot] = curves.len() - 1;
                }
                let label = |psi: f64, v: f64| screw.velocity(tube_point(psi, v)).dot(normal_at(psi, v)) > 0.0;
                // The patch over [α, β] and its exact inverse.
                let patch = |alpha: f64, beta: f64| -> Result<(NurbsSurface, usize, usize), KernelRefusal> {
                    let g = generatrix(alpha)?;
                    let surface = frag(make_revolution(origin, w, &g, beta - alpha), "torus_patch")?;
                    let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
                    let spans_v = (g.knots.len() - 6) / 2 + 1;
                    Ok((surface, spans_u, spans_v))
                };
                let inverse_pcurve = |curve: &NurbsCurve, alpha: f64, beta: f64, spans_u: usize, spans_v: usize, forward: bool| -> Result<NurbsCurve, KernelRefusal> {
                    let [d0, d1] = frag(curve.domain(), "domain")?;
                    let ts: Vec<f64> = pcurve_stations(d0, d1);
                    let mut uv = Vec::with_capacity(ts.len());
                    let read = |x: Vec3| -> [f64; 2] {
                        let r = x.sub(origin);
                        let mid = 0.5 * (alpha + beta);
                        let psi = mid + (r.dot(u2).atan2(r.dot(u1)) - mid + PI).rem_euclid(TAU) - PI;
                        let q = x.sub(tube_centre(psi));
                        let radial = u1.scale(psi.cos()).add(u2.scale(psi.sin()));
                        let vmid = 0.5 * (start + end);
                        let v = vmid + (q.dot(w).atan2(q.dot(radial)) - vmid + PI).rem_euclid(TAU) - PI;
                        [crate::circle_angle_to_parameter(spans_u, beta - alpha, psi - alpha), crate::circle_angle_to_parameter(spans_v, end - start, v - start)]
                    };
                    for &t in &ts {
                        let [u, v] = read(frag(curve.evaluate(t), "point")?);
                        uv.push(Vec3::new(u, v, 0.0));
                    }
                    if pcurve_joints_on() {
                        let joints = |spans: usize| (1..spans).map(|k| k as f64 / spans as f64).collect::<Vec<f64>>();
                        let at = |t: f64| -> Result<[f64; 2], KernelRefusal> { Ok(read(frag(curve.evaluate(t), "point")?)) };
                        if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &joints(spans_u), &joints(spans_v), curve)? {
                            return if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") };
                        }
                    }
                    let fitted = frag(fit_pcurve(&uv, &ts), "torus_pcurve")?;
                    if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") }
                };
                // The band away from K: a natural patch over [ψb, ψa + 2π].
                {
                    let (surface, _, band_spans_v) = patch(psi_b, psi_a + TAU)?;
                    let (bottom, top) = (rim_arcs[&(below, arc_index(below, psi_b)?)], rim_arcs[&(above, arc_index(above, psi_b)?)]);
                    let coedges = vec![
                        (bottom, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                        (cut[0], true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (top, false, uv_line([1.0, 1.0], [0.0, 1.0])?),
                        (cut[1], false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                    ];
                    let mid = psi_b + 0.5 * (psi_a + TAU - psi_b);
                    let chart_spans = spans_of(&surface.knots_u);
                    caps.push(Cap { surface, front: label(mid, 0.5 * (start + end)), coedges, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_b, theta: psi_a + TAU - psi_b, spans_u: chart_spans, meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: band_spans_v } }) });
                }
                // Over [ψa, ψb]: the lens between K and its rim, and the rest.
                let (surface, spans_u, spans_v) = patch(psi_a, psi_b)?;
                let rim_arc = rim_arcs[&(rim_k, arc_index(rim_k, psi_a)?)];
                let other_arc = rim_arcs[&(other_k, arc_index(other_k, psi_a)?)];
                let mid = 0.5 * (psi_a + psi_b);
                let v_k = root(mid).ok_or_else(|| refuse("torus_zone", "no grazing point mid-way".into()))?;
                let k_pc = |forward: bool| inverse_pcurve(&curves[k_curve].curve, psi_a, psi_b, spans_u, spans_v, forward);
                let lens_v = if at_end { 0.5 * (v_k + end) } else { 0.5 * (v_k + start) };
                let rest_v = if at_end { 0.5 * (v_k + start) } else { 0.5 * (v_k + end) };
                if at_end {
                    // Lens: K (ψa -> ψb) under the end rim, then the end rim back.
                    let lens = vec![(k_curve, true, k_pc(true)?), (rim_arc, false, uv_line([1.0, 1.0], [0.0, 1.0])?)];
                    caps.push(Cap { surface: surface.clone(), front: label(mid, lens_v), coedges: lens, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: spans_v } }) });
                    let rest = vec![
                        (other_arc, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                        (cut[1], true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (k_curve, false, k_pc(false)?),
                        (cut[0], false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                    ];
                    caps.push(Cap { surface, front: label(mid, rest_v), coedges: rest, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: spans_v } }) });
                } else {
                    let lens = vec![(rim_arc, true, uv_line([0.0, 0.0], [1.0, 0.0])?), (k_curve, false, k_pc(false)?)];
                    caps.push(Cap { surface: surface.clone(), front: label(mid, lens_v), coedges: lens, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: spans_v } }) });
                    let rest = vec![
                        (k_curve, true, k_pc(true)?),
                        (cut[1], true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (other_arc, false, uv_line([1.0, 1.0], [0.0, 1.0])?),
                        (cut[0], false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                    ];
                    caps.push(Cap { surface, front: label(mid, rest_v), coedges: rest, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre, start, sweep: end - start, spans: spans_v } }) });
                }
            }
            Kind::Ball { centre, radius } => {
                let m = frag(screw.velocity(*centre).normalized(), "ball_still")?;
                let d = frag(w.cross(m).normalized().or_else(|_| u1.cross(m).normalized()), "ball_axis")?;
                let (e1, e2) = (centre.add(d.scale(*radius)), centre.sub(d.scale(*radius)));
                points.push(e1);
                points.push(e2);
                let (i1, i2) = (points.len() - 2, points.len() - 1);
                let half = frag(make_revolution(*centre, m, &frag(make_line(e1, e1.add(m)), "line")?, PI), "great")?;
                let k_a = frag(half.iso_curve_v(0.0), "great")?;
                // Exact samples at even angles from e1 round to e2 (for a
                // lofted wall), turned the way the curve runs.
                let k_a_samples: Vec<Vec3> = {
                    let [a0, a1] = frag(k_a.domain(), "domain")?;
                    let middle = frag(k_a.evaluate(0.5 * (a0 + a1)), "middle")?;
                    let arm = e1.sub(*centre);
                    let sense = if arm.rotate_about(m, 0.5 * PI).add(*centre).sub(middle).length() < arm.rotate_about(m, -0.5 * PI).add(*centre).sub(middle).length() { 1.0 } else { -1.0 };
                    let mut list: Vec<Vec3> = (0..=LOFT_SAMPLES).map(|i| centre.add(arm.rotate_about(m, sense * PI * i as f64 / LOFT_SAMPLES as f64))).collect();
                    list[0] = e1;
                    list[LOFT_SAMPLES] = e2;
                    list
                };
                curves.push(ToolCurve { curve: k_a.clone(), start: i1, end: i2, degenerate: false, samples: Some(k_a_samples.clone()) });
                let k_a_index = curves.len() - 1;
                let [a0, a1] = frag(k_a.domain(), "domain")?;
                let h = frag(k_a.evaluate(0.5 * (a0 + a1)), "middle")?.sub(*centre);
                let mut k_b_index: Option<usize> = None;
                for front in [true, false] {
                    let toward = if front { m } else { m.scale(-1.0) };
                    let da = if d.cross(h).dot(toward) > 0.0 { d } else { d.scale(-1.0) };
                    let mut generatrix = k_a.clone();
                    let mut surface = frag(make_revolution(*centre, da, &generatrix, PI), "hemisphere")?;
                    let outward = |s: &NurbsSurface| -> Result<bool, KernelRefusal> {
                        let p = frag(s.evaluate(0.5, 0.5), "probe")?;
                        let su = frag(s.evaluate(0.5 + 1e-6, 0.5), "probe")?.sub(frag(s.evaluate(0.5 - 1e-6, 0.5), "probe")?);
                        let sv = frag(s.evaluate(0.5, 0.5 + 1e-6), "probe")?.sub(frag(s.evaluate(0.5, 0.5 - 1e-6), "probe")?);
                        Ok(su.cross(sv).dot(p.sub(*centre)) > 0.0)
                    };
                    let mut forward = true;
                    if !outward(&surface)? {
                        generatrix = frag(generatrix.reversed(), "reversed")?;
                        surface = frag(make_revolution(*centre, da, &generatrix, PI), "hemisphere")?;
                        forward = false;
                    }
                    let (start, end) = if forward { (i1, i2) } else { (i2, i1) };
                    let [g0, g1] = frag(generatrix.domain(), "domain")?;
                    let k_b = match k_b_index {
                        Some(index) => index,
                        None => {
                            // The other half of the great circle: the first half turned half way about `da`.
                            let mut samples: Vec<Vec3> = k_a_samples.iter().map(|&x| centre.add(x.sub(*centre).rotate_about(da, PI))).collect();
                            if !forward {
                                samples.reverse();
                            }
                            samples[0] = points[start];
                            samples[LOFT_SAMPLES] = points[end];
                            curves.push(ToolCurve { curve: frag(surface.iso_curve_u(1.0), "great")?, start, end, degenerate: false, samples: Some(samples) });
                            k_b_index = Some(curves.len() - 1);
                            curves.len() - 1
                        }
                    };
                    let k_b_forward = curves[k_b].start == start;
                    let k_b_pcurve = uv_line([1.0, g0], [1.0, g1])?;
                    let poles = [start, end].map(|p| {
                        curves.push(ToolCurve { curve: make_line(points[p], points[p]).expect("point"), start: p, end: p, degenerate: true, samples: None });
                        curves.len() - 1
                    });
                    let coedges = vec![
                        (poles[0], true, uv_line([0.0, g0], [1.0, g0])?),
                        (k_b, k_b_forward, k_b_pcurve),
                        (poles[1], true, uv_line([1.0, g1], [0.0, g1])?),
                        (k_a_index, !forward, uv_line([0.0, g1], [0.0, g0])?),
                    ];
                    caps.push(Cap { surface, front, coedges, chart: None });
                }
            }
            Kind::Sphere { centre, pole_at_start } => {
                let rim = if *pole_at_start { above } else { below };
                let angles = rim_angles[&rim].clone();
                if angles.len() != 2 {
                    return Err(refuse("revolved_rims", "a hemisphere's rim is not split in two".into()));
                }
                let pole_side = w.scale(if *pole_at_start { m0[1] - m1[1] } else { m1[1] - m0[1] }.signum());
                let m = screw.velocity(*centre);
                let e1 = points[rim_points[&(rim, 0)]];
                let d = frag(e1.sub(*centre).normalized(), "lune_axis")?;
                let mut great: Option<usize> = None;
                for j in 0..2 {
                    let arc = rim_arcs[&(rim, j)];
                    let [a0, a1] = frag(curves[arc].curve.domain(), "domain")?;
                    let middle = frag(curves[arc].curve.evaluate(0.5 * (a0 + a1)), "middle")?;
                    let hv = middle.sub(*centre);
                    let da = if d.cross(hv).dot(pole_side) > 0.0 { d } else { d.scale(-1.0) };
                    // Turn the rim arc about da until its middle reaches K's plane.
                    let theta = (-m.dot(hv)).atan2(m.dot(da.cross(hv))).rem_euclid(TAU);
                    let theta = if theta > PI { theta - PI } else { theta };
                    if !(theta > 1e-9 && theta < PI - 1e-9) {
                        return Err(refuse("revolved_lune", "the great circle meets the rim's plane edge-on".into()));
                    }
                    let mut generatrix = curves[arc].curve.clone();
                    let mut surface = frag(make_revolution(*centre, da, &generatrix, theta), "lune")?;
                    // Outward: (x − centre) at the lune's middle.
                    let probe = |s: &NurbsSurface| -> Result<bool, KernelRefusal> {
                        let p = frag(s.evaluate(0.5, 0.5), "probe")?;
                        let su = frag(s.evaluate(0.5 + 1e-6, 0.5), "probe")?.sub(frag(s.evaluate(0.5 - 1e-6, 0.5), "probe")?);
                        let sv = frag(s.evaluate(0.5, 0.5 + 1e-6), "probe")?.sub(frag(s.evaluate(0.5, 0.5 - 1e-6), "probe")?);
                        Ok(su.cross(sv).dot(p.sub(*centre)) > 0.0)
                    };
                    let mut forward = true;
                    if !probe(&surface)? {
                        generatrix = frag(generatrix.reversed(), "reversed")?;
                        surface = frag(make_revolution(*centre, da, &generatrix, theta), "lune")?;
                        forward = false;
                    }
                    let (start, end) = if forward { (curves[arc].start, curves[arc].end) } else { (curves[arc].end, curves[arc].start) };
                    let [g0, g1] = frag(generatrix.domain(), "domain")?;
                    // The great-circle half: the turned generatrix (u = 1), from
                    // `start` to `end`; made once, by the first lune.
                    let k_curve = match great {
                        Some(index) => index,
                        None => {
                            curves.push(ToolCurve { curve: frag(surface.iso_curve_u(1.0), "great")?, start, end, degenerate: false, samples: None });
                            great = Some(curves.len() - 1);
                            curves.len() - 1
                        }
                    };
                    let k_forward = curves[k_curve].start == start;
                    let poles = [start, end].map(|p| {
                        curves.push(ToolCurve { curve: make_line(points[p], points[p]).expect("point"), start: p, end: p, degenerate: true, samples: None });
                        curves.len() - 1
                    });
                    let k_pcurve = uv_line([1.0, g0], [1.0, g1])?;
                    let coedges = vec![
                        (poles[0], true, uv_line([0.0, g0], [1.0, g0])?),
                        (k_curve, k_forward, k_pcurve),
                        (poles[1], true, uv_line([1.0, g1], [0.0, g1])?),
                        (arc, !forward, uv_line([0.0, g1], [0.0, g0])?),
                    ];
                    let mid_psi = if j == 0 { 0.5 * (angles[0] + angles[1]) } else { 0.5 * (angles[1] + angles[0] + TAU) };
                    let front = m.dot(at(1.0, 0.0, mid_psi).sub(origin)) > 0.0;
                    caps.push(Cap { surface, front, coedges, chart: None });
                }
            }
        }
    }
    Ok(Split {
        points,
        curves,
        caps,
        slides,
        slide_sides,
        scale,
        shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() },
    })
}

/// `BREP_SWEEP_RING_WINDING` (default off): a ring whose grazing curves wind
/// round its tube as they go once round the ring is split along their
/// continuous unwrapping, in six pieces, instead of refused.
fn ring_winding_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_WINDING", false)
}

/// A ring split by grazing curves that wind round its tube: the curves' angle
/// v on the tube, unwrapped continuously along ψ, gains a whole number of
/// turns going once round the ring. None where it gains none (the two-halves
/// split serves). Six pieces along ψ keep each band's stretch of v short of a
/// whole turn while the curves turn, and leave the tumble's eight trial seams
/// (π/4 apart) four distinct placements.
#[allow(clippy::too_many_arguments)]
fn split_torus_wound(
    screw: &Screw,
    origin: Vec3,
    w: Vec3,
    u1: Vec3,
    u2: Vec3,
    centre: [f64; 2],
    radius: f64,
    meridian: &[OutlinePiece],
) -> Result<Option<Split>, KernelRefusal> {
    const PIECES: usize = 6;
    let (rho_c, z_c) = (centre[0], centre[1]);
    let radial = |psi: f64| u1.scale(psi.cos()).add(u2.scale(psi.sin()));
    let tube_centre = |psi: f64| origin.add(w.scale(z_c)).add(radial(psi).scale(rho_c));
    let point = |psi: f64, v: f64| tube_centre(psi).add(radial(psi).scale(radius * v.cos())).add(w.scale(radius * v.sin()));
    let g = |psi: f64, v: f64| screw.velocity(point(psi, v)).dot(radial(psi).scale(v.cos()).add(w.scale(v.sin())));
    let n = screw.loop_samples;
    let total = PIECES * n;
    // (ψ, lower, upper) round the whole ring, each base angle the one nearest
    // the last.
    let mut round: Vec<(f64, f64, f64)> = Vec::with_capacity(total + 1);
    let mut near = {
        let (a, b, _) = harmonic(|v| g(0.0, v));
        b.atan2(a)
    };
    for i in 0..=total {
        let psi = TAU * i as f64 / total as f64;
        let (a, b, c) = harmonic(|v| g(psi, v));
        let r = a.hypot(b);
        if !(c.abs() < r * (1.0 - 1e-9)) {
            return Err(refuse("ring_grazing", "the ring's grazing curves do not run all the way round it".into()));
        }
        let base = near + (b.atan2(a) - near + PI).rem_euclid(TAU) - PI;
        if i > 0 && (base - near).abs() > 0.5 {
            return Err(refuse("ring_grazing", "the ring's grazing curves turn too fast".into()));
        }
        near = base;
        let spread = (-c / r).acos();
        round.push((psi, base - spread, base + spread));
    }
    if ((round[total].1 - round[0].1) / TAU).round() == 0.0 {
        return Ok(None);
    }
    let samples: Vec<Vec<(f64, f64, f64)>> = (0..PIECES).map(|k| round[k * n..=(k + 1) * n].to_vec()).collect();
    let seam_psi = |k: usize| TAU * k as f64 / PIECES as f64;
    let params: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
    let mut points: Vec<Vec3> = Vec::new();
    let mut curves: Vec<ToolCurve> = Vec::new();
    // Points: at each seam, on each branch (the last piece ends on seam 0,
    // a whole number of turns round the tube on).
    let mut corner: HashMap<(usize, usize), usize> = HashMap::new(); // (seam, branch)
    for (seam, piece) in samples.iter().enumerate() {
        let (psi, lower, upper) = piece[0];
        for (branch, v) in [(0, lower), (1, upper)] {
            points.push(point(psi, v));
            corner.insert((seam, branch), points.len() - 1);
        }
    }
    let mut branch_curve: HashMap<(usize, usize), usize> = HashMap::new(); // (piece, branch)
    for (piece, list) in samples.iter().enumerate() {
        for branch in 0..2 {
            let mut xyz: Vec<Vec3> = list.iter().map(|&(psi, lo, hi)| point(psi, if branch == 0 { lo } else { hi })).collect();
            let (start, end) = (corner[&(piece, branch)], corner[&((piece + 1) % PIECES, branch)]);
            xyz[0] = points[start];
            xyz[n] = points[end];
            curves.push(ToolCurve { curve: frag(fit_tool_curve(&xyz, &params), "ring_branch")?, start, end, degenerate: false, samples: Some(xyz.clone()) });
            branch_curve.insert((piece, branch), curves.len() - 1);
        }
    }
    let tube_arc = |psi: f64, v0: f64, v1: f64| -> Result<NurbsCurve, KernelRefusal> {
        let start = point(psi, v0);
        let axis = radial(psi).cross(w);
        let surface = frag(make_revolution(tube_centre(psi), axis, &frag(make_line(start, start.add(axis)), "line")?, v1 - v0), "tube_arc")?;
        frag(surface.iso_curve_v(0.0), "tube_arc")
    };
    // Seam arcs of each band: band 0 (front) from lower to upper, band 1
    // (back) from upper to lower + 2π.
    let mut seam_arc: HashMap<(usize, usize), usize> = HashMap::new(); // (seam, band)
    for seam in 0..PIECES {
        let (psi, lower, upper) = samples[seam][0];
        let (lo_point, hi_point) = (corner[&(seam, 0)], corner[&(seam, 1)]);
        curves.push(ToolCurve { curve: tube_arc(psi, lower, upper)?, start: lo_point, end: hi_point, degenerate: false, samples: None });
        seam_arc.insert((seam, 0), curves.len() - 1);
        curves.push(ToolCurve { curve: tube_arc(psi, upper, lower + TAU)?, start: hi_point, end: lo_point, degenerate: false, samples: None });
        seam_arc.insert((seam, 1), curves.len() - 1);
    }
    let mut caps = Vec::new();
    for piece in 0..PIECES {
        let (psi_a, psi_b) = (seam_psi(piece), seam_psi(piece + 1));
        for band in 0..2 {
            let (lo_branch, hi_branch, hi_turn) = if band == 0 { (0usize, 1usize, 0.0) } else { (1, 0, TAU) };
            let pick = |s: &(f64, f64, f64), b: usize| if b == 0 { s.1 } else { s.2 };
            // As in the two-halves split: a margin each side for end trims.
            let low = samples[piece].iter().map(|s| pick(s, lo_branch)).fold(f64::INFINITY, f64::min);
            let high = samples[piece].iter().map(|s| pick(s, hi_branch) + hi_turn).fold(f64::NEG_INFINITY, f64::max);
            let margin = (0.25 * (TAU - (high - low))).clamp(1e-3, 0.25);
            let (v_low, v_high) = (low - margin, high + margin);
            let theta_v = v_high - v_low;
            if theta_v >= TAU {
                return Err(refuse("ring_grazing", "a band of the ring wraps all the way round its tube".into()));
            }
            let generatrix = tube_arc(psi_a, v_low, v_high)?;
            let surface = frag(make_revolution(origin, w, &generatrix, psi_b - psi_a), "ring_band")?;
            let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
            let spans_v = (generatrix.knots.len() - 6) / 2 + 1;
            let uv_of = |psi: f64, v: f64| {
                Vec3::new(
                    crate::circle_angle_to_parameter(spans_u, psi_b - psi_a, psi - psi_a),
                    crate::circle_angle_to_parameter(spans_v, theta_v, v - v_low),
                    0.0,
                )
            };
            let angles_of = |x: Vec3| -> (f64, f64) {
                let r = x.sub(origin);
                let psi = r.dot(u2).atan2(r.dot(u1));
                let psi = 0.5 * (psi_a + psi_b) + (psi - 0.5 * (psi_a + psi_b) + PI).rem_euclid(TAU) - PI;
                let q = x.sub(tube_centre(psi));
                let v = q.dot(w).atan2(q.dot(radial(psi)));
                let mid = 0.5 * (v_low + v_high);
                (psi, mid + (v - mid + PI).rem_euclid(TAU) - PI)
            };
            let inverse_pcurve = |curve: &NurbsCurve| -> Result<NurbsCurve, KernelRefusal> {
                let [d0, d1] = frag(curve.domain(), "domain")?;
                let ts: Vec<f64> = pcurve_stations(d0, d1);
                let mut uv = Vec::with_capacity(ts.len());
                for &t in &ts {
                    let (psi, v) = angles_of(frag(curve.evaluate(t), "point")?);
                    uv.push(uv_of(psi, v));
                }
                // Broken at the band's span joints (`BREP_SWEEP_PCURVE_JOINTS`).
                if pcurve_joints_on() {
                    let joints = |spans: usize| (1..spans).map(|k| k as f64 / spans as f64).collect::<Vec<f64>>();
                    let at = |t: f64| -> Result<[f64; 2], KernelRefusal> {
                        let (psi, v) = angles_of(frag(curve.evaluate(t), "point")?);
                        let p = uv_of(psi, v);
                        Ok([p.x, p.y])
                    };
                    if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &joints(spans_u), &joints(spans_v), curve)? {
                        return Ok(fitted);
                    }
                }
                frag(fit_pcurve(&uv, &ts), "ring_pcurve")
            };
            let lower_edge = branch_curve[&(piece, lo_branch)];
            let upper_edge = branch_curve[&(piece, hi_branch)];
            let right = seam_arc[&((piece + 1) % PIECES, band)];
            let left = seam_arc[&(piece, band)];
            let coedges = vec![
                (lower_edge, true, inverse_pcurve(&curves[lower_edge].curve)?),
                (right, true, inverse_pcurve(&curves[right].curve)?),
                (upper_edge, false, frag(inverse_pcurve(&curves[upper_edge].curve)?.reversed(), "reversed")?),
                (left, false, frag(inverse_pcurve(&curves[left].curve)?.reversed(), "reversed")?),
            ];
            let mid = &samples[piece][n / 2];
            let front = g(mid.0, 0.5 * (pick(mid, lo_branch) + pick(mid, hi_branch) + hi_turn)) > 0.0;
            caps.push(Cap { surface, front, coedges, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre: [rho_c, z_c], start: v_low, sweep: theta_v, spans: spans_v } }) });
        }
    }
    Ok(Some(Split {
        points,
        curves,
        caps,
        slides: Vec::new(),
        slide_sides: Vec::new(),
        scale: rho_c + radius,
        shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() },
    }))
}

/// A ring (torus: the circle of `radius` about `centre` in (ρ, z), off the
/// axis), split by its two grazing curves. For each angle ψ about the ring's
/// axis the grazing value along the tube's circle is A cos v + B sin v + C, so
/// each curve is a graph v(ψ) round the ring; the front band lies between
/// them. Each band is built in two halves (ψ in [0, π] and [π, 2π]) so no face
/// closes on itself.
#[allow(clippy::too_many_arguments)]
fn split_torus(
    screw: &Screw,
    origin: Vec3,
    w: Vec3,
    u1: Vec3,
    u2: Vec3,
    centre: [f64; 2],
    radius: f64,
    meridian: &[OutlinePiece],
) -> Result<Split, KernelRefusal> {
    if ring_winding_on() {
        if let Some(split) = split_torus_wound(screw, origin, w, u1, u2, centre, radius, meridian)? {
            return Ok(split);
        }
    }
    let (rho_c, z_c) = (centre[0], centre[1]);
    let radial = |psi: f64| u1.scale(psi.cos()).add(u2.scale(psi.sin()));
    let tube_centre = |psi: f64| origin.add(w.scale(z_c)).add(radial(psi).scale(rho_c));
    let point = |psi: f64, v: f64| tube_centre(psi).add(radial(psi).scale(radius * v.cos())).add(w.scale(radius * v.sin()));
    let g = |psi: f64, v: f64| screw.velocity(point(psi, v)).dot(radial(psi).scale(v.cos()).add(w.scale(v.sin())));
    let reference = {
        let (a, b, _) = harmonic(|v| g(0.0, v));
        b.atan2(a)
    };
    // The lower and upper grazing angles v at ψ, unwrapped; the front between.
    let branches = |psi: f64| -> Option<(f64, f64)> {
        let (a, b, c) = harmonic(|v| g(psi, v));
        let r = a.hypot(b);
        if !(c.abs() < r * (1.0 - 1e-9)) {
            return None;
        }
        let base = reference + (b.atan2(a) - reference + PI).rem_euclid(TAU) - PI;
        let spread = (-c / r).acos();
        Some((base - spread, base + spread))
    };
    let n = screw.loop_samples;
    let halves = [(0.0, PI), (PI, TAU)];
    let mut samples: Vec<Vec<(f64, f64, f64)>> = Vec::new(); // per half: (ψ, lower, upper)
    for &(a, b) in &halves {
        let mut list = Vec::with_capacity(n + 1);
        let mut previous: Option<(f64, f64)> = None;
        for i in 0..=n {
            let psi = a + (b - a) * i as f64 / n as f64;
            let Some(pair) = branches(psi) else {
                return Err(refuse("ring_grazing", "the ring's grazing curves do not run all the way round it".into()));
            };
            if previous.is_some_and(|p| (p.0 - pair.0).abs() > 0.5 || (p.1 - pair.1).abs() > 0.5) {
                return Err(refuse("ring_grazing", "the ring's grazing curves turn too fast".into()));
            }
            previous = Some(pair);
            list.push((psi, pair.0, pair.1));
        }
        samples.push(list);
    }
    let params: Vec<f64> = (0..=n).map(|i| i as f64 / n as f64).collect();
    let mut points: Vec<Vec3> = Vec::new();
    let mut curves: Vec<ToolCurve> = Vec::new();
    // Points: at ψ = 0 and π, on each branch.
    let mut corner: HashMap<(usize, usize), usize> = HashMap::new(); // (seam 0 | 1, branch)
    for seam in 0..2 {
        let (psi, (lower, upper)) = if seam == 0 { (0.0, (samples[0][0].1, samples[0][0].2)) } else { (PI, (samples[1][0].1, samples[1][0].2)) };
        for (branch, v) in [(0, lower), (1, upper)] {
            points.push(point(psi, v));
            corner.insert((seam, branch), points.len() - 1);
        }
    }
    let seam_of_end = |half: usize, at_end: bool| -> usize { if at_end { 1 - half } else { half } };
    // K: each branch over each half.
    let mut branch_curve: HashMap<(usize, usize), usize> = HashMap::new(); // (half, branch)
    for half in 0..2 {
        for branch in 0..2 {
            let mut xyz: Vec<Vec3> = samples[half].iter().map(|&(psi, lo, hi)| point(psi, if branch == 0 { lo } else { hi })).collect();
            let (start, end) = (corner[&(seam_of_end(half, false), branch)], corner[&(seam_of_end(half, true), branch)]);
            xyz[0] = points[start];
            xyz[n] = points[end];
            curves.push(ToolCurve { curve: frag(fit_tool_curve(&xyz, &params), "ring_branch")?, start, end, degenerate: false, samples: Some(xyz.clone()) });
            branch_curve.insert((half, branch), curves.len() - 1);
        }
    }
    // Seam arcs of each band at ψ = 0 and π: band 0 (front) from lower to
    // upper, band 1 (back) from upper to lower + 2π.
    let tube_arc = |psi: f64, v0: f64, v1: f64| -> Result<NurbsCurve, KernelRefusal> {
        let start = point(psi, v0);
        let axis = radial(psi).cross(w);
        let surface = frag(make_revolution(tube_centre(psi), axis, &frag(make_line(start, start.add(axis)), "line")?, v1 - v0), "tube_arc")?;
        frag(surface.iso_curve_v(0.0), "tube_arc")
    };
    let mut seam_arc: HashMap<(usize, usize), usize> = HashMap::new(); // (seam, band)
    for seam in 0..2 {
        let psi = if seam == 0 { 0.0 } else { PI };
        let (lower, upper) = (samples[seam][0].1, samples[seam][0].2);
        let (lo_point, hi_point) = (corner[&(seam, 0)], corner[&(seam, 1)]);
        curves.push(ToolCurve { curve: tube_arc(psi, lower, upper)?, start: lo_point, end: hi_point, degenerate: false, samples: None });
        seam_arc.insert((seam, 0), curves.len() - 1);
        curves.push(ToolCurve { curve: tube_arc(psi, upper, lower + TAU)?, start: hi_point, end: lo_point, degenerate: false, samples: None });
        seam_arc.insert((seam, 1), curves.len() - 1);
    }
    let mut caps = Vec::new();
    for half in 0..2 {
        let (psi_a, psi_b) = halves[half];
        for band in 0..2 {
            // The band's lower and upper edges (v), as branches.
            let (lo_branch, hi_branch, hi_turn) = if band == 0 { (0usize, 1usize, 0.0) } else { (1, 0, TAU) };
            let pick = |s: &(f64, f64, f64), b: usize| if b == 0 { s.1 } else { s.2 };
            // A margin each side: an end cap may be carried a little across its
            // grazing curve (see `end_trim`), as far as the tube leaves room.
            let low = samples[half].iter().map(|s| pick(s, lo_branch)).fold(f64::INFINITY, f64::min);
            let high = samples[half].iter().map(|s| pick(s, hi_branch) + hi_turn).fold(f64::NEG_INFINITY, f64::max);
            let margin = (0.25 * (TAU - (high - low))).clamp(1e-3, 0.25);
            let (v_low, v_high) = (low - margin, high + margin);
            let theta_v = v_high - v_low;
            if theta_v >= TAU {
                return Err(refuse("ring_grazing", "a band of the ring wraps all the way round its tube".into()));
            }
            let generatrix = tube_arc(psi_a, v_low, v_high)?;
            let surface = frag(make_revolution(origin, w, &generatrix, psi_b - psi_a), "ring_band")?;
            let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
            let spans_v = (generatrix.knots.len() - 6) / 2 + 1;
            let uv_of = |psi: f64, v: f64| {
                Vec3::new(
                    crate::circle_angle_to_parameter(spans_u, psi_b - psi_a, psi - psi_a),
                    crate::circle_angle_to_parameter(spans_v, theta_v, v - v_low),
                    0.0,
                )
            };
            // (ψ, v) of a point, unwrapped near the patch.
            let angles_of = |x: Vec3| -> (f64, f64) {
                let r = x.sub(origin);
                let psi = r.dot(u2).atan2(r.dot(u1));
                let psi = 0.5 * (psi_a + psi_b) + (psi - 0.5 * (psi_a + psi_b) + PI).rem_euclid(TAU) - PI;
                let q = x.sub(tube_centre(psi));
                let v = q.dot(w).atan2(q.dot(radial(psi)));
                let mid = 0.5 * (v_low + v_high);
                (psi, mid + (v - mid + PI).rem_euclid(TAU) - PI)
            };
            let inverse_pcurve = |curve: &NurbsCurve| -> Result<NurbsCurve, KernelRefusal> {
                let [d0, d1] = frag(curve.domain(), "domain")?;
                let ts: Vec<f64> = pcurve_stations(d0, d1);
                let mut uv = Vec::with_capacity(ts.len());
                for &t in &ts {
                    let (psi, v) = angles_of(frag(curve.evaluate(t), "point")?);
                    uv.push(uv_of(psi, v));
                }
                // Broken at the band's span joints (`BREP_SWEEP_PCURVE_JOINTS`).
                if pcurve_joints_on() {
                    let joints = |spans: usize| (1..spans).map(|k| k as f64 / spans as f64).collect::<Vec<f64>>();
                    let at = |t: f64| -> Result<[f64; 2], KernelRefusal> {
                        let (psi, v) = angles_of(frag(curve.evaluate(t), "point")?);
                        let p = uv_of(psi, v);
                        Ok([p.x, p.y])
                    };
                    if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &joints(spans_u), &joints(spans_v), curve)? {
                        return Ok(fitted);
                    }
                }
                frag(fit_pcurve(&uv, &ts), "ring_pcurve")
            };
            let branch_pcurve = |b: usize, _turn: f64| -> Result<NurbsCurve, KernelRefusal> { inverse_pcurve(&curves[branch_curve[&(half, b)]].curve) };
            let lower_edge = branch_curve[&(half, lo_branch)];
            let upper_edge = branch_curve[&(half, hi_branch)];
            let right = seam_arc[&(seam_of_end(half, true), band)];
            let left = seam_arc[&(seam_of_end(half, false), band)];
            let coedges = vec![
                (lower_edge, true, branch_pcurve(lo_branch, 0.0)?),
                (right, true, inverse_pcurve(&curves[right].curve)?),
                (upper_edge, false, frag(branch_pcurve(hi_branch, hi_turn)?.reversed(), "reversed")?),
                (left, false, frag(inverse_pcurve(&curves[left].curve)?.reversed(), "reversed")?),
            ];
            let mid = &samples[half][n / 2];
            let front = g(mid.0, 0.5 * (pick(mid, lo_branch) + pick(mid, hi_branch) + hi_turn)) > 0.0;
            caps.push(Cap { surface, front, coedges, chart: Some(Chart::Revolution { origin, w, u1, u2, alpha: psi_a, theta: psi_b - psi_a, spans_u, meridian: MeridianChart::Arc { centre: [rho_c, z_c], start: v_low, sweep: theta_v, spans: spans_v } }) });
        }
    }
    Ok(Split {
        points,
        curves,
        caps,
        slides: Vec::new(),
        slide_sides: Vec::new(),
        scale: rho_c + radius,
        shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() },
    })
}

/// Builder of a closed shell with shared vertices and edges.
struct Shell {
    vertices: Vec<VertexRecord>,
    edges: Vec<EdgeRecord>,
    faces: Vec<FaceRecord>,
    next: u64,
    /// Loops beyond each face's first (holes).
    holes: usize,
}

impl Shell {
    fn new() -> Self {
        Self { vertices: Vec::new(), edges: Vec::new(), faces: Vec::new(), next: 1, holes: 0 }
    }

    fn id(&mut self) -> u64 {
        self.next += 1;
        self.next
    }

    fn vertex(&mut self, point: Vec3) -> u64 {
        let id = self.id();
        self.vertices.push(VertexRecord { id, point });
        id
    }

    fn edge(&mut self, curve: NurbsCurve, start: u64, end: u64, degenerate: bool) -> Result<u64, KernelRefusal> {
        let domain = frag(curve.domain(), "edge_domain")?;
        let id = self.id();
        self.edges.push(EdgeRecord { id, curve, t0: domain[0], t1: domain[1], start_vertex_id: start, end_vertex_id: end, degenerate, name: None });
        Ok(id)
    }

    /// A face from its surface and its loop: (edge, traversed forward?, pcurve
    /// in the traversal direction).
    fn face(&mut self, surface: NurbsSurface, coedges: Vec<(u64, bool, NurbsCurve)>) -> u64 {
        let mut records = Vec::with_capacity(coedges.len());
        for (edge, forward, pcurve) in coedges {
            let id = self.id();
            records.push(CoedgeRecord { id, edge_id: edge, forward, pcurve });
        }
        let loop_id = self.id();
        let id = self.id();
        self.faces.push(FaceRecord { id, surface, same_sense: true, loops: vec![LoopRecord { id: loop_id, coedges: records }], name: None });
        id
    }

    /// A face from its surface and its loops, the outer one first.
    fn face_loops(&mut self, surface: NurbsSurface, loops: Vec<Vec<(u64, bool, NurbsCurve)>>) -> u64 {
        let mut records = Vec::with_capacity(loops.len());
        for coedges in loops {
            let mut list = Vec::with_capacity(coedges.len());
            for (edge, forward, pcurve) in coedges {
                let id = self.id();
                list.push(CoedgeRecord { id, edge_id: edge, forward, pcurve });
            }
            let id = self.id();
            records.push(LoopRecord { id, coedges: list });
        }
        self.holes += records.len().saturating_sub(1);
        let id = self.id();
        self.faces.push(FaceRecord { id, surface, same_sense: true, loops: records, name: None });
        id
    }

    fn solid(mut self) -> BrepSolid {
        let shell = self.id();
        let id = self.id();
        // One shell: V − E + F = 2 − 2g (every face one loop; a pole's
        // degenerate edge is not counted, as the validator counts).
        let edges = self.edges.iter().filter(|e| !e.degenerate).count() as i64;
        // A face with holes counts once for each loop beyond its first less.
        let euler = self.vertices.len() as i64 - edges + self.faces.len() as i64 - self.holes as i64;
        let genus = (2 - euler) / 2;
        BrepSolid {
            mass_properties_cache: Default::default(),
            id,
            vertices: self.vertices,
            edges: self.edges,
            shells: vec![ShellRecord { id: shell, faces: self.faces }],
            genus,
            kind: Default::default(),
            cells: Default::default(),
        }
    }
}

/// The whole sweep of `tool` along `motion` (a single screw, turn or slide),
/// its two end positions included.
pub fn sweep_solid(tool: &SolidTool, motion: &SolidMotion) -> Result<BrepSolid, KernelRefusal> {
    sweep_with(&Screw::from_motion(motion)?, tool)
}

fn sweep_with(screw: &Screw, tool: &SolidTool) -> Result<BrepSolid, KernelRefusal> {
    let screw = *screw;
    if let SolidTool::Revolved { meridian, .. } = tool {
        refuse_elliptic(meridian)?;
    }
    if let SolidTool::Revolved { axis_origin, axis_direction, meridian, .. } = tool {
        if let Some(body) = ball_across_axis(&screw, *axis_origin, *axis_direction, meridian)? {
            return Ok(body);
        }
        // (A tool whose radial direction lies along its axis is left to the
        // refusal below.)
        let radial_holds = match (tool, axis_direction.normalized()) {
            (SolidTool::Revolved { radial, .. }, Ok(w)) => radial.sub(w.scale(radial.dot(w))).normalized().is_ok(),
            _ => false,
        };
        if radial_holds && ball_screw::ball_screw_on() {
            if let Some(body) = ball_screw::ball_screwed_past_fold(&screw, *axis_origin, *axis_direction, meridian)? {
                return Ok(body);
            }
        }
    }
    if let SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } = tool {
        if ring_fold::ring_fold_on() {
            if let [OutlinePiece::Arc { centre, radius, start, end }] = meridian.as_slice() {
                if ((end - start).abs() - TAU).abs() < 1e-12 {
                    let w = axis_direction.normalized().map_err(|_| input("axis", "the tool's axis has no direction".into()))?;
                    let u1 = radial.sub(w.scale(radial.dot(w))).normalized().map_err(|_| input("radial", "the radial direction lies along the axis".into()))?;
                    if let Some(body) = ring_fold::ring_slid_past_fold(&screw, *axis_origin, w, u1, w.cross(u1), *centre, *radius)? {
                        return Ok(body);
                    }
                }
            }
        }
        if let Some(body) = flat_ring(&screw, *axis_origin, *axis_direction, *radial, meridian)? {
            return Ok(body);
        }
        if let Some(body) = rod_across_axis(&screw, *axis_origin, *axis_direction, meridian)? {
            return Ok(body);
        }
    }
    if let SolidTool::Polyhedron { vertices, faces } = tool {
        let whole = screw.omega != 0.0 && screw.c == 0.0 && screw.c2 == 0.0 && screw.frame.is_none() && screw.turns.is_none() && screw.omega.abs() * (screw.t1 - screw.t0) >= TAU * (1.0 - 1e-12);
        if whole {
            return whole_turn_shadow(&screw, vertices, faces);
        }
    }
    let split = match tool {
        SolidTool::Polyhedron { vertices, faces } => split_polyhedron(&screw, vertices, faces)?,
        SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } => {
            split_revolved(&screw, *axis_origin, *axis_direction, *radial, meridian)?
        }
    };
    let grazing = split.grazing()?;
    // A tool slid along its own axis has no grazing curve: its sliding sides
    // join its back at the start to its front at the end.
    if grazing.is_empty() && split.slide_sides.is_empty() {
        return Err(refuse("no_grazing", "the tool has no grazing loop under this motion".into()));
    }
    let times = row_times(&screw);
    let last = times.len() - 1;
    let mut shell = Shell::new();
    // Vertices and edges at the row times (0 and `last` are the end positions),
    // made on demand.
    let mut vertex_at: HashMap<(usize, usize), u64> = HashMap::new();
    let mut edge_at: HashMap<(usize, usize), u64> = HashMap::new();
    let mut at_time = |shell: &mut Shell, j: usize, p: usize| -> u64 {
        *vertex_at.entry((j, p)).or_insert_with(|| shell.vertex(screw.apply(times[j], split.points[p])))
    };
    let mut time_edge = |shell: &mut Shell, j: usize, c: usize, vertex: &mut dyn FnMut(&mut Shell, usize, usize) -> u64| -> Result<u64, KernelRefusal> {
        if let Some(&edge) = edge_at.get(&(j, c)) {
            return Ok(edge);
        }
        let curve = &split.curves[c];
        let (a, b) = (vertex(shell, j, curve.start), vertex(shell, j, curve.end));
        let edge = shell.edge(screw.move_curve(times[j], &curve.curve)?, a, b, curve.degenerate)?;
        edge_at.insert((j, c), edge);
        Ok(edge)
    };
    // Caps: the back at t0, the front at t1.
    for cap in &split.caps {
        let j = if cap.front { last } else { 0 };
        let surface = screw.move_surface(times[j], &cap.surface)?;
        let mut coedges = Vec::with_capacity(cap.coedges.len());
        for (c, forward, pcurve) in &cap.coedges {
            let edge = time_edge(&mut shell, j, *c, &mut at_time)?;
            coedges.push((edge, *forward, pcurve.clone()));
        }
        shell.face(surface, coedges);
    }
    // Walls, a row at a time. The back cap at t0 runs K's curve with the back
    // on its left, so the wall beside it runs the other way: u along the curve
    // that way, v from the row's start to its end, the loop (u0,0) (u1,0)
    // (u1,1) (u0,1).
    // The paths of K's corners, a row at a time: (row, point) -> edge.
    let mut orbit_at: HashMap<(usize, usize), u64> = HashMap::new();
    for row in 0..last {
        let (ta, tb) = (times[row], times[row + 1]);
        let segments = carry_segments(&screw, ta, tb);
        let knots_v = carry_knots(segments);
        let mut orbits: HashMap<usize, u64> = HashMap::new();
        for &(c, _) in &grazing {
            for r in [split.curves[c].start, split.curves[c].end] {
                if orbits.contains_key(&r) {
                    continue;
                }
                let (v0, v1) = (at_time(&mut shell, row, r), at_time(&mut shell, row + 1, r));
                let edge = shell.edge(carry_curve(&screw, split.points[r], ta, tb, segments)?, v0, v1, false)?;
                orbits.insert(r, edge);
                orbit_at.insert((row, r), edge);
            }
        }
        for &(c, back_forward) in &grazing {
            let tool_curve = &split.curves[c];
            let along = if back_forward { frag(tool_curve.curve.reversed(), "reversed")? } else { tool_curve.curve.clone() };
            let (from, to) = if back_forward { (tool_curve.end, tool_curve.start) } else { (tool_curve.start, tool_curve.end) };
            let mut rows = Vec::with_capacity(along.control_points.len());
            for p in &along.control_points {
                let x = frag(p.point(), "control_point")?;
                rows.push(carry_points(&screw, x, ta, tb, segments).into_iter().map(|q| Vec4::from_point(q, p.w)).collect::<Vec<_>>());
            }
            let surface = frag(NurbsSurface::new(along.degree, 5, along.knots.clone(), knots_v.clone(), rows), "wall")?;
            let [u0, u1] = frag(along.domain(), "domain")?;
            let bottom = time_edge(&mut shell, row, c, &mut at_time)?;
            let top = time_edge(&mut shell, row + 1, c, &mut at_time)?;
            let coedges = vec![
                (bottom, !back_forward, uv_line([u0, 0.0], [u1, 0.0])?),
                (orbits[&to], true, uv_line([u1, 0.0], [u1, 1.0])?),
                (top, back_forward, uv_line([u1, 1.0], [u0, 1.0])?),
                (orbits[&from], false, uv_line([u0, 1.0], [u0, 0.0])?),
            ];
            shell.face(surface, coedges);
        }
    }
    // Sliding ends: each rim arc where its neighbour's cap sits (back at t0,
    // front at t1), joined by the paths of the points where that changes.
    // Faces the tool slides within: the one-pass probe stands further off them.
    let mut sliding: std::collections::HashSet<u64> = std::collections::HashSet::new();
    let side_of: HashMap<usize, bool> = split.caps.iter().flat_map(|cap| cap.coedges.iter().map(move |&(c, _, _)| (c, cap.front))).collect();
    for slide in &split.slides {
        let centre = screw.apply(times[0], slide.centre);
        let u = screw.apply(times[0], slide.centre.add(slide.u)).sub(centre);
        let v = screw.apply(times[0], slide.centre.add(slide.v)).sub(centre);
        let row_of = |c: usize| -> Result<usize, KernelRefusal> {
            match side_of.get(&c) {
                Some(true) => Ok(last),
                Some(false) => Ok(0),
                None => Err(refuse("revolved_grazing", "a sliding end has no face beside its rim".into())),
            }
        };
        let mut coedges = Vec::new();
        let mut corners: Vec<Vec3> = Vec::new();
        let push = |coedges: &mut Vec<(u64, bool, NurbsCurve)>, corners: &mut Vec<Vec3>, edge: u64, forward: bool, curve: &NurbsCurve| -> Result<(), KernelRefusal> {
            let [d0, d1] = frag(curve.domain(), "domain")?;
            for k in 0..=8 {
                corners.push(frag(curve.evaluate(d0 + (d1 - d0) * k as f64 / 8.0), "slide")?);
            }
            coedges.push((edge, forward, plane_pcurve(centre, u, v, curve, forward)?));
            Ok(())
        };
        for (index, &(c, forward)) in slide.rim.iter().enumerate() {
            let j = row_of(c)?;
            let edge = time_edge(&mut shell, j, c, &mut at_time)?;
            push(&mut coedges, &mut corners, edge, forward, &screw.move_curve(times[j], &split.curves[c].curve)?)?;
            // On to the next arc's time along the path of the point between them.
            let (next, _) = slide.rim[(index + 1) % slide.rim.len()];
            let next_j = row_of(next)?;
            if next_j == j {
                continue;
            }
            let point = if forward { split.curves[c].end } else { split.curves[c].start };
            let rows: Vec<usize> = if next_j > j { (j..next_j).collect() } else { (next_j..j).rev().collect() };
            for row in rows {
                let Some(&orbit) = orbit_at.get(&(row, point)) else {
                    return Err(refuse("revolved_grazing", "a sliding end's rim changes side where no grazing curve meets it".into()));
                };
                let (ta, tb) = (times[row], times[row + 1]);
                let curve = carry_curve(&screw, split.points[point], ta, tb, carry_segments(&screw, ta, tb))?;
                push(&mut coedges, &mut corners, orbit, next_j > j, &curve)?;
            }
        }
        sliding.insert(shell.face(plane_patch(centre, u, v, &corners)?, coedges));
    }
    // Sliding cylinders: from the start rim where its neighbour sits to the
    // end rim where its neighbour sits, in halves seamed along generators.
    for side in &split.slide_sides {
        let row_of = |c: usize| -> Result<usize, KernelRefusal> {
            match side_of.get(&c) {
                Some(true) => Ok(last),
                Some(false) => Ok(0),
                None => Err(refuse("revolved_grazing", "a sliding side has no face beside its rim".into())),
            }
        };
        let origin = screw.apply(times[0], side.origin);
        let w = screw.apply(times[0], side.origin.add(side.w)).sub(origin);
        let rows: Vec<[usize; 2]> = side.patches.iter().map(|patch| Ok([row_of(patch.2)?, row_of(patch.3)?])).collect::<Result<_, KernelRefusal>>()?;
        if rows.iter().any(|r| *r != rows[0]) {
            return Err(refuse("revolved_grazing", "a sliding side's rim changes side along it".into()));
        }
        let [jb, ja] = rows[0];
        // Seams: the generator at each patch's start angle, rim point to rim point.
        let mut seams = Vec::with_capacity(side.patches.len());
        for patch in &side.patches {
            let (a, b) = (at_time(&mut shell, jb, patch.4[0]), at_time(&mut shell, ja, patch.4[1]));
            let line = frag(make_line(screw.apply(times[jb], split.points[patch.4[0]]), screw.apply(times[ja], split.points[patch.4[1]])), "seam")?;
            seams.push((shell.edge(line.clone(), a, b, false)?, line));
        }
        for (k, patch) in side.patches.iter().enumerate() {
            let surface = frag(make_revolution(origin, w, &seams[k].1, patch.1), "slide_side")?;
            let lower = time_edge(&mut shell, jb, patch.2, &mut at_time)?;
            let upper = time_edge(&mut shell, ja, patch.3, &mut at_time)?;
            let next = (k + 1) % side.patches.len();
            let face = shell.face(
                surface,
                vec![
                    (lower, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                    (seams[next].0, true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                    (upper, false, uv_line([1.0, 1.0], [0.0, 1.0])?),
                    (seams[k].0, false, uv_line([0.0, 1.0], [0.0, 0.0])?),
                ],
            );
            sliding.insert(face);
        }
    }
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse("assembly", format!("the assembled sweep is not valid: {:?}", issues.first())));
    }
    // A ball under one screw is proved one pass from its centre's helix
    // (`BREP_SWEEP_BALL_ONE_PASS`, default on): nothing for the walk to find.
    if !crate::solid_sweep_one_pass::ball_screw_one_pass(&screw, tool) {
        check_single_pass(&screw, &split, &body, &sliding)?;
    }
    Ok(body)
}

/// A rod (a cylinder with flat ends) TURNED about a line that crosses its
/// axis squarely between its ends, if `meridian` is one and the motion such.
///
/// Arm by arm: the rod is its arm one way from the turn's axis and its arm
/// the other way, and sweeps what they sweep. At a height z (half-width
/// w = √(r² − z²)) an arm of length L is the rectangle 0 ≤ x ≤ L, |y| ≤ w,
/// which a circle about the turn's axis of radius ρ ≤ L meets in one arc
/// holding the arm's own direction, and beyond, out to √(L² + w²), in two
/// arcs ±[a, b] with a ≤ acos(L / √(L² + r²)). Turned by more than 2a, the
/// two arcs sweep the same angles as the arc [−b, b] they leave a gap in:
/// the arm with the gap filled, its points beyond its end within
/// √(L² + r²) of the crossing point (a cap of that ball). The filled arm
/// meets every circle in one arc holding its direction, so it sweeps itself
/// where it starts and where it stops and, between, its section by the half
/// plane at its direction turned. So the sweep is the union of the filled rod
/// (one solid of revolution about the rod's axis) at both ends of the turn
/// and the two arms' sections turned.
fn rod_across_axis(screw: &Screw, origin: Vec3, axis: Vec3, meridian: &[OutlinePiece]) -> Result<Option<BrepSolid>, KernelRefusal> {
    if screw.frame.is_some() || screw.turns.is_some() || screw.omega == 0.0 || screw.c != 0.0 || screw.c2 != 0.0 {
        return Ok(None);
    }
    let Ok(w) = axis.normalized() else { return Ok(None) };
    // A rectangle against the axis: radius r from z0 to z1.
    let mut corners: Vec<[f64; 2]> = Vec::new();
    for piece in meridian {
        let OutlinePiece::Line { from, to } = piece else { return Ok(None) };
        corners.push(*from);
        corners.push(*to);
    }
    if meridian.len() != 4 {
        return Ok(None);
    }
    let r = corners.iter().map(|c| c[0]).fold(0.0f64, f64::max);
    let (z0, z1) = corners.iter().map(|c| c[1]).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), z| (lo.min(z), hi.max(z)));
    let scale = r.max(z0.abs()).max(z1.abs());
    let on = |a: f64, b: f64| (a - b).abs() <= 1e-12 * scale;
    if !(r > 0.0 && z1 > z0) || !corners.iter().all(|c| (on(c[0], 0.0) || on(c[0], r)) && (on(c[1], z0) || on(c[1], z1))) {
        return Ok(None);
    }
    // The turn's axis squarely across the rod's, between its ends.
    if w.dot(screw.e).abs() > 1e-12 {
        return Ok(None);
    }
    let offset = screw.a.sub(origin);
    let along = offset.dot(w);
    let foot = origin.add(w.scale(along));
    let apart = screw.a.sub(foot);
    if apart.sub(screw.e.scale(apart.dot(screw.e))).length() > 1e-12 * (scale + offset.length()) {
        return Ok(None);
    }
    if !(along > z0 + 1e-9 * scale && along < z1 - 1e-9 * scale) {
        return Ok(None);
    }
    let theta = screw.omega.abs() * (screw.t1 - screw.t0);
    let arms = [z1 - along, along - z0];
    let gap = arms.iter().map(|l| (l / l.hypot(r)).acos()).fold(0.0f64, f64::max);
    if !(theta > 2.0 * gap * (1.0 + 1e-9) && theta < TAU * (1.0 - 1e-9)) {
        return Err(refuse("rod_across", "a rod turned about a line through it is swept only for a turn wider than its ends' gaps and short of a whole turn".into()));
    }
    let arc = |centre: Vec3, from: Vec3, normal: Vec3, angle: f64| -> Result<NurbsCurve, KernelRefusal> {
        let surface = frag(make_revolution(centre, normal, &frag(make_line(from, from.add(normal)), "line")?, angle), "rod_across")?;
        frag(surface.iso_curve_v(0.0), "rod_across")
    };
    let line = |a: Vec3, b: Vec3| frag(make_line(a, b), "rod_across");
    // At one time: the crossing point, the rod's direction and the turn's.
    let filled_at = |t: f64| -> Result<BrepSolid, KernelRefusal> {
        let o = screw.apply(t, foot);
        let x = screw.apply(t, foot.add(w)).sub(o);
        let up = screw.e;
        let at = |rho: f64, s: f64| o.add(up.scale(rho)).add(x.scale(s));
        let n = up.cross(x);
        let (far_plus, far_minus) = (arms[0].hypot(r), arms[1].hypot(r));
        let profile = [
            arc(o, at(0.0, -far_minus), n, r.atan2(arms[1]))?,
            line(at(r, -arms[1]), at(r, arms[0]))?,
            arc(o, at(r, arms[0]), n, r.atan2(arms[0]))?,
            line(at(0.0, far_plus), at(0.0, -far_minus))?,
        ];
        frag(crate::revolve_profile_brep(&profile, o, x, TAU), "rod_across")
    };
    let o = screw.apply(screw.t0, foot);
    let x = screw.apply(screw.t0, foot.add(w)).sub(o);
    let turn_axis = screw.e.scale(screw.omega.signum());
    let section = |direction: Vec3, length: f64| -> Result<BrepSolid, KernelRefusal> {
        let at = |rho: f64, z: f64| o.add(direction.scale(rho)).add(screw.e.scale(z));
        let half = (r / length.hypot(r)).asin();
        let profile = [
            line(at(0.0, -r), at(length, -r))?,
            arc(o, at(length, -r), direction.cross(screw.e), 2.0 * half)?,
            line(at(length, r), at(0.0, r))?,
            line(at(0.0, r), at(0.0, -r))?,
        ];
        frag(crate::revolve_profile_brep(&profile, o, turn_axis, theta), "rod_across")
    };
    let parts = [filled_at(screw.t0)?, filled_at(screw.t1)?, section(x, arms[0])?, section(x.scale(-1.0), arms[1])?];
    crate::boolean_operation_nary(&parts, crate::BooleanOperation::Union).map(Some)
}

/// `BREP_SWEEP_RING_LENS_SEWN` (on by default; `=0` restores the booleans): a
/// ring slid flat short of its
/// hole's width is the filled ring's sweep less the lens its hole leaves open
/// at both ends. That lens is built here and sewn into the sweep's top and
/// bottom faces in place of the two booleans (the lens intersect and the
/// subtract): its caps are the two holes' top and bottom discs in common,
/// whose outlines become inner loops of the sweep's flat faces, and its walls
/// are the two holes' own tube surfaces, meeting in the slide's bisector
/// plane along `y(ψ) = √(ρ(ψ)² − D²/4)`, `ρ = R − r cos ψ`, `z = r sin ψ`
/// (the tube's section by that plane, interpolated through exact points and
/// read back against them). A lens that pinches at the tube's middle (`D ≥
/// 2 (R − r)`) or anything that does not validate is left to the booleans.
pub(crate) fn ring_lens_sewn_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_LENS_SEWN", true)
}

/// `BREP_SWEEP_RING_LENS_SEWN_CHECK` (default off): both built, the booleans'
/// body kept; the sewn one's volume and area compared with it
/// ([`ring_lens_check_take`]).
fn ring_lens_sewn_check_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_LENS_SEWN_CHECK", false)
}

thread_local! {
    static RING_LENS_CHECK: std::cell::Cell<(u64, u64, f64)> = const { std::cell::Cell::new((0, 0, 0.0)) };
}

/// (bodies compared, sewn bodies missing or differing past `1e-7` relative
/// in volume or area, the worst relative difference) on this thread since the
/// last call. The bar is the booleans' own error, the bench's tier: the sewn
/// body reads the closed form to `1e-10` where the booleans' body reads it
/// `2.9e-9` off (tube 2 round 6, slid 6) and `3.8e-8` off (tube 1 round 5,
/// slid 3).
pub fn ring_lens_check_take() -> (u64, u64, f64) {
    RING_LENS_CHECK.with(|cell| cell.replace((0, 0, 0.0)))
}

fn ring_lens_check_record(cut: &BrepSolid, sewn: Option<&BrepSolid>) {
    let worst = match (crate::solid_mass_properties(cut), sewn.map(crate::solid_mass_properties)) {
        (Ok(a), Some(Ok(b))) => ((a.volume - b.volume) / a.volume).abs().max(((a.surface_area - b.surface_area) / a.surface_area).abs()),
        _ => f64::INFINITY,
    };
    RING_LENS_CHECK.with(|cell| {
        let (n, bad, w) = cell.get();
        cell.set((n + 1, bad + u64::from(!(worst <= 1e-7)), w.max(worst)));
    });
}

/// The filled ring's sweep `whole` (a slide of the ring's centre from `c0` to
/// `c1`, square to its axis `w`, its tube `tube` round the circle of radius
/// `big`) less the lens both holes share, built directly. `None` where it
/// does not apply or does not validate.
fn sewn_lens(whole: &BrepSolid, c0: Vec3, c1: Vec3, w: Vec3, big: f64, tube: f64) -> Option<BrepSolid> {
    let slide = c1.sub(c0);
    let d = slide.length();
    let scale = (big + tube + d).max(1e-300);
    if !(slide.dot(w).abs() <= 1e-12 * scale && d > 1e-9 * scale && d < 2.0 * (big - tube) * (1.0 - 1e-9)) {
        return None;
    }
    let ed = slide.scale(1.0 / d);
    let ey = w.cross(ed);
    let mid = c0.add(ed.scale(0.5 * d));
    let half = 0.5 * d;
    let alpha = (half / big).acos();
    let y_top = (big * big - half * half).sqrt();
    let corner = |side: f64, z: f64| mid.add(ey.scale(side * y_top)).add(w.scale(z));
    let (t_plus, t_minus, b_plus, b_minus) = (corner(1.0, tube), corner(-1.0, tube), corner(1.0, -tube), corner(-1.0, -tube));
    // The flat faces the caps sit in.
    let shell = whole.shells.first()?;
    let flat_at = |z: f64| -> Option<usize> {
        shell.faces.iter().position(|f| {
            f.loops.len() == 1
                && f.surface.degree_u == 1
                && f.surface.degree_v == 1
                && f.surface.control_points.iter().flatten().all(|c| c.point().map(|p| (p.sub(c0).dot(w) - z).abs() <= 1e-12 * scale).unwrap_or(false))
        })
    };
    let (top, bottom) = (flat_at(tube)?, flat_at(-tube)?);
    // Edges. Arcs: A on the circle about c0 (the lens's side away from c0's
    // far rim: angles −α..α from ed), B about c1 (π−α..π+α).
    let arc = |centre: Vec3, from: f64, to: f64| crate::make_arc(centre, ed, ey, big, from, to).ok();
    let a_top = arc(c0.add(w.scale(tube)), -alpha, alpha)?; // t_minus -> t_plus
    let b_top = arc(c1.add(w.scale(tube)), PI - alpha, PI + alpha)?; // t_plus -> t_minus
    let a_bot = arc(c0.add(w.scale(-tube)), -alpha, alpha)?; // b_minus -> b_plus
    let b_bot = arc(c1.add(w.scale(-tube)), PI - alpha, PI + alpha)?; // b_plus -> b_minus
    // The walls' carriers: each hole's inner tube half turned a whole turn
    // about its axis, its generatrix drawn opposite the lens (the turn's seam
    // away from the wall).
    let wall_surface = |centre: Vec3, away: Vec3| -> Option<NurbsSurface> {
        let n = w.cross(away);
        let at = |rho: f64, z: f64| centre.add(away.scale(rho)).add(w.scale(z));
        let from = at(big, -tube);
        let half_round = crate::make_revolution(at(big, 0.0), n, &crate::make_line(from, from.add(n)).ok()?, PI).ok()?.iso_curve_v(0.0).ok()?;
        crate::make_revolution(centre, w, &half_round, TAU).ok()
    };
    let (wall_a, wall_b) = (wall_surface(c0, ed.scale(-1.0))?, wall_surface(c1, ed)?);
    // Seams, bottom to top, through exact points of the bisector section, with
    // their pcurves on both walls interpolated through the same points' feet
    // at the same parameters; all three read back at the mid-samples.
    // (Each seam in two halves, split at the tube's waist ψ = 0: the walls'
    // half-round generatrix is two quarter arcs, its parameter speed jumps
    // there, and a pcurve through it would kink.)
    let seam = |side: f64, from: f64, to: f64| -> Option<(NurbsCurve, NurbsCurve, NurbsCurve)> {
        let at = |psi: f64| {
            let rho = big - tube * psi.cos();
            mid.add(ey.scale(side * (rho * rho - half * half).max(0.0).sqrt())).add(w.scale(tube * psi.sin()))
        };
        let feet = |surface: &NurbsSurface, points: &[Vec3]| -> Option<Vec<Vec3>> {
            let mut out: Vec<Vec3> = Vec::with_capacity(points.len());
            for point in points {
                // From the last foot; where that lands off the point (the tube
                // turning fast near its top and bottom), the global reading.
                let seeded = out.last().and_then(|last| crate::project_point_to_surface_seeded(surface, *point, last.x, last.y).ok());
                let q = match seeded {
                    Some(q) if q.distance <= 1e-7 * scale => q,
                    _ => crate::project_point_to_surface(surface, *point).ok()?,
                };
                // The projector stops near 1e-9; the point is on the carrier
                // exactly, so Gauss-Newton on |S(u, v) − p|² takes the foot to
                // rounding.
                let (mut u, mut v) = (q.u, q.v);
                for _ in 0..4 {
                    let d = surface.derivatives(u, v, 1).ok()?;
                    let (f, su, sv) = (d[0][0].sub(*point), d[1][0], d[0][1]);
                    let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
                    let (g0, g1) = (su.dot(f), sv.dot(f));
                    let det = a * c - b * b;
                    if !(det > 0.0) {
                        return None;
                    }
                    u -= (c * g0 - b * g1) / det;
                    v -= (a * g1 - b * g0) / det;
                }
                if !(surface.evaluate(u, v).ok()?.sub(*point).length() <= 1e-11 * scale) {
                    return None;
                }
                out.push(Vec3::new(u, v, 0.0));
            }
            Some(out)
        };
        let mut n = 16usize;
        loop {
            let psis: Vec<f64> = (0..=n).map(|k| from + (to - from) * k as f64 / n as f64).collect();
            let points: Vec<Vec3> = psis.iter().map(|&psi| at(psi)).collect();
            let params: Vec<f64> = psis.iter().map(|psi| (psi - from) / (to - from)).collect();
            let curve = crate::interpolate_curve(&points, 3, &params).ok()?;
            let on_a = crate::interpolate_curve(&feet(&wall_a, &points)?, 3, &params).ok()?;
            let on_b = crate::interpolate_curve(&feet(&wall_b, &points)?, 3, &params).ok()?;
            let mut worst = 0.0f64;
            for k in 0..n {
                let psi = 0.5 * (psis[k] + psis[k + 1]);
                let t = (psi - from) / (to - from);
                let p = curve.evaluate(t).ok()?;
                worst = worst.max(p.sub(at(psi)).length());
                for (surface, pcurve) in [(&wall_a, &on_a), (&wall_b, &on_b)] {
                    let uv = pcurve.evaluate(t).ok()?;
                    worst = worst.max(surface.evaluate(uv.x, uv.y).ok()?.sub(p).length());
                }
            }
            if worst <= 1e-10 * scale {
                return Some((curve, on_a, on_b));
            }
            if n >= 4096 {
                return None;
            }
            n *= 2;
        }
    };
    let (lower_plus, upper_plus) = (seam(1.0, -0.5 * PI, 0.0)?, seam(1.0, 0.0, 0.5 * PI)?);
    let (lower_minus, upper_minus) = (seam(-1.0, -0.5 * PI, 0.0)?, seam(-1.0, 0.0, 0.5 * PI)?);
    let waist = |side: f64| mid.add(ey.scale(side * ((big - tube).powi(2) - half * half).sqrt()));
    let (m_plus, m_minus) = (waist(1.0), waist(-1.0));
    // Ids past every existing one.
    let mut next = whole.vertices.iter().map(|v| v.id).chain(whole.edges.iter().map(|e| e.id)).chain(shell.faces.iter().flat_map(|f| std::iter::once(f.id).chain(f.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id)))))).max().unwrap_or(0) + 1;
    let mut id = || {
        next += 1;
        next
    };
    let mut body = whole.clone();
    let vertex = |body: &mut BrepSolid, id: u64, point: Vec3| body.vertices.push(VertexRecord { id, point });
    let (vt_plus, vt_minus, vb_plus, vb_minus, vm_plus, vm_minus) = (id(), id(), id(), id(), id(), id());
    vertex(&mut body, vm_plus, m_plus);
    vertex(&mut body, vm_minus, m_minus);
    vertex(&mut body, vt_plus, t_plus);
    vertex(&mut body, vt_minus, t_minus);
    vertex(&mut body, vb_plus, b_plus);
    vertex(&mut body, vb_minus, b_minus);
    let mut edge = |body: &mut BrepSolid, curve: NurbsCurve, start: u64, end: u64| -> Option<u64> {
        let [t0, t1] = curve.domain().ok()?;
        let e = id();
        body.edges.push(EdgeRecord { id: e, curve, t0, t1, start_vertex_id: start, end_vertex_id: end, degenerate: false, name: None });
        Some(e)
    };
    // The arcs' ends must be the corners they are named for.
    for (curve, a, b) in [(&a_top, t_minus, t_plus), (&b_top, t_plus, t_minus), (&a_bot, b_minus, b_plus), (&b_bot, b_plus, b_minus), (&lower_plus.0, b_plus, m_plus), (&upper_plus.0, m_plus, t_plus), (&lower_minus.0, b_minus, m_minus), (&upper_minus.0, m_minus, t_minus)] {
        let [t0, t1] = curve.domain().ok()?;
        if curve.evaluate(t0).ok()?.sub(a).length() > 1e-9 * scale || curve.evaluate(t1).ok()?.sub(b).length() > 1e-9 * scale {
            return None;
        }
    }
    // The seams' pcurves, built with them: (edge, wall, pcurve along the edge).
    let mut given: Vec<(u64, usize, NurbsCurve)> = Vec::new();
    let e_a_top = edge(&mut body, a_top, vt_minus, vt_plus)?;
    let e_b_top = edge(&mut body, b_top, vt_plus, vt_minus)?;
    let e_a_bot = edge(&mut body, a_bot, vb_minus, vb_plus)?;
    let e_b_bot = edge(&mut body, b_bot, vb_plus, vb_minus)?;
    let mut seam_edge = |body: &mut BrepSolid, piece: (NurbsCurve, NurbsCurve, NurbsCurve), start: u64, end: u64, given: &mut Vec<(u64, usize, NurbsCurve)>| -> Option<u64> {
        let (curve, on_a, on_b) = piece;
        let e = edge(body, curve, start, end)?;
        given.extend([(e, 1, on_a), (e, 2, on_b)]);
        Some(e)
    };
    let e_lp = seam_edge(&mut body, lower_plus, vb_plus, vm_plus, &mut given)?;
    let e_up = seam_edge(&mut body, upper_plus, vm_plus, vt_plus, &mut given)?;
    let e_lm = seam_edge(&mut body, lower_minus, vb_minus, vm_minus, &mut given)?;
    let e_um = seam_edge(&mut body, upper_minus, vm_minus, vt_minus, &mut given)?;
    let curve_of = |body: &BrepSolid, e: u64| body.edges.iter().find(|x| x.id == e).map(|x| x.curve.clone());
    // A coedge of edge `e` on `surface`, forward or reversed, with its pcurve
    // fitted along the coedge.
    let coedge = |body: &BrepSolid, id: u64, surface: &NurbsSurface, e: u64, forward: bool, given: &[(u64, usize, NurbsCurve)], wall: usize| -> Option<CoedgeRecord> {
        if let Some((_, _, pcurve)) = given.iter().find(|(edge, on, _)| *edge == e && *on == wall) {
            let pcurve = if forward { pcurve.clone() } else { pcurve.reversed().ok()? };
            return Some(CoedgeRecord { id, edge_id: e, forward, pcurve });
        }
        let fit = crate::fit_pcurve_on_surface(surface, &curve_of(body, e)?).ok()?;
        if !(fit.report.residual <= 1e-7 * scale) {
            return None;
        }
        let pcurve = if forward { fit.curve } else { fit.curve.reversed().ok()? };
        Some(CoedgeRecord { id, edge_id: e, forward, pcurve })
    };
    // The loop's signed area in its face's parameters.
    let uv_area = |coedges: &[CoedgeRecord]| -> f64 {
        let mut area = 0.0;
        for c in coedges {
            let Ok([p0, p1]) = c.pcurve.domain() else { return f64::NAN };
            let samples = 64;
            let mut last = match c.pcurve.evaluate(p0) { Ok(p) => p, Err(_) => return f64::NAN };
            for k in 1..=samples {
                let Ok(p) = c.pcurve.evaluate(p0 + (p1 - p0) * k as f64 / samples as f64) else { return f64::NAN };
                area += last.x * p.y - p.x * last.y;
                last = p;
            }
        }
        0.5 * area
    };
    // Caps: the top's hole runs clockwise seen from outside (from +w), the
    // bottom's clockwise seen from −w.
    for (face_index, loop_edges) in [(top, [(e_a_top, false), (e_b_top, false)]), (bottom, [(e_a_bot, true), (e_b_bot, true)])] {
        let surface = body.shells[0].faces[face_index].surface.clone();
        let mut coedges = Vec::new();
        for (e, forward) in loop_edges {
            let c = id();
            coedges.push(coedge(&body, c, &surface, e, forward, &given, 0)?);
        }
        // An inner loop runs opposite its face's outer loop in the parameters.
        let outer = uv_area(&body.shells[0].faces[face_index].loops[0].coedges);
        let inner = uv_area(&coedges);
        if !(outer.is_finite() && inner.is_finite() && outer * inner < 0.0) {
            return None;
        }
        let loop_id = id();
        body.shells[0].faces[face_index].loops.push(LoopRecord { id: loop_id, coedges });
    }
    // Walls: A uses the arcs the caps reversed and B likewise, the seams once each way.
    for (wall, surface, centre, loop_edges) in [
        (1usize, wall_a, c0, vec![(e_a_top, true), (e_up, false), (e_lp, false), (e_a_bot, false), (e_lm, true), (e_um, true)]),
        (2usize, wall_b, c1, vec![(e_b_top, true), (e_um, false), (e_lm, false), (e_b_bot, false), (e_lp, true), (e_up, true)]),
    ] {
        let mut coedges = Vec::new();
        for (e, forward) in loop_edges {
            let c = id();
            coedges.push(coedge(&body, c, &surface, e, forward, &given, wall)?);
        }
        let area = uv_area(&coedges);
        if !(area.is_finite() && area != 0.0) {
            return None;
        }
        // Counter-clockwise in the parameters: the loop's outward is the
        // surface's normal side; it must face into the lens (towards the
        // hole's own axis).
        let same_sense = area > 0.0;
        let face_id = id();
        let loop_id = id();
        let face = FaceRecord { id: face_id, surface, same_sense, loops: vec![LoopRecord { id: loop_id, coedges }], name: None };
        // Sense check at the wall's middle: towards the hole's axis.
        let middle = mid.add(ed.scale(if centre.sub(c0).length() == 0.0 { big - tube - half } else { -(big - tube - half) }));
        let q = crate::project_point_to_surface(&face.surface, middle).ok()?;
        let n = face.surface.normal(q.u, q.v).ok()?;
        let n = if face.same_sense { n } else { n.scale(-1.0) };
        let toward_axis = centre.sub(middle);
        let toward_axis = toward_axis.sub(w.scale(toward_axis.dot(w)));
        if !(n.dot(toward_axis) > 0.0) {
            return None;
        }
        body.shells[0].faces.push(face);
    }
    // The lens runs through from the top face to the bottom: one handle more.
    body.genus = whole.genus + 1;
    body.mass_properties_cache = Default::default();
    if body.validate().iter().any(|issue| issue.severity == "error") {
        return None;
    }
    Some(body)
}

/// A ring (or another tool with a hole about its axis walled straight
/// through) lying flat to its motion (a turn about an axis parallel to the
/// tool's, a slide square to it), if `meridian` is one and the motion such.
///
/// Every point then moves in its own plane square to the ring's axis, where
/// the ring's section is an annulus. The tube passes over where the hole was,
/// so the sweep does not pass each point once; but an annulus sweeps what its
/// filled disc sweeps as soon as it travels far enough that no point stays in
/// the hole from start to end (the hole at the start and at the end share
/// nothing: travel beyond the hole's greatest width, the tube's centre
/// circle). So the ring sweeps what the FILLED ring sweeps: a tool with a
/// half-round rim and flat faces, which is swept whole. A whole turn sweeps
/// the solid of revolution of that tool's shadow about the turn's axis.
fn flat_ring(screw: &Screw, origin: Vec3, axis: Vec3, radial: Vec3, meridian: &[OutlinePiece]) -> Result<Option<BrepSolid>, KernelRefusal> {
    if screw.frame.is_some() || screw.turns.is_some() || screw.c2 != 0.0 {
        return Ok(None);
    }
    let Ok(w) = axis.normalized() else { return Ok(None) };
    let flat = if screw.omega != 0.0 { screw.e.cross(w).length() <= 1e-12 && screw.c == 0.0 } else { screw.e.dot(w).abs() <= 1e-12 };
    if !flat {
        return Ok(None);
    }
    // The holed tool: a torus (`torus`: its tube), or straight-sided pieces
    // whose hole is one upright wall from the tool's top to its bottom.
    // `big`: half the hole's greatest width. `filled_meridian`: the tool
    // with its hole closed, flat across at its top and bottom.
    let (big, level, torus, filled_meridian): (f64, f64, Option<f64>, Vec<OutlinePiece>) = match meridian {
        [OutlinePiece::Arc { centre, radius, start, end }] => {
            let (big, tube, level) = (centre[0], *radius, centre[1]);
            if ((end - start).abs() - TAU).abs() >= 1e-12 || big <= tube {
                return Ok(None);
            }
            (
                big,
                level,
                Some(tube),
                vec![
                    OutlinePiece::Line { from: [0.0, level - tube], to: [big, level - tube] },
                    OutlinePiece::Arc { centre: [big, level], radius: tube, start: -0.5 * PI, end: 0.5 * PI },
                    OutlinePiece::Line { from: [big, level + tube], to: [0.0, level + tube] },
                    OutlinePiece::Line { from: [0.0, level + tube], to: [0.0, level - tube] },
                ],
            )
        }
        pieces => {
            let mut ends: Vec<([f64; 2], [f64; 2])> = Vec::with_capacity(pieces.len());
            for piece in pieces {
                let OutlinePiece::Line { from, to } = piece else { return Ok(None) };
                ends.push((*from, *to));
            }
            let scale = ends.iter().flat_map(|(a, b)| [a[0].abs(), a[1].abs(), b[0].abs(), b[1].abs()]).fold(0.0f64, f64::max).max(1e-300);
            let inner = ends.iter().flat_map(|(a, b)| [a[0], b[0]]).fold(f64::INFINITY, f64::min);
            let (low, high) = ends.iter().flat_map(|(a, b)| [a[1], b[1]]).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), z| (lo.min(z), hi.max(z)));
            if ends.len() < 3 || inner <= 1e-9 * scale {
                return Ok(None);
            }
            let walls: Vec<usize> = (0..ends.len())
                .filter(|&k| {
                    let (a, b) = ends[k];
                    (a[0] - inner).abs() <= 1e-12 * scale && (b[0] - inner).abs() <= 1e-12 * scale && (a[1].min(b[1]) - low).abs() <= 1e-12 * scale && (a[1].max(b[1]) - high).abs() <= 1e-12 * scale
                })
                .collect();
            let [wall] = walls.as_slice() else { return Ok(None) };
            let (from, to) = ends[*wall];
            // From the wall's end round the outside back to its start, then
            // across to the axis and along it.
            let mut filled = vec![OutlinePiece::Line { from: [0.0, to[1]], to }];
            for k in 1..pieces.len() {
                filled.push(pieces[(wall + k) % pieces.len()].clone());
            }
            filled.push(OutlinePiece::Line { from, to: [0.0, from[1]] });
            filled.push(OutlinePiece::Line { from: [0.0, from[1]], to: [0.0, to[1]] });
            (inner, 0.5 * (low + high), None, filled)
        }
    };
    let centre0 = origin.add(w.scale(level));
    let filled = SolidTool::Revolved { axis_origin: origin, axis_direction: w, radial, meridian: filled_meridian };
    let short = || refuse("ring_flat", "a ring lying flat to its motion travels less than its hole is wide: the hole is not swept shut".into());
    if screw.omega == 0.0 {
        if screw.c.abs() * (screw.t1 - screw.t0) <= 2.0 * big * (1.0 + 1e-9) {
            let Some(tube) = torus else { return Err(short()) };
            // Short of its hole: what stays open, all the way through, is the
            // lens the hole shares where the ring starts and where it stops.
            // The hole at one place: inside the ring's inner half, between
            // its top and bottom planes.
            let hole = |t: f64| -> Result<BrepSolid, KernelRefusal> {
                let o = screw.apply(t, centre0);
                let u = frag(radial.sub(w.scale(radial.dot(w))).normalized(), "ring_flat")?;
                let at = |rho: f64, z: f64| o.add(u.scale(rho)).add(w.scale(z));
                let n = w.cross(u);
                let from = at(big, -tube);
                let inner = frag(frag(make_revolution(at(big, 0.0), n, &frag(make_line(from, from.add(n)), "line")?, PI), "ring_flat")?.iso_curve_v(0.0), "ring_flat")?;
                let profile = [
                    frag(make_line(at(0.0, -tube), at(big, -tube)), "ring_flat")?,
                    inner,
                    frag(make_line(at(big, tube), at(0.0, tube)), "ring_flat")?,
                    frag(make_line(at(0.0, tube), at(0.0, -tube)), "ring_flat")?,
                ];
                frag(crate::revolve_profile_brep(&profile, o, w, TAU), "ring_flat")
            };
            let (sewn_on, sewn_check) = (ring_lens_sewn_on(), ring_lens_sewn_check_on());
            if sewn_on || sewn_check {
                let whole = sweep_with(screw, &filled)?;
                let c0 = screw.apply(screw.t0, centre0);
                let c1 = screw.apply(screw.t1, centre0);
                let sewn = sewn_lens(&whole, c0, c1, w, big, tube);
                if sewn_check {
                    let lens = crate::boolean_operation_nary(&[hole(screw.t0)?, hole(screw.t1)?], crate::BooleanOperation::Intersect)?;
                    let cut = crate::boolean_operation_nary(&[whole, lens], crate::BooleanOperation::Subtract)?;
                    ring_lens_check_record(&cut, sewn.as_ref());
                    return Ok(Some(cut));
                }
                if let Some(body) = sewn {
                    return Ok(Some(body));
                }
            }
            let lens = crate::boolean_operation_nary(&[hole(screw.t0)?, hole(screw.t1)?], crate::BooleanOperation::Intersect)?;
            let whole = sweep_with(screw, &filled)?;
            return crate::boolean_operation_nary(&[whole, lens], crate::BooleanOperation::Subtract).map(Some);
        }
        return sweep_with(screw, &filled).map(Some);
    }
    // A turn: the ring's centre runs a circle of radius `reach` about the axis.
    let start_centre = screw.apply(screw.t0, centre0);
    let from_axis = start_centre.sub(screw.a);
    let across = from_axis.sub(screw.e.scale(from_axis.dot(screw.e)));
    let reach = across.length();
    let theta = screw.omega.abs() * (screw.t1 - screw.t0);
    if theta >= TAU * (1.0 - 1e-12) {
        let Some(tube) = torus else {
            return Err(refuse("ring_flat", "a whole turn of a holed tool in its own plane is swept for a ring only".into()));
        };
        if reach <= (big + tube) * (1.0 + 1e-9) {
            return Err(refuse("ring_flat", "a ring turned a whole turn in its own plane reaches the turn's axis".into()));
        }
        let out = frag(across.normalized(), "ring_flat")?;
        let foot = start_centre.sub(across);
        let at = |rho: f64, z: f64| foot.add(out.scale(rho)).add(screw.e.scale(z));
        let half_round = |centre: Vec3, from: Vec3| -> Result<NurbsCurve, KernelRefusal> {
            let n = out.cross(screw.e);
            let surface = frag(make_revolution(centre, n, &frag(make_line(from, from.add(n)), "line")?, PI), "ring_flat")?;
            frag(surface.iso_curve_v(0.0), "ring_flat")
        };
        let profile = [
            frag(make_line(at(reach - big, -tube), at(reach + big, -tube)), "ring_flat")?,
            half_round(at(reach + big, 0.0), at(reach + big, -tube))?,
            frag(make_line(at(reach + big, tube), at(reach - big, tube)), "ring_flat")?,
            half_round(at(reach - big, 0.0), at(reach - big, tube))?,
        ];
        return frag(crate::revolve_profile_brep(&profile, foot, screw.e, TAU), "ring_flat").map(Some);
    }
    let covered = if theta >= PI { big < reach } else { reach * (0.5 * theta).sin() > big * (1.0 + 1e-9) };
    if !covered {
        return Err(short());
    }
    sweep_with(screw, &filled).map(Some)
}

/// The sweep of `tool` along `motion`, where it lies in `clip` (the caller
/// cuts a workpiece inside that region). A TURN about the clip's own axis
/// whose two end positions stay outside the clip's wedge covers the wedge
/// wholly with the tool's shadow in (ρ, z): that shadow, turned over the wedge,
/// is the sweep there (a convex polyhedron's shadow is bounded by conic arcs
/// from its edges and straight lines from its faces, so every surface is
/// exact); a turn about that axis whose ends reach the wedge is refused. A
/// screw or turn about the region's axis whose tool reaches far past the
/// region's radius sweeps only the tool's part that can reach it (see
/// `trim_to_clip`). Any other motion gets the whole sweep ([`sweep_solid`]).
pub fn sweep_solid_clipped(tool: &SolidTool, motion: &SolidMotion, clip: &crate::SweepClip) -> Result<BrepSolid, KernelRefusal> {
    let screw = Screw::from_motion(motion)?;
    if let (SolidTool::Polyhedron { vertices, faces }, Some(wedge)) = (tool, clip.angle) {
        let axis = clip.axis_direction.normalized().map_err(|_| input("clip_axis", "the clip's axis has no direction".into()))?;
        let same_axis = screw.c == 0.0 && screw.c2 == 0.0
            && screw.e.cross(axis).length() < 1e-12
            && screw.a.sub(clip.axis_origin).cross(axis).length() < 1e-9 * (1.0 + clip.radius[1]);
        if same_axis {
            return turn_shadow(&screw, vertices, faces, clip, axis, wedge);
        }
    }
    if let SolidTool::Polyhedron { vertices, faces } = tool {
        if let Some((vertices, faces)) = trim_to_clip(&screw, vertices, faces, clip)? {
            return sweep_solid(&SolidTool::Polyhedron { vertices, faces }, motion);
        }
    }
    sweep_solid(tool, motion)
}

/// The sweep of `tool` along a turn whose rise grows as it turns, where it lies
/// in `clip` (as [`sweep_solid_clipped`]). Its front and back are those at
/// the middle of the motion; a tool whose faces change side along it is refused.
pub fn sweep_solid_rising(tool: &SolidTool, motion: &RisingTurn, clip: &crate::SweepClip) -> Result<BrepSolid, KernelRefusal> {
    let screw = Screw::from_rising(motion)?;
    let SolidTool::Polyhedron { vertices, faces } = tool else {
        return Err(refuse("rising_tool", "a growing rise sweeps a polyhedron only".into()));
    };
    let (vertices, faces) = trim_to_clip(&screw, vertices, faces, clip)?.unwrap_or_else(|| (vertices.clone(), faces.clone()));
    sweep_with(&screw, &SolidTool::Polyhedron { vertices, faces })
}

/// The sweep of `tool` along `motion` as parts whose union it is (a caller cuts
/// a workpiece by all of them). One part where the whole sweep passes once. A
/// convex polyhedron whose sweep folds over itself (a face facing away from a
/// screw's or turn's axis grazes along a line inside it, and the faces beyond
/// that line are swept over again) is split by the plane through the axis and
/// that line: each half grazes that face only along its cut edge, and is swept
/// whole. The halves' walls share cylinders about the axis.
pub fn sweep_solid_parts(tool: &SolidTool, motion: &SolidMotion) -> Result<Vec<BrepSolid>, KernelRefusal> {
    let folded = match sweep_solid(tool, motion) {
        Ok(body) => return Ok(vec![body]),
        Err(error) if refused_as(&error) == Some("folds") => error,
        Err(error) => return Err(error),
    };
    let SolidTool::Polyhedron { vertices, faces } = tool else { return Err(folded) };
    let screw = Screw::from_motion(motion)?;
    if screw.omega == 0.0 {
        return Err(folded);
    }
    let centroid = vertices.iter().fold(Vec3::new(0.0, 0.0, 0.0), |a, &p| a.add(p)).scale(1.0 / vertices.len() as f64);
    let mut planes: Vec<(Vec3, f64)> = Vec::new();
    for face in faces {
        let (p, q, r) = (vertices[face[0]], vertices[face[1]], vertices[face[2]]);
        let mut n = frag(q.sub(p).cross(r.sub(p)).normalized(), "face_normal")?;
        if n.dot(p.sub(centroid)) < 0.0 {
            n = n.scale(-1.0);
        }
        planes.push((n, n.dot(p)));
    }
    // A face square to the axis' plane through it (its normal square to the
    // axis) graze along the line through its foot from the axis; it folds if
    // it faces away from the axis and that line crosses it.
    for (n, d) in planes.clone() {
        if n.dot(screw.e).abs() > 1e-12 {
            continue;
        }
        let foot = screw.a.add(n.scale(d - n.dot(screw.a)));
        let radial = foot.sub(screw.a).sub(screw.e.scale(foot.sub(screw.a).dot(screw.e)));
        if radial.dot(n) <= 0.0 {
            continue; // faces the axis: its grazing line is the sweep's inner wall
        }
        let cut = match screw.e.cross(radial).normalized() {
            Ok(m) => m,
            Err(_) => continue,
        };
        let offset = cut.dot(screw.a);
        let mut parts = Vec::with_capacity(2);
        for sense in [1.0, -1.0] {
            let mut half = planes.clone();
            half.push((cut.scale(sense), offset * sense));
            let (corners, loops) = polyhedron_of_planes(&half)?;
            if corners.len() < 4 || loops.len() < 4 {
                parts.clear();
                break;
            }
            match sweep_solid(&SolidTool::Polyhedron { vertices: corners, faces: loops }, motion) {
                Ok(body) => parts.push(body),
                Err(_) => {
                    parts.clear();
                    break;
                }
            }
        }
        if parts.len() == 2 {
            return Ok(parts);
        }
    }
    Err(folded)
}

/// A screw or turn about the clip's axis maps the clip's cylinder (radius
/// ≤ R) onto itself, so inside it the sweep of the tool is the sweep of any
/// part of the tool holding the tool's part inside the cylinder. A tool
/// reaching far past it (a bar pointing at the axis) is replaced by its part
/// inside the prism over the shadow's piece of a slightly larger disc, with
/// that disc's arc replaced by the two tangents at its ends: those faces graze
/// only along their own end edges, where the far ends of a long tool would
/// fold the sweep over itself. None when the tool needs no trim.
fn trim_to_clip(screw: &Screw, vertices: &[Vec3], faces: &[Vec<usize>], clip: &crate::SweepClip) -> Result<Option<(Vec<Vec3>, Vec<Vec<usize>>)>, KernelRefusal> {
    let Ok(axis) = clip.axis_direction.normalized() else { return Ok(None) };
    let same_axis = screw.omega != 0.0
        && screw.e.cross(axis).length() < 1e-12
        && screw.a.sub(clip.axis_origin).cross(axis).length() < 1e-9 * (1.0 + clip.radius[1]);
    if !same_axis || !clip.radius[1].is_finite() {
        return Ok(None);
    }
    let ez = axis;
    let ex = Vec3::new(1.0, 0.0, 0.0).sub(ez.scale(ez.x)).normalized().or_else(|_| Vec3::new(0.0, 1.0, 0.0).sub(ez.scale(ez.y)).normalized()).map_err(|e| input("clip_axis", e))?;
    let ey = ez.cross(ex);
    let flat = |p: Vec3| {
        let r = p.sub(clip.axis_origin);
        [r.dot(ex), r.dot(ey)]
    };
    let radius = clip.radius[1] * 1.02;
    let shadow: Vec<[f64; 2]> = vertices.iter().map(|&p| flat(p)).collect();
    if shadow.iter().all(|q| q[0].hypot(q[1]) <= radius) {
        return Ok(None);
    }
    // The shadow's convex hull, counter-clockwise.
    let mut points = shadow.clone();
    points.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
    let cross = |o: [f64; 2], a: [f64; 2], b: [f64; 2]| (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    let mut hull: Vec<[f64; 2]> = Vec::new();
    for pass in 0..2 {
        let start = hull.len();
        let ordered: Vec<[f64; 2]> = if pass == 0 { points.clone() } else { points.iter().rev().copied().collect() };
        for p in ordered {
            while hull.len() >= start + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull.pop();
    }
    // Where the hull's boundary leaves and re-enters the disc.
    let m = hull.len();
    let inside = |q: [f64; 2]| q[0].hypot(q[1]) <= radius;
    // Leave a tool that does not reach the disc, or wraps the axis.
    let mut nearest = f64::INFINITY;
    let mut wraps = true;
    for k in 0..m {
        let (a, b) = (hull[k], hull[(k + 1) % m]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let t = (-(a[0] * d[0] + a[1] * d[1]) / (d[0] * d[0] + d[1] * d[1])).clamp(0.0, 1.0);
        nearest = nearest.min((a[0] + d[0] * t).hypot(a[1] + d[1] * t));
        if cross(a, b, [0.0, 0.0]) < 0.0 {
            wraps = false;
        }
    }
    if wraps || nearest >= radius || (0..m).all(|k| inside(hull[k])) {
        return Ok(None);
    }
    let mut leave: Vec<f64> = Vec::new();
    let mut enter: Vec<f64> = Vec::new();
    for k in 0..m {
        let (a, b) = (hull[k], hull[(k + 1) % m]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let (qa, qb, qc) = (d[0] * d[0] + d[1] * d[1], 2.0 * (a[0] * d[0] + a[1] * d[1]), a[0] * a[0] + a[1] * a[1] - radius * radius);
        let disc = qb * qb - 4.0 * qa * qc;
        if disc <= 0.0 {
            continue;
        }
        for t in [(-qb - disc.sqrt()) / (2.0 * qa), (-qb + disc.sqrt()) / (2.0 * qa)] {
            if t > 0.0 && t < 1.0 {
                let q = [a[0] + d[0] * t, a[1] + d[1] * t];
                let outward = 2.0 * (q[0] * d[0] + q[1] * d[1]) > 0.0;
                (if outward { &mut leave } else { &mut enter }).push(q[1].atan2(q[0]));
            }
        }
    }
    let ([a], [b]) = (leave.as_slice(), enter.as_slice()) else {
        return Err(refuse("clip_trim", "the tool's shadow leaves the clip's disc more than once".into()));
    };
    let width = (b - a).rem_euclid(TAU);
    if width >= 0.9 * PI {
        return Err(refuse("clip_trim", "the tool's shadow spans too much of the clip's circle to be trimmed".into()));
    }
    // Planes of the tool, and the two tangents (outward normals in the plane).
    let mut planes: Vec<(Vec3, f64)> = Vec::new();
    let centroid = vertices.iter().fold(Vec3::new(0.0, 0.0, 0.0), |s, &p| s.add(p)).scale(1.0 / vertices.len() as f64);
    for face in faces {
        let (p, q, r) = (vertices[face[0]], vertices[face[1]], vertices[face[2]]);
        let mut n = frag(q.sub(p).cross(r.sub(p)).normalized(), "face_normal")?;
        if n.dot(p.sub(centroid)) < 0.0 {
            n = n.scale(-1.0);
        }
        planes.push((n, n.dot(p)));
    }
    for angle in [*a, *b] {
        let n = ex.scale(angle.cos()).add(ey.scale(angle.sin()));
        planes.push((n, n.dot(clip.axis_origin) + radius));
    }
    Ok(Some(polyhedron_of_planes(&planes)?))
}

/// The convex polyhedron {n · p ≤ d for every plane}: its corners (where three
/// planes meet inside all) and a face per plane that bounds it.
pub fn polyhedron_of_planes(planes: &[(Vec3, f64)]) -> Result<(Vec<Vec3>, Vec<Vec<usize>>), KernelRefusal> {
    let k = planes.len();
    let mut corners: Vec<Vec3> = Vec::new();
    let scale = planes.iter().map(|p| p.1.abs()).fold(1.0f64, f64::max);
    for i in 0..k {
        for j in i + 1..k {
            for l in j + 1..k {
                let (a, b, c) = (planes[i].0, planes[j].0, planes[l].0);
                let det = a.dot(b.cross(c));
                if det.abs() < 1e-12 {
                    continue;
                }
                let p = b.cross(c).scale(planes[i].1).add(c.cross(a).scale(planes[j].1)).add(a.cross(b).scale(planes[l].1)).scale(1.0 / det);
                if planes.iter().all(|(n, d)| n.dot(p) - d <= 1e-9 * scale) && !corners.iter().any(|q| q.sub(p).length() <= 1e-9 * scale) {
                    corners.push(p);
                }
            }
        }
    }
    let mut faces = Vec::new();
    for (n, d) in planes {
        let on: Vec<usize> = (0..corners.len()).filter(|&c| (n.dot(corners[c]) - d).abs() <= 1e-9 * scale).collect();
        if on.len() < 3 {
            continue;
        }
        let middle = on.iter().fold(Vec3::new(0.0, 0.0, 0.0), |s, &c| s.add(corners[c])).scale(1.0 / on.len() as f64);
        let u = frag(corners[on[0]].sub(middle).normalized(), "face_axis")?;
        let v = n.cross(u);
        let mut ordered = on.clone();
        ordered.sort_by(|&p, &q| {
            let (rp, rq) = (corners[p].sub(middle), corners[q].sub(middle));
            rp.dot(v).atan2(rp.dot(u)).total_cmp(&rq.dot(v).atan2(rq.dot(u)))
        });
        faces.push(ordered);
    }
    Ok((corners, faces))
}

fn turn_shadow(screw: &Screw, vertices: &[Vec3], faces: &[Vec<usize>], clip: &crate::SweepClip, axis: Vec3, wedge: [f64; 2]) -> Result<BrepSolid, KernelRefusal> {
    let ez = axis;
    let ex = clip.reference.sub(ez.scale(clip.reference.dot(ez))).normalized().map_err(|_| input("clip_reference", "the clip's reference lies along its axis".into()))?;
    let ey = ez.cross(ex);
    // Turning by ω t about e = ±ez moves angles about ez by ±ω t.
    let rate = screw.omega * screw.e.dot(ez).signum();
    let local = |p: Vec3| {
        let r = p.sub(clip.axis_origin);
        [r.dot(ex), r.dot(ey), r.dot(ez)]
    };
    let corners: Vec<[f64; 3]> = vertices.iter().map(|&p| local(p)).collect();
    // The tool's angular extent about the axis (its corners'; the tool must
    // keep off the axis, so its shadow on the plane misses the origin).
    let mid = corners.iter().fold([0.0, 0.0], |s, c| [s[0] + c[0], s[1] + c[1]]);
    let mid_angle = mid[1].atan2(mid[0]);
    let angles: Vec<f64> = corners.iter().map(|c| mid_angle + (c[1].atan2(c[0]) - mid_angle + PI).rem_euclid(TAU) - PI).collect();
    let (low, high) = (angles.iter().copied().fold(f64::INFINITY, f64::min), angles.iter().copied().fold(f64::NEG_INFINITY, f64::max));
    if high - low >= PI {
        return Err(refuse("turn_shadow", "the tool reaches round the turn's axis".into()));
    }
    let (start, end) = (rate * screw.t0, rate * screw.t1);
    let (first, last) = if rate > 0.0 { (start, end) } else { (end, start) };
    let reference = 0.5 * (wedge[0] + wedge[1]);
    let near = |a: f64| reference + (a - reference + PI).rem_euclid(TAU) - PI;
    // Before the wedge at one end, after it at the other (angles taken near the wedge).
    let clear = near(high + first) < wedge[0] - 1e-9 && near(low + last) > wedge[1] + 1e-9 && (last - first) < TAU;
    if !clear {
        return Err(refuse("turn_shadow", "the turned tool's ends reach the clip's wedge; its sweep there is not its turned shadow".into()));
    }
    let profile = polyhedron_shadow(&corners, faces, false)?;
    // The profile in the half-plane at the wedge's start angle.
    let (c0, s0) = (wedge[0].cos(), wedge[0].sin());
    let world = |rho: f64, z: f64| clip.axis_origin.add(ez.scale(z)).add(ex.scale(rho * c0)).add(ey.scale(rho * s0));
    let mut curves = Vec::with_capacity(profile.len());
    for piece in &profile {
        let points: Vec<Vec4> = piece.iter().map(|&(rho, z, w)| Vec4::from_point(world(rho, z), w)).collect();
        let curve = if points.len() == 2 {
            frag(make_line(frag(points[0].point(), "point")?, frag(points[1].point(), "point")?), "line")?
        } else {
            frag(NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], points), "conic")?
        };
        curves.push(curve);
    }
    frag(crate::revolve_profile_brep(&curves, clip.axis_origin, ez, wedge[1] - wedge[0]), "turn_shadow")
}

/// A convex polyhedron turned a whole turn (no advance): the solid of
/// revolution of its shadow {(ρ, z)} about the turn's axis, which may pass
/// through it. Its corners' paths bound it only where an edge is farthest
/// from the axis; an edge skew to the axis turns a hyperboloid.
fn whole_turn_shadow(screw: &Screw, vertices: &[Vec3], faces: &[Vec<usize>]) -> Result<BrepSolid, KernelRefusal> {
    let ez = screw.e;
    let ex = frag(ez.perpendicular(), "turn_shadow")?;
    let ex = frag(ex.sub(ez.scale(ex.dot(ez))).normalized(), "turn_shadow")?;
    let ey = ez.cross(ex);
    let corners: Vec<[f64; 3]> = vertices
        .iter()
        .map(|&p| {
            let r = screw.apply(screw.t0, p).sub(screw.a);
            [r.dot(ex), r.dot(ey), r.dot(ez)]
        })
        .collect();
    let profile = polyhedron_shadow(&corners, faces, true)?;
    // A point of the outline on the axis to round-off is on it.
    let reach = corners.iter().map(|c| c[0].hypot(c[1]).max(c[2].abs())).fold(0.0f64, f64::max);
    let world = |rho: f64, z: f64| screw.a.add(ez.scale(z)).add(ex.scale(if rho.abs() <= 1e-10 * reach { 0.0 } else { rho }));
    let mut curves = Vec::with_capacity(profile.len());
    // The outline's stretch along the axis, as one edge (it comes a piece per
    // corner height).
    let on_axis = |piece: &ShadowPiece| piece.iter().all(|&(rho, _, _)| rho.abs() <= 1e-10 * reach);
    let mut along: Option<(f64, f64)> = None;
    for piece in &profile {
        if on_axis(piece) {
            let (za, zb) = (piece[0].1, piece[piece.len() - 1].1);
            along = Some(match along {
                Some((from, _)) => (from, zb),
                None => (za, zb),
            });
            continue;
        }
        if let Some((from, to)) = along.take() {
            curves.push(frag(make_line(world(0.0, from), world(0.0, to)), "line")?);
        }
        let points: Vec<Vec4> = piece.iter().map(|&(rho, z, w)| Vec4::from_point(world(rho, z), w)).collect();
        let (first, last) = (frag(points[0].point(), "point")?, frag(points[points.len() - 1].point(), "point")?);
        if first.sub(last).length() <= 1e-12 * (1.0 + first.length()) {
            continue;
        }
        let curve = if points.len() == 2 { frag(make_line(first, last), "line")? } else { frag(NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], points), "conic")? };
        curves.push(curve);
    }
    if let Some((from, to)) = along.take() {
        curves.push(frag(make_line(world(0.0, from), world(0.0, to)), "line")?);
    }
    frag(crate::revolve_profile_brep(&curves, screw.a, ez, TAU), "turn_shadow")
}

/// One piece of the shadow's outline: a segment (two points) or a rational
/// quadratic arc (three, the middle with its weight), as (ρ, z, weight).
type ShadowPiece = Vec<(f64, f64, f64)>;

/// The shadow {(ρ, z)} of a convex polyhedron (corners in the axis frame):
/// at each z its section is a convex polygon, the ρ it reaches an interval
/// from the section's nearest point to its farthest corner. The farthest is
/// always on an edge (ρ² = quadratic in z: a conic arc); the nearest is on a
/// face (ρ linear in z: a segment) or an edge.
///
/// `through`: the axis may pass through the tool (its nearest ρ is then 0 and
/// the outline runs along the axis there); refused otherwise.
fn polyhedron_shadow(corners: &[[f64; 3]], faces: &[Vec<usize>], through: bool) -> Result<Vec<ShadowPiece>, KernelRefusal> {
    let zs: Vec<f64> = corners.iter().map(|c| c[2]).collect();
    let (z_low, z_high) = (zs.iter().copied().fold(f64::INFINITY, f64::min), zs.iter().copied().fold(f64::NEG_INFINITY, f64::max));
    let scale = corners.iter().map(|c| c[0].hypot(c[1]).max(c[2].abs())).fold(1.0f64, f64::max);
    // The hull's edges (from the faces' loops) and the faces beside each.
    let mut edge_faces: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
    for (f, face) in faces.iter().enumerate() {
        for k in 0..face.len() {
            let (i, j) = (face[k], face[(k + 1) % face.len()]);
            edge_faces.entry((i.min(j), i.max(j))).or_default().push(f);
        }
    }
    let mut edges: Vec<(usize, usize)> = edge_faces.keys().copied().collect();
    edges.sort_unstable();
    // Each face's plane in the axis frame: n · p = d.
    let planes: Vec<([f64; 3], f64)> = faces
        .iter()
        .map(|face| {
            let (a, b, c) = (corners[face[0]], corners[face[1]], corners[face[2]]);
            let (u, v) = ([b[0] - a[0], b[1] - a[1], b[2] - a[2]], [c[0] - a[0], c[1] - a[1], c[2] - a[2]]);
            let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
            (n, n[0] * a[0] + n[1] * a[1] + n[2] * a[2])
        })
        .collect();
    #[derive(Clone, Copy, PartialEq, Debug)]
    enum Source {
        /// The crossing of edge k.
        Edge(usize),
        /// The nearest point of face f's section line.
        Face(usize),
        /// The axis itself, where it passes through the tool.
        Axis,
    }
    let edge_at = |k: usize, z: f64| -> [f64; 2] {
        let (a, b) = (corners[edges[k].0], corners[edges[k].1]);
        let t = (z - a[2]) / (b[2] - a[2]);
        [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
    };
    let crossing = |z: f64| -> Vec<usize> {
        (0..edges.len()).filter(|&k| (corners[edges[k].0][2] - z) * (corners[edges[k].1][2] - z) < 0.0).collect()
    };
    let farthest = |z: f64| -> (Source, f64) {
        let mut best = (Source::Edge(0), f64::NEG_INFINITY);
        for k in crossing(z) {
            let p = edge_at(k, z);
            let r = p[0].hypot(p[1]);
            if r > best.1 {
                best = (Source::Edge(k), r);
            }
        }
        best
    };
    // The section polygon: edge crossings ordered by angle about their mean.
    let nearest = |z: f64| -> Result<(Source, f64), KernelRefusal> {
        let list = crossing(z);
        let points: Vec<[f64; 2]> = list.iter().map(|&k| edge_at(k, z)).collect();
        let m = points.len();
        if m < 3 {
            return Err(refuse("turn_shadow", "a section of the tool is not a polygon".into()));
        }
        let centre = points.iter().fold([0.0, 0.0], |s, p| [s[0] + p[0] / m as f64, s[1] + p[1] / m as f64]);
        let mut order: Vec<usize> = (0..m).collect();
        order.sort_by(|&a, &b| {
            let (pa, pb) = (points[a], points[b]);
            (pa[1] - centre[1]).atan2(pa[0] - centre[0]).total_cmp(&(pb[1] - centre[1]).atan2(pb[0] - centre[0]))
        });
        let mut inside = true;
        let mut best = (Source::Edge(0), f64::INFINITY);
        for k in 0..m {
            let (ia, ib) = (order[k], order[(k + 1) % m]);
            let (a, b) = (points[ia], points[ib]);
            let d = [b[0] - a[0], b[1] - a[1]];
            if d[0] * (0.0 - a[1]) - d[1] * (0.0 - a[0]) < 0.0 {
                inside = false;
            }
            let length2 = d[0] * d[0] + d[1] * d[1];
            let t = -(a[0] * d[0] + a[1] * d[1]) / length2;
            let (source, r) = if t > 0.0 && t < 1.0 {
                let (ea, eb) = (edges[list[ia]], edges[list[ib]]);
                let shared = edge_faces[&ea].iter().find(|f| edge_faces[&eb].contains(f)).copied();
                let Some(face) = shared else {
                    return Err(refuse("turn_shadow", "a section side does not lie on one face".into()));
                };
                let q = [a[0] + d[0] * t, a[1] + d[1] * t];
                (Source::Face(face), q[0].hypot(q[1]))
            } else {
                let q = if t <= 0.0 { a } else { b };
                (Source::Edge(list[if t <= 0.0 { ia } else { ib }]), q[0].hypot(q[1]))
            };
            if r < best.1 {
                best = (source, r);
            }
        }
        if inside {
            if !through {
                return Err(refuse("turn_shadow", "the turn's axis passes through the tool".into()));
            }
            return Ok((Source::Axis, 0.0));
        }
        Ok(best)
    };
    // ρ of a source at z, from its own geometry (smooth across the piece).
    let value = |source: Source, z: f64| -> f64 {
        match source {
            Source::Edge(k) => {
                let p = edge_at(k, z);
                p[0].hypot(p[1])
            }
            Source::Face(f) => {
                let (n, d) = planes[f];
                (d - n[2] * z).abs() / n[0].hypot(n[1])
            }
            Source::Axis => 0.0,
        }
    };
    // Breaks: corner heights, and where the farthest or nearest source changes.
    let mut heights: Vec<f64> = zs.clone();
    heights.sort_by(|a, b| a.total_cmp(b));
    heights.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * scale);
    let mut pieces_far: Vec<(f64, f64, Source)> = Vec::new();
    let mut pieces_near: Vec<(f64, f64, Source)> = Vec::new();
    for window in heights.windows(2) {
        let (za, zb) = (window[0], window[1]);
        let samples = 400;
        let inner = |k: usize| za + (zb - za) * (k as f64 + 0.5) / samples as f64;
        for near in [false, true] {
            let source_at = |z: f64| -> Result<Source, KernelRefusal> { Ok(if near { nearest(z)?.0 } else { farthest(z).0 }) };
            let mut runs: Vec<(f64, Source)> = vec![(za, source_at(inner(0))?)];
            for k in 1..samples {
                let (z0, z1) = (inner(k - 1), inner(k));
                let (s0, s1) = (source_at(z0)?, source_at(z1)?);
                // Two sources tied over the stretch (edges equally far from
                // the axis) are one piece of the outline.
                let tied = [z0, 0.5 * (z0 + z1), z1].iter().all(|&z| (value(s0, z) - value(s1, z)).abs() <= 1e-12 * scale);
                if s0 == s1 || tied {
                    continue;
                }
                // Where the source changes, by bisection (a side handing over
                // to its corner touches it, so their values do not cross).
                let (mut lo, mut hi) = (z0, z1);
                for _ in 0..200 {
                    let midz = 0.5 * (lo + hi);
                    if source_at(midz)? == s0 { lo = midz } else { hi = midz }
                }
                runs.push((0.5 * (lo + hi), s1));
            }
            let list = if near { &mut pieces_near } else { &mut pieces_far };
            for k in 0..runs.len() {
                let z_end = if k + 1 < runs.len() { runs[k + 1].0 } else { zb };
                list.push((runs[k].0, z_end, runs[k].1));
            }
        }
    }
    // Exact pieces: a pair's ρ(z) is a conic arc, a side's a segment.
    let piece = |source: Source, za: f64, zb: f64| -> Vec<ShadowPiece> {
        let (ra, rb) = (value(source, za), value(source, zb));
        if matches!(source, Source::Face(..) | Source::Axis) {
            return vec![vec![(ra, za, 1.0), (rb, zb, 1.0)]];
        }
        // ρ² = α z² + β z + γ; split so each arc turns little.
        let rho = |z: f64| value(source, z);
        // Differences kept inside the piece (an edge through the axis has a
        // corner in ρ at its end there).
        let (z_min, z_max) = (za.min(zb), za.max(zb));
        let slope = |z: f64| {
            let h = 1e-6 * (zb - za).abs().max(1e-300);
            let (zp, zm) = ((z + h).min(z_max), (z - h).max(z_min));
            (rho(zp) - rho(zm)) / (zp - zm)
        };
        let parts = 4;
        let mut out = Vec::new();
        for k in 0..parts {
            let (z0, z1) = (za + (zb - za) * k as f64 / parts as f64, za + (zb - za) * (k + 1) as f64 / parts as f64);
            let (a, b) = ((rho(z0), z0), (rho(z1), z1));
            // Straight (an edge meeting the axis): a segment.
            let bend = [0.25, 0.5, 0.75].iter().map(|&t| (rho(z0 + (z1 - z0) * t) - (a.0 + (b.0 - a.0) * t)).abs()).fold(0.0f64, f64::max);
            if bend <= 1e-12 * scale {
                out.push(vec![(a.0, a.1, 1.0), (b.0, b.1, 1.0)]);
                continue;
            }
            let (ta, tb) = ((slope(z0), 1.0), (slope(z1), 1.0));
            // Tangent lines' intersection.
            let det = ta.0 * (-tb.1) - ta.1 * (-tb.0);
            if det.abs() < 1e-12 {
                out.push(vec![(a.0, a.1, 1.0), (b.0, b.1, 1.0)]);
                continue;
            }
            let rhs = (b.0 - a.0, b.1 - a.1);
            let s = (rhs.0 * (-tb.1) - rhs.1 * (-tb.0)) / det;
            let c = (a.0 + ta.0 * s, a.1 + ta.1 * s);
            // Shoulder: the conic's point on the line from the chord's middle to c.
            let m = (0.5 * (a.0 + b.0), 0.5 * (a.1 + b.1));
            let on = |u: f64| {
                let p = (m.0 + (c.0 - m.0) * u, m.1 + (c.1 - m.1) * u);
                p.0 - rho(p.1)
            };
            let (mut lo, mut hi) = (0.0, 1.0);
            let sign_lo = on(lo);
            for _ in 0..200 {
                let mid = 0.5 * (lo + hi);
                if (on(mid) > 0.0) == (sign_lo > 0.0) { lo = mid } else { hi = mid }
            }
            let u = 0.5 * (lo + hi);
            let weight = u / (1.0 - u);
            out.push(vec![(a.0, a.1, 1.0), (c.0, c.1, weight), (b.0, b.1, 1.0)]);
        }
        out
    };
    // Outline: up the near side, across the top, down the far side, across the bottom.
    let mut outline: Vec<ShadowPiece> = Vec::new();
    for &(za, zb, source) in &pieces_near {
        outline.extend(piece(source, za, zb));
    }
    let (near_top, far_top) = (value(pieces_near.last().expect("near").2, z_high), value(pieces_far.last().expect("far").2, z_high));
    if far_top - near_top > 1e-12 * scale {
        outline.push(vec![(near_top, z_high, 1.0), (far_top, z_high, 1.0)]);
    }
    for &(za, zb, source) in pieces_far.iter().rev() {
        for mut arc in piece(source, za, zb).into_iter().rev() {
            arc.reverse();
            outline.push(arc);
        }
    }
    let (near_bottom, far_bottom) = (value(pieces_near[0].2, z_low), value(pieces_far[0].2, z_low));
    if far_bottom - near_bottom > 1e-12 * scale {
        outline.push(vec![(far_bottom, z_low, 1.0), (near_bottom, z_low, 1.0)]);
    }
    // Snap consecutive ends together.
    for k in 0..outline.len() {
        let next = (k + 1) % outline.len();
        let end = *outline[k].last().expect("piece");
        outline[next][0] = (end.0, end.1, 1.0);
    }
    Ok(outline)
}

/// A ball TURNED about an axis that passes through it. Its grazing circle (in
/// the plane of the axis and the centre) crosses the axis at two poles N and S;
/// the part beyond the axis turns inside the sweep, and near the axis the ends
/// overlap. So the sweep has three faces: the wall, the outer arc of that
/// circle turned (a spindle torus's outer part, pinched at N and S); the
/// ball's back at the start less the ball at the end; and its front at the end
/// less the ball at the start. The two caps meet along the far arc, from S to
/// N, of the circle where the two spheres meet. None when the motion or tool
/// is not of that kind.
fn ball_across_axis(screw: &Screw, origin: Vec3, axis: Vec3, meridian: &[OutlinePiece]) -> Result<Option<BrepSolid>, KernelRefusal> {
    let [OutlinePiece::Arc { centre, radius, start, end }, OutlinePiece::Line { .. }] = meridian else { return Ok(None) };
    let (centre, r) = (*centre, *radius);
    if centre[0].abs() > 1e-12 * r || ((end - start).abs() - PI).abs() > 1e-12 || screw.c != 0.0 || screw.c2 != 0.0 || screw.omega == 0.0 {
        return Ok(None);
    }
    let w = axis.normalized().map_err(|_| input("axis", "the tool's axis has no direction".into()))?;
    let centre_tool = origin.add(w.scale(centre[1]));
    let turned = screw.omega * (screw.t1 - screw.t0);
    if turned.abs() >= TAU - 1e-9 {
        return Ok(None);
    }
    let e = screw.e.scale(screw.omega.signum()); // the sense the ball turns about
    let c0 = screw.apply(screw.t0, centre_tool);
    let c1 = screw.apply(screw.t1, centre_tool);
    let foot = |c: Vec3| screw.a.add(e.scale(c.sub(screw.a).dot(e)));
    let rho = c0.sub(foot(c0)).length();
    if rho >= r * (1.0 - 1e-9) || rho <= 1e-9 * r {
        return Ok(None);
    }
    let h0 = frag(c0.sub(foot(c0)).normalized(), "radial")?;
    let height = (r * r - rho * rho).sqrt();
    let (north, south) = (foot(c0).add(e.scale(height)), foot(c0).sub(e.scale(height)));
    // An arc from `from` turned about `axis` through `centre` by `angle`.
    let arc = |about: Vec3, k: Vec3, from: Vec3, angle: f64| -> Result<NurbsCurve, KernelRefusal> {
        let surface = frag(make_revolution(about, k, &frag(make_line(from, from.add(k)), "line")?, angle), "arc")?;
        frag(surface.iso_curve_v(0.0), "arc")
    };
    // The outer arc of the grazing circle, S -> N through c0 + r h0: in the
    // (h0, e) plane about c0, from angle -α to α through 0, α = atan2(height, -ρ).
    let alpha = height.atan2(-rho);
    let outer = arc(c0, h0.cross(e), south, 2.0 * alpha)?;
    let wall = frag(make_revolution(screw.a, e, &outer, turned.abs()), "wall")?;
    let outer_end = frag(wall.iso_curve_u(1.0), "wall_end")?;
    // The far arc where the spheres meet, S -> N beyond the axis.
    let mid = c0.add(c1).scale(0.5);
    let towards = frag(mid.sub(foot(mid)).normalized(), "bisector")?;
    let rho_mid = mid.sub(foot(mid)).length();
    let beta = height.atan2(rho_mid); // S and N seen from mid, off the far direction
    // From S at angle β − π (in the (towards, e) plane about mid) down through
    // π to N at π − β: the decreasing sense, about e × towards.
    let meet = arc(mid, e.cross(towards), south, 2.0 * beta)?;
    {
        // The arc must pass beyond the axis (its middle on the far side).
        let [d0, d1] = frag(meet.domain(), "domain")?;
        let middle = frag(meet.evaluate(0.5 * (d0 + d1)), "middle")?;
        if middle.sub(foot(middle)).dot(towards) > 0.0 {
            return Err(refuse("ball_across_axis", "the spheres' common circle runs the wrong way round".into()));
        }
    }
    // A trimmed sphere cap: a chart whose poles and seam lie where the cap is not.
    let cap = |c: Vec3, inward: Vec3, side: Vec3, other: Vec3| -> Result<(NurbsSurface, Box<dyn Fn(Vec3) -> Vec3>), KernelRefusal> {
        // South pole just off the far end of the grazing circle, on the removed
        // side (inside the other end's ball); the seam crosses the grazing
        // circle beyond the axis only.
        let toward_south = frag(inward.add(side.scale(-0.3)).normalized(), "pole")?;
        let polar = toward_south.scale(-1.0);
        let seam = frag(side.sub(polar.scale(side.dot(polar))).normalized(), "seam")?;
        for k in 0..=64 {
            // The seam's part on the cap's side of the grazing circle must lie
            // inside the other ball (where the cap is not).
            let a = PI * k as f64 / 64.0;
            let q = c.add(polar.scale(-r * a.cos())).add(seam.scale(r * a.sin()));
            if q.sub(c).dot(side) < 0.0 && q.sub(other).length() >= r * (1.0 - 1e-9) {
                return Err(refuse("ball_across_axis", "no seam place for a cap".into()));
            }
        }
        let south_pole = c.sub(polar.scale(r));
        if south_pole.sub(other).length() >= r * (1.0 - 1e-6) {
            return Err(refuse("ball_across_axis", "no pole place for a cap".into()));
        }
        let generatrix = arc(c, polar.scale(-1.0).cross(seam), south_pole, PI)?;
        let surface = frag(make_revolution(c, polar, &generatrix, TAU), "cap")?;
        let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
        let spans_v = (generatrix.knots.len() - 6) / 2 + 1;
        let around = polar.cross(seam);
        let inverse = move |x: Vec3| -> Vec3 {
            let d = x.sub(c).scale(1.0 / r);
            let latitude = d.dot(polar).clamp(-1.0, 1.0).asin();
            let longitude = d.dot(around).atan2(d.dot(seam)).rem_euclid(TAU);
            Vec3::new(
                crate::circle_angle_to_parameter(spans_u, TAU, longitude),
                crate::circle_angle_to_parameter(spans_v, PI, latitude + 0.5 * PI),
                0.0,
            )
        };
        Ok((surface, Box::new(inverse)))
    };
    let moving0 = e.cross(h0);
    let h1 = frag(c1.sub(foot(c1)).normalized(), "radial")?;
    let moving1 = e.cross(h1);
    // The back cap at the start (its removed part near c0 − r h0 on the back);
    // the front cap at the end.
    let (back, back_inverse) = cap(c0, h0.scale(-1.0), moving0.scale(1.0), c1)?;
    let (front, front_inverse) = cap(c1, h1.scale(-1.0), moving1.scale(-1.0), c0)?;
    let (back_joints, front_joints) = (surface_joints(&back), surface_joints(&front));
    let pcurve = |inverse: &dyn Fn(Vec3) -> Vec3, curve: &NurbsCurve, forward: bool| -> Result<NurbsCurve, KernelRefusal> {
        let [d0, d1] = frag(curve.domain(), "domain")?;
        let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
        let uv: Vec<Vec3> = ts.iter().map(|&t| curve.evaluate(t).map(|x| inverse(x))).collect::<Result<_, _>>().map_err(|e| refuse("pcurve", e))?;
        if pcurve_joints_on() {
            // The two caps share their structure (one make_revolution each).
            let (ju, jv) = (&back_joints.0, &back_joints.1);
            debug_assert_eq!(back_joints, front_joints);
            let at = |t: f64| -> Result<[f64; 2], KernelRefusal> {
                let p = inverse(frag(curve.evaluate(t), "point")?);
                Ok([p.x, p.y])
            };
            if let Some(fitted) = jointed_pcurve(&ts, &uv, at, ju, jv, curve)? {
                return if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") };
            }
        }
        let fitted = frag(crate::interpolate_curve(&uv, 5, &ts), "cap_pcurve")?;
        if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") }
    };
    let mut shell = Shell::new();
    let (vn, vs) = (shell.vertex(north), shell.vertex(south));
    let k0 = shell.edge(outer.clone(), vs, vn, false)?;
    let k1 = shell.edge(outer_end.clone(), vs, vn, false)?;
    let c = shell.edge(meet.clone(), vs, vn, false)?;
    let ds = shell.edge(frag(make_line(south, south), "point")?, vs, vs, true)?;
    let dn = shell.edge(frag(make_line(north, north), "point")?, vn, vn, true)?;
    // Wall: u turns, v runs S -> N; outward needs v the other way round the
    // circle's normal, so test and pick the loop's sense.
    let wall_outward = {
        let p = frag(wall.evaluate(0.5, 0.5), "probe")?;
        let su = frag(wall.evaluate(0.5 + 1e-6, 0.5), "probe")?.sub(frag(wall.evaluate(0.5 - 1e-6, 0.5), "probe")?);
        let sv = frag(wall.evaluate(0.5, 0.5 + 1e-6), "probe")?.sub(frag(wall.evaluate(0.5, 0.5 - 1e-6), "probe")?);
        su.cross(sv).dot(p.sub(foot(p))) > 0.0
    };
    let [g0, g1] = frag(outer.domain(), "domain")?;
    let wall_loop = vec![
        (ds, true, uv_line([0.0, g0], [1.0, g0])?),
        (k1, true, uv_line([1.0, g0], [1.0, g1])?),
        (dn, true, uv_line([1.0, g1], [0.0, g1])?),
        (k0, false, uv_line([0.0, g1], [0.0, g0])?),
    ];
    let reversed = |list: Vec<(u64, bool, NurbsCurve)>| -> Result<Vec<(u64, bool, NurbsCurve)>, KernelRefusal> {
        list.into_iter().rev().map(|(e, f, p)| Ok((e, !f, frag(p.reversed(), "reversed")?))).collect()
    };
    shell.face(wall.clone(), if wall_outward { wall_loop } else { reversed(wall_loop)? });
    // Caps: loop (grazing arc S -> N, then the meeting arc N -> S), sense by
    // the signed area of its parameters (the charts' u × v points outward).
    for (surface, inverse, k_edge, k_curve) in [(back, &back_inverse, k0, &outer), (front, &front_inverse, k1, &outer_end)] {
        let mut lp = vec![(k_edge, true, pcurve(inverse.as_ref(), k_curve, true)?), (c, false, pcurve(inverse.as_ref(), &meet, false)?)];
        let mut area = 0.0;
        for (_, _, pc) in &lp {
            let [d0, d1] = frag(pc.domain(), "domain")?;
            for j in 0..64 {
                let (a, b) = (frag(pc.evaluate(d0 + (d1 - d0) * j as f64 / 64.0), "uv")?, frag(pc.evaluate(d0 + (d1 - d0) * (j + 1) as f64 / 64.0), "uv")?);
                area += a.x * b.y - b.x * a.y;
            }
        }
        if area < 0.0 {
            lp = reversed(lp)?;
        }
        shell.face(surface, lp);
    }
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse("assembly", format!("the turned ball's sweep is not valid: {:?}", issues.first())));
    }
    let probe = Split { points: Vec::new(), curves: Vec::new(), caps: Vec::new(), slides: Vec::new(), slide_sides: Vec::new(), scale: r + rho, shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() } };
    check_single_pass(screw, &probe, &body, &std::collections::HashSet::new())?;
    Ok(Some(body))
}

/// A frame riding a cubic Bézier curve (`control`, in a plane square to
/// `up`): its origin on the curve, its first axis along the curve's tangent,
/// `up` fixed. The tool is given in the frame at the curve's start and carried
/// by it over the whole curve.
#[derive(Clone, Copy, Debug)]
pub struct FrameCurve {
    pub control: [Vec3; 4],
    pub up: Vec3,
}

/// The whole sweep of a solid of revolution carried by a frame along a
/// curve. In the riding frame the velocity field is the speed along the
/// tangent plus the turn about `up`, so the grazing loop moves on the tool with
/// the curve's curvature. It is split at sample times (its structure must not
/// change), each piece sampled exactly, and the walls lofted through those
/// samples (quintic both ways; the samples' count doubles until the wall is
/// within `LOFT_TARGET` of the exact loop at the times between). Caps are the tool's
/// back at the start and front at the end.
pub fn sweep_solid_along(tool: &SolidTool, path: &FrameCurve) -> Result<BrepSolid, KernelRefusal> {
    // An upright solid of revolution on a planar curve is translated by it:
    // its sweep is the planar distance to the curve within the tool's radius,
    // folds and crossings included (`BREP_SWEEP_UPRIGHT_SLOT`). With the
    // direct build on (`BREP_SWEEP_UPRIGHT_DIRECT`) the route answers only
    // where the general routes refuse and the curve may turn tighter than the
    // tool, built face by face; every sweep the general routes build is theirs
    // as before, and where the direct build declines their refusal stands.
    // With the direct build off the route takes every upright sweep.
    if crate::solid_sweep_upright::upright_slot_on() {
        if let Some(slot) = crate::solid_sweep_upright::upright_slot(tool, path) {
            if !crate::solid_sweep_upright::upright_direct_on() {
                return slot.solid();
            }
            let general = sweep_solid_along_general(tool, path);
            if general.is_err() && slot.turns_tighter() {
                if let Ok(solid) = slot.direct_solid() {
                    return Ok(solid);
                }
            }
            return general;
        }
    }
    sweep_solid_along_general(tool, path)
}

/// [`sweep_solid_along`] by its general routes (the planar ring, the lofted
/// grazing loop), without the upright route.
fn sweep_solid_along_general(tool: &SolidTool, path: &FrameCurve) -> Result<BrepSolid, KernelRefusal> {
    // A ring on a planar curve, from its slices (`BREP_SWEEP_PLANAR_RING`).
    if let Some(ring) = crate::solid_sweep_planar_ring::planar_ring_on().then(|| crate::solid_sweep_planar_ring::planar_ring_sweep(tool, path)).flatten() {
        return ring;
    }
    let up = path.up.normalized().map_err(|_| input("up", "the frame's up direction is zero".into()))?;
    let base = path.control[0];
    if path.control.iter().any(|p| p.sub(base).dot(up).abs() > 1e-9 * (1.0 + p.sub(base).length())) {
        return Err(input("plane", "the curve is not in a plane square to up".into()));
    }
    let tangent = path.control[1].sub(base);
    let tangent = if tangent.length() > 0.0 { tangent } else { path.control[2].sub(base) };
    let start_tangent = tangent.normalized().map_err(|_| input("tangent", "the curve has no start direction".into()))?;
    let frame = Frame { control: path.control, up, start_tangent };
    let (mut speed, mut rate, mut extent) = (0.0f64, 0.0f64, 0.0f64);
    let mut previous = 0.0;
    for k in 0..=512 {
        let t = k as f64 / 512.0;
        let first = frame.first(t);
        if first.length() <= 1e-12 {
            return Err(input("tangent", "the curve stops (zero tangent)".into()));
        }
        speed = speed.max(first.length());
        rate = rate.max(frame.angle_rate(t).abs());
        extent = extent.max(frame.point(t).sub(base).length());
        let angle = frame.angle(t);
        if (angle - previous).abs() > 0.5 {
            return Err(refuse("frame_turns", "the frame turns half way round".into()));
        }
        previous = angle;
    }
    let screw = Screw { a: base, e: up, omega: rate, c: speed, c2: 0.0, t0: 0.0, t1: 1.0, frame: Some(frame), turns: None, frozen: None,
        loop_samples: LOFT_SAMPLES, extent };
    let SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } = tool else {
        return Err(refuse("along_tool", "a frame along a curve carries a solid of revolution only".into()));
    };
    let axis = axis_direction.normalized().map_err(|_| input("axis", "the tool's axis has no direction".into()))?;
    // The splits name their pieces by angle about the tool's axis from its
    // radial direction: a grazing angle passing that reference renames them,
    // so try a few references.
    let mut last = None;
    for k in 0..8 {
        let reference = radial.rotate_about(axis, PI * k as f64 / 4.0);
        match lofted(&screw, *axis_origin, axis, reference, meridian) {
            Err(error) if refused_as(&error) == Some("loft_structure") => last = Some(error),
            other => return other,
        }
    }
    Err(last.expect("tried"))
}

/// The whole sweep of a prism carried by a frame along a planar curve (its
/// axis along the frame's up): the prism `outline` (a convex polygon in the
/// start frame: first coordinate along the start tangent, second along
/// up × tangent) between `from` and `to` along up. The motion is planar, so the
/// sweep is the plane region the moving outline covers, extruded. That region
/// is found exactly: its boundary lies on the outline's corner paths, the paths
/// of each edge's grazing point (where the edge's normal is square to the
/// velocity), and the outline's edges at the two ends; these are split where
/// they meet, and a piece is kept where one side of it is covered and the other
/// not (folds, where a corner digs past a side, come out right).
pub fn sweep_prism_along(outline: &[[f64; 2]], from: f64, to: f64, path: &FrameCurve) -> Result<BrepSolid, KernelRefusal> {
    let up = path.up.normalized().map_err(|_| input("up", "the frame's up direction is zero".into()))?;
    let base = path.control[0];
    if path.control.iter().any(|p| p.sub(base).dot(up).abs() > 1e-9 * (1.0 + p.sub(base).length())) {
        return Err(input("plane", "the curve is not in a plane square to up".into()));
    }
    let tangent = path.control[1].sub(base);
    let tangent = if tangent.length() > 0.0 { tangent } else { path.control[2].sub(base) };
    let e1 = tangent.normalized().map_err(|_| input("tangent", "the curve has no start direction".into()))?;
    let e2 = up.cross(e1);
    let frame = Frame { control: path.control, up, start_tangent: e1 };
    // The frame in plane coordinates: origin c(s), turned by φ(s).
    let flat = |p: Vec3| [p.sub(base).dot(e1), p.sub(base).dot(e2)];
    let pose = |s: f64| (flat(frame.point(s)), frame.angle(s));
    let rates = |s: f64| {
        let first = frame.first(s);
        ([first.dot(e1), first.dot(e2)], frame.angle_rate(s))
    };
    prism_region_sweep(outline, from, to, base, e1, e2, up, &pose, &rates, false)
}

/// The whole sweep of a prism (the convex polygon `outline` in the plane
/// through `base` spanned by `e1`, `e2`, between `from` and `to` along `up`)
/// under a planar rigid motion: at s ∈ [0, 1] a point q of the plane (in the
/// tool's own coordinates) is at origin(s) + R(angle(s)) q, `pose` giving
/// (origin, angle) and `rates` their derivatives in s. The sweep is the plane
/// region the moving outline covers, extruded. That region is found exactly:
/// its boundary lies on the outline's corner paths, the paths of each edge's
/// grazing point (where the edge's normal is square to the velocity), and,
/// unless the motion is `closed` (it ends where it starts), the outline's
/// edges at the two ends; these are split where they meet or cross
/// themselves, and a piece is kept where one side of it is covered and the
/// other not. The region may have holes.
#[allow(clippy::too_many_arguments)]
pub fn prism_region_sweep(
    outline: &[[f64; 2]],
    from: f64,
    to: f64,
    base: Vec3,
    e1: Vec3,
    e2: Vec3,
    up: Vec3,
    pose: &dyn Fn(f64) -> ([f64; 2], f64),
    rates: &dyn Fn(f64) -> ([f64; 2], f64),
    closed: bool,
) -> Result<BrepSolid, KernelRefusal> {
    if outline.len() < 3 || !(to > from) {
        return Err(input("prism", "a prism needs three corners and a positive height".into()));
    }
    // Counter-clockwise, convex.
    let n = outline.len();
    let mut corners = outline.to_vec();
    let area: f64 = (0..n).map(|k| corners[k][0] * corners[(k + 1) % n][1] - corners[(k + 1) % n][0] * corners[k][1]).sum();
    if area < 0.0 {
        corners.reverse();
    }
    for k in 0..n {
        let (a, b, c) = (corners[k], corners[(k + 1) % n], corners[(k + 2) % n]);
        if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) <= 0.0 {
            return Err(input("prism", "the outline is not convex".into()));
        }
    }
    let scale = corners.iter().map(|c| c[0].hypot(c[1])).fold(1.0f64, f64::max);
    let place = |s: f64, q: [f64; 2]| -> [f64; 2] {
        let (o, angle) = pose(s);
        let (sn, cs) = angle.sin_cos();
        [o[0] + cs * q[0] - sn * q[1], o[1] + sn * q[0] + cs * q[1]]
    };
    let local = |s: f64, p: [f64; 2]| -> [f64; 2] {
        let (o, angle) = pose(s);
        let (sn, cs) = angle.sin_cos();
        let d = [p[0] - o[0], p[1] - o[1]];
        [cs * d[0] + sn * d[1], -sn * d[0] + cs * d[1]]
    };
    // Edges: outward normals and offsets in the start frame.
    let edges: Vec<([f64; 2], f64)> = (0..n)
        .map(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % n]);
            let d = [b[0] - a[0], b[1] - a[1]];
            let length = d[0].hypot(d[1]);
            let normal = [d[1] / length, -d[0] / length];
            (normal, normal[0] * a[0] + normal[1] * a[1])
        })
        .collect();
    let inside = |q: [f64; 2]| edges.iter().map(|(nn, d)| nn[0] * q[0] + nn[1] * q[1] - d).fold(f64::NEG_INFINITY, f64::max);
    // Is p covered at some time? The least signed distance over the motion.
    let depth = |p: [f64; 2]| -> f64 {
        let samples = 2000;
        let f = |s: f64| inside(local(s, p));
        let mut best = (f64::INFINITY, 0usize);
        let values: Vec<f64> = (0..=samples).map(|k| f(k as f64 / samples as f64)).collect();
        for (k, &v) in values.iter().enumerate() {
            if v < best.0 {
                best = (v, k);
            }
        }
        // Refine every local minimum near the least.
        let mut least = best.0;
        for k in 0..=samples {
            let left = if k == 0 { f64::INFINITY } else { values[k - 1] };
            let right = if k == samples { f64::INFINITY } else { values[k + 1] };
            if values[k] <= left && values[k] <= right && values[k] < best.0 + 1e-3 * scale {
                let (mut lo, mut hi) = (((k as f64) - 1.0).max(0.0) / samples as f64, ((k as f64) + 1.0).min(samples as f64) / samples as f64);
                for _ in 0..80 {
                    let (m1, m2) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                    if f(m1) < f(m2) { hi = m2 } else { lo = m1 }
                }
                least = least.min(f(0.5 * (lo + hi)));
            }
        }
        least
    };
    // Candidate boundary paths, each a map [0, 1] -> plane.
    #[derive(Clone, Copy)]
    enum Path {
        Corner(usize),
        /// Edge k's grazing point over the times [s0, s1].
        Graze(usize, f64, f64),
        /// Edge k at the start (false) or end (true).
        End(usize, bool),
    }
    // Where along edge k's line (0 at its start corner, 1 at its end) the
    // grazing point is at s, on the edge or beyond it.
    let graze_at = |k: usize, s: f64| -> Option<f64> {
        // Along edge k at x (in the tool's coordinates) the velocity's part
        // along the edge's normal n is w·n + φ' (ẑ × x)·n, w the origin's
        // velocity read in the tool's frame.
        let (a, b) = (corners[k], corners[(k + 1) % n]);
        let (nn, _) = edges[k];
        let (velocity, rate) = rates(s);
        let (sn, cs) = pose(s).1.sin_cos();
        let w = [cs * velocity[0] + sn * velocity[1], -sn * velocity[0] + cs * velocity[1]];
        let along = w[0] * nn[0] + w[1] * nn[1];
        let rot = |x: [f64; 2]| -x[1] * nn[0] + x[0] * nn[1]; // (ẑ × x)·n
        // An edge the origin moves along grazes where (ẑ × x)·n = 0 whatever
        // the turning rate (also where it is zero, at an inflection).
        let (ga, gb) = if along.abs() < 1e-15 * w[0].hypot(w[1]) { (rot(a), rot(b)) } else { (along + rate * rot(a), along + rate * rot(b)) };
        if ga == gb {
            return None;
        }
        Some(ga / (ga - gb))
    };
    let graze_point = |k: usize, s: f64| -> Option<f64> { graze_at(k, s).filter(|&u| u > 0.0 && u < 1.0) };
    let mut paths: Vec<Path> = (0..n).map(Path::Corner).collect();
    // Where a grazing point enters or leaves its edge it is at a corner:
    // (that corner, the time), a place its path is split at.
    let mut junctions: Vec<(usize, f64)> = Vec::new();
    for k in 0..n {
        // Where along the motion edge k has a grazing point inside it, each
        // run's ends found where the point passes a corner.
        let steps = 2000;
        let mut run: Option<f64> = None;
        let mut passes = |inside: f64, outside: f64| -> f64 {
            let (mut a, mut b) = (inside, outside);
            for _ in 0..80 {
                let mid = 0.5 * (a + b);
                if graze_point(k, mid).is_some() { a = mid } else { b = mid }
            }
            let corner = if graze_at(k, a).is_some_and(|u| u < 0.5) { k } else { (k + 1) % n };
            junctions.push((corner, a));
            a
        };
        for j in 0..=steps {
            let s = j as f64 / steps as f64;
            let on = graze_point(k, s).is_some();
            match (on, run) {
                (true, None) => run = Some(if j == 0 { s } else { passes(s, s - 1.0 / steps as f64) }),
                (false, Some(s0)) => {
                    paths.push(Path::Graze(k, s0, passes(s - 1.0 / steps as f64, s)));
                    run = None;
                }
                _ => {}
            }
        }
        if let Some(s0) = run {
            paths.push(Path::Graze(k, s0, 1.0));
        }
        if !closed {
            paths.push(Path::End(k, false));
            paths.push(Path::End(k, true));
        }
    }
    let eval = |path: Path, t: f64| -> [f64; 2] {
        match path {
            Path::Corner(k) => place(t, corners[k]),
            Path::Graze(k, s0, s1) => {
                let s = s0 + (s1 - s0) * t;
                let u = graze_at(k, s).map_or(0.5, |u| u.clamp(0.0, 1.0));
                let (a, b) = (corners[k], corners[(k + 1) % n]);
                place(s, [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u])
            }
            Path::End(k, at_end) => {
                let (a, b) = (corners[k], corners[(k + 1) % n]);
                place(if at_end { 1.0 } else { 0.0 }, [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
            }
        }
    };
    let count = paths.len();
    region_from_paths(
        count,
        &|index: usize, t: f64| eval(paths[index], t),
        &|index: usize| matches!(paths[index], Path::End(..)),
        &|_: usize| closed,
        &junctions,
        &depth,
        scale,
        (base, e1, e2, up),
        (from, to),
    )
}

/// Everything a tube (the ball of `radius` swept along the curve
/// `centreline(s)`, s ∈ [0, 1], its ends included) passes through as it slides
/// by `slide`, inside the slab `slab[0] ≤ (x − slab_origin)·n ≤ slab[1]` (`n` =
/// `slab_normal` normalised), which the tube must cross wholly: before the
/// slab at the start, past it at the end (checked at 8001 points of the
/// centreline, with a margin of the longest step between them). The
/// returned prism along the slide agrees with the sweep inside the slab; the
/// caller cuts a workpiece that lies within it.
///
/// The centreline must be resolved by its samples (as every path of the plane
/// region arrangement is): smooth, its height changing between two of the
/// 8001 checking samples by no more than the step between them, so that the
/// samples bound it. A curve that wanders between samples is outside this
/// contract.
///
/// Why a prism: a point x of the slab is in the sweep iff the line through x
/// along the slide meets the tube. If it does, `n·` grows along the line, so
/// its topmost point y of the starting tube lies before x, and y + slide (a
/// point of the final tube) past it: x − y is a multiple of the slide in
/// [0, 1). So the sweep in the slab is the slab's part of the prism over the
/// tube's shadow along the slide: in the plane square to it the union of the
/// discs of `radius` about the shadow γ of the centreline. That region is
/// bounded by γ's two offset curves (each disc's grazing points, γ ± radius
/// along its normal) and the end discs' circles, built exactly by the plane
/// region arrangement of [`sweep_prism_along`].
pub fn sweep_tube_slid_across(
    centreline: &dyn Fn(f64) -> Vec3,
    radius: f64,
    slide: Vec3,
    slab_origin: Vec3,
    slab_normal: Vec3,
    slab: [f64; 2],
) -> Result<BrepSolid, KernelRefusal> {
    if !(radius.is_finite() && radius > 0.0) {
        return Err(input("tube_radius", "the tube's radius must be positive".into()));
    }
    let up = slide.normalized().map_err(|_| input("slide", "the slide has no length".into()))?;
    let normal = slab_normal.normalized().map_err(|_| input("slab", "the slab's normal has no direction".into()))?;
    if !(slab[1] > slab[0]) || up.dot(normal).abs() < 1e-9 {
        return Err(input("slab", "the slab must have thickness and the slide must cross it".into()));
    }
    // Orient so the slide runs up the slab's normal.
    let (normal, slab) = if up.dot(normal) > 0.0 { (normal, slab) } else { (normal.scale(-1.0), [-slab[1], -slab[0]]) };
    let points: Vec<Vec3> = (0..=8000).map(|k| centreline(k as f64 / 8000.0)).collect();
    if points.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return Err(input("centreline", "the centreline has a point that is not finite".into()));
    }
    let step = points.windows(2).map(|w| w[1].sub(w[0]).length()).fold(0.0f64, f64::max);
    let heights: Vec<f64> = points.iter().map(|q| q.sub(slab_origin).dot(normal)).collect();
    let (low, high) = heights.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), &h| (a.min(h), b.max(h)));
    let rise = slide.dot(normal);
    if !(high + radius + step < slab[0] && low - radius - step + rise > slab[1]) {
        return Err(refuse("tube_slab", "the tube does not pass wholly through the slab (it starts or ends within reach of it)".into()));
    }
    let e1 = {
        let seed = if up.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        seed.sub(up.scale(seed.dot(up))).normalized().map_err(|_| input("slide", "no frame".into()))?
    };
    let e2 = up.cross(e1);
    // The plane's origin on the centreline itself, so that every tolerance
    // reads the tube's own size, wherever it lies.
    let base = points[0];
    let flat = |p: Vec3| [p.sub(base).dot(e1), p.sub(base).dot(e2)];
    let centre = |s: f64| -> [f64; 2] { flat(centreline(s)) };
    let velocity = |s: f64| -> [f64; 2] {
        let h = 1e-7;
        let (a, b) = ((s - h).max(0.0), (s + h).min(1.0));
        let (p, q) = (centre(a), centre(b));
        [(q[0] - p[0]) / (b - a), (q[1] - p[1]) / (b - a)]
    };
    // The shadow must keep moving (a disc standing still has no grazing points).
    for k in 0..=2000 {
        let w = velocity(k as f64 / 2000.0);
        if w[0].hypot(w[1]) < 1e-9 * (1.0 + radius) {
            return Err(refuse("tube_shadow", "the tube's shadow along the slide stops (its centreline runs along the slide)".into()));
        }
    }
    // Tolerances read the tube's radius (a probe beside a boundary must stay
    // within the tube), never more than its extent; a tube longer than 1e6
    // radii is beyond what the coordinates resolve at that size.
    let extent = (0..=64).map(|k| { let c = centre(k as f64 / 64.0); c[0].hypot(c[1]) }).fold(radius, f64::max);
    if extent > 1e6 * radius {
        return Err(refuse("tube_shadow", "the tube is too long for its radius (over a million radii)".into()));
    }
    let scale = extent.min(1e3 * radius);
    // Paths: the two offset curves, then the two end circles.
    let eval = |index: usize, t: f64| -> [f64; 2] {
        match index {
            0 | 1 => {
                let c = centre(t);
                let w = velocity(t);
                let length = w[0].hypot(w[1]);
                let side = if index == 0 { 1.0 } else { -1.0 };
                [c[0] - side * radius * w[1] / length, c[1] + side * radius * w[0] / length]
            }
            _ => {
                let c = centre(if index == 2 { 0.0 } else { 1.0 });
                let angle = TAU * t;
                [c[0] + radius * angle.cos(), c[1] + radius * angle.sin()]
            }
        }
    };
    // Inside: nearer than `radius` to the shadow at some time.
    let depth = |p: [f64; 2]| -> f64 {
        let samples = 4000;
        let f = |s: f64| { let c = centre(s); (p[0] - c[0]).hypot(p[1] - c[1]) - radius };
        let values: Vec<f64> = (0..=samples).map(|k| f(k as f64 / samples as f64)).collect();
        let mut least = values.iter().cloned().fold(f64::INFINITY, f64::min);
        for k in 0..=samples {
            let left = if k == 0 { f64::INFINITY } else { values[k - 1] };
            let right = if k == samples { f64::INFINITY } else { values[k + 1] };
            if values[k] <= left && values[k] <= right && values[k] < least + 1e-3 * scale {
                let (mut lo, mut hi) = (((k as f64) - 1.0).max(0.0) / samples as f64, ((k as f64) + 1.0).min(samples as f64) / samples as f64);
                for _ in 0..80 {
                    let (m1, m2) = (lo + (hi - lo) / 3.0, hi - (hi - lo) / 3.0);
                    if f(m1) < f(m2) { hi = m2 } else { lo = m1 }
                }
                least = least.min(f(0.5 * (lo + hi)));
            }
        }
        least
    };
    // The prism spans the slab's crossing of every line through the region:
    // from below the starting tube to past the final one, along the slide.
    let (a0, a1) = points.iter().map(|q| q.sub(base).dot(up)).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), h| (a.min(h), b.max(h)));
    let margin = radius + step + 1e-3 * scale;
    // Each offset curve ends on its end circle, tangent to it there (both run
    // along the velocity), which no crossing test finds: split the circles at
    // those points (the angles of ±the normal there).
    let mut touches: Vec<(usize, f64)> = Vec::new();
    for (circle, s) in [(2usize, 0.0f64), (3, 1.0)] {
        let w = velocity(s);
        for side in [1.0f64, -1.0] {
            let angle = (side * w[0]).atan2(-side * w[1]);
            touches.push((circle, angle.rem_euclid(TAU) / TAU));
        }
    }
    region_from_paths(
        4,
        &eval,
        &|_| false,
        &|index| index >= 2,
        &touches,
        &depth,
        scale,
        (base, e1, e2, up),
        (a0 - margin, a1 + slide.length() + margin),
    )
}

/// The crossing search's box culling in [`region_from_paths`]
/// (`BREP_SWEEP_REGION_CROSS_BOXES`, default on; `=0` visits every segment
/// pair, the same crossings found more slowly: square-on-a-planet-carrier
/// tests 74.4M pairs without it, 0.75M with it, for the same 74 Newton runs).
fn region_cross_boxes_on() -> bool {
    crate::switch!("BREP_SWEEP_REGION_CROSS_BOXES", true)
}

/// The plane region bounded by pieces of the candidate `count` paths (each
/// `eval(index, t)`, t ∈ [0, 1]), extruded along `up` from `heights.0` to
/// `heights.1`: the paths are split where they cross each other or
/// themselves (and at `extra_cuts`), a piece is kept where `depth` (negative
/// inside the region) reads one side covered and the other not, and the kept
/// pieces chain into one outer loop and its holes. `straight_path(index)`: the
/// path is a segment (built as a line); `closed_path(index)`: its two ends
/// meet (neighbours for the self-crossing test).
#[allow(clippy::too_many_arguments)]
fn region_from_paths(
    count: usize,
    eval: &dyn Fn(usize, f64) -> [f64; 2],
    straight_path: &dyn Fn(usize) -> bool,
    closed_path: &dyn Fn(usize) -> bool,
    extra_cuts: &[(usize, f64)],
    depth: &dyn Fn([f64; 2]) -> f64,
    scale: f64,
    frame: (Vec3, Vec3, Vec3, Vec3),
    heights: (f64, f64),
) -> Result<BrepSolid, KernelRefusal> {
    let (base, e1, e2, up) = frame;
    let (from, to) = heights;
    // Split parameters: mutual crossings (polyline, then Newton), and each
    // path's ends lying on another.
    let m = 400;
    let polylines: Vec<Vec<[f64; 2]>> = (0..count).map(|p| (0..=m).map(|i| eval(p, i as f64 / m as f64)).collect()).collect();
    let mut cuts: Vec<Vec<f64>> = vec![vec![0.0, 1.0]; count];
    let tol = 1e-10 * scale;
    // Under `BREP_SWEEP_REGION_CROSS_BOXES` the segment pairs are visited
    // chunk by chunk (16 segments each), a chunk pair only where their boxes,
    // grown by 1e-6 of the scale and by the pair test's own reach past a
    // segment's ends, overlap: a crossing the test takes lies in both boxes,
    // and none is missed. The cuts are
    // sorted after, so the order they are found in does not matter.
    const CHUNK: usize = 16;
    let boxed = region_cross_boxes_on();
    let chunks = (m + CHUNK - 1) / CHUNK;
    let grow = 1e-6 * scale;
    let boxes: Vec<Vec<[f64; 4]>> = polylines
        .iter()
        .filter(|_| boxed)
        .map(|line| {
            (0..chunks)
                .map(|c| {
                    let (lo, hi) = (c * CHUNK, ((c + 1) * CHUNK).min(m));
                    let b = line[lo..=hi].iter().fold([f64::INFINITY, f64::INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY], |b, p| [b[0].min(p[0]), b[1].min(p[1]), b[2].max(p[0]), b[3].max(p[1])]);
                    // The pair test takes a crossing up to 1e-9 of a segment's
                    // length past its end: grow by twice that of the chunk's
                    // longest segment, so every crossing it takes lies in both
                    // segments' boxes.
                    let longest = line[lo..=hi].windows(2).map(|w| (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1])).fold(0.0f64, f64::max);
                    let g = grow + 2e-9 * longest;
                    [b[0] - g, b[1] - g, b[2] + g, b[3] + g]
                })
                .collect()
        })
        .collect();
    let overlap = |a: &[f64; 4], b: &[f64; 4]| a[0] <= b[2] && b[0] <= a[2] && a[1] <= b[3] && b[1] <= a[3];
    for a in 0..count {
        for b in a..count {
            // One segment pair: a crossing of the polylines, polished by Newton on the exact paths.
            let mut test = |i: usize, j: usize| {
                    // A path against itself: only where it crosses over
                    // (stretches apart along it; a closed path's ends are
                    // neighbours too).
                    if a == b && (j < i + 3 || (closed_path(a) && i < 3 && j + 3 > m)) {
                        return;
                    }
                    let (p0, p1, q0, q1) = (polylines[a][i], polylines[a][i + 1], polylines[b][j], polylines[b][j + 1]);
                    let d1 = [p1[0] - p0[0], p1[1] - p0[1]];
                    let d2 = [q1[0] - q0[0], q1[1] - q0[1]];
                    let det = d1[0] * d2[1] - d1[1] * d2[0];
                    if det.abs() < 1e-300 {
                        return;
                    }
                    let r = [q0[0] - p0[0], q0[1] - p0[1]];
                    let (x, y) = ((r[0] * d2[1] - r[1] * d2[0]) / det, (r[0] * d1[1] - r[1] * d1[0]) / det);
                    if !(-1e-9..=1.0 + 1e-9).contains(&x) || !(-1e-9..=1.0 + 1e-9).contains(&y) {
                        return;
                    }
                    // Newton on the exact paths.
                    let (mut ta, mut tb) = ((i as f64 + x) / m as f64, (j as f64 + y) / m as f64);
                    for _ in 0..50 {
                        let (pa, pb) = (eval(a, ta), eval(b, tb));
                        let h = 1e-7;
                        let da = { let q = eval(a, (ta + h).min(1.0)); let p = eval(a, (ta - h).max(0.0)); let w = (ta + h).min(1.0) - (ta - h).max(0.0); [(q[0] - p[0]) / w, (q[1] - p[1]) / w] };
                        let db = { let q = eval(b, (tb + h).min(1.0)); let p = eval(b, (tb - h).max(0.0)); let w = (tb + h).min(1.0) - (tb - h).max(0.0); [(q[0] - p[0]) / w, (q[1] - p[1]) / w] };
                        let f = [pa[0] - pb[0], pa[1] - pb[1]];
                        let det = -da[0] * db[1] + da[1] * db[0];
                        if det.abs() < 1e-300 {
                            break;
                        }
                        let dta = (-f[0] * db[1] + f[1] * db[0]) / det;
                        let dtb = (da[0] * f[1] - da[1] * f[0]) / det;
                        ta = (ta - dta).clamp(0.0, 1.0);
                        tb = (tb - dtb).clamp(0.0, 1.0);
                        if dta.abs() + dtb.abs() < 1e-15 {
                            break;
                        }
                    }
                    let (pa, pb) = (eval(a, ta), eval(b, tb));
                    if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) < tol && !(a == b && (ta - tb).abs() < 1.0 / m as f64) {
                        cuts[a].push(ta);
                        cuts[b].push(tb);
                    }
            };
            if boxed {
                // Only the segment pairs of chunks whose boxes overlap.
                for ca in 0..chunks {
                    for cb in 0..chunks {
                        if !overlap(&boxes[a][ca], &boxes[b][cb]) {
                            continue;
                        }
                        for i in ca * CHUNK..((ca + 1) * CHUNK).min(m) {
                            for j in cb * CHUNK..((cb + 1) * CHUNK).min(m) {
                                test(i, j);
                            }
                        }
                    }
                }
            } else {
                for i in 0..m {
                    for j in 0..m {
                        test(i, j);
                    }
                }
            }
        }
    }
    // Extra splits the caller names (a corner's path where a grazing point passes it).
    for &(path, t) in extra_cuts {
        cuts[path].push(t);
    }
    for c in &mut cuts {
        c.sort_by(|x, y| x.total_cmp(y));
        c.dedup_by(|x, y| (*x - *y).abs() < 1e-9);
    }
    // Pieces on the boundary: covered on exactly one side.
    struct Piece {
        path: usize,
        t0: f64,
        t1: f64,
    }
    let eps = 1e-7 * scale;
    let mut pieces: Vec<Piece> = Vec::new();
    for (index, c) in cuts.iter().enumerate() {
        for w in c.windows(2) {
            let (t0, t1) = (w[0], w[1]);
            if t1 - t0 < 1e-9 {
                continue;
            }
            let (pa, pb) = (eval(index, t0), eval(index, t1));
            if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) < 1e-9 * scale {
                continue;
            }
            let tm = 0.5 * (t0 + t1);
            let (p, q) = (eval(index, tm), eval(index, (tm + 1e-6).min(1.0)));
            let d = [q[0] - p[0], q[1] - p[1]];
            let length = d[0].hypot(d[1]);
            if length == 0.0 {
                continue;
            }
            let left = [p[0] - d[1] / length * eps, p[1] + d[0] / length * eps];
            let right = [p[0] + d[1] / length * eps, p[1] - d[0] / length * eps];
            let (l, r) = (depth(left) < -1e-12 * scale, depth(right) < -1e-12 * scale);
            match (l, r) {
                (true, false) => pieces.push(Piece { path: index, t0, t1 }),
                (false, true) => pieces.push(Piece { path: index, t0: t1, t1: t0 }),
                _ => {}
            }
        }
    }
    // Chain into loops, the region on their left (the outer one runs
    // counter-clockwise, a hole's clockwise).
    if pieces.is_empty() {
        return Err(refuse("prism_along", "no boundary found".into()));
    }
    let mut used = vec![false; pieces.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    while let Some(first) = (0..pieces.len()).find(|&k| !used[k]) {
        let mut order = vec![first];
        used[first] = true;
        loop {
            let last = &pieces[*order.last().expect("order")];
            let end = eval(last.path, last.t1);
            let start = eval(pieces[order[0]].path, pieces[order[0]].t0);
            if (end[0] - start[0]).hypot(end[1] - start[1]) < 1e-8 * scale {
                break;
            }
            let next = (0..pieces.len()).filter(|&k| !used[k]).min_by(|&x, &y| {
                let (px, py) = (eval(pieces[x].path, pieces[x].t0), eval(pieces[y].path, pieces[y].t0));
                (px[0] - end[0]).hypot(px[1] - end[1]).total_cmp(&(py[0] - end[0]).hypot(py[1] - end[1]))
            });
            let Some(next) = next else {
                return Err(refuse("prism_along", "the boundary does not close".into()));
            };
            let p = eval(pieces[next].path, pieces[next].t0);
            if (p[0] - end[0]).hypot(p[1] - end[1]) > 1e-8 * scale {
                return Err(refuse("prism_along", format!("the boundary does not close (gap {:e})", (p[0] - end[0]).hypot(p[1] - end[1]))));
            }
            used[next] = true;
            order.push(next);
        }
        loops.push(order);
    }
    // The outer loop first: the one of greatest area.
    let area_of = |order: &Vec<usize>| -> f64 {
        let mut points: Vec<[f64; 2]> = Vec::new();
        for &k in order {
            let piece = &pieces[k];
            for i in 0..32 {
                points.push(eval(piece.path, piece.t0 + (piece.t1 - piece.t0) * i as f64 / 32.0));
            }
        }
        (0..points.len()).map(|i| { let (a, b) = (points[i], points[(i + 1) % points.len()]); a[0] * b[1] - b[0] * a[1] }).sum::<f64>() * 0.5
    };
    let areas: Vec<f64> = loops.iter().map(area_of).collect();
    let outer = (0..loops.len()).max_by(|&x, &y| areas[x].abs().total_cmp(&areas[y].abs())).expect("loops");
    if areas[outer] <= 0.0 || (0..loops.len()).any(|k| k != outer && areas[k] >= 0.0) {
        return Err(refuse("prism_along", "the swept region is not one region with holes".into()));
    }
    loops.swap(0, outer);
    // Plane curves of the loops, then the extrusion.
    let world = |q: [f64; 2], h: f64| base.add(e1.scale(q[0])).add(e2.scale(q[1])).add(up.scale(h));
    let mut shell = Shell::new();
    let mut top_loops = Vec::with_capacity(loops.len());
    let mut bottom_loops = Vec::with_capacity(loops.len());
    let mut extent: Vec<[f64; 2]> = Vec::new();
    for order in &loops {
        let mut loop_points: Vec<[f64; 2]> = order.iter().map(|&k| eval(pieces[k].path, pieces[k].t0)).collect();
        loop_points.push(loop_points[0]);
        let count = order.len();
        let (bottom_v, top_v): (Vec<u64>, Vec<u64>) = (0..count).map(|k| (shell.vertex(world(loop_points[k], from)), shell.vertex(world(loop_points[k], to)))).unzip();
        let mut bottom_e = Vec::with_capacity(count);
        let mut top_e = Vec::with_capacity(count);
        let mut curves2d = Vec::with_capacity(count);
        for (slot, &k) in order.iter().enumerate() {
            let piece = &pieces[k];
            let straight = straight_path(piece.path);
            let next = (slot + 1) % count;
            for i in 0..=16 {
                extent.push(eval(piece.path, piece.t0 + (piece.t1 - piece.t0) * i as f64 / 16.0));
            }
            let curve_at = |h: f64| -> Result<NurbsCurve, KernelRefusal> {
                if straight {
                    return frag(make_line(world(loop_points[slot], h), world(loop_points[next], h)), "line");
                }
                let samples = 192;
                let ts: Vec<f64> = (0..=samples).map(|i| i as f64 / samples as f64).collect();
                let mut points: Vec<Vec3> = ts.iter().map(|&t| world(eval(piece.path, piece.t0 + (piece.t1 - piece.t0) * t), h)).collect();
                points[0] = world(loop_points[slot], h);
                points[samples] = world(loop_points[next], h);
                frag(crate::interpolate_curve(&points, 5, &ts), "plane_path")
            };
            let (cb, ct) = (curve_at(from)?, curve_at(to)?);
            bottom_e.push(shell.edge(cb.clone(), bottom_v[slot], bottom_v[next], false)?);
            top_e.push(shell.edge(ct.clone(), top_v[slot], top_v[next], false)?);
            curves2d.push((cb, ct));
        }
        let verticals: Vec<u64> = (0..count).map(|k| shell.edge(make_line(world(loop_points[k], from), world(loop_points[k], to)).expect("line"), bottom_v[k], top_v[k], false)).collect::<Result<_, _>>()?;
        // Top (outward up) and bottom (outward down) loops.
        let mut coedges = Vec::with_capacity(count);
        for k in 0..count {
            coedges.push((top_e[k], true, plane_pcurve(world([0.0, 0.0], to), e1, e2, &curves2d[k].1, true)?));
        }
        top_loops.push(coedges);
        let mut coedges = Vec::with_capacity(count);
        for k in (0..count).rev() {
            coedges.push((bottom_e[k], false, plane_pcurve(world([0.0, 0.0], from), e2, e1, &curves2d[k].0, false)?));
        }
        bottom_loops.push(coedges);
        // Sides: the bottom curve extruded (u along it, v up), outward on its right.
        for k in 0..count {
            let next = (k + 1) % count;
            let (cb, _) = &curves2d[k];
            let rows: Vec<Vec<Vec4>> = cb
                .control_points
                .iter()
                .map(|p| {
                    let x = p.point().expect("point");
                    vec![Vec4::from_point(x, p.w), Vec4::from_point(x.add(up.scale(to - from)), p.w)]
                })
                .collect();
            let surface = frag(NurbsSurface::new(cb.degree, 1, cb.knots.clone(), vec![0.0, 0.0, 1.0, 1.0], rows), "side")?;
            let [u0, u1] = frag(cb.domain(), "domain")?;
            let coedges = vec![
                (bottom_e[k], true, uv_line([u0, 0.0], [u1, 0.0])?),
                (verticals[next], true, uv_line([u1, 0.0], [u1, 1.0])?),
                (top_e[k], false, uv_line([u1, 1.0], [u0, 1.0])?),
                (verticals[k], false, uv_line([u0, 1.0], [u0, 0.0])?),
            ];
            shell.face(surface, coedges);
        }
    }
    let top_points: Vec<Vec3> = extent.iter().map(|&q| world(q, to)).collect();
    shell.face_loops(plane_patch(world([0.0, 0.0], to), e1, e2, &top_points)?, top_loops);
    let bottom_points: Vec<Vec3> = extent.iter().map(|&q| world(q, from)).collect();
    shell.face_loops(plane_patch(world([0.0, 0.0], from), e2, e1, &bottom_points)?, bottom_loops);
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse("assembly", format!("the swept prism is not valid: {:?}", issues.first())));
    }
    Ok(body)
}

/// A prism whose section is a convex polygon, carried by turns about axes
/// all along its own (a planar motion: a planet on its carrier) that ends
/// where it starts: its whole sweep (`prism_region_sweep`), or None when the
/// tool or motion is not of that kind. `steps`: each turn's axis point,
/// direction and rate, applied in order at every t ∈ [t0, t1].
#[allow(clippy::too_many_arguments)]
pub fn closed_planar_prism(
    origin: Vec3,
    normal: Vec3,
    x_axis: Vec3,
    outline: &[OutlinePiece],
    from: f64,
    to: f64,
    steps: &[(Vec3, Vec3, f64, f64)],
    t0: f64,
    t1: f64,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    let Ok(up) = normal.normalized() else { return Ok(None) };
    let Ok(e1) = x_axis.sub(up.scale(x_axis.dot(up))).normalized() else { return Ok(None) };
    let e2 = up.cross(e1);
    // A polygon of straight sides, in order.
    let mut corners: Vec<[f64; 2]> = Vec::with_capacity(outline.len());
    for (k, piece) in outline.iter().enumerate() {
        let OutlinePiece::Line { from: a, to: b } = piece else { return Ok(None) };
        let OutlinePiece::Line { from: next, .. } = &outline[(k + 1) % outline.len()] else { return Ok(None) };
        if (b[0] - next[0]).hypot(b[1] - next[1]) > 1e-9 * (1.0 + b[0].hypot(b[1])) {
            return Ok(None);
        }
        corners.push(*a);
    }
    if corners.len() < 3 || steps.is_empty() || !(t1 > t0) {
        return Ok(None);
    }
    // Convex (either way round), or it is another engine's.
    let turn_of = |k: usize| {
        let (a, b, c) = (corners[k], corners[(k + 1) % corners.len()], corners[(k + 2) % corners.len()]);
        (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
    };
    let first_turn = turn_of(0);
    if first_turn == 0.0 || (1..corners.len()).any(|k| turn_of(k) * first_turn <= 0.0) {
        return Ok(None);
    }
    // Turns only, about axes along the prism's own: (centre in the plane, rate about +up).
    let mut turns: Vec<([f64; 2], f64)> = Vec::with_capacity(steps.len());
    for &(point, direction, turn, advance) in steps {
        let Ok(axis) = direction.normalized() else { return Ok(None) };
        if advance != 0.0 || axis.cross(up).length() > 1e-12 {
            return Ok(None);
        }
        let r = point.sub(origin);
        turns.push(([r.dot(e1), r.dot(e2)], turn * axis.dot(up).signum()));
    }
    let moved = |t: f64, q: [f64; 2]| -> [f64; 2] {
        let mut x = q;
        for &(c, rate) in &turns {
            let (sn, cs) = (rate * t).sin_cos();
            let d = [x[0] - c[0], x[1] - c[1]];
            x = [c[0] + cs * d[0] - sn * d[1], c[1] + sn * d[0] + cs * d[1]];
        }
        x
    };
    let total: f64 = turns.iter().map(|t| t.1).sum();
    // The turns compose to one rigid motion of the plane: a point q (where
    // it stands before any motion) is at moved(t, 0) + R(total t) q.
    let pose = |s: f64| -> ([f64; 2], f64) {
        let t = t0 + (t1 - t0) * s;
        (moved(t, [0.0, 0.0]), total * t)
    };
    // The turns are defined at every t, so the origin's velocity is a
    // centred difference at the motion's ends too.
    let rates = |s: f64| -> ([f64; 2], f64) {
        let h = 1e-6;
        let (pa, pb) = (pose(s - h).0, pose(s + h).0);
        ([(pb[0] - pa[0]) / (2.0 * h), (pb[1] - pa[1]) / (2.0 * h)], total * (t1 - t0))
    };
    // Ends where it starts: the same turn (mod 2π) and place.
    let scale = corners.iter().map(|c| c[0].hypot(c[1])).fold(1.0f64, f64::max);
    let closes = corners.iter().all(|&c| {
        let (a, b) = (moved(t0, c), moved(t1, c));
        (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9 * scale
    });
    if !closes {
        return Ok(None);
    }
    prism_region_sweep(&corners, from, to, origin, e1, e2, up, &pose, &rates, true).map(Some)
}

/// Two turns driven by one parameter t: first by `rates[0]·t` about the
/// first axis (point, direction), then by `rates[1]·t` about the second; both
/// axes fixed in the world; t from `from` to `to`.
#[derive(Clone, Copy, Debug)]
pub struct TwoTurns {
    pub axes: [(Vec3, Vec3); 2],
    pub rates: [f64; 2],
    pub from: f64,
    pub to: f64,
}

/// The whole sweep of a solid of revolution tumbled by two turns about skew
/// axes. The grazing loop moves on the tool (the instantaneous screw changes
/// with t), so the walls are lofted through exact samples. Where the tumble
/// passes over a point twice (the one-pass probe), the time is bisected into
/// spans each passing once, and the spans' sweeps are joined (consecutive
/// spans meet along a grazing loop; distant ones overlap).
pub fn sweep_solid_tumbled(tool: &SolidTool, motion: &TwoTurns) -> Result<BrepSolid, KernelRefusal> {
    // A ring whose swept tube K bounds the solid outright, by the tube route
    // (BREP_SWEEP_TORUS_TUBE, default off); it declines anything else.
    if crate::solid_sweep_tube::torus_tube_on() {
        if let Some(tube) = ring_tube(tool, motion) {
            if let Ok(solid) = crate::solid_sweep_tube::sweep_torus_tube(&tube) {
                return Ok(solid);
            }
        }
    }
    let lofted = sweep_solid_tumbled_lofted(tool, motion);
    // A ring (a whole circle off its axis) the lofted walls refuse, by the
    // trim engine (BREP_SWEEP_RING_TRIM); the lofted refusal stands where it
    // refuses too. Rows the walls build are left to them.
    let Err(refusal) = lofted else {
        return lofted.map(|mut solid| {
            if cusp_first_on() {
                start_loops_at_cusps(&mut solid);
            }
            solid
        });
    };
    // A wall whose double curve reaches a corner: the trim route meets that
    // double curve's tip, which it does not yet write so a reader integrates
    // round it, and refuses after the work; declined before it.
    if !crate::solid_sweep_trim::ring_trim_on() || (refusal.to_string().contains(DOUBLE_CURVE_CORNER) && !crate::solid_sweep_trim::tip_splice_on()) {
        return Err(refusal);
    }
    let SolidTool::Revolved { axis_origin, axis_direction, meridian, .. } = tool else { return Err(refusal) };
    let [OutlinePiece::Arc { centre, radius, start, end }] = meridian.as_slice() else { return Err(refusal) };
    if !(((end - start).abs() - TAU).abs() < 1e-12 && centre[0] > *radius) {
        return Err(refusal);
    }
    let (Ok(axis), Ok(d0), Ok(d1)) = (axis_direction.normalized(), motion.axes[0].1.normalized(), motion.axes[1].1.normalized()) else { return Err(refusal) };
    let two = crate::solid_sweep_trim::TwoTurnMotion { points: [motion.axes[0].0, motion.axes[1].0], directions: [d0, d1], rates: motion.rates };
    let ring_centre = axis_origin.add(axis.scale(centre[1]));
    crate::solid_sweep_trim::sweep_ring_tumbled(ring_centre, axis, centre[0], *radius, &two, motion.from, motion.to).or(Err(refusal))
}

/// `BREP_SWEEP_CUSP_FIRST` (on unless =0): each loop of a tumbled sweep starts
/// at its cusp corner, where it has one.
fn cusp_first_on() -> bool {
    crate::switch!("BREP_SWEEP_CUSP_FIRST", true)
}

/// Each loop rotated to start at its cusp corner (a zero-angle corner: the
/// edge leaving it runs back along the edge arriving, within 0.05 rad), where
/// it has one; the sharpest such corner where it has several. Topology and
/// geometry are unchanged. A reader that places a loop on its surface by
/// inverting the edges' points, each edge seeded from the last one's end,
/// mislays a corner where an edge creeps into a cusp (its point all but still
/// while its parameters still move): from the cusp it starts clean.
pub(crate) fn start_loops_at_cusps(solid: &mut BrepSolid) {
    let ends: HashMap<u64, [(Vec3, Vec3); 2]> = solid
        .edges
        .iter()
        .filter_map(|e| {
            let span = e.t1 - e.t0;
            let at = |t: f64| e.curve.evaluate(t).ok();
            let (p0, p1) = (at(e.t0)?, at(e.t1)?);
            let (q0, q1) = (at(e.t0 + 1e-2 * span)?, at(e.t1 - 1e-2 * span)?);
            // (point, direction into the edge) at its start and at its end
            Some((e.id, [(p0, q0.sub(p0)), (p1, q1.sub(p1))]))
        })
        .collect();
    for face in solid.shells.iter_mut().flat_map(|s| s.faces.iter_mut()) {
        for l in face.loops.iter_mut() {
            let n = l.coedges.len();
            if n < 2 {
                continue;
            }
            let mut best: Option<(f64, usize)> = None;
            for k in 0..n {
                let (before, after) = (&l.coedges[(k + n - 1) % n], &l.coedges[k]);
                let (Some(b), Some(a)) = (ends.get(&before.edge_id), ends.get(&after.edge_id)) else { continue };
                // into the corner back along the edge arriving, and out along the edge leaving
                let back = if before.forward { b[1].1 } else { b[0].1 };
                let out = if after.forward { a[0].1 } else { a[1].1 };
                let (lb, lo) = (back.length(), out.length());
                if !(lb > 0.0 && lo > 0.0) {
                    continue;
                }
                let along = back.dot(out) / (lb * lo);
                if along > (0.05f64).cos() && best.map_or(true, |(x, _)| along > x) {
                    best = Some((along, k));
                }
            }
            if let Some((_, k)) = best {
                l.coedges.rotate_left(k);
            }
        }
    }
}

/// The tube route's reading of a ring tool (a whole circle off its axis) under two turns.
fn ring_tube(tool: &SolidTool, motion: &TwoTurns) -> Option<crate::solid_sweep_tube::Tube> {
    let SolidTool::Revolved { axis_origin, axis_direction, meridian, .. } = tool else { return None };
    let [OutlinePiece::Arc { centre, radius, start, end }] = meridian.as_slice() else { return None };
    if !(((end - start).abs() - TAU).abs() < 1e-12 && centre[0] > *radius && motion.from.is_finite() && motion.to > motion.from) {
        return None;
    }
    let (axis, d0, d1) = (axis_direction.normalized().ok()?, motion.axes[0].1.normalized().ok()?, motion.axes[1].1.normalized().ok()?);
    let two = crate::solid_sweep_trim::TwoTurnMotion { points: [motion.axes[0].0, motion.axes[1].0], directions: [d0, d1], rates: motion.rates };
    crate::solid_sweep_tube::Tube::new(axis_origin.add(axis.scale(centre[1])), axis, centre[0], *radius, two, motion.from, motion.to)
}

/// The lofted walls' refusal where a wall's double curve reaches a corner of
/// its grazing loop (the tumbled sweep reads it to skip the ring's trim route).
#[doc(hidden)]
pub const DOUBLE_CURVE_CORNER: &str = "the double curve of a wall reaches a corner of its grazing loop";

/// The tumbled sweep by the lofted walls (a polyhedron by the trim engine).
fn sweep_solid_tumbled_lofted(tool: &SolidTool, motion: &TwoTurns) -> Result<BrepSolid, KernelRefusal> {
    if !(motion.from.is_finite() && motion.to.is_finite() && motion.to > motion.from) {
        return Err(input("range", "the motion's range must run forward".into()));
    }
    let mut points = [Vec3::new(0.0, 0.0, 0.0); 2];
    let mut directions = [Vec3::new(0.0, 0.0, 1.0); 2];
    for k in 0..2 {
        points[k] = motion.axes[k].0;
        directions[k] = motion.axes[k].1.normalized().map_err(|_| input("axis", "a turn's axis has no direction".into()))?;
    }
    let turns = Turns { points, directions, rates: motion.rates };
    // A convex polyhedron, by the trim engine (default off).
    if let SolidTool::Polyhedron { vertices, faces } = tool {
        if crate::solid_sweep_trim::trim_engine_on() {
            let two = crate::solid_sweep_trim::TwoTurnMotion { points, directions, rates: motion.rates };
            return crate::solid_sweep_trim::sweep_convex_tumbled(vertices, faces, &two, motion.from, motion.to);
        }
    }
    let SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } = tool else {
        return Err(refuse("tumble_tool", "a tumble sweeps a solid of revolution only".into()));
    };
    let axis = axis_direction.normalized().map_err(|_| input("axis", "the tool's axis has no direction".into()))?;
    let base = Screw {
        a: points[0],
        e: directions[0],
        omega: motion.rates[0].abs() + motion.rates[1].abs(),
        c: 0.0,
        c2: 0.0,
        t0: motion.from,
        t1: motion.to,
        frame: None,
        turns: Some(turns),
        frozen: None,
        loop_samples: LOFT_SAMPLES,
        extent: points[1].sub(points[0]).length(),
    };
    let span = |t0: f64, t1: f64| -> Result<BrepSolid, KernelRefusal> {
        let mut screw = base;
        screw.t0 = t0;
        screw.t1 = t1;
        let mut last = None;
        // The loop's sampling doubles while the walls' rows miss between
        // samples (a thin ring's loop bends sharply before it folds).
        'samples: while screw.loop_samples <= 16 * LOFT_SAMPLES {
            for k in 0..8 {
                let reference = radial.rotate_about(axis, PI * k as f64 / 4.0);
                match lofted(&screw, *axis_origin, axis, reference, meridian) {
                    // A bulge that crosses a seam high up reads on the next
                    // wall as a part held away from its end: other seams too.
                    Err(error) if matches!(refused_as(&error), Some("loft_structure") | Some("end_trim_corner") | Some("end_trim_deep")) => last = Some(error),
                    Err(error) if refused_as(&error) == Some("loft_rows") => {
                        last = Some(error);
                        screw.loop_samples *= 2;
                        continue 'samples;
                    }
                    other => return other,
                }
            }
            break;
        }
        Err(last.expect("tried"))
    };
    // Bisect where the sweep folds over itself.
    let mut pending = vec![(motion.from, motion.to, 0usize)];
    let mut spans: Vec<(f64, BrepSolid)> = Vec::new();
    while let Some((t0, t1, depth)) = pending.pop() {
        match span(t0, t1) {
            Ok(body) => spans.push((t0, body)),
            Err(error) if refused_as(&error) == Some("folds") && depth < 6 => {
                let middle = 0.5 * (t0 + t1);
                pending.push((middle, t1, depth + 1));
                pending.push((t0, middle, depth + 1));
            }
            Err(error) => return Err(error),
        }
    }
    spans.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut bodies: Vec<BrepSolid> = spans.into_iter().map(|(_, body)| body).collect();
    if bodies.len() == 1 {
        return Ok(bodies.pop().expect("one"));
    }
    crate::boolean_operation_nary(&bodies, crate::BooleanOperation::Union)
}

/// A cap's pcurve for a lofted edge through the cap's exact chart (the edge
/// read back into the tool's frame at the cap's time).
fn charted_pcurve(chart: &Chart, screw: &Screw, t: f64, curve: &NurbsCurve, forward: bool) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = frag(curve.domain(), "domain")?;
    let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
    let mut uv = Vec::with_capacity(ts.len());
    let mut poles = Vec::with_capacity(ts.len());
    for &tt in &ts {
        let x = screw.invert(t, frag(curve.evaluate(tt), "point")?);
        let [u, v] = chart.invert(x);
        uv.push(Vec3::new(u, v, 0.0));
        poles.push(chart.at_pole(x));
    }
    // Where the path crosses a span joint of the chart, one quintic through
    // it rings between samples (the parameter's second derivative jumps
    // there): by the run between joints instead (`BREP_SWEEP_PCURVE_JOINTS`).
    if pcurve_joints_on() && !poles.iter().any(|&p| p) {
        let (joints_u, joints_v) = chart.joints();
        let at = |tt: f64| -> Result<[f64; 2], KernelRefusal> { Ok(chart.invert(screw.invert(t, frag(curve.evaluate(tt), "point")?))) };
        if let Some(fitted) = jointed_pcurve(&ts, &uv, at, &joints_u, &joints_v, curve)? {
            return if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") };
        }
    }
    continue_pole_angles(&mut uv, &ts, &poles)?;
    let fitted = frag(crate::interpolate_curve(&uv, 5, &ts), "charted_pcurve")?;
    if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") }
}

/// A surface's joints in each parameter: its interior knots that leave it C1
/// or less (a rational circle's spans meet at double knots), where a chart's
/// parameter bends.
fn surface_joints(surface: &NurbsSurface) -> (Vec<f64>, Vec<f64>) {
    let joints = |knots: &[f64], degree: usize| -> Vec<f64> {
        let (first, last) = (knots[0], knots[knots.len() - 1]);
        let mut out: Vec<f64> = Vec::new();
        for (i, &k) in knots.iter().enumerate() {
            if k > first && k < last && (i == 0 || knots[i - 1] != k) && knots.iter().filter(|&&x| x == k).count() + 1 >= degree {
                out.push(k);
            }
        }
        out
    };
    (joints(&surface.knots_u, surface.degree_u), joints(&surface.knots_v, surface.degree_v))
}

thread_local! {
    /// Set while a split is read only for its curves and structure (a loft's
    /// interior times): its caps' pcurves are then never used.
    static PCURVES_SKIPPED: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
}

/// The switch for skipping the cap pcurves of a loft's interior splits
/// (default on, `=0` fits them all; the body is the same either way).
fn loft_splits_lean_on() -> bool {
    crate::switch!("BREP_SWEEP_LOFT_SPLITS_LEAN", true)
}

/// `action` with the split's cap pcurves read at their ends only: a split at
/// a loft's interior time is read for its curves, samples and structure, its
/// caps' pcurves never (only the end poses' caps are built).
fn with_pcurves_skipped<T>(action: impl FnOnce() -> T) -> T {
    // Restored on the way out, an unwind too, so a caught panic leaves no
    // later sweep on this thread building its end caps' pcurves as lines.
    struct Restore(bool);
    impl Drop for Restore {
        fn drop(&mut self) {
            PCURVES_SKIPPED.with(|cell| cell.set(self.0));
        }
    }
    let _restore = Restore(PCURVES_SKIPPED.with(|cell| cell.replace(true)));
    action()
}

/// A grazing curve through its exact samples (quintic); in a split read only
/// for its curves' samples and structure ([`with_pcurves_skipped`]: a loft's
/// interior times), the line between its ends, never read as a curve.
fn fit_tool_curve(points: &[Vec3], params: &[f64]) -> Result<NurbsCurve, String> {
    if PCURVES_SKIPPED.with(|cell| cell.get()) && points.len() >= 2 && params.len() == points.len() {
        let (a, b) = (params[0], params[params.len() - 1]);
        return NurbsCurve::new(1, vec![a, a, b, b], vec![Vec4::from_point(points[0], 1.0), Vec4::from_point(points[points.len() - 1], 1.0)]);
    }
    crate::interpolate_curve(points, 5, params)
}

/// A cap pcurve's stations: the whole set, or its two ends where the pcurve
/// is never used ([`with_pcurves_skipped`]).
fn pcurve_stations(d0: f64, d1: f64) -> Vec<f64> {
    if PCURVES_SKIPPED.with(|cell| cell.get()) {
        return vec![d0, d1];
    }
    (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect()
}

/// The quintic through a cap pcurve's stations; through two (a skipped
/// pcurve), the line between its ends.
fn fit_pcurve(uv: &[Vec3], ts: &[f64]) -> Result<NurbsCurve, String> {
    if ts.len() == 2 {
        return NurbsCurve::new(1, vec![ts[0], ts[0], ts[1], ts[1]], uv.iter().map(|&p| Vec4::from_point(p, 1.0)).collect());
    }
    crate::interpolate_curve(uv, 5, ts)
}

/// The switch for [`jointed_pcurve`] (default on; `=0` fits one quintic as before).
fn pcurve_joints_on() -> bool {
    crate::switch!("BREP_SWEEP_PCURVE_JOINTS", true)
}

/// A cap's pcurve broken where its path crosses a span joint of its chart:
/// the edge's parameter at each crossing found by bisection (`at` reads the
/// path's chart parameters at an edge parameter), each run between them
/// sampled as densely as the whole and interpolated (quintic) on its own, the
/// runs joined C0 at the crossings, so each run interpolates a smooth function
/// and the trim keeps to its edge at the same parameter, not only at the
/// samples. `ts`, `uv`: the whole's samples; `edge`: the curve the path
/// reads (its own C1-or-less knots are cut too). None where the path crosses
/// no joint (the one quintic is smooth there).
fn jointed_pcurve(ts: &[f64], uv: &[Vec3], at: impl Fn(f64) -> Result<[f64; 2], KernelRefusal>, joints_u: &[f64], joints_v: &[f64], edge: &NurbsCurve) -> Result<Option<NurbsCurve>, KernelRefusal> {
    if ts.len() < 3 || ts.len() != uv.len() {
        return Ok(None);
    }
    let (d0, d1) = (ts[0], ts[ts.len() - 1]);
    // The edge's own joints: an interior knot that leaves it C1 or less (a
    // rational arc's spans) kinks its parameter, so the path's too.
    let mut cuts: Vec<f64> = Vec::new();
    for (index, &k) in edge.knots.iter().enumerate() {
        if k > d0 && k < d1 && (index == 0 || edge.knots[index - 1] != k) {
            let multiplicity = edge.knots.iter().filter(|&&x| x == k).count();
            if multiplicity + 1 >= edge.degree {
                cuts.push(k);
            }
        }
    }
    for j in 0..ts.len() - 1 {
        let (a, b) = (uv[j], uv[j + 1]);
        for (coord, joints) in [(0usize, joints_u), (1, joints_v)] {
            let (ca, cb) = if coord == 0 { (a.x, b.x) } else { (a.y, b.y) };
            for &k in joints {
                // A sample on the joint itself is the cut (a path that only
                // touches it is cut there too, harmlessly).
                if ca == k && j > 0 {
                    cuts.push(ts[j]);
                    continue;
                }
                if (ca - k) * (cb - k) < 0.0 {
                    // Bisect the edge's parameter for the crossing.
                    let (mut lo, mut hi) = (ts[j], ts[j + 1]);
                    let side = ca < k;
                    for _ in 0..80 {
                        let mid = 0.5 * (lo + hi);
                        if mid <= lo || mid >= hi {
                            break;
                        }
                        if (at(mid)?[coord] < k) == side { lo = mid } else { hi = mid }
                    }
                    cuts.push(0.5 * (lo + hi));
                }
            }
        }
    }
    if cuts.is_empty() {
        return Ok(None);
    }
    // Each run at least a millionth of the whole: a cut nearer an end or
    // another than that is dropped (its run would be too short to fit).
    let least = 1e-6 * (d1 - d0);
    cuts.sort_by(f64::total_cmp);
    let mut bounds = vec![d0];
    for c in cuts {
        if c - bounds[bounds.len() - 1] >= least && d1 - c >= least {
            bounds.push(c);
        }
    }
    if bounds.len() == 1 {
        return Ok(None);
    }
    bounds.push(d1);
    let samples = ts.len() - 1;
    let degree = 5;
    let mut knots: Vec<f64> = Vec::new();
    let mut poles: Vec<Vec4> = Vec::new();
    let runs = bounds.len() - 1;
    for (index, run) in bounds.windows(2).enumerate() {
        let (a, b) = (run[0], run[1]);
        let count = ((samples as f64 * (b - a) / (d1 - d0)).ceil() as usize).max(degree + 1);
        let params: Vec<f64> = (0..=count).map(|i| if i == count { b } else { a + (b - a) * i as f64 / count as f64 }).collect();
        let points: Vec<Vec3> = params.iter().map(|&tt| at(tt).map(|[u, v]| Vec3::new(u, v, 0.0))).collect::<Result<_, _>>()?;
        // A run that does not fit falls back to the one quintic, never a refusal.
        let Ok(piece) = crate::interpolate_curve(&points, degree, &params) else { return Ok(None) };
        if piece.degree != degree {
            return Ok(None);
        }
        let n = piece.knots.len();
        if index == 0 {
            knots.extend_from_slice(&piece.knots[..n - (degree + 1)]);
            poles.extend_from_slice(&piece.control_points);
        } else {
            // The joint: its parameter `degree` times (C0), this run's interior
            // knots; its first pole is the previous run's last.
            knots.extend(std::iter::repeat_n(a, degree));
            knots.extend_from_slice(&piece.knots[degree + 1..n - (degree + 1)]);
            poles.extend_from_slice(&piece.control_points[1..]);
        }
        if index + 1 == runs {
            knots.extend(std::iter::repeat_n(b, degree + 1));
        }
    }
    Ok(NurbsCurve::new(degree, knots, poles).ok())
}

/// An edge through its surface's pole (a ball carried square to its own
/// axis: every grazing circle runs pole to pole) has no angle there, and the
/// inverse returns any. Each sample at a pole (`poles`) takes the first
/// parameter its neighbours run to, continued in a line.
fn continue_pole_angles(uv: &mut [Vec3], ts: &[f64], poles: &[bool]) -> Result<(), KernelRefusal> {
    for j in 0..uv.len() {
        if !poles[j] {
            continue;
        }
        let near = if j + 2 < uv.len() && !poles[j + 1] && !poles[j + 2] {
            [j + 1, j + 2]
        } else if j >= 2 && !poles[j - 1] && !poles[j - 2] {
            [j - 1, j - 2]
        } else {
            return Err(refuse("cap_pole", "a lofted edge stays on its cap's pole".into()));
        };
        let (a, b) = (uv[near[0]].x, uv[near[1]].x);
        uv[j].x = a + (a - b) * (ts[j] - ts[near[0]]) / (ts[near[0]] - ts[near[1]]);
    }
    Ok(())
}

/// A cap's pcurve for a lofted edge: its dense image on the cap's surface.
fn projected_pcurve(surface: &NurbsSurface, curve: &NurbsCurve, forward: bool) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = frag(curve.domain(), "domain")?;
    let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
    let mut uv = Vec::with_capacity(ts.len());
    let mut poles = Vec::with_capacity(ts.len());
    let [u0, u1] = frag(surface.domain_u(), "domain")?;
    let step = 1e-3 * (u1 - u0);
    for &t in &ts {
        let x = frag(curve.evaluate(t), "point")?;
        let foot = frag(crate::project_point_to_surface(surface, x), "project")?;
        if foot.distance > 1e-6 * (1.0 + x.length()) {
            return Err(refuse("loft_cap", format!("a lofted edge leaves its cap by {} at {x:?}", foot.distance)));
        }
        uv.push(Vec3::new(foot.u, foot.v, 0.0));
        // A pole of the surface: its points do not move with the first
        // parameter there.
        let aside = if foot.u + step <= u1 { foot.u + step } else { foot.u - step };
        let moved = frag(surface.evaluate(aside, foot.v), "point")?.sub(frag(surface.evaluate(foot.u, foot.v), "point")?).length();
        poles.push(moved <= 1e-9 * (1.0 + x.length()));
    }
    continue_pole_angles(&mut uv, &ts, &poles)?;
    let fitted = frag(crate::interpolate_curve(&uv, 5, &ts), "projected_pcurve")?;
    if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") }
}

type Signature = (Vec<(usize, usize, bool)>, Vec<(bool, Vec<(usize, bool)>)>, Vec<(usize, bool)>);

fn signature(split: &Split) -> Result<Signature, KernelRefusal> {
    Ok((
        split.curves.iter().map(|c| (c.start, c.end, c.degenerate)).collect(),
        split.caps.iter().map(|cap| (cap.front, cap.coedges.iter().map(|&(c, f, _)| (c, f)).collect())).collect(),
        split.grazing()?,
    ))
}

fn lofted(screw: &Screw, origin: Vec3, axis: Vec3, radial: Vec3, meridian: &[OutlinePiece]) -> Result<BrepSolid, KernelRefusal> {
    let split_at = |t: f64| -> Result<Split, KernelRefusal> {
        let mut frozen = *screw;
        frozen.frozen = Some(t);
        split_revolved(&frozen, origin, axis, radial, meridian)
    };
    let mut lambdas: Vec<f64> = (0..=LOFT_SAMPLES).map(|i| i as f64 / LOFT_SAMPLES as f64).collect();
    // World samples of grazing curve c at time t, in the wall's direction.
    let samples_of = |split: &Split, c: usize, back_forward: bool, t: f64| -> Result<Vec<Vec3>, KernelRefusal> {
        let Some(list) = &split.curves[c].samples else {
            return Err(refuse("loft_samples", "a grazing curve has no exact samples to loft".into()));
        };
        let mut world: Vec<Vec3> = list.iter().map(|&x| screw.apply(t, x)).collect();
        if back_forward {
            world.reverse();
        }
        Ok(world)
    };
    // Time breaks start even and are refined only in the spans whose middle
    // misses, by the quintic's h^6 error: a smooth stretch of the motion keeps
    // its long spans (doubling everywhere tripled the wall for one bend).
    let mut fractions: Vec<f64> = (0..=15).map(|j| j as f64 / 15.0).collect();
    loop {
        let count = fractions.len() - 1;
        // The walls' second parameter runs 0 -> 1 over the motion.
        let times: Vec<f64> = fractions.iter().map(|&f| if f == 1.0 { screw.t1 } else { screw.t0 + (screw.t1 - screw.t0) * f }).collect();
        // Only the end poses' caps are built: the interior splits are read
        // for their curves and structure, their caps' pcurves skipped
        // (`BREP_SWEEP_LOFT_SPLITS_LEAN`).
        let lean = loft_splits_lean_on();
        let splits: Vec<Split> = times
            .iter()
            .enumerate()
            .map(|(j, &t)| if lean && j != 0 && j != count { with_pcurves_skipped(|| split_at(t)) } else { split_at(t) })
            .collect::<Result<_, _>>()?;
        if splits.iter().any(|split| !split.slides.is_empty() || !split.slide_sides.is_empty()) {
            return Err(refuse("loft_structure", "a flat end that slides within its own plane is not lofted".into()));
        }
        let shape = signature(&splits[0])?;
        for split in &splits[1..] {
            if signature(split)? != shape {
                return Err(refuse("loft_structure", "the grazing loop changes its structure along the curve".into()));
            }
        }
        let grazing = shape.2.clone();
        if grazing.is_empty() {
            return Err(refuse("no_grazing", "the tool has no grazing loop under this motion".into()));
        }
        let scale = splits[0].scale + screw.extent;
        let target = LOFT_TARGET * scale;
        // Walls: per grazing curve, rows along it at each time, then across time.
        let mut walls = Vec::with_capacity(grazing.len());
        // Every wall's exact samples at every time.
        let mut sampled: Vec<Vec<Vec<Vec3>>> = Vec::with_capacity(grazing.len());
        for &(c, back_forward) in &grazing {
            let mut at_times = Vec::with_capacity(times.len());
            for (j, split) in splits.iter().enumerate() {
                let points = samples_of(split, c, back_forward, times[j])?;
                if points.len() != lambdas.len() {
                    if points.len() < 13 || points.len() % 2 == 0 {
                        return Err(refuse("loft_samples", "a grazing curve's exact samples cannot be lofted".into()));
                    }
                    lambdas = (0..points.len()).map(|i| i as f64 / (points.len() - 1) as f64).collect();
                }
                at_times.push(points);
            }
            sampled.push(at_times);
        }
        // A row through every `stride`-th sample: its worst miss at the
        // samples it leaves out, over every wall and time.
        let thinned = |stride: usize| -> Result<f64, KernelRefusal> {
            let coarse_lambdas: Vec<f64> = lambdas.iter().step_by(stride).copied().collect();
            let mut worst = 0.0f64;
            for at_times in &sampled {
                for points in at_times {
                    let coarse_points: Vec<Vec3> = points.iter().step_by(stride).copied().collect();
                    let coarse = frag(crate::interpolate_curve(&coarse_points, 5, &coarse_lambdas), "loft_row")?;
                    for (i, x) in points.iter().enumerate().filter(|(i, _)| i % stride != 0) {
                        worst = worst.max(frag(coarse.evaluate(lambdas[i]), "loft_row")?.sub(*x).length());
                    }
                }
            }
            Ok(worst)
        };
        // Each wall's rows pass through as few of its samples as hold every
        // exact sample within a quarter of the target, at every time (a loop
        // smooth but for one sharp bend needs its samples only there, and the
        // wall's size is what the solid's users pay for): from every 16th,
        // a stretch that misses at a sample it leaves out gets its middle one.
        let spans = lambdas.len() - 1;
        let mut kept: Vec<Vec<usize>> = Vec::with_capacity(sampled.len());
        for at_times in &sampled {
            let coarsest = if spans % 16 == 0 && spans >= 96 { 16 } else { 1 };
            let mut subset: Vec<usize> = (0..=spans).step_by(coarsest).collect();
            loop {
                let subset_lambdas: Vec<f64> = subset.iter().map(|&i| lambdas[i]).collect();
                let mut misses = vec![0.0f64; subset.len() - 1];
                for points in at_times {
                    let row_points: Vec<Vec3> = subset.iter().map(|&i| points[i]).collect();
                    let row = frag(crate::interpolate_curve(&row_points, 5, &subset_lambdas), "loft_row")?;
                    for k in 0..subset.len() - 1 {
                        for i in subset[k] + 1..subset[k + 1] {
                            misses[k] = misses[k].max(frag(row.evaluate(lambdas[i]), "loft_row")?.sub(points[i]).length());
                        }
                    }
                }
                let mut refined = Vec::with_capacity(2 * subset.len());
                for k in 0..subset.len() - 1 {
                    refined.push(subset[k]);
                    if misses[k] > 0.25 * target {
                        refined.push((subset[k] + subset[k + 1]) / 2);
                    }
                }
                refined.push(spans);
                if refined.len() == subset.len() {
                    break;
                }
                subset = refined;
            }
            kept.push(subset);
        }
        // Where a wall keeps neighbouring samples there is none left out
        // between them to measure at: a row passes through its samples, and
        // between them it is as good as the loop is smooth at that spacing.
        // Its miss there is estimated from the rows through every other
        // sample, scaled by the quintic's 2^-6.
        let dense = kept.iter().any(|subset| subset.windows(2).any(|pair| pair[1] == pair[0] + 1));
        let between = if dense { thinned(2)? / 64.0 } else { 0.0 };
        for (index, &(c, back_forward)) in grazing.iter().enumerate() {
            let row_lambdas: Vec<f64> = kept[index].iter().map(|&i| lambdas[i]).collect();
            let mut rows: Vec<NurbsCurve> = Vec::with_capacity(times.len());
            for points in &sampled[index] {
                let row_points: Vec<Vec3> = kept[index].iter().map(|&i| points[i]).collect();
                rows.push(frag(crate::interpolate_curve(&row_points, 5, &row_lambdas), "loft_row")?);
            }
            let knots_u = rows[0].knots.clone();
            let mut net: Vec<Vec<Vec4>> = Vec::with_capacity(rows[0].control_points.len());
            let mut knots_v = Vec::new();
            for k in 0..rows[0].control_points.len() {
                let column: Vec<Vec3> = rows.iter().map(|row| row.control_points[k].point()).collect::<Result<_, _>>().map_err(|e| refuse("loft", e))?;
                let across = frag(crate::interpolate_curve(&column, 5, &fractions), "loft_column")?;
                knots_v = across.knots.clone();
                net.push(across.control_points.clone());
            }
            let surface = frag(NurbsSurface::new(5, 5, knots_u, knots_v, net), "loft_wall")?;
            walls.push((c, back_forward, surface, rows.first().cloned().expect("rows"), rows.last().cloned().expect("rows")));
        }
        if between > target {
            return Err(refuse("loft_rows", format!("the lofted wall's rows stay about {between:e} off the grazing loop between its samples")));
        }
        // Against the exact loop in the middle of each span, every sample.
        let mut misses = vec![0.0f64; count];
        for j in 0..count {
            let t = 0.5 * (times[j] + times[j + 1]);
            let fraction = 0.5 * (fractions[j] + fractions[j + 1]);
            let split = if lean { with_pcurves_skipped(|| split_at(t))? } else { split_at(t)? };
            if signature(&split)? != shape {
                return Err(refuse("loft_structure", "the grazing loop changes its structure along the curve".into()));
            }
            for (c, back_forward, surface, _, _) in &walls {
                let exact = samples_of(&split, *c, *back_forward, t)?;
                for (i, x) in exact.iter().enumerate() {
                    misses[j] = misses[j].max(frag(surface.evaluate(lambdas[i], fraction), "loft_check")?.sub(*x).length());
                }
            }
        }
        let worst = misses.iter().copied().fold(0.0f64, f64::max);
        if worst > target {
            if count > 2000 {
                return Err(refuse("loft_accuracy", format!("the lofted wall stays {worst:e} off the grazing loop")));
            }
            // A span missing by more than half the target is cut into
            // ceil((e / (target / 4))^(1/6)) pieces; the rest stay.
            let mut refined = Vec::with_capacity(2 * fractions.len());
            for j in 0..count {
                let pieces = if misses[j] > 0.5 * target { ((misses[j] / (0.25 * target)).powf(1.0 / 6.0).ceil() as usize).max(2) } else { 1 };
                for k in 0..pieces {
                    refined.push(fractions[j] + (fractions[j + 1] - fractions[j]) * k as f64 / pieces as f64);
                }
            }
            refined.push(1.0);
            fractions = refined;
            continue;
        }
        // Each K corner's path through the sample times.
        let mut paths: HashMap<usize, Vec<Vec3>> = HashMap::new();
        for &(c, _) in &grazing {
            for r in [splits[0].curves[c].start, splits[0].curves[c].end] {
                paths.entry(r).or_insert_with(|| splits.iter().zip(&times).map(|(split, &t)| screw.apply(t, split.points[r])).collect());
            }
        }
        return assemble_lofted(screw, &splits[0], &splits[count], &walls, &fractions, &paths, target);
    }
}

/// Refuse a lofted wall with an edge of regression: across one the wall's
/// normal turns over, and the band beside it lies inside nearby poses (the
/// sweep folds within a few degrees of the motion, which no probe walk is
/// dense enough to be sure of seeing). Normals are compared between
/// neighbours on every time row and between rows, outside what the end trims
/// take away (`removed`, chart rings) and a few columns off its edge (a fold
/// line meets the end at the cut's own ends). `fit`: how near the wall keeps
/// to its exact loop (a length).
fn wall_regular(surface: &NurbsSurface, fractions: &[f64], removed: &[&[[f64; 2]]], fit: f64) -> Result<(), KernelRefusal> {
    const COLUMNS: usize = 4 * LOFT_SAMPLES;
    let mut rows: Vec<f64> = Vec::with_capacity(2 * fractions.len());
    for j in 0..fractions.len() {
        rows.push(fractions[j]);
        if j + 1 < fractions.len() {
            rows.push(0.5 * (fractions[j] + fractions[j + 1]));
        }
    }
    let normal = |u: f64, v: f64| -> Result<Option<Vec3>, KernelRefusal> {
        let h = 1e-6;
        let (ua, ub, va, vb) = ((u - h).max(0.0), (u + h).min(1.0), (v - h).max(0.0), (v + h).min(1.0));
        let du = frag(surface.evaluate(ub, v), "regular")?.sub(frag(surface.evaluate(ua, v), "regular")?).scale(1.0 / (ub - ua));
        let dv = frag(surface.evaluate(u, vb), "regular")?.sub(frag(surface.evaluate(u, va), "regular")?).scale(1.0 / (vb - va));
        // Where the wall's two directions all but line up, or it is no
        // wider than its own fit's ripple (a corner it pinches to), it has
        // no normal to speak of: the fit's noise would turn it.
        let cross = du.cross(dv);
        if cross.length() <= 1e-6 * du.length() * dv.length() || du.length().min(dv.length()) <= 1e3 * fit {
            return Ok(None);
        }
        Ok(cross.normalized().ok())
    };
    // Each trimmed-away ring's box, grown by the reach of `near_ring` and a hair: a point
    // outside it is neither inside the ring (a closed polygon) nor that near it, so the box
    // answers for it as the ring's own tests would.
    let reach = 4.0 / COLUMNS as f64;
    let boxes: Vec<[f64; 4]> = removed
        .iter()
        .map(|ring| {
            let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
            for p in ring.iter() {
                lo = [lo[0].min(p[0]), lo[1].min(p[1])];
                hi = [hi[0].max(p[0]), hi[1].max(p[1])];
            }
            let pad = reach * (1.0 + 1e-9) + 1e-12;
            [lo[0] - pad, hi[0] + pad, lo[1] - pad, hi[1] + pad]
        })
        .collect();
    let mut previous: Option<Vec<Option<Vec3>>> = None;
    for &v in &rows {
        // What a trim takes away is not part of the body.
        let row: Vec<Option<Vec3>> = (0..=COLUMNS)
            .map(|i| {
                let u = i as f64 / COLUMNS as f64;
                let taken = removed.iter().zip(&boxes).any(|(ring, b)| {
                    !(u < b[0] || u > b[1] || v < b[2] || v > b[3]) && (inside_ring(ring, [u, v]) || near_ring(ring, [u, v], reach))
                });
                if taken { Ok(None) } else { normal(u, v) }
            })
            .collect::<Result<_, _>>()?;
        for i in 0..COLUMNS {
            if let (Some(a), Some(b)) = (row[i], row[i + 1]) {
                if a.dot(b) < -0.5 {
                    return Err(refuse("wall_folds", format!("a lofted wall turns over (an edge of regression near parameter ({:.4}, {v:.4})); the sweep folds onto itself", i as f64 / COLUMNS as f64)));
                }
            }
            if let Some(Some(Some(a))) = previous.as_ref().map(|p| p.get(i).copied()) {
                if let Some(b) = row[i] {
                    if a.dot(b) < -0.5 {
                        return Err(refuse("wall_folds", format!("a lofted wall turns over (an edge of regression near parameter ({:.4}, {v:.4})); the sweep folds onto itself", i as f64 / COLUMNS as f64)));
                    }
                }
            }
        }
        previous = Some(row);
    }
    Ok(())
}

/// A stretch of a wall's row at v, from u = a to u = b, as an edge's curve.
fn row_piece(wall: &NurbsSurface, v: f64, a: f64, b: f64) -> Result<NurbsCurve, KernelRefusal> {
    let ts: Vec<f64> = (0..=128).map(|j| j as f64 / 128.0).collect();
    let along: Vec<Vec3> = ts.iter().map(|&s| frag(wall.evaluate((a + (b - a) * s).clamp(0.0, 1.0), v), "wall")).collect::<Result<_, _>>()?;
    frag(crate::interpolate_curve(&along, 5, &ts), "loop_piece")
}

/// A wall's end row in pieces around its trims and breaks (one more than the
/// spans), at v.
fn row_pieces(wall: &NurbsSurface, v: f64, spans: &[TrimSpan]) -> Result<Vec<NurbsCurve>, KernelRefusal> {
    let mut pieces = Vec::with_capacity(spans.len() + 1);
    let mut from = 0.0;
    for span in spans {
        pieces.push(row_piece(wall, v, from, span.lambdas[0])?);
        from = span.lambdas[1];
    }
    pieces.push(row_piece(wall, v, from, 1.0)?);
    Ok(pieces)
}

/// The wall over u ∈ [a, b] only, in the same parameters.
/// Can `pcurve` (in a wall's chart) meet the split segment u = tip.u, v in
/// [v_lo, v_hi], anywhere but at the tip itself? A pcurve that ENDS at the
/// tip (a side of the double curve) touches it there: a short stub at that end
/// must leave the tip into one side of the line or away from the segment, and
/// the rest is tested. Every other piece is decided on the convex hull of its
/// control points (positive weights), halved where a hull straddles the
/// segment; one still straddling after 24 halvings counts as meeting it.
fn hull_meets_segment(pcurve: &NurbsCurve, tip: &[f64; 2], v_lo: f64, v_hi: f64) -> Result<bool, KernelRefusal> {
    let uv = |p: &Vec4| [p.x / p.w, p.y / p.w];
    let near_tip = |p: [f64; 2]| (p[0] - tip[0]).abs() <= 1e-9 && (p[1] - tip[1]).abs() <= 1e-9;
    let (first, last) = (uv(&pcurve.control_points[0]), uv(&pcurve.control_points[pcurve.control_points.len() - 1]));
    if !(near_tip(first) || near_tip(last)) {
        return hull_straddles(pcurve, tip, v_lo, v_hi, 0);
    }
    if near_tip(first) && near_tip(last) {
        return Ok(true);
    }
    let [d0, d1] = frag(pcurve.domain(), "domain")?;
    let at = if near_tip(first) { d0 + 1e-3 * (d1 - d0) } else { d1 - 1e-3 * (d1 - d0) };
    let (a, b) = frag(pcurve.split(at), "split")?;
    let (stub, rest) = if near_tip(first) { (a, b) } else { (b, a) };
    let side: Vec<f64> = stub.control_points.iter().map(|p| uv(p)[0] - tip[0]).collect();
    let one_side = side.iter().all(|d| *d >= -1e-12) || side.iter().all(|d| *d <= 1e-12);
    // The segment runs from the tip towards the far row: away is the other way.
    let segment_below = v_lo < tip[1] - 1e-12;
    let away = stub.control_points.iter().all(|p| if segment_below { uv(p)[1] >= tip[1] - 1e-9 } else { uv(p)[1] <= tip[1] + 1e-9 });
    if !(one_side || away) {
        return Ok(true);
    }
    hull_straddles(&rest, tip, v_lo, v_hi, 0)
}

fn hull_straddles(pcurve: &NurbsCurve, tip: &[f64; 2], v_lo: f64, v_hi: f64, depth: usize) -> Result<bool, KernelRefusal> {
    let points: Vec<[f64; 2]> = pcurve.control_points.iter().map(|p| [p.x / p.w, p.y / p.w]).collect();
    let (u_min, u_max) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[0]), hi.max(p[0])));
    let (w_min, w_max) = points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p[1]), hi.max(p[1])));
    if u_min > tip[0] + 1e-12 || u_max < tip[0] - 1e-12 || w_min > v_hi + 1e-12 || w_max < v_lo - 1e-12 {
        return Ok(false);
    }
    if depth >= 24 {
        return Ok(true);
    }
    let [d0, d1] = frag(pcurve.domain(), "domain")?;
    match pcurve.split(0.5 * (d0 + d1)) {
        Ok((a, b)) => Ok(hull_straddles(&a, tip, v_lo, v_hi, depth + 1)? || hull_straddles(&b, tip, v_lo, v_hi, depth + 1)?),
        // Too short to halve: count it as meeting the segment.
        Err(_) => Ok(true),
    }
}

/// `BREP_SWEEP_SPLIT_CROSSING` (default on; `=0` off): a lofted wall that
/// turns over near one end is split at its swallowtail's point whenever the
/// split segment to the far row crosses no trim curve, not only where the
/// trimmed row keeps to its side of the tip's u; otherwise the wall stays one
/// face carrying its double curve as a slot (one edge used twice), which
/// meshes poorly at the slot's tip.
fn split_crossing_on() -> bool {
    crate::switch!("BREP_SWEEP_SPLIT_CROSSING", true)
}

fn restrict_u(surface: &NurbsSurface, a: f64, b: f64) -> Result<NurbsSurface, KernelRefusal> {
    let [u0, u1] = frag(surface.domain_u(), "domain")?;
    if a <= u0 + 1e-12 && b >= u1 - 1e-12 {
        return Ok(surface.clone());
    }
    let columns = surface.control_points[0].len();
    let mut knots = Vec::new();
    let mut net: Vec<Vec<Vec4>> = Vec::new();
    for j in 0..columns {
        let line: Vec<Vec4> = surface.control_points.iter().map(|row| row[j]).collect();
        let mut curve = frag(NurbsCurve::new(surface.degree_u, surface.knots_u.clone(), line), "restrict")?;
        if b < u1 - 1e-12 {
            curve = frag(curve.split(b), "restrict")?.0;
        }
        if a > u0 + 1e-12 {
            curve = frag(curve.split(a), "restrict")?.1;
        }
        if j == 0 {
            knots = curve.knots.clone();
            net = vec![Vec::with_capacity(columns); curve.control_points.len()];
        }
        for (i, point) in curve.control_points.iter().enumerate() {
            net[i].push(*point);
        }
    }
    frag(NurbsSurface::new(surface.degree_u, surface.degree_v, knots, surface.knots_v.clone(), net), "restrict")
}

/// One place a wall is cut back where an end pose bulges past it: the wall
/// parameter range [λa, λb] along the grazing loop where the trim leaves and
/// rejoins it, the cut (from the point at λa to the point at λb, on both the
/// wall and the pose), the wall's side of the trim from λa to λb (`chain`),
/// the double curve and its swallowtail point if the trim follows one, and
/// the part of the wall's chart it takes away (`removed`, (u, v) points
/// around it).
struct TrimSpan {
    lambdas: [f64; 2],
    points: [Vec3; 2],
    /// None: no cut at all, only a break in the end's row at λa = λb (where
    /// a line splitting the wall lands on it).
    cut: Option<NurbsCurve>,
    chain: Vec<(TrimPiece, bool, NurbsCurve)>,
    slot: Option<(NurbsCurve, Vec3)>,
    /// The swallowtail's point in the wall's chart (u, v), where the trim
    /// follows a double curve.
    tip: Option<[f64; 2]>,
    removed: Vec<[f64; 2]>,
}

/// Every place an end pose bulges past one wall, in order along the loop, and
/// the loop's pieces around them (one more than the spans).
struct EndTrim {
    spans: Vec<TrimSpan>,
    pieces: Vec<NurbsCurve>,
}

/// Which edge a piece of a trimmed wall's end runs on: the cut it shares with
/// the cap, or the wall's own double curve (both of the slot's sides).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum TrimPiece {
    Cut,
    Slot,
}

/// The cut of a lofted wall (u along the grazing loop, v the motion's time
/// fraction) by the pose at its end (`end` 0: v = 0, 1: v = 1), if the pose
/// bulges past it, else None.
///
/// The wall meets the pose along K and leaves it to second order (K grazes),
/// so with ε the time fraction from the end, f = d / ε² (d the pose's signed
/// distance) is smooth up to the end. The part of the wall inside the pose is
/// where f < 0; its boundary is traced as the curve f = 0 from the end and back
/// to it. It need not be a graph over u: a tongue of it may run on past where
/// it left the end.
///
/// `fit`: how near the wall keeps to the exact loop (a length).
fn end_trim(screw: &Screw, split: &Split, t: f64, wall: &NurbsSurface, end: usize, fit: f64) -> Result<Option<EndTrim>, KernelRefusal> {
    let v_of = |e: f64| if end == 1 { 1.0 - e } else { e };
    let at = |u: f64, e: f64| -> Result<Vec3, KernelRefusal> { frag(wall.evaluate(u.clamp(0.0, 1.0), v_of(e)), "wall") };
    let f = |u: f64, e: f64| -> Result<f64, KernelRefusal> { Ok(split.distance(screw.invert(t, at(u, e)?)) / (e * e)) };
    // Below `floor` the walls' fit swamps d / ε².
    let floor = 1e-3;
    // How far into the motion (as a fraction of it) a trim may reach (a fold
    // band can run most of a span); past that the end pose meets a distant
    // part of the sweep (`end_trim_deep`).
    let window = 0.6;
    let noise = 2.0 * fit / (floor * floor);
    // Where the wall dips into the pose just off the end.
    let n = 2000;
    let edge: Vec<f64> = (0..=n).map(|i| f(i as f64 / n as f64, floor)).collect::<Result<_, _>>()?;
    // Inside: the stretches where f is negative, those that dip below the
    // fit's noise somewhere (a stretch that never does is the fit's own).
    let mut inside: Vec<bool> = edge.iter().map(|&x| x < 0.0).collect();
    let mut i = 0;
    while i <= n {
        if !inside[i] {
            i += 1;
            continue;
        }
        let mut j = i;
        while j < n && inside[j + 1] {
            j += 1;
        }
        if edge[i..=j].iter().all(|&x| x >= -noise) {
            for flag in &mut inside[i..=j] {
                *flag = false;
            }
        }
        i = j + 1;
    }
    if !inside.iter().any(|&x| x) {
        // Nothing near the end; a pocket deeper in is caught below.
        return pierced(&f, floor, window, noise, &[]);
    }
    if inside[0] || inside[n] || inside[1] || inside[n - 1] {
        return Err(refuse("end_trim_corner", "an end pose bulges past a wall across a corner of its grazing loop".into()));
    }
    // Crossings of the floor line, by bisection between samples of either sign.
    let mut crossings: Vec<f64> = Vec::new();
    for i in 0..n {
        if inside[i] != inside[i + 1] {
            let (mut a, mut b) = (i as f64 / n as f64, (i + 1) as f64 / n as f64);
            let a_inside = inside[i];
            for _ in 0..60 {
                let mid = 0.5 * (a + b);
                if (f(mid, floor)? < 0.0) == a_inside { a = mid } else { b = mid }
            }
            crossings.push(0.5 * (a + b));
        }
    }
    if crossings.len() % 2 != 0 || crossings.is_empty() {
        return Err(refuse("end_trim", "an end pose's bulges past a wall do not pair up".into()));
    }
    // Trace f = 0 from the first crossing of each pair into the wall until it
    // returns to the floor; it must come back at the pair's other crossing.
    let gradient = |u: f64, e: f64| -> Result<[f64; 2], KernelRefusal> {
        let h = 1e-6;
        Ok([(f(u + h, e)? - f(u - h, e)?) / (2.0 * h), (f(u, e + h)? - f(u, e - h)?) / (2.0 * h)])
    };
    let settle = |mut p: [f64; 2]| -> Result<[f64; 2], KernelRefusal> {
        for _ in 0..30 {
            let value = f(p[0], p[1])?;
            let g = gradient(p[0], p[1])?;
            let norm = g[0] * g[0] + g[1] * g[1];
            if norm == 0.0 {
                return Err(refuse("end_trim", "the cut of an end pose has a flat point".into()));
            }
            p = [p[0] - value * g[0] / norm, p[1] - value * g[1] / norm];
            if value.abs() * p[1] * p[1] < 1e-14 * split.scale {
                break;
            }
        }
        Ok(p)
    };
    let step = 1e-3;
    let mut cuts: Vec<Vec<[f64; 2]>> = Vec::new();
    // Each cut starts at the lowest crossing not yet used and ends at the
    // unused one it comes back to.
    let mut unused: Vec<f64> = crossings.clone();
    while !unused.is_empty() {
        let start = unused.remove(0);
        let mut points: Vec<[f64; 2]> = vec![[start, floor]];
        let mut heading = [0.0, 1.0];
        loop {
            let last = *points.last().expect("points");
            let g = gradient(last[0], last[1])?;
            let length = g[0].hypot(g[1]);
            let mut tangent = [-g[1] / length, g[0] / length];
            if tangent[0] * heading[0] + tangent[1] * heading[1] < 0.0 {
                tangent = [-tangent[0], -tangent[1]];
            }
            let next = settle([last[0] + step * tangent[0], last[1] + step * tangent[1]])?;
            if next[1] < floor {
                break;
            }
            if next[0] <= 1e-3 || next[0] >= 1.0 - 1e-3 {
                return Err(refuse("end_trim_corner", "an end pose bulges past a wall across a corner of its grazing loop".into()));
            }
            if next[1] > window {
                return Err(refuse("end_trim_deep", "an end pose holds a deep part of a wall".into()));
            }
            if points.len() > 20_000 {
                return Err(refuse("end_trim", "the cut of an end pose does not close".into()));
            }
            heading = [next[0] - last[0], next[1] - last[1]];
            points.push(next);
        }
        let back = *points.last().expect("points");
        let Some(at_end) = unused.iter().position(|&c| (back[0] - c).abs() <= 2.0 * step) else {
            return Err(refuse("end_trim", "the cut of an end pose does not come back where it should".into()));
        };
        let back_at = unused.remove(at_end);
        // A dip that never rises off the floor and comes straight back is
        // the fit's noise at the end, not a bulge.
        if points.len() == 1 && (back_at - start).abs() <= step {
            continue;
        }
        points.push([back_at, floor]);
        // Down to the end itself along the cut's direction at the floor (taken
        // from the first point clearly above it).
        let to_end = |p: [f64; 2], rest: &[[f64; 2]], reversed: bool| -> Result<[f64; 2], KernelRefusal> {
            // Along the cut's own tangent there (across the gradient of f), so
            // the run to the end continues it smoothly.
            let g = gradient(p[0], p[1])?;
            if g[0].abs() > 1e-3 * g[1].abs() {
                return Ok([p[0] + g[1] / g[0] * p[1], 0.0]);
            }
            let clear = |q: &&[f64; 2]| q[1] - p[1] > 0.25 * step;
            let found = if reversed { rest.iter().rev().find(clear) } else { rest.iter().find(clear) };
            Ok(match found {
                Some(q) => [p[0] - (q[0] - p[0]) / (q[1] - p[1]) * p[1], 0.0],
                None => [p[0], 0.0],
            })
        };
        let count = points.len();
        let (first, last) = (to_end(points[0], &points[1..], false)?, to_end(points[count - 1], &points[..count - 1], true)?);
        if [first[0], last[0]].iter().any(|&u| u <= 1e-3 || u >= 1.0 - 1e-3) {
            return Err(refuse("end_trim_corner", "an end pose bulges past a wall across a corner of its grazing loop".into()));
        }
        points.insert(0, first);
        points.push(last);
        if points[0][0] > points[points.len() - 1][0] {
            points.reverse();
        }
        cuts.push(points);
    }
    let rings: Vec<&[[f64; 2]]> = cuts.iter().map(|c| c.as_slice()).collect();
    pierced(&f, floor, window, noise, &rings)?;
    let chart = |p: [f64; 2]| [p[0], v_of(p[1])];
    // A curve on the wall through chart points and its pcurve, both on the
    // given parameters (normalized to [0, 1]); chart-length parameters by
    // default (the wall's own speed collapses near an edge of regression, so
    // 3D chord lengths would bunch them).
    let chart_lengths = |chart_points: &[[f64; 2]]| -> Vec<f64> {
        let mut params = vec![0.0];
        for k in 1..chart_points.len() {
            let gap = (chart_points[k][0] - chart_points[k - 1][0]).hypot(chart_points[k][1] - chart_points[k - 1][1]);
            params.push(params[k - 1] + gap.max(1e-15));
        }
        params
    };
    // Returns the curve, its pcurve, and how far the pcurve's image strays
    // from the curve between the points.
    let curve_through = |chart_points: &[[f64; 2]], params: &[f64], what: &'static str| -> Result<(NurbsCurve, NurbsCurve, f64), KernelRefusal> {
        let total = *params.last().expect("params");
        let params: Vec<f64> = params.iter().map(|x| x / total).collect();
        let world: Vec<Vec3> = chart_points.iter().map(|p| at(p[0], p[1])).collect::<Result<_, _>>()?;
        let uv: Vec<Vec3> = chart_points.iter().map(|p| Vec3::new(p[0], v_of(p[1]), 0.0)).collect();
        let (curve, pcurve) = (frag(crate::interpolate_curve(&world, 5, &params), what)?, frag(crate::interpolate_curve(&uv, 5, &params), what)?);
        // Between the points too: the pcurve's image stays on the curve.
        let mut worst = 0.0f64;
        for k in 0..params.len() - 1 {
            let s = 0.5 * (params[k] + params[k + 1]);
            let q = frag(pcurve.evaluate(s), "pcurve")?;
            let e = if end == 1 { 1.0 - q.y } else { q.y };
            worst = worst.max(at(q.x, e)?.sub(frag(curve.evaluate(s), "curve")?).length());
        }
        Ok((curve, pcurve, worst))
    };
    // A trim curve is refined until its pcurve's image is within `fine` of
    // it, and taken as near as it gets if within `loose` (a double curve has
    // a cusp at its swallowtail's point, where no finer trace helps; a
    // tighter bar costs a thin tool many rounds and buys no reader anything).
    let (fine, loose) = (1e-7 * split.scale, 1e-7 * split.scale);
    let missed = |what: &str| refuse("end_trim", format!("the {what}'s pcurve leaves it between its points however finely it is traced"));
    // The cut, refined where its fit misses: a settled point between each
    // pair of traced ones (on the stretches run straight to the end, their
    // middles).
    let cut_through = |mut chart_points: Vec<[f64; 2]>| -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
        let mut best: Option<(NurbsCurve, NurbsCurve, f64)> = None;
        for _ in 0..7 {
            let fitted = curve_through(&chart_points, &chart_lengths(&chart_points), "cut")?;
            let done = fitted.2 <= fine;
            if best.as_ref().map_or(true, |b| fitted.2 < b.2) {
                best = Some(fitted);
            }
            if done {
                break;
            }
            let mut finer = Vec::with_capacity(2 * chart_points.len());
            for k in 0..chart_points.len() - 1 {
                finer.push(chart_points[k]);
                let (p, q) = (chart_points[k], chart_points[k + 1]);
                let middle = [0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])];
                // Below the floor the cut runs straight to the end: split it there as a line.
                finer.push(if p[1] >= floor && q[1] >= floor { settle(middle)? } else { middle });
            }
            finer.push(chart_points[chart_points.len() - 1]);
            chart_points = finer;
        }
        match best {
            Some((curve, pcurve, worst)) if worst <= loose => Ok((curve, pcurve)),
            _ => Err(missed("cut")),
        }
    };
    let piece = |a: f64, b: f64| -> Result<NurbsCurve, KernelRefusal> {
        let ts: Vec<f64> = (0..=128).map(|j| j as f64 / 128.0).collect();
        let along: Vec<Vec3> = ts.iter().map(|&s| at(a + (b - a) * s, 0.0)).collect::<Result<_, _>>()?;
        frag(crate::interpolate_curve(&along, 5, &ts), "loop_piece")
    };
    // Where the wall turns over near the end (a swallowtail), the cut passes
    // through a point of the end's own loop K a second time, on the wall's far
    // sheet: there the wall's double curve starts, and the trim follows it to
    // the swallowtail's point and back instead of the cut's first stretch.
    let span_of = |points: &Vec<[f64; 2]>| -> Result<TrimSpan, KernelRefusal> {
        if let Some((k, ua, b)) = double_start(&at, points, floor)? {
            // The double curve, traced more finely while its fit misses.
            let mut traced: Option<(Vec<[f64; 2]>, Vec<[f64; 2]>, NurbsCurve, NurbsCurve, NurbsCurve, f64)> = None;
            for round in 0..5 {
                let (a_side, b_side) = double_curve(&at, [ua, 0.0], b, window, 2e-3 / f64::from(1u32 << round))?;
                // Both sides on the continuation's own arc length, so the double
                // curve and its two pcurves share one parameter.
                let mut sigma = vec![0.0];
                for k in 1..a_side.len() {
                    let step = (a_side[k][0] - a_side[k - 1][0]).hypot(a_side[k][1] - a_side[k - 1][1]).hypot((b_side[k][0] - b_side[k - 1][0]).hypot(b_side[k][1] - b_side[k - 1][1]));
                    sigma.push(sigma[k - 1] + step.max(1e-15));
                }
                let ((slot, pa, miss_a), (_, pb, miss_b)) = (curve_through(&a_side, &sigma, "double_curve")?, curve_through(&b_side, &sigma, "double_curve")?);
                let worst = miss_a.max(miss_b);
                if traced.as_ref().map_or(true, |t| worst < t.5) {
                    traced = Some((a_side, b_side, slot, pa, pb, worst));
                }
                if worst <= fine {
                    break;
                }
            }
            let Some((a_side, b_side, slot, pa, pb, _)) = traced.filter(|t| t.5 <= loose) else { return Err(missed("double curve")) };
            // The cut keeps the stretch from the far sheet's point to its end away
            // from where the double curve leaves K.
            let low = ua < points[0][0];
            if !low && ua <= points[points.len() - 1][0] {
                return Err(refuse("end_trim", "the double curve of a wall leaves its end inside the cut".into()));
            }
            let mut kept: Vec<[f64; 2]> = if low { points[k..].to_vec() } else { points[..=k].to_vec() };
            if low { kept[0] = b } else { *kept.last_mut().expect("kept") = b }
            let (cut, cut_pcurve) = cut_through(kept.clone())?;
            let tip = at(a_side[a_side.len() - 1][0], a_side[a_side.len() - 1][1])?;
            // The wall's side of the trim from the low end to the high one.
            let chain = if low {
                vec![(TrimPiece::Slot, true, pa), (TrimPiece::Slot, false, frag(pb.reversed(), "reversed")?), (TrimPiece::Cut, true, cut_pcurve)]
            } else {
                vec![(TrimPiece::Cut, true, cut_pcurve), (TrimPiece::Slot, true, pb), (TrimPiece::Slot, false, frag(pa.reversed(), "reversed")?)]
            };
            let lambdas = if low { [ua, points[points.len() - 1][0]] } else { [points[0][0], ua] };
            let mut removed: Vec<[f64; 2]> = Vec::new();
            if low {
                removed.extend(a_side.iter().map(|&p| chart(p)));
                removed.extend(b_side.iter().rev().map(|&p| chart(p)));
                removed.extend(kept.iter().map(|&p| chart(p)));
            } else {
                removed.extend(kept.iter().map(|&p| chart(p)));
                removed.extend(b_side.iter().map(|&p| chart(p)));
                removed.extend(a_side.iter().rev().map(|&p| chart(p)));
            }
            // Everything the end pose holds must be in what goes.
            let dropped: Vec<[f64; 2]> = if low { points[..k].to_vec() } else { points[k + 1..].to_vec() };
            for p in dropped.iter().filter(|p| p[1] > 2.0 * floor && (p[0] - b[0]).hypot(p[1] - b[1]) > 5e-3) {
                if !inside_ring(&removed, chart(*p)) {
                    return Err(refuse("end_trim", "the cut of an end pose leaves the part of the wall its double curve takes away".into()));
                }
            }
            let points3 = [at(lambdas[0], 0.0)?, at(lambdas[1], 0.0)?];
            // The double curve's sides part about its end: one keeps to
            // each side of the swallowtail's point along the loop, so a
            // line through that point splits the wall between them.
            let tip_chart = chart(a_side[a_side.len() - 1]);
            let side = |points: &[[f64; 2]]| points.iter().map(|q| q[0] - tip_chart[0]).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), d| (lo.min(d), hi.max(d)));
            let ((a_lo, a_hi), (b_lo, b_hi)) = (side(&a_side), side(&b_side));
            let parted = (a_lo >= -1e-9 && b_hi <= 1e-9) || (a_hi <= 1e-9 && b_lo >= -1e-9);
            if !parted {
                return Err(refuse("end_trim", "the two sides of a wall's double curve do not keep to either side of its end".into()));
            }
            return Ok(TrimSpan { lambdas, points: points3, cut: Some(cut), chain, slot: Some((slot, tip)), tip: Some(tip_chart), removed });
        }
        let (cut, wall_pcurve) = cut_through(points.clone())?;
        let (la, lb) = (points[0][0], points[points.len() - 1][0]);
        let removed: Vec<[f64; 2]> = points.iter().map(|&p| chart(p)).collect();
        Ok(TrimSpan { lambdas: [la, lb], points: [at(la, 0.0)?, at(lb, 0.0)?], cut: Some(cut), chain: vec![(TrimPiece::Cut, true, wall_pcurve)], slot: None, tip: None, removed })
    };
    if cuts.is_empty() {
        return pierced(&f, floor, window, noise, &[]);
    }
    let mut spans = Vec::with_capacity(cuts.len());
    for points in &cuts {
        spans.push(span_of(points)?);
    }
    spans.sort_by(|a, b| a.lambdas[0].total_cmp(&b.lambdas[0]));
    // A trim wholly inside what another takes away adds nothing (a bulge of
    // the end pose inside a swallowtail's slot).
    let edge_row = v_of(0.0);
    let within = |inner: &TrimSpan, outer: &TrimSpan| -> bool {
        inner.lambdas[0] >= outer.lambdas[0]
            && inner.lambdas[1] <= outer.lambdas[1]
            && inner.removed.iter().filter(|p| (p[1] - edge_row).abs() > floor).all(|&p| inside_ring(&outer.removed, p) || near_ring(&outer.removed, p, 1e-3))
    };
    let mut k = 0;
    while k < spans.len() {
        if (0..spans.len()).any(|j| j != k && within(&spans[k], &spans[j])) {
            spans.remove(k);
        } else {
            k += 1;
        }
    }
    for pair in spans.windows(2) {
        if pair[0].lambdas[1] >= pair[1].lambdas[0] {
            return Err(refuse("end_trim", "an end pose's trims of a wall overlap".into()));
        }
    }
    let mut pieces = Vec::with_capacity(spans.len() + 1);
    let mut from = 0.0;
    for span in &spans {
        pieces.push(piece(from, span.lambdas[0])?);
        from = span.lambdas[1];
    }
    pieces.push(piece(from, 1.0)?);
    Ok(Some(EndTrim { spans, pieces }))
}



/// Is q within `reach` of the closed polygon `ring`'s sides?
fn near_ring(ring: &[[f64; 2]], q: [f64; 2], reach: f64) -> bool {
    (0..ring.len()).any(|k| {
        let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let length = dx * dx + dy * dy;
        let s = if length > 0.0 { (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / length).clamp(0.0, 1.0) } else { 0.0 };
        (q[0] - a[0] - s * dx).hypot(q[1] - a[1] - s * dy) < reach
    })
}

/// Is q inside the closed polygon `ring` (its last point joins its first)?
fn inside_ring(ring: &[[f64; 2]], q: [f64; 2]) -> bool {
    let mut crossings = 0;
    for k in 0..ring.len() {
        let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
        if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

/// The point of an end pose's cut (chart points (u, ε), ε the time fraction
/// from the end) that lies on the end's own loop K a second time, away from
/// the cut's ends: (its index along the cut, the loop parameter u of the point
/// of K it meets, and the refined (u, ε) on the cut), if there is one.
fn double_start(at: &dyn Fn(f64, f64) -> Result<Vec3, KernelRefusal>, points: &[[f64; 2]], floor: f64) -> Result<Option<(usize, f64, [f64; 2])>, KernelRefusal> {
    let n = 4000;
    let row: Vec<Vec3> = (0..=n).map(|i| at(i as f64 / n as f64, 0.0)).collect::<Result<_, _>>()?;
    let scale = row.iter().fold(0.0f64, |m, p| m.max(p.sub(row[0]).length())).max(1e-300);
    let mut best: Option<(f64, usize, usize)> = None;
    for (k, p) in points.iter().enumerate() {
        if p[1] < 4.0 * floor {
            continue;
        }
        let x = at(p[0], p[1])?;
        let (i, d) = row.iter().enumerate().map(|(i, q)| (i, q.sub(x).length())).fold((0, f64::INFINITY), |a, b| if b.1 < a.1 { b } else { a });
        // The cut nears K at its own ends in proportion to ε; the second
        // meeting is where that ratio drops to nothing.
        if best.map_or(true, |b| d / p[1] < b.0 / points[b.1][1]) {
            best = Some((d, k, i));
        }
    }
    let Some((distance, k, i)) = best else { return Ok(None) };
    // Coarse: the cut's points are a few thousandths of the chart apart.
    if distance > 5e-3 * scale {
        return Ok(None);
    }
    // Newton on at(b) = at(u, 0) for (b_u, b_ε, u).
    let mut x = [points[k][0], points[k][1], i as f64 / n as f64];
    let h = 1e-7;
    for _ in 0..40 {
        let g = at(x[0], x[1])?.sub(at(x[2], 0.0)?);
        if g.length() < 1e-13 * scale {
            // Not the trivial meeting of the end row with itself.
            if x[1] < 2.0 * floor || !(0.0..=1.0).contains(&x[0]) || !(0.0..=1.0).contains(&x[2]) {
                return Ok(None);
            }
            return Ok(Some((k, x[2], [x[0], x[1]])));
        }
        let c0 = at(x[0] + h, x[1])?.sub(at(x[0] - h, x[1])?).scale(0.5 / h);
        let c1 = at(x[0], x[1] + h)?.sub(at(x[0], x[1] - h)?).scale(0.5 / h);
        let c2 = at(x[2] + h, 0.0)?.sub(at(x[2] - h, 0.0)?).scale(-0.5 / h);
        let det = c0.dot(c1.cross(c2));
        if det.abs() < 1e-300 {
            break;
        }
        // Cramer's rule for [c0 c1 c2] d = -g.
        let r = g.scale(-1.0);
        let d = [r.dot(c1.cross(c2)) / det, c0.dot(r.cross(c2)) / det, c0.dot(c1.cross(r)) / det];
        x = [x[0] + d[0], x[1] + d[1], x[2] + d[2]];
    }
    if distance < 1e-6 * scale {
        return Err(refuse("end_trim", "an end pose's cut meets its own loop, but not exactly enough to follow".into()));
    }
    Ok(None)
}

/// The double curve of a wall from where it leaves the end (a, on the end's
/// loop) and its partner b on the far sheet, traced by continuation of
/// at(a) = at(b) in (u, ε) for both, to the swallowtail point where the two
/// sides meet. Returns both sides' chart points, ending at the same point.
fn double_curve(at: &dyn Fn(f64, f64) -> Result<Vec3, KernelRefusal>, a: [f64; 2], b: [f64; 2], window: f64, step: f64) -> Result<(Vec<[f64; 2]>, Vec<[f64; 2]>), KernelRefusal> {
    let h = 1e-7;
    let scale = at(0.0, 0.0)?.sub(at(1.0, 0.0)?).length().max(at(0.5, 0.0)?.sub(at(0.0, 0.0)?).length()).max(1e-300);
    let jacobian = |x: [f64; 4]| -> Result<[Vec3; 4], KernelRefusal> {
        Ok([
            at(x[0] + h, x[1])?.sub(at(x[0] - h, x[1])?).scale(0.5 / h),
            at(x[0], x[1] + h)?.sub(at(x[0], (x[1] - h).max(0.0))?).scale(1.0 / (h + x[1].min(h))),
            at(x[2] + h, x[3])?.sub(at(x[2] - h, x[3])?).scale(-0.5 / h),
            at(x[2], x[3] + h)?.sub(at(x[2], (x[3] - h).max(0.0))?).scale(-1.0 / (h + x[3].min(h))),
        ])
    };
    let residual = |x: [f64; 4]| -> Result<Vec3, KernelRefusal> { Ok(at(x[0], x[1])?.sub(at(x[2], x[3])?)) };
    // The null direction of the 3x4 Jacobian: its signed 3x3 minors.
    let null = |j: &[Vec3; 4]| -> [f64; 4] {
        let minor = |p: usize, q: usize, r: usize| j[p].dot(j[q].cross(j[r]));
        let n = [minor(1, 2, 3), -minor(0, 2, 3), minor(0, 1, 3), -minor(0, 1, 2)];
        let length = n.iter().map(|x| x * x).sum::<f64>().sqrt().max(1e-300);
        [n[0] / length, n[1] / length, n[2] / length, n[3] / length]
    };
    // Minimum-norm Newton back onto at(a) = at(b).
    let correct = |mut x: [f64; 4]| -> Result<Option<[f64; 4]>, KernelRefusal> {
        for _ in 0..30 {
            let g = residual(x)?;
            if g.length() < 1e-13 * scale {
                return Ok(Some(x));
            }
            let j = jacobian(x)?;
            // J Jᵀ y = -g, step = Jᵀ y.
            let mut m = [[0.0; 3]; 3];
            let rows = [[j[0].x, j[1].x, j[2].x, j[3].x], [j[0].y, j[1].y, j[2].y, j[3].y], [j[0].z, j[1].z, j[2].z, j[3].z]];
            for p in 0..3 {
                for q in 0..3 {
                    m[p][q] = (0..4).map(|c| rows[p][c] * rows[q][c]).sum();
                }
            }
            let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
            if det.abs() < 1e-300 {
                return Ok(None);
            }
            let r = [-g.x, -g.y, -g.z];
            let solve = |col: usize| -> f64 {
                let mut a = m;
                for p in 0..3 {
                    a[p][col] = r[p];
                }
                (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0])) / det
            };
            let y = [solve(0), solve(1), solve(2)];
            for c in 0..4 {
                x[c] += (0..3).map(|p| rows[p][c] * y[p]).sum::<f64>();
            }
        }
        Ok(None)
    };
    let mut x = [a[0], a[1], b[0], b[1]];
    let mut a_side = vec![a];
    let mut b_side = vec![b];
    let mut heading: Option<[f64; 4]> = None;
    loop {
        let separation = (x[0] - x[2]).hypot(x[1] - x[3]);
        if separation < 3.0 * step && a_side.len() > 2 {
            let tip = [0.5 * (x[0] + x[2]), 0.5 * (x[1] + x[3])];
            a_side.push(tip);
            b_side.push(tip);
            return Ok((a_side, b_side));
        }
        let mut t = null(&jacobian(x)?);
        let reference = heading.unwrap_or([0.0, 1.0, 0.0, 0.0]);
        if (0..4).map(|c| t[c] * reference[c]).sum::<f64>() < 0.0 {
            t = [-t[0], -t[1], -t[2], -t[3]];
        }
        let mut next = None;
        let mut length = step;
        for _ in 0..6 {
            let guess = [x[0] + length * t[0], x[1] + length * t[1], x[2] + length * t[2], x[3] + length * t[3]];
            if let Some(y) = correct(guess)? {
                next = Some(y);
                break;
            }
            length *= 0.5;
        }
        let Some(y) = next else {
            // Near the swallowtail's point the two sheets turn tangent and
            // Newton gives out; close there if the sides are that near.
            if separation < 2e-2 && a_side.len() > 2 {
                let tip = [0.5 * (x[0] + x[2]), 0.5 * (x[1] + x[3])];
                a_side.push(tip);
                b_side.push(tip);
                return Ok((a_side, b_side));
            }
            return Err(refuse("end_trim", format!("the double curve of a wall cannot be followed (sides {separation:e} apart)")));
        };
        if [y[0], y[2]].iter().any(|&u| u <= 1e-3 || u >= 1.0 - 1e-3) {
            return Err(refuse("end_trim_corner", DOUBLE_CURVE_CORNER.into()));
        }
        if y[1] > window || y[3] > window || y[1] < 0.0 || y[3] < 0.0 {
            return Err(refuse("end_trim_deep", format!("the double curve of a wall runs deep into the motion or back past its end (its sides at time fractions {:.4} and {:.4} from the end, {} points in)", y[1], y[3], a_side.len())));
        }
        if a_side.len() > 20_000 {
            return Err(refuse("end_trim", "the double curve of a wall does not close".into()));
        }
        heading = Some([y[0] - x[0], y[1] - x[1], y[2] - x[2], y[3] - x[3]]);
        x = y;
        a_side.push([x[0], x[1]]);
        b_side.push([x[2], x[3]]);
    }
}

/// Refuse a wall the end pose holds a part of that no cut encloses: a pocket
/// of the wall inside the pose away from the end.
fn pierced(
    f: &dyn Fn(f64, f64) -> Result<f64, KernelRefusal>,
    floor: f64,
    window: f64,
    noise: f64,
    cuts: &[&[[f64; 2]]],
) -> Result<Option<EndTrim>, KernelRefusal> {
    let enclosed = |q: [f64; 2]| -> bool { cuts.iter().any(|ring| inside_ring(ring, q)) };
    for i in 1..200 {
        for j in 1..=40 {
            let (u, e) = (i as f64 / 200.0, floor + (window - floor) * (j as f64 / 40.0).powi(2));
            // d itself against the fit's noise: f grows as 1 / ε² near the end.
            if f(u, e)? * e * e < -noise * floor * floor && !enclosed([u, e]) {
                return Err(refuse("end_trim_deep", "an end pose holds a part of a wall away from its end".into()));
            }
        }
    }
    Ok(None)
}

#[allow(clippy::too_many_arguments)]
fn assemble_lofted(
    screw: &Screw,
    first: &Split,
    last: &Split,
    walls: &[(usize, bool, NurbsSurface, NurbsCurve, NurbsCurve)],
    times: &[f64],
    paths: &HashMap<usize, Vec<Vec3>>,
    fit: f64,
) -> Result<BrepSolid, KernelRefusal> {
    let mut shell = Shell::new();
    let ends = [(first, screw.t0), (last, screw.t1)];
    // Where an end pose bulges past a wall near its grazing loop (a saddle side
    // curving past the wall it touches), cut the wall back to where it
    // re-enters the pose and carry the end cap across the loop to there.
    let mut trims: HashMap<(usize, usize), EndTrim> = HashMap::new();
    // Walls split into strips along lines through their swallowtails' points.
    let mut split_walls: std::collections::HashSet<usize> = std::collections::HashSet::new();
    for (index, (_, _, surface, _, _)) in walls.iter().enumerate() {
        for end in 0..2 {
            if let Some(trim) = end_trim(screw, ends[end].0, ends[end].1, surface, end, fit)? {
                trims.insert((index, end), trim);
            }
        }
        let removed: Vec<&[[f64; 2]]> = (0..2).filter_map(|end| trims.get(&(index, end))).flat_map(|trim| trim.spans.iter().map(|span| span.removed.as_slice())).collect();
        wall_regular(surface, times, &removed, fit)?;
        // A trim along a double curve has that curve as one edge on both its
        // sides: on one face its two uses could not be told apart from its
        // 3D curve alone (the wall passes through it twice). The wall is split
        // along the parameter line from each swallowtail's point to the far
        // end's row, which that row is broken at.
        let tips_at = |end: usize| -> Vec<[f64; 2]> { trims.get(&(index, end)).map(|trim| trim.spans.iter().filter_map(|span| span.tip).collect()).unwrap_or_default() };
        let (tips0, tips1) = (tips_at(0), tips_at(1));
        if !tips0.is_empty() && !tips1.is_empty() {
            return Err(refuse("wall_split", "a wall turns over near both its ends".into()));
        }
        let (far, tips) = if tips0.is_empty() { (0usize, tips1) } else { (1usize, tips0) };
        // A straight split line serves only where the trimmed row keeps to
        // its own side of it: all that comes before the swallowtail's point
        // along the row at or below its u, all after at or above. (A cut
        // that bulges across it would be crossed by the line; such a wall
        // stays one face.)
        let straight = |trim: &EndTrim| -> Result<bool, KernelRefusal> {
            for (k, span) in trim.spans.iter().enumerate() {
                let Some(tip) = span.tip else { continue };
                let first_slot = span.chain.iter().position(|piece| piece.0 == TrimPiece::Slot).unwrap_or(0);
                for (j, other) in trim.spans.iter().enumerate() {
                    for (n, (_, _, pcurve)) in other.chain.iter().enumerate() {
                        let before = j < k || (j == k && n <= first_slot);
                        let [d0, d1] = frag(pcurve.domain(), "domain")?;
                        for i in 0..=32 {
                            let u = frag(pcurve.evaluate(d0 + (d1 - d0) * i as f64 / 32.0), "pcurve")?.x;
                            if (before && u > tip[0] + 1e-9) || (!before && u < tip[0] - 1e-9) {
                                return Ok(false);
                            }
                        }
                    }
                }
            }
            Ok(true)
        };
        // Or, where the trimmed row strays across the tip's u only beyond the
        // tip (near its own end), the split segment itself: from the tip to
        // the far row, crossed by none of either end's trim curves. Each strip
        // then lies on the wall over the u its own pcurves span.
        let crossing_free = |trim_near: &EndTrim| -> Result<bool, KernelRefusal> {
            let v_far = far as f64;
            for tip in &tips {
                let (v_lo, v_hi) = if v_far < tip[1] { (v_far, tip[1]) } else { (tip[1], v_far) };
                let ends = [Some(trim_near), trims.get(&(index, far))];
                for trim in ends.into_iter().flatten() {
                    for span in &trim.spans {
                        for (_, _, pcurve) in &span.chain {
                            if hull_meets_segment(pcurve, tip, v_lo, v_hi)? {
                                return Ok(false);
                            }
                        }
                    }
                }
            }
            Ok(true)
        };
        let split_here = !tips.is_empty()
            && match trims.get(&(index, 1 - far)) {
                Some(trim) => straight(trim)? || (split_crossing_on() && crossing_free(trim)?),
                None => false,
            };
        if split_here {
            split_walls.insert(index);
            let v_far = far as f64;
            let mut spans = trims.remove(&(index, far)).map(|trim| trim.spans).unwrap_or_default();
            for tip in &tips {
                if spans.iter().any(|span| tip[0] > span.lambdas[0] - 1e-9 && tip[0] < span.lambdas[1] + 1e-9) || tip[0] <= 1e-9 || tip[0] >= 1.0 - 1e-9 {
                    return Err(refuse("wall_split", "a wall's split line lands in a trim of its far end".into()));
                }
                let point = frag(surface.evaluate(tip[0], v_far), "wall")?;
                spans.push(TrimSpan { lambdas: [tip[0], tip[0]], points: [point, point], cut: None, chain: Vec::new(), slot: None, tip: None, removed: Vec::new() });
            }
            spans.sort_by(|a, b| a.lambdas[0].total_cmp(&b.lambdas[0]));
            let pieces = row_pieces(surface, v_far, &spans)?;
            trims.insert((index, far), EndTrim { spans, pieces });
        }
    }
    let mut vertex_at: HashMap<(usize, usize), u64> = HashMap::new();
    let mut vertex = |shell: &mut Shell, end: usize, p: usize| -> u64 {
        *vertex_at.entry((end, p)).or_insert_with(|| shell.vertex(screw.apply(ends[end].1, ends[end].0.points[p])))
    };
    let grazing: HashMap<usize, usize> = walls.iter().enumerate().map(|(index, w)| (w.0, index)).collect();
    // Edges at each end: K curves from the walls' end rows (in the tool
    // curve's direction), the rest the end split's own curves moved.
    let mut edge_at: HashMap<(usize, usize), u64> = HashMap::new();
    let mut edge_curve: HashMap<(usize, usize), NurbsCurve> = HashMap::new();
    // Per trimmed wall end: the loop's piece edges, and each span's cut and
    // double-curve edges (u64::MAX where a span has none).
    let mut trim_edges: HashMap<(usize, usize), (Vec<u64>, Vec<[u64; 2]>)> = HashMap::new();
    // Per trimmed wall end and span: the vertex at its low end, and at the
    // swallowtail's point where it has one.
    let mut trim_vertices: HashMap<(usize, usize), Vec<(u64, Option<u64>)>> = HashMap::new();
    for end in 0..2 {
        let (split, t) = ends[end];
        for (c, tool_curve) in split.curves.iter().enumerate() {
            let used = split.caps.iter().any(|cap| cap.front == (end == 1) && cap.coedges.iter().any(|e| e.0 == c));
            if !used {
                continue;
            }
            if let Some(&index) = grazing.get(&c) {
                if let Some(trim) = trims.get(&(index, end)) {
                    // Pieces in the wall's direction: K up to the cut, the cut, K after it.
                    let back_forward = walls[index].1;
                    let (from, to) = if back_forward { (tool_curve.end, tool_curve.start) } else { (tool_curve.start, tool_curve.end) };
                    let (vf, vt) = (vertex(&mut shell, end, from), vertex(&mut shell, end, to));
                    let mut piece_edges = Vec::with_capacity(trim.pieces.len());
                    let mut span_edges = Vec::with_capacity(trim.spans.len());
                    let mut start = vf;
                    let mut span_vertices = Vec::with_capacity(trim.spans.len());
                    for (span, piece) in trim.spans.iter().zip(&trim.pieces) {
                        let va = shell.vertex(span.points[0]);
                        piece_edges.push(shell.edge(piece.clone(), start, va, false)?);
                        let Some(cut_curve) = &span.cut else {
                            // A break only: the row goes on from the same vertex.
                            span_edges.push([u64::MAX, u64::MAX]);
                            span_vertices.push((va, None));
                            start = va;
                            continue;
                        };
                        let vb = shell.vertex(span.points[1]);
                        let cut = shell.edge(cut_curve.clone(), va, vb, false)?;
                        // The double curve runs from where it leaves K (at
                        // the low end of the span if its chain starts with
                        // it) to the swallowtail's point.
                        let (slot, tip_vertex) = match &span.slot {
                            Some((curve, tip)) => {
                                let from = if span.chain[0].0 == TrimPiece::Slot { va } else { vb };
                                let tip = shell.vertex(*tip);
                                (shell.edge(curve.clone(), from, tip, false)?, Some(tip))
                            }
                            None => (u64::MAX, None),
                        };
                        span_edges.push([cut, slot]);
                        span_vertices.push((va, tip_vertex));
                        start = vb;
                    }
                    trim_vertices.insert((index, end), span_vertices);
                    piece_edges.push(shell.edge(trim.pieces[trim.pieces.len() - 1].clone(), start, vt, false)?);
                    trim_edges.insert((index, end), (piece_edges, span_edges));
                    continue;
                }
            }
            let curve = match grazing.get(&c) {
                Some(&index) => {
                    let (_, back_forward, _, row0, row1) = &walls[index];
                    let row = if end == 0 { row0 } else { row1 };
                    if *back_forward { frag(row.reversed(), "reversed")? } else { row.clone() }
                }
                None => screw.move_curve(t, &tool_curve.curve)?,
            };
            let (a, b) = (vertex(&mut shell, end, tool_curve.start), vertex(&mut shell, end, tool_curve.end));
            let edge = shell.edge(curve.clone(), a, b, tool_curve.degenerate)?;
            edge_at.insert((end, c), edge);
            edge_curve.insert((end, c), curve);
        }
    }
    for end in 0..2 {
        let (split, t) = ends[end];
        for cap in split.caps.iter().filter(|cap| cap.front == (end == 1)) {
            let surface = screw.move_surface(t, &cap.surface)?;
            let mut coedges = Vec::with_capacity(cap.coedges.len());
            for (c, forward, pcurve) in &cap.coedges {
                if let Some(&index) = grazing.get(c) {
                    if let (Some(trim), Some((piece_edges, span_edges))) = (trims.get(&(index, end)), trim_edges.get(&(index, end))) {
                        let Some(chart) = cap.chart.as_ref() else {
                            return Err(refuse("end_trim", "an end pose bulges past a wall beside a cap without a chart".into()));
                        };
                        // The cap runs the tool's curve forward or back; the
                        // pieces run the wall's way: loop piece, cut, loop
                        // piece, ... loop piece.
                        let along = *forward != walls[index].1;
                        let mut pieces: Vec<(u64, &NurbsCurve)> = Vec::with_capacity(2 * trim.pieces.len());
                        for (k, piece) in trim.pieces.iter().enumerate() {
                            pieces.push((piece_edges[k], piece));
                            if let Some(cut) = trim.spans.get(k).and_then(|span| span.cut.as_ref()) {
                                pieces.push((span_edges[k][0], cut));
                            }
                        }
                        if !along {
                            pieces.reverse();
                        }
                        for (edge, curve) in pieces {
                            coedges.push((edge, along, charted_pcurve(chart, screw, t, curve, along)?));
                        }
                        continue;
                    }
                }
                let pcurve = if grazing.contains_key(c) && cap.chart.is_some() {
                    charted_pcurve(cap.chart.as_ref().expect("chart"), screw, t, &edge_curve[&(end, *c)], *forward)?
                } else if grazing.contains_key(c) {
                    match projected_pcurve(&surface, &edge_curve[&(end, *c)], *forward) {
                        Ok(p) => p,
                        Err(error) => {
                            // Which: the exact samples themselves, or the row between them?
                            let samples = split.curves[*c].samples.as_ref().map(|list| {
                                list.iter()
                                    .map(|&x| crate::project_point_to_surface(&surface, screw.apply(t, x)).map(|f| f.distance).unwrap_or(f64::NAN))
                                    .fold(0.0f64, f64::max)
                            });
                            let line = split.curves[*c].curve.degree;
                            return Err(KernelRefusal::unsupported(
                                KernelStage::Classify,
                                "loft_cap",
                                format!("{} (curve {c} of degree {line}, end {end}, its exact samples off the cap by {samples:?}, cap degrees {}x{})", error.message, surface.degree_u, surface.degree_v),
                            ));
                        }
                    }
                } else {
                    pcurve.clone()
                };
                coedges.push((edge_at[&(end, *c)], *forward, pcurve));
            }
            shell.face(surface, coedges);
        }
    }
    // Orbits of K's corners: interpolated through the sample times as the
    // walls' end columns are.
    let mut orbits: HashMap<usize, u64> = HashMap::new();
    let mut keys: Vec<usize> = paths.keys().copied().collect();
    keys.sort_unstable();
    for r in keys {
        let curve = frag(crate::interpolate_curve(&paths[&r], 5, times), "orbit")?;
        let (a, b) = (vertex(&mut shell, 0, r), vertex(&mut shell, 1, r));
        orbits.insert(r, shell.edge(curve, a, b, false)?);
    }
    // One piece of a wall's end row in the direction of u: (edge, traversed
    // forward?, pcurve that way), and where a split line meets the row right
    // after it: (u, v, vertex).
    type RowPiece = (u64, bool, NurbsCurve, Option<(f64, f64, u64)>);
    for (index, (c, back_forward, surface, _, _)) in walls.iter().enumerate() {
        let tool_curve = &first.curves[*c];
        let (from, to) = if *back_forward { (tool_curve.end, tool_curve.start) } else { (tool_curve.start, tool_curve.end) };
        let row = |end: usize| -> Result<Vec<RowPiece>, KernelRefusal> {
            let v = end as f64;
            let (Some(trim), Some((piece_edges, span_edges))) = (trims.get(&(index, end)), trim_edges.get(&(index, end))) else {
                return Ok(vec![(edge_at[&(end, *c)], !*back_forward, uv_line([0.0, v], [1.0, v])?, None)]);
            };
            let vertices = &trim_vertices[&(index, end)];
            let mut list: Vec<RowPiece> = Vec::new();
            let mut at = 0.0;
            for (k, span) in trim.spans.iter().enumerate() {
                let broken = if span.cut.is_none() { Some((span.lambdas[0], v, vertices[k].0)) } else { None };
                list.push((piece_edges[k], true, uv_line([at, v], [span.lambdas[0], v])?, broken));
                for (n, (piece, forward, pcurve)) in span.chain.iter().enumerate() {
                    let edge = if *piece == TrimPiece::Cut { span_edges[k][0] } else { span_edges[k][1] };
                    // The swallowtail's point lies between the double curve's two uses.
                    let between = *piece == TrimPiece::Slot && span.chain.get(n + 1).is_some_and(|next| next.0 == TrimPiece::Slot);
                    let tip = match (span.tip, vertices[k].1) {
                        (Some(tip), Some(vertex)) if between && split_walls.contains(&index) => Some((tip[0], tip[1], vertex)),
                        _ => None,
                    };
                    list.push((edge, *forward, pcurve.clone(), tip));
                }
                at = span.lambdas[1];
            }
            list.push((piece_edges[trim.spans.len()], true, uv_line([at, v], [1.0, v])?, None));
            Ok(list)
        };
        let (bottom, top) = (row(0)?, row(1)?);
        let (low, high): (Vec<(f64, f64, u64)>, Vec<(f64, f64, u64)>) = (bottom.iter().filter_map(|piece| piece.3).collect(), top.iter().filter_map(|piece| piece.3).collect());
        if low.len() != high.len() || low.iter().zip(&high).any(|(a, b)| (a.0 - b.0).abs() > 1e-9) {
            return Err(refuse("wall_split", "a wall's split lines do not run from one end's row to the other's".into()));
        }
        // The split lines, each from its lower point to its upper.
        let mut lines: Vec<(u64, f64, f64, f64)> = Vec::with_capacity(low.len());
        for (below, above) in low.iter().zip(&high) {
            let ts: Vec<f64> = (0..=128).map(|j| j as f64 / 128.0).collect();
            let along: Vec<Vec3> = ts.iter().map(|&s| frag(surface.evaluate(below.0, below.1 + (above.1 - below.1) * s), "wall")).collect::<Result<_, _>>()?;
            let edge = shell.edge(frag(crate::interpolate_curve(&along, 5, &ts), "split_line")?, below.2, above.2, false)?;
            lines.push((edge, below.0, below.1, above.1));
        }
        // Each row in the stretches between the split lines.
        let stretches = |list: Vec<RowPiece>| -> Vec<Vec<(u64, bool, NurbsCurve)>> {
            let mut out = vec![Vec::new()];
            for (edge, forward, pcurve, cut) in list {
                out.last_mut().expect("stretch").push((edge, forward, pcurve));
                if cut.is_some() {
                    out.push(Vec::new());
                }
            }
            out
        };
        let (bottoms, tops) = (stretches(bottom), stretches(top));
        // A strip between two split lines (or the wall's sides): along the
        // start row, up its far side, back along the end row, down its near
        // side; on the wall over its own stretch of u only.
        for j in 0..=lines.len() {
            let mut coedges = bottoms[j].clone();
            match lines.get(j) {
                Some(&(edge, u, v0, v1)) => coedges.push((edge, true, uv_line([u, v0], [u, v1])?)),
                None => coedges.push((orbits[&to], true, uv_line([1.0, 0.0], [1.0, 1.0])?)),
            }
            for (edge, forward, pcurve) in tops[j].iter().rev() {
                coedges.push((*edge, !*forward, frag(pcurve.reversed(), "reversed")?));
            }
            match j.checked_sub(1).map(|k| lines[k]) {
                Some((edge, u, v0, v1)) => coedges.push((edge, false, uv_line([u, v1], [u, v0])?)),
                None => coedges.push((orbits[&from], false, uv_line([0.0, 1.0], [0.0, 0.0])?)),
            }
            let (mut ua, mut ub) = (if j == 0 { 0.0 } else { lines[j - 1].1 }, lines.get(j).map_or(1.0, |line| line.1));
            // A strip whose trim strays past its split line (beyond the tip)
            // takes the wall over that u too: the parameters are the wall's.
            if lines.len() > 0 && split_crossing_on() {
                // Its control points bound each pcurve (positive weights).
                for (_, _, pcurve) in &coedges {
                    for point in &pcurve.control_points {
                        let u = point.x / point.w;
                        if u < ua - 1e-9 {
                            ua = u.max(0.0);
                        }
                        if u > ub + 1e-9 {
                            ub = u.min(1.0);
                        }
                    }
                }
            }
            shell.face(restrict_u(surface, ua, ub)?, coedges);
        }
    }
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        // Name the faces whose coedges stray: their pcurves' ends on the surface
        // against their edges' ends.
        let mut notes = Vec::new();
        for face in body.shells.iter().flat_map(|s| &s.faces) {
            for coedge in &face.loops[0].coedges {
                let edge = body.edges.iter().find(|e| e.id == coedge.edge_id).expect("edge");
                let [p0, p1] = frag(coedge.pcurve.domain(), "domain")?;
                let a = frag(coedge.pcurve.evaluate(p0), "uv")?;
                let on = frag(face.surface.evaluate_extended(a.x, a.y), "on")?;
                let at = frag(edge.curve.evaluate(if coedge.forward { edge.t0 } else { edge.t1 }), "at")?;
                if on.sub(at).length() > 1e-6 {
                    notes.push(format!("face {} (degrees {}x{}) coedge on edge {} starts {:?} on the surface, its edge {:?}", face.id, face.surface.degree_u, face.surface.degree_v, edge.id, on, at));
                }
                let _ = p1;
            }
        }
        return Err(refuse("assembly", format!("the lofted sweep is not valid: {:?}; {}", issues.first(), notes.iter().take(4).cloned().collect::<Vec<_>>().join("; "))));
    }
    check_single_pass(screw, first, &body, &std::collections::HashSet::new())?;
    Ok(body)
}

/// Is the world point y inside the moving tool at some time of the motion?
/// Steps t by the distance over a bound on the speed of any tool point near y.
fn covered(screw: &Screw, split: &Split, reach: Option<&ToolReach>, y: Vec3, normal: Vec3, margin: f64) -> Result<bool, KernelRefusal> {
    // A ball carried by a frame along a cubic: the question read off the cubic
    // (`BREP_SWEEP_BALL_COVER`, default on; its check mode walks as well).
    if crate::solid_sweep_one_pass::ball_cover_on() {
        if let (Some(frame), None, Shape::Revolved { origin, axis, meridian }) = (&screw.frame, &screw.turns, &split.shape) {
            if let Some((centre, r)) = crate::solid_sweep_one_pass::ball_of_revolved(*origin, *axis, meridian) {
                let offset = centre.sub(screw.a).length();
                if let Some(answer) = crate::solid_sweep_one_pass::ball_on_cubic_covered(frame.control, screw.t0, screw.t1, offset, r, y, margin) {
                    if !crate::solid_sweep_one_pass::ball_cover_check_on() {
                        return Ok(answer);
                    }
                    let walked = covered_walk(screw, split, reach, y, normal, margin)?;
                    crate::solid_sweep_one_pass::ball_cover_check_record(walked == answer);
                    return Ok(walked);
                }
            }
        }
    }
    covered_walk(screw, split, reach, y, normal, margin)
}

/// [`covered`] by walking the motion.
fn covered_walk(screw: &Screw, split: &Split, reach: Option<&ToolReach>, y: Vec3, normal: Vec3, margin: f64) -> Result<bool, KernelRefusal> {
    if reach.is_some_and(|reach| reach.never_inside(screw, y, normal, margin)) {
        return Ok(false);
    }
    if let Some(found) = reach.and_then(|reach| reach.turn_about_parallel_axis(screw, y, margin)) {
        return found;
    }
    let speed = preimage_speed_bound(screw, split, y);
    let floor = (screw.t1 - screw.t0) * 1e-7;
    let glide = Glide::of(screw, y);
    let mut t = screw.t0;
    let mut steps = 0usize;
    loop {
        let p = screw.invert(t, y);
        let d = split.distance(p);
        if d < -margin {
            return Ok(true);
        }
        if t >= screw.t1 {
            return Ok(false);
        }
        if steps > 2_000_000 {
            return Err(refuse("probe_budget", "the one-pass check could not walk the motion past a point near the sweep".into()));
        }
        let first = (d.abs() + margin) / speed;
        let second = glide.as_ref().map_or(0.0, |glide| glide.step(split, p, t, d, margin, speed, screw.t1 - t));
        let second = if second.is_finite() { second } else { 0.0 };
        t = (t + first.max(second).max(floor)).min(screw.t1);
        steps += 1;
    }
}

/// The preimage of a probe under a plain screw, as an explicit curve: with
/// `u` the probe's offset from the axis square to it and `v = e × u`,
/// `p(t) = a + (e·(y − a) − advance(t))·e + cos(ωt)·u − sin(ωt)·v`. Its
/// second derivative is no longer than `ω²·|u| + 2|c₂|` (`curve`).
///
/// The walk in time steps by the tool's distance over the preimage's speed:
/// beside a wall the tool grazes (a thread's flank, a ball's groove) the
/// distance stays the probe's tiny offset through the whole contact and the
/// walk crawls, some π·√(2R/δ) steps a probe. Where the path runs along the
/// tool rather than into it, its tangent line stays clear of the tool for
/// far longer, and the path leaves that line by no more than `curve·h²/2`:
/// the second-order step is the longest `h` the tool's distance from the
/// tangent segment, less that, keeps above −margin. It is never shorter than
/// the first-order step, and skips no time the probe is deeper than the margin.
struct Glide {
    e: Vec3,
    u: Vec3,
    v: Vec3,
    omega: f64,
    c: f64,
    c2: f64,
    curve: f64,
    /// Under `frame_glide_on`, a frame riding a curve: the probe's preimage
    /// `a + R(−θ(t))(y − P(t))` read through the frame (frame, y).
    riding: Option<(Frame, Vec3)>,
    /// Under `turns_glide_on`, two turns about fixed axes: the probe's preimage under them (turns, y).
    turning: Option<(Turns, Vec3)>,
    /// Under `frame_windows_on`, the bound on |p''| over each of equal windows
    /// of the motion (start, width, bounds), each no looser than `curve`.
    windows: Option<(f64, f64, Vec<f64>)>,
}

/// The riding frame's |p''| bound read window by window (default on; `=0`
/// keeps one bound over the whole motion). Either is a bound; a window's is
/// read where the step stays within it, so steps lengthen where the curve
/// bends gently.
fn frame_windows_on() -> bool {
    crate::switch!("BREP_SWEEP_FRAME_WINDOWS", true)
}

/// The second-order step for two turns about fixed axes (`BREP_SWEEP_TURNS_GLIDE`, default on;
/// `=0` restores the first-order walk).
fn turns_glide_on() -> bool {
    crate::switch!("BREP_SWEEP_TURNS_GLIDE", true)
}

/// The second-order step for frames riding a curve (default on; `=0` restores the first-order walk).
fn frame_glide_on() -> bool {
    crate::switch!("BREP_SWEEP_FRAME_GLIDE", true)
}

impl Glide {
    fn of(screw: &Screw, y: Vec3) -> Option<Glide> {
        if let (Some(frame), None, true) = (&screw.frame, &screw.turns, frame_glide_on()) {
            return Glide::riding(screw, frame, y);
        }
        if let (Some(turns), true) = (&screw.turns, turns_glide_on()) {
            return Glide::turning(turns, y);
        }
        if screw.frame.is_some() || screw.turns.is_some() {
            return None;
        }
        let r = y.sub(screw.a);
        let u = r.sub(screw.e.scale(r.dot(screw.e)));
        let curve = screw.omega * screw.omega * u.length() + 2.0 * screw.c2.abs();
        // A bound rounded to nothing, or below the normal range, is not one:
        // only a slide (no turn, no rise) is straight.
        let straight = screw.omega == 0.0 && screw.c2 == 0.0;
        if !straight && !(curve >= f64::MIN_POSITIVE && curve.is_finite()) {
            return None;
        }
        Some(Glide { e: screw.e, u, v: screw.e.cross(u), omega: screw.omega, c: screw.c, c2: screw.c2, curve, riding: None, turning: None, windows: None })
    }

    /// A frame riding a planar cubic: the preimage `p = a + R(−θ) q`,
    /// `q = y − P(t)`, turns about `up`, and its second derivative is
    /// `R(−θ)(θ'² up×(up×q) − θ'' up×q + 2θ' up×P' − P'')`, so
    /// `|p''| ≤ (θ'² + |θ''|)·|q| + 2|θ'|·|P'| + |P''|`. Over the motion's
    /// span |P'| and |P''| are bounded by their control polygons and |P'''|
    /// is a constant; `θ' = up·(P'×P'')/|P'|²` and θ'' are bounded through a
    /// lower bound on |P'| (its samples less |P''| times half their spacing;
    /// no glide where that is not positive); `|q| ≤ |y − P(t₀)| + |P'|·span`.
    fn riding(screw: &Screw, frame: &Frame, y: Vec3) -> Option<Glide> {
        let [p0, p1, p2, p3] = frame.control;
        let first = [p1.sub(p0), p2.sub(p1), p3.sub(p2)].map(|d| d.scale(3.0).length()).into_iter().fold(0.0f64, f64::max);
        let second = [p2.sub(p1.scale(2.0)).add(p0), p3.sub(p2.scale(2.0)).add(p1)].map(|d| d.scale(6.0).length()).into_iter().fold(0.0f64, f64::max);
        let third = p3.sub(p2.scale(3.0)).add(p1.scale(3.0)).sub(p0).scale(6.0).length();
        let (t0, t1) = (screw.t0, screw.t1);
        let n = 256;
        let slowest = (0..=n).map(|k| frame.first(t0 + (t1 - t0) * k as f64 / n as f64).length()).fold(f64::INFINITY, f64::min);
        let floor = slowest - second * 0.5 * (t1 - t0) / n as f64;
        if !(floor > 0.0) {
            return None;
        }
        let w1 = first * second / (floor * floor);
        let w2 = third * first / (floor * floor) + 2.0 * first * first * second * second / floor.powi(4);
        let q = y.sub(frame.point(t0)).length() + first * (t1 - t0);
        let curve = (w1 * w1 + w2) * q + 2.0 * w1 * first + second;
        if !(curve.is_finite() && curve >= f64::MIN_POSITIVE) {
            return None;
        }
        // Window by window: |P'| by the hull of its quadratic over the window,
        // |P''| (linear) at the window's ends, the floor on |P'| from eight
        // samples less |P''| times half their spacing, |q| from the window's
        // start; each bound kept no looser than the whole motion's.
        let windows = frame_windows_on().then(|| {
            let count = 32usize;
            let width = (t1 - t0) / count as f64;
            let bounds = (0..count)
                .map(|i| {
                    let (a, b) = (t0 + width * i as f64, t0 + width * (i + 1) as f64);
                    let (da, db) = (frame.first(a), frame.first(b));
                    let middle = da.add(frame.second(a).scale(0.5 * (b - a)));
                    let first_w = da.length().max(db.length()).max(middle.length());
                    let second_w = frame.second(a).length().max(frame.second(b).length());
                    let m = 8;
                    let slow = (0..=m).map(|j| frame.first(a + (b - a) * j as f64 / m as f64).length()).fold(f64::INFINITY, f64::min);
                    let floor_w = slow - second_w * 0.5 * (b - a) / m as f64;
                    if !(floor_w > 0.0) {
                        return curve;
                    }
                    let w1 = first_w * second_w / (floor_w * floor_w);
                    let w2 = third * first_w / (floor_w * floor_w) + 2.0 * first_w * first_w * second_w * second_w / floor_w.powi(4);
                    let q_w = y.sub(frame.point(a)).length() + first_w * (b - a);
                    let c = (w1 * w1 + w2) * q_w + 2.0 * w1 * first_w + second_w;
                    if c.is_finite() && c >= f64::MIN_POSITIVE { c.min(curve) } else { curve }
                })
                .collect::<Vec<f64>>();
            (t0, width, bounds)
        });
        let zero = Vec3::new(0.0, 0.0, 0.0);
        Some(Glide { e: frame.up, u: zero, v: zero, omega: 0.0, c: 0.0, c2: 0.0, curve, riding: Some((*frame, y)), turning: None, windows })
    }

    /// Two turns about fixed axes (unit `d₀`, `d₁` through `P₀`, `P₁` at rates `r₀`, `r₁`): the
    /// preimage `p = P₀ + R₀(−r₀t)(x − P₀)`, `x = P₁ + R₁(−r₁t)(y − P₁)`. With `x' = −r₁ d₁×(x − P₁)`
    /// and `x'' = −r₁ d₁×x'`, `p'' = R₀(x'') − 2r₀ d₀×R₀(x') + r₀² d₀×(d₀×(p − P₀))`, so
    /// `|p''| ≤ (r₁² + 2|r₀ r₁|)·|y − P₁| + r₀²·(|y − P₁| + |P₁ − P₀|)` (rotations keep lengths).
    fn turning(turns: &Turns, y: Vec3) -> Option<Glide> {
        let a = y.sub(turns.points[1]).length();
        let d = a + turns.points[1].sub(turns.points[0]).length();
        let (r0, r1) = (turns.rates[0].abs(), turns.rates[1].abs());
        let curve = (r1 * r1 + 2.0 * r0 * r1) * a + r0 * r0 * d;
        if !(curve.is_finite() && curve >= f64::MIN_POSITIVE) {
            return None;
        }
        let zero = Vec3::new(0.0, 0.0, 0.0);
        Some(Glide { e: zero, u: zero, v: zero, omega: 0.0, c: 0.0, c2: 0.0, curve, riding: None, turning: Some((*turns, y)), windows: None })
    }

    /// The bound on |p''| over [t, t + h]: the most of the windows it meets,
    /// or the whole motion's.
    fn curve_over(&self, t: f64, h: f64) -> f64 {
        match &self.windows {
            Some((t0, width, bounds)) => {
                let last = bounds.len() - 1;
                let index = |x: f64| -> usize {
                    let k = ((x - t0) / width).floor();
                    if k.is_nan() || k <= 0.0 { 0 } else { (k as usize).min(last) }
                };
                bounds[index(t)..=index(t + h)].iter().fold(0.0f64, |a, &b| a.max(b))
            }
            None => self.curve,
        }
    }

    /// `p'(t)`.
    fn velocity(&self, t: f64) -> Vec3 {
        if let Some((turns, y)) = &self.turning {
            let (p0, p1) = (turns.points[0], turns.points[1]);
            let (d0, d1) = (turns.directions[0], turns.directions[1]);
            let (r0, r1) = (turns.rates[0], turns.rates[1]);
            let arm1 = y.sub(p1).rotate_about(d1, -r1 * t);
            let x_rate = d1.cross(arm1).scale(-r1);
            let arm0 = p1.add(arm1).sub(p0).rotate_about(d0, -r0 * t);
            return x_rate.rotate_about(d0, -r0 * t).sub(d0.cross(arm0).scale(r0));
        }
        if let Some((frame, y)) = &self.riding {
            // p' = R(−θ)(−θ' up×q − P'), q = y − P(t).
            let q = y.sub(frame.point(t));
            let inner = frame.up.cross(q).scale(-frame.angle_rate(t)).sub(frame.first(t));
            return inner.rotate_about(frame.up, -frame.angle(t));
        }
        let (sin, cos) = (self.omega * t).sin_cos();
        self.e
            .scale(-(self.c + 2.0 * self.c2 * t))
            .sub(self.u.scale(self.omega * sin))
            .sub(self.v.scale(self.omega * cos))
    }

    /// The second-order step from `t`, where the preimage is `p` at the
    /// tool's distance `d` (at most `left`; 0 when it finds none).
    #[allow(clippy::too_many_arguments)]
    fn step(&self, split: &Split, p: Vec3, t: f64, d: f64, margin: f64, speed: f64, left: f64) -> f64 {
        let velocity = self.velocity(t);
        // (the planes' root is read in closed form under one bound: the
        // windows' over the rest of the motion)
        let m = if self.windows.is_some() { self.curve_over(t, left) } else { self.curve };
        match &split.shape {
            // Each face's plane alone bounds the distance from below:
            // g(t + h) ≥ g + g'·h − m·h²/2, held above −margin up to the root.
            Shape::Planes(planes) => {
                let mut best = 0.0f64;
                for (n, offset) in planes {
                    let g = n.dot(p) - offset;
                    if g + margin <= 0.0 {
                        continue;
                    }
                    let slope = n.dot(velocity);
                    // The root, in the form that neither overflows nor cancels.
                    let h = if m > 0.0 {
                        let root = slope.hypot((2.0 * m * (g + margin)).sqrt());
                        if slope >= 0.0 { (slope + root) / m } else { 2.0 * (g + margin) / (root - slope) }
                    } else if slope >= 0.0 {
                        left
                    } else {
                        (g + margin) / -slope
                    };
                    if h.is_finite() {
                        best = best.max(h);
                    }
                }
                best.min(left)
            }
            // In the meridian plane: the path (ρ, z) about the tool's axis and
            // its tangent segment, whose distance from the outline is bounded
            // below piece by piece; the path leaves the segment by at most
            // m₂·h²/2, m₂ = 2m + speed²/ρ_min (the radius' bend), ρ_min > 0.
            Shape::Revolved { origin, axis, meridian } => {
                if d < 0.0 {
                    return 0.0;
                }
                let r = p.sub(*origin);
                let z = r.dot(*axis);
                let flat = r.sub(axis.scale(z));
                let rho = flat.length();
                if rho <= 0.0 {
                    return 0.0;
                }
                let dz = velocity.dot(*axis);
                let dflat = velocity.sub(axis.scale(dz));
                let drho = flat.dot(dflat) / rho;
                let q = [rho, z];
                let walk_sqrt = walk_sqrt_on();
                let mut best = 0.0f64;
                let mut h = (d + margin) / speed;
                for _ in 0..64 {
                    h *= 2.0;
                    if h > left {
                        h = left;
                    }
                    // Under the windows, the speed over [t, t + h] by the
                    // step's own: |p'| ≤ |p'(t)| + m·h, no more than `speed`.
                    let (m, speed) = if self.windows.is_some() {
                        let m = self.curve_over(t, h);
                        (m, (velocity.length() + m * h).min(speed))
                    } else {
                        (m, speed)
                    };
                    let low = rho - speed * h;
                    if low <= 0.0 {
                        break;
                    }
                    let bend = 2.0 * m + speed * speed / low;
                    let end = [rho + drho * h, z + dz * h];
                    let clear = if walk_sqrt {
                        meridian.iter().map(|piece| segment_clearance_sqrt(piece, q, end)).fold(f64::INFINITY, f64::min)
                    } else {
                        meridian.iter().map(|piece| segment_clearance(piece, q, end)).fold(f64::INFINITY, f64::min)
                    };
                    // A segment that touches the outline may enter the tool: no bound.
                    let held = clear - 0.5 * bend * h * h;
                    if !held.is_finite() || clear <= 0.0 || held < -margin {
                        break;
                    }
                    best = h;
                    if h >= left {
                        break;
                    }
                }
                best
            }
        }
    }
}

/// A lower bound on the distance between the segment `a`–`b` and a meridian
/// piece: a line piece's exact distance (0 where they cross), an arc's from
/// its whole circle.
fn segment_clearance(piece: &OutlinePiece, a: [f64; 2], b: [f64; 2]) -> f64 {
    let sub = |p: [f64; 2], q: [f64; 2]| [p[0] - q[0], p[1] - q[1]];
    let dot = |p: [f64; 2], q: [f64; 2]| p[0] * q[0] + p[1] * q[1];
    let cross = |p: [f64; 2], q: [f64; 2]| p[0] * q[1] - p[1] * q[0];
    let to_segment = |x: [f64; 2], s: [f64; 2], e: [f64; 2]| -> f64 {
        let d = sub(e, s);
        let len2 = dot(d, d);
        let w = if len2 > 0.0 { (dot(sub(x, s), d) / len2).clamp(0.0, 1.0) } else { 0.0 };
        let foot = [s[0] + d[0] * w, s[1] + d[1] * w];
        let gap = sub(x, foot);
        gap[0].hypot(gap[1])
    };
    match *piece {
        OutlinePiece::Line { from, to } => {
            let (d1, d2) = (sub(b, a), sub(to, from));
            let denom = cross(d1, d2);
            if denom != 0.0 {
                let w = sub(from, a);
                let (s, t) = (cross(w, d2) / denom, cross(w, d1) / denom);
                if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
                    return 0.0;
                }
            }
            to_segment(a, from, to).min(to_segment(b, from, to)).min(to_segment(from, a, b)).min(to_segment(to, a, b))
        }
        OutlinePiece::Arc { centre, radius, .. } => {
            let near = to_segment(centre, a, b);
            let far = { let (x, y) = (sub(a, centre), sub(b, centre)); x[0].hypot(x[1]).max(y[0].hypot(y[1])) };
            (near - radius).max(radius - far).max(0.0)
        }
        // no bound read for an ellipse: zero is one
        OutlinePiece::Ellipse { .. } => 0.0,
    }
}

/// The switch for [`segment_clearance_sqrt`] in the walk's second-order step
/// (default on; `=0` reads `segment_clearance`).
fn walk_sqrt_on() -> bool {
    crate::switch!("BREP_SWEEP_WALK_SQRT", true)
}

/// [`segment_clearance`] without `hypot`: a line piece's least squared gap,
/// one square root; an arc's lengths by `sqrt` (within a rounding of `hypot`).
/// A square outside the range where squaring is exact to a rounding (below
/// 1e-290, which takes in an underflow to zero, or above 1e290) reads the
/// piece by `segment_clearance` instead.
fn segment_clearance_sqrt(piece: &OutlinePiece, a: [f64; 2], b: [f64; 2]) -> f64 {
    let in_range = |x: f64| (1e-290..=1e290).contains(&x);
    let sub = |p: [f64; 2], q: [f64; 2]| [p[0] - q[0], p[1] - q[1]];
    let dot = |p: [f64; 2], q: [f64; 2]| p[0] * q[0] + p[1] * q[1];
    let cross = |p: [f64; 2], q: [f64; 2]| p[0] * q[1] - p[1] * q[0];
    let to_segment_squared = |x: [f64; 2], s: [f64; 2], e: [f64; 2]| -> f64 {
        let d = sub(e, s);
        let len2 = dot(d, d);
        let w = if len2 > 0.0 { (dot(sub(x, s), d) / len2).clamp(0.0, 1.0) } else { 0.0 };
        let gap = sub(x, [s[0] + d[0] * w, s[1] + d[1] * w]);
        dot(gap, gap)
    };
    match *piece {
        OutlinePiece::Line { from, to } => {
            let (d1, d2) = (sub(b, a), sub(to, from));
            let denom = cross(d1, d2);
            if denom != 0.0 {
                let w = sub(from, a);
                let (s, t) = (cross(w, d2) / denom, cross(w, d1) / denom);
                if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
                    return 0.0;
                }
            }
            let least = to_segment_squared(a, from, to)
                .min(to_segment_squared(b, from, to))
                .min(to_segment_squared(from, a, b))
                .min(to_segment_squared(to, a, b));
            if !in_range(least) {
                return segment_clearance(piece, a, b);
            }
            least.sqrt()
        }
        OutlinePiece::Arc { centre, radius, .. } => {
            let near = to_segment_squared(centre, a, b);
            let (x, y) = (sub(a, centre), sub(b, centre));
            let far = dot(x, x).max(dot(y, y));
            if !(in_range(near) && in_range(far)) {
                return segment_clearance(piece, a, b);
            }
            (near.sqrt() - radius).max(radius - far.sqrt()).max(0.0)
        }
        OutlinePiece::Ellipse { .. } => 0.0,
    }
}

/// A bound on how fast the tool-frame preimage `screw.invert(t, y)` of the
/// world point `y` moves per unit `t`; the tool's distance (1-Lipschitz)
/// changes no faster along it.
///
/// A plain screw about one fixed axis carries the preimage round that axis at
/// its distance from it, `ρ`, and along it at the advance's rate, two motions
/// square to each other: no faster than `hypot(|ω|·ρ, rate)`. A point near the axis barely moves, so a probe beside
/// a tool point on the axis (a coin turned about a line through its rim) walks
/// the motion in a few long steps, not millions of floor-sized ones. A riding
/// frame or two turns keep the coarser bound through `a`.
fn preimage_speed_bound(screw: &Screw, split: &Split, y: Vec3) -> f64 {
    let speed = if screw.frame.is_none() && screw.turns.is_none() {
        let r = y.sub(screw.a);
        let rho = r.sub(screw.e.scale(r.dot(screw.e))).length();
        (screw.omega.abs() * rho).hypot(screw.rate_bound())
    } else {
        let reach = y.sub(screw.a).length() + split.scale + screw.extent;
        screw.omega.abs() * reach + screw.rate_bound()
    };
    speed.max(1e-300)
}

/// How far a revolved tool reaches about a plain screw's axis: its axial
/// coordinate `e·(x − a)` lies within `axial` and its distance from the axis
/// within `radial`, for every tool point `x` (bounds, not extremes: a meridian
/// line is bounded at its ends, an arc by its whole circle).
///
/// A plain screw keeps each point's distance from its axis and shifts its
/// axial coordinate only by the advance (a slide keeps every coordinate across
/// its direction), so a point `y` whose preimage stays outside these bounds
/// all through the motion is never inside the tool, and `covered` need not
/// walk the motion to say so. Without this a probe beside a face the tool runs
/// along all through the motion (the flat top of a coin turned about a line
/// through its rim, the floor of an end mill's pass) creeps through it in
/// steps of its own tiny distance from the tool.
struct ToolReach {
    /// The tool's revolution: its origin less the screw's point, its unit axis
    /// and its meridian (radius, height along the axis).
    base: Vec3,
    axis: Vec3,
    meridian: Vec<OutlinePiece>,
    axial: [f64; 2],
    radial: [f64; 2],
}

/// The largest of |m₀|·p + m₁·q over a meridian (p ≥ 0): a line at its ends,
/// an arc by its circle (Cauchy–Schwarz on the offset from its centre).
fn meridian_most(meridian: &[OutlinePiece], p: f64, q: f64) -> f64 {
    meridian
        .iter()
        .map(|piece| match *piece {
            OutlinePiece::Line { from, to } => (from[0].abs() * p + from[1] * q).max(to[0].abs() * p + to[1] * q),
            OutlinePiece::Arc { centre, radius, .. } => centre[0].abs() * p + centre[1] * q + radius * p.hypot(q),
            // within the circle of its larger semi-axis
            OutlinePiece::Ellipse { centre, semi, .. } => centre[0].abs() * p + centre[1] * q + semi[0].max(semi[1]) * p.hypot(q),
        })
        .fold(f64::NEG_INFINITY, f64::max)
}

impl ToolReach {
    fn of(screw: &Screw, split: &Split) -> Option<ToolReach> {
        if screw.frame.is_some() || screw.turns.is_some() {
            return None;
        }
        let Shape::Revolved { origin, axis, meridian } = &split.shape else { return None };
        let e = screw.e;
        // x − a = (origin − a) + m₁·axis + m₀·(unit vector ⊥ axis).
        let base = origin.sub(screw.a);
        let along = axis.dot(e);
        let across = axis.cross(e).length();
        let middle = base.dot(e);
        let top = middle + meridian_most(meridian, across, along);
        let bottom = middle - meridian_most(meridian, across, -along);
        // Off the axis: within |(origin − a)⊥| ± (|m₀| + |m₁|·|axis⊥e|).
        let off = base.sub(e.scale(middle)).length();
        let spread = meridian
            .iter()
            .map(|piece| match *piece {
                OutlinePiece::Line { from, to } => (from[0].abs() + from[1].abs() * across).max(to[0].abs() + to[1].abs() * across),
                OutlinePiece::Arc { centre, radius, .. } => centre[0].abs() + centre[1].abs() * across + radius * (1.0 + across * across).sqrt(),
                OutlinePiece::Ellipse { centre, semi, .. } => centre[0].abs() + centre[1].abs() * across + semi[0].max(semi[1]) * (1.0 + across * across).sqrt(),
            })
            .fold(f64::NEG_INFINITY, f64::max);
        let reach = ToolReach { base, axis: *axis, meridian: meridian.clone(), axial: [bottom, top], radial: [off - spread, off + spread] };
        (top.is_finite() && bottom.is_finite() && spread.is_finite()).then_some(reach)
    }

    /// `covered` for a turn (no advance) about an axis along the tool's own,
    /// walked in the tool's radius instead of in time; `None` when the motion
    /// is not such a turn.
    ///
    /// The preimage keeps its height along the tool's axis, and its distance
    /// from that axis is `ρ_w² = ρ² + s² − 2·ρ·s·cos(θ − θ_W)`, `ρ` its own
    /// distance from the turn's axis, `s` the tool axis' and `θ − θ_W` the
    /// angle between them, which the turn sweeps through an interval. So the
    /// preimage takes every radius between the least and the most `ρ_w` over
    /// that interval, and no other: it is ever deeper than the margin inside
    /// the tool if and only if some radius in that range is, at that height.
    /// The meridian's distance is 1-Lipschitz in the radius, so the range is
    /// walked by it. Each radius is met once, where the walk in time crawls
    /// past a probe the tool grazes (by the probe's own tiny offset over the
    /// turn's speed at each step).
    fn turn_about_parallel_axis(&self, screw: &Screw, y: Vec3, margin: f64) -> Option<Result<bool, KernelRefusal>> {
        if screw.c != 0.0 || screw.c2 != 0.0 || screw.omega == 0.0 || self.axis.cross(screw.e).length() != 0.0 {
            return None;
        }
        let e = screw.e;
        let r = y.sub(screw.a);
        // From the tool's origin to the probe, and its height along the tool's
        // axis (the turn keeps it).
        let from_tool = r.sub(self.base);
        let height = from_tool.dot(self.axis);
        // In the plane square to the axis, from the turn's axis: the probe `P`
        // and the tool's axis `W`, and the probe from the tool's axis, `P − W`
        // (formed as one difference, not as two far points less each other).
        let flat = |x: Vec3| x.sub(e.scale(x.dot(e)));
        let (probe, tool, apart) = (flat(r), flat(self.base), flat(from_tool));
        let (rho, s) = (probe.length(), tool.length());
        // Rounding of the coordinates read here, a few ulps of the largest: past
        // a quarter of the margin, the walk in time decides instead.
        let slack = 64.0 * f64::EPSILON * (r.length() + self.base.length() + height.abs());
        if slack > 0.25 * margin {
            return None;
        }
        // The preimage at time t is R(−ω·t)·P: its distance from the tool's axis
        // is |R(φ)·(P − W) + (R(φ) − I)·W|, with R(φ) − I = −2 sin²(φ/2) + sin φ·(e ×).
        let radius = |phi: f64| -> f64 {
            let half = (0.5 * phi).sin();
            let turned = apart.rotate_about(e, phi).add(tool.scale(-2.0 * half * half)).add(e.cross(tool).scale(phi.sin()));
            turned.length()
        };
        let (phi0, phi1) = (-screw.omega * screw.t0, -screw.omega * screw.t1);
        // The angle from W to P; R(φ)·P lies along W at start + φ ≡ 0 and opposite it at ≡ π.
        let start = if rho == 0.0 || s == 0.0 { 0.0 } else { e.dot(tool.cross(probe)).atan2(tool.dot(probe)) };
        let (lo, hi) = ((start + phi0).min(start + phi1), (start + phi0).max(start + phi1));
        // Does [lo, hi] hold an angle ≡ `at` (mod 2π)?
        let holds = |at: f64| (lo - at).div_euclid(TAU) != (hi - at).div_euclid(TAU) || (lo - at).rem_euclid(TAU) == 0.0;
        let ends = [radius(phi0), radius(phi1)];
        // | |P| − |W| | as |(P − W)·(P + W)| / (|P| + |W|): no cancellation.
        let nearest = if rho + s > 0.0 { apart.dot(probe.add(tool)).abs() / (rho + s) } else { 0.0 };
        let mut least = if holds(0.0) { nearest } else { ends[0].min(ends[1]) };
        let mut most = if holds(PI) { rho + s } else { ends[0].max(ends[1]) };
        least = (least - slack).max(0.0);
        most += slack;
        let mut at = least;
        // No floor on the step (it is never less than the margin): a floor
        // could step over a wall thinner than itself.
        let mut steps = 0usize;
        loop {
            let d = outline_signed_distance(&self.meridian, [at, height]);
            if d < -margin {
                return Some(Ok(true));
            }
            if at >= most {
                return Some(Ok(false));
            }
            if steps > 2_000_000 {
                return Some(Err(refuse("probe_budget", "the one-pass check could not walk the motion past a point near the sweep".into())));
            }
            at = (at + d.abs() + margin).min(most);
            steps += 1;
        }
    }

    /// A bound on `u·(x − a)` over the tool, `u` a unit direction.
    fn support(&self, u: Vec3) -> f64 {
        let along = u.dot(self.axis);
        let across = u.sub(self.axis.scale(along)).length();
        u.dot(self.base) + meridian_most(&self.meridian, across, along)
    }

    /// Is `y`'s preimage outside the tool's reach, or within half the margin
    /// of its edge, at every time of the motion? Then the tool's boundary is
    /// within half the margin of every preimage inside (straight out of the
    /// bound), and none is deeper than the margin. `normal`: the probe's face
    /// normal, across which a slide is also tried.
    fn never_inside(&self, screw: &Screw, y: Vec3, normal: Vec3, margin: f64) -> bool {
        let r = y.sub(screw.a);
        let z = r.dot(screw.e);
        let rho = r.sub(screw.e.scale(z)).length();
        let mut advances = vec![screw.advance(screw.t0), screw.advance(screw.t1)];
        if screw.c2 != 0.0 {
            let vertex = -screw.c / (2.0 * screw.c2);
            if vertex > screw.t0 && vertex < screw.t1 {
                advances.push(screw.advance(vertex));
            }
        }
        let low = advances.iter().copied().fold(f64::INFINITY, f64::min);
        let high = advances.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        // Half the margin, less what rounding may have cost the coordinates
        // and bounds compared (a few ulps of the largest of them).
        let size = r.length() + low.abs().max(high.abs()) + self.base.length() + self.radial[1].abs() + self.axial[0].abs().max(self.axial[1].abs());
        let half = 0.5 * margin - 1e-13 * size;
        if half <= 0.0 {
            return false;
        }
        if z - high >= self.axial[1] - half || z - low <= self.axial[0] + half {
            return true;
        }
        if rho >= self.radial[1] - half || (self.radial[0] > 0.0 && rho <= self.radial[0] + half) {
            return true;
        }
        // A slide carries the preimage along its direction only. `u` is meant
        // across it, but whatever of `e` rounding leaves in it moves `u·r`
        // with the advance: take the least `u·r` reaches over the motion.
        if screw.omega == 0.0 {
            if let Ok(u) = normal.sub(screw.e.scale(normal.dot(screw.e))).normalized() {
                let drift = u.dot(screw.e);
                let least = u.dot(r) - (drift * low).max(drift * high);
                if least >= self.support(u) - half {
                    return true;
                }
            }
        }
        false
    }
}

/// Refuse a sweep that covers points just outside its own faces.
///
/// `sliding`: faces the tool slides within all through the motion. A probe
/// just off one stays that near the tool the whole way, and the walk's step
/// is that distance: it stands a thousand times further off them (whatever
/// of the tool rides higher than that over such a face is still found).
fn check_single_pass(screw: &Screw, split: &Split, body: &BrepSolid, sliding: &std::collections::HashSet<u64>) -> Result<(), KernelRefusal> {
    let (on, check) = (parallel_turn_one_pass_on(), parallel_turn_one_pass_check_on());
    if (on || check) && parallel_turn_one_pass(screw, split) {
        if !check {
            return Ok(());
        }
        let walked = walk_single_pass(screw, split, body, sliding);
        PARALLEL_TURN_CHECK.with(|cell| {
            let (n, bad) = cell.get();
            cell.set((n + 1, bad + u64::from(walked.is_err())));
        });
        return walked;
    }
    walk_single_pass(screw, split, body, sliding)
}

/// `BREP_SWEEP_PARALLEL_TURN_ONE_PASS` (default on, `=0` walks): the one-pass check
/// answered in closed form for a solid of revolution turned (no advance)
/// about an axis parallel to its own, `D` from it, whose every section square
/// to the axes is one disc of radius `r(z) <= r_max < D`. A point's
/// distance from the turn's axis stays fixed; the disc at its height covers,
/// of the circle it runs on, one arc of half-angle at most `asin(r_max / D)`
/// centred on the tool's axis, and that arc moves round with the turn. The
/// point is inside over one interval of the turn unless the arc comes round
/// to it again: never while the turn is short of `2π − 2 asin(r_max / D)`.
/// The sampled walk is then not needed.
pub(crate) fn parallel_turn_one_pass_on() -> bool {
    crate::switch!("BREP_SWEEP_PARALLEL_TURN_ONE_PASS", true)
}

/// `BREP_SWEEP_PARALLEL_TURN_ONE_PASS_CHECK` (default off): where the closed
/// form says one pass, the walk runs too and decides; the tally counts the
/// walks that refused ([`parallel_turn_one_pass_check_take`]).
fn parallel_turn_one_pass_check_on() -> bool {
    crate::switch!("BREP_SWEEP_PARALLEL_TURN_ONE_PASS_CHECK", false)
}

thread_local! {
    static PARALLEL_TURN_CHECK: std::cell::Cell<(u64, u64)> = const { std::cell::Cell::new((0, 0)) };
}

/// (closed-form one-pass answers walked, walks that refused) on this thread
/// since the last call.
pub fn parallel_turn_one_pass_check_take() -> (u64, u64) {
    PARALLEL_TURN_CHECK.with(|cell| cell.replace((0, 0)))
}

/// Whether the closed form above proves the sweep one pass.
fn parallel_turn_one_pass(screw: &Screw, split: &Split) -> bool {
    if screw.frame.is_some() || screw.turns.is_some() || screw.c != 0.0 || screw.c2 != 0.0 || screw.omega == 0.0 {
        return false;
    }
    let Shape::Revolved { origin, axis, meridian } = &split.shape else { return false };
    if axis.cross(screw.e).length() > 1e-12 {
        return false;
    }
    let apart = origin.sub(screw.a);
    let d = apart.sub(screw.e.scale(apart.dot(screw.e))).length();
    // Every section one disc: the outline less its pieces on the axis is one
    // run whose axial coordinate never turns back (lines; arcs without a
    // turning point inside), and its largest radius.
    // On the axis exactly: a wall any way off it is a bore, however thin.
    let on_axis = |p: [f64; 2]| p[0] == 0.0;
    let axial: Vec<bool> = meridian.iter().map(|piece| matches!(*piece, OutlinePiece::Line { from, to } if on_axis(from) && on_axis(to))).collect();
    let changes = (0..axial.len()).filter(|&k| axial[k] != axial[(k + 1) % axial.len()]).count();
    if changes != 2 {
        return false; // the pieces on the axis are not one run of them
    }
    let mut direction = 0i32;
    let mut r_max = 0.0f64;
    let mut agree = |sign: i32| -> bool {
        if sign == 0 {
            return true;
        }
        if direction == 0 {
            direction = sign;
        }
        direction == sign
    };
    for piece in meridian {
        match *piece {
            OutlinePiece::Line { from, to } => {
                if on_axis(from) && on_axis(to) {
                    continue;
                }
                if from[0] < -1e-12 || to[0] < -1e-12 {
                    return false;
                }
                r_max = r_max.max(from[0]).max(to[0]);
                let rise = to[1] - from[1];
                if !agree(if rise > 0.0 { 1 } else if rise < 0.0 { -1 } else { 0 }) {
                    return false;
                }
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                let (lo, hi) = (start.min(end), start.max(end));
                if !(hi > lo) || !(radius > 0.0) {
                    return false;
                }
                // Turning points of the axial coordinate (cos a = 0) strictly inside.
                let first = ((lo - FRAC_PI_2) / PI).floor() as i64 + 1;
                if FRAC_PI_2 + first as f64 * PI < hi - 1e-12 {
                    return false;
                }
                // Its radial reach: the ends, and a = 0 (mod 2π) inside.
                let reach = |a: f64| centre[0] + radius * a.cos();
                r_max = r_max.max(reach(start)).max(reach(end));
                if (lo / TAU).ceil() * TAU < hi {
                    r_max = r_max.max(centre[0] + radius);
                }
                if centre[0] - radius < -1e-12 && (lo..hi).contains(&PI) {
                    return false;
                }
                let mid = 0.5 * (start + end);
                let rise = radius * mid.cos() * (end - start).signum();
                if !agree(if rise > 0.0 { 1 } else if rise < 0.0 { -1 } else { 0 }) {
                    return false;
                }
            }
            _ => return false,
        }
    }
    if !(r_max > 0.0 && r_max < d * (1.0 - 1e-9)) {
        return false;
    }
    let turn = (screw.omega * (screw.t1 - screw.t0)).abs();
    turn + 2.0 * (r_max / d).asin() < TAU * (1.0 - 1e-9)
}

fn walk_single_pass(screw: &Screw, split: &Split, body: &BrepSolid, sliding: &std::collections::HashSet<u64>) -> Result<(), KernelRefusal> {
    let margin = 1e-9 * split.scale;
    let reach = ToolReach::of(screw, split);
    for face in body.shells.iter().flat_map(|s| &s.faces) {
        let step = if sliding.contains(&face.id) { 1e-3 * split.scale } else { 1e-6 * split.scale };
        let surface = &face.surface;
        // Probe inside the face: towards each corner of its loop from the
        // average of its corners (caps are convex in their parameters), and on
        // a grid for the four-sided walls.
        let mut corners: Vec<[f64; 2]> = Vec::new();
        for coedge in &face.loops[0].coedges {
            let domain = frag(coedge.pcurve.domain(), "domain")?;
            for k in 0..4 {
                let p = frag(coedge.pcurve.evaluate(domain[0] + (domain[1] - domain[0]) * k as f64 / 4.0), "pcurve")?;
                corners.push([p.x, p.y]);
            }
        }
        let middle = corners.iter().fold([0.0, 0.0], |s, c| [s[0] + c[0], s[1] + c[1]]);
        let middle = [middle[0] / corners.len() as f64, middle[1] / corners.len() as f64];
        let mut probes: Vec<[f64; 2]> = vec![middle];
        for c in &corners {
            for w in [0.25, 0.5, 0.75, 0.9, 0.99] {
                probes.push([middle[0] + (c[0] - middle[0]) * w, middle[1] + (c[1] - middle[1]) * w]);
            }
        }
        if surface.degree_v == 5 {
            let [u0, u1] = frag(surface.domain_u(), "domain")?;
            for i in 1..8 {
                for j in 1..64 {
                    probes.push([u0 + (u1 - u0) * i as f64 / 8.0, j as f64 / 64.0]);
                }
            }
        }
        let ([u0, u1], [v0, v1]) = (frag(surface.domain_u(), "domain")?, frag(surface.domain_v(), "domain")?);
        // Keep the probes inside the face's own loop (a face under a grazing
        // curve that dips is not convex in its parameters).
        let mut ring: Vec<[f64; 2]> = Vec::new();
        for coedge in &face.loops[0].coedges {
            let domain = frag(coedge.pcurve.domain(), "domain")?;
            for k in 0..32 {
                let p = frag(coedge.pcurve.evaluate(domain[0] + (domain[1] - domain[0]) * k as f64 / 32.0), "pcurve")?;
                ring.push([p.x, p.y]);
            }
        }
        let (lo, hi) = ring.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), q| ([lo[0].min(q[0]), lo[1].min(q[1])], [hi[0].max(q[0]), hi[1].max(q[1])]));
        for i in 1..12 {
            for j in 1..12 {
                probes.push([lo[0] + (hi[0] - lo[0]) * i as f64 / 12.0, lo[1] + (hi[1] - lo[1]) * j as f64 / 12.0]);
            }
        }
        let inside = |q: [f64; 2]| -> bool {
            let mut crossings = 0;
            for k in 0..ring.len() {
                let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
                if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                    crossings += 1;
                }
            }
            crossings % 2 == 1
        };
        for [u, v] in probes.into_iter().filter(|&q| inside(q)) {
            let p = frag(surface.evaluate(u, v), "probe")?;
            let (hu, hv) = (1e-6 * (u1 - u0), 1e-6 * (v1 - v0));
            // Derivatives, not raw differences: their cross product is
            // measured against an absolute zero.
            let (ua, ub, va, vb) = ((u - hu).max(u0), (u + hu).min(u1), (v - hv).max(v0), (v + hv).min(v1));
            let du = frag(surface.evaluate(ub, v), "probe")?.sub(frag(surface.evaluate(ua, v), "probe")?).scale(1.0 / (ub - ua));
            let dv = frag(surface.evaluate(u, vb), "probe")?.sub(frag(surface.evaluate(u, va), "probe")?).scale(1.0 / (vb - va));
            let Ok(n) = du.cross(dv).normalized() else { continue };
            let n = if face.same_sense { n } else { n.scale(-1.0) };
            if covered(screw, split, reach.as_ref(), p.add(n.scale(step)), n, margin)? {
                return Err(refuse("folds", format!("the sweep passes over a point twice (just outside its own face at {p:?}); not one pass")));
            }
        }
    }
    Ok(())
}


#[path = "solid_sweep_ring_fold.rs"]
mod ring_fold;

#[path = "solid_sweep_ball_screw.rs"]
mod ball_screw;


