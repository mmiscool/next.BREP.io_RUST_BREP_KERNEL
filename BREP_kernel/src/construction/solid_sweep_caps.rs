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
use crate::{make_line, make_revolution, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, OutlinePiece, SolidMotion, Vec3, Vec4};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

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
struct Screw {
    a: Vec3,
    e: Vec3,
    omega: f64,
    c: f64,
    c2: f64,
    t0: f64,
    t1: f64,
    /// A frame riding a curve instead (then `a` is the curve's start, `e` the
    /// frame's fixed up direction, `omega` a bound on its turning rate and `c`
    /// on its speed).
    frame: Option<Frame>,
    /// Two turns about fixed axes instead (then `a` is the first axis' point).
    turns: Option<Turns>,
    /// The time whose velocity field splits the tool (default: the middle).
    frozen: Option<f64>,
    /// How far the frame's origin travels from `a` (for speed bounds).
    extent: f64,
}

/// Two turns driven by one parameter: first `rates[0]·t` about the first
/// axis, then `rates[1]·t` about the second; both axes fixed in the world.
#[derive(Clone, Copy, Debug)]
struct Turns {
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
struct Frame {
    control: [Vec3; 4],
    up: Vec3,
    /// The tangent's direction at the start.
    start_tangent: Vec3,
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
    fn from_motion(motion: &SolidMotion) -> Result<Self, KernelRefusal> {
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
        Ok(Self { a: step.axis_origin, e, omega: step.turn, c: step.advance, c2: 0.0, t0: motion.from, t1: motion.to, frame: None, turns: None, frozen: None, extent: 0.0 })
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
        Ok(Self { a: motion.axis_origin, e, omega: 1.0, c: motion.rise[0], c2: motion.rise[1], t0: motion.from, t1: motion.to, frame: None, turns: None, frozen: None, extent: 0.0 })
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
    fn velocity(&self, x: Vec3) -> Vec3 {
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
const LOFT_SAMPLES: usize = 128;

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
    scale: f64,
    shape: Shape,
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
    Ok(Split { points, curves, caps, scale, shape: Shape::Planes(planes) })
}

/// The grazing value `A cos ψ + B sin ψ + C` of a face of revolution at one
/// meridian point, from three samples.
fn harmonic(g: impl Fn(f64) -> f64) -> (f64, f64, f64) {
    let (g0, g90, g180) = (g(0.0), g(0.5 * PI), g(PI));
    let c = 0.5 * (g0 + g180);
    (0.5 * (g0 - g180), g90 - c, c)
}

fn piece_point(piece: &OutlinePiece, s: f64) -> [f64; 2] {
    match *piece {
        OutlinePiece::Line { from, to } => [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s],
        OutlinePiece::Arc { centre, radius, start, end } => {
            let angle = start + (end - start) * s;
            [centre[0] + radius * angle.cos(), centre[1] + radius * angle.sin()]
        }
    }
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
                    return Err(refuse("revolved_grazing", format!("meridian piece {i} grazes all over")));
                }
                let r0 = readings[0].clone();
                let still = readings.iter().all(|r| r.len() == r0.len() && r.iter().zip(&r0).all(|(x, y)| (x - y).abs() < 1e-10));
                if still {
                    kinds.push(Kind::Ruled(r0));
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
                if centre[0] > radius * (1.0 + 1e-9) && end > start && span < PI {
                    let at_rim = |v: f64| [centre[0] + radius * v.cos(), centre[1] + radius * v.sin()];
                    let g_tube = |psi: f64, v: f64| -> f64 {
                        let m = at_rim(v);
                        let x = at(m[0], m[1], psi);
                        let normal = w.scale(v.sin()).add(u1.scale(v.cos() * psi.cos())).add(u2.scale(v.cos() * psi.sin()));
                        screw.velocity(x).dot(normal)
                    };
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
                Kind::Ruled(roots) => angles.extend(roots.iter().copied()),
                Kind::Ball { .. } => {}
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
                Kind::Ruled(roots) => roots.clone(),
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
                Kind::Sphere { .. } | Kind::Ball { .. } => continue,
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
                if on_axis(m0) || on_axis(m1) {
                    return Err(refuse("revolved_side", format!("meridian piece {piece_index}: a split cone to a point is not swept here")));
                }
                // Generators at each root, from the start rim to the end rim.
                let mut generators = Vec::with_capacity(roots.len());
                for &root in roots {
                    let (a, b) = (rim_points[&(below, arc_index(below, root)?)], rim_points[&(above, arc_index(above, root)?)]);
                    let line_samples = (0..=LOFT_SAMPLES).map(|i| points[a].add(points[b].sub(points[a]).scale(i as f64 / LOFT_SAMPLES as f64))).collect();
                    curves.push(ToolCurve { curve: frag(make_line(points[a], points[b]), "generator")?, start: a, end: b, degenerate: false, samples: Some(line_samples) });
                    generators.push(curves.len() - 1);
                }
                for j in 0..roots.len() {
                    let (alpha, beta) = (roots[j], if j + 1 < roots.len() { roots[j + 1] } else { roots[0] + TAU });
                    let generatrix = frag(make_line(at(m0[0], m0[1], alpha), at(m1[0], m1[1], alpha)), "generatrix")?;
                    let surface = frag(make_revolution(origin, w, &generatrix, beta - alpha), "patch")?;
                    let front = sign_at(piece, 0.5 * (alpha + beta));
                    let (lower, upper) = (rim_arcs[&(below, arc_index(below, alpha)?)], rim_arcs[&(above, arc_index(above, alpha)?)]);
                    let (g_alpha, g_beta) = (generators[j], generators[(j + 1) % roots.len()]);
                    let coedges = vec![
                        (lower, true, uv_line([0.0, 0.0], [1.0, 0.0])?),
                        (g_beta, true, uv_line([1.0, 0.0], [1.0, 1.0])?),
                        (upper, false, uv_line([1.0, 1.0], [0.0, 1.0])?),
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
                    curves.push(ToolCurve { curve: frag(crate::interpolate_curve(&samples, 5, &params), "branch")?, start, end, degenerate: false, samples: Some(exact) });
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
                        let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
                        let mut uv = Vec::with_capacity(ts.len());
                        for &t in &ts {
                            let x = frag(curve.evaluate(t), "rim")?;
                            uv.push(Vec3::new(u_of(angle_near(x, alpha + 0.5 * theta)), v, 0.0));
                        }
                        frag(crate::interpolate_curve(&uv, 5, &ts), "rim_pcurve")
                    };
                    // A grazing branch's pcurve: the image of the built edge,
                    // through its own parameter (angle about the axis; place
                    // along the meridian line).
                    let branch_pcurve = |which: usize, _turn: f64| -> Result<NurbsCurve, KernelRefusal> {
                        let curve = &curves[branch_curves[which]].curve;
                        let [d0, d1] = frag(curve.domain(), "domain")?;
                        let (span_rho, span_z) = (m1[0] - m0[0], m1[1] - m0[1]);
                        let length2 = span_rho * span_rho + span_z * span_z;
                        let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
                        let mut uv = Vec::with_capacity(ts.len());
                        for &t in &ts {
                            let x = frag(curve.evaluate(t), "branch")?;
                            let r = x.sub(origin);
                            let z = r.dot(w);
                            let rho = r.sub(w.scale(z)).length();
                            let along = ((rho - m0[0]) * span_rho + (z - m0[1]) * span_z) / length2;
                            uv.push(Vec3::new(u_of(angle_near(x, alpha + 0.5 * theta)), along, 0.0));
                        }
                        frag(crate::interpolate_curve(&uv, 5, &ts), "branch_pcurve")
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
                curves.push(ToolCurve { curve: frag(crate::interpolate_curve(&xyz, 5, &params), "torus_k")?, start: ka, end: kb, degenerate: false, samples: Some(exact) });
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
                    let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
                    let mut uv = Vec::with_capacity(ts.len());
                    for &t in &ts {
                        let x = frag(curve.evaluate(t), "point")?;
                        let r = x.sub(origin);
                        let mid = 0.5 * (alpha + beta);
                        let psi = mid + (r.dot(u2).atan2(r.dot(u1)) - mid + PI).rem_euclid(TAU) - PI;
                        let q = x.sub(tube_centre(psi));
                        let radial = u1.scale(psi.cos()).add(u2.scale(psi.sin()));
                        let vmid = 0.5 * (start + end);
                        let v = vmid + (q.dot(w).atan2(q.dot(radial)) - vmid + PI).rem_euclid(TAU) - PI;
                        uv.push(Vec3::new(
                            crate::circle_angle_to_parameter(spans_u, beta - alpha, psi - alpha),
                            crate::circle_angle_to_parameter(spans_v, end - start, v - start),
                            0.0,
                        ));
                    }
                    let fitted = frag(crate::interpolate_curve(&uv, 5, &ts), "torus_pcurve")?;
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
                curves.push(ToolCurve { curve: k_a.clone(), start: i1, end: i2, degenerate: false, samples: None });
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
                            curves.push(ToolCurve { curve: frag(surface.iso_curve_u(1.0), "great")?, start, end, degenerate: false, samples: None });
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
        scale,
        shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() },
    })
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
    let n = 128;
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
            let exact = (n == LOFT_SAMPLES).then(|| xyz.clone());
            curves.push(ToolCurve { curve: frag(crate::interpolate_curve(&xyz, 5, &params), "ring_branch")?, start, end, degenerate: false, samples: exact });
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
                let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
                let mut uv = Vec::with_capacity(ts.len());
                for &t in &ts {
                    let (psi, v) = angles_of(frag(curve.evaluate(t), "point")?);
                    uv.push(uv_of(psi, v));
                }
                frag(crate::interpolate_curve(&uv, 5, &ts), "ring_pcurve")
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
}

impl Shell {
    fn new() -> Self {
        Self { vertices: Vec::new(), edges: Vec::new(), faces: Vec::new(), next: 1 }
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
    fn face(&mut self, surface: NurbsSurface, coedges: Vec<(u64, bool, NurbsCurve)>) {
        let mut records = Vec::with_capacity(coedges.len());
        for (edge, forward, pcurve) in coedges {
            let id = self.id();
            records.push(CoedgeRecord { id, edge_id: edge, forward, pcurve });
        }
        let loop_id = self.id();
        let id = self.id();
        self.faces.push(FaceRecord { id, surface, same_sense: true, loops: vec![LoopRecord { id: loop_id, coedges: records }], name: None });
    }

    fn solid(mut self) -> BrepSolid {
        let shell = self.id();
        let id = self.id();
        // One shell: V − E + F = 2 − 2g (every face one loop; a pole's
        // degenerate edge is not counted, as the validator counts).
        let edges = self.edges.iter().filter(|e| !e.degenerate).count() as i64;
        let euler = self.vertices.len() as i64 - edges + self.faces.len() as i64;
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
    if let SolidTool::Revolved { axis_origin, axis_direction, meridian, .. } = tool {
        if let Some(body) = ball_across_axis(&screw, *axis_origin, *axis_direction, meridian)? {
            return Ok(body);
        }
    }
    let split = match tool {
        SolidTool::Polyhedron { vertices, faces } => split_polyhedron(&screw, vertices, faces)?,
        SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } => {
            split_revolved(&screw, *axis_origin, *axis_direction, *radial, meridian)?
        }
    };
    let grazing = split.grazing()?;
    if grazing.is_empty() {
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
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse("assembly", format!("the assembled sweep is not valid: {:?}", issues.first())));
    }
    check_single_pass(&screw, &split, &body)?;
    Ok(body)
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
fn polyhedron_of_planes(planes: &[(Vec3, f64)]) -> Result<(Vec<Vec3>, Vec<Vec<usize>>), KernelRefusal> {
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
    let profile = polyhedron_shadow(&corners, faces)?;
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

/// One piece of the shadow's outline: a segment (two points) or a rational
/// quadratic arc (three, the middle with its weight), as (ρ, z, weight).
type ShadowPiece = Vec<(f64, f64, f64)>;

/// The shadow {(ρ, z)} of a convex polyhedron (corners in the axis frame):
/// at each z its section is a convex polygon, the ρ it reaches an interval
/// from the section's nearest point to its farthest corner. The farthest is
/// always on an edge (ρ² = quadratic in z: a conic arc); the nearest is on a
/// face (ρ linear in z: a segment) or an edge.
fn polyhedron_shadow(corners: &[[f64; 3]], faces: &[Vec<usize>]) -> Result<Vec<ShadowPiece>, KernelRefusal> {
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
            return Err(refuse("turn_shadow", "the turn's axis passes through the tool".into()));
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
                if s0 == s1 {
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
        if let Source::Face(..) = source {
            return vec![vec![(ra, za, 1.0), (rb, zb, 1.0)]];
        }
        // ρ² = α z² + β z + γ; split so each arc turns little.
        let rho = |z: f64| value(source, z);
        let slope = |z: f64| {
            let h = 1e-6 * (zb - za).abs().max(1e-300);
            (rho(z + h) - rho(z - h)) / (2.0 * h)
        };
        let parts = 4;
        let mut out = Vec::new();
        for k in 0..parts {
            let (z0, z1) = (za + (zb - za) * k as f64 / parts as f64, za + (zb - za) * (k + 1) as f64 / parts as f64);
            let (a, b) = ((rho(z0), z0), (rho(z1), z1));
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
    let pcurve = |inverse: &dyn Fn(Vec3) -> Vec3, curve: &NurbsCurve, forward: bool| -> Result<NurbsCurve, KernelRefusal> {
        let [d0, d1] = frag(curve.domain(), "domain")?;
        let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
        let uv: Vec<Vec3> = ts.iter().map(|&t| curve.evaluate(t).map(|x| inverse(x))).collect::<Result<_, _>>().map_err(|e| refuse("pcurve", e))?;
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
    let probe = Split { points: Vec::new(), curves: Vec::new(), caps: Vec::new(), scale: r + rho, shape: Shape::Revolved { origin, axis: w, meridian: meridian.to_vec() } };
    check_single_pass(screw, &probe, &body)?;
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
/// within 1e-10 of the exact loop at the times between). Caps are the tool's
/// back at the start and front at the end.
pub fn sweep_solid_along(tool: &SolidTool, path: &FrameCurve) -> Result<BrepSolid, KernelRefusal> {
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
    let screw = Screw { a: base, e: up, omega: rate, c: speed, c2: 0.0, t0: 0.0, t1: 1.0, frame: Some(frame), turns: None, frozen: None, extent };
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
    // The frame in plane coordinates: origin c(s), turned by φ(s).
    let flat = |p: Vec3| [p.sub(base).dot(e1), p.sub(base).dot(e2)];
    let place = |s: f64, q: [f64; 2]| -> [f64; 2] {
        let o = flat(frame.point(s));
        let (sn, cs) = frame.angle(s).sin_cos();
        [o[0] + cs * q[0] - sn * q[1], o[1] + sn * q[0] + cs * q[1]]
    };
    let local = |s: f64, p: [f64; 2]| -> [f64; 2] {
        let o = flat(frame.point(s));
        let (sn, cs) = frame.angle(s).sin_cos();
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
    let speed_ratio = |s: f64| frame.angle_rate(s); // κ |c'| / |c'| per unit s, over |c'|
    let graze_point = |k: usize, s: f64| -> Option<f64> {
        // Along edge k at u: v·n = |c'| (e1·n + κ (ẑ × x)·n), κ = φ'/|c'|.
        let (a, b) = (corners[k], corners[(k + 1) % n]);
        let (nn, _) = edges[k];
        let kappa = speed_ratio(s) / frame.first(s).length();
        let rot = |x: [f64; 2]| -x[1] * nn[0] + x[0] * nn[1]; // (ẑ × x)·n
        // An edge along the start tangent grazes where (ẑ × x)·n = 0 whatever
        // the curvature (also where it is zero, at an inflection).
        let (ga, gb) = if nn[0].abs() < 1e-15 { (rot(a), rot(b)) } else { (nn[0] + kappa * rot(a), nn[0] + kappa * rot(b)) };
        if ga == gb {
            return None;
        }
        let u = ga / (ga - gb);
        (u > 0.0 && u < 1.0).then_some(u)
    };
    let mut paths: Vec<Path> = (0..n).map(Path::Corner).collect();
    for k in 0..n {
        // Where along the motion edge k has a grazing point inside it.
        let steps = 2000;
        let mut run: Option<f64> = None;
        for j in 0..=steps {
            let s = j as f64 / steps as f64;
            let on = graze_point(k, s).is_some();
            match (on, run) {
                (true, None) => run = Some(s),
                (false, Some(s0)) => {
                    paths.push(Path::Graze(k, s0, s - 1.0 / steps as f64));
                    run = None;
                }
                _ => {}
            }
        }
        if let Some(s0) = run {
            paths.push(Path::Graze(k, s0, 1.0));
        }
        paths.push(Path::End(k, false));
        paths.push(Path::End(k, true));
    }
    let eval = |path: Path, t: f64| -> [f64; 2] {
        match path {
            Path::Corner(k) => place(t, corners[k]),
            Path::Graze(k, s0, s1) => {
                let s = s0 + (s1 - s0) * t;
                let u = graze_point(k, s).unwrap_or(if t < 0.5 { 0.0 } else { 1.0 });
                let (a, b) = (corners[k], corners[(k + 1) % n]);
                place(s, [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u])
            }
            Path::End(k, at_end) => {
                let (a, b) = (corners[k], corners[(k + 1) % n]);
                place(if at_end { 1.0 } else { 0.0 }, [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
            }
        }
    };
    // Split parameters: mutual crossings (polyline, then Newton), and each
    // path's ends lying on another.
    let m = 400;
    let polylines: Vec<Vec<[f64; 2]>> = paths.iter().map(|&p| (0..=m).map(|i| eval(p, i as f64 / m as f64)).collect()).collect();
    let mut cuts: Vec<Vec<f64>> = vec![vec![0.0, 1.0]; paths.len()];
    let tol = 1e-10 * scale;
    for a in 0..paths.len() {
        for b in a + 1..paths.len() {
            for i in 0..m {
                for j in 0..m {
                    let (p0, p1, q0, q1) = (polylines[a][i], polylines[a][i + 1], polylines[b][j], polylines[b][j + 1]);
                    let d1 = [p1[0] - p0[0], p1[1] - p0[1]];
                    let d2 = [q1[0] - q0[0], q1[1] - q0[1]];
                    let det = d1[0] * d2[1] - d1[1] * d2[0];
                    if det.abs() < 1e-300 {
                        continue;
                    }
                    let r = [q0[0] - p0[0], q0[1] - p0[1]];
                    let (x, y) = ((r[0] * d2[1] - r[1] * d2[0]) / det, (r[0] * d1[1] - r[1] * d1[0]) / det);
                    if !(-1e-9..=1.0 + 1e-9).contains(&x) || !(-1e-9..=1.0 + 1e-9).contains(&y) {
                        continue;
                    }
                    // Newton on the exact paths.
                    let (mut ta, mut tb) = ((i as f64 + x) / m as f64, (j as f64 + y) / m as f64);
                    for _ in 0..50 {
                        let (pa, pb) = (eval(paths[a], ta), eval(paths[b], tb));
                        let h = 1e-7;
                        let da = { let q = eval(paths[a], (ta + h).min(1.0)); let p = eval(paths[a], (ta - h).max(0.0)); let w = (ta + h).min(1.0) - (ta - h).max(0.0); [(q[0] - p[0]) / w, (q[1] - p[1]) / w] };
                        let db = { let q = eval(paths[b], (tb + h).min(1.0)); let p = eval(paths[b], (tb - h).max(0.0)); let w = (tb + h).min(1.0) - (tb - h).max(0.0); [(q[0] - p[0]) / w, (q[1] - p[1]) / w] };
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
                    let (pa, pb) = (eval(paths[a], ta), eval(paths[b], tb));
                    if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) < tol {
                        cuts[a].push(ta);
                        cuts[b].push(tb);
                    }
                }
            }
        }
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
            let (pa, pb) = (eval(paths[index], t0), eval(paths[index], t1));
            if (pa[0] - pb[0]).hypot(pa[1] - pb[1]) < 1e-9 * scale {
                continue;
            }
            let tm = 0.5 * (t0 + t1);
            let (p, q) = (eval(paths[index], tm), eval(paths[index], (tm + 1e-6).min(1.0)));
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
    // Chain into one loop, the region on its left.
    if pieces.is_empty() {
        return Err(refuse("prism_along", "no boundary found".into()));
    }
    let mut order = vec![0usize];
    let mut used = vec![false; pieces.len()];
    used[0] = true;
    loop {
        let last = &pieces[*order.last().expect("order")];
        let end = eval(paths[last.path], last.t1);
        let start = eval(paths[pieces[order[0]].path], pieces[order[0]].t0);
        if order.len() > 1 && (end[0] - start[0]).hypot(end[1] - start[1]) < 1e-8 * scale {
            break;
        }
        let next = (0..pieces.len()).filter(|&k| !used[k]).min_by(|&x, &y| {
            let (px, py) = (eval(paths[pieces[x].path], pieces[x].t0), eval(paths[pieces[y].path], pieces[y].t0));
            (px[0] - end[0]).hypot(px[1] - end[1]).total_cmp(&(py[0] - end[0]).hypot(py[1] - end[1]))
        });
        let Some(next) = next else {
            return Err(refuse("prism_along", "the boundary does not close".into()));
        };
        let p = eval(paths[pieces[next].path], pieces[next].t0);
        if (p[0] - end[0]).hypot(p[1] - end[1]) > 1e-8 * scale {
            return Err(refuse("prism_along", format!("the boundary does not close (gap {:e})", (p[0] - end[0]).hypot(p[1] - end[1]))));
        }
        used[next] = true;
        order.push(next);
    }
    if used.iter().any(|u| !u) {
        return Err(refuse("prism_along", "the swept region is not one simple loop".into()));
    }
    // Plane curves of the loop, then the extrusion.
    let world = |q: [f64; 2], h: f64| base.add(e1.scale(q[0])).add(e2.scale(q[1])).add(up.scale(h));
    let mut loop_points: Vec<[f64; 2]> = order.iter().map(|&k| eval(paths[pieces[k].path], pieces[k].t0)).collect();
    loop_points.push(loop_points[0]);
    let count = order.len();
    let mut shell = Shell::new();
    let (bottom_v, top_v): (Vec<u64>, Vec<u64>) = (0..count).map(|k| (shell.vertex(world(loop_points[k], from)), shell.vertex(world(loop_points[k], to)))).unzip();
    let mut bottom_e = Vec::with_capacity(count);
    let mut top_e = Vec::with_capacity(count);
    let mut curves2d = Vec::with_capacity(count);
    for (slot, &k) in order.iter().enumerate() {
        let piece = &pieces[k];
        let straight = matches!(paths[piece.path], Path::End(..));
        let next = (slot + 1) % count;
        let curve_at = |h: f64| -> Result<NurbsCurve, KernelRefusal> {
            if straight {
                return frag(make_line(world(loop_points[slot], h), world(loop_points[next], h)), "line");
            }
            let samples = 192;
            let ts: Vec<f64> = (0..=samples).map(|i| i as f64 / samples as f64).collect();
            let mut points: Vec<Vec3> = ts.iter().map(|&t| world(eval(paths[piece.path], piece.t0 + (piece.t1 - piece.t0) * t), h)).collect();
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
    // Top (outward up) and bottom (outward down).
    let top_points: Vec<Vec3> = loop_points.iter().map(|&q| world(q, to)).collect();
    let top_surface = plane_patch(world([0.0, 0.0], to), e1, e2, &top_points)?;
    let mut coedges = Vec::with_capacity(count);
    for k in 0..count {
        coedges.push((top_e[k], true, plane_pcurve(world([0.0, 0.0], to), e1, e2, &curves2d[k].1, true)?));
    }
    shell.face(top_surface, coedges);
    let bottom_points: Vec<Vec3> = loop_points.iter().map(|&q| world(q, from)).collect();
    let bottom_surface = plane_patch(world([0.0, 0.0], from), e2, e1, &bottom_points)?;
    let mut coedges = Vec::with_capacity(count);
    for k in (0..count).rev() {
        coedges.push((bottom_e[k], false, plane_pcurve(world([0.0, 0.0], from), e2, e1, &curves2d[k].0, false)?));
    }
    shell.face(bottom_surface, coedges);
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
    let body = shell.solid();
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse("assembly", format!("the swept prism is not valid: {:?}", issues.first())));
    }
    Ok(body)
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
        extent: points[1].sub(points[0]).length(),
    };
    let span = |t0: f64, t1: f64| -> Result<BrepSolid, KernelRefusal> {
        let mut screw = base;
        screw.t0 = t0;
        screw.t1 = t1;
        let mut last = None;
        for k in 0..8 {
            let reference = radial.rotate_about(axis, PI * k as f64 / 4.0);
            match lofted(&screw, *axis_origin, axis, reference, meridian) {
                Err(error) if matches!(refused_as(&error), Some("loft_structure") | Some("end_trim_corner")) => last = Some(error),
                other => return other,
            }
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
    for &tt in &ts {
        let x = screw.invert(t, frag(curve.evaluate(tt), "point")?);
        let [u, v] = chart.invert(x);
        uv.push(Vec3::new(u, v, 0.0));
    }
    let fitted = frag(crate::interpolate_curve(&uv, 5, &ts), "charted_pcurve")?;
    if forward { Ok(fitted) } else { frag(fitted.reversed(), "reversed") }
}

/// A cap's pcurve for a lofted edge: its dense image on the cap's surface.
fn projected_pcurve(surface: &NurbsSurface, curve: &NurbsCurve, forward: bool) -> Result<NurbsCurve, KernelRefusal> {
    let [d0, d1] = frag(curve.domain(), "domain")?;
    let ts: Vec<f64> = (0..=PCURVE_SAMPLES).map(|j| d0 + (d1 - d0) * j as f64 / PCURVE_SAMPLES as f64).collect();
    let mut uv = Vec::with_capacity(ts.len());
    for &t in &ts {
        let x = frag(curve.evaluate(t), "point")?;
        let foot = frag(crate::project_point_to_surface(surface, x), "project")?;
        if foot.distance > 1e-6 * (1.0 + x.length()) {
            return Err(refuse("loft_cap", format!("a lofted edge leaves its cap by {} at {x:?}", foot.distance)));
        }
        uv.push(Vec3::new(foot.u, foot.v, 0.0));
    }
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
    let lambdas: Vec<f64> = (0..=LOFT_SAMPLES).map(|i| i as f64 / LOFT_SAMPLES as f64).collect();
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
    let mut fractions: Vec<f64> = (0..=63).map(|j| j as f64 / 63.0).collect();
    loop {
        let count = fractions.len() - 1;
        // The walls' second parameter runs 0 -> 1 over the motion.
        let times: Vec<f64> = fractions.iter().map(|&f| if f == 1.0 { screw.t1 } else { screw.t0 + (screw.t1 - screw.t0) * f }).collect();
        let splits: Vec<Split> = times.iter().map(|&t| split_at(t)).collect::<Result<_, _>>()?;
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
        let target = 5e-10 * scale;
        // Walls: per grazing curve, rows along it at each time, then across time.
        let mut walls = Vec::with_capacity(grazing.len());
        for &(c, back_forward) in &grazing {
            let mut rows: Vec<NurbsCurve> = Vec::with_capacity(times.len());
            for (j, split) in splits.iter().enumerate() {
                let points = samples_of(split, c, back_forward, times[j])?;
                rows.push(frag(crate::interpolate_curve(&points, 5, &lambdas), "loft_row")?);
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
        // Against the exact loop in the middle of each span, every sample.
        let mut misses = vec![0.0f64; count];
        for j in 0..count {
            let t = 0.5 * (times[j] + times[j + 1]);
            let fraction = 0.5 * (fractions[j] + fractions[j + 1]);
            let split = split_at(t)?;
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
        return assemble_lofted(screw, &splits[0], &splits[count], &walls, &fractions, &paths);
    }
}

/// Refuse a lofted wall with an edge of regression: across one the wall's
/// normal turns over, and the band beside it lies inside nearby poses (the
/// sweep folds within a few degrees of the motion, which no probe walk is
/// dense enough to be sure of seeing). Normals are compared between
/// neighbours on every time row and between rows, outside what the end trims
/// take away (`removed`, chart rings) and a few columns off its edge (a fold
/// line meets the end at the cut's own ends).
fn wall_regular(surface: &NurbsSurface, fractions: &[f64], removed: &[&[[f64; 2]]]) -> Result<(), KernelRefusal> {
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
        Ok(du.cross(dv).normalized().ok())
    };
    let mut previous: Option<Vec<Option<Vec3>>> = None;
    for &v in &rows {
        // What a trim takes away is not part of the body.
        let row: Vec<Option<Vec3>> = (0..=COLUMNS)
            .map(|i| {
                let u = i as f64 / COLUMNS as f64;
                if removed.iter().any(|ring| inside_ring(ring, [u, v]) || near_ring(ring, [u, v], 4.0 / COLUMNS as f64)) { Ok(None) } else { normal(u, v) }
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

/// A wall cut back where an end pose bulges past it: the wall parameter range
/// [λa, λb] along the grazing loop where the trim leaves and rejoins it, the
/// cut (from the point at λa to the point at λb, on both the wall and the
/// pose), the loop's pieces before and after it, the wall's side of the trim
/// from λa to λb (`chain`), and the part of the wall's chart it takes away
/// (`removed`, (u, v) points around it).
struct EndTrim {
    lambdas: [f64; 2],
    points: [Vec3; 2],
    cut: NurbsCurve,
    before: NurbsCurve,
    after: NurbsCurve,
    chain: Vec<(TrimPiece, bool, NurbsCurve)>,
    slot: Option<(NurbsCurve, Vec3)>,
    removed: Vec<[f64; 2]>,
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
fn end_trim(screw: &Screw, split: &Split, t: f64, wall: &NurbsSurface, end: usize) -> Result<Option<EndTrim>, KernelRefusal> {
    let v_of = |e: f64| if end == 1 { 1.0 - e } else { e };
    let at = |u: f64, e: f64| -> Result<Vec3, KernelRefusal> { frag(wall.evaluate(u.clamp(0.0, 1.0), v_of(e)), "wall") };
    let f = |u: f64, e: f64| -> Result<f64, KernelRefusal> { Ok(split.distance(screw.invert(t, at(u, e)?)) / (e * e)) };
    // Below `floor` the walls' fit (5e-10 of the scale) swamps d / ε².
    let floor = 1e-3;
    let window = 0.25;
    let noise = 1e-9 * split.scale / (floor * floor);
    // Where the wall dips into the pose just off the end.
    let n = 2000;
    let edge: Vec<f64> = (0..=n).map(|i| f(i as f64 / n as f64, floor)).collect::<Result<_, _>>()?;
    let inside: Vec<bool> = edge.iter().map(|&x| x < -noise).collect();
    if !inside.iter().any(|&x| x) {
        // Nothing near the end; a pocket deeper in is caught below.
        return pierced(&f, floor, window, noise, None);
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
    if crossings.len() != 2 {
        return Err(refuse("end_trim", format!("an end pose bulges past a wall in {} places", crossings.len() / 2)));
    }
    // Trace f = 0 from the first crossing into the wall until it returns to the
    // floor; it must come back at the other crossing.
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
    let mut points: Vec<[f64; 2]> = vec![[crossings[0], floor]];
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
            return Err(refuse("end_trim", "an end pose holds a deep part of a wall".into()));
        }
        if points.len() > 20_000 {
            return Err(refuse("end_trim", "the cut of an end pose does not close".into()));
        }
        heading = [next[0] - last[0], next[1] - last[1]];
        points.push(next);
    }
    let back = *points.last().expect("points");
    if (back[0] - crossings[1]).abs() > 2.0 * step {
        return Err(refuse("end_trim", "the cut of an end pose does not come back where it should".into()));
    }
    points.push([crossings[1], floor]);
    // Down to the end itself along the cut's direction at the floor.
    let to_end = |p: [f64; 2], q: [f64; 2]| -> [f64; 2] {
        let slope = (q[0] - p[0]) / (q[1] - p[1]);
        [p[0] - slope * p[1], 0.0]
    };
    let (first, last) = (to_end(points[0], points[1]), to_end(points[points.len() - 1], points[points.len() - 2]));
    points.insert(0, first);
    points.push(last);
    if points[0][0] > points[points.len() - 1][0] {
        points.reverse();
    }
    pierced(&f, floor, window, noise, Some(&points))?;
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
    let curve_through = |chart_points: &[[f64; 2]], params: &[f64], what: &'static str| -> Result<(NurbsCurve, NurbsCurve), KernelRefusal> {
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
        if worst > 1e-7 * split.scale {
            return Err(refuse("end_trim", format!("a trim curve's pcurve leaves it by {worst:e} between its points")));
        }
        Ok((curve, pcurve))
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
    if let Some((k, ua, b)) = double_start(&at, &points, floor)? {
        let (a_side, b_side) = double_curve(&at, [ua, 0.0], b, window)?;
        // The cut keeps the stretch from the far sheet's point to its end away
        // from where the double curve leaves K.
        let low = ua < points[0][0];
        if !low && ua <= points[points.len() - 1][0] {
            return Err(refuse("end_trim", "the double curve of a wall leaves its end inside the cut".into()));
        }
        let mut kept: Vec<[f64; 2]> = if low { points[k..].to_vec() } else { points[..=k].to_vec() };
        if low { kept[0] = b } else { *kept.last_mut().expect("kept") = b }
        let (cut, cut_pcurve) = curve_through(&kept, &chart_lengths(&kept), "cut")?;
        // Both sides on the continuation's own arc length, so the double
        // curve and its two pcurves share one parameter.
        let mut sigma = vec![0.0];
        for k in 1..a_side.len() {
            let step = (a_side[k][0] - a_side[k - 1][0]).hypot(a_side[k][1] - a_side[k - 1][1]).hypot((b_side[k][0] - b_side[k - 1][0]).hypot(b_side[k][1] - b_side[k - 1][1]));
            sigma.push(sigma[k - 1] + step.max(1e-15));
        }
        let (slot, pa) = curve_through(&a_side, &sigma, "double_curve")?;
        let (_, pb) = curve_through(&b_side, &sigma, "double_curve")?;
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
        return Ok(Some(EndTrim { lambdas, points: points3, cut, before: piece(0.0, lambdas[0])?, after: piece(lambdas[1], 1.0)?, chain, slot: Some((slot, tip)), removed }));
    }
    let (cut, wall_pcurve) = curve_through(&points, &chart_lengths(&points), "cut")?;
    let (la, lb) = (points[0][0], points[points.len() - 1][0]);
    let removed: Vec<[f64; 2]> = points.iter().map(|&p| chart(p)).collect();
    Ok(Some(EndTrim {
        lambdas: [la, lb],
        points: [at(la, 0.0)?, at(lb, 0.0)?],
        cut,
        before: piece(0.0, la)?,
        after: piece(lb, 1.0)?,
        chain: vec![(TrimPiece::Cut, true, wall_pcurve)],
        slot: None,
        removed,
    }))
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
fn double_curve(at: &dyn Fn(f64, f64) -> Result<Vec3, KernelRefusal>, a: [f64; 2], b: [f64; 2], window: f64) -> Result<(Vec<[f64; 2]>, Vec<[f64; 2]>), KernelRefusal> {
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
    let step = 2e-3;
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
            return Err(refuse("end_trim", format!("the double curve of a wall cannot be followed (sides {separation:e} apart)")));
        };
        if [y[0], y[2]].iter().any(|&u| u <= 1e-3 || u >= 1.0 - 1e-3) {
            return Err(refuse("end_trim_corner", "the double curve of a wall reaches a corner of its grazing loop".into()));
        }
        if y[1] > window || y[3] > window || y[1] < 0.0 || y[3] < 0.0 {
            return Err(refuse("end_trim", "the double curve of a wall runs deep into the motion or back past its end".into()));
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

/// Refuse a wall the end pose holds a part of that the cut (if any) does not
/// enclose: a pocket of the wall inside the pose away from the end.
fn pierced(
    f: &dyn Fn(f64, f64) -> Result<f64, KernelRefusal>,
    floor: f64,
    window: f64,
    noise: f64,
    cut: Option<&[[f64; 2]]>,
) -> Result<Option<EndTrim>, KernelRefusal> {
    let enclosed = |q: [f64; 2]| -> bool {
        let Some(ring) = cut else { return false };
        let mut crossings = 0;
        for k in 0..ring.len() {
            let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
            if (a[1] > q[1]) != (b[1] > q[1]) && q[0] < a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
                crossings += 1;
            }
        }
        crossings % 2 == 1
    };
    for i in 1..200 {
        for j in 1..=40 {
            let (u, e) = (i as f64 / 200.0, floor + (window - floor) * (j as f64 / 40.0).powi(2));
            // d itself against the fit's noise: f grows as 1 / ε² near the end.
            if f(u, e)? * e * e < -noise * floor * floor && !enclosed([u, e]) {
                return Err(refuse("end_trim", "an end pose holds a part of a wall away from its end".into()));
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
) -> Result<BrepSolid, KernelRefusal> {
    let mut shell = Shell::new();
    let ends = [(first, screw.t0), (last, screw.t1)];
    // Where an end pose bulges past a wall near its grazing loop (a saddle side
    // curving past the wall it touches), cut the wall back to where it
    // re-enters the pose and carry the end cap across the loop to there.
    let mut trims: HashMap<(usize, usize), EndTrim> = HashMap::new();
    for (index, (_, _, surface, _, _)) in walls.iter().enumerate() {
        for end in 0..2 {
            if let Some(trim) = end_trim(screw, ends[end].0, ends[end].1, surface, end)? {
                trims.insert((index, end), trim);
            }
        }
        let removed: Vec<&[[f64; 2]]> = (0..2).filter_map(|end| trims.get(&(index, end)).map(|trim| trim.removed.as_slice())).collect();
        wall_regular(surface, times, &removed)?;
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
    let mut trim_edges: HashMap<(usize, usize), [u64; 4]> = HashMap::new();
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
                    let (va, vb) = (shell.vertex(trim.points[0]), shell.vertex(trim.points[1]));
                    let ka = shell.edge(trim.before.clone(), vf, va, false)?;
                    let cut = shell.edge(trim.cut.clone(), va, vb, false)?;
                    let kb = shell.edge(trim.after.clone(), vb, vt, false)?;
                    // The double curve runs from where it leaves K (at the
                    // low end of the trim if its chain starts with it) to the
                    // swallowtail's point.
                    let slot = match &trim.slot {
                        Some((curve, tip)) => {
                            let from = if trim.chain[0].0 == TrimPiece::Slot { va } else { vb };
                            let tip = shell.vertex(*tip);
                            shell.edge(curve.clone(), from, tip, false)?
                        }
                        None => u64::MAX,
                    };
                    trim_edges.insert((index, end), [ka, cut, kb, slot]);
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
                    if let (Some(trim), Some(edges)) = (trims.get(&(index, end)), trim_edges.get(&(index, end))) {
                        let Some(chart) = cap.chart.as_ref() else {
                            return Err(refuse("end_trim", "an end pose bulges past a wall beside a cap without a chart".into()));
                        };
                        // The cap runs the tool's curve forward or back; the
                        // pieces run the wall's way.
                        let along = *forward != walls[index].1;
                        let pieces = [(edges[0], &trim.before), (edges[1], &trim.cut), (edges[2], &trim.after)];
                        let order: Vec<usize> = if along { vec![0, 1, 2] } else { vec![2, 1, 0] };
                        for k in order {
                            coedges.push((pieces[k].0, along, charted_pcurve(chart, screw, t, pieces[k].1, along)?));
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
    for (index, (c, back_forward, surface, _, _)) in walls.iter().enumerate() {
        let tool_curve = &first.curves[*c];
        let (from, to) = if *back_forward { (tool_curve.end, tool_curve.start) } else { (tool_curve.start, tool_curve.end) };
        let mut coedges = Vec::new();
        match (trims.get(&(index, 0)), trim_edges.get(&(index, 0))) {
            (Some(trim), Some(edges)) => {
                coedges.push((edges[0], true, uv_line([0.0, 0.0], [trim.lambdas[0], 0.0])?));
                for (piece, forward, pcurve) in &trim.chain {
                    let edge = if *piece == TrimPiece::Cut { edges[1] } else { edges[3] };
                    coedges.push((edge, *forward, pcurve.clone()));
                }
                coedges.push((edges[2], true, uv_line([trim.lambdas[1], 0.0], [1.0, 0.0])?));
            }
            _ => coedges.push((edge_at[&(0, *c)], !*back_forward, uv_line([0.0, 0.0], [1.0, 0.0])?)),
        }
        coedges.push((orbits[&to], true, uv_line([1.0, 0.0], [1.0, 1.0])?));
        match (trims.get(&(index, 1)), trim_edges.get(&(index, 1))) {
            (Some(trim), Some(edges)) => {
                coedges.push((edges[2], false, uv_line([1.0, 1.0], [trim.lambdas[1], 1.0])?));
                for (piece, forward, pcurve) in trim.chain.iter().rev() {
                    let edge = if *piece == TrimPiece::Cut { edges[1] } else { edges[3] };
                    coedges.push((edge, !*forward, frag(pcurve.reversed(), "reversed")?));
                }
                coedges.push((edges[0], false, uv_line([trim.lambdas[0], 1.0], [0.0, 1.0])?));
            }
            _ => coedges.push((edge_at[&(1, *c)], *back_forward, uv_line([1.0, 1.0], [0.0, 1.0])?)),
        }
        coedges.push((orbits[&from], false, uv_line([0.0, 1.0], [0.0, 0.0])?));
        shell.face(surface.clone(), coedges);
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
    check_single_pass(screw, first, &body)?;
    Ok(body)
}

/// Is the world point y inside the moving tool at some time of the motion?
/// Steps t by the distance over a bound on the speed of any tool point near y.
fn covered(screw: &Screw, split: &Split, y: Vec3, margin: f64) -> Result<bool, KernelRefusal> {
    let reach = y.sub(screw.a).length() + split.scale + screw.extent;
    let speed = (screw.omega.abs() * reach + screw.rate_bound()).max(1e-300);
    let floor = (screw.t1 - screw.t0) * 1e-7;
    let mut t = screw.t0;
    let mut steps = 0usize;
    loop {
        let d = split.distance(screw.invert(t, y));
        if d < -margin {
            return Ok(true);
        }
        if t >= screw.t1 {
            return Ok(false);
        }
        if steps > 2_000_000 {
            return Err(refuse("probe_budget", "the one-pass check could not walk the motion past a point near the sweep".into()));
        }
        t = (t + ((d.abs() + margin) / speed).max(floor)).min(screw.t1);
        steps += 1;
    }
}

/// Refuse a sweep that covers points just outside its own faces.
fn check_single_pass(screw: &Screw, split: &Split, body: &BrepSolid) -> Result<(), KernelRefusal> {
    let step = 1e-6 * split.scale;
    let margin = 1e-9 * split.scale;
    for face in body.shells.iter().flat_map(|s| &s.faces) {
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
            if covered(screw, split, p.add(n.scale(step)), margin)? {
                return Err(refuse("folds", format!("the sweep passes over a point twice (just outside its own face at {p:?}); not one pass")));
            }
        }
    }
    Ok(())
}

