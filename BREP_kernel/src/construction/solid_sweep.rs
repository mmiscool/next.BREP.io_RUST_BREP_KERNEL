//! The sweep of a SOLID: everything a tool passes through as it moves, inside a
//! region the caller names (the workpiece it is cut from).
//!
//! ## What is built, and why it reduces to a plane
//!
//! The boundary of a swept solid is an envelope: at each instant the tool's
//! surface touches the sweep along its characteristic curve (where the motion's
//! velocity is tangent to the surface), and the sweep's wall is the union of
//! those curves. In general that wall has no simpler description. In the three
//! classes built here it is invariant under a motion, so the region is a planar
//! region carried by that motion:
//!
//! | class | tool | invariance | planar region | solid |
//! |---|---|---|---|---|
//! | SCREW | a solid of revolution | the screw | the section square to the screw axis | a loft through screwed copies |
//! | TURN | a solid of revolution | the turn | the meridian section | a revolve |
//! | PLANAR | a prism | the prism's direction | the union of the moving outline | an extrude |
//!
//! For a SCREW (axial advance `c` per radian) the reduced coordinates of a point
//! are `(ℓ, o) = (ρ, α − z/c)`, both unchanged by the screw; for a TURN they are
//! `(ρ, z)`; for a PLANAR motion `(ρ, α)` in the workpiece's frame. In each the
//! region is described per LEVEL `ℓ` as a set of intervals of `o`.
//!
//! ## The region engine
//!
//! At a level the interval ends are local extrema of `o` over the images of the
//! tool's boundary: on a face patch `S(u, v)` (a meridian piece revolved; an
//! outline piece carried through time) Newton solves `ℓ(S) = ℓ₀`,
//! `det(∇o, ∇ℓ) = 0` from seeds on a grid; on a sharp edge (a meridian corner
//! revolved; an outline vertex carried through time; the outline at either end of
//! the motion) the ends are the roots of `ℓ = ℓ₀`. These are CANDIDATES; a
//! membership oracle (does the point's preimage under the motion meet the tool,
//! for some time in the range, by the tool's signed distance) classifies the
//! gaps between them, so the intervals are exact at every level. This is what
//! survives where the characteristic degenerates: a grinding wheel set along its
//! helix has a whole circle of characteristic points, which tracing the
//! characteristic by parameter cannot follow, but the level extremes can.
//!
//! The levels are scanned for EVENTS — an interval born at a tip, two lobes
//! merging, an interval end changing the boundary piece it lies on — solved by
//! bisection. Between events each interval end is a smooth function of the
//! level, sampled at levels clustered at both ends (`ℓ − ℓa ∝ σ²`, so a square
//! root tip is smooth in `σ`) and interpolated as its own cubic edge. Along
//! each event level the boundary runs along the level itself (an arc, or a
//! straight line for the turn) where the intervals above and below differ.
//!
//! ## The contract
//!
//! Each returned body equals the sweep inside the clip region: levels in
//! `clip.radius`, axial positions in `clip.axial`, angles in `clip.angle`. The
//! caller cuts a workpiece that lies inside that region. A screw or turn whose
//! first or last position reaches the clip region is refused, because there the
//! sweep is not invariant; a planar sweep includes its end positions exactly.
//!
//! FOLDS. Two lobes of a space can merge where one face's envelope turns back
//! (the plus-cut wheel's arm tip, ρ = 20). Where the face's fold Newton reaches
//! the point it is a critical point like a tip; where it cannot (that envelope's
//! level varies by 1e-4 along it, too flat for nested differences), the merge is
//! located from the gap's own √ law: gap² fitted as a quadratic in the level at
//! three levels just below, its root the merge level (4e-11 from ρ = 20).
//!
//! ## Not built yet (refused, never approximated)
//!
//! - A source whose level is constant along it (a vertex turning about the
//!   clip's own axis) reads as roots everywhere; keep the clip's axis off such
//!   a turn's axis.
//! - Motions outside the three classes (two turns about skew axes: the
//!   tumbling torus).
use crate::{
    extrude_profile_brep, interpolate_curve, loft_profile_brep, make_arc, make_line,
    revolve_profile_brep, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, Vec3,
};
use std::f64::consts::{PI, TAU};

/// One piece of a planar outline, in the outline's own 2D coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OutlinePiece {
    /// The segment from `from` to `to`.
    Line { from: [f64; 2], to: [f64; 2] },
    /// The arc about `centre` of `radius`, from angle `start` to angle `end`
    /// (radians; `end < start` runs clockwise). A full circle is `0 → 2π`.
    Arc { centre: [f64; 2], radius: f64, start: f64, end: f64 },
}

impl OutlinePiece {
    fn point(&self, s: f64) -> [f64; 2] {
        match *self {
            OutlinePiece::Line { from, to } => {
                [from[0] + (to[0] - from[0]) * s, from[1] + (to[1] - from[1]) * s]
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                let angle = start + (end - start) * s;
                [centre[0] + radius * angle.cos(), centre[1] + radius * angle.sin()]
            }
        }
    }

    fn is_full_circle(&self) -> bool {
        matches!(*self, OutlinePiece::Arc { start, end, .. } if ((end - start).abs() - TAU).abs() < 1e-12)
    }

    /// Unsigned distance from `p` to the piece.
    fn distance(&self, p: [f64; 2]) -> f64 {
        match *self {
            OutlinePiece::Line { from, to } => {
                let d = [to[0] - from[0], to[1] - from[1]];
                let length2 = d[0] * d[0] + d[1] * d[1];
                let t = if length2 > 0.0 {
                    (((p[0] - from[0]) * d[0] + (p[1] - from[1]) * d[1]) / length2).clamp(0.0, 1.0)
                } else {
                    0.0
                };
                let q = [from[0] + d[0] * t - p[0], from[1] + d[1] * t - p[1]];
                q[0].hypot(q[1])
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                let v = [p[0] - centre[0], p[1] - centre[1]];
                let angle = v[1].atan2(v[0]);
                if self.is_full_circle() || angle_within(angle, start, end) {
                    (v[0].hypot(v[1]) - radius).abs()
                } else {
                    let a = self.point(0.0);
                    let b = self.point(1.0);
                    (a[0] - p[0]).hypot(a[1] - p[1]).min((b[0] - p[0]).hypot(b[1] - p[1]))
                }
            }
        }
    }

    /// Crossings of the ray from `p` towards +x with the piece (for the parity
    /// inside test). Half-open in y so a vertex is counted once.
    fn ray_crossings(&self, p: [f64; 2]) -> usize {
        match *self {
            OutlinePiece::Line { from, to } => {
                let (a, b) = (from, to);
                if (a[1] > p[1]) != (b[1] > p[1]) {
                    let x = a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
                    usize::from(x > p[0])
                } else {
                    0
                }
            }
            OutlinePiece::Arc { centre, radius, start, end } => {
                let dy = p[1] - centre[1];
                if dy.abs() >= radius {
                    return 0;
                }
                let dx = (radius * radius - dy * dy).sqrt();
                let mut count = 0;
                for x in [centre[0] - dx, centre[0] + dx] {
                    if x <= p[0] {
                        continue;
                    }
                    let angle = dy.atan2(x - centre[0]);
                    if self.is_full_circle() || angle_within(angle, start, end) {
                        // Half-open at the arc's own ends, like the segment rule:
                        // count an end only where the arc leaves upwards.
                        count += 1;
                    }
                }
                count
            }
        }
    }
}

/// Is `angle` on the arc that runs from `start` to `end` (either sense)?
fn angle_within(angle: f64, start: f64, end: f64) -> bool {
    let (low, high) = if end >= start { (start, end) } else { (end, start) };
    let shifted = low + (angle - low).rem_euclid(TAU);
    shifted <= high
}

fn scale_of(outline: &[OutlinePiece]) -> f64 {
    outline
        .iter()
        .flat_map(|piece| [piece.point(0.0), piece.point(1.0)])
        .map(|p| p[0].abs().max(p[1].abs()))
        .fold(1.0f64, f64::max)
}

/// Signed distance to a closed outline (negative inside).
pub(crate) fn outline_signed_distance(outline: &[OutlinePiece], p: [f64; 2]) -> f64 {
    let distance = outline
        .iter()
        .map(|piece| piece.distance(p))
        .fold(f64::INFINITY, f64::min);
    let crossings: usize = outline.iter().map(|piece| piece.ray_crossings(p)).sum();
    if crossings % 2 == 1 {
        -distance
    } else {
        distance
    }
}

fn validate_outline(outline: &[OutlinePiece], what: &str) -> Result<(), KernelRefusal> {
    if outline.is_empty() {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "outline_empty",
            format!("sweepSolidWithin: the {what} outline is empty"),
        ));
    }
    let scale = outline
        .iter()
        .flat_map(|piece| [piece.point(0.0), piece.point(1.0)])
        .map(|p| p[0].abs().max(p[1].abs()))
        .fold(1.0f64, f64::max);
    for (index, piece) in outline.iter().enumerate() {
        let next = &outline[(index + 1) % outline.len()];
        let a = piece.point(1.0);
        let b = next.point(0.0);
        if (a[0] - b[0]).hypot(a[1] - b[1]) > 1e-9 * scale {
            return Err(KernelRefusal::input(
                KernelStage::Collect,
                "outline_open",
                format!("sweepSolidWithin: the {what} outline is not closed after piece {index}"),
            ));
        }
    }
    Ok(())
}

/// The moving tool.
#[derive(Clone, Debug)]
pub enum SweptTool {
    /// A solid of revolution: the region `meridian` bounds in the half-plane
    /// spanned by `radial` (first coordinate, the distance from the axis, ≥ 0)
    /// and `axis_direction` (second coordinate), turned about the axis through
    /// `axis_origin`.
    Revolved {
        axis_origin: Vec3,
        axis_direction: Vec3,
        radial: Vec3,
        meridian: Vec<OutlinePiece>,
    },
    /// A prism: the region `outline` bounds in the plane through `origin`
    /// spanned by `x_axis` and `normal × x_axis`, extended along `normal` from
    /// `from` to `to`.
    Prism {
        origin: Vec3,
        normal: Vec3,
        x_axis: Vec3,
        outline: Vec<OutlinePiece>,
        from: f64,
        to: f64,
    },
}

/// One elementary motion, linear in the motion parameter `t`: a turn of
/// `turn·t` radians, right-handed about the axis through `axis_origin` along
/// `axis_direction`, together with an advance of `advance·t` along it. A turn
/// alone is `advance = 0`, a slide alone is `turn = 0`.
#[derive(Clone, Copy, Debug)]
pub struct ScrewStep {
    pub axis_origin: Vec3,
    pub axis_direction: Vec3,
    pub turn: f64,
    pub advance: f64,
}

/// A rigid motion: the steps applied in order (the first to the tool first),
/// for `t` from `from` to `to`.
#[derive(Clone, Debug)]
pub struct SolidMotion {
    pub steps: Vec<ScrewStep>,
    pub from: f64,
    pub to: f64,
}

/// The region the result must agree with the sweep in, about an axis: levels
/// (distance from the axis) in `radius`, axial positions in `axial` (measured
/// from `axis_origin`), and polar angles in `angle` (from `reference`,
/// right-handed about the axis), when given.
#[derive(Clone, Copy, Debug)]
pub struct SweepClip {
    pub axis_origin: Vec3,
    pub axis_direction: Vec3,
    pub reference: Vec3,
    pub radius: [f64; 2],
    pub axial: [f64; 2],
    pub angle: Option<[f64; 2]>,
}

/// The prefix of every refusal of this operation.
pub const SOLID_SWEEP_REFUSAL: &str = "sweepSolidWithin";

/// Words of the refusal for a level the sweep covers at every angle.
const WHOLE_LEVEL: &str = "the sweep covers every angle at level";

/// Turns of a thread each row of faces carries (measured: shorter rows cost more
/// meshing, longer ones more edge reading).
const TURNS_PER_ROW: f64 = 2.0;

/// `BREP_SWEEP_THREAD_ROWS` (default on; `=0` one body-long face per wall and the
/// boolean for the clip): a thread is built a few turns at a time, since readings
/// over a face cost the square of its length.
fn thread_rows_on() -> bool {
    crate::switch!("BREP_SWEEP_THREAD_ROWS", true)
}

/// `BREP_SWEEP_WHOLE_LEVEL` (default on; `=0` restores the refusal): a screw's
/// sweep whose turns meet at a crest, built as the clip less what stays.
fn whole_level_on() -> bool {
    crate::switch!("BREP_SWEEP_WHOLE_LEVEL", true)
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Classify, what, format!("{SOLID_SWEEP_REFUSAL}: {message}"))
}

fn rotate(x: Vec3, origin: Vec3, axis: Vec3, angle: f64) -> Vec3 {
    origin.add(x.sub(origin).rotate_about(axis, angle))
}

impl ScrewStep {
    fn apply(&self, t: f64, x: Vec3, axis: Vec3) -> Vec3 {
        rotate(x, self.axis_origin, axis, self.turn * t).add(axis.scale(self.advance * t))
    }
    fn invert(&self, t: f64, x: Vec3, axis: Vec3) -> Vec3 {
        rotate(x.sub(axis.scale(self.advance * t)), self.axis_origin, axis, -self.turn * t)
    }
}

/// Orthonormal frame of the clip: (ex, ey, ez).
#[derive(Clone, Copy, Debug)]
struct Frame {
    origin: Vec3,
    ex: Vec3,
    ey: Vec3,
    ez: Vec3,
}

impl Frame {
    fn local(&self, x: Vec3) -> [f64; 3] {
        let d = x.sub(self.origin);
        [d.dot(self.ex), d.dot(self.ey), d.dot(self.ez)]
    }
    fn world(&self, p: [f64; 3]) -> Vec3 {
        self.origin
            .add(self.ex.scale(p[0]))
            .add(self.ey.scale(p[1]))
            .add(self.ez.scale(p[2]))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
enum Class {
    /// Axial advance per radian of the screw, in the clip frame (nonzero).
    Screw(f64),
    Turn,
    Planar,
}

/// A tool-boundary source: a face patch over a 2D parameter box, or an edge
/// over a 1D parameter interval. `periodic` marks a parameter that wraps.
#[derive(Clone, Copy, Debug)]
enum Source {
    /// Revolved meridian piece `piece`, parameters (s ∈ [0,1], φ ∈ [0,2π)).
    RevolvedFace { piece: usize },
    /// Revolved meridian corner at the start of piece `piece`, parameter φ.
    RevolvedCorner { piece: usize },
    /// Prism outline piece `piece` carried through time: (s ∈ [0,1], t).
    PrismFace { piece: usize },
    /// Prism outline vertex at the start of piece `piece`, parameter t.
    PrismVertex { piece: usize },
    /// Prism outline piece `piece` at the motion's first (`end = false`) or last
    /// position, parameter s.
    PrismEnd { piece: usize, end: bool },
    /// Under a screw, at the tool's own place (the screw's reduction is invariant
    /// along it): side face of outline piece `piece`, (s ∈ [0,1], height).
    ScrewPrismSide { piece: usize },
    /// Under a screw: the upright edge at the start of piece `piece`, parameter height.
    ScrewPrismEdge { piece: usize },
    /// Under a screw: outline piece `piece` at the prism's low (`end = false`) or
    /// high end, parameter s. An end cap itself maps one-to-one into the reduced
    /// plane (polar coordinates at a fixed height), so only its rim can bound.
    ScrewPrismRim { piece: usize, end: bool },
}

struct Engine<'a> {
    tool: &'a SweptTool,
    motion: &'a SolidMotion,
    axes: Vec<Vec3>,
    frame: Frame,
    class: Class,
    clip: &'a SweepClip,
    /// The clip was narrowed to a crest level: below it some material is left.
    crest_gap: bool,
    sources: Vec<Source>,
    /// Per face source: its parameter grid (nu × nv) of reduced coordinates.
    grids: Vec<Option<Grid>>,
    /// Per source: its critical points of the level, as (level, o).
    critical: Vec<Vec<(f64, f64)>>,
    /// Per face source: the parameters of its critical points (tips), which
    /// seed the extremum Newton where the level's contour is smaller than a cell.
    critical_params: Vec<Vec<(f64, f64)>>,
    /// Angular `o` (wraps at 2π) or not.
    angular: bool,
    /// Prism height (along its normal) the planar sources are evaluated at.
    prism_height: f64,
    scale: f64,
    /// `intervals_at` by level bits: the sampling revisits levels (a refined
    /// band keeps every level of the coarser one).
    cache: std::cell::RefCell<std::collections::HashMap<u64, Vec<Interval>>>,
    /// The motion ends where it began as a set (the plus wheel: one turn of the
    /// wheel, one and a half of the tool), so there are no end positions; piece
    /// `k` at the end continues as piece `wrap[k]` at the start.
    wrap: Option<Vec<usize>>,
    /// Debug counters: (levels evaluated, oracle calls, oracle steps, seconds in
    /// candidates, seconds in the oracle).
    counters: std::cell::Cell<(usize, usize, usize, f64, f64)>,
}

struct Grid {
    nu: usize,
    nv: usize,
    u: [f64; 2],
    v: [f64; 2],
    v_periodic: bool,
    values: Vec<(f64, f64)>,
}

/// One interval of the region at a level, with the sources of its ends.
#[derive(Clone, Copy, Debug)]
struct Interval {
    lo: f64,
    hi: f64,
    lo_source: usize,
    hi_source: usize,
}

fn wrap_pi(angle: f64) -> f64 {
    let wrapped = (angle + PI).rem_euclid(TAU) - PI;
    wrapped
}

impl<'a> Engine<'a> {
    fn motion_apply(&self, t: f64, x: Vec3) -> Vec3 {
        self.motion
            .steps
            .iter()
            .zip(&self.axes)
            .fold(x, |x, (step, axis)| step.apply(t, x, *axis))
    }

    fn motion_invert(&self, t: f64, x: Vec3) -> Vec3 {
        self.motion
            .steps
            .iter()
            .zip(&self.axes)
            .rev()
            .fold(x, |x, (step, axis)| step.invert(t, x, *axis))
    }

    /// Reduced coordinates of a clip-frame point.
    fn reduce(&self, p: [f64; 3]) -> (f64, f64) {
        let level = p[0].hypot(p[1]);
        match self.class {
            Class::Screw(c) => (level, p[1].atan2(p[0]) - p[2] / c),
            Class::Turn => (level, p[2]),
            Class::Planar => (level, p[1].atan2(p[0])),
        }
    }

    fn tool_signed_distance(&self, x: Vec3) -> f64 {
        match self.tool {
            SweptTool::Revolved { axis_origin, axis_direction, meridian, .. } => {
                let axis = axis_direction.normalized_or(*axis_direction);
                let d = x.sub(*axis_origin);
                let e = d.dot(axis);
                let radial = d.sub(axis.scale(e)).length();
                outline_signed_distance(meridian, [radial, e])
            }
            SweptTool::Prism { origin, normal, x_axis, outline, from, to } => {
                let n = normal.normalized_or(*normal);
                let xa = x_axis.normalized_or(*x_axis);
                let ya = n.cross(xa);
                let d = x.sub(*origin);
                let h = d.dot(n);
                let planar = outline_signed_distance(outline, [d.dot(xa), d.dot(ya)]);
                planar.max(from - h).max(h - to)
            }
        }
    }

    /// The world point of a source at its parameters (at the tool's own place
    /// for revolved tools; carried by the motion for prisms).
    fn source_point(&self, source: Source, u: f64, v: f64) -> Vec3 {
        match (self.tool, source) {
            (SweptTool::Revolved { axis_origin, axis_direction, radial, meridian }, _) => {
                let axis = axis_direction.normalized_or(*axis_direction);
                let r0 = radial.sub(axis.scale(radial.dot(axis))).normalized_or(*radial);
                let r1 = axis.cross(r0);
                let (m, phi) = match source {
                    Source::RevolvedFace { piece } => (meridian[piece].point(u), v),
                    Source::RevolvedCorner { piece } => (meridian[piece].point(0.0), u),
                    _ => unreachable!(),
                };
                axis_origin
                    .add(r0.scale(m[0] * phi.cos()))
                    .add(r1.scale(m[0] * phi.sin()))
                    .add(axis.scale(m[1]))
            }
            (SweptTool::Prism { origin, normal, x_axis, outline, from, to }, _) => {
                let n = normal.normalized_or(*normal);
                let xa = x_axis.normalized_or(*x_axis);
                let ya = n.cross(xa);
                let at_own_place = match source {
                    Source::ScrewPrismSide { piece } => Some((outline[piece].point(u), v)),
                    Source::ScrewPrismEdge { piece } => Some((outline[piece].point(0.0), u)),
                    Source::ScrewPrismRim { piece, end } => Some((outline[piece].point(u), if end { *to } else { *from })),
                    _ => None,
                };
                if let Some((q, h)) = at_own_place {
                    return origin.add(xa.scale(q[0])).add(ya.scale(q[1])).add(n.scale(h));
                }
                let (q, t) = match source {
                    Source::PrismFace { piece } => (outline[piece].point(u), v),
                    Source::PrismVertex { piece } => (outline[piece].point(0.0), u),
                    Source::PrismEnd { piece, end } => (
                        outline[piece].point(u),
                        if end { self.motion.to } else { self.motion.from },
                    ),
                    _ => unreachable!(),
                };
                let x = origin
                    .add(xa.scale(q[0]))
                    .add(ya.scale(q[1]))
                    .add(n.scale(self.prism_height));
                self.motion_apply(t, x)
            }
        }
    }

    fn source_reduced(&self, source: Source, u: f64, v: f64) -> (f64, f64) {
        self.reduce(self.frame.local(self.source_point(source, u, v)))
    }

    fn is_face(source: Source) -> bool {
        matches!(source, Source::RevolvedFace { .. } | Source::PrismFace { .. } | Source::ScrewPrismSide { .. })
    }

    /// Parameter domain of a source: (u range, v range, v periodic).
    fn domain(&self, source: Source) -> ([f64; 2], [f64; 2], bool) {
        let time = [self.motion.from, self.motion.to];
        match source {
            Source::RevolvedFace { .. } => ([0.0, 1.0], [0.0, TAU], true),
            Source::RevolvedCorner { .. } => ([0.0, TAU], [0.0, 0.0], false),
            Source::PrismFace { .. } => ([0.0, 1.0], time, false),
            Source::PrismVertex { .. } => (time, [0.0, 0.0], false),
            Source::PrismEnd { .. } => ([0.0, 1.0], [0.0, 0.0], false),
            Source::ScrewPrismSide { .. } => ([0.0, 1.0], self.prism_extent(), false),
            Source::ScrewPrismEdge { .. } => (self.prism_extent(), [0.0, 0.0], false),
            Source::ScrewPrismRim { .. } => ([0.0, 1.0], [0.0, 0.0], false),
        }
    }

    fn prism_extent(&self) -> [f64; 2] {
        match self.tool {
            SweptTool::Prism { from, to, .. } => [*from, *to],
            SweptTool::Revolved { .. } => [0.0, 0.0],
        }
    }

    /// Is the face's first parameter periodic (a full circle in the meridian or
    /// outline, which has no corner to bound it)?
    fn u_periodic(&self, source: Source) -> bool {
        match (self.tool, source) {
            (SweptTool::Revolved { meridian, .. }, Source::RevolvedFace { piece }) => meridian[piece].is_full_circle(),
            (SweptTool::Prism { outline, .. }, Source::PrismFace { piece } | Source::ScrewPrismSide { piece }) => outline[piece].is_full_circle(),
            _ => false,
        }
    }

    fn o_difference(&self, a: f64, b: f64) -> f64 {
        if self.angular {
            wrap_pi(a - b)
        } else {
            a - b
        }
    }

    /// Newton on {ℓ = level, det(∇o, ∇ℓ) = 0} over a face source.
    fn face_extremum(&self, source: Source, level: f64, mut u: f64, mut v: f64) -> Option<(f64, f64, f64)> {
        let (ur, vr, periodic) = self.domain(source);
        let hu = 1e-6 * (ur[1] - ur[0]).max(1e-3);
        let hv = 1e-6 * (vr[1] - vr[0]).max(1e-3);
        let residual = |u: f64, v: f64| -> [f64; 2] {
            let (l, o) = self.source_reduced(source, u, v);
            let (lu1, ou1) = self.source_reduced(source, u + hu, v);
            let (lu0, ou0) = self.source_reduced(source, u - hu, v);
            let (lv1, ov1) = self.source_reduced(source, u, v + hv);
            let (lv0, ov0) = self.source_reduced(source, u, v - hv);
            let (lu, lv) = ((lu1 - lu0) / (2.0 * hu), (lv1 - lv0) / (2.0 * hv));
            let ou = self.o_difference(ou1, ou0) / (2.0 * hu);
            let ov = self.o_difference(ov1, ov0) / (2.0 * hv);
            // det scaled by the level's gradient so its size is that of ∇o.
            let norm = lu.hypot(lv).max(1e-300);
            let _ = o;
            [l - level, (ou * lv - ov * lu) / norm]
        };
        let mut f = residual(u, v);
        for _ in 0..60 {
            let (du_step, dv_step) = (hu * 10.0, hv * 10.0);
            let fu1 = residual(u + du_step, v);
            let fu0 = residual(u - du_step, v);
            let fv1 = residual(u, v + dv_step);
            let fv0 = residual(u, v - dv_step);
            let j = [
                [(fu1[0] - fu0[0]) / (2.0 * du_step), (fv1[0] - fv0[0]) / (2.0 * dv_step)],
                [(fu1[1] - fu0[1]) / (2.0 * du_step), (fv1[1] - fv0[1]) / (2.0 * dv_step)],
            ];
            let det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
            if !det.is_finite() || det.abs() < 1e-300 {
                return None;
            }
            let du = -(f[0] * j[1][1] - f[1] * j[0][1]) / det;
            let dv = -(j[0][0] * f[1] - j[1][0] * f[0]) / det;
            let norm0 = f[0].hypot(f[1]);
            let mut step = 1.0;
            let mut accepted = false;
            while step > 1e-4 {
                let (nu, nv) = (u + step * du, v + step * dv);
                let candidate = residual(nu, nv);
                if candidate[0].hypot(candidate[1]) < norm0 || norm0 < 1e-14 * self.scale {
                    u = nu;
                    v = nv;
                    f = candidate;
                    accepted = true;
                    break;
                }
                step *= 0.5;
            }
            if !accepted {
                break;
            }
            if (du * step).abs() < 1e-15 * (ur[1] - ur[0]) && (dv * step).abs() < 1e-15 * (vr[1] - vr[0]).max(1.0) {
                break;
            }
        }
        if periodic {
            v = vr[0] + (v - vr[0]).rem_euclid(vr[1] - vr[0]);
        }
        if self.u_periodic(source) {
            u = ur[0] + (u - ur[0]).rem_euclid(ur[1] - ur[0]);
        }
        let tolerance = 1e-9;
        if u < ur[0] - tolerance || u > ur[1] + tolerance || (!periodic && (v < vr[0] - tolerance || v > vr[1] + tolerance)) {
            return None;
        }
        if f[0].abs() > 1e-10 * self.scale || f[1].abs() > 1e-6 {
            return None;
        }
        let (l, o) = self.source_reduced(source, u.clamp(ur[0], ur[1]), v);
        Some((l, o, u))
    }

    fn build_grid(&self, source: Source) -> Grid {
        let (u, v, v_periodic) = self.domain(source);
        let (nu, nv) = match source {
            Source::RevolvedFace { .. } => (64, 256),
            Source::ScrewPrismSide { .. } => (64, 64),
            _ => {
                let turn = self
                    .motion
                    .steps
                    .iter()
                    .map(|step| step.turn.abs())
                    .fold(0.0, f64::max)
                    * (self.motion.to - self.motion.from).abs();
                (48, ((turn * 128.0) as usize).clamp(256, 4096))
            }
        };
        let mut values = Vec::with_capacity(nu * nv);
        for i in 0..nu {
            let uu = u[0] + (u[1] - u[0]) * i as f64 / (nu - 1) as f64;
            for j in 0..nv {
                let vv = if v_periodic {
                    v[0] + (v[1] - v[0]) * j as f64 / nv as f64
                } else {
                    v[0] + (v[1] - v[0]) * j as f64 / (nv - 1) as f64
                };
                values.push(self.source_reduced(source, uu, vv));
            }
        }
        Grid { nu, nv, u, v, v_periodic, values }
    }

    fn grid_param(grid: &Grid, i: usize, j: usize) -> (f64, f64) {
        let u = grid.u[0] + (grid.u[1] - grid.u[0]) * i as f64 / (grid.nu - 1) as f64;
        let v = if grid.v_periodic {
            grid.v[0] + (grid.v[1] - grid.v[0]) * j as f64 / grid.nv as f64
        } else {
            grid.v[0] + (grid.v[1] - grid.v[0]) * j as f64 / (grid.nv - 1) as f64
        };
        (u, v)
    }

    /// Candidates of one source at a level: (o, source index).
    fn source_candidates(&self, index: usize, level: f64, out: &mut Vec<(f64, usize)>) {
        let source = self.sources[index];
        if Self::is_face(source) {
            let grid = self.grids[index].as_ref().expect("face grid");
            // Crossings of the level along grid edges, then seeds where o is a
            // local extremum among the crossings of neighbouring cells.
            let at = |i: usize, j: usize| grid.values[i * grid.nv + j % grid.nv];
            let jmax = if grid.v_periodic { grid.nv } else { grid.nv - 1 };
            let mut crossings: Vec<(usize, usize, f64, f64, f64)> = Vec::new();
            for i in 0..grid.nu {
                for j in 0..jmax {
                    let a = at(i, j);
                    let b = at(i, j + 1);
                    if (a.0 - level) * (b.0 - level) <= 0.0 && a.0 != b.0 {
                        let w = (level - a.0) / (b.0 - a.0);
                        let o = a.1 + self.o_difference(b.1, a.1) * w;
                        let (u, v0) = Self::grid_param(grid, i, j);
                        let (_, v1) = Self::grid_param(grid, i, j + 1);
                        let v1 = if grid.v_periodic && j + 1 == grid.nv { grid.v[1] } else { v1 };
                        crossings.push((i, j, o, u, v0 + (v1 - v0) * w));
                    }
                }
            }
            for i in 0..grid.nu.saturating_sub(1) {
                for j in 0..grid.nv {
                    let a = at(i, j);
                    let b = at(i + 1, j);
                    if (a.0 - level) * (b.0 - level) <= 0.0 && a.0 != b.0 {
                        let w = (level - a.0) / (b.0 - a.0);
                        let o = a.1 + self.o_difference(b.1, a.1) * w;
                        let (u0, v) = Self::grid_param(grid, i, j);
                        let (u1, _) = Self::grid_param(grid, i + 1, j);
                        crossings.push((i, j, o, u0 + (u1 - u0) * w, v));
                    }
                }
            }
            let mut found: Vec<(f64, f64, f64)> = Vec::new();
            let push = |found: &mut Vec<(f64, f64, f64)>, result: Option<(f64, f64, f64)>| {
                if let Some((_, o_new, u_new)) = result {
                    if !found
                        .iter()
                        .any(|&(o_old, u_old, _)| self.o_difference(o_old, o_new).abs() < 1e-11 && (u_old - u_new).abs() < 1e-6)
                    {
                        found.push((o_new, u_new, 0.0));
                    }
                }
            };
            // Near a tip the contour can be smaller than a cell: seed from the
            // tip itself, pushed off it in four directions.
            if let Some(params) = self.critical_params.get(index) {
                let (ur, vr, _) = self.domain(source);
                for &(u, v) in params {
                    for (du, dv) in [(1.0, 0.0), (-1.0, 0.0), (0.0, 1.0), (0.0, -1.0)] {
                        let (su, sv) = (u + du * 1e-3 * (ur[1] - ur[0]), v + dv * 1e-3 * (vr[1] - vr[0]));
                        push(&mut found, self.face_extremum(source, level, su, sv));
                    }
                }
            }
            let mut buckets: std::collections::HashMap<(usize, usize), Vec<usize>> = std::collections::HashMap::new();
            for (k, &(i, j, _, _, _)) in crossings.iter().enumerate() {
                buckets.entry((i, j)).or_default().push(k);
            }
            for (k, &(i, j, o, u, v)) in crossings.iter().enumerate() {
                let mut is_max = true;
                let mut is_min = true;
                let mut neighbours = Vec::new();
                for di in [-1i64, 0, 1] {
                    for dj in [-1i64, 0, 1] {
                        let i2 = i as i64 + di;
                        if i2 < 0 {
                            continue;
                        }
                        let j2 = j as i64 + dj;
                        let j2 = if grid.v_periodic { j2.rem_euclid(grid.nv as i64) } else { j2 };
                        if j2 < 0 {
                            continue;
                        }
                        if let Some(list) = buckets.get(&(i2 as usize, j2 as usize)) {
                            neighbours.extend(list.iter().copied());
                        }
                    }
                }
                for m in neighbours {
                    if m == k {
                        continue;
                    }
                    let o2 = crossings[m].2;
                    {
                        let d = self.o_difference(o2, o);
                        if d > 0.0 {
                            is_max = false;
                        }
                        if d < 0.0 {
                            is_min = false;
                        }
                    }
                }
                if !(is_max || is_min) {
                    continue;
                }
                push(&mut found, self.face_extremum(source, level, u, v));
            }
            out.extend(found.into_iter().map(|(o, _, _)| (o, index)));
        } else {
            let (range, _, _) = self.domain(source);
            let periodic = matches!(source, Source::RevolvedCorner { .. });
            let n = match source {
                Source::RevolvedCorner { .. } | Source::PrismEnd { .. } | Source::ScrewPrismEdge { .. } | Source::ScrewPrismRim { .. } => 1024,
                _ => self.grids_len_for_time(),
            };
            let _ = periodic;
            // The uniform samples, with the curve's own level extrema merged in:
            // near a tip the two roots of `ℓ = ℓ₀` fall either side of the extremum,
            // closer than any fixed step, and would be missed together.
            let mut samples: Vec<f64> = (0..=n).map(|k| range[0] + (range[1] - range[0]) * k as f64 / n as f64).collect();
            if let Some(extra) = self.critical_params.get(index) {
                samples.extend(extra.iter().map(|p| p.0).filter(|t| *t > range[0] && *t < range[1]));
                samples.sort_by(|a, b| a.total_cmp(b));
            }
            let mut previous = self.source_reduced(source, samples[0], 0.0).0 - level;
            for k in 1..samples.len() {
                let p = samples[k];
                let value = self.source_reduced(source, p, 0.0).0 - level;
                if previous == 0.0 {
                    // A root exactly on the sample (a band's end level on the
                    // curve's own end): bisecting from a zero walks to the far
                    // end of the bracket, a whole sample step away.
                    out.push((self.source_reduced(source, samples[k - 1], 0.0).1, index));
                } else if previous * value < 0.0 {
                    let (mut a, mut b) = (samples[k - 1], p);
                    let fa0 = previous;
                    for _ in 0..200 {
                        let m = 0.5 * (a + b);
                        let fm = self.source_reduced(source, m, 0.0).0 - level;
                        if (fm < 0.0) == (fa0 < 0.0) && fm != 0.0 {
                            a = m;
                        } else {
                            b = m;
                        }
                        if (b - a).abs() < 1e-15 * (range[1] - range[0]).abs().max(1.0) {
                            break;
                        }
                    }
                    let (_, o) = self.source_reduced(source, 0.5 * (a + b), 0.0);
                    out.push((o, index));
                }
                previous = value;
            }
            if previous == 0.0 {
                let last = *samples.last().expect("nonempty");
                out.push((self.source_reduced(source, last, 0.0).1, index));
            }
        }
    }

    fn grids_len_for_time(&self) -> usize {
        let turn = self
            .motion
            .steps
            .iter()
            .map(|step| step.turn.abs())
            .fold(0.0, f64::max)
            * (self.motion.to - self.motion.from).abs();
        ((turn * 256.0) as usize).clamp(1024, 8192)
    }

    /// The clip-frame point a reduced coordinate is tested at.
    fn lift(&self, level: f64, o: f64) -> [f64; 3] {
        let axial = 0.5 * (self.clip.axial[0] + self.clip.axial[1]);
        match self.class {
            Class::Screw(c) => {
                let alpha = o + axial / c;
                [level * alpha.cos(), level * alpha.sin(), axial]
            }
            Class::Turn => {
                let theta = self.clip.angle.map(|a| 0.5 * (a[0] + a[1])).unwrap_or(0.0);
                [level * theta.cos(), level * theta.sin(), o]
            }
            Class::Planar => [level * o.cos(), level * o.sin(), axial],
        }
    }

    /// Is the reduced point inside the sweep: does the point's preimage under
    /// the motion meet the tool for some time in the range?
    ///
    /// Walked in time by the tool's signed distance: it bounds how far the
    /// preimage is from the tool, and the preimage moves no faster than
    /// `speed_bound`, so no time within `distance / speed` of the current one can
    /// be inside. The walk is exact for the tool's own distance (a lower bound for
    /// a prism, which is conservative), and the floor on the step only stops a
    /// grazing walk from creeping.
    fn removed(&self, level: f64, o: f64) -> bool {
        let x = self.frame.world(self.lift(level, o));
        let (t0, t1) = (self.motion.from, self.motion.to);
        let speed = self.speed_bound(x);
        // The floor only keeps a grazing walk from creeping; a coarse floor
        // jumps over a shallow pass of the tool (1e-5 of the plus wheel's range
        // is 0.02 mm of travel, and read a 4e-3 mm sliver of its tooth space as
        // material). A walk still grazing after `cap` steps is on the boundary.
        let floor = (t1 - t0) * 1e-8;
        let cap = 200_000usize;
        let touching = 1e-10 * self.scale;
        let clock = web_time::Instant::now();
        let mut t = t0;
        let mut steps = 0usize;
        let inside = loop {
            steps += 1;
            let distance = self.tool_signed_distance(self.motion_invert(t, x));
            if distance <= touching || steps > cap {
                break true;
            }
            if t >= t1 {
                break false;
            }
            t = (t + (distance / speed).max(floor)).min(t1);
        };
        let mut c = self.counters.get();
        c.1 += 1;
        c.2 += steps;
        c.4 += clock.elapsed().as_secs_f64();
        self.counters.set(c);
        inside
    }

    /// An upper bound on how fast the preimage `M(t)⁻¹ x` moves per unit `t`.
    fn speed_bound(&self, x: Vec3) -> f64 {
        let centre = self.frame.origin;
        let span = self.motion.from.abs().max(self.motion.to.abs());
        let mut reach = x.sub(centre).length();
        for step in &self.motion.steps {
            reach += 2.0 * step.axis_origin.sub(centre).length() + step.advance.abs() * span;
        }
        let speed: f64 = self
            .motion
            .steps
            .iter()
            .map(|step| step.turn.abs() * (reach + step.axis_origin.sub(centre).length()) + step.advance.abs())
            .sum();
        speed.max(1e-300)
    }

    /// The region's intervals at a level, sorted by `lo` (angular: `lo` in
    /// [0, 2π), `hi` may exceed 2π).
    fn intervals_at(&self, level: f64) -> Result<Vec<Interval>, KernelRefusal> {
        if let Some(hit) = self.cache.borrow().get(&level.to_bits()) {
            return Ok(hit.clone());
        }
        let clock = web_time::Instant::now();
        let before = self.counters.get();
        let intervals = self.intervals_uncached(level)?;
        if clock.elapsed().as_secs_f64() > 0.5 && std::env::var("BREP_DEBUG_SOLID_SWEEP").is_ok() {
            let after = self.counters.get();
            eprintln!(
                "  slow level {level:.12}: {:.2}s, candidates {:.2}s, oracle calls {} steps {} ({:.2}s), {} intervals",
                clock.elapsed().as_secs_f64(),
                after.3 - before.3,
                after.1 - before.1,
                after.2 - before.2,
                after.4 - before.4,
                intervals.len()
            );
        }
        self.cache.borrow_mut().insert(level.to_bits(), intervals.clone());
        Ok(intervals)
    }

    fn intervals_uncached(&self, level: f64) -> Result<Vec<Interval>, KernelRefusal> {
        let clock = web_time::Instant::now();
        let mut candidates = Vec::new();
        for index in 0..self.sources.len() {
            self.source_candidates(index, level, &mut candidates);
        }
        let mut c = self.counters.get();
        c.0 += 1;
        c.3 += clock.elapsed().as_secs_f64();
        self.counters.set(c);
        if self.angular {
            for candidate in &mut candidates {
                candidate.0 = candidate.0.rem_euclid(TAU);
            }
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0));
        candidates.dedup_by(|b, a| (b.0 - a.0).abs() < 1e-12);
        if candidates.is_empty() {
            let probes = 64;
            let (base, span) = if self.angular { (0.0, TAU) } else { (-self.scale * 4.0, self.scale * 8.0) };
            if (0..probes).all(|k| self.removed(level, base + span * (k as f64 + 0.5) / probes as f64)) {
                return Err(refuse(
                    "whole_level",
                    format!("{WHOLE_LEVEL} {level}, which this construction does not build"),
                ));
            }
            return Ok(Vec::new());
        }
        let n = candidates.len();
        let gaps = if self.angular { n } else { n - 1 };
        let mut removed = Vec::with_capacity(gaps);
        for k in 0..gaps {
            let a = candidates[k].0;
            let b = if k + 1 < n { candidates[k + 1].0 } else { candidates[0].0 + TAU };
            // A gap far below any feature is the two ends of one boundary
            // point; its middle is on the boundary, where the oracle can only
            // graze. It joins whatever is on either side.
            if (b - a) * level.max(1.0) < 1e-9 * self.scale {
                removed.push(true);
                continue;
            }
            removed.push(self.removed(level, 0.5 * (a + b)));
        }
        let mut intervals = Vec::new();
        // Below a crest some material is left, however narrow: the widest gap,
        // which the oracle can no longer see inside.
        if self.crest_gap && self.angular && removed.iter().all(|r| *r) {
            let width = |k: usize| if k + 1 < n { candidates[k + 1].0 - candidates[k].0 } else { candidates[0].0 + TAU - candidates[k].0 };
            if let Some(widest) = (0..gaps).max_by(|&a, &b| width(a).total_cmp(&width(b))) {
                removed[widest] = false;
            }
        }
        if self.angular && removed.iter().all(|r| *r) {
            return Err(refuse(
                "whole_level",
                format!("{WHOLE_LEVEL} {level}, which this construction does not build"),
            ));
        }
        // Start the walk after a material gap so a run never straddles the start.
        let start = if self.angular {
            removed.iter().position(|r| !r).map(|k| (k + 1) % n).unwrap_or(0)
        } else {
            0
        };
        let mut k = 0;
        while k < gaps {
            let g = (start + k) % n.max(1);
            if !removed[g] {
                k += 1;
                continue;
            }
            let first = g;
            let mut last = g;
            while k + 1 < gaps && removed[(start + k + 1) % n] {
                k += 1;
                last = (start + k) % n;
            }
            let lo = candidates[first];
            let hi_index = (last + 1) % n;
            let hi = candidates[hi_index];
            let mut hi_value = hi.0;
            if self.angular && hi_value <= lo.0 {
                hi_value += TAU;
            }
            intervals.push(Interval {
                lo: lo.0,
                hi: hi_value,
                lo_source: lo.1,
                hi_source: hi.1,
            });
            k += 1;
        }
        intervals.sort_by(|a, b| a.lo.total_cmp(&b.lo));
        Ok(intervals)
    }

    /// Critical points of the level over a source (tips and plateaus).
    /// The face's characteristic function: `det(∇o, ∇ℓ) / |∇ℓ|`, zero where
    /// the face's image turns back (its envelope), and the level there.
    fn characteristic(&self, source: Source, u: f64, v: f64) -> (f64, f64) {
        let (ur, vr, _) = self.domain(source);
        let hu = 1e-6 * (ur[1] - ur[0]).max(1e-3);
        let hv = 1e-6 * (vr[1] - vr[0]).max(1e-3);
        let (l, _) = self.source_reduced(source, u, v);
        let (lu1, ou1) = self.source_reduced(source, u + hu, v);
        let (lu0, ou0) = self.source_reduced(source, u - hu, v);
        let (lv1, ov1) = self.source_reduced(source, u, v + hv);
        let (lv0, ov0) = self.source_reduced(source, u, v - hv);
        let (lu, lv) = ((lu1 - lu0) / (2.0 * hu), (lv1 - lv0) / (2.0 * hv));
        let ou = self.o_difference(ou1, ou0) / (2.0 * hu);
        let ov = self.o_difference(ov1, ov0) / (2.0 * hv);
        ((ou * lv - ov * lu) / lu.hypot(lv).max(1e-300), l)
    }

    /// FOLDS and KINKS of a face, recorded as its critical points so events snap
    /// to them and band ends read them exactly:
    /// - a FOLD is where the level is extremal along the face's own envelope
    ///   (`D = 0`): two extrema of `o` on one level contour annihilate there
    ///   (the plus wheel's lobes merging at ρ = 20, the tip of its arm's flat
    ///   end). Newton on {D = 0, ℓ_u·D_v − ℓ_v·D_u = 0}, seeded where the level
    ///   is locally extremal among the grid's crossings of D = 0.
    /// - a KINK is where the envelope runs into the face's own boundary and the
    ///   boundary's edge takes over: roots of D along each non-periodic side.
    fn folds_and_kinks(&self, source: Source, grid: &Grid, out: &mut Vec<(f64, f64)>, params: &mut Vec<(f64, f64)>) {
        let (ur, vr, v_periodic) = self.domain(source);
        let u_periodic = self.u_periodic(source);
        let (nu, nv) = (grid.nu, grid.nv);
        let mut d = vec![0.0; nu * nv];
        for i in 0..nu {
            for j in 0..nv {
                let (u, v) = Self::grid_param(grid, i, j);
                d[i * nv + j] = self.characteristic(source, u, v).0;
            }
        }
        // Crossings of D = 0 along grid edges: (i, j, u, v, level).
        let mut crossings: Vec<(usize, usize, f64, f64, f64)> = Vec::new();
        let jmax = if v_periodic { nv } else { nv - 1 };
        for i in 0..nu {
            for j in 0..jmax {
                let (a, b) = (d[i * nv + j], d[i * nv + (j + 1) % nv]);
                if a * b < 0.0 {
                    let w = a / (a - b);
                    let (u, v0) = Self::grid_param(grid, i, j);
                    let (_, v1) = Self::grid_param(grid, i, j + 1);
                    let v1 = if v_periodic && j + 1 == nv { vr[1] } else { v1 };
                    let v = v0 + (v1 - v0) * w;
                    crossings.push((i, j, u, v, self.source_reduced(source, u, v).0));
                }
            }
        }
        for i in 0..nu - 1 {
            for j in 0..nv {
                let (a, b) = (d[i * nv + j], d[(i + 1) * nv + j]);
                if a * b < 0.0 {
                    let w = a / (a - b);
                    let (u0, v) = Self::grid_param(grid, i, j);
                    let (u1, _) = Self::grid_param(grid, i + 1, j);
                    let u = u0 + (u1 - u0) * w;
                    crossings.push((i, j, u, v, self.source_reduced(source, u, v).0));
                }
            }
        }
        let mut buckets: std::collections::HashMap<(usize, usize), Vec<usize>> = std::collections::HashMap::new();
        for (k, c) in crossings.iter().enumerate() {
            buckets.entry((c.0, c.1)).or_default().push(k);
        }
        let ku = 1e-4 * (ur[1] - ur[0]);
        let kv = 1e-4 * (vr[1] - vr[0]);
        let fold = |u: f64, v: f64| -> [f64; 2] {
            let (dd, _) = self.characteristic(source, u, v);
            let du = (self.characteristic(source, u + ku, v).0 - self.characteristic(source, u - ku, v).0) / (2.0 * ku);
            let dv = (self.characteristic(source, u, v + kv).0 - self.characteristic(source, u, v - kv).0) / (2.0 * kv);
            let lu = (self.source_reduced(source, u + ku, v).0 - self.source_reduced(source, u - ku, v).0) / (2.0 * ku);
            let lv = (self.source_reduced(source, u, v + kv).0 - self.source_reduced(source, u, v - kv).0) / (2.0 * kv);
            let norm = (lu.hypot(lv) * du.hypot(dv)).max(1e-300);
            [dd, (lu * dv - lv * du) / norm]
        };
        for (k, &(i, j, u0, v0, level)) in crossings.iter().enumerate() {
            let mut is_min = true;
            let mut is_max = true;
            for di in [-1i64, 0, 1] {
                for dj in [-1i64, 0, 1] {
                    let i2 = i as i64 + di;
                    let j2 = if v_periodic { (j as i64 + dj).rem_euclid(nv as i64) } else { j as i64 + dj };
                    if i2 < 0 || j2 < 0 {
                        continue;
                    }
                    if let Some(list) = buckets.get(&(i2 as usize, j2 as usize)) {
                        for &m in list {
                            if m == k {
                                continue;
                            }
                            if crossings[m].4 < level {
                                is_min = false;
                            }
                            if crossings[m].4 > level {
                                is_max = false;
                            }
                        }
                    }
                }
            }
            if !(is_min || is_max) {
                continue;
            }
            let (mut u, mut v) = (u0, v0);
            let mut converged = false;
            for _ in 0..40 {
                let f = fold(u, v);
                let (su, sv) = (ku * 10.0, kv * 10.0);
                let fu1 = fold(u + su, v);
                let fu0 = fold(u - su, v);
                let fv1 = fold(u, v + sv);
                let fv0 = fold(u, v - sv);
                let j00 = (fu1[0] - fu0[0]) / (2.0 * su);
                let j01 = (fv1[0] - fv0[0]) / (2.0 * sv);
                let j10 = (fu1[1] - fu0[1]) / (2.0 * su);
                let j11 = (fv1[1] - fv0[1]) / (2.0 * sv);
                let det = j00 * j11 - j01 * j10;
                if !det.is_finite() || det.abs() < 1e-300 {
                    break;
                }
                let du = -(f[0] * j11 - f[1] * j01) / det;
                let dv = -(j00 * f[1] - j10 * f[0]) / det;
                // Stay near the seed: a fold is local to its crossing.
                let cap = 4.0 * ((ur[1] - ur[0]) / (nu as f64)).max((vr[1] - vr[0]) / (nv as f64));
                let scale = (du.hypot(dv) / cap).max(1.0);
                u += du / scale;
                v += dv / scale;
                if du.hypot(dv) < 1e-12 * (ur[1] - ur[0]).max(vr[1] - vr[0]) {
                    converged = true;
                    break;
                }
            }
            let f = fold(u, v);
            if !(converged || (f[0].abs() < 1e-9 && f[1].abs() < 1e-5)) {
                continue;
            }
            if u_periodic {
                u = ur[0] + (u - ur[0]).rem_euclid(ur[1] - ur[0]);
            }
            if v_periodic {
                v = vr[0] + (v - vr[0]).rem_euclid(vr[1] - vr[0]);
            }
            if u > ur[0] && u < ur[1] && (v_periodic || (v > vr[0] && v < vr[1])) {
                out.push(self.source_reduced(source, u, v));
                params.push((u, v));
            }
        }
        // Kinks: roots of D along each non-periodic side of the face.
        let mut sides: Vec<(bool, f64)> = Vec::new(); // (along v at fixed u, the fixed value)
        if !u_periodic {
            sides.push((true, ur[0]));
            sides.push((true, ur[1]));
        }
        if !v_periodic {
            sides.push((false, vr[0]));
            sides.push((false, vr[1]));
        }
        for (along_v, fixed) in sides {
            let (range, n) = if along_v { (vr, nv * 4) } else { (ur, nu * 4) };
            let at = |t: f64| if along_v { (fixed, t) } else { (t, fixed) };
            let value = |t: f64| {
                let (u, v) = at(t);
                self.characteristic(source, u, v).0
            };
            let mut previous = value(range[0]);
            for k in 1..=n {
                let t = range[0] + (range[1] - range[0]) * k as f64 / n as f64;
                let current = value(t);
                if previous * current < 0.0 {
                    let (mut a, mut b) = (t - (range[1] - range[0]) / n as f64, t);
                    let fa = previous;
                    for _ in 0..100 {
                        let m = 0.5 * (a + b);
                        if (value(m) < 0.0) == (fa < 0.0) {
                            a = m;
                        } else {
                            b = m;
                        }
                    }
                    let (u, v) = at(0.5 * (a + b));
                    out.push(self.source_reduced(source, u, v));
                    params.push((u, v));
                }
                previous = current;
            }
        }
    }

    fn source_critical(&self, index: usize) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
        let source = self.sources[index];
        let mut out = Vec::new();
        let mut params = Vec::new();
        if Self::is_face(source) {
            let grid = self.grids[index].as_ref().expect("face grid");
            let (ur, vr, periodic) = self.domain(source);
            let at = |i: usize, j: usize| grid.values[i * grid.nv + j].0;
            let u_periodic = self.u_periodic(source);
            // A periodic first parameter has its last row equal to its first.
            let rows = if u_periodic { grid.nu - 1 } else { grid.nu };
            for i in 0..rows {
                if !u_periodic && (i == 0 || i + 1 == grid.nu) {
                    continue;
                }
                for j in 0..grid.nv {
                    if !periodic && (j == 0 || j + 1 == grid.nv) {
                        continue;
                    }
                    let here = at(i, j);
                    let mut is_min = true;
                    let mut is_max = true;
                    for di in [-1i64, 0, 1] {
                        for dj in [-1i64, 0, 1] {
                            if di == 0 && dj == 0 {
                                continue;
                            }
                            let i2 = ((i as i64 + di).rem_euclid(rows as i64)) as usize;
                            let j2 = ((j as i64 + dj).rem_euclid(grid.nv as i64)) as usize;
                            let there = at(i2, j2);
                            if there < here {
                                is_min = false;
                            }
                            if there > here {
                                is_max = false;
                            }
                        }
                    }
                    if !(is_min || is_max) {
                        continue;
                    }
                    // Newton on ∇ℓ = 0.
                    let (mut u, mut v) = Self::grid_param(grid, i, j);
                    let hu = 1e-5 * (ur[1] - ur[0]);
                    let hv = 1e-5 * (vr[1] - vr[0]);
                    let level = |u: f64, v: f64| self.source_reduced(source, u, v).0;
                    for _ in 0..50 {
                        let f = level(u, v);
                        let fu = (level(u + hu, v) - level(u - hu, v)) / (2.0 * hu);
                        let fv = (level(u, v + hv) - level(u, v - hv)) / (2.0 * hv);
                        let fuu = (level(u + hu, v) - 2.0 * f + level(u - hu, v)) / (hu * hu);
                        let fvv = (level(u, v + hv) - 2.0 * f + level(u, v - hv)) / (hv * hv);
                        let fuv = (level(u + hu, v + hv) - level(u + hu, v - hv) - level(u - hu, v + hv)
                            + level(u - hu, v - hv))
                            / (4.0 * hu * hv);
                        let det = fuu * fvv - fuv * fuv;
                        if det.abs() < 1e-300 {
                            break;
                        }
                        let du = -(fu * fvv - fv * fuv) / det;
                        let dv = -(fuu * fv - fuv * fu) / det;
                        u += du;
                        v += dv;
                        if du.abs() < 1e-14 && dv.abs() < 1e-14 {
                            break;
                        }
                    }
                    if u_periodic {
                        u = ur[0] + (u - ur[0]).rem_euclid(ur[1] - ur[0]);
                    }
                    if periodic {
                        v = vr[0] + (v - vr[0]).rem_euclid(vr[1] - vr[0]);
                    }
                    if u >= ur[0] && u <= ur[1] && (periodic || (v >= vr[0] && v <= vr[1])) {
                        out.push(self.source_reduced(source, u, v));
                        params.push((u, v));
                    }
                }
            }
            self.folds_and_kinks(source, grid, &mut out, &mut params);
        } else {
            let (range, _, _) = self.domain(source);
            let n = 2048;
            let param = |k: usize| range[0] + (range[1] - range[0]) * k as f64 / n as f64;
            let values: Vec<f64> = (0..=n).map(|k| self.source_reduced(source, param(k), 0.0).0).collect();
            for k in 1..n {
                let (a, b, c) = (values[k - 1], values[k], values[k + 1]);
                if (b <= a && b <= c) || (b >= a && b >= c) {
                    let sign = if b <= a && b <= c { 1.0 } else { -1.0 };
                    let (mut lo, mut hi) = (param(k - 1), param(k + 1));
                    let ratio = 0.618_033_988_749_894_9;
                    for _ in 0..100 {
                        let c1 = hi - ratio * (hi - lo);
                        let d1 = lo + ratio * (hi - lo);
                        if sign * self.source_reduced(source, c1, 0.0).0 < sign * self.source_reduced(source, d1, 0.0).0 {
                            hi = d1;
                        } else {
                            lo = c1;
                        }
                    }
                    out.push(self.source_reduced(source, 0.5 * (lo + hi), 0.0));
                    params.push((0.5 * (lo + hi), 0.0));
                }
            }
        }
        (out, params)
    }
}

/// Are two sources the same boundary piece of the sweep: the same source, or
/// a piece and the one that continues it across a set-periodic motion's wrap?
/// A face handing over to the edge that bounds it is NOT the same: the V
/// wheel's flank meets its tip circle's image at a kink, and fitting one curve
/// through it read the groove 1.5e-6 off (split: 1.1e-8).
fn same_or_bounding(engine: &Engine, a: usize, b: usize) -> bool {
    if a == b {
        return true;
    }
    let pieces = match engine.tool {
        SweptTool::Revolved { meridian, .. } => meridian.len(),
        SweptTool::Prism { outline, .. } => outline.len(),
    };
    // (piece of the face, piece whose start the edge is)
    let face = |s: Source| match s {
        Source::RevolvedFace { piece } | Source::PrismFace { piece } | Source::ScrewPrismSide { piece } => Some(piece),
        _ => None,
    };
    let edge = |s: Source| match s {
        Source::RevolvedCorner { piece } | Source::PrismVertex { piece } | Source::ScrewPrismEdge { piece } => Some(piece),
        _ => None,
    };
    let (sa, sb) = (engine.sources[a], engine.sources[b]);
    // Across the wrap, piece `k` continues as `wrap[k]`.
    let same_piece = |x: usize, y: usize| {
        x == y || engine.wrap.as_ref().is_some_and(|map| map[x] == y || map[y] == x)
    };
    let bounds = |f: usize, e: usize| {
        same_piece(e, f) || same_piece(e, (f + 1) % pieces)
    };
    let _ = bounds;
    match (face(sa), edge(sa), face(sb), edge(sb)) {
        (Some(f), _, Some(g), _) => same_piece(f, g),
        (_, Some(e), _, Some(g)) => same_piece(e, g),
        _ => false,
    }
}

/// Do two interval lists describe the same structure (same count, each end on
/// the same source or one that hands over to it smoothly, matched in order)?
fn compatible(engine: &Engine, a: &[Interval], b: &[Interval]) -> bool {
    let angular = engine.angular;
    if a.len() != b.len() {
        return false;
    }
    // Match each interval of `a` to the one of `b` nearest its middle.
    let mut used = vec![false; b.len()];
    for x in a {
        let mid = 0.5 * (x.lo + x.hi);
        let mut best = None;
        let mut best_distance = f64::INFINITY;
        for (k, y) in b.iter().enumerate() {
            let other = 0.5 * (y.lo + y.hi);
            let d = if angular { wrap_pi(mid - other).abs() } else { (mid - other).abs() };
            if d < best_distance {
                best_distance = d;
                best = Some(k);
            }
        }
        let Some(k) = best else { return false };
        let y = &b[k];
        let overlap = if angular {
            let shift = (wrap_pi(y.lo - x.lo)) - (y.lo - x.lo);
            let (ylo, yhi) = (y.lo + shift, y.hi + shift);
            x.hi.min(yhi) - x.lo.max(ylo)
        } else {
            x.hi.min(y.hi) - x.lo.max(y.lo)
        };
        if used[k]
            || overlap < 0.0
            || !same_or_bounding(engine, x.lo_source, y.lo_source)
            || !same_or_bounding(engine, x.hi_source, y.hi_source)
        {
            return false;
        }
        used[k] = true;
    }
    true
}

/// A boundary piece in reduced coordinates, as a polyline of exact samples
/// (the first and last are the piece's ends) with their `σ`.
struct Piece {
    points: Vec<(f64, f64)>,
    sigma: Vec<f64>,
    /// A straight level segment (built exactly, not fitted).
    level_segment: bool,
}

/// Build the sweep of `tool` along `motion` inside `clip`, as one body per
/// connected region of the reduced section.
pub fn sweep_solid_within(
    tool: &SweptTool,
    motion: &SolidMotion,
    clip: &SweepClip,
) -> Result<Vec<BrepSolid>, KernelRefusal> {
    let narrowed: SweepClip;
    // --- Frame and classification.
    let ez = clip
        .axis_direction
        .normalized()
        .or_refuse(KernelStage::Collect, "clip_axis")?;
    let ex = clip
        .reference
        .sub(ez.scale(clip.reference.dot(ez)))
        .normalized()
        .or_refuse(KernelStage::Collect, "clip_reference")?;
    let frame = Frame { origin: clip.axis_origin, ex, ey: ez.cross(ex), ez };
    if !(motion.from.is_finite() && motion.to.is_finite() && motion.to > motion.from) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "motion_range",
            format!("{SOLID_SWEEP_REFUSAL}: the motion's range must run forwards"),
        ));
    }
    if motion.steps.is_empty() {
        return Err(KernelRefusal::input(KernelStage::Collect, "motion_empty", format!("{SOLID_SWEEP_REFUSAL}: no motion")));
    }
    let mut axes = Vec::with_capacity(motion.steps.len());
    for step in &motion.steps {
        axes.push(step.axis_direction.normalized().or_refuse(KernelStage::Collect, "step_axis")?);
    }
    if !(clip.radius[0] >= 0.0 && clip.radius[1] > clip.radius[0] && clip.axial[1] > clip.axial[0]) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "clip_range",
            format!("{SOLID_SWEEP_REFUSAL}: the clip's ranges must be increasing"),
        ));
    }
    let on_axis = |origin: Vec3, axis: Vec3| -> bool {
        let parallel = axis.cross(ez).length() < 1e-12;
        let offset = origin.sub(clip.axis_origin);
        parallel && offset.sub(ez.scale(offset.dot(ez))).length() < 1e-9 * (1.0 + clip.radius[1])
    };
    let class = 'class: { match tool {
        SweptTool::Revolved { meridian, .. } => {
            validate_outline(meridian, "meridian")?;
            if motion.steps.len() != 1 || !on_axis(motion.steps[0].axis_origin, axes[0]) {
                return Err(refuse(
                    "motion_class",
                    "a solid of revolution is swept here only by one screw or turn about the clip's axis".into(),
                ));
            }
            let step = motion.steps[0];
            let sense = axes[0].dot(ez).signum();
            if step.turn == 0.0 {
                return Err(refuse("motion_class", "a slide of a solid of revolution is not built here".into()));
            }
            if step.advance == 0.0 {
                Class::Turn
            } else {
                // axial advance per radian about +ez
                Class::Screw((step.advance * sense) / (step.turn * sense))
            }
        }
        SweptTool::Prism { normal, outline, from, to, origin, .. } => {
            validate_outline(outline, "prism")?;
            let n = normal.normalized().or_refuse(KernelStage::Collect, "prism_normal")?;
            let screw = motion.steps.len() == 1
                && motion.steps[0].turn != 0.0
                && motion.steps[0].advance != 0.0
                && on_axis(motion.steps[0].axis_origin, axes[0])
                && n.cross(ez).length() <= 1e-12;
            if screw {
                // A screw carries the prism's ends along, so the clip may run past them.
                let step = motion.steps[0];
                let sense = axes[0].dot(ez).signum();
                break 'class Class::Screw((step.advance * sense) / (step.turn * sense));
            }
            if n.cross(ez).length() > 1e-12 {
                return Err(refuse("motion_class", "a prism is swept here only along its own direction's clip axis".into()));
            }
            for (step, axis) in motion.steps.iter().zip(&axes) {
                let turn_ok = step.turn == 0.0 || axis.cross(n).length() < 1e-12;
                let advance_ok = step.advance == 0.0 || (step.turn == 0.0 && axis.dot(n).abs() < 1e-12);
                if !(turn_ok && advance_ok) {
                    return Err(refuse(
                        "motion_class",
                        "a prism is swept here only by turns about axes along it and slides across it".into(),
                    ));
                }
            }
            // The clip's axial range must lie within the prism's extent.
            let base = clip.axis_origin.sub(*origin).dot(n);
            let sense = ez.dot(n);
            let h0 = base + sense * clip.axial[0];
            let h1 = base + sense * clip.axial[1];
            if h0.min(h1) < *from || h0.max(h1) > *to {
                return Err(refuse(
                    "clip_past_prism",
                    "the clip reaches past the prism's ends, where the sweep is not a prism".into(),
                ));
            }
            Class::Planar
        }
    } };
    let prism_height = match tool {
        SweptTool::Prism { from, to, .. } => 0.5 * (from + to),
        _ => 0.0,
    };

    // Does the motion end where it began, as a set? For a prism, every outline
    // vertex at the last position must sit on some vertex at the first (the plus
    // turns 540°, which its half-turn symmetry makes a whole period). The
    // permutation says which piece continues which across the wrap.
    let wrap: Option<Vec<usize>> = match tool {
        SweptTool::Prism { .. } if matches!(class, Class::Screw(_)) => None,
        SweptTool::Prism { origin, normal, x_axis, outline, .. } => {
            let probe = Engine {
                tool,
                motion,
                axes: axes.clone(),
                frame,
                class,
                clip,
                crest_gap: false,
                sources: Vec::new(),
                grids: Vec::new(),
                critical: Vec::new(),
                critical_params: Vec::new(),
                angular: true,
                prism_height,
                scale: 1.0,
                cache: std::cell::RefCell::new(std::collections::HashMap::new()),
                wrap: None,
                counters: std::cell::Cell::new((0, 0, 0, 0.0, 0.0)),
            };
            let n = normal.normalized_or(*normal);
            let xa = x_axis.normalized_or(*x_axis);
            let ya = n.cross(xa);
            let vertex = |k: usize, t: f64| {
                let q = outline[k].point(0.0);
                probe.motion_apply(t, origin.add(xa.scale(q[0])).add(ya.scale(q[1])).add(n.scale(prism_height)))
            };
            let size = scale_of(outline);
            let mut map = Vec::with_capacity(outline.len());
            for k in 0..outline.len() {
                let end = vertex(k, motion.to);
                match (0..outline.len()).find(|&j| vertex(j, motion.from).sub(end).length() < 1e-9 * size) {
                    Some(j) => map.push(j),
                    None => break,
                }
            }
            let consistent = map.len() == outline.len()
                && (0..outline.len()).all(|k| map[(k + 1) % outline.len()] == (map[k] + 1) % outline.len());
            consistent.then_some(map)
        }
        SweptTool::Revolved { .. } => None,
    };
    let periodic_motion = wrap.is_some();
    let mut sources = Vec::new();
    match tool {
        SweptTool::Revolved { meridian, .. } => {
            // A piece lying on the axis revolves into nothing, and a corner on the
            // axis into a point: neither bounds the tool.
            let on_axis = |p: [f64; 2]| p[0].abs() <= 1e-12 * scale_of(meridian);
            for piece in 0..meridian.len() {
                let lies_on_axis = matches!(meridian[piece], OutlinePiece::Line { from, to } if on_axis(from) && on_axis(to));
                if !lies_on_axis {
                    sources.push(Source::RevolvedFace { piece });
                }
                if !meridian[piece].is_full_circle() && !on_axis(meridian[piece].point(0.0)) {
                    sources.push(Source::RevolvedCorner { piece });
                }
            }
        }
        SweptTool::Prism { outline, .. } if matches!(class, Class::Screw(_)) => {
            for piece in 0..outline.len() {
                sources.push(Source::ScrewPrismSide { piece });
                if !outline[piece].is_full_circle() {
                    sources.push(Source::ScrewPrismEdge { piece });
                }
                sources.push(Source::ScrewPrismRim { piece, end: false });
                sources.push(Source::ScrewPrismRim { piece, end: true });
            }
        }
        SweptTool::Prism { outline, .. } => {
            for piece in 0..outline.len() {
                sources.push(Source::PrismFace { piece });
                if !outline[piece].is_full_circle() {
                    sources.push(Source::PrismVertex { piece });
                }
                if !periodic_motion {
                    sources.push(Source::PrismEnd { piece, end: false });
                    sources.push(Source::PrismEnd { piece, end: true });
                }
            }
        }
    }
    let scale = clip.radius[1].max(1.0);
    let mut engine = Engine {
        tool,
        motion,
        axes,
        frame,
        class,
        clip,
        crest_gap: false,
        sources,
        grids: Vec::new(),
        critical: Vec::new(),
        critical_params: Vec::new(),
        angular: class != Class::Turn,
        prism_height,
        scale,
        cache: std::cell::RefCell::new(std::collections::HashMap::new()),
        wrap,
        counters: std::cell::Cell::new((0, 0, 0, 0.0, 0.0)),
    };
    engine.grids = engine
        .sources
        .iter()
        .map(|source| if Engine::is_face(*source) { Some(engine.build_grid(*source)) } else { None })
        .collect();
    let critical: Vec<_> = (0..engine.sources.len()).map(|k| engine.source_critical(k)).collect();
    engine.critical_params = critical.iter().map(|c| c.1.clone()).collect();
    engine.critical = critical.into_iter().map(|c| c.0).collect();

    // --- Ends: a screw or turn must not reach the clip at either end.
    if matches!(class, Class::Screw(_) | Class::Turn) {
        check_ends_clear(&engine)?;
    }

    // --- A screw whose grooves meet: above a crest level nothing stays. Build
    //     what stays below it, and take that from the clip at the end.
    let mut crest = None;
    if matches!(class, Class::Screw(_)) && clip.radius[0] == 0.0 && clip.angle.is_none() && whole_level_on() {
        crest = crest_level(&engine)?;
        if let Some(level) = crest {
            narrowed = SweepClip { radius: [0.0, level], ..clip.clone() };
            engine.clip = &narrowed;
            engine.crest_gap = true;
        }
    }

    // --- Scan the levels for events.
    let (l0, l1) = (engine.clip.radius[0], engine.clip.radius[1]);
    let scan = 192;
    let mut levels: Vec<f64> = (0..=scan).map(|k| l0 + (l1 - l0) * k as f64 / scan as f64).collect();
    // Never sample exactly on a critical level: nudge the scan off them.
    for level in &mut levels {
        for critical in engine.critical.iter().flatten() {
            if (critical.0 - *level).abs() < 1e-9 * scale {
                *level += 1e-7 * (l1 - l0);
            }
        }
    }
    let mut structure = Vec::with_capacity(levels.len());
    for &level in &levels {
        structure.push(engine.intervals_at(level)?);
    }
    let debug = std::env::var("BREP_DEBUG_SOLID_SWEEP").is_ok();
    let mut breaks = vec![l0];
    for k in 0..scan {
        find_events(&engine, levels[k], &structure[k], levels[k + 1], &structure[k + 1], &mut breaks, 0)?;
    }
    breaks.push(l1);
    // Snap each event to a critical level of a source when one is that close.
    // A birth or death is found by bisection only to where the seeding first
    // sees the level's contour, which near a tip is a √ away from it: the
    // critical level is the exact one.
    let last = breaks.len() - 1;
    for level in breaks[1..last].iter_mut() {
        let mut nearest = *level;
        let mut distance = 1e-5 * scale;
        for critical in engine.critical.iter().flatten() {
            if (critical.0 - *level).abs() < distance {
                distance = (critical.0 - *level).abs();
                nearest = critical.0;
            }
        }
        *level = nearest;
    }
    breaks.sort_by(|a, b| a.total_cmp(b));
    // A source handing over to the next one at an edge can flicker over a few
    // bisection steps (the face's extremum sits a hair inside its domain); the
    // bands that leaves are far below any feature. Keep the first break of each
    // cluster, and the clip's own ends.
    let mut kept: Vec<f64> = Vec::with_capacity(breaks.len());
    for (index, &level) in breaks.iter().enumerate() {
        let is_end = index == 0 || index + 1 == breaks.len();
        match kept.last() {
            Some(&previous) if level - previous < 1e-5 * scale => {
                if is_end {
                    kept.pop();
                    kept.push(level);
                }
            }
            _ => kept.push(level),
        }
    }
    let mut breaks = kept;

    // --- Merges no critical point explains: a FOLD the face's own Newton cannot
    //     reach (the plus wheel's arm tip: its level varies by 1e-4 along the
    //     envelope there). The gap between the two merging ends closes as
    //     √(ℓf − ℓ), so gap² is smooth in ℓ: fit it as a quadratic through three
    //     levels below the event, outside the band where the extremum Newton
    //     flickers, and take its root as the merge level; the gap's middle,
    //     extrapolated the same way, is the merge point. Both ends' sources
    //     record it as a critical point, and the bands keep their samples away
    //     from it (`fold_ends`).
    let mut fold_ends: Vec<f64> = Vec::new();
    for index in 1..breaks.len().saturating_sub(1) {
        let level = breaks[index];
        let snapped = engine.critical.iter().flatten().any(|c| (c.0 - level).abs() < 1e-9 * scale);
        if snapped {
            continue;
        }
        let room = (level - breaks[index - 1]).min(breaks[index + 1] - level);
        // Close enough that the √ law holds to the fit's order, far enough to be
        // clear of the flicker (the plus wheel: 2e-4 out, flicker inside 1e-5).
        let delta = (5e-6 * scale).min(room / 8.0);
        let below = engine.intervals_at(level - delta)?;
        let above = engine.intervals_at(level + delta)?;
        if below.len() <= above.len() {
            continue;
        }
        // Adjacent intervals below whose facing ends close into one above.
        let mut refined: Vec<(f64, f64, usize, usize)> = Vec::new();
        for k in 0..below.len() {
            let (x, y) = (below[k], below[(k + 1) % below.len()]);
            if !engine.angular && k + 1 == below.len() {
                break;
            }
            let gap_at = |intervals: &[Interval]| -> Option<(f64, f64)> {
                // the pair nearest (x.hi, y.lo) at this level
                let mut best: Option<(f64, f64, f64)> = None;
                for i in 0..intervals.len() {
                    let (p, q) = (intervals[i], intervals[(i + 1) % intervals.len()]);
                    let d = engine.o_difference(p.hi, x.hi).abs() + engine.o_difference(q.lo, y.lo).abs();
                    if best.map_or(true, |b| d < b.2) {
                        let gap = engine.o_difference(q.lo, p.hi);
                        best = Some((gap, p.hi + 0.5 * gap, d));
                    }
                }
                best.map(|b| (b.0, b.1))
            };
            let mut samples = Vec::new();
            for factor in [1.0, 2.0, 4.0] {
                let l = level - delta * factor;
                let intervals = engine.intervals_at(l)?;
                if intervals.len() != below.len() {
                    break;
                }
                if let Some((gap, middle)) = gap_at(&intervals) {
                    samples.push((l, gap, middle));
                }
            }
            if samples.len() < 3 || samples.iter().any(|s| s.1 <= 0.0 || s.1 > 0.5) {
                continue;
            }
            // Quadratic through (l, gap²); its root above the samples.
            let quad = |values: [f64; 3]| -> (f64, f64, f64) {
                let (x0, x1, x2) = (samples[0].0, samples[1].0, samples[2].0);
                let (y0, y1, y2) = (values[0], values[1], values[2]);
                let d01 = (y0 - y1) / (x0 - x1);
                let d12 = (y1 - y2) / (x1 - x2);
                let a2 = (d01 - d12) / (x0 - x2);
                let a1 = d01 - a2 * (x0 + x1);
                let a0 = y0 - a1 * x0 - a2 * x0 * x0;
                (a0, a1, a2)
            };
            let (c0, c1, c2) = quad([samples[0].1.powi(2), samples[1].1.powi(2), samples[2].1.powi(2)]);
            let root = if c2.abs() < 1e-300 {
                -c0 / c1
            } else {
                let disc = c1 * c1 - 4.0 * c2 * c0;
                if disc < 0.0 {
                    continue;
                }
                let r1 = (-c1 + disc.sqrt()) / (2.0 * c2);
                let r2 = (-c1 - disc.sqrt()) / (2.0 * c2);
                if (r1 - level).abs() < (r2 - level).abs() { r1 } else { r2 }
            };
            if !(root > samples[0].0 && (root - level).abs() < delta) {
                continue;
            }
            let (m0, m1, m2) = quad([samples[0].2, samples[1].2, samples[2].2]);
            let middle = m0 + m1 * root + m2 * root * root;
            refined.push((root, middle, x.hi_source, y.lo_source));
        }
        if !refined.is_empty() {
            // Every merging pair records its own point; the break is their mean
            // level (equal to round-off where the spaces are copies of one).
            let root = refined.iter().map(|r| r.0).sum::<f64>() / refined.len() as f64;
            for &(own, middle, a_source, b_source) in &refined {
                if debug {
                    eprintln!("  fold: break {level:.12} -> {own:.12} at o {middle:.12}");
                }
                engine.critical[a_source].push((root, middle));
                engine.critical[b_source].push((root, middle));
            }
            breaks[index] = root;
            fold_ends.push(root);
        }
    }

    if debug {
        eprintln!("solid sweep: after events {:?} breaks {}", engine.counters.get(), breaks.len());
    }
    // --- Sample each band's interval ends.
    let mut pieces: Vec<Piece> = Vec::new();
    // Per band: the intervals at its bottom and top (for the level segments).
    let mut band_ends: Vec<(f64, f64, Vec<(usize, usize)>)> = Vec::new();
    for window in breaks.windows(2) {
        let (a, b) = (window[0], window[1]);
        if b - a < 1e-12 * scale {
            continue;
        }
        let mid = engine.intervals_at(0.5 * (a + b))?;
        if mid.is_empty() {
            continue;
        }
        let (band_pieces, _, _) = sample_band(&engine, a, b, &mid, &fold_ends)?;
        let first = pieces.len();
        let pairs = (0..band_pieces.len() / 2).map(|i| (first + 2 * i, first + 2 * i + 1)).collect();
        pieces.extend(band_pieces);
        band_ends.push((a, b, pairs));
    }
    if pieces.is_empty() {
        return Ok(Vec::new());
    }
    if debug {
        eprintln!("solid sweep: after bands {:?}", engine.counters.get());
        for (k, critical) in engine.critical.iter().enumerate() {
            eprintln!("  source {k} {:?}: critical {:?}", engine.sources[k], critical);
        }
    }

    // --- One value per interval end at each break. The band below and the band
    //     above read an end that continues across a break from their own
    //     sources, at a level found only to the flicker cluster; the two agree to
    //     far below any feature but not to the bit, and a sliver of level segment
    //     between them is an edge no solid should carry. Snap every end at a level
    //     to its cluster's first value (ends of one interval meeting at a tip
    //     included).
    let snap_length = 1e-5 * scale;
    let mut break_levels: Vec<f64> = band_ends.iter().flat_map(|e| [e.0, e.1]).collect();
    break_levels.sort_by(|a, b| a.total_cmp(b));
    break_levels.dedup_by(|b, a| (*b - *a).abs() < 1e-12 * scale);
    for &level in &break_levels {
        let tolerance = if engine.angular { snap_length / level.max(1e-12) } else { snap_length };
        let mut ends: Vec<(usize, bool)> = Vec::new();
        for (index, piece) in pieces.iter().enumerate() {
            if (piece.points[0].0 - level).abs() < 1e-12 * scale {
                ends.push((index, false));
            }
            if (piece.points.last().expect("nonempty").0 - level).abs() < 1e-12 * scale {
                ends.push((index, true));
            }
        }
        let value = |pieces: &Vec<Piece>, (index, last): (usize, bool)| {
            if last { pieces[index].points.last().expect("nonempty").1 } else { pieces[index].points[0].1 }
        };
        for i in 0..ends.len() {
            let anchor = value(&pieces, ends[i]);
            for j in i + 1..ends.len() {
                let other = value(&pieces, ends[j]);
                let d = engine.o_difference(other, anchor);
                if d.abs() < tolerance && d != 0.0 {
                    let (index, last) = ends[j];
                    let target = other - d;
                    let points = &mut pieces[index].points;
                    if last {
                        points.last_mut().expect("nonempty").1 = target;
                    } else {
                        points[0].1 = target;
                    }
                }
            }
        }
    }

    // --- Level segments: where the intervals above and below a level differ.
    let mut all_levels: Vec<f64> = band_ends.iter().flat_map(|e| [e.0, e.1]).collect();
    all_levels.sort_by(|a, b| a.total_cmp(b));
    all_levels.dedup_by(|b, a| (*b - *a).abs() < 1e-12 * scale);
    for &level in &all_levels {
        let mut below: Vec<(f64, f64)> = Vec::new();
        let mut above: Vec<(f64, f64)> = Vec::new();
        for (a, b, pairs) in &band_ends {
            for &(lo, hi) in pairs {
                let (lo, hi) = (&pieces[lo].points, &pieces[hi].points);
                if (b - level).abs() < 1e-12 * scale {
                    below.push((lo.last().expect("nonempty").1, hi.last().expect("nonempty").1));
                }
                if (a - level).abs() < 1e-12 * scale {
                    above.push((lo[0].1, hi[0].1));
                }
            }
        }
        let tolerance = if engine.angular { 1e-9 * scale / level.max(1e-12) } else { 1e-9 * scale };
        for (lo, hi) in symmetric_difference(&below, &above, engine.angular, tolerance) {
            let steps = 2;
            pieces.push(Piece {
                points: (0..=steps)
                    .map(|k| (level, lo + (hi - lo) * k as f64 / steps as f64))
                    .collect(),
                sigma: (0..=steps).map(|k| k as f64 / steps as f64).collect(),
                level_segment: true,
            });
        }
    }

    if std::env::var("BREP_DEBUG_SOLID_SWEEP").is_ok() {
        eprintln!("solid sweep: breaks {breaks:?}");
        for piece in &pieces {
            let a = piece.points[0];
            let b = *piece.points.last().expect("nonempty");
            eprintln!(
                "  piece {} ({:.9}, {:.9}) -> ({:.9}, {:.9}) n={}",
                if piece.level_segment { "LEVEL" } else { "edge " },
                a.0, a.1, b.0, b.1, piece.points.len()
            );
            if std::env::var("BREP_DEBUG_SOLID_SWEEP").as_deref() == Ok("2") {
                for p in piece.points.iter().take(6) {
                    eprintln!("      {:.12} {:.12}", p.0, p.1);
                }
            }
        }
    }
    // --- At a crest the walls have met: the top arc is no boundary of what stays.
    if let Some(level) = crest {
        pieces.retain(|piece| !(piece.level_segment && (piece.points[0].0 - level).abs() < 1e-9 * scale));
    }
    // --- Assemble loops and build the bodies.
    let loops = assemble_loops(&engine, pieces)?;
    let mut bodies = Vec::new();
    for pieces in loops {
        bodies.push(build_body(&engine, &pieces)?);
    }
    if crest.is_some() {
        let [kept] = bodies.as_slice() else {
            return Err(refuse("whole_level", "what stays below the crest is not one body".into()));
        };
        return clip_less(clip, frame_base(&engine), kept);
    }
    Ok(bodies)
}

/// Replace a screw loft's side surfaces and helical edges by quintic splines
/// through the exact screw and its first two derivatives. A cubic interpolant
/// through `segments + 1` sections misses a turn by h⁴; these by h⁶, which is
/// what lets a long fine thread be built with few spans. Sections, caps and
/// parameters are the loft's, so the trims stay as they are.
fn carry_along_screw(body: &mut BrepSolid, engine: &Engine, c: f64, span: f64, segments: usize) -> Result<(), KernelRefusal> {
    let origin = engine.clip.axis_origin;
    let ez = engine.frame.ez;
    // The six Bézier points of each span collapse to three a span, C² at the joints.
    let carry = |start: Vec3| -> Vec<Vec3> {
        let h = span / segments as f64;
        let axial = ez.scale(start.sub(origin).dot(ez));
        let radial = start.sub(origin).sub(axial);
        let at = |k: usize| {
            let angle = h * k as f64;
            let r = radial.rotate_about(ez, angle);
            let value = origin.add(axial).add(r).add(ez.scale(c * angle));
            (value, ez.cross(r).add(ez.scale(c)), r.scale(-1.0))
        };
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
    };
    let knots = |lo: f64, hi: f64| -> Vec<f64> {
        let mut knots = vec![lo; 6];
        for k in 1..segments {
            knots.extend([lo + (hi - lo) * k as f64 / segments as f64; 3]);
        }
        knots.extend([hi; 6]);
        knots
    };
    let point = |p: &crate::Vec4| Vec3::new(p.x / p.w, p.y / p.w, p.z / p.w);
    let moved = |p: &crate::Vec4, angle: f64| -> Vec3 {
        rotate(point(p), origin, ez, angle).add(ez.scale(c * angle))
    };
    let tolerance = 1e-7 * engine.scale;
    for face in body.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let rows = &face.surface.control_points;
        if rows[0].len() != segments + 1 {
            continue; // a cap
        }
        // The loft's columns must already be this screw, from section to section.
        if rows.iter().any(|row| moved(&row[0], span).sub(point(&row[segments])).length() > tolerance) {
            return Err(refuse("screw_loft", "a lofted wall does not follow the screw".into()));
        }
        let lifted: Vec<Vec<crate::Vec4>> = rows
            .iter()
            .map(|row| {
                let w = row[0].w;
                carry(point(&row[0])).into_iter().map(|q| crate::Vec4::from_point(q, w)).collect()
            })
            .collect();
        let v = &face.surface.knots_v;
        let (lo, hi) = (v[0], *v.last().expect("knots"));
        face.surface = crate::NurbsSurface::new(face.surface.degree_u, 5, face.surface.knots_u.clone(), knots(lo, hi), lifted)
            .or_refuse(KernelStage::Fragment, "screw_surface")?;
    }
    let height = c * span;
    let points: rustc_hash::FxHashMap<u64, Vec3> = body.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    for edge in &mut body.edges {
        let (from, to) = (points[&edge.start_vertex_id], points[&edge.end_vertex_id]);
        let rise = to.sub(from).dot(ez);
        if (rise.abs() - height.abs()).abs() > tolerance {
            continue; // lies in a cap
        }
        let domain = edge.curve.domain().or_refuse(KernelStage::Fragment, "domain")?;
        let forward = rise * height > 0.0;
        let mut path = carry(if forward { from } else { to });
        if !forward {
            path.reverse();
        }
        edge.curve = NurbsCurve::new(5, knots(domain[0], domain[1]), path.into_iter().map(|q| crate::Vec4::from_point(q, 1.0)).collect())
            .or_refuse(KernelStage::Fragment, "screw_edge")?;
    }
    Ok(())
}

/// `rows` copies of `row` along the screw, each the image of the last, sharing
/// the section edges and vertices between neighbours; only the first bottom cap
/// and the last top cap stay.
fn stack_screw_rows(
    row: &BrepSolid,
    engine: &Engine,
    c: f64,
    span: f64,
    segments: usize,
    rows: usize,
) -> Result<BrepSolid, KernelRefusal> {
    let origin = engine.clip.axis_origin;
    let ez = engine.frame.ez;
    let moved = |x: Vec3, copy: usize| -> Vec3 {
        let angle = span * copy as f64;
        rotate(x, origin, ez, angle).add(ez.scale(c * angle))
    };
    let tolerance = 1e-7 * engine.scale;
    let height = |x: Vec3| x.sub(origin).dot(ez);
    let points: rustc_hash::FxHashMap<u64, Vec3> = row.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for vertex in &row.vertices {
        low = low.min(height(vertex.point));
        high = high.max(height(vertex.point));
    }
    let at = |id: u64, level: f64| (height(points[&id]) - level).abs() <= tolerance;
    let in_level = |edge: &crate::topology::EdgeRecord, level: f64| at(edge.start_vertex_id, level) && at(edge.end_vertex_id, level);
    let (bottom, top) = if c * span > 0.0 { (low, high) } else { (high, low) };
    // Each bottom vertex and edge of a copy is a top one of the copy before.
    let mut vertex_below: rustc_hash::FxHashMap<u64, u64> = Default::default();
    for vertex in row.vertices.iter().filter(|vertex| at(vertex.id, bottom)) {
        let up = moved(vertex.point, 1);
        let below = row.vertices.iter().find(|other| at(other.id, top) && other.point.sub(up).length() <= tolerance);
        let Some(below) = below else { return Err(refuse("screw_rows", "a row's section does not meet the next".into())) };
        vertex_below.insert(vertex.id, below.id);
    }
    let mut edge_below: rustc_hash::FxHashMap<u64, u64> = Default::default();
    for edge in row.edges.iter().filter(|edge| in_level(edge, bottom)) {
        let ends = (vertex_below[&edge.start_vertex_id], vertex_below[&edge.end_vertex_id]);
        let below = row.edges.iter().find(|other| in_level(other, top) && (other.start_vertex_id, other.end_vertex_id) == ends);
        let Some(below) = below else { return Err(refuse("screw_rows", "a row's section edge does not meet the next".into())) };
        edge_below.insert(edge.id, below.id);
    }
    let face_ids = row.shells.iter().flat_map(|shell| &shell.faces).flat_map(|face| {
        std::iter::once(face.id).chain(face.loops.iter().flat_map(|l| std::iter::once(l.id).chain(l.coedges.iter().map(|c| c.id))))
    });
    let stride = row
        .vertices
        .iter()
        .map(|v| v.id)
        .chain(row.edges.iter().map(|e| e.id))
        .chain(face_ids)
        .max()
        .unwrap_or(0)
        + 1;
    let vertex_id = |id: u64, copy: usize| match vertex_below.get(&id) {
        Some(&below) if copy > 0 => below + stride * (copy as u64 - 1),
        _ => id + stride * copy as u64,
    };
    let edge_id = |id: u64, copy: usize| match edge_below.get(&id) {
        Some(&below) if copy > 0 => below + stride * (copy as u64 - 1),
        _ => id + stride * copy as u64,
    };
    let is_cap = |face: &crate::FaceRecord| face.surface.control_points[0].len() != 3 * segments + 3;
    let cap_level = |face: &crate::FaceRecord, level: f64| {
        face.loops.iter().flat_map(|l| &l.coedges).all(|coedge| {
            row.edges.iter().find(|edge| edge.id == coedge.edge_id).is_some_and(|edge| in_level(edge, level))
        })
    };
    let mut stacked = row.clone();
    stacked.mass_properties_cache = Default::default();
    stacked.vertices.clear();
    stacked.edges.clear();
    for shell in &mut stacked.shells {
        shell.faces.clear();
    }
    for copy in 0..rows {
        for vertex in &row.vertices {
            if copy == 0 || !vertex_below.contains_key(&vertex.id) {
                stacked.vertices.push(crate::topology::VertexRecord { id: vertex_id(vertex.id, copy), point: moved(vertex.point, copy) });
            }
        }
        for edge in &row.edges {
            if copy == 0 || !edge_below.contains_key(&edge.id) {
                stacked.edges.push(crate::topology::EdgeRecord {
                    id: edge_id(edge.id, copy),
                    curve: transform_curve(&edge.curve, |x| moved(x, copy))?,
                    t0: edge.t0,
                    t1: edge.t1,
                    start_vertex_id: vertex_id(edge.start_vertex_id, copy),
                    end_vertex_id: vertex_id(edge.end_vertex_id, copy),
                    degenerate: edge.degenerate,
                    name: edge.name.clone(),
                });
            }
        }
        for (target, shell) in stacked.shells.iter_mut().zip(&row.shells) {
            for face in &shell.faces {
                if is_cap(face) && !((copy == 0 && cap_level(face, bottom)) || (copy + 1 == rows && cap_level(face, top))) {
                    continue;
                }
                let control_points = face
                    .surface
                    .control_points
                    .iter()
                    .map(|line| {
                        line.iter()
                            .map(|p| {
                                let w = p.w;
                                crate::Vec4::from_point(moved(Vec3::new(p.x / w, p.y / w, p.z / w), copy), w)
                            })
                            .collect()
                    })
                    .collect();
                let surface = crate::NurbsSurface::new(
                    face.surface.degree_u,
                    face.surface.degree_v,
                    face.surface.knots_u.clone(),
                    face.surface.knots_v.clone(),
                    control_points,
                )
                .or_refuse(KernelStage::Fragment, "screw_rows")?;
                let mut copied = face.clone();
                copied.surface = surface;
                copied.id += stride * copy as u64;
                for loop_record in &mut copied.loops {
                    loop_record.id += stride * copy as u64;
                    for coedge in &mut loop_record.coedges {
                        coedge.id += stride * copy as u64;
                        coedge.edge_id = edge_id(coedge.edge_id, copy);
                    }
                }
                target.faces.push(copied);
            }
        }
    }
    let issues = stacked.validate();
    if issues.iter().any(|issue| issue.severity == "error") {
        return Err(refuse("screw_rows", format!("the stacked thread is invalid: {issues:?}")));
    }
    Ok(stacked)
}

/// Where the clip's solid starts, on its axis.
fn frame_base(engine: &Engine) -> Vec3 {
    engine.frame.world([0.0, 0.0, engine.clip.axial[0]])
}

/// The whole clip cylinder with `kept` taken out of it.
fn clip_less(clip: &SweepClip, base: Vec3, kept: &BrepSolid) -> Result<Vec<BrepSolid>, KernelRefusal> {
    let direction = clip.axis_direction.normalized().or_refuse(KernelStage::Collect, "clip_axis")?;
    let cylinder = crate::make_cylinder_brep(base, direction, clip.radius[1], clip.axial[1] - clip.axial[0])
        .or_refuse(KernelStage::Fragment, "clip_cylinder")?;
    if thread_rows_on() {
        if let Some(direct) = clip_less_direct(&cylinder, base, direction, kept)? {
            return Ok(vec![direct]);
        }
    }
    let r = crate::boolean_operation(&cylinder, kept, crate::BooleanOperation::Subtract, &Default::default())?;
    Ok(vec![r])
}

/// `cylinder` with `kept` taken out, where `kept` runs from cap to cap inside
/// the wall: nothing crosses, so the result is written down: `kept`'s end loops
/// become holes in the caps and its sides turn inside out. `None` when the
/// bodies are not that shape; the caller takes the boolean then.
fn clip_less_direct(
    cylinder: &BrepSolid,
    base: Vec3,
    direction: Vec3,
    kept: &BrepSolid,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    if kept.shells.len() != 1 || cylinder.shells.len() != 1 {
        return Ok(None);
    }
    let mut stride = 0;
    for shell in &cylinder.shells {
        for face in &shell.faces {
            stride = stride.max(face.id);
            for loop_record in &face.loops {
                stride = stride.max(loop_record.id);
                stride = loop_record.coedges.iter().fold(stride, |m, c| m.max(c.id));
            }
        }
    }
    stride = cylinder.vertices.iter().fold(stride, |m, v| m.max(v.id));
    stride = cylinder.edges.iter().fold(stride, |m, e| m.max(e.id)) + 1;
    let corners = |surface: &crate::NurbsSurface| -> Result<([f64; 4], [Vec3; 3]), String> {
        let (u0, u1) = (surface.knots_u[0], *surface.knots_u.last().ok_or("knots")?);
        let (v0, v1) = (surface.knots_v[0], *surface.knots_v.last().ok_or("knots")?);
        Ok(([u0, u1, v0, v1], [surface.evaluate(u0, v0)?, surface.evaluate(u1, v0)?, surface.evaluate(u0, v1)?]))
    };
    let mut result = cylinder.clone();
    result.mass_properties_cache = Default::default();
    result.genus = 1;
    result.vertices.extend(kept.vertices.iter().map(|v| crate::topology::VertexRecord { id: v.id + stride, point: v.point }));
    for edge in &kept.edges {
        let mut edge = edge.clone();
        edge.id += stride;
        edge.start_vertex_id += stride;
        edge.end_vertex_id += stride;
        result.edges.push(edge);
    }
    let mut holes = 0;
    let turned = |loop_record: &crate::topology::LoopRecord| -> Result<crate::topology::LoopRecord, KernelRefusal> {
        let mut coedges = Vec::with_capacity(loop_record.coedges.len());
        for coedge in loop_record.coedges.iter().rev() {
            coedges.push(crate::topology::CoedgeRecord {
                id: coedge.id + stride,
                edge_id: coedge.edge_id + stride,
                forward: !coedge.forward,
                pcurve: coedge.pcurve.reversed().or_refuse(KernelStage::Fragment, "clip_less_pcurve")?,
            });
        }
        Ok(crate::topology::LoopRecord { id: loop_record.id + stride, coedges })
    };
    for face in &kept.shells[0].faces {
        let mut loops = Vec::with_capacity(face.loops.len());
        for loop_record in &face.loops {
            loops.push(turned(loop_record)?);
        }
        if !face.surface.is_affine().unwrap_or(false) {
            result.shells[0].faces.push(crate::FaceRecord {
                id: face.id + stride,
                surface: face.surface.clone(),
                same_sense: !face.same_sense,
                loops,
                name: face.name.clone(),
            });
            continue;
        }
        // An end: its loops are holes in the cap at its height, read in that cap's chart.
        let (from_box, from) = corners(&face.surface).or_refuse(KernelStage::Fragment, "clip_less_chart")?;
        let height = from[0].sub(base).dot(direction);
        let mut caps = result.shells[0].faces.iter_mut().filter(|cap| {
            cap.surface.is_affine().unwrap_or(false)
                && cap.surface.evaluate(cap.surface.knots_u[0], cap.surface.knots_v[0]).is_ok_and(|p| (p.sub(base).dot(direction) - height).abs() < 1e-9 * (1.0 + height.abs()))
        });
        let (Some(cap), None) = (caps.next(), caps.next()) else { return Ok(None) };
        let (to_box, to) = corners(&cap.surface).or_refuse(KernelStage::Fragment, "clip_less_chart")?;
        let (eu, ev) = (to[1].sub(to[0]), to[2].sub(to[0]));
        if eu.dot(ev).abs() > 1e-9 * eu.length() * ev.length() {
            return Ok(None);
        }
        let chart = |p: Vec3| {
            let d = p.sub(to[0]);
            [
                to_box[0] + (to_box[1] - to_box[0]) * d.dot(eu) / eu.dot(eu),
                to_box[2] + (to_box[3] - to_box[2]) * d.dot(ev) / ev.dot(ev),
            ]
        };
        let source = |a: f64, b: f64| from[0].add(from[1].sub(from[0]).scale(a)).add(from[2].sub(from[0]).scale(b));
        let (origin, across, up) = (chart(source(0.0, 0.0)), chart(source(1.0, 0.0)), chart(source(0.0, 1.0)));
        for loop_record in &mut loops {
            for coedge in &mut loop_record.coedges {
                let curve = &coedge.pcurve;
                let control_points = curve
                    .control_points
                    .iter()
                    .map(|p| {
                        let a = (p.x / p.w - from_box[0]) / (from_box[1] - from_box[0]);
                        let b = (p.y / p.w - from_box[2]) / (from_box[3] - from_box[2]);
                        let x = origin[0] + a * (across[0] - origin[0]) + b * (up[0] - origin[0]);
                        let y = origin[1] + a * (across[1] - origin[1]) + b * (up[1] - origin[1]);
                        crate::Vec4 { x: x * p.w, y: y * p.w, z: p.z, w: p.w }
                    })
                    .collect();
                coedge.pcurve = NurbsCurve::new(curve.degree, curve.knots.clone(), control_points)
                    .or_refuse(KernelStage::Fragment, "clip_less_pcurve")?;
            }
        }
        cap.loops.extend(loops);
        holes += 1;
    }
    if holes != 2 || result.validate().iter().any(|issue| issue.severity == "error") {
        return Ok(None);
    }
    Ok(Some(result))
}

/// The level above which a screw's sweep covers every angle, up to the clip,
/// when the material between its turns narrows to nothing there; `None` when
/// the sweep leaves something at the clip's edge.
fn crest_level(engine: &Engine) -> Result<Option<f64>, KernelRefusal> {
    let whole = |level: f64| -> Result<bool, KernelRefusal> {
        match engine.intervals_at(level) {
            Ok(_) => Ok(false),
            Err(error) if error.message.contains(WHOLE_LEVEL) => Ok(true),
            Err(error) => Err(error),
        }
    };
    let [l0, l1] = engine.clip.radius;
    let scan = 64;
    let level_at = |k: usize| l0 + (l1 - l0) * k as f64 / scan as f64;
    if !whole(l1)? {
        return Ok(None);
    }
    let mut first = scan;
    while first > 0 && whole(level_at(first - 1))? {
        first -= 1;
    }
    if first == 0 {
        return Ok(None);
    }
    let (mut below, mut above) = (level_at(first - 1), level_at(first));
    for _ in 0..60 {
        let middle = 0.5 * (below + above);
        if whole(middle)? {
            above = middle;
        } else {
            below = middle;
        }
    }
    // The material must narrow to nothing there, not stop at some width. Near
    // the end the walls have already crossed unseen (the oracle stops seeing the
    // gap a little past the crest), so the crest is where the gap, measured
    // well below, extrapolates to zero.
    let gap = |level: f64| -> Result<f64, KernelRefusal> {
        Ok(TAU - engine.intervals_at(level)?.iter().map(|interval| interval.hi - interval.lo).sum::<f64>())
    };
    let step = 4e-5 * (l1 - l0);
    let (x0, x1, x2) = (below - 3.0 * step, below - 2.0 * step, below - step);
    let (g0, g1, g2) = (gap(x0)?, gap(x1)?, gap(x2)?);
    if !(g2 > 0.0 && g1 > g2 && g0 > g1) || g2 > 16.0 * (g1 - g2) {
        return Ok(None);
    }
    let curve = |x: f64| {
        g0 * (x - x1) * (x - x2) / ((x0 - x1) * (x0 - x2))
            + g1 * (x - x0) * (x - x2) / ((x1 - x0) * (x1 - x2))
            + g2 * (x - x0) * (x - x1) / ((x2 - x0) * (x2 - x1))
    };
    let (mut lo, mut hi) = (x2, below + step);
    if curve(hi) > 0.0 {
        return Ok(None);
    }
    for _ in 0..80 {
        let middle = 0.5 * (lo + hi);
        if curve(middle) > 0.0 {
            lo = middle;
        } else {
            hi = middle;
        }
    }
    Ok(Some(0.5 * (lo + hi)))
}

fn find_events(
    engine: &Engine,
    a: f64,
    sa: &[Interval],
    b: f64,
    sb: &[Interval],
    breaks: &mut Vec<f64>,
    depth: usize,
) -> Result<(), KernelRefusal> {
    if compatible(engine, sa, sb) {
        return Ok(());
    }
    if depth <= 2 && std::env::var("BREP_DEBUG_SOLID_SWEEP").is_ok() {
        let summary = |s: &[Interval]| -> Vec<(usize, usize, f64, f64)> {
            s.iter().map(|x| (x.lo_source, x.hi_source, (x.lo * 1e4).round() / 1e4, (x.hi * 1e4).round() / 1e4)).collect()
        };
        eprintln!("  find_events depth {depth} [{a:.9}, {b:.9}] {:?}\n      -> {:?} counters {:?}", summary(sa), summary(sb), engine.counters.get());
    }
    if b - a < 1e-13 * engine.scale || depth > 60 {
        breaks.push(0.5 * (a + b));
        return Ok(());
    }
    let m = 0.5 * (a + b);
    let sm = engine.intervals_at(m)?;
    // Both halves stay incompatible only where the reading flickers; follow the
    // half that changes first and let the other be found from the scan above.
    if !compatible(engine, sa, &sm) && !compatible(engine, &sm, sb) && depth > 8 {
        breaks.push(m);
        return Ok(());
    }
    find_events(engine, a, sa, m, &sm, breaks, depth + 1)?;
    find_events(engine, m, &sm, b, sb, breaks, depth + 1)
}

/// The candidate of `source` at `level` nearest `guess` (or its critical point
/// at that level); `fallback` when the source has none there.
fn end_value(engine: &Engine, source: usize, level: f64, guess: f64, fallback: f64) -> f64 {
    let mut candidates = Vec::new();
    engine.source_candidates(source, level, &mut candidates);
    let mut options: Vec<f64> = candidates.iter().map(|c| c.0).collect();
    for critical in &engine.critical[source] {
        if (critical.0 - level).abs() < 1e-7 * engine.scale {
            options.push(critical.1);
        }
    }
    let mut best = fallback;
    let mut distance = f64::INFINITY;
    for option in options {
        let d = engine.o_difference(option, guess).abs();
        if d < distance {
            distance = d;
            best = guess + engine.o_difference(option, guess);
        }
    }
    best
}

/// Sample a band (a, b) of constant structure; return its end pieces and the
/// intervals at its bottom and top.
#[allow(clippy::type_complexity)]
fn sample_band(
    engine: &Engine,
    a: f64,
    b: f64,
    mid: &[Interval],
    fold_ends: &[f64],
) -> Result<(Vec<Piece>, Vec<(f64, f64)>, Vec<(f64, f64)>), KernelRefusal> {
    let count = mid.len();
    let n = 24usize;
    // Next to a fold the extremum Newton flickers; keep every sample at least
    // this far from a fold end (the end itself reads the fold's critical point).
    let near_fold = fold_ends.iter().any(|f| (f - a).abs() < 1e-12 * engine.scale || (f - b).abs() < 1e-12 * engine.scale);
    let n_cap = if near_fold {
        let keep = (1e-4 * engine.scale).min((b - a) / 50.0);
        // smallest σ step: (b − a)(1 − cos(π/n))/2 ≥ keep
        let ratio = (1.0 - 2.0 * keep / (b - a)).clamp(-1.0, 1.0);
        ((PI / ratio.acos()).floor() as usize).max(8)
    } else {
        192
    };
    let mut n = n.min(n_cap);
    loop {
        let sigma: Vec<f64> = (0..=n).map(|k| k as f64 / n as f64).collect();
        let level_at = |s: f64| a + (b - a) * (1.0 - (PI * s).cos()) / 2.0;
        // Track every interval from the middle sample outward.
        let mut lo = vec![vec![f64::NAN; n + 1]; count];
        let mut hi = vec![vec![f64::NAN; n + 1]; count];
        let mut lo_src = vec![vec![0usize; n + 1]; count];
        let mut hi_src = vec![vec![0usize; n + 1]; count];
        let mid_index = n / 2;
        let mid_intervals = engine.intervals_at(level_at(sigma[mid_index]))?;
        if mid_intervals.len() != count {
            return Err(refuse("band_structure", format!("the structure changes inside the band {a}..{b}")));
        }
        for (i, interval) in mid_intervals.iter().enumerate() {
            lo[i][mid_index] = interval.lo;
            hi[i][mid_index] = interval.hi;
            lo_src[i][mid_index] = interval.lo_source;
            hi_src[i][mid_index] = interval.hi_source;
        }
        for direction in [-1i64, 1] {
            let mut k = mid_index as i64 + direction;
            while k > 0 && (k as usize) < n {
                let ku = k as usize;
                let previous = (ku as i64 - direction) as usize;
                let intervals = engine.intervals_at(level_at(sigma[ku]))?;
                if intervals.len() != count {
                    return Err(refuse("band_structure", format!("the structure changes inside the band {a}..{b}")));
                }
                for i in 0..count {
                    let centre = 0.5 * (lo[i][previous] + hi[i][previous]);
                    let nearest = intervals
                        .iter()
                        .min_by(|x, y| {
                            let dx = engine.o_difference(0.5 * (x.lo + x.hi), centre).abs();
                            let dy = engine.o_difference(0.5 * (y.lo + y.hi), centre).abs();
                            dx.total_cmp(&dy)
                        })
                        .expect("nonempty");
                    let shift = engine.o_difference(nearest.lo, lo[i][previous]) - (nearest.lo - lo[i][previous]);
                    lo[i][ku] = nearest.lo + shift;
                    hi[i][ku] = nearest.hi + shift;
                    lo_src[i][ku] = nearest.lo_source;
                    hi_src[i][ku] = nearest.hi_source;
                }
                k += direction;
            }
        }
        // Ends: the same source's value at exactly the band's end levels,
        // nearest the linear extrapolation of the two nearest samples.
        for i in 0..count {
            for (end, inner, inner2, inner3, level) in [(0usize, 1usize, 2usize, 3usize, a), (n, n - 1, n - 2, n - 3, b)] {
                // A quadratic in the sample index through the three nearest samples.
                // The levels cluster as σ² at the ends, so a regular end is quadratic
                // in σ and a √ end (a tip) linear: this is exact for both, where
                // `2v₁ − v₂` misses the first by the slope times a sample spacing. A
                // source whose domain ends at this level has no candidate there, so
                // the estimate IS the end value (1e-3 on the bar swung on its edge).
                let near = |values: &Vec<f64>| 2.0 * values[inner] - values[inner2];
                let quadratic = |values: &Vec<f64>| 3.0 * values[inner] - 3.0 * values[inner2] + values[inner3];
                lo[i][end] = end_value(engine, lo_src[i][inner], level, near(&lo[i]), quadratic(&lo[i]));
                hi[i][end] = end_value(engine, hi_src[i][inner], level, near(&hi[i]), quadratic(&hi[i]));
            }
        }
        // Accuracy: compare the fitted ends at the σ mid-points with exact values.
        let mut worst = 0.0f64;
        let mut worst_at = (0usize, 0usize, 0usize, 0.0f64, 0.0f64);
        for i in 0..count {
            for (side, values) in [&lo[i], &hi[i]].into_iter().enumerate() {
                let points: Vec<Vec3> = (0..=n)
                    .map(|k| reduced_to_plane(engine, level_at(sigma[k]), values[k]))
                    .collect();
                let curve = fit_piece(&points)?;
                let parameters = chord_parameters(&points);
                // The end spans too: an end value is read differently from the samples.
                for k in (1..n - 1).step_by((n / 8).max(1)).chain([0, n - 1]) {
                    let s = 0.5 * (sigma[k] + sigma[k + 1]);
                    let level = level_at(s);
                    let intervals = engine.intervals_at(level)?;
                    let guess = 0.5 * (values[k] + values[k + 1]);
                    let exact = intervals
                        .iter()
                        .flat_map(|x| [x.lo, x.hi])
                        .map(|o| guess + engine.o_difference(o, guess))
                        .min_by(|x, y| (x - guess).abs().total_cmp(&(y - guess).abs()))
                        .unwrap_or(guess);
                    let truth = reduced_to_plane(engine, level, exact);
                    let bracket = (parameters[k.saturating_sub(1)], parameters[(k + 2).min(n)]);
                    let off = distance_in_bracket(&curve, truth, bracket.0, bracket.1)?;
                    if off > worst {
                        worst = off;
                        worst_at = (i, side, k, level, exact);
                    }
                }
            }
        }
        if n >= n_cap && worst > 1e-6 * engine.scale {
            return Err(refuse(
                "fit_tolerance",
                format!(
                    "a boundary piece of the band {a}..{b} stays {worst:.3e} off its envelope at {n} samples \
                     (interval {}, {}, sample {}, level {}, o {})",
                    worst_at.0,
                    if worst_at.1 == 0 { "lo" } else { "hi" },
                    worst_at.2,
                    worst_at.3,
                    worst_at.4
                ),
            ));
        }
        if worst <= 1e-8 * engine.scale || n >= n_cap {
            let mut pieces = Vec::new();
            for i in 0..count {
                for values in [&lo[i], &hi[i]] {
                    pieces.push(Piece {
                        points: (0..=n).map(|k| (level_at(sigma[k]), values[k])).collect(),
                        sigma: sigma.clone(),
                        level_segment: false,
                    });
                }
            }
            let bottoms = (0..count).map(|i| (lo[i][0], hi[i][0])).collect();
            let tops = (0..count).map(|i| (lo[i][n], hi[i][n])).collect();
            return Ok((pieces, bottoms, tops));
        }
        n = (n * 2).min(n_cap);
    }
}

/// Chord-length parameters for interpolating `points`, normalised to [0, 1].
/// Not the sampling's own `σ`: the levels cluster at both ends of a band, which
/// near a tip is what keeps a √ end smooth, but at a regular end it makes the
/// curve's speed vanish there, and an interpolant through a zero-speed end
/// cusps (the screw groove's wall folded where it met the closing arc).
fn chord_parameters(points: &[Vec3]) -> Vec<f64> {
    let mut parameters = vec![0.0; points.len()];
    for k in 1..points.len() {
        parameters[k] = parameters[k - 1] + points[k].sub(points[k - 1]).length();
    }
    let total = parameters[points.len() - 1].max(1e-300);
    for parameter in &mut parameters {
        *parameter /= total;
    }
    let last = points.len() - 1;
    parameters[last] = 1.0;
    parameters
}

fn fit_piece(points: &[Vec3]) -> Result<NurbsCurve, KernelRefusal> {
    interpolate_curve(points, 3, &chord_parameters(points)).or_refuse(KernelStage::Fragment, "piece_fit")
}

/// Distance from `target` to `curve` over the parameter bracket [a, b].
fn distance_in_bracket(curve: &NurbsCurve, target: Vec3, a: f64, b: f64) -> Result<f64, KernelRefusal> {
    let distance = |t: f64| -> Result<f64, KernelRefusal> {
        Ok(curve.evaluate(t).or_refuse(KernelStage::Fragment, "piece_eval")?.sub(target).length())
    };
    let ratio = 0.618_033_988_749_894_9;
    let (mut lo, mut hi) = (a, b);
    for _ in 0..80 {
        let c = hi - ratio * (hi - lo);
        let d = lo + ratio * (hi - lo);
        if distance(c)? < distance(d)? {
            hi = d;
        } else {
            lo = c;
        }
    }
    distance(0.5 * (lo + hi))
}

/// `below Δ above` as intervals of `o` (both lists at one level).
fn symmetric_difference(below: &[(f64, f64)], above: &[(f64, f64)], angular: bool, tolerance: f64) -> Vec<(f64, f64)> {
    // Normalise to [base, base + 2π) for angular values, splitting wrapped ones.
    let normalise = |list: &[(f64, f64)]| -> Vec<(f64, f64)> {
        let mut out = Vec::new();
        for &(lo, hi) in list {
            if angular {
                let shift = lo.rem_euclid(TAU) - lo;
                let (lo, hi) = (lo + shift, hi + shift);
                if hi > TAU {
                    out.push((lo, TAU));
                    out.push((0.0, hi - TAU));
                } else {
                    out.push((lo, hi));
                }
            } else {
                out.push((lo, hi));
            }
        }
        out
    };
    let a = normalise(below);
    let b = normalise(above);
    let mut cuts: Vec<f64> = a.iter().chain(&b).flat_map(|&(lo, hi)| [lo, hi]).collect();
    cuts.sort_by(|x, y| x.total_cmp(y));
    cuts.dedup_by(|y, x| (*y - *x).abs() < tolerance);
    let inside = |list: &Vec<(f64, f64)>, x: f64| list.iter().any(|&(lo, hi)| x > lo && x < hi);
    let mut out: Vec<(f64, f64)> = Vec::new();
    for w in cuts.windows(2) {
        let m = 0.5 * (w[0] + w[1]);
        if inside(&a, m) != inside(&b, m) && w[1] - w[0] > tolerance {
            if let Some(last) = out.last_mut() {
                if (last.1 - w[0]).abs() < tolerance {
                    last.1 = w[1];
                    continue;
                }
            }
            out.push((w[0], w[1]));
        }
    }
    if angular && out.len() > 1 {
        let first = out[0];
        let last = *out.last().expect("nonempty");
        if first.0.abs() < tolerance && (last.1 - TAU).abs() < tolerance {
            out.pop();
            out[0] = (last.0 - TAU, first.1);
        }
    }
    out
}

/// A reduced point on the section plane the body is built from.
fn reduced_to_plane(engine: &Engine, level: f64, o: f64) -> Vec3 {
    let clip = engine.clip;
    match engine.class {
        Class::Screw(c) => {
            let z = clip.axial[0];
            let alpha = o + z / c;
            engine.frame.world([level * alpha.cos(), level * alpha.sin(), z])
        }
        Class::Turn => {
            let theta = clip.angle.map(|a| a[0]).unwrap_or(0.0);
            engine.frame.world([level * theta.cos(), level * theta.sin(), o])
        }
        Class::Planar => engine.frame.world([level * o.cos(), level * o.sin(), clip.axial[0]]),
    }
}

/// Join pieces end to end into closed loops (in the section plane).
fn assemble_loops(engine: &Engine, pieces: Vec<Piece>) -> Result<Vec<Vec<Piece>>, KernelRefusal> {
    let tolerance = 1e-6 * engine.scale;
    let ends: Vec<(Vec3, Vec3)> = pieces
        .iter()
        .map(|piece| {
            let first = piece.points[0];
            let last = *piece.points.last().expect("nonempty");
            (reduced_to_plane(engine, first.0, first.1), reduced_to_plane(engine, last.0, last.1))
        })
        .collect();
    let mut used = vec![false; pieces.len()];
    let mut orders: Vec<Vec<(usize, bool)>> = Vec::new();
    for start in 0..pieces.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let mut order = vec![(start, false)];
        let origin = ends[start].0;
        let mut cursor = ends[start].1;
        let mut guard = 0;
        while cursor.sub(origin).length() > tolerance {
            guard += 1;
            if guard > pieces.len() {
                return Err(refuse("loop_open", "the section's boundary does not close".into()));
            }
            let mut next = None;
            let mut best = tolerance;
            for k in 0..pieces.len() {
                if used[k] {
                    continue;
                }
                for (reversed, end) in [(false, ends[k].0), (true, ends[k].1)] {
                    let d = end.sub(cursor).length();
                    if d < best {
                        best = d;
                        next = Some((k, reversed));
                    }
                }
            }
            let Some((k, reversed)) = next else {
                return Err(refuse(
                    "loop_open",
                    format!("the section's boundary does not close (a gap at {:?})", cursor),
                ));
            };
            used[k] = true;
            order.push((k, reversed));
            cursor = if reversed { ends[k].0 } else { ends[k].1 };
        }
        orders.push(order);
    }
    if orders.len() > 1 {
        // A loop inside another is a hole; not built here.
        let polygon = |order: &Vec<(usize, bool)>| -> Vec<Vec3> {
            order
                .iter()
                .flat_map(|&(k, _)| pieces[k].points.iter().map(|p| reduced_to_plane(engine, p.0, p.1)))
                .collect()
        };
        let polygons: Vec<Vec<Vec3>> = orders.iter().map(polygon).collect();
        for (i, a) in polygons.iter().enumerate() {
            for (j, b) in polygons.iter().enumerate() {
                if i != j && point_in_polygon(engine, a[0], b) {
                    return Err(refuse("section_hole", "the section has a hole, which is not built here".into()));
                }
            }
        }
    }
    let mut slots: Vec<Option<Piece>> = pieces.into_iter().map(Some).collect();
    Ok(orders
        .into_iter()
        .map(|order| {
            order
                .into_iter()
                .map(|(k, reversed)| {
                    let mut piece = slots[k].take().expect("each piece once");
                    if reversed {
                        piece.points.reverse();
                        piece.sigma = piece.sigma.iter().rev().map(|s| 1.0 - s).collect();
                    }
                    piece
                })
                .collect()
        })
        .collect())
}

fn point_in_polygon(engine: &Engine, p: Vec3, polygon: &[Vec3]) -> bool {
    let local = |x: Vec3| -> [f64; 2] {
        let l = engine.frame.local(x);
        match engine.class {
            Class::Turn => [l[0].hypot(l[1]), l[2]],
            _ => [l[0], l[1]],
        }
    };
    let q = local(p);
    let mut inside = false;
    for k in 0..polygon.len() {
        let a = local(polygon[k]);
        let b = local(polygon[(k + 1) % polygon.len()]);
        if (a[1] > q[1]) != (b[1] > q[1]) {
            let x = a[0] + (q[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]);
            if x > q[0] {
                inside = !inside;
            }
        }
    }
    inside
}

/// Fit each piece and build the body for one loop.
fn build_body(engine: &Engine, pieces: &[Piece]) -> Result<BrepSolid, KernelRefusal> {
    let clip = engine.clip;
    let mut profile: Vec<NurbsCurve> = Vec::new();
    // Snap shared ends so consecutive curves meet exactly.
    let count = pieces.len();
    let mut start_points: Vec<Vec3> = pieces
        .iter()
        .map(|p| reduced_to_plane(engine, p.points[0].0, p.points[0].1))
        .collect();
    // A level segment is built exactly from its own ends, so the piece after
    // it starts where it ends.
    for (index, piece) in pieces.iter().enumerate() {
        if piece.level_segment && engine.class != Class::Turn {
            let (level, o1) = *piece.points.last().expect("nonempty");
            start_points[(index + 1) % count] = reduced_to_plane(engine, level, o1);
        }
    }
    for (index, piece) in pieces.iter().enumerate() {
        let start = start_points[index];
        let end = start_points[(index + 1) % count];
        if piece.level_segment {
            let (level, o0) = piece.points[0];
            let o1 = piece.points.last().expect("nonempty").1;
            let curve = match engine.class {
                Class::Turn => make_line(start, end),
                Class::Screw(c) => {
                    let shift = clip.axial[0] / c;
                    level_arc(engine, level, o0 + shift, o1 + shift)
                }
                Class::Planar => level_arc(engine, level, o0, o1),
            }
            .or_refuse(KernelStage::Fragment, "level_segment")?;
            profile.push(curve);
            continue;
        }
        let mut points: Vec<Vec3> = piece
            .points
            .iter()
            .map(|p| reduced_to_plane(engine, p.0, p.1))
            .collect();
        points[0] = start;
        let last = points.len() - 1;
        points[last] = end;
        profile.push(fit_piece(&points)?);
    }
    match engine.class {
        Class::Screw(c) => {
            let span = (clip.axial[1] - clip.axial[0]) / c; // radians of screw
            // Over a crest the walls are re-carried below to high order, so 12 a turn is enough.
            let per_turn = if engine.crest_gap { 12.0 } else { 64.0 };
            // Every turn is the screw image of the first: build one and stack
            // copies, so no surface or edge runs the thread's whole length.
            let rows = if engine.crest_gap && thread_rows_on() { (span.abs() / TAU / TURNS_PER_ROW).round().max(1.0) as usize } else { 1 };
            let span = span / rows as f64;
            let sections = ((span.abs() / TAU * per_turn).ceil() as usize).max(8) + 1;
            let mut lofted = Vec::with_capacity(sections);
            for k in 0..sections {
                let angle = span * k as f64 / (sections - 1) as f64;
                let lift = engine.frame.ez.scale(c * angle);
                let moved: Vec<NurbsCurve> = profile
                    .iter()
                    .map(|curve| transform_curve(curve, |x| rotate(x, clip.axis_origin, engine.frame.ez, angle).add(lift)))
                    .collect::<Result<_, _>>()?;
                lofted.push(moved);
            }
            let mut body = loft_profile_brep(&lofted)?;
            if engine.crest_gap {
                carry_along_screw(&mut body, engine, c, span, sections - 1)?;
            }
            if rows > 1 {
                body = stack_screw_rows(&body, engine, c, span, sections - 1, rows)?;
            }
            Ok(body)
        }
        Class::Turn => {
            let angle = clip.angle.map(|a| a[1] - a[0]).unwrap_or(TAU);
            revolve_profile_brep(&profile, clip.axis_origin, engine.frame.ez, angle)
                .or_refuse(KernelStage::Fragment, "revolve")
        }
        Class::Planar => extrude_profile_brep(&profile, engine.frame.ez, clip.axial[1] - clip.axial[0]),
    }
}

fn level_arc(engine: &Engine, level: f64, a0: f64, a1: f64) -> Result<NurbsCurve, String> {
    let centre = engine.frame.world([0.0, 0.0, match engine.class {
        Class::Screw(_) | Class::Planar => engine.clip.axial[0],
        Class::Turn => 0.0,
    }]);
    let (start, end) = if a1 >= a0 { (a0, a1) } else { (a1, a0) };
    let arc = make_arc(centre, engine.frame.ex, engine.frame.ey, level, start, end)?;
    if a1 >= a0 {
        Ok(arc)
    } else {
        arc.reversed()
    }
}

fn transform_curve(curve: &NurbsCurve, map: impl Fn(Vec3) -> Vec3) -> Result<NurbsCurve, KernelRefusal> {
    let control_points = curve
        .control_points
        .iter()
        .map(|p| {
            let w = p.w;
            let x = map(Vec3::new(p.x / w, p.y / w, p.z / w));
            crate::Vec4 { x: x.x * w, y: x.y * w, z: x.z * w, w }
        })
        .collect();
    NurbsCurve::new(curve.degree, curve.knots.clone(), control_points).or_refuse(KernelStage::Fragment, "curve_new")
}

/// Refuse a screw or turn whose first or last position reaches the clip.
fn check_ends_clear(engine: &Engine) -> Result<(), KernelRefusal> {
    let clip = engine.clip;
    // Tool boundary samples at its own place (clip frame).
    let mut samples: Vec<[f64; 3]> = Vec::new();
    for (index, source) in engine.sources.iter().enumerate() {
        if let Some(grid) = &engine.grids[index] {
            for i in 0..grid.nu {
                for j in 0..grid.nv {
                    let (u, v) = Engine::grid_param(grid, i, j);
                    samples.push(engine.frame.local(engine.source_point(*source, u, v)));
                }
            }
        }
    }
    let margin = 1e-6 * engine.scale;
    for t in [engine.motion.from, engine.motion.to] {
        for p in &samples {
            let x = engine.frame.local(engine.motion_apply(t, engine.frame.world(*p)));
            let level = x[0].hypot(x[1]);
            if level > clip.radius[1] + margin || level < clip.radius[0] - margin {
                continue;
            }
            if x[2] < clip.axial[0] - margin || x[2] > clip.axial[1] + margin {
                continue;
            }
            if let Some([a0, a1]) = clip.angle {
                let angle = x[1].atan2(x[0]);
                let mid = 0.5 * (a0 + a1);
                let rel = wrap_pi(angle - mid);
                if rel.abs() > 0.5 * (a1 - a0) + 1e-9 {
                    continue;
                }
            }
            return Err(refuse(
                "ends_reach_clip",
                format!(
                    "the tool reaches the clip region at the motion's {} (t = {t}); the sweep is \
                     not invariant there, so the region would be wrong",
                    if t == engine.motion.from { "start" } else { "end" }
                ),
            ));
        }
    }
    Ok(())
}

