//! Coaxial revolution certificate (`BREP_SOUNDNESS_COAXIAL_REVOLUTION`, default
//! on since the tier P promotion of 2026-11-20, `=0` restores the full scan;
//! `BREP_SOUNDNESS_COAXIAL_REVOLUTION_CHECK=1` runs the full scan as well,
//! returns ITS report, and says so on stderr when the scan flags a body the
//! certificate passed).
//!
//! The class it accepts, and nothing else:
//! - every face of every shell is a FULL-TURN revolution of a straight segment
//!   or of a circular arc (one to four rational quadratic spans, each the
//!   standard arc of one circle, turning one way, less than a whole turn in
//!   all, rebuilt exactly from the circle to twice the slack, its radius at
//!   least `MIN_RADIUS` of the profile's size), its stored net
//!   what `make_revolution` rebuilds from its own generatrix through a full
//!   turn, to `DELTA` of the body's size, with every weight matching;
//! - every generatrix lies in its own frame's meridian half-plane (a skew line
//!   would turn into a hyperboloid), to `DELTA`;
//! - all the faces turn about ONE axis: parallel or opposite directions (their
//!   cross product under `DELTA`) and origins on one line (to `DELTA`);
//! - every face has one loop, and that loop is its whole parameter domain:
//!   straight pcurves chained end to end, each on one side of the domain
//!   rectangle, enclosing the rectangle's own area once;
//! - the meridian profile, each face's segment read in the (ρ, z) half-plane of
//!   that axis, is a set of simple chains matching the topology: a piece
//!   touches the axis only at an end, and that end is a pole (the face's loop
//!   side there is degenerate edges only); every other end meets exactly one
//!   other piece's end, of another face, where the two faces' loops share an
//!   edge; no two pieces come within `SEPARATED` of each other except at such
//!   a shared end (distances exact for lines and circles), and two that share
//!   one neither double back over each other nor meet again (a line and a
//!   circle, or two circles, meet at most twice; one circle continued turns
//!   less than a whole turn in all).
//!
//! Why that is the full scan's verdict: a point of a full-turn revolution about
//! the axis is in it iff its (ρ, z) is on the face's meridian segment, so two
//! such faces share a point only where their segments do, and the segments
//! share only the ends the topology names as the faces' common circle (or no
//! point at all). A revolution chart (θ, v) is injective where ρ > 0 apart
//! from the seam the scan identifies, and collapses only along a pole row, which
//! is the face's degenerate boundary; the whole-domain trim leaves no trim the
//! chart could fold. So the body has no crossing and no fold, and the scan
//! would find none to confirm.
//!
//! A sheet's per-face slab in the thickener (a pan's floor, a cup's wall, a
//! hemisphere's or a torus's skin) is such a body: the full scan tessellated
//! its revolved disks and caps to thousands of triangles, 40 to 200 ms a body
//! cold. Anything else declines to the scan.
use super::*;

/// Why a body was left to the full scan.
pub type Decline = &'static str;

/// Carrier recognition slack, relative to the body's size.
const DELTA: f64 = 1e-12;
/// Two profile ends this close (relative to the profile's extent) are one
/// point. A shared end read `c` apart, on carriers each within the slack `d`
/// of its piece, lets two pieces meeting at a sine `s` cross up to about
/// `(c + 2d) / s` from it; with `d` = `HIDDEN` less `c` and `MIN_SINE`, that is
/// under 2e-8 of the profile's size, below the scan's crossing length (1e-7 of
/// the diagonal) and straddle band.
const COINCIDENT: f64 = 1e-12;
/// The farthest a body's coordinates may run from the origin, in profile
/// sizes, for the world-scaled carrier slack to stay inside that bound.
const FAR: f64 = 4.0;
/// How deep, relative to the profile's size, a crossing the tolerances above
/// could hide may reach through a face: the shared end's reading plus both
/// carriers' slack, `COINCIDENT + 2·(2·DELTA·FAR + 1e-13)` (an arc's spans are
/// held to twice the slack and their weights to 1e-13), rounded up. The scan's
/// default straddle band (1e-6 of the diagonal) is four orders above it.
const HIDDEN: f64 = 2e-11;
/// Two profile segments not sharing an end must stay this far apart
/// (relative), and no end may sit between `COINCIDENT` and this from another.
const SEPARATED: f64 = 1e-7;
/// Two segments from one shared end closer in angle than this (on the same
/// side) decline as doubling back; see `COINCIDENT` for why it is this wide.
const MIN_SINE: f64 = 1e-3;
/// The least arc radius, relative to the profile's size. Two carriers that
/// share an end at a sine of at least `MIN_SINE` meet again (if at all) at
/// least `2·r·MIN_SINE` (a line and a circle) or `r·MIN_SINE` (two circles)
/// from it, so with this floor a second meeting is never inside `SEPARATED`
/// of the shared end, where the check below exempts it; and an arc this wide
/// stays straight to far under the slack over the reach of a hidden crossing.
const MIN_RADIUS: f64 = 1e-3;

pub(super) fn enabled() -> bool {
    crate::switch!("BREP_SOUNDNESS_COAXIAL_REVOLUTION", true)
}

pub(super) fn check_mode() -> bool {
    crate::switch!("BREP_SOUNDNESS_COAXIAL_REVOLUTION_CHECK", false)
}


pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    match verdict_within(solid, options.straddle_band) {
        Ok(()) => true,
        Err(reason) => {
            if std::env::var_os("BREP_DEBUG_COAXIAL_REVOLUTION").is_some() {
                eprintln!("soundness.coaxial_revolution decline: {reason}");
            }
            false
        }
    }
}

/// One face's meridian piece in the axis's (ρ, z) half-plane, with the edges
/// its loop runs along at each end (`edges[0]` at the generatrix's start).
#[derive(Clone, Debug)]
pub struct Segment {
    pub face: usize,
    pub piece: Piece,
    pub edges: [Vec<u64>; 2],
    pub degenerate: [bool; 2],
}

/// A meridian: a straight segment, or an arc of one circle turning less than a
/// whole turn, with its ends exactly as the generatrix's end controls.
#[derive(Clone, Copy, Debug)]
pub enum Piece {
    Line([[f64; 2]; 2]),
    /// `centre`, `radius`, the start's angle about the centre and the signed
    /// turn to the end (counter-clockwise positive in (ρ, z)).
    Arc { centre: [f64; 2], radius: f64, start: f64, sweep: f64, ends: [[f64; 2]; 2] },
}

/// The whole-body verdict: `Ok` where the body is in the class the module
/// documents, else the first reason it is not.
pub fn verdict(solid: &BrepSolid) -> Result<(), Decline> {
    verdict_within(solid, f64::INFINITY)
}

/// [`verdict`] for a scan whose straddle band is `straddle_band`: a crossing
/// the certificate's tolerances could hide reaches at most `HIDDEN` of the
/// profile's size through either face, and a scan asked to confirm anything
/// that shallow declines here instead.
pub fn verdict_within(solid: &BrepSolid, straddle_band: f64) -> Result<(), Decline> {
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    if faces.is_empty() {
        return Err("no faces");
    }
    let mut scale = 0.0f64;
    for face in &faces {
        for control in face.surface.control_points.iter().flatten() {
            let point = control.point().map_err(|_| "a control at infinity")?;
            if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite() && control.w.is_finite()) {
                return Err("a control is not finite");
            }
            scale = scale.max(point.x.abs()).max(point.y.abs()).max(point.z.abs());
        }
    }
    if !(scale.is_finite() && scale > 0.0) {
        return Err("no size");
    }
    let slack = DELTA * scale;
    let mut axis_line: Option<(Vec3, Vec3)> = None;
    let mut segments = Vec::with_capacity(faces.len());
    let mut low = [f64::INFINITY; 2];
    let mut high = [f64::NEG_INFINITY; 2];
    for (index, face) in faces.iter().enumerate() {
        let (frame, generatrix) = full_turn_meridian(&face.surface, slack)?;
        let (origin, axis) = *axis_line.get_or_insert((frame.origin, frame.axis));
        if frame.axis.cross(axis).length() > DELTA {
            return Err("a face turns about another direction");
        }
        let offset = frame.origin.sub(origin);
        if offset.sub(axis.scale(offset.dot(axis))).length() > slack {
            return Err("a face turns about a parallel axis");
        }
        // the generatrix's controls in the meridian half-plane: ρ along its
        // own frame's radial direction, z along the common axis
        let mut points = Vec::with_capacity(generatrix.control_points.len());
        let mut weights = Vec::with_capacity(generatrix.control_points.len());
        for control in &generatrix.control_points {
            let point = control.point().map_err(|_| "a control at infinity")?;
            let own = point.sub(frame.origin);
            if own.dot(frame.y_axis).abs() > slack || own.dot(frame.x_axis) < -slack {
                return Err("a generatrix off its meridian half-plane");
            }
            let at = [own.dot(frame.x_axis).max(0.0), point.sub(origin).dot(axis)];
            for k in 0..2 {
                low[k] = low[k].min(at[k]);
                high[k] = high[k].max(at[k]);
            }
            points.push(at);
            weights.push(control.w);
        }
        let piece = match generatrix.degree {
            1 => Piece::Line([points[0], points[1]]),
            _ => arc_from_spans(&points, &weights, slack)?,
        };
        let (edges, sides) = whole_domain_sides(face)?;
        let mut degenerate = [false; 2];
        for k in 0..2 {
            if !sides[k] {
                return Err("a loop leaves a domain side unrun");
            }
            let mut all = true;
            let mut any = false;
            for id in &edges[k] {
                let edge = solid.edges.iter().find(|edge| edge.id == *id).ok_or("a coedge names a missing edge")?;
                all &= edge.degenerate;
                any |= edge.degenerate;
            }
            if any && !all {
                return Err("a domain side mixes degenerate and real edges");
            }
            degenerate[k] = all && !edges[k].is_empty();
        }
        segments.push(Segment { face: index, piece, edges, degenerate });
    }
    // The profile's own extent, not its distance from the world origin, sets
    // the chain's tolerances; the carrier slack is read in world coordinates,
    // so a body far off the origin for its size declines (see `COINCIDENT`).
    let size = (high[0] - low[0]).hypot(high[1] - low[1]);
    if !(size.is_finite() && size > 0.0) || scale > FAR * size {
        return Err("the body sits far off the origin for its size");
    }
    if !(straddle_band >= HIDDEN * size) {
        return Err("the scan's straddle band is finer than the certificate can answer for");
    }
    chain_verdict(&segments, size)
}

/// A full-turn revolution of a straight span or of a circular arc's
/// rational quadratic spans: its frame and generatrix, where the stored net is
/// the one `make_revolution` rebuilds through a full turn to `slack`.
fn full_turn_meridian(surface: &NurbsSurface, slack: f64) -> Result<(crate::RevolutionFrame, NurbsCurve), Decline> {
    let n = surface.control_points.first().map_or(0, |row| row.len());
    let shape = match surface.degree_v {
        // one straight span
        1 => n == 2 && surface.knots_v.len() == 4,
        // one to four quadratic spans joined at double knots
        2 => n % 2 == 1 && (3..=9).contains(&n) && surface.knots_v.len() == n + 3,
        _ => false,
    };
    if surface.degree_u != 2
        || surface.control_points.len() != 9
        || surface.control_points.iter().any(|row| row.len() != n)
        || surface.knots_u.len() != 12
        || !shape
    {
        return Err("not a four-span turn of a line or an arc");
    }
    if surface.degree_v == 2 {
        // clamped, every interior knot doubled: each span its own conic
        let knots = &surface.knots_v;
        let clamped = knots[0] == knots[1] && knots[1] == knots[2] && knots[n] == knots[n + 1] && knots[n + 1] == knots[n + 2];
        let doubled = (3..n).step_by(2).all(|i| knots[i] == knots[i + 1] && knots[i] > knots[i - 1]) && knots[n] > knots[n - 1];
        if !(clamped && doubled) {
            return Err("the arc's spans are not separate conics");
        }
    }
    let structure = crate::revolution_structure(surface).ok_or("not a revolution")?;
    let generatrix = structure.generatrix;
    if generatrix.degree != surface.degree_v || generatrix.control_points.len() != n || generatrix.control_points.iter().any(|c| !(c.w > 0.0)) {
        return Err("the generatrix is not the net's own");
    }
    let rebuilt = crate::make_revolution(structure.frame.origin, structure.frame.axis, &generatrix, std::f64::consts::TAU)
        .map_err(|_| "the full turn does not rebuild")?;
    if rebuilt.knots_u.len() != surface.knots_u.len()
        || rebuilt.knots_v.len() != surface.knots_v.len()
        || rebuilt.knots_u.iter().zip(&surface.knots_u).any(|(a, b)| (a - b).abs() > DELTA)
        || rebuilt.knots_v.iter().zip(&surface.knots_v).any(|(a, b)| (a - b).abs() > DELTA)
        || rebuilt.control_points.len() != surface.control_points.len()
    {
        return Err("not a full turn");
    }
    for (row, stored) in rebuilt.control_points.iter().zip(&surface.control_points) {
        if row.len() != stored.len() {
            return Err("not a full turn");
        }
        // the points themselves and the weights' ratio, never homogeneous
        // coordinates (a net scaled to tiny weights would read any motion small)
        for (a, b) in row.iter().zip(stored) {
            let (Ok(pa), Ok(pb)) = (a.point(), b.point()) else {
                return Err("a control at infinity");
            };
            if pa.sub(pb).length() > slack || !(b.w > 0.0) || (a.w - b.w).abs() > DELTA * b.w {
                return Err("the net is not the full turn of its generatrix");
            }
        }
    }
    Ok((structure.frame, generatrix))
}

/// The circle a run of rational quadratic spans traces, by exact
/// reconstruction: the first span's tangent lines give the centre and radius,
/// and EVERY span must then be the standard arc of that circle between its own
/// ends (end controls on the circle, the middle control where the end tangents
/// meet, its weight ratio cos(half the span's turn)), all turning one way, less
/// than a whole turn in all. Positions to twice `slack`, weights to 1e-13.
fn arc_from_spans(points: &[[f64; 2]], weights: &[f64], slack: f64) -> Result<Piece, Decline> {
    let tolerance = 2.0 * slack;
    let (p0, p1, p2) = (points[0], points[1], points[2]);
    let (a, b) = ([p1[0] - p0[0], p1[1] - p0[1]], [p1[0] - p2[0], p1[1] - p2[1]]);
    let det = a[0] * b[1] - a[1] * b[0];
    if !(det.abs() > 0.0) {
        return Err("a span with parallel end tangents");
    }
    let (ra, rb) = (a[0] * p0[0] + a[1] * p0[1], b[0] * p2[0] + b[1] * p2[1]);
    let centre = [(ra * b[1] - rb * a[1]) / det, (a[0] * rb - b[0] * ra) / det];
    let radius = distance(p0, centre);
    if !(radius.is_finite() && radius > tolerance) {
        return Err("an arc without a radius");
    }
    let angle = |p: [f64; 2]| (p[1] - centre[1]).atan2(p[0] - centre[0]);
    let mut sweep = 0.0;
    for span in 0..(points.len() - 1) / 2 {
        let (q0, q1, q2) = (points[2 * span], points[2 * span + 1], points[2 * span + 2]);
        let (w0, w1, w2) = (weights[2 * span], weights[2 * span + 1], weights[2 * span + 2]);
        if (distance(q0, centre) - radius).abs() > tolerance || (distance(q2, centre) - radius).abs() > tolerance {
            return Err("a span's end off the circle");
        }
        let (u0, u2) = ([q0[0] - centre[0], q0[1] - centre[1]], [q2[0] - centre[0], q2[1] - centre[1]]);
        let turn = (u0[0] * u2[1] - u0[1] * u2[0]).atan2(u0[0] * u2[0] + u0[1] * u2[1]);
        if !(turn.abs() > 1e-6 && turn.abs() < std::f64::consts::PI - 1e-6) || (sweep != 0.0 && turn.signum() != f64::signum(sweep)) {
            return Err("a span turning the wrong way or too far");
        }
        let half = 0.5 * turn;
        let start = angle(q0);
        let reach = radius / half.cos();
        let middle = [centre[0] + reach * (start + half).cos(), centre[1] + reach * (start + half).sin()];
        if distance(q1, middle) > tolerance {
            return Err("a span's middle control off the tangents' meeting");
        }
        if !(w0 > 0.0) || (w2 - w0).abs() > 1e-13 * w0 || (w1 / w0 - half.cos()).abs() > 1e-13 {
            return Err("a span's weights are not the circle's");
        }
        sweep += turn;
    }
    if !(sweep.abs() < std::f64::consts::TAU - 1e-6) {
        return Err("an arc of a whole turn or more");
    }
    Ok(Piece::Arc { centre, radius, start: angle(points[0]), sweep, ends: [points[0], points[points.len() - 1]] })
}

/// The face's one loop as its whole domain: the edges it runs along the
/// v = v0 and v = v1 sides, and whether it runs each of them.
fn whole_domain_sides(face: &FaceRecord) -> Result<([Vec<u64>; 2], [bool; 2]), Decline> {
    let [record] = face.loops.as_slice() else {
        return Err("not one loop");
    };
    let surface = &face.surface;
    let (Some(&u0), Some(&u1), Some(&v0), Some(&v1)) =
        (surface.knots_u.first(), surface.knots_u.last(), surface.knots_v.first(), surface.knots_v.last())
    else {
        return Err("no domain");
    };
    if !(u1 > u0 && v1 > v0) {
        return Err("an empty domain");
    }
    let (eu, ev) = (DELTA * (u1 - u0).max(u0.abs()).max(u1.abs()), DELTA * (v1 - v0).max(v0.abs()).max(v1.abs()));
    let mut edges: [Vec<u64>; 2] = [Vec::new(), Vec::new()];
    let mut sides = [false; 2];
    let mut twice_area = 0.0;
    let mut previous: Option<[f64; 2]> = None;
    let mut first: Option<[f64; 2]> = None;
    for coedge in &record.coedges {
        let pcurve = &coedge.pcurve;
        if pcurve.degree != 1 || pcurve.control_points.len() < 2 || pcurve.control_points.iter().any(|c| !(c.w > 0.0)) {
            return Err("a pcurve is not straight");
        }
        let points: Vec<[f64; 2]> = pcurve
            .control_points
            .iter()
            .map(|c| c.point().map(|p| [p.x, p.y]))
            .collect::<Result<_, _>>()
            .map_err(|_| "a pcurve control at infinity")?;
        if points.iter().flatten().any(|value| !value.is_finite()) {
            return Err("a pcurve control is not finite");
        }
        let start = points[0];
        if let Some(at) = previous {
            if (at[0] - start[0]).abs() > eu || (at[1] - start[1]).abs() > ev {
                return Err("the loop's pcurves do not chain");
            }
        }
        first.get_or_insert(start);
        for pair in points.windows(2) {
            let (a, b) = (pair[0], pair[1]);
            let on = |value: f64, side: f64, slack: f64| (value - side).abs() <= slack;
            let along_v0 = on(a[1], v0, ev) && on(b[1], v0, ev);
            let along_v1 = on(a[1], v1, ev) && on(b[1], v1, ev);
            let along_u = (on(a[0], u0, eu) && on(b[0], u0, eu)) || (on(a[0], u1, eu) && on(b[0], u1, eu));
            if !(along_v0 || along_v1 || along_u) {
                return Err("a pcurve leaves the domain boundary");
            }
            if (b[0] - a[0]).abs() > eu {
                if along_v0 {
                    sides[0] = true;
                    edges[0].push(coedge.edge_id);
                }
                if along_v1 {
                    sides[1] = true;
                    edges[1].push(coedge.edge_id);
                }
            }
            twice_area += a[0] * b[1] - b[0] * a[1];
        }
        previous = points.last().copied();
    }
    let (Some(first), Some(last)) = (first, previous) else {
        return Err("an empty loop");
    };
    if (first[0] - last[0]).abs() > eu || (first[1] - last[1]).abs() > ev {
        return Err("the loop does not close");
    }
    let domain = (u1 - u0) * (v1 - v0);
    // fails closed: a NaN anywhere reads as not the whole domain
    if !(((0.5 * twice_area).abs() - domain).abs() <= 1e-9 * domain) {
        return Err("the loop does not enclose the whole domain once");
    }
    for side in &mut edges {
        side.sort_unstable();
        side.dedup();
    }
    Ok((edges, sides))
}

fn distance(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn point_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let length2 = dx * dx + dy * dy;
    let t = if length2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length2).clamp(0.0, 1.0) } else { 0.0 };
    distance(p, [a[0] + t * dx, a[1] + t * dy])
}

/// Distance between two closed segments of the plane: zero where they cross,
/// else the least of the four end-to-segment distances.
fn segment_segment(a: [[f64; 2]; 2], b: [[f64; 2]; 2]) -> f64 {
    let orient = |p: [f64; 2], q: [f64; 2], r: [f64; 2]| (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0]);
    let (d1, d2) = (orient(a[0], a[1], b[0]), orient(a[0], a[1], b[1]));
    let (d3, d4) = (orient(b[0], b[1], a[0]), orient(b[0], b[1], a[1]));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return 0.0;
    }
    point_segment(a[0], b[0], b[1])
        .min(point_segment(a[1], b[0], b[1]))
        .min(point_segment(b[0], a[0], a[1]))
        .min(point_segment(b[1], a[0], a[1]))
}

impl Piece {
    pub fn ends(&self) -> [[f64; 2]; 2] {
        match *self {
            Piece::Line(ends) | Piece::Arc { ends, .. } => ends,
        }
    }

    /// Whether the arc runs through the direction `angle` from its centre.
    fn holds_angle(&self, angle: f64) -> bool {
        match *self {
            Piece::Line(_) => false,
            Piece::Arc { start, sweep, .. } => {
                let along = ((angle - start) * sweep.signum()).rem_euclid(std::f64::consts::TAU);
                along <= sweep.abs()
            }
        }
    }

    fn length(&self) -> f64 {
        match *self {
            Piece::Line([a, b]) => distance(a, b),
            Piece::Arc { radius, sweep, .. } => radius * sweep.abs(),
        }
    }

    /// The least distance from `p` to the piece.
    fn distance_to(&self, p: [f64; 2]) -> f64 {
        match *self {
            Piece::Line([a, b]) => point_segment(p, a, b),
            Piece::Arc { centre, radius, ends, .. } => {
                let away = distance(p, centre);
                let to_ends = distance(p, ends[0]).min(distance(p, ends[1]));
                if away > 0.0 && self.holds_angle((p[1] - centre[1]).atan2(p[0] - centre[0])) {
                    (away - radius).abs().min(to_ends)
                } else {
                    to_ends
                }
            }
        }
    }

    /// The unit direction the piece leaves its end `k` in.
    fn leaving(&self, k: usize) -> [f64; 2] {
        let unit = |v: [f64; 2]| {
            let length = v[0].hypot(v[1]);
            [v[0] / length, v[1] / length]
        };
        match *self {
            Piece::Line(ends) => unit([ends[1 - k][0] - ends[k][0], ends[1 - k][1] - ends[k][1]]),
            Piece::Arc { centre, sweep, ends, .. } => {
                let radial = unit([ends[k][0] - centre[0], ends[k][1] - centre[1]]);
                let forward = [-radial[1] * sweep.signum(), radial[0] * sweep.signum()];
                if k == 0 { forward } else { [-forward[0], -forward[1]] }
            }
        }
    }

    /// The least ρ the piece reaches away from its ends.
    fn least_inner_rho(&self) -> f64 {
        match *self {
            Piece::Line(_) => f64::INFINITY,
            Piece::Arc { centre, radius, .. } => {
                if self.holds_angle(std::f64::consts::PI) { centre[0] - radius } else { f64::INFINITY }
            }
        }
    }
}

/// The points two pieces' carriers (line or circle) share, besides any that
/// are the same carrier; `None` where the carriers are one line or one circle.
fn carrier_meets(a: &Piece, b: &Piece) -> Option<Vec<[f64; 2]>> {
    let line_circle = |[p, q]: [[f64; 2]; 2], centre: [f64; 2], radius: f64| -> Vec<[f64; 2]> {
        let d = [q[0] - p[0], q[1] - p[1]];
        let length = d[0].hypot(d[1]);
        let u = [d[0] / length, d[1] / length];
        let along = (centre[0] - p[0]) * u[0] + (centre[1] - p[1]) * u[1];
        let foot = [p[0] + along * u[0], p[1] + along * u[1]];
        let off2 = radius * radius - distance(foot, centre).powi(2);
        if off2 < 0.0 {
            return Vec::new();
        }
        let off = off2.sqrt();
        vec![[foot[0] + off * u[0], foot[1] + off * u[1]], [foot[0] - off * u[0], foot[1] - off * u[1]]]
    };
    match (*a, *b) {
        (Piece::Line(l), Piece::Line(m)) => {
            let (d, e) = ([l[1][0] - l[0][0], l[1][1] - l[0][1]], [m[1][0] - m[0][0], m[1][1] - m[0][1]]);
            let det = d[0] * e[1] - d[1] * e[0];
            if det == 0.0 {
                return None;
            }
            let t = ((m[0][0] - l[0][0]) * e[1] - (m[0][1] - l[0][1]) * e[0]) / det;
            Some(vec![[l[0][0] + t * d[0], l[0][1] + t * d[1]]])
        }
        (Piece::Line(l), Piece::Arc { centre, radius, .. }) | (Piece::Arc { centre, radius, .. }, Piece::Line(l)) => Some(line_circle(l, centre, radius)),
        (Piece::Arc { centre: c1, radius: r1, .. }, Piece::Arc { centre: c2, radius: r2, .. }) => {
            let gap = distance(c1, c2);
            if gap == 0.0 {
                return if r1 == r2 { None } else { Some(Vec::new()) };
            }
            let along = (gap * gap + r1 * r1 - r2 * r2) / (2.0 * gap);
            let off2 = r1 * r1 - along * along;
            if off2 < 0.0 {
                return Some(Vec::new());
            }
            let off = off2.sqrt();
            let u = [(c2[0] - c1[0]) / gap, (c2[1] - c1[1]) / gap];
            let base = [c1[0] + along * u[0], c1[1] + along * u[1]];
            Some(vec![[base[0] - off * u[1], base[1] + off * u[0]], [base[0] + off * u[1], base[1] - off * u[0]]])
        }
    }
}

/// The least distance between two pieces, exactly for lines and circular arcs:
/// zero where their carriers meet inside both, else the least over each end
/// against the other piece and the carriers' own closest approach (a circle's
/// to a line along the foot of its centre, two circles' along their centres'
/// line) where that falls inside both.
fn piece_distance(a: &Piece, b: &Piece) -> f64 {
    if let (Piece::Line(l), Piece::Line(m)) = (*a, *b) {
        return segment_segment(l, m);
    }
    let mut least = a.ends().iter().map(|&p| b.distance_to(p)).chain(b.ends().iter().map(|&p| a.distance_to(p))).fold(f64::INFINITY, f64::min);
    match carrier_meets(a, b) {
        // a point both carriers hold: its distances to the two pieces sum to
        // a real distance between them, near zero where they cross
        Some(points) => {
            for p in points {
                least = least.min(a.distance_to(p) + b.distance_to(p));
            }
        }
        // one carrier: the nearest approach is an end against the other piece
        None => return least,
    }
    match (*a, *b) {
        (Piece::Line(l), Piece::Arc { centre, radius, .. }) | (Piece::Arc { centre, radius, .. }, Piece::Line(l)) => {
            let d = [l[1][0] - l[0][0], l[1][1] - l[0][1]];
            let t = ((centre[0] - l[0][0]) * d[0] + (centre[1] - l[0][1]) * d[1]) / (d[0] * d[0] + d[1] * d[1]);
            if (0.0..=1.0).contains(&t) {
                let foot = [l[0][0] + t * d[0], l[0][1] + t * d[1]];
                let away = distance(foot, centre);
                let arc = if let Piece::Arc { .. } = *a { a } else { b };
                if away >= radius && away > 0.0 && arc.holds_angle((foot[1] - centre[1]).atan2(foot[0] - centre[0])) {
                    least = least.min(away - radius);
                }
            }
        }
        (Piece::Arc { centre: c1, radius: r1, .. }, Piece::Arc { centre: c2, radius: r2, .. }) => {
            let gap = distance(c1, c2);
            if gap == 0.0 {
                // concentric: |r1 - r2| where their turns overlap
                let overlap = a.ends().iter().chain(b.ends().iter()).any(|&p| {
                    let angle = (p[1] - c1[1]).atan2(p[0] - c1[0]);
                    a.holds_angle(angle) && b.holds_angle(angle)
                });
                if overlap {
                    least = least.min((r1 - r2).abs());
                }
            } else {
                let u = [(c2[0] - c1[0]) / gap, (c2[1] - c1[1]) / gap];
                for s1 in [1.0, -1.0] {
                    let q1 = [c1[0] + s1 * r1 * u[0], c1[1] + s1 * r1 * u[1]];
                    if !a.holds_angle((q1[1] - c1[1]).atan2(q1[0] - c1[0])) {
                        continue;
                    }
                    for s2 in [1.0, -1.0] {
                        let q2 = [c2[0] + s2 * r2 * u[0], c2[1] + s2 * r2 * u[1]];
                        if b.holds_angle((q2[1] - c2[1]).atan2(q2[0] - c2[0])) {
                            least = least.min(distance(q1, q2));
                        }
                    }
                }
            }
        }
        _ => {}
    }
    least
}

/// The profile's own verdict: simple chains, matching the topology, touching
/// the axis only at poles. `scale` is the profile's extent.
pub fn chain_verdict(segments: &[Segment], scale: f64) -> Result<(), Decline> {
    let (coincident, separated) = (COINCIDENT * scale, SEPARATED * scale);
    let pole = |p: [f64; 2]| p[0] <= coincident;
    for segment in segments {
        let ends = segment.piece.ends();
        if !(segment.piece.length() >= separated) {
            return Err("a segment shorter than the separation");
        }
        if let Piece::Line(_) = segment.piece {
            if pole(ends[0]) && pole(ends[1]) {
                return Err("a segment on the axis");
            }
        }
        if !(segment.piece.least_inner_rho() >= separated) {
            return Err("an arc reaches the axis inside");
        }
        if let Piece::Arc { radius, .. } = segment.piece {
            if !(radius >= MIN_RADIUS * scale) {
                return Err("an arc tighter than the radius floor");
            }
        }
        for k in 0..2 {
            if pole(ends[k]) != segment.degenerate[k] {
                return Err("a pole end and a degenerate side disagree");
            }
        }
    }
    // each end's partner: (segment, end) of the one other end it meets
    let mut partner: Vec<[Option<(usize, usize)>; 2]> = vec![[None; 2]; segments.len()];
    for (i, a) in segments.iter().enumerate() {
        for k in 0..2 {
            let p = a.piece.ends()[k];
            let mut found = None;
            for (j, b) in segments.iter().enumerate() {
                for l in 0..2 {
                    if (i, k) == (j, l) {
                        continue;
                    }
                    let gap = distance(p, b.piece.ends()[l]);
                    if gap <= coincident {
                        if pole(p) || found.is_some() || i == j {
                            return Err("an end meets more than one other end, or a pole is shared");
                        }
                        found = Some((j, l));
                    } else if gap < separated {
                        return Err("two ends nearly meet");
                    }
                }
            }
            if !pole(p) {
                let Some((j, l)) = found else {
                    return Err("an off-axis end meets no other end");
                };
                if !a.edges[k].iter().any(|id| segments[j].edges[l].contains(id)) {
                    return Err("two ends meet where their faces share no edge");
                }
            }
            partner[i][k] = found;
        }
    }
    for i in 0..segments.len() {
        for j in i + 1..segments.len() {
            let (a, b) = (&segments[i].piece, &segments[j].piece);
            let shared: Vec<(usize, usize)> = (0..2)
                .filter_map(|k| partner[i][k].filter(|&(other, _)| other == j).map(|(_, l)| (k, l)))
                .collect();
            match shared.as_slice() {
                [] => {
                    if !(piece_distance(a, b) >= separated) {
                        return Err("two segments meet off their shared ends");
                    }
                }
                [(k, l)] => {
                    let at = a.ends()[*k];
                    let (d1, d2) = (a.leaving(*k), b.leaving(*l));
                    let cross = d1[0] * d2[1] - d1[1] * d2[0];
                    let dot = d1[0] * d2[0] + d1[1] * d2[1];
                    if !(cross.abs() >= MIN_SINE) && !(dot < 0.0) {
                        return Err("two segments double back over each other");
                    }
                    // each far end stays clear of the other piece
                    if !(b.distance_to(a.ends()[1 - k]) >= separated && a.distance_to(b.ends()[1 - l]) >= separated) {
                        return Err("a segment's far end reaches its neighbour");
                    }
                    match carrier_meets(a, b) {
                        // one circle continued: together less than a whole turn
                        None => {
                            let turns = |piece: &Piece| if let Piece::Arc { sweep, .. } = *piece { sweep.abs() } else { 0.0 };
                            if !(dot < 0.0) || !(turns(a) + turns(b) < std::f64::consts::TAU - 1e-6) {
                                return Err("one circle runs over itself");
                            }
                        }
                        // the carriers' other meeting must miss one piece
                        Some(points) => {
                            for q in points {
                                if distance(q, at) > separated && a.distance_to(q) < separated && b.distance_to(q) < separated {
                                    return Err("two segments meet again away from their shared end");
                                }
                            }
                        }
                    }
                }
                _ => return Err("two segments share both ends"),
            }
        }
    }
    Ok(())
}
