//! Faces deleted from a TURNED part: a solid of revolution about one axis
//! (every face a plane square to it or a surface of revolution about it,
//! whole round the axis). Its meridian, the closed chain of lines and arcs in
//! the half-plane (radius, height) that the part is the revolution of, is
//! read off its faces. Each deleted face is one piece of it. The pieces left
//! on either side of a run of deleted pieces are extended along their own
//! line or circle until they meet (the axis counts as a piece where the part
//! reaches it), and the healed chain is revolved again. A rim round or bevel
//! comes back as the sharp corner of its two walls, a countersink as the
//! bore's edge, a boss as the flat it stood on, a groove in a bore as the bore.
//!
//! Each kept face keeps its id and name: an extended piece is its face grown,
//! and two pieces of one line joined across a deleted run are the first one's
//! face. A part this lane does not read (not turned, or a meridian it cannot
//! read or heal) is left to the other lanes (`None`).

use crate::analytic_surface::AnalyticSurface;
use crate::{BrepSolid, FaceRecord, KernelRefusal, KernelStage, Vec3};

/// A point of the meridian half-plane: (radius from the axis, height along it).
type P2 = [f64; 2];

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn length(a: P2) -> f64 {
    dot(a, a).sqrt()
}

/// One piece of the meridian, run from `start` to `end`.
#[derive(Clone, Debug)]
struct Piece {
    start: P2,
    end: P2,
    /// `None` for a straight piece; for an arc its circle's centre and radius
    /// and whether it turns counter-clockwise in (radius, height).
    arc: Option<(P2, f64, bool)>,
    /// The face it is the meridian of; `None` for the axis.
    face: Option<u64>,
}

impl Piece {
    fn reversed(&self) -> Piece {
        Piece { start: self.end, end: self.start, arc: self.arc.map(|(c, r, ccw)| (c, r, !ccw)), face: self.face }
    }

    /// The angle a counter-clockwise arc turns from `a` to `b` about `c`, in [0, 2π).
    fn ccw_turn(c: P2, a: P2, b: P2) -> f64 {
        let (u, v) = (sub(a, c), sub(b, c));
        let mut t = cross(u, v).atan2(dot(u, v));
        if t < 0.0 {
            t += std::f64::consts::TAU;
        }
        t
    }

    /// The arc's turn from start to end (in its own sense), or None if straight.
    fn turn(&self) -> Option<f64> {
        self.arc.map(|(c, _, ccw)| if ccw { Self::ccw_turn(c, self.start, self.end) } else { Self::ccw_turn(c, self.end, self.start) })
    }

    /// How far the piece's end moves to reach `x` along its own line or circle
    /// (in length), or None if `x` is off it or behind its start. A straight
    /// piece may also be shortened (the axis cut back to a flat that now
    /// reaches it), never past its start.
    fn ahead(&self, x: P2, tol: f64) -> Option<f64> {
        match self.arc {
            None => {
                let d = sub(self.end, self.start);
                let len = length(d);
                let off = cross(d, sub(x, self.end)).abs() / len;
                let t = dot(d, sub(x, self.end)) / len;
                (off <= tol && t > -len + tol).then_some(t.abs())
            }
            Some((c, r, ccw)) => {
                if (length(sub(x, c)) - r).abs() > tol {
                    return None;
                }
                let turn = if ccw { Self::ccw_turn(c, self.end, x) } else { Self::ccw_turn(c, x, self.end) };
                let turn = if turn > std::f64::consts::TAU - tol / r { 0.0 } else { turn };
                // Not back over the piece itself.
                (turn + self.turn().unwrap_or(0.0) < std::f64::consts::TAU - tol / r).then_some(turn * r)
            }
        }
    }

    /// The points where this piece's line or circle meets `other`'s.
    fn meets(&self, other: &Piece) -> Vec<P2> {
        match (self.arc, other.arc) {
            (None, None) => {
                let (d1, d2) = (sub(self.end, self.start), sub(other.end, other.start));
                let den = cross(d1, d2);
                if den.abs() <= 1e-14 * length(d1) * length(d2) {
                    return Vec::new();
                }
                let s = cross(sub(other.start, self.start), d2) / den;
                vec![[self.start[0] + s * d1[0], self.start[1] + s * d1[1]]]
            }
            (None, Some((c, r, _))) | (Some((c, r, _)), None) => {
                let line = if self.arc.is_none() { self } else { other };
                let d = sub(line.end, line.start);
                let len = length(d);
                let u = [d[0] / len, d[1] / len];
                let w = sub(line.start, c);
                let b = dot(w, u);
                let disc = b * b - (dot(w, w) - r * r);
                if disc < 0.0 {
                    return Vec::new();
                }
                let s = disc.sqrt();
                [-b - s, -b + s].iter().map(|t| [line.start[0] + t * u[0], line.start[1] + t * u[1]]).collect()
            }
            (Some((c1, r1, _)), Some((c2, r2, _))) => {
                let d = sub(c2, c1);
                let dist = length(d);
                if dist <= 0.0 || dist > r1 + r2 || dist < (r1 - r2).abs() {
                    return Vec::new();
                }
                let a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0 * dist);
                let h = (r1 * r1 - a * a).max(0.0).sqrt();
                let m = [c1[0] + a * d[0] / dist, c1[1] + a * d[1] / dist];
                vec![[m[0] - h * d[1] / dist, m[1] + h * d[0] / dist], [m[0] + h * d[1] / dist, m[1] - h * d[0] / dist]]
            }
        }
    }

    /// Whether `other` continues this piece along its own line: both
    /// straight, on one line, running the same way, `other` starting at or
    /// past this one's end (a join across the gap between them).
    fn same_line(&self, other: &Piece, tol: f64) -> bool {
        if self.arc.is_some() || other.arc.is_some() {
            return false;
        }
        let d = sub(self.end, self.start);
        let len = length(d);
        [other.start, other.end].iter().all(|p| cross(d, sub(*p, self.start)).abs() / len <= tol)
            && dot(d, sub(other.end, other.start)) > 0.0
            && dot(d, sub(other.start, self.end)) / len >= -tol
    }

    /// Whether this piece and `other` lie on one line (both straight).
    fn collinear(&self, other: &Piece, tol: f64) -> bool {
        if self.arc.is_some() || other.arc.is_some() {
            return false;
        }
        let d = sub(self.end, self.start);
        let len = length(d);
        [other.start, other.end].iter().all(|p| cross(d, sub(*p, self.start)).abs() / len <= tol)
    }
}

/// The turned part's axis and its meridian frame.
struct Axis {
    origin: Vec3,
    axis: Vec3,
    radial: Vec3,
}

impl Axis {
    fn to2(&self, p: Vec3) -> P2 {
        let d = p.sub(self.origin);
        let h = d.dot(self.axis);
        [d.sub(self.axis.scale(h)).length(), h]
    }

    fn world(&self, p: P2) -> Vec3 {
        self.origin.add(self.radial.scale(p[0])).add(self.axis.scale(p[1]))
    }
}

/// The common axis of the part's faces of revolution, or None.
fn read_axis(solid: &BrepSolid, tol: f64) -> Option<Axis> {
    let mut found: Option<Axis> = None;
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let frame = match face.surface.analytic()? {
            AnalyticSurface::Plane { .. } => continue,
            AnalyticSurface::RuledRevolution { frame, .. } | AnalyticSurface::Sphere { frame, .. } | AnalyticSurface::Torus { frame, .. } => frame,
            AnalyticSurface::Revolution { frame, sweep, .. } => {
                if (sweep - std::f64::consts::TAU).abs() > 1e-9 {
                    return None;
                }
                frame
            }
        };
        match &found {
            None => found = Some(Axis { origin: frame.origin, axis: frame.axis, radial: frame.x_axis }),
            Some(axis) => {
                let off = frame.origin.sub(axis.origin);
                if frame.axis.cross(axis.axis).length() > 1e-9 || off.sub(axis.axis.scale(off.dot(axis.axis))).length() > tol {
                    return None;
                }
            }
        }
    }
    found
}

/// The face's meridian piece, its ends read off its boundary circles (an end
/// on the axis where the face closes onto it), its curve off its carrier
/// sampled along the meridian. None where the face is not one piece of a
/// turned part.
fn read_piece(face: &FaceRecord, solid: &BrepSolid, axis: &Axis, tol: f64) -> Option<Piece> {
    // The ends: every boundary edge is a circle about the axis (one height and
    // radius at all its points), a point (degenerate), or the face's own seam.
    let mut ends: Vec<P2> = Vec::new();
    let mut push = |p: P2, ends: &mut Vec<P2>| {
        if !ends.iter().any(|q| length(sub(*q, p)) <= tol) {
            ends.push(p);
        }
    };
    for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
        let edge = solid.edges.iter().find(|e| e.id == coedge.edge_id)?;
        if let Some(at) = rim_of(&edge.curve, axis, tol) {
            push(at, &mut ends);
            continue;
        }
        // A seam runs along the meridian: its ends are the face's ends too.
        let used = face.loops.iter().flat_map(|lp| &lp.coedges).filter(|c| c.edge_id == edge.id).count();
        if used == 2 {
            continue;
        }
        return None;
    }
    let plane = matches!(face.surface.analytic()?, AnalyticSurface::Plane { .. });
    if plane {
        // Square to the axis: every end at one height; a disc closes on the axis.
        let h = ends.first()?[1];
        if ends.iter().any(|p| (p[1] - h).abs() > tol) {
            return None;
        }
        if ends.len() == 1 {
            push([0.0, h], &mut ends);
        }
        if ends.len() != 2 {
            return None;
        }
        return Some(Piece { start: ends[0], end: ends[1], arc: None, face: Some(face.id) });
    }
    if ends.len() == 1 {
        return None;
    }
    if ends.len() != 2 {
        return None;
    }
    // The curve, read exactly from the carrier (never sampled: a bulge between
    // samples would be read straight).
    let analytic = face.surface.analytic()?;
    let (line, circle): (bool, Option<(P2, f64)>) = match analytic {
        AnalyticSurface::RuledRevolution { .. } => (true, None),
        AnalyticSurface::Sphere { frame, radius } => (false, Some(([0.0, axis.to2(frame.origin)[1]], *radius))),
        AnalyticSurface::Torus { frame, major_radius, minor_radius } => (false, Some(([*major_radius, axis.to2(frame.origin)[1]], *minor_radius))),
        AnalyticSurface::Revolution { generatrix, frame, .. } => match read_generatrix(generatrix, axis, frame.x_axis, ends[0], ends[1], tol)? {
            Generatrix::Line => (true, None),
            Generatrix::Arc { centre, radius, ccw, from } => {
                return Some(Piece { start: from, end: if length(sub(from, ends[0])) <= tol { ends[1] } else { ends[0] }, arc: Some((centre, radius, ccw)), face: Some(face.id) });
            }
        },
        AnalyticSurface::Plane { .. } => return None,
    };
    if line {
        return Some(Piece { start: ends[0], end: ends[1], arc: None, face: Some(face.id) });
    }
    let (c, r) = circle?;
    if ends.iter().any(|p| (length(sub(*p, c)) - r).abs() > tol) {
        return None;
    }
    // A sphere's or a torus's arc: its sense is the signed turn its meridian
    // makes, summed in steps (each far under a half turn) along the parameter
    // that moves in the meridian.
    let ([u0, u1], [v0, v1]) = (face.surface.domain_u().ok()?, face.surface.domain_v().ok()?);
    let along = |fixed_u: bool| -> Option<Vec<P2>> {
        (0..=64).map(|k| {
            let s = k as f64 / 64.0;
            let (u, v) = if fixed_u { (0.5 * (u0 + u1), v0 + (v1 - v0) * s) } else { (u0 + (u1 - u0) * s, 0.5 * (v0 + v1)) };
            face.surface.evaluate(u, v).ok().map(|p| axis.to2(p))
        }).collect()
    };
    let (a, b) = (along(true)?, along(false)?);
    let spread = |s: &[P2]| s.iter().map(|p| length(sub(*p, s[0]))).fold(0.0f64, f64::max);
    let samples = if spread(&a) >= spread(&b) { a } else { b };
    let signed: f64 = samples.windows(2).map(|w| cross(sub(w[0], c), sub(w[1], c)).atan2(dot(sub(w[0], c), sub(w[1], c)))).sum();
    let (first, last) = (samples[0], samples[64]);
    let (start, end) = if length(sub(first, ends[0])) <= tol && length(sub(last, ends[1])) <= tol {
        (ends[0], ends[1])
    } else if length(sub(first, ends[1])) <= tol && length(sub(last, ends[0])) <= tol {
        (ends[1], ends[0])
    } else {
        return None;
    };
    Some(Piece { start, end, arc: Some((c, r, signed > 0.0)), face: Some(face.id) })
}

/// The (radius, height) of a curve that is exactly a circle about the axis, or
/// a point, read off its control points: all at one height square to the
/// axis, and either all one point or rational quadratic spans each an arc
/// about the axis (its ends at one radius, its middle control point where
/// their tangents meet, its weight the cosine of half its angle). The curve
/// lies in its control points' hull, so this holds for every point of it.
fn rim_of(curve: &crate::NurbsCurve, axis: &Axis, tol: f64) -> Option<P2> {
    let across = axis.axis.cross(axis.radial);
    let mut points: Vec<([f64; 2], f64, f64)> = Vec::with_capacity(curve.control_points.len());
    for control in &curve.control_points {
        let d = control.point().ok()?.sub(axis.origin);
        points.push(([d.dot(axis.radial), d.dot(across)], d.dot(axis.axis), control.w));
    }
    let h = points.first()?.1;
    if points.iter().any(|(_, z, w)| (z - h).abs() > tol || !(*w > 0.0)) {
        return None;
    }
    let p0 = points[0].0;
    if points.iter().all(|(q, _, _)| length(sub(*q, p0)) <= tol) {
        return Some([length(p0), h]);
    }
    if curve.degree != 2 || points.len() < 3 || points.len() % 2 == 0 {
        return None;
    }
    let interior: Vec<f64> = curve.knots.iter().copied().filter(|t| *t > curve.knots[0] && *t < *curve.knots.last().unwrap()).collect();
    if interior.len() != 2 * ((points.len() - 1) / 2 - 1) || interior.chunks(2).any(|pair| pair[0] != pair[1]) {
        return None;
    }
    let radius = length(p0);
    for k in 0..(points.len() - 1) / 2 {
        let ((u, w0), (q1, w1), (v, w2)) = ((points[2 * k].0, points[2 * k].2), (points[2 * k + 1].0, points[2 * k + 1].2), (points[2 * k + 2].0, points[2 * k + 2].2));
        let half = (0.5 * cross(u, v).atan2(dot(u, v))).cos();
        let tip = [(u[0] + v[0]) / (2.0 * half * half), (u[1] + v[1]) / (2.0 * half * half)];
        if !(half > 0.0) || (length(u) - radius).abs() > tol || (length(v) - radius).abs() > tol || length(sub(q1, tip)) > tol || (w1 / (w0 * w2).sqrt() - half).abs() > 1e-9 {
            return None;
        }
    }
    Some([radius, h])
}

/// What a general revolution's generatrix is, read off its control points.
enum Generatrix {
    Line,
    /// An arc from `from` (one of the face's ends) about `centre`.
    Arc { centre: P2, radius: f64, ccw: bool, from: P2 },
}

/// A generatrix that is exactly a line (every control point on the line
/// through its ends: the curve lies in their hull) or exactly a circular arc
/// (rational quadratic spans, each with its ends on one circle, its middle
/// control point where their tangents meet and its weight the cosine of half
/// its angle), running between the face's two ends. Anything else: None.
///
/// The control points are read in the generatrix's own meridian plane (the
/// carrier's `radial` direction, radius signed): every one must lie in it and
/// none past the axis, or a point folded over the axis would read as on the
/// near side.
fn read_generatrix(curve: &crate::NurbsCurve, axis: &Axis, radial: Vec3, a: P2, b: P2, tol: f64) -> Option<Generatrix> {
    let across = axis.axis.cross(radial);
    let mut points: Vec<(P2, f64)> = Vec::with_capacity(curve.control_points.len());
    for control in &curve.control_points {
        let d = control.point().ok()?.sub(axis.origin);
        let (r, h) = (d.dot(radial), d.dot(axis.axis));
        if d.dot(across).abs() > tol || r < -tol {
            return None;
        }
        points.push(([r, h], control.w));
    }
    let (first, last) = (points.first()?.0, points.last()?.0);
    let runs_between = (length(sub(first, a)) <= tol && length(sub(last, b)) <= tol) || (length(sub(first, b)) <= tol && length(sub(last, a)) <= tol);
    if !runs_between || points.iter().any(|(_, w)| !(*w > 0.0)) {
        return None;
    }
    let chord = sub(last, first);
    if length(chord) > tol && points.iter().all(|(p, _)| cross(chord, sub(*p, first)).abs() / length(chord) <= tol) {
        return Some(Generatrix::Line);
    }
    // Rational quadratic spans: knots of multiplicity two inside, control
    // points P0 P1 P2 | P2 P3 P4 | ...
    if curve.degree != 2 || points.len() < 3 || points.len() % 2 == 0 {
        return None;
    }
    let spans: Vec<[(P2, f64); 3]> = (0..(points.len() - 1) / 2).map(|k| [points[2 * k], points[2 * k + 1], points[2 * k + 2]]).collect();
    let interior: Vec<f64> = curve.knots.iter().copied().filter(|t| *t > curve.knots[0] && *t < *curve.knots.last().unwrap()).collect();
    if interior.len() != 2 * (spans.len() - 1) || interior.chunks(2).any(|pair| pair[0] != pair[1]) {
        return None;
    }
    // The circle: through the first span's ends, its centre where the normals
    // at them meet (the middle control point gives both tangents).
    let [(p0, _), (p1, _), (p2, _)] = spans[0];
    let (t0, t2) = (sub(p1, p0), sub(p2, p1));
    let (n0, n2) = ([-t0[1], t0[0]], [-t2[1], t2[0]]);
    let den = cross(n0, n2);
    if den.abs() <= 1e-300 {
        return None;
    }
    let s = cross(sub(p2, p0), n2) / den;
    let centre = [p0[0] + s * n0[0], p0[1] + s * n0[1]];
    let radius = length(sub(p0, centre));
    let mut signed = 0.0;
    for [(q0, w0), (q1, w1), (q2, w2)] in spans {
        let (u, v) = (sub(q0, centre), sub(q2, centre));
        let angle = cross(u, v).atan2(dot(u, v));
        // Ends on the circle, the middle where the tangents meet, the weight
        // (relative to the ends') the cosine of half the angle.
        let half = (0.5 * angle).cos();
        let tip = [centre[0] + (u[0] + v[0]) / (2.0 * half * half), centre[1] + (u[1] + v[1]) / (2.0 * half * half)];
        let weight = w1 / (w0 * w2).sqrt();
        if (length(u) - radius).abs() > tol || (length(v) - radius).abs() > tol || length(sub(q1, tip)) > tol || (weight - half).abs() > 1e-9 || !(half > 0.0) {
            return None;
        }
        signed += angle;
    }
    Some(Generatrix::Arc { centre, radius, ccw: signed > 0.0, from: first })
}

/// The closed meridian, its pieces in order, each run from the last one's end.
fn chain(mut pieces: Vec<Piece>, tol: f64) -> Option<Vec<Piece>> {
    // The axis: the stretch between the two ends on it, where the part reaches it.
    let on_axis: Vec<P2> = {
        let mut points: Vec<P2> = Vec::new();
        for p in pieces.iter().flat_map(|piece| [piece.start, piece.end]) {
            if p[0].abs() <= tol && !points.iter().any(|q| length(sub(*q, p)) <= tol) {
                points.push(p);
            }
        }
        points
    };
    match on_axis.len() {
        0 => {}
        2 => pieces.push(Piece { start: on_axis[0], end: on_axis[1], arc: None, face: None }),
        _ => return None,
    }
    let mut ordered = vec![pieces.remove(0)];
    while !pieces.is_empty() {
        let at = ordered.last()?.end;
        let next = pieces.iter().position(|p| length(sub(p.start, at)) <= tol || length(sub(p.end, at)) <= tol)?;
        let piece = pieces.remove(next);
        ordered.push(if length(sub(piece.start, at)) <= tol { piece } else { piece.reversed() });
    }
    (length(sub(ordered.last()?.end, ordered[0].start)) <= tol).then_some(ordered)
}

/// `solid` with the faces `face_ids` deleted and healed by its meridian, or
/// None where the part is not one this lane reads (see the module docs).
pub(super) fn heal_turned(solid: &BrepSolid, face_ids: &[u64]) -> Option<Result<BrepSolid, KernelRefusal>> {
    let faces: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    // A plain solid only: a cellular body's declared regions the rebuild would not carry.
    if !matches!(solid.kind, crate::BodyKind::Solid) || !solid.cells.is_empty() || face_ids.is_empty() || solid.shells.len() != 1 || !face_ids.iter().all(|id| faces.iter().any(|f| f.id == *id)) {
        return None;
    }
    let scale = crate::solid_model_scale(solid).max(1.0);
    let tol = 1e-7 * scale;
    let axis = read_axis(solid, tol)?;
    // Planes square to the axis.
    for face in &faces {
        if let Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) = face.surface.analytic() {
            if u_dir.cross(*v_dir).normalized().ok()?.cross(axis.axis).length() > 1e-9 {
                return None;
            }
        }
    }
    let pieces: Vec<Piece> = faces.iter().map(|face| read_piece(face, solid, &axis, tol)).collect::<Option<_>>()?;
    let mut loop_ = chain(pieces, tol)?;
    let deleted = |p: &Piece| p.face.is_some_and(|id| face_ids.contains(&id));
    if loop_.iter().all(|p| deleted(p)) || !loop_.iter().any(|p| deleted(p)) {
        return None;
    }
    // Start the loop on a kept piece, then heal each run of deleted pieces.
    let first = loop_.iter().position(|p| !deleted(p))?;
    loop_.rotate_left(first);
    let mut healed: Vec<Piece> = Vec::new();
    let mut i = 0;
    let n = loop_.len();
    while i < n {
        let piece = loop_[i].clone();
        if !deleted(&piece) {
            healed.push(piece);
            i += 1;
            continue;
        }
        let mut j = i;
        while j < n && deleted(&loop_[j]) {
            j += 1;
        }
        // A = the kept piece before the run, B = the one after (wrapping to the first).
        let before = healed.len() - 1;
        let a = healed[before].clone();
        let b = if j < n { loop_[j].clone() } else { healed[0].clone() };
        if a.same_line(&b, tol) {
            // One line across the run, one face: the lower of the two ids (and
            // its name) keeps it, whichever way round the chain was read.
            let face = match (a.face, b.face) {
                (Some(x), Some(y)) => Some(x.min(y)),
                (x, y) => x.or(y),
            };
            if j < n {
                healed[before].end = b.end;
                healed[before].face = face;
                i = j + 1;
            } else {
                healed[0].start = a.start;
                healed[0].face = face;
                healed.pop();
                i = j;
            }
            continue;
        }
        // On one line but not continuing it (folded back on itself): no join.
        if a.collinear(&b, tol) {
            return None;
        }
        let meet = a
            .meets(&b)
            .into_iter()
            .filter(|x| x[0] >= -tol)
            .filter_map(|x| Some((a.ahead(x, tol)? + b.reversed().ahead(x, tol)?, x)))
            .min_by(|p, q| p.0.total_cmp(&q.0));
        let Some((_, x)) = meet else {
            return Some(Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "turned_heal_no_meeting",
                "delete_faces_and_heal: the turned part's neighbours of a deleted face do not meet in its meridian".to_string(),
            )));
        };
        healed[before].end = x;
        if j < n {
            let mut b = b;
            b.start = x;
            healed.push(b);
            i = j + 1;
        } else {
            healed[0].start = x;
            i = j;
        }
    }
    Some(rebuild(solid, &axis, &healed, tol))
}

/// The healed meridian revolved, each face given its source face's id and name.
fn rebuild(solid: &BrepSolid, axis: &Axis, meridian: &[Piece], tol: f64) -> Result<BrepSolid, KernelRefusal> {
    let refuse = |message: String| KernelRefusal::internal(KernelStage::Refine, "turned_heal_rebuild", format!("delete_faces_and_heal: {message}"));
    // Pieces shrunk to nothing go; the rest must not cross each other.
    let meridian: Vec<&Piece> = meridian.iter().filter(|p| length(sub(p.end, p.start)) > tol || p.arc.is_some()).collect();
    let polyline = |p: &Piece| -> Vec<P2> {
        match (p.arc, p.turn()) {
            (Some((c, r, ccw)), Some(turn)) => {
                let a0 = (p.start[1] - c[1]).atan2(p.start[0] - c[0]);
                (0..=32).map(|k| {
                    let a = a0 + if ccw { 1.0 } else { -1.0 } * turn * k as f64 / 32.0;
                    [c[0] + r * a.cos(), c[1] + r * a.sin()]
                }).collect()
            }
            _ => vec![p.start, p.end],
        }
    };
    let lines: Vec<Vec<P2>> = meridian.iter().map(|p| polyline(p)).collect();
    let crosses = |a: (P2, P2), b: (P2, P2)| {
        let (d1, d2) = (sub(a.1, a.0), sub(b.1, b.0));
        let den = cross(d1, d2);
        if den.abs() <= 1e-300 {
            return false;
        }
        let s = cross(sub(b.0, a.0), d2) / den;
        let t = cross(sub(b.0, a.0), d1) / den;
        s > 1e-9 && s < 1.0 - 1e-9 && t > 1e-9 && t < 1.0 - 1e-9
    };
    for i in 0..lines.len() {
        for j in i + 1..lines.len() {
            for a in lines[i].windows(2) {
                for b in lines[j].windows(2) {
                    if crosses((a[0], a[1]), (b[0], b[1])) {
                        return Err(KernelRefusal::unsupported(KernelStage::Refine, "turned_heal_crossing", "delete_faces_and_heal: the turned part's healed meridian crosses itself".to_string()));
                    }
                }
            }
        }
    }
    if meridian.iter().any(|p| p.start[0] < -tol || p.end[0] < -tol) {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "turned_heal_past_axis", "delete_faces_and_heal: the turned part's healed meridian crosses the axis".to_string()));
    }
    let up = axis.axis;
    let mut curves = Vec::new();
    let mut tags = Vec::new();
    for (k, piece) in meridian.iter().enumerate() {
        let curve = match piece.arc {
            None => crate::make_line(axis.world([piece.start[0].max(0.0), piece.start[1]]), axis.world([piece.end[0].max(0.0), piece.end[1]])),
            Some((c, r, ccw)) => {
                let a0 = (piece.start[1] - c[1]).atan2(piece.start[0] - c[0]);
                let turn = piece.turn().unwrap_or(0.0);
                let centre = axis.world(c);
                if ccw {
                    crate::make_arc(centre, axis.radial, up, r, a0, a0 + turn)
                } else {
                    crate::make_arc(centre, axis.radial, up, r, a0 - turn, a0).and_then(|arc| arc.reversed())
                }
            }
        }
        .map_err(|e| refuse(e))?;
        curves.push(curve);
        tags.push(Some(format!("__turned_heal_{k}")));
    }
    let mut body = crate::revolve_profile_brep_named(&curves, axis.origin, up, std::f64::consts::TAU, &tags, &[]).map_err(|e| refuse(e))?;
    // Each face back to its source face: its id and name.
    let source = |id: u64| solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == id);
    // Every kept piece must come back as a face (the revolve builder reads a
    // piece within its own 1e-3 of the axis as the axis: a bore that fine would
    // be filled, not kept).
    let tagged: std::collections::HashSet<usize> = body
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .filter_map(|face| face.name.as_deref().and_then(|name| name.strip_prefix("__turned_heal_")).and_then(|k| k.parse::<usize>().ok()))
        .collect();
    if let Some(lost) = meridian.iter().enumerate().find(|(k, piece)| piece.face.is_some() && !tagged.contains(k)) {
        return Err(refuse(format!("the revolved meridian lost the piece of face {:?}", lost.1.face)));
    }
    let mut used = std::collections::HashSet::new();
    let mut next_id = solid.shells.iter().flat_map(|shell| &shell.faces).map(|face| face.id).max().unwrap_or(0);
    for face in body.shells.iter_mut().flat_map(|shell| &mut shell.faces) {
        let k = face.name.as_deref().and_then(|name| name.strip_prefix("__turned_heal_")).and_then(|k| k.parse::<usize>().ok());
        let origin = k.and_then(|k| meridian.get(k)).and_then(|piece| piece.face).and_then(source);
        match origin {
            Some(original) if used.insert(original.id) => {
                face.id = original.id;
                face.name = original.name.clone();
            }
            _ => {
                next_id = next_id.checked_add(1).ok_or_else(|| refuse("no face id is left for a new face".into()))?;
                face.id = next_id;
                face.name = None;
            }
        }
    }
    // Each kept rim keeps its edge's name: a circle of the rebuilt body at a
    // source circle's (radius, height).
    let circle_of = |edge: &crate::topology::EdgeRecord| rim_of(&edge.curve, axis, tol);
    // A rim stored as several named arcs comes back as one circle: it takes a
    // name only when every source edge on that circle carries the same one.
    let rims: Vec<(P2, Option<String>)> = solid.edges.iter().filter_map(|edge| Some((circle_of(edge)?, edge.name.clone()))).collect();
    for edge in body.edges.iter_mut() {
        if let Some(at) = circle_of(edge) {
            let names: std::collections::BTreeSet<Option<&String>> = rims.iter().filter(|(p, _)| length(sub(*p, at)) <= tol).map(|(_, name)| name.as_ref()).collect();
            if let (1, Some(Some(name))) = (names.len(), names.iter().next()) {
                edge.name = Some((*name).clone());
            }
        }
    }
    // Each kept face's seam (an edge its own loops use twice) keeps the name
    // of its source face's seam, where that face has exactly one named seam.
    let seams = |body: &BrepSolid, face: &FaceRecord| -> Vec<u64> {
        let uses: Vec<u64> = face.loops.iter().flat_map(|lp| &lp.coedges).map(|c| c.edge_id).collect();
        let mut seams: Vec<u64> = uses.iter().copied().filter(|id| uses.iter().filter(|u| *u == id).count() == 2).collect();
        seams.sort();
        seams.dedup();
        seams.retain(|id| body.edges.iter().any(|e| e.id == *id && !e.degenerate));
        seams
    };
    let mut renames: Vec<(u64, String)> = Vec::new();
    for face in body.shells.iter().flat_map(|shell| &shell.faces) {
        let Some(original) = source(face.id) else { continue };
        let named: Vec<String> = seams(solid, original).iter().filter_map(|id| solid.edges.iter().find(|e| e.id == *id).and_then(|e| e.name.clone())).collect();
        let rebuilt = seams(&body, face);
        if let ([name], [edge]) = (named.as_slice(), rebuilt.as_slice()) {
            renames.push((*edge, name.clone()));
        }
    }
    for (id, name) in renames {
        if let Some(edge) = body.edges.iter_mut().find(|e| e.id == id) {
            edge.name = Some(name);
        }
    }
    body.id = solid.id;
    let issues = body.validate();
    if !issues.is_empty() {
        return Err(refuse(format!("the revolved healed meridian is not valid: {:?}", issues.first())));
    }
    Ok(body)
}

