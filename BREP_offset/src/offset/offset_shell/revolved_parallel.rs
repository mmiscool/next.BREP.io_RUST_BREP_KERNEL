//! `BREP_OFFSET_REVOLVED_PARALLEL` (default on; `=0` restores the refusal): the
//! rounded GROWTH of a solid of revolution, built from its half-section.
//!
//! A solid of revolution's distance to a point is read in that point's
//! meridian plane, so its parallel body `K ⊕ B_d` is the revolution of its full
//! meridian section grown by the disk of radius `d`. Where every edge of the
//! section is convex, that grown section is bounded by each boundary piece
//! moved out by `d` (a line to the parallel line, an arc to the concentric arc
//! of radius `R ± d`) and, at each corner, the arc of radius `d` about the
//! corner from the one piece's normal to the next's: the rolling ball's torus
//! about a circular rim. The lane reads the half-section off the body (every
//! edge a circle about one axis, every face a plane square to it, a cylinder or
//! cone about it, a sphere centred on it, or a torus about it), offsets it, and
//! revolves it.
//!
//! Cause: a hemisphere, a ball cut by a plane
//! and the lens two balls share, each grown by 1, refused ("completion produced
//! non-integral genus", "unwelded rim leaves a non-watertight shell"). The
//! sharp body the pipeline rounds needs each face's carrier to run past its
//! own rim to meet its neighbour's offset; a plane's net slides out under its
//! trims, but a sphere's trim stays on the source rim, so the moved plane and
//! the moved sphere never meet and their imprint is empty.
//!
//! It runs only where the pipeline's own growth refused, and declines (the
//! pipeline's refusal stands) on anything else: a concave corner, a face of
//! another kind or about another axis, an edge that is not a circle about the
//! axis, a piece whose offset would pass through the axis, or a revolved body
//! that does not validate.
use super::*;
use crate::{make_arc, make_line, parameter_point_in_face, project_point_to_surface, revolve_profile_brep};
use crate::{AnalyticSurface, KernelRefusal, KernelStage, NurbsCurve, OrRefuse, PolygonClass, Vec2};

/// The switch, default on (`=0` restores the pipeline's refusal).
fn revolved_parallel_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_PARALLEL", true)
}


/// A point of the meridian half-plane: distance from the axis, height along it.
#[derive(Clone, Copy, Debug)]
struct P2 {
    rho: f64,
    z: f64,
}

impl P2 {
    fn sub(self, other: P2) -> P2 {
        P2 { rho: self.rho - other.rho, z: self.z - other.z }
    }
    fn add(self, other: P2) -> P2 {
        P2 { rho: self.rho + other.rho, z: self.z + other.z }
    }
    fn scale(self, k: f64) -> P2 {
        P2 { rho: self.rho * k, z: self.z * k }
    }
    fn length(self) -> f64 {
        self.rho.hypot(self.z)
    }
    fn cross(self, other: P2) -> f64 {
        self.rho * other.z - self.z * other.rho
    }
}

/// One piece of the half-section's boundary, from `start` to `end`.
#[derive(Clone, Copy, Debug)]
enum Piece {
    Line { start: P2, end: P2 },
    /// About `center`, anticlockwise (in (ρ, z)) when `ccw`.
    Arc { center: P2, radius: f64, start: P2, end: P2, ccw: bool },
}

impl Piece {
    fn start(&self) -> P2 {
        match *self {
            Piece::Line { start, .. } | Piece::Arc { start, .. } => start,
        }
    }
    fn end(&self) -> P2 {
        match *self {
            Piece::Line { end, .. } | Piece::Arc { end, .. } => end,
        }
    }
    fn reversed(&self) -> Piece {
        match *self {
            Piece::Line { start, end } => Piece::Line { start: end, end: start },
            Piece::Arc { center, radius, start, end, ccw } => Piece::Arc { center, radius, start: end, end: start, ccw: !ccw },
        }
    }
    /// Signed sweep (anticlockwise positive) of an arc from start to end.
    fn sweep(center: P2, start: P2, end: P2, ccw: bool) -> f64 {
        let a0 = (start.z - center.z).atan2(start.rho - center.rho);
        let a1 = (end.z - center.z).atan2(end.rho - center.rho);
        let mut sweep = a1 - a0;
        if ccw {
            while sweep <= 0.0 {
                sweep += std::f64::consts::TAU;
            }
        } else {
            while sweep >= 0.0 {
                sweep -= std::f64::consts::TAU;
            }
        }
        sweep
    }
    /// The point at `fraction` of the way along.
    fn at(&self, fraction: f64) -> P2 {
        match *self {
            Piece::Line { start, end } => start.add(end.sub(start).scale(fraction)),
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let a = a0 + fraction * Piece::sweep(center, start, end, ccw);
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            }
        }
    }
    /// The distance from `p` to this piece.
    fn distance(&self, p: P2) -> f64 {
        let ends = p.sub(self.start()).length().min(p.sub(self.end()).length());
        match *self {
            Piece::Line { start, end } => {
                let along = end.sub(start);
                let length2 = along.rho * along.rho + along.z * along.z;
                if !(length2 > 0.0) {
                    return ends;
                }
                let w = p.sub(start);
                let t = (w.rho * along.rho + w.z * along.z) / length2;
                if (0.0..=1.0).contains(&t) { p.sub(start.add(along.scale(t))).length() } else { ends }
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let sweep = Piece::sweep(center, start, end, ccw);
                let mut delta = (p.z - center.z).atan2(p.rho - center.rho) - a0;
                if ccw {
                    delta = delta.rem_euclid(std::f64::consts::TAU);
                } else {
                    delta = -(-delta).rem_euclid(std::f64::consts::TAU);
                }
                if delta.abs() <= sweep.abs() { (p.sub(center).length() - radius).abs() } else { ends }
            }
        }
    }
    /// The point halfway along.
    fn middle(&self) -> P2 {
        match *self {
            Piece::Line { start, end } => start.add(end).scale(0.5),
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let a = a0 + 0.5 * Piece::sweep(center, start, end, ccw);
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            }
        }
    }
    /// The unit tangent at the start (`at_end` false) or the end.
    fn tangent(&self, at_end: bool) -> Option<P2> {
        let t = match *self {
            Piece::Line { start, end } => end.sub(start),
            Piece::Arc { center, start, end, ccw, .. } => {
                let p = if at_end { end } else { start };
                let r = p.sub(center);
                if ccw { P2 { rho: -r.z, z: r.rho } } else { P2 { rho: r.z, z: -r.rho } }
            }
        };
        let length = t.length();
        (length > 0.0).then(|| t.scale(1.0 / length))
    }
    /// Twice the signed area this piece sweeps with the origin of (ρ, z)
    /// (exact for an arc: the chord's triangle plus the segment).
    fn area2(&self) -> f64 {
        let (start, end) = (self.start(), self.end());
        let chord = start.cross(end);
        match *self {
            Piece::Line { .. } => chord,
            Piece::Arc { center, radius, start, end, ccw } => {
                let sweep = Piece::sweep(center, start, end, ccw);
                chord + radius * radius * (sweep - sweep.sin())
            }
        }
    }
}

/// The lane's answer, or `None` when it declines or the switch is off.
pub(super) fn revolved_parallel(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    if !revolved_parallel_on() || !(distance > 0.0) {
        return None;
    }
    match revolved_parallel_impl(source, distance) {
        Ok(Some(solid)) => {
            let face_images = grown_provenance(source, &solid, distance);
            Some(OffsetShellResultRecord { solid, face_images })
        }
        Ok(None) => None,
        Err(error) => {
            os_debug!("revolved parallel lane declined: {error}");
            None
        }
    }
}

/// Each grown face's source face: the one whose trimmed face holds the face's
/// middle point moved back by `distance` along the face's outward normal, with
/// the same outward normal there. A face moved off one source face reads that
/// face; a rim's torus or a corner's ball reads 0 (moved back, its point lands
/// on the rim or the corner, where no source face has its normal).
fn grown_provenance(source: &BrepSolid, solid: &BrepSolid, distance: f64) -> Vec<OffsetShellFaceImageRecord> {
    let scale = crate::solid_scale(source).max(1.0);
    let sources: Vec<&FaceRecord> = source.shells.iter().flat_map(|shell| &shell.faces).collect();
    let source_of = |face: &FaceRecord| -> Option<u64> {
        let [u0, u1] = face.surface.domain_u().ok()?;
        let [v0, v1] = face.surface.domain_v().ok()?;
        let (u, v) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        let normal = super::face_normal(face, u, v).ok()?;
        let back = face.surface.evaluate(u, v).ok()?.sub(normal.scale(distance));
        sources.iter().find_map(|candidate| {
            let foot = project_point_to_surface(&candidate.surface, back).ok()?;
            if foot.distance > 1e-7 * scale {
                return None;
            }
            let along = super::face_normal(candidate, foot.u, foot.v).ok()?;
            // Inside that face's own trim: two faces may share one carrier.
            let inside = parameter_point_in_face(candidate, Vec2 { x: foot.u, y: foot.v }, 1e-9).is_ok_and(|class| class == PolygonClass::Inside);
            (inside && along.dot(normal) >= 1.0 - 1e-6).then_some(candidate.id)
        })
    };
    solid
        .shells
        .iter()
        .flat_map(|shell| &shell.faces)
        .map(|face| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: source_of(face).unwrap_or(0) })
        .collect()
}

/// The axis (a point and a unit direction) and the body's frame across it.
struct Axis {
    origin: Vec3,
    axis: Vec3,
    x: Vec3,
}

impl Axis {
    fn to_2d(&self, p: Vec3) -> P2 {
        let w = p.sub(self.origin);
        let z = w.dot(self.axis);
        P2 { rho: w.sub(self.axis.scale(z)).length(), z }
    }
    /// The point of the meridian half-plane at azimuth `phi` from `x`.
    fn to_3d(&self, p: P2, phi: f64) -> Vec3 {
        let y = self.axis.cross(self.x);
        let radial = self.x.scale(phi.cos()).add(y.scale(phi.sin()));
        self.origin.add(radial.scale(p.rho)).add(self.axis.scale(p.z))
    }
    fn on_axis(&self, p: Vec3, tolerance: f64) -> bool {
        let w = p.sub(self.origin);
        w.sub(self.axis.scale(w.dot(self.axis))).length() <= tolerance
    }
    fn parallel(&self, direction: Vec3) -> bool {
        direction.normalized().is_ok_and(|d| d.cross(self.axis).length() <= 1e-9)
    }
}

/// Whether the half-section point `p` (at an azimuth off any seam) lies on `face`.
fn on_face(face: &FaceRecord, axis: &Axis, p: P2, tolerance: f64) -> bool {
    let point = axis.to_3d(p, 1.0);
    let Ok(hit) = project_point_to_surface(&face.surface, point) else { return false };
    if hit.distance > tolerance {
        return false;
    }
    parameter_point_in_face(face, Vec2 { x: hit.u, y: hit.v }, 1e-9).is_ok_and(|class| class == PolygonClass::Inside)
}

/// `BREP_OFFSET_REVOLVED_COLLAPSED_ARC` (default on; `=0` restores the
/// refusal): a concave section arc grown by exactly its radius.
fn collapsed_arc_enabled() -> bool {
    switch!("BREP_OFFSET_REVOLVED_COLLAPSED_ARC", true)
}

/// `BREP_OFFSET_REVOLVED_HAIR_CONCAVE` (default on; `=0` restores the
/// 1e-12 bar): see [`concave_turn`].
fn hair_concave_enabled() -> bool {
    switch!("BREP_OFFSET_REVOLVED_HAIR_CONCAVE", true)
}

/// Whether a corner turning by `turn` (the cross product of the outward
/// normals either side) is concave. A join the faces leave a hair off tangent
/// is not: grown by `distance` its two offsets overlap by about
/// `|turn| * distance`, the same bar under which a convex hair's corner arc is
/// dropped as sweeping nothing. An exact fillet's rim solved to a few 1e-12
/// off the equator of the ball it is tangent to turns -3e-12 there, and was
/// read as a concave corner. Only normals that point the same way are a hair:
/// a cross product as small at nearly opposite normals is a cusp (a narrow
/// V-groove's floor), concave.
fn concave_turn(n1: P2, n2: P2, distance: f64, tolerance: f64) -> bool {
    let turn = n1.cross(n2);
    if !hair_concave_enabled() {
        return turn < -1e-12;
    }
    let same_way = n1.rho * n2.rho + n1.z * n2.z > 0.0;
    turn < -1e-12 && (!same_way || -turn * distance > tolerance.max(1e-7 * distance))
}

/// `BREP_OFFSET_REVOLVED_FITTED_TORUS` (default on; `=0` restores): see
/// [`fitted_meridian_circle`].
fn fitted_meridian_circle_enabled() -> bool {
    switch!("BREP_OFFSET_REVOLVED_FITTED_TORUS", true)
}

/// The meridian circle (centre, radius) a face with no analytic reading lies
/// on, or `None`: its surface sampled over the face's parameter box (and its
/// boundary), every sample read in the meridian half-plane, a circle fitted
/// to them, and every sample within `tolerance` of it. A filleted ball's root
/// (a ball on a plate, its tangent fillet rolled round the circle where they
/// meet) is such a patch: the lane declined the body ("not read") and its
/// rounded growth refused.
fn fitted_meridian_circle(face: &FaceRecord, axis: &Axis, tolerance: f64) -> Result<Option<(P2, f64)>, KernelRefusal> {
    // Stations that SETTLE the question, not a sample of it: on one knot
    // span the surface is (X, Y, Z) / W with polynomials of degree p in u and
    // q in v, so the torus's implicit quartic times W^4 is a polynomial of
    // degree 4p by 4q there. It vanishes on the whole span (the patch lies on
    // the torus) when it is small at 4p + 1 by 4q + 1 Chebyshev stations, up
    // to their Lebesgue constant (about 3 at these counts), which the
    // tolerance below keeps a factor 4 of. A bump of any shape on the span
    // (degree-elevated or not) is a non-zero polynomial of that degree, and
    // shows at its stations.
    let stations = |knots: &[f64], degree: usize| -> Vec<f64> {
        let mut distinct: Vec<f64> = Vec::new();
        for &knot in knots {
            if distinct.last().is_none_or(|last| knot > *last) {
                distinct.push(knot);
            }
        }
        let count = 4 * degree.max(1) + 1;
        let mut out = Vec::new();
        for pair in distinct.windows(2) {
            out.push(pair[0]);
            for k in 0..count {
                let node = 0.5 - 0.5 * (std::f64::consts::PI * (2 * k + 1) as f64 / (2 * count) as f64).cos();
                out.push(pair[0] + (pair[1] - pair[0]) * node);
            }
        }
        if let Some(last) = distinct.last() {
            out.push(*last);
        }
        out
    };
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Classify, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Classify, "domain_v")?;
    let in_domain = |t: f64, lo: f64, hi: f64| t >= lo && t <= hi;
    let us: Vec<f64> = stations(&face.surface.knots_u, face.surface.degree_u).into_iter().filter(|t| in_domain(*t, u0, u1)).collect();
    let vs: Vec<f64> = stations(&face.surface.knots_v, face.surface.degree_v).into_iter().filter(|t| in_domain(*t, v0, v1)).collect();
    if us.len() < 2 || vs.len() < 2 || us.len() * vs.len() > 40_000 {
        return Ok(None);
    }
    let mut points = Vec::new();
    for &u in &us {
        for &v in &vs {
            let p = face.surface.evaluate(u, v).or_refuse(KernelStage::Classify, "evaluate")?;
            points.push(axis.to_2d(p));
        }
    }
    // Algebraic (Kasa) fit: rho^2 + z^2 + D rho + E z + F = 0, centred on the
    // mean for conditioning.
    let n = points.len() as f64;
    let (mr, mz) = (points.iter().map(|p| p.rho).sum::<f64>() / n, points.iter().map(|p| p.z).sum::<f64>() / n);
    let mut m = [[0.0f64; 3]; 3];
    let mut rhs = [0.0f64; 3];
    for p in &points {
        let (x, y) = (p.rho - mr, p.z - mz);
        let row = [x, y, 1.0];
        let target = -(x * x + y * y);
        for r in 0..3 {
            for c in 0..3 {
                m[r][c] += row[r] * row[c];
            }
            rhs[r] += row[r] * target;
        }
    }
    let det = |m: &[[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let whole = det(&m);
    if !(whole.abs() > 1e-300) {
        return Ok(None);
    }
    let mut solution = [0.0f64; 3];
    for k in 0..3 {
        let mut replaced = m;
        for r in 0..3 {
            replaced[r][k] = rhs[r];
        }
        solution[k] = det(&replaced) / whole;
    }
    let [d, e, f] = solution;
    let center = P2 { rho: mr - d * 0.5, z: mz - e * 0.5 };
    let squared = d * d * 0.25 + e * e * 0.25 - f;
    if !(squared > 0.0) {
        return Ok(None);
    }
    let radius = squared.sqrt();
    let worst = points
        .iter()
        .map(|p| ((p.rho - center.rho).hypot(p.z - center.z) - radius).abs())
        .fold(0.0f64, f64::max);
    if worst > tolerance * 0.25 {
        os_debug!("revolved parallel lane: face {} leaves its meridian circle by {worst:.3e}", face.id);
        return Ok(None);
    }
    Ok(Some((center, radius)))
}

/// A solid of revolution's half-section when every piece of it is a segment:
/// the axis frame (`origin`, unit `axis`, unit radial `x`), the corners in
/// (ρ, z) counter-clockwise, and `on_axis[k]` for the run from corner `k` to
/// `k + 1` that closes the section along the axis (no face of the body).
pub(super) struct HalfPolygon {
    pub origin: Vec3,
    pub axis: Vec3,
    pub x: Vec3,
    pub corners: Vec<(f64, f64)>,
    pub on_axis: Vec<bool>,
}

/// [`HalfPolygon`] of `source`, or `None`.
pub(super) fn half_section_polygon(source: &BrepSolid) -> Option<HalfPolygon> {
    half_section_polygon_reading(source, false)
}

/// [`half_section_polygon`], also reading a cone to its apex on the axis as a
/// segment when `cone_apex` (the revolved shell's reading; the eroders and the
/// grower keep theirs).
pub(super) fn half_section_polygon_reading(source: &BrepSolid, cone_apex: bool) -> Option<HalfPolygon> {
    let (axis, chain) = read_half_section(source, true, cone_apex).ok()??;
    let snap = 1e-7 * crate::solid_scale(source).max(1.0);
    let mut runs: Vec<(P2, P2, bool)> = Vec::with_capacity(chain.len() + 1);
    for piece in &chain {
        let Piece::Line { start, end } = *piece else { return None };
        runs.push((start, end, false));
    }
    let (head, tail) = (runs[0].0, runs[runs.len() - 1].1);
    if head.rho.abs() <= snap && tail.rho.abs() <= snap {
        let last = runs.len() - 1;
        runs[0].0.rho = 0.0;
        runs[last].1.rho = 0.0;
        runs.push((P2 { rho: 0.0, z: tail.z }, P2 { rho: 0.0, z: head.z }, true));
    } else if head.sub(tail).length() > snap {
        return None;
    }
    let area2: f64 = runs.iter().map(|(a, b, _)| a.rho * b.z - b.rho * a.z).sum();
    if area2 < 0.0 {
        runs = runs.into_iter().rev().map(|(a, b, on)| (b, a, on)).collect();
    }
    Some(HalfPolygon {
        origin: axis.origin,
        axis: axis.axis,
        x: axis.x,
        corners: runs.iter().map(|(a, _, _)| (a.rho, a.z)).collect(),
        on_axis: runs.iter().map(|(_, _, on)| *on).collect(),
    })
}

/// Whether `face` is a revolved segment about `axis` (see
/// [`meridian_is_exactly_straight`]). It was read from a 7 × 7 grid of samples
/// until a spline side bumped inside one knot span read straight and the
/// eroders shrank the tube as if the bump were not there.
fn meridian_is_straight(face: &FaceRecord, axis: &Axis, tolerance: f64) -> Result<bool, KernelRefusal> {
    Ok(meridian_is_exactly_straight(face, axis, tolerance))
}

/// A meridian reads straight only by its construction, never by samples (a
/// bump inside one knot span, or weights that pin every station to the line,
/// pass any sampling): the face's net must
/// be `make_revolution` of its own generatrix about the lane's axis (points to
/// `tolerance`, weights' ratios to 1e-12), the generatrix in one meridian
/// half-plane, and its control points on one line of (ρ, z). With positive
/// weights the curve lies in its controls' hull, so on that line.
fn meridian_is_exactly_straight(face: &FaceRecord, axis: &Axis, tolerance: f64) -> bool {
    let surface = &face.surface;
    let Some(structure) = crate::revolution_structure(surface) else { return false };
    if !axis.parallel(structure.frame.axis) || !axis.on_axis(structure.frame.origin, tolerance) {
        return false;
    }
    let generatrix = &structure.generatrix;
    let Ok(rebuilt) = crate::make_revolution(structure.frame.origin, structure.frame.axis, generatrix, structure.sweep) else { return false };
    let same_knots = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() <= 1e-12);
    if !same_knots(&rebuilt.knots_u, &surface.knots_u)
        || !same_knots(&rebuilt.knots_v, &surface.knots_v)
        || rebuilt.degree_u != surface.degree_u
        || rebuilt.degree_v != surface.degree_v
        || rebuilt.control_points.len() != surface.control_points.len()
    {
        return false;
    }
    for (row, stored) in rebuilt.control_points.iter().zip(&surface.control_points) {
        if row.len() != stored.len() {
            return false;
        }
        for (built, held) in row.iter().zip(stored) {
            let (Ok(p), Ok(q)) = (built.point(), held.point()) else { return false };
            if !(held.w > 0.0) || p.sub(q).length() > tolerance || (built.w - held.w).abs() > 1e-12 * held.w {
                return false;
            }
        }
    }
    let mut controls = Vec::with_capacity(generatrix.control_points.len());
    for control in &generatrix.control_points {
        let Ok(point) = control.point() else { return false };
        if !(control.w > 0.0) {
            return false;
        }
        controls.push(point);
    }
    // One meridian half-plane: every control's radial part along one direction.
    let radial = |p: Vec3| {
        let from = p.sub(axis.origin);
        from.sub(axis.axis.scale(from.dot(axis.axis)))
    };
    let Some(reference) = controls.iter().map(|p| radial(*p)).find(|r| r.length() > tolerance) else { return false };
    let unit = reference.scale(1.0 / reference.length());
    if controls.iter().any(|p| {
        let r = radial(*p);
        r.cross(unit).length() > tolerance || r.dot(unit) < -tolerance
    }) {
        return false;
    }
    let points: Vec<P2> = controls.iter().map(|p| axis.to_2d(*p)).collect();
    let (mut a, mut b, mut span) = (points[0], points[0], 0.0);
    for p in &points {
        for q in &points {
            let d = p.sub(*q).length();
            if d > span {
                (a, b, span) = (*p, *q, d);
            }
        }
    }
    if span <= tolerance {
        return false;
    }
    let run = b.sub(a);
    points.iter().all(|p| run.cross(p.sub(a)).abs() / span <= tolerance)
}

/// An edge that is an arc of a circle: the circle through its ends and its
/// middle, every one of 32 samples on it within `tolerance`. `None` otherwise.
fn fit_rim_arc(edge: &EdgeRecord, tolerance: f64) -> Result<Option<super::rim_welds::RimCircle>, KernelRefusal> {
    let at = |fraction: f64| edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * fraction).or_refuse(KernelStage::Classify, "evaluate");
    let (a, m, b) = (at(0.0)?, at(0.5)?, at(1.0)?);
    let (u, w) = (m.sub(a), b.sub(a));
    let normal = u.cross(w);
    let Ok(axis) = normal.normalized() else { return Ok(None) };
    // The circumcentre of a, m, b.
    let n2 = normal.dot(normal);
    let center = a.add(w.cross(normal).scale(u.dot(u) / (2.0 * n2)).add(normal.cross(u).scale(w.dot(w) / (2.0 * n2))));
    let radius = a.sub(center).length();
    if !(radius > tolerance) {
        return Ok(None);
    }
    for k in 0..=32 {
        let radial = at(k as f64 / 32.0)?.sub(center);
        if radial.dot(axis).abs() > tolerance || (radial.length() - radius).abs() > tolerance {
            return Ok(None);
        }
    }
    Ok(Some(super::rim_welds::RimCircle { center, axis, radius }))
}

/// The body's axis and its half-section: the pieces in boundary order, from
/// an axis end round to the other or round a loop clear of the axis. `None`
/// where the body is not a solid of revolution this lane reads.
///
/// With `split_rims` a rim may be cut into arcs (a union cuts a drum's rim
/// where the other operand's seam meets it): each arc is read as the exact
/// circle through its ends and middle, and the arcs of one rim between the
/// same two faces are one rim.
fn read_half_section(source: &BrepSolid, split_rims: bool, cone_apex: bool) -> Result<Option<(Axis, Vec<Piece>)>, KernelRefusal> {
    let [shell] = source.shells.as_slice() else { return Ok(None) };
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;

    // Each edge: its faces, by how many coedges of each face walk it.
    let mut uses: std::collections::HashMap<u64, Vec<usize>> = std::collections::HashMap::new();
    for (index, face) in shell.faces.iter().enumerate() {
        for coedge in face.loops.iter().flat_map(|lp| &lp.coedges) {
            uses.entry(coedge.edge_id).or_default().push(index);
        }
    }
    // The rims: edges between two different faces. Each must be a circle,
    // all about one axis.
    let mut axis: Option<Axis> = None;
    let mut rims: Vec<(u64, P2, [usize; 2])> = Vec::new();
    for edge in &source.edges {
        let Some(faces) = uses.get(&edge.id) else { continue };
        if edge.degenerate {
            continue;
        }
        if faces.len() == 2 && faces[0] == faces[1] {
            continue; // a seam
        }
        let [a, b] = faces.as_slice() else { return Ok(None) };
        let fitted = match super::rim_welds::fit_rim_circle(edge, tolerance)? {
            Some(circle) => Some(circle),
            None if split_rims => fit_rim_arc(edge, 1e-7 * scale)?,
            None => None,
        };
        let Some(circle) = fitted else { return Ok(None) };
        if axis.is_none() {
            let x = circle.axis.perpendicular().or_refuse(KernelStage::Classify, "perpendicular")?;
            axis = Some(Axis { origin: circle.center, axis: circle.axis, x });
        }
        let frame = axis.as_ref().expect("set above");
        if !frame.parallel(circle.axis) || !frame.on_axis(circle.center, tolerance) {
            return Ok(None);
        }
        let at = frame.to_2d(edge.curve.evaluate(edge.t0).or_refuse(KernelStage::Classify, "evaluate")?);
        if split_rims && rims.iter().any(|(_, p, f)| (*f == [*a, *b] || *f == [*b, *a]) && p.sub(at).length() <= 1e-7 * scale) {
            continue;
        }
        rims.push((edge.id, at, [*a, *b]));
    }
    let Some(axis) = axis else { return Ok(None) };

    // Each face's piece of the half-section.
    let mut pieces: Vec<(usize, Piece, [Option<usize>; 2])> = Vec::new();
    for (index, face) in shell.faces.iter().enumerate() {
        let ends: Vec<usize> = rims.iter().enumerate().filter(|(_, rim)| rim.2.contains(&index)).map(|(k, _)| k).collect();
        let generatrix = match face.surface.analytic() {
            Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
                if !axis.parallel(u_dir.cross(*v_dir)) {
                    return Ok(None);
                }
                None
            }
            Some(AnalyticSurface::RuledRevolution { frame, .. }) => {
                if !axis.parallel(frame.axis) || !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                None
            }
            Some(AnalyticSurface::Sphere { frame, radius }) => {
                if !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                Some((axis.to_2d(frame.origin), *radius))
            }
            Some(AnalyticSurface::Torus { frame, major_radius, minor_radius }) => {
                if !axis.parallel(frame.axis) || !axis.on_axis(frame.origin, tolerance) {
                    return Ok(None);
                }
                let h = frame.origin.sub(axis.origin).dot(axis.axis);
                Some((P2 { rho: *major_radius, z: h }, *minor_radius))
            }
            // A face with no analytic reading (a fillet rolled round a
            // rim is a fitted NURBS patch), or an exact revolution of a
            // conic arc (a constant fillet round a sphere-plane rim, built
            // by the blend's exact revolution lane): it lies on a torus
            // about the axis when every point of it lies on one circle of
            // the meridian plane.
            // With split rims (the eroders' reading), a revolved face whose
            // meridian is straight (a revolved segment: a disk, an annulus, a
            // cylinder or cone) is a segment of the section.
            None | Some(AnalyticSurface::Revolution { .. }) if split_rims && meridian_is_straight(face, &axis, 1e-7 * scale)? => None,
            None | Some(AnalyticSurface::Revolution { .. }) if fitted_meridian_circle_enabled() => match fitted_meridian_circle(face, &axis, 1e-7 * scale)? {
                Some(circle) => Some(circle),
                None => return Ok(None),
            },
            _ => return Ok(None),
        };
        let piece = match (ends.as_slice(), generatrix) {
            // Between two rims: a segment, or the arc of the generatrix circle
            // through the face.
            ([a, b], None) => Piece::Line { start: rims[*a].1, end: rims[*b].1 },
            ([a, b], Some((center, radius))) => {
                let (start, end) = (rims[*a].1, rims[*b].1);
                let ccw = Piece::Arc { center, radius, start, end, ccw: true };
                let cw = Piece::Arc { center, radius, start, end, ccw: false };
                // The arc that stays in the half-plane and runs through the face.
                let holds = |piece: &Piece| piece.middle().rho > tolerance && on_face(face, &axis, piece.middle(), 1e-7 * scale);
                match (holds(&ccw), holds(&cw)) {
                    (true, false) => ccw,
                    (false, true) => cw,
                    _ => return Ok(None),
                }
            }
            // From one rim to the axis: a disk, a cone to its apex, a sphere to
            // the pole on the face.
            ([a], None) => {
                let start = rims[*a].1;
                let axis_end = match face.surface.analytic() {
                    Some(AnalyticSurface::Plane { origin, .. }) => P2 { rho: 0.0, z: axis.to_2d(*origin).z },
                    // A cone to its apex: the generatrix's end of radius 0.
                    Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) if cone_apex => {
                        let along = if rho1.abs() <= tolerance && rho0.abs() > tolerance {
                            *height
                        } else if rho0.abs() <= tolerance && rho1.abs() > tolerance {
                            0.0
                        } else {
                            return Ok(None);
                        };
                        P2 { rho: 0.0, z: axis.to_2d(frame.origin.add(frame.axis.scale(along))).z }
                    }
                    _ => return Ok(None),
                };
                Piece::Line { start, end: axis_end }
            }
            ([a], Some((center, radius))) => {
                if center.rho.abs() > tolerance {
                    return Ok(None);
                }
                let start = rims[*a].1;
                let mut found = None;
                for pole in [P2 { rho: 0.0, z: center.z + radius }, P2 { rho: 0.0, z: center.z - radius }] {
                    let ccw = Piece::sweep(center, start, pole, true).abs() <= std::f64::consts::PI + 1e-12;
                    let piece = Piece::Arc { center, radius, start, end: pole, ccw };
                    if on_face(face, &axis, piece.middle(), 1e-7 * scale) {
                        if found.is_some() {
                            return Ok(None);
                        }
                        found = Some(piece);
                    }
                }
                match found {
                    Some(piece) => piece,
                    None => return Ok(None),
                }
            }
            _ => return Ok(None),
        };
        let ends: Vec<Option<usize>> = ends.iter().map(|k| Some(*k)).collect();
        pieces.push((index, piece, [ends.first().copied().flatten(), ends.get(1).copied().flatten()]));
    }

    // Order the pieces into one chain from the axis back to the axis, or one
    // loop clear of it, joined at the rims.
    let chain = order_pieces(&pieces, &rims, tolerance)?;
    Ok(chain.map(|chain| (axis, chain)))
}

fn revolved_parallel_impl(source: &BrepSolid, distance: f64) -> Result<Option<BrepSolid>, KernelRefusal> {
    let scale = crate::solid_scale(source).max(1.0);
    let tolerance = 1e-9 * scale;
    // A boolean's result carries each rim as arcs, not one circle: the
    // trimmed lane reads those too (the eroders' reading).
    let read = match read_half_section(source, false, false)? {
        Some(read) => Some(read),
        None if trimmed_growth_on() => read_half_section(source, true, false)?,
        None => None,
    };
    let Some((axis, mut chain)) = read else { return Ok(None) };
    let touches_axis = chain.first().is_some_and(|p| p.start().rho.abs() <= tolerance);
    // A closed loop has no first piece: start it on a piece that does not
    // collapse (a concave arc grown by its radius is dropped between its two
    // neighbours, so it must not sit at either end of the walk).
    if !touches_axis {
        let collapses = |piece: &Piece| {
            matches!(*piece, Piece::Arc { radius, ccw: false, .. } if (radius - distance).abs() <= 1e-7 * scale)
        };
        let count = chain.len();
        if let Some(start) = (0..count).find(|&k| !collapses(&chain[k]) && !collapses(&chain[(k + count - 1) % count])) {
            chain.rotate_left(start);
        }
    }
    // The closing run along the axis (ρ = 0) sweeps no area about the origin
    // of (ρ, z), so the pieces alone give the orientation.
    let area2: f64 = chain.iter().map(Piece::area2).sum();
    if area2 < 0.0 {
        chain = chain.iter().rev().map(Piece::reversed).collect();
    }

    let grown = match walked_growth(&chain, touches_axis, distance, scale, tolerance) {
        Some(grown) => grown,
        None if trimmed_growth_on() => match trimmed_growth(&chain, touches_axis, distance, scale, tolerance) {
            Some(grown) => grown,
            None => return Ok(None),
        },
        None => return Ok(None),
    };

    // The grown half-section as curves in the plane of `x` and the axis, revolved.
    let mut curves: Vec<NurbsCurve> = Vec::with_capacity(grown.len());
    for piece in &grown {
        let curve = match *piece {
            Piece::Line { start, end } => {
                if start.sub(end).length() <= tolerance {
                    continue;
                }
                make_line(axis.to_3d(start, 0.0), axis.to_3d(end, 0.0)).or_refuse(KernelStage::Sew, "make_line")?
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let sweep = Piece::sweep(center, start, end, ccw);
                // A corner arc at a join the fitted faces leave a hair off
                // tangent (a fillet's rims) sweeps nothing: the pieces meet.
                if sweep.abs() * radius <= tolerance.max(1e-7 * radius) {
                    continue;
                }
                let c = axis.to_3d(center, 0.0);
                if ccw {
                    make_arc(c, axis.x, axis.axis, radius, a0, a0 + sweep).or_refuse(KernelStage::Sew, "make_arc")?
                } else {
                    make_arc(c, axis.x, axis.axis, radius, a0 + sweep, a0)
                        .and_then(|arc| arc.reversed())
                        .or_refuse(KernelStage::Sew, "make_arc")?
                }
            }
        };
        curves.push(curve);
    }
    let solid = revolve_profile_brep(&curves, axis.origin, axis.axis, std::f64::consts::TAU).or_refuse(KernelStage::Sew, "revolve")?;
    if !solid.validate().is_empty() {
        return Ok(None);
    }
    os_debug!("revolved parallel lane built {} faces from {} section pieces", solid.shells.iter().map(|s| s.faces.len()).sum::<usize>(), grown.len());
    Ok(Some(solid))
}

/// The growth read corner by corner: every piece moved out, every convex corner
/// rounded; `None` at a concave corner or anything else the walk declines.
fn walked_growth(chain: &[Piece], touches_axis: bool, distance: f64, scale: f64, tolerance: f64) -> Option<Vec<Piece>> {
    // Offset every piece outward; round every corner; decline a concave one.
    let outward = |piece: &Piece, at_end: bool| -> Option<P2> {
        piece.tangent(at_end).map(|t| P2 { rho: t.z, z: -t.rho })
    };
    let mut grown: Vec<Piece> = Vec::new();
    // A concave arc grown by exactly its radius (a fillet's root grown by the
    // fillet's radius) shrinks to its centre: it is dropped, and its two
    // neighbours' offsets meet there in a sharp edge, with no corner arc.
    let collapse = 1e-7 * scale;
    let mut after_collapse = false;
    // A concave arc grown PAST its radius vanishes: its neighbours' offsets
    // cross and are cut at the crossing, a sharp concave crease
    // (`BREP_OFFSET_REVOLVED_VANISHED_ARC`). The arc's centre picks the
    // crossing; the check below that every grown piece lies the distance from
    // the section stands guard.
    let mut vanished: Option<P2> = None;
    for (k, piece) in chain.iter().enumerate() {
        let moved = match *piece {
            Piece::Line { start, end } => {
                let Some(n) = outward(piece, false) else { return None };
                Piece::Line { start: start.add(n.scale(distance)), end: end.add(n.scale(distance)) }
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let r = if ccw { radius + distance } else { radius - distance };
                if !ccw && r.abs() <= collapse && collapsed_arc_enabled() && k > 0 && k + 1 < chain.len() {
                    after_collapse = true;
                    continue;
                }
                if !ccw && r < -collapse && vanished_arc_on() && vanished.is_none() && k > 0 && k + 1 < chain.len() {
                    // Only a FILLET vanishes so: an arc tangent to both
                    // neighbours, the rolling disk between them. A concave arc
                    // meeting either at an angle is no closing, and extending
                    // its neighbours would add material past the distance.
                    let joins = |a: Option<P2>, b: Option<P2>| matches!((a, b), (Some(a), Some(b)) if a.sub(b).length() <= 1e-6);
                    if !joins(outward(&chain[k - 1], true), outward(piece, false)) || !joins(outward(piece, true), outward(&chain[k + 1], false)) {
                        os_debug!("revolved parallel lane: a concave arc grown past its radius is not tangent to its neighbours");
                        return None;
                    }
                    vanished = Some(center);
                    continue;
                }
                if !(r > tolerance) {
                    return None;
                }
                let k = r / radius;
                Piece::Arc { center, radius: r, start: center.add(start.sub(center).scale(k)), end: center.add(end.sub(center).scale(k)), ccw }
            }
        };
        if let Some(hub) = vanished.take() {
            let Some(last) = grown.pop() else { return None };
            let Some(crease) = crossing_nearest(&last, &moved, hub) else {
                os_debug!("revolved parallel lane: the offsets beside a vanished arc do not cross");
                return None;
            };
            // The crease lies exactly the distance from the section, no farther
            // (the check below reads only the near side).
            let reach = chain.iter().map(|source| source.distance(crease)).fold(f64::INFINITY, f64::min);
            if (reach - distance).abs() > 1e-7 * scale {
                os_debug!("revolved parallel lane: the crease lies {reach:.9} from the section, not {distance:.9}");
                return None;
            }
            grown.push(with_end(&last, crease));
            grown.push(with_start(&moved, crease));
            continue;
        }
        if after_collapse {
            after_collapse = false;
            let Some(last) = grown.last() else { return None };
            if last.end().sub(moved.start()).length() > collapse {
                os_debug!("revolved parallel lane: the offsets beside a collapsed arc miss by {:.3e}", last.end().sub(moved.start()).length());
                return None;
            }
            grown.push(moved);
            continue;
        }
        if k > 0 {
            let previous = &chain[k - 1];
            let corner = piece.start();
            let (Some(n1), Some(n2)) = (outward(previous, true), outward(piece, false)) else { return None };
            let turn = n1.cross(n2);
            if concave_turn(n1, n2, distance, tolerance) {
                os_debug!("revolved parallel lane: concave corner at ({:.6}, {:.6}), turn {turn:.3e}", corner.rho, corner.z);
                return None; // concave
            }
            if turn > 1e-12 || n1.rho * n2.rho + n1.z * n2.z < 0.0 {
                grown.push(Piece::Arc {
                    center: corner,
                    radius: distance,
                    start: corner.add(n1.scale(distance)),
                    end: corner.add(n2.scale(distance)),
                    ccw: true,
                });
            }
        }
        grown.push(moved);
    }
    if touches_axis {
        let (first, last) = (grown[0].start(), grown[grown.len() - 1].end());
        if first.rho.abs() > tolerance || last.rho.abs() > tolerance {
            return None; // an end leaves the axis: an apex, not built here
        }
        grown.push(Piece::Line { start: last, end: first });
    } else {
        let (last, first) = (&chain[chain.len() - 1], &chain[0]);
        let corner = first.start();
        let (Some(n1), Some(n2)) = (outward(last, true), outward(first, false)) else { return None };
        let turn = n1.cross(n2);
        if concave_turn(n1, n2, distance, tolerance) {
            return None;
        }
        if turn > 1e-12 || n1.rho * n2.rho + n1.z * n2.z < 0.0 {
            grown.push(Piece::Arc {
                center: corner,
                radius: distance,
                start: corner.add(n1.scale(distance)),
                end: corner.add(n2.scale(distance)),
                ccw: true,
            });
        }
    }
    // A loop clear of the axis whose growth crosses it (a bore grown shut):
    // the grown half-section is the loop's own growth cut at ρ = 0, since a
    // point with ρ ≥ 0 is never nearer the section's mirror half than its own
    // ((ρ + a)² ≥ (ρ − a)² for a ≥ 0). The part kept must be one run, closed
    // up the axis.
    if !touches_axis && grown.iter().any(|piece| piece.start().rho < -tolerance || piece.middle().rho < -tolerance) {
        let split: Vec<Piece> = grown.iter().flat_map(|piece| split_at_axis(piece, tolerance)).collect();
        let kept: Vec<bool> = split.iter().map(|piece| piece.middle().rho >= -tolerance).collect();
        let count = split.len();
        let starts: Vec<usize> = (0..count).filter(|&k| kept[k] && !kept[(k + count - 1) % count]).collect();
        let [first] = starts.as_slice() else { return None };
        let mut run: Vec<Piece> = Vec::new();
        for offset in 0..count {
            let k = (first + offset) % count;
            if !kept[k] {
                break;
            }
            run.push(split[k]);
        }
        let (head, tail) = (run[0].start(), run[run.len() - 1].end());
        if head.rho.abs() > 1e-7 * scale || tail.rho.abs() > 1e-7 * scale {
            return None;
        }
        let snap = |p: P2| P2 { rho: 0.0, z: p.z };
        let last = run.len() - 1;
        run[0] = with_start(&run[0], snap(head));
        run[last] = with_end(&run[last], snap(tail));
        run.push(Piece::Line { start: snap(tail), end: snap(head) });
        grown = run;
    }
    // Every grown piece stays in the half-plane.
    for piece in &grown {
        for p in [piece.start(), piece.end(), piece.middle()] {
            if p.rho < -tolerance {
                return None;
            }
        }
    }

    // The corners were read one at a time, so growths that meet across a gap
    // (the two rims of a narrow groove) are not seen there. Every point of the
    // true boundary lies exactly the distance from the section: a grown piece
    // with a point nearer than that to any source piece is not the boundary,
    // and the walk declines.
    if !holds_the_distance(&grown, chain, distance, scale, tolerance) {
        return None;
    }
    Some(grown)
}

/// Every grown piece (but the closing run along the axis) lies at least
/// `distance` from the section, read at 15 points along it.
fn holds_the_distance(grown: &[Piece], chain: &[Piece], distance: f64, scale: f64, tolerance: f64) -> bool {
    for piece in grown {
        if matches!(piece, Piece::Line { start, end } if start.rho.abs() <= tolerance && end.rho.abs() <= tolerance) {
            continue; // the closing run along the axis
        }
        for step in 1..16 {
            let p = piece.at(step as f64 / 16.0);
            let nearest = chain.iter().map(|source| source.distance(p)).fold(f64::INFINITY, f64::min);
            if nearest < distance - 1e-7 * scale {
                os_debug!("revolved parallel lane declined: a grown piece passes {nearest:.6} from the section, under {distance:.6}");
                return false;
            }
        }
    }
    true
}

/// `BREP_OFFSET_REVOLVED_TRIMMED` (default off): where the walk declines (a
/// concave corner, or growths that meet across a gap, such as a groove
/// narrower than twice the distance) or the rims are arcs, the grown
/// half-section is read whole. It is the boundary of the section's points
/// within `distance`, so it lies on the pieces moved out and the arcs of
/// radius `distance` about the convex corners. Each candidate is cut wherever
/// it crosses another or the axis, and a cut piece is kept when its middle is
/// at least `distance` from every source piece but its own. The kept pieces
/// must chain into one run between two axis ends, or one loop clear of the
/// axis. A tie (a cut piece exactly `distance` from another source piece), an
/// ambiguous join, a second loop (a sealed void) or two kept pieces that come
/// close away from their join all decline.
fn trimmed_growth_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_TRIMMED", false)
}


/// The grown half-section read whole (see [`trimmed_growth_on`]); `None` where
/// it declines. `chain` runs anticlockwise.
fn trimmed_growth(chain: &[Piece], touches_axis: bool, distance: f64, scale: f64, tolerance: f64) -> Option<Vec<Piece>> {
    let near = 1e-7 * scale;
    let outward = |piece: &Piece, at_end: bool| piece.tangent(at_end).map(|t| P2 { rho: t.z, z: -t.rho });
    // Each candidate with the source pieces its points keep the distance from.
    let count = chain.len();
    let mut candidates: Vec<(Piece, [usize; 2])> = Vec::new();
    for (k, piece) in chain.iter().enumerate() {
        if k > 0 || !touches_axis {
            let previous = (k + count - 1) % count;
            let (n1, n2) = (outward(&chain[previous], true)?, outward(piece, false)?);
            let turn = n1.cross(n2);
            if !concave_turn(n1, n2, distance, tolerance) && (turn > 1e-12 || n1.rho * n2.rho + n1.z * n2.z < 0.0) {
                let corner = piece.start();
                let arc = Piece::Arc { center: corner, radius: distance, start: corner.add(n1.scale(distance)), end: corner.add(n2.scale(distance)), ccw: true };
                candidates.push((arc, [previous, k]));
            }
        }
        let moved = match *piece {
            Piece::Line { start, end } => {
                let n = outward(piece, false)?;
                Piece::Line { start: start.add(n.scale(distance)), end: end.add(n.scale(distance)) }
            }
            Piece::Arc { center, radius, start, end, ccw } => {
                let r = if ccw { radius + distance } else { radius - distance };
                if r <= near {
                    continue; // a concave arc grown to its centre or past it
                }
                let k = r / radius;
                Piece::Arc { center, radius: r, start: center.add(start.sub(center).scale(k)), end: center.add(end.sub(center).scale(k)), ccw }
            }
        };
        candidates.push((moved, [k, k]));
    }
    // Cut each candidate where it crosses another.
    let mut cuts: Vec<Vec<P2>> = vec![Vec::new(); candidates.len()];
    for i in 0..candidates.len() {
        for j in i + 1..candidates.len() {
            let (a, b) = (&candidates[i].0, &candidates[j].0);
            for p in carrier_crossings(a, b, near) {
                if a.distance(p) <= near && b.distance(p) <= near {
                    cuts[i].push(p);
                    cuts[j].push(p);
                }
            }
        }
    }
    let mut kept: Vec<Piece> = Vec::new();
    for ((candidate, sources), points) in candidates.iter().zip(&cuts) {
        for part in cut_at(candidate, points, near) {
            for part in split_at_axis(&part, tolerance) {
                let middle = part.middle();
                if middle.rho < -tolerance {
                    continue;
                }
                let others = chain
                    .iter()
                    .enumerate()
                    .filter(|(k, _)| !sources.contains(k))
                    .map(|(_, source)| source.distance(middle))
                    .fold(f64::INFINITY, f64::min);
                if (others - distance).abs() <= near {
                    os_debug!("revolved trimmed lane: a cut piece ties at ({:.6}, {:.6})", middle.rho, middle.z);
                    return None;
                }
                if others > distance {
                    kept.push(part);
                }
            }
        }
    }
    if kept.is_empty() {
        return None;
    }
    // Chain the kept pieces: each end meets exactly one start.
    let close = |a: P2, b: P2| a.sub(b).length() <= near;
    let starts_after = |p: P2, used: &[bool]| -> Vec<usize> { (0..kept.len()).filter(|&k| !used[k] && close(kept[k].start(), p)).collect() };
    let free_starts: Vec<usize> = (0..kept.len()).filter(|&k| !kept.iter().any(|other| close(other.end(), kept[k].start()))).collect();
    let first = match free_starts.as_slice() {
        [] => 0,
        [one] if kept[*one].start().rho.abs() <= near => *one,
        _ => {
            os_debug!("revolved trimmed lane: {} kept pieces start where none ends", free_starts.len());
            return None;
        }
    };
    let on_axis = !free_starts.is_empty();
    let mut used = vec![false; kept.len()];
    used[first] = true;
    let mut grown = vec![kept[first]];
    loop {
        let tail = grown[grown.len() - 1].end();
        let next = starts_after(tail, &used);
        match next.as_slice() {
            [] => break,
            [one] => {
                used[*one] = true;
                grown.push(with_start(&kept[*one], tail));
            }
            _ => {
                os_debug!("revolved trimmed lane: an ambiguous join at ({:.6}, {:.6})", tail.rho, tail.z);
                return None;
            }
        }
    }
    if used.iter().any(|u| !u) {
        os_debug!("revolved trimmed lane: the kept pieces make more than one loop");
        return None;
    }
    let (head, tail) = (grown[0].start(), grown[grown.len() - 1].end());
    if on_axis {
        if tail.rho.abs() > near {
            return None;
        }
        let snap = |p: P2| P2 { rho: 0.0, z: p.z };
        let last = grown.len() - 1;
        grown[0] = with_start(&grown[0], snap(head));
        grown[last] = with_end(&grown[last], snap(tail));
        grown.push(Piece::Line { start: snap(tail), end: snap(head) });
    } else {
        if !close(head, tail) {
            return None;
        }
        let last = grown.len() - 1;
        grown[last] = with_end(&grown[last], head);
    }
    // Pieces that are not neighbours stay apart (no pinch, no sealed void
    // touching the boundary).
    let count = grown.len();
    for i in 0..count {
        for j in i + 1..count {
            if j == i + 1 || (i == 0 && j == count - 1) {
                continue;
            }
            if piece_gap(&grown[i], &grown[j]) <= 1e-6 * scale {
                os_debug!("revolved trimmed lane: pieces {i} and {j} of the grown section touch");
                return None;
            }
        }
    }
    for piece in &grown {
        if [piece.start(), piece.end(), piece.middle()].iter().any(|p| p.rho < -tolerance) {
            return None;
        }
    }
    if !holds_the_distance(&grown, chain, distance, scale, tolerance) {
        return None;
    }
    os_debug!("revolved trimmed lane: {} pieces from {} candidates", grown.len(), candidates.len());
    Some(grown)
}

/// Where the carriers of `a` and `b` (a line or a circle each) cross: none
/// where they are parallel or apart; where they coincide, the ends of each that
/// lie on the other.
fn carrier_crossings(a: &Piece, b: &Piece, near: f64) -> Vec<P2> {
    let circle = |piece: &Piece| match *piece {
        Piece::Arc { center, radius, .. } => Some((center, radius)),
        Piece::Line { .. } => None,
    };
    let line = |piece: &Piece| (piece.start(), piece.end().sub(piece.start()));
    let ends_on_each_other = || -> Vec<P2> {
        let mut points = Vec::new();
        for p in [a.start(), a.end()] {
            if b.distance(p) <= near {
                points.push(p);
            }
        }
        for p in [b.start(), b.end()] {
            if a.distance(p) <= near {
                points.push(p);
            }
        }
        points
    };
    let line_circle = |(p, u): (P2, P2), (c, r): (P2, f64)| -> Vec<P2> {
        let uu = u.rho * u.rho + u.z * u.z;
        if !(uu > 0.0) {
            return Vec::new();
        }
        let w = p.sub(c);
        let (half_b, cc) = ((w.rho * u.rho + w.z * u.z) / uu, (w.rho * w.rho + w.z * w.z - r * r) / uu);
        let disc = half_b * half_b - cc;
        if disc < 0.0 {
            return Vec::new();
        }
        let root = disc.sqrt();
        [-half_b - root, -half_b + root].iter().map(|t| p.add(u.scale(*t))).collect()
    };
    match (circle(a), circle(b)) {
        (None, None) => {
            let ((p, u), (q, w)) = (line(a), line(b));
            let den = u.cross(w);
            if den.abs() <= 1e-12 * u.length() * w.length() {
                let offset = if u.length() > 0.0 { q.sub(p).cross(u).abs() / u.length() } else { f64::INFINITY };
                return if offset <= near { ends_on_each_other() } else { Vec::new() };
            }
            vec![p.add(u.scale(q.sub(p).cross(w) / den))]
        }
        (None, Some(c)) => line_circle(line(a), c),
        (Some(c), None) => line_circle(line(b), c),
        (Some((c1, r1)), Some((c2, r2))) => {
            let gap = c2.sub(c1).length();
            if gap <= near && (r1 - r2).abs() <= near {
                return ends_on_each_other();
            }
            if gap <= near || gap > r1 + r2 || gap < (r1 - r2).abs() {
                return Vec::new();
            }
            let along = (r1 * r1 - r2 * r2 + gap * gap) / (2.0 * gap);
            let half = (r1 * r1 - along * along).max(0.0).sqrt();
            let unit = c2.sub(c1).scale(1.0 / gap);
            let foot = c1.add(unit.scale(along));
            let across = P2 { rho: -unit.z, z: unit.rho };
            vec![foot.add(across.scale(half)), foot.sub(across.scale(half))]
        }
    }
}

/// `piece` cut at `points` (each on it), in order along it; points within
/// `near` of an end or of each other cut once.
fn cut_at(piece: &Piece, points: &[P2], near: f64) -> Vec<Piece> {
    let fraction = |p: P2| -> f64 {
        match *piece {
            Piece::Line { start, end } => {
                let along = end.sub(start);
                let w = p.sub(start);
                (w.rho * along.rho + w.z * along.z) / (along.rho * along.rho + along.z * along.z)
            }
            Piece::Arc { center, start, end, ccw, .. } => {
                let sweep = Piece::sweep(center, start, end, ccw);
                let a0 = (start.z - center.z).atan2(start.rho - center.rho);
                let mut delta = (p.z - center.z).atan2(p.rho - center.rho) - a0;
                delta = if ccw { delta.rem_euclid(std::f64::consts::TAU) } else { -(-delta).rem_euclid(std::f64::consts::TAU) };
                delta / sweep
            }
        }
    };
    let on = |p: P2| -> P2 {
        match *piece {
            Piece::Line { .. } => p,
            Piece::Arc { center, radius, .. } => {
                let r = p.sub(center);
                center.add(r.scale(radius / r.length()))
            }
        }
    };
    let mut cuts: Vec<(f64, P2)> = points.iter().map(|p| (fraction(*p), on(*p))).filter(|(t, _)| t.is_finite()).collect();
    cuts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut parts = Vec::new();
    let mut from = piece.start();
    for (_, p) in cuts {
        if p.sub(from).length() <= near || p.sub(piece.end()).length() <= near {
            continue;
        }
        parts.push(with_end(&with_start(piece, from), p));
        from = p;
    }
    parts.push(with_start(piece, from));
    parts
}

/// The least distance between two pieces that do not cross: at an end of
/// either, or where an arc faces the other piece's carrier.
fn piece_gap(a: &Piece, b: &Piece) -> f64 {
    let mut gap = [a.distance(b.start()), a.distance(b.end()), b.distance(a.start()), b.distance(a.end())]
        .into_iter()
        .fold(f64::INFINITY, f64::min);
    for (arc, other) in [(a, b), (b, a)] {
        let Piece::Arc { center, radius, .. } = *arc else { continue };
        let toward = match *other {
            Piece::Line { start, end } => {
                let along = end.sub(start);
                let w = center.sub(start);
                let t = (w.rho * along.rho + w.z * along.z) / (along.rho * along.rho + along.z * along.z);
                start.add(along.scale(t)).sub(center)
            }
            Piece::Arc { center: other_center, .. } => other_center.sub(center),
        };
        let length = toward.length();
        if !(length > 0.0) {
            continue;
        }
        for sign in [1.0, -1.0] {
            let p = center.add(toward.scale(sign * radius / length));
            if arc.distance(p) <= 1e-12 * (1.0 + radius) {
                gap = gap.min(other.distance(p));
            }
        }
    }
    gap
}

/// `BREP_OFFSET_REVOLVED_VANISHED_ARC` (default on; `=0` restores): a concave arc grown past
/// its radius is dropped and its neighbours' offsets cut where they cross.
fn vanished_arc_on() -> bool {
    switch!("BREP_OFFSET_REVOLVED_VANISHED_ARC", true)
}


/// Where the carriers of `a` and `b` (a line or a circle each) cross, nearest
/// `hub`; `None` when they do not cross.
fn crossing_nearest(a: &Piece, b: &Piece, hub: P2) -> Option<P2> {
    let circle = |piece: &Piece| match *piece {
        Piece::Arc { center, radius, .. } => Some((center, radius)),
        Piece::Line { .. } => None,
    };
    let line = |piece: &Piece| (piece.start(), piece.end().sub(piece.start()));
    let line_circle = |(p, u): (P2, P2), (c, r): (P2, f64)| -> Vec<P2> {
        let uu = u.rho * u.rho + u.z * u.z;
        let w = p.sub(c);
        let (half_b, cc) = ((w.rho * u.rho + w.z * u.z) / uu, (w.rho * w.rho + w.z * w.z - r * r) / uu);
        let disc = half_b * half_b - cc;
        if disc < 0.0 {
            return Vec::new();
        }
        let root = disc.sqrt();
        [-half_b - root, -half_b + root].iter().map(|t| p.add(u.scale(*t))).collect()
    };
    let points: Vec<P2> = match (circle(a), circle(b)) {
        (None, None) => {
            let ((p, u), (q, w)) = (line(a), line(b));
            let den = u.cross(w);
            if den.abs() <= 1e-15 * u.length() * w.length() {
                return None;
            }
            vec![p.add(u.scale(q.sub(p).cross(w) / den))]
        }
        (None, Some(c)) => line_circle(line(a), c),
        (Some(c), None) => line_circle(line(b), c),
        (Some((c1, r1)), Some((c2, r2))) => {
            let gap = c2.sub(c1).length();
            if gap <= 1e-15 || gap > r1 + r2 || gap < (r1 - r2).abs() {
                return None;
            }
            let along = (r1 * r1 - r2 * r2 + gap * gap) / (2.0 * gap);
            let half = (r1 * r1 - along * along).max(0.0).sqrt();
            let unit = c2.sub(c1).scale(1.0 / gap);
            let foot = c1.add(unit.scale(along));
            let across = P2 { rho: -unit.z, z: unit.rho };
            vec![foot.add(across.scale(half)), foot.sub(across.scale(half))]
        }
    };
    points.into_iter().min_by(|p, q| p.sub(hub).length().total_cmp(&q.sub(hub).length()))
}

/// The pieces in boundary order: from an axis end round to the other, or
/// round a loop, consecutive pieces sharing a rim. `None` where they do not
/// form one such run.
fn order_pieces(
    pieces: &[(usize, Piece, [Option<usize>; 2])],
    rims: &[(u64, P2, [usize; 2])],
    tolerance: f64,
) -> Result<Option<Vec<Piece>>, KernelRefusal> {
    let _ = rims;
    if pieces.is_empty() {
        return Ok(None);
    }
    let close = |a: P2, b: P2| a.sub(b).length() <= 1e-7 * (1.0 + a.length()).max(tolerance);
    let mut used = vec![false; pieces.len()];
    // Start at a piece with an axis end, if any, oriented to leave the axis.
    let start_index = pieces.iter().position(|(_, piece, _)| piece.start().rho.abs() <= tolerance || piece.end().rho.abs() <= tolerance);
    let (first_index, first_piece) = match start_index {
        Some(index) => {
            let piece = pieces[index].1;
            (index, if piece.start().rho.abs() <= tolerance { piece } else { piece.reversed() })
        }
        None => (0, pieces[0].1),
    };
    used[first_index] = true;
    let mut chain = vec![first_piece];
    loop {
        let tail = chain[chain.len() - 1].end();
        if start_index.is_some() && tail.rho.abs() <= tolerance {
            break;
        }
        let next = pieces.iter().enumerate().find_map(|(index, (_, piece, _))| {
            if used[index] {
                return None;
            }
            if close(piece.start(), tail) {
                Some((index, *piece))
            } else if close(piece.end(), tail) {
                Some((index, piece.reversed()))
            } else {
                None
            }
        });
        match next {
            Some((index, piece)) => {
                used[index] = true;
                chain.push(piece);
            }
            None => break,
        }
    }
    if used.iter().any(|u| !u) {
        return Ok(None);
    }
    if start_index.is_none() && !close(chain[chain.len() - 1].end(), chain[0].start()) {
        return Ok(None);
    }
    Ok(Some(chain))
}

/// `piece` with its start moved to `start` (a point already on it).
fn with_start(piece: &Piece, start: P2) -> Piece {
    match *piece {
        Piece::Line { end, .. } => Piece::Line { start, end },
        Piece::Arc { center, radius, end, ccw, .. } => Piece::Arc { center, radius, start, end, ccw },
    }
}

/// `piece` with its end moved to `end` (a point already on it).
fn with_end(piece: &Piece, end: P2) -> Piece {
    match *piece {
        Piece::Line { start, .. } => Piece::Line { start, end },
        Piece::Arc { center, radius, start, ccw, .. } => Piece::Arc { center, radius, start, end, ccw },
    }
}

/// `piece` split where it crosses the axis ρ = 0, in order along it.
fn split_at_axis(piece: &Piece, tolerance: f64) -> Vec<Piece> {
    match *piece {
        Piece::Line { start, end } => {
            if (start.rho < -tolerance && end.rho > tolerance) || (start.rho > tolerance && end.rho < -tolerance) {
                let t = start.rho / (start.rho - end.rho);
                let cut = start.add(end.sub(start).scale(t));
                vec![Piece::Line { start, end: cut }, Piece::Line { start: cut, end }]
            } else {
                vec![*piece]
            }
        }
        Piece::Arc { center, radius, start, end, ccw } => {
            let cosine = -center.rho / radius;
            if !(cosine.abs() < 1.0) {
                return vec![*piece];
            }
            let a0 = (start.z - center.z).atan2(start.rho - center.rho);
            let sweep = Piece::sweep(center, start, end, ccw);
            let mut fractions: Vec<f64> = Vec::new();
            for crossing in [cosine.acos(), -cosine.acos()] {
                let mut delta = crossing - a0;
                if ccw {
                    delta = delta.rem_euclid(std::f64::consts::TAU);
                } else {
                    delta = -(-delta).rem_euclid(std::f64::consts::TAU);
                }
                let fraction = delta / sweep;
                if fraction > 1e-12 && fraction < 1.0 - 1e-12 {
                    fractions.push(fraction);
                }
            }
            fractions.sort_by(f64::total_cmp);
            let at = |fraction: f64| {
                let a = a0 + sweep * fraction;
                P2 { rho: center.rho + radius * a.cos(), z: center.z + radius * a.sin() }
            };
            let mut pieces = Vec::new();
            let mut from = start;
            for fraction in fractions {
                let cut = at(fraction);
                pieces.push(Piece::Arc { center, radius, start: from, end: cut, ccw });
                from = cut;
            }
            pieces.push(Piece::Arc { center, radius, start: from, end, ccw });
            pieces
        }
    }
}

