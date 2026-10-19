//! Whole-body analytic certificate (lane K2 row 1a, `BREP_SOUNDNESS_ANALYTIC_BODY`,
//! on by default, `=0` to disable): a body, or the scoped subset an acceptance hands the scan,
//! made only of PLANE and CYLINDER faces whose every face pair is proved apart
//! or proved to meet only on its shared edges, holds no crossing and no fold
//! for the full scan to confirm, so the scan returns its empty report without
//! meshing. Anything this module cannot prove declines to the full scan.
use super::*;

/// Why a body was left to the full scan.
pub(super) type Decline = &'static str;

pub(super) fn enabled() -> bool {
    // The proof reads trims through the scan's own material test; with either
    // trim switch off the scan's notion of material is the whole carrier.
    if !super::trim_samples_enabled() || !super::node_membership_enabled() {
        return false;
    }
    // On by default (promoted 2026-10-15); `BREP_SOUNDNESS_ANALYTIC_BODY=0`
    // (any value but 1/on) is the escape hatch back to the full scan.
    match std::env::var("BREP_SOUNDNESS_ANALYTIC_BODY") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    match verdict(solid, options) {
        Ok(_) => true,
        Err(reason) => {
            if std::env::var_os("BREP_DEBUG_ANALYTIC_BODY").is_some() {
                eprintln!("soundness.analytic_body decline: {reason}");
            }
            false
        }
    }
}

// What is proved, and why it covers the full scan's verdict.
//
// The scan flags a body for a confirmed CROSSING (two faces' carriers
// straddling each other by more than `straddle_band` at a point both trims
// own, along a mesh crossing longer than `min_crossing_length`) or a confirmed
// FOLD (one face carrying two distinct trimmed parameters to one point).
//
// - Carriers. A plane is a clamped bilinear 2x2 patch with unit weights whose
//   twist is below `DELTA` of its size; its chart is affine and injective. A
//   cylinder is, to `DELTA`, the full-turn surface `make_cylinder_surface`
//   rebuilds from its own net; its chart is injective on the one-turn domain.
//   Every trim control lies in the chart's domain. So no face folds.
// - Faces that share no edge: conservative world boxes (the affine image of
//   the trims' control hull; the hull's axial range of a cylinder with its full
//   radius) separated by more than `tau` on a world axis.
// - Faces that share an edge. The two carriers meet only on their common
//   curve, so the faces meet only where BOTH trims reach that curve.
//   Plane-plane: each trim's set on the common line is bounded from above:
//   every boundary point within the band of the line falls in a contact
//   interval found by subdividing each pcurve span (rational convex hull), and
//   each gap between contact intervals holds no boundary point, so the scan's
//   own trim test at its midpoint classifies all of it. The two sets'
//   intersection must lie within `tau` of the shared edges, which must lie on
//   the line. Plane-cylinder: the axis must be the plane's normal, so the
//   common curve is the circle v = v_P of the cylinder chart, and A∩B ⊆
//   B∩plane(A); the cylinder trim's set on that chart line must lie within
//   `tau` of the shared coedges. One-sided by design: it may decline a sound
//   body (a wall stepped by a corner cut), never accept an unsound one. Every
//   other kind of adjacent pair declines.
//
// Tolerances. `tau` is the scan's own weld radius, 1e-9 of the WHOLE body's
// diagonal (a scoped subset scans with the whole body's options), three orders
// under the straddle band (1e-6) and the crossing length (1e-7): a contact
// this proof leaves inside `tau` of a shared edge is below anything the scan
// can confirm. Carrier recognition is held to `DELTA` = 1e-12 relative,
// three orders under `tau` again, and the common line is refused below a
// 1e-6 angle so that `DELTA` cannot move it by more than `tau`.

/// Carrier recognition slack, relative to the carrier's size.
const DELTA: f64 = 1e-12;
/// Smallest sine between two adjacent planes this proof reads a line from.
const MIN_SINE: f64 = 1e-6;
/// Subdivision depth for a pcurve span against a line.
const MAX_DEPTH: usize = 48;

#[derive(Clone, Copy, Debug)]
pub(super) struct Plane {
    origin: Vec3,
    du: Vec3,
    dv: Vec3,
    u0: f64,
    v0: f64,
    domain: [[f64; 2]; 2],
    normal: Vec3,
    /// Smallest world length of a unit chart step (the chart's smaller
    /// singular value), so a world band over it is a chart band.
    stretch: f64,
    size: f64,
}

impl Plane {
    pub(super) fn at(&self, u: f64, v: f64) -> Vec3 {
        self.origin.add(self.du.scale(u - self.u0)).add(self.dv.scale(v - self.v0))
    }

    pub(super) fn normal(&self) -> Vec3 {
        self.normal
    }

    pub(super) fn at_origin(&self) -> Vec3 {
        self.origin
    }

    /// Is (u, v) inside the chart's own domain?
    pub(super) fn contains_parameter(&self, u: f64, v: f64) -> bool {
        let [du, dv] = self.domain;
        u >= du[0] && u <= du[1] && v >= dv[0] && v <= dv[1]
    }

    /// Chart coordinates of a world displacement (least squares on the plane).
    fn chart_vector(&self, d: Vec3) -> Option<(f64, f64)> {
        let (a, b, c) = (self.du.dot(self.du), self.du.dot(self.dv), self.dv.dot(self.dv));
        let det = a * c - b * b;
        if !(det > 0.) {
            return None;
        }
        let (p, q) = (self.du.dot(d), self.dv.dot(d));
        Some(((c * p - b * q) / det, (a * q - b * p) / det))
    }

    fn chart(&self, point: Vec3) -> Option<(f64, f64)> {
        let (u, v) = self.chart_vector(point.sub(self.origin))?;
        Some((u + self.u0, v + self.v0))
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) struct Cylinder {
    base: Vec3,
    axis: Vec3,
    radius: f64,
    height: f64,
    u: [f64; 2],
    v: [f64; 2],
}

impl Cylinder {
    /// `(base, unit axis, radius)`.
    pub(super) fn frame(&self) -> (Vec3, Vec3, f64) {
        (self.base, self.axis, self.radius)
    }

    /// Chart v of the plane through `point` normal to the axis.
    fn v_at(&self, point: Vec3) -> f64 {
        self.v[0] + self.axis.dot(point.sub(self.base)) / self.height * (self.v[1] - self.v[0])
    }
}

#[derive(Clone, Copy, Debug)]
pub(super) enum Carrier {
    Plane(Plane),
    Cylinder(Cylinder),
}

fn finite(p: Vec3) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

fn clamped(knots: &[f64]) -> Option<[f64; 2]> {
    let [a, b] = [*knots.first()?, *knots.last()?];
    (knots.len() == 4 && knots[1] == a && knots[2] == b && a.is_finite() && b.is_finite() && b > a).then_some([a, b])
}

fn plane_of(s: &NurbsSurface) -> Option<Plane> {
    if s.degree_u != 1 || s.degree_v != 1 || s.control_points.len() != 2 || s.control_points.iter().any(|r| r.len() != 2) {
        return None;
    }
    if s.control_points.iter().flatten().any(|c| c.w != 1.0) {
        return None;
    }
    let [ku, kv] = [clamped(&s.knots_u)?, clamped(&s.knots_v)?];
    let p = |i: usize, j: usize| {
        let c = s.control_points[i][j];
        Vec3::new(c.x, c.y, c.z)
    };
    let (p00, p01, p10, p11) = (p(0, 0), p(0, 1), p(1, 0), p(1, 1));
    if ![p00, p01, p10, p11].into_iter().all(finite) {
        return None;
    }
    let size = [p00, p01, p10, p11].iter().map(|q| q.x.abs().max(q.y.abs()).max(q.z.abs())).fold(1.0, f64::max);
    let twist = p11.sub(p10).sub(p01).add(p00);
    if twist.length() > DELTA * size {
        return None;
    }
    let du = p10.sub(p00).scale(1. / (ku[1] - ku[0]));
    let dv = p01.sub(p00).scale(1. / (kv[1] - kv[0]));
    let normal = du.cross(dv).normalized().ok()?;
    // The chart's smaller singular value.
    let (a, b, c) = (du.dot(du), du.dot(dv), dv.dot(dv));
    let mean = 0.5 * (a + c);
    let spread = (0.25 * (a - c) * (a - c) + b * b).sqrt();
    let stretch = (mean - spread).max(0.).sqrt();
    if !(stretch > 0.) {
        return None;
    }
    Some(Plane { origin: p00, du, dv, u0: ku[0], v0: kv[0], domain: [ku, kv], normal, stretch, size })
}

fn cylinder_of(s: &NurbsSurface) -> Option<Cylinder> {
    if s.degree_u != 2 || s.degree_v != 1 || s.control_points.len() != 9 || s.control_points.iter().any(|r| r.len() != 2) {
        return None;
    }
    let c0 = s.control_points[0][0].point().ok()?;
    let c1 = s.control_points[0][1].point().ok()?;
    let opposite = s.control_points[4][0].point().ok()?;
    let along = c1.sub(c0);
    let height = along.length();
    let axis = along.normalized().ok()?;
    let base = c0.add(opposite).scale(0.5);
    let radius = c0.sub(base).length();
    if ![height, radius].iter().all(|v| v.is_finite() && *v > 0.) || !finite(base) {
        return None;
    }
    let rebuilt = crate::make_cylinder_surface(base, axis, radius, height).ok()?;
    let same_bits = |a: &[f64], b: &[f64]| a.len() == b.len() && a.iter().zip(b).all(|(x, y)| x.to_bits() == y.to_bits());
    if !same_bits(&s.knots_u, &rebuilt.knots_u) || !same_bits(&s.knots_v, &rebuilt.knots_v) {
        return None;
    }
    let size = base.x.abs().max(base.y.abs()).max(base.z.abs()) + radius + height;
    let close = s.control_points.iter().flatten().zip(rebuilt.control_points.iter().flatten()).all(|(a, b)| {
        (a.x - b.x).abs() <= DELTA * size
            && (a.y - b.y).abs() <= DELTA * size
            && (a.z - b.z).abs() <= DELTA * size
            && (a.w - b.w).abs() <= DELTA
            && a.w > 0.
    });
    if !close || s.control_points.len() != rebuilt.control_points.len() {
        return None;
    }
    let u = [*s.knots_u.first()?, *s.knots_u.last()?];
    let v = [*s.knots_v.first()?, *s.knots_v.last()?];
    Some(Cylinder { base, axis, radius, height, u, v })
}

pub(super) fn carrier_of(s: &NurbsSurface) -> Option<Carrier> {
    plane_of(s).map(Carrier::Plane).or_else(|| cylinder_of(s).map(Carrier::Cylinder))
}

/// One pcurve span in Bernstein form, homogeneous `(X, Y, W)`.
type Span = Vec<[f64; 3]>;

fn bezier_spans(curve: &NurbsCurve) -> Option<Vec<Span>> {
    let p = curve.degree;
    if p == 0
        || curve.control_points.iter().any(|c| !(c.w > 0.) || !c.w.is_finite() || !c.x.is_finite() || !c.y.is_finite())
    {
        return None;
    }
    let knots = &curve.knots;
    let (first, last) = (*knots.first()?, *knots.last()?);
    if knots.len() < 2 * (p + 1) || knots[..=p].iter().any(|k| *k != first) || knots[knots.len() - p - 1..].iter().any(|k| *k != last) {
        return None;
    }
    let mut interior: Vec<f64> = knots.iter().copied().filter(|k| *k > first && *k < last).collect();
    interior.dedup();
    let mut split = curve.clone();
    for &knot in &interior {
        split = split.insert_knot(knot, p).ok()?;
    }
    if split.control_points.len() != p * (interior.len() + 1) + 1 {
        return None;
    }
    Some(
        (0..=interior.len())
            .map(|i| split.control_points[i * p..=i * p + p].iter().map(|c| [c.x, c.y, c.w]).collect())
            .collect(),
    )
}

/// The line `{q : normal·q = offset}` of a chart, read at `along·q`
/// (`normal`, `along` orthonormal).
#[derive(Clone, Copy)]
struct ChartLine {
    normal: [f64; 2],
    offset: f64,
    along: [f64; 2],
    /// Distance from the line within which a boundary point counts as on it.
    band: f64,
}

impl ChartLine {
    fn point(&self, s: f64) -> Vec2 {
        Vec2 {
            x: self.normal[0] * self.offset + self.along[0] * s,
            y: self.normal[1] * self.offset + self.along[1] * s,
        }
    }
}

/// Every `along` reading of a span point within `line.band` of the line lies
/// in one of the intervals pushed (rational convex hull, by subdivision).
fn contacts(span: &[[f64; 3]], line: &ChartLine, depth: usize, out: &mut Vec<[f64; 2]>) {
    let distance = |q: &[f64; 3]| (line.normal[0] * q[0] + line.normal[1] * q[1]) / q[2] - line.offset;
    let reading = |q: &[f64; 3]| (line.along[0] * q[0] + line.along[1] * q[1]) / q[2];
    let d: Vec<f64> = span.iter().map(distance).collect();
    if d.iter().all(|x| *x > line.band) || d.iter().all(|x| *x < -line.band) {
        return;
    }
    let (lo, hi) = span.iter().map(reading).fold((f64::INFINITY, f64::NEG_INFINITY), |(a, b), s| (a.min(s), b.max(s)));
    if d.iter().all(|x| x.abs() <= line.band) || depth >= MAX_DEPTH || hi - lo <= line.band {
        out.push([lo, hi]);
        return;
    }
    // de Casteljau at the middle, in homogeneous coordinates.
    let mut rows = vec![span.to_vec()];
    while rows.last().unwrap().len() > 1 {
        let prev = rows.last().unwrap();
        rows.push(prev.windows(2).map(|w| [0.5 * (w[0][0] + w[1][0]), 0.5 * (w[0][1] + w[1][1]), 0.5 * (w[0][2] + w[1][2])]).collect());
    }
    let left: Span = rows.iter().map(|r| r[0]).collect();
    let right: Span = rows.iter().rev().map(|r| *r.last().unwrap()).collect();
    contacts(&left, line, depth + 1, out);
    contacts(&right, line, depth + 1, out);
}

fn merged(mut intervals: Vec<[f64; 2]>, join: f64) -> Vec<[f64; 2]> {
    intervals.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let mut out: Vec<[f64; 2]> = Vec::new();
    for i in intervals {
        match out.last_mut() {
            Some(last) if i[0] <= last[1] + join => last[1] = last[1].max(i[1]),
            _ => out.push(i),
        }
    }
    out
}

fn intersection(a: &[[f64; 2]], b: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    for x in a {
        for y in b {
            let (lo, hi) = (x[0].max(y[0]), x[1].min(y[1]));
            if lo <= hi {
                out.push([lo, hi]);
            }
        }
    }
    out
}

fn within(set: &[[f64; 2]], cover: &[[f64; 2]], slack: f64) -> bool {
    set.iter().all(|i| cover.iter().any(|c| i[0] >= c[0] - slack && i[1] <= c[1] + slack))
}

/// One face, recognised, with its trims cut into Bernstein spans.
pub(super) struct Face<'a> {
    record: &'a FaceRecord,
    carrier: Carrier,
    spans: Vec<Span>,
    /// Control hull of every trim, `[u_min, u_max, v_min, v_max]`.
    hull: [f64; 4],
}

impl<'a> Face<'a> {
    pub(super) fn new(record: &'a FaceRecord, carrier: Carrier, tau: f64) -> Result<Self, Decline> {
        let mut spans = Vec::new();
        for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
            spans.extend(bezier_spans(&coedge.pcurve).ok_or("a trim is not a clamped rational curve with positive weights")?);
        }
        if spans.is_empty() {
            return Err("a face has no trim");
        }
        let mut hull = [f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY];
        for q in spans.iter().flatten() {
            let (u, v) = (q[0] / q[2], q[1] / q[2]);
            hull = [hull[0].min(u), hull[1].max(u), hull[2].min(v), hull[3].max(v)];
        }
        let (domain, slack) = match carrier {
            Carrier::Plane(p) => (p.domain, [tau / p.stretch; 2]),
            Carrier::Cylinder(c) => {
                (
                    [c.u, c.v],
                    [tau / (8. * c.radius) * (c.u[1] - c.u[0]), tau / c.height * (c.v[1] - c.v[0])],
                )
            }
        };
        let inside = hull[0] >= domain[0][0] - slack[0]
            && hull[1] <= domain[0][1] + slack[0]
            && hull[2] >= domain[1][0] - slack[1]
            && hull[3] <= domain[1][1] + slack[1];
        if !inside {
            return Err("a trim leaves its carrier's chart");
        }
        Ok(Self { record, carrier, spans, hull })
    }

    /// The face's set on a chart line, from above: contact intervals plus the
    /// gaps the scan's trim test puts inside.
    fn on_line(&self, line: &ChartLine, tiny: f64) -> Result<Vec<[f64; 2]>, Decline> {
        let mut found = Vec::new();
        for span in &self.spans {
            contacts(span, line, 0, &mut found);
        }
        let found = merged(found, 0.);
        let mut set = found.clone();
        for pair in found.windows(2) {
            let (a, b) = (pair[0][1], pair[1][0]);
            if b - a <= tiny {
                set.push([a, b]);
                continue;
            }
            match parameter_point_in_face(self.record, line.point(0.5 * (a + b)), TRIM_SAMPLE_TOLERANCE) {
                Ok(PolygonClass::Inside) => set.push([a, b]),
                Ok(PolygonClass::Outside) => {}
                Ok(PolygonClass::Boundary) => return Err("a gap between trim contacts reads as boundary"),
                Err(_) => return Err("the trim test could not classify a gap"),
            }
        }
        Ok(merged(set, 0.))
    }

    fn world_box(&self) -> (Vec3, Vec3) {
        let [u0, u1, v0, v1] = self.hull;
        match self.carrier {
            Carrier::Plane(p) => {
                let corners = [p.at(u0, v0), p.at(u1, v0), p.at(u0, v1), p.at(u1, v1)];
                let pad = DELTA * p.size;
                let lo = corners.iter().fold(Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY), |m, q| Vec3::new(m.x.min(q.x), m.y.min(q.y), m.z.min(q.z)));
                let hi = corners.iter().fold(Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY), |m, q| Vec3::new(m.x.max(q.x), m.y.max(q.y), m.z.max(q.z)));
                (lo.sub(Vec3::new(pad, pad, pad)), hi.add(Vec3::new(pad, pad, pad)))
            }
            Carrier::Cylinder(c) => {
                let z = |v: f64| (v - c.v[0]) / (c.v[1] - c.v[0]) * c.height;
                let (a, b) = (c.base.add(c.axis.scale(z(v0))), c.base.add(c.axis.scale(z(v1))));
                let size = c.base.x.abs().max(c.base.y.abs()).max(c.base.z.abs()) + c.radius + c.height;
                let reach = |k: f64| c.radius * (1. - k * k).max(0.).sqrt() + DELTA * size;
                let r = Vec3::new(reach(c.axis.x), reach(c.axis.y), reach(c.axis.z));
                (
                    Vec3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z)).sub(r),
                    Vec3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z)).add(r),
                )
            }
        }
    }
}

fn separated(a: (Vec3, Vec3), b: (Vec3, Vec3), tau: f64) -> bool {
    let (alo, ahi, blo, bhi) = ([a.0.x, a.0.y, a.0.z], [a.1.x, a.1.y, a.1.z], [b.0.x, b.0.y, b.0.z], [b.1.x, b.1.y, b.1.z]);
    (0..3).any(|i| alo[i] > bhi[i] + tau || blo[i] > ahi[i] + tau)
}

/// The cylinder side of a plane-cylinder pair: the cylinder trim's set on its
/// chart line `v = v_p` lies within `tau` of the `shared` edges' coedges,
/// which lie on that line.
pub(super) fn cylinder_side(face: &Face, c: &Cylinder, v_p: f64, shared: &[u64], tau: f64) -> Result<(), Decline> {
    let band_v = tau / c.height * (c.v[1] - c.v[0]);
    let band_u = tau / (8. * c.radius) * (c.u[1] - c.u[0]);
    let line = ChartLine { normal: [0., 1.], offset: v_p, along: [1., 0.], band: band_v };
    let set = face.on_line(&line, 4. * band_u)?;
    let mut cover = Vec::new();
    for coedge in face.record.loops.iter().flat_map(|l| &l.coedges).filter(|ce| shared.contains(&ce.edge_id)) {
        let spans = bezier_spans(&coedge.pcurve).ok_or("a shared trim is not a clamped rational curve")?;
        if spans.iter().flatten().any(|q| (q[1] / q[2] - v_p).abs() > band_v) {
            return Err("a shared coedge leaves the cylinder's rim line");
        }
        let [t0, t1] = coedge.pcurve.domain().map_err(|_| "a shared trim has no domain")?;
        let (a, b) = (
            coedge.pcurve.evaluate(t0).map_err(|_| "a shared trim does not evaluate")?.x,
            coedge.pcurve.evaluate(t1).map_err(|_| "a shared trim does not evaluate")?.x,
        );
        let (lo, hi) = (a.min(b), a.max(b));
        if spans.iter().flatten().any(|q| q[0] / q[2] < lo - band_u || q[0] / q[2] > hi + band_u) {
            return Err("a shared coedge's hull bulges past its own ends");
        }
        cover.push([lo, hi]);
    }
    if within(&set, &merged(cover, band_u), band_u) {
        Ok(())
    } else {
        Err("the plane meets the cylinder's trim off the shared edges")
    }
}

fn plane_side(face: &Face, p: &Plane, origin: Vec3, direction: Vec3, tau: f64) -> Result<Vec<[f64; 2]>, Decline> {
    let (u, v) = p.chart(origin).ok_or("a plane chart is singular")?;
    let (du, dv) = p.chart_vector(direction).ok_or("a plane chart is singular")?;
    let speed = (du * du + dv * dv).sqrt();
    if !(speed > 0.) {
        return Err("the common line has no chart direction");
    }
    let along = [du / speed, dv / speed];
    let normal = [-along[1], along[0]];
    let line = ChartLine { normal, offset: normal[0] * u + normal[1] * v, along, band: tau / p.stretch };
    let s0 = along[0] * u + along[1] * v;
    let set = face.on_line(&line, 4. * tau / p.stretch)?;
    // World parameter t along the common line: s = s0 + t * speed.
    Ok(set.into_iter().map(|[a, b]| [(a - s0) / speed, (b - s0) / speed]).collect())
}

fn plane_pair(
    a: &Face,
    pa: &Plane,
    b: &Face,
    pb: &Plane,
    shared: &[u64],
    solid: &BrepSolid,
    tau: f64,
) -> Result<(), Decline> {
    let cross = pa.normal.cross(pb.normal);
    let sine = cross.length();
    if !(sine >= MIN_SINE) || DELTA * pa.size.max(pb.size) / sine > tau {
        return Err("adjacent planes are too close to parallel to read their common line");
    }
    let direction = cross.scale(1. / sine);
    // The point of the common line nearest the origin.
    let (da, db) = (pa.normal.dot(pa.origin), pb.normal.dot(pb.origin));
    let denominator = pa.normal.dot(pb.normal.cross(direction));
    let origin = pb.normal.cross(direction).scale(da).add(direction.cross(pa.normal).scale(db)).scale(1. / denominator);
    if !finite(origin) {
        return Err("the common line has no finite point");
    }
    let meet = intersection(&plane_side(a, pa, origin, direction, tau)?, &plane_side(b, pb, origin, direction, tau)?);
    let off_line = |q: Vec3| q.sub(origin).sub(direction.scale(q.sub(origin).dot(direction))).length();
    let mut cover = Vec::new();
    for &id in shared {
        let edge = solid.edges.iter().find(|e| e.id == id).ok_or("a shared edge has no record")?;
        let vertex = |vid: u64| solid.vertices.iter().find(|v| v.id == vid).map(|v| v.point).ok_or("a shared edge's vertex has no record");
        let (start, end) = (vertex(edge.start_vertex_id)?, vertex(edge.end_vertex_id)?);
        let ends = [edge.curve.evaluate(edge.t0), edge.curve.evaluate(edge.t1)];
        let [Ok(e0), Ok(e1)] = ends else { return Err("a shared edge does not evaluate") };
        if e0.sub(start).length() > tau || e1.sub(end).length() > tau {
            return Err("a shared edge's curve leaves its vertices");
        }
        if edge.curve.control_points.iter().any(|c| c.point().map_or(true, |q| off_line(q) > tau)) || off_line(start) > tau || off_line(end) > tau {
            return Err("a shared edge leaves the planes' common line");
        }
        let (t0, t1) = (start.sub(origin).dot(direction), end.sub(origin).dot(direction));
        cover.push([t0.min(t1), t0.max(t1)]);
    }
    if within(&meet, &merged(cover, tau), tau) {
        Ok(())
    } else {
        Err("the carriers' common line meets both trims off the shared edges")
    }
}

/// `Ok(adjacent pairs proved)` or the first reason the proof stops.
pub(super) fn verdict(solid: &BrepSolid, options: SelfIntersectionOptions) -> Result<usize, Decline> {
    verdict_with(solid, options, false)
}

/// The same proof with NURBS patch faces admitted (`blend_patch.rs`).
pub(super) fn verdict_with_patches(solid: &BrepSolid, options: SelfIntersectionOptions) -> Result<usize, Decline> {
    verdict_with(solid, options, true)
}

/// One face of the proof: a native carrier, or a patch proved fold-free.
enum Item<'a> {
    Native(Face<'a>),
    Patch(&'a FaceRecord, super::blend_patch::Patch),
}

impl Item<'_> {
    fn record(&self) -> &FaceRecord {
        match self {
            Item::Native(face) => face.record,
            Item::Patch(record, _) => record,
        }
    }

    fn world_box(&self) -> (Vec3, Vec3) {
        match self {
            Item::Native(face) => face.world_box(),
            Item::Patch(_, patch) => (patch.lo, patch.hi),
        }
    }
}

/// Two faces sharing no edge whose boxes overlap, apart cell by cell
/// (blend_curved.rs): every cell of a patch box-separated from the other face
/// or from each of its cells, or, against a plane, more than `tau` on one side.
fn cells_apart(a: &Item, box_a: (Vec3, Vec3), b: &Item, box_b: (Vec3, Vec3), tau: f64) -> bool {
    match (a, b) {
        (Item::Native(_), Item::Native(_)) => false,
        (Item::Patch(_, patch), Item::Native(face)) | (Item::Native(face), Item::Patch(_, patch)) => {
            let face_box = if matches!(a, Item::Native(_)) { box_a } else { box_b };
            patch.nets.iter().all(|net| {
                let cell = super::blend_curved::net_box(net);
                separated(cell, face_box, tau)
                    || match face.carrier {
                        Carrier::Plane(plane) => {
                            let (normal, level) = (plane.normal, plane.normal.dot(plane.origin));
                            let (lo, hi) = net.iter().flatten().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), h| {
                                let g = normal.dot(Vec3::new(h[0] / h[3], h[1] / h[3], h[2] / h[3])) - level;
                                (lo.min(g), hi.max(g))
                            });
                            lo > tau || hi < -tau
                        }
                        Carrier::Cylinder(_) => false,
                    }
            })
        }
        (Item::Patch(_, pa), Item::Patch(_, pb)) => {
            let near_b: Vec<(Vec3, Vec3)> = pb.nets.iter().map(super::blend_curved::net_box).filter(|cell| !separated(*cell, box_a, tau)).collect();
            pa.nets.iter().all(|net| {
                let cell = super::blend_curved::net_box(net);
                separated(cell, box_b, tau) || near_b.iter().all(|other| separated(cell, *other, tau))
            })
        }
    }
}

fn verdict_with(solid: &BrepSolid, options: SelfIntersectionOptions, patches: bool) -> Result<usize, Decline> {
    let tau = options.weld;
    if !(tau > 0.) || !tau.is_finite() {
        return Err("no weld radius to bound the proof");
    }
    let records: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    if records.is_empty() || records.len() > 256 {
        return Err("face count outside 1..=256");
    }
    // Carriers and adjacency first, no trims or cells: a body whose adjacency
    // kinds this proof does not cover declines before any face is built.
    let mut carriers = Vec::with_capacity(records.len());
    for record in &records {
        let carrier = carrier_of(&record.surface);
        if carrier.is_none() && !patches {
            return Err("a carrier is not a native plane or cylinder");
        }
        carriers.push(carrier);
    }
    // Which faces use each edge.
    let mut users: HashMap<u64, Vec<usize>> = HashMap::default();
    for (index, record) in records.iter().enumerate() {
        for coedge in record.loops.iter().flat_map(|l| &l.coedges) {
            users.entry(coedge.edge_id).or_default().push(index);
        }
    }
    let mut shared: HashMap<(usize, usize), Vec<u64>> = HashMap::default();
    for (&edge, faces_using) in &users {
        if faces_using.len() > 2 {
            return Err("an edge has more than two uses");
        }
        if solid.edges.iter().any(|e| e.id == edge && e.degenerate) {
            return Err("a degenerate edge");
        }
        if let [i, j] = faces_using[..] {
            if i != j {
                shared.entry((i.min(j), i.max(j))).or_default().push(edge);
            }
        }
    }
    let curved = patches && super::blend_curved::enabled();
    // A patch on a recognised torus (blend_curved.rs), when that is on.
    let tori: Vec<Option<super::blend_curved::Torus>> = records
        .iter()
        .zip(&carriers)
        .map(|(record, carrier)| if curved && carrier.is_none() { super::blend_curved::torus_of(&record.surface) } else { None })
        .collect();
    let mut adjacent: Vec<(usize, usize)> = shared.keys().copied().collect();
    adjacent.sort_unstable();
    for (i, j) in adjacent {
        match (carriers[i], carriers[j]) {
            (None, None) => {
                if tori[i].is_none() && tori[j].is_none() {
                    return Err("two patches meet");
                }
            }
            (None, Some(carrier)) | (Some(carrier), None) => {
                if !matches!(carrier, Carrier::Plane(_)) && !(curved && matches!(carrier, Carrier::Cylinder(_))) {
                    return Err("a patch meets a face this proof does not cover");
                }
            }
            (Some(Carrier::Plane(_)), Some(Carrier::Plane(_))) => {}
            (Some(Carrier::Plane(p)), Some(Carrier::Cylinder(c))) | (Some(Carrier::Cylinder(c)), Some(Carrier::Plane(p))) => {
                if p.normal.cross(c.axis).length() > DELTA {
                    return Err("a plane meets a cylinder off its axis");
                }
            }
            _ => return Err("an adjacent pair of kinds this proof does not cover"),
        }
    }
    // Patches first: their cells cost little next to a native face's trims,
    // and each check a patch takes against a face it shares an edge with
    // reads only the patch's cells and that face's carrier. A patch that
    // declines does so before any native face is built.
    let mut built: Vec<Option<super::blend_patch::Patch>> = Vec::with_capacity(records.len());
    for (record, carrier) in records.iter().zip(&carriers) {
        built.push(if carrier.is_none() { Some(super::blend_patch::patch(record, tau)?) } else { None });
    }
    let mut proved = 0;
    let mut adjacent: Vec<(usize, usize)> = shared.keys().copied().collect();
    adjacent.sort_unstable();
    for &(i, j) in &adjacent {
        match (&built[i], &built[j]) {
            (None, None) => {}
            (Some(patch), None) | (None, Some(patch)) => {
                let other = if built[i].is_some() { j } else { i };
                match carriers[other] {
                    Some(Carrier::Plane(plane)) => {
                        if !super::blend_patch::one_sided(&patch.points, &plane, tau) {
                            return Err("a patch reaches both sides of a plane it meets");
                        }
                    }
                    Some(Carrier::Cylinder(c)) if curved => {
                        let record = records[if built[i].is_some() { i } else { j }];
                        if !super::blend_curved::one_sided_patch(record, &patch.nets, &super::blend_curved::Implicit::cylinder(&c), tau) {
                            return Err("a patch reaches both sides of a cylinder it meets");
                        }
                    }
                    _ => return Err("a patch meets a face this proof does not cover"),
                }
                proved += 1;
            }
            (Some(pa), Some(pb)) => {
                // One of the two on the other's torus is enough.
                let on = |patch: &super::blend_patch::Patch, torus: Option<super::blend_curved::Torus>| {
                    torus.is_some_and(|t| super::blend_curved::one_sided(&patch.nets, &super::blend_curved::Implicit::Torus(t), tau))
                };
                if !(on(pa, tori[j]) || on(pb, tori[i])) {
                    return Err("a patch reaches both sides of a torus it meets");
                }
                proved += 1;
            }
        }
    }
    let mut faces = Vec::with_capacity(records.len());
    for ((record, carrier), patch) in records.iter().zip(carriers).zip(built) {
        match (carrier, patch) {
            (_, Some(patch)) => faces.push(Item::Patch(record, patch)),
            (Some(carrier), None) => faces.push(Item::Native(Face::new(record, carrier, tau)?)),
            (None, None) => unreachable!("every face without a native carrier was built as a patch"),
        }
    }
    let boxes: Vec<_> = faces.iter().map(Item::world_box).collect();
    for i in 0..faces.len() {
        for j in i + 1..faces.len() {
            let Some(edges) = shared.get(&(i, j)) else {
                if separated(boxes[i], boxes[j], tau) || (curved && cells_apart(&faces[i], boxes[i], &faces[j], boxes[j], tau)) {
                    continue;
                }
                return Err("two faces that share no edge are not separated");
            };
            let (a, b) = match (&faces[i], &faces[j]) {
                (Item::Native(a), Item::Native(b)) => (a, b),
                // Proved above, with the patches.
                _ => continue,
            };
            match (&a.carrier, &b.carrier) {
                (Carrier::Plane(pa), Carrier::Plane(pb)) => plane_pair(a, pa, b, pb, edges, solid, tau)?,
                (Carrier::Plane(p), Carrier::Cylinder(c)) | (Carrier::Cylinder(c), Carrier::Plane(p)) => {
                    let wall = if matches!(a.carrier, Carrier::Cylinder(_)) { a } else { b };
                    if p.normal.cross(c.axis).length() > DELTA {
                        return Err("a plane meets a cylinder off its axis");
                    }
                    cylinder_side(wall, c, c.v_at(p.origin), edges, tau)?;
                }
                _ => return Err("an adjacent pair of kinds this proof does not cover"),
            }
            proved += 1;
        }
    }
    Ok(proved)
}

