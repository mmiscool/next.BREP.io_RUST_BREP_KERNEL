//! The OUTWARD wall of a planar opening: the opening's plane trimmed by its
//! outer outline OFFSET outward by the pad, in the plane.
//!
//! An outward shell holds each opening face in place and extends it until it
//! meets the grown offsets of the retained faces around it. The wall used to be
//! built by padding the plane's 2 × 2 net under cloned pcurves, which SCALES the
//! outline about the patch centre. That is the offset outline only for a
//! rectangle filling its patch, or a circle centred on it. On an L-shaped
//! opening the two notch edges pass through the centre and do not move at all,
//! so the notch faces' grown offsets cross the plane outside the wall and the
//! shell reads "completion produced non-integral genus"
//! (`l-step-open-bottom`, d = −1.5, in the membrane census).
//!
//! Here each outline element is offset along its own outward normal: a line to
//! the parallel line, a circular arc to the concentric arc at `r ± pad`. Two
//! neighbours meet where their offsets intersect, nearest the vertex's own
//! offset, which mitres a convex corner and trims a reflex one; at a tangent
//! junction the offsets already meet at `v + n·pad`. Every edge and pcurve is
//! exact: a line or arc in 3D, and its image under the plane's affine
//! parameterization in uv, which is exact for a rational curve because an
//! affine map commutes with the rational combination.
//!
//! An element that is neither a line nor a circular arc (an ellipse, a spline)
//! is offset too, since 2026-09-26, but its offset has no exact spline form,
//! so it is FITTED against the exact `p + pad·n` (`fit_free_offset`). At a
//! tangent junction the two offsets meet at the same point. At a CORNER the
//! crossing is solved on the offsets themselves, the free-form one read past
//! its end on the curve's analytic continuation (`NurbsCurve::extend_natural`,
//! the construction the carrier's continuation uses) for a convex mitre, and
//! trimmed short of it at a reflex corner.
//!
//! What this declines, with the reason: a concave arc whose offset radius
//! reaches zero, or a concave free-form stretch tighter than the pad; a corner
//! whose crossing the solve does not reach; a fit that misses its bar at the
//! station cap; an offset element that reverses or collapses; a junction the
//! two offsets never reach; and an offset outline that crosses itself.

use crate::{KernelRefusal, KernelStage, OrRefuse};
use super::*;
use crate::{make_arc, make_line, NurbsCurve, NurbsSurface, Vec4};
use std::f64::consts::{PI, TAU};

type P2 = [f64; 2];

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}
fn add(a: P2, b: P2) -> P2 {
    [a[0] + b[0], a[1] + b[1]]
}
fn scale(a: P2, k: f64) -> P2 {
    [a[0] * k, a[1] * k]
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
fn unit(a: P2) -> P2 {
    scale(a, 1.0 / length(a))
}

/// A free-form outline edge in the plane's frame: its curve with every
/// control point mapped by the frame's affine `flat` (exact, weights kept),
/// traversed from parameter `a` to `b` — `b < a` for a reversed coedge.
#[derive(Clone, Debug)]
struct FreeCurve {
    curve: NurbsCurve,
    a: f64,
    b: f64,
    /// The same curve continued past both ends by one span of `a..b`
    /// (`NurbsCurve::extend_natural`, the construction the carrier's own
    /// `NurbsSurface::extend_natural` uses), so a fraction outside `[0, 1]`
    /// reads the analytic continuation a sharp corner's mitre lies on. `None`
    /// where the continuation is refused.
    continued: Option<NurbsCurve>,
}

impl FreeCurve {
    fn new(curve: NurbsCurve, a: f64, b: f64) -> Self {
        let span = (b - a).abs();
        let continued = curve
            .extend_natural(true, span)
            .and_then(|low| low.extend_natural(false, span))
            .ok();
        Self { curve, a, b, continued }
    }
    fn parameter(&self, fraction: f64) -> f64 {
        self.a + (self.b - self.a) * fraction
    }
    /// The curve that reads `fraction`: the original inside `[0, 1]`, the
    /// continuation within one span outside it.
    fn reading(&self, fraction: f64) -> Result<&NurbsCurve, KernelRefusal> {
        if (0.0..=1.0).contains(&fraction) {
            return Ok(&self.curve);
        }
        match &self.continued {
            Some(continued) if (-1.0..=2.0).contains(&fraction) => Ok(continued),
            _ => Err(KernelRefusal::internal(KernelStage::Refine, "offset_shell_free_form_continuation", format!(
                "offset_shell: a free-form outline edge has no continuation at fraction {fraction:.3}"
            ))),
        }
    }
    fn point(&self, fraction: f64) -> Result<P2, KernelRefusal> {
        let point = self.reading(fraction)?.evaluate(self.parameter(fraction)).or_refuse(KernelStage::Refine, "evaluate")?;
        Ok([point.x, point.y])
    }
    /// The unit tangent and the signed curvature, both in the TRAVERSAL sense
    /// (positive curvature turns counter-clockwise in the frame).
    fn tangent_and_curvature(&self, fraction: f64) -> Result<(P2, f64), KernelRefusal> {
        let d = self.reading(fraction)?.derivatives(self.parameter(fraction), 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let sense = (self.b - self.a).signum();
        let first = [d[1].x * sense, d[1].y * sense];
        let second = [d[2].x, d[2].y];
        let speed = length(first);
        if speed <= 1e-300 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_free_form_stationary", "offset_shell: a free-form outline edge is stationary"));
        }
        Ok((scale(first, 1.0 / speed), cross(first, second) / (speed * speed * speed)))
    }
}

/// One outline element in the plane's orthonormal frame, in traversal order.
#[derive(Clone, Debug)]
enum Element {
    Line { start: P2, end: P2 },
    /// `sweep` is the signed angle travelled from `start` to `end`: positive
    /// counter-clockwise in the frame. A whole circle is ±2π.
    Arc { centre: P2, radius: f64, start: P2, end: P2, sweep: f64 },
    /// Neither a line nor a circular arc: an ellipse, a spline.
    Free { base: std::rc::Rc<FreeCurve>, start: P2, end: P2 },
    /// A free-form element's offset, FITTED: a non-rational curve in the frame
    /// (z = 0) whose domain runs `start` → `end`.
    Fitted { curve: NurbsCurve, start: P2, end: P2 },
}

impl Element {
    fn start(&self) -> P2 {
        match self {
            Element::Line { start, .. }
            | Element::Arc { start, .. }
            | Element::Free { start, .. }
            | Element::Fitted { start, .. } => *start,
        }
    }
    fn end(&self) -> P2 {
        match self {
            Element::Line { end, .. }
            | Element::Arc { end, .. }
            | Element::Free { end, .. }
            | Element::Fitted { end, .. } => *end,
        }
    }
    /// The unit tangent at `point`, a point of a line or an arc. A free-form
    /// element answers at its two ends only (`start_tangent`, `end_tangent`).
    fn tangent_at(&self, point: P2) -> P2 {
        match self {
            Element::Line { start, end } => unit(sub(*end, *start)),
            Element::Free { .. } | Element::Fitted { .. } => {
                unreachable!("a free-form element's tangent is read at its ends")
            }
            Element::Arc { centre, sweep, .. } => {
                let radial = unit(sub(point, *centre));
                if *sweep > 0.0 {
                    [-radial[1], radial[0]]
                } else {
                    [radial[1], -radial[0]]
                }
            }
        }
    }
    fn start_tangent(&self) -> Result<P2, KernelRefusal> {
        match self {
            Element::Free { base, .. } => Ok(base.tangent_and_curvature(0.0)?.0),
            _ => Ok(self.tangent_at(self.start())),
        }
    }
    fn end_tangent(&self) -> Result<P2, KernelRefusal> {
        match self {
            Element::Free { base, .. } => Ok(base.tangent_and_curvature(1.0)?.0),
            _ => Ok(self.tangent_at(self.end())),
        }
    }
    fn sample(&self, fraction: f64) -> P2 {
        match self {
            Element::Line { start, end } => add(*start, scale(sub(*end, *start), fraction)),
            Element::Free { base, start, .. } => base.point(fraction).unwrap_or(*start),
            Element::Fitted { curve, start, .. } => curve
                .domain()
                .and_then(|[t0, t1]| curve.evaluate(t0 + (t1 - t0) * fraction))
                .map(|point| [point.x, point.y])
                .unwrap_or(*start),
            Element::Arc { centre, radius, start, sweep, .. } => {
                let angle = (start[1] - centre[1]).atan2(start[0] - centre[0]) + sweep * fraction;
                [centre[0] + radius * angle.cos(), centre[1] + radius * angle.sin()]
            }
        }
    }
}

/// The plane's frame: an origin, two orthonormal in-plane axes, and the affine
/// map back to the opening surface's own (u, v).
struct PlaneFrame {
    origin: Vec3,
    e1: Vec3,
    e2: Vec3,
    axis_u: Vec3,
    axis_v: Vec3,
    domain: [f64; 4],
}

impl PlaneFrame {
    fn new(surface: &NurbsSurface) -> Result<Self, KernelRefusal> {
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let origin = surface.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?;
        let axis_u = surface.evaluate(u1, v0).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
        let axis_v = surface.evaluate(u0, v1).or_refuse(KernelStage::Refine, "evaluate")?.sub(origin);
        let normal = axis_u.cross(axis_v).normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let e1 = axis_u.normalized().or_refuse(KernelStage::Refine, "normalized")?;
        let e2 = normal.cross(e1);
        Ok(Self { origin, e1, e2, axis_u, axis_v, domain: [u0, u1, v0, v1] })
    }
    fn flat(&self, point: Vec3) -> P2 {
        let relative = point.sub(self.origin);
        [relative.dot(self.e1), relative.dot(self.e2)]
    }
    fn world(&self, point: P2) -> Vec3 {
        self.origin.add(self.e1.scale(point[0])).add(self.e2.scale(point[1]))
    }
    fn uv(&self, point: Vec3) -> Vec3 {
        let relative = point.sub(self.origin);
        let (a, b) = (relative.dot(self.axis_u), relative.dot(self.axis_v));
        let (guu, guv, gvv) = (
            self.axis_u.dot(self.axis_u),
            self.axis_u.dot(self.axis_v),
            self.axis_v.dot(self.axis_v),
        );
        let determinant = guu * gvv - guv * guv;
        let s = (a * gvv - b * guv) / determinant;
        let t = (b * guu - a * guv) / determinant;
        let [u0, u1, v0, v1] = self.domain;
        Vec3::new(u0 + s * (u1 - u0), v0 + t * (v1 - v0), 0.0)
    }
    fn at_uv(&self, u: f64, v: f64) -> Vec3 {
        let [u0, u1, v0, v1] = self.domain;
        self.origin
            .add(self.axis_u.scale((u - u0) / (u1 - u0)))
            .add(self.axis_v.scale((v - v0) / (v1 - v0)))
    }
}

/// The circle through three points, or `None` when they are collinear.
fn circle_through(a: P2, b: P2, c: P2) -> Option<(P2, f64)> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    if d.abs() <= 1e-300 {
        return None;
    }
    let (aa, bb, cc) = (dot(a, a), dot(b, b), dot(c, c));
    let centre = [
        (aa * (b[1] - c[1]) + bb * (c[1] - a[1]) + cc * (a[1] - b[1])) / d,
        (aa * (c[0] - b[0]) + bb * (a[0] - c[0]) + cc * (b[0] - a[0])) / d,
    ];
    Some((centre, length(sub(a, centre))))
}

/// Read one coedge's edge as a line or a circular arc in the frame, traversed
/// in the coedge's direction.
fn classify_edge(
    frame: &PlaneFrame,
    edge: &EdgeRecord,
    forward: bool,
    start: Vec3,
    end: Vec3,
    tolerance: f64,
) -> Result<Result<Element, String>, KernelRefusal> {
    // Samples in the COEDGE's direction.
    let sample = |fraction: f64| -> Result<P2, KernelRefusal> {
        let along = if forward { fraction } else { 1.0 - fraction };
        Ok(frame.flat(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * along).or_refuse(KernelStage::Refine, "evaluate")?))
    };
    let (a, b) = (frame.flat(start), frame.flat(end));
    let samples = (1..16)
        .map(|index| sample(index as f64 / 16.0))
        .collect::<Result<Vec<_>, _>>()?;
    let chord = sub(b, a);
    if length(chord) > tolerance
        && samples
            .iter()
            .all(|p| (cross(sub(*p, a), chord) / length(chord)).abs() <= tolerance)
    {
        return Ok(Ok(Element::Line { start: a, end: b }));
    }
    let middle = samples[7];
    // A closed edge (a whole circle) has start == end: fit through three
    // interior samples instead.
    let fit = if length(chord) > tolerance {
        circle_through(a, middle, b)
    } else {
        circle_through(samples[2], samples[7], samples[12])
    };
    // Neither a line nor a circular arc: the edge's own curve, flattened by
    // the frame's affine map (exact on the homogeneous control points).
    let free = || -> Result<Result<Element, String>, KernelRefusal> {
        let flat = NurbsCurve::new(
            edge.curve.degree,
            edge.curve.knots.clone(),
            edge.curve
                .control_points
                .iter()
                .map(|control| {
                    let [x, y] = frame.flat(control.point().or_refuse(KernelStage::Refine, "point")?);
                    Ok(Vec4::from_point(Vec3::new(x, y, 0.0), control.w))
                })
                .collect::<Result<Vec<_>, KernelRefusal>>()?,
        ).or_refuse(KernelStage::Refine, "new")?;
        let (ta, tb) = if forward { (edge.t0, edge.t1) } else { (edge.t1, edge.t0) };
        Ok(Ok(Element::Free { base: std::rc::Rc::new(FreeCurve::new(flat, ta, tb)), start: a, end: b }))
    };
    let Some((centre, radius)) = fit else {
        return free();
    };
    if samples
        .iter()
        .chain([&a, &b])
        .any(|p| (length(sub(*p, centre)) - radius).abs() > tolerance)
    {
        return free();
    }
    // The signed sweep: accumulate the turning angle about the centre over the
    // samples, which also reads a whole circle as ±2π.
    let mut sweep = 0.0;
    let mut previous = a;
    for point in samples.iter().chain([&b]) {
        sweep += cross(sub(previous, centre), sub(*point, centre))
            .atan2(dot(sub(previous, centre), sub(*point, centre)));
        previous = *point;
    }
    Ok(Ok(Element::Arc { centre, radius, start: a, end: b, sweep }))
}

/// Intersections of two offset elements treated as whole lines and circles.
fn intersections(first: &Element, second: &Element) -> Vec<P2> {
    let line_circle = |p: P2, d: P2, c: P2, r: f64| -> Vec<P2> {
        let f = sub(p, c);
        let (b, cc) = (dot(f, d), dot(f, f) - r * r);
        let disc = b * b - cc;
        if disc < 0.0 {
            return Vec::new();
        }
        let root = disc.sqrt();
        vec![add(p, scale(d, -b - root)), add(p, scale(d, -b + root))]
    };
    match (first, second) {
        (Element::Line { start: p, end: q }, Element::Line { start: r, end: s }) => {
            let (d, e) = (unit(sub(*q, *p)), unit(sub(*s, *r)));
            let denominator = cross(d, e);
            if denominator.abs() <= 1e-12 {
                return Vec::new();
            }
            let t = cross(sub(*r, *p), e) / denominator;
            vec![add(*p, scale(d, t))]
        }
        (Element::Line { start: p, end: q }, Element::Arc { centre, radius, .. })
        | (Element::Arc { centre, radius, .. }, Element::Line { start: p, end: q }) => {
            line_circle(*p, unit(sub(*q, *p)), *centre, *radius)
        }
        (
            Element::Arc { centre: c1, radius: r1, .. },
            Element::Arc { centre: c2, radius: r2, .. },
        ) => {
            let between = sub(*c2, *c1);
            let distance = length(between);
            if distance <= 1e-12 || distance > r1 + r2 || distance < (r1 - r2).abs() {
                return Vec::new();
            }
            let a = (r1 * r1 - r2 * r2 + distance * distance) / (2.0 * distance);
            let h = (r1 * r1 - a * a).max(0.0).sqrt();
            let base = add(*c1, scale(between, a / distance));
            let normal = [-between[1] / distance, between[0] / distance];
            vec![add(base, scale(normal, h)), add(base, scale(normal, -h))]
        }
        // A corner with a free-form side is refused before it gets here.
        _ => Vec::new(),
    }
}

fn polar(point: P2, centre: P2) -> f64 {
    (point[1] - centre[1]).atan2(point[0] - centre[0])
}

/// The free-form element's offset `p + pad·n`, pinned to the junctions `from`
/// and `to`, as a non-rational cubic in the frame — or the reason it is not.
///
/// A free-form curve's offset has no exact spline form, so this is a FIT, and
/// it is checked against the exact offset rather than against its own
/// stations: the cubic interpolates the exact offset at `n` stations uniform
/// in the edge's parameter, is read OUT OF SAMPLE at the quarter, half and
/// three-quarter points of every span, and must lie within `tolerance` of the
/// exact offset there. A miss doubles `n`, from 8 up to
/// [`FREE_OFFSET_MAX_STATIONS`], where the lane refuses by name. The
/// comparison is point to point at the same parameter, which bounds the
/// distance to the curve from above. The bar is the module's own
/// `1e-6 · scale`, the tolerance that classifies an outline element as a line
/// or an arc.
fn fit_free_offset(
    base: &FreeCurve,
    pad: f64,
    [f0, f1]: [f64; 2],
    from: P2,
    to: P2,
    signed_area: f64,
    tolerance: f64,
) -> Result<Result<NurbsCurve, String>, KernelRefusal> {
    if f1 <= f0 {
        return Ok(Err("a free-form outline edge's offset collapses between its corners".into()));
    }
    // `s` in [0, 1] runs the grown piece, fractions `f0 → f1` of the edge.
    let exact = |s: f64| -> Result<P2, KernelRefusal> {
        let fraction = f0 + (f1 - f0) * s;
        let (tangent, curvature) = base.tangent_and_curvature(fraction)?;
        if 1.0 + pad * curvature * signed_area.signum() <= 1e-6 {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "offset_shell_free_form_offset_folds", format!(
                "offset_shell: a free-form outline edge's offset folds at fraction {fraction:.3} \
                 (a concave stretch of radius {:.6} under the pad {pad:.6})",
                1.0 / curvature.abs()
            )));
        }
        let n = if signed_area > 0.0 { [tangent[1], -tangent[0]] } else { [-tangent[1], tangent[0]] };
        Ok(add(base.point(fraction)?, scale(n, pad)))
    };
    let mut stations = 8usize;
    let mut worst = f64::INFINITY;
    while stations <= FREE_OFFSET_MAX_STATIONS {
        let parameters = (0..=stations).map(|index| index as f64 / stations as f64).collect::<Vec<_>>();
        let mut points = parameters
            .iter()
            .map(|fraction| exact(*fraction).map(|p| Vec3::new(p[0], p[1], 0.0)))
            .collect::<Result<Vec<_>, KernelRefusal>>()?;
        points[0] = Vec3::new(from[0], from[1], 0.0);
        points[stations] = Vec3::new(to[0], to[1], 0.0);
        let curve = crate::interpolate_curve(&points, 3, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")?;
        worst = 0.0;
        for index in 0..stations {
            for quarter in [0.25, 0.5, 0.75] {
                let fraction = (index as f64 + quarter) / stations as f64;
                let fitted = curve.evaluate(fraction).or_refuse(KernelStage::Refine, "evaluate")?;
                worst = worst.max(length(sub([fitted.x, fitted.y], exact(fraction)?)));
            }
        }
        if worst <= tolerance {
            return Ok(Ok(curve));
        }
        stations *= 2;
    }
    Ok(Err(format!(
        "a free-form outline edge's offset by {pad:.6} is not fitted within {tolerance:.3e} at \
         {FREE_OFFSET_MAX_STATIONS} stations ({worst:.3e} out of sample)"
    )))
}

/// Where the free-form offset fit stops refining and refuses.
const FREE_OFFSET_MAX_STATIONS: usize = 1024;

/// The planar opening's outward wall over its outline offset by `pad`, or the
/// reason it is not built.
pub(super) fn offset_outline_wall(
    source: &BrepSolid,
    face: &FaceRecord,
    pad: f64,
) -> Result<Result<BrepSolid, String>, KernelRefusal> {
    if !face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(Err("the opening is not planar".into()));
    }
    let tolerance = 1e-6 * crate::solid_scale(source).max(1.0);
    let frame = PlaneFrame::new(&face.surface)?;
    let edge_by_id = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    let vertex_by_id = source
        .vertices
        .iter()
        .map(|vertex| (vertex.id, vertex.point))
        .collect::<HashMap<_, _>>();
    // The outer loop: the largest |uv area|, as the scaled wall keeps.
    let mut outer = None;
    let mut outer_area = f64::NEG_INFINITY;
    for (index, record) in face.loops.iter().enumerate() {
        let area = parameter_space_area(&FaceRecord {
            id: 0,
            surface: face.surface.clone(),
            same_sense: true,
            loops: vec![record.clone()],
            name: None,
        }).or_refuse(KernelStage::Refine, "parameter_space_area")?
        .abs();
        if area > outer_area {
            outer_area = area;
            outer = Some(index);
        }
    }
    let Some(outer) = outer else {
        return Ok(Err("the opening has no loop".into()));
    };
    let mut elements = Vec::new();
    for coedge in &face.loops[outer].coedges {
        let edge = edge_by_id
            .get(&coedge.edge_id)
            .ok_or_else(|| format!("offset_shell: missing edge {}", coedge.edge_id)).or_refuse(KernelStage::Refine, "offset_shell_missing_edge")?;
        if edge.degenerate {
            continue;
        }
        let (start_id, end_id) = if coedge.forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        };
        let (start, end) = (vertex_by_id[&start_id], vertex_by_id[&end_id]);
        match classify_edge(&frame, edge, coedge.forward, start, end, tolerance)? {
            Ok(element) => elements.push(element),
            Err(reason) => return Ok(Err(reason)),
        }
    }
    let count = elements.len();
    if count == 0 {
        return Ok(Err("the outline has no edge".into()));
    }
    // Orientation from a dense sample of the whole loop.
    let mut ring = Vec::new();
    for element in &elements {
        for index in 0..16 {
            ring.push(element.sample(index as f64 / 16.0));
        }
    }
    let signed_area = (0..ring.len())
        .map(|index| cross(ring[index], ring[(index + 1) % ring.len()]))
        .sum::<f64>()
        * 0.5;
    if signed_area.abs() <= tolerance * tolerance {
        return Ok(Err("the outline encloses no area".into()));
    }
    let outward = |tangent: P2| {
        if signed_area > 0.0 {
            [tangent[1], -tangent[0]]
        } else {
            [-tangent[1], tangent[0]]
        }
    };
    // Each element's offset, as a whole line or circle.
    let mut offsets = Vec::with_capacity(count);
    for element in &elements {
        match element.clone() {
            Element::Free { base, .. } => {
                // The offset curve p + pad·n is regular only where the outline
                // does not turn toward the offset side tighter than the pad:
                // the offset's speed is |p'|·(1 + pad·κ) with κ signed toward
                // the outward side. Past that it has a cusp and a swallowtail.
                let orientation = signed_area.signum();
                for index in 0..=256 {
                    let (_, curvature) = base.tangent_and_curvature(index as f64 / 256.0)?;
                    if 1.0 + pad * curvature * orientation <= 1e-6 {
                        return Ok(Err(format!(
                            "a concave stretch of a free-form outline edge, radius {:.6}, has no \
                             offset at {pad:.6}",
                            1.0 / curvature.abs()
                        )));
                    }
                }
                offsets.push(element.clone());
            }
            Element::Fitted { .. } => unreachable!("an outline element is never already fitted"),
            Element::Line { start, end } => {
                let n = outward(element.tangent_at(start));
                offsets.push(Element::Line { start: add(start, scale(n, pad)), end: add(end, scale(n, pad)) });
            }
            Element::Arc { centre, radius, start, end, sweep } => {
                let middle = element.sample(0.5);
                let sign = dot(outward(element.tangent_at(middle)), unit(sub(middle, centre))).signum();
                let grown = radius + sign * pad;
                if grown <= tolerance {
                    return Ok(Err(format!(
                        "a concave outline arc of radius {radius:.6} has no offset at {pad:.6}"
                    )));
                }
                let k = grown / radius;
                offsets.push(Element::Arc {
                    centre,
                    radius: grown,
                    start: add(centre, scale(sub(start, centre), k)),
                    end: add(centre, scale(sub(end, centre), k)),
                    sweep,
                });
            }
        }
    }
    // Each element's offset at `fraction` of its own traversal, read PAST its
    // ends too: a line or arc offset as the whole line or circle, a free-form
    // one as the exact offset of its continuation.
    let offset_at = |index: usize, fraction: f64| -> Result<P2, KernelRefusal> {
        match &elements[index] {
            Element::Free { base, .. } => {
                let (tangent, _) = base.tangent_and_curvature(fraction)?;
                Ok(add(base.point(fraction)?, scale(outward(tangent), pad)))
            }
            _ => Ok(offsets[index].sample(fraction)),
        }
    };
    // Where a free-form element's grown piece starts and ends, as fractions of
    // its traversal: 0 and 1 at a tangent junction, the solved mitre at a
    // corner (past the end at a convex one, short of it at a reflex one).
    let mut free_range = vec![[0.0f64, 1.0f64]; count];
    // The junction at each element's START, between its predecessor and it.
    let mut junctions = Vec::with_capacity(count);
    for index in 0..count {
        let previous = &elements[(index + count - 1) % count];
        let current = &elements[index];
        let vertex = current.start();
        let n_in = outward(previous.end_tangent()?);
        let n_out = outward(current.start_tangent()?);
        let naive_in = add(vertex, scale(n_in, pad));
        let naive_out = add(vertex, scale(n_out, pad));
        if length(sub(n_in, n_out)) <= 1e-9 {
            junctions.push(naive_out);
            continue;
        }
        if matches!(previous, Element::Free { .. }) || matches!(current, Element::Free { .. }) {
            // A free-form side meets its neighbour TANGENTLY where the two
            // offset points agree within the tolerance: the offsets meet
            // there. At a true corner the junction would have to be solved on
            // the offset curves themselves, which this lane does not do yet.
            if length(sub(naive_in, naive_out)) <= tolerance {
                junctions.push(scale(add(naive_in, naive_out), 0.5));
                continue;
            }
            // A CORNER: the two offsets meet where they cross, each read past
            // its end if it must be (a convex corner's mitre) — Newton on
            // offset_prev(fa) = offset_current(fb) from the corner itself.
            let before = (index + count - 1) % count;
            let (mut fa, mut fb) = (1.0f64, 0.0f64);
            let mut solved = false;
            // An evaluation outside the continuation's window ends the solve
            // unsolved, not in error.
            let at = |index: usize, fraction: f64| offset_at(index, fraction).ok();
            for _ in 0..60 {
                let (Some(pa), Some(pb)) = (at(before, fa), at(index, fb)) else {
                    break;
                };
                let gap = sub(pa, pb);
                if length(gap) <= tolerance * 1e-3 {
                    solved = true;
                    break;
                }
                let h = 1e-6;
                let (Some(a_hi), Some(a_lo), Some(b_hi), Some(b_lo)) =
                    (at(before, fa + h), at(before, fa - h), at(index, fb + h), at(index, fb - h))
                else {
                    break;
                };
                let da = scale(sub(a_hi, a_lo), 0.5 / h);
                let db = scale(sub(b_hi, b_lo), 0.5 / h);
                // Solve [da, −db]·(Δa, Δb) = −gap.
                let determinant = cross(da, scale(db, -1.0));
                if determinant.abs() <= 1e-300 {
                    break;
                }
                let step_a = cross(scale(gap, -1.0), scale(db, -1.0)) / determinant;
                let step_b = cross(da, scale(gap, -1.0)) / determinant;
                let limit = 0.25f64 / step_a.abs().max(step_b.abs()).max(0.25);
                fa += step_a * limit;
                fb += step_b * limit;
                if !(-1.0..=2.0).contains(&fa) || !(-1.0..=2.0).contains(&fb) {
                    break;
                }
            }
            if !solved {
                return Ok(Err(format!(
                    "outline corner {index} has a free-form side, and its offsets' crossing was not \
                     solved (the normals differ by {:.3e})",
                    length(sub(n_in, n_out))
                )));
            }
            let junction = offset_at(index, fb)?;
            if length(sub(junction, vertex)) > 12.0 * pad + tolerance {
                return Ok(Err(format!("outline corner {index} mitres past 12 pads")));
            }
            if matches!(previous, Element::Free { .. }) {
                free_range[before][1] = fa;
            }
            if matches!(current, Element::Free { .. }) {
                free_range[index][0] = fb;
            }
            junctions.push(junction);
            continue;
        }
        let target = scale(add(naive_in, naive_out), 0.5);
        let best = intersections(&offsets[(index + count - 1) % count], &offsets[index])
            .into_iter()
            .min_by(|a, b| length(sub(*a, target)).total_cmp(&length(sub(*b, target))));
        let Some(junction) = best else {
            return Ok(Err(format!("the offsets meeting at outline corner {index} never meet")));
        };
        // A corner so sharp that its mitre runs far past the pad is not a
        // wall this construction should build.
        if length(sub(junction, vertex)) > 12.0 * pad + tolerance {
            return Ok(Err(format!("outline corner {index} mitres past 12 pads")));
        }
        junctions.push(junction);
    }
    // The grown elements, between consecutive junctions.
    let mut grown = Vec::with_capacity(count);
    for index in 0..count {
        let (from, to) = (junctions[index], junctions[(index + 1) % count]);
        match (elements[index].clone(), offsets[index].clone()) {
            (Element::Free { base, .. }, _) => match fit_free_offset(&base, pad, free_range[index], from, to, signed_area, tolerance)? {
                Ok(curve) => grown.push(Element::Fitted { curve, start: from, end: to }),
                Err(reason) => return Ok(Err(reason)),
            },
            (Element::Line { start, end }, _) => {
                if dot(sub(to, from), sub(end, start)) <= tolerance {
                    return Ok(Err(format!("offset outline edge {index} collapses or reverses")));
                }
                grown.push(Element::Line { start: from, end: to });
            }
            (Element::Arc { sweep, .. }, Element::Arc { centre, radius, .. }) => {
                let (a, b) = (polar(from, centre), polar(to, centre));
                let mut travelled = b - a;
                if sweep > 0.0 {
                    travelled = travelled.rem_euclid(TAU);
                    if travelled <= 1e-9 && sweep > PI {
                        travelled = TAU;
                    }
                } else {
                    travelled = -(-travelled).rem_euclid(TAU);
                    if travelled >= -1e-9 && sweep < -PI {
                        travelled = -TAU;
                    }
                }
                if travelled.abs() <= 1e-9 || (travelled - sweep).abs() > PI {
                    return Ok(Err(format!("offset outline arc {index} collapses or reverses")));
                }
                grown.push(Element::Arc { centre, radius, start: from, end: to, sweep: travelled });
            }
            _ => unreachable!("an offset keeps its element's kind"),
        }
    }
    // Simple: no two non-adjacent pieces of the sampled outline cross.
    let mut polyline = Vec::new();
    for element in &grown {
        let steps = match element {
            Element::Line { .. } => 1,
            Element::Fitted { .. } => 96,
            _ => 24,
        };
        for index in 0..steps {
            polyline.push(element.sample(index as f64 / steps as f64));
        }
    }
    let n = polyline.len();
    let proper = |a: P2, b: P2, c: P2, d: P2| {
        let (o1, o2) = (cross(sub(b, a), sub(c, a)), cross(sub(b, a), sub(d, a)));
        let (o3, o4) = (cross(sub(d, c), sub(a, c)), cross(sub(d, c), sub(b, c)));
        o1 * o2 < 0.0 && o3 * o4 < 0.0
    };
    for i in 0..n {
        for j in i + 2..n {
            if i == 0 && j == n - 1 {
                continue;
            }
            if proper(polyline[i], polyline[(i + 1) % n], polyline[j], polyline[(j + 1) % n]) {
                return Ok(Err("the offset outline crosses itself".into()));
            }
        }
    }

    // Build the wall: the same plane, its net enlarged over the grown outline.
    let uv_ring = polyline.iter().map(|p| frame.uv(frame.world(*p))).collect::<Vec<_>>();
    let margin = 1e-9;
    let ua = uv_ring.iter().map(|p| p.x).fold(f64::INFINITY, f64::min) - margin;
    let ub = uv_ring.iter().map(|p| p.x).fold(f64::NEG_INFINITY, f64::max) + margin;
    let va = uv_ring.iter().map(|p| p.y).fold(f64::INFINITY, f64::min) - margin;
    let vb = uv_ring.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max) + margin;
    let net = vec![
        vec![Vec4::from_point(frame.at_uv(ua, va), 1.0), Vec4::from_point(frame.at_uv(ua, vb), 1.0)],
        vec![Vec4::from_point(frame.at_uv(ub, va), 1.0), Vec4::from_point(frame.at_uv(ub, vb), 1.0)],
    ];
    let surface = NurbsSurface::new(1, 1, vec![ua, ua, ub, ub], vec![va, va, vb, vb], net).or_refuse(KernelStage::Refine, "new")?;
    let closed = count == 1;
    let vertices = (0..count)
        .map(|index| VertexRecord { id: index as u64 + 1, point: frame.world(junctions[index]) })
        .collect::<Vec<_>>();
    let mut edges = Vec::with_capacity(count);
    let mut coedges = Vec::with_capacity(count);
    for (index, element) in grown.iter().enumerate() {
        let curve = match element.clone() {
            Element::Fitted { curve, .. } => NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                curve
                    .control_points
                    .iter()
                    .map(|control| {
                        let point = control.point().or_refuse(KernelStage::Refine, "point")?;
                        Ok(Vec4::from_point(frame.world([point.x, point.y]), control.w))
                    })
                    .collect::<Result<Vec<_>, KernelRefusal>>()?,
            ).or_refuse(KernelStage::Refine, "new")?,
            Element::Free { .. } => unreachable!("a grown free-form element is fitted"),
            Element::Line { start, end } => make_line(frame.world(start), frame.world(end)).or_refuse(KernelStage::Refine, "make_line")?,
            Element::Arc { centre, radius, start, sweep, .. } => {
                let y_axis = if sweep > 0.0 { frame.e2 } else { frame.e2.scale(-1.0) };
                let local = sub(start, centre);
                let start_angle = if sweep > 0.0 { local[1].atan2(local[0]) } else { (-local[1]).atan2(local[0]) };
                make_arc(frame.world(centre), frame.e1, y_axis, radius, start_angle, start_angle + sweep.abs()).or_refuse(KernelStage::Refine, "make_arc")?
            }
        };
        let pcurve = NurbsCurve::new(
            curve.degree,
            curve.knots.clone(),
            curve
                .control_points
                .iter()
                .map(|control| Ok(Vec4::from_point(frame.uv(control.point().or_refuse(KernelStage::Refine, "point")?), control.w)))
                .collect::<Result<Vec<_>, KernelRefusal>>()?,
        ).or_refuse(KernelStage::Refine, "new")?;
        let [t0, t1] = curve.domain().or_refuse(KernelStage::Refine, "domain")?;
        let next = if closed { 0 } else { (index + 1) % count };
        edges.push(EdgeRecord {
            id: index as u64 + 1,
            curve,
            t0,
            t1,
            start_vertex_id: index as u64 + 1,
            end_vertex_id: next as u64 + 1,
            degenerate: false,
            name: None,
        });
        coedges.push(CoedgeRecord { id: index as u64 + 1, edge_id: index as u64 + 1, forward: true, pcurve });
    }
    Ok(Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        id: source.id,
        vertices,
        edges,
        shells: vec![ShellRecord {
            id: 1,
            faces: vec![FaceRecord {
                id: face.id,
                surface,
                same_sense: face.same_sense,
                loops: vec![LoopRecord { id: 1, coedges }],
                name: face.name.clone(),
            }],
        }],
        genus: 0,
    }))
}

/// Whether two wall carriers trim the same region: every sampled point of each
/// outline lies within `tolerance` of the other's TRIMMED BOUNDARY CURVES.
///
/// The scaled wall IS the offset outline for a rectangle filling its patch and
/// for a circle centred on it, which is most openings. Where the two agree, the
/// scaled wall is kept: it is the construction every existing shell was built
/// and named by, and rebuilding an identical region on a different patch would
/// renumber the fragments it later cuts into (measured: the outward
/// sphere-under-frustum case row's worst vector-area face moved from
/// `P.CO2_S_1` to `P.CO2_S`, with its volume, area and counts unchanged).
///
/// The comparison is point-to-CURVE, not polyline-to-polyline, because the two
/// walls do not share a parameterization: the scaled wall's outline is the
/// source's own pcurves, and this module's is `make_arc`'s. On a rim a boolean
/// has split, a 48-gon of one is a chord of the other, and the two read
/// `R(1 − cos(θ/2))` apart — 1e-4 of the radius, four decades over the band —
/// so a sampled comparison sends an already-correct wall down the rebuild.
/// A split rim also shows what the scaled wall does there, which is why it takes
/// this lane either way: on a cylinder whose top rim is split at t = 0.37, the
/// scaled wall's outline runs from radius 2.499869769 to 2.501280935 where the
/// offset outline is 2.5 exactly. Scaling two arcs about a sampled trim box is
/// not a circle.
pub(super) fn walls_trim_the_same_region(
    first: &BrepSolid,
    second: &BrepSolid,
    tolerance: f64,
) -> Result<bool, KernelRefusal> {
    let stations = |solid: &BrepSolid| -> Result<Vec<Vec3>, KernelRefusal> {
        let face = &solid.shells[0].faces[0];
        let mut points = Vec::new();
        for record in &face.loops {
            for coedge in &record.coedges {
                let [t0, t1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
                for index in 0..24 {
                    let uv = coedge.pcurve.evaluate(t0 + (t1 - t0) * index as f64 / 24.0).or_refuse(KernelStage::Refine, "evaluate")?;
                    points.push(face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?);
                }
            }
        }
        Ok(points)
    };
    // The distance from a point to a solid's trimmed boundary curves, exactly
    // for the lines and rational arcs both walls are made of.
    let to_boundary = |point: Vec3, solid: &BrepSolid| -> Result<f64, KernelRefusal> {
        let mut nearest = f64::INFINITY;
        for edge in &solid.edges {
            let projection = crate::project_point_to_curve(&edge.curve, point).or_refuse(KernelStage::Refine, "project_point_to_curve")?;
            let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
            let clamped = projection.u.clamp(low, high);
            nearest = nearest.min(edge.curve.evaluate(clamped).or_refuse(KernelStage::Refine, "evaluate")?.sub(point).length());
        }
        Ok(nearest)
    };
    let (a, b) = (stations(first)?, stations(second)?);
    if a.is_empty() || b.is_empty() || first.edges.is_empty() || second.edges.is_empty() {
        return Ok(false);
    }
    for point in &a {
        if to_boundary(*point, second)? > tolerance {
            return Ok(false);
        }
    }
    for point in &b {
        if to_boundary(*point, first)? > tolerance {
            return Ok(false);
        }
    }
    Ok(true)
}

/// Whether the SCALED outward wall still contains every retained neighbour's
/// grown offset SECTION with the opening plane — the curves the imprint
/// actually cuts it along: `Ok(Ok(()))`, or the worst uncovered distance.
///
/// The scaled wall is the fallback for an outline this module does not offset.
/// It does not have to BE the offset outline, only to contain each
/// neighbour's section, so the pair imprint cuts it and the wall fragment
/// between the source rim and that section survives. Until 2026-09-26 the
/// check asked a PROXY — whether the whole offset outline, the source outline
/// moved by the pad along its in-plane normal, was covered — and refused two
/// outward free-form members (a twisted loft grown by 0.3 and 0.2, and three
/// oblique cuts) at 8.803e-2 and 4.031e-2 outside, whose bodies had read
/// +4.3e-6 / +1.9e-5 / +9.0e-6 / +2.1e-6 / +3.5e-6 against kernel-free
/// divergence integrals: a point of the outline can lie outside the pad while
/// every section is inside it (the 2026-09-17 outline-walls record, "the
/// sharper question"). The check now asks the requirement itself. For each
/// station of a retained neighbour's rim, the neighbour's carrier surface —
/// the carrier the imprint will cut with, at the station's own (u, v), which
/// the cloned trim keeps — is walked along the isoline leaving the trim until
/// it crosses the opening plane, and that crossing must lie inside the wall.
/// A station whose isoline never crosses the plane within the carrier's
/// domain falls back to the proxy point, so nothing the proxy caught goes
/// uncaught. `carrier_surfaces` maps a retained source face to its carrier;
/// a neighbour with no carrier keeps the proxy too.
///
/// The scaled wall holds when the outline is star-shaped about the scale
/// centre with the pad to spare, and fails exactly where it did on the
/// L-step: an edge through the centre does not move.
pub(super) fn scaled_wall_covers_offset(
    source: &BrepSolid,
    face: &FaceRecord,
    wall: &BrepSolid,
    pad: f64,
    source_faces: &[&FaceRecord],
    opening_set: &HashSet<u64>,
    carrier_surfaces: &[(u64, &NurbsSurface)],
) -> Result<Result<(), f64>, KernelRefusal> {
    let plane_point = face.surface.evaluate(
        {
            let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            (u0 + u1) * 0.5
        },
        {
            let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            (v0 + v1) * 0.5
        },
    ).or_refuse(KernelStage::Refine, "evaluate")?;
    let plane_normal = {
        let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        face.surface.normal((u0 + u1) * 0.5, (v0 + v1) * 0.5).or_refuse(KernelStage::Refine, "normal")?
    };
    // The plane crossing of `surface`'s isoline through `uv` in the uv
    // direction `step` (a unit step scaled to the domain), nearest the rim.
    let section_crossing = |surface: &NurbsSurface, uv: [f64; 2], step: [f64; 2]| -> Result<Option<Vec3>, KernelRefusal> {
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
        let height = |t: f64| -> Result<(f64, Vec3), KernelRefusal> {
            let point = surface.evaluate(
                (uv[0] + t * step[0]).clamp(u0, u1),
                (uv[1] + t * step[1]).clamp(v0, v1),
            ).or_refuse(KernelStage::Refine, "evaluate")?;
            Ok((point.sub(plane_point).dot(plane_normal), point))
        };
        // How far the ray can go before it leaves the domain.
        let mut reach = f64::INFINITY;
        for (position, delta, low, high) in [(uv[0], step[0], u0, u1), (uv[1], step[1], v0, v1)] {
            if delta > 0.0 {
                reach = reach.min((high - position) / delta);
            } else if delta < 0.0 {
                reach = reach.min((low - position) / delta);
            }
        }
        if !reach.is_finite() || reach <= 0.0 {
            return Ok(None);
        }
        let (mut previous, _) = height(0.0)?;
        for index in 1..=64 {
            let t = reach * index as f64 / 64.0;
            let (current, point) = height(t)?;
            if current == 0.0 {
                return Ok(Some(point));
            }
            if (previous < 0.0) != (current < 0.0) {
                let (mut a, mut b) = (reach * (index - 1) as f64 / 64.0, t);
                let mut a_sign = previous < 0.0;
                for _ in 0..48 {
                    let mid = 0.5 * (a + b);
                    let (mid_height, _) = height(mid)?;
                    if (mid_height < 0.0) == a_sign {
                        a = mid;
                        a_sign = mid_height < 0.0;
                    } else {
                        b = mid;
                    }
                }
                return Ok(Some(height(0.5 * (a + b))?.1));
            }
            previous = current;
        }
        Ok(None)
    };
    let tolerance = 1e-6 * crate::solid_scale(source).max(1.0);
    let frame = PlaneFrame::new(&face.surface)?;
    let edge_by_id = source.edges.iter().map(|edge| (edge.id, edge)).collect::<HashMap<_, _>>();
    // The scaled wall's outline, sampled, in the source plane's frame.
    let wall_face = &wall.shells[0].faces[0];
    let mut hull = Vec::new();
    for record in &wall_face.loops {
        for coedge in &record.coedges {
            let [t0, t1] = coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            for index in 0..32 {
                let uv = coedge.pcurve.evaluate(t0 + (t1 - t0) * index as f64 / 32.0).or_refuse(KernelStage::Refine, "evaluate")?;
                hull.push(frame.flat(wall_face.surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?));
            }
        }
    }
    let inside = |point: P2| -> f64 {
        // 0 when inside or on the hull, else the distance to it.
        let mut winding = 0i32;
        let mut nearest = f64::INFINITY;
        for index in 0..hull.len() {
            let (a, b) = (hull[index], hull[(index + 1) % hull.len()]);
            let along = sub(b, a);
            let t = (dot(sub(point, a), along) / dot(along, along).max(1e-300)).clamp(0.0, 1.0);
            nearest = nearest.min(length(sub(point, add(a, scale(along, t)))));
            if a[1] <= point[1] {
                if b[1] > point[1] && cross(along, sub(point, a)) > 0.0 {
                    winding += 1;
                }
            } else if b[1] <= point[1] && cross(along, sub(point, a)) < 0.0 {
                winding -= 1;
            }
        }
        if winding != 0 { 0.0 } else { nearest }
    };
    // The source outline's orientation and its retained-edge offsets.
    let outer = face
        .loops
        .iter()
        .max_by(|a, b| {
            let area = |record: &LoopRecord| {
                parameter_space_area(&FaceRecord {
                    id: 0,
                    surface: face.surface.clone(),
                    same_sense: true,
                    loops: vec![record.clone()],
                    name: None,
                })
                .map(f64::abs)
                .unwrap_or(0.0)
            };
            area(a).total_cmp(&area(b))
        })
        .ok_or_else(|| "offset_shell: opening has no loop".to_string()).or_refuse(KernelStage::Refine, "offset_shell_opening_has_no_loop")?;
    let mut samples = Vec::new();
    for coedge in &outer.coedges {
        let edge = edge_by_id[&coedge.edge_id];
        for index in 0..=32 {
            let along = index as f64 / 32.0;
            let along = if coedge.forward { along } else { 1.0 - along };
            samples.push((coedge.edge_id, frame.flat(edge.curve.evaluate(edge.t0 + (edge.t1 - edge.t0) * along).or_refuse(KernelStage::Refine, "evaluate")?)));
        }
    }
    let signed_area = (0..samples.len())
        .map(|index| cross(samples[index].1, samples[(index + 1) % samples.len()].1))
        .sum::<f64>()
        * 0.5;
    let mut worst: f64 = 0.0;
    let (mut by_section, mut by_proxy) = (0usize, 0usize);
    for index in 0..samples.len() {
        let (edge_id, point) = samples[index];
        let Some((mate, mate_coedge)) = junction_mate(source_faces, face, edge_id) else {
            continue;
        };
        if opening_set.contains(&mate.id) {
            continue;
        }
        let (previous, next) = (samples[(index + samples.len() - 1) % samples.len()].1, samples[(index + 1) % samples.len()].1);
        let chord = sub(next, previous);
        if length(chord) <= tolerance {
            continue;
        }
        let tangent = unit(chord);
        let n = if signed_area > 0.0 { [tangent[1], -tangent[0]] } else { [-tangent[1], tangent[0]] };
        // The neighbour's own section with the plane, where its carrier is
        // known: the station's (u, v) on the mate, then along the isoline
        // that leaves the mate's trim, whichever side crosses the plane.
        let mut section = None;
        if let Some((_, carrier)) = carrier_surfaces.iter().find(|(id, _)| *id == mate.id) {
            let edge = edge_by_id[&edge_id];
            let along = {
                // `point` was sampled at this fraction of the edge from t0.
                let t = {
                    let mut best = (f64::INFINITY, 0.0);
                    for k in 0..=64 {
                        let candidate = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 64.0;
                        let d = length(sub(frame.flat(edge.curve.evaluate(candidate).or_refuse(KernelStage::Refine, "evaluate")?), point));
                        if d < best.0 {
                            best = (d, candidate);
                        }
                    }
                    best.1
                };
                (t - edge.t0) / (edge.t1 - edge.t0)
            };
            let [q0, q1] = mate_coedge.pcurve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let q = if mate_coedge.forward { q0 + (q1 - q0) * along } else { q1 - (q1 - q0) * along };
            let uv = mate_coedge.pcurve.evaluate(q).or_refuse(KernelStage::Refine, "evaluate")?;
            let ahead = mate_coedge.pcurve.evaluate((q + 1e-3 * (q1 - q0)).clamp(q0.min(q1), q0.max(q1))).or_refuse(KernelStage::Refine, "evaluate")?;
            let behind = mate_coedge.pcurve.evaluate((q - 1e-3 * (q1 - q0)).clamp(q0.min(q1), q0.max(q1))).or_refuse(KernelStage::Refine, "evaluate")?;
            let d = [ahead.x - behind.x, ahead.y - behind.y];
            let [su0, su1] = carrier.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
            let [sv0, sv1] = carrier.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
            let d = [d[0] / (su1 - su0).abs().max(1e-300), d[1] / (sv1 - sv0).abs().max(1e-300)];
            let l = (d[0] * d[0] + d[1] * d[1]).sqrt();
            if l > 0.0 {
                // The isoline most transverse to the rim: a v-isoline where
                // the rim runs mostly along u, else a u-isoline. Not the uv
                // perpendicular — at a station on the domain's u or v edge (a
                // wall corner) that leaves the domain in one sense and runs
                // away from the plane in the other.
                let perpendicular = if d[0].abs() >= d[1].abs() {
                    [0.0, (sv1 - sv0).abs()]
                } else {
                    [(su1 - su0).abs(), 0.0]
                };
                let mut best: Option<Vec3> = None;
                for sign in [1.0, -1.0] {
                    if let Some(crossing) =
                        section_crossing(carrier, [uv.x, uv.y], [sign * perpendicular[0], sign * perpendicular[1]])?
                    {
                        let closer = best.is_none_or(|kept: Vec3| {
                            crossing.sub(frame.world(point)).length() < kept.sub(frame.world(point)).length()
                        });
                        if closer {
                            best = Some(crossing);
                        }
                    }
                }
                section = best;
            }
        }
        match section {
            Some(crossing) => {
                by_section += 1;
                worst = worst.max(inside(frame.flat(crossing)));
            }
            None => {
                by_proxy += 1;
                worst = worst.max(inside(add(point, scale(n, pad))));
            }
        }
    }
    os_debug!(
        "scaled wall for opening face {}: {by_section} station(s) read by their neighbour's section, \
         {by_proxy} by the outline proxy; worst outside {worst:.3e}",
        face.id
    );
    Ok(if worst <= tolerance * 10.0 { Ok(()) } else { Err(worst) })
}

/// A RETAINED planar face's outward carrier rebuilt over its outline OFFSET,
/// when that outline has a free-form edge: `Ok(Some(carrier))`, or `Ok(None)`
/// to keep the carrier as built.
///
/// An outward carrier is grown by sliding the plane's net under cloned
/// pcurves, which scales the trim about the patch centre, the construction the
/// opening wall used to have. For lines and arcs the neighbours' offsets are
/// met anyway, and every such shell keeps its carrier. A free-form outline
/// scaled that way falls inside its own offset: an ellipse's floor, grown by
/// 0.3, kept the SCALED ellipse (semi-axes 4.3 × 2.3) as its trim while the
/// walls' offsets cut the plane along the parallel curve up to 7.9e-2 outside
/// it, so the imprint missed and the floor stayed one shell of its own. Here the
/// trim is the outline offset by the carrier's own extension, on the carrier's
/// own (offset) plane. Only a UNIFORM extension is rebuilt, so a face with a
/// smooth junction (which grows by nothing there) keeps its carrier.
pub(super) fn outline_offset_carrier(
    source: &BrepSolid,
    face: &FaceRecord,
    extension: f64,
    clearance: f64,
    carrier: &BrepSolid,
) -> Result<Option<BrepSolid>, KernelRefusal> {
    // Only a FREE-FORM outline: a line/arc outline keeps its slid-net carrier
    // even where that trim is not its offset (a concave arc's floor is not),
    // because every such shell was built and gated on it — re-gating on
    // "trim differs" split the notched box's outward shell into two shells.
    if extension <= 0.0 || !face.surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? || !outline_has_free_element(source, face)? {
        return Ok(None);
    }
    // Compared at the extension itself, built with the clearance.
    let Ok(exact) = offset_outline_wall(source, face, extension)? else {
        return Ok(None);
    };
    let Ok(wall) = offset_outline_wall(source, face, extension + clearance)? else {
        return Ok(None);
    };
    let carrier_face = &carrier.shells[0].faces[0];
    let frame = PlaneFrame::new(&face.surface)?;
    let normal = frame.e1.cross(frame.e2);
    let [u0, u1] = carrier_face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = carrier_face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let shift = normal.scale(carrier_face.surface.evaluate(u0, v0).or_refuse(KernelStage::Refine, "evaluate")?.sub(frame.origin).dot(normal));
    // The carrier plane must be the source plane translated: a tilted carrier
    // is not a plane this construction knows how to place.
    for (u, v) in [(u1, v0), (u0, v1), (u1, v1)] {
        let departure = carrier_face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?.sub(frame.origin).dot(normal) - shift.dot(normal);
        if departure.abs() > 1e-9 * crate::solid_scale(source).max(1.0) {
            return Ok(None);
        }
    }
    let translate = |control: &Vec4| -> Result<Vec4, KernelRefusal> { Ok(Vec4::from_point(control.point().or_refuse(KernelStage::Refine, "point")?.add(shift), control.w)) };
    let place = |wall: BrepSolid| -> Result<BrepSolid, KernelRefusal> {
    let mut rebuilt = wall;
    for vertex in &mut rebuilt.vertices {
        vertex.point = vertex.point.add(shift);
    }
    for edge in &mut rebuilt.edges {
        edge.curve = NurbsCurve::new(
            edge.curve.degree,
            edge.curve.knots.clone(),
            edge.curve.control_points.iter().map(translate).collect::<Result<_, _>>()?,
        ).or_refuse(KernelStage::Refine, "new")?;
    }
    let rebuilt_face = &mut rebuilt.shells[0].faces[0];
    let surface = &rebuilt_face.surface;
    rebuilt_face.surface = NurbsSurface::new(
        surface.degree_u,
        surface.degree_v,
        surface.knots_u.clone(),
        surface.knots_v.clone(),
        surface
            .control_points
            .iter()
            .map(|row| row.iter().map(translate).collect::<Result<Vec<_>, _>>())
            .collect::<Result<_, _>>()?,
    ).or_refuse(KernelStage::Refine, "new")?;
    rebuilt.id = carrier.id;
    rebuilt_face.id = carrier_face.id;
    rebuilt_face.same_sense = carrier_face.same_sense;
    rebuilt_face.name = carrier_face.name.clone();
    rebuilt.genus = carrier.genus;
    Ok(rebuilt)
    };
    // Where the slid net's trim already IS the offset outline — a rectangle
    // filling its patch, a circle centred on it — the carrier is kept, as the
    // opening wall keeps its scaled patch, so no shell that was right before
    // is rebuilt on a different patch (`walls_trim_the_same_region`).
    if walls_trim_the_same_region(&place(exact)?, carrier, 1e-9f64.max(crate::solid_scale(source) * 1e-11))? {
        return Ok(None);
    }
    Ok(Some(place(wall)?))
}

/// Whether the face's outer outline has an edge that is neither a line nor a
/// circular arc.
pub(super) fn outline_has_free_element(source: &BrepSolid, face: &FaceRecord) -> Result<bool, KernelRefusal> {
    let tolerance = 1e-6 * crate::solid_scale(source).max(1.0);
    let frame = PlaneFrame::new(&face.surface)?;
    let vertex_by_id = source.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect::<HashMap<_, _>>();
    for coedge in face.loops.iter().flat_map(|record| &record.coedges) {
        let Some(edge) = source.edges.iter().find(|edge| edge.id == coedge.edge_id) else {
            continue;
        };
        if edge.degenerate {
            continue;
        }
        let (start_id, end_id) = if coedge.forward {
            (edge.start_vertex_id, edge.end_vertex_id)
        } else {
            (edge.end_vertex_id, edge.start_vertex_id)
        };
        let (Some(start), Some(end)) = (vertex_by_id.get(&start_id), vertex_by_id.get(&end_id)) else {
            continue;
        };
        if let Ok(Element::Free { .. }) = classify_edge(&frame, edge, coedge.forward, *start, *end, tolerance)? {
            return Ok(true);
        }
    }
    Ok(false)
}
