//! Point queries against a solid's boundary, on its exact geometry: the
//! boundary points nearest a point, the outward normal there, and the
//! principal curvatures there; and the least box with sides square to the
//! axes round the solid.
//!
//! The nearest boundary point is the least of three kinds of foot: each
//! face's surface foot where it lies inside the face's trim, each edge's curve
//! foot within the edge's range, and each vertex. A face foot outside its trim
//! is no candidate: the face's nearest trimmed point then lies on its
//! boundary, which the edge and vertex feet cover.

use crate::{
    classify_point, project_point_to_curve, project_point_to_surface, BrepSolid, FaceRecord,
    KernelRefusal, KernelStage, PointClass, PolygonClass, Vec2, Vec3,
};

/// Where a boundary foot lies: inside one face, on one edge, or at a vertex.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BoundaryPlace {
    /// Inside the trim of the face with this id, at `(u, v)` on its surface.
    Face { face: u64, uv: [f64; 2] },
    /// On the edge with this id, at curve parameter `t`.
    Edge { edge: u64, t: f64 },
    /// At the vertex with this id.
    Vertex { vertex: u64 },
}

/// A point of the boundary near a query point.
#[derive(Clone, Copy, Debug)]
pub struct BoundaryFoot {
    pub point: Vec3,
    pub distance: f64,
    pub place: BoundaryPlace,
}

fn refusal(code: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::internal(KernelStage::Classify, code, message)
}

/// The trim tolerance in a face's parameters for a 3D `tolerance` at `uv`:
/// the 3D band over the larger of the two parameter speeds there.
fn parameter_tolerance(face: &FaceRecord, uv: [f64; 2], tolerance: f64) -> f64 {
    match face.surface.deriv1(uv[0], uv[1]) {
        Ok((_, su, sv)) => {
            let speed = su.length().max(sv.length());
            if speed > 0.0 && speed.is_finite() { tolerance / speed } else { tolerance }
        }
        Err(_) => tolerance,
    }
}

/// Where the parameter point lies against the face's trim, its boundary a
/// band of `tolerance` (3D).
fn trim_class(face: &FaceRecord, uv: [f64; 2], tolerance: f64) -> Result<PolygonClass, KernelRefusal> {
    let band = parameter_tolerance(face, uv, tolerance);
    crate::parameter_point_in_face(face, Vec2 { x: uv[0], y: uv[1] }, band).map_err(|e| refusal("boundary_trim", e))
}

/// Whether the parameter point lies inside the face's trim, its boundary
/// within `tolerance` (3D) included.
fn inside_trim(face: &FaceRecord, uv: [f64; 2], tolerance: f64) -> Result<bool, KernelRefusal> {
    Ok(trim_class(face, uv, tolerance)? != PolygonClass::Outside)
}

/// Whether a surface foot is a true foot: the point lies along the surface's
/// normal from it. A foot the projector clamped to the surface's domain (the
/// true nearest point lies past it) leaves a residual along the surface; its
/// place on the boundary is an edge's or a vertex's, which those feet give.
fn is_true_foot(face: &FaceRecord, uv: [f64; 2], foot: Vec3, point: Vec3, tolerance: f64) -> bool {
    let residual = point.sub(foot);
    let length = residual.length();
    if length <= tolerance {
        return true;
    }
    let Some(normal) = outward_normal(face, uv) else { return false };
    let along = residual.sub(normal.scale(residual.dot(normal))).length();
    along <= tolerance.max(1e-9 * length)
}

/// Samples per direction of the grid a face's trimmed region is searched from
/// when the surface's global foot lies outside the trim.
const TRIM_SEED_GRID: usize = 16;

/// The face's feet of `point` that lie on its trimmed region: the surface's
/// global foot, and the local foot Newton reaches from the nearest sample of a
/// grid over the trimmed region (a minimum inside the trim that is not the
/// surface's global one), each kept when it lies inside the trim (its band
/// included) and is a true foot.
#[doc(hidden)]
pub fn face_feet(face: &FaceRecord, point: Vec3, tolerance: f64) -> Result<Vec<(Vec3, f64, [f64; 2], bool)>, KernelRefusal> {
    let mut feet = Vec::new();
    let mut take = |foot: crate::SurfaceProjection| -> Result<bool, KernelRefusal> {
        let uv = [foot.u, foot.v];
        let class = trim_class(face, uv, tolerance)?;
        let inside = class != PolygonClass::Outside && is_true_foot(face, uv, foot.point, point, tolerance);
        if inside {
            feet.push((foot.point, foot.distance, uv, class == PolygonClass::Boundary));
        }
        Ok(inside)
    };
    let global = project_point_to_surface(&face.surface, point).map_err(|e| refusal("boundary_face_foot", e))?;
    // A plane holds no foot but its global one: the distance is convex over it,
    // so with that foot outside the trim the nearest point of the trimmed
    // region lies on its boundary, which the edge and vertex feet give.
    if take(global)? || matches!(face.surface.analytic(), Some(crate::AnalyticSurface::Plane { .. })) {
        return Ok(feet);
    }
    let surface = &face.surface;
    let ([u0, u1], [v0, v1]) = match (surface.domain_u(), surface.domain_v()) {
        (Ok(u), Ok(v)) => (u, v),
        _ => return Ok(feet),
    };
    let mut nearest: Option<(f64, [f64; 2])> = None;
    for i in 0..=TRIM_SEED_GRID {
        for j in 0..=TRIM_SEED_GRID {
            let uv = [u0 + (u1 - u0) * i as f64 / TRIM_SEED_GRID as f64, v0 + (v1 - v0) * j as f64 / TRIM_SEED_GRID as f64];
            if trim_class(face, uv, tolerance)? == PolygonClass::Outside {
                continue;
            }
            let Ok(at) = surface.evaluate(uv[0], uv[1]) else { continue };
            let distance = at.sub(point).length();
            if nearest.is_none_or(|(best, _)| distance < best) {
                nearest = Some((distance, uv));
            }
        }
    }
    if let Some((_, seed)) = nearest {
        // The surface's own Newton from the seed (the closed forms of the analytic
        // carriers answer the global foot, the one already rejected).
        if let Ok(local) = crate::projection::project_point_to_surface_seeded(surface, point, seed[0], seed[1]) {
            take(local)?;
        }
    }
    Ok(feet)
}

/// Every face foot of `point` on its face's trimmed region ([`face_feet`]), and
/// every edge and vertex foot, each with its distance; and whether each face
/// foot lies on its trim's boundary band.
fn boundary_feet(solid: &BrepSolid, point: Vec3, tolerance: f64) -> Result<Vec<(BoundaryFoot, bool)>, KernelRefusal> {
    let mut feet = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        for (foot, distance, uv, on_band) in face_feet(face, point, tolerance)? {
            feet.push((BoundaryFoot { point: foot, distance, place: BoundaryPlace::Face { face: face.id, uv } }, on_band));
        }
    }
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        let foot = project_point_to_curve(&edge.curve, point).map_err(|e| refusal("boundary_edge_foot", e))?;
        let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        if foot.u >= low && foot.u <= high {
            feet.push((BoundaryFoot { point: foot.point, distance: foot.distance, place: BoundaryPlace::Edge { edge: edge.id, t: foot.u } }, false));
        }
    }
    for vertex in &solid.vertices {
        feet.push((
            BoundaryFoot {
                point: vertex.point,
                distance: vertex.point.sub(point).length(),
                place: BoundaryPlace::Vertex { vertex: vertex.id },
            },
            false,
        ));
    }
    Ok(feet)
}

/// The boundary point nearest `point`, on the solid's exact geometry, and
/// where it lies. `tolerance` (mm) is the band a face foot may stand past its
/// trim and still count as inside it. `None` for a solid with no boundary.
/// A face foot on its trim's boundary band gives way to an edge or vertex foot
/// within `tolerance` of it: the point lies on that edge (or vertex) within the
/// tolerance, where the faces' normals decide crease or smooth.
pub fn nearest_boundary_point(solid: &BrepSolid, point: Vec3, tolerance: f64) -> Result<Option<BoundaryFoot>, KernelRefusal> {
    let feet = boundary_feet(solid, point, tolerance)?;
    let Some(&(best, on_band)) = feet.iter().min_by(|a, b| a.0.distance.total_cmp(&b.0.distance)) else { return Ok(None) };
    if on_band {
        let edge_or_vertex = feet
            .iter()
            .filter(|(foot, _)| !matches!(foot.place, BoundaryPlace::Face { .. }))
            .filter(|(foot, _)| foot.point.sub(best.point).length() <= tolerance)
            .min_by(|a, b| a.0.distance.total_cmp(&b.0.distance));
        if let Some(&(foot, _)) = edge_or_vertex {
            return Ok(Some(foot));
        }
    }
    Ok(Some(best))
}

/// The point of the solid nearest each point: the point itself where the
/// solid holds it (inside or on its boundary, at `tolerance`), otherwise its
/// nearest boundary point.
pub fn solid_closest_points(solid: &BrepSolid, points: &[Vec3], tolerance: f64) -> Result<Vec<Vec3>, KernelRefusal> {
    points
        .iter()
        .map(|&point| {
            let class = classify_point(point, solid, tolerance).map_err(|e| refusal("closest_classify", e))?.class;
            if class != PointClass::Out {
                return Ok(point);
            }
            nearest_boundary_point(solid, point, tolerance)?
                .map(|foot| foot.point)
                .ok_or_else(|| refusal("closest_empty", "the solid has no boundary".into()))
        })
        .collect()
}

fn face_by_id(solid: &BrepSolid, id: u64) -> Option<&FaceRecord> {
    solid.shells.iter().flat_map(|shell| &shell.faces).find(|face| face.id == id)
}

/// The outward unit normal of `face` at `uv`. At a pole (a side of the patch
/// collapsed to a point) the surface has no normal of its own, and it is read
/// a millionth of the way in, where it has one (as the face thicken does).
fn outward_normal(face: &FaceRecord, uv: [f64; 2]) -> Option<Vec3> {
    let surface = &face.surface;
    let normal = match surface.normal(uv[0], uv[1]) {
        Ok(normal) => normal,
        Err(_) => pole_normal(surface, uv)?,
    };
    Some(if face.same_sense { normal } else { normal.scale(-1.0) })
}

/// The normal at a degenerate point (a pole, an apex), read a millionth inside
/// along both parameters at five positions round it: their common normal when
/// they agree (a ball's pole), `None` when they do not (a cone's apex has no
/// tangent plane).
fn pole_normal(surface: &crate::NurbsSurface, uv: [f64; 2]) -> Option<Vec3> {
    let [u0, u1] = surface.domain_u().ok()?;
    let [v0, v1] = surface.domain_v().ok()?;
    let inward = |at: f64, low: f64, high: f64| at + (0.5 * (low + high) - at) * 1e-6;
    let (u, v) = (inward(uv[0], u0, u1), inward(uv[1], v0, v1));
    let first = surface.normal(u, v).ok()?;
    let round = [0.2, 0.4, 0.6, 0.8].map(|f| [(u0 + (u1 - u0) * f, v), (u, v0 + (v1 - v0) * f)]);
    for (ru, rv) in round.into_iter().flatten() {
        // Only the positions that stay at the degenerate point: those whose
        // 3D point is the same as the pole's.
        let (Ok(at), Ok(pole)) = (surface.evaluate(ru, rv), surface.evaluate(u, v)) else { continue };
        if at.sub(pole).length() > 1e-6 * (1.0 + pole.length()) {
            continue;
        }
        let normal = surface.normal(ru, rv).ok()?;
        if normal.dot(first) < POLE_AGREEMENT {
            return None;
        }
    }
    Some(first)
}

/// Two normals read a millionth from one degenerate point agree when their
/// cosine is at least this (a ball's pole reads within about 1e-6 rad).
const POLE_AGREEMENT: f64 = 0.999_999;

/// Two outward normals within this angle (radians) are one tangent plane: an
/// edge between them is smooth, not a crease.
const SMOOTH_ANGLE: f64 = 1e-6;

/// The outward unit normals the boundary has at `point`, one per face whose
/// trim holds the point within `tolerance`; empty off the boundary.
fn normals_at(solid: &BrepSolid, point: Vec3, tolerance: f64) -> Result<Vec<Vec3>, KernelRefusal> {
    let mut normals = Vec::new();
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        let foot = project_point_to_surface(&face.surface, point).map_err(|e| refusal("normal_face_foot", e))?;
        if foot.distance <= tolerance && inside_trim(face, [foot.u, foot.v], tolerance)? {
            if let Some(normal) = outward_normal(face, [foot.u, foot.v]) {
                normals.push(normal);
            }
        }
    }
    Ok(normals)
}

/// The outward unit normal at each point of the solid's boundary (within
/// `tolerance`, mm), where the boundary has a tangent plane there: `None` at a
/// crease, where faces meet at an angle, or off the boundary. A point on a
/// smooth edge, where the faces meet tangentially, has the faces' common
/// normal.
pub fn solid_outward_normals(solid: &BrepSolid, points: &[Vec3], tolerance: f64) -> Result<Vec<Option<Vec3>>, KernelRefusal> {
    points
        .iter()
        .map(|&point| {
            let normals = normals_at(solid, point, tolerance)?;
            let Some(&first) = normals.first() else { return Ok(None) };
            let cos = SMOOTH_ANGLE.cos();
            Ok(normals.iter().all(|normal| normal.dot(first) >= cos).then_some(first))
        })
        .collect()
}

/// The principal curvatures `[k1, k2]`, `k1 >= k2`, at the boundary point
/// nearest each point: positive where the boundary bends away from its
/// outward normal, as on a ball (`1 / r` both ways); `None` where that point
/// lies on a crease. A nearest point on an edge or at a vertex is a crease
/// unless every face there shares one tangent plane, and then the curvatures
/// are read on the first of them.
pub fn solid_principal_curvatures(solid: &BrepSolid, points: &[Vec3], tolerance: f64) -> Result<Vec<Option<[f64; 2]>>, KernelRefusal> {
    points
        .iter()
        .map(|&point| {
            let Some(foot) = nearest_boundary_point(solid, point, tolerance)? else { return Ok(None) };
            // The face the curvatures are read on: the foot's own, or on an edge
            // or a vertex the faces there when they share one tangent plane.
            let (face, uv) = match foot.place {
                BoundaryPlace::Face { face, uv } => match face_by_id(solid, face) {
                    Some(record) => (record, uv),
                    None => return Ok(None),
                },
                _ => {
                    let normals = normals_at(solid, foot.point, tolerance)?;
                    let Some(&first) = normals.first() else { return Ok(None) };
                    if !normals.iter().all(|normal| normal.dot(first) >= SMOOTH_ANGLE.cos()) {
                        return Ok(None);
                    }
                    let mut found = None;
                    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
                        let at = project_point_to_surface(&face.surface, foot.point).map_err(|e| refusal("curvature_face_foot", e))?;
                        if at.distance <= tolerance && inside_trim(face, [at.u, at.v], tolerance)? {
                            found = Some((face, [at.u, at.v]));
                            break;
                        }
                    }
                    match found {
                        Some(found) => found,
                        None => return Ok(None),
                    }
                }
            };
            // At a pole the parametrization is degenerate. A ball's pole has its
            // ball's curvature, signed against the surface's own normal there
            // (the kernel's convention: -1/r for a normal away from the centre);
            // any other pole is read a millionth inside, as the normal is.
            let surface = &face.surface;
            let curvatures = surface.principal_curvatures(uv[0], uv[1]).or_else(|error| {
                if let Some(crate::AnalyticSurface::Sphere { frame, radius }) = surface.analytic() {
                    let own = outward_normal(face, uv).map(|n| if face.same_sense { n } else { n.scale(-1.0) });
                    let away = own.is_some_and(|n| n.dot(foot.point.sub(frame.origin)) > 0.0);
                    let k = if away { -1.0 / radius } else { 1.0 / radius };
                    return Ok::<(f64, f64), String>((k, k));
                }
                let _ = error;
                pole_normal(surface, uv).ok_or_else(|| "a degenerate point with no tangent plane".to_string())?;
                let ([u0, u1], [v0, v1]) = (surface.domain_u()?, surface.domain_v()?);
                let inward = |at: f64, low: f64, high: f64| at + (0.5 * (low + high) - at) * 1e-6;
                surface.principal_curvatures(inward(uv[0], u0, u1), inward(uv[1], v0, v1))
            });
            let Ok((kappa_min, kappa_max)) = curvatures else { return Ok(None) };
            // The surface's curvatures are signed against its own normal (an
            // outward-facing cylinder reads -1/r); the protocol's are positive
            // where the boundary bends away from its OUTWARD normal.
            let sign = if face.same_sense { -1.0 } else { 1.0 };
            let (a, b) = (sign * kappa_min, sign * kappa_max);
            Ok(Some([a.max(b), a.min(b)]))
        })
        .collect()
}

/// Samples per knot span along an edge from which its extremes along each
/// axis are sought, and at most this many along the whole edge.
const BOX_EDGE_SAMPLES_PER_SPAN: usize = 8;
const BOX_EDGE_SAMPLES_MAX: usize = 256;
/// Samples per knot span each way over a face's trim box, at most this many
/// each way and at least the least (a small trim on a large surface).
const BOX_FACE_SAMPLES_PER_SPAN: usize = 4;
const BOX_FACE_SAMPLES_MAX: usize = 48;
const BOX_FACE_SAMPLES_LEAST: usize = 8;

/// Parameters from `low` to `high`: the knots between them and `per` steps
/// across each knot span; fewer steps per span when that would pass `cap`,
/// but never fewer than one, so that every knot stays a sample (a feature of
/// the spline lies across its knots); `least` even steps when there would be
/// fewer.
fn span_samples(knots: &[f64], low: f64, high: f64, per: usize, cap: usize, least: usize) -> Vec<f64> {
    let mut breaks = vec![low];
    for &knot in knots {
        if knot > low && knot < high && knot > *breaks.last().unwrap() {
            breaks.push(knot);
        }
    }
    breaks.push(high);
    let spans = breaks.len() - 1;
    if spans * per < least {
        return (0..=least).map(|i| low + (high - low) * i as f64 / least as f64).collect();
    }
    let per = if spans * per > cap { (cap / spans).max(1) } else { per };
    let mut samples = Vec::with_capacity((breaks.len() - 1) * per + 1);
    for pair in breaks.windows(2) {
        for i in 0..per {
            samples.push(pair[0] + (pair[1] - pair[0]) * i as f64 / per as f64);
        }
    }
    samples.push(high);
    samples
}

/// The low and high corners of a box growing round the points it is given.
struct GrowingBox(Option<[Vec3; 2]>);

impl GrowingBox {
    fn take(&mut self, point: Vec3) {
        if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite()) {
            return;
        }
        let [low, high] = self.0.get_or_insert([point, point]);
        *low = Vec3::new(low.x.min(point.x), low.y.min(point.y), low.z.min(point.z));
        *high = Vec3::new(high.x.max(point.x), high.y.max(point.y), high.z.max(point.z));
    }
}

fn axis_of(point: Vec3, axis: usize) -> f64 {
    [point.x, point.y, point.z][axis]
}

/// The parameter in `[a, b]` where `f` is greatest, `f` having one peak
/// there: golden-section search down to the parameter's own precision.
fn golden_peak(f: impl Fn(f64) -> f64, mut a: f64, mut b: f64) -> f64 {
    const RATIO: f64 = 0.618_033_988_749_894_9;
    let mut c = b - RATIO * (b - a);
    let mut d = a + RATIO * (b - a);
    let (mut fc, mut fd) = (f(c), f(d));
    for _ in 0..200 {
        if (b - a).abs() <= 4.0 * f64::EPSILON * (a.abs() + b.abs()).max(f64::MIN_POSITIVE) {
            break;
        }
        if fc >= fd {
            (b, d, fd) = (d, c, fc);
            c = b - RATIO * (b - a);
            fc = f(c);
        } else {
            (a, c, fc) = (c, d, fd);
            d = a + RATIO * (b - a);
            fd = f(d);
        }
    }
    if fc >= fd { c } else { d }
}

/// The edge's points where a coordinate is greatest or least along it: its
/// samples, and each sampled peak of `±x`, `±y`, `±z` sharpened by
/// golden-section search between the samples beside it (an end's peak
/// between the end and its neighbour, which may lie inside the edge).
fn edge_extremes(edge: &crate::topology::EdgeRecord, into: &mut GrowingBox) {
    let curve = &edge.curve;
    let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
    let ts = span_samples(&curve.knots, low, high, BOX_EDGE_SAMPLES_PER_SPAN, BOX_EDGE_SAMPLES_MAX, BOX_EDGE_SAMPLES_PER_SPAN);
    let points: Vec<Option<Vec3>> = ts.iter().map(|&t| curve.evaluate(t).ok()).collect();
    points.iter().flatten().for_each(|&p| into.take(p));
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            let value = |i: usize| points[i].map(|p| sign * axis_of(p, axis));
            let last = ts.len().saturating_sub(1);
            for i in 0..ts.len() {
                let (before, after) = (i.saturating_sub(1), (i + 1).min(last));
                let (Some(here), Some(b), Some(a)) = (value(i), value(before), value(after)) else { continue };
                if here >= b && here >= a && before < after {
                    let f = |t: f64| curve.evaluate(t).map_or(f64::NEG_INFINITY, |p| sign * axis_of(p, axis));
                    let t = golden_peak(f, ts[before], ts[after]);
                    if let Ok(p) = curve.evaluate(t) {
                        into.take(p);
                    }
                }
            }
        }
    }
}

/// From `seed`, the point of the surface where `sign` times coordinate `axis`
/// is greatest near it: damped Newton ascent on the coordinate over the
/// parameters, kept inside the domain. A ridge (a line of equal peaks, as
/// along a cylinder's side) has a flat direction, which the damping steps over.
fn surface_peak(surface: &crate::NurbsSurface, seed: [f64; 2], axis: usize, sign: f64, domain: [[f64; 2]; 2]) -> Option<[f64; 2]> {
    let clamp = |uv: [f64; 2]| [uv[0].clamp(domain[0][0], domain[0][1]), uv[1].clamp(domain[1][0], domain[1][1])];
    let value = |uv: [f64; 2]| surface.evaluate(uv[0], uv[1]).ok().map(|p| sign * axis_of(p, axis));
    let mut uv = seed;
    let mut here = value(uv)?;
    let span = (domain[0][1] - domain[0][0]).abs().max((domain[1][1] - domain[1][0]).abs());
    for _ in 0..60 {
        let d = surface.derivatives(uv[0], uv[1], 2).ok()?;
        let g = [sign * axis_of(d[1][0], axis), sign * axis_of(d[0][1], axis)];
        let h = [[sign * axis_of(d[2][0], axis), sign * axis_of(d[1][1], axis)], [sign * axis_of(d[1][1], axis), sign * axis_of(d[0][2], axis)]];
        let scale = g[0].abs().max(g[1].abs()).max(h[0][0].abs()).max(h[1][1].abs()).max(h[0][1].abs());
        // Relative tests only: the parameters' own scale is arbitrary.
        if scale == 0.0 || g[0].abs().max(g[1].abs()) <= 1e-15 * scale {
            break;
        }
        // The Hessian shifted down until it is negative definite, so the step
        // climbs: its largest eigenvalue, plus a sliver of the scale.
        let mean = 0.5 * (h[0][0] + h[1][1]);
        let largest = mean + (0.25 * (h[0][0] - h[1][1]).powi(2) + h[0][1].powi(2)).sqrt();
        let mut shift = largest.max(0.0) + 1e-9 * scale;
        let mut moved = false;
        for _ in 0..40 {
            let a = [[h[0][0] - shift, h[0][1]], [h[0][1], h[1][1] - shift]];
            let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
            if det == 0.0 || !det.is_finite() {
                shift = shift * 4.0 + scale;
                continue;
            }
            let step = [-(a[1][1] * g[0] - a[0][1] * g[1]) / det, -(a[0][0] * g[1] - a[1][0] * g[0]) / det];
            let next = clamp([uv[0] + step[0], uv[1] + step[1]]);
            match value(next) {
                Some(there) if there >= here => {
                    let small = (next[0] - uv[0]).abs().max((next[1] - uv[1]).abs()) <= 1e-15 * span;
                    uv = next;
                    here = there;
                    moved = !small;
                    break;
                }
                _ => shift = shift * 4.0 + scale,
            }
        }
        if !moved {
            break;
        }
    }
    Some(uv)
}

/// The box in a face's parameters holding its trim: the control points of its
/// loops' pcurves (a pcurve lies in their hull), within the surface's domain;
/// the whole domain where the face has no loops or they fall outside it.
fn trim_box(face: &FaceRecord, du: [f64; 2], dv: [f64; 2]) -> ([f64; 2], [f64; 2]) {
    let mut bounds: Option<[f64; 4]> = None;
    for coedge in face.loops.iter().flat_map(|l| &l.coedges) {
        for c in &coedge.pcurve.control_points {
            if c.w == 0.0 {
                continue;
            }
            let (u, v) = (c.x / c.w, c.y / c.w);
            let b = bounds.get_or_insert([u, u, v, v]);
            *b = [b[0].min(u), b[1].max(u), b[2].min(v), b[3].max(v)];
        }
    }
    match bounds {
        Some([u0, u1, v0, v1]) => {
            let u = [u0.max(du[0]), u1.min(du[1])];
            let v = [v0.max(dv[0]), v1.min(dv[1])];
            if u[0] < u[1] && v[0] < v[1] { (u, v) } else { (du, dv) }
        }
        None => (du, dv),
    }
}

/// The face's points where a coordinate is greatest or least on its trimmed
/// region away from its edges: a grid over its trim's box ([`trim_box`]), each grid
/// point inside the trim that is a peak of `±x`, `±y`, `±z` among its grid
/// neighbours climbed to the surface's peak ([`surface_peak`]), kept when the
/// peak lies inside the trim (its band of `tolerance` included). A peak the
/// climb takes outside the trim lies, on the trimmed face, on its boundary,
/// which the edges answer.
fn face_extremes(face: &FaceRecord, tolerance: f64, into: &mut GrowingBox) -> Result<(), KernelRefusal> {
    let surface = &face.surface;
    let (Ok(du), Ok(dv)) = (surface.domain_u(), surface.domain_v()) else { return Ok(()) };
    let (bu, bv) = trim_box(face, du, dv);
    let us = span_samples(&surface.knots_u, bu[0], bu[1], BOX_FACE_SAMPLES_PER_SPAN, BOX_FACE_SAMPLES_MAX, BOX_FACE_SAMPLES_LEAST);
    let vs = span_samples(&surface.knots_v, bv[0], bv[1], BOX_FACE_SAMPLES_PER_SPAN, BOX_FACE_SAMPLES_MAX, BOX_FACE_SAMPLES_LEAST);
    let mut grid: Vec<Vec<Option<Vec3>>> = vec![vec![None; vs.len()]; us.len()];
    for (i, &u) in us.iter().enumerate() {
        for (j, &v) in vs.iter().enumerate() {
            if trim_class(face, [u, v], tolerance)? != PolygonClass::Outside {
                grid[i][j] = surface.evaluate(u, v).ok();
            }
        }
    }
    grid.iter().flatten().flatten().for_each(|&p| into.take(p));
    for axis in 0..3 {
        for sign in [1.0, -1.0] {
            for i in 0..us.len() {
                for j in 0..vs.len() {
                    let Some(here) = grid[i][j].map(|p| sign * axis_of(p, axis)) else { continue };
                    let mut peak = true;
                    'neighbours: for a in i.saturating_sub(1)..(i + 2).min(us.len()) {
                        for b in j.saturating_sub(1)..(j + 2).min(vs.len()) {
                            if let Some(there) = grid[a][b] {
                                if sign * axis_of(there, axis) > here {
                                    peak = false;
                                    break 'neighbours;
                                }
                            }
                        }
                    }
                    if !peak {
                        continue;
                    }
                    let Some(uv) = surface_peak(surface, [us[i], vs[j]], axis, sign, [du, dv]) else { continue };
                    if trim_class(face, uv, tolerance)? != PolygonClass::Outside {
                        if let Ok(p) = surface.evaluate(uv[0], uv[1]) {
                            into.take(p);
                        }
                    }
                }
            }
        }
    }
    Ok(())
}

/// The least box with sides square to the axes that holds the solid, as its
/// low and high corners, on its exact geometry (not its control nets): the
/// box round its vertices, the points along each edge where a coordinate is
/// greatest or least, and the points inside each face's trim where one is (a
/// ball's or a ring's side). `tolerance` (mm) is the band a face's point may
/// stand past its trim and still count as on the face.
pub fn solid_least_box(solid: &BrepSolid, tolerance: f64) -> Result<[Vec3; 2], KernelRefusal> {
    let mut boxed = GrowingBox(None);
    solid.vertices.iter().for_each(|vertex| boxed.take(vertex.point));
    for edge in solid.edges.iter().filter(|edge| !edge.degenerate) {
        edge_extremes(edge, &mut boxed);
    }
    for face in solid.shells.iter().flat_map(|shell| &shell.faces) {
        face_extremes(face, tolerance, &mut boxed)?;
    }
    boxed.0.ok_or_else(|| refusal("least_box_empty", "the solid has no boundary".into()))
}

/// The distance from `point` to the boundary of `solid`, and the unit vector
/// from its foot to the point (`None` where the point lies on it).
fn boundary_distance(solid: &BrepSolid, point: Vec3, tolerance: f64) -> Result<Option<(f64, Option<Vec3>)>, KernelRefusal> {
    Ok(nearest_boundary_point(solid, point, tolerance)?.map(|foot| {
        let away = point.sub(foot.point);
        let length = away.length();
        (foot.distance, (length > 0.0).then(|| away.scale(1.0 / length)))
    }))
}

/// The parameter in `[a, b]` where `f` is least, `f` having one dip there.
fn golden_dip(f: impl Fn(f64) -> f64, a: f64, b: f64) -> f64 {
    golden_peak(|t| -f(t), a, b)
}

/// From `seed`, the point of `face` nearest `other` near it: Newton on the
/// distance over the face's parameters, its gradient `(S_u·n, S_v·n)` from the
/// unit vector `n` the other's foot points along (the distance to a set
/// changes along `n` alone), its Hessian by differences of that gradient,
/// damped so each step goes downhill and kept inside the domain. It stops
/// where the gradient across the whole domain changes the distance by no more
/// than `flat`, the round-off of the distances themselves.
fn surface_dip(face: &FaceRecord, other: &BrepSolid, seed: [f64; 2], domain: [[f64; 2]; 2], tolerance: f64, flat: f64) -> Result<Option<[f64; 2]>, KernelRefusal> {
    let surface = &face.surface;
    let clamp = |uv: [f64; 2]| [uv[0].clamp(domain[0][0], domain[0][1]), uv[1].clamp(domain[1][0], domain[1][1])];
    let span = (domain[0][1] - domain[0][0]).abs().max((domain[1][1] - domain[1][0]).abs());
    let at = |uv: [f64; 2]| -> Result<Option<(f64, [f64; 2])>, KernelRefusal> {
        let Ok((p, su, sv)) = surface.deriv1(uv[0], uv[1]) else { return Ok(None) };
        Ok(boundary_distance(other, p, tolerance)?.map(|(d, n)| (d, n.map_or([0.0, 0.0], |n| [su.dot(n), sv.dot(n)]))))
    };
    let mut uv = seed;
    let Some((mut here, mut g)) = at(uv)? else { return Ok(None) };
    let h = 1e-6 * span;
    for _ in 0..40 {
        if here == 0.0 || g[0].hypot(g[1]) * span <= flat || h == 0.0 {
            break;
        }
        // The gradient's differences, one each way, for the Hessian.
        let (Some((_, gu)), Some((_, gv))) = (at(clamp([uv[0] + h, uv[1]]))?, at(clamp([uv[0], uv[1] + h]))?) else { break };
        let (du, dv) = (clamp([uv[0] + h, uv[1]])[0] - uv[0], clamp([uv[0], uv[1] + h])[1] - uv[1]);
        if du == 0.0 || dv == 0.0 {
            break;
        }
        let hxx = (gu[0] - g[0]) / du;
        let hyy = (gv[1] - g[1]) / dv;
        let hxy = 0.5 * ((gu[1] - g[1]) / du + (gv[0] - g[0]) / dv);
        let scale = hxx.abs().max(hyy.abs()).max(hxy.abs()).max(g[0].abs()).max(g[1].abs());
        let mean = 0.5 * (hxx + hyy);
        let least = mean - (0.25 * (hxx - hyy).powi(2) + hxy * hxy).sqrt();
        // Shifted up until positive definite, so the step goes down.
        let mut shift = (-least).max(0.0) + 1e-9 * scale;
        let mut moved = false;
        for _ in 0..40 {
            let a = [[hxx + shift, hxy], [hxy, hyy + shift]];
            let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
            if det == 0.0 || !det.is_finite() {
                shift = shift * 4.0 + scale;
                continue;
            }
            let step = [-(a[1][1] * g[0] - a[0][1] * g[1]) / det, -(a[0][0] * g[1] - a[1][0] * g[0]) / det];
            let next = clamp([uv[0] + step[0], uv[1] + step[1]]);
            match at(next)? {
                Some((there, gn)) if there <= here => {
                    let small = (next[0] - uv[0]).abs().max((next[1] - uv[1]).abs()) <= 1e-15 * span;
                    (uv, here, g) = (next, there, gn);
                    moved = !small;
                    break;
                }
                _ => shift = shift * 4.0 + scale,
            }
        }
        if !moved {
            break;
        }
    }
    Ok(Some(uv))
}

/// The least distance between the boundaries of `a` and `b` (0 where they
/// touch or cross), on their exact geometry: the lesser of the searches from
/// each side ([`gap_from`]), once one reads more than round-off. `tolerance`
/// (mm) is the trim band.
pub fn solid_gap(a: &BrepSolid, b: &BrepSolid, tolerance: f64) -> Result<f64, KernelRefusal> {
    let flat = 1e-11 * crate::solid_scale(a).max(crate::solid_scale(b));
    let there = gap_from(a, b, tolerance, flat)?;
    if there <= flat {
        return Ok(there);
    }
    Ok(there.min(gap_from(b, a, tolerance, flat)?))
}

/// The least, over the points of `a`'s boundary, of the distance to `b`'s
/// ([`nearest_boundary_point`]), sought at `a`'s vertices, along its edges
/// (sampled per knot span, each dip sharpened by golden-section search) and
/// inside its faces' trims (grid seeds, each dip refined by Newton, kept
/// inside the trim); `b`'s vertices are read against `a` as well. A sample
/// whose neighbours all lie within `flat` of it sits on a level stretch
/// (faces or edges parallel to the other's), which it already reads; only a
/// dip below its neighbours is sharpened, and a feature of `b` between such
/// samples is the search from `b`'s side.
fn gap_from(a: &BrepSolid, b: &BrepSolid, tolerance: f64, flat: f64) -> Result<f64, KernelRefusal> {
    let mut least = f64::INFINITY;
    let take = |least: &mut f64, d: Option<(f64, Option<Vec3>)>| {
        if let Some((d, _)) = d {
            *least = least.min(d);
        }
    };
    for vertex in &a.vertices {
        take(&mut least, boundary_distance(b, vertex.point, tolerance)?);
    }
    for vertex in &b.vertices {
        take(&mut least, boundary_distance(a, vertex.point, tolerance)?);
    }
    for edge in a.edges.iter().filter(|edge| !edge.degenerate) {
        if least <= flat {
            return Ok(least);
        }
        let curve = &edge.curve;
        let (low, high) = (edge.t0.min(edge.t1), edge.t0.max(edge.t1));
        let ts = span_samples(&curve.knots, low, high, BOX_EDGE_SAMPLES_PER_SPAN, BOX_EDGE_SAMPLES_MAX, BOX_EDGE_SAMPLES_PER_SPAN);
        let mut values = Vec::with_capacity(ts.len());
        for &t in &ts {
            let d = match curve.evaluate(t) {
                Ok(p) => boundary_distance(b, p, tolerance)?.map(|(d, _)| d),
                Err(_) => None,
            };
            values.push(d);
        }
        values.iter().flatten().for_each(|&d| least = least.min(d));
        let last = ts.len().saturating_sub(1);
        for i in 0..ts.len() {
            let (before, after) = (i.saturating_sub(1), (i + 1).min(last));
            let (Some(here), Some(x), Some(y)) = (values[i], values[before], values[after]) else { continue };
            if here <= x && here <= y && before < after && here > 0.0 && x.max(y) - here > flat {
                let f = |t: f64| match curve.evaluate(t) {
                    Ok(p) => nearest_boundary_point(b, p, tolerance).ok().flatten().map_or(f64::INFINITY, |foot| foot.distance),
                    Err(_) => f64::INFINITY,
                };
                let t = golden_dip(f, ts[before], ts[after]);
                least = least.min(f(t));
            }
        }
    }
    for face in a.shells.iter().flat_map(|shell| &shell.faces) {
        if least <= flat {
            return Ok(least);
        }
        let surface = &face.surface;
        let (Ok(du), Ok(dv)) = (surface.domain_u(), surface.domain_v()) else { continue };
        let (bu, bv) = trim_box(face, du, dv);
        let us = span_samples(&surface.knots_u, bu[0], bu[1], BOX_FACE_SAMPLES_PER_SPAN, BOX_FACE_SAMPLES_MAX, BOX_FACE_SAMPLES_LEAST);
        let vs = span_samples(&surface.knots_v, bv[0], bv[1], BOX_FACE_SAMPLES_PER_SPAN, BOX_FACE_SAMPLES_MAX, BOX_FACE_SAMPLES_LEAST);
        let mut grid: Vec<Vec<Option<f64>>> = vec![vec![None; vs.len()]; us.len()];
        for (i, &u) in us.iter().enumerate() {
            for (j, &v) in vs.iter().enumerate() {
                if trim_class(face, [u, v], tolerance)? == PolygonClass::Outside {
                    continue;
                }
                if let Ok(p) = surface.evaluate(u, v) {
                    grid[i][j] = boundary_distance(b, p, tolerance)?.map(|(d, _)| d);
                }
            }
        }
        grid.iter().flatten().flatten().for_each(|&d| least = least.min(d));
        for i in 0..us.len() {
            for j in 0..vs.len() {
                let Some(here) = grid[i][j] else { continue };
                if here == 0.0 {
                    continue;
                }
                let around: Vec<f64> = (i.saturating_sub(1)..(i + 2).min(us.len()))
                    .flat_map(|x| (j.saturating_sub(1)..(j + 2).min(vs.len())).map(move |y| (x, y)))
                    .filter_map(|(x, y)| grid[x][y])
                    .collect();
                let dip = around.iter().all(|&there| there >= here) && around.iter().any(|&there| there - here > flat);
                if !dip {
                    continue;
                }
                let Some(uv) = surface_dip(face, b, [us[i], vs[j]], [du, dv], tolerance, flat)? else { continue };
                if trim_class(face, uv, tolerance)? == PolygonClass::Outside {
                    continue;
                }
                if let Ok(p) = surface.evaluate(uv[0], uv[1]) {
                    take(&mut least, boundary_distance(b, p, tolerance)?);
                }
            }
        }
    }
    if least.is_finite() { Ok(least) } else { Err(refusal("gap_empty", "a solid has no boundary".into())) }
}
