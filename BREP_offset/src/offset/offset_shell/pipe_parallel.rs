//! `BREP_OFFSET_PIPE_PARALLEL` (default on; `=0` restores the refusal): the
//! rounded GROWTH of a CONVEX body whose faces are planes, cylinders and
//! spheres and whose every sharp edge is a closed loop (no corners), built
//! directly: each face moved out by `d` under the exact affine map of its own
//! surface (a plane translated, a cylinder or sphere scaled about its axis or
//! centre), and each edge rounded by the PIPE of radius `d` about the SOURCE
//! edge itself, between the two moved faces.
//!
//! The rolling ball's centre on a convex edge of the grown body is `d` from
//! both grown faces, so it runs on the source edge; the round is the pipe
//! `c(t) + d f(s)` with `f` the arc from one face's unit normal `a(t)` to the
//! other's `b(t)`. With `k = a.b`, `f(s) = ((1 - s^2) a + 2 s (1 + k s) b) /
//! (1 + 2 k s + s^2)`: the rational quadratic with weights `(1, 1 + k, 2 + 2k)`
//! and middle point `(a + b) / (1 + k)`. Where `c` is rational of degree `p`
//! with weight `W`, and `a`, `b` are affine in `c` (true for planes, cylinders
//! and spheres), the pipe's homogeneous net times `W^3` is polynomial of
//! degree `3p` in `t` on every span of `c`: the pipe is an exact NURBS, read
//! here by interpolating each span at `3p + 1` stations.
//!
//! Every pcurve is exact as well: a moved face keeps its source surface's
//! parametrisation, so the source pcurves serve unchanged, and the pipe's
//! rails and seam are its own iso lines.
//!
//! Cause (brep-bench: a cylinder cut by a slanted plane, grown rounded):
//! refused at 45 degrees from d 1.3 and at 60 degrees at every distance. The
//! general blend rounding the grown sharp body's elliptic edge did not settle
//! ("rails stand 1.88e-6 off their carriers (bar 1e-6) after 190 stations and
//! 6 local rounds"; at 60 degrees its start "tangency Newton did not
//! converge"), although the round there is this exact pipe.
//!
//! It runs only where every other rounded-growth lane refused.
use super::*;
use super::carrier_rebuild::{junction_mate, junction_stations, JunctionStation};
use crate::{AnalyticSurface, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec4};

pub(super) fn pipe_parallel_on() -> bool {
    switch!("BREP_OFFSET_PIPE_PARALLEL", true)
}

/// `BREP_OFFSET_PIPE_CORNERS` (default on; `=0` restores the decline): edges
/// that end in CORNERS, vertices where three faces or more meet at convex
/// edges. Each pipe then ends in two arcs (its own iso lines) and each corner
/// takes the sphere patch of radius `d` about the source vertex, bounded by
/// the arcs of the pipes that end there (the rolling ball's centre stays on
/// the vertex while it turns from one edge to the next).
///
/// Cause (brep-bench probe): a cylinder cut by two planes into a roof, grown
/// rounded, was refused at every distance (the assembly of the grown body
/// left the ridge open), and the lane declined every edge with an end.
/// `BREP_OFFSET_PIPE_CONES` (default on; `=0` restores the decline): cone
/// faces, material inside, apex off the face. A cone's offset is the same
/// cone slid along its normals (each row of its net scaled about the axis and
/// shifted along it, exactly); its normal along an edge is a ratio of affine
/// forms in the point, so the pipe stays an exact rational, of degree 5p.
///
/// Cause (brep-bench probe): a frustum cut by a slanted plane, grown rounded,
/// was refused at 60 degrees at every distance and at 45 degrees from d 2.
pub(super) fn pipe_cones_on() -> bool {
    switch!("BREP_OFFSET_PIPE_CONES", true)
}

/// `BREP_OFFSET_PIPE_SMOOTH_JOINS` (default on; `=0` restores): an edge whose
/// two faces meet tangentially all along (no station of `junction_stations`
/// left, and the two faces' movers carry nine stations of it to one point
/// within 1e-9) needs no pipe. Both faces move it to the same curve, so it
/// is one moved edge shared by the two moved faces, and its vertices are one
/// moved point. A body rounded everywhere (a box grown once) then grows again
/// by its moved faces alone: a box grown by a then by b is the box grown by
/// a + b, exactly.
fn pipe_smooth_joins_on() -> bool {
    switch!("BREP_OFFSET_PIPE_SMOOTH_JOINS", true)
}

/// `BREP_OFFSET_PIPE_REVOLVED_CYLINDERS` (default on; `=0` restores): a
/// partial revolution whose generatrix keeps one distance from its axis is
/// read as the cylinder it lies on. The rounds this lane writes about a
/// straight edge are quarter turns, which recognition reads as the general
/// `Revolution`, so a body the lane grew was declined when grown again and
/// the general pipeline refused it (brep-conformance chain-offset-twice: a
/// box offset by 0.2, then by 0.6 or more).
fn pipe_revolved_cylinders_on() -> bool {
    switch!("BREP_OFFSET_PIPE_REVOLVED_CYLINDERS", true)
}

/// The one distance of `generatrix` from the axis through `origin` along
/// `axis`, read at its ends and seven interior stations, when every reading
/// agrees to 1e-12 of the radius: the revolved surface then lies on that
/// cylinder.
fn constant_radius(generatrix: &NurbsCurve, origin: Vec3, axis: Vec3) -> Result<Option<f64>, KernelRefusal> {
    let [t0, t1] = generatrix.domain().or_refuse(KernelStage::Refine, "domain")?;
    let Ok(axis) = axis.normalized() else { return Ok(None) };
    let radial = |point: Vec3| {
        let offset = point.sub(origin);
        offset.sub(axis.scale(offset.dot(axis))).length()
    };
    let first = radial(generatrix.evaluate(t0).or_refuse(KernelStage::Refine, "evaluate")?);
    if !(first > 0.0 && first.is_finite()) {
        return Ok(None);
    }
    for k in 1..=8 {
        let t = t0 + (t1 - t0) * k as f64 / 8.0;
        let r = radial(generatrix.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?);
        if (r - first).abs() > 1e-12 * first.max(1.0) {
            return Ok(None);
        }
    }
    Ok(Some(first))
}

pub(super) fn pipe_corners_on() -> bool {
    switch!("BREP_OFFSET_PIPE_CORNERS", true)
}

/// How a face's points move out by `d`, and its outward unit normal there.
#[derive(Clone, Copy)]
enum Mover {
    /// A plane: the constant outward normal.
    Plane { normal: Vec3 },
    /// A cylinder about (origin, axis), radius `radius`, outward `sign`
    /// (+1 when the material is inside it).
    Cylinder { origin: Vec3, axis: Vec3, radius: f64, sign: f64 },
    /// A sphere about `centre`.
    Sphere { centre: Vec3, radius: f64, sign: f64 },
    /// A cone about (origin, axis): radius `rho0` at the origin, growing by
    /// `slope` per unit along the axis; outward `sign` (+1 when the material
    /// is inside it).
    Cone { origin: Vec3, axis: Vec3, rho0: f64, slope: f64, sign: f64 },
}

impl Mover {
    fn normal(&self, x: Vec3) -> Vec3 {
        match *self {
            Mover::Plane { normal } => normal,
            Mover::Cylinder { origin, axis, radius, sign } => {
                let w = x.sub(origin);
                w.sub(axis.scale(w.dot(axis))).scale(sign / radius)
            }
            Mover::Sphere { centre, radius, sign } => x.sub(centre).scale(sign / radius),
            Mover::Cone { .. } => {
                let (numerator, denominator) = self.cone_ratio(x);
                numerator.scale(1.0 / denominator)
            }
        }
    }
    /// A cone's outward normal as `N(x) / D(x)`, both AFFINE in `x`: with
    /// `w = x - origin`, `z = w . axis`, `rho(z) = rho0 + slope z`,
    /// `N = sign (w - z axis - slope rho(z) axis)` and
    /// `D = rho(z) sqrt(1 + slope^2)`. On the cone it is the unit normal; on
    /// a control point it scales the point's radial part by the same factor
    /// as its row's circle, so moving a cone's net by it is exact.
    fn cone_ratio(&self, x: Vec3) -> (Vec3, f64) {
        let Mover::Cone { origin, axis, rho0, slope, sign } = *self else { unreachable!("a cone") };
        let w = x.sub(origin);
        let z = w.dot(axis);
        let rho = rho0 + slope * z;
        let numerator = w.sub(axis.scale(z + slope * rho)).scale(sign);
        (numerator, rho * (1.0 + slope * slope).sqrt())
    }
    /// The denominator of the normal as a ratio of affine forms (1 for a
    /// face whose normal is affine itself).
    fn denominator(&self, x: Vec3) -> f64 {
        match self {
            Mover::Cone { .. } => self.cone_ratio(x).1,
            _ => 1.0,
        }
    }
    fn is_cone(&self) -> bool {
        matches!(self, Mover::Cone { .. })
    }
    fn moved(&self, x: Vec3, distance: f64) -> Vec3 {
        x.add(self.normal(x).scale(distance))
    }
    /// Convex: a plane, or a cylinder or sphere with the material inside it.
    fn convex(&self) -> bool {
        match *self {
            Mover::Plane { .. } => true,
            Mover::Cylinder { sign, .. } | Mover::Sphere { sign, .. } | Mover::Cone { sign, .. } => sign > 0.0,
        }
    }
}

fn moved_point(point: Vec4, mover: &Mover, distance: f64) -> Result<Vec4, KernelRefusal> {
    let cartesian = point.point().or_refuse(KernelStage::Refine, "point")?;
    Ok(Vec4::from_point(mover.moved(cartesian, distance), point.w))
}

fn moved_curve(curve: &NurbsCurve, mover: &Mover, distance: f64) -> Result<NurbsCurve, KernelRefusal> {
    let points = curve.control_points.iter().map(|p| moved_point(*p, mover, distance)).collect::<Result<Vec<_>, _>>()?;
    NurbsCurve::new(curve.degree, curve.knots.clone(), points).or_refuse(KernelStage::Refine, "NurbsCurve::new")
}

fn moved_surface(surface: &NurbsSurface, mover: &Mover, distance: f64) -> Result<NurbsSurface, KernelRefusal> {
    let net = surface
        .control_points
        .iter()
        .map(|row| row.iter().map(|p| moved_point(*p, mover, distance)).collect::<Result<Vec<_>, _>>())
        .collect::<Result<Vec<_>, _>>()?;
    NurbsSurface::new(surface.degree_u, surface.degree_v, surface.knots_u.clone(), surface.knots_v.clone(), net)
        .or_refuse(KernelStage::Refine, "NurbsSurface::new")
}

/// The face's mover, its outward sense read off the face's own normal; `None`
/// for any other surface.
fn mover_of(face: &FaceRecord) -> Result<Option<Mover>, KernelRefusal> {
    let [u0, u1] = face.surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = face.surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let (u, v) = ((u0 + u1) * 0.5, (v0 + v1) * 0.5);
    let at = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
    let outward = super::face_normal(face, u, v)?;
    let mover = match face.surface.analytic() {
        Some(AnalyticSurface::Plane { u_dir, v_dir, .. }) => {
            let Ok(normal) = u_dir.cross(*v_dir).normalized() else { return Ok(None) };
            Mover::Plane { normal: if normal.dot(outward) >= 0.0 { normal } else { normal.scale(-1.0) } }
        }
        Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, .. }) if (rho0 - rho1).abs() <= 1e-12 * rho0.abs().max(1.0) && *rho0 > 0.0 => {
            let probe = Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius: *rho0, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius: *rho0, sign }
        }
        Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) if pipe_cones_on() && *height != 0.0 => {
            let slope = (rho1 - rho0) / height;
            let probe = Mover::Cone { origin: frame.origin, axis: frame.axis, rho0: *rho0, slope, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            let mover = Mover::Cone { origin: frame.origin, axis: frame.axis, rho0: *rho0, slope, sign };
            // The apex must stay off the face: the radius positive at every
            // corner of its domain.
            for (u, v) in [(u0, v0), (u0, v1), (u1, v0), (u1, v1)] {
                let point = face.surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
                if !(mover.denominator(point) > 1e-9 * (1.0 + rho0.abs().max(rho1.abs()))) {
                    return Ok(None);
                }
            }
            mover
        }
        Some(AnalyticSurface::Revolution { frame, generatrix, .. }) if pipe_revolved_cylinders_on() => {
            let Some(radius) = constant_radius(generatrix, frame.origin, frame.axis)? else { return Ok(None) };
            let probe = Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            Mover::Cylinder { origin: frame.origin, axis: frame.axis, radius, sign }
        }
        Some(AnalyticSurface::Sphere { frame, radius }) => {
            let probe = Mover::Sphere { centre: frame.origin, radius: *radius, sign: 1.0 };
            let sign = if probe.normal(at).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
            Mover::Sphere { centre: frame.origin, radius: *radius, sign }
        }
        _ => return Ok(None),
    };
    // The reading must agree with the face's own normal (a unit normal, the
    // right sense) or the face is not what the reading says.
    if (mover.normal(at).dot(outward) - 1.0).abs() > 1e-7 {
        return Ok(None);
    }
    Ok(Some(mover))
}

/// The coefficients of the degree-`n` Bernstein polynomial through `values`
/// at `stations` in [0, 1] (n + 1 of each): Gaussian elimination with partial
/// pivoting on the small collocation system.
fn bernstein_through(stations: &[f64], values: &[[f64; 4]]) -> Option<Vec<[f64; 4]>> {
    let n = stations.len() - 1;
    let binomial = |n: usize, k: usize| (0..k).fold(1.0f64, |acc, i| acc * (n - i) as f64 / (i + 1) as f64);
    let mut matrix: Vec<Vec<f64>> = stations
        .iter()
        .map(|&x| (0..=n).map(|i| binomial(n, i) * x.powi(i as i32) * (1.0 - x).powi((n - i) as i32)).collect())
        .collect();
    let mut rhs: Vec<[f64; 4]> = values.to_vec();
    for column in 0..=n {
        let pivot = (column..=n).max_by(|a, b| matrix[*a][column].abs().total_cmp(&matrix[*b][column].abs()))?;
        if matrix[pivot][column].abs() < 1e-14 {
            return None;
        }
        matrix.swap(column, pivot);
        rhs.swap(column, pivot);
        for row in (column + 1)..=n {
            let factor = matrix[row][column] / matrix[column][column];
            for k in column..=n {
                matrix[row][k] -= factor * matrix[column][k];
            }
            for c in 0..4 {
                rhs[row][c] -= factor * rhs[column][c];
            }
        }
    }
    let mut out = vec![[0.0f64; 4]; n + 1];
    for row in (0..=n).rev() {
        let mut value = rhs[row];
        for k in (row + 1)..=n {
            for c in 0..4 {
                value[c] -= matrix[row][k] * out[k][c];
            }
        }
        for c in 0..4 {
            value[c] /= matrix[row][row];
        }
        out[row] = value;
    }
    Some(out)
}

/// The pipe of radius `distance` about `spine` (t over its domain) from the
/// normal field `first` (s = 0) to `second` (s = 1).
fn pipe_surface(spine: &NurbsCurve, first: &Mover, second: &Mover, distance: f64) -> Result<Option<NurbsSurface>, KernelRefusal> {
    let mut breaks: Vec<f64> = Vec::new();
    for &knot in &spine.knots {
        if breaks.last().is_none_or(|last| knot > *last) {
            breaks.push(knot);
        }
    }
    // The rows times W^3 are polynomial of degree 3p where each normal is
    // affine in the point. A cone's is N/D with both affine: the rows times
    // W (W D_a)^2 (W D_b)^2 are polynomial of degree 5p.
    let cone = first.is_cone() || second.is_cone();
    let degree = if cone { 5 * spine.degree } else { 3 * spine.degree };
    let mut rows: [Vec<Vec4>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    let mut knots_u = Vec::new();
    for (span, pair) in breaks.windows(2).enumerate() {
        let (lo, hi) = (pair[0], pair[1]);
        let stations: Vec<f64> = (0..=degree).map(|j| j as f64 / degree as f64).collect();
        let mut values: [Vec<[f64; 4]>; 3] = [Vec::new(), Vec::new(), Vec::new()];
        for &x in &stations {
            let t = lo + (hi - lo) * x;
            let homogeneous = spine.evaluate_homogeneous(t).or_refuse(KernelStage::Refine, "evaluate_homogeneous")?;
            let weight = homogeneous.w;
            let c = homogeneous.point().or_refuse(KernelStage::Refine, "point")?;
            let (a, b) = (first.normal(c), second.normal(c));
            let k = a.dot(b);
            if !(k > -1.0 + 1e-9) {
                return Ok(None);
            }
            let cube = if cone {
                let (da, db) = (weight * first.denominator(c), weight * second.denominator(c));
                weight * da * da * db * db
            } else {
                weight * weight * weight
            };
            let rows_here = [
                (c.add(a.scale(distance)), 1.0),
                (c.add(a.add(b).scale(distance / (1.0 + k))), 1.0 + k),
                (c.add(b.scale(distance)), 2.0 + 2.0 * k),
            ];
            for (row, (point, w)) in rows_here.iter().enumerate() {
                let total = w * cube;
                values[row].push([point.x * total, point.y * total, point.z * total, total]);
            }
        }
        for row in 0..3 {
            let Some(coefficients) = bernstein_through(&stations, &values[row]) else { return Ok(None) };
            let skip = usize::from(span > 0);
            for coefficient in coefficients.into_iter().skip(skip) {
                rows[row].push(Vec4 { x: coefficient[0], y: coefficient[1], z: coefficient[2], w: coefficient[3] });
            }
        }
        if span == 0 {
            knots_u.extend(std::iter::repeat_n(lo, degree + 1));
        }
        let end_multiplicity = if span + 2 == breaks.len() { degree + 1 } else { degree };
        knots_u.extend(std::iter::repeat_n(hi, end_multiplicity));
    }
    // Net indexed [u = t][v = s].
    let net = (0..rows[0].len()).map(|i| vec![rows[0][i], rows[1][i], rows[2][i]]).collect::<Vec<_>>();
    if net.iter().flatten().any(|p| !(p.w > 0.0)) {
        return Ok(None);
    }
    let (degree, knots_u, net) = if pipe_canonical_on() { canonical(degree, spine.degree, knots_u, net) } else { (degree, knots_u, net) };
    let surface = NurbsSurface::new(degree, 2, knots_u, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], net).or_refuse(KernelStage::Refine, "NurbsSurface::new")?;
    Ok(Some(surface))
}

/// `BREP_OFFSET_PIPE_CANONICAL` (default on; `=0` restores the raw net): a
/// pipe written in the standard form of the surface it is, where it can be
/// exactly. Raw, a round between two planes on a straight edge is a degree-3
/// by rational-quadratic net with end weights 1 and 2 + 2k: the right surface,
/// but not the layout of a cylinder, so the next offset of the grown body
/// cannot read it as one and refused (brep-conformance chain-offset-twice: a
/// box offset by a, then by b).
fn pipe_canonical_on() -> bool {
    switch!("BREP_OFFSET_PIPE_CANONICAL", true)
}

/// The net with, where each holds exactly, (a) the arc across the pipe
/// reparametrised so its end weights are equal (s -> s c / (1 - s + s c),
/// which keeps s = 0 and 1 where they were, so the rails and their pcurves do
/// not move; possible when every row carries the same weight ratio) and (b)
/// each span along the spine reduced to the spine's own degree when the net is
/// that degree's elevation (a straight spine between faces whose normals do
/// not turn along it). Either that does not hold is left as it was.
fn canonical(degree: usize, spine_degree: usize, knots_u: Vec<f64>, mut net: Vec<Vec<Vec4>>) -> (usize, Vec<f64>, Vec<Vec<Vec4>>) {
    let ratio = |row: &Vec<Vec4>| row[0].w / row[2].w;
    let first = ratio(&net[0]);
    let middle = |row: &Vec<Vec4>| row[1].w / (row[0].w * row[2].w).sqrt();
    let uniform = net.iter().all(|row| (ratio(row) / first - 1.0).abs() <= 1e-12 && (middle(row) / middle(&net[0]) - 1.0).abs() <= 1e-12);
    if uniform {
        let c = first.sqrt();
        for row in net.iter_mut() {
            row[1] = scaled(row[1], c);
            row[2] = scaled(row[2], c * c);
        }
    }
    if spine_degree != 1 || degree != 3 {
        return (degree, knots_u, net);
    }
    // Each span of a degree-1 spine elevated to degree 3: P1 = (2 P0 + P3) / 3,
    // P2 = (P0 + 2 P3) / 3, homogeneous.
    let spans = (net.len() - 1) / 3;
    let scale = net.iter().flatten().map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs()).max(p.w.abs())).fold(1.0, f64::max);
    let near = |a: Vec4, b: Vec4| (a.x - b.x).abs().max((a.y - b.y).abs()).max((a.z - b.z).abs()).max((a.w - b.w).abs()) <= 1e-12 * scale;
    let mix = |a: Vec4, b: Vec4, wa: f64, wb: f64| Vec4 { x: wa * a.x + wb * b.x, y: wa * a.y + wb * b.y, z: wa * a.z + wb * b.z, w: wa * a.w + wb * b.w };
    for span in 0..spans {
        for j in 0..3 {
            let (p0, p1, p2, p3) = (net[3 * span][j], net[3 * span + 1][j], net[3 * span + 2][j], net[3 * span + 3][j]);
            if !near(p1, mix(p0, p3, 2.0 / 3.0, 1.0 / 3.0)) || !near(p2, mix(p0, p3, 1.0 / 3.0, 2.0 / 3.0)) {
                return (degree, knots_u, net);
            }
        }
    }
    let reduced: Vec<Vec<Vec4>> = (0..=spans).map(|span| net[3 * span].clone()).collect();
    let mut knots = Vec::with_capacity(spans + 3);
    let mut breaks: Vec<f64> = Vec::new();
    for &knot in &knots_u {
        if breaks.last().is_none_or(|last| knot > *last) {
            breaks.push(knot);
        }
    }
    knots.push(breaks[0]);
    knots.extend(breaks.iter().copied());
    knots.push(breaks[breaks.len() - 1]);
    (1, knots, reduced)
}

/// The same surface with its parameters swapped (u for v).
fn transposed(surface: &NurbsSurface) -> Result<NurbsSurface, KernelRefusal> {
    let rows = surface.control_points.len();
    let columns = surface.control_points[0].len();
    let net = (0..columns).map(|j| (0..rows).map(|i| surface.control_points[i][j]).collect()).collect();
    NurbsSurface::new(surface.degree_v, surface.degree_u, surface.knots_v.clone(), surface.knots_u.clone(), net).or_refuse(KernelStage::Refine, "NurbsSurface::new")
}

/// `p` with its weight and coordinates scaled by `factor` (the same point).
fn scaled(p: Vec4, factor: f64) -> Vec4 {
    Vec4 { x: p.x * factor, y: p.y * factor, z: p.z * factor, w: p.w * factor }
}

/// The lane's answer, or `None` when it declines (the caller's refusal stands).
pub(super) fn pipe_parallel(source: &BrepSolid, distance: f64) -> Option<OffsetShellResultRecord> {
    // The pipeline's own floor: a distance it refuses as too small is not
    // grown here either.
    if !pipe_parallel_on() || !distance.is_finite() || distance <= MINIMUM_DISTANCE {
        return None;
    }
    match pipe_parallel_impl(source, distance) {
        Ok(Some((solid, sources))) => {
            // Each moved face is its source face's offset image; a pipe has
            // no single source face.
            let face_images = sources
                .into_iter()
                .map(|source_face_id| OffsetShellFaceImageRecord { role: OffsetFaceRole::Offset, source_face_id: source_face_id.unwrap_or(0) })
                .collect();
            Some(OffsetShellResultRecord { solid, face_images })
        }
        Ok(None) => None,
        Err(error) => {
            os_debug!("pipe parallel lane declined: {error}");
            None
        }
    }
}


/// The grown solid and, per face in shell order, the source face it images
/// (`None` for a pipe).
fn pipe_parallel_impl(source: &BrepSolid, distance: f64) -> Result<Option<(BrepSolid, Vec<Option<u64>>)>, KernelRefusal> {
    let [shell] = source.shells.as_slice() else { return Ok(None) };
    let faces: Vec<&FaceRecord> = shell.faces.iter().collect();
    let edge_by_id: HashMap<u64, &EdgeRecord> = source.edges.iter().map(|edge| (edge.id, edge)).collect();
    let vertex_by_id: HashMap<u64, Vec3> = source.vertices.iter().map(|vertex| (vertex.id, vertex.point)).collect();
    let mut movers: HashMap<u64, Mover> = HashMap::default();
    for face in &faces {
        let Some(mover) = mover_of(face)? else {
            os_debug!("pipe parallel lane: face {} is not a plane, cylinder or sphere", face.id);
            return Ok(None);
        };
        // Only a CONVEX body: every face convex here and every edge convex
        // below make a closed, connected, locally convex surface, which bounds
        // a convex body, whose parallel body never folds. A concave face (a
        // dimple, a bore) can fold a pipe through its own spine, and
        // `validate` does not see a surface pass through itself (cross review:
        // a cylinder less an off-axis ball grown past the ball's openings).
        if !mover.convex() {
            os_debug!("pipe parallel lane: face {} is concave", face.id);
            return Ok(None);
        }
        movers.insert(face.id, mover);
    }
    // Every edge: a seam (twice in one face) or a closed convex edge between
    // two faces, used once by each, forward by one and backward by the other.
    let mut uses: HashMap<u64, Vec<(u64, bool)>> = HashMap::default();
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            uses.entry(coedge.edge_id).or_default().push((face.id, coedge.forward));
        }
    }
    // (edge, face used forward, face used backward)
    let mut rounds: Vec<(u64, u64, u64)> = Vec::new();
    // (edge, one face, the other): a tangent join, carried by both moved
    // faces as one moved edge and needing no pipe.
    let mut smooth: Vec<(u64, u64, u64)> = Vec::new();
    for (edge_id, edge_uses) in &uses {
        let Some(edge) = edge_by_id.get(edge_id) else { return Ok(None) };
        if edge.degenerate {
            // A pole: a point every mover of its one face carries to a point
            // (a corner ball's apex), moved with that face.
            if pipe_smooth_joins_on() && edge_uses.iter().all(|(face, _)| *face == edge_uses[0].0) {
                continue;
            }
            return Ok(None);
        }
        match edge_uses.as_slice() {
            [(first, _), (second, _)] if first == second => {}
            [(first, first_forward), (second, second_forward)] if first_forward != second_forward => {
                if edge.start_vertex_id != edge.end_vertex_id && !pipe_corners_on() {
                    os_debug!("pipe parallel lane: edge {edge_id} ends in corners");
                    return Ok(None);
                }
                let face = faces.iter().find(|face| face.id == *first).expect("used face");
                let coedge = face
                    .loops
                    .iter()
                    .flat_map(|loop_record| &loop_record.coedges)
                    .find(|coedge| coedge.edge_id == *edge_id)
                    .expect("its use");
                let Some((mate, _)) = junction_mate(&faces, face, *edge_id) else { return Ok(None) };
                let stations = junction_stations(face, coedge, edge, mate)?;
                // `junction_stations` skips every station where the two faces
                // meet tangentially, so a join with none left is tangent
                // all along; the movers' agreement below makes it exact.
                if pipe_smooth_joins_on() && stations.is_empty() {
                    smooth.push((*edge_id, *first, *second));
                    continue;
                }
                if stations.is_empty() || !stations.iter().all(JunctionStation::is_convex) {
                    os_debug!("pipe parallel lane: edge {edge_id} is not convex all along");
                    return Ok(None);
                }
                let (forward_face, backward_face) = if *first_forward { (*first, *second) } else { (*second, *first) };
                rounds.push((*edge_id, forward_face, backward_face));
            }
            _ => return Ok(None),
        }
    }
    if rounds.is_empty() && smooth.is_empty() {
        return Ok(None);
    }
    rounds.sort();
    smooth.sort();
    let seams: HashSet<u64> = uses
        .iter()
        .filter(|(_, edge_uses)| matches!(edge_uses.as_slice(), [(first, _), (second, _)] if first == second))
        .map(|(edge_id, _)| *edge_id)
        .collect();
    // Each CORNER: a source vertex an open round ends at. Every face round it
    // in order (seams skipped: a seam is no crease) with the round leaving the
    // vertex in it; three faces at least, each meeting the next at a round.
    let mut corners: Vec<(u64, Vec<(u64, u64)>)> = Vec::new();
    let round_ids: HashSet<u64> = rounds.iter().map(|(edge_id, _, _)| *edge_id).collect();
    let mut corner_ids: Vec<u64> = rounds
        .iter()
        .map(|(edge_id, _, _)| edge_by_id[edge_id])
        .filter(|edge| edge.start_vertex_id != edge.end_vertex_id)
        .flat_map(|edge| [edge.start_vertex_id, edge.end_vertex_id])
        .collect();
    corner_ids.sort_unstable();
    corner_ids.dedup();
    for vertex_id in &corner_ids {
        // (face, round leaving the vertex in that face's walk)
        let leaving = |face: &FaceRecord| -> Vec<u64> {
            face.loops
                .iter()
                .flat_map(|loop_record| &loop_record.coedges)
                .filter(|coedge| !seams.contains(&coedge.edge_id))
                .filter(|coedge| {
                    let edge = edge_by_id[&coedge.edge_id];
                    (if coedge.forward { edge.start_vertex_id } else { edge.end_vertex_id }) == *vertex_id
                })
                .map(|coedge| coedge.edge_id)
                .collect()
        };
        let Some(first) = faces.iter().find(|face| !leaving(face).is_empty()) else { return Ok(None) };
        let mut cycle: Vec<(u64, u64)> = Vec::new();
        let mut face = *first;
        loop {
            let out = leaving(face);
            let [edge_id] = out.as_slice() else {
                os_debug!("pipe parallel lane: vertex {vertex_id} is not one fan of faces");
                return Ok(None);
            };
            if !round_ids.contains(edge_id) {
                return Ok(None);
            }
            cycle.push((face.id, *edge_id));
            let Some((mate, _)) = junction_mate(&faces, face, *edge_id) else { return Ok(None) };
            if mate.id == first.id {
                break;
            }
            if cycle.len() > faces.len() {
                return Ok(None);
            }
            face = mate;
        }
        // A corner's patch needs three faces; a vertex two faces meet at is
        // not one (named limit).
        if cycle.len() < 3 {
            os_debug!("pipe parallel lane: vertex {vertex_id} joins {} face(s), not a corner", cycle.len());
            return Ok(None);
        }
        corners.push((*vertex_id, cycle));
    }

    // Ids.
    let mut next = 1u64;
    let mut take = || {
        next += 1;
        next
    };
    let mut vertices: Vec<VertexRecord> = Vec::new();
    let mut vertex_of: HashMap<(u64, u64), u64> = HashMap::default(); // (face, source vertex)
    // Across a tangent join both faces move a shared vertex to one point:
    // each (face, vertex) is read as its class's least member.
    let mut class: HashMap<(u64, u64), (u64, u64)> = HashMap::default();
    fn root(class: &HashMap<(u64, u64), (u64, u64)>, mut key: (u64, u64)) -> (u64, u64) {
        while let Some(next) = class.get(&key) {
            if *next == key {
                break;
            }
            key = *next;
        }
        key
    }
    for (edge_id, first, second) in &smooth {
        let edge = edge_by_id[edge_id];
        for vertex in [edge.start_vertex_id, edge.end_vertex_id] {
            let (a, b) = (root(&class, (*first, vertex)), root(&class, (*second, vertex)));
            if a != b {
                let (low, high) = if a < b { (a, b) } else { (b, a) };
                class.insert(high, low);
            }
        }
        // The two faces' movers agree along the join, or it is no join.
        for k in 0..=8 {
            let t = edge.t0 + (edge.t1 - edge.t0) * k as f64 / 8.0;
            let point = edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
            let gap = movers[first].moved(point, distance).sub(movers[second].moved(point, distance)).length();
            if gap > 1e-9 * (1.0 + point.length()) {
                os_debug!("pipe parallel lane: tangent edge {edge_id}'s faces move it apart by {gap:.3e}");
                return Ok(None);
            }
        }
    }
    let smooth_mate: HashMap<(u64, u64), u64> = smooth
        .iter()
        .flat_map(|(edge_id, first, second)| [((*first, *edge_id), *second), ((*second, *edge_id), *first)])
        .collect();
    let mut vertex = |face: u64, source_vertex: u64, take: &mut dyn FnMut() -> u64, vertices: &mut Vec<VertexRecord>| -> Result<u64, KernelRefusal> {
        let (face, source_vertex) = root(&class, (face, source_vertex));
        if let Some(id) = vertex_of.get(&(face, source_vertex)) {
            return Ok(*id);
        }
        let point = vertex_by_id.get(&source_vertex).copied().ok_or_else(|| "pipe parallel: vertex missing".to_string()).or_refuse(KernelStage::Refine, "vertex")?;
        let id = take();
        vertices.push(VertexRecord { id, point: movers[&face].moved(point, distance) });
        vertex_of.insert((face, source_vertex), id);
        Ok(id)
    };
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let mut edge_of: HashMap<(u64, u64), u64> = HashMap::default(); // (face, source edge) -> moved edge
    for face in &faces {
        for coedge in face.loops.iter().flat_map(|loop_record| &loop_record.coedges) {
            if edge_of.contains_key(&(face.id, coedge.edge_id)) {
                continue;
            }
            let edge = edge_by_id[&coedge.edge_id];
            let start = vertex(face.id, edge.start_vertex_id, &mut take, &mut vertices)?;
            let end = vertex(face.id, edge.end_vertex_id, &mut take, &mut vertices)?;
            let id = take();
            edges.push(EdgeRecord {
                id,
                curve: moved_curve(&edge.curve, &movers[&face.id], distance)?,
                t0: edge.t0,
                t1: edge.t1,
                start_vertex_id: start,
                end_vertex_id: end,
                degenerate: edge.degenerate,
                name: None,
            });
            edge_of.insert((face.id, coedge.edge_id), id);
            if let Some(mate) = smooth_mate.get(&(face.id, coedge.edge_id)) {
                edge_of.insert((*mate, coedge.edge_id), id);
            }
        }
    }
    // The moved faces: the source loops over the moved edges, pcurves as they are.
    let mut out_faces: Vec<FaceRecord> = Vec::new();
    let mut sources: Vec<Option<u64>> = Vec::new();
    for face in &faces {
        sources.push(Some(face.id));
        let loops = face
            .loops
            .iter()
            .map(|loop_record| LoopRecord {
                id: take(),
                coedges: loop_record
                    .coedges
                    .iter()
                    .map(|coedge| CoedgeRecord { id: take(), edge_id: edge_of[&(face.id, coedge.edge_id)], forward: coedge.forward, pcurve: coedge.pcurve.clone() })
                    .collect(),
            })
            .collect();
        out_faces.push(FaceRecord { id: take(), surface: moved_surface(&face.surface, &movers[&face.id], distance)?, same_sense: face.same_sense, loops, name: face.name.clone() });
    }
    // One pipe per closed edge, s = 0 on the face that walks the edge
    // backward (the pipe walks that rail forward), s = 1 on the other.
    let line = |a: f64, b: f64, c: f64, d: f64| crate::sweep_topology::parameter_line(a, b, c, d).or_refuse(KernelStage::Refine, "parameter_line");
    // (source edge, source vertex) -> the arc a pipe ends in there.
    let mut arcs: HashMap<(u64, u64), u64> = HashMap::default();
    for (edge_id, forward_face, backward_face) in &rounds {
        let edge = edge_by_id[edge_id];
        let Some(surface) = pipe_surface(&edge.curve, &movers[backward_face], &movers[forward_face], distance)? else {
            os_debug!("pipe parallel lane: edge {edge_id}'s pipe could not be read");
            return Ok(None);
        };
        let (t0, t1) = (edge.t0, edge.t1);
        let rail_low = edge_of[&(*backward_face, *edge_id)];
        let rail_high = edge_of[&(*forward_face, *edge_id)];
        // A cone's rail is the pipe's own iso line there: its moved edge is
        // not exact (a curve's net does not move by the cone's normal), and
        // the pipe's line is the exact image of the source edge.
        for (s, face_id, rail) in [(0.0, backward_face, rail_low), (1.0, forward_face, rail_high)] {
            if movers[face_id].is_cone() {
                let curve = surface.iso_curve_v(s).or_refuse(KernelStage::Refine, "iso_curve_v")?;
                edges.iter_mut().find(|e| e.id == rail).expect("rail").curve = curve;
            }
        }
        // The pipe must carry its rails: its iso lines s = 0, 1 against the
        // moved edges.
        for fraction in [0.0, 0.31, 0.62, 1.0] {
            let t = t0 + (t1 - t0) * fraction;
            for (s, rail) in [(0.0, rail_low), (1.0, rail_high)] {
                let on_pipe = surface.evaluate(t, s).or_refuse(KernelStage::Refine, "evaluate")?;
                let rail_edge = edges.iter().find(|e| e.id == rail).expect("rail");
                let on_rail = rail_edge.curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
                if on_pipe.sub(on_rail).length() > 1e-9 * (1.0 + on_rail.length()) {
                    os_debug!("pipe parallel lane: edge {edge_id}'s pipe leaves its rail by {:.3e}", on_pipe.sub(on_rail).length());
                    return Ok(None);
                }
            }
        }
        let rail_vertices = |rail: u64| {
            let record = edges.iter().find(|e| e.id == rail).expect("rail");
            (record.start_vertex_id, record.end_vertex_id)
        };
        let (low_start, low_end) = rail_vertices(rail_low);
        let (high_start, high_end) = rail_vertices(rail_high);
        let (start_arc, end_arc) = if edge.start_vertex_id == edge.end_vertex_id {
            // A closed edge: one seam, the pipe's iso line at t0, both ways.
            let seam_curve = surface.iso_curve_u(t0).or_refuse(KernelStage::Refine, "iso_curve_u")?;
            let seam = take();
            edges.push(EdgeRecord { id: seam, curve: seam_curve, t0: 0.0, t1: 1.0, start_vertex_id: low_start, end_vertex_id: high_start, degenerate: false, name: None });
            ((seam, false), (seam, true))
        } else {
            // An open edge ends in two arcs, each the pipe's iso line there,
            // each walked forward by the pipe: at the end from the low rail
            // to the high one, at the start back (the corner patch walks
            // every arc the other way).
            let end_curve = surface.iso_curve_u(t1).or_refuse(KernelStage::Refine, "iso_curve_u")?;
            let start_curve = surface
                .iso_curve_u(t0)
                .or_refuse(KernelStage::Refine, "iso_curve_u")?
                .reversed()
                .or_refuse(KernelStage::Refine, "reversed")?;
            let (end_id, start_id) = (take(), take());
            let [e0, e1] = end_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            let [s0, s1] = start_curve.domain().or_refuse(KernelStage::Refine, "domain")?;
            edges.push(EdgeRecord { id: end_id, curve: end_curve, t0: e0, t1: e1, start_vertex_id: low_end, end_vertex_id: high_end, degenerate: false, name: None });
            edges.push(EdgeRecord { id: start_id, curve: start_curve, t0: s0, t1: s1, start_vertex_id: high_start, end_vertex_id: low_start, degenerate: false, name: None });
            arcs.insert((*edge_id, edge.end_vertex_id), end_id);
            arcs.insert((*edge_id, edge.start_vertex_id), start_id);
            ((start_id, true), (end_id, true))
        };
        // A pipe the canonical form left straight along its spine is a
        // cylinder: written arc-first (u across, v along), the layout of a
        // cylinder of revolution, so the next offset reads it as one; its
        // pcurves swap their coordinates with it.
        let across_first = pipe_canonical_on() && surface.degree_u == 1;
        let surface = if across_first { transposed(&surface)? } else { surface };
        let at = |t: f64, s: f64| if across_first { (s, t) } else { (t, s) };
        let segment = |a: (f64, f64), b: (f64, f64)| {
            let ((u0, v0), (u1, v1)) = (at(a.0, a.1), at(b.0, b.1));
            line(u0, v0, u1, v1)
        };
        let mut face = FaceRecord {
            id: take(),
            surface,
            same_sense: true,
            name: None,
            loops: vec![LoopRecord {
                id: take(),
                coedges: vec![
                    CoedgeRecord { id: take(), edge_id: rail_low, forward: true, pcurve: segment((t0, 0.0), (t1, 0.0))? },
                    CoedgeRecord { id: take(), edge_id: end_arc.0, forward: end_arc.1, pcurve: segment((t1, 0.0), (t1, 1.0))? },
                    CoedgeRecord { id: take(), edge_id: rail_high, forward: false, pcurve: segment((t1, 1.0), (t0, 1.0))? },
                    CoedgeRecord { id: take(), edge_id: start_arc.0, forward: start_arc.1, pcurve: segment((t0, 1.0), (t0, 0.0))? },
                ],
            }],
        };
        // The loop's winding is fixed by its neighbours; the face's sense
        // follows it, and its normal must then point away from the spine.
        face.same_sense = parameter_space_area(&face).or_refuse(KernelStage::Refine, "parameter_space_area")? > 0.0;
        let (tm, sm) = ((t0 + t1) * 0.5, 0.5);
        let (um, vm) = at(tm, sm);
        let outward = super::face_normal(&face, um, vm)?;
        let spine = edge.curve.evaluate(tm).or_refuse(KernelStage::Refine, "evaluate")?;
        if outward.dot(face.surface.evaluate(um, vm).or_refuse(KernelStage::Refine, "evaluate")?.sub(spine)) <= 0.0 {
            os_debug!("pipe parallel lane: edge {edge_id}'s pipe would face its spine");
            return Ok(None);
        }
        out_faces.push(face);
        sources.push(None);
    }
    // A sphere patch of radius `distance` about each corner, bounded by the
    // arcs its pipes end in, round the faces in order.
    for (vertex_id, cycle) in &corners {
        let centre = vertex_by_id[vertex_id];
        let normals = cycle.iter().map(|(face, _)| movers[face].normal(centre)).collect::<Vec<_>>();
        let mut bounding = Vec::with_capacity(cycle.len());
        for (_, edge_id) in cycle {
            let Some(arc) = arcs.get(&(*edge_id, *vertex_id)) else { return Ok(None) };
            bounding.push(edges.iter().find(|e| e.id == *arc).expect("arc").clone());
        }
        let (patch, pole) = crate::blend::convex_parallel::build_corner_patch_exact(centre, distance, &normals, &bounding, true, None, &mut take)?;
        if let Some(pole) = pole {
            edges.push(pole);
        }
        out_faces.push(patch);
        sources.push(None);
    }
    let solid = BrepSolid {
        mass_properties_cache: Default::default(),
        cells: Default::default(),
        id: take(),
        genus: source.genus,
        kind: source.kind,
        vertices,
        edges,
        shells: vec![ShellRecord { id: take(), faces: out_faces }],
    };
    let issues = solid.validate();
    if !issues.is_empty() {
        os_debug!("pipe parallel lane declined: {} validation issue(s), first {:?}", issues.len(), issues.first());
        return Ok(None);
    }
    os_debug!("pipe parallel lane built {} faces ({} pipes)", solid.shells[0].faces.len(), rounds.len());
    Ok(Some((solid, sources)))
}
