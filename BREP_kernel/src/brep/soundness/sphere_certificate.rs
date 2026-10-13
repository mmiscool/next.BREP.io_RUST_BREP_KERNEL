//! Native trimmed sphere crossing certificates, with complete polynomial bounds.
use super::*;
use crate::make_sphere_surface;

macro_rules! decline {
    () => {{
        if std::env::var("BREP_DEBUG_SPHERE_CERTIFICATE").is_ok() {
            eprintln!("soundness.sphere_certificate decline line={}", line!());
        }
        return false;
    }};
}

#[derive(Clone, Copy)]
struct Ball {
    centre: Vec3,
    radius: f64,
}

pub(super) fn enabled() -> bool {
    // This proof uses the full scanner's sphere atlas and trim classification.
    // An explicit request for either alternative keeps that scanner path.
    if std::env::var_os("BREP_NO_SPHERE_CHARTS").is_some()
        || std::env::var_os("BREP_NO_SPHERE_CHART_TRIM").is_some()
        || !super::trim_samples_enabled()
    {
        return false;
    }
    match std::env::var("BREP_SOUNDNESS_SPHERE_CERTIFICATE") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

/// Select only the canonical z-axis constructor, with EVERY stored bit equal
/// to a fresh construction. Neither cached analytics nor approximate carrier
/// recognition can grant this certificate. Other sphere frames keep the scan.
fn native_ball(s: &NurbsSurface) -> Option<Ball> {
    if s.degree_u != 2
        || s.degree_v != 2
        || s.control_points.len() != 9
        || s.control_points.iter().any(|r| r.len() != 5)
    {
        return None;
    }
    let a = s.control_points[0][2].point().ok()?;
    let b = s.control_points[4][2].point().ok()?;
    let south = s.control_points[0][0].point().ok()?;
    let north = s.control_points[0][4].point().ok()?;
    let centre = Vec3::new(a.x, (a.y + b.y) * 0.5, a.z);
    let radius = (north.z - south.z) * 0.5;
    if ![centre.x, centre.y, centre.z, radius]
        .iter()
        .all(|v| v.is_finite())
        || radius <= 0.
    {
        return None;
    }
    let rebuilt = make_sphere_surface(centre, radius, Vec3::new(0., 0., 1.)).ok()?;
    if s.knots_u != rebuilt.knots_u || s.knots_v != rebuilt.knots_v {
        return None;
    }
    if !s
        .control_points
        .iter()
        .flatten()
        .zip(rebuilt.control_points.iter().flatten())
        .all(|(a, b)| {
            [a.x, a.y, a.z, a.w].map(f64::to_bits) == [b.x, b.y, b.z, b.w].map(f64::to_bits)
        })
    {
        return None;
    }
    // The atlas the full scanner would run is cache-driven. Require that
    // cache to agree with the freshly proved net; a stale analytic tag cannot
    // silently route the full scanner onto a different support or the polar
    // fallback. This check cannot grant native recognition by itself.
    let (cached_centre, cached_radius) = s.analytic()?.sphere_geometry()?;
    let reserve = 1e-10 * (1.0 + radius + centre.length());
    if cached_centre.sub(centre).length() > reserve || (cached_radius - radius).abs() > reserve {
        return None;
    }
    Some(Ball { centre, radius })
}

/// Only the standard scale-derived scanner options are covered. Native sphere
/// supports are injective away from their identified seam/poles. The constructor
/// roundoff is orders below all four scale-derived resolution bands here.
pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    if solid.shells.len() != 1 || !matches!(solid.shells[0].faces.len(), 2 | 6 | 7) {
        decline!();
    }
    let faces = &solid.shells[0].faces;
    let balls = faces
        .iter()
        .filter(|f| native_ball(&f.surface).is_some())
        .count();
    if (faces.len() == 2 && balls != 2) || (faces.len() != 2 && balls != 1) {
        decline!();
    }
    let scoped = solid.shells[0].faces.len() == 6;
    if solid.validate().iter().any(|issue| {
        !scoped
            || !matches!(
                issue.kind,
                crate::IssueKind::OpenEdge | crate::IssueKind::GenusMismatch
            )
    }) {
        decline!();
    }
    if ![
        options.chord_tolerance,
        options.straddle_band,
        options.min_crossing_length,
        options.weld,
    ]
    .iter()
    .all(|v| v.is_finite() && *v > 0.)
    {
        decline!();
    }
    let expected = SelfIntersectionOptions::for_solid(solid);
    if options.chord_tolerance.to_bits() != expected.chord_tolerance.to_bits()
        || options.straddle_band.to_bits() != expected.straddle_band.to_bits()
        || options.min_crossing_length.to_bits() != expected.min_crossing_length.to_bits()
        || options.weld.to_bits() != expected.weld.to_bits()
        || options.max_triangles != expected.max_triangles
    {
        decline!();
    }
    let faces = &solid.shells[0].faces;
    match faces.len() {
        2 => {
            let (Some(a), Some(b)) = (
                native_ball(&faces[0].surface),
                native_ball(&faces[1].surface),
            ) else {
                decline!();
            };
            if faces
                .iter()
                .any(|f| f.loops.len() != 2 || !domain_wire(&f.loops[0]))
            {
                decline!();
            }
            let delta = b.centre.sub(a.centre);
            let d = delta.length();
            if !d.is_finite()
                || d <= 0.
                || d >= a.radius + b.radius
                || d <= (a.radius - b.radius).abs()
            {
                decline!();
            }
            let n = delta.scale(1. / d);
            let h = (a.radius * a.radius - b.radius * b.radius + d * d) / (2. * d);
            let rho2 = a.radius * a.radius - h * h;
            if rho2 < (a.radius.max(b.radius) * 0.25).powi(2) {
                decline!();
            }
            let origin = a.centre.add(n.scale(h));
            let band = options.straddle_band * a.radius.min(b.radius)
                / (16. * d.max(a.radius.max(b.radius)));
            ring_certificate(&faces[0], solid, a, origin, n, band)
                && ring_certificate(&faces[1], solid, b, origin, n, band)
        }
        6 | 7 => box_ball_certificate(solid, options),
        _ => false,
    }
}

fn domain_wire(wire: &LoopRecord) -> bool {
    let corners = [(0., 0.), (1., 0.), (1., 1.), (0., 1.)];
    wire.coedges.len() == 4
        && wire.coedges.iter().enumerate().all(|(i, c)| {
            let p = &c.pcurve;
            p.degree == 1
                && p.control_points.len() == 2
                && p.knots.len() == 4
                && p.knots[0] == p.knots[1]
                && p.knots[2] == p.knots[3]
                && p.knots[2] > p.knots[1]
                && p.control_points.iter().all(|p| p.w == 1. && p.z == 0.)
                && (p.control_points[0].x, p.control_points[0].y) == corners[i]
                && (p.control_points[1].x, p.control_points[1].y) == corners[(i + 1) % 4]
        })
}

fn box_ball_certificate(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    let faces = &solid.shells[0].faces;
    let balls: Vec<_> = faces
        .iter()
        .filter_map(|f| native_ball(&f.surface).map(|b| (f, b)))
        .collect();
    if balls.len() != 1 {
        decline!();
    }
    let (sphere, ball) = balls[0];
    if sphere.loops.len() != 1 {
        decline!();
    }
    let mut planes = Vec::new();
    let mut bounds = Aabb::empty();
    for face in faces.iter().filter(|f| f.id != sphere.id) {
        let s = &face.surface;
        let (Ok([u0, u1]), Ok([v0, v1])) = (s.domain_u(), s.domain_v()) else {
            decline!();
        };
        if s.degree_u != 1
            || s.degree_v != 1
            || s.knots_u != [u0, u0, u1, u1]
            || s.knots_v != [v0, v0, v1, v1]
            || u1 <= u0
            || v1 <= v0
            || s.control_points.len() != 2
            || s.control_points.iter().any(|r| r.len() != 2)
            || s.control_points
                .iter()
                .flatten()
                .any(|p| p.w != 1. || ![p.x, p.y, p.z].iter().all(|v| v.is_finite()))
        {
            decline!();
        }
        let points: Vec<_> = s
            .control_points
            .iter()
            .flatten()
            .map(|p| Vec3::new(p.x, p.y, p.z))
            .collect();
        let box_of = Aabb::from_points(points.iter().copied());
        bounds.include(box_of);
        let lo = [box_of.minimum.x, box_of.minimum.y, box_of.minimum.z];
        let hi = [box_of.maximum.x, box_of.maximum.y, box_of.maximum.z];
        let constant: Vec<_> = (0..3).filter(|i| lo[*i] == hi[*i]).collect();
        if constant.len() != 1 {
            decline!();
        }
        let axis = constant[0];
        // Four different rectangle corners, no bilinear bow-tie or rank drop.
        let mut seen = HashSet::default();
        for p in &points {
            let q = [p.x, p.y, p.z];
            let mut mask = 0;
            for i in 0..3 {
                if i == axis {
                    continue;
                }
                if q[i] == hi[i] {
                    mask |= 1 << i;
                } else if q[i] != lo[i] {
                    decline!();
                }
            }
            if !seen.insert(mask) {
                decline!();
            }
        }
        let du = points[2].sub(points[0]);
        let dv = points[1].sub(points[0]);
        let expected = points[0].add(du).add(dv);
        if du.cross(dv).length() == 0.
            || (points[3].x, points[3].y, points[3].z) != (expected.x, expected.y, expected.z)
        {
            decline!();
        }
        // Trims only restrict the rectangle, never extend past it.
        if face.loops.is_empty()
            || face.loops.len() > 2
            || face.loops.iter().flat_map(|l| l.coedges.iter()).any(|c| {
                c.pcurve.control_points.iter().any(|p| {
                    p.w <= 0.
                        || p.point()
                            .map_or(true, |q| q.x < u0 || q.x > u1 || q.y < v0 || q.y > v1)
                })
            })
        {
            decline!();
        }
        planes.push((face, axis, lo[axis], box_of));
    }
    if planes.len() != faces.len() - 1 {
        decline!();
    }
    let low = [bounds.minimum.x, bounds.minimum.y, bounds.minimum.z];
    let high = [bounds.maximum.x, bounds.maximum.y, bounds.maximum.z];
    let centre = [ball.centre.x, ball.centre.y, ball.centre.z];
    let mut seen = HashSet::default();
    let mut section = None;
    for (face, axis, level, box_of) in planes {
        if level != low[axis] && level != high[axis] {
            decline!();
        }
        if !seen.insert((axis, level == high[axis])) {
            decline!();
        }
        let lo = [box_of.minimum.x, box_of.minimum.y, box_of.minimum.z];
        let hi = [box_of.maximum.x, box_of.maximum.y, box_of.maximum.z];
        if (0..3).any(|i| i != axis && (lo[i] != low[i] || hi[i] != high[i])) {
            decline!();
        }
        let distance = (centre[axis] - level).abs();
        if distance < ball.radius {
            if section.is_some() || face.loops.len() != 2 || centre[axis] != level {
                decline!();
            }
            let mut n = [0.; 3];
            n[axis] = 1.;
            let mut q = centre;
            q[axis] = level;
            section = Some((Vec3::new(q[0], q[1], q[2]), Vec3::new(n[0], n[1], n[2])));
        } else if distance <= ball.radius + 8. * options.straddle_band || face.loops.len() != 1 {
            decline!();
        }
    }
    let Some((origin, n)) = section else {
        decline!();
    };
    physical_circle_certificate(
        sphere,
        solid,
        ball,
        origin,
        n,
        options.straddle_band / 32.,
        2,
    )
}

type Polynomial = Vec<f64>;
fn binomial(n: usize, k: usize) -> f64 {
    static TABLE: std::sync::OnceLock<[[f64; 25]; 25]> = std::sync::OnceLock::new();
    TABLE.get_or_init(|| {
        let mut t = [[0.; 25]; 25];
        for n in 0..25 {
            t[n][0] = 1.;
            t[n][n] = 1.;
            for k in 1..n {
                t[n][k] = t[n - 1][k - 1] + t[n - 1][k];
            }
        }
        t
    })[n][k]
}
fn product(a: &[f64], b: &[f64]) -> Polynomial {
    let (m, n) = (a.len() - 1, b.len() - 1);
    let mut c = vec![0.; m + n + 1];
    for i in 0..=m {
        for j in 0..=n {
            c[i + j] += a[i] * b[j] * binomial(m, i) * binomial(n, j) / binomial(m + n, i + j);
        }
    }
    c
}
fn quadratic_basis(u: [f64; 4]) -> [Polynomial; 3] {
    let v = u.map(|p| 1. - p);
    let b = product(&u, &v).iter().map(|p| 2. * p).collect();
    [product(&v, &v), b, product(&u, &u)]
}

/// Compose a tensor quadratic homogeneous surface with a polynomial cubic
/// pcurve in Bernstein form. The result has degree 12, so every coefficient
/// participates in the bound; there are no off-node probes.
fn compose(
    face: &FaceRecord,
    uv: [[f64; 2]; 4],
    origin: Vec3,
    basis: [Vec3; 3],
    iu: usize,
    iv: usize,
) -> [Polynomial; 3] {
    let u = quadratic_basis(uv.map(|p| 4. * p[0] - iu as f64));
    let v = quadratic_basis(uv.map(|p| 2. * p[1] - iv as f64));
    let mut out = [vec![0.; 13], vec![0.; 13], vec![0.; 13]];
    for i in 0..3 {
        for j in 0..3 {
            let p = face.surface.control_points[2 * iu + i][2 * iv + j];
            let local = Vec3::new(
                p.x - origin.x * p.w,
                p.y - origin.y * p.w,
                p.z - origin.z * p.w,
            );
            let coeff = basis.map(|b| b.dot(local));
            let weight = product(&u[i], &v[j]);
            for d in 0..3 {
                for k in 0..13 {
                    out[d][k] += coeff[d] * weight[k];
                }
            }
        }
    }
    out
}

struct RingProof {
    rotation: f64,
    first: Option<Vec3>,
    last: Option<Vec3>,
    pieces: usize,
}
fn piece(
    face: &FaceRecord,
    uv: [[f64; 2]; 4],
    origin: Vec3,
    basis: [Vec3; 3],
    band: f64,
    round: f64,
    depth: usize,
    proof: &mut RingProof,
) -> bool {
    proof.pieces += 1;
    if proof.pieces > 4096 {
        decline!();
    }
    let umin = uv.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
    let umax = uv.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
    let vmin = uv.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min);
    let vmax = uv.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max);
    let indices = |low: f64, high: f64, count: usize| {
        let mut result: Vec<_> = (0..count)
            .filter(|i| {
                high > *i as f64 / count as f64 + 1e-12
                    && low < (*i + 1) as f64 / count as f64 - 1e-12
            })
            .collect();
        if result.is_empty() {
            result.push((((low + high) * 0.5 * count as f64).floor() as usize).min(count - 1));
        }
        result
    };
    let us = indices(umin, umax, 4);
    let vs = indices(vmin, vmax, 2);
    let cells: Vec<_> = us
        .iter()
        .flat_map(|iu| vs.iter().map(move |iv| (*iu, *iv)))
        .collect();
    if cells.is_empty() {
        decline!();
    }
    if cells.len() > 1 && depth < 14 {
        let lerp = |a: [f64; 2], b: [f64; 2]| [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5];
        let a = lerp(uv[0], uv[1]);
        let b = lerp(uv[1], uv[2]);
        let c = lerp(uv[2], uv[3]);
        let d = lerp(a, b);
        let e = lerp(b, c);
        let f = lerp(d, e);
        return piece(
            face,
            [uv[0], a, d, f],
            origin,
            basis,
            band,
            round,
            depth + 1,
            proof,
        ) && piece(
            face,
            [f, e, c, uv[3]],
            origin,
            basis,
            band,
            round,
            depth + 1,
            proof,
        );
    }
    let Ok(first) = face.surface.evaluate(uv[0][0], uv[0][1]) else {
        decline!();
    };
    let Ok(last) = face.surface.evaluate(uv[3][0], uv[3][1]) else {
        decline!();
    };
    let local = first.sub(origin);
    let end = last.sub(origin);
    let (x, y) = (basis[0].dot(local), basis[1].dot(local));
    let (xx, yy) = (basis[0].dot(end), basis[1].dot(end));
    for (iu, iv) in cells {
        let q = compose(face, uv, origin, basis, iu, iv);
        // At a support knot, bound EVERY patch the control hull can reach.
        // Each patch polynomial is used only on the part of the span that
        // belongs to it, where the positive tensor denominator is >=1/2.
        // Whole-span bounds on each polynomial conservatively cover that part.
        if q[2].iter().any(|p| p.abs() + round > 0.48 * band) {
            decline!();
        }
        let scale = q[0]
            .iter()
            .chain(q[1].iter())
            .map(|v| v.abs())
            .fold(0., f64::max)
            .max(1.);
        let reserve = round * scale * 1024.;
        if q[0]
            .iter()
            .zip(&q[1])
            .any(|(a, b)| x * a + y * b <= reserve)
        {
            decline!();
        }
    }
    // The entire piece stays in one open angular half-plane, including any
    // support-knot transitions, so endpoint angles cannot hide an extra wrap.
    proof.rotation += (x * yy - y * xx).atan2(x * xx + y * yy);
    if let Some(previous) = proof.last {
        if previous.sub(first).length() > band / 16. {
            decline!();
        }
    }
    if proof.first.is_none() {
        proof.first = Some(first);
    }
    proof.last = Some(last);
    true
}

fn ring_certificate(
    face: &FaceRecord,
    solid: &BrepSolid,
    ball: Ball,
    origin: Vec3,
    n: Vec3,
    band: f64,
) -> bool {
    let round = 1e-10 * (1. + ball.radius + origin.length());
    if !band.is_finite() || band <= round * 16. {
        decline!();
    }
    let Ok(x) = n.perpendicular() else {
        decline!();
    };
    let y = n.cross(x);
    let mut proof = RingProof {
        rotation: 0.,
        first: None,
        last: None,
        pieces: 0,
    };
    let mut uses = HashMap::default();
    for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
        *uses.entry(c.edge_id).or_insert(0usize) += 1;
    }
    let mut rings = 0;
    for wire in &face.loops {
        let coedges: Vec<_> = wire
            .coedges
            .iter()
            .filter(|c| {
                uses[&c.edge_id] == 1
                    && solid
                        .edges
                        .iter()
                        .any(|e| e.id == c.edge_id && !e.degenerate)
            })
            .collect();
        if coedges.is_empty() {
            continue;
        }
        rings += 1;
        if rings > 1 {
            decline!();
        }
        for c in coedges {
            let curve = &c.pcurve;
            if curve.degree > 3
                || curve.knots.windows(2).any(|k| k[1] < k[0])
                || curve.control_points.iter().any(|p| {
                    p.w != 1.
                        || p.z != 0.
                        || !p.x.is_finite()
                        || !p.y.is_finite()
                        || p.x < 0.
                        || p.x > 1.
                        || p.y < 0.
                        || p.y > 1.
                })
            {
                decline!();
            }
            if NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                curve.control_points.clone(),
            )
            .is_err()
            {
                decline!();
            }
            let Ok([start, end]) = curve.domain() else {
                decline!();
            };
            let mut breaks: Vec<_> = curve
                .knots
                .iter()
                .copied()
                .filter(|t| *t >= start && *t <= end)
                .collect();
            breaks.dedup();
            if breaks.iter().any(|t| {
                *t > start
                    && *t < end
                    && curve.knots.iter().filter(|k| **k == *t).count() > curve.degree
            }) {
                decline!();
            }
            if curve.knots[..=curve.degree].iter().any(|t| *t != start)
                || curve.knots[curve.knots.len() - 1 - curve.degree..]
                    .iter()
                    .any(|t| *t != end)
            {
                decline!();
            }
            for span in breaks.windows(2) {
                let (a, b) = (span[0], span[1]);
                let h = b - a;
                let samples = [a, a + h / 3., a + 2. * h / 3., b];
                let mut p = [[0.; 2]; 4];
                for i in 0..4 {
                    let Ok(v) = curve.evaluate(samples[i]) else {
                        decline!();
                    };
                    p[i] = [v.x, v.y];
                }
                let mut uv = [p[0], [0.; 2], [0.; 2], p[3]];
                for d in 0..2 {
                    uv[1][d] = (-5. * p[0][d] + 18. * p[1][d] - 9. * p[2][d] + 2. * p[3][d]) / 6.;
                    uv[2][d] = (2. * p[0][d] - 9. * p[1][d] + 18. * p[2][d] - 5. * p[3][d]) / 6.;
                }
                // Sampling rounded knot abscissae reconstructs a polynomial
                // with an error that grows with span conditioning. Charge it
                // against the geometric envelope, rather than assuming exact
                // thirds on arbitrarily small or translated parameter spans.
                let span_round = round
                    + 4096.0
                        * f64::EPSILON
                        * (1.0 + ball.radius + origin.length())
                        * (1.0 + a.abs() + b.abs())
                        / h;
                if !span_round.is_finite()
                    || !piece(face, uv, origin, [x, y, n], band, span_round, 0, &mut proof)
                {
                    decline!();
                }
            }
        }
    }
    if rings != 1
        || !proof.rotation.is_finite()
        || (proof.rotation.abs() - std::f64::consts::TAU).abs() > 1e-6
    {
        decline!();
    }
    let (Some(first), Some(last)) = (proof.first, proof.last) else {
        decline!();
    };
    first.sub(last).length() <= band / 16.
        && physical_circle_certificate(face, solid, ball, origin, n, band, 1)
}

/// Bound the complete shared rational boundary the watertight sphere atlas
/// actually reads, independently of the pcurve proof. Every span lies on the
/// native support and in the intersection-plane strip, with one closed winding.
/// For the box's EQUATOR the gnomonic chart boundary is a straight line: atlas
/// material lies in one hemisphere and triangles are convex hulls of vertices
/// on that side outside the charged band. For two spheres the pcurve proof
/// additionally confines the trimmed material; neither representation alone
/// grants that certificate. This includes the existing box pole proxy.
fn physical_circle_certificate(
    face: &FaceRecord,
    solid: &BrepSolid,
    ball: Ball,
    origin: Vec3,
    n: Vec3,
    band: f64,
    expected_edges: usize,
) -> bool {
    let round = 1e-10 * (1. + ball.radius + origin.length());
    if band <= round * 16. {
        decline!();
    }
    let Ok(x) = n.perpendicular() else {
        decline!();
    };
    let y = n.cross(x);
    let mut rotation = 0.;
    let mut first = None::<Vec3>;
    let mut last = None::<Vec3>;
    let mut uses = HashMap::default();
    for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
        *uses.entry(c.edge_id).or_insert(0usize) += 1;
    }
    let mut count = 0;
    for c in face.loops.iter().flat_map(|l| l.coedges.iter()) {
        let Some(edge) = solid.edges.iter().find(|e| e.id == c.edge_id) else {
            decline!();
        };
        if uses[&c.edge_id] != 1 || edge.degenerate {
            continue;
        }
        count += 1;
        let curve = &edge.curve;
        if curve.degree != 2
            || curve.knots.windows(2).any(|k| k[1] < k[0])
            || NurbsCurve::new(
                curve.degree,
                curve.knots.clone(),
                curve.control_points.clone(),
            )
            .is_err()
        {
            decline!();
        }
        // The rational basis is a positive partition. Bound all homogeneous
        // inputs before reconstructing local Bezier controls, so cancellation
        // at the interpolation sites cannot hide oversized input controls.
        if curve.control_points.iter().any(|p| {
            p.w < 0.49
                || p.w > 1.01
                || !p.w.is_finite()
                || !Vec3::new(
                    p.x - origin.x * p.w,
                    p.y - origin.y * p.w,
                    p.z - origin.z * p.w,
                )
                .length()
                .is_finite()
                || Vec3::new(
                    p.x - origin.x * p.w,
                    p.y - origin.y * p.w,
                    p.z - origin.z * p.w,
                )
                .length()
                    > 2.0 * ball.radius
        }) {
            decline!();
        }
        let Ok(domain) = curve.domain() else {
            decline!();
        };
        if curve.knots[..=curve.degree].iter().any(|k| *k != domain[0])
            || curve.knots[curve.knots.len() - 1 - curve.degree..]
                .iter()
                .any(|k| *k != domain[1])
        {
            decline!();
        }
        if edge.t0 < domain[0] || edge.t1 > domain[1] || edge.t1 <= edge.t0 {
            decline!();
        }
        let mut breaks = vec![edge.t0];
        breaks.extend(
            curve
                .knots
                .iter()
                .copied()
                .filter(|t| *t > edge.t0 && *t < edge.t1),
        );
        breaks.push(edge.t1);
        breaks.dedup();
        if breaks.iter().any(|t| {
            *t > edge.t0 && *t < edge.t1 && curve.knots.iter().filter(|k| **k == *t).count() > 2
        }) {
            decline!();
        }
        let mut spans: Vec<_> = breaks.windows(2).map(|p| (p[0], p[1])).collect();
        if !c.forward {
            spans.reverse();
            for p in &mut spans {
                std::mem::swap(&mut p.0, &mut p.1);
            }
        }
        for (a, b) in spans {
            let round = round
                + 4096.0
                    * f64::EPSILON
                    * (1.0 + ball.radius + origin.length())
                    * (1.0 + a.abs() + b.abs())
                    / (b - a).abs();
            if !round.is_finite() || round * 16.0 >= band {
                decline!();
            }
            let (Ok(p), Ok(mid), Ok(q)) = (
                curve.evaluate_homogeneous(a),
                curve.evaluate_homogeneous((a + b) * 0.5),
                curve.evaluate_homogeneous(b),
            ) else {
                decline!();
            };
            let h = [
                p,
                crate::Vec4 {
                    x: 2. * mid.x - 0.5 * (p.x + q.x),
                    y: 2. * mid.y - 0.5 * (p.y + q.y),
                    z: 2. * mid.z - 0.5 * (p.z + q.z),
                    w: 2. * mid.w - 0.5 * (p.w + q.w),
                },
                q,
            ];
            let w = h.map(|p| p.w);
            let wmin = w.iter().copied().fold(f64::INFINITY, f64::min);
            if wmin <= 0.49 || !w.iter().all(|w| w.is_finite() && *w <= 1.01) {
                decline!();
            }
            let local = h.map(|p| {
                Vec3::new(
                    p.x - origin.x * p.w,
                    p.y - origin.y * p.w,
                    p.z - origin.z * p.w,
                )
            });
            if local
                .iter()
                .any(|p| !p.length().is_finite() || p.length() > 2.0 * ball.radius)
            {
                decline!();
            }
            if local.iter().any(|p| n.dot(*p).abs() + round > band * wmin) {
                decline!();
            }
            let centre = h.map(|p| {
                Vec3::new(
                    p.x - ball.centre.x * p.w,
                    p.y - ball.centre.y * p.w,
                    p.z - ball.centre.z * p.w,
                )
            });
            let xx = product(&centre.map(|p| p.x), &centre.map(|p| p.x));
            let yy = product(&centre.map(|p| p.y), &centre.map(|p| p.y));
            let zz = product(&centre.map(|p| p.z), &centre.map(|p| p.z));
            let ww = product(&w, &w);
            if (0..5).any(|i| {
                (xx[i] + yy[i] + zz[i] - ball.radius * ball.radius * ww[i]).abs()
                    + round * ball.radius
                    > band * ball.radius * wmin * wmin
            }) {
                decline!();
            }
            let px = local.map(|p| x.dot(p));
            let py = local.map(|p| y.dot(p));
            // The bisector reference covers a native quadrant with a strict
            // open half-plane, so its complete polynomial cannot hide a wrap.
            let reference = (px[0] / w[0] + px[2] / w[2], py[0] / w[0] + py[2] / w[2]);
            if (0..3)
                .any(|i| reference.0 * px[i] + reference.1 * py[i] <= round * ball.radius * 1024.)
            {
                decline!();
            }
            rotation += (px[0] * py[2] - py[0] * px[2]).atan2(px[0] * px[2] + py[0] * py[2]);
            let (Ok(start), Ok(end)) = (p.point(), q.point()) else {
                decline!();
            };
            if last.is_some_and(|last| last.sub(start).length() > band / 16.) {
                decline!();
            }
            if first.is_none() {
                first = Some(start);
            }
            last = Some(end);
        }
    }
    let (Some(first), Some(last)) = (first, last) else {
        decline!();
    };
    count == expected_edges
        && rotation.is_finite()
        && (rotation.abs() - std::f64::consts::TAU).abs() < 1e-6
        && first.sub(last).length() < band / 16.
}

