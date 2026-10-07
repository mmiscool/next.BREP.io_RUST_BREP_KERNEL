//! Algebraic plane sections of a rational Bezier carrier linear in v.
//! Both curves share the u parameter; no sampled interpolation is involved.
use crate::{NurbsCurve, NurbsSurface, Vec4};

pub(crate) struct PlaneSectionGraph {
    pub curve: NurbsCurve,
    pub pcurve: NurbsCurve,
}

fn product(a: &[f64], b: &[f64]) -> Vec<f64> {
    fn choose(n: usize, k: usize) -> f64 {
        (0..k.min(n - k)).fold(1.0, |v, i| v * (n - i) as f64 / (i + 1) as f64)
    }
    let (p, q) = (a.len() - 1, b.len() - 1);
    (0..=p + q)
        .map(|k| {
            (k.saturating_sub(q)..=k.min(p))
                .map(|i| a[i] * b[k - i] * choose(p, i) * choose(q, k - i) / choose(p + q, k))
                .sum()
        })
        .collect()
}

/// Restriction and span decomposition use homogeneous knot insertion. Each
/// stored polynomial span gets its own algebraic graph. Full-multiplicity
/// joins preserve each segment's weights and parameter, with coincident
/// Euclidean endpoints required within floating-point construction error.
pub(crate) fn plane_section_graph_range(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    range: [f64; 2],
) -> Option<PlaneSectionGraph> {
    range_impl(plane, ruled, range, 0.0, 0.0)
}

/// Explicitly budgeted construction on the actual bilinear partner image.
/// Equal ruling weights also support bounded native v continuation. The
/// default exact constructor retains its strict represented-domain contract.
pub(crate) fn plane_section_graph_range_bounded(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    range: [f64; 2],
    tolerance: f64,
    max_extension: f64,
) -> Option<PlaneSectionGraph> {
    if !tolerance.is_finite()
        || tolerance < 0.0
        || !max_extension.is_finite()
        || max_extension < 0.0
    {
        return None;
    }
    range_impl(plane, ruled, range, tolerance, max_extension)
}
/// Restrict one native U span without an endpoint identity snap. Full knot
/// insertion exposes its positive Bernstein net, then de Casteljau clips the
/// requested interval. The caller partitions at every original knot.
fn restrict_u_span(surface: &NurbsSurface, range: [f64; 2]) -> Option<NurbsSurface> {
    let p = surface.degree_u;
    if p == 0 || p > 8 || range[0] >= range[1] {
        return None;
    }
    let pair = surface
        .knots_u
        .windows(2)
        .find(|k| k[0] <= range[0] && k[1] >= range[1] && k[0] < k[1])?;
    let (a, b) = (pair[0], pair[1]);
    fn divide(cp: &[Vec4], t: f64) -> (Vec<Vec4>, Vec<Vec4>) {
        let mut w = cp.to_vec();
        let mut l = vec![w[0]];
        let mut r = vec![*w.last().unwrap()];
        for n in (1..cp.len()).rev() {
            for i in 0..n {
                w[i] = Vec4 {
                    x: (1.0 - t) * w[i].x + t * w[i + 1].x,
                    y: (1.0 - t) * w[i].y + t * w[i + 1].y,
                    z: (1.0 - t) * w[i].z + t * w[i + 1].z,
                    w: (1.0 - t) * w[i].w + t * w[i + 1].w,
                };
            }
            l.push(w[0]);
            r.push(w[n - 1]);
        }
        r.reverse();
        (l, r)
    }
    let mut columns = Vec::new();
    for j in 0..surface.control_points.first()?.len() {
        let mut c = NurbsCurve::new(
            p,
            surface.knots_u.clone(),
            surface.control_points.iter().map(|r| r[j]).collect(),
        )
        .ok()?;
        for t in [a, b] {
            let mult = c.knots.iter().filter(|&&k| k == t).count();
            if mult < p {
                c = c.insert_knot(t, p - mult).ok()?;
            }
            if c.knots.iter().filter(|&&k| k == t).count() < p {
                return None;
            }
        }
        let k = (p..c.control_points.len()).find(|&k| c.knots[k] == a && c.knots[k + 1] == b)?;
        let mut cp = c.control_points[k - p..=k].to_vec();
        if range[1] < b {
            cp = divide(&cp, (range[1] - a) / (b - a)).0;
        }
        if range[0] > a {
            cp = divide(&cp, (range[0] - a) / (range[1] - a)).1;
        }
        if cp
            .iter()
            .any(|h| h.w <= 0.0 || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()))
        {
            return None;
        }
        columns.push(cp);
    }
    let controls = (0..=p)
        .map(|i| columns.iter().map(|c| c[i]).collect())
        .collect();
    NurbsSurface::new(
        p,
        surface.degree_v,
        [vec![range[0]; p + 1], vec![range[1]; p + 1]].concat(),
        surface.knots_v.clone(),
        controls,
    )
    .ok()
}

fn range_impl(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    range: [f64; 2],
    tolerance: f64,
    max_extension: f64,
) -> Option<PlaneSectionGraph> {
    let domain = ruled.domain_u().ok()?;
    if range[0] < domain[0] || range[1] > domain[1] || range[0] >= range[1] {
        return None;
    }
    let mut breaks = vec![range[0], range[1]];
    breaks.extend(
        ruled
            .knots_u
            .iter()
            .copied()
            .filter(|&t| t > range[0] && t < range[1]),
    );
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    if breaks.len() > 33 {
        return None;
    }
    fn join(a: NurbsCurve, b: NurbsCurve) -> Option<NurbsCurve> {
        if a.degree != b.degree {
            return None;
        }
        let end = a.domain().ok()?[1];
        if end != b.domain().ok()?[0] {
            return None;
        }
        let x = a.evaluate(end).ok()?;
        let y = b.evaluate(end).ok()?;
        if x.sub(y).length() > 4096.0 * f64::EPSILON * (1.0 + x.length().max(y.length())) {
            return None;
        }
        let mut knots = a.knots;
        knots.extend_from_slice(&b.knots[b.degree + 1..]);
        let mut controls = a.control_points;
        controls.extend(b.control_points);
        NurbsCurve::new(b.degree, knots, controls).ok()
    }
    let mut result: Option<PlaneSectionGraph> = None;
    for span in breaks.windows(2) {
        let patch = restrict_u_span(ruled, [span[0], span[1]])?;
        if patch.domain_u().ok()? != [span[0], span[1]] {
            return None;
        }
        let next = graph_impl(plane, &patch, tolerance, max_extension)?;
        result = Some(match result {
            None => next,
            Some(previous) => PlaneSectionGraph {
                curve: join(previous.curve, next.curve)?,
                pcurve: join(previous.pcurve, next.pcurve)?,
            },
        });
    }
    result
}

/// Declines poles, roots outside the stored patch, multiple spans, and high
/// degrees. Positive Bernstein weights bound the entire graph, including its
/// UV domain, rather than just stations. Open carrier extrapolation is never
/// treated as continuation of its rational polynomial.
pub(crate) fn plane_section_graph(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
) -> Option<PlaneSectionGraph> {
    graph_impl(plane, ruled, 0.0, 0.0)
}
fn graph_impl(
    plane: &NurbsSurface,
    ruled: &NurbsSurface,
    tolerance: f64,
    max_extension: f64,
) -> Option<PlaneSectionGraph> {
    if !plane.is_affine().ok()? || ruled.degree_v != 1 || ruled.degree_u == 0 || ruled.degree_u > 8
    {
        return None;
    }
    // Approximate affinity is only a classification. A nonzero bilinear
    // warp is not a plane under extrapolation, however small its controls.
    if plane.degree_u != 1
        || plane.degree_v != 1
        || plane.control_points.len() != 2
        || plane.control_points.iter().any(|r| r.len() != 2)
        || plane
            .control_points
            .iter()
            .flatten()
            .any(|h| h.w <= 0.0 || h.w != plane.control_points[0][0].w)
        || plane.closed_directions().ok()? != (false, false)
    {
        return None;
    }
    let corners: Vec<_> = plane
        .control_points
        .iter()
        .flatten()
        .map(|h| h.point())
        .collect::<Result<_, _>>()
        .ok()?;
    let warp = corners[3].sub(corners[2]).sub(corners[1]).add(corners[0]);
    if tolerance == 0.0 && (warp.x != 0.0 || warp.y != 0.0 || warp.z != 0.0) {
        return None;
    }
    let p = ruled.degree_u;
    if ruled.control_points.len() != p + 1 || ruled.control_points.iter().any(|r| r.len() != 2) {
        return None;
    }
    let [u0, u1] = ruled.domain_u().ok()?;
    let [v0, v1] = ruled.domain_v().ok()?;
    if ruled.knots_u.len() != 2 * (p + 1)
        || !ruled.knots_u[..=p].iter().all(|&u| u == u0)
        || !ruled.knots_u[p + 1..].iter().all(|&u| u == u1)
        || ruled.knots_v != vec![v0, v0, v1, v1]
    {
        return None;
    }
    let [pu, _] = plane.domain_u().ok()?;
    let [pv, _] = plane.domain_v().ok()?;
    let d = plane.derivatives(pu, pv, 1).ok()?;
    let n = d[1][0].cross(d[0][1]).normalized().ok()?;
    let offset = n.dot(d[0][0]);
    let form = |h: Vec4| n.x * h.x + n.y * h.y + n.z * h.z - offset * h.w;
    let a: Vec<_> = ruled.control_points.iter().map(|r| r[0]).collect();
    let b: Vec<_> = ruled.control_points.iter().map(|r| r[1]).collect();
    if a.iter().chain(&b).any(|h| !h.w.is_finite() || h.w <= 0.0) {
        return None;
    }
    let f: Vec<_> = a.iter().copied().map(form).collect();
    let g: Vec<_> = a.iter().zip(&b).map(|(&a, &b)| form(b) - form(a)).collect();
    let sign = if g.iter().all(|&g| g > 0.0) {
        1.0
    } else if g.iter().all(|&g| g < 0.0) {
        -1.0
    } else {
        return None;
    };
    let wg = product(&g, &vec![1.0, 1.0]);
    let uf = product(&g, &[u0, u1]);
    let vf: Vec<_> = g
        .iter()
        .zip(&f)
        .map(|(&g, &f)| v0 * g - (v1 - v0) * f)
        .collect();
    let vg = product(&vf, &[1.0, 1.0]);
    let uv: Vec<_> = (0..wg.len())
        .map(|i| Vec4 {
            x: sign * uf[i],
            y: sign * vg[i],
            z: 0.0,
            w: sign * wg[i],
        })
        .collect();
    if uv
        .iter()
        .any(|h| ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()) || h.w <= 0.0)
    {
        return None;
    }
    let excursion = uv
        .iter()
        .map(|h| (v0 - h.y / h.w).max(h.y / h.w - v1).max(0.0))
        .fold(0.0_f64, f64::max);
    if excursion > 0.0 {
        if max_extension == 0.0 || ruled.control_points.iter().any(|r| r[0].w != r[1].w) {
            return None;
        }
        let mut ruling = 0.0_f64;
        for row in &ruled.control_points {
            ruling = ruling.max(row[1].point().ok()?.sub(row[0].point().ok()?).length());
        }
        if excursion * ruling / (v1 - v0) > max_extension {
            return None;
        }
    }
    let coord = |h: &Vec4, c| match c {
        0 => h.x,
        1 => h.y,
        2 => h.z,
        _ => h.w,
    };
    let mut net = vec![
        Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0
        };
        2 * p + 1
    ];
    for c in 0..4 {
        let ac: Vec<_> = a.iter().map(|h| coord(h, c)).collect();
        let delta: Vec<_> = a
            .iter()
            .zip(&b)
            .map(|(a, b)| coord(b, c) - coord(a, c))
            .collect();
        let ag = product(&ac, &g);
        let df = product(&delta, &f);
        for i in 0..net.len() {
            let value = sign * (ag[i] - df[i]);
            match c {
                0 => net[i].x = value,
                1 => net[i].y = value,
                2 => net[i].z = value,
                _ => net[i].w = value,
            }
        }
    }
    if net
        .iter()
        .any(|h| ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()) || h.w <= 0.0)
    {
        return None;
    }
    if tolerance > 0.0 {
        let uu = d[1][0].dot(d[1][0]);
        let vv = d[0][1].dot(d[0][1]);
        let cross = d[1][0].dot(d[0][1]);
        let determinant = uu * vv - cross * cross;
        if determinant <= 0.0 || !determinant.is_finite() {
            return None;
        }
        let pu_domain = plane.domain_u().ok()?;
        let pv_domain = plane.domain_v().ok()?;
        let (mut umax, mut vmax, mut off, mut scale) = (0.0_f64, 0.0_f64, 0.0_f64, 1.0_f64);
        for h in &net {
            let point = h.point().ok()?;
            let delta = point.sub(d[0][0]);
            let x = delta.dot(d[1][0]);
            let y = delta.dot(d[0][1]);
            let u = (x * vv - y * cross) / determinant;
            let v = (y * uu - x * cross) / determinant;
            if !u.is_finite() || !v.is_finite() {
                return None;
            }
            umax = umax.max((u / (pu_domain[1] - pu_domain[0])).abs());
            vmax = vmax.max((v / (pv_domain[1] - pv_domain[0])).abs());
            off = off.max(delta.sub(d[1][0].scale(u)).sub(d[0][1].scale(v)).length());
            scale = scale.max(point.length() + d[0][0].length());
        }
        let bound = off + warp.length() * umax * vmax + 4096.0 * f64::EPSILON * scale;
        if !bound.is_finite() || bound > tolerance {
            return None;
        }
    }
    let knots = |degree| [vec![u0; degree + 1], vec![u1; degree + 1]].concat();
    Some(PlaneSectionGraph {
        curve: NurbsCurve::new(2 * p, knots(2 * p), net).ok()?,
        pcurve: NurbsCurve::new(p + 1, knots(p + 1), uv).ok()?,
    })
}

