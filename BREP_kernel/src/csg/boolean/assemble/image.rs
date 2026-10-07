//! Positive rational images and bounds, without approximate affine claims.
use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

pub(super) fn product(a: &[f64], b: &[f64]) -> Vec<f64> {
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

/// Exact Bernstein extraction by knot insertion; full-multiplicity joins
/// keep their separate endpoint weights. No call to curve.split is needed.
pub(super) fn bezier_spans(curve: &NurbsCurve) -> Option<Vec<NurbsCurve>> {
    let p = curve.degree;
    if p == 0
        || p > 16
        || curve
            .control_points
            .iter()
            .any(|h| h.w <= 0.0 || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()))
    {
        return None;
    }
    let [a, b] = curve.domain().ok()?;
    if !curve.knots[..=p].iter().all(|&t| t == a)
        || !curve.knots[curve.knots.len() - p - 1..]
            .iter()
            .all(|&t| t == b)
    {
        return None;
    }
    let mut breaks: Vec<_> = curve
        .knots
        .iter()
        .copied()
        .filter(|&t| t > a && t < b)
        .collect();
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    if breaks.len() > 4096 {
        return None;
    }
    let mut refined = curve.clone();
    for t in breaks {
        let mult = refined.knots.iter().filter(|&&k| k == t).count();
        if mult > p + 1 {
            return None;
        }
        if mult < p {
            refined = refined.insert_knot(t, p - mult).ok()?;
        }
    }
    let mut internal: Vec<_> = refined
        .knots
        .iter()
        .copied()
        .filter(|&t| t > a && t < b)
        .collect();
    internal.sort_by(f64::total_cmp);
    internal.dedup();
    if internal
        .iter()
        .any(|t| refined.knots.iter().filter(|&&k| k == *t).count() < p)
    {
        return None;
    }
    let mut spans = Vec::new();
    for k in p..refined.control_points.len() {
        let (a, b) = (refined.knots[k], refined.knots[k + 1]);
        if b > a {
            spans.push(
                NurbsCurve::new(
                    p,
                    [vec![a; p + 1], vec![b; p + 1]].concat(),
                    refined.control_points[k - p..=k].to_vec(),
                )
                .ok()?,
            );
        }
    }
    Some(spans)
}

fn casteljau(cp: &[Vec4], fraction: f64) -> (Vec<Vec4>, Vec<Vec4>) {
    let mut work = cp.to_vec();
    let mut left = vec![work[0]];
    let mut right = vec![*work.last().unwrap()];
    for count in (1..cp.len()).rev() {
        for i in 0..count {
            let (a, b) = (work[i], work[i + 1]);
            work[i] = Vec4 {
                x: (1.0 - fraction) * a.x + fraction * b.x,
                y: (1.0 - fraction) * a.y + fraction * b.y,
                z: (1.0 - fraction) * a.z + fraction * b.z,
                w: (1.0 - fraction) * a.w + fraction * b.w,
            };
        }
        left.push(work[0]);
        right.push(work[count - 1]);
    }
    right.reverse();
    (left, right)
}

/// All controls of exact restricted Bernstein spans. De Casteljau remains
/// valid for tiny ranges and full-multiplicity joins; knot snapping cannot
/// accidentally widen or corrupt a requested bound.
pub(super) fn restricted_controls(curve: &NurbsCurve, a: f64, b: f64) -> Option<Vec<Vec4>> {
    let [lo, hi] = curve.domain().ok()?;
    if a < lo || b > hi || a >= b {
        return None;
    }
    restricted_bezier_controls(&bezier_spans(curve)?, a, b)
}

/// Reuse immutable extraction across the bounded subdivision tree. This
/// avoids repeating knot insertion for every narrow clearance tile.
pub(super) fn restricted_bezier_controls(
    spans: &[NurbsCurve],
    a: f64,
    b: f64,
) -> Option<Vec<Vec4>> {
    let lo = spans.first()?.domain().ok()?[0];
    let hi = spans.last()?.domain().ok()?[1];
    if a < lo || b > hi || a > b {
        return None;
    }
    let mut result = Vec::new();
    for span in spans {
        let [x, y] = span.domain().ok()?;
        if b < x || a > y {
            continue;
        }
        let start = a.max(x);
        let end = b.min(y);
        if start > end {
            continue;
        }
        let mut cp = span.control_points.clone();
        if end < y {
            cp = casteljau(&cp, (end - x) / (y - x)).0;
        }
        if start > x {
            let fraction = if end > x {
                (start - x) / (end - x)
            } else {
                0.0
            };
            cp = casteljau(&cp, fraction.clamp(0.0, 1.0)).1;
        }
        result.extend(cp);
    }
    (!result.is_empty()).then_some(result)
}

pub(super) fn restricted_single_span_controls(
    spans: &[NurbsCurve],
    a: f64,
    b: f64,
) -> Option<Vec<Vec4>> {
    for span in spans {
        let [x, y] = span.domain().ok()?;
        if a < x || b > y || a >= b {
            continue;
        }
        let mut cp = span.control_points.clone();
        if b < y {
            cp = casteljau(&cp, (b - x) / (y - x)).0;
        }
        if a > x {
            cp = casteljau(&cp, (a - x) / (b - x)).1;
        }
        return Some(cp);
    }
    None
}
fn single_span_controls(curve: &NurbsCurve, a: f64, b: f64) -> Option<Vec<Vec4>> {
    restricted_single_span_controls(&bezier_spans(curve)?, a, b)
}

pub(super) fn restricted_curve(curve: &NurbsCurve, a: f64, b: f64) -> Option<NurbsCurve> {
    let domain = curve.domain().ok()?;
    if a < domain[0] || b > domain[1] || a >= b {
        return None;
    }
    if [a, b] == domain {
        return Some(curve.clone());
    }
    let mut result = None;
    for span in bezier_spans(curve)? {
        let [x, y] = span.domain().ok()?;
        let (lo, hi) = (a.max(x), b.min(y));
        if lo >= hi {
            continue;
        }
        let cp = single_span_controls(&span, lo, hi)?;
        let part = NurbsCurve::new(
            span.degree,
            [vec![lo; span.degree + 1], vec![hi; span.degree + 1]].concat(),
            cp,
        )
        .ok()?;
        result = Some(match result {
            None => part,
            Some(old) => join(old, part)?,
        });
    }
    let result: NurbsCurve = result?;
    for t in [a, b] {
        let p = curve.evaluate(t).ok()?;
        let q = result.evaluate(t).ok()?;
        if p.sub(q).length() > 64.0 * f64::EPSILON * (1.0 + p.length().max(q.length())) {
            return None;
        }
    }
    Some(result)
}

fn join(a: NurbsCurve, b: NurbsCurve) -> Option<NurbsCurve> {
    if a.degree != b.degree || a.domain().ok()?[1] != b.domain().ok()?[0] {
        return None;
    }
    let mut knots = a.knots;
    knots.extend_from_slice(&b.knots[b.degree + 1..]);
    let mut cp = a.control_points;
    cp.extend(b.control_points);
    NurbsCurve::new(b.degree, knots, cp).ok()
}

/// Algebraic inverse of an affine chart. Preserve the represented curve's
/// knots, weights and every control point; authorize only after composing
/// through the actual bilinear carrier, including any nonzero warp.
pub(super) fn affine_inverse(
    sf: &NurbsSurface,
    curve: &NurbsCurve,
    tolerance: f64,
) -> Option<NurbsCurve> {
    if !tolerance.is_finite() || tolerance <= 0.0 || !sf.is_affine().ok()? {
        return None;
    }
    // bilinear_image verifies the finite, positive, constant-weight chart.
    if sf.control_points.len() != 2 || sf.control_points.iter().any(|r| r.len() != 2) {
        return None;
    }
    let [u0, u1] = sf.domain_u().ok()?;
    let [v0, v1] = sf.domain_v().ok()?;
    let origin = sf.control_points[0][0].point().ok()?;
    let a = sf.control_points[1][0].point().ok()?.sub(origin);
    let b = sf.control_points[0][1].point().ok()?.sub(origin);
    let (aa, ab, bb) = (a.dot(a), a.dot(b), b.dot(b));
    let determinant = aa * bb - ab * ab;
    if !determinant.is_finite() || determinant <= 1024.0 * f64::EPSILON * aa * bb {
        return None;
    }
    let controls = curve
        .control_points
        .iter()
        .map(|h| {
            let n = Vec3::new(h.x, h.y, h.z).sub(origin.scale(h.w));
            let alpha = (n.dot(a) * bb - n.dot(b) * ab) / determinant;
            let beta = (n.dot(b) * aa - n.dot(a) * ab) / determinant;
            Vec4 {
                x: u0 * h.w + (u1 - u0) * alpha,
                y: v0 * h.w + (v1 - v0) * beta,
                z: 0.0,
                w: h.w,
            }
        })
        .collect();
    let pc = NurbsCurve::new(curve.degree, curve.knots.clone(), controls).ok()?;
    let actual = bilinear_image(sf, &pc)?;
    (difference_bound(curve, &actual)? <= tolerance).then_some(pc)
}

/// Exact image through a constant-weight bilinear patch, including its
/// native bilinear open-domain extension. Rational parameterized patches
/// and wrapping domains decline. A true affine net preserves input degree;
/// a warped net composes rational polynomials exactly on every span.
pub(super) fn bilinear_image(sf: &NurbsSurface, pc: &NurbsCurve) -> Option<NurbsCurve> {
    if sf.degree_u != 1
        || sf.degree_v != 1
        || sf.control_points.len() != 2
        || sf.control_points.iter().any(|r| r.len() != 2)
        || sf.closed_directions().ok()? != (false, false)
    {
        return None;
    }
    let weight = sf.control_points[0][0].w;
    if weight <= 0.0
        || sf
            .control_points
            .iter()
            .flatten()
            .any(|h| h.w != weight || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()))
    {
        return None;
    }
    let [u0, u1] = sf.domain_u().ok()?;
    let [v0, v1] = sf.domain_v().ok()?;
    if sf.knots_u != vec![u0, u0, u1, u1] || sf.knots_v != vec![v0, v0, v1, v1] {
        return None;
    }
    let p00 = sf.control_points[0][0].point().ok()?;
    let u = sf.control_points[1][0]
        .point()
        .ok()?
        .sub(p00)
        .scale(1.0 / (u1 - u0));
    let v = sf.control_points[0][1]
        .point()
        .ok()?
        .sub(p00)
        .scale(1.0 / (v1 - v0));
    let warp = sf.control_points[1][1]
        .point()
        .ok()?
        .sub(sf.control_points[1][0].point().ok()?)
        .sub(sf.control_points[0][1].point().ok()?)
        .add(p00)
        .scale(1.0 / ((u1 - u0) * (v1 - v0)));
    if pc
        .control_points
        .iter()
        .any(|h| h.w <= 0.0 || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()))
    {
        return None;
    }
    if warp.x == 0.0 && warp.y == 0.0 && warp.z == 0.0 {
        let cp = pc
            .control_points
            .iter()
            .map(|h| {
                let q = p00
                    .scale(h.w)
                    .add(u.scale(h.x - u0 * h.w))
                    .add(v.scale(h.y - v0 * h.w));
                Vec4 {
                    x: q.x,
                    y: q.y,
                    z: q.z,
                    w: h.w,
                }
            })
            .collect();
        return NurbsCurve::new(pc.degree, pc.knots.clone(), cp).ok();
    }
    let coord = |p: Vec3, i| match i {
        0 => p.x,
        1 => p.y,
        _ => p.z,
    };
    let mut result = None;
    for span in bezier_spans(pc)? {
        let p = span.degree;
        if p > 8 {
            return None;
        }
        let [a, b] = span.domain().ok()?;
        let w: Vec<_> = span.control_points.iter().map(|h| h.w).collect();
        let x: Vec<_> = span.control_points.iter().map(|h| h.x - u0 * h.w).collect();
        let y: Vec<_> = span.control_points.iter().map(|h| h.y - v0 * h.w).collect();
        let (ww, xw, yw, xy) = (
            product(&w, &w),
            product(&x, &w),
            product(&y, &w),
            product(&x, &y),
        );
        let mut cp = Vec::new();
        for k in 0..=2 * p {
            let q = std::array::from_fn::<_, 3, _>(|i| {
                coord(p00, i) * ww[k]
                    + coord(u, i) * xw[k]
                    + coord(v, i) * yw[k]
                    + coord(warp, i) * xy[k]
            });
            cp.push(Vec4 {
                x: q[0],
                y: q[1],
                z: q[2],
                w: ww[k],
            });
        }
        if cp
            .iter()
            .any(|h| h.w <= 0.0 || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()))
        {
            return None;
        }
        let image =
            NurbsCurve::new(2 * p, [vec![a; 2 * p + 1], vec![b; 2 * p + 1]].concat(), cp).ok()?;
        result = Some(match result {
            None => image,
            Some(old) => join(old, image)?,
        });
    }
    result
}

/// Bound same-parameter geometric separation by the positive rational
/// difference numerator. Both denominators are included; control-weight
/// differences cannot masquerade as Euclidean agreement.
pub(super) fn difference_bound(a: &NurbsCurve, b: &NurbsCurve) -> Option<f64> {
    if a.domain().ok()? != b.domain().ok()? {
        return None;
    }
    if a.degree == b.degree
        && a.knots == b.knots
        && a.control_points.len() == b.control_points.len()
        && a.control_points
            .iter()
            .zip(&b.control_points)
            .all(|(a, b)| [a.x, a.y, a.z, a.w] == [b.x, b.y, b.z, b.w])
    {
        return Some(0.0);
    }
    let mut breaks = a.knots.clone();
    breaks.extend(&b.knots);
    breaks.sort_by(f64::total_cmp);
    breaks.dedup();
    let mut worst = 0.0_f64;
    for span in breaks.windows(2) {
        let [lo, hi] = a.domain().ok()?;
        if span[0] < lo || span[1] > hi || span[0] >= span[1] {
            continue;
        }
        let (aa, bb) = (
            single_span_controls(a, span[0], span[1])?,
            single_span_controls(b, span[0], span[1])?,
        );
        // A common knot partition restricts each curve to one Bezier span.
        if aa.len() != a.degree + 1 || bb.len() != b.degree + 1 {
            return None;
        }
        worst = worst.max(bezier_difference_bound(aa, bb)?);
    }
    Some(worst)
}

pub(super) fn bezier_difference_bound(mut aa: Vec<Vec4>, mut bb: Vec<Vec4>) -> Option<f64> {
    if aa.is_empty() || bb.is_empty() || aa.len() > 17 || bb.len() > 17 {
        return None;
    }
    let mut worst = 0.0_f64;
    for cp in [&mut aa, &mut bb] {
        let scale = cp.iter().map(|h| h.w).fold(0.0_f64, f64::max);
        if scale <= 0.0 || !scale.is_finite() {
            return None;
        }
        for h in cp.iter_mut() {
            h.x /= scale;
            h.y /= scale;
            h.z /= scale;
            h.w /= scale;
            if h.w <= 0.0 || ![h.x, h.y, h.z, h.w].iter().all(|x| x.is_finite()) {
                return None;
            }
        }
    }
    let (wa, wb): (Vec<_>, Vec<_>) = (
        aa.iter().map(|h| h.w).collect(),
        bb.iter().map(|h| h.w).collect(),
    );
    let denominator = product(&wa, &wb);
    let mut differences = vec![[0.0; 3]; denominator.len()];
    let mut errors = vec![[0.0; 3]; denominator.len()];
    let gamma = 512.0 * f64::EPSILON;
    let tiny = 512.0 * f64::from_bits(1);
    for axis in 0..3 {
        let c = |h: &Vec4| match axis {
            0 => h.x,
            1 => h.y,
            _ => h.z,
        };
        let xa: Vec<_> = aa.iter().map(c).collect();
        let xb: Vec<_> = bb.iter().map(c).collect();
        let (ab, ba) = (product(&xa, &wb), product(&xb, &wa));
        let absolute_a: Vec<_> = xa.iter().map(|x| x.abs()).collect();
        let absolute_b: Vec<_> = xb.iter().map(|x| x.abs()).collect();
        let (absolute_ab, absolute_ba) = (product(&absolute_a, &wb), product(&absolute_b, &wa));
        for k in 0..ab.len() {
            differences[k][axis] = ab[k] - ba[k];
            errors[k][axis] = gamma * (absolute_ab[k] + absolute_ba[k]) + tiny;
        }
    }
    for ((d, error), w) in differences.iter().zip(&errors).zip(&denominator) {
        let lower = w * (1.0 - gamma) - tiny;
        if lower <= 0.0 {
            return None;
        }
        let norm = ((d[0].abs() + error[0]) / lower)
            .hypot((d[1].abs() + error[1]) / lower)
            .hypot((d[2].abs() + error[2]) / lower);
        if !norm.is_finite() {
            return None;
        }
        worst = worst.max(norm);
    }
    Some(worst)
}

