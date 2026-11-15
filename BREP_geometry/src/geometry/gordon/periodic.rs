//! Periodic cubic cardinal splines, represented on a clamped common knot
//! vector. The endpoint jets agree through order two; clamping is only a
//! representation choice for the exact compatibility algebra.
use super::*;

pub(super) fn direction_cardinals(parameters: &[f64], periodic: bool)
    -> Result<(usize, Vec<NurbsCurve>), String>
{
    if !periodic {
        return cardinal_functions(parameters);
    }
    let n = parameters.len();
    let mut h: Vec<f64> = parameters.windows(2).map(|p| p[1] - p[0]).collect();
    h.push(1.0 + parameters[0] - parameters[n - 1]);
    if h.iter().any(|&span| span <= KNOT_IDENTITY_TOL) {
        return Err("periodic cardinal sites coincide across the seam".into());
    }
    // Cyclic cubic-spline equations for the second derivatives. Use +=:
    // with two sites, the previous and next site are the same unknown.
    let mut matrix = vec![vec![0.0; n]; n];
    for i in 0..n {
        let prev = (i + n - 1) % n;
        matrix[i][prev] += h[prev];
        matrix[i][i] += 2.0 * (h[prev] + h[i]);
        matrix[i][(i + 1) % n] += h[i];
    }
    let mut breaks = vec![0.0];
    breaks.extend(parameters.iter().copied().filter(|&t| t > KNOT_IDENTITY_TOL));
    breaks.push(1.0);
    let mut knots = vec![0.0; 4];
    for &t in &breaks[1..breaks.len() - 1] {
        knots.extend([t; 3]);
    }
    knots.extend([1.0; 4]);
    let mut functions = Vec::with_capacity(n);
    for site in 0..n {
        let mut values = vec![0.0; n];
        values[site] = 1.0;
        let rhs = (0..n).map(|i| {
            let prev = (i + n - 1) % n;
            6.0 * ((values[(i + 1) % n] - values[i]) / h[i]
                - (values[i] - values[prev]) / h[prev])
        }).collect();
        let second = solve_dense(matrix.clone(), rhs)?;
        let mut points = Vec::new();
        for interval in breaks.windows(2) {
            let (a, b) = (interval[0], interval[1]);
            let mid = 0.5 * (a + b);
            let i = parameters.iter().rposition(|&t| t <= mid).unwrap_or(n - 1);
            let origin = if mid < parameters[0] { parameters[i] - 1.0 } else { parameters[i] };
            let next = (i + 1) % n;
            let slope = (values[next] - values[i]) / h[i]
                - h[i] * (2.0 * second[i] + second[next]) / 6.0;
            let cubic = (second[next] - second[i]) / (6.0 * h[i]);
            let jet = |t: f64| {
                let x = t - origin;
                (values[i] + x * (slope + x * (second[i] / 2.0 + x * cubic)),
                    slope + x * (second[i] + 3.0 * x * cubic))
            };
            let (fa, da) = jet(a);
            let (fb, db) = jet(b);
            if points.is_empty() { points.push(Vec4 { x: fa, y: 0.0, z: 0.0, w: 1.0 }); }
            for value in [fa + (b - a) * da / 3.0, fb - (b - a) * db / 3.0, fb] {
                points.push(Vec4 { x: value, y: 0.0, z: 0.0, w: 1.0 });
            }
        }
        functions.push(NurbsCurve::new(3, knots.clone(), points)?);
    }
    Ok((3, functions))
}

/// Lift a Euclidean cardinal basis into a family's common homogeneous gauge.
/// Without this factor, rational curves can agree in position at a seam but
/// the projective Boolean sum can acquire a derivative jump between curves.
/// Multiplication is exact Bernstein algebra, not a sampled fit.
pub(super) fn weight_cardinals(family: char, curves: &[NurbsCurve],
    cardinals: Vec<NurbsCurve>, parameters: &[f64]) -> Result<Vec<NurbsCurve>, GordonRefusal>
{
    let combined: Vec<_> = curves.iter().chain(&cardinals).cloned().collect();
    let (mut compatible, _) = compat::make_family_compatible(&combined, 1)
        .map_err(|detail| GordonRefusal::Incompatible { family, detail })?;
    let reference = &compatible[0];
    let w0 = reference.control_points[0].w;
    for curve in &compatible[..curves.len()] {
        let scale = curve.control_points[0].w / w0;
        if curve.control_points.iter().zip(&reference.control_points)
            .any(|(a, b)| (a.w - scale * b.w).abs() > WEIGHT_CLOSURE_BAND * a.w.abs().max((scale * b.w).abs())) {
            return Err(GordonRefusal::Incompatible { family,
                detail: "closed-network curves need proportional weight functions in each family".into() });
        }
    }
    // Constant denominators need no lifting and retain the smallest degree.
    if reference.control_points.iter().all(|p| p.w == w0) { return Ok(cardinals); }
    let degree = reference.degree;
    let mut breaks = vec![0.0];
    for &k in &reference.knots {
        if k > *breaks.last().unwrap() && k < 1.0 { breaks.push(k); }
    }
    breaks.push(1.0);
    for &k in &breaks[1..breaks.len() - 1] {
        let multiplicity = compatible[0].knots.iter().filter(|&&t| t == k).count();
        if multiplicity > degree {
            return Err(GordonRefusal::Incompatible { family,
                detail: "closed-network weight multiplication requires continuous curves".into() });
        }
        for curve in &mut compatible {
            for _ in multiplicity..degree { *curve = curve.insert_knot(k, 1).map_err(geometry)?; }
        }
    }
    let denominator = &compatible[0];
    let binomial = |n: usize, k: usize| -> f64 {
        (0..k.min(n-k)).fold(1.0, |value, i| value * (n-i) as f64 / (i+1) as f64)
    };
    let product_degree = 2 * degree;
    let mut knots = vec![0.0; product_degree + 1];
    for &t in &breaks[1..breaks.len() - 1] { knots.extend(vec![t; product_degree]); }
    knots.extend(vec![1.0; product_degree + 1]);
    let mut result = Vec::new();
    for (site, cardinal) in compatible[curves.len()..].iter().enumerate() {
        let divisor = denominator.evaluate_homogeneous(parameters[site]).map_err(geometry)?.w;
        let mut points = Vec::new();
        for span in 0..breaks.len() - 1 {
            for k in usize::from(span > 0)..=product_degree {
                let value = (k.saturating_sub(degree)..=degree.min(k)).map(|i| {
                    cardinal.control_points[span * degree + i].x
                        * denominator.control_points[span * degree + k - i].w
                        * binomial(degree, i) * binomial(degree, k-i) / binomial(product_degree, k)
                }).sum::<f64>() / divisor;
                points.push(Vec4 { x: value, y: 0.0, z: 0.0, w: 1.0 });
            }
        }
        result.push(NurbsCurve::new(product_degree, knots.clone(), points).map_err(geometry)?);
    }
    Ok(result)
}
