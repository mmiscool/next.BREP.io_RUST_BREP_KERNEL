//! Polynomial roots shared by geometric intersections.
const DEGENERATE: f64 = 1e-9;

/// Real quadratic roots, in minus-square-root then plus-square-root order.
/// Linear/constant degeneracy thresholds remain the caller's responsibility.
/// A repeated root is returned twice to preserve two-branch callers.
pub fn solve_quadratic(c0: f64, c1: f64, c2: f64) -> Vec<f64> {
    if c2 == 0.0 {
        return if c1 != 0.0 { vec![-c0 / c1] } else { Vec::new() };
    }
    let discriminant = c1 * c1 - 4.0 * c2 * c0;
    if !(discriminant >= 0.0) {
        return Vec::new();
    }
    let q = -0.5 * (c1 + c1.signum() * discriminant.sqrt());
    if q == 0.0 {
        return vec![-c1 / (2.0 * c2); 2];
    }
    if c1.is_sign_negative() {
        vec![c0 / q, q / c2]
    } else {
        vec![q / c2, c0 / q]
    }
}

/// Real roots up to degree four; decline near-multiple roots and ill-scaled leading terms.
pub fn real_roots(coefficients: &[f64]) -> Option<Vec<f64>> {
    let mut coefficients: Vec<f64> = coefficients.to_vec();
    if coefficients.iter().any(|c| !c.is_finite()) {
        return None;
    }
    let largest = coefficients.iter().fold(0.0_f64, |m, c| m.max(c.abs()));
    if !(largest > 0.0) {
        return None;
    }
    let leading = *coefficients.last()?;
    if leading.abs() <= DEGENERATE * largest {
        return None;
    }
    while coefficients.len() > 1 && coefficients.last() == Some(&0.0) {
        coefficients.pop();
    }
    let degree = coefficients.len() - 1;
    if degree == 0 {
        return Some(Vec::new());
    }
    let evaluate = |s: f64| coefficients.iter().rev().fold(0.0, |acc, c| acc * s + c);
    let terms = |s: f64| coefficients.iter().enumerate().fold(0.0, |acc, (i, c)| acc + (c * s.powi(i as i32)).abs());
    if degree == 1 {
        return Some(vec![-coefficients[0] / coefficients[1]]);
    }
    // Cauchy's bound on the root magnitude.
    let bound = 1.0 + coefficients[..degree].iter().map(|c| (c / leading).abs()).fold(0.0, f64::max);
    let derivative: Vec<f64> = coefficients.iter().enumerate().skip(1).map(|(i, c)| c * i as f64).collect();
    let mut critical = real_roots(&derivative)?;
    critical.retain(|c| c.abs() <= bound);
    critical.sort_by(f64::total_cmp);
    let mut nodes = vec![-bound];
    nodes.extend(critical.iter().copied());
    nodes.push(bound);
    // A critical value at zero is a double root: decline.
    for &c in &critical {
        if evaluate(c).abs() <= DEGENERATE * terms(c).max(largest * DEGENERATE) {
            return None;
        }
    }
    let mut roots = Vec::new();
    for pair in nodes.windows(2) {
        let (mut lo, mut hi) = (pair[0], pair[1]);
        let (mut f_lo, f_hi) = (evaluate(lo), evaluate(hi));
        if f_lo == 0.0 {
            roots.push(lo);
            continue;
        }
        if f_lo.signum() == f_hi.signum() {
            continue;
        }
        for _ in 0..200 {
            let mid = 0.5 * (lo + hi);
            if mid <= lo || mid >= hi {
                break;
            }
            let f_mid = evaluate(mid);
            if f_mid == 0.0 {
                lo = mid;
                hi = mid;
                break;
            }
            if f_mid.signum() == f_lo.signum() {
                lo = mid;
                f_lo = f_mid;
            } else {
                hi = mid;
            }
        }
        let mut root = 0.5 * (lo + hi);
        for _ in 0..3 {
            let slope = derivative.iter().rev().fold(0.0, |acc, c| acc * root + c);
            if slope == 0.0 {
                break;
            }
            let next = root - evaluate(root) / slope;
            if !next.is_finite() || next < pair[0] || next > pair[1] {
                break;
            }
            root = next;
        }
        roots.push(root);
    }
    if let Some(&last) = nodes.last() {
        if evaluate(last) == 0.0 {
            roots.push(last);
        }
    }
    Some(roots)
}

