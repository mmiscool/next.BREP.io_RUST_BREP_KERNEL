//! A curve trimmed to a parameter range, keeping its parameterization.

use crate::NurbsCurve;


/// A curve trimmed to the parameter range `[t0, t1]` (the `coalesce.rs` pattern):
/// skip a split whose parameter sits within [`crate::NurbsCurve::split`]'s absolute
/// knot tolerance of a domain end, or the guard inside `split` rejects it. The
/// result keeps the original parameterization, so its domain IS `[t0, t1]`.
pub fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    let [start, end] = curve.domain()?;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if t0 > start + epsilon && t0 < end - epsilon {
        result = result.split(t0)?.1;
    }
    let domain = result.domain()?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        result = result.split(t1)?.0;
    }
    Ok(result)
}
