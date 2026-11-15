//! The trimmed sub-curve an edge uses of its carrier curve.

use crate::topology::EdgeRecord;
use crate::NurbsCurve;


pub fn edge_subcurve(edge: &EdgeRecord) -> Result<NurbsCurve, String> {
    let [start, end] = edge.curve.domain()?;
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut curve = edge.curve.clone();
    if edge.t0 > start + epsilon && edge.t0 < end - epsilon {
        curve = curve.split(edge.t0)?.1;
    }
    let domain = curve.domain()?;
    if edge.t1 < domain[1] - epsilon && edge.t1 > domain[0] + epsilon {
        curve = curve.split(edge.t1)?.0;
    }
    Ok(curve)
}
