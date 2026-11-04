use crate::{KernelRefusal, KernelStage, OrRefuse, NurbsCurve};

pub(crate) fn trimmed_curve(curve: &NurbsCurve, t0: f64, t1: f64, context: &str) -> Result<NurbsCurve, KernelRefusal> {
    let [start, end] = curve.domain().or_refuse(KernelStage::Sew, "domain")?;
    // NurbsCurve::split refuses parameters within its ABSOLUTE knot
    // tolerance (1e-9) of the domain ends; the skip epsilon must cover that
    // or a near-edge trim parameter slips past the guard and errors.
    let epsilon = (1e-9 * (end - start)).max(2e-9);
    let mut result = curve.clone();
    if t0 > start + epsilon && t0 < end - epsilon {
        result = result.split(t0).or_refuse(KernelStage::Sew, "split")?.1;
    }
    let domain = result.domain().or_refuse(KernelStage::Sew, "domain")?;
    if t1 < domain[1] - epsilon && t1 > domain[0] + epsilon {
        result = result.split(t1).or_refuse(KernelStage::Sew, "split")?.0;
    } else if env_read!("BREP_DEBUG_SUBRANGE").is_some() && t1 < domain[1] - epsilon {
        eprintln!("abnormal subrange skip in {context}");
    }
    Ok(result)
}
