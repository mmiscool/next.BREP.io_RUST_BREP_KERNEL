use super::*;

/// Validate nonempty section sets before interpolation. Closed lofts retain
/// their generic compatibility error; other callers distinguish knots/weights.
pub(super) fn validate_sections(
    sections: &[Vec<NurbsCurve>],
    tolerance: f64,
    operation: &str,
    detailed_errors: bool,
) -> Result<usize, KernelRefusal> {
    let curve_count = sections[0].len();
    if sections.iter().any(|section| section.len() != curve_count) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "curve_count",
            format!(
                "{operation}: sections must have the same curve count"
            ),
        ));
    }
    for section in sections {
        closed_points(section, tolerance)?;
    }
    for curve_index in 0..curve_count {
        let reference = &sections[0][curve_index];
        for (section_index, section) in sections.iter().enumerate().skip(1) {
            let curve = &section[curve_index];
            if curve.degree != reference.degree
                || curve.control_points.len() != reference.control_points.len()
            {
                return Err(KernelRefusal::input(
                    KernelStage::Collect,
                    "section_degree",
                    format!(
                        "{operation}: section {section_index} curve {curve_index} incompatible with section 0"
                    ),
                ));
            }
            if curve.knots.len() != reference.knots.len()
                || curve
                    .knots
                    .iter()
                    .zip(&reference.knots)
                    .any(|(a, b)| (a - b).abs() > 1e-9)
            {
                let reason = if detailed_errors {
                    "has different knots"
                } else {
                    "incompatible with section 0"
                };
                return Err(KernelRefusal::input(
                    KernelStage::Collect,
                    "section_knots",
                    format!(
                        "{operation}: section {section_index} curve {curve_index} {reason}"
                    ),
                ));
            }
            if curve
                .control_points
                .iter()
                .zip(&reference.control_points)
                .any(|(a, b)| (a.w - b.w).abs() > 1e-9)
            {
                let reason = if detailed_errors {
                    "has different weights"
                } else {
                    "incompatible with section 0"
                };
                return Err(KernelRefusal::input(
                    KernelStage::Collect,
                    "section_weights",
                    format!(
                        "{operation}: section {section_index} curve {curve_index} {reason}"
                    ),
                ));
            }
        }
    }

    Ok(curve_count)
}

