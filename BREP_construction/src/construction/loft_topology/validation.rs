use super::*;

/// Validate nonempty section sets before interpolation. Closed lofts retain
/// their generic compatibility error; other callers distinguish knots/weights.
pub(super) fn validate_sections(
    sections: &[Vec<NurbsCurve>],
    tolerance: f64,
    operation: &str,
    detailed_errors: bool,
) -> Result<usize, KernelRefusal> {
    let curve_count = validate_section_loops(sections, tolerance, operation)?;
    validate_representations(sections, operation, detailed_errors)?;
    Ok(curve_count)
}

/// Loop topology is independent of the NURBS representation of each edge.
pub(super) fn validate_section_loops(
    sections: &[Vec<NurbsCurve>],
    tolerance: f64,
    operation: &str,
) -> Result<usize, KernelRefusal> {
    let curve_count = sections[0].len();
    if sections.iter().any(|section| section.len() != curve_count) {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "curve_count",
            format!("{operation}: sections must have the same curve count"),
        ));
    }
    for section in sections {
        closed_points(section, tolerance)?;
    }
    Ok(curve_count)
}

fn validate_representations(
    sections: &[Vec<NurbsCurve>],
    operation: &str,
    detailed_errors: bool,
) -> Result<(), KernelRefusal> {
    for curve_index in 0..sections[0].len() {
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
                    format!("{operation}: section {section_index} curve {curve_index} {reason}"),
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
                    format!("{operation}: section {section_index} curve {curve_index} {reason}"),
                ));
            }
        }
    }

    Ok(())
}

/// Whether two curves can be interpolated across: same degree, control count, knots and weights.
fn representation_matches(curve: &NurbsCurve, reference: &NurbsCurve) -> bool {
    curve.degree == reference.degree
        && curve.control_points.len() == reference.control_points.len()
        && curve.knots.len() == reference.knots.len()
        && curve.knots.iter().zip(&reference.knots).all(|(a, b)| (a - b).abs() <= 1e-9)
        && curve.control_points.iter().zip(&reference.control_points).all(|(a, b)| (a.w - b.w).abs() <= 1e-9)
}

/// Closed sections whose loops start at different places round the same outline (a rounded rectangle
/// starting at a line in one section and at an arc in the next). Only when EVERY later section is an exact
/// translated copy of section 0 at some start of its loop (equal sections: every curve's start the same offset
/// away) are the sections turned to those starts, so equal sections loft through without a twist. Any other
/// loft keeps its sections as given: a twisted sweep's stations are copies only where the twist reaches a
/// symmetry of the section (a square a quarter turn on), and turning those alone folds the loft.
pub(super) fn align_section_starts(sections: &mut [Vec<NurbsCurve>]) {
    let Some(reference) = sections.first().cloned() else { return };
    let count = reference.len();
    if count < 2 || sections.iter().any(|s| s.len() != count) {
        return;
    }
    let start = |curve: &NurbsCurve| curve.control_points.first().map(|p| [p.x / p.w, p.y / p.w, p.z / p.w]);
    let scale = reference
        .iter()
        .filter_map(start)
        .map(|p| p[0].abs().max(p[1].abs()).max(p[2].abs()))
        .fold(1.0_f64, f64::max);
    let copy = (1e-9 * scale).powi(2);
    // The start at which a section is a translated copy of section 0 (curve for curve, offsets all equal).
    let copy_start = |section: &[NurbsCurve]| -> Option<usize> {
        (0..count).find(|&shift| {
            if !(0..count).all(|i| representation_matches(&section[(i + shift) % count], &reference[i])) {
                return false;
            }
            let offsets: Option<Vec<[f64; 3]>> = (0..count)
                .map(|i| {
                    let (a, b) = (start(&section[(i + shift) % count])?, start(&reference[i])?);
                    Some([a[0] - b[0], a[1] - b[1], a[2] - b[2]])
                })
                .collect();
            let Some(offsets) = offsets else { return false };
            let mean = [0, 1, 2].map(|axis| offsets.iter().map(|o| o[axis]).sum::<f64>() / count as f64);
            offsets.iter().map(|o| (0..3).map(|axis| (o[axis] - mean[axis]).powi(2)).sum::<f64>()).sum::<f64>() <= copy
        })
    };
    let starts: Option<Vec<usize>> = sections.iter().skip(1).map(|section| copy_start(section)).collect();
    let Some(starts) = starts else { return };
    for (section, shift) in sections.iter_mut().skip(1).zip(starts) {
        section.rotate_left(shift);
    }
}

