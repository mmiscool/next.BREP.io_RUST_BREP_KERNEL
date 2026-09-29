use crate::{KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse};

pub(super) fn cross_section_basis(chamfer: bool) -> (usize, Vec<f64>) {
    if chamfer {
        (1, vec![0.0, 0.0, 1.0, 1.0])
    } else {
        (2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0])
    }
}

/// Assemble compatible fitted rows into a linear chamfer or rational quadratic
/// fillet section. Closed surfaces snap their last control row to the first;
/// callers keep pcurves unwrapped independently.
pub(super) fn surface_from_rows(
    degree: usize,
    first: &NurbsCurve,
    second: &NurbsCurve,
    middle: Option<&NurbsCurve>,
    closed: bool,
) -> Result<NurbsSurface, KernelRefusal> {
    let rows = first.control_points.len();
    let mut control = Vec::with_capacity(rows);
    for index in 0..rows {
        let mut column = vec![first.control_points[index]];
        if let Some(middle) = middle {
            column.push(middle.control_points[index]);
        }
        column.push(second.control_points[index]);
        control.push(column);
    }
    if closed {
        let last = rows - 1;
        for column in 0..control[0].len() {
            control[last][column] = control[0][column];
        }
    }
    let (degree_v, knots_v) = cross_section_basis(middle.is_none());
    NurbsSurface::new(degree, degree_v, first.knots.clone(), knots_v, control)
        .or_refuse(KernelStage::Refine, "surface_new")
}

