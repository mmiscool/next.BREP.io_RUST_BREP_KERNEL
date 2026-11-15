//! Basis rows shared by the closed wall's fixed section probes.
use crate::curve::{basis_functions_into, knot_clamp, knot_find_span, MAX_STACK_DEGREE, MAX_STACK_ORDER};
use crate::{NurbsSurface, Vec3, Vec4};

pub(super) struct Row {
    basis: Basis,
    collapsed: Option<Vec<Vec4>>,
}

pub(super) struct Basis {
    span: usize,
    values: [f64; MAX_STACK_ORDER],
}

fn basis(knots: &[f64], degree: usize, parameter: f64) -> Basis {
    let parameter = knot_clamp(knots, degree, parameter);
    let span = knot_find_span(knots, degree, parameter);
    let mut values = [0.0; MAX_STACK_ORDER];
    basis_functions_into(knots, degree, span, parameter, &mut values);
    Basis { span, values }
}

/// Bound to one immutable, validated patch and its fixed section parameters.
/// The historical road shares basis construction and preserves the original
/// accumulation order. The compact road also shares the u-direction sums
/// across the fixed v sections; each point still divides by its weight.
pub(super) struct WallSamples<'a> {
    surface: &'a NurbsSurface,
    sections: Vec<Basis>,
}

impl<'a> WallSamples<'a> {
    pub(super) fn new(surface: &'a NurbsSurface, sections: &[f64]) -> Option<Self> {
        surface.domain_u().ok()?;
        if surface.degree_u > MAX_STACK_DEGREE || surface.degree_v > MAX_STACK_DEGREE { return None; }
        let sections = sections.iter().map(|&v| basis(&surface.knots_v, surface.degree_v, v)).collect();
        Some(Self { surface, sections })
    }

    pub(super) fn row(&self, u: f64) -> Row {
        let surface = self.surface;
        let basis = basis(&surface.knots_u, surface.degree_u, u);
        // Tensor-product factorization is part of the optional compact-row
        // representation. All section points and wall checks are retained.
        let collapsed = if switch!("BREP_COMPACT_CLOSED_ROWS", true) {
            let mut columns = vec![Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 }; surface.control_points[0].len()];
            for (i, value) in basis.values[..=surface.degree_u].iter().enumerate() {
                for (column, control) in columns.iter_mut().zip(&surface.control_points[basis.span-surface.degree_u+i]) {
                    *column = column.add(control.scale(*value));
                }
            }
            Some(columns)
        } else { None };
        Row { basis, collapsed }
    }

    pub(super) fn point(&self, row: &Row, section: usize) -> Result<Vec3, String> {
        let surface = self.surface;
        let column = &self.sections[section];
        let mut point = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
        if let Some(columns) = &row.collapsed {
            for (j, value) in column.values[..=surface.degree_v].iter().enumerate() {
                point = point.add(columns[column.span-surface.degree_v+j].scale(*value));
            }
            return point.point();
        }
        let row = &row.basis;
        for (i, value_u) in row.values[..=surface.degree_u].iter().enumerate() {
            let controls = &surface.control_points[row.span - surface.degree_u + i];
            for (j, value_v) in column.values[..=surface.degree_v].iter().enumerate() {
                point = point.add(controls[column.span - surface.degree_v + j].scale(value_u * value_v));
            }
        }
        point.point()
    }
}

