use super::*;

type Basis = [[f64; MAX_STACK_ORDER]; 2];

struct Endpoint {
    parameter: u64,
    span: usize,
    basis: Basis,
}

impl Endpoint {
    fn new(knots: &[f64], degree: usize, value: f64) -> Self {
        let value = knot_clamp(knots, degree, value);
        let span = knot_find_span(knots, degree, value);
        let mut basis = [[0.0; MAX_STACK_ORDER]; 2];
        basis_derivatives_into(knots, degree, span, value, 1, &mut basis);
        Self { parameter: value.to_bits(), span, basis }
    }
}

/// An immutable face's extension chart and exact endpoint basis rows.
/// Endpoint reuse is keyed by the bits after the ordinary knot clamp;
/// nearby parameters are still evaluated independently.
pub(crate) struct PreparedTrimPartials<'a> {
    surface: &'a NurbsSurface,
    domains: [[f64; 2]; 2],
    closed: [bool; 2],
    endpoints: [[Endpoint; 2]; 2],
}

impl<'a> PreparedTrimPartials<'a> {
    pub(crate) fn new(surface: &'a NurbsSurface) -> Result<Option<Self>, String> {
        if !(1..=3).contains(&surface.degree_u) || !(1..=3).contains(&surface.degree_v) {
            return Ok(None);
        }
        let domains = [surface.domain_u()?, surface.domain_v()?];
        let (u, v) = surface.closed_directions()?;
        let endpoints = [
            domains[0].map(|t| Endpoint::new(&surface.knots_u, surface.degree_u, t)),
            domains[1].map(|t| Endpoint::new(&surface.knots_v, surface.degree_v, t)),
        ];
        Ok(Some(Self { surface, domains, closed: [u, v], endpoints }))
    }

    pub(crate) fn deriv1(&self, u: f64, v: f64) -> Result<(Vec3, Vec3, Vec3), String> {
        let coordinate = |value: f64, axis: usize| {
            let [lo, hi] = self.domains[axis];
            let value = if self.closed[axis] && (value < lo || value > hi) {
                lo + (value - lo).rem_euclid(hi - lo)
            } else { value };
            let outside = if value < lo { value - lo } else if value > hi { value - hi } else { 0.0 };
            (value, outside)
        };
        let (u, du) = coordinate(u, 0);
        let (v, dv) = coordinate(v, 1);
        match (self.surface.degree_u, self.surface.degree_v) {
            (1, 1) => self.at::<1, 1>(u, v, du, dv),
            (1, 2) => self.at::<1, 2>(u, v, du, dv),
            (1, 3) => self.at::<1, 3>(u, v, du, dv),
            (2, 1) => self.at::<2, 1>(u, v, du, dv),
            (2, 2) => self.at::<2, 2>(u, v, du, dv),
            (2, 3) => self.at::<2, 3>(u, v, du, dv),
            (3, 1) => self.at::<3, 1>(u, v, du, dv),
            (3, 2) => self.at::<3, 2>(u, v, du, dv),
            (3, 3) => self.at::<3, 3>(u, v, du, dv),
            _ => unreachable!(),
        }
    }

    #[inline]
    fn basis<'b>(&'b self, axis: usize, degree: usize, value: f64, scratch: &'b mut Basis) -> (usize, &'b Basis) {
        let knots = if axis == 0 { &self.surface.knots_u } else { &self.surface.knots_v };
        let value = knot_clamp(knots, degree, value);
        for endpoint in &self.endpoints[axis] {
            if value.to_bits() == endpoint.parameter { return (endpoint.span, &endpoint.basis); }
        }
        let span = knot_find_span(knots, degree, value);
        basis_derivatives_into(knots, degree, span, value, 1, scratch);
        (span, scratch)
    }

    fn at<const U: usize, const V: usize>(&self, u: f64, v: f64, du: f64, dv: f64) -> Result<(Vec3, Vec3, Vec3), String> {
        let extended = du != 0.0 || dv != 0.0;
        // Preserve the anchor subtraction, including its rounding, rather
        // than replacing an exterior anchor by a domain endpoint.
        let anchor_u = if extended { u - du } else { u };
        let anchor_v = if extended { v - dv } else { v };
        let mut scratch_u = [[0.0; MAX_STACK_ORDER]; 2];
        let mut scratch_v = [[0.0; MAX_STACK_ORDER]; 2];
        let (span_u, basis_u) = self.basis(0, U, anchor_u, &mut scratch_u);
        let (span_v, basis_v) = self.basis(1, V, anchor_v, &mut scratch_v);
        let sum = |k: usize, l: usize| {
            let mut value = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
            for i in 0..=U {
                let row = &self.surface.control_points[span_u - U + i];
                for j in 0..=V {
                    value = value.add(row[span_v - V + j].scale(basis_u[k][i] * basis_v[l][j]));
                }
            }
            value
        };
        let (h00, h01, h10) = (sum(0, 0), sum(0, 1), sum(1, 0));
        if h00.w.abs() <= EPS { return Err("NurbsSurface: zero evaluated weight".into()); }
        let inverse = 1.0 / h00.w;
        let point = Vec3::new(h00.x, h00.y, h00.z).scale(inverse);
        let sv = Vec3::new(h01.x, h01.y, h01.z).sub(point.scale(1.0 * h01.w)).scale(inverse);
        let su = Vec3::new(h10.x, h10.y, h10.z).sub(point.scale(1.0 * h10.w))
            .sub(Vec3::default().scale(1.0)).scale(inverse);
        if !extended { return Ok((point, su, sv)); }
        let h11 = sum(1, 1);
        let mixed = Vec3::default().add(point.scale(1.0 * h11.w));
        let suv = Vec3::new(h11.x, h11.y, h11.z).sub(su.scale(1.0 * h01.w))
            .sub(sv.scale(1.0 * h10.w)).sub(mixed.scale(1.0)).scale(inverse);
        Ok((point.add(su.scale(du)).add(sv.scale(dv)).add(suv.scale(du * dv)),
            su.add(suv.scale(dv)), sv.add(suv.scale(du))))
    }
}
