//! Exact knot insertion and refinement on a tensor-product NURBS surface
//! (Golovanov §3.16 builds the deformable surface on a refined control net;
//! Piegl & Tiller A5.3 / A5.5).
//!
//! Boehm's algorithm runs on the homogeneous control points `(wx, wy, wz, w)`
//! the surface stores, so rational surfaces refine exactly: the refined surface
//! is the same point set with the same parameterization. Insertion in u runs
//! the same one-knot blend down every column, in v along every row; the blend
//! coefficients depend only on the knot vector, so every column (row) is refined
//! by one alpha table and the net stays rectangular even where a whole row of
//! poles coincides (a sphere's pole).
//!
//! Unlike [`crate::NurbsCurve::insert_knot`], nothing is clamped or capped in
//! silence: a parameter outside the domain, a non-finite one, or one whose
//! multiplicity would exceed the degree is refused by name, and the caller is
//! told how many control rows it now has.

use super::NurbsSurface;
use crate::curve::KNOT_IDENTITY_TOL;
use crate::Vec4;

/// Which parameter direction a knot is inserted in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KnotDirection {
    /// The first parameter: inserting adds control ROWS (`control_points[i]`).
    U,
    /// The second parameter: inserting adds control COLUMNS (`control_points[i][j]`).
    V,
}

/// Why a knot refinement was refused. The surface is never changed in part.
#[derive(Clone, Debug, PartialEq)]
pub enum KnotRefinementRefusal {
    /// The parameter is NaN or infinite.
    NonFinite { direction: KnotDirection },
    /// The parameter lies outside the closed domain `[start, end]`.
    OutsideDomain { direction: KnotDirection, parameter: f64, start: f64, end: f64 },
    /// The knot would occur more than `degree` times in the interior (the
    /// surface would come apart there), or more than `degree + 1` at an end.
    MultiplicityExceeded { direction: KnotDirection, parameter: f64, existing: usize, requested: usize, limit: usize },
    /// The input surface itself is invalid.
    InvalidSurface(String),
}

impl std::fmt::Display for KnotRefinementRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NonFinite { direction } => write!(f, "knot refinement: non-finite {direction:?} parameter"),
            Self::OutsideDomain { direction, parameter, start, end } => write!(
                f,
                "knot refinement: {direction:?} parameter {parameter} is outside the domain [{start}, {end}]"
            ),
            Self::MultiplicityExceeded { direction, parameter, existing, requested, limit } => write!(
                f,
                "knot refinement: {direction:?} knot {parameter} already occurs {existing} times; \
                 {requested} more would exceed the limit {limit}"
            ),
            Self::InvalidSurface(message) => write!(f, "knot refinement: invalid surface: {message}"),
        }
    }
}

impl std::error::Error for KnotRefinementRefusal {}

impl NurbsSurface {
    /// Insert the knot `parameter` `count` times in `direction`. The result is
    /// the same surface (point for point at every parameter) on a finer net:
    /// `count` more rows (u) or columns (v). `count == 0` returns a copy.
    /// A parameter within [`KNOT_IDENTITY_TOL`] of an existing knot is that knot.
    pub fn insert_knot(
        &self,
        direction: KnotDirection,
        parameter: f64,
        count: usize,
    ) -> Result<NurbsSurface, KnotRefinementRefusal> {
        // No knot can take more than degree + 1 copies: refuse an oversized
        // count before allocating it.
        let degree = match direction {
            KnotDirection::U => self.degree_u,
            KnotDirection::V => self.degree_v,
        };
        if count > degree + 1 {
            return Err(KnotRefinementRefusal::MultiplicityExceeded {
                direction,
                parameter,
                existing: 0,
                requested: count,
                limit: degree + 1,
            });
        }
        self.refine_knots(direction, &vec![parameter; count])
    }

    /// Insert every parameter of `parameters` (any order, repeats allowed) in
    /// `direction`. All are checked before anything is inserted: a refusal
    /// leaves no partial result.
    pub fn refine_knots(
        &self,
        direction: KnotDirection,
        parameters: &[f64],
    ) -> Result<NurbsSurface, KnotRefinementRefusal> {
        self.domain_u().map_err(KnotRefinementRefusal::InvalidSurface)?;
        let (degree, knots) = match direction {
            KnotDirection::U => (self.degree_u, &self.knots_u),
            KnotDirection::V => (self.degree_v, &self.knots_v),
        };
        let start = knots[degree];
        let end = knots[knots.len() - 1 - degree];
        // Snap to existing knots, then check every multiplicity before inserting.
        let mut snapped = Vec::with_capacity(parameters.len());
        for &parameter in parameters {
            if !parameter.is_finite() {
                return Err(KnotRefinementRefusal::NonFinite { direction });
            }
            if parameter < start - KNOT_IDENTITY_TOL || parameter > end + KNOT_IDENTITY_TOL {
                return Err(KnotRefinementRefusal::OutsideDomain { direction, parameter, start, end });
            }
            // Snap to the NEAREST knot inside the domain (or an earlier
            // parameter of this call) within the identity tolerance; a knot
            // outside [start, end] (an unclamped vector's leading or trailing
            // knots) is never a candidate, so the knot vector stays sorted.
            let clamped = parameter.clamp(start, end);
            let value = knots[degree..knots.len() - degree]
                .iter()
                .chain(snapped.iter())
                .copied()
                .filter(|knot| (knot - clamped).abs() <= KNOT_IDENTITY_TOL)
                .min_by(|a, b| (a - clamped).abs().total_cmp(&(b - clamped).abs()))
                .unwrap_or(clamped);
            snapped.push(value);
        }
        snapped.sort_by(f64::total_cmp);
        let mut index = 0;
        while index < snapped.len() {
            let value = snapped[index];
            let requested = snapped[index..].iter().take_while(|&&other| other == value).count();
            let existing = knots.iter().filter(|&&knot| knot == value).count();
            let at_end = value == start || value == end;
            let limit = if at_end { degree + 1 } else { degree };
            if existing + requested > limit {
                return Err(KnotRefinementRefusal::MultiplicityExceeded {
                    direction,
                    parameter: value,
                    existing,
                    requested,
                    limit,
                });
            }
            index += requested;
        }
        let mut knots = knots.clone();
        let mut net = self.control_points.clone();
        for &value in &snapped {
            let (alphas, span) = boehm_table(&knots, degree, value);
            net = match direction {
                KnotDirection::U => apply_u(&net, &alphas, span, degree),
                KnotDirection::V => net.iter().map(|row| apply_line(row, &alphas, span, degree)).collect(),
            };
            knots.insert(span + 1, value);
        }
        let (knots_u, knots_v) = match direction {
            KnotDirection::U => (knots, self.knots_v.clone()),
            KnotDirection::V => (self.knots_u.clone(), knots),
        };
        NurbsSurface::new(self.degree_u, self.degree_v, knots_u, knots_v, net)
            .map_err(KnotRefinementRefusal::InvalidSurface)
    }
}

/// The span `k` (`knots[k] <= value < knots[k + 1]`, the last non-empty span
/// at the domain end) and the `p` Boehm blend coefficients for inserting
/// `value` once: new point `k - p + i` (`i = 1..=p`) is
/// `alpha_i P[k-p+i] + (1 - alpha_i) P[k-p+i-1]`.
fn boehm_table(knots: &[f64], degree: usize, value: f64) -> (Vec<f64>, usize) {
    let count = knots.len() - degree - 1;
    let mut span = degree;
    for k in degree..count {
        if knots[k] <= value {
            span = k;
        }
    }
    let alphas = (1..=degree)
        .map(|i| {
            let index = span - degree + i;
            let denominator = knots[index + degree] - knots[index];
            if denominator <= 0.0 { 0.0 } else { ((value - knots[index]) / denominator).clamp(0.0, 1.0) }
        })
        .collect();
    (alphas, span)
}

fn blend(previous: Vec4, current: Vec4, alpha: f64) -> Vec4 {
    current.scale(alpha).add(previous.scale(1.0 - alpha))
}

fn apply_line(points: &[Vec4], alphas: &[f64], span: usize, degree: usize) -> Vec<Vec4> {
    let mut out = Vec::with_capacity(points.len() + 1);
    out.extend_from_slice(&points[..=span - degree]);
    for (i, alpha) in alphas.iter().enumerate() {
        let index = span - degree + i + 1;
        out.push(blend(points[index - 1], points[index], *alpha));
    }
    out.extend_from_slice(&points[span..]);
    out
}

fn apply_u(net: &[Vec<Vec4>], alphas: &[f64], span: usize, degree: usize) -> Vec<Vec<Vec4>> {
    let mut out = Vec::with_capacity(net.len() + 1);
    out.extend(net[..=span - degree].iter().cloned());
    for (i, alpha) in alphas.iter().enumerate() {
        let index = span - degree + i + 1;
        out.push(
            net[index - 1]
                .iter()
                .zip(&net[index])
                .map(|(previous, current)| blend(*previous, *current, *alpha))
                .collect(),
        );
    }
    out.extend(net[span..].iter().cloned());
    out
}

impl NurbsSurface {
    /// Raise the degree in `direction` to `target` exactly (Bezier
    /// decomposition, binomial elevation per segment, surplus junction knots
    /// removed: [`crate::coons::compat::elevate_degree`] on every column (u)
    /// or row (v)). The same knot operations run on every line, so the lines
    /// must come back on one knot vector; a line that does not is refused
    /// rather than resampled. `target <= degree` returns a copy.
    pub fn elevate_degree(&self, direction: KnotDirection, target: usize) -> Result<NurbsSurface, String> {
        self.domain_u()?;
        let (degree, knots) = match direction {
            KnotDirection::U => (self.degree_u, &self.knots_u),
            KnotDirection::V => (self.degree_v, &self.knots_v),
        };
        if target <= degree {
            return Ok(self.clone());
        }
        let lines: Vec<Vec<Vec4>> = match direction {
            KnotDirection::U => (0..self.control_points[0].len())
                .map(|j| self.control_points.iter().map(|row| row[j]).collect())
                .collect(),
            KnotDirection::V => self.control_points.clone(),
        };
        let mut elevated_knots: Option<Vec<f64>> = None;
        let mut elevated_lines = Vec::with_capacity(lines.len());
        for line in lines {
            let curve = crate::NurbsCurve::new(degree, knots.clone(), line)?;
            let elevated = crate::coons::compat::elevate_degree(&curve, target)?;
            match &elevated_knots {
                None => elevated_knots = Some(elevated.knots.clone()),
                Some(first) if *first == elevated.knots => {}
                Some(_) => {
                    return Err("elevate_degree: lines of the net came back on different knot vectors".into())
                }
            }
            elevated_lines.push(elevated.control_points);
        }
        let elevated_knots = elevated_knots.ok_or("elevate_degree: empty net")?;
        match direction {
            KnotDirection::U => {
                let rows = elevated_lines[0].len();
                let net = (0..rows).map(|i| elevated_lines.iter().map(|line| line[i]).collect()).collect();
                NurbsSurface::new(target, self.degree_v, elevated_knots, self.knots_v.clone(), net)
            }
            KnotDirection::V => {
                NurbsSurface::new(self.degree_u, target, self.knots_u.clone(), elevated_knots, elevated_lines)
            }
        }
    }
}
