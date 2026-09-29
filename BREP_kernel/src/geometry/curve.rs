use crate::Vec3;
use serde::{Deserialize, Serialize};

const EPS: f64 = 1e-12;

/// Knot IDENTITY tolerance (parameter-space): two knot parameters within this
/// absolute band are treated as the SAME knot for snapping — insert_knot,
/// `NurbsCurve::split`, monotonicity and clamp checks.  This is the single
/// source for the "absolute 1e-9 knot tolerance" that `split` enforces and
/// that imprint's trim guards mirror.  Distinct in PURPOSE (not just value)
/// from [`KNOT_DEDUP_EPS`]; the two are deliberately NOT unified.
pub const KNOT_IDENTITY_TOL: f64 = 1e-9;

/// Numerical knot DEDUP epsilon (parameter-space): the tighter floor used when
/// *counting distinct* knots (SSI/CSI `interior_knot_count`) or comparing whole
/// knot vectors for exact reconstruction (analytic surface recognition).  This
/// answers "are these two knot floats the same value?", NOT "are these the same
/// knot for snapping?" — so it stays at 1e-12 and must NOT be unified with the
/// looser [`KNOT_IDENTITY_TOL`].
///
/// Comparand: the absolute difference of two knot values of ONE knot vector,
/// and a knot's distance from either end of the active domain
/// (`knots[degree]`, `knots[len − 1 − degree]`), in that curve's parameter
/// units. Two knots within it count as one, and a knot within it of an end is
/// not interior. The containment lanes' per-pcurve sample count
/// ([`crate::trim_sample_count`]) counts interior knots this way.
pub const KNOT_DEDUP_EPS: f64 = 1e-12;

/// Count distinct knots strictly inside a valid knot vector's active domain.
/// Multiplicities and values within [`KNOT_DEDUP_EPS`] of the domain ends are
/// ignored. Used to choose sampling density for intersections and containment.
pub(crate) fn interior_knot_count(knots: &[f64], degree: usize) -> usize {
    distinct_interior_knots(knots, degree).count()
}

fn distinct_interior_knots(knots: &[f64], degree: usize) -> impl Iterator<Item = f64> + '_ {
    let start = knots[degree];
    let end = knots[knots.len() - 1 - degree];
    let mut previous = None;
    knots.iter().copied().filter(move |&knot| {
        if knot <= start + KNOT_DEDUP_EPS || knot >= end - KNOT_DEDUP_EPS {
            return false;
        }
        if previous.is_none_or(|value: f64| (value - knot).abs() > KNOT_DEDUP_EPS) {
            previous = Some(knot);
            true
        } else {
            false
        }
    })
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct Vec4 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// Construct a line in the parameter plane (z = 0).
pub(crate) fn parameter_line(u0: f64, v0: f64, u1: f64, v1: f64) -> Result<NurbsCurve, String> {
    make_line(Vec3::new(u0, v0, 0.0), Vec3::new(u1, v1, 0.0))
}

/// Project a curve onto plane axes while preserving its knots and rational weights.
pub(crate) fn curve_to_plane_parameters(
    curve: &NurbsCurve,
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
) -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        curve.degree,
        curve.knots.clone(),
        curve
            .control_points
            .iter()
            .map(|point| {
                let euclidean = Vec3::new(point.x / point.w, point.y / point.w, point.z / point.w);
                let delta = euclidean.sub(origin);
                Vec4 {
                    x: delta.dot(x_axis) * point.w,
                    y: delta.dot(y_axis) * point.w,
                    z: 0.0,
                    w: point.w,
                }
            })
            .collect(),
    )
}

impl Vec4 {
    pub fn from_point(point: Vec3, weight: f64) -> Self {
        Self {
            x: point.x * weight,
            y: point.y * weight,
            z: point.z * weight,
            w: weight,
        }
    }

    pub(crate) fn add(self, rhs: Self) -> Self {
        Self {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
            w: self.w + rhs.w,
        }
    }

    pub(crate) fn scale(self, factor: f64) -> Self {
        Self {
            x: self.x * factor,
            y: self.y * factor,
            z: self.z * factor,
            w: self.w * factor,
        }
    }

    pub(crate) fn point(self) -> Result<Vec3, String> {
        if self.w.abs() <= EPS {
            return Err("cannot project a homogeneous point with zero weight".into());
        }
        Ok(Vec3::new(self.x / self.w, self.y / self.w, self.z / self.w))
    }
}

/// Largest degree served by the allocation-free stack-array basis engine.
/// Higher degrees (rare: only externally imported geometry) fall back to the
/// heap-allocating `KnotVector` path.
pub(crate) const MAX_STACK_DEGREE: usize = 7;
pub(crate) const MAX_STACK_ORDER: usize = MAX_STACK_DEGREE + 1;

/// `BREP_DEBUG_KNOTS=1` diagnostic for a rejected knot vector: dumps the whole
/// vector (plus a backtrace) to stderr and appends it to the returned message,
/// so a knot failure buried behind a `Result` that a caller only prints —
/// STEP import's `first_error`, say — still names the offending construction
/// site. Off by default; the message stays byte-identical without the var.
fn knot_reject(reason: &str, knots: &[f64], degree: usize) -> String {
    if std::env::var("BREP_DEBUG_KNOTS").is_err() {
        return reason.to_string();
    }
    let mut worst_descent = f64::NEG_INFINITY;
    let mut worst_index = 0usize;
    for (index, pair) in knots.windows(2).enumerate() {
        let descent = pair[0] - pair[1];
        if descent > worst_descent {
            worst_descent = descent;
            worst_index = index;
        }
    }
    let detail = format!(
        "{reason} [degree={degree} count={} worst_descent={worst_descent:.6e} at index \
         {worst_index} knots={knots:?}]",
        knots.len()
    );
    eprintln!(
        "KNOT-REJECT {detail}\n{}",
        std::backtrace::Backtrace::force_capture()
    );
    detail
}

/// Validation shared by `KnotVector::new` and the lazily-validated
/// curve/surface types. Must stay the single source of truth so cached
/// validation and eager validation reject exactly the same inputs.
pub(crate) fn validate_knots(knots: &[f64], degree: usize) -> Result<(), String> {
    if degree < 1 {
        return Err("KnotVector: degree must be >= 1".into());
    }
    if knots.len() < 2 * (degree + 1) {
        return Err(format!(
            "KnotVector: need at least {} knots for degree {}, got {}",
            2 * (degree + 1),
            degree,
            knots.len()
        ));
    }
    if knots.iter().any(|value| !value.is_finite()) {
        return Err("KnotVector: knots must be finite".into());
    }
    if knots
        .windows(2)
        .any(|pair| pair[1] < pair[0] - KNOT_IDENTITY_TOL)
    {
        return Err(knot_reject(
            "KnotVector: knots must be non-decreasing",
            knots,
            degree,
        ));
    }
    let first = knots[0];
    let last = knots[knots.len() - 1];
    for index in 0..=degree {
        if (knots[index] - first).abs() > KNOT_IDENTITY_TOL {
            return Err(knot_reject(
                "KnotVector: expected clamped start",
                knots,
                degree,
            ));
        }
        if (knots[knots.len() - 1 - index] - last).abs() > KNOT_IDENTITY_TOL {
            return Err(knot_reject("KnotVector: expected clamped end", knots, degree));
        }
    }
    if last - first <= KNOT_IDENTITY_TOL {
        return Err(knot_reject(
            "KnotVector: degenerate parameter range",
            knots,
            degree,
        ));
    }
    Ok(())
}

pub(crate) fn knot_domain(knots: &[f64], degree: usize) -> [f64; 2] {
    [knots[degree], knots[knots.len() - 1 - degree]]
}

pub(crate) fn knot_clamp(knots: &[f64], degree: usize, parameter: f64) -> f64 {
    let [start, end] = knot_domain(knots, degree);
    parameter.clamp(start, end)
}

pub(crate) fn knot_find_span(knots: &[f64], degree: usize, parameter: f64) -> usize {
    let parameter = knot_clamp(knots, degree, parameter);
    let n = knots.len() - degree - 2;
    if parameter >= knots[n + 1] {
        // At/after the domain end.  A properly clamped vector has the end knot
        // at multiplicity exactly `degree + 1`, so span `n` is the last
        // non-empty interval.  An over-clamped end (a vendor exporter can emit
        // multiplicity > degree + 1, e.g. two coincident knot VALUES) leaves
        // span `n` pointing at a zero-width interval whose basis functions are
        // all zero.  Walk back to the last interval that actually has width.
        let mut span = n;
        while span > degree && knots[span] >= knots[span + 1] {
            span -= 1;
        }
        return span;
    }
    if parameter <= knots[degree] {
        // At/before the domain start — the mirror of the case above.  For a
        // properly clamped start (multiplicity degree + 1) this returns
        // `degree`; for an over-clamped start it advances past the extra
        // coincident knots to the first non-empty interval so evaluation never
        // lands on a zero-width span (which would yield an all-zero,
        // zero-weight homogeneous point).
        let mut span = degree;
        while span < n && knots[span + 1] <= parameter {
            span += 1;
        }
        return span;
    }
    let mut low = degree;
    let mut high = n + 1;
    let mut middle = (low + high) / 2;
    while parameter < knots[middle] || parameter >= knots[middle + 1] {
        if parameter < knots[middle] {
            high = middle;
        } else {
            low = middle;
        }
        middle = (low + high) / 2;
    }
    middle
}

/// Cox–de Boor basis functions into a caller-provided stack array.
/// Requires `degree <= MAX_STACK_DEGREE`; entries `0..=degree` are written.
pub(crate) fn basis_functions_into(
    knots: &[f64],
    degree: usize,
    span: usize,
    parameter: f64,
    basis: &mut [f64; MAX_STACK_ORDER],
) {
    debug_assert!(degree <= MAX_STACK_DEGREE);
    let mut left = [0.0f64; MAX_STACK_ORDER];
    let mut right = [0.0f64; MAX_STACK_ORDER];
    basis[0] = 1.0;
    for j in 1..=degree {
        left[j] = parameter - knots[span + 1 - j];
        right[j] = knots[span + j] - parameter;
        let mut saved = 0.0;
        for r in 0..j {
            let denominator = right[r + 1] + left[j - r];
            let temporary = if denominator.abs() <= EPS {
                0.0
            } else {
                basis[r] / denominator
            };
            basis[r] = saved + right[r + 1] * temporary;
            saved = left[j - r] * temporary;
        }
        basis[j] = saved;
    }
}

/// Basis derivatives (The NURBS Book A2.3) into caller-provided stack rows.
/// Writes rows `0..=min(derivative_count, degree)` of `out`; rows above the
/// degree keep whatever the caller initialized them to (callers zero-fill,
/// matching the heap implementation's zero rows). Requires
/// `degree <= MAX_STACK_DEGREE` and `out.len() > derivative_count`.
pub(crate) fn basis_derivatives_into(
    knots: &[f64],
    degree: usize,
    span: usize,
    parameter: f64,
    derivative_count: usize,
    out: &mut [[f64; MAX_STACK_ORDER]],
) {
    debug_assert!(degree <= MAX_STACK_DEGREE);
    debug_assert!(out.len() > derivative_count);
    let p = degree;
    let n = derivative_count.min(p);
    let mut ndu = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
    let mut left = [0.0f64; MAX_STACK_ORDER];
    let mut right = [0.0f64; MAX_STACK_ORDER];
    ndu[0][0] = 1.0;

    for j in 1..=p {
        left[j] = parameter - knots[span + 1 - j];
        right[j] = knots[span + j] - parameter;
        let mut saved = 0.0;
        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temporary = if ndu[j][r].abs() <= EPS {
                0.0
            } else {
                ndu[r][j - 1] / ndu[j][r]
            };
            ndu[r][j] = saved + right[r + 1] * temporary;
            saved = left[j - r] * temporary;
        }
        ndu[j][j] = saved;
    }

    for j in 0..=p {
        out[0][j] = ndu[j][p];
    }
    let mut a = [[0.0f64; MAX_STACK_ORDER]; 2];
    for r in 0..=p {
        let mut s1 = 0;
        let mut s2 = 1;
        a[0][0] = 1.0;
        for k in 1..=n {
            a[s2] = [0.0; MAX_STACK_ORDER];
            let mut value = 0.0;
            let rk = r as isize - k as isize;
            let pk = p - k;
            if r >= k {
                let denominator = ndu[pk + 1][rk as usize];
                a[s2][0] = a[s1][0] / denominator;
                value = a[s2][0] * ndu[rk as usize][pk];
            }
            let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
            let j2 = if r <= pk + 1 { k - 1 } else { p - r };
            if j1 <= j2 {
                for j in j1..=j2 {
                    let index = (rk + j as isize) as usize;
                    a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][index];
                    value += a[s2][j] * ndu[index][pk];
                }
            }
            if r <= pk {
                a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                value += a[s2][k] * ndu[r][pk];
            }
            out[k][r] = value;
            std::mem::swap(&mut s1, &mut s2);
        }
    }
    let mut factor = p as f64;
    for k in 1..=n {
        for value in out[k][..=p].iter_mut() {
            *value *= factor;
        }
        factor *= (p - k) as f64;
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct KnotVector {
    pub knots: Vec<f64>,
    pub degree: usize,
}

/// Distinct interior knots, excluding endpoint neighborhoods within `1e-12`.
pub(crate) fn interior_knots(knots: &[f64], degree: usize) -> Vec<f64> {
    distinct_interior_knots(knots, degree).collect()
}

impl KnotVector {
    pub fn new(knots: Vec<f64>, degree: usize) -> Result<Self, String> {
        validate_knots(&knots, degree)?;
        Ok(Self { knots, degree })
    }

    pub fn control_point_count(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    pub fn domain(&self) -> [f64; 2] {
        [
            self.knots[self.degree],
            self.knots[self.knots.len() - 1 - self.degree],
        ]
    }

    pub fn clamp_param(&self, parameter: f64) -> f64 {
        knot_clamp(&self.knots, self.degree, parameter)
    }

    pub fn find_span(&self, parameter: f64) -> usize {
        knot_find_span(&self.knots, self.degree, parameter)
    }

    pub fn basis_functions(&self, span: usize, parameter: f64) -> Vec<f64> {
        if self.degree <= MAX_STACK_DEGREE {
            let mut basis = [0.0f64; MAX_STACK_ORDER];
            basis_functions_into(&self.knots, self.degree, span, parameter, &mut basis);
            return basis[..=self.degree].to_vec();
        }
        let mut basis = vec![0.0; self.degree + 1];
        let mut left = vec![0.0; self.degree + 1];
        let mut right = vec![0.0; self.degree + 1];
        basis[0] = 1.0;
        for j in 1..=self.degree {
            left[j] = parameter - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - parameter;
            let mut saved = 0.0;
            for r in 0..j {
                let denominator = right[r + 1] + left[j - r];
                let temporary = if denominator.abs() <= EPS {
                    0.0
                } else {
                    basis[r] / denominator
                };
                basis[r] = saved + right[r + 1] * temporary;
                saved = left[j - r] * temporary;
            }
            basis[j] = saved;
        }
        basis
    }

    pub fn basis_derivatives(
        &self,
        span: usize,
        parameter: f64,
        derivative_count: usize,
    ) -> Vec<Vec<f64>> {
        if self.degree <= MAX_STACK_DEGREE && derivative_count <= MAX_STACK_DEGREE {
            let mut rows = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            basis_derivatives_into(
                &self.knots,
                self.degree,
                span,
                parameter,
                derivative_count,
                &mut rows[..=derivative_count],
            );
            return rows[..=derivative_count]
                .iter()
                .map(|row| row[..=self.degree].to_vec())
                .collect();
        }
        let p = self.degree;
        let n = derivative_count.min(p);
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        ndu[0][0] = 1.0;

        for j in 1..=p {
            left[j] = parameter - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - parameter;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temporary = if ndu[j][r].abs() <= EPS {
                    0.0
                } else {
                    ndu[r][j - 1] / ndu[j][r]
                };
                ndu[r][j] = saved + right[r + 1] * temporary;
                saved = left[j - r] * temporary;
            }
            ndu[j][j] = saved;
        }

        let mut derivatives = vec![vec![0.0; p + 1]; derivative_count + 1];
        for j in 0..=p {
            derivatives[0][j] = ndu[j][p];
        }
        let mut a = vec![vec![0.0; p + 1]; 2];
        for r in 0..=p {
            let mut s1 = 0;
            let mut s2 = 1;
            a[0][0] = 1.0;
            for k in 1..=n {
                a[s2].fill(0.0);
                let mut value = 0.0;
                let rk = r as isize - k as isize;
                let pk = p - k;
                if r >= k {
                    let denominator = ndu[pk + 1][rk as usize];
                    a[s2][0] = a[s1][0] / denominator;
                    value = a[s2][0] * ndu[rk as usize][pk];
                }
                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if r <= pk + 1 { k - 1 } else { p - r };
                if j1 <= j2 {
                    for j in j1..=j2 {
                        let index = (rk + j as isize) as usize;
                        a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk + 1][index];
                        value += a[s2][j] * ndu[index][pk];
                    }
                }
                if r <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[pk + 1][r];
                    value += a[s2][k] * ndu[r][pk];
                }
                derivatives[k][r] = value;
                std::mem::swap(&mut s1, &mut s2);
            }
        }
        let mut factor = p as f64;
        for (k, row) in derivatives.iter_mut().enumerate().take(n + 1).skip(1) {
            for value in row {
                *value *= factor;
            }
            factor *= (p - k) as f64;
        }
        derivatives
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct NurbsCurve {
    pub degree: usize,
    pub knots: Vec<f64>,
    pub control_points: Vec<Vec4>,
    /// One-time validation cache. Deserialized curves (which bypass `new`)
    /// run the full `new`-equivalent checks on first geometric use instead of
    /// re-validating the knot vector on every evaluation.
    #[serde(skip, default)]
    validated: std::cell::Cell<bool>,
}

impl NurbsCurve {
    /// Whether the one-time validation has run. For the boolean's
    /// operand-state instrument (`BREP_DEBUG_OPERAND_STATE`).
    pub(crate) fn is_validated(&self) -> bool {
        self.validated.get()
    }

    /// Sets the validation cache: for the operand-state instrument only.
    pub(crate) fn set_validated(&mut self, validated: bool) {
        self.validated = std::cell::Cell::new(validated);
    }

    pub fn new(degree: usize, knots: Vec<f64>, control_points: Vec<Vec4>) -> Result<Self, String> {
        let curve = Self {
            degree,
            knots,
            control_points,
            validated: std::cell::Cell::new(false),
        };
        curve.ensure_valid()?;
        Ok(curve)
    }

    /// The full construction-time checks, run at most once per instance.
    fn ensure_valid(&self) -> Result<(), String> {
        if self.validated.get() {
            return Ok(());
        }
        validate_knots(&self.knots, self.degree)?;
        let expected = self.knots.len() - self.degree - 1;
        if self.control_points.len() != expected {
            return Err(format!(
                "NurbsCurve: knot vector implies {} control points, got {}",
                expected,
                self.control_points.len()
            ));
        }
        if self.control_points.iter().any(|point| {
            point.w <= EPS
                || ![point.x, point.y, point.z, point.w]
                    .iter()
                    .all(|value| value.is_finite())
        }) {
            return Err("NurbsCurve: control points must be finite with positive weights".into());
        }
        self.validated.set(true);
        Ok(())
    }

    fn knot_vector(&self) -> Result<KnotVector, String> {
        KnotVector::new(self.knots.clone(), self.degree)
    }

    pub fn domain(&self) -> Result<[f64; 2], String> {
        self.ensure_valid()?;
        Ok(knot_domain(&self.knots, self.degree))
    }

    pub fn evaluate_homogeneous(&self, parameter: f64) -> Result<Vec4, String> {
        self.ensure_valid()?;
        let span = knot_find_span(&self.knots, self.degree, parameter);
        let parameter = knot_clamp(&self.knots, self.degree, parameter);
        let mut point = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        if self.degree <= MAX_STACK_DEGREE {
            let mut basis = [0.0f64; MAX_STACK_ORDER];
            basis_functions_into(&self.knots, self.degree, span, parameter, &mut basis);
            for (index, value) in basis[..=self.degree].iter().enumerate() {
                point = point.add(self.control_points[span - self.degree + index].scale(*value));
            }
        } else {
            let knot_vector = self.knot_vector()?;
            let basis = knot_vector.basis_functions(span, parameter);
            for (index, value) in basis.iter().enumerate() {
                point = point.add(self.control_points[span - self.degree + index].scale(*value));
            }
        }
        Ok(point)
    }

    pub fn evaluate(&self, parameter: f64) -> Result<Vec3, String> {
        self.evaluate_homogeneous(parameter)?.point()
    }

    /// The straight segment this curve traces, as `(start, end)`, or `None`
    /// when it genuinely bends.
    ///
    /// What a direct-edit heal actually needs to know about an edge is
    /// *geometric* — "is this a straight segment I may rebuild between two
    /// re-solved corners?" — but it used to be answered by the
    /// *representation* `degree == 1 && control_points.len() == 2`. That
    /// answer is wrong for a line that carries an interior knot, which is what
    /// an offset shell or a boolean leaves behind when it splits a box edge and
    /// rejoins the pieces: the same line, three control points, and every heal
    /// that asked the representation question called it "curved".
    ///
    /// Weights are strictly positive (`ensure_valid`), so every point of the
    /// curve is a convex combination of its control points and the curve is a
    /// straight segment **iff** those control points are collinear — for any
    /// degree, not just degree 1. Monotone projections onto the chord are
    /// required too, so a polyline that doubles back along its own line (it
    /// covers the segment but does not parameterise it) is not mistaken for
    /// one.
    pub(crate) fn straight_segment(&self, tolerance: f64) -> Option<(Vec3, Vec3)> {
        self.ensure_valid().ok()?;
        let points = self
            .control_points
            .iter()
            .map(|point| point.point().ok())
            .collect::<Option<Vec<Vec3>>>()?;
        let start = *points.first()?;
        let end = *points.last()?;
        let chord = end.sub(start);
        let length = chord.length();
        if length <= tolerance {
            return None;
        }
        let direction = chord.scale(1.0 / length);
        let mut reached = 0.0f64;
        for point in &points {
            let offset = point.sub(start);
            let axial = offset.dot(direction);
            if offset.sub(direction.scale(axial)).length() > tolerance {
                return None; // off the chord — the curve genuinely bends
            }
            if axial < reached - tolerance {
                return None; // collinear, but the polygon doubles back
            }
            reached = reached.max(axial);
        }
        Some((start, end))
    }

    /// Value beyond the domain (Golovanov §2.15): every curve must answer
    /// out-of-domain queries because intersection and projection
    /// algorithms probe there.  Closed curves wrap the parameter
    /// cyclically; open curves extend linearly along the end tangent.
    pub fn evaluate_extended(&self, parameter: f64) -> Result<Vec3, String> {
        Ok(self.derivatives_extended(parameter, 0)?[0])
    }

    /// Derivatives beyond the domain (§2.15).  The open-end extension is
    /// linear: the first derivative is the boundary tangent and higher
    /// derivatives vanish.
    pub fn derivatives_extended(
        &self,
        parameter: f64,
        derivative_count: usize,
    ) -> Result<Vec<Vec3>, String> {
        let [start, end] = self.domain()?;
        if parameter >= start && parameter <= end {
            return self.derivatives(parameter, derivative_count);
        }
        let period = end - start;
        if period > 0.0 {
            let closed =
                self.evaluate(start)?.sub(self.evaluate(end)?).length() <= 1e-9 * (1.0 + period);
            if closed {
                let wrapped = start + (parameter - start).rem_euclid(period);
                return self.derivatives(wrapped, derivative_count);
            }
        }
        let boundary = if parameter < start { start } else { end };
        let base = self.derivatives(boundary, derivative_count.max(1))?;
        let mut result = Vec::with_capacity(derivative_count + 1);
        result.push(base[0].add(base[1].scale(parameter - boundary)));
        if derivative_count >= 1 {
            result.push(base[1]);
        }
        for _ in 2..=derivative_count {
            result.push(Vec3::default());
        }
        Ok(result)
    }

    pub fn derivatives(
        &self,
        parameter: f64,
        derivative_count: usize,
    ) -> Result<Vec<Vec3>, String> {
        self.ensure_valid()?;
        let parameter = knot_clamp(&self.knots, self.degree, parameter);
        let span = knot_find_span(&self.knots, self.degree, parameter);
        let calculated_count = derivative_count.min(self.degree);
        let mut homogeneous: Vec<Vec4> = Vec::with_capacity(calculated_count + 1);
        if self.degree <= MAX_STACK_DEGREE {
            let mut rows = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            basis_derivatives_into(
                &self.knots,
                self.degree,
                span,
                parameter,
                calculated_count,
                &mut rows[..=calculated_count],
            );
            for row in rows.iter().take(calculated_count + 1) {
                let mut point = Vec4 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 0.0,
                };
                for (index, value) in row.iter().enumerate().take(self.degree + 1) {
                    point =
                        point.add(self.control_points[span - self.degree + index].scale(*value));
                }
                homogeneous.push(point);
            }
        } else {
            let knot_vector = self.knot_vector()?;
            let basis = knot_vector.basis_derivatives(span, parameter, calculated_count);
            for row in basis.iter().take(calculated_count + 1) {
                let mut point = Vec4 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 0.0,
                };
                for (index, value) in row.iter().enumerate().take(self.degree + 1) {
                    point =
                        point.add(self.control_points[span - self.degree + index].scale(*value));
                }
                homogeneous.push(point);
            }
        }

        let mut result: Vec<Vec3> = Vec::with_capacity(derivative_count + 1);
        for k in 0..=calculated_count {
            let mut value = Vec3::new(homogeneous[k].x, homogeneous[k].y, homogeneous[k].z);
            for i in 1..=k {
                value = value.sub(result[k - i].scale(binomial(k, i) * homogeneous[i].w));
            }
            result.push(value.scale(1.0 / homogeneous[0].w));
        }
        result.resize(derivative_count + 1, Vec3::default());
        Ok(result)
    }

    /// The curve's unit TANGENT at `parameter`, read as the one-sided limit
    /// when the first derivative vanishes there.
    ///
    /// This is what `derivatives(t, 1)?[1].normalized()?` should always have
    /// been. That shape refuses on a curve class that is not exotic at all —
    /// any Hermite segment whose end tangent is the zero vector, of which an
    /// INVOLUTE flank is the standard source (speed `r_base * t`, so a profile
    /// sampled from the base circle outwards carries an exactly zero tangent at
    /// its start, one per flank per tooth). The direction of travel is still
    /// defined there, as the limit from inside the range, and
    /// [`Self::stationary_tangent_rescue`] reads it.
    ///
    /// Pass the range the CALLER owns — an edge's `[t0, t1]`, not the curve's
    /// domain, where the two differ (an `EdgeRecord` keeps the whole curve and
    /// records the sub-range it uses), so the probe never reads geometry
    /// outside the caller's own span.
    ///
    /// The error names the parameter and the range it probed, because the only
    /// curve that reaches it is stationary EVERYWHERE the range allows, and the
    /// caller's own identifier is the half this function cannot supply.
    ///
    /// Escape hatch for tamper-verification: `BREP_CUSP_TANGENT_RESCUE=0`
    /// restores the bare `normalized()` refusal, the same hatch the imprint
    /// driver's seed loop carries.
    pub fn unit_tangent(
        &self,
        parameter: f64,
        range_start: f64,
        range_end: f64,
    ) -> Result<Vec3, String> {
        let derivative = self.derivatives(parameter, 1)?[1];
        self.tangent_or_one_sided_limit(derivative, parameter, range_start, range_end)
    }

    /// The point and unit tangent that `derivatives_extended(parameter, 1)`
    /// reads, with the tangent taken as [`Self::unit_tangent`] takes it.
    ///
    /// The blend march reads its section plane — the point and the edge's
    /// unit tangent — through the EXTENDED evaluation at every station,
    /// overshoot stations past the edge's ends included, and `unit_tangent`
    /// cannot stand in for that read: it evaluates `derivatives`, which CLAMPS
    /// a parameter outside the domain, where `derivatives_extended` WRAPS it on
    /// a closed curve. A closed rim's overshoot station would read a different
    /// tangent. This is the same one evaluation the march always made, so a
    /// non-zero derivative returns the vector `derivatives[1].normalized()`
    /// returned, to the bit, at no extra cost.
    ///
    /// Where the derivative vanishes, the one-sided limit is read at the
    /// parameter the extended evaluation actually took its derivative from —
    /// the parameter itself inside the domain, the wrapped one on a closed
    /// curve, the boundary past an open end, where the extension's direction
    /// IS the boundary's one-sided limit. The caller's range is widened to
    /// contain that parameter, because an overshoot station reads outside the
    /// caller's span on purpose; the ladder still probes from the side the
    /// range lies on.
    pub fn point_and_unit_tangent_extended(
        &self,
        parameter: f64,
        range_start: f64,
        range_end: f64,
    ) -> Result<(Vec3, Vec3), String> {
        let derivatives = self.derivatives_extended(parameter, 1)?;
        if let Ok(tangent) = derivatives[1].normalized() {
            return Ok((derivatives[0], tangent));
        }
        let read_at = self.extended_read_parameter(parameter)?;
        let low = range_start.min(range_end).min(read_at);
        let high = range_start.max(range_end).max(read_at);
        let tangent = self.tangent_or_one_sided_limit(derivatives[1], read_at, low, high)?;
        Ok((derivatives[0], tangent))
    }

    /// The in-domain parameter whose derivatives `derivatives_extended` reads
    /// for `parameter`: itself inside the domain, wrapped on a closed curve,
    /// the nearer boundary past an open end. Mirrors that function's three
    /// cases; only the zero-derivative lane of
    /// [`Self::point_and_unit_tangent_extended`] pays for it.
    fn extended_read_parameter(&self, parameter: f64) -> Result<f64, String> {
        let [start, end] = self.domain()?;
        if parameter >= start && parameter <= end {
            return Ok(parameter);
        }
        if self.extension_wraps(start, end)? {
            return Ok(start + (parameter - start).rem_euclid(end - start));
        }
        Ok(if parameter < start { start } else { end })
    }

    /// `derivatives_extended`'s closed test: a curve whose ends meet WRAPS an
    /// out-of-domain parameter instead of extending it.
    fn extension_wraps(&self, start: f64, end: f64) -> Result<bool, String> {
        let period = end - start;
        Ok(period > 0.0
            && self.evaluate(start)?.sub(self.evaluate(end)?).length() <= 1e-9 * (1.0 + period))
    }

    /// The open end `parameter` lies past, when the first derivative VANISHES
    /// there — the one case in which `derivatives_extended`'s point does not
    /// move with its parameter: past an open end it returns
    /// `P(end) + C'(end)·(t − end)`, which is `P(end)` for every `t` when
    /// `C'(end) = 0`. `None` inside the domain, on a closed curve (which wraps
    /// onto the real curve), and past an end whose derivative is regular.
    ///
    /// The zero test is [`Self::point_and_unit_tangent_extended`]'s own: the
    /// derivative has no unit vector. A caller that extends past such an end
    /// supplies the SPEED the parameterization does not; the direction is the
    /// one-sided limit that function already reads there.
    pub(crate) fn stationary_open_end(&self, parameter: f64) -> Result<Option<f64>, String> {
        let [start, end] = self.domain()?;
        if parameter >= start && parameter <= end {
            return Ok(None);
        }
        let boundary = if parameter < start { start } else { end };
        if self.derivatives(boundary, 1)?[1].normalized().is_ok()
            || self.extension_wraps(start, end)?
        {
            return Ok(None);
        }
        Ok(Some(boundary))
    }

    /// `derivative`, normalized — or, where it vanishes, the one-sided limit
    /// from inside `[range_start, range_end]` at `parameter`, or the named
    /// refusal. The one failure lane [`Self::unit_tangent`] and
    /// [`Self::point_and_unit_tangent_extended`] share.
    fn tangent_or_one_sided_limit(
        &self,
        derivative: Vec3,
        parameter: f64,
        range_start: f64,
        range_end: f64,
    ) -> Result<Vec3, String> {
        let error = match derivative.normalized() {
            Ok(tangent) => return Ok(tangent),
            Err(error) => error,
        };
        if std::env::var("BREP_CUSP_TANGENT_RESCUE").as_deref() == Ok("0") {
            return Err(error);
        }
        self.stationary_tangent_rescue(parameter, range_start, range_end)
            .ok_or_else(|| {
                format!(
                    "curve tangent: the first derivative vanishes at t = {parameter} and \
                     everywhere in [{range_start}, {range_end}] the probe can reach, so this \
                     curve has no direction of travel to report"
                )
            })
    }

    /// The unit tangent just inside `[range_start, range_end]`, for a
    /// `parameter` where the first derivative VANISHES.
    ///
    /// A curve can be stationary at a parameter where the curve itself is
    /// perfectly well behaved, and the direction of travel is then still
    /// defined as the one-sided limit. The standard case is a cubic Bezier
    /// whose first two control points coincide — the shape a Hermite segment
    /// takes when its end tangent is the zero vector — and an INVOLUTE is the
    /// standard source of one: the involute's speed is `r_base * t`, so a gear
    /// flank sampled from the base circle outwards starts with an exactly zero
    /// tangent and every tooth profile carries one per flank. The 2026-09-14
    /// herringbone report is that shape: a hit landing exactly on the cusp
    /// parameter made `derivatives(t, 1)[1].normalized()` fail and killed the
    /// whole union.
    ///
    /// The range bounds the probe — pass an edge's own `[t0, t1]` and the
    /// rescue never reads the curve outside the span the caller owns. The
    /// ladder walks OUTWARDS, and it starts at 1e-6 of the span rather than at
    /// the smallest representable step: the rescued direction is a ONE-SIDED
    /// limit, so a nearer probe truncates less, but the derivative at a
    /// stationary point is a CANCELLING sum that shrinks with the step while
    /// its rounding error does not — at 1e-9 of the span the involute cusp
    /// below reads its own limit direction only to 1e-6, worse than at 1e-6 of
    /// the span. Later rungs exist for a higher-order stationary point, whose
    /// derivative is still under the zero-length floor at the first. `None`
    /// means the curve is stationary everywhere the range allows, where no
    /// direction exists to report at all.
    pub fn stationary_tangent_rescue(
        &self,
        parameter: f64,
        range_start: f64,
        range_end: f64,
    ) -> Option<Vec3> {
        let low = range_start.min(range_end);
        let high = range_start.max(range_end);
        let span = high - low;
        if !(span > 0.0) {
            return None;
        }
        for scale in [1e-6, 1e-4, 1e-2, 1e-1] {
            let step = span * scale;
            for probe in [parameter + step, parameter - step] {
                if probe < low || probe > high {
                    continue;
                }
                if let Some(unit) = self
                    .derivatives(probe, 1)
                    .ok()
                    .and_then(|derivatives| derivatives[1].normalized().ok())
                {
                    return Some(unit);
                }
            }
        }
        None
    }

    /// Allocation-free twin of [`Self::derivatives`] for the hot
    /// `derivative_count <= 2` path.
    ///
    /// Byte-for-byte faithful copy of the arithmetic in [`Self::derivatives`]:
    /// identical basis evaluation, identical summation order, and the identical
    /// rational de-homogenization recurrence. Only the storage differs — a
    /// fixed-size `[Vec4; 3]` / `[Vec3; 3]` on the stack instead of the heap
    /// `Vec`s. Entries beyond `derivative_count.min(degree)` stay
    /// `Vec3::default()`, exactly as the heap version's trailing `resize` leaves
    /// them.
    pub(crate) fn derivatives_small(
        &self,
        parameter: f64,
        derivative_count: usize,
    ) -> Result<[Vec3; 3], String> {
        debug_assert!(derivative_count <= 2);
        self.ensure_valid()?;
        let parameter = knot_clamp(&self.knots, self.degree, parameter);
        let span = knot_find_span(&self.knots, self.degree, parameter);
        let calculated_count = derivative_count.min(self.degree);
        let zero = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let mut homogeneous = [zero; 3];
        if self.degree <= MAX_STACK_DEGREE {
            let mut rows = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            basis_derivatives_into(
                &self.knots,
                self.degree,
                span,
                parameter,
                calculated_count,
                &mut rows[..=calculated_count],
            );
            for (k, row) in rows.iter().take(calculated_count + 1).enumerate() {
                let mut point = zero;
                for (index, value) in row.iter().enumerate().take(self.degree + 1) {
                    point =
                        point.add(self.control_points[span - self.degree + index].scale(*value));
                }
                homogeneous[k] = point;
            }
        } else {
            let knot_vector = self.knot_vector()?;
            let basis = knot_vector.basis_derivatives(span, parameter, calculated_count);
            for (k, row) in basis.iter().take(calculated_count + 1).enumerate() {
                let mut point = zero;
                for (index, value) in row.iter().enumerate().take(self.degree + 1) {
                    point =
                        point.add(self.control_points[span - self.degree + index].scale(*value));
                }
                homogeneous[k] = point;
            }
        }

        let mut result = [Vec3::default(); 3];
        for k in 0..=calculated_count {
            let mut value = Vec3::new(homogeneous[k].x, homogeneous[k].y, homogeneous[k].z);
            for i in 1..=k {
                value = value.sub(result[k - i].scale(binomial(k, i) * homogeneous[i].w));
            }
            result[k] = value.scale(1.0 / homogeneous[0].w);
        }
        Ok(result)
    }

    /// Point and first derivative `(C, C')` with zero heap allocation.
    /// Bit-identical to `derivatives(t, 1)` at indices `[0]` and `[1]`.
    #[inline]
    pub(crate) fn deriv1(&self, parameter: f64) -> Result<(Vec3, Vec3), String> {
        let d = self.derivatives_small(parameter, 1)?;
        Ok((d[0], d[1]))
    }

    pub fn reversed(&self) -> Result<Self, String> {
        let start = self.knots[0];
        let end = self.knots[self.knots.len() - 1];
        let knots = self
            .knots
            .iter()
            .rev()
            .map(|knot| start + end - knot)
            .collect();
        let control_points = self.control_points.iter().rev().copied().collect();
        Self::new(self.degree, knots, control_points)
    }

    pub fn insert_knot(&self, parameter: f64, requested: usize) -> Result<Self, String> {
        let knot_vector = self.knot_vector()?;
        let degree = self.degree;
        let parameter = knot_vector.clamp_param(parameter);
        // A parameter within knot tolerance of an existing knot must BE
        // that knot: counting it as a multiplicity while find_span places
        // it in the span below (parameter infinitesimally smaller) makes
        // the Boehm index bookkeeping inconsistent and corrupts the net.
        let parameter = self
            .knots
            .iter()
            .copied()
            .find(|knot| (knot - parameter).abs() <= KNOT_IDENTITY_TOL)
            .unwrap_or(parameter);
        let multiplicity = self
            .knots
            .iter()
            .filter(|knot| (**knot - parameter).abs() <= KNOT_IDENTITY_TOL)
            .count();
        let insertion_count = requested.min(degree.saturating_sub(multiplicity));
        if insertion_count == 0 {
            return Ok(self.clone());
        }
        let span = knot_vector.find_span(parameter);
        let last_control = self.control_points.len() - 1;
        let mut knots = Vec::with_capacity(self.knots.len() + insertion_count);
        knots.extend_from_slice(&self.knots[..=span]);
        knots.extend(std::iter::repeat_n(parameter, insertion_count));
        knots.extend_from_slice(&self.knots[span + 1..]);

        let mut output = vec![
            Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            };
            last_control + 1 + insertion_count
        ];
        output[..=span - degree].copy_from_slice(&self.control_points[..=span - degree]);
        for index in span - multiplicity..=last_control {
            output[index + insertion_count] = self.control_points[index];
        }
        let mut affected = vec![
            Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            };
            degree + 1
        ];
        affected[..=degree - multiplicity]
            .copy_from_slice(&self.control_points[span - degree..=span - multiplicity]);
        let mut left = 0;
        for insertion in 1..=insertion_count {
            left = span - degree + insertion;
            for index in 0..=degree - insertion - multiplicity {
                let denominator = self.knots[index + span + 1] - self.knots[left + index];
                let alpha = (parameter - self.knots[left + index]) / denominator;
                affected[index] = affected[index + 1]
                    .scale(alpha)
                    .add(affected[index].scale(1.0 - alpha));
            }
            output[left] = affected[0];
            output[span + insertion_count - insertion - multiplicity] =
                affected[degree - insertion - multiplicity];
        }
        for index in left + 1..span - multiplicity {
            output[index] = affected[index - left];
        }
        Self::new(degree, knots, output)
    }

    pub fn split(&self, parameter: f64) -> Result<(Self, Self), String> {
        let [start, end] = self.domain()?;
        if parameter <= start + KNOT_IDENTITY_TOL || parameter >= end - KNOT_IDENTITY_TOL {
            return Err(format!(
                "NurbsCurve.split: parameter {parameter} must be strictly inside domain [{start}, {end}]"
            ));
        }
        // Snap onto a coincident knot so multiplicity and span agree (see
        // insert_knot).
        let parameter = self
            .knots
            .iter()
            .copied()
            .find(|knot| (knot - parameter).abs() <= KNOT_IDENTITY_TOL)
            .unwrap_or(parameter);
        let multiplicity = self
            .knots
            .iter()
            .filter(|knot| (**knot - parameter).abs() <= KNOT_IDENTITY_TOL)
            .count();
        let refined = self.insert_knot(parameter, self.degree.saturating_sub(multiplicity))?;
        let first = refined
            .knots
            .iter()
            .position(|knot| (*knot - parameter).abs() <= KNOT_IDENTITY_TOL)
            .ok_or_else(|| "NurbsCurve.split: inserted knot not found".to_string())?;
        let mut left_knots = refined.knots[..first + self.degree].to_vec();
        left_knots.push(parameter);
        let left_points = refined.control_points[..first].to_vec();
        let mut right_knots = vec![parameter; self.degree + 1];
        right_knots.extend_from_slice(&refined.knots[first + self.degree..]);
        let right_points = refined.control_points[first - 1..].to_vec();
        Ok((
            Self::new(self.degree, left_knots, left_points)?,
            Self::new(self.degree, right_knots, right_points)?,
        ))
    }
}

pub fn make_line(start: Vec3, end: Vec3) -> Result<NurbsCurve, String> {
    NurbsCurve::new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![Vec4::from_point(start, 1.0), Vec4::from_point(end, 1.0)],
    )
}

pub fn make_arc(
    center: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    radius: f64,
    start_angle: f64,
    end_angle: f64,
) -> Result<NurbsCurve, String> {
    if radius <= EPS {
        return Err("makeArc: radius must be positive".into());
    }
    let x_axis = x_axis.normalized()?;
    let y_axis = y_axis.normalized()?;
    if x_axis.dot(y_axis).abs() > 1e-9 {
        return Err("makeArc: xAxis and yAxis must be orthogonal".into());
    }
    let mut theta = end_angle - start_angle;
    if theta <= EPS {
        return Err("makeArc: endAngle must exceed startAngle".into());
    }
    if theta > std::f64::consts::TAU + EPS {
        return Err("makeArc: sweep exceeds full circle".into());
    }
    theta = theta.min(std::f64::consts::TAU);
    let segment_count = ((theta / std::f64::consts::FRAC_PI_2 - EPS).ceil() as usize).clamp(1, 4);
    let segment_angle = theta / segment_count as f64;
    let middle_weight = (segment_angle / 2.0).cos();
    let point_at = |angle: f64| {
        center
            .add(x_axis.scale(radius * angle.cos()))
            .add(y_axis.scale(radius * angle.sin()))
    };
    let tangent_at = |angle: f64| x_axis.scale(-angle.sin()).add(y_axis.scale(angle.cos()));

    let mut points = Vec::with_capacity(2 * segment_count + 1);
    let mut angle = start_angle;
    let mut first_point = point_at(angle);
    let mut first_tangent = tangent_at(angle);
    points.push(Vec4::from_point(first_point, 1.0));
    for _ in 0..segment_count {
        angle += segment_angle;
        let end_point = point_at(angle);
        let end_tangent = tangent_at(angle);
        let cross = first_tangent.cross(end_tangent);
        let denominator = cross.length_squared();
        if denominator <= EPS {
            return Err("makeArc: arc tangents are parallel".into());
        }
        let distance = end_point.sub(first_point).cross(end_tangent).dot(cross) / denominator;
        let middle = first_point.add(first_tangent.scale(distance));
        points.push(Vec4::from_point(middle, middle_weight));
        points.push(Vec4::from_point(end_point, 1.0));
        first_point = end_point;
        first_tangent = end_tangent;
    }
    let mut knots = vec![0.0, 0.0, 0.0];
    for index in 1..segment_count {
        let knot = index as f64 / segment_count as f64;
        knots.extend([knot, knot]);
    }
    knots.extend([1.0, 1.0, 1.0]);
    NurbsCurve::new(2, knots, points)
}

pub fn make_circle(center: Vec3, normal: Vec3, radius: f64) -> Result<NurbsCurve, String> {
    let normal = normal.normalized()?;
    let x_axis = normal.perpendicular()?;
    let y_axis = normal.cross(x_axis).normalized()?;
    make_arc(center, x_axis, y_axis, radius, 0.0, std::f64::consts::TAU)
}

/// Build the exactly orthonormal local frame shared by the conic
/// constructors.  The conic algebra below (implicit equation, tangent
/// intersection, shoulder weight) is only exact in an orthonormal frame, so
/// the in-plane hint is Gram-Schmidt-projected against the primary direction
/// instead of trusted verbatim: callers only owe us non-parallel vectors.
fn conic_frame(fn_name: &str, primary: Vec3, hint: Vec3) -> Result<(Vec3, Vec3), String> {
    let x_axis = primary
        .normalized()
        .map_err(|_| format!("{fn_name}: primary axis must be non-zero"))?;
    if hint.length() <= EPS {
        return Err(format!("{fn_name}: in-plane direction must be non-zero"));
    }
    hint.sub(x_axis.scale(hint.dot(x_axis)))
        .normalized()
        .map(|y_axis| (x_axis, y_axis))
        .map_err(|_| format!("{fn_name}: frame directions must not be parallel"))
}

/// Trim-range validation shared by the conic constructors.  The knot span IS
/// the conic parameter range, so it must clear the knot identity band or the
/// resulting curve would fail knot validation with an unhelpful message.
fn conic_range(fn_name: &str, t0: f64, t1: f64) -> Result<(), String> {
    if !t0.is_finite() || !t1.is_finite() {
        return Err(format!("{fn_name}: parameter range must be finite"));
    }
    if t1 - t0 <= KNOT_IDENTITY_TOL {
        return Err(format!("{fn_name}: t1 ({t1}) must exceed t0 ({t0})"));
    }
    Ok(())
}

/// Extreme trim parameters overflow the conic point formulas (cosh exceeds
/// f64 near |t| ~ 710, focal·t² near |t| ~ 1e150); surface that as the
/// constructor's own honest error instead of the generic NurbsCurve
/// finiteness rejection.
fn conic_points_finite(fn_name: &str, points: &[Vec4]) -> Result<(), String> {
    if points.iter().any(|point| {
        ![point.x, point.y, point.z, point.w]
            .iter()
            .all(|value| value.is_finite())
    }) {
        return Err(format!(
            "{fn_name}: control points overflow f64 — parameter range too extreme"
        ));
    }
    Ok(())
}

/// Exact parabola segment y² = 4·focal·x in the local frame (vertex at the
/// origin, `axis` = +x, `latus_direction` = +y), parametrized the standard
/// way P(t) = (focal·t², 2·focal·t) and trimmed to t ∈ [t0, t1].  A parabola
/// segment is a plain quadratic polynomial in t (all weights 1), so a single
/// degree-2 Bézier over the knot span [t0, t1] reproduces both the point set
/// AND the parametrization exactly — no fitting, and evaluate(t) == P(t) for
/// every t, not just at the ends.
pub fn make_parabola(
    vertex: Vec3,
    axis: Vec3,
    latus_direction: Vec3,
    focal: f64,
    t0: f64,
    t1: f64,
) -> Result<NurbsCurve, String> {
    if !focal.is_finite() || focal <= EPS {
        return Err("make_parabola: focal distance must be positive".into());
    }
    conic_range("make_parabola", t0, t1)?;
    let (x_axis, y_axis) = conic_frame("make_parabola", axis, latus_direction)?;
    let point_at = |t: f64| {
        vertex
            .add(x_axis.scale(focal * t * t))
            .add(y_axis.scale(2.0 * focal * t))
    };
    // Bernstein middle point of the quadratic polynomial,
    // P(t0) + (t1−t0)/2·P'(t0), collapses to local (focal·t0·t1,
    // focal·(t0+t1)) — which is also the intersection of the two end
    // tangents, as it must be for any parabola segment.
    let middle = vertex
        .add(x_axis.scale(focal * t0 * t1))
        .add(y_axis.scale(focal * (t0 + t1)));
    let control_points = vec![
        Vec4::from_point(point_at(t0), 1.0),
        Vec4::from_point(middle, 1.0),
        Vec4::from_point(point_at(t1), 1.0),
    ];
    conic_points_finite("make_parabola", &control_points)?;
    NurbsCurve::new(2, vec![t0, t0, t0, t1, t1, t1], control_points)
}

/// Exact arc of the hyperbola branch x²/a² − y²/b² = 1, x > 0 in the local
/// frame (center at the origin, `major_axis` = +x, `minor_axis` = +y),
/// parametrized P(t) = (a·cosh t, b·sinh t) and trimmed to t ∈ [t0, t1].
/// A single rational quadratic Bézier is exact for any sweep on one branch:
/// endpoints on the curve, middle control point at the intersection of the
/// end tangents, and middle weight cosh((t1−t0)/2) chosen so the shoulder
/// point lands back on the branch.  Only the point set is hyperbola-exact
/// away from the ends; the NURBS parameter coincides with the hyperbolic
/// parameter t exactly at t0, (t0+t1)/2 and t1.
pub fn make_hyperbola(
    center: Vec3,
    major_axis: Vec3,
    minor_axis: Vec3,
    a: f64,
    b: f64,
    t0: f64,
    t1: f64,
) -> Result<NurbsCurve, String> {
    if !a.is_finite() || a <= EPS {
        return Err("make_hyperbola: semi-axis a must be positive".into());
    }
    if !b.is_finite() || b <= EPS {
        return Err("make_hyperbola: semi-axis b must be positive".into());
    }
    conic_range("make_hyperbola", t0, t1)?;
    let (x_axis, y_axis) = conic_frame("make_hyperbola", major_axis, minor_axis)?;
    let point_at = |t: f64| {
        center
            .add(x_axis.scale(a * t.cosh()))
            .add(y_axis.scale(b * t.sinh()))
    };
    let mid = 0.5 * (t0 + t1);
    let middle_weight = (0.5 * (t1 - t0)).cosh();
    // In the scaled frame (x/a, y/b) the branch is the unit hyperbola and the
    // tangent at t is the line X·cosh t − Y·sinh t = 1; Cramer's rule on the
    // two end-tangent lines collapses their intersection to
    // P(mid)/cosh(half-sweep), so no explicit line-line solve is needed.
    // With this middle point, weight cosh(half-sweep) puts the shoulder point
    // (P0 + 2w·P1 + P2)/(2 + 2w) back on the branch at P(mid).
    let apex = center
        .add(x_axis.scale(a * mid.cosh() / middle_weight))
        .add(y_axis.scale(b * mid.sinh() / middle_weight));
    let control_points = vec![
        Vec4::from_point(point_at(t0), 1.0),
        Vec4::from_point(apex, middle_weight),
        Vec4::from_point(point_at(t1), 1.0),
    ];
    conic_points_finite("make_hyperbola", &control_points)?;
    NurbsCurve::new(2, vec![t0, t0, t0, t1, t1, t1], control_points)
}

pub fn uniform_clamped_knots(
    control_point_count: usize,
    degree: usize,
) -> Result<Vec<f64>, String> {
    if control_point_count == 0 || control_point_count - 1 < degree {
        return Err("uniformClampedKnots: need at least degree+1 control points".into());
    }
    let n = control_point_count - 1;
    let mut knots = vec![0.0; degree + 1];
    let interior = n - degree;
    for index in 1..=interior {
        knots.push(index as f64 / (interior + 1) as f64);
    }
    knots.extend(std::iter::repeat_n(1.0, degree + 1));
    Ok(knots)
}

fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    (1..=k).fold(1.0, |value, index| {
        value * (n - k + index) as f64 / index as f64
    })
}



