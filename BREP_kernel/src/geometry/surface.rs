use crate::curve::{
    basis_derivatives_into, basis_functions_into, knot_clamp, knot_domain, knot_find_span,
    validate_knots, MAX_STACK_DEGREE, MAX_STACK_ORDER,
};
use crate::{KnotVector, NurbsCurve, Vec3, Vec4};
use serde::{Deserialize, Serialize};

#[path = "surface/extension.rs"]
mod extension;
pub use extension::{
    ExtendRefusal, SurfaceSide, WeightSite, MAXIMUM_FOLD_ANGLE, MAXIMUM_GROWTH_RATIO,
};

const EPS: f64 = 1e-12;
/// A generatrix control point that is neither ON the axis of revolution nor
/// this fraction of the generatrix's own radial extent clear of it is a fuzzy
/// pole, and `make_revolution` refuses it (see the comment at the check).
const NEAR_AXIS_RELATIVE_TOLERANCE: f64 = 1e-4;

/// The widest tensor block [`NurbsSurface::deriv1_tensor_each`] will evaluate,
/// per direction. The mass integrator reads a RATIONAL patch's cell at up to
/// this many stations; `props::mass_properties::rule::MAX_ORDER` is DEFINED as
/// this constant rather than repeating its value, because the block's arrays
/// here and the station arrays there must agree and a drift between them is a
/// hard failure, not a slow path (the callers propagate the refusal below
/// rather than falling back). 8 was the fixed rule's own width; the arrays are
/// stack-resident either way.
pub(crate) const TENSOR_MAX_BLOCK: usize = 24;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct NurbsSurface {
    pub degree_u: usize,
    pub degree_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub control_points: Vec<Vec<Vec4>>,
    /// Lazily recognized analytic carrier (plane/cylinder/cone/sphere/torus).
    /// Purely an acceleration cache: never serialized, cloned with the
    /// surface, and recomputed on demand after deserialization.
    #[serde(skip, default)]
    analytic: std::cell::OnceCell<Option<crate::AnalyticSurface>>,
    /// One-time validation cache (see `NurbsCurve::ensure_valid`).
    #[serde(skip, default)]
    validated: std::cell::Cell<bool>,
    /// Cached (closed_u, closed_v) seam test — a pure function of the
    /// immutable geometry, recomputed on demand after deserialization.
    #[serde(skip, default)]
    closed_directions: std::cell::OnceCell<(bool, bool)>,
    /// Cached Newton seed grid for general point projection (see
    /// `projection.rs`) — (u, v, point) samples over the knot spans.
    #[serde(skip, default)]
    pub(crate) projection_grid: std::cell::OnceCell<Vec<(f64, f64, Vec3)>>,
    /// Cached dense fallback grid for degenerate general projection
    /// (`projection.rs`) — (u, v, point) samples at fixed per-surface
    /// parameters, so the ~1.7M-call metre-mm fallback reuses the evals.
    #[serde(skip, default)]
    pub(crate) projection_dense_grid: std::cell::OnceCell<Vec<(f64, f64, Vec3)>>,
    /// Cached boundary-ring sample points for the degenerate projection
    /// fallback (`projection.rs`), flattened as 4 sides × 22 exponents ×
    /// 33 indices of evaluated surface points at fixed per-surface params.
    #[serde(skip, default)]
    pub(crate) projection_ring_grid: std::cell::OnceCell<Vec<Vec3>>,
    /// Cached convex-hull enclosure of every projection lattice cell
    /// (`projection.rs`'s global stage), built on the first query the local
    /// answer does not already settle. `None` when the net cannot be refined.
    #[serde(skip, default)]
    pub(crate) projection_cells: std::cell::OnceCell<Option<crate::projection::ProjectionCells>>,
}

impl NurbsSurface {
    pub fn new(
        degree_u: usize,
        degree_v: usize,
        knots_u: Vec<f64>,
        knots_v: Vec<f64>,
        control_points: Vec<Vec<Vec4>>,
    ) -> Result<Self, String> {
        let surface = Self {
            degree_u,
            degree_v,
            knots_u,
            knots_v,
            control_points,
            analytic: std::cell::OnceCell::new(),
            validated: std::cell::Cell::new(false),
            closed_directions: std::cell::OnceCell::new(),
            projection_grid: std::cell::OnceCell::new(),
            projection_dense_grid: std::cell::OnceCell::new(),
            projection_ring_grid: std::cell::OnceCell::new(),
            projection_cells: std::cell::OnceCell::new(),
        };
        surface.ensure_valid()?;
        Ok(surface)
    }

    /// Whether the surface joins itself along u (first) and v (second):
    /// opposite domain edges coincide at three sampled fractions. The 1e-6
    /// threshold matches the historical `LINEAR_TOLERANCE * 10` used by the
    /// projection and curve×surface intersection modules.
    pub fn closed_directions(&self) -> Result<(bool, bool), String> {
        if let Some(&cached) = self.closed_directions.get() {
            return Ok(cached);
        }
        const CLOSED_SEAM_TOLERANCE: f64 = 1e-6;
        let [u0, u1] = self.domain_u()?;
        let [v0, v1] = self.domain_v()?;
        let closed_along = |direction_u: bool| -> Result<bool, String> {
            // A direction whose two boundary rows coincide is closed only if
            // the surface TRAVELS between them: a sliver patch narrower than
            // the seam tolerance (a 7.24e-7-wide plane our own STL→STEP writer
            // emits for a degenerate facet, 2026-09-26) has coincident rows
            // too, and reading it as periodic wraps its far edge onto its near
            // one — `evaluate_extended` then images a trim on that edge a
            // whole patch width away.
            let mut travels = false;
            for fraction in [0.17, 0.5, 0.83] {
                let (a, b, middle) = if direction_u {
                    let v = v0 + (v1 - v0) * fraction;
                    (
                        self.evaluate(u0, v)?,
                        self.evaluate(u1, v)?,
                        self.evaluate(0.5 * (u0 + u1), v)?,
                    )
                } else {
                    let u = u0 + (u1 - u0) * fraction;
                    (
                        self.evaluate(u, v0)?,
                        self.evaluate(u, v1)?,
                        self.evaluate(u, 0.5 * (v0 + v1))?,
                    )
                };
                if a.sub(b).length() > CLOSED_SEAM_TOLERANCE {
                    return Ok(false);
                }
                if middle.sub(a).length() > CLOSED_SEAM_TOLERANCE {
                    travels = true;
                }
            }
            Ok(travels)
        };
        let value = (closed_along(true)?, closed_along(false)?);
        Ok(*self.closed_directions.get_or_init(|| value))
    }

    /// The full construction-time checks, run at most once per instance.
    fn ensure_valid(&self) -> Result<(), String> {
        if self.validated.get() {
            return Ok(());
        }
        validate_knots(&self.knots_u, self.degree_u)?;
        validate_knots(&self.knots_v, self.degree_v)?;
        let rows = self.knots_u.len() - self.degree_u - 1;
        let columns = self.knots_v.len() - self.degree_v - 1;
        if self.control_points.len() != rows {
            return Err(format!(
                "NurbsSurface: u knot vector implies {} rows, got {}",
                rows,
                self.control_points.len()
            ));
        }
        for row in &self.control_points {
            if row.len() != columns {
                return Err(format!(
                    "NurbsSurface: v knot vector implies {} columns, got {}",
                    columns,
                    row.len()
                ));
            }
            if row.iter().any(|point| {
                point.w <= EPS
                    || ![point.x, point.y, point.z, point.w]
                        .iter()
                        .all(|value| value.is_finite())
            }) {
                return Err(
                    "NurbsSurface: control points must be finite with positive weights".into(),
                );
            }
        }
        self.validated.set(true);
        Ok(())
    }

    pub fn domain_u(&self) -> Result<[f64; 2], String> {
        self.ensure_valid()?;
        Ok(knot_domain(&self.knots_u, self.degree_u))
    }

    pub fn domain_v(&self) -> Result<[f64; 2], String> {
        self.ensure_valid()?;
        Ok(knot_domain(&self.knots_v, self.degree_v))
    }

    /// The recognized analytic carrier, if this exact rational patch is a
    /// plane or a full revolution quadric/torus. Computed once per instance.
    pub fn analytic(&self) -> Option<&crate::AnalyticSurface> {
        self.analytic
            .get_or_init(|| crate::analytic_surface::recognize(self))
            .as_ref()
    }

    /// The names of this surface's caches that hold a value, in declaration
    /// order. For the boolean's operand-state instrument
    /// (`BREP_DEBUG_OPERAND_STATE`), which reads which caches arrive warm.
    pub(crate) fn warm_caches(&self) -> Vec<&'static str> {
        let mut warm = Vec::new();
        let cells = [
            ("analytic", self.analytic.get().is_some()),
            ("validated", self.validated.get()),
            ("closed", self.closed_directions.get().is_some()),
            ("grid", self.projection_grid.get().is_some()),
            ("dense", self.projection_dense_grid.get().is_some()),
            ("ring", self.projection_ring_grid.get().is_some()),
            ("cells", self.projection_cells.get().is_some()),
        ];
        for (name, held) in cells {
            if held {
                warm.push(name);
            }
        }
        warm
    }

    /// Empties the named caches (`warm_caches`' names; `all` for every one),
    /// or, with `from`, sets each to `from`'s value. For the operand-state
    /// instrument only: a cache that decides an answer is a defect, and this
    /// is how the bisect finds it.
    pub(crate) fn exchange_caches(&mut self, names: &[&str], from: Option<&Self>) {
        let wants = |name: &str| names.iter().any(|&n| n == name || n == "all");
        if wants("analytic") {
            self.analytic = from.map(|f| f.analytic.clone()).unwrap_or_default();
        }
        if wants("validated") {
            self.validated = std::cell::Cell::new(from.is_some_and(|f| f.validated.get()));
        }
        if wants("closed") {
            self.closed_directions = from.map(|f| f.closed_directions.clone()).unwrap_or_default();
        }
        if wants("grid") {
            self.projection_grid = from.map(|f| f.projection_grid.clone()).unwrap_or_default();
        }
        if wants("dense") {
            self.projection_dense_grid = from.map(|f| f.projection_dense_grid.clone()).unwrap_or_default();
        }
        if wants("ring") {
            self.projection_ring_grid = from.map(|f| f.projection_ring_grid.clone()).unwrap_or_default();
        }
        if wants("cells") {
            self.projection_cells = from.map(|f| f.projection_cells.clone()).unwrap_or_default();
        }
    }

    fn knot_vectors(&self) -> Result<(KnotVector, KnotVector), String> {
        Ok((
            KnotVector::new(self.knots_u.clone(), self.degree_u)?,
            KnotVector::new(self.knots_v.clone(), self.degree_v)?,
        ))
    }

    pub fn evaluate_homogeneous(&self, u: f64, v: f64) -> Result<Vec4, String> {
        self.ensure_valid()?;
        if self.degree_u > MAX_STACK_DEGREE || self.degree_v > MAX_STACK_DEGREE {
            return self.evaluate_homogeneous_heap(u, v);
        }
        let u = knot_clamp(&self.knots_u, self.degree_u, u);
        let v = knot_clamp(&self.knots_v, self.degree_v, v);
        let span_u = knot_find_span(&self.knots_u, self.degree_u, u);
        let span_v = knot_find_span(&self.knots_v, self.degree_v, v);
        let mut basis_u = [0.0f64; MAX_STACK_ORDER];
        let mut basis_v = [0.0f64; MAX_STACK_ORDER];
        basis_functions_into(&self.knots_u, self.degree_u, span_u, u, &mut basis_u);
        basis_functions_into(&self.knots_v, self.degree_v, span_v, v, &mut basis_v);
        let mut point = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        for (i, value_u) in basis_u[..=self.degree_u].iter().enumerate() {
            let row = &self.control_points[span_u - self.degree_u + i];
            for (j, value_v) in basis_v[..=self.degree_v].iter().enumerate() {
                point = point.add(row[span_v - self.degree_v + j].scale(value_u * value_v));
            }
        }
        Ok(point)
    }

    fn evaluate_homogeneous_heap(&self, u: f64, v: f64) -> Result<Vec4, String> {
        let (knot_u, knot_v) = self.knot_vectors()?;
        let u = knot_u.clamp_param(u);
        let v = knot_v.clamp_param(v);
        let span_u = knot_u.find_span(u);
        let span_v = knot_v.find_span(v);
        let basis_u = knot_u.basis_functions(span_u, u);
        let basis_v = knot_v.basis_functions(span_v, v);
        let mut point = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        for (i, value_u) in basis_u.iter().enumerate() {
            let row = &self.control_points[span_u - self.degree_u + i];
            for (j, value_v) in basis_v.iter().enumerate() {
                point = point.add(row[span_v - self.degree_v + j].scale(value_u * value_v));
            }
        }
        Ok(point)
    }

    pub fn evaluate(&self, u: f64, v: f64) -> Result<Vec3, String> {
        self.evaluate_homogeneous(u, v)?.point()
    }

    /// Value beyond the domain (Golovanov §3.15): closed directions wrap
    /// cyclically; open directions extend along the boundary tangent
    /// plane, with the bilinear corner form when both parameters are
    /// outside.  Intersection and projection algorithms probe outside the
    /// domain, so this is a required capability of every surface.
    pub fn evaluate_extended(&self, u: f64, v: f64) -> Result<Vec3, String> {
        Ok(self.derivatives_extended(u, v, 0)?[0][0])
    }

    /// Derivatives beyond the domain (§3.15).  The extension is C¹: first
    /// derivatives follow the ruled/bilinear extension, second derivatives
    /// are taken at the boundary anchor (zero across an extended
    /// direction).
    pub fn derivatives_extended(
        &self,
        u: f64,
        v: f64,
        derivative_count: usize,
    ) -> Result<Vec<Vec<Vec3>>, String> {
        let [u0, u1] = self.domain_u()?;
        let [v0, v1] = self.domain_v()?;
        let (closed_u, closed_v) = self.closed_directions()?;
        let mut uu = u;
        let mut vv = v;
        if closed_u && (u < u0 || u > u1) {
            uu = u0 + (u - u0).rem_euclid(u1 - u0);
        }
        if closed_v && (v < v0 || v > v1) {
            vv = v0 + (v - v0).rem_euclid(v1 - v0);
        }
        let du_out = if uu < u0 {
            uu - u0
        } else if uu > u1 {
            uu - u1
        } else {
            0.0
        };
        let dv_out = if vv < v0 {
            vv - v0
        } else if vv > v1 {
            vv - v1
        } else {
            0.0
        };
        if du_out == 0.0 && dv_out == 0.0 {
            return self.derivatives(uu, vv, derivative_count);
        }
        let anchor_u = uu - du_out;
        let anchor_v = vv - dv_out;
        let base = self.derivatives(anchor_u, anchor_v, derivative_count.max(2))?;
        let order = derivative_count + 1;
        let mut result = vec![vec![Vec3::default(); order]; order];
        // Bilinear extension: S = A00 + du·A10 + dv·A01 + du·dv·A11
        // (reduces to the ruled tangent-plane extension when only one
        // parameter is outside).
        result[0][0] = base[0][0]
            .add(base[1][0].scale(du_out))
            .add(base[0][1].scale(dv_out))
            .add(base[1][1].scale(du_out * dv_out));
        if derivative_count >= 1 {
            result[1][0] = base[1][0].add(base[1][1].scale(dv_out));
            result[0][1] = base[0][1].add(base[1][1].scale(du_out));
            result[1][1] = base[1][1];
        }
        if derivative_count >= 2 {
            let second = self.derivatives(anchor_u, anchor_v, 2)?;
            if du_out == 0.0 {
                result[2][0] = second[2][0];
            }
            if dv_out == 0.0 {
                result[0][2] = second[0][2];
            }
        }
        Ok(result)
    }

    pub fn derivatives(
        &self,
        u: f64,
        v: f64,
        derivative_count: usize,
    ) -> Result<Vec<Vec<Vec3>>, String> {
        self.ensure_valid()?;
        let du = derivative_count.min(self.degree_u);
        let dv = derivative_count.min(self.degree_v);
        let stride = derivative_count + 1;
        // Flat (k, l) grid: one allocation instead of nested per-row Vecs.
        let zero = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let mut homogeneous = vec![zero; stride * stride];
        if self.degree_u <= MAX_STACK_DEGREE && self.degree_v <= MAX_STACK_DEGREE {
            let u = knot_clamp(&self.knots_u, self.degree_u, u);
            let v = knot_clamp(&self.knots_v, self.degree_v, v);
            let span_u = knot_find_span(&self.knots_u, self.degree_u, u);
            let span_v = knot_find_span(&self.knots_v, self.degree_v, v);
            let mut basis_u = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            let mut basis_v = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            basis_derivatives_into(
                &self.knots_u,
                self.degree_u,
                span_u,
                u,
                du,
                &mut basis_u[..=du],
            );
            basis_derivatives_into(
                &self.knots_v,
                self.degree_v,
                span_v,
                v,
                dv,
                &mut basis_v[..=dv],
            );
            for k in 0..=du {
                for l in 0..=dv {
                    if k + l > derivative_count {
                        continue;
                    }
                    let mut point = zero;
                    for i in 0..=self.degree_u {
                        let row = &self.control_points[span_u - self.degree_u + i];
                        for j in 0..=self.degree_v {
                            point = point.add(
                                row[span_v - self.degree_v + j]
                                    .scale(basis_u[k][i] * basis_v[l][j]),
                            );
                        }
                    }
                    homogeneous[k * stride + l] = point;
                }
            }
        } else {
            let (knot_u, knot_v) = self.knot_vectors()?;
            let u = knot_u.clamp_param(u);
            let v = knot_v.clamp_param(v);
            let span_u = knot_u.find_span(u);
            let span_v = knot_v.find_span(v);
            let basis_u = knot_u.basis_derivatives(span_u, u, du);
            let basis_v = knot_v.basis_derivatives(span_v, v, dv);
            for k in 0..=du {
                for l in 0..=dv {
                    if k + l > derivative_count {
                        continue;
                    }
                    let mut point = zero;
                    for i in 0..=self.degree_u {
                        let row = &self.control_points[span_u - self.degree_u + i];
                        for j in 0..=self.degree_v {
                            point = point.add(
                                row[span_v - self.degree_v + j]
                                    .scale(basis_u[k][i] * basis_v[l][j]),
                            );
                        }
                    }
                    homogeneous[k * stride + l] = point;
                }
            }
        }

        let mut result = vec![vec![Vec3::default(); derivative_count + 1]; derivative_count + 1];
        let weight = homogeneous[0].w;
        if weight.abs() <= EPS {
            return Err("NurbsSurface: zero evaluated weight".into());
        }
        for k in 0..=derivative_count {
            for l in 0..=derivative_count - k {
                if k > du || l > dv {
                    continue;
                }
                let at = |row: usize, column: usize| homogeneous[row * stride + column];
                let mut value = Vec3::new(at(k, l).x, at(k, l).y, at(k, l).z);
                for j in 1..=l {
                    value = value.sub(result[k][l - j].scale(binomial(l, j) * at(0, j).w));
                }
                for i in 1..=k {
                    value = value.sub(result[k - i][l].scale(binomial(k, i) * at(i, 0).w));
                    let mut mixed = Vec3::default();
                    for j in 1..=l {
                        mixed = mixed.add(result[k - i][l - j].scale(binomial(l, j) * at(i, j).w));
                    }
                    value = value.sub(mixed.scale(binomial(k, i)));
                }
                result[k][l] = value.scale(1.0 / weight);
            }
        }
        Ok(result)
    }

    /// Allocation-free twin of [`Self::derivatives`] for the hot
    /// `derivative_count <= 2` path (point, first, and second partials).
    ///
    /// This is a byte-for-byte faithful copy of the arithmetic in
    /// [`Self::derivatives`]: identical basis evaluation, identical summation
    /// order (control-point row `i` outer, column `j` inner, `scale`-then-`add`),
    /// and the identical rational `A(u,v)/w(u,v)` de-homogenization recurrence.
    /// Only the storage differs — fixed-size stack arrays (`[Vec4; 9]` for the
    /// `(count+1)²` homogeneous grid, `[[Vec3; 3]; 3]` for the result) replace
    /// the heap `Vec<Vec4>` / `Vec<Vec<Vec3>>`. Grid entries outside
    /// `k + l <= derivative_count` (and outside `k <= du`, `l <= dv`) stay
    /// `Vec3::default()`, exactly as the heap version leaves them, so callers may
    /// index `[k][l]` for any `k + l <= 2` and read the same value the Vec API
    /// would have produced.
    pub(crate) fn derivatives_small(
        &self,
        u: f64,
        v: f64,
        derivative_count: usize,
    ) -> Result<[[Vec3; 3]; 3], String> {
        debug_assert!(derivative_count <= 2);
        self.ensure_valid()?;
        let du = derivative_count.min(self.degree_u);
        let dv = derivative_count.min(self.degree_v);
        let stride = derivative_count + 1;
        let zero = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let mut homogeneous = [zero; 9];
        if self.degree_u <= MAX_STACK_DEGREE && self.degree_v <= MAX_STACK_DEGREE {
            let u = knot_clamp(&self.knots_u, self.degree_u, u);
            let v = knot_clamp(&self.knots_v, self.degree_v, v);
            let span_u = knot_find_span(&self.knots_u, self.degree_u, u);
            let span_v = knot_find_span(&self.knots_v, self.degree_v, v);
            let mut basis_u = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            let mut basis_v = [[0.0f64; MAX_STACK_ORDER]; MAX_STACK_ORDER];
            basis_derivatives_into(
                &self.knots_u,
                self.degree_u,
                span_u,
                u,
                du,
                &mut basis_u[..=du],
            );
            basis_derivatives_into(
                &self.knots_v,
                self.degree_v,
                span_v,
                v,
                dv,
                &mut basis_v[..=dv],
            );
            for k in 0..=du {
                for l in 0..=dv {
                    if k + l > derivative_count {
                        continue;
                    }
                    let mut point = zero;
                    for i in 0..=self.degree_u {
                        let row = &self.control_points[span_u - self.degree_u + i];
                        for j in 0..=self.degree_v {
                            point = point.add(
                                row[span_v - self.degree_v + j]
                                    .scale(basis_u[k][i] * basis_v[l][j]),
                            );
                        }
                    }
                    homogeneous[k * stride + l] = point;
                }
            }
        } else {
            let (knot_u, knot_v) = self.knot_vectors()?;
            let u = knot_u.clamp_param(u);
            let v = knot_v.clamp_param(v);
            let span_u = knot_u.find_span(u);
            let span_v = knot_v.find_span(v);
            let basis_u = knot_u.basis_derivatives(span_u, u, du);
            let basis_v = knot_v.basis_derivatives(span_v, v, dv);
            for k in 0..=du {
                for l in 0..=dv {
                    if k + l > derivative_count {
                        continue;
                    }
                    let mut point = zero;
                    for i in 0..=self.degree_u {
                        let row = &self.control_points[span_u - self.degree_u + i];
                        for j in 0..=self.degree_v {
                            point = point.add(
                                row[span_v - self.degree_v + j]
                                    .scale(basis_u[k][i] * basis_v[l][j]),
                            );
                        }
                    }
                    homogeneous[k * stride + l] = point;
                }
            }
        }

        let mut result = [[Vec3::default(); 3]; 3];
        let weight = homogeneous[0].w;
        if weight.abs() <= EPS {
            return Err("NurbsSurface: zero evaluated weight".into());
        }
        for k in 0..=derivative_count {
            for l in 0..=derivative_count - k {
                if k > du || l > dv {
                    continue;
                }
                let at = |row: usize, column: usize| homogeneous[row * stride + column];
                let mut value = Vec3::new(at(k, l).x, at(k, l).y, at(k, l).z);
                for j in 1..=l {
                    value = value.sub(result[k][l - j].scale(binomial(l, j) * at(0, j).w));
                }
                for i in 1..=k {
                    value = value.sub(result[k - i][l].scale(binomial(k, i) * at(i, 0).w));
                    let mut mixed = Vec3::default();
                    for j in 1..=l {
                        mixed = mixed.add(result[k - i][l - j].scale(binomial(l, j) * at(i, j).w));
                    }
                    value = value.sub(mixed.scale(binomial(k, i)));
                }
                result[k][l] = value.scale(1.0 / weight);
            }
        }
        Ok(result)
    }

    /// Whether [`Self::deriv1_tensor_each`] can drive this patch: the stack
    /// basis tables only reach `MAX_STACK_DEGREE`, and a higher-degree patch
    /// must be evaluated station by station through [`Self::deriv1`].
    #[inline]
    pub(crate) fn deriv1_tensor_supported(&self) -> bool {
        self.degree_u <= MAX_STACK_DEGREE && self.degree_v <= MAX_STACK_DEGREE
    }

    /// `(S, S_u, S_v)` at every station of the TENSOR BLOCK `us x vs`, handed
    /// to `f` as `(u index, v index, S, S_u, S_v)` with `us` outer and `vs`
    /// inner.
    ///
    /// Bit-identical to calling [`Self::deriv1`] at each pair, and that is the
    /// whole point of it: the block's `us.len() + vs.len()` spans and basis
    /// rows are evaluated ONCE each instead of once per station, which is
    /// where a fixed tensor-Gauss cell spends most of its time (the basis
    /// recurrence carries a division per knot per derivative order, and an
    /// 8x8 cell asks for the same 8 u rows and 8 v rows sixty-four times).
    /// Nothing else changes: `knot_clamp` / `knot_find_span` /
    /// `basis_derivatives_into` see the same arguments and so return the same
    /// floats, the control-point sum runs in the same row-outer/column-inner
    /// order from the same zero `Vec4`, and the rational de-homogenization is
    /// [`Self::derivatives_small`]'s recurrence written out for
    /// `derivative_count == 1`.
    ///
    /// Two steps of that recurrence are elided because they are exact no-ops
    /// at this order: `binomial(1, 1)` is exactly `1.0`, and the mixed term it
    /// scales is the zero vector, whose subtraction leaves every component
    /// (including a negative zero) bit-unchanged.
    pub(crate) fn deriv1_tensor_each<F>(
        &self,
        us: &[f64],
        vs: &[f64],
        mut f: F,
    ) -> Result<(), String>
    where
        F: FnMut(usize, usize, Vec3, Vec3, Vec3) -> Result<(), String>,
    {
        self.ensure_valid()?;
        // Width and rationale: `TENSOR_MAX_BLOCK` at the top of this module.
        const MAX_BLOCK: usize = TENSOR_MAX_BLOCK;
        if !self.deriv1_tensor_supported() || us.len() > MAX_BLOCK || vs.len() > MAX_BLOCK {
            return Err("NurbsSurface: tensor block outside the stack evaluator".into());
        }
        let du = 1usize.min(self.degree_u);
        let dv = 1usize.min(self.degree_v);
        let mut span_u = [0usize; MAX_BLOCK];
        let mut span_v = [0usize; MAX_BLOCK];
        let mut basis_u = [[[0.0f64; MAX_STACK_ORDER]; 2]; MAX_BLOCK];
        let mut basis_v = [[[0.0f64; MAX_STACK_ORDER]; 2]; MAX_BLOCK];
        for (index, &value) in us.iter().enumerate() {
            let value = knot_clamp(&self.knots_u, self.degree_u, value);
            span_u[index] = knot_find_span(&self.knots_u, self.degree_u, value);
            basis_derivatives_into(
                &self.knots_u,
                self.degree_u,
                span_u[index],
                value,
                du,
                &mut basis_u[index][..=du],
            );
        }
        for (index, &value) in vs.iter().enumerate() {
            let value = knot_clamp(&self.knots_v, self.degree_v, value);
            span_v[index] = knot_find_span(&self.knots_v, self.degree_v, value);
            basis_derivatives_into(
                &self.knots_v,
                self.degree_v,
                span_v[index],
                value,
                dv,
                &mut basis_v[index][..=dv],
            );
        }
        let zero = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        for i in 0..us.len() {
            let rows = &self.control_points[span_u[i] - self.degree_u..];
            for j in 0..vs.len() {
                let column = span_v[j] - self.degree_v;
                // The (k, l) homogeneous grid `derivatives_small` builds for
                // `derivative_count == 1`: (0,0) always, (0,1) when dv >= 1,
                // (1,0) when du >= 1.
                let sum = |k: usize, l: usize| {
                    let mut point = zero;
                    for row_index in 0..=self.degree_u {
                        let row = &rows[row_index];
                        for column_index in 0..=self.degree_v {
                            point = point.add(
                                row[column + column_index]
                                    .scale(basis_u[i][k][row_index] * basis_v[j][l][column_index]),
                            );
                        }
                    }
                    point
                };
                let h00 = sum(0, 0);
                let h01 = if dv >= 1 { sum(0, 1) } else { zero };
                let h10 = if du >= 1 { sum(1, 0) } else { zero };
                let weight = h00.w;
                if weight.abs() <= EPS {
                    return Err("NurbsSurface: zero evaluated weight".into());
                }
                let inverse = 1.0 / weight;
                let point = Vec3::new(h00.x, h00.y, h00.z).scale(inverse);
                let partial_v = if dv >= 1 {
                    Vec3::new(h01.x, h01.y, h01.z)
                        .sub(point.scale(h01.w))
                        .scale(inverse)
                } else {
                    Vec3::default()
                };
                let partial_u = if du >= 1 {
                    Vec3::new(h10.x, h10.y, h10.z)
                        .sub(point.scale(h10.w))
                        .scale(inverse)
                } else {
                    Vec3::default()
                };
                f(i, j, point, partial_u, partial_v)?;
            }
        }
        Ok(())
    }

    /// Point and first partials `(S, S_u, S_v)` with zero heap allocation.
    /// Bit-identical to `derivatives(u, v, 1)` at `[0][0] / [1][0] / [0][1]`.
    #[inline]
    pub(crate) fn deriv1(&self, u: f64, v: f64) -> Result<(Vec3, Vec3, Vec3), String> {
        let g = self.derivatives_small(u, v, 1)?;
        Ok((g[0][0], g[1][0], g[0][1]))
    }

    /// Allocation-free `(S, S_u, S_v)` twin of `derivatives_extended(u, v, 1)`.
    ///
    /// Mirrors the exact C¹ ruled/bilinear extension arithmetic of
    /// [`Self::derivatives_extended`] (only the count == 1 partials), but drives
    /// it from [`Self::derivatives_small`] so the in-domain and boundary-anchor
    /// evaluations allocate nothing. Bit-identical to
    /// `derivatives_extended(u, v, 1)` at `[0][0] / [1][0] / [0][1]`.
    pub(crate) fn deriv1_extended(&self, u: f64, v: f64) -> Result<(Vec3, Vec3, Vec3), String> {
        let [u0, u1] = self.domain_u()?;
        let [v0, v1] = self.domain_v()?;
        let (closed_u, closed_v) = self.closed_directions()?;
        let mut uu = u;
        let mut vv = v;
        if closed_u && (u < u0 || u > u1) {
            uu = u0 + (u - u0).rem_euclid(u1 - u0);
        }
        if closed_v && (v < v0 || v > v1) {
            vv = v0 + (v - v0).rem_euclid(v1 - v0);
        }
        let du_out = if uu < u0 {
            uu - u0
        } else if uu > u1 {
            uu - u1
        } else {
            0.0
        };
        let dv_out = if vv < v0 {
            vv - v0
        } else if vv > v1 {
            vv - v1
        } else {
            0.0
        };
        if du_out == 0.0 && dv_out == 0.0 {
            let g = self.derivatives_small(uu, vv, 1)?;
            return Ok((g[0][0], g[1][0], g[0][1]));
        }
        let anchor_u = uu - du_out;
        let anchor_v = vv - dv_out;
        let base = self.derivatives_small(anchor_u, anchor_v, 2)?;
        // Bilinear extension: S = A00 + du·A10 + dv·A01 + du·dv·A11.
        // The mixed anchor derivative has total order two, even when
        // the caller requests only the value or first partials.
        let s = base[0][0]
            .add(base[1][0].scale(du_out))
            .add(base[0][1].scale(dv_out))
            .add(base[1][1].scale(du_out * dv_out));
        let su = base[1][0].add(base[1][1].scale(dv_out));
        let sv = base[0][1].add(base[1][1].scale(du_out));
        Ok((s, su, sv))
    }

    /// One first partial at `(u, v)` (clamped into the domain), `S_u` when
    /// `along_u`, else `S_v`, evaluated from the DIFFERENCES of the control net
    /// rather than from the basis derivatives.
    ///
    /// [`Self::derivatives`] forms `S_u = (A_u − W_u·S) / W` from `A_u = Σ N′ᵢ·Hᵢ`,
    /// a sum of terms the size of the control points whose exact total may be
    /// zero. Its rounding is therefore ABSOLUTE, a few ulps of the point, and
    /// where the partial itself vanishes (an isoline on which the
    /// parameterization is stationary, as on an extrusion of a cubic with
    /// coincident end poles) the DIRECTION of what is left is that rounding:
    /// measured 2.7e-15 of `x` on a partial of 7.7e-5 along `y`, a normal tilted
    /// by 3.5e-11.
    ///
    /// The hodograph `A_u = Σ N_{i+1,p−1}·p·(H_{i+1} − H_i)/(ξ_{i+p+1} − ξ_{i+1})`
    /// (The NURBS Book eq. 3.4, per row) carries the same partial with rounding
    /// that scales with its own terms: coincident control points difference to
    /// an exact zero, and every other difference is weighted by a basis function
    /// that vanishes with the partial. The rational part subtracts
    /// `W_u·S` with `W_u` taken from the weight differences, which is an exact
    /// zero wherever neighbouring weights are equal.
    ///
    /// Not bit-identical to `derivatives`, so it is read only where the bare
    /// partial has lost its direction (`offset/point.rs`). Refuses above the
    /// stack basis tables' degree.
    pub(crate) fn partial_by_differences(
        &self,
        along_u: bool,
        u: f64,
        v: f64,
    ) -> Result<Vec3, String> {
        self.ensure_valid()?;
        if self.degree_u > MAX_STACK_DEGREE || self.degree_v > MAX_STACK_DEGREE {
            return Err("NurbsSurface: partial by differences exceeds the stack degree".into());
        }
        let (pu, pv) = (self.degree_u, self.degree_v);
        let u = knot_clamp(&self.knots_u, pu, u);
        let v = knot_clamp(&self.knots_v, pv, v);
        let span_u = knot_find_span(&self.knots_u, pu, u);
        let span_v = knot_find_span(&self.knots_v, pv, v);
        let mut full_u = [0.0f64; MAX_STACK_ORDER];
        let mut full_v = [0.0f64; MAX_STACK_ORDER];
        basis_functions_into(&self.knots_u, pu, span_u, u, &mut full_u);
        basis_functions_into(&self.knots_v, pv, span_v, v, &mut full_v);
        let zero = Vec4 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 0.0,
        };
        let control = |i: usize, j: usize| self.control_points[span_u - pu + i][span_v - pv + j];
        let mut point = zero;
        for i in 0..=pu {
            for j in 0..=pv {
                point = point.add(control(i, j).scale(full_u[i] * full_v[j]));
            }
        }
        if point.w.abs() <= EPS {
            return Err("NurbsSurface: zero evaluated weight".into());
        }
        let source = Vec3::new(point.x, point.y, point.z).scale(1.0 / point.w);
        let (knots, degree, span, parameter) = if along_u {
            (&self.knots_u, pu, span_u, u)
        } else {
            (&self.knots_v, pv, span_v, v)
        };
        if degree == 0 {
            return Ok(Vec3::default());
        }
        let mut lowered = [0.0f64; MAX_STACK_ORDER];
        basis_functions_into(knots, degree - 1, span, parameter, &mut lowered);
        // The hodograph's homogeneous value, Σ over the (degree) differences of
        // the basis span times the other direction's full basis.
        let mut hodograph = zero;
        for r in 0..degree {
            // Difference r joins control indices (span − degree + r) and
            // (span − degree + r + 1); its lowered basis is N_{span−degree+r+1, degree−1}.
            let index = span - degree + r;
            let width = knots[index + degree + 1] - knots[index + 1];
            if width.abs() <= EPS {
                continue;
            }
            let factor = lowered[r] * degree as f64 / width;
            if along_u {
                for j in 0..=pv {
                    let difference = control(r + 1, j).add(control(r, j).scale(-1.0));
                    hodograph = hodograph.add(difference.scale(factor * full_v[j]));
                }
            } else {
                for i in 0..=pu {
                    let difference = control(i, r + 1).add(control(i, r).scale(-1.0));
                    hodograph = hodograph.add(difference.scale(factor * full_u[i]));
                }
            }
        }
        Ok(Vec3::new(hodograph.x, hodograph.y, hodograph.z)
            .sub(source.scale(hodograph.w))
            .scale(1.0 / point.w))
    }

    pub fn normal(&self, u: f64, v: f64) -> Result<Vec3, String> {
        let derivatives = self.derivatives(u, v, 1)?;
        derivatives[1][0].cross(derivatives[0][1]).normalized()
    }

    /// Principal curvatures (κ_min, κ_max) at (u, v), signed with respect to
    /// the parametrization normal n = Su × Sv / |Su × Sv| via the shape
    /// operator I⁻¹·II.  Convention check: a cylinder of radius R whose normal
    /// points AWAY from the axis has κ = −1/R along the circular direction, so
    /// an offset's regularity factor 1 − d·κ vanishes exactly when an inward
    /// offset (d = −R) reaches the axis.
    pub fn principal_curvatures(&self, u: f64, v: f64) -> Result<(f64, f64), String> {
        let derivatives = self.derivatives(u, v, 2)?;
        let su = derivatives[1][0];
        let sv = derivatives[0][1];
        let cross = su.cross(sv);
        let cross_length = cross.length();
        if cross_length <= 1e-12 {
            return Err(format!(
                "degenerate parametrization at (u={u:.4}, v={v:.4})"
            ));
        }
        let normal = cross.scale(1.0 / cross_length);
        let e1 = su.dot(su);
        let f1 = su.dot(sv);
        let g1 = sv.dot(sv);
        let l2 = derivatives[2][0].dot(normal);
        let m2 = derivatives[1][1].dot(normal);
        let n2 = derivatives[0][2].dot(normal);
        let denominator = e1 * g1 - f1 * f1;
        let mean_double = (l2 * g1 - 2.0 * m2 * f1 + n2 * e1) / denominator; // 2H
        let gauss = (l2 * n2 - m2 * m2) / denominator; // K
        let discriminant = (mean_double * mean_double * 0.25 - gauss).max(0.0).sqrt();
        Ok((
            mean_double * 0.5 - discriminant,
            mean_double * 0.5 + discriminant,
        ))
    }

    /// Whether this is a genuinely affine degree-1 by degree-1 patch.
    ///
    /// A tapered bilinear patch is planar but not affine, so degree alone is
    /// insufficient for exact inverse-parameter and integration shortcuts.
    pub fn is_affine(&self) -> Result<bool, String> {
        if self.degree_u != 1
            || self.degree_v != 1
            || self.control_points.len() != 2
            || self.control_points[0].len() != 2
            || self.control_points[1].len() != 2
        {
            return Ok(false);
        }
        let weight = self.control_points[0][0].w;
        if [
            self.control_points[0][1].w,
            self.control_points[1][0].w,
            self.control_points[1][1].w,
        ]
        .iter()
        .any(|other| (*other - weight).abs() > 1e-12 * other.abs().max(weight.abs()))
        {
            return Ok(false);
        }
        let p00 = self.control_points[0][0].point()?;
        let p01 = self.control_points[0][1].point()?;
        let p10 = self.control_points[1][0].point()?;
        let p11 = self.control_points[1][1].point()?;
        let gap = p11.sub(p10).sub(p01.sub(p00)).length();
        // The parallelogram defect measures the non-affine bilinear term.
        // Scale it by the patch extent, never its distance from the origin:
        // falsely accepting a translated warped patch also labels it a plane
        // and sends its pcurves and offsets through exact affine shortcuts.
        let extent = [
            p01.sub(p00),
            p10.sub(p00),
            p11.sub(p00),
            p10.sub(p01),
            p11.sub(p01),
            p11.sub(p10),
        ]
        .iter()
        .map(|delta| delta.length())
        .fold(0.0, f64::max);
        Ok(gap <= 1e-9 * (1.0 + extent))
    }

    pub fn iso_curve_u(&self, u: f64) -> Result<NurbsCurve, String> {
        self.ensure_valid()?;
        let u = knot_clamp(&self.knots_u, self.degree_u, u);
        let span = knot_find_span(&self.knots_u, self.degree_u, u);
        let basis = if self.degree_u <= MAX_STACK_DEGREE {
            let mut stack = [0.0f64; MAX_STACK_ORDER];
            basis_functions_into(&self.knots_u, self.degree_u, span, u, &mut stack);
            stack[..=self.degree_u].to_vec()
        } else {
            KnotVector::new(self.knots_u.clone(), self.degree_u)?.basis_functions(span, u)
        };
        let mut points = Vec::with_capacity(self.control_points[0].len());
        for column in 0..self.control_points[0].len() {
            let mut point = Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            };
            for (index, value) in basis.iter().enumerate() {
                point = point
                    .add(self.control_points[span - self.degree_u + index][column].scale(*value));
            }
            points.push(point);
        }
        NurbsCurve::new(self.degree_v, self.knots_v.clone(), points)
    }

    pub fn iso_curve_v(&self, v: f64) -> Result<NurbsCurve, String> {
        self.ensure_valid()?;
        let v = knot_clamp(&self.knots_v, self.degree_v, v);
        let span = knot_find_span(&self.knots_v, self.degree_v, v);
        let basis = if self.degree_v <= MAX_STACK_DEGREE {
            let mut stack = [0.0f64; MAX_STACK_ORDER];
            basis_functions_into(&self.knots_v, self.degree_v, span, v, &mut stack);
            stack[..=self.degree_v].to_vec()
        } else {
            KnotVector::new(self.knots_v.clone(), self.degree_v)?.basis_functions(span, v)
        };
        let mut points = Vec::with_capacity(self.control_points.len());
        for row in &self.control_points {
            let mut point = Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 0.0,
            };
            for (index, value) in basis.iter().enumerate() {
                point = point.add(row[span - self.degree_v + index].scale(*value));
            }
            points.push(point);
        }
        NurbsCurve::new(self.degree_u, self.knots_u.clone(), points)
    }
}

pub fn make_plane(
    origin: Vec3,
    u_direction: Vec3,
    v_direction: Vec3,
    u_max: f64,
    v_max: f64,
) -> Result<NurbsSurface, String> {
    if u_max <= EPS || v_max <= EPS {
        return Err("makePlane: parameter extents must be positive".into());
    }
    let p10 = origin.add(u_direction.scale(u_max));
    let p01 = origin.add(v_direction.scale(v_max));
    let p11 = p10.add(v_direction.scale(v_max));
    NurbsSurface::new(
        1,
        1,
        vec![0.0, 0.0, u_max, u_max],
        vec![0.0, 0.0, v_max, v_max],
        vec![
            vec![Vec4::from_point(origin, 1.0), Vec4::from_point(p01, 1.0)],
            vec![Vec4::from_point(p10, 1.0), Vec4::from_point(p11, 1.0)],
        ],
    )
}

pub fn make_extrusion(profile: &NurbsCurve, direction: Vec3) -> Result<NurbsSurface, String> {
    let rows = profile
        .control_points
        .iter()
        .map(|point| {
            vec![
                *point,
                Vec4 {
                    x: point.x + point.w * direction.x,
                    y: point.y + point.w * direction.y,
                    z: point.z + point.w * direction.z,
                    w: point.w,
                },
            ]
        })
        .collect();
    NurbsSurface::new(
        profile.degree,
        1,
        profile.knots.clone(),
        vec![0.0, 0.0, 1.0, 1.0],
        rows,
    )
}

pub fn make_revolution(
    axis_point: Vec3,
    axis_direction: Vec3,
    generatrix: &NurbsCurve,
    theta: f64,
) -> Result<NurbsSurface, String> {
    if theta <= EPS || theta > std::f64::consts::TAU + EPS {
        return Err("makeRevolution: theta must be in (0, 2*PI]".into());
    }
    let theta = theta.min(std::f64::consts::TAU);
    let axis = axis_direction.normalized()?;
    let arc_count = ((theta / std::f64::consts::FRAC_PI_2 - EPS).ceil() as usize).clamp(1, 4);
    let arc_angle = theta / arc_count as f64;
    let middle_weight = (arc_angle / 2.0).cos();
    let mut knots_u = vec![0.0, 0.0, 0.0];
    for index in 1..arc_count {
        let knot = index as f64 / arc_count as f64;
        knots_u.extend([knot, knot]);
    }
    knots_u.extend([1.0, 1.0, 1.0]);
    let mut rows = vec![
        vec![
            Vec4 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            };
            generatrix.control_points.len()
        ];
        2 * arc_count + 1
    ];

    // How far the generatrix reaches from the axis — the scale every "is this
    // control point on the axis?" question below is measured against.
    let mut radial_extent: f64 = 0.0;
    for control_point in &generatrix.control_points {
        let point = control_point.point()?;
        let offset = point.sub(axis_point);
        radial_extent = radial_extent.max(offset.sub(axis.scale(offset.dot(axis))).length());
    }

    for (column, control_point) in generatrix.control_points.iter().enumerate() {
        let point = control_point.point()?;
        let weight = control_point.w;
        let axis_origin = axis_point.add(axis.scale(point.sub(axis_point).dot(axis)));
        let radial_vector = point.sub(axis_origin);
        // On-axis classification must be RELATIVE like the fuzzy band below:
        // the absolute EPS floor alone broke when the importer's mm conversion
        // scaled vendor pole noise (1.5e-13 m -> 1.5e-10 mm) past it, hard-
        // rejecting files whose poles are intended on-axis. 1e-8 of the radial
        // extent snaps that noise to an EXACT pole (radius 0, so every
        // downstream `radius <= EPS` branch takes the degenerate path):
        // corpus vendor noise measures up to ~2e-9 relative, and the
        // 1e-6-relative nudge the fuzzy-pole test rejects stays two decades
        // above the snap.
        let measured_radius = radial_vector.length();
        let pole_tolerance = EPS.max(1e-8 * radial_extent);
        let radius = if measured_radius <= pole_tolerance {
            0.0
        } else {
            measured_radius
        };
        let (x_axis, y_axis) = if radius <= EPS {
            (Vec3::default(), Vec3::default())
        } else {
            // A control point is either ON the axis — the degenerate pole row
            // above, whose rotation is a fixed point — or clearly off it. In
            // between the revolution carries a fuzzy PINCH where a pole was
            // meant to be, which downstream code (analytic recognition, pair
            // classification, the imprint) reads as an ordinary tiny circle:
            // refuse it here instead. This refusal used to fall out of the
            // parallel-tangent guard below, whose ABSOLUTE floor made it
            // "radius <= 1e-3" — which on a metre-unit STEP file rejected every
            // healthy 1.524e-4 cylinder in the part. Relative to the
            // generatrix's own radial extent it catches the fuzzy pole at any
            // model scale and lets real geometry through at any size.
            if radius <= NEAR_AXIS_RELATIVE_TOLERANCE * radial_extent {
                return Err(format!(
                    "makeRevolution: generatrix control point {column} sits {radius:e} from the \
                     axis, neither on it nor clear of it against the generatrix's {radial_extent:e} \
                     radial extent (a fuzzy pole)"
                ));
            }
            let x_axis = radial_vector.scale(1.0 / radius);
            (x_axis, axis.cross(x_axis))
        };
        rows[0][column] = Vec4::from_point(point, weight);
        let mut start_point = point;
        let mut start_tangent = y_axis.scale(radius);
        let mut angle = 0.0;
        for arc in 1..=arc_count {
            angle += arc_angle;
            let end_point = if radius <= EPS {
                point
            } else {
                axis_origin
                    .add(x_axis.scale(radius * angle.cos()))
                    .add(y_axis.scale(radius * angle.sin()))
            };
            let end_tangent = if radius <= EPS {
                Vec3::default()
            } else {
                x_axis
                    .scale(-radius * angle.sin())
                    .add(y_axis.scale(radius * angle.cos()))
            };
            let row = 2 * (arc - 1);
            if radius <= EPS {
                rows[row + 1][column] = Vec4::from_point(point, weight * middle_weight);
            } else {
                let cross = start_tangent.cross(end_tangent);
                let denominator = cross.length_squared();
                // Both tangents have length `radius`, so `denominator` is
                // radius^4 * sin^2(arc_angle) and an ABSOLUTE floor here reads
                // as "radius <= 1e-3", conflating a small part with a
                // degenerate sweep. Parallel tangents are purely an ANGULAR
                // condition, so scale the floor by the tangent lengths: the
                // test becomes sin^2(arc_angle) <= EPS, exactly what make_arc
                // applies to its already-unit tangents. Near-axis control
                // points are the near-axis check's business, above.
                let tangent_scale = start_tangent.length_squared() * end_tangent.length_squared();
                if denominator <= EPS * tangent_scale {
                    return Err("makeRevolution: arc tangents are parallel".into());
                }
                let distance =
                    end_point.sub(start_point).cross(end_tangent).dot(cross) / denominator;
                let middle = start_point.add(start_tangent.scale(distance));
                rows[row + 1][column] = Vec4::from_point(middle, weight * middle_weight);
            }
            rows[row + 2][column] = Vec4::from_point(end_point, weight);
            start_point = end_point;
            start_tangent = end_tangent;
        }
    }
    NurbsSurface::new(
        2,
        generatrix.degree,
        knots_u,
        generatrix.knots.clone(),
        rows,
    )
}

pub fn make_cylinder_surface(
    base: Vec3,
    axis_direction: Vec3,
    radius: f64,
    height: f64,
) -> Result<NurbsSurface, String> {
    if radius <= EPS || height <= EPS {
        return Err("makeCylinder: radius and height must be positive".into());
    }
    let axis = axis_direction.normalized()?;
    let x_axis = axis.perpendicular()?;
    let start = base.add(x_axis.scale(radius));
    let end = start.add(axis.scale(height));
    let generatrix = crate::make_line(start, end)?;
    make_revolution(base, axis, &generatrix, std::f64::consts::TAU)
}

pub fn make_cone_surface(
    base: Vec3,
    axis_direction: Vec3,
    radius_bottom: f64,
    radius_top: f64,
    height: f64,
) -> Result<NurbsSurface, String> {
    if radius_bottom <= EPS || radius_top < 0.0 || height <= EPS {
        return Err("makeCone: invalid radius or height".into());
    }
    let axis = axis_direction.normalized()?;
    let x_axis = axis.perpendicular()?;
    let start = base.add(x_axis.scale(radius_bottom));
    let end = base.add(axis.scale(height)).add(x_axis.scale(radius_top));
    let generatrix = crate::make_line(start, end)?;
    make_revolution(base, axis, &generatrix, std::f64::consts::TAU)
}

pub fn make_sphere_surface(
    center: Vec3,
    radius: f64,
    polar_axis: Vec3,
) -> Result<NurbsSurface, String> {
    make_sphere_surface_framed(center, radius, polar_axis, None)
}

/// A sphere with BOTH halves of its frame chosen: the polar axis (where the two
/// degenerate poles sit) and the SEAM direction (where the `u = 0` meridian
/// runs). `seam_direction` is Gram-Schmidt'd against the axis; `None` — and a
/// seam parallel to the axis, which names no meridian — falls back to
/// [`Vec3::perpendicular`], i.e. exactly what [`make_sphere_surface`] builds.
///
/// A caller that KNOWS where the sphere will be trimmed uses this to park the
/// poles and the seam inside the region the trim removes, so the surviving face
/// carries neither. `feature_pipeline::features::tube` does that for its joint
/// balls: a seam that survives into the result is an edge bordering ONE face,
/// which is a real edge in the topology and a visible line on the model.
pub fn make_sphere_surface_framed(
    center: Vec3,
    radius: f64,
    polar_axis: Vec3,
    seam_direction: Option<Vec3>,
) -> Result<NurbsSurface, String> {
    if radius <= EPS {
        return Err("makeSphere: radius must be positive".into());
    }
    let axis = polar_axis.normalized()?;
    let x_axis = match seam_direction {
        Some(seam) => {
            let planar = seam.sub(axis.scale(seam.dot(axis)));
            match planar.normalized() {
                Ok(unit) if planar.length() > EPS => unit,
                _ => axis.perpendicular()?,
            }
        }
        None => axis.perpendicular()?,
    };
    let meridian = crate::make_arc(
        center,
        x_axis,
        axis,
        radius,
        -std::f64::consts::FRAC_PI_2,
        std::f64::consts::FRAC_PI_2,
    )?;
    make_revolution(center, axis, &meridian, std::f64::consts::TAU)
}

pub fn make_torus_surface(
    center: Vec3,
    axis_direction: Vec3,
    major_radius: f64,
    minor_radius: f64,
) -> Result<NurbsSurface, String> {
    if minor_radius <= EPS || major_radius <= minor_radius {
        return Err("makeTorus: requires positive minorRadius < majorRadius".into());
    }
    let axis = axis_direction.normalized()?;
    let x_axis = axis.perpendicular()?;
    let tube_center = center.add(x_axis.scale(major_radius));
    let tube = crate::make_arc(
        tube_center,
        x_axis,
        axis,
        minor_radius,
        0.0,
        std::f64::consts::TAU,
    )?;
    make_revolution(center, axis, &tube, std::f64::consts::TAU)
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



/// Diagnostic "carrier preview" patch (§3.15 applied to display): re-express
/// the surface over an INFLATED domain so an inspector can show where the
/// carrier continues beyond the face's trim. Open directions inflate about
/// the domain centre by `inflate` (a factor; 1 = unchanged), evaluated
/// through `evaluate_extended` — exact linear extension for affine carriers,
/// ruled/tangent extension generally. CLOSED (periodic) directions keep the
/// stored full period instead of inflating — wrapping further would overlap
/// the surface onto itself. A torus (closed both ways) returns unchanged.
///
/// Affine carriers rebuild EXACTLY from four extended corners; everything
/// else is a preview-quality degree-3 interpolation over a 33-sample grid.
pub fn carrier_preview_patch(surface: &NurbsSurface, inflate: f64) -> Result<NurbsSurface, String> {
    let inflate = if inflate.is_finite() {
        inflate.clamp(1.0, 16.0)
    } else {
        2.0
    };
    let (closed_u, closed_v) = surface.closed_directions()?;
    let [u0, u1] = surface.domain_u()?;
    let [v0, v1] = surface.domain_v()?;
    let stretch = |d0: f64, d1: f64, closed: bool| -> (f64, f64) {
        if closed || inflate <= 1.0 {
            (d0, d1)
        } else {
            let centre = 0.5 * (d0 + d1);
            let half = 0.5 * (d1 - d0) * inflate;
            (centre - half, centre + half)
        }
    };
    let (nu0, nu1) = stretch(u0, u1, closed_u);
    let (nv0, nv1) = stretch(v0, v1, closed_v);
    if (nu0, nu1) == (u0, u1) && (nv0, nv1) == (v0, v1) {
        return Ok(surface.clone());
    }
    if surface.is_affine()? {
        // Exact: the linear extension of an affine patch is the same affine
        // map over the larger rectangle.
        let corners = [
            surface.evaluate_extended(nu0, nv0)?,
            surface.evaluate_extended(nu1, nv0)?,
            surface.evaluate_extended(nu0, nv1)?,
            surface.evaluate_extended(nu1, nv1)?,
        ];
        return NurbsSurface::new(
            1,
            1,
            vec![nu0, nu0, nu1, nu1],
            vec![nv0, nv0, nv1, nv1],
            vec![
                vec![
                    Vec4::from_point(corners[0], 1.0),
                    Vec4::from_point(corners[2], 1.0),
                ],
                vec![
                    Vec4::from_point(corners[1], 1.0),
                    Vec4::from_point(corners[3], 1.0),
                ],
            ],
        );
    }
    const SAMPLES: usize = 32;
    let u_params: Vec<f64> = (0..=SAMPLES)
        .map(|index| nu0 + (nu1 - nu0) * index as f64 / SAMPLES as f64)
        .collect();
    let v_params: Vec<f64> = (0..=SAMPLES)
        .map(|index| nv0 + (nv1 - nv0) * index as f64 / SAMPLES as f64)
        .collect();
    // Two-pass grid interpolation: fit each v-column, then fit the resulting
    // control rows across u, sharing knots per pass (identical parameters).
    let mut column_curves = Vec::with_capacity(u_params.len());
    for &u in &u_params {
        let column: Vec<Vec3> = v_params
            .iter()
            .map(|&v| surface.evaluate_extended(u, v))
            .collect::<Result<_, _>>()?;
        column_curves.push(crate::interpolate_curve(
            &column,
            3,
            &normalized(&v_params),
        )?);
    }
    let v_knots = column_curves[0].knots.clone();
    let control_rows = column_curves[0].control_points.len();
    let mut grid: Vec<Vec<Vec4>> = Vec::with_capacity(u_params.len());
    let mut u_knots = Vec::new();
    for row_index in 0..control_rows {
        let row: Vec<Vec3> = column_curves
            .iter()
            .map(|curve| curve.control_points[row_index].point())
            .collect::<Result<_, _>>()?;
        let fitted = crate::interpolate_curve(&row, 3, &normalized(&u_params))?;
        u_knots = fitted.knots.clone();
        for (u_index, control) in fitted.control_points.iter().enumerate() {
            if grid.len() <= u_index {
                grid.push(Vec::new());
            }
            grid[u_index].push(*control);
        }
    }
    // Re-scale the normalized fit knots back onto the inflated domains so the
    // preview's parameter space lines up with the original surface's.
    let rescale = |knots: &[f64], d0: f64, d1: f64| -> Vec<f64> {
        knots.iter().map(|k| d0 + (d1 - d0) * k).collect()
    };
    NurbsSurface::new(
        3,
        3,
        rescale(&u_knots, nu0, nu1),
        rescale(&v_knots, nv0, nv1),
        grid,
    )
}

fn normalized(parameters: &[f64]) -> Vec<f64> {
    let first = parameters[0];
    let last = parameters[parameters.len() - 1];
    parameters
        .iter()
        .map(|p| (p - first) / (last - first))
        .collect()
}


