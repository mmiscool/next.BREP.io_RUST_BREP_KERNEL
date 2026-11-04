use serde::{Deserialize, Serialize};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

use crate::numerical::{linear_algebra, scalar};

#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize, Serialize)]
/// A three-dimensional vector with double-precision components.
pub struct Vec3 {
    /// The x component.
    pub x: f64,
    /// The y component.
    pub y: f64,
    /// The z component.
    pub z: f64,
}

impl Vec3 {
    /// The zero vector.
    pub const ZERO: Self = Self::new(0.0, 0.0, 0.0);
    /// The positive x-axis unit vector.
    pub const X: Self = Self::new(1.0, 0.0, 0.0);
    /// The positive y-axis unit vector.
    pub const Y: Self = Self::new(0.0, 1.0, 0.0);
    /// The positive z-axis unit vector.
    pub const Z: Self = Self::new(0.0, 0.0, 1.0);

    /// Creates a vector from Cartesian components.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    /// Returns the dot product with `rhs`.
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    /// Returns the right-handed cross product with `rhs`.
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }
    /// Returns the squared Euclidean length.
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }
    /// Returns the Euclidean length.
    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }
    /// Returns a unit vector in the same direction, or `None` if the vector
    /// is zero, too small to normalize reliably, or non-finite.
    pub fn normalized(self) -> Option<Self> {
        let length = self.length();
        (length.is_finite() && length > scalar::MIN_NORMALIZABLE_NORM).then(|| self / length)
    }
    /// Returns the Euclidean distance to `rhs`.
    pub fn distance(self, rhs: Self) -> f64 {
        (self - rhs).length()
    }
    /// Returns whether all components are finite.
    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }
    /// Returns the component at index 0, 1, or 2.
    ///
    /// # Panics
    ///
    /// Panics when `index` is greater than 2.
    pub fn component(self, index: usize) -> f64 {
        [self.x, self.y, self.z][index]
    }
    /// Chooses a deterministic sign by making the largest-magnitude component
    /// non-negative.
    pub fn canonicalized(self) -> Self {
        let values = [self.x, self.y, self.z];
        let index = (0..3)
            .max_by(|&a, &b| values[a].abs().total_cmp(&values[b].abs()))
            .unwrap();
        if values[index] < 0.0 {
            -self
        } else {
            self
        }
    }
    /// Builds two unit vectors perpendicular to this vector and each other.
    ///
    /// Returns `None` when this vector cannot be normalized.
    pub fn orthonormal_basis(self) -> Option<(Self, Self)> {
        let n = self.normalized()?;
        let seed = if n.x.abs() <= n.y.abs() && n.x.abs() <= n.z.abs() {
            Self::X
        } else if n.y.abs() <= n.z.abs() {
            Self::Y
        } else {
            Self::Z
        };
        let u = n.cross(seed).normalized()?;
        Some((u, n.cross(u)))
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, r: Self) -> Self {
        Self::new(self.x + r.x, self.y + r.y, self.z + r.z)
    }
}
impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, r: Self) -> Self {
        Self::new(self.x - r.x, self.y - r.y, self.z - r.z)
    }
}
impl Mul<f64> for Vec3 {
    type Output = Self;
    fn mul(self, s: f64) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }
}
impl Div<f64> for Vec3 {
    type Output = Self;
    fn div(self, s: f64) -> Self {
        self * (1.0 / s)
    }
}
impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        self * -1.0
    }
}
impl AddAssign for Vec3 {
    fn add_assign(&mut self, r: Self) {
        *self = *self + r;
    }
}
impl SubAssign for Vec3 {
    fn sub_assign(&mut self, r: Self) {
        *self = *self - r;
    }
}

pub(crate) fn outer_accumulate(matrix: &mut [[f64; 3]; 3], value: Vec3, weight: f64) {
    let v = [value.x, value.y, value.z];
    for i in 0..3 {
        for j in 0..3 {
            matrix[i][j] += weight * v[i] * v[j];
        }
    }
}

/// How [`jacobi_eigen_symmetric3`] turns a pivot pair with equal diagonals
/// (`tau` = ±0, where `(1 + tau²)` gives `t = ±1`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JacobiZeroTurn {
    /// `t = tau.signum()`: `-0.0` (equal diagonals, negative off-diagonal)
    /// turns by `-1`. The RANSAC fits' convention.
    Signed,
    /// `t = +1` at either zero. The kernel carrier fits' and the
    /// reconstruction copies' convention (`theta >= 0` / `theta == 0`).
    Positive,
}

/// Dependency-free Jacobi eigensolver for real symmetric 3x3 matrices.
///
/// Classic Jacobi (largest off-diagonal first, at most 64 rotations, stop at
/// `1e-15 * max|a|`). Returns the eigenvalues in ascending order and, for
/// each, its eigenvector as `[x, y, z]`: the accumulated rotation's column,
/// unit to rounding but NOT renormalised. The one eigensolver behind the
/// RANSAC fits (`eigen_symmetric3` renormalises it), the kernel's mesh
/// segmentation carrier fits and the reconstruction plane/axis reads.
///
/// `zero_turn` names the one place the former copies differed: a rotation
/// whose pivot pair has equal diagonals (`tau` = ±0). Every other `tau`
/// gives the same turn bit for bit under either convention.
pub fn jacobi_eigen_symmetric3(
    mut a: [[f64; 3]; 3],
    zero_turn: JacobiZeroTurn,
) -> ([f64; 3], [[f64; 3]; 3]) {
    let mut v = [[0.0; 3]; 3];
    for (i, row) in v.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    let scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |m, x| m.max(x.abs()))
        .max(linear_algebra::MATRIX_SCALE_FLOOR);
    for _ in 0..64 {
        let mut pair = (0, 1);
        for candidate in [(0, 2), (1, 2)] {
            if a[candidate.0][candidate.1].abs() > a[pair.0][pair.1].abs() {
                pair = candidate;
            }
        }
        let (p, q) = pair;
        if a[p][q].abs() <= linear_algebra::JACOBI_RELATIVE_CONVERGENCE * scale {
            break;
        }
        let tau = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
        let t = if tau == 0.0 && zero_turn == JacobiZeroTurn::Positive {
            1.0
        } else {
            tau.signum() / (tau.abs() + (1.0 + tau * tau).sqrt())
        };
        let c = 1.0 / (1.0 + t * t).sqrt();
        let s = t * c;
        let apq = a[p][q];
        a[p][p] -= t * apq;
        a[q][q] += t * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;
        for k in 0..3 {
            if k != p && k != q {
                let (akp, akq) = (a[k][p], a[k][q]);
                a[k][p] = c * akp - s * akq;
                a[p][k] = a[k][p];
                a[k][q] = s * akp + c * akq;
                a[q][k] = a[k][q];
            }
            let (vkp, vkq) = (v[k][p], v[k][q]);
            v[k][p] = c * vkp - s * vkq;
            v[k][q] = s * vkp + c * vkq;
        }
    }
    let mut order = [0, 1, 2];
    order.sort_by(|&i, &j| a[i][i].total_cmp(&a[j][j]));
    let values = order.map(|i| a[i][i]);
    let vectors = order.map(|i| [v[0][i], v[1][i], v[2][i]]);
    (values, vectors)
}

/// [`jacobi_eigen_symmetric3`] with each eigenvector renormalised.
pub(crate) fn eigen_symmetric3(a: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
    let (values, vectors) = jacobi_eigen_symmetric3(a, JacobiZeroTurn::Signed);
    let vectors = vectors.map(|[x, y, z]| Vec3::new(x, y, z).normalized().unwrap_or(Vec3::ZERO));
    (values, vectors)
}

/// Iteration and acceptance policy of [`refine_cone`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConeRefinePolicy {
    /// Gauss-Newton steps taken at most.
    pub steps: usize,
    /// Stop once a step's norm falls below this (`0.0`: always take `steps`).
    pub stop_below: f64,
    /// A point at most this far from the axis takes no part in a step.
    pub min_radius: f64,
    /// Clamp the half angle into `(1e-6, pi/2 - 1e-6)` after every step;
    /// `false` leaves it to the caller's own range test.
    pub clamp_angle: bool,
}

/// Gauss-Newton refinement of a cone over a point slice: apex, unit axis
/// into the nappe, and half angle, with the six parameters apex (3), two
/// axis tilts along an orthonormal frame of the axis, and the angle. The
/// residual of a point is `rho cos(a) - h sin(a)` (h along the axis, rho
/// from it). Each step solves the normal equations with `solve_linear`.
/// `None` when the axis cannot be normalised, a step is singular, or the
/// result is not finite.
pub fn refine_cone(
    points: &[[f64; 3]],
    apex: [f64; 3],
    axis: [f64; 3],
    half_angle: f64,
    policy: ConeRefinePolicy,
) -> Option<([f64; 3], [f64; 3], f64)> {
    let mut apex = Vec3::new(apex[0], apex[1], apex[2]);
    let mut axis = Vec3::new(axis[0], axis[1], axis[2]).normalized()?;
    let mut half_angle = half_angle;
    for _ in 0..policy.steps {
        let (e1, e2) = axis.orthonormal_basis()?;
        let (sin_a, cos_a) = half_angle.sin_cos();
        let mut ata = vec![vec![0.0_f64; 6]; 6];
        let mut atb = vec![0.0_f64; 6];
        for p in points {
            let d = Vec3::new(p[0], p[1], p[2]) - apex;
            let h = d.dot(axis);
            let radial = d - axis * h;
            let rho = radial.length();
            if rho <= policy.min_radius {
                continue;
            }
            let w = radial / rho;
            let residual = rho * cos_a - h * sin_a;
            let j_apex = axis * sin_a - w * cos_a;
            let c_tilt = -((h / rho) * cos_a + sin_a);
            let row = [
                j_apex.x,
                j_apex.y,
                j_apex.z,
                c_tilt * d.dot(e1),
                c_tilt * d.dot(e2),
                -rho * sin_a - h * cos_a,
            ];
            for i in 0..6 {
                for j in 0..6 {
                    ata[i][j] += row[i] * row[j];
                }
                atb[i] -= row[i] * residual;
            }
        }
        let step = solve_linear(ata, atb)?;
        apex = apex + Vec3::new(step[0], step[1], step[2]);
        axis = (axis + e1 * step[3] + e2 * step[4]).normalized()?;
        half_angle += step[5];
        if policy.clamp_angle {
            half_angle = half_angle.clamp(1e-6, std::f64::consts::FRAC_PI_2 - 1e-6);
        }
        let step_size = step.iter().map(|s| s * s).sum::<f64>().sqrt();
        if step_size < policy.stop_below {
            break;
        }
    }
    (apex.is_finite() && axis.is_finite() && half_angle.is_finite())
        .then_some(([apex.x, apex.y, apex.z], [axis.x, axis.y, axis.z], half_angle))
}

pub(crate) fn solve_linear(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    if a.len() != n || a.iter().any(|r| r.len() != n) {
        return None;
    }
    let scale = a
        .iter()
        .flatten()
        .fold(0.0_f64, |m, x| m.max(x.abs()))
        .max(1.0);
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))?;
        if a[pivot][col].abs() <= linear_algebra::LINEAR_PIVOT_RELATIVE_MIN * scale {
            return None;
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        let d = a[col][col];
        for j in col..n {
            a[col][j] /= d;
        }
        b[col] /= d;
        for i in 0..n {
            if i != col {
                let f = a[i][col];
                for j in col..n {
                    a[i][j] -= f * a[col][j];
                }
                b[i] -= f * b[col];
            }
        }
    }
    b.iter().all(|x| x.is_finite()).then_some(b)
}

/// Rows may be `Vec<f64>` or fixed arrays (`[f64; 3]` saves the hot torus and
/// circle initializers one allocation per sample); the arithmetic is the same.
pub(crate) fn least_squares<R: AsRef<[f64]>>(
    rows: &[R],
    rhs: &[f64],
    regularization: f64,
) -> Option<Vec<f64>> {
    let n = rows.first()?.as_ref().len();
    if rows.len() != rhs.len() || rows.iter().any(|r| r.as_ref().len() != n) {
        return None;
    }
    let mut ata = vec![vec![0.0; n]; n];
    let mut atb = vec![0.0; n];
    for (row, &value) in rows.iter().zip(rhs) {
        let row = row.as_ref();
        for i in 0..n {
            atb[i] += row[i] * value;
            for j in 0..n {
                ata[i][j] += row[i] * row[j];
            }
        }
    }
    for (i, row) in ata.iter_mut().enumerate() {
        row[i] += regularization;
    }
    solve_linear(ata, atb)
}

/// Solve a tall least-squares system with column-pivoted, reorthogonalized QR.
///
/// Unlike [`least_squares`], this does not form the normal equations. It is
/// intended for small parameter counts whose observation matrix can be badly
/// conditioned but still has observable full rank, such as tangent planes on
/// a narrowly trimmed cone.
pub(crate) fn least_squares_qr<R: AsRef<[f64]>>(
    rows: &[R],
    rhs: &[f64],
    relative_rank_min: f64,
) -> Option<Vec<f64>> {
    let parameter_count = rows.first()?.as_ref().len();
    if parameter_count == 0
        || rows.len() < parameter_count
        || rows.len() != rhs.len()
        || !relative_rank_min.is_finite()
        || relative_rank_min < 0.0
        || rows.iter().any(|row| {
            let row = row.as_ref();
            row.len() != parameter_count || row.iter().any(|value| !value.is_finite())
        })
        || rhs.iter().any(|value| !value.is_finite())
    {
        return None;
    }

    let mut columns: Vec<Vec<f64>> = (0..parameter_count)
        .map(|column| rows.iter().map(|row| row.as_ref()[column]).collect())
        .collect();
    let mut residual_rhs = rhs.to_vec();
    let mut permutation: Vec<usize> = (0..parameter_count).collect();
    let mut upper = vec![vec![0.0; parameter_count]; parameter_count];
    let mut projected_rhs = vec![0.0; parameter_count];
    let squared_norm = |values: &[f64]| values.iter().map(|value| value * value).sum::<f64>();
    let reference_norm = columns
        .iter()
        .map(|column| squared_norm(column).sqrt())
        .fold(0.0_f64, f64::max);
    if !reference_norm.is_finite() || reference_norm <= 0.0 {
        return None;
    }

    for rank in 0..parameter_count {
        let pivot = (rank..parameter_count).max_by(|&left, &right| {
            squared_norm(&columns[left]).total_cmp(&squared_norm(&columns[right]))
        })?;
        if pivot != rank {
            columns.swap(rank, pivot);
            permutation.swap(rank, pivot);
            for row in upper.iter_mut().take(rank) {
                row.swap(rank, pivot);
            }
        }
        let norm = squared_norm(&columns[rank]).sqrt();
        if !norm.is_finite() || norm <= relative_rank_min * reference_norm {
            return None;
        }
        upper[rank][rank] = norm;
        let direction: Vec<_> = columns[rank].iter().map(|value| value / norm).collect();
        projected_rhs[rank] = direction
            .iter()
            .zip(&residual_rhs)
            .map(|(left, right)| left * right)
            .sum();
        for (value, direction_value) in residual_rhs.iter_mut().zip(&direction) {
            *value -= direction_value * projected_rhs[rank];
        }
        for (column, values) in columns.iter_mut().enumerate().skip(rank + 1) {
            // A second modified Gram-Schmidt pass retains substantially more
            // of the small singular direction on narrow normal fans.
            let mut projection = direction
                .iter()
                .zip(&*values)
                .map(|(left, right)| left * right)
                .sum::<f64>();
            for (value, direction_value) in values.iter_mut().zip(&direction) {
                *value -= direction_value * projection;
            }
            let correction = direction
                .iter()
                .zip(&*values)
                .map(|(left, right)| left * right)
                .sum::<f64>();
            for (value, direction_value) in values.iter_mut().zip(&direction) {
                *value -= direction_value * correction;
            }
            projection += correction;
            upper[rank][column] = projection;
        }
    }

    let mut pivoted_solution = vec![0.0; parameter_count];
    for row in (0..parameter_count).rev() {
        let remainder = (row + 1..parameter_count)
            .map(|column| upper[row][column] * pivoted_solution[column])
            .sum::<f64>();
        pivoted_solution[row] = (projected_rhs[row] - remainder) / upper[row][row];
    }
    if pivoted_solution.iter().any(|value| !value.is_finite()) {
        return None;
    }
    let mut solution = vec![0.0; parameter_count];
    for (pivoted, original) in permutation.into_iter().enumerate() {
        solution[original] = pivoted_solution[pivoted];
    }
    Some(solution)
}

