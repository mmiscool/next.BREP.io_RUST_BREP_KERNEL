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

/// Dependency-free Jacobi eigensolver for real symmetric 3x3 matrices.
pub(crate) fn eigen_symmetric3(mut a: [[f64; 3]; 3]) -> ([f64; 3], [Vec3; 3]) {
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
        let t = tau.signum() / (tau.abs() + (1.0 + tau * tau).sqrt());
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
    let vectors = order.map(|i| {
        Vec3::new(v[0][i], v[1][i], v[2][i])
            .normalized()
            .unwrap_or(Vec3::ZERO)
    });
    (values, vectors)
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

pub(crate) fn least_squares(
    rows: &[Vec<f64>],
    rhs: &[f64],
    regularization: f64,
) -> Option<Vec<f64>> {
    let n = rows.first()?.len();
    if rows.len() != rhs.len() || rows.iter().any(|r| r.len() != n) {
        return None;
    }
    let mut ata = vec![vec![0.0; n]; n];
    let mut atb = vec![0.0; n];
    for (row, &value) in rows.iter().zip(rhs) {
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
pub(crate) fn least_squares_qr(
    rows: &[Vec<f64>],
    rhs: &[f64],
    relative_rank_min: f64,
) -> Option<Vec<f64>> {
    let parameter_count = rows.first()?.len();
    if parameter_count == 0
        || rows.len() < parameter_count
        || rows.len() != rhs.len()
        || !relative_rank_min.is_finite()
        || relative_rank_min < 0.0
        || rows
            .iter()
            .any(|row| row.len() != parameter_count || row.iter().any(|value| !value.is_finite()))
        || rhs.iter().any(|value| !value.is_finite())
    {
        return None;
    }

    let mut columns: Vec<Vec<f64>> = (0..parameter_count)
        .map(|column| rows.iter().map(|row| row[column]).collect())
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

