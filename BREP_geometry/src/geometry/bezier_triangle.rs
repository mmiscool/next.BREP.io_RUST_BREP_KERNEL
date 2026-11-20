//! Triangular (rational) Bézier surfaces (Golovanov 2020 §3.11) and their
//! exact conversion to tensor-product NURBS patches.
//!
//! A degree-`n` patch has `(n+1)(n+2)/2` homogeneous control points
//! `P_ijk = (w·p, w)`, `i + j + k = n`, and is evaluated at barycentric
//! coordinates `(a, b, c)` by de Casteljau on the homogeneous points
//! (3.11.7), then divided by the weight (3.11.9). Corner `P_n00` is at
//! `(1, 0, 0)`, `P_0n0` at `(0, 1, 0)`, `P_00n` at `(0, 0, 1)`.
//!
//! The patch is an input form, not a face carrier: [`BezierTriangle::to_nurbs_quads`]
//! re-expresses it exactly as three regular bidegree `(n, n)` patches (split
//! at the centroid and edge midpoints, no collapsed side), and
//! [`BezierTriangle::to_nurbs_collapsed`] as one patch with side `s = 0`
//! collapsed to `P_n00`. Both use the polar form (blossom), so the tensor
//! patches are the same rational function, not a fit.

use crate::{NurbsCurve, NurbsSurface, Vec3, Vec4};

/// Highest supported degree. The blossom and the binomial weights of the
/// conversion stay well inside f64 range far beyond this; the bound keeps a
/// malformed request from allocating a huge net.
pub const MAX_TRIANGLE_DEGREE: usize = 24;

/// Smallest admitted ratio of the least to the largest weight. Every point of
/// the converted tensor nets is a convex combination of the control points'
/// homogeneous values, so their weights stay at or above this ratio too.
pub const MIN_WEIGHT_RATIO: f64 = 1e-9;

/// Largest admitted Cartesian coordinate magnitude of a control point.
pub const MAX_TRIANGLE_COORDINATE: f64 = 1e100;

/// Why a triangle patch or one of its operations was refused.
#[derive(Clone, Debug, PartialEq)]
pub enum BezierTriangleRefusal {
    /// Degree zero or above [`MAX_TRIANGLE_DEGREE`].
    Degree { degree: usize },
    /// The control net does not have `(n+1)(n+2)/2` points.
    PointCount { degree: usize, expected: usize, found: usize },
    /// A control point or weight that is not finite.
    NonFinite { index: usize },
    /// A control point farther than [`MAX_TRIANGLE_COORDINATE`] from the origin: the
    /// derivative arithmetic would overflow.
    Magnitude { index: usize },
    /// A weight that is not positive: the rational patch would have a pole or
    /// change sign inside the domain.
    Weight { index: usize, weight: f64 },
    /// The smallest weight is below [`MIN_WEIGHT_RATIO`] of the largest: the
    /// converted tensor net could not hold it.
    WeightRatio { min: f64, max: f64 },
    /// The homogeneous weight vanishes or is negative at a point outside the
    /// triangle (inside it every weight is a positive convex combination).
    PoleOutside { a: f64, b: f64, c: f64, weight: f64 },
    /// Barycentric coordinates that are not finite or do not sum to one.
    Barycentric { a: f64, b: f64, c: f64 },
    /// A direction whose barycentric components do not sum to zero.
    Direction { da: f64, db: f64, dc: f64 },
    /// The tensor patch builder refused the converted net.
    Conversion { reason: String },
}

impl std::fmt::Display for BezierTriangleRefusal {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Degree { degree } => write!(
                formatter,
                "bezier triangle: degree {degree} is outside 1..={MAX_TRIANGLE_DEGREE}"
            ),
            Self::PointCount { degree, expected, found } => write!(
                formatter,
                "bezier triangle: degree {degree} needs {expected} control points, found {found}"
            ),
            Self::Magnitude { index } => write!(
                formatter,
                "bezier triangle: control point {index} lies beyond {MAX_TRIANGLE_COORDINATE} from the origin"
            ),
            Self::NonFinite { index } => {
                write!(formatter, "bezier triangle: control point {index} is not finite")
            }
            Self::WeightRatio { min, max } => write!(
                formatter,
                "bezier triangle: weights span {min}..{max}; the smallest must be at least {MIN_WEIGHT_RATIO} of the largest"
            ),
            Self::PoleOutside { a, b, c, weight } => write!(
                formatter,
                "bezier triangle: the rational patch has weight {weight} at ({a}, {b}, {c}) outside the triangle"
            ),
            Self::Weight { index, weight } => write!(
                formatter,
                "bezier triangle: control point {index} has weight {weight}; weights must be positive"
            ),
            Self::Barycentric { a, b, c } => write!(
                formatter,
                "bezier triangle: ({a}, {b}, {c}) are not barycentric coordinates (finite, summing to one)"
            ),
            Self::Direction { da, db, dc } => write!(
                formatter,
                "bezier triangle: direction ({da}, {db}, {dc}) does not sum to zero"
            ),
            Self::Conversion { reason } => {
                write!(formatter, "bezier triangle: tensor conversion refused: {reason}")
            }
        }
    }
}

impl std::error::Error for BezierTriangleRefusal {}

/// Which side of the triangle: the side opposite the named corner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TriangleSide {
    /// `a = 0`: from `P_0n0` (t = 0) to `P_00n` (t = 1).
    OppositeA,
    /// `b = 0`: from `P_00n` (t = 0) to `P_n00` (t = 1).
    OppositeB,
    /// `c = 0`: from `P_n00` (t = 0) to `P_0n0` (t = 1).
    OppositeC,
}

/// One tensor patch of a conversion and the bilinear map from its `(s, t)`
/// unit square into the triangle: barycentric corners `[Q00, Q10, Q01, Q11]`,
/// `φ(s, t) = (1−s)(1−t)Q00 + s(1−t)Q10 + (1−s)t Q01 + st Q11`.
#[derive(Clone, Debug)]
pub struct TrianglePiece {
    pub surface: NurbsSurface,
    pub corners: [[f64; 3]; 4],
}

impl TrianglePiece {
    /// The barycentric point of the triangle that `(s, t)` of this piece maps to.
    pub fn barycentric(&self, s: f64, t: f64) -> [f64; 3] {
        bilinear(&self.corners, s, t)
    }
}

/// A (rational) triangular Bézier patch.
#[derive(Clone, Debug)]
pub struct BezierTriangle {
    degree: usize,
    /// Homogeneous points in [`BezierTriangle::index`] order.
    points: Vec<Vec4>,
}

const BARYCENTRIC_SUM: f64 = 1e-12;

impl BezierTriangle {
    /// Number of control points of a degree-`n` patch.
    pub fn point_count(degree: usize) -> usize {
        (degree + 1) * (degree + 2) / 2
    }

    /// Storage index of `P_ijk` (`i = n − j − k`): rows of constant `k`, `j`
    /// increasing inside a row.
    pub fn index(degree: usize, j: usize, k: usize) -> usize {
        debug_assert!(j + k <= degree);
        k * (degree + 1) - k * k.saturating_sub(1) / 2 + j
    }

    /// A patch from homogeneous points `(w·p, w)` in [`BezierTriangle::index`] order.
    pub fn new(degree: usize, points: Vec<Vec4>) -> Result<Self, BezierTriangleRefusal> {
        if degree == 0 || degree > MAX_TRIANGLE_DEGREE {
            return Err(BezierTriangleRefusal::Degree { degree });
        }
        let expected = Self::point_count(degree);
        if points.len() != expected {
            return Err(BezierTriangleRefusal::PointCount { degree, expected, found: points.len() });
        }
        for (index, point) in points.iter().enumerate() {
            if !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite() && point.w.is_finite()) {
                return Err(BezierTriangleRefusal::NonFinite { index });
            }
            if !(point.w > 0.0) {
                return Err(BezierTriangleRefusal::Weight { index, weight: point.w });
            }
            if [point.x, point.y, point.z].iter().any(|value| !((value / point.w).abs() <= MAX_TRIANGLE_COORDINATE)) {
                return Err(BezierTriangleRefusal::Magnitude { index });
            }
        }
        // Normalise the homogeneous scale (geometry unchanged) so the largest
        // weight is one: the tensor constructors bound weights absolutely.
        let max = points.iter().map(|point| point.w).fold(0.0, f64::max);
        let min = points.iter().map(|point| point.w).fold(f64::INFINITY, f64::min);
        if min < MIN_WEIGHT_RATIO * max {
            return Err(BezierTriangleRefusal::WeightRatio { min, max });
        }
        // Divide (not multiply by 1/max, which overflows for a subnormal max).
        let points: Vec<Vec4> = points
            .into_iter()
            .map(|point| Vec4 { x: point.x / max, y: point.y / max, z: point.z / max, w: point.w / max })
            .collect();
        if let Some(index) = points.iter().position(|point| !(point.x.is_finite() && point.y.is_finite() && point.z.is_finite())) {
            return Err(BezierTriangleRefusal::NonFinite { index });
        }
        Ok(Self { degree, points })
    }

    /// A polynomial patch (all weights one) from Cartesian points.
    pub fn from_points(degree: usize, points: &[Vec3]) -> Result<Self, BezierTriangleRefusal> {
        Self::new(degree, points.iter().map(|point| Vec4::from_point(*point, 1.0)).collect())
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Homogeneous control points in [`BezierTriangle::index`] order.
    pub fn points(&self) -> &[Vec4] {
        &self.points
    }

    /// Homogeneous control point `P_ijk`, `i = n − j − k`.
    pub fn control(&self, j: usize, k: usize) -> Vec4 {
        self.points[Self::index(self.degree, j, k)]
    }

    /// The homogeneous polar form `R(x_1, …, x_n)`: de Casteljau with argument
    /// `x_m` at level `m`. Symmetric and linear in each argument (arguments
    /// are barycentric triples; points sum to one, directions to zero).
    /// `args.len()` must equal the degree.
    pub fn blossom_homogeneous(&self, args: &[[f64; 3]]) -> Vec4 {
        assert_eq!(args.len(), self.degree, "blossom needs one argument per degree");
        let n = self.degree;
        let mut level = self.points.clone();
        for (m, arg) in args.iter().enumerate() {
            let size = n - m - 1;
            let mut next = Vec::with_capacity(Self::point_count(size));
            for k in 0..=size {
                for j in 0..=size - k {
                    // Points of the previous level (degree size + 1):
                    // i+1 (same j, k), j+1, k+1.
                    let pa = level[Self::index(size + 1, j, k)];
                    let pb = level[Self::index(size + 1, j + 1, k)];
                    let pc = level[Self::index(size + 1, j, k + 1)];
                    next.push(pa.scale(arg[0]).add(pb.scale(arg[1])).add(pc.scale(arg[2])));
                }
            }
            level = next;
        }
        level[0]
    }

    fn check_barycentric(a: f64, b: f64, c: f64) -> Result<(), BezierTriangleRefusal> {
        if !(a.is_finite() && b.is_finite() && c.is_finite()) || (a + b + c - 1.0).abs() > BARYCENTRIC_SUM {
            return Err(BezierTriangleRefusal::Barycentric { a, b, c });
        }
        Ok(())
    }

    fn check_weight(h: Vec4, a: f64, b: f64, c: f64) -> Result<(), BezierTriangleRefusal> {
        if !(h.w > 0.0) {
            return Err(BezierTriangleRefusal::PoleOutside { a, b, c, weight: h.w });
        }
        Ok(())
    }

    /// Homogeneous point at barycentric `(a, b, c)` (3.11.7).
    pub fn evaluate_homogeneous(&self, a: f64, b: f64, c: f64) -> Result<Vec4, BezierTriangleRefusal> {
        Self::check_barycentric(a, b, c)?;
        Ok(self.blossom_homogeneous(&vec![[a, b, c]; self.degree]))
    }

    /// Point at barycentric `(a, b, c)`.
    pub fn evaluate(&self, a: f64, b: f64, c: f64) -> Result<Vec3, BezierTriangleRefusal> {
        let h = self.evaluate_homogeneous(a, b, c)?;
        Self::check_weight(h, a, b, c)?;
        Ok(project(h))
    }

    /// Point and directional derivative along the barycentric direction
    /// `(da, db, dc)` (components summing to zero), e.g. `(-1, 1, 0)` is
    /// `∂/∂b` with `c` held. The homogeneous derivative is
    /// `n·R(d, x, …, x)`, then the quotient rule.
    pub fn derivative(
        &self,
        a: f64,
        b: f64,
        c: f64,
        direction: [f64; 3],
    ) -> Result<(Vec3, Vec3), BezierTriangleRefusal> {
        Self::check_barycentric(a, b, c)?;
        let [da, db, dc] = direction;
        let scale = da.abs().max(db.abs()).max(dc.abs()).max(1.0);
        if !(da.is_finite() && db.is_finite() && dc.is_finite()) || (da + db + dc).abs() > BARYCENTRIC_SUM * scale {
            return Err(BezierTriangleRefusal::Direction { da, db, dc });
        }
        let n = self.degree;
        let mut args = vec![[a, b, c]; n];
        let h = self.blossom_homogeneous(&args);
        args[0] = direction;
        let dh = self.blossom_homogeneous(&args).scale(n as f64);
        Self::check_weight(h, a, b, c)?;
        let point = project(h);
        let derivative = Vec3::new(
            (dh.x - point.x * dh.w) / h.w,
            (dh.y - point.y * dh.w) / h.w,
            (dh.z - point.z * dh.w) / h.w,
        );
        Ok((point, derivative))
    }

    /// The boundary row on `side` as a rational Bézier curve on `[0, 1]`.
    pub fn side_curve(&self, side: TriangleSide) -> Result<NurbsCurve, BezierTriangleRefusal> {
        let n = self.degree;
        let row: Vec<Vec4> = (0..=n)
            .map(|step| match side {
                // a = 0: j + k = n, from (j = n, k = 0) to (j = 0, k = n).
                TriangleSide::OppositeA => self.control(n - step, step),
                // b = 0: j = 0, from k = n to k = 0.
                TriangleSide::OppositeB => self.control(0, n - step),
                // c = 0: k = 0, from j = 0 to j = n.
                TriangleSide::OppositeC => self.control(step, 0),
            })
            .collect();
        NurbsCurve::new(n, bezier_knots(n), row).map_err(|reason| BezierTriangleRefusal::Conversion { reason })
    }

    /// The same patch at degree `n + 1` (3.11.8):
    /// `P*_ijk = (i·P_{i−1,j,k} + j·P_{i,j−1,k} + k·P_{i,j,k−1}) / (n + 1)`.
    pub fn elevate(&self) -> Result<Self, BezierTriangleRefusal> {
        let n = self.degree;
        let up = n + 1;
        let mut points = Vec::with_capacity(Self::point_count(up));
        for k in 0..=up {
            for j in 0..=up - k {
                let i = up - j - k;
                let mut sum = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
                if i > 0 {
                    sum = sum.add(self.control(j, k).scale(i as f64));
                }
                if j > 0 {
                    sum = sum.add(self.control(j - 1, k).scale(j as f64));
                }
                if k > 0 {
                    sum = sum.add(self.control(j, k - 1).scale(k as f64));
                }
                points.push(sum.scale(1.0 / up as f64));
            }
        }
        Self::new(up, points)
    }

    /// The exact bidegree `(n, n)` tensor patch of the triangle composed with
    /// the bilinear map of `corners` (`[Q00, Q10, Q01, Q11]`, barycentric).
    /// Control point `(k, l)` is the symmetrized tensor blossom at
    /// `(1^k 0^(n−k); 1^l 0^(n−l))`: a hypergeometric average over how many
    /// `s = 1` slots pair with `t = 1` slots.
    pub fn compose_bilinear(&self, corners: [[f64; 3]; 4]) -> Result<TrianglePiece, BezierTriangleRefusal> {
        let n = self.degree;
        let [q00, q10, q01, q11] = corners;
        let mut rows = Vec::with_capacity(n + 1);
        for k in 0..=n {
            let mut row = Vec::with_capacity(n + 1);
            for l in 0..=n {
                let mut sum = Vec4 { x: 0.0, y: 0.0, z: 0.0, w: 0.0 };
                let low = (k + l).saturating_sub(n);
                for m in low..=k.min(l) {
                    let probability =
                        binomial(k, m) * binomial(n - k, l - m) / binomial(n, l);
                    let mut args = Vec::with_capacity(n);
                    args.extend(std::iter::repeat(q11).take(m));
                    args.extend(std::iter::repeat(q10).take(k - m));
                    args.extend(std::iter::repeat(q01).take(l - m));
                    args.extend(std::iter::repeat(q00).take(n + m - k - l));
                    sum = sum.add(self.blossom_homogeneous(&args).scale(probability));
                }
                row.push(sum);
            }
            rows.push(row);
        }
        let surface = NurbsSurface::new(n, n, bezier_knots(n), bezier_knots(n), rows)
            .map_err(|reason| BezierTriangleRefusal::Conversion { reason })?;
        Ok(TrianglePiece { surface, corners })
    }

    /// The exact conversion into three regular `(n, n)` patches, one per
    /// corner: piece `v` spans corner `V_v`, the midpoints of its two sides
    /// and the centroid, with `s` running toward the next corner's side
    /// midpoint and `t` toward the previous one's (each piece keeps the
    /// triangle's orientation). Adjacent pieces share their common side
    /// exactly: it is the image of one straight domain segment.
    pub fn to_nurbs_quads(&self) -> Result<[TrianglePiece; 3], BezierTriangleRefusal> {
        let vertex = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let midpoint = |p: usize, q: usize| -> [f64; 3] {
            let mut m = [0.0; 3];
            m[p] = 0.5;
            m[q] = 0.5;
            m
        };
        let third = 1.0 / 3.0;
        let centroid = [third, third, third];
        let piece = |v: usize| -> Result<TrianglePiece, BezierTriangleRefusal> {
            let next = (v + 1) % 3;
            let previous = (v + 2) % 3;
            self.compose_bilinear([vertex[v], midpoint(v, next), midpoint(previous, v), centroid])
        };
        Ok([piece(0)?, piece(1)?, piece(2)?])
    }

    /// The exact conversion into one `(n, n)` patch through
    /// `(s, t) ↦ (1 − s, s(1 − t), s t)`: side `s = 0` is collapsed to the
    /// corner `P_n00` (a pole), `s = 1` is the side `a = 0`.
    pub fn to_nurbs_collapsed(&self) -> Result<TrianglePiece, BezierTriangleRefusal> {
        let corner = [1.0, 0.0, 0.0];
        self.compose_bilinear([corner, [0.0, 1.0, 0.0], corner, [0.0, 0.0, 1.0]])
    }
}

fn project(h: Vec4) -> Vec3 {
    Vec3::new(h.x / h.w, h.y / h.w, h.z / h.w)
}

fn bilinear(corners: &[[f64; 3]; 4], s: f64, t: f64) -> [f64; 3] {
    let [q00, q10, q01, q11] = corners;
    let mut out = [0.0; 3];
    for axis in 0..3 {
        out[axis] = (1.0 - s) * (1.0 - t) * q00[axis]
            + s * (1.0 - t) * q10[axis]
            + (1.0 - s) * t * q01[axis]
            + s * t * q11[axis];
    }
    out
}

fn bezier_knots(degree: usize) -> Vec<f64> {
    let mut knots = vec![0.0; degree + 1];
    knots.extend(std::iter::repeat(1.0).take(degree + 1));
    knots
}

fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let k = k.min(n - k);
    let mut value = 1.0;
    for step in 0..k {
        value = value * (n - step) as f64 / (step + 1) as f64;
    }
    value
}

