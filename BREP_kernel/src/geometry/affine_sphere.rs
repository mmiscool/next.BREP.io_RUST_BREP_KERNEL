//! AFFINE SPHERES (2026-11-02): a sphere net under an affine map that is not a
//! similarity is an ellipsoid (27, 17], its ears and the orca's eyes are
//! scaled too). The net is not a recognized analytic carrier, so every
//! projection and inversion on it ran the general grid-seeded Newton
//! (`projection.rs`), 27,825 of them in the elephant's one boolean fold. This
//! module recognizes the net exactly (the affine map that carries the
//! canonical unit-sphere net onto it, verified over all 45 control points),
//! and answers point projection exactly: the foot of a point on an ellipsoid
//! is Eberly's one-dimensional root in the ellipsoid's principal frame
//! (bracketed bisection to machine precision; the foot is unique off the
//! medial axis of a convex quadric), and the parameters follow from the sphere
//! chart of the foot's preimage, because the net's parameterization IS the
//! canonical sphere's under the map. A plane section is an ellipse: the plane
//! pulled back through the map meets the unit sphere in a circle, whose
//! rational arcs map forward exactly.
//!
//! Switch `BREP_AFFINE_SPHERE` (on by default since the 2026-11-02 promotion;
//! `BREP_AFFINE_SPHERE=0` for a same-binary A/B): off, no surface recognizes
//! an affine sphere and every path is byte-identical to before this module.
//! The carrier lives in its own per-surface cache
//! ([`NurbsSurface::affine_sphere`]), not in [`crate::AnalyticSurface`], so
//! the 300-odd consumers of the analytic carrier never see it; only the
//! projection entry points, the pcurve inversion and the analytic pair
//! intersection consult it.

use crate::{circle_angle_to_parameter, make_arc, NurbsCurve, NurbsSurface, SurfaceProjection, Vec3, Vec4};

/// Same reconstruction tolerance as the analytic recognizers, relative to
/// the net's coordinate scale.
const RECOGNITION_TOLERANCE: f64 = 1e-9;

/// `BREP_AFFINE_SPHERE=0` reads every ellipsoid as a general patch again
/// (the reading before the 2026-11-02 promotion). Read once per process.
pub(crate) fn affine_sphere_enabled() -> bool {
    crate::switch!("BREP_AFFINE_SPHERE", true)
}

/// Whether `surface` has the full sphere net's shape: degree (2, 2), nine
/// control rows (four 90° spans about the axis) of five points (two 90° spans
/// of the meridian). A shape read only; [`recognize`] proves the net.
pub(crate) fn has_sphere_net_shape(surface: &NurbsSurface) -> bool {
    surface.degree_u == 2
        && surface.degree_v == 2
        && surface.control_points.len() == 9
        && surface.control_points.iter().all(|row| row.len() == 5)
}

/// The unit sphere's net this module measures every candidate against:
/// centre at the origin, polar axis +z, the u = 0 meridian through +x. Its
/// knots and homogeneous control points only (a `NurbsSurface` carries
/// per-instance caches and is not shared between threads).
struct CanonicalNet {
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    control_points: Vec<Vec<Vec4>>,
}

fn canonical_net() -> &'static CanonicalNet {
    static NET: std::sync::OnceLock<CanonicalNet> = std::sync::OnceLock::new();
    NET.get_or_init(|| {
        let sphere = crate::make_sphere_surface_framed(
            Vec3::new(0.0, 0.0, 0.0),
            1.0,
            Vec3::new(0.0, 0.0, 1.0),
            Some(Vec3::new(1.0, 0.0, 0.0)),
        )
        .expect("the canonical unit sphere net builds");
        CanonicalNet {
            knots_u: sphere.knots_u.clone(),
            knots_v: sphere.knots_v.clone(),
            control_points: sphere.control_points.clone(),
        }
    })
}

/// An ellipsoid: the image `p = M s + c` of the canonical unit sphere.
#[derive(Clone, Debug)]
pub(crate) struct AffineSphere {
    /// `c`.
    center: Vec3,
    /// The columns of `M`: the images of the unit x, y and z directions.
    columns: [Vec3; 3],
    /// The rows of `M⁻¹`, so the preimage of `d` is `(r0·d, r1·d, r2·d)`.
    inverse_rows: [Vec3; 3],
    /// Principal frame: unit axes, with the semi-axis lengths in the same
    /// order, descending (`semi[0] >= semi[1] >= semi[2] > 0`).
    axes: [Vec3; 3],
    semi: [f64; 3],
}

/// Recognize `surface` as an affine image of the canonical sphere net: the
/// same degrees, knots and weights, and one affine map (least squares over
/// the 45 dehomogenized control points, then verified at every one of them
/// within [`RECOGNITION_TOLERANCE`] of the net's scale) that carries the
/// canonical points onto the surface's. A similarity is still an affine
/// sphere here; the callers ask only when the analytic recognizers declined.
pub(crate) fn recognize(surface: &NurbsSurface) -> Option<AffineSphere> {
    if !has_sphere_net_shape(surface) {
        return None;
    }
    let canonical = canonical_net();
    if surface.knots_u.len() != canonical.knots_u.len()
        || surface.knots_v.len() != canonical.knots_v.len()
        || surface
            .knots_u
            .iter()
            .zip(&canonical.knots_u)
            .any(|(a, b)| (a - b).abs() > crate::curve::KNOT_DEDUP_EPS)
        || surface
            .knots_v
            .iter()
            .zip(&canonical.knots_v)
            .any(|(a, b)| (a - b).abs() > crate::curve::KNOT_DEDUP_EPS)
    {
        return None;
    }
    let mut scale = 0.0f64;
    let mut pairs: Vec<(Vec3, Vec3)> = Vec::with_capacity(45);
    for (row, canonical_row) in surface.control_points.iter().zip(&canonical.control_points) {
        for (control, reference) in row.iter().zip(canonical_row) {
            if !control.w.is_finite() || (control.w - reference.w).abs() > RECOGNITION_TOLERANCE {
                return None;
            }
            let point = control.point().ok()?;
            scale = scale.max(point.x.abs()).max(point.y.abs()).max(point.z.abs());
            pairs.push((reference.point().ok()?, point));
        }
    }
    // Normal equations of Q = M P + c over the pairs: one 4x4 system, three
    // right-hand sides (one per output coordinate).
    let mut ata = [[0.0f64; 4]; 4];
    let mut atb = [[0.0f64; 4]; 3];
    for (p, q) in &pairs {
        let row = [p.x, p.y, p.z, 1.0];
        let out = [q.x, q.y, q.z];
        for i in 0..4 {
            for j in 0..4 {
                ata[i][j] += row[i] * row[j];
            }
            for (k, value) in out.iter().enumerate() {
                atb[k][i] += row[i] * value;
            }
        }
    }
    let rows = [
        solve4(ata, atb[0])?,
        solve4(ata, atb[1])?,
        solve4(ata, atb[2])?,
    ];
    let columns = [
        Vec3::new(rows[0][0], rows[1][0], rows[2][0]),
        Vec3::new(rows[0][1], rows[1][1], rows[2][1]),
        Vec3::new(rows[0][2], rows[1][2], rows[2][2]),
    ];
    let center = Vec3::new(rows[0][3], rows[1][3], rows[2][3]);
    let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
    let image = |s: Vec3| {
        center
            .add(columns[0].scale(s.x))
            .add(columns[1].scale(s.y))
            .add(columns[2].scale(s.z))
    };
    if pairs.iter().any(|(p, q)| {
        let d = image(*p).sub(*q);
        d.x.abs() > tolerance || d.y.abs() > tolerance || d.z.abs() > tolerance
    }) {
        return None;
    }
    // Inverse by the adjugate; a flat map (an ellipsoid with a zero
    // semi-axis) is not a surface this module answers for.
    let [a, b, c] = columns;
    let cofactor_rows = [b.cross(c), c.cross(a), a.cross(b)];
    let determinant = a.dot(cofactor_rows[0]);
    let norm = scale.max(1.0);
    if !determinant.is_finite() || determinant.abs() <= 1e-18 * norm * norm * norm {
        return None;
    }
    // M⁻¹ = adj(M) / det, with adj(M)'s rows the cofactor cross products.
    let inverse_rows = cofactor_rows.map(|row| row.scale(1.0 / determinant));
    // Principal frame from M Mᵀ = U diag(semi²) Uᵀ.
    let mut gram = [[0.0f64; 3]; 3];
    for (i, gram_row) in gram.iter_mut().enumerate() {
        for (j, entry) in gram_row.iter_mut().enumerate() {
            *entry = columns.iter().map(|column| column[i] * column[j]).sum();
        }
    }
    let (values, vectors) = brep_ransac::jacobi_eigen_symmetric3(gram, brep_ransac::JacobiZeroTurn::Positive);
    // Ascending from the solver; this carrier wants descending.
    let mut axes = [Vec3::default(); 3];
    let mut semi = [0.0f64; 3];
    for (slot, index) in [2usize, 1, 0].into_iter().enumerate() {
        let [x, y, z] = vectors[index];
        axes[slot] = Vec3::new(x, y, z).normalized().ok()?;
        semi[slot] = values[index].max(0.0).sqrt();
    }
    if !(semi[2] > RECOGNITION_TOLERANCE * semi[0]) {
        return None;
    }
    Some(AffineSphere {
        center,
        columns,
        inverse_rows,
        axes,
        semi,
    })
}

/// Gaussian elimination with partial pivoting on a 4x4 system.
fn solve4(mut a: [[f64; 4]; 4], mut b: [f64; 4]) -> Option<[f64; 4]> {
    for column in 0..4 {
        let pivot = (column..4).max_by(|&i, &j| a[i][column].abs().total_cmp(&a[j][column].abs()))?;
        if a[pivot][column].abs() <= 1e-300 {
            return None;
        }
        a.swap(column, pivot);
        b.swap(column, pivot);
        for row in column + 1..4 {
            let factor = a[row][column] / a[column][column];
            if factor != 0.0 {
                for k in column..4 {
                    a[row][k] -= factor * a[column][k];
                }
                b[row] -= factor * b[column];
            }
        }
    }
    let mut x = [0.0f64; 4];
    for row in (0..4).rev() {
        let mut sum = b[row];
        for k in row + 1..4 {
            sum -= a[row][k] * x[k];
        }
        x[row] = sum / a[row][row];
    }
    x.iter().all(|value| value.is_finite()).then_some(x)
}

impl AffineSphere {
    /// `c`.
    pub(crate) fn center(&self) -> Vec3 {
        self.center
    }

    /// The semi-axis lengths, descending.
    pub(crate) fn semi_axes(&self) -> [f64; 3] {
        self.semi
    }

    /// The principal axes (unit), in the semi-axes' order.
    pub(crate) fn axes(&self) -> [Vec3; 3] {
        self.axes
    }

    /// `M s + c`.
    pub(crate) fn image(&self, s: Vec3) -> Vec3 {
        self.center
            .add(self.columns[0].scale(s.x))
            .add(self.columns[1].scale(s.y))
            .add(self.columns[2].scale(s.z))
    }

    /// `M⁻¹ (p − c)`.
    pub(crate) fn preimage(&self, p: Vec3) -> Vec3 {
        let d = p.sub(self.center);
        Vec3::new(self.inverse_rows[0].dot(d), self.inverse_rows[1].dot(d), self.inverse_rows[2].dot(d))
    }

    /// The nearest point of the ellipsoid to `point` (a global minimizer; on
    /// the medial axis one of the tied feet), in model space.
    pub(crate) fn foot(&self, point: Vec3) -> Vec3 {
        let d = point.sub(self.center);
        let y = [d.dot(self.axes[0]), d.dot(self.axes[1]), d.dot(self.axes[2])];
        let x = nearest_on_ellipsoid(self.semi, y);
        self.center
            .add(self.axes[0].scale(x[0]))
            .add(self.axes[1].scale(x[1]))
            .add(self.axes[2].scale(x[2]))
    }

    /// The canonical sphere chart of a unit preimage direction: the net's
    /// `(u, v)` at which it evaluates to the image of that direction.
    fn chart(preimage: Vec3) -> (f64, f64) {
        let rho = preimage.x.hypot(preimage.y);
        let u = if rho <= 1e-14 * (1.0 + preimage.z.abs()) {
            0.0
        } else {
            let mut theta = preimage.y.atan2(preimage.x);
            if theta < 0.0 {
                theta += std::f64::consts::TAU;
            }
            circle_angle_to_parameter(4, std::f64::consts::TAU, theta)
        };
        // Polar angle from the south pole (v = 0) over the meridian's two spans.
        let beta = rho.atan2(-preimage.z);
        (u, circle_angle_to_parameter(2, std::f64::consts::PI, beta))
    }

    /// Exact projection in the net's own parameterization: the foot's
    /// preimage on the unit sphere names `(u, v)`; the point is the net
    /// evaluated there, as every analytic carrier answers.
    pub(crate) fn project(&self, surface: &NurbsSurface, point: Vec3) -> Option<SurfaceProjection> {
        let foot = self.foot(point);
        let preimage = self.preimage(foot);
        if !preimage.x.is_finite() || !preimage.y.is_finite() || !preimage.z.is_finite() {
            return None;
        }
        let (u, v) = Self::chart(preimage);
        let projected = surface.evaluate(u, v).ok()?;
        Some(SurfaceProjection {
            u,
            v,
            point: projected,
            distance: projected.sub(point).length(),
        })
    }

    /// The section by the plane through `origin` with unit `normal`: the
    /// ellipse that is the image of the pulled-back plane's circle on the unit
    /// sphere, one full rational quadratic loop; `Some(vec![])` when the plane
    /// provably misses the ellipsoid (or grazes it within `tolerance`, the
    /// plane/sphere section's policy).
    pub(crate) fn plane_section(&self, origin: Vec3, normal: Vec3, tolerance: f64) -> Option<Vec<NurbsCurve>> {
        // n·(M s + c − o) = 0  ⇔  (Mᵀ n)·s = n·(o − c).
        let pulled = Vec3::new(self.columns[0].dot(normal), self.columns[1].dot(normal), self.columns[2].dot(normal));
        let length = pulled.length();
        if !(length > 0.0) || !length.is_finite() {
            return None;
        }
        let unit = pulled.scale(1.0 / length);
        let distance = normal.dot(origin.sub(self.center)) / length;
        // A model-space band of `tolerance` is at least `tolerance / semi[0]`
        // wide in sphere space; the conservative graze margin.
        if distance.abs() >= 1.0 - tolerance / self.semi[0] {
            return Some(Vec::new());
        }
        let radius = (1.0 - distance * distance).sqrt();
        let x_axis = unit.perpendicular().ok()?;
        let y_axis = unit.cross(x_axis);
        let circle = make_arc(unit.scale(distance), x_axis, y_axis, radius, 0.0, std::f64::consts::TAU).ok()?;
        let controls = circle
            .control_points
            .iter()
            .map(|control| {
                // Homogeneous: (x, y, z) = w·s, so the image is M (x, y, z) + w c.
                let mapped = self
                    .columns[0]
                    .scale(control.x)
                    .add(self.columns[1].scale(control.y))
                    .add(self.columns[2].scale(control.z))
                    .add(self.center.scale(control.w));
                Vec4 { x: mapped.x, y: mapped.y, z: mapped.z, w: control.w }
            })
            .collect();
        Some(vec![NurbsCurve::new(circle.degree, circle.knots.clone(), controls).ok()?])
    }
}

/// The nearest point of the ellipsoid `(x0/e0)² + (x1/e1)² + (x2/e2)² = 1`,
/// `e0 >= e1 >= e2 > 0`, to `y` (Eberly, "Distance from a Point to an
/// Ellipse, an Ellipsoid, or a Hyperellipsoid", §3): by symmetry the
/// problem is solved in the closed first octant and the signs restored.
pub(crate) fn nearest_on_ellipsoid(e: [f64; 3], y: [f64; 3]) -> [f64; 3] {
    let magnitude = [y[0].abs(), y[1].abs(), y[2].abs()];
    let x = nearest_first_octant_3(e, magnitude);
    [x[0].copysign(y[0]), x[1].copysign(y[1]), x[2].copysign(y[2])]
}

fn nearest_first_octant_3(e: [f64; 3], y: [f64; 3]) -> [f64; 3] {
    if y[2] > 0.0 {
        if y[1] > 0.0 {
            if y[0] > 0.0 {
                let z = [y[0] / e[0], y[1] / e[1], y[2] / e[2]];
                let g = z[0] * z[0] + z[1] * z[1] + z[2] * z[2] - 1.0;
                if g != 0.0 {
                    let r0 = (e[0] / e[2]) * (e[0] / e[2]);
                    let r1 = (e[1] / e[2]) * (e[1] / e[2]);
                    let s = root_3(r0, r1, z, g);
                    [r0 * y[0] / (s + r0), r1 * y[1] / (s + r1), y[2] / (s + 1.0)]
                } else {
                    y
                }
            } else {
                let [x1, x2] = nearest_first_quadrant_2([e[1], e[2]], [y[1], y[2]]);
                [0.0, x1, x2]
            }
        } else if y[0] > 0.0 {
            let [x0, x2] = nearest_first_quadrant_2([e[0], e[2]], [y[0], y[2]]);
            [x0, 0.0, x2]
        } else {
            [0.0, 0.0, e[2]]
        }
    } else {
        let denominator0 = e[0] * e[0] - e[2] * e[2];
        let denominator1 = e[1] * e[1] - e[2] * e[2];
        let numerator0 = e[0] * y[0];
        let numerator1 = e[1] * y[1];
        if numerator0 < denominator0 && numerator1 < denominator1 {
            let xde0 = numerator0 / denominator0;
            let xde1 = numerator1 / denominator1;
            let discriminant = 1.0 - xde0 * xde0 - xde1 * xde1;
            if discriminant > 0.0 {
                return [e[0] * xde0, e[1] * xde1, e[2] * discriminant.sqrt()];
            }
        }
        let [x0, x1] = nearest_first_quadrant_2([e[0], e[1]], [y[0], y[1]]);
        [x0, x1, 0.0]
    }
}

fn nearest_first_quadrant_2(e: [f64; 2], y: [f64; 2]) -> [f64; 2] {
    if y[1] > 0.0 {
        if y[0] > 0.0 {
            let z = [y[0] / e[0], y[1] / e[1]];
            let g = z[0] * z[0] + z[1] * z[1] - 1.0;
            if g != 0.0 {
                let r0 = (e[0] / e[1]) * (e[0] / e[1]);
                let s = root_2(r0, z, g);
                [r0 * y[0] / (s + r0), y[1] / (s + 1.0)]
            } else {
                y
            }
        } else {
            [0.0, e[1]]
        }
    } else {
        let numerator0 = e[0] * y[0];
        let denominator0 = e[0] * e[0] - e[1] * e[1];
        if numerator0 < denominator0 {
            let xde0 = numerator0 / denominator0;
            [e[0] * xde0, e[1] * (1.0 - xde0 * xde0).max(0.0).sqrt()]
        } else {
            [e[0], 0.0]
        }
    }
}

/// Iteration cap for the root solves; Newton converges in a handful of steps
/// and the bisection fallback in at most the exponent range.
const ROOT_ITERATIONS: usize = 1100;

/// The root `s` of `F(s) = Σ (n_i/(s+r_i))² − 1 = 0` on `[s0, s1]`, where
/// `F` is convex and strictly decreasing (each term is), `F(s0) >= 0` and
/// `F(s1) <= 0`. A Newton step from any point of the bracket lands at or left
/// of the root (the tangent of a convex function lies under it), so from the
/// upper end the iterates climb monotonically to the root and converge
/// quadratically; a step that leaves the bracket by rounding bisects it.
/// Eberly's bisection ("Distance from a Point to an Ellipse, an Ellipsoid,
/// or a Hyperellipsoid") reached the same root one bit at a time.
fn bracketed_root(mut s0: f64, mut s1: f64, residual: impl Fn(f64) -> (f64, f64)) -> f64 {
    let mut s = s1;
    for _ in 0..ROOT_ITERATIONS {
        let (value, slope) = residual(s);
        if value == 0.0 || !value.is_finite() || !(slope < 0.0) {
            break;
        }
        if value > 0.0 {
            s0 = s;
        } else {
            s1 = s;
        }
        let newton = s - value / slope;
        let next = if newton > s0 && newton < s1 { newton } else { 0.5 * (s0 + s1) };
        if next == s || next == s0 || next == s1 {
            break;
        }
        s = next;
    }
    s
}

fn root_3(r0: f64, r1: f64, z: [f64; 3], g: f64) -> f64 {
    let n0 = r0 * z[0];
    let n1 = r1 * z[1];
    let s0 = z[2] - 1.0;
    let s1 = if g < 0.0 { 0.0 } else { (n0 * n0 + n1 * n1 + z[2] * z[2]).sqrt() - 1.0 };
    bracketed_root(s0, s1, |s| {
        let d0 = s + r0;
        let d1 = s + r1;
        let d2 = s + 1.0;
        let q0 = n0 / d0;
        let q1 = n1 / d1;
        let q2 = z[2] / d2;
        (
            q0 * q0 + q1 * q1 + q2 * q2 - 1.0,
            -2.0 * (q0 * q0 / d0 + q1 * q1 / d1 + q2 * q2 / d2),
        )
    })
}

fn root_2(r0: f64, z: [f64; 2], g: f64) -> f64 {
    let n0 = r0 * z[0];
    let s0 = z[1] - 1.0;
    let s1 = if g < 0.0 { 0.0 } else { (n0 * n0 + z[1] * z[1]).sqrt() - 1.0 };
    bracketed_root(s0, s1, |s| {
        let d0 = s + r0;
        let d1 = s + 1.0;
        let q0 = n0 / d0;
        let q1 = z[1] / d1;
        (q0 * q0 + q1 * q1 - 1.0, -2.0 * (q0 * q0 / d0 + q1 * q1 / d1))
    })
}

