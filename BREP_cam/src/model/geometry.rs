//! Plain geometric value types shared by every CAM algorithm.
//!
//! Points and vectors serialize as `[x, y, z]` so toolpaths and meshes stay
//! compact in documents and worker payloads. Nothing here depends on the BREP
//! kernel or on a renderer math library.

use serde::{Deserialize, Serialize};

/// A point or vector in 3D. Serialized as a three-element array.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(from = "[f64; 3]", into = "[f64; 3]")]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A direction or displacement; the same representation as [`Point3`].
pub type Vector3 = Point3;

impl From<[f64; 3]> for Point3 {
    fn from(v: [f64; 3]) -> Self {
        Self { x: v[0], y: v[1], z: v[2] }
    }
}

impl From<Point3> for [f64; 3] {
    fn from(p: Point3) -> Self {
        [p.x, p.y, p.z]
    }
}

impl Point3 {
    pub const ZERO: Point3 = Point3 { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Point3 = Point3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Point3 = Point3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Point3 = Point3 { x: 0.0, y: 0.0, z: 1.0 };

    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    pub fn add(self, o: Point3) -> Point3 {
        Point3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }

    pub fn sub(self, o: Point3) -> Point3 {
        Point3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }

    pub fn scale(self, s: f64) -> Point3 {
        Point3::new(self.x * s, self.y * s, self.z * s)
    }

    pub fn dot(self, o: Point3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Point3) -> Point3 {
        Point3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    /// Unit vector in this direction; `None` for a zero or non-finite vector.
    pub fn normalized(self) -> Option<Point3> {
        let len = self.length();
        if !(len > 0.0) || !len.is_finite() {
            return None;
        }
        Some(self.scale(1.0 / len))
    }

    pub fn distance(self, o: Point3) -> f64 {
        self.sub(o).length()
    }

    /// Distance in the XY plane only (the cutter-axis projection).
    pub fn distance_xy(self, o: Point3) -> f64 {
        let dx = self.x - o.x;
        let dy = self.y - o.y;
        (dx * dx + dy * dy).sqrt()
    }

    pub fn lerp(self, o: Point3, t: f64) -> Point3 {
        self.add(o.sub(self).scale(t))
    }

    pub fn min(self, o: Point3) -> Point3 {
        Point3::new(self.x.min(o.x), self.y.min(o.y), self.z.min(o.z))
    }

    pub fn max(self, o: Point3) -> Point3 {
        Point3::new(self.x.max(o.x), self.y.max(o.y), self.z.max(o.z))
    }

    /// Component by axis index (0 = x, 1 = y, 2 = z).
    pub fn axis(self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

/// A point in 2D (sketch/region space). Serialized as `[x, y]`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(from = "[f64; 2]", into = "[f64; 2]")]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}

impl From<[f64; 2]> for Point2 {
    fn from(v: [f64; 2]) -> Self {
        Self { x: v[0], y: v[1] }
    }
}

impl From<Point2> for [f64; 2] {
    fn from(p: Point2) -> Self {
        [p.x, p.y]
    }
}

impl Point2 {
    pub const fn new(x: f64, y: f64) -> Self {
        Self { x, y }
    }

    pub fn is_finite(&self) -> bool {
        self.x.is_finite() && self.y.is_finite()
    }
}

/// Axis-aligned bounding box. An empty box has `min > max` on every axis.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Bounds3 {
    pub min: Point3,
    pub max: Point3,
}

impl Default for Bounds3 {
    fn default() -> Self {
        Self::empty()
    }
}

impl Bounds3 {
    /// The empty box: `union` with any point yields that point.
    pub fn empty() -> Self {
        Self {
            min: Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            max: Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        }
    }

    pub fn new(min: Point3, max: Point3) -> Self {
        Self { min, max }
    }

    pub fn from_point(p: Point3) -> Self {
        Self { min: p, max: p }
    }

    pub fn from_points<I: IntoIterator<Item = Point3>>(points: I) -> Self {
        let mut b = Self::empty();
        for p in points {
            b.include(p);
        }
        b
    }

    pub fn is_empty(&self) -> bool {
        self.min.x > self.max.x || self.min.y > self.max.y || self.min.z > self.max.z
    }

    pub fn is_finite(&self) -> bool {
        self.min.is_finite() && self.max.is_finite()
    }

    pub fn include(&mut self, p: Point3) {
        self.min = self.min.min(p);
        self.max = self.max.max(p);
    }

    pub fn union(&self, o: &Bounds3) -> Bounds3 {
        Bounds3 { min: self.min.min(o.min), max: self.max.max(o.max) }
    }

    /// Grow every side by `d` (shrink for negative `d`).
    pub fn expanded(&self, d: f64) -> Bounds3 {
        let e = Point3::new(d, d, d);
        Bounds3 { min: self.min.sub(e), max: self.max.add(e) }
    }

    /// Closed-interval overlap on all three axes (touching counts).
    pub fn intersects(&self, o: &Bounds3) -> bool {
        self.min.x <= o.max.x
            && o.min.x <= self.max.x
            && self.min.y <= o.max.y
            && o.min.y <= self.max.y
            && self.min.z <= o.max.z
            && o.min.z <= self.max.z
    }

    pub fn contains_point(&self, p: Point3) -> bool {
        p.x >= self.min.x
            && p.x <= self.max.x
            && p.y >= self.min.y
            && p.y <= self.max.y
            && p.z >= self.min.z
            && p.z <= self.max.z
    }

    pub fn center(&self) -> Point3 {
        self.min.add(self.max).scale(0.5)
    }

    pub fn size(&self) -> Point3 {
        self.max.sub(self.min)
    }

    /// Length of the box diagonal; `0.0` for an empty box.
    pub fn diagonal(&self) -> f64 {
        if self.is_empty() {
            0.0
        } else {
            self.size().length()
        }
    }

    /// Index of the axis with the largest extent (0, 1 or 2).
    pub fn longest_axis(&self) -> usize {
        let s = self.size();
        if s.x >= s.y && s.x >= s.z {
            0
        } else if s.y >= s.z {
            1
        } else {
            2
        }
    }
}

/// A proper rigid transform (rotation + translation, no scale, no mirror).
///
/// `rotation` is row-major: `apply_vector(v) = rotation · v`. Build one with
/// [`RigidTransform::new`] (validated) or the named constructors. The setup's
/// model-to-machine transform is the ONLY rigid transform CAM applies to
/// geometry; display conversion back uses [`RigidTransform::inverse`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RigidTransform {
    pub rotation: [[f64; 3]; 3],
    pub translation: Point3,
}

impl Default for RigidTransform {
    fn default() -> Self {
        Self::identity()
    }
}

/// Why a matrix was refused as a rigid transform.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum TransformError {
    NonFinite,
    NotOrthonormal,
    Mirrored,
}

impl std::fmt::Display for TransformError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransformError::NonFinite => write!(f, "rigid transform has a non-finite element"),
            TransformError::NotOrthonormal => {
                write!(f, "rigid transform rotation is not orthonormal")
            }
            TransformError::Mirrored => write!(f, "rigid transform mirrors (determinant is -1)"),
        }
    }
}

impl std::error::Error for TransformError {}

impl RigidTransform {
    /// Tolerance used by [`RigidTransform::validate`] on the orthonormality residual.
    pub const ORTHONORMAL_EPS: f64 = 1e-9;

    pub fn identity() -> Self {
        Self {
            rotation: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            translation: Point3::ZERO,
        }
    }

    /// Validated constructor: refuses non-finite, non-orthonormal or mirroring matrices.
    pub fn new(rotation: [[f64; 3]; 3], translation: Point3) -> Result<Self, TransformError> {
        let t = Self { rotation, translation };
        t.validate()?;
        Ok(t)
    }

    pub fn translation(t: Point3) -> Self {
        Self { rotation: Self::identity().rotation, translation: t }
    }

    /// Rotation by `angle_rad` about the unit `axis` (right-hand rule), then translation `t`.
    pub fn axis_angle(axis: Point3, angle_rad: f64, t: Point3) -> Result<Self, TransformError> {
        let a = axis.normalized().ok_or(TransformError::NonFinite)?;
        let (s, c) = angle_rad.sin_cos();
        let k = 1.0 - c;
        let rotation = [
            [c + a.x * a.x * k, a.x * a.y * k - a.z * s, a.x * a.z * k + a.y * s],
            [a.y * a.x * k + a.z * s, c + a.y * a.y * k, a.y * a.z * k - a.x * s],
            [a.z * a.x * k - a.y * s, a.z * a.y * k + a.x * s, c + a.z * a.z * k],
        ];
        Self::new(rotation, t)
    }

    /// The transform that maps a frame given in model coordinates (origin,
    /// X axis, Z axis) onto the machine frame: the frame origin goes to the
    /// machine origin, the frame's X to +X and its Z to +Z. `x_axis` is
    /// re-orthogonalized against `z_axis`.
    pub fn from_frame(origin: Point3, x_axis: Point3, z_axis: Point3) -> Result<Self, TransformError> {
        let z = z_axis.normalized().ok_or(TransformError::NonFinite)?;
        let x0 = x_axis.sub(z.scale(x_axis.dot(z)));
        let x = x0.normalized().ok_or(TransformError::NotOrthonormal)?;
        let y = z.cross(x);
        // Rows of the rotation are the frame axes: rotation · (p - origin)
        // yields the coordinates of p in the frame.
        let rotation = [[x.x, x.y, x.z], [y.x, y.y, y.z], [z.x, z.y, z.z]];
        let rotated_origin = Self { rotation, translation: Point3::ZERO }.apply_vector(origin);
        Self::new(rotation, rotated_origin.scale(-1.0))
    }

    pub fn validate(&self) -> Result<(), TransformError> {
        let r = &self.rotation;
        if !self.translation.is_finite() || r.iter().flatten().any(|v| !v.is_finite()) {
            return Err(TransformError::NonFinite);
        }
        let row = |i: usize| Point3::new(r[i][0], r[i][1], r[i][2]);
        let (a, b, c) = (row(0), row(1), row(2));
        let residual = (a.dot(a) - 1.0).abs().max((b.dot(b) - 1.0).abs()).max((c.dot(c) - 1.0).abs())
            .max(a.dot(b).abs()).max(a.dot(c).abs()).max(b.dot(c).abs());
        if residual > Self::ORTHONORMAL_EPS {
            return Err(TransformError::NotOrthonormal);
        }
        if a.cross(b).dot(c) < 0.0 {
            return Err(TransformError::Mirrored);
        }
        Ok(())
    }

    pub fn is_identity(&self) -> bool {
        *self == Self::identity()
    }

    pub fn apply_vector(&self, v: Point3) -> Point3 {
        let r = &self.rotation;
        Point3::new(
            r[0][0] * v.x + r[0][1] * v.y + r[0][2] * v.z,
            r[1][0] * v.x + r[1][1] * v.y + r[1][2] * v.z,
            r[2][0] * v.x + r[2][1] * v.y + r[2][2] * v.z,
        )
    }

    pub fn apply_point(&self, p: Point3) -> Point3 {
        self.apply_vector(p).add(self.translation)
    }

    /// Transform a box: the box of the eight transformed corners.
    pub fn apply_bounds(&self, b: &Bounds3) -> Bounds3 {
        if b.is_empty() {
            return Bounds3::empty();
        }
        let mut out = Bounds3::empty();
        for i in 0..8 {
            let p = Point3::new(
                if i & 1 == 0 { b.min.x } else { b.max.x },
                if i & 2 == 0 { b.min.y } else { b.max.y },
                if i & 4 == 0 { b.min.z } else { b.max.z },
            );
            out.include(self.apply_point(p));
        }
        out
    }

    /// The inverse: rotation transposed, translation rotated back.
    pub fn inverse(&self) -> Self {
        let r = &self.rotation;
        let rt = [[r[0][0], r[1][0], r[2][0]], [r[0][1], r[1][1], r[2][1]], [r[0][2], r[1][2], r[2][2]]];
        let back = Self { rotation: rt, translation: Point3::ZERO };
        let t = back.apply_vector(self.translation).scale(-1.0);
        Self { rotation: rt, translation: t }
    }

    /// `self` followed by `next`: `(next ∘ self)(p) = next(self(p))`.
    pub fn then(&self, next: &RigidTransform) -> Self {
        let mut rotation = [[0.0; 3]; 3];
        for (i, row) in rotation.iter_mut().enumerate() {
            for (j, cell) in row.iter_mut().enumerate() {
                *cell = (0..3).map(|k| next.rotation[i][k] * self.rotation[k][j]).sum();
            }
        }
        Self { rotation, translation: next.apply_point(self.translation) }
    }
}
