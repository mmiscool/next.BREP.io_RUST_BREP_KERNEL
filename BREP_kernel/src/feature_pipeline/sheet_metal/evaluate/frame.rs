use super::*;

/// An orthonormal placement frame for a flat: world origin `o` and basis columns
/// `u` (local +X, in-plane), `v` (local +Y, in-plane), `w` (local +Z, the flat
/// normal). A local point `(x, y, z)` maps to `o + x·u + y·v + z·w`.
#[derive(Clone, Copy)]
pub(super) struct Frame {
    pub(super) o: Vec3,
    pub(super) u: Vec3,
    pub(super) v: Vec3,
    pub(super) w: Vec3,
}

impl Frame {
    /// Build a frame from a tree's `root_transform` (`[origin, x, y, z]`).
    pub(super) fn from_placement(p: &[[f64; 3]; 4]) -> Self {
        let v = |a: [f64; 3]| Vec3::new(a[0], a[1], a[2]);
        Frame {
            o: v(p[0]),
            u: v(p[1]),
            v: v(p[2]),
            w: v(p[3]),
        }
    }

    /// Map a local point into world space.
    pub(super) fn point(&self, x: f64, y: f64, z: f64) -> Vec3 {
        self.o
            .add(self.u.scale(x))
            .add(self.v.scale(y))
            .add(self.w.scale(z))
    }

    /// The row-major affine that takes a canonically-built local solid (local
    /// axes = world axes through the origin) into this frame.
    pub(super) fn affine(&self) -> Result<AffineTransform, String> {
        AffineTransform::new([
            self.u.x, self.v.x, self.w.x, self.o.x, //
            self.u.y, self.v.y, self.w.y, self.o.y, //
            self.u.z, self.v.z, self.w.z, self.o.z, //
            0.0, 0.0, 0.0, 1.0,
        ])
    }
}

/// Rodrigues rotation of `v` about unit axis `k` by `angle` radians.
pub(super) fn rotate_about(v: Vec3, k: Vec3, angle: f64) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v.scale(c)
        .add(k.cross(v).scale(s))
        .add(k.scale(k.dot(v) * (1.0 - c)))
}
