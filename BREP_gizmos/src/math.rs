//! Minimal vector/ray math for gizmos (no external math dep — keeps the crate
//! light and the compile fast).

/// A world-space 3-vector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);
    pub const X: Vec3 = Vec3::new(1.0, 0.0, 0.0);
    pub const Y: Vec3 = Vec3::new(0.0, 1.0, 0.0);
    pub const Z: Vec3 = Vec3::new(0.0, 0.0, 1.0);

    pub fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
    pub fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
    pub fn scale(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }
    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }
    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }
    pub fn normalized(self) -> Vec3 {
        let l = self.length();
        if l < 1e-9 {
            Vec3::ZERO
        } else {
            self.scale(1.0 / l)
        }
    }
    /// Linear interpolation.
    pub fn lerp(self, o: Vec3, t: f32) -> Vec3 {
        self.add(o.sub(self).scale(t))
    }
    /// Any unit vector perpendicular to self.
    pub fn any_perp(self) -> Vec3 {
        let a = if self.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
        self.cross(a).normalized()
    }
}

impl From<Vec3> for [f32; 3] {
    fn from(v: Vec3) -> [f32; 3] {
        [v.x, v.y, v.z]
    }
}
impl From<[f32; 3]> for Vec3 {
    fn from(a: [f32; 3]) -> Vec3 {
        Vec3::new(a[0], a[1], a[2])
    }
}

/// A pick ray.
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: Vec3,
    pub dir: Vec3,
}

impl Ray {
    /// Point at parameter `t`.
    pub fn at(&self, t: f32) -> Vec3 {
        self.origin.add(self.dir.scale(t))
    }

    /// Closest distance from this ray to a point (for line/handle picking).
    pub fn distance_to_point(&self, p: Vec3) -> f32 {
        let w = p.sub(self.origin);
        let t = w.dot(self.dir).max(0.0);
        p.sub(self.at(t)).length()
    }

    /// Closest approach between this ray and a world segment [a,b]; returns the
    /// minimum distance (for edge/axis-handle picking).
    pub fn distance_to_segment(&self, a: Vec3, b: Vec3) -> f32 {
        // Ray r(s)=o+s*d, segment p(t)=a+t*(b-a), t in [0,1]. Minimize.
        let d1 = self.dir;
        let d2 = b.sub(a);
        let r = self.origin.sub(a);
        let aa = d1.dot(d1);
        let bb = d1.dot(d2);
        let cc = d2.dot(d2);
        let dd = d1.dot(r);
        let ee = d2.dot(r);
        let denom = aa * cc - bb * bb;
        let (mut s, mut t) = if denom.abs() < 1e-9 {
            (0.0, (ee / cc).clamp(0.0, 1.0))
        } else {
            let s = ((bb * ee - cc * dd) / denom).max(0.0);
            let t = ((aa * ee - bb * dd) / denom).clamp(0.0, 1.0);
            (s, t)
        };
        // one refine of s for the clamped t
        s = ((d1.dot(a.add(d2.scale(t)).sub(self.origin))) / aa).max(0.0);
        let _ = &mut t;
        let pr = self.at(s);
        let ps = a.add(d2.scale(t));
        pr.sub(ps).length()
    }

    /// Intersection parameter with a plane (point `p0`, unit normal `n`); None
    /// if parallel.
    pub fn intersect_plane(&self, p0: Vec3, n: Vec3) -> Option<f32> {
        let denom = self.dir.dot(n);
        if denom.abs() < 1e-9 {
            return None;
        }
        Some(p0.sub(self.origin).dot(n) / denom)
    }
}

/// Column-major orthographic world-to-clip matrix (depth 0..1). `fwd` must
/// be normalized; the up hint is orthogonalized to preserve camera roll.
pub(crate) fn ortho_view_proj(eye: Vec3, fwd: Vec3, up_hint: Vec3, half: f32, w: f32, h: f32) -> [[f32; 4]; 4] {
    let right = fwd.cross(up_hint).normalized();
    let up = right.cross(fwd).normalized();
    let view = [
        [right.x, up.x, -fwd.x, 0.0],
        [right.y, up.y, -fwd.y, 0.0],
        [right.z, up.z, -fwd.z, 0.0],
        [-right.dot(eye), -up.dot(eye), fwd.dot(eye), 1.0],
    ];
    let aspect = w / h;
    let (l, r, b, t) = (-half * aspect, half * aspect, -half, half);
    let (near, far) = (0.01f32, 100.0f32);
    let ortho = [
        [2.0 / (r - l), 0.0, 0.0, 0.0],
        [0.0, 2.0 / (t - b), 0.0, 0.0],
        [0.0, 0.0, -1.0 / (far - near), 0.0],
        [-(r + l) / (r - l), -(t + b) / (t - b), -near / (far - near), 1.0],
    ];
    mat_mul4(&ortho, &view)
}

fn mat_mul4(a: &[[f32; 4]; 4], b: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for col in 0..4 {
        for row in 0..4 {
            let mut sum = 0.0;
            for k in 0..4 {
                sum += a[k][row] * b[col][k];
            }
            out[col][row] = sum;
        }
    }
    out
}
