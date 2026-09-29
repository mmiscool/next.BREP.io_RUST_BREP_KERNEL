//! Orthographic artifact framing and orbit-camera math.
//! Calculations use f64, converting to f32 at the GPU boundary.

use crate::geometry3d::{cross3 as cross, dot3 as dot, norm3 as norm};

/// Axis-aligned bounding box (world space, f64).
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Aabb {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

impl Aabb {
    pub fn empty() -> Self {
        Self {
            min: [f64::INFINITY; 3],
            max: [f64::NEG_INFINITY; 3],
        }
    }

    pub fn is_empty(&self) -> bool {
        (0..3).any(|i| self.min[i] > self.max[i])
    }

    pub fn expand(&mut self, p: [f64; 3]) {
        for i in 0..3 {
            self.min[i] = self.min[i].min(p[i]);
            self.max[i] = self.max[i].max(p[i]);
        }
    }

    pub fn union(&mut self, other: &Aabb) {
        if other.is_empty() {
            return;
        }
        self.expand(other.min);
        self.expand(other.max);
    }

    pub fn center(&self) -> [f64; 3] {
        [
            (self.min[0] + self.max[0]) * 0.5,
            (self.min[1] + self.max[1]) * 0.5,
            (self.min[2] + self.max[2]) * 0.5,
        ]
    }

    pub fn size(&self) -> [f64; 3] {
        [
            self.max[0] - self.min[0],
            self.max[1] - self.min[1],
            self.max[2] - self.min[2],
        ]
    }
}

/// A camera resolved to GPU form: column-major view-projection matrix plus the
/// world-space view direction (camera → scene) the shader needs for
/// double-sided normal flipping and specular.
#[derive(Debug, Clone, Copy)]
pub struct Camera {
    /// Column-major 4x4, world → wgpu clip space (z in 0..1).
    pub view_proj: [[f32; 4]; 4],
    /// Normalized world-space view direction, from the camera toward the scene.
    pub forward: [f32; 3],
}

/// Right-handed look-at view matrix (camera looks down its -Z), column-major.
fn look_at(eye: [f64; 3], target: [f64; 3], up: [f64; 3]) -> [[f64; 4]; 4] {
    let f = norm([target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]]);
    let r = norm(cross(f, up));
    let u = cross(r, f);
    // Columns of the view matrix (world → view).
    [
        [r[0], u[0], -f[0], 0.0],
        [r[1], u[1], -f[1], 0.0],
        [r[2], u[2], -f[2], 0.0],
        [-dot(r, eye), -dot(u, eye), dot(f, eye), 1.0],
    ]
}

/// Orthographic projection to wgpu clip space (x,y in -1..1, z in 0..1),
/// column-major. View-space z of visible points is in [-far, -near].
fn ortho(l: f64, r: f64, b: f64, t: f64, near: f64, far: f64) -> [[f64; 4]; 4] {
    let sx = 2.0 / (r - l);
    let sy = 2.0 / (t - b);
    let sz = -1.0 / (far - near);
    [
        [sx, 0.0, 0.0, 0.0],
        [0.0, sy, 0.0, 0.0],
        [0.0, 0.0, sz, 0.0],
        [
            -(r + l) / (r - l),
            -(t + b) / (t - b),
            -near / (far - near),
            1.0,
        ],
    ]
}

fn mul(a: [[f64; 4]; 4], b: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut out = [[0.0; 4]; 4];
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

fn to_f32(m: [[f64; 4]; 4]) -> [[f32; 4]; 4] {
    let mut out = [[0.0f32; 4]; 4];
    for c in 0..4 {
        for r in 0..4 {
            out[c][r] = m[c][r] as f32;
        }
    }
    out
}

/// Build a camera from an explicit eye/target/up + ortho frustum (the shared
/// path for the artifact framing and the desktop orbit).
pub fn ortho_camera(
    eye: [f64; 3],
    target: [f64; 3],
    up: [f64; 3],
    half_width: f64,
    half_height: f64,
    near: f64,
    far: f64,
) -> Camera {
    let view = look_at(eye, target, up);
    let proj = ortho(-half_width, half_width, -half_height, half_height, near, far);
    let forward = norm([
        target[0] - eye[0],
        target[1] - eye[1],
        target[2] - eye[2],
    ]);
    Camera {
        view_proj: to_f32(mul(proj, view)),
        forward: [forward[0] as f32, forward[1] as f32, forward[2] as f32],
    }
}

/// The artifact framing — a faithful port of the retired artifact renderer:
/// orthographic, iso view direction (1, -1.2, 0.9), Z up, bbox-fit with a 1.15
/// margin, camera at center + dir·radius·6, near 0.01 / far radius·20.
///
/// The Z up is that retired renderer's own convention, kept because this is a
/// port of it — NOT the application's world, which is +Y up (`ViewCamera`, the
/// ViewCube, the kernel feature pipeline's `world_up`). A model authored in the
/// app is therefore framed rolled here, not standing the way the viewport draws
/// it. Nothing in the app's viewport path calls this: it is the offline
/// `bin/artifact` renderer's framing, and the render tests'.
pub fn artifact_camera(bbox: &Aabb, width: u32, height: u32) -> Camera {
    let bbox = if bbox.is_empty() {
        Aabb {
            min: [-1.0, -1.0, -1.0],
            max: [1.0, 1.0, 1.0],
        }
    } else {
        *bbox
    };
    let center = bbox.center();
    let size = bbox.size();
    let radius = size[0].max(size[1]).max(size[2]).max(1e-9) * 0.75;
    let dir = norm([1.0, -1.2, 0.9]);
    let eye = [
        center[0] + dir[0] * radius * 6.0,
        center[1] + dir[1] * radius * 6.0,
        center[2] + dir[2] * radius * 6.0,
    ];
    let aspect = width.max(1) as f64 / height.max(1) as f64;
    ortho_camera(
        eye,
        center,
        [0.0, 0.0, 1.0],
        radius * aspect * 1.15,
        radius * 1.15,
        0.01,
        radius * 20.0,
    )
}

/// Simple Z-up orbit state matching [`artifact_camera`]'s framing: azimuth /
/// elevation around the scene bbox, distance-scaled ortho frustum (zoom =
/// frustum scale).
///
/// Z up for the same reason [`artifact_camera`] is, and unused: the shell's
/// viewport camera is `crate::view::ViewCamera` — a free arcball, +Y up — and
/// nothing outside this module constructs an `Orbit`.
#[derive(Debug, Clone, Copy)]
pub struct Orbit {
    pub azimuth: f64,
    pub elevation: f64,
    pub zoom: f64,
}

impl Default for Orbit {
    fn default() -> Self {
        // Match the artifact iso direction: dir (1, -1.2, 0.9).
        let dir = norm([1.0, -1.2, 0.9]);
        Self {
            azimuth: dir[1].atan2(dir[0]),
            elevation: dir[2].asin(),
            zoom: 1.0,
        }
    }
}

impl Orbit {
    pub fn camera(&self, bbox: &Aabb, width: u32, height: u32) -> Camera {
        let bbox = if bbox.is_empty() {
            Aabb {
                min: [-1.0, -1.0, -1.0],
                max: [1.0, 1.0, 1.0],
            }
        } else {
            *bbox
        };
        let center = bbox.center();
        let size = bbox.size();
        let radius = size[0].max(size[1]).max(size[2]).max(1e-9) * 0.75;
        let el = self.elevation.clamp(-1.55, 1.55);
        let dir = [
            el.cos() * self.azimuth.cos(),
            el.cos() * self.azimuth.sin(),
            el.sin(),
        ];
        let eye = [
            center[0] + dir[0] * radius * 6.0,
            center[1] + dir[1] * radius * 6.0,
            center[2] + dir[2] * radius * 6.0,
        ];
        let aspect = width.max(1) as f64 / height.max(1) as f64;
        let half_h = radius * 1.15 * self.zoom;
        ortho_camera(
            eye,
            center,
            [0.0, 0.0, 1.0],
            half_h * aspect,
            half_h,
            0.01,
            radius * 20.0,
        )
    }
}

