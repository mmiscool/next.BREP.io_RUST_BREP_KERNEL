//! In-scene gizmos and overlay widgets — the shared CONTRACT every
//! gizmo builds against, plus the math it needs and a CPU rasterizer for
//! headless demos.
//!
//! Design: a gizmo is pure geometry + hit-testing. It consumes a [`GizmoCamera`]
//! (the render engine's view state, mirrored here with no wgpu/kernel dep) and
//! emits an [`Overlay`] of colored line segments and triangles in WORLD space,
//! which the render engine's overlay pass draws over the shaded solids. Where a
//! gizmo is interactive it also answers [`Gizmo::hit`] (which handle is under a
//! screen point) and, for draggables, a drag lifecycle in its own frame.
//!
//! The crate has no renderer or kernel dependency. The render engine converts
//! [`Overlay`] geometry into its overlay vertex buffers.
//!
//! Conventions (shared with the engine):
//! - Right-handed world space, **+Y up** — the application's world (the kernel
//!   feature pipeline's `world_up = (0, 1, 0)`, `ViewCamera`'s default and its
//!   standard views, the ViewCube's ±Y TOP/BOTTOM). Gizmo geometry is world
//!   space and takes its orientation from the [`GizmoCamera`] it is handed, so
//!   nothing here depends on the world up except a builder's own default (the
//!   +Z helix axis) and the headless demo cameras of [`raster::test_view_proj`],
//!   which frame Z-up scenes of their own.
//! - Column-major 4x4 matrices, world → wgpu clip (z in 0..1) — byte-identical
//!   to `brep-render`'s `Camera::view_proj`.
//! - Screen: origin top-left, x right, y DOWN, in CSS pixels (device-pixel
//!   ratio handled by the caller when it forwards viewport size).
//! - Screen-constant gizmo sizing uses [`GizmoCamera::world_per_pixel`].

pub mod hit_region;
pub mod math;
pub mod raster;

// Gizmo / overlay-widget modules (each built against the contract above).
pub mod curve_display;
pub mod datum;
pub mod dimension;
pub mod transform;
pub mod view_cube;

pub use math::{Ray, Vec3};

/// The engine camera state a gizmo needs, mirrored with no engine dependency.
/// Construct from the engine's live camera each frame at integration time.
#[derive(Debug, Clone, Copy)]
pub struct GizmoCamera {
    /// Column-major world→clip (z in 0..1), identical to the engine's
    /// `Camera::view_proj`.
    pub view_proj: [[f32; 4]; 4],
    /// Camera eye in world space.
    pub eye: Vec3,
    /// Normalized world-space view direction (eye → scene).
    pub forward: Vec3,
    /// The camera's TRUE world-space up (the second basis vector its view
    /// matrix is built from). The engine's camera is a free arcball — its up
    /// rolls arbitrarily — so widgets that must mirror the camera's exact
    /// orientation (the ViewCube) consume THIS, never a forward-derived
    /// heuristic. Ideally unit and perpendicular to `forward`; consumers
    /// re-orthonormalize defensively.
    pub up: Vec3,
    /// Viewport size in CSS pixels (width, height).
    pub viewport: [f32; 2],
    /// True for an orthographic projection (world_per_pixel is then constant
    /// in depth); false for perspective.
    pub orthographic: bool,
}

impl GizmoCamera {
    /// Project a world point to screen pixels (top-left origin, y down).
    /// Returns `None` when the point is behind the camera / on the clip plane.
    pub fn world_to_screen(&self, p: Vec3) -> Option<[f32; 2]> {
        let clip = mat_mul_point(&self.view_proj, p);
        let w = clip[3];
        if w <= 1e-6 {
            return None;
        }
        let ndc_x = clip[0] / w;
        let ndc_y = clip[1] / w;
        Some([
            (ndc_x * 0.5 + 0.5) * self.viewport[0],
            (0.5 - ndc_y * 0.5) * self.viewport[1],
        ])
    }

    /// World distance that projects to one CSS pixel at `at` (screen-constant
    /// sizing — a gizmo scales handles so they stay a fixed pixel size). For an
    /// orthographic camera this is independent of `at`.
    pub fn world_per_pixel(&self, at: Vec3) -> f32 {
        // Numerically: how far in world space (perpendicular to view) maps to
        // one pixel of NDC-to-screen. Nudge `at` by a small world delta along a
        // screen-horizontal axis and measure the screen displacement.
        let right = self.screen_right(at);
        let base = match self.world_to_screen(at) {
            Some(s) => s,
            None => return 1.0,
        };
        let delta = 1.0_f32; // 1 world unit probe
        let moved = match self.world_to_screen(at.add(right.scale(delta))) {
            Some(s) => s,
            None => return 1.0,
        };
        let px = ((moved[0] - base[0]).powi(2) + (moved[1] - base[1]).powi(2)).sqrt();
        if px <= 1e-6 {
            1.0
        } else {
            delta / px
        }
    }

    /// The view-space depth of a world point: the signed distance along the
    /// forward view axis from the eye. Positive in front of the eye plane. The
    /// screen-space region builder ([`crate::hit_region`]) keys the perspective
    /// front-clip off this — an orthographic camera renders behind-eye-plane
    /// geometry, so it is never clipped there.
    pub fn view_depth(&self, p: Vec3) -> f32 {
        p.sub(self.eye).dot(self.forward)
    }

    /// A world-space axis that is horizontal on screen at `at` (view right).
    pub fn screen_right(&self, _at: Vec3) -> Vec3 {
        // View right = normalize(cross(forward, up)) against the camera's OWN
        // up. A world axis cannot stand in for it: the engine's camera is a free
        // arcball whose up rolls (and the app's world up is +Y, not the +Z this
        // once assumed), so a fixed guess returns an axis that is not screen
        // horizontal and rolls every screen-constant quad built on it.
        let right = self.forward.cross(self.up);
        if right.length() < 1e-6 {
            // A degenerate up (parallel to forward, or never set): any
            // perpendicular still gives the caller a camera-facing axis.
            return self.forward.any_perp();
        }
        right.normalized()
    }

    /// A pick ray from a screen point (top-left origin, y down) into the scene.
    /// Perspective: origin at eye. Orthographic: origin on the near plane at the
    /// pixel, direction = forward. Requires the inverse view_proj, computed here.
    pub fn ray_from_screen(&self, x: f32, y: f32) -> Ray {
        let ndc_x = (x / self.viewport[0]) * 2.0 - 1.0;
        let ndc_y = 1.0 - (y / self.viewport[1]) * 2.0;
        let inv = mat_inverse(&self.view_proj);
        // Unproject near (z=0) and far (z=1) clip points.
        let near = mat_unproject(&inv, ndc_x, ndc_y, 0.0);
        let far = mat_unproject(&inv, ndc_x, ndc_y, 1.0);
        let dir = far.sub(near).normalized();
        if self.orthographic {
            Ray { origin: near, dir }
        } else {
            Ray {
                origin: self.eye,
                dir,
            }
        }
    }
}

/// The gizmo camera projects handle points to viewport-local px for the shared
/// screen-space region builder, so a gizmo's hit-test + its debug outline share
/// ONE projection (see [`crate::hit_region`]).
impl crate::hit_region::RegionCamera for GizmoCamera {
    fn is_orthographic(&self) -> bool {
        self.orthographic
    }
    fn depth(&self, p: [f64; 3]) -> f64 {
        self.view_depth(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32)) as f64
    }
    fn project_px(&self, p: [f64; 3]) -> Option<[f32; 2]> {
        self.world_to_screen(Vec3::new(p[0] as f32, p[1] as f32, p[2] as f32))
    }
}

/// One line-segment vertex in the overlay (world position + linear-space RGBA).
#[derive(Debug, Clone, Copy)]
pub struct LineVertex {
    pub pos: [f32; 3],
    pub color: [f32; 4],
}

/// One triangle vertex in the overlay (world position, normal, linear RGBA).
#[derive(Debug, Clone, Copy)]
pub struct TriVertex {
    pub pos: [f32; 3],
    pub normal: [f32; 3],
    pub color: [f32; 4],
}

/// Accumulated overlay geometry a gizmo emits for the engine's overlay pass.
/// `lines` are screen-constant-width segments (pairs); `tris` are shaded/flat
/// triangles (triples). Both are WORLD space.
#[derive(Debug, Clone, Default)]
pub struct Overlay {
    pub lines: Vec<LineVertex>,
    pub tris: Vec<TriVertex>,
}

impl Overlay {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append a colored world-space segment.
    pub fn line(&mut self, a: Vec3, b: Vec3, color: [f32; 4]) {
        self.lines.push(LineVertex {
            pos: a.into(),
            color,
        });
        self.lines.push(LineVertex {
            pos: b.into(),
            color,
        });
    }

    /// Append a flat-colored world-space triangle (normal auto-computed).
    pub fn tri(&mut self, a: Vec3, b: Vec3, c: Vec3, color: [f32; 4]) {
        let n = b.sub(a).cross(c.sub(a)).normalized();
        for p in [a, b, c] {
            self.tris.push(TriVertex {
                pos: p.into(),
                normal: n.into(),
                color,
            });
        }
    }

    pub fn extend(&mut self, other: &Overlay) {
        self.lines.extend_from_slice(&other.lines);
        self.tris.extend_from_slice(&other.tris);
    }
}

/// A gizmo handle id — an opaque token a gizmo returns from `hit` and the host
/// interaction layer echoes back to start a drag. `0` conventionally means
/// "the body / no specific handle".
pub type HandleId = u32;

/// The contract every gizmo implements.
pub trait Gizmo {
    /// Emit this gizmo's overlay geometry for the given camera. `hovered` is the
    /// handle currently under the pointer (for highlight), `active` the handle
    /// being dragged (if any).
    fn geometry(&self, camera: &GizmoCamera, hovered: Option<HandleId>, active: Option<HandleId>)
        -> Overlay;

    /// Which handle (if any) is under `screen` (top-left origin, y down).
    fn hit(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId>;
}

// --- matrix helpers (column-major, matching the engine) --------------------

/// `m * [p.x, p.y, p.z, 1]` → `[x, y, z, w]` (column-major m).
pub fn mat_mul_point(m: &[[f32; 4]; 4], p: Vec3) -> [f32; 4] {
    let v = [p.x, p.y, p.z, 1.0];
    let mut out = [0.0f32; 4];
    for row in 0..4 {
        let mut sum = 0.0;
        for k in 0..4 {
            sum += m[k][row] * v[k];
        }
        out[row] = sum;
    }
    out
}

/// Unproject an NDC point at clip depth `z` through an inverse view_proj.
fn mat_unproject(inv: &[[f32; 4]; 4], ndc_x: f32, ndc_y: f32, z: f32) -> Vec3 {
    let clip = [ndc_x, ndc_y, z, 1.0];
    let mut out = [0.0f32; 4];
    for row in 0..4 {
        let mut sum = 0.0;
        for k in 0..4 {
            sum += inv[k][row] * clip[k];
        }
        out[row] = sum;
    }
    let w = if out[3].abs() < 1e-9 { 1.0 } else { out[3] };
    Vec3::new(out[0] / w, out[1] / w, out[2] / w)
}

/// General 4x4 inverse (column-major), via cofactors. Adequate for camera
/// matrices (well-conditioned); returns identity on a singular matrix.
pub fn mat_inverse(m: &[[f32; 4]; 4]) -> [[f32; 4]; 4] {
    // Flatten column-major into row-major indexing a[r][c] = m[c][r].
    let a = |r: usize, c: usize| m[c][r] as f64;
    let mut inv = [[0.0f64; 4]; 4];
    // Standard adjugate/determinant inverse.
    let m00 = a(0, 0); let m01 = a(0, 1); let m02 = a(0, 2); let m03 = a(0, 3);
    let m10 = a(1, 0); let m11 = a(1, 1); let m12 = a(1, 2); let m13 = a(1, 3);
    let m20 = a(2, 0); let m21 = a(2, 1); let m22 = a(2, 2); let m23 = a(2, 3);
    let m30 = a(3, 0); let m31 = a(3, 1); let m32 = a(3, 2); let m33 = a(3, 3);

    let b00 = m00 * m11 - m01 * m10;
    let b01 = m00 * m12 - m02 * m10;
    let b02 = m00 * m13 - m03 * m10;
    let b03 = m01 * m12 - m02 * m11;
    let b04 = m01 * m13 - m03 * m11;
    let b05 = m02 * m13 - m03 * m12;
    let b06 = m20 * m31 - m21 * m30;
    let b07 = m20 * m32 - m22 * m30;
    let b08 = m20 * m33 - m23 * m30;
    let b09 = m21 * m32 - m22 * m31;
    let b10 = m21 * m33 - m23 * m31;
    let b11 = m22 * m33 - m23 * m32;

    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if det.abs() < 1e-18 {
        return identity();
    }
    let inv_det = 1.0 / det;

    let r = [
        [
            (m11 * b11 - m12 * b10 + m13 * b09) * inv_det,
            (m02 * b10 - m01 * b11 - m03 * b09) * inv_det,
            (m31 * b05 - m32 * b04 + m33 * b03) * inv_det,
            (m22 * b04 - m21 * b05 - m23 * b03) * inv_det,
        ],
        [
            (m12 * b08 - m10 * b11 - m13 * b07) * inv_det,
            (m00 * b11 - m02 * b08 + m03 * b07) * inv_det,
            (m32 * b02 - m30 * b05 - m33 * b01) * inv_det,
            (m20 * b05 - m22 * b02 + m23 * b01) * inv_det,
        ],
        [
            (m10 * b10 - m11 * b08 + m13 * b06) * inv_det,
            (m01 * b08 - m00 * b10 - m03 * b06) * inv_det,
            (m30 * b04 - m31 * b02 + m33 * b00) * inv_det,
            (m21 * b02 - m20 * b04 - m23 * b00) * inv_det,
        ],
        [
            (m11 * b07 - m10 * b09 - m12 * b06) * inv_det,
            (m00 * b09 - m01 * b07 + m02 * b06) * inv_det,
            (m31 * b01 - m30 * b03 - m32 * b00) * inv_det,
            (m20 * b03 - m21 * b01 + m22 * b00) * inv_det,
        ],
    ];
    // r is row-major inverse; store back column-major (out[c][r]).
    for row in 0..4 {
        for col in 0..4 {
            inv[col][row] = r[row][col];
        }
    }
    let mut out = [[0.0f32; 4]; 4];
    for c in 0..4 {
        for rr in 0..4 {
            out[c][rr] = inv[c][rr] as f32;
        }
    }
    out
}

fn identity() -> [[f32; 4]; 4] {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

