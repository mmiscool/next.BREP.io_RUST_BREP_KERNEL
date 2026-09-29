//! Interactive orthographic and perspective cameras with zoom-to-fit,
//! depth-range fitting, and world/screen conversion shared by native and web.
//!
//! Screen coordinates are CSS pixels: origin at the top left, y increasing down.
//! Device pixel ratio affects surface sizing, not camera math.

use crate::camera::{Aabb, Camera};

pub use crate::geometry3d::{add3, cross3, dot3, len3, norm3, rotate3, scale3, sub3};

/// Inverse of a column-major 4×4 matrix (index = `col*4 + row`
/// layout). Returns `None` if singular. Used to invert the view-projection for
/// the host overlays' screen→world path.
pub fn invert4_columns(m: &[f64; 16]) -> Option<[f64; 16]> {
    let a00 = m[0]; let a01 = m[1]; let a02 = m[2]; let a03 = m[3];
    let a10 = m[4]; let a11 = m[5]; let a12 = m[6]; let a13 = m[7];
    let a20 = m[8]; let a21 = m[9]; let a22 = m[10]; let a23 = m[11];
    let a30 = m[12]; let a31 = m[13]; let a32 = m[14]; let a33 = m[15];

    let b00 = a00 * a11 - a01 * a10;
    let b01 = a00 * a12 - a02 * a10;
    let b02 = a00 * a13 - a03 * a10;
    let b03 = a01 * a12 - a02 * a11;
    let b04 = a01 * a13 - a03 * a11;
    let b05 = a02 * a13 - a03 * a12;
    let b06 = a20 * a31 - a21 * a30;
    let b07 = a20 * a32 - a22 * a30;
    let b08 = a20 * a33 - a23 * a30;
    let b09 = a21 * a32 - a22 * a31;
    let b10 = a21 * a33 - a23 * a31;
    let b11 = a22 * a33 - a23 * a32;

    let det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;
    if det.abs() < 1e-300 {
        return None;
    }
    let inv = 1.0 / det;
    Some([
        (a11 * b11 - a12 * b10 + a13 * b09) * inv,
        (a02 * b10 - a01 * b11 - a03 * b09) * inv,
        (a31 * b05 - a32 * b04 + a33 * b03) * inv,
        (a22 * b04 - a21 * b05 - a23 * b03) * inv,
        (a12 * b08 - a10 * b11 - a13 * b07) * inv,
        (a00 * b11 - a02 * b08 + a03 * b07) * inv,
        (a32 * b02 - a30 * b05 - a33 * b01) * inv,
        (a20 * b05 - a22 * b02 + a23 * b01) * inv,
        (a10 * b10 - a11 * b08 + a13 * b06) * inv,
        (a01 * b08 - a00 * b10 - a03 * b06) * inv,
        (a30 * b04 - a31 * b02 + a33 * b00) * inv,
        (a21 * b02 - a20 * b04 - a23 * b00) * inv,
        (a11 * b07 - a10 * b09 - a12 * b06) * inv,
        (a00 * b09 - a01 * b07 + a02 * b06) * inv,
        (a31 * b01 - a30 * b03 - a32 * b00) * inv,
        (a20 * b03 - a21 * b01 + a22 * b00) * inv,
    ])
}

/// Which way the NAMED views face: the frame the standard-view table and the
/// ViewCube's labelled faces are read in, carried into the world by a rotation.
///
/// The table is written once, Y-up (FRONT on +Z, TOP on +Y), which is how a
/// modelled part is drawn. A PCB is not: the board is built Z-up — F.Cu's plane
/// at z = 0, the parts standing on +Z, the layout's +Y-down read as world −Y
/// (`engine_state::board_geometry`). Read in the Y-up frame, TOP shows the
/// board's 1.6 mm edge and FRONT its component side (eCAD third audit, B5).
///
/// The board is NOT turned to suit the viewer, because its world coordinates
/// are what STEP export writes, what the parts' placements are written in and
/// what `locate` reports; it is the names that turn. So a view is a direction
/// in the frame, and [`Self::to_world`] is the one place the frame is applied —
/// by the standard views and by the ViewCube alike, so the cube's T face is the
/// one the TOP view looks at.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ViewFrame {
    /// Y-up: the world itself. Every document without a board.
    #[default]
    World,
    /// Z-up, for a document that shows a board: frame +X → world +X (the
    /// layout's right), frame +Y (TOP) → world +Z (the component side), frame
    /// +Z (FRONT) → world −Y (the layout's bottom edge, the one nearest the
    /// reader in the 2D board view). A +90° turn about X, so it is a proper
    /// rotation and the ViewCube's letters stay unmirrored.
    ///
    /// TOP then looks down with world +Y up, which is the 2D board view's own
    /// orientation. BOTTOM is the Y-up table's BOTTOM carried over (up = frame
    /// +Z = world −Y): the physical underside, as if the board were tipped
    /// towards the reader over its front edge, NOT the left-right mirror a 2D
    /// editor's "flip board" shows.
    Board,
}

impl ViewFrame {
    /// A frame direction in world coordinates.
    pub fn to_world(self, v: [f64; 3]) -> [f64; 3] {
        match self {
            ViewFrame::World => v,
            ViewFrame::Board => [v[0], -v[2], v[1]],
        }
    }

    /// A world direction in frame coordinates — the inverse of [`Self::to_world`].
    pub fn from_world(self, v: [f64; 3]) -> [f64; 3] {
        match self {
            ViewFrame::World => v,
            ViewFrame::Board => [v[0], v[2], -v[1]],
        }
    }
}

/// The projection kind (R21): orthographic is the default; the toggle keeps the
/// apparent size at the target plane.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Projection {
    /// `half_height` is half the vertical world span at the target plane.
    Orthographic { half_height: f64 },
    Perspective { fov_y_deg: f64 },
}

/// A world-space ray for picking.
#[derive(Debug, Clone, Copy)]
pub struct Ray {
    pub origin: [f64; 3],
    pub dir: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct ViewCamera {
    pub eye: [f64; 3],
    pub target: [f64; 3],
    pub up: [f64; 3],
    pub projection: Projection,
    /// Viewport CSS size.
    pub width: f64,
    pub height: f64,
    /// View-space depth window (positive distances along the view direction);
    /// maintained by [`ViewCamera::fit_depth_range`]. Ortho near may go
    /// negative (scene behind the eye plane is still projectable).
    ///
    /// DEPTH-BUFFER WINDOW ONLY. `near`/`far` exist to map the GPU depth buffer
    /// over everything drawn (re-fitted each frame from the render path's
    /// depth bbox = `depth_range_bbox` ∪ the full widget overlay's world bounds
    /// ∪ the world origin, so construction geometry — datums / axes / gizmos —
    /// is always bracketed) — they are NEVER a visibility decision. No label, anchor, chip, or hit-region may consult them: the
    /// ONE screen-visibility rule for all of those is
    /// [`ViewCamera::projectable`]. Anything gating on `near`/`far` (or a bare
    /// `depth > 0` in ortho) is a bug — labels would vanish while their
    /// geometry still renders.
    pub near: f64,
    pub far: f64,
}

impl Default for ViewCamera {
    fn default() -> Self {
        // The retired viewer's startup vantage: eye (15,12,15) → origin, Y-up,
        // ortho half-height 10 ("viewSize").
        Self {
            eye: [15.0, 12.0, 15.0],
            target: [0.0, 0.0, 0.0],
            up: [0.0, 1.0, 0.0],
            projection: Projection::Orthographic { half_height: 10.0 },
            width: 800.0,
            height: 600.0,
            near: -100000.0,
            far: 100000.0,
        }
    }
}

impl ViewCamera {
    pub fn aspect(&self) -> f64 {
        (self.width / self.height.max(1.0)).max(1e-6)
    }

    /// Camera basis: (right, true-up, forward) with forward pointing INTO the
    /// scene (eye → target).
    pub fn basis(&self) -> ([f64; 3], [f64; 3], [f64; 3]) {
        let forward = norm3(sub3(self.target, self.eye));
        let right = norm3(cross3(forward, self.up));
        let up = cross3(right, forward);
        (right, up, forward)
    }

    pub fn distance(&self) -> f64 {
        len3(sub3(self.eye, self.target)).max(1e-9)
    }

    /// World units per CSS pixel at the target plane (R21/R25 — the query the
    /// pickers, gizmos and sketch glyph sizing key off).
    pub fn world_per_pixel(&self) -> f64 {
        match self.projection {
            Projection::Orthographic { half_height } => 2.0 * half_height / self.height.max(1.0),
            Projection::Perspective { fov_y_deg } => {
                let fov = fov_y_deg.to_radians();
                2.0 * (fov * 0.5).tan() * self.distance() / self.height.max(1.0)
            }
        }
    }

    /// The view-space depth of a world point: the signed distance along the
    /// forward view axis from the eye (identical to `project`'s third return).
    /// Positive in front of the eye plane. The screen-space region builder
    /// ([`brep_gizmos::hit_region`]) keys the perspective front-clip off this.
    pub fn view_depth(&self, world: [f64; 3]) -> f64 {
        let (_, _, forward) = self.basis();
        dot3(sub3(world, self.eye), forward)
    }

    /// Whether a world point is PROJECTABLE to a usable screen position — THE
    /// screen-visibility policy for every label / anchor / chip / hit-region
    /// consumer, in ONE place so no consumer can re-invent a depth cull:
    ///
    /// * ORTHOGRAPHIC (the app default): always `true`. Behind-eye-plane
    ///   geometry still renders in ortho, so its labels must too.
    /// * PERSPECTIVE: `false` only for a point at/behind the eye plane, where
    ///   the projection itself is mathematically undefined — the same
    ///   `FRONT_EPS` rule the region builder ([`brep_gizmos::hit_region`])
    ///   applies.
    ///
    /// The `near`/`far` fields NEVER factor in — they are the GPU depth-buffer
    /// window (see their field doc), not visibility. Route ANY new "should this
    /// world-anchored UI draw?" question through here.
    pub fn projectable(&self, world: [f64; 3]) -> bool {
        matches!(self.projection, Projection::Orthographic { .. })
            || self.view_depth(world) > 1e-6
    }

    /// Whether a world-anchored TEXT LABEL should draw: [`Self::projectable`]
    /// AND the anchor projects INSIDE the viewport rect. The second half is a
    /// screen-BOUNDS test, never a depth test — `near`/`far` still cull nothing
    /// (see [`Self::projectable`]) — so a chip whose 3D anchor scrolled out of
    /// view disappears instead of piling up clamped at the viewport edge (egui
    /// Areas constrain themselves on-screen). This is the `inFront` flag every
    /// app label pass keys its skip off (`world_to_screen_json`); hit-testable
    /// ANCHORS (gizmo handles, hit regions) intentionally stay on the pure
    /// `projectable` policy — an off-screen handle just can't be clicked.
    pub fn label_anchor_visible(&self, world: [f64; 3]) -> bool {
        if !self.projectable(world) {
            return false;
        }
        let (sx, sy, _) = self.project(world);
        sx >= 0.0 && sx <= self.width && sy >= 0.0 && sy <= self.height
    }

    /// The world→clip view-projection as column-major `[col][row]` in f64. This
    /// is the exact matrix [`resolve`] feeds the GPU, kept in f64 so the
    /// CSS-pixel projection the host overlays derive from it matches [`project`]
    /// to sub-pixel precision. wgpu clip space: x,y in −1..1, z in 0..1.
    pub fn view_proj_cols(&self) -> [[f64; 4]; 4] {
        let (right, up, forward) = self.basis();
        let half_h = match self.projection {
            Projection::Orthographic { half_height } => half_height,
            Projection::Perspective { fov_y_deg } => (fov_y_deg.to_radians() * 0.5).tan(),
        };
        let half_w = half_h * self.aspect();

        // View matrix rows from the basis (world → view; view looks down -Z).
        let ex = -dot3(right, self.eye);
        let ey = -dot3(up, self.eye);
        let ez = dot3(forward, self.eye);
        let view = [
            [right[0], up[0], -forward[0], 0.0],
            [right[1], up[1], -forward[1], 0.0],
            [right[2], up[2], -forward[2], 0.0],
            [ex, ey, ez, 1.0],
        ];

        let proj = match self.projection {
            Projection::Orthographic { .. } => {
                // wgpu clip space: z in 0..1.
                let sx = 1.0 / half_w;
                let sy = 1.0 / half_h;
                let sz = -1.0 / (self.far - self.near);
                [
                    [sx, 0.0, 0.0, 0.0],
                    [0.0, sy, 0.0, 0.0],
                    [0.0, 0.0, sz, 0.0],
                    [0.0, 0.0, -self.near / (self.far - self.near), 1.0],
                ]
            }
            Projection::Perspective { .. } => {
                // Finite-far wgpu perspective (z in 0..1, forward-Z: near→0,
                // far→1). TODO(depth): an INFINITE-FAR limit (col2 → [0,0,-1,-1],
                // col3 → [0,0,-near,0]) would stop anything clipping at `far` in
                // perspective, but is DEFERRED: the screen→ray unproject in
                // `GizmoCamera::ray_from_screen` (datum pick, transform drag,
                // ViewCube — all in `brep-gizmos`) reconstructs rays from NDC
                // z=0 AND z=1, and at z=1 the infinite-far inverse's w passes
                // through zero as the camera orbits → intermittently backward
                // pick rays. It is also redundant now: the render path folds the
                // FULL overlay (+origin) into the depth fit, so construction
                // geometry is bracketed regardless. Revisit alongside a
                // reversed-Z depth precision pass (would fix the unproject too).
                let near = self.near.max(1e-6);
                let far = self.far.max(near * 1.0001);
                let f = 1.0 / half_h;
                [
                    [f / self.aspect(), 0.0, 0.0, 0.0],
                    [0.0, f, 0.0, 0.0],
                    [0.0, 0.0, far / (near - far), -1.0],
                    [0.0, 0.0, near * far / (near - far), 0.0],
                ]
            }
        };

        let mut view_proj = [[0.0f64; 4]; 4];
        for col in 0..4 {
            for row in 0..4 {
                let mut sum = 0.0;
                for k in 0..4 {
                    sum += proj[k][row] * view[col][k];
                }
                view_proj[col][row] = sum;
            }
        }
        view_proj
    }

    /// Resolve to the GPU camera (column-major view-proj, f32).
    pub fn resolve(&self) -> Camera {
        let cols = self.view_proj_cols();
        let mut view_proj = [[0.0f32; 4]; 4];
        for col in 0..4 {
            for row in 0..4 {
                view_proj[col][row] = cols[col][row] as f32;
            }
        }
        let fwd = norm3(sub3(self.target, self.eye));
        Camera {
            view_proj,
            forward: [fwd[0] as f32, fwd[1] as f32, fwd[2] as f32],
        }
    }

    /// The view-projection flattened column-major (index = `col*4 + row`) — the
    /// world→clip matrix for the host overlays'
    /// per-frame world→screen hot path (dimensions + sketch), letting them drop
    /// the compat mirror camera. Pair with the CSS `viewport` for NDC→pixel.
    pub fn view_proj_flat(&self) -> [f64; 16] {
        let cols = self.view_proj_cols();
        let mut out = [0.0f64; 16];
        for col in 0..4 {
            for row in 0..4 {
                out[col * 4 + row] = cols[col][row];
            }
        }
        out
    }

    /// Inverse of [`view_proj_flat`] (clip→world), column-major, for the host
    /// overlays' screen→world / screen→ray path. Falls back to the identity if
    /// the matrix is singular (never in practice for a valid camera).
    pub fn view_proj_inverse_flat(&self) -> [f64; 16] {
        invert4_columns(&self.view_proj_flat())
            .unwrap_or([1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0])
    }

    /// Project a world point to CSS-pixel screen coordinates (origin top-left,
    /// y down). Returns `(x, y, view_depth)`; `view_depth` is the distance
    /// along the view direction (positive in front of the eye plane).
    pub fn project(&self, world: [f64; 3]) -> (f64, f64, f64) {
        let (right, up, forward) = self.basis();
        let rel = sub3(world, self.eye);
        let vx = dot3(rel, right);
        let vy = dot3(rel, up);
        let depth = dot3(rel, forward);
        match self.projection {
            Projection::Orthographic { half_height } => {
                let half_w = half_height * self.aspect();
                let sx = (vx / half_w * 0.5 + 0.5) * self.width;
                let sy = (0.5 - vy / half_height * 0.5) * self.height;
                (sx, sy, depth)
            }
            Projection::Perspective { fov_y_deg } => {
                let half_h = (fov_y_deg.to_radians() * 0.5).tan();
                let half_w = half_h * self.aspect();
                let d = depth.max(1e-9);
                let sx = (vx / (half_w * d) * 0.5 + 0.5) * self.width;
                let sy = (0.5 - vy / (half_h * d) * 0.5) * self.height;
                (sx, sy, depth)
            }
        }
    }

    /// A world-space picking ray through CSS-pixel `(x, y)`. Ortho rays start
    /// far behind the eye plane so huge scenes are always in front (the retired
    /// picker pushed its ray origin back the same way).
    pub fn pick_ray(&self, x: f64, y: f64) -> Ray {
        let (right, up, forward) = self.basis();
        let ndc_x = (x / self.width.max(1.0)) * 2.0 - 1.0;
        let ndc_y = -((y / self.height.max(1.0)) * 2.0 - 1.0);
        match self.projection {
            Projection::Orthographic { half_height } => {
                let half_w = half_height * self.aspect();
                let span = self.far.abs().max(self.near.abs()).max(half_height * 40.0).max(1.0);
                let on_plane = add3(
                    self.eye,
                    add3(scale3(right, ndc_x * half_w), scale3(up, ndc_y * half_height)),
                );
                Ray {
                    origin: sub3(on_plane, scale3(forward, span)),
                    dir: forward,
                }
            }
            Projection::Perspective { fov_y_deg } => {
                let half_h = (fov_y_deg.to_radians() * 0.5).tan();
                let half_w = half_h * self.aspect();
                let dir = norm3(add3(
                    forward,
                    add3(scale3(right, ndc_x * half_w), scale3(up, ndc_y * half_h)),
                ));
                Ray {
                    origin: self.eye,
                    dir,
                }
            }
        }
    }

    /// Fit the depth window to the scene (the `_updateDepthRange` port): the
    /// whole bbox lands inside `[near, far]` with generous padding.
    pub fn fit_depth_range(&mut self, bbox: &Aabb) {
        if bbox.is_empty() {
            // An EMPTY input (no solids, no overlay geometry) must NOT ride the
            // stale near/far from an earlier populated frame — that stale-tight
            // window would CLIP a newly-shown construction-only scene (datum
            // planes / world axes / gizmos). Reset to the SAME generous window
            // the camera constructs with (see `Default`: ortho ±100000;
            // perspective a sane 0.1 / 1e5) so an empty scene never clips. This
            // is a depth-WINDOW choice only — near/far are the depth-buffer
            // range, never a visibility decision (see their field doc).
            match self.projection {
                Projection::Orthographic { .. } => {
                    self.near = -100000.0;
                    self.far = 100000.0;
                }
                Projection::Perspective { .. } => {
                    self.near = 0.1;
                    self.far = 1e5;
                }
            }
            return;
        }
        let (_, _, forward) = self.basis();
        let mut min_d = f64::INFINITY;
        let mut max_d = f64::NEG_INFINITY;
        for i in 0..8 {
            let corner = [
                if i & 1 == 0 { bbox.min[0] } else { bbox.max[0] },
                if i & 2 == 0 { bbox.min[1] } else { bbox.max[1] },
                if i & 4 == 0 { bbox.min[2] } else { bbox.max[2] },
            ];
            let d = dot3(sub3(corner, self.eye), forward);
            min_d = min_d.min(d);
            max_d = max_d.max(d);
        }
        let diag = len3(sub3(bbox.max, bbox.min));
        let pad = ((max_d - min_d) * 0.1).max(diag * 0.1).max(0.5);
        match self.projection {
            Projection::Orthographic { .. } => {
                self.near = min_d - pad;
                self.far = max_d + pad;
            }
            Projection::Perspective { .. } => {
                let far = (max_d + pad).max(1.0);
                self.near = (far * 0.001).clamp(1e-4, 1.0).min((min_d - pad).max(1e-4));
                self.far = far;
            }
        }
    }

    /// Zoom-to-fit (R21): recenters the target on the bbox and scales the
    /// frustum/distance so the whole bbox fits with `margin`, preserving the
    /// view direction (the ArcballControls `focus` behavior).
    pub fn zoom_to_fit(&mut self, bbox: &Aabb, margin: f64) {
        if bbox.is_empty() {
            return;
        }
        let margin = margin.max(1.0);
        let (right, up, forward) = self.basis();
        let center = bbox.center();
        let mut half_w = 0.0f64;
        let mut half_h = 0.0f64;
        for i in 0..8 {
            let corner = [
                if i & 1 == 0 { bbox.min[0] } else { bbox.max[0] },
                if i & 2 == 0 { bbox.min[1] } else { bbox.max[1] },
                if i & 4 == 0 { bbox.min[2] } else { bbox.max[2] },
            ];
            let rel = sub3(corner, center);
            half_w = half_w.max(dot3(rel, right).abs());
            half_h = half_h.max(dot3(rel, up).abs());
        }
        half_w = (half_w * margin).max(1e-6);
        half_h = (half_h * margin).max(1e-6);

        let dist = self.distance();
        let aspect = self.aspect();
        self.target = center;
        match self.projection {
            Projection::Orthographic { ref mut half_height } => {
                *half_height = half_h.max(half_w / aspect);
                self.eye = sub3(center, scale3(forward, dist));
            }
            Projection::Perspective { fov_y_deg } => {
                let fov = fov_y_deg.to_radians();
                let dist_h = half_h / (fov * 0.5).tan().max(1e-6);
                let tan_half_h_fov = (fov * 0.5).tan() * aspect;
                let dist_w = half_w / tan_half_h_fov.max(1e-6);
                let target_dist = dist_h.max(dist_w).max(1e-3);
                self.eye = sub3(center, scale3(forward, target_dist));
            }
        }
        self.fit_depth_range(bbox);
    }

    /// Toggle ortho ↔ perspective preserving the apparent size at the target
    /// plane (the `toggleCameraProjection` port). Returns the new kind name.
    pub fn toggle_projection(&mut self) -> &'static str {
        const FOV: f64 = 50.0;
        let forward = norm3(sub3(self.target, self.eye));
        match self.projection {
            Projection::Orthographic { half_height } => {
                let denom = (FOV.to_radians() * 0.5).tan();
                let mut distance = half_height / denom.max(1e-9);
                if !distance.is_finite() || distance < 1e-4 {
                    distance = 10.0;
                }
                self.eye = sub3(self.target, scale3(forward, distance));
                self.projection = Projection::Perspective { fov_y_deg: FOV };
                "perspective"
            }
            Projection::Perspective { fov_y_deg } => {
                let dist = self.distance();
                let half_height = ((fov_y_deg.to_radians() * 0.5).tan() * dist).max(1e-6);
                self.projection = Projection::Orthographic { half_height };
                "orthographic"
            }
        }
    }

    /// Snap to a standard view (future ViewCube seam), preserving distance and
    /// frustum scale. Directions are world-axis views with sensible ups, named
    /// in the Y-up [`ViewFrame::World`] frame — see [`Self::standard_view_in`].
    pub fn standard_view(&mut self, name: &str) -> bool {
        self.standard_view_in(ViewFrame::World, name)
    }

    /// [`Self::standard_view`] with the names read in `frame`: the table below
    /// is the Y-up one, and the frame carries each direction and up into the
    /// world, so under [`ViewFrame::Board`] TOP looks down on +Z.
    pub fn standard_view_in(&mut self, frame: ViewFrame, name: &str) -> bool {
        let dist = self.distance();
        let iso = norm3([1.0, 1.0, 1.0]);
        let (dir, up): ([f64; 3], [f64; 3]) = match name.to_ascii_uppercase().as_str() {
            "FRONT" => ([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
            "BACK" => ([0.0, 0.0, -1.0], [0.0, 1.0, 0.0]),
            "RIGHT" => ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            "LEFT" => ([-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
            "TOP" => ([0.0, 1.0, 0.0], [0.0, 0.0, -1.0]),
            "BOTTOM" => ([0.0, -1.0, 0.0], [0.0, 0.0, 1.0]),
            "ISO" => (iso, [0.0, 1.0, 0.0]),
            _ => return false,
        };
        self.eye = add3(self.target, scale3(frame.to_world(dir), dist));
        self.up = frame.to_world(up);
        true
    }

    /// Serialize the full camera state (R3: the host holds plain JSON only).
    pub fn state_json(&self) -> String {
        let (kind, scale) = match self.projection {
            Projection::Orthographic { half_height } => ("orthographic", half_height),
            Projection::Perspective { fov_y_deg } => ("perspective", fov_y_deg),
        };
        serde_json::json!({
            "kind": kind,
            "eye": self.eye,
            "target": self.target,
            "up": self.up,
            // half_height for ortho, fov_y_deg for perspective.
            "scale": scale,
            "near": self.near,
            "far": self.far,
            "width": self.width,
            "height": self.height,
            "worldPerPixel": self.world_per_pixel(),
        })
        .to_string()
    }

    /// Restore from [`ViewCamera::state_json`] output (viewport size is NOT
    /// restored — it belongs to the canvas).
    pub fn apply_state_json(&mut self, json: &str) -> Result<(), String> {
        let value: serde_json::Value =
            serde_json::from_str(json).map_err(|error| format!("camera state parse: {error}"))?;
        let vec3 = |key: &str| -> Option<[f64; 3]> {
            let arr = value.get(key)?.as_array()?;
            Some([arr.first()?.as_f64()?, arr.get(1)?.as_f64()?, arr.get(2)?.as_f64()?])
        };
        if let Some(eye) = vec3("eye") {
            self.eye = eye;
        }
        if let Some(target) = vec3("target") {
            self.target = target;
        }
        if let Some(up) = vec3("up") {
            self.up = up;
        }
        let scale = value.get("scale").and_then(|v| v.as_f64());
        match value.get("kind").and_then(|v| v.as_str()) {
            Some("perspective") => {
                self.projection = Projection::Perspective {
                    fov_y_deg: scale.unwrap_or(50.0),
                }
            }
            Some("orthographic") => {
                self.projection = Projection::Orthographic {
                    half_height: scale.unwrap_or(10.0).max(1e-9),
                }
            }
            _ => {}
        }
        if let Some(near) = value.get("near").and_then(|v| v.as_f64()) {
            self.near = near;
        }
        if let Some(far) = value.get("far").and_then(|v| v.as_f64()) {
            self.far = far;
        }
        Ok(())
    }
}

/// The render camera projects dimension-gizmo handle points to viewport-local px
/// for the shared screen-space region builder, so a gizmo's hit-test + its debug
/// outline share ONE projection (see [`brep_gizmos::hit_region`]).
impl brep_gizmos::hit_region::RegionCamera for ViewCamera {
    fn is_orthographic(&self) -> bool {
        matches!(self.projection, Projection::Orthographic { .. })
    }
    fn depth(&self, p: [f64; 3]) -> f64 {
        self.view_depth(p)
    }
    fn project_px(&self, p: [f64; 3]) -> Option<[f32; 2]> {
        // ONE policy: [`ViewCamera::projectable`] (ortho always projects;
        // perspective omits only at/behind the eye plane).
        if !self.projectable(p) {
            return None;
        }
        let (sx, sy, _) = self.project(p);
        Some([sx as f32, sy as f32])
    }
}

