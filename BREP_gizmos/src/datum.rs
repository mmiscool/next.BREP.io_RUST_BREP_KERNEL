//! Datum / plane / axis / coordinate-frame display builders — the engine
//! overlay geometry that replaces the retired `editorDisplay` bodies for
//! datum features (`PlaneFeature`,
//! `DatiumFeature`) and the world-axis helper
//! (`axisHelpers`).
//!
//! Each builder emits an [`Overlay`] (world-space colored triangles + lines)
//! that the render engine's overlay pass draws over the shaded solids. No
//! kernel/GPU dependency — pure geometry + a `Gizmo` hit-test impl.
//!
//! # What SceneMap data feeds each builder (integration note)
//!
//! The engine's datum/plane/axis display path is a mechanical map from the
//! resident Rust `SceneMap` (`rust-kernel .../feature_pipeline/mod.rs`) to these
//! functions — no legacy overlay bodies:
//!
//! - **PLANE / DATUM plane** — `SceneMap.frames: HashMap<String, Frame>` where
//!   `Frame { origin, x_axis, y_axis, z_axis }`. Call
//!   [`datum_plane`]`(frame.origin, frame.x_axis, frame.y_axis, extent, color)`.
//!   `extent` is the plane's display size in world units (the previous app
//!   rendered planes at a fixed 5×5 → extent `5.0`; a future
//!   sized plane passes its own extent). When the feature has no meaningful
//!   world extent, use [`datum_plane_screen`] for a screen-constant card.
//!
//! - **DATUM coordinate frame (triad)** — the same `Frame`. Call
//!   [`datum_frame`]`(frame.origin, frame.x_axis, frame.y_axis, frame.z_axis,
//!   screen_px, camera)`. This is the reusable X/Y/Z triad (red/green/blue,
//!   screen-constant) for a `DATUM` coordinate system and for the
//!   transform-gizmo center.
//!
//! - **Revolve / sweep axis, construction line** — `SceneMap.axes:
//!   HashMap<String, Axis>` where `Axis { point, direction }`. Call
//!   [`datum_axis`]`(axis.point, axis.direction, length, color)`. `length` is
//!   the world span to draw (e.g. the operand's bounding extent along the axis).
//!
//! - **World axes helper** (`__WORLD_AXES__`) — no scene data; call
//!   [`world_axes`]`(camera, length_px)` for the origin triad.
//!
//! For selection/highlight the engine wraps a datum in [`DatumPlane`] /
//! [`DatumAxis`] (each carries one body [`HandleId`]) and calls [`Gizmo::hit`]
//! to know when the pointer is over the datum.
//!
//! # Screen-constant sizing
//!
//! A "screen-constant" widget stays a fixed pixel size regardless of zoom.
//! We size it in world units as `camera.world_per_pixel(anchor) * pixels`,
//! where [`GizmoCamera::world_per_pixel`] returns the world distance that
//! projects to one CSS pixel at the anchor point. Because it scales inversely
//! with zoom, the projected geometry keeps a constant on-screen size. The
//! triad ([`datum_frame`] / [`world_axes`]) is always screen-constant; the
//! plane card is world-sized ([`datum_plane`]) or screen-constant
//! ([`datum_plane_screen`]) — the two ways the plane extent can be sourced.

use crate::{Gizmo, GizmoCamera, HandleId, Overlay, Vec3};

/// Datum X axis color (red), matching `DEFAULT_AXIS_COLORS.x` (`#ff4d4d`).
pub const AXIS_X_COLOR: [f32; 4] = [1.0, 0.302, 0.302, 1.0];
/// Datum Y axis color (green), matching `DEFAULT_AXIS_COLORS.y` (`#4dff4d`).
pub const AXIS_Y_COLOR: [f32; 4] = [0.302, 1.0, 0.302, 1.0];
/// Datum Z axis color (blue), matching `DEFAULT_AXIS_COLORS.z` (`#4d7dff`).
pub const AXIS_Z_COLOR: [f32; 4] = [0.302, 0.490, 1.0, 1.0];
/// Default datum-plane green, matching `CADmaterials.PLANE.BASE` (`#2eff2e`).
pub const PLANE_COLOR: [f32; 4] = [0.180, 1.0, 0.180, 1.0];

/// Default full screen extent (CSS px) for a screen-constant plane card.
pub const DEFAULT_PLANE_SCREEN_PX: f32 = 140.0;
/// Default triad axis screen length (CSS px), matching `DEFAULT_AXIS_HELPER_PX`.
pub const DEFAULT_FRAME_PX: f32 = 70.0;
/// Pointer pick tolerance (CSS px) for datum hit-testing.
pub const PICK_PX: f32 = 6.0;

// --- plane -----------------------------------------------------------------

/// A bounded datum plane sized in **world units**. `size` is the full edge
/// length (a `size × size` square centered on `origin`, spanning the plane's
/// `x_axis`/`y_axis`). Emits, in order: the two fill triangles (semi-transparent
/// — the fill color's alpha is reduced), the four border segments (the loop),
/// and a small right-angle corner marker that fixes the +x/+y orientation.
///
/// `x_axis`/`y_axis` are expected orthonormal in-plane axes (a `Frame`'s
/// `x_axis`/`y_axis`); they are used as-is (no re-orthonormalization).
pub fn datum_plane(
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    size: f32,
    color: [f32; 4],
) -> Overlay {
    datum_plane_half(origin, x_axis, y_axis, size * 0.5, color)
}

/// A bounded datum plane whose on-screen size is **constant** (`screen_px` full
/// edge length in CSS pixels) regardless of zoom, anchored at `origin`. Use
/// when the datum feature has no intrinsic world extent. Same geometry layout
/// as [`datum_plane`].
pub fn datum_plane_screen(
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    screen_px: f32,
    color: [f32; 4],
    camera: &GizmoCamera,
) -> Overlay {
    let half = camera.world_per_pixel(origin) * screen_px * 0.5;
    datum_plane_half(origin, x_axis, y_axis, half, color)
}

/// Shared plane builder: `half` is the half-edge length in world units.
fn datum_plane_half(
    origin: Vec3,
    x_axis: Vec3,
    y_axis: Vec3,
    half: f32,
    color: [f32; 4],
) -> Overlay {
    let mut ov = Overlay::new();
    let hx = x_axis.scale(half);
    let hy = y_axis.scale(half);
    // Corners, CCW in (x,y): 00 = (-x,-y), 10 = (+x,-y), 11 = (+x,+y), 01 = (-x,+y).
    let c00 = origin.sub(hx).sub(hy);
    let c10 = origin.add(hx).sub(hy);
    let c11 = origin.add(hx).add(hy);
    let c01 = origin.sub(hx).add(hy);

    // Semi-transparent fill (two triangles). Dim + low-alpha so the card reads
    // as a translucent overlay rather than an opaque face.
    let fill = with_alpha(scale_rgb(color, 0.55), (color[3] * 0.28).min(0.28));
    ov.tri(c00, c10, c11, fill);
    ov.tri(c00, c11, c01, fill);

    // Border loop (4 segments), pushed FIRST among the lines.
    let border = [color[0], color[1], color[2], 1.0];
    ov.line(c00, c10, border);
    ov.line(c10, c11, border);
    ov.line(c11, c01, border);
    ov.line(c01, c00, border);

    // Corner orientation marker: a small right-angle at the (-x,-y) corner,
    // ticking toward +x and +y so the plane's local frame is legible.
    let m = half * 0.28;
    ov.line(c00, c00.add(x_axis.scale(m)), border);
    ov.line(c00, c00.add(y_axis.scale(m)), border);

    ov
}

// --- axis ------------------------------------------------------------------

/// A datum axis: a line segment from `point` extending `length` world units
/// along `direction`, with a small cone arrowhead at the `+` end (the `direction`
/// end). For revolve/sweep axes and construction lines. The shaft is the first
/// (only) line segment; the arrowhead is emitted as triangles.
pub fn datum_axis(point: Vec3, direction: Vec3, length: f32, color: [f32; 4]) -> Overlay {
    let mut ov = Overlay::new();
    let d = direction.normalized();
    let end = point.add(d.scale(length));
    ov.line(point, end, color); // shaft (first line segment)
    let head = length * 0.14;
    push_cone(&mut ov, end, d, head, head * 0.45, color, 12);
    ov
}

// --- coordinate frame (triad) ----------------------------------------------

/// A coordinate-frame triad: three **screen-constant** colored axes from
/// `origin` (X red, Y green, Z blue), each `screen_len_px` CSS pixels long, plus
/// a small origin marker (an octahedron "diamond"). The reusable triad for a
/// DATUM coordinate system and the transform-gizmo center.
///
/// The three axis shafts are the first three line segments (in X, Y, Z order);
/// arrowheads and the origin marker follow as triangles.
pub fn datum_frame(
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    screen_len_px: f32,
    camera: &GizmoCamera,
) -> Overlay {
    let mut ov = Overlay::new();
    let len = camera.world_per_pixel(origin) * screen_len_px;
    let axes = [
        (x.normalized(), AXIS_X_COLOR),
        (y.normalized(), AXIS_Y_COLOR),
        (z.normalized(), AXIS_Z_COLOR),
    ];
    // Shafts first (tests read these three segments).
    for (dir, col) in axes {
        ov.line(origin, origin.add(dir.scale(len)), col);
    }
    // Arrowheads.
    let head = len * 0.16;
    for (dir, col) in axes {
        push_cone(&mut ov, origin.add(dir.scale(len)), dir, head, head * 0.5, col, 10);
    }
    // Origin marker (small diamond).
    let marker = (len * 0.08).max(camera.world_per_pixel(origin) * 3.0);
    push_octahedron(&mut ov, origin, marker, [0.88, 0.88, 0.92, 1.0]);
    ov
}

/// The world-axis helper (`__WORLD_AXES__`): a screen-constant X/Y/Z triad at
/// the world origin. `length_px` is the axis length in CSS pixels.
pub fn world_axes(camera: &GizmoCamera, length_px: f32) -> Overlay {
    datum_frame(Vec3::ZERO, Vec3::X, Vec3::Y, Vec3::Z, length_px, camera)
}

// --- selectable wrappers (Gizmo impls) -------------------------------------

/// A selectable datum plane for hit-testing/highlight. `size` is the full world
/// edge length, or `None` for a screen-constant card. `handle` is the body
/// [`HandleId`] returned from [`Gizmo::hit`].
#[derive(Debug, Clone, Copy)]
pub struct DatumPlane {
    pub origin: Vec3,
    pub x_axis: Vec3,
    pub y_axis: Vec3,
    pub size: Option<f32>,
    pub color: [f32; 4],
    pub handle: HandleId,
}

impl DatumPlane {
    /// The plane normal (`x_axis × y_axis`, normalized).
    pub fn normal(&self) -> Vec3 {
        self.x_axis.cross(self.y_axis).normalized()
    }

    /// World half-edge length for the current camera (world- or screen-sized).
    /// The SAME expression [`datum_plane`] / [`datum_plane_screen`] draw with, read
    /// off the LIVE camera on every call — so the pickable card can never drift
    /// from the drawn one across a zoom (no bake).
    fn half(&self, camera: &GizmoCamera) -> f32 {
        match self.size {
            Some(s) => s * 0.5,
            None => camera.world_per_pixel(self.origin) * DEFAULT_PLANE_SCREEN_PX * 0.5,
        }
    }

    /// The world-space point where the pointer ray crosses this plane, but ONLY
    /// inside the DRAWN card (`|u| <= half`, `|v| <= half`) — a construction plane
    /// is mathematically infinite, yet only the rectangle the user can see is
    /// pickable. `None` when the ray misses the card, runs parallel to the plane,
    /// or crosses it behind the eye.
    ///
    /// Either FACE of the card picks: the test is on the ray parameter, not on the
    /// normal's sign, so a plane viewed from its reverse side is still selectable.
    /// [`Gizmo::hit`] is exactly this test — one bounds implementation, shared.
    pub fn hit_point(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<Vec3> {
        let ray = camera.ray_from_screen(screen[0], screen[1]);
        let t = ray.intersect_plane(self.origin, self.normal())?;
        if t < 0.0 {
            return None;
        }
        let p = ray.at(t);
        let rel = p.sub(self.origin);
        let u = rel.dot(self.x_axis.normalized());
        let v = rel.dot(self.y_axis.normalized());
        let half = self.half(camera);
        (u.abs() <= half && v.abs() <= half).then_some(p)
    }
}

impl Gizmo for DatumPlane {
    fn geometry(
        &self,
        camera: &GizmoCamera,
        hovered: Option<HandleId>,
        active: Option<HandleId>,
    ) -> Overlay {
        let hot = hovered == Some(self.handle) || active == Some(self.handle);
        let color = if hot { brighten(self.color) } else { self.color };
        match self.size {
            Some(s) => datum_plane(self.origin, self.x_axis, self.y_axis, s, color),
            None => datum_plane_screen(
                self.origin,
                self.x_axis,
                self.y_axis,
                DEFAULT_PLANE_SCREEN_PX,
                color,
                camera,
            ),
        }
    }

    fn hit(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId> {
        self.hit_point(camera, screen).map(|_| self.handle)
    }
}

/// A selectable datum axis for hit-testing/highlight. `handle` is the body
/// [`HandleId`] returned from [`Gizmo::hit`].
#[derive(Debug, Clone, Copy)]
pub struct DatumAxis {
    pub point: Vec3,
    pub direction: Vec3,
    pub length: f32,
    pub color: [f32; 4],
    pub handle: HandleId,
}

impl DatumAxis {
    fn end(&self) -> Vec3 {
        self.point.add(self.direction.normalized().scale(self.length))
    }
}

impl Gizmo for DatumAxis {
    fn geometry(
        &self,
        _camera: &GizmoCamera,
        hovered: Option<HandleId>,
        active: Option<HandleId>,
    ) -> Overlay {
        let hot = hovered == Some(self.handle) || active == Some(self.handle);
        let color = if hot { brighten(self.color) } else { self.color };
        datum_axis(self.point, self.direction, self.length, color)
    }

    fn hit(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId> {
        let ray = camera.ray_from_screen(screen[0], screen[1]);
        let end = self.end();
        let world_dist = ray.distance_to_segment(self.point, end);
        let mid = self.point.lerp(end, 0.5);
        let px = world_dist / camera.world_per_pixel(mid);
        if px <= PICK_PX {
            Some(self.handle)
        } else {
            None
        }
    }
}

// --- geometry helpers ------------------------------------------------------

/// Push a cone (arrowhead) as triangles: apex at `apex`, base ring of `radius`
/// centered `len` back along `-dir`. `dir` must be unit.
fn push_cone(
    ov: &mut Overlay,
    apex: Vec3,
    dir: Vec3,
    len: f32,
    radius: f32,
    color: [f32; 4],
    segments: usize,
) {
    let base = apex.sub(dir.scale(len));
    let p1 = dir.any_perp();
    let p2 = dir.cross(p1).normalized();
    let seg = segments.max(3);
    for i in 0..seg {
        let a0 = (i as f32) / (seg as f32) * std::f32::consts::TAU;
        let a1 = ((i + 1) as f32) / (seg as f32) * std::f32::consts::TAU;
        let r0 = base.add(p1.scale(a0.cos() * radius)).add(p2.scale(a0.sin() * radius));
        let r1 = base.add(p1.scale(a1.cos() * radius)).add(p2.scale(a1.sin() * radius));
        ov.tri(apex, r0, r1, color); // side
        ov.tri(base, r1, r0, color); // base cap
    }
}

/// Push a small octahedron ("diamond") marker centered at `center`, with
/// vertices at `center ± axis*half` on each world axis (8 triangular faces).
fn push_octahedron(ov: &mut Overlay, center: Vec3, half: f32, color: [f32; 4]) {
    let px = center.add(Vec3::X.scale(half));
    let nx = center.sub(Vec3::X.scale(half));
    let py = center.add(Vec3::Y.scale(half));
    let ny = center.sub(Vec3::Y.scale(half));
    let pz = center.add(Vec3::Z.scale(half));
    let nz = center.sub(Vec3::Z.scale(half));
    let faces = [
        (px, py, pz),
        (py, nx, pz),
        (nx, ny, pz),
        (ny, px, pz),
        (py, px, nz),
        (nx, py, nz),
        (ny, nx, nz),
        (px, ny, nz),
    ];
    for (a, b, c) in faces {
        ov.tri(a, b, c, color);
    }
}

fn scale_rgb(c: [f32; 4], s: f32) -> [f32; 4] {
    [c[0] * s, c[1] * s, c[2] * s, c[3]]
}
fn with_alpha(c: [f32; 4], a: f32) -> [f32; 4] {
    [c[0], c[1], c[2], a]
}
fn brighten(c: [f32; 4]) -> [f32; 4] {
    [
        (c[0] * 1.35 + 0.1).min(1.0),
        (c[1] * 1.35 + 0.1).min(1.0),
        (c[2] * 1.35 + 0.1).min(1.0),
        c[3],
    ]
}

// --- tests -----------------------------------------------------------------

