//! Stateless transform gizmo: three axis arrows, three planar translation
//! handles, three rotation rings, and an optional free-move handle.
//!
//! The host supplies a [`GizmoCamera`] and the selected feature's frame through
//! [`TransformGizmo::set_frame`]. [`Gizmo::geometry`] emits world-space overlay
//! geometry; [`Gizmo::hit`] resolves CSS-pixel pointer positions to handles.
//! The host owns the hovered and active handles.
//!
//! During a drag, pass the active handle and start/current camera rays to
//! [`TransformGizmo::drag_delta`]. Translation components are distances along
//! the frame axes: `world_delta = ex*v.x + ey*v.y + ez*v.z`. Rotation returns
//! a frame-axis index and signed angle. The host applies these to the feature.
//!
//! Using the pointer-down ray gives an absolute delta; using the previous
//! frame's ray gives an incremental delta and avoids rotation's ±180° wrap.

use crate::{Gizmo, GizmoCamera, HandleId, Overlay};
use crate::hit_region::{point_region, segment_region, HitShape};
use crate::math::{Ray, Vec3};

// --- stable handle ids -----------------------------------------------------

/// Body / no specific handle (contract convention).
pub const HANDLE_NONE: HandleId = 0;
/// Axis translate arrows.
pub const HANDLE_AXIS_X: HandleId = 1;
pub const HANDLE_AXIS_Y: HandleId = 2;
pub const HANDLE_AXIS_Z: HandleId = 3;
/// Planar translate quads (named by the two in-plane axes).
pub const HANDLE_PLANE_XY: HandleId = 4;
pub const HANDLE_PLANE_YZ: HandleId = 5;
pub const HANDLE_PLANE_ZX: HandleId = 6;
/// Rotation rings (named by the axis they turn about).
pub const HANDLE_RING_X: HandleId = 7;
pub const HANDLE_RING_Y: HandleId = 8;
pub const HANDLE_RING_Z: HandleId = 9;
/// Center free-move / uniform handle (screen-plane translate).
pub const HANDLE_CENTER: HandleId = 10;

// --- colors (linear RGBA) --------------------------------------------------

// Restyled gizmo look (matches the reference transform-controls image): an
// ORANGE center sphere, silver-grey axis shafts drawn as solid 3D rods (tubes)
// with orange CONE tips, three light-grey ROTATION ARCS joining adjacent axis
// tips (the rounded-triangle silhouette), and orange grab SPHERES for rotation.
// Colors are display sRGB values written ~directly by the overlay shader (with a
// per-face shade for depth), so use hex/255 — no linear conversion.
const C_ROD: [f32; 4] = [0.80, 0.81, 0.82, 1.0]; // silver-grey rod shafts (~0xccced1)
const C_ARROW: [f32; 4] = [0.961, 0.651, 0.137, 1.0]; // orange cone tips (#F5A623)
const C_RING: [f32; 4] = [0.91, 0.91, 0.91, 1.0]; // light-grey rotation arcs (~0xe8e8e8)
const C_DOT: [f32; 4] = [0.961, 0.651, 0.137, 1.0]; // orange rotation grab spheres (#F5A623)
const C_GOLD: [f32; 4] = [1.00, 0.85, 0.35, 1.0]; // hover/active highlight (amber)
const C_CENTER: [f32; 4] = [0.961, 0.651, 0.137, 1.0]; // orange center sphere (#F5A623)

// --- pixel sizing (screen-constant; multiplied by world_per_pixel) ---------

/// Arrow tip distance from the origin (CSS px). Public so the app can place the
/// egui axis labels (`XC`/`YC`/`ZC`) just past each cone tip. Chosen so the cone
/// BASE (`PX_AXIS_LEN - PX_HEAD_LEN` = 72) sits a few px OUTSIDE the rotation arc
/// (`PX_ARC_RAD` = 64): the arcs join the shafts and the cones poke past them, so
/// the white arcs never cut through the orange arrowheads.
pub const PX_AXIS_LEN: f32 = 90.0;
const PX_SHAFT_START: f32 = 8.0; // shaft begins this far out (emerges from the center sphere)
const PX_HEAD_LEN: f32 = 18.0; // arrowhead cone length
const PX_HEAD_RAD: f32 = 7.0; // arrowhead cone base radius
const PX_SHAFT_RAD: f32 = 2.2; // silver-rod shaft radius (thick, reads as a 3D rod)
const PX_ARC_RAD: f32 = 64.0; // rotation-arc radius (arcs join the SHAFTS, inside the cone bases)
/// Center free-move sphere radius (CSS px). Public so a debug overlay can outline
/// the exact pickable disc without re-deriving it.
pub const PX_CENTER_RAD: f32 = 7.0;
/// Orange rotation grab-sphere radius (CSS px). Public for the same reason.
pub const PX_RING_GRAB_RAD: f32 = 5.0;

/// Screen-pixel half-width of the axis-arrow (and rotation-arc) hit test: a
/// cursor within this many px of a handle's projected drawn segment grabs it (see
/// [`TransformGizmo::hit`]). Public so a debug overlay can outline the EXACT
/// pickable region (a stadium of this radius around the projected axis segment)
/// without re-deriving — the drawn radius can never drift from the hit radius.
pub const AXIS_HIT_THRESH_PX: f32 = 7.0;

const RING_SEGMENTS: usize = 24; // samples per quarter rotation arc
const CONE_SEGMENTS: usize = 16; // radial facets of an arrowhead cone
const TUBE_SEGMENTS: usize = 8; // radial facets of a shaft rod
const SPHERE_RINGS: usize = 6; // latitude bands of a handle sphere
const SPHERE_SECTORS: usize = 10; // longitude sectors of a handle sphere

/// The three rotation arcs as `(rotation-axis index, in-plane axis i, in-plane
/// axis j)`. Each arc sweeps from tip `i` to tip `j` in the plane whose normal
/// is the rotation axis, so grabbing it rotates about that axis.
const ARCS: [(usize, usize, usize); 3] = [(2, 0, 1), (0, 1, 2), (1, 2, 0)];

/// Screen-constant handle sizes in world units at the gizmo origin.
#[derive(Debug, Clone, Copy)]
struct Sizes {
    px: f32,
    axis_len: f32,
    shaft_start: f32,
    head_len: f32,
    head_rad: f32,
    shaft_rad: f32,
    arc_rad: f32,
    center_rad: f32,
    grab_rad: f32,
}

/// The result of a drag, expressed in the gizmo's FRAME (see module docs).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DragDelta {
    /// Translation whose components are distances along `ex`, `ey`, `ez`.
    /// World delta = `ex*v.x + ey*v.y + ez*v.z`.
    Translate(Vec3),
    /// Signed rotation about frame axis `axis_index` (0=ex, 1=ey, 2=ez).
    Rotate { axis_index: usize, radians: f32 },
    /// No usable delta (unknown handle or a degenerate ray/plane).
    None,
}

/// A move + rotate gizmo positioned at an origin with an orientation frame.
#[derive(Debug, Clone, Copy)]
pub struct TransformGizmo {
    /// Gizmo origin in world space.
    pub origin: Vec3,
    /// Frame X axis (unit, world space).
    pub ex: Vec3,
    /// Frame Y axis (unit, world space).
    pub ey: Vec3,
    /// Frame Z axis (unit, world space).
    pub ez: Vec3,
    /// Whether to draw + hit-test the center free-move handle.
    pub show_center: bool,
    /// Whether to draw + hit-test the three axis translate ARROWS. Off for a
    /// rotate-only gizmo (the component Move toggle's "rotate" step).
    pub show_axes: bool,
    /// Whether to draw + hit-test the three rotation ARCS + their grab spheres.
    /// Off for a translate-only gizmo (the component Move toggle's "translate"
    /// step).
    pub show_rings: bool,
}

impl Default for TransformGizmo {
    fn default() -> Self {
        Self {
            origin: Vec3::ZERO,
            ex: Vec3::X,
            ey: Vec3::Y,
            ez: Vec3::Z,
            show_center: true,
            show_axes: true,
            show_rings: true,
        }
    }
}

impl TransformGizmo {
    /// A gizmo at `origin` with the world XYZ frame.
    pub fn at(origin: Vec3) -> Self {
        Self { origin, ..Self::default() }
    }

    /// Set the origin + orientation frame (the feature's frame). The axes are
    /// re-orthonormalized defensively (ex kept, ez = ex×ey, ey = ez×ex).
    pub fn set_frame(&mut self, origin: Vec3, ex: Vec3, ey: Vec3, ez: Vec3) {
        self.origin = origin;
        let ex = ex.normalized();
        let mut ez = ez.normalized();
        if ez.length() < 1e-6 {
            ez = ex.cross(ey).normalized();
        }
        let ey = ez.cross(ex).normalized();
        let ez = ex.cross(ey).normalized();
        self.ex = ex;
        self.ey = ey;
        self.ez = ez;
    }

    /// World-space unit axis for `i` (0=ex, 1=ey, 2=ez).
    pub fn axis(&self, i: usize) -> Vec3 {
        match i {
            0 => self.ex,
            1 => self.ey,
            _ => self.ez,
        }
    }

    fn sizes(&self, camera: &GizmoCamera) -> Sizes {
        let px = camera.world_per_pixel(self.origin).max(1e-6);
        Sizes {
            px,
            axis_len: PX_AXIS_LEN * px,
            shaft_start: PX_SHAFT_START * px,
            head_len: PX_HEAD_LEN * px,
            head_rad: PX_HEAD_RAD * px,
            shaft_rad: PX_SHAFT_RAD * px,
            arc_rad: PX_ARC_RAD * px,
            center_rad: PX_CENTER_RAD * px,
            grab_rad: PX_RING_GRAB_RAD * px,
        }
    }

    /// The world-space (shaft-start, tip) endpoints of axis arrow `i`. Exposed
    /// so callers/tests can locate a handle without duplicating the sizing.
    pub fn axis_seg(&self, camera: &GizmoCamera, i: usize) -> (Vec3, Vec3) {
        let s = self.sizes(camera);
        let a = self.axis(i);
        (
            self.origin.add(a.scale(s.shaft_start)),
            self.origin.add(a.scale(s.axis_len)),
        )
    }

    /// The world-space center free-move / origin sphere point (the `HANDLE_CENTER`
    /// ball), or `None` when the center handle is hidden. Exposed so a debug
    /// overlay can outline the exact pickable disc (radius [`PX_CENTER_RAD`]).
    pub fn center_grab_point(&self) -> Option<Vec3> {
        self.show_center.then_some(self.origin)
    }

    /// The three rotation grab-sphere world points (one per rotation arc, in
    /// `ARCS` order), at the drawn arc-midpoint radius. Exposed so a debug overlay
    /// can outline each ball (radius [`PX_RING_GRAB_RAD`]).
    pub fn ring_grab_points(&self, camera: &GizmoCamera) -> [Vec3; 3] {
        let s = self.sizes(camera);
        let mut out = [Vec3::ZERO; 3];
        for (idx, (_, i, j)) in ARCS.iter().enumerate() {
            out[idx] = self.arc_grab_point(*i, *j, &s);
        }
        out
    }

    // -- drag math (pure; no state) -----------------------------------------

    /// Frame-space delta for the active `handle` given the drag's start & current
    /// rays (build both with `camera.ray_from_screen`). See [`DragDelta`].
    pub fn drag_delta(
        &self,
        camera: &GizmoCamera,
        handle: HandleId,
        start: Ray,
        current: Ray,
    ) -> DragDelta {
        match handle {
            HANDLE_AXIS_X => DragDelta::Translate(self.axis_translate(camera, 0, start, current)),
            HANDLE_AXIS_Y => DragDelta::Translate(self.axis_translate(camera, 1, start, current)),
            HANDLE_AXIS_Z => DragDelta::Translate(self.axis_translate(camera, 2, start, current)),
            HANDLE_PLANE_XY => DragDelta::Translate(self.plane_translate(0, 1, start, current)),
            HANDLE_PLANE_YZ => DragDelta::Translate(self.plane_translate(1, 2, start, current)),
            HANDLE_PLANE_ZX => DragDelta::Translate(self.plane_translate(2, 0, start, current)),
            HANDLE_RING_X => DragDelta::Rotate { axis_index: 0, radians: self.ring_rotate(0, start, current) },
            HANDLE_RING_Y => DragDelta::Rotate { axis_index: 1, radians: self.ring_rotate(1, start, current) },
            HANDLE_RING_Z => DragDelta::Rotate { axis_index: 2, radians: self.ring_rotate(2, start, current) },
            HANDLE_CENTER => DragDelta::Translate(self.center_translate(camera, start, current)),
            _ => DragDelta::None,
        }
    }

    /// Axis translate: project the camera-facing-plane movement onto the axis.
    /// Returns a frame-local vector with only component `i` set.
    pub fn axis_translate(&self, camera: &GizmoCamera, i: usize, start: Ray, current: Ray) -> Vec3 {
        let n = camera.forward.normalized();
        let (p0, p1) = match (
            plane_point(&start, self.origin, n),
            plane_point(&current, self.origin, n),
        ) {
            (Some(a), Some(b)) => (a, b),
            _ => return Vec3::ZERO,
        };
        let d = p1.sub(p0).dot(self.axis(i));
        let mut out = Vec3::ZERO;
        match i {
            0 => out.x = d,
            1 => out.y = d,
            _ => out.z = d,
        }
        out
    }

    /// Planar translate: intersect both rays with the plane spanned by frame
    /// axes `i` and `j` (normal = the third axis) through the origin. Returns a
    /// frame-local vector with components `i` and `j` set.
    pub fn plane_translate(&self, i: usize, j: usize, start: Ray, current: Ray) -> Vec3 {
        let k = 3 - i - j; // the remaining index → plane normal
        let n = self.axis(k);
        let (p0, p1) = match (
            plane_point(&start, self.origin, n),
            plane_point(&current, self.origin, n),
        ) {
            (Some(a), Some(b)) => (a, b),
            _ => return Vec3::ZERO,
        };
        let delta = p1.sub(p0);
        let a = delta.dot(self.axis(i));
        let b = delta.dot(self.axis(j));
        let mut out = Vec3::ZERO;
        set_comp(&mut out, i, a);
        set_comp(&mut out, j, b);
        out
    }

    /// Ring rotate: signed angle about frame axis `i` between the start & current
    /// ray intersections with the plane through the origin perpendicular to `i`.
    pub fn ring_rotate(&self, i: usize, start: Ray, current: Ray) -> f32 {
        let n = self.axis(i);
        let (p0, p1) = match (
            plane_point(&start, self.origin, n),
            plane_point(&current, self.origin, n),
        ) {
            (Some(a), Some(b)) => (a, b),
            _ => return 0.0,
        };
        let v0 = p0.sub(self.origin);
        let v1 = p1.sub(self.origin);
        if v0.length() < 1e-9 || v1.length() < 1e-9 {
            return 0.0;
        }
        let v0 = v0.normalized();
        let v1 = v1.normalized();
        let cross = v0.cross(v1).dot(n);
        let dot = v0.dot(v1).clamp(-1.0, 1.0);
        cross.atan2(dot)
    }

    /// Center free-move: screen-plane translate, returned in full frame-local
    /// coordinates (all three components may be set).
    pub fn center_translate(&self, camera: &GizmoCamera, start: Ray, current: Ray) -> Vec3 {
        let n = camera.forward.normalized();
        let (p0, p1) = match (
            plane_point(&start, self.origin, n),
            plane_point(&current, self.origin, n),
        ) {
            (Some(a), Some(b)) => (a, b),
            _ => return Vec3::ZERO,
        };
        let delta = p1.sub(p0);
        Vec3::new(delta.dot(self.ex), delta.dot(self.ey), delta.dot(self.ez))
    }

    // -- geometry helpers ---------------------------------------------------

    /// A point on the rotation arc that sweeps from tip `i` to tip `j` (in the
    /// plane whose normal is the third frame axis), parameter `t` in `0..=1`.
    fn arc_point(&self, i: usize, j: usize, t: f32, radius: f32) -> Vec3 {
        let ang = t * std::f32::consts::FRAC_PI_2;
        self.origin
            .add(self.axis(i).scale(ang.cos() * radius))
            .add(self.axis(j).scale(ang.sin() * radius))
    }

    /// The orange grab sphere position for the arc `(i, j)`: the MIDPOINT of the
    /// quarter arc (`t = 0.5`, the 45° bisector angle) at the arc radius, so the
    /// handle sits ON the drawn arc curve.
    fn arc_grab_point(&self, i: usize, j: usize, s: &Sizes) -> Vec3 {
        self.arc_point(i, j, 0.5, s.arc_rad)
    }

    fn draw_axis_arrow(
        &self,
        ov: &mut Overlay,
        s: &Sizes,
        i: usize,
        shaft_color: [f32; 4],
        head_color: [f32; 4],
    ) {
        let a = self.axis(i);
        let shaft_a = self.origin.add(a.scale(s.shaft_start));
        let base = self.origin.add(a.scale(s.axis_len - s.head_len));
        let tip = self.origin.add(a.scale(s.axis_len));
        // Silver rod shaft as a solid 3D tube (thick; per-element geometry since
        // the overlay line width is a single per-pass value).
        push_tube(ov, shaft_a, base, s.shaft_rad, shaft_color);
        // Orange arrowhead cone (side + base cap).
        push_cone(ov, base, tip, s.head_rad, head_color);
    }

    fn draw_arc(&self, ov: &mut Overlay, i: usize, j: usize, s: &Sizes, color: [f32; 4]) {
        let mut prev = self.arc_point(i, j, 0.0, s.arc_rad);
        for k in 1..=RING_SEGMENTS {
            let t = k as f32 / RING_SEGMENTS as f32;
            let cur = self.arc_point(i, j, t, s.arc_rad);
            ov.line(prev, cur, color);
            prev = cur;
        }
    }

    fn draw_center(&self, ov: &mut Overlay, s: &Sizes, color: [f32; 4]) {
        push_sphere(ov, self.origin, s.center_rad, color);
    }

    fn highlight(id: HandleId, base: [f32; 4], hovered: Option<HandleId>, active: Option<HandleId>) -> [f32; 4] {
        if active == Some(id) || hovered == Some(id) {
            C_GOLD
        } else {
            base
        }
    }
}

impl Gizmo for TransformGizmo {
    fn geometry(
        &self,
        camera: &GizmoCamera,
        hovered: Option<HandleId>,
        active: Option<HandleId>,
    ) -> Overlay {
        let s = self.sizes(camera);
        let mut ov = Overlay::new();

        let hl = |id, base| Self::highlight(id, base, hovered, active);

        // Rotation arcs — light-grey quarter arcs joining adjacent axis shafts
        // (the rounded-triangle silhouette), each with an orange grab sphere ON
        // the arc midpoint. Arc `(k, i, j)` rotates about axis `k`.
        if self.show_rings {
            for (k, i, j) in ARCS {
                let id = ring_handle(k);
                self.draw_arc(&mut ov, i, j, &s, hl(id, C_RING));
                push_sphere(&mut ov, self.arc_grab_point(i, j, &s), s.grab_rad, hl(id, C_DOT));
            }
        }

        // Axis translate arrows — silver rod shaft, orange cone head.
        if self.show_axes {
            for (i, id) in [(0, HANDLE_AXIS_X), (1, HANDLE_AXIS_Y), (2, HANDLE_AXIS_Z)] {
                self.draw_axis_arrow(&mut ov, &s, i, hl(id, C_ROD), hl(id, C_ARROW));
            }
        }

        // Center free-move handle — orange sphere.
        if self.show_center {
            self.draw_center(&mut ov, &s, hl(HANDLE_CENTER, C_CENTER));
        }

        ov
    }

    fn hit(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId> {
        // Test the cursor against the SAME screen-space regions the debug outline
        // draws ([`hit_regions`]): a 2D point-in-region test, so the grabbable
        // area IS the drawn outline. Rank breaks ambiguous overlaps — axis arrows
        // and the center handle are RANK 0 (arrows are always the top drag
        // priority), the rotation rings RANK 1, so an arrow beats a ring where
        // both contain the cursor; within a rank the nearest spine wins.
        let mut best: Option<(u8, f32, HandleId)> = None;
        for (id, shape) in self.hit_regions(camera) {
            let d = shape.spine_distance(screen);
            if d > shape.radius() {
                continue;
            }
            let rank = if is_ring(id) { 1 } else { 0 };
            match best {
                Some((br, bd, _)) if (br, bd) <= (rank, d) => {}
                _ => best = Some((rank, d, id)),
            }
        }
        best.map(|(_, _, id)| id)
    }
}

impl TransformGizmo {
    /// The authoritative screen-space pickable region of every draggable handle,
    /// each paired with its [`HandleId`]. The SINGLE source both [`Gizmo::hit`]
    /// (which 2D-tests the cursor against these) and the debug outline
    /// (`transform_hit_areas_json`, which strokes these) consume — so the
    /// grabbable area can never drift from the drawn outline. Projection + the
    /// perspective front-clip happen ONCE, in [`crate::hit_region`]. Kinds match
    /// the drawn handles exactly:
    ///   * center free-move sphere → CIRCLE (`PX_CENTER_RAD + 2`).
    ///   * 3 axis arrows → CAPSULE on the drawn `axis_seg` (`AXIS_HIT_THRESH_PX`).
    ///   * 3 rotation grab spheres → CIRCLE (`PX_RING_GRAB_RAD + 3`) — the ring's
    ///     ONLY hit region, matching its ONLY drawn outline (the grab sphere).
    /// The CENTER is emitted first so that on an EXACT tie it beats a coincident
    /// axis — when the view looks straight down an axis, that axis foreshortens
    /// onto the origin, and the (undraggable, screen-perpendicular) axis must not
    /// steal the center free-move handle (see [`Gizmo::hit`]'s first-wins tie).
    pub fn hit_regions(&self, camera: &GizmoCamera) -> Vec<(HandleId, HitShape)> {
        let s = self.sizes(camera);
        let mut out: Vec<(HandleId, HitShape)> = Vec::with_capacity(7);
        // Center free-move ball (FIRST — wins an exact tie with a coincident axis).
        if self.show_center {
            if let Some(shape) = point_region(camera, v3(self.origin), PX_CENTER_RAD + 2.0) {
                out.push((HANDLE_CENTER, shape));
            }
        }
        // Axis arrows — capsule on the drawn (shaft-start, tip) segment.
        if self.show_axes {
            for (i, id) in [(0, HANDLE_AXIS_X), (1, HANDLE_AXIS_Y), (2, HANDLE_AXIS_Z)] {
                let (a, b) = self.axis_seg(camera, i);
                if let Some(shape) = segment_region(camera, v3(a), v3(b), AXIS_HIT_THRESH_PX) {
                    out.push((id, shape));
                }
            }
        }
        // Rotation grab balls (one per arc, at the arc midpoint).
        if self.show_rings {
            for (k, i, j) in ARCS {
                let grab = self.arc_grab_point(i, j, &s);
                if let Some(shape) = point_region(camera, v3(grab), PX_RING_GRAB_RAD + 3.0) {
                    out.push((ring_handle(k), shape));
                }
            }
        }
        out
    }
}

/// A world point as `[f64; 3]` for the (f64) screen-space region builder.
fn v3(v: Vec3) -> [f64; 3] {
    [v.x as f64, v.y as f64, v.z as f64]
}

/// Whether `id` is a rotation-ring handle (ranked below the axis arrows).
fn is_ring(id: HandleId) -> bool {
    matches!(id, HANDLE_RING_X | HANDLE_RING_Y | HANDLE_RING_Z)
}

// --- free helpers ----------------------------------------------------------

fn plane_point(ray: &Ray, p0: Vec3, n: Vec3) -> Option<Vec3> {
    ray.intersect_plane(p0, n).map(|t| ray.at(t))
}

fn set_comp(v: &mut Vec3, i: usize, val: f32) {
    match i {
        0 => v.x = val,
        1 => v.y = val,
        _ => v.z = val,
    }
}

/// The rotation-ring handle id for rotation about frame axis `k`.
fn ring_handle(k: usize) -> HandleId {
    match k {
        0 => HANDLE_RING_X,
        1 => HANDLE_RING_Y,
        _ => HANDLE_RING_Z,
    }
}

/// A radially-symmetric perpendicular basis `(u, v)` for a unit `axis`.
fn axis_basis(axis: Vec3) -> (Vec3, Vec3) {
    let u = axis.any_perp();
    let v = axis.cross(u).normalized();
    (u, v)
}

/// Push a solid 3D rod (open-ended tube) from `a` to `b` with world `radius`.
/// Used for the silver axis shafts — thick per-element geometry, since the
/// overlay line width is a single per-pass value and can't be varied per shaft.
fn push_tube(ov: &mut Overlay, a: Vec3, b: Vec3, radius: f32, color: [f32; 4]) {
    let axis = b.sub(a);
    if axis.length() < 1e-9 || radius <= 0.0 {
        return;
    }
    let (u, v) = axis_basis(axis.normalized());
    let ring = |center: Vec3, k: usize| -> Vec3 {
        let ang = (k as f32 / TUBE_SEGMENTS as f32) * std::f32::consts::TAU;
        center
            .add(u.scale(ang.cos() * radius))
            .add(v.scale(ang.sin() * radius))
    };
    for k in 0..TUBE_SEGMENTS {
        let a0 = ring(a, k);
        let a1 = ring(a, k + 1);
        let b0 = ring(b, k);
        let b1 = ring(b, k + 1);
        ov.tri(a0, b0, b1, color);
        ov.tri(a0, b1, a1, color);
    }
}

/// Push a filled arrowhead cone: apex at `tip`, base circle of world `radius`
/// centered at `base`, radially symmetric about `tip - base` (side + base cap).
fn push_cone(ov: &mut Overlay, base: Vec3, tip: Vec3, radius: f32, color: [f32; 4]) {
    let axis = tip.sub(base);
    if axis.length() < 1e-9 || radius <= 0.0 {
        return;
    }
    let (u, v) = axis_basis(axis.normalized());
    let ring = |k: usize| -> Vec3 {
        let ang = (k as f32 / CONE_SEGMENTS as f32) * std::f32::consts::TAU;
        base.add(u.scale(ang.cos() * radius))
            .add(v.scale(ang.sin() * radius))
    };
    let mut prev = ring(0);
    for k in 1..=CONE_SEGMENTS {
        let cur = ring(k);
        ov.tri(tip, prev, cur, color); // side
        ov.tri(base, cur, prev, color); // base cap
        prev = cur;
    }
}

/// Push a filled UV sphere of world `radius` at `center`. Flat-shaded facets;
/// the overlay shader's per-face shade gives the 3D read.
fn push_sphere(ov: &mut Overlay, center: Vec3, radius: f32, color: [f32; 4]) {
    if radius <= 0.0 {
        return;
    }
    let point = |ring: usize, sector: usize| -> Vec3 {
        let lat = std::f32::consts::PI * (ring as f32 / SPHERE_RINGS as f32) - std::f32::consts::FRAC_PI_2;
        let lon = std::f32::consts::TAU * (sector as f32 / SPHERE_SECTORS as f32);
        center.add(Vec3::new(
            lat.cos() * lon.cos() * radius,
            lat.cos() * lon.sin() * radius,
            lat.sin() * radius,
        ))
    };
    for r in 0..SPHERE_RINGS {
        for sct in 0..SPHERE_SECTORS {
            let p00 = point(r, sct);
            let p01 = point(r, sct + 1);
            let p10 = point(r + 1, sct);
            let p11 = point(r + 1, sct + 1);
            ov.tri(p00, p10, p11, color);
            ov.tri(p00, p11, p01, color);
        }
    }
}

