//! Curve DISPLAY builders (R30): spline / helix editor visuals for the render
//! engine's overlay pass.
//!
//! A NURBS/spline or helix is a smooth analytic curve in the kernel. The render
//! engine samples that curve to a dense polyline of world points, then calls
//! [`polyline_display`] to turn the samples into overlay line segments. This
//! module owns NO curve math beyond a parametric-helix convenience: it consumes
//! already-sampled points so it never needs the kernel.
//!
//! # Integration
//! - The engine tessellates the kernel curve (the retired viewer did this
//!   via analytic curve sampling)
//!   into `&[Vec3]` and calls [`polyline_display`]. `closed` closes the loop.
//! - [`helix_display`] / [`helix_points`] are a self-contained fallback for when
//!   the helix is available only in parametric form (axis, radius, pitch,
//!   turns) rather than as kernel samples — matching the app's helix feature
//!   params (radius, optional end-radius taper, handedness, turns/pitch).
//! - Spline-editing control handles are optional: [`control_point_handles`]
//!   emits small screen-constant square markers at the control points and
//!   [`hit_control_point`] answers which one is under a screen pixel (for the
//!   interaction layer to start a drag).
//!
//! Conventions: +Y-up world (the app's — see the crate header), linear-space
//! RGBA. No kernel/GPU dep. [`HelixParams::new`]'s +Z axis is a curve default,
//! not a claim about the world up.

use crate::{GizmoCamera, Overlay, Vec3};

/// Default curve display color (linear-space RGBA) — a cool CAD blue.
pub const CURVE_COLOR: [f32; 4] = [0.45, 0.75, 1.0, 1.0];
/// Default control-point handle color.
pub const HANDLE_COLOR: [f32; 4] = [1.0, 1.0, 1.0, 1.0];
/// Control-point handle side length, CSS pixels (screen-constant).
pub const HANDLE_SIZE_PX: f32 = 8.0;

/// Build a curve display from a sampled polyline: consecutive points joined by
/// line segments. When `closed`, the last point is also joined back to the
/// first. Emits `N-1` segments for an open curve (`N` for a closed one); an
/// empty/one-point input yields an empty overlay.
pub fn polyline_display(points: &[Vec3], color: [f32; 4], closed: bool) -> Overlay {
    let mut ov = Overlay::new();
    if points.len() < 2 {
        return ov;
    }
    for w in points.windows(2) {
        ov.line(w[0], w[1], color);
    }
    if closed && points.len() >= 3 {
        ov.line(points[points.len() - 1], points[0], color);
    }
    ov
}

/// Parametric helix definition (a self-contained fallback when the kernel has
/// not sampled the curve). Matches the app's helix feature: a base `radius`
/// (optionally tapering to `end_radius`), axial `pitch` (rise per full turn),
/// a number of `turns`, a `start_angle`, and a `right_handed` winding flag.
#[derive(Debug, Clone, Copy)]
pub struct HelixParams {
    /// A point on the helix axis (turn 0 sits here, before the start angle).
    pub axis_origin: Vec3,
    /// Axis direction (need not be unit); +Z is the app default.
    pub axis_dir: Vec3,
    /// Base (turn-0) radius, world units.
    pub radius: f32,
    /// End (final-turn) radius for a tapering helix; equal to `radius` for a
    /// straight cylinder.
    pub end_radius: f32,
    /// Axial advance per full turn, world units.
    pub pitch: f32,
    /// Total number of turns (may be fractional).
    pub turns: f32,
    /// Angular offset of the first sample, radians.
    pub start_angle: f32,
    /// Right-handed winding when true, left-handed when false.
    pub right_handed: bool,
    /// Samples per full turn (clamped to >= 3).
    pub points_per_turn: usize,
}

impl HelixParams {
    /// A right-handed +Z helix with equal end radii and 24 samples/turn.
    pub fn new(axis_origin: Vec3, radius: f32, pitch: f32, turns: f32) -> Self {
        Self {
            axis_origin,
            axis_dir: Vec3::Z,
            radius,
            end_radius: radius,
            pitch,
            turns,
            start_angle: 0.0,
            right_handed: true,
            points_per_turn: 24,
        }
    }
}

/// Sample a parametric helix into world points (turn 0 → `turns`).
pub fn helix_points(p: &HelixParams) -> Vec<Vec3> {
    let axis = if p.axis_dir.length() < 1e-9 {
        Vec3::Z
    } else {
        p.axis_dir.normalized()
    };
    let u = axis.any_perp();
    let v = axis.cross(u).normalized();
    let ppt = p.points_per_turn.max(3);
    let total = ((p.turns.abs() * ppt as f32).ceil() as usize).max(1);
    let wind = if p.right_handed { 1.0 } else { -1.0 };
    let mut out = Vec::with_capacity(total + 1);
    for i in 0..=total {
        // fraction of full sweep completed (turns).
        let frac = (i as f32 / total as f32) * p.turns;
        let ang = p.start_angle + wind * frac * std::f32::consts::TAU;
        let (s, c) = ang.sin_cos();
        // Linear radius taper across the whole helix.
        let t = if p.turns.abs() > 1e-9 {
            frac / p.turns
        } else {
            0.0
        };
        let r = p.radius + (p.end_radius - p.radius) * t;
        let radial = u.scale(c * r).add(v.scale(s * r));
        let axial = axis.scale(p.pitch * frac);
        out.push(p.axis_origin.add(radial).add(axial));
    }
    out
}

/// Convenience: sample a parametric helix and build its polyline display.
pub fn helix_display(p: &HelixParams, color: [f32; 4]) -> Overlay {
    polyline_display(&helix_points(p), color, false)
}

/// Emit screen-constant square markers at each control point (for spline
/// editing). Each marker is a small camera-facing quad (two triangles) of side
/// [`HANDLE_SIZE_PX`]; pair with [`hit_control_point`] for picking.
pub fn control_point_handles(
    points: &[Vec3],
    camera: &GizmoCamera,
    color: [f32; 4],
) -> Overlay {
    let mut ov = Overlay::new();
    for &p in points {
        let wpp = camera.world_per_pixel(p);
        let half = HANDLE_SIZE_PX * 0.5 * wpp;
        let right = camera.screen_right(p);
        // Screen-vertical axis (sign irrelevant for a symmetric square).
        let mut up = right.cross(camera.forward);
        up = if up.length() < 1e-9 {
            right.any_perp()
        } else {
            up.normalized()
        };
        let c0 = p.add(right.scale(half)).add(up.scale(half));
        let c1 = p.add(right.scale(-half)).add(up.scale(half));
        let c2 = p.add(right.scale(-half)).add(up.scale(-half));
        let c3 = p.add(right.scale(half)).add(up.scale(-half));
        ov.tri(c0, c1, c2, color);
        ov.tri(c0, c2, c3, color);
    }
    ov
}

/// Which control point (if any) is under `screen` (top-left origin, y down).
/// Returns the nearest point whose screen projection is within half a handle
/// of the pixel (square hit region), or `None`.
pub fn hit_control_point(
    points: &[Vec3],
    camera: &GizmoCamera,
    screen: [f32; 2],
) -> Option<usize> {
    let threshold = HANDLE_SIZE_PX * 0.5 + 2.0;
    let mut best: Option<usize> = None;
    let mut best_d = threshold;
    for (i, &p) in points.iter().enumerate() {
        if let Some(s) = camera.world_to_screen(p) {
            let d = (s[0] - screen[0]).abs().max((s[1] - screen[1]).abs());
            if d <= best_d {
                best_d = d;
                best = Some(i);
            }
        }
    }
    best
}

