//! Arcball orbit/pan/zoom (R22) — the ArcballControls feel the user base has
//! muscle memory for, as a pure state machine over [`ViewCamera`]:
//!
//! - left drag  = trackball rotate (Shoemake sphere/hyperbola blend, 1:1),
//! - right/middle drag = pan (scene follows the cursor),
//! - wheel = zoom about the target, a gentle exponential step per notch
//!   (`enableAnimations` was false in the app, so there is deliberately NO
//!   inertia/damping; the smooth ramp comes from egui's own per-frame scroll
//!   smoothing — see [`ZOOM_PER_NOTCH`]).
//!
//! Input arrives as forwarded browser pointer/wheel events through the R3 API (CSS
//! pixels); the same struct drives the winit desktop shell.

use crate::view::{add3, cross3, dot3, len3, norm3, rotate3, scale3, sub3, Projection, ViewCamera};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Gesture {
    None,
    Rotate,
    Pan,
}

/// Buttons follow the browser `PointerEvent.button` values.
pub const BUTTON_LEFT: i32 = 0;
pub const BUTTON_MIDDLE: i32 = 1;
pub const BUTTON_RIGHT: i32 = 2;

/// Wheel-zoom sensitivity. Input `delta_y` is in egui POINTS (pixel-like): the
/// eframe viewport forwards `smooth_scroll_delta.y`, the browser bridge forwards
/// `WheelEvent.deltaY` (normalized to points by the host forwarder), and the winit
/// desktop shell forwards `LineDelta*100 / PixelDelta`. egui SMOOTHS one physical
/// wheel notch across several frames (see egui `WheelState::after_events`), so a
/// single notch can arrive as many small point-fragments.
///
/// We map points → a fractional notch count and apply an EXPONENTIAL zoom
/// `ZOOM_PER_NOTCH^notches`. Because the per-frame factors MULTIPLY, the product
/// over a smoothed notch telescopes to exactly
/// `ZOOM_PER_NOTCH^(total_points / WHEEL_POINTS_PER_NOTCH)` — so the feel is
/// independent of how egui splits the notch into frames, and zoom is smooth and
/// can never overshoot/jump through the target, on both native and wasm. (The
/// old code treated each sub-40-point smoothing fragment AS whole notches and
/// clamped to 3, compounding ~1.331× per frame → one notch overshot several-fold.)
///
/// `WHEEL_POINTS_PER_NOTCH = 70` sits between egui's native line notch
/// (`line_scroll_speed = 40` pt) and a browser's pixel notch (~100 pt), so a
/// native wheel notch ≈ 4.5% and a browser notch ≈ 11.6% view-distance change —
/// both a gentle, controllable step. Tune `ZOOM_PER_NOTCH` to taste.
const ZOOM_PER_NOTCH: f64 = 1.08;
/// egui points per physical wheel notch, used to normalize `delta_y` (see above).
const WHEEL_POINTS_PER_NOTCH: f64 = 70.0;
/// Cap a single pathological wheel event (e.g. a trackpad flick reporting a huge
/// one-frame delta) so it can't jump abruptly. This deliberately breaks the
/// telescoping property ONLY for such outliers; normal smoothed notches stay far
/// below the cap, so their per-frame factors still multiply cleanly.
const MAX_WHEEL_NOTCHES_PER_EVENT: f64 = 3.0;
const MIN_ORTHO_HALF_HEIGHT: f64 = 1e-9;
const MIN_PERSP_DISTANCE: f64 = 1e-6;

#[derive(Debug, Default)]
pub struct ArcballControls {
    pub enabled: bool,
    gesture: GestureState,
}

#[derive(Debug)]
struct GestureState {
    kind: Gesture,
    last: (f64, f64),
}

impl Default for GestureState {
    fn default() -> Self {
        Self {
            kind: Gesture::None,
            last: (0.0, 0.0),
        }
    }
}

impl ArcballControls {
    pub fn new() -> Self {
        Self {
            enabled: true,
            gesture: GestureState::default(),
        }
    }

    pub fn active_gesture(&self) -> Gesture {
        self.gesture.kind
    }

    /// Begin a gesture. Returns true when the pointer is captured for camera
    /// interaction.
    pub fn pointer_down(&mut self, x: f64, y: f64, button: i32) -> bool {
        if !self.enabled {
            return false;
        }
        self.gesture.kind = match button {
            BUTTON_LEFT => Gesture::Rotate,
            BUTTON_MIDDLE | BUTTON_RIGHT => Gesture::Pan,
            _ => Gesture::None,
        };
        self.gesture.last = (x, y);
        self.gesture.kind != Gesture::None
    }

    /// Advance the active gesture; mutates the camera. Returns true when the
    /// camera changed (the dirty signal).
    pub fn pointer_move(&mut self, camera: &mut ViewCamera, x: f64, y: f64) -> bool {
        if !self.enabled || self.gesture.kind == Gesture::None {
            return false;
        }
        let (lx, ly) = self.gesture.last;
        if (x - lx).abs() < f64::EPSILON && (y - ly).abs() < f64::EPSILON {
            return false;
        }
        match self.gesture.kind {
            Gesture::Rotate => rotate_arcball(camera, (lx, ly), (x, y)),
            Gesture::Pan => pan(camera, x - lx, y - ly),
            Gesture::None => {}
        }
        self.gesture.last = (x, y);
        true
    }

    /// End the gesture. Returns true if one was active.
    pub fn pointer_up(&mut self) -> bool {
        let was = self.gesture.kind != Gesture::None;
        self.gesture.kind = Gesture::None;
        was
    }

    /// Wheel zoom about the target. `delta_y` is in egui POINTS (see
    /// [`ZOOM_PER_NOTCH`]); negative = zoom in. Returns true when the camera
    /// changed.
    pub fn wheel(&mut self, camera: &mut ViewCamera, delta_y: f64, cursor: Option<[f64; 2]>) -> bool {
        if !self.enabled || delta_y == 0.0 {
            return false;
        }
        // Points → fractional notches, then an EXPONENTIAL per-notch zoom. The
        // clamp only guards a pathological single-frame delta; normal smoothed
        // notches stay well under it so their per-frame factors telescope.
        let notches = (delta_y / WHEEL_POINTS_PER_NOTCH)
            .clamp(-MAX_WHEEL_NOTCHES_PER_EVENT, MAX_WHEEL_NOTCHES_PER_EVENT);
        let factor = ZOOM_PER_NOTCH.powf(notches);
        match cursor {
            Some([cx, cy]) => zoom_toward(camera, factor, cx, cy),
            None => zoom(camera, factor),
        }
        true
    }
}

/// Zoom by `factor` while keeping the world point under the cursor fixed —
/// "zoom toward the mouse". `cx,cy` are cursor CSS px (top-left origin).
pub fn zoom_toward(camera: &mut ViewCamera, factor: f64, cx: f64, cy: f64) {
    let wpp = camera.world_per_pixel();
    let (right, up, _) = camera.basis();
    let sx = cx - camera.width * 0.5;
    let sy = -(cy - camera.height * 0.5); // screen-y-down → world-up
    // World offset from the target to the cursor point on the focus plane.
    let off = add3(scale3(right, sx * wpp), scale3(up, sy * wpp));
    match &mut camera.projection {
        Projection::Orthographic { half_height } => {
            let old = *half_height;
            *half_height = (old * factor).max(MIN_ORTHO_HALF_HEIGHT);
            let f = *half_height / old; // actual (clamped) factor
            let shift = scale3(off, 1.0 - f);
            camera.target = add3(camera.target, shift);
            camera.eye = add3(camera.eye, shift);
        }
        Projection::Perspective { .. } => {
            // Scale eye + target about the cursor world point (keeps the view
            // direction; the cursor point stays put → zoom toward the mouse).
            let cursor_world = add3(camera.target, off);
            let nt = add3(cursor_world, scale3(sub3(camera.target, cursor_world), factor));
            let ne = add3(cursor_world, scale3(sub3(camera.eye, cursor_world), factor));
            let dir = sub3(ne, nt);
            let dist = len3(dir).max(MIN_PERSP_DISTANCE);
            camera.target = nt;
            camera.eye = add3(nt, scale3(norm3(dir), dist));
        }
    }
}

/// Zoom by a frustum-scale factor (>1 zooms out).
pub fn zoom(camera: &mut ViewCamera, factor: f64) {
    match &mut camera.projection {
        Projection::Orthographic { half_height } => {
            *half_height = (*half_height * factor).max(MIN_ORTHO_HALF_HEIGHT);
        }
        Projection::Perspective { .. } => {
            let dir = sub3(camera.eye, camera.target);
            let dist = (len3(dir) * factor).max(MIN_PERSP_DISTANCE);
            camera.eye = add3(camera.target, scale3(norm3(dir), dist));
        }
    }
}

/// Pan by a screen-space delta in CSS px: the scene follows the cursor
/// (dragging right moves the model right, i.e. the camera left).
pub fn pan(camera: &mut ViewCamera, dx: f64, dy: f64) {
    let (right, up, _) = camera.basis();
    let wpp = camera.world_per_pixel();
    let offset = add3(scale3(right, -dx * wpp), scale3(up, dy * wpp));
    camera.eye = add3(camera.eye, offset);
    camera.target = add3(camera.target, offset);
}

/// Map a CSS-pixel cursor position onto the virtual trackball (camera-space
/// unit vector). Shoemake sphere with the ArcballControls hyperbolic skirt so
/// the rotation stays continuous past the sphere edge.
fn trackball_point(camera: &ViewCamera, x: f64, y: f64) -> [f64; 3] {
    let radius = 0.5 * camera.width.min(camera.height).max(1.0) * 0.75;
    let cx = camera.width * 0.5;
    let cy = camera.height * 0.5;
    let px = x - cx;
    let py = cy - y; // y up in trackball space
    let r2 = radius * radius;
    let d2 = px * px + py * py;
    let pz = if d2 <= r2 * 0.5 {
        (r2 - d2).sqrt()
    } else {
        // Hyperbolic sheet: z = (r²/2)/√d²
        r2 * 0.5 / d2.sqrt()
    };
    norm3([px, py, pz])
}

/// Trackball rotate from cursor `from` → `to` (CSS px): rotates the eye AND
/// the up vector around the target — a free arcball, no up-axis lock (the
/// ArcballControls behavior).
fn rotate_arcball(camera: &mut ViewCamera, from: (f64, f64), to: (f64, f64)) {
    let v0 = trackball_point(camera, from.0, from.1);
    let v1 = trackball_point(camera, to.0, to.1);
    let axis_cam = cross3(v0, v1);
    let axis_len = len3(axis_cam);
    if axis_len < 1e-12 {
        return;
    }
    let angle = dot3(v0, v1).clamp(-1.0, 1.0).acos();
    if angle.abs() < 1e-12 {
        return;
    }
    // Camera-space axis → world space through the camera basis; the scene
    // rotates WITH the drag, so the camera rotates by the inverse.
    let (right, up, forward) = camera.basis();
    let axis_cam = scale3(axis_cam, 1.0 / axis_len);
    let axis_world = norm3(add3(
        add3(scale3(right, axis_cam[0]), scale3(up, axis_cam[1])),
        scale3(forward, -axis_cam[2]),
    ));
    let offset = sub3(camera.eye, camera.target);
    camera.eye = add3(camera.target, rotate3(offset, axis_world, -angle));
    camera.up = norm3(rotate3(camera.up, axis_world, -angle));
}

