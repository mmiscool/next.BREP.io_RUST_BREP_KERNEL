//! ViewCube navigation gizmo — a small orientation cube that mirrors the main
//! camera and, when a face / edge / corner is clicked, hands back the standard
//! view the engine's camera animation should snap to.
//!
//! # Convention (world = app convention, +Y up, right-handed)
//! Each cube region has an outward direction built from the sign of each axis it
//! is extreme on. A **face** is extreme on one axis, an **edge** on two, a
//! **corner** on three. The region's outward normal is `normalize(sx, sy, sz)`,
//! and the view the camera snaps to looks *inward* along `-normal` (eye placed on
//! the `+normal` side, looking at the model) — exactly the Y-up direction the
//! engine's `standard_view` buttons use, so a face click == its named button. So:
//! - FRONT face normal `+Z` → `target_view` = `-Z` (eye at +Z looking toward the
//!   model's front), matching the FRONT button (`dir +Z, up +Y`).
//! - BACK `-Z`→`+Z`, RIGHT `+X`→`-X`, LEFT `-X`→`+X`, TOP `+Y`→`-Y`,
//!   BOTTOM `-Y`→`+Y`.
//! - Edges = 45° blends of two faces, corners = iso (all three components set).
//!
//! Face letters (stroked as overlay line-glyphs, no SDF text):
//! `F`=Front(+Z), `BK`=Back(−Z), `R`=Right(+X), `L`=Left(−X), `T`=Top(+Y),
//! `B`=Bottom(−Y).
//!
//! # HandleId encoding
//! A region is identified by the base-3 packing of its axis signs
//! `id = (sx+1) + (sy+1)*3 + (sz+1)*9 + 1` (see [`ViewCube::region_id`]). `0` is
//! reserved for "no handle" (the trait convention); the all-zero packing (`14`)
//! never occurs because a surface point is always extreme on at least one axis.
//! Valid ids are `1..=27` minus `14`, one per hit region (6 faces + 12 edges +
//! 8 corners = 26). [`ViewCube::target_view`] / [`ViewCube::target_up`] decode an
//! id straight back to the snap direction and up hint.
//!
//! # Cube orientation
//! Geometry uses world axes (FRONT at +Z, TOP at +Y). The
//! [`ViewCube::mini_camera`] copies both the main camera's forward and up
//! vectors so the cube mirrors its roll. [`cube_up`] orthogonalizes the up
//! vector, using the Y-up fallback only when the supplied frame is degenerate.
//! Back-facing faces, edges, and glyphs are culled for both CPU and GPU overlays.
//!
//! # Integration (engine wiring)
//! - **Corner viewport sub-rect.** The gizmo owns a fixed `size`×`size` pixel
//!   square in the bottom-right corner, `margin` px in from the edges — see
//!   [`ViewCube::sub_rect`] (returns `[x, y, w, h]`, top-left origin). The engine
//!   renders the ViewCube overlay in its own scissor pass using
//!   [`ViewCube::mini_camera`] rather than the main camera's `view_proj`.
//! - **Pointer forwarding.** The host maps a pointer event into cube-local pixels
//!   `local = (pointer.x - rect.x, pointer.y - rect.y)` in `0..size`
//!   (top-left origin, y down) and calls [`Gizmo::hit`] with those coords — this
//!   mirrors the original `ViewCube`'s `_pickObjectAtEvent`. Events outside `sub_rect` are
//!   not forwarded.
//! - **Snap.** On click, the engine takes the returned [`HandleId`] and drives
//!   the *shared* camera (same path the orbit controls use) toward
//!   `target_view(id)` (eye→target direction), keeping the current pivot
//!   distance. The up is snapped to a discrete, minimal-rotation "level" roll
//!   (the engine's `snap_view_up`): a face lands flat-on with a horizontal
//!   bottom edge, a corner on the nearest axis-up isometric. `target_up(id)` is
//!   only the degenerate fallback used when that projection is undefined.

use crate::{Gizmo, GizmoCamera, HandleId, Overlay, Ray, Vec3};

/// Outer fraction of a face (measured from the edge, as a share of the half-size)
/// that hit-tests as the neighbouring edge / corner region rather than the face.
const EDGE_BAND: f32 = 0.30;

// Overlay colors (linear RGBA).
const COL_HOVER: [f32; 4] = [0.42, 0.64, 0.96, 1.0];
const COL_ACTIVE: [f32; 4] = [0.60, 0.82, 1.0, 1.0];
const COL_EDGE: [f32; 4] = [0.255, 0.275, 0.310, 1.0]; // gray tube edges (~#41464f)
const COL_EDGE_HI: [f32; 4] = [0.62, 0.86, 1.0, 1.0];
const COL_GLYPH: [f32; 4] = [1.0, 1.0, 1.0, 1.0]; // white labels
const COL_MARK: [f32; 4] = [0.58, 0.82, 1.0, 1.0];
const COL_CORNER: [f32; 4] = [0.306, 0.439, 0.643, 1.0]; // muted-blue corner spheres (~#4e70a4)
// Navigation arrows (2D, screen-fixed): light-gray triangles/arcs like the old
// renderer's ViewCube (fill ~#d6d9de, dark outline ~#161a20), brighter on hover.
const COL_ARROW: [f32; 4] = [0.840, 0.851, 0.871, 1.0];
const COL_ARROW_HI: [f32; 4] = [0.965, 0.975, 1.000, 1.0];
const COL_ARROW_LINE: [f32; 4] = [0.086, 0.102, 0.125, 1.0];

/// Lift a color toward white by `amt` (0..1) — the face touch/glow highlight.
fn brighten_by(c: [f32; 4], amt: f32) -> [f32; 4] {
    [
        c[0] + (1.0 - c[0]) * amt,
        c[1] + (1.0 - c[1]) * amt,
        c[2] + (1.0 - c[2]) * amt,
        c[3],
    ]
}

/// The ViewCube gizmo. Cheap to construct; holds only its pixel footprint.
#[derive(Debug, Clone, Copy)]
pub struct ViewCube {
    /// Edge length of the square corner viewport, in CSS pixels.
    pub size: f32,
    /// Inset from the bottom-right corner, in CSS pixels.
    pub margin: f32,
}

impl Default for ViewCube {
    fn default() -> Self {
        Self::new()
    }
}

impl ViewCube {
    /// The default on-screen edge length of the corner viewport, in CSS pixels —
    /// the ONE source of the cube's default size. `RenderSettings::viewcube_size_px`
    /// defaults to this so "settings default == widget default" holds even when no
    /// saved settings exist (the boot path that never runs `apply_settings_json`).
    pub const DEFAULT_SIZE_PX: f32 = 135.0;

    // --- named face regions (edges/corners via `region_id`) ------------------
    // Y-up (matches the world + `standard_view` buttons): FRONT/BACK on the Z
    // axis, TOP/BOTTOM on the Y axis, RIGHT/LEFT on X.
    pub const RIGHT: HandleId = Self::region_id(1, 0, 0);
    pub const LEFT: HandleId = Self::region_id(-1, 0, 0);
    pub const FRONT: HandleId = Self::region_id(0, 0, 1);
    pub const BACK: HandleId = Self::region_id(0, 0, -1);
    pub const TOP: HandleId = Self::region_id(0, 1, 0);
    pub const BOTTOM: HandleId = Self::region_id(0, -1, 0);

    // --- 2D navigation arrows (screen-fixed, OUTSIDE the 1..=27 cube ids) -----
    // Four pan/orbit triangles on the cube's sides + two roll arcs at the top
    // corners. These are UI controls pinned to the corner viewport (they do NOT
    // rotate with the cube); clicking one applies a RELATIVE camera rotation.
    pub const ARROW_UP: HandleId = 101;
    pub const ARROW_DOWN: HandleId = 102;
    pub const ARROW_LEFT: HandleId = 103;
    pub const ARROW_RIGHT: HandleId = 104;
    pub const ROLL_CW: HandleId = 105;
    pub const ROLL_CCW: HandleId = 106;

    /// True when `id` is one of the six screen-fixed navigation arrows (pan
    /// triangles / roll arcs) rather than a cube face/edge/corner region. The
    /// engine branches on this to apply a relative rotation instead of a snap.
    pub fn is_arrow(id: HandleId) -> bool {
        (Self::ARROW_UP..=Self::ROLL_CCW).contains(&id)
    }

    pub fn new() -> Self {
        Self { size: Self::DEFAULT_SIZE_PX, margin: 12.0 }
    }

    /// A ViewCube with a custom pixel footprint (used by the demo to render big).
    pub fn with_size(size: f32) -> Self {
        Self { size, margin: 12.0 }
    }

    pub fn size(&self) -> f32 {
        self.size
    }

    /// Pack axis signs (each in `-1,0,1`, not all zero) into a [`HandleId`].
    pub const fn region_id(sx: i32, sy: i32, sz: i32) -> HandleId {
        ((sx + 1) + (sy + 1) * 3 + (sz + 1) * 9 + 1) as HandleId
    }

    /// `1` = face, `2` = edge, `3` = corner. Navigation arrows are not cube
    /// regions, so they classify as `0` (never a face/edge/corner highlight).
    pub fn region_kind(id: HandleId) -> u8 {
        if Self::is_arrow(id) {
            return 0;
        }
        decode(id).iter().filter(|&&x| x != 0).count() as u8
    }

    /// The world-space eye→target direction the main camera should look along to
    /// view `id` (the negated region normal — see the module docs).
    pub fn target_view(id: HandleId) -> Vec3 {
        if Self::is_arrow(id) {
            // Arrows carry no absolute snap direction (the engine applies a
            // relative rotation); return a harmless front-view default (look -Z).
            return Vec3::new(0.0, 0.0, -1.0);
        }
        let s = decode(id);
        Vec3::new(s[0] as f32, s[1] as f32, s[2] as f32)
            .normalized()
            .scale(-1.0)
    }

    /// A world-space up hint for the snapped view: `+Y`, or the button's Z-based
    /// up for the straight top / bottom views where `+Y` is degenerate (TOP looks
    /// `-Y` → up `-Z`; BOTTOM looks `+Y` → up `+Z`). This matches the engine's
    /// Y-up `standard_view` so a face click and its named button agree exactly.
    pub fn target_up(id: HandleId) -> Vec3 {
        let v = Self::target_view(id);
        if v.y.abs() > 0.9 {
            // Straight top/bottom: up follows the look direction's Y sign, so
            // TOP (look -Y) → -Z and BOTTOM (look +Y) → +Z, as the buttons do.
            Vec3::new(0.0, 0.0, v.y.signum())
        } else {
            Vec3::Y
        }
    }

    /// Human-readable region name, e.g. `"TOP-FRONT-RIGHT"` (handy for tooltips /
    /// logging at integration time).
    pub fn region_name(id: HandleId) -> String {
        if Self::is_arrow(id) {
            return match id {
                Self::ARROW_UP => "ARROW-UP",
                Self::ARROW_DOWN => "ARROW-DOWN",
                Self::ARROW_LEFT => "ARROW-LEFT",
                Self::ARROW_RIGHT => "ARROW-RIGHT",
                Self::ROLL_CW => "ROLL-CW",
                Self::ROLL_CCW => "ROLL-CCW",
                _ => "ARROW",
            }
            .to_string();
        }
        let s = decode(id);
        let mut parts: Vec<&str> = Vec::new();
        // Y-up: TOP/BOTTOM on the Y axis, FRONT/BACK on the Z axis, RIGHT/LEFT on X.
        match s[1] {
            1 => parts.push("TOP"),
            -1 => parts.push("BOTTOM"),
            _ => {}
        }
        match s[2] {
            1 => parts.push("FRONT"),
            -1 => parts.push("BACK"),
            _ => {}
        }
        match s[0] {
            1 => parts.push("RIGHT"),
            -1 => parts.push("LEFT"),
            _ => {}
        }
        parts.join("-")
    }

    /// The bottom-right corner rect this gizmo owns: `[x, y, w, h]`, top-left
    /// origin, y down, in CSS pixels. The host uses it to decide whether to forward a
    /// pointer event and to offset it into cube-local coords.
    pub fn sub_rect(&self, viewport: [f32; 2]) -> [f32; 4] {
        let w = self.size.min(viewport[0]);
        let h = self.size.min(viewport[1]);
        let x = (viewport[0] - w - self.margin).max(0.0);
        let y = (viewport[1] - h - self.margin).max(0.0);
        [x, y, w, h]
    }

    /// The mini-camera that renders the cube: same rotation as `main` — forward
    /// AND up, so the cube mirrors the camera's roll exactly (see the module
    /// docs) — positioned a fixed distance back looking at the cube origin,
    /// orthographic, viewport = `size`×`size`. The engine renders the ViewCube
    /// overlay with THIS camera.
    pub fn mini_camera(&self, main: &GizmoCamera) -> GizmoCamera {
        let f = main.forward.normalized();
        let up = cube_up(f, main.up);
        let dist = 4.0;
        let eye = f.scale(-dist);
        // Frame with margin so the corner spheres + tube edges (~0.82 at the
        // diagonal) AND the pan/roll nav arrows (pushed out to ~1.18 so they clear
        // the cube's clickable region) both fit without clipping.
        let half = 1.25;
        let view_proj = crate::math::ortho_view_proj(eye, f.normalized(), up, half, self.size, self.size);
        GizmoCamera {
            view_proj,
            eye,
            forward: f,
            up,
            viewport: [self.size, self.size],
            orthographic: true,
        }
    }

    /// Emit the oriented cube geometry, back-faces culled to the given view
    /// direction, with the hovered / active region highlighted.
    fn build(&self, view_forward: Vec3, hovered: Option<HandleId>, active: Option<HandleId>) -> Overlay {
        let mut o = Overlay::new();
        let h = 0.5f32;
        let fwd = view_forward.normalized();
        let hi = active.or(hovered);
        let hi_kind = hi.map(ViewCube::region_kind);

        // Faces (front-facing only) + stroked letters.
        for face in faces() {
            if face.n.dot(fwd) >= -1e-3 {
                continue; // back-facing
            }
            let (fa, fsgn) = face_axis_sign(face.n);
            let touches = hi.map_or(false, |hid| decode(hid)[fa] == fsgn);
            let col = if active == Some(face.id) {
                COL_ACTIVE
            } else if hovered == Some(face.id) {
                COL_HOVER
            } else if touches && matches!(hi_kind, Some(2) | Some(3)) {
                brighten_by(face.color, 0.16)
            } else {
                face.color
            };
            let c = face.n.scale(h);
            let corner = |su: f32, sv: f32| {
                c.add(face.u.scale(su * h)).add(face.v.scale(sv * h))
            };
            let p00 = corner(-1.0, -1.0);
            let p10 = corner(1.0, -1.0);
            let p11 = corner(1.0, 1.0);
            let p01 = corner(-1.0, 1.0);
            o.tri(p00, p10, p11, col);
            o.tri(p00, p11, p01, col);

            // Letter(s), lifted just proud of the face so they read on top.
            // Multi-char labels (e.g. "BK") are laid out in equal horizontal slots.
            let gc = face.n.scale(h + 0.012);
            let chars: Vec<char> = face.glyph.chars().collect();
            let nch = chars.len().max(1) as f32;
            for (i, ch) in chars.iter().enumerate() {
                let slot = |p: [f32; 2]| [(i as f32 + p[0]) / nch, p[1]];
                for stroke in letter_strokes(*ch) {
                    for seg in stroke.windows(2) {
                        let a = glyph_point(gc, face.u, face.v, slot(seg[0]));
                        let b = glyph_point(gc, face.u, face.v, slot(seg[1]));
                        o.line(a, b, COL_GLYPH);
                    }
                }
            }
        }

        // The 12 edges (only those touching a front-facing face), drawn as
        // thick camera-facing ribbons — the original view cube's tube edges.
        const TUBE_W: f32 = 0.052; // half-thickness of the edge tube (cube units)
        for (es, a, b) in edges() {
            let front = (0..3).any(|axis| {
                es[axis] != 0 && axis_vec(axis, es[axis] as f32).dot(fwd) < -1e-3
            });
            if !front {
                continue;
            }
            let eid = ViewCube::region_id(es[0], es[1], es[2]);
            let hot = hi == Some(eid)
                || (hi_kind == Some(3) && region_contains(hi.unwrap(), es));
            let col = if hot { COL_EDGE_HI } else { COL_EDGE };
            // Ribbon perpendicular to the edge in screen space (a "tube" facing
            // the camera). Falls back to a line when the edge points at the eye.
            let perp = b.sub(a).cross(fwd);
            if perp.length() > 1e-5 {
                let w = perp.normalized().scale(TUBE_W);
                o.tri(a.add(w), b.add(w), b.sub(w), col);
                o.tri(a.add(w), b.sub(w), a.sub(w), col);
            } else {
                o.line(a, b, col);
            }
        }

        // Blue corner spheres (billboarded quads) at the visible corners — the
        // rounded corner nodes of the original view cube.
        let right0 = fwd.any_perp();
        let up0 = fwd.cross(right0).normalized();
        let cr = 0.11f32;
        const CORNER_SEGS: usize = 12;
        for (sx, sy, sz) in [
            (-1.0, -1.0, -1.0), (1.0, -1.0, -1.0), (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0), (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
        ] {
            let visible = axis_vec(0, sx).dot(fwd) < -1e-3
                || axis_vec(1, sy).dot(fwd) < -1e-3
                || axis_vec(2, sz).dot(fwd) < -1e-3;
            if !visible {
                continue;
            }
            // Round node (camera-facing disc) so it reads as a corner sphere.
            let c = Vec3::new(sx * h, sy * h, sz * h);
            let mut prev = c.add(right0.scale(cr));
            for k in 1..=CORNER_SEGS {
                let a = (k as f32 / CORNER_SEGS as f32) * std::f32::consts::TAU;
                let cur = c
                    .add(right0.scale(a.cos() * cr))
                    .add(up0.scale(a.sin() * cr));
                o.tri(c, prev, cur, COL_CORNER);
                prev = cur;
            }
        }

        // Bright marker for a hovered/active edge or corner.
        if let Some(hid) = hi {
            match ViewCube::region_kind(hid) {
                2 => edge_bevel(&mut o, decode(hid), COL_MARK),
                3 => corner_cap(&mut o, decode(hid), COL_MARK, fwd),
                _ => {}
            }
        }

        o
    }

    /// Ray-cast the cube (unit box `[-0.5, 0.5]^3`) and classify the entry point
    /// into a face / edge / corner region.
    fn raycast(&self, ray: &Ray) -> Option<HandleId> {
        let h = 0.5f32;
        let o = [ray.origin.x, ray.origin.y, ray.origin.z];
        let d = [ray.dir.x, ray.dir.y, ray.dir.z];
        let mut tmin = f32::NEG_INFINITY;
        let mut tmax = f32::INFINITY;
        for a in 0..3 {
            if d[a].abs() < 1e-9 {
                if o[a] < -h || o[a] > h {
                    return None;
                }
            } else {
                let mut t1 = (-h - o[a]) / d[a];
                let mut t2 = (h - o[a]) / d[a];
                if t1 > t2 {
                    std::mem::swap(&mut t1, &mut t2);
                }
                tmin = tmin.max(t1);
                tmax = tmax.min(t2);
                if tmin > tmax {
                    return None;
                }
            }
        }
        let t = if tmin >= 0.0 {
            tmin
        } else if tmax >= 0.0 {
            tmax
        } else {
            return None;
        };
        Some(classify(ray.at(t)))
    }

    // --- navigation arrows (screen-fixed 2D controls) -----------------------

    /// Draw the four pan/orbit triangles (N/S/E/W) and two roll arcs (top
    /// corners). Vertices are placed at `right*sx + up*sy` where `right`/`up`
    /// are the mini-camera's screen axes, so every arrow projects to the SAME
    /// spot in the corner sub-rect regardless of cube orientation. `sx,sy` are
    /// in ~[-1,1]; the cube silhouette lives within |s| ≲ 0.82, so the pan
    /// triangles sit just outside it and the roll arcs hug the top corners.
    fn draw_arrows(&self, o: &mut Overlay, fwd: Vec3, cam_up: Vec3, hovered: Option<HandleId>) {
        let (right, up) = screen_axes(fwd, cam_up);
        let w2 = |sx: f32, sy: f32| right.scale(sx).add(up.scale(sy));

        // Pan/orbit triangles, tip pointing outward.
        const RB: f32 = 0.98; // base ring radius (pushed clear of the cube's
        // clickable region so the arrows don't steal cube clicks)
        const RT: f32 = 1.18; // tip radius (inside the widened sub-rect edge)
        const HW: f32 = 0.135; // half base width
        for (id, dx, dy) in PAN_ARROWS {
            let col = if hovered == Some(id) { COL_ARROW_HI } else { COL_ARROW };
            let (px, py) = (-dy, dx); // in-plane perpendicular
            let tip = w2(dx * RT, dy * RT);
            let b0 = w2(dx * RB + px * HW, dy * RB + py * HW);
            let b1 = w2(dx * RB - px * HW, dy * RB - py * HW);
            o.tri(tip, b0, b1, col);
            o.line(tip, b0, COL_ARROW_LINE);
            o.line(b0, b1, COL_ARROW_LINE);
            o.line(b1, tip, COL_ARROW_LINE);
        }

        // Roll arcs at the top corners (curved ribbon + arrowhead).
        for (id, cx, cy, spin) in ROLL_ARROWS {
            let col = if hovered == Some(id) { COL_ARROW_HI } else { COL_ARROW };
            draw_roll_arc(o, &w2, [cx, cy], spin, col);
        }
    }

    /// Hit-test the screen-fixed nav arrows: project each arrow's fixed anchor
    /// to sub-rect pixels via the mini-camera and pick the nearest one within a
    /// small radius of the incoming (cube-local px) `screen` point. Returns the
    /// arrow handle, or None to fall through to the cube raycast.
    fn arrow_hit(&self, mini: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId> {
        let (right, up) = screen_axes(mini.forward, mini.up);
        let w2 = |sx: f32, sy: f32| right.scale(sx).add(up.scale(sy));
        let mut best: Option<(f32, HandleId)> = None;
        let mut consider = |id: HandleId, anchor: Vec3, radius: f32| {
            if let Some(px) = mini.world_to_screen(anchor) {
                let d = ((px[0] - screen[0]).powi(2) + (px[1] - screen[1]).powi(2)).sqrt();
                if d <= radius && best.map_or(true, |(bd, _)| d < bd) {
                    best = Some((d, id));
                }
            }
        };
        // Pan triangles: click target at the triangle centroid.
        const RC: f32 = (1.18 + 2.0 * 0.98) / 3.0; // (RT + 2*RB)/3
        let pan_r = self.size * 0.13;
        for (id, dx, dy) in PAN_ARROWS {
            consider(id, w2(dx * RC, dy * RC), pan_r);
        }
        // Roll arcs: click target at the arc apex (straight up from the center).
        let roll_r = self.size * 0.18;
        for (id, cx, cy, _spin) in ROLL_ARROWS {
            consider(id, w2(cx, cy + ROLL_ARC_R), roll_r);
        }
        best.map(|(_, id)| id)
    }
}

/// The cube's render-up: the MAIN camera's up re-orthonormalized against the
/// view forward, so the cube mirrors the camera's roll exactly. Falls back to
/// the Y-up rule (+Y, or +Z when looking straight up/down the Y axis) only when
/// the supplied up is degenerate — near-parallel to `f` (never for a valid
/// camera). Shared by [`ViewCube::mini_camera`] and [`screen_axes`] so the cube
/// render, the cube raycast and the nav-arrow anchors all live in ONE frame.
fn cube_up(f: Vec3, up: Vec3) -> Vec3 {
    let right = f.cross(up);
    if right.length() > 1e-4 {
        return right.normalized().cross(f).normalized();
    }
    let up_hint = if f.y.abs() > 0.9 { Vec3::Z } else { Vec3::Y };
    f.cross(up_hint).normalized().cross(f).normalized()
}

/// The mini-camera's screen axes (right, up) derived from the view forward + the
/// main camera's up — identical to the axes [`ViewCube::mini_camera`] builds its
/// view matrix from, so a point at `right*sx + up*sy` lands at a fixed sub-rect
/// screen position no matter how the camera is oriented or rolled.
fn screen_axes(fwd: Vec3, cam_up: Vec3) -> (Vec3, Vec3) {
    let f = fwd.normalized();
    let up = cube_up(f, cam_up);
    let right = f.cross(up).normalized();
    (right, up)
}

/// Pan/orbit triangles: `(handle, outward_x, outward_y)` in screen-axis space.
const PAN_ARROWS: [(HandleId, f32, f32); 4] = [
    (ViewCube::ARROW_UP, 0.0, 1.0),
    (ViewCube::ARROW_DOWN, 0.0, -1.0),
    (ViewCube::ARROW_RIGHT, 1.0, 0.0),
    (ViewCube::ARROW_LEFT, -1.0, 0.0),
];

/// Roll arcs: `(handle, center_x, center_y, spin)` (spin +1 = CCW, -1 = CW).
const ROLL_ARROWS: [(HandleId, f32, f32, f32); 2] = [
    (ViewCube::ROLL_CCW, -0.78, 0.78, 1.0),
    (ViewCube::ROLL_CW, 0.78, 0.78, -1.0),
];

/// Radius of the roll arc (screen-axis units); the click apex sits `+ROLL_ARC_R`
/// above the arc center.
const ROLL_ARC_R: f32 = 0.19;

/// Draw one roll arrow as a thick circular ribbon spanning ~171° over the top
/// of `center`, with a triangular arrowhead at the swept end pointing along the
/// direction of rotation (`spin` +1 = CCW, -1 = CW).
fn draw_roll_arc<F: Fn(f32, f32) -> Vec3>(
    o: &mut Overlay,
    w2: &F,
    center: [f32; 2],
    spin: f32,
    col: [f32; 4],
) {
    let r = ROLL_ARC_R;
    let th = 0.05f32; // ribbon half-thickness
    let sweep = std::f32::consts::PI * 0.95; // ~171°, centered on straight-up
    let mid = std::f32::consts::FRAC_PI_2;
    let a0 = mid - spin * sweep * 0.5;
    let a1 = mid + spin * sweep * 0.5;
    let pt = |ang: f32, rad: f32| w2(center[0] + rad * ang.cos(), center[1] + rad * ang.sin());
    const SEGS: usize = 12;
    let mut prev = a0;
    for k in 1..=SEGS {
        let ang = a0 + (a1 - a0) * (k as f32 / SEGS as f32);
        let i0 = pt(prev, r - th);
        let o0 = pt(prev, r + th);
        let i1 = pt(ang, r - th);
        let o1 = pt(ang, r + th);
        o.tri(i0, o0, o1, col);
        o.tri(i0, o1, i1, col);
        prev = ang;
    }
    // Arrowhead at the swept end (a1), pointing along the travel tangent.
    let (s1, c1) = a1.sin_cos();
    let rad_dir = [c1, s1];
    let tangent = [-s1 * spin, c1 * spin];
    let hl = 0.15f32; // head length
    let hw = 0.11f32; // head half-width
    let tip = w2(
        center[0] + r * rad_dir[0] + tangent[0] * hl,
        center[1] + r * rad_dir[1] + tangent[1] * hl,
    );
    let base0 = w2(center[0] + (r + hw) * rad_dir[0], center[1] + (r + hw) * rad_dir[1]);
    let base1 = w2(center[0] + (r - hw) * rad_dir[0], center[1] + (r - hw) * rad_dir[1]);
    o.tri(tip, base0, base1, col);
}

impl Gizmo for ViewCube {
    fn geometry(&self, camera: &GizmoCamera, hovered: Option<HandleId>, active: Option<HandleId>) -> Overlay {
        // Orientation comes from the camera's rotation (mini_camera mirrors
        // forward AND up); cube vertex positions are world-axis-fixed, so the
        // face/edge geometry consumes only forward (culling), while the
        // screen-pinned nav arrows need the full (forward, up) frame.
        let mut o = self.build(camera.forward, hovered, active);
        // The nav arrows are 2D controls pinned to the corner viewport — they
        // are placed along the mini-camera's screen axes so they stay fixed no
        // matter how the cube is oriented, and are drawn on top of the cube.
        self.draw_arrows(&mut o, camera.forward, camera.up, hovered);
        o
    }

    fn hit(&self, camera: &GizmoCamera, screen: [f32; 2]) -> Option<HandleId> {
        // `screen` is cube-local (0..size, top-left origin) — see module docs.
        let mini = self.mini_camera(camera);
        // Screen-fixed nav arrows sit outside the cube silhouette; test them
        // first so a click on an arrow never falls through to the cube.
        if let Some(id) = self.arrow_hit(&mini, screen) {
            return Some(id);
        }
        let ray = mini.ray_from_screen(screen[0], screen[1]);
        // The corner spheres protrude past the box, so hit-test them directly —
        // the whole sphere is clickable and selects that corner region. Nearest
        // (front-most) visible sphere wins over the box hit.
        let fwd = mini.forward.normalized();
        let h = 0.5f32;
        let cr = 0.135f32; // corner-node radius + a little click tolerance
        let mut best: Option<(f32, HandleId)> = None;
        for (sx, sy, sz) in [
            (-1.0f32, -1.0f32, -1.0f32), (1.0, -1.0, -1.0), (1.0, 1.0, -1.0), (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0), (1.0, -1.0, 1.0), (1.0, 1.0, 1.0), (-1.0, 1.0, 1.0),
        ] {
            let visible = axis_vec(0, sx).dot(fwd) < -1e-3
                || axis_vec(1, sy).dot(fwd) < -1e-3
                || axis_vec(2, sz).dot(fwd) < -1e-3;
            if !visible {
                continue;
            }
            let c = Vec3::new(sx * h, sy * h, sz * h);
            if ray.distance_to_point(c) <= cr {
                let t = c.sub(ray.origin).dot(ray.dir);
                if best.map_or(true, |(bt, _)| t < bt) {
                    best = Some((t, ViewCube::region_id(sx as i32, sy as i32, sz as i32)));
                }
            }
        }
        if let Some((_, id)) = best {
            return Some(id);
        }
        self.raycast(&ray)
    }
}

// --- region math -----------------------------------------------------------

fn decode(id: HandleId) -> [i32; 3] {
    let v = id as i32 - 1;
    [v % 3 - 1, (v / 3) % 3 - 1, (v / 9) % 3 - 1]
}

/// Classify a point on the box surface into a region by counting how many axes
/// are "extreme" (within [`EDGE_BAND`] of a face). One extreme → face, two →
/// edge, three → corner.
fn classify(p: Vec3) -> HandleId {
    let h = 0.5f32;
    let band = h * EDGE_BAND;
    let c = [p.x, p.y, p.z];
    let mut s = [0i32; 3];
    for a in 0..3 {
        if c[a] >= h - band {
            s[a] = 1;
        } else if c[a] <= -(h - band) {
            s[a] = -1;
        }
    }
    if s == [0, 0, 0] {
        // Interior of a face: snap to the dominant (entry) axis.
        let mut da = 0usize;
        for a in 1..3 {
            if c[a].abs() > c[da].abs() {
                da = a;
            }
        }
        s[da] = if c[da] >= 0.0 { 1 } else { -1 };
    }
    ViewCube::region_id(s[0], s[1], s[2])
}

/// True if the edge (signs `es`, two nonzero) lies on the corner `corner_id`.
fn region_contains(corner_id: HandleId, es: [i32; 3]) -> bool {
    let cs = decode(corner_id);
    (0..3).all(|a| es[a] == 0 || cs[a] == es[a])
}

fn face_axis_sign(n: Vec3) -> (usize, i32) {
    if n.x.abs() > 0.5 {
        (0, if n.x > 0.0 { 1 } else { -1 })
    } else if n.y.abs() > 0.5 {
        (1, if n.y > 0.0 { 1 } else { -1 })
    } else {
        (2, if n.z > 0.0 { 1 } else { -1 })
    }
}

fn axis_vec(axis: usize, s: f32) -> Vec3 {
    match axis {
        0 => Vec3::new(s, 0.0, 0.0),
        1 => Vec3::new(0.0, s, 0.0),
        _ => Vec3::new(0.0, 0.0, s),
    }
}

// --- cube geometry tables --------------------------------------------------

struct Face {
    id: HandleId,
    /// Outward normal.
    n: Vec3,
    /// In-plane axis that is screen-right when viewing the face head-on.
    u: Vec3,
    /// In-plane axis that is screen-up (`u × v == n`, so quads wind outward).
    v: Vec3,
    glyph: &'static str,
    /// Base face fill color (the original view-cube color scheme).
    color: [f32; 4],
}

fn faces() -> [Face; 6] {
    // Y-up cube: FRONT/BACK live on ±Z, TOP/BOTTOM on ±Y, RIGHT/LEFT on ±X, so
    // each face matches the same-named `standard_view` button. Per face, `u`/`v`
    // are the screen-right / screen-up axes when viewing head-on from that
    // button's vantage (world +Y up, with TOP/BOTTOM using the button's ∓Z/±Z
    // up), so the letters read upright; `u × v == n` keeps the quad wound
    // outward. Glyphs + hues stay bound to each named face.
    [
        // Original view-cube per-face hues (ff4d4d/005eff/55ff00/ffea00/ff0084/00e5ff).
        Face { id: ViewCube::RIGHT, n: Vec3::new(1.0, 0.0, 0.0), u: Vec3::new(0.0, 0.0, -1.0), v: Vec3::new(0.0, 1.0, 0.0), glyph: "R", color: [1.000, 0.302, 0.302, 1.0] },
        Face { id: ViewCube::LEFT, n: Vec3::new(-1.0, 0.0, 0.0), u: Vec3::new(0.0, 0.0, 1.0), v: Vec3::new(0.0, 1.0, 0.0), glyph: "L", color: [0.000, 0.369, 1.000, 1.0] },
        Face { id: ViewCube::BACK, n: Vec3::new(0.0, 0.0, -1.0), u: Vec3::new(-1.0, 0.0, 0.0), v: Vec3::new(0.0, 1.0, 0.0), glyph: "BK", color: [0.000, 0.898, 1.000, 1.0] },
        Face { id: ViewCube::FRONT, n: Vec3::new(0.0, 0.0, 1.0), u: Vec3::new(1.0, 0.0, 0.0), v: Vec3::new(0.0, 1.0, 0.0), glyph: "F", color: [1.000, 0.000, 0.518, 1.0] },
        Face { id: ViewCube::TOP, n: Vec3::new(0.0, 1.0, 0.0), u: Vec3::new(1.0, 0.0, 0.0), v: Vec3::new(0.0, 0.0, -1.0), glyph: "T", color: [0.333, 1.000, 0.000, 1.0] },
        Face { id: ViewCube::BOTTOM, n: Vec3::new(0.0, -1.0, 0.0), u: Vec3::new(1.0, 0.0, 0.0), v: Vec3::new(0.0, 0.0, 1.0), glyph: "B", color: [1.000, 0.918, 0.000, 1.0] },
    ]
}

/// The 12 edges as `(signs, endpoint_a, endpoint_b)`.
fn edges() -> Vec<([i32; 3], Vec3, Vec3)> {
    let h = 0.5f32;
    let mut out = Vec::with_capacity(12);
    // (i, j) are the two extreme axes; k is the axis the edge runs along.
    for &(i, j, k) in &[(0usize, 1usize, 2usize), (0, 2, 1), (1, 2, 0)] {
        for &si in &[-1i32, 1] {
            for &sj in &[-1i32, 1] {
                let mut s = [0i32; 3];
                s[i] = si;
                s[j] = sj;
                let mut e0 = [0.0f32; 3];
                e0[i] = si as f32 * h;
                e0[j] = sj as f32 * h;
                e0[k] = h;
                let mut e1 = e0;
                e1[k] = -h;
                out.push((s, Vec3::from(e0), Vec3::from(e1)));
            }
        }
    }
    out
}

/// Map a letter-space point (`[0,1]^2`, origin bottom-left) onto a face plane.
fn glyph_point(center: Vec3, u: Vec3, v: Vec3, p: [f32; 2]) -> Vec3 {
    const SCALE: f32 = 0.60; // letter spans 60% of the 1.0-wide face
    let du = (p[0] - 0.5) * SCALE;
    let dv = (p[1] - 0.5) * SCALE;
    center.add(u.scale(du)).add(v.scale(dv))
}

/// Stroked line-glyphs (polylines in `[0,1]^2`, origin bottom-left) for the six
/// face letters. Blocky approximations — enough to orient, no font needed.
fn letter_strokes(glyph: char) -> Vec<Vec<[f32; 2]>> {
    match glyph {
        'F' => vec![
            vec![[0.18, 0.10], [0.18, 0.90]],
            vec![[0.18, 0.90], [0.78, 0.90]],
            vec![[0.18, 0.52], [0.64, 0.52]],
        ],
        'B' => vec![
            vec![[0.18, 0.10], [0.18, 0.90]],
            vec![[0.18, 0.90], [0.62, 0.90], [0.74, 0.78], [0.74, 0.64], [0.62, 0.52], [0.18, 0.52]],
            vec![[0.18, 0.52], [0.66, 0.52], [0.80, 0.40], [0.80, 0.22], [0.66, 0.10], [0.18, 0.10]],
        ],
        'R' => vec![
            vec![[0.18, 0.10], [0.18, 0.90]],
            vec![[0.18, 0.90], [0.62, 0.90], [0.74, 0.78], [0.74, 0.64], [0.62, 0.52], [0.18, 0.52]],
            vec![[0.42, 0.52], [0.78, 0.10]],
        ],
        'L' => vec![vec![[0.24, 0.90], [0.24, 0.10], [0.78, 0.10]]],
        'K' => vec![
            vec![[0.20, 0.10], [0.20, 0.90]],
            vec![[0.20, 0.48], [0.80, 0.90]],
            vec![[0.20, 0.48], [0.80, 0.10]],
        ],
        'T' => vec![
            vec![[0.12, 0.90], [0.88, 0.90]],
            vec![[0.50, 0.90], [0.50, 0.10]],
        ],
        'D' => vec![
            vec![[0.20, 0.10], [0.20, 0.90]],
            vec![[0.20, 0.90], [0.55, 0.90], [0.76, 0.72], [0.82, 0.50], [0.76, 0.28], [0.55, 0.10], [0.20, 0.10]],
        ],
        _ => Vec::new(),
    }
}

// --- highlight markers -----------------------------------------------------

/// A bright chamfer strip straddling a hovered/active edge.
fn edge_bevel(o: &mut Overlay, es: [i32; 3], col: [f32; 4]) {
    let h = 0.5f32;
    let eps = 0.014f32;
    let d = 0.16f32;
    let k = (0..3).find(|&a| es[a] == 0).unwrap();
    let rest: Vec<usize> = (0..3).filter(|&a| a != k).collect();
    let (i, j) = (rest[0], rest[1]);
    let si = es[i] as f32;
    let sj = es[j] as f32;
    let mk = |ival: f32, jval: f32, kval: f32| {
        let mut arr = [0.0f32; 3];
        arr[i] = ival;
        arr[j] = jval;
        arr[k] = kval;
        Vec3::from(arr)
    };
    let a0 = mk(si * (h + eps), sj * (h - d), h);
    let a1 = mk(si * (h + eps), sj * (h - d), -h);
    let b0 = mk(si * (h - d), sj * (h + eps), h);
    let b1 = mk(si * (h - d), sj * (h + eps), -h);
    o.tri(a0, a1, b1, col);
    o.tri(a0, b1, b0, col);
}

/// A bright cap (one small triangle per adjacent front-facing face) at a
/// hovered/active corner.
fn corner_cap(o: &mut Overlay, cs: [i32; 3], col: [f32; 4], fwd: Vec3) {
    let h = 0.5f32;
    let eps = 0.016f32;
    let d = 0.20f32;
    for a in 0..3 {
        let sa = cs[a] as f32;
        if axis_vec(a, sa).dot(fwd) >= -1e-3 {
            continue; // this face is back-facing
        }
        let rest: Vec<usize> = (0..3).filter(|&x| x != a).collect();
        let (b, cc) = (rest[0], rest[1]);
        let sb = cs[b] as f32;
        let sc = cs[cc] as f32;
        let mk = |av: f32, bv: f32, cv: f32| {
            let mut arr = [0.0f32; 3];
            arr[a] = av;
            arr[b] = bv;
            arr[cc] = cv;
            Vec3::from(arr)
        };
        let p0 = mk(sa * (h + eps), sb * h, sc * h);
        let p1 = mk(sa * (h + eps), sb * (h - d), sc * h);
        let p2 = mk(sa * (h + eps), sb * h, sc * (h - d));
        o.tri(p0, p1, p2, col);
    }
}

