//! In-scene overlay widgets: the engine-side registry that hosts the
//! `brep-gizmos` gizmos natively. It owns the widget STATE (fed as plain JSON
//! over the R3 boundary — datum planes/axes/frames, curve display, feature
//! dimensions, the transform gizmo, the always-on ViewCube), constructs a
//! [`GizmoCamera`] from the engine's live [`ViewCamera`] each frame, and emits
//! the `brep-gizmos` [`Overlay`] geometry the render core converts into overlay
//! GPU buffers. It also answers pointer hit-tests (ViewCube snap target, datum
//! pick, transform-gizmo hover/pick) and computes transform drag deltas.
//!
//! No renderer, no GPU: pure geometry + hit-testing, exactly like the gizmo crate.
//! The host UI keeps the drag→feature-commit logic and the dimension text labels;
//! the engine supplies the geometry and the frame-space deltas / label anchors.
//!
//! # Overlay-feed API surface (the contract the Rust UI programs against)
//!
//! Five feeds set overlay geometry. Each is `wasm Engine.* → EngineState.* →`
//! the `WidgetRegistry` method named below. Four are SPECIALIZED (they carry
//! structured state their own pick/anchor/interaction reads); one is GENERAL
//! (arbitrary geometry, no interaction). The general channel is the ONE uniform
//! feed; the specialized feeds stay specialized (see "Why not one feed").
//!
//! | Feed (wasm)           | Registry setter        | Geometry it carries                                   | Screen-constant? | Specialized query / interaction |
//! |-----------------------|------------------------|-------------------------------------------------------|------------------|---------------------------------|
//! | `set_datums`          | `set_datums_json`    | datum planes (world- or screen-sized), axes, frames, curves | frames + screen-sized planes: yes | `datum_pick` → hit plane/axis name |
//! | `set_dimensions`      | `set_dimensions_json`| linear / angular / radial dimension leaders + arrows  | yes              | `dimension_anchors` → `(id, world label anchor)` (the host projects to place text) |
//! | `set_overlay`         | `set_overlay_json`   | GENERAL named groups of raw tris / lines / points     | no (points billboard) | none — pure display geometry |
//! | `set_transform_gizmo` | `set_transform_json` | the move+rotate gizmo at a feature frame               | yes              | `transform_hover` / `transform_pick` / `transform_drag` / `transform_drag_end` |
//! | `set_viewcube_enabled`| `set_viewcube_enabled`| the always-on ViewCube (own mini-camera + corner rect)| yes              | `viewcube_rect` / `viewcube_hover` / `viewcube_clear_hover` / `viewcube_click` |
//!
//! ## Draw path
//! `build_main_overlay` merges datums + dimensions + transform + the general
//! `set_overlay` groups into ONE full-viewport [`Overlay`] drawn in the render
//! core's depth-cleared overlay pass, in a fixed order: planes, axes, frames,
//! curves, dimensions, transform gizmo, then the general groups sorted by their
//! `renderOrder`. The ViewCube alone draws in its own pass (`build_viewcube`),
//! with a mini-camera in a scissored corner rect.
//!
//! ## Why not one uniform feed (rewrite-time note)
//! The general `set_overlay` groups pre-expand their tris/lines into GPU-ready
//! vertices AT FEED TIME (only point billboards are rebuilt per frame). The
//! specialized widgets can't: dimension leaders, `datum_frame`s, screen-sized
//! `datum_plane`s, the transform gizmo, and the ViewCube are all SCREEN-CONSTANT
//! — sized from `world_per_pixel` against the LIVE camera every frame — so they
//! must be rebuilt in `build_main_overlay`, not stored pre-expanded. They also
//! carry structured state their interaction needs (datum names for
//! `datum_pick`, dimension ids+types for `dimension_anchors`, the feature
//! frame for `transform_drag`, region handles for the ViewCube). Routing any of
//! them through the flat `set_overlay` group buffers would drop either the
//! zoom-invariant sizing or that interaction, so it is NOT done here. The one
//! genuinely static subset (world-sized planes, axes, curves) is left on
//! `set_datums` rather than fragmenting a single feed across two paths. Any
//! deeper unification is deferred to the engine-native UI rewrite.

use crate::style::parse_css_hex;
use crate::view::{Projection, ViewCamera, ViewFrame};
use brep_gizmos::datum::{DatumAxis, DatumPlane};
use brep_gizmos::transform::{DragDelta, TransformGizmo};
use brep_gizmos::view_cube::ViewCube;
use brep_gizmos::{
    curve_display, datum, dimension, Gizmo, GizmoCamera, HandleId, LineVertex, Overlay, TriVertex,
    Vec3,
};
use serde_json::Value;

/// Build the gizmo camera the widgets consume from the engine's live camera.
/// The `view_proj` is byte-identical to the render core's `Camera::view_proj`
/// (both from [`ViewCamera::resolve`]), so widgets project to exactly the view
/// the engine renders solids with. `viewport` is CSS pixels (what the gizmo
/// screen-constant sizing keys off).
pub fn gizmo_camera(view: &ViewCamera) -> GizmoCamera {
    let resolved = view.resolve();
    // The TRUE orthonormal basis the view matrix is built from — `up` is the
    // camera's actual (arcball-rolled) up, not `view.up` raw, so widgets that
    // mirror the camera's orientation (the ViewCube) match the render exactly.
    let (_, up, forward) = view.basis();
    GizmoCamera {
        view_proj: resolved.view_proj,
        eye: v3f(view.eye),
        forward: v3f(forward),
        up: v3f(up),
        viewport: [view.width as f32, view.height as f32],
        orthographic: matches!(view.projection, Projection::Orthographic { .. }),
    }
}

fn v3f(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0] as f32, a[1] as f32, a[2] as f32)
}

fn vec3_of(v: &Value) -> Option<Vec3> {
    let a = v.as_array()?;
    Some(Vec3::new(
        a.first()?.as_f64()? as f32,
        a.get(1)?.as_f64()? as f32,
        a.get(2)?.as_f64()? as f32,
    ))
}

fn color_of(v: Option<&Value>, default: [f32; 4]) -> [f32; 4] {
    match v.and_then(|v| v.as_str()).and_then(parse_css_hex) {
        Some(rgb) => [rgb[0], rgb[1], rgb[2], 1.0],
        None => default,
    }
}

fn flag(v: &Value, key: &str) -> bool {
    v.get(key).and_then(|b| b.as_bool()).unwrap_or(false)
}

/// A flat `[f32, …]` array from a JSON field (missing / wrong-typed → empty).
fn f32_array(v: Option<&Value>) -> Vec<f32> {
    v.and_then(|v| v.as_array())
        .map(|a| a.iter().filter_map(|x| x.as_f64().map(|f| f as f32)).collect())
        .unwrap_or_default()
}

/// The RGBA of vertex `i` from a flat rgb color array (last color repeats past
/// the end; white when none), alpha forced to 1.
fn rgb_at(colors: &[f32], i: usize) -> [f32; 4] {
    let base = i * 3;
    if base + 2 < colors.len() {
        [colors[base], colors[base + 1], colors[base + 2], 1.0]
    } else if colors.len() >= 3 {
        let n = colors.len();
        [colors[n - 3], colors[n - 2], colors[n - 1], 1.0]
    } else {
        [1.0, 1.0, 1.0, 1.0]
    }
}

/// `[f32;3]` position of index `i` from a flat xyz array (zero past the end).
fn pos_at(positions: &[f32], i: usize) -> Vec3 {
    let base = i * 3;
    if base + 2 < positions.len() {
        Vec3::new(positions[base], positions[base + 1], positions[base + 2])
    } else {
        Vec3::ZERO
    }
}

/// Emit a camera-facing, screen-constant-size quad (two tris) for an overlay
/// point at `center`. Reuses the overlay tri pass — no point pipeline needed.
fn push_point_quad(ov: &mut Overlay, center: Vec3, color: [f32; 4], size_px: f32, cam: &GizmoCamera) {
    let half = 0.5 * size_px.max(1.0) * cam.world_per_pixel(center);
    let right = cam.screen_right(center);
    let up = right.cross(cam.forward).normalized();
    let rx = right.scale(half);
    let uy = up.scale(half);
    let normal: [f32; 3] = cam.forward.scale(-1.0).into();
    let a = center.sub(rx).sub(uy);
    let b = center.add(rx).sub(uy);
    let c = center.add(rx).add(uy);
    let d = center.sub(rx).add(uy);
    for p in [a, b, c, a, c, d] {
        ov.tris.push(TriVertex { pos: p.into(), normal, color });
    }
}

/// Brighten a display color for the selected/hovered emphasis (matches the
/// gizmo crate's private `brighten`).
fn brighten(c: [f32; 4]) -> [f32; 4] {
    [
        (c[0] * 1.35 + 0.1).min(1.0),
        (c[1] * 1.35 + 0.1).min(1.0),
        (c[2] * 1.35 + 0.1).min(1.0),
        c[3],
    ]
}

const DEFAULT_AXIS_COLOR: [f32; 4] = [0.72, 0.74, 0.80, 1.0];

// --- widget state ----------------------------------------------------------

struct PlaneW {
    name: String,
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    size: Option<f32>,
    color: [f32; 4],
    hot: bool,
}

struct AxisW {
    name: String,
    point: Vec3,
    direction: Vec3,
    length: f32,
    color: [f32; 4],
    hot: bool,
}

struct FrameW {
    origin: Vec3,
    x: Vec3,
    y: Vec3,
    z: Vec3,
    px: f32,
}

struct CurveW {
    points: Vec<Vec3>,
    closed: bool,
    color: [f32; 4],
}

/// One screen-constant-size overlay point (billboarded to a small camera-facing
/// quad each frame, since the overlay pass has no dedicated point pipeline).
struct OverlayPointW {
    center: Vec3,
    color: [f32; 4],
}

/// A GENERAL named overlay group (`set_overlay`): arbitrary triangle + line +
/// point geometry fed straight from the host, drawn in the same depth-cleared overlay
/// pass as the datum/dimension widgets. Groups are keyed by `name` (upsert) and
/// ordered by `render_order`. Triangles/lines are pre-expanded at feed time; the
/// camera-dependent point quads are built per frame.
struct OverlayGroupW {
    name: String,
    render_order: i32,
    tris: Vec<TriVertex>,
    lines: Vec<LineVertex>,
    points: Vec<OverlayPointW>,
    point_size: f32,
}

/// Default screen size (CSS px) for an overlay point when unspecified.
const DEFAULT_OVERLAY_POINT_PX: f32 = 6.0;

impl OverlayGroupW {
    /// Parse one group object from the `set_overlay` JSON. Flat layout:
    /// `{name, renderOrder?, tris:{positions,colors,normals?},
    ///   lines:{positions,colors}, points?:{positions,colors,size?}}`.
    /// Positions are flat xyz; tri/line/point colors are flat rgb per vertex.
    fn from_json(g: &Value) -> Self {
        let name = g.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string();
        let render_order = g.get("renderOrder").and_then(|v| v.as_i64()).unwrap_or(0) as i32;

        // Triangles: 3 vertices each; per-vertex color; optional per-vertex
        // normal (else a computed flat face normal).
        let mut tris = Vec::new();
        if let Some(t) = g.get("tris") {
            let positions = f32_array(t.get("positions"));
            let colors = f32_array(t.get("colors"));
            let normals = f32_array(t.get("normals"));
            let vcount = positions.len() / 3;
            let tri_count = vcount / 3;
            for tri in 0..tri_count {
                let vi = [tri * 3, tri * 3 + 1, tri * 3 + 2];
                let p = [pos_at(&positions, vi[0]), pos_at(&positions, vi[1]), pos_at(&positions, vi[2])];
                let face_n = p[1].sub(p[0]).cross(p[2].sub(p[0])).normalized();
                for k in 0..3 {
                    let idx = vi[k];
                    let normal: [f32; 3] = if normals.len() >= (idx + 1) * 3 {
                        [normals[idx * 3], normals[idx * 3 + 1], normals[idx * 3 + 2]]
                    } else {
                        face_n.into()
                    };
                    tris.push(TriVertex {
                        pos: p[k].into(),
                        normal,
                        color: rgb_at(&colors, idx),
                    });
                }
            }
        }

        // Lines: consecutive vertex pairs; per-vertex color (segment shading
        // reads the first endpoint's color, matching the widget line pass).
        let mut lines = Vec::new();
        if let Some(l) = g.get("lines") {
            let positions = f32_array(l.get("positions"));
            let colors = f32_array(l.get("colors"));
            let vcount = positions.len() / 3;
            let seg_count = vcount / 2;
            for seg in 0..seg_count {
                let a = seg * 2;
                let b = seg * 2 + 1;
                lines.push(LineVertex { pos: pos_at(&positions, a).into(), color: rgb_at(&colors, a) });
                lines.push(LineVertex { pos: pos_at(&positions, b).into(), color: rgb_at(&colors, b) });
            }
        }

        // Points: screen-constant billboarded quads (expanded per frame).
        let mut points = Vec::new();
        let mut point_size = DEFAULT_OVERLAY_POINT_PX;
        if let Some(pt) = g.get("points") {
            let positions = f32_array(pt.get("positions"));
            let colors = f32_array(pt.get("colors"));
            point_size = pt.get("size").and_then(|v| v.as_f64()).unwrap_or(DEFAULT_OVERLAY_POINT_PX as f64) as f32;
            let count = positions.len() / 3;
            for i in 0..count {
                points.push(OverlayPointW { center: pos_at(&positions, i), color: rgb_at(&colors, i) });
            }
        }

        Self { name, render_order, tris, lines, points, point_size }
    }

    fn is_empty(&self) -> bool {
        self.tris.is_empty() && self.lines.is_empty() && self.points.is_empty()
    }
}

enum DimW {
    Linear {
        id: String,
        a: Vec3,
        b: Vec3,
        offset_dir: Vec3,
        offset: f32,
        color: [f32; 4],
    },
    Angular {
        id: String,
        vertex: Vec3,
        dir_a: Vec3,
        dir_b: Vec3,
        radius: f32,
        color: [f32; 4],
    },
    Radial {
        id: String,
        center: Vec3,
        point: Vec3,
        color: [f32; 4],
    },
}

/// The engine-side widget registry (one per engine).
pub struct WidgetRegistry {
    /// The ViewCube is always-on once enabled by the host UI (its retired
    /// counterpart is deleted in the same change set).
    pub viewcube_enabled: bool,
    viewcube: ViewCube,
    viewcube_hover: Option<HandleId>,
    /// The frame the cube's labelled faces are read in ([`ViewFrame`]): the
    /// cube is built Y-up in its own space, and this carries it into the world,
    /// so with a board shown its T face sits on world +Z.
    view_frame: ViewFrame,
    planes: Vec<PlaneW>,
    axes: Vec<AxisW>,
    frames: Vec<FrameW>,
    curves: Vec<CurveW>,
    dims: Vec<DimW>,
    /// General overlay geometry channel (`set_overlay`): arbitrary named groups
    /// of tris/lines/points (e.g. feature-dialog previews), engine-drawn.
    overlay_groups: Vec<OverlayGroupW>,
    transform: Option<TransformGizmo>,
    transform_hover: Option<HandleId>,
    transform_active: Option<HandleId>,
}

impl Default for WidgetRegistry {
    fn default() -> Self {
        Self {
            viewcube_enabled: false,
            viewcube: ViewCube::new(),
            viewcube_hover: None,
            view_frame: ViewFrame::World,
            planes: Vec::new(),
            axes: Vec::new(),
            frames: Vec::new(),
            curves: Vec::new(),
            dims: Vec::new(),
            overlay_groups: Vec::new(),
            transform: None,
            transform_hover: None,
            transform_active: None,
        }
    }
}

/// The ViewCube render frame: its overlay drawn with a mini-camera in a
/// scissored corner sub-rect.
pub struct ViewCubeFrame {
    pub overlay: Overlay,
    /// Column-major mini-camera world→clip.
    pub view_proj: [[f32; 4]; 4],
    pub forward: [f32; 3],
    /// Corner rect `[x, y, w, h]` in CSS px (top-left origin, y down).
    pub rect_css: [f32; 4],
}

/// A whole frame's worth of overlay-widget geometry, ready for the render
/// core's overlay pass: the main overlay (datums / dimensions / curves /
/// transform gizmo, full-viewport) + the optional ViewCube (own mini-camera +
/// corner rect).
pub struct WidgetOverlay {
    pub main: Overlay,
    /// Number of LEADING `main.tris` vertices belonging to the datum/construction
    /// planes (drawn with a no-depth-write pipeline so they never occlude the
    /// gizmos/dimensions that follow). The remaining tris are the gizmo geometry.
    pub plane_tri_verts: usize,
    pub viewcube: Option<ViewCubeFrame>,
}

impl WidgetOverlay {
    /// Nothing to draw (skip the overlay passes entirely).
    pub fn is_empty(&self) -> bool {
        self.main.lines.is_empty() && self.main.tris.is_empty() && self.viewcube.is_none()
    }

    /// The world-space AABB of the MAIN overlay (datums, axes, frames, curves,
    /// dimensions, transform gizmo, general groups) — folded into the camera
    /// depth-range fit so construction geometry beyond the solids never clips.
    /// EXCLUDES the ViewCube: it lives in the separate `viewcube` field and is
    /// drawn with its own mini-camera, so its coords are NOT world space —
    /// bboxing them would corrupt the fit. Point groups are already expanded
    /// into `main.tris` (`push_point_quad`), so tris + lines cover everything.
    /// Empty when the main overlay is empty. A single vertex absurdly far out
    /// (|coord| > 1e6) is skipped rather than unioned, so a pathological
    /// "infinite" helper can't blow the depth window open (world axes are
    /// screen-constant and finite, but be defensive).
    pub fn world_bbox(&self) -> crate::camera::Aabb {
        let mut bbox = crate::camera::Aabb::empty();
        let mut fold = |pos: &[f32; 3]| {
            if pos.iter().any(|c| c.abs() > 1.0e6) {
                return;
            }
            bbox.expand([pos[0] as f64, pos[1] as f64, pos[2] as f64]);
        };
        for v in &self.main.tris {
            fold(&v.pos);
        }
        for v in &self.main.lines {
            fold(&v.pos);
        }
        bbox
    }
}

impl WidgetRegistry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Any main-overlay widget geometry present (excludes the ViewCube, which
    /// draws in its own pass).
    pub fn has_main_overlay(&self) -> bool {
        !self.planes.is_empty()
            || !self.axes.is_empty()
            || !self.frames.is_empty()
            || !self.curves.is_empty()
            || !self.dims.is_empty()
            || !self.overlay_groups.is_empty()
            || self.transform.is_some()
    }

    /// World-space AABB of the pushed overlay GROUPS — the `set_overlay` geometry
    /// (the sketch curves + points, dimension leaders, constraint glyphs, datums,
    /// curves). EXCLUDES the screen-constant transform gizmo + ViewCube (own passes).
    /// Folded into the camera depth-range fit so orbiting an editing sketch doesn't
    /// clip the overlay against the SOLIDS-ONLY scene bounds (the reported
    /// sketch-clipping bug when "Lock to sketch" is off). Empty when no groups.
    pub fn overlay_groups_bbox(&self) -> crate::camera::Aabb {
        let mut bbox = crate::camera::Aabb::empty();
        for group in &self.overlay_groups {
            for v in &group.tris {
                bbox.expand([v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64]);
            }
            for v in &group.lines {
                bbox.expand([v.pos[0] as f64, v.pos[1] as f64, v.pos[2] as f64]);
            }
            for point in &group.points {
                let c: [f32; 3] = point.center.into();
                bbox.expand([c[0] as f64, c[1] as f64, c[2] as f64]);
            }
        }
        bbox
    }

    // --- JSON feed --------------------------------------------------------

    /// Replace the datum / curve display set. Shape:
    /// `{planes:[{name,origin,x,y,size?,color?,selected?,hovered?}],
    ///   axes:[{name,point,direction,length,color?,selected?,hovered?}],
    ///   frames:[{origin,x,y,z,px?}],
    ///   curves:[{points:[[x,y,z]...],closed?,color?,selected?,hovered?}]}`.
    pub fn set_datums_json(&mut self, json: &str) -> Result<(), String> {
        let value: Value =
            serde_json::from_str(json).map_err(|e| format!("datums parse: {e}"))?;
        self.planes.clear();
        self.axes.clear();
        self.frames.clear();
        self.curves.clear();

        if let Some(list) = value.get("planes").and_then(|v| v.as_array()) {
            for p in list {
                let (Some(origin), Some(x), Some(y)) = (
                    p.get("origin").and_then(vec3_of),
                    p.get("x").and_then(vec3_of),
                    p.get("y").and_then(vec3_of),
                ) else {
                    continue;
                };
                self.planes.push(PlaneW {
                    name: p.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                    origin,
                    x,
                    y,
                    size: p.get("size").and_then(|v| v.as_f64()).map(|s| s as f32),
                    color: color_of(p.get("color"), datum::PLANE_COLOR),
                    hot: flag(p, "selected") || flag(p, "hovered"),
                });
            }
        }
        if let Some(list) = value.get("axes").and_then(|v| v.as_array()) {
            for a in list {
                let (Some(point), Some(direction)) = (
                    a.get("point").and_then(vec3_of),
                    a.get("direction").and_then(vec3_of),
                ) else {
                    continue;
                };
                self.axes.push(AxisW {
                    name: a.get("name").and_then(|v| v.as_str()).unwrap_or("").to_string(),
                    point,
                    direction,
                    length: a.get("length").and_then(|v| v.as_f64()).unwrap_or(10.0) as f32,
                    color: color_of(a.get("color"), DEFAULT_AXIS_COLOR),
                    hot: flag(a, "selected") || flag(a, "hovered"),
                });
            }
        }
        if let Some(list) = value.get("frames").and_then(|v| v.as_array()) {
            for f in list {
                let (Some(origin), Some(x), Some(y), Some(z)) = (
                    f.get("origin").and_then(vec3_of),
                    f.get("x").and_then(vec3_of),
                    f.get("y").and_then(vec3_of),
                    f.get("z").and_then(vec3_of),
                ) else {
                    continue;
                };
                self.frames.push(FrameW {
                    origin,
                    x,
                    y,
                    z,
                    px: f.get("px").and_then(|v| v.as_f64()).unwrap_or(datum::DEFAULT_FRAME_PX as f64)
                        as f32,
                });
            }
        }
        if let Some(list) = value.get("curves").and_then(|v| v.as_array()) {
            for c in list {
                let points: Vec<Vec3> = c
                    .get("points")
                    .and_then(|v| v.as_array())
                    .map(|arr| arr.iter().filter_map(vec3_of).collect())
                    .unwrap_or_default();
                if points.len() < 2 {
                    continue;
                }
                let mut color = color_of(c.get("color"), curve_display::CURVE_COLOR);
                if flag(c, "selected") || flag(c, "hovered") {
                    color = brighten(color);
                }
                self.curves.push(CurveW {
                    points,
                    closed: flag(c, "closed"),
                    color,
                });
            }
        }
        Ok(())
    }

    /// Replace/upsert the GENERAL overlay geometry channel (`set_overlay`).
    /// Shape: `{groups:[{name, renderOrder?, tris:{positions,colors,normals?},
    /// lines:{positions,colors}, points?:{positions,colors,size?}}]}`. Each group
    /// UPSERTS by `name`; a group whose geometry is entirely empty REMOVES that
    /// name; an empty (or missing) `groups` array CLEARS every group.
    pub fn set_overlay_json(&mut self, json: &str) -> Result<(), String> {
        let value: Value =
            serde_json::from_str(json).map_err(|e| format!("overlay parse: {e}"))?;
        let Some(list) = value.get("groups").and_then(|v| v.as_array()) else {
            self.overlay_groups.clear();
            return Ok(());
        };
        if list.is_empty() {
            self.overlay_groups.clear();
            return Ok(());
        }
        for g in list {
            let group = OverlayGroupW::from_json(g);
            // Upsert by name (a duplicate name replaces the prior group).
            self.overlay_groups.retain(|x| x.name != group.name);
            if !group.is_empty() {
                self.overlay_groups.push(group);
            }
        }
        Ok(())
    }

    /// The names of the currently-loaded (non-empty) general overlay groups — a read
    /// accessor for tests / verification. An empty group is auto-removed on upsert, so
    /// a name present here always carries geometry.
    pub fn overlay_group_names(&self) -> Vec<&str> {
        self.overlay_groups.iter().map(|g| g.name.as_str()).collect()
    }

    /// The currently-fed datum PLANES as `(name, emphasized)` — a read accessor for
    /// tests / verification (the datum planes replace their set wholesale each
    /// `set_datums_json`, so this is exactly the current construction-datum feed).
    /// `emphasized` is the selected/hovered `hot` flag (a selected datum reads true).
    pub fn datum_plane_names(&self) -> Vec<(&str, bool)> {
        self.planes.iter().map(|p| (p.name.as_str(), p.hot)).collect()
    }

    /// Replace the feature-dimension set. Shape: an array of
    /// `{id, type:"linear"|"angular"|"radial", ...}` — linear: `a,b,offsetDir,
    /// offset`; angular: `vertex,dirA,dirB,radius`; radial: `center,pointOnCircle`.
    pub fn set_dimensions_json(&mut self, json: &str) -> Result<(), String> {
        let value: Value =
            serde_json::from_str(json).map_err(|e| format!("dimensions parse: {e}"))?;
        self.dims.clear();
        let Some(list) = value.as_array() else {
            return Ok(());
        };
        for d in list {
            let id = d.get("id").and_then(|v| v.as_str()).unwrap_or("").to_string();
            let color = color_of(d.get("color"), dimension::DIMENSION_COLOR);
            match d.get("type").and_then(|v| v.as_str()) {
                Some("linear") => {
                    if let (Some(a), Some(b), Some(offset_dir)) = (
                        d.get("a").and_then(vec3_of),
                        d.get("b").and_then(vec3_of),
                        d.get("offsetDir").and_then(vec3_of),
                    ) {
                        self.dims.push(DimW::Linear {
                            id,
                            a,
                            b,
                            offset_dir,
                            offset: d.get("offset").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                            color,
                        });
                    }
                }
                Some("angular") => {
                    if let (Some(vertex), Some(dir_a), Some(dir_b)) = (
                        d.get("vertex").and_then(vec3_of),
                        d.get("dirA").and_then(vec3_of),
                        d.get("dirB").and_then(vec3_of),
                    ) {
                        self.dims.push(DimW::Angular {
                            id,
                            vertex,
                            dir_a,
                            dir_b,
                            radius: d.get("radius").and_then(|v| v.as_f64()).unwrap_or(0.0) as f32,
                            color,
                        });
                    }
                }
                Some("radial") => {
                    if let (Some(center), Some(point)) = (
                        d.get("center").and_then(vec3_of),
                        d.get("pointOnCircle").and_then(vec3_of),
                    ) {
                        self.dims.push(DimW::Radial {
                            id,
                            center,
                            point,
                            color,
                        });
                    }
                }
                _ => {}
            }
        }
        Ok(())
    }

    /// Set (or clear, when `json == "null"`) the transform gizmo. Shape:
    /// `{origin,x,y,z,showCenter?,showAxes?,showRings?}` — the selected
    /// feature's frame (R28); the optional show flags (default true) carve the
    /// translate-only / rotate-only variants the component Move toggle cycles.
    pub fn set_transform_json(&mut self, json: &str) -> Result<(), String> {
        let value: Value =
            serde_json::from_str(json).map_err(|e| format!("transform parse: {e}"))?;
        if value.is_null() {
            self.transform = None;
            self.transform_hover = None;
            self.transform_active = None;
            return Ok(());
        }
        let origin = value.get("origin").and_then(vec3_of).unwrap_or(Vec3::ZERO);
        let x = value.get("x").and_then(vec3_of).unwrap_or(Vec3::X);
        let y = value.get("y").and_then(vec3_of).unwrap_or(Vec3::Y);
        let z = value.get("z").and_then(vec3_of).unwrap_or(Vec3::Z);
        let mut gz = TransformGizmo::default();
        gz.set_frame(origin, x, y, z);
        let flag = |key: &str| value.get(key).and_then(|v| v.as_bool()).unwrap_or(true);
        gz.show_center = flag("showCenter");
        gz.show_axes = flag("showAxes");
        gz.show_rings = flag("showRings");
        self.transform = Some(gz);
        Ok(())
    }

    pub fn set_viewcube_enabled(&mut self, enabled: bool) {
        self.viewcube_enabled = enabled;
        if !enabled {
            self.viewcube_hover = None;
        }
    }

    /// Set the frame the ViewCube's faces are named in. The engine sets it
    /// from whether a board is shown, the same test its standard views use.
    pub fn set_view_frame(&mut self, frame: ViewFrame) {
        self.view_frame = frame;
    }

    pub fn view_frame(&self) -> ViewFrame {
        self.view_frame
    }

    /// The main camera as the cube sees it: its orientation expressed in the
    /// view frame. The cube's mini-camera, its geometry and its hit test read
    /// only the camera's forward and up, so turning those two (and the eye,
    /// for consistency) is all it takes for the cube to be drawn and picked in
    /// the frame. The identity for [`ViewFrame::World`].
    fn cube_camera(&self, cam: &GizmoCamera) -> GizmoCamera {
        let turn = |v: Vec3| {
            let t = self.view_frame.from_world([v.x as f64, v.y as f64, v.z as f64]);
            Vec3::new(t[0] as f32, t[1] as f32, t[2] as f32)
        };
        GizmoCamera { eye: turn(cam.eye), forward: turn(cam.forward), up: turn(cam.up), ..*cam }
    }

    /// Set the ViewCube's on-screen edge length (CSS px). ONE size field feeds both
    /// the rendered mini-camera viewport (`build_viewcube`) and the hit-test corner
    /// rect (`viewcube_rect`), so the drawn cube and its clickable region always
    /// scale together. Driven from `RenderSettings::viewcube_size_px` on every
    /// settings apply. Guarded to a >= 1px positive size so a bad feed can't
    /// collapse the rect.
    pub fn set_viewcube_size(&mut self, size_px: f32) {
        if size_px.is_finite() {
            self.viewcube.size = size_px.max(1.0);
        }
    }

    // --- geometry ---------------------------------------------------------

    /// Build the main-overlay geometry (everything but the ViewCube).
    /// Build the main overlay geometry AND the count of leading tri vertices that
    /// belong to the datum/construction PLANES (always emitted FIRST). The render
    /// core draws those with a NO-depth-write pipeline so a translucent plane can
    /// never occlude the gizmos/dimensions that follow.
    pub fn build_main_overlay(&self, cam: &GizmoCamera) -> (Overlay, usize) {
        let mut ov = Overlay::new();

        for p in &self.planes {
            let color = if p.hot { brighten(p.color) } else { p.color };
            let plane = match p.size {
                Some(size) => datum::datum_plane(p.origin, p.x, p.y, size, color),
                None => datum::datum_plane_screen(
                    p.origin,
                    p.x,
                    p.y,
                    datum::DEFAULT_PLANE_SCREEN_PX,
                    color,
                    cam,
                ),
            };
            ov.extend(&plane);
        }
        // Everything after this point (axes, frames, curves, dimensions, gizmo,
        // groups) is drawn with the depth-writing pipeline so it self-occludes.
        let plane_tri_verts = ov.tris.len();
        for a in &self.axes {
            let color = if a.hot { brighten(a.color) } else { a.color };
            ov.extend(&datum::datum_axis(a.point, a.direction, a.length, color));
        }
        for f in &self.frames {
            ov.extend(&datum::datum_frame(f.origin, f.x, f.y, f.z, f.px, cam));
        }
        for c in &self.curves {
            ov.extend(&curve_display::polyline_display(&c.points, c.color, c.closed));
        }
        for d in &self.dims {
            ov.extend(&self.build_dim(d, cam).overlay);
        }
        if let Some(gz) = &self.transform {
            ov.extend(&gz.geometry(cam, self.transform_hover, self.transform_active));
        }
        // General overlay groups (set_overlay), in render_order (stable within
        // equal orders). Tris/lines are pre-expanded; point quads are built here
        // camera-facing + screen-constant since the overlay pass has no point
        // pipeline of its own.
        let mut order: Vec<&OverlayGroupW> = self.overlay_groups.iter().collect();
        order.sort_by_key(|g| g.render_order);
        for g in order {
            ov.tris.extend_from_slice(&g.tris);
            ov.lines.extend_from_slice(&g.lines);
            for p in &g.points {
                push_point_quad(&mut ov, p.center, p.color, g.point_size, cam);
            }
        }
        (ov, plane_tri_verts)
    }

    fn build_dim(&self, d: &DimW, cam: &GizmoCamera) -> dimension::DimensionAnnotation {
        match d {
            DimW::Linear {
                a,
                b,
                offset_dir,
                offset,
                color,
                ..
            } => dimension::linear_dimension_colored(*a, *b, *offset_dir, *offset, cam, *color),
            DimW::Angular {
                vertex,
                dir_a,
                dir_b,
                radius,
                color,
                ..
            } => dimension::angular_dimension_colored(*vertex, *dir_a, *dir_b, *radius, cam, *color),
            DimW::Radial {
                center,
                point,
                color,
                ..
            } => dimension::radial_dimension_colored(*center, *point, cam, *color),
        }
    }

    /// `(id, world label anchor)` for every dimension — the host projects
    /// each with `world_to_screen` to place its text label (R29).
    pub fn dimension_anchors(&self, cam: &GizmoCamera) -> Vec<(String, [f32; 3])> {
        self.dims
            .iter()
            .map(|d| {
                let id = match d {
                    DimW::Linear { id, .. } | DimW::Angular { id, .. } | DimW::Radial { id, .. } => {
                        id.clone()
                    }
                };
                (id, self.build_dim(d, cam).label_anchor.into())
            })
            .collect()
    }

    /// Any overlay widget is present (main geometry or the ViewCube).
    pub fn any_visible(&self) -> bool {
        self.has_main_overlay() || self.viewcube_enabled
    }

    /// Build a whole frame's overlay geometry from the live camera.
    pub fn build_overlay(&self, cam: &GizmoCamera) -> WidgetOverlay {
        let (main, plane_tri_verts) = self.build_main_overlay(cam);
        WidgetOverlay {
            main,
            plane_tri_verts,
            viewcube: self.build_viewcube(cam),
        }
    }

    /// The ViewCube render frame (overlay + mini-camera + corner rect), or None
    /// when disabled.
    pub fn build_viewcube(&self, cam: &GizmoCamera) -> Option<ViewCubeFrame> {
        if !self.viewcube_enabled {
            return None;
        }
        let cam = &self.cube_camera(cam);
        let overlay = self.viewcube.geometry(cam, self.viewcube_hover, None);
        let mini = self.viewcube.mini_camera(cam);
        Some(ViewCubeFrame {
            overlay,
            view_proj: mini.view_proj,
            forward: mini.forward.into(),
            rect_css: self.viewcube.sub_rect(cam.viewport),
        })
    }

    // --- hit testing / interaction ---------------------------------------

    /// The ViewCube corner rect `[x, y, w, h]` (CSS px) — the host decides
    /// whether to forward a pointer event and offsets it into cube-local coords.
    pub fn viewcube_rect(&self, cam: &GizmoCamera) -> [f32; 4] {
        self.viewcube.sub_rect(cam.viewport)
    }

    /// Hit-test the ViewCube at cube-local pixels; returns the region handle.
    pub fn viewcube_hit(&self, cam: &GizmoCamera, local_x: f32, local_y: f32) -> Option<HandleId> {
        if !self.viewcube_enabled {
            return None;
        }
        self.viewcube.hit(&self.cube_camera(cam), [local_x, local_y])
    }

    /// Set the ViewCube hover region (drives the highlight). Returns whether it
    /// changed.
    pub fn set_viewcube_hover(&mut self, handle: Option<HandleId>) -> bool {
        if self.viewcube_hover != handle {
            self.viewcube_hover = handle;
            true
        } else {
            false
        }
    }

    /// The world eye→target look direction + up hint for a ViewCube region —
    /// the cube's own (frame) direction carried into the world by the frame.
    pub fn viewcube_target(&self, handle: HandleId) -> ([f32; 3], [f32; 3]) {
        let world = |v: Vec3| {
            let w = self.view_frame.to_world([v.x as f64, v.y as f64, v.z as f64]);
            [w[0] as f32, w[1] as f32, w[2] as f32]
        };
        (world(ViewCube::target_view(handle)), world(ViewCube::target_up(handle)))
    }

    /// EVERY fed datum PLANE whose DRAWN card the pointer ray crosses, as
    /// `(name, world hit point)` — the multi-hit sibling of [`datum_pick`], which
    /// stops at the first hit and also considers axes.
    ///
    /// The bound is the rectangle the renderer draws, not the infinite plane:
    /// [`DatumPlane::hit_point`] is the same `half()` extent [`build_main_overlay`]
    /// draws with, evaluated against the LIVE camera on every call — so a
    /// screen-constant card's pickable region tracks its drawn size across a zoom
    /// (nothing is baked). Either face of the card hits. Unnamed planes are
    /// skipped (nothing could be selected by them).
    ///
    /// The engine turns these into `PickKind::Plane` candidates so a construction
    /// plane competes in the ordinary pick list instead of only on a geometry
    /// miss (see `EngineState::pick_candidates_at`).
    ///
    /// [`datum_pick`]: Self::datum_pick
    /// [`build_main_overlay`]: Self::build_main_overlay
    pub fn datum_plane_hits(&self, cam: &GizmoCamera, x: f32, y: f32) -> Vec<(String, [f32; 3])> {
        self.planes
            .iter()
            .filter(|p| !p.name.is_empty())
            .filter_map(|p| {
                let gz = DatumPlane {
                    origin: p.origin,
                    x_axis: p.x,
                    y_axis: p.y,
                    size: p.size,
                    color: p.color,
                    handle: 1,
                };
                let point = gz.hit_point(cam, [x, y])?;
                Some((p.name.clone(), [point.x, point.y, point.z]))
            })
            .collect()
    }

    /// Pick the datum plane/axis under a screen pixel; returns its name.
    pub fn datum_pick(&self, cam: &GizmoCamera, x: f32, y: f32) -> Option<String> {
        // Nearest wins by depth of the hit; planes and axes both tested. We keep
        // it simple: axes first (thin, priority), then planes.
        for a in &self.axes {
            let gz = DatumAxis {
                point: a.point,
                direction: a.direction,
                length: a.length,
                color: a.color,
                handle: 1,
            };
            if gz.hit(cam, [x, y]).is_some() && !a.name.is_empty() {
                return Some(a.name.clone());
            }
        }
        for p in &self.planes {
            let gz = DatumPlane {
                origin: p.origin,
                x_axis: p.x,
                y_axis: p.y,
                size: p.size,
                color: p.color,
                handle: 1,
            };
            if gz.hit(cam, [x, y]).is_some() && !p.name.is_empty() {
                return Some(p.name.clone());
            }
        }
        None
    }

    pub fn has_transform(&self) -> bool {
        self.transform.is_some()
    }

    /// The VISIBLE transform gizmo's current frame origin in world space, or `None`
    /// when the gizmo is hidden. Reflects the last [`set_transform_json`] feed, so
    /// it tracks the live-follow re-sync during a drag (Fix 3) — distinct from a
    /// params-derived anchor, this proves the drawn widget actually moved.
    pub fn transform_origin(&self) -> Option<[f32; 3]> {
        self.transform.as_ref().map(|gz| [gz.origin.x, gz.origin.y, gz.origin.z])
    }

    /// Hit-test the transform gizmo; returns the handle (0 = none).
    pub fn transform_hit(&self, cam: &GizmoCamera, x: f32, y: f32) -> HandleId {
        self.transform
            .as_ref()
            .and_then(|gz| gz.hit(cam, [x, y]))
            .unwrap_or(0)
    }

    /// The world-space `(shaft-start, tip)` endpoints of axis arrow `i` on the
    /// LIVE transform gizmo — the SAME instance + `axis_seg` the hit test
    /// (`transform_hit` → `TransformGizmo::hit`) measures against — so a debug
    /// overlay can outline the exact pickable region without re-deriving it.
    /// `None` when the gizmo is hidden.
    pub fn transform_axis_seg(&self, cam: &GizmoCamera, i: usize) -> Option<(Vec3, Vec3)> {
        self.transform.as_ref().map(|gz| gz.axis_seg(cam, i))
    }

    /// The world-space center free-move / origin ball point of the LIVE transform
    /// gizmo, or `None` when hidden / the center handle is off. For the debug
    /// hit-area outline (radius [`brep_gizmos::transform::PX_CENTER_RAD`]).
    pub fn transform_center_grab(&self) -> Option<Vec3> {
        self.transform.as_ref().and_then(|gz| gz.center_grab_point())
    }

    /// The three rotation grab-sphere world points of the LIVE transform gizmo (in
    /// `ARCS` order), or `None` when hidden. For the debug hit-area outline (radius
    /// [`brep_gizmos::transform::PX_RING_GRAB_RAD`]).
    pub fn transform_ring_grabs(&self, cam: &GizmoCamera) -> Option<[Vec3; 3]> {
        self.transform.as_ref().map(|gz| gz.ring_grab_points(cam))
    }

    /// The authoritative screen-space pickable regions of the LIVE transform gizmo
    /// — the SAME `hit_regions` [`TransformGizmo::hit`] consumes — each paired with
    /// its handle. `[]` when the gizmo is hidden. The debug-outline exposer
    /// serializes these, so the drawn region IS exactly the pickable region.
    pub fn transform_hit_regions(
        &self,
        cam: &GizmoCamera,
    ) -> Vec<(HandleId, brep_gizmos::hit_region::HitShape)> {
        self.transform
            .as_ref()
            .map(|gz| gz.hit_regions(cam))
            .unwrap_or_default()
    }

    pub fn set_transform_hover(&mut self, handle: HandleId) -> bool {
        let handle = (handle != 0).then_some(handle);
        if self.transform_hover != handle {
            self.transform_hover = handle;
            true
        } else {
            false
        }
    }

    pub fn set_transform_active(&mut self, handle: HandleId) {
        self.transform_active = (handle != 0).then_some(handle);
    }

    /// Compute a transform drag: frame-space delta + its world resolution, as
    /// JSON for the host's feature-edit commit (R28). Resolved against the LIVE gizmo
    /// frame (`self.transform`).
    pub fn transform_drag_json(
        &self,
        cam: &GizmoCamera,
        handle: HandleId,
        sx: f32,
        sy: f32,
        cx: f32,
        cy: f32,
    ) -> String {
        let Some(gz) = &self.transform else {
            return "{\"kind\":\"none\"}".to_string();
        };
        drag_delta_json(gz, cam, handle, sx, sy, cx, cy)
    }

    /// Like [`transform_drag_json`] but resolved against an EXPLICIT frozen frame
    /// (`{origin,x,y,z}`, the grab-time feature frame) instead of the live widget
    /// gizmo. This lets the engine re-sync the VISIBLE gizmo to the moving feature
    /// pose every drag frame (Fix 3 live-follow) while the delta stays anchored to
    /// the grab frame, so the visual sync can't feed back into the drag math.
    pub fn transform_drag_json_with_frame(
        &self,
        cam: &GizmoCamera,
        frame_json: &str,
        handle: HandleId,
        sx: f32,
        sy: f32,
        cx: f32,
        cy: f32,
    ) -> String {
        let value: Value = match serde_json::from_str(frame_json) {
            Ok(v) => v,
            Err(_) => return "{\"kind\":\"none\"}".to_string(),
        };
        let origin = value.get("origin").and_then(vec3_of).unwrap_or(Vec3::ZERO);
        let x = value.get("x").and_then(vec3_of).unwrap_or(Vec3::X);
        let y = value.get("y").and_then(vec3_of).unwrap_or(Vec3::Y);
        let z = value.get("z").and_then(vec3_of).unwrap_or(Vec3::Z);
        let mut gz = TransformGizmo::default();
        gz.set_frame(origin, x, y, z);
        drag_delta_json(&gz, cam, handle, sx, sy, cx, cy)
    }
}

/// The shared body of [`WidgetRegistry::transform_drag_json`] + its frozen-frame
/// twin: resolve `gz`'s frame-space drag delta into the feature-edit-commit JSON.
fn drag_delta_json(
    gz: &TransformGizmo,
    cam: &GizmoCamera,
    handle: HandleId,
    sx: f32,
    sy: f32,
    cx: f32,
    cy: f32,
) -> String {
    let start = cam.ray_from_screen(sx, sy);
    let current = cam.ray_from_screen(cx, cy);
    match gz.drag_delta(cam, handle, start, current) {
        DragDelta::Translate(v) => {
            let world = gz.ex.scale(v.x).add(gz.ey.scale(v.y)).add(gz.ez.scale(v.z));
            serde_json::json!({
                "kind": "translate",
                "local": [v.x, v.y, v.z],
                "world": [world.x, world.y, world.z],
            })
            .to_string()
        }
        DragDelta::Rotate { axis_index, radians } => {
            let axis = gz.axis(axis_index);
            serde_json::json!({
                "kind": "rotate",
                "axisIndex": axis_index,
                "axisWorld": [axis.x, axis.y, axis.z],
                "radians": radians,
            })
            .to_string()
        }
        DragDelta::None => "{\"kind\":\"none\"}".to_string(),
    }
}

