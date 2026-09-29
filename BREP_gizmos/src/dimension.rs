//! Feature-dimension LEADER geometry (R29) for the render engine's overlay pass.
//!
//! A dimension annotation is the classic CAD witness/leader drawing: extension
//! (witness) lines running out from the measured feature to an offset dimension
//! line, the dimension line itself, and an arrowhead at each end. This module
//! builds ONLY that 3D leader/extension/arrow geometry as an [`Overlay`] of
//! world-space line segments and flat arrowhead triangles. The numeric TEXT
//! label is drawn by the UI layer — never here — anchored at a world point
//! the engine projects with [`GizmoCamera::world_to_screen`]. Each builder
//! returns that point as [`DimensionAnnotation::label_anchor`].
//!
//! # Where the inputs come from (integration)
//! The main-repo feature-dimension resolver
//! (`FeatureDimensionOverlay`, geometry in
//! `dimensionGeometry`) already extracts, per feature, the
//! measured endpoints / vertex / directions / radius from the selected topology
//! (edges, faces, sketch profiles). That resolver feeds this module:
//! - [`linear_dimension`]: measured points `a`, `b`; `offset_dir` = which way to
//!   push the dimension line off the measured segment (the resolver's `tangent`,
//!   typically `normal × (b-a)`); `offset_dist` = world offset.
//! - [`angular_dimension`]: the corner `vertex` and the two ray `dir_a`/`dir_b`
//!   plus a world `radius` for the arc.
//! - [`radial_dimension`]: the circle `center` (or an edge point) and a
//!   `point_on_circle`.
//!
//! # Label placement (integration)
//! The engine calls `camera.world_to_screen(annotation.label_anchor)` and pins
//! the text label at that pixel (nudging in screen space for arrow
//! clearance — the UI layer owns collision avoidance). `label_anchor` is a pure
//! attach point; it is NOT part of the drawn overlay.
//!
//! # Sizing
//! Arrowheads are screen-CONSTANT: their world length/width are derived from
//! [`GizmoCamera::world_per_pixel`] at the arrowhead tip, so they stay a fixed
//! pixel size at any zoom (orthographic or perspective). Distances the caller
//! passes (`offset_dist`, angular `radius`) are world units.
//!
//! Conventions: +Y-up world (the app's — see the crate header), RGBA colors are
//! linear-space. No kernel/GPU dep. Nothing here reads the world up: every
//! direction comes from the caller's points or the camera.

use crate::{GizmoCamera, Overlay, Vec3};

/// Screen-constant arrowhead length, CSS pixels.
pub const ARROW_LEN_PX: f32 = 14.0;
/// Screen-constant arrowhead half-width, CSS pixels.
pub const ARROW_HALF_WIDTH_PX: f32 = 5.0;
/// Radial facets of an arrowhead cone.
pub const ARROW_CONE_SEGMENTS: usize = 12;
/// Default world radius for an angular arc when the caller passes `radius <= 0`,
/// expressed in pixels (converted via `world_per_pixel`).
pub const DEFAULT_ANGLE_RADIUS_PX: f32 = 120.0;
/// Minimum on-screen arc radius (pixels) so a tiny world radius stays readable.
pub const MIN_ANGLE_RADIUS_PX: f32 = 40.0;
/// How far (pixels) the angular direction (witness) lines overshoot the arc.
pub const ANGLE_EXT_PX: f32 = 18.0;
/// Gap (pixels) from the measured feature to the label attach point on a radial
/// or the small landing leader.
pub const LABEL_GAP_PX: f32 = 10.0;

/// Default dimension color (linear-space RGBA) — a warm CAD amber matching the
/// main-repo feature-dimension line color.
pub const DIMENSION_COLOR: [f32; 4] = [1.0, 0.72, 0.30, 1.0];

/// The return value of every dimension builder: the overlay geometry to draw
/// plus the world-space point the engine projects to place the text label.
///
/// `label_anchor` is not drawn — see the module docs. The engine does
/// `camera.world_to_screen(label_anchor)` and pins the text label there.
#[derive(Debug, Clone)]
pub struct DimensionAnnotation {
    /// Extension lines, dimension line(s)/arc (in `overlay.lines`) and filled
    /// arrowheads (in `overlay.tris`), all world-space.
    pub overlay: Overlay,
    /// World point for text placement (dimension-line midpoint / arc
    /// midpoint / just outside the circle). Not part of the drawn geometry.
    pub label_anchor: Vec3,
}

// --- small local geometry helpers ------------------------------------------

/// Rotate `v` about a UNIT `axis` by `angle` radians (Rodrigues).
fn rotate_about_axis(v: Vec3, axis: Vec3, angle: f32) -> Vec3 {
    let (s, c) = angle.sin_cos();
    v.scale(c)
        .add(axis.cross(v).scale(s))
        .add(axis.scale(axis.dot(v) * (1.0 - c)))
}

/// Emit one filled, radially-symmetric arrowhead CONE: sharp apex at `apex`
/// aiming along unit `dir` (the arrow "points" this way), base circle of world
/// `radius` centered `len` back from the apex. Reads as a solid 3D cone (side
/// facets + a base cap) from any view — no billboarding needed.
fn arrowhead(
    ov: &mut Overlay,
    apex: Vec3,
    dir: Vec3,
    len: f32,
    radius: f32,
    color: [f32; 4],
) {
    let axis = dir.normalized();
    if axis.length() < 1e-9 || len <= 0.0 || radius <= 0.0 {
        return;
    }
    let base = apex.sub(axis.scale(len));
    let u = axis.any_perp();
    let v = axis.cross(u).normalized();
    let ring = |k: usize| -> Vec3 {
        let ang = (k as f32 / ARROW_CONE_SEGMENTS as f32) * std::f32::consts::TAU;
        base.add(u.scale(ang.cos() * radius))
            .add(v.scale(ang.sin() * radius))
    };
    let mut prev = ring(0);
    for k in 1..=ARROW_CONE_SEGMENTS {
        let cur = ring(k);
        ov.tri(apex, prev, cur, color); // side facet
        ov.tri(base, cur, prev, color); // base cap
        prev = cur;
    }
}

// --- linear dimension -------------------------------------------------------

/// A linear (distance) dimension between measured points `a` and `b`, with the
/// dimension line pushed off the segment by `offset_dist` along `offset_dir`.
///
/// Emits: extension line `a → da`, extension line `b → db`, the dimension line
/// `da → db`, and an arrowhead at each end (`da`, `db`) whose sharp apex sits on
/// the extension line and whose body opens inward toward the label — the
/// standard `|<——>|` CAD look. `offset_dir` is orthogonalized against `a→b`, so
/// the extension lines are perpendicular to the dimension line and the
/// dimension line stays parallel to the measured segment.
///
/// The label anchor is the dimension-line midpoint `(da + db) / 2` — the engine
/// projects it for text placement.
pub fn linear_dimension(
    a: Vec3,
    b: Vec3,
    offset_dir: Vec3,
    offset_dist: f32,
    camera: &GizmoCamera,
) -> DimensionAnnotation {
    linear_dimension_colored(a, b, offset_dir, offset_dist, camera, DIMENSION_COLOR)
}

/// [`linear_dimension`] with an explicit color.
pub fn linear_dimension_colored(
    a: Vec3,
    b: Vec3,
    offset_dir: Vec3,
    offset_dist: f32,
    camera: &GizmoCamera,
    color: [f32; 4],
) -> DimensionAnnotation {
    let mut ov = Overlay::new();

    let dir_raw = b.sub(a);
    let len_ab = dir_raw.length();
    let dir = if len_ab < 1e-9 {
        Vec3::X
    } else {
        dir_raw.scale(1.0 / len_ab)
    };

    // Orthogonalize the offset direction against the measured segment so the
    // witness lines come off perpendicular (CAD standard).
    let mut od = offset_dir.sub(dir.scale(offset_dir.dot(dir)));
    if od.length() < 1e-9 {
        od = dir.any_perp();
    }
    od = od.normalized();

    let da = a.add(od.scale(offset_dist));
    let db = b.add(od.scale(offset_dist));

    // Extension (witness) lines: measured point out to the dimension line.
    ov.line(a, da, color);
    ov.line(b, db, color);
    // Dimension line.
    ov.line(da, db, color);

    // Arrowheads: apex on each extension line, opening inward.
    let wpp_a = camera.world_per_pixel(da);
    let wpp_b = camera.world_per_pixel(db);
    arrowhead(
        &mut ov,
        da,
        dir.scale(-1.0),
        ARROW_LEN_PX * wpp_a,
        ARROW_HALF_WIDTH_PX * wpp_a,
        color,
    );
    arrowhead(
        &mut ov,
        db,
        dir,
        ARROW_LEN_PX * wpp_b,
        ARROW_HALF_WIDTH_PX * wpp_b,
        color,
    );

    let label_anchor = da.lerp(db, 0.5);
    DimensionAnnotation {
        overlay: ov,
        label_anchor,
    }
}

// --- angular dimension ------------------------------------------------------

/// The polyline of world points sampling the dimension arc, from the `dir_a`
/// ray to the `dir_b` ray, swept about the plane normal `dir_a × dir_b` at
/// `radius` from `vertex`. Exposed so callers/tests can inspect the arc (and the
/// engine can reuse the raw samples). Returns an empty vec for parallel rays.
pub fn angular_arc_points(vertex: Vec3, dir_a: Vec3, dir_b: Vec3, radius: f32) -> Vec<Vec3> {
    let a = dir_a.normalized();
    let b = dir_b.normalized();
    let mut normal = a.cross(b);
    if normal.length() < 1e-9 {
        return Vec::new();
    }
    normal = normal.normalized();
    let sweep = a.dot(b).clamp(-1.0, 1.0).acos();
    // ~1 sample per 5 degrees, minimum 8 segments.
    let steps = (((sweep / (5.0_f32).to_radians()).ceil()) as usize).max(8);
    let mut out = Vec::with_capacity(steps + 1);
    for i in 0..=steps {
        let t = i as f32 / steps as f32;
        let d = rotate_about_axis(a, normal, sweep * t);
        out.push(vertex.add(d.scale(radius)));
    }
    out
}

/// An angular (included-angle) dimension at `vertex` between rays `dir_a` and
/// `dir_b`, drawn as an arc at `radius` with an arrowhead at each arc end and a
/// short witness line along each ray running just past the arc.
///
/// `radius` is world units; pass `<= 0` to get a screen-constant default
/// ([`DEFAULT_ANGLE_RADIUS_PX`]). The label anchor is the arc midpoint (on the
/// angle bisector, at `radius`).
pub fn angular_dimension(
    vertex: Vec3,
    dir_a: Vec3,
    dir_b: Vec3,
    radius: f32,
    camera: &GizmoCamera,
) -> DimensionAnnotation {
    angular_dimension_colored(vertex, dir_a, dir_b, radius, camera, DIMENSION_COLOR)
}

/// [`angular_dimension`] with an explicit color.
pub fn angular_dimension_colored(
    vertex: Vec3,
    dir_a: Vec3,
    dir_b: Vec3,
    radius: f32,
    camera: &GizmoCamera,
    color: [f32; 4],
) -> DimensionAnnotation {
    let mut ov = Overlay::new();
    let a = dir_a.normalized();
    let b = dir_b.normalized();
    let wpp = camera.world_per_pixel(vertex);

    // Resolve a readable world radius.
    let mut r = if radius > 1e-6 {
        radius
    } else {
        DEFAULT_ANGLE_RADIUS_PX * wpp
    };
    r = r.max(MIN_ANGLE_RADIUS_PX * wpp);

    let mut normal = a.cross(b);
    if normal.length() < 1e-9 {
        // Parallel/degenerate rays: no arc, anchor along the (single) direction.
        return DimensionAnnotation {
            overlay: ov,
            label_anchor: vertex.add(a.scale(r)),
        };
    }
    normal = normal.normalized();
    let sweep = a.dot(b).clamp(-1.0, 1.0).acos();

    let pts = angular_arc_points(vertex, a, b, r);
    for w in pts.windows(2) {
        ov.line(w[0], w[1], color);
    }

    // Witness lines along each ray, out past the arc.
    let ext = r + ANGLE_EXT_PX * wpp;
    ov.line(vertex, vertex.add(a.scale(ext)), color);
    ov.line(vertex, vertex.add(b.scale(ext)), color);

    // Arrowheads at the arc ends, pointing tangentially outward.
    if pts.len() >= 2 {
        let n = pts.len();
        let start_dir = pts[0].sub(pts[1]).normalized();
        arrowhead(
            &mut ov,
            pts[0],
            start_dir,
            ARROW_LEN_PX * wpp,
            ARROW_HALF_WIDTH_PX * wpp,
            color,
        );
        let end_dir = pts[n - 1].sub(pts[n - 2]).normalized();
        arrowhead(
            &mut ov,
            pts[n - 1],
            end_dir,
            ARROW_LEN_PX * wpp,
            ARROW_HALF_WIDTH_PX * wpp,
            color,
        );
    }

    // Label anchor at the arc midpoint (on the bisector, at radius).
    let mid_dir = rotate_about_axis(a, normal, sweep * 0.5);
    let label_anchor = vertex.add(mid_dir.scale(r));

    DimensionAnnotation {
        overlay: ov,
        label_anchor,
    }
}

// --- radial dimension -------------------------------------------------------

/// A radial dimension: a leader from `center` out to `point_on_circle` with an
/// arrowhead at the circle (apex on the circle, opening inward along the
/// radius), plus a short landing leader continuing outward to the label anchor.
///
/// The label anchor sits just outside the circle, along the radial leader.
pub fn radial_dimension(
    center: Vec3,
    point_on_circle: Vec3,
    camera: &GizmoCamera,
) -> DimensionAnnotation {
    radial_dimension_colored(center, point_on_circle, camera, DIMENSION_COLOR)
}

/// [`radial_dimension`] with an explicit color.
pub fn radial_dimension_colored(
    center: Vec3,
    point_on_circle: Vec3,
    camera: &GizmoCamera,
    color: [f32; 4],
) -> DimensionAnnotation {
    let mut ov = Overlay::new();
    let radial = point_on_circle.sub(center);
    let dist = radial.length();
    let dir = if dist < 1e-9 {
        Vec3::X
    } else {
        radial.scale(1.0 / dist)
    };
    let wpp = camera.world_per_pixel(point_on_circle);

    // Leader from center to the circle.
    ov.line(center, point_on_circle, color);
    // Arrowhead at the circle, apex on the circle pointing outward, body inward.
    arrowhead(
        &mut ov,
        point_on_circle,
        dir,
        ARROW_LEN_PX * wpp,
        ARROW_HALF_WIDTH_PX * wpp,
        color,
    );
    // Short landing leader outward to the label.
    let gap = LABEL_GAP_PX * wpp;
    let anchor = point_on_circle.add(dir.scale(gap.max(0.0)));
    ov.line(point_on_circle, anchor, color);

    DimensionAnnotation {
        overlay: ov,
        label_anchor: anchor,
    }
}

