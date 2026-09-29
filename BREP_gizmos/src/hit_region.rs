//! Screen-space pickable regions — the SINGLE source of truth a gizmo's
//! hit-test AND its debug outline both consume, so "what you see (the red
//! outline) is exactly what you can click/drag."
//!
//! A gizmo projects each draggable handle, ONCE, to a cursor-independent
//! viewport-local screen-px [`HitShape`]:
//!   * a **capsule** (stadium) for SEGMENT handles — axis arrows, linear
//!     dimension leaders — the projected segment inflated by the hit radius;
//!   * a **circle** (disc) for POINT/BALL handles — center/origin spheres,
//!     rotation grab spheres, the angular arc handle.
//!
//! The engine hit path 2D-tests the cursor against these shapes
//! ([`HitShape::contains`]); the app strokes the very same shapes. Because both
//! consume the identical value they cannot drift.
//!
//! Projection + near-plane handling live here, in ONE place:
//!   * PERSPECTIVE — a segment crossing behind the eye is FRONT-CLIPPED to the
//!     eye-plane crossing (view-depth is affine along the segment) and only its
//!     visible sub-segment is used; a fully-behind segment / point is omitted.
//!   * ORTHOGRAPHIC (the app default) — behind-eye-plane geometry still renders,
//!     so everything is projected in FULL and nothing is clipped or omitted.

/// A viewport-local screen-px pickable region for ONE gizmo handle. The SAME
/// value the engine 2D-tests the cursor against and the app strokes as the red
/// debug outline — so the grabbable area and the drawn outline are identical.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum HitShape {
    /// Stadium/capsule: the segment `a → b` inflated by radius `r` (px). The
    /// region of a SEGMENT handle (axis arrow, linear dimension leader).
    Capsule { a: [f32; 2], b: [f32; 2], r: f32 },
    /// Disc: center `c`, radius `r` (px). The region of a POINT/BALL handle
    /// (center / origin sphere, rotation grab sphere, angular arc handle).
    Circle { c: [f32; 2], r: f32 },
}

impl HitShape {
    /// The hit radius (px).
    pub fn radius(&self) -> f32 {
        match self {
            HitShape::Capsule { r, .. } | HitShape::Circle { r, .. } => *r,
        }
    }

    /// 2D distance (px) from screen point `p` to this region's SPINE — the
    /// segment for a capsule, the center for a circle. The region CONTAINS `p`
    /// exactly when this is `<= radius()`.
    pub fn spine_distance(&self, p: [f32; 2]) -> f32 {
        match self {
            HitShape::Capsule { a, b, .. } => point_segment_distance(p, *a, *b),
            HitShape::Circle { c, .. } => dist2(p, *c),
        }
    }

    /// Whether the cursor `p` (viewport-local px) is inside this region.
    pub fn contains(&self, p: [f32; 2]) -> bool {
        self.spine_distance(p) <= self.radius()
    }
}

/// A perspective point at or behind the eye plane doesn't project; this is the
/// same `> 1e-6` in-front threshold the projection's `w > 1e-6` test uses.
const FRONT_EPS: f64 = 1e-6;

/// The minimal projection the region builders need — implemented for BOTH the
/// gizmo camera (f32) and the render camera (f64) so the front-clip + near-plane
/// rule live in exactly ONE place regardless of which gizmo asks.
pub trait RegionCamera {
    /// True for an orthographic projection: behind-eye-plane geometry still
    /// projects + renders, so it is never clipped or omitted.
    fn is_orthographic(&self) -> bool;
    /// Signed view-space depth of a world point (positive in front of the eye
    /// plane). Used only to decide front/behind for the perspective clip.
    fn depth(&self, p: [f64; 3]) -> f64;
    /// Project a world point to viewport-local screen px, or `None` if it is not
    /// projectable (perspective, at/behind the eye). Ortho always projects.
    fn project_px(&self, p: [f64; 3]) -> Option<[f32; 2]>;
}

/// The capsule region for the world segment `[a, b]` at pixel radius `r`.
/// PERSPECTIVE: front-clipped to the eye-plane crossing so a leader whose far
/// end is behind the eye still yields a capsule over its VISIBLE part; a segment
/// fully behind the eye returns `None`. ORTHOGRAPHIC: projected in full (both
/// endpoints), never clipped.
pub fn segment_region(cam: &impl RegionCamera, a: [f64; 3], b: [f64; 3], r: f32) -> Option<HitShape> {
    if cam.is_orthographic() {
        return Some(HitShape::Capsule {
            a: cam.project_px(a)?,
            b: cam.project_px(b)?,
            r,
        });
    }
    let da = cam.depth(a);
    let db = cam.depth(b);
    match (da > FRONT_EPS, db > FRONT_EPS) {
        (true, true) => Some(HitShape::Capsule {
            a: cam.project_px(a)?,
            b: cam.project_px(b)?,
            r,
        }),
        (true, false) => Some(HitShape::Capsule {
            a: cam.project_px(a)?,
            b: cam.project_px(clip_to_front(a, b, da, db))?,
            r,
        }),
        (false, true) => Some(HitShape::Capsule {
            a: cam.project_px(b)?,
            b: cam.project_px(clip_to_front(b, a, db, da))?,
            r,
        }),
        (false, false) => None, // both behind → genuinely invisible
    }
}

/// The circle region for the world point `p` at pixel radius `r`. `None` only
/// when `p` is behind the eye in PERSPECTIVE (invisible); ORTHOGRAPHIC always
/// projects.
pub fn point_region(cam: &impl RegionCamera, p: [f64; 3], r: f32) -> Option<HitShape> {
    if !cam.is_orthographic() && cam.depth(p) <= FRONT_EPS {
        return None;
    }
    Some(HitShape::Circle {
        c: cam.project_px(p)?,
        r,
    })
}

/// The point on the world segment from `front` (view-depth `d_front > 0`) toward
/// `back` (view-depth `d_back <= 0`) just on the visible side of the eye-plane
/// crossing. View-depth is affine along the segment, so the crossing fraction is
/// `d_front / (d_front - d_back)`; the `0.995` backoff keeps the projection off
/// the exact near plane (where it would emit exploded coords).
fn clip_to_front(front: [f64; 3], back: [f64; 3], d_front: f64, d_back: f64) -> [f64; 3] {
    let denom = d_front - d_back;
    let t = if denom.abs() < 1e-12 {
        0.0
    } else {
        (d_front / denom).clamp(0.0, 1.0) * 0.995
    };
    [
        front[0] + (back[0] - front[0]) * t,
        front[1] + (back[1] - front[1]) * t,
        front[2] + (back[2] - front[2]) * t,
    ]
}

/// 2D distance between two screen points.
fn dist2(p: [f32; 2], q: [f32; 2]) -> f32 {
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2)).sqrt()
}

/// 2D distance from point `p` to the segment `[a, b]` (both screen px). A
/// degenerate (zero-length) segment collapses to the point distance to `a`.
fn point_segment_distance(p: [f32; 2], a: [f32; 2], b: [f32; 2]) -> f32 {
    let abx = b[0] - a[0];
    let aby = b[1] - a[1];
    let len2 = abx * abx + aby * aby;
    if len2 < 1e-12 {
        return dist2(p, a);
    }
    let t = (((p[0] - a[0]) * abx + (p[1] - a[1]) * aby) / len2).clamp(0.0, 1.0);
    let foot = [a[0] + abx * t, a[1] + aby * t];
    dist2(p, foot)
}

