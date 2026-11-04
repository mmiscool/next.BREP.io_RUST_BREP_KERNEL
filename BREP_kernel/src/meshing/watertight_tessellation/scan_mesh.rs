//! The self-intersection scan's mesh on analytic faces (owner-approved option A,
//! 2026-11-03), behind `BREP_SOUNDNESS_SCAN_MESH` (default ON since the
//! 2026-11-03 promotion; `BREP_SOUNDNESS_SCAN_MESH=0` restores the display mesh for a same-binary A/B).
//!
//! The display refinement splits an interior edge when the surface at its uv
//! midpoint sags more than the chord from the edge (`refine_interior`): one
//! surface evaluation per edge per pass, plus an angular-normal gate. That is
//! a heuristic: nothing bounds how far a triangle's interior strays from the
//! surface. For the scan, on faces whose surface is a plane, a sphere, a
//! cylinder or cone (`RuledRevolution`) or an affine sphere, this module gives
//! a CLOSED-FORM upper bound on the distance from every point of a mesh
//! triangle to the surface, from the triangle's three vertex positions alone
//! (the vertices are surface evaluations), and the scan's refinement splits
//! until every triangle that has a splittable edge is within the chord:
//!
//! - plane: every vertex on the plane, so the triangle is in it; the bound is
//!   the rounding pad.
//! - sphere (centre c, radius r): the triangle lies in the ball through its
//!   vertices (convex), so no point is further outside than the furthest
//!   vertex; inside, a point x is at least `dist(c, T)` from the centre, so
//!   `r - |x - c| <= r - dist(c, T)`. Bound: `max(r, max_i |v_i - c|) -
//!   dist(c, T)` plus the vertices' outward residual.
//! - cylinder / cone (`RuledRevolution`, generatrix a line from rho0 at z = 0
//!   to rho1 at z = height): the distance rho(x) to the axis is convex and the
//!   surface's radius at x's height is linear in x, so their difference is
//!   convex and never above its vertex values (no point lies outside); inside,
//!   the horizontal gap to the surface at x's height is at most
//!   `max_i rho_i - rho(x) <= max_i rho_i - dist(axis, T)`, and the distance to
//!   the surface is at most that gap.
//! - affine sphere (`M s + c`): map the triangle back by `M^-1` to a triangle
//!   with vertices on the unit sphere, bound it there as a sphere, and scale by
//!   the largest semi-axis (the largest singular value of `M`).
//!
//! Every bound carries a rounding pad. Tori and general revolutions keep the
//! display refinement (no closed form here). The candidate test (exact
//! triangle crossings) and the exact confirmation on the carriers are
//! unchanged; only the mesh's guarantee and its cost change.
use crate::topology::FaceRecord;
use crate::Vec3;
use std::cell::Cell;
use std::sync::atomic::{AtomicU64, Ordering};

thread_local! {
    static ACTIVE: Cell<bool> = const { Cell::new(false) };
}

/// `BREP_SOUNDNESS_SCAN_MESH` (default ON; `=0` meshes the scan with the display refinement).
pub(crate) fn enabled() -> bool {
    crate::switch!(live "BREP_SOUNDNESS_SCAN_MESH", true)
}

/// Open while the soundness scan tessellates: the analytic faces it meshes take
/// the bounded scan refinement when the switch is on. Display callers never
/// open it, so their meshes are unchanged.
pub(crate) struct ScanMeshScope {
    previous: bool,
}

impl ScanMeshScope {
    pub(crate) fn open() -> Self {
        Self::set(enabled())
    }

    /// Carry the calling thread's mode onto a rayon worker for one face: set
    /// explicitly per face (true or false), so a worker that steals unrelated
    /// meshing work never inherits a stale mode.
    pub(super) fn set(on: bool) -> Self {
        Self { previous: ACTIVE.with(|cell| cell.replace(on)) }
    }
}

impl Drop for ScanMeshScope {
    fn drop(&mut self) {
        ACTIVE.with(|cell| cell.set(self.previous));
    }
}

pub(super) fn active() -> bool {
    ACTIVE.with(|cell| cell.get())
}

static BOUNDED: AtomicU64 = AtomicU64::new(0);
static FALLBACKS: AtomicU64 = AtomicU64::new(0);

/// Count one analytic face meshed by the bounded refinement (`true`) or sent
/// back to the display refinement after the pass budget (`false`).
pub(super) fn count(bounded: bool) {
    if bounded { &BOUNDED } else { &FALLBACKS }.fetch_add(1, Ordering::Relaxed);
}

/// `(bounded, fallbacks)` since the last call (the `soundness.scan_mesh`
/// profile line).
pub(crate) fn take_counts() -> (u64, u64) {
    (BOUNDED.swap(0, Ordering::Relaxed), FALLBACKS.swap(0, Ordering::Relaxed))
}

/// A face's closed-form deviation bound, when its surface has one.
#[derive(Clone, Debug)]
pub(super) enum FaceBound {
    Plane,
    Sphere { centre: Vec3, radius: f64 },
    Ruled { origin: Vec3, axis: Vec3, rho0: f64, rho1: f64, height: f64 },
    Affine { sphere: crate::affine_sphere::AffineSphere, largest_semi_axis: f64 },
}

/// The bound kind of `face`, or `None` (keep the display refinement).
pub(super) fn face_bound(face: &FaceRecord) -> Option<FaceBound> {
    if !active() {
        return None;
    }
    use crate::AnalyticSurface;
    match face.surface.analytic() {
        Some(AnalyticSurface::Plane { .. }) => return Some(FaceBound::Plane),
        Some(AnalyticSurface::Sphere { frame, radius }) => {
            return (radius.is_finite() && *radius > 0.0).then_some(FaceBound::Sphere { centre: frame.origin, radius: *radius });
        }
        Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) => {
            return (rho0.is_finite() && rho1.is_finite() && height.is_finite() && *height > 0.0).then_some(FaceBound::Ruled {
                origin: frame.origin,
                axis: frame.axis,
                rho0: *rho0,
                rho1: *rho1,
                height: *height,
            });
        }
        _ => {}
    }
    let affine = face.surface.affine_sphere()?;
    let largest = affine.semi_axes()[0];
    if !(largest.is_finite() && largest > 0.0) {
        return None;
    }
    Some(FaceBound::Affine { sphere: affine.clone(), largest_semi_axis: largest })
}

/// Rounding pad for a quantity at magnitude `scale`.
fn pad(scale: f64) -> f64 {
    256.0 * f64::EPSILON * scale.max(1.0)
}

/// Does a cone face's mesh reach (within `margin`) the carrier's APEX, the
/// one point where a ruled carrier's normal is not unique? The fold
/// confirmation holds a same-face crossing's feet to the mesh's chord band, and
/// within that band of an apex two feet on different rulings read as one point
/// with normals the cone's full angle apart: a dense mesh there offers a fold
/// that is not one (2026-11-03, `a_lone_conical_pocket_caps`: face 7's feet
/// 5.7e-2 apart against a 6e-2 band, normals 51.8 degrees). Such a face keeps
/// the display refinement, whose mesh the scan has always read. Axial heights
/// only, so a face trimmed short of the apex is never sent back by accident of
/// its radius.
pub(super) fn reaches_apex(bound: &FaceBound, positions: impl Iterator<Item = Vec3>, margin: f64) -> bool {
    let FaceBound::Ruled { origin, axis, rho0, rho1, height } = *bound else {
        return false;
    };
    if rho0 == rho1 {
        return false;
    }
    let apex = rho0 * height / (rho0 - rho1);
    let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in positions {
        let z = p.sub(origin).dot(axis);
        low = low.min(z);
        high = high.max(z);
    }
    // No vertex at all (or a non-finite one): treat as reaching, the safe side.
    !(low <= high) || (apex >= low - margin && apex <= high + margin)
}

/// An upper bound on the distance from any point of the triangle `t` to the
/// face's surface (see the module note). `None` for a non-finite input.
pub(super) fn triangle_bound(bound: &FaceBound, t: [Vec3; 3]) -> Option<f64> {
    if t.iter().any(|p| !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite())) {
        return None;
    }
    let scale = t.iter().map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs())).fold(0.0f64, f64::max);
    let value = match *bound {
        FaceBound::Plane => 0.0,
        FaceBound::Sphere { centre, radius } => sphere_bound(centre, radius, t),
        FaceBound::Ruled { origin, axis, rho0, rho1, height } => {
            let radial = |p: Vec3| {
                let d = p.sub(origin);
                d.sub(axis.scale(d.dot(axis)))
            };
            let projected = [radial(t[0]), radial(t[1]), radial(t[2])];
            // The generatrix radius at each vertex's height, and the vertex's own.
            let surface_rho = |p: Vec3| rho0 + (rho1 - rho0) * (p.sub(origin).dot(axis) / height);
            let mut reach = 0.0f64;
            let mut outward = 0.0f64;
            for (p, q) in t.iter().zip(&projected) {
                let (rho, surface) = (q.length(), surface_rho(*p));
                reach = reach.max(rho).max(surface);
                outward = outward.max(rho - surface);
            }
            // Inside: the horizontal gap `rho_s(z(x)) - rho(x)`. Two upper
            // bounds, the smaller taken. (a) `rho(x) >= dist(axis, T)` and
            // `rho_s <= reach`. (b) For any unit radial direction n,
            // `rho(x) >= <x_perp, n>`, so the gap is below the LINEAR function
            // `rho_s(z(x)) - <x_perp, n>`, whose maximum is at a vertex; n is
            // the bisector of the widest pair of radial directions (exact sag
            // on a cylinder; tight on a cone, where (a) reads the radius change).
            let inner = point_triangle_distance(Vec3::default(), projected);
            let mut inside = reach - inner;
            if let Some(n) = bisector(projected) {
                let linear = t
                    .iter()
                    .zip(&projected)
                    .map(|(p, q)| surface_rho(*p) - q.dot(n))
                    .fold(f64::NEG_INFINITY, f64::max);
                inside = inside.min(linear);
            }
            inside.max(0.0) + outward.max(0.0)
        }
        FaceBound::Affine { ref sphere, largest_semi_axis } => {
            // `M` is affine, so the preimage of the triangle is the triangle of
            // the preimages, and `|M x - M y| <= sigma_max |x - y|` with the
            // largest singular value sigma_max = the largest semi-axis.
            let back = [sphere.preimage(t[0]), sphere.preimage(t[1]), sphere.preimage(t[2])];
            largest_semi_axis * sphere_bound(Vec3::default(), 1.0, back)
        }
    };
    value.is_finite().then(|| value + pad(scale))
}

/// The unit bisector of the two radial directions of `projected` that are
/// furthest apart, when the third lies between them (the three span under a
/// half turn); any unit radial direction keeps the bound valid, this one makes
/// it the exact sag of the angular span. `None` when a direction is
/// undefined or the span is too wide.
fn bisector(projected: [Vec3; 3]) -> Option<Vec3> {
    let mut directions = [Vec3::default(); 3];
    for (slot, q) in directions.iter_mut().zip(&projected) {
        let length = q.length();
        if !(length > 0.0) {
            return None;
        }
        *slot = q.scale(1.0 / length);
    }
    let pairs = [(0, 1, 2), (1, 2, 0), (0, 2, 1)];
    let (i, j, k) = pairs
        .into_iter()
        .min_by(|a, b| directions[a.0].dot(directions[a.1]).total_cmp(&directions[b.0].dot(directions[b.1])))?;
    let sum = directions[i].add(directions[j]);
    let length = sum.length();
    if !(length > 1e-6) {
        return None;
    }
    let n = sum.scale(1.0 / length);
    // The third direction must sit inside the arc: no closer to -n than the
    // pair's own ends.
    (directions[k].dot(n) >= directions[i].dot(n) - 1e-12).then_some(n)
}

fn sphere_bound(centre: Vec3, radius: f64, t: [Vec3; 3]) -> f64 {
    let reach = t.iter().map(|p| p.sub(centre).length()).fold(0.0f64, f64::max);
    let inner = point_triangle_distance(centre, t);
    (reach.max(radius) - inner).max(0.0) + (reach - radius).max(0.0)
}

/// A LOWER bound on the distance from `p` to the closed triangle `t`: the
/// exact distance (Ericson's closest-point regions), less a rounding pad,
/// with degenerate triangles read as their three edges.
pub(super) fn point_triangle_distance(p: Vec3, t: [Vec3; 3]) -> f64 {
    let segment = |a: Vec3, b: Vec3| -> f64 {
        let ab = b.sub(a);
        let length_squared = ab.dot(ab);
        if !(length_squared > 0.0) {
            return p.sub(a).length();
        }
        let s = (p.sub(a).dot(ab) / length_squared).clamp(0.0, 1.0);
        p.sub(a.add(ab.scale(s))).length()
    };
    let edges = segment(t[0], t[1]).min(segment(t[1], t[2])).min(segment(t[2], t[0]));
    let normal = t[1].sub(t[0]).cross(t[2].sub(t[0]));
    let area_squared = normal.dot(normal);
    let magnitude = t.iter().map(|q| q.sub(p).length()).fold(0.0f64, f64::max);
    let mut best = edges;
    if area_squared > 0.0 {
        // The foot of p on the plane, and whether it is inside the triangle.
        let n = normal.scale(1.0 / area_squared.sqrt());
        let height = p.sub(t[0]).dot(n);
        let foot = p.sub(n.scale(height));
        let inside = (0..3).all(|i| {
            let a = t[i];
            let b = t[(i + 1) % 3];
            b.sub(a).cross(foot.sub(a)).dot(normal) >= 0.0
        });
        if inside {
            best = best.min(height.abs());
        }
    }
    (best - pad(magnitude)).max(0.0)
}

