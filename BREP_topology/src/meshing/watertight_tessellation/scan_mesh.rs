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
pub fn enabled() -> bool {
    crate::switch!(live "BREP_SOUNDNESS_SCAN_MESH", true)
}

/// Open while the soundness scan tessellates: the analytic faces it meshes take
/// the bounded scan refinement when the switch is on. Display callers never
/// open it, so their meshes are unchanged.
pub struct ScanMeshScope {
    previous: bool,
}

impl ScanMeshScope {
    pub fn open() -> Self {
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

/// `(bounded, fallbacks)` so far, without resetting them (the check mode
/// reads a delta, leaving the counters to the profile line and tests).
pub fn peek_counts() -> (u64, u64) {
    (BOUNDED.load(Ordering::Relaxed), FALLBACKS.load(Ordering::Relaxed))
}

/// `(bounded, fallbacks)` since the last call (the `soundness.scan_mesh`
/// profile line).
pub fn take_counts() -> (u64, u64) {
    (BOUNDED.swap(0, Ordering::Relaxed), FALLBACKS.swap(0, Ordering::Relaxed))
}

/// A face's closed-form deviation bound, when its surface has one.
#[derive(Clone, Debug)]
pub enum FaceBound {
    Plane,
    Sphere { centre: Vec3, radius: f64 },
    Ruled { origin: Vec3, axis: Vec3, rho0: f64, rho1: f64, height: f64 },
    Affine { sphere: crate::affine_sphere::AffineSphere, largest_semi_axis: f64 },
    /// A polynomial spline carrier (`BREP_SCAN_MESH_SPLINE`). The bound is
    /// read from the triangle's uv corners and the net's second differences
    /// ([`SplineBound::triangle`]), plus each corner's residual, in the
    /// interior bounded refinement. The rectangle grid is skipped for it.
    Spline(std::sync::Arc<SplineBound>),
}

/// `BREP_SCAN_MESH_SPLINE` (default on since the 2026-11-20 tier-P
/// promotion, after the boundary sag gate made the Utah teapot neutral; `=0`
/// restores the display refinement): the scan meshes a polynomial
/// spline face with a SOUND deviation bound ([`SplineBound::triangle`]) in
/// the bounded refinement, in place of the display's sampled chord and
/// angular checks (and its rectangle grid attempt).
pub fn spline_enabled() -> bool {
    spline_force().unwrap_or_else(|| crate::switch!(live "BREP_SCAN_MESH_SPLINE", true))
}

/// `BREP_SCAN_MESH_SPLINE_CHECK=1`: the scan runs twice, the spline bound
/// forced off and then on, and prints one `soundness.spline_check` line
/// comparing the reports. The forced-off report is returned.
pub fn spline_check() -> bool {
    crate::debug_flag!("BREP_SCAN_MESH_SPLINE_CHECK")
}

thread_local! {
    static SPLINE_FORCE: Cell<Option<bool>> = const { Cell::new(None) };
}

/// The check mode's forced reading of the switch on this thread, if any.
pub fn spline_force() -> Option<bool> {
    SPLINE_FORCE.with(|cell| cell.get())
}

/// Force the spline bound on this thread (`None` clears it), restoring the
/// previous state on drop; carried onto the tessellator's workers per face.
pub struct SplineForceScope {
    previous: Option<bool>,
}

impl SplineForceScope {
    pub fn set(force: Option<bool>) -> Self {
        Self { previous: SPLINE_FORCE.with(|cell| cell.replace(force)) }
    }
}

impl Drop for SplineForceScope {
    fn drop(&mut self) {
        SPLINE_FORCE.with(|cell| cell.set(self.previous));
    }
}

/// Second-derivative bounds of a polynomial tensor spline, per knot span.
///
/// The surface is `S(u, v) = Σ N_i,p(u) N_j,q(v) P_ij` with all weights equal,
/// so a polynomial in each span. Its second partials are splines of the same
/// form, whose controls are scaled second differences of the net:
/// - `S_uu`: `R_ij = (p-1) (Q_{i+1,j} - Q_ij) / (u_{i+p+1} - u_{i+2})`, with
///   `Q_ij = p (P_{i+1,j} - P_ij) / (u_{i+p+1} - u_{i+1})`;
/// - `S_vv`: the same in v;
/// - `S_uv`: `p q (P_{i+1,j+1} - P_{i,j+1} - P_{i+1,j} + P_ij) / ((u_{i+p+1} - u_{i+1}) (v_{j+q+1} - v_{j+1}))`.
///
/// A control whose denominator vanishes multiplies a basis function that
/// vanishes identically, and is skipped. On the span `[u_k, u_{k+1}] ×
/// [v_l, v_{l+1}]` only the controls whose basis functions are nonzero there
/// weigh. The basis is nonnegative and sums to one, so the length of each
/// partial is at most the largest length among those controls. That is the
/// span's entry `[M_uu, M_uv, M_vv]`.
///
/// A carrier with an interior knot of multiplicity at least its degree (a
/// C0 line, where `S_u` or `S_v` jumps) gets no bound.
#[derive(Debug)]
pub struct SplineBound {
    knots_u: Vec<f64>,
    knots_v: Vec<f64>,
    degree_u: usize,
    degree_v: usize,
    /// Spans `k` in `degree_u..rows`, `l` in `degree_v..columns`.
    spans_v: usize,
    table: Vec<[f64; 3]>,
}

impl SplineBound {
    pub fn new(surface: &crate::NurbsSurface) -> Option<Self> {
        let net = &surface.control_points;
        let (p, q) = (surface.degree_u, surface.degree_v);
        let rows = net.len();
        let columns = net.first()?.len();
        let (ku, kv) = (&surface.knots_u, &surface.knots_v);
        if rows <= p || columns <= q || net.iter().any(|row| row.len() != columns)
            || ku.len() != rows + p + 1 || kv.len() != columns + q + 1
        {
            return None;
        }
        let w0 = net[0][0].w;
        if !(w0.is_finite() && w0 > 0.0) || net.iter().flatten().any(|c| c.w != w0) {
            return None;
        }
        let points: Vec<Vec<Vec3>> = net
            .iter()
            .map(|row| row.iter().map(|c| c.point().ok().filter(|x| x.x.is_finite() && x.y.is_finite() && x.z.is_finite())).collect())
            .collect::<Option<_>>()?;
        // No interior C0 line.
        let c0 = |knots: &[f64], degree: usize, count: usize| {
            (degree + 1..count).any(|k| knots[k] > knots[degree] && knots[k] < knots[count] && {
                knots.iter().filter(|&&x| x == knots[k]).count() >= degree.max(1)
            })
        };
        if c0(ku, p, rows) || c0(kv, q, columns) {
            return None;
        }
        let ratio = |num: f64, den: f64| if den > 0.0 { num / den } else { 0.0 };
        // First differences along u: Qu[i][j], i < rows - 1.
        let qu: Vec<Vec<Vec3>> = (0..rows.saturating_sub(1))
            .map(|i| (0..columns).map(|j| points[i + 1][j].sub(points[i][j]).scale(ratio(p as f64, ku[i + p + 1] - ku[i + 1]))).collect())
            .collect();
        let qv: Vec<Vec<Vec3>> = (0..rows)
            .map(|i| (0..columns.saturating_sub(1)).map(|j| points[i][j + 1].sub(points[i][j]).scale(ratio(q as f64, kv[j + q + 1] - kv[j + 1]))).collect())
            .collect();
        let spans_u = rows - p;
        let spans_v = columns - q;
        let mut table = vec![[0.0f64; 3]; spans_u * spans_v];
        for k in p..rows {
            for l in q..columns {
                let mut m = [0.0f64; 3];
                if p >= 2 {
                    for i in k - p..=k - 2 {
                        let den = ku[i + p + 1] - ku[i + 2];
                        if !(den > 0.0) { continue; }
                        for j in l - q..=l {
                            m[0] = m[0].max(qu[i + 1][j].sub(qu[i][j]).scale((p - 1) as f64 / den).length());
                        }
                    }
                }
                if p >= 1 && q >= 1 {
                    for i in k - p..=k - 1 {
                        let den_u = ku[i + p + 1] - ku[i + 1];
                        if !(den_u > 0.0) { continue; }
                        for j in l - q..=l - 1 {
                            let den_v = kv[j + q + 1] - kv[j + 1];
                            if !(den_v > 0.0) { continue; }
                            let twist = points[i + 1][j + 1].sub(points[i][j + 1]).sub(points[i + 1][j]).add(points[i][j]);
                            m[1] = m[1].max(twist.scale((p * q) as f64 / (den_u * den_v)).length());
                        }
                    }
                }
                if q >= 2 {
                    for j in l - q..=l - 2 {
                        let den = kv[j + q + 1] - kv[j + 2];
                        if !(den > 0.0) { continue; }
                        for i in k - p..=k {
                            m[2] = m[2].max(qv[i][j + 1].sub(qv[i][j]).scale((q - 1) as f64 / den).length());
                        }
                    }
                }
                if !m.iter().all(|x| x.is_finite()) {
                    return None;
                }
                table[(k - p) * spans_v + (l - q)] = m;
            }
        }
        Some(Self { knots_u: ku.clone(), knots_v: kv.clone(), degree_u: p, degree_v: q, spans_v, table })
    }

    /// The length of the uv edge `a`–`b` in the bound's own metric,
    /// `√(M_uu du² + |M_uv du dv| + M_vv dv²)` over the spans it crosses:
    /// what a split of that edge takes out of the bound. Splitting the
    /// longest edge in this metric shrinks a sliver across its long side;
    /// the largest-own-bound rule could keep bisecting its short one.
    pub fn edge_metric(&self, a: [f64; 2], b: [f64; 2]) -> Option<f64> {
        let [muu, muv, mvv] = self.over([a[0].min(b[0]), a[0].max(b[0]), a[1].min(b[1]), a[1].max(b[1])])?;
        let (du, dv) = (b[0] - a[0], b[1] - a[1]);
        let value = (muu * du * du + muv * (du * dv).abs() + mvv * dv * dv).sqrt();
        value.is_finite().then_some(value)
    }

    /// The largest `[M_uu, M_uv, M_vv]` over the spans meeting `[u0, u1] ×
    /// [v0, v1]`; `None` outside the domain.
    fn over(&self, [u0, u1, v0, v1]: [f64; 4]) -> Option<[f64; 3]> {
        let span = |knots: &[f64], degree: usize, x: f64| -> Option<usize> {
            let count = knots.len() - degree - 1;
            if !(x >= knots[degree] && x <= knots[count]) {
                return None;
            }
            // The last span whose start is at or below x (the closed spans
            // meeting x are this one and, at a knot, the one before).
            let mut k = degree;
            while k + 1 < count && knots[k + 1] <= x {
                k += 1;
            }
            Some(k)
        };
        let (p, q) = (self.degree_u, self.degree_v);
        // A corner on a knot also touches the span that ends there.
        let low = |knots: &[f64], degree: usize, x: f64| span(knots, degree, x).map(|k| if k > degree && knots[k] == x { k - 1 } else { k });
        let (ka, kb) = (low(&self.knots_u, p, u0)?, span(&self.knots_u, p, u1)?);
        let (la, lb) = (low(&self.knots_v, q, v0)?, span(&self.knots_v, q, v1)?);
        let mut m = [0.0f64; 3];
        for k in ka..=kb {
            for l in la..=lb {
                let entry = self.table[(k - p) * self.spans_v + (l - q)];
                for slot in 0..3 {
                    m[slot] = m[slot].max(entry[slot]);
                }
            }
        }
        Some(m)
    }

    /// An upper bound on `|L(x) - S(x)|` over the uv triangle `uv`, where `L`
    /// is the linear interpolant of `S` at its corners.
    ///
    /// Proof. For `x = Σ λ_i v_i`, Taylor's theorem with integral remainder
    /// along each segment `x → v_i` gives
    /// `Σ λ_i S(v_i) − S(x) = Σ λ_i ∫₀¹ (1−t) D²S[v_i − x]² dt`.
    /// The linear terms cancel because `Σ λ_i (v_i − x) = 0`. This needs `S`
    /// to be C1 with a bounded, piecewise continuous second derivative, which
    /// holds without interior C0 lines.
    ///
    /// Next, `|D²S[d]²| ≤ M_uu du² + 2 M_uv |du dv| + M_vv dv² ≤ Q(d)`, with
    /// `Q(d) = (M_uu + α M_uv) du² + (M_vv + M_uv / α) dv²` for any `α > 0`
    /// (`2|du dv| ≤ α du² + dv² / α`). So the error is at most
    /// `½ Σ λ_i Q(v_i − x)`. The weight is chosen per centre as
    /// `√(Dv / Du)` from the corners' squared extents about it, which keeps
    /// a long, thin rim strip from paying its long side's length in the cross
    /// term. When every corner shares one u (or one v), each `d` inside has
    /// no cross term at all.
    ///
    /// For the quadratic form `Q` and any centre `c`, that sum equals
    /// `Σ λ_i Q(v_i − c) − Q(x − c) ≤ max_i Q(v_i − c)`. The bound takes the
    /// best of the centroid and the three edge midpoints. For a right
    /// triangle the hypotenuse midpoint gives exactly a quarter of `Q` of the
    /// hypotenuse.
    pub fn triangle(&self, uv: [[f64; 2]; 3]) -> Option<f64> {
        let lo = |k: usize| uv.iter().map(|p| p[k]).fold(f64::INFINITY, f64::min);
        let hi = |k: usize| uv.iter().map(|p| p[k]).fold(f64::NEG_INFINITY, f64::max);
        let [muu, muv, mvv] = self.over([lo(0), hi(0), lo(1), hi(1)])?;
        // About centre c, with the cross term split by the weight α that
        // suits this triangle's extents (any α > 0 is valid).
        let form = |c: [f64; 2]| {
            let du = uv.iter().map(|p| (p[0] - c[0]).powi(2)).fold(0.0f64, f64::max);
            let dv = uv.iter().map(|p| (p[1] - c[1]).powi(2)).fold(0.0f64, f64::max);
            let (a, b) = if du > 0.0 && dv > 0.0 {
                let alpha = (dv / du).sqrt();
                (muu + alpha * muv, mvv + muv / alpha)
            } else {
                // Every corner on one iso line: every d along it has no cross term.
                (muu, mvv)
            };
            uv.iter().map(|p| a * (p[0] - c[0]).powi(2) + b * (p[1] - c[1]).powi(2)).fold(0.0f64, f64::max)
        };
        let mid = |i: usize, j: usize| [(uv[i][0] + uv[j][0]) * 0.5, (uv[i][1] + uv[j][1]) * 0.5];
        let centroid = [(uv[0][0] + uv[1][0] + uv[2][0]) / 3.0, (uv[0][1] + uv[1][1] + uv[2][1]) / 3.0];
        let best = [form(centroid), form(mid(0, 1)), form(mid(1, 2)), form(mid(2, 0))].into_iter().fold(f64::INFINITY, f64::min);
        let value = 0.5 * best;
        value.is_finite().then_some(value)
    }
}

/// The bound kind of `face`, or `None` (keep the display refinement).
pub fn face_bound(face: &FaceRecord) -> Option<FaceBound> {
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
    if let Some(affine) = face.surface.affine_sphere() {
        let largest = affine.semi_axes()[0];
        if !(largest.is_finite() && largest > 0.0) {
            return None;
        }
        return Some(FaceBound::Affine { sphere: affine.clone(), largest_semi_axis: largest });
    }
    if spline_enabled() && face.surface.analytic().is_none() {
        return SplineBound::new(&face.surface).map(|bound| FaceBound::Spline(std::sync::Arc::new(bound)));
    }
    None
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
pub fn triangle_bound(bound: &FaceBound, t: [Vec3; 3]) -> Option<f64> {
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
        // Read from the uv corners by the bounded refinement (`SplineBound`).
        FaceBound::Spline(_) => return None,
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

