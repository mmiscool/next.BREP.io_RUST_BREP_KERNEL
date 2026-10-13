//! TANGENT-CURVE LANE — march the curve along which two carriers TOUCH.
//!
//! Where two surfaces are tangent along a whole curve (a cylinder resting on
//! a plane, a torus standing on a table, a sphere inscribed in its bore) the
//! ordinary marcher has nothing to walk: its step direction `n_a × n_b`
//! vanishes everywhere on the contact and its corrector, which solves
//! `S_a = S_b` alone, is free to drift ACROSS the contact band — the
//! surfaces agree to first order there, so the gap only grows with the SQUARE
//! of the across-offset and a gap tolerance resolves nothing. The imprint
//! therefore refuses such pairs by name (`csg/imprint/tangent_contact.rs`), or
//! worse, an exact lane reports them "provably empty" and the contact is lost
//! without a word.
//!
//! * **The direction.** Write both carriers as graphs over their common
//!   tangent plane at a contact point. The height difference `g = f_a − f_b`
//!   is stationary there and its Hessian is `H = II_a − II_b`, the difference
//!   of the two second fundamental forms against ONE normal. Along a contact
//!   curve the two surfaces agree to second order, so `H` is rank-deficient
//!   and its null eigenvector IS the curve's tangent.
//! * **The corrector.** Every station solves the CONTACT system
//!   `{S_a = S_b, n_a × n_b = 0}` (five residuals, the two cross components in
//!   `n_a`'s tangent basis) plus the section-plane constraint
//!   `(S_a − predicted) · d = 0`, by damped Gauss–Newton. The cross-normal
//!   residual is LINEAR in the across-offset where the gap is quadratic, so
//!   the station is pinned onto the contact to the same precision an ordinary
//!   station is pinned onto its curve.
//! * **The step.** The reactive controller of the ordinary march (turn of the
//!   direction between stations) under the §4.12 normal-turn ceiling read
//!   along the null direction, a stop at a non-closed domain boundary of
//!   either carrier, and closure when the trace returns to its start.
//! * **The type.** The curve comes back as a [`SurfaceIntersectionCurve`]
//!   with `tangent = true`, so a consumer can tell a crease that separates
//!   nothing from a section that separates inside from outside.
//!
//! What this module does NOT decide: whether the contact lies inside the
//! faces' trims, and what the boolean should DO with a tangent section (a
//! tangent edge both of whose sides survive is a non-manifold incidence the
//! assembler cannot represent, see `csg/boolean/mod.rs`'s pinch gate). Those
//! are the imprint's and the boolean's questions; this lane only finds the
//! curve and names it.

use crate::curve::interior_knot_count;
use crate::fit::solve_small;
use crate::surface_surface_intersection::{SurfaceIntersectionCurve, SurfaceIntersectionOptions};
use crate::{project_point_to_surface, NurbsSurface, Vec3};

const EPSILON: f64 = 1e-12;

/// Smallest `min|λ| / κ` of `H = II_a − II_b` at which the contact counts as
/// an isolated node or point rather than EXTENDED along `H`'s null direction.
/// The same number `csg/imprint/tangent_contact.rs` reads: genuine nodes
/// measure `0.7..1.0`, extended contacts `1e-15` and below, so this is a rank
/// test, not a tolerance band.
pub const EXTENDED_RANK_FLOOR: f64 = 1e-6;

/// Gauss–Newton iterations refining a witness onto the contact.
const REFINE_ITERATIONS: usize = 60;
/// Corrector iterations per station.
const CORRECT_ITERATIONS: usize = 30;
/// Relative residual improvement that counts as progress, and the number of
/// consecutive iterations without it that end a refinement.
const RESIDUAL_PROGRESS: f64 = 1e-3;
const REFINE_PATIENCE: usize = 4;
/// Levenberg diagonal relative to the largest normal-equation diagonal: large
/// enough to clear the solver's pivot floor on the rank-deficient system an
/// extended contact produces, small enough to leave a well-conditioned step
/// unchanged to nine figures.
const LEVENBERG_DAMPING: f64 = 1e-10;
/// Relative finite-difference step for the Jacobian, in each parameter span.
const JACOBIAN_STEP: f64 = 1e-7;
/// Direction turn across a step that pops the step (re-stepped at half), the
/// turn under which the step grows, and the normal-turn budget per step —
/// the ordinary march's values.
const TURN_POP: f64 = 0.35;
const TURN_GROW: f64 = 0.03;
const TURN_SHRINK: f64 = 0.15;
const NORMAL_TURN_BUDGET: f64 = 0.1;
/// Reach floor of the trace in carrier extents, as the ordinary march's.
const MARCH_REACH_EXTENTS: f64 = 4.0;
/// Distance gate (fraction of the carriers' extent) inside which a grid sample
/// is a contact candidate at all.
const CANDIDATE_GATE_FRACTION: f64 = 1e-2;
/// How many ranked grid samples the detection tries to refine before giving up.
const CANDIDATE_ATTEMPTS: usize = 4;

#[derive(Clone, Copy)]
struct Domain {
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
    closed_u: bool,
    closed_v: bool,
}

impl Domain {
    fn of(surface: &NurbsSurface) -> Result<Self, String> {
        let [u0, u1] = surface.domain_u()?;
        let [v0, v1] = surface.domain_v()?;
        let mut closed_u = true;
        let mut closed_v = true;
        for fraction in [0.17, 0.5, 0.83] {
            let v = v0 + (v1 - v0) * fraction;
            if surface.evaluate(u0, v)?.sub(surface.evaluate(u1, v)?).length() > 1e-6 {
                closed_u = false;
            }
            let u = u0 + (u1 - u0) * fraction;
            if surface.evaluate(u, v0)?.sub(surface.evaluate(u, v1)?).length() > 1e-6 {
                closed_v = false;
            }
        }
        Ok(Self { u0, u1, v0, v1, closed_u, closed_v })
    }

    fn spans(self) -> [f64; 2] {
        [self.u1 - self.u0, self.v1 - self.v0]
    }

    /// `value` brought into the domain: wrapped when closed, clamped
    /// otherwise. The flag says whether a clamp actually moved it.
    fn fit(self, value: f64, direction_u: bool) -> (f64, bool) {
        let (minimum, maximum, closed) = if direction_u {
            (self.u0, self.u1, self.closed_u)
        } else {
            (self.v0, self.v1, self.closed_v)
        };
        if closed {
            ((value - minimum).rem_euclid(maximum - minimum) + minimum, false)
        } else {
            let fitted = value.clamp(minimum, maximum);
            (fitted, fitted != value)
        }
    }
}

/// One point of the contact on both carriers.
#[derive(Clone, Copy, Debug)]
struct Station {
    ua: f64,
    va: f64,
    ub: f64,
    vb: f64,
    point: Vec3,
}

impl Station {
    fn parameter(&self, index: usize) -> f64 {
        match index {
            0 => self.ua,
            1 => self.va,
            2 => self.ub,
            _ => self.vb,
        }
    }

    fn with_parameter(mut self, index: usize, value: f64) -> Self {
        match index {
            0 => self.ua = value,
            1 => self.va = value,
            2 => self.ub = value,
            _ => self.vb = value,
        }
        self
    }
}

/// The tangent-plane frame at a station: `n_a`, `n_b`, and an orthonormal
/// basis of `n_a`'s tangent plane in which the cross-normal residual and the
/// fundamental forms are written.
#[derive(Clone, Copy)]
struct Frame {
    normal_a: Vec3,
    normal_b: Vec3,
    e1: Vec3,
    e2: Vec3,
}

fn frame_at(first: &NurbsSurface, second: &NurbsSurface, station: Station) -> Option<Frame> {
    let normal_a = first.normal(station.ua, station.va).ok()?;
    let normal_b = second.normal(station.ub, station.vb).ok()?;
    let e1 = normal_a.perpendicular().ok()?;
    let e2 = normal_a.cross(e1).normalized().ok()?;
    Some(Frame { normal_a, normal_b, e1, e2 })
}

/// The five contact residuals at `station`, the cross-normal components in
/// the FIXED basis `(e1, e2)` so a finite difference across the station reads
/// one smooth function and not a basis choice.
fn contact_residual(
    first: &NurbsSurface,
    second: &NurbsSurface,
    station: Station,
    e1: Vec3,
    e2: Vec3,
) -> Option<[f64; 5]> {
    let point_a = first.evaluate(station.ua, station.va).ok()?;
    let point_b = second.evaluate(station.ub, station.vb).ok()?;
    let normal_a = first.normal(station.ua, station.va).ok()?;
    let normal_b = second.normal(station.ub, station.vb).ok()?;
    let cross = normal_a.cross(normal_b);
    let gap = point_a.sub(point_b);
    Some([gap.x, gap.y, gap.z, cross.dot(e1), cross.dot(e2)])
}

/// One damped Gauss–Newton step for the `free` parameters of a station,
/// from a residual vector and its finite-difference Jacobian.
fn gauss_newton_step<const R: usize>(
    residual: [f64; R],
    jacobian: [[f64; 4]; R],
    free: [bool; 4],
) -> Option<[f64; 4]> {
    let mut matrix = [[0.0f64; 4]; 4];
    let mut rhs = [0.0f64; 4];
    for row in 0..4 {
        if !free[row] {
            // A frozen parameter solves to zero: unit diagonal, zero right side.
            matrix[row][row] = 1.0;
            continue;
        }
        for column in 0..4 {
            if !free[column] {
                continue;
            }
            matrix[row][column] = (0..R).map(|k| jacobian[k][row] * jacobian[k][column]).sum();
        }
        rhs[row] = -(0..R).map(|k| jacobian[k][row] * residual[k]).sum::<f64>();
    }
    let diagonal = (0..4)
        .filter(|&index| free[index])
        .fold(0.0f64, |largest, index| largest.max(matrix[index][index]));
    if diagonal <= 0.0 {
        return None;
    }
    for index in 0..4 {
        if free[index] {
            matrix[index][index] += LEVENBERG_DAMPING * diagonal;
        }
    }
    solve_small(matrix, rhs, 4).ok()
}

/// Finite-difference Jacobian of `residual_at` over the four parameters, each
/// stepped by `JACOBIAN_STEP` of its own span; frozen parameters get a zero
/// column.
fn jacobian_of<const R: usize>(
    station: Station,
    spans: [f64; 4],
    free: [bool; 4],
    fit: impl Fn(Station) -> Station,
    residual_at: impl Fn(Station) -> Option<[f64; R]>,
) -> Option<[[f64; 4]; R]> {
    let mut jacobian = [[0.0f64; 4]; R];
    for index in 0..4 {
        if !free[index] {
            continue;
        }
        let step = spans[index] * JACOBIAN_STEP;
        if step <= 0.0 {
            return None;
        }
        let plus = residual_at(fit(station.with_parameter(index, station.parameter(index) + step)))?;
        let minus = residual_at(fit(station.with_parameter(index, station.parameter(index) - step)))?;
        for row in 0..R {
            jacobian[row][index] = (plus[row] - minus[row]) / (2.0 * step);
        }
    }
    Some(jacobian)
}

/// Refine `seed` onto `{S_a = S_b, n_a × n_b = 0}`: Gauss–Newton on the
/// overdetermined 5×4 system, iterated to round-off because the caller reads a
/// RANK off the result and an extended contact stopped a few microns short of
/// the curve reads as a node (`csg/imprint/tangent_contact.rs` explains the
/// one-sided error). Returns the station and the cross-normal residual it
/// reached; `None` when the seed is not near any contact.
fn refine_contact(
    first: &NurbsSurface,
    second: &NurbsSurface,
    domain_a: Domain,
    domain_b: Domain,
    seed: Station,
    tolerance: f64,
) -> Result<Option<(Station, f64)>, String> {
    let spans = [domain_a.spans()[0], domain_a.spans()[1], domain_b.spans()[0], domain_b.spans()[1]];
    let fit = |station: Station| Station {
        ua: domain_a.fit(station.ua, true).0,
        va: domain_a.fit(station.va, false).0,
        ub: domain_b.fit(station.ub, true).0,
        vb: domain_b.fit(station.vb, false).0,
        point: station.point,
    };
    let free = [true; 4];
    let mut current = fit(seed);
    let mut best = current;
    let mut best_residual = f64::INFINITY;
    let mut best_tangency = f64::INFINITY;
    let mut without_progress = 0usize;
    for _ in 0..REFINE_ITERATIONS {
        let Some(frame) = frame_at(first, second, current) else {
            return Ok(None);
        };
        let residual_at = |station: Station| contact_residual(first, second, station, frame.e1, frame.e2);
        let Some(residual) = residual_at(current) else {
            return Ok(None);
        };
        let gap = (residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]).sqrt();
        let tangency = (residual[3] * residual[3] + residual[4] * residual[4]).sqrt();
        let worst = gap.max(tangency);
        if worst < best_residual * (1.0 - RESIDUAL_PROGRESS) {
            without_progress = 0;
        } else {
            without_progress += 1;
        }
        if worst < best_residual {
            best_residual = worst;
            best_tangency = tangency;
            best = current;
        }
        if worst == 0.0 || without_progress >= REFINE_PATIENCE {
            break;
        }
        let Some(jacobian) = jacobian_of::<5>(current, spans, free, fit, residual_at) else {
            return Ok(None);
        };
        let Some(changes) = gauss_newton_step(residual, jacobian, free) else {
            return Ok(None);
        };
        // Never more than a quarter of a span at once: near a tangency the
        // system is ill-conditioned by construction and an unbounded step
        // lands on an unrelated sheet.
        let mut scale = 1.0f64;
        for index in 0..4 {
            let limit = spans[index] / 4.0;
            if changes[index].abs() > limit {
                scale = scale.min(limit / changes[index].abs());
            }
        }
        let next = fit(Station {
            ua: current.ua + changes[0] * scale,
            va: current.va + changes[1] * scale,
            ub: current.ub + changes[2] * scale,
            vb: current.vb + changes[3] * scale,
            point: current.point,
        });
        let moved = (0..4)
            .map(|index| (next.parameter(index) - current.parameter(index)).abs() / spans[index])
            .fold(0.0f64, f64::max);
        current = next;
        if moved <= 1e-12 {
            break;
        }
    }
    if best_residual > tolerance {
        return Ok(None);
    }
    let point_a = first.evaluate(best.ua, best.va)?;
    let point_b = second.evaluate(best.ub, best.vb)?;
    best.point = point_a.add(point_b).scale(0.5);
    Ok(Some((best, best_tangency)))
}

/// The second fundamental form of `surface` at `(u, v)` in the orthonormal
/// tangent basis `(e1, e2)`, measured against `normal`.
fn second_fundamental_form(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    e1: Vec3,
    e2: Vec3,
    normal: Vec3,
) -> Option<[[f64; 2]; 2]> {
    let derivatives = surface.derivatives(u, v, 2).ok()?;
    let s_u = derivatives[1][0];
    let s_v = derivatives[0][1];
    let s_uu = derivatives[2][0];
    let s_uv = derivatives[1][1];
    let s_vv = derivatives[0][2];
    let a11 = s_u.dot(s_u);
    let a12 = s_u.dot(s_v);
    let a22 = s_v.dot(s_v);
    let determinant = a11 * a22 - a12 * a12;
    if determinant.abs() <= f64::MIN_POSITIVE {
        return None;
    }
    let coefficients = |target: Vec3| -> (f64, f64) {
        let b1 = s_u.dot(target);
        let b2 = s_v.dot(target);
        ((b1 * a22 - b2 * a12) / determinant, (a11 * b2 - a12 * b1) / determinant)
    };
    let along_1 = coefficients(e1);
    let along_2 = coefficients(e2);
    let l = s_uu.dot(normal);
    let m = s_uv.dot(normal);
    let n = s_vv.dot(normal);
    let form = |x: (f64, f64), y: (f64, f64)| l * x.0 * y.0 + m * (x.0 * y.1 + x.1 * y.0) + n * x.1 * y.1;
    Some([
        [form(along_1, along_1), form(along_1, along_2)],
        [form(along_2, along_1), form(along_2, along_2)],
    ])
}

fn eigenvalues(matrix: [[f64; 2]; 2]) -> (f64, f64) {
    let trace = matrix[0][0] + matrix[1][1];
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    let discriminant = (trace * trace / 4.0 - determinant).max(0.0).sqrt();
    (trace / 2.0 + discriminant, trace / 2.0 - discriminant)
}

fn spectral_norm(matrix: [[f64; 2]; 2]) -> f64 {
    let (first, second) = eigenvalues(matrix);
    first.abs().max(second.abs())
}

/// Unit eigenvector of a symmetric 2×2 matrix for the eigenvalue `lambda`.
fn eigenvector(matrix: [[f64; 2]; 2], lambda: f64) -> [f64; 2] {
    let candidates = [
        [matrix[0][1], lambda - matrix[0][0]],
        [lambda - matrix[1][1], matrix[0][1]],
    ];
    let mut best = [1.0, 0.0];
    let mut best_length = 0.0;
    for candidate in candidates {
        let length = (candidate[0] * candidate[0] + candidate[1] * candidate[1]).sqrt();
        if length > best_length {
            best_length = length;
            best = [candidate[0] / length, candidate[1] / length];
        }
    }
    if best_length == 0.0 {
        // The matrix is a multiple of the identity: every direction is an
        // eigenvector, and none is a null direction unless the multiple is 0.
        return [1.0, 0.0];
    }
    best
}

/// The local structure of the contact at a station: the eigenvalues of
/// `H = II_a − II_b`, the rank verdict and the null direction in 3D.
#[derive(Clone, Copy, Debug)]
struct HeightDifference {
    rank_ratio: f64,
    curvature_scale: f64,
    saddle: bool,
    /// The null eigenvector in 3D, meaningful only where `rank_ratio` is
    /// under [`EXTENDED_RANK_FLOOR`].
    null_direction: Vec3,
    /// `|II_a(d, d)|` and `|II_b(d, d)|` along the null direction — the
    /// normal curvatures the §4.12 ceiling reads.
    normal_curvature_along: f64,
}

fn height_difference(
    first: &NurbsSurface,
    second: &NurbsSurface,
    station: Station,
    frame: Frame,
) -> Option<HeightDifference> {
    // Both forms against the SAME normal, or their difference is the SUM of
    // the curvatures and every contact reads as definite.
    let orientation = if frame.normal_a.dot(frame.normal_b) < 0.0 { -1.0 } else { 1.0 };
    let form_a = second_fundamental_form(first, station.ua, station.va, frame.e1, frame.e2, frame.normal_a)?;
    let form_b = second_fundamental_form(
        second,
        station.ub,
        station.vb,
        frame.e1,
        frame.e2,
        frame.normal_b.scale(orientation),
    )?;
    let mut difference = [[0.0f64; 2]; 2];
    for row in 0..2 {
        for column in 0..2 {
            difference[row][column] = form_a[row][column] - form_b[row][column];
        }
    }
    let curvature_scale = spectral_norm(form_a).max(spectral_norm(form_b));
    if curvature_scale <= 0.0 {
        // Two planes: a 2-D coincidence, which the cosurface path owns.
        return None;
    }
    let (lambda_first, lambda_second) = eigenvalues(difference);
    let (smallest, largest) = if lambda_first.abs() <= lambda_second.abs() {
        (lambda_first, lambda_second)
    } else {
        (lambda_second, lambda_first)
    };
    let _ = largest;
    let [x, y] = eigenvector(difference, smallest);
    let null_direction = frame.e1.scale(x).add(frame.e2.scale(y)).normalized().ok()?;
    let along = |form: [[f64; 2]; 2]| form[0][0] * x * x + 2.0 * form[0][1] * x * y + form[1][1] * y * y;
    Some(HeightDifference {
        rank_ratio: smallest.abs() / curvature_scale,
        curvature_scale,
        saddle: lambda_first * lambda_second < 0.0,
        null_direction,
        normal_curvature_along: along(form_a).abs().max(along(form_b).abs()),
    })
}

/// A tangential contact between two carriers, refined onto the contact and
/// classified by the rank of the height-difference Hessian there.
#[derive(Clone, Copy, Debug)]
pub struct TangentContact {
    /// The refined contact point.
    pub point: Vec3,
    /// Its parameters on the first and second carrier.
    pub params_a: [f64; 2],
    pub params_b: [f64; 2],
    /// `min|λ| / κ` of `H = II_a − II_b`: under [`EXTENDED_RANK_FLOOR`] the
    /// contact extends along a curve.
    pub rank_ratio: f64,
    /// The local curvature scale `κ` the ratio is read against.
    pub curvature_scale: f64,
    /// `|n_a × n_b|` the refinement actually reached.
    pub tangency_residual: f64,
    /// The two eigenvalues have opposite signs: two transverse branches cross
    /// here (an isolated node), which the ordinary march owns.
    pub saddle: bool,
    /// The contact curve's unit tangent at `point`, present only for an
    /// extended contact.
    pub direction: Option<Vec3>,
}

impl TangentContact {
    /// The carriers touch along a CURVE through this point.
    pub fn is_extended(&self) -> bool {
        self.direction.is_some()
    }
}

fn classify_station(
    first: &NurbsSurface,
    second: &NurbsSurface,
    station: Station,
    tangency_residual: f64,
) -> Option<TangentContact> {
    let frame = frame_at(first, second, station)?;
    let difference = height_difference(first, second, station, frame)?;
    let extended = difference.rank_ratio < EXTENDED_RANK_FLOOR;
    Some(TangentContact {
        point: station.point,
        params_a: [station.ua, station.va],
        params_b: [station.ub, station.vb],
        rank_ratio: difference.rank_ratio,
        curvature_scale: difference.curvature_scale,
        tangency_residual,
        saddle: difference.saddle,
        direction: extended.then_some(difference.null_direction),
    })
}

/// Refine `witness` onto the tangential contact of the two carriers and
/// classify it. `None` when the witness is not near any contact (the
/// refinement leaves a residual above `tolerance`) or the carriers have no
/// curvature to classify by (two planes).
pub fn classify_tangent_contact_at(
    first: &NurbsSurface,
    second: &NurbsSurface,
    witness: Vec3,
    tolerance: f64,
) -> Result<Option<TangentContact>, String> {
    let domain_a = Domain::of(first)?;
    let domain_b = Domain::of(second)?;
    let on_a = project_point_to_surface(first, witness)?;
    let on_b = project_point_to_surface(second, witness)?;
    let seed = Station { ua: on_a.u, va: on_a.v, ub: on_b.u, vb: on_b.v, point: witness };
    let Some((station, tangency)) = refine_contact(first, second, domain_a, domain_b, seed, tolerance)? else {
        return Ok(None);
    };
    Ok(classify_station(first, second, station, tangency))
}

fn extent(surface: &NurbsSurface) -> Result<f64, String> {
    let mut minimum = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut maximum = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for point in surface.control_points.iter().flatten() {
        let point = point.point()?;
        minimum = Vec3::new(minimum.x.min(point.x), minimum.y.min(point.y), minimum.z.min(point.z));
        maximum = Vec3::new(maximum.x.max(point.x), maximum.y.max(point.y), maximum.z.max(point.z));
    }
    Ok(maximum.sub(minimum).length())
}

fn grid_count(surface: &NurbsSurface, direction_u: bool) -> usize {
    let (knots, degree) = if direction_u {
        (&surface.knots_u, surface.degree_u)
    } else {
        (&surface.knots_v, surface.degree_v)
    };
    (8.0_f64)
        .max(((interior_knot_count(knots, degree) + 1) * (degree + 1)) as f64)
        .ceil()
        .min(24.0) as usize
}

/// Find a tangential contact between two carriers without a witness: grid the
/// first carrier, project each sample onto the second, rank the samples by
/// their distance and the angle between the normals, and refine the best few
/// onto the contact. The grid is then repeated from the second carrier, so a
/// contact one carrier samples poorly is still reached from the other.
///
/// `None` means no sample refines onto a contact — the carriers do not touch
/// tangentially anywhere the grids see.
pub fn find_tangent_contact(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
) -> Result<Option<TangentContact>, String> {
    let diagonal = extent(first)?.min(extent(second)?);
    let gate = CANDIDATE_GATE_FRACTION * diagonal + tolerance * 100.0;
    for swapped in [false, true] {
        let (grid_surface, other) = if swapped { (second, first) } else { (first, second) };
        let domain_grid = Domain::of(grid_surface)?;
        let nu = grid_count(grid_surface, true);
        let nv = grid_count(grid_surface, false);
        let mut candidates: Vec<(f64, Station)> = Vec::new();
        for i in 0..=nu {
            for j in 0..=nv {
                let u = domain_grid.u0 + (domain_grid.u1 - domain_grid.u0) * i as f64 / nu as f64;
                let v = domain_grid.v0 + (domain_grid.v1 - domain_grid.v0) * j as f64 / nv as f64;
                let point = grid_surface.evaluate(u, v)?;
                let projection = project_point_to_surface(other, point)?;
                if projection.distance > gate {
                    continue;
                }
                let (Ok(normal_grid), Ok(normal_other)) =
                    (grid_surface.normal(u, v), other.normal(projection.u, projection.v))
                else {
                    continue;
                };
                let score = projection.distance / diagonal.max(EPSILON) + normal_grid.cross(normal_other).length();
                let station = if swapped {
                    Station { ua: projection.u, va: projection.v, ub: u, vb: v, point }
                } else {
                    Station { ua: u, va: v, ub: projection.u, vb: projection.v, point }
                };
                candidates.push((score, station));
            }
        }
        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));
        let domain_a = Domain::of(first)?;
        let domain_b = Domain::of(second)?;
        for (_, seed) in candidates.into_iter().take(CANDIDATE_ATTEMPTS) {
            if let Some((station, tangency)) = refine_contact(first, second, domain_a, domain_b, seed, tolerance)? {
                if let Some(contact) = classify_station(first, second, station, tangency) {
                    return Ok(Some(contact));
                }
            }
        }
    }
    Ok(None)
}

/// Correct the predicted station onto the contact curve inside the section
/// plane through `predicted` normal to `direction`. A parameter that clamps at
/// a non-closed domain end is frozen there and the plane constraint is
/// dropped — the boundary is then what pins the station along the curve — and
/// the flag reports that the trace has reached a boundary.
fn correct(
    first: &NurbsSurface,
    second: &NurbsSurface,
    domain_a: Domain,
    domain_b: Domain,
    from: Station,
    predicted: Vec3,
    direction: Vec3,
    tolerance: f64,
) -> Result<Option<(Station, bool)>, String> {
    let spans = [domain_a.spans()[0], domain_a.spans()[1], domain_b.spans()[0], domain_b.spans()[1]];
    let fit = |station: Station| Station {
        ua: domain_a.fit(station.ua, true).0,
        va: domain_a.fit(station.va, false).0,
        ub: domain_b.fit(station.ub, true).0,
        vb: domain_b.fit(station.vb, false).0,
        point: station.point,
    };
    let bar = tolerance * 0.01 + EPSILON;
    let mut current = from;
    let mut free = [true; 4];
    let mut boundary = false;
    for _ in 0..CORRECT_ITERATIONS {
        let Some(frame) = frame_at(first, second, current) else {
            return Ok(None);
        };
        let residual_at = |station: Station| -> Option<[f64; 6]> {
            let contact = contact_residual(first, second, station, frame.e1, frame.e2)?;
            let point_a = first.evaluate(station.ua, station.va).ok()?;
            let plane = if boundary { 0.0 } else { point_a.sub(predicted).dot(direction) };
            Some([contact[0], contact[1], contact[2], contact[3], contact[4], plane])
        };
        let Some(residual) = residual_at(current) else {
            return Ok(None);
        };
        let gap = (residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2]).sqrt();
        let tangency = (residual[3] * residual[3] + residual[4] * residual[4]).sqrt();
        if gap <= bar && tangency <= bar && residual[5].abs() <= bar {
            let point_a = first.evaluate(current.ua, current.va)?;
            let point_b = second.evaluate(current.ub, current.vb)?;
            current.point = point_a.add(point_b).scale(0.5);
            return Ok(Some((current, boundary)));
        }
        let Some(jacobian) = jacobian_of::<6>(current, spans, free, fit, residual_at) else {
            return Ok(None);
        };
        let Some(changes) = gauss_newton_step(residual, jacobian, free) else {
            return Ok(None);
        };
        let mut scale = 1.0f64;
        for index in 0..4 {
            let limit = spans[index] / 4.0;
            if changes[index].abs() > limit {
                scale = scale.min(limit / changes[index].abs());
            }
        }
        let mut next = current;
        let mut moved = 0.0f64;
        for index in 0..4 {
            if !free[index] {
                continue;
            }
            let raw = current.parameter(index) + changes[index] * scale;
            let (fitted, clamped) = match index {
                0 => domain_a.fit(raw, true),
                1 => domain_a.fit(raw, false),
                2 => domain_b.fit(raw, true),
                _ => domain_b.fit(raw, false),
            };
            if clamped {
                free[index] = false;
                boundary = true;
            }
            moved = moved.max((fitted - current.parameter(index)).abs() / spans[index]);
            next = next.with_parameter(index, fitted);
        }
        current = next;
        if !free.iter().any(|&f| f) {
            return Ok(None);
        }
        if moved <= 1e-14 {
            return Ok(None);
        }
    }
    Ok(None)
}

struct Trace {
    stations: Vec<Station>,
    tangents: Vec<Vec3>,
    closed: bool,
}

fn vector_angle(a: Vec3, b: Vec3) -> f64 {
    (a.dot(b) / (a.length() * b.length())).clamp(-1.0, 1.0).acos()
}

/// §4.12 ceiling along the contact direction: the advance that turns either
/// carrier's normal by [`NORMAL_TURN_BUDGET`]; `None` where neither carrier
/// curves along the direction.
fn normal_turn_ceiling(difference: &HeightDifference) -> Option<f64> {
    (difference.normal_curvature_along > 1e-12).then(|| NORMAL_TURN_BUDGET / difference.normal_curvature_along)
}

#[allow(clippy::too_many_arguments)]
fn trace(
    first: &NurbsSurface,
    second: &NurbsSurface,
    domain_a: Domain,
    domain_b: Domain,
    start: Station,
    start_direction: Vec3,
    tolerance: f64,
    initial_step: f64,
    minimum_step: f64,
    maximum_step: f64,
    maximum_steps: usize,
) -> Result<Trace, String> {
    let mut stations = vec![start];
    let mut tangents = vec![start_direction];
    let mut current = start;
    let mut direction = start_direction;
    let mut step_size = initial_step.min(maximum_step);
    if let Some(frame) = frame_at(first, second, start) {
        if let Some(difference) = height_difference(first, second, start, frame) {
            if let Some(ceiling) = normal_turn_ceiling(&difference) {
                step_size = step_size.min(ceiling).clamp(minimum_step, maximum_step);
            }
        }
    }
    let debug = std::env::var_os("BREP_DEBUG_SSI_TRACE").is_some();
    let mut terminated = false;
    for _ in 0..maximum_steps {
        let mut accepted = None;
        while step_size >= minimum_step {
            let predicted = current.point.add(direction.scale(step_size));
            if let Some((corrected, boundary)) =
                correct(first, second, domain_a, domain_b, current, predicted, direction, tolerance)?
            {
                if corrected.point.sub(current.point).length() <= 2.5 * step_size {
                    accepted = Some((corrected, boundary));
                    break;
                }
            }
            step_size /= 2.0;
        }
        let Some((candidate, boundary)) = accepted else {
            terminated = true;
            break;
        };
        let next = frame_at(first, second, candidate)
            .and_then(|frame| height_difference(first, second, candidate, frame));
        let next_direction = match next {
            Some(difference) if difference.rank_ratio < EXTENDED_RANK_FLOOR => {
                let null = difference.null_direction;
                Some((if null.dot(direction) < 0.0 { null.scale(-1.0) } else { null }, difference))
            }
            _ => None,
        };
        if debug {
            let at = candidate.point;
            eprintln!(
                "TANGENT-TRACE step {step_size:.4e} jump {:.4e} boundary {boundary} extended {} p ({:.6},{:.6},{:.6})",
                at.sub(current.point).length(),
                next_direction.is_some(),
                at.x,
                at.y,
                at.z
            );
        }
        let Some((next_direction, difference)) = next_direction else {
            stations.push(candidate);
            tangents.push(direction);
            terminated = true;
            break;
        };
        if boundary {
            stations.push(candidate);
            tangents.push(next_direction);
            terminated = true;
            break;
        }
        let angle = vector_angle(direction, next_direction);
        if angle > TURN_POP && step_size > minimum_step * 1.01 {
            step_size = minimum_step.max(step_size / 2.0);
            continue;
        }
        stations.push(candidate);
        tangents.push(next_direction);
        let reactive = if angle < TURN_GROW {
            step_size * 1.5
        } else if angle > TURN_SHRINK {
            step_size * 0.6
        } else {
            step_size
        };
        step_size = match normal_turn_ceiling(&difference) {
            Some(ceiling) => reactive.min(ceiling),
            None => reactive,
        }
        .clamp(minimum_step, maximum_step);
        if stations.len() > 4 && candidate.point.sub(start.point).length() <= step_size * 1.2 {
            let to_start = start.point.sub(candidate.point);
            if to_start.length() <= EPSILON || to_start.dot(next_direction) >= 0.0 {
                stations.push(start);
                tangents.push(tangents[0]);
                return Ok(Trace { stations, tangents, closed: true });
            }
        }
        current = candidate;
        direction = next_direction;
    }
    if !terminated {
        return Err(format!(
            "tangent-curve trace exhausted {maximum_steps} steps ({} stations, step {step_size:.3e}) \
             without reaching a boundary or closure",
            stations.len()
        ));
    }
    Ok(Trace { stations, tangents, closed: false })
}

/// March the tangent curve through an EXTENDED contact, both ways from the
/// contact point, and return it typed as a tangent section. `None` when the
/// contact is not extended (an isolated node or point has no curve to march).
///
/// Step sizes are the ordinary march's (`initial = extent / 100`,
/// `maximum = extent / 15` or the caller's ceiling, `minimum = max(100 tol,
/// 1e-6 extent)`), with the corrector run at `options.tolerance`.
pub fn trace_tangent_curve(
    first: &NurbsSurface,
    second: &NurbsSurface,
    contact: &TangentContact,
    options: &SurfaceIntersectionOptions,
) -> Result<Option<SurfaceIntersectionCurve>, String> {
    let Some(direction) = contact.direction else {
        return Ok(None);
    };
    let domain_a = Domain::of(first)?;
    let domain_b = Domain::of(second)?;
    let diagonal = extent(first)?.min(extent(second)?);
    let initial_step = diagonal / 100.0;
    let minimum_step = (options.tolerance * 100.0).max(diagonal * 1e-6);
    let maximum_step = (diagonal / 15.0).min(options.maximum_step.unwrap_or(f64::INFINITY));
    let needed = (MARCH_REACH_EXTENTS * diagonal / maximum_step).ceil();
    let maximum_steps = if needed.is_finite() && needed > options.maximum_steps as f64 {
        needed as usize
    } else {
        options.maximum_steps
    };
    let start = Station {
        ua: contact.params_a[0],
        va: contact.params_a[1],
        ub: contact.params_b[0],
        vb: contact.params_b[1],
        point: contact.point,
    };
    let forward = trace(
        first, second, domain_a, domain_b, start, direction, options.tolerance,
        initial_step, minimum_step, maximum_step, maximum_steps,
    )?;
    let (stations, tangents, closed) = if forward.closed {
        (forward.stations, forward.tangents, true)
    } else {
        let backward = trace(
            first, second, domain_a, domain_b, start, direction.scale(-1.0), options.tolerance,
            initial_step, minimum_step, maximum_step, maximum_steps,
        )?;
        let mut stations: Vec<Station> = backward.stations.into_iter().skip(1).rev().collect();
        let mut tangents: Vec<Vec3> = backward
            .tangents
            .into_iter()
            .skip(1)
            .rev()
            .map(|tangent| tangent.scale(-1.0))
            .collect();
        stations.extend(forward.stations);
        tangents.extend(forward.tangents);
        (stations, tangents, false)
    };
    if stations.len() < 2 {
        return Ok(None);
    }
    Ok(Some(SurfaceIntersectionCurve {
        points: stations.iter().map(|station| station.point).collect(),
        params_a: stations.iter().map(|station| [station.ua, station.va]).collect(),
        params_b: stations.iter().map(|station| [station.ub, station.vb]).collect(),
        closed,
        tangents: Some(tangents),
        span_deviation: None,
        tangent: true,
    }))
}

