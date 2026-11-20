//! Constrained deformation of one NURBS surface by its control net
//! (Golovanov 2020 §3.16, the deformable surface (3.16.3): a vector field of
//! deviations on a rectangular net added to a base surface).
//!
//! Here the deviation field is spanned by the base surface's OWN rational
//! basis: every free control point `P_ij` moves by `D_ij` with its weight held,
//! so the deformed surface `Σ R_ij (P_ij + D_ij)` is again an exact NURBS
//! surface on the same knots and weights, and the change at any parameter is
//! linear in the `D_ij`. The `D_ij` minimise a thin-plate bending energy of the
//! displacement (plus a small membrane and size term so the system is definite)
//! subject to hard linear constraints:
//!
//! - a pick `(u, v) -> target`: the surface passes through `target` at `(u, v)`;
//! - an optional pick normal `n`: both first partials at `(u, v)` are
//!   perpendicular to `n` (two linear rows, coupling x, y and z);
//! - held edges: the first row of control points on that side is fixed
//!   (position), or the first two (position and cross-boundary tangent).
//!
//! It is ONE constrained quadratic programme, solved exactly by the Schur
//! complement of the banded energy matrix — not a tessellation deformed and
//! refitted. Before the solve the surface may be degree-elevated and its spans
//! split (exact operations, `surface/refine.rs`) to give the net room.
//!
//! Every way the problem can be ill-posed is refused by name
//! ([`DeformRefusal`]): duplicate or dependent picks, picks the held rows leave
//! no freedom for, a net or pick count over the bounds, a constraint system too
//! ill-conditioned to trust, a displacement out of proportion to the surface,
//! invalid weights, a seam or collapsed row left free, and a result whose
//! normal turns over anywhere on a sampling grid (a fold).

use crate::surface::{KnotDirection, KnotRefinementRefusal};
use crate::{KnotVector, NurbsSurface, Vec3, Vec4};

/// Largest control net the solver accepts after elevation and refinement.
/// The banded factorisation costs `free · bandwidth²` with a bandwidth of about
/// `degree_u · columns`; at this bound a solve stays well under a second.
pub const MAX_DEFORM_POLES: usize = 4096;
/// Highest degree the solver elevates to.
pub const MAX_DEFORM_DEGREE: usize = 9;
/// Most picks one deformation takes.
pub const MAX_DEFORM_PICKS: usize = 64;
/// Largest accepted condition estimate of the constraint system (the squared
/// ratio of the largest to the smallest Cholesky pivot of its Schur complement).
pub const MAX_DEFORM_CONDITION: f64 = 1e12;
/// A constraint whose Schur pivot keeps less than this fraction of its own
/// diagonal is (numerically) implied by the constraints before it.
const DEPENDENCE_FRACTION: f64 = 1e-10;
/// Largest pick displacement, as a multiple of the control net's bounding-box
/// diagonal.
pub const MAX_DEFORM_DISPLACEMENT_RATIO: f64 = 1.0;
/// Thin-plate weight 1; the membrane and size terms that make the energy
/// definite even with no held edge (rigid motion then costs only these).
const MEMBRANE_WEIGHT: f64 = 1e-2;
const SIZE_WEIGHT: f64 = 1e-6;
/// Largest accepted ratio of the furthest control point move to the largest
/// change asked for (a pick's offset, or a normal pick's tilt times the net's
/// diagonal).
pub const MAX_DEFORM_AMPLIFICATION: f64 = 25.0;
/// Relative size below which a requested change is evaluation round-off.
const ROUND_OFF: f64 = 1e-14;
/// Samples per knot span (each direction) of the fold check.
const FOLD_SAMPLES_PER_SPAN: usize = 6;

/// One interpolation pick.
#[derive(Clone, Copy, Debug)]
pub struct DeformPick {
    pub u: f64,
    pub v: f64,
    /// Where the surface must pass at `(u, v)`.
    pub target: Vec3,
    /// When given, the surface normal at `(u, v)` must be parallel to it.
    pub normal: Option<Vec3>,
}

/// What a boundary side keeps.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum EdgeHold {
    /// The side's rows move freely.
    #[default]
    Free,
    /// The boundary row of control points is fixed: the edge curve is kept.
    Position,
    /// The first two rows are fixed: the edge curve and the cross-boundary
    /// tangent (so G1 with a neighbour across that edge) are kept.
    Tangent,
}

/// The four sides, in the order [`DeformSpec::hold`] lists them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeformSide {
    UMin,
    UMax,
    VMin,
    VMax,
}

const SIDES: [DeformSide; 4] = [DeformSide::UMin, DeformSide::UMax, DeformSide::VMin, DeformSide::VMax];

#[derive(Clone, Debug)]
pub struct DeformSpec {
    pub picks: Vec<DeformPick>,
    /// Held sides: `[u_min, u_max, v_min, v_max]`.
    pub hold: [EdgeHold; 4],
    /// Each direction is elevated (exactly) to at least this degree first;
    /// at least 2 (a bilinear net has no bending to minimise). Default 3.
    pub min_degree: usize,
    /// Each non-empty knot span is split into this many equal spans in u and
    /// v first (1 = no split). Default `[1, 1]`.
    pub split_spans: [usize; 2],
    /// When given, `[u0, u1, v0, v1]`: only knot spans that overlap this
    /// parameter box are split (a face's box on a larger carrier), so the
    /// pole bound is spent where the deformation is.
    pub split_window: Option<[f64; 4]>,
}

impl Default for DeformSpec {
    fn default() -> Self {
        Self { picks: Vec::new(), hold: [EdgeHold::Free; 4], min_degree: 3, split_spans: [1, 1], split_window: None }
    }
}

/// A deformed surface and what the solve measured.
#[derive(Clone, Debug)]
pub struct DeformResult {
    pub surface: NurbsSurface,
    /// Control rows and columns of the result.
    pub net: (usize, usize),
    /// Control points the solve could move.
    pub free_poles: usize,
    /// Largest pick distance of the result, read back through
    /// `NurbsSurface::evaluate` (not the solver's residual).
    pub max_pick_error: f64,
    /// Largest normal-pick angle (radians) of the result, read back through
    /// `NurbsSurface::deriv1`.
    pub max_normal_error: f64,
    /// Largest control point move.
    pub max_pole_shift: f64,
    /// Condition estimate of the constraint system.
    pub condition: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub enum DeformRefusal {
    InvalidSurface(String),
    /// A weight is not finite and positive.
    InvalidWeights { row: usize, column: usize },
    /// Held rows need clamped ends: an unclamped side's boundary curve is not
    /// its first row.
    UnclampedKnots { direction: KnotDirection },
    NoPicks,
    TooManyPicks { count: usize, limit: usize },
    NetTooLarge { poles: usize, limit: usize },
    /// `min_degree` below 2 or a split count of 0.
    InvalidSpec(String),
    /// A pick parameter outside the domain, or a non-finite target or normal,
    /// or a zero normal.
    InvalidPick { index: usize, reason: String },
    /// Two picks at the same parameters.
    DuplicatePicks { first: usize, second: usize },
    /// The pick asks for a change where every control point that reaches it
    /// is held.
    PickOnHeldRegion { index: usize },
    /// The pick's constraints are implied by (or contradict) earlier ones on
    /// this net: picks too close for the net to separate, or more than its
    /// free control points can meet. Split spans to give it room.
    DependentConstraints { index: usize },
    IllConditioned { estimate: f64, limit: f64 },
    DisplacementTooLarge { index: usize, displacement: f64, limit: f64 },
    /// A closed (seam) direction needs both seam sides held with their
    /// tangents, or the seam opens (free) or creases (position only).
    ClosedDirectionNotHeld { direction: KnotDirection },
    /// A side whose boundary row is collapsed to one point (a pole) must be
    /// held with its tangent, or the pole splits or kinks.
    CollapsedSideNotHeld { side: DeformSide },
    /// The result's normal turns over (or vanishes) at `(u, v)`.
    Folded { u: f64, v: f64 },
    /// The net had to move a control point `ratio` times further than the
    /// largest change asked for: the picks ask for more bending than the free
    /// region can give smoothly (a pick right next to a held edge, or picks
    /// pulling hard against each other). Split spans or move the pick.
    Amplified { ratio: f64, limit: f64 },
    /// Shell deformation: a tie between two surfaces is malformed (an index
    /// out of range, a surface tied to itself on one side).
    InvalidTie { tie: usize, reason: String },
    /// The two boundary rows a tie joins do not coincide after the exact
    /// elevation and knot merging that make them comparable.
    EdgeRowsDisagree { tie: usize, gap: f64 },
    /// A shared edge that is smooth (G1) in the base but whose rows cannot
    /// keep it so by the linear frozen-ratio rows (a ratio that varies along
    /// the edge, or rational rows): refused rather than creased.
    EdgeNotUniformlyTangent { tie: usize, reason: String },
    /// A tie's rows are implied by (or contradict) the other constraints.
    DependentTie { tie: usize },
    /// A tied system with more constraint rows than its bound.
    TooManyConstraints { rows: usize, limit: usize },
    /// Two surfaces meeting at a sharp shared edge have swung through each
    /// other there (the dihedral turned over) at edge parameter `t`.
    EdgeFoldedThrough { tie: usize, t: f64 },
    Refinement(KnotRefinementRefusal),
}

impl std::fmt::Display for DeformRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "deform surface: {self:?}")
    }
}

impl std::error::Error for DeformRefusal {}

/// Deform `surface` as `spec` asks. The input is never changed.
pub fn deform_surface(surface: &NurbsSurface, spec: &DeformSpec) -> Result<DeformResult, DeformRefusal> {
    deform_surface_where(surface, spec, &|_: PoleSupport| true)
}

/// One control point's support, as [`deform_surface_where`] shows it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoleSupport {
    /// The closed parameter box `[u0, u1, v0, v1]` (knots `t_i ..= t_(i+p+1)`
    /// each way) outside which the pole's basis function is zero.
    pub bounds: [f64; 4],
    /// The order to which the basis function vanishes at each side
    /// `[u0, u1, v0, v1]`: `p + 1 - m`, `m` the multiplicity of that end knot
    /// among the pole's own `p + 2` knots. Order 1: the value is zero there;
    /// order 2: the value and the first derivative across the side; and so on.
    /// Moving the pole changes nothing on that side below that order.
    pub vanishing: [usize; 4],
}

/// [`deform_surface`] where, besides the held sides, a control point may move
/// only when `may_move` accepts its support. A point of the surface depends
/// only on the poles whose supports contain it (and on a support's side only
/// to the side's vanishing order), so holding every pole whose support meets a
/// region keeps the surface identical there to every order, and admitting one
/// that only touches the region on a side of vanishing order 2 keeps the
/// surface's points and tangent planes there.
pub fn deform_surface_where(
    surface: &NurbsSurface,
    spec: &DeformSpec,
    may_move: &dyn Fn(PoleSupport) -> bool,
) -> Result<DeformResult, DeformRefusal> {
    surface.domain_u().map_err(DeformRefusal::InvalidSurface)?;
    for (row, line) in surface.control_points.iter().enumerate() {
        for (column, pole) in line.iter().enumerate() {
            if !(pole.w.is_finite() && pole.w > 0.0) || ![pole.x, pole.y, pole.z].iter().all(|c| c.is_finite()) {
                return Err(DeformRefusal::InvalidWeights { row, column });
            }
        }
    }
    if spec.min_degree < 2 || spec.min_degree > MAX_DEFORM_DEGREE || spec.split_spans.contains(&0) {
        return Err(DeformRefusal::InvalidSpec(format!(
            "min_degree {} (2 to {MAX_DEFORM_DEGREE}) and split_spans {:?} (at least 1)",
            spec.min_degree, spec.split_spans
        )));
    }
    if spec.picks.is_empty() {
        return Err(DeformRefusal::NoPicks);
    }
    if spec.picks.len() > MAX_DEFORM_PICKS {
        return Err(DeformRefusal::TooManyPicks { count: spec.picks.len(), limit: MAX_DEFORM_PICKS });
    }
    for direction in [KnotDirection::U, KnotDirection::V] {
        let (degree, knots) = knots_of(surface, direction);
        let n = knots.len();
        if knots[..=degree].iter().any(|&k| k != knots[degree])
            || knots[n - 1 - degree..].iter().any(|&k| k != knots[n - 1])
        {
            return Err(DeformRefusal::UnclampedKnots { direction });
        }
    }
    let [u0, u1] = surface.domain_u().map_err(DeformRefusal::InvalidSurface)?;
    let [v0, v1] = surface.domain_v().map_err(DeformRefusal::InvalidSurface)?;
    for (index, pick) in spec.picks.iter().enumerate() {
        let finite = [pick.u, pick.v, pick.target.x, pick.target.y, pick.target.z].iter().all(|c| c.is_finite());
        if !finite {
            return Err(DeformRefusal::InvalidPick { index, reason: "non-finite parameter or target".into() });
        }
        if pick.u < u0 || pick.u > u1 || pick.v < v0 || pick.v > v1 {
            return Err(DeformRefusal::InvalidPick {
                index,
                reason: format!("({}, {}) outside the domain [{u0}, {u1}] x [{v0}, {v1}]", pick.u, pick.v),
            });
        }
        if let Some(normal) = pick.normal {
            let length = normal.length();
            if !length.is_finite() || length <= f64::MIN_POSITIVE.sqrt() {
                return Err(DeformRefusal::InvalidPick { index, reason: "zero or non-finite normal".into() });
            }
        }
    }
    let (du, dv) = ((u1 - u0).max(f64::MIN_POSITIVE), (v1 - v0).max(f64::MIN_POSITIVE));
    for first in 0..spec.picks.len() {
        for second in first + 1..spec.picks.len() {
            let (a, b) = (spec.picks[first], spec.picks[second]);
            if ((a.u - b.u) / du).abs() <= 1e-9 && ((a.v - b.v) / dv).abs() <= 1e-9 {
                return Err(DeformRefusal::DuplicatePicks { first, second });
            }
        }
    }
    let base = prepare_base(surface, spec)?;
    let rows = base.control_points.len();
    let columns = base.control_points[0].len();

    // Free poles: everything outside the held rows.
    let depth = |hold: EdgeHold| match hold {
        EdgeHold::Free => 0,
        EdgeHold::Position => 1,
        EdgeHold::Tangent => 2,
    };
    let (i_lo, i_hi) = (depth(spec.hold[0]), rows.saturating_sub(depth(spec.hold[1])));
    let (j_lo, j_hi) = (depth(spec.hold[2]), columns.saturating_sub(depth(spec.hold[3])));
    let mut free_index = vec![vec![usize::MAX; columns]; rows];
    let mut free = Vec::new();
    let (pu, pv) = (base.degree_u, base.degree_v);
    for i in i_lo..i_hi.max(i_lo) {
        for j in j_lo..j_hi.max(j_lo) {
            let (ku, kv) = (&base.knots_u[i..=i + pu + 1], &base.knots_v[j..=j + pv + 1]);
            let order = |local: &[f64], end: f64, degree: usize| (degree + 1).saturating_sub(local.iter().filter(|&&k| k == end).count());
            let support = PoleSupport {
                bounds: [ku[0], ku[pu + 1], kv[0], kv[pv + 1]],
                vanishing: [order(ku, ku[0], pu), order(ku, ku[pu + 1], pu), order(kv, kv[0], pv), order(kv, kv[pv + 1], pv)],
            };
            if may_move(support) {
                free_index[i][j] = free.len();
                free.push((i, j));
            }
        }
    }
    // Read off the free set (held sides and `may_move` together): a seam's
    // two rows each side, and a collapsed side's two rows, must all be held,
    // or the seam opens or creases and the pole splits or kinks.
    let row_free = |i: usize| (0..columns).any(|j| free_index[i][j] != usize::MAX);
    let column_free = |j: usize| (0..rows).any(|i| free_index[i][j] != usize::MAX);
    let (closed_u, closed_v) = base.closed_directions().map_err(DeformRefusal::InvalidSurface)?;
    if closed_u && [0, 1, rows - 2, rows - 1].into_iter().any(row_free) {
        return Err(DeformRefusal::ClosedDirectionNotHeld { direction: KnotDirection::U });
    }
    if closed_v && [0, 1, columns - 2, columns - 1].into_iter().any(column_free) {
        return Err(DeformRefusal::ClosedDirectionNotHeld { direction: KnotDirection::V });
    }
    for side in SIDES {
        let near = match side {
            DeformSide::UMin => row_free(0) || row_free(1),
            DeformSide::UMax => row_free(rows - 1) || row_free(rows - 2),
            DeformSide::VMin => column_free(0) || column_free(1),
            DeformSide::VMax => column_free(columns - 1) || column_free(columns - 2),
        };
        if near && side_collapsed(&base, side) {
            return Err(DeformRefusal::CollapsedSideNotHeld { side });
        }
    }

    // Constraints, and the displacement bound.
    let diagonal = net_diagonal(&base);
    let displacement_limit = MAX_DEFORM_DISPLACEMENT_RATIO * diagonal;
    let mut constraints: Vec<Constraint> = Vec::new();
    let mut requested = 0.0f64;
    pick_constraints(&base, &spec.picks, &free_index, 0, 0, displacement_limit, &mut constraints, &mut requested)?;

    // Nothing asks for a change: the (elevated, split) base comes back as it is.
    let (displacements, condition) = if constraints.iter().all(|c| c.rhs == 0.0) {
        (vec![0.0; 3 * free.len()], 1.0)
    } else {
        let energy = BandedEnergy::build(&base, &free, &free_index)?;
        solve(&energy, &constraints)?
    };

    // The deformed net: weights held, Cartesian points moved.
    let mut net = base.control_points.clone();
    let mut max_pole_shift = 0.0f64;
    for (f, &(i, j)) in free.iter().enumerate() {
        let d = Vec3::new(displacements[3 * f], displacements[3 * f + 1], displacements[3 * f + 2]);
        if d.x == 0.0 && d.y == 0.0 && d.z == 0.0 {
            continue;
        }
        let pole = net[i][j];
        net[i][j] = Vec4 { x: pole.x + d.x * pole.w, y: pole.y + d.y * pole.w, z: pole.z + d.z * pole.w, w: pole.w };
        max_pole_shift = max_pole_shift.max(d.length());
    }
    let deformed = NurbsSurface::new(base.degree_u, base.degree_v, base.knots_u.clone(), base.knots_v.clone(), net)
        .map_err(DeformRefusal::InvalidSurface)?;

    if max_pole_shift > MAX_DEFORM_AMPLIFICATION * requested {
        return Err(DeformRefusal::Amplified {
            ratio: max_pole_shift / requested.max(f64::MIN_POSITIVE),
            limit: MAX_DEFORM_AMPLIFICATION,
        });
    }
    check_fold(&base, &deformed)?;
    let mut max_pick_error = 0.0f64;
    let mut max_normal_error = 0.0f64;
    for pick in &spec.picks {
        let point = deformed.evaluate(pick.u, pick.v).map_err(DeformRefusal::InvalidSurface)?;
        max_pick_error = max_pick_error.max(point.sub(pick.target).length());
        if let Some(normal) = pick.normal {
            let (_, su, sv) = deformed.deriv1(pick.u, pick.v).map_err(DeformRefusal::InvalidSurface)?;
            let n = su.cross(sv);
            let sine = n.cross(normal).length() / (n.length() * normal.length()).max(f64::MIN_POSITIVE);
            max_normal_error = max_normal_error.max(sine.min(1.0).asin());
        }
    }
    Ok(DeformResult {
        surface: deformed,
        net: (rows, columns),
        free_poles: free.len(),
        max_pick_error,
        max_normal_error,
        max_pole_shift,
        condition,
    })
}

/// Check a surface and the spec's room parameters, then elevate and split it
/// (both exact) as the spec asks, refusing a net over the pole bound before
/// building it. The result is the deformation's base: the same surface on a
/// finer net.
pub(crate) fn prepare_base(surface: &NurbsSurface, spec: &DeformSpec) -> Result<NurbsSurface, DeformRefusal> {
    surface.domain_u().map_err(DeformRefusal::InvalidSurface)?;
    for (row, line) in surface.control_points.iter().enumerate() {
        for (column, pole) in line.iter().enumerate() {
            if !(pole.w.is_finite() && pole.w > 0.0) || ![pole.x, pole.y, pole.z].iter().all(|c| c.is_finite()) {
                return Err(DeformRefusal::InvalidWeights { row, column });
            }
        }
    }
    if spec.min_degree < 2 || spec.min_degree > MAX_DEFORM_DEGREE || spec.split_spans.contains(&0) {
        return Err(DeformRefusal::InvalidSpec(format!(
            "min_degree {} (2 to {MAX_DEFORM_DEGREE}) and split_spans {:?} (at least 1)",
            spec.min_degree, spec.split_spans
        )));
    }
    for direction in [KnotDirection::U, KnotDirection::V] {
        let (degree, knots) = knots_of(surface, direction);
        let n = knots.len();
        if knots[..=degree].iter().any(|&k| k != knots[degree])
            || knots[n - 1 - degree..].iter().any(|&k| k != knots[n - 1])
        {
            return Err(DeformRefusal::UnclampedKnots { direction });
        }
    }
    // The net size elevation and splitting will give, predicted (with checked
    // arithmetic) before anything is allocated: elevating a direction to
    // degree q adds (q - p) poles per non-empty span, splitting each
    // span into k adds (k - 1) per span.
    let mut predicted = 1usize;
    for (axis, direction) in [KnotDirection::U, KnotDirection::V].into_iter().enumerate() {
        let (degree, knots) = knots_of(surface, direction);
        let spans = knots.windows(2).filter(|pair| pair[1] > pair[0]).count();
        let split = knots.windows(2).filter(|pair| pair[1] > pair[0] && in_window(spec, axis, pair)).count();
        let count = knots.len() - degree - 1;
        let raise = spec.min_degree.saturating_sub(degree);
        let lines = raise
            .checked_mul(spans)
            .and_then(|extra| extra.checked_add(count))
            .and_then(|lines| (spec.split_spans[axis] - 1).checked_mul(split).and_then(|extra| lines.checked_add(extra)));
        predicted = match lines.and_then(|lines| predicted.checked_mul(lines)) {
            Some(value) if value <= MAX_DEFORM_POLES => value,
            _ => return Err(DeformRefusal::NetTooLarge { poles: usize::MAX, limit: MAX_DEFORM_POLES }),
        };
    }

    // Room: elevate, then split spans (both exact).
    let mut base = surface.clone();
    for direction in [KnotDirection::U, KnotDirection::V] {
        let (degree, _) = knots_of(&base, direction);
        if degree < spec.min_degree {
            base = base.elevate_degree(direction, spec.min_degree).map_err(DeformRefusal::InvalidSurface)?;
        }
        let pieces = spec.split_spans[match direction {
            KnotDirection::U => 0,
            KnotDirection::V => 1,
        }];
        if pieces > 1 {
            let axis = match direction {
                KnotDirection::U => 0,
                KnotDirection::V => 1,
            };
            let (_, knots) = knots_of(&base, direction);
            let mut inserts = Vec::new();
            for pair in knots.windows(2) {
                if pair[1] > pair[0] && in_window(spec, axis, pair) {
                    for k in 1..pieces {
                        inserts.push(pair[0] + (pair[1] - pair[0]) * k as f64 / pieces as f64);
                    }
                }
            }
            base = base.refine_knots(direction, &inserts).map_err(DeformRefusal::Refinement)?;
        }
    }
    let (rows, columns) = (base.control_points.len(), base.control_points[0].len());
    if rows * columns > MAX_DEFORM_POLES {
        return Err(DeformRefusal::NetTooLarge { poles: rows * columns, limit: MAX_DEFORM_POLES });
    }
    Ok(base)
}

/// The constraint rows of one surface's picks, its unknowns at `offset`
/// (control points) in the stacked vector; refusals name picks from
/// `first_pick` on. `requested` gathers the largest change asked for.
#[allow(clippy::too_many_arguments)]
fn pick_constraints(
    base: &NurbsSurface,
    picks: &[DeformPick],
    free_index: &[Vec<usize>],
    offset: usize,
    first_pick: usize,
    displacement_limit: f64,
    constraints: &mut Vec<Constraint>,
    requested: &mut f64,
) -> Result<(), DeformRefusal> {
    let diagonal = net_diagonal(base);
    for (local_index, pick) in picks.iter().enumerate() {
        let index = first_pick + local_index;
        let local = RationalBasis::at(base, pick.u, pick.v).map_err(DeformRefusal::InvalidSurface)?;
        let delta = pick.target.sub(local.point);
        let displacement = delta.length();
        if !(displacement <= displacement_limit) {
            return Err(DeformRefusal::DisplacementTooLarge { index, displacement, limit: displacement_limit });
        }
        *requested = requested.max(displacement);
        if let Some(normal) = pick.normal {
            let current = local.du.cross(local.dv);
            let sine = current.cross(normal).length() / (current.length() * normal.length()).max(f64::MIN_POSITIVE);
            *requested = requested.max(sine.min(1.0).asin() * diagonal);
        }
        // Σ R = 1: the whole reach of a position row is 1. The round-off of
        // one coordinate of a point read back from the surface is relative
        // to that coordinate of the poles that make it (Σ R_k P_k[c], R >= 0),
        // not to the point's distance from the origin.
        for axis in 0..3 {
            let terms: Vec<(usize, f64)> =
                local.terms(free_index, |r, _, _| r, axis).into_iter().map(|(i, c)| (i + 3 * offset, c)).collect();
            let scale = local.cartesian.iter().map(|p| component(*p, axis).abs()).fold(0.0, f64::max);
            push_constraint(constraints, index, terms, component(delta, axis), scale, 1.0)?;
        }
        if let Some(normal) = pick.normal {
            let normal = normal.normalized().map_err(DeformRefusal::InvalidSurface)?;
            for partial in [Partial::U, Partial::V] {
                let mut terms = Vec::new();
                let mut rhs = 0.0;
                let mut whole_reach = 0.0f64;
                let mut round_off_scale = 0.0f64;
                for (k, (ru, rv)) in local.partials.iter().enumerate() {
                    let r = match partial {
                        Partial::U => *ru,
                        Partial::V => *rv,
                    };
                    whole_reach = whole_reach.max(r.abs());
                    round_off_scale += (r * local.cartesian[k].dot(normal)).abs();
                    let (i, j) = local.poles[k];
                    rhs -= r * local.cartesian[k].dot(normal);
                    let f = free_index[i][j];
                    if f != usize::MAX && r != 0.0 {
                        for axis in 0..3 {
                            terms.push((3 * (offset + f) + axis, r * component(normal, axis)));
                        }
                    }
                }
                push_constraint(constraints, index, terms, rhs, round_off_scale, whole_reach)?;
            }
        }
    }
    Ok(())
}

/// Does the knot span `pair` overlap the split window in `axis` (0 = u)?
fn in_window(spec: &DeformSpec, axis: usize, pair: &[f64]) -> bool {
    match spec.split_window {
        None => true,
        Some(window) => {
            let (lo, hi) = (window[2 * axis], window[2 * axis + 1]);
            pair[1] > lo && pair[0] < hi
        }
    }
}

fn knots_of(surface: &NurbsSurface, direction: KnotDirection) -> (usize, &Vec<f64>) {
    match direction {
        KnotDirection::U => (surface.degree_u, &surface.knots_u),
        KnotDirection::V => (surface.degree_v, &surface.knots_v),
    }
}

fn component(v: Vec3, axis: usize) -> f64 {
    match axis {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

fn cartesian(pole: Vec4) -> Vec3 {
    Vec3::new(pole.x / pole.w, pole.y / pole.w, pole.z / pole.w)
}

fn side_collapsed(surface: &NurbsSurface, side: DeformSide) -> bool {
    let rows = surface.control_points.len();
    let columns = surface.control_points[0].len();
    let line: Vec<Vec3> = match side {
        DeformSide::UMin => surface.control_points[0].iter().copied().map(cartesian).collect(),
        DeformSide::UMax => surface.control_points[rows - 1].iter().copied().map(cartesian).collect(),
        DeformSide::VMin => surface.control_points.iter().map(|row| cartesian(row[0])).collect(),
        DeformSide::VMax => surface.control_points.iter().map(|row| cartesian(row[columns - 1])).collect(),
    };
    let scale = net_diagonal(surface).max(f64::MIN_POSITIVE);
    line.iter().all(|p| p.sub(line[0]).length() <= 1e-12 * scale)
}

fn net_diagonal(surface: &NurbsSurface) -> f64 {
    let mut lo = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for pole in surface.control_points.iter().flatten() {
        let p = cartesian(*pole);
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
    }
    hi.sub(lo).length()
}

#[derive(Clone, Copy)]
enum Partial {
    U,
    V,
}

/// The rational basis `R_ij` and its first partials at one parameter pair,
/// over the `(degree_u + 1)(degree_v + 1)` poles that reach it, and the
/// surface point and partials they give.
struct RationalBasis {
    poles: Vec<(usize, usize)>,
    values: Vec<f64>,
    partials: Vec<(f64, f64)>,
    cartesian: Vec<Vec3>,
    point: Vec3,
    du: Vec3,
    dv: Vec3,
}

impl RationalBasis {
    fn at(surface: &NurbsSurface, u: f64, v: f64) -> Result<Self, String> {
        let ku = KnotVector::new(surface.knots_u.clone(), surface.degree_u)?;
        let kv = KnotVector::new(surface.knots_v.clone(), surface.degree_v)?;
        let (u, v) = (ku.clamp_param(u), kv.clamp_param(v));
        let (su, sv) = (ku.find_span(u), kv.find_span(v));
        let nu = ku.basis_derivatives(su, u, 1);
        let nv = kv.basis_derivatives(sv, v, 1);
        let (pu, pv) = (surface.degree_u, surface.degree_v);
        let (mut w, mut wu, mut wv) = (0.0, 0.0, 0.0);
        for a in 0..=pu {
            for b in 0..=pv {
                let weight = surface.control_points[su - pu + a][sv - pv + b].w;
                w += nu[0][a] * nv[0][b] * weight;
                wu += nu[1][a] * nv[0][b] * weight;
                wv += nu[0][a] * nv[1][b] * weight;
            }
        }
        if !(w > 0.0) {
            return Err(format!("deform: rational denominator {w} at ({u}, {v})"));
        }
        let mut basis = Self {
            poles: Vec::new(),
            values: Vec::new(),
            partials: Vec::new(),
            cartesian: Vec::new(),
            point: Vec3::default(),
            du: Vec3::default(),
            dv: Vec3::default(),
        };
        for a in 0..=pu {
            for b in 0..=pv {
                let (i, j) = (su - pu + a, sv - pv + b);
                let pole = surface.control_points[i][j];
                let n = nu[0][a] * nv[0][b] * pole.w;
                let r = n / w;
                let ru = (nu[1][a] * nv[0][b] * pole.w * w - n * wu) / (w * w);
                let rv = (nu[0][a] * nv[1][b] * pole.w * w - n * wv) / (w * w);
                let p = cartesian(pole);
                basis.point = basis.point.add(p.scale(r));
                basis.du = basis.du.add(p.scale(ru));
                basis.dv = basis.dv.add(p.scale(rv));
                basis.poles.push((i, j));
                basis.values.push(r);
                basis.partials.push((ru, rv));
                basis.cartesian.push(p);
            }
        }
        Ok(basis)
    }

    /// The sparse row `Σ coefficient(R, R_u, R_v) D_ij[axis]` over the free poles.
    fn terms(
        &self,
        free_index: &[Vec<usize>],
        coefficient: impl Fn(f64, f64, f64) -> f64,
        axis: usize,
    ) -> Vec<(usize, f64)> {
        let mut out = Vec::new();
        for (k, &(i, j)) in self.poles.iter().enumerate() {
            let f = free_index[i][j];
            let c = coefficient(self.values[k], self.partials[k].0, self.partials[k].1);
            if f != usize::MAX && c != 0.0 {
                out.push((3 * f + axis, c));
            }
        }
        out
    }
}

/// One linear constraint `Σ coefficient · x[index] = rhs` on the stacked
/// displacement vector `x = (D_0.x, D_0.y, D_0.z, D_1.x, ...)`.
struct Constraint {
    pick: usize,
    /// A tie row's tie (shell deformation), named by a dependence refusal.
    tie: Option<usize>,
    terms: Vec<(usize, f64)>,
    rhs: f64,
}

/// Add a constraint, or drop it when no free pole reaches it and it already
/// holds (a pick on a held edge at its current place); refuse it when no free
/// pole reaches it and it asks for a change. "Reaches" is relative to
/// `whole_reach`, the row's largest coefficient over every pole held or free,
/// so a parameterisation's scale (a derivative row's coefficients go as
/// 1 / domain length) does not decide it. The kept row is scaled to unit
/// length, so the Schur complement's conditioning is the geometry's, not the
/// mix of position and derivative units.
fn push_constraint(
    constraints: &mut Vec<Constraint>,
    pick: usize,
    terms: Vec<(usize, f64)>,
    rhs: f64,
    scale: f64,
    whole_reach: f64,
) -> Result<(), DeformRefusal> {
    // A change within round-off of the evaluation (a pick read back from the
    // surface itself) is no change: the input comes back exactly.
    let rhs = if rhs.abs() <= ROUND_OFF * scale { 0.0 } else { rhs };
    let reach = terms.iter().map(|(_, c)| c.abs()).fold(0.0, f64::max);
    if !(reach > 1e-12 * whole_reach) {
        if rhs == 0.0 {
            return Ok(());
        }
        return Err(DeformRefusal::PickOnHeldRegion { index: pick });
    }
    let norm = terms.iter().map(|(_, c)| c * c).sum::<f64>().sqrt();
    let terms = terms.into_iter().map(|(index, c)| (index, c / norm)).collect();
    constraints.push(Constraint { pick, tie: None, terms, rhs: rhs / norm });
    Ok(())
}

/// The thin-plate energy of the displacement on the free poles, one banded
/// symmetric matrix shared by x, y and z:
/// `H = G2u⊗G0v + 2 G1u⊗G1v + G0u⊗G2v + μ (G1u⊗G0v + G0u⊗G1v) + ε G0u⊗G0v`,
/// `Gk` the Gram matrix of the k-th derivatives of the (polynomial) B-spline
/// basis over the domain mapped to [0, 1]. Stored as its lower band.
struct BandedEnergy {
    n: usize,
    bandwidth: usize,
    /// `factor[k][d]` = L[k][k - d] after `build` (Cholesky factor).
    factor: Vec<Vec<f64>>,
}

impl BandedEnergy {
    fn build(surface: &NurbsSurface, free: &[(usize, usize)], free_index: &[Vec<usize>]) -> Result<Self, DeformRefusal> {
        let gu = gram(&surface.knots_u, surface.degree_u);
        let gv = gram(&surface.knots_v, surface.degree_v);
        let (pu, pv) = (surface.degree_u, surface.degree_v);
        let rows = surface.control_points.len();
        let columns = surface.control_points[0].len();
        let n = free.len();
        let mut bandwidth = 0;
        for (k, &(i, j)) in free.iter().enumerate() {
            for i2 in i.saturating_sub(pu)..=(i + pu).min(rows - 1) {
                for j2 in j.saturating_sub(pv)..=(j + pv).min(columns - 1) {
                    let f = free_index[i2][j2];
                    if f != usize::MAX && f <= k {
                        bandwidth = bandwidth.max(k - f);
                    }
                }
            }
        }
        let mut band = vec![vec![0.0; bandwidth + 1]; n];
        for (k, &(i, j)) in free.iter().enumerate() {
            for i2 in i.saturating_sub(pu)..=(i + pu).min(rows - 1) {
                for j2 in j.saturating_sub(pv)..=(j + pv).min(columns - 1) {
                    let f = free_index[i2][j2];
                    if f == usize::MAX || f > k {
                        continue;
                    }
                    let (a, b) = (gu.at(i, i2), gv.at(j, j2));
                    let value = a[2] * b[0]
                        + 2.0 * a[1] * b[1]
                        + a[0] * b[2]
                        + MEMBRANE_WEIGHT * (a[1] * b[0] + a[0] * b[1])
                        + SIZE_WEIGHT * a[0] * b[0];
                    band[k][k - f] = value;
                }
            }
        }
        // Banded Cholesky in place.
        for i in 0..n {
            let lo = i.saturating_sub(bandwidth);
            for j in lo..=i {
                let mut sum = band[i][i - j];
                let t_lo = lo.max(j.saturating_sub(bandwidth));
                for t in t_lo..j {
                    sum -= band[i][i - t] * band[j][j - t];
                }
                if i == j {
                    if !(sum > 0.0) {
                        return Err(DeformRefusal::IllConditioned { estimate: f64::INFINITY, limit: MAX_DEFORM_CONDITION });
                    }
                    band[i][0] = sum.sqrt();
                } else {
                    band[i][i - j] = sum / band[j][0];
                }
            }
        }
        Ok(Self { n, bandwidth, factor: band })
    }

    /// Solve `H y = b` in place.
    fn solve(&self, b: &mut [f64]) {
        let (n, w, l) = (self.n, self.bandwidth, &self.factor);
        for i in 0..n {
            let mut sum = b[i];
            for t in i.saturating_sub(w)..i {
                sum -= l[i][i - t] * b[t];
            }
            b[i] = sum / l[i][0];
        }
        for i in (0..n).rev() {
            let mut sum = b[i];
            for t in i + 1..(i + w + 1).min(n) {
                sum -= l[t][t - i] * b[t];
            }
            b[i] = sum / l[i][0];
        }
    }
}

/// Minimise `½ xᵀ (I₃ ⊗ H) x` subject to `A x = b`: `x = H⁻¹ Aᵀ λ`,
/// `(A H⁻¹ Aᵀ) λ = b`, the Schur complement factored by Cholesky with each
/// pivot checked for dependence and the pivots' spread for conditioning.
fn solve(energy: &BandedEnergy, constraints: &[Constraint]) -> Result<(Vec<f64>, f64), DeformRefusal> {
    solve_blocks(&[(energy, 0)], energy.n, constraints)
}

/// [`solve`] over several surfaces at once: `blocks` are their energies and
/// the offset (in control points) of each one's unknowns in the stacked
/// vector of `total` points. The energy is block diagonal (each surface's own
/// bending), so `H⁻¹` is one banded solve per block; the constraint rows may
/// couple blocks (ties along shared edges).
fn solve_blocks(
    blocks: &[(&BandedEnergy, usize)],
    total: usize,
    constraints: &[Constraint],
) -> Result<(Vec<f64>, f64), DeformRefusal> {
    let n = total;
    let m = constraints.len();
    // Y_r = (I₃ ⊗ H)⁻¹ a_r, one banded solve per block and axis the row touches.
    let mut columns: Vec<Vec<f64>> = Vec::with_capacity(m);
    for constraint in constraints {
        let mut y = vec![0.0; 3 * n];
        for &(energy, offset) in blocks {
            for axis in 0..3 {
                let mut rhs = vec![0.0; energy.n];
                let mut any = false;
                for &(index, c) in &constraint.terms {
                    let point = index / 3;
                    if index % 3 == axis && point >= offset && point < offset + energy.n {
                        rhs[point - offset] += c;
                        any = true;
                    }
                }
                if any {
                    energy.solve(&mut rhs);
                    for (f, value) in rhs.into_iter().enumerate() {
                        y[3 * (offset + f) + axis] = value;
                    }
                }
            }
        }
        columns.push(y);
    }
    let mut schur = vec![vec![0.0; m]; m];
    for r in 0..m {
        for s in 0..=r {
            let value: f64 = constraints[s].terms.iter().map(|&(index, c)| c * columns[r][index]).sum();
            schur[r][s] = value;
            schur[s][r] = value;
        }
    }
    // Cholesky with a dependence test on each pivot.
    let diagonal: Vec<f64> = (0..m).map(|r| schur[r][r]).collect();
    let mut factor = vec![vec![0.0; m]; m];
    let (mut largest, mut smallest) = (0.0f64, f64::INFINITY);
    for i in 0..m {
        for j in 0..=i {
            let mut sum = schur[i][j];
            for t in 0..j {
                sum -= factor[i][t] * factor[j][t];
            }
            if i == j {
                if !(sum > DEPENDENCE_FRACTION * diagonal[i]) {
                    return Err(match constraints[i].tie {
                        Some(tie) => DeformRefusal::DependentTie { tie },
                        None => DeformRefusal::DependentConstraints { index: constraints[i].pick },
                    });
                }
                factor[i][i] = sum.sqrt();
                largest = largest.max(factor[i][i]);
                smallest = smallest.min(factor[i][i]);
            } else {
                factor[i][j] = sum / factor[j][j];
            }
        }
    }
    let condition = (largest / smallest).powi(2);
    if !(condition <= MAX_DEFORM_CONDITION) {
        return Err(DeformRefusal::IllConditioned { estimate: condition, limit: MAX_DEFORM_CONDITION });
    }
    let mut lambda: Vec<f64> = constraints.iter().map(|c| c.rhs).collect();
    for i in 0..m {
        let mut sum = lambda[i];
        for t in 0..i {
            sum -= factor[i][t] * lambda[t];
        }
        lambda[i] = sum / factor[i][i];
    }
    for i in (0..m).rev() {
        let mut sum = lambda[i];
        for t in i + 1..m {
            sum -= factor[t][i] * lambda[t];
        }
        lambda[i] = sum / factor[i][i];
    }
    let mut x = vec![0.0; 3 * n];
    for (r, column) in columns.iter().enumerate() {
        if lambda[r] != 0.0 {
            for (value, y) in x.iter_mut().zip(column) {
                *value += lambda[r] * y;
            }
        }
    }
    Ok((x, condition))
}

/// Gram matrices `∫ N_i^(k) N_j^(k)`, k = 0, 1, 2, of one direction's
/// B-spline basis over the domain mapped to [0, 1], by Gauss–Legendre on each
/// span (degree + 1 points: exact for the degree-2p products).
struct Gram {
    /// `entries[i][d]` for `j = i + d - degree`, `d` in `0..=2 degree`.
    entries: Vec<Vec<[f64; 3]>>,
    degree: usize,
}

impl Gram {
    fn at(&self, i: usize, j: usize) -> [f64; 3] {
        let d = j + self.degree - i;
        self.entries[i][d]
    }
}

fn gram(knots: &[f64], degree: usize) -> Gram {
    let count = knots.len() - degree - 1;
    let start = knots[degree];
    let length = (knots[count] - start).max(f64::MIN_POSITIVE);
    let mut entries = vec![vec![[0.0; 3]; 2 * degree + 1]; count];
    let (nodes, weights) = gauss_legendre(degree + 1);
    let vector = KnotVector::new(knots.to_vec(), degree).expect("validated knots");
    let order = 2.min(degree);
    for span in degree..count {
        let (a, b) = (knots[span], knots[span + 1]);
        if b <= a {
            continue;
        }
        for (node, weight) in nodes.iter().zip(&weights) {
            let t = 0.5 * (a + b) + 0.5 * (b - a) * node;
            let w = weight * 0.5 * (b - a) / length;
            let ders = vector.basis_derivatives(span, t, order);
            for x in 0..=degree {
                for y in 0..=degree {
                    let (i, j) = (span - degree + x, span - degree + y);
                    let d = j + degree - i;
                    for k in 0..=order {
                        // d/ds = length · d/du on the unit parameter s.
                        let scale = length.powi(2 * k as i32);
                        entries[i][d][k] += w * scale * ders[k][x] * ders[k][y];
                    }
                }
            }
        }
    }
    Gram { entries, degree }
}

/// Gauss–Legendre nodes and weights on [-1, 1] (Newton on P_n).
fn gauss_legendre(n: usize) -> (Vec<f64>, Vec<f64>) {
    let mut nodes = vec![0.0; n];
    let mut weights = vec![0.0; n];
    for i in 0..n {
        let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (n as f64 + 0.5)).cos();
        let mut derivative = 1.0;
        for _ in 0..100 {
            let (mut p0, mut p1) = (1.0, x);
            for k in 2..=n {
                let p2 = ((2 * k - 1) as f64 * x * p1 - (k - 1) as f64 * p0) / k as f64;
                p0 = p1;
                p1 = p2;
            }
            // n >= 1: p1 = P_n(x), p0 = P_(n-1)(x).
            derivative = n as f64 * (x * p1 - p0) / (x * x - 1.0);
            let step = p1 / derivative;
            x -= step;
            if step.abs() < 1e-16 {
                break;
            }
        }
        nodes[i] = x;
        weights[i] = 2.0 / ((1.0 - x * x) * derivative * derivative);
    }
    (nodes, weights)
}

/// Refuse a result whose normal turns over against the base surface's (at the
/// same parameters) or vanishes, on a grid of every knot span.
fn check_fold(base: &NurbsSurface, deformed: &NurbsSurface) -> Result<(), DeformRefusal> {
    let grid = |knots: &[f64]| {
        let mut out: Vec<f64> = Vec::new();
        for pair in knots.windows(2) {
            if pair[1] > pair[0] {
                for k in 0..=FOLD_SAMPLES_PER_SPAN {
                    out.push(pair[0] + (pair[1] - pair[0]) * k as f64 / FOLD_SAMPLES_PER_SPAN as f64);
                }
            }
        }
        out.dedup();
        out
    };
    // Partials on the unit parameter square, so neither the knot values'
    // scale nor the model's decides what counts as a vanishing normal.
    let scale = net_diagonal(base).max(f64::MIN_POSITIVE);
    let length = |knots: &[f64], degree: usize| knots[knots.len() - 1 - degree] - knots[degree];
    let (lu, lv) = (length(&base.knots_u, base.degree_u), length(&base.knots_v, base.degree_v));
    // A sample is skipped only where the BASE has no normal: a partial that
    // vanishes against the model's size (a pole) or two partials parallel to
    // 1e-9 of their own lengths. A thin surface (one partial tiny but
    // regular) is still checked.
    let floor = 1e-12 * scale;
    for &u in &grid(&base.knots_u) {
        for &v in &grid(&base.knots_v) {
            let before = RationalBasis::at(base, u, v).map_err(DeformRefusal::InvalidSurface)?;
            let after = RationalBasis::at(deformed, u, v).map_err(DeformRefusal::InvalidSurface)?;
            let (du0, dv0) = (before.du.scale(lu), before.dv.scale(lv));
            let (du1, dv1) = (after.du.scale(lu), after.dv.scale(lv));
            let (n0, n1) = (du0.cross(dv0), du1.cross(dv1));
            if !(du0.length() > floor && dv0.length() > floor && n0.length() > 1e-9 * du0.length() * dv0.length()) {
                continue;
            }
            let regular = n1.length() > 1e-9 * du1.length() * dv1.length();
            if !(regular && n1.dot(n0) > 1e-9 * n1.length() * n0.length()) {
                return Err(DeformRefusal::Folded { u, v });
            }
        }
    }
    Ok(())
}

#[path = "deform/tied.rs"]
mod tied;
pub use tied::{deform_surfaces_tied, SurfaceTie, TieContinuity, TiedResult, TiedSurface, MAX_TIED_POLES, MAX_TIED_ROWS};
