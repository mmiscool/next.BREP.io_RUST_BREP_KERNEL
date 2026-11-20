//! Several surfaces deformed together, tied along shared boundary sides.
//!
//! Each surface is prepared as for one ([`prepare_base`]); then every tie's
//! two boundary rows are made comparable EXACTLY (elevation to one degree and
//! the union of the knots along the edge, both directions mapped onto each
//! other affinely, reversed when the tie says so) and must then coincide pole
//! for pole. One quadratic programme over all the surfaces' free poles
//! (block-diagonal thin-plate energy, [`solve_blocks`]) carries:
//!
//! - the picks of every surface;
//! - **G0** tie rows: each pair of matching boundary poles moves together;
//! - **G1** tie rows where the BASE edge is smooth: with `c_a = P0 − P1a` (into
//!   the edge from surface a) and `c_b = P1b − P0` (out of it into b) collinear
//!   with ONE ratio `λ = |c_b| / |c_a|` along the whole edge, the rows
//!   `(D1b − D0) = λ (D0 − D1a)` keep them so (linear: λ is frozen). That is a
//!   common tangent plane along the edge for polynomial rows; a smooth edge
//!   whose ratio varies, or whose rows are rational, is refused
//!   ([`DeformRefusal::EdgeNotUniformlyTangent`]) rather than creased;
//! - at a surface corner where two of its tied sides meet, the 2×2 pole block
//!   is held (G1 there would need twist compatibility the rows do not give).
//!
//! A sharp shared edge (not smooth in the base) takes G0 only; its dihedral
//! must keep its sign along the edge, or the surfaces have swung through each
//! other there ([`DeformRefusal::EdgeFoldedThrough`]).

use super::*;

/// One surface of a tied deformation: its picks, holds and room in `spec`
/// (picks may be empty), and which of its control points may move.
pub struct TiedSurface<'a> {
    pub surface: &'a NurbsSurface,
    pub spec: DeformSpec,
    pub may_move: &'a dyn Fn(PoleSupport) -> bool,
}

/// Surfaces `a` and `b` share their boundary sides `side_a` and `side_b`;
/// `reversed` when the two sides run opposite ways along the edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceTie {
    pub a: usize,
    pub side_a: DeformSide,
    pub b: usize,
    pub side_b: DeformSide,
    pub reversed: bool,
}

/// What a tie kept across its edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TieContinuity {
    /// Position only: the base edge is sharp, and its dihedral kept its sign.
    Sharp,
    /// Position and tangent plane, with the frozen cross ratio.
    Tangent { ratio: f64 },
}

#[derive(Clone, Debug)]
pub struct TiedResult {
    /// One per input surface, in order.
    pub results: Vec<DeformResult>,
    /// One per tie, in order.
    pub continuity: Vec<TieContinuity>,
    /// Corner blocks held because two tied sides meet there.
    pub held_corners: usize,
}

/// The direction along a side (the parameter that varies on it).
fn along(side: DeformSide) -> KnotDirection {
    match side {
        DeformSide::UMin | DeformSide::UMax => KnotDirection::V,
        DeformSide::VMin | DeformSide::VMax => KnotDirection::U,
    }
}

/// The net index `(i, j)` of the pole `depth` rows in from `side`, `k` along it.
fn index_on(surface: &NurbsSurface, side: DeformSide, depth: usize, k: usize) -> (usize, usize) {
    let rows = surface.control_points.len();
    let columns = surface.control_points[0].len();
    match side {
        DeformSide::UMin => (depth, k),
        DeformSide::UMax => (rows - 1 - depth, k),
        DeformSide::VMin => (k, depth),
        DeformSide::VMax => (k, columns - 1 - depth),
    }
}

fn side_len(surface: &NurbsSurface, side: DeformSide) -> usize {
    match along(side) {
        KnotDirection::V => surface.control_points[0].len(),
        KnotDirection::U => surface.control_points.len(),
    }
}

/// The edge parameter range of `side` (the domain of the along direction).
fn along_domain(surface: &NurbsSurface, side: DeformSide) -> [f64; 2] {
    let (degree, knots) = knots_of(surface, along(side));
    [knots[degree], knots[knots.len() - 1 - degree]]
}

/// Interior knots (with multiplicity) of the along direction.
fn interior_knots(surface: &NurbsSurface, direction: KnotDirection) -> Vec<f64> {
    let (degree, knots) = knots_of(surface, direction);
    knots[degree + 1..knots.len() - 1 - degree].to_vec()
}

/// Insert into `target` (along `direction`) every knot of `source` (mapped by
/// `map`) that `target` lacks, with multiplicity. Returns whether it changed.
fn merge_knots(
    target: &mut NurbsSurface,
    direction: KnotDirection,
    source: &[f64],
    map: impl Fn(f64) -> f64,
) -> Result<bool, DeformRefusal> {
    let mine = interior_knots(target, direction);
    // One knot, relative to the domain: an absolute bar would merge distinct
    // knots on a small domain (the rows then match while the curves do not).
    let (degree, knots) = knots_of(target, direction);
    let snap = KNOT_RELATIVE * (knots[knots.len() - 1 - degree] - knots[degree]);
    let mut inserts = Vec::new();
    let mut i = 0;
    while i < source.len() {
        let value = source[i];
        let count = source[i..].iter().take_while(|&&k| k == value).count();
        let mapped = map(value);
        let have = mine.iter().filter(|&&k| (k - mapped).abs() <= snap).count();
        for _ in have..count {
            inserts.push(mapped);
        }
        i += count;
    }
    if inserts.is_empty() {
        return Ok(false);
    }
    *target = target.refine_knots(direction, &inserts).map_err(DeformRefusal::Refinement)?;
    Ok(true)
}

/// Knots this close, relative to the domain, are one.
const KNOT_RELATIVE: f64 = 1e-12;
/// The whole tied system's bounds: free control points over every surface,
/// and constraint rows (the Schur complement and its columns are dense:
/// rows × 3 × points doubles, about 300 MB at these bounds).
pub const MAX_TIED_POLES: usize = 8192;
pub const MAX_TIED_ROWS: usize = 1536;

/// Deform `surfaces` together under `ties`. Every surface comes back on its
/// prepared (elevated, split, merged) net; a tie's two boundary rows come back
/// identical, and tangent-continuous where the base edge was.
pub fn deform_surfaces_tied(surfaces: &[TiedSurface], ties: &[SurfaceTie]) -> Result<TiedResult, DeformRefusal> {
    let count = surfaces.len();
    for (index, tie) in ties.iter().enumerate() {
        if tie.a >= count || tie.b >= count || (tie.a == tie.b && tie.side_a == tie.side_b) {
            return Err(DeformRefusal::InvalidTie { tie: index, reason: format!("{tie:?} on {count} surfaces") });
        }
    }
    let total_picks: usize = surfaces.iter().map(|s| s.spec.picks.len()).sum();
    if total_picks == 0 {
        return Err(DeformRefusal::NoPicks);
    }
    if total_picks > MAX_DEFORM_PICKS {
        return Err(DeformRefusal::TooManyPicks { count: total_picks, limit: MAX_DEFORM_PICKS });
    }
    // Each surface's picks: finite, inside its domain, a usable normal, no two
    // at one parameter pair (the single-surface checks, per surface).
    let mut first = 0;
    for surface in surfaces {
        let [u0, u1] = surface.surface.domain_u().map_err(DeformRefusal::InvalidSurface)?;
        let [v0, v1] = surface.surface.domain_v().map_err(DeformRefusal::InvalidSurface)?;
        let picks = &surface.spec.picks;
        for (k, pick) in picks.iter().enumerate() {
            let index = first + k;
            if ![pick.u, pick.v, pick.target.x, pick.target.y, pick.target.z].iter().all(|c| c.is_finite()) {
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
            for (other, earlier) in picks[..k].iter().enumerate() {
                if ((earlier.u - pick.u) / (u1 - u0).max(f64::MIN_POSITIVE)).abs() <= 1e-9
                    && ((earlier.v - pick.v) / (v1 - v0).max(f64::MIN_POSITIVE)).abs() <= 1e-9
                {
                    return Err(DeformRefusal::DuplicatePicks { first: first + other, second: index });
                }
            }
        }
        first += picks.len();
    }
    let mut bases = Vec::with_capacity(count);
    for surface in surfaces {
        bases.push(prepare_base(surface.surface, &surface.spec)?);
    }

    // Make every tie's rows comparable: one degree along the edge, then the
    // union of the knots, until nothing changes.
    // Each pass can carry a knot one tie further, so a chain of n ties needs
    // up to n passes (plus one to see nothing change); a pass bound that is
    // not reached is a refusal, never a silent stop.
    let passes = 2 * ties.len() + 2;
    let mut converged = false;
    for _ in 0..passes {
        let mut changed = false;
        for tie in ties {
            let (da, db) = (along(tie.side_a), along(tie.side_b));
            let pa = knots_of(&bases[tie.a], da).0;
            let pb = knots_of(&bases[tie.b], db).0;
            if pa < pb {
                bases[tie.a] = bases[tie.a].elevate_degree(da, pb).map_err(DeformRefusal::InvalidSurface)?;
                changed = true;
            } else if pb < pa {
                bases[tie.b] = bases[tie.b].elevate_degree(db, pa).map_err(DeformRefusal::InvalidSurface)?;
                changed = true;
            }
            let [a0, a1] = along_domain(&bases[tie.a], tie.side_a);
            let [b0, b1] = along_domain(&bases[tie.b], tie.side_b);
            let reversed = tie.reversed;
            let to_a = move |t: f64| {
                let s = (t - b0) / (b1 - b0);
                a0 + (a1 - a0) * if reversed { 1.0 - s } else { s }
            };
            let to_b = move |t: f64| {
                let s = (t - a0) / (a1 - a0);
                b0 + (b1 - b0) * if reversed { 1.0 - s } else { s }
            };
            let from_b = interior_knots(&bases[tie.b], db);
            if tie.a != tie.b || da != db {
                let mut target = bases[tie.a].clone();
                changed |= merge_knots(&mut target, da, &from_b, to_a)?;
                bases[tie.a] = target;
                let from_a = interior_knots(&bases[tie.a], da);
                let mut target = bases[tie.b].clone();
                changed |= merge_knots(&mut target, db, &from_a, to_b)?;
                bases[tie.b] = target;
            }
        }
        if !changed {
            converged = true;
            break;
        }
    }
    if !converged {
        return Err(DeformRefusal::InvalidTie { tie: 0, reason: format!("knot merging did not settle in {passes} passes") });
    }
    for base in &bases {
        let poles = base.control_points.len() * base.control_points[0].len();
        if poles > MAX_DEFORM_POLES {
            return Err(DeformRefusal::NetTooLarge { poles, limit: MAX_DEFORM_POLES });
        }
    }

    // The rows must now coincide pole for pole, with proportional weights.
    let scale = bases.iter().map(net_diagonal).fold(0.0, f64::max).max(f64::MIN_POSITIVE);
    for (index, tie) in ties.iter().enumerate() {
        let (na, nb) = (side_len(&bases[tie.a], tie.side_a), side_len(&bases[tie.b], tie.side_b));
        if na != nb {
            return Err(DeformRefusal::EdgeRowsDisagree { tie: index, gap: f64::INFINITY });
        }
        // The along-edge knot vectors must be one (mapped onto each other),
        // or equal rows would still be different curves.
        let (da, db) = (along(tie.side_a), along(tie.side_b));
        let (ka, kb) = (interior_knots(&bases[tie.a], da), interior_knots(&bases[tie.b], db));
        let [a0, a1] = along_domain(&bases[tie.a], tie.side_a);
        let [b0, b1] = along_domain(&bases[tie.b], tie.side_b);
        let mapped: Vec<f64> = {
            let mut m: Vec<f64> = kb
                .iter()
                .map(|&t| {
                    let s = (t - b0) / (b1 - b0);
                    a0 + (a1 - a0) * if tie.reversed { 1.0 - s } else { s }
                })
                .collect();
            m.sort_by(f64::total_cmp);
            m
        };
        if knots_of(&bases[tie.a], da).0 != knots_of(&bases[tie.b], db).0
            || ka.len() != mapped.len()
            || ka.iter().zip(&mapped).any(|(x, y)| (x - y).abs() > KNOT_RELATIVE * (a1 - a0))
        {
            return Err(DeformRefusal::EdgeRowsDisagree { tie: index, gap: f64::INFINITY });
        }
        let mut gap = 0.0f64;
        let mut ratio: Option<f64> = None;
        for k in 0..na {
            let kb = if tie.reversed { na - 1 - k } else { k };
            let (ia, ja) = index_on(&bases[tie.a], tie.side_a, 0, k);
            let (ib, jb) = index_on(&bases[tie.b], tie.side_b, 0, kb);
            let (pa, pb) = (bases[tie.a].control_points[ia][ja], bases[tie.b].control_points[ib][jb]);
            gap = gap.max(cartesian(pa).sub(cartesian(pb)).length());
            let r = pb.w / pa.w;
            match ratio {
                None => ratio = Some(r),
                Some(first) if (r - first).abs() <= 1e-12 * first => {}
                Some(_) => gap = f64::INFINITY,
            }
        }
        if !(gap <= 1e-12 * scale) {
            return Err(DeformRefusal::EdgeRowsDisagree { tie: index, gap });
        }
    }

    // Free poles of every surface: its held sides, its own `may_move`, and the
    // corner blocks where two of its tied sides meet.
    let mut tied_sides: Vec<Vec<DeformSide>> = vec![Vec::new(); count];
    for tie in ties {
        tied_sides[tie.a].push(tie.side_a);
        tied_sides[tie.b].push(tie.side_b);
    }
    let mut held_corners = 0;
    let mut free_index: Vec<Vec<Vec<usize>>> = Vec::with_capacity(count);
    let mut free: Vec<Vec<(usize, usize)>> = Vec::with_capacity(count);
    let mut offsets = Vec::with_capacity(count);
    let mut total = 0usize;
    for (s, base) in bases.iter().enumerate() {
        let rows = base.control_points.len();
        let columns = base.control_points[0].len();
        let depth = |hold: EdgeHold| match hold {
            EdgeHold::Free => 0,
            EdgeHold::Position => 1,
            EdgeHold::Tangent => 2,
        };
        let spec = &surfaces[s].spec;
        let (i_lo, i_hi) = (depth(spec.hold[0]), rows.saturating_sub(depth(spec.hold[1])));
        let (j_lo, j_hi) = (depth(spec.hold[2]), columns.saturating_sub(depth(spec.hold[3])));
        let sides = &tied_sides[s];
        let corner_u = |side: DeformSide| sides.contains(&side);
        let mut corners: Vec<(bool, bool)> = Vec::new(); // (u at max, v at max)
        for (us, vs, key) in [
            (DeformSide::UMin, DeformSide::VMin, (false, false)),
            (DeformSide::UMin, DeformSide::VMax, (false, true)),
            (DeformSide::UMax, DeformSide::VMin, (true, false)),
            (DeformSide::UMax, DeformSide::VMax, (true, true)),
        ] {
            if corner_u(us) && corner_u(vs) {
                corners.push(key);
                held_corners += 1;
            }
        }
        let in_corner = |i: usize, j: usize| {
            corners.iter().any(|&(umax, vmax)| {
                let iu = if umax { i + 2 >= rows } else { i < 2 };
                let jv = if vmax { j + 2 >= columns } else { j < 2 };
                iu && jv
            })
        };
        let (pu, pv) = (base.degree_u, base.degree_v);
        let mut index = vec![vec![usize::MAX; columns]; rows];
        let mut list = Vec::new();
        for i in i_lo..i_hi.max(i_lo) {
            for j in j_lo..j_hi.max(j_lo) {
                if in_corner(i, j) {
                    continue;
                }
                let (ku, kv) = (&base.knots_u[i..=i + pu + 1], &base.knots_v[j..=j + pv + 1]);
                let order =
                    |local: &[f64], end: f64, degree: usize| (degree + 1).saturating_sub(local.iter().filter(|&&k| k == end).count());
                let support = PoleSupport {
                    bounds: [ku[0], ku[pu + 1], kv[0], kv[pv + 1]],
                    vanishing: [order(ku, ku[0], pu), order(ku, ku[pu + 1], pu), order(kv, kv[0], pv), order(kv, kv[pv + 1], pv)],
                };
                if (surfaces[s].may_move)(support) {
                    index[i][j] = list.len();
                    list.push((i, j));
                }
            }
        }
        offsets.push(total);
        total += list.len();
        free_index.push(index);
        free.push(list);
    }

    if total > MAX_TIED_POLES {
        return Err(DeformRefusal::NetTooLarge { poles: total, limit: MAX_TIED_POLES });
    }
    // Constraints: every surface's picks, then the ties.
    let mut constraints: Vec<Constraint> = Vec::new();
    let mut requested = 0.0f64;
    let mut first_pick = 0;
    for (s, base) in bases.iter().enumerate() {
        let limit = MAX_DEFORM_DISPLACEMENT_RATIO * net_diagonal(base);
        pick_constraints(base, &surfaces[s].spec.picks, &free_index[s], offsets[s], first_pick, limit, &mut constraints, &mut requested)?;
        first_pick += surfaces[s].spec.picks.len();
    }
    let mut continuity = Vec::with_capacity(ties.len());
    for (t, tie) in ties.iter().enumerate() {
        let n = side_len(&bases[tie.a], tie.side_a);
        let global = |s: usize, (i, j): (usize, usize)| {
            let f = free_index[s][i][j];
            (f != usize::MAX).then(|| offsets[s] + f)
        };
        // G0 rows.
        for k in 0..n {
            let kb = if tie.reversed { n - 1 - k } else { k };
            let a0 = global(tie.a, index_on(&bases[tie.a], tie.side_a, 0, k));
            let b0 = global(tie.b, index_on(&bases[tie.b], tie.side_b, 0, kb));
            for axis in 0..3 {
                let mut terms = Vec::new();
                if let Some(g) = a0 {
                    terms.push((3 * g + axis, 1.0));
                }
                if let Some(g) = b0 {
                    terms.push((3 * g + axis, -1.0));
                }
                push_tie(&mut constraints, t, terms, 0.0)?;
            }
        }
        // Smooth in the base? Read the two carriers' normals along the edge.
        let smooth = edge_is_smooth(&bases[tie.a], tie.side_a, &bases[tie.b], tie.side_b, tie.reversed)?;
        if !smooth {
            continuity.push(TieContinuity::Sharp);
            continue;
        }
        // The cross pairs: collinear with ONE ratio, on polynomial rows.
        let mut ratio: Option<f64> = None;
        for k in 0..n {
            let kb = if tie.reversed { n - 1 - k } else { k };
            let pole = |s: usize, side: DeformSide, depth: usize, k: usize| {
                let (i, j) = index_on(&bases[s], side, depth, k);
                bases[s].control_points[i][j]
            };
            let (p0, p1a, p1b) = (pole(tie.a, tie.side_a, 0, k), pole(tie.a, tie.side_a, 1, k), pole(tie.b, tie.side_b, 1, kb));
            if [p0.w, p1a.w, p1b.w].iter().any(|&w| w != 1.0) {
                return Err(DeformRefusal::EdgeNotUniformlyTangent { tie: t, reason: "rational rows".into() });
            }
            let (c_a, c_b) = (cartesian(p0).sub(cartesian(p1a)), cartesian(p1b).sub(cartesian(p0)));
            let (la, lb) = (c_a.length(), c_b.length());
            if la <= 1e-12 * scale || lb <= 1e-12 * scale {
                return Err(DeformRefusal::EdgeNotUniformlyTangent { tie: t, reason: format!("a collapsed cross pair at {k}") });
            }
            if c_a.cross(c_b).length() > 1e-9 * la * lb || c_a.dot(c_b) <= 0.0 {
                return Err(DeformRefusal::EdgeNotUniformlyTangent { tie: t, reason: format!("cross pair {k} not collinear") });
            }
            let r = lb / la;
            match ratio {
                None => ratio = Some(r),
                Some(first) if (r - first).abs() <= 1e-9 * first => {}
                Some(first) => {
                    return Err(DeformRefusal::EdgeNotUniformlyTangent {
                        tie: t,
                        reason: format!("cross ratio {r} at {k}, {first} at 0"),
                    })
                }
            }
        }
        let lambda = ratio.unwrap_or(1.0);
        for k in 0..n {
            let kb = if tie.reversed { n - 1 - k } else { k };
            let d0 = global(tie.a, index_on(&bases[tie.a], tie.side_a, 0, k));
            let d1a = global(tie.a, index_on(&bases[tie.a], tie.side_a, 1, k));
            let d1b = global(tie.b, index_on(&bases[tie.b], tie.side_b, 1, kb));
            for axis in 0..3 {
                let mut terms = Vec::new();
                for (g, c) in [(d1b, 1.0), (d0, -(1.0 + lambda)), (d1a, lambda)] {
                    if let Some(g) = g {
                        terms.push((3 * g + axis, c));
                    }
                }
                push_tie(&mut constraints, t, terms, 0.0)?;
            }
        }
        continuity.push(TieContinuity::Tangent { ratio: lambda });
    }

    if constraints.len() > MAX_TIED_ROWS {
        return Err(DeformRefusal::TooManyConstraints { rows: constraints.len(), limit: MAX_TIED_ROWS });
    }
    // One solve over every surface.
    let changing = constraints.iter().any(|c| c.rhs != 0.0);
    let mut energies = Vec::with_capacity(count);
    for s in 0..count {
        energies.push(BandedEnergy::build(&bases[s], &free[s], &free_index[s])?);
    }
    let (displacements, condition) = if changing {
        let blocks: Vec<(&BandedEnergy, usize)> = energies.iter().zip(&offsets).map(|(e, &o)| (e, o)).collect();
        solve_blocks(&blocks, total, &constraints)?
    } else {
        (vec![0.0; 3 * total], 1.0)
    };

    // The deformed nets.
    let mut nets: Vec<Vec<Vec<Vec4>>> = Vec::with_capacity(count);
    let mut max_pole_shift = 0.0f64;
    let mut shifts = vec![0.0f64; count];
    for (s, base) in bases.iter().enumerate() {
        let mut net = base.control_points.clone();
        for (f, &(i, j)) in free[s].iter().enumerate() {
            let g = offsets[s] + f;
            let d = Vec3::new(displacements[3 * g], displacements[3 * g + 1], displacements[3 * g + 2]);
            if d.x == 0.0 && d.y == 0.0 && d.z == 0.0 {
                continue;
            }
            let pole = net[i][j];
            net[i][j] = Vec4 { x: pole.x + d.x * pole.w, y: pole.y + d.y * pole.w, z: pole.z + d.z * pole.w, w: pole.w };
            shifts[s] = shifts[s].max(d.length());
        }
        max_pole_shift = max_pole_shift.max(shifts[s]);
        nets.push(net);
    }
    // G0 exactly: the tie rows hold to the solve's precision (~1e-10); where
    // either pole of a tied pair could move, b's pole is made a's (scaled by
    // the pair's constant weight ratio), so the shared edge is one curve bit
    // for bit. A pair held on both sides is left as it was.
    for tie in ties {
        let n = side_len(&bases[tie.a], tie.side_a);
        for k in 0..n {
            let kb = if tie.reversed { n - 1 - k } else { k };
            let (ia, ja) = index_on(&bases[tie.a], tie.side_a, 0, k);
            let (ib, jb) = index_on(&bases[tie.b], tie.side_b, 0, kb);
            if free_index[tie.a][ia][ja] == usize::MAX && free_index[tie.b][ib][jb] == usize::MAX {
                continue;
            }
            let pa = nets[tie.a][ia][ja];
            let w = nets[tie.b][ib][jb].w;
            let ratio = w / pa.w;
            nets[tie.b][ib][jb] = Vec4 { x: pa.x * ratio, y: pa.y * ratio, z: pa.z * ratio, w };
        }
    }
    // G1 exactly, likewise: on a tangent tie the inner pole of b is made
    // P0 + λ (P0 − P1a) from the deformed a side (polynomial rows, checked
    // above), wherever it could move. The rows hold that to ~1e-10 already;
    // this makes the cross pairs collinear to round-off.
    for (t, tie) in ties.iter().enumerate() {
        let TieContinuity::Tangent { ratio: lambda } = continuity[t] else { continue };
        let n = side_len(&bases[tie.a], tie.side_a);
        for k in 0..n {
            let kb = if tie.reversed { n - 1 - k } else { k };
            let (i0, j0) = index_on(&bases[tie.a], tie.side_a, 0, k);
            let (i1, j1) = index_on(&bases[tie.a], tie.side_a, 1, k);
            let (ib, jb) = index_on(&bases[tie.b], tie.side_b, 1, kb);
            if free_index[tie.b][ib][jb] == usize::MAX {
                continue;
            }
            let (p0, p1a) = (cartesian(nets[tie.a][i0][j0]), cartesian(nets[tie.a][i1][j1]));
            let p1b = p0.add(p0.sub(p1a).scale(lambda));
            nets[tie.b][ib][jb] = Vec4 { x: p1b.x, y: p1b.y, z: p1b.z, w: 1.0 };
        }
    }
    let mut deformed = Vec::with_capacity(count);
    for (base, net) in bases.iter().zip(nets) {
        deformed.push(
            NurbsSurface::new(base.degree_u, base.degree_v, base.knots_u.clone(), base.knots_v.clone(), net)
                .map_err(DeformRefusal::InvalidSurface)?,
        );
    }
    if max_pole_shift > MAX_DEFORM_AMPLIFICATION * requested {
        return Err(DeformRefusal::Amplified {
            ratio: max_pole_shift / requested.max(f64::MIN_POSITIVE),
            limit: MAX_DEFORM_AMPLIFICATION,
        });
    }
    for s in 0..count {
        check_fold(&bases[s], &deformed[s])?;
    }
    for (t, tie) in ties.iter().enumerate() {
        if continuity[t] == TieContinuity::Sharp {
            check_dihedral(t, tie, &bases, &deformed)?;
        }
    }

    let mut results = Vec::with_capacity(count);
    let mut pick_index = 0;
    for s in 0..count {
        let mut max_pick_error = 0.0f64;
        let mut max_normal_error = 0.0f64;
        for pick in &surfaces[s].spec.picks {
            let point = deformed[s].evaluate(pick.u, pick.v).map_err(DeformRefusal::InvalidSurface)?;
            max_pick_error = max_pick_error.max(point.sub(pick.target).length());
            if let Some(normal) = pick.normal {
                let (_, su, sv) = deformed[s].deriv1(pick.u, pick.v).map_err(DeformRefusal::InvalidSurface)?;
                let n = su.cross(sv);
                let sine = n.cross(normal).length() / (n.length() * normal.length()).max(f64::MIN_POSITIVE);
                max_normal_error = max_normal_error.max(sine.min(1.0).asin());
            }
            pick_index += 1;
        }
        let rows = deformed[s].control_points.len();
        let columns = deformed[s].control_points[0].len();
        results.push(DeformResult {
            surface: deformed[s].clone(),
            net: (rows, columns),
            free_poles: free[s].len(),
            max_pick_error,
            max_normal_error,
            max_pole_shift: shifts[s],
            condition,
        });
    }
    let _ = pick_index;
    Ok(TiedResult { results, continuity, held_corners })
}

/// A tie row: dropped when no free pole reaches it (both ends held), else
/// pushed unit-normalised and named by its tie.
fn push_tie(constraints: &mut Vec<Constraint>, tie: usize, terms: Vec<(usize, f64)>, rhs: f64) -> Result<(), DeformRefusal> {
    if terms.is_empty() {
        return Ok(());
    }
    let norm = terms.iter().map(|(_, c)| c * c).sum::<f64>().sqrt();
    let terms = terms.into_iter().map(|(index, c)| (index, c / norm)).collect();
    constraints.push(Constraint { pick: usize::MAX, tie: Some(tie), terms, rhs: rhs / norm });
    Ok(())
}

/// Stations along a tie's edge: (parameter on a's side, on b's side).
fn edge_stations(a: &NurbsSurface, side_a: DeformSide, b: &NurbsSurface, side_b: DeformSide, reversed: bool) -> Vec<(f64, f64)> {
    let [a0, a1] = along_domain(a, side_a);
    let [b0, b1] = along_domain(b, side_b);
    (1..24)
        .map(|k| {
            let s = k as f64 / 24.0;
            (a0 + (a1 - a0) * s, b0 + (b1 - b0) * if reversed { 1.0 - s } else { s })
        })
        .collect()
}

/// The `(u, v)` of edge parameter `t` on `side`.
fn on_side(surface: &NurbsSurface, side: DeformSide, t: f64) -> (f64, f64) {
    let [u0, u1] = surface.domain_u().unwrap_or([0.0, 1.0]);
    let [v0, v1] = surface.domain_v().unwrap_or([0.0, 1.0]);
    match side {
        DeformSide::UMin => (u0, t),
        DeformSide::UMax => (u1, t),
        DeformSide::VMin => (t, v0),
        DeformSide::VMax => (t, v1),
    }
}

/// The unit normal (`S_u × S_v`) of `surface` on `side` at edge parameter `t`.
fn side_normal(surface: &NurbsSurface, side: DeformSide, t: f64) -> Result<Vec3, DeformRefusal> {
    let (u, v) = on_side(surface, side, t);
    let basis = RationalBasis::at(surface, u, v).map_err(DeformRefusal::InvalidSurface)?;
    let n = basis.du.cross(basis.dv);
    let length = n.length();
    Ok(if length > 0.0 { n.scale(1.0 / length) } else { n })
}

/// Whether the base edge is smooth: the two carriers' normals parallel (either
/// sense: the orientations of two carriers need not agree) at every station.
fn edge_is_smooth(
    a: &NurbsSurface,
    side_a: DeformSide,
    b: &NurbsSurface,
    side_b: DeformSide,
    reversed: bool,
) -> Result<bool, DeformRefusal> {
    for (ta, tb) in edge_stations(a, side_a, b, side_b, reversed) {
        let (na, nb) = (side_normal(a, side_a, ta)?, side_normal(b, side_b, tb)?);
        if na.cross(nb).length() > 1e-7 {
            return Ok(false);
        }
    }
    Ok(true)
}

/// The dihedral of a sharp edge keeps its sign: `(n_a × n_b) · T` (T the edge
/// tangent along a's side) at every station, deformed against base.
fn check_dihedral(t: usize, tie: &SurfaceTie, bases: &[NurbsSurface], deformed: &[NurbsSurface]) -> Result<(), DeformRefusal> {
    let turn = |surfaces: &[NurbsSurface], ta: f64, tb: f64| -> Result<f64, DeformRefusal> {
        let (a, b) = (&surfaces[tie.a], &surfaces[tie.b]);
        let (na, nb) = (side_normal(a, tie.side_a, ta)?, side_normal(b, tie.side_b, tb)?);
        let (u, v) = on_side(a, tie.side_a, ta);
        let basis = RationalBasis::at(a, u, v).map_err(DeformRefusal::InvalidSurface)?;
        let tangent = match along(tie.side_a) {
            KnotDirection::U => basis.du,
            KnotDirection::V => basis.dv,
        };
        let length = tangent.length().max(f64::MIN_POSITIVE);
        Ok(na.cross(nb).dot(tangent.scale(1.0 / length)))
    };
    for (ta, tb) in edge_stations(&bases[tie.a], tie.side_a, &bases[tie.b], tie.side_b, tie.reversed) {
        let before = turn(bases, ta, tb)?;
        let after = turn(deformed, ta, tb)?;
        if !(before * after > 0.0) {
            return Err(DeformRefusal::EdgeFoldedThrough { tie: t, t: ta });
        }
    }
    Ok(())
}
