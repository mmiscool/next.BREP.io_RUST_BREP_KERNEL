//! Whole-body glued patch-grid certificate (lane K2 row b,
//! `BREP_SOUNDNESS_TUBE_GRID`, on by default, `=0` to disable): a body made of NURBS WALLS
//! glued into one ring along their u boundaries and closed by two planar
//! CAPS, whose control nets prove that no two distinct parameter points meet,
//! holds no crossing and no fold for the full scan to confirm. That is the
//! shape a sweep or a loft accepts; nothing here reads who built the body.
//! Anything this module cannot prove declines to the full scan.
use super::analytic_body::{carrier_of, Carrier, Plane};
use crate::topology::EdgeRecord;
use crate::Vec4;
use super::*;

/// Why a body was left to the full scan.
pub(super) type Decline = &'static str;

pub(super) fn enabled() -> bool {
    // Same gate as the analytic body: with either trim switch off the scan's
    // notion of material is the whole carrier.
    if !super::trim_samples_enabled() || !super::node_membership_enabled() {
        return false;
    }
    // On by default (promoted 2026-10-15); `BREP_SOUNDNESS_TUBE_GRID=0`
    // (any value but 1/on) is the escape hatch back to the full scan.
    match std::env::var("BREP_SOUNDNESS_TUBE_GRID") {
        Ok(value) => value == "1" || value == "on",
        Err(std::env::VarError::NotPresent) => true,
        Err(_) => false,
    }
}

pub(super) fn certifies(solid: &BrepSolid, options: SelfIntersectionOptions) -> bool {
    match verdict(solid, options) {
        Ok(_) => true,
        Err(reason) => {
            if std::env::var_os("BREP_DEBUG_TUBE_GRID").is_some() {
                eprintln!("soundness.tube_grid decline: {reason}");
            }
            false
        }
    }
}

// What is proved, and why it covers the full scan's verdict.
//
// - Walls. Every wall is rational in u with weights constant along v (so a
//   polynomial in v) and already in Bezier form in u; v is converted to
//   Bezier spans by knot insertion. Each u span is halved again (de Casteljau
//   in homogeneous space), giving CELLS. Walls whose u-boundary rows agree to
//   `DELTA` of the body form a ring; the cells form one grid, cyclic in u.
// - Tangent cones. Over a cell with positive weights, the u-partial is a
//   non-negative combination of the pairwise differences E[k2][l] - E[k1][l]
//   (k2 > k1, every column l): a rational Bezier's derivative is such a
//   combination of its own control differences, and the u-isocurve's control
//   points share one denominator. The v-partial is a convex combination of
//   the rows' derivatives, each in the cone of consecutive differences.
// - Injectivity of a 2x2 block of cells. Project on the plane normal to
//   (mean u-difference x mean v-difference). If every projected u-direction
//   lies in an arc A, every projected v-direction in an arc B, and every angle
//   of B is ahead of every angle of A by more than zero and less than pi, then
//   for two parameter points p != q of the block's (convex, glued) domain,
//   F(p) - F(q) = du * U + dv * V with U in cone A and V in cone B (the mean
//   value integral along the segment, valid across the C0 seams), and U, V are
//   independent: F(p) != F(q). So no point pair inside one block meets: no
//   fold, and no crossing between faces of the block.
// - Cells sharing no block: their control points (whose hull holds each
//   patch) are separated by more than `tau` along a world axis, the centroid
//   difference, or either cell's mean v-direction.
// - Caps. A cap is a native plane (`analytic_body`'s chart: affine, injective).
//   Its loop traces, through that chart, exactly the edges that are the walls'
//   rows at one v end, and those rows lie within `tau` of the plane. A cap is
//   bounded by its rim, so it lies in the convex hull of the rim cells' end
//   columns. Every cell of the rim row keeps its other control points more
//   than `tau` on ONE side of the plane: near the rim no wall reaches the
//   other side by more than `tau`, and a confirmed crossing of a wall and a
//   cap needs a straddle of the band (1e-6 of the diagonal) one way or the
//   other: the corner confirmation needs both, the pierce needs the pierced
//   surface or the piercing edge across the other carrier. Every other cell
//   is either more than `tau` on one side (it misses the plane) or more than
//   `tau` apart from the rim's hull along a direction in the plane (it misses
//   the cap): a tube may bend back across its cap's plane outside the cap.
//   The two rims' hulls are apart by more than `tau`, so the caps are too.
//
// Tolerances are the analytic body's: `tau` is the scan's weld radius,
// recognition is held to `DELTA` = 1e-12 of the body, and an angular margin
// below `MIN_MARGIN` declines rather than trust rounding.

const DELTA: f64 = 1e-12;
/// Each u Bezier span is split in this many cells: a quarter-circle span
/// halved turns its tangent by 45 degrees, two of them stay under pi.
pub(super) const SPLIT_U: usize = 2;
const MIN_MARGIN: f64 = 1e-6;
const MAX_FACES: usize = 64;
pub(super) const MAX_CELLS: usize = 8192;

/// One cell: a (pu+1) x (pv+1) Euclidean net, rational in u only.
pub(super) struct Cell {
    pub(super) net: Vec<Vec<Vec3>>,
    /// The weight of each u row (the same in every v column).
    #[cfg_attr(not(test), allow(dead_code))]
    weights: Vec<f64>,
    pub(super) lo: Vec3,
    pub(super) hi: Vec3,
    centroid: Vec3,
    u: Vec<Vec3>,
    v: Vec<Vec3>,
    mean_v: Vec3,
}

impl Cell {
    pub(super) fn new(net: Vec<Vec<Vec3>>, weights: Vec<f64>) -> Option<Self> {
        let points: Vec<Vec3> = net.iter().flatten().copied().collect();
        if !points.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) {
            return None;
        }
        let mut lo = points[0];
        let mut hi = points[0];
        let mut sum = Vec3::new(0., 0., 0.);
        for p in &points {
            lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            sum = sum.add(*p);
        }
        let (ku, kv) = (net.len(), net[0].len());
        let mut u = Vec::new();
        for l in 0..kv {
            for k1 in 0..ku {
                for k2 in k1 + 1..ku {
                    u.push(net[k2][l].sub(net[k1][l]));
                }
            }
        }
        let mut v = Vec::new();
        for row in &net {
            for l in 0..kv - 1 {
                v.push(row[l + 1].sub(row[l]));
            }
        }
        // An exactly repeated control point contributes nothing to a cone.
        u.retain(|d| d.length_squared() > 0.);
        v.retain(|d| d.length_squared() > 0.);
        if u.is_empty() || v.is_empty() {
            return None;
        }
        let mean_v = v.iter().fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d)).scale(1. / v.len() as f64);
        Some(Self { centroid: sum.scale(1. / points.len() as f64), net, weights, lo, hi, u, v, mean_v })
    }
}

impl Cell {
    /// A cell whose tangent cones are given (blend_curved.rs: a net rational
    /// in both directions, its cones read from the hodographs' Bernstein
    /// coefficients).
    pub(super) fn with_cones(net: Vec<Vec<Vec3>>, u: Vec<Vec3>, v: Vec<Vec3>) -> Option<Self> {
        let points: Vec<Vec3> = net.iter().flatten().copied().collect();
        if points.is_empty() || !points.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) || u.is_empty() || v.is_empty() {
            return None;
        }
        let mut lo = points[0];
        let mut hi = points[0];
        let mut sum = Vec3::new(0., 0., 0.);
        for p in &points {
            lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
            sum = sum.add(*p);
        }
        let mean_v = v.iter().fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d)).scale(1. / v.len() as f64);
        Some(Self { centroid: sum.scale(1. / points.len() as f64), net, weights: Vec::new(), lo, hi, u, v, mean_v })
    }
}

/// Clamped polynomial B-spline -> its Bezier spans (degree + 1 points each),
/// by inserting every interior knot to multiplicity `degree` (Piegl & Tiller
/// A5.6: one pass, each span refined in place, O(n p^2)).
pub(super) fn bezier_spans(knots: &[f64], points: &[Vec3], degree: usize) -> Option<Vec<Vec<Vec3>>> {
    let p = degree;
    let n = points.len();
    if p == 0 || n < p + 1 || knots.len() != n + p + 1 || knots.iter().any(|k| !k.is_finite()) || knots.windows(2).any(|w| w[1] < w[0]) {
        return None;
    }
    let (a, b) = (knots[0], knots[knots.len() - 1]);
    if !(b > a) || knots[..=p].iter().any(|k| *k != a) || knots[knots.len() - p - 1..].iter().any(|k| *k != b) {
        return None;
    }
    let m = knots.len() - 1;
    let (mut a, mut b) = (p, p + 1);
    let mut spans = Vec::with_capacity(n);
    let mut span: Vec<Vec3> = points[..=p].to_vec();
    let mut alphas = vec![0.; p];
    while b < m {
        let first = b;
        while b < m && knots[b + 1] == knots[b] {
            b += 1;
        }
        let mult = b - first + 1;
        if b < m && mult > p {
            return None;
        }
        let mut next = vec![Vec3::new(0., 0., 0.); p + 1];
        if mult < p {
            let numer = knots[b] - knots[a];
            for j in (mult + 1..=p).rev() {
                alphas[j - mult - 1] = numer / (knots[a + j] - knots[a]);
            }
            let r = p - mult;
            for j in 1..=r {
                let s = mult + j;
                for k in (s..=p).rev() {
                    let alpha = alphas[k - s];
                    span[k] = span[k].scale(alpha).add(span[k - 1].scale(1. - alpha));
                }
                if b < m {
                    next[r - j] = span[p];
                }
            }
        }
        spans.push(span);
        if b >= m {
            break;
        }
        for i in p - mult..=p {
            next[i] = points[b - p + i];
        }
        span = next;
        a = b;
        b += 1;
    }
    Some(spans)
}

/// Split a cell's homogeneous net (rows k, columns l) into `parts` equal u
/// pieces. The weight of a row is the same in every column, so it stays so.
pub(super) fn split_u(net: &[Vec<[f64; 4]>], parts: usize) -> Option<Vec<(Vec<Vec<Vec3>>, Vec<f64>)>> {
    let mut pieces = Vec::with_capacity(parts);
    let mut rest = net.to_vec();
    for remaining in (2..=parts).rev() {
        let t = 1. / remaining as f64;
        let columns = rest[0].len();
        let degree = rest.len() - 1;
        let mut left = vec![vec![[0.; 4]; columns]; degree + 1];
        let mut right = vec![vec![[0.; 4]; columns]; degree + 1];
        for l in 0..columns {
            let mut level: Vec<[f64; 4]> = rest.iter().map(|row| row[l]).collect();
            left[0][l] = level[0];
            right[degree][l] = level[degree];
            for r in 1..=degree {
                level = (0..level.len() - 1)
                    .map(|i| std::array::from_fn(|c| (1. - t) * level[i][c] + t * level[i + 1][c]))
                    .collect();
                left[r][l] = level[0];
                right[degree - r][l] = level[level.len() - 1];
            }
        }
        pieces.push(left);
        rest = right;
    }
    pieces.push(rest);
    pieces
        .into_iter()
        .map(|piece| {
            let weights: Vec<f64> = piece.iter().map(|row| row[0][3]).collect();
            let net = piece
                .into_iter()
                .map(|row| {
                    let w = row[0][3];
                    if !(w > 0.) || !w.is_finite() || row.iter().any(|h| h[3] != w) {
                        return None;
                    }
                    Some(row.into_iter().map(|h| Vec3::new(h[0] / w, h[1] / w, h[2] / w)).collect())
                })
                .collect::<Option<_>>()?;
            Some((net, weights))
        })
        .collect()
}

/// One wall's cells as columns of the ring grid: `[u column][v span]`.
fn wall_columns(surface: &NurbsSurface) -> Result<Vec<Vec<Cell>>, Decline> {
    let (pu, pv) = (surface.degree_u, surface.degree_v);
    let net = &surface.control_points;
    if pu == 0 || pv == 0 || net.len() < 2 || net.iter().any(|row| row.len() != net[0].len()) {
        return Err("a wall net is not a grid");
    }
    if surface.knots_u.len() != net.len() + pu + 1 {
        return Err("a wall's u knots do not fit its net");
    }
    for row in net {
        let w = row[0].w;
        if !(w > 0.) || !w.is_finite() || row.iter().any(|c| c.w != w) {
            return Err("a wall's weights vary along v");
        }
    }
    // u must already be Bezier: clamped, every interior knot of multiplicity pu.
    let ku = &surface.knots_u;
    let (a, b) = (ku[0], ku[ku.len() - 1]);
    if !(b > a) || ku[..=pu].iter().any(|k| *k != a) || ku[ku.len() - pu - 1..].iter().any(|k| *k != b) {
        return Err("a wall's u knots are not clamped");
    }
    let mut interior = ku[pu + 1..ku.len() - pu - 1].to_vec();
    interior.dedup();
    if interior.iter().any(|k| ku.iter().filter(|x| *x == k).count() != pu) {
        return Err("a wall is not Bezier in u");
    }
    let rows: Vec<Vec<Vec<Vec3>>> = net
        .iter()
        .map(|row| {
            let euclid: Vec<Vec3> = row.iter().map(|c| Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w)).collect();
            bezier_spans(&surface.knots_v, &euclid, pv)
        })
        .collect::<Option<_>>()
        .ok_or("a wall's v knots are not a clamped spline")?;
    let spans_v = rows[0].len();
    let spans_u = (net.len() - 1) / pu;
    let mut columns = Vec::with_capacity(spans_u * SPLIT_U);
    for span in 0..spans_u {
        let mut pieces: Vec<Vec<Cell>> = (0..SPLIT_U).map(|_| Vec::with_capacity(spans_v)).collect();
        for j in 0..spans_v {
            let homogeneous: Vec<Vec<[f64; 4]>> = (0..=pu)
                .map(|k| {
                    let w = net[span * pu + k][0].w;
                    rows[span * pu + k][j].iter().map(|p| [p.x * w, p.y * w, p.z * w, w]).collect()
                })
                .collect();
            for (q, (piece, weights)) in split_u(&homogeneous, SPLIT_U).ok_or("a split cell lost its weights")?.into_iter().enumerate() {
                pieces[q].push(Cell::new(piece, weights).ok_or("a degenerate cell")?);
            }
        }
        columns.extend(pieces);
    }
    Ok(columns)
}

/// The smallest arc holding every angle, as (start, width), when it is
/// narrower than pi.
fn arc(mut angles: Vec<f64>) -> Option<(f64, f64)> {
    let tau = std::f64::consts::TAU;
    for a in &mut angles {
        *a = a.rem_euclid(tau);
    }
    angles.sort_by(f64::total_cmp);
    let mut widest = (angles[0] + tau - angles[angles.len() - 1], angles.len() - 1);
    for i in 0..angles.len() - 1 {
        let gap = angles[i + 1] - angles[i];
        if gap > widest.0 {
            widest = (gap, i);
        }
    }
    let start = angles[(widest.1 + 1) % angles.len()];
    let width = tau - widest.0;
    (width < std::f64::consts::PI).then_some((start, width))
}

/// The projected-cone margin of a block (radians): positive proves the block
/// injective, see the module notes.
fn block_margin(cells: &[&Cell]) -> f64 {
    let sum = |vs: &mut dyn Iterator<Item = &Vec3>| vs.fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d));
    let mean_u = sum(&mut cells.iter().flat_map(|c| &c.u));
    let mean_v = sum(&mut cells.iter().flat_map(|c| &c.v));
    let Ok(normal) = mean_u.cross(mean_v).normalized() else { return -1. };
    let Ok(e1) = mean_u.sub(normal.scale(mean_u.dot(normal))).normalized() else { return -1. };
    let e2 = normal.cross(e1);
    let angles = |vs: &mut dyn Iterator<Item = &Vec3>| -> Option<Vec<f64>> {
        vs.map(|d| {
            let (x, y) = (d.dot(e1), d.dot(e2));
            (x != 0. || y != 0.).then(|| y.atan2(x))
        })
        .collect()
    };
    let (Some(au), Some(av)) = (angles(&mut cells.iter().flat_map(|c| &c.u)), angles(&mut cells.iter().flat_map(|c| &c.v))) else {
        return -1.;
    };
    let (Some((start_u, width_u)), Some((start_v, width_v))) = (arc(au), arc(av)) else { return -2. };
    let lead = (start_v - start_u).rem_euclid(std::f64::consts::TAU);
    // Every v angle minus every u angle lies in (lead - width_u, lead + width_v).
    (lead - width_u).min(std::f64::consts::PI - (lead + width_v))
}

/// How far two cells' control points sit apart along the best tried axis.
fn gap(a: &Cell, b: &Cell) -> f64 {
    let axes = [
        Vec3::new(1., 0., 0.),
        Vec3::new(0., 1., 0.),
        Vec3::new(0., 0., 1.),
        b.centroid.sub(a.centroid),
        a.mean_v,
        b.mean_v,
    ];
    let mut best = f64::NEG_INFINITY;
    for axis in axes {
        let Ok(axis) = axis.normalized() else { continue };
        let range = |c: &Cell| {
            c.net.iter().flatten().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
                let t = p.dot(axis);
                (lo.min(t), hi.max(t))
            })
        };
        let ((alo, ahi), (blo, bhi)) = (range(a), range(b));
        best = best.max(blo - ahi).max(alo - bhi);
    }
    best
}

/// How far two point sets sit apart along the best of `axes` (any length;
/// a zero axis is skipped). Positive: their convex hulls are disjoint.
fn apart(a: &[Vec3], b: &[Vec3], axes: &[Vec3]) -> f64 {
    let mut best = f64::NEG_INFINITY;
    for axis in axes {
        let Ok(axis) = axis.normalized() else { continue };
        let range = |points: &[Vec3]| points.iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| (lo.min(p.dot(axis)), hi.max(p.dot(axis))));
        let ((alo, ahi), (blo, bhi)) = (range(a), range(b));
        best = best.max(blo - ahi).max(alo - bhi);
    }
    best
}

/// Do two control polygons (homogeneous `Vec4`, with knots) describe one
/// curve, in the same or the reversed order? `size` scales `DELTA`.
fn same_curve(a: &[(Vec3, f64)], a_knots: &[f64], b: &[(Vec3, f64)], b_knots: &[f64], size: f64) -> bool {
    if a.len() != b.len() || a_knots.len() != b_knots.len() {
        return false;
    }
    let close = |p: &(Vec3, f64), q: &(Vec3, f64)| {
        p.0.sub(q.0).length() <= DELTA * size && (p.1 - q.1).abs() <= DELTA * p.1.abs().max(q.1.abs())
    };
    let span = |k: &[f64]| (k[0], k[k.len() - 1]);
    let ((a0, a1), (b0, b1)) = (span(a_knots), span(b_knots));
    let knot_slack = DELTA * (a1 - a0).abs().max(b1 - b0).max(1.);
    // Same parameterisation up to an affine change of parameter.
    let scaled = |k: f64, from: (f64, f64), to: (f64, f64)| to.0 + (k - from.0) * (to.1 - to.0) / (from.1 - from.0);
    let forward = a.iter().zip(b).all(|(p, q)| close(p, q))
        && a_knots.iter().zip(b_knots).all(|(x, y)| (scaled(*x, (a0, a1), (b0, b1)) - y).abs() <= knot_slack);
    let reversed = a.iter().zip(b.iter().rev()).all(|(p, q)| close(p, q))
        && a_knots.iter().zip(b_knots.iter().rev()).all(|(x, y)| (scaled(*x, (a0, a1), (b1, b0)) - y).abs() <= knot_slack);
    forward || reversed
}

fn euclid_weighted(points: &[Vec4]) -> Vec<(Vec3, f64)> {
    points.iter().map(|c| (Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w), c.w)).collect()
}

/// The cells of one grid carry no two distinct parameter points to one
/// point: every 2x2 block (cyclic in u when `cyclic`) is injective by its
/// tangent cones, and every cell pair sharing no block is separated by more
/// than `tau` (sweep along x first).
pub(super) fn injective(grid: &[Vec<Cell>], cyclic: bool, tau: f64) -> Result<(), Decline> {
    let (nu, nv) = (grid.len(), grid[0].len());
    let blocks_u = if cyclic { nu } else { nu.saturating_sub(1).max(1) };
    for a in 0..blocks_u {
        let b = if cyclic { (a + 1) % nu } else { (a + 1).min(nu - 1) };
        for j in 0..nv.saturating_sub(1).max(1) {
            let mut block = vec![&grid[a][j]];
            if b != a {
                block.push(&grid[b][j]);
            }
            if nv > 1 {
                block.push(&grid[a][j + 1]);
                if b != a {
                    block.push(&grid[b][j + 1]);
                }
            }
            if !(block_margin(&block) > MIN_MARGIN) {
                return Err("a block's tangent cones do not prove it injective");
            }
        }
    }
    let mut by_x: Vec<(usize, usize)> = (0..nu).flat_map(|a| (0..nv).map(move |j| (a, j))).collect();
    by_x.sort_by(|p, q| grid[p.0][p.1].lo.x.total_cmp(&grid[q.0][q.1].lo.x));
    for (i, &(a, j)) in by_x.iter().enumerate() {
        let cell = &grid[a][j];
        for &(b, k) in &by_x[i + 1..] {
            let other = &grid[b][k];
            if other.lo.x > cell.hi.x + tau {
                break;
            }
            let du = if cyclic { let d = (a + nu - b) % nu; d.min(nu - d) } else { a.abs_diff(b) };
            if du <= 1 && j.abs_diff(k) <= 1 {
                continue;
            }
            let boxes_apart = other.lo.y > cell.hi.y + tau
                || cell.lo.y > other.hi.y + tau
                || other.lo.z > cell.hi.z + tau
                || cell.lo.z > other.hi.z + tau;
            if !boxes_apart && !(gap(cell, other) > tau) {
                return Err("two cells sharing no block are not separated");
            }
        }
    }
    Ok(())
}

/// `Ok(cells)` when the body is proved free of crossings and folds.
pub(super) fn verdict(solid: &BrepSolid, options: SelfIntersectionOptions) -> Result<usize, Decline> {
    let mut profile = crate::loft_topology::ConstructionProfile::new("tube_grid");
    let tau = options.weld;
    if !(tau > 0.) || !tau.is_finite() {
        return Err("no weld radius to bound the proof");
    }
    let records: Vec<&FaceRecord> = solid.shells.iter().flat_map(|shell| &shell.faces).collect();
    if records.len() < 3 || records.len() > MAX_FACES {
        return Err("face count outside 3..=64");
    }
    // Cheap structure first: exactly two native planes, the rest walls.
    let mut caps: Vec<(&FaceRecord, Plane)> = Vec::new();
    let mut walls: Vec<&FaceRecord> = Vec::new();
    for record in &records {
        let s = &record.surface;
        if s.degree_u == 1 && s.degree_v == 1 && s.control_points.len() == 2 {
            if let Some(Carrier::Plane(plane)) = carrier_of(s) {
                caps.push((record, plane));
                continue;
            }
        }
        walls.push(record);
    }
    if caps.len() != 2 {
        return Err("not exactly two planar caps");
    }
    // Topology: a closed shell of shared edges, none degenerate.
    let mut uses: HashMap<u64, usize> = HashMap::default();
    for record in &records {
        if record.loops.len() != 1 {
            return Err("a face with other than one loop");
        }
        for coedge in &record.loops[0].coedges {
            *uses.entry(coedge.edge_id).or_default() += 1;
        }
    }
    if uses.values().any(|n| *n != 2) {
        return Err("an edge without exactly two uses");
    }
    let edges: HashMap<u64, &EdgeRecord> = solid.edges.iter().map(|e| (e.id, e)).collect();
    if uses.keys().any(|id| edges.get(id).is_none_or(|e| e.degenerate)) {
        return Err("a missing or degenerate edge");
    }
    // Every wall trim stays in its chart, so every trimmed point is a point
    // of the proved map.
    for wall in &walls {
        let s = &wall.surface;
        let (u0, u1) = (s.knots_u[0], s.knots_u[s.knots_u.len() - 1]);
        let (v0, v1) = (s.knots_v[0], s.knots_v[s.knots_v.len() - 1]);
        for coedge in &wall.loops[0].coedges {
            for c in &coedge.pcurve.control_points {
                if !(c.w > 0.) {
                    return Err("a wall pcurve weight is not positive");
                }
                let (u, v) = (c.x / c.w, c.y / c.w);
                if !(u >= u0 && u <= u1 && v >= v0 && v <= v1) {
                    return Err("a wall trim leaves its chart");
                }
            }
        }
    }
    let all: Vec<Vec3> = records
        .iter()
        .flat_map(|r| r.surface.control_points.iter().flatten())
        .map(|c| Vec3::new(c.x / c.w, c.y / c.w, c.z / c.w))
        .collect();
    let size = all.iter().map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs())).fold(1., f64::max);
    // Glue the walls into one ring: a wall's last u row is the next wall's
    // first, with one v knot vector throughout.
    let rows = |w: &FaceRecord, last: bool| {
        let net = &w.surface.control_points;
        euclid_weighted(if last { &net[net.len() - 1] } else { &net[0] })
    };
    let rows_meet = |a: &[(Vec3, f64)], b: &[(Vec3, f64)]| {
        a.len() == b.len()
            && a.iter().zip(b).all(|(p, q)| p.0.sub(q.0).length() <= DELTA * size && (p.1 - q.1).abs() <= DELTA * p.1.abs().max(q.1.abs()))
    };
    let knots_v = &walls[0].surface.knots_v;
    if walls.iter().any(|w| w.surface.knots_v != *knots_v) {
        return Err("walls with different v knots");
    }
    let mut order = vec![0];
    while order.len() < walls.len() {
        let last = rows(walls[*order.last().unwrap()], true);
        let next: Vec<usize> = (0..walls.len()).filter(|i| !order.contains(i) && rows_meet(&last, &rows(walls[*i], false))).collect();
        let [next] = next[..] else { return Err("the walls do not glue into one ring") };
        order.push(next);
    }
    if !rows_meet(&rows(walls[*order.last().unwrap()], true), &rows(walls[order[0]], false)) {
        return Err("the wall ring does not close");
    }
    // A cap trim must stay in its plane's chart; checked again below with
    // the rim, but here it declines before any cell is built.
    for (cap, plane) in &caps {
        for coedge in &cap.loops[0].coedges {
            if coedge.pcurve.control_points.iter().any(|c| !(c.w > 0.) || !plane.contains_parameter(c.x / c.w, c.y / c.w)) {
                return Err("a cap trim leaves its chart");
            }
        }
    }
    profile.lap("structure");
    let mut grid: Vec<Vec<Cell>> = Vec::new();
    for index in &order {
        grid.extend(wall_columns(&walls[*index].surface)?);
        profile.lap("polynomial_v_wall");
    }
    let (nu, nv) = (grid.len(), grid[0].len());
    if nu < 3 || nu * nv > MAX_CELLS || grid.iter().any(|column| column.len() != nv) {
        return Err("a cell grid outside the proof's size");
    }
    // Caps first: they cost one pass over the nets, so a wall that meets a
    // cap declines here before the pairwise passes. Each cap traces the walls'
    // rows at one v end, which lie on its plane. The cap face lies in the
    // convex hull of that rim, so in the hull of the rim cells' end columns.
    // A cell of the rim row keeps every other control point strictly on one
    // side of the plane; any other cell is either strictly on one side or
    // apart from the rim's hull along a direction in the plane.
    let pv = walls[0].surface.degree_v;
    let mut ends = Vec::new();
    let mut rims: Vec<Vec<Vec3>> = Vec::new();
    for (cap, plane) in &caps {
        let normal = plane.normal();
        let level = normal.dot(plane.at_origin());
        let g = |p: Vec3| normal.dot(p) - level;
        let rim = |end: usize| (0..nu).all(|a| {
            let cell = &grid[a][if end == 0 { 0 } else { nv - 1 }];
            cell.net.iter().all(|row| g(row[if end == 0 { 0 } else { pv }]).abs() <= tau)
        });
        let end = match (rim(0), rim(1)) {
            (true, false) => 0,
            (false, true) => 1,
            _ => return Err("a cap's plane does not hold exactly one wall end"),
        };
        let (rim_row, rim_column) = if end == 0 { (0, 0) } else { (nv - 1, pv) };
        let rim_points: Vec<Vec3> = grid.iter().flat_map(|column| column[rim_row].net.iter().map(|row| row[rim_column])).collect();
        for column in &grid {
            for (j, cell) in column.iter().enumerate() {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for row in &cell.net {
                    for (l, p) in row.iter().enumerate() {
                        if !(j == rim_row && l == rim_column) {
                            lo = lo.min(g(*p));
                            hi = hi.max(g(*p));
                        }
                    }
                }
                if lo > tau || hi < -tau {
                    continue;
                }
                if j == rim_row {
                    return Err("a rim cell leaves its cap's plane on both sides");
                }
                let centre = |points: &mut dyn Iterator<Item = &Vec3>| {
                    let (sum, count) = points.fold((Vec3::new(0., 0., 0.), 0.), |(s, c), q| (s.add(*q), c + 1.));
                    sum.scale(1. / count)
                };
                let cell_points: Vec<Vec3> = cell.net.iter().flatten().copied().collect();
                let towards = centre(&mut cell_points.iter()).sub(centre(&mut rim_points.iter()));
                let axes = [towards, Vec3::new(1., 0., 0.), Vec3::new(0., 1., 0.), Vec3::new(0., 0., 1.)]
                    .map(|d| d.sub(normal.scale(d.dot(normal))));
                if !(apart(&cell_points, &rim_points, &axes) > tau) {
                    return Err("a wall cell straddles a cap's plane inside its rim's hull");
                }
            }
        }
        rims.push(rim_points);
        // The cap's loop is the rim: each coedge traces, through the affine
        // chart, an edge that is one wall's row at this end.
        let coedges = &cap.loops[0].coedges;
        if coedges.len() != walls.len() {
            return Err("a cap's loop is not one edge per wall");
        }
        let mut matched = vec![false; walls.len()];
        for coedge in coedges {
            let edge = edges[&coedge.edge_id];
            let k = &edge.curve.knots;
            if edge.t0 != k[0] || edge.t1 != k[k.len() - 1] {
                return Err("a rim edge is a sub-range of its curve");
            }
            let image: Option<Vec<(Vec3, f64)>> = coedge
                .pcurve
                .control_points
                .iter()
                .map(|c| {
                    let (u, v) = (c.x / c.w, c.y / c.w);
                    plane.contains_parameter(u, v).then(|| (plane.at(u, v), c.w))
                })
                .collect();
            let image = image.ok_or("a cap trim leaves its chart")?;
            let edge_points = euclid_weighted(&edge.curve.control_points);
            if !same_curve(&image, &coedge.pcurve.knots, &edge_points, k, size) {
                return Err("a cap pcurve does not trace its edge");
            }
            let wall = (0..walls.len()).find(|w| {
                let s = &walls[*w].surface;
                let row: Vec<Vec4> = s.control_points.iter().map(|r| r[if end == 0 { 0 } else { r.len() - 1 }]).collect();
                !matched[*w] && same_curve(&euclid_weighted(&row), &s.knots_u, &edge_points, k, size)
            });
            let wall = wall.ok_or("a cap edge is no wall's end row")?;
            matched[wall] = true;
        }
        ends.push(end);
    }
    if ends[0] == ends[1] {
        return Err("both caps close the same wall end");
    }
    // The two caps are apart: each lies in its rim's hull.
    let towards = rims[1].iter().fold(Vec3::new(0., 0., 0.), |s, q| s.add(*q)).sub(rims[0].iter().fold(Vec3::new(0., 0., 0.), |s, q| s.add(*q)));
    let axes = [caps[0].1.normal(), caps[1].1.normal(), towards, Vec3::new(1., 0., 0.), Vec3::new(0., 1., 0.), Vec3::new(0., 0., 1.)];
    if !(apart(&rims[0], &rims[1], &axes) > tau) {
        return Err("the two caps are not apart");
    }
    profile.lap("caps");
    injective(&grid, true, tau)?;
    profile.lap("injectivity");
    Ok(nu * nv)
}

