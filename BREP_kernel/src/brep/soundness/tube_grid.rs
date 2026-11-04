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

// Promoted together; setting an individual switch to 0 restores its original path.
fn compact_cells_enabled() -> bool {
    switch!("BREP_TUBE_GRID_COMPACT_CELLS", true)
}

fn point_bounds(mut points: impl Iterator<Item = Vec3>) -> Option<(Vec3, Vec3, Vec3, usize)> {
    let first = points.next()?;
    if !(first.x.is_finite() && first.y.is_finite() && first.z.is_finite()) { return None; }
    let (mut lo, mut hi, mut sum, mut count) = (first, first, Vec3::new(0., 0., 0.).add(first), 1);
    for p in points {
        if !(p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) { return None; }
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        sum = sum.add(p);
        count += 1;
    }
    Some((lo, hi, sum, count))
}

// A cell keeps its rectangular control net in one allocation. Rows remain
// slices in the original order, so every certificate fold reads the same bits.
pub(super) struct CellNet {
    points: Vec<Vec3>,
    columns: usize,
}

impl CellNet {
    fn from_rows(rows: Vec<Vec<Vec3>>) -> Option<Self> {
        let columns = rows.first()?.len();
        if columns == 0 || rows.iter().any(|row| row.len() != columns) { return None; }
        Some(Self { points: rows.into_iter().flatten().collect(), columns })
    }
    pub(super) fn iter(&self) -> std::slice::Chunks<'_, Vec3> { self.points.chunks(self.columns) }
    fn len(&self) -> usize { self.points.len() / self.columns }
}

impl std::ops::Index<usize> for CellNet {
    type Output = [Vec3];
    fn index(&self, row: usize) -> &Self::Output {
        &self.points[row * self.columns..(row + 1) * self.columns]
    }
}

impl<'a> IntoIterator for &'a CellNet {
    type Item = &'a [Vec3];
    type IntoIter = std::slice::Chunks<'a, Vec3>;
    fn into_iter(self) -> Self::IntoIter { self.iter() }
}

// Both storage layouts use the same subdivision arithmetic and weight checks.
trait SplitNet {
    fn allocate(rows: usize, columns: usize) -> Self;
    fn put(&mut self, row: usize, column: usize, point: Vec3);
}
impl SplitNet for CellNet {
    fn allocate(rows: usize, columns: usize) -> Self {
        Self { points: vec![Vec3::default(); rows * columns], columns }
    }
    fn put(&mut self, row: usize, column: usize, point: Vec3) {
        self.points[row * self.columns + column] = point;
    }
}
impl SplitNet for Vec<Vec<Vec3>> {
    fn allocate(rows: usize, columns: usize) -> Self { vec![vec![Vec3::default(); columns]; rows] }
    fn put(&mut self, row: usize, column: usize, point: Vec3) { self[row][column] = point; }
}

/// One cell: a Euclidean net rational in u only. Grouped tube cells concatenate
/// consecutive polynomial v-span controls, including their repeated endpoints;
/// consumers use the aggregate hull and cones, never a single polynomial evaluator.
pub(super) struct Cell {
    pub(super) net: CellNet,
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
        Self::from_net(CellNet::from_rows(net)?, weights)
    }

    fn from_net(net: CellNet, weights: Vec<f64>) -> Option<Self> {
        let compact = compact_cells_enabled();
        let points = (!compact).then(|| net.iter().flatten().copied().collect::<Vec<_>>());
        let (lo, hi, sum, point_count) = if let Some(points) = &points {
            // Retain the original passes in the disabled arm.
            if !points.iter().all(|p| p.x.is_finite() && p.y.is_finite() && p.z.is_finite()) { return None; }
            let mut lo = points[0];
            let mut hi = points[0];
            let mut sum = Vec3::new(0., 0., 0.);
            for p in points {
                lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
                hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
                sum = sum.add(*p);
            }
            Some((lo, hi, sum, points.len()))
        } else {
            point_bounds(net.iter().flatten().copied())
        }?;
        let (ku, kv) = (net.len(), net[0].len());
        let mut u = if compact { Vec::with_capacity(ku * (ku - 1) / 2 * kv) } else { Vec::new() };
        for l in 0..kv {
            for k1 in 0..ku {
                for k2 in k1 + 1..ku {
                    u.push(net[k2][l].sub(net[k1][l]));
                }
            }
        }
        let mut v = if compact { Vec::with_capacity(ku * (kv - 1)) } else { Vec::new() };
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
        Some(Self { centroid: sum.scale(1. / point_count as f64), net, weights, lo, hi, u, v, mean_v })
    }
}

impl Cell {
    /// A cell whose tangent cones are given (blend_curved.rs: a net rational
    /// in both directions, its cones read from the hodographs' Bernstein
    /// coefficients).
    pub(super) fn with_cones(net: Vec<Vec<Vec3>>, u: Vec<Vec3>, v: Vec<Vec3>) -> Option<Self> {
        let net = CellNet::from_rows(net)?;
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
    crate::curve::bezier_decompose_raw(knots, points, degree, Vec3::new(0., 0., 0.),
        |a, b, alpha| a.scale(alpha).add(b.scale(1. - alpha)))
}

/// Split a cell's homogeneous net (rows k, columns l) into `parts` equal u
/// pieces. The weight of a row is the same in every column, so it stays so.
pub(super) fn split_u(net: &[Vec<[f64; 4]>], parts: usize) -> Option<Vec<(Vec<Vec<Vec3>>, Vec<f64>)>> {
    split_u_with_storage(net, parts, compact_cells_enabled())
}

// Convert each split column directly into its final Euclidean storage.
// The homogeneous de Casteljau arithmetic and weight checks are identical
// to the general path; only the intermediate nested heap nets disappear.
fn split_halves_compact(net: &[Vec<[f64; 4]>]) -> Option<Vec<(Vec<Vec<Vec3>>, Vec<f64>)>> {
    split_halves_from(net.len() - 1, net[0].len(), |k, l| net[k][l])
}

// Read homogeneous controls directly into the stack column. Wall conversion
// can supply the same products without materializing a temporary heap net.
fn split_halves_from<N: SplitNet>(degree: usize, columns: usize, control: impl Fn(usize, usize) -> [f64; 4]) -> Option<Vec<(N, Vec<f64>)>> {
    // The common linear, quadratic and cubic walls use the same arithmetic
    // with a compile-time order. This lets the compiler unroll de Casteljau
    // and remove unused stack slots without changing any weight check.
    match degree {
        1 => split_halves_order::<2, N>(degree, columns, control),
        2 => split_halves_order::<3, N>(degree, columns, control),
        3 => split_halves_order::<4, N>(degree, columns, control),
        _ => split_halves_order::<{ crate::curve::MAX_STACK_ORDER }, N>(degree, columns, control),
    }
}

fn split_halves_order<const ORDER: usize, N: SplitNet>(degree: usize, columns: usize, control: impl Fn(usize, usize) -> [f64; 4]) -> Option<Vec<(N, Vec<f64>)>> {
    let mut left_net = N::allocate(degree + 1, columns);
    let mut right_net = N::allocate(degree + 1, columns);
    let mut left_weights = vec![0.; degree + 1];
    let mut right_weights = vec![0.; degree + 1];
    for l in 0..columns {
        let mut level = [[0.; 4]; ORDER];
        let mut left = level;
        let mut right = level;
        for k in 0..=degree { level[k] = control(k, l); }
        left[0] = level[0];
        right[degree] = level[degree];
        for r in 1..=degree {
            for i in 0..=degree - r {
                level[i] = std::array::from_fn(|c| 0.5 * level[i][c] + 0.5 * level[i + 1][c]);
            }
            left[r] = level[0];
            right[degree - r] = level[degree - r];
        }
        for k in 0..=degree {
            let (a, b) = (left[k], right[k]);
            if !(a[3] > 0. && b[3] > 0. && a[3].is_finite() && b[3].is_finite()) { return None; }
            if l == 0 { left_weights[k] = a[3]; right_weights[k] = b[3]; }
            else if left_weights[k] != a[3] || right_weights[k] != b[3] { return None; }
            left_net.put(k, l, Vec3::new(a[0] / a[3], a[1] / a[3], a[2] / a[3]));
            right_net.put(k, l, Vec3::new(b[0] / b[3], b[1] / b[3], b[2] / b[3]));
        }
    }
    Some(vec![(left_net, left_weights), (right_net, right_weights)])
}

fn split_u_with_storage(net: &[Vec<[f64; 4]>], parts: usize, compact: bool) -> Option<Vec<(Vec<Vec<Vec3>>, Vec<f64>)>> {
    if compact && parts == 2 && net.len() - 1 <= crate::curve::MAX_STACK_DEGREE {
        return split_halves_compact(net);
    }
    let mut pieces = Vec::with_capacity(parts);
    let mut rest: std::borrow::Cow<'_, [Vec<[f64; 4]>]> = if compact { std::borrow::Cow::Borrowed(net) } else { std::borrow::Cow::Owned(net.to_vec()) };
    for remaining in (2..=parts).rev() {
        let t = 1. / remaining as f64;
        let columns = rest[0].len();
        let degree = rest.len() - 1;
        let mut left = vec![vec![[0.; 4]; columns]; degree + 1];
        let mut right = vec![vec![[0.; 4]; columns]; degree + 1];
        for l in 0..columns {
            if compact && degree <= crate::curve::MAX_STACK_DEGREE {
                let mut level = [[0.; 4]; crate::curve::MAX_STACK_ORDER];
                for (k, row) in rest.iter().enumerate() { level[k] = row[l]; }
                left[0][l] = level[0];
                right[degree][l] = level[degree];
                for r in 1..=degree {
                    for i in 0..=degree - r {
                        level[i] = std::array::from_fn(|c| (1. - t) * level[i][c] + t * level[i + 1][c]);
                    }
                    left[r][l] = level[0];
                    right[degree - r][l] = level[degree - r];
                }
                continue;
            }
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
        rest = std::borrow::Cow::Owned(right);
    }
    pieces.push(rest.into_owned());
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
    let direct_halves = compact_cells_enabled() && SPLIT_U == 2 && pu <= crate::curve::MAX_STACK_DEGREE;
    let mut columns = Vec::with_capacity(spans_u * SPLIT_U);
    for span in 0..spans_u {
        let mut pieces: Vec<Vec<Cell>> = (0..SPLIT_U).map(|_| Vec::with_capacity(spans_v)).collect();
        for j in 0..spans_v {
            let split = if direct_halves {
                split_halves_from(pu, rows[span * pu][j].len(), |k, l| {
                    let w = net[span * pu + k][0].w;
                    let p = rows[span * pu + k][j][l];
                    [p.x * w, p.y * w, p.z * w, w]
                })
            } else {
                let homogeneous: Vec<Vec<[f64; 4]>> = (0..=pu)
                .map(|k| {
                    let w = net[span * pu + k][0].w;
                    rows[span * pu + k][j].iter().map(|p| [p.x * w, p.y * w, p.z * w, w]).collect()
                })
                .collect();
                split_u(&homogeneous, SPLIT_U).and_then(|pieces| pieces.into_iter()
                    .map(|(net, weights)| Some((CellNet::from_rows(net)?, weights))).collect())
            };
            for (q, (piece, weights)) in split.ok_or("a split cell lost its weights")?.into_iter().enumerate() {
                pieces[q].push(Cell::from_net(piece, weights).ok_or("a degenerate cell")?);
            }
        }
        columns.extend(pieces);
    }
    Ok(columns)
}

// Consecutive Bezier spans remain piecewise polynomial: concatenation retains
// every control and every derivative generator. Repeated seam endpoints add
// only a zero difference, removed by Cell::from_net. Exact equality below
// proves continuity at internal boundaries (no jump is hidden in a cone).
// Positive span widths merely rescale derivative generators, preserving cones.
// A grouped cell and adjacent grouped cells have rectangular parameter domains,
// so the existing segment-integral proof covers their internal span boundaries.
// Hull separation uses ALL controls, never just the group endpoints.
fn wall_columns_grouped(surface: &NurbsSurface, group: usize) -> Result<Vec<Vec<Cell>>, Decline> {
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
    if group > 1 {
        for row in &rows {
            for pair in row.windows(2) {
                let (a, b) = (pair[0].last().unwrap(), pair[1].first().unwrap());
                if a.x != b.x || a.y != b.y || a.z != b.z {
                    return Err("grouped spans do not share an exact endpoint");
                }
            }
        }
    }
    // Keep both cap-end spans intact: their original endpoint-column and
    // strict half-space checks are unchanged. Interior aggregate hull checks
    // can be stronger than fine checks; any failure retries the fine proof.
    let mut ranges = Vec::new();
    let mut first = 0;
    while first < spans_v {
        let end = if first == 0 || first == spans_v - 1 { first + 1 }
            else { (first + group).min(spans_v - 1) };
        ranges.push((first, end));
        first = end;
    }
    let direct_halves = compact_cells_enabled() && SPLIT_U == 2 && pu <= crate::curve::MAX_STACK_DEGREE;
    let mut columns = Vec::with_capacity(spans_u * SPLIT_U);
    for span in 0..spans_u {
        let mut pieces: Vec<Vec<Cell>> = (0..SPLIT_U).map(|_| Vec::with_capacity(ranges.len())).collect();
        for &(first, end) in &ranges {
            let count = (end - first) * (pv + 1);
            let control = |k: usize, l: usize| rows[span * pu + k][first + l / (pv + 1)][l % (pv + 1)];
            let split = if direct_halves {
                split_halves_from(pu, count, |k, l| {
                    let w = net[span * pu + k][0].w;
                    let p = control(k, l);
                    [p.x * w, p.y * w, p.z * w, w]
                })
            } else {
                let homogeneous: Vec<Vec<[f64; 4]>> = (0..=pu)
                .map(|k| {
                    let w = net[span * pu + k][0].w;
                    (0..count).map(|l| { let p = control(k, l); [p.x * w, p.y * w, p.z * w, w] }).collect()
                })
                .collect();
                split_u(&homogeneous, SPLIT_U).and_then(|pieces| pieces.into_iter()
                    .map(|(net, weights)| Some((CellNet::from_rows(net)?, weights))).collect())
            };
            for (q, (piece, weights)) in split.ok_or("a split cell lost its weights")?.into_iter().enumerate() {
                pieces[q].push(Cell::from_net(piece, weights).ok_or("a degenerate cell")?);
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

/// `arc` for the two selected endpoint angles, with its identical arithmetic.
fn arc_pair(a: f64, b: f64) -> Option<(f64, f64)> {
    let tau = std::f64::consts::TAU;
    let mut angles = [a.rem_euclid(tau), b.rem_euclid(tau)];
    angles.sort_by(f64::total_cmp);
    let mut widest = (angles[0] + tau - angles[1], 1);
    let gap = angles[1] - angles[0];
    if gap > widest.0 { widest = (gap, 0); }
    let width = tau - widest.0;
    (width < std::f64::consts::PI).then_some((angles[(widest.1 + 1) % 2], width))
}

/// The projected-cone margin of a block (radians): positive proves the block
/// injective, see the module notes.
fn block_margin_with_extrema(cells: &[&Cell], extrema: bool) -> f64 {
    let sum = |vs: &mut dyn Iterator<Item = &Vec3>| vs.fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d));
    let (mean_u, mean_v) = if extrema {
        (cells.iter().flat_map(|c| &c.u).fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d)),
         cells.iter().flat_map(|c| &c.v).fold(Vec3::new(0., 0., 0.), |a, d| a.add(*d)))
    } else {
        (sum(&mut cells.iter().flat_map(|c| &c.u)), sum(&mut cells.iter().flat_map(|c| &c.v)))
    };
    let Ok(normal) = mean_u.cross(mean_v).normalized() else { return -1. };
    let Ok(e1) = mean_u.sub(normal.scale(mean_u.dot(normal))).normalized() else { return -1. };
    let e2 = normal.cross(e1);
    if extrema {
        // In these open half-planes the angular order is the order of y/x
        // (u), or -x/y (v). Read only the endpoint angles, using the same
        // projection and atan2 as the complete search. Other cones fall back.
        fn ends<'a>(vs: impl Iterator<Item = &'a Vec3>, e1: Vec3, e2: Vec3, u: bool) -> Option<(f64, f64)> {
            let mut low = (f64::INFINITY, (0., 0.));
            let mut high = (f64::NEG_INFINITY, (0., 0.));
            for d in vs {
                let (x, y) = (d.dot(e1), d.dot(e2));
                if !(x.is_finite() && y.is_finite() && if u { x > 0. } else { y > 0. }) { return None; }
                let slope = if u { y / x } else { -x / y };
                if !slope.is_finite() { return None; }
                if slope < low.0 { low = (slope, (x, y)); }
                if slope > high.0 { high = (slope, (x, y)); }
            }
            if !low.0.is_finite() { return None; }
            Some((low.1.1.atan2(low.1.0), high.1.1.atan2(high.1.0)))
        }
        if let (Some(au), Some(av)) = (ends(cells.iter().flat_map(|c| &c.u), e1, e2, true), ends(cells.iter().flat_map(|c| &c.v), e1, e2, false)) {
            if let (Some((su, wu)), Some((sv, wv))) = (arc_pair(au.0, au.1), arc_pair(av.0, av.1)) {
                let lead = (sv - su).rem_euclid(std::f64::consts::TAU);
                let margin = (lead - wu).min(std::f64::consts::PI - (lead + wv));
                // Rounded slope ties can exchange adjacent endpoints by a
                // few ulps. Re-read the full cone near the decision boundary;
                // this guard is far wider than division/atan2 roundoff.
                if (margin - MIN_MARGIN).abs() > 1e-12 { return margin; }
            }
        }
    }
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

// A larger rectangular patch union has the same mean-value cone proof as
// a 2x2 block. Certifying it covers every constituent 2x2 domain. Keep the
// original pair-separation checks, and fall back to the original blocks if
// any larger cover fails. No derivative or angular threshold is omitted.
fn cover_column(grid: &[Vec<Cell>], a: usize, b: usize, first: usize, end: usize) -> bool {
    let mut cells = Vec::with_capacity(2 * (end - first + 1));
    for j in first..=end {
        cells.push(&grid[a][j]);
        if a != b { cells.push(&grid[b][j]); }
    }
    if block_margin_with_extrema(&cells, true) > MIN_MARGIN { return true; }
    if end - first <= 1 { return false; }
    let middle = first + (end - first) / 2;
    cover_column(grid, a, b, first, middle) && cover_column(grid, a, b, middle, end)
}

fn check_blocks(grid: &[Vec<Cell>], cyclic: bool, extrema: bool, cover: bool) -> Result<(), Decline> {
    check_blocks_with_stride(grid, cyclic, extrema, cover, 8)
}

fn check_blocks_with_stride(grid: &[Vec<Cell>], cyclic: bool, extrema: bool, cover: bool, stride: usize) -> Result<(), Decline> {
    let (nu, nv) = (grid.len(), grid[0].len());
    let blocks_u = if cyclic { nu } else { nu.saturating_sub(1).max(1) };
    for a in 0..blocks_u {
        let b = if cyclic { (a + 1) % nu } else { (a + 1).min(nu - 1) };
        if cover && extrema && nv > 2 && (0..nv - 1).step_by(stride).all(|first| cover_column(grid, a, b, first, (first + stride).min(nv - 1))) { continue; }
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
            if !(block_margin_with_extrema(&block, extrema) > MIN_MARGIN) {
                return Err("a block's tangent cones do not prove it injective");
            }
        }
    }
    Ok(())
}

// Keep the broad-phase data contiguous: most x candidates separate in y/z
// without needing their cell's heap nets or cyclic-neighbor arithmetic.
fn separated_pairs_compact(grid: &[Vec<Cell>], cyclic: bool, tau: f64) -> Result<(usize, usize), Decline> {
    struct Bounds<'a> { cell: &'a Cell, a: usize, j: usize, lo: Vec3, hi_tau: Vec3 }
    let nu = grid.len();
    let mut ordered: Vec<_> = grid.iter().enumerate().flat_map(|(a, column)| {
        column.iter().enumerate().map(move |(j, cell)| Bounds { cell, a, j, lo: cell.lo,
            hi_tau: Vec3::new(cell.hi.x + tau, cell.hi.y + tau, cell.hi.z + tau) })
    }).collect();
    ordered.sort_by(|a, b| a.lo.x.total_cmp(&b.lo.x));
    let (mut candidates, mut gaps) = (0, 0);
    for (i, cell) in ordered.iter().enumerate() {
        for other in &ordered[i + 1..] {
            if other.lo.x > cell.hi_tau.x { break; }
            candidates += 1;
            if other.lo.y > cell.hi_tau.y || cell.lo.y > other.hi_tau.y
                || other.lo.z > cell.hi_tau.z || cell.lo.z > other.hi_tau.z { continue; }
            let du = cell.a.abs_diff(other.a);
            let du = if cyclic { du.min(nu - du) } else { du };
            if du <= 1 && cell.j.abs_diff(other.j) <= 1 { continue; }
            gaps += 1;
            if !(gap(cell.cell, other.cell) > tau) { return Err("two cells sharing no block are not separated"); }
        }
    }
    Ok((candidates, gaps))
}

/// The cells of one grid carry no two distinct parameter points to one
/// point: every 2x2 block (cyclic in u when `cyclic`) is injective by its
/// tangent cones, and every cell pair sharing no block is separated by more
/// than `tau` (sweep along x first).
pub(super) fn injective(grid: &[Vec<Cell>], cyclic: bool, tau: f64) -> Result<(), Decline> {
    injective_with_stride(grid, cyclic, tau, 8)
}

fn injective_with_stride(grid: &[Vec<Cell>], cyclic: bool, tau: f64, stride: usize) -> Result<(), Decline> {
    let compact = compact_cells_enabled();
    let extrema = std::env::var_os("BREP_TUBE_GRID_CONE_EXTREMA").is_none_or(|v| v != "0");
    let started = crate::imprint::profile_enabled().then(web_time::Instant::now);
    let (nu, nv) = (grid.len(), grid[0].len());
    let cover = std::env::var_os("BREP_TUBE_GRID_BLOCK_COVER").is_none_or(|v| v != "0");
    if stride == 8 { check_blocks(grid, cyclic, extrema, cover)?; }
    else { check_blocks_with_stride(grid, cyclic, extrema, cover, stride)?; }
    let blocks_elapsed = started.map(|t| t.elapsed());
    let (mut candidate_pairs, mut gap_calls) = (0_usize, 0_usize);
    if compact {
        (candidate_pairs, gap_calls) = separated_pairs_compact(grid, cyclic, tau)?;
    } else {
    let mut by_x: Vec<(usize, usize)> = (0..nu).flat_map(|a| (0..nv).map(move |j| (a, j))).collect();
    by_x.sort_by(|p, q| grid[p.0][p.1].lo.x.total_cmp(&grid[q.0][q.1].lo.x));
    for (i, &(a, j)) in by_x.iter().enumerate() {
        let cell = &grid[a][j];
        for &(b, k) in &by_x[i + 1..] {
            let other = &grid[b][k];
            if other.lo.x > cell.hi.x + tau {
                break;
            }
            candidate_pairs += usize::from(started.is_some());
            let du = if cyclic {
                let d = if compact { a.abs_diff(b) } else { (a + nu - b) % nu };
                d.min(nu - d)
            } else { a.abs_diff(b) };
            if du <= 1 && j.abs_diff(k) <= 1 {
                continue;
            }
            let boxes_apart = other.lo.y > cell.hi.y + tau
                || cell.lo.y > other.hi.y + tau
                || other.lo.z > cell.hi.z + tau
                || cell.lo.z > other.hi.z + tau;
            gap_calls += usize::from(started.is_some() && !boxes_apart);
            if !boxes_apart && !(gap(cell, other) > tau) {
                return Err("two cells sharing no block are not separated");
            }
        }
    }
    }
    if let (Some(started), Some(blocks)) = (started, blocks_elapsed) {
        crate::imprint::profile_emit(format_args!("tube_grid.injective cells={} nu={} nv={} candidates={} gaps={} blocks_ms={:.6} pairs_ms={:.6}",
            nu * nv, nu, nv, candidate_pairs, gap_calls, blocks.as_secs_f64() * 1000., started.elapsed().saturating_sub(blocks).as_secs_f64() * 1000.));
    }
    Ok(())
}

/// `Ok(cells)` when the body is proved free of crossings and folds.
pub(super) fn verdict(solid: &BrepSolid, options: SelfIntersectionOptions) -> Result<usize, Decline> {
    let group = if switch!("BREP_TUBE_GRID_GROUPED_CELLS", true) { 4 } else { 1 };
    verdict_with_fallback(solid, options, group)
}

fn verdict_with_fallback(solid: &BrepSolid, options: SelfIntersectionOptions, group: usize) -> Result<usize, Decline> {
    if group > 1 {
        if let Ok(cells) = verdict_grouped(solid, options, group) { return Ok(cells); }
    }
    verdict_grouped(solid, options, 1)
}

fn verdict_grouped(solid: &BrepSolid, options: SelfIntersectionOptions, group: usize) -> Result<usize, Decline> {
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
        grid.extend(if group == 1 { wall_columns(&walls[*index].surface)? }
            else { wall_columns_grouped(&walls[*index].surface, group)? });
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
    // Adjacent covers share their last/first group, so every neighboring
    // pair is covered even when the number of groups per cover shrinks.
    if group == 1 { injective(&grid, true, tau)?; }
    else { injective_with_stride(&grid, true, tau, (8 / group).max(1))?; }
    profile.lap("injectivity");
    Ok(nu * nv)
}

