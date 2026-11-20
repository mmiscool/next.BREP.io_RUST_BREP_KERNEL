//! An exact certificate that a NURBS surface is regular: its normal
//! `N = ∂S/∂u × ∂S/∂v` vanishes nowhere and never turns back, so the sheet
//! neither collapses nor folds over onto itself.
//!
//! This is a local property (the surface is an immersion): it does not
//! certify that the sheet is embedded. A sheet that winds round and lies over
//! itself without ever folding (an annulus traversed twice) has a regular
//! normal everywhere and is certified. Positivity is judged to a roundoff
//! band of 1e-13 of the coefficients' size: a fold confined to a parameter
//! region of that order at a domain corner where the normal vanishes is
//! indistinguishable from the tangent corner the exemption below admits.
//!
//! # Method
//!
//! The surface is cut at its interior knots into Bézier pieces (exact knot
//! insertion). On a polynomial piece of degree `(p, q)` each component of `N`
//! is a polynomial of degree `(2p − 1, 2q − 1)`; on a rational piece with
//! homogeneous points `P` and weights `w`,
//! `w⁴ (S_u × S_v) = w (w P_u × P_v + w_v P × P_u + w_u P_v × P)`, so the
//! bracket, of degree `(3p − 1, 3q − 1)`, points the same way as `N` (the
//! weights are positive). Either way `N` is held exactly as a grid of
//! Bernstein coefficient vectors.
//!
//! A box of a piece is certified when every coefficient vector has a positive
//! component along one reference direction (the normal at the box's centre):
//! `N · n` is then a sum of non-negative Bernstein basis functions with
//! positive weights, positive everywhere on the box, so `N` neither vanishes
//! there nor reverses. A box that is not certified is halved (de Casteljau on
//! its longer side) and its halves judged in turn; the coefficients converge
//! to the values, so a regular surface is certified after a few levels.
//!
//! At the four corners of the domain the normal may vanish legitimately: where
//! two sides meet tangentially (a disc bounded by four arcs) the corner is a
//! parameter singularity of a regular sheet. There the corner coefficient
//! (the normal's exact value at the corner) may be zero, and so may others
//! beside it where the tangency is of higher order; the box is certified when
//! no coefficient is negative, every other corner's is positive and each side
//! of the box has a positive one, which keeps `N · n` positive everywhere in
//! the box except at that corner.
//!
//! Where the interior knot lines leave the surface only C0, the normals on the
//! two sides of the line are compared the same way: `N⁻ · N⁺` along the line
//! is a polynomial whose coefficients must be positive (a crease that turns
//! back by more than a right angle is a fold).
//!
//! # Refusals
//!
//! A box that cannot be certified is searched depth first for a witness, a
//! point where the surface demonstrably fails:
//!
//! - [`Irregularity::Collapses`]: the normal at a box corner vanishes
//!   against the derivatives there (`|S_u × S_v| ≤ 1e-9 |S_u| |S_v|`);
//! - [`Irregularity::FoldsOver`]: the exact normals at two corners of a box
//!   no larger than 1/1024 of its piece each way point into opposite half
//!   spaces (or across a C0 knot line, the two sides' normals do);
//! - [`Irregularity::Uncertified`]: neither a certificate nor a witness within
//!   the subdivision budget, or an input the method does not take (an
//!   unclamped knot vector, a non-positive weight). Never a silent pass.

use crate::KnotDirection;
use crate::{NurbsSurface, Vec3};

/// Why a surface is not certified regular. Points are in the surface's own
/// parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum Irregularity {
    /// The normal vanishes at `(u, v)`.
    Collapses { u: f64, v: f64 },
    /// The normal turns back near `(u, v)`.
    FoldsOver { u: f64, v: f64 },
    /// Neither certified nor refuted near `(u, v)`; `reason` says why.
    Uncertified { u: f64, v: f64, reason: String },
}

impl std::fmt::Display for Irregularity {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Collapses { u, v } => write!(f, "the normal vanishes at ({u:.6}, {v:.6})"),
            Self::FoldsOver { u, v } => write!(f, "the normal turns back near ({u:.6}, {v:.6})"),
            Self::Uncertified { u, v, reason } => write!(f, "regularity not certified near ({u:.6}, {v:.6}): {reason}"),
        }
    }
}

/// What a certificate cost: Bézier pieces, boxes judged, C0 knot lines checked.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RegularityReport {
    pub pieces: usize,
    pub boxes: usize,
    pub creases: usize,
}

/// Boxes judged before a surface is reported uncertified.
const BOX_BUDGET: usize = 400_000;
/// Halvings of one box's side before it is reported uncertified.
const MAX_DEPTH: u32 = 40;
/// A box this small (fraction of its piece, each way) whose corner normals
/// oppose is a fold.
const WITNESS_SIZE: f64 = 1.0 / 1024.0;
/// A corner normal this small against its piece's largest coefficient is a
/// collapse (a domain corner's may be).
const VANISHING: f64 = 1e-9;
/// Roundoff margin of a positive coefficient, against the box's largest.
const MARGIN: f64 = 1e-13;

/// Certify `surface` regular (see the module notes), or name where it fails.
pub fn certify_regular(surface: &NurbsSurface) -> Result<RegularityReport, Irregularity> {
    let uncertified = |reason: String| Irregularity::Uncertified { u: f64::NAN, v: f64::NAN, reason };
    let [u0, u1] = surface.domain_u().map_err(&uncertified)?;
    let [v0, v1] = surface.domain_v().map_err(&uncertified)?;
    let (p, q) = (surface.degree_u, surface.degree_v);
    if p == 0 || q == 0 {
        return Err(uncertified(format!("degree ({p}, {q}) has no normal")));
    }
    for (knots, degree, name) in [(&surface.knots_u, p, "u"), (&surface.knots_v, q, "v")] {
        let n = knots.len();
        if knots[..=degree].iter().any(|k| *k != knots[0]) || knots[n - degree - 1..].iter().any(|k| *k != knots[n - 1]) {
            return Err(uncertified(format!("the {name} knot vector is not clamped")));
        }
    }
    let rational = {
        let w0 = surface.control_points[0][0].w;
        surface.control_points.iter().flatten().any(|point| point.w != w0)
    };
    if surface.control_points.iter().flatten().any(|point| !(point.w > 0.0) || !point.w.is_finite()) {
        return Err(uncertified("a weight is not positive".into()));
    }
    // Every interior knot to multiplicity = degree: the net is then the Bézier
    // pieces' nets side by side.
    let (breaks_u, insert_u) = breaks(&surface.knots_u, p, u0, u1);
    let (breaks_v, insert_v) = breaks(&surface.knots_v, q, v0, v1);
    let refined = surface
        .refine_knots(KnotDirection::U, &insert_u)
        .and_then(|refined| refined.refine_knots(KnotDirection::V, &insert_v))
        .map_err(|error| uncertified(error.to_string()))?;
    let (pieces_u, pieces_v) = (breaks_u.len() - 1, breaks_v.len() - 1);
    if refined.control_points.len() != pieces_u * p + 1 || refined.control_points.iter().any(|row| row.len() != pieces_v * q + 1) {
        return Err(uncertified("the refined net does not split into Bézier pieces".into()));
    }
    let mut report = RegularityReport::default();
    let mut normals: Vec<Vec<Grid<Vec3>>> = Vec::with_capacity(pieces_u);
    for a in 0..pieces_u {
        let mut row = Vec::with_capacity(pieces_v);
        for b in 0..pieces_v {
            let net: Vec<Vec<crate::Vec4>> =
                (0..=p).map(|i| refined.control_points[a * p + i][b * q..=b * q + q].to_vec()).collect();
            let normal = piece_normal(&net, p, q, rational);
            let piece = Piece {
                u: [breaks_u[a], breaks_u[a + 1]],
                v: [breaks_v[b], breaks_v[b + 1]],
                corners: [
                    a == 0 && b == 0,
                    a == pieces_u - 1 && b == 0,
                    a == 0 && b == pieces_v - 1,
                    a == pieces_u - 1 && b == pieces_v - 1,
                ],
            };
            certify_piece(surface, &normal, &piece, &mut report)?;
            report.pieces += 1;
            row.push(normal);
        }
        normals.push(row);
    }
    // C0 knot lines: the two sides' normals must not turn back across them.
    let multiplicity = |knots: &[f64], value: f64| knots.iter().filter(|k| **k == value).count();
    for a in 0..pieces_u.saturating_sub(1) {
        if multiplicity(&surface.knots_u, breaks_u[a + 1]) < p {
            continue;
        }
        for b in 0..pieces_v {
            let (before, after) = (&normals[a][b], &normals[a + 1][b]);
            let left: Vec<Vec3> = (0..=before.n).map(|j| before.at(before.m, j)).collect();
            let right: Vec<Vec3> = (0..=after.n).map(|j| after.at(0, j)).collect();
            report.creases += 1;
            certify_crease(&left, &right, breaks_v[b], breaks_v[b + 1])
                .map_err(|t| Irregularity::FoldsOver { u: breaks_u[a + 1], v: t })?;
        }
    }
    for b in 0..pieces_v.saturating_sub(1) {
        if multiplicity(&surface.knots_v, breaks_v[b + 1]) < q {
            continue;
        }
        for a in 0..pieces_u {
            let (before, after) = (&normals[a][b], &normals[a][b + 1]);
            let below: Vec<Vec3> = (0..=before.m).map(|i| before.at(i, before.n)).collect();
            let above: Vec<Vec3> = (0..=after.m).map(|i| after.at(i, 0)).collect();
            report.creases += 1;
            certify_crease(&below, &above, breaks_u[a], breaks_u[a + 1])
                .map_err(|s| Irregularity::FoldsOver { u: s, v: breaks_v[b + 1] })?;
        }
    }
    Ok(report)
}

/// The distinct knot values of the domain (its breakpoints) and the knots to
/// insert so every interior one has multiplicity `degree`.
fn breaks(knots: &[f64], degree: usize, start: f64, end: f64) -> (Vec<f64>, Vec<f64>) {
    let mut values: Vec<f64> = Vec::new();
    let mut insert = Vec::new();
    for &knot in knots {
        if knot < start || knot > end || values.last() == Some(&knot) {
            continue;
        }
        values.push(knot);
        if knot > start && knot < end {
            let have = knots.iter().filter(|k| **k == knot).count();
            insert.extend(std::iter::repeat(knot).take(degree.saturating_sub(have)));
        }
    }
    (values, insert)
}

/// A Bézier piece's place: its parameter box and which of its corners
/// (`(0,0)`, `(1,0)`, `(0,1)`, `(1,1)` in its own coordinates) are corners of
/// the surface's domain.
struct Piece {
    u: [f64; 2],
    v: [f64; 2],
    corners: [bool; 4],
}

/// A tensor-product Bernstein polynomial of degree `(m, n)` on the unit square:
/// coefficients `c[i (n + 1) + j]`.
#[derive(Clone, Debug)]
struct Grid<T> {
    m: usize,
    n: usize,
    c: Vec<T>,
}

trait Linear: Copy {
    fn zero() -> Self;
    fn plus(self, other: Self) -> Self;
    fn times(self, factor: f64) -> Self;
}

impl Linear for f64 {
    fn zero() -> Self {
        0.0
    }
    fn plus(self, other: Self) -> Self {
        self + other
    }
    fn times(self, factor: f64) -> Self {
        self * factor
    }
}

impl Linear for Vec3 {
    fn zero() -> Self {
        Vec3::new(0.0, 0.0, 0.0)
    }
    fn plus(self, other: Self) -> Self {
        self.add(other)
    }
    fn times(self, factor: f64) -> Self {
        self.scale(factor)
    }
}

fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k);
    (0..k).fold(1.0, |value, i| value * (n - i) as f64 / (i + 1) as f64)
}

impl<T: Linear> Grid<T> {
    fn at(&self, i: usize, j: usize) -> T {
        self.c[i * (self.n + 1) + j]
    }

    /// `∂/∂u` (scaled to the unit square), degree `(m − 1, n)`.
    fn du(&self) -> Grid<T> {
        let (m, n) = (self.m, self.n);
        let c = (0..m)
            .flat_map(|i| (0..=n).map(move |j| (i, j)))
            .map(|(i, j)| self.at(i + 1, j).plus(self.at(i, j).times(-1.0)).times(m as f64))
            .collect();
        Grid { m: m - 1, n, c }
    }

    /// `∂/∂v` (scaled to the unit square), degree `(m, n − 1)`.
    fn dv(&self) -> Grid<T> {
        let (m, n) = (self.m, self.n);
        let c = (0..=m)
            .flat_map(|i| (0..n).map(move |j| (i, j)))
            .map(|(i, j)| self.at(i, j + 1).plus(self.at(i, j).times(-1.0)).times(n as f64))
            .collect();
        Grid { m, n: n - 1, c }
    }

    fn plus(&self, other: &Grid<T>) -> Grid<T> {
        debug_assert!(self.m == other.m && self.n == other.n);
        Grid { m: self.m, n: self.n, c: self.c.iter().zip(&other.c).map(|(a, b)| a.plus(*b)).collect() }
    }

    /// The two halves at `u = 1/2` (`along_u`) or `v = 1/2`, by de Casteljau.
    fn halves(&self, along_u: bool) -> (Grid<T>, Grid<T>) {
        let (m, n) = (self.m, self.n);
        let mut low = self.clone();
        let mut high = self.clone();
        let (count, length) = if along_u { (n + 1, m + 1) } else { (m + 1, n + 1) };
        let index = |line: usize, k: usize| if along_u { k * (n + 1) + line } else { line * (n + 1) + k };
        let mut work = vec![T::zero(); length];
        for line in 0..count {
            for (k, slot) in work.iter_mut().enumerate() {
                *slot = self.c[index(line, k)];
            }
            for level in 0..length {
                low.c[index(line, level)] = work[0];
                high.c[index(line, length - 1 - level)] = work[length - 1 - level];
                for k in 0..length - 1 - level {
                    work[k] = work[k].plus(work[k + 1]).times(0.5);
                }
            }
        }
        (low, high)
    }

    /// The value at `(s, t)` of the unit square, by de Casteljau.
    fn value(&self, s: f64, t: f64) -> T {
        let rows: Vec<T> = (0..=self.m)
            .map(|i| casteljau(&(0..=self.n).map(|j| self.at(i, j)).collect::<Vec<_>>(), t))
            .collect();
        casteljau(&rows, s)
    }
}

fn casteljau<T: Linear>(points: &[T], t: f64) -> T {
    let mut work = points.to_vec();
    for level in 1..work.len() {
        for k in 0..work.len() - level {
            work[k] = work[k].times(1.0 - t).plus(work[k + 1].times(t));
        }
    }
    work[0]
}

/// The Bernstein product of two grids, combining coefficients by `combine`.
fn product<A: Linear, B: Linear, C: Linear>(a: &Grid<A>, b: &Grid<B>, combine: impl Fn(A, B) -> C) -> Grid<C> {
    let (m, n) = (a.m + b.m, a.n + b.n);
    let mut c = vec![C::zero(); (m + 1) * (n + 1)];
    for i in 0..=a.m {
        for j in 0..=a.n {
            for k in 0..=b.m {
                for l in 0..=b.n {
                    let factor = binomial(a.m, i) * binomial(b.m, k) / binomial(m, i + k) * binomial(a.n, j)
                        * binomial(b.n, l)
                        / binomial(n, j + l);
                    let slot = &mut c[(i + k) * (n + 1) + j + l];
                    *slot = slot.plus(combine(a.at(i, j), b.at(k, l)).times(factor));
                }
            }
        }
    }
    Grid { m, n, c }
}

/// The Bernstein coefficients of a direction of the normal of the Bézier
/// piece with homogeneous net `net` (see the module notes).
fn piece_normal(net: &[Vec<crate::Vec4>], p: usize, q: usize, rational: bool) -> Grid<Vec3> {
    let cells = |f: &dyn Fn(&crate::Vec4) -> Vec3| Grid {
        m: p,
        n: q,
        c: net.iter().flat_map(|row| row.iter().map(f)).collect::<Vec<Vec3>>(),
    };
    if !rational {
        let points = cells(&|point| Vec3::new(point.x / point.w, point.y / point.w, point.z / point.w));
        return product(&points.du(), &points.dv(), Vec3::cross);
    }
    let points = cells(&|point| Vec3::new(point.x, point.y, point.z));
    let weights = Grid { m: p, n: q, c: net.iter().flat_map(|row| row.iter().map(|point| point.w)).collect::<Vec<f64>>() };
    let (pu, pv) = (points.du(), points.dv());
    let first = product(&weights, &product(&pu, &pv, Vec3::cross), |w, n| n.scale(w));
    let second = product(&weights.dv(), &product(&points, &pu, Vec3::cross), |w, n| n.scale(w));
    let third = product(&weights.du(), &product(&pv, &points, Vec3::cross), |w, n| n.scale(w));
    first.plus(&second).plus(&third)
}

/// One box of a piece in the piece's unit square: `[s0, s1] × [t0, t1]`.
struct Cell {
    normal: Grid<Vec3>,
    s: [f64; 2],
    t: [f64; 2],
    depth: u32,
}

fn certify_piece(
    surface: &NurbsSurface,
    normal: &Grid<Vec3>,
    piece: &Piece,
    report: &mut RegularityReport,
) -> Result<(), Irregularity> {
    let scale = normal.c.iter().map(|c| c.length()).fold(0.0_f64, f64::max);
    let at = |s: f64, t: f64| (piece.u[0] + (piece.u[1] - piece.u[0]) * s, piece.v[0] + (piece.v[1] - piece.v[0]) * t);
    if !(scale > 0.0) || !scale.is_finite() {
        let (u, v) = at(0.5, 0.5);
        return Err(Irregularity::Collapses { u, v });
    }
    // The normal vanishes at a point when it is negligible against the
    // derivatives there (`|S_u × S_v| ≤ 1e-9 |S_u| |S_v|`, a vanishing
    // derivative included): the sine of their angle, not the piece's scale,
    // so a sheet whose speed merely dips is not called collapsed.
    let vanishes = |s: f64, t: f64| {
        let (u, v) = at(s, t);
        match surface.deriv1(u, v) {
            Ok((_, su, sv)) => !(su.cross(sv).length() > VANISHING * su.length() * sv.length()),
            Err(_) => true,
        }
    };
    let mut stack = vec![Cell { normal: normal.clone(), s: [0.0, 1.0], t: [0.0, 1.0], depth: 0 }];
    while let Some(cell) = stack.pop() {
        report.boxes += 1;
        let grid = &cell.normal;
        let (m, n) = (grid.m, grid.n);
        // Which of this box's corners are corners of the domain (the normal
        // may vanish there).
        let corner_index = [(0, 0), (m, 0), (0, n), (m, n)];
        let corner_st = |k: usize| (if k % 2 == 0 { cell.s[0] } else { cell.s[1] }, if k < 2 { cell.t[0] } else { cell.t[1] });
        let exempt = |k: usize| {
            let (s, t) = ([cell.s[0], cell.s[1], cell.s[0], cell.s[1]][k], [cell.t[0], cell.t[0], cell.t[1], cell.t[1]][k]);
            let piece_corner = match (s, t) {
                (s, t) if s == 0.0 && t == 0.0 => Some(0),
                (s, t) if s == 1.0 && t == 0.0 => Some(1),
                (s, t) if s == 0.0 && t == 1.0 => Some(2),
                (s, t) if s == 1.0 && t == 1.0 => Some(3),
                _ => None,
            };
            piece_corner.is_some_and(|c| piece.corners[c])
        };
        let mut reference = grid.value(0.5, 0.5);
        if !(reference.length() > VANISHING * scale) {
            reference = grid.c.iter().fold(Vec3::new(0.0, 0.0, 0.0), |sum, c| sum.add(*c));
        }
        let largest = grid.c.iter().map(|c| c.length()).fold(0.0_f64, f64::max);
        if reference.length() > 0.0 && largest > 0.0 {
            let direction = reference.scale(1.0 / reference.length());
            let margin = MARGIN * largest;
            let dots: Vec<f64> = grid.c.iter().map(|c| c.dot(direction)).collect();
            let certified = if dots.iter().all(|d| *d > margin) {
                true
            } else {
                // A box at a vanishing domain corner: N . n >= 0 everywhere
                // and > 0 away from that corner when no coefficient is
                // negative, every other corner is positive and each side of
                // the box has a positive coefficient (which is positive
                // along that side's interior and across the box's).
                let corner_ok = (0..4).all(|k| {
                    let (ci, cj) = corner_index[k];
                    dots[ci * (n + 1) + cj] > margin || (exempt(k) && vanishes(corner_st(k).0, corner_st(k).1))
                });
                let side_ok = |side: &dyn Fn(usize) -> Option<usize>| (0..=m.max(n)).filter_map(side).any(|index| dots[index] > margin);
                corner_ok
                    && dots.iter().all(|d| *d >= -margin)
                    && side_ok(&|j| (j <= n).then_some(j))
                    && side_ok(&|j| (j <= n).then_some(m * (n + 1) + j))
                    && side_ok(&|i| (i <= m).then_some(i * (n + 1)))
                    && side_ok(&|i| (i <= m).then_some(i * (n + 1) + n))
            };
            if certified {
                continue;
            }
        }
        // A witness at the box's corners: the exact normals there.
        let corner_point = |k: usize| {
            let s = if k % 2 == 0 { cell.s[0] } else { cell.s[1] };
            let t = if k < 2 { cell.t[0] } else { cell.t[1] };
            at(s, t)
        };
        let corners: Vec<(usize, Vec3)> = (0..4).filter(|&k| !exempt(k)).map(|k| (k, grid.at(corner_index[k].0, corner_index[k].1))).collect();
        if let Some(&(k, _)) = corners.iter().find(|(k, _)| vanishes(corner_st(*k).0, corner_st(*k).1)) {
            let (u, v) = corner_point(k);
            return Err(Irregularity::Collapses { u, v });
        }
        let small = cell.s[1] - cell.s[0] <= WITNESS_SIZE && cell.t[1] - cell.t[0] <= WITNESS_SIZE;
        if small && corners.iter().any(|(_, a)| corners.iter().any(|(_, b)| a.dot(*b) < 0.0)) {
            let (u, v) = at(0.5 * (cell.s[0] + cell.s[1]), 0.5 * (cell.t[0] + cell.t[1]));
            return Err(Irregularity::FoldsOver { u, v });
        }
        if cell.depth >= MAX_DEPTH || report.boxes >= BOX_BUDGET {
            let (u, v) = at(0.5 * (cell.s[0] + cell.s[1]), 0.5 * (cell.t[0] + cell.t[1]));
            let reason = if cell.depth >= MAX_DEPTH { "the subdivision reached its depth limit" } else { "the subdivision budget is spent" };
            return Err(Irregularity::Uncertified { u, v, reason: reason.into() });
        }
        let along_u = cell.s[1] - cell.s[0] >= cell.t[1] - cell.t[0];
        let (low, high) = grid.halves(along_u);
        let (first, second) = if along_u {
            let mid = 0.5 * (cell.s[0] + cell.s[1]);
            (
                Cell { normal: low, s: [cell.s[0], mid], t: cell.t, depth: cell.depth + 1 },
                Cell { normal: high, s: [mid, cell.s[1]], t: cell.t, depth: cell.depth + 1 },
            )
        } else {
            let mid = 0.5 * (cell.t[0] + cell.t[1]);
            (
                Cell { normal: low, s: cell.s, t: [cell.t[0], mid], depth: cell.depth + 1 },
                Cell { normal: high, s: cell.s, t: [mid, cell.t[1]], depth: cell.depth + 1 },
            )
        };
        stack.push(second);
        stack.push(first);
    }
    Ok(())
}

/// `before · after > 0` along a C0 knot line whose two sides' normals have
/// the univariate Bernstein coefficients `before` and `after` over
/// `[start, end]`; `Err` carries where they turn back.
fn certify_crease(before: &[Vec3], after: &[Vec3], start: f64, end: f64) -> Result<(), f64> {
    let to_grid = |c: &[Vec3]| Grid { m: 0, n: c.len() - 1, c: c.to_vec() };
    let dot = product(&to_grid(before), &to_grid(after), |a, b| a.dot(b));
    let mut stack = vec![(dot, [0.0_f64, 1.0_f64], 0_u32)];
    while let Some((grid, range, depth)) = stack.pop() {
        let largest = grid.c.iter().map(|c| c.abs()).fold(0.0_f64, f64::max);
        if grid.c.iter().all(|c| *c > MARGIN * largest) {
            continue;
        }
        let place = |r: f64| start + (end - start) * r;
        // the exact values at the ends
        if let Some(r) = [(grid.c[0], range[0]), (grid.c[grid.n], range[1])].iter().find(|(c, _)| *c <= 0.0).map(|(_, r)| *r) {
            return Err(place(r));
        }
        if depth >= MAX_DEPTH {
            return Err(place(0.5 * (range[0] + range[1])));
        }
        let mid = 0.5 * (range[0] + range[1]);
        let (low, high) = grid.halves(false);
        stack.push((high, [mid, range[1]], depth + 1));
        stack.push((low, [range[0], mid], depth + 1));
    }
    Ok(())
}

