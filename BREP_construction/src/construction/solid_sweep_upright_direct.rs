//! The upright sweep built directly, face by face, with no boolean
//! (`BREP_SWEEP_UPRIGHT_DIRECT`, default on; `=0` refuses a turn tighter than
//! the tool as before).
//!
//! The solid is `S = { p : d(p) <= R(h) }` (the planar distance `d` to the
//! cubic, the tool's radius `R` at the height `h`; see the parent module).
//! Its boundary lies on four kinds of sheet:
//! - the offset sheets `c(t) + s R(v) n(t) + z(v) up`, one per side `s = ±1`
//!   of the curve and per side piece of the meridian (`v` its parameter):
//!   exact tensor NURBS once the unit normal `n(t)` is fitted (the cubic and
//!   the meridian enter exactly);
//! - the half tools at the curve's ends (the side pieces turned half a turn
//!   about the upright through `c(0)`, `c(1)`);
//! - the floor and the top (the meridian's flat ends).
//!
//! At one height the section's outline is a cycle of RUNS (stretches of one
//! offset sheet that no other foot covers) joined at CORNERS (two sheet
//! points that coincide) and at the caps. Between EVENT heights (a fold's
//! pivot born where the radius reaches a lobe's radius of curvature, two
//! corners merging as the run between them vanishes, a hole closing to a
//! point) that cycle keeps its form, so each band between events is a stack
//! of faces: one per run on its sheet, bounded by the corners' traced curves
//! (fitted through exact solutions, with the pcurves through the same
//! parameters) and the levels' iso curves. Events are found by reading the
//! outline at sampled heights and solved exactly; any other change declines.
use super::{dot, sub, world_piece, UprightSlot};
use crate::topology::{CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord};
use crate::{interpolate_curve, make_plane, BodyKind, BrepSolid, KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OutlinePiece, Vec3, Vec4};

/// The switch for the direct build in [`UprightSlot::solid`] (default on).
pub fn upright_direct_on() -> bool {
    crate::switch!("BREP_SWEEP_UPRIGHT_DIRECT", true)
}

fn refuse(what: &'static str, message: String) -> KernelRefusal {
    KernelRefusal::unsupported(KernelStage::Collect, what, format!("uprightDirect: {message}"))
}

fn geo<T>(result: Result<T, String>, what: &'static str) -> Result<T, KernelRefusal> {
    result.map_err(|e| refuse(what, e))
}

/// Samples of t per side when reading an outline.
const OUTLINE_SAMPLES: usize = 6000;
/// Heights read per side piece when looking for events.
const EVENT_SCAN: usize = 48;
/// Samples along a corner's traced curve.
const CORNER_SAMPLES: usize = 48;

/// A side piece of the meridian: its curve in (radius, 0, height) and domain.
struct Piece {
    curve: NurbsCurve,
    domain: [f64; 2],
}

impl Piece {
    fn at(&self, v: f64) -> Result<[f64; 2], String> {
        let p = self.curve.evaluate(v)?;
        Ok([p.x, p.z])
    }
}

/// One stretch of an offset sheet on the outline: side index (0: s = +1,
/// 1: s = -1) and its ends in t.
#[derive(Clone, Copy, Debug)]
struct Run {
    side: usize,
    t: [f64; 2],
}

/// A corner: two sheet points that coincide, (side, t) each.
#[derive(Clone, Copy, Debug)]
struct Corner {
    a: (usize, f64),
    b: (usize, f64),
}

fn sign(side: usize) -> f64 {
    if side == 0 {
        1.0
    } else {
        -1.0
    }
}

/// A clamped uniform cubic basis on [0, 1]: its knots, and the span and the
/// four nonzero functions at t.
fn knots_uniform(spans: usize) -> Vec<f64> {
    let mut k = vec![0.0; 4];
    for i in 1..spans {
        k.push(i as f64 / spans as f64);
    }
    k.extend([1.0; 4]);
    k
}

fn basis(knots: &[f64], t: f64) -> (usize, [f64; 4]) {
    let n = knots.len() - 4; // control count
    let mut span = 3;
    while span + 1 < n && knots[span + 1] <= t {
        span += 1;
    }
    let mut left = [0.0; 4];
    let mut right = [0.0; 4];
    let mut b = [1.0, 0.0, 0.0, 0.0];
    for j in 1..=3 {
        left[j] = t - knots[span + 1 - j];
        right[j] = knots[span + j] - t;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = b[r] / (right[r + 1] + left[j - r]);
            b[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        b[j] = saved;
    }
    (span, b)
}

/// Solves the square system `a x = y` (columns of `y` at once), partial pivoting.
fn solve(mut a: Vec<Vec<f64>>, mut y: Vec<Vec<f64>>) -> Result<Vec<Vec<f64>>, String> {
    let n = a.len();
    for col in 0..n {
        let pivot = (col..n).max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs())).ok_or("empty system")?;
        if !(a[pivot][col].abs() > 1e-300) {
            return Err("a singular fit".into());
        }
        a.swap(col, pivot);
        y.swap(col, pivot);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            if f != 0.0 {
                for k in col..n {
                    a[row][k] -= f * a[col][k];
                }
                for k in 0..y[row].len() {
                    y[row][k] -= f * y[col][k];
                }
            }
        }
    }
    for col in (0..n).rev() {
        for k in 0..y[col].len() {
            let mut s = y[col][k];
            for j in col + 1..n {
                s -= a[col][j] * y[j][k];
            }
            y[col][k] = s / a[col][col];
        }
    }
    Ok(y)
}

/// Solves a collocation system whose rows hold at most four neighbouring
/// nonzeros (B-spline interpolation at Greville points: no pivoting needed).
fn solve_banded(mut a: Vec<Vec<f64>>, mut y: Vec<Vec<f64>>) -> Result<Vec<Vec<f64>>, String> {
    let n = a.len();
    for col in 0..n {
        let p = a[col][col];
        if !(p.abs() > 1e-300) {
            return Err("a singular fit".into());
        }
        for row in col + 1..(col + 4).min(n) {
            let f = a[row][col] / p;
            if f != 0.0 {
                for k in col..(col + 4).min(n) {
                    a[row][k] -= f * a[col][k];
                }
                for k in 0..y[row].len() {
                    y[row][k] -= f * y[col][k];
                }
            }
        }
    }
    for col in (0..n).rev() {
        for k in 0..y[col].len() {
            let mut s = y[col][k];
            for j in col + 1..(col + 4).min(n) {
                s -= a[col][j] * y[j][k];
            }
            y[col][k] = s / a[col][col];
        }
    }
    Ok(y)
}

/// The cubic and its fitted unit normal in one clamped cubic basis on [0, 1]:
/// `c_i` (exact: a cubic lies in the space) and `n_i`.
struct Fitted {
    knots: Vec<f64>,
    c: Vec<[f64; 2]>,
    n: Vec<[f64; 2]>,
}

impl Fitted {
    fn eval(&self, controls: &[[f64; 2]], t: f64) -> [f64; 2] {
        let (span, b) = basis(&self.knots, t);
        let mut p = [0.0, 0.0];
        for (k, w) in b.iter().enumerate() {
            let q = controls[span - 3 + k];
            p[0] += w * q[0];
            p[1] += w * q[1];
        }
        p
    }
}

/// The cubic and its unit normal over [a, b] in a clamped cubic basis, spans
/// doubled until the normal holds to 1e-10 (the sheets within 1e-10 of the
/// tool's width; the cubic lies in the space).
fn fit_on(slot: &UprightSlot, a: f64, b: f64, scale: f64) -> Result<Fitted, KernelRefusal> {
    let mut spans = 8;
    loop {
        let knots: Vec<f64> = knots_uniform(spans).iter().map(|k| if *k == 0.0 { a } else if *k == 1.0 { b } else { a + (b - a) * k }).collect();
        let count = spans + 3;
        let greville: Vec<f64> = (0..count).map(|i| (knots[i + 1] + knots[i + 2] + knots[i + 3]) / 3.0).collect();
        let mut m = vec![vec![0.0; count]; count];
        let mut y = vec![vec![0.0; 4]; count];
        for (i, &g) in greville.iter().enumerate() {
            let (span, w) = basis(&knots, g);
            for k in 0..4 {
                m[i][span - 3 + k] = w[k];
            }
            let (p, d) = (slot.point(g), slot.derivative(g));
            let l = dot(d, d).sqrt();
            y[i] = vec![p[0], p[1], -d[1] / l, d[0] / l];
        }
        let x = geo(solve_banded(m, y), "upright_direct_fit")?;
        let fit = Fitted { knots, c: x.iter().map(|r| [r[0], r[1]]).collect(), n: x.iter().map(|r| [r[2], r[3]]).collect() };
        let mut err = 0.0f64;
        for k in 0..=20 * spans {
            let t = a + (b - a) * k as f64 / (20 * spans) as f64;
            let (p, d) = (slot.point(t), slot.derivative(t));
            let l = dot(d, d).sqrt();
            let (cp, np) = (fit.eval(&fit.c, t), fit.eval(&fit.n, t));
            err = err.max((cp[0] - p[0]).abs().max((cp[1] - p[1]).abs()) / scale).max((np[0] + d[1] / l).abs().max((np[1] - d[0] / l).abs()));
        }
        if err <= 1e-10 {
            return Ok(fit);
        }
        if spans >= 8192 {
            return Err(refuse("upright_direct_fit", format!("the curve's normal does not fit to 1e-10 ({err:e} at {spans} spans)")));
        }
        spans *= 2;
    }
}

/// The geometry the build reads: the slot's 2D curve, its pieces and sheets.
struct Build<'a> {
    slot: &'a UprightSlot,
    pieces: Vec<Piece>,
    floor: f64,
    top: f64,
    fit: Fitted,
    /// The tool's widest radius (the length scale).
    scale: f64,
}

impl<'a> Build<'a> {
    fn normal(&self, t: f64) -> [f64; 2] {
        let d = self.slot.derivative(t);
        let l = dot(d, d).sqrt();
        [-d[1] / l, d[0] / l]
    }
    /// Signed curvature (positive turning left).
    fn curvature(&self, t: f64) -> f64 {
        let (d, dd) = (self.slot.derivative(t), self.slot.second(t));
        (d[0] * dd[1] - d[1] * dd[0]) / dot(d, d).powf(1.5)
    }
    fn sheet(&self, side: usize, t: f64, r: f64) -> [f64; 2] {
        let (p, n) = (self.slot.point(t), self.normal(t));
        [p[0] + sign(side) * r * n[0], p[1] + sign(side) * r * n[1]]
    }
    /// d/dt of the offset point: c'(t) (1 - s r κ).
    fn sheet_speed(&self, side: usize, t: f64, r: f64) -> [f64; 2] {
        let d = self.slot.derivative(t);
        let f = 1.0 - sign(side) * r * self.curvature(t);
        [d[0] * f, d[1] * f]
    }
    fn radius(&self, piece: usize, v: f64) -> Result<f64, String> {
        Ok(self.pieces[piece].at(v)?[0])
    }
    fn free(&self, side: usize, t: f64, r: f64) -> bool {
        self.slot.distance(self.sheet(side, t, r)).0 >= r * (1.0 - 1e-9)
    }

    /// The outline's runs at radius r, read on a grid of t (each end then
    /// solved to its corner, or 0/1 at a cap).
    fn runs(&self, r: f64) -> Vec<Run> {
        let mut out = Vec::new();
        for side in 0..2 {
            let n = OUTLINE_SAMPLES;
            let free: Vec<bool> = (0..=n).map(|k| self.free(side, k as f64 / n as f64, r)).collect();
            let mut k = 0;
            while k <= n {
                if free[k] {
                    let mut j = k;
                    while j < n && free[j + 1] {
                        j += 1;
                    }
                    if j > k {
                        // the ends between the last free and first covered samples
                        let lo = if k == 0 { 0.0 } else { self.edge_of(side, (k - 1) as f64 / n as f64, k as f64 / n as f64, r) };
                        let hi = if j == n { 1.0 } else { self.edge_of(side, (j + 1) as f64 / n as f64, j as f64 / n as f64, r) };
                        out.push(Run { side, t: [lo, hi] });
                    }
                    k = j + 1;
                } else {
                    k += 1;
                }
            }
        }
        out
    }
    /// Where coverage changes between a covered t and a free t (bisection).
    fn edge_of(&self, side: usize, covered: f64, freed: f64, r: f64) -> f64 {
        let (mut a, mut b) = (covered, freed);
        for _ in 0..40 {
            let m = 0.5 * (a + b);
            if self.free(side, m, r) {
                b = m;
            } else {
                a = m;
            }
        }
        b
    }

    /// Solves the corner (sa, ta) ~ (sb, tb) at radius r by Newton.
    fn corner(&self, c: Corner, r: f64) -> Option<Corner> {
        let ((sa, mut ta), (sb, mut tb)) = (c.a, c.b);
        for _ in 0..60 {
            let (pa, pb) = (self.sheet(sa, ta, r), self.sheet(sb, tb, r));
            let f = sub(pa, pb);
            let (da, db) = (self.sheet_speed(sa, ta, r), self.sheet_speed(sb, tb, r));
            let det = da[0] * (-db[1]) - (-db[0]) * da[1];
            if !(det.abs() > 0.0) {
                return None;
            }
            let dta = (f[0] * (-db[1]) - (-db[0]) * f[1]) / det;
            let dtb = (da[0] * f[1] - f[0] * da[1]) / det;
            ta -= dta;
            tb -= dtb;
            if !(ta.is_finite() && tb.is_finite()) || !(-0.5..=1.5).contains(&ta) || !(-0.5..=1.5).contains(&tb) {
                return None;
            }
            if dta.abs().max(dtb.abs()) <= 1e-15 {
                break;
            }
        }
        let gap = sub(self.sheet(sa, ta, r), self.sheet(sb, tb, r));
        (dot(gap, gap).sqrt() <= 1e-11 * self.scale).then_some(Corner { a: (sa, ta), b: (sb, tb) })
    }
}

/// The outline at one radius: runs, and for each run's two ends what closes
/// it there (a corner partner, or a cap at t = 0/1).
#[derive(Clone, Debug)]
struct Outline {
    runs: Vec<Run>,
    /// For each run, at each end: Some(corner, with this run's point as `a`) or None (a cap).
    ends: Vec<[Option<Corner>; 2]>,
    /// For each run, at each end: the run end it meets (None at a cap).
    partner: Vec<[Option<(usize, usize)>; 2]>,
}

impl Build<'_> {
    /// The outline at radius r: runs from the grid, their open ends paired
    /// closest first, each corner solved.
    fn outline(&self, r: f64) -> Result<Outline, String> {
        let mut runs = self.runs(r);
        let n = runs.len();
        let mut ends: Vec<[Option<Corner>; 2]> = vec![[None, None]; n];
        let mut partner: Vec<[Option<(usize, usize)>; 2]> = vec![[None, None]; n];
        let at = |k: usize, e: usize| self.sheet(runs[k].side, runs[k].t[e], r);
        let open: Vec<(usize, usize)> = (0..n).flat_map(|k| [(k, 0), (k, 1)]).filter(|&(k, e)| !(e == 0 && runs[k].t[0] == 0.0 || e == 1 && runs[k].t[1] == 1.0)).collect();
        let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
        for i in 0..open.len() {
            for j in i + 1..open.len() {
                let g = sub(at(open[i].0, open[i].1), at(open[j].0, open[j].1));
                pairs.push((dot(g, g).sqrt(), i, j));
            }
        }
        pairs.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut taken = vec![false; open.len()];
        let mut solved_pairs = Vec::new();
        for (gap, i, j) in pairs {
            if taken[i] || taken[j] {
                continue;
            }
            if gap > 0.05 * self.scale {
                break;
            }
            taken[i] = true;
            taken[j] = true;
            solved_pairs.push((open[i], open[j]));
        }
        if let Some(k) = taken.iter().position(|t| !t) {
            return Err(format!("a run end at t = {} (side {}) meets nothing within reach", runs[open[k].0].t[open[k].1], runs[open[k].0].side));
        }
        for ((ka, ea), (kb, eb)) in solved_pairs {
            let seed = Corner { a: (runs[ka].side, runs[ka].t[ea]), b: (runs[kb].side, runs[kb].t[eb]) };
            let solved = self.corner(seed, r).ok_or_else(|| format!("the corner near {:?} does not solve", (seed.a, seed.b)))?;
            ends[ka][ea] = Some(solved);
            ends[kb][eb] = Some(Corner { a: solved.b, b: solved.a });
            partner[ka][ea] = Some((kb, eb));
            partner[kb][eb] = Some((ka, ea));
        }
        for (k, run) in runs.iter_mut().enumerate() {
            for e in 0..2 {
                if let Some(c) = ends[k][e] {
                    run.t[e] = c.a.1;
                }
            }
        }
        Ok(Outline { runs, ends, partner })
    }
}

impl<'a> Build<'a> {
    /// The build's reads for a slot whose meridian is a flat floor from the
    /// axis, side pieces rising, and a flat top back to the axis; None for
    /// any other shape.
    fn new(slot: &'a UprightSlot) -> Result<Self, KernelRefusal> {
        let shape = |m: String| refuse("upright_direct_shape", m);
        let chain = slot.chain().ok_or_else(|| shape("the meridian has no axis piece".into()))?;
        let scale = slot.widest();
        let tol = 1e-12 * scale;
        let flat = |piece: &OutlinePiece| match *piece {
            OutlinePiece::Line { from, to } => (from[1] - to[1]).abs() <= tol,
            _ => false,
        };
        let (Some(first), Some(last)) = (chain.first(), chain.last()) else { return Err(shape("an empty meridian".into())) };
        let (OutlinePiece::Line { from: f0, to: f1 }, OutlinePiece::Line { from: l0, to: l1 }) = (first, last) else {
            return Err(shape("the meridian does not start and end with flat lines".into()));
        };
        if !(flat(first) && flat(last) && f0[0].abs() <= tol && l1[0].abs() <= tol && f1[0] > tol && l0[0] > tol && chain.len() >= 3) {
            return Err(shape("the meridian is not a flat floor, rising sides and a flat top".into()));
        }
        let (floor, top) = (f0[1], l0[1]);
        let mut pieces = Vec::new();
        for piece in &chain[1..chain.len() - 1] {
            let curve = geo(world_piece(piece, Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0)), "upright_direct_piece")?;
            let domain = geo(curve.domain(), "upright_direct_piece")?;
            let piece = Piece { curve, domain };
            let (a, b) = (geo(piece.at(domain[0]), "upright_direct_piece")?, geo(piece.at(domain[1]), "upright_direct_piece")?);
            if !(b[1] > a[1] + tol) || a[0] <= tol || b[0] <= tol {
                return Err(shape("a side piece does not rise away from the axis".into()));
            }
            pieces.push(piece);
        }
        let fit = fit_on(slot, 0.0, 1.0, scale)?;
        Ok(Build { slot, pieces, floor, top, fit, scale })
    }
}

/// What changes the outline at an event height.
#[derive(Clone, Debug)]
enum Event {
    /// Pivots born, each where the radius reaches a lobe's radius of
    /// curvature: the cusps (side, t).
    Birth(Vec<(usize, f64)>),
    /// The run between two corners vanished: (a ~ m) and (m ~ b) became (a ~ b).
    Merge { a: (usize, f64), m: (usize, f64), b: (usize, f64) },
    /// A hole of two runs closed to a point: its last two feet.
    Close { l: (usize, f64), r: (usize, f64) },
}

/// A height where bands meet: the piece and parameter on each side.
#[derive(Clone, Debug)]
struct Level {
    h: f64,
    below: Option<(usize, f64)>,
    above: Option<(usize, f64)>,
    event: Option<Event>,
}

/// Newton on a small square system with central-difference Jacobian.
fn newton(x: &mut [f64], f: &dyn Fn(&[f64]) -> Option<Vec<f64>>, tol: f64) -> bool {
    let n = x.len();
    for _ in 0..80 {
        let Some(f0) = f(x) else { return false };
        if f0.iter().map(|v| v.abs()).fold(0.0, f64::max) <= tol {
            return true;
        }
        let mut jac = vec![vec![0.0; n]; n];
        for k in 0..n {
            let step = 1e-7 * x[k].abs().max(1.0);
            let (mut xp, mut xm) = (x.to_vec(), x.to_vec());
            xp[k] += step;
            xm[k] -= step;
            let (Some(fp), Some(fm)) = (f(&xp), f(&xm)) else { return false };
            for i in 0..n {
                jac[i][k] = (fp[i] - fm[i]) / (2.0 * step);
            }
        }
        let Ok(dx) = solve(jac, f0.iter().map(|v| vec![*v]).collect()) else { return false };
        for k in 0..n {
            x[k] -= dx[k][0];
        }
        if !x.iter().all(|v| v.is_finite()) {
            return false;
        }
    }
    f(x).is_some_and(|f0| f0.iter().map(|v| v.abs()).fold(0.0, f64::max) <= tol)
}

impl Build<'_> {
    fn r_at(&self, piece: usize, v: f64) -> Option<f64> {
        self.radius(piece, v).ok()
    }
    fn counts(o: &Outline) -> [usize; 2] {
        let mut c = [0, 0];
        for run in &o.runs {
            c[run.side] += 1;
        }
        c
    }
    /// The derivative of the signed curvature (the cubic's third derivative is constant).
    fn curvature_slope(&self, t: f64) -> f64 {
        let (d, dd) = (self.slot.derivative(t), self.slot.second(t));
        let c = &self.slot.curve;
        let ddd = [6.0 * (c[3][0] - 3.0 * c[2][0] + 3.0 * c[1][0] - c[0][0]), 6.0 * (c[3][1] - 3.0 * c[2][1] + 3.0 * c[1][1] - c[0][1])];
        let l2 = dot(d, d);
        let cross = |a: [f64; 2], b: [f64; 2]| a[0] * b[1] - a[1] * b[0];
        cross(d, ddd) / l2.powf(1.5) - 3.0 * cross(d, dd) * dot(d, dd) / l2.powf(2.5)
    }

    /// The events in one side piece, in order.
    fn events_in(&self, piece: usize) -> Result<Vec<(f64, Event)>, KernelRefusal> {
        let [a, b] = self.pieces[piece].domain;
        let fail = |m: String| refuse("upright_direct_event", m);
        let read = |v: f64| -> Result<Outline, KernelRefusal> {
            let r = self.r_at(piece, v).ok_or_else(|| fail(format!("no radius at {v}")))?;
            self.outline(r).map_err(|e| fail(format!("the outline at the meridian's {v}: {e}")))
        };
        let mut found = Vec::new();
        let mut prev = (a, read(a)?);
        for i in 1..=EVENT_SCAN {
            let v = a + (b - a) * i as f64 / EVENT_SCAN as f64;
            let next = (v, read(v)?);
            if Self::counts(&prev.1) != Self::counts(&next.1) {
                self.resolve(piece, &prev, &next, 0, &mut found)?;
            }
            prev = next;
        }
        Ok(found)
    }

    fn resolve(&self, piece: usize, lo: &(f64, Outline), hi: &(f64, Outline), depth: usize, found: &mut Vec<(f64, Event)>) -> Result<(), KernelRefusal> {
        let fail = |m: String| refuse("upright_direct_event", m);
        if let Some((v, event)) = self.classify(piece, lo, hi) {
            found.push((v, event));
            return Ok(());
        }
        if depth > 40 {
            return Err(fail(format!("the outline changes between the meridian's {} and {} in a way this build does not read ({:?} runs to {:?})", lo.0, hi.0, Self::counts(&lo.1), Self::counts(&hi.1))));
        }
        let v = 0.5 * (lo.0 + hi.0);
        let r = self.r_at(piece, v).ok_or_else(|| fail(format!("no radius at {v}")))?;
        let mid = (v, self.outline(r).map_err(|e| fail(format!("the outline at the meridian's {v}: {e}")))?);
        if Self::counts(&lo.1) != Self::counts(&mid.1) {
            self.resolve(piece, lo, &mid, depth + 1, found)?;
        }
        if Self::counts(&mid.1) != Self::counts(&hi.1) {
            self.resolve(piece, &mid, hi, depth + 1, found)?;
        }
        Ok(())
    }

    /// One known event between two outlines, solved, with its parameter
    /// inside the bracket; None when the change is not one of them.
    fn classify(&self, piece: usize, lo: &(f64, Outline), hi: &(f64, Outline)) -> Option<(f64, Event)> {
        let (cl, ch) = (Self::counts(&lo.1), Self::counts(&hi.1));
        let r_hi = self.r_at(piece, hi.0)?;
        let inside = |v: f64| v > lo.0 && v < hi.0;
        // births: one side gains runs, each a new pivot (two points of that side close in t)
        for s in 0..2 {
            if ch[s] > cl[s] && ch[1 - s] == cl[1 - s] {
                let m = ch[s] - cl[s];
                let new: Vec<Corner> = hi.1.ends.iter().flatten().flatten().copied().filter(|c| c.a.0 == s && c.b.0 == s && c.a.1 < c.b.1 && c.b.1 - c.a.1 < 0.25).filter(|c| {
                    !lo.1.ends.iter().flatten().flatten().any(|o| o.a.0 == s && o.b.0 == s && (o.a.1 - c.a.1).abs() < 0.02 && (o.b.1 - c.b.1).abs() < 0.02)
                }).collect();
                if new.len() != m {
                    return None;
                }
                let mut cusps = Vec::new();
                let mut heights = Vec::new();
                for c in new {
                    // the lobe's curvature peak between the pivot's two feet
                    let (mut x0, mut x1) = (c.a.1, c.b.1);
                    let (g0, g1) = (self.curvature_slope(x0), self.curvature_slope(x1));
                    if !(g0 * g1 < 0.0) {
                        return None;
                    }
                    for _ in 0..200 {
                        let xm = 0.5 * (x0 + x1);
                        if xm <= x0 || xm >= x1 {
                            break;
                        }
                        if (self.curvature_slope(xm) < 0.0) == (g0 < 0.0) {
                            x0 = xm;
                        } else {
                            x1 = xm;
                        }
                    }
                    let tc = 0.5 * (x0 + x1);
                    let kappa = sign(s) * self.curvature(tc);
                    if !(kappa > 0.0) {
                        return None;
                    }
                    let target = 1.0 / kappa;
                    let (mut v0, mut v1) = (lo.0, hi.0);
                    let (r0, r1) = (self.r_at(piece, v0)?, self.r_at(piece, v1)?);
                    if !((r0 - target) * (r1 - target) <= 0.0) {
                        return None;
                    }
                    for _ in 0..200 {
                        let vm = 0.5 * (v0 + v1);
                        if vm <= v0 || vm >= v1 {
                            break;
                        }
                        if (self.r_at(piece, vm)? < target) == (r0 < target) {
                            v0 = vm;
                        } else {
                            v1 = vm;
                        }
                    }
                    cusps.push((s, tc));
                    heights.push(0.5 * (v0 + v1));
                }
                let v = heights[0];
                if heights.iter().any(|h| (h - v).abs() > 1e-12 * (1.0 + v.abs())) || !inside(v) {
                    return None;
                }
                return Some((v, Event::Birth(cusps)));
            }
        }
        // runs of the lower outline whose middle no longer lies on the outline above
        let gone: Vec<usize> = (0..lo.1.runs.len()).filter(|&k| {
            let run = lo.1.runs[k];
            !self.free(run.side, 0.5 * (run.t[0] + run.t[1]), r_hi)
        }).collect();
        let total = |c: [usize; 2]| c[0] + c[1];
        if total(cl) == total(ch) + 1 && gone.len() == 1 {
            let k = gone[0];
            let (Some(ca), Some(cb)) = (lo.1.ends[k][0], lo.1.ends[k][1]) else { return None };
            let sm = lo.1.runs[k].side;
            let (sa, sb) = (ca.b.0, cb.b.0);
            let mut x = vec![ca.b.1, 0.5 * (lo.1.runs[k].t[0] + lo.1.runs[k].t[1]), cb.b.1, 0.5 * (lo.0 + hi.0)];
            let f = |x: &[f64]| -> Option<Vec<f64>> {
                let r = self.r_at(piece, x[3])?;
                let (pa, pm, pb) = (self.sheet(sa, x[0], r), self.sheet(sm, x[1], r), self.sheet(sb, x[2], r));
                Some(vec![pa[0] - pm[0], pa[1] - pm[1], pm[0] - pb[0], pm[1] - pb[1]])
            };
            if !newton(&mut x, &f, 1e-13 * self.scale) || !inside(x[3]) {
                return None;
            }
            return Some((x[3], Event::Merge { a: (sa, x[0]), m: (sm, x[1]), b: (sb, x[2]) }));
        }
        if total(cl) == total(ch) + 2 && gone.len() == 2 {
            let (k0, k1) = (gone[0], gone[1]);
            // the two runs close a hole: each end meets the other run
            let closed = (0..2).all(|e| lo.1.partner[k0][e].is_some_and(|(k, _)| k == k1));
            if !closed {
                return None;
            }
            let (s0, s1) = (lo.1.runs[k0].side, lo.1.runs[k1].side);
            let mut x = vec![0.5 * (lo.1.runs[k0].t[0] + lo.1.runs[k0].t[1]), 0.5 * (lo.1.runs[k1].t[0] + lo.1.runs[k1].t[1]), lo.0];
            let f = |x: &[f64]| -> Option<Vec<f64>> {
                let r = self.r_at(piece, x[2])?;
                let (p0, p1) = (self.sheet(s0, x[0], r), self.sheet(s1, x[1], r));
                let (d0, d1) = (self.sheet_speed(s0, x[0], r), self.sheet_speed(s1, x[1], r));
                let cross = (d0[0] * d1[1] - d0[1] * d1[0]) / (dot(d0, d0).sqrt() * dot(d1, d1).sqrt());
                Some(vec![p0[0] - p1[0], p0[1] - p1[1], cross * self.scale])
            };
            if !newton(&mut x, &f, 1e-12 * self.scale) || !inside(x[2]) {
                return None;
            }
            return Some((x[2], Event::Close { l: (s0, x[0]), r: (s1, x[1]) }));
        }
        None
    }

    /// Every level: the floor, each event, the joints between side pieces, the top.
    fn levels(&self) -> Result<Vec<Level>, KernelRefusal> {
        let n = self.pieces.len();
        let z = |k: usize, v: f64| -> Result<f64, KernelRefusal> { Ok(geo(self.pieces[k].at(v), "upright_direct_piece")?[1]) };
        let mut levels = vec![Level { h: self.floor, below: None, above: Some((0, self.pieces[0].domain[0])), event: None }];
        for k in 0..n {
            for (v, event) in self.events_in(k)? {
                levels.push(Level { h: z(k, v)?, below: Some((k, v)), above: Some((k, v)), event: Some(event) });
            }
            let end = self.pieces[k].domain[1];
            if k + 1 < n {
                levels.push(Level { h: z(k, end)?, below: Some((k, end)), above: Some((k + 1, self.pieces[k + 1].domain[0])), event: None });
            } else {
                levels.push(Level { h: self.top, below: Some((k, end)), above: None, event: None });
            }
        }
        Ok(levels)
    }
}

/// A band between two levels, on one side piece: its outline (at its middle)
/// and each corner traced over sigma in [0, 1], v(sigma) = v0 + (v1 - v0)(1 - cos(pi sigma)) / 2.
struct Band {
    piece: usize,
    v: [f64; 2],
    /// The levels below and above (indices).
    level: [usize; 2],
    runs: Vec<Run>,
    /// Each run's ends: Some((corner index, role 0 = a / 1 = b)) or None (a cap).
    ends: Vec<[Option<(usize, usize)>; 2]>,
    corners: Vec<Trace>,
}

/// A corner's trace: the two sides, and per sample sigma its two parameters.
struct Trace {
    side: [usize; 2],
    sigma: Vec<f64>,
    t: Vec<[f64; 2]>,
}

fn v_of(v: [f64; 2], sigma: f64) -> f64 {
    v[0] + (v[1] - v[0]) * 0.5 * (1.0 - (std::f64::consts::PI * sigma).cos())
}

impl Build<'_> {
    fn band(&self, levels: &[Level], i: usize) -> Result<Band, KernelRefusal> {
        let fail = |m: String| refuse("upright_direct_band", m);
        let (Some((piece, v0)), Some((piece1, v1))) = (levels[i].above, levels[i + 1].below) else { return Err(fail("a band without its levels".into())) };
        if piece != piece1 {
            return Err(fail("a band across two side pieces".into()));
        }
        let v = [v0, v1];
        let r_mid = self.r_at(piece, v_of(v, 0.5)).ok_or_else(|| fail("no radius".into()))?;
        let o = self.outline(r_mid).map_err(|e| fail(format!("the outline at the band's middle: {e}")))?;
        // corners: one per pair of run ends
        let mut ends: Vec<[Option<(usize, usize)>; 2]> = vec![[None, None]; o.runs.len()];
        let mut corners: Vec<Trace> = Vec::new();
        let k_count = CORNER_SAMPLES;
        let sigma: Vec<f64> = (0..=k_count).map(|k| k as f64 / k_count as f64).collect();
        for k in 0..o.runs.len() {
            for e in 0..2 {
                let (Some(c), Some((kb, eb))) = (o.ends[k][e], o.partner[k][e]) else { continue };
                if ends[k][e].is_some() {
                    continue;
                }
                let index = corners.len();
                ends[k][e] = Some((index, 0));
                ends[kb][eb] = Some((index, 1));
                // march from the middle both ways
                let mut t = vec![[f64::NAN; 2]; k_count + 1];
                let half = k_count / 2;
                t[half] = [c.a.1, c.b.1];
                for range in [((half + 1)..k_count).collect::<Vec<_>>(), (1..half).rev().collect::<Vec<_>>()] {
                    let mut prev = t[half];
                    for j in range {
                        let r = self.r_at(piece, v_of(v, sigma[j])).ok_or_else(|| fail("no radius".into()))?;
                        let solved = self.corner(Corner { a: (c.a.0, prev[0]), b: (c.b.0, prev[1]) }, r).ok_or_else(|| fail(format!("the corner {:?} / {:?} does not follow to sigma {}", c.a, c.b, sigma[j])))?;
                        prev = [solved.a.1, solved.b.1];
                        t[j] = prev;
                    }
                }
                corners.push(Trace { side: [c.a.0, c.b.0], sigma: sigma.clone(), t });
            }
        }
        Ok(Band { piece, v, level: [i, i + 1], runs: o.runs, ends, corners })
    }

    /// Pins a band's corner ends at its two levels: an event's own values for
    /// the corners it involves, Newton from the next sample for the others.
    fn pin(&self, levels: &[Level], band: &mut Band) -> Result<(), KernelRefusal> {
        let fail = |m: String| refuse("upright_direct_pin", m);
        let last = band.corners.first().map_or(CORNER_SAMPLES, |c| c.sigma.len() - 1);
        for end in 0..2 {
            let (j, near) = if end == 0 { (0, 1) } else { (last, last - 1) };
            let level = &levels[band.level[end]];
            let r = self.r_at(band.piece, band.v[end]).ok_or_else(|| fail("no radius".into()))?;
            let mut used = vec![false; band.corners.len()];
            match (&level.event, end) {
                (Some(Event::Birth(cusps)), 0) => {
                    for &(s, tc) in cusps {
                        let k = (0..band.corners.len()).filter(|&k| !used[k] && band.corners[k].side == [s, s]).min_by(|&x, &y| {
                            let d = |k: usize| (band.corners[k].t[near][0] - tc).abs() + (band.corners[k].t[near][1] - tc).abs();
                            d(x).total_cmp(&d(y))
                        });
                        let k = k.ok_or_else(|| fail("a born pivot has no corner in the band above".into()))?;
                        used[k] = true;
                        band.corners[k].t[j] = [tc, tc];
                    }
                }
                (Some(Event::Merge { a, m, b }), 1) => {
                    // below: the corners (a ~ m) and (m ~ b)
                    for (p, q) in [(*a, *m), (*m, *b)] {
                        let k = self.nearest(band, &used, near, p, q).ok_or_else(|| fail("a merging corner is missing below".into()))?;
                        used[k] = true;
                        band.corners[k].t[j] = Self::oriented(&band.corners[k], near, p, q);
                    }
                }
                (Some(Event::Merge { a, b, .. }), 0) => {
                    let k = self.nearest(band, &used, near, *a, *b).ok_or_else(|| fail("the merged corner is missing above".into()))?;
                    used[k] = true;
                    band.corners[k].t[j] = Self::oriented(&band.corners[k], near, *a, *b);
                }
                (Some(Event::Close { l, r: rr }), 1) => {
                    for _ in 0..2 {
                        let k = self.nearest(band, &used, near, *l, *rr).ok_or_else(|| fail("a closing corner is missing below".into()))?;
                        used[k] = true;
                        band.corners[k].t[j] = Self::oriented(&band.corners[k], near, *l, *rr);
                    }
                }
                _ => {}
            }
            for k in 0..band.corners.len() {
                if used[k] {
                    continue;
                }
                let c = &band.corners[k];
                let seed = Corner { a: (c.side[0], c.t[near][0]), b: (c.side[1], c.t[near][1]) };
                let solved = self.corner(seed, r).ok_or_else(|| fail(format!("the corner {:?} does not reach its level at {}", seed, level.h)))?;
                band.corners[k].t[j] = [solved.a.1, solved.b.1];
            }
        }
        Ok(())
    }

    /// The unused corner whose sample `near` lies closest to the pair (p, q) (either way round).
    fn nearest(&self, band: &Band, used: &[bool], near: usize, p: (usize, f64), q: (usize, f64)) -> Option<usize> {
        let cost = |k: usize| {
            let c = &band.corners[k];
            let way = |x: (usize, f64), y: (usize, f64)| if c.side == [x.0, y.0] { (c.t[near][0] - x.1).abs() + (c.t[near][1] - y.1).abs() } else { f64::INFINITY };
            way(p, q).min(way(q, p))
        };
        (0..band.corners.len()).filter(|&k| !used[k] && cost(k).is_finite()).min_by(|&x, &y| cost(x).total_cmp(&cost(y))).filter(|&k| cost(k) < 0.2)
    }
    fn oriented(c: &Trace, near: usize, p: (usize, f64), q: (usize, f64)) -> [f64; 2] {
        let way = |x: (usize, f64), y: (usize, f64)| if c.side == [x.0, y.0] { (c.t[near][0] - x.1).abs() + (c.t[near][1] - y.1).abs() } else { f64::INFINITY };
        if way(p, q) <= way(q, p) {
            [p.1, q.1]
        } else {
            [q.1, p.1]
        }
    }

    /// A run's ends at sample j: from its corners, or 0/1 at a cap.
    fn run_ends(band: &Band, k: usize, j: usize) -> [f64; 2] {
        let mut t = band.runs[k].t;
        for e in 0..2 {
            if let Some((c, role)) = band.ends[k][e] {
                t[e] = band.corners[c].t[j][role];
            }
        }
        t
    }
}

/// A degree-1 curve from a to b (as (x, y, 0)) over [t0, t1].
fn line2(a: [f64; 2], b: [f64; 2], t0: f64, t1: f64) -> Result<NurbsCurve, String> {
    NurbsCurve::new(1, vec![t0, t0, t1, t1], vec![Vec4 { x: a[0], y: a[1], z: 0.0, w: 1.0 }, Vec4 { x: b[0], y: b[1], z: 0.0, w: 1.0 }])
}

/// The part of a curve over [a, b] (exact, by splitting).
fn part(curve: &NurbsCurve, a: f64, b: f64) -> Result<NurbsCurve, String> {
    let [d0, d1] = curve.domain()?;
    let span = (d1 - d0).abs();
    let mut c = curve.clone();
    if a > d0 + 1e-14 * span {
        c = c.split(a)?.1;
    }
    if b < d1 - 1e-14 * span {
        c = c.split(b)?.0;
    }
    Ok(c)
}

/// One use of an edge in a loop, and the pcurve in the edge's own direction.
struct Use {
    edge: usize,
    forward: bool,
    pcurve: NurbsCurve,
}

impl Build<'_> {
    fn lift(&self, p: [f64; 2], h: f64) -> Vec3 {
        self.slot.origin.add(self.slot.x.scale(p[0])).add(self.slot.y.scale(p[1])).add(self.slot.up.scale(h))
    }
    fn rz(&self, piece: usize, v: f64) -> Result<[f64; 2], String> {
        self.pieces[piece].at(v)
    }
    /// The offset sheet of one side over one side piece: c(t) + s R(v) N(t) + z(v) up.
    /// The meridian piece over [v0, v1] widened by a hair (each face's surface spans its own band only).
    fn piece_band(&self, piece: usize, v: [f64; 2]) -> Result<NurbsCurve, String> {
        let d = self.pieces[piece].domain;
        let margin = 1e-3 * (v[1] - v[0]);
        part(&self.pieces[piece].curve, (v[0] - margin).max(d[0]), (v[1] + margin).min(d[1]))
    }
    fn sheet_surface(&self, side: usize, piece: usize, fit: &Fitted, v: [f64; 2]) -> Result<NurbsSurface, String> {
        let m = &self.piece_band(piece, v)?;
        let net: Vec<Vec<Vec4>> = (0..fit.c.len())
            .map(|i| {
                m.control_points
                    .iter()
                    .map(|q| {
                        let (rr, z) = (q.x / q.w, q.z / q.w);
                        let xy = [fit.c[i][0] + sign(side) * rr * fit.n[i][0], fit.c[i][1] + sign(side) * rr * fit.n[i][1]];
                        Vec4::from_point(self.lift(xy, z), q.w)
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(3, m.degree, fit.knots.clone(), m.knots.clone(), net)
    }
    /// The iso curve of a sheet at v (a cubic B-spline in t).
    fn sheet_level(&self, side: usize, piece: usize, v: f64) -> Result<NurbsCurve, String> {
        let [rr, z] = self.rz(piece, v)?;
        let points = (0..self.fit.c.len()).map(|i| Vec4::from_point(self.lift([self.fit.c[i][0] + sign(side) * rr * self.fit.n[i][0], self.fit.c[i][1] + sign(side) * rr * self.fit.n[i][1]], z), 1.0)).collect();
        NurbsCurve::new(3, self.fit.knots.clone(), points)
    }
    /// The iso curve of a sheet at the curve's end t = e (a curve in v).
    fn sheet_seam(&self, side: usize, piece: usize, e: usize) -> Result<NurbsCurve, String> {
        let m = &self.pieces[piece].curve;
        let i = if e == 0 { 0 } else { self.fit.c.len() - 1 };
        let points = m.control_points.iter().map(|q| {
            let (rr, z) = (q.x / q.w, q.z / q.w);
            Vec4::from_point(self.lift([self.fit.c[i][0] + sign(side) * rr * self.fit.n[i][0], self.fit.c[i][1] + sign(side) * rr * self.fit.n[i][1]], z), q.w)
        });
        NurbsCurve::new(m.degree, m.knots.clone(), points.collect())
    }
    /// The iso curve of a sheet at an inner t (a curve in v).
    fn sheet_upright(&self, side: usize, piece: usize, t: f64) -> Result<NurbsCurve, String> {
        let m = &self.pieces[piece].curve;
        let (c, n) = (self.fit.eval(&self.fit.c, t), self.fit.eval(&self.fit.n, t));
        let points = m.control_points.iter().map(|q| {
            let (rr, z) = (q.x / q.w, q.z / q.w);
            Vec4::from_point(self.lift([c[0] + sign(side) * rr * n[0], c[1] + sign(side) * rr * n[1]], z), q.w)
        });
        NurbsCurve::new(m.degree, m.knots.clone(), points.collect())
    }
    /// The half tool at the curve's end e: its foot, and the five control
    /// directions of the half turn from side +1 through the end's outward
    /// tangent to side -1, with their weights (u in [0, 2]).
    fn cap_frame(&self, e: usize) -> ([f64; 2], [[f64; 2]; 5], [f64; 5]) {
        let t = e as f64;
        let i = if e == 0 { 0 } else { self.fit.c.len() - 1 };
        let n = self.fit.n[i];
        let d = self.slot.derivative(t);
        let l = dot(d, d).sqrt();
        let o = if e == 0 { [-d[0] / l, -d[1] / l] } else { [d[0] / l, d[1] / l] };
        let s2 = std::f64::consts::FRAC_1_SQRT_2;
        (self.fit.c[i], [n, [n[0] + o[0], n[1] + o[1]], o, [o[0] - n[0], o[1] - n[1]], [-n[0], -n[1]]], [1.0, s2, 1.0, s2, 1.0])
    }
    fn cap_surface(&self, e: usize, piece: usize, v: [f64; 2]) -> Result<NurbsSurface, String> {
        let m = &self.piece_band(piece, v)?;
        let (foot, dirs, weights) = self.cap_frame(e);
        let net: Vec<Vec<Vec4>> = (0..5)
            .map(|k| {
                m.control_points
                    .iter()
                    .map(|q| {
                        let (rr, z) = (q.x / q.w, q.z / q.w);
                        Vec4::from_point(self.lift([foot[0] + rr * dirs[k][0], foot[1] + rr * dirs[k][1]], z), q.w * weights[k])
                    })
                    .collect()
            })
            .collect();
        NurbsSurface::new(2, m.degree, vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0], m.knots.clone(), net)
    }
    fn cap_level(&self, e: usize, piece: usize, v: f64) -> Result<NurbsCurve, String> {
        let [rr, z] = self.rz(piece, v)?;
        let (foot, dirs, weights) = self.cap_frame(e);
        let points = (0..5).map(|k| Vec4::from_point(self.lift([foot[0] + rr * dirs[k][0], foot[1] + rr * dirs[k][1]], z), weights[k])).collect();
        NurbsCurve::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 2.0, 2.0, 2.0], points)
    }
}

/// Which way is out of the solid at a sheet point: (s n, -R'(h)) lifted.
fn outward_sheet(build: &Build, side: usize, t: f64, piece: usize, v: f64) -> Result<Vec3, String> {
    let n = build.normal(t);
    let dv = 1e-6 * (build.pieces[piece].domain[1] - build.pieces[piece].domain[0]);
    let (p0, p1) = (build.rz(piece, v - dv)?, build.rz(piece, v + dv)?);
    let slope = (p1[0] - p0[0]) / (p1[1] - p0[1]);
    Ok(build.slot.x.scale(sign(side) * n[0]).add(build.slot.y.scale(sign(side) * n[1])).sub(build.slot.up.scale(slope)))
}

fn surface_normal(surface: &NurbsSurface, u: f64, v: f64, du: f64, dv: f64) -> Result<Vec3, String> {
    let su = surface.evaluate(u + du, v)?.sub(surface.evaluate(u - du, v)?);
    let sv = surface.evaluate(u, v + dv)?.sub(surface.evaluate(u, v - dv)?);
    Ok(su.cross(sv))
}

impl Build<'_> {
    /// The whole solid.
    fn solid(&self) -> Result<BrepSolid, KernelRefusal> {
        let fail = |m: String| refuse("upright_direct_build", m);
        let levels = self.levels()?;
        let mut bands = Vec::new();
        for i in 0..levels.len() - 1 {
            let mut band = self.band(&levels, i)?;
            self.pin(&levels, &mut band)?;
            bands.push(band);
        }
        // caps: every band's outline must reach both ends of the curve on both sides
        for band in &bands {
            for side in 0..2 {
                let starts = band.runs.iter().any(|r| r.side == side && r.t[0] == 0.0);
                let ends = band.runs.iter().any(|r| r.side == side && r.t[1] == 1.0);
                if !(starts && ends) {
                    return Err(fail("a curve end is not on the outline (an end inside the sweep)".into()));
                }
            }
        }
        let last = CORNER_SAMPLES;
        // A run whose two ends are one corner (a hole's wall closed by one
        // corner) is split along an upright at its middle, so that no face
        // meets itself and each face's own sheet is one-to-one.
        let mut splits: Vec<Vec<(usize, f64)>> = Vec::new();
        for band in &bands {
            let mut list = Vec::new();
            for k in 0..band.runs.len() {
                let (Some((c0, _)), Some((c1, _))) = (band.ends[k][0], band.ends[k][1]) else { continue };
                if c0 != c1 {
                    continue;
                }
                let mid = Build::run_ends(band, k, last / 2);
                let ts = 0.5 * (mid[0] + mid[1]);
                if !(0..=last).all(|j| {
                    let t = Build::run_ends(band, k, j);
                    t[0] < ts && ts < t[1]
                }) {
                    return Err(fail("a closed wall has no upright to split it along".into()));
                }
                list.push((k, ts));
            }
            // a run whose closure holds two cusps (pivots born at one of its
            // levels) is split between them: a face meets the sheet's
            // singular points at one corner at most, where its loops start
            let cusps_at = |level: usize| -> Vec<(usize, f64)> {
                match &levels[level].event {
                    Some(Event::Birth(c)) => c.clone(),
                    _ => Vec::new(),
                }
            };
            for k in 0..band.runs.len() {
                let side = band.runs[k].side;
                let mut held: Vec<f64> = Vec::new();
                for (end, j) in [(0usize, 0usize), (1, last)] {
                    let t = Build::run_ends(band, k, j);
                    held.extend(cusps_at(band.level[end]).into_iter().filter(|&(s, tc)| s == side && t[0] - 1e-12 <= tc && tc <= t[1] + 1e-12).map(|x| x.1));
                }
                held.sort_by(f64::total_cmp);
                for w in held.windows(2) {
                    let ts = 0.5 * (w[0] + w[1]);
                    if list.iter().any(|&(kk, x): &(usize, f64)| kk == k && (x - ts).abs() <= 1e-9) {
                        continue;
                    }
                    // inside the run at every height, or the point it closes to at an end
                    if !(0..=last).all(|j| {
                        let t = Build::run_ends(band, k, j);
                        t[0] < ts && ts < t[1] || (j == 0 || j == last) && (t[1] - t[0]).abs() <= 1e-9 && (ts - t[0]).abs() <= 1e-9
                    }) {
                        return Err(fail("two cusps of one wall have no upright between them".into()));
                    }
                    list.push((k, ts));
                }
            }
            splits.push(list);
        }
        // vertices per level: (side, t) keys grouped by point
        struct Key {
            side: usize,
            t: f64,
            vertex: usize,
        }
        let mut points: Vec<Vec3> = Vec::new();
        let mut keys: Vec<Vec<Key>> = (0..levels.len()).map(|_| Vec::new()).collect();
        let r_level = |b: &Band, end: usize| self.r_at(b.piece, b.v[end]);
        for (bi, band) in bands.iter().enumerate() {
            for end in 0..2 {
                let level = band.level[end];
                let j = if end == 0 { 0 } else { last };
                let r = r_level(band, end).ok_or_else(|| fail("no radius".into()))?;
                let h = levels[level].h;
                let mut entries: Vec<(usize, f64)> = Vec::new();
                for c in &band.corners {
                    entries.push((c.side[0], c.t[j][0]));
                    entries.push((c.side[1], c.t[j][1]));
                }
                for side in 0..2 {
                    entries.push((side, 0.0));
                    entries.push((side, 1.0));
                }
                for &(k, ts) in &splits[bi] {
                    entries.push((band.runs[k].side, ts));
                }
                for (side, t) in entries {
                    if keys[level].iter().any(|k| k.side == side && (k.t - t).abs() <= 1e-9) {
                        continue;
                    }
                    let p = self.lift(self.sheet(side, t, r), h);
                    let vertex = match keys[level].iter().find(|k| points[k.vertex].sub(p).length() <= 1e-9 * self.scale) {
                        Some(k) => k.vertex,
                        None => {
                            points.push(p);
                            points.len() - 1
                        }
                    };
                    keys[level].push(Key { side, t, vertex });
                }
            }
        }
        // snap the bands' corner ends to the level keys
        for band in bands.iter_mut() {
            for end in 0..2 {
                let j = if end == 0 { 0 } else { last };
                for c in band.corners.iter_mut() {
                    for role in 0..2 {
                        let k = keys[band.level[end]].iter().find(|k| k.side == c.side[role] && (k.t - c.t[j][role]).abs() <= 1e-9).ok_or_else(|| fail("a corner end has no key".into()))?;
                        c.t[j][role] = k.t;
                    }
                }
            }
        }
        let key_vertex = |level: usize, side: usize, t: f64| keys[level].iter().find(|k| k.side == side && (k.t - t).abs() <= 1e-9).map(|k| k.vertex);
        let mut edges: Vec<EdgeRecord> = Vec::new();
        let add_edge = |curve: NurbsCurve, from: usize, to: usize, edges: &mut Vec<EdgeRecord>| -> Result<usize, KernelRefusal> {
            let [t0, t1] = geo(curve.domain(), "upright_direct_edge")?;
            edges.push(EdgeRecord { id: (edges.len() + 1) as u64 + 1_000_000, curve, t0, t1, start_vertex_id: from as u64 + 1, end_vertex_id: to as u64 + 1, degenerate: false, name: None });
            Ok(edges.len() - 1)
        };
        // level edges: per level and side, the elementary segments between keys inside a run
        let mut level_edges: Vec<Vec<(usize, f64, f64, usize)>> = (0..levels.len()).map(|_| Vec::new()).collect();
        let mut cap_edges: Vec<[Option<usize>; 2]> = vec![[None, None]; levels.len()];
        for level in 0..levels.len() {
            // the bands touching it and their run intervals there
            let mut intervals: Vec<(usize, f64, f64)> = Vec::new();
            let mut piece_v = None;
            for band in &bands {
                for end in 0..2 {
                    if band.level[end] != level {
                        continue;
                    }
                    let j = if end == 0 { 0 } else { last };
                    piece_v.get_or_insert((band.piece, band.v[end]));
                    for k in 0..band.runs.len() {
                        let t = Build::run_ends(band, k, j);
                        if t[1] > t[0] {
                            intervals.push((band.runs[k].side, t[0], t[1]));
                        }
                    }
                }
            }
            let Some((piece, v)) = piece_v else { continue };
            for side in 0..2 {
                let mut ts: Vec<f64> = keys[level].iter().filter(|k| k.side == side).map(|k| k.t).collect();
                ts.sort_by(f64::total_cmp);
                let curve = geo(self.sheet_level(side, piece, v), "upright_direct_level")?;
                for w in ts.windows(2) {
                    let mid = 0.5 * (w[0] + w[1]);
                    if !intervals.iter().any(|&(s, a, b)| s == side && a <= mid && mid <= b) {
                        continue;
                    }
                    let (from, to) = (key_vertex(level, side, w[0]).unwrap(), key_vertex(level, side, w[1]).unwrap());
                    let e = add_edge(geo(part(&curve, w[0], w[1]), "upright_direct_level")?, from, to, &mut edges)?;
                    level_edges[level].push((side, w[0], w[1], e));
                }
            }
            for e in 0..2 {
                let from = key_vertex(level, 0, e as f64).unwrap();
                let to = key_vertex(level, 1, e as f64).unwrap();
                cap_edges[level][e] = Some(add_edge(geo(self.cap_level(e, piece, v), "upright_direct_cap")?, from, to, &mut edges)?);
            }
        }
        // faces
        let mut faces: Vec<(NurbsSurface, Vec3, [f64; 2], Vec<Use>)> = Vec::new();
        let segments = |level: usize, side: usize, a: f64, b: f64| -> Vec<(usize, f64, f64)> {
            let mut out: Vec<(usize, f64, f64)> = level_edges[level].iter().filter(|&&(s, x, y, _)| s == side && x >= a - 1e-12 && y <= b + 1e-12).map(|&(_, x, y, e)| (e, x, y)).collect();
            out.sort_by(|p, q| p.1.total_cmp(&q.1));
            out
        };
        for (bi, band) in bands.iter().enumerate() {
            let (piece, [v0, v1]) = (band.piece, band.v);
            let (l0, l1) = (band.level[0], band.level[1]);
            // the corner edges and their pcurves per role
            let mut corner_edges = Vec::new();
            for c in &band.corners {
                let mut pts = Vec::new();
                let mut uv = [Vec::new(), Vec::new()];
                for (k, &sg) in c.sigma.iter().enumerate() {
                    let v = v_of(band.v, sg);
                    let [rr, z] = geo(self.rz(piece, v), "upright_direct_corner")?;
                    let p = if k == 0 {
                        points[key_vertex(l0, c.side[0], c.t[0][0]).unwrap()]
                    } else if k == last {
                        points[key_vertex(l1, c.side[0], c.t[last][0]).unwrap()]
                    } else {
                        self.lift(self.sheet(c.side[0], c.t[k][0], rr), z)
                    };
                    pts.push(p);
                    for role in 0..2 {
                        uv[role].push(Vec3::new(c.t[k][role], v, 0.0));
                    }
                }
                let curve = geo(interpolate_curve(&pts, 3, &c.sigma), "upright_direct_corner")?;
                let from = key_vertex(l0, c.side[0], c.t[0][0]).unwrap();
                let to = key_vertex(l1, c.side[0], c.t[last][0]).unwrap();
                let e = add_edge(curve, from, to, &mut edges)?;
                let pc = [geo(interpolate_curve(&uv[0], 3, &c.sigma), "upright_direct_pcurve")?, geo(interpolate_curve(&uv[1], 3, &c.sigma), "upright_direct_pcurve")?];
                corner_edges.push((e, pc));
            }
            // seams per side and end
            let mut seams = [[0usize; 2]; 2];
            for side in 0..2 {
                for e in 0..2 {
                    let curve = geo(part(&geo(self.sheet_seam(side, piece, e), "upright_direct_seam")?, v0, v1), "upright_direct_seam")?;
                    let (from, to) = (key_vertex(l0, side, e as f64).unwrap(), key_vertex(l1, side, e as f64).unwrap());
                    seams[side][e] = add_edge(curve, from, to, &mut edges)?;
                }
            }
            // the split uprights of this band
            let mut split_edges: Vec<(usize, f64, usize)> = Vec::new();
            for &(k, ts) in &splits[bi] {
                let side = band.runs[k].side;
                let curve = geo(part(&geo(self.sheet_upright(side, piece, ts), "upright_direct_split")?, v0, v1), "upright_direct_split")?;
                let (from, to) = (key_vertex(l0, side, ts).unwrap(), key_vertex(l1, side, ts).unwrap());
                split_edges.push((k, ts, add_edge(curve, from, to, &mut edges)?));
            }
            #[derive(Clone, Copy)]
            enum Bound {
                Corner(usize, usize),
                Cap(usize),
                Split(f64, usize),
            }
            let at = |b: Bound, j: usize| -> f64 {
                match b {
                    Bound::Corner(c, role) => band.corners[c].t[j][role],
                    Bound::Cap(e) => e as f64,
                    Bound::Split(t, _) => t,
                }
            };
            for k in 0..band.runs.len() {
                let side = band.runs[k].side;
                let mut bounds = vec![match band.ends[k][0] { Some((c, role)) => Bound::Corner(c, role), None => Bound::Cap(0) }];
                let mut inner: Vec<(f64, usize)> = split_edges.iter().filter(|x| x.0 == k).map(|x| (x.1, x.2)).collect();
                inner.sort_by(|p, q| p.0.total_cmp(&q.0));
                bounds.extend(inner.into_iter().map(|(t, e)| Bound::Split(t, e)));
                bounds.push(match band.ends[k][1] { Some((c, role)) => Bound::Corner(c, role), None => Bound::Cap(1) });
                for w in bounds.windows(2) {
                    let (lb, rb) = (w[0], w[1]);
                    // this face's own sheet: the cubic and normal fitted over its t-range only
                    let (mut ta, mut tb) = (f64::INFINITY, f64::NEG_INFINITY);
                    for j in 0..=last {
                        ta = ta.min(at(lb, j));
                        tb = tb.max(at(rb, j));
                    }
                    let margin = 1e-3 * (tb - ta);
                    let fit = fit_on(self.slot, (ta - margin).max(0.0), (tb + margin).min(1.0), self.scale)?;
                    let surface = geo(self.sheet_surface(side, piece, &fit, band.v), "upright_direct_sheet")?;
                    let (b0, b1) = ([at(lb, 0), at(rb, 0)], [at(lb, last), at(rb, last)]);
                    let mut uses = Vec::new();
                    for (e, a, b) in segments(l0, side, b0[0], b0[1]) {
                        uses.push(Use { edge: e, forward: true, pcurve: geo(line2([a, v0], [b, v0], a, b), "upright_direct_pcurve")? });
                    }
                    let upright = |t: f64| line2([t, v0], [t, v1], v0, v1);
                    match rb {
                        Bound::Corner(c, role) => uses.push(Use { edge: corner_edges[c].0, forward: true, pcurve: corner_edges[c].1[role].clone() }),
                        Bound::Cap(_) => uses.push(Use { edge: seams[side][1], forward: true, pcurve: geo(upright(1.0), "upright_direct_pcurve")? }),
                        Bound::Split(t, e) => uses.push(Use { edge: e, forward: true, pcurve: geo(upright(t), "upright_direct_pcurve")? }),
                    }
                    for (e, a, b) in segments(l1, side, b1[0], b1[1]).into_iter().rev() {
                        uses.push(Use { edge: e, forward: false, pcurve: geo(line2([a, v1], [b, v1], a, b), "upright_direct_pcurve")? });
                    }
                    match lb {
                        Bound::Corner(c, role) => uses.push(Use { edge: corner_edges[c].0, forward: false, pcurve: corner_edges[c].1[role].clone() }),
                        Bound::Cap(_) => uses.push(Use { edge: seams[side][0], forward: false, pcurve: geo(upright(0.0), "upright_direct_pcurve")? }),
                        Bound::Split(t, e) => uses.push(Use { edge: e, forward: false, pcurve: geo(upright(t), "upright_direct_pcurve")? }),
                    }
                    let tm = 0.5 * (at(lb, last / 2) + at(rb, last / 2));
                    let vm = v_of(band.v, 0.5);
                    let out = geo(outward_sheet(self, side, tm, piece, vm), "upright_direct_orient")?;
                    faces.push((surface, out, [tm, vm], uses));
                }
            }
            for e in 0..2 {
                let surface = geo(self.cap_surface(e, piece, band.v), "upright_direct_cap")?;
                let mut uses = Vec::new();
                uses.push(Use { edge: cap_edges[l0][e].unwrap(), forward: true, pcurve: geo(line2([0.0, v0], [2.0, v0], 0.0, 2.0), "upright_direct_pcurve")? });
                uses.push(Use { edge: seams[1][e], forward: true, pcurve: geo(line2([2.0, v0], [2.0, v1], v0, v1), "upright_direct_pcurve")? });
                uses.push(Use { edge: cap_edges[l1][e].unwrap(), forward: false, pcurve: geo(line2([0.0, v1], [2.0, v1], 0.0, 2.0), "upright_direct_pcurve")? });
                uses.push(Use { edge: seams[0][e], forward: false, pcurve: geo(line2([0.0, v0], [0.0, v1], v0, v1), "upright_direct_pcurve")? });
                let vm = v_of(band.v, 0.5);
                let (foot, dirs, _) = self.cap_frame(e);
                let [rr, z] = geo(self.rz(piece, vm), "upright_direct_cap")?;
                let radial = self.lift([foot[0] + rr * dirs[2][0], foot[1] + rr * dirs[2][1]], z).sub(self.lift(foot, z));
                let dvv = 1e-6;
                let (p0, p1) = (geo(self.rz(piece, vm - dvv), "upright_direct_cap")?, geo(self.rz(piece, vm + dvv), "upright_direct_cap")?);
                let slope = (p1[0] - p0[0]) / (p1[1] - p0[1]);
                let out = radial.scale(1.0 / rr.max(1e-300)).sub(self.slot.up.scale(slope));
                faces.push((surface, out, [1.0, vm], uses));
            }
        }
        // records
        let vertices: Vec<VertexRecord> = points.iter().enumerate().map(|(k, p)| VertexRecord { id: k as u64 + 1, point: *p }).collect();
        let mut next_id = 2_000_000u64;
        let mut id = || {
            next_id += 1;
            next_id
        };
        let coedges_of = |uses: Vec<Use>, flip: bool, id: &mut dyn FnMut() -> u64| -> Result<Vec<CoedgeRecord>, KernelRefusal> {
            let mut out = Vec::new();
            let mut list: Vec<Use> = uses;
            if flip {
                list.reverse();
            }
            for u in list {
                let forward = u.forward != flip;
                let pcurve = if forward { u.pcurve } else { geo(u.pcurve.reversed(), "upright_direct_pcurve")? };
                out.push(CoedgeRecord { id: id(), edge_id: edges[u.edge].id, forward, pcurve });
            }
            Ok(out)
        };
        // the cusp vertices (pivots born): each loop through one starts there
        let mut cusp_vertices: Vec<u64> = Vec::new();
        for (level, l) in levels.iter().enumerate() {
            if let Some(Event::Birth(c)) = &l.event {
                for &(side, tc) in c {
                    if let Some(vx) = key_vertex(level, side, tc) {
                        cusp_vertices.push(vx as u64 + 1);
                    }
                }
            }
        }
        let start_at_cusp = |coedges: &mut Vec<CoedgeRecord>| {
            let start = |c: &CoedgeRecord| edges.iter().find(|e| e.id == c.edge_id).map(|e| if c.forward { e.start_vertex_id } else { e.end_vertex_id });
            if let Some(k) = coedges.iter().position(|c| start(c).is_some_and(|v| cusp_vertices.contains(&v))) {
                coedges.rotate_left(k);
            }
        };
        let mut face_records = Vec::new();
        for (surface, out, [u, v], uses) in faces {
            let span_u = surface.knots_u[surface.knots_u.len() - 1] - surface.knots_u[0];
            let span_v = surface.knots_v[surface.knots_v.len() - 1] - surface.knots_v[0];
            let normal = geo(surface_normal(&surface, u, v, 1e-6 * span_u, 1e-6 * span_v), "upright_direct_orient")?;
            let same = normal.dot(out) > 0.0;
            let mut coedges = coedges_of(uses, !same, &mut id)?;
            start_at_cusp(&mut coedges);
            face_records.push(FaceRecord { id: id(), surface, same_sense: same, loops: vec![LoopRecord { id: id(), coedges }], name: None });
        }
        // floor and top: every level edge there, the other way round from its side face's use, chained into loops
        let mut used_forward: std::collections::HashMap<u64, bool> = std::collections::HashMap::new();
        for f in &face_records {
            for l in &f.loops {
                for c in &l.coedges {
                    used_forward.insert(c.edge_id, c.forward);
                }
            }
        }
        let mut planar: Vec<(NurbsSurface, Vec3, Vec<Vec<Use>>)> = Vec::new();
        for (level, outward) in [(0usize, self.slot.up.scale(-1.0)), (levels.len() - 1, self.slot.up)] {
            let h = levels[level].h;
            let mut all: Vec<usize> = level_edges[level].iter().map(|x| x.3).collect();
            all.extend(cap_edges[level].iter().flatten().copied());
            let reach = 4.0 * self.scale + self.slot.curve.iter().map(|p| p[0].abs().max(p[1].abs())).fold(0.0, f64::max);
            let origin = self.lift([-reach, -reach], h);
            let plane = geo(make_plane(origin, self.slot.x, self.slot.y, 2.0 * reach, 2.0 * reach), "upright_direct_plane")?;
            let chart = |c: &NurbsCurve| -> Result<NurbsCurve, String> {
                let pts = c.control_points.iter().map(|q| {
                    let p = Vec3::new(q.x / q.w, q.y / q.w, q.z / q.w).sub(origin);
                    Vec4 { x: p.dot(self.slot.x) * q.w, y: p.dot(self.slot.y) * q.w, z: 0.0, w: q.w }
                });
                NurbsCurve::new(c.degree, c.knots.clone(), pts.collect())
            };
            // each edge as this face runs it: (edge, forward, from, to)
            let mut left: Vec<(usize, bool, u64, u64)> = Vec::new();
            for e in all {
                let forward = !*used_forward.get(&edges[e].id).ok_or_else(|| fail(format!("a level edge at {h} has no side face")))?;
                let (a, b) = (edges[e].start_vertex_id, edges[e].end_vertex_id);
                left.push(if forward { (e, true, a, b) } else { (e, false, b, a) });
            }
            let mut loops: Vec<Vec<Use>> = Vec::new();
            while let Some(first) = left.pop() {
                let mut cycle = vec![first];
                let start = first.2;
                let mut at = first.3;
                while at != start {
                    let next = left.iter().position(|x| x.2 == at).ok_or_else(|| fail(format!("the outline at {h} does not close")))?;
                    let x = left.swap_remove(next);
                    at = x.3;
                    cycle.push(x);
                }
                let mut uses = Vec::new();
                for (e, forward, _, _) in cycle {
                    uses.push(Use { edge: e, forward, pcurve: geo(chart(&edges[e].curve), "upright_direct_plane")? });
                }
                loops.push(uses);
            }
            planar.push((plane, outward, loops));
        }
        for (surface, out, loops) in planar {
            // the plane's own normal (its chart runs along the slot's x and y)
            let same = self.slot.x.cross(self.slot.y).dot(out) > 0.0;
            let mut loop_records = Vec::new();
            for uses in loops {
                let coedges = coedges_of(uses, false, &mut id)?;
                loop_records.push(LoopRecord { id: id(), coedges });
            }
            face_records.push(FaceRecord { id: id(), surface, same_sense: same, loops: loop_records, name: None });
        }
        // genus from the shell's Euler count, the faces' inner loops counted
        // (V - E + F - inner loops = 2 - 2 genus): a hole through the tool's whole height is a handle
        let inner: usize = face_records.iter().map(|f| f.loops.len().saturating_sub(1)).sum();
        let euler = vertices.len() as i64 - edges.len() as i64 + face_records.len() as i64 - inner as i64;
        if euler > 2 || (2 - euler) % 2 != 0 {
            return Err(fail(format!("the shell's Euler count {euler} is no closed surface's")));
        }
        let genus = (2 - euler) / 2;
        let solid = BrepSolid {
            mass_properties_cache: Default::default(),
            id: id(),
            vertices,
            edges,
            shells: vec![ShellRecord { id: id(), faces: face_records }],
            genus,
            kind: BodyKind::Solid,
            cells: Default::default(),
        };
        Ok(solid)
    }
}

/// The direct build of a slot's solid (see the module docs).
pub(super) fn direct_solid(slot: &UprightSlot) -> Result<BrepSolid, KernelRefusal> {
    Build::new(slot)?.solid()
}

