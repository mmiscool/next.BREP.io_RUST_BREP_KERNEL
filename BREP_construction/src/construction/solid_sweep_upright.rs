//! An upright solid of revolution carried along a planar cubic (the riding
//! frame's `up` along the tool's axis, the curve in a plane square to it).
//!
//! Turned about `up` by the frame, the tool is unchanged (it is turned about
//! its own axis), so the motion is a translation along the curve, and what the
//! tool passes through is exactly
//!
//! ```text
//! S = { p : the planar distance from p to the curve <= radius(h) },
//! ```
//!
//! `h` the height of `p` above the curve's plane and `radius(h)` the tool's
//! radius there (its section at `h` is a disc about the axis). The motion's
//! frame turning, the curve's loops and its turns tighter than the tool, which
//! fold a swept wall, are all one read: the planar distance to the curve.
//!
//! This module holds that read: the gate ([`upright_slot`]), the distance
//! ([`UprightSlot::distance`]), membership ([`UprightSlot::contains`]), the
//! level outlines of the planar strokes `{ d <= rho }`
//! ([`UprightSlot::level_loops`]) and the swept volume they integrate to
//! ([`UprightSlot::volume`]), the referee of the solid built from them.

use crate::{FrameCurve, OutlinePiece, SolidTool, Vec3};

/// The envelope the reads hold in, checked by the gate before any arithmetic:
/// the tool's height within `ENVELOPE` of its widest radius either way, the
/// curve's extent within `ENVELOPE` of that radius, all three between 1e-60
/// and 1e60, every arc's radius within `ENVELOPE` of the tool's, the curve regular (its
/// speed nowhere below 1e-4 of its control polygon's, so its feet are read
/// to the last bits), and every meridian arc at most one turn. Outside it the
/// gate declines (the general sweep answers).
pub const ENVELOPE: f64 = 1e6;

/// The switch for the upright route in [`crate::sweep_solid_along`] (default
/// off until promoted).
pub fn upright_slot_on() -> bool {
    crate::switch!("BREP_SWEEP_UPRIGHT_SLOT", false)
}

/// What an upright solid of revolution passes through carried along a planar
/// cubic ([`UprightSlot::solid`]), or `None` where the tool and the curve are
/// not of this class ([`upright_slot`]). Whatever the switch: the call a
/// caller makes for this class by name.
pub fn sweep_upright_along(tool: &SolidTool, path: &FrameCurve) -> Option<Result<crate::BrepSolid, crate::KernelRefusal>> {
    upright_slot(tool, path).map(|slot| slot.solid())
}

/// An upright revolved tool's sweep along a planar cubic, in the curve's
/// plane: the cubic's controls in plane coordinates and the tool's radius as
/// a function of height above the plane (its meridian). Its reads rely on the
/// envelope [`upright_slot`] checks: build one through it (a slot assembled
/// by hand outside the envelope reads no further than its budgets allow).
#[derive(Clone, Debug)]
pub struct UprightSlot {
    /// The plane's origin (the curve's start) and its axes; `up` is its normal.
    pub origin: Vec3,
    pub x: Vec3,
    pub y: Vec3,
    pub up: Vec3,
    /// The cubic's controls in (x, y).
    pub curve: [[f64; 2]; 4],
    /// The meridian in (radius, height above the plane).
    pub meridian: Vec<OutlinePiece>,
    /// The tool's height range above the plane.
    pub heights: [f64; 2],
}

/// The angles `a + 2πk` inside [lo, hi] (any number of turns).
fn turns_within(a: f64, lo: f64, hi: f64) -> impl Iterator<Item = f64> {
    let tau = std::f64::consts::TAU;
    let (first, last) = (((lo - a) / tau).ceil(), ((hi - a) / tau).floor());
    let count = if first.is_finite() && last.is_finite() && last >= first && last - first < 1e6 { (last - first) as i64 + 1 } else { 0 };
    (0..count).map(move |k| a + tau * (first + k as f64))
}

/// A meridian piece's greatest |radius| and |height| over its own extent (an
/// arc over its angles, not its whole circle). Ellipses read as unbounded.
fn piece_extent(piece: &OutlinePiece) -> (f64, f64) {
    match *piece {
        OutlinePiece::Line { from, to } => (from[0].abs().max(to[0].abs()), from[1].abs().max(to[1].abs())),
        OutlinePiece::Arc { centre, radius, start, end } => {
            let (a, b) = (start.min(end), start.max(end));
            let at = |t: f64| [centre[0] + radius * t.cos(), centre[1] + radius * t.sin()];
            let mut points = vec![at(start), at(end)];
            for k in [0.0, std::f64::consts::FRAC_PI_2, std::f64::consts::PI, -std::f64::consts::FRAC_PI_2] {
                points.extend(turns_within(k, a, b).take(1).map(at));
            }
            points.iter().fold((0.0f64, 0.0f64), |(r, h), p| (r.max(p[0].abs()), h.max(p[1].abs())))
        }
        OutlinePiece::Ellipse { .. } => (f64::INFINITY, f64::INFINITY),
    }
}

fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

/// The sweep of `tool` along `path` when it is of this class: a solid of
/// revolution whose axis lies along `up` through the curve's start, the curve
/// in the plane square to `up` there, and every section of the tool square to
/// its axis a disc about it (its meridian a radius per height). `None`
/// otherwise.
pub fn upright_slot(tool: &SolidTool, path: &FrameCurve) -> Option<UprightSlot> {
    let SolidTool::Revolved { axis_origin, axis_direction, radial, meridian } = tool else { return None };
    let finite = |p: Vec3| p.x.is_finite() && p.y.is_finite() && p.z.is_finite();
    let pieces_finite = meridian.iter().all(|piece| match *piece {
        OutlinePiece::Line { from, to } => from.iter().chain(&to).all(|x| x.is_finite()),
        OutlinePiece::Arc { centre, radius, start, end } => centre.iter().all(|x| x.is_finite()) && radius.is_finite() && start.is_finite() && end.is_finite(),
        OutlinePiece::Ellipse { .. } => false,
    });
    if !(pieces_finite && path.control.iter().all(|p| finite(*p)) && finite(path.up) && finite(*axis_origin) && finite(*axis_direction) && finite(*radial)) {
        return None;
    }
    // Unit directions read without overflow (scaled by the largest component first).
    let unit = |d: Vec3| -> Option<Vec3> {
        let big = d.x.abs().max(d.y.abs()).max(d.z.abs());
        if !(big > 0.0 && big.is_finite()) {
            return None;
        }
        let n = d.scale(1.0 / big).normalized().ok()?;
        ((n.length() - 1.0).abs() <= 1e-12).then_some(n)
    };
    let up = unit(path.up)?;
    let axis = unit(*axis_direction)?;
    let along = axis.dot(up).signum();
    let start = path.control[0];
    // Every tolerance a displacement in the tool's own widest radius: the
    // axis's sideways offset, its drift over the tool's height (its tilt), and
    // the curve's rise off its plane over its whole run.
    let (widest, tall) = meridian.iter().map(piece_extent).fold((0.0f64, 0.0f64), |(r, h), (pr, ph)| (r.max(pr), h.max(ph)));
    if !(widest > 0.0 && widest.is_finite() && tall.is_finite()) {
        return None;
    }
    // Arcs no wider than the envelope allows (a sliver of a huge circle reads
    // its crossings no finer than its radius).
    if meridian.iter().any(|piece| matches!(*piece, OutlinePiece::Arc { radius, .. } if !(radius > 0.0 && radius <= ENVELOPE * widest))) {
        return None;
    }
    // The envelope (see `ENVELOPE`).
    let extent = path.control.iter().map(|p| p.sub(start).length()).fold(0.0f64, f64::max);
    let one_turn = meridian.iter().all(|piece| match *piece {
        OutlinePiece::Arc { start, end, .. } => (end - start).abs() <= std::f64::consts::TAU * (1.0 + 1e-12),
        _ => true,
    });
    // And sizes whose products stay in range (the quintic's coefficients
    // are fourth powers of the curve's extent).
    let sized = |x: f64| (1e-60..=1e60).contains(&x);
    if !(one_turn && sized(widest) && sized(extent) && sized(tall.max(widest)) && tall <= ENVELOPE * widest && tall >= widest / ENVELOPE && extent <= ENVELOPE * widest && extent >= widest / ENVELOPE) {
        return None;
    }
    let band = 1e-12 * widest;
    if axis.cross(up).length() * tall.max(widest) > band {
        return None;
    }
    // The axis through the curve's start, the curve in the plane square to up.
    let off = axis_origin.sub(start);
    if off.sub(up.scale(off.dot(up))).length() > band {
        return None;
    }
    if path.control.iter().any(|p| p.sub(start).dot(up).abs() > band) {
        return None;
    }
    // The radial direction square to the axis (it only names the meridian's
    // first coordinate; the tool is turned about the axis whatever it is).
    let radial_unit = unit(*radial)?;
    if radial_unit.dot(axis).abs() > 1e-12 {
        return None;
    }
    // A curve with somewhere to go: its controls not all at its start.
    if path.control.iter().all(|p| p.sub(start).length() <= band) {
        return None;
    }
    let lift = off.dot(up);
    let meridian: Vec<OutlinePiece> = meridian
        .iter()
        .map(|piece| match *piece {
            OutlinePiece::Line { from, to } => Some(OutlinePiece::Line { from: [from[0], lift + along * from[1]], to: [to[0], lift + along * to[1]] }),
            OutlinePiece::Arc { centre, radius, start, end } => {
                // Reflected in the height when the axis runs down: angles negate.
                if along > 0.0 {
                    Some(OutlinePiece::Arc { centre: [centre[0], lift + centre[1]], radius, start, end })
                } else {
                    Some(OutlinePiece::Arc { centre: [centre[0], lift - centre[1]], radius, start: -start, end: -end })
                }
            }
            OutlinePiece::Ellipse { .. } => None,
        })
        .collect::<Option<_>>()?;
    let tangent = path.control[1].sub(start);
    let x = tangent.sub(up.scale(tangent.dot(up))).normalized().or_else(|_| up.perpendicular()).ok()?;
    let y = up.cross(x);
    let flat = |p: Vec3| [p.sub(start).dot(x), p.sub(start).dot(y)];
    let mut slot = UprightSlot { origin: start, x, y, up, curve: path.control.map(flat), meridian, heights: [0.0; 2] };
    if !slot.regular() {
        return None;
    }
    // The height range, and a disc at every height: the meridian crosses each
    // level line once off the axis.
    let ends = slot.meridian.iter().flat_map(|piece| match *piece {
        OutlinePiece::Line { from, to } => vec![from[1], to[1]],
        OutlinePiece::Arc { centre, radius, start, end } => {
            let mut hs = vec![centre[1] + radius * start.sin(), centre[1] + radius * end.sin()];
            for a in turns_within(std::f64::consts::FRAC_PI_2, start.min(end), start.max(end)).chain(turns_within(-std::f64::consts::FRAC_PI_2, start.min(end), start.max(end))) {
                hs.push(centre[1] + radius * a.sin());
            }
            hs
        }
        OutlinePiece::Ellipse { .. } => vec![],
    });
    let (low, high) = ends.fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), h| (lo.min(h), hi.max(h)));
    if !(high > low) {
        return None;
    }
    slot.heights = [low, high];
    if !slot.sections_are_discs() {
        return None;
    }
    Some(slot)
}

impl UprightSlot {
    /// Whether every section of the tool square to its axis is a disc about
    /// it, read off the meridian's pieces (not sampled): one piece runs along
    /// the axis over the whole height range, and the others chain from its
    /// bottom end to its top end with the height never turning back along the
    /// chain (a line's height monotone, an arc holding no vertical tangent
    /// inside it), so each level line meets the chain once.
    fn sections_are_discs(&self) -> bool {
        let [low, high] = self.heights;
        // Heights in the tool's height, radii in its widest radius.
        let tol = 1e-12 * (high - low).abs().max(1e-300);
        let tol_r = 1e-12 * self.widest().max(1e-300);
        let ends = |piece: &OutlinePiece| -> Option<([f64; 2], [f64; 2])> {
            match *piece {
                OutlinePiece::Line { from, to } => Some((from, to)),
                OutlinePiece::Arc { centre, radius, start, end } => Some((
                    [centre[0] + radius * start.cos(), centre[1] + radius * start.sin()],
                    [centre[0] + radius * end.cos(), centre[1] + radius * end.sin()],
                )),
                OutlinePiece::Ellipse { .. } => None,
            }
        };
        let on_axis = |p: [f64; 2]| p[0].abs() <= tol_r;
        let axis_piece = |piece: &OutlinePiece| matches!(*piece, OutlinePiece::Line { from, to } if on_axis(from) && on_axis(to) && (from[1] - to[1]).abs() >= high - low - tol);
        let Some(axis_index) = self.meridian.iter().position(axis_piece) else { return false };
        // The chain: the pieces after the axis piece, in order, from its end round to its start.
        let n = self.meridian.len();
        let chain: Vec<&OutlinePiece> = (1..n).map(|k| &self.meridian[(axis_index + k) % n]).collect();
        let Some((first, _)) = chain.first().and_then(|p| ends(p)) else { return false };
        let Some((_, last)) = chain.last().and_then(|p| ends(p)) else { return false };
        // The chain closes the meridian: from the axis piece's end back to its start.
        let Some((axis_from, axis_to)) = ends(&self.meridian[axis_index]) else { return false };
        let same = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).abs() <= tol_r && (a[1] - b[1]).abs() <= tol;
        if !(same(first, axis_to) && same(last, axis_from)) {
            return false;
        }
        // Rising or falling along the whole chain, the same way throughout.
        let rising = last[1] > first[1];
        let mut previous = first;
        let last_index = chain.len() - 1;
        for (index, piece) in chain.iter().enumerate() {
            let Some((from, to)) = ends(piece) else { return false };
            if (from[0] - previous[0]).abs() > tol_r || (from[1] - previous[1]).abs() > tol {
                return false;
            }
            let monotone = match **piece {
                // A flat line may only run along an end (the chain's first or
                // last piece at the bottom or the top height).
                OutlinePiece::Line { from, to } => {
                    let dh = to[1] - from[1];
                    // Flat to the tool's radius scale (a tall tool's height
                    // tolerance would pass a slanted floor as flat).
                    if dh.abs() <= tol_r.min(tol) {
                        // Only the chain's first or last piece, along an end.
                        (index == 0 || index == last_index) && ((from[1] - low).abs() <= tol || (from[1] - high).abs() <= tol)
                    } else {
                        (dh > 0.0) == rising
                    }
                }
                OutlinePiece::Arc { radius, start, end, .. } => {
                    // dh/dθ = r cos θ keeps one sign over the arc's interior.
                    let (a, b) = (start.min(end), start.max(end));
                    let inner = 1e-12 * (b - a).abs().max(1.0);
                    let turns = turns_within(std::f64::consts::FRAC_PI_2, a + inner, b - inner)
                        .chain(turns_within(-std::f64::consts::FRAC_PI_2, a + inner, b - inner))
                        .next()
                        .is_some();
                    !turns && radius > 0.0 && ((to[1] - from[1] > 0.0) == rising || (to[1] - from[1]).abs() <= tol)
                }
                OutlinePiece::Ellipse { .. } => false,
            };
            if !monotone {
                return false;
            }
            // Off the axis between the chain's ends (a chain touching the axis
            // in its middle makes two solids meeting at a point).
            if index != last_index && on_axis(to) {
                return false;
            }
            // Nor anywhere inside a piece: a line's radius is never negative at
            // its ends (so it stays off the axis between off-axis ends); an arc's
            // least radius, where it faces the axis (θ = π + 2πk inside it),
            // stays off the axis.
            let off = match **piece {
                OutlinePiece::Line { from, to } => from[0] >= -tol_r && to[0] >= -tol_r,
                OutlinePiece::Arc { centre, radius, start, end } => {
                    let (a, b) = (start.min(end), start.max(end));
                    let inner = 1e-12 * (b - a).abs().max(1.0);
                    // Its ends in the half-plane, and where it faces the axis
                    // inside, its least radius off the axis.
                    let ends_in = centre[0] + radius * start.cos() >= -tol_r && centre[0] + radius * end.cos() >= -tol_r;
                    ends_in && (turns_within(std::f64::consts::PI, a + inner, b - inner).next().is_none() || centre[0] - radius > tol_r)
                }
                OutlinePiece::Ellipse { .. } => false,
            };
            if !off {
                return false;
            }
            previous = to;
        }
        true
    }

    /// Whether the curve's speed stays above 1e-4 of its control polygon's
    /// (the envelope): the least of |B'|² over [0, 1], read at the ends and at
    /// every root of its derivative (a cubic) bracketed on 256 spans.
    fn regular(&self) -> bool {
        let c = &self.curve;
        let polygon = (0..3).map(|k| sub(c[k + 1], c[k])).map(|d| 3.0 * dot(d, d).sqrt()).fold(0.0f64, f64::max);
        if !(polygon > 0.0) {
            return false;
        }
        let speed2 = |t: f64| {
            let d = self.derivative(t);
            dot(d, d)
        };
        let slope = |t: f64| dot(self.derivative(t), self.second(t));
        let mut least = speed2(0.0).min(speed2(1.0));
        let n = 256;
        let mut previous = slope(0.0);
        for k in 1..=n {
            let t1 = k as f64 / n as f64;
            let current = slope(t1);
            if (previous < 0.0) != (current < 0.0) {
                let (mut a, mut b, mut sa) = ((k - 1) as f64 / n as f64, t1, previous);
                for _ in 0..100 {
                    let m = 0.5 * (a + b);
                    let sm = slope(m);
                    if (sm < 0.0) == (sa < 0.0) {
                        a = m;
                        sa = sm;
                    } else {
                        b = m;
                    }
                }
                least = least.min(speed2(0.5 * (a + b)));
            }
            previous = current;
        }
        least.sqrt() >= 1e-4 * polygon
    }

    /// The meridian's crossings with the level line at `h`, off the axis.
    fn crossings(&self, h: f64) -> Vec<f64> {
        let mut found: Vec<f64> = Vec::new();
        for piece in &self.meridian {
            match *piece {
                OutlinePiece::Line { from, to } => {
                    let (a, b) = (from[1] - h, to[1] - h);
                    if a == 0.0 && b == 0.0 {
                        continue;
                    }
                    if (a <= 0.0 && b >= 0.0) || (a >= 0.0 && b <= 0.0) {
                        let s = a / (a - b);
                        found.push(from[0] + s * (to[0] - from[0]));
                    }
                }
                OutlinePiece::Arc { centre, radius, start, end } => {
                    let s = (h - centre[1]) / radius;
                    if s.abs() > 1.0 {
                        continue;
                    }
                    let base = s.asin();
                    for a in [base, std::f64::consts::PI - base] {
                        for angle in turns_within(a, start.min(end), start.max(end)) {
                            found.push(centre[0] + radius * angle.cos());
                        }
                    }
                }
                OutlinePiece::Ellipse { .. } => {}
            }
        }
        // Off the axis, in the tool's own size.
        let widest = found.iter().fold(0.0f64, |m, r| m.max(r.abs()));
        let floor = 1e-12 * widest;
        let mut off: Vec<f64> = found.into_iter().filter(|r| *r > floor).collect();
        off.sort_by(f64::total_cmp);
        off.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * a.abs());
        off
    }

    /// The tool's radius at height `h` above the plane (0 outside its range).
    pub fn radius(&self, h: f64) -> f64 {
        if !(h >= self.heights[0] && h <= self.heights[1]) {
            return 0.0;
        }
        self.crossings(h).last().copied().unwrap_or(0.0)
    }

    fn point(&self, t: f64) -> [f64; 2] {
        let c = &self.curve;
        let s = 1.0 - t;
        let w = [s * s * s, 3.0 * s * s * t, 3.0 * s * t * t, t * t * t];
        [(0..4).map(|k| w[k] * c[k][0]).sum(), (0..4).map(|k| w[k] * c[k][1]).sum()]
    }

    fn derivative(&self, t: f64) -> [f64; 2] {
        let c = &self.curve;
        let s = 1.0 - t;
        let w = [3.0 * s * s, 6.0 * s * t, 3.0 * t * t];
        let d = [sub(c[1], c[0]), sub(c[2], c[1]), sub(c[3], c[2])];
        [(0..3).map(|k| w[k] * d[k][0]).sum(), (0..3).map(|k| w[k] * d[k][1]).sum()]
    }

    fn second(&self, t: f64) -> [f64; 2] {
        let c = &self.curve;
        let a = [c[2][0] - 2.0 * c[1][0] + c[0][0], c[2][1] - 2.0 * c[1][1] + c[0][1]];
        let b = [c[3][0] - 2.0 * c[2][0] + c[1][0], c[3][1] - 2.0 * c[2][1] + c[1][1]];
        [6.0 * ((1.0 - t) * a[0] + t * b[0]), 6.0 * ((1.0 - t) * a[1] + t * b[1])]
    }

    /// The planar distance from `q` (plane coordinates) to the curve and the
    /// foot's parameter: the least over the ends and every root in [0, 1] of
    /// the stationarity `g(t) = (B(t) − q)·B'(t)`, a quintic, every root
    /// isolated by subdividing its Bernstein form (a span whose coefficients
    /// keep one sign holds none; one sign change, exactly one, bisected to
    /// the last bit; spans that halve to nothing still undecided, a double
    /// root, read at their middle).
    pub fn distance(&self, q: [f64; 2]) -> (f64, f64) {
        let at = |t: f64| {
            let r = sub(self.point(t), q);
            dot(r, r)
        };
        let mut best = (at(0.0), 0.0);
        let end = at(1.0);
        if end < best.0 {
            best = (end, 1.0);
        }
        for t in self.stationary(q) {
            let d2 = at(t);
            if d2 < best.0 {
                best = (d2, t);
            }
        }
        (best.0.sqrt(), best.1)
    }

    /// Every root in [0, 1] of `g(t) = (B(t) − q)·B'(t)` (see [`Self::distance`]).
    fn stationary(&self, q: [f64; 2]) -> Vec<f64> {
        // Power coefficients of B − q and B', per coordinate.
        let c = &self.curve;
        let mut g = [0.0f64; 6];
        for k in 0..2 {
            let (p0, p1, p2, p3) = (c[0][k], c[1][k], c[2][k], c[3][k]);
            let a = [p0 - q[k], 3.0 * (p1 - p0), 3.0 * (p2 - 2.0 * p1 + p0), p3 - 3.0 * p2 + 3.0 * p1 - p0];
            let d = [a[1], 2.0 * a[2], 3.0 * a[3]];
            for i in 0..4 {
                for j in 0..3 {
                    g[i + j] += a[i] * d[j];
                }
            }
        }
        // To the Bernstein basis of degree 5: b_k = Σ_{i ≤ k} C(k, i) / C(5, i) · g_i.
        let choose = |n: usize, r: usize| -> f64 { (0..r).fold(1.0, |acc, i| acc * (n - i) as f64 / (i + 1) as f64) };
        let mut bern = [0.0f64; 6];
        for k in 0..6 {
            bern[k] = (0..=k).map(|i| choose(k, i) / choose(5, i) * g[i]).sum();
        }
        let value = |t: f64| g.iter().rev().fold(0.0, |acc, &x| acc * t + x);
        // A stationarity that vanishes everywhere (a curve standing still) has
        // no roots to isolate; the ends answer.
        let size = bern.iter().fold(0.0f64, |m, b| m.max(b.abs()));
        if !(size > 0.0 && size.is_finite()) {
            return Vec::new();
        }
        let mut roots = Vec::new();
        let mut spans = 0usize;
        let mut stack: Vec<([f64; 6], f64, f64, u32)> = vec![(bern, 0.0, 1.0, 0)];
        while let Some((b, lo, hi, depth)) = stack.pop() {
            // A quintic has at most five roots; a few hundred spans isolate them.
            spans += 1;
            if spans > 4096 {
                roots.push(0.5 * (lo + hi));
                continue;
            }
            let changes = (0..5).filter(|&i| (b[i] < 0.0) != (b[i + 1] < 0.0) || b[i] == 0.0).count() + usize::from(b[5] == 0.0);
            if changes == 0 {
                continue;
            }
            let (glo, ghi) = (value(lo), value(hi));
            if changes == 1 && (glo < 0.0) != (ghi < 0.0) {
                // One root: bisect it to the last bit.
                let (mut a, mut z, mut ga) = (lo, hi, glo);
                for _ in 0..200 {
                    let m = 0.5 * (a + z);
                    if m <= a || m >= z {
                        break;
                    }
                    let gm = value(m);
                    if gm == 0.0 {
                        a = m;
                        z = m;
                        break;
                    }
                    if (gm < 0.0) == (ga < 0.0) {
                        a = m;
                        ga = gm;
                    } else {
                        z = m;
                    }
                }
                // Polished on the stable form: (B(t) − q)·B'(t) by de
                // Casteljau, not the power coefficients (which cancel).
                let mut t = 0.5 * (a + z);
                for _ in 0..4 {
                    let r = sub(self.point(t), q);
                    let d = self.derivative(t);
                    let slope = dot(d, d) + dot(r, self.second(t));
                    if !(slope != 0.0 && slope.is_finite()) {
                        break;
                    }
                    let next = t - dot(r, d) / slope;
                    if !(next >= lo && next <= hi) {
                        break;
                    }
                    t = next;
                }
                roots.push(t);
                continue;
            }
            if depth >= 60 || hi - lo <= 1e-15 {
                // Undecided to the last bit (a focal point's double root).
                roots.push(0.5 * (lo + hi));
                continue;
            }
            // De Casteljau at the middle.
            let mut left = [0.0f64; 6];
            let mut right = [0.0f64; 6];
            let mut work = b;
            for level in 0..6 {
                left[level] = work[0];
                right[5 - level] = work[5 - level];
                for i in 0..5 - level {
                    work[i] = 0.5 * (work[i] + work[i + 1]);
                }
            }
            let mid = 0.5 * (lo + hi);
            stack.push((right, mid, hi, depth + 1));
            stack.push((left, lo, mid, depth + 1));
        }
        roots
    }

    /// Whether the tool passes through `p`: its planar distance to the curve
    /// within the tool's radius at its height.
    pub fn contains(&self, p: Vec3) -> bool {
        let r = p.sub(self.origin);
        let h = r.dot(self.up);
        if !(h >= self.heights[0] && h <= self.heights[1]) {
            return false;
        }
        // A zero radius (an apex on the axis) holds the curve itself.
        self.distance([r.dot(self.x), r.dot(self.y)]).0 <= self.radius(h)
    }

    /// The outlines of the planar stroke `{ d <= rho }`: closed loops of
    /// points on its boundary, each exactly at distance `rho` (bisected on a
    /// grid of `cell`, to 1e-13 of the cell), the outer ones counter-clockwise
    /// and the holes clockwise (marching squares, a saddle resolved by its
    /// cell's middle). They resolve the stroke's features larger than a cell:
    /// a hole or a neck narrower than one may be missed, which moves the area
    /// by less than a cell's square (inside its stated O(cell²) error). The
    /// solid's topology is not read from them.
    pub fn level_loops(&self, rho: f64, cell: f64) -> Vec<Vec<[f64; 2]>> {
        if !(cell > 0.0 && cell.is_finite() && rho >= 0.0 && rho.is_finite()) {
            return Vec::new();
        }
        let (lo, hi) = self.curve.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), c| {
            ([lo[0].min(c[0]), lo[1].min(c[1])], [hi[0].max(c[0]), hi[1].max(c[1])])
        });
        let pad = rho + 2.0 * cell;
        let (x0, y0) = (lo[0] - pad, lo[1] - pad);
        let (wx, wy) = (((hi[0] - lo[0] + 2.0 * pad) / cell).ceil(), ((hi[1] - lo[1] + 2.0 * pad) / cell).ceil());
        // A grid of at most 1e8 nodes, else no outline.
        if !(wx.is_finite() && wy.is_finite() && (wx + 1.0) * (wy + 1.0) <= 1e8) {
            return Vec::new();
        }
        let (nx, ny) = (wx as usize + 1, wy as usize + 1);
        let node = |i: usize, j: usize| [x0 + cell * i as f64, y0 + cell * j as f64];
        let f = |p: [f64; 2]| self.distance(p).0 - rho;
        let values: Vec<f64> = (0..ny).flat_map(|j| (0..nx).map(move |i| (i, j))).map(|(i, j)| f(node(i, j))).collect();
        let value = |i: usize, j: usize| values[j * nx + i];
        // The level point on the grid edge between two nodes (inside first).
        let crossing = |a: [f64; 2], b: [f64; 2]| -> [f64; 2] {
            let (mut fa, mut lo, mut hi) = (f(a), 0.0f64, 1.0f64);
            for _ in 0..60 {
                let m = 0.5 * (lo + hi);
                let fm = f([a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m]);
                if (fm <= 0.0) == (fa <= 0.0) {
                    lo = m;
                    fa = fm;
                } else {
                    hi = m;
                }
            }
            let m = 0.5 * (lo + hi);
            [a[0] + (b[0] - a[0]) * m, a[1] + (b[1] - a[1]) * m]
        };
        // Edges keyed by (cell edge id): horizontal (i, j)->(i+1, j) as 2*(j*nx+i), vertical as +1.
        let h_edge = |i: usize, j: usize| 2 * (j * nx + i);
        let v_edge = |i: usize, j: usize| 2 * (j * nx + i) + 1;
        let mut next: std::collections::HashMap<usize, usize> = std::collections::HashMap::new();
        for j in 0..ny - 1 {
            for i in 0..nx - 1 {
                let inside = [value(i, j) <= 0.0, value(i + 1, j) <= 0.0, value(i + 1, j + 1) <= 0.0, value(i, j + 1) <= 0.0];
                // Cell edges in counter-clockwise order: bottom, right, top, left.
                let edges = [h_edge(i, j), v_edge(i + 1, j), h_edge(i, j + 1), v_edge(i, j)];
                // A segment goes from the edge where the walk leaves the inside to where it re-enters,
                // keeping the inside on its left (counter-clockwise outer loops).
                let mut exits: Vec<usize> = Vec::new();
                let mut entries: Vec<usize> = Vec::new();
                for k in 0..4 {
                    let (a, b) = (inside[k], inside[(k + 1) % 4]);
                    if a && !b {
                        exits.push(k);
                    } else if !a && b {
                        entries.push(k);
                    }
                }
                match (exits.as_slice(), entries.as_slice()) {
                    ([e], [n]) => {
                        next.insert(edges[*n], edges[*e]);
                    }
                    ([e0, e1], [n0, n1]) => {
                        // A saddle: the middle decides which corners join.
                        let middle = f([x0 + cell * (i as f64 + 0.5), y0 + cell * (j as f64 + 0.5)]) <= 0.0;
                        // Entries at n pair with the exit after them (inside joined) or before.
                        let pair = |n: usize, exits: [usize; 2]| -> usize {
                            let after = |e: usize| (e + 4 - n) % 4;
                            if middle { *exits.iter().max_by_key(|&&e| after(e)).unwrap() } else { *exits.iter().min_by_key(|&&e| after(e)).unwrap() }
                        };
                        next.insert(edges[*n0], edges[pair(*n0, [*e0, *e1])]);
                        next.insert(edges[*n1], edges[pair(*n1, [*e0, *e1])]);
                    }
                    _ => {}
                }
            }
        }
        // The points: each crossing edge's level point.
        let edge_point = |id: usize| -> [f64; 2] {
            let (cell_id, vertical) = (id / 2, id % 2 == 1);
            let (i, j) = (cell_id % nx, cell_id / nx);
            let (a, b) = if vertical { (node(i, j), node(i, j + 1)) } else { (node(i, j), node(i + 1, j)) };
            if f(a) <= 0.0 { crossing(a, b) } else { crossing(b, a) }
        };
        let mut loops = Vec::new();
        let mut seen: std::collections::HashSet<usize> = std::collections::HashSet::new();
        let mut starts: Vec<usize> = next.keys().copied().collect();
        starts.sort_unstable();
        for start in starts {
            if seen.contains(&start) {
                continue;
            }
            let mut ring = Vec::new();
            let mut at = start;
            while seen.insert(at) {
                ring.push(edge_point(at));
                match next.get(&at) {
                    Some(&n) => at = n,
                    None => break,
                }
            }
            if ring.len() >= 3 {
                // The walk keeps the inside on its right: reversed, the outer
                // loops run counter-clockwise and the holes clockwise.
                ring.reverse();
                loops.push(ring);
            }
        }
        loops
    }

    /// The area of the stroke `{ d <= rho }` from its outlines at `cell`
    /// (signed shoelace: outer loops counter-clockwise, holes clockwise).
    /// `None` where the grid is not one [`Self::level_loops`] reads (a cell not
    /// positive and finite, or past its node budget).
    pub fn level_area(&self, rho: f64, cell: f64) -> Option<f64> {
        if !self.grid_fits(rho, cell) {
            return None;
        }
        Some(
            self.level_loops(rho, cell)
                .iter()
                .map(|ring| {
                    let n = ring.len();
                    0.5 * (0..n).map(|k| ring[k][0] * ring[(k + 1) % n][1] - ring[(k + 1) % n][0] * ring[k][1]).sum::<f64>()
                })
                .sum(),
        )
    }

    /// Whether [`Self::level_loops`] reads a grid of `cell` for `rho` (positive,
    /// finite, at most a quarter of `rho`, within its 1e8-node budget).
    fn grid_fits(&self, rho: f64, cell: f64) -> bool {
        // A cell no coarser than a quarter of the radius (a coarser grid can
        // step over a whole stroke and read it as nothing).
        if !(cell > 0.0 && cell.is_finite() && rho >= 0.0 && rho.is_finite() && cell <= 0.25 * rho) {
            return false;
        }
        let (lo, hi) = self.curve.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), c| {
            ([lo[0].min(c[0]), lo[1].min(c[1])], [hi[0].max(c[0]), hi[1].max(c[1])])
        });
        let pad = rho + 2.0 * cell;
        let (wx, wy) = (((hi[0] - lo[0] + 2.0 * pad) / cell).ceil(), ((hi[1] - lo[1] + 2.0 * pad) / cell).ceil());
        wx.is_finite() && wy.is_finite() && (wx + 1.0) * (wy + 1.0) <= 1e8
    }

    /// The volume the tool passes through between heights `h0` and `h1`: the
    /// stroke areas at the tool's radius integrated over the height, piece by
    /// piece of the meridian's chain in the piece's own parameter (a line's
    /// length, an arc's angle: there the radius and the height are smooth, as
    /// they are not in the height where an arc meets a flat end), each piece
    /// in four spans of 8-point Gauss–Legendre, the areas from outlines at `cell`.
    /// `None` where a stroke's grid is not read (see [`Self::level_area`]).
    pub fn volume(&self, h0: f64, h1: f64, cell: f64) -> Option<f64> {
        let (a, b) = (h0.max(self.heights[0]), h1.min(self.heights[1]));
        if !(b > a) {
            return Some(0.0);
        }
        if !self.grid_fits(self.widest(), cell) {
            return None;
        }
        // Each stroke's own grid is checked where it is read (a radius too
        // small for the cell reads None, and the volume with it).
        const X: [f64; 4] = [0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
        const W: [f64; 4] = [0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];
        let gauss = |f: &dyn Fn(f64) -> f64, lo: f64, hi: f64| -> f64 {
            let mut total = 0.0;
            for k in 0..4 {
                let (s0, s1) = (lo + (hi - lo) * k as f64 / 4.0, lo + (hi - lo) * (k + 1) as f64 / 4.0);
                let (mid, half) = (0.5 * (s0 + s1), 0.5 * (s1 - s0));
                for i in 0..4 {
                    total += half * W[i] * (f(mid - half * X[i]) + f(mid + half * X[i]));
                }
            }
            total
        };
        let mut total = 0.0;
        for piece in &self.meridian {
            match *piece {
                OutlinePiece::Line { from, to } => {
                    // Off the axis and rising or falling: its height range clipped to
                    // [a, b] (the axis piece, by the gate's own tolerance, sweeps nothing).
                    let tol_r = 1e-12 * self.widest();
                    if from[0].abs() <= tol_r && to[0].abs() <= tol_r {
                        continue;
                    }
                    let dh = to[1] - from[1];
                    if dh == 0.0 {
                        continue;
                    }
                    let (s0, s1) = (((a - from[1]) / dh).clamp(0.0, 1.0), ((b - from[1]) / dh).clamp(0.0, 1.0));
                    let (lo, hi) = (s0.min(s1), s0.max(s1));
                    if hi <= lo {
                        continue;
                    }
                    let radius = |s: f64| from[0] + s * (to[0] - from[0]);
                    // A stretch reaching the axis: its strokes shrink to the curve, read
                    // as the radius times the length plus the disc, by the closed form
                    // only where the grid cannot read them.
                    // A side whose radius holds: one stroke for the whole stretch.
                    if from[0] == to[0] {
                        total += dh.abs() * (hi - lo) * self.level_area(from[0], cell)?;
                    } else {
                        total += dh.abs() * gauss(&|s| self.level_area(radius(s), cell).unwrap_or(f64::NAN), lo, hi);
                    }
                }
                OutlinePiece::Arc { centre, radius, start, end } => {
                    // θ over the arc where its height lies in [a, b] (the arc
                    // holds no vertical tangent inside, so height is monotone in θ).
                    let (t0, t1) = (start.min(end), start.max(end));
                    let height = |t: f64| centre[1] + radius * t.sin();
                    let clip = |h: f64| -> f64 {
                        // The θ in [t0, t1] where the height is h (bisection on the monotone height).
                        let (mut lo, mut hi) = (t0, t1);
                        let rising = height(t1) > height(t0);
                        for _ in 0..200 {
                            let m = 0.5 * (lo + hi);
                            if (height(m) < h) == rising { lo = m } else { hi = m }
                        }
                        0.5 * (lo + hi)
                    };
                    let (ha, hb) = (height(t0), height(t1));
                    let (hlo, hhi) = (ha.min(hb), ha.max(hb));
                    let (ca, cb) = (a.max(hlo), b.min(hhi));
                    if cb <= ca {
                        continue;
                    }
                    let (u0, u1) = (clip(ca), clip(cb));
                    let (lo, hi) = (u0.min(u1), u0.max(u1));
                    let f = |t: f64| self.level_area(centre[0] + radius * t.cos(), cell).unwrap_or(f64::NAN) * (radius * t.cos()).abs();
                    total += gauss(&f, lo, hi);
                }
                OutlinePiece::Ellipse { .. } => {}
            }
        }
        total.is_finite().then_some(total)
    }

    /// The section's reach as its curves' control points place it: the
    /// greatest radius of the meridian's pieces' control points (built as the
    /// sweep builds them).
    fn section_reach(&self) -> f64 {
        let (o, x, z) = (Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 1.0));
        self.meridian
            .iter()
            .filter_map(|piece| world_piece(piece, o, x, z).ok())
            .flat_map(|curve| curve.control_points.into_iter())
            .filter_map(|c| c.point().ok())
            .map(|p| p.x.abs())
            .fold(0.0f64, f64::max)
    }

    /// The tool's widest radius.
    fn widest(&self) -> f64 {
        self.meridian.iter().map(|piece| piece_extent(piece).0).fold(0.0f64, f64::max)
    }
}

/// A meridian piece in the world: in the plane through `foot` spanned by the
/// radial direction `out` (the piece's radius along it) and `up` (its height
/// above the curve's plane), run as written.
fn world_piece(piece: &OutlinePiece, foot: Vec3, out: Vec3, up: Vec3) -> Result<crate::NurbsCurve, String> {
    let at = |p: [f64; 2]| foot.add(out.scale(p[0])).add(up.scale(p[1]));
    match *piece {
        OutlinePiece::Line { from, to } => crate::make_line(at(from), at(to)),
        OutlinePiece::Arc { centre, radius, start, end } => {
            let c = at(centre);
            if end > start {
                crate::make_arc(c, out, up, radius, start, end)
            } else {
                crate::make_arc(c, out, up, radius, end, start)?.reversed()
            }
        }
        OutlinePiece::Ellipse { .. } => Err("an elliptic meridian piece".into()),
    }
}

impl UprightSlot {
    /// The meridian's chain (every piece but the one along the axis), in order
    /// from the axis piece's end round to its start ([`Self::sections_are_discs`]).
    fn chain(&self) -> Option<Vec<OutlinePiece>> {
        let tol_r = 1e-12 * self.widest();
        let along_axis = |piece: &OutlinePiece| matches!(*piece, OutlinePiece::Line { from, to } if from[0].abs() <= tol_r && to[0].abs() <= tol_r);
        let axis_index = self.meridian.iter().position(along_axis)?;
        let n = self.meridian.len();
        Some((1..n).map(|k| self.meridian[(axis_index + k) % n].clone()).collect())
    }

    /// The tool where the curve has carried it to parameter `t`: its meridian
    /// turned about the upright axis through B(t).
    fn tool_at(&self, t: f64) -> Result<crate::BrepSolid, crate::KernelRefusal> {
        let refuse = |message: String| crate::KernelRefusal::internal(crate::KernelStage::Collect, "upright_stroke", format!("uprightStroke: {message}"));
        let p = self.point(t);
        let foot = self.origin.add(self.x.scale(p[0])).add(self.y.scale(p[1]));
        let curves: Vec<crate::NurbsCurve> = self.meridian.iter().map(|piece| world_piece(piece, foot, self.x, self.up)).collect::<Result<_, _>>().map_err(refuse)?;
        // The revolve builder's own refusal (a profile it reads as touching the
        // axis, at its absolute 1e-3) is the construction's range, by name.
        crate::revolve_profile_brep(&curves, foot, self.up, std::f64::consts::TAU).map_err(|e| {
            crate::KernelRefusal::input(crate::KernelStage::Collect, "upright_scale", format!("uprightStroke: the tool is outside the construction's range: the revolve builder declined it ({e})"))
        })
    }

    /// The tool's axial section at parameter `t` as a closed loop in the
    /// vertical plane square to the curve there: the chain on the left of the
    /// curve, then its mirror on the right, back to the start.
    fn section_at(&self, t: f64) -> Result<Vec<crate::NurbsCurve>, String> {
        let chain = self.chain().ok_or("no axis piece")?;
        let p = self.point(t);
        let d = self.derivative(t);
        let length = dot(d, d).sqrt();
        if !(length > 0.0) {
            return Err("the curve stops".into());
        }
        let normal = [-d[1] / length, d[0] / length];
        let foot = self.origin.add(self.x.scale(p[0])).add(self.y.scale(p[1]));
        let left = self.x.scale(normal[0]).add(self.y.scale(normal[1]));
        let mut loop_curves: Vec<crate::NurbsCurve> = chain.iter().map(|piece| world_piece(piece, foot, left, self.up)).collect::<Result<_, _>>()?;
        let mirrored: Vec<crate::NurbsCurve> = chain.iter().rev().map(|piece| world_piece(piece, foot, left.scale(-1.0), self.up).and_then(|c| c.reversed())).collect::<Result<_, _>>()?;
        loop_curves.extend(mirrored);
        Ok(loop_curves)
    }

    /// The cubic's piece over [a, b] as a world curve (de Casteljau).
    fn world_piece_of_curve(&self, a: f64, b: f64) -> Result<crate::NurbsCurve, String> {
        let split = |c: [[f64; 2]; 4], t: f64| -> ([[f64; 2]; 4], [[f64; 2]; 4]) {
            let l = |x: [f64; 2], y: [f64; 2]| [x[0] + (y[0] - x[0]) * t, x[1] + (y[1] - x[1]) * t];
            let (p01, p12, p23) = (l(c[0], c[1]), l(c[1], c[2]), l(c[2], c[3]));
            let (p012, p123) = (l(p01, p12), l(p12, p23));
            let m = l(p012, p123);
            ([c[0], p01, p012, m], [m, p123, p23, c[3]])
        };
        let right = if a > 0.0 { split(self.curve, a).1 } else { self.curve };
        let piece = if b < 1.0 { split(right, (b - a) / (1.0 - a)).0 } else { right };
        let world = |p: [f64; 2]| crate::Vec4 { x: self.origin.x + self.x.x * p[0] + self.y.x * p[1], y: self.origin.y + self.x.y * p[0] + self.y.y * p[1], z: self.origin.z + self.x.z * p[0] + self.y.z * p[1], w: 1.0 };
        crate::NurbsCurve::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], piece.iter().map(|p| world(*p)).collect())
    }

    /// The full section carried along [a, b] (no end tools).
    fn section_sweep(&self, a: f64, b: f64) -> Result<crate::BrepSolid, crate::KernelRefusal> {
        let refuse = |message: String| crate::KernelRefusal::internal(crate::KernelStage::Collect, "upright_stroke", format!("uprightStroke: {message}"));
        let section = self.section_at(a).map_err(refuse)?;
        let path = self.world_piece_of_curve(a, b).map_err(refuse)?;
        let chain = crate::SweepPath::new(vec![path], Vec::new())?;
        crate::sweep_profile_along_chain(&section, &chain, 0.0, None, None, crate::SectionPlacement::Rigid, "an upright tool's section along one piece of a cubic")
    }

    /// The curve's pieces: tight where the tool's widest radius may reach past
    /// the curve's radius of curvature (1% to spare; bounded, not sampled),
    /// gentle elsewhere, each cut where its turning reaches 150° (no piece
    /// crosses itself). (a, b, tight).
    pub fn pieces(&self) -> Vec<(f64, f64, bool)> {
        // Tightness against the section's reach as the section sweep reads it:
        // its curves' control points (an arc's lie outside the arc), no less
        // than the tool's own widest radius.
        let widest = self.section_reach().max(self.widest());
        // Tight spans, found without missing one: on each span an upper bound
        // of the curvature |B'×B''|/|B'|³ from its middle and the Lipschitz
        // bounds of the cross (its derivative B'×B''' ≤ |B'|max·|B'''|) and of
        // the speed (|B''|max), bisected while the bound reaches the tool but
        // the middle does not; a span the bound cannot clear is tight.
        let c = &self.curve;
        let polygon = (0..3).map(|k| 3.0 * dot(sub(c[k + 1], c[k]), sub(c[k + 1], c[k])).sqrt()).fold(0.0f64, f64::max);
        let second_most = { let (a, b) = (self.second(0.0), self.second(1.0)); dot(a, a).sqrt().max(dot(b, b).sqrt()) };
        let third = { let d = sub(self.second(1.0), self.second(0.0)); dot(d, d).sqrt() };
        let cross = |t: f64| { let (d, dd) = (self.derivative(t), self.second(t)); (d[0] * dd[1] - d[1] * dd[0]).abs() };
        let speed = |t: f64| dot(self.derivative(t), self.derivative(t)).sqrt();
        let threshold = 0.99 / widest;
        let may_be_tight = |a: f64, b: f64| -> bool {
            let (m, h) = (0.5 * (a + b), 0.5 * (b - a));
            let low = speed(m) - second_most * h;
            if !(low > 0.0) {
                return true;
            }
            (cross(m) + polygon * third * h) / (low * low * low) >= threshold
        };
        let is_tight_at = |t: f64| cross(t) / speed(t).powi(3) >= threshold;
        let mut tight_spans: Vec<(f64, f64)> = Vec::new();
        let mut stack: Vec<(f64, f64, u32)> = (0..256).rev().map(|k| (k as f64 / 256.0, (k + 1) as f64 / 256.0, 0)).collect();
        // A budget of spans read (the envelope's regular curve needs few); past
        // it every span left is taken as tight.
        let mut read = 0usize;
        while let Some((a, b, depth)) = stack.pop() {
            read += 1;
            if !may_be_tight(a, b) {
                continue;
            }
            // Decided tight where its middle is, or where it cannot be split further.
            if read > 1 << 20 || is_tight_at(0.5 * (a + b)) || depth >= 40 {
                match tight_spans.last_mut() {
                    Some(last) if (last.1 - a).abs() <= 1e-15 => last.1 = b,
                    _ => tight_spans.push((a, b)),
                }
                continue;
            }
            let m = 0.5 * (a + b);
            stack.push((m, b, depth + 1));
            stack.push((a, m, depth + 1));
        }
        // Runs: the tight spans and the gentle stretches between them.
        let mut runs: Vec<(f64, f64, bool)> = Vec::new();
        let mut at = 0.0;
        for (a, b) in tight_spans {
            if a > at {
                runs.push((at, a, false));
            }
            runs.push((a, b, true));
            at = b;
        }
        if at < 1.0 {
            runs.push((at, 1.0, false));
        }
        // Each run cut where its turning reaches 150° (a planar curve that
        // turns less than half a turn cannot cross itself).
        let heading = |t: f64| {
            let d = self.derivative(t);
            d[1].atan2(d[0])
        };
        let most = 150f64.to_radians();
        let mut pieces = Vec::new();
        for (a, b, is_tight) in runs {
            if !(b > a) {
                continue;
            }
            let steps = 2048;
            let mut turn = 0.0;
            let mut previous = heading(a);
            let mut start = a;
            for k in 1..=steps {
                let t = a + (b - a) * k as f64 / steps as f64;
                let h = heading(t);
                let mut delta = h - previous;
                while delta > std::f64::consts::PI { delta -= std::f64::consts::TAU; }
                while delta < -std::f64::consts::PI { delta += std::f64::consts::TAU; }
                turn += delta.abs();
                previous = h;
                if turn >= most && k < steps {
                    pieces.push((start, t, is_tight));
                    start = t;
                    turn = 0.0;
                }
            }
            pieces.push((start, b, is_tight));
        }
        pieces
    }

    /// Everything the tool passes through along the whole curve, where it
    /// turns nowhere tighter than the tool reaches: the tool at every break
    /// and each piece's section sweep, united (the union also answers the
    /// curve crossing itself). A tighter turn is refused by name.
    pub fn solid(&self) -> Result<crate::BrepSolid, crate::KernelRefusal> {
        // The construction's own range (its curves and booleans read absolute
        // tolerances): the tool's widest radius within [5e-2, 1e6], every arc
        // at least 1e-6.
        let small_arc = self.meridian.iter().any(|piece| matches!(*piece, OutlinePiece::Arc { radius, .. } if radius < 1e-6));
        let widest = self.widest();
        if small_arc || !(5e-2..=1e6).contains(&widest) {
            return Err(crate::KernelRefusal::input(
                crate::KernelStage::Collect,
                "upright_scale",
                format!("uprightStroke: a tool {widest:e} wide is outside the construction's range (5e-2 to 1e6, its arcs at least 1e-6: the revolve builder reads 1e-3 from the axis)"),
            ));
        }
        let pieces = self.pieces();
        // A turn tighter than the tool folds its concave wall; the trimmed
        // fold is not built yet (the route refuses rather than guess it).
        if let Some(&(a, b, _)) = pieces.iter().find(|p| p.2) {
            return Err(crate::KernelRefusal::unsupported(
                crate::KernelStage::Collect,
                "upright_tight_turn",
                format!("uprightStroke: the curve turns tighter than the tool reaches over [{a:.4}, {b:.4}]; its folded concave wall is not built"),
            ));
        }
        // The tools at every break first, one after another (overlapping
        // tools join cleanly with each other), then each piece's section
        // sweep (the union answers the curve crossing itself).
        let mut whole = self.tool_at(0.0)?;
        for (index, &(_, b, _)) in pieces.iter().enumerate() {
            whole = crate::boolean_operation_nary(&[whole, self.tool_at(b)?], crate::BooleanOperation::Union)
                .map_err(|e| e.with_message(|m| format!("{m} (adding the tool at break {index}, t = {b})")))?;
        }
        for (index, &(a, b, _)) in pieces.iter().enumerate() {
            whole = crate::boolean_operation_nary(&[whole, self.section_sweep(a, b)?], crate::BooleanOperation::Union)
                .map_err(|e| e.with_message(|m| format!("{m} (adding sweep {index} of {}, [{a}, {b}])", pieces.len())))?;
        }
        Ok(whole)
    }

    /// What the tool passes through over the curve's piece [a, b], where the
    /// piece turns nowhere tighter than the tool reaches and does not cross
    /// itself: its axial section carried by the curve's own rigid motion (the
    /// section sweep, which refuses a bend tighter than the section), and the
    /// tool itself at the two ends, united.
    pub fn stroke(&self, a: f64, b: f64) -> Result<crate::BrepSolid, crate::KernelRefusal> {
        let refuse = |message: String| crate::KernelRefusal::internal(crate::KernelStage::Collect, "upright_stroke", format!("uprightStroke: {message}"));
        if !(0.0 <= a && a < b && b <= 1.0) {
            return Err(crate::KernelRefusal::input(crate::KernelStage::Collect, "upright_stroke_range", format!("uprightStroke: the piece [{a}, {b}] is not within the curve's [0, 1]")));
        }
        let section = self.section_at(a).map_err(refuse)?;
        let path = self.world_piece_of_curve(a, b).map_err(refuse)?;
        let chain = crate::SweepPath::new(vec![path], Vec::new())?;
        let swept = crate::sweep_profile_along_chain(&section, &chain, 0.0, None, None, crate::SectionPlacement::Rigid, "an upright tool's section along one piece of a cubic")?;
        crate::boolean_operation_nary(&[swept, self.tool_at(a)?, self.tool_at(b)?], crate::BooleanOperation::Union)
    }
}

