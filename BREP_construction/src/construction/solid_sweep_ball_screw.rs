//! A ball SCREWED along a helix that curves tighter than the ball, with its
//! turns overlapping (`BREP_SWEEP_BALL_SCREW`, default on; `=0` restores the
//! level engine): everything within the ball's radius r of the helix's arc,
//! built from the arc's distance itself, without the level engine (which
//! refuses the fold).
//!
//! The solid's boundary lies on the tube about the helix, P(t, θ) = c(t) +
//! r(cos θ N + sin θ B) in the helix's Frenet frame, and on the two end balls.
//! A tube point is on the boundary where no point of the arc is nearer than r:
//! - Across the fold (r cos θ > ρ, ρ the helix's radius of curvature) the
//!   neighbouring circles hold the tube: the strip |θ| < θf, cos θf = ρ / r, is
//!   inside all along; it is left out of the face, whose edge it touches at
//!   two corners only (the cusps below).
//! - Where the next turn lies beside a point, its tube holds the point beyond
//!   one angle. The screw carries the helix onto itself, so that angle is one
//!   constant θu on the side of the turn ahead (the turn at t + Δ, Δ the lag
//!   found with it) and −θu on the side of the turn behind: the tube meets
//!   itself along one helix, P(t, θu) = P(t + Δ, −θu).
//! - Near the ends, where there is no turn ahead (behind), the end ball holds
//!   the point instead: |P − c(end)| = r, one equation in θ in closed form
//!   (A cos θ + B sin θ = C). That curve leaves the end circle exactly where
//!   the fold does (θ = ∓θf), so the face meets each end ball at a corner on
//!   the fold line (a cusp of zero angle).
//! So the body is one tube face (cut here in quarter turns, so each piece's
//! carrier has few spans), the start ball's back less the tube's, and the end
//! ball's front less the tube's: 4 vertices, 5 edges and 3 faces before the
//! quarter-turn cuts. The layout is checked against the arc's own distance on
//! a grid of tube and ball points before it is built; anything else (a left
//! hand helix, turns that do not overlap, no fold, a third turn reaching) is
//! declined (None), for the level engine to answer as before.
use super::*;

/// The switch (default on; `BREP_SWEEP_BALL_SCREW=0` restores the level engine).
pub(super) fn ball_screw_on() -> bool {
    crate::switch!("BREP_SWEEP_BALL_SCREW", true)
}

/// Rows of each tube piece's carrier, and the stations of each edge piece.
const ROWS: usize = 33;
const STATIONS: usize = 48;

/// The helix of the ball's centre, in its own frame: c(t) = o + a(cos t ex +
/// sin t ey) + b t ez, t in [0, span].
struct Helix {
    o: Vec3,
    ex: Vec3,
    ey: Vec3,
    ez: Vec3,
    a: f64,
    b: f64,
    r: f64,
    speed: f64,
    span: f64,
}

impl Helix {
    fn c(&self, t: f64) -> Vec3 {
        self.o.add(self.ex.scale(self.a * t.cos())).add(self.ey.scale(self.a * t.sin())).add(self.ez.scale(self.b * t))
    }

    fn d(&self, t: f64) -> Vec3 {
        self.ex.scale(-self.a * t.sin()).add(self.ey.scale(self.a * t.cos())).add(self.ez.scale(self.b))
    }

    /// (T, N, B) at t: N toward the axis.
    fn frame(&self, t: f64) -> (Vec3, Vec3, Vec3) {
        let tangent = self.d(t).scale(1.0 / self.speed);
        let normal = self.ex.scale(-t.cos()).add(self.ey.scale(-t.sin()));
        (tangent, normal, tangent.cross(normal))
    }

    /// The tube point at t and angle θ from N toward B.
    fn p(&self, t: f64, theta: f64) -> Vec3 {
        let (_, n, b) = self.frame(t);
        self.c(t).add(n.scale(self.r * theta.cos())).add(b.scale(self.r * theta.sin()))
    }

    /// The least distance from x to the arc over [lo, hi], and where: by
    /// samples every 0.05 rad, then golden sections about the least.
    fn nearest(&self, x: Vec3, lo: f64, hi: f64) -> (f64, f64) {
        let n = (((hi - lo) / 0.05).ceil() as usize).max(2);
        let at = |k: usize| lo + (hi - lo) * k as f64 / n as f64;
        let mut best = (f64::INFINITY, 0usize);
        for k in 0..=n {
            let d = x.sub(self.c(at(k))).length();
            if d < best.0 {
                best = (d, k);
            }
        }
        let (mut a, mut b) = (at(best.1.saturating_sub(1)), at((best.1 + 1).min(n)));
        let g = 0.5 * (5f64.sqrt() - 1.0);
        for _ in 0..60 {
            let (p, q) = (b - g * (b - a), a + g * (b - a));
            if x.sub(self.c(p)).length() < x.sub(self.c(q)).length() {
                b = q;
            } else {
                a = p;
            }
        }
        // Newton on (c(s) − x)·c'(s) = 0 from there, kept within the bracket.
        let mut s = 0.5 * (a + b);
        for _ in 0..4 {
            let (e, d) = (self.c(s).sub(x), self.d(s));
            let dd = self.ex.scale(-self.a * s.cos()).add(self.ey.scale(-self.a * s.sin()));
            let slope = d.dot(d) + e.dot(dd);
            if !(slope > 0.0) {
                break;
            }
            let next = s - e.dot(d) / slope;
            if !(next >= lo && next <= hi) {
                break;
            }
            s = next;
        }
        (x.sub(self.c(s)).length().min(best.0), s)
    }

    /// The least distance from x, a point within r of c(t), to the arc: only
    /// the part of it within 2r of c(t)'s height can come within r of x.
    fn nearest_about(&self, x: Vec3, t: f64) -> (f64, f64) {
        let reach = 2.0 * self.r / self.b + 0.1;
        self.nearest(x, (t - reach).max(0.0), (t + reach).min(self.span))
    }

    /// The angle θ near `near` where the tube circle at t meets the ball at
    /// `centre`: (c(t) − centre)·u(θ) = −|c(t) − centre|² / 2r.
    fn on_ball(&self, t: f64, centre: Vec3, near: f64) -> Option<f64> {
        let (_, n, b) = self.frame(t);
        let d = self.c(t).sub(centre);
        let (p, q, c) = (n.dot(d), b.dot(d), -d.dot(d) / (2.0 * self.r));
        let reach = p.hypot(q);
        if !(reach > 0.0) || (c / reach).abs() > 1.0 {
            return None;
        }
        let (base, spread) = (q.atan2(p), (c / reach).acos());
        let wrap = |x: f64| x - TAU * ((x - near) / TAU).round();
        let (x, y) = (wrap(base + spread), wrap(base - spread));
        Some(if (x - near).abs() < (y - near).abs() { x } else { y })
    }
}

/// The face layout in (t, ψ), ψ = −θ mod 2π the tube carrier's angle (from N
/// toward −B, so its normal points out): the face lies between `lower` and
/// `upper` at every t.
struct Layout {
    theta_u: f64,
    lag: f64,
    fold: f64,
}

impl Layout {
    fn lower(&self, h: &Helix, t: f64) -> f64 {
        if t >= self.lag {
            return self.theta_u;
        }
        if t <= 0.0 {
            return self.fold;
        }
        // The branch nearer the straight line from the fold (t = 0) to the
        // helix edge (t = Δ), which `layout` checked the curve keeps to.
        let near = -self.fold + (self.fold - self.theta_u) * t / self.lag;
        -h.on_ball(t, h.c(0.0), near).unwrap_or(near)
    }

    fn upper(&self, h: &Helix, t: f64) -> f64 {
        if t <= h.span - self.lag {
            return TAU - self.theta_u;
        }
        if t >= h.span {
            return TAU - self.fold;
        }
        let near = self.fold + (self.theta_u - self.fold) * (h.span - t) / self.lag;
        TAU - h.on_ball(t, h.c(h.span), near).unwrap_or(near)
    }
}

/// The sweep of a ball along a single screw past its fold, or None when the
/// tool, the motion or the layout is not of that kind.
pub(super) fn ball_screwed_past_fold(screw: &Screw, origin: Vec3, axis: Vec3, meridian: &[OutlinePiece]) -> Result<Option<BrepSolid>, KernelRefusal> {
    let [OutlinePiece::Arc { centre, radius, start, end }, OutlinePiece::Line { from, to }] = meridian else { return Ok(None) };
    let r = *radius;
    if !(r > 0.0 && r.is_finite() && start.is_finite() && end.is_finite()) || centre[0].abs() > 1e-12 * r || ((end - start).abs() - PI).abs() > 1e-12 {
        return Ok(None);
    }
    // A ball: the half circle about a centre on the axis, closed along the
    // axis from its end back to its start.
    let at = |angle: f64| [centre[0] + r * angle.cos(), centre[1] + r * angle.sin()];
    let (a_end, a_start) = (at(*end), at(*start));
    let near = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).hypot(p[1] - q[1]) <= 1e-9 * r;
    if !(near(*from, a_end) && near(*to, a_start)) || a_end[0].abs() > 1e-9 * r || a_start[0].abs() > 1e-9 * r {
        return Ok(None);
    }
    if screw.frame.is_some() || screw.turns.is_some() || screw.c2 != 0.0 || screw.omega == 0.0 || screw.c == 0.0 || !(screw.t1 > screw.t0) {
        return Ok(None);
    }
    let Ok(w) = axis.normalized() else { return Ok(None) };
    let centre_tool = origin.add(w.scale(centre[1]));
    let c0 = screw.apply(screw.t0, centre_tool);
    let ez = screw.e.scale(screw.omega.signum());
    let foot = screw.a.add(ez.scale(c0.sub(screw.a).dot(ez)));
    let a = c0.sub(foot).length();
    let b = screw.c / screw.omega;
    // A right-handed helix about the turn's own sense only (a left hand one
    // would need the layout mirrored).
    if !(a > 1e-9 * r) || !(b > 1e-12 * r) {
        return Ok(None);
    }
    let Ok(ex) = c0.sub(foot).normalized() else { return Ok(None) };
    let h = Helix { o: foot, ex, ey: ez.cross(ex), ez, a, b, r, speed: a.hypot(b), span: screw.omega.abs() * (screw.t1 - screw.t0) };
    // The work grows with the turns (a tube piece and a strip of the layout
    // check each quarter turn): past 64 turns, left to the level engine.
    if !(h.span <= 128.0 * PI) {
        return Ok(None);
    }
    let rho = (a * a + b * b) / a;
    if !(rho < r * (1.0 - 1e-6)) {
        return Ok(None);
    }
    let fold = (rho / r).acos();
    let Some(layout) = layout(&h, fold) else { return Ok(None) };
    if !layout_holds(&h, &layout) {
        return Ok(None);
    }
    let body = match build(&h, &layout) {
        Ok(body) => body,
        Err(_) => return Ok(None),
    };
    if body.validate().iter().any(|issue| issue.severity == "error") {
        return Ok(None);
    }
    Ok(Some(body))
}

/// θu and the lag Δ (the tube at t meets the turn ahead at θu, its nearest
/// point there at t + Δ), and both end balls' curves; None where the turns do
/// not meet within the span or the ball curves do not run from the helix edge
/// to the fold.
fn layout(h: &Helix, fold: f64) -> Option<Layout> {
    // On the side of the turn ahead (θ from the fold toward B and round to π),
    // where the next turn (t from π to 3π on the endless helix) first holds
    // the circle at t = 0 no longer.
    let reach = |theta: f64| -> (f64, f64) {
        let (d, s) = h.nearest(h.p(0.0, theta), PI, 3.0 * PI);
        (d - h.r, s)
    };
    let n = 256;
    let mut bracket = None;
    for k in 0..n {
        let (x, y) = (fold + (PI - fold) * k as f64 / n as f64, fold + (PI - fold) * (k + 1) as f64 / n as f64);
        if reach(x).0 < 0.0 && reach(y).0 >= 0.0 {
            bracket = Some((x, y));
        }
    }
    let (mut lo, mut hi) = bracket?;
    for _ in 0..100 {
        let mid = 0.5 * (lo + hi);
        if reach(mid).0 < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let theta_u = 0.5 * (lo + hi);
    let lag = reach(theta_u).1;
    // The tube meets itself there: P(0, θu) = P(Δ, −θu) (the screw's symmetry).
    if h.p(0.0, theta_u).sub(h.p(lag, -theta_u)).length() > 1e-9 * h.r || !(lag < h.span) {
        return None;
    }
    // The start ball's curve from the helix edge (t = Δ, θ = −θu) back toward
    // the start, and the end ball's from t = span − Δ (θ = θu) on to the end.
    let track = |centre: Vec3, from: f64, to: f64, theta: f64| -> Option<Vec<(f64, f64)>> {
        let mut out = Vec::new();
        let mut near = theta;
        let m = 4 * STATIONS * 8;
        for k in 0..=m {
            let t = from + (to - from) * k as f64 / m as f64;
            if (t - 0.0).abs() < 1e-12 * h.span.max(1.0) || (t - h.span).abs() < 1e-12 * h.span.max(1.0) {
                continue;
            }
            let x = h.on_ball(t, centre, near)?;
            if (x - near).abs() > 0.2 {
                return None;
            }
            near = x;
            out.push((t, x));
        }
        Some(out)
    };
    let start_ball = track(h.c(0.0), lag, 0.0, -theta_u)?;
    let end_ball = track(h.c(h.span), h.span - lag, h.span, theta_u)?;
    // Both reach the fold line at the ends.
    let last_start = start_ball.last()?.1;
    let last_end = end_ball.last()?.1;
    if (last_start + fold).abs() > 0.05 || (last_end - fold).abs() > 0.05 {
        return None;
    }
    let layout = Layout { theta_u, lag, fold };
    // The tracked curves are the branches `lower` and `upper` read.
    for &(t, x) in &start_ball {
        if (-layout.lower(h, t) - x).abs() > 1e-9 {
            return None;
        }
    }
    for &(t, x) in &end_ball {
        if (TAU - layout.upper(h, t) - x).abs() > 1e-9 {
            return None;
        }
    }
    Some(layout)
}

/// The layout against the arc's own distance: on a grid of tube points (all
/// round each circle, the fold strip included) and of points on each end
/// ball's outer half, kept exactly where nothing of the arc is nearer than r,
/// away from the layout's own curves.
fn layout_holds(h: &Helix, layout: &Layout) -> bool {
    let (nt, npsi) = (((h.span / (PI / 16.0)).ceil() as usize).max(16), 96);
    let margin = 2e-3;
    for i in 0..=nt {
        let t = h.span * i as f64 / nt as f64;
        let (lo, hi) = (layout.lower(h, t), layout.upper(h, t));
        for j in 0..npsi {
            let psi = TAU * (j as f64 + 0.5) / npsi as f64;
            if (psi - lo).abs() < margin || (psi - hi).abs() < margin {
                continue;
            }
            let x = h.p(t, -psi);
            let kept = psi > lo && psi < hi;
            let (d, _) = h.nearest_about(x, t);
            let free = d >= h.r * (1.0 - 1e-9);
            // A kept point is free; a point left out is held by some part of
            // the arc (by more than the margin's worth, away from the trims).
            if kept != free && (d - h.r).abs() > 1e-6 * h.r {
                return false;
            }
        }
    }
    // The end balls: their outer halves, free exactly on the cap's side of
    // the ball curve (seen from the far pole, the cap is the region the end
    // circle's free arc and the curve bound).
    for at_end in [false, true] {
        let (t, sign) = if at_end { (h.span, 1.0) } else { (0.0, -1.0) };
        let (tangent, n, b) = h.frame(t);
        let centre = h.c(t);
        // The ball curve and the end circle's free arc, as a polygon in the
        // stereographic plane from the inner pole (c − sign·r·T).
        let plane = |x: Vec3| -> [f64; 2] {
            let d = x.sub(centre).scale(1.0 / h.r);
            let k = 1.0 / (1.0 + sign * d.dot(tangent)).max(1e-300);
            [d.dot(n) * k, d.dot(b) * k]
        };
        let mut polygon: Vec<[f64; 2]> = Vec::new();
        let m = 96;
        // The free arc of the end circle (ψ between the layout's bounds).
        let (lo, hi) = (layout.lower(h, t), layout.upper(h, t));
        for k in 0..=m {
            polygon.push(plane(h.p(t, -(lo + (hi - lo) * k as f64 / m as f64))));
        }
        // The ball curve back (each station on the tube at its own time).
        // (from the free arc's last point: the helix edge at the start, the
        // cusp at the end)
        for k in 1..m {
            let s = if at_end { h.span - layout.lag * k as f64 / m as f64 } else { layout.lag * (m - k) as f64 / m as f64 };
            let psi = if at_end { layout.upper(h, s) } else { layout.lower(h, s) };
            polygon.push(plane(h.p(s, -psi)));
        }
        let inside = |q: [f64; 2]| -> bool {
            let mut c = false;
            for k in 0..polygon.len() {
                let (p, q2) = (polygon[k], polygon[(k + 1) % polygon.len()]);
                if (p[1] > q[1]) != (q2[1] > q[1]) && q[0] < p[0] + (q[1] - p[1]) * (q2[0] - p[0]) / (q2[1] - p[1]) {
                    c = !c;
                }
            }
            c
        };
        let near_polygon = |q: [f64; 2]| -> bool {
            (0..polygon.len()).any(|k| {
                let (p, s) = (polygon[k], polygon[(k + 1) % polygon.len()]);
                let (dx, dy) = (s[0] - p[0], s[1] - p[1]);
                let l = dx * dx + dy * dy;
                let f = if l > 0.0 { (((q[0] - p[0]) * dx + (q[1] - p[1]) * dy) / l).clamp(0.0, 1.0) } else { 0.0 };
                (q[0] - p[0] - f * dx).hypot(q[1] - p[1] - f * dy) < 1e-2
            })
        };
        for i in 1..24 {
            let lat = 0.5 * PI * i as f64 / 24.0;
            for j in 0..48 {
                let lon = TAU * j as f64 / 48.0;
                let dir = n.scale(lat.cos() * lon.cos()).add(b.scale(lat.cos() * lon.sin())).add(tangent.scale(sign * lat.sin()));
                let x = centre.add(dir.scale(h.r));
                let q = plane(x);
                if near_polygon(q) {
                    continue;
                }
                let reach = 2.0 * h.r / h.b + 0.1;
                let (lo_s, hi_s) = if at_end { ((h.span - reach).max(0.0), h.span - 1e-9) } else { (1e-9, reach.min(h.span)) };
                let (d, _) = h.nearest(x, lo_s, hi_s);
                let free = d >= h.r * (1.0 - 1e-9);
                if inside(q) != free && (d - h.r).abs() > 1e-6 * h.r {
                    return false;
                }
            }
        }
    }
    true
}

/// The ball's carrier at an end: poles off the cap, on the line through the
/// held part of the end circle (just off it toward the tube), its seam on the
/// side of the tube.
fn end_ball(h: &Helix, at_end: bool, layout: &Layout) -> Result<NurbsSurface, KernelRefusal> {
    let (t, ahead) = if at_end { (h.span, 1.0) } else { (0.0, -1.0) };
    let (tangent, n, b) = h.frame(t);
    let centre = h.c(t);
    // The middle of the end circle's held arc.
    let mid = if at_end { -0.5 * (layout.theta_u - layout.fold) } else { 0.5 * (layout.theta_u - layout.fold) };
    let u1 = n.scale(mid.cos()).add(b.scale(mid.sin()));
    let tilt: f64 = 0.25;
    let w = u1.scale(-tilt.cos()).sub(tangent.scale(ahead * tilt.sin()));
    let toward = tangent.scale(-ahead);
    let m = frag(toward.sub(w.scale(toward.dot(w))).normalized(), "ball_seam")?;
    let generatrix = frag(crate::make_arc(centre, w, m, h.r, 0.0, PI), "ball_meridian")?;
    frag(make_revolution(centre, w, &generatrix, TAU), "ball_carrier")
}

/// The distinct knots strictly inside a knot vector's domain.
fn interior(knots: &[f64]) -> Vec<f64> {
    let (lo, hi) = (knots[0], knots[knots.len() - 1]);
    let mut out: Vec<f64> = knots.iter().copied().filter(|&k| k > lo + 1e-12 && k < hi - 1e-12).collect();
    out.dedup_by(|a, b| (*a - *b).abs() < 1e-12);
    out
}

/// One edge piece: its 3D curve through `points` and the pcurve on each face
/// that uses it, all at the same parameters.
struct Piece {
    id: u64,
    curve: NurbsCurve,
}

fn build(h: &Helix, layout: &Layout) -> Result<BrepSolid, KernelRefusal> {
    let (fold, theta_u, lag, span, r) = (layout.fold, layout.theta_u, layout.lag, h.span, h.r);
    let mut next = 0u64;
    let mut id = || {
        next += 1;
        next
    };
    // The tube's carrier rows: arcs in ψ from the fold line to the other, the
    // same parameter for every row; v of an angle by bisection on one of them.
    let reference = frag(crate::make_arc(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 1.0, fold, TAU - fold), "tube_row")?;
    let angle_of = |v: f64| -> Result<f64, KernelRefusal> {
        let p = frag(reference.evaluate(v.clamp(0.0, 1.0)), "tube_row")?;
        let a = p.y.atan2(p.x);
        Ok(if a < 0.5 * fold { a + TAU } else { a })
    };
    let v_of = |psi: f64| -> Result<f64, KernelRefusal> {
        let (mut lo, mut hi) = (0.0, 1.0);
        for _ in 0..64 {
            let mid = 0.5 * (lo + hi);
            if angle_of(mid)? < psi {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Ok(0.5 * (lo + hi))
    };
    let pieces_n = ((span / (0.5 * PI)).ceil() as usize).max(1);
    let cuts: Vec<f64> = (0..=pieces_n).map(|k| span * k as f64 / pieces_n as f64).collect();
    // The second half's pieces run both parameters backwards (t down, ψ
    // down: the same normal): the helix's half-turn symmetry then carries
    // each onto its mirror in the first half, parameters and all, so the end
    // ball's cusp is the corner (0, 0) of its piece as the start ball's is.
    let backward = |k: usize| 2 * k >= pieces_n;
    let row_arc = |t: f64, back: bool| -> Result<NurbsCurve, KernelRefusal> {
        let (_, n, b) = h.frame(t);
        if back {
            frag(crate::make_arc(h.c(t), n, b, r, fold, TAU - fold), "tube_row")
        } else {
            frag(crate::make_arc(h.c(t), n, b.scale(-1.0), r, fold, TAU - fold), "tube_row")
        }
    };
    let mut carriers = Vec::new();
    for k in 0..pieces_n {
        let (t0, t1) = (cuts[k], cuts[k + 1]);
        let (t0, t1) = if backward(k) { (t1, t0) } else { (t0, t1) };
        let arcs: Vec<NurbsCurve> = (0..ROWS).map(|i| row_arc(t0 + (t1 - t0) * i as f64 / (ROWS - 1) as f64, backward(k))).collect::<Result<_, _>>()?;
        let params: Vec<f64> = (0..ROWS).map(|i| i as f64 / (ROWS - 1) as f64).collect();
        let count = arcs[0].control_points.len();
        let mut columns = Vec::with_capacity(count);
        for j in 0..count {
            let points: Vec<Vec3> = arcs.iter().map(|arc| frag(arc.control_points[j].point(), "tube_row")).collect::<Result<_, _>>()?;
            columns.push(frag(crate::interpolate_curve(&points, 5, &params), "tube_column")?);
        }
        let rows_u = columns[0].control_points.len();
        let mut net = Vec::with_capacity(rows_u);
        for i in 0..rows_u {
            let mut row = Vec::with_capacity(count);
            for j in 0..count {
                row.push(Vec4::from_point(frag(columns[j].control_points[i].point(), "tube_column")?, arcs[0].control_points[j].w));
            }
            net.push(row);
        }
        carriers.push(frag(NurbsSurface::new(columns[0].degree, arcs[0].degree, columns[0].knots.clone(), arcs[0].knots.clone(), net), "tube_carrier")?);
    }
    // A tube point's (u, v) on piece k.
    let uv_on = |k: usize, t: f64, psi: f64| -> Result<[f64; 2], KernelRefusal> {
        let u = (t - cuts[k]) / (cuts[k + 1] - cuts[k]);
        Ok(if backward(k) { [1.0 - u, v_of(TAU - psi)?] } else { [u, v_of(psi)?] })
    };
    let point_at = |t: f64, psi: f64| h.p(t, -psi);

    // Vertices, keyed by where they lie.
    let mut vertices: Vec<VertexRecord> = Vec::new();
    let mut vertex_at: Vec<(Vec3, u64)> = Vec::new();
    let mut vertex = |x: Vec3, vertices: &mut Vec<VertexRecord>, id: &mut dyn FnMut() -> u64| -> u64 {
        if let Some(&(_, v)) = vertex_at.iter().find(|(p, _)| p.sub(x).length() <= 1e-9 * r) {
            return v;
        }
        let v = id();
        vertices.push(VertexRecord { id: v, point: x });
        vertex_at.push((x, v));
        v
    };
    let mut edges: Vec<EdgeRecord> = Vec::new();
    let fit3 = |points: &[Vec3], params: &[f64]| frag(crate::interpolate_curve(points, 5, params), "edge_curve");
    let line2 = |p: [f64; 2], q: [f64; 2]| frag(make_line(Vec3::new(p[0], p[1], 0.0), Vec3::new(q[0], q[1], 0.0)), "edge_pcurve");
    let fit2 = |uv: &[[f64; 2]], params: &[f64]| {
        let points: Vec<Vec3> = uv.iter().map(|p| Vec3::new(p[0], p[1], 0.0)).collect();
        frag(crate::interpolate_curve(&points, 5, params), "edge_pcurve")
    };
    let stations: Vec<f64> = (0..=STATIONS).map(|i| i as f64 / STATIONS as f64).collect();

    // Edges along the tube in t: (kind, t0, t1) -> piece. Kind 0: the helix
    // edge (keyed by its own time, the upper use's), 1: the start ball's
    // curve, 2: the end ball's.
    let mut along: Vec<((u8, f64, f64), u64)> = Vec::new();
    let breaks = |lo: f64, hi: f64, extra: &[f64]| -> Vec<f64> {
        let mut out: Vec<f64> = cuts.iter().copied().chain(extra.iter().copied()).filter(|&t| t > lo + 1e-12 * span && t < hi - 1e-12 * span).collect();
        out.push(lo);
        out.push(hi);
        out.sort_by(f64::total_cmp);
        out.dedup_by(|a, b| (*a - *b).abs() <= 1e-12 * span);
        out
    };
    // The helix edge's own breaks: where the upper use crosses a cut, and
    // where the lower use (Δ later) does.
    let helix_breaks = breaks(0.0, span - lag, &cuts.iter().map(|&t| t - lag).collect::<Vec<_>>());
    // The rows' v runs smoothly with ψ only within each of their knot spans:
    // a ball curve's pieces break where it crosses a span's end, so that each
    // piece's (u, v) is smooth (as its 3D curve is).
    let mut knot_angles: Vec<f64> = Vec::new();
    for &k in &reference.knots {
        if k > 1e-12 && k < 1.0 - 1e-12 && !knot_angles.iter().any(|&a| (angle_of(k).unwrap_or(a) - a).abs() < 1e-12) {
            knot_angles.push(angle_of(k)?);
        }
    }
    let crossing = |f: &dyn Fn(f64) -> f64, lo: f64, hi: f64, target: f64| -> Option<f64> {
        let (flo, fhi) = (f(lo) - target, f(hi) - target);
        if !(flo * fhi < 0.0) {
            return None;
        }
        let (mut a, mut b) = (lo, hi);
        for _ in 0..80 {
            let m = 0.5 * (a + b);
            if (f(m) - target) * flo > 0.0 {
                a = m;
            } else {
                b = m;
            }
        }
        Some(0.5 * (a + b))
    };
    let lower_of = |t: f64| layout.lower(h, t);
    let upper_of = |t: f64| layout.upper(h, t);
    let start_breaks: Vec<f64> = knot_angles.iter().filter_map(|&a| crossing(&lower_of, 0.0, lag, a)).collect();
    let end_breaks: Vec<f64> = knot_angles.iter().filter_map(|&a| crossing(&upper_of, span - lag, span, a)).collect();
    // The points of a ball curve at time t (kind 1 start, 2 end).
    let ball_point = |kind: u8, t: f64| -> f64 { if kind == 1 { layout.lower(h, t) } else { layout.upper(h, t) } };
    // Build (or find) the piece of `kind` over [t0, t1] (the helix in its own
    // time), with its 3D curve in increasing t.
    let mut piece_ids: Vec<((u8, f64, f64), Piece)> = Vec::new();
    for &(kind, lo, hi) in &[(0u8, 0.0, span - lag), (1u8, 0.0, lag), (2u8, span - lag, span)] {
        let marks = match kind {
            0 => helix_breaks.clone(),
            1 => breaks(lo, hi, &start_breaks),
            _ => breaks(lo, hi, &end_breaks),
        };
        for pair in marks.windows(2) {
            let (t0, t1) = (pair[0], pair[1]);
            // The end ball's curve runs from its cusp back (as the start
            // ball's runs from its own): mirror images, parameters and all.
            let down = kind == 2;
            let points: Vec<Vec3> = stations
                .iter()
                .map(|&s| {
                    let t = if down { t1 - (t1 - t0) * s } else { t0 + (t1 - t0) * s };
                    match kind {
                        0 => point_at(t, TAU - theta_u),
                        _ => point_at(t, ball_point(kind, t)),
                    }
                })
                .collect();
            let curve = fit3(&points, &stations)?;
            let (vs, ve) = (vertex(points[0], &mut vertices, &mut id), vertex(points[STATIONS], &mut vertices, &mut id));
            let e = id();
            edges.push(EdgeRecord { id: e, curve: curve.clone(), t0: 0.0, t1: 1.0, start_vertex_id: vs, end_vertex_id: ve, degenerate: false, name: None });
            along.push(((kind, t0, t1), e));
            piece_ids.push(((kind, t0, t1), Piece { id: e, curve }));
        }
    }
    // Circle edges at each cut, from the lower bound to the upper, with ψ at
    // even steps of the carrier's v.
    let mut circles: Vec<(f64, u64, f64, f64)> = Vec::new(); // (t, edge, v_lo, v_hi)
    for &t in &cuts {
        let (vl, vh) = (v_of(layout.lower(h, t))?, v_of(layout.upper(h, t))?);
        let (vl, vh) = if t > 0.5 * span { (vh, vl) } else { (vl, vh) };
        // The row arc at t from v = vl to vh, its knots taken onto [0, 1]
        // (so its parameter is the pcurves' own).
        let whole = row_arc(t, false)?;
        let (lo_v, hi_v) = (vl.min(vh), vl.max(vh));
        let mut piece = whole;
        if hi_v < 1.0 - 1e-12 {
            piece = frag(piece.split(hi_v), "circle_edge")?.0;
        }
        if lo_v > 1e-12 {
            piece = frag(piece.split(lo_v), "circle_edge")?.1;
        }
        let [d0, d1] = frag(piece.domain(), "circle_edge")?;
        let knots: Vec<f64> = piece.knots.iter().map(|k| ((k - d0) / (d1 - d0)).clamp(0.0, 1.0)).collect();
        let piece = frag(NurbsCurve::new(piece.degree, knots, piece.control_points.clone()), "circle_edge")?;
        let curve = if vl > vh { frag(piece.reversed(), "circle_edge")? } else { piece };
        let (p0, p1) = (frag(curve.evaluate(0.0), "circle_edge")?, frag(curve.evaluate(1.0), "circle_edge")?);
        let (vs, ve) = (vertex(p0, &mut vertices, &mut id), vertex(p1, &mut vertices, &mut id));
        let e = id();
        edges.push(EdgeRecord { id: e, curve, t0: 0.0, t1: 1.0, start_vertex_id: vs, end_vertex_id: ve, degenerate: false, name: None });
        circles.push((t, e, vl, vh));
    }

    // The tube's faces.
    let mut faces = Vec::new();
    for k in 0..pieces_n {
        let (lo_t, hi_t) = (cuts[k], cuts[k + 1]);
        let within = |t0: f64, t1: f64| t0 >= lo_t - 1e-12 * span && t1 <= hi_t + 1e-12 * span;
        let mut coedges = Vec::new();
        // Bottom, left to right: the start ball's curve, then the helix's
        // lower use (its own time Δ earlier).
        let mut bottom: Vec<(f64, f64, u64, u8)> = Vec::new();
        for ((kind, t0, t1), e) in &along {
            match kind {
                1 if within(*t0, *t1) => bottom.push((*t0, *t1, *e, 1)),
                0 if within(t0 + lag, t1 + lag) => bottom.push((t0 + lag, t1 + lag, *e, 0)),
                _ => {}
            }
        }
        bottom.sort_by(|a, b| a.0.total_cmp(&b.0));
        for &(t0, t1, e, kind) in &bottom {
            let uv: Vec<[f64; 2]> = stations
                .iter()
                .map(|&s| {
                    let t = t0 + (t1 - t0) * s;
                    let psi = if kind == 0 { theta_u } else { layout.lower(h, t) };
                    uv_on(k, t, psi)
                })
                .collect::<Result<_, _>>()?;
            let pcurve = if kind == 0 { line2(uv[0], uv[STATIONS])? } else { fit2(&uv, &stations)? };
            coedges.push(CoedgeRecord { id: id(), edge_id: e, forward: kind != 2, pcurve });
        }
        // Right side, up.
        let &(_, e, _, _) = circles.iter().find(|c| (c.0 - hi_t).abs() <= 1e-12 * span).expect("cut");
        let (p, q) = (uv_on(k, hi_t, layout.lower(h, hi_t))?, uv_on(k, hi_t, layout.upper(h, hi_t))?);
        coedges.push(CoedgeRecord { id: id(), edge_id: e, forward: !(hi_t > 0.5 * span), pcurve: line2(p, q)? });
        // Top, right to left: the helix's upper use, then the end ball's curve.
        let mut top: Vec<(f64, f64, u64, u8)> = Vec::new();
        for ((kind, t0, t1), e) in &along {
            match kind {
                2 if within(*t0, *t1) => top.push((*t0, *t1, *e, 2)),
                0 if within(*t0, *t1) => top.push((*t0, *t1, *e, 0)),
                _ => {}
            }
        }
        top.sort_by(|a, b| b.0.total_cmp(&a.0));
        for &(t0, t1, e, kind) in &top {
            let uv: Vec<[f64; 2]> = stations
                .iter()
                .map(|&s| {
                    let t = t1 - (t1 - t0) * s;
                    let psi = if kind == 0 { TAU - theta_u } else { layout.upper(h, t) };
                    uv_on(k, t, psi)
                })
                .collect::<Result<_, _>>()?;
            let pcurve = if kind == 0 { line2(uv[0], uv[STATIONS])? } else { fit2(&uv, &stations)? };
            coedges.push(CoedgeRecord { id: id(), edge_id: e, forward: kind == 2, pcurve });
        }
        // Left side, down.
        let &(_, e, _, _) = circles.iter().find(|c| (c.0 - lo_t).abs() <= 1e-12 * span).expect("cut");
        let (p, q) = (uv_on(k, lo_t, layout.upper(h, lo_t))?, uv_on(k, lo_t, layout.lower(h, lo_t))?);
        coedges.push(CoedgeRecord { id: id(), edge_id: e, forward: lo_t > 0.5 * span, pcurve: line2(p, q)? });
        // Each loop starts where its mirror does: on a backward piece at its
        // top (the end ball's cusp on the last), as the first starts at the
        // start ball's cusp. A reader that follows the loop from its first
        // point (the bench's measurer does) then reads both alike.
        if backward(k) {
            if let Some(at) = coedges.iter().position(|c| top.iter().any(|t| t.2 == c.edge_id)) {
                coedges.rotate_left(at);
            }
        }
        faces.push(FaceRecord { id: id(), surface: carriers[k].clone(), same_sense: true, loops: vec![LoopRecord { id: id(), coedges }], name: None });
    }

    // The end balls: the end circle and the ball curve's pieces.
    for at_end in [false, true] {
        let carrier = end_ball(h, at_end, layout)?;
        // The pcurve of an edge on the ball, run along the coedge (`forward`
        // with the edge or against it): fitted piece by piece between the
        // places where it crosses a knot line of the ball's carrier (where its
        // (u, v) is only once differentiable), the pieces joined into one
        // curve at those places with full multiplicity, at the edge's own
        // parameter.
        let (knots_u, knots_v) = (interior(&carrier.knots_u), interior(&carrier.knots_v));
        let on_ball = |curve: &NurbsCurve, forward: bool| -> Result<NurbsCurve, KernelRefusal> {
            let uv_at = |s: f64| -> Result<[f64; 2], KernelRefusal> {
                let x = frag(curve.evaluate(if forward { s } else { 1.0 - s }), "edge_curve")?;
                let q = frag(crate::project_point_to_surface(&carrier, x), "ball_pcurve")?;
                Ok([q.u, q.v])
            };
            // Where the pcurve crosses a knot line: sign changes on a fine
            // sampling, bisected.
            let fine = 512;
            let samples: Vec<[f64; 2]> = (0..=fine).map(|i| uv_at(i as f64 / fine as f64)).collect::<Result<_, _>>()?;
            // (and every sixteenth of the edge: a long edge passing near the
            // carrier's pole bends sharply in (u, v))
            let mut cuts: Vec<f64> = (0..16).map(|i| i as f64 / 16.0).collect();
            for i in 0..fine {
                for (axis, knots) in [(0usize, &knots_u), (1, &knots_v)] {
                    for &k in knots.iter() {
                        let (a, b) = (samples[i][axis] - k, samples[i + 1][axis] - k);
                        if a * b < 0.0 {
                            let (mut lo, mut hi) = (i as f64 / fine as f64, (i + 1) as f64 / fine as f64);
                            for _ in 0..60 {
                                let m = 0.5 * (lo + hi);
                                if (uv_at(m)?[axis] - k) * a > 0.0 {
                                    lo = m;
                                } else {
                                    hi = m;
                                }
                            }
                            cuts.push(0.5 * (lo + hi));
                        }
                    }
                }
            }
            cuts.push(1.0);
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
            let mut joined: Option<NurbsCurve> = None;
            for pair in cuts.windows(2) {
                let params: Vec<f64> = stations.iter().map(|&s| pair[0] + (pair[1] - pair[0]) * s).collect();
                let uv: Vec<[f64; 2]> = params.iter().map(|&s| uv_at(s)).collect::<Result<_, _>>()?;
                let piece = fit2(&uv, &params)?;
                joined = Some(match joined {
                    None => piece,
                    Some(left) => {
                        let degree = left.degree;
                        let mut knots = left.knots[..left.knots.len() - 1].to_vec();
                        knots.extend_from_slice(&piece.knots[degree + 1..]);
                        let mut points = left.control_points.clone();
                        points.extend_from_slice(&piece.control_points[1..]);
                        frag(NurbsCurve::new(degree, knots, points), "ball_pcurve")?
                    }
                });
            }
            joined.ok_or_else(|| refuse("ball_pcurve", "an edge on an end ball has no pcurve".into()))
        };
        let (t, kind) = if at_end { (span, 2u8) } else { (0.0, 1u8) };
        let &(_, circle, _, _) = circles.iter().find(|c| (c.0 - t).abs() <= 1e-12 * span).expect("end");
        let circle_curve = edges.iter().find(|e| e.id == circle).expect("edge").curve.clone();
        let mut ball_pieces: Vec<&((u8, f64, f64), Piece)> = piece_ids.iter().filter(|p| p.0 .0 == kind).collect();
        let mut coedges = Vec::new();
        if at_end {
            // The circle down from the cusp to the helix edge, then the curve
            // on from there to the cusp.
            coedges.push(CoedgeRecord { id: id(), edge_id: circle, forward: true, pcurve: on_ball(&circle_curve, true)? });
            ball_pieces.sort_by(|a, b| a.0 .1.total_cmp(&b.0 .1));
            for p in ball_pieces {
                coedges.push(CoedgeRecord { id: id(), edge_id: p.1.id, forward: false, pcurve: on_ball(&p.1.curve, false)? });
            }
        } else {
            // The circle up from the cusp to the helix edge, then the curve
            // back to the cusp.
            coedges.push(CoedgeRecord { id: id(), edge_id: circle, forward: true, pcurve: on_ball(&circle_curve, true)? });
            ball_pieces.sort_by(|a, b| b.0 .1.total_cmp(&a.0 .1));
            for p in ball_pieces {
                coedges.push(CoedgeRecord { id: id(), edge_id: p.1.id, forward: false, pcurve: on_ball(&p.1.curve, false)? });
            }
        }
        // The loop's sense is the tube's (each edge run the other way): the
        // face keeps the carrier's normal where that points out, and its loop
        // then runs anticlockwise in (u, v); a loop the other way is a layout
        // this does not build.
        let mut area = 0.0;
        for c in &coedges {
            let [d0, d1] = frag(c.pcurve.domain(), "edge_pcurve")?;
            for i in 0..32 {
                let (p, q) = (frag(c.pcurve.evaluate(d0 + (d1 - d0) * i as f64 / 32.0), "edge_pcurve")?, frag(c.pcurve.evaluate(d0 + (d1 - d0) * (i + 1) as f64 / 32.0), "edge_pcurve")?);
                area += p.x * q.y - q.x * p.y;
            }
        }
        let first = frag(coedges[0].pcurve.evaluate(0.5), "edge_pcurve")?;
        let d = frag(carrier.derivatives(first.x, first.y, 1), "ball_carrier")?;
        let outward = d[1][0].cross(d[0][1]).dot(d[0][0].sub(h.c(t))) > 0.0;
        if (area > 0.0) != outward {
            return Err(refuse("ball_screw_cap", "an end ball's loop runs against its carrier".into()));
        }
        faces.push(FaceRecord { id: id(), surface: carrier, same_sense: outward, loops: vec![LoopRecord { id: id(), coedges }], name: None });
    }
    let shell = id();
    Ok(BrepSolid {
        mass_properties_cache: Default::default(),
        id: id(),
        vertices,
        edges,
        shells: vec![ShellRecord { id: shell, faces }],
        genus: 0,
        kind: Default::default(),
        cells: Default::default(),
    })
}

