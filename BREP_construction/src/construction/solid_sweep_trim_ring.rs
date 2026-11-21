//! The trim engine's sheets for a ring (a torus) under a two-turn motion.
//!
//! At time t the ring's points on the tube circle at angle ψ about its axis,
//! x(ψ, v) = c + (R + r cos v) e(ψ) + r sin v w, with normal n(ψ, v) =
//! cos v e(ψ) + sin v w, move along the motion's velocity field, which is
//! affine in x; so g(v) = n · velocity is a first harmonic a cos v + b sin v +
//! c₀ exactly, and where |c₀| < √(a² + b²) it vanishes at two angles, the
//! grazing branches: v = atan2(b, a) ∓ arccos(−c₀ / √(a² + b²)). The leading
//! band (g > 0) lies between them; the trailing band is the rest.
//!
//! Sheets: the envelope of each branch over each half of ψ, and each end
//! pose's band (the trailing band at the start, the leading band at the end)
//! over each half of ψ, charted (ψ fraction, fraction across the band from
//! one branch to the other), so the branches are the band's sides.

use super::*;
use std::f64::consts::{PI, TAU};
use crate::{make_line, make_revolution};

impl<'a> ConvexSweep<'a> {
    /// The ring's axis frame: two unit vectors square to its axis and to
    /// each other (e(ψ) = cos ψ e1 + sin ψ e2).
    fn ring_frame(&self) -> (Vec3, Vec3) {
        let w = self.ring().axis;
        let seed = if w.x.abs() < 0.9 { Vec3::new(1.0, 0.0, 0.0) } else { Vec3::new(0.0, 1.0, 0.0) };
        let e1 = seed.sub(w.scale(seed.dot(w))).normalized().unwrap_or(seed);
        (e1, w.cross(e1))
    }

    /// The ring's point at (ψ, v), in its own frame.
    #[doc(hidden)]
    pub fn ring_at(&self, psi: f64, v: f64) -> Vec3 {
        let torus = self.ring();
        let (e1, e2) = self.ring_frame();
        let radial = e1.scale(psi.cos()).add(e2.scale(psi.sin()));
        torus.centre.add(radial.scale(torus.major + torus.minor * v.cos())).add(torus.axis.scale(torus.minor * v.sin()))
    }

    /// The ring's outward normal at (ψ, v), in its own frame.
    #[doc(hidden)]
    pub fn ring_normal(&self, psi: f64, v: f64) -> Vec3 {
        let (e1, e2) = self.ring_frame();
        let radial = e1.scale(psi.cos()).add(e2.scale(psi.sin()));
        radial.scale(v.cos()).add(self.ring().axis.scale(v.sin()))
    }

    /// (ψ, v) of a tool-frame point (ψ in [0, 2π), v in (−π, π]).
    #[doc(hidden)]
    pub fn ring_angles(&self, x: Vec3) -> (f64, f64) {
        let torus = self.ring();
        let (e1, e2) = self.ring_frame();
        let d = x.sub(torus.centre);
        let psi = d.dot(e2).atan2(d.dot(e1)).rem_euclid(TAU);
        let radial = e1.scale(psi.cos()).add(e2.scale(psi.sin()));
        let q = d.sub(radial.scale(torus.major));
        (psi, q.dot(torus.axis).atan2(q.dot(radial)))
    }

    /// The grazing branches at time t on the tube circle at ψ: (lower, upper),
    /// the leading band between them; None where the circle grazes nowhere or
    /// all round.
    #[doc(hidden)]
    pub fn ring_branches(&self, t: f64, psi: f64) -> Option<(f64, f64)> {
        let (a, b, c) = self.ring_harmonic(t, psi);
        let r = a.hypot(b);
        if !(c.abs() < r) {
            return None;
        }
        let (base, spread) = (b.atan2(a), (-c / r).acos());
        Some((base - spread, base + spread))
    }

        /// The grazing value round the tube circle at ψ at time t, a cos v +
    /// b sin v + c: (a, b, c).
    #[doc(hidden)]
    pub fn ring_harmonic(&self, t: f64, psi: f64) -> (f64, f64, f64) {
        let pose = self.motion.pose(t);
        let g = |v: f64| pose.rotate(self.ring_normal(psi, v)).dot(self.motion.velocity(t, pose.apply(self.ring_at(psi, v))));
        let (g0, g1, g2, g3) = (g(0.0), g(0.5 * PI), g(PI), g(1.5 * PI));
        (0.5 * (g0 - g2), 0.5 * (g1 - g3), 0.25 * (g0 + g1 + g2 + g3))
    }

    /// ψ over a half, from the sweep's seam: half 0 from the seam through π,
    /// half 1 the rest.
    fn half_psi(&self, half: u8, u: f64) -> f64 {
        self.seam + PI * (f64::from(half) + u)
    }

    /// A ring sheet's point (None where its branches are not defined).
    pub(super) fn ring_point(&self, sheet: SheetId, p: [f64; 2]) -> Option<SheetPoint> {
        match sheet {
            SheetId::Envelope(branch, half) => {
                let t = self.t_of(p[1]);
                let psi = self.half_psi(half, p[0]);
                let (lower, upper) = self.ring_branches(t, psi)?;
                let x = self.ring_at(psi, if branch == 0 { lower } else { upper });
                let pose = self.motion.pose(t);
                let y = pose.apply(x);
                // The depth at times beside t (the point is on the ring at
                // t, grazing): its second difference says whether the
                // neighbouring poses hold it (a fold); per radian of the
                // fastest turn, as for a polyhedron's strips.
                let rate = self.motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs())).max(1e-300);
                let h = 1e-4 / rate;
                let depth = |s: f64| self.depth(self.motion.pose(s).invert(y));
                let local = -(depth(t + h) + depth(t - h)) / 1e-8;
                Some(SheetPoint { world: y, tool: x, t, own: self.own_index(t), local })
            }
            SheetId::Band(end, half) => {
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                let (t, pose) = (self.grid.times[own], self.grid.poses[own]);
                let psi = self.half_psi(half, p[0]);
                let (lower, upper) = self.ring_branches(t, psi)?;
                // The end's leading band from the lower branch to the upper;
                // the start's trailing band from the upper round to the lower.
                let v = if end == 1 { lower + p[1] * (upper - lower) } else { upper + p[1] * (lower + TAU - upper) };
                let x = self.ring_at(psi, v);
                let y = pose.apply(x);
                let velocity = self.motion.velocity(t, y);
                let side = if end == 0 { 1.0 } else { -1.0 };
                let local = -side * velocity.dot(pose.rotate(self.ring_normal(psi, v))) / velocity.length().max(1e-300);
                Some(SheetPoint { world: y, tool: x, t, own, local })
            }
            _ => None,
        }
    }

    /// A band's local sign with no domain test (zero on its branches).
    pub(super) fn ring_local_raw(&self, sheet: SheetId, p: [f64; 2]) -> Option<f64> {
        match sheet {
            SheetId::Band(..) => self.ring_point(sheet, p).map(|q| q.local),
            _ => None,
        }
    }

    /// The branch along a band's side w = 0 (`far` false) or w = 1.
    fn band_branch(end: u8, far: bool) -> u8 {
        // The end's leading band runs lower (0) to upper (1); the start's
        // trailing band upper to lower.
        match (end, far) {
            (1, false) | (0, true) => 0,
            _ => 1,
        }
    }

    /// Who borders a ring sheet along a side of its domain (side: 0 first
    /// row, 1 last row, 2 first column, 3 last column; an envelope's rows are
    /// its times, a band's rows its ψ).
    pub(super) fn ring_side_border(&self, sheet: SheetId, side: usize) -> Border {
        match sheet {
            SheetId::Envelope(branch, half) => match side {
                0 => Border::Sheet(SheetId::Band(0, half)),
                1 => Border::Sheet(SheetId::Band(1, half)),
                _ => Border::Sheet(SheetId::Envelope(branch, 1 - half)),
            },
            SheetId::Band(end, half) => match side {
                0 | 1 => Border::Sheet(SheetId::Band(end, 1 - half)),
                2 => Border::Sheet(SheetId::Envelope(Self::band_branch(end, false), half)),
                _ => Border::Sheet(SheetId::Envelope(Self::band_branch(end, true), half)),
            },
            _ => Border::Unknown,
        }
    }

    /// The partner across a ring sheet's contour point (as `partner`).
    pub(super) fn ring_partner(&self, sheet: SheetId, point: &SheetPoint, active: Active) -> Border {
        match active {
            Active::Fold => {
                let n = self.grid.times.len();
                let (t0, t1) = (self.grid.times[0], self.grid.times[n - 1]);
                let step = (t1 - t0) / (n - 1) as f64;
                let depth_at = |s: f64| self.depth(self.motion.pose(s).invert(point.world));
                let g = 0.5 * (5f64.sqrt() - 1.0);
                let mut best: Option<(f64, f64)> = None;
                for sign in [1.0, -1.0] {
                    let (mut a, mut b) = (point.t + sign * 1e-9 * step, point.t + sign * 2.0 * step);
                    if sign < 0.0 {
                        std::mem::swap(&mut a, &mut b);
                    }
                    let (a0, b0) = (a.max(t0), b.min(t1));
                    if b0 <= a0 {
                        continue;
                    }
                    let (mut a, mut b) = (a0, b0);
                    for _ in 0..80 {
                        let (c, d) = (b - g * (b - a), a + g * (b - a));
                        if depth_at(c) > depth_at(d) {
                            b = d;
                        } else {
                            a = c;
                        }
                    }
                    let s = 0.5 * (a + b);
                    let depth = depth_at(s);
                    if best.map_or(true, |x| depth > x.1) {
                        best = Some((s, depth));
                    }
                }
                match best {
                    Some((s, _)) => {
                        // Held deepest at the motion's end: by that end's band
                        // (a neighbour beyond the end does not exist).
                        let tiny = 1e-9 * (t1 - t0);
                        let end = if s >= t1 - tiny { Some(1u8) } else if s <= t0 + tiny { Some(0u8) } else { None };
                        match self.ring_feature_at(point.world, s, end) {
                            Border::Unknown => Border::Sheet(sheet),
                            other => other,
                        }
                    }
                    None => Border::Sheet(sheet),
                }
            }
            // An envelope point whose own fold sign turns: it meets itself
            // along the fold; a band point on a branch: that branch's envelope.
            Active::Local => match sheet {
                SheetId::Band(end, half) => {
                    let (psi, v) = self.ring_angles(point.tool);
                    match self.ring_branches(point.t, psi) {
                        Some((lower, upper)) => {
                            let near = |a: f64| ((v - a + PI).rem_euclid(TAU) - PI).abs();
                            let _ = end;
                            Border::Sheet(SheetId::Envelope(if near(lower) <= near(upper) { 0 } else { 1 }, half))
                        }
                        None => Border::Unknown,
                    }
                }
                _ => Border::Sheet(sheet),
            },
            Active::Competitor(at) => {
                let n = self.grid.times.len();
                let depth_at = |s: f64| self.depth(self.motion.pose(s).invert(point.world));
                let s_star = if at == 0 || at == n - 1 {
                    self.grid.times[at]
                } else {
                    let (mut a, mut b) = (self.grid.times[at - 1], self.grid.times[at + 1]);
                    let g = 0.5 * (5f64.sqrt() - 1.0);
                    for _ in 0..80 {
                        let (c, d) = (b - g * (b - a), a + g * (b - a));
                        if depth_at(c) > depth_at(d) {
                            b = d;
                        } else {
                            a = c;
                        }
                    }
                    0.5 * (a + b)
                };
                let end = if at == 0 { Some(0u8) } else if at == n - 1 { Some(1u8) } else { None };
                self.ring_feature_at(point.world, s_star, end)
            }
        }
    }

    /// The ring sheet of the pose at time `s` nearest the world point `y`:
    /// the end pose's band, or the envelope of the branch nearest it.
    pub(super) fn ring_feature_at(&self, y: Vec3, s: f64, end: Option<u8>) -> Border {
        let x = self.motion.pose(s).invert(y);
        let (psi, v) = self.ring_angles(x);
        let half = if (psi - self.seam).rem_euclid(TAU) < PI { 0 } else { 1 };
        if let Some(e) = end {
            return Border::Sheet(SheetId::Band(e, half));
        }
        match self.ring_branches(s, psi) {
            Some((lower, upper)) => {
                let near = |a: f64| ((v - a + PI).rem_euclid(TAU) - PI).abs();
                Border::Sheet(SheetId::Envelope(if near(lower) <= near(upper) { 0 } else { 1 }, half))
            }
            None => Border::Unknown,
        }
    }

    /// The tube angles a band spans over its half (unwrapped against the
    /// half's start): the end's leading band from the lower branch to the
    /// upper, the start's trailing band from the upper to the lower a turn on.
    fn band_span(&self, end: u8, half: u8, psi: f64, reference: f64) -> Option<(f64, f64)> {
        let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
        let (lower, upper) = self.ring_branches(self.grid.times[own], psi)?;
        let (from, to) = if end == 1 { (lower, upper) } else { (upper, lower + TAU) };
        let shift = reference + (from - reference + PI).rem_euclid(TAU) - PI - from;
        let _ = half;
        Some((from + shift, to + shift))
    }

    /// A ring sheet's surface over the time fractions [lo, hi] (an
    /// envelope) or its whole chart (a band):
    /// - an envelope: interpolated (quintic both ways) through exact grazing
    ///   points over (ψ fraction, τ), its parameters the sheet's own, the
    ///   rows doubled until the cell middles miss by under 1e-10 of the
    ///   ring's size;
    /// - a band: the end pose's ring over the half's ψ and the band's tube
    ///   angles (with a margin), exact (rational).
    pub(super) fn ring_sheet_surface(&self, sheet: SheetId, lo: f64, hi: f64) -> Result<SheetSurface, KernelRefusal> {
        match sheet {
            SheetId::Envelope(..) => {
                let (mut nu, mut nt) = (32usize, 16usize);
                loop {
                    let us: Vec<f64> = (0..=nu).map(|i| i as f64 / nu as f64).collect();
                    let taus: Vec<f64> = (0..=nt).map(|k| lo + (hi - lo) * k as f64 / nt as f64).collect();
                    let exact = |u: f64, tau: f64| -> Result<Vec3, KernelRefusal> {
                        self.ring_point(sheet, [u, tau]).map(|q| q.world).ok_or_else(|| refuse("trim_ring_rows", "a ring envelope's rows are undefined in its span".into()))
                    };
                    let mut rows: Vec<Vec<Vec3>> = Vec::with_capacity(taus.len());
                    let mut knots_u = Vec::new();
                    for &tau in &taus {
                        let points = us.iter().map(|&u| exact(u, tau)).collect::<Result<Vec<_>, _>>()?;
                        let curve = frag(crate::interpolate_curve(&points, 5, &us), "trim_ring_row")?;
                        knots_u = curve.knots.clone();
                        rows.push(curve.control_points.iter().map(|c| c.point().unwrap_or(Vec3::ZERO)).collect());
                    }
                    let count = rows[0].len();
                    let mut net: Vec<Vec<Vec4>> = Vec::with_capacity(count);
                    let mut knots_v = Vec::new();
                    for i in 0..count {
                        let column: Vec<Vec3> = rows.iter().map(|r| r[i]).collect();
                        let curve = frag(crate::interpolate_curve(&column, 5, &taus), "trim_ring_column")?;
                        knots_v = curve.knots.clone();
                        net.push(curve.control_points.clone());
                    }
                    let surface = frag(NurbsSurface::new(5, 5, knots_u, knots_v, net), "trim_ring_envelope")?;
                    // The misses between samples along u (on the rows) and
                    // along τ (on the columns): each direction refined only
                    // while its own misses exceed the target.
                    let (mut along_u, mut along_t) = (0.0f64, 0.0f64);
                    for i in 0..nu {
                        for k in 0..=nt {
                            let u = 0.5 * (us[i] + us[i + 1]);
                            along_u = along_u.max(frag(surface.evaluate(u, taus[k]), "trim_ring_envelope")?.sub(exact(u, taus[k])?).length());
                        }
                    }
                    for i in 0..=nu {
                        for k in 0..nt {
                            let tau = 0.5 * (taus[k] + taus[k + 1]);
                            along_t = along_t.max(frag(surface.evaluate(us[i], tau), "trim_ring_envelope")?.sub(exact(us[i], tau)?).length());
                        }
                    }
                    let worst = along_u.max(along_t);
                    match next_fit_counts(nu, nt, along_u, along_t, 1e-8 * self.size) {
                        Some((u, t)) => (nu, nt) = (u, t),
                        None => {
                            if worst > 1e-7 * self.size {
                                return Err(refuse("trim_sheet_fit", format!("a ring envelope misses by {worst:.2e}")));
                            }
                            return Ok(SheetSurface { surface, strip: None, band: None, cap: None });
                        }
                    }
                }
            }
            SheetId::Band(end, half) => {
                let own = if end == 0 { 0 } else { self.grid.times.len() - 1 };
                let pose = self.grid.poses[own];
                let psi0 = self.half_psi(half, 0.0);
                let reference = self.band_span(end, half, psi0, 0.0).ok_or_else(|| refuse("trim_ring_band", "a ring band's branches are undefined".into()))?.0;
                let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
                for k in 0..=256 {
                    let psi = self.half_psi(half, k as f64 / 256.0);
                    let (a, b) = self.band_span(end, half, psi, reference).ok_or_else(|| refuse("trim_ring_band", "a ring band's branches are undefined".into()))?;
                    low = low.min(a);
                    high = high.max(b);
                }
                let margin = (0.25 * (TAU - (high - low))).clamp(1e-3, 0.25);
                let (v_low, v_high) = (low - margin, high + margin);
                let theta_v = v_high - v_low;
                if theta_v >= TAU {
                    return Err(refuse("trim_ring_band", "a ring band wraps all the way round its tube".into()));
                }
                let torus = self.ring();
                let (e1, e2) = self.ring_frame();
                let radial = e1.scale(psi0.cos()).add(e2.scale(psi0.sin()));
                let start = self.ring_at(psi0, v_low);
                let tube_axis = radial.cross(torus.axis);
                let tube_centre = torus.centre.add(radial.scale(torus.major));
                let generatrix = frag(frag(make_revolution(tube_centre, tube_axis, &frag(make_line(start, start.add(tube_axis)), "line")?, theta_v), "trim_ring_tube")?.iso_curve_v(0.0), "trim_ring_tube")?;
                let surface = frag(make_revolution(torus.centre, torus.axis, &generatrix, PI), "trim_ring_band")?;
                let moved = frag(
                    NurbsSurface::new(
                        surface.degree_u,
                        surface.degree_v,
                        surface.knots_u.clone(),
                        surface.knots_v.clone(),
                        surface.control_points.iter().map(|row| row.iter().map(|p| Vec4::from_point(pose.apply(p.point().unwrap_or(Vec3::ZERO)), p.w)).collect()).collect(),
                    ),
                    "trim_ring_band",
                )?;
                let spans_u = (surface.knots_u.len() - 6) / 2 + 1;
                let spans_v = (generatrix.knots.len() - 6) / 2 + 1;
                Ok(SheetSurface { surface: moved, strip: None, band: Some(BandChart { psi0, v_low, theta_v, spans_u, spans_v }), cap: None })
            }
            _ => Err(refuse("trim_ring", "not a ring sheet".into())),
        }
    }

    /// A band point's parameters on its torus patch.
    pub(super) fn band_uv(&self, chart: &BandChart, end: u8, half: u8, p: [f64; 2]) -> [f64; 2] {
        let psi = self.half_psi(half, p[0]);
        let mid = chart.v_low + 0.5 * chart.theta_v;
        let Some((a, b)) = self.band_span(end, half, psi, mid) else { return p };
        let v = a + p[1] * (b - a);
        [crate::circle_angle_to_parameter(chart.spans_u, PI, psi - chart.psi0), crate::circle_angle_to_parameter(chart.spans_v, chart.theta_v, v - chart.v_low)]
    }
}

/// The least margin between a ring's grazing branches, over the grazing
/// value's largest swing, its trim route takes on.
pub const RING_BRANCH_MARGIN: f64 = 0.015;

/// `BREP_SWEEP_RING_TRIM` (on unless =0): a ring tumbled by two turns that the
/// lofted walls refuse is swept by the trim engine.
pub fn ring_trim_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_TRIM", true)
}

/// The sweep of a ring (the torus about `axis` through `centre`, radii
/// `major` and `minor`) tumbled by two turns over [from, to], by the trim
/// engine: its sheets arranged on a pose grid, closed into one oriented
/// sphere (seams tried in turn where a corner below the grid leaves arcs
/// unmated), refined onto the exact sheets and built; its volume must agree
/// with the trims' flux.
pub fn sweep_ring_tumbled(centre: Vec3, axis: Vec3, major: f64, minor: f64, motion: &TwoTurnMotion, from: f64, to: f64) -> Result<BrepSolid, KernelRefusal> {
    if !(from.is_finite() && to.is_finite() && to > from) {
        return Err(refuse("trim_range", "the motion's range must run forward".into()));
    }
    let torus = Torus::new(centre, axis, major, minor)?;
    let degrees = motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs())) * (to - from).to_degrees();
    // Past about 200° of the fastest turn the pose grid grows past the time a
    // caller can wait (a whole turn takes a minute): refused before the work.
    if degrees > 200.0 + 1e-9 {
        return Err(refuse("trim_ring_range", format!("a tumble of {degrees:.0}° is longer than the ring's trim route takes in time")));
    }
    // Branches all but meeting somewhere (the margin between the tube's
    // grazing roots under a sixty-fifth of the grazing value's largest
    // swing): the arrangement does not close there; declined before the work
    // (a sweep that builds keeps 0.045 and more, the crossed tumbles' are
    // 0.0045 and 0.0011).
    {
        let probe = PoseGrid::new(motion, &torus, from, to, 2);
        let sweep = ConvexSweep::new_ring(&torus, motion, &probe)?;
        let (mut least, mut most) = (f64::INFINITY, 0.0f64);
        for i in 0..=120 {
            let t = from + (to - from) * i as f64 / 120.0;
            for k in 0..128 {
                let (a, b, c) = sweep.ring_harmonic(t, TAU * k as f64 / 128.0);
                let r = a.hypot(b);
                least = least.min(r - c.abs());
                most = most.max(r);
            }
        }
        if !(least > RING_BRANCH_MARGIN * most) {
            return Err(refuse("trim_ring_branches_near", format!("the ring's grazing branches come within {:.1e} of meeting (of a swing {most:.2}): not closed here", least.max(0.0))));
        }
    }
    let poses = ((10.0 * degrees).ceil() as usize).clamp(750, 6000) + 1;
    let rows = ((2.0 * degrees).ceil() as usize).clamp(150, 1200) + 1;
    let columns = 81;
    let grid = PoseGrid::new(motion, &torus, from, to, poses);
    let size = major + minor;
    let mut last = None;
    for seam in [45f64, 100.0, 20.0, 135.0, 70.0] {
        let sweep = ConvexSweep::new_ring_with_seam(&torus, motion, &grid, seam.to_radians())?;
        let mut arrangement = sweep.arrangement(rows, columns);
        let closure = arrangement.close(1e-2 * size, 2e-2 * size);
        if !closure.is_closed_surface() {
            last = Some(refuse("trim_arrangement", format!("the ring's trimmed sheets do not close into one oriented shell on a {poses}-pose grid")));
            continue;
        }
        // A surface meeting itself at points (a double curve's tip: the
        // envelope's cross-cap) is built valid, but a reader that integrates
        // round its faces' loops does not converge at the tip: refused (the
        // lofted walls after) until it is written so it reads.
        if closure.link_vertices != closure.mating.vertices {
            return Err(refuse("trim_ring_tips", "the ring's sweep meets itself at the tips of its double curves, which are not yet written so a reader integrates round them".into()));
        }
        let refined = sweep.refine_arcs(&arrangement, &closure);
        let solved = sweep.solve_vertices(&arrangement, &closure, &refined);
        let body = sweep.build(&arrangement, &closure, &refined, &solved)?;
        let problems = body.validate();
        if !problems.is_empty() {
            let first: Vec<String> = problems.iter().take(3).map(|p| p.message.clone()).collect();
            return Err(refuse("trim_assembly", format!("the ring's built sweep does not validate ({} problems: {})", problems.len(), first.join("; "))));
        }
        let built = frag(crate::solid_mass_properties(&body).map(|p| p.volume), "trim_volume")?;
        // The trims' flux as a check against a region missed or taken twice
        // (those miss by 1e-3 and more): to 1e-4, which 400 scanlines hold
        // even on a thin ring's many faces.
        let (flux, _, _) = sweep.flux_volume(&arrangement, &closure, 400);
        if (built - flux).abs() > 1e-4 * flux.abs().max(size.powi(3)) {
            return Err(refuse("trim_volume", format!("the ring's built sweep's volume {built:.7} disagrees with its trims' {flux:.7}")));
        }
        return Ok(body);
    }
    Err(last.expect("tried"))
}

/// The envelope fit's next sample counts (u, τ): each direction whose misses
/// exceed `target` doubles, up to 1024; None when none can (all within
/// target, or every direction over it already at the cap).
#[doc(hidden)]
pub fn next_fit_counts(nu: usize, nt: usize, along_u: f64, along_t: f64, target: f64) -> Option<(usize, usize)> {
    const CAP: usize = 1024;
    let grow_u = along_u > target && nu < CAP;
    let grow_t = along_t > target && nt < CAP;
    if !grow_u && !grow_t {
        return None;
    }
    Some((if grow_u { 2 * nu } else { nu }, if grow_t { 2 * nt } else { nt }))
}
