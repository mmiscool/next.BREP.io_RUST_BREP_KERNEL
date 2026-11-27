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
        Self::branches_of(self.ring_harmonic(t, psi))
    }

    /// [`ConvexSweep::ring_branches`] from the harmonic (a, b, c) read there.
    fn branches_of((a, b, c): (f64, f64, f64)) -> Option<(f64, f64)> {
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

    /// The world point a ring envelope's carrier is fitted through. That is
    /// the envelope itself, except deep inside a collar's disc: there no face
    /// lies, and the branch turns through every tube angle about the root
    /// (no continuous branch fills the disc). There it is blended, smoothly
    /// in (t, ψ), into the ring's core point: true from 0.99 of the disc's radius
    /// out, all core within a twentieth of it (a wide blend keeps the
    /// carriers small).
    pub(super) fn ring_fit_world(&self, sheet: SheetId, p: [f64; 2]) -> Option<Vec3> {
        if let SheetId::Envelope(branch, half) = sheet {
            let q = self.warp.as_ref().map_or(p, |w| w.chart(half, p));
            let (t, psi) = (self.t_of(q[1]), self.half_psi(half, q[0]));
            for c in &self.collars {
                let dpsi = (psi - c.psi + PI).rem_euclid(TAU) - PI;
                if !((t - c.t).abs() <= c.reach[0] && dpsi.abs() <= c.reach[1]) {
                    continue;
                }
                // The harmonic read once, for the disc's measure and the
                // envelope's point both (as `ring_point_whole` reads it).
                let harmonic = self.ring_harmonic(t, psi);
                let (a, b) = (harmonic.0, harmonic.1);
                let x = (a.hypot(b) / c.radius - 0.05) / 0.94;
                if x >= 1.0 {
                    continue;
                }
                let e = |y: f64| if y > 0.0 { (-1.0 / y).exp() } else { 0.0 };
                let weight = e(x) / (e(x) + e(1.0 - x));
                let core = self.motion.pose(t).apply(self.ring_at(psi, 0.0).add(self.ring_at(psi, PI)).scale(0.5));
                if weight <= 0.0 {
                    return Some(core);
                }
                let (lower, upper) = Self::branches_of(harmonic)?;
                let truth = self.grazing_point(t, psi, if branch == 0 { lower } else { upper }).world;
                return Some(core.add(truth.sub(core).scale(weight)));
            }
        }
        self.ring_point_whole(sheet, p).map(|q| q.world)
    }

    /// A ring sheet's point (None where its branches are not defined, and on
    /// an envelope inside a collar's disc).
    pub(super) fn ring_point(&self, sheet: SheetId, p: [f64; 2]) -> Option<SheetPoint> {
        self.ring_point_cut(sheet, p, true)
    }

    /// [`ConvexSweep::ring_point`] with no disc cut from the envelopes: for
    /// the carriers, which run across a disc, and the solves' raw points.
    pub(super) fn ring_point_whole(&self, sheet: SheetId, p: [f64; 2]) -> Option<SheetPoint> {
        self.ring_point_cut(sheet, p, false)
    }

    fn ring_point_cut(&self, sheet: SheetId, p: [f64; 2], cut: bool) -> Option<SheetPoint> {
        match sheet {
            SheetId::Envelope(branch, half) => {
                let p = self.warp.as_ref().map_or(p, |w| w.chart(half, p));
                let t = self.t_of(p[1]);
                let psi = self.half_psi(half, p[0]);
                // A collar's disc is cut from the envelope.
                if cut && self.collar_holding(t, psi).is_some() {
                    return None;
                }
                let (lower, upper) = self.ring_branches(t, psi)?;
                Some(self.grazing_point(t, psi, if branch == 0 { lower } else { upper }))
            }
            SheetId::Collar(k) => {
                let collar = self.collars.get(usize::from(k))?;
                let (lambda, v) = (collar.radius * (2.0 * p[0] - 1.0), TAU * p[1]);
                let (t, psi) = self.collar_at(collar, lambda, v)?;
                Some(self.grazing_point(t, psi, v))
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

    /// The envelope's point at time t, ψ and the grazing tube angle v.
    fn grazing_point(&self, t: f64, psi: f64, v: f64) -> SheetPoint {
        let x = self.ring_at(psi, v);
        let pose = self.motion.pose(t);
        let y = pose.apply(x);
        // The depth at times beside t (the point is on the ring at t,
        // grazing): its second difference says whether the neighbouring poses
        // hold it (a fold); per radian of the fastest turn, as for a
        // polyhedron's strips.
        let rate = self.motion.rates.iter().fold(0.0f64, |a, r| a.max(r.abs())).max(1e-300);
        let h = 1e-4 / rate;
        let depth = |s: f64| self.depth(self.motion.pose(s).invert(y));
        let local = -(depth(t + h) + depth(t - h)) / 1e-8;
        SheetPoint { world: y, tool: x, t, own: self.own_index(t), local }
    }

    /// The grazing harmonic's (a, b) at (t, ψ).
    fn harmonic_ab(&self, t: f64, psi: f64) -> [f64; 2] {
        let (a, b, _) = self.ring_harmonic(t, psi);
        [a, b]
    }

    /// The collar whose disc holds (t, ψ): within its box about the root, and
    /// (a, b) shorter than its radius there.
    pub(super) fn collar_holding(&self, t: f64, psi: f64) -> Option<usize> {
        self.collars.iter().position(|c| {
            let dpsi = (psi - c.psi + PI).rem_euclid(TAU) - PI;
            (t - c.t).abs() <= c.reach[0] && dpsi.abs() <= c.reach[1] && {
                let [a, b] = self.harmonic_ab(t, psi);
                a.hypot(b) < c.radius
            }
        })
    }

    /// Who borders an envelope where its kept region runs into a cut disc
    /// (the parameter point just outside the sheet): the collar.
    pub(super) fn ring_hole_border(&self, sheet: SheetId, p: [f64; 2]) -> Option<Border> {
        let SheetId::Envelope(_, half) = sheet else { return None };
        if self.collars.is_empty() {
            return None;
        }
        let p = self.warp.as_ref().map_or(p, |w| w.chart(half, p));
        let (t, psi) = (self.t_of(p[1]), self.half_psi(half, p[0]));
        self.collar_holding(t, psi).map(|k| Border::Sheet(SheetId::Collar(k as u8)))
    }

    /// (t, ψ) where (a, b) = λ(−sin v, cos v) about the collar's root
    /// (Newton from the root's linear step).
    fn collar_at(&self, collar: &RingCollar, lambda: f64, v: f64) -> Option<(f64, f64)> {
        let target = [-lambda * v.sin(), lambda * v.cos()];
        let solve = |m: [[f64; 2]; 2], r: [f64; 2]| -> Option<[f64; 2]> {
            let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
            (det.abs() > 1e-300).then(|| [(m[1][1] * r[0] - m[0][1] * r[1]) / det, (m[0][0] * r[1] - m[1][0] * r[0]) / det])
        };
        let step = solve(collar.jacobian, target)?;
        let (mut t, mut psi) = (collar.t + step[0], collar.psi + step[1]);
        let scale = collar.radius.max(1e-300);
        // Newton's first step on the Jacobian there, then that Jacobian kept
        // (the disc is small, the steps after the first tiny): the same
        // residual reached for a fifth of the readings.
        let mut kept: Option<[[f64; 2]; 2]> = None;
        for _ in 0..40 {
            let f = self.harmonic_ab(t, psi);
            let r = [f[0] - target[0], f[1] - target[1]];
            if r[0].hypot(r[1]) <= 1e-14 * scale {
                return Some((t, psi));
            }
            let m = *kept.get_or_insert_with(|| self.harmonic_jacobian(t, psi));
            let d = solve(m, r)?;
            t -= d[0];
            psi -= d[1];
        }
        let f = self.harmonic_ab(t, psi);
        ((f[0] - target[0]).hypot(f[1] - target[1]) <= 1e-11 * scale).then_some((t, psi))
    }

    /// ∂(a, b)/∂(t, ψ), central differences.
    fn harmonic_jacobian(&self, t: f64, psi: f64) -> [[f64; 2]; 2] {
        let h = 1e-6;
        let (tp, tm, pp, pm) = (self.harmonic_ab(t + h, psi), self.harmonic_ab(t - h, psi), self.harmonic_ab(t, psi + h), self.harmonic_ab(t, psi - h));
        [[(tp[0] - tm[0]) / (2.0 * h), (pp[0] - pm[0]) / (2.0 * h)], [(tp[1] - tm[1]) / (2.0 * h), (pp[1] - pm[1]) / (2.0 * h)]]
    }

    /// The points over [from, to] where the whole tube circle grazes (a = b
    /// = 0), each with a collar whose disc keeps clear of the motion's ends
    /// and of the halves' seam; None where one cannot be given one.
    pub(super) fn ring_collars(&self, from: f64, to: f64) -> Option<Vec<RingCollar>> {
        let (nt, np) = (400usize, 360usize);
        let at = |i: usize, k: usize| (from + (to - from) * i as f64 / nt as f64, TAU * k as f64 / np as f64);
        let r: Vec<Vec<f64>> = (0..=nt).map(|i| (0..np).map(|k| { let (t, psi) = at(i, k); let [a, b] = self.harmonic_ab(t, psi); a.hypot(b) }).collect()).collect();
        let most = r.iter().flatten().fold(0.0f64, |m, &x| m.max(x));
        let mut out: Vec<RingCollar> = Vec::new();
        for i in 1..nt {
            for k in 0..np {
                let here = r[i][k];
                let around = [r[i - 1][k], r[i + 1][k], r[i][(k + 1) % np], r[i][(k + np - 1) % np]];
                if !(here < 0.05 * most && around.iter().all(|&x| here <= x)) {
                    continue;
                }
                let (mut t, mut psi) = at(i, k);
                for _ in 0..40 {
                    let f = self.harmonic_ab(t, psi);
                    let m = self.harmonic_jacobian(t, psi);
                    let det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
                    if !(det.abs() > 1e-300) {
                        break;
                    }
                    t -= (m[1][1] * f[0] - m[0][1] * f[1]) / det;
                    psi -= (m[0][0] * f[1] - m[1][0] * f[0]) / det;
                }
                let f = self.harmonic_ab(t, psi);
                let psi = psi.rem_euclid(TAU);
                if !(f[0].hypot(f[1]) <= 1e-12 * most && t > from && t < to) || out.iter().any(|c| (c.t - t).abs() < 1e-6 && ((c.psi - psi + PI).rem_euclid(TAU) - PI).abs() < 1e-6) {
                    continue;
                }
                let jacobian = self.harmonic_jacobian(t, psi);
                let half = if (psi - self.seam).rem_euclid(TAU) < PI { 0u8 } else { 1u8 };
                // The radius: a thirtieth of the largest swing, halved until
                // the disc keeps a tenth of its size clear of the motion's ends
                // and the seam.
                let mut radius = most / 30.0;
                let mut found = None;
                for _ in 0..6 {
                    let collar = RingCollar { t, psi, radius, half, jacobian, reach: [f64::INFINITY; 2] };
                    let rim: Option<Vec<(f64, f64)>> = (0..96).map(|j| self.collar_at(&collar, radius, TAU * j as f64 / 96.0)).collect();
                    if let Some(rim) = rim {
                        let dt = rim.iter().fold(0.0f64, |m, p| m.max((p.0 - t).abs()));
                        let dpsi = rim.iter().fold(0.0f64, |m, p| m.max(((p.1 - psi + PI).rem_euclid(TAU) - PI).abs()));
                        let seam_gap = |x: f64| { let d = (x - self.seam).rem_euclid(PI); d.min(PI - d) };
                        if t - 1.1 * dt > from && t + 1.1 * dt < to && seam_gap(psi) > 1.1 * dpsi {
                            found = Some(RingCollar { reach: [1.05 * dt, 1.05 * dpsi], ..collar });
                            break;
                        }
                    }
                    radius *= 0.5;
                }
                out.push(found?);
            }
        }
        Some(out)
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
            // A collar: its tube angle's two ends meet each other; at λ = −ε
            // the upper branch's envelope, at +ε the lower's.
            SheetId::Collar(k) => {
                let half = self.collars.get(usize::from(k)).map_or(0, |c| c.half);
                match side {
                    0 | 1 => Border::Sheet(SheetId::Collar(k)),
                    2 => Border::Sheet(SheetId::Envelope(1, half)),
                    _ => Border::Sheet(SheetId::Envelope(0, half)),
                }
            }
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
        if let Some(k) = self.collar_holding(s, psi) {
            return Border::Sheet(SheetId::Collar(k as u8));
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
    /// A ring envelope's carrier through its exact points at stations refined
    /// only where it misses: as the uniform fit (rows through the stations in
    /// u, columns through the rows' controls in τ, the same parameters (u, τ)),
    /// but each u or τ interval whose own midpoint misses exceed the target
    /// (1e-8 of the size, read on every station of the other direction) is
    /// halved, and neighbours kept within twice each other's length; up to
    /// 1024 intervals a direction, past which a carrier within 1e-7 is kept
    /// and any other refused, as the uniform fit does.
    fn ring_envelope_adaptive(&self, sheet: SheetId, lo: f64, hi: f64, span: (f64, f64)) -> Result<SheetSurface, KernelRefusal> {
        const CAP: usize = 1024;
        let exact = |u: f64, tau: f64| -> Result<Vec3, KernelRefusal> {
            self.ring_fit_world(sheet, [u, tau]).ok_or_else(|| refuse("trim_ring_rows", "a ring envelope's rows are undefined in its span".into()))
        };
        let mut us: Vec<f64> = (0..=32).map(|i| span.0 + (span.1 - span.0) * i as f64 / 32.0).collect();
        let mut taus: Vec<f64> = (0..=16).map(|k| lo + (hi - lo) * k as f64 / 16.0).collect();
        let target = 1e-8 * self.size;
        loop {
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
            // Each interval's own misses: at its midpoint, on every station of
            // the other direction.
            let mut miss_u = vec![0.0f64; us.len() - 1];
            for i in 0..us.len() - 1 {
                let u = 0.5 * (us[i] + us[i + 1]);
                for &tau in &taus {
                    miss_u[i] = miss_u[i].max(frag(surface.evaluate(u, tau), "trim_ring_envelope")?.sub(exact(u, tau)?).length());
                }
            }
            let mut miss_t = vec![0.0f64; taus.len() - 1];
            for k in 0..taus.len() - 1 {
                let tau = 0.5 * (taus[k] + taus[k + 1]);
                for &u in &us {
                    miss_t[k] = miss_t[k].max(frag(surface.evaluate(u, tau), "trim_ring_envelope")?.sub(exact(u, tau)?).length());
                }
            }
            let worst = miss_u.iter().chain(&miss_t).fold(0.0f64, |a, &b| a.max(b));
            let grow_u = miss_u.iter().any(|&m| m > target) && us.len() - 1 < CAP;
            let grow_t = miss_t.iter().any(|&m| m > target) && taus.len() - 1 < CAP;
            if !grow_u && !grow_t {
                if !(worst <= 1e-7 * self.size) {
                    return Err(refuse("trim_sheet_fit", format!("a ring envelope misses by {worst:.2e}")));
                }
                return Ok(SheetSurface { surface, strip: None, band: None, cap: None });
            }
            if grow_u {
                us = refined_stations(&us, &miss_u, target, CAP);
            }
            if grow_t {
                taus = refined_stations(&taus, &miss_t, target, CAP);
            }
        }
    }

    pub(super) fn ring_sheet_surface(&self, sheet: SheetId, lo: f64, hi: f64) -> Result<SheetSurface, KernelRefusal> {
        self.ring_sheet_surface_over(sheet, lo, hi, (0.0, 1.0))
    }

    /// [`ConvexSweep::ring_sheet_surface`] with an envelope's ψ fractions
    /// `span` (a band takes its whole chart).
    pub(super) fn ring_sheet_surface_over(&self, sheet: SheetId, lo: f64, hi: f64, span: (f64, f64)) -> Result<SheetSurface, KernelRefusal> {
        match sheet {
            SheetId::Envelope(..) | SheetId::Collar(..) if ring_adaptive_on() || matches!(sheet, SheetId::Collar(..)) => self.ring_envelope_adaptive(sheet, lo, hi, span),
            SheetId::Envelope(..) => {
                let (mut nu, mut nt) = (32usize, 16usize);
                loop {
                    let us: Vec<f64> = (0..=nu).map(|i| span.0 + (span.1 - span.0) * i as f64 / nu as f64).collect();
                    let taus: Vec<f64> = (0..=nt).map(|k| lo + (hi - lo) * k as f64 / nt as f64).collect();
                    let exact = |u: f64, tau: f64| -> Result<Vec3, KernelRefusal> {
                        self.ring_fit_world(sheet, [u, tau]).ok_or_else(|| refuse("trim_ring_rows", "a ring envelope's rows are undefined in its span".into()))
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
                    // Past a collar's root the grazing branch can turn by more
                    // than the band leaves of the tube over a half: no patch
                    // of the ring holds the band, but it is smooth in its own
                    // chart, where its carrier is fitted as an envelope's is
                    // (a sweep without collars too, under its switch).
                    if !self.collars.is_empty() || ring_band_fit_on() {
                        return self.ring_envelope_adaptive(sheet, -0.02, 1.02, (0.0, 1.0));
                    }
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

/// One direction's stretch of a chart about `centre` (a fraction in [0, 1]):
/// the chart's fraction s reads the sheet's fraction x where
/// F(x) = s · F(1), F(x) = x + gain · width · (atan((x − c)/width) +
/// atan(c/width)), so the stations cluster about c as 1 + gain over the
/// width, the ends fixed.
#[derive(Clone, Copy, Debug)]
pub struct Stretch {
    pub centre: f64,
    pub width: f64,
    pub gain: f64,
}

impl Stretch {
    fn cumulative(&self, x: f64) -> f64 {
        x + self.gain * self.width * (((x - self.centre) / self.width).atan() + (self.centre / self.width).atan())
    }

    /// The sheet's fraction at the chart's fraction `s` (monotone, ends fixed).
    pub fn sheet(&self, s: f64) -> f64 {
        if !(self.gain > 0.0) || s <= 0.0 || s >= 1.0 {
            return s;
        }
        let target = s * self.cumulative(1.0);
        let (mut lo, mut hi) = (0.0f64, 1.0f64);
        let mut x = s;
        for _ in 0..80 {
            let f = self.cumulative(x) - target;
            if f == 0.0 || hi - lo <= 4.0 * f64::EPSILON {
                break;
            }
            if f > 0.0 { hi = x } else { lo = x }
            let slope = 1.0 + self.gain / (1.0 + ((x - self.centre) / self.width).powi(2));
            let next = x - f / slope;
            // A Newton step strictly inside the bracket, else its middle (a
            // step onto the bracket's end can swing between its ends without
            // closing it); a step that does not move has converged.
            let next = if next > lo && next < hi { next } else { 0.5 * (lo + hi) };
            if next == x {
                break;
            }
            x = next;
        }
        x
    }
}

/// A ring's envelope chart stretched about where its grazing branches all
/// but meet (`BREP_SWEEP_RING_MEETING`): there the grazing harmonic's
/// amplitude √(a² + b²) nearly vanishes, its phase swings round in a small
/// (ψ, t) disc, and the envelopes move fast across it; a uniform chart's
/// rows and columns step over it. The time stretch applies to both halves,
/// the ψ stretch to the half holding the meeting.
#[derive(Clone, Copy, Debug)]
pub struct RingWarp {
    pub time: Stretch,
    pub psi: Stretch,
    pub half: u8,
}

impl RingWarp {
    /// The sheet's (ψ fraction, τ) at the chart's (u, τ').
    pub fn chart(&self, half: u8, p: [f64; 2]) -> [f64; 2] {
        let u = if half == self.half { self.psi.sheet(p[0]) } else { p[0] };
        [u, self.time.sheet(p[1])]
    }
}

/// A collar about a point where the whole tube circle grazes: the root
/// (t, ψ) of the grazing harmonic's (a, b), its half, the disc's radius in
/// (a, b), ∂(a, b)/∂(t, ψ) at the root, and the disc's reach in t and ψ.
#[derive(Clone, Copy, Debug)]
pub struct RingCollar {
    pub t: f64,
    pub psi: f64,
    pub radius: f64,
    pub half: u8,
    jacobian: [[f64; 2]; 2],
    reach: [f64; 2],
}

/// `BREP_SWEEP_RING_COLLAR` (default on; `=0` restores the decline): a ring
/// whose whole tube circle grazes at some point of the motion (its grazing
/// harmonic's a = b = 0, where the branches' names turn) is swept with a
/// collar there ([`RingCollar`], `SheetId::Collar`) instead of declined by
/// [`RING_BRANCH_MARGIN`]. A motion with no such point is still declined by
/// the margin where its branches all but meet.
pub fn ring_collar_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_COLLAR", true)
}

/// `BREP_SWEEP_RING_BAND_FIT` (default off): an end band whose grazing
/// branch turns so far over a half that the band wraps past the whole tube
/// is fitted in its own chart instead of refused (with collars it always is).
pub fn ring_band_fit_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_BAND_FIT", false)
}

/// `BREP_SWEEP_RING_MEETING` (default off): a ring whose grazing branches
/// all but meet is swept on an envelope chart stretched about the meeting
/// ([`RingWarp`]) instead of declined by [`RING_BRANCH_MARGIN`].
pub fn ring_meeting_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_MEETING", false)
}

impl<'a> ConvexSweep<'a> {
    /// Where the grazing branches come nearest meeting over [from, to] and
    /// each half's ψ: the least of √(a² + b²) − |c|, refined, and the
    /// stretch that puts `cells` grid cells across the fast region (its
    /// width the least over the gradient of (a, b)), given `rows` × `columns`.
    pub(super) fn ring_meeting_warp(&self, from: f64, to: f64, rows: usize, columns: usize) -> Option<RingWarp> {
        let margin = |tau: f64, psi: f64| {
            let (a, b, c) = self.ring_harmonic(from + (to - from) * tau, psi);
            a.hypot(b) - c.abs()
        };
        let (mut best, mut at) = (f64::INFINITY, (0.0, 0.0));
        for i in 0..=240 {
            for k in 0..256 {
                let (tau, psi) = (i as f64 / 240.0, TAU * k as f64 / 256.0);
                let m = margin(tau, psi);
                if m < best { best = m; at = (tau, psi); }
            }
        }
        // Two refinements on a finer grid about the best.
        let (mut dt, mut dp) = (1.0 / 240.0, TAU / 256.0);
        for _ in 0..2 {
            let centre = at;
            for i in -10..=10 {
                for k in -10..=10 {
                    let (tau, psi) = ((centre.0 + dt * i as f64 / 5.0).clamp(0.0, 1.0), centre.1 + dp * k as f64 / 5.0);
                    let m = margin(tau, psi);
                    if m < best { best = m; at = (tau, psi); }
                }
            }
            dt /= 5.0;
            dp /= 5.0;
        }
        let (tau, psi) = at;
        let psi = psi.rem_euclid(TAU);
        // The gradient of (a, b) there, per chart fraction (τ, and u = ψ / π).
        let ab = |tau: f64, psi: f64| { let (a, b, _) = self.ring_harmonic(from + (to - from) * tau, psi); (a, b) };
        let h = 1e-6;
        let (a1, b1) = ab((tau + h).min(1.0), psi);
        let (a0, b0) = ab((tau - h).max(0.0), psi);
        let g_tau = (a1 - a0).hypot(b1 - b0) / ((tau + h).min(1.0) - (tau - h).max(0.0));
        let (a1, b1) = ab(tau, psi + h);
        let (a0, b0) = ab(tau, psi - h);
        let g_u = (a1 - a0).hypot(b1 - b0) / (2.0 * h) * PI;
        let amplitude = { let (a, b) = ab(tau, psi); a.hypot(b) };
        if !(g_tau > 0.0 && g_u > 0.0 && amplitude.is_finite()) {
            return None;
        }
        const CELLS: f64 = 24.0;
        let stretch = |centre: f64, width: f64, stations: usize| {
            let width = width.clamp(1e-6, 0.5);
            let gain = (CELLS / ((stations - 1) as f64 * width) - 1.0).clamp(0.0, 400.0);
            Stretch { centre, width, gain }
        };
        let half = if (psi - self.seam).rem_euclid(TAU) < PI { 0u8 } else { 1u8 };
        let u = ((psi - self.seam).rem_euclid(TAU) - PI * f64::from(half)) / PI;
        Some(RingWarp { time: stretch(tau, 3.0 * amplitude / g_tau, rows), psi: stretch(u, 3.0 * amplitude / g_u, columns), half })
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
    // caller can wait: refused before the work. A whole turn (about a minute)
    // is taken under the tip switch, which closes the turn's self-meetings.
    let longest = if tip_splice_on() { 360.0 } else { 200.0 };
    if degrees > longest + 1e-9 {
        return Err(refuse("trim_ring_range", format!("a tumble of {degrees:.0}° is longer than the ring's trim route takes in time")));
    }
    // Branches all but meeting somewhere (the margin between the tube's
    // grazing roots under a sixty-fifth of the grazing value's largest
    // swing): the arrangement does not close there; declined before the work
    // (a sweep that builds keeps 0.045 and more, the crossed tumbles' are
    // 0.0045 and 0.0011).
    // The seam the early read found the margin on is tried first.
    let mut seams = RING_SEAMS.to_vec();
    if !ring_meeting_on() {
        match ring_branches_declined(&torus, motion, from, to)? {
            Err(refusal) => return Err(refusal),
            Ok(seam) => {
                seams.retain(|&s| s != seam);
                seams.insert(0, seam);
            }
        }
    }
    let poses = ((10.0 * degrees).ceil() as usize).clamp(750, 6000) + 1;
    let rows = ((2.0 * degrees).ceil() as usize).clamp(150, 1200) + 1;
    let columns = 81;
    // Where the branches all but meet (`BREP_SWEEP_RING_MEETING`): the poses
    // stretched in time as the envelope chart's rows are.
    let stretch = if ring_meeting_on() && !ring_collar_on() {
        let probe = PoseGrid::new(motion, &torus, from, to, 2);
        ConvexSweep::new_ring(&torus, motion, &probe).ok().and_then(|s| s.ring_meeting_warp(from, to, rows, columns)).map(|w| w.time)
    } else {
        None
    };
    let grid = stretch
        .and_then(|time| PoseGrid::new_with_times(motion, &torus, (0..poses).map(|i| from + (to - from) * time.sheet(i as f64 / (poses - 1) as f64)).collect()))
        .unwrap_or_else(|| PoseGrid::new(motion, &torus, from, to, poses));
    let size = major + minor;
    let mut last = None;
    for seam in seams {
        let mut sweep = ConvexSweep::new_ring_with_seam(&torus, motion, &grid, seam.to_radians())?;
        if ring_collar_on() {
            match sweep.ring_collars(from, to) {
                Some(collars) => sweep.collars = collars,
                None => {
                    last = Some(refuse("trim_ring_collar", "a point where the ring's whole tube circle grazes takes no collar on this seam".into()));
                    continue;
                }
            }
        } else if ring_meeting_on() {
            sweep.warp = sweep.ring_meeting_warp(from, to, rows, columns);
        }
        let mut arrangement = sweep.arrangement(rows, columns);
        let mut closure = arrangement.close(1e-2 * size, 2e-2 * size);
        // Each face through a double curve's two preimages cut between them
        // (the switch) and closed again; cuts after which the sheets no
        // longer close as before are undone.
        if tip_splice_on() && closure.is_closed_surface() {
            let before = arrangement.clone();
            if sweep.cut_self_meeting_loops(&mut arrangement, 2e-2 * size) > 0 {
                let again = arrangement.close(1e-2 * size, 2e-2 * size);
                if again.is_closed_surface() && again.euler == closure.euler {
                    closure = again;
                } else {
                    arrangement = before;
                }
            }
        }
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
        // The trims' flux at 400 scanlines, read once: its sign orients the
        // faces (as build's own 200-scanline reading would; that reading is
        // kept where the flux is too small to trust its sign,
        // `BREP_SWEEP_RING_ONE_FLUX=0` keeps it always), its size checks the
        // built volume below.
        let raw = sweep.flux_volume_raw(&arrangement, &closure, 400);
        let given = (ring_one_flux_on() && raw.abs() > 1e-3 * size.powi(3)).then_some(raw);
        let body = sweep.build_with_sign(&arrangement, &closure, &refined, &solved, given)?;
        let problems = body.validate();
        if !problems.is_empty() {
            let first: Vec<String> = problems.iter().take(3).map(|p| p.message.clone()).collect();
            return Err(refuse("trim_assembly", format!("the ring's built sweep does not validate ({} problems: {})", problems.len(), first.join("; "))));
        }
        let built = frag(crate::solid_mass_properties(&body).map(|p| p.volume), "trim_volume")?;
        // The trims' flux as a check against a region missed or taken twice
        // (those miss by 1e-3 and more): to 1e-4, which 400 scanlines hold
        // even on a thin ring's many faces.
        let flux = raw.abs();
        if (built - flux).abs() > 1e-4 * flux.abs().max(size.powi(3)) {
            return Err(refuse("trim_volume", format!("the ring's built sweep's volume {built:.7} disagrees with its trims' {flux:.7}")));
        }
        return Ok(body);
    }
    Err(last.expect("tried"))
}

/// The seams the ring route tries, in order.
const RING_SEAMS: [f64; 5] = [45.0, 100.0, 20.0, 135.0, 70.0];

/// The ring route's early decline: the branches all but meet somewhere (the
/// margin between the tube's grazing roots under [`RING_BRANCH_MARGIN`] of the
/// grazing value's largest swing), so the arrangement does not close there.
/// Under the collar a true root of (a, b) is the collar's, so the margin is
/// read outside the collars' discs, on each of the route's seams with the
/// collars it takes, and the motion is declined only where it fails on every
/// one; a motion whose branches only come near meeting elsewhere (no root
/// there) is still declined before the work.
/// Some(refusal) to decline.
pub(super) fn ring_branches_declined(torus: &Torus, motion: &TwoTurnMotion, from: f64, to: f64) -> Result<Result<f64, KernelRefusal>, KernelRefusal> {
    let probe = PoseGrid::new(motion, torus, from, to, 2);
    // The margin on one seam: outside the collars it takes (none without the
    // switch), against the largest swing. (least, most)
    let margin_on = |seam: f64| -> Result<(f64, f64), KernelRefusal> {
        let mut sweep = ConvexSweep::new_ring_with_seam(torus, motion, &probe, seam.to_radians())?;
        if ring_collar_on() {
            if let Some(collars) = sweep.ring_collars(from, to) {
                sweep.collars = collars;
            }
        }
        let (mut least, mut most) = (f64::INFINITY, 0.0f64);
        for i in 0..=120 {
            let t = from + (to - from) * i as f64 / 120.0;
            for k in 0..128 {
                let psi = TAU * k as f64 / 128.0;
                let (a, b, c) = sweep.ring_harmonic(t, psi);
                let r = a.hypot(b);
                most = most.max(r);
                if sweep.collar_holding(t, psi).is_none() {
                    least = least.min(r - c.abs());
                }
            }
        }
        Ok((least, most))
    };
    // Without the collar the seam changes nothing; with it, any seam on which
    // the margin holds is one the route can sweep on.
    let seams: &[f64] = if ring_collar_on() { &RING_SEAMS } else { &RING_SEAMS[..1] };
    let mut first = None;
    for &seam in seams {
        let (least, most) = margin_on(seam)?;
        if least > RING_BRANCH_MARGIN * most {
            return Ok(Ok(seam));
        }
        first.get_or_insert((least, most));
    }
    let (least, most) = first.expect("one seam read");
    Ok(Err(refuse("trim_ring_branches_near", format!("the ring's grazing branches come within {:.1e} of meeting (of a swing {most:.2}): not closed here", least.max(0.0)))))
}

/// `BREP_SWEEP_RING_ONE_FLUX` (default on; `=0` restores build's own sign
/// reading): the ring route reads its trims' flux once, for the faces'
/// orientation and the volume check both.
pub fn ring_one_flux_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_ONE_FLUX", true)
}

/// `BREP_SWEEP_RING_ADAPTIVE` (default on; `=0` restores the uniform fit): a ring envelope's carrier
/// refined only where it misses ([`ConvexSweep::ring_envelope_adaptive`])
/// instead of doubling every interval.
pub fn ring_adaptive_on() -> bool {
    crate::switch!("BREP_SWEEP_RING_ADAPTIVE", true)
}

/// The stations with each interval whose miss exceeds `target` halved, the
/// worst first, each with the halvings it then needs so that no interval is
/// more than twice a neighbour's length (graded, so the interpolant through
/// them does not ripple); a halving whose grading would take the stations
/// past `cap` intervals is not made, so the result is always graded (given
/// graded stations).
#[doc(hidden)]
pub fn refined_stations(stations: &[f64], misses: &[f64], target: f64, cap: usize) -> Vec<f64> {
    let n = stations.len() - 1;
    let mut order: Vec<usize> = (0..n).filter(|&i| misses[i] > target).collect();
    order.sort_by(|&a, &b| misses[b].total_cmp(&misses[a]));
    let mut out = stations.to_vec();
    for i in order {
        if out.len() - 1 >= cap {
            break;
        }
        let middle = 0.5 * (stations[i] + stations[i + 1]);
        let Err(at) = out.binary_search_by(|x| x.total_cmp(&middle)) else { continue };
        let mut candidate = out.clone();
        candidate.insert(at, middle);
        let candidate = graded(candidate);
        if candidate.len() - 1 <= cap {
            out = candidate;
        }
    }
    out
}

/// `stations` with any interval more than twice a neighbour's length halved
/// until none is.
fn graded(mut out: Vec<f64>) -> Vec<f64> {
    loop {
        let lengths: Vec<f64> = out.windows(2).map(|w| w[1] - w[0]).collect();
        let wide = (0..lengths.len()).find(|&i| {
            (i > 0 && lengths[i] > 2.0 * lengths[i - 1] * (1.0 + 1e-12)) || (i + 1 < lengths.len() && lengths[i] > 2.0 * lengths[i + 1] * (1.0 + 1e-12))
        });
        match wide {
            Some(i) => out.insert(i + 1, 0.5 * (out[i] + out[i + 1])),
            None => return out,
        }
    }
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

