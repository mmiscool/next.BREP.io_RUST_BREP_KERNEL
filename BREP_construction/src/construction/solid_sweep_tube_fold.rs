//! The torus tube's read near a station's own time: whether a point q of K
//! (at distance r from the core circle at its own time t) is reached closer
//! than r by the ring at the times around t. The cover read elsewhere passes
//! over that window, where the reach f(s) = dist(q, core circle at s) is r
//! with zero slope at s = t; near a fold its curvature there vanishes, and the
//! flank beyond is reached from just beside t.
//!
//! Three answers, each proved:
//! - covered: a witness time s with f(s) < r − tie;
//! - clear: over the window f >= r − tie, by branch and bound with a
//!   second-order lower bound on each interval: f's second derivative is at
//!   most M2 = |x''| + |x'|²·‖H‖, x(s) the preimage of q (|x''| by the two
//!   turns' bend, |x'| by the motion's speed bound at q) and H the Hessian of
//!   the distance to a circle, ‖H‖ <= max(1/d, 1/ρ) (d the distance, ρ the
//!   distance from the circle's axis), each bounded below over the interval
//!   from its ends by its Lipschitz constant |x'|;
//! - tie: neither shown within the budget (at a double curve, a tip).
//!
//! Both reads hold only inside an envelope, in the ring's own units (lengths
//! in its major radius, time in its fastest rate): [`Tube::in_envelope`].
//! Outside it they answer conservatively (a tie, no fold measure) before any
//! arithmetic. The fold measure is one-sided: an upper bound, its rounding
//! added, so "below 1" is certified and a value near 1 is not.
use super::Tube;
use crate::Vec3;

/// What the read near a station's own time shows.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OwnRead {
    /// A time beside the station's own whose ring holds q closer than r − tie.
    Covered(f64),
    /// No time of the window holds q closer than r − tie.
    Clear,
    /// Neither shown within the budget.
    Tie,
}

/// Reads of f a window's read may spend before it answers a tie.
const OWN_BUDGET: usize = 4096;

/// The envelope, in the ring's own units: the most a pose turns over the
/// range (radians), the farthest a turning axis, or a read point, lies from
/// the ring's centre, the narrowest tube (minor over major), and the ring's
/// size (major radius; so no length's square over- or underflows). Bench rows
/// sit orders of magnitude inside.
const ENVELOPE_SIZE: [f64; 2] = [1e-100, 1e100];
const ENVELOPE_ANGLE: f64 = 1e4;
const ENVELOPE_ARM: f64 = 1e3;
const ENVELOPE_THIN: f64 = 1e-6;

impl Tube {
    /// Whether the ring and its motion lie inside the reads' envelope (see
    /// the module docs and the ENVELOPE constants).
    pub(crate) fn in_envelope(&self) -> bool {
        let size = self.major;
        let w = self.motion.rates.iter().fold(0.0f64, |m, r| m.max(r.abs()));
        let reach = self.from.abs().max(self.to.abs());
        let arms = (0..2).filter(|&i| self.motion.rates[i] != 0.0).map(|i| self.motion.points[i].sub(self.centre).length()).fold(0.0f64, f64::max);
        let thin = self.minor / size;
        (ENVELOPE_SIZE[0]..=ENVELOPE_SIZE[1]).contains(&size)
            && w.is_finite()
            && self.from.is_finite()
            && self.to.is_finite()
            && w * reach <= ENVELOPE_ANGLE
            && arms <= ENVELOPE_ARM * size
            && (ENVELOPE_THIN..=1.0).contains(&thin)
    }

    /// The motion read about the ring's own centre: its axes' points moved by
    /// −centre (a shift leaves the turns as they are), so no large
    /// coordinates enter a pose however far the ring lies from the origin.
    fn centred_motion(&self) -> crate::solid_sweep_trim::TwoTurnMotion {
        let mut motion = self.motion;
        motion.points = [motion.points[0].sub(self.centre), motion.points[1].sub(self.centre)];
        motion
    }

    /// The motion in the ring's own units: about its centre, lengths in its
    /// major radius L, time τ = w·t in its fastest rate w (rates divided by
    /// w; the poses are the same). Nothing then overflows or underflows
    /// however large, small, fast or slow the ring and its motion are.
    /// (motion, L, w).
    fn unit_motion(&self) -> (crate::solid_sweep_trim::TwoTurnMotion, f64, f64) {
        let size = self.major;
        let w = self.motion.rates.iter().fold(0.0f64, |m, r| m.max(r.abs()));
        let w = if w > 0.0 && w.is_finite() { w } else { 1.0 };
        let mut motion = self.centred_motion();
        motion.points = [motion.points[0].scale(1.0 / size), motion.points[1].scale(1.0 / size)];
        motion.rates = [motion.rates[0] / w, motion.rates[1] / w];
        (motion, size, w)
    }

    /// f(s) = the distance from q to the core circle placed at time s, its
    /// derivative in s, and the distance ρ of q's preimage from the circle's
    /// axis (None where q lies on that axis, where f has no derivative).
    pub(crate) fn reach_slope(&self, q: Vec3, s: f64) -> Option<(f64, f64, f64)> {
        // In the ring's own units (`unit_motion`), then scaled back.
        let (motion, size, w) = self.unit_motion();
        let (d, slope, rho) = self.unit_reach_slope(&motion, q.sub(self.centre).scale(1.0 / size), w * s)?;
        let out = (d * size, slope * size * w, rho * size);
        (out.0.is_finite() && out.1.is_finite() && out.2.is_finite()).then_some(out)
    }

    /// [`Self::reach_slope`] in the ring's own units: `motion` from
    /// [`Self::unit_motion`], `q` about the centre in L, at τ.
    fn unit_reach_slope(&self, motion: &crate::solid_sweep_trim::TwoTurnMotion, q: Vec3, tau: f64) -> Option<(f64, f64, f64)> {
        let pose = motion.pose(tau);
        let local = pose.invert(q);
        let up = local.dot(self.axis);
        let flat = local.sub(self.axis.scale(up));
        let rho = flat.length();
        if !(rho > 0.0 && rho.is_finite()) {
            return None;
        }
        // From the circle's nearest point, relative to its centre (no large
        // coordinates cancel): up along the axis, ρ − 1 out along `flat`.
        let away = self.axis.scale(up).add(flat.scale((rho - 1.0) / rho));
        let distance = (rho - 1.0).hypot(up);
        if !(distance > 0.0 && distance.is_finite()) {
            return None;
        }
        // The preimage's velocity: x' = −Rᵀ v, v the material velocity at q,
        // turned back directly (q − v would lose a velocity below q's ulp).
        let v = motion.velocity(tau, q);
        let dx = pose.unrotate(v).scale(-1.0);
        let slope = away.dot(dx) / distance;
        slope.is_finite().then_some((distance, slope, rho))
    }

    /// An upper bound (its rounding added; see the module docs) on the sheet
    /// Σ + `sheet`·r·N's fold measure at (t, ψ): the larger of
    /// `sheet`·r·κ over Σ's principal curvatures κ (signed against N, the
    /// frame's normal Σ_t × Σ_ψ); the sheet folds where it reaches 1. Σ_ψ and
    /// Σ_ψψ are the circle's own turned, Σ_t the motion's velocity, Σ_tψ its
    /// change along Σ_ψ (the velocity field is affine in the point, so exact
    /// up to rounding), Σ_tt the material point's exact acceleration. Infinite
    /// where Σ is singular, outside the envelope, or where rounding could
    /// move the measure past 1e-6.
    pub(crate) fn fold_ratio_signed(&self, t: f64, psi: f64, sheet: f64) -> f64 {
        if !(self.in_envelope() && t.is_finite() && psi.is_finite()) {
            return f64::INFINITY;
        }
        let (s, c) = psi.sin_cos();
        // In the ring's own units (`unit_motion`): lengths in L, time in
        // τ = w·t; the measure r·κ does not depend on either.
        let (motion, size, w) = self.unit_motion();
        let local = self.x.scale(c).add(self.y.scale(s));
        let tangent = self.x.scale(-s).add(self.y.scale(c));
        let bent = self.x.scale(-c).add(self.y.scale(-s));
        let tau = w * t;
        // At times so large the poses' angles round past 1e-6, no measure.
        let [r0, r1] = motion.rates;
        if !(tau.is_finite() && 64.0 * f64::EPSILON * (r0.abs() + r1.abs()) * tau.abs() <= 1e-6) {
            return f64::INFINITY;
        }
        let pose = motion.pose(tau);
        let point = pose.apply(local);
        let sp = pose.rotate(tangent);
        let spp = pose.rotate(bent);
        let st = motion.velocity(tau, point);
        let stp = motion.velocity(tau, point.add(sp)).sub(st);
        // The material point's acceleration, exactly: with y = T1(z) and z
        // turned about the first axis, v = ω1×(y − p1) + R1(ω0×(z − p0)), so
        // a = ω1×v + ω1×R1(u) + R1(ω0×u), u = ω0×(z − p0). No differences, so
        // nothing cancels however the turns combine.
        let stt = {
            let (w0, w1) = (motion.directions[0].scale(motion.rates[0]), motion.directions[1].scale(motion.rates[1]));
            let second = crate::solid_sweep_trim::Rigid::turn(motion.points[1], motion.directions[1], motion.rates[1] * tau);
            let z = second.invert(point);
            let u = w0.cross(z.sub(motion.points[0]));
            w1.cross(st).add(w1.cross(second.rotate(u))).add(second.rotate(w0.cross(u)))
        };
        let normal = st.cross(sp);
        let length = normal.length();
        if !(length > 0.0 && length.is_finite()) {
            return f64::INFINITY;
        }
        let n = normal.scale(1.0 / length);
        let (e, f, g) = (st.dot(st), st.dot(sp), sp.dot(sp));
        let (l, m, nn) = (stt.dot(n), stp.dot(n), spp.dot(n));
        // The rounding the time derivatives carry: their terms are as large
        // as the turns' rates times the arms, however much they cancel (two
        // nearly opposite turns move the ring slowly from fast parts). Where
        // that rounding could move the measure past 1e-6, no measure.
        // The first form's determinant is |Σ_t × Σ_ψ|² (Lagrange), read from
        // the cross product: e·g − f² would cancel where the two lie nearly
        // parallel. The normal carries the cross product's rounding over its
        // length; each second-form entry its own rounding and the normal's.
        let rates = motion.rates[0].abs() + motion.rates[1].abs();
        let arm = 1.0 + point.length() + motion.points[0].length() + motion.points[1].length();
        let (dv, da) = (64.0 * f64::EPSILON * rates * arm, 64.0 * f64::EPSILON * rates * rates * arm);
        let (vt, vp) = (e.sqrt(), g.sqrt());
        let dn = (dv * vp + 8.0 * f64::EPSILON * vt * vp) / length;
        let near = dn + 4.0 * f64::EPSILON;
        let (dl, dm, dnn) = (da + near * stt.length(), 2.0 * dv + near * stp.length(), near * spp.length());
        let det = length * length;
        // The curvatures are the eigenvalues of the symmetric L⁻¹·II·L⁻ᵀ, L
        // the first form's Cholesky factor; by Weyl they move by at most the
        // norm of L⁻¹·δII·L⁻ᵀ, bounded entrywise in absolute values.
        let (la, lb, lc) = (vt, f / vt, length / vt);
        let (i00, i10, i11) = (1.0 / la, (lb / (la * lc)).abs(), 1.0 / lc);
        let m00 = i00 * i00 * dl;
        let m01 = i00 * (i10 * dl + i11 * dm);
        let m11 = i10 * i10 * dl + 2.0 * i10 * i11 * dm + i11 * i11 * dnn;
        let moved = (m00 * m00 + 2.0 * m01 * m01 + m11 * m11).sqrt();
        let r_unit = self.minor / size;
        if !(dn <= 1e-8 && r_unit * moved <= 1e-6) {
            return f64::INFINITY;
        }
        if !(det > 0.0 && det.is_finite()) {
            return f64::INFINITY;
        }
        // The principal curvatures: the eigenvalues of the symmetric
        // A = L⁻¹·II·L⁻ᵀ (L the first form's Cholesky factor, as above), read
        // as its mean ± hypot(half difference, off-diagonal), which no
        // cancellation erases (mean² − gauss would).
        let a00 = l / (la * la);
        let a01 = (m - lb * a00 * la) / (la * lc);
        let a11 = (nn - 2.0 * lb * (m / la) + lb * lb * a00) / (lc * lc);
        let centre = 0.5 * (a00 + a11);
        let spread = (0.5 * (a00 - a11)).hypot(a01);
        let (k1, k2) = (centre - spread, centre + spread);
        // And A's own rounding, a few ulps of its entries.
        let moved = moved + 16.0 * f64::EPSILON * (a00.abs() + 2.0 * a01.abs() + a11.abs() + (lb * lb / (lc * lc)) * a00.abs() + (lb / lc).abs() * (a01.abs() + m.abs() / (la * lc)));
        let r = r_unit;
        let measure = (sheet * r * k1).max(sheet * r * k2);
        // One-sided: the shape operator's rounding (Weyl, above) and the
        // poses' angle rounding (a relative shift of the geometry of
        // 64·ε·rate·τ, carried into r·κ through its arms) added, so the value
        // returned is an upper bound.
        let angles = 64.0 * f64::EPSILON * (rates * tau.abs() + 1.0);
        let bound = r * moved + 16.0 * angles * arm * (measure.abs() + r);
        let upper = measure + bound;
        // Anything unread (an overflow, a NaN) is no measure: a fold.
        if upper.is_finite() { upper } else { f64::INFINITY }
    }

    /// The bound on |x'| at q: the motion's speed bound (a rotation keeps
    /// lengths, so the preimage moves as fast as the material point at q).
    fn preimage_speed(&self, q: Vec3) -> f64 {
        let [r0, r1] = self.motion.rates;
        let (p0, p1) = (self.motion.points[0], self.motion.points[1]);
        ((r0.abs() + r1.abs()) * q.sub(p1).length() + r0.abs() * p1.sub(p0).length()) * (1.0 + 1e-12)
    }

    /// Whether the ring holds q closer than r − `tie` at a time within
    /// `window` of `own` (both ways, within the motion's range): see the
    /// module docs.
    pub(crate) fn own_read(&self, q: Vec3, own: f64, window: f64, tie: f64) -> OwnRead {
        // Outside the envelope (or a point far off the ring, or a window finer
        // than its times resolve), a tie before any arithmetic.
        // (the window's floor in the motion's own time, τ = w·t)
        // (as `unit_motion`: a ring at rest reads in its own time unchanged)
        let w = self.motion.rates.iter().fold(0.0f64, |m, r| m.max(r.abs()));
        let w = if w > 0.0 && w.is_finite() { w } else { 1.0 };
        let resolve = 1e-9 * (1.0 + (w * own).abs());
        let own_time = own;
        // An own time outside the motion's range has no poses to read from.
        if !(self.in_envelope() && tie > 0.0 && w * window >= resolve && q.sub(self.centre).length() <= ENVELOPE_ARM * self.major && self.from <= own && own <= self.to) {
            return OwnRead::Tie;
        }
        // A window the times cannot hold (own ± window rounds to own inside
        // the range) is read nowhere: a tie, never a clear.
        if (own < self.to && own + window == own) || (own > self.from && own - window == own) {
            return OwnRead::Tie;
        }
        // All of it in the ring's own units (`unit_motion`): lengths in L,
        // time in τ = w·t, so no bound or read under- or overflows.
        let (motion, size, w) = self.unit_motion();
        let qn = q.sub(self.centre).scale(1.0 / size);
        let (r, tie) = (self.minor / size, tie / size);
        let (own, window, from, to) = (w * own, w * window, w * self.from, w * self.to);
        let bend = crate::solid_sweep_trim::turns_bend(&motion, qn);
        let [r0, r1] = motion.rates;
        let (p0, p1) = (motion.points[0], motion.points[1]);
        let speed = ((r0.abs() + r1.abs()) * qn.sub(p1).length() + r0.abs() * p1.sub(p0).length()) * (1.0 + 1e-12);
        if !(bend.is_finite() && speed.is_finite() && r.is_finite() && tie > 0.0 && window > 0.0 && [own, window, from, to].iter().all(|x| x.is_finite())) {
            return OwnRead::Tie;
        }
        // The rounding of a reach read: of the ring's size, the point's and the
        // turning axes' distances from its centre (a pose composed about a far
        // axis that turns rounds by its distance), so a witness and a clear
        // are shown beyond it.
        let arms = (if r0 != 0.0 { p0.length() } else { 0.0 }) + (if r1 != 0.0 { p1.length() } else { 0.0 });
        // And of the poses' angles, rate·τ rounded (large at large times),
        // times the arm they turn the point through.
        let angles = (r0.abs() + r1.abs()) * from.abs().max(to.abs());
        let err = 64.0 * f64::EPSILON * ((1.0 + r + qn.length() + arms) * (1.0 + angles));
        if !err.is_finite() {
            return OwnRead::Tie;
        }
        let mut reads = 0usize;
        let mut tied = false;
        let read = |s: f64, reads: &mut usize| -> Option<(f64, f64, f64)> {
            *reads += 1;
            self.unit_reach_slope(&motion, qn, s)
        };
        // A range that collapses once normalised is read nowhere: a tie.
        if !(to > from) {
            return OwnRead::Tie;
        }
        // The own time itself, read first (so no answer is given unread).
        match read(own, &mut reads) {
            Some(f) if f.0 + err < r - tie => return OwnRead::Covered(own_time),
            Some(_) => {}
            None => return OwnRead::Tie,
        }
        let ranges = [(own, (own + window).min(to)), ((own - window).max(from), own)];
        // Normalised, a window must still be held: an interval that should run
        // (own inside the range on that side) but collapses is read nowhere.
        if (own < to && !(ranges[0].1 > own)) || (own > from && !(own > ranges[1].0)) {
            return OwnRead::Tie;
        }
        for (a0, b0) in ranges {
            if !(b0 > a0) {
                continue;
            }
            let (Some(fa), Some(fb)) = (read(a0, &mut reads), read(b0, &mut reads)) else { return OwnRead::Tie };
            let mut stack = vec![(a0, fa, b0, fb)];
            while let Some((a, fa, b, fb)) = stack.pop() {
                for (s, f) in [(a, fa.0), (b, fb.0)] {
                    if f + err < r - tie {
                        // Back in the caller's time, kept inside the range
                        // (dividing by w may round past an end).
                        return OwnRead::Covered((s / w).clamp(self.from, self.to));
                    }
                }
                let width = b - a;
                let half = 0.5 * width;
                // Lower bounds over the interval on d and ρ (1-Lipschitz in
                // the preimage, which moves no faster than `speed`).
                let d_least = fa.0.min(fb.0) - speed * half;
                let rho_least = fa.2.min(fb.2) - speed * half;
                let lower = if d_least > 0.0 && rho_least > 0.0 {
                    let m2 = bend + speed * speed * (1.0 / d_least).max(1.0 / rho_least);
                    // Each end's concave lower bound, over its own half.
                    let from_a = fa.0 + fa.1 * half - 0.5 * m2 * half * half;
                    let from_b = fb.0 - fb.1 * half - 0.5 * m2 * half * half;
                    // Any part unread (an overflow, a NaN that `min` would
                    // pass over) certifies nothing.
                    if [m2, from_a, from_b, fa.0, fb.0].iter().all(|x| x.is_finite()) { fa.0.min(fb.0).min(from_a).min(from_b) } else { f64::NEG_INFINITY }
                } else {
                    f64::NEG_INFINITY
                };
                if lower - err >= r - tie {
                    continue;
                }
                if reads >= OWN_BUDGET || !(width > 1e-13 * (to - from)) {
                    tied = true;
                    continue;
                }
                let m = a + half;
                let Some(fm) = read(m, &mut reads) else { return OwnRead::Tie };
                stack.push((m, fm, b, fb));
                stack.push((a, fa, m, fm));
            }
        }
        if tied { OwnRead::Tie } else { OwnRead::Clear }
    }
}

