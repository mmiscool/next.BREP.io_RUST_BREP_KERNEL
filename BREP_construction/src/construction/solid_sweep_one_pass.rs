//! A ball screwed about an axis passes over each point at most once: a proof
//! from its centre's helix alone, in place of the sampled one-pass walk
//! (`BREP_SWEEP_BALL_ONE_PASS`, default on; `=0` walks).
//!
//! A ball of radius r under a rigid motion covers at time t exactly the points
//! within r of its centre γ(t); its sweep folds (passes over a point twice)
//! exactly where some point y has f(t) = |y − γ(t)|² < r² on two separate
//! intervals of time. Extend the helix past both ends of the motion (an
//! advance makes f grow without bound either way): each component of
//! {f < r²} is then bounded and holds a minimum of f, where y − γ(t) is square
//! to the helix, so y lies in the normal disc of radius r at that t. Two
//! components give two such discs through y. So the sweep is one pass where
//! the normal discs of radius r about the whole helix are disjoint, that is
//! where r is below the helix's thickness Δ = min(ρ, ½·dcsd): ρ its radius of
//! curvature (R² + k²)/R, dcsd the least distance between two of its points
//! whose chord is square to both tangents (Gonzalez & Maddocks; a helix of
//! radius R rising k a radian).
//!
//! By the screw symmetry every pair is congruent to (0, s), with chord length
//! d(s)² = 2R²(1 − cos s) + k²s², and the chord is square to both tangents
//! exactly where R² sin s + k² s = 0, which has no root in (0, π] (both terms
//! positive). So ½·dcsd ≥ ½·min over s ≥ π of d(s), which is bounded below
//! piecewise: on [a, b] ⊂ [π, 2π], 1 − cos s falls and s² rises, so
//! d² ≥ 2R²(1 − cos b) + k²a²; past 2π, d ≥ |k|·s ≥ 2π|k|.

use crate::solid_sweep::OutlinePiece;
use crate::solid_sweep_caps::{Screw, SolidTool};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// The switch (default on; `BREP_SWEEP_BALL_ONE_PASS=0` walks every ball sweep).
pub(crate) fn ball_one_pass_on() -> bool {
    crate::switch!("BREP_SWEEP_BALL_ONE_PASS", true)
}

/// The ball a solid of revolution is (its meridian a half disc on its axis:
/// one semicircle about a point of the axis and the axis segment between its
/// ends), as its centre and radius.
fn ball_of(tool: &SolidTool) -> Option<(crate::Vec3, f64)> {
    let SolidTool::Revolved { axis_origin, axis_direction, meridian, .. } = tool else { return None };
    ball_of_revolved(*axis_origin, *axis_direction, meridian)
}

/// [`ball_of`] for a solid of revolution given by its axis and meridian.
pub(crate) fn ball_of_revolved(axis_origin: crate::Vec3, axis_direction: crate::Vec3, meridian: &[OutlinePiece]) -> Option<(crate::Vec3, f64)> {
    let w = axis_direction.normalized().ok()?;
    let [first, second] = meridian else { return None };
    let (arc, line) = match (first, second) {
        (arc @ OutlinePiece::Arc { .. }, line @ OutlinePiece::Line { .. }) | (line @ OutlinePiece::Line { .. }, arc @ OutlinePiece::Arc { .. }) => (arc, line),
        _ => return None,
    };
    let (OutlinePiece::Arc { centre, radius, start, end }, OutlinePiece::Line { from, to }) = (*arc, *line) else { return None };
    if !(radius > 0.0 && radius.is_finite()) {
        return None;
    }
    let hair = 1e-12 * radius;
    let on = |a: f64, b: f64| (a - b).abs() <= 1e-12;
    // A semicircle on the axis's positive side: from −π/2 to π/2 either way.
    let swept = (on(start, -FRAC_PI_2) && on(end, FRAC_PI_2)) || (on(start, FRAC_PI_2) && on(end, -FRAC_PI_2));
    if !(swept && centre[0].abs() <= hair) {
        return None;
    }
    // The line joins its two ends along the axis.
    let (top, bottom) = ([0.0, centre[1] + radius], [0.0, centre[1] - radius]);
    let at = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).abs() <= hair && (p[1] - q[1]).abs() <= hair;
    if !((at(from, top) && at(to, bottom)) || (at(from, bottom) && at(to, top))) {
        return None;
    }
    Some((axis_origin.add(w.scale(centre[1])), radius))
}

/// `BREP_SWEEP_BALL_COVER` (default on; `=0` walks): the one-pass check's
/// question for a ball carried by a frame along a cubic, answered from the
/// cubic itself.
pub(crate) fn ball_cover_on() -> bool {
    crate::switch!("BREP_SWEEP_BALL_COVER", true)
}

/// `BREP_SWEEP_BALL_COVER_CHECK` (default off): every ball-on-cubic answer is
/// walked as well and the two compared (this thread's
/// [`ball_cover_check_take`]); the walk's answer is kept.
pub(crate) fn ball_cover_check_on() -> bool {
    crate::switch!("BREP_SWEEP_BALL_COVER_CHECK", false)
}

thread_local! {
    static BALL_COVER_CHECK: std::cell::Cell<(usize, usize)> = const { std::cell::Cell::new((0, 0)) };
}

/// Record one check: the two answers agreed or not.
pub(crate) fn ball_cover_check_record(agree: bool) {
    BALL_COVER_CHECK.with(|c| {
        let (n, bad) = c.get();
        c.set((n + 1, bad + (!agree) as usize));
    });
}

/// This thread's (answers compared, disagreements), reset.
#[allow(dead_code)]
pub(crate) fn ball_cover_check_take() -> (usize, usize) {
    BALL_COVER_CHECK.with(|c| c.replace((0, 0)))
}

/// Is `y` ever deeper than `margin` inside a ball of radius `r` whose centre
/// stands within `offset` of the cubic Bézier `control` at each time of
/// `[t0, t1]` (a ball carried by a frame riding the cubic, its centre `offset`
/// from the frame's origin)? With h(s) = |y − P(s)|²: some s with
/// √h + offset < r − margin answers yes; h ≥ (r − margin + offset)² throughout
/// answers no. Each piece [a, b] of the span holds h above its ends' lesser
/// value less M·(b − a)²/8, M a bound on |h''| = 2|P'|² + 2(P − y)·P''
/// (|P'|, |P''| by their control polygons, |P − y| by its value at a plus
/// |P'|·(b − a)); pieces that this does not clear are halved. `None` where it
/// stays undecided (the caller walks).
pub(crate) fn ball_on_cubic_covered(control: [crate::Vec3; 4], t0: f64, t1: f64, offset: f64, r: f64, y: crate::Vec3, margin: f64) -> Option<bool> {
    let [p0, p1, p2, p3] = control;
    let at = |s: f64| {
        let q = 1.0 - s;
        p0.scale(q * q * q).add(p1.scale(3.0 * q * q * s)).add(p2.scale(3.0 * q * s * s)).add(p3.scale(s * s * s))
    };
    let first = [p1.sub(p0), p2.sub(p1), p3.sub(p2)].iter().map(|d| 3.0 * d.length()).fold(0.0f64, f64::max);
    let second = [p2.sub(p1.scale(2.0)).add(p0), p3.sub(p2.scale(2.0)).add(p1)].iter().map(|d| 6.0 * d.length()).fold(0.0f64, f64::max);
    let deep = r - margin - offset;
    let clear = r - margin + offset;
    if !(t1 > t0 && clear > 0.0 && first.is_finite() && second.is_finite()) {
        return None;
    }
    let (deep_sq, clear_sq) = (if deep > 0.0 { deep * deep } else { f64::NEG_INFINITY }, clear * clear);
    let h = |s: f64| at(s).sub(y).length_squared();
    let mut stack = vec![(t0, t1, h(t0), h(t1))];
    let mut pieces = 0usize;
    while let Some((a, b, ha, hb)) = stack.pop() {
        if ha < deep_sq || hb < deep_sq {
            return Some(true);
        }
        let w = b - a;
        let reach = ha.max(0.0).sqrt() + first * w;
        let bound = 2.0 * (first * first + reach * second);
        if ha.min(hb) - bound * w * w / 8.0 >= clear_sq {
            continue;
        }
        pieces += 1;
        if pieces > 20_000 || w <= 1e-15 * (t1 - t0) {
            return None;
        }
        let m = 0.5 * (a + b);
        let hm = h(m);
        stack.push((m, b, hm, hb));
        stack.push((a, m, ha, hm));
    }
    Some(false)
}

/// Whether the ball `tool` under `screw` is proved to pass over each point at
/// most once (so the sampled one-pass check has nothing to find). `false` for
/// any other tool or motion, or where the proof does not reach.
pub(crate) fn ball_screw_one_pass(screw: &Screw, tool: &SolidTool) -> bool {
    if !ball_one_pass_on() {
        return false;
    }
    // One plain screw: a turn with a steady advance along its axis.
    if screw.frame.is_some() || screw.turns.is_some() || screw.c2 != 0.0 || screw.omega == 0.0 || screw.c == 0.0 {
        return false;
    }
    let (Some((centre, r)), Ok(e)) = (ball_of(tool), screw.e.normalized()) else { return false };
    let offset = centre.sub(screw.a);
    let radial = offset.sub(e.scale(offset.dot(e))).length();
    let k = (screw.c / screw.omega).abs();
    if !(radial.is_finite() && k.is_finite() && k > 0.0) {
        return false;
    }
    let margin = 1.0 - 1e-9;
    // Exactly on the axis the centre moves along a line: every point is passed
    // once. Off it, however near, the bounds below decide (a ball far wider
    // than a tiny helix comes back over its own points each turn).
    if radial == 0.0 {
        return true;
    }
    let curvature = (radial * radial + k * k) / radial;
    if !(r < margin * curvature) {
        return false;
    }
    let pieces = 4096;
    let mut least = TAU * k;
    let mut least_sq = least * least;
    for j in 0..pieces {
        let a = PI + PI * j as f64 / pieces as f64;
        let b = PI + PI * (j + 1) as f64 / pieces as f64;
        let bound = 2.0 * radial * radial * (1.0 - b.cos()) + k * k * a * a;
        least_sq = least_sq.min(bound);
    }
    least = least.min(least_sq.max(0.0).sqrt());
    r < margin * 0.5 * least
}

