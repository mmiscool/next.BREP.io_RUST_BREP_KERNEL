//! One pcurve fitter for a curve that lies on a surface, verified out of sample.
//!
//! # Why a fit through samples of the track is not enough
//!
//! A curve lying on a surface has an image in the surface's parameter space,
//! its TRACK, and a pcurve is a cubic fitted to that track. The track is only
//! as smooth as the surface's parameterisation. A rational-quadratic
//! revolution — every sphere, cylinder and torus `make_revolution` builds —
//! carries double knots at its quarter points, where the parameterisation is
//! C1 and not C2. A smooth 3D arc crossing one of those knot lines has a track
//! whose tangent is continuous and whose second derivative JUMPS there, and a
//! single cubic through uniform samples across that jump converges as O(h²),
//! not O(h⁴). Measured 2026-09-17 on the 10-cube star fillet's corner patch:
//! uniform doubling first reaches the 1e-7 floor at 2048 samples and still
//! misses it at r = 9; broken at the crossing it reaches the floor at 129.
//! That was the whole of the patch's closure residual, 5.3e-7·r², and a
//! +1.09e-4 volume error at r = 9 that `fillet_edges` accepted.
//!
//! So the fit BREAKS at every knot-line crossing — the crossing is inserted as
//! an exact sample, the pieces either side are fitted separately, and they are
//! joined with the EXACT track tangent at the crossing prescribed on both
//! sides, so the pcurve is C1 there as the track is. Joined without it, the
//! two pieces kink by 2.1e-6 rad on the rung that reaches the floor, which an
//! offset or an imprint would read as a real corner.
//!
//! # Why the check is at midpoints
//!
//! Every rung is measured at the midpoints between the dense track samples,
//! each projected on its own. Measured at the samples a rung interpolates, the
//! miss reads zero by construction whatever lies between (the corner patch's
//! 33-sample pcurves read 1e-16 there and 1.9e-5 in uv between). The measure
//! is 3D — the pcurve pushed through the surface against the curve's own point
//! on the carrier — so it needs no period unwrap.
//!
//! The fitter REPORTS: the fit, the samples on the rung it stopped at, and the
//! measured miss. The caller decides what a miss over the floor means, and says
//! so where it decides (see [`TrackFit::on_floor`]).

use crate::{KernelRefusal, KernelStage, NurbsCurve, NurbsSurface, OrRefuse, Vec3, Vec4};

use super::stations::FIT_DEGREE;

/// The refusal for a pcurve whose top rung still misses the refinement floor
/// out of sample.
pub(crate) const PCURVE_OFF_FLOOR: &str =
    "blend: a pcurve does not reach the refinement floor out of sample —";

/// Fewest track intervals a piece between two breaks spans; a shorter piece is
/// densified to this many.
const MIN_PIECE_INTERVALS: usize = 6;

/// Breaks closer than this (as a curve fraction) to an end or to each other are
/// merged into it.
const MERGE: f64 = 1e-6;

/// A curve's track on a surface: dense samples of the curve, each inverted
/// onto the surface, unwrapped across closed directions, with every crossing of
/// a knot line the parameterisation is not C2 across inserted as an exact
/// sample and recorded as a break.
#[derive(Clone)]
pub(in crate::blend) struct Track {
    /// Curve fractions in `[0, 1]`, strictly increasing.
    pub(in crate::blend) fractions: Vec<f64>,
    /// The track, unwrapped: consecutive samples never jump a period.
    pub(in crate::blend) uv: Vec<[f64; 2]>,
    /// The raw inversion at each sample — the seed its neighbours project from.
    pub(in crate::blend) feet: Vec<[f64; 2]>,
    /// Interior sample indices the fit breaks at.
    pub(in crate::blend) breaks: Vec<usize>,
}

/// A fitted pcurve and what it measured.
pub(in crate::blend) struct TrackFit {
    /// The pcurve, parameterised on the curve fraction over `[0, 1]`.
    pub(in crate::blend) curve: NurbsCurve,
    /// The largest 3D distance, at the dense midpoints, between the pcurve
    /// pushed through the surface and the curve's own point on it.
    pub(in crate::blend) miss: f64,
    /// Track samples the returned rung interpolates.
    pub(in crate::blend) samples: usize,
    /// Whether `miss` is within the tolerance the fit was asked for. A caller
    /// that uses a fit with this false states, where it does so, the bar it
    /// accepts it under and why.
    pub(in crate::blend) on_floor: bool,
    /// `BREP_DEBUG_TRACK_FIT` only (None otherwise): where the returned rung's
    /// miss sits — the curve fraction of the worst dense midpoint and the dense
    /// track interval bracketing it. Diagnostic; nothing reads it to decide.
    pub(in crate::blend) worst: Option<WorstCheck>,
    /// The dense intervals whose out-of-sample midpoint check exceeds the
    /// tolerance on the returned rung: what local refinement inserts into.
    pub(in crate::blend) over: Vec<usize>,
    /// The rung: how many track sections it interpolates (the whole track's
    /// intervals at the top rung).
    pub(in crate::blend) sections: usize,
    /// The track fractions of the samples the rung interpolates (ends and
    /// breaks included), so a later rung can keep exactly them.
    pub(in crate::blend) kept: Vec<f64>,
}

/// Provenance of a track fit's worst dense-midpoint miss (diagnostic only).
#[derive(Clone, Copy, Debug)]
pub(in crate::blend) struct WorstCheck {
    pub(in crate::blend) fraction: f64,
    pub(in crate::blend) interval: usize,
    pub(in crate::blend) bracket: [f64; 2],
}

/// Sample `at` over `[0, 1]` at `dense` uniform intervals onto `surface`, each
/// inversion seeded from its neighbour's foot, unwrapped across the closed
/// directions, with the knot-line crossings inserted
/// ([`insert_knot_crossings`]).
pub(in crate::blend) fn project_track(
    surface: &NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    dense: usize,
    curve_breaks: &[f64],
) -> Result<Track, KernelRefusal> {
    let dense = dense.max(2 * MIN_PIECE_INTERVALS);
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain")?;
    let periods = [
        closed_u.then_some(u1 - u0),
        closed_v.then_some(v1 - v0),
    ];
    let mut track = Track {
        fractions: Vec::with_capacity(dense + 1),
        uv: Vec::with_capacity(dense + 1),
        feet: Vec::with_capacity(dense + 1),
        breaks: Vec::new(),
    };
    for index in 0..=dense {
        let fraction = index as f64 / dense as f64;
        let point = at(fraction).or_refuse(KernelStage::Refine, "at")?;
        let raw = foot(surface, point, track.feet.last().copied())?;
        let mut uv = raw;
        if let Some(previous) = track.uv.last() {
            unwrap_near(&mut uv, previous, periods);
        }
        track.fractions.push(fraction);
        track.uv.push(uv);
        track.feet.push(raw);
    }
    insert_knot_crossings(surface, at, &mut track, curve_breaks)?;
    Ok(track)
}

/// Refine a surface foot of `point` by Newton on the tangency conditions until
/// its step vanishes. The kernel's seeded projector stops as soon as the foot is
/// within `LINEAR_TOLERANCE` (1e-7) of the point, which on a curve lying on the
/// surface leaves the foot up to that far along the surface: a track built from
/// such feet jitters at 1e-7, and a fit verified against them cannot be read at
/// a 1e-7 floor (measured 2026-09-17 on the revolution × NURBS rim: the ladder
/// stalled at 1.6e-7–3.1e-7 however dense). An analytic carrier's projection is
/// closed-form and exact, so it is returned as it is.
fn polish(surface: &NurbsSurface, point: Vec3, uv: [f64; 2]) -> Result<[f64; 2], KernelRefusal> {
    if surface.analytic().is_some() {
        return Ok(uv);
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain")?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let inside = |value: f64, low: f64, high: f64, closed: bool| -> Option<f64> {
        if value >= low && value <= high {
            Some(value)
        } else if closed {
            Some(low + (value - low).rem_euclid(high - low))
        } else {
            None
        }
    };
    let mut current = uv;
    for _ in 0..8 {
        let d = surface.derivatives_small(current[0], current[1], 2).or_refuse(KernelStage::Refine, "derivatives")?;
        let residual = d[0][0].sub(point);
        let (su, sv) = (d[1][0], d[0][1]);
        let (f, g) = (su.dot(residual), sv.dot(residual));
        let j00 = d[2][0].dot(residual) + su.length_squared();
        let j01 = d[1][1].dot(residual) + su.dot(sv);
        let j11 = d[0][2].dot(residual) + sv.length_squared();
        let determinant = j00 * j11 - j01 * j01;
        if !(determinant.abs() > 1e-300) {
            break;
        }
        let du = (-f * j11 + g * j01) / determinant;
        let dv = (-g * j00 + f * j01) / determinant;
        // A polish refines a foot; a large step is a different foot.
        if !(du.abs() <= 1e-3 * (u1 - u0) && dv.abs() <= 1e-3 * (v1 - v0)) {
            break;
        }
        let (Some(u), Some(v)) = (
            inside(current[0] + du, u0, u1, closed_u),
            inside(current[1] + dv, v0, v1, closed_v),
        ) else {
            break;
        };
        current = [u, v];
        if du.abs() <= 1e-15 * (u1 - u0) && dv.abs() <= 1e-15 * (v1 - v0) {
            break;
        }
    }
    Ok(current)
}

/// `uv` with each CLOSED direction of `surface` wrapped into its domain; an
/// open direction, and a coordinate already inside, unchanged.
fn wrap_closed(surface: &NurbsSurface, uv: [f64; 2]) -> Result<[f64; 2], KernelRefusal> {
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    if uv[0] >= u0 && uv[0] <= u1 && uv[1] >= v0 && uv[1] <= v1 {
        return Ok(uv);
    }
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let wrap = |value: f64, closed: bool, low: f64, high: f64| {
        if closed && (value < low || value > high) && high > low {
            low + (value - low).rem_euclid(high - low)
        } else {
            value
        }
    };
    Ok([wrap(uv[0], closed_u, u0, u1), wrap(uv[1], closed_v, v0, v1)])
}

/// The foot of `point` seeded from `seed` (or the nearest point without one),
/// polished ([`polish`]).
///
/// A seed past a CLOSED direction's domain (an unwrapped track's uv, or a
/// shipped pcurve's after its period shift) is wrapped into the domain first,
/// as `pcurve_image` wraps an image: the seeded projector does not, and a
/// closed rim's pcurve read at u = 1.5 of [0, 1] started its Newton half a
/// period from the foot and stood 2.139 off its rail (the mixed analytic x
/// NURBS rim). A seed inside the domain is passed as before, bit for bit.
fn foot(surface: &NurbsSurface, point: Vec3, seed: Option<[f64; 2]>) -> Result<[f64; 2], KernelRefusal> {
    let projected = match seed {
        None => crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project")?,
        Some(seed) => crate::projection::project_point_to_surface_from_seed(surface, point, wrap_closed(surface, seed)?)
            .or_refuse(KernelStage::Refine, "project")?,
    };
    polish(surface, point, [projected.u, projected.v])
}

/// The foot of `point` NEAREST it among the feet sought from each of `seeds`
/// ([`foot`]), the first seed first. The seeded projector is local and
/// returns a stalled Newton's iterate without saying so — seeded at u = 0.1
/// for a foot at 0.316 on a carrier with a stationary edge it returns the seed
/// itself — and a seed ON a stationary edge (S_u = 0) cannot leave it, so no
/// single seed is trusted: a point on its carrier has its true foot nearer
/// than any stall. A foot on the carrier to rounding ends the search.
fn foot_among(surface: &NurbsSurface, point: Vec3, seeds: &[[f64; 2]]) -> Result<[f64; 2], KernelRefusal> {
    let on_carrier = 1e-12 * (1.0 + point.length());
    let mut best: Option<([f64; 2], f64)> = None;
    for (index, seed) in seeds.iter().enumerate() {
        if seeds[..index].contains(seed) {
            continue;
        }
        let found = foot(surface, point, Some(*seed))?;
        let distance = surface.evaluate_extended(found[0], found[1]).or_refuse(KernelStage::Refine, "evaluate_extended")?.sub(point).length();
        if best.map_or(true, |(_, known)| distance < known) {
            best = Some((found, distance));
        }
        if distance <= on_carrier {
            break;
        }
    }
    best.map(|(found, _)| found).ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "foot_seeds", "blend: a foot with no seed"))
}

/// Polish the interior feet of a track a caller built with the kernel's
/// projector, moving each unwrapped sample by its foot's own change. The two
/// ends are left as the caller set them (a closed chain pins a crossing end
/// onto the seam).
pub(in crate::blend) fn polish_track(
    surface: &NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    track: &mut Track,
) -> Result<(), KernelRefusal> {
    let last = track.fractions.len() - 1;
    for index in 1..last {
        let raw = track.feet[index];
        let polished = polish(surface, at(track.fractions[index]).or_refuse(KernelStage::Refine, "at")?, raw)?;
        let (du, dv) = (polished[0] - raw[0], polished[1] - raw[1]);
        // A wrap inside the polish moved the foot a period; the track does not.
        if du.abs() > 0.25 || dv.abs() > 0.25 {
            continue;
        }
        track.feet[index] = polished;
        track.uv[index] = [track.uv[index][0] + du, track.uv[index][1] + dv];
    }
    Ok(())
}

/// The 3D image of a pcurve point `uv` on `surface`, its CLOSED directions
/// wrapped into the domain first. A track is unwrapped (`unwrap_near`), so a
/// pcurve fitted to one that crosses a closed carrier's seam runs past the
/// domain there, and `NurbsSurface::evaluate` CLAMPS a parameter into the
/// domain: the image would stop at the seam, a gross false miss (the network's
/// refit support trims read 0.25 to 0.41 that way on the 15-degree three-arc
/// row once their rails crossed the turned fat cylinder's seam). A point
/// already inside the domain is evaluated exactly as before, bit for bit;
/// an open direction is evaluated as before (clamped).
fn pcurve_image(surface: &NurbsSurface, u: f64, v: f64) -> Result<Vec3, String> {
    let ([u0, u1], [v0, v1]) = (surface.domain_u()?, surface.domain_v()?);
    if u >= u0 && u <= u1 && v >= v0 && v <= v1 {
        return surface.evaluate(u, v);
    }
    let (closed_u, closed_v) = surface.closed_directions()?;
    let wrap = |value: f64, closed: bool, low: f64, high: f64| {
        if closed && (value < low || value > high) && high > low {
            low + (value - low).rem_euclid(high - low)
        } else {
            value
        }
    };
    surface.evaluate(wrap(u, closed_u, u0, u1), wrap(v, closed_v, v0, v1))
}


/// Move each closed coordinate of `uv` by whole periods to within half a period
/// of `near`.
fn unwrap_near(uv: &mut [f64; 2], near: &[f64; 2], periods: [Option<f64>; 2]) {
    for axis in 0..2 {
        let Some(period) = periods[axis] else {
            continue;
        };
        while uv[axis] - near[axis] > 0.5 * period {
            uv[axis] -= period;
        }
        while near[axis] - uv[axis] > 0.5 * period {
            uv[axis] += period;
        }
    }
}

/// The parameter lines of `surface`, per direction, across which it is not C2:
/// interior knots of multiplicity at least the degree, and — in a closed
/// direction — the seam, where the parameterisation wraps.
fn knot_lines(surface: &NurbsSurface) -> Result<[Vec<f64>; 2], KernelRefusal> {
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let lines = |knots: &[f64], degree: usize, closed: bool| -> Vec<f64> {
        let (low, high) = (knots[0], knots[knots.len() - 1]);
        let mut lines = Vec::new();
        let mut index = 0;
        while index < knots.len() {
            let value = knots[index];
            let run = knots[index..].iter().take_while(|knot| **knot == value).count();
            if value > low && value < high && run >= degree.max(1) {
                lines.push(value);
            }
            index += run;
        }
        if closed {
            lines.push(low);
        }
        lines
    };
    Ok([
        lines(&surface.knots_u, surface.degree_u, closed_u),
        lines(&surface.knots_v, surface.degree_v, closed_v),
    ])
}

/// The fractions, over `[t0, t1]` walked forward or back, where `curve` itself
/// is not C2: interior knots of multiplicity at least its degree. A
/// rational-quadratic arc sweeping past a quarter turn is two segments joined
/// at a double knot, and its track has the same second-derivative jump there a
/// surface knot line gives (measured 2026-09-17 on the very acute corner's
/// patch: every rung's worst miss sat at fraction 0.499 and stalled at 3.2e-7).
pub(in crate::blend) fn curve_breaks(curve: &NurbsCurve, t0: f64, t1: f64, forward: bool) -> Vec<f64> {
    let mut fractions = Vec::new();
    let mut index = 0;
    while index < curve.knots.len() {
        let value = curve.knots[index];
        let run = curve.knots[index..].iter().take_while(|knot| **knot == value).count();
        let (low, high) = (t0.min(t1), t0.max(t1));
        if value > low && value < high && run >= curve.degree.max(1) {
            let along = (value - t0) / (t1 - t0);
            fractions.push(if forward { along } else { 1.0 - along });
        }
        index += run;
    }
    fractions
}

/// Insert every crossing of a knot line into `track` as an exact sample and a
/// break, and mark a sample that already lies ON a line as one. The curve's
/// own breaks (`curve_breaks`, fractions) are inserted and broken at too.
///
/// A crossing is found by sign change between neighbouring samples and bisected
/// on the curve fraction to the parameter's own resolution. Two cases a sign
/// test alone gets wrong are handled here, because both occur on the corner
/// patch: a crossing that lands EXACTLY on a sample (the meridian arc of a
/// symmetric corner meets the equator at fraction 0.5, which an even grid
/// samples), which reads as no sign change on either side; and an END of the
/// curve lying on a line (every corner of the patch sits on one), which reads
/// as a crossing at fraction 1 and must not become a piece of zero length.
pub(in crate::blend) fn insert_knot_crossings(
    surface: &NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    track: &mut Track,
    curve_breaks: &[f64],
) -> Result<(), KernelRefusal> {
    let lines = knot_lines(surface)?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain")?;
    let periods = [
        closed_u.then_some(u1 - u0),
        closed_v.then_some(v1 - v0),
    ];
    // The line images near a coordinate: in a closed direction every whole
    // period of each line, since the unwrapped track may run outside the domain.
    let images = |axis: usize, low: f64, high: f64| -> Vec<f64> {
        let mut images = Vec::new();
        for &line in &lines[axis] {
            match periods[axis] {
                None => images.push(line),
                Some(period) => {
                    let first = ((low - line) / period).floor() as i64 - 1;
                    let last = ((high - line) / period).ceil() as i64 + 1;
                    for k in first..=last {
                        images.push(line + k as f64 * period);
                    }
                }
            }
        }
        images
    };
    let resolution = 1e-12;
    // (fraction, uv, foot) to insert, and indices already on a line.
    let mut inserts: Vec<(usize, f64, [f64; 2], [f64; 2])> = Vec::new();
    let mut on_line: Vec<usize> = Vec::new();
    let last = track.fractions.len() - 1;
    for index in 0..last {
        for axis in 0..2 {
            let (a, b) = (track.uv[index][axis], track.uv[index + 1][axis]);
            for line in images(axis, a.min(b), a.max(b)) {
                let (da, db) = (a - line, b - line);
                if da.abs() <= resolution {
                    if index > 0 {
                        on_line.push(index);
                    }
                    continue;
                }
                if db.abs() <= resolution || da.signum() == db.signum() {
                    continue;
                }
                // Bisect on the fraction, each probe seeded from the left foot
                // and unwrapped onto the left sample.
                let (mut low, mut high) = (track.fractions[index], track.fractions[index + 1]);
                let mut low_value = da;
                let mut found = (low, track.uv[index], track.feet[index]);
                for _ in 0..80 {
                    if high - low <= 1e-15 {
                        break;
                    }
                    let middle = 0.5 * (low + high);
                    let raw = foot(surface, at(middle).or_refuse(KernelStage::Refine, "at")?, Some(track.feet[index]))?;
                    let mut uv = raw;
                    unwrap_near(&mut uv, &track.uv[index], periods);
                    found = (middle, uv, raw);
                    let value = uv[axis] - line;
                    if value.signum() == low_value.signum() {
                        low = middle;
                        low_value = value;
                    } else {
                        high = middle;
                    }
                }
                inserts.push((index, found.0, found.1, found.2));
            }
        }
    }
    for &fraction in curve_breaks {
        if !(fraction > 0.0 && fraction < 1.0) {
            continue;
        }
        let index = track
            .fractions
            .windows(2)
            .position(|pair| pair[0] <= fraction && fraction <= pair[1])
            .ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "curve_break", "blend: a curve break outside its track"))?;
        let raw = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
        let mut uv = raw;
        unwrap_near(&mut uv, &track.uv[index], periods);
        inserts.push((index, fraction, uv, raw));
    }
    let mut breaks: Vec<f64> = on_line.iter().map(|&index| track.fractions[index]).collect();
    // Insert from the back so earlier indices stay valid.
    inserts.sort_by(|a, b| b.1.total_cmp(&a.1));
    let spacing = 1.0 / last as f64;
    for (index, fraction, uv, foot) in inserts {
        // A crossing within a sliver of either neighbour IS that neighbour.
        let near_left = fraction - track.fractions[index] <= 1e-9 * spacing;
        let near_right = track.fractions[index + 1] - fraction <= 1e-9 * spacing;
        if inserted_at(&track.fractions, fraction) {
            breaks.push(fraction);
            continue;
        }
        if near_left || near_right {
            breaks.push(track.fractions[if near_left { index } else { index + 1 }]);
            continue;
        }
        track.fractions.insert(index + 1, fraction);
        track.uv.insert(index + 1, uv);
        track.feet.insert(index + 1, foot);
        breaks.push(fraction);
    }
    // Breaks by fraction: interior, and apart. A kink within MERGE of an end
    // or of another break is left unbroken — over a stretch that short the
    // second-derivative jump moves the fit by well under 1e-12.
    breaks.sort_by(f64::total_cmp);
    let mut fractions: Vec<f64> = Vec::new();
    for fraction in breaks {
        let previous = fractions.last().copied().unwrap_or(0.0);
        if fraction - previous > MERGE && 1.0 - fraction > MERGE {
            fractions.push(fraction);
        }
    }
    // A piece shorter than the fit needs is DENSIFIED, not dropped: a crossing
    // a fraction of a sample from an end is still a kink (the acute prism's
    // corner arc crosses the equator 0.0031 from its start, and left unbroken
    // there its pcurve stalled at 1.7e-7 on the top rung).
    let mut bounds = vec![0.0];
    bounds.extend(fractions.iter().copied());
    bounds.push(1.0);
    for pair in bounds.windows(2) {
        let (low, high) = (pair[0], pair[1]);
        let inside = track.fractions.iter().filter(|f| **f > low && **f < high).count();
        if inside + 1 >= MIN_PIECE_INTERVALS {
            continue;
        }
        for k in 1..MIN_PIECE_INTERVALS {
            let fraction = low + (high - low) * k as f64 / MIN_PIECE_INTERVALS as f64;
            if inserted_at(&track.fractions, fraction) {
                continue;
            }
            let index = track
                .fractions
                .windows(2)
                .position(|pair| pair[0] < fraction && fraction < pair[1])
                .ok_or_else(|| {
                    KernelRefusal::internal(KernelStage::Refine, "densified_sample", "blend: a densified sample outside its track")
                })?;
            let raw = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
            let mut uv = raw;
            unwrap_near(&mut uv, &track.uv[index], periods);
            track.fractions.insert(index + 1, fraction);
            track.uv.insert(index + 1, uv);
            track.feet.insert(index + 1, raw);
        }
    }
    let kept: Vec<usize> = fractions
        .iter()
        .filter_map(|fraction| track.fractions.iter().position(|f| f == fraction))
        .collect();
    track.breaks = kept;
    Ok(())
}

/// Fit `track` on the ladder: 8 samples, doubled until the pcurve lies within
/// `tolerance` of the curve at every dense midpoint, or until the rung
/// interpolates every dense sample. The rung with the smallest miss is returned
/// with its measure.
///
/// With no breaks, a rung is one cubic interpolant through uniformly spaced
/// samples — exactly the ladder `project_piece_pcurve` climbed before it was
/// hoisted here. With breaks, each piece between them takes a share of the
/// rung's samples in proportion to its length (at least enough for the degree)
/// and is fitted with the exact track tangent prescribed at every break it
/// ends on, so the joined pcurve is C1 there.
pub(in crate::blend) fn fit_track(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    tolerance: f64,
) -> Result<TrackFit, KernelRefusal> {
    fit_track_from(surface, track, at, tolerance, 8)
}

/// [`fit_track`] climbing its rungs from `start` sections (the coarsest it
/// tries) rather than from 8.
pub(in crate::blend) fn fit_track_from(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    tolerance: f64,
    start: usize,
) -> Result<TrackFit, KernelRefusal> {
    let checks = rung_checks(surface, track, at)?;
    fit_track_with(surface, track, at, &checks, tolerance, start)
}

/// What every rung of one track is measured against and joined with: the
/// out-of-sample midpoint check of each dense interval (the curve's own foot,
/// projected from the interval's left foot) and the break tangents.
pub(in crate::blend) struct RungChecks {
    checks: Vec<(f64, Vec3)>,
    tangents: Vec<[[f64; 2]; 2]>,
}

pub(in crate::blend) fn rung_checks(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
) -> Result<RungChecks, KernelRefusal> {
    let last = track.fractions.len() - 1;
    let mut checks: Vec<(f64, Vec3)> = Vec::with_capacity(last);
    for index in 0..last {
        let fraction = 0.5 * (track.fractions[index] + track.fractions[index + 1]);
        let [u, v] = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
        checks.push((fraction, surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?));
    }
    let tangents: Vec<[[f64; 2]; 2]> = track
        .breaks
        .iter()
        .map(|&index| track_tangent(surface, track, at, index))
        .collect::<Result<_, _>>()?;
    Ok(RungChecks { checks, tangents })
}

/// The ladder of [`fit_track_from`] on checks already read.
fn fit_track_with(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    checks: &RungChecks,
    tolerance: f64,
    start: usize,
) -> Result<TrackFit, KernelRefusal> {
    let last = track.fractions.len() - 1;
    let mut best: Option<TrackFit> = None;
    let mut sections = start.max(8);
    loop {
        let sections_here = sections.min(last);
        let pieces = ladder_pieces(track, sections_here);
        let fit = measure_rung(surface, track, at, checks, &pieces, sections_here, tolerance)?;
        let miss = fit.miss;
        if best.as_ref().map_or(true, |known| miss < known.miss) {
            best = Some(fit);
        }
        if miss <= tolerance || sections_here == last {
            break;
        }
        sections *= 2;
    }
    best.ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "no_rung", "blend: the track fit produced no rung"))
}

/// A sample closer to its piece's bound (a break or an end) than this
/// fraction of the interval beyond it is not interpolated by any rung.
const BOUND_SLIVER: f64 = 1e-3;

/// Drop, from one piece's interpolation indices, an interior sample lying
/// within [`BOUND_SLIVER`] of the piece's first or last sample relative to the
/// interval on its far side. Such a sample (the C2 bore: the uniform sample at
/// fraction 0.5, 5.6e-9 after the knot-line crossing at 0.499999994394, with
/// 3.83e-5 to the next) is the bound itself to the fit: interpolating it
/// beside the bound's prescribed tangent turns a rounding-level slope
/// mismatch (about 1e-15 in uv over 5.6e-9) into a bend across the next
/// interval, h2^2 / (4 h1) times that mismatch -- the 2.418e-7 the original
/// bore refused at, on every rung that kept it. The sample stays in the
/// track and its interval's checks still read the rung; only the fit leaves
/// it out, and never below `FIT_DEGREE + 1` samples.
fn drop_bound_slivers(track: &Track, indices: Vec<usize>) -> Vec<usize> {
    let mut indices = indices;
    let f = |index: usize| track.fractions[index];
    if indices.len() > FIT_DEGREE + 1 {
        let (a, b, c) = (indices[0], indices[1], indices[2]);
        if f(b) - f(a) < BOUND_SLIVER * (f(c) - f(b)) {
            indices.remove(1);
        }
    }
    let n = indices.len();
    if n > FIT_DEGREE + 1 {
        let (a, b, c) = (indices[n - 3], indices[n - 2], indices[n - 1]);
        if f(c) - f(b) < BOUND_SLIVER * (f(b) - f(a)) {
            indices.remove(n - 2);
        }
    }
    indices
}

/// The sample indices a LADDER rung of `sections` interpolates, per piece
/// between breaks: with no breaks one piece of `sections` uniformly strided
/// indices; with breaks each piece's share in proportion to its length (at
/// least `MIN_PIECE_INTERVALS`, at most every sample), strided within it.
fn ladder_pieces(track: &Track, sections: usize) -> Vec<Vec<usize>> {
    let last = track.fractions.len() - 1;
    if track.breaks.is_empty() {
        return vec![drop_bound_slivers(track, (0..=sections).map(|k| (k * last / sections).min(last)).collect())];
    }
    let mut bounds = vec![0usize];
    bounds.extend(track.breaks.iter().copied());
    bounds.push(last);
    bounds
        .windows(2)
        .map(|pair| {
            let (from, to) = (pair[0], pair[1]);
            let span = to - from;
            let share = ((sections as f64 * span as f64 / last as f64).ceil() as usize)
                .max(MIN_PIECE_INTERVALS)
                .min(span);
            let mut indices: Vec<usize> = (0..=share).map(|k| from + k * span / share).collect();
            indices.dedup();
            drop_bound_slivers(track, indices)
        })
        .collect()
}

/// The sample indices a NESTED rung interpolates: exactly the track samples at
/// `kept` (fractions, matched bit for bit; absent ones skipped) plus the ends
/// and every break, split into pieces at the breaks. `None` when a piece would
/// have fewer than `FIT_DEGREE + 1` samples.
fn nested_pieces(track: &Track, kept: &[f64]) -> Option<Vec<Vec<usize>>> {
    let last = track.fractions.len() - 1;
    let wanted: std::collections::HashSet<u64> = kept.iter().map(|fraction| fraction.to_bits()).collect();
    let mut bounds = vec![0usize];
    bounds.extend(track.breaks.iter().copied());
    bounds.push(last);
    let mut pieces = Vec::with_capacity(bounds.len() - 1);
    for pair in bounds.windows(2) {
        let (from, to) = (pair[0], pair[1]);
        let indices: Vec<usize> =
            (from..=to).filter(|&index| index == from || index == to || wanted.contains(&track.fractions[index].to_bits())).collect();
        if indices.len() < FIT_DEGREE + 1 {
            return None;
        }
        pieces.push(drop_bound_slivers(track, indices));
    }
    Some(pieces)
}

/// Fit one rung through `pieces` (sample indices per piece between breaks:
/// one piece and a plain cubic interpolant when the track has no breaks, else
/// each piece with the break tangents prescribed and joined C1) and measure it
/// at every dense midpoint check.
fn measure_rung(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    checks: &RungChecks,
    pieces: &[Vec<usize>],
    sections: usize,
    tolerance: f64,
) -> Result<TrackFit, KernelRefusal> {
    let last = track.fractions.len() - 1;
    let debug = std::env::var_os("BREP_DEBUG_TRACK_FIT").is_some();
    let (curve, samples) = if track.breaks.is_empty() {
        let indices = &pieces[0];
        let points: Vec<Vec4> = indices
            .iter()
            .map(|&i| Vec4::from_point(Vec3::new(track.uv[i][0], track.uv[i][1], 0.0), 1.0))
            .collect();
        let parameters: Vec<f64> = indices.iter().map(|&i| track.fractions[i]).collect();
        (
            crate::fit::interpolate_homogeneous(&points, FIT_DEGREE, &parameters)
                .or_refuse(KernelStage::Refine, "interpolate")?,
            indices.len(),
        )
    } else {
        fit_in_c1_pieces(track, &checks.tangents, pieces)?
    };
    let mut worst: Option<(usize, f64)> = None;
    let mut distances = Vec::with_capacity(checks.checks.len());
    for (index, (fraction, on_carrier)) in checks.checks.iter().enumerate() {
        let uv = curve.evaluate(*fraction).or_refuse(KernelStage::Refine, "evaluate")?;
        let distance = pcurve_image(surface, uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?.sub(*on_carrier).length();
        if debug && worst.map_or(true, |(_, known)| distance > known) {
            worst = Some((index, distance));
        }
        distances.push(distance);
    }
    let (miss, over) = classify_checks(&distances, tolerance)?;
    let worst = worst.map(|(index, _)| WorstCheck {
        fraction: checks.checks[index].0,
        interval: index,
        bracket: [track.fractions[index], track.fractions[index + 1]],
    });
    let kept: Vec<f64> = pieces.iter().flatten().map(|&index| track.fractions[index]).collect();
    if let Some(at_worst) = worst {
        eprintln!(
            "TRACK_FIT rung: sections {sections} samples {samples} miss {miss:.3e} worst at fraction {:.9} \
             (dense interval {} of {}: [{:.9}, {:.9}]; {} breaks)",
            at_worst.fraction, at_worst.interval, last, at_worst.bracket[0], at_worst.bracket[1], track.breaks.len()
        );
        if std::env::var_os("BREP_DEBUG_TRACK_WINDOW").is_some() {
            debug_rung_window(surface, track, at, &curve, &kept, at_worst.interval);
        }
    }
    Ok(TrackFit {
        curve,
        miss,
        samples,
        on_floor: miss <= tolerance,
        worst,
        over,
        sections,
        kept,
    })
}

/// `BREP_DEBUG_TRACK_WINDOW` only: the track around dense interval `interval`
/// as the rung saw it -- each sample's fraction, whether it is a break and
/// whether the rung interpolates it, the curve's own standoff from its foot
/// (|S(foot) - C|), and the rung's image error there; and at each break in the
/// window the prescribed (metric) tangent beside the EXACT foot derivative
/// (the foot condition's full Jacobian, with the residual x second-derivative
/// terms) and the secant slopes of the samples either side. Diagnostic: it
/// decides nothing.
fn debug_rung_window(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    curve: &NurbsCurve,
    kept: &[f64],
    interval: usize,
) {
    let last = track.fractions.len() - 1;
    let kept: std::collections::HashSet<u64> = kept.iter().map(|fraction| fraction.to_bits()).collect();
    let breaks: Vec<String> = track.breaks.iter().map(|&index| format!("{:.12}", track.fractions[index])).collect();
    eprintln!("TRACK_WINDOW breaks [{}]", breaks.join(", "));
    for index in interval.saturating_sub(3)..=(interval + 4).min(last) {
        let fraction = track.fractions[index];
        let [u, v] = track.feet[index];
        let standoff = at(fraction).ok().zip(surface.evaluate(u, v).ok()).map(|(point, image)| image.sub(point).length());
        let image_error = curve
            .evaluate(fraction)
            .ok()
            .and_then(|uv| pcurve_image(surface, uv.x, uv.y).ok())
            .zip(surface.evaluate(u, v).ok())
            .map(|(image, on)| image.sub(on).length());
        let is_break = track.breaks.contains(&index);
        eprintln!(
            "TRACK_WINDOW sample {index} fraction {fraction:.12} uv [{:.12}, {:.12}] break {is_break} kept {} standoff {standoff:?} rung error {image_error:?}",
            track.uv[index][0], track.uv[index][1], kept.contains(&fraction.to_bits())
        );
        if is_break && index > 0 && index < last {
            let secant = |a: usize, b: usize| {
                let h = track.fractions[b] - track.fractions[a];
                [(track.uv[b][0] - track.uv[a][0]) / h, (track.uv[b][1] - track.uv[a][1]) / h]
            };
            let metric = track_tangent(surface, track, at, index).ok();
            let exact = one_sided_derivatives(at, fraction).ok().map(|(left, right)| {
                [exact_track_derivative(surface, track.feet[index], at(fraction).ok(), left), exact_track_derivative(surface, track.feet[index], at(fraction).ok(), right)]
            });
            eprintln!(
                "TRACK_WINDOW break {index}: metric tangent {metric:?} exact {exact:?} secant left {:?} right {:?}",
                secant(index - 1, index),
                secant(index, index + 1)
            );
        }
    }
}

/// The EXACT derivative of the foot of a moving point C(f) on `surface`,
/// d(uv)/df at the foot `uv` (where C = `point`, C' = `derivative`): the foot
/// condition S_u.(S - C) = S_v.(S - C) = 0 differentiated, so its Jacobian
/// carries the residual x second-derivative terms the metric-only resolve of
/// [`track_tangent`] omits. Equal to it when C lies on the surface. `None`
/// when unreadable or singular.
pub(in crate::blend) fn exact_track_derivative(surface: &NurbsSurface, uv: [f64; 2], point: Option<Vec3>, derivative: Vec3) -> Option<[f64; 2]> {
    let point = point?;
    let d = surface.derivatives_small(uv[0], uv[1], 2).ok()?;
    let residual = d[0][0].sub(point);
    let (su, sv) = (d[1][0], d[0][1]);
    let j00 = d[2][0].dot(residual) + su.length_squared();
    let j01 = d[1][1].dot(residual) + su.dot(sv);
    let j11 = d[0][2].dot(residual) + sv.length_squared();
    let determinant = j00 * j11 - j01 * j01;
    if !(determinant.abs() > 1e-300) {
        return None;
    }
    let (x, y) = (su.dot(derivative), sv.dot(derivative));
    let result = [(j11 * x - j01 * y) / determinant, (j00 * y - j01 * x) / determinant];
    (result[0].is_finite() && result[1].is_finite()).then_some(result)
}

/// LOCAL refinement of a track: one sample at the curve-fraction midpoint of
/// each listed dense interval — exactly where `fit_track` read that interval's
/// out-of-sample check — projected from the interval's left foot and unwrapped
/// onto its left sample, as every other inserted sample is. Every existing
/// sample (its fraction, uv and foot), the end samples and the break samples
/// are kept bit for bit; break indices follow their samples.
///
/// THE CROSSING CONTRACT HOLDS FOR WHAT A SPLIT EXPOSES. Every knot-line
/// crossing of the track is an exact sample and a break
/// ([`insert_knot_crossings`]); a sign test between neighbours cannot see a
/// pair of crossings inside one interval, and splitting that interval can.
/// Each half of a split interval is therefore read again for a sign change
/// across a knot line, and a crossing it finds is bisected, inserted and broken
/// at exactly as `insert_knot_crossings` does (a crossing within a sliver of a
/// neighbour breaks at that neighbour; a break within `MERGE` of an end or of
/// another break is left unbroken; a piece shorter than `MIN_PIECE_INTERVALS`
/// is densified). Returns how many samples were inserted.
pub(in crate::blend) fn refine_track_intervals(
    surface: &NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    track: &mut Track,
    intervals: &[usize],
    max_intervals: usize,
) -> Result<Option<usize>, KernelRefusal> {
    let lines = knot_lines(surface)?;
    let (closed_u, closed_v) = surface.closed_directions().or_refuse(KernelStage::Refine, "closed_directions")?;
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain")?;
    let periods = [closed_u.then_some(u1 - u0), closed_v.then_some(v1 - v0)];
    let images = |axis: usize, low: f64, high: f64| -> Vec<f64> {
        let mut images = Vec::new();
        for &line in &lines[axis] {
            match periods[axis] {
                None => images.push(line),
                Some(period) => {
                    let first = ((low - line) / period).floor() as i64 - 1;
                    let last = ((high - line) / period).ceil() as i64 + 1;
                    for k in first..=last {
                        images.push(line + k as f64 * period);
                    }
                }
            }
        }
        images
    };
    let resolution = 1e-12;
    // The round is STAGED: every insertion goes into a copy, bounded by
    // `max_intervals` as it is made; the copy replaces `track` only if the
    // whole round — midpoints, the crossings they expose, the densification
    // those breaks require — fits.
    let original = &mut *track;
    let mut staged = original.clone();
    let track = &mut staged;
    let mut intervals: Vec<usize> = intervals.iter().copied().filter(|&index| index + 1 < track.fractions.len()).collect();
    intervals.sort_unstable();
    intervals.dedup();
    let mut inserted = 0;
    let mut new_breaks: Vec<f64> = Vec::new();
    // One sample at `index + 1`, shifting the break indices after it.
    let insert_at = |track: &mut Track, index: usize, fraction: f64, uv: [f64; 2], raw: [f64; 2]| -> bool {
        if track.fractions.len() - 1 >= max_intervals {
            return false;
        }
        track.fractions.insert(index + 1, fraction);
        track.uv.insert(index + 1, uv);
        track.feet.insert(index + 1, raw);
        for brk in track.breaks.iter_mut() {
            if *brk > index {
                *brk += 1;
            }
        }
        true
    };
    // From the back, so earlier indices stay valid.
    for &index in intervals.iter().rev() {
        let fraction = 0.5 * (track.fractions[index] + track.fractions[index + 1]);
        if !(fraction > track.fractions[index] && fraction < track.fractions[index + 1]) {
            continue;
        }
        let raw = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
        let mut uv = raw;
        unwrap_near(&mut uv, &track.uv[index], periods);
        if !insert_at(track, index, fraction, uv, raw) {
            return Ok(None);
        }
        inserted += 1;
        // The two halves, right first so the left half's index stays valid.
        for half in [index + 1, index] {
            let spacing = 1.0 / (track.fractions.len() - 1) as f64;
            let mut crossings: Vec<(f64, [f64; 2], [f64; 2])> = Vec::new();
            for axis in 0..2 {
                let (a, b) = (track.uv[half][axis], track.uv[half + 1][axis]);
                for line in images(axis, a.min(b), a.max(b)) {
                    let (da, db) = (a - line, b - line);
                    if da.abs() <= resolution {
                        // A new sample ON a line is a break, as an old one is.
                        if half == index + 1 {
                            new_breaks.push(track.fractions[half]);
                        }
                        continue;
                    }
                    if db.abs() <= resolution || da.signum() == db.signum() {
                        continue;
                    }
                    let (mut low, mut high) = (track.fractions[half], track.fractions[half + 1]);
                    let mut low_value = da;
                    let mut found = (low, track.uv[half], track.feet[half]);
                    for _ in 0..80 {
                        if high - low <= 1e-15 {
                            break;
                        }
                        let middle = 0.5 * (low + high);
                        let raw = foot(surface, at(middle).or_refuse(KernelStage::Refine, "at")?, Some(track.feet[half]))?;
                        let mut probe = raw;
                        unwrap_near(&mut probe, &track.uv[half], periods);
                        found = (middle, probe, raw);
                        let value = probe[axis] - line;
                        if value.signum() == low_value.signum() {
                            low = middle;
                            low_value = value;
                        } else {
                            high = middle;
                        }
                    }
                    crossings.push(found);
                }
            }
            crossings.sort_by(|a, b| b.0.total_cmp(&a.0));
            for (fraction, uv, raw) in crossings {
                let near_left = fraction - track.fractions[half] <= 1e-9 * spacing;
                let near_right = track.fractions[half + 1] - fraction <= 1e-9 * spacing;
                if inserted_at(&track.fractions, fraction) {
                    new_breaks.push(fraction);
                    continue;
                }
                if near_left || near_right {
                    new_breaks.push(track.fractions[if near_left { half } else { half + 1 }]);
                    continue;
                }
                if !insert_at(track, half, fraction, uv, raw) {
            return Ok(None);
        }
                inserted += 1;
                new_breaks.push(fraction);
            }
        }
    }
    if new_breaks.is_empty() {
        *original = staged;
        return Ok(Some(inserted));
    }
    // New breaks join the old ones under the existing merge rule; the old
    // breaks are kept as they are.
    let mut fractions: Vec<f64> = track.breaks.iter().map(|&index| track.fractions[index]).collect();
    new_breaks.sort_by(f64::total_cmp);
    for fraction in new_breaks {
        let apart = fractions.iter().all(|known| (known - fraction).abs() > MERGE);
        if fraction > MERGE && 1.0 - fraction > MERGE && apart {
            fractions.push(fraction);
        }
    }
    fractions.sort_by(f64::total_cmp);
    // A piece shorter than the fit needs is densified, as `insert_knot_crossings`
    // densifies one.
    let mut bounds = vec![0.0];
    bounds.extend(fractions.iter().copied());
    bounds.push(1.0);
    for pair in bounds.windows(2) {
        let (low, high) = (pair[0], pair[1]);
        let inside = track.fractions.iter().filter(|f| **f > low && **f < high).count();
        if inside + 1 >= MIN_PIECE_INTERVALS {
            continue;
        }
        for k in 1..MIN_PIECE_INTERVALS {
            let fraction = low + (high - low) * k as f64 / MIN_PIECE_INTERVALS as f64;
            // A densified point that IS an existing sample to within a sliver
            // of the piece (the split's own midpoint sits halfway between two
            // bisected crossings) is that sample, not a near-duplicate.
            if track.fractions.iter().any(|known| (known - fraction).abs() <= 1e-9 * (high - low)) {
                continue;
            }
            let index = track
                .fractions
                .windows(2)
                .position(|pair| pair[0] < fraction && fraction < pair[1])
                .ok_or_else(|| {
                    KernelRefusal::internal(KernelStage::Refine, "densified_sample", "blend: a densified sample outside its track")
                })?;
            let raw = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
            let mut uv = raw;
            unwrap_near(&mut uv, &track.uv[index], periods);
            if !insert_at(track, index, fraction, uv, raw) {
            return Ok(None);
        }
            inserted += 1;
        }
    }
    track.breaks = fractions
        .iter()
        .filter_map(|fraction| track.fractions.iter().position(|f| f == fraction))
        .collect();
    *original = staged;
    Ok(Some(inserted))
}

/// The miss of a set of check distances and the checks over `tolerance`. An
/// UNREADABLE check (a non-finite distance) is not a measurement: it is the
/// typed refine refusal `track_fit_unreadable`, before any classification, so
/// no fit is judged, refined against or returned on it (`f64::max` alone drops
/// a NaN). For finite distances the miss is the same left-to-right max from
/// zero `fit_track` always took, bit for bit.
fn classify_checks(distances: &[f64], tolerance: f64) -> Result<(f64, Vec<usize>), KernelRefusal> {
    if let Some((index, distance)) = distances.iter().enumerate().find(|(_, distance)| !distance.is_finite()) {
        return Err(KernelRefusal::non_convergence(KernelStage::Refine, "track_fit_unreadable", format!(
            "{PCURVE_OFF_FLOOR} check {index} of {} on the track is unreadable (distance {distance})",
            distances.len()
        )));
    }
    let mut miss: f64 = 0.0;
    let mut over = Vec::new();
    for (index, &distance) in distances.iter().enumerate() {
        if !(distance <= tolerance) {
            over.push(index);
        }
        miss = miss.max(distance);
    }
    Ok((miss, over))
}

/// A SHIPPED support pcurve read against its rail at the rail's own
/// parameter, after every edit made to it: at each of the pcurve's knots in
/// `[d0, d1]`, 15 interior points j/16 of every knot span and 513 uniform
/// points, the rail point `rail(t)` is projected onto `surface` (seeded from
/// the pcurve's own (u, v), polished as the fitter's feet are) and the
/// pcurve's image `surface(pcurve(t))` — read through `evaluate_extended`,
/// which wraps only the surface's actually closed directions — is measured
/// from that foot: the fitter's own track residual. Returns `(miss, rail
/// offset, samples)`: the largest such residual, and separately the largest
/// distance of the rail itself from its foot (the rail's own standoff from
/// its carrier, which no pcurve on the carrier can remove), the sample count,
/// and the parameter of the largest residual. An unreadable sample refuses
/// (`track_fit_unreadable`).
pub(in crate::blend) fn read_shipped_pcurve(
    surface: &NurbsSurface,
    rail: &dyn Fn(f64) -> Result<Vec3, String>,
    pcurve: &crate::NurbsCurve,
    d0: f64,
    d1: f64,
) -> Result<(f64, f64, usize, f64), KernelRefusal> {
    let mut parameters: Vec<f64> = pcurve.knots.iter().copied().filter(|knot| *knot >= d0 && *knot <= d1).collect();
    parameters.push(d0);
    parameters.push(d1);
    parameters.sort_by(f64::total_cmp);
    parameters.dedup();
    let spans: Vec<f64> = parameters.windows(2).flat_map(|pair| (1..16).map(move |j| pair[0] + (pair[1] - pair[0]) * j as f64 / 16.0)).collect();
    parameters.extend(spans);
    parameters.extend((0..=512).map(|k| d0 + (d1 - d0) * k as f64 / 512.0));
    let mut residuals = Vec::with_capacity(parameters.len());
    let mut offset = 0.0_f64;
    for &t in &parameters {
        let read = (|| -> Result<(f64, f64), KernelRefusal> {
            let uv = pcurve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
            let point = rail(t).or_refuse(KernelStage::Refine, "rail")?;
            let [u, v] = foot(surface, point, Some([uv.x, uv.y]))?;
            let on_carrier = surface.evaluate_extended(u, v).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            let image = surface.evaluate_extended(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate_extended")?;
            Ok((image.sub(on_carrier).length(), point.sub(on_carrier).length()))
        })();
        // A sample is read only when BOTH its residual and the rail's
        // standoff are finite; anything else is unreadable and refuses.
        match read {
            Ok((residual, standoff)) if residual.is_finite() && standoff.is_finite() => {
                residuals.push(residual);
                offset = offset.max(standoff);
            }
            _ => residuals.push(f64::NAN),
        }
    }
    let (miss, _) = classify_checks(&residuals, f64::INFINITY)?;
    let worst = residuals.iter().zip(&parameters).fold((0.0_f64, d0), |best, (&residual, &t)| if residual > best.0 { (residual, t) } else { best }).1;
    Ok((miss, offset, residuals.len(), worst))
}

/// How much farther the rail stands from its foot under pcurve `candidate`
/// than under pcurve `reference`, read at the SAME rail parameters: every
/// knot of either curve in `[d0, d1]`, j/16 of each span between them, and
/// 513 uniform points. At each parameter the rail point is projected from
/// each curve's own (u, v) (`foot`), and the largest excess of the
/// candidate's standoff over the reference's is returned. A candidate on the
/// reference's branch reads its standoff to rounding; one on another branch
/// reads the distance between the branches. Two `read_shipped_pcurve` maxima
/// cannot stand in for this: each samples at its own curve's knots, and
/// where the rail's standoff peaks sharply they differ by the sampling alone
/// (6e-7 at a 1.007e-4 peak on the 2026-09-27 offset-shell revolve). An
/// unreadable or non-finite sample refuses (`track_fit_unreadable`).
pub(in crate::blend) fn standoff_excess(
    surface: &NurbsSurface,
    rail: &dyn Fn(f64) -> Result<Vec3, String>,
    reference: &crate::NurbsCurve,
    candidate: &crate::NurbsCurve,
    d0: f64,
    d1: f64,
) -> Result<f64, KernelRefusal> {
    let mut parameters: Vec<f64> = reference.knots.iter().chain(&candidate.knots).copied().filter(|knot| *knot >= d0 && *knot <= d1).collect();
    parameters.push(d0);
    parameters.push(d1);
    parameters.sort_by(f64::total_cmp);
    parameters.dedup();
    let spans: Vec<f64> = parameters.windows(2).flat_map(|pair| (1..16).map(move |j| pair[0] + (pair[1] - pair[0]) * j as f64 / 16.0)).collect();
    parameters.extend(spans);
    parameters.extend((0..=512).map(|k| d0 + (d1 - d0) * k as f64 / 512.0));
    let standoff = |pcurve: &crate::NurbsCurve, point: Vec3, t: f64| -> Result<f64, KernelRefusal> {
        let uv = pcurve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let [u, v] = foot(surface, point, Some([uv.x, uv.y]))?;
        Ok(point.sub(surface.evaluate_extended(u, v).or_refuse(KernelStage::Refine, "evaluate_extended")?).length())
    };
    let mut excess = f64::NEG_INFINITY;
    for &t in &parameters {
        let point = rail(t).or_refuse(KernelStage::Refine, "rail")?;
        let difference = standoff(candidate, point, t)? - standoff(reference, point, t)?;
        if !difference.is_finite() {
            return Err(KernelRefusal::non_convergence(KernelStage::Refine, "track_fit_unreadable", format!(
                "blend: a support trim's standoff is unreadable at t = {t}"
            )));
        }
        excess = excess.max(difference);
    }
    Ok(excess)
}

/// How much farther the rail stands from its foot under pcurve `candidate`
/// than from its NEAREST point on `surface` (the kernel's global nearest-point
/// projector), read at every knot of `candidate` in `[d0, d1]` and 513
/// uniform points: the rail's own deviation from its carrier, read
/// independently of any trim, as the absolute bound a shipped support trim is
/// held to. A trim on the rail's branch reads its foot at the nearest point
/// (excess at rounding); a trim on another branch -- even one the march's own
/// trim shares, which a comparison with that trim cannot see -- reads the
/// distance between the branches. Where the carrier comes back nearer than
/// the rail's own branch the excess is real distance too, and refuses. An
/// unreadable or non-finite sample refuses (`track_fit_unreadable`).
pub(in crate::blend) fn standoff_over_nearest(
    surface: &NurbsSurface,
    rail: &dyn Fn(f64) -> Result<Vec3, String>,
    candidate: &crate::NurbsCurve,
    d0: f64,
    d1: f64,
) -> Result<f64, KernelRefusal> {
    let mut parameters: Vec<f64> = candidate.knots.iter().copied().filter(|knot| *knot >= d0 && *knot <= d1).collect();
    parameters.extend((0..=512).map(|k| d0 + (d1 - d0) * k as f64 / 512.0));
    parameters.sort_by(f64::total_cmp);
    parameters.dedup();
    let mut excess = f64::NEG_INFINITY;
    for &t in &parameters {
        let point = rail(t).or_refuse(KernelStage::Refine, "rail")?;
        let uv = candidate.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
        let [u, v] = foot(surface, point, Some([uv.x, uv.y]))?;
        let standoff = point.sub(surface.evaluate_extended(u, v).or_refuse(KernelStage::Refine, "evaluate_extended")?).length();
        let nearest = crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project")?.distance;
        let difference = standoff - nearest;
        if !difference.is_finite() {
            return Err(KernelRefusal::non_convergence(KernelStage::Refine, "track_fit_unreadable", format!(
                "blend: a support trim's standoff is unreadable at t = {t}"
            )));
        }
        excess = excess.max(difference);
    }
    Ok(excess)
}

/// Points of every interval the local-repair path judges a returned fit on:
/// j/16, j = 1..15 — the midpoint `fit_track` checks and the quarters among them.
const LOCAL_STENCIL: usize = 16;

/// The local-repair verdict on a returned pcurve: the curve's own foot at
/// every stencil point of every track interval (projected from that interval's
/// left foot, as `fit_track`'s checks are) against the pcurve pushed through
/// the surface. Returns the classified miss and the intervals with any point
/// over `tolerance`, and where the largest reading sits (the fraction of the
/// stencil point and its interval) -- the STENCIL's own worst, which is what a
/// caller that reports the stencil miss must name, not a midpoint's.
fn stencil_verdict(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    curve: &NurbsCurve,
    tolerance: f64,
) -> Result<(f64, Vec<usize>, Option<WorstCheck>, Vec<(f64, f64, f64, f64)>), KernelRefusal> {
    let mut per_interval = Vec::with_capacity(track.fractions.len() - 1);
    // Each interval's own (miss, fraction of its worst stencil point, low, high).
    let mut located = Vec::with_capacity(track.fractions.len() - 1);
    let mut worst: Option<(f64, WorstCheck)> = None;
    for index in 0..track.fractions.len() - 1 {
        let (low, high) = (track.fractions[index], track.fractions[index + 1]);
        let mut distances = Vec::with_capacity(LOCAL_STENCIL - 1);
        let mut interval_worst = (0.0_f64, 0.5 * (low + high));
        for j in 1..LOCAL_STENCIL {
            let fraction = low + (high - low) * j as f64 / LOCAL_STENCIL as f64;
            let [u, v] = foot_among(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, &[track.feet[index], track.feet[index + 1]])?;
            let on_carrier = surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?;
            let uv = curve.evaluate(fraction).or_refuse(KernelStage::Refine, "evaluate")?;
            let distance = pcurve_image(surface, uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?.sub(on_carrier).length();
            if worst.as_ref().map_or(true, |(known, _)| distance > *known) {
                worst = Some((distance, WorstCheck { fraction, interval: index, bracket: [low, high] }));
            }
            if distance > interval_worst.0 {
                interval_worst = (distance, fraction);
            }
            distances.push(distance);
        }
        let (miss, _) = classify_checks(&distances, tolerance)?;
        per_interval.push(miss);
        located.push((miss, interval_worst.1, low, high));
    }
    let (miss, over) = classify_checks(&per_interval, tolerance)?;
    Ok((miss, over, worst.map(|(_, at)| at), located))
}

/// [`fit_track`]; a fit on the floor that ALSO passes [`stencil_verdict`] is
/// returned exactly as `fit_track` returned it. A fit on the floor by its
/// midpoint checks alone is not trusted with that: on the 20-degree crossing's
/// mirrored single closed exit edge the top rung read 9.898e-8 at its
/// midpoints while the curve stood 1.497e-7 off the rail's track between them
/// (fraction 0.7189; c1a4e971c + the section-fit fix), and nothing read it
/// until the shipped curve was. Such a fit, and every fit off the floor, takes
/// LOCAL REPAIR: every fit this path returns is judged on
/// [`stencil_verdict`] — the maximum over 15 points of every final interval,
/// not the midpoint alone — and the intervals that verdict finds over
/// `tolerance` (with those the midpoint checks found) take one sample each
/// ([`refine_track_intervals`]), for up to `rounds` rounds while the track
/// stays within `max_intervals` INTERVALS (`max_intervals + 1` samples). The
/// returned fit's `miss`, `over` and `on_floor` are that stencil verdict, so
/// a miss the midpoint did not see either drives another round or reaches
/// the caller's unchanged refusal.
pub(in crate::blend) fn fit_track_locally(
    surface: &NurbsSurface,
    track: &mut Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    tolerance: f64,
    rounds: usize,
    max_intervals: usize,
) -> Result<(TrackFit, usize), KernelRefusal> {
    let mut fit = fit_track(surface, track, at, tolerance)?;
    if fit.on_floor {
        #[cfg(not(test))]
        let trust_midpoints = false;
        if trust_midpoints {
            return Ok((fit, 0));
        }
        let (_, over, _, _) = stencil_verdict(surface, track, at, &fit.curve, tolerance)?;
        if over.is_empty() {
            return Ok((fit, 0));
        }
    }
    let debug = std::env::var_os("BREP_DEBUG_TRACK_FIT").is_some();
    let mut inserted = 0;
    let mut round = 0;
    // The coarsest rung every later fit may use: raised when a rung passed
    // its midpoints but not its stencil, so a refit after insertion never
    // falls back to the rung that just failed.
    let mut start = 8usize;
    // The stencil verdict of the fit a round chose, when the choice already
    // read it (so it is read once, as it always was, not twice).
    let mut judged: Option<(f64, Vec<usize>, Option<WorstCheck>, Vec<(f64, f64, f64, f64)>)> = None;
    // The previous round's stencil miss: the measured convergence rate.
    let mut previous_miss: Option<f64> = None;
    // This round's over intervals, located: (miss, worst fraction, low, high).
    let mut over_located: Vec<(f64, f64, f64, f64)> = Vec::new();
    loop {
        #[cfg(not(test))]
        let midpoint_only = false;
        let (miss, over) = if midpoint_only {
            (fit.miss, fit.over.clone())
        } else {
            let (miss, over, worst, located) = match judged.take() {
                Some(verdict) => verdict,
                None => stencil_verdict(surface, track, at, &fit.curve, tolerance)?,
            };
            over_located = over.iter().filter_map(|&index| located.get(index).copied()).collect();
            // The reported miss is the stencil's, so is where it sits
            // (diagnostic provenance only; `worst` stays None off debug).
            if fit.worst.is_some() {
                fit.worst = worst;
            }
            (miss, over)
        };
        let midpoint_over = std::mem::take(&mut fit.over);
        fit.miss = miss;
        fit.on_floor = miss <= tolerance && over.is_empty();
        fit.over = over;
        if debug {
            eprintln!(
                "TRACK_FIT local round {round}: {} intervals, stencil miss {:.3e} ({} over; midpoint checks {} over), on floor {}",
                track.fractions.len() - 1, fit.miss, fit.over.len(), midpoint_over.len(), fit.on_floor
            );
        }
        if fit.on_floor || round >= rounds {
            break;
        }
        // A rung whose MIDPOINT checks all pass while its stencil misses is
        // short of sections, not of samples: `fit_track` stops at the coarsest
        // rung its midpoints accept, so a stencil miss between its midpoints
        // would otherwise send samples into a rung that never asked for them.
        // Such a round takes the next rung -- twice the sections, on the track
        // as it is -- before any sample is inserted. Within the same rounds;
        // the top rung (every interval) inserts. (An alias repair with its own
        // control; it is NOT the C2 bore's stall: there every round had
        // midpoint checks over, and this branch never fires.)
        // The climb is a CHALLENGER, judged on the same stencil: it replaces
        // the fit only when it reads strictly smaller. Otherwise the round
        // inserts at the stencil's misses as any other (the original bore:
        // at round 2 the nested rung read 1.002e-7 with ONE interval over and
        // its midpoints clean, and the unconditional climb replaced it with a
        // 1258-section rung reading 1.508e-5).
        let last = track.fractions.len() - 1;
        if !midpoint_only && midpoint_over.is_empty() && fit.sections < last {
            let climb = (fit.sections * 2).min(last);
            // OPTIONAL: a climb that cannot be fitted or read is a declined
            // challenger -- the standing fit and its bounded repair go on,
            // never a refusal of the whole repair.
            #[cfg(not(test))]
            let climb_fails = false;
            let challenger = if climb_fails {
                Err(KernelRefusal::internal(KernelStage::Refine, "climb_control", "blend: control: the climbed rung cannot be read"))
            } else {
                fit_track_from(surface, track, at, tolerance, climb)
                    .and_then(|climbed| stencil_verdict(surface, track, at, &climbed.curve, tolerance).map(|verdict| (climbed, verdict)))
            };
            match challenger {
                Ok((climbed, verdict)) => {
                    if debug {
                        eprintln!(
                            "TRACK_FIT local round {round}: its midpoints passed at {} sections (stencil {:.3e}); the climb from {climb} reads {:.3e}",
                            fit.sections, fit.miss, verdict.0
                        );
                    }
                    if verdict.0 < fit.miss {
                        start = climb;
                        round += 1;
                        fit = climbed;
                        judged = Some(verdict);
                        continue;
                    }
                }
                Err(error) => {
                    if debug {
                        eprintln!("TRACK_FIT local round {round}: the climb from {climb} was declined: {error}");
                    }
                }
            }
        }
        let mut targets: Vec<usize> = fit.over.iter().chain(&midpoint_over).copied().collect();
        targets.sort_unstable();
        targets.dedup();
        // A round that cannot fit under the ceiling — with every crossing and
        // densification it requires — is declined whole: the track and this
        // off-floor verdict stand, and the caller's refusal reads them.
        let before: std::collections::HashSet<u64> = track.fractions.iter().map(|fraction| fraction.to_bits()).collect();
        let Some(added) = refine_track_intervals(surface, at, track, &targets, max_intervals)? else {
            if debug {
                eprintln!("TRACK_FIT local round {round}: declined — it does not fit within {max_intervals} intervals");
            }
            break;
        };
        if added == 0 {
            break;
        }
        inserted += added;
        // LOW-ORDER convergence: a smooth track's stencil miss falls ~16x per
        // bisection (h^4), but a track with a square-root foot -- a curve
        // reaching a carrier's STATIONARY isoline, where S_u vanishes and the
        // foot moves like the square root of the distance -- falls only ~2x
        // per bisection of the interval touching the singular point
        // (zero_tangent_sites' stationary end halved 8.79e-3 -> 1.36e-4 in six
        // rounds, worst at fraction 0.99995 of a track ending there, and
        // refused at the floor). The singular point is a SAMPLE -- the track's
        // end or a stationary junction -- so each over interval is graded
        // toward its endpoint nearer its own worst stencil point: the
        // sub-interval touching that endpoint is bisected again,
        // log2(miss / tolerance) times in all this round, when the miss fell
        // less than 2.5x since the last round (measured, from the stencil).
        // Same round, same ceiling (each step is staged and declined whole
        // past it), same floor.
        let low_order = previous_miss.is_some_and(|previous| previous.is_finite() && miss > previous / 2.5);
        previous_miss = Some(miss);
        if low_order && !midpoint_only {
            // (endpoint bits, the interval lies BELOW it?, bisections owed)
            let anchors: Vec<(u64, bool, usize)> = over_located
                .iter()
                .map(|&(interval_miss, worst, low, high)| {
                    let owed = ((interval_miss / tolerance).log2().ceil().max(1.0) as usize).saturating_sub(1).min(64);
                    if worst - low < high - worst { (low.to_bits(), false, owed) } else { (high.to_bits(), true, owed) }
                })
                .collect();
            let steps = anchors.iter().map(|anchor| anchor.2).max().unwrap_or(0);
            for step in 0..steps {
                let mut graded: Vec<usize> = anchors
                    .iter()
                    .filter(|anchor| anchor.2 > step)
                    .filter_map(|&(bits, below, _)| {
                        let at = track.fractions.iter().position(|known| known.to_bits() == bits)?;
                        if below { at.checked_sub(1) } else { (at + 1 < track.fractions.len()).then_some(at) }
                    })
                    .collect();
                graded.sort_unstable();
                graded.dedup();
                if graded.is_empty() {
                    break;
                }
                match refine_track_intervals(surface, at, track, &graded, max_intervals)? {
                    Some(more) if more > 0 => inserted += more,
                    _ => break,
                }
            }
            if debug {
                eprintln!("TRACK_FIT local round {round}: low-order convergence; graded {steps} bisection(s) toward the nearer endpoints");
            }
        }
        round += 1;
        // The ladder re-strides every rung uniformly by INDEX over the grown
        // track, so the samples this round inserted where the returned rung
        // missed MAY be absent from the rung the ladder next returns (only the
        // top rung, which interpolates every sample, is sure to keep them; the
        // q1 trace logged no index sets, so which witnesses its 512 and 1024
        // rungs actually dropped is not known). The NESTED rung keeps the
        // returned rung's own samples and adds exactly the new ones (every
        // sample this round inserted: midpoints, the crossings they exposed
        // and their densification -- all inside the track, which
        // `refine_track_intervals` already held to `max_intervals`).
        //
        // It is an OPTIONAL challenger: the ladder is fitted and judged first,
        // exactly as before; the nested rung is taken only when the SAME
        // authoritative stencil verdict that judges every returned fit reads it
        // strictly smaller (the ladder on a tie), and any failure to build or
        // read it leaves the ladder standing. Same checks, rounds, ceiling and
        // floor.
        let checks = rung_checks(surface, track, at)?;
        let ladder = fit_track_with(surface, track, at, &checks, tolerance, start)?;
        if midpoint_only {
            fit = ladder;
            continue;
        }
        let ladder_verdict = stencil_verdict(surface, track, at, &ladder.curve, tolerance)?;
        let mut kept = fit.kept.clone();
        kept.extend(track.fractions.iter().copied().filter(|fraction| !before.contains(&fraction.to_bits())));
        #[cfg(not(test))]
        let nested_off = false;
        #[cfg(not(test))]
        let nested_fails = false;
        let challenger = if nested_off {
            None
        } else {
            nested_pieces(track, &kept).and_then(|pieces| {
                let sections = pieces.iter().map(|piece| piece.len() - 1).sum();
                let measured = if nested_fails {
                    Err(KernelRefusal::internal(KernelStage::Refine, "nested_rung_control", "blend: control: the nested rung cannot be read"))
                } else {
                    measure_rung(surface, track, at, &checks, &pieces, sections, tolerance)
                };
                let nested = measured.ok()?;
                let verdict = stencil_verdict(surface, track, at, &nested.curve, tolerance).ok()?;
                Some((nested, verdict))
            })
        };
        if debug {
            eprintln!(
                "TRACK_FIT local round {round}: ladder {} sections stencil {:.3e}; nested {:?}",
                ladder.sections,
                ladder_verdict.0,
                challenger.as_ref().map(|(nested, verdict)| (nested.sections, verdict.0))
            );
        }
        match challenger {
            Some((nested, verdict)) if verdict.0 < ladder_verdict.0 => {
                fit = nested;
                judged = Some(verdict);
            }
            _ => {
                fit = ladder;
                judged = Some(ladder_verdict);
            }
        }
    }
    Ok((fit, inserted))
}




/// Project and fit a curve's track, doubling the dense grid from `dense` up to
/// `dense_limit` while the top rung of a grid still misses `tolerance` — the
/// floor is absolute, so a larger model needs more samples of the same arc (a
/// 1000-cube's corner patch uses all 514 of a 512 grid). The last fit is
/// returned with its measure either way; the caller decides.
pub(in crate::blend) fn fit_curve_track(
    surface: &NurbsSurface,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    curve_breaks: &[f64],
    tolerance: f64,
    dense: usize,
    dense_limit: usize,
) -> Result<TrackFit, KernelRefusal> {
    let mut dense = dense;
    loop {
        let track = project_track(surface, at, dense, curve_breaks)?;
        let fit = fit_track(surface, &track, at, tolerance)?;
        if fit.on_floor || dense * 2 > dense_limit {
            return Ok(fit);
        }
        dense *= 2;
    }
}

/// The track's tangent from each side, d(uv)/d(fraction), at sample `index`:
/// the curve's own one-sided derivatives, each resolved onto the surface's
/// first derivatives ON ITS OWN SIDE of the crossing (`side_derivatives`).
/// A knot line of multiplicity equal to the degree is only C0 in its
/// parameterisation, even where the surface is G1 — a revolved composite
/// Bezier profile has S_v jump across its segment joints, and the track's
/// slope with it (3.44 to 2.38 on the 2026-09-27 revolve; resolved on one
/// side's derivatives, both pieces took that side's tangent and every rung
/// missed by h times the jump, halving per bisection and never converging).
/// Where the two sides agree (a C1 line) one tangent is returned for both.
fn track_tangent(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    index: usize,
) -> Result<[[f64; 2]; 2], KernelRefusal> {
    let fraction = track.fractions[index];
    let (left, right) = one_sided_derivatives(at, fraction)?;
    let resolve = |foot: [f64; 2], derivative: Vec3| -> Result<[f64; 2], KernelRefusal> {
        let (_, su, sv) = surface.deriv1(foot[0], foot[1]).or_refuse(KernelStage::Refine, "deriv1")?;
        let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
        let determinant = a * c - b * b;
        if !(determinant.abs() > 1e-300) {
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "pole_tangent",
                "blend: the track tangent is undefined at a knot-line crossing (a pole)",
            ));
        }
        let (x, y) = (su.dot(derivative), sv.dot(derivative));
        Ok([(c * x - b * y) / determinant, (a * y - b * x) / determinant])
    };
    let [left_foot, right_foot] = side_feet(surface, track, index)?;
    let (left, right) = (resolve(left_foot, left)?, resolve(right_foot, right)?);
    // Where the two sides agree to the difference's own accuracy the track is
    // C1 and both pieces take ONE tangent, so the join is exactly C1.
    let scale = (left[0].hypot(left[1])).max(right[0].hypot(right[1])).max(1e-300);
    if (left[0] - right[0]).hypot(left[1] - right[1]) <= 1e-7 * scale {
        let mean = [0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1])];
        return Ok([mean, mean]);
    }
    Ok([left, right])
}

/// The feet at which each side of sample `index` reads the surface's first
/// derivatives: the sample's own foot moved a hair (1e-9 of the larger
/// parameter span, at most half the way to the next distinct knot line)
/// toward the nearest distinct sample on that side, so a foot
/// lying ON a knot line is read on the span its side of the track runs in
/// (`deriv1` at the line itself reads one span for both). Off a line the
/// shift moves the derivatives by that hair times the surface's second
/// derivative, far below the 1e-7 agreement `track_tangent` merges at. A
/// side with no distinct sample reads the foot itself.
fn side_feet(surface: &NurbsSurface, track: &Track, index: usize) -> Result<[[f64; 2]; 2], KernelRefusal> {
    let foot = track.feet[index];
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let hair = 1e-9 * (u1 - u0).max(v1 - v0);
    let here = track.uv[index];
    // Half the way, along a unit direction component, to the next DISTINCT
    // knot of that direction past the foot: the hair never reaches a span
    // beyond its side's own. A knot within a tenth of the hair IS the line
    // the foot lies on (a crossing bisected to the fraction's resolution sits
    // 5e-12 off its line in uv), which the hair must cross, not stop short of.
    let room = |knots: &[f64], value: f64, component: f64, _span: f64| -> f64 {
        if component.abs() <= f64::EPSILON {
            return f64::INFINITY;
        }
        let distinct = 0.1 * hair;
        let next = knots
            .iter()
            .map(|knot| (knot - value) * component.signum())
            .filter(|ahead| *ahead > distinct)
            .fold(f64::INFINITY, f64::min);
        0.5 * next / component.abs()
    };
    let toward = |neighbours: &mut dyn Iterator<Item = usize>| -> [f64; 2] {
        for other in neighbours {
            let (du, dv) = (track.uv[other][0] - here[0], track.uv[other][1] - here[1]);
            let length = du.hypot(dv);
            if length > 1e-14 && length.is_finite() {
                let (cu, cv) = (du / length, dv / length);
                // Not clamped to the neighbour: a crossing's neighbour can be
                // a sliver (1e-12) away, and half of that is the line itself.
                let step = hair
                    .min(room(&surface.knots_u, foot[0], cu, u1 - u0))
                    .min(room(&surface.knots_v, foot[1], cv, v1 - v0));
                return [foot[0] + step * cu, foot[1] + step * cv];
            }
        }
        foot
    };
    Ok([toward(&mut (0..index).rev()), toward(&mut (index + 1..track.uv.len()))])
}



/// The curve's one-sided derivatives d C / d fraction at `fraction`, by
/// second-order ONE-SIDED differences: a curve break may be a genuine corner,
/// where a central difference would average two tangents neither side has.
fn one_sided_derivatives(at: &dyn Fn(f64) -> Result<Vec3, String>, fraction: f64) -> Result<(Vec3, Vec3), KernelRefusal> {
    let step = 1e-5;
    let here = at(fraction).or_refuse(KernelStage::Refine, "at")?;
    let left = here
        .scale(3.0)
        .sub(at(fraction - step).or_refuse(KernelStage::Refine, "at")?.scale(4.0))
        .add(at(fraction - 2.0 * step).or_refuse(KernelStage::Refine, "at")?)
        .scale(1.0 / (2.0 * step));
    let right = at(fraction + step).or_refuse(KernelStage::Refine, "at")?
        .scale(4.0)
        .sub(here.scale(3.0))
        .sub(at(fraction + 2.0 * step).or_refuse(KernelStage::Refine, "at")?)
        .scale(1.0 / (2.0 * step));
    Ok((left, right))
}

/// One rung with breaks: each piece interpolated through its share of samples
/// with the break tangents prescribed — the one tangent where the track is C1,
/// each side's own where the curve itself has a corner — joined at knots of
/// multiplicity 3 into one curve.
fn fit_in_c1_pieces(
    track: &Track,
    tangents: &[[[f64; 2]; 2]],
    piece_indices: &[Vec<usize>],
) -> Result<(NurbsCurve, usize), KernelRefusal> {
    let pieces = piece_indices.len();
    let mut knots: Vec<f64> = Vec::new();
    let mut control: Vec<Vec4> = Vec::new();
    let mut samples = 0usize;
    for piece in 0..pieces {
        let indices = &piece_indices[piece];
        samples += indices.len() - usize::from(piece > 0);
        let points: Vec<[f64; 2]> = indices.iter().map(|&i| track.uv[i]).collect();
        let parameters: Vec<f64> = indices.iter().map(|&i| track.fractions[i]).collect();
        let start = (piece > 0).then(|| tangents[piece - 1][1]);
        let end = (piece + 1 < pieces).then(|| tangents[piece][0]);
        let fitted = interpolate_with_tangents(&points, &parameters, start, end)?;
        if piece == 0 {
            knots.extend(std::iter::repeat(parameters[0]).take(FIT_DEGREE + 1));
        }
        let interior = &fitted.knots[FIT_DEGREE + 1..fitted.knots.len() - FIT_DEGREE - 1];
        knots.extend(interior.iter().copied());
        let multiplicity = if piece + 1 == pieces { FIT_DEGREE + 1 } else { FIT_DEGREE };
        knots.extend(std::iter::repeat(parameters[parameters.len() - 1]).take(multiplicity));
        control.extend(fitted.control_points.iter().skip(usize::from(piece > 0)).copied());
    }
    Ok((NurbsCurve::new(FIT_DEGREE, knots, control).or_refuse(KernelStage::Refine, "curve_new")?, samples))
}

/// Cubic interpolation of 2D `points` at `parameters`, clamped over the
/// parameters' own interval, with an optional prescribed first derivative at
/// either end (The NURBS Book §9.2.2). Each prescribed derivative adds one
/// control point, and the interior knots average the parameters with that end
/// parameter counted twice, which is Eq. 9.22 when both are prescribed and the
/// A9.1 averaging when neither is.
fn interpolate_with_tangents(
    points: &[[f64; 2]],
    parameters: &[f64],
    start: Option<[f64; 2]>,
    end: Option<[f64; 2]>,
) -> Result<NurbsCurve, KernelRefusal> {
    let degree = FIT_DEGREE;
    let n = points.len() - 1;
    if n < degree || parameters.len() != points.len() {
        return Err(KernelRefusal::internal(
            KernelStage::Refine,
            "piece_short",
            "blend: a track piece is too short for the fit degree",
        ));
    }
    let (t0, t1) = (parameters[0], parameters[n]);
    let mut averaged: Vec<f64> = Vec::with_capacity(n + 3);
    if start.is_some() {
        averaged.push(t0);
    }
    averaged.extend_from_slice(parameters);
    if end.is_some() {
        averaged.push(t1);
    }
    let controls = averaged.len();
    let mut knots = vec![t0; degree + 1];
    for j in 1..controls - degree {
        knots.push(averaged[j..j + degree].iter().sum::<f64>() / degree as f64);
    }
    knots.extend(std::iter::repeat(t1).take(degree + 1));
    let knot_vector = crate::KnotVector::new(knots.clone(), degree).or_refuse(KernelStage::Refine, "knot_vector")?;
    let mut matrix = vec![vec![0.0; controls]; controls];
    let mut rhs: Vec<[f64; 2]> = vec![[0.0; 2]; controls];
    let mut row = 0usize;
    for (index, &parameter) in parameters.iter().enumerate() {
        if index == 0 {
            matrix[row][0] = 1.0;
            rhs[row] = points[0];
            row += 1;
            if let Some(tangent) = start {
                let scale = degree as f64 / (knots[degree + 1] - t0);
                matrix[row][0] = -scale;
                matrix[row][1] = scale;
                rhs[row] = tangent;
                row += 1;
            }
            continue;
        }
        if index == n {
            if let Some(tangent) = end {
                let scale = degree as f64 / (t1 - knots[controls - 1]);
                matrix[row][controls - 2] = -scale;
                matrix[row][controls - 1] = scale;
                rhs[row] = tangent;
                row += 1;
            }
            matrix[row][controls - 1] = 1.0;
            rhs[row] = points[n];
            continue;
        }
        let span = knot_vector.find_span(parameter);
        for (offset, value) in knot_vector.basis_functions(span, parameter).into_iter().enumerate() {
            matrix[row][span - degree + offset] = value;
        }
        rhs[row] = points[index];
        row += 1;
    }
    let solve = |axis: usize| {
        crate::fit::solve_collocation(&matrix, &rhs.iter().map(|value| value[axis]).collect::<Vec<_>>(), degree)
    };
    let (us, vs) = (
        solve(0).or_refuse(KernelStage::Refine, "collocation")?,
        solve(1).or_refuse(KernelStage::Refine, "collocation")?,
    );
    NurbsCurve::new(
        degree,
        knots,
        (0..controls)
            .map(|index| Vec4::from_point(Vec3::new(us[index], vs[index], 0.0), 1.0))
            .collect(),
    )
    .or_refuse(KernelStage::Refine, "curve_new")
}

/// Whether `fraction` is already a sample of `fractions` (two breaks — a curve
/// knot and a surface crossing — can name one point).
fn inserted_at(fractions: &[f64], fraction: f64) -> bool {
    fractions.iter().any(|known| (known - fraction).abs() <= 1e-15)
}
