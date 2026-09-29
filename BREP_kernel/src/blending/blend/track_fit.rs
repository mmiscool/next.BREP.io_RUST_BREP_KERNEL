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

/// The foot of `point` seeded from `seed` (or the nearest point without one),
/// polished ([`polish`]).
fn foot(surface: &NurbsSurface, point: Vec3, seed: Option<[f64; 2]>) -> Result<[f64; 2], KernelRefusal> {
    let projected = match seed {
        None => crate::project_point_to_surface(surface, point).or_refuse(KernelStage::Refine, "project")?,
        Some(seed) => crate::projection::project_point_to_surface_from_seed(surface, point, seed)
            .or_refuse(KernelStage::Refine, "project")?,
    };
    polish(surface, point, [projected.u, projected.v])
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
        let raw = foot(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, Some(track.feet[index]))?;
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
            let raw = foot(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, Some(track.feet[index]))?;
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
    let last = track.fractions.len() - 1;
    let mut checks: Vec<(f64, Vec3)> = Vec::with_capacity(last);
    for index in 0..last {
        let fraction = 0.5 * (track.fractions[index] + track.fractions[index + 1]);
        let [u, v] = foot(surface, at(fraction).or_refuse(KernelStage::Refine, "at")?, Some(track.feet[index]))?;
        checks.push((fraction, surface.evaluate(u, v).or_refuse(KernelStage::Refine, "evaluate")?));
    }
    let tangents: Vec<[[f64; 2]; 2]> = track
        .breaks
        .iter()
        .map(|&index| track_tangent(surface, track, at, index))
        .collect::<Result<_, _>>()?;
    let mut best: Option<TrackFit> = None;
    let mut sections = 8usize;
    loop {
        let sections_here = sections.min(last);
        let (curve, samples) = if track.breaks.is_empty() {
            let indices: Vec<usize> =
                (0..=sections_here).map(|k| (k * last / sections_here).min(last)).collect();
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
            fit_in_c1_pieces(track, &tangents, sections_here)?
        };
        let mut miss: f64 = 0.0;
        for (fraction, on_carrier) in &checks {
            let uv = curve.evaluate(*fraction).or_refuse(KernelStage::Refine, "evaluate")?;
            miss = miss.max(surface.evaluate(uv.x, uv.y).or_refuse(KernelStage::Refine, "evaluate")?.sub(*on_carrier).length());
        }
        if best.as_ref().map_or(true, |known| miss < known.miss) {
            best = Some(TrackFit {
                curve,
                miss,
                samples,
                on_floor: miss <= tolerance,
            });
        }
        if miss <= tolerance || sections_here == last {
            break;
        }
        sections *= 2;
    }
    best.ok_or_else(|| KernelRefusal::internal(KernelStage::Refine, "no_rung", "blend: the track fit produced no rung"))
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
/// the curve's own one-sided derivatives resolved onto the surface's first
/// derivatives there. The surface is C1 across a knot line, so at a surface
/// crossing the two sides agree and one tangent is returned for both.
fn track_tangent(
    surface: &NurbsSurface,
    track: &Track,
    at: &dyn Fn(f64) -> Result<Vec3, String>,
    index: usize,
) -> Result<[[f64; 2]; 2], KernelRefusal> {
    let fraction = track.fractions[index];
    // Second-order ONE-SIDED differences: a curve break may be a genuine corner,
    // where a central difference would average two tangents neither side has.
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
    let [u, v] = track.feet[index];
    let (_, su, sv) = surface.deriv1(u, v).or_refuse(KernelStage::Refine, "deriv1")?;
    let (a, b, c) = (su.dot(su), su.dot(sv), sv.dot(sv));
    let determinant = a * c - b * b;
    if !(determinant.abs() > 1e-300) {
        return Err(KernelRefusal::unsupported(
            KernelStage::Refine,
            "pole_tangent",
            "blend: the track tangent is undefined at a knot-line crossing (a pole)",
        ));
    }
    let resolve = |derivative: Vec3| {
        let (x, y) = (su.dot(derivative), sv.dot(derivative));
        [(c * x - b * y) / determinant, (a * y - b * x) / determinant]
    };
    let (left, right) = (resolve(left), resolve(right));
    // Where the two sides agree to the difference's own accuracy the track is
    // C1 and both pieces take ONE tangent, so the join is exactly C1.
    let scale = (left[0].hypot(left[1])).max(right[0].hypot(right[1])).max(1e-300);
    if (left[0] - right[0]).hypot(left[1] - right[1]) <= 1e-7 * scale {
        let mean = [0.5 * (left[0] + right[0]), 0.5 * (left[1] + right[1])];
        return Ok([mean, mean]);
    }
    Ok([left, right])
}

/// One rung with breaks: each piece interpolated through its share of samples
/// with the break tangents prescribed — the one tangent where the track is C1,
/// each side's own where the curve itself has a corner — joined at knots of
/// multiplicity 3 into one curve.
fn fit_in_c1_pieces(
    track: &Track,
    tangents: &[[[f64; 2]; 2]],
    sections: usize,
) -> Result<(NurbsCurve, usize), KernelRefusal> {
    let last = track.fractions.len() - 1;
    let mut bounds = vec![0usize];
    bounds.extend(track.breaks.iter().copied());
    bounds.push(last);
    let pieces = bounds.len() - 1;
    let mut knots: Vec<f64> = Vec::new();
    let mut control: Vec<Vec4> = Vec::new();
    let mut samples = 0usize;
    for piece in 0..pieces {
        let (from, to) = (bounds[piece], bounds[piece + 1]);
        let span = to - from;
        let share = ((sections as f64 * span as f64 / last as f64).ceil() as usize)
            .max(MIN_PIECE_INTERVALS)
            .min(span);
        let mut indices: Vec<usize> = (0..=share).map(|k| from + k * span / share).collect();
        indices.dedup();
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
