//! The FOLD LOCUS: the curve `1 − δ·κ = 0` in a surface's `(u, v)` domain,
//! traced rather than merely bracketed.
//!
//! `offset/regularity.rs` answers a yes/no question about a region — does the
//! equidistant surface fold anywhere in it — by censusing the sign of the fold
//! factor over a derived sample grid.  When a region is folded in PART, that
//! census brackets the boundary between the two halves on its own grid edges
//! and then throws the bracket away.  This module marches it out.
//!
//! # The field
//!
//! One scalar field, [`crate::offset_regularity::fold_sample_at`], shared with
//! the scan so the trace and the census cannot disagree about the same face:
//!
//! ```text
//! g(u, v) = min over the requested displacements δ and both principal
//!           curvatures κ of (1 − δ·κ)   −   level
//! ```
//!
//! with δ along the parametrization normal `Su × Sv` and κ signed against that
//! same normal, exactly as the scan states it.  `level` is the caller's: the
//! carve passes its own collapse factor so the kept side of the split is
//! precisely the side the census calls regular, and a closed-form test passes
//! `0.0` to get the textbook locus.
//!
//! # The march
//!
//! Seeds are sign changes on the SCAN'S OWN grid edges — [`region_grid`] is
//! extracted for that reason, so "the scan can already bracket the fold" is
//! literally true and not approximately.  Each seed is bisected onto `g = 0`,
//! then the curve is followed by predictor–corrector in a normalized `(û, v̂)`
//! box: predict along the tangent `(−g_v, g_u)`, correct back along `∇g` by
//! Newton.  Working in box-normalized coordinates is what makes one step size
//! meaningful on a domain whose two parameters have unrelated scales.
//!
//! An open branch is marched in both directions from its seed and ends on the
//! box boundary, where a final 1-D Newton along that boundary edge places the
//! endpoint exactly on the locus instead of wherever the last step landed.
//!
//! # What it refuses, and why that matters more than what it traces
//!
//! A fold locus that is silently wrong is a trim split in the wrong place,
//! which is a wrong solid that validates.  So every way of losing the curve is
//! a named refusal carrying the `(u, v)` where it was lost:
//!
//! * **stationary field** — `|∇g|` under the floor: a bracket led somewhere the
//!   level set has no direction to follow, so there is no curve there to
//!   march.  (A field that is constant over the WHOLE region, as a torus
//!   offset past its tube radius is on its meridian branch, brackets nothing in
//!   the first place and comes back empty — that is not this refusal.);
//! * **a jump, not a crossing** — the bisection converges on a parameter where
//!   `|g|` is still large.  A knot line across which the curvature jumps (a
//!   line → arc → line sheet) changes the sign of `g` without ever passing
//!   through zero, and marching such a "locus" would follow the knot line;
//! * **the corrector left the locus** — Newton did not return, after halving;
//! * **the march ran away** — neither closed nor left the domain in budget.
//!
//! None of these is downgraded to an empty result: an empty [`FoldLocus`] means
//! "there is no fold locus here", which is a different claim.

use crate::offset_regularity::{
    fold_sample_at, refine_between_nodes, region_grid, ScanBudget, TrimRegion,
};
use crate::{KernelRefusal, KernelStage, OrRefuse};
use crate::{interpolate_curve, interpolate_curve_closed, NurbsCurve, NurbsSurface, Vec3};

/// Newton's target on `g`. The field is dimensionless and O(1) near a fold, and
/// the gradient is the only finite-differenced quantity, so the corrector
/// reaches this on most geometry; it is a convergence target, not an accuracy
/// claim.
const CORRECTOR_TOLERANCE: f64 = 1e-11;
/// What the corrector accepts when Newton STALLS — the step stops moving before
/// the target is met.
///
/// `g` is read from a surface's second derivatives, and near a knot line of a
/// fitted patch those carry more relative noise than the target allows: on a
/// cubic B-spline cone the corrector sat at ~1e-10 and could not do better, and
/// refusing there would have thrown away a locus that is correct to ten digits.
/// A STALL at 1e-7 is convergence; it is nine decades away from the O(1)
/// residual a genuine discontinuity leaves, which is what the refusals below
/// are for.
const CORRECTOR_STALL_CEILING: f64 = 1e-7;
/// Parameter-space step, in box-normalized units, below which Newton has
/// stopped moving.
const CORRECTOR_STALL_STEP: f64 = 1e-13;
/// `|∇g|` in box-normalized units below which the field is flat enough that a
/// tangent direction is meaningless.
const GRADIENT_FLOOR: f64 = 1e-9;
/// Central-difference half-step, in box-normalized units.
const DERIVATIVE_STEP: f64 = 1e-6;
const CORRECTOR_ITERATIONS: usize = 16;
const PREDICTOR_HALVINGS: usize = 6;
const MAX_MARCH_STEPS: usize = 20_000;
/// Points kept when a marched branch is turned into a pcurve.
const MAX_PCURVE_POINTS: usize = 128;

/// The offset family's own trace switch, the same `BREP_OS_DEBUG` the scan reads.
fn debug_enabled() -> bool {
    std::env::var("BREP_OS_DEBUG").is_ok_and(|value| !value.is_empty() && value != "0")
}

/// One traced branch of the fold locus, as a `(u, v)` polyline in the carrier's
/// own parameter domain.
#[derive(Clone, Debug)]
pub(crate) struct FoldCurve {
    /// Marched points, ordered along the curve. A closed branch does NOT repeat
    /// its first point.
    pub(crate) points: Vec<[f64; 2]>,
    /// A branch that came back to its seed, as opposed to one that left the
    /// region's parameter box through two boundary edges.
    pub(crate) closed: bool,
}

/// The residual an interpolated locus pcurve is refined down to, measured in
/// the fold factor at the interpolant's own span midpoints.
const PCURVE_TARGET_RESIDUAL: f64 = 1e-9;
/// Ceiling on the stations a refinement may add.
const MAX_REFINED_STATIONS: usize = 512;
const REFINEMENT_ROUNDS: usize = 8;

/// Newton a `(u, v)` back onto `1 − δ·κ = level`, in the surface's OWN
/// parameters rather than a normalized box — the refinement below works on
/// stations that already live in `(u, v)`.
fn correct_uv(
    surface: &NurbsSurface,
    displacements: &[f64],
    level: f64,
    start: [f64; 2],
    bounds: [f64; 4],
) -> Option<[f64; 2]> {
    let step_u = (bounds[1] - bounds[0]) * DERIVATIVE_STEP;
    let step_v = (bounds[3] - bounds[2]) * DERIVATIVE_STEP;
    let at = |point: [f64; 2]| {
        fold_sample_at(surface, point[0], point[1], displacements)
            .map(|sample| sample.factor - level)
    };
    let scale_u = (bounds[1] - bounds[0]).max(1e-300);
    let scale_v = (bounds[3] - bounds[2]).max(1e-300);
    let mut point = start;
    for _ in 0..CORRECTOR_ITERATIONS {
        let value = at(point)?;
        if value.abs() <= CORRECTOR_TOLERANCE {
            return Some(point);
        }
        let mut gradient = [0.0; 2];
        for (axis, step) in [step_u, step_v].into_iter().enumerate() {
            let mut lo = point;
            let mut hi = point;
            lo[axis] = (point[axis] - step).max(bounds[2 * axis]);
            hi[axis] = (point[axis] + step).min(bounds[2 * axis + 1]);
            let span = hi[axis] - lo[axis];
            if span <= 0.0 {
                return None;
            }
            gradient[axis] = (at(hi)? - at(lo)?) / span;
        }
        let square = gradient[0] * gradient[0] + gradient[1] * gradient[1];
        if square <= 0.0 {
            return None;
        }
        let factor = value / square;
        let next = [
            (point[0] - factor * gradient[0]).clamp(bounds[0], bounds[1]),
            (point[1] - factor * gradient[1]).clamp(bounds[2], bounds[3]),
        ];
        let moved =
            ((next[0] - point[0]) / scale_u).hypot((next[1] - point[1]) / scale_v);
        point = next;
        if moved <= CORRECTOR_STALL_STEP {
            break;
        }
    }
    at(point)
        .filter(|value| value.abs() <= CORRECTOR_STALL_CEILING)
        .map(|_| point)
}

/// Interpolate a pcurve through marched stations and REFINE it until the curve
/// ITSELF names the locus, not just the stations it was built from.
///
/// Returns the curve and the worst residual measured on it, in the fold factor.
///
/// This is not polish.  The interpolant is what becomes a trim boundary, and a
/// consumer that re-reads the field along it — the carve's own verification
/// does exactly that — sees the interpolant, not the march.  On a fitted ridge
/// the first fit sagged 1.7e-4 in the factor between stations the march had
/// placed to 1e-11, which is two decades worse than the curve it claims to be
/// and enough to put a strip of folded parameter back inside the kept region.
/// So: measure at every span midpoint, insert a Newton-corrected station in
/// each span that misses, repeat.  The residual is RETURNED rather than
/// asserted, because the caller is the one that knows what it has to be good
/// enough for.
pub(crate) fn fit_locus_pcurve(
    surface: &NurbsSurface,
    displacements: &[f64],
    level: f64,
    stations: &[[f64; 2]],
    closed: bool,
) -> Result<(NurbsCurve, f64), KernelRefusal> {
    let floor = if closed { 4 } else { 2 };
    if stations.len() < floor {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "fold_locus_not_a_curve", format!(
            "fold locus: a {} branch of {} point(s) is not a curve",
            if closed { "closed" } else { "open" },
            stations.len()
        )));
    }
    let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain_u")?;
    let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain_v")?;
    let bounds = [u0, u1, v0, v1];

    // COARSE FIRST. The march places a station every half grid cell because that
    // is what keeps it on the curve; a pcurve needs however many the
    // INTERPOLANT needs, which is usually far fewer. Every fold locus on a
    // surface of revolution is an iso-line, and a cubic through five collinear
    // stations reproduces it exactly while a cubic through sixty-three carries
    // sixty-three control points into every image fit downstream. Start at five
    // and double only while the measurement says to.
    let mut count = floor.max(5).min(stations.len());
    let mut chosen = subsample(stations, count, closed);
    let mut curve = interpolate_stations(&chosen, closed)?;
    let mut residual =
        measure_against_locus(surface, displacements, level, &curve, &chosen, closed)?;
    while residual > PCURVE_TARGET_RESIDUAL && count < stations.len() {
        count = (count * 2).min(stations.len());
        chosen = subsample(stations, count, closed);
        curve = interpolate_stations(&chosen, closed)?;
        residual =
            measure_against_locus(surface, displacements, level, &curve, &chosen, closed)?;
    }

    // Past the march's own stations, refine by halving every span and pulling
    // the new station onto the locus with Newton.
    for _ in 0..REFINEMENT_ROUNDS {
        if residual <= PCURVE_TARGET_RESIDUAL || chosen.len() * 2 > MAX_REFINED_STATIONS {
            break;
        }
        // UNIFORM subdivision, not insertion where it misses. Refining only the
        // spans that miss is the obvious move and it makes the fit WORSE: a
        // chord-length interpolant through spans of two different lengths rings
        // across the join, which is the same defect a near-duplicate station
        // causes. Measured on a fitted ridge, selective insertion took 1.7e-4 to
        // 5.5e-4. Halving every span keeps the parameterization even, so the
        // error falls with the span like the interpolation order says it should.
        let parameters = station_parameters(&chosen, closed);
        let spans = if closed { chosen.len() } else { chosen.len() - 1 };
        let mut refined = Vec::with_capacity(chosen.len() * 2);
        for span in 0..spans {
            refined.push(chosen[span]);
            let middle = 0.5 * (parameters[span] + parameters[span + 1]);
            let point = curve.evaluate(middle).or_refuse(KernelStage::Refine, "evaluate")?;
            let seed = [point.x, point.y];
            if let Some(corrected) = correct_uv(surface, displacements, level, seed, bounds) {
                refined.push(corrected);
            }
        }
        if !closed {
            refined.push(chosen[chosen.len() - 1]);
        }
        chosen = refined;
        curve = interpolate_stations(&chosen, closed)?;
        residual =
            measure_against_locus(surface, displacements, level, &curve, &chosen, closed)?;
    }
    if debug_enabled() {
        eprintln!(
            "fold locus: pcurve over {} stations (the march placed {}) names the locus to \
             {residual:.3e}",
            chosen.len(),
            stations.len()
        );
    }
    Ok((curve, residual))
}

/// `count` stations spread evenly over the branch, both ends included (an open
/// branch's two ends are its exact boundary points and a closed one's first
/// station closes the loop).
fn subsample(stations: &[[f64; 2]], count: usize, closed: bool) -> Vec<[f64; 2]> {
    if count >= stations.len() {
        return stations.to_vec();
    }
    if closed {
        let step = stations.len() as f64 / count as f64;
        return (0..count)
            .map(|index| stations[((index as f64) * step) as usize])
            .collect();
    }
    let last = stations.len() - 1;
    let step = last as f64 / (count - 1) as f64;
    (0..count)
        .map(|index| stations[((index as f64) * step).round().min(last as f64) as usize])
        .collect()
}

/// The worst `|1 − δ·κ − level|` the interpolant itself reads, sampled between
/// the stations it was built from — where an interpolant is free to wander.
fn measure_against_locus(
    surface: &NurbsSurface,
    displacements: &[f64],
    level: f64,
    curve: &NurbsCurve,
    stations: &[[f64; 2]],
    closed: bool,
) -> Result<f64, KernelRefusal> {
    let parameters = station_parameters(stations, closed);
    let spans = if closed { stations.len() } else { stations.len() - 1 };
    let mut worst = 0.0f64;
    let mut worst_span = 0usize;
    let mut worst_point = [0.0f64; 2];
    for span in 0..spans {
        for fraction in [0.25, 0.5, 0.75] {
            let t = parameters[span] + (parameters[span + 1] - parameters[span]) * fraction;
            let point = curve.evaluate(t).or_refuse(KernelStage::Refine, "evaluate")?;
            if let Some(sample) = fold_sample_at(surface, point.x, point.y, displacements) {
                let miss = (sample.factor - level).abs();
                if miss > worst {
                    worst = miss;
                    worst_span = span;
                    worst_point = [point.x, point.y];
                }
            }
        }
    }
    if debug_enabled() {
        let a = stations[worst_span];
        let b = stations[(worst_span + 1) % stations.len()];
        eprintln!(
            "fold locus: worst span {worst_span}/{spans} = {worst:.3e} at ({:.9},{:.9}); \
             stations ({:.9},{:.9}) → ({:.9},{:.9}), span parameter width {:.3e}",
            worst_point[0],
            worst_point[1],
            a[0],
            a[1],
            b[0],
            b[1],
            parameters[worst_span + 1] - parameters[worst_span]
        );
    }
    Ok(worst)
}

/// Chord-length parameters for the stations; a closed run carries one extra,
/// naming where the first station comes round again.
fn station_parameters(stations: &[[f64; 2]], closed: bool) -> Vec<f64> {
    let count = stations.len();
    let spans = if closed { count } else { count - 1 };
    let mut parameters = Vec::with_capacity(spans + 1);
    let mut total = 0.0;
    parameters.push(0.0);
    for span in 0..spans {
        let a = stations[span];
        let b = stations[(span + 1) % count];
        total += (b[0] - a[0]).hypot(b[1] - a[1]).max(1e-12);
        parameters.push(total);
    }
    parameters
}

fn interpolate_stations(stations: &[[f64; 2]], closed: bool) -> Result<NurbsCurve, KernelRefusal> {
    let points = stations
        .iter()
        .map(|point| Vec3::new(point[0], point[1], 0.0))
        .collect::<Vec<_>>();
    let parameters = station_parameters(stations, closed);
    if closed {
        return interpolate_curve_closed(&points, &parameters).or_refuse(KernelStage::Refine, "interpolate_curve_closed");
    }
    interpolate_curve(&points, 3.min(points.len() - 1), &parameters).or_refuse(KernelStage::Refine, "interpolate_curve")
}

impl FoldCurve {
    /// The branch as a parameter-space curve, in the `(u, v, 0)` convention
    /// every pcurve in the kernel uses, plus the residual the fit reports
    /// against the locus equation itself.
    ///
    /// Thinned to [`MAX_PCURVE_POINTS`] stations before interpolation: the
    /// march's step is set by the fold's own scale and produces far more points
    /// than a pcurve needs, and an interpolant through thousands of nearly
    /// collinear stations is numerically worse, not better.  Whatever the
    /// thinning costs, [`fit_locus_pcurve`] measures and puts back.
    pub(crate) fn pcurve(
        &self,
        surface: &NurbsSurface,
        displacements: &[f64],
        level: f64,
    ) -> Result<(NurbsCurve, f64), KernelRefusal> {
        fit_locus_pcurve(
            surface,
            displacements,
            level,
            &self.thinned(),
            self.closed,
        )
    }

    fn thinned(&self) -> Vec<[f64; 2]> {
        if self.points.len() <= MAX_PCURVE_POINTS {
            return self.points.clone();
        }
        let step = (self.points.len() - 1) as f64 / (MAX_PCURVE_POINTS - 1) as f64;
        let mut thinned = (0..MAX_PCURVE_POINTS)
            .map(|index| self.points[((index as f64) * step).round() as usize])
            .collect::<Vec<_>>();
        if self.closed {
            // The closed interpolant wraps on its own; a duplicated last
            // station would be a zero-length span.
            if thinned.len() > 1 && thinned[0] == thinned[thinned.len() - 1] {
                thinned.pop();
            }
        }
        thinned
    }

    /// Whether any station of this branch lies inside `region` — a branch of
    /// the locus can live entirely in the part of the parameter BOX the trim
    /// does not cover.
    pub(crate) fn meets(&self, region: &TrimRegion) -> bool {
        self.points
            .iter()
            .any(|point| region.contains(point[0], point[1]))
    }
}

/// Every branch of the fold locus over a region's parameter box.
#[derive(Clone, Debug, Default)]
pub(crate) struct FoldLocus {
    pub(crate) curves: Vec<FoldCurve>,
    /// The grid edges that bracketed a crossing, before deduplication — what
    /// the scan saw. A trace that produces no curve from a non-zero bracket
    /// count is a bug, not an answer, and the carve checks exactly that.
    pub(crate) brackets: usize,
}

/// The field, plus the box it is normalized against.
struct Field<'a> {
    surface: &'a NurbsSurface,
    displacements: &'a [f64],
    level: f64,
    bounds: [f64; 4],
}

impl Field<'_> {
    fn to_uv(&self, point: [f64; 2]) -> [f64; 2] {
        [
            self.bounds[0] + point[0] * (self.bounds[1] - self.bounds[0]),
            self.bounds[2] + point[1] * (self.bounds[3] - self.bounds[2]),
        ]
    }

    fn to_box(&self, uv: [f64; 2]) -> [f64; 2] {
        [
            (uv[0] - self.bounds[0]) / (self.bounds[1] - self.bounds[0]),
            (uv[1] - self.bounds[2]) / (self.bounds[3] - self.bounds[2]),
        ]
    }

    /// `g` at a box-normalized point, `None` where the curvature is unreadable.
    fn at(&self, point: [f64; 2]) -> Option<f64> {
        let uv = self.to_uv(point);
        fold_sample_at(self.surface, uv[0], uv[1], self.displacements)
            .map(|sample| sample.factor - self.level)
    }

    /// Central differences in box-normalized coordinates, one-sided where the
    /// step would leave the box.
    fn gradient(&self, point: [f64; 2]) -> Option<[f64; 2]> {
        let mut gradient = [0.0; 2];
        for axis in 0..2 {
            let mut lo = point;
            let mut hi = point;
            lo[axis] = (point[axis] - DERIVATIVE_STEP).max(0.0);
            hi[axis] = (point[axis] + DERIVATIVE_STEP).min(1.0);
            let span = hi[axis] - lo[axis];
            if span <= 0.0 {
                return None;
            }
            gradient[axis] = (self.at(hi)? - self.at(lo)?) / span;
        }
        Some(gradient)
    }

    /// Newton back onto `g = 0` along `∇g`.
    ///
    /// Converged means the target is met OR the step has stalled with the
    /// residual already under [`CORRECTOR_STALL_CEILING`] — see that constant
    /// for why a stall is an answer and not a failure.
    fn correct(&self, start: [f64; 2]) -> Option<[f64; 2]> {
        let mut point = start;
        for _ in 0..CORRECTOR_ITERATIONS {
            let value = self.at(point)?;
            if value.abs() <= CORRECTOR_TOLERANCE {
                return Some(point);
            }
            let gradient = self.gradient(point)?;
            let square = gradient[0] * gradient[0] + gradient[1] * gradient[1];
            if square <= GRADIENT_FLOOR * GRADIENT_FLOOR {
                return None;
            }
            let scale = value / square;
            let next = [
                (point[0] - scale * gradient[0]).clamp(0.0, 1.0),
                (point[1] - scale * gradient[1]).clamp(0.0, 1.0),
            ];
            let moved = (next[0] - point[0]).hypot(next[1] - point[1]);
            point = next;
            if moved <= CORRECTOR_STALL_STEP {
                break;
            }
        }
        self.at(point)
            .filter(|value| value.abs() <= CORRECTOR_STALL_CEILING)
            .map(|_| point)
    }

    fn report(&self, point: [f64; 2]) -> String {
        let uv = self.to_uv(point);
        format!("(u={:.6}, v={:.6})", uv[0], uv[1])
    }
}

/// Bisect a bracketed segment onto `g = 0`.
///
/// Returns `Err` when the bracket is a JUMP rather than a crossing: the segment
/// shrinks to machine width with `|g|` still large on both sides, which is what
/// a knot line carrying a curvature discontinuity does to a sign test.
fn bisect(field: &Field, mut lo: [f64; 2], mut hi: [f64; 2]) -> Result<[f64; 2], KernelRefusal> {
    let (Some(mut value_lo), Some(mut value_hi)) = (field.at(lo), field.at(hi)) else {
        return Err(KernelRefusal::internal(KernelStage::Refine, "fold_factor_unreadable", format!(
            "fold locus: the fold factor is unreadable between {} and {}",
            field.report(lo),
            field.report(hi)
        )));
    };
    for _ in 0..80 {
        let mid = [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5];
        let Some(value_mid) = field.at(mid) else {
            return Err(KernelRefusal::internal(KernelStage::Refine, "fold_factor_unreadable", format!(
                "fold locus: the fold factor is unreadable at {}",
                field.report(mid)
            )));
        };
        if value_mid.abs() <= CORRECTOR_TOLERANCE {
            return Ok(mid);
        }
        if (value_mid < 0.0) == (value_lo < 0.0) {
            lo = mid;
            value_lo = value_mid;
        } else {
            hi = mid;
            value_hi = value_mid;
        }
        let width = (hi[0] - lo[0]).abs().max((hi[1] - lo[1]).abs());
        if width <= 1e-15 {
            break;
        }
    }
    let residual = value_lo.abs().min(value_hi.abs());
    if residual > CORRECTOR_STALL_CEILING {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "fold_factor_jump", format!(
            "fold locus: the fold factor JUMPS across {} (from {:+.6e} to {:+.6e} over a \
             machine-width interval) — that sign change is a curvature discontinuity, a knot \
             line the surface is only C⁰ across, not a crossing of 1 − δ·κ = 0",
            field.report(lo),
            value_lo,
            value_hi
        )));
    }
    Ok(if value_lo.abs() <= value_hi.abs() { lo } else { hi })
}

/// Trace every branch of `1 − δ·κ = level` over `region`'s parameter box.
///
/// `budget` is the caller's own scan budget, so the bracket population is the
/// one its census already saw.
pub(crate) fn trace_fold_locus(
    surface: &NurbsSurface,
    region: &TrimRegion,
    displacements: &[f64],
    level: f64,
    budget: ScanBudget,
) -> Result<FoldLocus, KernelRefusal> {
    let bounds = region.bounds();
    if !(bounds[1] > bounds[0]) || !(bounds[3] > bounds[2]) {
        return Err(KernelRefusal::internal(KernelStage::Refine, "fold_region_box", "fold locus: the region's parameter box is degenerate"));
    }
    let reach = displacements
        .iter()
        .fold(0.0f64, |worst, distance| worst.max(distance.abs()));
    if reach == 0.0 || surface.is_affine().or_refuse(KernelStage::Refine, "is_affine")? {
        return Ok(FoldLocus::default());
    }
    let field = Field {
        surface,
        displacements,
        level,
        bounds,
    };
    let grid = region_grid(surface, bounds, reach, budget);
    if grid.samples_u.len() < 2 || grid.samples_v.len() < 2 {
        return Err(KernelRefusal::unsupported(KernelStage::Refine, "fold_grid_coarse", format!(
            "fold locus: the region's sample grid is {}×{} — too coarse to bracket a fold",
            grid.samples_u.len(),
            grid.samples_v.len()
        )));
    }
    let nodes: Vec<Vec<[f64; 2]>> = grid
        .samples_u
        .iter()
        .map(|&u| {
            grid.samples_v
                .iter()
                .map(|&v| field.to_box([u, v]))
                .collect()
        })
        .collect();
    let values: Vec<Vec<Option<f64>>> = nodes
        .iter()
        .map(|column| column.iter().map(|point| field.at(*point)).collect())
        .collect();

    // Every grid edge that changes sign — the scan's own brackets.
    let mut seeds = Vec::new();
    let crossed = |a: (usize, usize), b: (usize, usize), seeds: &mut Vec<([f64; 2], [f64; 2])>| {
        let (Some(first), Some(second)) = (values[a.0][a.1], values[b.0][b.1]) else {
            return;
        };
        if (first < 0.0) != (second < 0.0) {
            seeds.push((nodes[a.0][a.1], nodes[b.0][b.1]));
        }
    };
    for iu in 0..nodes.len() {
        for iv in 0..nodes[iu].len() {
            if iu + 1 < nodes.len() {
                crossed((iu, iv), (iu + 1, iv), &mut seeds);
            }
            if iv + 1 < nodes[iu].len() {
                crossed((iu, iv), (iu, iv + 1), &mut seeds);
            }
        }
    }
    // And every sign change BETWEEN nodes the scan's refinement finds — a fold
    // island narrower than the node spacing brackets no grid edge at all, and
    // the scan that asked for this trace already counted it.
    let between = refine_between_nodes(
        surface,
        region,
        &grid.samples_u,
        &grid.samples_v,
        &values,
        displacements,
        level,
    );
    seeds.extend(
        between
            .brackets
            .iter()
            .map(|(inside, corner)| (field.to_box(*inside), field.to_box(*corner))),
    );
    let mut locus = FoldLocus {
        curves: Vec::new(),
        brackets: seeds.len(),
    };
    if seeds.is_empty() {
        return Ok(locus);
    }

    // One march step: half the finer grid cell, so a branch is sampled at least
    // twice per cell the census read, and never so fine that a long locus
    // exhausts the step budget.
    let cell_u = 1.0 / (nodes.len() - 1) as f64;
    let cell_v = 1.0 / (nodes[0].len() - 1) as f64;
    let step = (cell_u.min(cell_v) * 0.5).clamp(1e-5, 0.05);

    for (lo, hi) in seeds {
        let seed = bisect(&field, lo, hi)?;
        if locus
            .curves
            .iter()
            .any(|curve| curve.passes_within(&field, seed, step))
        {
            continue;
        }
        locus.curves.push(march(&field, seed, step)?);
    }
    Ok(locus)
}

impl FoldCurve {
    /// Whether an already-traced branch passes within `radius` (box-normalized)
    /// of a candidate seed — the deduplication that keeps one branch from being
    /// re-marched once per grid edge it crosses.
    fn passes_within(&self, field: &Field, seed: [f64; 2], radius: f64) -> bool {
        let boxed = self
            .points
            .iter()
            .map(|point| field.to_box(*point))
            .collect::<Vec<_>>();
        // Against the SEGMENTS, not only the stations: a branch marched with a
        // long step passes a seed at up to half a step from its nearest
        // station, and the station test alone would re-march the same branch
        // once per grid edge it crosses.
        boxed.windows(2).any(|pair| {
            let (a, b) = (pair[0], pair[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let square = dx * dx + dy * dy;
            let t = if square <= 0.0 {
                0.0
            } else {
                (((seed[0] - a[0]) * dx + (seed[1] - a[1]) * dy) / square).clamp(0.0, 1.0)
            };
            (a[0] + t * dx - seed[0]).hypot(a[1] + t * dy - seed[1]) <= radius
        }) || boxed
            .iter()
            .any(|point| (point[0] - seed[0]).hypot(point[1] - seed[1]) <= radius)
    }
}

/// Follow the branch through `seed`, forward then backward.
fn march(field: &Field, seed: [f64; 2], step: f64) -> Result<FoldCurve, KernelRefusal> {
    let (forward, closed) = follow(field, seed, step, 1.0)?;
    if closed {
        return Ok(FoldCurve {
            points: forward.iter().map(|point| field.to_uv(*point)).collect(),
            closed: true,
        });
    }
    let (backward, _) = follow(field, seed, step, -1.0)?;
    // backward[0] is the seed, shared with forward[0]; drop it once.
    let mut points = backward;
    points.reverse();
    points.pop();
    points.extend(forward);
    Ok(FoldCurve {
        points: points.iter().map(|point| field.to_uv(*point)).collect(),
        closed: false,
    })
}

/// One direction of the march. Returns the points from `seed` onward and
/// whether the branch closed back on its seed.
fn follow(
    field: &Field,
    seed: [f64; 2],
    step: f64,
    direction: f64,
) -> Result<(Vec<[f64; 2]>, bool), KernelRefusal> {
    let mut points = vec![seed];
    let mut previous_tangent: Option<[f64; 2]> = None;
    for _ in 0..MAX_MARCH_STEPS {
        let current = *points.last().unwrap();
        let gradient = field.gradient(current).ok_or_else(|| {
            format!(
                "fold locus: the fold factor is unreadable at {}",
                field.report(current)
            )
        }).or_refuse(KernelStage::Refine, "fold_factor_unreadable")?;
        let length = gradient[0].hypot(gradient[1]);
        if length <= GRADIENT_FLOOR {
            return Err(KernelRefusal::unsupported(KernelStage::Refine, "fold_stationary", format!(
                "fold locus: the fold factor is STATIONARY at {} (|∇(1 − δ·κ)| = {length:.3e} \
                 over the region's own parameter box) — the fold is not a curve there. A torus \
                 offset past its tube radius does this: the meridian branch 1 − δ/r is constant, \
                 so the region folds everywhere or nowhere and there is nothing to split",
                field.report(current)
            )));
        }
        // Tangent to the level set, oriented to continue the march rather than
        // double back at an inflection.
        let mut tangent = [
            -gradient[1] / length * direction,
            gradient[0] / length * direction,
        ];
        if let Some(previous) = previous_tangent {
            if tangent[0] * previous[0] + tangent[1] * previous[1] < 0.0 {
                tangent = [-tangent[0], -tangent[1]];
            }
        }

        let mut attempt = step;
        let mut advanced = None;
        for _ in 0..PREDICTOR_HALVINGS {
            let predicted = [
                current[0] + tangent[0] * attempt,
                current[1] + tangent[1] * attempt,
            ];
            // `<=` / `>=`, not `<` / `>`: a predictor that lands exactly ON a
            // wall must leave through it here, while the current point is
            // still strictly inside. Marching onto the wall first and asking
            // afterwards leaves `leave_box` with a zero distance to the wall
            // it is actually crossing.
            if predicted[0] <= 0.0
                || predicted[0] >= 1.0
                || predicted[1] <= 0.0
                || predicted[1] >= 1.0
            {
                let exit = leave_box(field, current, tangent)?;
                // A STUB last span is worse than no last span. The exit can
                // land a small fraction of a step from the station before it,
                // and a chord-length interpolant through stations that are
                // evenly spaced except for one short final gap oscillates
                // across that gap — measured at 1.7e-4 in the fold factor on
                // the fitted ridge, two decades worse than the march itself.
                // Replace the last station instead of appending after it.
                let previous = *points.last().unwrap();
                let gap = (exit[0] - previous[0]).hypot(exit[1] - previous[1]);
                if gap < attempt * 0.3 && points.len() >= 2 {
                    points.pop();
                }
                points.push(exit);
                return Ok((points, false));
            }
            if let Some(corrected) = field.correct(predicted) {
                // A corrector that lands further than it predicted has jumped
                // onto another branch; halve instead of accepting it.
                let drift = (corrected[0] - predicted[0]).hypot(corrected[1] - predicted[1]);
                if drift <= attempt {
                    advanced = Some(corrected);
                    break;
                }
            }
            attempt *= 0.5;
        }
        let Some(next) = advanced else {
            return Err(KernelRefusal::non_convergence(KernelStage::Refine, "fold_corrector", format!(
                "fold locus: the corrector did not return to 1 − δ·κ = {:.3e} beyond {} after {} \
                 halvings of a {:.3e} step",
                field.level,
                field.report(current),
                PREDICTOR_HALVINGS,
                step
            )));
        };
        previous_tangent = Some(tangent);
        // Closure: back within one step of the seed, having gone far enough
        // away first that this is a return and not the first step's neighbour.
        if points.len() >= 4 && (next[0] - seed[0]).hypot(next[1] - seed[1]) <= attempt {
            return Ok((points, true));
        }
        points.push(next);
    }
    Err(KernelRefusal::non_convergence(KernelStage::Refine, "fold_march", format!(
        "fold locus: the march neither closed nor left the region's parameter box within {} \
         steps — lost at {}",
        MAX_MARCH_STEPS,
        field.report(*points.last().unwrap())
    )))
}

/// The point where the branch leaves the parameter box, placed exactly on the
/// locus by a 1-D Newton along the boundary edge it leaves through.
///
/// Bisecting the predictor step alone would put the endpoint on the box but
/// only within a step of the locus; the trim split needs a boundary point the
/// locus actually passes through, because that point becomes a VERTEX of the
/// carved loop.
fn leave_box(field: &Field, from: [f64; 2], tangent: [f64; 2]) -> Result<[f64; 2], KernelRefusal> {
    // Distance to each of the four walls along the tangent; the nearest is the
    // one the branch leaves through.
    //
    // The DIRECTION FLOOR is not defensive tidying. A locus that runs exactly
    // along an iso-line — every fold locus on a surface of revolution does —
    // has one tangent component that is analytically zero and numerically
    // ~1e-11, the roundoff of two curvature evaluations. Without the floor that
    // component names a wall 1e10 steps away, which beats a genuine wall the
    // march is a hundredth of a step from, and the exit lands in a domain
    // CORNER instead of on the locus. Measured on a 45° cone: the branch ended
    // at (u=0, v=0) reading a fold factor of −7.
    let dominant = tangent[0].abs().max(tangent[1].abs());
    let mut best: Option<(f64, usize, f64)> = None;
    for (axis, &component) in tangent.iter().enumerate() {
        if component.abs() <= 1e-9 * dominant {
            continue;
        }
        for wall in [0.0f64, 1.0f64] {
            let distance = (wall - from[axis]) / component;
            if distance < 0.0 {
                continue;
            }
            if best.is_none_or(|(current, _, _)| distance < current) {
                best = Some((distance, axis, wall));
            }
        }
    }
    let Some((distance, axis, wall)) = best else {
        return Err(KernelRefusal::internal(KernelStage::Refine, "fold_march_exit", format!(
            "fold locus: the march left the parameter box at {} along no axis",
            field.report(from)
        )));
    };
    let free = 1 - axis;
    let mut exit = [0.0; 2];
    exit[axis] = wall;
    exit[free] = (from[free] + tangent[free] * distance).clamp(0.0, 1.0);
    // Newton in the ONE remaining freedom, along the wall.
    for _ in 0..CORRECTOR_ITERATIONS {
        let Some(value) = field.at(exit) else { break };
        if value.abs() <= CORRECTOR_TOLERANCE {
            return Ok(exit);
        }
        let mut lo = exit;
        let mut hi = exit;
        lo[free] = (exit[free] - DERIVATIVE_STEP).max(0.0);
        hi[free] = (exit[free] + DERIVATIVE_STEP).min(1.0);
        let span = hi[free] - lo[free];
        let (Some(value_lo), Some(value_hi)) = (field.at(lo), field.at(hi)) else {
            break;
        };
        let slope = (value_hi - value_lo) / span.max(1e-300);
        if slope.abs() <= GRADIENT_FLOOR {
            break;
        }
        exit[free] = (exit[free] - value / slope).clamp(0.0, 1.0);
    }
    // The wall Newton is a refinement, not a requirement: a branch tangent to
    // the wall has no crossing along it and the geometric exit point stands.
    Ok(exit)
}

