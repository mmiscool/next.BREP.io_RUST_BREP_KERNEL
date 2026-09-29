//! CENSUS ONLY (`BREP_FIT_CENSUS=1`, observation, no behaviour change): the
//! two straightness gates in front of the exact intersection lanes, counted
//! and compared per pair.
//!
//! `planar_linear_iso_intersection` and `intersect_analytic_pair`'s generatrix
//! gate each run one of two predicates — the count proxy ([`IsoGate::Count`],
//! [`crate::RuledGate::Count`]) or the geometry the lane consumes
//! (`Geometry`). For every pair that reaches the iso lane the census works out
//! which lane the pair takes under BOTH trees — every gate by count, every
//! gate by geometry — and prints
//!
//! ```text
//! FITCENSUS lane id=N faces=AxB production=L count=L geometry=L
//! ```
//!
//! beside the per-gate lines explaining any disagreement. Where the boolean
//! MARCHES a pair that the other tree answers exactly, the marched lane runs
//! as it always does, and the curves it fits are compared against the exact
//! ones as POINT SETS when the pair is done:
//!
//! ```text
//! FITCENSUS compare id=N lane=L fit_tolerance=… exact_to_marched=… marched_to_exact=…
//! ```
//!
//! — both directions, with each curve's distance from both carriers so that a
//! disagreement names which curve is off. The four on-carrier columns carry an
//! `unprojected=a/b/c/d` beside them, in their own order, counting the points
//! whose projection the kernel refused: an off column is only as wide as the
//! points it managed to measure, and a `NaN` there means it measured none of
//! them (see [`max_measured`]).
//!
//! Running the census with the gates set to the count tree is how a lane change
//! is compared; running it on the geometry tree counts the lanes the boolean
//! takes.

use super::*;
use crate::{
    intersect_analytic_pair_with, ruled_gate, ruled_gate_census, KernelRefusal, PolylineFit,
    PolylineFitReport,
    RuledGate,
};
use std::sync::atomic::{AtomicUsize, Ordering};

pub(super) fn fit_census_enabled() -> bool {
    std::env::var_os("BREP_FIT_CENSUS").is_some()
}

/// The running thread's name, which is the test's name under the test
/// harness — so a line from a parallel run names its fixture.
fn thread_name() -> String {
    std::thread::current().name().unwrap_or("unnamed").replace(' ', "_")
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Lane {
    PlanarIso,
    Analytic,
    Marched,
}

impl Lane {
    fn label(self) -> &'static str {
        match self {
            Lane::PlanarIso => "planar_iso",
            Lane::Analytic => "analytic",
            Lane::Marched => "marched",
        }
    }
}

/// The lane and exact curves one tree of gates gives a pair.
fn tree_lane(
    first: &NurbsSurface,
    second: &NurbsSurface,
    tolerance: f64,
    iso: IsoGate,
    ruled: RuledGate,
) -> Result<(Lane, Vec<NurbsCurve>), KernelRefusal> {
    if let Some(curve) = planar_iso_intersection_with(first, second, tolerance, iso)? {
        return Ok((Lane::PlanarIso, vec![curve]));
    }
    match intersect_analytic_pair_with(first, second, tolerance, ruled) {
        Some(curves) => Ok((Lane::Analytic, curves)),
        None => Ok((Lane::Marched, Vec::new())),
    }
}

/// One pair's census, held for as long as the boolean works on the pair. When
/// it goes out of scope — the pair finished, skipped or refused — it prints the
/// point-set comparison of whatever the marched lane fitted.
pub(super) struct PairCensus<'a> {
    id: usize,
    first: TaggedFace<'a>,
    second: TaggedFace<'a>,
    fit_tolerance: f64,
    lane: Lane,
    exact: Vec<NurbsCurve>,
    stations: Vec<Vec<Vec3>>,
    fitted: Vec<(NurbsCurve, PolylineFitReport)>,
}

/// Count the pair's lanes under both trees and print them. Returns a census
/// to fill in from the marched lane when the boolean marches a pair that one
/// of the trees answers exactly.
pub(super) fn census_pair<'a>(
    first: TaggedFace<'a>,
    second: TaggedFace<'a>,
    tolerance: f64,
) -> Result<Option<PairCensus<'a>>, KernelRefusal> {
    static NEXT_ID: AtomicUsize = AtomicUsize::new(1);
    let (a, b) = (&first.face.surface, &second.face.surface);
    let (production, _) = tree_lane(a, b, tolerance, iso_gate(), ruled_gate())?;
    let (count, count_curves) = tree_lane(a, b, tolerance, IsoGate::Count, RuledGate::Count)?;
    let (geometry, geometry_curves) =
        tree_lane(a, b, tolerance, IsoGate::Geometry, RuledGate::Geometry)?;
    let id = NEXT_ID.fetch_add(1, Ordering::Relaxed);
    let same_curves = count_curves.len() == geometry_curves.len()
        && count_curves
            .iter()
            .zip(&geometry_curves)
            .all(|(x, y)| curves_bitwise_equal(x, y));
    eprintln!(
        "FITCENSUS lane id={id} faces={}x{} production={} count={} geometry={} \
         count_curves={} geometry_curves={} same_curves={same_curves} thread={}",
        first.face.id,
        second.face.id,
        production.label(),
        count.label(),
        geometry.label(),
        count_curves.len(),
        geometry_curves.len(),
        thread_name(),
    );
    report_iso_gate_census(a, b, tolerance)?;
    for surface in [a, b] {
        if let Some(description) = ruled_gate_census(surface, tolerance) {
            eprintln!("FITCENSUS ruled_gate id={id} {description} thread={}", thread_name());
        }
    }
    if count != Lane::Marched && geometry != Lane::Marched && !same_curves {
        // Both trees are exact and disagree on the curves: compare them to
        // each other, whole.
        let count_samples = sample_curves(&count_curves, 512);
        let geometry_samples = sample_curves(&geometry_curves, 512);
        let forward = max_distance_to_curves(&count_samples, &geometry_curves);
        let backward = max_distance_to_curves(&geometry_samples, &count_curves);
        // A section lane may return a carrier's whole section and leave the
        // trimming to the imprint, so what can differ in the RESULT is only the
        // part of each set that reaches both faces.
        let count_touching = touching_both(&count_samples, first, second);
        let geometry_touching = touching_both(&geometry_samples, first, second);
        eprintln!(
            "FITCENSUS exact_pair id={id} count={} geometry={} count_to_geometry={forward:.3e} \
             geometry_to_count={backward:.3e} count_touching={} geometry_touching={} \
             count_to_geometry_touching={:.3e} geometry_to_count_touching={:.3e} thread={}",
            count.label(),
            geometry.label(),
            count_touching.len(),
            geometry_touching.len(),
            max_distance_to_curves(&count_touching, &geometry_curves),
            max_distance_to_curves(&geometry_touching, &count_curves),
            thread_name(),
        );
    }
    if production != Lane::Marched {
        return Ok(None);
    }
    let (lane, exact) = if geometry != Lane::Marched {
        (geometry, geometry_curves)
    } else if count != Lane::Marched {
        (count, count_curves)
    } else {
        return Ok(None);
    };
    Ok(Some(PairCensus {
        id,
        first,
        second,
        fit_tolerance: tolerance.max(1e-7),
        lane,
        exact,
        stations: Vec::new(),
        fitted: Vec::new(),
    }))
}

impl PairCensus<'_> {
    /// A clipped marched run, before it is fitted.
    pub(super) fn record_stations(&mut self, run: &[Vec3]) {
        self.stations.push(run.to_vec());
    }

    /// A curve the marched lane fitted and handed to the imprint.
    pub(super) fn record_fit(&mut self, fit: &PolylineFit) {
        self.fitted.push((fit.curve.clone(), fit.report));
    }

    fn report(&self) {
        let fitted: Vec<NurbsCurve> = self.fitted.iter().map(|(curve, _)| curve.clone()).collect();
        // Exact curves are whole carrier sections; only the part inside both
        // trims is something the marched lane could have produced, so whole-
        // curve samples count where they classify strictly Inside both faces.
        // That misses a short section, and one riding a trim boundary, so the
        // stretch of each exact curve between the feet of every fitted curve's
        // ends — the extent the marched lane claims — is sampled densely and
        // counted whatever it classifies.
        let mut claimed = Vec::new();
        for curve in &fitted {
            let Ok([start, end]) = curve.domain() else {
                continue;
            };
            let (Ok(first), Ok(last)) = (curve.evaluate(start), curve.evaluate(end)) else {
                continue;
            };
            for exact in &self.exact {
                let (Ok(a), Ok(b)) = (
                    project_point_to_curve(exact, first),
                    project_point_to_curve(exact, last),
                ) else {
                    continue;
                };
                let (low, high) = (a.u.min(b.u), a.u.max(b.u));
                for index in 0..=256 {
                    if let Ok(point) = exact.evaluate(low + (high - low) * index as f64 / 256.0) {
                        claimed.push(point);
                    }
                }
            }
        }
        let mut exact_samples: Vec<Vec3> = sample_curves(&self.exact, 1024)
            .into_iter()
            .filter(|point| {
                [self.first, self.second]
                    .iter()
                    .all(|face| face_class(face.face, *point) == Some(PolygonClass::Inside))
            })
            .collect();
        let inside_samples = exact_samples.len();
        let touching_samples =
            touching_both(&sample_curves(&self.exact, 1024), self.first, self.second).len();
        exact_samples.extend(claimed);
        let marched_samples = sample_curves(&fitted, 1024);
        let stations: Vec<Vec3> = self.stations.iter().flatten().copied().collect();
        let exact_to_marched = max_distance_to_curves(&exact_samples, &fitted);
        let marched_to_exact = max_distance_to_curves(&marched_samples, &self.exact);
        let stations_to_exact = max_distance_to_curves(&stations, &self.exact);
        let off = |points: &[Vec3], face: TaggedFace<'_>| {
            max_measured(
                points
                    .iter()
                    .map(|point| distance_to_surface(&face.face.surface, *point)),
            )
        };
        let (exact_off_first, unprojected_ef) = off(&exact_samples, self.first);
        let (exact_off_second, unprojected_es) = off(&exact_samples, self.second);
        let (marched_off_first, unprojected_mf) = off(&marched_samples, self.first);
        let (marched_off_second, unprojected_ms) = off(&marched_samples, self.second);
        let fit_deviation = self
            .fitted
            .iter()
            .map(|(_, report)| report.deviation)
            .fold(0.0_f64, f64::max);
        let unmet = self
            .fitted
            .iter()
            .filter(|(_, report)| !report.met_requested_tolerance())
            .count();
        eprintln!(
            "FITCENSUS compare id={} faces={}x{} lane={} fit_tolerance={:.3e} exact_curves={} \
             fitted_curves={} stations={} exact_samples_inside={} exact_samples_touching={} \
             exact_samples={} \
             exact_to_marched={exact_to_marched:.3e} marched_to_exact={marched_to_exact:.3e} \
             stations_to_exact={stations_to_exact:.3e} fit_deviation={fit_deviation:.3e} \
             unmet_fits={unmet} exact_off_first={exact_off_first:.3e} \
             exact_off_second={exact_off_second:.3e} \
             marched_off_first={marched_off_first:.3e} \
             marched_off_second={marched_off_second:.3e} \
             unprojected={unprojected_ef}/{unprojected_es}/{unprojected_mf}/{unprojected_ms} \
             thread={}",
            self.id,
            self.first.face.id,
            self.second.face.id,
            self.lane.label(),
            self.fit_tolerance,
            self.exact.len(),
            self.fitted.len(),
            stations.len(),
            inside_samples,
            touching_samples,
            exact_samples.len(),
            thread_name(),
        );
    }
}

impl Drop for PairCensus<'_> {
    fn drop(&mut self) {
        if !std::thread::panicking() {
            self.report();
        }
    }
}

/// How `face` classifies `point`: `None` when the point is not ON the face's
/// carrier (a projection clamps an off-patch point to the patch's edge, whose
/// parameter then classifies as if the point were there), otherwise the trim
/// classification of its foot.
fn face_class(face: &FaceRecord, point: Vec3) -> Option<PolygonClass> {
    let projection = project_point_to_surface(&face.surface, point).ok()?;
    // Written as a negated `<=` so that a refusal (`NaN`, which compares false
    // against everything) excludes the point instead of falling through to the
    // trim classification and being counted as on-carrier.
    if !(distance_to_surface(&face.surface, point) <= 1e-6) {
        return None;
    }
    parameter_point_in_face(
        face,
        Vec2 {
            x: projection.u,
            y: projection.v,
        },
        1e-6,
    )
    .ok()
}

/// The samples that lie on both carriers and that neither face classifies
/// Outside.
fn touching_both(points: &[Vec3], first: TaggedFace<'_>, second: TaggedFace<'_>) -> Vec<Vec3> {
    points
        .iter()
        .copied()
        .filter(|point| {
            [first, second].iter().all(|face| {
                face_class(face.face, *point).is_some_and(|class| class != PolygonClass::Outside)
            })
        })
        .collect()
}

/// Distance from `point` to `surface`, resolved past the projector: its foot
/// seeds a Newton iteration on the squared distance, which a projector that
/// stops refining near the comparison's own bar cannot give.
fn distance_to_surface(surface: &NurbsSurface, point: Vec3) -> f64 {
    let Ok(projection) = project_point_to_surface(surface, point) else {
        return f64::NAN;
    };
    let (Ok([u0, u1]), Ok([v0, v1])) = (surface.domain_u(), surface.domain_v()) else {
        return projection.distance;
    };
    let (mut u, mut v) = (projection.u, projection.v);
    let mut best = projection.distance;
    for _ in 0..16 {
        let Ok(d) = surface.derivatives(u, v, 2) else {
            break;
        };
        let r = d[0][0].sub(point);
        best = best.min(r.length());
        let (su, sv) = (d[1][0], d[0][1]);
        let (a, b, c) = (
            su.dot(su) + r.dot(d[2][0]),
            su.dot(sv) + r.dot(d[1][1]),
            sv.dot(sv) + r.dot(d[0][2]),
        );
        let (gu, gv) = (r.dot(su), r.dot(sv));
        let determinant = a * c - b * b;
        if determinant.abs() <= f64::MIN_POSITIVE || !determinant.is_finite() {
            break;
        }
        u = (u - (c * gu - b * gv) / determinant).clamp(u0, u1);
        v = (v - (a * gv - b * gu) / determinant).clamp(v0, v1);
    }
    if let Ok(foot) = surface.evaluate(u, v) {
        best = best.min(foot.sub(point).length());
    }
    best
}

/// `count` uniform parameter samples of every curve, ends included.
fn sample_curves(curves: &[NurbsCurve], count: usize) -> Vec<Vec3> {
    let mut samples = Vec::new();
    for curve in curves {
        let Ok([start, end]) = curve.domain() else {
            continue;
        };
        for index in 0..=count {
            if let Ok(point) = curve.evaluate(start + (end - start) * index as f64 / count as f64) {
                samples.push(point);
            }
        }
    }
    samples
}

/// The largest MEASURED value of a run of distances, with the count that could
/// not be measured at all.
///
/// `distance_to_surface` and `distance_to_curve` return `NaN` when the
/// projector refuses, meaning "unmeasured" — and the obvious fold,
/// `fold(0.0_f64, f64::max)`, turns that into the strongest statement the
/// census can make. Rust's `f64::max` returns the OTHER operand when one side
/// is `NaN`, so `max(0.0, NaN) == 0.0`: a point whose projection refused
/// reported as lying exactly on the carrier. Every refusal read as a perfect
/// result, and a pair whose every point refused reported `0.000e0` on all four
/// on-carrier columns.
///
/// So the refusals are counted instead of absorbed, and a run with nothing
/// measured in it — including an empty run — reports `NaN` rather than a zero
/// with a count printed beside it. The count alone would not be enough: a
/// reader scanning an `exact_off_first=0.000e0` does not go looking for a
/// refusal column further along the line, which is the same mistake one column
/// to the right.
pub(super) fn max_measured(values: impl IntoIterator<Item = f64>) -> (f64, usize) {
    let mut best = f64::NAN;
    let mut unmeasured = 0_usize;
    for value in values {
        if value.is_nan() {
            unmeasured += 1;
        } else if !(value <= best) {
            // NaN-safe and seeds itself: the comparison is false while `best`
            // is still NaN, so the first measured value takes it.
            best = value;
        }
    }
    (best, unmeasured)
}

/// The largest over `points` of the distance to the nearest of `curves`;
/// infinite when there is a point and no curve.
///
/// The inner `f64::min` drops a refused curve's `NaN` rather than counting it,
/// and that is deliberate: a point whose nearest curve refused is then measured
/// against a FARTHER one, so the error is upward and shows as a disagreement
/// rather than as agreement. It is not the [`max_measured`] hazard and must not
/// be "fixed" by folding refusals in — a point every curve refuses still
/// reports infinity, which is the loud answer.
fn max_distance_to_curves(points: &[Vec3], curves: &[NurbsCurve]) -> f64 {
    points
        .iter()
        .map(|point| {
            curves
                .iter()
                .map(|curve| distance_to_curve(curve, *point))
                .fold(f64::INFINITY, f64::min)
        })
        .fold(0.0_f64, f64::max)
}

/// Distance from `point` to `curve`, resolved past the projector's own
/// stopping bar: `project_point_to_curve` stops refining near 1e-7, which is
/// the bar this comparison reads against, so its foot seeds a golden-section
/// minimisation over the parameter bracket one knot span either side.
fn distance_to_curve(curve: &NurbsCurve, point: Vec3) -> f64 {
    let Ok([start, end]) = curve.domain() else {
        return f64::NAN;
    };
    let Ok(projection) = project_point_to_curve(curve, point) else {
        return f64::NAN;
    };
    let closed = match (curve.evaluate(start), curve.evaluate(end)) {
        (Ok(a), Ok(b)) => a.sub(b).length() <= 1e-9 * (1.0 + a.length()),
        _ => false,
    };
    let period = end - start;
    let at = |parameter: f64| -> f64 {
        let parameter = if closed {
            start + (parameter - start).rem_euclid(period)
        } else {
            parameter.clamp(start, end)
        };
        curve
            .evaluate(parameter)
            .map(|sample| sample.sub(point).length_squared())
            .unwrap_or(f64::INFINITY)
    };
    let spans = curve.control_points.len().saturating_sub(curve.degree).max(1);
    let half = period / spans as f64;
    let (mut low, mut high) = (projection.u - half, projection.u + half);
    let ratio = (5.0_f64.sqrt() - 1.0) / 2.0;
    let mut left = high - ratio * (high - low);
    let mut right = low + ratio * (high - low);
    let (mut f_left, mut f_right) = (at(left), at(right));
    for _ in 0..80 {
        if f_left < f_right {
            high = right;
            right = left;
            f_right = f_left;
            left = high - ratio * (high - low);
            f_left = at(left);
        } else {
            low = left;
            left = right;
            f_left = f_right;
            right = low + ratio * (high - low);
            f_right = at(right);
        }
    }
    let refined = f_left.min(f_right).sqrt();
    refined.min(projection.distance)
}
