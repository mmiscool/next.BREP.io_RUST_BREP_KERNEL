use crate::{project_point_to_surface, KnotVector, NurbsSurface, Vec3};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SurfacePairRelation {
    Disjoint,
    Candidate,
    Transverse,
    NearTangent,
    Cosurface,
    Singular,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct SurfacePairClassification {
    pub relation: SurfacePairRelation,
    pub minimum_separation: f64,
    pub minimum_normal_cross: f64,
    /// Every sample within contact distance of the other surface has
    /// PARALLEL normals there: the surfaces only touch tangentially, so
    /// their intersection cannot contain a transverse curve.  Marching
    /// such a pair walks the tangency band's noise (it neither closes nor
    /// exits — the distance-0 glue-extrude runaway); the shared topology
    /// comes from boundary-curve exchange instead.
    pub tangential_only: bool,
}

use crate::spatial::Aabb as Bounds;

fn same_surface(first: &NurbsSurface, second: &NurbsSurface, tolerance: f64) -> bool {
    first.degree_u == second.degree_u
        && first.degree_v == second.degree_v
        && first.knots_u.len() == second.knots_u.len()
        && first.knots_v.len() == second.knots_v.len()
        && first.control_points.len() == second.control_points.len()
        && first
            .knots_u
            .iter()
            .zip(&second.knots_u)
            .all(|(a, b)| (a - b).abs() <= tolerance)
        && first
            .knots_v
            .iter()
            .zip(&second.knots_v)
            .all(|(a, b)| (a - b).abs() <= tolerance)
        && first
            .control_points
            .iter()
            .zip(&second.control_points)
            .all(|(first_row, second_row)| {
                first_row.len() == second_row.len()
                    && first_row.iter().zip(second_row).all(|(a, b)| {
                        (a.x - b.x).abs() <= tolerance
                            && (a.y - b.y).abs() <= tolerance
                            && (a.z - b.z).abs() <= tolerance
                            && (a.w - b.w).abs() <= tolerance
                    })
            })
}

fn planar_support(surface: &NurbsSurface, tolerance: f64) -> Result<Option<(Vec3, Vec3)>, String> {
    let u = KnotVector::new(surface.knots_u.clone(), surface.degree_u)?.domain();
    let v = KnotVector::new(surface.knots_v.clone(), surface.degree_v)?.domain();
    let origin = surface.evaluate((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0)?;
    // A carrier with no normal at its domain centre (a cusp or pinch line
    // through it) is no plane. `BREP_SINGULAR_CENTRE_NOT_PLANAR` (default
    // off; `check` reports each surface) answers "not planar" instead of
    // failing the whole pair classification.
    let normal = match surface.normal((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0) {
        Ok(normal) => normal,
        Err(error) => {
            let on = crate::switch!("BREP_SINGULAR_CENTRE_NOT_PLANAR", false);
            let check = !on && env_read!("BREP_SINGULAR_CENTRE_NOT_PLANAR").as_deref() == Some("check");
            if check {
                eprintln!("SINGULAR_CENTRE_NOT_PLANAR: planar_support: no normal at the carrier's domain centre ({error}); the switch reads it as not planar");
            }
            if on {
                return Ok(None);
            }
            return Err(error);
        }
    };
    let controls = surface
        .control_points
        .iter()
        .flatten()
        .map(|control| control.point())
        .collect::<Result<Vec<_>, _>>()?;
    let scale = controls
        .iter()
        .map(|point| point.sub(origin).length())
        .fold(1.0f64, f64::max);
    let plane_tolerance = (tolerance * 100.0).max(1e-7) * scale;
    Ok(controls
        .iter()
        .all(|point| point.sub(origin).dot(normal).abs() <= plane_tolerance)
        .then_some((origin, normal)))
}



/// PLANE PAIR STRADDLE (2026-11-06; `BREP_PLANE_PAIR_STRADDLE`,
/// default on, `=0` restores the parallel reading). Two planes within the angular tolerance of parallel are
/// still two planes when one crosses the other inside the patches: the
/// hair-turned cube (brep-bench `tangent/cube-turned-a-hair`, 0.001 deg =
/// 1.7e-5 rad) has side pairs whose `|n x n'|` reads 1.7e-5 <= 1e-4 and whose
/// midpoints stand 7.6e-10 apart, so the parallel branch called them
/// Cosurface, while their corners stand +-8.7e-5 off each other's plane. A
/// pair is read as crossing when some control point of either patch stands
/// more than `band` on each side of the other's plane (a degree-1 patch lies
/// in its control hull, so the crossing line passes through it). A pair
/// within the band on one side keeps the parallel reading.
pub fn plane_pair_straddle_enabled() -> bool {
    crate::switch!("BREP_PLANE_PAIR_STRADDLE", true)
}

/// Whether the planar patches `first` (through `first_origin`, normal
/// `first_normal`) and `second` (likewise) cross each other by more than
/// `band` on both sides: see [`plane_pair_straddle_enabled`].
pub fn planar_patches_straddle(
    first: &NurbsSurface,
    first_origin: Vec3,
    first_normal: Vec3,
    second: &NurbsSurface,
    second_origin: Vec3,
    second_normal: Vec3,
    band: f64,
) -> bool {
    let straddles = |surface: &NurbsSurface, origin: Vec3, normal: Vec3| {
        let mut below = false;
        let mut above = false;
        for control in surface.control_points.iter().flatten() {
            let Ok(point) = control.point() else { return false };
            let distance = point.sub(origin).dot(normal);
            below |= distance < -band;
            above |= distance > band;
        }
        below && above
    };
    straddles(second, first_origin, first_normal) || straddles(first, second_origin, second_normal)
}

fn surface_samples(surface: &NurbsSurface) -> Result<Vec<(f64, f64, Vec3)>, String> {
    let u = surface.domain_u()?;
    let v = surface.domain_v()?;
    let mut samples = Vec::with_capacity(25);
    for i in 0..=4 {
        for j in 0..=4 {
            let uu = u[0] + (u[1] - u[0]) * i as f64 / 4.0;
            let vv = v[0] + (v[1] - v[0]) * j as f64 / 4.0;
            samples.push((uu, vv, surface.evaluate(uu, vv)?));
        }
    }
    Ok(samples)
}

/// Per-surface data the pair classifier needs, precomputable once per face
/// and reused across every pair that face participates in.
pub struct SurfaceClassifyData {
    bounds: Bounds,
    planar: Option<(Vec3, Vec3)>,
    /// (u, v, point, normal); normal is None where the surface is singular
    /// (poles) — the classifier treats close singular samples specially.
    samples: Vec<(f64, f64, Vec3, Option<Vec3>)>,
}

thread_local! {
    static CLASSIFY_MEMO: std::cell::RefCell<rustc_hash::FxHashMap<[u64; 2], std::rc::Rc<SurfaceClassifyData>>> = Default::default();
    static CLASSIFY_READS: std::cell::Cell<usize> = const { std::cell::Cell::new(0) };
}

pub fn classify_reads() -> usize { CLASSIFY_READS.with(std::cell::Cell::get) }

pub fn clear_classify_memo() {
    CLASSIFY_MEMO.with(|memo| memo.borrow_mut().clear());
}

impl SurfaceClassifyData {
    /// Only geometry and tolerance enter this read. Renumbering a face or
    /// moving it to the next boolean operand does not require new samples.
    pub fn memoized(surface: &NurbsSurface, tolerance: f64) -> Result<std::rc::Rc<Self>, String> {
        let enabled = std::env::var("BREP_CLASSIFY_MEMO").as_deref() != Ok("0");
        let mut hash = crate::mass_properties::TwoLane::new();
        crate::mass_properties::hash_surface(&mut hash, surface);
        hash.f64(tolerance);
        let key = hash.finish();
        if enabled {
            if let Some(data) = CLASSIFY_MEMO.with(|memo| memo.borrow().get(&key).cloned()) {
                return Ok(data);
            }
        }
        CLASSIFY_READS.with(|reads| reads.set(reads.get() + 1));
        let data = std::rc::Rc::new(Self::build(surface, tolerance)?);
        if enabled {
            CLASSIFY_MEMO.with(|memo| {
                let mut memo = memo.borrow_mut();
                if memo.len() >= 4096 { memo.clear(); }
                memo.insert(key, data.clone());
            });
        }
        Ok(data)
    }

    pub fn build(surface: &NurbsSurface, model_tolerance: f64) -> Result<Self, String> {
        let samples = surface_samples(surface)?
            .into_iter()
            .map(|(u, v, point)| (u, v, point, surface.normal(u, v).ok()))
            .collect();
        Ok(Self {
            bounds: Bounds::from_surface_controls(surface)?,
            planar: planar_support(surface, model_tolerance)?,
            samples,
        })
    }
}

/// Classify a pair before generic SSI runs.  The classifier is conservative:
/// only separated control hulls are declared disjoint; uncertain pairs stay
/// `Candidate` rather than being incorrectly culled.
pub fn classify_surface_pair(
    first: &NurbsSurface,
    second: &NurbsSurface,
    model_tolerance: f64,
    angular_tolerance: f64,
) -> Result<SurfacePairClassification, String> {
    let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::CLASSIFY);
    let first_data = SurfaceClassifyData::build(first, model_tolerance)?;
    let second_data = SurfaceClassifyData::build(second, model_tolerance)?;
    classify_surface_pair_cached(
        first,
        &first_data,
        second,
        &second_data,
        model_tolerance,
        angular_tolerance,
    )
}

/// `classify_surface_pair` with the per-surface work hoisted out — callers
/// with many pairs per face build each face's `SurfaceClassifyData` once.
pub fn classify_surface_pair_cached(
    first: &NurbsSurface,
    first_data: &SurfaceClassifyData,
    second: &NurbsSurface,
    second_data: &SurfaceClassifyData,
    model_tolerance: f64,
    angular_tolerance: f64,
) -> Result<SurfacePairClassification, String> {
    let _wall = crate::wall_profile::Scope::enter(crate::wall_profile::CLASSIFY);
    let first_bounds = first_data.bounds;
    let second_bounds = second_data.bounds;
    let hull_separation = first_bounds.separation(second_bounds);
    let scale = first_bounds.pair_scale(second_bounds);
    let contact = (model_tolerance * 100.0).max(scale * 1e-7);
    if hull_separation > contact {
        return Ok(SurfacePairClassification {
            relation: SurfacePairRelation::Disjoint,
            minimum_separation: hull_separation,
            minimum_normal_cross: 1.0,
            tangential_only: false,
        });
    }
    if same_surface(first, second, model_tolerance) {
        return Ok(SurfacePairClassification {
            relation: SurfacePairRelation::Cosurface,
            minimum_separation: 0.0,
            minimum_normal_cross: 0.0,
            tangential_only: true,
        });
    }
    if let (Some((first_origin, first_normal)), Some((second_origin, second_normal))) =
        (first_data.planar, second_data.planar)
    {
        let normal_cross = first_normal.cross(second_normal).length();
        let crossing = normal_cross <= angular_tolerance
            && plane_pair_straddle_enabled()
            && planar_patches_straddle(first, first_origin, first_normal, second, second_origin, second_normal, contact);
        if normal_cross <= angular_tolerance && !crossing {
            let separation = second_origin.sub(first_origin).dot(first_normal).abs();
            return Ok(SurfacePairClassification {
                relation: if separation <= contact {
                    SurfacePairRelation::Cosurface
                } else {
                    SurfacePairRelation::Disjoint
                },
                minimum_separation: separation,
                minimum_normal_cross: normal_cross,
                tangential_only: separation <= contact,
            });
        }
        return Ok(SurfacePairClassification {
            relation: SurfacePairRelation::Transverse,
            minimum_separation: 0.0,
            minimum_normal_cross: normal_cross,
            tangential_only: false,
        });
    }

    let cull = classify_sample_cull_enabled();
    let reading = sampled_classification(first, first_data, second, second_data, hull_separation, contact, angular_tolerance, cull);
    if cull && classify_sample_cull_check() {
        let fresh = sampled_classification(first, first_data, second, second_data, hull_separation, contact, angular_tolerance, false);
        assert!(same_reading(&reading, &fresh), "pair sample cull check: culled {reading:?} vs fresh {fresh:?}");
    }
    reading
}

/// The sampled reading of a pair whose control boxes meet within `contact`:
/// every sample of each surface projected onto the other. With `cull`, a
/// sample is not projected when its distance to the other surface's control
/// box already exceeds `contact` ([`provably_beyond`]): the surface lies in the
/// convex hull of its controls (finite, positive weights), so the projection's
/// foot, a point of the surface, is at least that far away, and the sample can
/// be neither close nor parallel. Only `minimum_separation` can differ, and only
/// when no sample is close (`Candidate`): it is then the least of the projected
/// distances and the culled samples' box bounds, a lower bound of the sampled
/// reading, which the imprint reads only for grazing pairs (that always have a
/// close sample, so there it is the same reading).
#[allow(clippy::too_many_arguments)]
fn sampled_classification(
    first: &NurbsSurface,
    first_data: &SurfaceClassifyData,
    second: &NurbsSurface,
    second_data: &SurfaceClassifyData,
    hull_separation: f64,
    contact: f64,
    angular_tolerance: f64,
    cull: bool,
) -> Result<SurfacePairClassification, String> {
    let mut minimum_separation = f64::INFINITY;
    let mut minimum_normal_cross = f64::INFINITY;
    let mut close_samples = 0usize;
    let mut parallel_close_samples = 0usize;
    let mut singular_close_sample = false;
    for (source_data, target, target_bounds) in [(first_data, second, second_data.bounds), (second_data, first, first_data.bounds)] {
        let hull_holds = cull && controls_bound_the_surface(target);
        for &(_, _, point, source_normal) in &source_data.samples {
            if hull_holds {
                if let Some(reach) = provably_beyond(point, target_bounds, contact) {
                    minimum_separation = minimum_separation.min(reach);
                    continue;
                }
            }
            let projection = project_point_to_surface(target, point)?;
            minimum_separation = minimum_separation.min(projection.distance);
            if projection.distance > contact {
                continue;
            }
            close_samples += 1;
            match (source_normal, target.normal(projection.u, projection.v)) {
                (Some(first_normal), Ok(second_normal)) => {
                    let cross = first_normal.cross(second_normal).length();
                    minimum_normal_cross = minimum_normal_cross.min(cross);
                    if cross <= angular_tolerance {
                        parallel_close_samples += 1;
                    }
                }
                _ => singular_close_sample = true,
            }
        }
    }
    if minimum_separation == f64::INFINITY {
        minimum_separation = hull_separation;
    }
    if minimum_normal_cross == f64::INFINITY {
        minimum_normal_cross = 1.0;
    }
    let relation = if close_samples == 0 {
        SurfacePairRelation::Candidate
    } else if singular_close_sample {
        SurfacePairRelation::Singular
    } else if close_samples >= 18 && parallel_close_samples == close_samples {
        SurfacePairRelation::Cosurface
    } else if minimum_normal_cross <= angular_tolerance {
        SurfacePairRelation::NearTangent
    } else {
        SurfacePairRelation::Transverse
    };
    Ok(SurfacePairClassification {
        relation,
        minimum_separation,
        minimum_normal_cross,
        tangential_only: close_samples > 0 && parallel_close_samples == close_samples,
    })
}



/// Whether every control of `surface` is finite with a finite, positive weight:
/// the premise under which the surface lies in its control box.
fn controls_bound_the_surface(surface: &NurbsSurface) -> bool {
    surface.control_points.iter().flatten().all(|control| {
        control.w.is_finite() && control.w > 0.0 && control.x.is_finite() && control.y.is_finite() && control.z.is_finite()
    })
}

/// A lower bound of `point`'s distance to any point within `bounds`, when it
/// exceeds `contact`; `None` otherwise. The box gap is cushioned for the
/// rounding of the gap itself and of an evaluated surface point: relative
/// 1e-9, plus 64 ulps of the coordinates' size.
fn provably_beyond(point: Vec3, bounds: Bounds, contact: f64) -> Option<f64> {
    let gap = (Bounds { minimum: point, maximum: point }).separation(bounds);
    let cushion = 64.0 * f64::EPSILON * (point.length() + bounds.minimum.length() + bounds.maximum.length());
    let reach = gap * (1.0 - 1e-9) - cushion;
    (reach.is_finite() && reach > contact).then_some(reach)
}

/// `BREP_CLASSIFY_SAMPLE_CULL` (default on; =0 projects every sample): the pair classifier does not
/// project a sample its box bound already places beyond the contact band
/// ([`sampled_classification`]); every field the kernel reads is the same.
pub fn classify_sample_cull_enabled() -> bool {
    crate::switch!("BREP_CLASSIFY_SAMPLE_CULL", true)
}

/// `BREP_CLASSIFY_SAMPLE_CULL_CHECK=1`: classify both ways and stop on any
/// difference in the relation, `tangential_only` or `minimum_normal_cross`,
/// or in `minimum_separation` of a pair that is not a `Candidate`.
fn classify_sample_cull_check() -> bool {
    crate::switch!("BREP_CLASSIFY_SAMPLE_CULL_CHECK", false)
}

fn same_reading(culled: &Result<SurfacePairClassification, String>, fresh: &Result<SurfacePairClassification, String>) -> bool {
    match (culled, fresh) {
        (Ok(a), Ok(b)) => {
            a.relation == b.relation
                && a.tangential_only == b.tangential_only
                && a.minimum_normal_cross.to_bits() == b.minimum_normal_cross.to_bits()
                && (a.relation == SurfacePairRelation::Candidate || a.minimum_separation.to_bits() == b.minimum_separation.to_bits())
        }
        (Err(a), Err(b)) => a == b,
        _ => false,
    }
}


