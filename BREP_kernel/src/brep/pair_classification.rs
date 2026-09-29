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

#[derive(Clone, Copy)]
struct Bounds {
    low: Vec3,
    high: Vec3,
}

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
    let normal = surface.normal((u[0] + u[1]) / 2.0, (v[0] + v[1]) / 2.0)?;
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

impl Bounds {
    fn of(surface: &NurbsSurface) -> Result<Self, String> {
        let mut low = Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut high = Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
        for control in surface.control_points.iter().flatten() {
            let point = control.point()?;
            low.x = low.x.min(point.x);
            low.y = low.y.min(point.y);
            low.z = low.z.min(point.z);
            high.x = high.x.max(point.x);
            high.y = high.y.max(point.y);
            high.z = high.z.max(point.z);
        }
        Ok(Self { low, high })
    }

    fn separation(self, other: Self) -> f64 {
        let axis_gap = |a0: f64, a1: f64, b0: f64, b1: f64| {
            if a1 < b0 {
                b0 - a1
            } else if b1 < a0 {
                a0 - b1
            } else {
                0.0
            }
        };
        Vec3::new(
            axis_gap(self.low.x, self.high.x, other.low.x, other.high.x),
            axis_gap(self.low.y, self.high.y, other.low.y, other.high.y),
            axis_gap(self.low.z, self.high.z, other.low.z, other.high.z),
        )
        .length()
    }

    fn scale(self, other: Self) -> f64 {
        self.high
            .sub(self.low)
            .length()
            .max(other.high.sub(other.low).length())
            .max(1.0)
    }
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

impl SurfaceClassifyData {
    pub fn build(surface: &NurbsSurface, model_tolerance: f64) -> Result<Self, String> {
        let samples = surface_samples(surface)?
            .into_iter()
            .map(|(u, v, point)| (u, v, point, surface.normal(u, v).ok()))
            .collect();
        Ok(Self {
            bounds: Bounds::of(surface)?,
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
    let first_bounds = first_data.bounds;
    let second_bounds = second_data.bounds;
    let hull_separation = first_bounds.separation(second_bounds);
    let scale = first_bounds.scale(second_bounds);
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
        if normal_cross <= angular_tolerance {
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

    let mut minimum_separation = f64::INFINITY;
    let mut minimum_normal_cross = f64::INFINITY;
    let mut close_samples = 0usize;
    let mut parallel_close_samples = 0usize;
    let mut singular_close_sample = false;
    for (source_data, target) in [(first_data, second), (second_data, first)] {
        for &(_, _, point, source_normal) in &source_data.samples {
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

