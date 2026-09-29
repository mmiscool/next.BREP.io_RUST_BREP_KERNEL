//! Primitive initialization, constrained geometric refinement, and validation.
//!
//! The fitter consumes area-weighted observations rather than raw mesh
//! storage.  Mesh preprocessing is responsible for making the observation
//! weights sum to represented triangle area (for example `area / 3` for the
//! three vertices of a triangle).  This prevents dense portions of an
//! otherwise exact CAD tessellation from dominating a fit.

use crate::math::{eigen_symmetric3, least_squares, least_squares_qr, outer_accumulate};
use crate::numerical::{fitting as numerical, linear_algebra, scalar};
use crate::{
    AnalyticSurface, AnalyzedMesh, ConeSurface, ConstraintMask, CylinderSurface, FitDiagnostics,
    FitMetrics, FitPath, GeometricError, PhaseTimings, PlaneSurface, RecognitionError,
    RecognitionOptions, SamplingMode, SphereSurface, SurfaceFitResult, SurfaceParameterDelta,
    SurfaceType, TorusSurface, Vec3,
};
use std::collections::BTreeMap;
use web_time::Instant;

const MIN_ANGLE: f64 = numerical::MIN_CONE_HALF_ANGLE;
const MAX_ANGLE: f64 = std::f64::consts::FRAC_PI_2 - MIN_ANGLE;

/// One quadrature/sample observation used by the numerical fitters.
#[derive(Clone, Copy, Debug)]
pub(crate) struct FitObservation {
    pub point: Vec3,
    /// Unit mesh normal when available. Position-only observations use None.
    pub normal: Option<Vec3>,
    /// Represented surface area. Must be finite and positive.
    pub weight: f64,
    pub triangle: usize,
}

#[derive(Clone, Debug)]
pub(crate) struct FittedModel {
    pub surface: AnalyticSurface,
    pub orientation: i8,
    pub metrics: FitMetrics,
    /// Change in normalized objective during the final accepted iteration.
    pub final_improvement: f64,
    pub initial_surface: Option<AnalyticSurface>,
    pub initial_error: Option<GeometricError>,
    pub phase_timings: PhaseTimings,
}

pub(crate) fn fit_surface_with_path(
    mesh: &AnalyzedMesh,
    triangle_indices: &[usize],
    kind: SurfaceType,
    initial: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    options: &RecognitionOptions,
    path: FitPath,
) -> Result<SurfaceFitResult, RecognitionError> {
    mesh.validate_selection(triangle_indices)?;
    let samples = observations_for_selection(mesh, triangle_indices, options.sampling);
    let mut fitted = fit_primitive(kind, &samples, initial, fixed, options)?;
    set_selection_support(&mut fitted.metrics, mesh, triangle_indices);
    Ok(to_result(fitted, initial, fixed, options, path, false))
}

pub(crate) fn fit_surface_from_vertices_with_path(
    mesh: &AnalyzedMesh,
    vertex_indices: &[usize],
    kind: SurfaceType,
    initial: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    options: &RecognitionOptions,
    path: FitPath,
) -> Result<SurfaceFitResult, RecognitionError> {
    mesh.validate_vertex_selection(vertex_indices)?;
    let samples = observations_for_vertex_selection(mesh, vertex_indices);
    let mut fitted = fit_primitive(kind, &samples, initial, fixed, options)?;
    set_vertex_selection_support(&mut fitted.metrics, mesh, vertex_indices);
    Ok(to_result(fitted, initial, fixed, options, path, false))
}

/// Evaluate an existing candidate exactly as supplied. No canonicalization or
/// optimization occurs, which is important for trusted source parameters.
pub(crate) fn evaluate_surface(
    mesh: &AnalyzedMesh,
    triangle_indices: &[usize],
    surface: AnalyticSurface,
    options: &RecognitionOptions,
    path: FitPath,
) -> Result<SurfaceFitResult, RecognitionError> {
    mesh.validate_selection(triangle_indices)?;
    let samples = observations_for_selection(mesh, triangle_indices, options.sampling);
    let mut fitted = validate_surface(surface, &samples, options.collect_phase_timings)?;
    set_selection_support(&mut fitted.metrics, mesh, triangle_indices);
    Ok(to_result(
        fitted,
        Some(surface),
        ConstraintMask::default(),
        options,
        path,
        matches!(path, FitPath::ExactCandidateReused | FitPath::HintReused),
    ))
}

pub(crate) fn evaluate_surface_from_vertices(
    mesh: &AnalyzedMesh,
    vertex_indices: &[usize],
    surface: AnalyticSurface,
    options: &RecognitionOptions,
    path: FitPath,
) -> Result<SurfaceFitResult, RecognitionError> {
    mesh.validate_vertex_selection(vertex_indices)?;
    let samples = observations_for_vertex_selection(mesh, vertex_indices);
    let mut fitted = validate_surface(surface, &samples, options.collect_phase_timings)?;
    set_vertex_selection_support(&mut fitted.metrics, mesh, vertex_indices);
    Ok(to_result(
        fitted,
        Some(surface),
        ConstraintMask::default(),
        options,
        path,
        matches!(path, FitPath::ExactCandidateReused | FitPath::HintReused),
    ))
}

fn set_selection_support(metrics: &mut FitMetrics, mesh: &AnalyzedMesh, ids: &[usize]) {
    let mut unique = ids.to_vec();
    unique.sort_unstable();
    unique.dedup();
    metrics.support_triangles = unique
        .iter()
        .filter(|&&id| mesh.triangles[id].area > 0.0)
        .count();
    metrics.supported_area = unique.iter().map(|&id| mesh.triangles[id].area).sum();
}

fn set_vertex_selection_support(
    metrics: &mut FitMetrics,
    mesh: &AnalyzedMesh,
    vertex_indices: &[usize],
) {
    let selected: std::collections::BTreeSet<_> = vertex_indices.iter().copied().collect();
    metrics.support_triangles = mesh
        .triangles
        .iter()
        .filter(|triangle| {
            triangle.area > 0.0
                && triangle
                    .vertices
                    .iter()
                    .any(|vertex| selected.contains(vertex))
        })
        .count();
    metrics.supported_area = vertex_indices
        .iter()
        .map(|&vertex| mesh.vertex_area_weights[vertex])
        .sum();
}

pub(crate) fn selection_scale(mesh: &AnalyzedMesh, triangle_indices: &[usize]) -> f64 {
    let center = mesh.selection_centroid(triangle_indices);
    triangle_indices
        .iter()
        .flat_map(|&tid| mesh.triangles[tid].vertices)
        .map(|vid| mesh.vertices[vid].distance(center))
        .fold(0.0_f64, f64::max)
        .max(scalar::GEOMETRIC_SCALE_FLOOR)
}

pub(crate) fn vertex_selection_scale(mesh: &AnalyzedMesh, vertex_indices: &[usize]) -> f64 {
    let center = vertex_indices
        .iter()
        .map(|&vertex| mesh.vertices[vertex])
        .fold(Vec3::ZERO, |sum, point| sum + point)
        / vertex_indices.len() as f64;
    vertex_indices
        .iter()
        .map(|&vertex| mesh.vertices[vertex].distance(center))
        .fold(0.0_f64, f64::max)
        .max(scalar::GEOMETRIC_SCALE_FLOOR)
}

fn to_result(
    fitted: FittedModel,
    supplied: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    options: &RecognitionOptions,
    path: FitPath,
    reused: bool,
) -> SurfaceFitResult {
    let distance_score =
        (-0.5 * (fitted.metrics.rms_error / options.distance_tolerance).powi(2)).exp();
    let normal_score = if options.normal_tolerance > 0.0 {
        (-0.5 * (fitted.metrics.rms_normal_error / options.normal_tolerance).powi(2)).exp()
    } else if fitted.metrics.rms_normal_error == 0.0 {
        1.0
    } else {
        0.0
    };
    let confidence = (distance_score * normal_score).clamp(0.0, 1.0);
    let refined_error = GeometricError::from(&fitted.metrics);
    let parameter_delta = fitted
        .initial_surface
        .and_then(|initial| SurfaceParameterDelta::between(initial, fitted.surface));
    SurfaceFitResult {
        surface: fitted.surface,
        orientation: fitted.orientation,
        metrics: fitted.metrics,
        confidence,
        diagnostics: FitDiagnostics {
            path,
            supplied_surface: supplied,
            fixed_parameters: fixed,
            parameters_refined: !reused,
            generic_classification_skipped: !matches!(
                path,
                FitPath::GenericRecognition | FitPath::HintRejectedFallback
            ),
            exact_parameters_reused: reused,
            hypotheses_generated: 0,
            candidates_evaluated: 1,
            reason: if reused {
                "supplied parameters validated unchanged".into()
            } else {
                format!(
                    "area-weighted geometric fit completed; final relative improvement {:.3e}",
                    fitted.final_improvement
                )
            },
            rejected_competitors: Vec::new(),
            metadata_trust: Default::default(),
            initial_error: fitted.initial_error,
            refined_error: Some(refined_error),
            parameter_delta,
            phase_timings: fitted.phase_timings,
        },
    }
}

fn observations_for_selection(
    mesh: &AnalyzedMesh,
    triangle_indices: &[usize],
    mode: SamplingMode,
) -> Vec<FitObservation> {
    let centroid_fraction = if matches!(mode, SamplingMode::CentroidsAndVertices) {
        0.5
    } else {
        1.0
    };
    let vertex_fraction = centroid_fraction;
    let mut result = Vec::new();
    if matches!(
        mode,
        SamplingMode::TriangleCentroids | SamplingMode::CentroidsAndVertices
    ) {
        for &tid in triangle_indices {
            let tri = &mesh.triangles[tid];
            if tri.area > 0.0 {
                result.push(FitObservation {
                    point: tri.centroid,
                    normal: Some(tri.normal),
                    weight: tri.area * centroid_fraction,
                    triangle: tid,
                });
            }
        }
    }
    if matches!(
        mode,
        SamplingMode::Vertices | SamplingMode::CentroidsAndVertices
    ) {
        // Accumulate both selected area and its normal at each vertex. The
        // deterministic BTreeMap order makes fits repeatable across platforms.
        let mut vertices: BTreeMap<usize, (f64, Vec3, usize)> = BTreeMap::new();
        for &tid in triangle_indices {
            let tri = &mesh.triangles[tid];
            if tri.area <= 0.0 {
                continue;
            }
            for &vid in &tri.vertices {
                let entry = vertices.entry(vid).or_insert((0.0, Vec3::ZERO, tid));
                entry.0 += tri.area / 3.0;
                entry.1 += tri.normal * (tri.area / 3.0);
            }
        }
        let supplied_sense = mesh.vertex_normals.as_ref().map(|supplied| {
            let alignment: f64 = vertices
                .iter()
                .map(|(&vid, (_, winding_normal, _))| supplied[vid].dot(*winding_normal))
                .sum();
            if alignment < 0.0 {
                -1.0
            } else {
                1.0
            }
        });
        result.extend(
            vertices
                .into_iter()
                .map(|(vid, (weight, winding_normal, tid))| {
                    let normal = mesh.vertex_normals.as_ref().map_or_else(
                        || winding_normal.normalized(),
                        |supplied| {
                            // Supplied derivatives provide the accurate local
                            // direction. Winding remains authoritative for one
                            // region-wide sign, but isolated reversed/sliver
                            // facets must not destroy normal-field coherence.
                            Some(supplied[vid] * supplied_sense.unwrap_or(1.0))
                        },
                    );
                    FitObservation {
                        point: mesh.vertices[vid],
                        normal,
                        weight: weight * vertex_fraction,
                        triangle: tid,
                    }
                }),
        );
    }
    result
}

fn observations_for_vertex_selection(
    mesh: &AnalyzedMesh,
    vertex_indices: &[usize],
) -> Vec<FitObservation> {
    let vertices = vertex_indices
        .iter()
        .filter_map(|&vertex| {
            let weight = mesh.vertex_area_weights[vertex];
            if weight <= 0.0 {
                return None;
            }
            let mut incident_normal = Vec3::ZERO;
            let mut representative_triangle = 0;
            for (triangle_id, triangle) in mesh.triangles.iter().enumerate() {
                if triangle.area > 0.0 && triangle.vertices.contains(&vertex) {
                    incident_normal += triangle.normal * (triangle.area / 3.0);
                    representative_triangle = triangle_id;
                }
            }
            Some((vertex, weight, incident_normal, representative_triangle))
        })
        .collect::<Vec<_>>();
    let supplied_sense = mesh.vertex_normals.as_ref().map(|supplied| {
        let alignment: f64 = vertices
            .iter()
            .map(|(vertex, _, winding_normal, _)| supplied[*vertex].dot(*winding_normal))
            .sum();
        if alignment < 0.0 {
            -1.0
        } else {
            1.0
        }
    });
    vertices
        .into_iter()
        .map(
            |(vertex, weight, incident_normal, representative_triangle)| {
                let normal = mesh.vertex_normals.as_ref().map_or_else(
                    || incident_normal.normalized(),
                    |supplied| Some(supplied[vertex] * supplied_sense.unwrap_or(1.0)),
                );
                FitObservation {
                    point: mesh.vertices[vertex],
                    normal,
                    weight,
                    triangle: representative_triangle,
                }
            },
        )
        .collect()
}

/// Fit one requested primitive. An initial surface is used when it has the
/// requested type; fields selected by `fixed` are never changed.
pub(crate) fn fit_primitive(
    kind: SurfaceType,
    observations: &[FitObservation],
    initial: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    options: &RecognitionOptions,
) -> Result<FittedModel, RecognitionError> {
    validate_observations(observations, kind)?;
    let centroid =
        weighted_centroid(observations).ok_or_else(|| fit_error(kind, "zero sample weight"))?;
    let scale = observation_scale(observations, centroid);
    let matching_initial = initial.filter(|s| s.surface_type() == kind && s.is_valid());
    if fixed != ConstraintMask::default() && matching_initial.is_none() {
        return Err(fit_error(
            kind,
            "fixed constraints require a valid matching initial surface",
        ));
    }
    let coarse_initialization = (matching_initial.is_none()
        && observations.len() > MAX_COARSE_REFINEMENT_SAMPLES
        && options.max_refinement_iterations > MAX_FULL_DATA_POLISH_ITERATIONS)
        .then(|| stratified_refinement_samples(observations));
    let initialization_observations = coarse_initialization.as_deref().unwrap_or(observations);
    let initialization_started = options.collect_phase_timings.then(Instant::now);
    let (seed, alternatives) = match matching_initial {
        Some(surface) => (surface, Vec::new()),
        None if kind == SurfaceType::Cylinder => {
            let seeds = initialize_cylinder_seeds(initialization_observations, centroid, scale)?;
            (seeds.primary, seeds.alternatives)
        }
        None if kind == SurfaceType::Torus => {
            let seeds = initialize_torus_seeds(initialization_observations, centroid, scale)?;
            (seeds.primary, seeds.alternatives)
        }
        None => (
            initialize(kind, initialization_observations, centroid, scale)?,
            Vec::new(),
        ),
    };
    let initialization_seconds = elapsed(initialization_started);
    let refinement_started = options.collect_phase_timings.then(Instant::now);
    let (surface, final_improvement, selected_seed) = match kind {
        SurfaceType::Plane => (refine_plane(seed, observations, fixed)?, 0.0, seed),
        SurfaceType::Cylinder if !alternatives.is_empty() => {
            let (mut best_surface, mut best_improvement) = refine_lm(
                seed,
                observations,
                fixed,
                scale,
                options.max_refinement_iterations,
            )?;
            let mut best_seed = seed;
            let mut best_loss =
                cylinder_initialization_objective(best_surface, observations, scale);
            let primary_max_normal = measure_surface(best_surface, observations)
                .1
                .max_normal_error;
            let alternatives =
                if primary_max_normal > numerical::CYLINDER_SEED_NORMAL_EQUIVALENCE_RADIANS {
                    alternatives
                } else {
                    Vec::new()
                };
            for alternative_seed in alternatives {
                if let Ok((alternative_surface, alternative_improvement)) = refine_lm(
                    alternative_seed,
                    observations,
                    fixed,
                    scale,
                    options.max_refinement_iterations,
                ) {
                    let alternative_loss =
                        cylinder_initialization_objective(alternative_surface, observations, scale);
                    if alternative_surface.is_valid() && alternative_loss < best_loss {
                        best_surface = alternative_surface;
                        best_improvement = alternative_improvement;
                        best_seed = alternative_seed;
                        best_loss = alternative_loss;
                    }
                }
            }
            (best_surface, best_improvement, best_seed)
        }
        SurfaceType::Torus if !alternatives.is_empty() => {
            let (mut best_surface, mut best_improvement) = refine_lm(
                seed,
                observations,
                fixed,
                scale,
                options.max_refinement_iterations,
            )?;
            let mut best_seed = seed;
            let mut best_loss = torus_initialization_objective(best_surface, observations, scale);
            for alternative_seed in alternatives {
                if let Ok((alternative_surface, alternative_improvement)) = refine_lm(
                    alternative_seed,
                    observations,
                    fixed,
                    scale,
                    options.max_refinement_iterations,
                ) {
                    let alternative_loss =
                        torus_initialization_objective(alternative_surface, observations, scale);
                    if alternative_surface.is_valid() && alternative_loss < best_loss {
                        best_surface = alternative_surface;
                        best_improvement = alternative_improvement;
                        best_seed = alternative_seed;
                        best_loss = alternative_loss;
                    }
                }
            }
            (best_surface, best_improvement, best_seed)
        }
        _ => {
            let refined = refine_lm(
                seed,
                observations,
                fixed,
                scale,
                options.max_refinement_iterations,
            )?;
            (refined.0, refined.1, seed)
        }
    };
    let refinement_seconds = elapsed(refinement_started);
    // A supplied hint's starting residual is part of the public accuracy
    // contract. For unknown/type-only fits, collect the generated seed's
    // residual only when detailed phase diagnostics were explicitly enabled.
    // Multi-start torus diagnostics describe the seed of the selected basin.
    let initial_error = if matching_initial.is_some() || options.collect_phase_timings {
        Some(GeometricError::from(
            &measure_surface(selected_seed, observations).1,
        ))
    } else {
        None
    };
    if !surface.is_valid() {
        return Err(fit_error(kind, "refinement produced invalid parameters"));
    }
    let validation_started = options.collect_phase_timings.then(Instant::now);
    let (orientation, metrics) = measure_surface(surface, observations);
    let validation_seconds = elapsed(validation_started);
    Ok(FittedModel {
        surface,
        orientation,
        metrics,
        final_improvement,
        initial_surface: Some(selected_seed),
        initial_error,
        phase_timings: PhaseTimings {
            candidate_generation_seconds: initialization_seconds,
            refinement_seconds,
            validation_seconds,
            ..Default::default()
        },
    })
}

/// Validate a supplied model without changing any parameter.
pub(crate) fn validate_surface(
    surface: AnalyticSurface,
    observations: &[FitObservation],
    collect_phase_timings: bool,
) -> Result<FittedModel, RecognitionError> {
    validate_observation_values(observations, surface.surface_type())?;
    if !surface.is_valid() {
        return Err(fit_error(
            surface.surface_type(),
            "candidate parameters are invalid",
        ));
    }
    let validation_started = collect_phase_timings.then(Instant::now);
    let (orientation, metrics) = measure_surface(surface, observations);
    let validation_seconds = elapsed(validation_started);
    Ok(FittedModel {
        surface,
        orientation,
        initial_error: Some(GeometricError::from(&metrics)),
        metrics,
        final_improvement: 0.0,
        initial_surface: Some(surface),
        phase_timings: PhaseTimings {
            validation_seconds,
            ..Default::default()
        },
    })
}

fn elapsed(started: Option<Instant>) -> Option<f64> {
    started.map(|started| started.elapsed().as_secs_f64())
}

fn fit_error(kind: SurfaceType, reason: impl Into<String>) -> RecognitionError {
    RecognitionError::FitFailed {
        surface: Some(kind.name()),
        reason: reason.into(),
    }
}

fn validate_observations(
    observations: &[FitObservation],
    kind: SurfaceType,
) -> Result<(), RecognitionError> {
    let required = match kind {
        SurfaceType::Plane => 3,
        SurfaceType::Sphere => 4,
        SurfaceType::Cylinder => 3,
        SurfaceType::Cone => 4,
        SurfaceType::Torus => 6,
    };
    if observations.len() < required {
        return Err(fit_error(
            kind,
            format!("requires at least {required} observations"),
        ));
    }
    validate_observation_values(observations, kind)
}

fn validate_observation_values(
    observations: &[FitObservation],
    kind: SurfaceType,
) -> Result<(), RecognitionError> {
    if observations.is_empty() {
        return Err(fit_error(kind, "requires at least one observation"));
    }
    if observations.iter().any(|s| {
        !s.point.is_finite()
            || !s.weight.is_finite()
            || s.weight <= 0.0
            || s.normal.is_some_and(|n| !n.is_finite())
    }) {
        return Err(fit_error(
            kind,
            "non-finite observation or non-positive weight",
        ));
    }
    Ok(())
}

fn weighted_centroid(samples: &[FitObservation]) -> Option<Vec3> {
    let weight: f64 = samples.iter().map(|s| s.weight).sum();
    (weight.is_finite() && weight > 0.0).then(|| {
        samples
            .iter()
            .fold(Vec3::ZERO, |sum, s| sum + s.point * s.weight)
            / weight
    })
}

fn observation_scale(samples: &[FitObservation], center: Vec3) -> f64 {
    samples
        .iter()
        .map(|s| s.point.distance(center))
        .fold(0.0_f64, f64::max)
        .max(scalar::GEOMETRIC_SCALE_FLOOR)
}

fn initialize(
    kind: SurfaceType,
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<AnalyticSurface, RecognitionError> {
    match kind {
        SurfaceType::Plane => initialize_plane(samples, centroid),
        SurfaceType::Sphere => initialize_sphere(samples, centroid, scale),
        SurfaceType::Cylinder => initialize_cylinder(samples, centroid, scale),
        SurfaceType::Cone => initialize_cone(samples, centroid, scale),
        SurfaceType::Torus => initialize_torus(samples, centroid, scale),
    }
}

fn point_covariance(samples: &[FitObservation], center: Vec3) -> [[f64; 3]; 3] {
    let mut covariance = [[0.0; 3]; 3];
    for sample in samples {
        outer_accumulate(&mut covariance, sample.point - center, sample.weight);
    }
    covariance
}

fn normal_statistics(samples: &[FitObservation]) -> Option<(Vec3, [[f64; 3]; 3])> {
    let mut mean = Vec3::ZERO;
    let mut weight = 0.0;
    for sample in samples {
        if let Some(normal) = sample.normal.and_then(Vec3::normalized) {
            mean += normal * sample.weight;
            weight += sample.weight;
        }
    }
    if weight <= 0.0 {
        return None;
    }
    mean = mean / weight;
    let mut covariance = [[0.0; 3]; 3];
    for sample in samples {
        if let Some(normal) = sample.normal.and_then(Vec3::normalized) {
            outer_accumulate(&mut covariance, normal - mean, sample.weight);
        }
    }
    Some((mean, covariance))
}

fn normal_second_moment(samples: &[FitObservation]) -> Option<[[f64; 3]; 3]> {
    let mut moment = [[0.0; 3]; 3];
    let mut weight = 0.0;
    for sample in samples {
        if let Some(normal) = sample.normal.and_then(Vec3::normalized) {
            outer_accumulate(&mut moment, normal, sample.weight);
            weight += sample.weight;
        }
    }
    (weight > 0.0).then_some(moment)
}

fn initialize_plane(
    samples: &[FitObservation],
    centroid: Vec3,
) -> Result<AnalyticSurface, RecognitionError> {
    let (values, vectors) = eigen_symmetric3(point_covariance(samples, centroid));
    let covariance_normal = vectors[0]
        .normalized()
        .ok_or_else(|| fit_error(SurfaceType::Plane, "rank-deficient covariance"))?;
    if values[1] <= linear_algebra::MACHINE_RANK_RELATIVE_MIN * values[2].abs().max(1.0) {
        // Positions on a line admit infinitely many planes, but a coherent
        // supplied normal field identifies the sampled plane directly.  This
        // occurs on very thin CAD trims whose second transverse extent is
        // lost at coordinate precision.  Residual and normal acceptance still
        // validate the resulting carrier, so inconsistent normals cannot turn
        // a genuinely non-planar selection into an accepted plane.
        let supplied_normal = normal_statistics(samples)
            .and_then(|(mean, _)| mean.normalized())
            .ok_or_else(|| {
                fit_error(
                    SurfaceType::Plane,
                    "observations are collinear and normals do not identify a plane",
                )
            })?;
        return Ok(AnalyticSurface::Plane(PlaneSurface {
            origin: centroid,
            normal: supplied_normal,
        }));
    }
    let mut normal = orient_from_normals(covariance_normal, samples);
    if values[1]
        <= numerical::PLANE_TRANSVERSE_COVARIANCE_RELATIVE_MAX
            * values[2].abs().max(scalar::POSITIVE_DENOMINATOR_FLOOR)
    {
        if let Some((mean, _)) = normal_statistics(samples) {
            if let Some(supplied_normal) = mean.normalized() {
                let supplied_normal = orient_from_normals(supplied_normal, samples);
                let coherent = samples.iter().all(|sample| {
                    sample
                        .normal
                        .and_then(Vec3::normalized)
                        .is_none_or(|sample_normal| sample_normal.dot(supplied_normal) > 0.0)
                });
                if coherent {
                    let position_error = |candidate: Vec3| {
                        let mut weighted_squared = 0.0;
                        let mut total_weight = 0.0;
                        let mut maximum = 0.0_f64;
                        for sample in samples {
                            let residual = (sample.point - centroid).dot(candidate).abs();
                            weighted_squared += sample.weight * residual * residual;
                            total_weight += sample.weight;
                            maximum = maximum.max(residual);
                        }
                        ((weighted_squared / total_weight).sqrt(), maximum)
                    };
                    let covariance_error = position_error(normal);
                    let supplied_error = position_error(supplied_normal);
                    let coordinate_scale = samples.iter().fold(1.0_f64, |largest, sample| {
                        largest
                            .max(sample.point.x.abs())
                            .max(sample.point.y.abs())
                            .max(sample.point.z.abs())
                    });
                    let material = crate::numerical::recognition::EXACT_VERTEX_ROUNDOFF_MULTIPLIER
                        * f64::EPSILON
                        * coordinate_scale;
                    if supplied_error.0 + material < covariance_error.0
                        && supplied_error.1 + material < covariance_error.1
                    {
                        normal = supplied_normal;
                    }
                }
            }
        }
    }
    Ok(AnalyticSurface::Plane(PlaneSurface {
        origin: centroid,
        normal,
    }))
}

fn initialize_sphere(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<AnalyticSurface, RecognitionError> {
    // Centered, nondimensional algebraic fit:
    // 2 (q / scale).c + k = |q / scale|^2. Centering is essential for CAD
    // models translated far from the world origin; scaling coordinates and
    // normalizing area weights are equally important for tiny and huge
    // carriers, because otherwise the normal-equation pivots inherit units of
    // area * length^2 and can be falsely classified as singular.
    let total_weight: f64 = samples.iter().map(|sample| sample.weight).sum();
    let mut rows = Vec::with_capacity(samples.len());
    let mut rhs = Vec::with_capacity(samples.len());
    for sample in samples {
        let q = (sample.point - centroid) / scale;
        let w = (sample.weight / total_weight).sqrt();
        rows.push(vec![2.0 * q.x * w, 2.0 * q.y * w, 2.0 * q.z * w, w]);
        rhs.push(q.length_squared() * w);
    }
    let x = least_squares(&rows, &rhs, numerical::ALGEBRAIC_REGULARIZATION_RELATIVE)
        .ok_or_else(|| fit_error(SurfaceType::Sphere, "singular algebraic sphere system"))?;
    let center = centroid + Vec3::new(x[0], x[1], x[2]) * scale;
    let radius = weighted_mean(samples, |s| s.point.distance(center));
    if !radius.is_finite() || radius <= numerical::MIN_RADIUS_RELATIVE_TO_SCALE * scale {
        return Err(fit_error(
            SurfaceType::Sphere,
            "non-positive or unobservable radius",
        ));
    }
    Ok(AnalyticSurface::Sphere(SphereSurface { center, radius }))
}

fn initialize_cylinder(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<AnalyticSurface, RecognitionError> {
    // Use the uncentered normal moment. Cylinder normals lie in the plane
    // perpendicular to the axis, so its least-eigenvalue direction is the
    // axis even when a short patch contains only two distinct normal rays.
    // Centering the normals makes such a two-ray fan rank one and discards the
    // axis information together with the mean radial direction.
    let moment = normal_second_moment(samples).ok_or_else(|| {
        fit_error(
            SurfaceType::Cylinder,
            "cylinder initialization requires normals",
        )
    })?;
    let (values, vectors) = eigen_symmetric3(moment);
    if values[1]
        <= numerical::CYLINDER_NORMAL_FAN_RANK_RELATIVE
            * values[2].abs().max(scalar::POSITIVE_DENOMINATOR_FLOOR)
    {
        return Err(fit_error(
            SurfaceType::Cylinder,
            "normal fan does not determine an axis",
        ));
    }
    let axis = vectors[0]
        .normalized()
        .ok_or_else(|| fit_error(SurfaceType::Cylinder, "invalid axis"))?
        .canonicalized();
    let (u, v) = axis
        .orthonormal_basis()
        .ok_or_else(|| fit_error(SurfaceType::Cylinder, "invalid axis basis"))?;

    // Point-normal circle fit is stable on short angular arcs. The signed
    // radius naturally handles inward-facing cavity meshes.
    let solution = point_normal_circle(samples, centroid, axis)
        .map(|(x, y, r)| vec![x, y, r])
        .or_else(|| projected_circle(samples, centroid, axis).map(|(x, y, r)| vec![x, y, r]))
        .ok_or_else(|| fit_error(SurfaceType::Cylinder, "singular transverse circle fit"))?;
    let axis_origin = centroid + u * solution[0] + v * solution[1];
    let radius = solution[2].abs();
    if !radius.is_finite()
        || radius <= numerical::MIN_RADIUS_RELATIVE_TO_SCALE * scale
        || radius > numerical::MAX_RADIUS_RELATIVE_TO_SCALE * scale
    {
        return Err(fit_error(
            SurfaceType::Cylinder,
            "non-positive or unobservable radius",
        ));
    }
    Ok(AnalyticSurface::Cylinder(CylinderSurface {
        axis_origin,
        axis,
        radius,
    }))
}

struct CylinderInitializationSeeds {
    primary: AnalyticSurface,
    alternatives: Vec<AnalyticSurface>,
}

fn initialize_cylinder_seeds(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<CylinderInitializationSeeds, RecognitionError> {
    let primary = initialize_cylinder(samples, centroid, scale)?;
    let moment = normal_second_moment(samples).ok_or_else(|| {
        fit_error(
            SurfaceType::Cylinder,
            "cylinder initialization requires normals",
        )
    })?;
    let (normal_values, _) = eigen_symmetric3(moment);
    let normal_rank = normal_values[1]
        / normal_values[2]
            .abs()
            .max(scalar::POSITIVE_DENOMINATOR_FLOOR);
    if normal_rank > numerical::CYLINDER_NARROW_FAN_MULTISTART_RANK_MAX {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    }

    let (point_values, point_vectors) = eigen_symmetric3(point_covariance(samples, centroid));
    let point_rank = point_values[1].abs()
        / point_values[2]
            .abs()
            .max(scalar::POSITIVE_DENOMINATOR_FLOOR);
    if point_rank > numerical::CYLINDER_GENERATOR_POINT_RANK_MAX {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    }
    let Some(axis) = point_vectors[2].normalized().map(Vec3::canonicalized) else {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    };
    let mut normal_weight = 0.0;
    let mut squared_dot = 0.0;
    for sample in samples {
        if let Some(normal) = sample.normal.and_then(Vec3::normalized) {
            normal_weight += sample.weight;
            squared_dot += sample.weight * normal.dot(axis).powi(2);
        }
    }
    if normal_weight <= 0.0
        || (squared_dot / normal_weight).sqrt()
            > numerical::CYLINDER_GENERATOR_NORMAL_ORTHOGONALITY_MAX
    {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    }
    let Some((x, y, signed_radius)) = point_normal_circle(samples, centroid, axis) else {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    };
    let Some((u, v)) = axis.orthonormal_basis() else {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    };
    let radius = signed_radius.abs();
    if !radius.is_finite()
        || radius <= numerical::MIN_RADIUS_RELATIVE_TO_SCALE * scale
        || radius > numerical::MAX_RADIUS_RELATIVE_TO_SCALE * scale
    {
        return Ok(CylinderInitializationSeeds {
            primary,
            alternatives: Vec::new(),
        });
    }
    let alternative = AnalyticSurface::Cylinder(CylinderSurface {
        axis_origin: centroid + u * x + v * y,
        axis,
        radius,
    });
    Ok(CylinderInitializationSeeds {
        primary,
        alternatives: vec![alternative],
    })
}

fn point_normal_circle(
    samples: &[FitObservation],
    origin: Vec3,
    axis: Vec3,
) -> Option<(f64, f64, f64)> {
    let (u, v) = axis.orthonormal_basis()?;
    let mut total = 0.0;
    let mut mean_q = [0.0; 2];
    let mut mean_n = [0.0; 2];
    let mut projected = Vec::with_capacity(samples.len());
    for sample in samples {
        let normal = sample.normal?;
        let normal = (normal - axis * normal.dot(axis)).normalized()?;
        let q = sample.point - origin;
        let q = [q.dot(u), q.dot(v)];
        let n = [normal.dot(u), normal.dot(v)];
        total += sample.weight;
        mean_q[0] += sample.weight * q[0];
        mean_q[1] += sample.weight * q[1];
        mean_n[0] += sample.weight * n[0];
        mean_n[1] += sample.weight * n[1];
        projected.push((q, n, sample.weight));
    }
    if total <= 0.0 {
        return None;
    }
    for coordinate in 0..2 {
        mean_q[coordinate] /= total;
        mean_n[coordinate] /= total;
    }
    // Centering analytically eliminates the two circle-center coordinates.
    // This avoids forming an ill-conditioned 3x3 normal equation when a CAD
    // face spans only a tiny cylinder arc.
    let mut covariance = 0.0;
    let mut normal_variance = 0.0;
    for (q, n, weight) in projected {
        let dq = [q[0] - mean_q[0], q[1] - mean_q[1]];
        let dn = [n[0] - mean_n[0], n[1] - mean_n[1]];
        covariance += weight * (dq[0] * dn[0] + dq[1] * dn[1]);
        normal_variance += weight * (dn[0] * dn[0] + dn[1] * dn[1]);
    }
    if normal_variance <= numerical::NORMAL_VARIANCE_RELATIVE_MIN * total {
        return None;
    }
    let signed_radius = covariance / normal_variance;
    signed_radius.is_finite().then_some((
        mean_q[0] - signed_radius * mean_n[0],
        mean_q[1] - signed_radius * mean_n[1],
        signed_radius,
    ))
}

fn initialize_cone(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<AnalyticSurface, RecognitionError> {
    // All tangent planes of a cone contain its apex.
    // Solve the centered tangent-plane system directly with pivoted QR rather
    // than forming normal equations or adding a fixed diagonal regularizer.
    // On a shallow, tightly trimmed CAD patch the observable apex can be
    // thousands of patch diameters away and the last singular value is
    // legitimately tiny. Squaring that condition number or regularizing it
    // biases the apex back toward the patch, which can make exact derivative
    // normals appear to support a different cone.
    let normal_weight: f64 = samples
        .iter()
        .filter(|sample| sample.normal.is_some())
        .map(|sample| sample.weight)
        .sum();
    if !normal_weight.is_finite() || normal_weight <= 0.0 {
        return Err(fit_error(
            SurfaceType::Cone,
            "cone initialization requires normals",
        ));
    }
    let mut tangent_rows = Vec::new();
    let mut tangent_rhs = Vec::new();
    for sample in samples {
        if let Some(n) = sample.normal.and_then(Vec3::normalized) {
            let weight = (sample.weight / normal_weight).sqrt();
            tangent_rows.push(vec![n.x * weight, n.y * weight, n.z * weight]);
            tangent_rhs.push(n.dot(sample.point - centroid) * weight);
        }
    }
    // A numerically rounded rank-two tangent fan can acquire a tiny positive
    // eigenvalue and appear full rank. Prefer the directly observed
    // two-generator intersection when that intersection is well conditioned.
    let generator_apex = sparse_cone_apex_from_two_dominant_generators(samples, centroid, scale);
    let apex = if let Some(apex) = generator_apex {
        apex
    } else if let Some(apex_offset) = least_squares_qr(
        &tangent_rows,
        &tangent_rhs,
        numerical::TANGENT_FAN_SINGULAR_VALUE_RELATIVE_MIN,
    ) {
        centroid + Vec3::new(apex_offset[0], apex_offset[1], apex_offset[2])
    } else {
        return near_cylindrical_cone_seed(samples, centroid, scale).ok_or_else(|| {
            fit_error(SurfaceType::Cone, "tangent planes do not determine an apex")
        });
    };
    if apex.distance(centroid) > scale * numerical::MAX_OBSERVABLE_APEX_SCALE {
        return Err(fit_error(
            SurfaceType::Cone,
            "apex is numerically unobservable",
        ));
    }

    // Once the apex is known, combine generator directions with normals.
    // For a cone, q_hat*cos(angle) +/- n*sin(angle) is the same axis at every
    // observation (the sign accommodates globally reversed mesh normals).
    // Fitting this relation is substantially better conditioned than taking
    // the null vector of centered normals alone on a very short azimuth arc.
    if let Some((axis, half_angle)) = cone_axis_angle_from_apex(samples, apex) {
        return Ok(AnalyticSurface::Cone(ConeSurface {
            apex,
            axis,
            half_angle,
        }));
    }

    let (_, covariance) = normal_statistics(samples)
        .ok_or_else(|| fit_error(SurfaceType::Cone, "cone initialization requires normals"))?;
    let (values, vectors) = eigen_symmetric3(covariance);
    if values[1]
        <= numerical::CONE_NORMAL_FAN_RANK_RELATIVE
            * values[2].abs().max(scalar::POSITIVE_DENOMINATOR_FLOOR)
    {
        return Err(fit_error(
            SurfaceType::Cone,
            "normal fan does not determine an axis",
        ));
    }
    let mut axis = vectors[0]
        .normalized()
        .ok_or_else(|| fit_error(SurfaceType::Cone, "invalid axis"))?;
    if (centroid - apex).dot(axis) < 0.0 {
        axis = -axis;
    }
    let half_angle = weighted_mean(samples, |s| {
        let q = s.point - apex;
        let h = q.dot(axis);
        (q - axis * h).length().atan2(h.max(0.0))
    })
    .clamp(MIN_ANGLE, MAX_ANGLE);
    Ok(AnalyticSurface::Cone(ConeSurface {
        apex,
        axis,
        half_angle,
    }))
}

/// Recover a cone apex when a sparse ruled tessellation has exactly two
/// dominant generator groups. In that case the tangent-plane normal system
/// is rank deficient: each generator has one constant analytic normal, so its
/// repeated tangent planes contribute no information along their common
/// intersection. The vertex positions still observe the carrier, because
/// the distinct generator lines meet at the apex. Numerically negligible
/// sliver groups may be ignored here, but all observations remain in the
/// downstream axis/angle fit and refinement. Final fit acceptance still gates
/// maximum positional residual and area-weighted RMS normal residual across
/// the complete observation set.
fn sparse_cone_apex_from_two_dominant_generators(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Option<Vec3> {
    struct NormalGroup {
        normal_sum: Vec3,
        weight: f64,
        samples: Vec<FitObservation>,
    }

    let mut groups: Vec<NormalGroup> = Vec::new();
    for sample in samples {
        let normal = sample.normal.and_then(Vec3::normalized)?;
        let matching = groups.iter().position(|group| {
            group
                .normal_sum
                .normalized()
                .is_some_and(|mean| mean.dot(normal) >= 1.0 - numerical::NORMAL_GROUP_COSINE_GAP)
        });
        if let Some(index) = matching {
            let group = &mut groups[index];
            group.normal_sum += normal * sample.weight;
            group.weight += sample.weight;
            group.samples.push(*sample);
        } else {
            groups.push(NormalGroup {
                normal_sum: normal * sample.weight,
                weight: sample.weight,
                samples: vec![*sample],
            });
        }
    }
    let total_group_weight: f64 = groups.iter().map(|group| group.weight).sum();
    // The affected STEP faces contain additional zero-area/sliver groups at
    // only 9.15e-14 and 5.94e-15 of represented area. This named, scale-free
    // threshold ignores those numerical artifacts while treating any group
    // at or above 1e-10 of represented area as geometrically meaningful.
    let negligible_group_weight = total_group_weight * numerical::NEGLIGIBLE_GROUP_WEIGHT_FRACTION;

    let mut generator_lines = Vec::new();
    for group in groups {
        if group.weight < negligible_group_weight {
            continue;
        }
        if group.samples.len() < 2 || group.weight <= 0.0 {
            return None;
        }
        let center = weighted_centroid(&group.samples)?;
        let (values, vectors) = eigen_symmetric3(point_covariance(&group.samples, center));
        let line_variance = values[2];
        if !line_variance.is_finite()
            || line_variance
                <= group.weight * scale * scale * numerical::GENERATOR_LINE_VARIANCE_RELATIVE_MIN
            || values[1].abs() > line_variance * numerical::GENERATOR_SECOND_VARIANCE_RELATIVE_MAX
        {
            return None;
        }
        let direction = vectors[2].normalized()?;
        let normal = group.normal_sum.normalized()?;
        if direction.dot(normal).abs() > numerical::GENERATOR_NORMAL_ORTHOGONALITY_MAX {
            return None;
        }
        generator_lines.push((center, direction, group.weight));
    }
    // Both normal groups must independently describe a generator line.
    if generator_lines.len() != 2 {
        return None;
    }

    // Area-weighted least-squares intersection of the recovered lines. A
    // line contributes (I - d d^T)(apex - center) = 0.
    let mut matrix = [[0.0; 3]; 3];
    let mut rhs = Vec3::ZERO;
    for (center, direction, weight) in generator_lines {
        let offset = center - centroid;
        for (row, matrix_row) in matrix.iter_mut().enumerate() {
            for (column, entry) in matrix_row.iter_mut().enumerate() {
                let identity = f64::from(row == column);
                *entry +=
                    weight * (identity - direction.component(row) * direction.component(column));
            }
        }
        rhs += (offset - direction * offset.dot(direction)) * weight;
    }
    let (values, vectors) = eigen_symmetric3(matrix);
    let largest = values[2].abs();
    if !largest.is_finite()
        || largest <= 0.0
        || values.iter().any(|&value| {
            !value.is_finite()
                || value <= largest * numerical::GENERATOR_INTERSECTION_RANK_RELATIVE_MIN
        })
    {
        return None;
    }
    let apex_offset = values
        .iter()
        .zip(&vectors)
        .fold(Vec3::ZERO, |offset, (&value, &direction)| {
            offset + direction * (direction.dot(rhs) / value)
        });
    let apex = centroid + apex_offset;
    (apex.is_finite() && apex.distance(centroid) <= scale * numerical::MAX_OBSERVABLE_APEX_SCALE)
        .then_some(apex)
}

fn cone_axis_angle_from_apex(samples: &[FitObservation], apex: Vec3) -> Option<(Vec3, f64)> {
    let mut observations = Vec::with_capacity(samples.len());
    let mut total = 0.0;
    let mut mean_q = Vec3::ZERO;
    let mut mean_n = Vec3::ZERO;
    for sample in samples {
        let q = (sample.point - apex).normalized()?;
        let n = sample.normal.and_then(Vec3::normalized)?;
        observations.push((q, n, sample.weight));
        total += sample.weight;
        mean_q += q * sample.weight;
        mean_n += n * sample.weight;
    }
    if total <= 0.0 {
        return None;
    }
    mean_q = mean_q / total;
    mean_n = mean_n / total;
    let mut q_variance = 0.0;
    let mut n_variance = 0.0;
    let mut covariance = 0.0;
    for &(q, n, weight) in &observations {
        let dq = q - mean_q;
        let dn = n - mean_n;
        q_variance += weight * dq.length_squared();
        n_variance += weight * dn.length_squared();
        covariance += weight * dq.dot(dn);
    }
    let mut best: Option<(f64, Vec3, f64)> = None;
    for normal_sign in [-1.0, 1.0] {
        let cross = normal_sign * covariance;
        let discriminant = (q_variance - n_variance).hypot(2.0 * cross);
        let eigenvalue = 0.5 * (q_variance + n_variance - discriminant);
        let mut cosine = -cross;
        let mut sine = q_variance - eigenvalue;
        if cosine.abs() + sine.abs() <= numerical::TRIGONOMETRIC_MAGNITUDE_FLOOR {
            cosine = n_variance - eigenvalue;
            sine = -cross;
        }
        if cosine < 0.0 {
            cosine = -cosine;
            sine = -sine;
        }
        if sine <= 0.0 {
            continue;
        }
        let norm = cosine.hypot(sine);
        let cosine = cosine / norm;
        let sine = sine / norm;
        let half_angle = sine.atan2(cosine);
        if !(MIN_ANGLE..MAX_ANGLE).contains(&half_angle) {
            continue;
        }
        let axis_sum = observations
            .iter()
            .fold(Vec3::ZERO, |sum, &(q, n, weight)| {
                sum + (q * cosine + n * (normal_sign * sine)) * weight
            });
        let axis = axis_sum.normalized()?;
        let dispersion = observations
            .iter()
            .map(|&(q, n, weight)| {
                let candidate = q * cosine + n * (normal_sign * sine);
                weight * (candidate - axis).length_squared()
            })
            .sum::<f64>();
        if best
            .as_ref()
            .is_none_or(|&(best_dispersion, _, _)| dispersion < best_dispersion)
        {
            best = Some((dispersion, axis, half_angle));
        }
    }
    best.map(|(_, axis, angle)| (axis, angle))
}

fn near_cylindrical_cone_seed(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Option<AnalyticSurface> {
    let AnalyticSurface::Cylinder(cylinder) = initialize_cylinder(samples, centroid, scale).ok()?
    else {
        unreachable!()
    };
    // A cone approaches a cylinder as its apex recedes. This fallback is used
    // only when the tangent fan cannot numerically observe an apex. It gives a
    // KnownType cone fit a finite local parameterization, while generic model
    // selection still compares its measured score against the simpler,
    // directly fitted cylinder and therefore does not invent cone evidence.
    let coordinate_scale = centroid
        .x
        .abs()
        .max(centroid.y.abs())
        .max(centroid.z.abs())
        .max(scale)
        .max(scalar::GEOMETRIC_SCALE_FLOOR);
    let half_angle = MIN_ANGLE
        .max(
            (cylinder.radius
                / (coordinate_scale * numerical::NEAR_CYLINDRICAL_APEX_COORDINATE_SCALE_MAX))
                .atan(),
        )
        .min(MAX_ANGLE - MIN_ANGLE);
    Some(AnalyticSurface::Cone(ConeSurface {
        apex: cylinder.axis_origin - cylinder.axis * (cylinder.radius / half_angle.tan()),
        axis: cylinder.axis,
        half_angle,
    }))
}

fn initialize_torus(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<AnalyticSurface, RecognitionError> {
    Ok(initialize_torus_seeds(samples, centroid, scale)?.primary)
}

struct TorusInitializationSeeds {
    primary: AnalyticSurface,
    alternatives: Vec<AnalyticSurface>,
}

#[derive(Default)]
struct TorusSeedRanking {
    primary: Option<(AnalyticSurface, f64)>,
    axis_frontier: Vec<(AnalyticSurface, f64, f64)>,
    best_max_normal_chord: Option<f64>,
}

impl TorusSeedRanking {
    fn consider(&mut self, surface: AnalyticSurface, samples: &[FitObservation], scale: f64) {
        let position_loss = position_objective(surface, samples, scale);
        let combined_loss = torus_initialization_objective(surface, samples, scale);
        let normal_stats = oriented_normal_chord_loss(surface, samples);
        let normal_loss = normal_stats.map_or(f64::INFINITY, |loss| loss.0);
        if let Some((_, max_chord)) = normal_stats {
            if self.best_max_normal_chord.is_none_or(|old| max_chord < old) {
                self.best_max_normal_chord = Some(max_chord);
            }
        }
        if self
            .primary
            .is_none_or(|(_, old_loss)| combined_loss < old_loss)
        {
            self.primary = Some((surface, combined_loss));
        }
        let AnalyticSurface::Torus(torus) = surface else {
            unreachable!()
        };
        let matching_axis = self.axis_frontier.iter().position(|(candidate, _, _)| {
            let AnalyticSurface::Torus(candidate) = candidate else {
                unreachable!()
            };
            candidate.axis.dot(torus.axis).abs() >= numerical::TORUS_MULTISTART_AXIS_CLUSTER_COSINE
        });
        if let Some(index) = matching_axis {
            let (_, old_normal, old_position) = self.axis_frontier[index];
            if normal_loss < old_normal
                || (normal_loss == old_normal && position_loss < old_position)
            {
                self.axis_frontier[index] = (surface, normal_loss, position_loss);
            }
        } else {
            self.axis_frontier
                .push((surface, normal_loss, position_loss));
        }
    }

    fn finish(
        self,
        samples: &[FitObservation],
    ) -> Result<TorusInitializationSeeds, RecognitionError> {
        let Some((primary, _)) = self.primary else {
            return Err(fit_error(
                SurfaceType::Torus,
                "no stable axis/meridian initialization",
            ));
        };
        let mut alternatives = Vec::new();
        if self.needs_multistart(samples) {
            let AnalyticSurface::Torus(primary_torus) = primary else {
                unreachable!()
            };
            let mut frontier = self.axis_frontier;
            frontier.sort_by(|left, right| {
                left.1
                    .total_cmp(&right.1)
                    .then_with(|| left.2.total_cmp(&right.2))
            });
            for (candidate, _, _) in frontier {
                let AnalyticSurface::Torus(candidate_torus) = candidate else {
                    unreachable!()
                };
                if candidate_torus.axis.dot(primary_torus.axis).abs()
                    >= numerical::TORUS_MULTISTART_AXIS_CLUSTER_COSINE
                {
                    continue;
                }
                alternatives.push(candidate);
                if alternatives.len() + 1 >= numerical::TORUS_MAX_REFINEMENT_STARTS {
                    break;
                }
            }
        }
        Ok(TorusInitializationSeeds {
            primary,
            alternatives,
        })
    }

    fn needs_multistart(&self, samples: &[FitObservation]) -> bool {
        if self.primary.is_none() {
            return false;
        }
        self.needs_normal_disambiguation(samples)
            && torus_multistart_has_observable_curvature(samples)
    }

    fn needs_normal_disambiguation(&self, samples: &[FitObservation]) -> bool {
        let Some((primary, _)) = self.primary else {
            return false;
        };
        let primary_max_chord =
            oriented_normal_chord_loss(primary, samples).map_or(f64::INFINITY, |loss| loss.1);
        let lower = 1.0 - numerical::TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS.cos();
        let upper = 1.0 - numerical::TORUS_MULTISTART_NORMAL_FIDELITY_MAX_RADIANS.cos();
        primary_max_chord > lower
            && self
                .best_max_normal_chord
                .is_some_and(|best_max_chord| best_max_chord <= upper)
    }

    fn needs_roundoff_recovery(&self, samples: &[FitObservation]) -> bool {
        let Some((primary, _)) = self.primary else {
            return false;
        };
        if !torus_multistart_has_observable_curvature(samples) {
            return false;
        }
        let coordinate_scale = samples.iter().fold(1.0_f64, |largest, sample| {
            largest
                .max(sample.point.x.abs())
                .max(sample.point.y.abs())
                .max(sample.point.z.abs())
        });
        let roundoff = crate::numerical::recognition::EXACT_VERTEX_ROUNDOFF_MULTIPLIER
            * f64::EPSILON
            * coordinate_scale;
        samples
            .iter()
            .map(|sample| primary.signed_distance(sample.point).abs())
            .fold(0.0_f64, f64::max)
            > roundoff
    }
}

/// Require the observed normal field to span both toroidal curvature
/// directions before spending work on alternate full-data LM basins. The
/// position-ranked primary is deliberately irrelevant: rescuing a wrong
/// cylinder-limit primary is precisely why multistart exists.
fn torus_multistart_has_observable_curvature(samples: &[FitObservation]) -> bool {
    torus_normal_moment_rank(samples)
        .is_some_and(|rank| rank >= numerical::TORUS_MULTISTART_NORMAL_MOMENT_RANK_MIN)
}

/// Smallest normalized eigenvalue of the uncentered normal second moment.
/// The quantity is invariant under rotation, uniform weight scaling, and
/// global normal reversal. Its magnitude still depends on trim coverage.
fn torus_normal_moment_rank(samples: &[FitObservation]) -> Option<f64> {
    let mut moment = [[0.0; 3]; 3];
    let mut weight = 0.0;
    for sample in samples {
        let Some(normal) = sample.normal.and_then(Vec3::normalized) else {
            continue;
        };
        outer_accumulate(&mut moment, normal, sample.weight);
        weight += sample.weight;
    }
    if weight <= 0.0 {
        return None;
    }
    let (values, _) = eigen_symmetric3(moment);
    let trace = values.iter().sum::<f64>();
    (trace.is_finite() && trace > 0.0).then_some((values[0] / trace).max(0.0))
}

fn initialize_torus_seeds(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Result<TorusInitializationSeeds, RecognitionError> {
    let (_, ncov) = normal_statistics(samples)
        .ok_or_else(|| fit_error(SurfaceType::Torus, "torus initialization requires normals"))?;
    let (_, nvectors) = eigen_symmetric3(ncov);
    let mut moment = [[0.0; 3]; 3];
    for sample in samples {
        if let Some(n) = sample.normal.and_then(Vec3::normalized) {
            outer_accumulate(
                &mut moment,
                (sample.point - centroid).cross(n),
                sample.weight,
            );
        }
    }
    let (_, mvectors) = eigen_symmetric3(moment);
    let normal_offset_seeds = torus_coplanarity_seeds(samples, centroid, scale);
    let mut coplanarity_seeds = Vec::new();
    for &(axis, signed_radius) in &normal_offset_seeds {
        if coplanarity_seeds.iter().all(|(old, _): &(Vec3, f64)| {
            old.dot(axis).abs() < 1.0 - numerical::AXIS_SEED_COSINE_GAP
        }) {
            coplanarity_seeds.push((axis, signed_radius));
        }
    }
    let mut normal_offset_candidates = Vec::new();
    let mut seeds = vec![mvectors[0], nvectors[0], nvectors[1], nvectors[2]];
    seeds.extend(coplanarity_seeds.iter().map(|&(axis, _)| axis));
    let mut ranking = TorusSeedRanking::default();
    if let Some(surface) = initialize_torus_from_constant_latitude(samples, centroid, scale) {
        ranking.consider(surface, samples, scale);
    }
    for &(axis, signed_minor) in &coplanarity_seeds {
        if let Some(surface) =
            initialize_torus_from_two_centerline_clusters(samples, centroid, signed_minor, scale)
        {
            ranking.consider(surface, samples, scale);
        }
        if let Some(surface) =
            initialize_torus_from_signed_normal_offset(samples, centroid, axis, signed_minor, scale)
        {
            ranking.consider(surface, samples, scale);
        }
    }
    for seed in seeds {
        let Some(mut axis) = seed.normalized() else {
            continue;
        };
        axis = axis.canonicalized();
        // Thin tori are poorly conditioned in meridian coordinates: a small
        // axis error is magnified by the major/minor radius ratio. Recover the
        // tube centerline directly from p - r*n before trying the more general
        // normal-coplanarity initializer below.
        if let Some(surface) = initialize_torus_from_normal_offsets(samples, centroid, axis, scale)
        {
            ranking.consider(surface, samples, scale);
        }
        let mut axis_point = centroid;
        // Alternate the normal-line coplanarity center and axis estimates.
        for _ in 0..12 {
            let mut rows = Vec::new();
            let mut rhs = Vec::new();
            for sample in samples {
                let Some(n) = sample.normal.and_then(Vec3::normalized) else {
                    continue;
                };
                let wv = n.cross(axis);
                let w = sample.weight.sqrt();
                rows.push(vec![wv.x * w, wv.y * w, wv.z * w]);
                rhs.push((sample.point - centroid).cross(n).dot(axis) * w);
            }
            // Gauge pin along the current axis.
            let reg = numerical::TORUS_AXIS_GAUGE_WEIGHT;
            rows.push(vec![axis.x * reg, axis.y * reg, axis.z * reg]);
            rhs.push(0.0);
            let Some(c) = least_squares(&rows, &rhs, numerical::ALGEBRAIC_REGULARIZATION_RELATIVE)
            else {
                break;
            };
            axis_point = centroid + Vec3::new(c[0], c[1], c[2]);
            let mut m = [[0.0; 3]; 3];
            for sample in samples {
                if let Some(n) = sample.normal.and_then(Vec3::normalized) {
                    outer_accumulate(&mut m, (sample.point - axis_point).cross(n), sample.weight);
                }
            }
            axis = eigen_symmetric3(m).1[0]
                .normalized()
                .unwrap_or(axis)
                .canonicalized();
        }
        let Some((major, z0, minor)) = meridian_circle(samples, axis_point, axis) else {
            continue;
        };
        if major <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
            || minor <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
        {
            continue;
        }
        let surface = AnalyticSurface::Torus(TorusSurface {
            center: axis_point + axis * z0,
            axis,
            major_radius: major,
            minor_radius: minor,
        });
        ranking.consider(surface, samples, scale);
    }
    if ranking.needs_normal_disambiguation(samples) || ranking.needs_roundoff_recovery(samples) {
        for &(axis, signed_minor) in &normal_offset_seeds {
            if let Some(surface) =
                refine_torus_normal_offset_radius(samples, centroid, axis, signed_minor, scale)
            {
                let normal_fidelity_max =
                    1.0 - numerical::TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS.cos();
                if oriented_normal_chord_loss(surface, samples)
                    .is_some_and(|(_, max_chord)| max_chord <= normal_fidelity_max)
                {
                    ranking.consider(surface, samples, scale);
                    normal_offset_candidates.push(surface);
                }
            }
        }
    }
    let mut seeds = ranking.finish(samples)?;
    normal_offset_candidates.sort_by(|left, right| {
        torus_initialization_objective(*left, samples, scale)
            .total_cmp(&torus_initialization_objective(*right, samples, scale))
    });
    for candidate in normal_offset_candidates {
        let AnalyticSurface::Torus(candidate_torus) = candidate else {
            unreachable!()
        };
        let same_basin = std::iter::once(seeds.primary)
            .chain(seeds.alternatives.iter().copied())
            .any(|existing| {
                let AnalyticSurface::Torus(existing) = existing else {
                    unreachable!()
                };
                existing.axis.dot(candidate_torus.axis).abs()
                    >= numerical::TORUS_MULTISTART_AXIS_CLUSTER_COSINE
                    && existing.center.distance(candidate_torus.center)
                        <= numerical::TORUS_NORMAL_OFFSET_BASIN_RELATIVE_GAP * scale
                    && (existing.major_radius - candidate_torus.major_radius).abs()
                        <= numerical::TORUS_NORMAL_OFFSET_BASIN_RELATIVE_GAP * scale
                    && (existing.minor_radius - candidate_torus.minor_radius).abs()
                        <= numerical::TORUS_NORMAL_OFFSET_BASIN_RELATIVE_GAP * scale
            });
        if !same_basin {
            seeds.alternatives.push(candidate);
            if seeds.alternatives.len() + 1 >= numerical::TORUS_MAX_REFINEMENT_STARTS {
                break;
            }
        }
    }
    Ok(seeds)
}

/// Recover a torus when normal offsets expose exactly two major-centerline
/// clusters.
///
/// Two complete meridian rings are position-wise cospherical, and a circle
/// through their two centerline points is underdetermined. Their normal fans
/// nevertheless determine the tangent of the major circle at each cluster.
/// The cross product of those tangents gives the torus axis, and the two
/// radial lines then determine the major-circle center. This path is enabled
/// only for a rank-one centerline cloud with two nonempty sides, so ordinary
/// multi-azimuth circle fitting remains unchanged.
fn initialize_torus_from_two_centerline_clusters(
    samples: &[FitObservation],
    centroid: Vec3,
    signed_minor: f64,
    scale: f64,
) -> Option<AnalyticSurface> {
    if !signed_minor.is_finite()
        || signed_minor.abs() <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
    {
        return None;
    }
    let offset_samples: Vec<_> = samples
        .iter()
        .filter_map(|sample| {
            let normal = sample.normal.and_then(Vec3::normalized)?;
            Some((
                (sample.point - centroid) - normal * signed_minor,
                normal,
                sample.weight,
            ))
        })
        .collect();
    if offset_samples.len() < 6 {
        return None;
    }
    let total: f64 = offset_samples.iter().map(|sample| sample.2).sum();
    if total <= 0.0 {
        return None;
    }
    let line_centroid = offset_samples
        .iter()
        .fold(Vec3::ZERO, |sum, sample| sum + sample.0 * sample.2)
        / total;
    let mut covariance = [[0.0; 3]; 3];
    for &(point, _, weight) in &offset_samples {
        outer_accumulate(&mut covariance, point - line_centroid, weight);
    }
    let (values, vectors) = eigen_symmetric3(covariance);
    let line_variance = values[2].abs();
    if !line_variance.is_finite()
        || line_variance <= total * scale * scale * numerical::NORMAL_VARIANCE_RELATIVE_MIN
        || values[1].abs() > line_variance * numerical::CIRCLE_QR_SPREAD_RELATIVE_MIN
    {
        return None;
    }
    let split_axis = vectors[2].normalized()?;

    let mut groups = [Vec::new(), Vec::new()];
    for sample in offset_samples {
        let projection = (sample.0 - line_centroid).dot(split_axis);
        groups[usize::from(projection >= 0.0)].push(sample);
    }
    if groups.iter().any(|group| group.len() < 3) {
        return None;
    }

    let mut centers = [Vec3::ZERO; 2];
    let mut tangents = [Vec3::ZERO; 2];
    let mut group_weights = [0.0; 2];
    for (index, group) in groups.iter().enumerate() {
        let weight: f64 = group.iter().map(|sample| sample.2).sum();
        if weight <= 0.0 {
            return None;
        }
        group_weights[index] = weight;
        centers[index] = group
            .iter()
            .fold(Vec3::ZERO, |sum, sample| sum + sample.0 * sample.2)
            / weight;
        let mut normal_moment = [[0.0; 3]; 3];
        for &(_, normal, sample_weight) in group {
            outer_accumulate(&mut normal_moment, normal, sample_weight);
        }
        let (normal_values, normal_vectors) = eigen_symmetric3(normal_moment);
        if normal_values[1] <= normal_values[2].abs() * numerical::NORMAL_VARIANCE_RELATIVE_MIN {
            return None;
        }
        tangents[index] = normal_vectors[0].normalized()?;
    }
    let within_cluster_variance = groups
        .iter()
        .enumerate()
        .flat_map(|(index, group)| {
            group
                .iter()
                .map(move |sample| sample.2 * (sample.0 - centers[index]).length_squared())
        })
        .sum::<f64>();
    let between_cluster_variance = centers
        .iter()
        .zip(group_weights)
        .map(|(&center, weight)| weight * (center - line_centroid).length_squared())
        .sum::<f64>();
    if between_cluster_variance <= 0.0
        || within_cluster_variance
            > between_cluster_variance * numerical::CIRCLE_QR_SPREAD_RELATIVE_MIN
    {
        return None;
    }

    let axis = tangents[0].cross(tangents[1]).normalized()?.canonicalized();
    let radial = [
        axis.cross(tangents[0]).normalized()?,
        axis.cross(tangents[1]).normalized()?,
    ];
    if radial[0].cross(radial[1]).length() <= numerical::GENERATOR_INTERSECTION_RANK_RELATIVE_MIN {
        return None;
    }

    // Weighted least-squares intersection of the two radial lines. Each line
    // contributes (I - e e^T)(center - point) = 0.
    let mut matrix = [[0.0; 3]; 3];
    let mut rhs = Vec3::ZERO;
    for index in 0..2 {
        let direction = radial[index];
        let weight = group_weights[index];
        let offset = centers[index];
        for (row, matrix_row) in matrix.iter_mut().enumerate() {
            for (column, entry) in matrix_row.iter_mut().enumerate() {
                let identity = f64::from(row == column);
                *entry +=
                    weight * (identity - direction.component(row) * direction.component(column));
            }
        }
        rhs += (offset - direction * offset.dot(direction)) * weight;
    }
    let (intersection_values, intersection_vectors) = eigen_symmetric3(matrix);
    let largest = intersection_values[2].abs();
    if !largest.is_finite()
        || largest <= 0.0
        || intersection_values.iter().any(|&value| {
            !value.is_finite()
                || value <= largest * numerical::GENERATOR_INTERSECTION_RANK_RELATIVE_MIN
        })
    {
        return None;
    }
    let center_offset = intersection_values
        .iter()
        .zip(&intersection_vectors)
        .fold(Vec3::ZERO, |sum, (&value, &direction)| {
            sum + direction * (direction.dot(rhs) / value)
        });
    let center = centroid + center_offset;
    let major_radius = group_weights
        .iter()
        .enumerate()
        .map(|(index, &weight)| {
            let offset = centers[index] - center_offset;
            weight * (offset - axis * offset.dot(axis)).length()
        })
        .sum::<f64>()
        / group_weights.iter().sum::<f64>();
    let minor_radius = signed_minor.abs();
    if !center.is_finite()
        || major_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
        || minor_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
    {
        return None;
    }
    Some(AnalyticSurface::Torus(TorusSurface {
        center,
        axis,
        major_radius,
        minor_radius,
    }))
}

/// Recover a finite torus witness for a constant-latitude ring.
///
/// Such a ring determines its axis, transverse center, radius, and tangent
/// field, but not the torus major/minor split or axial center. Choosing one
/// finite tube radius and preserving the measured radial/axial normal
/// components yields an exact sampled carrier. The validation layer can then
/// distinguish this genuine parameter ambiguity from a wrong fit.
fn initialize_torus_from_constant_latitude(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Option<AnalyticSurface> {
    // On a constant-latitude ring the centered normal covariance has the
    // carrier axis as its null direction. It is substantially better
    // conditioned than subtracting nearly equal world-coordinate heights.
    let total: f64 = samples
        .iter()
        .filter(|sample| sample.normal.is_some())
        .map(|sample| sample.weight)
        .sum();
    if total <= 0.0 {
        return None;
    }
    let mut point_moment = point_covariance(samples, centroid);
    for row in &mut point_moment {
        for value in row {
            *value /= total;
        }
    }
    let (_, point_vectors) = eigen_symmetric3(point_moment);
    let axis = point_vectors[0].normalized()?.canonicalized();
    let (cx, cy, ring_radius) = projected_circle(samples, centroid, axis)?;
    if ring_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale {
        return None;
    }
    let (basis_u, basis_v) = axis.orthonormal_basis()?;
    let axis_point = centroid + basis_u * cx + basis_v * cy;
    let mut mean_height = 0.0;
    let mut mean_normal_height = 0.0;
    let mut mean_normal_radial = 0.0;
    for sample in samples {
        let normal = sample.normal.and_then(Vec3::normalized)?;
        let offset = sample.point - axis_point;
        let height = offset.dot(axis);
        let radial = (offset - axis * height).normalized()?;
        mean_height += sample.weight * height;
        mean_normal_height += sample.weight * normal.dot(axis);
        mean_normal_radial += sample.weight * normal.dot(radial);
    }
    mean_height /= total;
    mean_normal_height /= total;
    mean_normal_radial /= total;
    let mut height_variance = 0.0;
    let mut normal_height_variance = 0.0;
    for sample in samples {
        let normal = sample.normal.and_then(Vec3::normalized)?;
        let height = (sample.point - axis_point).dot(axis);
        height_variance += sample.weight * (height - mean_height).powi(2);
        normal_height_variance += sample.weight * (normal.dot(axis) - mean_normal_height).powi(2);
    }
    if height_variance > numerical::TORUS_PLANAR_VARIANCE_RELATIVE_MAX * total * scale * scale
        || normal_height_variance > numerical::TORUS_PLANAR_VARIANCE_RELATIVE_MAX * total
    {
        return None;
    }

    let minor_radius = 0.25 * ring_radius;
    let major_radius = ring_radius - minor_radius * mean_normal_radial;
    if major_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale {
        return None;
    }
    Some(AnalyticSurface::Torus(TorusSurface {
        center: axis_point + axis * (mean_height - minor_radius * mean_normal_height),
        axis,
        major_radius,
        minor_radius,
    }))
}

/// Recover torus-axis seeds by making the normal-offset points coplanar.
///
/// For a torus, `q = p - r*n` lies on its major circle, so the centered
/// covariance of `q` has a zero eigenvalue in the axis direction.  Its
/// covariance is a quadratic 3x3 matrix in the signed tube radius `r`; after
/// accumulating the point/normal moments, testing radius candidates is
/// constant-time and does not add another pass over a dense mesh.
fn torus_coplanarity_seeds(
    samples: &[FitObservation],
    centroid: Vec3,
    scale: f64,
) -> Vec<(Vec3, f64)> {
    let Some((mean_normal, _)) = normal_statistics(samples) else {
        return Vec::new();
    };
    let mut pp = [[0.0; 3]; 3];
    let mut pn = [[0.0; 3]; 3];
    let mut nn = [[0.0; 3]; 3];
    let mut normal_weight = 0.0;
    for sample in samples {
        let Some(normal) = sample.normal.and_then(Vec3::normalized) else {
            continue;
        };
        let p = sample.point - centroid;
        let n = normal - mean_normal;
        let w = sample.weight;
        outer_accumulate(&mut pp, p, w);
        outer_accumulate(&mut nn, n, w);
        let pa = [p.x, p.y, p.z];
        let na = [n.x, n.y, n.z];
        for (row, &point_component) in pa.iter().enumerate() {
            for (column, &normal_component) in na.iter().enumerate() {
                pn[row][column] += w * point_component * normal_component;
            }
        }
        normal_weight += w;
    }
    if normal_weight <= 0.0 {
        return Vec::new();
    }
    let covariance = |signed_radius: f64| {
        let mut matrix = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                matrix[row][column] = pp[row][column]
                    - signed_radius * (pn[row][column] + pn[column][row])
                    + signed_radius * signed_radius * nn[row][column];
            }
        }
        matrix
    };
    // On a very small doubly curved patch, the exact coplanarity minimum can
    // be much narrower than one scale-invariant logarithmic grid cell.  The
    // local normal-to-position regression `dp = A dn` exposes the two
    // principal signed curvature radii without searching radius space.  Its
    // eigenvalues are initialization hints only: the established
    // coplanarity, normal-fidelity, refinement, and final acceptance gates
    // remain authoritative.
    let mut regression_seeds = Vec::new();
    let (normal_values, normal_vectors) = eigen_symmetric3(nn);
    let normal_reference = normal_values[2]
        .abs()
        .max(scalar::POSITIVE_DENOMINATOR_FLOOR);
    let observed: Vec<_> = (0..3)
        .filter(|&index| {
            normal_values[index]
                > numerical::TORUS_MULTISTART_NORMAL_MOMENT_RANK_MIN * normal_reference
        })
        .collect();
    if observed.len() >= 2 {
        let mut regression = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                for &index in &observed {
                    let direction = normal_vectors[index];
                    let projected_cross = pn[row][0] * direction.x
                        + pn[row][1] * direction.y
                        + pn[row][2] * direction.z;
                    regression[row][column] += projected_cross
                        * [direction.x, direction.y, direction.z][column]
                        / normal_values[index];
                }
            }
        }
        let mut symmetric_regression = [[0.0; 3]; 3];
        for row in 0..3 {
            for column in 0..3 {
                symmetric_regression[row][column] =
                    0.5 * (regression[row][column] + regression[column][row]);
            }
        }
        let (signed_radii, _) = eigen_symmetric3(symmetric_regression);
        let minimum_radius = scale * numerical::LOG_SEARCH_MIN.exp();
        let maximum_radius = scale * numerical::LOG_SEARCH_MAX.exp();
        for signed_radius in signed_radii {
            if !signed_radius.is_finite()
                || signed_radius.abs() < minimum_radius
                || signed_radius.abs() > maximum_radius
            {
                continue;
            }
            let matrix = covariance(signed_radius);
            let (values, vectors) = eigen_symmetric3(matrix);
            let trace = (values[0] + values[1] + values[2])
                .abs()
                .max(scalar::POSITIVE_DENOMINATOR_FLOOR);
            if values[0].max(0.0) / trace > numerical::TORUS_NORMAL_OFFSET_POSITION_LOSS_MAX {
                continue;
            }
            if let Some(axis) = vectors[0].normalized() {
                regression_seeds.push((axis, signed_radius));
            }
        }
    }
    let score = |log_radius: f64, sign: f64| {
        let matrix = covariance(sign * scale * log_radius.exp());
        let (values, _) = eigen_symmetric3(matrix);
        let trace = (values[0] + values[1] + values[2])
            .abs()
            .max(scalar::POSITIVE_DENOMINATOR_FLOOR);
        values[0].max(0.0) / trace
    };

    // The face scale is a patch extent, not a carrier radius. Cover very thin
    // tubes and very small patches of large carriers, then refine only local
    // minima of the cheap moment objective.
    const GRID_STEPS: usize = 81;
    let spacing = (numerical::LOG_SEARCH_MAX - numerical::LOG_SEARCH_MIN) / GRID_STEPS as f64;
    let mut minima = Vec::new();
    for sign in [-1.0, 1.0] {
        let values: Vec<_> = (0..=GRID_STEPS)
            .map(|index| {
                let x = numerical::LOG_SEARCH_MIN + spacing * index as f64;
                (x, score(x, sign))
            })
            .collect();
        for index in 0..values.len() {
            let left = index.saturating_sub(1);
            let right = (index + 1).min(values.len() - 1);
            if values[index].1 <= values[left].1 && values[index].1 <= values[right].1 {
                minima.push((values[index].1, sign, values[left].0, values[right].0));
            }
        }
    }
    minima.sort_by(|left, right| left.0.total_cmp(&right.0));
    minima.truncate(6);

    let mut seeds = Vec::new();
    for (_, sign, mut left, mut right) in minima {
        // Golden-section minimization in log-radius keeps the search
        // scale-invariant while retaining the sign carried by mesh normals.
        let ratio = 0.6180339887498949;
        let mut x1 = right - ratio * (right - left);
        let mut x2 = left + ratio * (right - left);
        let mut y1 = score(x1, sign);
        let mut y2 = score(x2, sign);
        for _ in 0..numerical::TORUS_COPLANARITY_REFINEMENT_STEPS {
            if y1 <= y2 {
                right = x2;
                x2 = x1;
                y2 = y1;
                x1 = right - ratio * (right - left);
                y1 = score(x1, sign);
            } else {
                left = x1;
                x1 = x2;
                y1 = y2;
                x2 = left + ratio * (right - left);
                y2 = score(x2, sign);
            }
        }
        let signed_radius = sign * scale * (0.5 * (left + right)).exp();
        let (_, vectors) = eigen_symmetric3(covariance(signed_radius));
        if let Some(axis) = vectors[0].normalized() {
            // Keep every bounded local radius minimum, including same-axis
            // and opposite-sign alternatives. Narrow torus patches can have
            // position-flat inner/outer gauges that only the observed normal
            // field distinguishes downstream. `minima` is already capped at
            // six, so retaining them cannot make the start set unbounded.
            seeds.push((axis, signed_radius));
        }
    }
    seeds.extend(regression_seeds);
    seeds
}

fn initialize_torus_from_signed_normal_offset(
    samples: &[FitObservation],
    centroid: Vec3,
    axis: Vec3,
    signed_minor: f64,
    scale: f64,
) -> Option<AnalyticSurface> {
    if !signed_minor.is_finite()
        || signed_minor.abs() <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
    {
        return None;
    }
    let centerline: Vec<_> = samples
        .iter()
        .filter_map(|sample| {
            let normal = sample.normal.and_then(Vec3::normalized)?;
            Some(FitObservation {
                point: sample.point - normal * signed_minor,
                normal: None,
                weight: sample.weight,
                triangle: sample.triangle,
            })
        })
        .collect();
    let line_centroid = weighted_centroid(&centerline)?;
    let (cx, cy, major_radius) = projected_circle(&centerline, centroid, axis)?;
    let (u, v) = axis.orthonormal_basis()?;
    let axial_offset = (line_centroid - centroid).dot(axis);
    let minor_radius = signed_minor.abs();
    if major_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
        || minor_radius <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
    {
        return None;
    }
    Some(AnalyticSurface::Torus(TorusSurface {
        center: centroid + u * cx + v * cy + axis * axial_offset,
        axis: axis.canonicalized(),
        major_radius,
        minor_radius,
    }))
}

/// Resolve a roundoff-flat coplanarity-radius minimum with the observed
/// normal field before LM refinement.
///
/// A very small inner-equator patch can make both the true tube radius and a
/// collapsed offset near `major_radius - minor_radius` appear coplanar. The
/// radius search remains bounded to the logarithmic neighborhood that
/// produced the seed, and candidates must already be position-plausible.
/// This is initialization only: the production LM solve below remains
/// position-only and the ordinary final residual gates are unchanged.
fn refine_torus_normal_offset_radius(
    samples: &[FitObservation],
    centroid: Vec3,
    seed_axis: Vec3,
    signed_seed: f64,
    scale: f64,
) -> Option<AnalyticSurface> {
    if !signed_seed.is_finite() || signed_seed.abs() <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
    {
        return None;
    }
    let axis = seed_axis.normalized()?.canonicalized();
    let candidate = |log_factor: f64| {
        let signed_minor = signed_seed * log_factor.exp();
        let centerline: Vec<_> = samples
            .iter()
            .filter_map(|sample| {
                let normal = sample.normal.and_then(Vec3::normalized)?;
                Some(FitObservation {
                    point: sample.point - normal * signed_minor,
                    normal: None,
                    weight: sample.weight,
                    triangle: sample.triangle,
                })
            })
            .collect();
        let dynamic_axis = weighted_centroid(&centerline).and_then(|centerline_centroid| {
            eigen_symmetric3(point_covariance(&centerline, centerline_centroid)).1[0].normalized()
        });
        [Some(axis), dynamic_axis]
            .into_iter()
            .flatten()
            .filter_map(|candidate_axis| {
                let surface = initialize_torus_from_signed_normal_offset(
                    samples,
                    centroid,
                    candidate_axis,
                    signed_minor,
                    scale,
                )?;
                let position_loss = position_objective(surface, samples, scale);
                if !position_loss.is_finite()
                    || position_loss > numerical::TORUS_NORMAL_OFFSET_POSITION_LOSS_MAX
                {
                    return None;
                }
                let normal_loss = oriented_normal_chord_loss(surface, samples)?.0;
                let score = normal_loss
                    + numerical::TORUS_NORMAL_OFFSET_POSITION_TIE_WEIGHT * position_loss;
                score.is_finite().then_some((surface, score))
            })
            .min_by(|left, right| left.1.total_cmp(&right.1))
    };

    let grid_steps = numerical::TORUS_NORMAL_OFFSET_GRID_STEPS;
    // The moment-based coplanarity objective can leave a small absolute
    // radius error on a very narrow patch.  In that regime the correct axis
    // and carrier occupy a correspondingly narrow basin: a coarse logarithmic
    // grid can step over it entirely.  Polish a scale-bounded neighborhood of
    // the coplanarity seed with a deterministic nested grid before considering
    // the wider search below.
    let mut local_center = signed_seed;
    let mut local_half_width = 0.01 * scale;
    let mut local_best: Option<(AnalyticSurface, f64)> = None;
    for _ in 0..4 {
        let spacing = 2.0 * local_half_width / grid_steps as f64;
        let mut best_radius = local_center;
        for index in 0..=grid_steps {
            let signed_minor = local_center - local_half_width + spacing * index as f64;
            if signed_minor.signum() != signed_seed.signum()
                || signed_minor.abs() <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
            {
                continue;
            }
            let Some((surface, score)) = candidate((signed_minor / signed_seed).ln()) else {
                continue;
            };
            if local_best.is_none_or(|(_, old_score)| score < old_score) {
                local_best = Some((surface, score));
                best_radius = signed_minor;
            }
        }
        local_center = best_radius;
        local_half_width = spacing;
    }
    if let Some((surface, _)) = local_best {
        let normal_fidelity_max = 1.0 - numerical::TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS.cos();
        if oriented_normal_chord_loss(surface, samples)
            .is_some_and(|(_, max_chord)| max_chord <= normal_fidelity_max)
        {
            return Some(surface);
        }
    }
    let initial_half_width = numerical::TORUS_NORMAL_OFFSET_INITIAL_LOG_HALF_WIDTH;
    let initial_spacing = 2.0 * initial_half_width / grid_steps as f64;
    let initial_best = (0..=grid_steps)
        .filter_map(|index| {
            let x = -initial_half_width + initial_spacing * index as f64;
            candidate(x).map(|(_, score)| (index, score))
        })
        .min_by(|left, right| left.1.total_cmp(&right.1));
    let half_width = if initial_best.is_some_and(|(index, _)| index == 0 || index == grid_steps) {
        numerical::TORUS_NORMAL_OFFSET_LOG_HALF_WIDTH
    } else {
        initial_half_width
    };
    let spacing = 2.0 * half_width / grid_steps as f64;
    let mut grid = Vec::with_capacity(grid_steps + 1);
    let mut best: Option<(usize, AnalyticSurface, f64)> = None;
    for index in 0..=grid_steps {
        let x = -half_width + spacing * index as f64;
        let evaluated = candidate(x);
        if let Some((surface, score)) = evaluated {
            if best.is_none_or(|(_, _, old_score)| score < old_score) {
                best = Some((index, surface, score));
            }
        }
        grid.push((
            x,
            evaluated.map(|(_, score)| score).unwrap_or(f64::INFINITY),
        ));
    }
    let (best_index, mut best_surface, mut best_score) = best?;
    let mut left = grid[best_index.saturating_sub(1)].0;
    let mut right = grid[(best_index + 1).min(grid_steps)].0;
    if right <= left {
        return Some(best_surface);
    }
    let ratio = 0.618_033_988_749_894_9;
    let mut x1 = right - ratio * (right - left);
    let mut x2 = left + ratio * (right - left);
    let mut first = candidate(x1);
    let mut second = candidate(x2);
    for _ in 0..numerical::TORUS_NORMAL_OFFSET_REFINEMENT_STEPS {
        let y1 = first.map_or(f64::INFINITY, |(_, score)| score);
        let y2 = second.map_or(f64::INFINITY, |(_, score)| score);
        if let Some((surface, score)) = first {
            if score < best_score {
                best_surface = surface;
                best_score = score;
            }
        }
        if let Some((surface, score)) = second {
            if score < best_score {
                best_surface = surface;
                best_score = score;
            }
        }
        if y1 <= y2 {
            right = x2;
            x2 = x1;
            second = first;
            x1 = right - ratio * (right - left);
            first = candidate(x1);
        } else {
            left = x1;
            x1 = x2;
            first = second;
            x2 = left + ratio * (right - left);
            second = candidate(x2);
        }
    }
    Some(best_surface)
}

/// Initialize a torus from its normal-offset tube centerline.
///
/// For consistently oriented normals, points on a torus obey
/// `centerline = point - signed_minor_radius * normal`. Along the torus axis,
/// this is a weighted linear regression of point height against normal height.
/// The recovered centerline points then determine the axis plane and major
/// circle without relying on a numerically fragile meridian circle fit.
fn initialize_torus_from_normal_offsets(
    samples: &[FitObservation],
    centroid: Vec3,
    seed_axis: Vec3,
    scale: f64,
) -> Option<AnalyticSurface> {
    let mut axis = seed_axis;
    let mut centerline = Vec::new();
    let mut best: Option<(AnalyticSurface, f64)> = None;
    for _ in 0..8 {
        let total: f64 = samples
            .iter()
            .filter(|sample| sample.normal.is_some())
            .map(|sample| sample.weight)
            .sum();
        if total <= 0.0 {
            return None;
        }
        let mean_normal_height = samples
            .iter()
            .filter_map(|sample| {
                sample
                    .normal
                    .and_then(Vec3::normalized)
                    .map(|normal| sample.weight * normal.dot(axis))
            })
            .sum::<f64>()
            / total;
        let mean_point_height = samples
            .iter()
            .filter(|sample| sample.normal.is_some())
            .map(|sample| sample.weight * (sample.point - centroid).dot(axis))
            .sum::<f64>()
            / total;
        let mut covariance = 0.0;
        let mut variance = 0.0;
        for sample in samples {
            let Some(normal) = sample.normal.and_then(Vec3::normalized) else {
                continue;
            };
            let dn = normal.dot(axis) - mean_normal_height;
            let dp = (sample.point - centroid).dot(axis) - mean_point_height;
            covariance += sample.weight * dn * dp;
            variance += sample.weight * dn * dn;
        }
        if variance <= numerical::TORUS_PLANAR_VARIANCE_RELATIVE_MAX * total {
            return None;
        }
        let signed_minor = covariance / variance;
        let axial_offset = mean_point_height - signed_minor * mean_normal_height;
        if !signed_minor.is_finite()
            || signed_minor.abs() <= numerical::TORUS_RADIUS_RELATIVE_MIN * scale
        {
            return None;
        }
        centerline.clear();
        centerline.extend(samples.iter().filter_map(|sample| {
            let normal = sample.normal.and_then(Vec3::normalized)?;
            Some(FitObservation {
                point: sample.point - normal * signed_minor,
                normal: None,
                weight: sample.weight,
                triangle: sample.triangle,
            })
        }));
        if let Some((cx, cy, major_radius)) = projected_circle(&centerline, centroid, axis) {
            let (u, v) = axis.orthonormal_basis()?;
            let minor_radius = signed_minor.abs();
            if major_radius > numerical::TORUS_RADIUS_RELATIVE_MIN * scale
                && minor_radius > numerical::TORUS_RADIUS_RELATIVE_MIN * scale
            {
                let surface = AnalyticSurface::Torus(TorusSurface {
                    center: centroid + u * cx + v * cy + axis * axial_offset,
                    axis: axis.canonicalized(),
                    major_radius,
                    minor_radius,
                });
                let loss = torus_initialization_objective(surface, samples, scale);
                if best.as_ref().is_none_or(|(_, old)| loss < *old) {
                    best = Some((surface, loss));
                }
            }
        }
        let line_centroid = weighted_centroid(&centerline)?;
        let (_, vectors) = eigen_symmetric3(point_covariance(&centerline, line_centroid));
        let next = vectors[0].normalized()?;
        if next.dot(axis) < 0.0 {
            axis = -next;
        } else {
            axis = next;
        }
    }
    best.map(|(surface, _)| surface)
}

fn weighted_mean(samples: &[FitObservation], value: impl Fn(&FitObservation) -> f64) -> f64 {
    let total: f64 = samples.iter().map(|s| s.weight).sum();
    samples.iter().map(|s| s.weight * value(s)).sum::<f64>() / total
}

fn orient_from_normals(mut direction: Vec3, samples: &[FitObservation]) -> Vec3 {
    let alignment: f64 = samples
        .iter()
        .filter_map(|s| s.normal.map(|n| s.weight * n.dot(direction)))
        .sum();
    if alignment < 0.0 {
        direction = -direction;
    }
    direction
}

fn projected_circle(
    samples: &[FitObservation],
    origin: Vec3,
    axis: Vec3,
) -> Option<(f64, f64, f64)> {
    let (u, v) = axis.orthonormal_basis()?;
    let points: Vec<(f64, f64, f64)> = samples
        .iter()
        .map(|s| {
            let q = s.point - origin;
            (q.dot(u), q.dot(v), s.weight)
        })
        .collect();
    fit_circle_2d(&points)
}

fn meridian_circle(
    samples: &[FitObservation],
    origin: Vec3,
    axis: Vec3,
) -> Option<(f64, f64, f64)> {
    let points: Vec<(f64, f64, f64)> = samples
        .iter()
        .map(|s| {
            let q = s.point - origin;
            let z = q.dot(axis);
            let rho = (q - axis * z).length();
            (rho, z, s.weight)
        })
        .collect();
    fit_circle_2d(&points)
}

fn fit_circle_2d(points: &[(f64, f64, f64)]) -> Option<(f64, f64, f64)> {
    if points.len() < 3 {
        return None;
    }
    let total: f64 = points.iter().map(|p| p.2).sum();
    if total <= 0.0 {
        return None;
    }
    let mx = points.iter().map(|p| p.0 * p.2).sum::<f64>() / total;
    let my = points.iter().map(|p| p.1 * p.2).sum::<f64>() / total;
    let mut rows = Vec::with_capacity(points.len());
    let mut rhs = Vec::with_capacity(points.len());
    for &(px, py, weight) in points {
        let x = px - mx;
        let y = py - my;
        // Normalize quadrature weights so the algebraic system is invariant
        // to the selected face area. Tiny CAD trims can otherwise fall below
        // the absolute rank threshold despite having a well-spread circle.
        let w = (weight / total).sqrt();
        rows.push(vec![x * w, y * w, w]);
        rhs.push((x * x + y * y) * w);
    }
    let circle = |solution: Vec<f64>| {
        let cx = 0.5 * solution[0];
        let cy = 0.5 * solution[1];
        let r2 = solution[2] + cx * cx + cy * cy;
        (r2.is_finite() && r2 > 0.0).then(|| (cx + mx, cy + my, r2.sqrt()))
    };
    let error = |candidate: (f64, f64, f64)| {
        points
            .iter()
            .map(|&(px, py, weight)| {
                weight * ((px - candidate.0).hypot(py - candidate.1) - candidate.2).powi(2)
            })
            .sum::<f64>()
            / total
    };
    let normal_equation =
        least_squares(&rows, &rhs, numerical::ALGEBRAIC_REGULARIZATION_RELATIVE).and_then(circle);
    let qr = least_squares_qr(&rows, &rhs, numerical::ALGEBRAIC_REGULARIZATION_RELATIVE)
        .and_then(circle);
    let covariance = points.iter().fold([0.0; 3], |mut sum, &(px, py, weight)| {
        let x = px - mx;
        let y = py - my;
        sum[0] += weight * x * x / total;
        sum[1] += weight * x * y / total;
        sum[2] += weight * y * y / total;
        sum
    });
    let trace = covariance[0] + covariance[2];
    let discriminant =
        ((covariance[0] - covariance[2]).powi(2) + 4.0 * covariance[1] * covariance[1]).sqrt();
    let spread_denominator = trace + discriminant;
    let spread_ratio = if spread_denominator > scalar::POSITIVE_DENOMINATOR_FLOOR {
        ((trace - discriminant) / spread_denominator).max(0.0)
    } else {
        0.0
    };
    // Preserve the established parameter gauge when the normal-equation
    // circle is already exact at coordinate-conditioned roundoff. On a very
    // narrow but full-rank arc, forming normal equations can instead erase
    // the weak curvature direction; select QR when it materially lowers the
    // actual radial residual.
    if spread_ratio < numerical::CIRCLE_QR_SPREAD_RELATIVE_MIN {
        return normal_equation;
    }
    if let Some(candidate) = normal_equation {
        let coordinate_scale = points
            .iter()
            .flat_map(|&(px, py, _)| [px.abs(), py.abs()])
            .chain([candidate.2])
            .fold(1.0_f64, f64::max);
        let exact_threshold = 256.0 * f64::EPSILON * coordinate_scale;
        if error(candidate).sqrt() <= exact_threshold {
            return Some(candidate);
        }
    }
    [normal_equation, qr]
        .into_iter()
        .flatten()
        .min_by(|left, right| error(*left).total_cmp(&error(*right)))
}

fn refine_plane(
    seed: AnalyticSurface,
    samples: &[FitObservation],
    fixed: ConstraintMask,
) -> Result<AnalyticSurface, RecognitionError> {
    let AnalyticSurface::Plane(mut plane) = seed else {
        unreachable!()
    };
    let centroid = weighted_centroid(samples).unwrap();
    if !fixed.axis_or_normal {
        if fixed.origin_or_center {
            let (values, vectors) = eigen_symmetric3(point_covariance(samples, plane.origin));
            if values[1]
                <= linear_algebra::MACHINE_RANK_RELATIVE_MIN
                    * values[2].abs().max(scalar::POSITIVE_DENOMINATOR_FLOOR)
            {
                return Err(fit_error(SurfaceType::Plane, "observations are collinear"));
            }
            plane.normal = orient_from_normals(
                vectors[0]
                    .normalized()
                    .ok_or_else(|| fit_error(SurfaceType::Plane, "invalid constrained normal"))?,
                samples,
            );
        } else {
            plane = match initialize_plane(samples, centroid)? {
                AnalyticSurface::Plane(p) => p,
                _ => unreachable!(),
            };
        }
    }
    if !fixed.origin_or_center {
        // Only the normal offset is observable; retain a deterministic point.
        let offset = weighted_mean(samples, |s| s.point.dot(plane.normal));
        plane.origin = centroid + plane.normal * (offset - centroid.dot(plane.normal));
    }
    Ok(AnalyticSurface::Plane(plane))
}

/// Local, non-redundant parameter perturbations. Axis changes use two tangent
/// coordinates and are renormalized after every accepted step.
fn active_parameters(surface: AnalyticSurface, fixed: ConstraintMask) -> Vec<usize> {
    match surface {
        AnalyticSurface::Plane(_) => vec![],
        AnalyticSurface::Sphere(_) => {
            groups(&[(0, 3, fixed.origin_or_center), (3, 1, fixed.radius)])
        }
        AnalyticSurface::Cylinder(_) => groups(&[
            (0, 2, fixed.origin_or_center),
            (2, 2, fixed.axis_or_normal),
            (4, 1, fixed.radius),
        ]),
        AnalyticSurface::Cone(_) => groups(&[
            (0, 3, fixed.origin_or_center),
            (3, 2, fixed.axis_or_normal),
            (5, 1, fixed.angle),
        ]),
        AnalyticSurface::Torus(_) => groups(&[
            (0, 3, fixed.origin_or_center),
            (3, 2, fixed.axis_or_normal),
            (5, 1, fixed.major_radius),
            (6, 1, fixed.radius),
        ]),
    }
}
fn groups(spec: &[(usize, usize, bool)]) -> Vec<usize> {
    let mut out = Vec::new();
    for &(start, len, fixed) in spec {
        if !fixed {
            out.extend(start..start + len);
        }
    }
    out
}

fn perturb(
    surface: AnalyticSurface,
    index: usize,
    delta: f64,
    centroid: Vec3,
) -> Option<AnalyticSurface> {
    Some(match surface {
        AnalyticSurface::Sphere(mut s) => {
            match index {
                0 => s.center.x += delta,
                1 => s.center.y += delta,
                2 => s.center.z += delta,
                3 => s.radius = (s.radius + delta).max(scalar::MIN_NORMALIZABLE_NORM),
                _ => return None,
            }
            AnalyticSurface::Sphere(s)
        }
        AnalyticSurface::Cylinder(mut s) => {
            let (u, v) = s.axis.orthonormal_basis()?;
            match index {
                0 => s.axis_origin += u * delta,
                1 => s.axis_origin += v * delta,
                2 => s.axis = (s.axis + u * delta).normalized()?,
                3 => s.axis = (s.axis + v * delta).normalized()?,
                4 => s.radius = (s.radius + delta).max(scalar::MIN_NORMALIZABLE_NORM),
                _ => return None,
            }
            s.axis_origin += s.axis * (centroid - s.axis_origin).dot(s.axis);
            AnalyticSurface::Cylinder(s)
        }
        AnalyticSurface::Cone(mut s) => {
            let (u, v) = s.axis.orthonormal_basis()?;
            match index {
                0 => s.apex.x += delta,
                1 => s.apex.y += delta,
                2 => s.apex.z += delta,
                3 => s.axis = (s.axis + u * delta).normalized()?,
                4 => s.axis = (s.axis + v * delta).normalized()?,
                5 => s.half_angle = (s.half_angle + delta).clamp(MIN_ANGLE, MAX_ANGLE),
                _ => return None,
            }
            AnalyticSurface::Cone(s)
        }
        AnalyticSurface::Torus(mut s) => {
            let (u, v) = s.axis.orthonormal_basis()?;
            match index {
                0 => s.center.x += delta,
                1 => s.center.y += delta,
                2 => s.center.z += delta,
                3 => s.axis = (s.axis + u * delta).normalized()?,
                4 => s.axis = (s.axis + v * delta).normalized()?,
                5 => s.major_radius = (s.major_radius + delta).max(scalar::MIN_NORMALIZABLE_NORM),
                6 => s.minor_radius = (s.minor_radius + delta).max(scalar::MIN_NORMALIZABLE_NORM),
                _ => return None,
            }
            AnalyticSurface::Torus(s)
        }
        AnalyticSurface::Plane(_) => return None,
    })
}

const MAX_COARSE_REFINEMENT_SAMPLES: usize = 8_192;
const MAX_FULL_DATA_POLISH_ITERATIONS: usize = 16;

fn stratified_refinement_samples(samples: &[FitObservation]) -> Vec<FitObservation> {
    debug_assert!(samples.len() > MAX_COARSE_REFINEMENT_SAMPLES);
    let last = samples.len() - 1;
    let denominator = MAX_COARSE_REFINEMENT_SAMPLES - 1;
    (0..MAX_COARSE_REFINEMENT_SAMPLES)
        .map(|index| samples[index * last / denominator])
        .collect()
}

fn refine_lm(
    seed: AnalyticSurface,
    samples: &[FitObservation],
    fixed: ConstraintMask,
    scale: f64,
    max_iterations: usize,
) -> Result<(AnalyticSurface, f64), RecognitionError> {
    if samples.len() <= MAX_COARSE_REFINEMENT_SAMPLES {
        return refine_lm_core(seed, samples, fixed, scale, max_iterations);
    }

    let polish_iterations = max_iterations.min(MAX_FULL_DATA_POLISH_ITERATIONS);
    let coarse_iterations = max_iterations - polish_iterations;
    if coarse_iterations == 0 {
        return refine_lm_core(seed, samples, fixed, scale, polish_iterations);
    }

    // LM cost is linear in observation count and can otherwise dominate dense
    // CAD tessellations. Use a stable, storage-order stratification for the
    // coarse solve, then always polish and validate on the complete weighted
    // observation set. Endpoints are retained and no randomness is involved.
    let coarse = stratified_refinement_samples(samples);
    let (coarse_surface, _) = refine_lm_core(seed, &coarse, fixed, scale, coarse_iterations)?;
    refine_lm_core(coarse_surface, samples, fixed, scale, polish_iterations)
}

fn refine_lm_core(
    seed: AnalyticSurface,
    samples: &[FitObservation],
    fixed: ConstraintMask,
    scale: f64,
    max_iterations: usize,
) -> Result<(AnalyticSurface, f64), RecognitionError> {
    let centroid = weighted_centroid(samples).unwrap();
    let active = active_parameters(seed, fixed);
    if active.is_empty() {
        return Ok((seed, 0.0));
    }
    let mut surface = seed;
    let mut loss = position_objective(surface, samples, scale);
    let mut lambda = numerical::LM_INITIAL_DAMPING;
    let mut final_improvement = 0.0;
    for _ in 0..max_iterations {
        let n = active.len();
        let mut ata = vec![vec![0.0; n]; n];
        let mut atb = vec![0.0; n];
        for sample in samples {
            let residual = surface.signed_distance(sample.point) / scale;
            let huber = if residual.abs() <= 1.0 {
                1.0
            } else {
                1.0 / residual.abs()
            };
            let weight = sample.weight * huber;
            let derivatives = signed_distance_derivatives(surface, sample.point, scale)
                .unwrap_or_else(|| {
                    numerical_distance_derivatives(surface, sample.point, centroid, scale)
                });
            for i in 0..n {
                let ji = derivatives[active[i]];
                atb[i] -= weight * ji * residual;
                for j in 0..n {
                    ata[i][j] += weight * ji * derivatives[active[j]];
                }
            }
        }
        for (i, row) in ata.iter_mut().enumerate().take(active.len()) {
            row[i] += lambda * row[i].abs().max(numerical::LM_DIAGONAL_FLOOR);
        }
        let Some(step) = crate::math::solve_linear(ata, atb) else {
            lambda *= numerical::LM_DAMPING_GROWTH;
            if lambda > numerical::LM_DAMPING_MAX {
                break;
            }
            continue;
        };
        let proposal = apply_lm_step(surface, &active, &step, centroid, scale)
            .ok_or_else(|| fit_error(surface.surface_type(), "invalid LM update"))?;
        let proposal_loss = position_objective(proposal, samples, scale);
        if proposal_loss.is_finite() && proposal_loss < loss {
            final_improvement =
                (loss - proposal_loss) / loss.max(scalar::POSITIVE_DENOMINATOR_FLOOR);
            surface = proposal;
            loss = proposal_loss;
            lambda = (lambda * numerical::LM_DAMPING_SHRINK).max(numerical::LM_DAMPING_MIN);
            let step_norm = step.iter().map(|x| x * x).sum::<f64>().sqrt();
            if step_norm < numerical::LM_STEP_CONVERGENCE
                || final_improvement < numerical::LM_RELATIVE_IMPROVEMENT_CONVERGENCE
            {
                break;
            }
        } else {
            lambda *= numerical::LM_DAMPING_GROWTH;
            if lambda > numerical::LM_DAMPING_MAX {
                break;
            }
        }
    }
    Ok((surface, final_improvement))
}

fn oriented_normal_sense(surface: AnalyticSurface, samples: &[FitObservation]) -> f64 {
    let alignment = samples
        .iter()
        .filter_map(|sample| {
            Some(
                sample.weight
                    * sample
                        .normal
                        .and_then(Vec3::normalized)?
                        .dot(surface.normal_at(sample.point)?),
            )
        })
        .sum::<f64>();
    if alignment < 0.0 {
        -1.0
    } else {
        1.0
    }
}

/// Derivatives of signed distance with respect to the local parameters used
/// by [`perturb`]. Translation/radius-like parameters are scaled when the LM
/// step is applied, so their raw distance derivatives are already the
/// derivative of the normalized residual. Axis entries are divided by the
/// observation scale because their LM steps are dimensionless radians.
///
/// Dense CAD faces commonly contain tens of thousands of vertices. Computing
/// these derivatives analytically avoids two complete carrier evaluations per
/// active parameter and per vertex. `None` is returned only at a carrier
/// singularity, where the existing central difference remains the safer
/// definition.
fn signed_distance_derivatives(
    surface: AnalyticSurface,
    point: Vec3,
    scale: f64,
) -> Option<[f64; 7]> {
    let mut out = [0.0; 7];
    // Preserve the previous central-difference behavior near constrained
    // parameter bounds. `perturb` clamps these values, so an unconstrained
    // analytic derivative would describe a step the optimizer cannot apply.
    let finite_difference_step =
        numerical::FINITE_DIFFERENCE_RELATIVE_STEP * scale.max(scalar::GEOMETRIC_SCALE_FLOOR);
    match surface {
        AnalyticSurface::Plane(_) => return Some(out),
        AnalyticSurface::Sphere(s) => {
            if s.radius - finite_difference_step <= scalar::MIN_NORMALIZABLE_NORM {
                return None;
            }
            let normal = (point - s.center).normalized()?;
            out[0] = -normal.x;
            out[1] = -normal.y;
            out[2] = -normal.z;
            out[3] = -1.0;
        }
        AnalyticSurface::Cylinder(s) => {
            if s.radius - finite_difference_step <= scalar::MIN_NORMALIZABLE_NORM {
                return None;
            }
            let q = point - s.axis_origin;
            let height = q.dot(s.axis);
            let normal = (q - s.axis * height).normalized()?;
            let (u, v) = s.axis.orthonormal_basis()?;
            out[0] = -normal.dot(u);
            out[1] = -normal.dot(v);
            out[2] = -height * normal.dot(u) / scale;
            out[3] = -height * normal.dot(v) / scale;
            out[4] = -1.0;
        }
        AnalyticSurface::Cone(s) => {
            if s.half_angle - finite_difference_step <= MIN_ANGLE
                || s.half_angle + finite_difference_step >= MAX_ANGLE
            {
                return None;
            }
            let q = point - s.apex;
            let height = q.dot(s.axis);
            let radial = (q - s.axis * height).normalized()?;
            let (sin, cos) = s.half_angle.sin_cos();
            let normal = radial * cos - s.axis * sin;
            let (u, v) = s.axis.orthonormal_basis()?;
            out[0] = -normal.x;
            out[1] = -normal.y;
            out[2] = -normal.z;
            out[3] = (-height * cos * radial.dot(u) - sin * q.dot(u)) / scale;
            out[4] = (-height * cos * radial.dot(v) - sin * q.dot(v)) / scale;
            let rho = (q - s.axis * height).length();
            out[5] = -rho * sin - height * cos;
        }
        AnalyticSurface::Torus(s) => {
            if s.major_radius - finite_difference_step <= scalar::MIN_NORMALIZABLE_NORM
                || s.minor_radius - finite_difference_step <= scalar::MIN_NORMALIZABLE_NORM
            {
                return None;
            }
            let q = point - s.center;
            let height = q.dot(s.axis);
            let radial_vector = q - s.axis * height;
            let rho = radial_vector.length();
            if rho <= scalar::MIN_NORMALIZABLE_NORM {
                return None;
            }
            let radial = radial_vector / rho;
            let meridian_r = rho - s.major_radius;
            let tube_radius = meridian_r.hypot(height);
            if tube_radius <= scalar::MIN_NORMALIZABLE_NORM {
                return None;
            }
            let normal = radial * (meridian_r / tube_radius) + s.axis * (height / tube_radius);
            let (u, v) = s.axis.orthonormal_basis()?;
            out[0] = -normal.x;
            out[1] = -normal.y;
            out[2] = -normal.z;
            out[3] = height * s.major_radius * radial.dot(u) / (tube_radius * scale);
            out[4] = height * s.major_radius * radial.dot(v) / (tube_radius * scale);
            out[5] = -meridian_r / tube_radius;
            out[6] = -1.0;
        }
    }
    Some(out)
}

fn numerical_distance_derivatives(
    surface: AnalyticSurface,
    point: Vec3,
    centroid: Vec3,
    scale: f64,
) -> [f64; 7] {
    let mut out = [0.0; 7];
    for parameter in active_parameters(surface, ConstraintMask::default()) {
        let axis_parameter = matches!(surface, AnalyticSurface::Cylinder(_))
            && (2..4).contains(&parameter)
            || matches!(
                surface,
                AnalyticSurface::Cone(_) | AnalyticSurface::Torus(_)
            ) && (3..5).contains(&parameter);
        let h = if axis_parameter {
            numerical::FINITE_DIFFERENCE_RELATIVE_STEP
        } else {
            numerical::FINITE_DIFFERENCE_RELATIVE_STEP * scale.max(scalar::GEOMETRIC_SCALE_FLOOR)
        };
        let plus = perturb(surface, parameter, h, centroid).unwrap();
        let minus = perturb(surface, parameter, -h, centroid).unwrap();
        out[parameter] = (plus.signed_distance(point) - minus.signed_distance(point)) / (2.0 * h);
        if axis_parameter {
            out[parameter] /= scale;
        }
    }
    out
}

/// Apply a local LM step using the same tangent basis used to build its
/// Jacobian. Applying the two axis coordinates one at a time can make
/// `orthonormal_basis` choose a different seed between coordinates (notably
/// near a coordinate axis), turning the second coordinate into a different
/// direction than the linear solve intended.
fn apply_lm_step(
    surface: AnalyticSurface,
    active: &[usize],
    step: &[f64],
    centroid: Vec3,
    scale: f64,
) -> Option<AnalyticSurface> {
    let axis_range = match surface {
        AnalyticSurface::Cylinder(_) => Some(2..4),
        AnalyticSurface::Cone(_) | AnalyticSurface::Torus(_) => Some(3..5),
        AnalyticSurface::Plane(_) | AnalyticSurface::Sphere(_) => None,
    };
    let mut proposal = surface;
    for (&parameter, &normalized_delta) in active.iter().zip(step) {
        if axis_range
            .as_ref()
            .is_some_and(|range| range.contains(&parameter))
        {
            continue;
        }
        proposal = perturb(proposal, parameter, normalized_delta * scale, centroid)?;
    }
    let Some(axis_range) = axis_range else {
        return Some(proposal);
    };
    if !active
        .iter()
        .any(|parameter| axis_range.contains(parameter))
    {
        return Some(proposal);
    }
    let delta = |parameter| {
        active
            .iter()
            .position(|&candidate| candidate == parameter)
            .map_or(0.0, |index| step[index])
    };
    let original_axis = match surface {
        AnalyticSurface::Cylinder(s) => s.axis,
        AnalyticSurface::Cone(s) => s.axis,
        AnalyticSurface::Torus(s) => s.axis,
        _ => unreachable!(),
    };
    let (u, v) = original_axis.orthonormal_basis()?;
    let axis = (original_axis + u * delta(axis_range.start) + v * delta(axis_range.start + 1))
        .normalized()?;
    match &mut proposal {
        AnalyticSurface::Cylinder(s) => {
            s.axis = axis;
            s.axis_origin += axis * (centroid - s.axis_origin).dot(axis);
        }
        AnalyticSurface::Cone(s) => s.axis = axis,
        AnalyticSurface::Torus(s) => s.axis = axis,
        _ => unreachable!(),
    }
    Some(proposal)
}

fn position_objective(surface: AnalyticSurface, samples: &[FitObservation], scale: f64) -> f64 {
    let total: f64 = samples.iter().map(|s| s.weight).sum();
    samples
        .iter()
        .map(|s| {
            let r = surface.signed_distance(s.point) / scale;
            let a = r.abs();
            s.weight * if a <= 1.0 { 0.5 * r * r } else { a - 0.5 }
        })
        .sum::<f64>()
        / total
}

fn cylinder_initialization_objective(
    surface: AnalyticSurface,
    samples: &[FitObservation],
    scale: f64,
) -> f64 {
    let position_loss = position_objective(surface, samples, scale);
    let Some((normal_loss, max_normal_chord_loss)) = oriented_normal_chord_loss(surface, samples)
    else {
        return position_loss;
    };
    let normal_trigger_chord = 1.0 - numerical::CYLINDER_SEED_NORMAL_EQUIVALENCE_RADIANS.cos();
    if max_normal_chord_loss > normal_trigger_chord {
        position_loss + numerical::CYLINDER_NORMAL_OBJECTIVE_WEIGHT * normal_loss
    } else {
        position_loss
    }
}

fn torus_initialization_objective(
    surface: AnalyticSurface,
    samples: &[FitObservation],
    scale: f64,
) -> f64 {
    let position_loss = position_objective(surface, samples, scale);

    // A sparse torus patch can be position-exact for multiple carriers. Two
    // complete meridian circles, for example, are also exactly cospherical.
    // Their observed normal field contains the missing first-order geometry,
    // so retain it when ranking initialization candidates. LM refinement stays
    // position-only after that carrier gauge has been chosen. One global
    // orientation sign accommodates consistently reversed meshes.
    let Some((normal_loss, max_normal_chord_loss)) = oriented_normal_chord_loss(surface, samples)
    else {
        return position_loss;
    };
    let normal_trigger_chord = 1.0 - numerical::TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS.cos();
    if max_normal_chord_loss > normal_trigger_chord {
        position_loss + numerical::TORUS_NORMAL_OBJECTIVE_WEIGHT * normal_loss
    } else {
        position_loss
    }
}

/// Area-weighted, globally oriented normal disagreement used only to select
/// a torus initialization basin. The LM objective remains position-only.
fn oriented_normal_chord_loss(
    surface: AnalyticSurface,
    samples: &[FitObservation],
) -> Option<(f64, f64)> {
    let sense = oriented_normal_sense(surface, samples);
    let mut normal_weight = 0.0;
    let mut max_normal_chord_loss = 0.0_f64;
    let normal_loss = samples
        .iter()
        .filter_map(|sample| {
            let observed = sample.normal.and_then(Vec3::normalized)?;
            let model = surface.normal_at(sample.point)?;
            normal_weight += sample.weight;
            let chord_loss = 1.0 - sense * observed.dot(model).clamp(-1.0, 1.0);
            max_normal_chord_loss = max_normal_chord_loss.max(chord_loss);
            Some(sample.weight * chord_loss)
        })
        .sum::<f64>();
    (normal_weight > 0.0).then_some((normal_loss / normal_weight, max_normal_chord_loss))
}

fn measure_surface(surface: AnalyticSurface, samples: &[FitObservation]) -> (i8, FitMetrics) {
    let total: f64 = samples.iter().map(|s| s.weight).sum();
    let mut sum_sq = 0.0;
    let mut max_error = 0.0_f64;
    let mut alignment = 0.0;
    for sample in samples {
        let r = surface.signed_distance(sample.point).abs();
        sum_sq += sample.weight * r * r;
        max_error = max_error.max(r);
        if let (Some(mesh), Some(model)) = (sample.normal, surface.normal_at(sample.point)) {
            alignment += sample.weight * mesh.dot(model);
        }
    }
    let orientation = if alignment < 0.0 { -1 } else { 1 };
    let sense = orientation as f64;
    let mut normal_sq = 0.0;
    let mut normal_max = 0.0_f64;
    let mut normal_weight = 0.0;
    for sample in samples {
        if let (Some(mesh), Some(model)) = (
            sample.normal.and_then(Vec3::normalized),
            surface.normal_at(sample.point),
        ) {
            let angle = (sense * mesh.dot(model)).clamp(-1.0, 1.0).acos();
            normal_sq += sample.weight * angle * angle;
            normal_max = normal_max.max(angle);
            normal_weight += sample.weight;
        }
    }
    let mut triangles: Vec<usize> = samples.iter().map(|s| s.triangle).collect();
    triangles.sort_unstable();
    triangles.dedup();
    (
        orientation,
        FitMetrics {
            rms_error: (sum_sq / total).sqrt(),
            max_error,
            rms_normal_error: if normal_weight > 0.0 {
                (normal_sq / normal_weight).sqrt()
            } else {
                0.0
            },
            max_normal_error: normal_max,
            support_triangles: triangles.len(),
            supported_area: total,
        },
    )
}

