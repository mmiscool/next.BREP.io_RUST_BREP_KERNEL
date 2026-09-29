use crate::fit::{
    evaluate_surface, evaluate_surface_from_vertices, fit_surface_from_vertices_with_path,
    fit_surface_with_path, selection_scale, vertex_selection_scale,
};
use crate::numerical::recognition as numerical;
use crate::{
    AnalyticSurface, AnalyzedMesh, ConstraintMask, FitPath, Mesh, MeshAnalysisOptions,
    MetadataTrust, RecognitionError, RecognitionOptions, RecognitionResult, SurfaceFitResult,
    SurfaceHint, SurfaceRegion, SurfaceType, UnresolvedRegionDiagnostic,
};
use std::cmp::Ordering;
use std::collections::{BTreeSet, VecDeque};
use web_time::Instant;

const TYPES: [SurfaceType; 5] = [
    SurfaceType::Plane,
    SurfaceType::Sphere,
    SurfaceType::Cylinder,
    SurfaceType::Cone,
    SurfaceType::Torus,
];

const MODEL_COMPLEXITY_PENALTY_UNIT: f64 = numerical::MODEL_COMPLEXITY_PENALTY_UNIT;
// The probe supplies spatial/normal diversity without increasing the number
// of triangles actually passed to a primitive fitter.
const HYPOTHESIS_DIVERSITY_PROBE_TRIANGLES: usize = 128;

#[derive(Clone, Copy, Debug)]
struct ModelRank {
    numerically_exact: bool,
    score: f64,
}

fn compare_model_rank(
    left: ModelRank,
    left_kind: SurfaceType,
    right: ModelRank,
    right_kind: SurfaceType,
) -> Ordering {
    right
        .numerically_exact
        .cmp(&left.numerically_exact)
        .then_with(|| left.score.total_cmp(&right.score))
        .then(left_kind.cmp(&right_kind))
}

/// Reconstruct one selected triangle region using an explicit metadata hint.
///
/// The selection and options are validated before fitting. Exact candidates
/// are reused unchanged only after their position, normal, and support gates
/// pass; weaker hints select the corresponding refinement path.
pub fn reconstruct_surface(
    mesh: &Mesh,
    triangle_indices: &[usize],
    hint: &SurfaceHint,
    options: &RecognitionOptions,
) -> Result<SurfaceFitResult, RecognitionError> {
    options.validate()?;
    let analyzed = mesh.analyze(&MeshAnalysisOptions {
        feature_angle: options.feature_angle,
        ..Default::default()
    })?;
    analyzed.validate_selection(triangle_indices)?;
    reconstruct_analyzed(&analyzed, triangle_indices, hint, options)
}

/// Reconstruct one surface from an explicit subset of mesh vertices.
///
/// Unlike [`reconstruct_surface`], this entry point fits exactly the requested
/// vertices; it does not expand them to their incident triangles. Per-vertex
/// normals are used when supplied, otherwise area-weighted incident triangle
/// normals are derived. `minimum_support` and `minimum_support_area` continue
/// to gate the usable incident-triangle support represented by the vertices.
pub fn reconstruct_surface_from_vertices(
    mesh: &Mesh,
    vertex_indices: &[usize],
    hint: &SurfaceHint,
    options: &RecognitionOptions,
) -> Result<SurfaceFitResult, RecognitionError> {
    options.validate()?;
    let analyzed = mesh.analyze(&MeshAnalysisOptions {
        feature_angle: options.feature_angle,
        ..Default::default()
    })?;
    analyzed.validate_vertex_selection(vertex_indices)?;
    reconstruct_selected(
        &analyzed,
        Selection::Vertices(vertex_indices),
        hint,
        options,
    )
}

#[derive(Clone, Copy)]
enum Selection<'a> {
    Triangles(&'a [usize]),
    Vertices(&'a [usize]),
}

impl Selection<'_> {
    fn scale(self, mesh: &AnalyzedMesh) -> f64 {
        match self {
            Self::Triangles(ids) => selection_scale(mesh, ids),
            Self::Vertices(ids) => vertex_selection_scale(mesh, ids),
        }
    }

    fn fit(
        self,
        mesh: &AnalyzedMesh,
        kind: SurfaceType,
        initial: Option<AnalyticSurface>,
        fixed: ConstraintMask,
        options: &RecognitionOptions,
        path: FitPath,
    ) -> Result<SurfaceFitResult, RecognitionError> {
        match self {
            Self::Triangles(ids) => {
                fit_surface_with_path(mesh, ids, kind, initial, fixed, options, path)
            }
            Self::Vertices(ids) => {
                fit_surface_from_vertices_with_path(mesh, ids, kind, initial, fixed, options, path)
            }
        }
    }

    fn evaluate(
        self,
        mesh: &AnalyzedMesh,
        surface: AnalyticSurface,
        options: &RecognitionOptions,
        path: FitPath,
    ) -> Result<SurfaceFitResult, RecognitionError> {
        match self {
            Self::Triangles(ids) => evaluate_surface(mesh, ids, surface, options, path),
            Self::Vertices(ids) => {
                evaluate_surface_from_vertices(mesh, ids, surface, options, path)
            }
        }
    }

    fn accepted(self, model: &SurfaceFitResult, options: &RecognitionOptions, scale: f64) -> bool {
        accepted_impl(
            model,
            options,
            scale,
            matches!(self, Self::Vertices(_))
                || matches!(options.sampling, crate::SamplingMode::Vertices),
        )
    }
}

/// Discover and reconstruct analytic regions in an entire mesh.
///
/// This convenience entry point returns only accepted regions. Use
/// [`recognize_surfaces_with_unresolved`] when the unassigned triangle
/// partition is also required.
pub fn recognize_surfaces(
    mesh: &Mesh,
    options: &RecognitionOptions,
) -> Result<Vec<SurfaceRegion>, RecognitionError> {
    Ok(recognize_surfaces_with_unresolved(mesh, options)?.regions)
}

/// Discover analytic regions and retain triangles that no model accepted.
///
/// Metadata sidecars are evaluated before generic connected-component
/// extraction. Returned regions and unresolved triangle IDs are ordered
/// deterministically for a fixed mesh, options, and seed.
pub fn recognize_surfaces_with_unresolved(
    mesh: &Mesh,
    options: &RecognitionOptions,
) -> Result<RecognitionResult, RecognitionError> {
    options.validate()?;
    let analyzed = mesh.analyze(&MeshAnalysisOptions {
        feature_angle: options.feature_angle,
        ..Default::default()
    })?;
    let mut assigned = vec![false; analyzed.triangles.len()];
    let mut regions = Vec::new();
    let mut metadata_failures = Vec::new();
    // Metadata regions are deliberately inspected before generic extraction.
    for metadata in &analyzed.source_metadata {
        let ids: Vec<_> = metadata
            .triangle_indices
            .iter()
            .copied()
            .filter(|&i| !assigned[i] && analyzed.triangles[i].area > 0.0)
            .collect();
        if ids.is_empty() {
            continue;
        }
        let mut metadata_options = options.clone();
        if let Some(source_tolerance) = metadata.source_tolerance {
            metadata_options.distance_tolerance =
                metadata_options.distance_tolerance.max(source_tolerance);
        }
        match reconstruct_analyzed(&analyzed, &ids, &metadata.hint, &metadata_options) {
            Ok(mut fit)
                if metadata
                    .orientation
                    .is_none_or(|orientation| orientation == fit.orientation) =>
            {
                if let Some(orientation) = metadata.orientation {
                    fit.orientation = orientation;
                }
                if metadata.source_tolerance.is_some() || metadata.orientation.is_some() {
                    fit.diagnostics.reason.push_str(
                        "; source orientation and tolerance metadata validated when supplied",
                    );
                }
                for &i in &ids {
                    assigned[i] = true;
                }
                regions.push(to_region(fit, ids));
            }
            Ok(fit) => metadata_failures.push((
                metadata.clone(),
                ids,
                format!(
                    "supplied metadata orientation {:?} disagrees with reconstructed orientation {}",
                    metadata.orientation, fit.orientation
                ),
            )),
            Err(error) => metadata_failures.push((metadata.clone(), ids, error.to_string())),
        }
    }
    let remaining: Vec<_> = analyzed
        .all_non_degenerate()
        .into_iter()
        .filter(|&i| !assigned[i])
        .collect();
    let components = if options.discover_regions {
        analyzed.connected_components(&remaining, options.respect_features)
    } else {
        vec![remaining]
    };
    for component in components {
        extract_component(&analyzed, &component, options, &mut regions);
    }
    if options.allow_disconnected_same_surface {
        merge_disconnected_regions(&analyzed, options, &mut regions);
    }
    regions.sort_by_key(|region| {
        region
            .triangle_indices
            .first()
            .copied()
            .unwrap_or(usize::MAX)
    });
    let mut covered = vec![false; analyzed.triangles.len()];
    for region in &regions {
        for &id in &region.triangle_indices {
            covered[id] = true;
        }
    }
    let unresolved_triangles = analyzed
        .all_non_degenerate()
        .into_iter()
        .filter(|&id| !covered[id])
        .collect::<Vec<_>>();
    let unresolved_set: BTreeSet<_> = unresolved_triangles.iter().copied().collect();
    let mut unresolved_diagnostics = metadata_failures
        .into_iter()
        .filter_map(|(metadata, ids, metadata_error)| {
            let mut triangle_indices: Vec<_> = ids
                .into_iter()
                .filter(|id| unresolved_set.contains(id))
                .collect();
            triangle_indices.sort_unstable();
            triangle_indices.dedup();
            (!triangle_indices.is_empty()).then(|| UnresolvedRegionDiagnostic {
                triangle_indices,
                source_face_id: metadata.source_face_id,
                source_face_name: metadata.source_face_name,
                source_surface_id: metadata.source_surface_id,
                reason: format!(
                    "source metadata validation or reconstruction failed ({metadata_error}); generic analytic fallback also left this subset unresolved"
                ),
            })
        })
        .collect::<Vec<_>>();
    unresolved_diagnostics.sort_by(|left, right| {
        left.triangle_indices
            .cmp(&right.triangle_indices)
            .then_with(|| left.source_face_id.cmp(&right.source_face_id))
            .then_with(|| left.source_face_name.cmp(&right.source_face_name))
            .then_with(|| left.source_surface_id.cmp(&right.source_surface_id))
            .then_with(|| left.reason.cmp(&right.reason))
    });
    Ok(RecognitionResult {
        regions,
        unresolved_triangles,
        unresolved_diagnostics,
    })
}

/// Reconstruct a selected triangle region from an already analyzed mesh.
///
/// Integration crates can use this seam to avoid repeating mesh analysis when
/// validating many independently owned source faces.
#[doc(hidden)]
pub fn reconstruct_analyzed(
    mesh: &AnalyzedMesh,
    ids: &[usize],
    hint: &SurfaceHint,
    options: &RecognitionOptions,
) -> Result<SurfaceFitResult, RecognitionError> {
    // This crate-private entry point is used by STEP validation to reuse mesh
    // analysis across recognition modes. It must preserve every public-entry
    // invariant rather than assuming its caller already validated options.
    options.validate()?;
    mesh.validate_selection(ids)?;
    reconstruct_selected(mesh, Selection::Triangles(ids), hint, options)
}

fn reconstruct_selected(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    hint: &SurfaceHint,
    options: &RecognitionOptions,
) -> Result<SurfaceFitResult, RecognitionError> {
    let scale = selection.scale(mesh);
    let metadata_started = options.collect_phase_timings.then(Instant::now);
    let metadata_trust = match hint {
        SurfaceHint::Unknown => MetadataTrust::Unknown,
        SurfaceHint::KnownType { .. } => MetadataTrust::TypeOnly,
        SurfaceHint::InitialGuess { trust, .. } | SurfaceHint::Constrained { trust, .. } => *trust,
        SurfaceHint::ExactCandidate { .. } => MetadataTrust::Exact,
    };
    let metadata_seconds = metadata_started.map(|started| started.elapsed().as_secs_f64());
    let result = match hint {
        SurfaceHint::Unknown => best_model(
            mesh,
            selection,
            scale,
            options,
            FitPath::GenericRecognition,
            None,
            ConstraintMask::default(),
        ),
        SurfaceHint::KnownType { surface_type } => fit_result(
            mesh,
            selection,
            *surface_type,
            None,
            ConstraintMask::default(),
            scale,
            options,
            FitPath::KnownTypeFit,
            true,
            None,
        ),
        SurfaceHint::InitialGuess { surface, trust } => {
            if matches!(
                trust,
                crate::MetadataTrust::Exact | crate::MetadataTrust::StrongHint
            ) {
                let mut valid = selection.evaluate(mesh, *surface, options, FitPath::HintReused)?;
                if selection.accepted(&valid, options, scale) {
                    valid.diagnostics.reason = "supplied parameters validated".into();
                    valid.diagnostics.metadata_trust = metadata_trust;
                    valid.diagnostics.phase_timings.metadata_inspection_seconds = metadata_seconds;
                    return Ok(valid);
                }
            }
            fit_result(
                mesh,
                selection,
                surface.surface_type(),
                Some(*surface),
                ConstraintMask::default(),
                scale,
                options,
                FitPath::UnconstrainedRefinement,
                true,
                Some(*surface),
            )
        }
        SurfaceHint::Constrained {
            surface_type,
            constraints,
            ..
        } => {
            if constraints.initial.is_none() && constraints.fixed != ConstraintMask::default() {
                return Err(RecognitionError::InvalidSelection(
                    "fixed constraints require initial parameter values".into(),
                ));
            }
            fit_result(
                mesh,
                selection,
                *surface_type,
                constraints.initial,
                constraints.fixed,
                scale,
                options,
                FitPath::ConstrainedRefinement,
                true,
                constraints.initial,
            )
        }
        SurfaceHint::ExactCandidate { surface } => {
            let mut valid =
                selection.evaluate(mesh, *surface, options, FitPath::ExactCandidateReused)?;
            if selection.accepted(&valid, options, scale) {
                valid.diagnostics.fixed_parameters = ConstraintMask {
                    origin_or_center: true,
                    axis_or_normal: true,
                    radius: true,
                    major_radius: true,
                    angle: true,
                };
                valid.diagnostics.reason =
                    "exact source parameters validated and were reused unchanged".into();
                valid.diagnostics.metadata_trust = metadata_trust;
                valid.diagnostics.phase_timings.metadata_inspection_seconds = metadata_seconds;
                return Ok(valid);
            }
            fit_result(
                mesh,
                selection,
                surface.surface_type(),
                Some(*surface),
                ConstraintMask::default(),
                scale,
                options,
                FitPath::HintRejectedFallback,
                true,
                Some(*surface),
            )
            .or_else(|_| {
                best_model(
                    mesh,
                    selection,
                    scale,
                    options,
                    FitPath::HintRejectedFallback,
                    Some(*surface),
                    ConstraintMask::default(),
                )
            })
        }
    }?;
    Ok(with_metadata_diagnostics(
        result,
        metadata_trust,
        metadata_seconds,
    ))
}

fn with_metadata_diagnostics(
    mut result: SurfaceFitResult,
    trust: MetadataTrust,
    elapsed_seconds: Option<f64>,
) -> SurfaceFitResult {
    result.diagnostics.metadata_trust = trust;
    result.diagnostics.phase_timings.metadata_inspection_seconds = elapsed_seconds;
    result
}

#[allow(clippy::too_many_arguments)]
fn fit_result(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    kind: SurfaceType,
    initial: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    scale: f64,
    options: &RecognitionOptions,
    path: FitPath,
    skipped: bool,
    supplied: Option<AnalyticSurface>,
) -> Result<SurfaceFitResult, RecognitionError> {
    let mut fitted = selection.fit(mesh, kind, initial, fixed, options, path)?;
    let evaluation_started = options.collect_phase_timings.then(Instant::now);
    if !selection.accepted(&fitted, options, scale) {
        return Err(RecognitionError::FitFailed {
            surface: Some(kind.name()),
            reason: format!(
                "residuals exceed tolerance (max {:.3e}, normal {:.3e} rad)",
                fitted.metrics.max_error, fitted.metrics.max_normal_error
            ),
        });
    }
    fitted.diagnostics.generic_classification_skipped = skipped;
    fitted.diagnostics.supplied_surface = supplied;
    fitted.diagnostics.reason = "requested model fitted and validated".into();
    fitted
        .diagnostics
        .phase_timings
        .candidate_evaluation_seconds =
        evaluation_started.map(|started| started.elapsed().as_secs_f64());
    Ok(fitted)
}

fn best_model(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    scale: f64,
    options: &RecognitionOptions,
    path: FitPath,
    supplied: Option<AnalyticSurface>,
    fixed: ConstraintMask,
) -> Result<SurfaceFitResult, RecognitionError> {
    best_model_impl(mesh, selection, scale, options, path, supplied, fixed, true)
}

#[allow(clippy::too_many_arguments)]
fn best_model_impl(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    scale: f64,
    options: &RecognitionOptions,
    path: FitPath,
    supplied: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    use_score_bounds: bool,
) -> Result<SurfaceFitResult, RecognitionError> {
    let evaluation_started = options.collect_phase_timings.then(Instant::now);
    let tolerance = options.distance_tolerance + options.relative_tolerance * scale.max(1.0);
    let mut candidates = Vec::new();
    let mut rejected = Vec::new();
    for kind in TYPES {
        match selection.fit(mesh, kind, None, fixed, options, path) {
            Ok(model) if selection.accepted(&model, options, scale) => {
                let rank = model_rank(
                    mesh,
                    selection,
                    kind,
                    &model,
                    tolerance,
                    options.normal_tolerance,
                );
                candidates.push((rank, kind, model));
            }
            Ok(model) => rejected.push((
                kind,
                format!(
                    "max distance {:.3e}, max normal {:.3e}",
                    model.metrics.max_error, model.metrics.max_normal_error
                ),
            )),
            Err(e) => rejected.push((kind, e.to_string())),
        }
        if use_score_bounds {
            if let Some(bound) = unseen_model_score_lower_bound(kind) {
                if let Some((best_index, rank, best_kind)) = candidates
                    .iter()
                    .enumerate()
                    .min_by(|(_, a), (_, b)| compare_model_rank(a.0, a.1, b.0, b.1))
                    .map(|(index, candidate)| (index, candidate.0, candidate.1))
                {
                    // A merely tolerance-valid simple model cannot rule out a
                    // more complex carrier that fits at conditioned roundoff.
                    // Once the current best is itself numerically exact, the
                    // residual terms are non-negative and the ordinary score
                    // lower bound is safe within the exact tier.
                    if rank.numerically_exact && rank.score < bound {
                        let (_, _, model) = candidates.remove(best_index);
                        record_tolerance_valid_losers(&mut rejected, candidates, best_kind, rank);
                        rejected.sort_by_key(|(candidate_kind, _)| *candidate_kind);
                        let reason = format!(
                            "{} selected after {} candidate(s) from a score below every unseen model's proven lower bound",
                            best_kind.name(),
                            kind as usize + 1,
                        );
                        return Ok(decorate_best_model(
                            model,
                            supplied,
                            fixed,
                            kind as usize + 1,
                            rejected,
                            &reason,
                            evaluation_started.as_ref(),
                        ));
                    }
                }
            }
        }
    }
    // A sphere is the R = 0 limit of the torus parameterization. Triangle
    // centroids from a faceted sphere have latitude-dependent chord error, so
    // an unconstrained torus can otherwise absorb that discretization with a
    // tiny, numerically meaningless major radius. If both carriers pass every
    // geometric gate and R is below the configured spatial resolution, the
    // torus's extra axis and radius are not identifiable; retain the simpler
    // sphere. This does not suppress a torus when the sphere fails validation.
    let valid_sphere = candidates
        .iter()
        .any(|(_, kind, _)| *kind == SurfaceType::Sphere);
    if valid_sphere {
        let mut retained = Vec::with_capacity(candidates.len());
        for (rank, kind, model) in candidates {
            let sphere_limit_major_radius = match &model.surface {
                AnalyticSurface::Torus(torus) if torus.major_radius <= tolerance => {
                    Some(torus.major_radius)
                }
                _ => None,
            };
            if kind == SurfaceType::Torus {
                if let Some(major_radius) = sphere_limit_major_radius {
                    rejected.push((
                        kind,
                        format!(
                            "tolerance-valid candidate excluded as a non-identifiable sphere-limit torus: score {:.6e}, numerically_exact={}, major radius {major_radius:.3e} <= spatial tolerance {tolerance:.3e}; rms distance {:.3e}, max distance {:.3e}, rms normal {:.3e}, max normal {:.3e}",
                            rank.score,
                            rank.numerically_exact,
                            model.metrics.rms_error,
                            model.metrics.max_error,
                            model.metrics.rms_normal_error,
                            model.metrics.max_normal_error,
                        ),
                    ));
                    continue;
                }
            }
            retained.push((rank, kind, model));
        }
        candidates = retained;
    }
    // A cylinder is the infinite-major-radius limit of a torus.  On a short
    // faceted cylindrical patch an unconstrained torus can move its spine far
    // away and absorb a few units of coordinate quantization while providing
    // no second-curvature evidence in the normal field.  When both candidates
    // pass every geometric gate, retain the simpler cylinder if the torus is
    // not in a stronger numerical-exactness tier, does not improve RMS normal
    // agreement, and its RMS-position gain is below the configured fraction
    // of spatial resolution.  A genuine exact torus still outranks an inexact
    // cylinder, and a measurably curved torus remains available to the
    // observability safeguard or ordinary residual ranking.
    if let Some((cylinder_rank, _, cylinder)) = candidates
        .iter()
        .find(|(_, kind, _)| *kind == SurfaceType::Cylinder)
        .cloned()
    {
        let mut retained = Vec::with_capacity(candidates.len());
        for (rank, kind, model) in candidates {
            if kind == SurfaceType::Torus
                && torus_is_non_identifiable_cylinder_limit(
                    rank,
                    &model.metrics,
                    cylinder_rank,
                    &cylinder.metrics,
                    tolerance,
                )
            {
                let rms_gain = (cylinder.metrics.rms_error - model.metrics.rms_error).max(0.0);
                rejected.push((
                    kind,
                    format!(
                        "tolerance-valid candidate excluded as a non-identifiable cylinder-limit torus: score {:.6e}, numerically_exact={}, RMS-position gain {rms_gain:.3e} <= {:.3e} (1% of spatial tolerance), torus RMS normal {:.3e} >= cylinder RMS normal {:.3e}; rms distance {:.3e}, max distance {:.3e}, max normal {:.3e}",
                        rank.score,
                        rank.numerically_exact,
                        tolerance * numerical::TORUS_CYLINDER_LIMIT_RMS_GAIN_FRACTION,
                        model.metrics.rms_normal_error,
                        cylinder.metrics.rms_normal_error,
                        model.metrics.rms_error,
                        model.metrics.max_error,
                        model.metrics.max_normal_error,
                    ),
                ));
                continue;
            }
            retained.push((rank, kind, model));
        }
        candidates = retained;
    }
    // Complexity is a tie-breaker, not authority to discard observed second
    // curvature. Apply this only after the non-identifiable cylinder-limit
    // filter above: a surviving inexact torus may displace the cylinder when
    // it Pareto-improves every reported residual and either resolves position
    // at the existing spatial threshold or its oriented carrier-normal field
    // is distinguishable on the exact same samples. Exact-tier ordering stays
    // authoritative and therefore never enters this override.
    if let (Some((torus_rank, _, torus)), Some((cylinder_rank, _, cylinder))) = (
        candidates
            .iter()
            .find(|(_, kind, _)| *kind == SurfaceType::Torus)
            .cloned(),
        candidates
            .iter()
            .find(|(_, kind, _)| *kind == SurfaceType::Cylinder)
            .cloned(),
    ) {
        let carrier_normal_disagreement = max_oriented_carrier_normal_disagreement(
            mesh,
            selection,
            options.sampling,
            &torus,
            &cylinder,
        );
        if torus_observably_dominates_cylinder(
            torus_rank,
            &torus.metrics,
            cylinder_rank,
            &cylinder.metrics,
            tolerance,
            carrier_normal_disagreement,
        ) {
            let rms_position_gain = cylinder.metrics.rms_error - torus.metrics.rms_error;
            candidates.retain(|(_, kind, _)| *kind != SurfaceType::Cylinder);
            rejected.push((
                SurfaceType::Cylinder,
                format!(
                    "tolerance-valid cylinder excluded because the torus observably resolves second curvature: RMS-position gain {rms_position_gain:.3e} (threshold {:.3e}), oriented carrier-normal disagreement {:.3e} (threshold {:.3e}); torus RMS/max distance {:.3e}/{:.3e} versus cylinder {:.3e}/{:.3e}; torus RMS/max normal {:.3e}/{:.3e} versus cylinder {:.3e}/{:.3e}",
                    tolerance * numerical::TORUS_CYLINDER_LIMIT_RMS_GAIN_FRACTION,
                    carrier_normal_disagreement.unwrap_or(0.0),
                    numerical::EXACT_MODEL_NORMAL_ROUNDOFF_RADIANS,
                    torus.metrics.rms_error,
                    torus.metrics.max_error,
                    cylinder.metrics.rms_error,
                    cylinder.metrics.max_error,
                    torus.metrics.rms_normal_error,
                    torus.metrics.max_normal_error,
                    cylinder.metrics.rms_normal_error,
                    cylinder.metrics.max_normal_error,
                ),
            ));
        }
    }
    candidates.sort_by(|a, b| compare_model_rank(a.0, a.1, b.0, b.1));
    if candidates.is_empty() {
        return Err(RecognitionError::FitFailed {
            surface: None,
            reason: "no analytic model passed distance, normal, and support gates".into(),
        });
    }
    let (selected_rank, selected_kind, model) = candidates.remove(0);
    record_tolerance_valid_losers(&mut rejected, candidates, selected_kind, selected_rank);
    rejected.sort_by_key(|(candidate_kind, _)| *candidate_kind);
    Ok(decorate_best_model(
        model,
        supplied,
        fixed,
        TYPES.len(),
        rejected,
        "best tolerance-valid model selected by numerical-exactness tier, residual score, and simplicity penalty",
        evaluation_started.as_ref(),
    ))
}

fn torus_observably_dominates_cylinder(
    torus_rank: ModelRank,
    torus: &crate::FitMetrics,
    cylinder_rank: ModelRank,
    cylinder: &crate::FitMetrics,
    spatial_tolerance: f64,
    carrier_normal_disagreement: Option<f64>,
) -> bool {
    !torus_rank.numerically_exact
        && !cylinder_rank.numerically_exact
        && torus.rms_error <= cylinder.rms_error
        && torus.max_error <= cylinder.max_error
        && torus.rms_normal_error <= cylinder.rms_normal_error
        && torus.max_normal_error <= cylinder.max_normal_error
        && ((cylinder.rms_error - torus.rms_error)
            > spatial_tolerance * numerical::TORUS_CYLINDER_LIMIT_RMS_GAIN_FRACTION
            || carrier_normal_disagreement.is_some_and(|disagreement| {
                disagreement > numerical::EXACT_MODEL_NORMAL_ROUNDOFF_RADIANS
                    && torus.rms_normal_error < cylinder.rms_normal_error
            }))
}

fn max_oriented_carrier_normal_disagreement(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    sampling: crate::SamplingMode,
    left: &SurfaceFitResult,
    right: &SurfaceFitResult,
) -> Option<f64> {
    let mut max_disagreement = 0.0_f64;
    let mut samples = 0_usize;
    let mut measure = |point| -> Option<()> {
        let left_normal = left.surface.normal_at(point)? * left.orientation as f64;
        let right_normal = right.surface.normal_at(point)? * right.orientation as f64;
        if !left_normal.is_finite() || !right_normal.is_finite() {
            return None;
        }
        let disagreement = left_normal.dot(right_normal).clamp(-1.0, 1.0).acos();
        if !disagreement.is_finite() {
            return None;
        }
        max_disagreement = max_disagreement.max(disagreement);
        samples += 1;
        Some(())
    };

    match selection {
        Selection::Triangles(ids) => {
            if matches!(
                sampling,
                crate::SamplingMode::TriangleCentroids | crate::SamplingMode::CentroidsAndVertices
            ) {
                for &id in ids {
                    if mesh.triangles[id].area > 0.0 {
                        measure(mesh.triangles[id].centroid)?;
                    }
                }
            }
            if matches!(
                sampling,
                crate::SamplingMode::Vertices | crate::SamplingMode::CentroidsAndVertices
            ) {
                let vertices: BTreeSet<_> = ids
                    .iter()
                    .filter(|&&id| mesh.triangles[id].area > 0.0)
                    .flat_map(|&id| mesh.triangles[id].vertices)
                    .collect();
                for vertex in vertices {
                    measure(mesh.vertices[vertex])?;
                }
            }
        }
        Selection::Vertices(vertices) => {
            for &vertex in vertices {
                measure(mesh.vertices[vertex])?;
            }
        }
    }
    (samples > 0).then_some(max_disagreement)
}

fn torus_is_non_identifiable_cylinder_limit(
    torus_rank: ModelRank,
    torus: &crate::FitMetrics,
    cylinder_rank: ModelRank,
    cylinder: &crate::FitMetrics,
    spatial_tolerance: f64,
) -> bool {
    (!torus_rank.numerically_exact || cylinder_rank.numerically_exact)
        && torus.rms_normal_error >= cylinder.rms_normal_error
        && (cylinder.rms_error - torus.rms_error).max(0.0)
            <= spatial_tolerance * numerical::TORUS_CYLINDER_LIMIT_RMS_GAIN_FRACTION
}

fn record_tolerance_valid_losers(
    rejected: &mut Vec<(SurfaceType, String)>,
    mut candidates: Vec<(ModelRank, SurfaceType, SurfaceFitResult)>,
    selected_kind: SurfaceType,
    selected_rank: ModelRank,
) {
    candidates.sort_by_key(|(_, kind, _)| *kind);
    rejected.extend(candidates.into_iter().map(|(rank, kind, model)| {
        (
            kind,
            format!(
                "tolerance-valid candidate not selected: score {:.6e}, numerically_exact={} versus selected {} score {:.6e}, numerically_exact={}; rms distance {:.3e}, max distance {:.3e}, rms normal {:.3e}, max normal {:.3e}",
                rank.score,
                rank.numerically_exact,
                selected_kind.name(),
                selected_rank.score,
                selected_rank.numerically_exact,
                model.metrics.rms_error,
                model.metrics.max_error,
                model.metrics.rms_normal_error,
                model.metrics.max_normal_error,
            ),
        )
    }));
}


fn unseen_model_score_lower_bound(after: SurfaceType) -> Option<f64> {
    let degrees = match after {
        SurfaceType::Plane => 4.0,
        SurfaceType::Sphere => 5.0,
        SurfaceType::Cylinder => 6.0,
        SurfaceType::Cone => 8.0,
        SurfaceType::Torus => return None,
    };
    Some(degrees * MODEL_COMPLEXITY_PENALTY_UNIT)
}

fn decorate_best_model(
    mut answer: SurfaceFitResult,
    supplied: Option<AnalyticSurface>,
    fixed: ConstraintMask,
    candidates_evaluated: usize,
    rejected_competitors: Vec<(SurfaceType, String)>,
    reason: &str,
    evaluation_started: Option<&Instant>,
) -> SurfaceFitResult {
    answer.diagnostics.supplied_surface = supplied;
    answer.diagnostics.fixed_parameters = fixed;
    answer.diagnostics.generic_classification_skipped = false;
    answer.diagnostics.reason = reason.into();
    answer.diagnostics.candidates_evaluated = candidates_evaluated;
    answer.diagnostics.rejected_competitors = rejected_competitors;
    answer
        .diagnostics
        .phase_timings
        .candidate_evaluation_seconds =
        evaluation_started.map(|started| started.elapsed().as_secs_f64());
    answer
}

fn model_rank(
    mesh: &AnalyzedMesh,
    selection: Selection<'_>,
    kind: SurfaceType,
    model: &SurfaceFitResult,
    distance_tolerance: f64,
    normal_tolerance: f64,
) -> ModelRank {
    let vertices: BTreeSet<_> = match selection {
        Selection::Triangles(ids) => ids
            .iter()
            .flat_map(|&id| mesh.triangles[id].vertices)
            .collect(),
        Selection::Vertices(ids) => ids.iter().copied().collect(),
    };
    let coordinate_scale = vertices
        .into_iter()
        .map(|vertex| {
            let point = mesh.vertices[vertex];
            point.x.abs().max(point.y.abs()).max(point.z.abs())
        })
        .fold(1.0_f64, f64::max);
    let position_roundoff =
        numerical::EXACT_VERTEX_ROUNDOFF_MULTIPLIER * f64::EPSILON * coordinate_scale;
    ModelRank {
        numerically_exact: model.metrics.max_error <= position_roundoff
            && model.metrics.max_normal_error <= numerical::EXACT_MODEL_NORMAL_ROUNDOFF_RADIANS,
        score: model_selection_score(kind, &model.metrics, distance_tolerance, normal_tolerance),
    }
}

fn model_selection_score(
    kind: SurfaceType,
    metrics: &crate::FitMetrics,
    distance_tolerance: f64,
    normal_tolerance: f64,
) -> f64 {
    let complexity = match kind {
        SurfaceType::Plane => 3.,
        SurfaceType::Sphere => 4.,
        SurfaceType::Cylinder => 5.,
        SurfaceType::Cone => 6.,
        SurfaceType::Torus => 8.,
    };
    // Complexity regularizes candidates within the same numerical-exactness
    // tier. `compare_model_rank` prevents that penalty from allowing a visibly
    // worse simple carrier to override a carrier fitted at conditioned
    // roundoff. This matters on tiny CAD patches where a sphere can pass a
    // loose acceptance tolerance while the source torus fits exactly.
    metrics.rms_error / distance_tolerance.max(numerical::SCORE_DISTANCE_DENOMINATOR_FLOOR)
        + metrics.rms_normal_error / normal_tolerance.max(numerical::SCORE_NORMAL_DENOMINATOR_FLOOR)
        + complexity * MODEL_COMPLEXITY_PENALTY_UNIT
}


fn accepted_impl(
    model: &SurfaceFitResult,
    options: &RecognitionOptions,
    scale: f64,
    vertex_samples_only: bool,
) -> bool {
    let distance = options.distance_tolerance + options.relative_tolerance * scale.max(1.0);
    // A trimmed CAD tessellation can contain pole/seam vertices whose supplied
    // derivative normals are singular or locally reversed even though the
    // vertices lie exactly on one carrier. In that machine-exact positional
    // case, retain the normal field as a global orientation/coherence check:
    // its area-weighted RMS must remain in the selected carrier normal's open
    // hemisphere. A few zero-area or low-area antipodal samples may therefore
    // survive, while a tangential, balanced, or generally incoherent field may
    // not. This positional threshold is intentionally far stricter than the
    // user tolerance, so ordinary noisy meshes cannot bypass the normal gate.
    let numerical_position =
        numerical::EXACT_VERTEX_ROUNDOFF_MULTIPLIER * f64::EPSILON * scale.max(1.0);
    let exact_vertices_with_coherent_sense = vertex_samples_only
        && model.metrics.max_error <= numerical_position
        && model.metrics.rms_normal_error < std::f64::consts::FRAC_PI_2;
    model.metrics.max_error <= distance
        // Pole/seam slivers can have poorly conditioned chord normals even
        // when their vertices lie on the exact CAD carrier. Preserve the true
        // maximum in diagnostics, but use area-weighted RMS for acceptance.
        && (model.metrics.rms_normal_error <= options.normal_tolerance
            || exact_vertices_with_coherent_sense)
        && model.metrics.support_triangles >= options.minimum_support
        && model.metrics.supported_area >= options.minimum_support_area
}
fn to_region(fit: SurfaceFitResult, ids: Vec<usize>) -> SurfaceRegion {
    SurfaceRegion {
        surface: fit.surface,
        orientation: fit.orientation,
        triangle_indices: ids,
        metrics: fit.metrics,
        confidence: fit.confidence,
        diagnostics: fit.diagnostics,
    }
}

fn merge_disconnected_regions(
    mesh: &AnalyzedMesh,
    options: &RecognitionOptions,
    regions: &mut Vec<SurfaceRegion>,
) {
    let mut left = 0;
    while left < regions.len() {
        let mut right = left + 1;
        while right < regions.len() {
            if regions[left].surface.surface_type() != regions[right].surface.surface_type()
                || regions[left].orientation != regions[right].orientation
            {
                right += 1;
                continue;
            }
            let mut ids = regions[left].triangle_indices.clone();
            ids.extend_from_slice(&regions[right].triangle_indices);
            ids.sort_unstable();
            let hint = SurfaceHint::InitialGuess {
                surface: regions[left].surface,
                trust: crate::MetadataTrust::InitialGuess,
            };
            let Ok(mut fit) = reconstruct_analyzed(mesh, &ids, &hint, options) else {
                right += 1;
                continue;
            };
            if fit.orientation != regions[left].orientation {
                right += 1;
                continue;
            }
            fit.diagnostics
                .reason
                .push_str("; disconnected supports were jointly refitted and merged by request");
            regions[left] = to_region(fit, ids);
            regions.remove(right);
            // The enlarged region may now validate another disconnected patch.
            right = left + 1;
        }
        left += 1;
    }
}

fn extract_component(
    mesh: &AnalyzedMesh,
    component: &[usize],
    options: &RecognitionOptions,
    out: &mut Vec<SurfaceRegion>,
) {
    if component.len() < options.minimum_support {
        return;
    }
    if let Ok(fit) = reconstruct_analyzed(mesh, component, &SurfaceHint::Unknown, options) {
        out.push(to_region(fit, component.to_vec()));
        return;
    }
    // CAD-aware hypothesis generation: fit small connected neighborhoods and
    // grow their geometric support through adjacency. This is deterministic
    // with a supplied seed and lets tangent analytic regions separate.
    let mut remaining: BTreeSet<usize> = component.iter().copied().collect();
    let mut rng = SplitMix(options.deterministic_seed.unwrap_or(0x52414e534143));
    while remaining.len() >= options.minimum_support {
        let ids: Vec<_> = remaining.iter().copied().collect();
        // One independently shuffled, without-replacement seed schedule per
        // primitive. At the full cap every remaining triangle is tried once
        // for every carrier; random-with-replacement sampling could waste
        // roughly a third of that budget on duplicate seeds.
        let seed_orders: Vec<_> = TYPES
            .iter()
            .map(|_| shuffled_seed_order(&ids, &mut rng))
            .collect();
        let mut best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
        let attempt_cap = options.max_hypotheses.min(ids.len() * TYPES.len());
        let mut attempts_required = attempt_cap;
        let mut attempt = 0;
        let mut region_growth_seconds = options.collect_phase_timings.then_some(0.0);
        while attempt < attempts_required {
            let attempt_index = attempt;
            attempt += 1;
            let kind_index = attempt_index % TYPES.len();
            let round = attempt_index / TYPES.len();
            let seed = seed_orders[kind_index][round];
            let kind = TYPES[kind_index];
            // Generate each primitive from the smallest diverse triangle set
            // that supplies its fitter's required independent observations in
            // every public sampling mode. Final support is still grown and
            // validated against the complete connected region below. Keeping
            // the fitted sample primitive-specific avoids fitting every
            // candidate to the full diversity probe while preserving
            // aggressive rank/degeneracy rejection in the fitter.
            let sample_size = hypothesis_sample_triangles(kind);
            let mut probe_limit = HYPOTHESIS_DIVERSITY_PROBE_TRIANGLES
                .max(sample_size)
                .min(remaining.len());
            let mut probe_best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
            loop {
                let probe = neighborhood(mesh, seed, &remaining, probe_limit);
                if probe.len() < sample_size {
                    break;
                }
                let sample = diverse_hypothesis_sample(mesh, seed, &probe, sample_size);
                let mut grew_beyond_probe = false;
                let mut reconstructed_probe = false;
                if let Ok(candidate) = fit_surface_with_path(
                    mesh,
                    &sample,
                    kind,
                    None,
                    ConstraintMask::default(),
                    options,
                    FitPath::GenericRecognition,
                ) {
                    let growth_started = options.collect_phase_timings.then(Instant::now);
                    let support = support_component(mesh, &ids, seed, candidate.surface, options);
                    if let (Some(total), Some(started)) =
                        (&mut region_growth_seconds, growth_started)
                    {
                        *total += started.elapsed().as_secs_f64();
                    }
                    grew_beyond_probe = support.len() > probe.len();
                    if support.len() >= options.minimum_support {
                        if let Ok(fit) = reconstruct_analyzed(
                            mesh,
                            &support,
                            &SurfaceHint::InitialGuess {
                                surface: candidate.surface,
                                trust: crate::MetadataTrust::InitialGuess,
                            },
                            options,
                        ) {
                            reconstructed_probe = true;
                            let area = fit.metrics.supported_area;
                            let support_scale = selection_scale(mesh, &support);
                            let support_tolerance = options.distance_tolerance
                                + options.relative_tolerance * support_scale.max(1.0);
                            let rank = model_rank(
                                mesh,
                                Selection::Triangles(&support),
                                fit.surface.surface_type(),
                                &fit,
                                support_tolerance,
                                options.normal_tolerance,
                            );
                            let replace =
                                probe_best.as_ref().is_none_or(|(count, old_rank, old, _)| {
                                    support.len() > *count
                                        || (support.len() == *count
                                            && (area > old.metrics.supported_area
                                                || (area == old.metrics.supported_area
                                                    && compare_model_rank(
                                                        rank,
                                                        fit.surface.surface_type(),
                                                        *old_rank,
                                                        old.surface.surface_type(),
                                                    ) == Ordering::Less)))
                                });
                            if replace {
                                probe_best = Some((support.len(), rank, fit, support));
                            }
                        }
                    }
                }
                let exhausted = probe.len() == remaining.len() || probe.len() < probe_limit;
                if (grew_beyond_probe && reconstructed_probe) || exhausted {
                    break;
                }
                // A candidate explaining no more triangles than its local
                // probe is under-observed (for example one planar strip of a
                // finely faceted cylinder). Expand only the diversity probe;
                // every fitter call remains primitive-minimal.
                probe_limit = probe_limit.saturating_mul(2).min(remaining.len());
            }
            let Some((_, rank, mut fit, support)) = probe_best else {
                continue;
            };
            fit.diagnostics.hypotheses_generated = attempt_index + 1;
            let area = fit.metrics.supported_area;
            let replace = best.as_ref().is_none_or(|(count, old_rank, old, _)| {
                support.len() > *count
                    || (support.len() == *count
                        && (area > old.metrics.supported_area
                            || (area == old.metrics.supported_area
                                && compare_model_rank(
                                    rank,
                                    fit.surface.surface_type(),
                                    *old_rank,
                                    old.surface.surface_type(),
                                ) == Ordering::Less)))
            });
            if replace {
                let support_fraction = support.len() as f64 / ids.len() as f64;
                attempts_required = attempts_required.min(required_seed_hypotheses(
                    options.confidence,
                    support_fraction,
                    TYPES.len(),
                    attempt_cap,
                ));
                attempts_required = attempts_required.max(attempt);
                best = Some((support.len(), rank, fit, support));
            }
        }
        let Some((_, _, mut fit, support)) = best else {
            break;
        };
        fit.diagnostics.phase_timings.region_growth_seconds = region_growth_seconds;
        for id in &support {
            remaining.remove(id);
        }
        out.push(to_region(fit, support));
    }
}

fn shuffled_seed_order(ids: &[usize], rng: &mut SplitMix) -> Vec<usize> {
    let mut order = ids.to_vec();
    for upper in (2..=order.len()).rev() {
        let upper_u64 = upper as u64;
        // Rejection avoids modulo bias, retaining the stated without-
        // replacement sampling probability for any practical mesh size.
        let zone = (u64::MAX / upper_u64) * upper_u64;
        let index = loop {
            let value = rng.next();
            if value < zone {
                break (value % upper_u64) as usize;
            }
        };
        order.swap(upper - 1, index);
    }
    order
}

/// Near-minimal triangle counts for generic primitive hypotheses.
///
/// `TriangleCentroids` yields exactly one observation per triangle, so these
/// counts match the primitive fitter's mathematical observation minima.
/// Vertex-containing sampling modes provide additional point/normal evidence
/// from the same compact connected patch.  Degenerate configurations are not
/// padded with arbitrary extra triangles: the fitter rejects them and RANSAC
/// tries another deterministic seed.
const fn hypothesis_sample_triangles(kind: SurfaceType) -> usize {
    match kind {
        SurfaceType::Plane => 3,
        SurfaceType::Sphere => 4,
        // Three centroid/normal observations are algebraically minimal, but
        // six are the near-minimal stable set on finely faceted CAD cylinders:
        // three can all lie in one numerically under-observed angular strip.
        SurfaceType::Cylinder => 6,
        SurfaceType::Cone => 4,
        SurfaceType::Torus => 6,
    }
}

/// Select a deterministic, normal-diverse near-minimal subset from a compact
/// connected probe.  Merely taking adjacent triangles would let one planar
/// facet strip of a tessellated cylinder masquerade as a supported plane.
/// Normal diversity is therefore the primary farthest-point criterion;
/// centroid separation breaks ties and spans genuinely planar patches.
fn diverse_hypothesis_sample(
    mesh: &AnalyzedMesh,
    seed: usize,
    probe: &[usize],
    count: usize,
) -> Vec<usize> {
    debug_assert!(probe.contains(&seed));
    debug_assert!(count > 0 && probe.len() >= count);
    let mut selected = Vec::with_capacity(count);
    selected.push(seed);
    while selected.len() < count {
        let mut best: Option<(f64, f64, usize)> = None;
        for &candidate in probe {
            if selected.contains(&candidate) {
                continue;
            }
            let triangle = &mesh.triangles[candidate];
            let (normal_gap, spatial_gap) = selected.iter().fold(
                (f64::INFINITY, f64::INFINITY),
                |(normal_gap, spatial_gap), &chosen| {
                    let other = &mesh.triangles[chosen];
                    (
                        normal_gap
                            .min(1.0 - triangle.normal.dot(other.normal).abs().clamp(0.0, 1.0)),
                        spatial_gap.min((triangle.centroid - other.centroid).length_squared()),
                    )
                },
            );
            let key = (normal_gap, spatial_gap, std::cmp::Reverse(candidate));
            if best
                .as_ref()
                .is_none_or(|&(best_normal, best_spatial, best_id)| {
                    (normal_gap, spatial_gap, std::cmp::Reverse(candidate))
                        > (best_normal, best_spatial, std::cmp::Reverse(best_id))
                })
            {
                best = Some((key.0, key.1, candidate));
            }
        }
        selected.push(best.expect("probe has enough distinct triangles").2);
    }
    selected.sort_unstable();
    selected
}

/// Conservative RANSAC stopping bound for round-robin primitive hypotheses.
/// Each primitive visits a separate without-replacement seed permutation.
/// For a region occupying fraction `w`, its exact hypergeometric miss
/// probability is no greater than the with-replacement bound `(1-w)^rounds`
/// used here. This controls the chance of visiting that support; candidate
/// rank and degeneracy checks still decide whether a visited seed is usable.
fn required_seed_hypotheses(
    confidence: f64,
    support_fraction: f64,
    kinds: usize,
    cap: usize,
) -> usize {
    let minimum = kinds.min(cap).max(1);
    if confidence <= 0.0 || support_fraction >= 1.0 {
        return minimum;
    }
    if support_fraction <= 0.0 || cap <= minimum {
        return cap.max(1);
    }
    let rounds = ((1.0 - confidence).ln() / (1.0 - support_fraction).ln())
        .ceil()
        .max(1.0) as usize;
    rounds.saturating_mul(kinds).clamp(minimum, cap)
}

fn neighborhood(
    mesh: &AnalyzedMesh,
    seed: usize,
    allowed: &BTreeSet<usize>,
    limit: usize,
) -> Vec<usize> {
    let mut seen = BTreeSet::new();
    let mut queue = VecDeque::from([seed]);
    seen.insert(seed);
    while let Some(id) = queue.pop_front() {
        if seen.len() >= limit {
            break;
        }
        for n in mesh.triangles[id].neighbors.iter().flatten() {
            if allowed.contains(n) && seen.insert(*n) {
                queue.push_back(*n);
            }
        }
    }
    seen.into_iter().collect()
}
fn triangle_support(
    mesh: &AnalyzedMesh,
    id: usize,
    surface: AnalyticSurface,
    options: &RecognitionOptions,
) -> bool {
    let t = &mesh.triangles[id];
    let tolerance =
        options.distance_tolerance + options.relative_tolerance * mesh.diagonal.max(1.0);
    if t.vertices
        .iter()
        .any(|&v| surface.signed_distance(mesh.vertices[v]).abs() > tolerance)
    {
        return false;
    }
    let Some(normal) = surface.normal_at(t.centroid) else {
        return false;
    };
    t.normal.dot(normal).abs().clamp(-1.0, 1.0).acos() <= options.normal_tolerance
}
fn support_component(
    mesh: &AnalyzedMesh,
    candidates: &[usize],
    seed: usize,
    surface: AnalyticSurface,
    options: &RecognitionOptions,
) -> Vec<usize> {
    let allowed: BTreeSet<_> = candidates
        .iter()
        .copied()
        .filter(|&i| triangle_support(mesh, i, surface, options))
        .collect();
    if !allowed.contains(&seed) {
        return Vec::new();
    }
    let mut seen = BTreeSet::from([seed]);
    let mut queue = VecDeque::from([seed]);
    while let Some(id) = queue.pop_front() {
        for n in mesh.triangles[id].neighbors.iter().flatten() {
            if allowed.contains(n) && seen.insert(*n) {
                queue.push_back(*n);
            }
        }
    }
    seen.into_iter().collect()
}
struct SplitMix(u64);
impl SplitMix {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e3779b97f4a7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
        z ^ (z >> 31)
    }
}

