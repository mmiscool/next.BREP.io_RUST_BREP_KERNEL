//! STEP face-level validation against exact analytic carriers imported by
//! `brep_kernel`.
//!
//! This module is feature-gated because it is a development/validation bridge,
//! not part of the standalone recognizer. It never mutates kernel topology.

use crate::numerical::{scalar, step_validation as numerical};
use crate::{
    brep::{analytic_truth_from_face, mesh_from_kernel},
    reconstruct_analyzed, AnalyticSurface, AnalyzedMesh, ConstraintMask, MetadataTrust,
    RecognitionOptions, SamplingMode, SurfaceConstraints, SurfaceFitResult, SurfaceHint, Vec3,
};
use serde::{Deserialize, Serialize};
use std::{fs, path::Path};
use web_time::Instant;

mod torus_fallback;
use torus_fallback::ObservationFallback;

/// Machine-readable progress from the synchronous STEP validation pipeline.
/// A completed mode report is included so watchdog-driven corpus runs retain
/// useful evidence when a later face or mode times out.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepValidationProgress {
    /// Pipeline phase represented by this event.
    pub phase: StepValidationPhase,
    /// STEP input path.
    pub path: String,
    /// Zero-based imported-solid index, when known.
    pub solid_index: Option<usize>,
    /// Zero-based shell index, when known.
    pub shell_index: Option<usize>,
    /// Zero-based face index, when known.
    pub face_index: Option<usize>,
    /// Stable source topology face ID, when known.
    pub face_id: Option<u64>,
    /// Recognition mode involved in the event.
    pub mode: Option<ValidationMode>,
    /// Exact imported carrier truth, when available.
    pub truth: Option<AnalyticSurface>,
    /// Observation triangle count, when tessellation completed.
    pub triangles: Option<usize>,
    /// Observation vertex count, when tessellation completed.
    pub vertices: Option<usize>,
    /// Mode validation decision, when completed.
    pub passed: Option<bool>,
    /// Additional human-readable evidence.
    pub detail: Option<String>,
    /// Complete mode evidence attached to a completion event.
    pub mode_report: Option<ModeValidationReport>,
}

/// Synchronous STEP validation lifecycle phase.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StepValidationPhase {
    /// Reading STEP bytes has started.
    ReadStarted,
    /// Reading STEP bytes completed.
    ReadCompleted,
    /// Kernel import has started.
    ImportStarted,
    /// Kernel import completed.
    ImportCompleted,
    /// Processing one supported face has started.
    FaceStarted,
    /// Face tessellation has started.
    TessellationStarted,
    /// Face tessellation completed.
    TessellationCompleted,
    /// Face tessellation failed.
    TessellationFailed,
    /// One recognition mode has started.
    ModeStarted,
    /// One recognition mode completed.
    ModeCompleted,
    /// All selected modes for a face completed.
    FaceCompleted,
    /// The complete file validation finished.
    FileCompleted,
    /// A pipeline operation failed.
    Failed,
}

/// Configuration for importing, tessellating, and validating STEP faces.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepValidationOptions {
    /// Kernel tessellation subdivision count in the first parameter direction.
    pub tessellation_slabs_u: usize,
    /// Kernel tessellation subdivision count in the second parameter direction.
    pub tessellation_steps_v: usize,
    /// Absolute floor for recognition distance tolerance.
    pub distance_tolerance: f64,
    /// Additional tolerance proportional to the face mesh bounding-box diagonal.
    pub relative_face_tolerance: f64,
    /// Maximum accepted normal error, in degrees.
    pub normal_tolerance_degrees: f64,
    /// Maximum generic hypothesis attempts per face.
    pub max_hypotheses: usize,
    /// Maximum nonlinear refinement iterations per fit.
    pub max_refinement_iterations: usize,
    /// Recognition modes exercised for each analytic face. Missing values in
    /// older serialized options default to the complete five-mode matrix.
    #[serde(default = "all_validation_modes")]
    pub enabled_modes: Vec<ValidationMode>,
}

impl Default for StepValidationOptions {
    fn default() -> Self {
        Self {
            tessellation_slabs_u: 12,
            tessellation_steps_v: 12,
            distance_tolerance: 1.0e-7,
            relative_face_tolerance: 1.0e-6,
            normal_tolerance_degrees: 12.0,
            max_hypotheses: 512,
            max_refinement_iterations: 160,
            enabled_modes: all_validation_modes(),
        }
    }
}

/// Complete validation and coverage report for one STEP input file.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepFileReport {
    /// Input STEP path.
    pub path: String,
    /// Fatal import error, if no usable report could be produced.
    pub import_error: Option<String>,
    /// Number of individual solids that failed within a partial import.
    pub failed_imported_solids: usize,
    /// Nonfatal partial-import diagnostic.
    pub partial_import_error: Option<String>,
    /// Number of successfully imported solids.
    pub solids: usize,
    /// Total number of imported faces.
    pub faces_total: usize,
    /// Faces whose imported carrier maps to one of the five supported
    /// primitives, before applying an optional focused face-id filter.
    pub analytic_faces: usize,
    /// Supported analytic faces selected for tessellation and validation.
    #[serde(default)]
    pub selected_analytic_faces: usize,
    /// Selected analytic faces that reached recognition and produced a case.
    #[serde(default)]
    pub validated_faces: usize,
    /// Compatibility counter for faces without supported truth. This includes
    /// both expected freeform/unsupported faces and truth-extraction errors;
    /// [`StepFaceDiagnosticKind`] distinguishes those categories.
    pub unsupported_analytic_faces: usize,
    /// Faces that appeared supported but could not yield valid truth carriers.
    #[serde(default)]
    pub truth_extraction_failures: usize,
    /// Supported faces that failed to produce an observation mesh.
    pub tessellation_failures: usize,
    /// Supported faces for which the legacy per-face tessellator returned no
    /// triangles but a trim-aware, source-projected fallback produced a valid
    /// observation mesh.
    #[serde(default)]
    pub tessellation_fallbacks: usize,
    /// Durable evidence for every successful fallback observation mesh.
    #[serde(default)]
    pub tessellation_fallback_evidence: Vec<TessellationFallbackEvidence>,
    /// Durable per-face evidence for every face that could not reach mode
    /// validation. Progress JSONL carries the same operational evidence, but
    /// the final report must remain independently auditable.
    #[serde(default)]
    pub face_diagnostics: Vec<StepFaceDiagnostic>,
    /// Successfully validated per-face cases.
    pub cases: Vec<FaceValidationReport>,
}

impl StepFileReport {
    /// Check the coverage accounting invariants for a completed report.
    pub fn coverage_invariants_hold(&self) -> bool {
        let unsupported_diagnostics = self
            .face_diagnostics
            .iter()
            .filter(|diagnostic| {
                matches!(
                    diagnostic.kind,
                    StepFaceDiagnosticKind::UnsupportedOrNonAnalytic
                        | StepFaceDiagnosticKind::TruthExtractionFailed
                )
            })
            .count();
        let extraction_failure_diagnostics = self
            .face_diagnostics
            .iter()
            .filter(|diagnostic| {
                matches!(
                    diagnostic.kind,
                    StepFaceDiagnosticKind::TruthExtractionFailed
                )
            })
            .count();
        self.faces_total == self.analytic_faces + self.unsupported_analytic_faces
            && self.validated_faces == self.cases.len()
            && self.selected_analytic_faces == self.validated_faces + self.tessellation_failures
            && unsupported_diagnostics == self.unsupported_analytic_faces
            && extraction_failure_diagnostics == self.truth_extraction_failures
            && self
                .face_diagnostics
                .iter()
                .filter(|diagnostic| {
                    matches!(diagnostic.kind, StepFaceDiagnosticKind::TessellationFailed)
                })
                .count()
                == self.tessellation_failures
            && self.tessellation_fallbacks == self.tessellation_fallback_evidence.len()
            && self
                .tessellation_fallback_evidence
                .iter()
                .enumerate()
                .all(|(index, evidence)| {
                    let method_matches_truth = match evidence.method {
                        TessellationFallbackMethod::WatertightFaceStride => {
                            matches!(evidence.truth, AnalyticSurface::Torus(_))
                        }
                        TessellationFallbackMethod::ProjectedTrimCurveTriangle => {
                            matches!(evidence.truth, AnalyticSurface::Plane(_))
                                && evidence.triangles == 1
                                && evidence.vertices == 3
                        }
                    };
                    method_matches_truth
                        && evidence.chord_tolerance.is_finite()
                        && evidence.chord_tolerance > 0.0
                        && evidence.triangles > 0
                        && evidence.vertices > 0
                        && evidence.surface_projected_vertices == evidence.vertices
                        && evidence.max_surface_projection_distance.is_finite()
                        && evidence.max_surface_projection_distance >= 0.0
                        && evidence.surface_projection_tolerance.is_finite()
                        && evidence.surface_projection_tolerance > 0.0
                        && evidence.max_surface_projection_distance
                            <= evidence.surface_projection_tolerance
                        && self.tessellation_fallback_evidence[index + 1..]
                            .iter()
                            .all(|other| {
                                (
                                    other.solid_index,
                                    other.shell_index,
                                    other.face_index,
                                    other.face_id,
                                ) != (
                                    evidence.solid_index,
                                    evidence.shell_index,
                                    evidence.face_index,
                                    evidence.face_id,
                                )
                            })
                        && self.cases.iter().any(|case| {
                            case.solid_index == evidence.solid_index
                                && case.shell_index == evidence.shell_index
                                && case.face_index == evidence.face_index
                                && case.face_id == evidence.face_id
                        })
                })
    }
}

/// Alternate tessellation mechanism used after primary face tessellation fails.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum TessellationFallbackMethod {
    /// Trim-aware watertight solid tessellation sliced by sequential face ID.
    WatertightFaceStride,
    /// One authored triangular plane trim reconstructed from its retained
    /// straight edge curves after source-tolerance vertex welding collapsed
    /// the ordinary topology and pcurves.
    ProjectedTrimCurveTriangle,
}

/// Auditable evidence for one successful fallback observation mesh.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TessellationFallbackEvidence {
    /// Zero-based imported-solid index.
    pub solid_index: usize,
    /// Zero-based shell index.
    pub shell_index: usize,
    /// Zero-based face index.
    pub face_index: usize,
    /// Stable source topology face ID.
    pub face_id: u64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    /// Optional stable source face name.
    pub face_name: Option<String>,
    /// Exact imported carrier truth.
    pub truth: AnalyticSurface,
    /// Diagnostic from the failed primary tessellator.
    pub primary_failure: String,
    /// Fallback method that produced the mesh.
    pub method: TessellationFallbackMethod,
    /// Chord tolerance used by the fallback context. For a retained-curve
    /// triangle it is the upper cap on source-surface projection correction.
    pub chord_tolerance: f64,
    /// Number of fallback triangles.
    pub triangles: usize,
    /// Number of fallback vertices.
    pub vertices: usize,
    /// Number of topology- or trim-curve-derived vertices projected back onto
    /// the same source face's NURBS carrier before recognition.
    pub surface_projected_vertices: usize,
    /// Largest correction from a shared-edge tessellation vertex to that source
    /// face. This quantifies, rather than hides, kernel edge/carrier mismatch.
    pub max_surface_projection_distance: f64,
    /// Maximum correction permitted for this raw fallback mesh, derived from
    /// the configured observation tolerance and capped by chord tolerance.
    pub surface_projection_tolerance: f64,
}

/// Reason a STEP face did not reach recognition-mode validation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StepFaceDiagnosticKind {
    /// The imported face is freeform or otherwise outside the five supported
    /// analytic carrier types. No RANSAC mode is expected for this face.
    UnsupportedOrNonAnalytic,
    /// The face appeared analytic, but the adapter could not extract a valid
    /// supported truth carrier. This is validation infrastructure evidence.
    TruthExtractionFailed,
    /// Supported truth existed, but no valid observation mesh was produced.
    TessellationFailed,
}

/// Durable location and failure evidence for one unvalidated STEP face.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StepFaceDiagnostic {
    /// Zero-based imported-solid index.
    pub solid_index: usize,
    /// Zero-based shell index.
    pub shell_index: usize,
    /// Zero-based face index.
    pub face_index: usize,
    /// Stable source topology face ID.
    pub face_id: u64,
    /// Optional stable source face name.
    pub face_name: Option<String>,
    /// Diagnostic category.
    pub kind: StepFaceDiagnosticKind,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    /// Exact carrier truth when extraction succeeded.
    pub truth: Option<AnalyticSurface>,
    /// Human-readable failure evidence.
    pub detail: String,
}

/// Exact truth, observation size, and per-mode results for one STEP face.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FaceValidationReport {
    /// Zero-based imported-solid index.
    pub solid_index: usize,
    /// Zero-based shell index.
    pub shell_index: usize,
    /// Zero-based face index.
    pub face_index: usize,
    /// Stable source topology face ID.
    pub face_id: u64,
    /// Optional stable source face name.
    pub face_name: Option<String>,
    /// Imported STEP `same_sense` face orientation.
    pub same_sense: bool,
    /// Exact imported analytic carrier.
    pub truth: AnalyticSurface,
    /// Observation triangle count.
    pub triangles: usize,
    /// Observation vertex count.
    pub vertices: usize,
    /// Bounding-box scale of the face observation mesh.
    pub face_scale: f64,
    /// Effective absolute distance tolerance used by validation.
    pub distance_tolerance: f64,
    /// Face tessellation time in milliseconds.
    pub tessellation_millis: f64,
    /// Results for every enabled trust path.
    pub modes: Vec<ModeValidationReport>,
}

/// Recognition result and independent oracle decisions for one trust mode.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ModeValidationReport {
    /// Trust path exercised.
    pub mode: ValidationMode,
    /// Recognition and validation time in milliseconds.
    pub elapsed_millis: f64,
    /// Recognition error text when the mode returned no result.
    pub error: Option<String>,
    /// Returned fit, when recognition succeeded.
    pub result: Option<SurfaceFitResult>,
    /// True only when all validation gates for this mode pass.
    #[serde(default)]
    pub passed: bool,
    /// The sampled face does not contain enough geometric information to
    /// distinguish the STEP carrier from another machine-precision analytic
    /// carrier. This is an oracle outcome, not a relaxed recognizer success.
    #[serde(default)]
    pub unobservable: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    /// Evidence supporting an `unobservable` oracle outcome.
    pub ambiguity_evidence: Option<CarrierAmbiguityEvidence>,
    /// Whether the returned primitive type matches imported truth.
    pub type_matches: bool,
    /// Whether the recovered carrier parameters are equivalent to STEP truth
    /// at the coordinate-conditioned numerical precision used by the
    /// observability oracle. Recognition-tolerance agreement is not enough.
    /// Legacy JSON reports omit this evidence and therefore deserialize to the
    /// conservative `false`, regardless of their historical `passed` value.
    #[serde(default)]
    pub parameter_equivalent: bool,
    /// Whether the fitted carrier orientation agrees with the STEP face's
    /// `same_sense` flag after carrier-gauge normalization.
    #[serde(default)]
    pub orientation_matches: bool,
    /// Mirrors the recognizer diagnostic. Exact-candidate validation requires
    /// this to be true; returning the same type after refitting is not enough.
    #[serde(default)]
    pub exact_parameters_reused: bool,
    /// Present for constrained validation. This verifies both that the
    /// requested mask reached the fitter and that every fixed value remained
    /// bit-for-bit equal to the supplied STEP truth.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fixed_parameters_preserved: Option<bool>,
    /// Gauge-aware parameter errors relative to imported truth.
    pub parameter_error: Option<ParameterError>,
}

/// Quantitative proof that sampled observations cannot distinguish two carriers.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CarrierAmbiguityEvidence {
    /// Successfully returned alternative carrier.
    pub alternative_surface: AnalyticSurface,
    /// Largest sampled position error to imported truth.
    pub max_truth_error: f64,
    /// Largest sampled position error to the alternative carrier.
    pub max_alternative_error: f64,
    /// Largest sampled oriented-normal disagreement between carriers.
    pub max_oriented_normal_disagreement: f64,
    /// Roundoff-scale positional indistinguishability threshold.
    pub position_threshold: f64,
    /// Angular indistinguishability threshold in radians.
    pub normal_threshold: f64,
}

/// Metadata trust path exercised against each imported analytic face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ValidationMode {
    /// Classify and fit without source carrier information.
    Unknown,
    /// Supply only the imported primitive type.
    KnownType,
    /// Refine a deliberately perturbed imported carrier.
    InitialGuess,
    /// Refine while preserving selected imported parameter groups.
    Constrained,
    /// Validate and reuse the exact imported carrier.
    ExactCandidate,
}

impl ValidationMode {
    /// Complete stable mode order used by default validation.
    pub const ALL: [Self; 5] = [
        Self::Unknown,
        Self::KnownType,
        Self::InitialGuess,
        Self::Constrained,
        Self::ExactCandidate,
    ];

    /// Return the stable command-line spelling for this mode.
    pub fn cli_name(self) -> &'static str {
        match self {
            Self::Unknown => "unknown",
            Self::KnownType => "known-type",
            Self::InitialGuess => "initial-guess",
            Self::Constrained => "constrained",
            Self::ExactCandidate => "exact-candidate",
        }
    }

    /// Parse a command-line mode spelling, ignoring surrounding whitespace.
    pub fn from_cli_name(value: &str) -> Option<Self> {
        Self::ALL
            .into_iter()
            .find(|mode| mode.cli_name() == value.trim().to_ascii_lowercase())
    }
}

fn all_validation_modes() -> Vec<ValidationMode> {
    ValidationMode::ALL.to_vec()
}

/// Gauge-aware recovered-parameter error relative to STEP truth.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct ParameterError {
    /// Point, center, apex, plane-offset, or axis-line error in model units.
    pub position: f64,
    /// Axis/normal error in degrees. Plane, cylinder, and torus directions are
    /// unoriented; cone axes are oriented because reversing one changes the
    /// represented nappe.
    pub direction_degrees: f64,
    /// Radius or minor-radius error in model units.
    pub radius: f64,
    /// Torus major-radius error in model units.
    pub major_radius: f64,
    /// Cone half-angle error in degrees.
    pub angle_degrees: f64,
}

/// Import and validate every supported analytic face in one STEP file.
pub fn validate_step_file(path: &Path, validation: &StepValidationOptions) -> StepFileReport {
    validate_step_file_faces(path, validation, &[])
}

/// Import one STEP file and validate only the requested stable face ids.
///
/// An empty filter has the same behavior as [`validate_step_file`]. Import and
/// analytic inventory counters still describe the complete file so a focused
/// regression cannot hide fixture drift.
pub fn validate_step_file_faces(
    path: &Path,
    validation: &StepValidationOptions,
    face_ids: &[u64],
) -> StepFileReport {
    validate_step_file_faces_with_progress(path, validation, face_ids, &mut |_| {})
}

#[allow(clippy::too_many_arguments)]
fn emit_progress(
    sink: &mut dyn FnMut(&StepValidationProgress),
    path: &str,
    phase: StepValidationPhase,
    location: Option<(usize, usize, usize, u64)>,
    mode: Option<ValidationMode>,
    truth: Option<AnalyticSurface>,
    mesh_size: Option<(usize, usize)>,
    passed: Option<bool>,
    detail: Option<String>,
    mode_report: Option<ModeValidationReport>,
) {
    let (solid_index, shell_index, face_index, face_id) = location
        .map(|value| (Some(value.0), Some(value.1), Some(value.2), Some(value.3)))
        .unwrap_or((None, None, None, None));
    sink(&StepValidationProgress {
        phase,
        path: path.to_owned(),
        solid_index,
        shell_index,
        face_index,
        face_id,
        mode,
        truth,
        triangles: mesh_size.map(|value| value.0),
        vertices: mesh_size.map(|value| value.1),
        passed,
        detail,
        mode_report,
    });
}

fn fallback_failure_detail(
    evidence: Option<&TessellationFallbackEvidence>,
    failure: &str,
) -> String {
    evidence.map_or_else(
        || failure.to_owned(),
        |evidence| {
            format!(
                "{}; {:?} fallback {failure}",
                evidence.primary_failure, evidence.method
            )
        },
    )
}

#[allow(clippy::too_many_arguments)]
fn record_tessellation_failure(
    report: &mut StepFileReport,
    progress: &mut dyn FnMut(&StepValidationProgress),
    path: &str,
    location: Option<(usize, usize, usize, u64)>,
    solid_index: usize,
    shell_index: usize,
    face_index: usize,
    face: &brep_kernel::FaceRecord,
    truth: AnalyticSurface,
    mesh_size: Option<(usize, usize)>,
    detail: String,
) {
    report.tessellation_failures += 1;
    report.face_diagnostics.push(StepFaceDiagnostic {
        solid_index,
        shell_index,
        face_index,
        face_id: face.id,
        face_name: face.name.clone(),
        kind: StepFaceDiagnosticKind::TessellationFailed,
        truth: Some(truth),
        detail: detail.clone(),
    });
    emit_progress(
        progress,
        path,
        StepValidationPhase::TessellationFailed,
        location,
        None,
        Some(truth),
        mesh_size,
        Some(false),
        Some(detail),
        None,
    );
}

/// Validate selected faces while synchronously reporting pipeline progress.
/// The callback runs on the validation thread and should return quickly.
pub fn validate_step_file_faces_with_progress(
    path: &Path,
    validation: &StepValidationOptions,
    face_ids: &[u64],
    progress: &mut dyn FnMut(&StepValidationProgress),
) -> StepFileReport {
    let path_string = path.display().to_string();
    let mut report = StepFileReport {
        path: path_string.clone(),
        import_error: None,
        failed_imported_solids: 0,
        partial_import_error: None,
        solids: 0,
        faces_total: 0,
        analytic_faces: 0,
        selected_analytic_faces: 0,
        validated_faces: 0,
        unsupported_analytic_faces: 0,
        truth_extraction_failures: 0,
        tessellation_failures: 0,
        tessellation_fallbacks: 0,
        tessellation_fallback_evidence: Vec::new(),
        face_diagnostics: Vec::new(),
        cases: Vec::new(),
    };
    emit_progress(
        progress,
        &path_string,
        StepValidationPhase::ReadStarted,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );
    let text = match fs::read_to_string(path) {
        Ok(text) => text,
        Err(error) => {
            report.import_error = Some(format!("read failed: {error}"));
            emit_progress(
                progress,
                &path_string,
                StepValidationPhase::Failed,
                None,
                None,
                None,
                None,
                Some(false),
                report.import_error.clone(),
                None,
            );
            return report;
        }
    };
    emit_progress(
        progress,
        &path_string,
        StepValidationPhase::ReadCompleted,
        None,
        None,
        None,
        None,
        Some(true),
        Some(format!("{} bytes", text.len())),
        None,
    );
    emit_progress(
        progress,
        &path_string,
        StepValidationPhase::ImportStarted,
        None,
        None,
        None,
        None,
        None,
        None,
        None,
    );
    let brep_kernel::StepImportReport { solids, failed, first_error: partial, .. } = match brep_kernel::import_step_report(&text) {
        Ok(value) => value,
        Err(error) => {
            report.import_error = Some(error);
            emit_progress(
                progress,
                &path_string,
                StepValidationPhase::Failed,
                None,
                None,
                None,
                None,
                Some(false),
                report.import_error.clone(),
                None,
            );
            return report;
        }
    };
    report.failed_imported_solids = failed;
    report.partial_import_error = partial;
    report.solids = solids.len();
    emit_progress(
        progress,
        &path_string,
        StepValidationPhase::ImportCompleted,
        None,
        None,
        None,
        None,
        Some(true),
        Some(format!("solids={}, failed_solids={failed}", solids.len())),
        None,
    );
    for (solid_index, solid) in solids.iter().enumerate() {
        let mut global_face_index = 0usize;
        let mut observation_fallback = ObservationFallback::new(
            solid,
            validation.distance_tolerance,
            validation.relative_face_tolerance,
        );
        for (shell_index, shell) in solid.shells.iter().enumerate() {
            for (face_index, face) in shell.faces.iter().enumerate() {
                let source_face_index = global_face_index;
                global_face_index += 1;
                report.faces_total += 1;
                let kernel_truth = match analytic_truth_from_face(face, face_index as u32) {
                    Ok(Some(truth)) => truth,
                    Ok(None) => {
                        report.unsupported_analytic_faces += 1;
                        report.face_diagnostics.push(StepFaceDiagnostic {
                            solid_index,
                            shell_index,
                            face_index,
                            face_id: face.id,
                            face_name: face.name.clone(),
                            kind: StepFaceDiagnosticKind::UnsupportedOrNonAnalytic,
                            truth: None,
                            detail: "face has no supported analytic carrier".into(),
                        });
                        continue;
                    }
                    Err(error) => {
                        report.unsupported_analytic_faces += 1;
                        report.truth_extraction_failures += 1;
                        report.face_diagnostics.push(StepFaceDiagnostic {
                            solid_index,
                            shell_index,
                            face_index,
                            face_id: face.id,
                            face_name: face.name.clone(),
                            kind: StepFaceDiagnosticKind::TruthExtractionFailed,
                            truth: None,
                            detail: error.to_string(),
                        });
                        continue;
                    }
                };
                let truth = kernel_truth.surface;
                report.analytic_faces += 1;
                if !face_ids.is_empty() && !face_ids.contains(&face.id) {
                    continue;
                }
                report.selected_analytic_faces += 1;
                let location = Some((solid_index, shell_index, face_index, face.id));
                emit_progress(
                    progress,
                    &path_string,
                    StepValidationPhase::FaceStarted,
                    location,
                    None,
                    Some(truth),
                    None,
                    None,
                    face.name.clone(),
                    None,
                );
                let started = Instant::now();
                emit_progress(
                    progress,
                    &path_string,
                    StepValidationPhase::TessellationStarted,
                    location,
                    None,
                    Some(truth),
                    None,
                    None,
                    None,
                    None,
                );
                let mut fallback_evidence = None;
                let kernel_mesh = match brep_kernel::tessellate_face(
                    face,
                    brep_kernel::TessellationOptions {
                        slabs_per_span_u: validation.tessellation_slabs_u.max(1),
                        steps_per_span_v: validation.tessellation_steps_v.max(1),
                    },
                    face_index as u32,
                ) {
                    Ok(mesh) if !mesh.indices.is_empty() => mesh,
                    Ok(_)
                        if matches!(
                            truth,
                            AnalyticSurface::Torus(_) | AnalyticSurface::Plane(_)
                        ) =>
                    {
                        let primary_failure = "tessellator returned no triangles".to_owned();
                        let (method_name, fallback_result) = match truth {
                            AnalyticSurface::Torus(_) => (
                                torus_fallback::METHOD_NAME,
                                observation_fallback.tessellate(source_face_index, face),
                            ),
                            AnalyticSurface::Plane(_) => (
                                torus_fallback::TRIM_CURVE_TRIANGLE_METHOD_NAME,
                                observation_fallback.tessellate_projected_trim_curve_triangle(
                                    source_face_index,
                                    face,
                                ),
                            ),
                            _ => unreachable!(),
                        };
                        match fallback_result {
                            Ok(fallback) if !fallback.mesh.indices.is_empty() => {
                                fallback_evidence = Some(TessellationFallbackEvidence {
                                    solid_index,
                                    shell_index,
                                    face_index,
                                    face_id: face.id,
                                    face_name: face.name.clone(),
                                    truth,
                                    primary_failure,
                                    method: fallback.method,
                                    chord_tolerance: observation_fallback.chord_tolerance(),
                                    triangles: fallback.mesh.indices.len() / 3,
                                    vertices: fallback.mesh.positions.len() / 3,
                                    surface_projected_vertices: fallback.surface_projected_vertices,
                                    max_surface_projection_distance: fallback
                                        .max_surface_projection_distance,
                                    surface_projection_tolerance: fallback
                                        .surface_projection_tolerance,
                                });
                                fallback.mesh
                            }
                            Ok(_) => {
                                let detail = format!(
                                    "{primary_failure}; {} fallback returned no triangles",
                                    method_name
                                );
                                record_tessellation_failure(
                                    &mut report,
                                    progress,
                                    &path_string,
                                    location,
                                    solid_index,
                                    shell_index,
                                    face_index,
                                    face,
                                    truth,
                                    None,
                                    detail,
                                );
                                continue;
                            }
                            Err(error) => {
                                let detail = format!(
                                    "{primary_failure}; {} fallback failed: {error}",
                                    method_name
                                );
                                record_tessellation_failure(
                                    &mut report,
                                    progress,
                                    &path_string,
                                    location,
                                    solid_index,
                                    shell_index,
                                    face_index,
                                    face,
                                    truth,
                                    None,
                                    detail,
                                );
                                continue;
                            }
                        }
                    }
                    Ok(_) => {
                        let detail = "tessellator returned no triangles".to_owned();
                        record_tessellation_failure(
                            &mut report,
                            progress,
                            &path_string,
                            location,
                            solid_index,
                            shell_index,
                            face_index,
                            face,
                            truth,
                            None,
                            detail,
                        );
                        continue;
                    }
                    Err(error) => {
                        report.tessellation_failures += 1;
                        report.face_diagnostics.push(StepFaceDiagnostic {
                            solid_index,
                            shell_index,
                            face_index,
                            face_id: face.id,
                            face_name: face.name.clone(),
                            kind: StepFaceDiagnosticKind::TessellationFailed,
                            truth: Some(truth),
                            detail: error.clone(),
                        });
                        emit_progress(
                            progress,
                            &path_string,
                            StepValidationPhase::TessellationFailed,
                            location,
                            None,
                            Some(truth),
                            None,
                            Some(false),
                            Some(error),
                            None,
                        );
                        continue;
                    }
                };
                let tessellation_millis = started.elapsed().as_secs_f64() * 1_000.0;
                let mesh = match mesh_from_kernel(&kernel_mesh) {
                    Ok(mesh) => mesh,
                    Err(error) => {
                        let detail = fallback_failure_detail(
                            fallback_evidence.as_ref(),
                            &format!("observation mesh conversion failed: {error}"),
                        );
                        record_tessellation_failure(
                            &mut report,
                            progress,
                            &path_string,
                            location,
                            solid_index,
                            shell_index,
                            face_index,
                            face,
                            truth,
                            None,
                            detail,
                        );
                        continue;
                    }
                };
                let mesh_size = Some((mesh.triangles.len(), mesh.vertices.len()));
                let scale = mesh_scale(&mesh.vertices);
                // Conversion validates buffers and indices, while analysis is
                // what detects tessellations whose triangles are all
                // geometrically degenerate. Classify those as tessellation
                // failures once instead of producing one fitter failure per
                // requested recognition mode.
                let analyzed = match mesh.analyze(&crate::MeshAnalysisOptions::default()) {
                    Ok(analyzed) => analyzed,
                    Err(error) => {
                        let detail = fallback_failure_detail(
                            fallback_evidence.as_ref(),
                            &format!("observation mesh analysis failed: {error}"),
                        );
                        record_tessellation_failure(
                            &mut report,
                            progress,
                            &path_string,
                            location,
                            solid_index,
                            shell_index,
                            face_index,
                            face,
                            truth,
                            mesh_size,
                            detail,
                        );
                        continue;
                    }
                };
                if let Some(evidence) = fallback_evidence {
                    report.tessellation_fallbacks += 1;
                    report.tessellation_fallback_evidence.push(evidence);
                }
                emit_progress(
                    progress,
                    &path_string,
                    StepValidationPhase::TessellationCompleted,
                    location,
                    None,
                    Some(truth),
                    mesh_size,
                    Some(true),
                    Some(report.tessellation_fallback_evidence.last().filter(|evidence| {
                        evidence.solid_index == solid_index
                            && evidence.shell_index == shell_index
                            && evidence.face_index == face_index
                    }).map_or_else(
                        || format!("{tessellation_millis:.3} ms"),
                        |evidence| format!(
                            "{tessellation_millis:.3} ms; fallback={:?}, primary_failure={}, chord_tolerance={:.6e}",
                            evidence.method, evidence.primary_failure, evidence.chord_tolerance
                        ),
                    )),
                    None,
                );
                let tolerance = validation
                    .distance_tolerance
                    .max(validation.relative_face_tolerance * scale);
                let options = RecognitionOptions {
                    distance_tolerance: tolerance,
                    // The relative face tolerance is already folded into the
                    // scale-aware absolute tolerance immediately above.
                    relative_tolerance: 0.0,
                    normal_tolerance: validation.normal_tolerance_degrees.to_radians(),
                    minimum_support: 1,
                    max_hypotheses: validation.max_hypotheses.max(1),
                    max_refinement_iterations: validation.max_refinement_iterations.max(1),
                    discover_regions: false,
                    sampling: SamplingMode::Vertices,
                    ..RecognitionOptions::default()
                };
                let triangles: Vec<_> = (0..mesh.triangles.len()).collect();
                let initial_guess = perturbed_initial(truth, scale);
                let fixed = constrained_fixed_parameters(truth);
                let constrained_initial =
                    restore_fixed_parameters(truth, perturbed_initial(truth, scale), fixed);
                let mut modes = Vec::with_capacity(validation.enabled_modes.len());
                for mode in ValidationMode::ALL.into_iter().filter(|mode| {
                    validation.enabled_modes.is_empty() || validation.enabled_modes.contains(mode)
                }) {
                    let hint = match mode {
                        ValidationMode::Unknown => SurfaceHint::Unknown,
                        ValidationMode::KnownType => SurfaceHint::KnownType {
                            surface_type: truth.surface_type(),
                        },
                        ValidationMode::InitialGuess => SurfaceHint::InitialGuess {
                            surface: initial_guess,
                            trust: MetadataTrust::InitialGuess,
                        },
                        ValidationMode::Constrained => SurfaceHint::Constrained {
                            surface_type: truth.surface_type(),
                            constraints: SurfaceConstraints {
                                initial: Some(constrained_initial),
                                fixed,
                            },
                            trust: MetadataTrust::StrongHint,
                        },
                        ValidationMode::ExactCandidate => {
                            SurfaceHint::ExactCandidate { surface: truth }
                        }
                    };
                    emit_progress(
                        progress,
                        &path_string,
                        StepValidationPhase::ModeStarted,
                        location,
                        Some(mode),
                        Some(truth),
                        mesh_size,
                        None,
                        None,
                        None,
                    );
                    let mut mode_report = run_mode(
                        mode,
                        &hint,
                        ModeRunInput {
                            mesh: &mesh,
                            analyzed: &analyzed,
                            triangles: &triangles,
                            options: &options,
                            truth,
                            expected_orientation: kernel_truth.orientation,
                        },
                    );
                    apply_ambiguity_evidence(&mut mode_report);
                    let mode_detail = mode_report.ambiguity_evidence.as_ref().map_or_else(
                        || mode_report.error.clone(),
                        |evidence| {
                            Some(format!(
                                "carrier unobservable; sampled mesh also matches {}",
                                evidence.alternative_surface.surface_type().name()
                            ))
                        },
                    );
                    emit_progress(
                        progress,
                        &path_string,
                        StepValidationPhase::ModeCompleted,
                        location,
                        Some(mode),
                        Some(truth),
                        mesh_size,
                        Some(mode_report.passed),
                        mode_detail,
                        Some(mode_report.clone()),
                    );
                    modes.push(mode_report);
                }
                report.cases.push(FaceValidationReport {
                    solid_index,
                    shell_index,
                    face_index,
                    face_id: face.id,
                    face_name: face.name.clone(),
                    same_sense: face.same_sense,
                    truth,
                    triangles: mesh.triangles.len(),
                    vertices: mesh.vertices.len(),
                    face_scale: scale,
                    distance_tolerance: tolerance,
                    tessellation_millis,
                    modes,
                });
                report.validated_faces += 1;
                emit_progress(
                    progress,
                    &path_string,
                    StepValidationPhase::FaceCompleted,
                    location,
                    None,
                    Some(truth),
                    mesh_size,
                    Some(report.cases.last().is_some_and(|face| {
                        face.modes
                            .iter()
                            .all(|mode| mode.passed || mode.unobservable)
                    })),
                    None,
                    None,
                );
            }
        }
    }
    debug_assert!(report.coverage_invariants_hold());
    emit_progress(
        progress,
        &path_string,
        StepValidationPhase::FileCompleted,
        None,
        None,
        None,
        None,
        Some(report.cases.iter().all(|face| {
            face.modes
                .iter()
                .all(|mode| mode.passed || mode.unobservable)
        })),
        Some(format!(
            "analytic_faces={}, selected_analytic_faces={}, validated_faces={}, tessellation_failures={}, tessellation_fallbacks={}",
            report.analytic_faces,
            report.selected_analytic_faces,
            report.validated_faces,
            report.tessellation_failures,
            report.tessellation_fallbacks
        )),
        None,
    );
    report
}

struct ModeRunInput<'a> {
    mesh: &'a crate::Mesh,
    analyzed: &'a AnalyzedMesh,
    triangles: &'a [usize],
    options: &'a RecognitionOptions,
    truth: AnalyticSurface,
    expected_orientation: i8,
}

fn run_mode(
    mode: ValidationMode,
    hint: &SurfaceHint,
    input: ModeRunInput<'_>,
) -> ModeValidationReport {
    let ModeRunInput {
        mesh,
        analyzed,
        triangles,
        options,
        truth,
        expected_orientation,
    } = input;
    let started = Instant::now();
    match reconstruct_analyzed(analyzed, triangles, hint, options) {
        Ok(result) => {
            let type_matches = result.surface.surface_type() == truth.surface_type();
            let parameter = type_matches.then(|| parameter_error(truth, result.surface));
            let parameter_equivalent =
                type_matches && carrier_parameters_equivalent(mesh, truth, result.surface);
            let orientation_matches =
                type_matches && orientation_against_truth(&result, truth) == expected_orientation;
            let exact_parameters_reused = result.diagnostics.exact_parameters_reused;
            let fixed_parameters_preserved =
                matches!(mode, ValidationMode::Constrained).then(|| {
                    let fixed = constrained_fixed_parameters(truth);
                    result.diagnostics.fixed_parameters == fixed
                        && fixed_parameters_match(truth, result.surface, fixed)
                });
            let ambiguity_evidence = (!parameter_equivalent)
                .then(|| {
                    carrier_ambiguity_between(
                        mesh,
                        result.surface,
                        result.orientation,
                        truth,
                        expected_orientation,
                    )
                })
                .flatten();
            let passed = type_matches
                && parameter_equivalent
                && orientation_matches
                && (!matches!(mode, ValidationMode::ExactCandidate) || exact_parameters_reused)
                && fixed_parameters_preserved.unwrap_or(true);
            ModeValidationReport {
                mode,
                elapsed_millis: started.elapsed().as_secs_f64() * 1_000.0,
                error: (!parameter_equivalent && type_matches).then(|| {
                    let error = parameter.unwrap_or_default();
                    format!(
                        "observable parameter mismatch: position={:.3e}, direction={:.3e}deg, radius={:.3e}, major_radius={:.3e}, angle={:.3e}deg",
                        error.position,
                        error.direction_degrees,
                        error.radius,
                        error.major_radius,
                        error.angle_degrees,
                    )
                }),
                parameter_error: parameter,
                result: Some(result),
                passed,
                unobservable: false,
                ambiguity_evidence,
                type_matches,
                parameter_equivalent,
                orientation_matches,
                exact_parameters_reused,
                fixed_parameters_preserved,
            }
        }
        Err(error) => ModeValidationReport {
            mode,
            elapsed_millis: started.elapsed().as_secs_f64() * 1_000.0,
            error: Some(error.to_string()),
            result: None,
            passed: false,
            unobservable: false,
            ambiguity_evidence: None,
            type_matches: false,
            parameter_equivalent: false,
            orientation_matches: false,
            exact_parameters_reused: false,
            fixed_parameters_preserved: matches!(mode, ValidationMode::Constrained)
                .then_some(false),
            parameter_error: None,
        },
    }
}

fn apply_ambiguity_evidence(report: &mut ModeValidationReport) {
    // Exact metadata is the highest-authority truth and must pass exact reuse;
    // it can never receive an ambiguity exception. An exception also requires
    // this mode's own successfully returned carrier and may not conceal a
    // constraint-preservation defect. `run_mode` computes the retained witness
    // directly from that returned carrier over every sampled point and oriented
    // normal; evidence from another recognition path is never substituted.
    let evidence_matches_returned_carrier = report
        .result
        .as_ref()
        .zip(report.ambiguity_evidence.as_ref())
        .is_some_and(|(result, evidence)| result.surface == evidence.alternative_surface);
    let own_carrier_is_ambiguous = !report.passed
        && !matches!(report.mode, ValidationMode::ExactCandidate)
        && report.fixed_parameters_preserved != Some(false)
        && evidence_matches_returned_carrier;
    if own_carrier_is_ambiguous {
        report.unobservable = true;
        // A quantitative ambiguity is an oracle outcome, not a recognizer
        // execution error or an observable parameter mismatch.
        report.error = None;
    } else {
        report.ambiguity_evidence = None;
        report.unobservable = false;
    }
}

fn carrier_ambiguity_between(
    mesh: &crate::Mesh,
    alternative_surface: AnalyticSurface,
    alternative_orientation: i8,
    truth: AnalyticSurface,
    expected_orientation: i8,
) -> Option<CarrierAmbiguityEvidence> {
    if mesh.vertices.is_empty() {
        return None;
    }
    let (position_threshold, normal_threshold) = carrier_ambiguity_thresholds(mesh, truth);
    let carriers_distinct = !carrier_parameters_equivalent(mesh, truth, alternative_surface);
    if !carriers_distinct {
        return None;
    }

    let mut max_truth_error = 0.0_f64;
    let mut max_alternative_error = 0.0_f64;
    let mut max_normal_disagreement = 0.0_f64;
    for &point in &mesh.vertices {
        if !point.is_finite() {
            return None;
        }
        let truth_error = truth.signed_distance(point).abs();
        let alternative_error = alternative_surface.signed_distance(point).abs();
        let truth_normal = truth.normal_at(point)? * expected_orientation as f64;
        let alternative_normal =
            alternative_surface.normal_at(point)? * alternative_orientation as f64;
        let normal_disagreement = truth_normal.dot(alternative_normal).clamp(-1.0, 1.0).acos();
        if !truth_error.is_finite()
            || !alternative_error.is_finite()
            || !truth_normal.is_finite()
            || !alternative_normal.is_finite()
            || !normal_disagreement.is_finite()
        {
            return None;
        }
        max_truth_error = max_truth_error.max(truth_error);
        max_alternative_error = max_alternative_error.max(alternative_error);
        max_normal_disagreement = max_normal_disagreement.max(normal_disagreement);
    }
    if max_truth_error > position_threshold
        || max_alternative_error > position_threshold
        || max_normal_disagreement > normal_threshold
    {
        return None;
    }
    Some(CarrierAmbiguityEvidence {
        alternative_surface,
        max_truth_error,
        max_alternative_error,
        max_oriented_normal_disagreement: max_normal_disagreement,
        position_threshold,
        normal_threshold,
    })
}

fn carrier_equivalence_thresholds(mesh: &crate::Mesh, truth: AnalyticSurface) -> (f64, f64) {
    carrier_thresholds(mesh, truth, numerical::COORDINATE_ROUNDOFF_RELATIVE)
}

fn carrier_ambiguity_thresholds(mesh: &crate::Mesh, truth: AnalyticSurface) -> (f64, f64) {
    // Ambiguity is deliberately wider than strict parameter equivalence. It
    // must cover the accumulated coordinate roundoff of tessellation plus
    // imported placements, while still requiring both carriers and their
    // oriented normal fields to agree at every sampled vertex.
    carrier_thresholds(
        mesh,
        truth,
        numerical::TESSELLATED_COORDINATE_EVIDENCE_ROUNDOFF_RELATIVE,
    )
}

fn carrier_thresholds(
    mesh: &crate::Mesh,
    truth: AnalyticSurface,
    coordinate_roundoff_relative: f64,
) -> (f64, f64) {
    // Observability is a property of the sampled evidence, not of an arbitrary
    // carrier gauge. A plane or cylinder origin can be moved extremely far
    // from a small local patch without reducing the precision of the mesh
    // coordinates, so those carrier coordinates must not loosen this gate.
    // Far-origin meshes still receive the appropriate coordinate-conditioned
    // allowance because their vertex coordinates themselves carry that scale.
    let coordinate_scale = mesh
        .vertices
        .iter()
        .map(|point| point.x.abs().max(point.y.abs()).max(point.z.abs()))
        .fold(1.0_f64, f64::max);
    // Imported STEP tessellations routinely accumulate several stages of coordinate
    // transform and analytic-surface evaluation roundoff. A cone's apex is not a
    // gauge: evaluating a genuinely shallow cone subtracts that intrinsic, remote
    // point and has a correspondingly larger roundoff bound. Keep that conditioning
    // term truth-only so an arbitrary alternative cannot enlarge its own gate.
    let cone_conditioning_scale = match truth {
        AnalyticSurface::Cone(cone) => cone
            .apex
            .x
            .abs()
            .max(cone.apex.y.abs())
            .max(cone.apex.z.abs()),
        _ => 0.0,
    };
    let position_threshold = (coordinate_roundoff_relative * coordinate_scale)
        .max(numerical::CONE_APEX_ROUNDOFF_RELATIVE * cone_conditioning_scale);
    // About 7.3e-4 degrees: tight enough to represent numerical derivative
    // and tiny-patch conditioning noise, far below the recognition gate.
    let normal_threshold = numerical::NORMAL_EQUIVALENCE_RADIANS;
    (position_threshold, normal_threshold)
}

fn carrier_parameters_equivalent(
    mesh: &crate::Mesh,
    truth: AnalyticSurface,
    actual: AnalyticSurface,
) -> bool {
    if truth.surface_type() != actual.surface_type() {
        return false;
    }
    let (position_threshold, normal_threshold) = carrier_equivalence_thresholds(mesh, truth);
    let parameter = parameter_error(truth, actual);
    parameter.position <= position_threshold
        && parameter.radius <= position_threshold
        && parameter.major_radius <= position_threshold
        && parameter.direction_degrees.to_radians() <= normal_threshold
        && parameter.angle_degrees.to_radians() <= normal_threshold
}

fn perturbed_initial(surface: AnalyticSurface, face_scale: f64) -> AnalyticSurface {
    let position_delta =
        face_scale.max(scalar::GEOMETRIC_SCALE_FLOOR) * numerical::INITIAL_PERTURBATION_RELATIVE;
    let tilt = numerical::INITIAL_PERTURBATION_RELATIVE;
    match surface {
        AnalyticSurface::Plane(mut plane) => {
            let tangent = plane
                .normal
                .orthonormal_basis()
                .map(|basis| basis.0)
                .unwrap_or(Vec3::X);
            plane.origin += plane.normal * position_delta;
            plane.normal = (plane.normal + tangent * tilt)
                .normalized()
                .unwrap_or(plane.normal);
            AnalyticSurface::Plane(plane)
        }
        AnalyticSurface::Sphere(mut sphere) => {
            sphere.center +=
                Vec3::new(position_delta, -0.5 * position_delta, 0.25 * position_delta);
            sphere.radius *= 1.001;
            AnalyticSurface::Sphere(sphere)
        }
        AnalyticSurface::Cylinder(mut cylinder) => {
            let tangent = cylinder
                .axis
                .orthonormal_basis()
                .map(|basis| basis.0)
                .unwrap_or(Vec3::X);
            cylinder.axis_origin += tangent * position_delta;
            cylinder.axis = (cylinder.axis + tangent * tilt)
                .normalized()
                .unwrap_or(cylinder.axis);
            cylinder.radius *= 1.001;
            AnalyticSurface::Cylinder(cylinder)
        }
        AnalyticSurface::Cone(mut cone) => {
            let tangent = cone
                .axis
                .orthonormal_basis()
                .map(|basis| basis.0)
                .unwrap_or(Vec3::X);
            cone.apex += tangent * position_delta;
            cone.axis = (cone.axis + tangent * tilt)
                .normalized()
                .unwrap_or(cone.axis);
            // Move slightly toward 45 degrees, which stays strictly inside
            // the valid open interval for every valid cone.
            cone.half_angle += (std::f64::consts::FRAC_PI_4 - cone.half_angle)
                * numerical::INITIAL_PERTURBATION_RELATIVE;
            AnalyticSurface::Cone(cone)
        }
        AnalyticSurface::Torus(mut torus) => {
            let tangent = torus
                .axis
                .orthonormal_basis()
                .map(|basis| basis.0)
                .unwrap_or(Vec3::X);
            torus.center += tangent * position_delta;
            torus.axis = (torus.axis + tangent * tilt)
                .normalized()
                .unwrap_or(torus.axis);
            torus.major_radius *= 1.001;
            torus.minor_radius *= 1.001;
            AnalyticSurface::Torus(torus)
        }
    }
}

fn constrained_fixed_parameters(surface: AnalyticSurface) -> ConstraintMask {
    match surface {
        AnalyticSurface::Plane(_) => ConstraintMask {
            axis_or_normal: true,
            ..ConstraintMask::default()
        },
        AnalyticSurface::Sphere(_) | AnalyticSurface::Cylinder(_) => ConstraintMask {
            radius: true,
            ..ConstraintMask::default()
        },
        AnalyticSurface::Cone(_) => ConstraintMask {
            angle: true,
            ..ConstraintMask::default()
        },
        AnalyticSurface::Torus(_) => ConstraintMask {
            major_radius: true,
            ..ConstraintMask::default()
        },
    }
}

fn fixed_parameters_match(
    expected: AnalyticSurface,
    actual: AnalyticSurface,
    fixed: ConstraintMask,
) -> bool {
    match (expected, actual) {
        (AnalyticSurface::Plane(a), AnalyticSurface::Plane(b)) => {
            (!fixed.origin_or_center || a.origin == b.origin)
                && (!fixed.axis_or_normal || a.normal == b.normal)
        }
        (AnalyticSurface::Sphere(a), AnalyticSurface::Sphere(b)) => {
            (!fixed.origin_or_center || a.center == b.center)
                && (!fixed.radius || a.radius == b.radius)
        }
        (AnalyticSurface::Cylinder(a), AnalyticSurface::Cylinder(b)) => {
            (!fixed.origin_or_center || a.axis_origin == b.axis_origin)
                && (!fixed.axis_or_normal || a.axis == b.axis)
                && (!fixed.radius || a.radius == b.radius)
        }
        (AnalyticSurface::Cone(a), AnalyticSurface::Cone(b)) => {
            (!fixed.origin_or_center || a.apex == b.apex)
                && (!fixed.axis_or_normal || a.axis == b.axis)
                && (!fixed.angle || a.half_angle == b.half_angle)
        }
        (AnalyticSurface::Torus(a), AnalyticSurface::Torus(b)) => {
            (!fixed.origin_or_center || a.center == b.center)
                && (!fixed.axis_or_normal || a.axis == b.axis)
                && (!fixed.major_radius || a.major_radius == b.major_radius)
                && (!fixed.radius || a.minor_radius == b.minor_radius)
        }
        _ => false,
    }
}

fn restore_fixed_parameters(
    expected: AnalyticSurface,
    perturbed: AnalyticSurface,
    fixed: ConstraintMask,
) -> AnalyticSurface {
    match (expected, perturbed) {
        (AnalyticSurface::Plane(a), AnalyticSurface::Plane(mut b)) => {
            if fixed.origin_or_center {
                b.origin = a.origin;
            }
            if fixed.axis_or_normal {
                b.normal = a.normal;
            }
            AnalyticSurface::Plane(b)
        }
        (AnalyticSurface::Sphere(a), AnalyticSurface::Sphere(mut b)) => {
            if fixed.origin_or_center {
                b.center = a.center;
            }
            if fixed.radius {
                b.radius = a.radius;
            }
            AnalyticSurface::Sphere(b)
        }
        (AnalyticSurface::Cylinder(a), AnalyticSurface::Cylinder(mut b)) => {
            if fixed.origin_or_center {
                b.axis_origin = a.axis_origin;
            }
            if fixed.axis_or_normal {
                b.axis = a.axis;
            }
            if fixed.radius {
                b.radius = a.radius;
            }
            AnalyticSurface::Cylinder(b)
        }
        (AnalyticSurface::Cone(a), AnalyticSurface::Cone(mut b)) => {
            if fixed.origin_or_center {
                b.apex = a.apex;
            }
            if fixed.axis_or_normal {
                b.axis = a.axis;
            }
            if fixed.angle {
                b.half_angle = a.half_angle;
            }
            AnalyticSurface::Cone(b)
        }
        (AnalyticSurface::Torus(a), AnalyticSurface::Torus(mut b)) => {
            if fixed.origin_or_center {
                b.center = a.center;
            }
            if fixed.axis_or_normal {
                b.axis = a.axis;
            }
            if fixed.major_radius {
                b.major_radius = a.major_radius;
            }
            if fixed.radius {
                b.minor_radius = a.minor_radius;
            }
            AnalyticSurface::Torus(b)
        }
        (_, perturbed) => perturbed,
    }
}

fn parameter_error(expected: AnalyticSurface, actual: AnalyticSurface) -> ParameterError {
    let mut error = ParameterError::default();
    match (expected, actual) {
        (AnalyticSurface::Plane(a), AnalyticSurface::Plane(b)) => {
            error.position = (b.origin - a.origin).dot(a.normal).abs();
            error.direction_degrees = direction_error(a.normal, b.normal);
        }
        (AnalyticSurface::Sphere(a), AnalyticSurface::Sphere(b)) => {
            error.position = (a.center - b.center).length();
            error.radius = (a.radius - b.radius).abs();
        }
        (AnalyticSurface::Cylinder(a), AnalyticSurface::Cylinder(b)) => {
            error.position = line_error(a.axis_origin, a.axis, b.axis_origin, b.axis);
            error.direction_degrees = direction_error(a.axis, b.axis);
            error.radius = (a.radius - b.radius).abs();
        }
        (AnalyticSurface::Cone(a), AnalyticSurface::Cone(b)) => {
            error.position = (a.apex - b.apex).length();
            error.direction_degrees = oriented_direction_error(a.axis, b.axis);
            error.angle_degrees = (a.half_angle - b.half_angle).abs().to_degrees();
        }
        (AnalyticSurface::Torus(a), AnalyticSurface::Torus(b)) => {
            error.position = (a.center - b.center).length();
            error.direction_degrees = direction_error(a.axis, b.axis);
            error.major_radius = (a.major_radius - b.major_radius).abs();
            error.radius = (a.minor_radius - b.minor_radius).abs();
        }
        _ => {}
    }
    error
}

/// Express the fit's observed mesh orientation in the parameter gauge of the
/// STEP truth carrier. Plane fitting is free to reverse its normal, in which
/// case the fit's orientation sign reverses too even though the oriented face
/// is unchanged. Other supported carrier gauge changes (cylinder/torus axis
/// reversal) leave their geometric normals unchanged.
fn orientation_against_truth(result: &SurfaceFitResult, truth: AnalyticSurface) -> i8 {
    let gauge = match (truth, result.surface) {
        (AnalyticSurface::Plane(expected), AnalyticSurface::Plane(actual))
            if expected.normal.dot(actual.normal) < 0.0 =>
        {
            -1
        }
        _ => 1,
    };
    result.orientation * gauge
}

fn direction_error(a: Vec3, b: Vec3) -> f64 {
    a.dot(b).abs().clamp(-1.0, 1.0).acos().to_degrees()
}

fn oriented_direction_error(a: Vec3, b: Vec3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos().to_degrees()
}

fn line_error(a_origin: Vec3, a_axis: Vec3, b_origin: Vec3, b_axis: Vec3) -> f64 {
    let direction = (a_axis + b_axis * a_axis.dot(b_axis).signum())
        .normalized()
        .unwrap_or(a_axis);
    ((b_origin - a_origin) - direction * (b_origin - a_origin).dot(direction)).length()
}

fn mesh_scale(vertices: &[Vec3]) -> f64 {
    let mut min = vertices[0];
    let mut max = vertices[0];
    for &point in &vertices[1..] {
        min.x = min.x.min(point.x);
        min.y = min.y.min(point.y);
        min.z = min.z.min(point.z);
        max.x = max.x.max(point.x);
        max.y = max.y.max(point.y);
        max.z = max.z.max(point.z);
    }
    (max - min).length().max(scalar::GEOMETRIC_SCALE_FLOOR)
}

/// Stable, spreadsheet-friendly summary. Detailed parameters remain in JSON.
pub fn reports_to_csv(reports: &[StepFileReport]) -> String {
    let mut out = String::from("path,solid,shell,face,face_id,truth,triangles,mode,success,type_matches,orientation_matches,exact_parameters_reused,fixed_parameters_preserved,rms_error,max_error,position_error,direction_degrees,radius_error,major_radius_error,angle_degrees,elapsed_ms,error,unobservable,ambiguity_alternative,ambiguity_truth_max,ambiguity_alternative_max,ambiguity_normal_max,parameter_equivalent,tessellation_fallback,fallback_method,fallback_chord_tolerance\n");
    for file in reports {
        for face in &file.cases {
            let fallback = file.tessellation_fallback_evidence.iter().find(|evidence| {
                evidence.solid_index == face.solid_index
                    && evidence.shell_index == face.shell_index
                    && evidence.face_index == face.face_index
                    && evidence.face_id == face.face_id
            });
            for mode in &face.modes {
                let fit = mode.result.as_ref();
                let parameter = mode.parameter_error.unwrap_or_default();
                let error = mode.error.as_deref().unwrap_or("").replace('"', "\"\"");
                let ambiguity = mode.ambiguity_evidence.as_ref();
                let row = [
                    format!("\"{}\"", file.path.replace('"', "\"\"")),
                    face.solid_index.to_string(),
                    face.shell_index.to_string(),
                    face.face_index.to_string(),
                    face.face_id.to_string(),
                    face.truth.surface_type().name().to_string(),
                    face.triangles.to_string(),
                    format!("{:?}", mode.mode),
                    mode.passed.to_string(),
                    mode.type_matches.to_string(),
                    mode.orientation_matches.to_string(),
                    mode.exact_parameters_reused.to_string(),
                    mode.fixed_parameters_preserved
                        .map(|value| value.to_string())
                        .unwrap_or_default(),
                    fit.map(|v| v.metrics.rms_error)
                        .unwrap_or(f64::NAN)
                        .to_string(),
                    fit.map(|v| v.metrics.max_error)
                        .unwrap_or(f64::NAN)
                        .to_string(),
                    parameter.position.to_string(),
                    parameter.direction_degrees.to_string(),
                    parameter.radius.to_string(),
                    parameter.major_radius.to_string(),
                    parameter.angle_degrees.to_string(),
                    mode.elapsed_millis.to_string(),
                    format!("\"{error}\""),
                    mode.unobservable.to_string(),
                    ambiguity
                        .map(|value| value.alternative_surface.surface_type().name())
                        .unwrap_or("")
                        .to_string(),
                    ambiguity
                        .map(|value| value.max_truth_error)
                        .unwrap_or(f64::NAN)
                        .to_string(),
                    ambiguity
                        .map(|value| value.max_alternative_error)
                        .unwrap_or(f64::NAN)
                        .to_string(),
                    ambiguity
                        .map(|value| value.max_oriented_normal_disagreement)
                        .unwrap_or(f64::NAN)
                        .to_string(),
                    mode.parameter_equivalent.to_string(),
                    fallback.is_some().to_string(),
                    fallback
                        .map(|evidence| format!("{:?}", evidence.method))
                        .unwrap_or_default(),
                    fallback
                        .map(|evidence| evidence.chord_tolerance.to_string())
                        .unwrap_or_default(),
                ];
                out.push_str(&row.join(","));
                out.push('\n');
            }
        }
    }
    out
}

