//! STL-mesh to STEP conversion orchestration.
//!
//! The kernel's mesh segmentation runs first on every closed mesh. When it
//! reads the mesh as a whole primitive — every triangle on one of at most
//! three analytic regions — RANSAC recognition runs and its fitted
//! parameters build an exact sphere, torus, capped cylinder or capped cone.
//! Otherwise RANSAC is skipped: every consumer of its result needs that
//! whole-primitive shape, and on an eleven-thousand-facet part it cost 75 s
//! for a result nothing read. Safe plane, cylinder, cone, and sphere regions
//! can be rebuilt analytically beside unsupported source-triangle faces; an
//! analytic region is locally demoted when its shared boundary cannot be
//! represented exactly. If no validated mixed shell can be built, the complete
//! source mesh is sent through the kernel's repairable faceted importer. No
//! source triangle is omitted.
//!
//! Every one of those analytic lanes needs a closed two-manifold, which a mesh
//! FILE frequently is not — a dropped facet, a duplicated sheet, a degenerate
//! sliver. Such a body is repaired FIRST (`repair_triangle_soup`, the same
//! weld/prune/cap the faceted importer applies) and the analytic lanes run on
//! the repaired mesh, so a defect costs a body only the facets it touches
//! instead of every curved surface in it. A body whose repair still does not
//! close keeps the refusal it has always had, and a closed file is never
//! repaired: its output is unchanged.

use crate::{
    recognize_surfaces_with_unresolved, reconstruct_surface, AnalyticSurface, Mesh,
    MeshAnalysisOptions, PhaseTimings, RecognitionOptions, SamplingMode, SurfaceFitResult,
    SurfaceHint, SurfaceRegion, SurfaceType, Vec3,
};
use brep_kernel::{
    audit_step_manifold, export_step, import_step, make_cone_brep, make_cylinder_brep,
    make_sphere_brep, make_torus_brep, merge_same_surface_faces, mesh_regions_to_brep,
    mesh_to_faceted_brep, repair_triangle_soup, segment_mesh_faces, MeshSegmentation,
    RegionCarrier, SegmentOptions,
    UNASSIGNED_REGION,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::fmt::{Display, Formatter};
use web_time::Instant;

/// Whether an unprovable analytic conversion may preserve the mesh as a
/// repaired faceted STEP solid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize)]
pub enum ConversionPolicy {
    /// Preserve arbitrary repairable input by falling back to faceted BREP.
    #[default]
    AllowFacetedFallback,
    /// Return an error unless the complete mesh can be rebuilt analytically.
    RequireFullyAnalytic,
}

/// Controls recognition and STEP topology construction.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
pub struct StlConversionOptions {
    /// RANSAC recognition settings. Recognition runs only when the kernel
    /// segmentation reads the mesh as a whole primitive (see the module doc);
    /// every other mesh is rebuilt from the segmentation alone.
    pub recognition: RecognitionOptions,
    /// Analytic-only versus repairable-faceted behavior.
    pub policy: ConversionPolicy,
    /// Try the kernel's plane/cylinder/cone region rebuilder when the kernel
    /// segmentation proves that the whole mesh is plane/cylinder/cone.
    pub try_kernel_analytic_rebuild: bool,
    /// Try the conservative mixed plane/cylinder/cone/sphere/faceted topology builder.
    ///
    /// This is enabled by default so recognized safe regions remain analytic
    /// even when other regions cannot be represented analytically. Disabling
    /// it is primarily useful for diagnosing the fully faceted repair path.
    pub try_hybrid_rebuild: bool,
    /// Vertex weld tolerance for hybrid segmentation and faceted repair;
    /// non-positive derives a scale-relative tolerance in the kernel.
    pub weld_tolerance: f64,
    /// Absolute coordinate uncertainty introduced by the source encoding.
    ///
    /// The converter will not run recognition below this distance even when
    /// [`RecognitionOptions::distance_tolerance`] requests a smaller value.
    /// Leave this at zero for text or double-precision sources. The CLI sets
    /// it automatically for binary STL, whose coordinates are IEEE-754 f32.
    pub coordinate_precision_tolerance: f64,
    /// Deflection angle used by the kernel analytic segmentation paths.
    pub kernel_deflection_angle_degrees: f64,
    /// Scale-relative carrier tolerance used by the kernel analytic segmentation paths.
    pub kernel_fit_tolerance: f64,
    /// Normal gate used by the kernel analytic segmentation paths.
    pub kernel_normal_tolerance_degrees: f64,
}

impl Default for StlConversionOptions {
    fn default() -> Self {
        let recognition = RecognitionOptions {
            collect_phase_timings: true,
            // Facet centroids lie on polygon chords rather than on the
            // underlying design carrier. Welded STL vertices retain the best
            // available samples of cylinders and other curved CAD surfaces.
            sampling: SamplingMode::Vertices,
            ..RecognitionOptions::default()
        };
        Self {
            recognition,
            policy: ConversionPolicy::AllowFacetedFallback,
            try_kernel_analytic_rebuild: true,
            try_hybrid_rebuild: true,
            weld_tolerance: -1.0,
            coordinate_precision_tolerance: 0.0,
            kernel_deflection_angle_degrees: 30.0,
            kernel_fit_tolerance: 1.0e-3,
            kernel_normal_tolerance_degrees: 15.0,
        }
    }
}

/// Topology path used to produce the final STEP body.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ConversionBackend {
    /// Exact full sphere from the RANSAC carrier.
    RansacSphere,
    /// Exact full ring torus from the RANSAC carrier.
    RansacTorus,
    /// Exact cylinder wall and caps from RANSAC carriers.
    RansacCappedCylinder,
    /// Exact cone wall and cap or caps from RANSAC carriers.
    RansacCappedCone,
    /// The kernel segmentation proved full plane/cylinder/cone coverage and
    /// the kernel region rebuilder built the shell from it.
    KernelAnalyticRebuild,
    /// A conservative local topology builder reconstructed a complete shell
    /// with exact plane/cylinder/cone/sphere regions and faceted unsupported regions.
    #[serde(alias = "hybrid_plane_cylinder_rebuild")]
    HybridAnalyticRebuild,
    /// Complete source triangles were preserved through mesh repair.
    FacetedRepair,
    /// Complete source triangles were repaired, then adjacent coplanar
    /// triangle faces were safely coalesced by the kernel.
    FacetedRepairCoplanarMerged,
    /// No body: every lane, faceted repair included, refused this component
    /// of a multi-body mesh. It is NOT in the STEP document; its record's
    /// `refusal` says why. The document's own backend is never this.
    Refused,
}

/// Human- and machine-readable details for one RANSAC region.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ConversionRegionReport {
    /// Selected analytic family.
    pub surface_type: SurfaceType,
    /// Fitted carrier parameters.
    pub surface: AnalyticSurface,
    /// Number of supporting input triangles.
    pub support_triangles: usize,
    /// Total area of the supporting triangles.
    pub supported_area: f64,
    /// Root-mean-square positional residual.
    pub rms_error: f64,
    /// Maximum positional residual.
    pub max_error: f64,
    /// Root-mean-square normal residual, in radians.
    pub rms_normal_error_radians: f64,
    /// Maximum normal residual, in radians.
    pub max_normal_error_radians: f64,
    /// Residual-evidence confidence score.
    pub confidence: f64,
    /// Carrier orientation relative to mesh winding.
    pub orientation: i8,
    /// Recognition-path explanation.
    pub reason: String,
    /// Production recognizer's detailed phase measurements.
    pub phase_timings: PhaseTimings,
}

/// Wall-clock measurements for conversion stages.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct ConversionTimings {
    /// Time spent in the kernel mesh segmentation that gates recognition.
    #[serde(default)]
    pub segmentation_seconds: f64,
    /// Time spent in RANSAC recognition; zero when it was skipped.
    pub recognition_seconds: f64,
    /// Time spent building and validating in-memory topology.
    pub topology_build_seconds: f64,
    /// Time spent serializing STEP.
    pub step_export_seconds: f64,
    /// Time spent auditing and round-trip importing STEP.
    pub step_validation_seconds: f64,
    /// End-to-end conversion time.
    pub total_seconds: f64,
}

/// Face and source-triangle accounting for a mixed analytic/faceted rebuild.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct HybridConversionReport {
    /// Total output BREP faces.
    pub total_faces: usize,
    /// Exact analytic plane faces.
    pub analytic_plane_faces: usize,
    /// Source triangles represented by analytic plane faces.
    pub analytic_plane_triangles: usize,
    /// Exact analytic cylinder faces.
    pub analytic_cylinder_faces: usize,
    /// Source triangles represented by analytic cylinder faces.
    pub analytic_cylinder_triangles: usize,
    /// Exact analytic cone faces.
    #[serde(default)]
    pub analytic_cone_faces: usize,
    /// Source triangles represented by analytic cone faces.
    #[serde(default)]
    pub analytic_cone_triangles: usize,
    /// Exact analytic sphere faces.
    #[serde(default)]
    pub analytic_sphere_faces: usize,
    /// Source triangles represented by analytic sphere faces.
    #[serde(default)]
    pub analytic_sphere_triangles: usize,
    /// Exact analytic torus faces.
    #[serde(default)]
    pub analytic_torus_faces: usize,
    /// Source triangles represented by analytic torus faces.
    #[serde(default)]
    pub analytic_torus_triangles: usize,
    /// Coplanar planar faces retained for unsupported or unsafe regions.
    pub faceted_faces: usize,
    /// Source triangles represented by those coplanar fallback faces.
    pub faceted_triangles: usize,
    /// Segmented analytic regions conservatively demoted to facets.
    pub demoted_regions: usize,
    /// Source triangles in demoted regions.
    pub demoted_triangles: usize,
    /// Plane regions found by the independent topology segmentation.
    pub segmentation_plane_regions: usize,
    /// Cylinder regions found by the independent topology segmentation.
    pub segmentation_cylinder_regions: usize,
    /// Cone regions found by the independent topology segmentation.
    #[serde(default)]
    pub segmentation_cone_regions: usize,
    /// Sphere regions found by the independent topology segmentation.
    #[serde(default)]
    pub segmentation_sphere_regions: usize,
    /// Torus regions found by the independent topology segmentation.
    #[serde(default)]
    pub segmentation_torus_regions: usize,
    /// Unsupported regions found by the independent topology segmentation.
    pub segmentation_unsupported_regions: usize,
}

/// One body of the document: a single vertex-connected component of the input
/// mesh, reconstructed on its own.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ComponentConversionReport {
    /// Position in the document, counting from zero.
    pub index: usize,
    /// The lowest source triangle index this component holds; what orders the
    /// bodies, and what names one in a refusal.
    pub first_triangle: usize,
    /// Vertices of this component's recognition mesh.
    pub input_vertices: usize,
    /// Triangles of this component.
    pub input_triangles: usize,
    /// Topology construction path this body took.
    pub backend: ConversionBackend,
    /// Explanation of why that backend was selected.
    pub backend_reason: String,
    /// Why an analytic backend was unavailable, for a faceted body.
    pub fallback_cause: Option<String>,
    /// Why RANSAC recognition did not run for this body; `None` when it ran.
    pub recognition_skipped: Option<String>,
    /// Accepted analytic regions for this body.
    pub recognized_regions: usize,
    /// Triangles assigned to those regions.
    pub recognized_triangles: usize,
    /// Non-degenerate triangles left unresolved.
    pub unresolved_triangles: usize,
    /// Whether this component alone is a consistently wound closed manifold.
    pub source_closed_manifold: bool,
    /// Whether the analytic lanes ran on the REPAIRED mesh because the source
    /// was not a closed two-manifold. The `input_*` counts above are always
    /// the source's. This says only which mesh the lanes were given, never
    /// that one of them succeeded — `backend` says that.
    #[serde(default)]
    pub analytic_used_repaired_mesh: bool,
    /// Mixed analytic/faceted accounting for this body.
    pub hybrid_rebuild: Option<HybridConversionReport>,
    /// Face count immediately after faceted repair, when that path was used.
    pub faceted_faces_before_merge: Option<usize>,
    /// Face count after the safe coplanar merge attempt.
    pub faceted_faces_after_merge: Option<usize>,
    /// BREP faces of the constructed solid.
    pub faces: usize,
    /// Why this body is NOT in the document: every lane refused it, and the
    /// other bodies were built without it. `None` for every built body.
    #[serde(default)]
    pub refusal: Option<String>,
    /// The source's own name for this body (a 3MF object's `name`), when the
    /// caller knows it. The converter never sets it.
    #[serde(default)]
    pub source_name: Option<String>,
}

/// Detailed conversion outcome suitable for terminal or JSON reporting.
///
/// Counts are TOTALS over every body and `regions` is their concatenation. The
/// fields that can only describe one body — `backend`, `topology_refit`, the
/// recognition tolerances — are the FIRST body's; `components` is the per-body
/// record, and `backend_reason` names every body when there is more than one.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StlConversionReport {
    /// Topology construction path used.
    pub backend: ConversionBackend,
    /// Explanation of why the backend was selected.
    pub backend_reason: String,
    /// Why an analytic backend was unavailable, for faceted output.
    pub fallback_cause: Option<String>,
    /// Number of vertices in the recognition mesh.
    pub input_vertices: usize,
    /// Number of triangles in the input.
    pub input_triangles: usize,
    /// Number of accepted analytic regions.
    pub recognized_regions: usize,
    /// Number of triangles assigned to analytic regions.
    pub recognized_triangles: usize,
    /// Number of non-degenerate triangles left unresolved.
    pub unresolved_triangles: usize,
    /// Why RANSAC recognition did not run; `None` when it ran. The region
    /// counts above are zero when this is set.
    #[serde(default)]
    pub recognition_skipped: Option<String>,
    /// Whether indexed input topology is a consistently wound closed manifold.
    pub source_closed_manifold: bool,
    /// Whether ANY body's analytic lanes ran on a repaired mesh because the
    /// source was not a closed two-manifold. Which mesh they were given, not
    /// whether one of them succeeded.
    #[serde(default)]
    pub analytic_used_repaired_mesh: bool,
    /// Absolute recognition distance requested by the caller.
    #[serde(default)]
    pub requested_distance_tolerance: f64,
    /// Minimum distance imposed by the coordinate encoding.
    #[serde(default)]
    pub coordinate_precision_tolerance: f64,
    /// Absolute distance actually supplied to recognition.
    #[serde(default)]
    pub effective_distance_tolerance: f64,
    /// Accepted region count by analytic family name.
    pub region_type_counts: BTreeMap<String, usize>,
    /// Per-region carrier and quality diagnostics.
    pub regions: Vec<ConversionRegionReport>,
    /// Topology-informed production refit used by a direct backend, when the
    /// initially selected carrier was an observational limiting family.
    pub topology_refit: Option<ConversionRegionReport>,
    /// Detailed accounting when the mixed analytic builder was used. With
    /// several bodies this sums every body that took that backend.
    #[serde(default)]
    pub hybrid_rebuild: Option<HybridConversionReport>,
    /// One record per body, in document order — the authoritative per-body
    /// account when the input held several closed shells. Always populated;
    /// a single-body conversion has exactly one entry.
    #[serde(default)]
    pub components: Vec<ComponentConversionReport>,
    /// Serialized STEP document size.
    pub exported_step_bytes: usize,
    /// Number of serialized `ADVANCED_FACE` entities.
    pub exported_advanced_faces: usize,
    /// Face count immediately after faceted repair, when that path was used.
    #[serde(default)]
    pub faceted_faces_before_merge: Option<usize>,
    /// Face count after the safe coplanar merge attempt, when faceted repair
    /// was used. This equals `faceted_faces_before_merge` if no merge was
    /// possible or the validated merge was unavailable.
    #[serde(default)]
    pub faceted_faces_after_merge: Option<usize>,
    /// Number of solids recovered by round-trip STEP import.
    pub roundtrip_solids: usize,
    /// Serialized manifold audit issues; empty on every successful result.
    pub manifold_audit_issues: Vec<String>,
    /// Stage and total wall-clock measurements.
    pub timings: ConversionTimings,
    /// Useful chronological terminal messages.
    pub messages: Vec<String>,
}

/// STEP text and its complete conversion report.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct StlConversionOutput {
    /// Validated AP242 STEP Part 21 document.
    pub step_text: String,
    /// Detailed conversion outcome.
    pub report: StlConversionReport,
}

/// A conversion failure. The message is intentionally ready for terminal use.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct StlConversionError(pub String);

impl Display for StlConversionError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(&self.0)
    }
}

impl std::error::Error for StlConversionError {}

impl From<String> for StlConversionError {
    fn from(value: String) -> Self {
        Self(value)
    }
}

/// Returns a conservative positional-error bound for binary STL coordinates.
///
/// Binary STL stores each coordinate as an IEEE-754 `f32`. For a normally
/// sized finite value, round-to-nearest introduces at most half a relative
/// `f32` epsilon per coordinate. Multiplying the largest absolute coordinate
/// by that bound and by `sqrt(3)` covers simultaneous x/y/z rounding. The
/// model diagonal is also considered so origin-centered geometry receives a
/// useful scale-aware floor. This is deliberately a source-encoding bound,
/// not a general modeling tolerance.
pub fn binary_stl_coordinate_precision_tolerance(mesh: &Mesh) -> f64 {
    let maximum_absolute_coordinate = mesh
        .vertices
        .iter()
        .flat_map(|point| [point.x.abs(), point.y.abs(), point.z.abs()])
        .fold(0.0_f64, f64::max);
    let coordinate_scale = maximum_absolute_coordinate.max(mesh_diagonal(mesh));
    coordinate_scale * (f32::EPSILON as f64) * 0.5 * 3.0_f64.sqrt()
}

/// Recognize a parsed STL mesh and serialize a validated AP242 STEP document.
///
/// The triangle soup is SPLIT FIRST. Every vertex-connected component of the
/// mesh is a body: the whole chain below — segmentation, recognition, the
/// analytic and hybrid rebuilds, faceted repair and the coplanar merge — runs
/// on one component at a time, and the resulting solids are written into ONE
/// STEP document. That is what lets a multi-body STL, OBJ or 3MF import as N
/// solids: the shell validator sums V-E+F over a solid against 2, so three
/// disjoint closed shells in one solid read 6 and are refused.
///
/// A mesh that holds ONE component — the overwhelming case, and every
/// single-body fixture — takes the caller's own buffers through untouched, so
/// its document is byte-for-byte what this converter has always produced.
///
/// `source_positions` and `source_indices` must describe the same triangles
/// represented by `mesh`. They are kept separately because an STL reader may
/// preserve the original triangle soup while welding a second copy for robust
/// recognition. The original buffers are always used by faceted repair.
pub fn convert_stl_mesh_to_step(
    mesh: &Mesh,
    source_positions: &[f64],
    source_indices: Option<&[u32]>,
    options: &StlConversionOptions,
    part_name: &str,
    unit: &str,
    timestamp: &str,
) -> Result<StlConversionOutput, StlConversionError> {
    let total_started = Instant::now();
    validate_source_buffers(mesh, source_positions, source_indices)?;
    if !options.coordinate_precision_tolerance.is_finite()
        || options.coordinate_precision_tolerance < 0.0
    {
        return Err(StlConversionError(
            "coordinate_precision_tolerance must be finite and non-negative".to_owned(),
        ));
    }

    let components = triangle_components(mesh);
    let count = components.len();
    if count == 0 {
        return Err(StlConversionError(
            "the mesh contains no triangles to reconstruct".to_owned(),
        ));
    }

    let mut solids = Vec::with_capacity(count);
    let mut details = Vec::with_capacity(count);
    let mut first_refusal = None;
    for (index, triangles) in components.iter().enumerate() {
        if count == 1 {
            let built = convert_component(mesh, source_positions, source_indices, options)?;
            solids.push(built.0);
            details.push(built.1);
            continue;
        }
        let part = component_mesh(mesh, source_positions, source_indices, triangles);
        match convert_component(
            &part.mesh,
            &part.positions,
            part.indices.as_deref(),
            options,
        ) {
            Ok(built) => {
                solids.push(built.0);
                details.push(built.1);
            }
            // One body that no lane can build — faceted repair included —
            // must not sink the others: it is left out of the document and
            // its record says why, by body and first source triangle.
            Err(error) => {
                first_refusal.get_or_insert_with(|| {
                    format!(
                        "body {} of {count} (from source triangle {}, {} triangle(s)): {error}",
                        index + 1,
                        triangles[0],
                        triangles.len()
                    )
                });
                details.push(refused_component(&part.mesh, triangles.len(), error.0));
            }
        }
    }
    // Nothing survived: the first body's refusal is the import's.
    if solids.is_empty() {
        let refusal = first_refusal.unwrap_or_default();
        return Err(StlConversionError(if count > 1 {
            format!("{refusal} (every one of the {count} bodies was refused)")
        } else {
            refusal
        }));
    }

    let export_started = Instant::now();
    let step_text = export_step(&solids, part_name, unit, timestamp)
        .map_err(|error| StlConversionError(format!("STEP export failed: {error}")))?;
    let step_export_seconds = export_started.elapsed().as_secs_f64();

    let validation_started = Instant::now();
    let manifold_audit_issues = audit_step_manifold(&step_text);
    if !manifold_audit_issues.is_empty() {
        return Err(StlConversionError(format!(
            "STEP manifold audit failed: {}",
            manifold_audit_issues.join("; ")
        )));
    }
    let imported = import_step(&step_text).map_err(|error| {
        StlConversionError(format!("STEP round-trip validation failed: {error}"))
    })?;
    if let Some(issues) = imported
        .iter()
        .map(|solid| solid.validate())
        .find(|issues| !issues.is_empty())
    {
        return Err(StlConversionError(format!(
            "round-tripped STEP has invalid topology: {issues:?}"
        )));
    }
    if imported.len() != solids.len() {
        return Err(StlConversionError(format!(
            "{} body/bodies were written but {} came back out of the STEP document",
            solids.len(),
            imported.len()
        )));
    }
    let step_validation_seconds = validation_started.elapsed().as_secs_f64();

    let mut report = aggregate_report(&components, details, count, options);
    report.messages.push(format!(
        "STEP manifold audit and round-trip import passed for {} solid(s)",
        imported.len()
    ));
    report.messages.push(report.backend_reason.clone());
    report.exported_step_bytes = step_text.len();
    report.exported_advanced_faces = step_text.matches("ADVANCED_FACE(").count();
    report.roundtrip_solids = imported.len();
    report.manifold_audit_issues = manifold_audit_issues;
    report.timings.step_export_seconds = step_export_seconds;
    report.timings.step_validation_seconds = step_validation_seconds;
    report.timings.total_seconds = total_started.elapsed().as_secs_f64();
    Ok(StlConversionOutput { step_text, report })
}

/// The record of a body every lane refused: its source counts, the refusal,
/// and nothing else — it contributes no face, region or timing, and no solid.
fn refused_component(mesh: &Mesh, triangles: usize, reason: String) -> ComponentDetail {
    ComponentDetail {
        backend: ConversionBackend::Refused,
        backend_reason: format!("refused, not in the model: {reason}"),
        fallback_cause: None,
        input_vertices: mesh.vertices.len(),
        input_triangles: triangles,
        recognized_regions: 0,
        recognized_triangles: 0,
        unresolved_triangles: 0,
        recognition_skipped: None,
        source_closed_manifold: closed_manifold(mesh),
        analytic_used_repaired_mesh: false,
        requested_distance_tolerance: 0.0,
        effective_distance_tolerance: 0.0,
        region_type_counts: BTreeMap::new(),
        regions: Vec::new(),
        topology_refit: None,
        hybrid_rebuild: None,
        faceted_faces_before_merge: None,
        faceted_faces_after_merge: None,
        faces: 0,
        segmentation_seconds: 0.0,
        recognition_seconds: 0.0,
        topology_build_seconds: 0.0,
        messages: vec![format!("refused, not in the model: {reason}")],
        refusal: Some(reason),
    }
}

/// Everything one component contributes to the document's report, plus the
/// per-body record the report publishes.
struct ComponentDetail {
    backend: ConversionBackend,
    backend_reason: String,
    fallback_cause: Option<String>,
    input_vertices: usize,
    input_triangles: usize,
    recognized_regions: usize,
    recognized_triangles: usize,
    unresolved_triangles: usize,
    recognition_skipped: Option<String>,
    source_closed_manifold: bool,
    analytic_used_repaired_mesh: bool,
    requested_distance_tolerance: f64,
    effective_distance_tolerance: f64,
    region_type_counts: BTreeMap<String, usize>,
    regions: Vec<ConversionRegionReport>,
    topology_refit: Option<ConversionRegionReport>,
    hybrid_rebuild: Option<HybridConversionReport>,
    faceted_faces_before_merge: Option<usize>,
    faceted_faces_after_merge: Option<usize>,
    faces: usize,
    segmentation_seconds: f64,
    recognition_seconds: f64,
    topology_build_seconds: f64,
    messages: Vec<String>,
    /// Why every lane refused this body; set only by [`refused_component`].
    refusal: Option<String>,
}

/// ONE body: the complete mesh-to-BREP chain over a single closed shell,
/// stopping at the validated solid. Export, the manifold audit and the STEP
/// round trip belong to the document and run once over every body.
fn convert_component(
    mesh: &Mesh,
    source_positions: &[f64],
    source_indices: Option<&[u32]>,
    options: &StlConversionOptions,
) -> Result<(brep_kernel::BrepSolid, ComponentDetail), StlConversionError> {
    let mut recognition_options = options.recognition.clone();
    recognition_options.collect_phase_timings = true;
    let requested_distance_tolerance = recognition_options.distance_tolerance;
    recognition_options.distance_tolerance = recognition_options
        .distance_tolerance
        .max(options.coordinate_precision_tolerance);
    let encoding_distance_tolerance = recognition_options.distance_tolerance;

    let source_closed_manifold = closed_manifold(mesh);
    let input_vertices = mesh.vertices.len();
    let input_triangles = mesh.triangles.len();
    // A mesh file is routinely a few triangles short of a closed
    // two-manifold — one dropped facet, a duplicated sheet, a degenerate
    // sliver — and EVERY analytic lane below requires one. The faceted
    // fallback already repairs exactly that and builds a valid closed solid
    // from the result, so the repaired mesh is offered to the analytic lanes
    // first instead of letting a handful of stray triangles cost the body
    // every cylinder, cone and sphere the segmentation can see. A closed
    // source is never repaired, so its conversion is unchanged.
    let repaired = if source_closed_manifold {
        None
    } else {
        repaired_closed_mesh(source_positions, source_indices, options)
    };
    let (mesh, source_positions, source_indices) = match &repaired {
        Some(repaired) => (
            &repaired.mesh,
            repaired.positions.as_slice(),
            Some(repaired.indices.as_slice()),
        ),
        None => (mesh, source_positions, source_indices),
    };
    // What the lanes below may assume, as opposed to what the FILE was;
    // `source_closed_manifold` stays the report's record of the input.
    let closed_for_analytics = source_closed_manifold || repaired.is_some();

    let segmentation_started = Instant::now();
    let segmentation = if closed_for_analytics {
        gate_segmentation(source_positions, source_indices, options)
    } else {
        Err("the source mesh is not a closed two-manifold".to_owned())
    };
    let segmentation_seconds = segmentation_started.elapsed().as_secs_f64();
    let recognition_gate = recognition_gate(&segmentation);
    // The encoding floor above bounds only what storing a coordinate cost.
    // An exporter can snap vertices far more coarsely than that — OpenSCAD's
    // spheres, cylinders AND flat caps sit ~6e-5 off their carriers where
    // f32 accounts for ~4e-6 — and RANSAC held below the data's own noise
    // shreds one sphere into slivers, costs minutes doing it, and never proves
    // the coverage its lanes need. When the gate passes, the segmentation has
    // already fitted every region and measured how far every VERTEX sits from
    // its carrier, so recognition is not asked to be tighter than that.
    let measured_region_deviation = match (&recognition_gate, &segmentation) {
        (Ok(()), Ok(segmentation)) => segmentation
            .regions
            .iter()
            .map(|region| region.max_deviation)
            .fold(0.0_f64, f64::max),
        _ => 0.0,
    };
    recognition_options.distance_tolerance = recognition_options
        .distance_tolerance
        .max(measured_region_deviation);
    let effective_distance_tolerance = recognition_options.distance_tolerance;

    let recognition_started = Instant::now();
    let (recognition, completed_plane_regions) = if recognition_gate.is_ok() {
        let mut recognition = recognize_surfaces_with_unresolved(mesh, &recognition_options)
            .map_err(|error| {
                StlConversionError(format!("surface recognition failed: {error}"))
            })?;
        let completed_plane_regions = complete_small_planar_regions(
            mesh,
            &recognition_options,
            &mut recognition.regions,
            &mut recognition.unresolved_triangles,
        )?;
        (recognition, completed_plane_regions)
    } else {
        (
            crate::RecognitionResult {
                regions: Vec::new(),
                unresolved_triangles: Vec::new(),
                unresolved_diagnostics: Vec::new(),
            },
            0,
        )
    };
    let recognition_seconds = if recognition_gate.is_ok() {
        recognition_started.elapsed().as_secs_f64()
    } else {
        0.0
    };
    let recognized_triangles = recognition
        .regions
        .iter()
        .map(|region| region.triangle_indices.len())
        .sum();
    let regions = recognition
        .regions
        .iter()
        .map(region_report)
        .collect::<Vec<_>>();
    let mut region_type_counts = BTreeMap::new();
    for region in &recognition.regions {
        *region_type_counts
            .entry(region.surface.surface_type().name().to_owned())
            .or_insert(0) += 1;
    }

    let topology_started = Instant::now();
    let analytic_coverage = recognition.unresolved_triangles.is_empty()
        && recognized_triangles == mesh.triangles.len()
        && disjoint_complete_partition(&recognition.regions, mesh.triangles.len());
    let primitive = if analytic_coverage && closed_for_analytics {
        direct_analytic_solid(mesh, &recognition.regions, &recognition_options)
    } else {
        None
    };

    let mut fallback_cause = None;
    let mut messages = vec![match &recognition_gate {
        Ok(()) => format!(
            "RANSAC recognized {} region(s) covering {recognized_triangles}/{} triangles",
            recognition.regions.len(),
            mesh.triangles.len()
        ),
        Err(reason) => format!("RANSAC recognition skipped: {reason}"),
    }];
    if let Some(repaired) = &repaired {
        messages.push(repaired.message.clone());
    }
    if encoding_distance_tolerance > requested_distance_tolerance {
        messages.push(format!(
            "source coordinate precision raised the absolute recognition distance from {requested_distance_tolerance:.6e} to {encoding_distance_tolerance:.6e}"
        ));
    }
    if effective_distance_tolerance > encoding_distance_tolerance {
        messages.push(format!(
            "the segmentation's measured vertex deviation raised the absolute recognition distance from {encoding_distance_tolerance:.6e} to {effective_distance_tolerance:.6e}"
        ));
    }
    if completed_plane_regions > 0 {
        messages.push(format!(
            "a plane-only completion pass recovered {completed_plane_regions} small feature-bounded planar region(s)"
        ));
    }

    let (
        solid,
        backend,
        backend_reason,
        topology_refit,
        hybrid_rebuild,
        faceted_faces_before_merge,
        faceted_faces_after_merge,
    ) = if let Some(primitive) = primitive {
        (
            primitive.solid?,
            primitive.backend,
            primitive.reason,
            primitive.topology_refit,
            None,
            None,
            None,
        )
    } else {
        let direct_cause = match &recognition_gate {
            Err(reason) => format!("whole-primitive recognition skipped: {reason}"),
            Ok(()) if !analytic_coverage => format!(
                "RANSAC did not prove complete analytic coverage ({} unresolved triangle(s))",
                recognition.unresolved_triangles.len()
            ),
            Ok(()) if !closed_for_analytics => {
                "recognized mesh is not a closed two-manifold".to_owned()
            }
            Ok(()) => {
                "recognized regions do not match a safe complete primitive topology".to_owned()
            }
        };

        let kernel_eligible = segmentation.as_ref().is_ok_and(|segmentation| {
            segmentation_is_complete(segmentation)
                && segmentation.regions.iter().all(|region| {
                    matches!(
                        region.carrier,
                        RegionCarrier::Plane { .. }
                            | RegionCarrier::Cylinder { .. }
                            | RegionCarrier::Cone { .. }
                    )
                })
        });
        let kernel_attempt = if closed_for_analytics
            && kernel_eligible
            && options.try_kernel_analytic_rebuild
        {
            try_kernel_analytic(source_positions, source_indices, options)
        } else {
            Err("kernel analytic rebuild requires the kernel segmentation to prove complete plane/cylinder/cone coverage".to_owned())
        };

        let analytic_attempt = match kernel_attempt {
            Ok(solid) => Ok((
                solid,
                ConversionBackend::KernelAnalyticRebuild,
                "kernel segmentation proved complete plane/cylinder/cone coverage; the kernel region rebuilder then reconstructed and validated the complete analytic shell".to_owned(),
                None,
            )),
            Err(kernel_error) => {
                let hybrid_attempt = if closed_for_analytics && options.try_hybrid_rebuild {
                    try_hybrid_analytic(source_positions, source_indices, options)
                } else if !options.try_hybrid_rebuild {
                    Err("mixed analytic/faceted rebuild was disabled".to_owned())
                } else {
                    Err("mixed analytic/faceted rebuild requires a closed two-manifold".to_owned())
                };
                match hybrid_attempt {
                    Ok(output) => {
                        let hybrid_report = hybrid_report(output.stats);
                        if options.policy == ConversionPolicy::RequireFullyAnalytic
                            && hybrid_report.faceted_faces != 0
                        {
                            Err(format!(
                                "{kernel_error}; mixed rebuild retained {} faceted face(s) for {} source triangle(s)",
                                hybrid_report.faceted_faces, hybrid_report.faceted_triangles
                            ))
                        } else {
                            let reason = if hybrid_report.faceted_faces == 0 {
                                "independent segmentation reconstructed and validated a fully analytic plane/cylinder/cone/sphere shell"
                            } else {
                                "independent segmentation reconstructed exact plane/cylinder/cone/sphere regions and retained unsupported or unsafe regions as facets"
                            };
                            Ok((
                                output.solid,
                                ConversionBackend::HybridAnalyticRebuild,
                                reason.to_owned(),
                                Some(hybrid_report),
                            ))
                        }
                    }
                    Err(hybrid_error) => Err(format!("{kernel_error}; {hybrid_error}")),
                }
            }
        };

        match analytic_attempt {
            Ok((solid, backend, reason, hybrid)) => {
                (solid, backend, reason, None, hybrid, None, None)
            }
            Err(analytic_error) => {
                let cause = format!("{direct_cause}; {analytic_error}");
                if options.policy == ConversionPolicy::RequireFullyAnalytic {
                    return Err(StlConversionError(format!(
                        "strict analytic conversion refused faceted fallback: {cause}"
                    )));
                }
                fallback_cause = Some(cause.clone());
                messages.push(format!("analytic rebuild unavailable: {cause}"));
                let faceted =
                    mesh_to_faceted_brep(source_positions, source_indices, options.weld_tolerance)
                        .map_err(|error| {
                            StlConversionError(format!("faceted mesh repair failed: {error}"))
                        })?;
                let faces_before = solid_face_count(&faceted);
                let merge_tolerance = coplanar_merge_tolerance(mesh);
                match merge_same_surface_faces(&faceted, merge_tolerance) {
                    Ok(merged) => {
                        let faces_after = solid_face_count(&merged);
                        if faces_after < faces_before {
                            messages.push(format!(
                                "coplanar faceted merge reduced faces from {faces_before} to {faces_after} (tolerance {merge_tolerance:.3e})"
                            ));
                            (
                                merged,
                                ConversionBackend::FacetedRepairCoplanarMerged,
                                "the complete source triangle set was repaired as a faceted BREP, then adjacent coplanar facets were merged without introducing curved replacement topology".to_owned(),
                                None,
                                None,
                                Some(faces_before),
                                Some(faces_after),
                            )
                        } else {
                            messages.push(format!(
                                "coplanar faceted merge found no mergeable faces (tolerance {merge_tolerance:.3e})"
                            ));
                            (
                                faceted,
                                ConversionBackend::FacetedRepair,
                                "the complete source triangle set was repaired and preserved as a faceted BREP; no adjacent coplanar facets could be merged".to_owned(),
                                None,
                                None,
                                Some(faces_before),
                                Some(faces_before),
                            )
                        }
                    }
                    Err(error) => {
                        messages.push(format!(
                            "coplanar faceted merge was unavailable; retained the validated repaired mesh: {error}"
                        ));
                        (
                            faceted,
                            ConversionBackend::FacetedRepair,
                            "the complete source triangle set was repaired and preserved as a faceted BREP; the optional coplanar merge was unavailable".to_owned(),
                            None,
                            None,
                            Some(faces_before),
                            Some(faces_before),
                        )
                    }
                }
            }
        }
    };
    let topology_build_seconds = topology_started.elapsed().as_secs_f64();

    let topology_issues = solid.validate();
    if !topology_issues.is_empty() {
        return Err(StlConversionError(format!(
            "constructed BREP failed topology validation: {topology_issues:?}"
        )));
    }

    let faces = solid_face_count(&solid);
    let detail = ComponentDetail {
        backend,
        backend_reason,
        fallback_cause,
        input_vertices,
        input_triangles,
        recognized_regions: recognition.regions.len(),
        recognized_triangles,
        unresolved_triangles: recognition.unresolved_triangles.len(),
        recognition_skipped: recognition_gate.err(),
        source_closed_manifold,
        analytic_used_repaired_mesh: repaired.is_some(),
        requested_distance_tolerance,
        effective_distance_tolerance,
        region_type_counts,
        regions,
        topology_refit,
        hybrid_rebuild,
        faceted_faces_before_merge,
        faceted_faces_after_merge,
        faces,
        segmentation_seconds,
        recognition_seconds,
        topology_build_seconds,
        messages,
        refusal: None,
    };
    Ok((solid, detail))
}

/// Fold every body's detail into the document's single report. Counts are
/// TOTALS and `regions` is the concatenation; the fields that can only
/// describe one body (`backend`, `topology_refit`) are the first body's, and
/// `components` is the authoritative per-body record.
fn aggregate_report(
    components: &[Vec<usize>],
    details: Vec<ComponentDetail>,
    count: usize,
    options: &StlConversionOptions,
) -> StlConversionReport {
    let mut messages = Vec::new();
    if count > 1 {
        messages.push(format!(
            "the triangle soup holds {count} vertex-connected components; each was reconstructed as its own body"
        ));
    }
    let refused = details
        .iter()
        .enumerate()
        .filter_map(|(index, detail)| detail.refusal.as_ref().map(|reason| (index, reason)))
        .collect::<Vec<_>>();
    if !refused.is_empty() {
        messages.push(format!(
            "{} of {count} bodies could not be reconstructed and are NOT in the model: {}",
            refused.len(),
            refused
                .iter()
                .map(|(index, reason)| format!(
                    "body {} (from source triangle {}): {reason}",
                    index + 1,
                    components[*index][0]
                ))
                .collect::<Vec<_>>()
                .join("; ")
        ));
    }
    for (index, detail) in details.iter().enumerate() {
        for message in &detail.messages {
            messages.push(if count > 1 {
                format!("body {}: {message}", index + 1)
            } else {
                message.clone()
            });
        }
    }

    // The document's backend is its first BUILT body's; the caller refuses
    // the import before this when no body was built.
    let first = details
        .iter()
        .find(|detail| detail.refusal.is_none())
        .unwrap_or(&details[0]);
    let backend_reason = if count == 1 {
        first.backend_reason.clone()
    } else if details
        .iter()
        .all(|detail| detail.backend == first.backend)
    {
        format!(
            "{count} bodies were reconstructed separately; each: {}",
            first.backend_reason
        )
    } else {
        format!(
            "{count} bodies were reconstructed separately: {}",
            details
                .iter()
                .enumerate()
                .map(|(index, detail)| format!("body {} — {}", index + 1, detail.backend_reason))
                .collect::<Vec<_>>()
                .join("; ")
        )
    };
    let fallback_cause = details
        .iter()
        .enumerate()
        .find_map(|(index, detail)| {
            detail.fallback_cause.as_ref().map(|cause| {
                if count > 1 {
                    format!("body {}: {cause}", index + 1)
                } else {
                    cause.clone()
                }
            })
        });
    let recognition_skipped = details
        .iter()
        .enumerate()
        .map(|(index, detail)| {
            detail
                .recognition_skipped
                .as_ref()
                .map(|reason| (index, reason))
        })
        .collect::<Option<Vec<_>>>()
        .map(|skipped| {
            if count == 1 || skipped.iter().all(|(_, reason)| *reason == skipped[0].1) {
                skipped[0].1.clone()
            } else {
                skipped
                    .iter()
                    .map(|(index, reason)| format!("body {}: {reason}", index + 1))
                    .collect::<Vec<_>>()
                    .join("; ")
            }
        });

    let mut region_type_counts = BTreeMap::new();
    for detail in &details {
        for (name, count) in &detail.region_type_counts {
            *region_type_counts.entry(name.clone()).or_insert(0) += count;
        }
    }
    let hybrid_rebuild = details
        .iter()
        .any(|detail| detail.hybrid_rebuild.is_some())
        .then(|| {
            details
                .iter()
                .filter_map(|detail| detail.hybrid_rebuild)
                .fold(HybridConversionReport::default(), add_hybrid_reports)
        });
    let faceted_faces_before_merge = sum_optional(&details, |detail| detail.faceted_faces_before_merge);
    let faceted_faces_after_merge = sum_optional(&details, |detail| detail.faceted_faces_after_merge);

    let component_reports = components
        .iter()
        .zip(&details)
        .enumerate()
        .map(|(index, (triangles, detail))| ComponentConversionReport {
            index,
            first_triangle: triangles[0],
            input_vertices: detail.input_vertices,
            input_triangles: detail.input_triangles,
            backend: detail.backend,
            backend_reason: detail.backend_reason.clone(),
            fallback_cause: detail.fallback_cause.clone(),
            recognition_skipped: detail.recognition_skipped.clone(),
            recognized_regions: detail.recognized_regions,
            recognized_triangles: detail.recognized_triangles,
            unresolved_triangles: detail.unresolved_triangles,
            source_closed_manifold: detail.source_closed_manifold,
            analytic_used_repaired_mesh: detail.analytic_used_repaired_mesh,
            hybrid_rebuild: detail.hybrid_rebuild,
            faceted_faces_before_merge: detail.faceted_faces_before_merge,
            faceted_faces_after_merge: detail.faceted_faces_after_merge,
            faces: detail.faces,
            refusal: detail.refusal.clone(),
            source_name: None,
        })
        .collect();

    StlConversionReport {
        backend: first.backend,
        backend_reason,
        fallback_cause,
        input_vertices: details.iter().map(|detail| detail.input_vertices).sum(),
        input_triangles: details.iter().map(|detail| detail.input_triangles).sum(),
        recognized_regions: details.iter().map(|detail| detail.recognized_regions).sum(),
        recognized_triangles: details
            .iter()
            .map(|detail| detail.recognized_triangles)
            .sum(),
        unresolved_triangles: details
            .iter()
            .map(|detail| detail.unresolved_triangles)
            .sum(),
        recognition_skipped,
        source_closed_manifold: details.iter().all(|detail| detail.source_closed_manifold),
        analytic_used_repaired_mesh: details
            .iter()
            .any(|detail| detail.analytic_used_repaired_mesh),
        requested_distance_tolerance: first.requested_distance_tolerance,
        coordinate_precision_tolerance: options.coordinate_precision_tolerance,
        effective_distance_tolerance: first.effective_distance_tolerance,
        region_type_counts,
        regions: details
            .iter()
            .flat_map(|detail| detail.regions.iter().cloned())
            .collect(),
        topology_refit: details
            .iter()
            .find_map(|detail| detail.topology_refit.clone()),
        hybrid_rebuild,
        components: component_reports,
        exported_step_bytes: 0,
        exported_advanced_faces: 0,
        faceted_faces_before_merge,
        faceted_faces_after_merge,
        roundtrip_solids: 0,
        manifold_audit_issues: Vec::new(),
        timings: ConversionTimings {
            segmentation_seconds: details
                .iter()
                .map(|detail| detail.segmentation_seconds)
                .sum(),
            recognition_seconds: details
                .iter()
                .map(|detail| detail.recognition_seconds)
                .sum(),
            topology_build_seconds: details
                .iter()
                .map(|detail| detail.topology_build_seconds)
                .sum(),
            step_export_seconds: 0.0,
            step_validation_seconds: 0.0,
            total_seconds: 0.0,
        },
        messages,
    }
}

fn sum_optional(
    details: &[ComponentDetail],
    read: impl Fn(&ComponentDetail) -> Option<usize>,
) -> Option<usize> {
    details
        .iter()
        .any(|detail| read(detail).is_some())
        .then(|| details.iter().filter_map(read).sum())
}

fn add_hybrid_reports(
    total: HybridConversionReport,
    body: HybridConversionReport,
) -> HybridConversionReport {
    HybridConversionReport {
        total_faces: total.total_faces + body.total_faces,
        analytic_plane_faces: total.analytic_plane_faces + body.analytic_plane_faces,
        analytic_plane_triangles: total.analytic_plane_triangles + body.analytic_plane_triangles,
        analytic_cylinder_faces: total.analytic_cylinder_faces + body.analytic_cylinder_faces,
        analytic_cylinder_triangles: total.analytic_cylinder_triangles
            + body.analytic_cylinder_triangles,
        analytic_cone_faces: total.analytic_cone_faces + body.analytic_cone_faces,
        analytic_cone_triangles: total.analytic_cone_triangles + body.analytic_cone_triangles,
        analytic_sphere_faces: total.analytic_sphere_faces + body.analytic_sphere_faces,
        analytic_sphere_triangles: total.analytic_sphere_triangles + body.analytic_sphere_triangles,
        analytic_torus_faces: total.analytic_torus_faces + body.analytic_torus_faces,
        analytic_torus_triangles: total.analytic_torus_triangles + body.analytic_torus_triangles,
        faceted_faces: total.faceted_faces + body.faceted_faces,
        faceted_triangles: total.faceted_triangles + body.faceted_triangles,
        demoted_regions: total.demoted_regions + body.demoted_regions,
        demoted_triangles: total.demoted_triangles + body.demoted_triangles,
        segmentation_plane_regions: total.segmentation_plane_regions
            + body.segmentation_plane_regions,
        segmentation_cylinder_regions: total.segmentation_cylinder_regions
            + body.segmentation_cylinder_regions,
        segmentation_cone_regions: total.segmentation_cone_regions + body.segmentation_cone_regions,
        segmentation_sphere_regions: total.segmentation_sphere_regions
            + body.segmentation_sphere_regions,
        segmentation_torus_regions: total.segmentation_torus_regions
            + body.segmentation_torus_regions,
        segmentation_unsupported_regions: total.segmentation_unsupported_regions
            + body.segmentation_unsupported_regions,
    }
}

/// The triangle indices of each vertex-connected component of `mesh`, each
/// component's own triangles in source order and the components themselves
/// ordered by the lowest source triangle they hold — so the bodies of one file
/// always come out in the same order, and so do their names.
///
/// Vertices are welded by EXACT position first, the canonical-bits weld the
/// STL reader's zero-tolerance path uses (`-0.0` and `0.0` are one point), so
/// two triangles that meet at the same coordinate join even when the file gave
/// that point two indices — a 3MF build gives every instance its own vertex
/// range, and an OBJ may repeat a corner per object.
///
/// Connectivity is by shared VERTEX, not by shared edge. Two closed shells
/// that touch at a single point are therefore ONE component, and the shell
/// validator refuses them exactly as it does today.
fn triangle_components(mesh: &Mesh) -> Vec<Vec<usize>> {
    fn find(parent: &mut [usize], mut node: usize) -> usize {
        while parent[node] != node {
            parent[node] = parent[parent[node]];
            node = parent[node];
        }
        node
    }

    let bits = |value: f64| if value == 0.0 { 0_u64 } else { value.to_bits() };
    let mut position_ids = HashMap::<[u64; 3], usize>::new();
    let mut vertex_position = Vec::with_capacity(mesh.vertices.len());
    for point in &mesh.vertices {
        let key = [bits(point.x), bits(point.y), bits(point.z)];
        let next = position_ids.len();
        vertex_position.push(*position_ids.entry(key).or_insert(next));
    }

    let mut parent = (0..position_ids.len()).collect::<Vec<_>>();
    for triangle in &mesh.triangles {
        let first = vertex_position[triangle[0] as usize];
        for &corner in &triangle[1..] {
            let (left, right) = (
                find(&mut parent, first),
                find(&mut parent, vertex_position[corner as usize]),
            );
            if left != right {
                parent[left] = right;
            }
        }
    }

    let mut slots = HashMap::<usize, usize>::new();
    let mut components = Vec::<Vec<usize>>::new();
    for (index, triangle) in mesh.triangles.iter().enumerate() {
        let root = find(&mut parent, vertex_position[triangle[0] as usize]);
        let slot = match slots.get(&root) {
            Some(&slot) => slot,
            None => {
                components.push(Vec::new());
                slots.insert(root, components.len() - 1);
                components.len() - 1
            }
        };
        components[slot].push(index);
    }
    components
}

/// One component's buffers, rebased onto its own vertices.
struct ComponentMesh {
    mesh: Mesh,
    positions: Vec<f64>,
    indices: Option<Vec<u32>>,
}

/// Cut `triangles` out of the whole input as a standalone mesh and matching
/// source buffers. The recognition mesh and the source buffers describe the
/// same triangles in the same order (`validate_source_buffers` proves it), so
/// one triangle-index list rebases both. An unindexed source stays a soup.
fn component_mesh(
    mesh: &Mesh,
    positions: &[f64],
    indices: Option<&[u32]>,
    triangles: &[usize],
) -> ComponentMesh {
    let mut vertex_map = HashMap::<u32, u32>::new();
    let mut vertices = Vec::new();
    let mut component_triangles = Vec::with_capacity(triangles.len());
    for &triangle in triangles {
        let mut corners = [0_u32; 3];
        for (corner, &index) in mesh.triangles[triangle].iter().enumerate() {
            corners[corner] = *vertex_map.entry(index).or_insert_with(|| {
                vertices.push(mesh.vertices[index as usize]);
                vertices.len() as u32 - 1
            });
        }
        component_triangles.push(corners);
    }
    let mut component = Mesh::new(vertices, component_triangles);
    if let Some(normals) = &mesh.vertex_normals {
        if normals.len() == mesh.vertices.len() {
            let mut component_normals = vec![Vec3::default(); component.vertices.len()];
            for (&source, &target) in &vertex_map {
                component_normals[target as usize] = normals[source as usize];
            }
            component.vertex_normals = Some(component_normals);
        }
    }
    if !mesh.source_metadata.is_empty() {
        let local = triangles
            .iter()
            .enumerate()
            .map(|(local, &source)| (source, local))
            .collect::<HashMap<_, _>>();
        component.source_metadata = mesh
            .source_metadata
            .iter()
            .filter_map(|metadata| {
                let triangle_indices = metadata
                    .triangle_indices
                    .iter()
                    .filter_map(|source| local.get(source).copied())
                    .collect::<Vec<_>>();
                (!triangle_indices.is_empty()).then(|| crate::SourceMetadata {
                    triangle_indices,
                    ..metadata.clone()
                })
            })
            .collect();
    }

    let (component_positions, component_indices) = match indices {
        Some(indices) => {
            let mut source_map = HashMap::<u32, u32>::new();
            let mut component_positions = Vec::new();
            let mut component_indices = Vec::with_capacity(triangles.len() * 3);
            for &triangle in triangles {
                for &index in &indices[triangle * 3..triangle * 3 + 3] {
                    let mapped = *source_map.entry(index).or_insert_with(|| {
                        let base = index as usize * 3;
                        component_positions.extend_from_slice(&positions[base..base + 3]);
                        component_positions.len() as u32 / 3 - 1
                    });
                    component_indices.push(mapped);
                }
            }
            (component_positions, Some(component_indices))
        }
        None => {
            let mut component_positions = Vec::with_capacity(triangles.len() * 9);
            for &triangle in triangles {
                component_positions.extend_from_slice(&positions[triangle * 9..triangle * 9 + 9]);
            }
            (component_positions, None)
        }
    };
    ComponentMesh {
        mesh: component,
        positions: component_positions,
        indices: component_indices,
    }
}

fn validate_source_buffers(
    mesh: &Mesh,
    positions: &[f64],
    indices: Option<&[u32]>,
) -> Result<(), StlConversionError> {
    if positions.is_empty() || !positions.len().is_multiple_of(3) {
        return Err(StlConversionError(
            "source position buffer must contain complete non-empty xyz triples".to_owned(),
        ));
    }
    if positions.iter().any(|value| !value.is_finite()) {
        return Err(StlConversionError(
            "source position buffer contains a non-finite coordinate".to_owned(),
        ));
    }
    let source_triangles = match indices {
        Some(indices) => {
            if !indices.len().is_multiple_of(3) {
                return Err(StlConversionError(
                    "source index buffer must contain complete triangles".to_owned(),
                ));
            }
            if indices
                .iter()
                .any(|&index| index as usize >= positions.len() / 3)
            {
                return Err(StlConversionError(
                    "source index buffer references a missing position".to_owned(),
                ));
            }
            indices.len() / 3
        }
        None => {
            if !positions.len().is_multiple_of(9) {
                return Err(StlConversionError(
                    "unindexed source positions must be a triangle soup".to_owned(),
                ));
            }
            positions.len() / 9
        }
    };
    if source_triangles != mesh.triangles.len() {
        return Err(StlConversionError(format!(
            "source buffers contain {source_triangles} triangles but recognition mesh contains {}",
            mesh.triangles.len()
        )));
    }
    Ok(())
}

/// Recover only small coplanar feature-bounded components that the generic
/// pass skipped because its minimum support deliberately protects curved
/// model selection. A two-triangle plane is fully determined without giving
/// tiny cylinder/torus hypotheses authority over output topology.
fn complete_small_planar_regions(
    mesh: &Mesh,
    options: &RecognitionOptions,
    regions: &mut Vec<SurfaceRegion>,
    unresolved: &mut Vec<usize>,
) -> Result<usize, StlConversionError> {
    if unresolved.len() < 2 {
        return Ok(0);
    }
    let analyzed = mesh
        .analyze(&MeshAnalysisOptions {
            feature_angle: options.feature_angle,
            ..MeshAnalysisOptions::default()
        })
        .map_err(|error| {
            StlConversionError(format!("plane completion analysis failed: {error}"))
        })?;
    let allowed = unresolved.iter().copied().collect::<BTreeSet<_>>();
    let mut visited = BTreeSet::new();
    let mut accepted = BTreeSet::new();
    let mut completed = 0;
    let mut plane_options = options.clone();
    plane_options.minimum_support = 2;

    for &seed in unresolved.iter() {
        if !visited.insert(seed) {
            continue;
        }
        let mut component = Vec::new();
        let mut queue = VecDeque::from([seed]);
        while let Some(triangle) = queue.pop_front() {
            component.push(triangle);
            let data = &analyzed.triangles[triangle];
            for edge in 0..3 {
                if data.feature_edges[edge] {
                    continue;
                }
                if let Some(neighbor) = data.neighbors[edge] {
                    if allowed.contains(&neighbor) && visited.insert(neighbor) {
                        queue.push_back(neighbor);
                    }
                }
            }
        }
        component.sort_unstable();
        if component.len() < 2 {
            continue;
        }
        let Ok(fit) = reconstruct_surface(
            mesh,
            &component,
            &SurfaceHint::KnownType {
                surface_type: SurfaceType::Plane,
            },
            &plane_options,
        ) else {
            continue;
        };
        if fit.metrics.support_triangles != component.len()
            || !matches!(fit.surface, AnalyticSurface::Plane(_))
        {
            continue;
        }
        accepted.extend(component.iter().copied());
        regions.push(SurfaceRegion {
            surface: fit.surface,
            orientation: fit.orientation,
            triangle_indices: component,
            metrics: fit.metrics,
            confidence: fit.confidence,
            diagnostics: fit.diagnostics,
        });
        completed += 1;
    }
    unresolved.retain(|triangle| !accepted.contains(triangle));
    Ok(completed)
}

fn region_report(region: &SurfaceRegion) -> ConversionRegionReport {
    fit_fields(
        region.surface,
        region.orientation,
        &region.metrics,
        region.confidence,
        &region.diagnostics,
    )
}

fn refit_report(fit: &SurfaceFitResult) -> ConversionRegionReport {
    fit_fields(
        fit.surface,
        fit.orientation,
        &fit.metrics,
        fit.confidence,
        &fit.diagnostics,
    )
}

fn fit_fields(
    surface: AnalyticSurface,
    orientation: i8,
    metrics: &crate::FitMetrics,
    confidence: f64,
    diagnostics: &crate::FitDiagnostics,
) -> ConversionRegionReport {
    ConversionRegionReport {
        surface_type: surface.surface_type(),
        surface,
        support_triangles: metrics.support_triangles,
        supported_area: metrics.supported_area,
        rms_error: metrics.rms_error,
        max_error: metrics.max_error,
        rms_normal_error_radians: metrics.rms_normal_error,
        max_normal_error_radians: metrics.max_normal_error,
        confidence,
        orientation,
        reason: diagnostics.reason.clone(),
        phase_timings: diagnostics.phase_timings,
    }
}

fn disjoint_complete_partition(regions: &[SurfaceRegion], triangle_count: usize) -> bool {
    let mut seen = BTreeSet::new();
    regions.iter().all(|region| {
        region
            .triangle_indices
            .iter()
            .all(|&triangle| triangle < triangle_count && seen.insert(triangle))
    }) && seen.len() == triangle_count
}

/// A source mesh the repair closed, with the buffers every downstream lane
/// reads bound to it.
struct RepairedMesh {
    mesh: Mesh,
    positions: Vec<f64>,
    indices: Vec<u32>,
    message: String,
}

/// Repair a mesh that is not a closed two-manifold, and hand it back only
/// when the repair actually closed it — a mesh too broken to close keeps the
/// refusal it has always had, and a file whose repair would CHANGE a closed
/// mesh never reaches here.
fn repaired_closed_mesh(
    source_positions: &[f64],
    source_indices: Option<&[u32]>,
    options: &StlConversionOptions,
) -> Option<RepairedMesh> {
    let (positions, indices, report) =
        repair_triangle_soup(source_positions, source_indices, options.weld_tolerance).ok()?;
    let vertices = positions
        .chunks_exact(3)
        .map(|point| Vec3::new(point[0], point[1], point[2]))
        .collect::<Vec<_>>();
    let triangles = indices
        .chunks_exact(3)
        .map(|triangle| [triangle[0], triangle[1], triangle[2]])
        .collect::<Vec<_>>();
    let mesh = Mesh::new(vertices, triangles);
    if !closed_manifold(&mesh) {
        return None;
    }
    let message = format!(
        "the source mesh is not a closed two-manifold; the analytic lanes ran on the repaired mesh \
         ({} welded vertices, {} triangles: {} dropped as degenerate or duplicate, {} surplus \
         sheet(s) pruned, {} reoriented to agree with their neighbours, {} boundary loop(s) \
         capped with {} triangle(s))",
        report.welded_vertices,
        report.triangles,
        report.dropped_triangles,
        report.pruned_triangles,
        report.reoriented_triangles,
        report.capped_loops,
        report.cap_triangles,
    );
    Some(RepairedMesh {
        mesh,
        positions,
        indices,
        message,
    })
}

fn closed_manifold(mesh: &Mesh) -> bool {
    let mut uses = BTreeMap::<(u32, u32), (usize, i32)>::new();
    for triangle in &mesh.triangles {
        if triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0] {
            return false;
        }
        for edge in [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ] {
            let key = if edge.0 < edge.1 {
                (edge.0, edge.1)
            } else {
                (edge.1, edge.0)
            };
            let entry = uses.entry(key).or_default();
            entry.0 += 1;
            entry.1 += if edge == key { 1 } else { -1 };
        }
    }
    !uses.is_empty()
        && uses
            .values()
            .all(|&(count, sense)| count == 2 && sense == 0)
}

struct DirectAnalyticBuild {
    solid: Result<brep_kernel::BrepSolid, StlConversionError>,
    backend: ConversionBackend,
    reason: String,
    topology_refit: Option<ConversionRegionReport>,
}

fn direct_analytic_solid(
    mesh: &Mesh,
    regions: &[SurfaceRegion],
    options: &RecognitionOptions,
) -> Option<DirectAnalyticBuild> {
    if regions.len() == 1 {
        return match regions[0].surface {
            AnalyticSurface::Sphere(sphere) => Some(DirectAnalyticBuild {
                solid: make_sphere_brep(
                    kernel_vec(sphere.center),
                    sphere.radius,
                    kernel_vec(Vec3::new(0.0, 0.0, 1.0)),
                )
                .map_err(StlConversionError),
                backend: ConversionBackend::RansacSphere,
                reason: "RANSAC proved complete closed spherical coverage; exported an exact analytic sphere".to_owned(),
                topology_refit: None,
            }),
            AnalyticSurface::Torus(torus) if torus.major_radius > torus.minor_radius => Some(DirectAnalyticBuild {
                solid: make_torus_brep(
                    kernel_vec(torus.center),
                    kernel_vec(torus.axis),
                    torus.major_radius,
                    torus.minor_radius,
                )
                .map_err(StlConversionError),
                backend: ConversionBackend::RansacTorus,
                reason: "RANSAC proved complete closed ring-torus coverage; exported an exact analytic torus".to_owned(),
                topology_refit: None,
            }),
            _ => None,
        };
    }

    let curved = regions
        .iter()
        .filter(|region| !matches!(region.surface, AnalyticSurface::Plane(_)))
        .collect::<Vec<_>>();
    let planes = regions
        .iter()
        .filter_map(|region| match region.surface {
            AnalyticSurface::Plane(plane) => Some(plane),
            _ => None,
        })
        .collect::<Vec<_>>();
    if curved.len() != 1 {
        return None;
    }
    // A finite cylinder/cone strip sampled at only a handful of axial levels
    // can be represented to roundoff by the very-large-radius limit of a
    // torus. Planar cap topology removes that ambiguity. In that narrowly
    // gated case, ask the production recognizer for a topology-informed
    // known-type refit and retain all of its ordinary residual/normal gates.
    let topology_refit;
    let mut topology_refit_report = None;
    let curved = if let AnalyticSurface::Torus(torus) = curved[0].surface {
        let scale = mesh_diagonal(mesh);
        let collapsed = torus.major_radius.min(torus.minor_radius) > 20.0 * scale;
        let forced_type = match planes.len() {
            2 if collapsed => Some(SurfaceType::Cylinder),
            1 if collapsed => Some(SurfaceType::Cone),
            _ => None,
        };
        if let Some(surface_type) = forced_type {
            topology_refit = reconstruct_surface(
                mesh,
                &curved[0].triangle_indices,
                &SurfaceHint::KnownType { surface_type },
                options,
            )
            .ok()?;
            topology_refit_report = Some(refit_report(&topology_refit));
            &topology_refit.surface
        } else {
            &curved[0].surface
        }
    } else {
        &curved[0].surface
    };
    match *curved {
        AnalyticSurface::Cylinder(cylinder) if planes.len() == 2 => {
            let (low, high) = axial_range(mesh, cylinder.axis_origin, cylinder.axis)?;
            let tolerance = conversion_tolerance(mesh, options);
            if high - low <= tolerance
                || !caps_match(
                    &planes,
                    cylinder.axis_origin,
                    cylinder.axis,
                    &[low, high],
                    tolerance,
                )
            {
                return None;
            }
            let base = cylinder.axis_origin + cylinder.axis * low;
            Some(DirectAnalyticBuild {
                solid: make_cylinder_brep(
                    kernel_vec(base),
                    kernel_vec(cylinder.axis),
                    cylinder.radius,
                    high - low,
                )
                .map_err(StlConversionError),
                backend: ConversionBackend::RansacCappedCylinder,
                reason: if topology_refit_report.is_some() {
                    "two-cap topology disambiguated a large-radius torus limit; the production known-type cylinder refit passed all gates and supplied the exact capped-cylinder parameters".to_owned()
                } else {
                    "RANSAC proved one cylindrical wall plus two matching planar caps; exported an exact capped cylinder".to_owned()
                },
                topology_refit: topology_refit_report,
            })
        }
        AnalyticSurface::Cone(cone) if planes.len() == 1 || planes.len() == 2 => {
            let (low, high) = axial_range(mesh, cone.apex, cone.axis)?;
            let tolerance = conversion_tolerance(mesh, options);
            if low < -tolerance || high - low <= tolerance {
                return None;
            }
            let expected_caps = if low <= tolerance {
                vec![high]
            } else {
                vec![low, high]
            };
            if planes.len() != expected_caps.len()
                || !caps_match(&planes, cone.apex, cone.axis, &expected_caps, tolerance)
            {
                return None;
            }
            let radius_bottom = low.max(0.0) * cone.half_angle.tan();
            let radius_top = high * cone.half_angle.tan();
            if radius_top <= tolerance || (low > tolerance && radius_bottom <= tolerance) {
                return None;
            }
            let (base, build_axis, build_bottom, build_top) = if low <= tolerance {
                // The kernel's capped-cone constructor requires a positive
                // bottom radius. Build a pointed cone from its wide end back
                // toward the apex.
                (
                    cone.apex + cone.axis * high,
                    cone.axis * -1.0,
                    radius_top,
                    0.0,
                )
            } else {
                (
                    cone.apex + cone.axis * low,
                    cone.axis,
                    radius_bottom,
                    radius_top,
                )
            };
            Some(DirectAnalyticBuild {
                solid: make_cone_brep(
                    kernel_vec(base),
                    kernel_vec(build_axis),
                    build_bottom,
                    build_top,
                    high - low.max(0.0),
                )
                .map_err(StlConversionError),
                backend: ConversionBackend::RansacCappedCone,
                reason: if topology_refit_report.is_some() {
                    "cone-cap topology disambiguated a large-radius torus limit; the production known-type cone refit passed all gates and supplied the exact capped-cone parameters".to_owned()
                } else {
                    "RANSAC proved one conical wall and matching planar cap topology; exported an exact capped cone".to_owned()
                },
                topology_refit: topology_refit_report,
            })
        }
        _ => None,
    }
}

fn axial_range(mesh: &Mesh, origin: Vec3, axis: Vec3) -> Option<(f64, f64)> {
    let mut low = f64::INFINITY;
    let mut high = f64::NEG_INFINITY;
    for &point in &mesh.vertices {
        let height = (point - origin).dot(axis);
        low = low.min(height);
        high = high.max(height);
    }
    (low.is_finite() && high.is_finite()).then_some((low, high))
}

fn caps_match(
    planes: &[crate::PlaneSurface],
    origin: Vec3,
    axis: Vec3,
    expected_heights: &[f64],
    tolerance: f64,
) -> bool {
    let mut actual = Vec::with_capacity(planes.len());
    for plane in planes {
        if plane.normal.dot(axis).abs() < 1.0 - 1.0e-6 {
            return false;
        }
        actual.push((plane.origin - origin).dot(axis));
    }
    actual.sort_by(f64::total_cmp);
    let mut expected = expected_heights.to_vec();
    expected.sort_by(f64::total_cmp);
    actual.len() == expected.len()
        && actual
            .iter()
            .zip(expected)
            .all(|(actual, expected)| (*actual - expected).abs() <= tolerance)
}

fn mesh_tolerance(mesh: &Mesh) -> f64 {
    let mut low = mesh.vertices[0];
    let mut high = mesh.vertices[0];
    for &point in &mesh.vertices[1..] {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    (high - low).length().max(1.0) * 1.0e-5
}

fn conversion_tolerance(mesh: &Mesh, options: &RecognitionOptions) -> f64 {
    mesh_tolerance(mesh)
        .max(options.distance_tolerance + options.relative_tolerance * mesh_diagonal(mesh))
}

fn mesh_diagonal(mesh: &Mesh) -> f64 {
    let mut low = mesh.vertices[0];
    let mut high = mesh.vertices[0];
    for &point in &mesh.vertices[1..] {
        low.x = low.x.min(point.x);
        low.y = low.y.min(point.y);
        low.z = low.z.min(point.z);
        high.x = high.x.max(point.x);
        high.y = high.y.max(point.y);
        high.z = high.z.max(point.z);
    }
    (high - low).length()
}

fn coplanar_merge_tolerance(mesh: &Mesh) -> f64 {
    (mesh_diagonal(mesh) * 1.0e-9).max(1.0e-12)
}

fn solid_face_count(solid: &brep_kernel::BrepSolid) -> usize {
    solid.shells.iter().map(|shell| shell.faces.len()).sum()
}

/// Most analytic regions a whole primitive can have: a capped cylinder is a
/// wall and two caps; a capped cone a wall and one or two caps; a sphere or
/// ring torus is one region.
const MAX_WHOLE_PRIMITIVE_REGIONS: usize = 3;

fn segmentation_options(options: &StlConversionOptions) -> SegmentOptions {
    SegmentOptions {
        deflection_angle_deg: options.kernel_deflection_angle_degrees,
        fit_tolerance: options.kernel_fit_tolerance,
        normal_tolerance_deg: options.kernel_normal_tolerance_degrees,
        weld_tolerance: options.weld_tolerance,
        ..SegmentOptions::default()
    }
}

/// The kernel segmentation that decides whether RANSAC recognition and the
/// kernel region rebuilder can pay for themselves.
fn gate_segmentation(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &StlConversionOptions,
) -> Result<MeshSegmentation, String> {
    let owned_indices;
    let indices = match indices {
        Some(indices) => indices,
        None => {
            owned_indices = (0..positions.len() as u32 / 3).collect::<Vec<_>>();
            &owned_indices
        }
    };
    segment_mesh_faces(positions, indices, &segmentation_options(options))
        .map_err(|error| format!("kernel segmentation failed: {error}"))
}

/// Every triangle assigned, every region on an analytic carrier.
fn segmentation_is_complete(segmentation: &MeshSegmentation) -> bool {
    !segmentation
        .triangle_region_ids
        .contains(&UNASSIGNED_REGION)
        && segmentation
            .regions
            .iter()
            .all(|region| !matches!(region.carrier, RegionCarrier::Freeform))
}

/// Whether RANSAC recognition can feed a conversion lane.
///
/// Its only consumers are the exact sphere / torus / capped cylinder /
/// capped cone lanes, which need a closed mesh whose every triangle lies on
/// one of at most [`MAX_WHOLE_PRIMITIVE_REGIONS`] analytic regions. The
/// kernel segmentation decides that in a fraction of RANSAC's time, so
/// RANSAC runs only when it can pay. `Err` carries the reason it cannot.
fn recognition_gate(segmentation: &Result<MeshSegmentation, String>) -> Result<(), String> {
    let segmentation = segmentation.as_ref().map_err(Clone::clone)?;
    let unassigned = segmentation
        .triangle_region_ids
        .iter()
        .filter(|id| **id == UNASSIGNED_REGION)
        .count();
    let unsupported = segmentation
        .regions
        .iter()
        .filter(|region| matches!(region.carrier, RegionCarrier::Freeform))
        .count();
    if unassigned > 0 || unsupported > 0 {
        return Err(format!(
            "kernel segmentation left {unassigned} triangle(s) unassigned and {unsupported} region(s) without an analytic carrier, so no whole-primitive lane can fire"
        ));
    }
    if segmentation.regions.len() > MAX_WHOLE_PRIMITIVE_REGIONS {
        return Err(format!(
            "kernel segmentation found {} analytic regions; a whole primitive has at most {MAX_WHOLE_PRIMITIVE_REGIONS}",
            segmentation.regions.len()
        ));
    }
    Ok(())
}

fn try_kernel_analytic(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &StlConversionOptions,
) -> Result<brep_kernel::BrepSolid, String> {
    let owned_indices;
    let indices = match indices {
        Some(indices) => indices,
        None => {
            owned_indices = (0..positions.len() as u32 / 3).collect::<Vec<_>>();
            &owned_indices
        }
    };
    let segmentation_options = segmentation_options(options);
    let segmentation = segment_mesh_faces(positions, indices, &segmentation_options)?;
    if segmentation
        .triangle_region_ids
        .contains(&UNASSIGNED_REGION)
    {
        return Err("independent kernel segmentation left triangles unassigned".to_owned());
    }
    if let Some(region) = segmentation.regions.iter().find(|region| {
        matches!(
            region.carrier,
            RegionCarrier::Freeform | RegionCarrier::Sphere { .. } | RegionCarrier::Torus { .. }
        )
    }) {
        return Err(format!(
            "independent kernel segmentation produced unsupported {} region {}",
            region.carrier.kind(),
            region.id
        ));
    }
    mesh_regions_to_brep(positions, indices, &segmentation_options)
}

fn try_hybrid_analytic(
    positions: &[f64],
    indices: Option<&[u32]>,
    options: &StlConversionOptions,
) -> Result<crate::hybrid_region_brep::HybridBrepOutput, String> {
    crate::hybrid_region_brep::hybrid_plane_cylinder_brep(
        positions,
        indices,
        &segmentation_options(options),
    )
}

fn hybrid_report(stats: crate::hybrid_region_brep::HybridBrepStats) -> HybridConversionReport {
    HybridConversionReport {
        total_faces: stats.total_faces,
        analytic_plane_faces: stats.analytic_plane_faces,
        analytic_plane_triangles: stats.analytic_plane_triangles,
        analytic_cylinder_faces: stats.analytic_cylinder_faces,
        analytic_cylinder_triangles: stats.analytic_cylinder_triangles,
        analytic_cone_faces: stats.analytic_cone_faces,
        analytic_cone_triangles: stats.analytic_cone_triangles,
        analytic_sphere_faces: stats.analytic_sphere_faces,
        analytic_sphere_triangles: stats.analytic_sphere_triangles,
        analytic_torus_faces: stats.analytic_torus_faces,
        analytic_torus_triangles: stats.analytic_torus_triangles,
        faceted_faces: stats.faceted_faces,
        faceted_triangles: stats.faceted_triangles,
        demoted_regions: stats.demoted_regions,
        demoted_triangles: stats.demoted_triangles,
        segmentation_plane_regions: stats.segmentation_plane_regions,
        segmentation_cylinder_regions: stats.segmentation_cylinder_regions,
        segmentation_cone_regions: stats.segmentation_cone_regions,
        segmentation_sphere_regions: stats.segmentation_sphere_regions,
        segmentation_torus_regions: stats.segmentation_torus_regions,
        segmentation_unsupported_regions: stats.segmentation_unsupported_regions,
    }
}

fn kernel_vec(value: Vec3) -> brep_kernel::Vec3 {
    brep_kernel::Vec3::new(value.x, value.y, value.z)
}
