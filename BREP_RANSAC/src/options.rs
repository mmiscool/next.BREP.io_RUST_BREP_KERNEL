use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
/// Selects which geometric samples are used during fitting.
pub enum SamplingMode {
    /// Sample one point at each triangle centroid.
    TriangleCentroids,
    /// Sample mesh vertices.
    Vertices,
    /// Sample both triangle centroids and mesh vertices.
    CentroidsAndVertices,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
/// Controls surface recognition, refinement, and region discovery.
pub struct RecognitionOptions {
    /// Maximum absolute positional residual accepted as an inlier.
    pub distance_tolerance: f64,
    /// Scale-relative positional tolerance added to the absolute tolerance.
    pub relative_tolerance: f64,
    /// Maximum accepted normal-angle residual, in radians.
    pub normal_tolerance: f64,
    /// Minimum number of supporting triangles required for a fit.
    pub minimum_support: usize,
    /// Minimum total supporting triangle area required for a fit.
    pub minimum_support_area: f64,
    /// Desired probability that adaptive generic hypothesis generation has
    /// sampled a seed from the best support observed so far. This controls
    /// support visitation, not whether a boundary/degenerate seed yields a
    /// usable hypothesis and not the reported residual-evidence quality score.
    pub confidence: f64,
    /// Seed for deterministic hypothesis sampling, or `None` for entropy-based
    /// sampling.
    pub deterministic_seed: Option<u64>,
    /// Maximum number of generic recognition hypotheses to generate.
    pub max_hypotheses: usize,
    /// Maximum number of numerical refinement iterations per candidate.
    pub max_refinement_iterations: usize,
    /// Dihedral-angle threshold, in radians, used to identify feature edges.
    pub feature_angle: f64,
    /// Whether region traversal stops at detected feature edges.
    pub respect_features: bool,
    /// Whether to split the input selection into recognized surface regions.
    pub discover_regions: bool,
    /// Merge disconnected regions only when a joint fit validates that they
    /// lie on one carrier and have the same observed orientation.
    pub allow_disconnected_same_surface: bool,
    /// Collect wall-clock phase timings in fit diagnostics. Disabled by
    /// default so deterministic result comparisons do not contain clock data.
    pub collect_phase_timings: bool,
    /// Geometric sampling strategy used by fitting and validation.
    pub sampling: SamplingMode,
}

impl Default for RecognitionOptions {
    fn default() -> Self {
        Self {
            distance_tolerance: 1.0e-6,
            relative_tolerance: 1.0e-8,
            normal_tolerance: 5.0_f64.to_radians(),
            minimum_support: 6,
            minimum_support_area: 0.0,
            confidence: 0.999,
            deterministic_seed: Some(0x4341_4452_414e_5341),
            max_hypotheses: 512,
            // Small, partial torus patches have a shallow coupled
            // center/axis/radii valley.  They can require well over 40
            // monotonically improving LM steps to reach CAD-level accuracy.
            max_refinement_iterations: 160,
            feature_angle: 30.0_f64.to_radians(),
            respect_features: true,
            discover_regions: true,
            allow_disconnected_same_surface: false,
            collect_phase_timings: false,
            sampling: SamplingMode::CentroidsAndVertices,
        }
    }
}

impl RecognitionOptions {
    pub(crate) fn validate(&self) -> Result<(), crate::RecognitionError> {
        if !self.distance_tolerance.is_finite() || self.distance_tolerance <= 0.0 {
            return Err(crate::RecognitionError::InvalidOptions(
                "distance_tolerance must be finite and positive".into(),
            ));
        }
        if !self.relative_tolerance.is_finite() || self.relative_tolerance < 0.0 {
            return Err(crate::RecognitionError::InvalidOptions(
                "relative_tolerance must be finite and non-negative".into(),
            ));
        }
        if !(0.0..=std::f64::consts::PI).contains(&self.normal_tolerance) {
            return Err(crate::RecognitionError::InvalidOptions(
                "normal_tolerance must be in [0, pi]".into(),
            ));
        }
        if self.minimum_support == 0 {
            return Err(crate::RecognitionError::InvalidOptions(
                "minimum_support must be non-zero".into(),
            ));
        }
        if !self.minimum_support_area.is_finite() || self.minimum_support_area < 0.0 {
            return Err(crate::RecognitionError::InvalidOptions(
                "minimum_support_area must be finite and non-negative".into(),
            ));
        }
        if !(0.0..1.0).contains(&self.confidence) {
            return Err(crate::RecognitionError::InvalidOptions(
                "confidence must be in [0, 1)".into(),
            ));
        }
        if self.max_hypotheses == 0 || self.max_refinement_iterations == 0 {
            return Err(crate::RecognitionError::InvalidOptions(
                "iteration limits must be non-zero".into(),
            ));
        }
        if !(0.0..=std::f64::consts::PI).contains(&self.feature_angle) {
            return Err(crate::RecognitionError::InvalidOptions(
                "feature_angle must be finite and in [0, pi]".into(),
            ));
        }
        Ok(())
    }
}

