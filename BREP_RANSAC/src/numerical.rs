//! Internal numerical policy for scale-independent floating-point safeguards.
//!
//! These constants are deliberately separate from [`crate::RecognitionOptions`].
//! Options define caller-visible geometric acceptance semantics; this module
//! only controls conditioning, observability, normalization, and iterative
//! solver convergence. Keeping the two concepts separate prevents an internal
//! solver epsilon from silently becoming a reconstruction tolerance.

/// General scalar and geometric safeguards shared across modules.
pub(crate) mod scalar {
    /// Smallest norm that may safely be normalized or used as a radius.
    pub(crate) const MIN_NORMALIZABLE_NORM: f64 = 1.0e-15;
    /// Scale floor used for geometric normalization.
    pub(crate) const GEOMETRIC_SCALE_FLOOR: f64 = 1.0e-12;
    /// Positive denominator floor for normalized objectives and ratios.
    pub(crate) const POSITIVE_DENOMINATOR_FLOOR: f64 = 1.0e-30;
    /// Unit-vector validation allowance after floating-point computation.
    pub(crate) const UNIT_LENGTH_TOLERANCE: f64 = 1.0e-8;
    /// Cross-product norm below which two line directions are parallel.
    pub(crate) const PARALLEL_LINE_CROSS_NORM: f64 = 1.0e-12;
}

/// Conditioning policy for the dependency-free small-matrix solvers.
pub(crate) mod linear_algebra {
    /// Prevents a zero scale while retaining subnormal-scale matrices.
    pub(crate) const MATRIX_SCALE_FLOOR: f64 = 1.0e-300;
    /// Relative off-diagonal threshold for Jacobi convergence.
    pub(crate) const JACOBI_RELATIVE_CONVERGENCE: f64 = 1.0e-15;
    /// Relative pivot threshold used to reject singular linear systems.
    pub(crate) const LINEAR_PIVOT_RELATIVE_MIN: f64 = 1.0e-14;
    /// Machine-roundoff rank cutoff for covariance matrices.
    pub(crate) const MACHINE_RANK_RELATIVE_MIN: f64 = f64::EPSILON;
}

/// Primitive initialization, observability, and refinement policy.
pub(crate) mod fitting {
    pub(crate) const MIN_CONE_HALF_ANGLE: f64 = 1.0e-7;
    pub(crate) const ALGEBRAIC_REGULARIZATION_RELATIVE: f64 = 1.0e-15;
    pub(crate) const MIN_RADIUS_RELATIVE_TO_SCALE: f64 = 1.0e-12;
    pub(crate) const MAX_RADIUS_RELATIVE_TO_SCALE: f64 = 1.0e12;
    pub(crate) const CYLINDER_NORMAL_FAN_RANK_RELATIVE: f64 = 1.0e-12;
    /// Normal fans below this relative transverse rank are sufficiently narrow
    /// that their least-moment cylinder axis can select a remote tangent
    /// carrier.  Such patches may try one additional positional generator-axis
    /// seed, subject to the independent point-spectrum and normal-orthogonality
    /// gates below.
    pub(crate) const CYLINDER_NARROW_FAN_MULTISTART_RANK_MAX: f64 = 1.0e-10;
    /// Maximum transverse/longitudinal point-covariance ratio for treating the
    /// dominant point direction as an observed cylinder generator direction.
    pub(crate) const CYLINDER_GENERATOR_POINT_RANK_MAX: f64 = 1.0e-9;
    /// Maximum weighted RMS dot product between a positional generator-axis
    /// seed and the supplied unit normals.
    pub(crate) const CYLINDER_GENERATOR_NORMAL_ORTHOGONALITY_MAX: f64 = 1.0e-6;
    /// Relative oriented-normal weight used only to rank alternate cylinder
    /// initialization basins.  LM and final acceptance remain position-only
    /// and retain their existing public tolerances.
    pub(crate) const CYLINDER_NORMAL_OBJECTIVE_WEIGHT: f64 = 1.490_116_119_384_765_6e-8;
    /// Normal error below which the legacy cylinder seed already identifies
    /// the observed carrier gauge and no additional LM start is warranted.
    pub(crate) const CYLINDER_SEED_NORMAL_EQUIVALENCE_RADIANS: f64 = 1.28e-5;
    pub(crate) const CONE_NORMAL_FAN_RANK_RELATIVE: f64 = 1.0e-13;
    pub(crate) const NORMAL_VARIANCE_RELATIVE_MIN: f64 = 1.0e-24;
    /// Below this transverse/longitudinal point-covariance ratio, a planar
    /// trim is ill-conditioned enough that the covariance eigenvector must be
    /// checked against a coherent supplied-normal carrier.
    pub(crate) const PLANE_TRANSVERSE_COVARIANCE_RELATIVE_MAX: f64 = 1.0e-8;
    /// Minimum singular-value ratio for a cone tangent-plane fan.
    pub(crate) const TANGENT_FAN_SINGULAR_VALUE_RELATIVE_MIN: f64 = 1.0e-9;
    /// Below this projected point-covariance ratio a circle arc's curvature
    /// direction is too weak for an unregularized QR solution to define a
    /// stable carrier gauge.
    pub(crate) const CIRCLE_QR_SPREAD_RELATIVE_MIN: f64 = 1.0e-8;
    pub(crate) const MAX_OBSERVABLE_APEX_SCALE: f64 = 1.0e12;
    /// Maximum cylinder-limit fallback apex distance relative to the absolute
    /// coordinate scale. A more remote constructed apex loses enough bits in
    /// point subtraction to cease being a roundoff-scale local cone witness.
    pub(crate) const NEAR_CYLINDRICAL_APEX_COORDINATE_SCALE_MAX: f64 = 524_288.0;
    pub(crate) const NORMAL_GROUP_COSINE_GAP: f64 = 1.0e-12;
    pub(crate) const NEGLIGIBLE_GROUP_WEIGHT_FRACTION: f64 = 1.0e-10;
    pub(crate) const GENERATOR_LINE_VARIANCE_RELATIVE_MIN: f64 = 1.0e-24;
    pub(crate) const GENERATOR_SECOND_VARIANCE_RELATIVE_MAX: f64 = 1.0e-10;
    pub(crate) const GENERATOR_NORMAL_ORTHOGONALITY_MAX: f64 = 1.0e-6;
    pub(crate) const GENERATOR_INTERSECTION_RANK_RELATIVE_MIN: f64 = 1.0e-6;
    pub(crate) const TRIGONOMETRIC_MAGNITUDE_FLOOR: f64 = 1.0e-30;
    pub(crate) const TORUS_AXIS_GAUGE_WEIGHT: f64 = 1.0e-6;
    /// Relative weight of the dimensionless oriented-normal chord loss when
    /// ranking torus initialization candidates. The square root of machine precision is
    /// enough to break point-exact carrier aliases (such as two meridian
    /// circles that are also cospherical) without making ordinary noisy
    /// position fitting subordinate to normal noise.
    pub(crate) const TORUS_NORMAL_OBJECTIVE_WEIGHT: f64 = 1.490_116_119_384_765_6e-8;
    /// Normal error below which competing torus seeds are treated as
    /// orientation-equivalent and retain the established position-only
    /// ranking.
    pub(crate) const TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS: f64 = 1.28e-5;
    /// Seeds whose unoriented axes differ by less than five degrees share one
    /// bounded torus-refinement basin representative.
    pub(crate) const TORUS_MULTISTART_AXIS_CLUSTER_COSINE: f64 = 0.996_194_698_091_745_5;
    /// Multistart is reserved for candidate sets that contain at least one
    /// carrier agreeing with the observed normals within this angle. Larger
    /// discrepancies are ordinary facet-normal approximation, not reliable
    /// evidence for choosing another carrier basin.
    pub(crate) const TORUS_MULTISTART_NORMAL_FIDELITY_MAX_RADIANS: f64 = 1.0e-2;
    /// Minimum sign-invariant rank of the normal second moment required for a
    /// second toroidal curvature direction to be observable. Cylinder normals
    /// lie in one plane for every trim and therefore have exactly zero smallest
    /// eigenvalue; doubly curved torus patches have rank three.
    pub(crate) const TORUS_MULTISTART_NORMAL_MOMENT_RANK_MIN: f64 = 1.0e-8;
    /// Hard ceiling on torus LM starts, including the legacy best seed.
    /// Extra starts are used only when that seed disagrees with the observed
    /// normal field beyond `TORUS_SEED_NORMAL_EQUIVALENCE_RADIANS`.
    pub(crate) const TORUS_MAX_REFINEMENT_STARTS: usize = 6;
    /// Initial multiplicative search half-width for disambiguating a flat
    /// signed normal-offset coplanarity minimum (`exp(+-0.5)`).
    pub(crate) const TORUS_NORMAL_OFFSET_INITIAL_LOG_HALF_WIDTH: f64 = 0.5;
    /// Maximum logarithmic half-width reached only when the initial
    /// normal-offset radius optimum is pinned to a search endpoint.
    pub(crate) const TORUS_NORMAL_OFFSET_LOG_HALF_WIDTH: f64 = 1.5;
    pub(crate) const TORUS_NORMAL_OFFSET_GRID_STEPS: usize = 20;
    pub(crate) const TORUS_NORMAL_OFFSET_REFINEMENT_STEPS: usize = 24;
    /// Loose normalized position-loss ceiling for a normal-guided
    /// initialization candidate. Final fit acceptance remains unchanged.
    pub(crate) const TORUS_NORMAL_OFFSET_POSITION_LOSS_MAX: f64 = 1.0e-4;
    /// Position is only a deterministic tie-breaker within the normal-offset
    /// radius search; observed normal agreement selects the carrier basin.
    pub(crate) const TORUS_NORMAL_OFFSET_POSITION_TIE_WEIGHT: f64 = 1.0e-6;
    /// Parameter-space separation retained between normal-guided torus LM
    /// starts, relative to the sampled patch extent.
    pub(crate) const TORUS_NORMAL_OFFSET_BASIN_RELATIVE_GAP: f64 = 1.0e-8;
    pub(crate) const TORUS_RADIUS_RELATIVE_MIN: f64 = 1.0e-10;
    pub(crate) const TORUS_PLANAR_VARIANCE_RELATIVE_MAX: f64 = 1.0e-12;
    pub(crate) const AXIS_SEED_COSINE_GAP: f64 = 1.0e-8;
    pub(crate) const LOG_SEARCH_MIN: f64 = -13.815510557964274; // ln(1e-6)
    pub(crate) const LOG_SEARCH_MAX: f64 = 9.210340371976184; // ln(1e4)
    /// Golden-section iterations used to refine normal-offset torus-radius
    /// candidates after the coarse logarithmic search.
    pub(crate) const TORUS_COPLANARITY_REFINEMENT_STEPS: usize = 32;

    pub(crate) const LM_INITIAL_DAMPING: f64 = 1.0e-4;
    pub(crate) const LM_DIAGONAL_FLOOR: f64 = 1.0e-12;
    pub(crate) const LM_DAMPING_GROWTH: f64 = 10.0;
    pub(crate) const LM_DAMPING_SHRINK: f64 = 0.3;
    pub(crate) const LM_DAMPING_MIN: f64 = 1.0e-12;
    pub(crate) const LM_DAMPING_MAX: f64 = 1.0e14;
    pub(crate) const LM_STEP_CONVERGENCE: f64 = 1.0e-11;
    pub(crate) const LM_RELATIVE_IMPROVEMENT_CONVERGENCE: f64 = 1.0e-13;
    pub(crate) const FINITE_DIFFERENCE_RELATIVE_STEP: f64 = 2.0e-6;
}

/// Mesh-analysis safeguards derived from machine precision.
pub(crate) mod mesh {
    /// Multiplier on the local edge/coordinate cross-product uncertainty used
    /// to classify triangles whose area is not numerically observable.
    pub(crate) const MACHINE_AREA_FLOOR_MULTIPLIER: f64 = 16.0;
}

/// Generic recognition and deterministic model-selection safeguards.
pub(crate) mod recognition {
    /// Complexity tie-breaker contributed by each fitted degree of freedom.
    /// One unit is 0.01% of a normalized validation threshold: enough to
    /// prefer a simpler carrier when faceting lets a high-radius limit model
    /// absorb imperceptible chord error, but far too small to excuse a visibly
    /// worse positional or normal fit.
    pub(crate) const MODEL_COMPLEXITY_PENALTY_UNIT: f64 = 1.0e-4;
    /// Maximum RMS-position advantage, as a fraction of the accepted spatial
    /// tolerance, that cannot identify a torus over an otherwise valid
    /// cylinder when the torus also fails to improve the normal fit.
    pub(crate) const TORUS_CYLINDER_LIMIT_RMS_GAIN_FRACTION: f64 = 1.0e-2;
    /// Positive floors used only to normalize model-selection residuals.
    pub(crate) const SCORE_DISTANCE_DENOMINATOR_FLOOR: f64 = 1.0e-30;
    pub(crate) const SCORE_NORMAL_DENOMINATOR_FLOOR: f64 = 1.0e-12;
    /// Machine-roundoff multiplier for the exact-vertex normal exception.
    pub(crate) const EXACT_VERTEX_ROUNDOFF_MULTIPLIER: f64 = 256.0;
    /// Normal-angle ceiling for placing a fitted carrier in the numerically
    /// exact model-selection tier. This accommodates `acos` roundoff and
    /// derivative-normal conditioning while remaining far below recognition
    /// tolerances used for ordinary geometric acceptance.
    pub(crate) const EXACT_MODEL_NORMAL_ROUNDOFF_RADIANS: f64 = 1.28e-5;
}
