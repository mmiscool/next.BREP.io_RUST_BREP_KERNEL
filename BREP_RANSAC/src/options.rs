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

/// How generic hypotheses are drawn when a connected component is not one
/// carrier.
///
/// Hidden and outside the semver contract: no census has favoured a policy
/// other than [`HypothesisPolicy::Classical`] (validation records of
/// 2026-10-28), so the adaptive policy stays an experiment until one does.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
pub enum HypothesisPolicy {
    /// Every primitive kind in turn, each over its own uniform permutation of
    /// the remaining triangles, from a probe of 128 triangles doubled while a
    /// candidate does not outgrow it; a round stops at the classical bound
    /// `ln(1 - confidence) / ln(1 - w)` seeds per kind, `w` the incumbent's
    /// share of the remaining triangles.
    Classical,
    /// Seeds, kinds and probe sizes drawn by [`AdaptiveSampling`]: a
    /// Boltzmann distribution over the triangles' history mixed with a
    /// uniform floor, a cost-aware bandit over kinds and over probe sizes,
    /// and a risk bound that credits the uniform floor alone.
    Adaptive(AdaptiveSampling),
}

impl Default for HypothesisPolicy {
    fn default() -> Self {
        Self::Classical
    }
}

/// Parameters of [`HypothesisPolicy::Adaptive`].
///
/// The policy's three temperatures, one per axis of the search:
/// - spatial: each hypothesis draws its seed from a mixture, with probability
///   `seed_exploration` uniformly over the remaining triangles, otherwise from
///   `exp(-explored / T)` over the triangles outside the round's incumbent
///   that this kind has not seeded yet, where `explored` counts the losing
///   supports a triangle has sat in and `T` is `seed_temperature` times one
///   of {0.05, 0.1, 0.25, 0.5, 1, 2, ∞}, chosen by Thompson sampling on how
///   often draws at that temperature improved the incumbent and by how much
///   (the infinite temperature is uniform over those triangles, so reheating
///   is one arm among the others);
/// - primitive family: the kind is drawn by Thompson sampling on the support
///   a kind's hypotheses gain per unit of fitting cost, learned over the
///   component's rounds, every kind worth a draw keeping `kind_floor` of the
///   round's draws;
/// - sample: the probe around the seed starts at 128 triangles and doubles
///   while the candidate does not outgrow it, as in the classical policy, but
///   at the first probe whose candidate explained fewer than `minimum_support`
///   triangles the chain is kept only with the probability that such a chain
///   has been rescued for this kind so far, never below `probe_floor`; a kept
///   chain runs on as the classical one does.
///
/// A round stops for one of three separately reported reasons (see
/// [`crate::RecognitionWork`]): the risk bound, seed exhaustion, or the
/// hypothesis cap. The risk bound of each kind is
/// `(1 - seed_exploration * w)` to the power of the kind's draws, `w` the
/// incumbent's share of the round's triangles, and it certifies the round
/// when every kind's bound is at most the round's share of the component's
/// error budget: `(1 - confidence)` times the incumbent's share of the
/// component. Extracted regions are disjoint, so the certified rounds' shares
/// sum to at most `1 - confidence`, which bounds the probability that any
/// certified round missed a support larger than the one it extracted; rounds
/// that stop for another reason certify nothing and spend nothing. The uniform
/// part is the only part of the draw the bound credits, so it stays valid
/// however the Boltzmann part and the bandits adapt.
#[doc(hidden)]
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(default)]
#[non_exhaustive]
pub struct AdaptiveSampling {
    /// Probability that a hypothesis takes its seed uniformly from the
    /// remaining triangles, in (0, 1]. The risk bound credits these draws
    /// alone, so a smaller value explores more and certifies more slowly.
    pub seed_exploration: f64,
    /// Scale of the spatial temperature bank over the explored counts;
    /// positive. Lower concentrates every arm on unexplored triangles.
    pub seed_temperature: f64,
    /// Share of a round's draws every eligible primitive kind receives at
    /// least, divided evenly among the kinds, in [0, 1]. The remainder goes to
    /// the kinds the posterior ranks best on support per cost.
    pub kind_floor: f64,
    /// Probability, at least, of keeping a probe chain past its first size
    /// whose candidate explained fewer than `minimum_support` triangles, in
    /// [0, 1]; one coin per chain. 1 restores the classical chain for every
    /// hypothesis; 0 keeps a chain only as often as the kind's rescue rate
    /// says.
    pub probe_floor: f64,
}

impl Default for AdaptiveSampling {
    fn default() -> Self {
        Self {
            seed_exploration: 0.5,
            seed_temperature: 1.0,
            kind_floor: 0.5,
            probe_floor: 0.1,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(default)]
/// Controls surface recognition, refinement, and region discovery.
///
/// Non-exhaustive: start from [`RecognitionOptions::default`] and set the
/// fields you need, so a later field is not a breaking change.
#[non_exhaustive]
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
    /// How generic hypotheses are drawn for a component no single carrier
    /// explains. [`HypothesisPolicy::Classical`] by default. Hidden and
    /// outside the semver contract, like [`HypothesisPolicy`].
    #[doc(hidden)]
    pub hypothesis_policy: HypothesisPolicy,
    /// Worker threads for the work recognition splits without changing its
    /// result: the connected components of a run without an attempt budget,
    /// and the hypotheses of one classical extraction round, which are
    /// evaluated in batches and applied in draw order. 0 uses one worker per
    /// core the process may run on; 1 runs everything on the caller's thread.
    /// On `wasm32` every value runs on the caller's thread. The regions,
    /// their diagnostics and the reported work are the same for every value.
    pub threads: usize,
    /// Lane U row 3: run a torus seed's roundoff-recovery normal-offset
    /// refinement only when some seed already agrees with the observed
    /// normals within the multistart fidelity (1e-2 rad), the condition its
    /// normal-disambiguation twin already requires. The refinement keeps only
    /// candidates whose every normal agrees to 1.28e-5 rad, which a sample
    /// whose best seed is 780 times further off has not been seen to reach;
    /// on noisy meshes it ran on every torus fit and kept nothing. Off by
    /// default.
    pub torus_fast: bool,
    /// A hypothesis's diversity probe does not cross a mesh edge whose two
    /// triangle normals differ by more than `normal_tolerance`, so a sample
    /// is drawn from one smooth wall. On a coarse tessellation of two crossing
    /// cylinders one region of ~800 long triangles held both walls, a
    /// 128-triangle probe spanned the crease, and every normal-diverse
    /// cylinder sample mixed the walls (fitted radii 9-2 900 against 10).
    /// Off by default.
    pub probe_crease_gate: bool,
    /// The triangles a hypothesis's diversity probe first grows to (it
    /// doubles while a candidate explains no more than its probe). On a
    /// coarse tessellation whose walls hold about 100 long triangles each, a
    /// 128-triangle probe spans two walls whatever its seed. Defaults to 128.
    pub hypothesis_probe_triangles: usize,
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
            hypothesis_policy: HypothesisPolicy::Classical,
            threads: 0,
            torus_fast: false,
            probe_crease_gate: false,
            hypothesis_probe_triangles: 128,
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
        if let HypothesisPolicy::Adaptive(adaptive) = self.hypothesis_policy {
            if !(adaptive.seed_exploration > 0.0 && adaptive.seed_exploration <= 1.0) {
                return Err(crate::RecognitionError::InvalidOptions(
                    "seed_exploration must be in (0, 1]".into(),
                ));
            }
            if !(adaptive.seed_temperature.is_finite() && adaptive.seed_temperature > 0.0) {
                return Err(crate::RecognitionError::InvalidOptions(
                    "seed_temperature must be finite and positive".into(),
                ));
            }
            if !(0.0..=1.0).contains(&adaptive.kind_floor)
                || !(0.0..=1.0).contains(&adaptive.probe_floor)
            {
                return Err(crate::RecognitionError::InvalidOptions(
                    "kind_floor and probe_floor must be in [0, 1]".into(),
                ));
            }
        }
        Ok(())
    }
}

