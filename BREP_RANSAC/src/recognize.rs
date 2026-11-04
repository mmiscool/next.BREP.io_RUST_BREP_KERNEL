use crate::fit::{
    evaluate_surface, evaluate_surface_from_vertices, fit_profile_clock, fit_profile_note,
    fit_profile_start, fit_profile_take, fit_surface_from_vertices_with_path, fit_surface_with_path,
    selection_scale, vertex_selection_scale, FitProfile,
};
use crate::numerical::recognition as numerical;
use crate::{
    AdaptiveSampling, AnalyticSurface, AnalyzedMesh, ConstraintMask, FitPath, HypothesisPolicy,
    Mesh, MeshAnalysisOptions, MetadataTrust, RecognitionError, RecognitionOptions,
    RecognitionResult, SurfaceFitResult, SurfaceHint, SurfaceRegion, SurfaceType,
    UnresolvedRegionDiagnostic,
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
    recognize_budgeted(mesh, options, &mut AttemptBudget::unlimited(), &[])
}

/// Work spent by [`recognize_surfaces_with_attempt_budget`].
///
/// Non-exhaustive: a later counter is not a breaking change.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[non_exhaustive]
pub struct RecognitionWork {
    /// Generic hypotheses drawn, over every extraction round of every
    /// connected component.
    pub attempts: usize,
    /// Regions accepted by generic extraction.
    pub rounds: usize,
    /// Whether the budget ran out before extraction finished. The regions
    /// are then those accepted before the budget ran out (the round in
    /// progress is dropped), so they are a prefix of the unbudgeted run's.
    pub exhausted: bool,
    /// Connected components accepted whole on a supplied
    /// [`ComponentCandidate`] after their own whole fit was refused.
    pub seeded_components: usize,
    /// Generic hypotheses drawn per primitive kind, in the recognizer's kind
    /// order (plane, sphere, cylinder, cone, torus).
    pub hypotheses_by_kind: [usize; 5],
    /// Extraction rounds that stopped because every kind's miss bound fell
    /// to `1 - confidence` (the classical seed bound, or the adaptive
    /// policy's product bound).
    pub stops_at_risk_bound: usize,
    /// Extraction rounds that stopped at `max_hypotheses` (or at one draw per
    /// remaining triangle per kind, when that is smaller) with the bound not
    /// yet met.
    pub stops_at_hypothesis_cap: usize,
    /// Extraction rounds of the adaptive policy that stopped because every
    /// remaining triangle had seeded every kind.
    pub stops_at_seed_exhaustion: usize,
    /// Extraction rounds cut short by the attempt budget; the round's
    /// candidates are dropped and `exhausted` is set.
    pub stops_at_budget: usize,
}

impl RecognitionWork {
    /// Adds the work of a component extracted on its own budget.
    fn absorb(&mut self, other: &RecognitionWork) {
        self.attempts += other.attempts;
        self.rounds += other.rounds;
        self.exhausted |= other.exhausted;
        self.seeded_components += other.seeded_components;
        for (mine, theirs) in self.hypotheses_by_kind.iter_mut().zip(&other.hypotheses_by_kind) {
            *mine += theirs;
        }
        self.stops_at_risk_bound += other.stops_at_risk_bound;
        self.stops_at_hypothesis_cap += other.stops_at_hypothesis_cap;
        self.stops_at_seed_exhaustion += other.stops_at_seed_exhaustion;
        self.stops_at_budget += other.stops_at_budget;
    }
}

/// Where generic extraction spent its wall time and its work, returned by
/// [`recognize_surfaces_with_extraction_profile`]. A measuring aid for the
/// validation probes, not a stable interface: the fields follow the
/// extraction loop. Seconds are summed over every hypothesis, so with more
/// than one worker they exceed the wall time.
#[doc(hidden)]
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExtractionProfile {
    /// Wall time of generic extraction over every component (whole fits,
    /// supplied candidates and the rounds), the denominator of the shares.
    pub extraction_seconds: f64,
    /// Whole-component fits and supplied-candidate validation.
    pub whole_fit_seconds: f64,
    /// Rebuilding the round's remaining ids and its seed schedules
    /// (classical: one shuffle per kind; adaptive: the per-round tables).
    pub seed_order_seconds: f64,
    /// Adaptive only: the kind bandit and the Boltzmann seed draw.
    pub seed_choice_seconds: f64,
    /// Breadth-first probes around the seeds.
    pub neighborhood_seconds: f64,
    /// Normal-diverse near-minimal samples.
    pub sample_seconds: f64,
    /// Sample fits per kind, in the recognizer's kind order (plane, sphere,
    /// cylinder, cone, torus).
    pub fit_seconds: [f64; 5],
    /// Sample fits per kind.
    pub fits: [usize; 5],
    /// Support growth (`support_component`).
    pub support_seconds: f64,
    /// Support refits whose support count was at least the round's
    /// incumbent count when the hypothesis was drawn.
    pub refit_winning_seconds: f64,
    /// Support refits whose support count was below that incumbent: they
    /// cannot replace it.
    pub refit_dominated_seconds: f64,
    /// Refit events (a probe chain can refit more than once).
    pub refits_winning: usize,
    /// Dominated refit events.
    pub refits_dominated: usize,
    /// Dominated refits whose outcome cannot change the probe chain's next
    /// step (the support did not grow beyond its probe, or the probe was
    /// exhausted), so skipping them is exact.
    pub refits_dominated_exact: usize,
    /// Seconds of those.
    pub refit_dominated_exact_seconds: f64,
    /// Ranking refits (`selection_scale`, `model_rank`).
    pub rank_seconds: f64,
    /// The share of `rank_seconds` spent on dominated refits.
    pub rank_dominated_seconds: f64,
    /// Rounds run, including abandoned ones.
    pub rounds: usize,
    /// Hypotheses drawn in those rounds.
    pub seeds_consumed: usize,
    /// `remaining × kinds` summed over those rounds: the seeds a full
    /// schedule offers.
    pub seed_slots: usize,
    /// Triangles the support test ran on.
    pub support_triangles_tested: usize,
    /// Vertex distance evaluations of the support test.
    pub distance_evaluations: usize,
    /// Distinct vertices among those, per hypothesis, summed.
    pub distinct_vertices_evaluated: usize,
    /// Rounds that found no candidate and abandoned the rest of the
    /// component.
    pub abandoned_rounds: usize,
    /// Triangles those rounds left unresolved.
    pub abandoned_triangles: usize,
    /// Abandoned remainders that still held a connected piece of at least
    /// `minimum_support` triangles.
    pub abandoned_with_piece: usize,
    /// Triangles in such pieces.
    pub abandoned_piece_triangles: usize,
    /// Inside the sample fits (lane U row 1).
    pub sample_fitter: FitProfile,
    /// Inside the whole-component fits: model selection per kind.
    pub whole_fitter: FitProfile,
    /// Inside the support refits.
    pub refit_fitter: FitProfile,
    /// Torus sample fits by what came of them: the fit returned an error,
    /// its support stayed under `minimum_support`, the support refit failed,
    /// the refit succeeded.
    pub torus_sample_outcomes: [usize; 4],
    /// Seconds of those torus sample fits, by the same outcome.
    pub torus_sample_outcome_seconds: [f64; 4],
}

impl ExtractionProfile {
    fn absorb(&mut self, other: &ExtractionProfile) {
        self.whole_fit_seconds += other.whole_fit_seconds;
        self.seed_order_seconds += other.seed_order_seconds;
        self.seed_choice_seconds += other.seed_choice_seconds;
        self.neighborhood_seconds += other.neighborhood_seconds;
        self.sample_seconds += other.sample_seconds;
        for kind in 0..5 {
            self.fit_seconds[kind] += other.fit_seconds[kind];
            self.fits[kind] += other.fits[kind];
        }
        self.support_seconds += other.support_seconds;
        self.refit_winning_seconds += other.refit_winning_seconds;
        self.refit_dominated_seconds += other.refit_dominated_seconds;
        self.refits_winning += other.refits_winning;
        self.refits_dominated += other.refits_dominated;
        self.refits_dominated_exact += other.refits_dominated_exact;
        self.refit_dominated_exact_seconds += other.refit_dominated_exact_seconds;
        self.rank_seconds += other.rank_seconds;
        self.rank_dominated_seconds += other.rank_dominated_seconds;
        self.rounds += other.rounds;
        self.seeds_consumed += other.seeds_consumed;
        self.seed_slots += other.seed_slots;
        self.support_triangles_tested += other.support_triangles_tested;
        self.distance_evaluations += other.distance_evaluations;
        self.distinct_vertices_evaluated += other.distinct_vertices_evaluated;
        self.abandoned_rounds += other.abandoned_rounds;
        self.abandoned_triangles += other.abandoned_triangles;
        self.abandoned_with_piece += other.abandoned_with_piece;
        self.abandoned_piece_triangles += other.abandoned_piece_triangles;
        self.sample_fitter.absorb(&other.sample_fitter);
        self.whole_fitter.absorb(&other.whole_fitter);
        self.refit_fitter.absorb(&other.refit_fitter);
        for outcome in 0..4 {
            self.torus_sample_outcomes[outcome] += other.torus_sample_outcomes[outcome];
            self.torus_sample_outcome_seconds[outcome] += other.torus_sample_outcome_seconds[outcome];
        }
    }

    /// Moves what this thread's fitter collected since [`fit_profile_start`]
    /// into the bucket `slot` picks.
    fn take_fitter(profile: &mut Option<Self>, slot: fn(&mut Self) -> &mut FitProfile) {
        if let (Some(profile), Some(fitter)) = (profile, fit_profile_take()) {
            slot(profile).absorb(&fitter);
        }
    }

    /// One torus sample fit of `seconds` that ended in `outcome` (see
    /// [`Self::torus_sample_outcomes`]).
    fn torus_sample(&mut self, outcome: usize, seconds: f64) {
        self.torus_sample_outcomes[outcome] += 1;
        self.torus_sample_outcome_seconds[outcome] += seconds;
    }

    /// Adds the seconds since `started` to the field `slot` picks.
    fn charge(profile: &mut Option<Self>, started: Option<Instant>, slot: fn(&mut Self) -> &mut f64) {
        if let (Some(profile), Some(started)) = (profile, started) {
            *slot(profile) += started.elapsed().as_secs_f64();
        }
    }

    /// One support refit of `count` triangles against an incumbent of
    /// `incumbent`; `exact` when skipping it could not change the chain.
    fn refit(&mut self, started: Option<Instant>, count: usize, incumbent: usize, exact: bool) {
        let seconds = started.map_or(0.0, |started| started.elapsed().as_secs_f64());
        if count < incumbent {
            self.refits_dominated += 1;
            self.refit_dominated_seconds += seconds;
            if exact {
                self.refits_dominated_exact += 1;
                self.refit_dominated_exact_seconds += seconds;
            }
        } else {
            self.refits_winning += 1;
            self.refit_winning_seconds += seconds;
        }
    }

    /// An abandoned round: how much of `remaining` was a connected piece the
    /// next round could still have drawn from.
    fn abandon(&mut self, mesh: &AnalyzedMesh, remaining: &Remaining, options: &RecognitionOptions) {
        self.abandoned_rounds += 1;
        self.abandoned_triangles += remaining.len();
        let ids: Vec<usize> = remaining.iter().copied().collect();
        let pieces: usize = mesh
            .connected_components(&ids, options.respect_features)
            .iter()
            .filter(|piece| piece.len() >= options.minimum_support)
            .map(Vec::len)
            .sum();
        if pieces > 0 {
            self.abandoned_with_piece += 1;
            self.abandoned_piece_triangles += pieces;
        }
    }
}

/// Distance work of one support growth.
#[derive(Default)]
struct SupportTally {
    triangles: usize,
    evaluations: usize,
    vertices: std::collections::HashSet<usize>,
}

/// [`recognize_surfaces_with_attempt_budget`], also returning where generic
/// extraction spent its time. The regions and the work are those of the
/// unprofiled call; only the clock reads differ.
#[doc(hidden)]
pub fn recognize_surfaces_with_extraction_profile(
    mesh: &Mesh,
    options: &RecognitionOptions,
    max_attempts: usize,
    kinds: &[SurfaceType],
) -> Result<(RecognitionResult, RecognitionWork, ExtractionProfile), RecognitionError> {
    let kinds: Vec<_> = TYPES.into_iter().filter(|kind| kinds.is_empty() || kinds.contains(kind)).collect();
    let mut budget = AttemptBudget {
        limit: Some(max_attempts),
        kinds,
        work: RecognitionWork::default(),
        profile: Some(ExtractionProfile::default()),
    };
    let result = recognize_budgeted(mesh, options, &mut budget, &[])?;
    Ok((result, budget.work, budget.profile.unwrap_or_default()))
}

fn kind_index(kind: SurfaceType) -> usize {
    TYPES
        .iter()
        .position(|&candidate| candidate == kind)
        .expect("every primitive kind is in TYPES")
}

/// A carrier another stage already fitted to exactly the triangles of one
/// connected component, offered to [`recognize_surfaces_with_candidates`].
#[derive(Clone, Debug, PartialEq)]
pub struct ComponentCandidate {
    /// The component's triangles, in any order.
    pub triangle_indices: Vec<usize>,
    /// The carrier to validate on them.
    pub surface: AnalyticSurface,
}

/// Roundoff allowance, in units of `f64::EPSILON` times the coordinate
/// magnitude, for a [`ComponentCandidate`] whose fitting stage measured its
/// maximum vertex deviation with the same expression (see
/// [`AnalyticSurface::signed_distance`]) in a different operation order.
///
/// Each distance is at most ~30 floating-point operations (the torus: 3
/// subtractions, a 5-operation dot product, 6 for the axial projection, 6
/// and a square root for its length, then 5 more and a second square root),
/// every intermediate bounded by twice the magnitude `M` of the coordinates
/// and parameters involved. Each operation contributes at most `eps / 2`
/// relative error, so one evaluation is within `30 * eps * M` of the exact
/// value and two evaluations differ by at most `60 * eps * M`; 64 rounds that
/// up. It is not a tolerance on the fit: on the measured near-misses this
/// path exists for, the refit missed by 1e-6 to 4e-5 relative, and the
/// allowance is about 1e-14.
const CANDIDATE_ROUNDOFF_EPSILONS: f64 = 64.0;

/// [`recognize_surfaces_with_unresolved`] with a cap on the generic
/// hypotheses drawn over the whole call, and optionally fewer primitive
/// kinds to draw them for.
///
/// `max_hypotheses` bounds one extraction round only, and extraction keeps
/// running rounds while any candidate reaches `minimum_support`; on a mesh
/// where only small patches fit, the rounds, and the time, are unbounded.
/// Up to the cap the run is identical to the unbudgeted one: hypotheses are
/// drawn in the same deterministic order.
///
/// `kinds` limits generic hypotheses to those primitive kinds, drawn in the
/// recognizer's own kind order; empty means every kind, and then the run is
/// the unbudgeted one up to the cap. A component that one carrier explains
/// whole is still classified among every kind.
pub fn recognize_surfaces_with_attempt_budget(
    mesh: &Mesh,
    options: &RecognitionOptions,
    max_attempts: usize,
    kinds: &[SurfaceType],
) -> Result<(RecognitionResult, RecognitionWork), RecognitionError> {
    let kinds: Vec<_> = TYPES.into_iter().filter(|kind| kinds.is_empty() || kinds.contains(kind)).collect();
    let mut budget = AttemptBudget {
        limit: Some(max_attempts),
        kinds,
        work: RecognitionWork::default(),
        profile: None,
    };
    let result = recognize_budgeted(mesh, options, &mut budget, &[])?;
    Ok((result, budget.work))
}

/// [`recognize_surfaces_with_unresolved`], except that a connected component
/// whose own whole fit is refused is first validated on a supplied candidate
/// carrier for exactly its triangles before generic extraction divides it.
///
/// The candidate is accepted by the ordinary distance, normal and support
/// gates, with only `CANDIDATE_ROUNDOFF_EPSILONS` added to the distance
/// tolerance. A component whose whole fit passes, or that has no candidate,
/// runs exactly as in [`recognize_surfaces_with_unresolved`], so with no
/// candidates the result is identical. The returned work counts the
/// generic hypotheses drawn and the components accepted on a candidate.
pub fn recognize_surfaces_with_candidates(
    mesh: &Mesh,
    options: &RecognitionOptions,
    candidates: &[ComponentCandidate],
) -> Result<(RecognitionResult, RecognitionWork), RecognitionError> {
    let mut budget = AttemptBudget::unlimited();
    // Sorted triangle sets and their candidate carriers.
    let candidates: Vec<(Vec<usize>, AnalyticSurface)> = candidates
        .iter()
        .map(|candidate| {
            let mut ids = candidate.triangle_indices.clone();
            ids.sort_unstable();
            ids.dedup();
            (ids, candidate.surface)
        })
        .collect();
    let result = recognize_budgeted(mesh, options, &mut budget, &candidates)?;
    Ok((result, budget.work))
}

struct AttemptBudget {
    limit: Option<usize>,
    /// Kinds generic hypotheses are drawn for, in [`TYPES`] order.
    kinds: Vec<SurfaceType>,
    work: RecognitionWork,
    /// Collected only by [`recognize_surfaces_with_extraction_profile`].
    profile: Option<ExtractionProfile>,
}

impl AttemptBudget {
    fn unlimited() -> Self {
        Self {
            limit: None,
            kinds: TYPES.to_vec(),
            work: RecognitionWork::default(),
            profile: None,
        }
    }

    /// Takes up to `want` attempts, as many as the limit allows, without
    /// marking the budget exhausted: the caller knows whether the run would
    /// have asked for the attempt the limit refuses.
    fn admit(&mut self, want: usize) -> usize {
        if self.work.exhausted {
            return 0;
        }
        let admitted = match self.limit {
            Some(limit) => want.min(limit.saturating_sub(self.work.attempts)),
            None => want,
        };
        self.work.attempts += admitted;
        admitted
    }

    /// Returns admitted attempts the run turned out not to draw.
    fn refund(&mut self, count: usize) {
        self.work.attempts -= count;
    }

    /// Takes one attempt, or marks the budget exhausted and refuses.
    fn take(&mut self) -> bool {
        if self.work.exhausted || self.limit.is_some_and(|limit| self.work.attempts >= limit) {
            self.work.exhausted = true;
            return false;
        }
        self.work.attempts += 1;
        true
    }
}

fn recognize_budgeted(
    mesh: &Mesh,
    options: &RecognitionOptions,
    budget: &mut AttemptBudget,
    candidates: &[(Vec<usize>, AnalyticSurface)],
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
    let extraction_started = budget.profile.is_some().then(Instant::now);
    extract_components(&analyzed, &components, options, candidates, &mut regions, budget);
    ExtractionProfile::charge(&mut budget.profile, extraction_started, |p| &mut p.extraction_seconds);
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
    let mut tally = SelectionTally::start(selection);
    let tolerance = options.distance_tolerance + options.relative_tolerance * scale.max(1.0);
    let mut candidates = Vec::new();
    let mut rejected = Vec::new();
    for kind in TYPES {
        let fit_started = tally.clock(&candidates);
        match selection.fit(mesh, kind, None, fixed, options, path) {
            Ok(model) if selection.accepted(&model, options, scale) => {
                let rank_started = tally.fitted(fit_started, kind, 0);
                let rank = model_rank(
                    mesh,
                    selection,
                    kind,
                    &model,
                    tolerance,
                    options.normal_tolerance,
                );
                tally.ranked(rank_started);
                candidates.push((rank, kind, model));
            }
            Ok(model) => {
                tally.fitted(fit_started, kind, 1);
                rejected.push((
                    kind,
                    format!(
                        "max distance {:.3e}, max normal {:.3e}",
                        model.metrics.max_error, model.metrics.max_normal_error
                    ),
                ))
            }
            Err(e) => {
                tally.fitted(fit_started, kind, 2);
                rejected.push((kind, e.to_string()))
            }
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
                        tally.finish(Some(best_kind), true);
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
    let filter_started = tally.filter_clock();
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
    tally.filtered(filter_started);
    candidates.sort_by(|a, b| compare_model_rank(a.0, a.1, b.0, b.1));
    tally.finish(candidates.first().map(|candidate| candidate.1), false);
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

/// [`FitProfile`] bookkeeping of one `best_model_impl` call, inert unless the
/// thread collects a profile.
struct SelectionTally {
    started: Option<Instant>,
    size: usize,
    /// Per kind: seconds of its fit, when it ran.
    seconds: [Option<f64>; 5],
    /// Per kind: 0 accepted, 1 failed the gates, 2 error.
    outcome: [u8; 5],
    after_exact: [bool; 5],
    after_valid: [bool; 5],
    rank_seconds: f64,
    filter_seconds: f64,
}

impl SelectionTally {
    fn start(selection: Selection<'_>) -> Self {
        let size = match selection {
            Selection::Triangles(ids) | Selection::Vertices(ids) => ids.len(),
        };
        Self {
            started: fit_profile_clock(),
            size,
            seconds: [None; 5],
            outcome: [0; 5],
            after_exact: [false; 5],
            after_valid: [false; 5],
            rank_seconds: 0.0,
            filter_seconds: 0.0,
        }
    }

    /// The clock before a kind's fit, noting what the earlier kinds left.
    fn clock(&mut self, candidates: &[(ModelRank, SurfaceType, SurfaceFitResult)]) -> Option<(Instant, bool, bool)> {
        self.started?;
        let exact = candidates.iter().any(|(rank, ..)| rank.numerically_exact);
        Some((Instant::now(), exact, !candidates.is_empty()))
    }

    fn fitted(&mut self, started: Option<(Instant, bool, bool)>, kind: SurfaceType, outcome: u8) -> Option<Instant> {
        let (started, exact, valid) = started?;
        let index = kind_index(kind);
        self.seconds[index] = Some(started.elapsed().as_secs_f64());
        self.outcome[index] = outcome;
        self.after_exact[index] = exact;
        self.after_valid[index] = valid;
        Some(Instant::now())
    }

    fn ranked(&mut self, started: Option<Instant>) {
        if let Some(started) = started {
            self.rank_seconds += started.elapsed().as_secs_f64();
        }
    }

    fn filter_clock(&self) -> Option<Instant> {
        self.started.map(|_| Instant::now())
    }

    fn filtered(&mut self, started: Option<Instant>) {
        if let Some(started) = started {
            self.filter_seconds += started.elapsed().as_secs_f64();
        }
    }

    fn finish(&self, selected: Option<SurfaceType>, bound_exit: bool) {
        let tally = self;
        fit_profile_note(self.started, |profile, seconds| {
            profile.selections += 1;
            profile.selection_size += tally.size;
            profile.selections_bound_exit += usize::from(bound_exit);
            if selected.is_none() {
                profile.selections_failed += 1;
                profile.selections_failed_seconds += seconds;
            }
            profile.rank_seconds += tally.rank_seconds;
            profile.filter_seconds += tally.filter_seconds;
            for index in 0..5 {
                let Some(fit_seconds) = tally.seconds[index] else {
                    continue;
                };
                profile.kind_fits[index] += 1;
                profile.kind_seconds[index] += fit_seconds;
                match tally.outcome[index] {
                    1 => profile.kind_rejected[index] += 1,
                    2 => profile.kind_errors[index] += 1,
                    _ => {}
                }
                if selected.is_some_and(|kind| kind_index(kind) == index) {
                    profile.kind_won[index] += 1;
                } else {
                    if tally.outcome[index] == 0 {
                        profile.kind_valid_lost[index] += 1;
                    }
                    profile.kind_lost_seconds[index] += fit_seconds;
                }
                if tally.after_exact[index] {
                    profile.kind_after_exact[index] += 1;
                    profile.kind_after_exact_seconds[index] += fit_seconds;
                }
                if tally.after_valid[index] {
                    profile.kind_after_valid[index] += 1;
                    profile.kind_after_valid_seconds[index] += fit_seconds;
                }
            }
        });
    }
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

/// Worker threads the options ask for: the cores the process may run on
/// when `threads` is 0, else `threads`; one on `wasm32`, which has no
/// threads to spawn.
fn worker_threads(options: &RecognitionOptions) -> usize {
    #[cfg(target_arch = "wasm32")]
    {
        let _ = options;
        1
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        match options.threads {
            0 => std::thread::available_parallelism().map_or(1, |cores| cores.get()),
            threads => threads,
        }
    }
}

/// `f(0)..f(count)` in index order. With more than one worker on a native
/// target the indices are handed out dynamically to `workers` scoped
/// threads; otherwise they run in order on the caller's thread. A panic in
/// a worker is resumed on the caller's thread.
fn parallel_map<T: Send>(workers: usize, count: usize, f: impl Fn(usize) -> T + Sync) -> Vec<T> {
    parallel_map_on_workers(workers, count, |_, index| f(index))
}

/// [`parallel_map`], also passing the worker evaluating each index (below
/// `workers`, and 0 on the caller's thread), so each worker can own scratch.
fn parallel_map_on_workers<T: Send>(
    workers: usize,
    count: usize,
    f: impl Fn(usize, usize) -> T + Sync,
) -> Vec<T> {
    #[cfg(target_arch = "wasm32")]
    let _ = workers;
    #[cfg(not(target_arch = "wasm32"))]
    {
        let workers = workers.min(count);
        if workers > 1 {
            let next = std::sync::atomic::AtomicUsize::new(0);
            let collected: Vec<Vec<(usize, T)>> = std::thread::scope(|scope| {
                let handles: Vec<_> = (0..workers)
                    .map(|worker| {
                        let (next, f) = (&next, &f);
                        scope.spawn(move || {
                            let mut mine = Vec::new();
                            loop {
                                let index = next.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                                if index >= count {
                                    break;
                                }
                                mine.push((index, f(worker, index)));
                            }
                            mine
                        })
                    })
                    .collect();
                handles
                    .into_iter()
                    .map(|handle| handle.join().unwrap_or_else(|payload| std::panic::resume_unwind(payload)))
                    .collect()
            });
            let mut slots: Vec<Option<T>> = (0..count).map(|_| None).collect();
            for (index, value) in collected.into_iter().flatten() {
                slots[index] = Some(value);
            }
            return slots.into_iter().map(|slot| slot.expect("every index was evaluated once")).collect();
        }
    }
    (0..count).map(|index| f(0, index)).collect()
}

/// Generic extraction over the connected components. When there are at least
/// as many components that can need extraction as workers, the workers take
/// the components, largest first, each on a local budget without a limit, and
/// the results are merged in component order, which is what the one-at-a-time
/// loop produces (also for the merge of disconnected regions that follows).
/// Under an attempt limit the one-at-a-time loop charges the components in
/// order and stops at the first draw the limit refuses, so the merge replays
/// that accounting: a component whose draws fit in what is left is taken as
/// computed, the first one that does not is run again on the real budget (it
/// is cut inside, and marks the budget exhausted), and the components after
/// it contribute nothing, as in the one-at-a-time loop. Otherwise the
/// components run in order and the workers go to the rounds' hypothesis
/// batches.
fn extract_components(
    mesh: &AnalyzedMesh,
    components: &[Vec<usize>],
    options: &RecognitionOptions,
    candidates: &[(Vec<usize>, AnalyticSurface)],
    out: &mut Vec<SurfaceRegion>,
    budget: &mut AttemptBudget,
) {
    let workers = worker_threads(options);
    let eligible = components
        .iter()
        .filter(|component| component.len() >= options.minimum_support)
        .count();
    if workers > 1 && eligible >= workers && !budget.work.exhausted {
        let mut largest_first: Vec<usize> = (0..components.len()).collect();
        largest_first.sort_by_key(|&index| std::cmp::Reverse(components[index].len()));
        let kinds = &budget.kinds;
        let profiling = budget.profile.is_some();
        let mut extracted = parallel_map(workers, components.len(), |slot| {
            let index = largest_first[slot];
            let mut local = AttemptBudget {
                limit: None,
                kinds: kinds.clone(),
                work: RecognitionWork::default(),
                profile: profiling.then(ExtractionProfile::default),
            };
            let mut regions = Vec::new();
            extract_component(mesh, &components[index], options, candidates, &mut regions, &mut local, 1);
            (index, regions, local.work, local.profile)
        });
        extracted.sort_by_key(|(index, ..)| *index);
        for (index, regions, work, profile) in extracted {
            debug_assert!(!work.exhausted && work.stops_at_budget == 0, "an unlimited budget refuses nothing");
            if budget.work.exhausted {
                break;
            }
            let fits = budget
                .limit
                .is_none_or(|limit| budget.work.attempts.saturating_add(work.attempts) <= limit);
            if fits {
                out.extend(regions);
                budget.work.absorb(&work);
                if let (Some(total), Some(profile)) = (&mut budget.profile, &profile) {
                    total.absorb(profile);
                }
            } else {
                // The one-at-a-time loop is cut inside this component: run it
                // on the real budget, which refuses the draw the limit
                // refuses there and marks the budget exhausted.
                extract_component(mesh, &components[index], options, candidates, out, budget, 1);
                debug_assert!(budget.work.exhausted);
            }
        }
        return;
    }
    for component in components {
        extract_component(mesh, component, options, candidates, out, budget, workers);
    }
}

fn extract_component(
    mesh: &AnalyzedMesh,
    component: &[usize],
    options: &RecognitionOptions,
    candidates: &[(Vec<usize>, AnalyticSurface)],
    out: &mut Vec<SurfaceRegion>,
    budget: &mut AttemptBudget,
    workers: usize,
) {
    if component.len() < options.minimum_support || budget.work.exhausted {
        return;
    }
    let whole_started = budget.profile.is_some().then(Instant::now);
    fit_profile_start(budget.profile.is_some());
    let whole = reconstruct_analyzed(mesh, component, &SurfaceHint::Unknown, options);
    ExtractionProfile::take_fitter(&mut budget.profile, |p| &mut p.whole_fitter);
    if let Ok(fit) = whole {
        ExtractionProfile::charge(&mut budget.profile, whole_started, |p| &mut p.whole_fit_seconds);
        out.push(to_region(fit, component.to_vec()));
        return;
    }
    if let Some(fit) = candidate_fit(mesh, component, options, candidates) {
        ExtractionProfile::charge(&mut budget.profile, whole_started, |p| &mut p.whole_fit_seconds);
        budget.work.seeded_components += 1;
        out.push(to_region(fit, component.to_vec()));
        return;
    }
    ExtractionProfile::charge(&mut budget.profile, whole_started, |p| &mut p.whole_fit_seconds);
    if let HypothesisPolicy::Adaptive(policy) = &options.hypothesis_policy {
        return extract_component_adaptive(mesh, component, options, out, budget, policy);
    }
    // CAD-aware hypothesis generation: fit small connected neighborhoods and
    // grow their geometric support through adjacency. This is deterministic
    // with a supplied seed and lets tangent analytic regions separate.
    //
    // A round's hypotheses are fixed by its seed permutations, and each one
    // reads only the round's remaining triangles, so they are evaluated in
    // batches (of one on a single worker) and applied in draw order: the
    // regions, their diagnostics and the work are those of the one-at-a-time
    // loop, including where an attempt budget cuts the round.
    let mut remaining = Remaining::new(mesh, component);
    let mut rng = SplitMix(options.deterministic_seed.unwrap_or(0x52414e534143));
    let batch_size = if workers > 1 { workers * budget.kinds.len() } else { 1 };
    let scratch: Vec<std::sync::Mutex<Scratch>> =
        (0..workers.max(1)).map(|_| std::sync::Mutex::new(Scratch::new(mesh))).collect();
    let profiling = budget.profile.is_some();
    while remaining.len() >= options.minimum_support {
        let seed_order_started = profiling.then(Instant::now);
        let ids: Vec<_> = remaining.iter().copied().collect();
        // One independently shuffled, without-replacement seed schedule per
        // primitive. At the full cap every remaining triangle is tried once
        // for every carrier; random-with-replacement sampling could waste
        // roughly a third of that budget on duplicate seeds.
        let kinds = budget.kinds.clone();
        let seed_orders: Vec<_> = kinds
            .iter()
            .map(|_| shuffled_seed_order(&ids, &mut rng))
            .collect();
        ExtractionProfile::charge(&mut budget.profile, seed_order_started, |p| &mut p.seed_order_seconds);
        let mut best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
        let attempt_cap = options.max_hypotheses.min(ids.len() * kinds.len());
        let mut attempts_required = attempt_cap;
        let mut attempt = 0;
        let mut region_growth_seconds = options.collect_phase_timings.then_some(0.0);
        while attempt < attempts_required {
            let want = batch_size.min(attempts_required - attempt);
            let admitted = budget.admit(want);
            let drawn: Vec<(usize, SurfaceType)> = (attempt..attempt + admitted)
                .map(|index| {
                    let slot = index % kinds.len();
                    (seed_orders[slot][index / kinds.len()], kinds[slot])
                })
                .collect();
            for &(_, kind) in &drawn {
                budget.work.hypotheses_by_kind[kind_index(kind)] += 1;
            }
            let incumbent = best.as_ref().map_or(0, |(count, ..)| *count);
            let evaluated = parallel_map_on_workers(workers, admitted, |worker, offset| {
                let (seed, kind) = drawn[offset];
                // Each worker locks only its own scratch, so this never waits.
                let mut scratch = scratch[worker].lock().unwrap_or_else(|poisoned| poisoned.into_inner());
                classical_hypothesis(mesh, &remaining, &mut scratch, seed, kind, options, incumbent, profiling)
            });
            let mut applied = 0;
            for (offset, (probe_best, growth_seconds, profile)) in evaluated.into_iter().enumerate() {
                let attempt_index = attempt + offset;
                if attempt_index >= attempts_required {
                    break;
                }
                applied += 1;
                if let (Some(total), Some(seconds)) = (&mut region_growth_seconds, growth_seconds) {
                    *total += seconds;
                }
                if let (Some(total), Some(profile)) = (&mut budget.profile, &profile) {
                    total.absorb(profile);
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
                        kinds.len(),
                        attempt_cap,
                    ));
                    attempts_required = attempts_required.max(attempt_index + 1);
                    best = Some((support.len(), rank, fit, support));
                }
            }
            // Hypotheses past the bound a winner in this batch shrank were
            // never drawn by the one-at-a-time loop.
            for &(_, kind) in &drawn[applied..] {
                budget.work.hypotheses_by_kind[kind_index(kind)] -= 1;
            }
            budget.refund(admitted - applied);
            attempt += applied;
            if applied < admitted {
                break;
            }
            if admitted < want {
                // The round's next hypothesis is one the limit refuses.
                budget.work.exhausted = true;
                budget.work.stops_at_budget += 1;
                return;
            }
        }
        // The round ran to the classical bound, or to the cap when the bound
        // asked for more than the cap allowed.
        if attempts_required < attempt_cap {
            budget.work.stops_at_risk_bound += 1;
        } else {
            budget.work.stops_at_hypothesis_cap += 1;
        }
        if let Some(profile) = &mut budget.profile {
            profile.rounds += 1;
            profile.seeds_consumed += attempt;
            profile.seed_slots += ids.len() * kinds.len();
            if best.is_none() {
                profile.abandon(mesh, &remaining, options);
            }
        }
        let Some((_, _, mut fit, support)) = best else {
            break;
        };
        fit.diagnostics.phase_timings.region_growth_seconds = region_growth_seconds;
        for id in &support {
            remaining.remove(id);
        }
        budget.work.rounds += 1;
        out.push(to_region(fit, support));
    }
}

/// One classical hypothesis: the probe chain around `seed` for `kind` over
/// `remaining`, returning the best candidate any probe size produced
/// (support count, rank, refit, support), the seconds spent growing support
/// when phase timings are collected, and its extraction profile when
/// `profiling` (refits measured against `incumbent`, the round's best count
/// when it was drawn). It reads nothing a round writes, so a round's
/// hypotheses can run in any order or at once.
#[allow(clippy::type_complexity, clippy::too_many_arguments)]
fn classical_hypothesis(
    mesh: &AnalyzedMesh,
    remaining: &Remaining,
    scratch: &mut Scratch,
    seed: usize,
    kind: SurfaceType,
    options: &RecognitionOptions,
    incumbent: usize,
    profiling: bool,
) -> (
    Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)>,
    Option<f64>,
    Option<ExtractionProfile>,
) {
    let mut region_growth_seconds = options.collect_phase_timings.then_some(0.0);
    let mut profile = profiling.then(ExtractionProfile::default);
    // Generate each primitive from the smallest diverse triangle set that
    // supplies its fitter's required independent observations in every
    // public sampling mode. Final support is still grown and validated
    // against the complete connected region below. Keeping the fitted sample
    // primitive-specific avoids fitting every candidate to the full
    // diversity probe while preserving aggressive rank/degeneracy rejection
    // in the fitter.
    let sample_size = hypothesis_sample_triangles(kind);
    // The probe supplies spatial/normal diversity without increasing the
    // number of triangles actually passed to a primitive fitter.
    let mut probe_limit = options
        .hypothesis_probe_triangles
        .max(sample_size)
        .min(remaining.len());
    let mut probe_best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
    let mut search = Neighborhood::start(scratch, seed);
    loop {
        let started = profiling.then(Instant::now);
        let probe = search.grow(mesh, remaining, scratch, probe_limit, probe_crease(options));
        ExtractionProfile::charge(&mut profile, started, |p| &mut p.neighborhood_seconds);
        if probe.len() < sample_size {
            break;
        }
        let started = profiling.then(Instant::now);
        let sample = diverse_hypothesis_sample(mesh, seed, &probe, sample_size);
        ExtractionProfile::charge(&mut profile, started, |p| &mut p.sample_seconds);
        let mut grew_beyond_probe = false;
        let mut reconstructed_probe = false;
        let started = profiling.then(Instant::now);
        fit_profile_start(profiling);
        let fitted = fit_surface_with_path(
            mesh,
            &sample,
            kind,
            None,
            ConstraintMask::default(),
            options,
            FitPath::GenericRecognition,
        );
        ExtractionProfile::take_fitter(&mut profile, |p| &mut p.sample_fitter);
        let fit_seconds = started.map_or(0.0, |started| started.elapsed().as_secs_f64());
        // What came of a torus sample fit (row 1): see `torus_sample_outcomes`.
        let mut torus_outcome = 0;
        if let Some(profile) = &mut profile {
            profile.fit_seconds[kind_index(kind)] += fit_seconds;
            profile.fits[kind_index(kind)] += 1;
        }
        if let Ok(candidate) = fitted {
            torus_outcome = 1;
            let growth_started = options.collect_phase_timings.then(Instant::now);
            let started = profiling.then(Instant::now);
            let mut tally = profiling.then(SupportTally::default);
            let support = support_component(
                mesh,
                remaining,
                scratch,
                seed,
                candidate.surface,
                options,
                tally.as_mut(),
            );
            ExtractionProfile::charge(&mut profile, started, |p| &mut p.support_seconds);
            if let (Some(profile), Some(tally)) = (&mut profile, tally) {
                profile.support_triangles_tested += tally.triangles;
                profile.distance_evaluations += tally.evaluations;
                profile.distinct_vertices_evaluated += tally.vertices.len();
            }
            if let (Some(total), Some(started)) = (&mut region_growth_seconds, growth_started) {
                *total += started.elapsed().as_secs_f64();
            }
            grew_beyond_probe = support.len() > probe.len();
            if support.len() >= options.minimum_support {
                let started = profiling.then(Instant::now);
                fit_profile_start(profiling);
                let refitted = reconstruct_analyzed(
                    mesh,
                    &support,
                    &SurfaceHint::InitialGuess {
                        surface: candidate.surface,
                        trust: crate::MetadataTrust::InitialGuess,
                    },
                    options,
                );
                ExtractionProfile::take_fitter(&mut profile, |p| &mut p.refit_fitter);
                torus_outcome = if refitted.is_ok() { 3 } else { 2 };
                if let Some(profile) = &mut profile {
                    let exhausted = probe.len() == remaining.len() || probe.len() < probe_limit;
                    profile.refit(started, support.len(), incumbent, !grew_beyond_probe || exhausted);
                }
                if let Ok(fit) = refitted {
                    reconstructed_probe = true;
                    let started = profiling.then(Instant::now);
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
                    let replace = probe_best.as_ref().is_none_or(|(count, old_rank, old, _)| {
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
                    ExtractionProfile::charge(&mut profile, started, |p| &mut p.rank_seconds);
                    if support.len() < incumbent {
                        ExtractionProfile::charge(&mut profile, started, |p| &mut p.rank_dominated_seconds);
                    }
                    if replace {
                        probe_best = Some((support.len(), rank, fit, support));
                    }
                }
            }
        }
        if let (Some(profile), SurfaceType::Torus) = (&mut profile, kind) {
            profile.torus_sample(torus_outcome, fit_seconds);
        }
        let exhausted = probe.len() == remaining.len() || probe.len() < probe_limit;
        if (grew_beyond_probe && reconstructed_probe) || exhausted {
            break;
        }
        // A candidate explaining no more triangles than its local probe is
        // under-observed (for example one planar strip of a finely faceted
        // cylinder). Expand only the diversity probe; every fitter call
        // remains primitive-minimal.
        probe_limit = probe_limit.saturating_mul(2).min(remaining.len());
    }
    (probe_best, region_growth_seconds, profile)
}

/// Relative cost of one generic hypothesis per kind, in units of a plane
/// hypothesis, in [`TYPES`] order. The sample fit dominates a losing
/// hypothesis, and on brep-vm2 (release, 2026-10-27, the jitter-1.0 d0.02
/// torus and the jitter-0.3 d0.02 crossing fixtures) it measured 0.01 ms for
/// a plane, 0.05 sphere, 0.13 cylinder, 0.27 cone and 9.2 torus. The bandit
/// divides a kind's reward by this instead of reading the clock, so a run
/// stays reproducible under `deterministic_seed`.
const HYPOTHESIS_COST_UNITS: [f64; 5] = [1.0, 5.0, 13.0, 27.0, 920.0];

/// Uniform index in `0..n` by rejection, as [`shuffled_seed_order`] draws.
fn uniform_index(rng: &mut SplitMix, n: usize) -> usize {
    debug_assert!(n > 0);
    let n_u64 = n as u64;
    let zone = (u64::MAX / n_u64) * n_u64;
    loop {
        let value = rng.next();
        if value < zone {
            return (value % n_u64) as usize;
        }
    }
}

/// Uniform `f64` in `[0, 1)` from the top 53 bits of one draw.
fn unit_interval(rng: &mut SplitMix) -> f64 {
    (rng.next() >> 11) as f64 / (1u64 << 53) as f64
}

/// Standard normal by Box–Muller (one of the pair).
fn standard_normal(rng: &mut SplitMix) -> f64 {
    let u = loop {
        let u = unit_interval(rng);
        if u > 0.0 {
            break u;
        }
    };
    let v = unit_interval(rng);
    (-2.0 * u.ln()).sqrt() * (2.0 * std::f64::consts::PI * v).cos()
}

/// Gamma(`shape`, 1) for `shape >= 1` by Marsaglia–Tsang.
fn gamma_sample(rng: &mut SplitMix, shape: f64) -> f64 {
    debug_assert!(shape >= 1.0);
    let d = shape - 1.0 / 3.0;
    let c = 1.0 / (9.0 * d).sqrt();
    loop {
        let x = standard_normal(rng);
        let v = 1.0 + c * x;
        if v <= 0.0 {
            continue;
        }
        let v = v * v * v;
        let u = unit_interval(rng);
        if u > 0.0 && u.ln() < 0.5 * x * x + d - d * v + d * v.ln() {
            return d * v;
        }
    }
}

/// Beta(`successes + 1`, `failures + 1`): the posterior of a rate under a
/// uniform prior.
fn beta_sample(rng: &mut SplitMix, successes: usize, failures: usize) -> f64 {
    let x = gamma_sample(rng, successes as f64 + 1.0);
    let y = gamma_sample(rng, failures as f64 + 1.0);
    x / (x + y)
}

/// The arm a bandit draws next: first any eligible arm below its floor share
/// of the `total` draws so far (the most deficient, lowest index on ties),
/// otherwise the eligible arm with the largest `score`. At least one arm must
/// be eligible.
fn choose_arm(draws: &[usize], scores: &[f64], eligible: &[bool], floor_share: f64, total: usize) -> usize {
    let arms = draws.len();
    debug_assert!(arms > 0 && scores.len() == arms && eligible.len() == arms);
    let minimum = (floor_share * (total + 1) as f64 / arms as f64).ceil() as usize;
    let deficient = (0..arms)
        .filter(|&i| eligible[i] && draws[i] < minimum)
        .min_by(|&a, &b| draws[a].cmp(&draws[b]).then(a.cmp(&b)));
    if let Some(arm) = deficient {
        return arm;
    }
    (0..arms)
        .filter(|&i| eligible[i])
        .fold(None, |best: Option<usize>, i| match best {
            Some(b) if scores[i] <= scores[b] => Some(b),
            _ => Some(i),
        })
        .expect("at least one eligible arm")
}

/// What the adaptive policy has learned about one primitive kind on a
/// component, carried from round to round (halved at each round's start, so
/// the extracted regions stop weighing on what remains).
#[derive(Clone, Copy, Debug, Default)]
struct KindPosterior {
    /// Hypotheses whose support reached `minimum_support`.
    reached: f64,
    /// Hypotheses that did the work and did not.
    failed: f64,
    /// Sum of the reached candidates' support shares of the round's triangles.
    support_share: f64,
    /// Doubling chains past a hopeless probe that later reached
    /// `minimum_support`, and chains that never did.
    rescued: f64,
    /// See `rescued`.
    unrescued: f64,
}

impl KindPosterior {
    fn halve(&mut self) {
        self.reached *= 0.5;
        self.failed *= 0.5;
        self.support_share *= 0.5;
        self.rescued *= 0.5;
        self.unrescued *= 0.5;
    }

    /// Posterior mean of the chance that doubling past a hopeless probe is
    /// rescued, under a uniform prior.
    fn rescue_rate(&self) -> f64 {
        (self.rescued + 1.0) / (self.rescued + self.unrescued + 2.0)
    }

    /// One Thompson draw of the support share a hypothesis of this kind
    /// gains: a sampled success rate times the mean support share of its
    /// successes (one pseudo-success of `prior_share` while none is seen).
    fn sample_value(&self, rng: &mut SplitMix, prior_share: f64) -> f64 {
        let success = beta_sample(rng, self.reached.round() as usize, self.failed.round() as usize);
        let mean_share = (self.support_share + prior_share) / (self.reached + 1.0);
        success * mean_share
    }
}

/// Spatial temperatures of the adaptive policy's Boltzmann seed distribution,
/// as multiples of `AdaptiveSampling::seed_temperature`; each is a bandit arm
/// ([`TemperatureArm`]). The last, infinite, is uniform over the triangles the
/// distribution ranges over, so a controller that has stopped improving at
/// the cooler arms reheats by choosing it.
const TEMPERATURE_BANK: [f64; 7] = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, f64::INFINITY];

/// Position of the infinite temperature in [`TEMPERATURE_BANK`]; the uniform
/// part of the seed mixture reports its outcomes to this arm.
const UNIFORM_ARM: usize = TEMPERATURE_BANK.len() - 1;

/// What the adaptive policy has learned about one spatial temperature on a
/// component (halved at each round's start, like [`KindPosterior`]).
#[derive(Clone, Copy, Debug, Default)]
struct TemperatureArm {
    /// Draws whose seed this arm chose.
    tried: f64,
    /// Those that replaced the round's incumbent.
    improved: f64,
    /// Sum of those replacements' gains in share of the round's triangles.
    gain: f64,
}

impl TemperatureArm {
    fn halve(&mut self) {
        self.tried *= 0.5;
        self.improved *= 0.5;
        self.gain *= 0.5;
    }

    /// Mean gain of this arm's replacements, one pseudo-replacement of
    /// `prior_gain` while none is seen.
    fn mean_gain(&self, prior_gain: f64) -> f64 {
        (self.gain + prior_gain) / (self.improved + 1.0)
    }

    /// One Thompson draw of the improvement a draw at this temperature makes
    /// to the incumbent: a sampled improvement rate (uniform prior) times the
    /// mean gain.
    fn sample_value(&self, rng: &mut SplitMix, prior_gain: f64) -> f64 {
        let improved = self.improved.round().max(0.0) as usize;
        let failed = (self.tried - self.improved).round().max(0.0) as usize;
        beta_sample(rng, improved, failed) * self.mean_gain(prior_gain)
    }
}

/// The temperature arm with the largest Thompson draw (the coolest on a tie).
fn choose_temperature(bank: &[TemperatureArm], rng: &mut SplitMix, prior_gain: f64) -> usize {
    let mut best = 0;
    let mut best_value = f64::NEG_INFINITY;
    for (arm, state) in bank.iter().enumerate() {
        let value = state.sample_value(rng, prior_gain);
        if value > best_value {
            best = arm;
            best_value = value;
        }
    }
    best
}

/// The share of the component's error budget `delta_total` a round that
/// extracted `extracted` of the component's `component_size` triangles may
/// spend: proportional to what it extracted, so that the disjoint extracted
/// regions of the certified rounds sum to at most `delta_total`.
fn round_error_share(delta_total: f64, extracted: usize, component_size: usize) -> f64 {
    delta_total * extracted as f64 / component_size as f64
}

/// One seed from the Boltzmann part of the adaptive mixture: weight
/// `exp(-explored / temperature)` over the positions of `ids` that `kind` has
/// not seeded and that are outside the incumbent; `None` when no such
/// position remains.
fn boltzmann_seed(
    rng: &mut SplitMix,
    ids: &[usize],
    tried: &[bool],
    incumbent: &[bool],
    explored: &[f64],
    temperature: f64,
) -> Option<usize> {
    let weight = |position: usize| -> f64 {
        if tried[position] || incumbent[position] {
            0.0
        } else {
            (-explored[ids[position]] / temperature).exp()
        }
    };
    let total: f64 = (0..ids.len()).map(weight).sum();
    if !(total > 0.0) {
        return None;
    }
    let mut target = unit_interval(rng) * total;
    let mut last = None;
    for position in 0..ids.len() {
        let w = weight(position);
        if w <= 0.0 {
            continue;
        }
        last = Some(position);
        if target < w {
            return Some(position);
        }
        target -= w;
    }
    last
}

/// Generic extraction under [`HypothesisPolicy::Adaptive`]. Same component
/// partition, fitters, support growth, refit and ranking as the classical
/// loop; what changes is how each hypothesis is drawn and when a round stops.
///
/// Per round over the remaining triangles `R`, `n = |R|`, `n0` the component's
/// triangle count, `w` the incumbent's share of `n` (0 before a candidate):
/// - the kind is drawn by Thompson sampling on [`KindPosterior`]: the sampled
///   support share a hypothesis of the kind gains, divided by its cost
///   ([`HYPOTHESIS_COST_UNITS`] plus the mean kind cost, the price of the
///   round slot a draw occupies whatever its kind), among the kinds some
///   remaining triangle has not seeded yet, every such kind keeping
///   `kind_floor` of the draws; the posterior carries across rounds, halved;
/// - the seed is uniform over `R` with probability `seed_exploration`,
///   otherwise Boltzmann over the unexplored triangles outside the incumbent
///   that this kind has not seeded, at a temperature drawn by Thompson
///   sampling over [`TEMPERATURE_BANK`] scaled by `seed_temperature`
///   ([`TemperatureArm`]); the uniform draws report to the infinite arm;
/// - the probe is the classical chain (128 triangles, doubled while the
///   candidate reaches `minimum_support` without outgrowing the probe), except
///   that at the first probe whose candidate did not reach `minimum_support`
///   the chain is kept, once, with probability `max(probe_floor, rescue
///   rate)` and otherwise dropped; a kept chain runs on as the classical one
///   does. The rescue rate is how often a kept chain of this kind went on to
///   reach `minimum_support`, so the chance a rescuable chain is kept equals
///   the rate however many doublings the rescue takes;
/// - after each draw the round stops for the first of these that holds, each
///   counted in [`RecognitionWork`]:
///   1. risk bound: every kind's miss bound `(1 - seed_exploration * w)^d_k`
///      (`d_k` its draws so far; 0 once every remaining triangle has seeded
///      it) is at most the round's share of the component's error budget,
///      `(1 - confidence) * |incumbent| / n0` ([`round_error_share`]). The
///      best support `B` of the round's triangles has some kind `k*` and
///      `|B| >= |incumbent|` from the first draw on, so every uniform draw of
///      kind `k*`, whenever it was made, hit a seed of `B` with probability at
///      least `w`; the probability that every draw of `k*` missed `B` is at
///      most that kind's bound (the classical rule credits its earlier draws
///      the same way), and requiring every kind's bound at or below the
///      round's share bounds the miss by that share whatever `k*` is. The
///      extracted regions are disjoint, so the certified rounds' shares sum
///      to at most `1 - confidence` over the component; rounds that stop
///      otherwise certify nothing and spend nothing. As in the classical
///      rule, this is a seed-visitation bound: a visited seed still has to
///      produce a usable hypothesis;
///   2. seed exhaustion: every remaining triangle has seeded every kind;
///   3. the cap `max_hypotheses` (or one draw per remaining triangle per
///      kind, when smaller), reached without any of the above.
///   An attempt budget cut ends the round and the component at once.
/// A draw that repeats a (seed, kind) pair already tried in the round counts
/// toward the bound and the cap and does no fitting.
fn extract_component_adaptive(
    mesh: &AnalyzedMesh,
    component: &[usize],
    options: &RecognitionOptions,
    out: &mut Vec<SurfaceRegion>,
    budget: &mut AttemptBudget,
    policy: &AdaptiveSampling,
) {
    let mut remaining = Remaining::new(mesh, component);
    let mut scratch = Scratch::new(mesh);
    let mut rng = SplitMix(options.deterministic_seed.unwrap_or(0x52414e534143));
    let kinds = budget.kinds.clone();
    let kind_count = kinds.len();
    let mean_cost =
        kinds.iter().map(|&kind| HYPOTHESIS_COST_UNITS[kind_index(kind)]).sum::<f64>() / kind_count as f64;
    let kind_cost: Vec<f64> =
        kinds.iter().map(|&kind| HYPOTHESIS_COST_UNITS[kind_index(kind)] + mean_cost).collect();
    let mut posterior = vec![KindPosterior::default(); kind_count];
    let mut bank = [TemperatureArm::default(); TEMPERATURE_BANK.len()];
    // Losing supports a triangle has sat in, over every round of this
    // component: the seed distribution's history.
    let mut explored = vec![0.0f64; mesh.triangles.len()];
    let delta_total = 1.0 - options.confidence;
    let component_size = component.len();
    let profiling = budget.profile.is_some();
    while remaining.len() >= options.minimum_support {
        let seed_order_started = profiling.then(Instant::now);
        let ids: Vec<usize> = remaining.iter().copied().collect();
        let n = ids.len();
        posterior.iter_mut().for_each(KindPosterior::halve);
        bank.iter_mut().for_each(TemperatureArm::halve);
        let mut tried: Vec<Vec<bool>> = vec![vec![false; n]; kind_count];
        let mut tried_count = vec![0usize; kind_count];
        let mut kind_draws = vec![0usize; kind_count];
        let mut incumbent = vec![false; n];
        let mut best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
        let attempt_cap = options.max_hypotheses.min(n * kind_count);
        let mut attempt = 0;
        let mut region_growth_seconds = options.collect_phase_timings.then_some(0.0);
        let mut stopped_early = false;
        ExtractionProfile::charge(&mut budget.profile, seed_order_started, |p| &mut p.seed_order_seconds);
        while attempt < attempt_cap {
            if !budget.take() {
                budget.work.stops_at_budget += 1;
                return;
            }
            let attempt_index = attempt;
            attempt += 1;
            let seed_choice_started = profiling.then(Instant::now);
            let incumbent_count = best.as_ref().map_or(0, |(count, ..)| *count);
            let w = best.as_ref().map_or(0.0, |(count, ..)| *count as f64 / n as f64);
            let prior_gain = if w > 0.0 { w } else { 1.0 / kind_count as f64 };
            let eligible: Vec<bool> = tried_count.iter().map(|&tried| tried < n).collect();
            debug_assert!(eligible.iter().any(|&open| open), "seed exhaustion leaves a kind to draw");
            let scores: Vec<f64> = posterior
                .iter()
                .zip(&kind_cost)
                .map(|(kind, cost)| kind.sample_value(&mut rng, prior_gain) / cost)
                .collect();
            let kind_slot = choose_arm(&kind_draws, &scores, &eligible, policy.kind_floor, attempt_index);
            let kind = kinds[kind_slot];
            budget.work.hypotheses_by_kind[kind_index(kind)] += 1;
            kind_draws[kind_slot] += 1;
            let (arm, position) = if unit_interval(&mut rng) < policy.seed_exploration {
                (UNIFORM_ARM, uniform_index(&mut rng, n))
            } else {
                let arm = choose_temperature(&bank, &mut rng, prior_gain);
                let position = boltzmann_seed(
                    &mut rng,
                    &ids,
                    &tried[kind_slot],
                    &incumbent,
                    &explored,
                    TEMPERATURE_BANK[arm] * policy.seed_temperature,
                )
                .unwrap_or_else(|| uniform_index(&mut rng, n));
                (arm, position)
            };
            ExtractionProfile::charge(&mut budget.profile, seed_choice_started, |p| &mut p.seed_choice_seconds);
            // The gain, in share of the round's triangles, when this draw
            // replaced the incumbent.
            let mut improvement = None;
            if !tried[kind_slot][position] {
                tried[kind_slot][position] = true;
                tried_count[kind_slot] += 1;
                let seed = ids[position];
                let sample_size = hypothesis_sample_triangles(kind);
                let mut probe_limit =
                    options.hypothesis_probe_triangles.max(sample_size).min(remaining.len());
                let mut probe_best: Option<(usize, ModelRank, SurfaceFitResult, Vec<usize>)> = None;
                let mut did_work = false;
                let mut reached_any = false;
                let mut doubled_past_hopeless = false;
                let mut search = Neighborhood::start(&mut scratch, seed);
                loop {
                    let started = profiling.then(Instant::now);
                    let probe = search.grow(mesh, &remaining, &mut scratch, probe_limit, probe_crease(options));
                    ExtractionProfile::charge(&mut budget.profile, started, |p| &mut p.neighborhood_seconds);
                    if probe.len() < sample_size {
                        break;
                    }
                    let started = profiling.then(Instant::now);
                    let sample = diverse_hypothesis_sample(mesh, seed, &probe, sample_size);
                    ExtractionProfile::charge(&mut budget.profile, started, |p| &mut p.sample_seconds);
                    let mut grew_beyond_probe = false;
                    let mut reconstructed_probe = false;
                    let mut reached = false;
                    did_work = true;
                    let started = profiling.then(Instant::now);
                    fit_profile_start(profiling);
                    let fitted = fit_surface_with_path(
                        mesh,
                        &sample,
                        kind,
                        None,
                        ConstraintMask::default(),
                        options,
                        FitPath::GenericRecognition,
                    );
                    ExtractionProfile::take_fitter(&mut budget.profile, |p| &mut p.sample_fitter);
                    let fit_seconds = started.map_or(0.0, |started| started.elapsed().as_secs_f64());
                    // What came of a torus sample fit (row 1): see `torus_sample_outcomes`.
                    let mut torus_outcome = 0;
                    if let Some(profile) = &mut budget.profile {
                        profile.fit_seconds[kind_index(kind)] += fit_seconds;
                        profile.fits[kind_index(kind)] += 1;
                    }
                    if let Ok(candidate) = fitted {
                        torus_outcome = 1;
                        let growth_started = options.collect_phase_timings.then(Instant::now);
                        let started = profiling.then(Instant::now);
                        let mut tally = profiling.then(SupportTally::default);
                        let support = support_component(
                            mesh,
                            &remaining,
                            &mut scratch,
                            seed,
                            candidate.surface,
                            options,
                            tally.as_mut(),
                        );
                        ExtractionProfile::charge(&mut budget.profile, started, |p| &mut p.support_seconds);
                        if let (Some(profile), Some(tally)) = (&mut budget.profile, tally) {
                            profile.support_triangles_tested += tally.triangles;
                            profile.distance_evaluations += tally.evaluations;
                            profile.distinct_vertices_evaluated += tally.vertices.len();
                        }
                        if let (Some(total), Some(started)) =
                            (&mut region_growth_seconds, growth_started)
                        {
                            *total += started.elapsed().as_secs_f64();
                        }
                        grew_beyond_probe = support.len() > probe.len();
                        if support.len() >= options.minimum_support {
                            reached = true;
                            let started = profiling.then(Instant::now);
                            fit_profile_start(profiling);
                            let refitted = reconstruct_analyzed(
                                mesh,
                                &support,
                                &SurfaceHint::InitialGuess {
                                    surface: candidate.surface,
                                    trust: crate::MetadataTrust::InitialGuess,
                                },
                                options,
                            );
                            ExtractionProfile::take_fitter(&mut budget.profile, |p| &mut p.refit_fitter);
                            torus_outcome = if refitted.is_ok() { 3 } else { 2 };
                            if let Some(profile) = &mut budget.profile {
                                let exhausted =
                                    probe.len() == remaining.len() || probe.len() < probe_limit;
                                profile.refit(
                                    started,
                                    support.len(),
                                    incumbent_count,
                                    !grew_beyond_probe || exhausted,
                                );
                            }
                            if let Ok(fit) = refitted {
                                reconstructed_probe = true;
                                let started = profiling.then(Instant::now);
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
                                ExtractionProfile::charge(&mut budget.profile, started, |p| &mut p.rank_seconds);
                                if support.len() < incumbent_count {
                                    ExtractionProfile::charge(&mut budget.profile, started, |p| {
                                        &mut p.rank_dominated_seconds
                                    });
                                }
                                if replace {
                                    probe_best = Some((support.len(), rank, fit, support));
                                }
                            }
                        }
                    }
                    if let (Some(profile), SurfaceType::Torus) = (&mut budget.profile, kind) {
                        profile.torus_sample(torus_outcome, fit_seconds);
                    }
                    if reached {
                        if doubled_past_hopeless && !reached_any {
                            posterior[kind_slot].rescued += 1.0;
                        }
                        reached_any = true;
                    }
                    let exhausted = probe.len() == remaining.len() || probe.len() < probe_limit;
                    if (grew_beyond_probe && reconstructed_probe) || exhausted {
                        break;
                    }
                    if !reached_any && !doubled_past_hopeless {
                        // The chain's first hopeless probe: keep the chain
                        // as often as such chains of this kind have been
                        // rescued, never below the floor. One coin decides
                        // the whole chain, so a rescue two doublings away is
                        // as likely to be kept as one a single doubling away.
                        let continue_probability =
                            policy.probe_floor.max(posterior[kind_slot].rescue_rate());
                        if unit_interval(&mut rng) >= continue_probability {
                            break;
                        }
                        doubled_past_hopeless = true;
                    }
                    probe_limit = probe_limit.saturating_mul(2).min(remaining.len());
                }
                if doubled_past_hopeless && !reached_any {
                    posterior[kind_slot].unrescued += 1.0;
                }
                if did_work {
                    if reached_any {
                        posterior[kind_slot].reached += 1.0;
                    } else {
                        posterior[kind_slot].failed += 1.0;
                    }
                }
                if let Some((count, rank, mut fit, support)) = probe_best {
                    posterior[kind_slot].support_share += count as f64 / n as f64;
                    let area = fit.metrics.supported_area;
                    let replace = best.as_ref().is_none_or(|(best_count, old_rank, old, _)| {
                        count > *best_count
                            || (count == *best_count
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
                        let previous = best.as_ref().map_or(0, |(count, ..)| *count);
                        improvement = Some(count.saturating_sub(previous) as f64 / n as f64);
                        if let Some((_, _, _, old_support)) = &best {
                            for &id in old_support {
                                explored[id] += 1.0;
                            }
                        }
                        incumbent.iter_mut().for_each(|flag| *flag = false);
                        for &id in &support {
                            if let Ok(at) = ids.binary_search(&id) {
                                incumbent[at] = true;
                            }
                        }
                        fit.diagnostics.hypotheses_generated = attempt_index + 1;
                        best = Some((count, rank, fit, support));
                    } else {
                        for &id in &support {
                            explored[id] += 1.0;
                        }
                    }
                }
            }
            bank[arm].tried += 1.0;
            if let Some(gain) = improvement {
                bank[arm].improved += 1.0;
                bank[arm].gain += gain;
            }
            if let Some((count, ..)) = &best {
                // Whichever part of the mixture fired, each uniform draw of
                // the best support's kind hit it with probability at least
                // seed_exploration * w, whenever it was made.
                let w = *count as f64 / n as f64;
                let round_delta = round_error_share(delta_total, *count, component_size);
                let certified = (0..kind_count).all(|slot| {
                    tried_count[slot] >= n
                        || (1.0 - policy.seed_exploration * w).powi(kind_draws[slot] as i32) <= round_delta
                });
                if certified {
                    budget.work.stops_at_risk_bound += 1;
                    stopped_early = true;
                    break;
                }
            }
            if tried_count.iter().all(|&count| count >= n) {
                budget.work.stops_at_seed_exhaustion += 1;
                stopped_early = true;
                break;
            }
        }
        if !stopped_early {
            budget.work.stops_at_hypothesis_cap += 1;
        }
        if let Some(profile) = &mut budget.profile {
            profile.rounds += 1;
            profile.seeds_consumed += attempt;
            profile.seed_slots += n * kind_count;
            if best.is_none() {
                profile.abandon(mesh, &remaining, options);
            }
        }
        let Some((_, _, mut fit, support)) = best else {
            break;
        };
        fit.diagnostics.phase_timings.region_growth_seconds = region_growth_seconds;
        for id in &support {
            remaining.remove(id);
        }
        budget.work.rounds += 1;
        out.push(to_region(fit, support));
    }
}

/// The supplied candidate for exactly `component`'s non-degenerate
/// triangles, validated by the ordinary gates plus the roundoff allowance.
fn candidate_fit(
    mesh: &AnalyzedMesh,
    component: &[usize],
    options: &RecognitionOptions,
    candidates: &[(Vec<usize>, AnalyticSurface)],
) -> Option<SurfaceFitResult> {
    if candidates.is_empty() {
        return None;
    }
    let mut ids = component.to_vec();
    ids.sort_unstable();
    let surface = candidates.iter().find_map(|(candidate, surface)| {
        let mut usable = candidate
            .iter()
            .copied()
            .filter(|&id| id < mesh.triangles.len() && mesh.triangles[id].area > 0.0);
        usable.by_ref().eq(ids.iter().copied()).then_some(*surface)
    })?;
    let magnitude = ids
        .iter()
        .flat_map(|&tid| mesh.triangles[tid].vertices)
        .map(|vid| {
            let p = mesh.vertices[vid];
            p.x.abs().max(p.y.abs()).max(p.z.abs())
        })
        .fold(1.0_f64, f64::max);
    let mut seeded = options.clone();
    seeded.distance_tolerance += CANDIDATE_ROUNDOFF_EPSILONS * f64::EPSILON * magnitude;
    let scale = selection_scale(mesh, &ids);
    let mut fit = evaluate_surface(mesh, &ids, surface, &seeded, FitPath::ExactCandidateReused).ok()?;
    let vertex_samples_only = matches!(seeded.sampling, crate::SamplingMode::Vertices);
    if !accepted_impl(&fit, &seeded, scale, vertex_samples_only) {
        return None;
    }
    fit.diagnostics.reason = "a supplied component carrier validated whole after the component's own whole fit was refused".into();
    Some(fit)
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

/// A component's triangles not yet extracted: ordered, for the rounds' seed
/// tables, and flat over the mesh's triangles, for the membership tests of
/// the probes and of support growth.
struct Remaining {
    ordered: BTreeSet<usize>,
    member: Vec<bool>,
}

impl Remaining {
    fn new(mesh: &AnalyzedMesh, ids: &[usize]) -> Self {
        let mut member = vec![false; mesh.triangles.len()];
        for &id in ids {
            member[id] = true;
        }
        Self {
            ordered: ids.iter().copied().collect(),
            member,
        }
    }

    fn len(&self) -> usize {
        self.ordered.len()
    }

    fn iter(&self) -> std::collections::btree_set::Iter<'_, usize> {
        self.ordered.iter()
    }

    fn contains(&self, id: usize) -> bool {
        self.member[id]
    }

    fn remove(&mut self, id: &usize) {
        self.ordered.remove(id);
        self.member[*id] = false;
    }
}

/// Per-worker scratch for the probes and support growth: epoch stamps over
/// the mesh's triangles, so a hypothesis neither allocates nor clears a set.
struct Scratch {
    probe: Vec<u32>,
    probe_epoch: u32,
    support: Vec<u32>,
    support_epoch: u32,
}

impl Scratch {
    fn new(mesh: &AnalyzedMesh) -> Self {
        Self {
            probe: vec![0; mesh.triangles.len()],
            probe_epoch: 0,
            support: vec![0; mesh.triangles.len()],
            support_epoch: 0,
        }
    }

    /// A stamp no probe triangle carries yet.
    fn probe_stamp(&mut self) -> u32 {
        if self.probe_epoch == u32::MAX {
            self.probe.fill(0);
            self.probe_epoch = 0;
        }
        self.probe_epoch += 1;
        self.probe_epoch
    }

    /// Two stamps no support triangle carries yet: rejected, then accepted.
    fn support_stamps(&mut self) -> (u32, u32) {
        if self.support_epoch >= u32::MAX - 1 {
            self.support.fill(0);
            self.support_epoch = 0;
        }
        self.support_epoch += 2;
        (self.support_epoch - 1, self.support_epoch)
    }
}

/// The breadth-first probe around a seed over the remaining triangles,
/// stopping once it holds `limit` triangles. A probe chain only raises the
/// limit, so the search resumes where it stopped instead of starting again:
/// a search to the larger limit passes through the stopped state (it pops the
/// same triangles in the same order until then), so the probe is the one a
/// fresh search returns.
struct Neighborhood {
    stamp: u32,
    visited: Vec<usize>,
    queue: VecDeque<usize>,
}

impl Neighborhood {
    fn start(scratch: &mut Scratch, seed: usize) -> Self {
        let stamp = scratch.probe_stamp();
        scratch.probe[seed] = stamp;
        Self {
            stamp,
            visited: vec![seed],
            queue: VecDeque::from([seed]),
        }
    }

    /// The probe at `limit` (no smaller than the last one asked for),
    /// sorted; with `crease` (a cosine), across no edge whose triangle
    /// normals are further apart ([`RecognitionOptions::probe_crease_gate`]).
    fn grow(
        &mut self,
        mesh: &AnalyzedMesh,
        allowed: &Remaining,
        scratch: &mut Scratch,
        limit: usize,
        crease: Option<f64>,
    ) -> Vec<usize> {
        while let Some(id) = self.queue.pop_front() {
            if self.visited.len() >= limit {
                // A fresh search to a larger limit expands this triangle next.
                self.queue.push_front(id);
                break;
            }
            for &n in mesh.triangles[id].neighbors.iter().flatten() {
                if crease.is_some_and(|cos| mesh.triangles[id].normal.dot(mesh.triangles[n].normal) < cos) {
                    continue;
                }
                if allowed.contains(n) && scratch.probe[n] != self.stamp {
                    scratch.probe[n] = self.stamp;
                    self.visited.push(n);
                    self.queue.push_back(n);
                }
            }
        }
        let mut probe = self.visited.clone();
        probe.sort_unstable();
        probe
    }
}

/// [`RecognitionOptions::probe_crease_gate`]: the cosine a probe's edge
/// normals must reach.
fn probe_crease(options: &RecognitionOptions) -> Option<f64> {
    options.probe_crease_gate.then(|| options.normal_tolerance.cos())
}

fn triangle_support(
    mesh: &AnalyzedMesh,
    id: usize,
    surface: AnalyticSurface,
    options: &RecognitionOptions,
    mut tally: Option<&mut SupportTally>,
) -> bool {
    let t = &mesh.triangles[id];
    let tolerance =
        options.distance_tolerance + options.relative_tolerance * mesh.diagonal.max(1.0);
    if let Some(tally) = tally.as_deref_mut() {
        tally.triangles += 1;
    }
    if t.vertices.iter().any(|&v| {
        if let Some(tally) = tally.as_deref_mut() {
            tally.evaluations += 1;
            tally.vertices.insert(v);
        }
        surface.signed_distance(mesh.vertices[v]).abs() > tolerance
    }) {
        return false;
    }
    let Some(normal) = surface.normal_at(t.centroid) else {
        return false;
    };
    t.normal.dot(normal).abs().clamp(-1.0, 1.0).acos() <= options.normal_tolerance
}
/// The connected component of `candidates` that `surface` supports and that
/// contains `seed`, grown through adjacency. Each candidate's support test
/// runs when the growth first reaches it, so the scan costs the component
/// and its boundary rather than every remaining triangle; the set is the
/// one a scan of every candidate followed by the same growth would return.
/// `tally`, when given, counts the distance work.
#[allow(clippy::too_many_arguments)]
fn support_component(
    mesh: &AnalyzedMesh,
    candidates: &Remaining,
    scratch: &mut Scratch,
    seed: usize,
    surface: AnalyticSurface,
    options: &RecognitionOptions,
    mut tally: Option<&mut SupportTally>,
) -> Vec<usize> {
    if !candidates.contains(seed)
        || !triangle_support(mesh, seed, surface, options, tally.as_deref_mut())
    {
        return Vec::new();
    }
    let (rejected, accepted) = scratch.support_stamps();
    let stamps = &mut scratch.support;
    stamps[seed] = accepted;
    let mut seen = vec![seed];
    let mut queue = VecDeque::from([seed]);
    while let Some(id) = queue.pop_front() {
        for &n in mesh.triangles[id].neighbors.iter().flatten() {
            if !candidates.contains(n) || stamps[n] == accepted || stamps[n] == rejected {
                continue;
            }
            if triangle_support(mesh, n, surface, options, tally.as_deref_mut()) {
                stamps[n] = accepted;
                seen.push(n);
                queue.push_back(n);
            } else {
                stamps[n] = rejected;
            }
        }
    }
    seen.sort_unstable();
    seen
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

