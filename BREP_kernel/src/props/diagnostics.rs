use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum DiagnosticSeverity {
    Info,
    Warning,
    Degraded,
    Error,
}

#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum KernelStage {
    Collect,
    Classify,
    Intersect,
    Refine,
    Fragment,
    Select,
    Sew,
    Validate,
    Export,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct DiagnosticEvent {
    pub severity: DiagnosticSeverity,
    pub stage: KernelStage,
    /// Stable machine-readable identifier suitable for tests and UI routing.
    pub code: String,
    pub message: String,
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct KernelDiagnostics {
    pub events: Vec<DiagnosticEvent>,
    pub counters: BTreeMap<String, u64>,
    pub measurements: BTreeMap<String, f64>,
}

impl KernelDiagnostics {
    pub fn event(
        &mut self,
        severity: DiagnosticSeverity,
        stage: KernelStage,
        code: impl Into<String>,
        message: impl Into<String>,
    ) {
        self.events.push(DiagnosticEvent {
            severity,
            stage,
            code: code.into(),
            message: message.into(),
        });
    }

    pub fn count(&mut self, code: impl Into<String>) {
        *self.counters.entry(code.into()).or_default() += 1;
    }

    pub fn count_n(&mut self, code: impl Into<String>, amount: u64) {
        *self.counters.entry(code.into()).or_default() += amount;
    }

    pub fn measure_max(&mut self, code: impl Into<String>, value: f64) {
        let entry = self.measurements.entry(code.into()).or_insert(value);
        *entry = entry.max(value);
    }

    pub fn worst(&self) -> Option<DiagnosticSeverity> {
        self.events.iter().map(|event| event.severity).max()
    }

    pub fn shippable(&self) -> bool {
        self.events
            .iter()
            .all(|event| event.severity < DiagnosticSeverity::Error)
    }
}

/// A kernel REFUSAL: the fail-safe contract's carrier. The kernel never
/// degrades a result; where it cannot certify one it refuses, and this is the
/// refusal — a closed [`RefusalClass`] that consumers dispatch on (the boolean's
/// perturbation-retry gate, the CI baseline's per-class diff), the stage that
/// minted it, and the human text, which `Display` prints verbatim so every
/// message a user or a test saw before typing is unchanged.
///
/// There is deliberately no `From<String>` for this type: a refusal is minted
/// at its origin with a class, never re-derived from prose. The only string
/// conversion is the lossy exit (`From<KernelRefusal> for String`) that
/// stringly callers above the boolean stack still use; those callers are
/// retired stack by stack (offset/fillet/heal, then the feature pipeline).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct KernelRefusal {
    #[serde(flatten)]
    pub class: RefusalClass,
    pub stage: KernelStage,
    /// The human text, verbatim. `Display` prints exactly this.
    pub message: String,
}

/// The closed set of refusal classes. Dispatchers match on it exhaustively;
/// adding a variant is a deliberate taxonomy change, never a message edit.
///
/// The first five are ARRANGEMENT DEGENERACIES — a coincident or near-tangent
/// carrier pair made the exact arrangement structurally inconsistent — and are
/// the only classes the boolean's Simulation-of-Simplicity retry may act on,
/// because their [`Backing`] is undecided by a degeneracy
/// ([`RefusalClass::backing`], [`RefusalClass::perturbation_eligible`]). The
/// rest are honest refusals that a perturbation cannot and must not "fix".
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "class", rename_all = "snake_case")]
pub enum RefusalClass {
    /// The assembled shell has open edges / invalid topology (the arrangement
    /// left a boundary unclosed).
    DegenerateArrangement { open_edges: u32, issues: u32 },
    /// Euler characteristic did not yield an integral genus.
    NonIntegralGenus { shells: u32, euler: i64 },
    /// The assembled solid's signed volume is not positive.
    NonPositiveVolume,
    /// A singular / tangent-node surface intersection the marcher declines.
    TangentNodeSingularity,
    /// The FINAL validation of the result found topology issues.
    InvalidResultTopology { issues: u32 },
    /// The operation produced no boundary faces while contact / graze
    /// evidence exists — refusing rather than blessing an empty result.
    ConservativeEmptyOverlap,
    /// An iterative lane (edge conformance, an SSI march, a Newton polish) did
    /// not converge within its budget.
    NonConvergence { what: String },
    /// A named deferral: geometry the kernel does not model yet.
    UnsupportedGeometry { what: String },
    /// A caller error: bad ids, non-finite parameters, an unusable policy.
    InvalidInput { what: String },
    /// The operation, as specified, has no single answer and the input does
    /// not pick one: a turned face part of which moves into the body and part
    /// out, a carrier whose side the geometry does not decide, several drops
    /// of a crossing that all rebuild sound. Refusing rather than choosing;
    /// `what` is the origin's stable slug.
    IllPosed { what: String },
    /// The soundness scan convicted the RESULT a lane built — a face folded
    /// through itself, two faces crossing, a hole loop that left its face —
    /// and the repair that owns the defect declined it. Geometry, not
    /// incidence: `validate()`'s verdict is [`Self::InvalidResultTopology`].
    UnsoundResult {
        defect: SoundnessDefect,
        faces: Vec<u64>,
    },
    /// The long tail — an internal consistency check tripped. Curated
    /// batteries assert zero of these among their expected refusals.
    Internal { what: String },
}

/// What a refusal's evidence establishes about the claim every refusal
/// judges: "this input has a result the kernel can certify". A refusal is
/// either REFUTED — a witness in the input itself disproves the claim, and no
/// re-run of the construction can change that — or UNDECIDED — the
/// construction failed to certify and the input has not been shown to be at
/// fault. Retry policy reads this, never a list of variants: a refuted claim
/// is never retried, and only one undecided cause is one a perturbation
/// addresses ([`UndecidedCause::Degeneracy`]).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Backing {
    /// The input refutes the claim: a bad parameter, geometry the kernel does
    /// not model, an operation that as specified has no single answer.
    Refuted,
    /// The available evidence decides nothing; `cause` names what was missing.
    Undecided { cause: UndecidedCause },
}

/// Why an undecided refusal is undecided — what the evidence lacked. The
/// boolean's Simulation-of-Simplicity retry acts on exactly one of these.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum UndecidedCause {
    /// A coincident or near-tangent carrier pair made the exact arrangement
    /// structurally inconsistent. A rigid perturbation of one operand lifts the
    /// coincidence, so this is the ONE cause a retry addresses.
    Degeneracy,
    /// Contact / graze evidence with no boundary faces. Undecided, but a nudge
    /// would MANUFACTURE the answer (a sliver, or nothing) rather than reveal
    /// it, so it is never retried.
    Contact,
    /// An iterative lane ran out of budget.
    Budget,
    /// An internal consistency check tripped — a defect, kept visible rather
    /// than retried.
    Defect,
}

/// What the soundness scan found in a result it refused.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SoundnessDefect {
    /// One face carries two points of its own domain to the same 3D point.
    Fold,
    /// Two faces of the shell cross each other.
    Crossing,
    /// A hole loop is no longer contained by the face that carries it.
    HoleBreach,
    /// A CLOSED shell's trims enclose a vector area no pcurve fit at the
    /// refinement floor accounts for (`shell_vector_areas`): two faces that
    /// share an edge are trimmed at different curves, and the sliver between
    /// them leaves the body's volume dependent on the integration reference.
    /// Drawn by the STEP import's closure gate (`io/step_import/builder/closure.rs`).
    VectorArea,
}

/// A MEASURED approximation a SUCCESSFUL result carries: the result stands,
/// and this says by how much, and why, it is not exact. Distinct from a
/// refusal (the result does not stand) and from a fulfilment (what was done of
/// what was asked) — the typed-refusal plan's third kind of report.
///
/// The one producer today is the STEP import's shell-closure gate
/// (`io/step_import/builder/closure.rs`): a body whose closed shell's trims
/// enclose more vector area than the pcurve fit floor accounts for, but less
/// than the size-relative cap that refuses it, is imported and carries this.
/// `volume_bound` is what the measurement implies for the body's volume: an
/// open shell's divergence integral reads `V − R·A/3` about a reference `R`,
/// so two references inside the body disagree by up to `|A| × diagonal / 3`,
/// which is the band inside which no oracle can say the volume is wrong.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Approximation {
    /// Stable code, e.g. `import.shell_closure`.
    pub code: String,
    /// The body it is about, by the name the consumer knows it under: the
    /// import's file-order index until the feature names it, then the scene
    /// name.
    pub body: String,
    /// The measured quantity (for `import.shell_closure`, the shell's vector
    /// area residual, mm²) and the bar it is read against.
    pub measured: f64,
    pub bar: f64,
    /// The bound the measurement puts on the body's VOLUME, mm³ — the
    /// reference-lever term above. `None` when the code does not imply one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub volume_bound: Option<f64>,
    /// The edges that carry the approximation, worst first, with their origin.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub edges: Vec<OffCarrierEdge>,
    /// Why a construction REQUEST went unmet (`blend.wall_model`), or why a
    /// blend wall shipped from a rung that never met its acceptance bars
    /// (`blend.wall_acceptance`), and the budget it spent. `None` for every
    /// other code.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub budget: Option<ApproximationBudget>,
    /// The human text, verbatim.
    pub message: String,
}

/// The spent budget of an unmet construction request, typed: why refinement
/// stopped, and the rounds and stations it used against their limits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ApproximationBudget {
    pub reason: BudgetReason,
    /// The budget steps ATTEMPTED, of the kind `mechanism` names: local
    /// refinement rounds (acceptance and request share them), or an open
    /// edge's global station rungs (past the accepted rung for a request,
    /// past the first rung for an unaccepted one). On `Rejected` this counts
    /// the refused attempt, one past the rung that ships.
    pub rounds_used: usize,
    pub rounds_limit: usize,
    /// The stations of the rung that SHIPS (a closed chain: the largest count
    /// of any one segment), against the ceiling — never the refused attempt's.
    pub stations: usize,
    pub station_limit: usize,
    /// What `rounds_used` / `rounds_limit` count.
    #[serde(default)]
    pub mechanism: BudgetMechanism,
    /// What the approximation's `measured` value is the reading OF: the wall
    /// between the rails (the default, what every report meant before), or
    /// the support rails, when they are the worse part of a request that
    /// covers both.
    #[serde(default)]
    pub measured_component: MeasuredComponent,
    /// Intervals the request could NOT read. When non-zero the true worst
    /// reading is UNKNOWN and `measured` is only the largest readable value;
    /// no consumer may read it as the worst.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub unread: usize,
}

fn is_zero(count: &usize) -> bool {
    *count == 0
}

/// What a request report's `measured` value is the reading of.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeasuredComponent {
    /// The fillet wall between its rails, against the rolling ball's sweep.
    #[default]
    Wall,
    /// The support rails (the shipped support curves), against their exact
    /// construction: their carriers or the exact ball contacts.
    Rails,
}

/// The budget mechanism an [`ApproximationBudget`] counts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BudgetMechanism {
    /// Local refinement rounds inside one rung (a chain's or closed edge's
    /// wall request).
    #[default]
    LocalRounds,
    /// Whole-march station rungs (an open edge's ladder: the uniform station
    /// count doubling, each rung with its own local rail refinement, which
    /// these counts do not include).
    StationRungs,
}

/// Why a construction request stopped short of what it asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BudgetReason {
    /// Every refinement round was spent.
    RoundsSpent,
    /// Another round would pass the per-segment station ceiling.
    StationCeiling,
    /// A refining station could not be placed coherently (it did not reach
    /// its retained neighbour's branch, or its continuation did not converge),
    /// so the last accepted rung stands.
    Incoherent,
    /// The refined rung was built and then failed its own acceptance or
    /// refused, so the last accepted rung stands.
    Rejected,
    /// No rung met its acceptance bars; the top rung ships as built
    /// (`blend.wall_acceptance`, its worst reading against its bar).
    Unaccepted,
    /// No rung was accepted and a reading acceptance needs could not be taken
    /// (non-finite, or no shipped coverage); the top rung ships as built.
    Unread,
}

/// One edge whose built curve stands off one of its carriers, as an
/// [`Approximation`] names it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OffCarrierEdge {
    /// The built edge's id in the body.
    pub edge_id: u64,
    /// The file's `EDGE_CURVE.edge_geometry` entity, when the edge has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub curve_ref: Option<usize>,
    /// The `ADVANCED_FACE` and surface entities of the carrier it stands off.
    pub face_ref: usize,
    pub surface_ref: usize,
    /// How far, mm, over the gate's stations (an upper bound: the projector's
    /// distance never under-reads).
    pub off_carrier_mm: f64,
    /// Whose curve stands off: the FILE's, as written, or one an importer
    /// pass moved (`reconciled`: `reconcile_edges_onto_surfaces` replaced it
    /// with its projection onto a different carrier).
    pub origin: EdgeOrigin,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EdgeOrigin {
    File,
    Reconciled,
}

impl RefusalClass {
    /// The backing of every class, exhaustively: a variant added without a
    /// row here does not compile, so no class is retryable or not "by whoever
    /// remembers to edit the list".
    pub fn backing(&self) -> Backing {
        use UndecidedCause::*;
        match self {
            Self::DegenerateArrangement { .. }
            | Self::NonIntegralGenus { .. }
            | Self::NonPositiveVolume
            | Self::TangentNodeSingularity
            | Self::InvalidResultTopology { .. } => Backing::Undecided { cause: Degeneracy },
            Self::ConservativeEmptyOverlap => Backing::Undecided { cause: Contact },
            Self::NonConvergence { .. } => Backing::Undecided { cause: Budget },
            Self::Internal { .. } | Self::UnsoundResult { .. } => {
                Backing::Undecided { cause: Defect }
            }
            Self::UnsupportedGeometry { .. }
            | Self::InvalidInput { .. }
            | Self::IllPosed { .. } => Backing::Refuted,
        }
    }

    /// Whether the boolean's perturbation retry may act on this refusal: an
    /// UNDECIDED refusal whose cause is an arrangement degeneracy, read off
    /// [`RefusalClass::backing`]. The resulting set is pinned by
    /// `refusal_retry_parity` against the former substring matcher and by
    /// `backing_reproduces_the_retry_set` below; moving a class between
    /// backings is a deliberate change.
    pub fn perturbation_eligible(&self) -> bool {
        matches!(
            self.backing(),
            Backing::Undecided {
                cause: UndecidedCause::Degeneracy
            }
        )
    }

    /// The snake_case tag serde emits — the name CI baselines and logs use.
    pub fn tag(&self) -> &'static str {
        match self {
            Self::DegenerateArrangement { .. } => "degenerate_arrangement",
            Self::NonIntegralGenus { .. } => "non_integral_genus",
            Self::NonPositiveVolume => "non_positive_volume",
            Self::TangentNodeSingularity => "tangent_node_singularity",
            Self::InvalidResultTopology { .. } => "invalid_result_topology",
            Self::ConservativeEmptyOverlap => "conservative_empty_overlap",
            Self::NonConvergence { .. } => "non_convergence",
            Self::UnsupportedGeometry { .. } => "unsupported_geometry",
            Self::InvalidInput { .. } => "invalid_input",
            Self::IllPosed { .. } => "ill_posed",
            Self::UnsoundResult { .. } => "unsound_result",
            Self::Internal { .. } => "internal",
        }
    }
}

impl KernelRefusal {
    pub fn new(class: RefusalClass, stage: KernelStage, message: impl Into<String>) -> Self {
        Self {
            class,
            stage,
            message: message.into(),
        }
    }

    /// An internal consistency failure. `what` is a short stable slug (the
    /// check that tripped), the message the full text.
    pub fn internal(
        stage: KernelStage,
        what: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::new(RefusalClass::Internal { what: what.into() }, stage, message)
    }

    /// A caller error.
    pub fn input(stage: KernelStage, what: impl Into<String>, message: impl Into<String>) -> Self {
        Self::new(
            RefusalClass::InvalidInput { what: what.into() },
            stage,
            message,
        )
    }

    /// A named deferral.
    pub fn unsupported(
        stage: KernelStage,
        what: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::new(
            RefusalClass::UnsupportedGeometry { what: what.into() },
            stage,
            message,
        )
    }

    /// An operation that, as specified, has no single answer.
    pub fn ill_posed(
        stage: KernelStage,
        what: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::new(RefusalClass::IllPosed { what: what.into() }, stage, message)
    }

    /// A result the soundness scan convicted.
    pub fn unsound(
        stage: KernelStage,
        defect: SoundnessDefect,
        faces: Vec<u64>,
        message: impl Into<String>,
    ) -> Self {
        Self::new(RefusalClass::UnsoundResult { defect, faces }, stage, message)
    }

    /// An iterative lane that ran out of budget.
    pub fn non_convergence(
        stage: KernelStage,
        what: impl Into<String>,
        message: impl Into<String>,
    ) -> Self {
        Self::new(
            RefusalClass::NonConvergence { what: what.into() },
            stage,
            message,
        )
    }

    /// The same refusal with its message rewritten by `f` — the way a wrapping
    /// site adds context ("assembly failed: …") WITHOUT changing the class, so
    /// an inner degeneracy stays retry-eligible through every wrapper exactly
    /// as the substring matcher saw it in the wrapped text.
    pub fn with_message(mut self, f: impl FnOnce(&str) -> String) -> Self {
        self.message = f(&self.message);
        self
    }
}

impl std::fmt::Display for KernelRefusal {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.message)
    }
}

impl std::error::Error for KernelRefusal {}

/// The lossy exit for stringly callers: the class is dropped, the text kept.
/// Legal only ABOVE the typed stacks (never inside `csg/` or the healing
/// modules that mint refusals); each stack that adopts `KernelRefusal`
/// removes its uses.
impl From<KernelRefusal> for String {
    fn from(refusal: KernelRefusal) -> Self {
        refusal.message
    }
}

/// Mechanical conversion of a stringly LOWER-layer error consumed inside a
/// typed stack (geometry evaluation, tolerance policy checks): the text is
/// kept and the refusal is classed `Internal` under `what` at `stage`. This is
/// a constructor at the consuming site, not a blanket `From`.
pub trait OrRefuse<T> {
    fn or_refuse(self, stage: KernelStage, what: &'static str) -> Result<T, KernelRefusal>;
    fn or_input(self, stage: KernelStage, what: &'static str) -> Result<T, KernelRefusal>;
}

impl<T> OrRefuse<T> for Result<T, String> {
    fn or_refuse(self, stage: KernelStage, what: &'static str) -> Result<T, KernelRefusal> {
        self.map_err(|message| KernelRefusal::internal(stage, what, message))
    }

    fn or_input(self, stage: KernelStage, what: &'static str) -> Result<T, KernelRefusal> {
        self.map_err(|message| KernelRefusal::input(stage, what, message))
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct KernelOutcome<T> {
    pub value: T,
    pub diagnostics: KernelDiagnostics,
}

