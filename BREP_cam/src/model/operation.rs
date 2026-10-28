//! Operations: one typed parameter set per strategy.
//!
//! `Strategy` is `#[non_exhaustive]`: consumers outside this crate match with
//! a wildcard arm. Inside the crate every match is exhaustive, so adding a
//! variant is a deliberate change reviewed with the lanes that match on it.
//! Every parameter struct derives `Default` and tags every field
//! `#[serde(default)]` so documents saved before a field existed still load.

use super::{FeedsSpeeds, Heights, OperationId, Point3, SetupId, TargetSelection, ToolId, WorkOffset};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// One machining operation inside a setup.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Operation {
    pub id: OperationId,
    #[serde(default)]
    pub name: String,
    pub setup: SetupId,
    pub tool: ToolId,
    /// Disabled operations are kept but neither generated nor posted.
    #[serde(default = "default_true")]
    pub enabled: bool,
    #[serde(default)]
    pub targets: TargetSelection,
    pub strategy: Strategy,
    /// Machining tolerance (chordal deviation of the path from the exact surface).
    #[serde(default = "default_tolerance")]
    pub tolerance: f64,
    #[serde(default)]
    pub stock_to_leave: StockToLeave,
    /// Overrides the tool's defaults when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feeds: Option<FeedsSpeeds>,
    /// Overrides the setup's heights when set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub heights: Option<Heights>,
    #[serde(default)]
    pub linking: LinkingParams,
    /// Fixed tool orientation for 3+2 work, in SETUP machine coordinates (`None`: +Z).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_axis: Option<Point3>,
    /// None selects live tooling automatically for milling on a mill-turn machine.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub live_tool: Option<bool>,
    /// Post-processor properties specific to this operation (free-form, ordered).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub post_properties: BTreeMap<String, String>,
    /// Feed optimisation and smoothing applied after generation (lane C18); off by default.
    #[serde(default)]
    pub polish: PolishParams,
}

fn default_true() -> bool {
    true
}

fn default_tolerance() -> f64 {
    0.01
}

impl Default for Operation {
    fn default() -> Self {
        Self::new(OperationId::default(), "", SetupId::default(), ToolId::default(), Strategy::default())
    }
}

impl Operation {
    pub fn new(
        id: impl Into<OperationId>,
        name: impl Into<String>,
        setup: impl Into<SetupId>,
        tool: impl Into<ToolId>,
        strategy: Strategy,
    ) -> Self {
        Self {
            id: id.into(),
            name: name.into(),
            setup: setup.into(),
            tool: tool.into(),
            enabled: true,
            targets: TargetSelection::default(),
            strategy,
            tolerance: default_tolerance(),
            stock_to_leave: StockToLeave::default(),
            feeds: None,
            heights: None,
            linking: LinkingParams::default(),
            tool_axis: None,
            live_tool: None,
            post_properties: BTreeMap::new(),
            polish: PolishParams::default(),
        }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !self.tolerance.is_finite() || self.tolerance <= 0.0 {
            return Err("machining tolerance must be finite and positive".into());
        }
        self.targets.validate()?;
        if self.enabled
            && self.targets.target_solids.is_empty()
            && self.targets.chains.is_empty()
            && !self.strategy.is_turning()
            && !self.strategy.is_targetless()
        {
            return Err("an enabled operation needs at least one target body".into());
        }
        if let Some(axis) = self.tool_axis {
            if axis.normalized().is_none() {
                return Err("tool axis must be a non-zero vector".into());
            }
        }
        if let Some(f) = &self.feeds {
            f.validate()?;
        }
        if let Some(h) = &self.heights {
            h.validate()?;
        }
        self.stock_to_leave.validate()?;
        self.linking.validate()?;
        self.polish.validate()?;
        self.strategy.validate()
    }
}

/// One enum-valued JSON field: its dotted path and the names it may take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EnumChoice {
    pub path: &'static str,
    pub variants: &'static [&'static str],
}

impl Operation {
    /// Enum-valued fields of an operation's JSON outside `strategy`.
    pub const ENUM_CHOICES: &'static [EnumChoice] = &[
        EnumChoice { path: "linking.retractPolicy", variants: RetractPolicy::VARIANTS },
        EnumChoice { path: "linking.leadIn.kind", variants: LeadKind::VARIANTS },
        EnumChoice { path: "linking.leadOut.kind", variants: LeadKind::VARIANTS },
        EnumChoice { path: "linking.ramp.kind", variants: RampKind::VARIANTS },
        EnumChoice { path: "feeds.spindleDirection", variants: super::SpindleDirection::VARIANTS },
        EnumChoice { path: "targets.chains[].source.kind", variants: super::ChainSource::VARIANTS },
        EnumChoice { path: "targets.chains[].side", variants: super::ChainSide::VARIANTS },
    ];
}

/// Feed optimisation and smoothing of a generated toolpath (lane C18).
/// Disabled settings are inert: their values are not validated.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct PolishParams {
    pub feed_optimization: FeedOptimizationParams,
    pub smoothing: SmoothingParams,
}

impl PolishParams {
    pub fn validate(&self) -> Result<(), String> {
        self.feed_optimization.validate()?;
        self.smoothing.validate()
    }
}

/// Slow the feed at corners (travel direction change above `max_angle_deg`,
/// either turn sense) and through arcs of radius up to `max_radius`, over
/// `distance` of path length on either side, to `reduced_feed_ratio` of the
/// cutting feed.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct FeedOptimizationParams {
    pub enabled: bool,
    pub max_angle_deg: f64,
    pub max_radius: f64,
    pub reduced_feed_ratio: f64,
    pub distance: f64,
}

impl Default for FeedOptimizationParams {
    fn default() -> Self {
        Self { enabled: false, max_angle_deg: 30.0, max_radius: 2.0, reduced_feed_ratio: 0.5, distance: 2.0 }
    }
}

impl FeedOptimizationParams {
    pub fn validate(&self) -> Result<(), String> {
        if !self.enabled {
            return Ok(());
        }
        if !self.max_angle_deg.is_finite() || self.max_angle_deg < 0.0 || self.max_angle_deg > 180.0 {
            return Err("feed optimisation angle must be finite and within [0, 180] degrees".into());
        }
        positive_or_zero("feed optimisation radius", self.max_radius)?;
        if !self.reduced_feed_ratio.is_finite() || self.reduced_feed_ratio <= 0.0 || self.reduced_feed_ratio > 1.0 {
            return Err("reduced feed ratio must be within (0, 1]".into());
        }
        positive_or_zero("feed optimisation distance", self.distance)
    }
}

/// Smooth the path within `tolerance`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SmoothingParams {
    pub enabled: bool,
    pub tolerance: f64,
}

impl Default for SmoothingParams {
    fn default() -> Self {
        Self { enabled: false, tolerance: 0.01 }
    }
}

impl SmoothingParams {
    pub fn validate(&self) -> Result<(), String> {
        if self.enabled {
            positive("smoothing tolerance", self.tolerance)?;
        }
        Ok(())
    }
}

/// Material intentionally left for a later operation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct StockToLeave {
    pub radial: f64,
    pub axial: f64,
}

impl StockToLeave {
    pub fn validate(&self) -> Result<(), String> {
        if !self.radial.is_finite() || !self.axial.is_finite() {
            return Err("stock to leave is not finite".into());
        }
        Ok(())
    }
}

/// How passes are connected and how the tool enters and leaves material.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct LinkingParams {
    pub retract_policy: RetractPolicy,
    /// Links shorter than this stay down at feed instead of retracting.
    pub keep_down_distance: f64,
    pub lead_in: Lead,
    pub lead_out: Lead,
    pub ramp: Ramp,
    /// Fit arcs (G2/G3) to the output where the path allows.
    pub fit_arcs: bool,
    /// Smooth the path within this deviation (`0`: off).
    pub smoothing_tolerance: f64,
    /// Minimum distance kept above checked material by minimum retracts and feed heights.
    pub safe_distance: f64,
}

impl Default for LinkingParams {
    fn default() -> Self {
        Self {
            retract_policy: RetractPolicy::MinimumRetract,
            keep_down_distance: 0.0,
            lead_in: Lead::default(),
            lead_out: Lead::default(),
            ramp: Ramp::default(),
            fit_arcs: true,
            smoothing_tolerance: 0.0,
            safe_distance: 2.0,
        }
    }
}

impl LinkingParams {
    pub fn validate(&self) -> Result<(), String> {
        if !self.keep_down_distance.is_finite() || self.keep_down_distance < 0.0 {
            return Err("keep-down distance must be finite and non-negative".into());
        }
        if !self.smoothing_tolerance.is_finite() || self.smoothing_tolerance < 0.0 {
            return Err("smoothing tolerance must be finite and non-negative".into());
        }
        if !self.safe_distance.is_finite() || self.safe_distance < 0.0 {
            return Err("safe distance must be finite and non-negative".into());
        }
        self.lead_in.validate()?;
        self.lead_out.validate()?;
        self.ramp.validate()
    }
}

/// Where the tool retracts to between passes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RetractPolicy {
    /// Always up to the clearance height.
    FullRetract,
    /// Up to the retract height.
    RetractHeight,
    /// Only as high as needed to clear checked material (plus feed height).
    #[default]
    MinimumRetract,
}

impl RetractPolicy {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["full-retract", "retract-height", "minimum-retract"];
}

/// Lead-in / lead-out geometry.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Lead {
    pub kind: LeadKind,
    /// Arc radius or linear length.
    pub radius: f64,
    /// Arc sweep in degrees (arc leads).
    pub sweep_deg: f64,
    /// Straight extension before the arc.
    pub linear_length: f64,
    /// Vertical component of the lead.
    pub vertical_radius: f64,
}

impl Default for Lead {
    fn default() -> Self {
        Self { kind: LeadKind::None, radius: 0.0, sweep_deg: 90.0, linear_length: 0.0, vertical_radius: 0.0 }
    }
}

impl Lead {
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [
            ("lead radius", self.radius),
            ("lead sweep", self.sweep_deg),
            ("lead linear length", self.linear_length),
            ("lead vertical radius", self.vertical_radius),
        ] {
            if !v.is_finite() || v < 0.0 {
                return Err(format!("{name} must be finite and non-negative"));
            }
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum LeadKind {
    #[default]
    None,
    Arc,
    Linear,
    Vertical,
}

impl LeadKind {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["none", "arc", "linear", "vertical"];
}

/// How the tool descends into material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Ramp {
    pub kind: RampKind,
    /// Maximum ramp angle in degrees.
    pub angle_deg: f64,
    /// Helix diameter (helix ramps).
    pub helix_diameter: f64,
    /// Clearance above the floor where the ramp levels out.
    pub clearance: f64,
}

impl Default for Ramp {
    fn default() -> Self {
        Self { kind: RampKind::Plunge, angle_deg: 2.0, helix_diameter: 0.0, clearance: 0.0 }
    }
}

impl Ramp {
    pub fn validate(&self) -> Result<(), String> {
        if !self.angle_deg.is_finite() || self.angle_deg < 0.0 || self.angle_deg > 90.0 {
            return Err("ramp angle must be between 0 and 90 degrees".into());
        }
        if !self.helix_diameter.is_finite() || self.helix_diameter < 0.0 {
            return Err("helix diameter must be finite and non-negative".into());
        }
        if !self.clearance.is_finite() || self.clearance < 0.0 {
            return Err("ramp clearance must be finite and non-negative".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RampKind {
    #[default]
    Plunge,
    Helix,
    Profile,
    ZigZag,
    Smooth,
}

impl RampKind {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["plunge", "helix", "profile", "zig-zag", "smooth"];
}

/// Depth control shared by 2.5D and roughing strategies. Heights are MACHINE Z
/// unless `relative_to_stock` is set.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Depths {
    /// Top of cut (`None`: stock top).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top: Option<f64>,
    /// Bottom of cut (`None`: model bottom / selected floor).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottom: Option<f64>,
    /// Maximum roughing step-down (`0`: one depth).
    pub step_down: f64,
    /// Finishing step-down applied to the last `finishing_passes` depths.
    pub finishing_step_down: f64,
    pub finishing_passes: u32,
    /// Order: every depth of one region before moving on (`true`) or every
    /// region at one depth before the next depth (`false`).
    pub depth_first: bool,
}

impl Default for Depths {
    fn default() -> Self {
        Self {
            top: None,
            bottom: None,
            step_down: 0.0,
            finishing_step_down: 0.0,
            finishing_passes: 0,
            depth_first: true,
        }
    }
}

impl Depths {
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [("step down", self.step_down), ("finishing step down", self.finishing_step_down)] {
            if !v.is_finite() || v < 0.0 {
                return Err(format!("{name} must be finite and non-negative"));
            }
        }
        if let (Some(t), Some(b)) = (self.top, self.bottom) {
            if !t.is_finite() || !b.is_finite() || t < b {
                return Err("top of cut must be above bottom of cut".into());
            }
        }
        Ok(())
    }
}

/// Radial stepover between adjacent passes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Stepover {
    /// Distance between passes in model units.
    pub distance: f64,
    /// Climb (default) or conventional milling.
    pub direction: CutDirection,
}

impl Default for Stepover {
    fn default() -> Self {
        Self { distance: 1.0, direction: CutDirection::Climb }
    }
}

impl Stepover {
    pub fn validate(&self) -> Result<(), String> {
        if !self.distance.is_finite() || self.distance <= 0.0 {
            return Err("stepover must be finite and positive".into());
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CutDirection {
    #[default]
    Climb,
    Conventional,
    /// Alternate every pass (zig-zag).
    Either,
}

impl CutDirection {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["climb", "conventional", "either"];
}

/// Which way a boundary limits the toolpath.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ContainmentMode {
    /// Tool centre stays inside the boundary.
    #[default]
    ToolCenterInside,
    /// Tool stays fully inside the boundary.
    ToolInside,
    /// Tool may reach outside until its far side touches the boundary.
    ToolOutside,
}

impl ContainmentMode {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["tool-center-inside", "tool-inside", "tool-outside"];
}

/// Region limits for 3D strategies.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Containment {
    /// Closed 2D boundary loops in machine XY (`empty`: the target silhouette).
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub boundary: Vec<Vec<super::Point2>>,
    pub mode: ContainmentMode,
    /// Extra offset applied to the boundary (positive grows it).
    pub offset: f64,
    /// Only machine surface slopes within this range, degrees from horizontal.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub slope_deg: Option<[f64; 2]>,
    /// Only machine between these machine Z levels.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_range: Option<[f64; 2]>,
}

impl Containment {
    pub fn validate(&self) -> Result<(), String> {
        if !self.offset.is_finite() {
            return Err("containment offset is not finite".into());
        }
        if let Some([lo, hi]) = self.slope_deg {
            if !lo.is_finite() || !hi.is_finite() || lo < 0.0 || hi > 90.0 || lo > hi {
                return Err("slope range must lie within 0..90 degrees".into());
            }
        }
        if let Some([lo, hi]) = self.z_range {
            if !lo.is_finite() || !hi.is_finite() || lo > hi {
                return Err("z range must have min <= max".into());
            }
        }
        for loop_ in &self.boundary {
            if loop_.len() < 3 {
                return Err("a containment boundary loop needs at least three points".into());
            }
            if loop_.iter().any(|p| !p.is_finite()) {
                return Err("a containment boundary point is not finite".into());
            }
        }
        Ok(())
    }
}

/// Drilling cycle selection.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum DrillCycle {
    /// Simple drilling (G81).
    #[default]
    Drill,
    /// Drilling with dwell at depth (G82).
    DrillDwell,
    /// Deep drilling with full retract (G83).
    Peck,
    /// Chip breaking with short retract (G73).
    ChipBreak,
    /// Rigid tapping (G84 with synchronous feed).
    TapRigid,
    /// Floating-holder tapping (G84).
    TapFloat,
    /// Reaming: feed in, feed out (G85).
    Ream,
    /// Boring with spindle stop and shift (G76).
    BoreFine,
    /// Boring, stop spindle at bottom, rapid out (G86).
    Bore,
    /// Back boring (G87).
    BackBore,
    /// Boring with dwell and feed out (G89).
    BoreDwell,
}

impl DrillCycle {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["drill", "drill-dwell", "peck", "chip-break", "tap-rigid", "tap-float", "ream", "bore-fine", "bore", "back-bore", "bore-dwell"];
}

/// The strategy and its parameters. Variants beyond the 3D-finishing core
/// start with the parameters a first implementation needs; lanes extend them.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
#[non_exhaustive]
pub enum Strategy {
    // --- 2D / 2.5D ---
    Face(FaceParams),
    Pocket2D(Pocket2dParams),
    Contour2D(Contour2dParams),
    Adaptive2D(Adaptive2dParams),
    Slot(SlotParams),
    Trace(TraceParams),
    Chamfer2D(Chamfer2dParams),
    Engrave(EngraveParams),
    Bore(BoreParams),
    Circular(CircularParams),
    ThreadMill(ThreadMillParams),
    // --- Drilling ---
    Drill(DrillParams),
    // --- 3D roughing ---
    Adaptive3D(Adaptive3dParams),
    Pocket3D(Pocket3dParams),
    // --- 3D finishing ---
    Parallel(ParallelParams),
    Contour3D(Contour3dParams),
    Scallop(ScallopParams),
    Pencil(PencilParams),
    SteepShallow(SteepShallowParams),
    Radial(RadialParams),
    Spiral(SpiralParams),
    MorphedSpiral(MorphedSpiralParams),
    Horizontal(HorizontalParams),
    Ramp(RampFinishParams),
    Project(ProjectParams),
    Flow(FlowParams),
    Morph(MorphParams),
    RestFinish(RestFinishParams),
    /// Probing a work offset (lane C14).
    ProbeWcs(super::ProbeWcsParams),
    /// Probing geometry for inspection (lane C14).
    ProbeGeometry(super::ProbeGeometryParams),
    /// Pattern of other enabled operations: linear, circular, mirror or WCS offsets (lane C15).
    Pattern(PatternParams),
    /// A manual NC entry: stop, dwell, comment, pass-through text or tool change (lane C15).
    ManualNc(ManualNcParams),
    // --- Multi-axis ---
    Swarf(SwarfParams),
    MultiAxisContour(MultiAxisContourParams),
    RotaryWrap(RotaryWrapParams),
    /// Rotary finishing: the tool axis radial to a 4th axis, line or spiral passes (lane C10).
    RotaryFinish(RotaryFinishParams),
    /// Multi-axis finishing over an inner 3-axis finishing operation (lane C10b).
    MultiAxisFinish(MultiAxisFinishParams),
    // --- Turning ---
    TurnFace(TurnFaceParams),
    TurnProfile(TurnProfileParams),
    TurnGroove(TurnGrooveParams),
    TurnThread(TurnThreadParams),
    TurnPartOff(TurnPartOffParams),
}

impl Default for Strategy {
    fn default() -> Self {
        Strategy::Parallel(ParallelParams::default())
    }
}

impl Strategy {
    /// Every strategy's kind name (the serde tag), in declaration order. The
    /// contract the application lists strategies from; `kind_name` reads it.
    pub const KINDS: &'static [&'static str] = &[
        "face",
        "pocket2D",
        "contour2D",
        "adaptive2D",
        "slot",
        "trace",
        "chamfer2D",
        "engrave",
        "bore",
        "circular",
        "threadMill",
        "drill",
        "adaptive3D",
        "pocket3D",
        "parallel",
        "contour3D",
        "scallop",
        "pencil",
        "steepShallow",
        "radial",
        "spiral",
        "morphedSpiral",
        "horizontal",
        "ramp",
        "project",
        "flow",
        "morph",
        "restFinish",
        "probeWcs",
        "probeGeometry",
        "pattern",
        "manualNc",
        "swarf",
        "multiAxisContour",
        "rotaryWrap",
        "rotaryFinish",
        "multiAxisFinish",
        "turnFace",
        "turnProfile",
        "turnGroove",
        "turnThread",
        "turnPartOff",
    ];

    /// Position of this variant in [`Strategy::KINDS`].
    pub fn kind_index(&self) -> usize {
        match self {
            Strategy::Face(_) => 0,
            Strategy::Pocket2D(_) => 1,
            Strategy::Contour2D(_) => 2,
            Strategy::Adaptive2D(_) => 3,
            Strategy::Slot(_) => 4,
            Strategy::Trace(_) => 5,
            Strategy::Chamfer2D(_) => 6,
            Strategy::Engrave(_) => 7,
            Strategy::Bore(_) => 8,
            Strategy::Circular(_) => 9,
            Strategy::ThreadMill(_) => 10,
            Strategy::Drill(_) => 11,
            Strategy::Adaptive3D(_) => 12,
            Strategy::Pocket3D(_) => 13,
            Strategy::Parallel(_) => 14,
            Strategy::Contour3D(_) => 15,
            Strategy::Scallop(_) => 16,
            Strategy::Pencil(_) => 17,
            Strategy::SteepShallow(_) => 18,
            Strategy::Radial(_) => 19,
            Strategy::Spiral(_) => 20,
            Strategy::MorphedSpiral(_) => 21,
            Strategy::Horizontal(_) => 22,
            Strategy::Ramp(_) => 23,
            Strategy::Project(_) => 24,
            Strategy::Flow(_) => 25,
            Strategy::Morph(_) => 26,
            Strategy::RestFinish(_) => 27,
            Strategy::ProbeWcs(_) => 28,
            Strategy::ProbeGeometry(_) => 29,
            Strategy::Pattern(_) => 30,
            Strategy::ManualNc(_) => 31,
            Strategy::Swarf(_) => 32,
            Strategy::MultiAxisContour(_) => 33,
            Strategy::RotaryWrap(_) => 34,
            Strategy::RotaryFinish(_) => 35,
            Strategy::MultiAxisFinish(_) => 36,
            Strategy::TurnFace(_) => 37,
            Strategy::TurnProfile(_) => 38,
            Strategy::TurnGroove(_) => 39,
            Strategy::TurnThread(_) => 40,
            Strategy::TurnPartOff(_) => 41,
        }
    }

    /// Stable lowercase name of the strategy (the serde tag).
    pub fn kind_name(&self) -> &'static str {
        Self::KINDS[self.kind_index()]
    }

    /// Enum-valued fields of the strategy kind `kind`: the dotted JSON path of
    /// every string-tagged field and the names it may take. Variant-dependent
    /// payload fields are listed too where a nested tag selects them.
    pub fn enum_choices(kind: &str) -> &'static [EnumChoice] {
        const STEPOVER: EnumChoice = EnumChoice { path: "stepover.direction", variants: CutDirection::VARIANTS };
        const DIRECTION: EnumChoice = EnumChoice { path: "direction", variants: CutDirection::VARIANTS };
        const CONTAINMENT: EnumChoice = EnumChoice { path: "containment.mode", variants: ContainmentMode::VARIANTS };
        match kind {
            "face" | "pocket2D" | "circular" => &[STEPOVER],
            "contour2D" => &[DIRECTION],
            "adaptive2D" => &[DIRECTION, EnumChoice { path: "restSource", variants: RestSource::VARIANTS }],
            "slot" | "trace" | "chamfer2D" | "engrave" | "bore" | "threadMill" | "swarf" | "turnFace" | "turnGroove"
            | "turnThread" | "turnPartOff" | "probeGeometry" => &[],
            "drill" => &[EnumChoice { path: "cycle", variants: DrillCycle::VARIANTS }],
            "pencil" | "radial" | "project" | "multiAxisContour" => &[CONTAINMENT],
            "adaptive3D" => &[CONTAINMENT, EnumChoice { path: "restSource", variants: RestSource::VARIANTS }],
            "parallel" | "scallop" | "spiral" | "morphedSpiral" | "horizontal" | "flow" | "morph" => {
                &[STEPOVER, CONTAINMENT]
            }
            "pocket3D" => &[STEPOVER, CONTAINMENT, EnumChoice { path: "restSource", variants: RestSource::VARIANTS }],
            "restFinish" => &[STEPOVER, CONTAINMENT, EnumChoice { path: "source", variants: RestSource::VARIANTS }],
            "contour3D" | "ramp" => &[DIRECTION, CONTAINMENT],
            "steepShallow" => &[EnumChoice { path: "shallowStepover.direction", variants: CutDirection::VARIANTS }, CONTAINMENT],
            "rotaryWrap" | "rotaryFinish" => &[STEPOVER],
            "multiAxisFinish" => &[EnumChoice { path: "mode.kind", variants: AxisMode::VARIANTS }],
            "probeWcs" => &[EnumChoice { path: "probeKind", variants: super::ProbeKind::VARIANTS }],
            "pattern" => &[
                EnumChoice { path: "patternKind.kind", variants: PatternKind::VARIANTS },
                EnumChoice { path: "order", variants: PatternOrder::VARIANTS },
            ],
            "manualNc" => &[EnumChoice { path: "manualKind.kind", variants: ManualNcKind::VARIANTS }],
            "turnProfile" => &[EnumChoice { path: "direction", variants: RoughingDirection::VARIANTS }],
            _ => &[],
        }
    }

    /// The default parameters of the strategy named `kind`, or `None` for an
    /// unknown name. The returned value may still fail `validate` where a
    /// strategy needs user input (morph curves, a reference tool).
    pub fn default_of(kind: &str) -> Option<Strategy> {
        Some(match kind {
            "face" => Strategy::Face(Default::default()),
            "pocket2D" => Strategy::Pocket2D(Default::default()),
            "contour2D" => Strategy::Contour2D(Default::default()),
            "adaptive2D" => Strategy::Adaptive2D(Default::default()),
            "slot" => Strategy::Slot(Default::default()),
            "trace" => Strategy::Trace(Default::default()),
            "chamfer2D" => Strategy::Chamfer2D(Default::default()),
            "engrave" => Strategy::Engrave(Default::default()),
            "bore" => Strategy::Bore(Default::default()),
            "circular" => Strategy::Circular(Default::default()),
            "threadMill" => Strategy::ThreadMill(Default::default()),
            "drill" => Strategy::Drill(Default::default()),
            "adaptive3D" => Strategy::Adaptive3D(Default::default()),
            "pocket3D" => Strategy::Pocket3D(Default::default()),
            "parallel" => Strategy::Parallel(Default::default()),
            "contour3D" => Strategy::Contour3D(Default::default()),
            "scallop" => Strategy::Scallop(Default::default()),
            "pencil" => Strategy::Pencil(Default::default()),
            "steepShallow" => Strategy::SteepShallow(Default::default()),
            "radial" => Strategy::Radial(Default::default()),
            "spiral" => Strategy::Spiral(Default::default()),
            "morphedSpiral" => Strategy::MorphedSpiral(Default::default()),
            "horizontal" => Strategy::Horizontal(Default::default()),
            "ramp" => Strategy::Ramp(Default::default()),
            "project" => Strategy::Project(Default::default()),
            "flow" => Strategy::Flow(Default::default()),
            "morph" => Strategy::Morph(Default::default()),
            "restFinish" => Strategy::RestFinish(Default::default()),
            "probeWcs" => Strategy::ProbeWcs(Default::default()),
            "probeGeometry" => Strategy::ProbeGeometry(Default::default()),
            "pattern" => Strategy::Pattern(Default::default()),
            "manualNc" => Strategy::ManualNc(Default::default()),
            "swarf" => Strategy::Swarf(Default::default()),
            "multiAxisContour" => Strategy::MultiAxisContour(Default::default()),
            "rotaryWrap" => Strategy::RotaryWrap(Default::default()),
            "rotaryFinish" => Strategy::RotaryFinish(Default::default()),
            "multiAxisFinish" => Strategy::MultiAxisFinish(Default::default()),
            "turnFace" => Strategy::TurnFace(Default::default()),
            "turnProfile" => Strategy::TurnProfile(Default::default()),
            "turnGroove" => Strategy::TurnGroove(Default::default()),
            "turnThread" => Strategy::TurnThread(Default::default()),
            "turnPartOff" => Strategy::TurnPartOff(Default::default()),
            _ => return None,
        })
    }

    /// Strategies that machine nothing themselves and need no target body:
    /// patterns of other operations and manual NC entries.
    pub fn is_targetless(&self) -> bool {
        matches!(self, Strategy::Pattern(_) | Strategy::ManualNc(_))
    }

    pub fn is_turning(&self) -> bool {
        matches!(
            self,
            Strategy::TurnFace(_)
                | Strategy::TurnProfile(_)
                | Strategy::TurnGroove(_)
                | Strategy::TurnThread(_)
                | Strategy::TurnPartOff(_)
        )
    }

    pub fn validate(&self) -> Result<(), String> {
        match self {
            Strategy::Face(p) => {
                p.depths.validate()?;
                p.stepover.validate()
            }
            Strategy::Pocket2D(p) => {
                p.depths.validate()?;
                p.stepover.validate()
            }
            Strategy::Contour2D(p) => {
                p.depths.validate()?;
                positive_or_zero("finishing overlap", p.finishing_overlap)?;
                positive_or_zero("roughing stepover", p.roughing_stepover)
            }
            Strategy::Adaptive2D(p) => {
                p.depths.validate()?;
                positive("optimal load", p.optimal_load)?;
                positive_or_zero("minimum cutting radius", p.minimum_cutting_radius)
            }
            Strategy::Slot(p) => {
                p.depths.validate()?;
                positive("slot width", p.width)
            }
            Strategy::Trace(p) => p.depths.validate(),
            Strategy::Chamfer2D(p) => {
                positive("chamfer width", p.width)?;
                positive_or_zero("chamfer tip offset", p.tip_offset)
            }
            Strategy::Engrave(p) => p.depths.validate(),
            Strategy::Bore(p) => {
                p.depths.validate()?;
                positive_or_zero("bore pitch", p.pitch)
            }
            Strategy::Circular(p) => {
                p.depths.validate()?;
                p.stepover.validate()
            }
            Strategy::ThreadMill(p) => {
                p.depths.validate()?;
                positive("thread pitch", p.pitch)?;
                positive("thread diameter", p.diameter)
            }
            Strategy::Drill(p) => {
                positive_or_zero("peck depth", p.peck_depth)?;
                positive_or_zero("dwell", p.dwell_seconds)?;
                positive_or_zero("break-through depth", p.break_through)?;
                positive_or_zero("chip-break retract", p.chip_break_retract)?;
                positive_or_zero("bore shift", p.bore_shift)?;
                positive_or_zero("back-bore clearance", p.back_bore_clearance)?;
                if p.cycle == DrillCycle::ChipBreak {
                    positive("chip-break retract", p.chip_break_retract)?;
                }
                if p.cycle == DrillCycle::BackBore {
                    positive("back-bore clearance", p.back_bore_clearance)?;
                }
                Ok(())
            }
            Strategy::Adaptive3D(p) => {
                p.depths.validate()?;
                positive("optimal load", p.optimal_load)?;
                positive_or_zero("fine step down", p.fine_step_down)
            }
            Strategy::Pocket3D(p) => {
                p.depths.validate()?;
                p.stepover.validate()
            }
            Strategy::Parallel(p) => {
                p.stepover.validate()?;
                p.containment.validate()?;
                finite("pass angle", p.pass_angle_deg)
            }
            Strategy::Contour3D(p) => {
                p.containment.validate()?;
                positive("step down", p.step_down)
            }
            Strategy::Scallop(p) => {
                p.stepover.validate()?;
                if let Some(h) = p.cusp_height {
                    positive("cusp height", h)?;
                }
                p.containment.validate()
            }
            Strategy::Pencil(p) => {
                p.containment.validate()?;
                positive_or_zero("pencil bitangency angle", p.bitangency_angle_deg)
            }
            Strategy::SteepShallow(p) => {
                p.containment.validate()?;
                positive("steep step down", p.steep_step_down)?;
                p.shallow_stepover.validate()?;
                positive_or_zero("steep/shallow overlap", p.overlap)?;
                finite("threshold angle", p.threshold_angle_deg)
            }
            Strategy::Radial(p) => {
                p.containment.validate()?;
                positive("radial angular step", p.angular_step_deg)
            }
            Strategy::Spiral(p) => {
                p.containment.validate()?;
                p.stepover.validate()
            }
            Strategy::MorphedSpiral(p) => {
                p.containment.validate()?;
                p.stepover.validate()
            }
            Strategy::Horizontal(p) => {
                p.containment.validate()?;
                p.stepover.validate()
            }
            Strategy::Ramp(p) => {
                p.containment.validate()?;
                positive("ramp step down", p.step_down)
            }
            Strategy::Project(p) => p.containment.validate(),
            Strategy::Flow(p) => {
                p.containment.validate()?;
                p.stepover.validate()
            }
            Strategy::Morph(p) => {
                p.containment.validate()?;
                p.stepover.validate()?;
                for curve in [&p.first_curve, &p.second_curve] {
                    if curve.len() < 2 || !curve.iter().all(super::Point2::is_finite) {
                        return Err("morph needs two finite boundary polylines".into());
                    }
                }
                Ok(())
            }
            Strategy::RestFinish(p) => {
                p.containment.validate()?;
                p.stepover.validate()?;
                positive_or_zero("rest threshold", p.minimum_rest)?;
                if !p.pass_angle_deg.is_finite() || (p.source == RestSource::ReferenceTool && p.reference_tool.0.is_empty()) {
                    return Err("rest finishing needs a reference tool and finite pass angle".into());
                }
                Ok(())
            }
            Strategy::ProbeWcs(p) => p.validate(),
            Strategy::ProbeGeometry(p) => p.validate(),
            Strategy::Pattern(p) => p.validate(),
            Strategy::ManualNc(p) => p.validate(),
            Strategy::Swarf(p) => positive_or_zero("swarf tilt", p.tilt_deg),
            Strategy::MultiAxisContour(p) => {
                p.containment.validate()?;
                positive_or_zero("lead angle", p.lead_deg)?;
                positive_or_zero("tilt angle", p.tilt_deg)
            }
            Strategy::RotaryWrap(p) => {
                p.stepover.validate()?;
                positive("wrap diameter", p.wrap_diameter)
            }
            Strategy::RotaryFinish(p) => p.validate(),
            Strategy::MultiAxisFinish(p) => p.validate(),
            Strategy::TurnFace(p) => {
                positive("step over", p.step_over)?;
                positive_or_zero("facing end x", p.end_x)?;
                positive_or_zero("finish allowance", p.finish_allowance)
            }
            Strategy::TurnProfile(p) => positive("depth of cut", p.depth_of_cut),
            Strategy::TurnGroove(p) => {
                positive("groove width", p.width)?;
                positive_or_zero("groove depth", p.depth)?;
                positive("groove step over", p.step_over)?;
                positive_or_zero("finish allowance", p.finish_allowance)
            }
            Strategy::TurnThread(p) => {
                positive("thread pitch", p.pitch)?;
                positive("thread depth", p.depth)?;
                if p.passes == 0 {
                    return Err("thread passes must be at least 1".into());
                }
                if !p.infeed_angle_deg.is_finite() || p.infeed_angle_deg < 0.0 || p.infeed_angle_deg >= 60.0 {
                    return Err("thread infeed angle must be finite and within [0, 60) degrees".into());
                }
                Ok(())
            }
            Strategy::TurnPartOff(p) => {
                positive_or_zero("part-off x", p.end_x)?;
                positive_or_zero("part-off chamfer", p.chamfer)?;
                positive_or_zero("peck depth", p.peck_depth)?;
                positive_or_zero("peck retract", p.peck_retract)?;
                positive_or_zero("reduced feed radius", p.reduced_feed_radius)?;
                if !p.reduced_feed_factor.is_finite() || p.reduced_feed_factor <= 0.0 || p.reduced_feed_factor > 1.0 {
                    return Err("reduced feed factor must be within (0, 1]".into());
                }
                Ok(())
            }
        }
    }
}

fn finite(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() {
        Ok(())
    } else {
        Err(format!("{name} is not finite"))
    }
}

fn positive(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v > 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be finite and positive"))
    }
}

fn positive_or_zero(name: &str, v: f64) -> Result<(), String> {
    if v.is_finite() && v >= 0.0 {
        Ok(())
    } else {
        Err(format!("{name} must be finite and non-negative"))
    }
}

// ---------------------------------------------------------------------------
// 2D / 2.5D
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct FaceParams {
    pub depths: Depths,
    pub stepover: Stepover,
    /// Pass direction in degrees from machine +X.
    pub pass_angle_deg: f64,
    /// Extend passes past the stock edge by this much.
    pub pass_extension: f64,
    pub zig_zag: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Pocket2dParams {
    pub depths: Depths,
    pub stepover: Stepover,
    /// Offset pattern (true) or morphed spiral (false).
    pub offset_pattern: bool,
    /// Run a finishing wall pass at each depth.
    pub wall_finish: bool,
    /// Islands are kept (true) or cut through (false).
    pub keep_islands: bool,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Contour2dParams {
    pub depths: Depths,
    /// Tool stays outside the chain (true) or inside (false).
    pub outside: bool,
    /// Number of radial roughing passes before the finishing pass.
    pub roughing_passes: u32,
    pub roughing_stepover: f64,
    /// Repeat the finishing pass this many times.
    pub finishing_passes: u32,
    /// Overlap at the start/end of a closed contour.
    pub finishing_overlap: f64,
    pub tabs: Vec<Tab>,
    pub direction: CutDirection,
}

impl Default for Contour2dParams {
    fn default() -> Self {
        Self {
            depths: Depths::default(),
            outside: true,
            roughing_passes: 0,
            roughing_stepover: 0.0,
            finishing_passes: 1,
            finishing_overlap: 0.0,
            tabs: Vec::new(),
            direction: CutDirection::Climb,
        }
    }
}

/// A holding tab left on a contour.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Tab {
    /// Position along the chain as a fraction of its length.
    pub at: f64,
    pub width: f64,
    pub height: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Adaptive2dParams {
    pub depths: Depths,
    /// Target radial engagement (constant-engagement roughing).
    pub optimal_load: f64,
    pub minimum_cutting_radius: f64,
    /// Use the rest material of previous operations as the starting stock.
    pub rest_machining: bool,
    /// Where the rest material comes from when `rest_machining` is on.
    pub rest_source: RestSource,
    pub direction: CutDirection,
}

impl Default for Adaptive2dParams {
    fn default() -> Self {
        Self {
            depths: Depths::default(),
            optimal_load: 1.0,
            minimum_cutting_radius: 0.0,
            rest_machining: false,
            rest_source: RestSource::default(),
            direction: CutDirection::Climb,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SlotParams {
    pub depths: Depths,
    /// Slot width; the tool diameter when equal.
    pub width: f64,
    pub trochoidal: bool,
}

impl Default for SlotParams {
    fn default() -> Self {
        Self { depths: Depths::default(), width: 6.0, trochoidal: false }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TraceParams {
    pub depths: Depths,
    /// Tool centre on the chain (0), left (-1) or right (+1) of it.
    pub side: i8,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Chamfer2dParams {
    /// Chamfer leg length.
    pub width: f64,
    /// Keep the tool tip this far below the chamfer's lower edge.
    pub tip_offset: f64,
}

impl Default for Chamfer2dParams {
    fn default() -> Self {
        Self { width: 1.0, tip_offset: 0.5 }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct EngraveParams {
    pub depths: Depths,
    /// V-carve: depth follows the region width.
    pub v_carve: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct BoreParams {
    pub depths: Depths,
    /// Helical pitch per revolution (`0`: circular passes at each depth).
    pub pitch: f64,
    pub finish_pass: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct CircularParams {
    pub depths: Depths,
    pub stepover: Stepover,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ThreadMillParams {
    pub depths: Depths,
    pub pitch: f64,
    /// Major diameter of the thread.
    pub diameter: f64,
    pub internal: bool,
    pub left_hand: bool,
    pub passes: u32,
}

impl Default for ThreadMillParams {
    fn default() -> Self {
        Self { depths: Depths::default(), pitch: 1.0, diameter: 10.0, internal: true, left_hand: false, passes: 1 }
    }
}

// ---------------------------------------------------------------------------
// Drilling
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct DrillParams {
    pub cycle: DrillCycle,
    /// Hole depth below the hole top (`None`: through the recognised hole).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub depth: Option<f64>,
    pub peck_depth: f64,
    pub dwell_seconds: f64,
    /// Extra depth past the hole bottom for through holes.
    pub break_through: f64,
    /// Order holes to minimise travel.
    pub optimise_order: bool,
    /// Use the tool's tip (true) or shoulder (false) as the depth reference.
    pub tip_depth: bool,
    /// G73 partial retract distance; must be positive for chip-break.
    pub chip_break_retract: f64,
    /// G76/G87 lateral shift along machine +X.
    pub bore_shift: f64,
    /// G87 axial entry below finished back face (positive distance).
    pub back_bore_clearance: f64,
}

// ---------------------------------------------------------------------------
// 3D roughing
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Adaptive3dParams {
    pub depths: Depths,
    pub optimal_load: f64,
    /// Smaller step-down used to clear flat areas between the main depths.
    pub fine_step_down: f64,
    pub rest_machining: bool,
    /// Where the rest material comes from when `rest_machining` is on.
    pub rest_source: RestSource,
    pub containment: Containment,
}

impl Default for Adaptive3dParams {
    fn default() -> Self {
        Self {
            depths: Depths::default(),
            optimal_load: 1.0,
            fine_step_down: 0.0,
            rest_machining: false,
            rest_source: RestSource::default(),
            containment: Containment::default(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Pocket3dParams {
    pub depths: Depths,
    pub stepover: Stepover,
    pub rest_machining: bool,
    /// Where the rest material comes from when `rest_machining` is on.
    pub rest_source: RestSource,
    pub containment: Containment,
}

// ---------------------------------------------------------------------------
// 3D finishing
// ---------------------------------------------------------------------------

/// Parallel (raster) finishing: drop-cutter along parallel lines.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ParallelParams {
    pub stepover: Stepover,
    /// Pass direction in degrees from machine +X.
    pub pass_angle_deg: f64,
    /// Alternate pass direction (zig-zag) instead of unidirectional passes.
    pub zig_zag: bool,
    /// Spacing of samples along a pass before adaptive refinement.
    pub sample_step: f64,
    /// Refine samples until the chord error is within the operation tolerance.
    pub adaptive_sampling: bool,
    pub containment: Containment,
    /// Lowest Z the cutter may drop to when nothing is below it.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub floor_z: Option<f64>,
}

fn default_sample_step() -> f64 {
    0.5
}

impl Default for ParallelParams {
    fn default() -> Self {
        Self {
            stepover: Stepover::default(),
            pass_angle_deg: 0.0,
            zig_zag: true,
            sample_step: default_sample_step(),
            adaptive_sampling: true,
            containment: Containment::default(),
            floor_z: None,
        }
    }
}

/// Contour / waterline finishing: closed loops at constant Z.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Contour3dParams {
    pub step_down: f64,
    /// Add intermediate levels where the slope leaves scallops above tolerance.
    /// Off by default until the waterline generator supports it (lane C3).
    pub adaptive_step_down: bool,
    /// Fiber spacing for the push-cutter sweep before adaptive refinement.
    pub fiber_step: f64,
    pub direction: CutDirection,
    pub containment: Containment,
}

impl Default for Contour3dParams {
    fn default() -> Self {
        Self {
            step_down: 1.0,
            adaptive_step_down: false,
            fiber_step: default_sample_step(),
            direction: CutDirection::Climb,
            containment: Containment::default(),
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ScallopParams {
    pub stepover: Stepover,
    pub containment: Containment,
    /// Keep passes inside-out (true) or outside-in (false).
    pub inside_out: bool,
    /// Maximum cusp (scallop) height in model units. When set it wins over
    /// `stepover`: the stepover is derived from the cutter and this height.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub cusp_height: Option<f64>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct PencilParams {
    pub containment: Containment,
    /// Only trace corners whose bitangency angle is sharper than this.
    pub bitangency_angle_deg: f64,
    /// Extra parallel passes beside the pencil trace.
    pub extra_passes: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SteepShallowParams {
    pub containment: Containment,
    /// Slope that separates steep (waterline) from shallow (scallop) regions.
    pub threshold_angle_deg: f64,
    pub steep_step_down: f64,
    pub shallow_stepover: Stepover,
    /// Distance the steep and shallow regions overlap across the threshold
    /// slope (Fusion's steep/shallow overlap), in model units.
    pub overlap: f64,
}

impl Default for SteepShallowParams {
    fn default() -> Self {
        Self {
            containment: Containment::default(),
            threshold_angle_deg: 45.0,
            steep_step_down: 1.0,
            shallow_stepover: Stepover::default(),
            overlap: 0.0,
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct RadialParams {
    pub containment: Containment,
    /// Centre of the radial pattern in machine XY.
    pub center: super::Point2,
    pub angular_step_deg: f64,
    pub inner_radius: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outer_radius: Option<f64>,
}

impl Default for RadialParams {
    fn default() -> Self {
        Self {
            containment: Containment::default(),
            center: super::Point2::default(),
            angular_step_deg: 1.0,
            inner_radius: 0.0,
            outer_radius: None,
        }
    }
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SpiralParams {
    pub containment: Containment,
    pub center: super::Point2,
    pub stepover: Stepover,
    pub inside_out: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct MorphedSpiralParams {
    pub containment: Containment,
    pub stepover: Stepover,
}

/// Horizontal (flat-area) finishing.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct HorizontalParams {
    pub containment: Containment,
    pub stepover: Stepover,
    /// Treat surfaces within this slope of horizontal as flat.
    pub flat_angle_deg: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct RampFinishParams {
    pub containment: Containment,
    pub step_down: f64,
    pub direction: CutDirection,
}

impl Default for RampFinishParams {
    fn default() -> Self {
        Self { containment: Containment::default(), step_down: 1.0, direction: CutDirection::Climb }
    }
}

/// Project 2D curves onto the surface.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ProjectParams {
    pub containment: Containment,
    /// Curves to project, as 2D polylines in machine XY.
    pub curves: Vec<Vec<super::Point2>>,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct FlowParams {
    pub containment: Containment,
    pub stepover: Stepover,
    /// Follow the surface's U (true) or V (false) direction.
    pub along_u: bool,
}

/// Boundary curves use the same direction and normalized arc-length correspondence.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct MorphParams {
    pub containment: Containment,
    pub stepover: Stepover,
    pub first_curve: Vec<super::Point2>,
    pub second_curve: Vec<super::Point2>,
}

/// Source of residual material for rest finishing.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum RestSource { #[default] ReferenceTool, Stock }

impl RestSource {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["referenceTool", "stock"];
}

/// Reference-tool accessibility or simulated-stock rest finishing.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase", default)]
pub struct RestFinishParams {
    pub source: RestSource,
    pub containment: Containment,
    pub stepover: Stepover,
    pub reference_tool: super::ToolId,
    pub minimum_rest: f64,
    pub pass_angle_deg: f64,
}

// ---------------------------------------------------------------------------
// Multi-axis
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SwarfParams {
    /// Extra tilt away from the wall in degrees.
    pub tilt_deg: f64,
    /// Number of depth passes down the wall.
    pub passes: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct MultiAxisContourParams {
    pub containment: Containment,
    pub lead_deg: f64,
    pub tilt_deg: f64,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct RotaryWrapParams {
    pub stepover: Stepover,
    /// Diameter the flat toolpath is wrapped onto.
    pub wrap_diameter: f64,
    /// Rotary axis letter (`A` or `B`).
    pub axis: String,
}

fn default_rotary_axis() -> String {
    "A".to_string()
}

impl Default for RotaryWrapParams {
    fn default() -> Self {
        Self { stepover: Stepover::default(), wrap_diameter: 50.0, axis: default_rotary_axis() }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct RotaryFinishParams {
    /// Rotary axis letter (`A` about X, `B` about Y).
    pub axis: String,
    /// Diameter the passes start from (the stock), also the stepover's reference.
    pub stock_diameter: f64,
    /// Pass spacing as arc length at the stock diameter (line passes) or
    /// axial advance per turn (spiral).
    pub stepover: Stepover,
    /// One continuous spiral instead of line passes.
    pub spiral: bool,
    /// Alternate the direction of successive line passes.
    pub zig_zag: bool,
    /// Sample spacing along the axis before refinement.
    pub sample_step: f64,
    /// Axial range to machine (`None`: the target mesh's extent along the axis).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub range: Option<[f64; 2]>,
}

impl Default for RotaryFinishParams {
    fn default() -> Self {
        Self { axis: "A".into(), stock_diameter: 50.0, stepover: Stepover::default(), spiral: false, zig_zag: true, sample_step: 1.0, range: None }
    }
}

impl RotaryFinishParams {
    pub fn validate(&self) -> Result<(), String> {
        if !matches!(self.axis.as_str(), "A" | "a" | "B" | "b") {
            return Err("rotary axis must be A or B".into());
        }
        if !self.stock_diameter.is_finite() || self.stock_diameter <= 0.0 {
            return Err("rotary stock diameter must be positive".into());
        }
        if !self.stepover.distance.is_finite() || self.stepover.distance <= 0.0 {
            return Err("rotary stepover must be positive".into());
        }
        if !self.sample_step.is_finite() || self.sample_step <= 0.0 {
            return Err("rotary sample step must be positive".into());
        }
        if let Some([lo, hi]) = self.range {
            if !lo.is_finite() || !hi.is_finite() || lo >= hi {
                return Err("rotary range must be finite with lo < hi".into());
            }
        }
        Ok(())
    }
}

/// How a multi-axis finishing pass orients the tool axis at each contact
/// (lane C10b; the geometric evaluation lives in `multiaxis::orientation`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum AxisMode {
    /// The surface normal rotated by `lead_deg` about the binormal (forward
    /// positive) and `tilt_deg` about the tangent (left positive).
    Normal {
        #[serde(default)]
        lead_deg: f64,
        #[serde(default)]
        tilt_deg: f64,
    },
    /// Along the line from `point` through the contact (`away: true`) or the
    /// reverse (the tool points at `point`).
    Point { point: Point3, away: bool },
    /// Perpendicular to the line through `origin` with `direction`, through
    /// the contact point, away from (`away: true`) or toward the line.
    Line { origin: Point3, direction: Point3, away: bool },
    /// Constant: `angle_deg` from +Z toward the azimuth `azimuth_deg` (from +X).
    FixedFromVertical { angle_deg: f64, azimuth_deg: f64 },
}

impl AxisMode {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["normal", "point", "line", "fixedFromVertical"];
}

impl Default for AxisMode {
    fn default() -> Self {
        AxisMode::Normal { lead_deg: 0.0, tilt_deg: 0.0 }
    }
}

impl AxisMode {
    pub fn validate(&self) -> Result<(), String> {
        match self {
            AxisMode::Normal { lead_deg, tilt_deg } => {
                for (n, v) in [("lead", lead_deg), ("tilt", tilt_deg)] {
                    if !v.is_finite() || v.abs() >= 90.0 {
                        return Err(format!("axis {n} must be within (-90, 90) degrees"));
                    }
                }
                Ok(())
            }
            AxisMode::Point { point, .. } => {
                if !point.is_finite() {
                    return Err("axis point is not finite".into());
                }
                Ok(())
            }
            AxisMode::Line { origin, direction, .. } => {
                if !origin.is_finite() || direction.normalized().is_none() {
                    return Err("axis line needs a finite origin and a non-zero direction".into());
                }
                Ok(())
            }
            AxisMode::FixedFromVertical { angle_deg, azimuth_deg } => {
                if !angle_deg.is_finite() || *angle_deg < 0.0 || *angle_deg >= 90.0 || !azimuth_deg.is_finite() {
                    return Err("fixed axis angle must be in [0, 90) degrees with a finite azimuth".into());
                }
                Ok(())
            }
        }
    }
}

/// Multi-axis finishing over an inner 3-axis finishing operation (lane C10b).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct MultiAxisFinishParams {
    pub mode: AxisMode,
    /// Largest axis change between consecutive samples, degrees.
    pub max_axis_step_deg: f64,
    /// Largest angle of the axis from +Z, degrees.
    pub max_from_vertical_deg: f64,
    /// Largest extra tilt used to clear the shank/holder or the machine
    /// limits, degrees.
    pub reach_tilt_deg: f64,
    /// What a CL point no tilt within the reach clears does to the path
    /// (absent: `trim`, so documents written before the option read the same).
    #[serde(default, skip_serializing_if = "UnreachablePolicy::is_trim")]
    pub unreachable: UnreachablePolicy,
}

/// A multi-axis finish's handling of CL points where the shank or holder
/// collides at every tilt within the reach.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum UnreachablePolicy {
    /// Break the path there: retract, link and re-enter past them; the
    /// material left is reported (a warning and an empty region per point).
    #[default]
    Trim,
    /// Refuse the operation (its report fails) when any point is unreachable.
    Fail,
}

impl UnreachablePolicy {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["trim", "fail"];

    pub fn is_trim(&self) -> bool {
        *self == UnreachablePolicy::Trim
    }
}

impl Default for MultiAxisFinishParams {
    fn default() -> Self {
        Self { mode: AxisMode::default(), max_axis_step_deg: 5.0, max_from_vertical_deg: 60.0, reach_tilt_deg: 30.0, unreachable: UnreachablePolicy::Trim }
    }
}

impl MultiAxisFinishParams {
    pub fn validate(&self) -> Result<(), String> {
        let bad = |m: &str| Err(format!("multi-axis finish: {m}"));
        if let Err(e) = self.mode.validate() {
            return bad(&e);
        }
        if !self.max_axis_step_deg.is_finite() || self.max_axis_step_deg <= 0.0 {
            return bad("axis step must be positive");
        }
        if !self.max_from_vertical_deg.is_finite() || self.max_from_vertical_deg < 0.0 || self.max_from_vertical_deg >= 90.0 {
            return bad("angle from vertical must be in [0, 90)");
        }
        if !self.reach_tilt_deg.is_finite() || self.reach_tilt_deg < 0.0 || self.reach_tilt_deg >= 90.0 {
            return bad("reach tilt must be in [0, 90)");
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Turning
// ---------------------------------------------------------------------------

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TurnFaceParams {
    pub step_over: f64,
    pub finish_pass: bool,
    /// X (radius) the facing passes end at; `0` faces to centre.
    pub end_x: f64,
    /// Material left for the finishing pass.
    pub finish_allowance: f64,
}

impl Default for TurnFaceParams {
    fn default() -> Self {
        Self { step_over: 1.0, finish_pass: true, end_x: 0.0, finish_allowance: 0.1 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TurnProfileParams {
    pub depth_of_cut: f64,
    pub roughing: bool,
    pub finishing: bool,
    /// Outside diameter (true) or inside/bore (false).
    pub outside: bool,
    /// Roughing pass direction: along the axis (Z) or radial (X plunges).
    pub direction: RoughingDirection,
}

impl Default for TurnProfileParams {
    fn default() -> Self {
        Self { depth_of_cut: 1.0, roughing: true, finishing: true, outside: true, direction: RoughingDirection::Axial }
    }
}

/// Direction of turning roughing passes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum RoughingDirection {
    #[default]
    Axial,
    Radial,
}

impl RoughingDirection {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["axial", "radial"];
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TurnGrooveParams {
    pub width: f64,
    pub depth: f64,
    /// Axial step between plunges when the groove is wider than the blade.
    pub step_over: f64,
    pub finish_allowance: f64,
    pub finish_pass: bool,
}

impl Default for TurnGrooveParams {
    fn default() -> Self {
        Self { width: 3.0, depth: 2.0, step_over: 1.0, finish_allowance: 0.1, finish_pass: true }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TurnThreadParams {
    pub pitch: f64,
    pub depth: f64,
    pub passes: u32,
    pub internal: bool,
    /// Equal chip area per pass (true) or equal depth per pass (false).
    pub constant_area: bool,
    /// Infeed angle in degrees: `0` radial, positive along the flank.
    pub infeed_angle_deg: f64,
    pub alternate_flanks: bool,
    pub spring_passes: u32,
}

impl Default for TurnThreadParams {
    fn default() -> Self {
        Self { pitch: 1.5, depth: 0.92, passes: 6, internal: false, constant_area: true, infeed_angle_deg: 0.0, alternate_flanks: false, spring_passes: 0 }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct TurnPartOffParams {
    /// X (radius) the part-off ends at; `0` cuts to centre.
    pub end_x: f64,
    pub chamfer: f64,
    /// Peck depth in X (`0`: no pecking).
    pub peck_depth: f64,
    pub peck_retract: f64,
    /// Below this radius the feed is multiplied by `reduced_feed_factor`.
    pub reduced_feed_radius: f64,
    pub reduced_feed_factor: f64,
}

impl Default for TurnPartOffParams {
    fn default() -> Self {
        Self { end_x: 0.0, chamfer: 0.0, peck_depth: 0.0, peck_retract: 0.0, reduced_feed_radius: 0.0, reduced_feed_factor: 1.0 }
    }
}

// ---------------------------------------------------------------------------
// Patterns and manual NC (lane C15)
// ---------------------------------------------------------------------------

/// A pattern of other enabled source operations. Count INCLUDES the original
/// (linear, circular); a mirror emits the original and the reflected instance;
/// WCS offsets re-run the sources at each listed work offset with identical
/// source-relative tool geometry.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct PatternParams {
    pub sources: Vec<OperationId>,
    /// Serialized as `patternKind`: the strategy's own `kind` tag sits at the same level.
    #[serde(rename = "patternKind")]
    pub kind: PatternKind,
    pub order: PatternOrder,
}

impl Default for PatternParams {
    fn default() -> Self {
        Self { sources: Vec::new(), kind: PatternKind::default(), order: PatternOrder::default() }
    }
}

impl PatternParams {
    pub fn validate(&self) -> Result<(), String> {
        if self.sources.is_empty() || self.sources.iter().any(|s| s.as_str().is_empty()) {
            return Err("a pattern needs at least one source operation".into());
        }
        self.kind.validate()
    }
}

/// Shape of a pattern.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum PatternKind {
    /// `count` instances (including the original) `spacing` apart along `direction`.
    Linear { count: u32, spacing: f64, direction: Point3 },
    /// `count` instances (including the original) stepped by `angle_deg` about `axis` through `origin`.
    Circular { count: u32, angle_deg: f64, axis: Point3, origin: Point3 },
    /// The original and its reflection in the plane through `origin` with `normal`.
    Mirror { origin: Point3, normal: Point3 },
    /// The sources re-run at each work offset.
    WcsOffsets { list: Vec<WorkOffset> },
}

impl PatternKind {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["linear", "circular", "mirror", "wcsOffsets"];
}

impl Default for PatternKind {
    fn default() -> Self {
        PatternKind::Linear { count: 2, spacing: 10.0, direction: Point3::X }
    }
}

impl PatternKind {
    pub fn validate(&self) -> Result<(), String> {
        match self {
            PatternKind::Linear { count, spacing, direction } => {
                if *count < 1 {
                    return Err("linear pattern count must be at least 1".into());
                }
                positive("pattern spacing", *spacing)?;
                if direction.normalized().is_none() {
                    return Err("linear pattern direction must be a non-zero vector".into());
                }
                Ok(())
            }
            PatternKind::Circular { count, angle_deg, axis, origin } => {
                if *count < 1 {
                    return Err("circular pattern count must be at least 1".into());
                }
                if !angle_deg.is_finite() || *angle_deg == 0.0 {
                    return Err("circular pattern angle must be finite and non-zero".into());
                }
                if axis.normalized().is_none() || !origin.is_finite() {
                    return Err("circular pattern needs a non-zero axis and a finite origin".into());
                }
                Ok(())
            }
            PatternKind::Mirror { origin, normal } => {
                if normal.normalized().is_none() || !origin.is_finite() {
                    return Err("mirror pattern needs a non-zero normal and a finite origin".into());
                }
                Ok(())
            }
            PatternKind::WcsOffsets { list } => {
                if list.is_empty() {
                    return Err("WCS pattern needs at least one work offset".into());
                }
                if list.iter().any(|w| w.number == 0 || !w.origin.is_finite()) {
                    return Err("WCS pattern offsets need a number of at least 1 and a finite origin".into());
                }
                Ok(())
            }
        }
    }
}

/// Order of a pattern's expanded instances.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PatternOrder {
    /// Every source at one instance, then the next instance.
    #[default]
    ByInstance,
    /// Every instance of one tool's operations before the next tool.
    ByTool,
}

impl PatternOrder {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["by-instance", "by-tool"];
}

/// A manual NC entry: exactly the requested typed entry, no implicit
/// machining state (no tool, spindle or coolant preamble).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ManualNcParams {
    /// Serialized as `manualKind`: the strategy's own `kind` tag sits at the same level.
    #[serde(rename = "manualKind")]
    pub kind: ManualNcKind,
}

impl ManualNcParams {
    pub fn validate(&self) -> Result<(), String> {
        match &self.kind {
            ManualNcKind::Dwell { seconds } => positive_or_zero("manual dwell", *seconds),
            ManualNcKind::ToolChange { tool } if tool.as_str().is_empty() => Err("manual tool change needs a tool".into()),
            _ => Ok(()),
        }
    }
}

/// The manual entries. `PassThrough` text reaches the post byte for byte.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum ManualNcKind {
    Stop,
    OptionalStop,
    Dwell { seconds: f64 },
    Comment { text: String },
    PassThrough { text: String },
    ToolChange { tool: ToolId },
}

impl ManualNcKind {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["stop", "optionalStop", "dwell", "comment", "passThrough", "toolChange"];
}

impl Default for ManualNcKind {
    fn default() -> Self {
        ManualNcKind::Comment { text: String::new() }
    }
}
