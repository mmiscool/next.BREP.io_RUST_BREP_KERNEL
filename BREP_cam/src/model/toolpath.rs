//! The toolpath: an ordered stream of typed motions plus the operation report.
//!
//! Every strategy produces this stream; simulation, preview and the post read
//! the same stream, so they cannot disagree about rapid/plunge/cut order. Each
//! motion carries its own feed; the enclosing [`Toolpath`] carries the
//! operation and tool identity, and a program-level stream carries one
//! [`ProgramMotion`] per motion so identity survives concatenation.

use super::{Coolant, OperationId, Point3, SpindleDirection, ToolId};
use serde::{Deserialize, Serialize};

/// Feed for one motion. `Rapid` posts as G0.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum Feed {
    /// Rapid traverse (serialized as the string `"rapid"`).
    Rapid(RapidTag),
    /// Feed rate in model units per minute.
    Rate(f64),
}

/// The unit payload of [`Feed::Rapid`]; serializes as the literal `"rapid"`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum RapidTag {
    #[default]
    Rapid,
}

impl Feed {
    pub const RAPID: Feed = Feed::Rapid(RapidTag::Rapid);

    pub fn rate(rate: f64) -> Feed {
        Feed::Rate(rate)
    }

    pub fn is_rapid(&self) -> bool {
        matches!(self, Feed::Rapid(_))
    }

    /// Feed rate, `None` for rapids.
    pub fn value(&self) -> Option<f64> {
        match self {
            Feed::Rapid(_) => None,
            Feed::Rate(r) => Some(*r),
        }
    }
}

impl Default for Feed {
    fn default() -> Self {
        Feed::RAPID
    }
}

/// Plane of an arc motion.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ArcPlane {
    #[default]
    Xy,
    Xz,
    Yz,
}

/// One typed motion. Positions are the TOOL REFERENCE POINT in machine
/// coordinates; every variant that moves the tool ends at `to`. The start of a
/// motion is the end of the previous one.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
#[non_exhaustive]
pub enum MotionKind {
    /// An expanded probe trigger move (lane C14); never ordinary cutting.
    Probe { cycle: super::ProbeCycle },
    /// Rapid traverse clear of material.
    Rapid { to: Point3 },
    /// Vertical (or near-vertical) descent into material at plunge feed.
    Plunge { to: Point3 },
    /// Cutting move.
    Cut { to: Point3 },
    /// Clockwise arc (looking down the plane's normal) about `center`.
    /// `pitch` is the axial advance per full turn for helices, `0` for planar arcs.
    ArcCw {
        to: Point3,
        center: Point3,
        plane: ArcPlane,
        #[serde(default)]
        pitch: f64,
    },
    /// Counter-clockwise arc, same conventions as `ArcCw`.
    ArcCcw {
        to: Point3,
        center: Point3,
        plane: ArcPlane,
        #[serde(default)]
        pitch: f64,
    },
    /// Move up and away from material.
    Retract { to: Point3 },
    /// Approach into the cut.
    LeadIn { to: Point3 },
    /// Departure out of the cut.
    LeadOut { to: Point3 },
    /// Non-cutting connection between passes that stays near the surface.
    Link { to: Point3 },
    /// Pause for `seconds`.
    Dwell { seconds: f64 },
    /// Tool change to `tool`.
    ToolChange { tool: ToolId },
    /// A canned hole cycle executed at `at` (XY; Z is the retract plane).
    Cycle { at: Point3, cycle: Cycle },
    /// Spindle on at `rpm`.
    SpindleOn { rpm: f64, direction: SpindleDirection },
    SpindleOff,
    /// Orient the stopped spindle to `angle_deg` (fine boring, back boring: M19).
    SpindleOrient { angle_deg: f64 },
    /// Rigid tapping mode on/off (spindle synchronized with the feed: M29).
    SpindleSync { enabled: bool },
    /// Coolant mode change.
    Coolant { mode: Coolant },
    /// A comment passed to the post verbatim (operation start, notes).
    Comment { text: String },
    /// Tilted work plane on (G68.2 / CYCLE800 / PLANE SPATIAL) for a 3+2
    /// indexed section. `origin`, `x_axis` and `z_axis` are in setup
    /// coordinates; positions in the stream STAY in setup coordinates (the
    /// post derives the local coordinates); `z_axis` is the section's tool
    /// axis, and canned-cycle depths inside the section run along it.
    WorkPlane { origin: Point3, x_axis: Point3, z_axis: Point3 },
    /// Tilted work plane off (G69 / CYCLE800() / PLANE RESET).
    WorkPlaneOff,
    /// Position the rotary axes with the tool clear of the part (3+2
    /// indexing); angles in degrees in the machine's axis order.
    RotaryPosition { axes: Vec<RotaryWord> },
    /// Rotary tool-centre-point control on/off (G43.4 / TRAORI / M128).
    TcpMode { enabled: bool },
    /// Enter G18 XZ turning; internal X coordinates remain radius. Diameter
    /// mode is post conversion only.
    LatheMode { diameter_mode: bool },
    /// G96 constant surface speed with a G50 maximum rpm; units/min
    /// consistent with model units.
    SpindleSurfaceSpeed { surface_speed: f64, max_rpm: f64, direction: SpindleDirection },
    /// G33/G32 synchronized constant-pitch thread cut; the start is the
    /// previous position. `pitch` is the authoritative synchronisation: never
    /// convert it to an ordinary feed move silently.
    ThreadCut { to: Point3, pitch: f64 },
    /// Canned turning thread schedule; the expanded equivalent is one
    /// `ThreadCut` per pass.
    TurnThreadCycle { cycle: TurnThreadCycle },
    /// Polar interpolation on/off (G12.1 / TRANSMIT): XY programmed as X and C (lane C11b).
    PolarInterpolation { enabled: bool },
    /// Cylindrical interpolation (G7.1 / TRACYL) on a cylinder of `radius`;
    /// `None` cancels it.
    CylindricalInterpolation { radius: Option<f64> },
    /// Synchronized linear and rotary feed move: the linear axes to `to` while
    /// the rotary axes reach `axes` (lane C11b).
    LinearRotary { to: Point3, axes: Vec<RotaryWord> },
    /// Program stop (M0) or optional stop (M1) (lane C15).
    Stop { optional: bool },
    /// Explicit user text passed to the post byte for byte, in order; never
    /// interpolated or generated (lane C15).
    PassThrough { text: String },
}

/// A canned turning thread schedule (G76-class). X is radius.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TurnThreadCycle {
    pub start: Point3,
    pub end: Point3,
    pub pitch: f64,
    pub depths: Vec<f64>,
    pub infeed_offsets: Vec<f64>,
    pub clearance_x: f64,
    pub internal: bool,
    pub spring_passes: u32,
}

impl TurnThreadCycle {
    pub fn is_finite(&self) -> bool {
        self.start.is_finite()
            && self.end.is_finite()
            && self.pitch.is_finite()
            && self.clearance_x.is_finite()
            && self.depths.iter().all(|d| d.is_finite())
            && self.infeed_offsets.iter().all(|d| d.is_finite())
    }
}

/// One rotary axis word of a [`MotionKind::RotaryPosition`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RotaryWord {
    /// Axis letter (`A`, `B`, `C`).
    pub name: String,
    pub angle_deg: f64,
}

impl MotionKind {
    /// Destination of a motion that moves the tool.
    pub fn end(&self) -> Option<Point3> {
        match self {
            MotionKind::Rapid { to }
            | MotionKind::Plunge { to }
            | MotionKind::Cut { to }
            | MotionKind::ArcCw { to, .. }
            | MotionKind::ArcCcw { to, .. }
            | MotionKind::Retract { to }
            | MotionKind::LeadIn { to }
            | MotionKind::LeadOut { to }
            | MotionKind::Link { to }
            | MotionKind::ThreadCut { to, .. }
            | MotionKind::LinearRotary { to, .. } => Some(*to),
            MotionKind::Probe { cycle } => Some(cycle.target),
            MotionKind::Cycle { at, .. } => Some(*at),
            // The canned thread returns to the clearance X on the start's Z.
            MotionKind::TurnThreadCycle { cycle } => Some(Point3::new(cycle.clearance_x, 0.0, cycle.start.z)),
            MotionKind::Dwell { .. }
            | MotionKind::ToolChange { .. }
            | MotionKind::SpindleOn { .. }
            | MotionKind::SpindleOff
            | MotionKind::SpindleOrient { .. }
            | MotionKind::SpindleSync { .. }
            | MotionKind::Coolant { .. }
            | MotionKind::Comment { .. }
            | MotionKind::WorkPlane { .. }
            | MotionKind::WorkPlaneOff
            | MotionKind::RotaryPosition { .. }
            | MotionKind::TcpMode { .. }
            | MotionKind::LatheMode { .. }
            | MotionKind::SpindleSurfaceSpeed { .. }
            | MotionKind::PolarInterpolation { .. }
            | MotionKind::CylindricalInterpolation { .. }
            | MotionKind::Stop { .. }
            | MotionKind::PassThrough { .. } => None,
        }
    }

    /// Motions that remove material (cuts, arcs, leads, cycles, plunges).
    pub fn is_cutting(&self) -> bool {
        matches!(
            self,
            MotionKind::Plunge { .. }
                | MotionKind::Cut { .. }
                | MotionKind::ArcCw { .. }
                | MotionKind::ArcCcw { .. }
                | MotionKind::LeadIn { .. }
                | MotionKind::LeadOut { .. }
                | MotionKind::Cycle { .. }
                | MotionKind::ThreadCut { .. }
                | MotionKind::TurnThreadCycle { .. }
                | MotionKind::LinearRotary { .. }
        )
    }

    /// Stable lowercase name (the serde tag).
    pub fn kind_name(&self) -> &'static str {
        match self {
            MotionKind::Probe { .. } => "probe",
            MotionKind::Rapid { .. } => "rapid",
            MotionKind::Plunge { .. } => "plunge",
            MotionKind::Cut { .. } => "cut",
            MotionKind::ArcCw { .. } => "arcCw",
            MotionKind::ArcCcw { .. } => "arcCcw",
            MotionKind::Retract { .. } => "retract",
            MotionKind::LeadIn { .. } => "leadIn",
            MotionKind::LeadOut { .. } => "leadOut",
            MotionKind::Link { .. } => "link",
            MotionKind::Dwell { .. } => "dwell",
            MotionKind::ToolChange { .. } => "toolChange",
            MotionKind::Cycle { .. } => "cycle",
            MotionKind::SpindleOn { .. } => "spindleOn",
            MotionKind::SpindleOff => "spindleOff",
            MotionKind::SpindleOrient { .. } => "spindleOrient",
            MotionKind::SpindleSync { .. } => "spindleSync",
            MotionKind::Coolant { .. } => "coolant",
            MotionKind::Comment { .. } => "comment",
            MotionKind::WorkPlane { .. } => "workPlane",
            MotionKind::WorkPlaneOff => "workPlaneOff",
            MotionKind::RotaryPosition { .. } => "rotaryPosition",
            MotionKind::TcpMode { .. } => "tcpMode",
            MotionKind::LatheMode { .. } => "latheMode",
            MotionKind::SpindleSurfaceSpeed { .. } => "spindleSurfaceSpeed",
            MotionKind::ThreadCut { .. } => "threadCut",
            MotionKind::TurnThreadCycle { .. } => "turnThreadCycle",
            MotionKind::PolarInterpolation { .. } => "polarInterpolation",
            MotionKind::CylindricalInterpolation { .. } => "cylindricalInterpolation",
            MotionKind::LinearRotary { .. } => "linearRotary",
            MotionKind::Stop { .. } => "stop",
            MotionKind::PassThrough { .. } => "passThrough",
        }
    }

    /// Every coordinate finite.
    pub fn is_finite(&self) -> bool {
        match self {
            MotionKind::ArcCw { to, center, pitch, .. } | MotionKind::ArcCcw { to, center, pitch, .. } => {
                to.is_finite() && center.is_finite() && pitch.is_finite()
            }
            MotionKind::Probe { cycle } => cycle.is_finite(),
            MotionKind::Dwell { seconds } => seconds.is_finite(),
            MotionKind::SpindleOn { rpm, .. } => rpm.is_finite(),
            MotionKind::SpindleOrient { angle_deg } => angle_deg.is_finite(),
            MotionKind::Cycle { at, cycle } => at.is_finite() && cycle.is_finite(),
            MotionKind::WorkPlane { origin, x_axis, z_axis } => {
                origin.is_finite() && x_axis.is_finite() && z_axis.is_finite()
            }
            MotionKind::RotaryPosition { axes } => axes.iter().all(|a| a.angle_deg.is_finite()),
            MotionKind::LatheMode { .. } => true,
            MotionKind::SpindleSurfaceSpeed { surface_speed, max_rpm, .. } => surface_speed.is_finite() && max_rpm.is_finite(),
            MotionKind::ThreadCut { to, pitch } => to.is_finite() && pitch.is_finite(),
            MotionKind::TurnThreadCycle { cycle } => cycle.is_finite(),
            MotionKind::CylindricalInterpolation { radius } => radius.map_or(true, |r| r.is_finite() && r > 0.0),
            MotionKind::LinearRotary { to, axes } => to.is_finite() && axes.iter().all(|a| a.angle_deg.is_finite()),
            other => other.end().map_or(true, |p| p.is_finite()),
        }
    }
}

/// A canned hole-making cycle. Depths are absolute MACHINE Z of the hole
/// bottom; `retract_z` is the plane the cycle retracts to.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
#[non_exhaustive]
pub enum Cycle {
    /// G81 / G82 (with dwell).
    Drill {
        depth_z: f64,
        retract_z: f64,
        #[serde(default)]
        dwell_seconds: f64,
    },
    /// G83 deep drilling with full retract between pecks.
    Peck { depth_z: f64, retract_z: f64, peck: f64 },
    /// G73 chip breaking with a short retract between pecks.
    ChipBreak {
        depth_z: f64,
        retract_z: f64,
        peck: f64,
        #[serde(default)]
        back_off: f64,
    },
    /// G84 tapping. `rigid` requests synchronous (rigid) tapping.
    Tap {
        depth_z: f64,
        retract_z: f64,
        pitch: f64,
        rigid: bool,
        #[serde(default)]
        left_hand: bool,
    },
    /// G85 reaming (feed in, feed out).
    Ream { depth_z: f64, retract_z: f64 },
    /// G76/G86/G89 boring. `shift` is the fine-boring tool shift, `dwell` at bottom.
    Bore {
        depth_z: f64,
        retract_z: f64,
        #[serde(default)]
        dwell_seconds: f64,
        #[serde(default)]
        shift: f64,
        #[serde(default)]
        feed_out: bool,
    },
    /// G87 back boring: `depth_z` is the finished (upper) face of the back cut;
    /// `entry_z` the axial entry position below it.
    BackBore {
        depth_z: f64,
        retract_z: f64,
        shift: f64,
        #[serde(default)]
        entry_z: f64,
    },
}

impl Cycle {
    pub fn depth_z(&self) -> f64 {
        match self {
            Cycle::Drill { depth_z, .. }
            | Cycle::Peck { depth_z, .. }
            | Cycle::ChipBreak { depth_z, .. }
            | Cycle::Tap { depth_z, .. }
            | Cycle::Ream { depth_z, .. }
            | Cycle::Bore { depth_z, .. }
            | Cycle::BackBore { depth_z, .. } => *depth_z,
        }
    }

    pub fn retract_z(&self) -> f64 {
        match self {
            Cycle::Drill { retract_z, .. }
            | Cycle::Peck { retract_z, .. }
            | Cycle::ChipBreak { retract_z, .. }
            | Cycle::Tap { retract_z, .. }
            | Cycle::Ream { retract_z, .. }
            | Cycle::Bore { retract_z, .. }
            | Cycle::BackBore { retract_z, .. } => *retract_z,
        }
    }

    pub fn is_finite(&self) -> bool {
        let extra = match self {
            Cycle::Drill { dwell_seconds, .. } => dwell_seconds.is_finite(),
            Cycle::Peck { peck, .. } => peck.is_finite(),
            Cycle::ChipBreak { peck, back_off, .. } => peck.is_finite() && back_off.is_finite(),
            Cycle::Tap { pitch, .. } => pitch.is_finite(),
            Cycle::Ream { .. } => true,
            Cycle::Bore { dwell_seconds, shift, .. } => dwell_seconds.is_finite() && shift.is_finite(),
            Cycle::BackBore { shift, entry_z, .. } => shift.is_finite() && entry_z.is_finite(),
        };
        extra && self.depth_z().is_finite() && self.retract_z().is_finite()
    }
}

/// One motion with its feed and optional multi-axis tool orientation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Motion {
    #[serde(flatten)]
    pub kind: MotionKind,
    #[serde(default)]
    pub feed: Feed,
    /// Tool axis direction at the end of the motion (`None`: machine +Z).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool_axis: Option<Point3>,
    /// Id of the source path/pass within the operation, for preview grouping
    /// and slider snap points.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path_id: Option<u32>,
}

impl Motion {
    pub fn new(kind: MotionKind, feed: Feed) -> Self {
        Self { kind, feed, tool_axis: None, path_id: None }
    }

    pub fn rapid(to: Point3) -> Self {
        Self::new(MotionKind::Rapid { to }, Feed::RAPID)
    }

    pub fn plunge(to: Point3, feed: f64) -> Self {
        Self::new(MotionKind::Plunge { to }, Feed::Rate(feed))
    }

    pub fn cut(to: Point3, feed: f64) -> Self {
        Self::new(MotionKind::Cut { to }, Feed::Rate(feed))
    }

    pub fn retract(to: Point3, feed: Feed) -> Self {
        Self::new(MotionKind::Retract { to }, feed)
    }

    pub fn link(to: Point3, feed: f64) -> Self {
        Self::new(MotionKind::Link { to }, Feed::Rate(feed))
    }

    pub fn with_path(mut self, path_id: u32) -> Self {
        self.path_id = Some(path_id);
        self
    }

    pub fn end(&self) -> Option<Point3> {
        self.kind.end()
    }
}

/// The result of generating one operation.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Toolpath {
    pub operation: OperationId,
    pub tool: ToolId,
    #[serde(default)]
    pub motions: Vec<Motion>,
    #[serde(default)]
    pub report: OperationReport,
}

impl Toolpath {
    pub fn new(operation: impl Into<OperationId>, tool: impl Into<ToolId>) -> Self {
        Self { operation: operation.into(), tool: tool.into(), motions: Vec::new(), report: OperationReport::default() }
    }

    /// Total length of cutting motions (straight-line approximation for arcs).
    pub fn cutting_length(&self) -> f64 {
        let mut total = 0.0;
        let mut at: Option<Point3> = None;
        for m in &self.motions {
            if let Some(end) = m.end() {
                if let Some(start) = at {
                    if m.kind.is_cutting() {
                        total += start.distance(end);
                    }
                }
                at = Some(end);
            }
        }
        total
    }

    /// Every motion coordinate finite; the first offender's index otherwise.
    pub fn first_non_finite(&self) -> Option<usize> {
        self.motions.iter().position(|m| {
            !m.kind.is_finite()
                || m.feed.value().map_or(false, |f| !f.is_finite())
                || m.tool_axis.map_or(false, |a| !a.is_finite())
        })
    }
}

/// A motion with the operation it came from, for program-level streams.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProgramMotion {
    pub operation: OperationId,
    pub motion: Motion,
}

/// Outcome of generating an operation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum OperationStatus {
    /// Not generated yet.
    #[default]
    Pending,
    /// Generated without warnings.
    Ok,
    /// Generated; the warnings describe degradations.
    Warning,
    /// Generation stopped; the errors say why. No usable path.
    Error,
    /// Cancelled; the previous accepted result stands.
    Cancelled,
}

/// A warning or error with a stable code.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Diagnostic {
    /// Stable machine-readable code, kebab-case (`empty-after-clipping`, `gouge-detected`).
    pub code: String,
    /// User-facing message.
    pub message: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
    /// Where it happened, in machine coordinates.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub location: Option<Point3>,
}

impl Diagnostic {
    pub fn new(code: impl Into<String>, message: impl Into<String>) -> Self {
        Self { code: code.into(), message: message.into(), detail: None, location: None }
    }

    pub fn at(mut self, location: Point3) -> Self {
        self.location = Some(location);
        self
    }

    pub fn detail(mut self, detail: impl Into<String>) -> Self {
        self.detail = Some(detail.into());
        self
    }
}

/// A region the operation requested but could not machine.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EmptyRegion {
    /// Why: `no-triangles`, `clipped-away`, `unreachable`, `below-floor`, ...
    pub reason: String,
    /// Drive face or boundary that named the region, when there is one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target: Option<super::TargetRef>,
}

/// Where the generated path knowingly deviates from the exact surface.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ApproximationLimit {
    /// `chord-tolerance`, `arc-discretized`, `sampling-cap`, `solver-tolerance`, ...
    pub kind: String,
    /// Deviation bound in model units when known.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bound: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
}

/// Work counters a generation reports.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct WorkCounts {
    pub triangles: u64,
    pub candidates: u64,
    pub samples: u64,
    pub contacts: u64,
    pub passes: u64,
    pub motions: u64,
}

/// Status, diagnostics and counters for one operation.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct OperationReport {
    #[serde(default)]
    pub status: OperationStatus,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub errors: Vec<Diagnostic>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<Diagnostic>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub empty_regions: Vec<EmptyRegion>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub approximations: Vec<ApproximationLimit>,
    #[serde(default)]
    pub counts: WorkCounts,
    /// Estimated machining time in seconds, when the strategy computes one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub estimated_seconds: Option<f64>,
}

impl OperationReport {
    pub fn warn(&mut self, d: Diagnostic) {
        self.warnings.push(d);
        if self.status == OperationStatus::Ok || self.status == OperationStatus::Pending {
            self.status = OperationStatus::Warning;
        }
    }

    pub fn fail(&mut self, d: Diagnostic) {
        self.errors.push(d);
        self.status = OperationStatus::Error;
    }

    /// Mark generated: `Ok` unless warnings or errors were recorded.
    pub fn finish(&mut self) {
        self.status = if !self.errors.is_empty() {
            OperationStatus::Error
        } else if !self.warnings.is_empty() {
            OperationStatus::Warning
        } else {
            OperationStatus::Ok
        };
    }

    pub fn is_usable(&self) -> bool {
        matches!(self.status, OperationStatus::Ok | OperationStatus::Warning)
    }
}
