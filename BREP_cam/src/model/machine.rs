//! Machine definitions: kind, kinematics and limits.

use super::{MachineId, Point3};
use serde::{Deserialize, Serialize};

/// A machine tool the setup is posted for.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Machine {
    pub id: MachineId,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub kind: MachineKind,
    /// Linear travel limits in machine coordinates (`None`: unlimited).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub travel: Option<TravelLimits>,
    #[serde(default)]
    pub spindle: SpindleLimits,
    #[serde(default)]
    pub feed: FeedLimits,
    /// Identifier of the post-processor script this machine uses.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub post: Option<String>,
    /// Seconds a tool change takes, for cycle-time estimates.
    #[serde(default = "default_tool_change_seconds")]
    pub tool_change_seconds: f64,
    /// Collision geometry: solids attached to the chain's links (lane C10c/C17).
    #[serde(default, skip_serializing_if = "MachineModel::is_empty")]
    pub collision: MachineModel,
}

fn default_tool_change_seconds() -> f64 {
    5.0
}

impl Default for Machine {
    fn default() -> Self {
        Self {
            id: MachineId::default(),
            name: String::new(),
            kind: MachineKind::default(),
            travel: None,
            spindle: SpindleLimits::default(),
            feed: FeedLimits::default(),
            post: None,
            tool_change_seconds: default_tool_change_seconds(),
            collision: MachineModel::default(),
        }
    }
}

/// A rigid link of the machine that solids attach to (the kinematics chain
/// resolves its pose).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum Link {
    /// The bed and column: never moves.
    Base,
    /// The link carried by joint `index` of the chain (table or head).
    Joint { index: usize },
    /// The part (and its fixture and stock): the innermost table joint.
    Part,
    /// The spindle (and tool): the innermost head joint plus the linear axes.
    Tool,
}

/// Shape of a machine solid in its link's frame. Geometric queries
/// (containment, surface sampling) live with the simulation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum LinkShape {
    Box { min: Point3, max: Point3 },
    Cylinder { base: Point3, axis: Point3, radius: f64, height: f64 },
}

impl LinkShape {
    pub fn validate(&self) -> Result<(), String> {
        match self {
            LinkShape::Box { min, max } => {
                if !min.is_finite() || !max.is_finite() || min.x >= max.x || min.y >= max.y || min.z >= max.z {
                    return Err("machine solid: box needs finite min < max on every axis".into());
                }
            }
            LinkShape::Cylinder { base, axis, radius, height } => {
                if !base.is_finite()
                    || axis.normalized().is_none()
                    || !(*radius > 0.0)
                    || !(*height > 0.0)
                    || !radius.is_finite()
                    || !height.is_finite()
                {
                    return Err("machine solid: cylinder needs a finite base, a non-zero axis and positive radius and height".into());
                }
            }
        }
        Ok(())
    }
}

/// One machine solid: a shape attached to a link.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LinkSolid {
    pub name: String,
    pub link: Link,
    pub shape: LinkShape,
}

/// The machine's collision geometry.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct MachineModel {
    pub solids: Vec<LinkSolid>,
}

impl MachineModel {
    pub fn is_empty(&self) -> bool {
        self.solids.is_empty()
    }

    pub fn validate(&self) -> Result<(), String> {
        for solid in &self.solids {
            if solid.name.is_empty() {
                return Err("machine solid needs a name".into());
            }
            solid.shape.validate().map_err(|m| format!("{m} ({})", solid.name))?;
        }
        Ok(())
    }
}

/// Machine family and, for multi-axis mills, the rotary arrangement.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
#[non_exhaustive]
pub enum MachineKind {
    /// Three linear axes.
    Mill3,
    /// Three linear axes plus one rotary (wrap/indexing).
    Mill4 { rotary: RotaryAxis },
    /// Five-axis mill with the given kinematic arrangement.
    Mill5 { kinematics: Kinematics },
    /// Two-axis lathe (X/Z), optional C and live tooling later.
    Lathe,
    /// Lathe with a C axis (and optionally Y) for live-tool milling (lane C11b).
    /// `c_axis` must be named `C`, point along +Z and sit on the spindle axis.
    MillTurn { c_axis: RotaryAxis, y_axis: bool },
}

impl Default for MachineKind {
    fn default() -> Self {
        MachineKind::Mill3
    }
}

/// Where the two rotary axes of a five-axis machine live.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum Kinematics {
    /// Both rotaries carry the part (trunnion). `primary` carries `secondary`.
    TableTable { primary: RotaryAxis, secondary: RotaryAxis },
    /// Both rotaries carry the spindle.
    HeadHead { primary: RotaryAxis, secondary: RotaryAxis },
    /// One rotary on the head (tilts the tool), one on the table (turns the part).
    HeadTable { head: RotaryAxis, table: RotaryAxis },
}

/// A rotary axis: its name, direction and pivot in machine coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct RotaryAxis {
    /// Axis letter the post emits (`A`, `B`, `C`).
    pub name: String,
    /// Unit direction of the rotation axis.
    pub direction: Point3,
    /// A point on the axis.
    #[serde(default)]
    pub origin: Point3,
    /// Angular limits in degrees (`None`: continuous).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub limits_deg: Option<[f64; 2]>,
}

/// Linear travel limits in machine coordinates.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TravelLimits {
    pub min: Point3,
    pub max: Point3,
}

/// Spindle speed range in rpm.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct SpindleLimits {
    pub min_rpm: f64,
    pub max_rpm: f64,
}

impl Default for SpindleLimits {
    fn default() -> Self {
        Self { min_rpm: 0.0, max_rpm: 24_000.0 }
    }
}

/// Feed limits in model units per minute.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct FeedLimits {
    /// Maximum cutting feed.
    pub max_cutting: f64,
    /// Rapid traverse rate (used for cycle-time estimates).
    pub rapid: f64,
    /// Axis acceleration in model units per second squared, for cycle-time
    /// estimates (`None`: instantaneous).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub acceleration: Option<f64>,
}

impl Default for FeedLimits {
    fn default() -> Self {
        Self { max_cutting: 10_000.0, rapid: 30_000.0, acceleration: None }
    }
}

impl Machine {
    pub fn new(id: impl Into<MachineId>, name: impl Into<String>, kind: MachineKind) -> Self {
        Self { id: id.into(), name: name.into(), kind, ..Default::default() }
    }

    pub fn validate(&self) -> Result<(), String> {
        if !(self.spindle.max_rpm >= self.spindle.min_rpm) || self.spindle.min_rpm < 0.0 {
            return Err("spindle limits must satisfy 0 <= min <= max".into());
        }
        if !(self.feed.max_cutting > 0.0) || !(self.feed.rapid > 0.0) {
            return Err("feed limits must be positive".into());
        }
        if let Some(a) = self.feed.acceleration {
            if !a.is_finite() || a <= 0.0 {
                return Err("acceleration must be finite and positive".into());
            }
        }
        if !self.tool_change_seconds.is_finite() || self.tool_change_seconds < 0.0 {
            return Err("tool change time must be finite and non-negative".into());
        }
        self.collision.validate()?;
        if let Some(t) = &self.travel {
            if !t.min.is_finite() || !t.max.is_finite() {
                return Err("travel limits are not finite".into());
            }
            if t.min.x > t.max.x || t.min.y > t.max.y || t.min.z > t.max.z {
                return Err("travel limits must have min <= max".into());
            }
        }
        if let MachineKind::MillTurn { c_axis, .. } = &self.kind {
            if !matches!(c_axis.name.as_str(), "C" | "c") {
                return Err("mill-turn C axis must be named C".into());
            }
            if c_axis.direction.normalized().map_or(true, |d| d.distance(Point3::Z) > 1e-9) {
                return Err("mill-turn C axis must point along +Z".into());
            }
            if c_axis.origin.x.abs() > 1e-9 || c_axis.origin.y.abs() > 1e-9 {
                return Err("mill-turn C axis origin must lie on the spindle axis (x = y = 0)".into());
            }
        }
        let axes: Vec<&RotaryAxis> = match &self.kind {
            MachineKind::Mill3 | MachineKind::Lathe => Vec::new(),
            MachineKind::MillTurn { c_axis, .. } => vec![c_axis],
            MachineKind::Mill4 { rotary } => vec![rotary],
            MachineKind::Mill5 { kinematics } => match kinematics {
                Kinematics::TableTable { primary, secondary }
                | Kinematics::HeadHead { primary, secondary } => vec![primary, secondary],
                Kinematics::HeadTable { head, table } => vec![head, table],
            },
        };
        for axis in axes {
            axis.validate()?;
        }
        Ok(())
    }
}

impl RotaryAxis {
    pub fn validate(&self) -> Result<(), String> {
        if self.name.is_empty() {
            return Err("rotary axis has no name".into());
        }
        if self.direction.normalized().is_none() || !self.origin.is_finite() {
            return Err(format!("rotary axis {} has an invalid direction or origin", self.name));
        }
        if let Some([lo, hi]) = self.limits_deg {
            if !lo.is_finite() || !hi.is_finite() || lo > hi {
                return Err(format!("rotary axis {} has invalid limits", self.name));
            }
        }
        Ok(())
    }
}
