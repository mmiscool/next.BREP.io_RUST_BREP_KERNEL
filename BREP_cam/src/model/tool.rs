//! Tools, holders, feeds and speeds.

use super::{HolderId, ToolId};
use serde::{Deserialize, Serialize};

/// A cutting tool as the user defines it. Geometry is a flat parameter set
/// (`None` where the kind does not use a field); [`crate::cutter::Cutter`]
/// turns it into a validated profile for the algorithms.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Tool {
    pub id: ToolId,
    /// Tool number in the machine's carousel (`T` word).
    #[serde(default)]
    pub number: u32,
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub description: String,
    #[serde(default)]
    pub kind: ToolKind,
    #[serde(default)]
    pub geometry: ToolGeometry,
    /// Explicit insert/blade geometry for turning; never inferred from a mill diameter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turning_geometry: Option<crate::turning::TurningGeometry>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holder: Option<Holder>,
    #[serde(default)]
    pub coolant: Coolant,
    #[serde(default)]
    pub feeds: FeedsSpeeds,
    /// Length offset register (`H` word), when it differs from `number`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub length_offset: Option<u32>,
    /// Diameter offset register (`D` word), when it differs from `number`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub diameter_offset: Option<u32>,
    /// Material this tool is made of, free text (carbide, HSS, ...).
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub material: String,
}

impl Tool {
    pub fn new(id: impl Into<ToolId>, number: u32, kind: ToolKind, geometry: ToolGeometry) -> Self {
        Self { id: id.into(), number, kind, geometry, ..Default::default() }
    }

    pub fn radius(&self) -> f64 {
        self.geometry.diameter * 0.5
    }
}

/// Tool families from the capability matrix.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
#[non_exhaustive]
pub enum ToolKind {
    /// Flat end mill.
    #[default]
    Flat,
    /// Ball nose end mill.
    Ball,
    /// Bull nose / corner-radius end mill.
    Bull,
    /// Chamfer mill (conical, uses `included_angle_deg`, optional tip diameter).
    Chamfer,
    /// V-bit / engraving cone (conical to a point).
    VBit,
    /// Tapered mill with a ball end (ball-cone compound).
    TaperedBall,
    /// Tapered mill with a flat end.
    TaperedFlat,
    /// Lollipop (undercut) cutter.
    Lollipop,
    /// Face mill (large flat, indexable).
    FaceMill,
    /// Slot cutter (side-and-face).
    Slot,
    /// T-slot cutter.
    TSlot,
    /// Dovetail cutter.
    Dovetail,
    /// Engraver (fine V point).
    Engraver,
    /// Thread mill.
    ThreadMill,
    /// Twist drill.
    Drill,
    /// Spot drill.
    SpotDrill,
    /// Centre drill.
    CenterDrill,
    /// Tap (right-hand unless `left_hand`).
    Tap,
    /// Reamer.
    Reamer,
    /// Boring bar (milling).
    BoringBar,
    /// Lathe insert tool (turning).
    LatheInsert,
}

impl ToolKind {
    /// Stable lowercase name, the same string serde writes.
    pub fn as_str(&self) -> &'static str {
        match self {
            ToolKind::Flat => "flat",
            ToolKind::Ball => "ball",
            ToolKind::Bull => "bull",
            ToolKind::Chamfer => "chamfer",
            ToolKind::VBit => "v-bit",
            ToolKind::TaperedBall => "tapered-ball",
            ToolKind::TaperedFlat => "tapered-flat",
            ToolKind::Lollipop => "lollipop",
            ToolKind::FaceMill => "face-mill",
            ToolKind::Slot => "slot",
            ToolKind::TSlot => "t-slot",
            ToolKind::Dovetail => "dovetail",
            ToolKind::Engraver => "engraver",
            ToolKind::ThreadMill => "thread-mill",
            ToolKind::Drill => "drill",
            ToolKind::SpotDrill => "spot-drill",
            ToolKind::CenterDrill => "center-drill",
            ToolKind::Tap => "tap",
            ToolKind::Reamer => "reamer",
            ToolKind::BoringBar => "boring-bar",
            ToolKind::LatheInsert => "lathe-insert",
        }
    }

    /// Hole-making tools that run canned cycles rather than milling paths.
    pub fn is_hole_tool(&self) -> bool {
        matches!(
            self,
            ToolKind::Drill
                | ToolKind::SpotDrill
                | ToolKind::CenterDrill
                | ToolKind::Tap
                | ToolKind::Reamer
                | ToolKind::BoringBar
        )
    }
}

/// Tool dimensions in model units. Angles in degrees.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ToolGeometry {
    /// Maximum cutting diameter.
    pub diameter: f64,
    /// Usable cutting (flute) length.
    pub cutting_length: f64,
    /// Shank diameter above the flutes (`None`: equals `diameter`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shaft_diameter: Option<f64>,
    /// Shank length above the cutting length used for collision checks.
    #[serde(default = "default_shaft_length")]
    pub shaft_length: f64,
    /// Overall length from tip to holder gauge face (`None`: cutting + shaft).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub overall_length: Option<f64>,
    /// Corner radius (bull nose). `None` or `0` for sharp corners.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_radius: Option<f64>,
    /// Included tip angle for cones, chamfers, V-bits, drills and spot drills.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub included_angle_deg: Option<f64>,
    /// Ball diameter of a tapered ball (ball-cone) tool; `None` means the full diameter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ball_diameter: Option<f64>,
    /// Tip (small-end) diameter of a chamfer or tapered flat tool.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_diameter: Option<f64>,
    /// Thread pitch (taps and thread mills).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub thread_pitch: Option<f64>,
    /// Number of flutes/teeth; feeds per tooth multiply by it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flute_count: Option<u32>,
    /// Left-hand cutting direction/thread.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub left_hand: bool,
}

fn default_shaft_length() -> f64 {
    30.0
}

impl Default for ToolGeometry {
    fn default() -> Self {
        Self {
            diameter: 6.0,
            cutting_length: 20.0,
            shaft_diameter: None,
            shaft_length: default_shaft_length(),
            overall_length: None,
            corner_radius: None,
            included_angle_deg: None,
            ball_diameter: None,
            tip_diameter: None,
            thread_pitch: None,
            flute_count: None,
            left_hand: false,
        }
    }
}

impl ToolGeometry {
    pub fn flat(diameter: f64, cutting_length: f64) -> Self {
        Self { diameter, cutting_length, ..Default::default() }
    }

    pub fn shaft_diameter(&self) -> f64 {
        self.shaft_diameter.unwrap_or(self.diameter)
    }

    pub fn overall_length(&self) -> f64 {
        self.overall_length.unwrap_or(self.cutting_length + self.shaft_length)
    }
}

/// A tool holder as stacked frusta from the tool side (bottom) upward.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Holder {
    #[serde(default)]
    pub id: HolderId,
    #[serde(default)]
    pub name: String,
    /// Bottom segment first.
    #[serde(default)]
    pub segments: Vec<HolderSegment>,
}

/// One frustum of a holder profile.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct HolderSegment {
    pub height: f64,
    /// Diameter at the bottom (tool side) of this segment.
    pub lower_diameter: f64,
    /// Diameter at the top of this segment.
    pub upper_diameter: f64,
}

impl Holder {
    /// Total holder height (sum of segments).
    pub fn height(&self) -> f64 {
        self.segments.iter().map(|s| s.height).sum()
    }

    /// Holder radius at `h` above the holder's bottom face, `None` above the holder.
    pub fn radius_at_height(&self, h: f64) -> Option<f64> {
        if h < 0.0 {
            return None;
        }
        let mut base = 0.0;
        for s in &self.segments {
            if h <= base + s.height {
                let t = if s.height > 0.0 { (h - base) / s.height } else { 0.0 };
                return Some(0.5 * (s.lower_diameter + t * (s.upper_diameter - s.lower_diameter)));
            }
            base += s.height;
        }
        None
    }

    pub fn validate(&self) -> Result<(), String> {
        for (i, s) in self.segments.iter().enumerate() {
            let finite = s.height.is_finite() && s.lower_diameter.is_finite() && s.upper_diameter.is_finite();
            if !finite {
                return Err(format!("holder segment {i} has a non-finite dimension"));
            }
            if s.height <= 0.0 || s.lower_diameter <= 0.0 || s.upper_diameter <= 0.0 {
                return Err(format!("holder segment {i} must have positive dimensions"));
            }
        }
        Ok(())
    }
}

/// Coolant request posted with the tool.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Coolant {
    #[default]
    Off,
    Flood,
    Mist,
    ThroughTool,
    Air,
    FloodAndThroughTool,
    FloodAndMist,
}

/// Spindle rotation sense.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SpindleDirection {
    #[default]
    Cw,
    Ccw,
}

impl SpindleDirection {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["cw", "ccw"];
}

/// Default feeds and speeds for a tool. Feeds in model units per minute.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FeedsSpeeds {
    pub spindle_rpm: f64,
    #[serde(default)]
    pub spindle_direction: SpindleDirection,
    pub cutting_feed: f64,
    pub plunge_feed: f64,
    /// Lead-in/lead-out feed (`None`: cutting feed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub lead_feed: Option<f64>,
    /// Ramp feed (`None`: cutting feed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub ramp_feed: Option<f64>,
    /// Retract feed when retracting at feed (`None`: rapid).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retract_feed: Option<f64>,
    /// Surface speed the rpm was derived from, informational.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed: Option<f64>,
    /// Feed per tooth the cutting feed was derived from, informational.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed_per_tooth: Option<f64>,
    /// Maximum rpm under constant surface speed (turning, G50); required when
    /// `css` is on.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub surface_speed_limit_rpm: Option<f64>,
    /// Constant surface speed (G96) for turning: an explicit choice. Off, the
    /// spindle runs at `spindle_rpm` and `surface_speed` is the informational
    /// cutting speed (milling Vc).
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub css: bool,
}

impl Default for FeedsSpeeds {
    fn default() -> Self {
        Self {
            spindle_rpm: 10_000.0,
            spindle_direction: SpindleDirection::Cw,
            cutting_feed: 1_000.0,
            plunge_feed: 300.0,
            lead_feed: None,
            ramp_feed: None,
            retract_feed: None,
            surface_speed: None,
            feed_per_tooth: None,
            surface_speed_limit_rpm: None,
            css: false,
        }
    }
}

impl FeedsSpeeds {
    pub fn lead_feed(&self) -> f64 {
        self.lead_feed.unwrap_or(self.cutting_feed)
    }

    pub fn ramp_feed(&self) -> f64 {
        self.ramp_feed.unwrap_or(self.cutting_feed)
    }

    pub fn validate(&self) -> Result<(), String> {
        let fields = [
            ("spindle rpm", self.spindle_rpm, false),
            ("cutting feed", self.cutting_feed, true),
            ("plunge feed", self.plunge_feed, true),
        ];
        for (name, v, positive) in fields {
            if !v.is_finite() || v < 0.0 || (positive && v == 0.0) {
                return Err(format!("{name} must be finite and positive"));
            }
        }
        for (name, v) in [
            ("lead feed", self.lead_feed),
            ("ramp feed", self.ramp_feed),
            ("retract feed", self.retract_feed),
        ] {
            if let Some(v) = v {
                if !v.is_finite() || v <= 0.0 {
                    return Err(format!("{name} must be finite and positive"));
                }
            }
        }
        if let Some(v) = self.surface_speed {
            if !v.is_finite() || v <= 0.0 {
                return Err("surface speed must be finite and positive".into());
            }
        }
        if let Some(max) = self.surface_speed_limit_rpm {
            if !max.is_finite() || max <= 0.0 {
                return Err("surface speed limit rpm must be finite and positive".into());
            }
        }
        if self.css {
            if self.surface_speed.is_none() {
                return Err("constant surface speed needs a surface speed".into());
            }
            if self.surface_speed_limit_rpm.is_none() {
                return Err("constant surface speed needs a finite positive maximum rpm".into());
            }
        }
        Ok(())
    }
}
