//! Cutter profiles: the rotationally symmetric shape of the tool below the
//! shank, as the drop-cutter, push-cutter, preview and simulation see it.
//!
//! Cutter-local coordinates: the axis is +Z, the tool REFERENCE POINT is the
//! origin. For flat and cone tools that is the physical tip/bottom; for ball
//! and bull tools it is the lowest point of the ball/toroid. `height_at_radius(r)`
//! is the profile height above the reference at radial distance `r`;
//! `radius_at_height(h)` the largest cutting radius at local height `h`.
//!
//! Profiles are piecewise: each [`ProfileSegment`] names its surface kind and
//! the radius and height intervals it owns, so compound cutters (ball-cone)
//! evaluate segment by segment and contact solvers can pick the formulation
//! for the surface they touch.

use crate::model::{Tool, ToolKind};
use serde::{Deserialize, Serialize};

mod extra;
mod solid;

pub use extra::{DovetailCutter, FrustumCutter, LollipopCutter};
pub use solid::{ProfilePoint, ToolMesh, ToolMeshGroup, ToolPart, ToolProfile, DEFAULT_CHORD, DEFAULT_RADIAL_SEGMENTS};

/// The five selected cutter shapes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum CutterKind {
    Flat,
    Ball,
    Bull,
    Cone,
    BallCone,
    /// Flat tip widening into a cone (chamfer mill, tapered flat mill).
    Frustum,
    /// Ball on a neck (undercutting).
    Lollipop,
    /// Inverted cone on a neck (undercutting).
    Dovetail,
}

impl CutterKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            CutterKind::Flat => "flat",
            CutterKind::Ball => "ball",
            CutterKind::Bull => "bull",
            CutterKind::Cone => "cone",
            CutterKind::BallCone => "ball-cone",
            CutterKind::Frustum => "frustum",
            CutterKind::Lollipop => "lollipop",
            CutterKind::Dovetail => "dovetail",
        }
    }
}

impl std::fmt::Display for CutterKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Why a cutter was refused. User-facing through `Display`.
#[derive(Clone, Debug, PartialEq)]
pub enum CutterError {
    NonFinite { field: &'static str },
    NotPositive { field: &'static str },
    /// Bull: `0 < corner_radius < radius` violated.
    CornerRadiusOutOfRange { corner_radius: f64, radius: f64 },
    /// Cone / ball-cone: `0 < included_angle < 180` violated.
    IncludedAngleOutOfRange { included_angle_deg: f64 },
    /// Ball-cone: `ball_diameter <= maximum_diameter` violated.
    BallLargerThanMaximum { ball_diameter: f64, maximum_diameter: f64 },
    /// The tool kind has no cutter profile (drills, taps, lathe inserts, ...).
    UnsupportedToolKind { kind: ToolKind },
    /// A field the tool kind requires is missing.
    MissingField { kind: ToolKind, field: &'static str },
    /// Two consecutive profile segments do not meet in height at `radius`.
    JoinGap { radius: f64, gap: f64 },
    /// Two consecutive curved segments meet with a corner at `radius`.
    JoinNotTangent { radius: f64, sine: f64 },
    /// Frustum: the tip must be narrower than the tool.
    TipNotSmallerThanDiameter { tip_diameter: f64, diameter: f64 },
    /// Lollipop / dovetail: the neck must be narrower than the tool.
    NeckNotSmallerThanDiameter { neck_diameter: f64, diameter: f64 },
}

impl std::fmt::Display for CutterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CutterError::NonFinite { field } => write!(f, "cutter {field} is not finite"),
            CutterError::NotPositive { field } => write!(f, "cutter {field} must be positive"),
            CutterError::CornerRadiusOutOfRange { corner_radius, radius } => write!(
                f,
                "corner radius {corner_radius} must be greater than 0 and smaller than the tool radius {radius}"
            ),
            CutterError::IncludedAngleOutOfRange { included_angle_deg } => {
                write!(f, "included angle {included_angle_deg} must be between 0 and 180 degrees")
            }
            CutterError::BallLargerThanMaximum { ball_diameter, maximum_diameter } => write!(
                f,
                "ball diameter {ball_diameter} must not exceed the maximum diameter {maximum_diameter}"
            ),
            CutterError::UnsupportedToolKind { kind } => {
                write!(f, "tool kind {} has no milling cutter profile", kind.as_str())
            }
            CutterError::MissingField { kind, field } => {
                write!(f, "tool kind {} needs {field}", kind.as_str())
            }
            CutterError::JoinGap { radius, gap } => {
                write!(f, "cutter profile has a height gap of {gap} at radius {radius}")
            }
            CutterError::JoinNotTangent { radius, sine } => {
                write!(f, "cutter profile has a corner at radius {radius} (join sine {sine})")
            }
            CutterError::TipNotSmallerThanDiameter { tip_diameter, diameter } => {
                write!(f, "tip diameter {tip_diameter} must be smaller than the diameter {diameter}")
            }
            CutterError::NeckNotSmallerThanDiameter { neck_diameter, diameter } => {
                write!(f, "neck diameter {neck_diameter} must be smaller than the diameter {diameter}")
            }
        }
    }
}

impl std::error::Error for CutterError {}

/// Surface of one profile segment.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum SegmentKind {
    /// Horizontal disc at `z_offset`.
    Flat,
    /// Sphere of radius `profile_radius` centred on the axis at `z_offset`.
    Sphere,
    /// Torus: tube radius `profile_radius`, ring radius `ring_radius`, centre height `z_offset`.
    Torus,
    /// Cone with half-angle `half_angle_rad`, apex on the axis at `z_offset`.
    Cone,
    /// Vertical cylinder of radius `profile_radius`.
    Cylinder,
    /// Upper half of a sphere (radius DEcreasing with height) centred on the
    /// axis at `z_offset`: a lollipop above its equator. Never a drop contact.
    SphereUpper,
    /// Inverted cone (radius decreasing with height) with half-angle
    /// `half_angle_rad`, apex on the axis at `z_offset` ABOVE the segment: a
    /// dovetail flank. Never a drop contact.
    ConeInverted,
}

/// One piece of a piecewise profile, with the parameters its surface needs.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProfileSegment {
    pub kind: SegmentKind,
    /// Radial interval `[r0, r1]` this segment owns.
    pub radius_range: [f64; 2],
    /// Height interval `[h0, h1]` this segment owns.
    pub height_range: [f64; 2],
    /// Sphere/torus tube radius or cylinder radius; `0` otherwise.
    pub profile_radius: f64,
    /// Torus ring radius; `0` otherwise.
    pub ring_radius: f64,
    /// Cone half-angle in radians; `0` otherwise.
    pub half_angle_rad: f64,
    /// Height of the sphere/torus centre, the cone apex, or the flat disc.
    pub z_offset: f64,
}

impl ProfileSegment {
    pub fn contains_radius(&self, r: f64) -> bool {
        r >= self.radius_range[0] && r <= self.radius_range[1]
    }

    pub fn contains_height(&self, h: f64) -> bool {
        h >= self.height_range[0] && h <= self.height_range[1]
    }
}

impl ProfileSegment {
    /// Relative distance to a sphere/torus equator below which the tangent is
    /// taken as vertical. Round-off there is ~1e-16·ρ; a 50 µm ball with a 1°
    /// taper has its join ~1e-6 (2e-5·ρ) from the equator and must keep its
    /// real tangent, so the threshold is relative, not `distance_eps`.
    pub const EQUATOR_SNAP: f64 = 1e-9;

    /// Unit tangent `(dr, dh)` of the profile curve at radius `r` on this
    /// segment, pointing outward/upward. Analytic; within `EQUATOR_SNAP` (relative) of
    /// a sphere's or torus's equator the tangent is snapped to vertical, because
    /// `sqrt(ρ² - d²)` there turns a one-ulp error in `d` into a ~1e-8 angle.
    /// `tol` is kept for callers that want to pass the policy object; the snap is relative.
    pub fn tangent_at(&self, r: f64, tol: &crate::numeric::Tolerance) -> [f64; 2] {
        let _ = tol;
        let unit = |dr: f64, dh: f64| {
            let n = (dr * dr + dh * dh).sqrt();
            if n > 0.0 {
                [dr / n, dh / n]
            } else {
                [1.0, 0.0]
            }
        };
        let round = |rho: f64, d: f64| {
            if rho - d.abs() <= Self::EQUATOR_SNAP * rho {
                [0.0, d.signum()]
            } else {
                unit((rho * rho - d * d).max(0.0).sqrt(), d)
            }
        };
        match self.kind {
            SegmentKind::Flat => [1.0, 0.0],
            SegmentKind::Cylinder => [0.0, 1.0],
            SegmentKind::Cone => unit(self.half_angle_rad.sin(), self.half_angle_rad.cos()),
            SegmentKind::Sphere => round(self.profile_radius, r),
            SegmentKind::Torus => round(self.profile_radius, r - self.ring_radius),
            SegmentKind::SphereUpper => {
                let t = round(self.profile_radius, r);
                [-t[0], t[1]]
            }
            SegmentKind::ConeInverted => unit(-self.half_angle_rad.sin(), self.half_angle_rad.cos()),
        }
    }
}

/// Every consecutive pair of segments meets in height, and meets tangentially
/// unless the pair is one of the physical sharp corners (flat bottom or cone
/// into the cylindrical shank). `distance_eps` bounds the height gap,
/// `angle_eps` the sine of the angle between the two tangents.
pub fn check_segment_joins(segments: &[ProfileSegment], tol: &crate::numeric::Tolerance) -> Result<(), CutterError> {
    for pair in segments.windows(2) {
        let (a, b) = (&pair[0], &pair[1]);
        let radius = a.radius_range[1];
        let gap = (a.height_range[1] - b.height_range[0]).abs();
        if gap > tol.distance_eps {
            return Err(CutterError::JoinGap { radius, gap });
        }
        // Physical sharp corners: a flat bottom's edge, a cone or inverted cone
        // meeting the shank, a lollipop's upper ball meeting its neck.
        let sharp_allowed = matches!(
            (a.kind, b.kind),
            (SegmentKind::Flat, SegmentKind::Cylinder)
                | (SegmentKind::Cone, SegmentKind::Cylinder)
                | (SegmentKind::Flat, SegmentKind::Cone)
                | (SegmentKind::Flat, SegmentKind::ConeInverted)
                | (SegmentKind::ConeInverted, SegmentKind::Cylinder)
                | (SegmentKind::SphereUpper, SegmentKind::Cylinder)
        );
        if sharp_allowed {
            continue;
        }
        let ta = a.tangent_at(radius, tol);
        let tb = b.tangent_at(radius, tol);
        let sine = (ta[0] * tb[1] - ta[1] * tb[0]).abs();
        if sine > tol.angle_eps {
            return Err(CutterError::JoinNotTangent { radius, sine });
        }
    }
    Ok(())
}

/// The contract every cutter implements.
/// `Sync`: a profile is shared read-only by parallel workers (feature
/// `parallel`); every implementor is plain data.
pub trait CutterProfile: Sync {
    fn kind(&self) -> CutterKind;
    /// Maximum cutting radius.
    fn radius(&self) -> f64;
    fn diameter(&self) -> f64 {
        2.0 * self.radius()
    }
    /// Usable cutting height above the reference point.
    fn cutting_length(&self) -> f64;
    /// Shank length above the cutting length, for collision checks and preview.
    fn shaft_length(&self) -> f64;
    /// Shank radius; equals the cutting radius unless the tool says otherwise.
    fn shaft_radius(&self) -> f64 {
        self.radius()
    }
    /// Height at which the curved/sloped profile ends and the cylindrical part begins.
    fn profile_height(&self) -> f64;
    /// Profile height above the reference at radial distance `r`; `None` outside `[0, radius]`.
    fn height_at_radius(&self, r: f64) -> Option<f64>;
    /// Largest cutting radius at local height `h`; `None` outside `[0, cutting_length]`.
    fn radius_at_height(&self, h: f64) -> Option<f64>;
    /// `radius_at_height` extended by the shank up to `cutting_length + shaft_length`.
    fn max_radius_at_height(&self, h: f64) -> Option<f64> {
        if h < 0.0 || !h.is_finite() {
            return None;
        }
        if h <= self.cutting_length() {
            return self.radius_at_height(h);
        }
        if h <= self.cutting_length() + self.shaft_length() {
            return Some(self.shaft_radius());
        }
        None
    }
    /// The piecewise description, bottom segment first.
    fn segments(&self) -> Vec<ProfileSegment>;
    /// User-facing dimension check.
    fn validate(&self) -> Result<(), CutterError>;
    /// Total modelled height: cutting length plus shank.
    fn total_height(&self) -> f64 {
        self.cutting_length() + self.shaft_length()
    }
}

pub(super) fn check_positive(field: &'static str, v: f64) -> Result<(), CutterError> {
    if !v.is_finite() {
        return Err(CutterError::NonFinite { field });
    }
    if v <= 0.0 {
        return Err(CutterError::NotPositive { field });
    }
    Ok(())
}

pub(super) fn check_non_negative(field: &'static str, v: f64) -> Result<(), CutterError> {
    if !v.is_finite() {
        return Err(CutterError::NonFinite { field });
    }
    if v < 0.0 {
        return Err(CutterError::NotPositive { field });
    }
    Ok(())
}

pub(super) fn in_radius(r: f64, radius: f64) -> bool {
    r.is_finite() && r >= 0.0 && r <= radius
}

pub(super) fn in_height(h: f64, cutting_length: f64) -> bool {
    h.is_finite() && h >= 0.0 && h <= cutting_length
}

/// Shank dimensions shared by every profile.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Shank {
    pub cutting_length: f64,
    pub shaft_length: f64,
    /// `None`: the cutting radius.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shaft_radius: Option<f64>,
}

impl Shank {
    pub fn new(cutting_length: f64, shaft_length: f64) -> Self {
        Self { cutting_length, shaft_length, shaft_radius: None }
    }

    pub(super) fn validate(&self) -> Result<(), CutterError> {
        check_positive("cutting length", self.cutting_length)?;
        check_non_negative("shaft length", self.shaft_length)?;
        if let Some(r) = self.shaft_radius {
            check_positive("shaft radius", r)?;
        }
        Ok(())
    }

    pub(super) fn cylinder_segment(&self, radius: f64, from: f64) -> ProfileSegment {
        ProfileSegment {
            kind: SegmentKind::Cylinder,
            radius_range: [radius, radius],
            height_range: [from, self.cutting_length],
            profile_radius: radius,
            ring_radius: 0.0,
            half_angle_rad: 0.0,
            z_offset: from,
        }
    }
}

/// Flat end mill: planar bottom.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FlatCutter {
    pub radius: f64,
    pub shank: Shank,
}

impl CutterProfile for FlatCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Flat
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.radius)
    }
    fn profile_height(&self) -> f64 {
        0.0
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        in_radius(r, self.radius).then_some(0.0)
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        in_height(h, self.shank.cutting_length).then_some(self.radius)
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        vec![
            ProfileSegment {
                kind: SegmentKind::Flat,
                radius_range: [0.0, self.radius],
                height_range: [0.0, 0.0],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: 0.0,
            },
            self.shank.cylinder_segment(self.radius, 0.0),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("diameter", self.radius)?;
        self.shank.validate()
    }
}

/// Ball nose end mill: hemispherical bottom, centre at local height `radius`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BallCutter {
    pub radius: f64,
    pub shank: Shank,
}

impl CutterProfile for BallCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Ball
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.radius)
    }
    fn profile_height(&self) -> f64 {
        self.radius
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.radius) {
            return None;
        }
        let rr = self.radius;
        Some(rr - (rr * rr - r * r).max(0.0).sqrt())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        let rr = self.radius;
        if h >= rr {
            return Some(rr);
        }
        Some((rr * rr - (rr - h) * (rr - h)).max(0.0).sqrt())
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        vec![
            ProfileSegment {
                kind: SegmentKind::Sphere,
                radius_range: [0.0, self.radius],
                height_range: [0.0, self.radius],
                profile_radius: self.radius,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: self.radius,
            },
            self.shank.cylinder_segment(self.radius, self.radius),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("diameter", self.radius)?;
        self.shank.validate()?;
        if self.shank.cutting_length < self.radius {
            return Err(CutterError::NotPositive { field: "cutting length above the ball" });
        }
        Ok(())
    }
}

/// Bull nose: flat centre of radius `radius - corner_radius`, toroidal corner.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BullCutter {
    pub radius: f64,
    pub corner_radius: f64,
    pub shank: Shank,
}

impl BullCutter {
    /// Radius of the flat centre and of the torus ring.
    pub fn flat_radius(&self) -> f64 {
        self.radius - self.corner_radius
    }
}

impl CutterProfile for BullCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Bull
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.radius)
    }
    fn profile_height(&self) -> f64 {
        self.corner_radius
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.radius) {
            return None;
        }
        let rf = self.flat_radius();
        if r <= rf {
            return Some(0.0);
        }
        let rc = self.corner_radius;
        let d = r - rf;
        Some(rc - (rc * rc - d * d).max(0.0).sqrt())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        let rc = self.corner_radius;
        if h >= rc {
            return Some(self.radius);
        }
        Some(self.flat_radius() + (rc * rc - (rc - h) * (rc - h)).max(0.0).sqrt())
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        let rf = self.flat_radius();
        vec![
            ProfileSegment {
                kind: SegmentKind::Flat,
                radius_range: [0.0, rf],
                height_range: [0.0, 0.0],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: 0.0,
            },
            ProfileSegment {
                kind: SegmentKind::Torus,
                radius_range: [rf, self.radius],
                height_range: [0.0, self.corner_radius],
                profile_radius: self.corner_radius,
                ring_radius: rf,
                half_angle_rad: 0.0,
                z_offset: self.corner_radius,
            },
            self.shank.cylinder_segment(self.radius, self.corner_radius),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("diameter", self.radius)?;
        if !self.corner_radius.is_finite() {
            return Err(CutterError::NonFinite { field: "corner radius" });
        }
        if self.corner_radius <= 0.0 || self.corner_radius >= self.radius {
            return Err(CutterError::CornerRadiusOutOfRange {
                corner_radius: self.corner_radius,
                radius: self.radius,
            });
        }
        self.shank.validate()?;
        if self.shank.cutting_length < self.corner_radius {
            return Err(CutterError::NotPositive { field: "cutting length above the corner" });
        }
        Ok(())
    }
}

/// Cone / V-bit: apex at the reference point, half-angle `half_angle_rad`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ConeCutter {
    /// Maximum cutting radius (where the cone meets the shank).
    pub radius: f64,
    pub included_angle_deg: f64,
    pub shank: Shank,
}

impl ConeCutter {
    pub fn half_angle_rad(&self) -> f64 {
        self.included_angle_deg.to_radians() * 0.5
    }

    /// Height of the cone from apex to the maximum radius.
    pub fn cone_height(&self) -> f64 {
        self.radius / self.half_angle_rad().tan()
    }
}

impl CutterProfile for ConeCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Cone
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.radius)
    }
    fn profile_height(&self) -> f64 {
        self.cone_height()
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.radius) {
            return None;
        }
        Some(r / self.half_angle_rad().tan())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        Some((h * self.half_angle_rad().tan()).min(self.radius))
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        // A cone taller than the cutting length is truncated there: the
        // profile ends at the flute end, below the maximum radius.
        let hc = self.cone_height().min(self.shank.cutting_length);
        let top_radius = (hc * self.half_angle_rad().tan()).min(self.radius);
        vec![
            ProfileSegment {
                kind: SegmentKind::Cone,
                radius_range: [0.0, top_radius],
                height_range: [0.0, hc],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: self.half_angle_rad(),
                z_offset: 0.0,
            },
            self.shank.cylinder_segment(top_radius, hc),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("maximum diameter", self.radius)?;
        if !self.included_angle_deg.is_finite() {
            return Err(CutterError::NonFinite { field: "included angle" });
        }
        if self.included_angle_deg <= 0.0 || self.included_angle_deg >= 180.0 {
            return Err(CutterError::IncludedAngleOutOfRange { included_angle_deg: self.included_angle_deg });
        }
        self.shank.validate()
    }
}

/// Ball-to-conical compound cutter: a ball nose of radius `ball_radius`
/// joined tangentially to a cone of half-angle `included_angle_deg / 2` that
/// widens to `radius`; shank above.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct BallConeCutter {
    pub ball_radius: f64,
    /// Maximum cutting radius at the top of the cone.
    pub radius: f64,
    pub included_angle_deg: f64,
    pub shank: Shank,
}

impl BallConeCutter {
    pub fn half_angle_rad(&self) -> f64 {
        self.included_angle_deg.to_radians() * 0.5
    }

    /// Radius where the cone leaves the ball tangentially: `Rb · cos(A)`.
    pub fn tangent_radius(&self) -> f64 {
        self.ball_radius * self.half_angle_rad().cos()
    }

    /// Ball height at the tangent radius.
    pub fn tangent_height(&self) -> f64 {
        let rb = self.ball_radius;
        let rt = self.tangent_radius();
        rb - (rb * rb - rt * rt).max(0.0).sqrt()
    }

    /// Apex height of the cone extended to the axis (negative: below the tip).
    pub fn cone_apex_z(&self) -> f64 {
        self.tangent_height() - self.tangent_radius() / self.half_angle_rad().tan()
    }

    /// Height where the cone reaches the maximum radius.
    pub fn cone_top_height(&self) -> f64 {
        self.cone_apex_z() + self.radius / self.half_angle_rad().tan()
    }

    fn is_plain_ball(&self) -> bool {
        self.radius <= self.ball_radius
    }
}

impl CutterProfile for BallConeCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::BallCone
    }
    fn radius(&self) -> f64 {
        self.radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.radius)
    }
    fn profile_height(&self) -> f64 {
        if self.is_plain_ball() {
            self.ball_radius
        } else {
            self.cone_top_height()
        }
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.radius) {
            return None;
        }
        let rb = self.ball_radius;
        if self.is_plain_ball() || r <= self.tangent_radius() {
            return Some(rb - (rb * rb - r * r).max(0.0).sqrt());
        }
        Some(self.cone_apex_z() + r / self.half_angle_rad().tan())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        let rb = self.ball_radius;
        if self.is_plain_ball() {
            return Some(if h >= rb { rb } else { (rb * rb - (rb - h) * (rb - h)).max(0.0).sqrt() });
        }
        if h <= self.tangent_height() {
            return Some((rb * rb - (rb - h) * (rb - h)).max(0.0).sqrt());
        }
        Some(((h - self.cone_apex_z()) * self.half_angle_rad().tan()).min(self.radius))
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        let rb = self.ball_radius;
        if self.is_plain_ball() {
            return vec![
                ProfileSegment {
                    kind: SegmentKind::Sphere,
                    radius_range: [0.0, rb],
                    height_range: [0.0, rb],
                    profile_radius: rb,
                    ring_radius: 0.0,
                    half_angle_rad: 0.0,
                    z_offset: rb,
                },
                self.shank.cylinder_segment(rb, rb),
            ];
        }
        let rt = self.tangent_radius();
        let ht = self.tangent_height();
        // The cone is truncated at the cutting length when it is taller.
        let top = self.cone_top_height().min(self.shank.cutting_length).max(ht);
        let top_radius = ((top - self.cone_apex_z()) * self.half_angle_rad().tan()).clamp(rt, self.radius);
        vec![
            ProfileSegment {
                kind: SegmentKind::Sphere,
                radius_range: [0.0, rt],
                height_range: [0.0, ht],
                profile_radius: rb,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: rb,
            },
            ProfileSegment {
                kind: SegmentKind::Cone,
                radius_range: [rt, top_radius],
                height_range: [ht, top],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: self.half_angle_rad(),
                z_offset: self.cone_apex_z(),
            },
            self.shank.cylinder_segment(top_radius, top),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("ball diameter", self.ball_radius)?;
        check_positive("maximum diameter", self.radius)?;
        if self.ball_radius > self.radius {
            return Err(CutterError::BallLargerThanMaximum {
                ball_diameter: 2.0 * self.ball_radius,
                maximum_diameter: 2.0 * self.radius,
            });
        }
        if !self.included_angle_deg.is_finite() {
            return Err(CutterError::NonFinite { field: "included angle" });
        }
        if self.included_angle_deg <= 0.0 || self.included_angle_deg >= 180.0 {
            return Err(CutterError::IncludedAngleOutOfRange { included_angle_deg: self.included_angle_deg });
        }
        self.shank.validate()
    }
}

/// Any of the five cutters, with static dispatch over the profile trait.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case")]
pub enum Cutter {
    Flat(FlatCutter),
    Ball(BallCutter),
    Bull(BullCutter),
    Cone(ConeCutter),
    BallCone(BallConeCutter),
    Frustum(FrustumCutter),
    Lollipop(LollipopCutter),
    Dovetail(DovetailCutter),
}

impl Cutter {
    pub fn flat(diameter: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Flat(FlatCutter { radius: 0.5 * diameter, shank: Shank::new(cutting_length, shaft_length) })
    }

    pub fn ball(diameter: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Ball(BallCutter { radius: 0.5 * diameter, shank: Shank::new(cutting_length, shaft_length) })
    }

    pub fn bull(diameter: f64, corner_radius: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Bull(BullCutter {
            radius: 0.5 * diameter,
            corner_radius,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }

    pub fn cone(maximum_diameter: f64, included_angle_deg: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Cone(ConeCutter {
            radius: 0.5 * maximum_diameter,
            included_angle_deg,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }

    pub fn ball_cone(
        ball_diameter: f64,
        maximum_diameter: f64,
        included_angle_deg: f64,
        cutting_length: f64,
        shaft_length: f64,
    ) -> Self {
        Cutter::BallCone(BallConeCutter {
            ball_radius: 0.5 * ball_diameter,
            radius: 0.5 * maximum_diameter,
            included_angle_deg,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }

    /// Default included angle of a hole-making tool kind when the geometry has none.
    pub fn default_point_angle_deg(kind: ToolKind) -> Option<f64> {
        match kind {
            ToolKind::Drill => Some(118.0),
            ToolKind::SpotDrill => Some(90.0),
            ToolKind::CenterDrill => Some(60.0),
            _ => None,
        }
    }

    /// The validated profile of a tool, or why it has none.
    ///
    /// Flat, tap, reamer, boring bar, thread mill, slot and T-slot cutters are
    /// cylinders (slot cutters on their arbor through `shaft_diameter`); a face
    /// mill is a bull when it has a corner radius; drills, spot and centre drills
    /// are cones with their point angle (118°, 90°, 60° unless given); V-bits and
    /// engravers are cones; a chamfer or tapered flat mill with a tip diameter is
    /// a frustum, without one a cone; a tapered ball is a ball-cone; a lollipop
    /// and a dovetail need `shaft_diameter` as their neck. Lathe inserts have no
    /// milling profile.
    pub fn from_tool(tool: &Tool) -> Result<Cutter, CutterError> {
        let g = &tool.geometry;
        let mut shank = Shank::new(g.cutting_length, g.shaft_length);
        if let Some(d) = g.shaft_diameter {
            shank.shaft_radius = Some(0.5 * d);
        }
        let angle = |field: &'static str| -> Result<f64, CutterError> {
            g.included_angle_deg
                .or_else(|| Self::default_point_angle_deg(tool.kind))
                .ok_or(CutterError::MissingField { kind: tool.kind, field })
        };
        let neck = || -> Result<f64, CutterError> {
            g.shaft_diameter.map(|d| 0.5 * d).ok_or(CutterError::MissingField { kind: tool.kind, field: "shaft diameter (neck)" })
        };
        let cutter = match tool.kind {
            ToolKind::Flat
            | ToolKind::Tap
            | ToolKind::Reamer
            | ToolKind::BoringBar
            | ToolKind::ThreadMill
            | ToolKind::Slot
            | ToolKind::TSlot => Cutter::Flat(FlatCutter { radius: 0.5 * g.diameter, shank }),
            ToolKind::FaceMill => match g.corner_radius {
                Some(corner_radius) if corner_radius > 0.0 => {
                    Cutter::Bull(BullCutter { radius: 0.5 * g.diameter, corner_radius, shank })
                }
                _ => Cutter::Flat(FlatCutter { radius: 0.5 * g.diameter, shank }),
            },
            ToolKind::Ball => Cutter::Ball(BallCutter { radius: 0.5 * g.diameter, shank }),
            ToolKind::Bull => {
                let corner_radius = g.corner_radius.ok_or(CutterError::MissingField {
                    kind: tool.kind,
                    field: "corner radius",
                })?;
                Cutter::Bull(BullCutter { radius: 0.5 * g.diameter, corner_radius, shank })
            }
            ToolKind::Drill | ToolKind::SpotDrill | ToolKind::CenterDrill | ToolKind::VBit | ToolKind::Engraver => {
                Cutter::Cone(ConeCutter { radius: 0.5 * g.diameter, included_angle_deg: angle("included angle")?, shank })
            }
            ToolKind::Chamfer | ToolKind::TaperedFlat => {
                let included_angle_deg = angle("included angle")?;
                match g.tip_diameter {
                    Some(tip) if tip > 0.0 => Cutter::Frustum(FrustumCutter {
                        tip_radius: 0.5 * tip,
                        radius: 0.5 * g.diameter,
                        included_angle_deg,
                        shank,
                    }),
                    _ => Cutter::Cone(ConeCutter { radius: 0.5 * g.diameter, included_angle_deg, shank }),
                }
            }
            ToolKind::Lollipop => {
                Cutter::Lollipop(LollipopCutter { ball_radius: 0.5 * g.diameter, neck_radius: neck()?, shank })
            }
            ToolKind::Dovetail => Cutter::Dovetail(DovetailCutter {
                radius: 0.5 * g.diameter,
                neck_radius: neck()?,
                included_angle_deg: angle("included angle")?,
                shank,
            }),
            ToolKind::TaperedBall => {
                let included_angle_deg = g.included_angle_deg.ok_or(CutterError::MissingField {
                    kind: tool.kind,
                    field: "included angle",
                })?;
                let ball_diameter = g.ball_diameter.unwrap_or(g.diameter);
                Cutter::BallCone(BallConeCutter {
                    ball_radius: 0.5 * ball_diameter,
                    radius: 0.5 * g.diameter,
                    included_angle_deg,
                    shank,
                })
            }
            other => return Err(CutterError::UnsupportedToolKind { kind: other }),
        };
        cutter.validate()?;
        Ok(cutter)
    }

    fn profile(&self) -> &dyn CutterProfile {
        match self {
            Cutter::Flat(c) => c,
            Cutter::Ball(c) => c,
            Cutter::Bull(c) => c,
            Cutter::Cone(c) => c,
            Cutter::BallCone(c) => c,
            Cutter::Frustum(c) => c,
            Cutter::Lollipop(c) => c,
            Cutter::Dovetail(c) => c,
        }
    }

    pub fn frustum(tip_diameter: f64, diameter: f64, included_angle_deg: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Frustum(FrustumCutter {
            tip_radius: 0.5 * tip_diameter,
            radius: 0.5 * diameter,
            included_angle_deg,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }

    pub fn lollipop(ball_diameter: f64, neck_diameter: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Lollipop(LollipopCutter {
            ball_radius: 0.5 * ball_diameter,
            neck_radius: 0.5 * neck_diameter,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }

    pub fn dovetail(diameter: f64, neck_diameter: f64, included_angle_deg: f64, cutting_length: f64, shaft_length: f64) -> Self {
        Cutter::Dovetail(DovetailCutter {
            radius: 0.5 * diameter,
            neck_radius: 0.5 * neck_diameter,
            included_angle_deg,
            shank: Shank::new(cutting_length, shaft_length),
        })
    }
}

impl CutterProfile for Cutter {
    fn kind(&self) -> CutterKind {
        self.profile().kind()
    }
    fn radius(&self) -> f64 {
        self.profile().radius()
    }
    fn cutting_length(&self) -> f64 {
        self.profile().cutting_length()
    }
    fn shaft_length(&self) -> f64 {
        self.profile().shaft_length()
    }
    fn shaft_radius(&self) -> f64 {
        self.profile().shaft_radius()
    }
    fn profile_height(&self) -> f64 {
        self.profile().profile_height()
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        self.profile().height_at_radius(r)
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        self.profile().radius_at_height(h)
    }
    fn max_radius_at_height(&self, h: f64) -> Option<f64> {
        self.profile().max_radius_at_height(h)
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        self.profile().segments()
    }
    fn validate(&self) -> Result<(), CutterError> {
        self.profile().validate()?;
        check_segment_joins(&self.profile().segments(), &crate::numeric::Tolerance::default())
    }
}

