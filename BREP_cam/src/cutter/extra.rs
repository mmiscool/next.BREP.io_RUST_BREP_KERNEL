//! Cutters whose profile is not monotone in radius: a flat tip widening into a
//! cone (chamfer mills, tapered flat mills, drill-point shoulders), a ball on
//! a neck (lollipop) and an inverted cone on a neck (dovetail).
//!
//! `height_at_radius` stays the LOWER envelope (what a drop cutter touches
//! from above); `radius_at_height` is the largest radius at a height, which
//! decreases upward on the undercut parts. Upper surfaces are described by
//! the `SphereUpper` and `ConeInverted` segment kinds so collision, preview
//! and simulation see them while the drop solver skips them.

use super::{check_non_negative, check_positive, in_height, in_radius, CutterError, CutterKind, CutterProfile, ProfileSegment, SegmentKind, Shank};
use serde::{Deserialize, Serialize};

/// Flat tip of radius `tip_radius`, then a cone of half-angle
/// `included_angle_deg / 2` widening to `radius`, then the shank.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct FrustumCutter {
    pub tip_radius: f64,
    pub radius: f64,
    pub included_angle_deg: f64,
    pub shank: Shank,
}

impl FrustumCutter {
    pub fn half_angle_rad(&self) -> f64 {
        self.included_angle_deg.to_radians() * 0.5
    }

    /// Height where the cone reaches the maximum radius (`r = tip + h·tan A`).
    pub fn cone_top_height(&self) -> f64 {
        (self.radius - self.tip_radius) / self.half_angle_rad().tan()
    }

    /// Apex height of the cone extended to the axis (negative: below the tip).
    pub fn cone_apex_z(&self) -> f64 {
        -self.tip_radius / self.half_angle_rad().tan()
    }
}

impl CutterProfile for FrustumCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Frustum
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
        self.cone_top_height().min(self.shank.cutting_length)
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.radius) {
            return None;
        }
        if r <= self.tip_radius {
            return Some(0.0);
        }
        Some((r - self.tip_radius) / self.half_angle_rad().tan())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        Some((self.tip_radius + h * self.half_angle_rad().tan()).min(self.radius))
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        let top = self.cone_top_height().min(self.shank.cutting_length);
        let top_radius = (self.tip_radius + top * self.half_angle_rad().tan()).min(self.radius);
        vec![
            ProfileSegment {
                kind: SegmentKind::Flat,
                radius_range: [0.0, self.tip_radius],
                height_range: [0.0, 0.0],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: 0.0,
            },
            ProfileSegment {
                kind: SegmentKind::Cone,
                radius_range: [self.tip_radius, top_radius],
                height_range: [0.0, top],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: self.half_angle_rad(),
                z_offset: self.cone_apex_z(),
            },
            self.shank.cylinder_segment(top_radius, top),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("diameter", self.radius)?;
        check_non_negative("tip diameter", self.tip_radius)?;
        if self.tip_radius >= self.radius {
            return Err(CutterError::TipNotSmallerThanDiameter {
                tip_diameter: 2.0 * self.tip_radius,
                diameter: 2.0 * self.radius,
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

/// A full ball of radius `ball_radius` on a neck of radius `neck_radius`
/// (undercutting cutter). The ball centre sits at local height `ball_radius`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct LollipopCutter {
    pub ball_radius: f64,
    pub neck_radius: f64,
    pub shank: Shank,
}

impl LollipopCutter {
    /// Height where the upper ball surface meets the neck.
    pub fn neck_height(&self) -> f64 {
        let rb = self.ball_radius;
        rb + (rb * rb - self.neck_radius * self.neck_radius).max(0.0).sqrt()
    }
}

impl CutterProfile for LollipopCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Lollipop
    }
    fn radius(&self) -> f64 {
        self.ball_radius
    }
    fn cutting_length(&self) -> f64 {
        self.shank.cutting_length
    }
    fn shaft_length(&self) -> f64 {
        self.shank.shaft_length
    }
    fn shaft_radius(&self) -> f64 {
        self.shank.shaft_radius.unwrap_or(self.neck_radius)
    }
    fn profile_height(&self) -> f64 {
        self.neck_height().min(self.shank.cutting_length)
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        if !in_radius(r, self.ball_radius) {
            return None;
        }
        let rb = self.ball_radius;
        Some(rb - (rb * rb - r * r).max(0.0).sqrt())
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        let rb = self.ball_radius;
        if h <= self.neck_height() {
            return Some((rb * rb - (h - rb) * (h - rb)).max(0.0).sqrt());
        }
        Some(self.neck_radius)
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        let rb = self.ball_radius;
        let neck_h = self.neck_height().min(self.shank.cutting_length);
        let neck_r = if neck_h < self.neck_height() {
            (rb * rb - (neck_h - rb) * (neck_h - rb)).max(0.0).sqrt()
        } else {
            self.neck_radius
        };
        vec![
            ProfileSegment {
                kind: SegmentKind::Sphere,
                radius_range: [0.0, rb],
                height_range: [0.0, rb],
                profile_radius: rb,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: rb,
            },
            ProfileSegment {
                kind: SegmentKind::SphereUpper,
                radius_range: [rb, neck_r],
                height_range: [rb, neck_h],
                profile_radius: rb,
                ring_radius: 0.0,
                half_angle_rad: 0.0,
                z_offset: rb,
            },
            self.shank.cylinder_segment(neck_r, neck_h),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("ball diameter", self.ball_radius)?;
        check_positive("neck diameter", self.neck_radius)?;
        if self.neck_radius >= self.ball_radius {
            return Err(CutterError::NeckNotSmallerThanDiameter {
                neck_diameter: 2.0 * self.neck_radius,
                diameter: 2.0 * self.ball_radius,
            });
        }
        self.shank.validate()?;
        if self.shank.cutting_length < self.ball_radius {
            return Err(CutterError::NotPositive { field: "cutting length above the ball centre" });
        }
        Ok(())
    }
}

/// Flat bottom of radius `radius`, an inverted cone of half-angle
/// `included_angle_deg / 2` narrowing to `neck_radius`, then the neck.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct DovetailCutter {
    pub radius: f64,
    pub neck_radius: f64,
    pub included_angle_deg: f64,
    pub shank: Shank,
}

impl DovetailCutter {
    pub fn half_angle_rad(&self) -> f64 {
        self.included_angle_deg.to_radians() * 0.5
    }

    /// Height where the inverted cone reaches the neck radius.
    pub fn neck_height(&self) -> f64 {
        (self.radius - self.neck_radius) / self.half_angle_rad().tan()
    }
}

impl CutterProfile for DovetailCutter {
    fn kind(&self) -> CutterKind {
        CutterKind::Dovetail
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
        self.shank.shaft_radius.unwrap_or(self.neck_radius)
    }
    fn profile_height(&self) -> f64 {
        self.neck_height().min(self.shank.cutting_length)
    }
    fn height_at_radius(&self, r: f64) -> Option<f64> {
        in_radius(r, self.radius).then_some(0.0)
    }
    fn radius_at_height(&self, h: f64) -> Option<f64> {
        if !in_height(h, self.shank.cutting_length) {
            return None;
        }
        Some((self.radius - h * self.half_angle_rad().tan()).max(self.neck_radius))
    }
    fn segments(&self) -> Vec<ProfileSegment> {
        let neck_h = self.neck_height().min(self.shank.cutting_length);
        let neck_r = (self.radius - neck_h * self.half_angle_rad().tan()).max(self.neck_radius);
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
            ProfileSegment {
                kind: SegmentKind::ConeInverted,
                radius_range: [self.radius, neck_r],
                height_range: [0.0, neck_h],
                profile_radius: 0.0,
                ring_radius: 0.0,
                half_angle_rad: self.half_angle_rad(),
                z_offset: self.radius / self.half_angle_rad().tan(),
            },
            self.shank.cylinder_segment(neck_r, neck_h),
        ]
    }
    fn validate(&self) -> Result<(), CutterError> {
        check_positive("diameter", self.radius)?;
        check_positive("neck diameter", self.neck_radius)?;
        if self.neck_radius >= self.radius {
            return Err(CutterError::NeckNotSmallerThanDiameter {
                neck_diameter: 2.0 * self.neck_radius,
                diameter: 2.0 * self.radius,
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
