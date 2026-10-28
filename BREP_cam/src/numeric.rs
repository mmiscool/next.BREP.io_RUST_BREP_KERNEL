//! The single numeric policy every CAM algorithm reads.
//!
//! No algorithm hard-codes a tolerance: it takes a [`Tolerance`] and uses the
//! named epsilon for the comparison it is making. Tolerances scale with the
//! model's bounding-box diagonal (see [`Tolerance::scaled_for_diagonal`]) so
//! very large or very small models keep sensible relative precision.

use crate::model::Point3;
use serde::{Deserialize, Serialize};

/// Named epsilons. Lengths are in model units.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Tolerance {
    /// Point equality.
    pub point_eps: f64,
    /// Geometric distance comparisons (contact heights, offsets, clipping).
    pub distance_eps: f64,
    /// Zero-area loop/triangle checks.
    pub area_eps: f64,
    /// Dot/cross comparisons (unitless).
    pub angle_eps: f64,
    /// Minimum accepted span/sample distance.
    pub sample_eps: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self {
            point_eps: 1e-6,
            distance_eps: 1e-6,
            area_eps: 1e-10,
            angle_eps: 1e-9,
            sample_eps: 1e-4,
        }
    }
}

impl Tolerance {
    /// Bounding-box diagonal the defaults are tuned for (a part about a metre across).
    pub const REFERENCE_DIAGONAL: f64 = 1000.0;
    /// Scale factor clamp: tolerances never move more than this far from the defaults.
    pub const MAX_SCALE: f64 = 1e3;

    /// Defaults scaled for a model whose bounding-box diagonal is `diagonal`.
    /// Length epsilons scale linearly, the area epsilon quadratically, the
    /// angle epsilon not at all. A non-positive or non-finite diagonal yields
    /// the defaults.
    pub fn scaled_for_diagonal(diagonal: f64) -> Self {
        let base = Self::default();
        if !(diagonal > 0.0) || !diagonal.is_finite() {
            return base;
        }
        let s = (diagonal / Self::REFERENCE_DIAGONAL).clamp(1.0 / Self::MAX_SCALE, Self::MAX_SCALE);
        Self {
            point_eps: base.point_eps * s,
            distance_eps: base.distance_eps * s,
            area_eps: base.area_eps * s * s,
            angle_eps: base.angle_eps,
            sample_eps: base.sample_eps * s,
        }
    }

    /// Every epsilon finite and positive.
    pub fn validate(&self) -> Result<(), NumericError> {
        let fields = [
            ("pointEps", self.point_eps),
            ("distanceEps", self.distance_eps),
            ("areaEps", self.area_eps),
            ("angleEps", self.angle_eps),
            ("sampleEps", self.sample_eps),
        ];
        for (name, value) in fields {
            if !value.is_finite() {
                return Err(NumericError::NonFinite { field: name });
            }
            if value <= 0.0 {
                return Err(NumericError::NotPositive { field: name });
            }
        }
        Ok(())
    }

    pub fn points_equal(&self, a: Point3, b: Point3) -> bool {
        a.distance(b) <= self.point_eps
    }

    pub fn distances_equal(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.distance_eps
    }

    /// `a` is below `b` by more than the distance epsilon.
    pub fn distance_less(&self, a: f64, b: f64) -> bool {
        a < b - self.distance_eps
    }

    pub fn is_zero_area(&self, area: f64) -> bool {
        area.abs() <= self.area_eps
    }

    /// Two unit vectors are parallel (same or opposite sense).
    pub fn parallel(&self, a: Point3, b: Point3) -> bool {
        a.cross(b).length() <= self.angle_eps
    }

    /// Two unit vectors are perpendicular.
    pub fn perpendicular(&self, a: Point3, b: Point3) -> bool {
        a.dot(b).abs() <= self.angle_eps
    }

    /// A unit direction is vertical (along ±Z) within the angle epsilon.
    pub fn is_vertical(&self, dir: Point3) -> bool {
        self.parallel(dir, Point3::Z)
    }

    /// A unit direction is horizontal (perpendicular to Z) within the angle epsilon.
    pub fn is_horizontal(&self, dir: Point3) -> bool {
        self.perpendicular(dir, Point3::Z)
    }

    /// A span shorter than the sample epsilon is not worth a sample.
    pub fn span_too_short(&self, length: f64) -> bool {
        length < self.sample_eps
    }
}

/// A refused tolerance field.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NumericError {
    NonFinite { field: &'static str },
    NotPositive { field: &'static str },
}

impl std::fmt::Display for NumericError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            NumericError::NonFinite { field } => write!(f, "tolerance {field} is not finite"),
            NumericError::NotPositive { field } => write!(f, "tolerance {field} must be positive"),
        }
    }
}

impl std::error::Error for NumericError {}

/// Finite check for a coordinate triple; the caller turns `false` into a warning.
pub fn finite_point(p: Point3) -> bool {
    p.is_finite()
}

