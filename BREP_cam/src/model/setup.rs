//! Setups: the rigid model-to-machine transform, work offset, stock and heights.

use super::{Bounds3, MachineId, Point3, RigidTransform, SetupId, TargetRef};
use serde::{Deserialize, Serialize};

/// One machining setup: a fixturing of the part on the machine.
///
/// Every algorithm runs in MACHINE coordinates (cutter axis +Z, depths descend
/// in Z). `model_to_machine` is the only transform CAM applies to geometry;
/// the default aligns model +Z with machine +Z.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Setup {
    pub id: SetupId,
    #[serde(default)]
    pub name: String,
    /// Rigid transform from model coordinates to machine coordinates.
    #[serde(default)]
    pub model_to_machine: RigidTransform,
    /// Work coordinate system the post emits (G54..., or a fixture offset).
    #[serde(default)]
    pub wcs: WorkOffset,
    /// Raw material before this setup's operations.
    #[serde(default)]
    pub stock: Stock,
    /// Unclamped length measured back from the cylindrical stock's front.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub turning_stick_out: Option<f64>,
    /// Clearance/retract/feed heights in MACHINE Z.
    #[serde(default)]
    pub heights: Heights,
    /// Machine the setup is posted for; `None` falls back to a generic 3-axis mill.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub machine: Option<MachineId>,
    /// Bodies being machined in this setup (whole bodies).
    #[serde(default)]
    pub targets: Vec<TargetRef>,
    /// Fixtures, vise jaws and other protected bodies (never cut, always checked).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub protected: Vec<TargetRef>,
    /// Program identity for the post (number and/or comment).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub program: Option<ProgramInfo>,
}

impl Setup {
    pub fn new(id: impl Into<SetupId>, name: impl Into<String>) -> Self {
        Self { id: id.into(), name: name.into(), ..Default::default() }
    }

    /// Machine-to-model transform for display conversion.
    pub fn machine_to_model(&self) -> RigidTransform {
        self.model_to_machine.inverse()
    }
}

/// The work offset the post program uses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct WorkOffset {
    /// Work offset number: 1 = G54, 2 = G55, ... (controller-specific beyond 6).
    pub number: u32,
    /// Position of the WCS origin in MACHINE coordinates (usually the origin).
    pub origin: Point3,
}

impl Default for WorkOffset {
    fn default() -> Self {
        Self { number: 1, origin: Point3::ZERO }
    }
}

/// Program number and comment for the NC output.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ProgramInfo {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub number: Option<u32>,
    #[serde(default)]
    pub comment: String,
}

/// Raw material definition. Geometric stock is given in MODEL coordinates and
/// transformed with the setup; `FromRest` chains setups.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase")]
#[non_exhaustive]
pub enum Stock {
    /// The targets' bounding box grown by per-side offsets (Fusion's "relative size box").
    BoundsOffset {
        #[serde(default)]
        x: f64,
        #[serde(default)]
        y: f64,
        #[serde(default)]
        top: f64,
        #[serde(default)]
        bottom: f64,
    },
    /// An explicit box.
    Box { min: Point3, max: Point3 },
    /// A cylinder along `axis` starting at `base`.
    Cylinder { base: Point3, axis: Point3, radius: f64, height: f64 },
    /// Another body in the document is the stock.
    FromSolid { solid: String },
    /// The rest material left by a previous setup.
    FromRest { setup: SetupId },
}

impl Default for Stock {
    fn default() -> Self {
        Stock::BoundsOffset { x: 0.0, y: 0.0, top: 0.0, bottom: 0.0 }
    }
}

impl Stock {
    /// Model-space bounding box of the stock when it is geometric and the
    /// targets' model bounds are `target_bounds`. `FromSolid`/`FromRest` need
    /// their referenced geometry and return `None`.
    pub fn model_bounds(&self, target_bounds: &Bounds3) -> Option<Bounds3> {
        match self {
            Stock::BoundsOffset { x, y, top, bottom } => {
                if target_bounds.is_empty() {
                    return None;
                }
                Some(Bounds3::new(
                    Point3::new(target_bounds.min.x - x, target_bounds.min.y - y, target_bounds.min.z - bottom),
                    Point3::new(target_bounds.max.x + x, target_bounds.max.y + y, target_bounds.max.z + top),
                ))
            }
            Stock::Box { min, max } => Some(Bounds3::new(*min, *max)),
            Stock::Cylinder { base, axis, radius, height } => {
                let a = axis.normalized()?;
                let top = base.add(a.scale(*height));
                // Extent of a circle of radius r perpendicular to a along each axis.
                let ext = Point3::new(
                    radius * (1.0 - a.x * a.x).max(0.0).sqrt(),
                    radius * (1.0 - a.y * a.y).max(0.0).sqrt(),
                    radius * (1.0 - a.z * a.z).max(0.0).sqrt(),
                );
                Some(Bounds3::new(base.min(top).sub(ext), base.max(top).add(ext)))
            }
            Stock::FromSolid { .. } | Stock::FromRest { .. } => None,
        }
    }

    /// Every finite dimension finite and positive where it must be.
    pub fn validate(&self) -> Result<(), String> {
        match self {
            Stock::BoundsOffset { x, y, top, bottom } => {
                for (name, v) in [("x", x), ("y", y), ("top", top), ("bottom", bottom)] {
                    if !v.is_finite() {
                        return Err(format!("stock offset {name} is not finite"));
                    }
                }
                Ok(())
            }
            Stock::Box { min, max } => {
                if !min.is_finite() || !max.is_finite() {
                    return Err("stock box has a non-finite corner".into());
                }
                if min.x >= max.x || min.y >= max.y || min.z >= max.z {
                    return Err("stock box must have positive size on every axis".into());
                }
                Ok(())
            }
            Stock::Cylinder { base, axis, radius, height } => {
                if !base.is_finite() || !axis.is_finite() || !radius.is_finite() || !height.is_finite() {
                    return Err("stock cylinder has a non-finite parameter".into());
                }
                if axis.normalized().is_none() {
                    return Err("stock cylinder axis must be non-zero".into());
                }
                if *radius <= 0.0 || *height <= 0.0 {
                    return Err("stock cylinder radius and height must be positive".into());
                }
                Ok(())
            }
            Stock::FromSolid { solid } => {
                if solid.is_empty() {
                    Err("stock body name is empty".into())
                } else {
                    Ok(())
                }
            }
            Stock::FromRest { setup } => {
                if setup.as_str().is_empty() {
                    Err("rest stock setup id is empty".into())
                } else {
                    Ok(())
                }
            }
        }
    }
}

/// Safe heights in MACHINE Z. Absolute values; the operation may override.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct Heights {
    /// Rapid traverse height between operations and over fixtures.
    pub clearance: f64,
    /// Retract height between passes within an operation.
    pub retract: f64,
    /// Height at which rapids turn into feed moves on the way down.
    pub feed: f64,
    /// Top of the material to cut (`None`: from stock).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub top: Option<f64>,
    /// Lowest Z the operation may reach (`None`: from the model).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bottom: Option<f64>,
}

impl Default for Heights {
    fn default() -> Self {
        Self { clearance: 15.0, retract: 5.0, feed: 2.0, top: None, bottom: None }
    }
}

impl Heights {
    /// `clearance >= retract >= feed`, all finite; top above bottom when both given.
    pub fn validate(&self) -> Result<(), String> {
        for (name, v) in [("clearance", self.clearance), ("retract", self.retract), ("feed", self.feed)] {
            if !v.is_finite() {
                return Err(format!("{name} height is not finite"));
            }
        }
        if self.clearance < self.retract {
            return Err("clearance height must not be below retract height".into());
        }
        if self.retract < self.feed {
            return Err("retract height must not be below feed height".into());
        }
        if let (Some(top), Some(bottom)) = (self.top, self.bottom) {
            if !top.is_finite() || !bottom.is_finite() {
                return Err("top/bottom height is not finite".into());
            }
            if top < bottom {
                return Err("top height must not be below bottom height".into());
            }
        }
        Ok(())
    }
}
