//! Contact diagnostics attached to cutter-location points.

use super::Point3;
use serde::{Deserialize, Serialize};

/// Which part of the mesh and the cutter met.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ContactType {
    #[default]
    None,
    Vertex,
    Facet,
    Edge,
    Shaft,
    ToolProfile,
    NumericFallback,
}

/// The winning contact of a cutter-location solve.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Contact {
    #[serde(rename = "type")]
    pub contact_type: ContactType,
    /// Contact point on the mesh, in machine coordinates.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub point: Option<Point3>,
    /// Triangle id in the mesh snapshot.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub triangle_id: Option<u32>,
    /// Distance from the cutter reference to the contact, when the solver reports one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub distance: Option<f64>,
    /// Free-form solver detail (e.g. the fallback used).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
}

/// A cutter-location point: the tool reference point in machine coordinates
/// plus the contact that fixed its height.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct ClPoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact: Option<Contact>,
}

impl ClPoint {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z, contact: None }
    }

    pub fn position(&self) -> Point3 {
        Point3::new(self.x, self.y, self.z)
    }

    pub fn is_finite(&self) -> bool {
        self.position().is_finite()
    }
}
