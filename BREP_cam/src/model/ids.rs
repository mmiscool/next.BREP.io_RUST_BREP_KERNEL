//! Document-level identities and target references.
//!
//! Setups, tools, operations and machines are addressed by string ids the
//! document assigns and keeps stable across saves. Bodies and faces are
//! addressed by the BREP's persistent NAMES (the kernel propagates face names
//! through booleans), which is the identity the application's selection and
//! feature references already use.

use super::Point3;
use serde::{Deserialize, Serialize};

macro_rules! string_id {
    ($(#[$doc:meta])* $name:ident) => {
        $(#[$doc])*
        #[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
        #[serde(transparent)]
        pub struct $name(pub String);

        impl $name {
            pub fn new(id: impl Into<String>) -> Self {
                Self(id.into())
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }

        impl From<&str> for $name {
            fn from(s: &str) -> Self {
                Self(s.to_string())
            }
        }

        impl From<String> for $name {
            fn from(s: String) -> Self {
                Self(s)
            }
        }

        impl std::fmt::Display for $name {
            fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                f.write_str(&self.0)
            }
        }
    };
}

string_id!(
    /// Identity of a [`crate::model::Setup`] within a document.
    SetupId
);
string_id!(
    /// Identity of a [`crate::model::Tool`] within a document or library.
    ToolId
);
string_id!(
    /// Identity of an [`crate::model::Operation`] within a document.
    OperationId
);
string_id!(
    /// Identity of a [`crate::model::Machine`] definition.
    MachineId
);
string_id!(
    /// Identity of a [`crate::model::Holder`] definition.
    HolderId
);

/// A stable reference to a BREP body or to one face of it.
///
/// `face == None` names the whole body. Resolution against a mesh snapshot
/// happens by name; an unresolvable reference is a reported error, never a
/// silently empty target.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TargetRef {
    /// Persistent body (solid) name.
    pub solid: String,
    /// Persistent face name within `solid`, or `None` for the whole body.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub face: Option<String>,
}

impl TargetRef {
    pub fn solid(solid: impl Into<String>) -> Self {
        Self { solid: solid.into(), face: None }
    }

    pub fn face(solid: impl Into<String>, face: impl Into<String>) -> Self {
        Self { solid: solid.into(), face: Some(face.into()) }
    }

    pub fn is_whole_body(&self) -> bool {
        self.face.is_none()
    }
}

/// What an operation machines and what it must not touch.
///
/// `drive_faces` restrict where cutting is REQUESTED; collision and gouge
/// checks always protect every triangle of `target_solids` and
/// `protected_solids`. An empty `target_solids` list is a reported error for
/// an enabled operation.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct TargetSelection {
    /// Bodies to machine.
    #[serde(default)]
    pub target_solids: Vec<String>,
    /// Faces (on the target bodies) that bound the requested cutting region.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub drive_faces: Vec<TargetRef>,
    /// Faces to avoid (checked, never cut) in addition to the protected bodies.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub avoid_faces: Vec<TargetRef>,
    /// Fixtures, clamps and other bodies that are protected but not machined.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub protected_solids: Vec<String>,
    /// Ordered edge chains and sketch curves/profiles to machine along (2D
    /// contour, slot, chamfer, trace, engrave, ...).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub chains: Vec<ChainRef>,
}

impl TargetSelection {
    /// Names and chain shapes are well formed (resolution against the model is
    /// the dispatcher's).
    pub fn validate(&self) -> Result<(), String> {
        for (i, chain) in self.chains.iter().enumerate() {
            chain.validate().map_err(|m| format!("chain {i}: {m}"))?;
        }
        Ok(())
    }

    pub fn solids<I, S>(solids: I) -> Self
    where
        I: IntoIterator<Item = S>,
        S: Into<String>,
    {
        Self { target_solids: solids.into_iter().map(Into::into).collect(), ..Default::default() }
    }

    /// Every body whose triangles take part in protection checks, de-duplicated and sorted.
    pub fn protected_bodies(&self) -> Vec<String> {
        let mut all: Vec<String> =
            self.target_solids.iter().chain(self.protected_solids.iter()).cloned().collect();
        all.sort();
        all.dedup();
        all
    }
}

/// One BREP edge by persistent names, like [`TargetRef`] for faces.
#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct EdgeRef {
    /// Persistent body (solid) name.
    pub solid: String,
    /// Persistent edge name within `solid`.
    pub edge: String,
}

impl EdgeRef {
    pub fn new(solid: impl Into<String>, edge: impl Into<String>) -> Self {
        Self { solid: solid.into(), edge: edge.into() }
    }
}

/// An ordered chain of curves to machine along.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
#[serde(default)]
pub struct ChainRef {
    pub source: ChainSource,
    /// The chain closes on itself; resolution checks it and reports a mismatch.
    pub closed: bool,
    /// Traverse opposite to the stored order.
    pub reversed: bool,
    /// Which side of the chain the tool stays on.
    pub side: ChainSide,
    /// Extra offset from the chain towards `side`, in model units (may be negative).
    pub offset: f64,
    /// Where the tool starts along the chain (nearest chain point to this, in
    /// model coordinates); `None`: the chain's first point.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub start: Option<Point3>,
}

impl Default for ChainRef {
    fn default() -> Self {
        Self { source: ChainSource::default(), closed: false, reversed: false, side: ChainSide::Auto, offset: 0.0, start: None }
    }
}

impl ChainRef {
    pub fn validate(&self) -> Result<(), String> {
        self.source.validate()?;
        if !self.offset.is_finite() {
            return Err("chain offset is not finite".into());
        }
        if self.start.map_or(false, |p| !p.is_finite()) {
            return Err("chain start point is not finite".into());
        }
        if !self.closed && matches!(self.side, ChainSide::Inside | ChainSide::Outside) {
            return Err("inside/outside needs a closed chain; use left or right on an open chain".into());
        }
        Ok(())
    }
}

/// Where a chain's curves come from.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "camelCase", rename_all_fields = "camelCase")]
pub enum ChainSource {
    /// BREP edges in chain order (consecutive edges share a vertex).
    Edges { edges: Vec<EdgeRef> },
    /// Sketch curves by geometry id (the `{id}` of the edge name `{sketch}:G{id}`), in chain order.
    SketchCurves { sketch: String, curves: Vec<String> },
    /// A sketch's closed profile (the face `{sketch}:PROFILE`); `regions` picks
    /// regions by index in the kernel's SketchProfile region order, each region
    /// an outer loop with its islands (empty: every region).
    SketchProfile {
        sketch: String,
        #[serde(default)]
        regions: Vec<u32>,
    },
}

impl Default for ChainSource {
    fn default() -> Self {
        ChainSource::Edges { edges: Vec::new() }
    }
}

impl ChainSource {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["edges", "sketchCurves", "sketchProfile"];

    pub fn validate(&self) -> Result<(), String> {
        match self {
            ChainSource::Edges { edges } => {
                if edges.is_empty() {
                    return Err("an edge chain needs at least one edge".into());
                }
                let mut seen = std::collections::BTreeSet::new();
                for e in edges {
                    if e.solid.is_empty() || e.edge.is_empty() {
                        return Err("an edge reference needs a body and an edge name".into());
                    }
                    if !seen.insert(e) {
                        return Err(format!("edge {:?} of {:?} appears twice in the chain", e.edge, e.solid));
                    }
                }
                Ok(())
            }
            ChainSource::SketchCurves { sketch, curves } => {
                if sketch.is_empty() || curves.is_empty() || curves.iter().any(|c| c.is_empty()) {
                    return Err("a sketch chain needs a sketch id and at least one curve id".into());
                }
                let unique: std::collections::BTreeSet<&String> = curves.iter().collect();
                if unique.len() != curves.len() {
                    return Err("a sketch chain lists a curve twice".into());
                }
                Ok(())
            }
            ChainSource::SketchProfile { sketch, regions } => {
                if sketch.is_empty() {
                    return Err("a sketch profile needs a sketch id".into());
                }
                let unique: std::collections::BTreeSet<&u32> = regions.iter().collect();
                if unique.len() != regions.len() {
                    return Err("a sketch profile lists a region twice".into());
                }
                Ok(())
            }
        }
    }
}

/// Which side of a chain the tool stays on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ChainSide {
    /// The strategy decides (contour: outside of a closed chain).
    #[default]
    Auto,
    Left,
    Right,
    Inside,
    Outside,
    /// The tool centre runs on the chain.
    On,
}

impl ChainSide {
    /// Serde names of every variant, in declaration order.
    pub const VARIANTS: &'static [&'static str] = &["auto", "left", "right", "inside", "outside", "on"];
}
