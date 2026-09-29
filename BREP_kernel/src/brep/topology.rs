use crate::{
    make_cylinder_surface, make_line, make_plane, KernelTolerances, NurbsCurve, NurbsSurface, Vec2,
    Vec3, Vec4,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct VertexRecord {
    pub id: u64,
    pub point: Vec3,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EdgeRecord {
    pub id: u64,
    pub curve: NurbsCurve,
    pub t0: f64,
    pub t1: f64,
    pub start_vertex_id: u64,
    pub end_vertex_id: u64,
    #[serde(default)]
    pub degenerate: bool,
    /// Persistent name carried through splits, welds, and booleans so the
    /// application never has to re-derive edge identity geometrically.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CoedgeRecord {
    pub id: u64,
    pub edge_id: u64,
    pub forward: bool,
    pub pcurve: NurbsCurve,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct LoopRecord {
    pub id: u64,
    pub coedges: Vec<CoedgeRecord>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct FaceRecord {
    pub id: u64,
    pub surface: NurbsSurface,
    pub same_sense: bool,
    pub loops: Vec<LoopRecord>,
    /// Persistent name propagated through booleans (split fragments get
    /// deterministic `_1`, `_2` suffixes; merges keep the first name).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ShellRecord {
    pub id: u64,
    pub faces: Vec<FaceRecord>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct BrepSolid {
    /// Content-checked measurements; cloned with this solid, never serialized.
    /// Initialize with `Default::default()` when constructing a solid.
    #[serde(skip)]
    pub mass_properties_cache: crate::MassPropertiesCache,
    pub id: u64,
    pub vertices: Vec<VertexRecord>,
    pub edges: Vec<EdgeRecord>,
    pub shells: Vec<ShellRecord>,
    #[serde(default)]
    pub genus: i64,
}

/// The closed kind of a [`ValidationIssue`]. Only the kinds a dispatcher
/// needs are minted (the boolean's endpoint-gap repair lane keys on
/// `CurveVertexGap`); everything else is `Other` and carries its message as
/// the payload. A dispatcher matches the kind, never the message text.
///
/// On the wire it is a nested object, `"kind":{"name":"curve_vertex_gap",
/// "at_start":true}`, NOT flattened into the issue: serde's `default` does not
/// apply to a flattened internally-tagged enum, and a report written before
/// kinds existed must still read back (as `Other`).
#[derive(Clone, Copy, Debug, Default, Deserialize, Eq, PartialEq, Serialize)]
#[serde(tag = "name", rename_all = "snake_case")]
pub enum IssueKind {
    /// An edge curve's start (`at_start`) or end does not land on its vertex.
    CurveVertexGap { at_start: bool },
    /// An edge used fewer than twice (a boundary the shell leaves open).
    OpenEdge,
    /// An edge used more than twice.
    OverUsedEdge,
    /// The Euler formula does not close.
    GenusMismatch,
    #[default]
    Other,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ValidationIssue {
    pub severity: &'static str,
    #[serde(default)]
    pub kind: IssueKind,
    pub message: String,
}

impl ValidationIssue {
    fn error(message: impl Into<String>) -> Self {
        Self::error_kind(IssueKind::Other, message)
    }

    fn error_kind(kind: IssueKind, message: impl Into<String>) -> Self {
        Self {
            severity: "error",
            kind,
            message: message.into(),
        }
    }

    fn warning(message: impl Into<String>) -> Self {
        Self {
            severity: "warning",
            kind: IssueKind::Other,
            message: message.into(),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct ValidationReport {
    pub issues: Vec<ValidationIssue>,
    pub wire_warnings: Vec<ValidationIssue>,
    pub max_pcurve_error: f64,
}

#[path = "topology/queries.rs"]
mod queries;
pub(crate) use queries::{nearest_edge, edge_endpoint_alignment, coedge_neighbor_endpoints, edge_use_counts};

#[path = "topology/seam.rs"]
mod seam;
#[path = "topology/validate.rs"]
mod validate;
#[path = "topology/primitives.rs"]
mod primitives;

pub use seam::{
    analyze_doubly_periodic_seam_band, loop_seam_offsets, seam_band_uv_polygon, seam_band_uv_polygon_with,
};
pub(crate) use seam::doubly_periodic_has_only_collapsed_loops;
pub(crate) use validate::{adaptive_coedge_error, coedge_error_floored};
pub use primitives::{make_box_brep, make_cylinder_brep, make_pyramid_brep};

