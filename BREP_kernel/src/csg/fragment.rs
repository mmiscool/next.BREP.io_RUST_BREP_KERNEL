use crate::classification::{parameter_point_in_face, PolygonClass};
use crate::imprint::{FaceKey, ImprintPieceRecord, ImprintResultRecord};
use crate::topology::{BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord};
use crate::{
    arrange_segments, interpolate_curve, project_point_to_curve, KnotVector, NurbsCurve,
    NurbsSurface, Segment2, Vec2, Vec3,
};
use rustc_hash::{FxHashMap as HashMap, FxHashSet as HashSet};
use serde::Serialize;
use serde_json::json;

#[path = "fragment/types.rs"]
mod types;
#[path = "fragment/sampling.rs"]
mod sampling;
#[path = "fragment/loops.rs"]
mod loops;
#[path = "fragment/face_split.rs"]
mod face_split;

pub use face_split::{fragment_face, fragment_solid};
pub use types::{FaceFragmentRecord, FragmentCoedge, FragmentEdgeSource, FragmentLoop};
