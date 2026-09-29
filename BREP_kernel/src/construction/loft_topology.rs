use crate::sweep_topology::{curve_to_plane_parameters, parameter_line, profile_area};
use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{interpolate_curve, make_plane, NurbsCurve, NurbsSurface, Vec3, Vec4};
use crate::{KernelRefusal, KernelStage, OrRefuse};

#[path = "loft_topology/basic.rs"]
mod basic;
#[path = "loft_topology/guided.rs"]
mod guided;
#[path = "loft_topology/closed.rs"]
mod closed;

pub use basic::{loft_profile_brep, loft_profile_brep_tangent};
use basic::closed_points;

pub use closed::loft_profile_brep_closed;
pub(crate) use closed::loft_profile_brep_closed_shifted;

pub use guided::{loft_profile_brep_guided, loft_profile_brep_guided_frame};

#[path = "loft_topology/validation.rs"]
mod validation;
use validation::validate_sections;
