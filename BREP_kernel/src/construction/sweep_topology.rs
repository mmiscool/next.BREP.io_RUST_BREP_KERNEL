use crate::topology::{
    BrepSolid, CoedgeRecord, EdgeRecord, FaceRecord, LoopRecord, ShellRecord, VertexRecord,
};
use crate::{
    boolean_operation, build_pcurve_on_surface, interpolate_curve, loft_profile_brep, make_arc,
    make_extrusion, make_line, make_plane, project_point_to_curve, BooleanOperation,
    BooleanOptions, NurbsCurve, NurbsSurface, Vec3, Vec4,
};
use crate::{KernelRefusal, KernelStage, OrRefuse};

#[path = "sweep_topology/extrude.rs"]
mod extrude;
#[path = "sweep_topology/sweep.rs"]
mod sweep;
#[path = "sweep_topology/envelope.rs"]
mod envelope;
#[path = "sweep_topology/path.rs"]
mod path;
#[path = "sweep_topology/draft.rs"]
mod draft;
#[path = "sweep_topology/miter.rs"]
mod miter;
#[path = "sweep_topology/twist.rs"]
mod twist;
#[path = "sweep_topology/stations.rs"]
mod stations;
#[path = "sweep_topology/rib.rs"]
mod rib;

pub use extrude::extrude_profile_brep;
pub(crate) use crate::curve::{curve_to_plane_parameters, parameter_line};
pub(crate) use extrude::profile_area;

pub use path::{JointContinuity, PathJoint, SweepPath, MAX_JOINT_TANGENT_BREAK};

pub use envelope::{
    build_swept_envelope, recognize_swept_envelope, sweep_envelope_bodies, swept_envelope_volume,
    SweptEnvelope, SWEEP_ENVELOPE_REFUSAL,
};

pub use sweep::{
    fit_helix_curve, helix_sample_points, profile_anchor, sweep_bend_profile, sweep_closure,
    sweep_profile_along_chain, sweep_profile_along_chain_reported,
    sweep_profile_along_chain_with_stations, sweep_profile_along_chain_with_stations_reported,
    sweep_profile_along_path, sweep_profile_along_path_anchored,
    sweep_profile_helix, sweep_profile_twisted, sweep_profile_twisted_anchored, BendStation,
    ProfileAnchor, SectionPlacement, SweepClosure, SweptChain, SWEEP_TIGHT_BEND_REFUSAL,
};
pub use stations::{
    StationBudget, SweepStationReport, SWEEP_DEFAULT_TOLERANCE, SWEEP_MAX_STATIONS,
    SWEEP_REFINEMENT_ROUNDS,
};

pub(crate) use sweep::is_sweep_tight_bend;
pub use twist::SWEEP_TWIST_CLOSURE_REFUSAL;
use twist::{closing_twist, ClosingTwist};

pub use draft::extrude_profile_brep_draft;

pub use rib::{rib_from_profile, RibExtrusion, RibNames};
use rib::{
    arc_window_subrange, classify_profile_segments, junction_edge_curve, offset_junction,
    ruled_between, SegGeom,
};
