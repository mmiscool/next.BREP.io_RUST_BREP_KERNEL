//! The shared CAM data model: everything the document persists and every
//! strategy reads or writes. One owner (lane C0); field additions arrive as
//! hunks through the integrator.
//!
//! Serialization is a persistence contract: structs are `camelCase`, enums
//! with payloads are internally tagged on `kind`, optional and newer fields
//! carry `#[serde(default)]`, maps are ordered.

mod contact;
mod geometry;
mod ids;
mod machine;
mod operation;
mod probe;
mod setup;
mod tool;
mod toolpath;

pub use contact::{ClPoint, Contact, ContactType};
pub use geometry::{Bounds3, Point2, Point3, RigidTransform, TransformError, Vector3};
pub use ids::{
    ChainRef, ChainSide, ChainSource, EdgeRef, HolderId, MachineId, OperationId, SetupId, TargetRef, TargetSelection, ToolId,
};
pub use machine::{
    FeedLimits, Kinematics, Link, LinkShape, LinkSolid, Machine, MachineKind, MachineModel, RotaryAxis, SpindleLimits,
    TravelLimits,
};
pub use operation::{
    Adaptive2dParams, Adaptive3dParams, AxisMode, UnreachablePolicy, BoreParams, Chamfer2dParams, CircularParams, Containment,
    ContainmentMode, Contour2dParams, Contour3dParams, CutDirection, Depths, DrillCycle, DrillParams,
    EngraveParams, EnumChoice, FaceParams, FeedOptimizationParams, FlowParams, HorizontalParams, Lead, LeadKind, LinkingParams,
    ManualNcKind, ManualNcParams, MorphParams, MorphedSpiralParams, MultiAxisContourParams, MultiAxisFinishParams, Operation, ParallelParams, PencilParams,
    PatternKind, PatternOrder, PatternParams, Pocket2dParams, Pocket3dParams, PolishParams, ProjectParams, RadialParams, Ramp, RampFinishParams, RampKind,
    RestFinishParams, RestSource, RetractPolicy, RotaryFinishParams, RotaryWrapParams, RoughingDirection, ScallopParams, SlotParams, SpiralParams, SteepShallowParams,
    SmoothingParams, Stepover, StockToLeave, Strategy, SwarfParams, Tab, ThreadMillParams, TraceParams,
    TurnFaceParams, TurnGrooveParams, TurnPartOffParams, TurnProfileParams, TurnThreadParams,
};
pub use probe::{ProbeCycle, ProbeGeometryParams, ProbeKind, ProbePoint, ProbeSettings, ProbeWcsParams};
pub use setup::{Heights, ProgramInfo, Setup, Stock, WorkOffset};
pub use tool::{
    Coolant, FeedsSpeeds, Holder, HolderSegment, SpindleDirection, Tool, ToolGeometry, ToolKind,
};
pub use toolpath::{
    ApproximationLimit, ArcPlane, Cycle, Diagnostic, EmptyRegion, Feed, Motion, MotionKind,
    OperationReport, OperationStatus, ProgramMotion, RapidTag, RotaryWord, Toolpath, TurnThreadCycle, WorkCounts,
};

