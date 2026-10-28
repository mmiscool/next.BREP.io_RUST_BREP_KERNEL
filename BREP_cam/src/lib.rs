//! CAM toolpath engine for the BREP kernel: contracts and geometric core.
//!
//! This crate is pure Rust and builds for native targets and for
//! `wasm32-unknown-unknown`. Algorithms work in MACHINE coordinates with the
//! cutter axis along `+Z`; the setup's rigid transform is the single place
//! model coordinates become machine coordinates. Every strategy emits the
//! same typed motion stream ([`model::Toolpath`]) that preview, simulation
//! and the post-processors read, and the same inputs always produce a
//! byte-identical stream.
//!
//! Module map:
//! - [`model`]: setups, machines, tools, operations, targets, toolpaths and reports.
//! - [`numeric`]: the tolerance object.
//! - [`mesh`]: the immutable triangle snapshot with body/face identity.
//! - [`index`]: broad-phase triangle queries.
//! - [`cutter`]: validated cutter profiles.
//! - [`cancel`]: cancellation and progress at bounded work chunks.
//! - [`stock`]: the setup's stock resolved into machine coordinates.
//! - [`stock_sim`]: the Z-dexel stock and exact 3-axis material removal.
//! - [`collision`]: shank/holder/rapid collision checks during simulation.
//! - [`verify`]: the stock-versus-part deviation map.
//! - [`cycle_time`]: machining time estimates from the motion stream.
//! - [`error`]: the crate error type.
//! - `kernel_adapter` (feature `kernel`): BREP bodies to mesh snapshots, the only kernel dependency.

pub mod cancel;
pub mod collision;
pub mod cutter;
pub mod cycle_time;
pub mod error;
pub mod index;
#[cfg(feature = "kernel")]
pub mod kernel_adapter;
pub mod mesh;
pub mod model;
pub mod numeric;
pub mod stock;
pub mod stock_sim;
pub mod verify;
// Lane C2: drop cutter and parallel finishing.
pub mod drop;
#[path = "strategy/parallel.rs"]
pub mod parallel;
// Lane C3: push cutter, waterline and contour finishing.
pub mod push;
pub mod waterline;
// Lane C2 (C9 rows): drop-cutter finishing strategies.
#[path = "strategy/project.rs"]
pub mod project;
#[path = "strategy/radial.rs"]
pub mod radial;
#[path = "strategy/spiral.rs"]
pub mod spiral;
#[path = "strategy/morphed_spiral.rs"]
pub mod morphed_spiral;
#[path = "strategy/pencil.rs"]
pub mod pencil;
#[path = "strategy/scallop.rs"]
pub mod scallop;
// Lane C4: filtering, arc fitting, linking, program assembly and post-processing.
pub mod arcfit;
pub mod filter;
pub mod link;
pub mod post;
pub mod program;
// Lane C8: drilling cycles, hole recognition and ordering.
pub mod drill;
pub mod holes;
// Lane C11: turning.
pub mod turning;
// Lane C12: tool library, feeds & speeds, templates.
pub mod feeds;
pub mod library;
pub mod template;
// Lane C10: machine kinematics and multi-axis strategies.
pub mod kinematics;
// Lane C5: the 2D geometry engine and the 2.5D operations.
pub mod planar;
#[cfg(feature = "parallel")]
pub(crate) mod par;
// Lane C13: 2.5D feature recognition and automatic programming.
pub mod features;
pub mod autoprogram;
#[path = "strategy/face.rs"]
pub mod face;
#[path = "strategy/pocket2d.rs"]
pub mod pocket2d;
#[path = "strategy/contour2d.rs"]
pub mod contour2d;
#[path = "strategy/slot.rs"]
pub mod slot;
#[path = "strategy/trace.rs"]
pub mod trace;
#[path = "strategy/chamfer2d.rs"]
pub mod chamfer2d;
#[path = "strategy/engrave.rs"]
pub mod engrave;
#[path = "strategy/bore.rs"]
pub mod bore;
#[path = "strategy/circular.rs"]
pub mod circular;
#[path = "strategy/thread_mill.rs"]
pub mod thread_mill;
pub mod multiaxis;
#[path = "strategy/rotary.rs"]
pub mod rotary;
#[path = "strategy/swarf.rs"]
pub mod swarf;
#[path = "strategy/multiaxis_contour.rs"]
pub mod multiaxis_contour;
#[path = "strategy/multiaxis_finish.rs"]
pub mod multiaxis_finish;
// Lane C6: adaptive clearing, 3D pocket clearing and rest roughing.
pub mod adaptive;
#[path = "strategy/pocket3d.rs"]
pub mod pocket3d;
#[path = "strategy/rest_rough.rs"]
pub mod rest_rough;
#[path = "strategy/adaptive2d.rs"]
pub mod adaptive2d;
#[path = "strategy/adaptive3d.rs"]
pub mod adaptive3d;

pub use collision::{simulate_checked, simulate_checked_on_machine, CheckedSimulation, Collision, CollisionAgainst, CollisionOptions, CollisionPart};
pub use cancel::{CancelToken, Cancelled, NullSink, Progress, ProgressSink, RecordingSink, Runner, WorkBudget};
pub use cutter::{
    BallConeCutter, BallCutter, BullCutter, ConeCutter, Cutter, CutterError, CutterKind, CutterProfile,
    FlatCutter, ProfilePoint, ProfileSegment, SegmentKind, Shank, ToolMesh, ToolMeshGroup, ToolPart, ToolProfile,
};
pub use cycle_time::{estimate_cycle_time, CycleTime, MotionTime};
pub use error::CamError;
pub use index::{AnyIndex, BruteForceIndex, IndexStats, ProjectionMode, TreeIndex, TreeOptions, TriangleIndex};
pub use mesh::{to_machine, FaceSource, MeshBuildReport, MeshBuilder, MeshError, MeshSnapshot, Triangle, TriangleSource};
pub use model::*;
pub use numeric::{NumericError, Tolerance};
pub use stock::{resolve_stock, ResolvedStock, StockBox, StockCylinder, StockError};
pub use stock_sim::{
    mesh::{stock_mesh, stock_mesh_region, touched_region, StockMesh},
    remaining::{remaining_stock_mesh, remaining_stock_mesh_region, RemainingStockMesh},
    removal::{RemovalOptions, SimulationReport},
    carry::{carry_from_setup, carry_stock, CarriedStock},
    envelope::StockEnvelope,
    job::{job_dexel_options, job_stock, unavailable_warning, uses_pre_op_stock, JobStock, JobStockResult, SetupHistory, SetupStart},
    preop::{PlannedOp, PreOpStockCache, PreOpStockError, PreOperationStock},
    Column, DexelGrid, DexelOptions, StockSimError,
};

#[path = "strategy/flow.rs"]
pub mod flow;

#[path = "strategy/morph.rs"]
pub mod morph;

#[path = "strategy/rest_finish.rs"]
pub mod rest_finish;

#[path = "strategy/linking.rs"]
pub mod strategy_linking;
pub use verify::{deviation_map, DeviationClass, DeviationMap, DeviationOptions, FaceDeviation};

pub mod probe;
#[path="strategy/probe_wcs.rs"]
pub mod probe_wcs;
#[path="strategy/probe_geometry.rs"]
pub mod probe_geometry;

// Lane C1: one generator entry point over every strategy, for the application's worker.
pub mod dispatch;

// Operation patterns and explicit manual NC entries.
pub mod pattern;

// Optional feed optimisation and certified smoothing of linked toolpaths.
pub mod polish;
