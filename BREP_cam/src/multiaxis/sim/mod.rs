//! Multi-axis stock simulation and machine collision (lane C10c): the
//! tri-dexel stock cut along tilted tool axes, link solids on the kinematic
//! chain checked along the resolved machine motion, and deviation maps of
//! multi-axis results through C7's verify module. C7 owns `stock_sim`;
//! this module shares only through its public API.

pub mod deviation;
pub mod machine;
pub mod player;
pub mod tridexel;

pub use deviation::{axis_rotation, deviation_maps, dexel_grid_of};
pub use machine::{check_machine, Link, LinkShape, LinkShapeGeometry, LinkSolid, MachineCheckOptions, MachineCheckReport, MachineCollision, MachineModel};
pub use player::{AdvanceOutcome, DriverOptions, OperationRange, RangeCutter, PlayState, Player, Position, SimDriver, StockModel, Timeline, sim_chain, uses_rotary};
pub use tridexel::{line_in_profile, AxisGrid, CutReport, TriDexel, TriDexelOptions, TriDexelReport};

