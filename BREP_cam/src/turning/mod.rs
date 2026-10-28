//! Turning geometry in machine XZ: X is always a physical radius internally.
//! Diameter programming is a post conversion, never a geometry scale factor.
mod tooling;
pub use tooling::*;


pub mod profile;
pub mod envelope;

pub mod schedule;
mod generate;
#[path = "../strategy/turn_face.rs"]
pub mod face;
#[path = "../strategy/turn_part_off.rs"]
pub mod part_off;

#[path = "../strategy/turn_groove.rs"]
pub mod groove;

#[path = "../strategy/turn_thread.rs"]
pub mod thread;
pub mod thread_cycle;
pub mod thread_generation;

#[path = "../strategy/turn_drill.rs"]
pub mod drill;

pub mod profile_path;
#[path = "../strategy/turn_profile_rough.rs"]
pub mod profile_rough;
#[path = "../strategy/turn_profile_finish.rs"]
pub mod profile_finish;
// Lane C11b: mill-turn live tooling and axisymmetric turning stock simulation.
pub mod millturn;
pub mod sim;
