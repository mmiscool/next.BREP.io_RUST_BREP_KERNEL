//! Manufacturing instructions for BREP assemblies.
//!
//! An instructions document is its own file: it places one revision of an
//! assembly as a single grounded component and keeps what the author writes
//! in its `instructions` block ([`model`]). This crate is the plain-data half:
//! the block, the step-order [`wizard`], step editing ([`edit`]), re-targeting
//! to a new assembly revision ([`retarget`]), printed page layout
//! ([`layout`]) and the HTML [`player`]. It needs neither the kernel nor a GPU,
//! so a server can serve the player and a test can run the wizard alone.

pub mod edit;
pub mod layout;
pub mod model;
pub mod player;
pub mod retarget;
pub mod source;
pub mod wizard;

pub use model::{Instructions, Step, Target};

