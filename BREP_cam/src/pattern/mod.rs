//! Derived operation-pattern expansion and manual NC entries.
//! Shared persisted parameters live in model; source paths are never mutated.
mod transform;
pub use transform::{Placement, transform_passes};
#[path="../strategy/pattern.rs"]
pub mod generate;
#[path="../strategy/manual_nc.rs"]
pub mod manual_nc;
