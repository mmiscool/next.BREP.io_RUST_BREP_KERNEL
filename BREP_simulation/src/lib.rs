//! Motion studies for BREP assemblies.
//!
//! A study lives in its assembly document's `simulation` block ([`model`]).
//! Motors are mates: the author picks distance or angle mates and drives each
//! at a constant speed or along a motion profile, and every other part follows
//! through the assembly solver on each frame. Clips drive the rest of the
//! timeline (mate on/off, camera, visibility, transparency, sections, exploded
//! views). [`sample`] turns a study at one time into what the caller applies:
//! mate overrides for the solver and the display state. This crate needs
//! neither the kernel nor a GPU.

pub mod forms;
pub mod model;
pub mod sample;

pub use model::{Clip, ClipBody, Drive, Motor, Simulation, Study};
pub use sample::{FrameSample, MateOverride};
