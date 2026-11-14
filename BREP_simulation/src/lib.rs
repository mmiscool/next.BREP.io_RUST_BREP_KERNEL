//! Motion studies for BREP assemblies.
//!
//! A study lives in its assembly document's `simulation` block ([`model`]).
//! Motors are mates: the author picks distance or angle mates and drives each
//! at a constant speed or along a motion profile, and every other part follows
//! through the assembly solver on each frame. Clips drive the rest of the
//! timeline (mate on/off, camera, visibility, transparency, sections, exploded
//! views). [`sample`] turns a study at one time into what the caller applies:
//! mate overrides for the solver and the display state. [`forms`] describes a
//! motor's and a clip's fields for a schema-driven form. This crate needs
//! neither the kernel nor a GPU.
//!
//! ```rust
//! use brep_simulation::{Drive, Motor, Simulation};
//!
//! let mut sim = Simulation::default();
//! let study_id = sim.add_study("Open the lid");
//! let motor_id = sim.next_id("MOT");
//! let study = sim.study_mut(&study_id).unwrap();
//! study.motors.push(Motor {
//!     id: motor_id,
//!     constraint: "ANG1".into(),
//!     from: 0.0,
//!     to: 2.0,
//!     drive: Drive::Speed { speed: 45.0 },
//! });
//! // One second in, a 45 degree/s motor has moved the mate 45 degrees on from
//! // its document value (10 here).
//! let frame = study.sample(1.0, &|_: &str| Some(10.0));
//! assert_eq!(frame.mates["ANG1"].value, Some(55.0));
//! ```

pub mod forms;
pub mod model;
pub mod sample;

pub use model::{Clip, ClipBody, Drive, Motor, Simulation, Study};
pub use sample::{FrameSample, MateOverride};
