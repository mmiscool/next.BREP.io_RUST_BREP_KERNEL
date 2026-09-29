//! Integration boundary between neutral recognition and BREP topology.

pub use brep_ransac::*;

mod hybrid_region_brep;
mod numerical;

pub mod brep;
pub mod step_validation;
pub mod stl;
pub mod stl_conversion;
