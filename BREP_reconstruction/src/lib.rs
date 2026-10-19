//! Integration boundary between neutral recognition and BREP topology.

pub use brep_ransac::*;

mod hybrid_region_brep;
pub use hybrid_region_brep::segment_for_hybrid;
mod numerical;

pub mod brep;
pub mod step_validation;
pub mod stl;
pub mod stl_conversion;

mod hybrid_freeform_split;
mod hybrid_recovery;
mod hybrid_whole_freeform;
