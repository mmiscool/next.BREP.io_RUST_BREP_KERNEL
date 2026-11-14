//! Integration boundary between neutral recognition and BREP topology.
//!
//! This crate joins the kernel-free `brep_ransac` recognizer (whose public API
//! it re-exports) to `brep_kernel`:
//!
//! - [`brep`] converts between kernel meshes/surfaces and recognition carriers
//!   and attaches face metadata;
//! - [`stl`] reads ASCII and binary STL with deterministic vertex welding;
//! - [`stl_conversion`] turns a welded STL mesh into a validated STEP document
//!   ([`stl_conversion::convert_stl_mesh_to_step`]), keeping analytic regions
//!   where their boundaries can be rebuilt and otherwise able to fall back to
//!   a faceted BREP;
//! - [`import_session`] exposes the region analysis and per-region refits of an
//!   interactive mesh import;
//! - [`step_validation`] checks STEP faces against exact analytic carriers.
//!
//! See the README for an overview.

pub use brep_ransac::*;

mod hybrid_region_brep;
pub use hybrid_region_brep::segment_for_hybrid;
mod numerical;

pub mod brep;
pub mod import_session;
pub mod step_validation;
pub mod stl;
pub mod stl_conversion;

mod hybrid_freeform_split;
mod hybrid_recovery;
mod hybrid_whole_freeform;
mod hypothesis_switch;
