//! Fillets, chamfers and blends of the BREP kernel.
//!
//! It is one of the implementation crates the kernel is built from. Applications
//! should depend on [`BREP_kernel`](https://crates.io/crates/BREP_kernel), which
//! re-exports the public API of every kernel crate at its root (`brep_kernel::…`)
//! and adds the feature pipeline and the STEP, IGES, Parasolid (import), 3MF,
//! GLB, STL and OBJ exchange.
//! Many items here are public only so the other kernel crates can reach them;
//! those are hidden from the documentation and carry no stability promise beyond
//! the kernel's own. The kernel crates are released together, at one version.
//!
//! It contains:
//!
//! - constant, variable (radius-law) and conic edge fillets;
//! - chamfers;
//! - the blend surfaces, their trims and their junctions with the neighbouring faces.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology), [`BREP_surface_offset`](https://crates.io/crates/BREP_surface_offset), [`BREP_boolean`](https://crates.io/crates/BREP_boolean), [`BREP_construction`](https://crates.io/crates/BREP_construction). It is required by [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
//!
//! ## Building
//!
//! The crate builds natively and for `wasm32-unknown-unknown`. The `parallel`
//! feature runs some per-face loops on rayon; `profile-wall` turns on a diagnostic
//! wall-time partition. Both are off by default, and `BREP_kernel` forwards them.
//!
//! The packaged crate ships the library sources, the README, the licence and the
//! manifest; it carries no test suite.
//!
//! ## License
//!
//! The Autodrop3d licence in `LICENSE.md`, shipped in the crate (`license-file` in
//! the manifest). Source: <https://github.com/mmiscool/next.BREP.io_RUST_BREP_KERNEL>
// Implementation crate of the kernel: its formerly crate-private items are public for the crates above
// (documented hidden), and their docs may name private helpers.
#![allow(rustdoc::private_intra_doc_links)]
use serde::{Deserialize, Serialize};
#[allow(unused_imports)]
use wasm_bindgen::prelude::*;
#[macro_use]
extern crate brep_geometry;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_geometry::*;
#[macro_use]
extern crate brep_topology;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_topology::*;
#[macro_use]
extern crate brep_surface_offset;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_surface_offset::*;
#[macro_use]
extern crate brep_boolean;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_boolean::*;
#[macro_use]
extern crate brep_construction;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_construction::*;

#[path = "blending/blend/mod.rs"]
#[doc(hidden)]
pub mod blend;
pub use blend::{
    take_blend_approximations, take_blend_notes,
    blend_timing_capture, take_blend_timings, BlendTiming,
    blend_closed_edge, blend_edge_variable, blend_open_edge, blend_smooth_chain,
    round_convex_corner,
};
#[path = "blending/fillet.rs"]
#[doc(hidden)]
pub mod fillet;
pub use fillet::{
    chamfer_edge, chamfer_edge_angle, chamfer_edge_asymmetric, chamfer_edges_angle,
    chamfer_edges_asymmetric, fillet_edge, fillet_edge_conic, fillet_edges, fillet_edges_conic, fillet_edges_reported, fillet_edges_reported_at_offset,
    fillet_edges_variable, fillet_edges_variable_law, fillet_edges_variable_vertex_radii,
    BlendSelection,
};
#[path = "blending/law.rs"]
#[doc(hidden)]
pub mod law;
pub use law::{LawSegment, RadiusLaw};

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
