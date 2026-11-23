//! Booleans and healing of the BREP kernel: imprint, fragment, boolean assembly, sewing and face merging.
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
//! - face splitting, imprinting, edge splits and fragmentation;
//! - boolean union, subtraction and intersection (`boolean_operation`) with its acceptance checks and oracle;
//! - healing: curve coalescing, same-surface face merging, sewing and pinched-vertex splits;
//! - triangle-soup repair into a faceted B-rep, mesh segmentation and spherical mesh unions.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology). It is required by [`BREP_construction`](https://crates.io/crates/BREP_construction), [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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

#[path = "edit/split.rs"]
#[doc(hidden)]
pub mod split;
pub use split::{
    split_solid_by_face_surface, solid_piece_by_plane, split_solid_by_plane, split_solid_by_surface, SplitSurface,
};
#[path = "csg/imprint.rs"]
#[doc(hidden)]
pub mod imprint;
pub use imprint::{
    build_imprints, EdgeSplitRecord, FaceImprints, FaceKey, FacePcurve, ImprintOptions,
    ImprintPieceRecord, ImprintResultRecord, ImprintVertex, TangentSectionRecord,
};
#[path = "csg/feature_scale.rs"]
#[doc(hidden)]
pub mod feature_scale;
#[path = "csg/edge_split.rs"]
#[doc(hidden)]
pub mod edge_split;
pub use edge_split::{apply_edge_splits, apply_edge_splits_with_map};
#[path = "csg/fragment.rs"]
#[doc(hidden)]
pub mod fragment;
pub use fragment::{
    fragment_face, fragment_solid, FaceFragmentRecord, FragmentCoedge, FragmentEdgeSource,
    FragmentLoop, FragmentOrder,
};
#[path = "csg/boolean/mod.rs"]
#[doc(hidden)]
pub mod boolean;
pub use boolean::{
    boolean_operation, boolean_operation_nary, boolean_operation_nary_with_options, boolean_operation_with_diagnostics, boolean_operation_with_diagnostics_at_offset, boolean_split_operands,
    BooleanOperation, BooleanOptions,
};
#[path = "csg/oracle.rs"]
#[doc(hidden)]
pub mod oracle;
pub use oracle::{
    boolean_residual_fusables, boolean_semantic_disagreement, boolean_semantic_disagreement_nary,
    OracleReport, ResidualFusable, ResidualKind, SemanticDisagreement, DISAGREEMENT_THRESHOLD,
};
#[path = "healing/heal.rs"]
#[doc(hidden)]
pub mod heal;
// The soundness ACCEPTANCE: the one place a construction lane asks "is this
// result sound enough to return", repairs it where it can and refuses it by
// name where it cannot. Declared beside the heal pre-pass it is the mirror of
// — that one removes noise from an operand before the work, this one refuses a
// wrong answer after it.
#[path = "healing/accept.rs"]
#[doc(hidden)]
pub mod accept;
pub use accept::{
    accept_sound, accept_sound_against, accept_sound_against_scoped, resolve_face_crossing, resolve_face_crossing_between,
    take_crossing_repairs,
    CrossingRepairRecord,
};
#[path = "healing/coalesce.rs"]
#[doc(hidden)]
pub mod coalesce;
pub use coalesce::{concatenate_exact_curve_pieces, merge_curve_continuation_edges};
#[path = "healing/face_merge.rs"]
#[doc(hidden)]
pub mod face_merge;
// Retained for potential mesh-import repair; no exact operation may fall
// back to it (offset shell must produce real surfaces or fail loudly).
#[allow(dead_code)]
#[path = "healing/faceted_repair.rs"]
#[doc(hidden)]
pub mod faceted_repair;
pub use face_merge::{merge_faces_within, merge_same_surface_faces, merge_same_surface_faces_excluding, merge_same_surface_faces_scoped};
pub use faceted_repair::{mesh_to_faceted_brep, repair_triangle_soup, MeshRepairReport};
#[path = "healing/sew.rs"]
#[doc(hidden)]
pub mod sew;
pub use sew::{sew_rotated_copies, sew_solid, split_pinched_vertices, SewReport};
#[path = "meshing/mesh_segment.rs"]
#[doc(hidden)]
pub mod mesh_segment;
pub use mesh_segment::{
    mesh_regions_to_brep, segment_mesh_faces, MeshRegion, MeshSegmentation, RegionCarrier,
    SegmentOptions, UNASSIGNED_REGION,
};

#[path = "meshing/spherical_union.rs"]
#[doc(hidden)]
pub mod spherical_union;
pub use spherical_union::reconstruct_spherical_mesh_union;

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
