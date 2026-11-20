//! Direct editing of B-rep faces: move, rotate, offset, draft, delete and refit.
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
//! - moving, rotating and offsetting faces, each checked against the soundness floor (with `*_unchecked` siblings);
//! - drafting faces, deleting faces and healing the gap, and refitting the neighbours;
//! - the typed reasons a face edit is refused.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology), [`BREP_surface_offset`](https://crates.io/crates/BREP_surface_offset), [`BREP_boolean`](https://crates.io/crates/BREP_boolean), [`BREP_construction`](https://crates.io/crates/BREP_construction), [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset). It is required by [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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
#[macro_use]
extern crate brep_blend;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_blend::*;
#[macro_use]
extern crate brep_offset;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_offset::*;

#[path = "edit/direct_edit.rs"]
#[doc(hidden)]
pub mod direct_edit;
pub use direct_edit::{
    delete_face_and_heal, delete_faces_and_heal, draft_faces, draft_faces_parted, draft_faces_unchecked, move_faces, move_faces_unchecked,
    offset_freeform_face, offset_freeform_face_unchecked,
    offset_revolution_face, offset_revolution_face_unchecked, offset_ruled_face, offset_ruled_face_unchecked, offset_sphere_face,
    offset_sphere_face_unchecked, offset_torus_face, offset_torus_face_unchecked,
    rebuild_displaced_blends, with_blend_rebuild_decline,
    recut_moved_planes, recut_moved_planes_rigid, redrill_moved_bore, recut_convex_body, recut_moved_planes_unchecked, reblend_lost_fillets, resolve_face_by_point, rotate_faces, rotate_faces_unchecked,
    route_reading, RouteReading,
};
pub use direct_edit::{face_refusal_slugs, face_transform_reason, FaceTransformReason};
pub use direct_edit::{deform_shell, ShellDeformPick, ShellDeformReport, ShellDeformSpec};
pub use direct_edit::{deform_face, face_deform_default_anchor, FaceDeformPick, FaceDeformReport, FaceDeformSpec};
pub use direct_edit::{
    refit_faces_as_one, refit_refusal_phrases, RefitCarrier, RefitReport, RefitSurfaceKind,
};

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
