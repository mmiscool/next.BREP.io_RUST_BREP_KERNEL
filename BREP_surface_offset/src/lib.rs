//! Surface offsets of the BREP kernel: offset carriers, trims, regularity and fold loci.
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
//! - offset carriers of analytic and NURBS faces;
//! - re-trimming and re-intersecting offset faces;
//! - regularity checks, fold loci and carving of the parts an offset folds over.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology). It is required by [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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

// The shared re-trim helpers — ONE definition of "rebuild a face's carrier and
// pcurves around a boundary that has already moved", shared by push-face,
// face-move and the direct-edit heals. This is the RE-INTERSECTION sense of
// trimming; offset-shell's parametric-image trimming is a different operation
// and deliberately not here (see the module doc). Crate-internal: no consumer
// outside the kernel names these.
#[path = "offset/retrim.rs"]
#[doc(hidden)]
pub mod offset_retrim;
// The RE-INTERSECTION seam — ONE definition of "intersect two carriers to
// remake a trim boundary", with the exact analytic lane first and the general
// marched lane behind it. `offset_retrim` above re-trims a face around a
// boundary that has ALREADY moved; this is the step that moves it, and it is
// what makes a neighbour re-intersection surface-type-blind the way
// offset-shell's imprint driver already is. Crate-internal: no consumer outside
// the kernel names these.
#[path = "offset/reintersect.rs"]
#[doc(hidden)]
pub mod offset_reintersect;
#[path = "offset/offset.rs"]
#[doc(hidden)]
pub mod offset;
pub use offset::{
    offset_face_carrier, offset_face_carrier_measured, offset_face_carrier_sided, offset_surface,
    offset_surface_measured, offset_surface_measured_sided, CarrierDeviation, CarrierExtension,
    MeasuredOffsetSurface, OffsetFaceCarrier, OffsetSurfaceLane,
};
// Region-scoped offset REGULARITY — ONE definition of "does the equidistant
// surface fold over THIS trimmed region", shared by the sheet thickener's gate
// and offset-shell's carrier-collapse lane. Both used to sweep a fixed grid
// over the whole surface DOMAIN, which answers about the carrier instead of the
// face. Crate-internal: no consumer outside the kernel names these.
#[path = "offset/regularity.rs"]
#[doc(hidden)]
pub mod offset_regularity;
// The fold LOCUS — the curve `1 − d·k = 0` itself, marched out of the brackets
// the regularity scan above already finds on its own grid. `regularity` answers
// "does this region fold"; this answers "WHERE does it stop folding", which is
// the boundary a partially-folded trim has to be split along. Crate-internal:
// no consumer outside the kernel names these.
#[path = "offset/fold_locus.rs"]
#[doc(hidden)]
pub mod offset_fold_locus;
// The CARVE — split a trim along the fold locus, keep the regular side, hand
// back the folded one. The two modules above decide THAT a region folds and
// WHERE it stops; this is what both are for, and it is shared by the sheet
// thickener and offset shell for the same reason the scan is. Crate-internal:
// no consumer outside the kernel names these.
#[path = "offset/carve.rs"]
#[doc(hidden)]
pub mod offset_carve;

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
