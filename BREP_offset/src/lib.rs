//! Solid offsets, shells and thickening of the BREP kernel.
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
//! - shelling and offset shells, sharp and rounded;
//! - thickening of sheets and faces, and sheets extruded from curves.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology), [`BREP_surface_offset`](https://crates.io/crates/BREP_surface_offset), [`BREP_boolean`](https://crates.io/crates/BREP_boolean), [`BREP_construction`](https://crates.io/crates/BREP_construction), [`BREP_blend`](https://crates.io/crates/BREP_blend). It is required by [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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

#[path = "offset/thicken_faces.rs"]
#[doc(hidden)]
pub mod thicken_faces;
pub use thicken_faces::{extrude_curves_to_sheet, thicken_sheet_faces, THICKEN_FACES_REFUSAL};
#[path = "offset/shell_rounded.rs"]
#[doc(hidden)]
pub mod shell_rounded;
pub use shell_rounded::{offset_shell_rounded, SHELL_ROUNDED_REFUSAL};
#[path = "offset/thicken.rs"]
#[doc(hidden)]
pub mod thicken;
pub use thicken::{thicken_face_sheet, thicken_trimmed_sheet};
#[path = "offset/offset_shell.rs"]
#[doc(hidden)]
pub mod offset_shell;
pub use offset_shell::{
    offset_shell, offset_shell_by_boolean, offset_shell_closed, offset_shell_with_diagnostics, offset_shell_with_options, offset_solid, OffsetCorners, ShellOptions,
    OffsetFaceRole, OffsetShellFaceImageRecord, OffsetShellResultRecord,
};
pub use offset_shell::{is_offset_shell_no_retained_face, is_offset_shell_open_curved_rim};

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
