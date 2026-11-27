//! Constraint solvers of the BREP kernel: 2D and 3D sketches and assemblies.
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
//! - the 2D sketch solver;
//! - the 3D sketch solver;
//! - the assembly mate solver, its resolution of mates against parts, and assembly ids.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology). It is required by [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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

#[path = "solvers/sketch_solver.rs"]
#[doc(hidden)]
pub mod sketch_solver;
pub use sketch_solver::{
    is_sketch_origin_point_id, sketch_id_key, mint_sketch_id, solve_sketch, solve_sketch_from_json,
    solve_sketch_min_change,
    SketchSolverSettings, SolveSketchRequest, SKETCH_ORIGIN_POINT_ID,
};
#[path = "solvers/assembly_solver.rs"]
#[doc(hidden)]
pub mod assembly_solver;
pub use assembly_solver::{
    solve_assembly, AssemblyBody, AssemblyMate, AssemblySolution, AssemblySolveOptions, BodyFreedom, BodyPose,
    LinearAlgebra, MateAlign, MateAxis, MateCurve, MateKind, MatePlane, MateResidualReport,
    SolveStrategy, SymmetricElement,
};
// Assembly selection resolution: a namespaced selection ref resolved to an
// analytic frame (plane/axis/sphere/circle/line/point) read from the exact
// BREP, feeding `MateKind` inputs in component-local coordinates.
#[path = "solvers/sketch3d_solver.rs"]
#[doc(hidden)]
pub mod sketch3d_solver;
pub use sketch3d_solver::{
    sketch3d_constraint_type, solve_sketch3d, solve_sketch3d_doc, solve_sketch3d_from_json,
    Sketch3dConstraint, Sketch3dConstraintType, Sketch3dDiagnostics, Sketch3dDoc,
    Sketch3dGeometry, Sketch3dPoint, Sketch3dSolveOptions, SolveSketch3dRequest,
    SKETCH3D_CONSTRAINT_TYPES,
};
#[path = "solvers/assembly_resolve.rs"]
#[doc(hidden)]
pub mod assembly_resolve;
pub use assembly_resolve::{
    is_component_reference, resolve_component_point, resolve_edge_selection,
    resolve_face_selection, resolve_named_selection, resolve_vertex_selection,
    split_component_namespace, ResolveError, SelectionGeometry,
};

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
#[doc(hidden)]
#[path = "solvers/assembly_ids.rs"]
pub mod assembly_ids;
