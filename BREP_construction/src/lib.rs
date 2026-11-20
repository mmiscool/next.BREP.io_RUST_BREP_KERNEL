//! Solid construction of the BREP kernel: extrusions, revolutions, sweeps and lofts.
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
//! - extrusions (including up to a face), revolutions and lofts, with lofts along guide curves and guide chains;
//! - sweeps of a profile along a path, with round corners, and solid sweeps of a body along a motion;
//! - ellipses, ellipsoids and periodic B-splines;
//! - faces and solids from surfaces, sheets and polygons.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry), [`BREP_topology`](https://crates.io/crates/BREP_topology), [`BREP_boolean`](https://crates.io/crates/BREP_boolean). It is required by [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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
extern crate brep_boolean;
#[allow(unused_imports)]
#[doc(no_inline)]
pub use brep_boolean::*;

#[path = "construction/solid_sweep.rs"]
#[doc(hidden)]
pub mod solid_sweep;
pub use solid_sweep::{
    sweep_solid_within, OutlinePiece, ScrewStep, SolidMotion, SweepClip, SweptTool, SOLID_SWEEP_REFUSAL,
};
pub use planar_region::{cut_planar_sweep_from_prism, region_cut_on, swept_region_holds, target_outline_holds};
#[path = "construction/turn_window.rs"]
mod turn_window;
#[path = "construction/slide_prism.rs"]
mod slide_prism;
#[path = "construction/solid_sweep_bar.rs"]
mod solid_sweep_bar;
#[path = "construction/solid_sweep_caps.rs"]
#[doc(hidden)]
pub mod solid_sweep_caps;
#[path = "construction/solid_sweep_trim.rs"]
#[doc(hidden)]
pub mod solid_sweep_trim;
#[path = "construction/planar_region.rs"]
#[doc(hidden)]
pub mod planar_region;
pub use solid_sweep_caps::{sweep_solid, sweep_prism_along, sweep_solid_along, sweep_solid_clipped, sweep_solid_parts, sweep_solid_rising, sweep_solid_tumbled, sweep_tube_slid_across, FrameCurve, RisingTurn, TwoTurns, SolidTool, SOLID_SWEEP_WHOLE_REFUSAL};
#[path = "construction/mill_program.rs"]
pub mod mill_program;
pub use mill_program::{flat_mill_region_prisms, MillMove, MILL_REGION_REFUSAL};
#[path = "geometry/closed_curves.rs"]
#[doc(hidden)]
pub mod closed_curves;
pub use closed_curves::{make_ellipse, make_ellipse_arc, make_ellipsoid_brep, make_periodic_bspline};
#[path = "construction/surface_faces.rs"]
#[doc(hidden)]
pub mod surface_faces;
pub use surface_faces::{face_from_surface, face_from_surface_trimmed, sheet_from_sheets, solid_from_sheets, SURFACE_FACE_REFUSAL};
#[path = "construction/polygon_solids.rs"]
#[doc(hidden)]
pub mod polygon_solids;
pub use polygon_solids::{make_pyramid_on_polygon, solid_from_polygons};
#[path = "construction/loft_guide_chain.rs"]
#[doc(hidden)]
pub mod loft_guide_chain;
pub use loft_guide_chain::loft_profile_brep_guided_chain;
#[path = "construction/loft_guides.rs"]
#[doc(hidden)]
pub mod loft_guides;
pub use loft_guides::{loft_profile_brep_on_guide_chains, loft_profile_brep_on_guides, loft_profile_brep_on_loose_guide_chains, GuideLoftWall, LOFT_ON_GUIDES_REFUSAL};
#[path = "construction/extrude_up_to.rs"]
#[doc(hidden)]
pub mod extrude_up_to;
pub use extrude_up_to::{extrude_profile_up_to_face, solid_pieces, EXTRUDE_UP_TO_REFUSAL};
#[path = "construction/sweep_round_corners.rs"]
#[doc(hidden)]
pub mod sweep_round_corners;
pub use sweep_round_corners::{sweep_profile_round_corners, SWEEP_ROUND_CORNERS_REFUSAL};
#[path = "construction/sweep_ellipsoid.rs"]
#[doc(hidden)]
pub mod sweep_ellipsoid;
pub use sweep_ellipsoid::sweep_ellipsoid_along_line;
#[path = "construction/sweep_topology.rs"]
#[doc(hidden)]
pub mod sweep_topology;
pub use sweep_topology::is_sweep_tight_bend;
pub use sweep_topology::{
    extrude_profile_brep, extrude_profile_brep_draft, fit_helix_curve, helix_sample_points,
    profile_anchor, rib_from_profile, sweep_bend_profile, sweep_closure, BendStation,
    JointContinuity, PathJoint, SweepClosure,
    RibExtrusion, RibNames, SweepPath, MAX_JOINT_TANGENT_BREAK, SWEEP_TIGHT_BEND_REFUSAL,
    SWEEP_TWIST_CLOSURE_REFUSAL,
    build_swept_envelope, recognize_swept_envelope, swept_envelope_volume, SweptEnvelope,
    SWEEP_ENVELOPE_REFUSAL, sweep_envelope_bodies,
    sweep_profile_along_chain, sweep_profile_along_chain_reported,
    sweep_profile_along_chain_with_stations, sweep_profile_along_chain_with_stations_reported,
    sweep_profile_along_path,
    sweep_profile_along_path_anchored,
    sweep_profile_helix,
    sweep_profile_twisted, sweep_profile_twisted_anchored, sweep_profile_twisted_scaled,
    ProfileAnchor, SectionPlacement,
    StationBudget, SweepStationReport, SweptChain, SWEEP_DEFAULT_TOLERANCE, SWEEP_MAX_STATIONS,
    SWEEP_REFINEMENT_ROUNDS,
};
#[path = "construction/gears.rs"]
pub mod gears;
pub use gears::{cycloidal_gear_section, cycloidal_gear_solid, CycloidalGear, cycloidal_disc_section, cycloidal_disc_solid, cycloidal_pin_ring, CycloidalDisc, worm_section, worm_solid, Worm, geneva_section, geneva_sizes, geneva_solid, GenevaWheel, ratchet_section, ratchet_solid, Ratchet, timing_pulley_section, timing_pulley_solid, TimingPulley, hub_from_section, straight_spline_hub, straight_spline_section, straight_spline_shaft, StraightSpline, helical_solid, helical_transverse, twisted_prism, Hand, HelicalGear, Twist, rack_section, rack_solid, ring_section, ring_solid, ring_solid_helical, sprocket_section, sprocket_solid, spur_section, spur_solid, GearData, GearSection, Rack, RingGear, SectionCurve, Sprocket, SprocketForm, SpurGear, GEAR_FIT_TOLERANCE};
#[path = "construction/revolve_topology.rs"]
#[doc(hidden)]
pub mod revolve_topology;
pub use revolve_topology::{revolve_profile_brep, revolve_profile_brep_named};
#[path = "construction/loft_topology.rs"]
#[doc(hidden)]
pub mod loft_topology;
pub use loft_topology::{
    loft_profile_brep, loft_profile_brep_closed, loft_profile_brep_guided,
    loft_profile_brep_guided_frame, loft_profile_brep_tangent,
};
pub use loft_topology::loft_profile_brep_closed_shifted;

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
