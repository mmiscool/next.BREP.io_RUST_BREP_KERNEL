//! Topology layer of the BREP kernel: B-rep solids, validation, soundness, classification, mass properties and tessellation.
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
//! - the B-rep records (`BrepSolid` and its vertices, edges, coedges, loops, faces and shells) and their arena and serialized form;
//! - structural and geometric validation and the soundness checks;
//! - primitive solids (box, cylinder, cone, sphere, torus) and affine transforms, recentring and rescaling;
//! - mass properties integrated from the B-rep;
//! - tessellation, including the watertight display mesh, and mesh welding;
//! - point-in-solid and pair classification;
//! - entity tolerances, canonical carriers, edge sub-curves, orientation and face names.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC), [`BREP_geometry`](https://crates.io/crates/BREP_geometry). It is required by [`BREP_surface_offset`](https://crates.io/crates/BREP_surface_offset), [`BREP_boolean`](https://crates.io/crates/BREP_boolean), [`BREP_solvers`](https://crates.io/crates/BREP_solvers), [`BREP_construction`](https://crates.io/crates/BREP_construction), [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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

// Per-entity measured tolerances: the lazy, capped band an individual edge or
// vertex earns from its own redundant representations, on top of the global
// policy above. Measurement only — no record field, no serialization; see the
// module doc for invariants I1/I2.
#[path = "geometry/entity_tolerance.rs"]
#[doc(hidden)]
pub mod entity_tolerance;
pub use entity_tolerance::{EntityTolerances, EDGE_CAP_FRACTION, VERTEX_CAP_FRACTION};
#[path = "geometry/control_cage.rs"]
#[doc(hidden)]
pub mod control_cage;
pub use control_cage::{
    CagePoint, CagePrimitive, ControlCage, CAGE_DEGREE, MAX_RINGS, MAX_SEGMENTS, MIN_RINGS,
    MIN_SEGMENTS,
};
#[path = "brep/solid_codec.rs"]
#[doc(hidden)]
pub mod solid_codec;
pub use solid_codec::{decode_solid, encode_solid, SolidNames, SOLID_CODEC_VERSION, SOLID_CODEC_VERSION_KIND};
#[path = "brep/topology.rs"]
#[doc(hidden)]
pub mod topology;
pub use topology::{
    make_box_brep, make_cylinder_brep, make_pyramid_brep, topology_kind, BodyKind, BrepSolid, CellRecord, CoedgeRecord,
    FaceRecord, IssueKind, KindReport, LoopRecord, TopologyKind, ValidationReport,
};
#[path = "brep/soundness.rs"]
#[doc(hidden)]
pub mod soundness;
pub use soundness::{
    face_self_intersections, loop_self_crossings, shell_vector_areas, solid_connectivity,
    solid_euler, solid_self_intersections, ConnectivityReport, CrossingConfirmation, EulerReport,
    CertificateScope, CertifiedFacePair, FaceCrossing, FaceFold, FaceVectorArea,
    LoopCrossing, LoopCrossingReport, SelfIntersectionOptions, SelfIntersectionReport,
    ClosureReading, ShellConnectivity, ShellVectorArea, VectorAreaReport, CrossingUndecidedCause, UndecidedCrossing,
    MAX_LISTED_UNDECIDED,
};
#[path = "brep/topology_arena.rs"]
#[doc(hidden)]
pub mod topology_arena;
pub use topology_arena::{
    ArenaCoedge, ArenaEdge, ArenaFace, ArenaLoop, ArenaShell, ArenaVertex, CoedgeId, EdgeId,
    FaceId, LoopId, ShellId, TopologyArena, VertexId,
};
#[path = "brep/analytic_topology.rs"]
#[doc(hidden)]
pub mod analytic_topology;
pub use analytic_topology::{
    make_cone_brep, make_sphere_brep, make_sphere_brep_framed, make_torus_brep,
};
#[path = "geometry/curve_chain.rs"]
#[doc(hidden)]
pub mod curve_chain;
pub use curve_chain::{join_curve_chain, split_surface};
#[path = "brep/transform_topology.rs"]
#[doc(hidden)]
pub mod transform_topology;
#[path = "brep/recentre.rs"]
#[doc(hidden)]
pub mod recentre;
#[path = "brep/boundary_query.rs"]
pub mod boundary_query;
pub use boundary_query::{
    nearest_boundary_point, solid_closest_points, solid_gap, solid_least_box, solid_outward_normals, solid_principal_curvatures, BoundaryFoot,
    BoundaryPlace,
};
#[path = "brep/rescale.rs"]
#[doc(hidden)]
pub mod rescale;
pub use transform_topology::{mirror_brep, transform_brep, AffineTransform};
#[path = "props/mass_properties.rs"]
#[doc(hidden)]
pub mod mass_properties;
pub use mass_properties::{
    curve_arc_length, edge_arc_length, face_area, face_boundary_length, face_volume_contribution,
    face_set_identity, mass_caller, parameter_space_area, solid_edge_length_total,
    solid_mass_properties,
    solid_mass_properties_full, signed_volume_against, solid_signed_volume, solid_signed_volume_for_sign, trim_polygons,
    DensityMassProperties,
    FaceSetIdentity, FullMassProperties, MassCaller, MassProperties, MassPropertiesCache,
};
#[path = "props/carrier_dimensions.rs"]
#[doc(hidden)]
pub mod carrier_dimensions;
pub use carrier_dimensions::{curve_carrier_dimensions, surface_carrier_dimensions, CarrierDimensions};
#[path = "meshing/tessellation.rs"]
#[doc(hidden)]
pub mod tessellation;
pub use tessellation::{tessellate_brep, tessellate_face, TessellationOptions};
#[path = "meshing/watertight_tessellation/mod.rs"]
#[doc(hidden)]
pub mod watertight_tessellation;
pub use watertight_tessellation::{
    sample_edge_polyline_checked, EdgeChordError, EdgeChordFailure, TessellationError,
    tessellate_brep_watertight_checked,
    sample_edge_polylines, sample_edges_encoded, tessellate_brep_watertight,
    tessellate_brep_watertight_face_stride, tessellate_brep_watertight_face_stride_with_samples,
};
#[path = "brep/classification.rs"]
#[doc(hidden)]
pub mod classification;
pub use classification::{
    classify_point, containment_lane, horizon_cross_frame, parameter_point_in_face,
    trim_sample_count, trim_station, ContainmentLane, PointClass, PointClassification,
    PolygonClass, SolidClassifier, COVERING_RIM_STRIP_SWITCH, HORIZON_CONTAINMENT_SWITCH,
    HORIZON_CROSS_FRAME_SWITCH, HORIZON_SINGLE_LOOP_SWITCH, NO_SPHERE_CHART_TRIM_SWITCH,
    SEAM_BAND_MERGED_SWITCH,
};
#[doc(hidden)]
pub use classification::parameter_point_in_face_scan;
#[path = "brep/pair_classification.rs"]
#[doc(hidden)]
pub mod pair_classification;
pub use pair_classification::{
    classify_surface_pair, classify_surface_pair_cached, SurfaceClassifyData,
    SurfacePairClassification, SurfacePairRelation,
};
// The content-keyed memo of the per-coedge readings the heal pre-pass and
// the assembly's inherited-trim challenger take on every boolean.
#[path = "healing/reading_memo.rs"]
#[doc(hidden)]
pub mod reading_memo;
// The MEASURED half of the tolerance model: the deviation actually observed
// between an offset construction and the geometry it was built to reproduce.
// Everything in `geometry/tolerance.rs` above `MeasuredTolerance` DERIVES a
// band from a size before the geometry exists; this measures what the
// construction then did, and compares it against that band. The comparison is
// not itself a gate: the deviation rides the transient construction result
// (`OffsetFaceCarrier::deviation`, `#[serde(skip)]`) and refusing on an
// exceedance is the caller's call, not this module's. Declared before `offset`
// because that module is its first consumer.
#[path = "offset/measure.rs"]
#[doc(hidden)]
pub mod offset_measure;
pub use offset_measure::{
    measure_edge_against_pcurve_image, measure_surface_fit_against_pointwise_offset,
    span_midpoint_error, vertex_endpoint_gap,
};
#[path = "meshing/mesh_weld.rs"]
#[doc(hidden)]
pub mod mesh_weld;
pub use mesh_weld::{canonical_bits, VertexWeld};

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
#[doc(hidden)]
#[path = "brep/canonical_carriers.rs"]
pub mod canonical_carriers;
pub use canonical_carriers::{canonical_carriers_enabled, canonicalise_carriers};
#[doc(hidden)]
#[path = "brep/edge_subcurve.rs"]
pub mod edge_subcurve;
#[doc(hidden)]
#[path = "brep/solid_tolerance.rs"]
pub mod solid_tolerance;
pub use solid_tolerance::{display_chord_tolerance, solid_model_scale, solid_scale};
#[doc(hidden)]
#[path = "brep/construction_profile.rs"]
pub mod construction_profile;
#[doc(hidden)]
#[path = "brep/orientation.rs"]
pub mod orientation;
#[doc(hidden)]
#[path = "brep/face_names.rs"]
pub mod face_names;
