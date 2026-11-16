//! A native and WebAssembly boundary-representation (B-rep) geometry kernel.
//!
//! Published as `BREP_kernel`; the library name is `brep_kernel`. It is the exact
//! geometry engine of the BREP CAD application and is usable on its own:
//!
//! - exact rational NURBS curves and surfaces with recognised analytic carriers;
//! - manifold B-rep topology with validation ([`BrepSolid`]), and primitives
//!   ([`make_box_brep`], [`make_cylinder_brep`], [`make_sphere_brep`], ...);
//! - booleans ([`boolean_operation`]), fillets and chamfers ([`fillet_edges`],
//!   [`chamfer_edge`]), sweeps ([`sweep_profile_along_path`],
//!   [`sweep_profile_helix`]), lofts including guide curves ([`loft_profile_brep`],
//!   [`loft_profile_brep_on_guides`]), offsets ([`offset_shell`]) and thickening;
//! - mass properties ([`solid_mass_properties`]) and watertight tessellation
//!   ([`tessellate_brep_watertight`]);
//! - STEP ([`import_step`], [`export_step`]), IGES ([`import_iges`],
//!   [`export_iges`]), STL, OBJ and 3MF exchange;
//! - a JSON feature-history pipeline ([`execute_history_json`]).
//!
//! Booleans, blends, sweeps and lofts refuse with a typed [`KernelRefusal`]
//! rather than returning a wrong body; many other entry points return
//! `Result<_, String>`.
//!
//! ```
//! use brep_kernel::{
//!     boolean_operation, make_box_brep, make_cylinder_brep, solid_mass_properties,
//!     tessellate_brep_watertight, BooleanOperation, BooleanOptions, Vec3,
//! };
//!
//! fn main() -> Result<(), String> {
//!     // A 40 x 30 x 20 block with a vertical radius-8 hole through it.
//!     let block = make_box_brep(Vec3::new(0.0, 0.0, 0.0), 40.0, 30.0, 20.0)?;
//!     let hole = make_cylinder_brep(
//!         Vec3::new(20.0, 15.0, -1.0), // base point
//!         Vec3::new(0.0, 0.0, 1.0),    // axis direction
//!         8.0,                         // radius
//!         22.0,                        // height (through the block)
//!     )?;
//!     let part = boolean_operation(&block, &hole, BooleanOperation::Subtract, &BooleanOptions::default())
//!         .map_err(|error| error.to_string())?;
//!
//!     // Mass properties integrated from the B-rep (no display mesh involved).
//!     let props = solid_mass_properties(&part)?;
//!     let expected = 40.0 * 30.0 * 20.0 - std::f64::consts::PI * 8.0 * 8.0 * 20.0;
//!     assert!((props.volume - expected).abs() < 1e-6 * expected);
//!
//!     // Watertight display mesh at a 0.05 chord tolerance.
//!     let mesh = tessellate_brep_watertight(&part, 0.05)?;
//!     assert!(!mesh.indices.is_empty());
//!     Ok(())
//! }
//! ```
//!
//! The crate README lists the capabilities in detail.


#[macro_use]
extern crate brep_solvers;
#[allow(unused_imports)]
use brep_solvers::*;

// The public root API of the kernel, from the crates below.
pub use brep_solvers::sketch_solver::{
    is_sketch_origin_point_id, sketch_id_key, mint_sketch_id, solve_sketch, solve_sketch_from_json,
    SketchSolverSettings, SolveSketchRequest, SKETCH_ORIGIN_POINT_ID,
};
pub use brep_solvers::assembly_solver::{
    solve_assembly, AssemblyBody, AssemblyMate, AssemblySolution, AssemblySolveOptions, BodyFreedom, BodyPose,
    LinearAlgebra, MateAlign, MateAxis, MateCurve, MateKind, MatePlane, MateResidualReport,
    SolveStrategy, SymmetricElement,
};
pub use brep_solvers::sketch3d_solver::{
    sketch3d_constraint_type, solve_sketch3d, solve_sketch3d_doc, solve_sketch3d_from_json,
    Sketch3dConstraint, Sketch3dConstraintType, Sketch3dDiagnostics, Sketch3dDoc,
    Sketch3dGeometry, Sketch3dPoint, Sketch3dSolveOptions, SolveSketch3dRequest,
    SKETCH3D_CONSTRAINT_TYPES,
};
pub use brep_solvers::assembly_resolve::{
    is_component_reference, resolve_component_point, resolve_edge_selection,
    resolve_face_selection, resolve_named_selection, resolve_vertex_selection,
    split_component_namespace, ResolveError, SelectionGeometry,
};


#[macro_use]
extern crate brep_surface_offset;
#[allow(unused_imports)]
use brep_surface_offset::*;
#[macro_use]
extern crate brep_boolean;
#[allow(unused_imports)]
use brep_boolean::*;
#[macro_use]
extern crate brep_construction;
#[allow(unused_imports)]
use brep_construction::*;
#[macro_use]
extern crate brep_blend;
#[allow(unused_imports)]
use brep_blend::*;
#[macro_use]
extern crate brep_offset;
#[allow(unused_imports)]
use brep_offset::*;
#[macro_use]
extern crate brep_direct_edit;
#[allow(unused_imports)]
use brep_direct_edit::*;

// The public root API of the kernel, from the crates below.
pub use brep_surface_offset::offset::{
    offset_face_carrier, offset_face_carrier_measured, offset_face_carrier_sided, offset_surface,
    offset_surface_measured, offset_surface_measured_sided, CarrierDeviation, CarrierExtension,
    MeasuredOffsetSurface, OffsetFaceCarrier, OffsetSurfaceLane,
};
pub use brep_boolean::split::{
    split_solid_by_face_surface, split_solid_by_plane, split_solid_by_surface, SplitSurface,
};
pub use brep_boolean::imprint::{
    build_imprints, EdgeSplitRecord, FaceImprints, FaceKey, FacePcurve, ImprintOptions,
    ImprintPieceRecord, ImprintResultRecord, ImprintVertex, TangentSectionRecord,
};
pub use brep_boolean::edge_split::{apply_edge_splits, apply_edge_splits_with_map};
pub use brep_boolean::fragment::{
    fragment_face, fragment_solid, FaceFragmentRecord, FragmentCoedge, FragmentEdgeSource,
    FragmentLoop, FragmentOrder,
};
pub use brep_boolean::boolean::{
    boolean_operation, boolean_operation_nary, boolean_operation_nary_with_options, boolean_operation_with_diagnostics, boolean_operation_with_diagnostics_at_offset, boolean_split_operands,
    BooleanOperation, BooleanOptions,
};
pub use brep_boolean::oracle::{
    boolean_residual_fusables, boolean_semantic_disagreement, boolean_semantic_disagreement_nary,
    OracleReport, ResidualFusable, ResidualKind, SemanticDisagreement, DISAGREEMENT_THRESHOLD,
};
pub use brep_boolean::accept::{
    accept_sound, accept_sound_against, accept_sound_against_scoped, resolve_face_crossing, resolve_face_crossing_between,
    take_crossing_repairs,
    CrossingRepairRecord,
};
pub use brep_boolean::coalesce::{concatenate_exact_curve_pieces, merge_curve_continuation_edges};
pub use brep_boolean::face_merge::{merge_faces_within, merge_same_surface_faces, merge_same_surface_faces_excluding, merge_same_surface_faces_scoped};
pub use brep_boolean::faceted_repair::{mesh_to_faceted_brep, repair_triangle_soup, MeshRepairReport};
pub use brep_boolean::sew::{sew_solid, split_pinched_vertices, SewReport};
pub use brep_boolean::mesh_segment::{
    mesh_regions_to_brep, segment_mesh_faces, MeshRegion, MeshSegmentation, RegionCarrier,
    SegmentOptions, UNASSIGNED_REGION,
};
pub use brep_boolean::spherical_union::reconstruct_spherical_mesh_union;
pub use brep_construction::solid_sweep::{
    sweep_solid_within, OutlinePiece, ScrewStep, SolidMotion, SweepClip, SweptTool, SOLID_SWEEP_REFUSAL,
};
pub use brep_construction::solid_sweep_caps::{sweep_solid, sweep_prism_along, sweep_solid_along, sweep_solid_clipped, sweep_solid_parts, sweep_solid_rising, sweep_solid_tumbled, FrameCurve, RisingTurn, TwoTurns, SolidTool, SOLID_SWEEP_WHOLE_REFUSAL};
pub use brep_construction::closed_curves::{make_ellipse, make_ellipse_arc, make_ellipsoid_brep, make_periodic_bspline};
pub use brep_construction::surface_faces::{face_from_surface, face_from_surface_trimmed, solid_from_sheets, SURFACE_FACE_REFUSAL};
pub use brep_construction::polygon_solids::{make_pyramid_on_polygon, solid_from_polygons};
pub use brep_construction::loft_guide_chain::loft_profile_brep_guided_chain;
pub use brep_construction::loft_guides::{loft_profile_brep_on_guide_chains, loft_profile_brep_on_guides, loft_profile_brep_on_loose_guide_chains, GuideLoftWall, LOFT_ON_GUIDES_REFUSAL};
pub use brep_construction::extrude_up_to::{extrude_profile_up_to_face, solid_pieces, EXTRUDE_UP_TO_REFUSAL};
pub use brep_construction::sweep_round_corners::{sweep_profile_round_corners, SWEEP_ROUND_CORNERS_REFUSAL};
pub use brep_construction::sweep_ellipsoid::sweep_ellipsoid_along_line;
pub use brep_construction::sweep_topology::{
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
pub use brep_construction::revolve_topology::{revolve_profile_brep, revolve_profile_brep_named};
pub use brep_construction::loft_topology::{
    loft_profile_brep, loft_profile_brep_closed, loft_profile_brep_guided,
    loft_profile_brep_guided_frame, loft_profile_brep_tangent,
};
pub use brep_blend::blend::{
    take_blend_approximations, take_blend_notes,
    blend_timing_capture, take_blend_timings, BlendTiming,
    blend_closed_edge, blend_edge_variable, blend_open_edge, blend_smooth_chain,
    round_convex_corner,
};
pub use brep_blend::fillet::{
    chamfer_edge, chamfer_edge_angle, chamfer_edge_asymmetric, chamfer_edges_angle,
    chamfer_edges_asymmetric, fillet_edge, fillet_edge_conic, fillet_edges, fillet_edges_conic, fillet_edges_reported, fillet_edges_reported_at_offset,
    fillet_edges_variable, fillet_edges_variable_law, fillet_edges_variable_vertex_radii,
    BlendSelection,
};
pub use brep_blend::law::{LawSegment, RadiusLaw};
pub use brep_offset::thicken_faces::{extrude_curves_to_sheet, thicken_sheet_faces, THICKEN_FACES_REFUSAL};
pub use brep_offset::shell_rounded::{offset_shell_rounded, SHELL_ROUNDED_REFUSAL};
pub use brep_offset::thicken::{thicken_face_sheet, thicken_trimmed_sheet};
pub use brep_offset::offset_shell::{
    offset_shell, offset_shell_by_boolean, offset_shell_closed, offset_shell_with_diagnostics, offset_solid, OffsetCorners,
    OffsetFaceRole, OffsetShellFaceImageRecord, OffsetShellResultRecord,
};
pub use brep_direct_edit::direct_edit::{
    delete_face_and_heal, delete_faces_and_heal, draft_faces, draft_faces_parted, draft_faces_unchecked, move_faces, move_faces_unchecked,
    offset_freeform_face, offset_freeform_face_unchecked,
    offset_revolution_face, offset_revolution_face_unchecked, offset_ruled_face, offset_ruled_face_unchecked, offset_sphere_face,
    offset_sphere_face_unchecked, offset_torus_face, offset_torus_face_unchecked,
    rebuild_displaced_blends, with_blend_rebuild_decline,
    recut_moved_planes, recut_moved_planes_rigid, recut_moved_planes_unchecked, reblend_lost_fillets, resolve_face_by_point, rotate_faces, rotate_faces_unchecked,
    route_reading, RouteReading,
};
pub use brep_direct_edit::direct_edit::{face_refusal_slugs, face_transform_reason, FaceTransformReason};
pub use brep_direct_edit::direct_edit::{
    refit_faces_as_one, refit_refusal_phrases, RefitCarrier, RefitReport, RefitSurfaceKind,
};


#[macro_use]
extern crate brep_geometry;
#[allow(unused_imports)]
use brep_geometry::*;
#[macro_use]
extern crate brep_topology;
#[allow(unused_imports)]
use brep_topology::*;

// The public root API of the kernel, from the crates below.
pub use brep_topology::{display_chord_tolerance, solid_model_scale, solid_scale};
pub use brep_geometry::switch;
pub use brep_geometry::debug_flag;
pub use brep_geometry::mode_switch;
pub use brep_geometry::env_read;
pub use brep_geometry::panic_hook;
pub use brep_geometry::text;
pub use brep_geometry::tolerance::{
    curve_model_scale, model_scale, report_scale_migration, SolidScale,
    offset_construction_band, vertex_tolerance_from_edges, KernelTolerances,
    MeasuredTolerance, OFFSET_CONSTRUCTION_FLOOR, OFFSET_CONSTRUCTION_REL,
    VERTEX_MATCH_FLOOR, VOLUME_DRIFT_ABS, VOLUME_DRIFT_REL,
};
pub use brep_geometry::diagnostics::{
    Approximation, ApproximationBudget, Backing, BudgetMechanism, BudgetReason, MeasuredComponent, DiagnosticEvent, DiagnosticSeverity, EdgeOrigin, KernelDiagnostics,
    KernelOutcome, KernelRefusal, OffCarrierEdge,
    KernelStage, OrRefuse, RefusalClass, SoundnessDefect, UndecidedCause,
};
pub use brep_geometry::polygon;
pub use brep_geometry::curve::{
    make_arc, make_circle, make_hyperbola, make_line, make_parabola, uniform_clamped_knots,
    KnotVector, NurbsCurve, Vec4, KNOT_DEDUP_EPS, KNOT_IDENTITY_TOL,
};
pub use brep_geometry::fit::{
    fit_polyline, fit_polyline_at, fit_polyline_of_degree, interpolate_curve, interpolate_curve_closed,
    interpolate_curve_local, interpolate_curve_thinned, interpolate_curve_with_end_tangents,
    simplify_polyline, simplify_polyline_indices, solve_banded, solve_dense, PolylineFit,
    PolylineFitExit, PolylineFitLedger, PolylineFitReport, PolylineFitScope, MAX_FIT_STATIONS,
};
pub use brep_geometry::image_curve::{affine_image_curve, image_curve, image_curve_pair, ImageCurve, ImageCurveTier};
pub use brep_geometry::surface::{
    carrier_preview_patch, make_cone_surface, make_cylinder_surface, make_extrusion, make_plane,
    make_revolution, make_sphere_surface, make_sphere_surface_framed, make_torus_surface,
    ExtendRefusal, NurbsSurface, SurfaceSide, WeightSite, MAXIMUM_FOLD_ANGLE,
    MAXIMUM_GROWTH_RATIO,
};
pub use brep_geometry::coons::{
    coons_bilinear, coons_bilinear_reported, coons_with_ribbons, CompatWork, CoonsRefusal,
    CoonsReport, RibbonReport,
};
pub use brep_geometry::gordon::{gordon_surface, gordon_surface_reported, GordonRefusal, GordonReport};
pub use brep_geometry::analytic_surface::{
    circle_angle_to_parameter, intersect_analytic_pair, plane_ruled_section_arc,
    revolution_structure, AnalyticSurface, RevolutionFrame, RevolutionStructure,
};
pub use brep_geometry::sphere_chart::{
    canonicalize_points, chart_grid_divisions, chart_grid_parameters, corner_index,
    cube_edge_corner, cube_edge_divisions, cube_edge_index, cube_edge_parameters,
    cube_edge_parts, Chart, ChartSegment, ChartSide, SphereAtlas,
    SphericalRegion, CHART_COUNT, CUBE_CORNER_COUNT, CUBE_EDGE_COUNT,
};
#[doc(hidden)]
pub use brep_geometry::projection_profile::report as projection_profile_report;
pub use brep_geometry::projection::{
    project_point_to_curve, project_point_to_surface, project_point_to_surface_seeded,
    CurveProjection, SurfaceProjection,
};
#[doc(hidden)]
pub use brep_geometry::projection::{
    project_point_to_surface_basin_census, project_point_to_surface_general_lanes, BasinCensus,
    GlobalStats,
};
pub use brep_geometry::curve_surface_intersection::{intersect_curve_surface, CurveSurfaceIntersection};
pub use brep_geometry::curve_curve_intersection::{intersect_curves, CurveCurveIntersection};
pub use brep_geometry::surface_surface_intersection::{
    intersect_surfaces, intersect_surfaces_supplemental, SurfaceIntersectionCurve,
    SurfaceIntersectionOptions,
};
pub use brep_geometry::crossing_node::{crossing_node_split_enabled, split_at_crossing_nodes};
pub use brep_geometry::tangent_curve::{
    classify_tangent_contact_at, find_tangent_contact, trace_tangent_curve, TangentContact,
    EXTENDED_RANK_FLOOR,
};
pub use brep_geometry::spatial::{Aabb, Bvh};
pub use brep_geometry::arrangement::{
    arrange_segments, point_in_polygon, segment_intersection, ArrangementPiece, ArrangementRegion,
    CycleUse, Segment2, Vec2,
};
pub use brep_geometry::pcurve::{
    build_pcurve_on_surface, build_pcurve_on_surface_marched, build_pcurve_on_surface_range,
    build_pcurve_on_surface_range_dense, build_pcurve_on_surface_stations, fit_pcurve_on_surface,
    fit_pcurve_on_surface_marched, fit_pcurve_on_surface_range, fit_pcurve_on_surface_range_dense,
    fit_pcurve_on_surface_stations, PcurveFit, PcurveFitExit, PcurveFitLedger, PcurveFitReport,
    PcurveFitScope, PCURVE_OFF_BAR, PCURVE_REFINEMENT_TOLERANCE,
};
pub use brep_geometry::offset_point::{OffsetEvaluator, OffsetNormal, OffsetSample};
pub use brep_geometry::Vec3;
pub use brep_geometry::Mesh;
pub use brep_geometry::make_box;
pub use brep_geometry::make_cylinder;
pub use brep_geometry::union_find;
pub use brep_geometry::union_find::UnionFind;
pub use brep_geometry::poly;
pub use brep_geometry::analytic_surface::{circumcenter, circumcenter_2d, recognize_circle, RecognizedCircle};
pub use brep_geometry::triangle::closest_point_on_triangle;
pub use brep_geometry::tolerance::RIGID_TOL;
pub use brep_topology::entity_tolerance::{EntityTolerances, EDGE_CAP_FRACTION, VERTEX_CAP_FRACTION};
pub use brep_topology::control_cage::{
    CagePoint, CagePrimitive, ControlCage, CAGE_DEGREE, MAX_RINGS, MAX_SEGMENTS, MIN_RINGS,
    MIN_SEGMENTS,
};
pub use brep_topology::solid_codec::{decode_solid, encode_solid, SolidNames, SOLID_CODEC_VERSION, SOLID_CODEC_VERSION_KIND};
pub use brep_topology::topology::{
    make_box_brep, make_cylinder_brep, make_pyramid_brep, topology_kind, BodyKind, BrepSolid, CellRecord, CoedgeRecord,
    FaceRecord, IssueKind, KindReport, LoopRecord, TopologyKind, ValidationReport,
};
pub use brep_topology::soundness::{
    face_self_intersections, loop_self_crossings, shell_vector_areas, solid_connectivity,
    solid_euler, solid_self_intersections, ConnectivityReport, CrossingConfirmation, EulerReport,
    CertificateScope, CertifiedFacePair, FaceCrossing, FaceFold, FaceVectorArea,
    LoopCrossing, LoopCrossingReport, SelfIntersectionOptions, SelfIntersectionReport,
    ClosureReading, ShellConnectivity, ShellVectorArea, VectorAreaReport, CrossingUndecidedCause, UndecidedCrossing,
    MAX_LISTED_UNDECIDED,
};
pub use brep_topology::topology_arena::{
    ArenaCoedge, ArenaEdge, ArenaFace, ArenaLoop, ArenaShell, ArenaVertex, CoedgeId, EdgeId,
    FaceId, LoopId, ShellId, TopologyArena, VertexId,
};
pub use brep_topology::analytic_topology::{
    make_cone_brep, make_sphere_brep, make_sphere_brep_framed, make_torus_brep,
};
pub use brep_topology::curve_chain::{join_curve_chain, split_surface};
pub use brep_topology::transform_topology::{mirror_brep, transform_brep, AffineTransform};
pub use brep_topology::mass_properties::{
    curve_arc_length, edge_arc_length, face_area, face_boundary_length, face_volume_contribution,
    face_set_identity, mass_caller, parameter_space_area, solid_edge_length_total,
    solid_mass_properties,
    solid_mass_properties_full, signed_volume_against, solid_signed_volume, solid_signed_volume_for_sign, trim_polygons,
    DensityMassProperties,
    FaceSetIdentity, FullMassProperties, MassCaller, MassProperties, MassPropertiesCache,
};
pub use brep_topology::carrier_dimensions::{curve_carrier_dimensions, surface_carrier_dimensions, CarrierDimensions};
pub use brep_topology::tessellation::{tessellate_brep, tessellate_face, TessellationOptions};
pub use brep_topology::watertight_tessellation::{
    sample_edge_polyline_checked, EdgeChordError, EdgeChordFailure, TessellationError,
    tessellate_brep_watertight_checked,
    sample_edge_polylines, sample_edges_encoded, tessellate_brep_watertight,
    tessellate_brep_watertight_face_stride, tessellate_brep_watertight_face_stride_with_samples,
};
pub use brep_topology::classification::{
    classify_point, containment_lane, horizon_cross_frame, parameter_point_in_face,
    trim_sample_count, trim_station, ContainmentLane, PointClass, PointClassification,
    PolygonClass, SolidClassifier, COVERING_RIM_STRIP_SWITCH, HORIZON_CONTAINMENT_SWITCH,
    HORIZON_CROSS_FRAME_SWITCH, HORIZON_SINGLE_LOOP_SWITCH, NO_SPHERE_CHART_TRIM_SWITCH,
    SEAM_BAND_MERGED_SWITCH,
};
#[doc(hidden)]
pub use brep_topology::classification::parameter_point_in_face_scan;
pub use brep_topology::pair_classification::{
    classify_surface_pair, classify_surface_pair_cached, SurfaceClassifyData,
    SurfacePairClassification, SurfacePairRelation,
};
pub use brep_topology::offset_measure::{
    measure_edge_against_pcurve_image, measure_surface_fit_against_pointwise_offset,
    span_midpoint_error, vertex_endpoint_gap,
};
pub use brep_topology::mesh_weld::{canonical_bits, VertexWeld};


use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

// The wasm global allocator, when the `wasm-talc` feature asks for it.
#[cfg(all(feature = "wasm-talc", target_family = "wasm", not(target_feature = "atomics")))]
#[global_allocator]
static TALC: talc::wasm::WasmDynamicTalc = talc::wasm::new_wasm_dynamic_allocator();

// The native global allocator, when the `native-rusty-alloc` feature asks for it.
#[cfg(all(feature = "native-rusty-alloc", not(target_family = "wasm")))]
#[cfg(not(feature = "profile-wall"))]
#[global_allocator]
static RUSTY_ALLOC: rusty_alloc_api::RustyAlloc = rusty_alloc_api::RustyAlloc;

#[cfg(all(feature = "native-rusty-alloc", not(target_family = "wasm"), feature = "profile-wall"))]
#[global_allocator]
static RUSTY_ALLOC: wall_profile::Allocator<rusty_alloc_api::RustyAlloc> = wall_profile::Allocator(rusty_alloc_api::RustyAlloc);
#[path = "construction/draft_roof.rs"]
mod draft_roof;
pub use draft_roof::{extrude_profile_brep_draft_roof, DRAFT_ROOF_REFUSAL};
// The Rust feature-history execution engine (migration-plan Stage 4/5 foundation).
// Deserializes the serialized `{ type, inputParams, ... }` descriptor shape, runs
// each feature against a live scene-map of resident handles, and returns per-feature
// results (handles + face/edge names). See feature_pipeline/mod.rs for the contract.
#[path = "feature_pipeline/mod.rs"]
pub mod feature_pipeline;
pub use feature_pipeline::execute_history_json;
pub use feature_pipeline::{first_reference_name, imported_solid_names, is_weld_type, reference_names};
// Transform Face's default pivot (the selection's boundary centre) over the
// results of a replayed prefix, so the gizmo and the stored `pivot` agree with
// the feature by construction.
pub use feature_pipeline::face_transform_pivot;
// Typed, in-process pipeline surface for native consumers (brep-render): run a
// whole history and read the results without a JSON round trip.
pub use feature_pipeline::{
    clear_history_cache, execute_history, execute_history_observed, AddedSolid, Axis,
    ComponentRecord, FeatureDescriptor, FeatureRefusal, FeatureResult, Frame, Fulfilment,
    HistoryProgress, HistoryRequest, Rejected, RejectedKind,
    HistoryResult, PortKind, PortRecord, ProfileLoop, ScenePoint, SketchProfile,
};
// Wire harness: the document's `wireHarness` block (connections), the tail's
// routing report (per-connection status / length / route, per-segment bundle),
// and the sided-port vocabulary the spline attachments share.
pub use feature_pipeline::wire_harness::{
    bundle_diameter, Attachment as SplineAttachment, BundleStatus, PortSide, RouteResult,
    RouteStatus,
    WireHarnessBundle, WireHarnessConnection,
    WireHarnessEndpoint, WireHarnessReport, WireHarnessState, BUNDLE_SOLID_PREFIX,
    WIRE_HARNESS_FEATURE_ID, WIRE_HARNESS_FEATURE_TYPE,
};
// A part's pins ARE its declared connection points, bound BY NAME within one
// symbol unit and its port group: the report of what does not pair, following
// an edit on one side to the other, the unit-to-group mapping, and a placed
// part's pin resolved to a point ADDRESS (refused by name when it names none).
pub use feature_pipeline::part_pins::{
    declarations as declared_ports, declared_endpoints as declared_point_endpoints,
    declared_points, endpoint_occurrence, follow_pin_edit, follow_point_edit, pin_labels,
    pin_point_report, pins,
    point_renames, port_group_for_unit, resolve_component_pin, resolve_part_pin,
    unit_count as symbol_unit_count, unit_groups, DeclaredPin, DeclaredPoint, PinPoint,
    PinPointChange, PinPointProblem, PinPointReport, PointRenames, DEFAULT_PORT_NAME,
    DEFAULT_PORT_PURPOSE, PADS_BLOCK, SYMBOL_BLOCK,
};
// The declared-ports block itself: the schema a part document carries, the
// address scheme, the unit map and the encapsulation-boundary rule — and the
// SEAT, the frame a point's placement is an offset in, with the two mappings
// an editor needs to put a gizmo on one and write the drag back.
pub use feature_pipeline::ports::{
    address as port_address, document_is_boundary, seat_of as port_seat_of, seat_point,
    seat_vector, seated_direction, split_address, unit_map, unseat_point, unseat_vector,
    world_seat, PortDeclaration, PortPoint, PortPointRow, PortSeat, PortsReport, UnitProblem,
    BOUNDARY_KEY, PORTS_BLOCK, PORTS_FEATURE_ID, PORTS_FEATURE_TYPE,
};
// PMI: the document's `pmi` block (views + annotations), the type table with
// its schemas / selection predicates, the tail's resolved report, and the
// presentation layout shared by the viewport and the AP242 export.
pub use feature_pipeline::pmi::{
    balloon_head as pmi_balloon_head, clamp_text_size, format_dimension, format_number,
    nearest_point_on_edge, nearest_point_on_solid, pmi_schema_catalogue, pmi_type,
    nearest_displayed_point, nearest_displayed_point_on_edge, pose_apply as pmi_pose_apply, pose_unapply as pmi_pose_unapply,
    resolve_state as pmi_resolve_state, entry_names_solid, baseline_hide_id, BalloonAnchor, BalloonAnchorMode, CenterCircle, FcfFrame, HalfSpace as PmiHalfSpace, HeadScope as PmiHeadScope, PmiAnnotation,
    PmiAnnotationReport, PmiCamera, PmiDisplay, PmiGeometry, PmiPlane, PmiPose, PmiProjection, PmiReport,
    PmiState, PmiStatus, PmiTypeDef, PmiView, PmiViewReport, SolidPoses as PmiSolidPoses, ToleranceBlock, ToleranceMode, PMI_TYPES,
};
pub use feature_pipeline::pmi::layout::{present as pmi_present, LayoutStyle as PmiLayoutStyle, Presentation as PmiPresentation};
pub use feature_pipeline::pmi::annotations::fcf::{characteristic as pmi_characteristic, Characteristic as PmiCharacteristic, CHARACTERISTICS as PMI_CHARACTERISTICS};
// The sketch loop-id write-back: stamps each closed loop's stable id onto its
// geometries so per-loop face names survive deleting the edge an id came from.
// The editor calls it when a sketch is committed.
pub use feature_pipeline::assign_sketch_loop_ids;
// The 3D sketch feature's reference resolver and record shape, shared with the
// engine-native 3D sketch editor so the editor materializes exactly what the
// feature re-resolves on every run.
pub use feature_pipeline::features::sketch3d::{
    reference_record as sketch3d_reference_record, resolve_reference as resolve_sketch3d_reference,
    ResolvedReference as Sketch3dResolvedReference, REFERENCE_TIP_LENGTH as SKETCH3D_REFERENCE_TIP_LENGTH,
    common_edge_end as sketch3d_common_edge_end, curve_reference_controls as sketch3d_curve_reference_controls, REFERENCE_ANCHOR_EDGES_KEY as SKETCH3D_ANCHOR_EDGES_KEY,
    SKETCH3D_DOC_KEY, SKETCH3D_EDIT_BUTTON, SKETCH3D_FEATURE_TYPE, SKETCH3D_REFERENCES_PARAM,
    SKETCH3D_REFS_KEY,
};
// Sheet-metal flat-pattern (unfold) → 2D vector export, keyed by resident handle.
pub use feature_pipeline::{flat_pattern_dxf, flat_pattern_svg, is_sheet_metal_handle};
// The part Pattern's per-instance suppression list.
pub use feature_pipeline::{pattern_component_instance, pattern_parse_suppressed};
// Native IMPORT3D payload: finished solids → the `io/snapshot` container with
// IMPORT3D's naming stamped on, i.e. the `inputParams.nativeBrep` an import lane
// bakes into a part document (`features/import3d.rs` reads it back verbatim).
pub use feature_pipeline::{native_import_payload, native_import_payload_with_appearance};
// The scene-metadata isolation bracket a payload producer holds across the
// encode, so ambient records of the LIVE document never leak into a new part.
pub use feature_pipeline::IsolatedSceneMetadata;
// The imported-colour read of the name-keyed scene-metadata store. Thread-local
// like the rest of it, so an off-thread runner reads it and ships the result.
pub use feature_pipeline::{scene_metadata_colors_json, scene_metadata_weld_colors_json};
// Family tables (rows, the expression rewrite, the persisted fingerprints): one
// copy for the CAD app and the PLM server.
pub use feature_pipeline::family;
// The assemblies parts library (unique part payloads for ACOMP instances):
// insert / update-refresh mutations and the save-side serialization surface.
pub use feature_pipeline::{
    add_part_to_library, add_part_to_library_impl, install_parts_library, missing_library_parts,
    parts_library_json, parts_library_map, parts_library_revision, refresh_library_entry,
    refresh_library_entry_impl, same_build, follow_saved_rows, follow_saved_rows_known, kernel_source_stamp, stable_json_hash, PartsLibraryEntry, PartsLibraryMap,
};
// Assembly constraint state + exported ABI (state/statuses/DOF/overlay reads,
// constraint CRUD with auto-solve, the document pose/isFixed write-back fold).
pub use feature_pipeline::assembly::{
    assembly_add_constraint_json, assembly_apply_document_json,
    assembly_apply_inferred_constraints_json, assembly_component_entities_json, assembly_dof_json,
    assembly_infer_constraints_json, assembly_inferable_types_json,
    assembly_move_constraint_json, assembly_overlay_json, assembly_pose_updates_json,
    assembly_drag_solve_json, assembly_frame_solve_json, assembly_remove_constraint_json, assembly_resolve_refs_json,
    assembly_run_solve_json,
    assembly_set_constraint_enabled_json, assembly_set_constraint_open_json,
    assembly_state_json, assembly_statuses_json, assembly_update_constraint_json,
    constraint_hidden_params, constraint_schema_catalogue,
    // The native doors: the same mutations with their errors as text. A native
    // caller must use these — a `JsValue` error aborts a native build.
    assembly_add_constraint_impl, assembly_apply_document_impl, assembly_move_constraint_impl,
    assembly_drag_solve_impl, assembly_frame_solve_impl, assembly_flip_plan_json, assembly_rebase_mechanical_impl, assembly_remove_constraint_impl, assembly_run_solve_impl,
    assembly_set_constraint_enabled_impl, assembly_set_constraint_open_impl,
    assembly_update_constraint_impl,
    assembly_session_export, assembly_session_import, assembly_session_serial, RetiredSession, SessionTransfer,
};
pub use feature_pipeline::{AssemblyState, ConstraintEntry};
// The ONE matrix → `{translate, rotateEulerDeg}` (intrinsic XYZ, degrees) pose
// encoder, shared by the solver write-back and any out-of-crate ACOMP author.
pub use feature_pipeline::assembly::transform_to_pose_params;
// The feature-schema catalogue (feature definitions: name + `inputParamsSchema`),
// so a native UI can drive schema-driven feature dialogs without the JSON export.
pub use feature_pipeline::feature_schema_catalogue;
// The dialog field-visibility hook that pairs with the catalogue: given a feature
// type + current param values, which params the schema dialog should hide.
pub use feature_pipeline::feature_hidden_params;
// Selection-context applicability: the context bar's per-feature/-constraint
// show/no-show predicates, plus the constraint type table they pair with.
pub use feature_pipeline::assembly::{constraint_type, ConstraintTypeDef, CONSTRAINT_TYPES, FIXED_IS_NOT_A_CONSTRAINT};
pub use feature_pipeline::{feature_context_applicable, SelectionProbe};
// The pipeline expression evaluator (the history's `expressions` + `configurator`
// variable sheet), so a native UI (brep-render's engine-native sketcher) can
// evaluate a LIVE dimension `valueExpr` against the same variables a committed
// solve would (`features/sketch.rs` `pre_evaluate_expressions`). `Env::build`/
// `eval` are pure arithmetic (no `std::time`), so this stays wasm-clean.
pub use feature_pipeline::Env;
// Expressions sources are kept one statement per line: the editor, the open
// path and the automation setter normalise through this.
pub use feature_pipeline::format_expressions;

/// One-shot: build an [`Env`] from a history's `expressions` source + its
/// `configurator` JSON, then evaluate `source` to a scalar. The small surface the
/// engine-native sketcher's live dimension-value edit uses (deliverable S5.0) —
/// equivalent to `Env::build(expressions, configurator_json)?.eval(source)`.
pub fn eval_expression(
    expressions: &str,
    configurator_json: &serde_json::Value,
    source: &str,
) -> Result<f64, String> {
    Env::build(expressions, configurator_json).and_then(|env| env.eval(source))
}

// In the shared-memory-threaded wasm build, re-export wasm-bindgen-rayon's
// thread-pool initializer as `initThreadPool(numThreads)`. The JS side must
// await it once after wasm init (needs a cross-origin-isolated page —
// COOP/COEP — for SharedArrayBuffer). With the pool live, the ordinary
// `tessellate_watertight_buffers` parallelizes its per-face loop across threads
// (see `tessellate_faces_stride` under the `parallel` feature).
#[cfg(feature = "wasm-threads")]
pub use wasm_bindgen_rayon::init_thread_pool;
#[path = "io/step_matrix.rs"]
mod step_matrix;
#[path = "io/step.rs"]
mod step;
pub use step::{
    assembly_export_tree, audit_step_manifold, audit_step_pcurves, export_step,
    export_step_assembly, export_step_assembly_report, export_step_report,
    export_step_report_named, Mat4, StepAssemblyExport, StepColors, StepExportOccurrence,
    StepExportProduct, StepExportReport, StepPmi, PART_ATTRIBUTES, PART_NUMBER,
};
// Imported appearance (colour) — the carrier between an importer that READ a
// colour and the pipeline that stamps it as name-keyed scene metadata. Not a
// storage layer: see the module doc for the `{"color": "#RRGGBB"}` convention.
#[path = "io/appearance.rs"]
mod appearance;
pub use appearance::{BodyAppearance, ImportedColor, COLOR_METADATA_KEY};
#[path = "io/step_import/mod.rs"]
mod step_import;
pub use step_import::{
    import_step, import_step_bodies, import_step_report, import_step_trim_readings, import_step_with_appearance,
    occurrence_ref_parts, read_step_assembly, read_step_pmi, rewrite_occurrence_refs,
    ImportedStep, StepAssembly, StepBodyError, StepBodyTrimReadings, StepEdgeReading, StepFaceReading, StepImportReport,
    StepOccurrence, StepProduct, StepSuppliedTrim, StepTrimReadings, OCCURRENCE_REF_PREFIX,
    STATED_PRECISION_INCONSISTENCY_RATIO,
};
/// Test seams for the Part 21 text codec of the PMI writer / reader.
#[doc(hidden)]
pub fn io_step_text_for_tests(value: &str) -> String {
    step::pmi::step_text(value)
}
#[doc(hidden)]
pub fn io_step_decode_text_for_tests(value: &str) -> String {
    step_import::decode_step_text_for_tests(value)
}
#[path = "io/iges/mod.rs"]
mod iges;
pub use iges::{
    export_iges, export_iges_assembly, export_iges_bodies, import_iges, import_iges_report,
    read_iges_assembly, IgesBody, IgesImportReport,
};
#[path = "io/parasolid/mod.rs"]
pub mod parasolid;
#[path = "io/solidworks/mod.rs"]
pub mod solidworks;
#[path = "io/mesh_io.rs"]
mod mesh_io;
pub use mesh_io::{
    parse_stl_triangles, read_obj, read_stl, write_ascii_stl, write_binary_stl, write_obj,
    ObjReadResult, StlError, StlFormat, StlReadResult, StlTriangles,
};
// 3MF (3D Manufacturing Format) import, core specification only: the OPC/ZIP
// container, the model XML, and the build's instances flattened into one
// millimetre triangle soup for the SAME mesh-to-solid chain STL and OBJ use.
// The container, the deflate decoder and the XML reader are hand-rolled and
// bounded — see the module doc for the dependency review behind that.
#[path = "io/three_mf/mod.rs"]
mod three_mf;
pub use three_mf::{
    read_3mf, write_3mf, write_3mf_meshes, ThreeMfBeam, ThreeMfBeamLattice, ThreeMfAppearance, ThreeMfTexture, ThreeMfError, ThreeMfInstance, ThreeMfMesh, ThreeMfReadResult,
    ThreeMfSolid, ThreeMfUnit, ThreeMfWriteReport,
};
// Binary glTF 2.0 of the DISPLAY mesh — tessellated exchange for viewers, with
// the axis convention and the unit carried by the root node's transform and the
// colours converted from sRGB to glTF's linear `baseColorFactor`.
#[path = "io/glb.rs"]
mod glb;
pub use glb::{write_glb, GlbReport, GlbSolid, GlbUpAxis, GLB_MAGIC};
// Native serialized exact-BREP snapshot: the assemblies parts-library fast
// lane (instant component insert from a cached payload; fails detectably so
// the ACOMP self-heal lane can re-execute the embedded part document).
#[path = "io/snapshot.rs"]
mod snapshot;
pub use snapshot::{
    restore_solids, snapshot_resident_solids, snapshot_solids, RestoredSnapshot, RestoredSolid,
    SNAPSHOT_FORMAT_VERSION,
};


mod abi;
pub use abi::*;


/// The source hashes of the kernel's crates (each crate's `build.rs`); together they are the stamp the
/// parts library compares snapshots against, so a change in any kernel crate invalidates them.
pub(crate) static KERNEL_SOURCE_HASHES: &[Option<&str>] = &[option_env!("BREP_KERNEL_SOURCE_HASH"), brep_geometry::SOURCE_HASH, brep_topology::SOURCE_HASH, brep_surface_offset::SOURCE_HASH, brep_boolean::SOURCE_HASH, brep_construction::SOURCE_HASH, brep_blend::SOURCE_HASH, brep_offset::SOURCE_HASH, brep_direct_edit::SOURCE_HASH, brep_solvers::SOURCE_HASH];

/// One stamp over the crates' hashes; `None` when any crate could not compute its own.
pub(crate) fn combined_source_stamp(parts: &[Option<&str>]) -> Option<String> {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for part in parts {
        for byte in part.as_ref()?.bytes().chain([0]) {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
    Some(format!("{hash:016x}"))
}

/// Drop every reading memo (the history cache's clear, a document switch): the
/// topology memos and the boolean, heal and face-merge memos keyed on them.
pub(crate) fn clear_reading_memos() {
    brep_geometry::projection::clear_curve_seed_cache();
    crate::reading_memo::clear();
    crate::heal::clear_claim_memo();
    crate::face_merge::clear_decline_memo();
    crate::boolean::clear_candidate_memo();
    crate::imprint::clear_window_memo();
    crate::imprint::clear_touch_memo();
}


