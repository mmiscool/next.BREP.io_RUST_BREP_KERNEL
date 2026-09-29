//! Clean-room, topology-aware analytic surface recognition for CAD meshes.
//!
//! This crate is a pure geometry leaf. It does not parse or write STEP, call
//! subprocess oracles, download corpora, or depend on a BREP kernel.

mod debug_export;
mod design_intent;
mod error;
mod fit;
mod math;
mod mesh;
mod numerical;
mod options;
mod recognize;
mod surface;


pub use debug_export::{export_debug_obj, DebugObjExport};
pub use design_intent::{
    constrain_concentric_centers, constrain_cylinder_coaxial, constrain_equal_radii,
    constrain_plane_perpendicular_to_axis, constrain_planes_coplanar, constrain_planes_parallel,
    constrain_planes_perpendicular, constrain_shared_axis, constrain_tangent, DesignIntentError,
};
pub use error::RecognitionError;
pub use math::Vec3;
pub use mesh::{AnalyzedMesh, Mesh, MeshAnalysisOptions, SourceMetadata, TriangleData};
pub use options::{RecognitionOptions, SamplingMode};
pub use recognize::{
    recognize_surfaces, recognize_surfaces_with_unresolved, reconstruct_analyzed,
    reconstruct_surface, reconstruct_surface_from_vertices,
};
pub use surface::{
    AnalyticSurface, ConeSurface, ConstraintMask, CylinderSurface, FitDiagnostics, FitMetrics,
    FitPath, GeometricError, MetadataTrust, PhaseTimings, PlaneSurface, RecognitionResult,
    SphereSurface, SurfaceConstraints, SurfaceFitResult, SurfaceHint, SurfaceParameterDelta,
    SurfaceRegion, SurfaceType, TorusSurface, UnresolvedRegionDiagnostic,
};
