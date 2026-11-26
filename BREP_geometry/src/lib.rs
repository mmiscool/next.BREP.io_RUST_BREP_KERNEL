//! Geometry layer of the BREP kernel: vectors, tolerances, diagnostics, NURBS curves and surfaces, projection and intersection.
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
//! - vectors, tolerances, diagnostics, runtime switches and text helpers;
//! - exact rational NURBS curves and surfaces, curve and surface fitting, Coons and Gordon surfaces;
//! - analytic carrier recognition (plane, cylinder, cone, sphere, torus) and sphere charts;
//! - point projection onto curves and surfaces;
//! - curve/curve, curve/surface and surface/surface intersection, tangent curves and crossing nodes;
//! - bounding boxes and bounding-volume hierarchies, planar segment arrangements and polygons;
//! - p-curves and offset-point evaluation;
//! - small linear-algebra helpers, union-find, and the work-count and timing instruments the kernel reports.
//!
//! Kernel crates it requires: [`BREP_RANSAC`](https://crates.io/crates/BREP_RANSAC). It is required by [`BREP_topology`](https://crates.io/crates/BREP_topology), [`BREP_surface_offset`](https://crates.io/crates/BREP_surface_offset), [`BREP_boolean`](https://crates.io/crates/BREP_boolean), [`BREP_solvers`](https://crates.io/crates/BREP_solvers), [`BREP_construction`](https://crates.io/crates/BREP_construction), [`BREP_blend`](https://crates.io/crates/BREP_blend), [`BREP_offset`](https://crates.io/crates/BREP_offset), [`BREP_direct_edit`](https://crates.io/crates/BREP_direct_edit), [`BREP_kernel`](https://crates.io/crates/BREP_kernel).
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


/// One environment variable read through its call site's once-cell
/// ([`env_read::read`], lane O row 19): `std::env::var(name).ok()` as an
/// `Option<Cow<str>>`, kept for the process when `BREP_ENV_READ_ONCE` is on.
/// Only for variables nothing sets at run time.
#[macro_export]
macro_rules! env_read {
    ($name:literal) => {{
        static CELL: std::sync::OnceLock<Option<String>> = std::sync::OnceLock::new();
        $crate::env_read::read($name, &CELL)
    }};
}
/// One on/off switch (consolidation S1): `switch!("BREP_X", default)` reads
/// `BREP_X` through its call site's once-cell ([`env_read::switch`], kept for
/// the process while `BREP_ENV_READ_ONCE` is on); `switch!(live "BREP_X",
/// default)` reads it on every call, for a variable a probe sets at run time.
/// One grammar (owner decision 2026-11-01): `0|off|false` off, `1|on|true`
/// on, unset or anything else the default. In the kernel's tests one keyed
/// override (`env_read::with_switch_for_test`) forces either form.
#[macro_export]
macro_rules! switch {
    (live $name:expr, $default:expr) => {
        $crate::env_read::switch_live($name, $default, $crate::env_read::Grammar::Switch)
    };
    ($name:expr, $default:expr) => {{
        static CELL: std::sync::OnceLock<Option<bool>> = std::sync::OnceLock::new();
        $crate::env_read::switch($name, &CELL, $default, $crate::env_read::Grammar::Switch)
    }};
}

/// One debug, trace or `BREP_NO_*` flag (consolidation S1): on when its
/// variable is set to anything but `0`, `off` or `false`.
/// `debug_flag!(live "BREP_X")` reads it on every call.
#[macro_export]
macro_rules! debug_flag {
    (live $name:expr) => {
        $crate::env_read::switch_live($name, false, $crate::env_read::Grammar::Flag)
    };
    ($name:expr) => {{
        static CELL: std::sync::OnceLock<Option<bool>> = std::sync::OnceLock::new();
        $crate::env_read::switch($name, &CELL, false, $crate::env_read::Grammar::Flag)
    }};
}

/// One `0|off / check / on` mode (consolidation X1): an
/// [`env_read::Mode`]; unset, `On` when `default_on` and `Off` otherwise.
/// `mode_switch!(live "BREP_X", default_on)` reads it on every call.
#[macro_export]
macro_rules! mode_switch {
    (live $name:expr, $default_on:expr) => {
        $crate::env_read::mode_live($name, $default_on)
    };
    ($name:expr, $default_on:expr) => {{
        static CELL: std::sync::OnceLock<Option<$crate::env_read::Mode>> = std::sync::OnceLock::new();
        $crate::env_read::mode($name, &CELL, $default_on)
    }};
}

/// `eprintln!` for a `BREP_PROFILE` line: buffered while a
/// [`profile::ProfileSink`] is open (consolidation I8).
#[macro_export]
macro_rules! profile_line {
    ($($arg:tt)*) => { $crate::profile::profile_emit(format_args!($($arg)*)) };
}

#[doc(hidden)]

pub mod profile;
#[doc(hidden)]
pub mod claim_slots;
#[cfg(feature = "profile-wall")]
#[doc(hidden)]
pub mod wall_profile;
#[cfg(not(feature = "profile-wall"))]
#[path = "wall_profile_off.rs"]
#[doc(hidden)]
pub mod wall_profile;
#[cfg(feature = "profile-wall")]
#[doc(hidden)]
pub mod reader_profile;
#[cfg(not(feature = "profile-wall"))]
#[path = "reader_profile_off.rs"]
#[doc(hidden)]
pub mod reader_profile;


/// Switch reading for the kernel's dependants ([`switch!`],
/// [`debug_flag!`], [`mode_switch!`]); not a stable interface.
#[doc(hidden)]
#[doc(hidden)]
pub mod env_read;


/// The wasm panic hook every `#[wasm_bindgen]` entry point installs. Replaces
/// the `console_error_panic_hook` crate for the whole BREP family — reached as
/// `brep_kernel::panic_hook` and, through the render engine's re-export, as
/// `brep_render::brep_kernel::panic_hook`.
#[doc(hidden)]
pub mod panic_hook;

/// Plain-text formats shared by the CAD app and the PLM server
/// (`text::delimited`: CSV and tab-separated text).
#[doc(hidden)]
pub mod text;

#[path = "geometry/tolerance.rs"]
#[doc(hidden)]
pub mod tolerance;
pub use tolerance::{
    curve_model_scale, model_scale, report_scale_migration, SolidScale,
    offset_construction_band, vertex_tolerance_from_edges, KernelTolerances,
    MeasuredTolerance, OFFSET_CONSTRUCTION_FLOOR, OFFSET_CONSTRUCTION_REL,
    VERTEX_MATCH_FLOOR, VOLUME_DRIFT_ABS, VOLUME_DRIFT_REL,
};
#[path = "props/diagnostics.rs"]
#[doc(hidden)]
pub mod diagnostics;
pub use diagnostics::{
    Approximation, ApproximationBudget, Backing, BudgetMechanism, BudgetReason, MeasuredComponent, DiagnosticEvent, DiagnosticSeverity, EdgeOrigin, KernelDiagnostics,
    KernelOutcome, KernelRefusal, OffCarrierEdge,
    KernelStage, OrRefuse, RefusalClass, SoundnessDefect, UndecidedCause,
};
#[path = "geometry/polygon.rs"]
#[doc(hidden)]
pub mod polygon;
#[path = "geometry/curve.rs"]
#[doc(hidden)]
pub mod curve;
pub use curve::{
    make_arc, make_circle, make_hyperbola, make_line, make_parabola, uniform_clamped_knots,
    KnotVector, NurbsCurve, Vec4, KNOT_DEDUP_EPS, KNOT_IDENTITY_TOL,
};
#[path = "geometry/fit.rs"]
#[doc(hidden)]
pub mod fit;
pub use fit::{
    fit_polyline, fit_polyline_at, fit_polyline_of_degree, interpolate_curve, interpolate_curve_closed,
    interpolate_curve_local, interpolate_curve_thinned, interpolate_curve_with_end_tangents,
    simplify_polyline, simplify_polyline_indices, solve_banded, solve_dense, PolylineFit,
    PolylineFitExit, PolylineFitLedger, PolylineFitReport, PolylineFitScope, MAX_FIT_STATIONS,
};
#[path = "geometry/image_curve.rs"]
#[doc(hidden)]
pub mod image_curve;
pub use image_curve::{affine_image_curve, image_curve, image_curve_pair, ImageCurve, ImageCurveTier};
#[path = "geometry/surface.rs"]
#[doc(hidden)]
pub mod surface;
pub use surface::{
    carrier_preview_patch, make_cone_surface, make_cylinder_surface, make_extrusion, make_plane,
    make_revolution, make_revolution_one_span, make_sphere_surface, make_sphere_surface_framed, make_torus_surface,
    ExtendRefusal, KnotDirection, KnotRefinementRefusal, NurbsSurface, SurfaceSide, WeightSite,
    MAXIMUM_FOLD_ANGLE, MAXIMUM_GROWTH_RATIO,
};
#[path = "geometry/deform.rs"]
#[doc(hidden)]
pub mod deform;
pub use deform::{
    deform_surface, deform_surface_where, deform_surfaces_tied, DeformPick, PoleSupport, SurfaceTie, TieContinuity,
    TiedResult, TiedSurface, DeformRefusal, DeformResult, DeformSide, DeformSpec,
    EdgeHold,
};
#[path = "geometry/bezier_triangle.rs"]
#[doc(hidden)]
pub mod bezier_triangle;
pub use bezier_triangle::{BezierTriangle, BezierTriangleRefusal, TrianglePiece, TriangleSide};
#[path = "geometry/t_spline.rs"]
#[doc(hidden)]
pub mod t_spline;
pub use t_spline::{TAnchor, TMesh, TMeshSegment, TSpline, TSplineRefusal};
#[path = "geometry/coons.rs"]
#[doc(hidden)]
pub mod coons;
pub use coons::{
    coons_bilinear, coons_bilinear_reported, coons_with_ribbons, CompatWork, CoonsRefusal,
    CoonsReport, RibbonReport,
};
#[path = "geometry/gordon.rs"]
#[doc(hidden)]
pub mod gordon;
pub use gordon::{gordon_surface, gordon_surface_reported, GordonRefusal, GordonReport};
#[path = "geometry/n_sided.rs"]
#[doc(hidden)]
pub mod n_sided;
pub use n_sided::{n_sided_fill, n_sided_fill_centre_solved, n_sided_fill_shortening_reach, n_sided_fill_shortening_reach_spokes_off_sides, reentrant_corners as n_sided_reentrant_corners, NSidedRefusal, NSidedReport};
#[path = "geometry/regularity.rs"]
#[doc(hidden)]
pub mod regularity;
pub use regularity::{certify_regular, Irregularity, RegularityReport};
#[path = "geometry/analytic_surface.rs"]
#[doc(hidden)]
pub mod analytic_surface;
pub use analytic_surface::{
    circle_angle_to_parameter, intersect_analytic_pair, plane_ruled_section_arc,
    revolution_structure, AnalyticSurface, RevolutionFrame, RevolutionStructure,
};
pub use analytic_surface::{
    intersect_analytic_pair_with, ruled_gate,
    ruled_gate_census, RuledGate,
};
#[path = "geometry/sphere_chart.rs"]
#[doc(hidden)]
pub mod sphere_chart;
pub use sphere_chart::{
    canonicalize_points, chart_grid_divisions, chart_grid_parameters, corner_index,
    cube_edge_corner, cube_edge_divisions, cube_edge_index, cube_edge_parameters,
    cube_edge_parts, Chart, ChartSegment, ChartSide, SphereAtlas,
    SphericalRegion, CHART_COUNT, CUBE_CORNER_COUNT, CUBE_EDGE_COUNT,
};
#[path = "geometry/projection.rs"]
#[doc(hidden)]
pub mod projection;
#[path = "geometry/projection_profile.rs"]
#[doc(hidden)]
pub mod projection_profile;
#[doc(hidden)]
pub use projection_profile::report as projection_profile_report;
#[path = "geometry/affine_sphere.rs"]
#[doc(hidden)]
pub mod affine_sphere;
pub use projection::{
    project_point_to_curve, project_point_to_surface, project_point_to_surface_seeded,
    CurveProjection, SurfaceProjection,
};
#[doc(hidden)]
pub use projection::{
    project_point_to_surface_basin_census, project_point_to_surface_general_lanes, BasinCensus,
    GlobalStats,
};
#[path = "intersect/curve_surface_intersection.rs"]
#[doc(hidden)]
pub mod curve_surface_intersection;
pub use curve_surface_intersection::{intersect_curve_surface, CurveSurfaceIntersection};
#[path = "intersect/exact_line_surface.rs"]
#[doc(hidden)]
pub mod exact_line_surface;
#[path = "intersect/curve_clear.rs"]
#[doc(hidden)]
pub mod curve_clear;
#[path = "intersect/curve_curve_intersection.rs"]
#[doc(hidden)]
pub mod curve_curve_intersection;
pub use curve_curve_intersection::{intersect_curves, CurveCurveIntersection};
#[path = "intersect/certified_planar.rs"]
#[doc(hidden)]
pub mod certified_planar;
#[path = "intersect/surface_surface_intersection.rs"]
#[doc(hidden)]
pub mod surface_surface_intersection;
pub use surface_surface_intersection::{
    intersect_surfaces, intersect_surfaces_supplemental, SurfaceIntersectionCurve,
    SurfaceIntersectionOptions,
};
pub use surface_surface_intersection::TRANSVERSE_SEED_CROSS;
pub use surface_surface_intersection::{carriers_proved_apart, hull_sets_proved_apart, separation_prefilter_enabled, SpanHullSet};
#[path = "intersect/tangent_curve.rs"]
#[doc(hidden)]
pub mod tangent_curve;
#[path = "intersect/crossing_node.rs"]
#[doc(hidden)]
pub mod crossing_node;
pub use crossing_node::{crossing_node_split_enabled, split_at_crossing_nodes};
pub use tangent_curve::{
    classify_tangent_contact_at, find_tangent_contact, trace_tangent_curve, TangentContact,
    EXTENDED_RANK_FLOOR,
};
#[path = "geometry/spatial.rs"]
#[doc(hidden)]
pub mod spatial;
pub use spatial::{Aabb, Bvh};
#[path = "intersect/arrangement.rs"]
#[doc(hidden)]
pub mod arrangement;
pub use arrangement::{
    arrange_segments, arrange_segments_lens_apart, point_in_polygon, segment_intersection, ArrangementPiece, ArrangementRegion,
    CycleUse, LensApart, LensPair, Segment2, Vec2,
};
#[path = "geometry/pcurve.rs"]
#[doc(hidden)]
pub mod pcurve;
pub use pcurve::{
    build_pcurve_on_surface, build_pcurve_on_surface_marched, build_pcurve_on_surface_range,
    build_pcurve_on_surface_range_dense, build_pcurve_on_surface_stations, fit_pcurve_on_surface,
    fit_pcurve_on_surface_marched, fit_pcurve_on_surface_range, fit_pcurve_on_surface_range_dense,
    fit_pcurve_on_surface_stations, PcurveFit, PcurveFitExit, PcurveFitLedger, PcurveFitReport,
    PcurveFitScope, PCURVE_OFF_BAR, PCURVE_REFINEMENT_TOLERANCE,
};
// The pointwise offset evaluator — ONE definition of "the offset of a surface
// at a parameter", shared by `offset_surface`'s Greville sampling, the blend
// march's tangency residual, and the push-face offset residual gates. Declared
// before `offset` because that module is its first consumer.
#[path = "offset/point.rs"]
#[doc(hidden)]
pub mod offset_point;
pub use offset_point::{OffsetEvaluator, OffsetNormal, OffsetSample};
// The LOCUS-level offset seam — ONE definition of "intersect two offset
// analytic carriers in closed form", shared by the blend's corner closures.
// The pointwise evaluator above answers "where is the offset at this (u, v)";
// this answers "where do two offsets MEET", which is what the corner solves
// need and what no pointwise evaluator can produce (audit §9.4, §11).
// Crate-internal: no consumer outside the kernel names these.
#[path = "offset/analytic_pair.rs"]
#[doc(hidden)]
pub mod offset_analytic_pair;
#[path = "solvers/linear_algebra.rs"]
#[doc(hidden)]
pub mod solver_linear_algebra;

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Self = Self { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Self = Self { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Self = Self { x: 0.0, y: 0.0, z: 1.0 };

    /// Normalize using the standard vector guard, or return the caller's fallback.
    pub fn normalized_or(self, fallback: Self) -> Self {
        self.normalized().unwrap_or(fallback)
    }

    /// Rodrigues rotation about a unit axis through the origin.
    #[inline]
    pub fn rotate_about(self, axis: Self, angle: f64) -> Self {
        let (sin, cos) = angle.sin_cos();
        self.scale(cos)
            .add(axis.cross(self).scale(sin))
            .add(axis.scale(axis.dot(self) * (1.0 - cos)))
    }

    /// Distance to a segment. `guard` is the caller's squared-length threshold.
    #[inline]
    pub fn distance_to_segment(self, start: Self, end: Self, guard: f64) -> f64 {
        let segment = end.sub(start);
        let length_squared = segment.dot(segment);
        if length_squared <= guard {
            return self.sub(start).length();
        }
        let parameter = (self.sub(start).dot(segment) / length_squared).clamp(0.0, 1.0);
        self.sub(start.add(segment.scale(parameter))).length()
    }

    /// Squared distance, retaining the seed sweep's projection on zero-length segments.
    #[inline]
    pub fn distance_to_segment_squared(self, start: Self, end: Self) -> f64 {
        let segment = end.sub(start);
        let length_squared = segment.dot(segment);
        let parameter = if length_squared > 0.0 {
            (self.sub(start).dot(segment) / length_squared).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let residual = self.sub(start.add(segment.scale(parameter)));
        residual.dot(residual)
    }

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
    pub fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
    pub fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
    pub fn scale(self, factor: f64) -> Self {
        Self::new(self.x * factor, self.y * factor, self.z * factor)
    }
    pub fn cross(self, rhs: Self) -> Self {
        Self::new(
            self.y * rhs.z - self.z * rhs.y,
            self.z * rhs.x - self.x * rhs.z,
            self.x * rhs.y - self.y * rhs.x,
        )
    }
    pub fn dot(self, rhs: Self) -> f64 {
        self.x * rhs.x + self.y * rhs.y + self.z * rhs.z
    }
    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }
    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }
    pub fn normalized(self) -> Result<Self, String> {
        let length = self.length();
        if length <= 1e-12 {
            return Err("Vec3.normalized: zero-length vector".into());
        }
        Ok(self.scale(1.0 / length))
    }
    pub fn perpendicular(self) -> Result<Self, String> {
        let x = self.x.abs();
        let y = self.y.abs();
        let z = self.z.abs();
        let axis = if x <= y && x <= z {
            Self::new(1.0, 0.0, 0.0)
        } else if y <= z {
            Self::new(0.0, 1.0, 0.0)
        } else {
            Self::new(0.0, 0.0, 1.0)
        };
        self.cross(axis).normalized()
    }
}

#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct Mesh {
    pub positions: Vec<f64>,
    pub normals: Vec<f64>,
    pub indices: Vec<u32>,
    pub face_ids: Vec<u32>,
}

impl Mesh {
    fn push_vertex(&mut self, point: Vec3, normal: Vec3) -> u32 {
        self.positions.extend([point.x, point.y, point.z]);
        self.normals.extend([normal.x, normal.y, normal.z]);
        (self.positions.len() / 3 - 1) as u32
    }

    fn quad(&mut self, points: [Vec3; 4], normal: Vec3, face_id: u32) {
        let base = self.push_vertex(points[0], normal);
        for point in points.iter().skip(1) {
            self.push_vertex(*point, normal);
        }
        self.indices
            .extend([base, base + 1, base + 2, base, base + 2, base + 3]);
        self.face_ids.extend([face_id, face_id]);
    }

    pub fn signed_volume(&self) -> f64 {
        self.indices
            .chunks_exact(3)
            .map(|tri| {
                let point = |index: u32| {
                    let i = index as usize * 3;
                    Vec3::new(
                        self.positions[i],
                        self.positions[i + 1],
                        self.positions[i + 2],
                    )
                };
                point(tri[0]).dot(point(tri[1]).cross(point(tri[2]))) / 6.0
            })
            .sum()
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.positions.len() % 3 != 0 || self.normals.len() != self.positions.len() {
            return Err("invalid vertex or normal buffer".into());
        }
        self.validate_geometry()
    }

    /// Geometry-only validation for consumers that use positions + indices but
    /// NOT normals (e.g. `signed_volume`, whose tetrahedron sum never reads a
    /// normal). Mesh-only solids such as sheet-metal bodies supply metrics
    /// meshes with an empty normal buffer, which is legitimate for volume.
    pub fn validate_geometry(&self) -> Result<(), String> {
        if self.positions.len() % 3 != 0 {
            return Err("invalid vertex buffer".into());
        }
        if self.indices.len() % 3 != 0 {
            return Err("index buffer is not triangular".into());
        }
        let count = self.positions.len() / 3;
        if self.indices.iter().any(|index| *index as usize >= count) {
            return Err("index outside vertex buffer".into());
        }
        if !self.positions.iter().all(|value| value.is_finite()) {
            return Err("non-finite vertex".into());
        }
        Ok(())
    }
}

pub fn make_box(size_x: f64, size_y: f64, size_z: f64) -> Mesh {
    let x = size_x.abs();
    let y = size_y.abs();
    let z = size_z.abs();
    let p = [
        Vec3::new(0.0, 0.0, 0.0),
        Vec3::new(x, 0.0, 0.0),
        Vec3::new(x, y, 0.0),
        Vec3::new(0.0, y, 0.0),
        Vec3::new(0.0, 0.0, z),
        Vec3::new(x, 0.0, z),
        Vec3::new(x, y, z),
        Vec3::new(0.0, y, z),
    ];
    let mut mesh = Mesh::default();
    mesh.quad([p[0], p[3], p[2], p[1]], Vec3::new(0.0, 0.0, -1.0), 0);
    mesh.quad([p[4], p[5], p[6], p[7]], Vec3::new(0.0, 0.0, 1.0), 1);
    mesh.quad([p[0], p[1], p[5], p[4]], Vec3::new(0.0, -1.0, 0.0), 2);
    mesh.quad([p[3], p[7], p[6], p[2]], Vec3::new(0.0, 1.0, 0.0), 3);
    mesh.quad([p[0], p[4], p[7], p[3]], Vec3::new(-1.0, 0.0, 0.0), 4);
    mesh.quad([p[1], p[2], p[6], p[5]], Vec3::new(1.0, 0.0, 0.0), 5);
    mesh
}

pub fn make_cylinder(radius: f64, height: f64, segments: usize) -> Mesh {
    let radius = radius.abs();
    let height = height.abs();
    let segments = segments.max(3);
    let mut mesh = Mesh::default();
    for i in 0..segments {
        let a = std::f64::consts::TAU * i as f64 / segments as f64;
        let b = std::f64::consts::TAU * (i + 1) as f64 / segments as f64;
        let (sa, ca) = a.sin_cos();
        let (sb, cb) = b.sin_cos();
        let pa = Vec3::new(radius * ca, 0.0, -radius * sa);
        let pb = Vec3::new(radius * cb, 0.0, -radius * sb);
        let ta = Vec3::new(pa.x, height, pa.z);
        let tb = Vec3::new(pb.x, height, pb.z);
        mesh.quad(
            [pa, pb, tb, ta],
            Vec3::new((ca + cb) * 0.5, 0.0, -(sa + sb) * 0.5),
            0,
        );
        let bottom = mesh.push_vertex(Vec3::default(), Vec3::new(0.0, -1.0, 0.0));
        let bi = mesh.push_vertex(pb, Vec3::new(0.0, -1.0, 0.0));
        let bj = mesh.push_vertex(pa, Vec3::new(0.0, -1.0, 0.0));
        mesh.indices.extend([bottom, bi, bj]);
        mesh.face_ids.push(1);
        let top = mesh.push_vertex(Vec3::new(0.0, height, 0.0), Vec3::new(0.0, 1.0, 0.0));
        let ti = mesh.push_vertex(ta, Vec3::new(0.0, 1.0, 0.0));
        let tj = mesh.push_vertex(tb, Vec3::new(0.0, 1.0, 0.0));
        mesh.indices.extend([top, ti, tj]);
        mesh.face_ids.push(2);
    }
    mesh
}

#[path = "healing/curve_helpers.rs"]
#[doc(hidden)]
pub mod curve_helpers;

#[path = "geometry/eigen2.rs"]
#[doc(hidden)]
pub mod eigen2;

#[path = "geometry/union_find.rs"]
#[doc(hidden)]
pub mod union_find;
pub use union_find::UnionFind;

#[path = "geometry/poly.rs"]
#[doc(hidden)]
pub mod poly;

pub use analytic_surface::{circumcenter, circumcenter_2d, recognize_circle, RecognizedCircle};

#[path = "geometry/triangle.rs"]
#[doc(hidden)]
pub mod triangle;
pub use triangle::closest_point_on_triangle;

pub use tolerance::RIGID_TOL;

impl std::ops::Index<usize> for Vec3 {
    type Output = f64;
    fn index(&self, index: usize) -> &Self::Output {
        match index { 0 => &self.x, 1 => &self.y, 2 => &self.z, _ => panic!("Vec3 index out of bounds: {index}") }
    }
}
impl std::ops::IndexMut<usize> for Vec3 {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        match index { 0 => &mut self.x, 1 => &mut self.y, 2 => &mut self.z, _ => panic!("Vec3 index out of bounds: {index}") }
    }
}

/// Hash of this crate's source (`build.rs`), one part of the kernel's source stamp.
#[doc(hidden)]
pub const SOURCE_HASH: Option<&str> = option_env!("BREP_KERNEL_SOURCE_HASH");
#[doc(hidden)]
#[path = "instrument/work_counts.rs"]
pub mod work_counts;
#[doc(hidden)]
#[path = "instrument/reuse_profile.rs"]
pub mod reuse_profile;
#[doc(hidden)]
#[path = "instrument/carried_uv.rs"]
pub mod carried_uv;
#[doc(hidden)]
#[path = "instrument/blend_timing.rs"]
pub mod blend_timing;
#[doc(hidden)]
#[path = "geometry/identity_hash.rs"]
pub mod identity_hash;
#[doc(hidden)]
#[path = "geometry/trimmed_curve.rs"]
pub mod trimmed_curve;
