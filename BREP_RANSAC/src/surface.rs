use crate::numerical::scalar;
use crate::Vec3;
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize)]
/// The supported analytic primitive families.
pub enum SurfaceType {
    /// A plane.
    Plane,
    /// A sphere.
    Sphere,
    /// A circular cylinder.
    Cylinder,
    /// A right circular cone.
    Cone,
    /// A circular torus.
    Torus,
}

impl SurfaceType {
    /// Returns the lowercase human-readable primitive name.
    pub fn name(self) -> &'static str {
        match self {
            Self::Plane => "plane",
            Self::Sphere => "sphere",
            Self::Cylinder => "cylinder",
            Self::Cone => "cone",
            Self::Torus => "torus",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// A plane represented by a point and unit normal.
pub struct PlaneSurface {
    /// A point on the plane.
    pub origin: Vec3,
    /// The plane's unit normal.
    pub normal: Vec3,
}
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// A sphere represented by its center and radius.
pub struct SphereSurface {
    /// The sphere center.
    pub center: Vec3,
    /// The positive sphere radius.
    pub radius: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// An infinite circular cylinder.
pub struct CylinderSurface {
    /// A point on the cylinder axis.
    pub axis_origin: Vec3,
    /// The unit direction of the cylinder axis.
    pub axis: Vec3,
    /// The positive cylinder radius.
    pub radius: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// One nappe of a right circular cone.
pub struct ConeSurface {
    /// The cone apex.
    pub apex: Vec3,
    /// Unit axis directed into the represented nappe.
    pub axis: Vec3,
    /// Angle between the axis and surface generators, in radians.
    pub half_angle: f64,
}
#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
/// A circular torus represented by its center, axis, and radii.
pub struct TorusSurface {
    /// Center of the torus's generating circle.
    pub center: Vec3,
    /// Unit normal of the generating circle's plane.
    pub axis: Vec3,
    /// Radius from `center` to the centerline of the tube.
    pub major_radius: f64,
    /// Radius of the torus tube.
    pub minor_radius: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Deserialize, Serialize)]
#[serde(tag = "type", content = "parameters")]
/// Parameters for one supported analytic surface.
pub enum AnalyticSurface {
    /// A planar surface.
    Plane(PlaneSurface),
    /// A spherical surface.
    Sphere(SphereSurface),
    /// A cylindrical surface.
    Cylinder(CylinderSurface),
    /// A conical surface.
    Cone(ConeSurface),
    /// A toroidal surface.
    Torus(TorusSurface),
}

impl AnalyticSurface {
    /// Returns this surface's primitive family.
    pub fn surface_type(self) -> SurfaceType {
        match self {
            Self::Plane(_) => SurfaceType::Plane,
            Self::Sphere(_) => SurfaceType::Sphere,
            Self::Cylinder(_) => SurfaceType::Cylinder,
            Self::Cone(_) => SurfaceType::Cone,
            Self::Torus(_) => SurfaceType::Torus,
        }
    }
    /// Evaluates the surface's signed implicit distance-like residual at `point`.
    ///
    /// For normalized valid parameters, its magnitude is the geometric distance
    /// for planes, spheres, cylinders, and tori; the cone expression is its
    /// signed meridian distance.
    pub fn signed_distance(self, point: Vec3) -> f64 {
        match self {
            Self::Plane(s) => (point - s.origin).dot(s.normal),
            Self::Sphere(s) => (point - s.center).length() - s.radius,
            Self::Cylinder(s) => {
                let q = point - s.axis_origin;
                (q - s.axis * q.dot(s.axis)).length() - s.radius
            }
            Self::Cone(s) => {
                let q = point - s.apex;
                let h = q.dot(s.axis);
                let radial = (q - s.axis * h).length();
                radial * s.half_angle.cos() - h * s.half_angle.sin()
            }
            Self::Torus(s) => {
                let q = point - s.center;
                let z = q.dot(s.axis);
                let rho = (q - s.axis * z).length();
                ((rho - s.major_radius).powi(2) + z * z).sqrt() - s.minor_radius
            }
        }
    }
    /// Returns the analytic unit normal at `point`, if it is defined there.
    pub fn normal_at(self, point: Vec3) -> Option<Vec3> {
        match self {
            Self::Plane(s) => Some(s.normal),
            Self::Sphere(s) => (point - s.center).normalized(),
            Self::Cylinder(s) => {
                let q = point - s.axis_origin;
                (q - s.axis * q.dot(s.axis)).normalized()
            }
            Self::Cone(s) => {
                let q = point - s.apex;
                let h = q.dot(s.axis);
                let radial = (q - s.axis * h).normalized()?;
                (radial * s.half_angle.cos() - s.axis * s.half_angle.sin()).normalized()
            }
            Self::Torus(s) => {
                let q = point - s.center;
                let z = q.dot(s.axis);
                let radial = q - s.axis * z;
                let rho = radial.length();
                if rho <= scalar::MIN_NORMALIZABLE_NORM {
                    return None;
                }
                let tube = radial * (1.0 - s.major_radius / rho) + s.axis * z;
                tube.normalized()
            }
        }
    }
    /// Returns whether all carrier parameters are finite and geometrically valid.
    pub fn is_valid(self) -> bool {
        match self {
            Self::Plane(s) => s.origin.is_finite() && unit(s.normal),
            Self::Sphere(s) => s.center.is_finite() && s.radius.is_finite() && s.radius > 0.0,
            Self::Cylinder(s) => {
                s.axis_origin.is_finite() && unit(s.axis) && s.radius.is_finite() && s.radius > 0.0
            }
            Self::Cone(s) => {
                s.apex.is_finite()
                    && unit(s.axis)
                    && s.half_angle.is_finite()
                    && s.half_angle > 0.0
                    && s.half_angle < std::f64::consts::FRAC_PI_2
            }
            Self::Torus(s) => {
                s.center.is_finite()
                    && unit(s.axis)
                    && s.major_radius.is_finite()
                    && s.minor_radius.is_finite()
                    && s.major_radius > 0.0
                    && s.minor_radius > 0.0
            }
        }
    }
    /// Choose deterministic orientation/gauge conventions without changing
    /// the represented infinite surface.
    pub fn canonicalized(self, centroid: Vec3) -> Self {
        match self {
            Self::Plane(mut s) => {
                s.normal = s.normal.normalized().unwrap_or(s.normal).canonicalized();
                s.origin = centroid + s.normal * (s.origin - centroid).dot(s.normal);
                Self::Plane(s)
            }
            Self::Sphere(s) => Self::Sphere(s),
            Self::Cylinder(mut s) => {
                s.axis = s.axis.normalized().unwrap_or(s.axis).canonicalized();
                s.axis_origin += s.axis * (centroid - s.axis_origin).dot(s.axis);
                Self::Cylinder(s)
            }
            Self::Cone(mut s) => {
                s.axis = s.axis.normalized().unwrap_or(s.axis);
                Self::Cone(s)
            }
            Self::Torus(mut s) => {
                s.axis = s.axis.normalized().unwrap_or(s.axis).canonicalized();
                Self::Torus(s)
            }
        }
    }
}
fn unit(v: Vec3) -> bool {
    v.is_finite() && (v.length() - 1.0).abs() <= scalar::UNIT_LENGTH_TOLERANCE
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
/// Describes how strongly imported metadata should influence fitting.
pub enum MetadataTrust {
    /// Parameters are asserted to describe the exact source carrier.
    Exact,
    /// Parameters are a strong prior but must still be validated.
    StrongHint,
    /// Parameters only initialize numerical refinement.
    InitialGuess,
    /// Only the primitive family is trusted.
    TypeOnly,
    /// No trust information was supplied.
    Unknown,
}

impl Default for MetadataTrust {
    fn default() -> Self {
        Self::Unknown
    }
}

/// Parameter-level constraints. Vector fields constrain all three components;
/// axis origins/centers still use the conventional along-axis gauge freedom.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize)]
pub struct ConstraintMask {
    /// Fix the plane origin, sphere/torus center, cylinder axis origin, or cone apex.
    pub origin_or_center: bool,
    /// Fix the axis or normal direction.
    pub axis_or_normal: bool,
    /// Fix the ordinary radius or torus minor radius.
    pub radius: bool,
    /// Fix the torus major radius.
    pub major_radius: bool,
    /// Fix the cone half-angle.
    pub angle: bool,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// Initial surface parameters and the subset held fixed during fitting.
pub struct SurfaceConstraints {
    /// Initial carrier parameters, when numeric values are available.
    pub initial: Option<AnalyticSurface>,
    /// Parameter groups that refinement must preserve.
    pub fixed: ConstraintMask,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// Optional source knowledge used to guide surface recognition.
pub enum SurfaceHint {
    /// No prior surface information is available.
    Unknown,
    /// The primitive family is known but its parameters are not.
    KnownType {
        /// The known primitive family.
        surface_type: SurfaceType,
    },
    /// Numeric parameters are available as a refinement starting point.
    InitialGuess {
        /// Initial carrier parameters.
        surface: AnalyticSurface,
        /// Confidence assigned to the supplied parameters.
        trust: MetadataTrust,
    },
    /// The family is known and selected parameter groups may be fixed.
    Constrained {
        /// The required primitive family.
        surface_type: SurfaceType,
        /// Initial values and fixed-parameter mask.
        constraints: SurfaceConstraints,
        /// Confidence assigned to the supplied constraints.
        trust: MetadataTrust,
    },
    /// A complete carrier that may be reused after validation.
    ExactCandidate {
        /// The proposed exact carrier.
        surface: AnalyticSurface,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
/// The recognition or refinement route that produced a fit.
pub enum FitPath {
    /// The primitive family and parameters were recognized without metadata.
    GenericRecognition,
    /// Parameters were fitted for a metadata-specified primitive family.
    KnownTypeFit,
    /// Supplied hint parameters were accepted without refinement.
    HintReused,
    /// A supplied exact candidate was accepted without refinement.
    ExactCandidateReused,
    /// Refinement honored one or more fixed parameter groups.
    ConstrainedRefinement,
    /// Supplied parameters initialized unconstrained refinement.
    UnconstrainedRefinement,
    /// A rejected hint was followed by generic recognition.
    HintRejectedFallback,
}

#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
/// Residual and support measurements for a fitted surface.
pub struct FitMetrics {
    /// Root-mean-square positional residual.
    pub rms_error: f64,
    /// Maximum absolute positional residual.
    pub max_error: f64,
    /// Root-mean-square normal-angle residual, in radians.
    pub rms_normal_error: f64,
    /// Maximum normal-angle residual, in radians.
    pub max_normal_error: f64,
    /// Number of supporting triangles.
    pub support_triangles: usize,
    /// Total area of supporting triangles.
    pub supported_area: f64,
}

/// Residual-only snapshot used to compare a supplied/initialized carrier with
/// the final refined carrier without duplicating support bookkeeping.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize, Serialize)]
pub struct GeometricError {
    /// Root-mean-square positional residual.
    pub rms_position: f64,
    /// Maximum absolute positional residual.
    pub max_position: f64,
    /// Root-mean-square normal-angle residual, in radians.
    pub rms_normal_radians: f64,
    /// Maximum normal-angle residual, in radians.
    pub max_normal_radians: f64,
}

impl From<&FitMetrics> for GeometricError {
    fn from(metrics: &FitMetrics) -> Self {
        Self {
            rms_position: metrics.rms_error,
            max_position: metrics.max_error,
            rms_normal_radians: metrics.rms_normal_error,
            max_normal_radians: metrics.max_normal_error,
        }
    }
}

/// Gauge-aware change from an initial/supplied carrier to its final carrier.
/// Fields that do not apply to a primitive remain `None`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct SurfaceParameterDelta {
    /// Gauge-aware displacement of an origin, center, axis line, or apex.
    pub origin_or_center: Option<f64>,
    /// Change in axis or normal direction, in radians.
    pub axis_or_normal_radians: Option<f64>,
    /// Change in the primitive radius, or torus minor radius.
    pub radius: Option<f64>,
    /// Change in torus major radius.
    pub major_radius: Option<f64>,
    /// Change in torus minor radius.
    pub minor_radius: Option<f64>,
    /// Change in cone half-angle, in radians.
    pub angle_radians: Option<f64>,
}

impl SurfaceParameterDelta {
    /// Compare compatible carrier parameterizations. Returns `None` when the
    /// primitive types differ. Axis/normal angles respect each carrier's gauge:
    /// cone axes are directed, while plane, cylinder, and torus directions are
    /// sign-equivalent.
    pub fn between(initial: AnalyticSurface, refined: AnalyticSurface) -> Option<Self> {
        fn angle(a: Vec3, b: Vec3, oriented: bool) -> f64 {
            let dot = a.dot(b).clamp(-1.0, 1.0);
            (if oriented { dot } else { dot.abs() }).acos()
        }
        fn line_distance(p: Vec3, a: Vec3, q: Vec3, b: Vec3) -> f64 {
            let cross = a.cross(b);
            let length = cross.length();
            if length > scalar::PARALLEL_LINE_CROSS_NORM {
                (q - p).dot(cross).abs() / length
            } else {
                let delta = q - p;
                (delta - a * delta.dot(a)).length()
            }
        }
        Some(match (initial, refined) {
            (AnalyticSurface::Plane(a), AnalyticSurface::Plane(b)) => Self {
                origin_or_center: Some((b.origin - a.origin).dot(a.normal).abs()),
                axis_or_normal_radians: Some(angle(a.normal, b.normal, false)),
                ..Default::default()
            },
            (AnalyticSurface::Sphere(a), AnalyticSurface::Sphere(b)) => Self {
                origin_or_center: Some(a.center.distance(b.center)),
                radius: Some((a.radius - b.radius).abs()),
                ..Default::default()
            },
            (AnalyticSurface::Cylinder(a), AnalyticSurface::Cylinder(b)) => Self {
                origin_or_center: Some(line_distance(a.axis_origin, a.axis, b.axis_origin, b.axis)),
                axis_or_normal_radians: Some(angle(a.axis, b.axis, false)),
                radius: Some((a.radius - b.radius).abs()),
                ..Default::default()
            },
            (AnalyticSurface::Cone(a), AnalyticSurface::Cone(b)) => Self {
                origin_or_center: Some(a.apex.distance(b.apex)),
                axis_or_normal_radians: Some(angle(a.axis, b.axis, true)),
                angle_radians: Some((a.half_angle - b.half_angle).abs()),
                ..Default::default()
            },
            (AnalyticSurface::Torus(a), AnalyticSurface::Torus(b)) => Self {
                origin_or_center: Some(a.center.distance(b.center)),
                axis_or_normal_radians: Some(angle(a.axis, b.axis, false)),
                radius: Some((a.minor_radius - b.minor_radius).abs()),
                major_radius: Some((a.major_radius - b.major_radius).abs()),
                minor_radius: Some((a.minor_radius - b.minor_radius).abs()),
                ..Default::default()
            },
            _ => return None,
        })
    }
}

/// Optional wall-clock phase measurements. Timing is disabled by default so
/// deterministic results remain bit-for-bit comparable.
#[derive(Clone, Copy, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(default)]
pub struct PhaseTimings {
    /// Time spent inspecting and validating metadata.
    pub metadata_inspection_seconds: Option<f64>,
    /// Time spent generating candidate carriers.
    pub candidate_generation_seconds: Option<f64>,
    /// Time spent evaluating candidate residuals and support.
    pub candidate_evaluation_seconds: Option<f64>,
    /// Time spent discovering or growing connected regions.
    pub region_growth_seconds: Option<f64>,
    /// Time spent numerically refining carrier parameters.
    pub refinement_seconds: Option<f64>,
    /// Time spent validating the final carrier.
    pub validation_seconds: Option<f64>,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// Provenance and execution details for a fitted surface.
pub struct FitDiagnostics {
    /// The recognition or refinement path used.
    pub path: FitPath,
    /// Carrier supplied by metadata, when one was available.
    pub supplied_surface: Option<AnalyticSurface>,
    /// Supplied parameter groups held fixed during refinement.
    pub fixed_parameters: ConstraintMask,
    /// Whether numerical refinement changed carrier parameters.
    pub parameters_refined: bool,
    /// Whether metadata made generic primitive classification unnecessary.
    pub generic_classification_skipped: bool,
    /// Whether exact supplied parameters were returned unchanged.
    pub exact_parameters_reused: bool,
    /// Number of hypotheses generated during recognition.
    pub hypotheses_generated: usize,
    /// Number of candidate carriers evaluated.
    pub candidates_evaluated: usize,
    /// Human-readable summary of why this path or result was selected.
    pub reason: String,
    /// Primitive competitors rejected during selection and their reasons.
    pub rejected_competitors: Vec<(SurfaceType, String)>,
    /// Trust attached to the selected input path (`Unknown` and `TypeOnly` are
    /// recorded explicitly even though they carry no numeric prior).
    #[serde(default)]
    pub metadata_trust: MetadataTrust,
    /// Geometric residual before refinement when an initial carrier exists.
    #[serde(default)]
    pub initial_error: Option<GeometricError>,
    /// Geometric residual of the returned carrier.
    #[serde(default)]
    pub refined_error: Option<GeometricError>,
    /// Gauge-aware parameter change from initial/supplied to returned carrier.
    #[serde(default)]
    pub parameter_delta: Option<SurfaceParameterDelta>,
    #[serde(default)]
    /// Optional wall-clock timing measurements by recognition phase.
    pub phase_timings: PhaseTimings,
}

impl Default for FitDiagnostics {
    fn default() -> Self {
        Self {
            path: FitPath::GenericRecognition,
            supplied_surface: None,
            fixed_parameters: ConstraintMask::default(),
            parameters_refined: false,
            generic_classification_skipped: false,
            exact_parameters_reused: false,
            hypotheses_generated: 0,
            candidates_evaluated: 0,
            reason: String::new(),
            rejected_competitors: Vec::new(),
            metadata_trust: MetadataTrust::Unknown,
            initial_error: None,
            refined_error: None,
            parameter_delta: None,
            phase_timings: PhaseTimings::default(),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// The fitted carrier and quality information for one requested selection.
pub struct SurfaceFitResult {
    /// The fitted analytic carrier.
    pub surface: AnalyticSurface,
    /// Carrier orientation relative to mesh winding, as `-1` or `1`.
    pub orientation: i8,
    /// Residual and support measurements for the fit.
    pub metrics: FitMetrics,
    /// Normalized evidence score for the returned fit.
    pub confidence: f64,
    /// Provenance and execution details for the fit.
    pub diagnostics: FitDiagnostics,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// A recognized surface and the mesh triangles assigned to it.
pub struct SurfaceRegion {
    /// The region's analytic carrier.
    pub surface: AnalyticSurface,
    /// Carrier orientation relative to mesh winding, as `-1` or `1`.
    pub orientation: i8,
    /// Indices of triangles assigned to this region.
    pub triangle_indices: Vec<usize>,
    /// Residual and support measurements for the region.
    pub metrics: FitMetrics,
    /// Normalized evidence score for the region's fit.
    pub confidence: f64,
    /// Provenance and execution details for the region's fit.
    pub diagnostics: FitDiagnostics,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// Why a metadata-associated triangle subset remained unresolved after both
/// metadata-guided reconstruction and generic analytic fallback failed.
pub struct UnresolvedRegionDiagnostic {
    /// Still-unresolved triangles from the metadata subset.
    pub triangle_indices: Vec<usize>,
    /// Stable source-system face identifier, when supplied.
    pub source_face_id: Option<u64>,
    /// Human-readable source-system face name, when supplied.
    pub source_face_name: Option<String>,
    /// Stable source-system surface identifier, when supplied.
    pub source_surface_id: Option<String>,
    /// Explicit metadata-validation or reconstruction failure followed by the
    /// fact that generic analytic extraction left these triangles unresolved.
    pub reason: String,
}

#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
/// All recognized regions and triangles left unresolved by discovery.
pub struct RecognitionResult {
    /// Successfully recognized surface regions.
    pub regions: Vec<SurfaceRegion>,
    /// Input triangle indices not assigned to any recognized region.
    pub unresolved_triangles: Vec<usize>,
    /// Failure provenance for unresolved subsets that carried source metadata.
    #[serde(default)]
    pub unresolved_diagnostics: Vec<UnresolvedRegionDiagnostic>,
}
