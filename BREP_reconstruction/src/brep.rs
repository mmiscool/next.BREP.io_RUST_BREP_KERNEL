//! Optional interoperability with the public `brep_kernel` API.
//!
//! This module contains representation conversion only. Recognition remains
//! independent of kernel topology, boolean execution, and global scene state.

use crate::numerical;
use crate::{AnalyticSurface, Mesh, RecognitionError, SourceMetadata, SurfaceHint, Vec3};

/// Exact analytic information carried by one kernel face.
///
/// `surface` describes the unoriented infinite mathematical carrier. Face
/// orientation is deliberately retained in `orientation` instead of being
/// folded into an axis or normal. It combines `FaceRecord::same_sense` with
/// any parameter-normal gauge removed while canonicalizing the carrier, and
/// prevents a reversed STEP `ADVANCED_FACE` from changing its geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct KernelFaceTruth {
    /// Sequential face number used by the kernel tessellator's `face_ids`.
    pub mesh_face_id: u32,
    /// Stable topology id from `FaceRecord::id`.
    pub source_face_id: u64,
    /// Optional stable source face name.
    pub source_face_name: Option<String>,
    /// Observed face orientation in the gauge of `surface`.
    ///
    /// This is normally `+1` for `same_sense` and `-1` otherwise. A kernel
    /// ruled revolution with negative generatrix height has the opposite
    /// parametric normal from our canonical cylinder/cone, so its sign is
    /// reversed during carrier conversion.
    pub orientation: i8,
    /// Exact infinite carrier extracted from the kernel face.
    pub surface: AnalyticSurface,
}

fn kernel_carrier_orientation_gauge(source: &brep_kernel::AnalyticSurface) -> i8 {
    match source {
        // For S(theta, t), dS/dtheta x dS/dt reverses when the axial
        // generatrix height reverses. Our cylinder/cone carrier deliberately
        // canonicalizes that parameterization away, so retain its normal
        // gauge here as a separate orientation sign.
        brep_kernel::AnalyticSurface::RuledRevolution { height, .. } if *height < 0.0 => -1,
        _ => 1,
    }
}

fn kernel_face_orientation(source: &brep_kernel::AnalyticSurface, same_sense: bool) -> i8 {
    (if same_sense { 1 } else { -1 }) * kernel_carrier_orientation_gauge(source)
}

fn invalid_kernel_analytic(reason: impl Into<String>) -> RecognitionError {
    RecognitionError::FitFailed {
        surface: None,
        reason: reason.into(),
    }
}

/// Convert an exact kernel analytic carrier into this crate's carrier model.
///
/// The kernel's general `Revolution` variant is intentionally returned as
/// `None`: it is not necessarily one of the five primitives represented by
/// [`AnalyticSurface`]. Invalid or degenerate primitive parameters are errors,
/// so corrupt source truth cannot silently become an exact RANSAC prior.
pub fn surface_from_kernel_analytic(
    source: &brep_kernel::AnalyticSurface,
) -> Result<Option<AnalyticSurface>, RecognitionError> {
    use brep_kernel::AnalyticSurface as KernelSurface;

    let converted = match source {
        KernelSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        } => {
            let normal = vec3_from_kernel(*u_dir)
                .cross(vec3_from_kernel(*v_dir))
                .normalized()
                .ok_or_else(|| invalid_kernel_analytic("kernel plane has degenerate directions"))?;
            AnalyticSurface::Plane(crate::PlaneSurface {
                origin: vec3_from_kernel(*origin),
                normal,
            })
        }
        KernelSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } => {
            if !rho0.is_finite()
                || !rho1.is_finite()
                || !height.is_finite()
                || *rho0 < 0.0
                || *rho1 < 0.0
            {
                return Err(invalid_kernel_analytic(
                    "kernel ruled revolution has invalid radius or height",
                ));
            }
            let axis = vec3_from_kernel(frame.axis)
                .normalized()
                .ok_or_else(|| invalid_kernel_analytic("kernel revolution axis is degenerate"))?;
            let origin = vec3_from_kernel(frame.origin);
            let radius_scale = rho0.abs().max(rho1.abs()).max(1.0);
            if (*rho1 - *rho0).abs()
                <= numerical::brep::REVOLUTION_EQUAL_RADIUS_RELATIVE * radius_scale
            {
                if *rho0 <= 0.0 || height.abs() <= numerical::brep::REVOLUTION_MIN_ABSOLUTE_HEIGHT {
                    return Err(invalid_kernel_analytic(
                        "kernel cylinder has non-positive radius or zero height",
                    ));
                }
                AnalyticSurface::Cylinder(crate::CylinderSurface {
                    axis_origin: origin,
                    axis,
                    radius: *rho0,
                })
            } else {
                if height.abs() <= numerical::brep::REVOLUTION_MIN_ABSOLUTE_HEIGHT {
                    return Err(invalid_kernel_analytic("kernel cone has zero height"));
                }
                // rho(z) = rho0 + slope*z. Our cone axis points from the apex
                // into the represented nappe, hence the sign(slope) adjustment.
                let slope = (*rho1 - *rho0) / *height;
                if !slope.is_finite() || slope == 0.0 {
                    return Err(invalid_kernel_analytic("kernel cone has invalid slope"));
                }
                let apex_z = -*rho0 / slope;
                AnalyticSurface::Cone(crate::ConeSurface {
                    apex: origin + axis * apex_z,
                    axis: axis * slope.signum(),
                    half_angle: slope.abs().atan(),
                })
            }
        }
        KernelSurface::Sphere { frame, radius } => {
            let _axis = vec3_from_kernel(frame.axis)
                .normalized()
                .ok_or_else(|| invalid_kernel_analytic("kernel sphere frame is degenerate"))?;
            AnalyticSurface::Sphere(crate::SphereSurface {
                center: vec3_from_kernel(frame.origin),
                radius: *radius,
            })
        }
        KernelSurface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => AnalyticSurface::Torus(crate::TorusSurface {
            center: vec3_from_kernel(frame.origin),
            axis: vec3_from_kernel(frame.axis)
                .normalized()
                .ok_or_else(|| invalid_kernel_analytic("kernel torus axis is degenerate"))?,
            major_radius: *major_radius,
            minor_radius: *minor_radius,
        }),
        KernelSurface::Revolution { .. } => return Ok(None),
    };

    if !converted.is_valid() {
        return Err(invalid_kernel_analytic(
            "kernel analytic carrier has invalid primitive parameters",
        ));
    }
    Ok(Some(converted))
}

/// Extract exact primitive truth from one face while preserving face sense as
/// separate metadata.
pub fn analytic_truth_from_face(
    face: &brep_kernel::FaceRecord,
    mesh_face_id: u32,
) -> Result<Option<KernelFaceTruth>, RecognitionError> {
    let Some(source) = face.surface.analytic() else {
        return Ok(None);
    };
    let Some(surface) = surface_from_kernel_analytic(source)? else {
        return Ok(None);
    };
    Ok(Some(KernelFaceTruth {
        mesh_face_id,
        source_face_id: face.id,
        source_face_name: face.name.clone(),
        orientation: kernel_face_orientation(source, face.same_sense),
        surface,
    }))
}

/// Enumerate exact primitive truth in the same shell/face order used by
/// `tessellate_brep_watertight` to assign sequential `face_ids`.
pub fn analytic_truths_from_solid(
    solid: &brep_kernel::BrepSolid,
) -> Result<Vec<KernelFaceTruth>, RecognitionError> {
    let mut truths = Vec::new();
    let mut mesh_face_id = 0u32;
    for shell in &solid.shells {
        for face in &shell.faces {
            if let Some(truth) = analytic_truth_from_face(face, mesh_face_id)? {
                truths.push(truth);
            }
            mesh_face_id = mesh_face_id.checked_add(1).ok_or_else(|| {
                RecognitionError::InvalidMesh("kernel solid has more than u32::MAX faces".into())
            })?;
        }
    }
    Ok(truths)
}

/// Populate exact source metadata for every tessellated face whose kernel
/// carrier is one of the five supported primitives.
///
/// Returns the number of metadata records added. Analytic faces that produced
/// no triangles (for example a degenerate trim) are skipped.
pub fn attach_solid_analytic_metadata(
    converted: &mut KernelMeshConversion,
    solid: &brep_kernel::BrepSolid,
    source_tolerance: Option<f64>,
) -> Result<usize, RecognitionError> {
    if source_tolerance.is_some_and(|value| !value.is_finite() || value <= 0.0) {
        return Err(RecognitionError::InvalidSelection(
            "source tolerance must be finite and positive".into(),
        ));
    }
    let start = converted.mesh.source_metadata.len();
    for truth in analytic_truths_from_solid(solid)? {
        let triangle_indices: Vec<usize> = converted
            .triangle_face_ids
            .iter()
            .enumerate()
            .filter_map(|(triangle, &face)| (face == Some(truth.mesh_face_id)).then_some(triangle))
            .collect();
        if triangle_indices.is_empty() {
            continue;
        }
        converted.mesh.source_metadata.push(SourceMetadata {
            version: 1,
            triangle_indices,
            hint: SurfaceHint::ExactCandidate {
                surface: truth.surface,
            },
            source_face_id: Some(truth.source_face_id),
            source_face_name: truth.source_face_name,
            source_surface_id: None,
            orientation: Some(truth.orientation),
            source_tolerance,
        });
    }
    Ok(converted.mesh.source_metadata.len() - start)
}

/// A converted kernel mesh together with its per-triangle, tessellator-local
/// face ownership.
///
/// `triangle_face_ids` contains `None` when the source mesh did not carry face
/// ownership. A present id is the kernel tessellator's sequential face index,
/// not the stable `FaceRecord::id`.
#[derive(Clone, Debug, PartialEq)]
pub struct KernelMeshConversion {
    /// Portable indexed recognition mesh.
    pub mesh: Mesh,
    /// Sequential tessellator face ownership for each triangle.
    pub triangle_face_ids: Vec<Option<u32>>,
}

/// Convert a kernel vector into the neutral recognition DTO.
pub fn vec3_from_kernel(value: brep_kernel::Vec3) -> Vec3 {
    Vec3::new(value.x, value.y, value.z)
}

/// Convert a neutral recognition vector into the kernel boundary type.
pub fn vec3_to_kernel(value: Vec3) -> brep_kernel::Vec3 {
    brep_kernel::Vec3::new(value.x, value.y, value.z)
}

/// Convert the kernel's flat indexed mesh without discarding its transient
/// per-triangle face ownership.
///
/// Kernel derivative normals are retained as per-vertex fitting hints.
/// Triangle normals are still derived from winding and remain authoritative
/// for adjacency, feature detection, region support, and orientation sign.
pub fn convert_kernel_mesh(
    source: &brep_kernel::Mesh,
) -> Result<KernelMeshConversion, RecognitionError> {
    if source.positions.is_empty() {
        return Err(RecognitionError::InvalidMesh(
            "kernel mesh has no positions".into(),
        ));
    }
    if !source.positions.len().is_multiple_of(3) {
        return Err(RecognitionError::InvalidMesh(
            "kernel position buffer must contain xyz triples".into(),
        ));
    }
    if source.indices.is_empty() || !source.indices.len().is_multiple_of(3) {
        return Err(RecognitionError::InvalidMesh(
            "kernel index buffer must contain triangles".into(),
        ));
    }
    if source
        .positions
        .iter()
        .any(|coordinate| !coordinate.is_finite())
    {
        return Err(RecognitionError::InvalidMesh(
            "kernel position buffer contains a non-finite coordinate".into(),
        ));
    }
    if !source.normals.is_empty() && source.normals.len() != source.positions.len() {
        return Err(RecognitionError::InvalidMesh(format!(
            "kernel normal buffer has {} coordinates for {} position coordinates",
            source.normals.len(),
            source.positions.len()
        )));
    }
    if source
        .normals
        .iter()
        .any(|coordinate| !coordinate.is_finite())
    {
        return Err(RecognitionError::InvalidMesh(
            "kernel normal buffer contains a non-finite coordinate".into(),
        ));
    }

    let vertex_count = source.positions.len() / 3;
    if source
        .indices
        .iter()
        .any(|&index| index as usize >= vertex_count)
    {
        return Err(RecognitionError::InvalidMesh(
            "kernel triangle index is outside the position buffer".into(),
        ));
    }
    let triangle_count = source.indices.len() / 3;
    if !source.face_ids.is_empty() && source.face_ids.len() != triangle_count {
        return Err(RecognitionError::InvalidMesh(format!(
            "kernel face-id buffer has {} entries for {triangle_count} triangles",
            source.face_ids.len()
        )));
    }

    let vertices = source
        .positions
        .chunks_exact(3)
        .map(|point| Vec3::new(point[0], point[1], point[2]))
        .collect();
    let vertex_normals = (!source.normals.is_empty()).then(|| {
        source
            .normals
            .chunks_exact(3)
            .map(|normal| Vec3::new(normal[0], normal[1], normal[2]))
            .collect()
    });
    let triangles = source
        .indices
        .chunks_exact(3)
        .map(|triangle| [triangle[0], triangle[1], triangle[2]])
        .collect();
    let triangle_face_ids = if source.face_ids.is_empty() {
        vec![None; triangle_count]
    } else {
        source.face_ids.iter().copied().map(Some).collect()
    };

    Ok(KernelMeshConversion {
        mesh: Mesh {
            vertices,
            triangles,
            vertex_normals,
            source_metadata: Vec::new(),
        },
        triangle_face_ids,
    })
}

/// Convenience conversion for callers that do not need source-face routing.
/// Prefer [`convert_kernel_mesh`] when constructing boolean-healing metadata.
pub fn mesh_from_kernel(source: &brep_kernel::Mesh) -> Result<Mesh, RecognitionError> {
    Ok(convert_kernel_mesh(source)?.mesh)
}

/// A finite, strictly increasing interval used to construct a native kernel
/// NURBS patch. Private fields prevent callers from bypassing validation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FiniteInterval {
    start: f64,
    end: f64,
}

impl FiniteInterval {
    /// Validate and construct a finite, strictly increasing interval.
    pub fn new(start: f64, end: f64) -> Result<Self, RecognitionError> {
        let length = end - start;
        if !start.is_finite() || !end.is_finite() || !length.is_finite() || length <= 0.0 {
            return Err(RecognitionError::InvalidSelection(
                "surface interval bounds must be finite and strictly increasing".into(),
            ));
        }
        Ok(Self { start, end })
    }

    /// Return the inclusive lower construction bound.
    pub fn start(self) -> f64 {
        self.start
    }

    /// Return the inclusive upper construction bound.
    pub fn end(self) -> f64 {
        self.end
    }

    /// Return `end - start`.
    pub fn length(self) -> f64 {
        self.end - self.start
    }
}

/// Required finite construction bounds for carriers that are infinite in at
/// least one direction. These bounds create an untrimmed rectangular NURBS
/// patch; they are not BREP trim loops or p-curves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum FinitePatchBounds {
    /// Two finite parameter intervals for an untrimmed plane patch.
    Plane {
        /// First plane-basis interval.
        u: FiniteInterval,
        /// Second plane-basis interval.
        v: FiniteInterval,
    },
    /// Signed coordinates along a cylinder axis or cone axis. Cone bounds must
    /// lie strictly on the represented positive nappe.
    Axial(FiniteInterval),
}

impl FinitePatchBounds {
    /// Validate plane parameter bounds and construct [`Self::Plane`].
    pub fn plane(
        u_start: f64,
        u_end: f64,
        v_start: f64,
        v_end: f64,
    ) -> Result<Self, RecognitionError> {
        Ok(Self::Plane {
            u: FiniteInterval::new(u_start, u_end)?,
            v: FiniteInterval::new(v_start, v_end)?,
        })
    }

    /// Validate an axial interval and construct [`Self::Axial`].
    pub fn axial(start: f64, end: f64) -> Result<Self, RecognitionError> {
        Ok(Self::Axial(FiniteInterval::new(start, end)?))
    }
}

/// A native exact rational surface plus the recovered face orientation.
///
/// Orientation is deliberately not baked into the NURBS parameterization:
/// callers constructing a future `FaceRecord` must apply it as face sense
/// after creating valid trim curves, p-curves, and shared topology.
#[derive(Clone, Debug)]
pub struct OrientedNurbsSurface {
    /// Exact native kernel NURBS carrier geometry.
    pub surface: brep_kernel::NurbsSurface,
    /// Observed face sense, always `-1` or `+1`.
    pub orientation: i8,
}

fn native_surface_error(surface: AnalyticSurface, reason: impl Into<String>) -> RecognitionError {
    RecognitionError::FitFailed {
        surface: Some(surface.surface_type().name()),
        reason: format!("native NURBS construction failed: {}", reason.into()),
    }
}

/// Convert a recovered carrier to the kernel's exact native rational NURBS
/// representation over explicit finite construction bounds.
///
/// Plane, cylinder, and cone carriers require matching bounds. Sphere and
/// torus constructors already produce finite closed surfaces and therefore
/// require `None`. This function constructs surface geometry only: it does not
/// infer a trim domain, create a `FaceRecord`, or sew a shell.
pub fn nurbs_surface_from_analytic(
    surface: AnalyticSurface,
    orientation: i8,
    bounds: Option<FinitePatchBounds>,
) -> Result<OrientedNurbsSurface, RecognitionError> {
    if !matches!(orientation, -1 | 1) {
        return Err(RecognitionError::InvalidSelection(
            "surface orientation must be -1 or +1".into(),
        ));
    }
    if !surface.is_valid() {
        return Err(native_surface_error(surface, "invalid analytic parameters"));
    }
    let native = match (surface, bounds) {
        (AnalyticSurface::Plane(plane), Some(FinitePatchBounds::Plane { u, v })) => {
            let (u_direction, v_direction) = plane
                .normal
                .orthonormal_basis()
                .ok_or_else(|| native_surface_error(surface, "invalid plane basis"))?;
            let origin = plane.origin + u_direction * u.start() + v_direction * v.start();
            brep_kernel::make_plane(
                vec3_to_kernel(origin),
                vec3_to_kernel(u_direction),
                vec3_to_kernel(v_direction),
                u.length(),
                v.length(),
            )
        }
        (AnalyticSurface::Cylinder(cylinder), Some(FinitePatchBounds::Axial(axial))) => {
            let base = cylinder.axis_origin + cylinder.axis * axial.start();
            brep_kernel::make_cylinder_surface(
                vec3_to_kernel(base),
                vec3_to_kernel(cylinder.axis),
                cylinder.radius,
                axial.length(),
            )
        }
        (AnalyticSurface::Cone(cone), Some(FinitePatchBounds::Axial(axial))) => {
            if axial.start() <= 0.0 {
                return Err(RecognitionError::InvalidSelection(
                    "cone axial bounds must lie strictly on the positive nappe".into(),
                ));
            }
            let tangent = cone.half_angle.tan();
            let base = cone.apex + cone.axis * axial.start();
            brep_kernel::make_cone_surface(
                vec3_to_kernel(base),
                vec3_to_kernel(cone.axis),
                axial.start() * tangent,
                axial.end() * tangent,
                axial.length(),
            )
        }
        (AnalyticSurface::Sphere(sphere), None) => brep_kernel::make_sphere_surface(
            vec3_to_kernel(sphere.center),
            sphere.radius,
            brep_kernel::Vec3::new(0.0, 0.0, 1.0),
        ),
        (AnalyticSurface::Torus(torus), None) => brep_kernel::make_torus_surface(
            vec3_to_kernel(torus.center),
            vec3_to_kernel(torus.axis),
            torus.major_radius,
            torus.minor_radius,
        ),
        (AnalyticSurface::Plane(_), _) => {
            return Err(RecognitionError::InvalidSelection(
                "plane conversion requires plane u/v bounds".into(),
            ));
        }
        (AnalyticSurface::Cylinder(_), _) => {
            return Err(RecognitionError::InvalidSelection(
                "cylinder conversion requires axial bounds".into(),
            ));
        }
        (AnalyticSurface::Cone(_), _) => {
            return Err(RecognitionError::InvalidSelection(
                "cone conversion requires axial bounds".into(),
            ));
        }
        (AnalyticSurface::Sphere(_) | AnalyticSurface::Torus(_), Some(_)) => {
            return Err(RecognitionError::InvalidSelection(
                "sphere and torus conversion use their complete native domains and take no bounds"
                    .into(),
            ));
        }
    }
    .map_err(|reason| native_surface_error(surface, reason))?;
    Ok(OrientedNurbsSurface {
        surface: native,
        orientation,
    })
}

/// Tessellate one complete kernel solid, preserve face ownership and supplied
/// derivative normals, and attach exact metadata for every supported analytic
/// source face.
///
/// This is the preferred bridge for whole-body recognition and STEP corpus
/// validation. Unsupported/freeform faces receive no metadata and therefore
/// continue through generic recognition or remain unresolved.
pub fn tessellate_kernel_solid_with_metadata(
    solid: &brep_kernel::BrepSolid,
    chord_tolerance: f64,
    source_tolerance: Option<f64>,
) -> Result<KernelMeshConversion, RecognitionError> {
    if !chord_tolerance.is_finite() || chord_tolerance <= 0.0 {
        return Err(RecognitionError::InvalidSelection(
            "chord tolerance must be finite and positive".into(),
        ));
    }
    let source =
        brep_kernel::tessellate_brep_watertight(solid, chord_tolerance).map_err(|error| {
            RecognitionError::InvalidMesh(format!("kernel tessellation failed: {error}"))
        })?;
    let mut converted = convert_kernel_mesh(&source)?;
    attach_solid_analytic_metadata(&mut converted, solid, source_tolerance)?;
    Ok(converted)
}

/// Attach one host-provided source hint to every triangle carrying a specified
/// sequential kernel face id.
///
/// Stable topology identity is supplied separately because `mesh_face_id` is
/// only the flattened face number emitted by the tessellator.
#[allow(clippy::too_many_arguments)]
pub fn attach_face_metadata(
    converted: &mut KernelMeshConversion,
    mesh_face_id: u32,
    hint: SurfaceHint,
    source_face_id: Option<u64>,
    source_face_name: Option<String>,
    source_surface_id: Option<String>,
    orientation: Option<i8>,
    source_tolerance: Option<f64>,
) -> Result<(), RecognitionError> {
    if orientation.is_some_and(|sense| !matches!(sense, -1 | 1)) {
        return Err(RecognitionError::InvalidMesh(
            "source orientation must be -1 or +1".into(),
        ));
    }
    let triangle_indices: Vec<usize> = converted
        .triangle_face_ids
        .iter()
        .enumerate()
        .filter_map(|(triangle, &face)| (face == Some(mesh_face_id)).then_some(triangle))
        .collect();
    if triangle_indices.is_empty() {
        return Err(RecognitionError::InvalidSelection(format!(
            "kernel mesh contains no triangles for sequential face {mesh_face_id}"
        )));
    }
    converted.mesh.source_metadata.push(SourceMetadata {
        version: 1,
        triangle_indices,
        hint,
        source_face_id,
        source_face_name,
        source_surface_id,
        orientation,
        source_tolerance,
    });
    Ok(())
}

/// Convert a recognized mathematical carrier to the kernel mesh segmenter's
/// public carrier representation.
///
/// `orientation` is the observed face sense and must be `-1` or `+1`. Plane
/// carriers encode it by orienting their normal; curved kernel carriers expose
/// it as `sense`.
pub fn region_carrier_from_surface(
    surface: AnalyticSurface,
    orientation: i8,
) -> Result<brep_kernel::RegionCarrier, RecognitionError> {
    if !matches!(orientation, -1 | 1) {
        return Err(RecognitionError::InvalidSelection(
            "surface orientation must be -1 or +1".into(),
        ));
    }
    if !surface.is_valid() {
        return Err(RecognitionError::FitFailed {
            surface: Some(surface.surface_type().name()),
            reason: "cannot convert invalid analytic parameters".into(),
        });
    }
    let sign = orientation as f64;
    Ok(match surface {
        AnalyticSurface::Plane(plane) => brep_kernel::RegionCarrier::Plane {
            origin: vec3_to_kernel(plane.origin),
            normal: vec3_to_kernel(plane.normal * sign),
        },
        AnalyticSurface::Cylinder(cylinder) => brep_kernel::RegionCarrier::Cylinder {
            axis_point: vec3_to_kernel(cylinder.axis_origin),
            axis_dir: vec3_to_kernel(cylinder.axis),
            radius: cylinder.radius,
            sense: orientation,
        },
        AnalyticSurface::Cone(cone) => brep_kernel::RegionCarrier::Cone {
            apex: vec3_to_kernel(cone.apex),
            axis_dir: vec3_to_kernel(cone.axis),
            half_angle_rad: cone.half_angle,
            sense: orientation,
        },
        AnalyticSurface::Sphere(sphere) => brep_kernel::RegionCarrier::Sphere {
            center: vec3_to_kernel(sphere.center),
            radius: sphere.radius,
            sense: orientation,
        },
        AnalyticSurface::Torus(torus) => brep_kernel::RegionCarrier::Torus {
            center: vec3_to_kernel(torus.center),
            axis_dir: vec3_to_kernel(torus.axis),
            major_radius: torus.major_radius,
            minor_radius: torus.minor_radius,
            sense: orientation,
        },
    })
}

