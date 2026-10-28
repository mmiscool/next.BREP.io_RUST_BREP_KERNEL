//! The ONE place BREP_cam talks to the BREP kernel (feature `kernel`).
//!
//! A kernel body becomes a [`MeshSnapshot`] in MODEL coordinates at a
//! requested chordal tolerance, with every triangle tagged by the persistent
//! name of the face it tessellates. Everything else in this crate works on
//! the snapshot, so a CAM build without this feature never compiles the kernel.
//!
//! Face identity: the kernel's watertight tessellation tags each triangle with
//! the SEQUENTIAL index of its face over `solid.shells[].faces[]`; this module
//! maps that index to `FaceRecord::name`. A face without a name gets the
//! deterministic fallback `#<index>` so it stays addressable and distinct.

use crate::mesh::{MeshBuilder, MeshError, MeshSnapshot};
use crate::model::{Bounds3, Point3, Setup};
use crate::numeric::Tolerance;
use brep_kernel::{tessellate_brep_watertight, BrepSolid};

/// Why a body could not become a snapshot.
#[derive(Clone, Debug, PartialEq)]
pub enum AdapterError {
    /// The kernel refused to tessellate (its message verbatim).
    Tessellation { solid: String, message: String },
    /// The kernel mesh did not fit the per-triangle face-id layout.
    Mesh { solid: String, error: MeshError },
    /// The chordal tolerance was not finite and positive.
    ChordTolerance(f64),
}

impl std::fmt::Display for AdapterError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            AdapterError::Tessellation { solid, message } => {
                write!(f, "tessellating body {solid:?}: {message}")
            }
            AdapterError::Mesh { solid, error } => write!(f, "mesh of body {solid:?}: {error}"),
            AdapterError::ChordTolerance(v) => write!(f, "chord tolerance {v} must be finite and positive"),
        }
    }
}

impl std::error::Error for AdapterError {}

/// Persistent face names of `solid` in the kernel's sequential face order,
/// with `#<index>` for unnamed faces.
pub fn face_names(solid: &BrepSolid) -> Vec<String> {
    solid
        .shells
        .iter()
        .flat_map(|shell| shell.faces.iter())
        .enumerate()
        .map(|(i, face)| match &face.name {
            Some(name) if !name.is_empty() => name.clone(),
            _ => format!("#{i}"),
        })
        .collect()
}

/// A body to snapshot: its scene name, the kernel solid, and whether it is a
/// fixture (protected, never machined).
pub struct BodyInput<'a> {
    pub name: &'a str,
    pub solid: &'a BrepSolid,
    pub protected_only: bool,
}

/// Tessellate `bodies` at `chord_tolerance` into one snapshot in MODEL
/// coordinates. Bodies are added in the given order; triangle ids follow.
pub fn snapshot_from_bodies(
    bodies: &[BodyInput<'_>],
    chord_tolerance: f64,
    tolerance: Tolerance,
) -> Result<MeshSnapshot, AdapterError> {
    if !(chord_tolerance > 0.0) || !chord_tolerance.is_finite() {
        return Err(AdapterError::ChordTolerance(chord_tolerance));
    }
    let mut builder = MeshBuilder::new(tolerance);
    for body in bodies {
        add_body(&mut builder, body, chord_tolerance)?;
    }
    Ok(builder.finish())
}

/// Tessellate one body into an existing builder.
pub fn add_body(builder: &mut MeshBuilder, body: &BodyInput<'_>, chord_tolerance: f64) -> Result<(), AdapterError> {
    let mesh = tessellate_brep_watertight(body.solid, chord_tolerance)
        .map_err(|message| AdapterError::Tessellation { solid: body.name.to_string(), message })?;
    let names = face_names(body.solid);
    builder
        .add_body_buffers(body.name, body.protected_only, &mesh.positions, &mesh.indices, &mesh.face_ids, &names)
        .map_err(|error| AdapterError::Mesh { solid: body.name.to_string(), error })
}

/// Tessellate the setup's bodies and hand back the snapshot in MACHINE
/// coordinates: the setup's model-to-machine transform applied once, here.
/// `resolve` maps a body name from the setup's target/protected lists to its
/// kernel solid; names it cannot resolve are returned in the second element
/// so the caller reports them per operation instead of silently machining less.
pub fn machine_snapshot<'a, F>(
    setup: &'a Setup,
    chord_tolerance: f64,
    tolerance: Tolerance,
    mut resolve: F,
) -> Result<(MeshSnapshot, Vec<String>), AdapterError>
where
    F: FnMut(&str) -> Option<&'a BrepSolid>,
{
    let mut bodies: Vec<BodyInput<'a>> = Vec::new();
    let mut unresolved = Vec::new();
    let mut seen: Vec<&str> = Vec::new();
    for (list, protected_only) in [(&setup.targets, false), (&setup.protected, true)] {
        for target in list {
            let name = target.solid.as_str();
            if seen.contains(&name) {
                continue;
            }
            seen.push(name);
            match resolve(name) {
                Some(solid) => bodies.push(BodyInput { name, solid, protected_only }),
                None => unresolved.push(target.solid.clone()),
            }
        }
    }
    let model = snapshot_from_bodies(&bodies, chord_tolerance, tolerance)?;
    Ok((crate::mesh::to_machine(setup, &model), unresolved))
}

/// Model-space bounds of a kernel body from its tessellation, for stock sizing.
pub fn body_bounds(solid: &BrepSolid, chord_tolerance: f64) -> Result<Bounds3, AdapterError> {
    let mesh = tessellate_brep_watertight(solid, chord_tolerance)
        .map_err(|message| AdapterError::Tessellation { solid: String::new(), message })?;
    let mut bounds = Bounds3::empty();
    for p in mesh.positions.chunks_exact(3) {
        bounds.include(Point3::new(p[0], p[1], p[2]));
    }
    Ok(bounds)
}

// Lane C5: exact planar regions and chains from bodies and sketches.
pub mod planar;
// Lane C13: the face-adjacency graph for feature recognition.
pub mod features;


/// Exact recognized ruled-revolution carriers; trim/void checks are in holes.
pub fn revolved_faces(body: &BodyInput<'_>, setup: &Setup) -> Vec<crate::holes::RevolvedFace> {
    let names = face_names(body.solid);
    let point = |v: brep_kernel::Vec3| Point3::new(v.x, v.y, v.z);
    body.solid.shells.iter().flat_map(|s| s.faces.iter()).enumerate()
        .filter_map(|(i, face)| {
            if let Some(brep_kernel::AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) = face.surface.analytic() {
                Some(crate::holes::RevolvedFace {
                    solid: body.name.to_string(), face: names[i].clone(),
                    origin: setup.model_to_machine.apply_point(point(frame.origin)),
                    axis: setup.model_to_machine.apply_vector(point(frame.axis)),
                    radius0: *rho0, radius1: *rho1, height: *height,
                })
            } else { None }
        }).collect()
}

/// Certified coaxial analytic carriers for the upper XZ turning section.
/// Unsupported/non-axisymmetric faces refuse by persistent body/face name.
pub fn turning_faces(body: &BodyInput<'_>, setup: &Setup, tolerance: Tolerance) -> Result<Vec<crate::turning::profile::AxialFace>, crate::CamError> {
    use crate::turning::profile::{AxialFace, AxialCarrier};
    use brep_kernel::AnalyticSurface;
    setup.model_to_machine.validate()?;
    tolerance.validate()?;
    let names = face_names(body.solid);
    let point = |v: brep_kernel::Vec3| Point3::new(v.x, v.y, v.z);
    let mut result = Vec::new();
    for (i, face) in body.solid.shells.iter().flat_map(|s| s.faces.iter()).enumerate() {
        let refuse = || crate::CamError::InvalidInput(format!("non-axisymmetric or unsupported turning face {}/{}", body.name, names[i]));
        let coaxial = |origin: Point3, axis: Point3| origin.x.hypot(origin.y) <= tolerance.distance_eps && (axis.z.abs()-1.).abs() <= tolerance.angle_eps;
        let carrier = match face.surface.analytic().ok_or_else(refuse)? {
            AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => {
                let normal = setup.model_to_machine.apply_vector(point(u_dir.cross(*v_dir).normalized().map_err(|_| refuse())?));
                if (normal.z.abs()-1.).abs() > tolerance.angle_eps { return Err(refuse()); }
                AxialCarrier::Plane { z: setup.model_to_machine.apply_point(point(*origin)).z }
            }
            AnalyticSurface::RuledRevolution { frame, rho0, rho1, height } => {
                let origin = setup.model_to_machine.apply_point(point(frame.origin));
                let axis = setup.model_to_machine.apply_vector(point(frame.axis));
                if !coaxial(origin, axis) { return Err(refuse()); }
                AxialCarrier::Line { a: Point3::new(*rho0, 0., origin.z), b: Point3::new(*rho1, 0., origin.z+axis.z*height) }
            }
            AnalyticSurface::Sphere { frame, radius } => {
                let origin = setup.model_to_machine.apply_point(point(frame.origin));
                if origin.x.hypot(origin.y) > tolerance.distance_eps { return Err(refuse()); }
                AxialCarrier::Circle { centre: Point3::new(0., 0., origin.z), radius: *radius }
            }
            AnalyticSurface::Torus { frame, major_radius, minor_radius } => {
                let origin = setup.model_to_machine.apply_point(point(frame.origin));
                let axis = setup.model_to_machine.apply_vector(point(frame.axis));
                if !coaxial(origin, axis) || major_radius <= minor_radius { return Err(refuse()); }
                AxialCarrier::Circle { centre: Point3::new(*major_radius, 0., origin.z), radius: *minor_radius }
            }
            _ => return Err(refuse()),
        };
        result.push(AxialFace { solid: body.name.into(), face: names[i].clone(), carrier });
    }
    Ok(result)
}

/// A selected BREP face sampled in machine space for iso-parametric finishing.
/// Trim polygons have the kernel integration sampler's chordal resolution.
pub struct FlowFace<'a> {
    face: &'a brep_kernel::FaceRecord,
    target: crate::TargetRef,
    transform: crate::RigidTransform,
    domain: crate::flow::UvDomain,
}
impl<'a> FlowFace<'a> {
    pub fn new(face: &'a brep_kernel::FaceRecord, target: crate::TargetRef, setup: &Setup) -> Result<Self, crate::CamError> {
        let fail=crate::CamError::InvalidInput;
        setup.model_to_machine.validate().map_err(|e| fail(e.to_string()))?;
        let u=face.surface.domain_u().map_err(fail)?;
        let v=face.surface.domain_v().map_err(fail)?;
        let trims=brep_kernel::trim_polygons(face).map_err(fail)?.into_iter()
            .map(|l|l.into_iter().map(|p|crate::Point2::new(p[0],p[1])).collect()).collect();
        Ok(Self {face,target,transform:setup.model_to_machine,domain:crate::flow::UvDomain {u,v,trims}})
    }
}
impl crate::flow::ParametricFace for FlowFace<'_> {
    fn target(&self)->crate::TargetRef {self.target.clone()}
    fn domain(&self)->crate::flow::UvDomain {self.domain.clone()}
    fn sample(&self,u:f64,v:f64)->Result<crate::flow::SurfaceSample,crate::CamError> {
        let p=self.face.surface.evaluate(u,v).map_err(crate::CamError::InvalidInput)?;
        let n=self.face.surface.normal(u,v).map_err(crate::CamError::InvalidInput)?;
        let sign=if self.face.same_sense {1.}else{-1.};
        Ok(crate::flow::SurfaceSample {
            point:self.transform.apply_point(Point3::new(p.x,p.y,p.z)),
            normal:self.transform.apply_vector(Point3::new(sign*n.x,sign*n.y,sign*n.z)),
        })
    }
}
