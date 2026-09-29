//! Safe translation of neighboring analytic geometry into constrained hints.
//!
//! These helpers deliberately produce ordinary [`SurfaceHint::Constrained`]
//! values. They do not add a hidden coupled solver or claim relations that the
//! current parameter-group constraints cannot preserve.

use crate::{
    AnalyticSurface, ConstraintMask, CylinderSurface, MetadataTrust, PlaneSurface, SphereSurface,
    SurfaceConstraints, SurfaceHint, SurfaceType, Vec3,
};
use std::fmt::{Display, Formatter};

/// Failure to express a requested design-intent relation with the current
/// parameter-group constraint model.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum DesignIntentError {
    /// The target or reference carrier contains invalid parameters.
    InvalidSurface {
        /// Whether the invalid carrier was the `target` or `reference`.
        role: &'static str,
    },
    /// The supplied geometry does not determine a unique requested relation.
    DegenerateRelation {
        /// Human-readable name of the degenerate relation.
        relation: &'static str,
    },
    /// The relation cannot be represented by the current parameter groups.
    UnsupportedRelation {
        /// Human-readable name of the unsupported relation.
        relation: &'static str,
        /// Primitive type that would be constrained.
        target: SurfaceType,
        /// Primitive type used as design-intent evidence.
        reference: SurfaceType,
    },
}

impl Display for DesignIntentError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidSurface { role } => write!(formatter, "invalid {role} analytic surface"),
            Self::DegenerateRelation { relation } => {
                write!(formatter, "cannot derive a unique {relation} relation")
            }
            Self::UnsupportedRelation {
                relation,
                target,
                reference,
            } => write!(
                formatter,
                "{relation} is not representable for {target:?} from {reference:?} with current parameter groups"
            ),
        }
    }
}

impl std::error::Error for DesignIntentError {}

fn validate(target: AnalyticSurface, reference: AnalyticSurface) -> Result<(), DesignIntentError> {
    if !target.is_valid() {
        return Err(DesignIntentError::InvalidSurface { role: "target" });
    }
    if !reference.is_valid() {
        return Err(DesignIntentError::InvalidSurface { role: "reference" });
    }
    Ok(())
}

fn hint(surface: AnalyticSurface, fixed: ConstraintMask) -> SurfaceHint {
    SurfaceHint::Constrained {
        surface_type: surface.surface_type(),
        constraints: SurfaceConstraints {
            initial: Some(surface),
            fixed,
        },
        trust: MetadataTrust::StrongHint,
    }
}

fn axial_line(surface: AnalyticSurface) -> Option<(Vec3, Vec3)> {
    match surface {
        AnalyticSurface::Cylinder(value) => Some((value.axis_origin, value.axis)),
        AnalyticSurface::Cone(value) => Some((value.apex, value.axis)),
        AnalyticSurface::Torus(value) => Some((value.center, value.axis)),
        _ => None,
    }
}

fn aligned(reference: Vec3, preferred: Vec3) -> Vec3 {
    if reference.dot(preferred) < 0.0 {
        -reference
    } else {
        reference
    }
}

/// Fix a target cylinder, cone, or torus axis direction to an existing axial
/// carrier. Cone-to-nondirected-carrier transfer is rejected because a cone's
/// nappe direction is not sign-equivalent.
pub fn constrain_shared_axis(
    target: AnalyticSurface,
    reference: AnalyticSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    validate(target, reference)?;
    let (_, reference_axis) =
        axial_line(reference).ok_or_else(|| unsupported("shared axis", target, reference))?;
    let constrained = match target {
        AnalyticSurface::Cylinder(mut value) => {
            value.axis = aligned(reference_axis, value.axis);
            AnalyticSurface::Cylinder(value)
        }
        AnalyticSurface::Torus(mut value) => {
            value.axis = aligned(reference_axis, value.axis);
            AnalyticSurface::Torus(value)
        }
        AnalyticSurface::Cone(mut value) if matches!(reference, AnalyticSurface::Cone(_)) => {
            value.axis = reference_axis;
            AnalyticSurface::Cone(value)
        }
        _ => return Err(unsupported("shared axis", target, reference)),
    };
    Ok(hint(
        constrained,
        ConstraintMask {
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Constrain a cylinder to the complete axis line of another axial carrier.
/// This is safe because a cylinder's axis origin has gauge freedom along the
/// line; cone apex and torus center locations do not, so those target types are
/// intentionally unsupported here.
pub fn constrain_cylinder_coaxial(
    target: CylinderSurface,
    reference: AnalyticSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    let target_surface = AnalyticSurface::Cylinder(target);
    validate(target_surface, reference)?;
    let (reference_origin, reference_axis) = axial_line(reference)
        .ok_or_else(|| unsupported("coaxial axes", target_surface, reference))?;
    let constrained = AnalyticSurface::Cylinder(CylinderSurface {
        axis_origin: reference_origin,
        axis: aligned(reference_axis, target.axis),
        radius: target.radius,
    });
    Ok(hint(
        constrained,
        ConstraintMask {
            origin_or_center: true,
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Constrain a plane to be perpendicular to a known axis. The plane normal is
/// the axis direction; its offset remains free.
pub fn constrain_plane_perpendicular_to_axis(
    target: PlaneSurface,
    axis: Vec3,
) -> Result<SurfaceHint, DesignIntentError> {
    let axis = axis.normalized().filter(|axis| axis.is_finite()).ok_or(
        DesignIntentError::DegenerateRelation {
            relation: "plane perpendicular to axis",
        },
    )?;
    let target_surface = AnalyticSurface::Plane(target);
    if !target_surface.is_valid() {
        return Err(DesignIntentError::InvalidSurface { role: "target" });
    }
    Ok(hint(
        AnalyticSurface::Plane(PlaneSurface {
            normal: aligned(axis, target.normal),
            ..target
        }),
        ConstraintMask {
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Preserve parallelism to a reference plane while leaving plane offset free.
pub fn constrain_planes_parallel(
    target: PlaneSurface,
    reference: PlaneSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    let target_surface = AnalyticSurface::Plane(target);
    let reference_surface = AnalyticSurface::Plane(reference);
    validate(target_surface, reference_surface)?;
    Ok(hint(
        AnalyticSurface::Plane(PlaneSurface {
            normal: aligned(reference.normal, target.normal),
            ..target
        }),
        ConstraintMask {
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Preserve perpendicularity to a reference plane using the target's current
/// normal to select the otherwise non-unique perpendicular direction.
pub fn constrain_planes_perpendicular(
    target: PlaneSurface,
    reference: PlaneSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    let target_surface = AnalyticSurface::Plane(target);
    let reference_surface = AnalyticSurface::Plane(reference);
    validate(target_surface, reference_surface)?;
    let projected = target.normal - reference.normal * target.normal.dot(reference.normal);
    let normal = projected
        .normalized()
        .ok_or(DesignIntentError::DegenerateRelation {
            relation: "perpendicular planes",
        })?;
    Ok(hint(
        AnalyticSurface::Plane(PlaneSurface { normal, ..target }),
        ConstraintMask {
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Fix a plane to the same infinite carrier as a reference plane. Tangential
/// origin coordinates are a gauge and are preserved from the target.
pub fn constrain_planes_coplanar(
    target: PlaneSurface,
    reference: PlaneSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    let target_surface = AnalyticSurface::Plane(target);
    let reference_surface = AnalyticSurface::Plane(reference);
    validate(target_surface, reference_surface)?;
    let normal = aligned(reference.normal, target.normal);
    let origin = target.origin + normal * (reference.origin - target.origin).dot(normal);
    Ok(hint(
        AnalyticSurface::Plane(PlaneSurface { origin, normal }),
        ConstraintMask {
            origin_or_center: true,
            axis_or_normal: true,
            ..Default::default()
        },
    ))
}

/// Fix sphere or torus centers to an existing sphere/torus center.
pub fn constrain_concentric_centers(
    target: AnalyticSurface,
    reference: AnalyticSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    validate(target, reference)?;
    let reference_center = match reference {
        AnalyticSurface::Sphere(value) => value.center,
        AnalyticSurface::Torus(value) => value.center,
        _ => return Err(unsupported("concentric centers", target, reference)),
    };
    let constrained = match target {
        AnalyticSurface::Sphere(mut value) => {
            value.center = reference_center;
            AnalyticSurface::Sphere(value)
        }
        AnalyticSurface::Torus(mut value) => {
            value.center = reference_center;
            AnalyticSurface::Torus(value)
        }
        _ => return Err(unsupported("concentric centers", target, reference)),
    };
    Ok(hint(
        constrained,
        ConstraintMask {
            origin_or_center: true,
            ..Default::default()
        },
    ))
}

/// Fix equal radius groups for two carriers of the same type. For tori this
/// copies and fixes both major and minor radii; mixed-type radius semantics are
/// deliberately rejected.
pub fn constrain_equal_radii(
    target: AnalyticSurface,
    reference: AnalyticSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    validate(target, reference)?;
    let (constrained, fixed) = match (target, reference) {
        (AnalyticSurface::Sphere(mut target), AnalyticSurface::Sphere(reference)) => {
            target.radius = reference.radius;
            (
                AnalyticSurface::Sphere(target),
                ConstraintMask {
                    radius: true,
                    ..Default::default()
                },
            )
        }
        (AnalyticSurface::Cylinder(mut target), AnalyticSurface::Cylinder(reference)) => {
            target.radius = reference.radius;
            (
                AnalyticSurface::Cylinder(target),
                ConstraintMask {
                    radius: true,
                    ..Default::default()
                },
            )
        }
        (AnalyticSurface::Torus(mut target), AnalyticSurface::Torus(reference)) => {
            target.major_radius = reference.major_radius;
            target.minor_radius = reference.minor_radius;
            (
                AnalyticSurface::Torus(target),
                ConstraintMask {
                    radius: true,
                    major_radius: true,
                    ..Default::default()
                },
            )
        }
        _ => return Err(unsupported("equal radii", target, reference)),
    };
    Ok(hint(constrained, fixed))
}

/// Derive exact sphere/plane tangency from the target's current side and
/// orientation. Other tangency pairs require coupled parameters and are
/// rejected explicitly.
pub fn constrain_tangent(
    target: AnalyticSurface,
    reference: AnalyticSurface,
) -> Result<SurfaceHint, DesignIntentError> {
    validate(target, reference)?;
    match (target, reference) {
        (AnalyticSurface::Plane(target), AnalyticSurface::Sphere(reference)) => {
            let signed = (target.origin - reference.center).dot(target.normal);
            let side = if signed < 0.0 { -1.0 } else { 1.0 };
            let origin = reference.center + target.normal * (side * reference.radius);
            Ok(hint(
                AnalyticSurface::Plane(PlaneSurface { origin, ..target }),
                ConstraintMask {
                    origin_or_center: true,
                    axis_or_normal: true,
                    ..Default::default()
                },
            ))
        }
        (AnalyticSurface::Sphere(target), AnalyticSurface::Plane(reference)) => {
            let signed = (target.center - reference.origin).dot(reference.normal);
            let side = if signed < 0.0 { -1.0 } else { 1.0 };
            let tangent_component = target.center - reference.normal * signed;
            let center = tangent_component + reference.normal * (side * target.radius);
            Ok(hint(
                AnalyticSurface::Sphere(SphereSurface { center, ..target }),
                ConstraintMask {
                    origin_or_center: true,
                    radius: true,
                    ..Default::default()
                },
            ))
        }
        _ => Err(unsupported("tangency", target, reference)),
    }
}

fn unsupported(
    relation: &'static str,
    target: AnalyticSurface,
    reference: AnalyticSurface,
) -> DesignIntentError {
    DesignIntentError::UnsupportedRelation {
        relation,
        target: target.surface_type(),
        reference: reference.surface_type(),
    }
}

