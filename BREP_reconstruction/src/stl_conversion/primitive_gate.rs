//! Cheap geometric eligibility check for whole-primitive recognition when
//! tessellation artifacts fragment the kernel's initial segmentation.
use super::*;

/// This only opens the recognition gate. RANSAC must still establish a complete
/// partition and the primitive builder must still validate the cap topology.
pub(super) fn has_whole_primitive_candidate(
    mesh: &Mesh,
    segmentation: &MeshSegmentation,
    options: &RecognitionOptions,
) -> bool {
    let mut candidates = segmentation
        .regions
        .iter()
        .filter(|r| {
            matches!(
                r.carrier,
                RegionCarrier::Cylinder { .. }
                    | RegionCarrier::Cone { .. }
                    | RegionCarrier::Sphere { .. }
                    | RegionCarrier::Torus { .. }
            )
        })
        .collect::<Vec<_>>();
    candidates.sort_by(|a, b| b.area.total_cmp(&a.area));
    // Bound the work on large assemblies: this path is eligibility for one
    // whole primitive, not another general surface-recognition pass.
    candidates
        .into_iter()
        .take(3)
        .any(|r| covers(mesh, &r.carrier, options))
}

fn covers(mesh: &Mesh, carrier: &RegionCarrier, options: &RecognitionOptions) -> bool {
    if mesh.vertices.is_empty() || mesh.triangles.is_empty() {
        return false;
    }
    let cv = |p: brep_kernel::Vec3| Vec3::new(p.x, p.y, p.z);
    let (surface, axial) = match *carrier {
        RegionCarrier::Sphere { center, radius, .. } => (
            AnalyticSurface::Sphere(crate::SphereSurface {
                center: cv(center),
                radius,
            }),
            None,
        ),
        RegionCarrier::Torus {
            center,
            axis_dir,
            major_radius,
            minor_radius,
            ..
        } => (
            AnalyticSurface::Torus(crate::TorusSurface {
                center: cv(center),
                axis: cv(axis_dir),
                major_radius,
                minor_radius,
            }),
            None,
        ),
        RegionCarrier::Cylinder {
            axis_point,
            axis_dir,
            radius,
            ..
        } => (
            AnalyticSurface::Cylinder(crate::CylinderSurface {
                axis_origin: cv(axis_point),
                axis: cv(axis_dir),
                radius,
            }),
            Some((cv(axis_point), cv(axis_dir))),
        ),
        RegionCarrier::Cone {
            apex,
            axis_dir,
            half_angle_rad,
            ..
        } => (
            AnalyticSurface::Cone(crate::ConeSurface {
                apex: cv(apex),
                axis: cv(axis_dir),
                half_angle: half_angle_rad,
            }),
            Some((cv(apex), cv(axis_dir))),
        ),
        _ => return false,
    };
    let mut lo = mesh.vertices[0];
    let mut hi = lo;
    let mut low_station = f64::INFINITY;
    let mut high_station = f64::NEG_INFINITY;
    for &p in &mesh.vertices {
        lo = Vec3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
        hi = Vec3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        if let Some((origin, axis)) = axial {
            let h = (p - origin).dot(axis);
            low_station = low_station.min(h);
            high_station = high_station.max(h);
        }
    }
    let tolerance = options
        .distance_tolerance
        .max(options.relative_tolerance * (hi - lo).length());
    let membership = mesh
        .vertices
        .iter()
        .map(|&p| {
            let residual = surface.signed_distance(p).abs();
            let mut bits = if residual.is_finite() && residual <= tolerance {
                1u8
            } else {
                0
            };
            if let Some((origin, axis)) = axial {
                let h = (p - origin).dot(axis);
                let radial = ((p - origin) - axis * h).length();
                let radius = match surface {
                    AnalyticSurface::Cylinder(c) => c.radius,
                    AnalyticSurface::Cone(c) => h.max(0.0) * c.half_angle.tan(),
                    _ => unreachable!(),
                };
                if radial <= radius + tolerance {
                    if (h - low_station).abs() <= tolerance {
                        bits |= 2;
                    }
                    if (h - high_station).abs() <= tolerance {
                        bits |= 4;
                    }
                }
            }
            bits
        })
        .collect::<Vec<_>>();
    // Every triangle must fit one carrier or one end-cap plane. Merely having
    // vertices somewhere on the union would admit triangles bridging cuts.
    mesh.triangles.iter().all(|t| {
        t.iter()
            .try_fold(7u8, |bits, &v| membership.get(v as usize).map(|m| bits & m))
            .is_some_and(|bits| bits != 0)
    })
}

