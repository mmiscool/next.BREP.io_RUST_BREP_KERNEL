//! Cut / split a body by a plane (Golovanov §6.4).
//!
//! The split reuses the ROBUST boolean rather than a bespoke classifier: the
//! plane is realised as two very large half-space TOOL boxes (one covering each
//! side of the plane), and each output piece is `Intersect(solid, tool)`. The
//! boolean machinery imprints the cut plane onto the solid and re-closes the
//! shell, so both pieces inherit the kernel's validated, watertight topology.

use crate::boolean::{boolean_operation, BooleanOperation, BooleanOptions};
use crate::spatial::Aabb;
use crate::topology::{make_box_brep, make_cylinder_brep, BrepSolid};
use crate::transform_topology::{transform_brep, AffineTransform};
use crate::{
    make_cone_brep, make_sphere_brep, make_torus_brep, AnalyticSurface, NurbsSurface, Vec3,
};
use serde::Deserialize;

/// Axis-aligned bounding box of a solid's vertices.
fn solid_aabb(solid: &BrepSolid) -> Aabb {
    let mut bounds = Aabb::empty();
    for vertex in &solid.vertices {
        bounds.include_point(vertex.point);
    }
    bounds
}

/// A boolean result counts as an empty piece when it carries no face geometry —
/// i.e. the half-space tool did not overlap the solid on that side.
fn is_empty_piece(solid: &BrepSolid) -> bool {
    solid.shells.is_empty() || solid.shells.iter().all(|shell| shell.faces.is_empty())
}

/// Build the affine placing a local, origin-centred cube so its local +Z axis
/// maps to `n`, +X to `u`, +Y to `v`, and its centre lands at `center`. The
/// columns of the rotation are `[u v n]` (a right-handed, det = +1 frame), so
/// the map is a proper rigid motion (no orientation reversal needed).
fn frame_transform(u: Vec3, v: Vec3, n: Vec3, center: Vec3) -> Result<AffineTransform, String> {
    AffineTransform::new([
        u.x, v.x, n.x, center.x, //
        u.y, v.y, n.y, center.y, //
        u.z, v.z, n.z, center.z, //
        0.0, 0.0, 0.0, 1.0,
    ])
}

/// Split `solid` into two pieces by the plane through `plane_point` with normal
/// `plane_normal`. Returns `(below, above)` where `below` is the piece on the
/// −n side of the plane and `above` the piece on the +n side.
///
/// Contract: when the plane does not actually divide the solid into two
/// non-degenerate pieces (it misses the body, or is tangent so one side is
/// empty), this returns `Err("split_solid_by_plane: plane does not intersect
/// the solid")` rather than a degenerate/empty piece.
pub fn split_solid_by_plane(
    solid: &BrepSolid,
    plane_point: Vec3,
    plane_normal: Vec3,
) -> Result<(BrepSolid, BrepSolid), String> {
    let n = plane_normal.normalized()?;
    // Orthonormal frame (n, u, v): u ⟂ n (unit), v = n × u (unit); [u v n] is
    // right-handed so the tool placement is a proper rotation.
    let u = n.perpendicular()?;
    let v = n.cross(u);

    let bounds = solid_aabb(solid);
    if !bounds.minimum.x.is_finite() {
        return Err("split_solid_by_plane: solid has no geometry".into());
    }
    let diagonal = bounds.diagonal();
    if diagonal <= 0.0 {
        return Err("split_solid_by_plane: solid is degenerate".into());
    }
    // A tool box 3× the solid diagonal on every side easily covers the body in
    // the plane's tangent directions.
    let length = 3.0 * diagonal;
    let half = 0.5 * length;

    // Centre the tool in the tangent (u, v) plane on the projection of the AABB
    // centre onto the cut plane, so the box brackets the whole solid regardless
    // of where `plane_point` sits within it. Along n it is offset by ±half so
    // the tool's cut face lands exactly on the plane.
    let center = bounds.minimum.add(bounds.maximum).scale(0.5);
    let center_on_plane = center.sub(n.scale(center.sub(plane_point).dot(n)));

    // Local cube centred at the local origin, spanning [-half, half]³.
    let cube = make_box_brep(Vec3::new(-half, -half, -half), length, length, length)?;

    // BELOW: tool centred at plane − n·half, so its +n face lies on the plane
    // and it extends distance `length` along −n, covering the −n side.
    let below_center = center_on_plane.sub(n.scale(half));
    let tool_below = transform_brep(&cube, frame_transform(u, v, n, below_center)?, false)?;

    // ABOVE: mirror to the +n side (centre at plane + n·half).
    let above_center = center_on_plane.add(n.scale(half));
    let tool_above = transform_brep(&cube, frame_transform(u, v, n, above_center)?, false)?;

    let options = BooleanOptions::default();
    let below = boolean_operation(solid, &tool_below, BooleanOperation::Intersect, &options);
    let above = boolean_operation(solid, &tool_above, BooleanOperation::Intersect, &options);

    match (below, above) {
        (Ok(below), Ok(above)) if !is_empty_piece(&below) && !is_empty_piece(&above) => {
            Ok((below, above))
        }
        _ => Err("split_solid_by_plane: plane does not intersect the solid".into()),
    }
}

// ---------------------------------------------------------------------------
// Generalized split by an analytic surface (Golovanov §6.4).
//
// The plane case above splits a body against two large half-space TOOL boxes.
// The analytic cases generalize that idea: an unbounded/bounded analytic
// surface (cylinder, cone, sphere, torus) is realised as ONE CLOSED SOLID
// region big enough to span the body wherever it matters, and the two output
// pieces are the boolean `Intersect` (inside the tool region) and `Subtract`
// (outside it) of the body against that region.  Because every cut runs
// through the validated boolean machinery, each piece inherits watertight
// topology, the cut surface is imprinted onto the body, and the two volumes
// sum to the original.
// ---------------------------------------------------------------------------

/// A closed analytic tool region used to cut a body.  Each variant is realised
/// as a closed solid (unbounded carriers are capped well beyond the body) whose
/// interior is one side of the analytic surface.
#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(tag = "type", rename_all = "lowercase")]
pub enum SplitSurface {
    /// Infinite plane through `point` with `normal`; delegates to the plane path.
    Plane { point: Vec3, normal: Vec3 },
    /// Infinite cylinder about the axis line through `axis_point` along
    /// `axis_dir`, of the given `radius`.  Interior = inside the cylinder.
    Cylinder {
        axis_point: Vec3,
        axis_dir: Vec3,
        radius: f64,
    },
    /// Sphere centred at `center`.  Interior = inside the ball.
    Sphere { center: Vec3, radius: f64 },
    /// Single-nappe cone with its apex at `apex`, opening along `+axis_dir`,
    /// with the given `half_angle` (radians, apex half-angle).  Interior =
    /// inside the cone.
    Cone {
        apex: Vec3,
        axis_dir: Vec3,
        half_angle: f64,
    },
    /// Torus centred at `center` about `axis_dir`.  Interior = inside the tube.
    Torus {
        center: Vec3,
        axis_dir: Vec3,
        major_radius: f64,
        minor_radius: f64,
    },
}

/// Bounds + a sane margin/diagonal for a body, erroring on empty/degenerate
/// geometry the same way the plane path does.
fn solid_extent(solid: &BrepSolid) -> Result<(Aabb, f64), String> {
    let bounds = solid_aabb(solid);
    if !bounds.minimum.x.is_finite() {
        return Err("split_solid_by_surface: solid has no geometry".into());
    }
    let diagonal = bounds.diagonal();
    if diagonal <= 0.0 {
        return Err("split_solid_by_surface: solid is degenerate".into());
    }
    Ok((bounds, diagonal))
}

/// Build the closed tool solid for an analytic tool, sized to fully span the
/// body wherever the analytic surface passes through it.
fn build_tool_solid(solid: &BrepSolid, tool: &SplitSurface) -> Result<BrepSolid, String> {
    let (_, diagonal) = solid_extent(solid)?;
    let margin = diagonal.max(1.0);
    match *tool {
        SplitSurface::Plane { .. } => {
            Err("build_tool_solid: plane is handled by the plane path".into())
        }
        SplitSurface::Cylinder {
            axis_point,
            axis_dir,
            radius,
        } => {
            if radius <= 0.0 {
                return Err("split_solid_by_surface: cylinder radius must be positive".into());
            }
            let axis = axis_dir.normalized()?;
            // Extend the capped cylinder a full margin beyond the body's span
            // along the axis so its caps never cut the body.
            let (t_min, t_max) = axis_span(solid, axis_point, axis);
            let base = axis_point.add(axis.scale(t_min - margin));
            let height = (t_max - t_min) + 2.0 * margin;
            make_cylinder_brep(base, axis, radius, height)
        }
        SplitSurface::Sphere { center, radius } => {
            if radius <= 0.0 {
                return Err("split_solid_by_surface: sphere radius must be positive".into());
            }
            // A sphere is already a closed, bounded region.
            make_sphere_brep(center, radius, Vec3::new(0.0, 0.0, 1.0))
        }
        SplitSurface::Cone {
            apex,
            axis_dir,
            half_angle,
        } => {
            if !(half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2) {
                return Err("split_solid_by_surface: cone half-angle must be in (0, pi/2)".into());
            }
            let axis = axis_dir.normalized()?;
            // Distance of the farthest body point along +axis from the apex.
            let (_, d_max) = axis_span(solid, apex, axis);
            if d_max <= 0.0 {
                return Err(
                    "split_solid_by_surface: cone does not reach the solid (body is behind the apex)"
                        .into(),
                );
            }
            let big_h = d_max + margin;
            // Realise the nappe as a cone whose apex sits at `apex` and whose
            // base cap lands `big_h` past it along +axis: base at apex+axis·H,
            // built with axis pointing back to the apex so top(0-radius)=apex.
            let base = apex.add(axis.scale(big_h));
            let base_radius = big_h * half_angle.tan();
            make_cone_brep(base, axis.scale(-1.0), base_radius, 0.0, big_h)
        }
        SplitSurface::Torus {
            center,
            axis_dir,
            major_radius,
            minor_radius,
        } => {
            if minor_radius <= 0.0 || major_radius <= 0.0 {
                return Err("split_solid_by_surface: torus radii must be positive".into());
            }
            // A torus is already a closed, bounded region.
            make_torus_brep(center, axis_dir, major_radius, minor_radius)
        }
    }
}

/// Signed span `[min, max]` of the body's vertices projected onto the axis line
/// through `origin` along the unit direction `axis`.
fn axis_span(solid: &BrepSolid, origin: Vec3, axis: Vec3) -> (f64, f64) {
    let mut t_min = f64::INFINITY;
    let mut t_max = f64::NEG_INFINITY;
    for vertex in &solid.vertices {
        let t = vertex.point.sub(origin).dot(axis);
        t_min = t_min.min(t);
        t_max = t_max.max(t);
    }
    (t_min, t_max)
}

/// Split `solid` into pieces by an analytic tool surface (Golovanov §6.4).
///
/// For the `Plane` tool this is exactly `split_solid_by_plane`, returned as
/// `[below, above]`.  For a closed analytic tool region (cylinder / sphere /
/// cone / torus) the two pieces are `[inside, outside]` where `inside =
/// solid ∩ tool` and `outside = solid − tool`.  Both pieces are guaranteed
/// non-empty and valid; their volumes sum to the original.
///
/// Contract: when the tool does not actually divide the body into two
/// non-degenerate pieces (it misses the body, or wholly contains / is wholly
/// contained so one side is empty), this returns a clear `Err` rather than a
/// degenerate/empty piece.
pub fn split_solid_by_surface(
    solid: &BrepSolid,
    tool: &SplitSurface,
) -> Result<Vec<BrepSolid>, String> {
    if let SplitSurface::Plane { point, normal } = *tool {
        let (below, above) = split_solid_by_plane(solid, point, normal)?;
        return Ok(vec![below, above]);
    }

    let tool_solid = build_tool_solid(solid, tool)?;
    let options = BooleanOptions::default();
    let inside = boolean_operation(solid, &tool_solid, BooleanOperation::Intersect, &options);
    let outside = boolean_operation(solid, &tool_solid, BooleanOperation::Subtract, &options);

    match (inside, outside) {
        (Ok(inside), Ok(outside))
            if !is_empty_piece(&inside)
                && !is_empty_piece(&outside)
                && inside.validate().is_empty()
                && outside.validate().is_empty() =>
        {
            Ok(vec![inside, outside])
        }
        _ => Err("split_solid_by_surface: tool surface does not divide the solid".into()),
    }
}

/// Map a face's exact analytic carrier to the closed tool region that splits a
/// body by that surface.  Reuses the kernel's own analytic recognition so the
/// caller only has to hand over the selected face's surface (no host-side
/// geometry extraction).  Unrecognized / general-revolution carriers are
/// reported as unsupported (deferred), never approximated.
fn recognized_split_surface(surface: &NurbsSurface) -> Result<SplitSurface, String> {
    let analytic = surface
        .analytic()
        .ok_or("split_solid_by_face_surface: selected face is not an analytic surface")?;
    match analytic {
        AnalyticSurface::Plane {
            origin,
            u_dir,
            v_dir,
            ..
        } => {
            let normal = u_dir.cross(*v_dir).normalized()?;
            Ok(SplitSurface::Plane {
                point: *origin,
                normal,
            })
        }
        AnalyticSurface::RuledRevolution {
            frame,
            rho0,
            rho1,
            height,
        } => {
            // Cylinder when the two radii coincide, otherwise a cone/frustum.
            let scale = rho0.abs().max(rho1.abs()).max(1.0);
            if (rho0 - rho1).abs() <= 1e-9 * scale {
                Ok(SplitSurface::Cylinder {
                    axis_point: frame.origin,
                    axis_dir: frame.axis,
                    radius: 0.5 * (rho0 + rho1),
                })
            } else {
                // radius(axial) = rho0 + slope·axial, apex where radius = 0.
                let slope = (rho1 - rho0) / height;
                let axial_apex = -rho0 / slope;
                let apex = frame.origin.add(frame.axis.scale(axial_apex));
                // The nappe opens in the direction of increasing radius.
                let axis_dir = if slope >= 0.0 {
                    frame.axis
                } else {
                    frame.axis.scale(-1.0)
                };
                Ok(SplitSurface::Cone {
                    apex,
                    axis_dir,
                    half_angle: slope.abs().atan(),
                })
            }
        }
        AnalyticSurface::Sphere { frame, radius } => Ok(SplitSurface::Sphere {
            center: frame.origin,
            radius: *radius,
        }),
        AnalyticSurface::Torus {
            frame,
            major_radius,
            minor_radius,
        } => Ok(SplitSurface::Torus {
            center: frame.origin,
            axis_dir: frame.axis,
            major_radius: *major_radius,
            minor_radius: *minor_radius,
        }),
        AnalyticSurface::Revolution { .. } => Err(
            "split_solid_by_face_surface: general revolved surfaces are not supported as a cut tool"
                .into(),
        ),
    }
}

/// Split `solid` by the analytic carrier of a selected face `surface`
/// (Golovanov §6.4).  The face may be a plane, cylinder, cone, or sphere; the
/// carrier is extended to fully span the body.  Returns the two pieces
/// (`[below, above]` for a plane, `[inside, outside]` otherwise).  Errors on
/// non-analytic / general-revolution faces, or when the carrier does not
/// cleanly divide the body.
pub fn split_solid_by_face_surface(
    solid: &BrepSolid,
    surface: &NurbsSurface,
) -> Result<Vec<BrepSolid>, String> {
    let tool = recognized_split_surface(surface)?;
    split_solid_by_surface(solid, &tool)
}

