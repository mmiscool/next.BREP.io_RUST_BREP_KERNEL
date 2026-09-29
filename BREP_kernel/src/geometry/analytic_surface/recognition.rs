use super::*;

pub(super) fn nearly_equal_points(a: &NurbsSurface, b: &NurbsSurface, scale: f64) -> bool {
    if a.degree_u != b.degree_u
        || a.degree_v != b.degree_v
        || a.knots_u.len() != b.knots_u.len()
        || a.knots_v.len() != b.knots_v.len()
        || a.control_points.len() != b.control_points.len()
    {
        return false;
    }
    // Numerical knot dedup for exact-reconstruction recognition (see curve.rs).
    let knot_tolerance = crate::curve::KNOT_DEDUP_EPS;
    if a.knots_u
        .iter()
        .zip(&b.knots_u)
        .any(|(x, y)| (x - y).abs() > knot_tolerance)
        || a.knots_v
            .iter()
            .zip(&b.knots_v)
            .any(|(x, y)| (x - y).abs() > knot_tolerance)
    {
        return false;
    }
    let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
    a.control_points
        .iter()
        .zip(&b.control_points)
        .all(|(row_a, row_b)| {
            row_a.len() == row_b.len()
                && row_a.iter().zip(row_b).all(|(p, q)| {
                    (p.x - q.x).abs() <= tolerance
                        && (p.y - q.y).abs() <= tolerance
                        && (p.z - q.z).abs() <= tolerance
                        && (p.w - q.w).abs() <= RECOGNITION_TOLERANCE
                })
        })
}

pub(super) fn surface_scale(surface: &NurbsSurface) -> f64 {
    surface
        .control_points
        .iter()
        .flatten()
        .map(|p| (p.x.abs() / p.w).max(p.y.abs() / p.w).max(p.z.abs() / p.w))
        .fold(0.0, f64::max)
}

pub(super) fn full_circle_knots(spans: usize) -> Vec<f64> {
    let mut knots = vec![0.0, 0.0, 0.0];
    for index in 1..spans {
        let knot = index as f64 / spans as f64;
        knots.extend([knot, knot]);
    }
    knots.extend([1.0, 1.0, 1.0]);
    knots
}

fn recognize_plane(surface: &NurbsSurface) -> Option<AnalyticSurface> {
    if !surface.is_affine().unwrap_or(false) {
        return None;
    }
    let u_domain = [surface.knots_u[0], *surface.knots_u.last()?];
    let v_domain = [surface.knots_v[0], *surface.knots_v.last()?];
    let extent_u = u_domain[1] - u_domain[0];
    let extent_v = v_domain[1] - v_domain[0];
    if extent_u <= 0.0 || extent_v <= 0.0 {
        return None;
    }
    let p00 = surface.control_points[0][0].point().ok()?;
    let p10 = surface.control_points[1][0].point().ok()?;
    let p01 = surface.control_points[0][1].point().ok()?;
    Some(AnalyticSurface::Plane {
        origin: p00,
        u_dir: p10.sub(p00).scale(1.0 / extent_u),
        v_dir: p01.sub(p00).scale(1.0 / extent_v),
        u_domain,
        v_domain,
    })
}

fn recognize_revolution(surface: &NurbsSurface) -> Option<AnalyticSurface> {
    if surface.degree_u != 2 || surface.control_points.len() != 9 {
        return None;
    }
    let expected_knots = full_circle_knots(4);
    if surface.knots_u.len() != expected_knots.len()
        || surface
            .knots_u
            .iter()
            .zip(&expected_knots)
            .any(|(a, b)| (a - b).abs() > 1e-12)
    {
        return None;
    }
    let scale = surface_scale(surface);
    let rows = &surface.control_points;
    // Antipodal full-circle control points straddle the axis; their midpoint
    // lies on it for every generatrix column.
    let mut frame: Option<RevolutionFrame> = None;
    for column in 0..rows[0].len() {
        let p0 = rows[0][column].point().ok()?;
        let p180 = rows[4][column].point().ok()?;
        let p90 = rows[2][column].point().ok()?;
        let center = p0.add(p180).scale(0.5);
        let radial = p0.sub(center);
        let radius = radial.length();
        if radius <= RECOGNITION_TOLERANCE * scale.max(1.0) {
            continue;
        }
        let toward_90 = p90.sub(center);
        let axis = radial.cross(toward_90).normalized().ok()?;
        let x_axis = radial.scale(1.0 / radius);
        let y_axis = axis.cross(x_axis);
        frame = Some(RevolutionFrame {
            origin: center,
            axis,
            x_axis,
            y_axis,
        });
        break;
    }
    let frame = frame?;
    let generatrix =
        NurbsCurve::new(surface.degree_v, surface.knots_v.clone(), rows[0].clone()).ok()?;
    let rebuilt =
        make_revolution(frame.origin, frame.axis, &generatrix, std::f64::consts::TAU).ok()?;
    if !nearly_equal_points(surface, &rebuilt, scale) {
        return None;
    }
    classify_generatrix(surface, frame, &generatrix, scale)
}

fn classify_generatrix(
    surface: &NurbsSurface,
    frame: RevolutionFrame,
    generatrix: &NurbsCurve,
    scale: f64,
) -> Option<AnalyticSurface> {
    let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
    let controls = &generatrix.control_points;
    if generatrix.degree == 1
        && controls.len() == 2
        && (controls[0].w - 1.0).abs() <= RECOGNITION_TOLERANCE
        && (controls[1].w - 1.0).abs() <= RECOGNITION_TOLERANCE
        && (surface.knots_v[0], *surface.knots_v.last()?) == (0.0, 1.0)
        && generatrix_is_meridional_half_ray(generatrix, &frame)
    {
        let p0 = controls[0].point().ok()?;
        let p1 = controls[1].point().ok()?;
        let (_, rho0, z0) = frame.cylindrical(p0);
        let (_, rho1, z1) = frame.cylindrical(p1);
        let height = z1 - z0;
        if height.abs() <= tolerance {
            return None;
        }
        // Rebase the frame origin to the generatrix start's axial position
        // so v = axial / height exactly.
        let origin = frame.origin.add(frame.axis.scale(z0));
        return Some(AnalyticSurface::RuledRevolution {
            frame: RevolutionFrame { origin, ..frame },
            rho0,
            rho1,
            height,
        });
    }
    if generatrix.degree == 2 && controls.len() == 5 {
        // Polar meridian: poles at both ends, equator at the middle column.
        let south = controls[0].point().ok()?;
        let north = controls[4].point().ok()?;
        let (_, rho_south, z_south) = frame.cylindrical(south);
        let (_, rho_north, z_north) = frame.cylindrical(north);
        if rho_south <= tolerance && rho_north <= tolerance {
            let radius = (z_north - z_south) * 0.5;
            if radius <= tolerance {
                return None;
            }
            let center = frame.origin.add(frame.axis.scale(z_south + radius));
            let meridian = make_arc(
                center,
                frame.x_axis,
                frame.axis,
                radius,
                -std::f64::consts::FRAC_PI_2,
                std::f64::consts::FRAC_PI_2,
            )
            .ok()?;
            if curves_nearly_equal(generatrix, &meridian, scale) {
                return Some(AnalyticSurface::Sphere {
                    frame: RevolutionFrame {
                        origin: center,
                        ..frame
                    },
                    radius,
                });
            }
        }
        return None;
    }
    if generatrix.degree == 2 && controls.len() == 9 {
        let tube_start = controls[0].point().ok()?;
        let tube_opposite = controls[4].point().ok()?;
        let tube_center = tube_start.add(tube_opposite).scale(0.5);
        let minor = tube_start.sub(tube_center).length();
        let (_, major, axial) = frame.cylindrical(tube_center);
        if minor <= tolerance || major <= minor || axial.abs() > tolerance {
            return None;
        }
        let tube = make_arc(
            tube_center,
            frame.x_axis,
            frame.axis,
            minor,
            0.0,
            std::f64::consts::TAU,
        )
        .ok()?;
        if curves_nearly_equal(generatrix, &tube, scale) {
            return Some(AnalyticSurface::Torus {
                frame,
                major_radius: major,
                minor_radius: minor,
            });
        }
        return None;
    }
    None
}

fn curves_nearly_equal(a: &NurbsCurve, b: &NurbsCurve, scale: f64) -> bool {
    if a.degree != b.degree
        || a.knots.len() != b.knots.len()
        || a.control_points.len() != b.control_points.len()
    {
        return false;
    }
    if a.knots
        .iter()
        .zip(&b.knots)
        .any(|(x, y)| (x - y).abs() > 1e-12)
    {
        return false;
    }
    let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
    a.control_points
        .iter()
        .zip(&b.control_points)
        .all(|(p, q)| {
            (p.x - q.x).abs() <= tolerance
                && (p.y - q.y).abs() <= tolerance
                && (p.z - q.z).abs() <= tolerance
                && (p.w - q.w).abs() <= RECOGNITION_TOLERANCE
        })
}

pub fn recognize(surface: &NurbsSurface) -> Option<AnalyticSurface> {
    recognize_plane(surface)
        .or_else(|| recognize_revolution(surface))
        .or_else(|| recognize_general_revolution(surface))
}

/// Fallback recognizer: any `make_revolution` product the specialized
/// quadric recognizers above rejected (arbitrary generatrix, partial
/// sweep), validated by exact reconstruction in `revolution_structure`.
fn recognize_general_revolution(surface: &NurbsSurface) -> Option<AnalyticSurface> {
    let structure = revolution_structure(surface)?;
    let spans = (surface.control_points.len() - 1) / 2;
    Some(AnalyticSurface::Revolution {
        frame: structure.frame,
        spans,
        sweep: structure.sweep,
        generatrix: structure.generatrix,
    })
}

/// Whether the generatrix lies in the axis frame's starting meridian
/// half-plane.  Only then can a surface projection be reduced to projection
/// onto the unrotated 3D generatrix.
pub(super) fn generatrix_is_meridional_half_ray(
    generatrix: &NurbsCurve,
    frame: &RevolutionFrame,
) -> bool {
    let mut scale = 0.0_f64;
    for control in &generatrix.control_points {
        let Ok(point) = control.point() else {
            return false;
        };
        let offset = point.sub(frame.origin);
        let radial = offset.sub(frame.axis.scale(offset.dot(frame.axis)));
        scale = scale.max(radial.length());
    }
    let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
    generatrix.control_points.iter().all(|control| {
        let Ok(point) = control.point() else {
            return false;
        };
        let offset = point.sub(frame.origin);
        let radial = offset.sub(frame.axis.scale(offset.dot(frame.axis)));
        radial.dot(frame.y_axis).abs() <= tolerance
            && radial.dot(frame.x_axis) >= -tolerance
    })
}

impl AnalyticSurface {
    /// The centre and radius of a carrier that IS a sphere, whichever way
    /// it is parameterized.
    ///
    /// The `Sphere` variant is the south-to-north meridian swept
    /// counter-clockwise about its polar axis — exactly what `make_sphere_surface`
    /// builds.  A REFLECTED sphere (the mirror feature, any negative-determinant
    /// `transform_brep`) is the same point set with the opposite handedness:
    /// relative to the u-consistent right-handed frame the recognizer derives,
    /// its meridian runs north → south, so the `Sphere` template rejects it and
    /// it recognizes as a general `Revolution`.  That representation is
    /// faithful (explicit generatrix, exact projection), but a closed-form
    /// consumer keyed on the `Sphere` variant alone then falls back to the
    /// marcher — which cannot terminate on a point tangency (a corner blend's
    /// sphere kissing the mirrored copy's face plane).
    ///
    /// Verified by exact reconstruction like every recognition: the generatrix
    /// must be the polar meridian semicircle of `(centre, radius)` in either
    /// direction, every control point and weight matching.
    pub fn sphere_geometry(&self) -> Option<(Vec3, f64)> {
        match self {
            AnalyticSurface::Sphere { frame, radius } => Some((frame.origin, *radius)),
            AnalyticSurface::Revolution {
                frame,
                sweep,
                generatrix,
                ..
            } => {
                if *sweep < std::f64::consts::TAU - 1e-9 {
                    return None;
                }
                let controls = &generatrix.control_points;
                if generatrix.degree != 2 || controls.len() != 5 {
                    return None;
                }
                let start = controls[0].point().ok()?;
                let end = controls[4].point().ok()?;
                let scale = curve_scale(generatrix);
                let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
                // Both meridian ends sit on the axis: the poles.
                let (_, rho_start, z_start) = frame.cylindrical(start);
                let (_, rho_end, z_end) = frame.cylindrical(end);
                if rho_start > tolerance || rho_end > tolerance {
                    return None;
                }
                let radius = (z_end - z_start).abs() * 0.5;
                if radius <= tolerance {
                    return None;
                }
                // The frame origin is on the axis but not necessarily at the
                // centre (`revolution_structure` anchors it at the generatrix's
                // first control circle — a pole here); the centre is the poles'
                // midpoint.
                let center = start.add(end).scale(0.5);
                // Rebuild the meridian in the direction this generatrix runs.
                let polar = if z_end > z_start {
                    frame.axis
                } else {
                    frame.axis.scale(-1.0)
                };
                let meridian = make_arc(
                    center,
                    frame.x_axis,
                    polar,
                    radius,
                    -std::f64::consts::FRAC_PI_2,
                    std::f64::consts::FRAC_PI_2,
                )
                .ok()?;
                curves_nearly_equal(generatrix, &meridian, scale).then_some((center, radius))
            }
            _ => None,
        }
    }

    /// The centre, radius and a RIGHT-HANDED orthonormal basis of a carrier
    /// that IS a sphere — [`Self::sphere_geometry`] plus the orientation the
    /// pole-free cube atlas (`geometry/sphere_chart.rs`) is built on.
    ///
    /// The basis is `[x_axis, y_axis, axis]` of the recognition frame, so
    /// `basis[2]` is the polar axis: the atlas puts the two degenerate poles at
    /// the CENTRES of its `±z` charts, which is the whole point — a pole is then
    /// an ordinary interior point of a regular chart rather than a coordinate
    /// singularity. Derived only from data stored on the surface, so two call
    /// sites looking at the same surface always build the same atlas, and a
    /// REFLECTED sphere (which recognizes as a general `Revolution`) gets a
    /// basis exactly as a direct one does.
    pub fn sphere_frame(&self) -> Option<(Vec3, f64, [Vec3; 3])> {
        let (center, radius) = self.sphere_geometry()?;
        let frame = match self {
            AnalyticSurface::Sphere { frame, .. } => frame,
            AnalyticSurface::Revolution { frame, .. } => frame,
            _ => return None,
        };
        Some((center, radius, [frame.x_axis, frame.y_axis, frame.axis]))
    }

    /// The frame (origin ON the axis at the tube centre's axial position),
    /// major and minor radius of a carrier that IS a torus, whichever way it
    /// is parameterized — the `Torus` variant, or the general `Revolution` a
    /// reflected torus recognizes as (its tube circle runs the opposite way
    /// round, so the `Torus` template rejects it; see [`Self::sphere_geometry`]).
    /// Verified by exact reconstruction of the tube circle in either direction.
    pub fn torus_geometry(&self) -> Option<(RevolutionFrame, f64, f64)> {
        match self {
            AnalyticSurface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => Some((frame.clone(), *major_radius, *minor_radius)),
            AnalyticSurface::Revolution {
                frame,
                sweep,
                generatrix,
                ..
            } => {
                if *sweep < std::f64::consts::TAU - 1e-9 {
                    return None;
                }
                let controls = &generatrix.control_points;
                if generatrix.degree != 2 || controls.len() != 9 {
                    return None;
                }
                let tube_start = controls[0].point().ok()?;
                let tube_opposite = controls[4].point().ok()?;
                let tube_center = tube_start.add(tube_opposite).scale(0.5);
                let minor = tube_start.sub(tube_center).length();
                let scale = curve_scale(generatrix);
                let tolerance = RECOGNITION_TOLERANCE * scale.max(1.0);
                let (_, major, axial) = frame.cylindrical(tube_center);
                if minor <= tolerance || major <= minor {
                    return None;
                }
                for polar in [frame.axis, frame.axis.scale(-1.0)] {
                    let tube = make_arc(
                        tube_center,
                        frame.x_axis,
                        polar,
                        minor,
                        0.0,
                        std::f64::consts::TAU,
                    )
                    .ok()?;
                    if curves_nearly_equal(generatrix, &tube, scale) {
                        return Some((
                            RevolutionFrame {
                                origin: frame.origin.add(frame.axis.scale(axial)),
                                ..frame.clone()
                            },
                            major,
                            minor,
                        ));
                    }
                }
                None
            }
            _ => None,
        }
    }
}

/// Largest absolute Cartesian coordinate over a curve's control points — the
/// curve counterpart of [`surface_scale`], for scale-relative tolerances.
fn curve_scale(curve: &NurbsCurve) -> f64 {
    curve
        .control_points
        .iter()
        .map(|p| (p.x.abs() / p.w).max(p.y.abs() / p.w).max(p.z.abs() / p.w))
        .fold(0.0, f64::max)
}
