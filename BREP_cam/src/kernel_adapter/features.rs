//! Analytic carriers of a kernel body's faces in machine space (lane C13
//! row 1), for `features::recognize`. Planes, cylinders, cones, spheres and
//! tori come straight from the kernel's recognised carriers; a general
//! surface of revolution (what an extruded arc or a fillet along a straight
//! edge recognises as) is classified by sampling its generatrix.

use crate::features::{Carrier, FaceCarrier};
use crate::model::{Point3, RigidTransform, Setup};
use crate::numeric::Tolerance;
use crate::planar::Vec2;
use brep_kernel::{AnalyticSurface, BrepSolid, FaceRecord, NurbsCurve, RevolutionFrame, Vec3};

use super::{face_names, BodyInput};

fn pt(t: &RigidTransform, v: Vec3) -> Point3 {
    t.apply_point(Point3::new(v.x, v.y, v.z))
}

fn dir(t: &RigidTransform, v: Vec3) -> Option<Point3> {
    t.apply_vector(Point3::new(v.x, v.y, v.z)).normalized()
}

/// Radial and axial coordinates of a point about a frame.
fn cylindrical(frame: &RevolutionFrame, p: Vec3) -> (f64, f64) {
    let d = p.sub(frame.origin);
    let axial = d.dot(frame.axis);
    let radial = d.sub(frame.axis.scale(axial)).length();
    (radial, axial)
}

/// Classify a general revolution by its generatrix: straight and parallel to
/// the axis → cylinder, straight → cone, circular → torus.
fn revolution_carrier(frame: &RevolutionFrame, generatrix: &NurbsCurve, t: &RigidTransform, tol: &Tolerance) -> Carrier {
    let Ok([t0, t1]) = generatrix.domain() else { return Carrier::Other { label: "Surface of revolution".into() } };
    let samples: Vec<(f64, f64)> = (0..9)
        .filter_map(|i| generatrix.evaluate(t0 + (t1 - t0) * i as f64 / 8.0).ok())
        .map(|p| cylindrical(frame, p))
        .collect();
    if samples.len() != 9 {
        return Carrier::Other { label: "Surface of revolution".into() };
    }
    let origin = pt(t, frame.origin);
    let Some(axis) = dir(t, frame.axis) else { return Carrier::Other { label: "Surface of revolution".into() } };
    let eps = tol.distance_eps.max(1e-9 * samples.iter().map(|s| s.0.abs().max(s.1.abs())).fold(0.0, f64::max));
    let (r0, h0) = samples[0];
    let (r1, h1) = samples[8];
    // Straight generatrix: every sample on the chord in the (r, h) plane.
    let chord = crate::planar::Segment::line(crate::model::Point2::new(r0, h0), crate::model::Point2::new(r1, h1));
    if samples.iter().all(|&(r, h)| chord.distance(crate::model::Point2::new(r, h)) <= eps) {
        if (r1 - r0).abs() <= eps {
            return Carrier::Cylinder { origin, axis, radius: 0.5 * (r0 + r1) };
        }
        return Carrier::Cone { origin: origin.add(axis.scale(h0)), axis, radius0: r0, radius1: r1, height: h1 - h0 };
    }
    // Circular generatrix: a torus whose tube centre is the circumcentre.
    let (rm, hm) = samples[4];
    let p = |r: f64, h: f64| crate::model::Point2::new(r, h);
    let (a, b, c) = (p(r0, h0), p(rm, hm), p(r1, h1));
    let d = 2.0 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    if d.abs() > 1e-12 {
        let a2 = a.dot(a);
        let b2 = b.dot(b);
        let c2 = c.dot(c);
        let cx = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
        let cy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
        let centre = p(cx, cy);
        let minor = centre.distance(a);
        if samples.iter().all(|&(r, h)| (centre.distance(p(r, h)) - minor).abs() <= eps) {
            return Carrier::Torus { origin: origin.add(axis.scale(cy)), axis, major_radius: cx, minor_radius: minor };
        }
    }
    Carrier::Other { label: "Surface of revolution".into() }
}

/// The carrier of one face record in machine space.
pub fn carrier_of(face: &FaceRecord, t: &RigidTransform, tol: &Tolerance) -> Carrier {
    match face.surface.analytic() {
        Some(AnalyticSurface::Plane { origin, u_dir, v_dir, .. }) => {
            let n = u_dir.cross(*v_dir);
            match dir(t, n) {
                Some(n) => {
                    let normal = if face.same_sense { n } else { n.scale(-1.0) };
                    Carrier::Plane { point: pt(t, *origin), normal }
                }
                None => Carrier::Other { label: "Plane".into() },
            }
        }
        Some(AnalyticSurface::RuledRevolution { frame, rho0, rho1, height }) => match dir(t, frame.axis) {
            Some(axis) if (rho0 - rho1).abs() <= tol.distance_eps.max(1e-9 * rho0.abs().max(1.0)) => {
                Carrier::Cylinder { origin: pt(t, frame.origin), axis, radius: 0.5 * (rho0 + rho1) }
            }
            Some(axis) => Carrier::Cone { origin: pt(t, frame.origin), axis, radius0: *rho0, radius1: *rho1, height: *height },
            None => Carrier::Other { label: "Cylinder".into() },
        },
        Some(AnalyticSurface::Sphere { frame, radius }) => Carrier::Sphere { center: pt(t, frame.origin), radius: *radius },
        Some(AnalyticSurface::Torus { frame, major_radius, minor_radius }) => match dir(t, frame.axis) {
            Some(axis) => Carrier::Torus { origin: pt(t, frame.origin), axis, major_radius: *major_radius, minor_radius: *minor_radius },
            None => Carrier::Other { label: "Torus".into() },
        },
        Some(AnalyticSurface::Revolution { frame, generatrix, .. }) => revolution_carrier(frame, generatrix, t, tol),
        Some(other) => Carrier::Other { label: other.kind_label().into() },
        None => Carrier::Other { label: "free-form".into() },
    }
}

/// Every face of the body with its carrier, addressed by the adapter's
/// persistent names, in the kernel's face order.
pub fn face_carriers(body: &BodyInput<'_>, setup: &Setup, tol: &Tolerance) -> Vec<FaceCarrier> {
    let names = face_names(body.solid);
    body.solid
        .shells
        .iter()
        .flat_map(|s| s.faces.iter())
        .zip(names)
        .map(|(face, name)| FaceCarrier { solid: body.name.to_string(), face: name, carrier: carrier_of(face, &setup.model_to_machine, tol) })
        .collect()
}

/// Number of faces of a body (kept for the row-1 probe test).
pub fn face_count(solid: &BrepSolid, _setup: &Setup) -> usize {
    solid.shells.iter().map(|s| s.faces.len()).sum()
}

