//! The DIMENSIONS of a face's carrier surface and an edge's curve, for the
//! Properties Info panel: a cylinder's radius and diameter, a cone's
//! half-angle, a sphere's radius, a torus's two radii, a circular edge's
//! radius and diameter.
//!
//! Read-only reporting. Nothing here changes geometry or feeds a modelling
//! decision; the labels below may therefore name a partial revolution of a
//! straight generatrix a "Cylinder" (what a user calls a half-bore), where
//! `AnalyticSurface::kind_label` keeps calling it a "Surface of revolution".

use crate::{AnalyticSurface, NurbsCurve, NurbsSurface, RevolutionFrame, Vec3};

/// Samples taken along a generatrix or an edge to classify it.
const SAMPLES: usize = 17;

/// Relative band within which samples count as on one line or one circle.
const RELATIVE_TOLERANCE: f64 = 1e-7;

/// A carrier's classification and its named dimensions in display order
/// (millimetres, or degrees for an angle key ending in `Deg`).
#[derive(Clone, Debug, PartialEq)]
pub struct CarrierDimensions {
    /// `"Cylinder"`, `"Cone"`, `"Sphere"`, `"Torus"`, `"Circle"` or `"Arc"`.
    pub label: &'static str,
    /// `(key, value)` pairs: `radius`, `diameter`, `halfAngleDeg`,
    /// `majorRadius`, `minorRadius`, `sweepDeg`.
    pub values: Vec<(&'static str, f64)>,
}

fn round_dimensions(radius: f64) -> Vec<(&'static str, f64)> {
    vec![("radius", radius), ("diameter", 2.0 * radius)]
}

/// The dimensions of a surface's recognised carrier, or `None` for a plane,
/// a free-form patch, or a revolution whose generatrix is neither straight
/// nor circular.
pub fn surface_carrier_dimensions(surface: &NurbsSurface) -> Option<CarrierDimensions> {
    match surface.analytic()? {
        AnalyticSurface::Plane { .. } => None,
        AnalyticSurface::RuledRevolution { rho0, rho1, height, .. } => {
            Some(ruled_dimensions(*rho0, *rho1, *height))
        }
        AnalyticSurface::Sphere { radius, .. } => Some(CarrierDimensions {
            label: "Sphere",
            values: round_dimensions(*radius),
        }),
        AnalyticSurface::Torus { major_radius, minor_radius, .. } => Some(torus(*major_radius, *minor_radius)),
        AnalyticSurface::Revolution { frame, generatrix, .. } => revolution_dimensions(frame, generatrix),
    }
}

fn torus(major_radius: f64, minor_radius: f64) -> CarrierDimensions {
    CarrierDimensions {
        label: "Torus",
        values: vec![("majorRadius", major_radius), ("minorRadius", minor_radius)],
    }
}

/// A straight generatrix from radius `rho0` to `rho1` over axial `height`:
/// a cylinder when the radii agree, else a cone with its half-angle.
fn ruled_dimensions(rho0: f64, rho1: f64, height: f64) -> CarrierDimensions {
    let scale = rho0.abs().max(rho1.abs()).max(height.abs()).max(1.0);
    if (rho0 - rho1).abs() <= RELATIVE_TOLERANCE * scale {
        return CarrierDimensions {
            label: "Cylinder",
            values: round_dimensions(0.5 * (rho0 + rho1).abs()),
        };
    }
    CarrierDimensions {
        label: "Cone",
        values: vec![("halfAngleDeg", (rho1 - rho0).abs().atan2(height.abs()).to_degrees())],
    }
}

/// Classify a general revolution by its generatrix in the (radial, axial)
/// half-plane: straight → cylinder or cone, circular → sphere (centre on the
/// axis) or torus.
fn revolution_dimensions(frame: &RevolutionFrame, generatrix: &NurbsCurve) -> Option<CarrierDimensions> {
    let [t0, t1] = generatrix.domain().ok()?;
    let mut points = Vec::with_capacity(SAMPLES);
    for index in 0..SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / (SAMPLES - 1) as f64;
        let d = generatrix.evaluate(t).ok()?.sub(frame.origin);
        let axial = d.dot(frame.axis);
        let radial = d.sub(frame.axis.scale(axial)).length();
        points.push(Vec3::new(radial, axial, 0.0));
    }
    let (first, last) = (points[0], points[SAMPLES - 1]);
    let scale = points.iter().map(|p| p.x.abs().max(p.y.abs())).fold(1.0, f64::max);
    let tolerance = RELATIVE_TOLERANCE * scale;
    if on_line(&points, tolerance) {
        return Some(ruled_dimensions(first.x, last.x, last.y - first.y));
    }
    let (centre, radius) = circle_through(&points, tolerance)?;
    if centre.x.abs() <= tolerance {
        return Some(CarrierDimensions { label: "Sphere", values: round_dimensions(radius) });
    }
    Some(torus(centre.x, radius))
}

/// The dimensions of a curve over `[t0, t1]` when it is a circle or an arc
/// of one (radius, diameter, and an arc's sweep), else `None`.
pub fn curve_carrier_dimensions(curve: &NurbsCurve, t0: f64, t1: f64) -> Option<CarrierDimensions> {
    let (lo, hi) = (t0.min(t1), t0.max(t1));
    if !(hi > lo) {
        return None;
    }
    let mut points = Vec::with_capacity(SAMPLES);
    for index in 0..SAMPLES {
        points.push(curve.evaluate(lo + (hi - lo) * index as f64 / (SAMPLES - 1) as f64).ok()?);
    }
    let scale = points
        .iter()
        .map(|p| p.x.abs().max(p.y.abs()).max(p.z.abs()))
        .fold(1.0, f64::max);
    let tolerance = RELATIVE_TOLERANCE * scale;
    if on_line(&points, tolerance) {
        return None;
    }
    let (centre, radius) = circle_through(&points, tolerance)?;
    let mut values = round_dimensions(radius);
    let closed = points[0].sub(points[SAMPLES - 1]).length() <= tolerance;
    if closed {
        return Some(CarrierDimensions { label: "Circle", values });
    }
    // Sweep as the sum of the sampled steps' angles, so an arc past a half
    // turn reads as such.
    let sweep: f64 = points
        .windows(2)
        .map(|pair| {
            let a = pair[0].sub(centre);
            let b = pair[1].sub(centre);
            a.cross(b).length().atan2(a.dot(b))
        })
        .sum();
    values.push(("sweepDeg", sweep.to_degrees()));
    Some(CarrierDimensions { label: "Arc", values })
}

/// Whether every point lies within `tolerance` of the line through the first
/// and last (or, for a closed run, the first and the farthest).
fn on_line(points: &[Vec3], tolerance: f64) -> bool {
    let first = points[0];
    let Some(far) = points
        .iter()
        .copied()
        .max_by(|a, b| a.sub(first).length().total_cmp(&b.sub(first).length()))
    else {
        return true;
    };
    let Ok(direction) = far.sub(first).normalized() else {
        return true;
    };
    points.iter().all(|p| {
        let d = p.sub(first);
        d.sub(direction.scale(d.dot(direction))).length() <= tolerance
    })
}

/// The circle through three well-spread samples, accepted only when every
/// sample lies on it (radius and plane) within `tolerance`.
fn circle_through(points: &[Vec3], tolerance: f64) -> Option<(Vec3, f64)> {
    let n = points.len();
    let closed = points[0].sub(points[n - 1]).length() <= tolerance;
    // A closed run's start and end coincide, so take three thirds instead.
    let (a, b, c) = if closed {
        (points[0], points[n / 3], points[2 * n / 3])
    } else {
        (points[0], points[n / 2], points[n - 1])
    };
    let ab = b.sub(a);
    let ac = c.sub(a);
    let normal = ab.cross(ac);
    let denominator = 2.0 * normal.length_squared();
    if denominator <= f64::MIN_POSITIVE {
        return None;
    }
    let centre = a.add(
        normal
            .cross(ab)
            .scale(ac.length_squared())
            .add(ac.cross(normal).scale(ab.length_squared()))
            .scale(1.0 / denominator),
    );
    let radius = a.sub(centre).length();
    let unit_normal = normal.normalized().ok()?;
    let on_circle = points.iter().all(|p| {
        let d = p.sub(centre);
        (d.length() - radius).abs() <= tolerance && d.dot(unit_normal).abs() <= tolerance
    });
    on_circle.then_some((centre, radius))
}

