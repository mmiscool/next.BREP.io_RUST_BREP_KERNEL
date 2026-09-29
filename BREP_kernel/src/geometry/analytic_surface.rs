//! Analytic carrier recognition and closed-form geometry for the exact
//! rational-NURBS surfaces the kernel builds by revolution.
//!
//! Every analytic surface in this kernel *is* an exact rational NURBS patch
//! (built by `make_plane` / `make_revolution`), so recognition never changes
//! geometry — it only unlocks closed-form fast paths (projection now, exact
//! intersection curves next) that bypass grid seeding and Newton marching.
//! Recognition is by exact reconstruction: candidate parameters are extracted
//! from the control net, the surface is rebuilt with the same constructor,
//! and every knot/control/weight must match to a scale-relative tolerance.
//! A surface that fails reconstruction is simply not analytic — there are no
//! partial matches and no approximation.

use crate::{make_arc, make_revolution, NurbsCurve, NurbsSurface, SurfaceProjection, Vec3};

const RECOGNITION_TOLERANCE: f64 = 1e-9;

#[derive(Clone, Debug)]
pub struct RevolutionFrame {
    /// A point on the revolution axis.
    pub origin: Vec3,
    /// Unit axis direction; positive rotation follows the right-hand rule.
    pub axis: Vec3,
    /// Unit radial direction of the u = 0 meridian.
    pub x_axis: Vec3,
    /// axis × x_axis, so azimuth θ = atan2(d·y_axis, d·x_axis).
    pub y_axis: Vec3,
}

impl RevolutionFrame {
    /// Azimuth of `point` about the axis in [0, 2π), plus its radial
    /// distance and signed axial coordinate relative to `origin`.
    /// Returns `None` for the azimuth when the point lies on the axis.
    fn cylindrical(&self, point: Vec3) -> (Option<f64>, f64, f64) {
        let d = point.sub(self.origin);
        let axial = d.dot(self.axis);
        let radial_vector = d.sub(self.axis.scale(axial));
        let radius = radial_vector.length();
        if radius <= 1e-14 * (1.0 + axial.abs()) {
            return (None, radius, axial);
        }
        let mut theta = radial_vector
            .dot(self.y_axis)
            .atan2(radial_vector.dot(self.x_axis));
        if theta < 0.0 {
            theta += std::f64::consts::TAU;
        }
        (Some(theta), radius, axial)
    }
}

#[derive(Clone, Debug)]
pub enum AnalyticSurface {
    /// Affine patch: S(u,v) = origin + u·u_dir + v·v_dir over the knot domain.
    Plane {
        origin: Vec3,
        u_dir: Vec3,
        v_dir: Vec3,
        u_domain: [f64; 2],
        v_domain: [f64; 2],
    },
    /// Full revolution of a straight generatrix: cylinders (rho0 == rho1)
    /// and cones/frusta, with v linear along the generatrix over [0, 1].
    RuledRevolution {
        frame: RevolutionFrame,
        rho0: f64,
        rho1: f64,
        height: f64,
    },
    /// Full revolution of a -π/2..π/2 polar meridian arc (two 90° spans).
    Sphere { frame: RevolutionFrame, radius: f64 },
    /// Full revolution of a full tube circle (four 90° spans in v).
    Torus {
        frame: RevolutionFrame,
        major_radius: f64,
        minor_radius: f64,
    },
    /// GENERAL revolution (full or partial sweep) of an arbitrary
    /// generatrix — every other `make_revolution` product the classic
    /// quadric variants above do not cover.  The closest surface point to
    /// a query lies in the query's meridian half-plane, so projection
    /// reduces exactly to a 1D projection onto the generatrix (rotated
    /// rigidly about the axis); out-of-sweep queries compare the two
    /// boundary meridians instead (Golovanov §4.13: prefer analytic
    /// constructions — this was the unrecognized carrier that sent
    /// tangent glue pairs into the marcher).
    Revolution {
        frame: RevolutionFrame,
        spans: usize,
        sweep: f64,
        generatrix: NurbsCurve,
    },
}

impl AnalyticSurface {
    /// A short, human-readable classification of this analytic carrier for the
    /// Properties Info panel: `"Plane"`, `"Cylinder"`, `"Cone"`, `"Sphere"`,
    /// `"Torus"`, or `"Surface of revolution"`. A `RuledRevolution` is a
    /// cylinder when its two generatrix radii match (relative tolerance, same
    /// style as recognition) and a cone otherwise — a zero end-radius apex is
    /// still a cone, no special case needed. A partial-sweep revolve of a
    /// straight profile recognizes as the general `Revolution` (not
    /// `RuledRevolution`), so it reads "Surface of revolution" rather than
    /// "Cylinder"/"Cone".
    pub fn kind_label(&self) -> &'static str {
        match self {
            AnalyticSurface::Plane { .. } => "Plane",
            AnalyticSurface::RuledRevolution { rho0, rho1, .. } => {
                let scale = rho0.abs().max(rho1.abs()).max(1.0);
                if (rho0 - rho1).abs() <= RECOGNITION_TOLERANCE * scale {
                    "Cylinder"
                } else {
                    "Cone"
                }
            }
            AnalyticSurface::Sphere { .. } => "Sphere",
            AnalyticSurface::Torus { .. } => "Torus",
            AnalyticSurface::Revolution { .. } => "Surface of revolution",
        }
    }
}

/// Parameter of the standard tangent-intersection rational quadratic arc
/// construction (`make_arc` / `make_revolution`): a sweep split uniformly
/// into `spans` segments, each with middle weight cos(segment/2).  Maps an
/// angle in [0, sweep] to the curve parameter in [0, 1] exactly.
pub fn circle_angle_to_parameter(spans: usize, sweep: f64, angle: f64) -> f64 {
    let segment = sweep / spans as f64;
    let clamped = angle.clamp(0.0, sweep);
    let mut span = (clamped / segment).floor() as usize;
    if span >= spans {
        span = spans - 1;
    }
    let local = clamped - span as f64 * segment;
    // Derived from tan(θ/2) = t·sin(α/2) / (1 − t + t·cos(α/2)) for the
    // symmetric rational quadratic arc of sweep α: exact, monotone, and
    // singularity-free for α ≤ π/2.
    let half = (0.5 * local).tan();
    let s = (0.5 * segment).sin();
    let c = (0.5 * segment).cos();
    let t = half / (s + half * (1.0 - c));
    (span as f64 + t.clamp(0.0, 1.0)) / spans as f64
}

#[path = "analytic_surface/recognition.rs"]
mod recognition;
#[path = "analytic_surface/intersect.rs"]
mod intersect;
#[path = "analytic_surface/revolution.rs"]
mod revolution;

pub use intersect::{intersect_analytic_pair, plane_ruled_section_arc};
pub(crate) use intersect::{intersect_analytic_pair_with, ruled_gate, ruled_gate_census, RuledGate};
pub use recognition::recognize;
pub use revolution::{revolution_structure, RevolutionStructure};
pub(crate) use revolution::circumcenter;
use recognition::generatrix_is_meridional_half_ray;

impl AnalyticSurface {
    /// Closed-form point projection in the surface's own parameterization.
    /// Returns the exact nearest parameter; the caller evaluates the NURBS at
    /// that parameter so results stay bit-consistent with the carrier.
    pub fn project(&self, surface: &NurbsSurface, point: Vec3) -> Option<SurfaceProjection> {
        let (u, v) = match self {
            AnalyticSurface::Plane {
                origin,
                u_dir,
                v_dir,
                u_domain,
                v_domain,
            } => {
                // Least-squares foot on the (possibly non-orthogonal) affine
                // patch, clamped to the trimmed domain.
                let d = point.sub(*origin);
                let a = u_dir.dot(*u_dir);
                let b = u_dir.dot(*v_dir);
                let c = v_dir.dot(*v_dir);
                let determinant = a * c - b * b;
                if determinant.abs() <= 1e-16 * (a * c).max(1.0) {
                    return None;
                }
                let fu = u_dir.dot(d);
                let fv = v_dir.dot(d);
                let u =
                    ((fu * c - fv * b) / determinant + u_domain[0]).clamp(u_domain[0], u_domain[1]);
                let v =
                    ((fv * a - fu * b) / determinant + v_domain[0]).clamp(v_domain[0], v_domain[1]);
                (u, v)
            }
            AnalyticSurface::RuledRevolution {
                frame,
                rho0,
                rho1,
                height,
            } => {
                let (theta, rho, z) = frame.cylindrical(point);
                let u = theta
                    .map(|angle| circle_angle_to_parameter(4, std::f64::consts::TAU, angle))
                    .unwrap_or(0.0);
                // 2D meridian problem: project (rho, z) onto the generatrix
                // segment (rho0, 0) -> (rho1, height).
                let delta_rho = rho1 - rho0;
                let length_squared = delta_rho * delta_rho + height * height;
                let t = (((rho - rho0) * delta_rho + z * height) / length_squared).clamp(0.0, 1.0);
                (u, t)
            }
            AnalyticSurface::Sphere { frame, radius: _ } => {
                let (theta, rho, z) = frame.cylindrical(point);
                let u = theta
                    .map(|angle| circle_angle_to_parameter(4, std::f64::consts::TAU, angle))
                    .unwrap_or(0.0);
                // Polar angle from the south pole: β ∈ [0, π] over two spans.
                let beta = rho.atan2(-z);
                let v = circle_angle_to_parameter(2, std::f64::consts::PI, beta);
                (u, v)
            }
            AnalyticSurface::Torus {
                frame,
                major_radius,
                minor_radius: _,
            } => {
                let (theta, rho, z) = frame.cylindrical(point);
                let u = theta
                    .map(|angle| circle_angle_to_parameter(4, std::f64::consts::TAU, angle))
                    .unwrap_or(0.0);
                let mut psi = z.atan2(rho - major_radius);
                if psi < 0.0 {
                    psi += std::f64::consts::TAU;
                }
                let v = circle_angle_to_parameter(4, std::f64::consts::TAU, psi);
                (u, v)
            }
            AnalyticSurface::Revolution {
                frame,
                spans,
                sweep,
                generatrix,
            } => {
                if !generatrix_is_meridional_half_ray(generatrix, frame) {
                    return None;
                }
                let (theta, rho, z) = frame.cylindrical(point);
                let full = *sweep >= std::f64::consts::TAU - 1e-9;
                // Rotating the query rigidly about the axis to a meridian
                // preserves its distance to that meridian's generatrix copy,
                // so each candidate reduces to a 1D curve projection.
                let point_at_angle = |angle: f64| {
                    let radial = frame
                        .x_axis
                        .scale(angle.cos())
                        .add(frame.y_axis.scale(angle.sin()));
                    frame.origin.add(radial.scale(rho)).add(frame.axis.scale(z))
                };
                let mut candidates: Vec<(f64, Vec3)> = Vec::with_capacity(2);
                match theta {
                    None => candidates.push((0.0, point)),
                    Some(angle) if full || angle <= *sweep => {
                        // Rotate the query to the generatrix meridian (angle 0).
                        candidates.push((
                            circle_angle_to_parameter(*spans, *sweep, angle),
                            point_at_angle(0.0),
                        ));
                    }
                    Some(angle) => {
                        // Outside the sweep: the minimizer sits on one of the
                        // two boundary meridians; compare both.  The start
                        // meridian IS the generatrix, so the original point
                        // projects onto it directly; the end meridian maps to
                        // the generatrix by a rigid rotation of the query.
                        candidates.push((0.0, point));
                        candidates.push((1.0, point_at_angle(angle - *sweep)));
                    }
                }
                let mut best: Option<(f64, f64, f64)> = None;
                for (u, query) in candidates {
                    let Ok(projection) = crate::project_point_to_curve(generatrix, query) else {
                        return None;
                    };
                    if best
                        .map(|(_, _, distance)| projection.distance < distance)
                        .unwrap_or(true)
                    {
                        best = Some((u, projection.u, projection.distance));
                    }
                }
                let (u, v, _) = best?;
                (u, v)
            }
        };
        let projected = surface.evaluate(u, v).ok()?;
        Some(SurfaceProjection {
            u,
            v,
            point: projected,
            distance: projected.sub(point).length(),
        })
    }

    /// True when the closed-form projection is the exact global minimizer
    /// (everywhere except on-axis queries, where any meridian ties).
    pub fn frame(&self) -> Option<&RevolutionFrame> {
        match self {
            AnalyticSurface::Plane { .. } => None,
            AnalyticSurface::RuledRevolution { frame, .. }
            | AnalyticSurface::Sphere { frame, .. }
            | AnalyticSurface::Torus { frame, .. }
            | AnalyticSurface::Revolution { frame, .. } => Some(frame),
        }
    }
}
