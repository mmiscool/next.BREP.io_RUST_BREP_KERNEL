//! The point three carriers share — the corner a vertex blend was cut from.
//!
//! Un-rounding a corner asks for the vertex the blend replaced: the one point
//! lying on all three walls. Three planes answer in closed form
//! (`corner_heal.rs` intersects them directly). Once a wall is a cylinder, a
//! cone, a sphere or a torus there is no such form in general, and "one point"
//! is not even true: two planes and a cylinder meet in TWO points, a plane and
//! two cylinders in up to four. What is well-posed is the LOCAL question — the
//! triple point nearest a seed the caller took from the deleted corner's own
//! geometry — and this module answers that, with what the answer is worth.
//!
//! ## The solve
//!
//! Each carrier is read as the zero set of a SIGNED DISTANCE with a unit
//! gradient ([`TripleCarrier`]). An analytic carrier is its INFINITE surface —
//! the plane, the cylinder or cone about its axis, the sphere, the torus —
//! never the NURBS patch that happens to carry the face. A patch stops where
//! its face stopped and extended evaluation of it continues along a tangent
//! plane without bound, so a Newton run on it can converge onto a surface that
//! does not exist. A fitted carrier has no such reading; it is its patch, and a
//! point whose foot on it is not a perpendicular foot (it sits on a domain edge
//! the point overruns) is refused rather than accepted on an extrapolation.
//!
//! The iteration is Newton on the three distances: `J·δ = −f`, where the rows
//! of `J` are the three unit normals at the current iterate. Each step moves to
//! the intersection of the three tangent planes, and it converges
//! quadratically wherever the three normals are independent.
//!
//! ## What is reported
//!
//! [`TriplePoint`] carries the iteration count, the RESIDUAL (the largest
//! distance from the point to any of the three carriers), and σ_min, the
//! smallest singular value of the unit-normal matrix. σ_min is the conditioning:
//! three mutually perpendicular walls read 1, and two walls meeting at a
//! dihedral deviation θ read about `√(1 − cos θ)`. Two first-order bounds follow
//! from it, both reported:
//!
//! * `error_bound = √3·residual/σ_min` — how far the returned point can be from
//!   the exact triple point of THESE carriers;
//! * `sensitivity = √3·tolerance/σ_min` — how far the triple point itself moves
//!   when each carrier moves by the model tolerance. A small residual on
//!   near-tangent walls still has a large sensitivity, and that is the case a
//!   residual alone reads as a success.
//!
//! The solve refuses when the sensitivity exceeds the caller's position bar —
//! the incidence bar the heal holds its vertices to, so no new constant enters
//! — as well as when the normals are singular, when the iterate leaves the
//! caller's reach of the seed, when it does not converge, or when it lands on a
//! carrier's focal set (an axis, a centre, a tube circle) where the distance has
//! no gradient. Every refusal quotes what was measured.
//!
//! ## What it does not decide
//!
//! Which root is the right one. A converged point is a root; whether it is the
//! corner the blend was cut from — on the side of each rim the blend removed,
//! and the only admissible root near the corner — is the caller's question,
//! because only the caller holds the rims. [`carrier_crossings`] enumerates the
//! roots along a curve so the caller can ask it.

use super::*;
use crate::{project_point_to_surface, project_point_to_surface_seeded};
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// One carrier of a triple-point solve, read as a signed distance.
#[derive(Clone, Debug)]
pub(super) enum TripleCarrier<'a> {
    Plane {
        origin: Vec3,
        normal: Vec3,
    },
    /// A cylinder or cone: the infinite line `ρ = rho0 + slope·axial` in the
    /// meridian half-plane about `axis` through `origin`.
    Ruled {
        origin: Vec3,
        axis: Vec3,
        rho0: f64,
        slope: f64,
    },
    Sphere {
        center: Vec3,
        radius: f64,
    },
    Torus {
        origin: Vec3,
        axis: Vec3,
        major: f64,
        minor: f64,
    },
    /// A fitted patch, read through its own projector.
    Patch(&'a NurbsSurface),
}

/// The kind name a refusal quotes.
pub(super) fn carrier_kind(carrier: &TripleCarrier) -> &'static str {
    match carrier {
        TripleCarrier::Plane { .. } => "plane",
        TripleCarrier::Ruled { .. } => "cylinder/cone",
        TripleCarrier::Sphere { .. } => "sphere",
        TripleCarrier::Torus { .. } => "torus",
        TripleCarrier::Patch(_) => "fitted patch",
    }
}

impl<'a> TripleCarrier<'a> {
    /// Read `surface` as its infinite analytic carrier when it has one, and as
    /// its patch otherwise.
    pub(super) fn of(surface: &'a NurbsSurface) -> Result<Self, KernelRefusal> {
        let Some(analytic) = surface.analytic() else {
            return Ok(Self::Patch(surface));
        };
        Ok(match analytic {
            AnalyticSurface::Plane { origin, u_dir, v_dir, .. } => Self::Plane {
                origin: *origin,
                normal: u_dir.cross(*v_dir).normalized().or_refuse(KernelStage::Classify, "carrier_normal")?,
            },
            AnalyticSurface::RuledRevolution {
                frame,
                rho0,
                rho1,
                height,
            } => {
                if height.abs() <= PARALLEL_EPS {
                    return Ok(Self::Patch(surface));
                }
                Self::Ruled {
                    origin: frame.origin,
                    axis: frame.axis.normalized().or_refuse(KernelStage::Classify, "carrier_axis")?,
                    rho0: *rho0,
                    slope: (rho1 - rho0) / height,
                }
            }
            AnalyticSurface::Sphere { frame, radius } => Self::Sphere {
                center: frame.origin,
                radius: *radius,
            },
            AnalyticSurface::Torus {
                frame,
                major_radius,
                minor_radius,
            } => Self::Torus {
                origin: frame.origin,
                axis: frame.axis.normalized().or_refuse(KernelStage::Classify, "carrier_axis")?,
                major: *major_radius,
                minor: *minor_radius,
            },
            AnalyticSurface::Revolution {
                frame, generatrix, ..
            } => {
                // A straight generatrix sweeps a cone, a cylinder or an annulus;
                // anything else is read through its patch.
                let axis = frame.axis.normalized().or_refuse(KernelStage::Classify, "carrier_axis")?;
                let Some((start, end)) = generatrix.straight_segment(PARALLEL_EPS) else {
                    return Ok(Self::Patch(surface));
                };
                let meridian = |point: Vec3| {
                    let delta = point.sub(frame.origin);
                    let axial = delta.dot(axis);
                    (delta.sub(axis.scale(axial)).length(), axial)
                };
                let (rho_start, axial_start) = meridian(start);
                let (rho_end, axial_end) = meridian(end);
                let rise = axial_end - axial_start;
                if rise.abs() <= PARALLEL_EPS {
                    // Perpendicular to the axis: a flat annulus.
                    Self::Plane {
                        origin: start,
                        normal: axis,
                    }
                } else {
                    let slope = (rho_end - rho_start) / rise;
                    Self::Ruled {
                        origin: frame.origin,
                        axis,
                        rho0: rho_start - slope * axial_start,
                        slope,
                    }
                }
            }
        })
    }

    /// `(signed distance, unit gradient, true distance)` at `point`. The true
    /// distance equals `|signed|` on an analytic carrier; on a patch it is the
    /// distance to the foot, which exceeds the signed reading when the foot is
    /// clamped to the patch's edge. `foot` carries a patch's previous foot so
    /// the projection stays on one branch.
    fn distance(
        &self,
        point: Vec3,
        foot: &mut Option<(f64, f64)>,
    ) -> Result<(f64, Vec3, f64), KernelRefusal> {
        const FOCAL: f64 = 1e-12;
        match self {
            Self::Plane { origin, normal } => {
                let signed = point.sub(*origin).dot(*normal);
                Ok((signed, *normal, signed.abs()))
            }
            Self::Ruled {
                origin,
                axis,
                rho0,
                slope,
            } => {
                let delta = point.sub(*origin);
                let axial = delta.dot(*axis);
                let radial = delta.sub(axis.scale(axial));
                let rho = radial.length();
                if rho <= FOCAL * (1.0 + axial.abs()) {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "focal_axis",
                        "the point lies on the cylinder/cone axis",
                    ));
                }
                let norm = (1.0 + slope * slope).sqrt();
                let signed = (rho - rho0 - slope * axial) / norm;
                let gradient = radial.scale(1.0 / rho).sub(axis.scale(*slope)).scale(1.0 / norm);
                Ok((signed, gradient, signed.abs()))
            }
            Self::Sphere { center, radius } => {
                let delta = point.sub(*center);
                let length = delta.length();
                if length <= FOCAL * (1.0 + radius) {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "focal_centre",
                        "the point lies on the sphere's centre",
                    ));
                }
                let signed = length - radius;
                Ok((signed, delta.scale(1.0 / length), signed.abs()))
            }
            Self::Torus {
                origin,
                axis,
                major,
                minor,
            } => {
                let delta = point.sub(*origin);
                let axial = delta.dot(*axis);
                let radial = delta.sub(axis.scale(axial));
                let rho = radial.length();
                if rho <= FOCAL * (1.0 + major) {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "focal_torus_axis",
                        "the point lies on the torus axis",
                    ));
                }
                let across = rho - major;
                let tube = (across * across + axial * axial).sqrt();
                if tube <= FOCAL * (1.0 + minor) {
                    return Err(KernelRefusal::ill_posed(
                        KernelStage::Classify,
                        "focal_tube",
                        "the point lies on the torus's tube circle",
                    ));
                }
                let signed = tube - minor;
                let gradient = radial
                    .scale(across / (rho * tube))
                    .add(axis.scale(axial / tube));
                Ok((signed, gradient, signed.abs()))
            }
            Self::Patch(surface) => {
                let projection = match *foot {
                    Some((u, v)) => project_point_to_surface_seeded(surface, point, u, v)
                        .or_refuse(KernelStage::Classify, "project")?,
                    None => project_point_to_surface(surface, point).or_refuse(KernelStage::Classify, "project")?,
                };
                *foot = Some((projection.u, projection.v));
                let normal = surface
                    .normal(projection.u, projection.v)
                    .or_refuse(KernelStage::Classify, "normal")?
                    .normalized()
                    .or_refuse(KernelStage::Classify, "normal")?;
                let offset = point.sub(projection.point);
                Ok((offset.dot(normal), normal, offset.length()))
            }
        }
    }

    /// The signed and the true distance together, for a reading that must see
    /// both which side of the carrier a point is on and how far past a patch's
    /// edge it lies. `foot` seeds a patch's projection and carries its foot out.
    pub(super) fn signed_and_true_distance(
        &self,
        point: Vec3,
        foot: &mut Option<(f64, f64)>,
    ) -> Result<(f64, f64), KernelRefusal> {
        self.distance(point, foot).map(|(signed, _, distance)| (signed, distance))
    }

    /// The signed distance alone, for root enumeration along a curve.
    pub(super) fn signed_distance(&self, point: Vec3) -> Result<f64, KernelRefusal> {
        let mut foot = None;
        self.distance(point, &mut foot).map(|(signed, _, _)| signed)
    }

    /// The unit normal of the carrier at (or nearest) `point`.
    pub(super) fn normal_at(&self, point: Vec3) -> Result<Vec3, KernelRefusal> {
        let mut foot = None;
        self.distance(point, &mut foot).map(|(_, gradient, _)| gradient)
    }
}

/// What a triple-point solve is asked to respect.
#[derive(Clone, Copy, Debug)]
pub(super) struct TriplePolicy {
    /// The model tolerance: the accuracy each carrier is known to.
    pub(super) tolerance: f64,
    /// How far the solution may lie from the seed.
    pub(super) reach: f64,
    /// The largest first-order movement of the point under a model-tolerance
    /// movement of the carriers that the caller accepts.
    pub(super) position_bar: f64,
}

/// A converged triple point and what it is worth.
#[derive(Clone, Copy, Debug)]
pub(super) struct TriplePoint {
    pub(super) point: Vec3,
    pub(super) iterations: usize,
    /// The largest distance from `point` to any of the three carriers.
    pub(super) residual: f64,
    /// The smallest singular value of the unit-normal matrix at `point`.
    pub(super) sigma_min: f64,
    /// `√3·residual/σ_min`: first-order distance to the exact triple point.
    pub(super) error_bound: f64,
    /// `√3·tolerance/σ_min`: first-order movement of the triple point when each
    /// carrier moves by the model tolerance.
    pub(super) sensitivity: f64,
    /// How far the solution lies from the seed.
    pub(super) seed_distance: f64,
}

/// Why a triple-point solve declined, with what it measured.
#[derive(Clone, Debug)]
pub(super) enum TripleRefusal {
    /// A carrier could not be read at an iterate; `reason` keeps the reading's
    /// own refusal (a focal point, a projection that failed) and its class.
    Unreadable { carrier: usize, reason: KernelRefusal },
    /// The three normals are dependent: the carriers do not cross transversally.
    Singular { iteration: usize, sigma_min: f64 },
    /// The iterate left the caller's reach of the seed.
    Wandered { iteration: usize, distance: f64, reach: f64 },
    /// Newton did not settle within its iteration budget.
    NotConverged { iterations: usize, residual: f64, step: f64 },
    /// The point is off a fitted patch: its foot is clamped to the patch edge.
    OffPatch { carrier: usize, distance: f64 },
    /// Converged, but the carriers are too near tangent for the position bar.
    IllConditioned { sigma_min: f64, sensitivity: f64, bar: f64 },
}

impl TripleRefusal {
    pub(super) fn describe(&self) -> String {
        match self {
            Self::Unreadable { carrier, reason } => {
                format!("carrier {carrier} could not be read ({reason})")
            }
            Self::Singular { iteration, sigma_min } => format!(
                "the three carriers' normals are dependent at iteration {iteration} \
                 (σ_min {sigma_min:.3e})"
            ),
            Self::Wandered {
                iteration,
                distance,
                reach,
            } => format!(
                "the solve left the seed's neighbourhood at iteration {iteration} \
                 ({distance:.3e} from the seed, reach {reach:.3e})"
            ),
            Self::NotConverged {
                iterations,
                residual,
                step,
            } => format!(
                "the solve did not converge in {iterations} iterations (residual \
                 {residual:.3e}, last step {step:.3e})"
            ),
            Self::OffPatch { carrier, distance } => format!(
                "the point lies {distance:.3e} off fitted carrier {carrier}, beyond its patch"
            ),
            Self::IllConditioned {
                sigma_min,
                sensitivity,
                bar,
            } => format!(
                "the carriers are too near tangent to place the point: σ_min {sigma_min:.3e} \
                 moves it {sensitivity:.3e} per model tolerance, over the bar {bar:.3e}"
            ),
        }
    }

    /// This refusal as a [`KernelRefusal`], its text rewritten by `wrap` (the
    /// caller's own sentence around [`Self::describe`]) and its class read off
    /// the variant, at the origin: an unreadable carrier keeps the reading's
    /// class; dependent normals and near-tangent carriers are ill-posed (the
    /// carriers do not pin one point within the bar); a solve that wandered or
    /// ran out of iterations did not converge; a point off a fitted patch is a
    /// deferral, since a patch is not read past its own domain.
    pub(super) fn into_refusal(self, wrap: impl FnOnce(&str) -> String) -> KernelRefusal {
        let text = wrap(&self.describe());
        match self {
            Self::Unreadable { reason, .. } => reason.with_message(|_| text),
            Self::Singular { .. } => KernelRefusal::ill_posed(KernelStage::Refine, "singular_normals", text),
            Self::Wandered { .. } => KernelRefusal::non_convergence(KernelStage::Refine, "wandered", text),
            Self::NotConverged { .. } => {
                KernelRefusal::non_convergence(KernelStage::Refine, "newton_budget", text)
            }
            Self::OffPatch { .. } => KernelRefusal::unsupported(KernelStage::Refine, "off_patch", text),
            Self::IllConditioned { .. } => {
                KernelRefusal::ill_posed(KernelStage::Refine, "ill_conditioned", text)
            }
        }
    }
}

const MAX_ITERATIONS: usize = 64;

/// The determinant of the matrix whose rows are `rows`.
fn determinant(rows: &[Vec3; 3]) -> f64 {
    rows[0].dot(rows[1].cross(rows[2]))
}

/// The smallest singular value of the matrix whose rows are `rows`.
///
/// Read as `|det|/(σ_max·σ_mid)`: the two larger eigenvalues of `JᵀJ` come out
/// of the symmetric-cubic closed form accurately, and the determinant is exact
/// to round-off, where the smallest eigenvalue itself would lose half its
/// digits.
pub(super) fn smallest_singular_value(rows: &[Vec3; 3]) -> f64 {
    let column = |index: usize| -> Vec3 {
        let pick = |row: &Vec3| match index {
            0 => row.x,
            1 => row.y,
            _ => row.z,
        };
        Vec3::new(pick(&rows[0]), pick(&rows[1]), pick(&rows[2]))
    };
    let columns = [column(0), column(1), column(2)];
    let a = |i: usize, j: usize| columns[i].dot(columns[j]);
    let (a00, a11, a22, a01, a02, a12) = (a(0, 0), a(1, 1), a(2, 2), a(0, 1), a(0, 2), a(1, 2));
    let off = a01 * a01 + a02 * a02 + a12 * a12;
    let q = (a00 + a11 + a22) / 3.0;
    let spread = (a00 - q).powi(2) + (a11 - q).powi(2) + (a22 - q).powi(2) + 2.0 * off;
    let (largest, middle) = if spread <= 0.0 {
        (q, q)
    } else {
        let p = (spread / 6.0).sqrt();
        let b = |value: f64| value / p;
        let (b00, b11, b22) = (b(a00 - q), b(a11 - q), b(a22 - q));
        let (b01, b02, b12) = (b(a01), b(a02), b(a12));
        let half_det = 0.5
            * (b00 * (b11 * b22 - b12 * b12) - b01 * (b01 * b22 - b12 * b02)
                + b02 * (b01 * b12 - b11 * b02));
        let phi = half_det.clamp(-1.0, 1.0).acos() / 3.0;
        let largest = q + 2.0 * p * phi.cos();
        let smallest = q + 2.0 * p * (phi + 2.0 * std::f64::consts::FRAC_PI_3).cos();
        (largest, 3.0 * q - largest - smallest)
    };
    let product = (largest.max(0.0) * middle.max(0.0)).sqrt();
    if product <= 0.0 {
        return 0.0;
    }
    determinant(rows).abs() / product
}

/// Solve for the point on all three carriers nearest `seed`.
pub(super) fn solve_triple_point(
    carriers: [&TripleCarrier; 3],
    seed: Vec3,
    policy: &TriplePolicy,
) -> Result<TriplePoint, TripleRefusal> {
    let mut point = seed;
    let mut feet: [Option<(f64, f64)>; 3] = [None; 3];
    let step_floor = (policy.tolerance * 1e-4).max(1e-15 * (1.0 + seed.length()));
    let mut last_step = f64::INFINITY;
    for iteration in 0..MAX_ITERATIONS {
        let mut signed = [0.0f64; 3];
        let mut rows = [Vec3::default(); 3];
        for index in 0..3 {
            let (distance, gradient, _) = carriers[index]
                .distance(point, &mut feet[index])
                .map_err(|reason| TripleRefusal::Unreadable {
                    carrier: index,
                    reason,
                })?;
            signed[index] = distance;
            rows[index] = gradient;
        }
        let det = determinant(&rows);
        if det.abs() <= 1e-14 {
            return Err(TripleRefusal::Singular {
                iteration,
                sigma_min: smallest_singular_value(&rows),
            });
        }
        // Cramer's rule on J·δ = −f.
        let rhs = Vec3::new(-signed[0], -signed[1], -signed[2]);
        let replace = |column: usize| -> [Vec3; 3] {
            let mut replaced = rows;
            for (row, value) in replaced.iter_mut().zip([rhs.x, rhs.y, rhs.z]) {
                match column {
                    0 => row.x = value,
                    1 => row.y = value,
                    _ => row.z = value,
                }
            }
            replaced
        };
        let step = Vec3::new(
            determinant(&replace(0)) / det,
            determinant(&replace(1)) / det,
            determinant(&replace(2)) / det,
        );
        point = point.add(step);
        last_step = step.length();
        let wandered = point.sub(seed).length();
        if !(wandered <= policy.reach) {
            return Err(TripleRefusal::Wandered {
                iteration,
                distance: wandered,
                reach: policy.reach,
            });
        }
        if last_step <= step_floor {
            return finish(carriers, point, seed, iteration + 1, &mut feet, policy);
        }
    }
    let mut residual = 0.0f64;
    for index in 0..3 {
        if let Ok((_, _, distance)) = carriers[index].distance(point, &mut feet[index]) {
            residual = residual.max(distance);
        }
    }
    Err(TripleRefusal::NotConverged {
        iterations: MAX_ITERATIONS,
        residual,
        step: last_step,
    })
}

/// Measure a converged iterate and apply the conditioning bar.
fn finish(
    carriers: [&TripleCarrier; 3],
    point: Vec3,
    seed: Vec3,
    iterations: usize,
    feet: &mut [Option<(f64, f64)>; 3],
    policy: &TriplePolicy,
) -> Result<TriplePoint, TripleRefusal> {
    let mut residual = 0.0f64;
    let mut rows = [Vec3::default(); 3];
    for index in 0..3 {
        let (_, gradient, distance) = carriers[index]
            .distance(point, &mut feet[index])
            .map_err(|reason| TripleRefusal::Unreadable {
                carrier: index,
                reason,
            })?;
        // A patch whose foot is clamped reads a small SIGNED distance along
        // its tangent plane while the point itself lies off the patch.
        if matches!(carriers[index], TripleCarrier::Patch(_)) && distance > policy.tolerance {
            return Err(TripleRefusal::OffPatch {
                carrier: index,
                distance,
            });
        }
        residual = residual.max(distance);
        rows[index] = gradient;
    }
    let sigma_min = smallest_singular_value(&rows);
    if !(sigma_min > 0.0) {
        return Err(TripleRefusal::Singular {
            iteration: iterations,
            sigma_min,
        });
    }
    let sensitivity = 3f64.sqrt() * policy.tolerance / sigma_min;
    if sensitivity > policy.position_bar {
        return Err(TripleRefusal::IllConditioned {
            sigma_min,
            sensitivity,
            bar: policy.position_bar,
        });
    }
    Ok(TriplePoint {
        point,
        iterations,
        residual,
        sigma_min,
        error_bound: 3f64.sqrt() * residual / sigma_min,
        sensitivity,
        seed_distance: point.sub(seed).length(),
    })
}

/// Every point where `curve` crosses `carrier` over `[t0, t1]` and within
/// `reach` of `center`: sampled sign changes of the signed distance, each
/// refined by bisection. A tangential touch (no sign change) is not listed.
pub(super) fn carrier_crossings(
    curve: &NurbsCurve,
    range: [f64; 2],
    carrier: &TripleCarrier,
    center: Vec3,
    reach: f64,
) -> Vec<Vec3> {
    const SAMPLES: usize = 256;
    let [t0, t1] = range;
    let height = |t: f64| -> Option<(f64, Vec3)> {
        let point = curve.evaluate(t).ok()?;
        carrier.signed_distance(point).ok().map(|value| (value, point))
    };
    let mut crossings: Vec<Vec3> = Vec::new();
    let Some((mut previous_h, _)) = height(t0) else {
        return crossings;
    };
    let mut previous_t = t0;
    for index in 1..=SAMPLES {
        let t = t0 + (t1 - t0) * index as f64 / SAMPLES as f64;
        let Some((h, _)) = height(t) else {
            previous_t = t;
            continue;
        };
        if previous_h * h <= 0.0 && !(previous_h == 0.0 && index > 1) {
            let (mut low, mut high, mut low_h) = (previous_t, t, previous_h);
            for _ in 0..80 {
                let mid = 0.5 * (low + high);
                let Some((mid_h, _)) = height(mid) else {
                    break;
                };
                if low_h * mid_h <= 0.0 {
                    high = mid;
                } else {
                    low = mid;
                    low_h = mid_h;
                }
            }
            if let Ok(point) = curve.evaluate(0.5 * (low + high)) {
                if point.sub(center).length() <= reach {
                    crossings.push(point);
                }
            }
        }
        previous_t = t;
        previous_h = h;
    }
    crossings
}

/// The kind names of three carriers, for a refusal.
pub(super) fn carrier_kinds(carriers: [&TripleCarrier; 3]) -> String {
    format!(
        "{} × {} × {}",
        carrier_kind(carriers[0]),
        carrier_kind(carriers[1]),
        carrier_kind(carriers[2])
    )
}
