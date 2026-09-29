//! TANGENT CONTACT CLASSIFICATION — is a tangency inside the trims an
//! isolated NODE, or an EXTENDED contact?
//!
//! Where two carriers touch with parallel normals, the intersection is no
//! longer a manifold curve and the marcher's step direction `n_a × n_b`
//! vanishes.  Two entirely different shapes hide behind that one symptom, and
//! the imprint's ability to handle them is entirely different:
//!
//! * **An isolated node.**  Two transverse branches CROSS at the tangency —
//!   the classic equal-radius pair, whose two sections meet at `node ±` the
//!   common perpendicular.  The section is still a curve everywhere except one
//!   point, the 2D arrangement carves the pinch on both faces, and
//!   `fragment::loops` derives the sub-edges each region needs.  The imprint
//!   ALREADY assembles this (two crossing equal-radius cylinders build exact
//!   against Steinmetz), so it must not be refused.
//!
//! * **An extended contact.**  The carriers are tangent along a whole CURVE —
//!   a sphere inscribed in its cylinder, or the G1 join where an analytic
//!   torus elbow meets its own-radius arm.  There is no section curve to
//!   imprint at all: the contact is a shared smooth EDGE the imprint has no
//!   representation for, and pushing such a pair through the marcher yields a
//!   torn shell (measured: non-integral genus one operand order, five open
//!   edges the other).  That class must be refused, and named.
//!
//! # Why this is a RANK question and not a tolerance
//!
//! Write both carriers as graphs over their common tangent plane at the
//! contact.  Their difference `g = f_a − f_b` has a critical point there, and
//! its Hessian is `H = II_a − II_b`, the difference of the two second
//! fundamental forms measured against the SAME normal.  The local structure of
//! the intersection is exactly the sign structure of `H`:
//!
//! | `H` | contact | measured |
//! |---|---|---|
//! | two eigenvalues of opposite sign | Morse saddle: two branches crossing | equal-radius cylinders `±0.5`, ratio `1.0`; equal-radius tori `±0.4635`, ratio `1.0` |
//! | rank-deficient | the contact extends along `H`'s null direction | sphere in its cylinder `(−1.1e-16, −0.5)`; G1 elbow/arm `(3.9e-33, −3.9e-33)` against a curvature scale of `0.5` |
//!
//! Those are FIFTEEN AND THIRTY orders of separation, because an extended
//! contact makes the null eigenvalue exactly zero by construction — both
//! surfaces carry the same contact curve, so they agree to second order along
//! it.  [`NODE_RANK_FLOOR`] therefore separates a rank-2 matrix from a
//! rank-deficient one; it is NOT a band, and no case sits near it.
//!
//! The separation only exists AT the contact.  At the raw witness point the
//! caller hands us — a marched sample merely within the transverse-seed
//! angular gate — the G1 elbow reads `0.039` against the torus pair's `0.695`,
//! eighteen-fold, which would be a band and would have to be tuned.  So the
//! witness is REFINED onto `{S_a = S_b, n_a × n_b = 0}` first, and only the
//! refined point is classified.  Refinement before classification is the whole
//! design.

use super::*;
use crate::{KernelRefusal, KernelStage, OrRefuse};

/// Smallest `min|λ| / κ` of `H = II_a − II_b` (against the local curvature
/// scale `κ`) at which a contact counts as an isolated NODE rather than an
/// extended tangency.  See the module docs: genuine nodes measure `0.695` and
/// `1.0`, extended contacts `1e-15` and `1e-32`, so any value across several
/// decades gives the same answer. This is a rank test, not a tolerance band.
const NODE_RANK_FLOOR: f64 = 1e-6;

/// Gauss-Newton iterations refining a witness onto the contact.  The system is
/// consistent at a real contact and the step is a full least-squares one, so a
/// witness inside the transverse-seed gate reaches round-off in well under this
/// many; the budget only bounds a witness that was never near a contact.
const REFINE_ITERATIONS: usize = 60;

/// Relative parameter step below which the refinement cannot move further.
const PARAMETER_STALL: f64 = 1e-12;

/// Relative residual improvement that counts as progress.
const RESIDUAL_PROGRESS: f64 = 1e-3;

/// Consecutive iterations without residual progress that end the refinement.
const REFINE_PATIENCE: usize = 4;

/// Levenberg diagonal added to the normal equations, relative to their largest
/// diagonal entry. Large enough to clear `solve_small`'s `1e-13` pivot floor on
/// the rank-deficient system an extended contact produces, small enough that a
/// well-conditioned node's step is unchanged to nine figures.
const LEVENBERG_DAMPING: f64 = 1e-10;

/// Relative step for the finite-difference Jacobian, in each surface's own
/// parameter span.  The residual is smooth in the parameters, so a central
/// difference at this step resolves the Jacobian far better than the
/// refinement needs.
const JACOBIAN_STEP: f64 = 1e-7;

/// How a tangency inside both trims is shaped.
#[derive(Clone, Copy, Debug)]
pub(super) struct TangentContact {
    /// The refined contact point.
    pub point: Vec3,
    /// `min|λ| / κ` of the height-difference Hessian there.
    pub rank_ratio: f64,
    /// `|n̂_a × n̂_b|` actually achieved by the refinement — the measurement's
    /// own error bar. See [`TangentContact::is_isolated_node`].
    pub tangency_residual: f64,
    /// `max|λ| / κ`: how much the two carriers' curvatures differ at all,
    /// relative to the curvature they have.
    pub curvature_contrast: f64,
    /// The two eigenvalues have opposite signs: two branches cross.
    pub saddle: bool,
}

impl TangentContact {
    /// An isolated node the arrangement can carve — the section is a curve
    /// everywhere but this one point.
    ///
    /// This is a FIRST FILTER, not the whole verdict. It answers cleanly where
    /// the contact is exactly rank-deficient — a sphere inscribed in its own
    /// cylinder reads `2.2e-16` — and it cannot answer at a third-order
    /// singularity, where the transverse branch is TANGENT to the contact curve
    /// and every second-order quantity vanishes together. The G1 elbow's
    /// witness is always such a cusp: the clipped run IS the transverse branch
    /// `x = 6 − y²/24`, and its parallel-normal samples are exactly where that
    /// branch kisses the junction circle. There `|n̂_a × n̂_b| = O(y²)` while
    /// `min|λ| = O(y)`, so refining harder makes the contact look MORE like a
    /// node, not less — measured at four successive stopping rules. A pair this
    /// filter admits wrongly is caught downstream instead, by
    /// `csg::boolean`'s tangent-node attribution: the imprint records every
    /// node it admits, and an assembly that then tears is reported against the
    /// node rather than as an anonymous degeneracy.
    pub(super) fn is_isolated_node(&self) -> bool {
        self.saddle && self.rank_ratio >= NODE_RANK_FLOOR
    }

    /// What the contact is, for a refusal message: its shape, its place, and
    /// the number the verdict was read off.
    pub(super) fn describe(&self) -> String {
        let where_ = format!(
            "({:.6},{:.6},{:.6})",
            self.point.x, self.point.y, self.point.z
        );
        if self.rank_ratio < NODE_RANK_FLOOR {
            format!(
                "the contact at {where_} is EXTENDED, not an isolated node — the two carriers \
                 agree to second order along one direction there (height-difference Hessian \
                 rank ratio {:.3e}, curvature contrast {:.3e}, resolved to {:.3e}), so they \
                 touch along a CURVE and there is no section to imprint",
                self.rank_ratio, self.curvature_contrast, self.tangency_residual
            )
        } else {
            format!(
                "the carriers touch at {where_} without crossing (height-difference Hessian is \
                 definite, rank ratio {:.3e}), so the transverse section found elsewhere does \
                 not pass through it",
                self.rank_ratio
            )
        }
    }
}

/// Parameters of one point of the contact, on both carriers.
#[derive(Clone, Copy)]
struct ContactParameters {
    ua: f64,
    va: f64,
    ub: f64,
    vb: f64,
}

struct SurfaceDomain {
    u0: f64,
    u1: f64,
    v0: f64,
    v1: f64,
}

fn domain_of(surface: &NurbsSurface) -> Result<SurfaceDomain, KernelRefusal> {
    let [u0, u1] = surface
        .domain_u()
        .or_refuse(KernelStage::Intersect, "domain_u")?;
    let [v0, v1] = surface
        .domain_v()
        .or_refuse(KernelStage::Intersect, "domain_v")?;
    Ok(SurfaceDomain { u0, u1, v0, v1 })
}

/// An orthonormal basis of the plane perpendicular to `normal`.
fn tangent_basis(normal: Vec3) -> Result<(Vec3, Vec3), KernelRefusal> {
    let first = normal
        .perpendicular()
        .or_refuse(KernelStage::Intersect, "perpendicular")?;
    let second = normal
        .cross(first)
        .normalized()
        .or_refuse(KernelStage::Intersect, "normalized")?;
    Ok((first, second))
}

/// The five-component contact residual at `parameters`: the 3D gap between the
/// two carriers, then the two independent components of `n̂_a × n̂_b` (which is
/// perpendicular to `n̂_a`, so its components in `n̂_a`'s tangent basis are the
/// whole of it).  Zero exactly at a tangential contact.
fn contact_residual(
    first: &NurbsSurface,
    second: &NurbsSurface,
    parameters: ContactParameters,
) -> Option<[f64; 5]> {
    let point_a = first.evaluate(parameters.ua, parameters.va).ok()?;
    let point_b = second.evaluate(parameters.ub, parameters.vb).ok()?;
    let normal_a = first.normal(parameters.ua, parameters.va).ok()?;
    let normal_b = second.normal(parameters.ub, parameters.vb).ok()?;
    let (basis_u, basis_v) = tangent_basis(normal_a).ok()?;
    let cross = normal_a.cross(normal_b);
    let gap = point_a.sub(point_b);
    Some([
        gap.x,
        gap.y,
        gap.z,
        cross.dot(basis_u),
        cross.dot(basis_v),
    ])
}

/// Refine `seed` onto `{S_a = S_b, n̂_a × n̂_b = 0}` by Gauss-Newton on the
/// overdetermined 5×4 system.
///
/// A genuine contact makes the system CONSISTENT, so the least-squares
/// solution drives both residuals to zero: at an isolated node onto the node
/// itself, along an extended contact onto the nearest point of the contact
/// curve (any point of it answers the rank question identically).  A witness
/// that is not near a real tangency leaves a residual and is reported as
/// unrefinable, which the caller treats as "cannot classify".
///
/// # Why this iterates to ROUND-OFF and not to the model tolerance
///
/// The caller measures a RANK here, not a position, and the two want different
/// stopping rules.  Stopping at the model tolerance leaves the point a few
/// microns off the contact, and `min|λ|` of the height-difference Hessian grows
/// LINEARLY with that offset — so an extended contact stopped at `1e-6` reports
/// a rank ratio of `1e-5`, ten times the node floor, and reads as a node.
/// (Measured exactly that way: the G1 elbow refined to `y = 6.4e-5` and
/// reported `1.06e-5`.)  Driven to round-off instead, the same contact reports
/// `~1e-12` and the node still reports `1.0`, because a rank-2 Hessian's
/// eigenvalues do not care where in the neighbourhood they were sampled.
///
/// The error is therefore ONE-SIDED — residual offset can only make an extended
/// contact look more like a node, never the reverse — so a refinement that runs
/// out of iterations without stalling is reported as unrefinable rather than
/// classified on a position we did not actually resolve.
fn refine_contact(
    first: &NurbsSurface,
    second: &NurbsSurface,
    seed: Vec3,
    tolerance: f64,
) -> Result<Option<(ContactParameters, f64)>, KernelRefusal> {
    let seed_a = project_point_to_surface(first, seed)
        .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
    let seed_b = project_point_to_surface(second, seed)
        .or_refuse(KernelStage::Intersect, "project_point_to_surface")?;
    let first_domain = domain_of(first)?;
    let second_domain = domain_of(second)?;
    let spans = [
        first_domain.u1 - first_domain.u0,
        first_domain.v1 - first_domain.v0,
        second_domain.u1 - second_domain.u0,
        second_domain.v1 - second_domain.v0,
    ];
    let mut current = ContactParameters {
        ua: seed_a.u,
        va: seed_a.v,
        ub: seed_b.u,
        vb: seed_b.v,
    };
    let clamp = |parameters: ContactParameters| ContactParameters {
        ua: parameters.ua.clamp(first_domain.u0, first_domain.u1),
        va: parameters.va.clamp(first_domain.v0, first_domain.v1),
        ub: parameters.ub.clamp(second_domain.u0, second_domain.u1),
        vb: parameters.vb.clamp(second_domain.v0, second_domain.v1),
    };
    let shifted = |parameters: ContactParameters, index: usize, delta: f64| {
        let mut moved = parameters;
        match index {
            0 => moved.ua += delta,
            1 => moved.va += delta,
            2 => moved.ub += delta,
            _ => moved.vb += delta,
        }
        clamp(moved)
    };
    let mut best = current;
    let mut best_residual = f64::INFINITY;
    let mut best_tangency = f64::INFINITY;
    let mut without_progress = 0usize;
    for _ in 0..REFINE_ITERATIONS {
        let Some(residual) = contact_residual(first, second, current) else {
            return Ok(None);
        };
        // Both residual blocks are already in their natural units: the gap in
        // model length, the cross product in radians of normal disagreement.
        let gap = (residual[0] * residual[0] + residual[1] * residual[1] + residual[2] * residual[2])
            .sqrt();
        let tangency = (residual[3] * residual[3] + residual[4] * residual[4]).sqrt();
        let worst = gap.max(tangency);
        if worst < best_residual * (1.0 - RESIDUAL_PROGRESS) {
            without_progress = 0;
        } else {
            without_progress += 1;
        }
        if worst < best_residual {
            best_residual = worst;
            best_tangency = tangency;
            best = current;
        }
        // Converged as far as this system allows. Along an EXTENDED contact the
        // solution set is a curve, so the iterate keeps drifting ALONG it at
        // the level the damping amplifies round-off to — the parameters never
        // settle even though the residual has. Progress in the RESIDUAL is
        // therefore the only convergence test that means the same thing for
        // both shapes.
        if worst == 0.0 || without_progress >= REFINE_PATIENCE {
            break;
        }
        let mut jacobian = [[0.0f64; 4]; 5];
        for index in 0..4 {
            let step = spans[index] * JACOBIAN_STEP;
            if step <= 0.0 {
                return Ok(None);
            }
            let (Some(plus), Some(minus)) = (
                contact_residual(first, second, shifted(current, index, step)),
                contact_residual(first, second, shifted(current, index, -step)),
            ) else {
                return Ok(None);
            };
            for row in 0..5 {
                jacobian[row][index] = (plus[row] - minus[row]) / (2.0 * step);
            }
        }
        // Normal equations of the least-squares step, JᵀJ δ = −Jᵀ F, with a
        // Levenberg diagonal.
        //
        // The damping is not a fudge, it is what makes the EXTENDED case
        // solvable at all: where the two carriers touch along a curve the
        // solution set is one-dimensional, so `J` is rank 3 and `JᵀJ` is
        // exactly singular — the very case this refinement exists to reach.
        // `Jᵀ F` is orthogonal to `null(J)` for any `F` (`(JᵀF)·v = F·(Jv) = 0`),
        // so the damped solve returns a bounded step that moves ACROSS the
        // contact and leaves the along-contact direction alone. On a
        // well-conditioned node it changes the step by its own relative size.
        let mut matrix = [[0.0f64; 4]; 4];
        let mut rhs = [0.0f64; 4];
        for row in 0..4 {
            for column in 0..4 {
                matrix[row][column] = (0..5).map(|k| jacobian[k][row] * jacobian[k][column]).sum();
            }
            rhs[row] = -(0..5).map(|k| jacobian[k][row] * residual[k]).sum::<f64>();
        }
        let diagonal = (0..4).fold(0.0f64, |largest, index| largest.max(matrix[index][index]));
        if diagonal <= 0.0 {
            return Ok(None);
        }
        for index in 0..4 {
            matrix[index][index] += LEVENBERG_DAMPING * diagonal;
        }
        let Ok(changes) = crate::fit::solve_small(matrix, rhs, 4) else {
            return Ok(None);
        };
        // Never step more than a quarter of any parameter span at once: the
        // system is ill-conditioned by construction near a tangency, and an
        // unbounded step lands on an unrelated sheet of the surface.
        let mut scale = 1.0f64;
        for index in 0..4 {
            let limit = spans[index] / 4.0;
            if changes[index].abs() > limit {
                scale = scale.min(limit / changes[index].abs());
            }
        }
        let next = clamp(ContactParameters {
            ua: current.ua + changes[0] * scale,
            va: current.va + changes[1] * scale,
            ub: current.ub + changes[2] * scale,
            vb: current.vb + changes[3] * scale,
        });
        let moved = (0..4)
            .map(|index| {
                let (a, b) = match index {
                    0 => (next.ua, current.ua),
                    1 => (next.va, current.va),
                    2 => (next.ub, current.ub),
                    _ => (next.vb, current.vb),
                };
                (a - b).abs() / spans[index]
            })
            .fold(0.0f64, f64::max);
        current = next;
        if moved <= PARAMETER_STALL {
            break;
        }
    }
    if std::env::var("BREP_DEBUG_PAIRS").is_ok() {
        eprintln!(
            "  refine_contact: best gap/tangency {best_residual:.3e} tangency {best_tangency:.3e}"
        );
    }
    Ok((best_residual <= tolerance).then_some((best, best_tangency)))
}

/// The second fundamental form of `surface` at `(u, v)`, expressed in the
/// orthonormal tangent basis `(basis_u, basis_v)` and measured against
/// `normal`.  For a surface written as a graph over that basis with a
/// stationary gradient, this IS the Hessian of the graph.
fn second_fundamental_form(
    surface: &NurbsSurface,
    u: f64,
    v: f64,
    basis_u: Vec3,
    basis_v: Vec3,
    normal: Vec3,
) -> Option<[[f64; 2]; 2]> {
    let derivatives = surface.derivatives(u, v, 2).ok()?;
    let s_u = derivatives[1][0];
    let s_v = derivatives[0][1];
    let s_uu = derivatives[2][0];
    let s_uv = derivatives[1][1];
    let s_vv = derivatives[0][2];
    // Express each basis direction in the surface's own parameter directions
    // (least squares in the tangent plane, which the basis lies in).
    let a11 = s_u.dot(s_u);
    let a12 = s_u.dot(s_v);
    let a22 = s_v.dot(s_v);
    let determinant = a11 * a22 - a12 * a12;
    if determinant.abs() <= f64::MIN_POSITIVE {
        return None;
    }
    let coefficients = |target: Vec3| -> (f64, f64) {
        let b1 = s_u.dot(target);
        let b2 = s_v.dot(target);
        (
            (b1 * a22 - b2 * a12) / determinant,
            (a11 * b2 - a12 * b1) / determinant,
        )
    };
    let along_u = coefficients(basis_u);
    let along_v = coefficients(basis_v);
    let l = s_uu.dot(normal);
    let m = s_uv.dot(normal);
    let n = s_vv.dot(normal);
    let form = |x: (f64, f64), y: (f64, f64)| l * x.0 * y.0 + m * (x.0 * y.1 + x.1 * y.0) + n * x.1 * y.1;
    Some([
        [form(along_u, along_u), form(along_u, along_v)],
        [form(along_v, along_u), form(along_v, along_v)],
    ])
}

/// Eigenvalues of a symmetric 2×2 matrix.
fn eigenvalues(matrix: [[f64; 2]; 2]) -> (f64, f64) {
    let trace = matrix[0][0] + matrix[1][1];
    let determinant = matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
    let discriminant = (trace * trace / 4.0 - determinant).max(0.0).sqrt();
    (trace / 2.0 + discriminant, trace / 2.0 - discriminant)
}

/// Spectral norm of a symmetric 2×2 matrix — the local curvature scale a
/// carrier contributes.
fn spectral_norm(matrix: [[f64; 2]; 2]) -> f64 {
    let (first, second) = eigenvalues(matrix);
    first.abs().max(second.abs())
}

/// Classify the tangential contact `witness` lies on: refine onto the contact,
/// then rank-test the height-difference Hessian there.
///
/// `None` means the witness could not be refined onto a contact at all, so
/// nothing is proven about it and the caller must keep refusing.
pub(super) fn classify_tangent_contact(
    first: &NurbsSurface,
    second: &NurbsSurface,
    witness: Vec3,
    tolerance: f64,
) -> Result<Option<TangentContact>, KernelRefusal> {
    let Some((parameters, tangency_residual)) = refine_contact(first, second, witness, tolerance)?
    else {
        return Ok(None);
    };
    let point = first
        .evaluate(parameters.ua, parameters.va)
        .or_refuse(KernelStage::Intersect, "evaluate")?;
    let normal_a = first
        .normal(parameters.ua, parameters.va)
        .or_refuse(KernelStage::Intersect, "normal")?;
    let normal_b = second
        .normal(parameters.ub, parameters.vb)
        .or_refuse(KernelStage::Intersect, "normal")?;
    // Both forms must be measured against the SAME normal, or their difference
    // is the SUM of the curvatures and every contact reads as definite.
    let orientation = if normal_a.dot(normal_b) < 0.0 { -1.0 } else { 1.0 };
    let (basis_u, basis_v) = tangent_basis(normal_a)?;
    let (Some(form_a), Some(form_b)) = (
        second_fundamental_form(first, parameters.ua, parameters.va, basis_u, basis_v, normal_a),
        second_fundamental_form(
            second,
            parameters.ub,
            parameters.vb,
            basis_u,
            basis_v,
            normal_b.scale(orientation),
        ),
    ) else {
        return Ok(None);
    };
    let mut difference = [[0.0f64; 2]; 2];
    for row in 0..2 {
        for column in 0..2 {
            difference[row][column] = form_a[row][column] - form_b[row][column];
        }
    }
    let curvature_scale = spectral_norm(form_a).max(spectral_norm(form_b));
    let (lambda_first, lambda_second) = eigenvalues(difference);
    let smallest = lambda_first.abs().min(lambda_second.abs());
    let rank_ratio = if curvature_scale > 0.0 {
        smallest / curvature_scale
    } else {
        // Two planes: no curvature anywhere, so the "contact" is coincidence,
        // which the cosurface path owns and never reaches here.
        0.0
    };
    let largest = lambda_first.abs().max(lambda_second.abs());
    Ok(Some(TangentContact {
        point,
        rank_ratio,
        tangency_residual,
        curvature_contrast: if curvature_scale > 0.0 {
            largest / curvature_scale
        } else {
            0.0
        },
        saddle: lambda_first * lambda_second < 0.0,
    }))
}

