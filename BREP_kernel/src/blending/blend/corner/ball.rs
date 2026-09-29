use super::*;

// ======================================================================
// The corner ball (§6.9.7), solved the way a station is solved
// ======================================================================
//
// Every quantity a corner closure needs is a consequence of ONE point: the
// centre C* of the radius-r ball seated in the corner, touching all N faces
// that meet there.  Writing that solve generally is what lets a corner be
// planned BEFORE anything is cut, instead of being reconstructed afterwards
// from whatever a cutter left behind.
//
// The system is the march's own tangency equation (4.9.2) with the section
// plane dropped and one row per additional face:
//
//     C* = p_j(u_j, v_j) + ρ_j · n_j(u_j, v_j)      for every face j
//
// — 2N unknowns (the surface parameters), 3(N−1) equations (every face's
// offset evaluation must land on the same centre).  N = 3 is square and has
// an isolated root; N ≥ 4 is over-determined and its least-squares RESIDUAL
// is the "is there a common ball here" test, which is why no separate
// concurrency check appears anywhere below.
//
// This is the pointwise case of the offset-intersection identity in the
// module header: `p + ρ·n` on each support, required to coincide.  Unlike
// the locus solves next door (`mixed_concave`, `mixed_curved`), a corner
// ball is a POINT, so [`crate::OffsetEvaluator`] answers it directly and no
// carrier-specific algebra is needed — planes, cylinders, cones, spheres,
// tori, revolutions and free-form patches all go through the same Newton.

/// Where the corner ball touches one of the faces meeting at the vertex.
pub(in crate::blend) struct CornerContact {
    pub(in crate::blend) face_id: u64,
    /// Converged surface parameters of the tangency point — where on the
    /// face's carrier the ball touches, which may lie OUTSIDE the face's
    /// trim (a re-entrant corner, where the ball touches the wall's plane
    /// beyond a concave edge).
    pub(in crate::blend) uv: [f64; 2],
    /// The tangency point itself — the vertex at which the two blend stripes
    /// sharing this face both stop.
    pub(in crate::blend) point: Vec3,
    /// Raw surface normal there (the march's `OffsetNormal::Raw` convention,
    /// with orientation carried by the sign of ρ).
    pub(in crate::blend) normal: Vec3,
}

/// The seated ball: its centre, and where it touches each incident face.
pub(in crate::blend) struct CornerBall {
    pub(in crate::blend) center: Vec3,
    pub(in crate::blend) contacts: Vec<CornerContact>,
    /// Worst disagreement between the per-face offset evaluations at
    /// convergence, in model units.  Zero to solver tolerance for N = 3;
    /// for N ≥ 4 a non-zero value means the N cylinders have NO common
    /// inscribed ball and the corner needs an N-sided fill instead of a
    /// spherical patch.
    pub(in crate::blend) residual: f64,
}

/// One face of the corner, with the signed offset the stripes touching it
/// were marched at.
pub(in crate::blend) struct CornerFace<'a> {
    pub(in crate::blend) face: &'a FaceRecord,
    /// Signed radius ρ: `centre = p + ρ·n_raw(p)`.  Taken from the stripe
    /// that lands on this face (`signed_radii`), never re-derived, so the
    /// ball and the stripes are guaranteed to be the same ball.
    pub(in crate::blend) rho: f64,
    /// Starting parameters — normally the sharp corner's own (u, v) on this
    /// face.
    pub(in crate::blend) seed: [f64; 2],
}

/// Damped Gauss-Newton iterations.  Matches the station solver's budget:
/// the residual is the same offset equation and the seed (the sharp corner)
/// is one radius away from the root.
const CORNER_ITERATIONS: usize = 40;

/// Offset-evaluate every face at `x` and return the per-face centres.
fn centers(faces: &[CornerFace<'_>], x: &[f64]) -> Result<Vec<Vec3>, KernelRefusal> {
    faces
        .iter()
        .enumerate()
        .map(|(index, face)| {
            blend_offset(&face.face.surface)
                .at(x[2 * index], x[2 * index + 1], face.rho)
                .map(|sample| sample.point)
                .or_refuse(KernelStage::Refine, "offset_at")
        })
        .collect()
}

/// Residual: every face's offset centre minus the FIRST face's, stacked.
/// 3(N−1) rows.
fn residual(faces: &[CornerFace<'_>], x: &[f64]) -> Result<Vec<f64>, KernelRefusal> {
    let centers = centers(faces, x)?;
    let mut rows = Vec::with_capacity(3 * (faces.len() - 1));
    for center in &centers[1..] {
        let delta = center.sub(centers[0]);
        rows.extend_from_slice(&[delta.x, delta.y, delta.z]);
    }
    Ok(rows)
}

fn norm(values: &[f64]) -> f64 {
    values.iter().fold(0.0f64, |worst, v| worst.max(v.abs()))
}

/// Seat the radius-r ball in the corner formed by `faces`.
///
/// `scale` is the march's model scale, used for the convergence bar exactly
/// as [`solve_station`] uses it (`1e-11·(1 + scale)`), so a corner solved
/// here is accurate to the same bar as the stations of the stripes that end
/// on it.
pub(in crate::blend) fn solve_corner_ball(
    faces: &[CornerFace<'_>],
    scale: f64,
) -> Result<CornerBall, KernelRefusal> {
    if faces.len() < 3 {
        return Err(KernelRefusal::input(
            KernelStage::Collect,
            "face_count",
            format!(
                "corner ball: needs at least three faces at the vertex, got {}",
                faces.len()
            ),
        ));
    }
    let unknowns = 2 * faces.len();
    let equations = 3 * (faces.len() - 1);
    let mut x: Vec<f64> = faces
        .iter()
        .flat_map(|face| [face.seed[0], face.seed[1]])
        .collect();
    let tolerance = 1e-11 * (1.0 + scale);
    let step = 1e-7;
    let mut current = residual(faces, &x)?;
    for _ in 0..CORNER_ITERATIONS {
        if norm(&current) <= tolerance {
            break;
        }
        // Finite-difference Jacobian, one column per unknown — the same
        // scheme (and the same step) as the station Newton, so a corner and
        // the stripes ending on it are conditioned alike.
        let mut jacobian = vec![vec![0.0f64; unknowns]; equations];
        for column in 0..unknowns {
            let mut probe = x.clone();
            probe[column] += step;
            let probed = residual(faces, &probe)?;
            for row in 0..equations {
                jacobian[row][column] = (probed[row] - current[row]) / step;
            }
        }
        // Normal equations JᵀJ δ = Jᵀ r.  Square for N = 3 (the exact root)
        // and least-squares for N ≥ 4 (where the residual is the answer).
        let mut normal_matrix = vec![vec![0.0f64; unknowns]; unknowns];
        let mut normal_rhs = vec![0.0f64; unknowns];
        for row in 0..equations {
            for a in 0..unknowns {
                normal_rhs[a] += jacobian[row][a] * current[row];
                for b in 0..unknowns {
                    normal_matrix[a][b] += jacobian[row][a] * jacobian[row][b];
                }
            }
        }
        let delta = fit::solve_dense(normal_matrix, normal_rhs).map_err(|error| {
            KernelRefusal::ill_posed(
                KernelStage::Refine,
                "singular_jacobian",
                format!("corner ball: Gauss-Newton is singular ({error})"),
            )
        })?;
        // Step halving on the residual norm: a corner seed sits a full radius
        // from its root, far enough that an undamped step can overshoot a
        // curved carrier's domain.
        let mut accepted = false;
        let mut damping = 1.0f64;
        for _ in 0..8 {
            let trial: Vec<f64> = x
                .iter()
                .zip(&delta)
                .map(|(value, correction)| value - damping * correction)
                .collect();
            if let Ok(trial_residual) = residual(faces, &trial) {
                if norm(&trial_residual) < norm(&current) {
                    x = trial;
                    current = trial_residual;
                    accepted = true;
                    break;
                }
            }
            damping *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    let final_residual = norm(&current);
    let centers = centers(faces, &x)?;
    let center = centers
        .iter()
        .fold(Vec3::default(), |sum, point| sum.add(*point))
        .scale(1.0 / centers.len() as f64);
    let mut contacts = Vec::with_capacity(faces.len());
    for (index, face) in faces.iter().enumerate() {
        let (u, v) = (x[2 * index], x[2 * index + 1]);
        let offset = blend_offset(&face.face.surface);
        // The root is only a corner ball if it landed on the CARRIERS.
        // `Raw` evaluates through `deriv1_extended`, which continues an open
        // direction along its boundary tangent plane rather than refusing, so
        // a configuration with no seated ball still converges — onto that
        // continuation, with a residual of zero, because the extended point
        // is a genuine root of the EXTENDED system.  At N = 3 the system is
        // square, so `final_residual` cannot distinguish the two: it is ~0
        // either way (which is why the residual test above is scoped to
        // N >= 4).  Domain membership is the only thing that can.
        //
        // The test is geometric, not a domain comparison: a plane is a
        // bounded patch here and a cylinder's axial direction is open, and
        // BOTH extend exactly, so refusing every out-of-domain tangency would
        // reject the re-entrant corners that touch a wall's carrier beyond a
        // concave edge — which `CornerContact::uv` documents and depends on.
        // Only an excursion along a direction with normal curvature is
        // fiction.  `tolerance` is the solver's own bar, so the ball is only
        // accepted where the extension cannot have moved it further than the
        // accuracy this solve claims.
        if !offset
            .on_real_carrier(u, v, tolerance)
            .or_refuse(KernelStage::Refine, "on_real_carrier")?
        {
            let [u0, u1] = face
                .face
                .surface
                .domain_u()
                .or_refuse(KernelStage::Refine, "domain")?;
            let [v0, v1] = face
                .face
                .surface
                .domain_v()
                .or_refuse(KernelStage::Refine, "domain")?;
            trace_corner_ball("REFUSED by on_real_carrier", faces, &x, final_residual, tolerance)?;
            return Err(KernelRefusal::unsupported(
                KernelStage::Refine,
                "carrier_extension",
                format!(
                    "corner ball: the tangency on face {} converged onto a fictitious EXTENSION of \
                 that carrier — (u={u:.6}, v={v:.6}) against u∈[{u0:.6}, {u1:.6}], \
                 v∈[{v0:.6}, {v1:.6}], far enough past a curved direction that the evaluator's \
                 ruled continuation has left the surface. No ball of this radius is tangent to \
                 the real face here.",
                    face.face.id
                ),
            ));
        }
        let sample = offset
            .at(u, v, face.rho)
            .or_refuse(KernelStage::Refine, "offset_at")?;
        contacts.push(CornerContact {
            face_id: face.face.id,
            uv: [u, v],
            point: sample.source,
            normal: sample.normal,
        });
    }
    trace_corner_ball("ACCEPTED", faces, &x, final_residual, tolerance)?;
    Ok(CornerBall {
        center,
        contacts,
        residual: final_residual,
    })
}

/// `BREP_DEBUG_CORNER_BALL`: one line per solve outcome recording where every
/// tangency landed relative to its support's parameter domain. Overshoot is
/// measured the way `on_real_carrier` measures it: zero along a CLOSED
/// direction (the evaluator wraps there, so an out-of-range parameter is the
/// same point, not an excursion), otherwise the signed excess past `[lo,
/// hi]`, also given as a fraction of the span. `faithful` is
/// `on_real_carrier`'s own verdict on that contact, so a reader can separate
/// a straight-direction excursion (a wall's carrier beyond a concave edge,
/// which the guard accepts by design) from a curved one. No behaviour change:
/// the line is only printed.
fn trace_corner_ball(
    verdict: &str,
    faces: &[CornerFace<'_>],
    x: &[f64],
    residual: f64,
    tolerance: f64,
) -> Result<(), KernelRefusal> {
    if std::env::var("BREP_DEBUG_CORNER_BALL").is_err() {
        return Ok(());
    }
    let mut line = format!(
        "corner ball {verdict}: n={} residual={residual:.3e} tolerance={tolerance:.3e}",
        faces.len()
    );
    for (index, face) in faces.iter().enumerate() {
        let (u, v) = (x[2 * index], x[2 * index + 1]);
        let surface = &face.face.surface;
        let [u0, u1] = surface.domain_u().or_refuse(KernelStage::Refine, "domain")?;
        let [v0, v1] = surface.domain_v().or_refuse(KernelStage::Refine, "domain")?;
        let (closed_u, closed_v) = surface
            .closed_directions()
            .or_refuse(KernelStage::Refine, "closed_directions")?;
        let out = |value: f64, lo: f64, hi: f64, closed: bool| -> f64 {
            if closed {
                0.0
            } else if value < lo {
                value - lo
            } else if value > hi {
                value - hi
            } else {
                0.0
            }
        };
        let du = out(u, u0, u1, closed_u);
        let dv = out(v, v0, v1, closed_v);
        let span = |lo: f64, hi: f64| (hi - lo).abs().max(f64::MIN_POSITIVE);
        let faithful = blend_offset(surface)
            .on_real_carrier(u, v, tolerance)
            .or_refuse(KernelStage::Refine, "on_real_carrier")?;
        line.push_str(&format!(
            " | face={} u={u:.6} v={v:.6} du=[{u0:.6},{u1:.6}]{} dv=[{v0:.6},{v1:.6}]{} \
             out_u={du:.3e} out_v={dv:.3e} out_frac={:.3e} outside={} faithful={faithful}",
            face.face.id,
            if closed_u { "c" } else { "" },
            if closed_v { "c" } else { "" },
            (du.abs() / span(u0, u1)).max(dv.abs() / span(v0, v1)),
            du != 0.0 || dv != 0.0,
        ));
    }
    eprintln!("{line}");
    Ok(())
}

/// Build the solver input for the faces meeting at `corner`, seeding each
/// from the corner's own projection onto that face.
///
/// `faces` are the incident carriers in any order; `rho` is looked up per
/// face id from the stripes that end at this corner, so the ball is offset
/// on the same side as every stripe it will close.
pub(in crate::blend) fn corner_faces<'a>(
    faces: &[&'a FaceRecord],
    rho_of: &dyn Fn(u64) -> Option<f64>,
    corner: Vec3,
) -> Result<Vec<CornerFace<'a>>, KernelRefusal> {
    let mut prepared = Vec::with_capacity(faces.len());
    for face in faces {
        let rho = rho_of(face.id).ok_or_else(|| {
            KernelRefusal::internal(
                KernelStage::Collect,
                "signed_radius",
                format!(
                    "corner ball: no stripe supplies a signed radius for face {}",
                    face.id
                ),
            )
        })?;
        let projection = crate::project_point_to_surface(&face.surface, corner)
            .or_refuse(KernelStage::Collect, "project")?;
        prepared.push(CornerFace {
            face,
            rho,
            seed: [projection.u, projection.v],
        });
    }
    Ok(prepared)
}
