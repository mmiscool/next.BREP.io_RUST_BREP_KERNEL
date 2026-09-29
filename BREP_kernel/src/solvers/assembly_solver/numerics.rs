use crate::solver_linear_algebra::{row_reduce, solve_spd};
use super::*;

// ---------------------------------------------------------------------------
// Residual / Jacobian evaluation over the increment vector
// ---------------------------------------------------------------------------

/// One 6-column block of solver unknowns. A singleton group reproduces the
/// v1 per-body update exactly; a multi-member group (a rigid cluster, §7.5)
/// moves all its members with ONE rigid motion: rotation about the anchor
/// member's origin plus a shared translation.
#[derive(Clone, Debug)]
pub(super) struct SolveGroup {
    /// Body indices moved by this block; `members[0]` is the anchor.
    pub(super) members: Vec<usize>,
}

pub(super) fn singleton_groups(bodies: &[usize]) -> Vec<SolveGroup> {
    bodies
        .iter()
        .map(|&b| SolveGroup { members: vec![b] })
        .collect()
}

/// Effective poses under a per-group increment `delta` (blocks of
/// `[δθ(3), δt(3)]`). `delta = 0` reproduces `base` bit-exactly.
fn effective_poses(base: &[Pose], groups: &[SolveGroup], delta: &[f64], out: &mut Vec<Pose>) {
    out.clear();
    out.extend_from_slice(base);
    for (k, group) in groups.iter().enumerate() {
        let b = 6 * k;
        let dq = quat_from_rotvec(Vec3::new(delta[b], delta[b + 1], delta[b + 2]));
        let dt = Vec3::new(delta[b + 3], delta[b + 4], delta[b + 5]);
        if let [only] = group.members[..] {
            let pose = &mut out[only];
            pose.q = quat_mul(dq, pose.q);
            pose.t = pose.t.add(dt);
        } else {
            // Rigid motion of the whole group about the anchor body's origin.
            let anchor = base[group.members[0]].t;
            for &bi in &group.members {
                let pose = &mut out[bi];
                pose.q = quat_mul(dq, pose.q);
                pose.t = anchor.add(quat_rotate(dq, pose.t.sub(anchor))).add(dt);
            }
        }
    }
}

pub(super) fn eval_residuals(
    atoms: &[Atom],
    base: &[Pose],
    groups: &[SolveGroup],
    delta: &[f64],
    pose_scratch: &mut Vec<Pose>,
    out: &mut Vec<f64>,
) {
    effective_poses(base, groups, delta, pose_scratch);
    out.clear();
    for atom in atoms {
        atom.eval(pose_scratch, out);
    }
}

/// Central-difference Jacobian of the stacked residual with respect to the
/// increment vector, evaluated at `delta = 0` around `base`. Mirrors
/// `sketch_solver::finite_diff_jacobian`, with per-column steps (radians for
/// rotation columns, scale-relative lengths for translation columns).
pub(super) fn finite_diff_jacobian(
    atoms: &[Atom],
    base: &[Pose],
    groups: &[SolveGroup],
    col_step: &[f64],
    m: usize,
) -> Vec<Vec<f64>> {
    let n = col_step.len();
    let mut jac = vec![vec![0.0f64; n]; m];
    if n == 0 || m == 0 {
        return jac;
    }
    let mut delta = vec![0.0f64; n];
    let mut poses: Vec<Pose> = Vec::with_capacity(base.len());
    let mut rp: Vec<f64> = Vec::with_capacity(m);
    let mut rm: Vec<f64> = Vec::with_capacity(m);
    for col in 0..n {
        let h = col_step[col];
        delta[col] = h;
        eval_residuals(atoms, base, groups, &delta, &mut poses, &mut rp);
        delta[col] = -h;
        eval_residuals(atoms, base, groups, &delta, &mut poses, &mut rm);
        delta[col] = 0.0;
        let inv = 1.0 / (2.0 * h);
        for i in 0..m {
            jac[i][col] = (rp[i] - rm[i]) * inv;
        }
    }
    jac
}

/// Per-column finite-difference steps for `groups` (radians on rotation
/// columns, scale-relative lengths on translation columns).
pub(super) fn column_steps(group_count: usize, scale: f64) -> Vec<f64> {
    let mut col_step = vec![0.0f64; 6 * group_count];
    for k in 0..group_count {
        for c in 0..3 {
            col_step[6 * k + c] = ASM_ROT_FD_STEP;
            col_step[6 * k + 3 + c] = ASM_TRANS_FD_STEP * scale;
        }
    }
    col_step
}

// ---------------------------------------------------------------------------
// Matrix rank
// ---------------------------------------------------------------------------

/// Numerical rank of a dense `m x n` matrix via Gauss-Jordan elimination with
/// partial pivoting (the null-space basis is not needed here).
pub(super) fn numerical_rank(mut a: Vec<Vec<f64>>, m: usize, n: usize) -> usize {
    if n == 0 || m == 0 {
        return 0;
    }
    let mut maxabs = 0.0f64;
    for row in &a {
        for &v in row {
            maxabs = maxabs.max(v.abs());
        }
    }
    let tol = if maxabs > 0.0 {
        ASM_RANK_REL_TOL * maxabs
    } else {
        ASM_RANK_REL_TOL
    };
    row_reduce(&mut a, m, n, tol, |_| {})
}

// ---------------------------------------------------------------------------
// Levenberg-Marquardt core (shared by every strategy and sub-solve)
// ---------------------------------------------------------------------------

/// Work done by one LM run.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct LmRun {
    /// Accepted LM steps.
    pub(super) iterations: usize,
    /// Scalar residual rows evaluated (step trials + FD Jacobian columns).
    pub(super) rows: usize,
}

impl LmRun {
    pub(super) fn absorb(&mut self, other: LmRun) {
        self.iterations += other.iterations;
        self.rows += other.rows;
    }
}

/// Levenberg-Marquardt on the damped normal equations over the `groups`
/// increment vector: the §7.3 loop, generalized from per-body blocks to
/// solve groups. `poses` holds ALL body poses; only members of `groups`
/// move. On return `poses` holds the best (max-residual) iterate found.
pub(super) fn lm_core(
    atoms: &[Atom],
    poses: &mut Vec<Pose>,
    groups: &[SolveGroup],
    scale: f64,
    tight: f64,
    max_iterations: usize,
) -> Result<LmRun, String> {
    let n = 6 * groups.len();
    let m: usize = atoms.iter().map(Atom::rows).sum();
    let step_tol = ASM_STEP_REL_TOL * scale;
    let col_step = column_steps(groups.len(), scale);
    let mut run = LmRun::default();

    let max_abs = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
    let sum_sq = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc + x * x);

    let zeros = vec![0.0f64; n];
    let mut pose_scratch: Vec<Pose> = Vec::with_capacity(poses.len());
    let mut r: Vec<f64> = Vec::with_capacity(m);
    eval_residuals(atoms, poses, groups, &zeros, &mut pose_scratch, &mut r);
    run.rows += m;
    if !max_abs(&r).is_finite() {
        return Err("assembly solve: non-finite mate residual at the initial poses".into());
    }

    let mut s = sum_sq(&r);
    let mut best_poses = poses.clone();
    let mut best_max = max_abs(&r);
    let mut lambda = -1.0f64;

    // Levenberg-Marquardt with re-centred pose increments (mirrors
    // sketch_solver::newton_polish's damping schedule and guards).
    if n > 0 && m > 0 {
        'outer: for _iter in 0..max_iterations {
            if best_max <= tight {
                break;
            }
            let jac = finite_diff_jacobian(atoms, poses, groups, &col_step, m);
            run.rows += 2 * n * m;
            // Normal equations: H = JᵀJ (n x n), g = Jᵀ r (n).
            let mut h = vec![vec![0.0f64; n]; n];
            let mut g = vec![0.0f64; n];
            for i in 0..m {
                let row = &jac[i];
                let ri = r[i];
                for a in 0..n {
                    let ja = row[a];
                    if ja == 0.0 {
                        continue;
                    }
                    g[a] += ja * ri;
                    for b in a..n {
                        h[a][b] += ja * row[b];
                    }
                }
            }
            for a in 0..n {
                for b in (a + 1)..n {
                    h[b][a] = h[a][b];
                }
            }
            let mut max_diag = 0.0f64;
            for a in 0..n {
                max_diag = max_diag.max(h[a][a]);
            }
            if !(max_diag > 0.0) {
                break; // No sensitivity to any pose increment.
            }
            if lambda < 0.0 {
                lambda = 1e-9 * max_diag;
            }
            let lambda_ceiling = 1e12 * max_diag;

            let mut stepped = false;
            let mut step_small = false;
            for _try in 0..ASM_MAX_DAMPING {
                let mut a_mat = h.clone();
                for d in 0..n {
                    a_mat[d][d] += lambda;
                }
                let mut delta: Vec<f64> = g.iter().map(|v| -v).collect();
                if !solve_spd(&mut a_mat, &mut delta) {
                    lambda *= ASM_LAMBDA_UP;
                    if lambda > lambda_ceiling {
                        break;
                    }
                    continue;
                }
                let mut r_trial: Vec<f64> = Vec::with_capacity(m);
                eval_residuals(
                    atoms,
                    poses,
                    groups,
                    &delta,
                    &mut pose_scratch,
                    &mut r_trial,
                );
                run.rows += m;
                let s_trial = sum_sq(&r_trial);
                if s_trial.is_finite() && s_trial < s {
                    // Fold the increment into the base poses and re-centre.
                    let step_norm = sum_sq(&delta).sqrt();
                    effective_poses(poses, groups, &delta, &mut pose_scratch);
                    for (bi, pose) in pose_scratch.iter().enumerate() {
                        poses[bi] = Pose {
                            q: quat_normalize(pose.q)
                                .map_err(|e| format!("assembly solve: {e}"))?,
                            t: pose.t,
                        };
                    }
                    eval_residuals(atoms, poses, groups, &zeros, &mut pose_scratch, &mut r);
                    run.rows += m;
                    s = sum_sq(&r);
                    let m_now = max_abs(&r);
                    if m_now < best_max {
                        best_max = m_now;
                        best_poses.copy_from_slice(poses);
                    }
                    lambda = (lambda * ASM_LAMBDA_DOWN).max(1e-12 * max_diag);
                    run.iterations += 1;
                    stepped = true;
                    step_small = step_norm <= step_tol;
                    break;
                }
                lambda *= ASM_LAMBDA_UP;
                if lambda > lambda_ceiling {
                    break;
                }
            }
            if !stepped || step_small {
                break 'outer; // Converged, stalled, or damping exhausted.
            }
        }
    }

    poses.copy_from_slice(&best_poses);
    Ok(run)
}

// ---------------------------------------------------------------------------
// Problem preparation (validation, atom expansion, scale) and diagnostics
// ---------------------------------------------------------------------------

pub(super) struct Prepared {
    /// Normalized initial poses (grounded bodies keep these forever).
    pub(super) base: Vec<Pose>,
    /// Indices of the non-fixed bodies, ascending.
    pub(super) movable: Vec<usize>,
    pub(super) atoms: Vec<Atom>,
    /// Atom index → mate index.
    pub(super) atom_mate: Vec<usize>,
    /// Residual row → mate index.
    row_mate: Vec<usize>,
    /// Total residual rows.
    pub(super) m: usize,
    /// Total effective rows (per-atom theoretical rank), for redundancy.
    effective_rows: usize,
    /// Characteristic model length scale.
    pub(super) scale: f64,
    /// Absolute convergence target (`tolerance * scale`).
    pub(super) tight: f64,
}

pub(super) fn prepare(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    options: &AssemblySolveOptions,
) -> Result<Prepared, String> {
    let mut base: Vec<Pose> = Vec::with_capacity(bodies.len());
    for body in bodies {
        let q = quat_normalize(body.rotation).map_err(|e| format!("body '{}': {e}", body.id))?;
        let t = body.translation;
        if !(t[0].is_finite() && t[1].is_finite() && t[2].is_finite()) {
            return Err(format!("body '{}': translation must be finite", body.id));
        }
        base.push(Pose { q, t: vec3(t) });
    }
    let movable: Vec<usize> = (0..bodies.len()).filter(|&i| !bodies[i].fixed).collect();

    let (atoms, atom_mate) = expand_mates(bodies, mates)?;
    let m: usize = atoms.iter().map(Atom::rows).sum();
    let effective_rows: usize = atoms.iter().map(Atom::eff_rows).sum();
    let mut row_mate: Vec<usize> = Vec::with_capacity(m);
    for (ai, atom) in atoms.iter().enumerate() {
        for _ in 0..atom.rows() {
            row_mate.push(atom_mate[ai]);
        }
    }

    // Characteristic length scale for finite-difference steps and tolerances.
    let mut scale = 1.0f64;
    for pose in &base {
        scale = scale
            .max(pose.t.x.abs())
            .max(pose.t.y.abs())
            .max(pose.t.z.abs());
    }
    for atom in &atoms {
        let mut track = |p: Vec3| {
            scale = scale.max(p.x.abs()).max(p.y.abs()).max(p.z.abs());
        };
        match *atom {
            Atom::PointsCoincide(a, b) | Atom::PointDistance(a, b, _) => {
                track(a.p);
                track(b.p);
            }
            Atom::PointOnPlane(p, o, _, _)
            | Atom::AxisLateral(p, o, _)
            | Atom::PointLineDistance(p, o, _, _) => {
                track(p.p);
                track(o.p);
            }
            Atom::LineLineDistance(a, _, b, _, _) | Atom::LineLineTouch(a, _, b, _) => {
                track(a.p);
                track(b.p);
            }
            Atom::DirMatch(..) | Atom::DirCross(..) | Atom::DirDot(..) => {}
        }
    }
    let tight = options.tolerance * scale;

    Ok(Prepared {
        base,
        movable,
        atoms,
        atom_mate,
        row_mate,
        m,
        effective_rows,
        scale,
        tight,
    })
}

/// Final residuals, per-mate attribution, rank/DOF diagnostics, and result
/// assembly at `best_poses`. Strategy independent: diagnostics always use the
/// full Jacobian over every movable body, so `dof`/`rank`/`redundant` match
/// between strategies.
pub(super) fn finalize(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    prep: &Prepared,
    best_poses: &[Pose],
    run: LmRun,
    strategy: &str,
    steps: Vec<SolveStepReport>,
) -> Result<AssemblySolution, String> {
    let movable_groups = singleton_groups(&prep.movable);
    let n = 6 * prep.movable.len();
    let m = prep.m;

    let mut pose_scratch: Vec<Pose> = Vec::with_capacity(best_poses.len());
    let mut r_final: Vec<f64> = Vec::with_capacity(m);
    eval_residuals(
        &prep.atoms,
        best_poses,
        &[],
        &[],
        &mut pose_scratch,
        &mut r_final,
    );
    let max_residual = r_final.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
    let mut per_mate = vec![0.0f64; mates.len()];
    for (row, &mi) in prep.row_mate.iter().enumerate() {
        per_mate[mi] = per_mate[mi].max(r_final[row].abs());
    }

    // DOF diagnostics at the solution (Jacobian rank, sketch_solver-style).
    let col_step = column_steps(prep.movable.len(), prep.scale);
    let jac = finite_diff_jacobian(&prep.atoms, best_poses, &movable_groups, &col_step, m);
    let rank = numerical_rank(jac, m, n);
    let dof = n - rank;
    let redundant = prep.effective_rows.saturating_sub(rank);
    let status = if redundant > 0 {
        "over"
    } else if dof > 0 {
        "under"
    } else {
        "well"
    };

    if max_residual > prep.tight {
        let mut worst: Vec<(usize, f64)> = per_mate
            .iter()
            .copied()
            .enumerate()
            .filter(|(_, res)| *res > prep.tight)
            .collect();
        worst.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        let listing = worst
            .iter()
            .take(5)
            .map(|(mi, res)| format!("{} (residual {res:.3e})", mate_name(*mi, &mates[*mi])))
            .collect::<Vec<_>>()
            .join(", ");
        let cause = if redundant > 0 {
            "conflicting mates (over-constrained)"
        } else {
            "assembly solve did not converge"
        };
        let tight = prep.tight;
        return Err(format!(
            "{cause}: max residual {max_residual:.3e} exceeds tolerance {tight:.3e}; worst mates: {listing}"
        ));
    }

    let poses = bodies
        .iter()
        .zip(best_poses)
        .map(|(body, pose)| BodyPose {
            id: body.id.clone(),
            rotation: pose.q,
            translation: [pose.t.x, pose.t.y, pose.t.z],
        })
        .collect();
    let mate_residuals = per_mate
        .iter()
        .enumerate()
        .map(|(mi, &res)| MateResidualReport {
            mate: mate_name(mi, &mates[mi]),
            residual: res,
        })
        .collect();

    Ok(AssemblySolution {
        poses,
        mate_residuals,
        max_residual,
        iterations: run.iterations,
        strategy: strategy.into(),
        steps,
        residual_rows_evaluated: run.rows,
        dof,
        rank,
        redundant,
        status: status.into(),
    })
}
