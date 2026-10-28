use super::*;

// ---------------------------------------------------------------------------
// §7.5–7.6 decomposition pipeline
// ---------------------------------------------------------------------------

/// Deterministic union-find (smallest index wins as representative).
struct UnionFind {
    parent: Vec<usize>,
}

impl UnionFind {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn find(&mut self, x: usize) -> usize {
        let mut root = x;
        while self.parent[root] != root {
            root = self.parent[root];
        }
        let mut cur = x;
        while self.parent[cur] != root {
            let next = self.parent[cur];
            self.parent[cur] = root;
            cur = next;
        }
        root
    }

    fn union(&mut self, a: usize, b: usize) {
        let (ra, rb) = (self.find(a), self.find(b));
        if ra != rb {
            let (lo, hi) = if ra < rb { (ra, rb) } else { (rb, ra) };
            self.parent[hi] = lo;
        }
    }
}

/// Every body a mate reaches: its two sides and, for a symmetric mate, the
/// mirror's body when that is a third one.
fn mate_bodies(mate: &AssemblyMate) -> impl Iterator<Item = usize> {
    let (a, b) = (mate.body_a, mate.body_b);
    let third = mate.body_c.filter(|&c| c != a && c != b);
    [a, b].into_iter().chain(third)
}

struct DecompOutcome {
    steps: Vec<SolveStepReport>,
    run: LmRun,
}

/// §7.6 sequential peeling + §7.5 rigid-cluster contraction, with a
/// monolithic fallback for the genuinely coupled remainder. Solves in place:
/// `poses` starts at the initial guesses and ends at the decomposed solution.
fn solve_decomposed(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    prep: &Prepared,
    poses: &mut Vec<Pose>,
    max_iterations: usize,
) -> Result<DecompOutcome, String> {
    let nb = bodies.len();
    let mut solved: Vec<bool> = bodies.iter().map(|b| b.fixed).collect();

    // Atom indices per mate; incident mate indices per body.
    let mut mate_atoms: Vec<Vec<usize>> = vec![Vec::new(); mates.len()];
    for (ai, &mi) in prep.atom_mate.iter().enumerate() {
        mate_atoms[mi].push(ai);
    }
    let mut incident: Vec<Vec<usize>> = vec![Vec::new(); nb];
    for (mi, mate) in mates.iter().enumerate() {
        for body in mate_bodies(mate) {
            incident[body].push(mi);
        }
    }

    // Live decomposition nodes: a singleton per movable body, contracted into
    // clusters as §7.5 verifies them. `node_of[b]` is b's live node, if any.
    let mut nodes: Vec<Option<Vec<usize>>> = Vec::new();
    let mut node_of: Vec<Option<usize>> = vec![None; nb];
    for &b in &prep.movable {
        node_of[b] = Some(nodes.len());
        nodes.push(Some(vec![b]));
    }

    let ids = |members: &[usize]| -> Vec<String> {
        members.iter().map(|&b| bodies[b].id.clone()).collect()
    };
    let atoms_of = |mate_set: &[usize]| -> Vec<Atom> {
        mate_set
            .iter()
            .flat_map(|&mi| mate_atoms[mi].iter().map(|&ai| prep.atoms[ai].clone()))
            .collect()
    };

    let mut steps: Vec<SolveStepReport> = Vec::new();
    let mut run = LmRun::default();

    loop {
        // ---- §7.6 sequential peeling ------------------------------------
        // Repeatedly solve the first node (creation order = ascending body
        // index) that is READY: either every incident mate references a
        // solved body (nothing is deferred; free DOF simply keep the guess),
        // or the mates to already-solved bodies alone FULLY constrain the
        // node (Jacobian rank 6 over its rigid DOF), in which case the mates
        // to not-yet-solved bodies are safely deferred — they are enforced
        // later by moving those bodies. Each pick is a small 6-DOF LM instead
        // of a slice of the monolithic one.
        let mut peeled = false;
        loop {
            let mut pick: Option<(usize, Vec<usize>)> = None;
            for (ni, node) in nodes.iter().enumerate() {
                let Some(members) = node else { continue };
                // Partition this node's external mates by the other side.
                let mut to_solved: Vec<usize> = Vec::new();
                let mut deferred = false;
                // A mate's other bodies are every body it reaches outside
                // this node (a symmetric mate's mirror included): it counts
                // toward the solved side only when ALL of them are solved.
                for &b in members {
                    for &mi in &incident[b] {
                        let mut outside = mate_bodies(&mates[mi]).filter(|&other| node_of[other] != Some(ni)).peekable();
                        if outside.peek().is_none() {
                            continue; // Internal to the node.
                        }
                        if outside.all(|other| solved[other]) {
                            to_solved.push(mi);
                        } else {
                            deferred = true;
                        }
                    }
                }
                to_solved.sort_unstable();
                to_solved.dedup();
                let ready = if !deferred {
                    true
                } else if to_solved.is_empty() {
                    false
                } else {
                    // Fully constrained by the solved side alone?
                    let atoms = atoms_of(&to_solved);
                    let m: usize = atoms.iter().map(Atom::rows).sum();
                    let groups = vec![SolveGroup {
                        members: members.clone(),
                    }];
                    let jac = finite_diff_jacobian(
                        &atoms,
                        poses,
                        &groups,
                        &column_steps(1, prep.scale),
                        m,
                    );
                    run.rows += 2 * 6 * m;
                    numerical_rank(jac, m, 6) == 6
                };
                if ready {
                    pick = Some((ni, to_solved));
                    break;
                }
            }
            let Some((ni, mate_set)) = pick else { break };
            let members = nodes[ni].take().expect("picked node is live");
            let atoms = atoms_of(&mate_set);
            let (method, iterations) = if atoms.is_empty() {
                ("unconstrained", 0)
            } else {
                let groups = vec![SolveGroup {
                    members: members.clone(),
                }];
                let sub = lm_core(
                    &atoms,
                    poses,
                    &groups,
                    prep.scale,
                    prep.tight,
                    max_iterations,
                    prep.linear,
                )?;
                let iters = sub.iterations;
                run.absorb(sub);
                let method = if members.len() == 1 {
                    "sequential"
                } else {
                    "cluster_sequential"
                };
                (method, iters)
            };
            steps.push(SolveStepReport {
                bodies: ids(&members),
                method: method.into(),
                iterations,
            });
            for &b in &members {
                solved[b] = true;
                node_of[b] = None;
            }
            peeled = true;
        }
        if nodes.iter().all(|n| n.is_none()) {
            break; // Everything solved.
        }

        // ---- §7.5 rigid-cluster detection -------------------------------
        // Peeling stalled: look for pairs of unsolved lone bodies whose
        // mutual mates fully bind them (relative Jacobian rank 6 at the
        // current poses), union-find them into candidate clusters, solve each
        // cluster internally (anchor pinned) and verify rigidity by the
        // internal-mate Jacobian having rank exactly 6·(k−1) at the internal
        // solution. Verified clusters contract into one rigid node.
        let mut contracted = false;
        let lone = |b: usize, nodes: &[Option<Vec<usize>>], node_of: &[Option<usize>]| -> bool {
            node_of[b]
                .and_then(|ni| nodes[ni].as_ref())
                .is_some_and(|members| members.len() == 1)
        };
        let mut pair_mates: std::collections::BTreeMap<(usize, usize), Vec<usize>> =
            std::collections::BTreeMap::new();
        for (mi, mate) in mates.iter().enumerate() {
            let (a, b) = (mate.body_a, mate.body_b);
            // Pairs only: a mate reaching a third body binds no PAIR.
            if mate_bodies(mate).count() > 2 {
                continue;
            }
            if !solved[a] && !solved[b] && lone(a, &nodes, &node_of) && lone(b, &nodes, &node_of) {
                pair_mates.entry((a.min(b), a.max(b))).or_default().push(mi);
            }
        }
        let mut uf = UnionFind::new(nb);
        let mut any_binding = false;
        for (&(a, b), pm) in &pair_mates {
            let atoms = atoms_of(pm);
            let m: usize = atoms.iter().map(Atom::rows).sum();
            let groups = singleton_groups(&[b]);
            let jac = finite_diff_jacobian(&atoms, poses, &groups, &column_steps(1, prep.scale), m);
            run.rows += 2 * 6 * m;
            if numerical_rank(jac, m, 6) == 6 {
                uf.union(a, b);
                any_binding = true;
            }
        }
        if any_binding {
            let mut components: std::collections::BTreeMap<usize, Vec<usize>> =
                std::collections::BTreeMap::new();
            for &(a, b) in pair_mates.keys() {
                for body in [a, b] {
                    let root = uf.find(body);
                    let entry = components.entry(root).or_default();
                    if !entry.contains(&body) {
                        entry.push(body);
                    }
                }
            }
            for (_, mut comp) in components {
                if comp.len() < 2 {
                    continue;
                }
                comp.sort_unstable();
                // Internal solve: pin the anchor member, move the rest.
                let saved: Vec<Pose> = comp.iter().map(|&b| poses[b]).collect();
                let mut internal: Vec<usize> = comp
                    .iter()
                    .flat_map(|&b| incident[b].iter().copied())
                    .filter(|&mi| mate_bodies(&mates[mi]).all(|body| comp.contains(&body)))
                    .collect();
                internal.sort_unstable();
                internal.dedup();
                let atoms = atoms_of(&internal);
                let groups = singleton_groups(&comp[1..]);
                let sub = lm_core(
                    &atoms,
                    poses,
                    &groups,
                    prep.scale,
                    prep.tight,
                    max_iterations,
                    prep.linear,
                )?;
                let sub_iters = sub.iterations;
                run.absorb(sub);
                // Verify §7.5 rigidity at the internal solution.
                let m_int: usize = atoms.iter().map(Atom::rows).sum();
                let all_groups = singleton_groups(&comp);
                let mut scratch: Vec<Pose> = Vec::new();
                let mut r_int: Vec<f64> = Vec::new();
                eval_residuals(&atoms, poses, &[], &[], &mut scratch, &mut r_int);
                let max_int = r_int.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
                let jac = finite_diff_jacobian(
                    &atoms,
                    poses,
                    &all_groups,
                    &column_steps(comp.len(), prep.scale),
                    m_int,
                );
                let rank = numerical_rank(jac, m_int, 6 * comp.len());
                run.rows += m_int + 2 * 6 * comp.len() * m_int;
                if max_int <= prep.tight && rank == 6 * (comp.len() - 1) {
                    steps.push(SolveStepReport {
                        bodies: ids(&comp),
                        method: "cluster_internal".into(),
                        iterations: sub_iters,
                    });
                    // Contract: one live node owning every member.
                    let cluster = nodes.len();
                    for &b in &comp {
                        if let Some(old) = node_of[b] {
                            nodes[old] = None;
                        }
                        node_of[b] = Some(cluster);
                    }
                    nodes.push(Some(comp));
                    contracted = true;
                } else {
                    // Not actually rigid: restore and leave for the fallback.
                    for (&b, pose) in comp.iter().zip(&saved) {
                        poses[b] = *pose;
                    }
                }
            }
        }
        if peeled || contracted {
            continue;
        }

        // ---- genuinely coupled remainder: monolithic fallback -----------
        // Neither peelable nor clusterable (a coupled cycle of mates): solve
        // everything still unsolved with one monolithic LM over exactly the
        // remaining bodies and every not-yet-applied mate.
        let remaining: Vec<usize> = (0..nb).filter(|&b| node_of[b].is_some()).collect();
        let mate_set: Vec<usize> = (0..mates.len())
            .filter(|&mi| mate_bodies(&mates[mi]).any(|body| !solved[body]))
            .collect();
        let atoms = atoms_of(&mate_set);
        let groups = singleton_groups(&remaining);
        let sub = lm_core(
            &atoms,
            poses,
            &groups,
            prep.scale,
            prep.tight,
            max_iterations,
            prep.linear,
        )?;
        let iters = sub.iterations;
        run.absorb(sub);
        steps.push(SolveStepReport {
            bodies: ids(&remaining),
            method: "monolithic_fallback".into(),
            iterations: iters,
        });
        for node in nodes.iter_mut() {
            *node = None;
        }
        for &b in &remaining {
            solved[b] = true;
            node_of[b] = None;
        }
        break;
    }

    Ok(DecompOutcome { steps, run })
}

// ---------------------------------------------------------------------------
// Solver entry point
// ---------------------------------------------------------------------------

/// Solve the assembly mates in least squares from the bodies' initial poses.
///
/// Grounded bodies never move; free degrees of freedom of movable bodies stay
/// where the initial guess put them (minimum-norm LM steps). By default
/// (`SolveStrategy::Auto`) the solve is decomposed per §7.5–7.6 — sequential
/// peeling from the grounded bodies plus rigid-cluster contraction, with a
/// monolithic fallback for coupled cycles — and agrees with the monolithic
/// strategy within the solve tolerance on well-posed inputs. Returns `Err`
/// when the mates cannot all be satisfied (conflict) or the solve fails to
/// converge, naming the worst-residual mates.
pub fn solve_assembly(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    options: &AssemblySolveOptions,
) -> Result<AssemblySolution, String> {
    let result = solve_with(bodies, mates, options);
    // The sparse/dense A/B over the test corpus: a test run with
    // BREP_ASSEMBLY_AB set re-solves every sparse solve densely and compares.
    // Compiled out of every non-test build (`cfg!(test)` is a constant).
    if cfg!(test) && options.linear_algebra == LinearAlgebra::Sparse && std::env::var_os("BREP_ASSEMBLY_AB").is_some() {
        let dense = solve_with(bodies, mates, &AssemblySolveOptions { linear_algebra: LinearAlgebra::Dense, ..*options });
        if let Err(difference) = compare_solutions(&result, &dense) {
            panic!("sparse/dense A/B: {difference}");
        }
    }
    result
}

/// The A/B acceptance: both fail, or both succeed with poses within 1e-9 mm
/// and 1e-9 rad and the same iteration count or the same status. Along a FREE
/// direction (`dof > 0`) the damped step is the gradient's roundoff divided by
/// a tiny λ, so the two factorizations may come to rest at different, equally
/// valid points: there the bar is 1e-6 and the drift is printed (`AB-DRIFT`).
fn compare_solutions(
    sparse: &Result<AssemblySolution, String>,
    dense: &Result<AssemblySolution, String>,
) -> Result<(), String> {
    match (sparse, dense) {
        (Err(_), Err(_)) => Ok(()),
        (Ok(_), Err(error)) => Err(format!("sparse solved, dense failed: {error}")),
        (Err(error), Ok(_)) => Err(format!("dense solved, sparse failed: {error}")),
        (Ok(a), Ok(b)) => {
            for (pa, pb) in a.poses.iter().zip(&b.poses) {
                let dt = (0..3).map(|i| (pa.translation[i] - pb.translation[i]).abs()).fold(0.0, f64::max);
                // The angle of qa·qb⁻¹: twice the norm of its vector part.
                let (qa, qb) = (pa.rotation, pb.rotation);
                let rel = quat_mul(qa, [qb[0], -qb[1], -qb[2], -qb[3]]);
                let angle = 2.0 * (rel[1] * rel[1] + rel[2] * rel[2] + rel[3] * rel[3]).sqrt();
                let bar = if a.dof > 0 { 1e-6 } else { 1e-9 };
                if dt > bar || angle > bar {
                    return Err(format!("body {}: translation {dt:.3e}, rotation {angle:.3e} (dof {})", pa.id, a.dof));
                }
                if dt > 1e-9 || angle > 1e-9 {
                    eprintln!("AB-DRIFT body {} dof {}: translation {dt:.3e}, rotation {angle:.3e}", pa.id, a.dof);
                }
            }
            if a.iterations != b.iterations && a.status != b.status {
                return Err(format!(
                    "iterations {} vs {} and status {} vs {}",
                    a.iterations, b.iterations, a.status, b.status
                ));
            }
            Ok(())
        }
    }
}

fn solve_with(
    bodies: &[AssemblyBody],
    mates: &[AssemblyMate],
    options: &AssemblySolveOptions,
) -> Result<AssemblySolution, String> {
    if !(options.tolerance.is_finite() && options.tolerance > 0.0) {
        return Err("options.tolerance must be finite and positive".into());
    }
    let prep = prepare(bodies, mates, options)?;
    let movable_ids: Vec<String> = prep.movable.iter().map(|&b| bodies[b].id.clone()).collect();

    let run_monolithic = |poses: &mut Vec<Pose>| -> Result<LmRun, String> {
        let groups = singleton_groups(&prep.movable);
        lm_core(
            &prep.atoms,
            poses,
            &groups,
            prep.scale,
            prep.tight,
            options.max_iterations,
            prep.linear,
        )
    };

    // A mate reaching a third body (a symmetric mate's mirror) takes part in
    // the decomposition: peeling waits for all of its bodies, and only the
    // cluster detection, which binds pairs, leaves it out.
    if options.strategy == SolveStrategy::Monolithic {
        let mut poses = prep.base.clone();
        let run = run_monolithic(&mut poses)?;
        let steps = vec![SolveStepReport {
            bodies: movable_ids,
            method: "monolithic".into(),
            iterations: run.iterations,
        }];
        return finalize(bodies, mates, &prep, &poses, run, "monolithic", steps);
    }

    // Auto / Decomposed: §7.5–7.6 pipeline, then a global acceptance check.
    let mut poses = prep.base.clone();
    let outcome = solve_decomposed(bodies, mates, &prep, &mut poses, options.max_iterations)?;
    let mut run = outcome.run;

    let mut scratch: Vec<Pose> = Vec::new();
    let mut r: Vec<f64> = Vec::new();
    eval_residuals(&prep.atoms, &poses, &[], &[], &mut scratch, &mut r);
    run.rows += prep.m;
    let max_res = r.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
    if max_res.is_finite() && max_res <= prep.tight {
        return finalize(
            bodies,
            mates,
            &prep,
            &poses,
            run,
            "decomposed",
            outcome.steps,
        );
    }

    // The decomposed result missed the global tolerance (an inconsistency
    // spanning steps, or an ill-posed input): redo everything monolithically
    // from the ORIGINAL poses so the decomposed strategies are never worse
    // than `Monolithic`. `finalize` raises the v1-style error if even that
    // does not converge.
    let mut poses = prep.base.clone();
    let sub = run_monolithic(&mut poses)?;
    let iters = sub.iterations;
    run.absorb(sub);
    let mut method = "monolithic_fallback";
    if let Some((flipped, sub)) = flip_restart(mates, &prep, &poses, &run_monolithic) {
        run.absorb(sub);
        poses = flipped;
        method = "flip_restart";
    }
    let steps = vec![SolveStepReport {
        bodies: movable_ids,
        method: method.into(),
        iterations: iters,
    }];
    finalize(bodies, mates, &prep, &poses, run, method, steps)
}

/// The most bodies a failed solve is restarted for, turned over.
const FLIP_RESTART_BODIES: usize = 4;
/// Above this many movable bodies a failed solve is not restarted: each
/// restart is a whole monolithic solve.
const FLIP_RESTART_MAX_MOVABLE: usize = 64;

/// Two starts the solve cannot leave, because the residual has no gradient
/// there:
/// - a mate asked to FLIP (a Touch Align or Concentric reversed, a
///   Distance's normals opposed) starts exactly antiparallel to its goal
///   ("did not converge", residual 2);
/// - a Distance between coaxial cylinders starts where the separation
///   direction is undefined (residual = the whole distance).
/// When `solved` still misses the tolerance, each movable body of the
/// failing mates is restarted from the ORIGINAL poses, turned half a turn
/// about each world axis through its origin, or nudged a thousandth of the
/// model's size along each world axis; of the restarts that converge, the
/// one that moves the bodies least is kept. `None` when none does (a real
/// conflict), or when the solve converged in the first place.
fn flip_restart(
    mates: &[AssemblyMate],
    prep: &Prepared,
    solved: &[Pose],
    run_monolithic: &dyn Fn(&mut Vec<Pose>) -> Result<LmRun, String>,
) -> Option<(Vec<Pose>, LmRun)> {
    if prep.movable.len() > FLIP_RESTART_MAX_MOVABLE {
        return None;
    }
    let residuals = |poses: &[Pose]| -> Vec<f64> {
        let (mut scratch, mut r) = (Vec::new(), Vec::new());
        eval_residuals(&prep.atoms, poses, &[], &[], &mut scratch, &mut r);
        r
    };
    let converged = |poses: &[Pose]| residuals(poses).iter().all(|x| x.is_finite() && x.abs() <= prep.tight);
    let r = residuals(solved);
    let mut per_mate = vec![0.0f64; mates.len()];
    for (row, &mi) in prep.row_mate.iter().enumerate() {
        per_mate[mi] = per_mate[mi].max(r[row].abs());
    }
    let mut failing: Vec<usize> = (0..mates.len()).filter(|&mi| per_mate[mi] > prep.tight).collect();
    if failing.is_empty() {
        return None;
    }
    failing.sort_by(|&a, &b| per_mate[b].total_cmp(&per_mate[a]));
    let mut candidates: Vec<usize> = Vec::new();
    for &mi in &failing {
        for body in mate_bodies(&mates[mi]) {
            if prep.movable.contains(&body) && !candidates.contains(&body) {
                candidates.push(body);
            }
        }
    }
    candidates.truncate(FLIP_RESTART_BODIES);
    let moved = |poses: &[Pose]| -> f64 {
        poses.iter().zip(&prep.base).map(|(p, b)| p.t.sub(b.t).length()).sum()
    };
    for body in candidates {
        let mut best: Option<(f64, Vec<Pose>, LmRun)> = None;
        let nudge = 1e-3 * prep.scale.max(1e-9);
        let starts = [
            ([0.0, 1.0, 0.0, 0.0], [0.0; 3]),
            ([0.0, 0.0, 1.0, 0.0], [0.0; 3]),
            ([0.0, 0.0, 0.0, 1.0], [0.0; 3]),
            ([1.0, 0.0, 0.0, 0.0], [nudge, 0.0, 0.0]),
            ([1.0, 0.0, 0.0, 0.0], [0.0, nudge, 0.0]),
            ([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, nudge]),
        ];
        for (turn, shift) in starts {
            let mut poses = prep.base.clone();
            poses[body].q = quat_mul(turn, poses[body].q);
            poses[body].t = poses[body].t.add(crate::Vec3::new(shift[0], shift[1], shift[2]));
            let Ok(run) = run_monolithic(&mut poses) else { continue };
            if converged(&poses) {
                let cost = moved(&poses);
                if best.as_ref().is_none_or(|(c, _, _)| cost < *c - 1e-9) {
                    best = Some((cost, poses, run));
                }
            }
        }
        if let Some((_, poses, run)) = best {
            return Some((poses, run));
        }
    }
    None
}
