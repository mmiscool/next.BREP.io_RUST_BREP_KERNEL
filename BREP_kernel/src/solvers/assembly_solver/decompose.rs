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
        incident[mate.body_a].push(mi);
        incident[mate.body_b].push(mi);
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
            .flat_map(|&mi| mate_atoms[mi].iter().map(|&ai| prep.atoms[ai]))
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
                for &b in members {
                    for &mi in &incident[b] {
                        let other = if mates[mi].body_a == b {
                            mates[mi].body_b
                        } else {
                            mates[mi].body_a
                        };
                        if node_of[other] == Some(ni) {
                            continue; // Internal to the node.
                        }
                        if solved[other] {
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
                    .filter(|&mi| {
                        comp.contains(&mates[mi].body_a) && comp.contains(&mates[mi].body_b)
                    })
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
            .filter(|&mi| !solved[mates[mi].body_a] || !solved[mates[mi].body_b])
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
        )
    };

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
    let steps = vec![SolveStepReport {
        bodies: movable_ids,
        method: "monolithic_fallback".into(),
        iterations: iters,
    }];
    finalize(
        bodies,
        mates,
        &prep,
        &poses,
        run,
        "monolithic_fallback",
        steps,
    )
}
