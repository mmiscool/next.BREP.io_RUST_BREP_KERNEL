use super::*;

// ---------------------------------------------------------------------------
// Block-sparse Jacobian, normal equations and Cholesky (§7.3, large assemblies)
// ---------------------------------------------------------------------------
//
// A mate touches two or three bodies, so the Jacobian over N movable bodies is
// block-sparse: an atom's rows are non-zero only in the 6-column blocks of the
// solve groups that hold its bodies. The dense path evaluates every atom for
// every column (2·n·m rows per Jacobian) and builds and factors an n x n matrix
// (O(m·n²) + O(n³)). Here:
//
// - the FD Jacobian perturbs one group at a time and evaluates only the atoms
//   incident to it. An atom that touches no member of the perturbed group
//   evaluates to the same value at ±h, so its dense difference is exactly 0:
//   the blocks kept here are bit-identical to the dense Jacobian's;
// - `JᵀJ` and `Jᵀr` are accumulated per atom, rows ascending, which is the
//   dense loop's summation order for every entry, so H and g are bit-identical
//   too;
// - H is factored by a right-looking block Cholesky over 6 x 6 blocks, in a
//   greedy minimum-degree elimination order (ties by group index, so the order
//   is deterministic). A non-positive pivot fails exactly where the dense
//   `solve_spd` would report "not positive definite", and the LM raises λ.
//
// Only the factorization's roundoff differs from the dense path.

/// The bodies an atom reads, deduplicated, ascending.
pub(super) fn atom_bodies(atom: &Atom) -> Vec<usize> {
    let mut bodies: Vec<usize> = Vec::with_capacity(4);
    let mut p = |lp: &LocalPoint| bodies.push(lp.body);
    let mut points: Vec<LocalPoint> = Vec::new();
    let mut dirs: Vec<LocalDir> = Vec::new();
    match *atom {
        Atom::PointsCoincide(a, b) | Atom::PointDistance(a, b, _) => points.extend([a, b]),
        Atom::PointOnPlane(a, o, n, _) | Atom::AxisLateral(a, o, n) | Atom::PointLineDistance(a, o, n, _) => {
            points.extend([a, o]);
            dirs.push(n);
        }
        Atom::DirMatch(a, b, _) | Atom::DirCross(a, b) | Atom::DirDot(a, b, _) => dirs.extend([a, b]),
        Atom::DirAngle(a, b, n, _) => dirs.extend([a, b, n]),
        Atom::LineLineDistance(oa, da, ob, db, _) | Atom::LineLineTouch(oa, da, ob, db) => {
            points.extend([oa, ob]);
            dirs.extend([da, db]);
        }
        Atom::Limit(measure, ..) => match measure {
            Measure::PointOnPlane(a, o, n) | Measure::PointLineDistance(a, o, n) => {
                points.extend([a, o]);
                dirs.push(n);
            }
            Measure::DirDot(a, b) => dirs.extend([a, b]),
            Measure::DirAngle(a, b, n) => dirs.extend([a, b, n]),
            Measure::PointDistance(a, b) => points.extend([a, b]),
            Measure::LineLineDistance(oa, da, ob, db) => {
                points.extend([oa, ob]);
                dirs.extend([da, db]);
            }
        },
        Atom::SymPoints(a, b, o, n) => {
            points.extend([a, b, o]);
            dirs.push(n);
        }
        Atom::SymDirMatch(a, b, n) | Atom::SymDirCross(a, b, n) => dirs.extend([a, b, n]),
        Atom::SymPointOnPlane(pb, oa, na, o, n) => {
            points.extend([pb, oa, o]);
            dirs.extend([na, n]);
        }
        Atom::SymAxisLateral(oa, ob, db, o, n) => {
            points.extend([oa, ob, o]);
            dirs.extend([db, n]);
        }
        Atom::GearPhase(a, b, ..) => dirs.extend([a.reference, a.axis, b.reference, b.axis]),
        Atom::RackTravel(pinion, pt, d, ..) => {
            points.push(pt);
            dirs.extend([pinion.reference, pinion.axis, d]);
        }
        // The curve's body is read through its pose (the closest point).
        Atom::PointOnCurve(a, ref curve) | Atom::CurveAlong(a, None, ref curve, ..) => {
            points.push(a);
            dirs.push(LocalDir { body: curve.body, d: Vec3::new(0.0, 0.0, 1.0) });
        }
        Atom::AxisOnCurve(a, d, ref curve) | Atom::CurveAlong(a, Some(d), ref curve, ..) => {
            points.push(a);
            dirs.extend([d, LocalDir { body: curve.body, d: Vec3::new(0.0, 0.0, 1.0) }]);
        }
        Atom::CamContact(a, axis, ref curve, normal, ..) => {
            points.push(a);
            dirs.extend(axis);
            dirs.extend([normal, LocalDir { body: curve.body, d: Vec3::new(0.0, 0.0, 1.0) }]);
        }
        Atom::ScrewAdvance(oa, da, ra, rb, pb, ..) => {
            points.extend([oa, pb]);
            dirs.extend([da, ra, rb]);
        }
    }
    for lp in &points {
        p(lp);
    }
    bodies.extend(dirs.iter().map(|ld| ld.body));
    bodies.sort_unstable();
    bodies.dedup();
    bodies
}

/// Which atoms touch which solve groups (and the reverse), for one solve.
pub(super) struct Incidence {
    /// Atom → first residual row.
    row_start: Vec<usize>,
    /// Atom → the groups it touches, ascending.
    atom_groups: Vec<Vec<usize>>,
    /// Group → the atoms touching it, ascending.
    group_atoms: Vec<Vec<usize>>,
    /// Atom → is a wrapped angular row (the dense FD unwraps these).
    angular: Vec<bool>,
}

impl Incidence {
    pub(super) fn new(atoms: &[Atom], groups: &[SolveGroup], body_count: usize) -> Self {
        let mut group_of = vec![usize::MAX; body_count];
        for (k, group) in groups.iter().enumerate() {
            for &b in &group.members {
                group_of[b] = k;
            }
        }
        let mut row_start = Vec::with_capacity(atoms.len());
        let mut atom_groups = Vec::with_capacity(atoms.len());
        let mut group_atoms = vec![Vec::new(); groups.len()];
        let mut row = 0;
        for (ai, atom) in atoms.iter().enumerate() {
            row_start.push(row);
            row += atom.rows();
            let mut touched: Vec<usize> = atom_bodies(atom)
                .into_iter()
                .filter_map(|b| group_of.get(b).copied().filter(|&k| k != usize::MAX))
                .collect();
            touched.sort_unstable();
            touched.dedup();
            for &k in &touched {
                group_atoms[k].push(ai);
            }
            atom_groups.push(touched);
        }
        let angular = atoms.iter().map(|atom| matches!(atom, Atom::DirAngle(..))).collect();
        Self { row_start, atom_groups, group_atoms, angular }
    }
}

/// The FD Jacobian as per-atom blocks: for atom `a`, one `rows x 6` row-major
/// block per group in `Incidence::atom_groups[a]`, in that order.
pub(super) struct SparseJacobian {
    blocks: Vec<Vec<Vec<f64>>>,
}

/// Apply ONE column's increment `value` (column `c` of group `group`) to the
/// group's members — the singleton/cluster formulas of `effective_poses`,
/// with every other component of the increment zero.
fn perturb_group(base: &[Pose], group: &SolveGroup, c: usize, value: f64, out: &mut [Pose]) {
    let mut rot = [0.0f64; 3];
    let mut tr = [0.0f64; 3];
    if c < 3 {
        rot[c] = value;
    } else {
        tr[c - 3] = value;
    }
    let dq = quat_from_rotvec(Vec3::new(rot[0], rot[1], rot[2]));
    let dt = Vec3::new(tr[0], tr[1], tr[2]);
    if let [only] = group.members[..] {
        let pose = base[only];
        out[only] = Pose { q: quat_mul(dq, pose.q), t: pose.t.add(dt) };
    } else {
        let anchor = base[group.members[0]].t;
        for &bi in &group.members {
            let pose = base[bi];
            out[bi] = Pose {
                q: quat_mul(dq, pose.q),
                t: anchor.add(quat_rotate(dq, pose.t.sub(anchor))).add(dt),
            };
        }
    }
}

/// Central-difference Jacobian, block-sparse; bit-identical to the non-zero
/// entries of `finite_diff_jacobian`. Returns the blocks and the residual rows
/// evaluated.
pub(super) fn sparse_fd_jacobian(
    atoms: &[Atom],
    base: &[Pose],
    groups: &[SolveGroup],
    col_step: &[f64],
    inc: &Incidence,
) -> (SparseJacobian, usize) {
    let mut blocks: Vec<Vec<Vec<f64>>> = atoms
        .iter()
        .enumerate()
        .map(|(ai, atom)| vec![vec![0.0f64; atom.rows() * 6]; inc.atom_groups[ai].len()])
        .collect();
    let mut poses: Vec<Pose> = base.to_vec();
    let mut plus: Vec<f64> = Vec::new();
    let mut minus: Vec<f64> = Vec::new();
    let mut rows = 0;
    for (k, group) in groups.iter().enumerate() {
        for c in 0..6 {
            let h = col_step[6 * k + c];
            let inv = 1.0 / (2.0 * h);
            for &ai in &inc.group_atoms[k] {
                let atom = &atoms[ai];
                perturb_group(base, group, c, h, &mut poses);
                plus.clear();
                atom.eval(&poses, &mut plus);
                perturb_group(base, group, c, -h, &mut poses);
                minus.clear();
                atom.eval(&poses, &mut minus);
                rows += 2 * plus.len();
                let slot = inc.atom_groups[ai].iter().position(|&g| g == k).expect("incident group");
                let block = &mut blocks[ai][slot];
                for i in 0..plus.len() {
                    let difference = plus[i] - minus[i];
                    let difference = if inc.angular[ai] {
                        difference.sin().atan2(difference.cos())
                    } else {
                        difference
                    };
                    block[i * 6 + c] = difference * inv;
                }
            }
            for &bi in &group.members {
                poses[bi] = base[bi];
            }
        }
    }
    (SparseJacobian { blocks }, rows)
}

impl SparseJacobian {
    /// The dense `m x n` matrix (diagnostics: rank, and the dense A/B path).
    pub(super) fn to_dense(&self, atoms: &[Atom], inc: &Incidence, n: usize) -> Vec<Vec<f64>> {
        let m: usize = atoms.iter().map(Atom::rows).sum();
        let mut jac = vec![vec![0.0f64; n]; m];
        for (ai, atom) in atoms.iter().enumerate() {
            for (slot, &k) in inc.atom_groups[ai].iter().enumerate() {
                for i in 0..atom.rows() {
                    for c in 0..6 {
                        jac[inc.row_start[ai] + i][6 * k + c] = self.blocks[ai][slot][i * 6 + c];
                    }
                }
            }
        }
        jac
    }
}

/// `H = JᵀJ` as 6 x 6 blocks (diagonal + strictly-lower off-diagonal, keyed
/// `(row group, column group)` with row > column) and `g = Jᵀr`.
pub(super) struct BlockNormal {
    diag: Vec<[f64; 36]>,
    lower: std::collections::BTreeMap<(usize, usize), [f64; 36]>,
    pub(super) g: Vec<f64>,
}

impl BlockNormal {
    pub(super) fn new(jac: &SparseJacobian, atoms: &[Atom], inc: &Incidence, r: &[f64], group_count: usize) -> Self {
        let mut diag = vec![[0.0f64; 36]; group_count];
        let mut lower: std::collections::BTreeMap<(usize, usize), [f64; 36]> = std::collections::BTreeMap::new();
        let mut g = vec![0.0f64; 6 * group_count];
        for (ai, atom) in atoms.iter().enumerate() {
            let touched = &inc.atom_groups[ai];
            for i in 0..atom.rows() {
                let ri = r[inc.row_start[ai] + i];
                for (sa, &ka) in touched.iter().enumerate() {
                    let row_a = &jac.blocks[ai][sa][i * 6..i * 6 + 6];
                    for ca in 0..6 {
                        let ja = row_a[ca];
                        if ja == 0.0 {
                            continue;
                        }
                        g[6 * ka + ca] += ja * ri;
                        for (sb, &kb) in touched.iter().enumerate() {
                            let row_b = &jac.blocks[ai][sb][i * 6..i * 6 + 6];
                            if kb == ka {
                                for cb in ca..6 {
                                    diag[ka][ca * 6 + cb] += ja * row_b[cb];
                                }
                            } else if ka > kb {
                                // Row group ka, column group kb: column index
                                // 6kb+cb < 6ka+ca, the dense loop's (b, a) entry
                                // is filled from the same products.
                                let block = lower.entry((ka, kb)).or_insert([0.0; 36]);
                                for cb in 0..6 {
                                    block[ca * 6 + cb] += ja * row_b[cb];
                                }
                            }
                        }
                    }
                }
            }
        }
        for block in &mut diag {
            for a in 0..6 {
                for b in (a + 1)..6 {
                    block[b * 6 + a] = block[a * 6 + b];
                }
            }
        }
        Self { diag, lower, g }
    }

    /// The largest diagonal entry (the LM's damping scale).
    pub(super) fn max_diag(&self) -> f64 {
        self.diag
            .iter()
            .flat_map(|block| (0..6).map(move |d| block[d * 6 + d]))
            .fold(0.0f64, f64::max)
    }

    /// Solve `(H + λI) x = b` in place by block Cholesky; `false` on a
    /// non-positive pivot (the caller raises λ and retries).
    pub(super) fn solve_damped(&self, lambda: f64, b: &mut [f64]) -> bool {
        let count = self.diag.len();
        // Working copy: diagonal blocks and a symmetric adjacency of lower blocks.
        let mut diag: Vec<[f64; 36]> = self.diag.clone();
        for block in &mut diag {
            for d in 0..6 {
                block[d * 6 + d] += lambda;
            }
        }
        // off[i] holds (j, block_ij) for j != i, as row-major 6x6 with
        // block_ij = H[i rows][j cols].
        let mut off: Vec<std::collections::BTreeMap<usize, [f64; 36]>> = vec![Default::default(); count];
        for (&(i, j), block) in &self.lower {
            off[i].insert(j, *block);
            off[j].insert(i, transpose(block));
        }
        let mut eliminated = vec![false; count];
        // Elimination record: (k, L_kk, [(i, L_ik)]).
        let mut record: Vec<(usize, [f64; 36], Vec<(usize, [f64; 36])>)> = Vec::with_capacity(count);
        for _ in 0..count {
            // Greedy minimum degree, ties by index: deterministic.
            let k = (0..count)
                .filter(|&k| !eliminated[k])
                .min_by_key(|&k| (off[k].len(), k))
                .expect("a node remains");
            let Some(lkk) = cholesky6(&diag[k]) else {
                return false;
            };
            let neighbours: Vec<usize> = std::mem::take(&mut off[k]).into_keys().collect();
            // L_ik = H_ik · L_kk^{-T}, with H_ik (rows of i, columns of k) read
            // from i's side; the entry leaves the working matrix with k.
            let lik: Vec<(usize, [f64; 36])> = neighbours
                .iter()
                .map(|&i| {
                    let hik = off[i].remove(&k).expect("symmetric adjacency");
                    (i, right_solve_lt(&hik, &lkk))
                })
                .collect();
            // Schur update: H_ij -= L_ik · L_jkᵀ for every neighbour pair.
            for (a, (i, li)) in lik.iter().enumerate() {
                let update = mul_abt(li, li);
                sub_assign(&mut diag[*i], &update);
                for (j, lj) in lik.iter().skip(a + 1) {
                    let update = mul_abt(li, lj);
                    let entry = off[*i].entry(*j).or_insert([0.0; 36]);
                    sub_assign(entry, &update);
                    let entry = off[*j].entry(*i).or_insert([0.0; 36]);
                    sub_assign(entry, &transpose(&update));
                }
            }
            eliminated[k] = true;
            record.push((k, lkk, lik));
        }
        // Forward: L y = b, in elimination order.
        for (k, lkk, lik) in &record {
            let yk = forward6(lkk, &b[6 * k..6 * k + 6]);
            b[6 * k..6 * k + 6].copy_from_slice(&yk);
            for (i, l) in lik {
                for r in 0..6 {
                    let mut s = 0.0;
                    for c in 0..6 {
                        s += l[r * 6 + c] * yk[c];
                    }
                    b[6 * i + r] -= s;
                }
            }
        }
        // Backward: Lᵀ x = y, in reverse elimination order.
        for (k, lkk, lik) in record.iter().rev() {
            let mut rhs = [0.0f64; 6];
            rhs.copy_from_slice(&b[6 * k..6 * k + 6]);
            for (i, l) in lik {
                for c in 0..6 {
                    let mut s = 0.0;
                    for r in 0..6 {
                        s += l[r * 6 + c] * b[6 * i + r];
                    }
                    rhs[c] -= s;
                }
            }
            let xk = backward6(lkk, &rhs);
            b[6 * k..6 * k + 6].copy_from_slice(&xk);
        }
        b.iter().all(|v| v.is_finite())
    }
}

fn transpose(a: &[f64; 36]) -> [f64; 36] {
    let mut t = [0.0; 36];
    for r in 0..6 {
        for c in 0..6 {
            t[c * 6 + r] = a[r * 6 + c];
        }
    }
    t
}

/// `a · bᵀ` for 6 x 6 row-major blocks.
fn mul_abt(a: &[f64; 36], b: &[f64; 36]) -> [f64; 36] {
    let mut out = [0.0; 36];
    for r in 0..6 {
        for c in 0..6 {
            let mut s = 0.0;
            for k in 0..6 {
                s += a[r * 6 + k] * b[c * 6 + k];
            }
            out[r * 6 + c] = s;
        }
    }
    out
}

fn sub_assign(a: &mut [f64; 36], b: &[f64; 36]) {
    for (x, y) in a.iter_mut().zip(b) {
        *x -= y;
    }
}

/// Lower Cholesky factor of a symmetric 6 x 6 block; `None` on a
/// non-positive pivot (as `solve_spd`).
fn cholesky6(a: &[f64; 36]) -> Option<[f64; 36]> {
    let mut l = [0.0f64; 36];
    for i in 0..6 {
        for j in 0..=i {
            let mut sum = a[i * 6 + j];
            for k in 0..j {
                sum -= l[i * 6 + k] * l[j * 6 + k];
            }
            if i == j {
                if !(sum > 0.0) {
                    return None;
                }
                l[i * 6 + i] = sum.sqrt();
            } else {
                l[i * 6 + j] = sum / l[j * 6 + j];
            }
        }
    }
    Some(l)
}

/// `X = H · L^{-T}` (solve `X · Lᵀ = H` row by row).
fn right_solve_lt(h: &[f64; 36], l: &[f64; 36]) -> [f64; 36] {
    let mut x = [0.0f64; 36];
    for r in 0..6 {
        for c in 0..6 {
            let mut s = h[r * 6 + c];
            for k in 0..c {
                s -= x[r * 6 + k] * l[c * 6 + k];
            }
            x[r * 6 + c] = s / l[c * 6 + c];
        }
    }
    x
}

fn forward6(l: &[f64; 36], b: &[f64]) -> [f64; 6] {
    let mut y = [0.0f64; 6];
    for i in 0..6 {
        let mut s = b[i];
        for k in 0..i {
            s -= l[i * 6 + k] * y[k];
        }
        y[i] = s / l[i * 6 + i];
    }
    y
}

fn backward6(l: &[f64; 36], y: &[f64; 6]) -> [f64; 6] {
    let mut x = [0.0f64; 6];
    for i in (0..6).rev() {
        let mut s = y[i];
        for k in (i + 1)..6 {
            s -= l[k * 6 + i] * x[k];
        }
        x[i] = s / l[i * 6 + i];
    }
    x
}
