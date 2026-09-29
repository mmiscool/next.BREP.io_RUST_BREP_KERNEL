//! Dense linear algebra shared by the sketch and assembly solvers.

/// Solve `a x = b` via Cholesky factorization, reading only `a`'s lower triangle.
/// `a` receives the factor and `b` the solution. Returns false for a non-positive
/// pivot (including NaN), allowing callers to increase damping and retry.
pub(crate) fn solve_spd(a: &mut [Vec<f64>], b: &mut [f64]) -> bool {
    let n = b.len();
    for i in 0..n {
        for j in 0..=i {
            let mut sum = a[i][j];
            for k in 0..j {
                sum -= a[i][k] * a[j][k];
            }
            if i == j {
                if !(sum > 0.0) {
                    return false;
                }
                a[i][i] = sum.sqrt();
            } else {
                a[i][j] = sum / a[j][j];
            }
        }
    }
    // Forward substitution: L y = b.
    for i in 0..n {
        let mut sum = b[i];
        for k in 0..i {
            sum -= a[i][k] * b[k];
        }
        b[i] = sum / a[i][i];
    }
    // Back substitution: Lᵀ x = y.
    for i in (0..n).rev() {
        let mut sum = b[i];
        for k in (i + 1)..n {
            sum -= a[k][i] * b[k];
        }
        b[i] = sum / a[i][i];
    }
    true
}

/// Reduce a dense matrix in place using partial pivoting. Report each pivot
/// column to the caller and return the rank; callers choose their own tolerance.
pub(crate) fn row_reduce(
    a: &mut [Vec<f64>],
    m: usize,
    n: usize,
    tol: f64,
    mut on_pivot: impl FnMut(usize),
) -> usize {
    let mut rank = 0usize;
    let mut row = 0usize;
    for col in 0..n {
        if row >= m {
            break;
        }
        let mut sel = row;
        let mut best = a[row][col].abs();
        for r in (row + 1)..m {
            let av = a[r][col].abs();
            if av > best {
                best = av;
                sel = r;
            }
        }
        if best <= tol {
            continue;
        }
        a.swap(sel, row);
        let piv = a[row][col];
        for j in 0..n {
            a[row][j] /= piv;
        }
        for r in 0..m {
            if r == row {
                continue;
            }
            let f = a[r][col];
            if f != 0.0 {
                for j in 0..n {
                    a[r][j] -= f * a[row][j];
                }
            }
        }
        on_pivot(col);
        rank += 1;
        row += 1;
    }
    rank
}

