//! The smallest-change solve one interactive drag frame tries first.
//!
//! Relaxation satisfies constraints one at a time, each by moving whichever of its
//! points it picks, so a drag that re-solves every frame from the last frame's result
//! lets free geometry wander: a point the drag never touches walks a little each
//! frame, and an unconstrained line grows. Here the free coordinates instead take the
//! smallest step that meets every constraint row: a damped Gauss-Newton solve on the
//! same residual model as the exactness polish, started from where the points stand
//! (the previous frame's solve, the dragged points pinned). Its steps solve
//! `(JᵀJ + λI) δ = −Jᵀr`, which for a small `λ` is the minimum-norm step, so a
//! coordinate no row needs to move stays put. A frame this cannot satisfy (a large
//! jump the linearization cannot follow, or rows past its size limit) leaves the
//! points as they were and falls back to relaxation.

use super::dof_polish::{eval_residuals, finite_diff_jacobian};
use super::*;
use crate::solver_linear_algebra::solve_spd;

/// Outer Gauss-Newton iterations per drag frame.
const DRAG_MAX_ITER: usize = 40;
/// Damping increases per iteration before the frame gives up.
const DRAG_MAX_DAMPING: usize = 12;
/// A frame counts as solved when every row is within this of zero, relative to the
/// sketch scale: the exactness polish's own target, so the frame needs no polish.
const DRAG_ACCEPT_REL_TOL: f64 = 1e-11;
/// Above this many free coordinates the dense solve is too slow for a frame.
const DRAG_MAX_FREE_COORDS: usize = 1200;

impl Engine {
    /// Move the free coordinates by the smallest step that meets every constraint
    /// row. Returns whether it did; on `false` the coordinates are as they were and
    /// the caller relaxes instead.
    pub(super) fn min_change_solve(&mut self) -> Result<bool, String> {
        // The caller's `fixed` flags pin the dragged points for this step only.
        // Relaxation clears every flag and re-fixes grounded points, so the solved
        // sketch (and its freedom diagnostics) carries the same flags either way.
        let pinned: Vec<bool> = self.points.iter().map(|p| p.fixed).collect();
        for point in &mut self.points {
            point.fixed = false;
        }
        // Grounded points take their pinned place first and drop out of the unknowns,
        // as they do at the start of relaxation.
        self.process_constraints_of_type(&Filter::Type(CType::Ground, "⏚"))?;
        let grounded: Vec<bool> = self.points.iter().map(|p| p.fixed).collect();
        for (point, &pin) in self.points.iter_mut().zip(&pinned) {
            point.fixed |= pin;
        }
        let before: Vec<(f64, f64)> = self.points.iter().map(|p| (p.x, p.y)).collect();
        let stepped = self.min_change_step();
        for (point, &ground) in self.points.iter_mut().zip(&grounded) {
            point.fixed = ground;
        }
        // The step only saw rows with a free coordinate. A row between pinned points
        // (two dragged points a constraint ties) is checked here, over every row with
        // only the grounded points held: anything still open goes to relaxation.
        let solved = stepped && self.every_row_met();
        if !solved {
            for (point, (x, y)) in self.points.iter_mut().zip(before) {
                point.x = x;
                point.y = y;
            }
        }
        Ok(solved)
    }

    /// Whether every constraint row (including the engine's implied ones) is within
    /// the drag frame's acceptance at the current coordinates.
    fn every_row_met(&self) -> bool {
        let (point_cols, _free, coords, scale) = self.build_free_layout(true);
        let residuals = self.build_dof_residuals(&coords, &point_cols, true);
        let accept = DRAG_ACCEPT_REL_TOL * scale.max(1.0);
        eval_residuals(&residuals, &coords)
            .iter()
            .all(|r| r.is_finite() && r.abs() <= accept)
    }

    /// The damped Gauss-Newton solve over the free coordinates (points not `fixed`).
    fn min_change_step(&mut self) -> bool {
        let (point_cols, free_global, coords0, scale) = self.build_free_layout(true);
        let n = free_global.len();
        if n == 0 || n > DRAG_MAX_FREE_COORDS {
            return false;
        }
        let residuals = self.build_dof_residuals(&coords0, &point_cols, true);
        let mut r = eval_residuals(&residuals, &coords0);
        let m = r.len();
        let accept = DRAG_ACCEPT_REL_TOL * scale.max(1.0);
        let max_abs = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
        let sum_sq = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc + x * x);
        if m == 0 {
            return true;
        }
        let f0 = max_abs(&r);
        if !f0.is_finite() {
            return false;
        }
        let mut coords = coords0;
        let mut s = sum_sq(&r);
        let mut best = f0;
        let mut lambda = -1.0f64;
        for _ in 0..DRAG_MAX_ITER {
            if best <= accept {
                break;
            }
            let jac = finite_diff_jacobian(&residuals, &coords, &free_global, scale, m);
            let mut h = vec![vec![0.0f64; n]; n];
            let mut g = vec![0.0f64; n];
            for (row, ri) in jac.iter().zip(&r) {
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
            let mut max_diag = 0.0f64;
            for a in 0..n {
                for b in (a + 1)..n {
                    h[b][a] = h[a][b];
                }
                max_diag = max_diag.max(h[a][a]);
            }
            if !(max_diag > 0.0) {
                break;
            }
            if lambda < 0.0 {
                // Small against the rows' scale: a near minimum-norm first step.
                lambda = 1e-9 * max_diag;
            }
            let mut stepped = false;
            for _ in 0..DRAG_MAX_DAMPING {
                let mut a_mat = h.clone();
                for d in 0..n {
                    a_mat[d][d] += lambda;
                }
                let mut delta: Vec<f64> = g.iter().map(|v| -v).collect();
                if solve_spd(&mut a_mat, &mut delta) {
                    let mut trial = coords.clone();
                    for col in 0..n {
                        trial[free_global[col]] += delta[col];
                    }
                    let r_trial = eval_residuals(&residuals, &trial);
                    let s_trial = sum_sq(&r_trial);
                    if s_trial.is_finite() && s_trial < s {
                        coords = trial;
                        best = max_abs(&r_trial);
                        r = r_trial;
                        s = s_trial;
                        lambda = (lambda * 0.25).max(1e-12 * max_diag);
                        stepped = true;
                        break;
                    }
                }
                lambda *= 8.0;
            }
            if !stepped {
                break;
            }
        }
        if best > accept {
            return false;
        }
        for col in 0..n {
            let gcol = free_global[col];
            let i = gcol / 2;
            if gcol % 2 == 0 {
                self.points[i].x = coords[gcol];
            } else {
                self.points[i].y = coords[gcol];
            }
        }
        true
    }
}
