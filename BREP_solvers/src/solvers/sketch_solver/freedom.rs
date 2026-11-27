//! Precise freedom diagnostics: which way each point can still move, how many
//! freedoms each geometry keeps, and which constraints say nothing new.
//!
//! The counts the diagnostics already report (`dof`, `redundant`) are totals over
//! the sketch, and per-point mobility is only movable / locked. The editor needs
//! the detail to point at what to fix: an under-constrained point's free
//! directions (drawn as arrows), its and its geometry's remaining freedoms (the
//! hover readout), and the constraints whose rows depend on earlier ones (listed
//! for deletion). All of it is read off the same Jacobian and null space the
//! totals come from, at the solved configuration.

use super::dof_polish::{rank_and_nullspace, DOF_MOBILITY_TOL};
use super::*;
use serde_json::json;

/// A null-space direction weaker than this (relative to the strongest at the
/// point) is a rounding residue, not a freedom.
const FREEDOM_REL_TOL: f64 = 1e-6;
/// A constraint row whose part outside the earlier rows' span is below this
/// fraction of its length depends on them.
const REDUNDANT_REL_TOL: f64 = 1e-7;

/// The freedom report, as the three diagnostics fields it fills.
pub(super) struct FreedomReport {
    /// `point id -> {dof, directions: [[dx, dy], ...]}`, for points that can move.
    pub(super) points: Map<String, Value>,
    /// `geometry id -> remaining freedoms of its defining points together`.
    pub(super) geometries: Map<String, Value>,
    /// Constraints (document ids) whose rows all depend on earlier constraints'.
    pub(super) redundant: Vec<Value>,
}

impl Engine {
    /// The freedom report for the solved sketch. `basis` is the Jacobian's null
    /// space over the free columns (`point_cols`), `jac` its rows (one per entry of
    /// `row_owner`, the index of the constraint each row states), and `conflicting`
    /// the constraint ids already named as conflicting (never also named redundant).
    pub(super) fn freedom_report(
        &self,
        basis: &[Vec<f64>],
        point_cols: &[Option<(usize, usize)>],
        jac: &[Vec<f64>],
        row_owner: &[usize],
        conflicting: &[Value],
    ) -> FreedomReport {
        // Each basis vector scaled to unit max-norm, so no one vector dominates.
        let unit: Vec<Vec<f64>> = basis
            .iter()
            .map(|v| {
                let m = v.iter().fold(0.0f64, |a, &x| a.max(x.abs()));
                if m > 0.0 { v.iter().map(|x| x / m).collect() } else { v.clone() }
            })
            .collect();

        let mut points = Map::new();
        for (i, cols) in point_cols.iter().enumerate() {
            let Some((cx, cy)) = *cols else { continue };
            // The 2x2 Gram matrix of the point's motion over the null space: its
            // eigenvectors are the point's free directions, its rank their count.
            let (mut gxx, mut gxy, mut gyy) = (0.0f64, 0.0f64, 0.0f64);
            for v in &unit {
                gxx += v[cx] * v[cx];
                gxy += v[cx] * v[cy];
                gyy += v[cy] * v[cy];
            }
            let half_tr = 0.5 * (gxx + gyy);
            let det = gxx * gyy - gxy * gxy;
            let disc = (half_tr * half_tr - det).max(0.0).sqrt();
            let (l1, l2) = (half_tr + disc, half_tr - disc);
            let floor = DOF_MOBILITY_TOL * DOF_MOBILITY_TOL;
            if !(l1 > floor) {
                continue;
            }
            let e1 = if gxy.abs() > 1e-300 {
                let (x, y) = (l1 - gyy, gxy);
                let n = x.hypot(y);
                [x / n, y / n]
            } else if gxx >= gyy {
                [1.0, 0.0]
            } else {
                [0.0, 1.0]
            };
            let two = l2 > floor && l2 > FREEDOM_REL_TOL * l1;
            let mut directions = vec![json!([e1[0], e1[1]])];
            if two {
                directions.push(json!([-e1[1], e1[0]]));
            }
            points.insert(
                fmt_id(&self.points[i].id),
                json!({ "dof": if two { 2 } else { 1 }, "directions": directions }),
            );
        }

        let mut geometries = Map::new();
        for geo in &self.geometries {
            let Some(obj) = geo.as_object() else { continue };
            let key = fmt_id(obj.get("id").unwrap_or(&Value::Null));
            let mut cols: Vec<usize> = Vec::new();
            if let Some(pts) = obj.get("points").and_then(Value::as_array) {
                for pid in pts {
                    let Some(&idx) = self.point_index.get(&point_key(pid)) else { continue };
                    if let Some((cx, cy)) = point_cols[idx] {
                        if !cols.contains(&cx) {
                            cols.push(cx);
                            cols.push(cy);
                        }
                    }
                }
            }
            let dof = if cols.is_empty() || unit.is_empty() {
                0
            } else {
                // The rank of the null space restricted to the geometry's columns:
                // how many independent ways its points can still move together.
                let rows: Vec<Vec<f64>> = unit.iter().map(|v| cols.iter().map(|&c| v[c]).collect()).collect();
                rank_and_nullspace(rows, unit.len(), cols.len()).0
            };
            geometries.insert(key, json!(dof));
        }

        // Redundant constraints: walk the rows in order, keeping an orthonormal basis
        // of the span so far; a constraint none of whose rows adds to it says
        // nothing the earlier ones did not.
        let mut span: Vec<Vec<f64>> = Vec::new();
        let mut adds = vec![false; self.constraints.len()];
        let mut has_rows = vec![false; self.constraints.len()];
        for (row, &owner) in jac.iter().zip(row_owner) {
            let norm = row.iter().fold(0.0f64, |a, &v| a + v * v).sqrt();
            if !(norm > 0.0) || owner >= adds.len() {
                continue;
            }
            has_rows[owner] = true;
            let mut rest = row.clone();
            for q in &span {
                let d: f64 = rest.iter().zip(q).map(|(a, b)| a * b).sum();
                for (r, qv) in rest.iter_mut().zip(q) {
                    *r -= d * qv;
                }
            }
            let left = rest.iter().fold(0.0f64, |a, &v| a + v * v).sqrt();
            if left > REDUNDANT_REL_TOL * norm {
                for r in rest.iter_mut() {
                    *r /= left;
                }
                span.push(rest);
                adds[owner] = true;
            }
        }
        let named: std::collections::HashSet<String> =
            conflicting.iter().map(|v| v.as_str().map(str::to_string).unwrap_or_else(|| fmt_id(v))).collect();
        let mut redundant = Vec::new();
        for (ci, c) in self.constraints.iter().enumerate() {
            if !has_rows[ci] || adds[ci] || c.temporary {
                continue;
            }
            let Some(id) = c.raw.get("id") else { continue };
            let key = fmt_id(id);
            if !named.contains(&key) {
                redundant.push(Value::String(key));
            }
        }
        FreedomReport { points, geometries, redundant }
    }
}
