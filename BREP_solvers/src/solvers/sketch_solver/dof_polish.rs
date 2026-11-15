use crate::solver_linear_algebra::{row_reduce, solve_spd};
use super::*;

impl Engine {
    // -------------------------------------------------------------------
    // Constraint diagnostics: degrees of freedom, rank, over/under status,
    // per-point mobility via the constraint Jacobian null space.
    // -------------------------------------------------------------------

    /// Build the list of scalar-residual atoms that make up the constraint
    /// Jacobian rows, evaluated against a flat coordinate vector (`coords`
    /// stores `[x0,y0,x1,y1,...]`). Constants (dimension targets, tangent
    /// side) are frozen from the solved configuration so a base residual can
    /// flag genuine conflicts; the Jacobian itself is target-independent.
    /// Ground constraints emit no rows (they are represented by removing the
    /// fixed point's columns), and any constraint whose points are all fixed
    /// is skipped so it cannot inflate the redundant count.
    pub(super) fn build_dof_residuals(
        &self,
        coords: &[f64],
        point_cols: &[Option<(usize, usize)>],
        include_temporary: bool,
    ) -> Vec<DofResidual> {
        self.build_dof_residuals_owned(coords, point_cols, include_temporary)
            .0
    }

    /// [`build_dof_residuals`] plus, for every emitted atom, the index into
    /// `self.constraints` of the constraint that produced it. The conflict
    /// diagnosis needs that provenance to name which constraints a dependency
    /// implicates; the Newton polish does not and calls the plain wrapper.
    pub(super) fn build_dof_residuals_owned(
        &self,
        coords: &[f64],
        point_cols: &[Option<(usize, usize)>],
        include_temporary: bool,
    ) -> (Vec<DofResidual>, Vec<usize>) {
        let gx = |i: usize| coords[2 * i];
        let gy = |i: usize| coords[2 * i + 1];
        let is_free = |i: usize| point_cols[i].is_some();
        let mut out: Vec<DofResidual> = Vec::new();
        let mut owners: Vec<usize> = Vec::new();
        for (ci, c) in self.constraints.iter().enumerate() {
            // The engine's own implied helper constraints (arc equal-chord,
            // bezier colinear handles) belong to every model that describes what
            // the solve actually enforces — the diagnostics and the Newton polish
            // both pass `true`. `false` is for a caller that deliberately wants
            // only the constraints the user authored.
            if c.temporary && !include_temporary {
                continue;
            }
            let idx = &c.point_idx;
            let get = |slot: usize| idx.get(slot).copied().flatten();
            let value = c.value_num.unwrap_or(c.value_parsed);
            let mut atoms: Vec<DofResidual> = Vec::new();
            match c.ctype {
                CType::Horizontal => {
                    if let (Some(a), Some(b)) = (get(0), get(1)) {
                        atoms.push(DofResidual::Horizontal(a, b));
                    }
                }
                CType::Vertical => {
                    if let (Some(a), Some(b)) = (get(0), get(1)) {
                        atoms.push(DofResidual::Vertical(a, b));
                    }
                }
                CType::Distance => {
                    if let (Some(a), Some(b)) = (get(0), get(1)) {
                        let t = if value.is_finite() {
                            value.abs()
                        } else {
                            ((gx(b) - gx(a)).powi(2) + (gy(b) - gy(a)).powi(2)).sqrt()
                        };
                        atoms.push(DofResidual::Distance(a, b, t));
                    }
                }
                CType::PointLine => {
                    if let (Some(a), Some(b), Some(cc)) = (get(0), get(1), get(2)) {
                        let dx = gx(b) - gx(a);
                        let dy = gy(b) - gy(a);
                        let len = (dx * dx + dy * dy).sqrt();
                        if len < 1e-9 {
                            let t = if value.is_finite() {
                                value.abs()
                            } else {
                                ((gx(cc) - gx(a)).powi(2) + (gy(cc) - gy(a)).powi(2)).sqrt()
                            };
                            atoms.push(DofResidual::Distance(a, cc, t));
                        } else {
                            let signed = (-(gx(cc) - gx(a)) * dy + (gy(cc) - gy(a)) * dx) / len;
                            let side = if signed >= 0.0 { 1.0 } else { -1.0 };
                            let t = if value.is_finite() {
                                side * value.abs()
                            } else {
                                signed
                            };
                            atoms.push(DofResidual::PerpDistance(a, b, cc, t));
                        }
                    }
                }
                CType::EqualDistance => {
                    if let (Some(a), Some(b), Some(cc), Some(d)) = (get(0), get(1), get(2), get(3))
                    {
                        atoms.push(DofResidual::EqualDistance(a, b, cc, d));
                    }
                }
                CType::Parallel => {
                    if let (Some(a), Some(b), Some(cc), Some(d)) = (get(0), get(1), get(2), get(3))
                    {
                        atoms.push(DofResidual::Parallel(a, b, cc, d));
                    }
                }
                CType::Perpendicular => {
                    if let (Some(a), Some(b), Some(cc), Some(d)) = (get(0), get(1), get(2), get(3))
                    {
                        atoms.push(DofResidual::Perpendicular(a, b, cc, d));
                    }
                }
                CType::Angle => {
                    if let (Some(a), Some(b), Some(cc), Some(d)) = (get(0), get(1), get(2), get(3))
                    {
                        let t = if value.is_finite() {
                            value * std::f64::consts::PI / 180.0
                        } else {
                            (gy(b) - gy(a)).atan2(gx(b) - gx(a))
                                - (gy(d) - gy(cc)).atan2(gx(d) - gx(cc))
                        };
                        atoms.push(DofResidual::Angle(a, b, cc, d, t));
                    }
                }
                CType::Coincident => {
                    if let (Some(a), Some(b)) = (get(0), get(1)) {
                        atoms.push(DofResidual::Coincident(a, b));
                    }
                }
                CType::PointOnLine => {
                    if let (Some(a), Some(b), Some(cc)) = (get(0), get(1), get(2)) {
                        atoms.push(DofResidual::PerpDistance(a, b, cc, 0.0));
                    }
                }
                CType::Midpoint => {
                    if let (Some(a), Some(b), Some(cc)) = (get(0), get(1), get(2)) {
                        atoms.push(DofResidual::Midpoint(a, b, cc));
                    }
                }
                CType::Ground => { /* no rows: the fixed point's columns are removed */ }
                CType::Tangent => {
                    if let (Some(p0), Some(p1), Some(p2), Some(p3)) =
                        (get(0), get(1), get(2), get(3))
                    {
                        // Spline anchor pairs (end or interior) first,
                        // mirroring c_tangent's dispatch. Spline tangency is
                        // direction-only (G1): vs a spline or line it is a
                        // parallel row, vs a circle a perpendicular row
                        // against the radial direction at the anchor. Both
                        // cover the ± orientation, so no side constant is
                        // frozen.
                        let s01 = self.spline_anchor_info(p0, p1);
                        let s23 = self.spline_anchor_info(p2, p3);
                        if let (Some((a1, h1)), Some((a2, h2))) = (s01, s23) {
                            atoms.push(DofResidual::Parallel(a1, h1, a2, h2));
                        } else if let Some((anchor, handle)) = s01 {
                            if self.is_line_pair(p2, p3) {
                                atoms.push(DofResidual::Parallel(anchor, handle, p2, p3));
                            } else {
                                // dot(handle−anchor, anchor−center) = 0
                                atoms.push(DofResidual::Perpendicular(anchor, handle, p2, anchor));
                            }
                        } else if let Some((anchor, handle)) = s23 {
                            if self.is_line_pair(p0, p1) {
                                atoms.push(DofResidual::Parallel(anchor, handle, p0, p1));
                            } else {
                                atoms.push(DofResidual::Perpendicular(anchor, handle, p0, anchor));
                            }
                        } else if self.is_line_pair(p0, p1) {
                            let dx = gx(p1) - gx(p0);
                            let dy = gy(p1) - gy(p0);
                            let len = (dx * dx + dy * dy).sqrt();
                            if len >= 1e-9 {
                                let signed =
                                    (-(gx(p2) - gx(p0)) * dy + (gy(p2) - gy(p0)) * dx) / len;
                                let side = if signed >= 0.0 { 1.0 } else { -1.0 };
                                atoms.push(DofResidual::TangentLineCircle(p0, p1, p2, p3, side));
                            }
                        } else {
                            let r1 = ((gx(p1) - gx(p0)).powi(2) + (gy(p1) - gy(p0)).powi(2)).sqrt();
                            let r2 = ((gx(p3) - gx(p2)).powi(2) + (gy(p3) - gy(p2)).powi(2)).sqrt();
                            let d = ((gx(p2) - gx(p0)).powi(2) + (gy(p2) - gy(p0)).powi(2)).sqrt();
                            let sign = if (d - (r1 + r2)).abs() <= (d - (r1 - r2).abs()).abs() {
                                1.0
                            } else {
                                -1.0
                            };
                            atoms.push(DofResidual::TangentCircleCircle(p0, p1, p2, p3, sign));
                        }
                    }
                }
                CType::SplineCurvature => {
                    if let (Some(p0), Some(p1), Some(p2), Some(p3)) =
                        (get(0), get(1), get(2), get(3))
                    {
                        // Same dispatch as `c_spline_curvature`: spline pairs
                        // first, then a circle/arc. A degenerate handle (or a
                        // degenerate circle) drops the WHOLE constraint rather
                        // than contributing a zero row, which would read as a
                        // redundant equation on a sketch that is only reporting
                        // a degenerate handle.
                        let s01 = self.spline_curvature_side(p0, p1);
                        let s23 = self.spline_curvature_side(p2, p3);
                        let xy = |i: usize| [gx(i), gy(i)];
                        // The SAME bar the relaxation refuses on, so the named
                        // error and the row are one statement: a handle (or a
                        // radius) the constraint calls degenerate contributes
                        // no equation either.
                        let tolerance = self.tolerance();
                        let usable = |k: Option<(f64, f64)>| {
                            k.filter(|&(_, length)| length >= tolerance).map(|(k, _)| k)
                        };
                        match (s01, s23) {
                            (Some(side1), Some(side2)) => {
                                if let (Some(second1), Some(second2)) = (side1.second, side2.second)
                                {
                                    let k1 = usable(spline_end_curvature(
                                        xy(side1.anchor),
                                        xy(side1.handle),
                                        xy(second1),
                                    ));
                                    let k2 = usable(spline_end_curvature(
                                        xy(side2.anchor),
                                        xy(side2.handle),
                                        xy(second2),
                                    ));
                                    if k1.is_some() && k2.is_some() && side1.handle != side2.handle {
                                        // The tangency the join needs, unless the
                                        // implied `⏛` at an interior anchor is
                                        // already holding it (emitting it there
                                        // too would duplicate that row).
                                        if !self.joint_g1_is_implied(&side1, &side2) {
                                            atoms.push(DofResidual::Parallel(
                                                side1.anchor,
                                                side1.handle,
                                                side2.anchor,
                                                side2.handle,
                                            ));
                                        }
                                        atoms.push(DofResidual::SplineG2Joint(
                                            second1,
                                            side1.handle,
                                            side1.anchor,
                                            side2.handle,
                                            second2,
                                        ));
                                    }
                                }
                            }
                            (Some(side), None) | (None, Some(side)) => {
                                let (center, boundary) = if s01.is_some() { (p2, p3) } else { (p0, p1) };
                                let radius = ((gx(boundary) - gx(center)).powi(2)
                                    + (gy(boundary) - gy(center)).powi(2))
                                .sqrt();
                                let curvature = side.second.and_then(|second| {
                                    usable(spline_end_curvature(
                                        xy(side.anchor),
                                        xy(side.handle),
                                        xy(second),
                                    ))
                                });
                                // A line's two ends are not a circle: `⌒`'s own
                                // dispatch refuses that pair too.
                                if let (Some(second), Some(_)) = (side.second, curvature) {
                                    if radius >= tolerance && !self.is_line_pair(center, boundary) {
                                        // dot(handle−anchor, anchor−center) = 0
                                        atoms.push(DofResidual::Perpendicular(
                                            side.anchor,
                                            side.handle,
                                            center,
                                            side.anchor,
                                        ));
                                        atoms.push(DofResidual::SplineG2Circle {
                                            anchor: side.anchor,
                                            handle: side.handle,
                                            second,
                                            center,
                                            boundary,
                                            sigma: circle_curvature_sign(
                                                xy(side.anchor),
                                                xy(side.handle),
                                                xy(center),
                                            ),
                                        });
                                    }
                                }
                            }
                            (None, None) => {}
                        }
                    }
                }
                CType::SplineFootTangent => {
                    // Tangency at a point INTERIOR to a span: ONE row, the
                    // residual evaluated at the eliminated foot parameter.
                    //
                    // Two constants are frozen here, the way the tangent side
                    // signs are: the SPAN the foot sits in, and the parameter the
                    // eval-time foot solve seeds from. Both come from re-solving
                    // the foot on THIS configuration (seeded by the persisted
                    // `_splineFootT` that relaxation just updated), so the model
                    // is frozen to where the solve actually arrived and not to
                    // where the document last was. Freezing is what keeps the
                    // finite-difference Jacobian smooth: `t*` may slide with the
                    // coordinates, but it never changes which span, and never
                    // jumps to another root.
                    //
                    // No row when there is no simple root: a configuration with
                    // no touch point, or a foot sitting on an inflection (a double
                    // root, where `dt*/dcoords` is unbounded), is a state this
                    // construction has no equation for. The relaxation names both
                    // by error — the same division of labour the `ϰ` lanes make
                    // for a degenerate handle.
                    if let (Some(p0), Some(p1), Some(p2), Some(p3)) =
                        (get(0), get(1), get(2), get(3))
                    {
                        let indices = [Some(p0), Some(p1), Some(p2), Some(p3)];
                        if let Ok((body, q0, q1)) = self.foot_roles(&indices) {
                            let xy = |i: usize| [gx(i), gy(i)];
                            let length = ((gx(q1) - gx(q0)).powi(2) + (gy(q1) - gy(q0)).powi(2))
                                .sqrt();
                            let tolerance = self.tolerance();
                            let is_line = self.is_line_pair(q0, q1);
                            let target = (length >= tolerance).then(|| {
                                if is_line {
                                    FootTarget::Line {
                                        origin: xy(q0),
                                        dir: [
                                            (gx(q1) - gx(q0)) / length,
                                            (gy(q1) - gy(q0)) / length,
                                        ],
                                    }
                                } else {
                                    FootTarget::Circle {
                                        center: xy(q0),
                                        rho: length,
                                    }
                                }
                            });
                            if let Some(target) = target {
                                let spans: Vec<SpanControls> = (0..body.seg_count)
                                    .map(|span| {
                                        Engine::span_point_indices(&body, span).map(xy)
                                    })
                                    .collect();
                                if let Some(hit) = solve_spline_foot(&target, &spans, c.foot_t) {
                                    if hit.simple {
                                        let span = Engine::span_point_indices(&body, hit.span);
                                        atoms.push(if is_line {
                                            DofResidual::SplineFootLine {
                                                span,
                                                a: q0,
                                                b: q1,
                                                seed: hit.t,
                                            }
                                        } else {
                                            DofResidual::SplineFootCircle {
                                                span,
                                                center: q0,
                                                boundary: q1,
                                                seed: hit.t,
                                            }
                                        });
                                    }
                                }
                            }
                        }
                    }
                }
                CType::Concentric => {
                    if let (Some(a), Some(b)) = (get(0), get(1)) {
                        atoms.push(DofResidual::Concentric(a, b));
                    }
                }
                CType::EqualRadius => {
                    if let (Some(a), Some(b), Some(cc), Some(d)) = (get(0), get(1), get(2), get(3))
                    {
                        atoms.push(DofResidual::EqualRadius(a, b, cc, d));
                    }
                }
                CType::Collinear => {
                    let resolved: Vec<usize> = idx.iter().filter_map(|s| *s).collect();
                    if resolved.len() >= 3 {
                        let a = resolved[0];
                        let b = resolved[1];
                        for &pk in &resolved[2..] {
                            atoms.push(DofResidual::PerpDistance(a, b, pk, 0.0));
                        }
                    }
                }
                CType::Symmetric => {
                    if let (Some(a), Some(b), Some(p), Some(q)) = (get(0), get(1), get(2), get(3)) {
                        atoms.push(DofResidual::Symmetric(a, b, p, q));
                    }
                }
                CType::Other => {}
            }
            if atoms.is_empty() {
                continue;
            }
            // Only keep constraints that act on at least one free coordinate;
            // constraints entirely between fixed points are trivially handled
            // by grounding and would otherwise appear as spurious redundancy.
            let any_free = atoms
                .iter()
                .any(|atom| atom.points().iter().any(|&p| is_free(p)));
            if !any_free {
                continue;
            }
            owners.extend(std::iter::repeat(ci).take(atoms.len()));
            out.extend(atoms);
        }
        (out, owners)
    }

    /// Free-coordinate layout shared by the diagnostics and the Newton polish.
    /// Each movable point contributes two unknown columns (its x and y);
    /// fixed/grounded points are removed. `exclude_external` additionally drops
    /// external-reference points from the unknowns (used by the polish, which
    /// must not move geometry anchored to external references). Returns the
    /// per-point column map, the flat list of free global coordinate indices,
    /// the flattened `[x0,y0,x1,y1,...]` coordinates, and a characteristic
    /// length scale for finite-difference steps.
    pub(super) fn build_free_layout(
        &self,
        exclude_external: bool,
    ) -> (Vec<Option<(usize, usize)>>, Vec<usize>, Vec<f64>, f64) {
        let npts = self.points.len();
        let mut point_cols: Vec<Option<(usize, usize)>> = vec![None; npts];
        let mut free_global: Vec<usize> = Vec::new();
        for i in 0..npts {
            let movable =
                !self.points[i].fixed && !(exclude_external && self.points[i].external_reference);
            if movable {
                let cx = free_global.len();
                free_global.push(2 * i);
                let cy = free_global.len();
                free_global.push(2 * i + 1);
                point_cols[i] = Some((cx, cy));
            }
        }

        let mut coords = vec![0.0f64; 2 * npts];
        let mut scale = 1.0f64;
        for i in 0..npts {
            coords[2 * i] = self.points[i].x;
            coords[2 * i + 1] = self.points[i].y;
            // The part-origin reference sits wherever the world origin projects,
            // possibly far from the drawing; it must not loosen the tolerances.
            if is_sketch_origin_point_id(&self.points[i].id) {
                continue;
            }
            if self.points[i].x.is_finite() {
                scale = scale.max(self.points[i].x.abs());
            }
            if self.points[i].y.is_finite() {
                scale = scale.max(self.points[i].y.abs());
            }
        }
        if !scale.is_finite() || scale <= 0.0 {
            scale = 1.0;
        }
        (point_cols, free_global, coords, scale)
    }

    /// `polish_complete` says whether [`newton_polish`](Self::newton_polish)
    /// drove the residuals as far as this engine can. The conflict gate needs
    /// it: reading "a residual is still standing" as "these constraints cannot
    /// hold together" is only sound once everything reachable has been tried.
    pub(super) fn compute_diagnostics(&self, polish_complete: bool) -> Value {
        let npts = self.points.len();

        // Free-coordinate column map: each non-fixed point contributes two
        // unknowns (its x and y). Fixed/ground points are removed entirely.
        let (point_cols, free_global, coords, scale) = self.build_free_layout(false);
        let n = free_global.len();

        // The implied helper rows (arc equal-chord, bezier colinear handles) are
        // part of the model: they are pushed before the solve, relaxation and the
        // Newton polish both enforce them, and they state real geometric
        // properties — an arc's two endpoints share its radius. Analysing the
        // sketch WITHOUT them measured a strictly weaker system than the one that
        // was solved: a lone arc read 6 DOF against 0 equations (three unrelated
        // free points) where an arc has 5, and a contradiction that only closes
        // through the implied row was invisible to the rank structure.
        let (residuals, atom_owner) = self.build_dof_residuals_owned(&coords, &point_cols, true);

        // Base residuals at the solution (used only for conflict detection).
        let r0 = eval_residuals(&residuals, &coords);
        let m = r0.len();

        // Per-ROW ownership. One atom can contribute more than one scalar row
        // (coincident pushes dx AND dy), so the row span of each atom is
        // MEASURED with the same `eval` that stacked `r0` rather than assumed
        // to be one, keeping `row_owner` exactly as long as `r0`.
        let mut row_owner: Vec<usize> = Vec::with_capacity(m);
        {
            let mut scratch: Vec<f64> = Vec::new();
            for (atom, &owner) in residuals.iter().zip(atom_owner.iter()) {
                scratch.clear();
                atom.eval(&coords, &mut scratch);
                row_owner.extend(std::iter::repeat(owner).take(scratch.len()));
            }
        }

        // Constraint Jacobian J (m x n) by central finite differences.
        let jac = finite_diff_jacobian(&residuals, &coords, &free_global, scale, m);

        // Jᵀ (n x m), taken before `rank_and_nullspace` consumes `jac`: its
        // null space is J's LEFT null space, which is what names a conflict.
        let jac_t: Vec<Vec<f64>> = (0..n)
            .map(|col| (0..m).map(|row| jac[row][col]).collect())
            .collect();

        // Per-row residual in LENGTH units. The raw residuals are a mix of
        // lengths (distance, coincident), areas (parallel, perpendicular) and
        // angles, so no single tolerance can read them as authored. Dividing a
        // row by the norm of its own Jacobian row converts it to the first-order
        // coordinate distance that would zero it — "this constraint is off by
        // 0.4 mm" — which is what `conflict_tol` is scaled to measure.
        let row_norms: Vec<f64> = jac
            .iter()
            .map(|row| row.iter().fold(0.0f64, |acc, &v| acc + v * v).sqrt())
            .collect();
        let max_row_norm = row_norms.iter().fold(0.0f64, |a, &v| a.max(v));
        let norm_floor = if max_row_norm > 0.0 {
            1e-9 * max_row_norm
        } else {
            1.0
        };
        let scaled_res: Vec<f64> = r0
            .iter()
            .zip(row_norms.iter())
            .map(|(&v, &norm)| v / norm.max(norm_floor))
            .collect();

        let (rank, basis) = rank_and_nullspace(jac, m, n);
        let dof = n - rank;
        let redundant = m.saturating_sub(rank);

        // Conflict: the solve came to rest with a constraint still unsatisfied.
        // That residual IS the evidence — rank deficiency is only how the group
        // gets NAMED. Requiring `redundant > 0` to raise the flag missed every
        // contradiction that stays first-order independent at the configuration
        // the solver stops on (two tangent lengths from a shared apex dimensioned
        // 20 and 13: rank-full, unsolvable, and formerly reported as a plain
        // under-constrained sketch).
        let conflict_tol = 1e-3 * scale;
        let max_res = scaled_res.iter().fold(0.0f64, |acc, &v| acc.max(v.abs()));
        // ...but only once the solve HAS come to rest. The residual reading is
        // sound because the Newton polish is a global least-squares pass over
        // this same residual set, so a residual it leaves standing really does
        // mean "no solution near here" rather than "ran out of iterations".
        // Skip the polish — a caller passing `polish: false`, or a sketch over
        // `POLISH_MAX_FREE_COORDS` where it declines on cost — and what is left
        // standing is relaxation's own floor, which is coarse enough to read as
        // a conflict: a SATISFIABLE arc with a tangent line leaves its tangent
        // row 0.03 out that way, at every iteration cap up to 1000. So when the
        // polish did not run, fall back to the historical predicate, which asks
        // the rank structure for a dependency before believing the residual.
        let mut conflicting = if polish_complete {
            max_res > conflict_tol
        } else {
            redundant > 0 && max_res > conflict_tol
        };

        // WHICH constraints conflict. `redundant > 0` means J has a left null
        // space: vectors `y` with Jᵀy = 0 — weightings of the constraint rows
        // whose first-order effects cancel exactly. Such a `y` with yᵀr != 0
        // certifies that those rows cannot be driven to zero together (any step
        // Δ leaves yᵀ(r + JΔ) = yᵀr unchanged), and the rows where `y` is
        // non-zero ARE the mutually inconsistent group. Reading the group off
        // `y` rather than off the residuals matters because relaxation is
        // sequential and tends to dump the whole leftover error on whichever
        // constraint it applied last, which would name one arbitrary member of
        // the group instead of the group.
        let mut conflicting_ids: Vec<Value> = Vec::new();
        if conflicting {
            // `redundant == 0` means J has full row rank and its left null space
            // is `{0}` — there is no certificate to look for, so skip the second
            // elimination entirely and go straight to naming the violated rows.
            let left_basis = if redundant > 0 {
                rank_and_nullspace(jac_t, n, m).1
            } else {
                Vec::new()
            };
            let mut rows = vec![false; m];
            let mut certified = false;
            for y in &left_basis {
                let ymax = y.iter().fold(0.0f64, |a, &v| a.max(v.abs()));
                if ymax <= 0.0 {
                    continue;
                }
                // Normalized by |y|_inf so a large certificate cannot fake a
                // conflict, then held to the same tolerance that raised the flag.
                let dot: f64 = y.iter().zip(r0.iter()).map(|(a, b)| a * b).sum::<f64>() / ymax;
                if dot.abs() <= conflict_tol {
                    continue;
                }
                certified = true;
                for (i, &v) in y.iter().enumerate() {
                    if (v / ymax).abs() > DOF_MOBILITY_TOL {
                        rows[i] = true;
                    }
                }
            }
            if !certified {
                // No certificate cleared the tolerance — either there is no left
                // null space at all (a rank-full contradiction), or an unconverged
                // solve left the dependency numerically ragged. Name the rows that
                // are themselves violated instead, so the flag is never raised
                // without saying what raised it.
                for (i, &v) in scaled_res.iter().enumerate() {
                    if v.abs() > conflict_tol {
                        rows[i] = true;
                    }
                }
            }
            let mut flagged = vec![false; self.constraints.len()];
            for (i, &owner) in row_owner.iter().enumerate() {
                if rows[i] {
                    flagged[owner] = true;
                }
            }
            // One entry per constraint in doc order — a multi-row atom group
            // (collinear, coincident) collapses back to its single constraint.
            // The engine's own implied rows are analysed but never NAMED: they
            // carry a synthetic id that matches nothing in the document, so the
            // app could neither paint nor delete them, and the count in the
            // status bar would include a constraint the user cannot see.
            for (ci, &hit) in flagged.iter().enumerate() {
                if !hit || self.constraints[ci].temporary {
                    continue;
                }
                if let Some(id) = self.constraints[ci].raw.get("id") {
                    conflicting_ids.push(Value::String(fmt_id(id)));
                }
            }
            // A flag that names nothing is worse than no flag: the readout would
            // say "conflicting" with an empty group and no glyph would turn red.
            if conflicting_ids.is_empty() {
                conflicting = false;
            }
        }

        let status = if redundant > 0 {
            "over"
        } else if dof > 0 {
            "under"
        } else {
            "well"
        };

        // Per-point mobility from the Jacobian null space. A free point can
        // still move iff some null-space basis vector has a non-negligible
        // component at its x or y column. Fixed points are always locked.
        let basis_maxabs: Vec<f64> = basis
            .iter()
            .map(|v| v.iter().fold(0.0f64, |a, &x| a.max(x.abs())).max(1e-300))
            .collect();
        let mut point_mobility = Map::new();
        for i in 0..npts {
            let key = fmt_id(&self.points[i].id);
            let locked = match point_cols[i] {
                None => true,
                Some((cx, cy)) => {
                    let mut movable = false;
                    for (k, v) in basis.iter().enumerate() {
                        let comp = (v[cx] / basis_maxabs[k]).hypot(v[cy] / basis_maxabs[k]);
                        if comp > DOF_MOBILITY_TOL {
                            movable = true;
                            break;
                        }
                    }
                    !movable
                }
            };
            point_mobility.insert(
                key,
                Value::String(if locked { "locked" } else { "movable" }.into()),
            );
        }

        // A geometry is locked iff all of its defining points are locked.
        let mut geometry_mobility = Map::new();
        for geo in &self.geometries {
            let Some(obj) = geo.as_object() else { continue };
            let key = fmt_id(obj.get("id").unwrap_or(&Value::Null));
            let mut locked = true;
            if let Some(pts) = obj.get("points").and_then(Value::as_array) {
                for pid in pts {
                    let pk = point_key(pid);
                    if let Some(idx) = self.points.iter().position(|p| point_key(&p.id) == pk) {
                        let mk = fmt_id(&self.points[idx].id);
                        if point_mobility.get(&mk).and_then(Value::as_str) == Some("movable") {
                            locked = false;
                            break;
                        }
                    }
                }
            }
            geometry_mobility.insert(
                key,
                Value::String(if locked { "locked" } else { "movable" }.into()),
            );
        }

        let mut diag = Map::new();
        diag.insert("dof".into(), Value::Number(dof.into()));
        diag.insert("rank".into(), Value::Number(rank.into()));
        diag.insert("unknowns".into(), Value::Number(n.into()));
        diag.insert("equations".into(), Value::Number(m.into()));
        diag.insert("redundant".into(), Value::Number(redundant.into()));
        diag.insert("status".into(), Value::String(status.into()));
        diag.insert("conflicting".into(), Value::Bool(conflicting));
        diag.insert(
            "conflictingConstraints".into(),
            Value::Array(conflicting_ids),
        );
        diag.insert("pointMobility".into(), Value::Object(point_mobility));
        diag.insert("geometryMobility".into(), Value::Object(geometry_mobility));
        Value::Object(diag)
    }

    /// Global Levenberg-Marquardt least-squares polish, run once after the
    /// iterative relaxation solve has stopped moving. Relaxation only converges
    /// to its residual floor (~1e-5), which leaves constraint chains
    /// (perpendicular, distance, implied equalities) a few 1e-5 from exact and
    /// feeds that noise into downstream booleans/fillets. This drives the
    /// maximum constraint residual to machine precision without disturbing free
    /// degrees of freedom.
    ///
    /// Each step solves the damped normal equations `(JᵀJ + λI)Δ = -Jᵀr`. The
    /// right-hand side `Jᵀr` lies in the row space of `J`, so its solution has
    /// no component in the null space (the under-constrained / free directions):
    /// the step is the minimum-norm move that reduces the constraints, and free
    /// DOF are left exactly where relaxation put them. The damping λ also
    /// regularizes the (generally rank-deficient) normal-equations matrix, and
    /// grows on a rejected step / factorization failure. A strict non-worsening
    /// guard keeps the relaxation result whenever the polish fails to reduce the
    /// maximum residual, so the sketch can never end up worse than relaxation.
    ///
    /// Returns whether the constraint residuals were driven as far as this
    /// engine can drive them. `false` means relaxation's ~1e-5 floor is still
    /// standing untouched, which the conflict diagnosis must know: that floor is
    /// large enough to read as an unsatisfiable constraint. Measured on a
    /// SATISFIABLE arc-and-tangent sketch, the tangent row stays 0.03 out with
    /// the polish skipped — at every iteration cap up to 1000, so it is a floor
    /// and not a budget — and the residual gate would call that a conflict.
    pub(super) fn newton_polish(&mut self) -> bool {
        if !self.settings.newton_polish {
            return false;
        }
        // Unknowns: movable point coordinates (exclude grounded + external
        // references), the same free set the DOF diagnostics use.
        let (point_cols, free_global, coords0, scale) = self.build_free_layout(true);
        let n = free_global.len();
        if n == 0 {
            return true; // Nothing is free, so no row carries a free coordinate.
        }
        if n > POLISH_MAX_FREE_COORDS {
            return false; // Skipped for cost; relaxation's result stands as-is.
        }
        // Residual model includes the engine's implied helper constraints so the
        // polish honours exactly what relaxation enforced (arc equal-chord,
        // bezier colinear handles). Constants (dimension targets, tangent side)
        // are frozen once from the relaxation result and held fixed across the
        // Newton iterations for stability.
        let residuals = self.build_dof_residuals(&coords0, &point_cols, true);
        if residuals.is_empty() {
            return true; // No constraint to enforce, so nothing is left standing.
        }
        let mut r = eval_residuals(&residuals, &coords0);
        let m = r.len();
        if m == 0 {
            return true;
        }

        let max_abs = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc.max(x.abs()));
        let sum_sq = |v: &[f64]| v.iter().fold(0.0f64, |acc, &x| acc + x * x);

        let f0 = max_abs(&r);
        if !f0.is_finite() {
            return false; // Degenerate coordinates; nothing was driven anywhere.
        }
        let tight = POLISH_TIGHT_TOL * scale.max(1.0);
        if f0 <= tight {
            return true; // Relaxation is already exact-to-tolerance; nothing to gain.
        }

        let mut coords = coords0;
        let mut s = sum_sq(&r);
        let mut best_coords = coords.clone();
        let mut best_max = f0;
        let step_tol = POLISH_STEP_REL_TOL * scale.max(1.0);
        let mut lambda = -1.0f64; // Initialized from the first Jacobian scale.

        for _iter in 0..POLISH_MAX_ITER {
            if best_max <= tight {
                break;
            }
            let jac = finite_diff_jacobian(&residuals, &coords, &free_global, scale, m);
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
                break; // No sensitivity to any free coordinate.
            }
            if lambda < 0.0 {
                lambda = 1e-9 * max_diag;
            }
            let lambda_ceiling = 1e12 * max_diag;

            let mut stepped = false;
            let mut step_small = false;
            for _try in 0..POLISH_MAX_DAMPING {
                let mut a_mat = h.clone();
                for d in 0..n {
                    a_mat[d][d] += lambda;
                }
                let mut delta: Vec<f64> = g.iter().map(|v| -v).collect();
                if !solve_spd(&mut a_mat, &mut delta) {
                    lambda *= POLISH_LAMBDA_UP;
                    if lambda > lambda_ceiling {
                        break;
                    }
                    continue;
                }
                let mut trial = coords.clone();
                let mut step_sq = 0.0f64;
                for col in 0..n {
                    trial[free_global[col]] += delta[col];
                    step_sq += delta[col] * delta[col];
                }
                let r_trial = eval_residuals(&residuals, &trial);
                let s_trial = sum_sq(&r_trial);
                if s_trial.is_finite() && s_trial < s {
                    let m_trial = max_abs(&r_trial);
                    coords = trial;
                    r = r_trial;
                    s = s_trial;
                    if m_trial < best_max {
                        best_max = m_trial;
                        best_coords.copy_from_slice(&coords);
                    }
                    lambda = (lambda * POLISH_LAMBDA_DOWN).max(1e-12 * max_diag);
                    stepped = true;
                    step_small = step_sq.sqrt() <= step_tol;
                    break;
                }
                lambda *= POLISH_LAMBDA_UP;
                if lambda > lambda_ceiling {
                    break;
                }
            }
            if !stepped || step_small {
                break; // Converged, stalled, or damping exhausted.
            }
        }

        // Non-worsening guard: never leave the sketch worse than relaxation.
        // Only commit when the polish strictly reduced the maximum residual.
        if best_max < f0 {
            for col in 0..n {
                let gcol = free_global[col];
                let i = gcol / 2;
                if gcol % 2 == 0 {
                    self.points[i].x = best_coords[gcol];
                } else {
                    self.points[i].y = best_coords[gcol];
                }
            }
        }
        true
    }
}

// ---------------------------------------------------------------------------
// Degrees-of-freedom helpers (dense numerical rank + null space, no crates)
// ---------------------------------------------------------------------------

/// Relative pivot tolerance for the numerical rank. Comfortably above the
/// central-difference noise floor yet below any genuine constraint derivative.
const DOF_RANK_REL_TOL: f64 = 1e-9;
/// A null-space component below this (per basis vector, L-inf normalized) is
/// treated as zero when deciding whether a point can still move.
const DOF_MOBILITY_TOL: f64 = 1e-6;

// --- Levenberg-Marquardt polish tuning (post-relaxation exactness pass) -----
/// Maximum outer LM iterations.
const POLISH_MAX_ITER: usize = 60;
/// Maximum damping increases per outer iteration before abandoning the step.
const POLISH_MAX_DAMPING: usize = 16;
/// Damping growth on a rejected step / Cholesky failure.
const POLISH_LAMBDA_UP: f64 = 8.0;
/// Damping shrink after an accepted step (drives λ toward pure Gauss-Newton).
const POLISH_LAMBDA_DOWN: f64 = 0.25;
/// Convergence target for the maximum residual, relative to the sketch scale.
/// Comfortably below the ~1e-9 exactness downstream features require.
const POLISH_TIGHT_TOL: f64 = 1e-12;
/// A Newton step below this (relative to scale) counts as converged.
const POLISH_STEP_REL_TOL: f64 = 1e-13;
/// Skip the dense O(n^3) polish above this many free coordinates so a very
/// large sketch cannot stall an interactive drag frame; relaxation stands and
/// the non-worsening guard trivially holds.
const POLISH_MAX_FREE_COORDS: usize = 1200;

/// One scalar (or paired) residual equation contributing rows to the Jacobian.
/// Indices reference points by their engine index; `coords` is `[x0,y0,...]`.
#[derive(Clone, Copy, Debug)]
pub(super) enum DofResidual {
    Horizontal(usize, usize),
    Vertical(usize, usize),
    Distance(usize, usize, f64),
    /// Signed perpendicular distance of `c` from line `a→b` minus a target.
    PerpDistance(usize, usize, usize, f64),
    EqualDistance(usize, usize, usize, usize),
    Parallel(usize, usize, usize, usize),
    Perpendicular(usize, usize, usize, usize),
    Angle(usize, usize, usize, usize, f64),
    Coincident(usize, usize),
    Midpoint(usize, usize, usize),
    Concentric(usize, usize),
    EqualRadius(usize, usize, usize, usize),
    Symmetric(usize, usize, usize, usize),
    /// line `a→b`, circle (`center`,`boundary`), frozen side sign.
    TangentLineCircle(usize, usize, usize, usize, f64),
    /// circles (`c1`,`b1`) & (`c2`,`b2`), +1 external / -1 internal.
    TangentCircleCircle(usize, usize, usize, usize, f64),
    /// Curvature (G2) at a spline joint: `(l2, l, a, r, r2)` — the arriving
    /// side's second control and handle, the shared anchor, then the leaving
    /// side's handle and second control. One row, `(κ₁ − κ₂)·ℓ²`, in length
    /// units. Swapping the two sides leaves the row unchanged.
    SplineG2Joint(usize, usize, usize, usize, usize),
    /// Curvature (G2) between a spline end (`anchor`, `handle`, `second`) and a
    /// circle (`center`, `boundary`), with the side sign frozen from the solved
    /// configuration. One row, `(κρ − σ)·ρ`, in length units.
    SplineG2Circle {
        anchor: usize,
        handle: usize,
        second: usize,
        center: usize,
        boundary: usize,
        sigma: f64,
    },
    /// Tangency between a line `a→b` and the spline span whose four control
    /// points are `span`, at a point INTERIOR to that span. One row, the signed
    /// distance of the touch point from the line, in length units.
    ///
    /// The touch parameter is no unknown: it is the root of `cross(d̂, B′) = 0`
    /// nearest `seed`, solved in closed form inside `eval` at whatever
    /// coordinates it is handed — which is what lets the finite-difference
    /// Jacobian differentiate through the elimination. `span` and `seed` are
    /// frozen by the builder; everything else the row depends on moves.
    SplineFootLine {
        span: [usize; 4],
        a: usize,
        b: usize,
        seed: f64,
    },
    /// Tangency between a circle (`center`, `boundary`) and a spline span at a
    /// point interior to it. One row, `|B(t*) − c| − ρ`, in length units, with
    /// `t*` the root of `dot(B − c, B′) = 0` reached by Newton from `seed`.
    ///
    /// No side constant, unlike [`DofResidual::TangentLineCircle`]: internal and
    /// external tangency both zero this residual, and which of them the sketch
    /// is on is carried by the BRANCH the seed keeps rather than by a sign.
    SplineFootCircle {
        span: [usize; 4],
        center: usize,
        boundary: usize,
        seed: f64,
    },
}

impl DofResidual {
    fn eval(&self, c: &[f64], out: &mut Vec<f64>) {
        let gx = |i: usize| c[2 * i];
        let gy = |i: usize| c[2 * i + 1];
        match *self {
            DofResidual::Horizontal(a, b) => out.push(gy(a) - gy(b)),
            DofResidual::Vertical(a, b) => out.push(gx(a) - gx(b)),
            DofResidual::Distance(a, b, t) => {
                out.push(((gx(b) - gx(a)).powi(2) + (gy(b) - gy(a)).powi(2)).sqrt() - t);
            }
            DofResidual::PerpDistance(a, b, cc, t) => {
                let dx = gx(b) - gx(a);
                let dy = gy(b) - gy(a);
                let len = (dx * dx + dy * dy).sqrt();
                if len < 1e-12 {
                    out.push(0.0);
                } else {
                    out.push((-(gx(cc) - gx(a)) * dy + (gy(cc) - gy(a)) * dx) / len - t);
                }
            }
            DofResidual::EqualDistance(a, b, cc, d) => {
                let d1 = ((gx(b) - gx(a)).powi(2) + (gy(b) - gy(a)).powi(2)).sqrt();
                let d2 = ((gx(d) - gx(cc)).powi(2) + (gy(d) - gy(cc)).powi(2)).sqrt();
                out.push(d1 - d2);
            }
            DofResidual::Parallel(a, b, cc, d) => {
                let dx1 = gx(b) - gx(a);
                let dy1 = gy(b) - gy(a);
                let dx2 = gx(d) - gx(cc);
                let dy2 = gy(d) - gy(cc);
                out.push(dx1 * dy2 - dy1 * dx2);
            }
            DofResidual::Perpendicular(a, b, cc, d) => {
                let dx1 = gx(b) - gx(a);
                let dy1 = gy(b) - gy(a);
                let dx2 = gx(d) - gx(cc);
                let dy2 = gy(d) - gy(cc);
                out.push(dx1 * dx2 + dy1 * dy2);
            }
            DofResidual::Angle(a, b, cc, d, t) => {
                let a1 = (gy(b) - gy(a)).atan2(gx(b) - gx(a));
                let a2 = (gy(d) - gy(cc)).atan2(gx(d) - gx(cc));
                let mut e = a1 - a2 - t;
                let two_pi = 2.0 * std::f64::consts::PI;
                while e > std::f64::consts::PI {
                    e -= two_pi;
                }
                while e < -std::f64::consts::PI {
                    e += two_pi;
                }
                out.push(e);
            }
            DofResidual::Coincident(a, b) => {
                out.push(gx(a) - gx(b));
                out.push(gy(a) - gy(b));
            }
            DofResidual::Midpoint(a, b, cc) => {
                out.push(2.0 * gx(cc) - gx(a) - gx(b));
                out.push(2.0 * gy(cc) - gy(a) - gy(b));
            }
            DofResidual::Concentric(a, b) => {
                out.push(gx(a) - gx(b));
                out.push(gy(a) - gy(b));
            }
            DofResidual::EqualRadius(c1, b1, c2, b2) => {
                let r1 = ((gx(b1) - gx(c1)).powi(2) + (gy(b1) - gy(c1)).powi(2)).sqrt();
                let r2 = ((gx(b2) - gx(c2)).powi(2) + (gy(b2) - gy(c2)).powi(2)).sqrt();
                out.push(r1 - r2);
            }
            DofResidual::Symmetric(a, b, p, q) => {
                let dx = gx(b) - gx(a);
                let dy = gy(b) - gy(a);
                let len = (dx * dx + dy * dy).sqrt();
                let mx = (gx(p) + gx(q)) / 2.0;
                let my = (gy(p) + gy(q)) / 2.0;
                if len < 1e-12 {
                    out.push(0.0);
                } else {
                    out.push((-(mx - gx(a)) * dy + (my - gy(a)) * dx) / len);
                }
                out.push((gx(q) - gx(p)) * dx + (gy(q) - gy(p)) * dy);
            }
            DofResidual::TangentLineCircle(a, b, center, boundary, side) => {
                let dx = gx(b) - gx(a);
                let dy = gy(b) - gy(a);
                let len = (dx * dx + dy * dy).sqrt();
                let radius = ((gx(boundary) - gx(center)).powi(2)
                    + (gy(boundary) - gy(center)).powi(2))
                .sqrt();
                if len < 1e-12 {
                    out.push(0.0);
                } else {
                    let signed = (-(gx(center) - gx(a)) * dy + (gy(center) - gy(a)) * dx) / len;
                    out.push(signed - side * radius);
                }
            }
            DofResidual::TangentCircleCircle(c1, b1, c2, b2, sign) => {
                let r1 = ((gx(b1) - gx(c1)).powi(2) + (gy(b1) - gy(c1)).powi(2)).sqrt();
                let r2 = ((gx(b2) - gx(c2)).powi(2) + (gy(b2) - gy(c2)).powi(2)).sqrt();
                let d = ((gx(c2) - gx(c1)).powi(2) + (gy(c2) - gy(c1)).powi(2)).sqrt();
                out.push(d - (r1 + sign * r2));
            }
            // The two curvature rows always push exactly one value, degenerate
            // or not: the builder is what decides whether a G2 row exists at
            // all, and the row count has to stay fixed while the
            // finite-difference Jacobian perturbs coordinates through a
            // degeneracy.
            DofResidual::SplineG2Joint(l2, l, a, r, r2) => {
                let xy = |i: usize| [gx(i), gy(i)];
                match (
                    spline_end_curvature(xy(a), xy(l), xy(l2)),
                    spline_end_curvature(xy(a), xy(r), xy(r2)),
                ) {
                    (Some((k1, a1)), Some((k2, a2))) => {
                        out.push(joint_curvature_residual(k1, k2, a1, a2))
                    }
                    _ => out.push(0.0),
                }
            }
            DofResidual::SplineG2Circle { anchor, handle, second, center, boundary, sigma } => {
                let xy = |i: usize| [gx(i), gy(i)];
                let radius = ((gx(boundary) - gx(center)).powi(2)
                    + (gy(boundary) - gy(center)).powi(2))
                .sqrt();
                match spline_end_curvature(xy(anchor), xy(handle), xy(second)) {
                    Some((k, _)) => out.push(circle_curvature_residual(k, radius, sigma)),
                    None => out.push(0.0),
                }
            }
            // The two FOOT rows re-solve the eliminated parameter here, at the
            // coordinates they are handed: that is the whole mechanism — the
            // central-difference Jacobian then differentiates through `t*(x)`
            // with no changes of its own. The frozen `seed` is what keeps the
            // root on one branch while a coordinate is perturbed; the span is
            // frozen by the builder. Like every other atom, exactly one value is
            // pushed on every path, degenerate or not, because the row COUNT has
            // to survive a perturbation that makes the geometry degenerate.
            DofResidual::SplineFootLine { span, a, b, seed } => {
                let controls: SpanControls = span.map(|i| [gx(i), gy(i)]);
                let (dx, dy) = (gx(b) - gx(a), gy(b) - gy(a));
                let length = dx.hypot(dy);
                if length < 1e-12 {
                    out.push(0.0);
                } else {
                    let target = FootTarget::Line {
                        origin: [gx(a), gy(a)],
                        dir: [dx / length, dy / length],
                    };
                    out.push(foot_row(&target, &controls, seed));
                }
            }
            DofResidual::SplineFootCircle { span, center, boundary, seed } => {
                let controls: SpanControls = span.map(|i| [gx(i), gy(i)]);
                let rho = ((gx(boundary) - gx(center)).powi(2)
                    + (gy(boundary) - gy(center)).powi(2))
                .sqrt();
                if rho < 1e-12 {
                    out.push(0.0);
                } else {
                    let target = FootTarget::Circle {
                        center: [gx(center), gy(center)],
                        rho,
                    };
                    out.push(foot_row(&target, &controls, seed));
                }
            }
        }
    }

    /// Engine point indices this residual depends on.
    fn points(&self) -> Vec<usize> {
        match *self {
            DofResidual::Horizontal(a, b)
            | DofResidual::Vertical(a, b)
            | DofResidual::Distance(a, b, _)
            | DofResidual::Coincident(a, b)
            | DofResidual::Concentric(a, b) => vec![a, b],
            DofResidual::PerpDistance(a, b, c, _) | DofResidual::Midpoint(a, b, c) => vec![a, b, c],
            DofResidual::EqualDistance(a, b, c, d)
            | DofResidual::Parallel(a, b, c, d)
            | DofResidual::Perpendicular(a, b, c, d)
            | DofResidual::Angle(a, b, c, d, _)
            | DofResidual::EqualRadius(a, b, c, d)
            | DofResidual::Symmetric(a, b, c, d)
            | DofResidual::TangentLineCircle(a, b, c, d, _)
            | DofResidual::TangentCircleCircle(a, b, c, d, _) => vec![a, b, c, d],
            DofResidual::SplineG2Joint(l2, l, a, r, r2) => vec![l2, l, a, r, r2],
            DofResidual::SplineG2Circle { anchor, handle, second, center, boundary, .. } => {
                vec![anchor, handle, second, center, boundary]
            }
            // All four span controls: the row moves with every one of them
            // (through `B(t*)` AND through the foot the stationarity condition
            // puts there), so `any_free` and the per-point mobility null space
            // must see them all.
            DofResidual::SplineFootLine { span, a, b, .. } => {
                vec![span[0], span[1], span[2], span[3], a, b]
            }
            DofResidual::SplineFootCircle { span, center, boundary, .. } => {
                vec![span[0], span[1], span[2], span[3], center, boundary]
            }
        }
    }
}

/// One foot row's value: the residual at the foot, or — when a perturbation has
/// destroyed the root the builder froze this row on — at the frozen seed itself.
///
/// The fallback is reachable only where `dt*/dcoords` is already unbounded, which
/// is the state the builder refuses to emit a row for at all; it exists so the
/// row COUNT is a constant of the residual model, which the Jacobian assembly
/// requires. The search follows the root a bounded [`FOOT_EVAL_MARGIN`] past the
/// span's ends, so a foot that solves to a span SEAM stays differentiable there
/// instead of falling off its own span under a finite difference.
fn foot_row(target: &FootTarget, controls: &SpanControls, seed: f64) -> f64 {
    let t = foot_in_span(
        target,
        controls,
        seed,
        -FOOT_EVAL_MARGIN,
        1.0 + FOOT_EVAL_MARGIN,
    )
    .unwrap_or_else(|| seed.clamp(0.0, 1.0));
    target.residual(controls, t)
}

/// Stack every residual atom into the flat residual vector `r(x)` at `coords`.
/// Shared by the DOF diagnostics and the Newton polish so both see the exact
/// same constraint residual model.
pub(super) fn eval_residuals(residuals: &[DofResidual], coords: &[f64]) -> Vec<f64> {
    let mut r: Vec<f64> = Vec::with_capacity(residuals.len());
    for res in residuals {
        res.eval(coords, &mut r);
    }
    r
}

/// Central-difference constraint Jacobian `J` (`m x n`) of `residuals` with
/// respect to the free coordinates listed in `free_global`. `m` is the length
/// of the stacked residual vector. Shared by the diagnostics rank/null-space
/// computation and the Newton polish step assembly.
fn finite_diff_jacobian(
    residuals: &[DofResidual],
    coords: &[f64],
    free_global: &[usize],
    scale: f64,
    m: usize,
) -> Vec<Vec<f64>> {
    let n = free_global.len();
    let mut jac = vec![vec![0.0f64; n]; m];
    if n == 0 || m == 0 {
        return jac;
    }
    let h = 1e-6 * scale;
    let inv = 1.0 / (2.0 * h);
    let mut cpert = coords.to_vec();
    let mut rp: Vec<f64> = Vec::with_capacity(m);
    let mut rm: Vec<f64> = Vec::with_capacity(m);
    for col in 0..n {
        let g = free_global[col];
        let saved = cpert[g];
        cpert[g] = saved + h;
        rp.clear();
        for res in residuals {
            res.eval(&cpert, &mut rp);
        }
        cpert[g] = saved - h;
        rm.clear();
        for res in residuals {
            res.eval(&cpert, &mut rm);
        }
        cpert[g] = saved;
        for i in 0..m {
            jac[i][col] = (rp[i] - rm[i]) * inv;
        }
    }
    jac
}

/// Numerical rank and null-space basis of a dense `m x n` matrix via
/// Gauss-Jordan elimination with partial pivoting. Returns `(rank, basis)`
/// where `basis` holds `n - rank` null-space vectors (each length `n`).
fn rank_and_nullspace(mut a: Vec<Vec<f64>>, m: usize, n: usize) -> (usize, Vec<Vec<f64>>) {
    if n == 0 {
        return (0, Vec::new());
    }
    let mut maxabs = 0.0f64;
    for row in &a {
        for &v in row {
            let av = v.abs();
            if av > maxabs {
                maxabs = av;
            }
        }
    }
    let tol = if maxabs > 0.0 {
        DOF_RANK_REL_TOL * maxabs
    } else {
        DOF_RANK_REL_TOL
    };

    let mut pivot_cols: Vec<usize> = Vec::new();
    row_reduce(&mut a, m, n, tol, |col| pivot_cols.push(col));

    let rank = pivot_cols.len();
    let mut is_pivot = vec![false; n];
    for &pc in &pivot_cols {
        is_pivot[pc] = true;
    }
    let mut basis: Vec<Vec<f64>> = Vec::new();
    for free_col in 0..n {
        if is_pivot[free_col] {
            continue;
        }
        let mut v = vec![0.0f64; n];
        v[free_col] = 1.0;
        for (i, &pc) in pivot_cols.iter().enumerate() {
            v[pc] = -a[i][free_col];
        }
        basis.push(v);
    }
    (rank, basis)
}
